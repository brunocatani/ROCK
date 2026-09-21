#include "RockConfig.h"
#include "api/BorrowedCallbackContext.h"
#include "api/OwnerBindingPolicy.h"
#include "api/ProviderRuntimeServices.h"
#include <ROCK/Configuration.h>
#include <cassert>
#include <barrier>
#include <fstream>
#include <future>
#include <latch>
#include <map>
#include <thread>

namespace rock::api::configuration { const ApiV1& table() noexcept; }
namespace {
    using rock::api::Status;
    namespace config = rock::api::configuration;
    const auto runtimeThread = std::this_thread::get_id();
    std::filesystem::path directory;
    rock::provider::InterfaceBinding binding{1,3};
    bool revoked{};
    using Catalog = std::map<std::string,std::string>;
    void ROCK_CALL collect(const config::SettingV1* setting, void* context) {
        static_cast<Catalog*>(context)->emplace(setting->key,setting->value);
    }
    Catalog catalog(config::Group group) {
        Catalog result;
        assert(config::table().visit(42,group,collect,&result)==Status::Ok);
        assert(!result.empty());
        return result;
    }
    Status set(config::Group group, const char* section, const char* key, const char* value) {
        char error[256]{};
        return config::table().setValue(42,group,section,key,value,error,sizeof(error));
    }
}
// Replace only engine-independent owner authorization and the Documents root.
// The provider, configuration owner, catalog, reload, watcher and file writes
// are production code; no test operation can reach production INIs.
namespace rock::resources {
    std::string getPathInDocuments(const std::string&) { assert(!directory.empty());return directory.string(); }
}
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t owner, api::InterfaceId family, std::uint32_t permission, bool requireThread, OwnerAccess access) {
        if (borrowed_callback::active) return Status::Busy;
        if (owner!=42) return Status::OwnerNotRegistered;
        assert(family==api::InterfaceId::Configuration);
        const auto status=authorizeBinding(binding,revoked,permission,access);
        if (status!=Status::Ok) return status;
        if (requireThread && std::this_thread::get_id()!=runtimeThread) return Status::WrongThread;
        return Status::Ok;
    }
    void revoke(std::uint64_t owner) { assert(owner==42);revoked=true; }
    void reportBoundaryFailure(std::uint64_t,api::InterfaceId) noexcept { assert(false); }
}
int main() {
    namespace fs=std::filesystem;
    using rock::g_rockConfig;
    directory=fs::temp_directory_path()/std::format("ROCK-config-provider-{}-{}",GetCurrentProcessId(),GetTickCount64());
    fs::create_directory(directory);
    struct Cleanup {
        ~Cleanup() {
            g_rockConfig.stopFileWatch();
            assert(fs::equivalent(directory.parent_path(),fs::temp_directory_path()) && directory.filename().string().starts_with("ROCK-config-provider-"));
            std::error_code error;
            fs::remove_all(directory,error);
        }
    } cleanup;
    const auto& api=config::table();
    Catalog missing;
    assert(api.visit(42,config::Group::Consumer,collect,&missing)==Status::NotReady);
    assert(api.visit(99,config::Group::Consumer,collect,&missing)==Status::OwnerNotRegistered);
    g_rockConfig.load();
    g_rockConfig.stopFileWatch(); // Keep reload timing deterministic; writes still request it.
    assert(!g_rockConfig.rockBladePenetrationEnabled);
    assert(catalog(config::Group::Consumer).at("bBladePenetrationEnabled")=="false");
    assert(!catalog(config::Group::Developer).contains("bBladePenetrationEnabled"));
    assert(set(config::Group::Consumer,"PhysicsInteraction","bBladePenetrationEnabled","true")==Status::Ok);
    g_rockConfig.reload();
    assert(g_rockConfig.rockBladePenetrationEnabled);
    assert(set(config::Group::Consumer,"PhysicsInteraction","bBladePenetrationEnabled","false")==Status::Ok);
    g_rockConfig.reload();
    assert(!g_rockConfig.rockBladePenetrationEnabled);
    const auto initialRevision=g_rockConfig.configRevision();
    assert(initialRevision && !fs::exists(directory/"ROCK_Developer.ini"));
    const auto originalLogLevel=g_rockConfig.rockLogLevel;

    auto task=std::async(std::launch::async,[&] {
        assert(std::this_thread::get_id()!=runtimeThread);
        assert(catalog(config::Group::Consumer).contains("iLogLevel"));
        assert(catalog(config::Group::Developer).at("bDeveloperModeEnabled")=="false");
        assert(set(config::Group::Consumer,"Logging","iLogLevel","3")==Status::Ok);
        assert(set(config::Group::Developer,"Debug","bDeveloperModeEnabled","true")==Status::Ok);
        std::uint64_t revision{};
        assert(api.revision(42,&revision)==Status::Ok && revision==initialRevision);
    });
    task.get();
    assert(g_rockConfig.rockLogLevel==originalLogLevel && !g_rockConfig.rockDeveloperModeEnabled);
    assert(fs::exists(directory/"ROCK_Developer.ini"));
    g_rockConfig.reload();
    assert(g_rockConfig.rockLogLevel==3 && g_rockConfig.rockDeveloperModeEnabled);
    assert(g_rockConfig.configRevision()>initialRevision);

    // Two simultaneous edits in one owning file must preserve each other.
    std::barrier start(3);
    auto first=std::async(std::launch::async,[&] {
        start.arrive_and_wait();
        return set(config::Group::Developer,"Debug","bDeveloperModeEnabled","false");
    });
    auto second=std::async(std::launch::async,[&] {
        start.arrive_and_wait();
        return set(config::Group::Developer,"PhysicsInteraction","fDebugVideoSyncMarkerSize","8");
    });
    start.arrive_and_wait();
    assert(first.get()==Status::Ok && second.get()==Status::Ok);
    g_rockConfig.reload();
    const auto developer=catalog(config::Group::Developer);
    assert(developer.at("bDeveloperModeEnabled")=="false" && developer.at("fDebugVideoSyncMarkerSize")=="8");

    // Hold a borrowed catalog entry while a runtime reload becomes pending.
    // Reload must return without waiting and apply after the visitor finishes.
    struct Borrow { std::latch entered{1},finish{1};bool first=true; } borrow;
    auto reader=std::async(std::launch::async,[&] {
        return api.visit(42,config::Group::Consumer,+[](const config::SettingV1* value,void* opaque) {
            auto& b=*static_cast<Borrow*>(opaque);
            if(!b.first)return;
            b.first=false;
            const std::string retained=value->value;
            std::uint64_t revision{};
            assert(config::table().revision(42,&revision)==Status::Busy);
            b.entered.count_down();b.finish.wait();
            assert(retained==value->value);
        },&borrow);
    });
    borrow.entered.wait();
    { std::ofstream file(directory/"ROCK.ini");file<<"[Logging]\niLogLevel=4\n"; }
    g_rockConfig.reload();
    assert(g_rockConfig.rockLogLevel==3);
    borrow.finish.count_down();
    assert(reader.get()==Status::Ok);
    std::this_thread::sleep_for(std::chrono::milliseconds(210)); // Existing reload debounce.
    g_rockConfig.processPendingConfigReload();
    assert(g_rockConfig.rockLogLevel==4);

    // Reentrant subscribers run after releasing catalog ownership.
    bool notified=false;
    g_rockConfig.subscribeForConfigChanged("test",[&](const std::string&) {
        notified=true;assert(catalog(config::Group::Consumer).at("iLogLevel")=="4");
    });
    assert(set(config::Group::Developer,"PhysicsInteraction","fDebugVideoSyncMarkerSize","4")==Status::Ok);
    assert(!fs::exists(directory/"ROCK_Developer.ini"));
    g_rockConfig.reload();assert(notified);
    g_rockConfig.unsubscribeFromConfigChanged("test");
    binding.permissions=1;
    assert(set(config::Group::Consumer,"Logging","iLogLevel","2")==Status::PermissionDenied);
    binding.permissions=3;
    assert(api.visit(42,config::Group::Consumer,+[](const config::SettingV1*,void*) {throw 1;},nullptr)==Status::OwnerRevoked);
    assert(revoked && set(config::Group::Consumer,"Logging","iLogLevel","2")==Status::OwnerRevoked);
}

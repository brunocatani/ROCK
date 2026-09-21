#include "api/BorrowedCallbackContext.h"
#include <Windows.h>
#include <ROCK/Configuration.h>
#include "Boundary.h"
#include "RockConfig.h"

namespace rock::api::configuration {
namespace {
    struct VisitContext { VisitorV1 visitor; void* user; bool fault{}; };
    bool invokeVisitor(VisitorV1 visitor, const SettingV1* setting, void* user) {
#if defined(_MSC_VER)
        __try { visitor(setting,user); return true; }
        __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
#else
        visitor(setting,user); return true;
#endif
    }
    void visitSetting(const configuration_api::SettingV1* value, void* opaque) noexcept {
        auto& context=*static_cast<VisitContext*>(opaque);
        if (context.fault) return;
        const SettingV1 setting{value->section,value->key,value->value,value->defaultValue,
            value->category,value->description,static_cast<ValueType>(value->type),value->overridden};
        provider::borrowed_callback::Scope callbackScope;
        try { context.fault=!invokeVisitor(context.visitor,&setting,context.user); }
        catch (...) { context.fault=true; }
    }
    Status ROCK_CALL revision(OwnerToken owner, std::uint64_t* value) noexcept {
        if (!value) return Status::InvalidArgument;
        *value=0;
        return boundary::invoke(owner,kInterfaceId,1,false,[&]() {
            *value=g_rockConfig.configRevision(); return Status::Ok;
        });
    }
    Status ROCK_CALL visit(OwnerToken owner, Group group, VisitorV1 visitor, void* user) noexcept {
        if (!visitor) return Status::InvalidArgument;
        // Configuration tasks are not FRIK animation callbacks. RockConfig
        // serializes catalog access; borrowed visitors still forbid reentry.
        return boundary::invoke(owner,kInterfaceId,1,false,[&]() {
            VisitContext context{visitor,user};
            const bool available=g_rockConfig.visitSettings(static_cast<configuration_api::Group>(group),visitSetting,&context);
            if (context.fault) { provider::runtime::revoke(owner); return Status::OwnerRevoked; }
            return available?Status::Ok:Status::NotReady;
        });
    }
    Status ROCK_CALL setValue(OwnerToken owner, Group group, const char* section,
        const char* key, const char* value, char* error, std::uint32_t capacity) noexcept {
        if (error && capacity) error[0]='\0';
        if ((capacity && !error) || !section || !key || !value) return Status::InvalidArgument;
        return boundary::invoke(owner,kInterfaceId,2,false,[&]() {
            std::string message;
            const bool saved=g_rockConfig.persistSetting(static_cast<configuration_api::Group>(group),section,key,value,message);
            if (error && capacity) {
                const auto count=std::min(message.size(),static_cast<std::size_t>(capacity-1));
                std::copy_n(message.data(),count,error); error[count]='\0';
            }
            return saved?Status::Ok:Status::InvalidArgument;
        });
    }
}
const ApiV1& table() noexcept {
    static const ApiV1 value{revision,visit,setValue}; return value;
}
}

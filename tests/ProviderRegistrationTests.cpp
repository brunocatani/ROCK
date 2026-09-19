#include "api/ProviderRuntimeServices.h"
#include "api/ProviderStatePolicy.h"
#include "api/OwnedEventStream.h"
#include <ROCK/Core.h>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstring>
#include <future>
#include <latch>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>

namespace Version {constexpr std::string_view NAME="test";}
namespace rock::provider::events {
    api::Status bind(api::OwnerToken,api::InterfaceId);
    void remove(api::OwnerToken);
}
namespace provider_debug_overlay {void clear(std::uint64_t){}}
namespace provider_collider_visualization {void clear(std::uint64_t){}}
namespace rock::provider::runtime {
    // Exercise the actual registration/teardown service. Unrelated gameplay
    // resources are absent; the event stream uses the production bounded ring.
    struct Consumer {
        std::uint64_t token{};std::uint32_t grantedCapabilities{},providerGeneration{};
        InterfaceBinding interfaces[13]{};char modName[64]{};
    };
    std::array<Consumer,64> s_consumers{};
    std::mutex s_consumerMutex,s_snapshotMutex,s_interactionCommandMutex,s_handInputSuppressionMutex,
        s_weaponPartMutex,s_nativeAnimationAuthorityMutex,s_equippedWeaponHandlingAuthorityMutex,
        s_externalBodyMutex,s_touchGrabMutex,s_offhandReservationMutex,s_callbackMutex,s_animationPhaseCallbackMutex;
    struct Resource {void clearOwner(std::uint64_t){}} s_externalBodies,s_touchGrabTargets;
    struct Reservation {std::uint64_t ownerToken{};} s_offhandReservationSlot;
    struct CallbackSlot {std::uint64_t token{},ownerToken{};RockProviderFrameCallback callback{};void* userData{};};
    struct AnimationPhaseCallbackSlot {std::uint64_t token{},ownerToken{};RockProviderAnimationPhaseCallbackV1 callback{};void* userData{};};
    std::array<CallbackSlot,16> s_callbacks{};
    std::array<AnimationPhaseCallbackSlot,16> s_animationPhaseCallbacks{};
    std::atomic<std::uint64_t> s_nextCallbackToken{1},s_nextAnimationPhaseCallbackToken{1};
    RockProviderFrameSnapshot s_lastSnapshot{};bool s_hasSnapshot{};
    struct Physics {
        bool isProviderReady(){return false;}
        void releaseProviderPowerArmorGrabs(std::uint64_t){}
    };
    struct Instance {struct Read {Physics* get()const{return nullptr;}};Read borrow(){return {};}} s_physicsInteraction;
    constexpr std::uint32_t kImplementedConsumerCapabilitiesV1=0xFFFFFFFFu;
    std::uint32_t currentProviderGenerationForRegistration(){return 1;}
    std::uint64_t nextConsumerToken(){static std::uint64_t next=1;return next++;}
    std::size_t boundedStringLength(const char* text,std::size_t max){std::size_t n=0;while(n<max && text[n])++n;return n;}
    bool modNameEquals(const Consumer& slot,const char* name,std::size_t n){return slot.token && std::strlen(slot.modName)==n && std::memcmp(slot.modName,name,n)==0;}
    Consumer* findConsumerSlotLocked(std::uint64_t owner){for(auto& s:s_consumers)if(s.token==owner)return &s;return nullptr;}
    RockProviderResultV1 validateRegisteredOwnerCapabilityLocked(std::uint64_t owner,RockProviderConsumerCapabilityV1){return findConsumerSlotLocked(owner)?RockProviderResultV1::Ok:RockProviderResultV1::OwnerNotRegistered;}
    void clearInteractionCommandsForOwnerLocked(std::uint64_t,RockProviderInteractionFailureV1){}
    void clearHandInputSuppressionsForOwnerLocked(std::uint64_t,RockProviderHand,RockProviderSuppressionInvalidationReasonV1){}
    void clearWeaponPartTargetsForOwnerLocked(std::uint64_t){}
    void clearWeaponPartDrivesForOwnerLocked(std::uint64_t){}
    void clearNativeAnimationAuthorityForOwnerLocked(std::uint64_t){}
    void clearEquippedWeaponHandlingAuthorityForOwnerLocked(std::uint64_t){}
    void clearOffhandReservationLocked(RockProviderSuppressionInvalidationReasonV1){}
    void clearAnimationPhaseCallbacksForOwnerLocked(std::uint64_t){}
    bool clearHandVisualAuthorityForOwner(std::uint64_t,RockProviderHand,bool){return true;}
    void clearNativeAnimationRuntimePublicationForOwner(std::uint64_t){}
    #include "api/services/CoreService.inl"
}
namespace {
    rock::provider::OwnedEventStream<rock::api::core::EventV1> stream;
    std::latch removalEntered{1},allowRemoval{1},registrationStarted{1};
    bool pauseRemoval=false;
}
namespace rock::provider::events {
    api::Status bind(api::OwnerToken owner,api::InterfaceId){return stream.bind(owner);}
    void remove(api::OwnerToken owner){
        // Owner capacity must remain reserved until every event slot is retired.
        assert(runtime::findConsumerSlotLocked(owner)!=nullptr);
        if(pauseRemoval){removalEntered.count_down();allowRemoval.wait();}
        stream.remove(owner);
    }
}
int main(){
    using namespace rock::provider;using namespace rock::provider::runtime;
    std::array<RockProviderConsumerHandleV1,64> owners{};
    for(unsigned i=0;i<owners.size();++i){
        RockProviderConsumerRegistrationV1 registration{};
        const auto name="Owner"+std::to_string(i);std::memcpy(registration.modName,name.c_str(),name.size()+1);
        assert(apiRegisterConsumerV1(&registration,&owners[i])==RockProviderResultV1::Ok);
    }
    pauseRemoval=true;
    auto retire=std::async(std::launch::async,[&]{return apiUnregisterConsumerV1(owners[0].ownerToken);});
    removalEntered.wait();
    RockProviderConsumerHandleV1 replacement{};
    auto registerNext=std::async(std::launch::async,[&]{
        RockProviderConsumerRegistrationV1 registration{};std::strcpy(registration.modName,"Replacement");
        registrationStarted.count_down();return apiRegisterConsumerV1(&registration,&replacement);
    });
    registrationStarted.wait();allowRemoval.count_down();
    assert(retire.get()==RockProviderResultV1::Ok && registerNext.get()==RockProviderResultV1::Ok);
    assert(replacement.ownerToken!=owners[0].ownerToken);
    rock::api::StreamV1 state{};
    assert(stream.copy(replacement.ownerToken,0,nullptr,0,state)==rock::api::Status::Ok);
    assert(stream.copy(owners[0].ownerToken,0,nullptr,0,state)==rock::api::Status::PermissionDenied);
    for(unsigned i=1;i<owners.size();++i)assert(stream.copy(owners[i].ownerToken,0,nullptr,0,state)==rock::api::Status::Ok);
    pauseRemoval=false;
    assert(apiUnregisterConsumerV1(replacement.ownerToken)==RockProviderResultV1::Ok);
    for(unsigned i=1;i<owners.size();++i)assert(apiUnregisterConsumerV1(owners[i].ownerToken)==RockProviderResultV1::Ok);
    // If event admission fails, registration must not leak its reserved owner.
    for(unsigned i=0;i<64;++i)assert(stream.bind(1000+i)==rock::api::Status::Ok);
    RockProviderConsumerRegistrationV1 registration{};std::strcpy(registration.modName,"NoEventCapacity");
    assert(apiRegisterConsumerV1(&registration,&replacement)==RockProviderResultV1::CapacityFull);
    for(const auto& owner:s_consumers)assert(owner.token==0);
}

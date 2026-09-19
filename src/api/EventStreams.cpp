#include "BorrowedCallbackContext.h"
#include <Windows.h>
#include "EventStreams.h"
#include "OwnedEventStream.h"
#include <algorithm>
#include <shared_mutex>
#include "ProviderRuntimeServices.h"
namespace rock::provider::events {
namespace {
    struct Callback { api::OwnerToken owner{}; api::grab::EventCallbackV1 function{}; void* user{}; };
    std::array<Callback,64> callbacks{};
    std::shared_mutex callbackMutex;
    bool invokeGrabCallback(const Callback& slot,const api::grab::EventV1& event) {
#if defined(_MSC_VER)
        __try { slot.function(&event,slot.user); return true; }
        __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
#else
        slot.function(&event,slot.user); return true;
#endif
    }

    OwnedEventStream<api::core::EventV1,api::core::kEventCapacityPerOwner,api::core::kMaxOwners> coreEvents;
    OwnedEventStream<api::collision::EventV1,api::collision::kEventCapacityPerOwner,api::core::kMaxOwners> collisionEvents;
    OwnedEventStream<api::grab::EventV1,api::grab::kEventCapacityPerOwner,api::core::kMaxOwners> grabEvents;
    OwnedEventStream<api::touch::EventV1,api::touch::kEventCapacityPerOwner,api::core::kMaxOwners> touchEvents;
    OwnedEventStream<api::weapon::EventV1,api::weapon::kEventCapacityPerOwner,api::core::kMaxOwners> weaponEvents;
    OwnedEventStream<api::weaponparts::EventV1,api::weaponparts::kEventCapacityPerOwner,api::core::kMaxOwners> weaponpartsEvents;
    OwnedEventStream<api::animation::EventV1,api::animation::kEventCapacityPerOwner,api::core::kMaxOwners> animationEvents;
    OwnedEventStream<api::input::EventV1,api::input::kEventCapacityPerOwner,api::core::kMaxOwners> inputEvents;
    OwnedEventStream<api::diagnostics::EventV1,api::diagnostics::kEventCapacityPerOwner,api::core::kMaxOwners> diagnosticsEvents;
    template<class Event> void stamp(Event& value,const api::SampleV1& sample) {
        value.frameIndex=sample.frameIndex; value.worldGeneration=sample.worldGeneration;
        value.skeletonGeneration=sample.skeletonGeneration; value.providerGeneration=sample.providerGeneration;
    }
}
void bind(api::OwnerToken owner,api::InterfaceId id) { switch(id) {
    case api::InterfaceId::Core: coreEvents.bind(owner); break;
    case api::InterfaceId::Collision: collisionEvents.bind(owner); break;
    case api::InterfaceId::Grab: grabEvents.bind(owner); break;
    case api::InterfaceId::Touch: touchEvents.bind(owner); break;
    case api::InterfaceId::Weapon: weaponEvents.bind(owner); break;
    case api::InterfaceId::WeaponParts: weaponpartsEvents.bind(owner); break;
    case api::InterfaceId::Animation: animationEvents.bind(owner); break;
    case api::InterfaceId::Input: inputEvents.bind(owner); break;
    case api::InterfaceId::Diagnostics: diagnosticsEvents.bind(owner); break;
    default: break; } }
void remove(api::OwnerToken owner) {
    clearGrabCallback(owner);
    coreEvents.remove(owner);
    collisionEvents.remove(owner);
    grabEvents.remove(owner);
    touchEvents.remove(owner);
    weaponEvents.remove(owner);
    weaponpartsEvents.remove(owner);
    animationEvents.remove(owner);
    inputEvents.remove(owner);
    diagnosticsEvents.remove(owner);
}
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::core::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return coreEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::collision::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return collisionEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::grab::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return grabEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::touch::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return touchEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::weapon::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return weaponEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::weaponparts::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return weaponpartsEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::animation::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return animationEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::input::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return inputEvents.copy(owner,after,output,capacity,state); }
api::Status copy(api::OwnerToken owner,std::uint64_t after,api::diagnostics::EventV1* output,std::uint32_t capacity,api::StreamV1& state) { return diagnosticsEvents.copy(owner,after,output,capacity,state); }

void publishGrab(const rock::GrabEventData& source,const api::SampleV1& sample) {
    api::grab::EventV1 event{}; stamp(event,sample);
    event.kind=static_cast<std::uint32_t>(source.type); event.hand=source.isLeft?api::Hand::Left:api::Hand::Right;
    event.formId=source.formID; event.primaryBodyId=source.primaryBodyId; event.secondaryBodyId=source.secondaryBodyId;
    event.collisionLayer=source.collisionLayer; event.flags=source.flags; event.sourceKind=static_cast<std::uint32_t>(source.sourceKind);
    std::copy_n(source.positionGame,3,event.positionGame); std::copy_n(source.velocityGame,3,event.velocityGame);
    event.mass=source.mass; event.speedGameUnitsPerSecond=source.speedGameUnitsPerSecond; event.intensityHint=source.intensityHint;
    struct Delivery { api::OwnerToken owner{}; api::grab::EventV1 event{}; };
    std::array<Delivery,64> deliveries{};
    std::size_t count=0;
    std::array<api::OwnerToken,64> faults{};
    std::size_t faultCount=0;
    {
        std::shared_lock lock(callbackMutex);
        grabEvents.publish(event,0,[&](api::OwnerToken owner,const api::grab::EventV1& value) {
            for (const auto& slot:callbacks) if (slot.owner==owner) { deliveries[count++]={owner,value}; break; }
        });
        for (std::size_t i=0;i<count;++i) for (const auto& slot:callbacks) if (slot.owner==deliveries[i].owner && slot.function) {
            borrowed_callback::Scope callbackScope;
            bool healthy=false;
            try { healthy=invokeGrabCallback(slot,deliveries[i].event); } catch (...) {}
            if (!healthy) faults[faultCount++]=slot.owner;
            break;
        }
    }
    for (std::size_t i=0;i<faultCount;++i) { clearGrabCallback(faults[i]); runtime::deferRevoke(faults[i]); }
}
void publishPhysics(std::uint32_t kind,bool left,std::uint32_t form,std::uint32_t layer,const api::SampleV1& sample) {
    if (kind==100 || kind==101) {
        api::collision::EventV1 event{}; stamp(event,sample); event.kind=kind-99;
        event.hand=left?api::Hand::Left:api::Hand::Right; event.formId=form; event.collisionLayer=layer;
        collisionEvents.publish(event);
    } else if (kind==102 || kind==103) {
        api::grab::EventV1 event{}; stamp(event,sample); event.kind=kind;
        event.hand=left?api::Hand::Left:api::Hand::Right; event.formId=form; event.collisionLayer=layer;
        grabEvents.publish(event);
    } else if (kind==104 || kind==105) {
        api::core::EventV1 event{}; stamp(event,sample); event.kind=kind-101;
        coreEvents.publish(event);
    }
}
void publish(const RockProviderEventV1& source) {
    const api::SampleV1 sample{source.frameIndex,0,source.worldGeneration,source.skeletonGeneration,source.providerGeneration,0};
    switch(source.kind) {
    case RockProviderEventKindV1::LifecycleChanged: {
        api::core::EventV1 event{}; stamp(event,sample); event.kind=1;
        event.lifecycleFlags=source.data[0]; event.providerReady=source.data[1]; event.reason=source.result;
        coreEvents.publish(event); break;
    }
    case RockProviderEventKindV1::GrabStateChanged: {
        api::grab::EventV1 event{}; stamp(event,sample); event.kind=100; event.hand=static_cast<api::Hand>(source.hand);
        event.formId=source.formId; event.subjectSequence=source.subjectSequence; event.phase=source.result;
        event.primaryBodyId=source.data[1]; event.flags=source.data[2]; grabEvents.publish(event); break;
    }
    case RockProviderEventKindV1::InteractionCommandTerminal: {
        api::grab::EventV1 event{}; stamp(event,sample); event.kind=101; event.hand=static_cast<api::Hand>(source.hand);
        event.ownerToken=source.ownerToken; event.subjectSequence=source.subjectSequence; event.commandState=source.result;
        event.formId=source.formId; event.commandKind=source.data[0]; event.failure=source.data[1];
        grabEvents.publish(event,source.ownerToken); break;
    }
    case RockProviderEventKindV1::EquippedWeaponTransitionTerminal: {
        api::weapon::EventV1 event{}; stamp(event,sample); event.kind=1; event.hand=static_cast<api::Hand>(source.hand);
        event.formId=source.formId; event.weaponGenerationKey=source.weaponGenerationKey; event.transitionSequence=source.subjectSequence;
        event.terminalResult=source.result; event.source=source.data[0]; event.flags=source.data[1]; weaponEvents.publish(event); break;
    }
    case RockProviderEventKindV1::AuthorityLost: {
        const auto authority=static_cast<RockProviderAuthorityKindV1>(source.data[0]);
        const auto owner=source.ownerToken;
        switch(authority) {
        case RockProviderAuthorityKindV1::NativeAnimation: { api::animation::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=1; event.reason=source.result; animationEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::HandVisual: { api::animation::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=2; event.reason=source.result; animationEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::NativeAnimationRuntime: { api::animation::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=3; event.reason=source.result; animationEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::HandInputSuppression: { api::input::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=1; event.reason=source.result; inputEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::WeaponPartTargets: { api::weaponparts::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=1; event.reason=source.result; weaponpartsEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::WeaponPartDrive: { api::weaponparts::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=2; event.reason=source.result; weaponpartsEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::EquippedWeaponHandling: { api::weapon::EventV1 event{}; stamp(event,sample); event.kind=2; event.ownerToken=owner; event.authority=1; event.reason=source.result; weaponEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::DebugOverlay: { api::diagnostics::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=1; event.reason=source.result; diagnosticsEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::ColliderVisualization: { api::diagnostics::EventV1 event{}; stamp(event,sample); event.kind=1; event.ownerToken=owner; event.authority=2; event.reason=source.result; diagnosticsEvents.publish(event,owner); break; }
        case RockProviderAuthorityKindV1::OffhandReservation: { api::grab::EventV1 event{}; stamp(event,sample); event.kind=104; event.ownerToken=owner; event.failure=source.result; grabEvents.publish(event,owner); break; }
        default: { api::core::EventV1 event{}; stamp(event,sample); event.kind=2; event.ownerToken=owner; event.reason=source.result; coreEvents.publish(event,owner); break; }
        }
        break;
    }
    default: break;
    }
}

bool inSynchronousCallback() noexcept { return borrowed_callback::active; }
api::Status setGrabCallback(api::OwnerToken owner,api::grab::EventCallbackV1 function,void* user) {
    std::unique_lock lock(callbackMutex);
    for (auto& slot:callbacks) if (slot.owner==owner) { slot={owner,function,user}; return api::Status::Ok; }
    for (auto& slot:callbacks) if (!slot.owner) { slot={owner,function,user}; return api::Status::Ok; }
    return api::Status::CapacityFull;
}
void clearGrabCallback(api::OwnerToken owner) {
    std::unique_lock lock(callbackMutex);
    for (auto& slot:callbacks) if (slot.owner==owner) slot={};
}

void publishTouch(std::uint64_t owner,std::uint64_t scope,const RockProviderTouchGrabStateV1& state,std::uint64_t frame) {
    api::touch::EventV1 event{};
    stamp(event,{frame,0,state.worldGeneration,state.skeletonGeneration,state.providerGeneration,state.collisionGeneration});
    event.kind=1; event.ownerToken=owner; event.scopeToken=scope; event.targetId=state.targetId;
    event.targetGeneration=state.targetGeneration; event.phase=static_cast<std::uint32_t>(state.phase);
    event.releaseReason=static_cast<std::uint32_t>(state.releaseReason); event.activeHandMask=state.activeHandMask;
    touchEvents.publish(event,owner);
}
}

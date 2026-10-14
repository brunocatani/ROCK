#include "GrabEventCallbacks.h"
#include "BorrowedCallbackContext.h"
#include "ProviderRuntimeServices.h"
#include <Windows.h>
#include <algorithm>
#include <array>
#include <shared_mutex>
namespace rock::provider::grab_callbacks {
namespace {
    struct Callback { api::OwnerToken owner{}; api::grab::EventCallback function{}; void* user{}; };
    std::array<Callback,64> callbacks{};
    std::shared_mutex callbackMutex;
    bool invoke(const Callback& slot,const api::grab::Event& event) {
#if defined(_MSC_VER)
        __try { slot.function(&event,slot.user); return true; }
        __except(EXCEPTION_EXECUTE_HANDLER) { return false; }
#else
        slot.function(&event,slot.user); return true;
#endif
    }
}
api::Status set(api::OwnerToken owner,api::grab::EventCallback function,void* user) {
    std::unique_lock lock(callbackMutex);
    for (auto& slot:callbacks) if (slot.owner==owner) { slot={owner,function,user}; return api::Status::Ok; }
    for (auto& slot:callbacks) if (!slot.owner) { slot={owner,function,user}; return api::Status::Ok; }
    return api::Status::CapacityFull;
}
void clear(api::OwnerToken owner) {
    std::unique_lock lock(callbackMutex);
    for (auto& slot:callbacks) if (slot.owner==owner) slot={};
}
void deliver(const rock::GrabEventData& source,const api::Sample& sample) {
    api::grab::Event event{};
    event.frameIndex=sample.frameIndex; event.worldGeneration=sample.worldGeneration;
    event.skeletonGeneration=sample.skeletonGeneration; event.providerGeneration=sample.providerGeneration;
    event.kind=static_cast<std::uint32_t>(source.type); event.hand=source.isLeft?api::Hand::Left:api::Hand::Right;
    event.formId=source.formID; event.primaryBodyId=source.primaryBodyId; event.secondaryBodyId=source.secondaryBodyId;
    event.collisionLayer=source.collisionLayer; event.flags=source.flags; event.sourceKind=static_cast<std::uint32_t>(source.sourceKind);
    std::copy_n(source.positionGame,3,event.positionGame); std::copy_n(source.velocityGame,3,event.velocityGame);
    event.mass=source.mass; event.speedGameUnitsPerSecond=source.speedGameUnitsPerSecond; event.intensityHint=source.intensityHint;
    std::array<api::OwnerToken,64> faults{};
    std::size_t faultCount=0;
    {
        std::shared_lock lock(callbackMutex);
        for (const auto& slot:callbacks) {
            if (!slot.owner || !slot.function) continue;
            borrowed_callback::Scope callbackScope;
            bool healthy=false;
            try { healthy=invoke(slot,event); } catch (...) {}
            if (!healthy) faults[faultCount++]=slot.owner;
        }
    }
    for (std::size_t i=0;i<faultCount;++i) { clear(faults[i]); runtime::deferRevoke(faults[i]); }
}
}

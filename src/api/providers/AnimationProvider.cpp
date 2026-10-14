#include "Boundary.h"
#include "EventBoundary.h"
#include <ROCK/Animation.h>

namespace rock::api::animation {
namespace {
using namespace boundary;
Status ROCK_CALL setNativeAnimationAuthority(OwnerToken owner, const NativeAnimationAuthorityRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    if ((request->flags & static_cast<std::uint32_t>(NativeAnimationAuthorityFlag::Weapon)) && !weaponSlotPublished(request->weaponSlot))
        return Status::TargetUnavailable;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiSetNativeAnimationAuthorityV1(owner, request));
    });
}
Status ROCK_CALL clearNativeAnimationAuthority(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiClearNativeAnimationAuthorityV1(owner));
    });
}
Status ROCK_CALL getNativeAnimationAuthorityState(OwnerToken owner, NativeAnimationAuthorityState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::apiGetNativeAnimationAuthorityStateV1(outState) ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL setHandVisualAuthority(OwnerToken owner, const HandVisualAuthorityRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiSetHandVisualAuthorityV1(owner, request));
    });
}
Status ROCK_CALL clearHandVisualAuthority(OwnerToken owner, Hand hand) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiClearHandVisualAuthorityV1(owner, static_cast<provider::RockProviderHand>(hand)));
    });
}
Status ROCK_CALL publishNativeAnimationRuntime(OwnerToken owner, const NativeAnimationRuntimePublication* publication) noexcept {
    if (const auto s = checkInput(publication); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiPublishNativeAnimationRuntimeV1(owner, publication));
    });
}
Status ROCK_CALL clearNativeAnimationRuntime(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiClearNativeAnimationRuntimeV1(owner));
    });
}
Status ROCK_CALL registerAnimationPhaseCallback(OwnerToken owner, PhaseCallback callback, void* userData, std::uint64_t* outCallbackToken) noexcept {
    if (outCallbackToken) *outCallbackToken = 0;
    return invoke(owner, kInterfaceId, 4, true, [&] {
        return static_cast<Status>(provider::runtime::apiRegisterAnimationPhaseCallbackV1(owner, callback, userData, outCallbackToken));
    });
}
Status ROCK_CALL unregisterAnimationPhaseCallback(OwnerToken owner, std::uint64_t callbackToken) noexcept {
    return invoke(owner, kInterfaceId, 4, true, [&] {
        return static_cast<Status>(provider::runtime::apiUnregisterAnimationPhaseCallbackV1(owner, callbackToken));
    });
}
Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return readSample(owner, kInterfaceId, outSample);
}
Status ROCK_CALL copyEvents(OwnerToken owner, std::uint64_t after, Event* events, std::uint32_t capacity, Stream* stream) noexcept {
    return copyEventStream(owner, kInterfaceId, after, events, capacity, kEventCapacityPerOwner, stream);
}
}
const Api& table() noexcept {
    static const Api value{
        &setNativeAnimationAuthority,
        &clearNativeAnimationAuthority,
        &getNativeAnimationAuthorityState,
        &setHandVisualAuthority,
        &clearHandVisualAuthority,
        &publishNativeAnimationRuntime,
        &clearNativeAnimationRuntime,
        &registerAnimationPhaseCallback,
        &unregisterAnimationPhaseCallback,
        &getSample,
        &copyEvents,
    };
    return value;
}
}

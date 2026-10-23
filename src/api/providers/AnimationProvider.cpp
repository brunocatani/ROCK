#include "Boundary.h"
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
        return provider::runtime::setNativeAnimationAuthority(owner, *request);
    });
}
Status ROCK_CALL clearNativeAnimationAuthority(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::clearNativeAnimationAuthority(owner);
    });
}
Status ROCK_CALL getNativeAnimationAuthorityState(OwnerToken owner, NativeAnimationAuthorityState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::getNativeAnimationAuthorityState(*outState) ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL setHandVisualAuthority(OwnerToken owner, const HandVisualAuthorityRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::setHandVisualAuthority(owner, *request);
    });
}
Status ROCK_CALL clearHandVisualAuthority(OwnerToken owner, Hand hand) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::clearHandVisualAuthority(owner, hand);
    });
}
Status ROCK_CALL publishNativeAnimationRuntime(OwnerToken owner, const NativeAnimationRuntimePublication* publication) noexcept {
    if (const auto s = checkInput(publication); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::publishNativeAnimationRuntime(owner, *publication);
    });
}
Status ROCK_CALL clearNativeAnimationRuntime(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return provider::runtime::clearNativeAnimationRuntime(owner);
    });
}
Status ROCK_CALL registerAnimationPhaseCallback(OwnerToken owner, const PhaseCallbackRegistration* registration, std::uint64_t* outCallbackToken) noexcept {
    // Validate the record before touching the token: a binary built against
    // the earlier (callback, userData, token) form lands here with a function
    // pointer as the record and its userData as the token pointer.
    if (const auto s = checkInput(registration); s != Status::Ok) return s;
    if (!outCallbackToken) return Status::InvalidArgument;
    *outCallbackToken = 0;
    if (!registration->callback || registration->phaseMask == 0 || (registration->phaseMask & ~kAllPhases) != 0)
        return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 4, true, [&] {
        return static_cast<Status>(provider::runtime::registerAnimationPhaseCallback(owner, *registration, outCallbackToken));
    });
}
Status ROCK_CALL unregisterAnimationPhaseCallback(OwnerToken owner, std::uint64_t callbackToken) noexcept {
    return invoke(owner, kInterfaceId, 4, true, [&] {
        return static_cast<Status>(provider::runtime::unregisterAnimationPhaseCallback(owner, callbackToken));
    });
}
Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return readSample(owner, kInterfaceId, outSample);
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
    };
    return value;
}
}

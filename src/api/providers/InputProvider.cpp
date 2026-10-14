#include "Boundary.h"
#include "EventBoundary.h"
#include <ROCK/Input.h>
#include "physics-interaction/input/PhysicalReloadInput.h"
#include "physics-interaction/input/SupportTriggerInput.h"

namespace rock::api::input {
namespace {
using namespace boundary;
bool validHand(Hand hand) noexcept { return hand == Hand::Right || hand == Hand::Left; }
Status ROCK_CALL setHandInputSuppression(OwnerToken owner, const HandInputSuppressionRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiSetHandInputSuppressionV1(owner, request));
    });
}
Status ROCK_CALL clearHandInputSuppression(OwnerToken owner, Hand hand) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(provider::runtime::apiClearHandInputSuppressionV1(owner, static_cast<api::Hand>(hand)));
    });
}
Status ROCK_CALL getHandInputSuppressionState(OwnerToken owner, Hand hand, HandInputSuppressionState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return static_cast<Status>(provider::runtime::apiGetHandInputSuppressionStateV1(owner, static_cast<api::Hand>(hand), outState));
    });
}
Status ROCK_CALL getRawWandButtonState(OwnerToken owner, Hand hand, std::uint32_t buttonId, RawWandButtonState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::apiGetRawWandButtonStateV1(static_cast<api::Hand>(hand), buttonId, outState) ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getRawWandThumbstick(OwnerToken owner, Hand hand, float* outX, float* outY) noexcept {
    if (!outX || !outY) return Status::InvalidArgument;
    *outX = 0.0f;
    *outY = 0.0f;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::apiGetRawWandThumbstickV1(static_cast<api::Hand>(hand), outX, outY) ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL isNativePipboyInputSuppressed(OwnerToken owner, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = 0;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        *outValue = provider::runtime::apiIsNativePipboyInputSuppressedV1() ? 1u : 0u;
        return Status::Ok;
    });
}
Status ROCK_CALL getNativeInputContext(OwnerToken owner, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = 0;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        *outValue = provider::runtime::apiGetNativeInputContextV1();
        return Status::Ok;
    });
}
Status ROCK_CALL getLogicalInputActionState(OwnerToken owner, LogicalInputAction action, LogicalInputActionState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return static_cast<Status>(provider::runtime::apiGetLogicalInputActionStateV1(owner, action, outState));
    });
}
Status ROCK_CALL getPlacementClickState(OwnerToken owner, PlacementClickState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return provider::runtime::getPlacementClickState(*outState); });
}
Status ROCK_CALL setReloadGesture(OwnerToken owner, const ReloadGestureRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return physical_reload_input::set(owner, *request); }, provider::OwnerAccess::Active);
}
Status ROCK_CALL clearReloadGesture(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return physical_reload_input::clear(owner); });
}
Status ROCK_CALL getReloadGesture(OwnerToken owner, ReloadGestureState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return physical_reload_input::read(*outState); });
}
Status ROCK_CALL setSupportTrigger(OwnerToken owner, const SupportTriggerRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return support_trigger_input::set(owner, *request); }, provider::OwnerAccess::Active);
}
Status ROCK_CALL clearSupportTrigger(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return support_trigger_input::clear(owner); });
}
Status ROCK_CALL getSupportTrigger(OwnerToken owner, SupportTriggerState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return support_trigger_input::read(*outState); });
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
        &setHandInputSuppression,
        &clearHandInputSuppression,
        &getHandInputSuppressionState,
        &getRawWandButtonState,
        &getRawWandThumbstick,
        &isNativePipboyInputSuppressed,
        &getNativeInputContext,
        &getLogicalInputActionState,
        &getPlacementClickState,
        &setReloadGesture,
        &clearReloadGesture,
        &getReloadGesture,
        &setSupportTrigger,
        &clearSupportTrigger,
        &getSupportTrigger,
        &getSample,
        &copyEvents,
    };
    return value;
}
}

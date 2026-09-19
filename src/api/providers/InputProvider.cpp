#include "InputMarshalling.h"
#include <ROCK/Discovery.h>
#include "EventBoundary.h"

namespace rock::api::input {
namespace {
using namespace boundary;
Status ROCK_CALL setHandInputSuppressionV1(std::uint64_t ownerToken, const HandInputSuppressionRequestV1* request) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        if (const auto s = checkInput(request); s != Status::Ok) return s;
        provider::RockProviderHandInputSuppressionRequestV1 native_request{};
        convert(native_request, *request);
        const auto result = provider::runtime::apiSetHandInputSuppressionV1(ownerToken, &native_request);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL clearHandInputSuppressionV1(std::uint64_t ownerToken, Hand hand) noexcept {
    return invoke(ownerToken, kInterfaceId, 2, true, [&]() -> Status {
        const auto result = provider::runtime::apiClearHandInputSuppressionV1(ownerToken, static_cast<provider::RockProviderHand>(hand));
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getRawWandButtonStateV1(OwnerToken ownerToken, Hand hand, std::uint32_t buttonId, RawWandButtonStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderRawWandButtonStateV1 native_outState{};
        const auto result = provider::runtime::apiGetRawWandButtonStateV1(static_cast<provider::RockProviderHand>(hand), buttonId, &native_outState);
        convert(*outState, native_outState);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL isNativePipboyInputSuppressedV1(OwnerToken ownerToken, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<std::uint32_t>(provider::runtime::apiIsNativePipboyInputSuppressedV1());
        return Status::Ok;
    });
}
Status ROCK_CALL getHandInputSuppressionStateV1(std::uint64_t ownerToken, Hand hand, HandInputSuppressionStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderHandInputSuppressionStateV1 native_outState{};
        const auto result = provider::runtime::apiGetHandInputSuppressionStateV1(ownerToken, static_cast<provider::RockProviderHand>(hand), &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getLogicalInputActionStateV1(std::uint64_t ownerToken, LogicalInputActionV1 action, LogicalInputActionStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderLogicalInputActionStateV1 native_outState{};
        const auto result = provider::runtime::apiGetLogicalInputActionStateV1(ownerToken, static_cast<provider::RockProviderLogicalInputActionV1>(action), &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}
Status ROCK_CALL getRawWandThumbstickV1(OwnerToken ownerToken, Hand hand, float* outX, float* outY) noexcept {
    if (!outX) return Status::InvalidArgument;
    *outX = {};
    if (!outY) return Status::InvalidArgument;
    *outY = {};
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        const auto result = provider::runtime::apiGetRawWandThumbstickV1(static_cast<provider::RockProviderHand>(hand), outX, outY);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getNativeInputContextV1(OwnerToken ownerToken, std::uint32_t* outValue) noexcept {
    if (!outValue) return Status::InvalidArgument;
    *outValue = {};
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        *outValue = static_cast<std::uint32_t>(provider::runtime::apiGetNativeInputContextV1());
        return Status::Ok;
    });
}

#include "InputEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &setHandInputSuppressionV1,
        &clearHandInputSuppressionV1,
        &getRawWandButtonStateV1,
        &isNativePipboyInputSuppressedV1,
        &getHandInputSuppressionStateV1,
        &getLogicalInputActionStateV1,
        &getRawWandThumbstickV1,
        &getNativeInputContextV1,
        &getSample,
        &copyEvents,
    };
    return value;
}
}

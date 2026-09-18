#include "HandsMarshalling.h"
#include <ROCK/Discovery.h>

namespace rock::api::hands {
namespace {
using namespace boundary;
Status ROCK_CALL getHandFrameV1(OwnerToken ownerToken, Hand hand, HandFrameV1* outFrame) noexcept {
    if (const auto s = checkOutput(outFrame); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderHandFrameV1 native_outFrame{};
        const auto result = provider::runtime::apiGetHandFrameV1(static_cast<provider::RockProviderHand>(hand), &native_outFrame);
        convert(*outFrame, native_outFrame);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getPresentedHandFrameV1(OwnerToken ownerToken, Hand hand, HandFrameV1* outFrame) noexcept {
    if (const auto s = checkOutput(outFrame); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderHandFrameV1 native_outFrame{};
        const auto result = provider::runtime::apiGetPresentedHandFrameV1(static_cast<provider::RockProviderHand>(hand), &native_outFrame);
        convert(*outFrame, native_outFrame);
        return result ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL getPresentedHandPoseV1(std::uint64_t ownerToken, Hand hand, PresentedHandPoseV1* outPose) noexcept {
    if (const auto s = checkOutput(outPose); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, true, [&]() -> Status {
        provider::RockProviderPresentedHandPoseV1 native_outPose{};
        const auto result = provider::runtime::apiGetPresentedHandPoseV1(ownerToken, static_cast<provider::RockProviderHand>(hand), &native_outPose);
        convert(*outPose, native_outPose);
        return static_cast<Status>(result);
    });
}

#include "HandsEndpoints.inl"
}
const ApiV1& table() noexcept {
    static const ApiV1 value{
        &getHandFrameV1,
        &getPresentedHandFrameV1,
        &getPresentedHandPoseV1,
        &getSample,
        &getHeadPose,
    };
    return value;
}
}

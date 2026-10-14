#include "SnapshotReads.h"
#include "Boundary.h"

namespace rock::api::hands {
namespace {
using namespace boundary;
Status ROCK_CALL getPresentedHandFrame(OwnerToken owner, Hand hand, HandFrame* outFrame) noexcept {
    if (const auto s = checkOutput(outFrame); s != Status::Ok) return s;
    if (hand != Hand::Right && hand != Hand::Left) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::getPresentedHandFrame(hand, *outFrame);
    });
}
Status ROCK_CALL getPresentedHandPose(OwnerToken owner, Hand hand, PresentedHandPose* outPose) noexcept {
    if (const auto s = checkOutput(outPose); s != Status::Ok) return s;
    if (hand != Hand::Right && hand != Hand::Left) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        return provider::runtime::getPresentedHandPose(owner, hand, *outPose);
    });
}
Status ROCK_CALL getSample(OwnerToken owner, Sample* outSample) noexcept {
    return readSample(owner, kInterfaceId, outSample);
}
}
const Api& table() noexcept {
    static const Api value{
        &getHandFrame,
        &getPresentedHandFrame,
        &getPresentedHandPose,
        &getHeadPose,
        &getRoles,
        &getSample,
    };
    return value;
}
}

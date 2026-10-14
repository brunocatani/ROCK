#include "SnapshotReads.h"
#include "Boundary.h"
#include "CommonMarshalling.h"

namespace rock::api::hands {
using namespace boundary;
Status ROCK_CALL getHandFrame(OwnerToken owner, Hand hand, HandFrame* outFrame) noexcept {
    if (const auto s = checkOutput(outFrame); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, false, [&] {
        return provider::runtime::apiGetHandFrame(hand, *outFrame);
    });
}

Status ROCK_CALL getHeadPose(OwnerToken owner, HeadPose* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample = sampleFromSnapshot(frame);
        using RockFlag = provider::RockProviderFrameEnrichmentFlagV1;
        if (frame.enrichmentFlags & static_cast<std::uint32_t>(RockFlag::HmdTransformValid))
            result->flags |= static_cast<std::uint32_t>(HeadPoseFlag::TransformValid);
        if (frame.enrichmentFlags & static_cast<std::uint32_t>(RockFlag::HmdForwardValid))
            result->flags |= static_cast<std::uint32_t>(HeadPoseFlag::ForwardValid);
        convert(result->transform, frame.hmdTransform);
        std::copy_n(frame.hmdForwardWorld, 3, result->forwardWorld);
        return Status::Ok;
    });
}

Status ROCK_CALL getRoles(OwnerToken owner, Roles* result) noexcept {
    if (const auto status=checkOutput(result); status!=Status::Ok) return status;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample=sampleFromSnapshot(frame);
        result->primary=static_cast<Hand>(frame.primaryHand);
        result->offhand=static_cast<Hand>(frame.offhandHand);
        return Status::Ok;
    });
}

}

namespace rock::api::collision {
using namespace boundary;
Status ROCK_CALL getEnvironment(OwnerToken owner, Environment* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample=sampleFromSnapshot(frame);
        result->gameToHavokScale=frame.gameToHavokScale;
        result->havokToGameScale=frame.havokToGameScale;
        result->physicsScaleRevision=frame.physicsScaleRevision;
        auto& weapon=result->weapons[0];
        weapon.weaponGenerationKey=frame.weaponGenerationKey;
        weapon.bodyCount=frame.weaponBodyCount;
        std::copy_n(frame.weaponBodyIds,std::size(weapon.bodyIds),weapon.bodyIds);
        return Status::Ok;
    });
}

}

namespace rock::api::weaponparts {
using namespace boundary;
Status ROCK_CALL getGripState(OwnerToken ownerToken, Hand hand, GripState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() -> Status {
        const auto result = provider::runtime::apiGetWeaponPartGripStateV1(static_cast<provider::RockProviderHand>(hand), outState);
        return result ? Status::Ok : Status::NotReady;
    });
}

}

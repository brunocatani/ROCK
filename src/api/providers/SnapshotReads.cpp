#include "SnapshotReads.h"
#include "HandsMarshalling.h"
#include "CollisionMarshalling.h"
#include "WeaponMarshalling.h"
#include "WeaponPartsMarshalling.h"

namespace rock::api::hands {
using namespace boundary;
Status ROCK_CALL getHandFrameV1(OwnerToken ownerToken, Hand hand, HandFrameV1* outFrame) noexcept {
    if (const auto s = checkOutput(outFrame); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() -> Status {
        provider::RockProviderHandFrameV1 native_outFrame{};
        const auto result = provider::runtime::apiGetHandFrameV1(static_cast<provider::RockProviderHand>(hand), &native_outFrame);
        convert(*outFrame, native_outFrame);
        return result ? Status::Ok : Status::NotReady;
    });
}

Status ROCK_CALL getHeadPose(OwnerToken owner, HeadPoseV1* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample = sampleFromSnapshot(frame);
        result->valid = (frame.enrichmentFlags >> 1) & 3u;
        convert(result->transform, frame.hmdTransform);
        std::copy_n(frame.hmdForwardWorld, 3, result->forwardWorld);
        return Status::Ok;
    });
}

Status ROCK_CALL getRoles(OwnerToken owner, RolesV1* result) noexcept {
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
Status ROCK_CALL getEnvironment(OwnerToken owner, EnvironmentV1* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::RockProviderFrameSnapshot frame{};
        if (!provider::runtime::apiGetFrameSnapshot(&frame)) return Status::NotReady;
        result->sample=sampleFromSnapshot(frame);
        result->externalBodyCount=frame.externalBodyCount;
        result->gameToHavokScale=frame.gameToHavokScale;
        result->havokToGameScale=frame.havokToGameScale;
        result->physicsScaleRevision=frame.physicsScaleRevision;
        result->weaponGenerationKey=frame.weaponGenerationKey;
        result->weaponBodyCount=frame.weaponBodyCount;
        std::copy_n(frame.weaponBodyIds,8,result->weaponBodyIds);
        return Status::Ok;
    });
}

}

namespace rock::api::weapon {
using namespace boundary;
Status ROCK_CALL getEquippedWeaponStateV1(std::uint64_t ownerToken, EquippedWeaponStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() -> Status {
        provider::RockProviderEquippedWeaponStateV1 native_outState{};
        const auto result = provider::runtime::apiGetEquippedWeaponStateV1(ownerToken, &native_outState);
        convert(*outState, native_outState);
        return static_cast<Status>(result);
    });
}

}

namespace rock::api::weaponparts {
using namespace boundary;
Status ROCK_CALL getWeaponPartGripStateV1(OwnerToken ownerToken, Hand hand, WeaponPartGripStateV1* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(ownerToken, kInterfaceId, 1, false, [&]() -> Status {
        const auto result = provider::runtime::apiGetWeaponPartGripStateV1(static_cast<provider::RockProviderHand>(hand), outState);
        return result ? Status::Ok : Status::NotReady;
    });
}

}

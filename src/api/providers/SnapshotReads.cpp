#include "SnapshotReads.h"
#include "Boundary.h"

namespace rock::api::hands {
using namespace boundary;
Status ROCK_CALL getHandFrame(OwnerToken owner, Hand hand, HandFrame* outFrame) noexcept {
    if (const auto s = checkOutput(outFrame); s != Status::Ok) return s;
    if (hand!=Hand::Right && hand!=Hand::Left) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, false, [&] {
        return provider::runtime::getHandFrame(hand, *outFrame);
    });
}

Status ROCK_CALL getHeadPose(OwnerToken owner, HeadPose* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::Publication frame{};
        if (!provider::runtime::copyPublication(frame)) return Status::NotReady;
        result->sample = frame.sample();
        using RockFlag = provider::PublicationFlag;
        if (frame.has(RockFlag::HmdTransformValid))
            result->flags |= static_cast<std::uint32_t>(HeadPoseFlag::TransformValid);
        if (frame.has(RockFlag::HmdForwardValid))
            result->flags |= static_cast<std::uint32_t>(HeadPoseFlag::ForwardValid);
        result->transform = frame.hmdTransform;
        std::copy_n(frame.hmdForwardWorld, 3, result->forwardWorld);
        return Status::Ok;
    });
}

Status ROCK_CALL getRoles(OwnerToken owner, Roles* result) noexcept {
    if (const auto status=checkOutput(result); status!=Status::Ok) return status;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::Publication frame{};
        if (!provider::runtime::copyPublication(frame)) return Status::NotReady;
        result->sample=frame.sample();
        result->right=frame.handRoles[0];
        result->left=frame.handRoles[1];
        return Status::Ok;
    });
}

}

namespace rock::api::collision {
using namespace boundary;
Status ROCK_CALL getEnvironment(OwnerToken owner, Environment* result) noexcept {
    if (const auto s=checkOutput(result); s!=Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, false, [&]() {
        provider::Publication frame{};
        if (!provider::runtime::copyPublication(frame)) return Status::NotReady;
        result->sample=frame.sample();
        result->gameToHavokScale=frame.gameToHavokScale;
        result->havokToGameScale=frame.havokToGameScale;
        result->physicsScaleRevision=frame.physicsScaleRevision;
        result->weapons[0]=frame.weapon;
        result->weapons[1]=frame.slotOneWeapon;
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
        return provider::runtime::copyPartGripState(hand, *outState) ? Status::Ok : Status::NotReady;
    });
}

}

#include "Boundary.h"
#include "EventBoundary.h"
#include <ROCK/Grab.h>
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"

namespace rock::api::grab {
namespace {
using namespace boundary;
namespace p = provider;

bool validHand(Hand hand) noexcept { return hand == Hand::Right || hand == Hand::Left; }

Status startCommand(std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = 0;
    return Status::Ok;
}

Status ROCK_CALL requestForceGrab(OwnerToken owner, const ForceGrabRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (request->flags & ~static_cast<std::uint32_t>(ForceGrabFlag::UsePreferredGrabPointGame)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::requestForceGrab(owner, *request, *outCommandId); });
}
Status ROCK_CALL requestForceRelease(OwnerToken owner, const ForceReleaseRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::requestForceRelease(owner, *request, *outCommandId); });
}
Status ROCK_CALL requestThrownDrop(OwnerToken owner, const ThrownDropRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::requestThrownDrop(owner, *request, *outCommandId); });
}
Status ROCK_CALL requestPowerArmorGrab(OwnerToken owner, const PowerArmorGrabRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::requestPowerArmorGrab(owner, *request, *outCommandId); });
}
// An inventory grab is a force grab that first drops one owned item.
Status ROCK_CALL requestInventoryGrab(OwnerToken owner, const InventoryGrabRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        ForceGrabRequest grab{};
        grab.hand = request->hand;
        grab.flags = p::kForceGrabFromPlayerInventory;
        grab.targetFormId = request->baseFormId;
        grab.worldGeneration = request->worldGeneration;
        grab.skeletonGeneration = request->skeletonGeneration;
        grab.providerGeneration = request->providerGeneration;
        return p::runtime::requestForceGrab(owner, grab, *outCommandId);
    });
}
Status ROCK_CALL getInteractionCommandResult(OwnerToken owner, std::uint64_t commandId, InteractionCommandResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getInteractionCommandResult(owner, commandId, *outResult); });
}
Status ROCK_CALL cancelInteractionCommand(OwnerToken owner, std::uint64_t commandId) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::cancelInteractionCommand(owner, commandId); });
}
Status ROCK_CALL getHandInteractionState(OwnerToken owner, Hand hand, HandInteractionState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    // The service copies an immutable-value publication under s_snapshotMutex;
    // it does not read live hands, so F4SE tasks get the same safe snapshot.
    return invoke(owner, kInterfaceId, 1, false, [&] { return p::runtime::getHandInteractionState(owner, hand, *outState); });
}
Status ROCK_CALL getHandTargetDetails(OwnerToken owner, Hand hand, HandTargetDetails* outDetails) noexcept {
    if (const auto s = checkOutput(outDetails); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getHandTargetDetails(owner, hand, *outDetails); });
}
Status ROCK_CALL acquireHandReservation(OwnerToken owner, const HandReservationRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::acquireHandReservation(owner, *request); });
}
Status ROCK_CALL renewHandReservation(OwnerToken owner, const HandReservationRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::renewHandReservation(owner, *request); });
}
Status ROCK_CALL releaseHandReservation(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::releaseHandReservation(owner); });
}
Status ROCK_CALL getHandReservationState(OwnerToken owner, HandReservationState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getHandReservationState(owner, *outState); });
}
Status ROCK_CALL getHeldPlacementState(OwnerToken owner, HeldPlacementState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getHeldPlacementState(*outState); });
}
Status ROCK_CALL submitHeldPlacementIntent(OwnerToken owner, const HeldPlacementIntent* intent) noexcept {
    if (const auto s = checkInput(intent); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::submitHeldPlacementIntent(owner, *intent); });
}
Status ROCK_CALL clearHeldPlacementIntent(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::clearHeldPlacementIntent(owner); });
}
Status ROCK_CALL setDefaultGrabPoses(OwnerToken owner, const DefaultGrabPoses* poses) noexcept {
    if (const auto s = checkInput(poses); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return saved_grab_offset::setDefaultGrabPoses(owner, *poses); });
}
Status ROCK_CALL clearDefaultGrabPoses(OwnerToken owner, std::uint32_t baseFormId) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return saved_grab_offset::clearDefaultGrabPoses(owner, baseFormId); });
}
Status ROCK_CALL setHeldObjectTarget(OwnerToken owner, const HeldObjectTarget* target) noexcept {
    if (const auto s = checkInput(target); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(target->connectedWeaponSlot); s != Status::Ok) return s;
    if ((target->connectedWeaponFormId || target->connectedWeaponGeneration) && !weaponSlotPublished(target->connectedWeaponSlot))
        return Status::TargetUnavailable;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::setHeldObjectTarget(owner, *target); });
}
Status ROCK_CALL clearHeldObjectTarget(OwnerToken owner, Hand hand) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::clearHeldObjectTarget(owner, hand); });
}
Status ROCK_CALL getHeldObjectPose(OwnerToken owner, Hand hand, HeldObjectPose* outPose) noexcept {
    if (const auto s = checkOutput(outPose); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::getHeldObjectPose(owner, hand, *outPose); });
}
Status ROCK_CALL queryReferenceInteraction(OwnerToken owner, const ReferenceQuery* query, ReferenceInteraction* outInteraction) noexcept {
    if (const auto s = checkOutput(outInteraction); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::queryReferenceInteraction(owner, *query, *outInteraction); });
}
Status ROCK_CALL queryPowerArmorTarget(OwnerToken owner, const ReferenceQuery* query, PowerArmorTarget* outTarget) noexcept {
    if (const auto s = checkOutput(outTarget); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::runtime::queryPowerArmorTarget(owner, *query, *outTarget); });
}
Status ROCK_CALL setEventCallback(OwnerToken owner, EventCallback callback, void* userData) noexcept {
    if (!callback) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] { return p::events::setGrabCallback(owner, callback, userData); }, p::OwnerAccess::Active);
}
Status ROCK_CALL clearEventCallback(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 1, true, [&] { p::events::clearGrabCallback(owner); return Status::Ok; });
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
        &requestForceGrab,
        &requestForceRelease,
        &requestThrownDrop,
        &requestPowerArmorGrab,
        &requestInventoryGrab,
        &getInteractionCommandResult,
        &cancelInteractionCommand,
        &getHandInteractionState,
        &getHandTargetDetails,
        &acquireHandReservation,
        &renewHandReservation,
        &releaseHandReservation,
        &getHandReservationState,
        &getHeldPlacementState,
        &submitHeldPlacementIntent,
        &clearHeldPlacementIntent,
        &setDefaultGrabPoses,
        &clearDefaultGrabPoses,
        &setHeldObjectTarget,
        &clearHeldObjectTarget,
        &getHeldObjectPose,
        &queryReferenceInteraction,
        &queryPowerArmorTarget,
        &setEventCallback,
        &clearEventCallback,
        &getSample,
        &copyEvents,
    };
    return value;
}
}

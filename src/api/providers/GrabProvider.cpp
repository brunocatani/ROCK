#include "Boundary.h"
#include "EventBoundary.h"
#include <ROCK/Grab.h>
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include <algorithm>

namespace rock::api::grab {
namespace {
using namespace boundary;
namespace p = provider;

// ROCK's runtime keeps its own command, hand-state and reference records; the
// conversions below are the only place they meet the public major-2 records.
api::Hand toProvider(Hand hand) noexcept { return static_cast<api::Hand>(hand); }
bool validHand(Hand hand) noexcept { return hand == Hand::Right || hand == Hand::Left; }

p::RockProviderForceGrabRequestV1 toProvider(const ForceGrabRequest& in) noexcept {
    p::RockProviderForceGrabRequestV1 out{};
    out.hand = toProvider(in.hand);
    out.flags = in.flags;
    out.targetFormId = in.targetFormId;
    out.targetBodyId = in.targetBodyId;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    out.maxDistanceGame = in.maxDistanceGame;
    std::copy_n(in.preferredGrabPointGame, 3, out.preferredGrabPointGame);
    return out;
}
template <class Native, class Public>
Native toProviderRelease(const Public& in) noexcept {
    Native out{};
    out.hand = toProvider(in.hand);
    out.flags = in.flags;
    out.targetFormId = in.targetFormId;
    out.targetBodyId = in.targetBodyId;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    std::copy_n(in.linearVelocityHavok, 3, out.linearVelocityHavok);
    std::copy_n(in.angularVelocityRadiansPerSecond, 3, out.angularVelocityRadiansPerSecond);
    return out;
}
p::RockProviderReferenceQueryV1 toProvider(const ReferenceKey& in) noexcept {
    p::RockProviderReferenceQueryV1 out{};
    out.referenceFormId = in.referenceFormId;
    out.referenceNativeHandle = in.referenceNativeHandle;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    return out;
}
p::RockProviderReferenceQueryV1 toProvider(const ReferenceQuery& in) noexcept {
    auto out = toProvider(ReferenceKey{ in.referenceFormId, in.referenceNativeHandle, in.worldGeneration,
        in.skeletonGeneration, in.providerGeneration });
    out.furnitureMarkerIndex = in.furnitureMarkerIndex;
    return out;
}
p::RockProviderOffhandReservationRequestV1 toProvider(const HandReservationRequest& in) noexcept {
    p::RockProviderOffhandReservationRequestV1 out{};
    out.hand = toProvider(in.hand);
    out.reservation = static_cast<api::grab::HandReservationKind>(in.reservation);
    out.leaseFrames = in.leaseFrames;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    return out;
}

void fill(InteractionCommandResult& out, const p::RockProviderInteractionCommandResultV1& in) noexcept {
    out.ownerToken = in.ownerToken;
    out.commandId = in.commandId;
    out.kind = static_cast<InteractionCommandKind>(in.kind);
    out.state = static_cast<InteractionCommandState>(in.state);
    out.failure = static_cast<InteractionFailure>(in.failure);
    out.hand = static_cast<Hand>(in.hand);
    out.targetFormId = in.targetFormId;
    out.targetBodyId = in.targetBodyId;
    out.frameIndex = in.frameIndex;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    out.stage = static_cast<CommandStage>(in.stage);
    out.failureStage = static_cast<InteractionFailure>(in.failureStage);
    out.acceptedFrame = in.acceptedFrame;
    out.committedFrame = in.committedFrame;
    out.appliedFrame = in.appliedFrame;
}
void fill(HandInteractionState& out, const p::RockProviderHandInteractionStateV1& in) noexcept {
    out.frameIndex = in.frameIndex;
    out.hand = static_cast<Hand>(in.hand);
    out.phase = static_cast<HandInteractionPhase>(in.phase);
    out.targetKind = static_cast<BodyContactTargetKind>(in.targetKind);
    out.flags = in.flags;
    out.reservedTargetIdentity = in.reservedTargetIdentity;
    out.targetFormId = in.targetFormId;
    out.primaryBodyId = in.primaryBodyId;
    out.heldBodyCount = in.heldBodyCount;
    std::copy_n(in.heldBodyIds, std::size(out.heldBodyIds), out.heldBodyIds);
    out.stateSequence = in.stateSequence;
    out.targetSequence = in.targetSequence;
    out.gripSequence = in.gripSequence;
    out.releaseSequence = in.releaseSequence;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    out.collisionGeneration = in.collisionGeneration;
    out.surfaceAnchorGame = in.surfaceAnchorGame;
    out.surfaceGripMode = static_cast<touch::SurfaceGripMode>(in.surfaceGripMode);
}
void fill(ReferenceInteraction& out, const p::RockProviderReferenceInteractionV1& in) noexcept {
    out.flags = in.flags;
    out.referenceFormId = in.referenceFormId;
    out.referenceNativeHandle = in.referenceNativeHandle;
    out.baseFormId = in.baseFormId;
    out.baseFormType = in.baseFormType;
    out.activationBlocked = in.activationBlocked;
    out.openState = static_cast<NativeOpenState>(in.openState);
    out.furnitureInUse = in.furnitureInUse;
    out.furnitureInUseIncludingReservations = in.furnitureInUseIncludingReservations;
    out.furnitureMarkerIndex = in.furnitureMarkerIndex;
    out.worldGeneration = in.worldGeneration;
    out.frameIndex = in.frameIndex;
}
void fill(HandTargetDetails& out, const p::RockProviderHandTargetDetailsV1& in) noexcept {
    fill(out.handState, in.handState);
    out.referenceFormId = in.reference.referenceFormId;
    out.referenceNativeHandle = in.reference.referenceNativeHandle;
    out.flags = in.flags;
    out.collisionLayer = in.collisionLayer;
    out.anchorGame = in.anchorGame;
    out.normalGame = in.normalGame;
    out.powerArmorPoint = static_cast<PowerArmorPoint>(in.powerArmorPoint);
    out.sourceTriangleIndex = in.sourceTriangleIndex;
    std::copy_n(in.collisionNodeName, std::size(out.collisionNodeName), out.collisionNodeName);
    std::copy_n(in.meshPartName, std::size(out.meshPartName), out.meshPartName);
}
void fill(HandReservationState& out, const p::RockProviderOffhandReservationStateV1& in) noexcept {
    out.hand = static_cast<Hand>(in.hand);
    out.reservation = static_cast<HandReservationKind>(in.reservation);
    out.ownerToken = in.ownerToken;
    out.expiresAfterFrame = in.expiresAfterFrame;
    out.active = in.active;
    out.remainingFrames = in.remainingFrames;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
}
void fill(PowerArmorTarget& out, const p::RockProviderPowerArmorTargetV1& in) noexcept {
    fill(out.touchedReference, in.touchedReference);
    fill(out.frameReference, in.frameReference);
    out.flags = in.flags;
    out.actorFormId = in.actorFormId;
    for (std::size_t i = 0; i < std::size(out.points); ++i) {
        out.points[i].point = static_cast<PowerArmorPoint>(in.points[i].point);
        out.points[i].valid = in.points[i].valid;
        out.points[i].world = in.points[i].world;
        out.points[i].frameLocal = in.points[i].frameLocal;
    }
}

Status startCommand(std::uint64_t* outCommandId) noexcept {
    if (!outCommandId) return Status::InvalidArgument;
    *outCommandId = 0;
    return Status::Ok;
}

Status ROCK_CALL requestForceGrab(OwnerToken owner, const ForceGrabRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (request->flags & ~static_cast<std::uint32_t>(ForceGrabFlag::UsePreferredGrabPointGame)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        const auto native = toProvider(*request);
        return static_cast<Status>(p::runtime::apiRequestForceGrabV1(owner, &native, outCommandId));
    });
}
Status ROCK_CALL requestForceRelease(OwnerToken owner, const ForceReleaseRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        const auto native = toProviderRelease<p::RockProviderForceReleaseRequestV1>(*request);
        return static_cast<Status>(p::runtime::apiRequestForceReleaseV1(owner, &native, outCommandId));
    });
}
Status ROCK_CALL requestThrownDrop(OwnerToken owner, const ThrownDropRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        const auto native = toProviderRelease<p::RockProviderThrownDropRequestV1>(*request);
        return static_cast<Status>(p::runtime::apiRequestThrownDropV1(owner, &native, outCommandId));
    });
}
Status ROCK_CALL requestPowerArmorGrab(OwnerToken owner, const PowerArmorGrabRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        p::RockProviderPowerArmorGrabRequestV1 native{};
        native.target = toProvider(request->target);
        native.hand = toProvider(request->hand);
        native.point = static_cast<api::PowerArmorPoint>(request->point);
        native.maxDistanceGame = request->maxDistanceGame;
        return static_cast<Status>(p::runtime::apiRequestPowerArmorGrabV1(owner, &native, outCommandId));
    });
}
Status ROCK_CALL requestInventoryGrab(OwnerToken owner, const InventoryGrabRequest* request, std::uint64_t* outCommandId) noexcept {
    if (const auto s = startCommand(outCommandId); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        p::RockProviderForceGrabRequestV1 native{};
        native.hand = toProvider(request->hand);
        native.targetFormId = request->baseFormId;
        native.worldGeneration = request->worldGeneration;
        native.skeletonGeneration = request->skeletonGeneration;
        native.providerGeneration = request->providerGeneration;
        native.flags = static_cast<std::uint32_t>(p::RockProviderForceGrabFlagV1::FromPlayerInventory);
        return static_cast<Status>(p::runtime::apiRequestForceGrabV1(owner, &native, outCommandId));
    });
}
Status ROCK_CALL getInteractionCommandResult(OwnerToken owner, std::uint64_t commandId, InteractionCommandResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        p::RockProviderInteractionCommandResultV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiGetInteractionCommandResultV1(owner, commandId, &native));
        fill(*outResult, native);
        return status;
    });
}
Status ROCK_CALL cancelInteractionCommand(OwnerToken owner, std::uint64_t commandId) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(p::runtime::apiCancelInteractionCommandV1(owner, commandId));
    });
}
Status ROCK_CALL getHandInteractionState(OwnerToken owner, Hand hand, HandInteractionState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    // The service copies an immutable-value publication under s_snapshotMutex;
    // it does not read live hands, so F4SE tasks get the same safe snapshot.
    return invoke(owner, kInterfaceId, 1, false, [&] {
        p::RockProviderHandInteractionStateV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiGetHandInteractionStateV1(owner, toProvider(hand), &native));
        fill(*outState, native);
        return status;
    });
}
Status ROCK_CALL getHandTargetDetails(OwnerToken owner, Hand hand, HandTargetDetails* outDetails) noexcept {
    if (const auto s = checkOutput(outDetails); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        p::RockProviderHandTargetDetailsV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiGetHandTargetDetailsV1(owner, toProvider(hand), &native));
        fill(*outDetails, native);
        return status;
    });
}
Status ROCK_CALL acquireHandReservation(OwnerToken owner, const HandReservationRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        const auto native = toProvider(*request);
        return static_cast<Status>(p::runtime::apiAcquireOffhandReservationV1(owner, &native));
    });
}
Status ROCK_CALL renewHandReservation(OwnerToken owner, const HandReservationRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        const auto native = toProvider(*request);
        return static_cast<Status>(p::runtime::apiRenewOffhandReservationV1(owner, &native));
    });
}
Status ROCK_CALL releaseHandReservation(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return static_cast<Status>(p::runtime::apiReleaseOffhandReservationV1(owner));
    });
}
Status ROCK_CALL getHandReservationState(OwnerToken owner, HandReservationState* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        p::RockProviderOffhandReservationStateV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiGetOffhandReservationStateV1(owner, &native));
        fill(*outState, native);
        return status;
    });
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
    return invoke(owner, kInterfaceId, 1, true, [&] {
        const auto nativeQuery = toProvider(*query);
        p::RockProviderReferenceInteractionV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiQueryReferenceInteractionV1(owner, &nativeQuery, &native));
        fill(*outInteraction, native);
        return status;
    });
}
Status ROCK_CALL queryPowerArmorTarget(OwnerToken owner, const ReferenceQuery* query, PowerArmorTarget* outTarget) noexcept {
    if (const auto s = checkOutput(outTarget); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        const auto nativeQuery = toProvider(*query);
        p::RockProviderPowerArmorTargetV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiQueryPowerArmorTargetV1(owner, &nativeQuery, &native));
        fill(*outTarget, native);
        return status;
    });
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

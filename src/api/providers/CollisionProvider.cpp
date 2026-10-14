#include "Boundary.h"
#include "EventBoundary.h"
#include "SnapshotReads.h"
#include <ROCK/Collision.h>
#include <algorithm>
#include <array>

namespace rock::api::collision {
namespace {
using namespace boundary;
namespace p = provider;

// ROCK's runtime keeps its own contact, collider and raycast records; these
// conversions are the only place they meet the public major-2 records.
bool validHand(Hand hand) noexcept { return hand == Hand::Right || hand == Hand::Left; }

void fill(SemanticHandContact& out, const p::RockProviderSemanticHandContactV1& in) noexcept {
    out.frameIndex = in.frameIndex;
    out.hand = static_cast<Hand>(in.hand);
    out.role = in.role;
    out.finger = in.finger;
    out.segment = in.segment;
    out.handBodyId = in.handBodyId;
    out.targetBodyId = in.targetBodyId;
    out.targetFormId = in.targetFormId;
    out.flags = in.flags;
    out.contactState = static_cast<SemanticContactState>(in.contactState);
    out.framesSinceContact = in.framesSinceContact;
    out.contactSequence = in.contactSequence;
    out.contactPointGame = in.contactPointGame;
    out.contactNormalGame = in.contactNormalGame;
    out.collisionGeneration = in.collisionGeneration;
}
void fill(PlayerColliderDescriptor& out, const p::RockProviderPlayerColliderDescriptorV1& in) noexcept {
    out.frameIndex = in.frameIndex;
    out.kind = static_cast<PlayerColliderKind>(in.kind);
    out.hand = static_cast<Hand>(in.hand);
    out.bodyId = in.bodyId;
    out.role = in.role;
    out.zone = static_cast<BodyZoneKind>(in.zone);
    out.side = static_cast<BodyZoneSide>(in.side);
    out.descriptorIndex = in.descriptorIndex;
    out.flags = in.flags;
    out.lengthGameUnits = in.lengthGameUnits;
    out.radiusGameUnits = in.radiusGameUnits;
    out.transform = in.transform;
    out.collisionGeneration = in.collisionGeneration;
}
void fill(BodyContact& out, const p::RockProviderBodyContactV1& in) noexcept {
    out.frameIndex = in.frameIndex;
    out.bodyId = in.bodyId;
    out.targetBodyId = in.targetBodyId;
    out.bodyLayer = in.bodyLayer;
    out.targetLayer = in.targetLayer;
    out.zone = static_cast<BodyZoneKind>(in.zone);
    out.side = static_cast<BodyZoneSide>(in.side);
    out.role = in.role;
    out.descriptorIndex = in.descriptorIndex;
    out.targetKind = static_cast<BodyContactTargetKind>(in.targetKind);
    out.targetZone = static_cast<BodyZoneKind>(in.targetZone);
    out.targetSide = static_cast<BodyZoneSide>(in.targetSide);
    out.targetRole = in.targetRole;
    out.targetDescriptorIndex = in.targetDescriptorIndex;
    out.inPowerArmor = in.inPowerArmor;
    out.targetInPowerArmor = in.targetInPowerArmor;
    out.hasContactPointGame = in.hasContactPointGame;
    out.contactPointGame = in.contactPointGame;
}
void fill(HandCollisionAvailability& out, const p::RockProviderHandCollisionAvailabilityV1& in) noexcept {
    out.frameIndex = in.frameIndex;
    out.hand = static_cast<Hand>(in.hand);
    out.flags = in.flags;
    out.collisionSequence = in.collisionSequence;
    out.collisionGeneration = in.collisionGeneration;
    out.handBodyCount = in.handBodyCount;
    out.dynamicTwinCount = in.dynamicTwinCount;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    // The runtime record carries these four counts in its spare words.
    out.otherHandContactSlots = in.reserved[0];
    out.weaponContactSlots = in.reserved[1];
    out.dynamicCollisionLayer = in.reserved[2];
    out.pairSuppressionLeaseCount = in.reserved[3];
    out.collisionEnabledBodyCount = in.collisionEnabledBodyCount;
    out.filterKnownBodyCount = in.filterKnownBodyCount;
}
void fill(WorldRaycastResult& out, const p::RockProviderWorldRaycastResultV1& in) noexcept {
    out.hit = in.hit;
    out.flags = in.flags;
    out.hitFraction = in.hitFraction;
    out.hitDistanceGame = in.hitDistanceGame;
    out.hitPointGame = in.hitPointGame;
    out.hitNormalGame = in.hitNormalGame;
    out.frameIndex = in.frameIndex;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
}
p::RockProviderWorldRaycastRequestV1 toProvider(const WorldRaycastRequest& in) noexcept {
    p::RockProviderWorldRaycastRequestV1 out{};
    out.startGame = in.startGame;
    out.directionGame = in.directionGame;
    out.maxDistanceGame = in.maxDistanceGame;
    out.worldGeneration = in.worldGeneration;
    out.skeletonGeneration = in.skeletonGeneration;
    out.providerGeneration = in.providerGeneration;
    return out;
}

// Validates a caller array of public records before any write.
template <class Record>
Status checkOutputs(Record* records, std::uint32_t capacity, std::uint32_t maxCapacity, std::uint32_t* outCount) noexcept {
    if (!outCount) return Status::InvalidArgument;
    *outCount = 0;
    if (capacity > maxCapacity) return Status::CapacityFull;
    if (capacity && !records) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < capacity; ++i)
        if (const auto s = checkOutput(records + i); s != Status::Ok) return s;
    return Status::Ok;
}

Status ROCK_CALL copySemanticHandContacts(OwnerToken owner, Hand hand, std::uint32_t maxFramesSinceContact, SemanticHandContact* outContacts, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outContacts, capacity, kMaxSemanticContacts, outCount); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        std::array<p::RockProviderSemanticHandContactV1, kMaxSemanticContacts> native{};
        const auto status = static_cast<Status>(p::runtime::apiCopySemanticHandContactsV1(
            owner, static_cast<api::Hand>(hand), maxFramesSinceContact, native.data(), capacity, outCount));
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) fill(outContacts[i], native[i]);
        return status;
    });
}
Status ROCK_CALL copyPlayerColliderDescriptors(OwnerToken owner, PlayerColliderDescriptor* outDescriptors, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outDescriptors, capacity, kMaxColliderDescriptors, outCount); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        std::array<p::RockProviderPlayerColliderDescriptorV1, kMaxColliderDescriptors> native{};
        const auto status = static_cast<Status>(p::runtime::apiCopyPlayerColliderDescriptorsV1(owner, native.data(), capacity, outCount));
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) fill(outDescriptors[i], native[i]);
        return status;
    });
}
Status ROCK_CALL copyBodyContacts(OwnerToken owner, BodyContact* outContacts, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outContacts, capacity, kMaxBodyContacts, outCount); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        std::array<p::RockProviderBodyContactV1, kMaxBodyContacts> native{};
        *outCount = p::runtime::apiGetBodyContactSnapshotV1(native.data(), capacity);
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) fill(outContacts[i], native[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL getHandCollisionAvailability(OwnerToken owner, Hand hand, HandCollisionAvailability* outState) noexcept {
    if (const auto s = checkOutput(outState); s != Status::Ok) return s;
    if (!validHand(hand)) return Status::InvalidArgument;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        p::RockProviderHandCollisionAvailabilityV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiGetHandCollisionAvailabilityV1(owner, static_cast<api::Hand>(hand), &native));
        fill(*outState, native);
        return status;
    });
}
Status ROCK_CALL queryWorldRaycast(OwnerToken owner, const WorldRaycastRequest* request, WorldRaycastResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        const auto nativeRequest = toProvider(*request);
        p::RockProviderWorldRaycastResultV1 native{};
        const auto status = static_cast<Status>(p::runtime::apiQueryWorldRaycastV1(owner, &nativeRequest, &native));
        fill(*outResult, native);
        return status;
    });
}
Status ROCK_CALL requestEjectionClearance(OwnerToken owner, const EjectionClearanceRequest* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = requireWeaponSlot(request->weaponSlot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::requestEjectionClearance(owner, *request); });
}
Status ROCK_CALL cancelEjectionClearance(OwnerToken owner, std::uint32_t referenceFormId) noexcept {
    return invoke(owner, kInterfaceId, 0, true, [&] { return p::runtime::cancelEjectionClearance(owner, referenceFormId); });
}
Status ROCK_CALL watchObjectRest(OwnerToken owner, const ObjectRestRequest* request, ObjectRestState* outState) noexcept {
    if (!outState) return Status::InvalidArgument;
    *outState = ObjectRestState::Waiting;
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return p::runtime::watchObjectRest(owner, *request, *outState); });
}
Status ROCK_CALL cancelObjectRest(OwnerToken owner, std::uint32_t referenceFormId) noexcept {
    return invoke(owner, kInterfaceId, 0, true, [&] { return p::runtime::cancelObjectRest(owner, referenceFormId); });
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
        &copySemanticHandContacts,
        &copyPlayerColliderDescriptors,
        &copyBodyContacts,
        &getHandCollisionAvailability,
        &queryWorldRaycast,
        &getEnvironment,
        &requestEjectionClearance,
        &cancelEjectionClearance,
        &watchObjectRest,
        &cancelObjectRest,
        &getSample,
        &copyEvents,
    };
    return value;
}
}

#include "Boundary.h"
#include "CommonMarshalling.h"
#include "EventBoundary.h"
#include "SnapshotReads.h"
#include "api/LooseWeaponStateService.h"
#include "api/WeaponPartCollisionService.h"
#include <ROCK/WeaponParts.h>
#include <algorithm>
#include <array>
#include <cstring>

namespace rock::api::weaponparts {
namespace {
using namespace boundary;
namespace p = provider;

// ROCK publishes only the first equipped weapon (equip index 0) until the
// runtime keeps state per slot; slot 1 is valid but not yet published.
Status checkSlot(std::uint32_t slot) noexcept { return slot < kMaxWeaponSlots ? Status::Ok : Status::InvalidArgument; }
bool published(std::uint32_t slot) noexcept { return slot == 0; }

template <class Record>
Status checkOutputs(Record* records, std::uint32_t capacity, std::uint32_t maxCapacity, std::uint32_t* outCount) noexcept {
    if (!outCount) return Status::InvalidArgument;
    *outCount = 0;
    if (capacity > maxCapacity) return Status::CapacityFull;
    if (capacity && !records) return Status::InvalidArgument;
    if constexpr (requires { records->size; })
        for (std::uint32_t i = 0; i < capacity; ++i)
            if (const auto s = checkOutput(records + i); s != Status::Ok) return s;
    return Status::Ok;
}

// A name selector resolves to its live source on the weapon generation.
template <class T> Status validateSourceSelector(T& target) {
    constexpr auto byName = static_cast<std::uint32_t>(p::RockProviderWeaponPartTargetFlagV1::MatchSourceName);
    constexpr auto byKey = static_cast<std::uint32_t>(p::RockProviderWeaponPartTargetFlagV1::MatchSourceRoot);
    if (target.flags & byName) {
        if (!std::memchr(target.sourceName, 0, sizeof(target.sourceName))) return Status::InvalidArgument;
        const auto node = p::runtime::resolveSourceName(target.weaponGenerationKey, target.sourceName);
        if (!node) return Status::Ambiguous;
        if ((target.flags & byKey) && target.sourceRoot != node) return Status::TargetInvalid;
        target.sourceRoot = node;
        target.flags = (target.flags & ~byName) | byKey;
    }
    return Status::Ok;
}

p::RockProviderWeaponPartTargetV1 toProvider(const Target& in) {
    p::RockProviderWeaponPartTargetV1 out{};
    out.flags = in.flags;
    out.grabMode = static_cast<decltype(out.grabMode)>(in.grabMode);
    out.weaponGenerationKey = in.weaponGenerationKey;
    out.sourceRoot = p::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
    out.bodyId = in.bodyId;
    out.partKind = in.partKind;
    out.reloadRole = in.reloadRole;
    out.supportRole = in.supportRole;
    out.socketRole = in.socketRole;
    out.actionRole = in.actionRole;
    out.groupId = in.groupId;
    out.priority = in.priority;
    std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
    return out;
}
p::RockProviderWeaponPartDriveTargetV1 toProvider(const DriveTarget& in) {
    p::RockProviderWeaponPartDriveTargetV1 out{};
    out.flags = in.flags;
    out.driveSpace = static_cast<decltype(out.driveSpace)>(in.driveSpace);
    out.weaponGenerationKey = in.weaponGenerationKey;
    out.sourceRoot = p::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
    out.bodyId = in.bodyId;
    out.groupId = in.groupId;
    out.priority = in.priority;
    out.leaseFrames = in.leaseFrames;
    convert(out.targetTransform, in.targetTransform);
    std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
    return out;
}
p::RockProviderWeaponPartResolutionQueryV1 toProvider(const ResolutionQuery& in) {
    p::RockProviderWeaponPartResolutionQueryV1 out{};
    out.weaponGenerationKey = in.weaponGenerationKey;
    out.bodyId = in.bodyId;
    out.partKind = in.partKind;
    out.reloadRole = in.reloadRole;
    out.supportRole = in.supportRole;
    out.socketRole = in.socketRole;
    out.actionRole = in.actionRole;
    out.sourceRoot = p::runtime::resolveSourceKey(in.weaponGenerationKey, in.sourceKey);
    std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
    return out;
}
void fill(ContactResult& out, const p::RockProviderWeaponContactResult& in) {
    out.valid = in.valid;
    out.bodyId = in.bodyId;
    out.partKind = in.partKind;
    out.reloadRole = in.reloadRole;
    out.supportRole = in.supportRole;
    out.socketRole = in.socketRole;
    out.actionRole = in.actionRole;
    out.interactionKey = p::runtime::sourceKey(in.weaponGenerationKey, in.interactionRoot);
    out.sourceKey = p::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
    out.weaponGenerationKey = in.weaponGenerationKey;
    out.probeDistanceGame = in.probeDistanceGame;
}
void fill(EvidenceDetail& out, const p::RockProviderWeaponEvidenceDetailV1& in) {
    out.bodyId = in.bodyId;
    out.partKind = in.partKind;
    out.reloadRole = in.reloadRole;
    out.supportRole = in.supportRole;
    out.socketRole = in.socketRole;
    out.actionRole = in.actionRole;
    out.fallbackGripPose = in.fallbackGripPose;
    out.pointCount = in.pointCount;
    out.interactionKey = p::runtime::sourceKey(in.weaponGenerationKey, in.interactionRoot);
    out.sourceKey = p::runtime::sourceKey(in.weaponGenerationKey, in.sourceRoot);
    out.weaponGenerationKey = in.weaponGenerationKey;
    convert(out.localBoundsGame, in.localBoundsGame);
    out.omodFormId = in.omodFormId;
    out.attachPointFormId = in.attachPointFormId;
    out.classificationSource = in.classificationSource;
    std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
}
void fill(PartPose& out, const p::RockProviderWeaponPartPoseV1& in) {
    out.frameIndex = in.frameIndex;
    out.weaponGenerationKey = in.weaponGenerationKey;
    out.sourceKey = p::runtime::sourceKeyForBody(in.weaponGenerationKey, in.bodyId);
    out.bodyId = in.bodyId;
    out.partKind = in.partKind;
    out.omodFormId = in.omodFormId;
    out.attachPointFormId = in.attachPointFormId;
    out.flags = in.flags;
    out.actionRole = in.actionRole;
    convert(out.sourceParentLocal, in.sourceParentLocal);
    convert(out.weaponRootLocal, in.weaponRootLocal);
    std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
}
void fill(DriveResult& out, const p::RockProviderWeaponPartDriveApplicationResultV1& in) {
    out.frameIndex = in.frameIndex;
    out.owner = in.ownerToken;
    out.weaponGenerationKey = in.weaponGenerationKey;
    out.bodyId = in.bodyId;
    out.groupId = in.groupId;
    out.priority = in.priority;
    out.result = static_cast<DriveApplication>(in.result);
    convert(out.appliedSourceParentLocal, in.appliedSourceParentLocal);
    std::copy_n(in.sourceName, std::size(out.sourceName), out.sourceName);
}

Status ROCK_CALL queryContact(OwnerToken owner, std::uint32_t slot, const ContactQuery* query, ContactResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    outResult->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        p::RockProviderWeaponContactQuery native{};
        native.pointGame[0] = query->pointGame.x;
        native.pointGame[1] = query->pointGame.y;
        native.pointGame[2] = query->pointGame.z;
        native.radiusGame = query->radiusGame;
        native.flags = query->flags;
        p::RockProviderWeaponContactResult result{};
        const bool ok = p::runtime::apiQueryWeaponContactAtPoint(&native, &result);
        fill(*outResult, result);
        return ok ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL copyEvidenceDetails(OwnerToken owner, std::uint32_t slot, EvidenceDetail* outDetails, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outDetails, capacity, kMaxEvidenceDetails, outCount); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::RockProviderWeaponEvidenceDetailV1, kMaxEvidenceDetails> native{};
        *outCount = p::runtime::apiCopyWeaponEvidenceDetailsV1(native.data(), kMaxEvidenceDetails);
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) fill(outDetails[i], native[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL copyEvidencePoints(OwnerToken owner, std::uint32_t slot, std::uint32_t bodyId, Point3* outPoints, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outPoints, capacity, kMaxEvidencePoints, outCount); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        std::array<p::RockProviderPoint3, kMaxEvidencePoints> native{};
        *outCount = p::runtime::apiCopyWeaponEvidenceDetailPointsV1(bodyId, native.data(), kMaxEvidencePoints);
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) convert(outPoints[i], native[i]);
        return Status::Ok;
    });
}
Status ROCK_CALL setPartTargets(OwnerToken owner, std::uint32_t slot, const Target* targets, std::uint32_t count) noexcept {
    if (count > kMaxTargets) return Status::CapacityFull;
    if (count && !targets) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < count; ++i)
        if (const auto s = checkInput(targets + i); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::RockProviderWeaponPartTargetV1, kMaxTargets> native{};
        for (std::uint32_t i = 0; i < count; ++i) {
            native[i] = toProvider(targets[i]);
            if (const auto s = validateSourceSelector(native[i]); s != Status::Ok) return s;
            if (targets[i].sourceKey && !native[i].sourceRoot) return Status::GenerationMismatch;
        }
        return static_cast<Status>(p::runtime::apiSetWeaponPartTargetsV1(owner, native.data(), count));
    });
}
Status ROCK_CALL clearPartTargets(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return published(slot) ? static_cast<Status>(p::runtime::apiClearWeaponPartTargetsV1(owner)) : Status::Ok;
    });
}
Status ROCK_CALL setDriveTargets(OwnerToken owner, std::uint32_t slot, const DriveTarget* targets, std::uint32_t count) noexcept {
    if (count > kMaxDrives) return Status::CapacityFull;
    if (count && !targets) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < count; ++i)
        if (const auto s = checkInput(targets + i); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::RockProviderWeaponPartDriveTargetV1, kMaxDrives> native{};
        for (std::uint32_t i = 0; i < count; ++i) {
            native[i] = toProvider(targets[i]);
            if (const auto s = validateSourceSelector(native[i]); s != Status::Ok) return s;
            if (targets[i].sourceKey && !native[i].sourceRoot) return Status::GenerationMismatch;
        }
        return static_cast<Status>(p::runtime::apiSetWeaponPartDriveTargetsV1(owner, native.data(), count));
    });
}
Status ROCK_CALL clearDriveTargets(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return published(slot) ? static_cast<Status>(p::runtime::apiClearWeaponPartDriveTargetsV1(owner)) : Status::Ok;
    });
}
Status ROCK_CALL queryTargetResolution(OwnerToken owner, std::uint32_t slot, const ResolutionQuery* query, ResolutionResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        const auto native = toProvider(*query);
        if (query->sourceKey && !native.sourceRoot) return Status::GenerationMismatch;
        p::RockProviderWeaponPartResolutionResultV1 result{};
        const auto status = static_cast<Status>(p::runtime::apiQueryWeaponPartTargetResolutionV1(owner, &native, &result));
        outResult->winningOwner = result.winningOwnerToken;
        outResult->weaponGenerationKey = result.weaponGenerationKey;
        outResult->frameIndex = result.frameIndex;
        outResult->whitelistActive = result.whitelistActive;
        outResult->matched = result.matched;
        outResult->grabMode = static_cast<GrabMode>(result.grabMode);
        outResult->groupId = result.groupId;
        outResult->priority = result.priority;
        return status;
    });
}
Status ROCK_CALL copyPartPoses(OwnerToken owner, std::uint32_t slot, PartPose* outPoses, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outPoses, capacity, kMaxPoses, outCount); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::RockProviderWeaponPartPoseV1, kMaxPoses> native{};
        const auto status = static_cast<Status>(p::runtime::apiCopyWeaponPartPoseSnapshotV1(owner, native.data(), kMaxPoses, outCount));
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) fill(outPoses[i], native[i]);
        return status;
    });
}
Status ROCK_CALL copyDriveResults(OwnerToken owner, std::uint32_t slot, DriveResult* outResults, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outResults, capacity, kMaxDrives, outCount); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::RockProviderWeaponPartDriveApplicationResultV1, kMaxDrives> native{};
        const auto status = static_cast<Status>(p::runtime::apiCopyWeaponPartDriveApplicationResultsV1(owner, native.data(), kMaxDrives, outCount));
        for (std::uint32_t i = 0; i < (std::min)(capacity, *outCount); ++i) fill(outResults[i], native[i]);
        return status;
    });
}
Status ROCK_CALL copySources(OwnerToken owner, std::uint32_t slot, std::uint64_t generation, Source* outSources, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outSources, capacity, kMaxSources, outCount); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::WeaponSourceRecord, 64> records{};
        std::uint32_t chunk = 0;
        auto status = p::runtime::copySources(generation, 0, records.data(), 0, chunk, *outCount);
        if (status != Status::Ok) return status;
        const auto requested = (std::min)(capacity, *outCount);
        std::uint32_t copied = 0;
        while (copied < requested) {
            std::uint32_t total = 0;
            status = p::runtime::copySources(generation, copied, records.data(), (std::min<std::uint32_t>)(64, requested - copied), chunk, total);
            if (status != Status::Ok || !chunk) {
                for (std::uint32_t i = 0; i < copied; ++i) outSources[i] = {};
                *outCount = 0;
                return status == Status::Ok ? Status::GenerationMismatch : status;
            }
            for (std::uint32_t i = 0; i < chunk; ++i) {
                auto& out = outSources[copied + i];
                out.bodyId = records[i].bodyId;
                out.weaponGenerationKey = records[i].weaponGenerationKey;
                out.sourceKey = records[i].sourceKey;
                out.parentKey = records[i].parentKey;
                out.sourceParentLocal = records[i].sourceParentLocal;
                std::copy_n(records[i].name, std::size(out.name), out.name);
            }
            copied += chunk;
        }
        return Status::Ok;
    });
}
Status ROCK_CALL querySourcePose(OwnerToken owner, std::uint32_t slot, std::uint64_t generation, std::uint64_t key, SourcePose* outPose) noexcept {
    if (const auto s = checkOutput(outPose); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    outPose->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::WeaponSourcePose pose{};
        const auto status = p::runtime::querySourcePose(generation, key, pose);
        if (status != Status::Ok) return status;
        outPose->valid = 1;
        outPose->sample = p::runtime::sample();
        outPose->weaponGenerationKey = pose.weaponGenerationKey;
        outPose->sourceKey = pose.sourceKey;
        outPose->sourceParentLocal = pose.sourceParentLocal;
        outPose->weaponRootLocal = pose.weaponRootLocal;
        outPose->world = pose.world;
        return Status::Ok;
    });
}
Status ROCK_CALL querySourcePath(OwnerToken owner, std::uint32_t slot, std::uint64_t generation, std::uint64_t key, std::uint64_t* outParentKey, std::uint32_t* outChildIndex) noexcept {
    if (!outParentKey || !outChildIndex) return Status::InvalidArgument;
    *outParentKey = 0;
    *outChildIndex = 0;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        return p::runtime::querySourcePath(generation, key, *outParentKey, *outChildIndex);
    });
}
Status ROCK_CALL setCollisionSuppression(OwnerToken owner, std::uint32_t slot, const CollisionSuppression* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return published(slot) ? weapon_part_collision::set(owner, *request) : Status::TargetUnavailable;
    });
}
Status ROCK_CALL clearCollisionSuppression(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return published(slot) ? weapon_part_collision::clear(owner) : Status::Ok; });
}
Status ROCK_CALL setGrabPoints(OwnerToken owner, std::uint32_t slot, const GrabPoint* points, std::uint32_t count) noexcept {
    if (!points || !count || count > kMaxGrabPoints) return Status::InvalidArgument;
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        if (!published(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        return p::runtime::setWeaponPartGrabPoints(owner, { points, count });
    });
}
Status ROCK_CALL clearGrabPoints(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return published(slot) ? p::runtime::clearWeaponPartGrabPoints(owner) : Status::Ok; });
}
Status ROCK_CALL setItemState(OwnerToken owner, const ItemKey* key, const ItemState* state) noexcept {
    return invoke(owner, kInterfaceId, 2, false, [&] { return key && state ? loose_weapon_state::set(owner, *key, *state) : Status::InvalidArgument; });
}
Status ROCK_CALL moveItemState(OwnerToken owner, const ItemKey* from, const ItemKey* to) noexcept {
    return invoke(owner, kInterfaceId, 2, false, [&] { return from && to ? loose_weapon_state::move(owner, *from, *to) : Status::InvalidArgument; });
}
Status ROCK_CALL clearItemStates(OwnerToken owner) noexcept {
    return invoke(owner, kInterfaceId, 2, false, [&] { return loose_weapon_state::clear(owner); });
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
        &queryContact,
        &copyEvidenceDetails,
        &copyEvidencePoints,
        &setPartTargets,
        &clearPartTargets,
        &setDriveTargets,
        &clearDriveTargets,
        &getGripState,
        &queryTargetResolution,
        &copyPartPoses,
        &copyDriveResults,
        &copySources,
        &querySourcePose,
        &querySourcePath,
        &setCollisionSuppression,
        &clearCollisionSuppression,
        &setGrabPoints,
        &clearGrabPoints,
        &setItemState,
        &moveItemState,
        &clearItemStates,
        &getSample,
        &copyEvents,
    };
    return value;
}
}

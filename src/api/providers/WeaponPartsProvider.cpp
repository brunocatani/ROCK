#include "Boundary.h"
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

// Resolves a request's source selector to the live node of its weapon
// generation: a name to its unique source, a key to its node.
template <class Request> Status resolveSource(Request& request) {
    constexpr auto byName = static_cast<std::uint32_t>(TargetFlag::MatchSourceName);
    constexpr auto byKey = static_cast<std::uint32_t>(TargetFlag::MatchSourceKey);
    if (request.sourceKey) {
        request.sourceRoot = p::runtime::resolveSourceKey(request.weaponGenerationKey, request.sourceKey);
        if (!request.sourceRoot) return Status::GenerationMismatch;
    }
    if (request.flags & byName) {
        if (!std::memchr(request.sourceName, 0, sizeof(request.sourceName))) return Status::InvalidArgument;
        const auto node = p::runtime::resolveSourceName(request.weaponGenerationKey, request.sourceName);
        if (!node) return Status::Ambiguous;
        if ((request.flags & byKey) && request.sourceRoot != node) return Status::TargetInvalid;
        request.sourceRoot = node;
        request.flags = (request.flags & ~byName) | byKey;
    }
    return Status::Ok;
}

Status ROCK_CALL queryContact(OwnerToken owner, std::uint32_t slot, const ContactQuery* query, ContactResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outResult->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        const bool ok = p::runtime::queryWeaponContact(*query, *outResult);
        outResult->weaponSlot = slot;
        return ok ? Status::Ok : Status::NotReady;
    });
}
Status ROCK_CALL copyEvidenceDetails(OwnerToken owner, std::uint32_t slot, EvidenceDetail* outDetails, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outDetails, capacity, kMaxEvidenceDetails, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<EvidenceDetail, kMaxEvidenceDetails> all{};
        *outCount = p::runtime::copyWeaponEvidenceDetails(all.data(), kMaxEvidenceDetails);
        std::copy_n(all.begin(), (std::min)(capacity, *outCount), outDetails);
        return Status::Ok;
    });
}
Status ROCK_CALL copyEvidencePoints(OwnerToken owner, std::uint32_t slot, std::uint32_t bodyId, Point3* outPoints, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outPoints, capacity, kMaxEvidencePoints, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        std::array<Point3, kMaxEvidencePoints> all{};
        *outCount = p::runtime::copyWeaponEvidencePoints(bodyId, all.data(), kMaxEvidencePoints);
        std::copy_n(all.begin(), (std::min)(capacity, *outCount), outPoints);
        return Status::Ok;
    });
}
Status ROCK_CALL setPartTargets(OwnerToken owner, std::uint32_t slot, const Target* targets, std::uint32_t count) noexcept {
    if (count > kMaxTargets) return Status::CapacityFull;
    if (count && !targets) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < count; ++i)
        if (const auto s = checkInput(targets + i); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::PartTarget, kMaxTargets> resolved{};
        for (std::uint32_t i = 0; i < count; ++i) {
            static_cast<Target&>(resolved[i]) = targets[i];
            if (const auto s = resolveSource(resolved[i]); s != Status::Ok) return s;
        }
        return p::runtime::setWeaponPartTargets(owner, std::span(resolved).first(count));
    });
}
Status ROCK_CALL clearPartTargets(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return weaponSlotPublished(slot) ? p::runtime::clearWeaponPartTargets(owner) : Status::Ok;
    });
}
Status ROCK_CALL setDriveTargets(OwnerToken owner, std::uint32_t slot, const DriveTarget* targets, std::uint32_t count) noexcept {
    if (count > kMaxDrives) return Status::CapacityFull;
    if (count && !targets) return Status::InvalidArgument;
    for (std::uint32_t i = 0; i < count; ++i)
        if (const auto s = checkInput(targets + i); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<p::PartDrive, kMaxDrives> resolved{};
        for (std::uint32_t i = 0; i < count; ++i) {
            static_cast<DriveTarget&>(resolved[i]) = targets[i];
            if (const auto s = resolveSource(resolved[i]); s != Status::Ok) return s;
        }
        return p::runtime::setWeaponPartDrives(owner, std::span(resolved).first(count));
    });
}
Status ROCK_CALL clearDriveTargets(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return weaponSlotPublished(slot) ? p::runtime::clearWeaponPartDrives(owner) : Status::Ok;
    });
}
Status ROCK_CALL queryTargetResolution(OwnerToken owner, std::uint32_t slot, const ResolutionQuery* query, ResolutionResult* outResult) noexcept {
    if (const auto s = checkOutput(outResult); s != Status::Ok) return s;
    if (const auto s = checkInput(query); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        provider::PartQuery part{};
        part.weaponGenerationKey = query->weaponGenerationKey;
        part.bodyId = query->bodyId;
        part.partKind = query->partKind;
        part.reloadRole = query->reloadRole;
        part.supportRole = query->supportRole;
        part.socketRole = query->socketRole;
        part.actionRole = query->actionRole;
        part.sourceRoot = p::runtime::resolveSourceKey(query->weaponGenerationKey, query->sourceKey);
        if (query->sourceKey && !part.sourceRoot) return Status::GenerationMismatch;
        std::copy_n(query->sourceName, std::size(part.sourceName) - 1, part.sourceName);
        return p::runtime::queryWeaponPartTargetResolution(part, *outResult);
    });
}
Status ROCK_CALL copyPartPoses(OwnerToken owner, std::uint32_t slot, PartPose* outPoses, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outPoses, capacity, kMaxPoses, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        std::array<PartPose, kMaxPoses> all{};
        const auto status = p::runtime::copyWeaponPartPoses(all.data(), kMaxPoses, *outCount);
        std::copy_n(all.begin(), (std::min)(capacity, *outCount), outPoses);
        return status;
    });
}
Status ROCK_CALL copyDriveResults(OwnerToken owner, std::uint32_t slot, DriveResult* outResults, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outResults, capacity, kMaxDrives, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        std::array<DriveResult, kMaxDrives> all{};
        const auto status = p::runtime::copyWeaponPartDriveResults(owner, all.data(), kMaxDrives, *outCount);
        std::copy_n(all.begin(), (std::min)(capacity, *outCount), outResults);
        return status;
    });
}
Status ROCK_CALL copySources(OwnerToken owner, std::uint32_t slot, std::uint64_t generation, Source* outSources, std::uint32_t capacity, std::uint32_t* outCount) noexcept {
    if (const auto s = checkOutputs(outSources, capacity, kMaxSources, outCount); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
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
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    outPose->weaponSlot = slot;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
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
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 1, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        return p::runtime::querySourcePath(generation, key, *outParentKey, *outChildIndex);
    });
}
Status ROCK_CALL setCollisionSuppression(OwnerToken owner, std::uint32_t slot, const CollisionSuppression* request) noexcept {
    if (const auto s = checkInput(request); s != Status::Ok) return s;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        return weaponSlotPublished(slot) ? weapon_part_collision::set(owner, *request) : Status::TargetUnavailable;
    });
}
Status ROCK_CALL clearCollisionSuppression(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return weaponSlotPublished(slot) ? weapon_part_collision::clear(owner) : Status::Ok; });
}
Status ROCK_CALL setGrabPoints(OwnerToken owner, std::uint32_t slot, const GrabPoint* points, std::uint32_t count) noexcept {
    if (!points || !count || count > kMaxGrabPoints) return Status::InvalidArgument;
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] {
        if (!weaponSlotPublished(slot)) return Status::TargetUnavailable;
        p::runtime::refreshSources();
        return p::runtime::setWeaponPartGrabPoints(owner, { points, count });
    });
}
Status ROCK_CALL clearGrabPoints(OwnerToken owner, std::uint32_t slot) noexcept {
    if (const auto s = checkWeaponSlot(slot); s != Status::Ok) return s;
    return invoke(owner, kInterfaceId, 2, true, [&] { return weaponSlotPublished(slot) ? p::runtime::clearWeaponPartGrabPoints(owner) : Status::Ok; });
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
    };
    return value;
}
}

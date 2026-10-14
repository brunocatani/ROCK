#include "api/providers/Boundary.h"
#include "api/LooseWeaponStateService.h"
#include "api/WeaponPartCollisionService.h"
#include <ROCK/WeaponParts.h>
#include <cassert>
#include <cstring>

namespace rock::api::weaponparts { const Api& table() noexcept; }

namespace {
    using namespace rock;
    constexpr api::OwnerToken testOwner = 42;
    std::uint32_t permissions = 3;
    std::uint32_t evidenceCount = 5;
    unsigned targetWrites = 0, grabPointWrites = 0, suppressionWrites = 0, itemWrites = 0;
    std::uint32_t lastTargetCount = 0;
}

// Runtime boundary fixture: the published WeaponParts table is the code under test.
namespace rock::provider::runtime {
    api::Status authorize(std::uint64_t owner, api::InterfaceId family, std::uint32_t permission, bool, OwnerAccess) {
        if (owner != testOwner) return api::Status::OwnerNotRegistered;
        if (family != api::InterfaceId::WeaponParts || (permission & ~permissions)) return api::Status::PermissionDenied;
        return api::Status::Ok;
    }
    api::Sample sample() { return { 20, 30, 1, 2, 3, 4 }; }
    void reportBoundaryFailure(std::uint64_t, api::InterfaceId) noexcept { assert(false); }
    void refreshSources() {}
    // Source keys map to fake node values key+1000 on generation 456.
    std::uintptr_t resolveSourceKey(std::uint64_t generation, std::uint64_t key) { return generation == 456 && key ? key + 1000 : 0; }
    std::uintptr_t resolveSourceName(std::uint64_t generation, const char* name) { return generation == 456 && !std::strcmp(name, "Bolt") ? 1077 : 0; }
    bool queryWeaponContact(const api::weaponparts::ContactQuery& query, api::weaponparts::ContactResult& out) {
        assert(query.pointGame.z == 3 && query.radiusGame == 4);
        out.valid = 1; out.bodyId = 9; out.weaponGenerationKey = 456; out.sourceKey = 11; out.interactionKey = 12;
        return true;
    }
    std::uint32_t copyWeaponEvidenceDetails(api::weaponparts::EvidenceDetail* out, std::uint32_t capacity) {
        const auto copied = (std::min)(capacity, evidenceCount);
        for (std::uint32_t i = 0; i < copied; ++i) { out[i].bodyId = 10 + i; out[i].weaponGenerationKey = 456; out[i].sourceKey = 20 + i; }
        return copied;
    }
    std::uint32_t copyWeaponEvidencePoints(std::uint32_t, api::Point3* out, std::uint32_t capacity) {
        for (std::uint32_t i = 0; i < (std::min)(capacity, 3u); ++i) out[i].x = static_cast<float>(i);
        return (std::min)(capacity, 3u);
    }
    api::Status setWeaponPartTargets(std::uint64_t, std::span<const PartTarget> targets) {
        ++targetWrites; lastTargetCount = static_cast<std::uint32_t>(targets.size());
        // The name selector arrives resolved to its node, selected by key.
        if (!targets.empty()) assert(targets[0].sourceRoot == 1077 && targets[0].groupId == 7 &&
            targets[0].flags == static_cast<std::uint32_t>(api::weaponparts::TargetFlag::MatchSourceKey));
        return api::Status::Ok;
    }
    api::Status clearWeaponPartTargets(std::uint64_t) { ++targetWrites; return api::Status::Ok; }
    api::Status setWeaponPartDrives(std::uint64_t, std::span<const PartDrive>) { return api::Status::Ok; }
    api::Status clearWeaponPartDrives(std::uint64_t) { return api::Status::Ok; }
    api::Status queryWeaponPartTargetResolution(const provider::PartQuery& query, api::weaponparts::ResolutionResult& out) {
        assert(query.sourceRoot == 1005);
        out.matched = 1; out.winningOwner = testOwner; out.grabMode = api::weaponparts::GrabMode::AttachOnly; return api::Status::Ok;
    }
    api::Status copyWeaponPartPoses(api::weaponparts::PartPose*, std::uint32_t, std::uint32_t& count) { count = 0; return api::Status::Ok; }
    api::Status copyWeaponPartDriveResults(std::uint64_t, api::weaponparts::DriveResult*, std::uint32_t, std::uint32_t& count) { count = 0; return api::Status::Ok; }
    api::Status copySources(std::uint64_t, std::uint32_t, WeaponSourceRecord*, std::uint32_t, std::uint32_t& chunk, std::uint32_t& total) { chunk = 0; total = 0; return api::Status::Ok; }
    api::Status querySourcePose(std::uint64_t, std::uint64_t, WeaponSourcePose&) { return api::Status::TargetUnavailable; }
    api::Status querySourcePath(std::uint64_t, std::uint64_t, std::uint64_t&, std::uint32_t&) { return api::Status::TargetUnavailable; }
    api::Status setWeaponPartGrabPoints(api::OwnerToken, std::span<const api::weaponparts::GrabPoint> points) { ++grabPointWrites; return points.size() == 2 ? api::Status::Ok : api::Status::InvalidArgument; }
    api::Status clearWeaponPartGrabPoints(api::OwnerToken) { return api::Status::Ok; }
}
namespace rock::weapon_part_collision {
    api::Status set(api::OwnerToken, const api::weaponparts::CollisionSuppression&) { ++suppressionWrites; return api::Status::Ok; }
    api::Status clear(api::OwnerToken) { return api::Status::Ok; }
}
namespace rock::loose_weapon_state {
    api::Status set(api::OwnerToken, const contract::ItemKey&, const contract::ItemState&) { ++itemWrites; return api::Status::Ok; }
    api::Status move(api::OwnerToken, const contract::ItemKey&, const contract::ItemKey&) { return api::Status::Ok; }
    api::Status clear(api::OwnerToken) { return api::Status::Ok; }
}
namespace rock::api::weaponparts {
    Status ROCK_CALL getGripState(OwnerToken, Hand, GripState*) noexcept { return Status::Ok; }
}

int main() {
    using namespace rock::api;
    using namespace rock::api::weaponparts;
    const auto& api = table();

    // Slots: only equip index 0 is published; slot 1 is valid but unavailable.
    ContactQuery query;
    query.pointGame = { 1, 2, 3 }; query.radiusGame = 4;
    ContactResult contact;
    assert(api.queryContact(testOwner, kMaxWeaponSlots, &query, &contact) == Status::InvalidArgument);
    assert(api.queryContact(testOwner, 1, &query, &contact) == Status::TargetUnavailable && contact.weaponSlot == 1 && contact.valid == 0);
    assert(api.queryContact(testOwner, 0, &query, &contact) == Status::Ok && contact.weaponSlot == 0 && contact.valid == 1 &&
           contact.sourceKey == 11 && contact.interactionKey == 12 && contact.bodyId == 9);
    query.version = 2;
    assert(api.queryContact(testOwner, 0, &query, &contact) == Status::UnsupportedVersion);
    query.version = 1;

    // Copy calls report the total and copy what fits; capacity 0 just counts.
    EvidenceDetail details[2];
    std::uint32_t count = 99;
    assert(api.copyEvidenceDetails(testOwner, 0, details, 2, &count) == Status::Ok && count == 5 && details[1].bodyId == 11 && details[1].sourceKey == 21);
    assert(api.copyEvidenceDetails(testOwner, 0, nullptr, 0, &count) == Status::Ok && count == 5);
    assert(api.copyEvidenceDetails(testOwner, 0, details, kMaxEvidenceDetails + 1, &count) == Status::CapacityFull && count == 0);
    assert(api.copyEvidenceDetails(testOwner, 1, details, 2, &count) == Status::TargetUnavailable);
    Point3 points[1];
    assert(api.copyEvidencePoints(testOwner, 0, 10, points, 1, &count) == Status::Ok && count == 3);

    // Part targets: a name selector resolves to its source; clearing an
    // unpublished slot is a no-op that never touches slot 0's targets.
    Target target;
    target.flags = static_cast<std::uint32_t>(TargetFlag::MatchSourceName); target.weaponGenerationKey = 456; target.groupId = 7;
    std::strcpy(target.sourceName, "Bolt");
    assert(api.setPartTargets(testOwner, 0, &target, 1) == Status::Ok && targetWrites == 1 && lastTargetCount == 1);
    assert(api.setPartTargets(testOwner, 1, &target, 1) == Status::TargetUnavailable && targetWrites == 1);
    assert(api.clearPartTargets(testOwner, 1) == Status::Ok && targetWrites == 1);
    assert(api.clearPartTargets(testOwner, 0) == Status::Ok && targetWrites == 2);
    std::strcpy(target.sourceName, "Missing");
    assert(api.setPartTargets(testOwner, 0, &target, 1) == Status::Ambiguous);
    assert(api.setPartTargets(testOwner, 0, &target, kMaxTargets + 1) == Status::CapacityFull);

    ResolutionQuery resolution;
    resolution.weaponGenerationKey = 456; resolution.sourceKey = 5;
    ResolutionResult resolved;
    assert(api.queryTargetResolution(testOwner, 0, &resolution, &resolved) == Status::Ok && resolved.matched == 1 &&
           resolved.winningOwner == testOwner && resolved.grabMode == GrabMode::AttachOnly);
    resolution.weaponGenerationKey = 455; // A key of another generation never resolves.
    assert(api.queryTargetResolution(testOwner, 0, &resolution, &resolved) == Status::GenerationMismatch);

    // One grab-point form; the count is checked before ROCK sees it.
    GrabPoint grab[2];
    assert(api.setGrabPoints(testOwner, 0, grab, 2) == Status::Ok && grabPointWrites == 1);
    assert(api.setGrabPoints(testOwner, 0, grab, 0) == Status::InvalidArgument && grabPointWrites == 1);
    assert(api.setGrabPoints(testOwner, 1, grab, 2) == Status::TargetUnavailable && grabPointWrites == 1);

    CollisionSuppression suppression;
    assert(api.setCollisionSuppression(testOwner, 0, &suppression) == Status::Ok && suppressionWrites == 1);
    suppression.size = 8;
    assert(api.setCollisionSuppression(testOwner, 0, &suppression) == Status::InvalidSize && suppressionWrites == 1);

    // Item state is item-keyed, not slot-keyed.
    ItemKey key{ 123, 0x14, 7 };
    ItemState state;
    assert(api.setItemState(testOwner, &key, &state) == Status::Ok && itemWrites == 1);
    assert(api.setItemState(testOwner, nullptr, &state) == Status::InvalidArgument);

    // Permissions are WeaponParts' own.
    permissions = 1;
    target.flags = 0;
    assert(api.setPartTargets(testOwner, 0, &target, 1) == Status::PermissionDenied);
    assert(api.setItemState(testOwner, &key, &state) == Status::PermissionDenied && itemWrites == 1);
    permissions = 0;
    assert(api.queryContact(testOwner, 0, &query, &contact) == Status::PermissionDenied && contact.valid == 0);
}

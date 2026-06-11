#pragma once

#include <algorithm>
#include <cstdint>
#include <unordered_set>
#include <vector>

#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"

namespace rock::mechanical_connected_body_set
{
    inline constexpr std::uint32_t kInvalidBodyId = object_physics_body_set::INVALID_BODY_ID;

    enum class ScopeKind : std::uint8_t
    {
        SingleDynamic,
        RefTreeConnectedDynamic,
        ArticulatedMechanical,
        FixedAttached,
        IncompleteDiscovery,
    };

    struct MechanicalScope
    {
        std::uint32_t primaryBodyId = kInvalidBodyId;
        ScopeKind kind = ScopeKind::IncompleteDiscovery;
        std::vector<std::uint32_t> committedBodyIds;
        held_object_drive_policy::HeldBodySetDriveDecision driveDecision{};
        std::uint32_t acceptedBodyCount = 0;
        std::uint32_t uniqueMotionCount = 0;
        std::uint32_t rejectedFixedOrNonDynamicCount = 0;
        std::uint32_t scanFailureCount = 0;
        std::uint32_t invalidPhysicsSystemCount = 0;
        bool targetPrefersMechanical = false;
        bool strictPocketAuthorityRelaxed = false;
        bool incompleteDiscovery = false;
        const char* reason = "not-built";
    };

    [[nodiscard]] inline const char* scopeKindName(ScopeKind kind) noexcept
    {
        switch (kind) {
        case ScopeKind::SingleDynamic:
            return "singleDynamic";
        case ScopeKind::RefTreeConnectedDynamic:
            return "refTreeConnectedDynamic";
        case ScopeKind::ArticulatedMechanical:
            return "articulatedMechanical";
        case ScopeKind::FixedAttached:
            return "fixedAttached";
        case ScopeKind::IncompleteDiscovery:
        default:
            return "incompleteDiscovery";
        }
    }

    [[nodiscard]] inline bool containsBody(const std::vector<std::uint32_t>& bodyIds, std::uint32_t bodyId) noexcept
    {
        return std::find(bodyIds.begin(), bodyIds.end(), bodyId) != bodyIds.end();
    }

    [[nodiscard]] inline std::uint32_t rejectCount(
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        physics_body_classifier::BodyRejectReason reason) noexcept
    {
        const auto index = static_cast<std::size_t>(reason);
        return index < bodySet.diagnostics.rejectCounts.size() ? bodySet.diagnostics.rejectCounts[index] : 0u;
    }

    [[nodiscard]] inline std::uint32_t acceptedRecordCount(const object_physics_body_set::ObjectPhysicsBodySet& bodySet) noexcept
    {
        return static_cast<std::uint32_t>(std::count_if(bodySet.records.begin(), bodySet.records.end(), [](const object_physics_body_set::ObjectPhysicsBodyRecord& record) {
            return record.accepted;
        }));
    }

    [[nodiscard]] inline std::uint32_t uniqueAcceptedMotionCount(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
    {
        std::unordered_set<std::uint32_t> seenMotions;
        for (const auto& record : bodySet.records) {
            if (record.accepted) {
                seenMotions.insert(record.motionId);
            }
        }
        return static_cast<std::uint32_t>(seenMotions.size());
    }

    [[nodiscard]] inline std::vector<std::uint32_t> acceptedBodyIds(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
    {
        std::vector<std::uint32_t> result;
        result.reserve(bodySet.records.size());
        for (const auto& record : bodySet.records) {
            if (record.accepted) {
                result.push_back(record.bodyId);
            }
        }
        return result;
    }

    [[nodiscard]] inline MechanicalScope buildFromPreparedBodySet(
        const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
        const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet,
        std::uint32_t primaryBodyId,
        grab_target::Kind targetKind,
        bool lifecycleIncompleteNativeScan)
    {
        MechanicalScope result{};
        result.primaryBodyId = primaryBodyId;
        result.targetPrefersMechanical = grab_target::prefersMechanicalScope(targetKind);
        result.strictPocketAuthorityRelaxed = grab_target::relaxesStrictPocketAuthorityForMechanicalGrab(targetKind);
        result.acceptedBodyCount = acceptedRecordCount(preparedBodySet);
        result.uniqueMotionCount = uniqueAcceptedMotionCount(preparedBodySet);
        result.rejectedFixedOrNonDynamicCount =
            rejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::StaticMotion) +
            rejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep);
        result.scanFailureCount = beforePrepBodySet.diagnostics.scanFailures + preparedBodySet.diagnostics.scanFailures;
        result.invalidPhysicsSystemCount = beforePrepBodySet.diagnostics.invalidPhysicsSystems + preparedBodySet.diagnostics.invalidPhysicsSystems;
        result.incompleteDiscovery = lifecycleIncompleteNativeScan || result.scanFailureCount > 0 || result.invalidPhysicsSystemCount > 0;

        result.committedBodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, acceptedBodyIds(preparedBodySet));
        if (result.committedBodyIds.empty() && primaryBodyId != kInvalidBodyId) {
            result.committedBodyIds.push_back(primaryBodyId);
        }

        result.driveDecision = held_object_drive_policy::evaluateHeldBodySetDrive(held_object_drive_policy::HeldBodySetDriveInput{
            .acceptedBodyCount = result.acceptedBodyCount,
            .uniqueMotionCount = result.uniqueMotionCount,
            .rejectedFixedOrNonDynamicCount = result.rejectedFixedOrNonDynamicCount,
            .scanFailureCount = result.scanFailureCount,
            .invalidPhysicsSystemCount = result.invalidPhysicsSystemCount,
            .incompleteNativeScan = result.incompleteDiscovery,
        });

        if (result.driveDecision.mode == held_object_drive_policy::HeldBodySetDriveMode::IncompleteNativeScan) {
            result.kind = ScopeKind::IncompleteDiscovery;
            result.reason = result.driveDecision.reason;
        } else if (result.driveDecision.mode == held_object_drive_policy::HeldBodySetDriveMode::FixedAttached) {
            result.kind = ScopeKind::FixedAttached;
            result.reason = result.driveDecision.reason;
        } else if (result.targetPrefersMechanical && (targetKind == grab_target::Kind::DeadActorBody || result.acceptedBodyCount > 1 || result.uniqueMotionCount > 1)) {
            result.kind = ScopeKind::ArticulatedMechanical;
            result.reason = targetKind == grab_target::Kind::DeadActorBody ? "dead-actor-mechanical-scope" : "mechanical-target-body-set";
        } else if (result.driveDecision.mode == held_object_drive_policy::HeldBodySetDriveMode::ComplexArticulated) {
            result.kind = ScopeKind::ArticulatedMechanical;
            result.reason = result.driveDecision.reason;
        } else if (result.driveDecision.mode == held_object_drive_policy::HeldBodySetDriveMode::ConnectedDynamic) {
            result.kind = ScopeKind::RefTreeConnectedDynamic;
            result.reason = result.driveDecision.reason;
        } else {
            result.kind = ScopeKind::SingleDynamic;
            result.reason = result.driveDecision.reason;
        }

        return result;
    }
}

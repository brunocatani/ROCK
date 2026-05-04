#pragma once

/*
 * Active grab lifecycle is captured as a full object-body snapshot because
 * multi-body props can contain Fallout-owned bodies with different original
 * motion states. HIGGS preserves the held relationship and restores engine-owned
 * state at release boundaries; ROCK needs the same ownership boundary while
 * keeping ordinary loose dynamic props dynamic. This policy layer keeps those
 * release decisions pure, so runtime code can apply them without re-deriving
 * intent from whichever body happened to become the primary constraint target.
 */

#include "ObjectPhysicsBodySet.h"
#include "PhysicsBodyClassifier.h"

#include <algorithm>
#include <cstdint>
#include <vector>

namespace frik::rock::active_grab_body_lifecycle
{
    inline constexpr std::uint32_t kInvalidBodyId = object_physics_body_set::INVALID_BODY_ID;
    inline constexpr std::uint16_t kDynamicMotionPropertiesId = 1;

    enum class MotionRole : std::uint8_t
    {
        Unknown,
        LooseDynamic,
        SystemOwnedNonDynamic,
    };

    enum class BodyRestorePolicy : std::uint8_t
    {
        ProtectComplexSystemOwned,
        RestoreAllChanged,
    };

    enum class BodyRestoreReason : std::uint8_t
    {
        None,
        FailedGrabSetup,
        Release,
    };

    struct BodyLifecycleRecord
    {
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t motionIndex = 0;
        std::uint16_t motionPropertiesId = kDynamicMotionPropertiesId;
        std::uint32_t bodyFlags = 0;
        std::uint32_t filterInfo = 0;
        std::uintptr_t ownerKey = 0;
        physics_body_classifier::BodyMotionType originalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        MotionRole motionRole = MotionRole::Unknown;
        bool acceptedBeforePrep = false;
        bool motionChangedByRock = false;
        bool filterChangedByRock = false;
        bool hasDampingSnapshot = false;
        bool hasInertiaSnapshot = false;
    };

    struct BodyRestorePlanEntry
    {
        BodyLifecycleRecord record{};
        bool restoreMotion = false;
        bool restoreFilter = false;
    };

    struct MotionRestoreCommand
    {
        std::uintptr_t ownerKey = 0;
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint16_t motionPropertiesId = kDynamicMotionPropertiesId;
        physics_body_classifier::BodyMotionType motionType = physics_body_classifier::BodyMotionType::Unknown;
        bool recursive = false;
        bool force = true;
        bool activate = false;
    };

    struct BodyRestorePlan
    {
        BodyRestoreReason reason = BodyRestoreReason::None;
        BodyRestorePolicy policy = BodyRestorePolicy::ProtectComplexSystemOwned;
        std::vector<BodyRestorePlanEntry> entries;
        std::uint32_t motionRestoreCount = 0;
        std::uint32_t filterRestoreCount = 0;

        bool shouldRestoreMotion(std::uint32_t bodyId) const
        {
            const auto it = std::find_if(entries.begin(), entries.end(), [&](const BodyRestorePlanEntry& entry) {
                return entry.record.bodyId == bodyId;
            });
            return it != entries.end() && it->restoreMotion;
        }
    };

    struct BodyLifecycleAudit
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t convertedCount = 0;
        std::uint32_t restoredMotionCount = 0;
        std::uint32_t restoredFilterCount = 0;
        std::uint32_t dampingSnapshotCount = 0;
        std::uint32_t inertiaSnapshotCount = 0;
        std::uint32_t primaryBodyId = kInvalidBodyId;
        BodyRestoreReason reason = BodyRestoreReason::None;
        BodyRestorePolicy policy = BodyRestorePolicy::ProtectComplexSystemOwned;
    };

    inline MotionRole motionRoleFromBodyMotionType(physics_body_classifier::BodyMotionType motionType)
    {
        return motionType == physics_body_classifier::BodyMotionType::Dynamic ? MotionRole::LooseDynamic : MotionRole::SystemOwnedNonDynamic;
    }

    inline BodyLifecycleRecord makeLifecycleRecord(const object_physics_body_set::ObjectPhysicsBodyRecord& record)
    {
        BodyLifecycleRecord result{};
        result.bodyId = record.bodyId;
        result.motionIndex = record.motionId;
        result.motionPropertiesId = record.motionPropertiesId;
        result.bodyFlags = record.bodyFlags;
        result.filterInfo = record.filterInfo;
        result.ownerKey = reinterpret_cast<std::uintptr_t>(record.owningNode);
        result.originalMotionType = record.motionType;
        result.motionRole = motionRoleFromBodyMotionType(record.motionType);
        result.acceptedBeforePrep = record.accepted;
        return result;
    }

    inline bool shouldCaptureBeforeActivePrep(physics_body_classifier::BodyRejectReason reason)
    {
        using physics_body_classifier::BodyRejectReason;
        switch (reason) {
        case BodyRejectReason::None:
        case BodyRejectReason::StaticMotion:
        case BodyRejectReason::KeyframedPassive:
        case BodyRejectReason::NotDynamicAfterActivePrep:
            return true;
        default:
            return false;
        }
    }

    class BodyLifecycleSnapshot
    {
    public:
        void clear() { _records.clear(); }

        void capture(const BodyLifecycleRecord& record)
        {
            if (record.bodyId == kInvalidBodyId) {
                return;
            }
            if (find(record.bodyId)) {
                return;
            }
            _records.push_back(record);
        }

        void captureAcceptedBeforePrep(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
        {
            for (const auto& record : bodySet.records) {
                if (record.accepted) {
                    capture(makeLifecycleRecord(record));
                }
            }
        }

        void captureBeforeActivePrep(const object_physics_body_set::ObjectPhysicsBodySet& bodySet)
        {
            for (const auto& record : bodySet.records) {
                if (shouldCaptureBeforeActivePrep(record.rejectReason)) {
                    capture(makeLifecycleRecord(record));
                }
            }
        }

        BodyLifecycleRecord* find(std::uint32_t bodyId)
        {
            const auto it = std::find_if(_records.begin(), _records.end(), [&](const BodyLifecycleRecord& record) {
                return record.bodyId == bodyId;
            });
            return it != _records.end() ? &*it : nullptr;
        }

        const BodyLifecycleRecord* find(std::uint32_t bodyId) const
        {
            const auto it = std::find_if(_records.begin(), _records.end(), [&](const BodyLifecycleRecord& record) {
                return record.bodyId == bodyId;
            });
            return it != _records.end() ? &*it : nullptr;
        }

        void markPreparedBody(std::uint32_t bodyId, bool motionChangedByRock)
        {
            if (auto* record = find(bodyId)) {
                record->motionChangedByRock = record->motionChangedByRock || motionChangedByRock;
            }
        }

        void markPreparedBodies(const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet)
        {
            for (const auto& record : preparedBodySet.records) {
                if (!record.accepted) {
                    continue;
                }
                if (auto* captured = find(record.bodyId)) {
                    captured->motionChangedByRock = true;
                }
            }
        }

        void markFilterChanged(std::uint32_t bodyId)
        {
            if (auto* record = find(bodyId)) {
                record->filterChangedByRock = true;
            }
        }

        BodyRestorePlan restorePlanForFailure() const { return makeRestorePlan(BodyRestoreReason::FailedGrabSetup, BodyRestorePolicy::RestoreAllChanged); }

        BodyRestorePlan restorePlanForRelease(BodyRestorePolicy policy) const { return makeRestorePlan(BodyRestoreReason::Release, policy); }

        std::vector<MotionRestoreCommand> makeMotionRestoreCommands(const BodyRestorePlan& plan) const
        {
            std::vector<MotionRestoreCommand> commands;
            commands.reserve(plan.motionRestoreCount);
            for (const auto& entry : plan.entries) {
                if (!entry.restoreMotion || entry.record.ownerKey == 0 || entry.record.bodyId == kInvalidBodyId) {
                    continue;
                }
                commands.push_back(MotionRestoreCommand{
                    .ownerKey = entry.record.ownerKey,
                    .bodyId = entry.record.bodyId,
                    .motionPropertiesId = entry.record.motionPropertiesId,
                    .motionType = entry.record.originalMotionType == physics_body_classifier::BodyMotionType::Unknown ?
                                       physics_body_classifier::motionTypeFromMotionPropertiesId(entry.record.motionPropertiesId) :
                                       entry.record.originalMotionType,
                    .recursive = false,
                    .force = true,
                    .activate = false,
                });
            }
            return commands;
        }

        std::size_t size() const { return _records.size(); }

        std::uint32_t convertedCount() const
        {
            return static_cast<std::uint32_t>(std::count_if(_records.begin(), _records.end(), [](const BodyLifecycleRecord& record) {
                return record.motionChangedByRock;
            }));
        }

        std::uint32_t dampingSnapshotCount() const
        {
            return static_cast<std::uint32_t>(std::count_if(_records.begin(), _records.end(), [](const BodyLifecycleRecord& record) {
                return record.hasDampingSnapshot;
            }));
        }

        std::uint32_t inertiaSnapshotCount() const
        {
            return static_cast<std::uint32_t>(std::count_if(_records.begin(), _records.end(), [](const BodyLifecycleRecord& record) {
                return record.hasInertiaSnapshot;
            }));
        }

        const std::vector<BodyLifecycleRecord>& records() const { return _records; }

    private:
        BodyRestorePlan makeRestorePlan(BodyRestoreReason reason, BodyRestorePolicy policy) const
        {
            BodyRestorePlan plan{};
            plan.reason = reason;
            plan.policy = policy;
            plan.entries.reserve(_records.size());

            for (const auto& record : _records) {
                BodyRestorePlanEntry entry{};
                entry.record = record;
                entry.restoreFilter = true;

                if (reason == BodyRestoreReason::FailedGrabSetup) {
                    entry.restoreMotion = record.motionChangedByRock;
                } else if (policy == BodyRestorePolicy::RestoreAllChanged) {
                    entry.restoreMotion = record.motionChangedByRock;
                } else {
                    entry.restoreMotion = record.motionChangedByRock && record.motionRole == MotionRole::SystemOwnedNonDynamic;
                }

                if (entry.restoreMotion) {
                    ++plan.motionRestoreCount;
                }
                if (entry.restoreFilter) {
                    ++plan.filterRestoreCount;
                }
                plan.entries.push_back(entry);
            }

            return plan;
        }

        std::vector<BodyLifecycleRecord> _records;
    };

    inline BodyLifecycleAudit makeLifecycleAudit(const BodyLifecycleSnapshot& snapshot, const BodyRestorePlan& plan, std::uint32_t primaryBodyId)
    {
        return BodyLifecycleAudit{
            .bodyCount = static_cast<std::uint32_t>(snapshot.size()),
            .convertedCount = snapshot.convertedCount(),
            .restoredMotionCount = plan.motionRestoreCount,
            .restoredFilterCount = plan.filterRestoreCount,
            .dampingSnapshotCount = snapshot.dampingSnapshotCount(),
            .inertiaSnapshotCount = snapshot.inertiaSnapshotCount(),
            .primaryBodyId = primaryBodyId,
            .reason = plan.reason,
            .policy = plan.policy,
        };
    }
}

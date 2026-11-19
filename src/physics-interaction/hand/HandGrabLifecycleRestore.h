// Restores what a grab or pull changed on its object's bodies (motion type,
// collision) when the grab ends or its preparation is abandoned.
#pragma once

#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock::hand_grab_internal
{
    inline physics_recursive_wrappers::MotionPreset motionPresetFromMotionType(
        physics_body_classifier::BodyMotionType motionType, std::uint16_t fallbackMotionPropertiesId)
    {
        switch (motionType) {
        case physics_body_classifier::BodyMotionType::Static:
            return physics_recursive_wrappers::MotionPreset::Static;
        case physics_body_classifier::BodyMotionType::Keyframed:
            return physics_recursive_wrappers::MotionPreset::Keyframed;
        case physics_body_classifier::BodyMotionType::Dynamic:
            return physics_recursive_wrappers::MotionPreset::Dynamic;
        default:
            break;
        }

        switch (fallbackMotionPropertiesId & 0xFF) {
        case 0:
            return physics_recursive_wrappers::MotionPreset::Static;
        case 2:
            return physics_recursive_wrappers::MotionPreset::Keyframed;
        case 1:
        default:
            return physics_recursive_wrappers::MotionPreset::Dynamic;
        }
    }

    inline active_grab_body_lifecycle::BodyLifecycleAudit restoreActiveGrabLifecycle(RE::hknpWorld* world,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& snapshot,
        const active_grab_body_lifecycle::BodyRestorePlan& plan, std::uint32_t primaryBodyId, const char* handName,
        const char* context)
    {
        auto audit = active_grab_body_lifecycle::makeLifecycleAudit(snapshot, plan, primaryBodyId);
        if (!world) {
            return audit;
        }

        for (const auto& entry : plan.entries) {
            const auto bodyId = entry.record.bodyId;
            if (bodyId == INVALID_BODY_ID) {
                continue;
            }

            if (entry.restoreFilter) {
                body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, entry.record.filterInfo);
            }
        }

        for (const auto& command : snapshot.makeMotionRestoreCommands(plan)) {
            auto* ownerNode = reinterpret_cast<RE::NiAVObject*>(command.ownerKey);
            if (!ownerNode) {
                continue;
            }
            physics_recursive_wrappers::setMotionRecursive(ownerNode,
                motionPresetFromMotionType(command.motionType, command.motionPropertiesId), command.recursive,
                command.force, command.activate);
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand grab lifecycle audit {}: targetKind={} intent={} bodies={} converted={} restoredMotion={} "
            "restoredFilter={} preservedMotion={} preservedFilter={} dampingSnapshots={} inertiaSnapshots={} "
            "latePrepared={} incompleteScan={} primaryBody={}",
            handName ? handName : "?", context ? context : "", grab_target::name(audit.targetKind),
            active_grab_body_lifecycle::releaseIntentName(audit.intent), audit.bodyCount, audit.convertedCount,
            audit.restoredMotionCount, audit.restoredFilterCount, audit.preservedConvertedMotionCount,
            audit.preservedConvertedFilterCount, audit.dampingSnapshotCount, audit.inertiaSnapshotCount,
            audit.latePreparedBodyCount, audit.incompleteNativeScan ? "yes" : "no", audit.primaryBodyId);
        return audit;
    }

    inline bool restoreIncompleteActivePrepRoot(RE::NiAVObject* rootNode, std::uint16_t originalMotionPropsId,
        const char* handName, const char* context)
    {
        /*
             * Late-discovered bodies have already been touched by recursive
             * active prep, but their original per-body state was not captured.
             * Individual restore would manufacture state. When a scan is known
             * incomplete, restore the object root with the selected body's
             * original motion preset so the whole Fallout-owned system returns
             * to one coherent motion mode.
             */
        if (!rootNode) {
            return false;
        }

        const auto motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(originalMotionPropsId);
        const bool restored = physics_recursive_wrappers::setMotionRecursive(rootNode,
            motionPresetFromMotionType(motionType, originalMotionPropsId), true, true, false);

        ROCK_LOG_WARN(Hand,
            "{} hand {}: recursive root restore after incomplete object scan root='{}' motionProps={} result={}",
            handName ? handName : "?", context ? context : "incomplete-scan", nodeDebugName(rootNode),
            originalMotionPropsId, restored ? "ok" : "failed");
        return restored;
    }
}

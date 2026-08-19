#include "physics-interaction/hand/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/HandGrabTrace.h"

#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace rock::hand_grab_detail
{
    namespace
    {
        constexpr std::uint32_t kHeldCollisionParticipationFlags = 0x80u;
        constexpr std::uint32_t kHeldCollisionParticipationFlagMode = 0u;
        constexpr std::uint32_t kHeldAuthorityBodyFlags = 0x08000000u;
        constexpr std::uint32_t kHeldAuthorityBodyFlagMode = 1u;
    }

    float HeldBodyMassSummary::motorMass() const noexcept
    {
        if (std::isfinite(aggregateMass) && aggregateMass > 0.0f) {
            return aggregateMass;
        }
        return (std::isfinite(primaryMass) && primaryMass > 0.0f) ? primaryMass : 0.0f;
    }

    float effectiveGrabMotorMass(float mass)
    {
        return grab_motion_controller::effectiveMotorMass(
            mass,
            g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
            g_rockConfig.rockGrabEffectiveMotorMassFloor);
    }
    
    std::uintptr_t heldBodyFlagLeaseOwner(const Hand* hand)
    {
        return reinterpret_cast<std::uintptr_t>(hand) ^ 0x524F434B48454C44ull;
    }
    
    HeldBodyActivationSummary activateHeldObjectBodySet(
        RE::hknpWorld* world,
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds)
    {
        HeldBodyActivationSummary summary{};
        if (!world) {
            return summary;
        }
    
        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (physics_recursive_wrappers::activateBody(world, bodyId)) {
                ++summary.activatedCount;
            } else {
                ++summary.failedActivationCount;
            }
        }
        return summary;
    }
    
    HeldBodyFlagLeaseSummary acquireHeldObjectBodyFlagLeases(
        RE::hknpWorld* world,
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        std::uintptr_t ownerToken)
    {
        /*
         * Proxy-constraint grab replaced the native held-object action, but the
         * old path owned two different flag contracts: 0x80 was leased across
         * the accepted held body set by ROCK, while the native action leased
         * 0x08000000 only on its selected primary body. Keeping that split is
         * important for multipart weapons because secondary collision bodies
         * should participate in the hold without all becoming grab-authority
         * bodies.
         */
        HeldBodyFlagLeaseSummary summary{};
        if (!world || ownerToken == 0) {
            return summary;
        }
    
        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (havok_runtime::acquireBodyFlagLease(
                    world,
                    bodyId,
                    kHeldCollisionParticipationFlags,
                    kHeldCollisionParticipationFlagMode,
                    ownerToken)) {
                ++summary.collisionLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
    
        if (primaryBodyId != INVALID_BODY_ID) {
            if (havok_runtime::acquireBodyFlagLease(
                    world,
                    primaryBodyId,
                    kHeldAuthorityBodyFlags,
                    kHeldAuthorityBodyFlagMode,
                    ownerToken)) {
                ++summary.authorityLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
        return summary;
    }
    
    HeldBodyFlagLeaseSummary releaseHeldObjectBodyFlagLeases(
        RE::hknpWorld* world,
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        std::uintptr_t ownerToken,
        bool restoreOnFinalLease)
    {
        HeldBodyFlagLeaseSummary summary{};
        if (!world || ownerToken == 0) {
            return summary;
        }
    
        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (havok_runtime::releaseBodyFlagLease(
                    world,
                    bodyId,
                    kHeldCollisionParticipationFlags,
                    kHeldCollisionParticipationFlagMode,
                    ownerToken,
                    restoreOnFinalLease)) {
                ++summary.collisionLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
    
        if (primaryBodyId != INVALID_BODY_ID) {
            if (havok_runtime::releaseBodyFlagLease(
                    world,
                    primaryBodyId,
                    kHeldAuthorityBodyFlags,
                    kHeldAuthorityBodyFlagMode,
                    ownerToken,
                    restoreOnFinalLease)) {
                ++summary.authorityLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
        return summary;
    }
    
    std::vector<std::uint32_t> buildCommittedHeldBodyIds(
        std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& mechanicalScopeBodyIds,
        const GrabSharedObjectContext& sharedContext,
        bool& adoptedPeerHeldBodyIds)
    {
        /*
         * The second hand has its own selected primary body for drive-frame
         * capture, but the object body set is already owned by the first hand.
         * Reusing the peer's committed body list keeps multipart activation,
         * flag leases, release velocity, and final restoration on the same
         * bodies instead of depending on a second close-selection rescan.
         */
        adoptedPeerHeldBodyIds = false;
        std::vector<std::uint32_t> sourceBodyIds;
        if (sharedContext.hasPeerState() && sharedContext.peerHeldBodyIds && !sharedContext.peerHeldBodyIds->empty()) {
            sourceBodyIds = *sharedContext.peerHeldBodyIds;
            adoptedPeerHeldBodyIds = true;
        } else {
            sourceBodyIds = mechanicalScopeBodyIds;
        }
    
        return held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, sourceBodyIds);
    }
    
    physics_recursive_wrappers::MotionPreset motionPresetFromMotionType(
        physics_body_classifier::BodyMotionType motionType,
        std::uint16_t fallbackMotionPropertiesId)
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
    
    active_grab_body_lifecycle::BodyLifecycleAudit restoreActiveGrabLifecycle(RE::hknpWorld* world,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& snapshot,
        const active_grab_body_lifecycle::BodyRestorePlan& plan,
        std::uint32_t primaryBodyId,
        const char* handName,
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
            physics_recursive_wrappers::setMotionRecursive(
                ownerNode,
                motionPresetFromMotionType(command.motionType, command.motionPropertiesId),
                command.recursive,
                command.force,
                command.activate);
        }
    
        ROCK_LOG_DEBUG(Hand,
            "{} hand grab lifecycle audit {}: targetKind={} intent={} bodies={} converted={} restoredMotion={} restoredFilter={} preservedMotion={} preservedFilter={} dampingSnapshots={} inertiaSnapshots={} latePrepared={} incompleteScan={} primaryBody={}",
            handName ? handName : "?",
            context ? context : "",
            grab_target::name(audit.targetKind),
            active_grab_body_lifecycle::releaseIntentName(audit.intent),
            audit.bodyCount,
            audit.convertedCount,
            audit.restoredMotionCount,
            audit.restoredFilterCount,
            audit.preservedConvertedMotionCount,
            audit.preservedConvertedFilterCount,
            audit.dampingSnapshotCount,
            audit.inertiaSnapshotCount,
            audit.latePreparedBodyCount,
            audit.incompleteNativeScan ? "yes" : "no",
            audit.primaryBodyId);
        return audit;
    }
    
    HeldObjectMotionSample sampleHeldObjectMotion(RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        bool includeConnectedBodies)
    {
        HeldObjectMotionSample result{};
        if (!world) {
            return result;
        }
    
        constexpr std::size_t kMaxSampledMotionSlots = 96;
        std::array<std::uint32_t, kMaxSampledMotionSlots> sampledMotionSlots{};
        std::size_t sampledMotionSlotCount = 0;
    
        auto motionSlotAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
            for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                if (sampledMotionSlots[i] == motionIndex) {
                    return true;
                }
            }
            return false;
        };
    
        auto sampleBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }
    
            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body) {
                return;
            }
    
            const std::uint32_t motionIndex = body->motionIndex;
            if (!body_frame::hasUsableMotionIndex(motionIndex) || motionSlotAlreadySampled(motionIndex)) {
                return;
            }
    
            if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                return;
            }
    
            auto* motion = havok_runtime::getMotion(world, motionIndex);
            if (!motion) {
                return;
            }
    
            sampledMotionSlots[sampledMotionSlotCount++] = motionIndex;
    
            const RE::NiPoint3 localLinearVelocity{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z };
    
            if (bodyId == primaryBodyId.value) {
                result.primaryLocalLinearVelocity = localLinearVelocity;
                result.hasPrimaryVelocity = true;
            }
        };
    
        sampleBody(primaryBodyId.value);
        if (includeConnectedBodies) {
            for (const auto bodyId : heldBodyIds) {
                sampleBody(bodyId);
            }
        }
    
        return result;
    }
    
    void setHeldVelocity(RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        const RE::NiPoint3& linearVelocity,
        const RE::NiPoint3& angularVelocity,
        bool overrideAngularVelocity,
        float angularVelocityKeep,
        bool includeConnectedLinearVelocity,
        bool includeConnectedAngularVelocity)
    {
        if (!world) {
            return;
        }
    
        constexpr std::size_t kMaxVelocityMotionSlots = 96;
        std::array<std::uint32_t, kMaxVelocityMotionSlots> updatedMotionSlots{};
        std::size_t updatedMotionSlotCount = 0;
    
        auto alreadyUpdated = [&updatedMotionSlots, &updatedMotionSlotCount](std::uint32_t motionIndex) {
            for (std::size_t i = 0; i < updatedMotionSlotCount; ++i) {
                if (updatedMotionSlots[i] == motionIndex) {
                    return true;
                }
            }
            return false;
        };
    
        auto setBody = [&](std::uint32_t bodyId, bool primaryBody) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }
    
            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
            if (!body) {
                return;
            }
    
            const std::uint32_t motionIndex = body->motionIndex;
            if (!body_frame::hasUsableMotionIndex(motionIndex) || alreadyUpdated(motionIndex)) {
                return;
            }
    
            if (updatedMotionSlotCount >= updatedMotionSlots.size()) {
                return;
            }
    
            auto* motion = havok_runtime::getMotion(world, motionIndex);
            if (!motion) {
                return;
            }
    
            updatedMotionSlots[updatedMotionSlotCount++] = motionIndex;
            const bool applyLinearForBody = primaryBody || includeConnectedLinearVelocity;
            const float angularKeep = std::clamp(std::isfinite(angularVelocityKeep) ? angularVelocityKeep : 1.0f, 0.0f, 1.0f);
            const bool overrideAngularForBody = overrideAngularVelocity && (primaryBody || includeConnectedAngularVelocity);
            const RE::hkVector4f linearHavok = applyLinearForBody ?
                RE::hkVector4f{ linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f } :
                RE::hkVector4f{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z, 0.0f };
            const RE::hkVector4f angularHavok = overrideAngularForBody ?
                RE::hkVector4f{ angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f } :
                RE::hkVector4f{ motion->angularVelocity.x * angularKeep, motion->angularVelocity.y * angularKeep, motion->angularVelocity.z * angularKeep, 0.0f };
            havok_runtime::setBodyVelocityDeferred(world,
                bodyId,
                linearHavok,
                angularHavok);
        };
    
        setBody(primaryBodyId.value, true);
        if (includeConnectedLinearVelocity || includeConnectedAngularVelocity) {
            for (const auto bodyId : heldBodyIds) {
                setBody(bodyId, false);
            }
        }
    }
    
    void setHeldLinearVelocity(RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        const RE::NiPoint3& linearVelocity,
        float angularVelocityKeep,
        bool includeConnectedBodies)
    {
        setHeldVelocity(world, primaryBodyId, heldBodyIds, linearVelocity, RE::NiPoint3{}, false, angularVelocityKeep, includeConnectedBodies, false);
    }
    
    float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        if (!world || bodyId.value == INVALID_BODY_ID) {
            return 0.0f;
        }
    
        auto* motion = havok_runtime::getBodyMotion(world, bodyId);
        if (!motion) {
            return 0.0f;
        }
    
        const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
        if (packedInvMass == 0) {
            return 0.0f;
        }
    
        return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
    }
    
    HeldBodyMassSummary readHeldBodyMassSummary(RE::hknpWorld* world,
        RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds,
        bool includeConnectedBodies)
    {
        /*
         * Dynamic grab owns one held object even when FO4VR exposes that
         * object as several hknp bodies. Lifecycle, inertia normalization,
         * release velocity, and nearby damping already operate on the whole
         * accepted body set. The motor mass budget must use the same object
         * scope, with unique-motion dedupe, so multipart loose weapons are
         * not budgeted from whichever child body happened to be selected.
         */
        HeldBodyMassSummary summary{};
        summary.primaryMass = readBodyMass(world, primaryBodyId);
        if (!world) {
            summary.aggregateMass = summary.primaryMass;
            return summary;
        }
    
        constexpr std::size_t kMaxMassMotionSlots = 96;
        std::array<std::uint32_t, kMaxMassMotionSlots> sampledMotionSlots{};
        std::size_t sampledMotionSlotCount = 0;
    
        auto motionAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
            for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                if (sampledMotionSlots[i] == motionIndex) {
                    return true;
                }
            }
            return false;
        };
    
        auto sampleBody = [&](std::uint32_t rawBodyId) {
            if (rawBodyId == INVALID_BODY_ID) {
                return;
            }
    
            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ rawBodyId });
            if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) || motionAlreadySampled(body->motionIndex)) {
                return;
            }
            if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                return;
            }
    
            const float mass = readBodyMass(world, RE::hknpBodyId{ rawBodyId });
            if (!std::isfinite(mass) || mass <= 0.0f) {
                return;
            }
    
            sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
            summary.aggregateMass += mass;
            ++summary.sampledBodies;
            summary.uniqueMotions = static_cast<std::uint32_t>(sampledMotionSlotCount);
        };
    
        sampleBody(primaryBodyId.value);
        if (includeConnectedBodies) {
            for (const auto bodyId : heldBodyIds) {
                sampleBody(bodyId);
            }
        }
    
        if (!(std::isfinite(summary.aggregateMass) && summary.aggregateMass > 0.0f)) {
            summary.aggregateMass = summary.primaryMass;
        }
        return summary;
    }
    
    held_object_contact_policy::HeldContactOtherMotion classifyHeldContactOtherMotion(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        if (!world || bodyId == INVALID_BODY_ID) {
            return held_object_contact_policy::HeldContactOtherMotion::Unknown;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
        if (!motion) {
            return held_object_contact_policy::HeldContactOtherMotion::Unknown;
        }

        const float mass = readBodyMass(world, RE::hknpBodyId{ bodyId });
        return mass > 0.0f ?
            held_object_contact_policy::HeldContactOtherMotion::Dynamic :
            held_object_contact_policy::HeldContactOtherMotion::FixedOrStatic;
    }
    const char* bodyMotionTypeName(physics_body_classifier::BodyMotionType motionType)
    {
        using physics_body_classifier::BodyMotionType;
        switch (motionType) {
        case BodyMotionType::Static:
            return "Static";
        case BodyMotionType::Dynamic:
            return "Dynamic";
        case BodyMotionType::Keyframed:
            return "Keyframed";
        case BodyMotionType::Other:
            return "Other";
        case BodyMotionType::Unknown:
        default:
            return "Unknown";
        }
    }
    
    const object_physics_body_set::ObjectPhysicsBodyRecord* diagnosticRejectedBodyRecord(
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t preferredBodyId)
    {
        if (const auto* preferred = bodySet.findRecord(preferredBodyId); preferred && !preferred->accepted) {
            return preferred;
        }
        for (const auto& record : bodySet.records) {
            if (!record.accepted) {
                return &record;
            }
        }
        return nullptr;
    }
    
    bool restoreIncompleteActivePrepRoot(
        RE::NiAVObject* rootNode,
        std::uint16_t originalMotionPropsId,
        const char* handName,
        const char* context)
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
        const bool restored = physics_recursive_wrappers::setMotionRecursive(
            rootNode,
            motionPresetFromMotionType(motionType, originalMotionPropsId),
            true,
            true,
            false);
    
        ROCK_LOG_WARN(Hand,
            "{} hand {}: recursive root restore after incomplete object scan root='{}' motionProps={} result={}",
            handName ? handName : "?",
            context ? context : "incomplete-scan",
            nodeDebugName(rootNode),
            originalMotionPropsId,
            restored ? "ok" : "failed");
        return restored;
    }
}

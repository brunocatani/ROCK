#include "physics-interaction/hand/DynamicHandCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"
#include "physics-interaction/hand/DynamicHandCollisionPolicy.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"

#include <algorithm>
#include <cmath>

namespace rock
{
    namespace
    {
        constexpr const char* RIGHT_DYNAMIC_HAND_TAG = "ROCK_DynamicHand_Right";
        constexpr const char* LEFT_DYNAMIC_HAND_TAG = "ROCK_DynamicHand_Left";
        constexpr const char* SURFACE_FINGER_POSE_TAG =
            "ROCK_SurfaceFingerCollision";
        constexpr const char* SURFACE_MESH_GRAB_POSE_TAG =
            "ROCK_SurfaceMeshGrab";
        constexpr std::uint32_t kDynamicRightHandProxyCollisionGroup = 0x000C;
        constexpr std::uint32_t kDynamicLeftHandProxyCollisionGroup = 0x000E;
        static_assert(kDynamicRightHandProxyCollisionGroup != kDynamicLeftHandProxyCollisionGroup);
        constexpr std::uint32_t kRaiseManifoldProcessedEvents = 0x40u;
        constexpr std::uint32_t kRebuildBodyCollisionState = 0u;
        constexpr int kSurfaceLatchVisualPriority = 100;
        constexpr float kCompoundContactRetentionSeconds = 0.050f;
        constexpr float kHandCompoundMass = 2.0f;
        constexpr float kHandCompoundInverseInertiaMultiplier = 1.0f;

        const char* dynamicHandTag(bool isLeft)
        {
            return isLeft ? LEFT_DYNAMIC_HAND_TAG : RIGHT_DYNAMIC_HAND_TAG;
        }

        std::size_t handIndex(bool isLeft)
        {
            return isLeft ? 1u : 0u;
        }

        std::uint32_t dynamicHandProxyFilterInfo(
            bool isLeft,
            bool suppressCollision = false)
        {
            const auto collisionGroup = isLeft ?
                kDynamicLeftHandProxyCollisionGroup :
                kDynamicRightHandProxyCollisionGroup;
            const auto baseFilter =
                (collisionGroup << 16) |
                (collision_layer_policy::dynamicHandProxyLayerForHand(isLeft) &
                    collision_layer_policy::FO4_LAYER_FILTER_MASK);
            return suppressCollision ?
                (baseFilter | collision_suppression_registry::kSuppressionNoCollideBit) :
                baseFilter;
        }

        bool isFinitePoint(const RE::NiPoint3& value)
        {
            return vector_math::hasFiniteComponents(value);
        }

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            if (!isFinitePoint(transform.translate) || !std::isfinite(transform.scale)) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        /*
         * Generated hand and forearm collider frames store physical axes in
         * matrix columns. TransformMath relative composition expects physical
         * axes in rows. Convert only at that math boundary. The compound root
         * body continues to consume the original generated-collider frame.
         */
        RE::NiTransform colliderFrameToSceneFrame(
            const RE::NiTransform& colliderFrame)
        {
            RE::NiTransform result = colliderFrame;
            result.rotate =
                transform_math::transposeRotation(colliderFrame.rotate);
            return result;
        }

        bool makeCompoundChildTransform(
            const RE::NiTransform& childInCompound,
            havok_compound_shape_builder::ChildTransform& outTransform)
        {
            outTransform = {};
            if (!isFiniteTransform(childInCompound) ||
                std::abs(childInCompound.scale - 1.0f) > 0.0001f) {
                return false;
            }
            const float gameToHavok = physics_scale::gameToHavok();
            if (!physics_scale::isUsableScale(gameToHavok)) {
                return false;
            }

            // Current ROCK convention: NiTransform local axes are rows, and
            // each row maps directly to one hkTransformf child column.
            outTransform.column0 = {
                childInCompound.rotate.entry[0][0],
                childInCompound.rotate.entry[0][1],
                childInCompound.rotate.entry[0][2],
                0.0f,
            };
            outTransform.column1 = {
                childInCompound.rotate.entry[1][0],
                childInCompound.rotate.entry[1][1],
                childInCompound.rotate.entry[1][2],
                0.0f,
            };
            outTransform.column2 = {
                childInCompound.rotate.entry[2][0],
                childInCompound.rotate.entry[2][1],
                childInCompound.rotate.entry[2][2],
                0.0f,
            };
            outTransform.translation = {
                childInCompound.translate.x * gameToHavok,
                childInCompound.translate.y * gameToHavok,
                childInCompound.translate.z * gameToHavok,
                1.0f,
            };
            return true;
        }

        bool applyHandCompoundMassProperties(
            RE::hknpWorld* world,
            const RE::hknpBodyId bodyId,
            const bool isLeft,
            const RE::NiPoint3& boundsMinGame,
            const RE::NiPoint3& boundsMaxGame)
        {
            const auto geometry =
                dynamic_weapon_collision_policy::makeBoundingBoxGeometry(
                    boundsMinGame,
                    boundsMaxGame);
            const auto envelope =
                dynamic_weapon_collision_policy::makeBoundingBoxMassProperties(
                    geometry,
                    1.0f,
                    0.0f,
                    physics_scale::gameToHavok(),
                    kHandCompoundMass);
            if (!envelope.valid) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound envelope mass properties invalid: body={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value);
                return false;
            }

            const auto normalizedInertia =
                grab_inertia_policy::normalizeInverseInertiaAxesForGrab(
                    envelope.inverseInertia.x *
                        kHandCompoundInverseInertiaMultiplier,
                    envelope.inverseInertia.y *
                        kHandCompoundInverseInertiaMultiplier,
                    envelope.inverseInertia.z *
                        kHandCompoundInverseInertiaMultiplier,
                    g_rockConfig.rockGrabMaxInertiaRatio,
                    g_rockConfig.rockGrabMinInertia);
            if (!normalizedInertia.valid) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand compound inertia normalization failed: body={}",
                    isLeft ? "Left" : "Right",
                    bodyId.value);
                return false;
            }

            const auto initialMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!initialMotion.valid || !initialMotion.motion) {
                return false;
            }
            auto* initialPacked = reinterpret_cast<std::int16_t*>(
                reinterpret_cast<char*>(initialMotion.motion) +
                MOTION_PACKED_INERTIA_OFFSET);
            const std::int16_t desiredPackedInertia[3] = {
                repackBfloat16(normalizedInertia.normalized[0]),
                repackBfloat16(normalizedInertia.normalized[1]),
                repackBfloat16(normalizedInertia.normalized[2]),
            };
            const std::int16_t desiredPackedMass = initialPacked[3];
            if (desiredPackedInertia[0] <= 0 ||
                desiredPackedInertia[1] <= 0 ||
                desiredPackedInertia[2] <= 0 || desiredPackedMass <= 0) {
                return false;
            }

            initialPacked[0] = desiredPackedInertia[0];
            initialPacked[1] = desiredPackedInertia[1];
            initialPacked[2] = desiredPackedInertia[2];
            if (!havok_runtime::rebuildMotionMassProperties(
                    world,
                    initialMotion.motionIndex)) {
                return false;
            }
            const auto rebuiltMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!rebuiltMotion.valid || !rebuiltMotion.motion ||
                rebuiltMotion.motionIndex != initialMotion.motionIndex) {
                return false;
            }
            auto* rebuiltPacked = reinterpret_cast<std::int16_t*>(
                reinterpret_cast<char*>(rebuiltMotion.motion) +
                MOTION_PACKED_INERTIA_OFFSET);
            rebuiltPacked[0] = desiredPackedInertia[0];
            rebuiltPacked[1] = desiredPackedInertia[1];
            rebuiltPacked[2] = desiredPackedInertia[2];
            rebuiltPacked[3] = desiredPackedMass;
            return true;
        }

        float pointLength(const RE::NiPoint3& value)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            return std::isfinite(lengthSquared) && lengthSquared >= 0.0f ? std::sqrt(lengthSquared) : 0.0f;
        }

        RE::NiPoint3 subtractPoints(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{
                lhs.x - rhs.x,
                lhs.y - rhs.y,
                lhs.z - rhs.z,
            };
        }

        const dynamic_hand_twin::TwinSlotFrame* twinFrameForSlot(
            const dynamic_hand_twin::TwinTargets& handTwins,
            const dynamic_hand_twin::ForearmTwinTargets& forearmTwins,
            bool isLeft,
            std::size_t bodyIndex)
        {
            if (bodyIndex == DynamicHandCollisionRuntime::kPalmSlot) {
                return &handTwins.palm;
            }
            if (bodyIndex < DynamicHandCollisionRuntime::kFirstForearmSlot) {
                const std::size_t fingerIndex =
                    dynamic_hand_collision_telemetry::fingerIndexForBodyIndex(
                        bodyIndex);
                const std::size_t segmentIndex =
                    dynamic_hand_collision_telemetry::
                        fingerSegmentIndexForBodyIndex(bodyIndex);
                return fingerIndex < handTwins.fingers.size() &&
                               segmentIndex <
                                   handTwins.fingers[fingerIndex].size() ?
                    &handTwins.fingers[fingerIndex][segmentIndex] :
                    nullptr;
            }

            const std::size_t forearmIndex = bodyIndex - DynamicHandCollisionRuntime::kFirstForearmSlot;
            const auto& sideForearms = forearmTwins.forHand(isLeft);
            return forearmIndex < sideForearms.size() ? &sideForearms[forearmIndex] : nullptr;
        }

        float dotPoints(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return vector_math::dot(lhs, rhs);
        }

        bool buildPoseFingerSegmentCenters(
            const RE::NiTransform& rawHandWorld,
            const dynamic_hand_twin::TwinTargets& handTwins,
            const frik_visual_authority::FingerLocalTransformOverride& pose,
            std::array<RE::NiPoint3,
                hand_collider_semantics::kHandFingerRoleCount>& outCenters)
        {
            constexpr std::uint16_t kFullFingerMask = 0x7FFFu;
            if ((pose.enabledMask & kFullFingerMask) != kFullFingerMask ||
                !isFiniteTransform(rawHandWorld)) {
                return false;
            }

            for (std::size_t finger = 0;
                 finger < hand_collider_semantics::kHandFingerCount;
                 ++finger) {
                const std::size_t baseIndex =
                    finger * hand_collider_semantics::kHandFingerSegmentCount;
                const RE::NiTransform baseWorld =
                    transform_math::composeTransforms(
                        rawHandWorld,
                        pose.localTransforms[baseIndex]);
                const RE::NiTransform middleWorld =
                    transform_math::composeTransforms(
                        baseWorld,
                        pose.localTransforms[baseIndex + 1]);
                const RE::NiTransform tipWorld =
                    transform_math::composeTransforms(
                        middleWorld,
                        pose.localTransforms[baseIndex + 2]);
                if (!isFiniteTransform(baseWorld) ||
                    !isFiniteTransform(middleWorld) ||
                    !isFiniteTransform(tipWorld)) {
                    return false;
                }

                outCenters[baseIndex] = RE::NiPoint3{
                    (baseWorld.translate.x + middleWorld.translate.x) * 0.5f,
                    (baseWorld.translate.y + middleWorld.translate.y) * 0.5f,
                    (baseWorld.translate.z + middleWorld.translate.z) * 0.5f,
                };
                outCenters[baseIndex + 1] = RE::NiPoint3{
                    (middleWorld.translate.x + tipWorld.translate.x) * 0.5f,
                    (middleWorld.translate.y + tipWorld.translate.y) * 0.5f,
                    (middleWorld.translate.z + tipWorld.translate.z) * 0.5f,
                };
                const auto& tipFrame =
                    handTwins.fingers[finger][static_cast<std::size_t>(
                        hand_collider_semantics::HandFingerSegment::Tip)];
                if (!tipFrame.valid || !std::isfinite(tipFrame.length) ||
                    tipFrame.length <= 0.0f) {
                    return false;
                }
                const RE::NiPoint3 tipOffset =
                    transform_math::localVectorToWorld(
                        tipWorld,
                        RE::NiPoint3{ tipFrame.length * 0.5f, 0.0f, 0.0f });
                outCenters[baseIndex + 2] =
                    tipWorld.translate + tipOffset;
            }
            return true;
        }

        /*
         * Rigid-hand combination of per-body deviations: each twin's deviation
         * is a half-space push-out for its own contacts, so the minimal hand
         * correction satisfying all of them is the sequential projection over
         * the set (same math family the 6e41044 manifold solver validated).
         * Overlapping deviations from one shared surface collapse instead of
         * double-counting; corner deviations from different directions compose.
         */
        RE::NiPoint3 combineTwinDeviations(
            const std::array<RE::NiPoint3, DynamicHandCollisionRuntime::kBodiesPerHand>& deviations,
            const std::array<bool, DynamicHandCollisionRuntime::kBodiesPerHand>& deviationValid)
        {
            constexpr float kTinyDeviation = 1.0e-4f;
            RE::NiPoint3 combined{};
            for (int pass = 0; pass < 3; ++pass) {
                bool changed = false;
                for (std::size_t i = 0; i < deviations.size(); ++i) {
                    if (!deviationValid[i] || !isFinitePoint(deviations[i])) {
                        continue;
                    }
                    const float length = std::sqrt(
                        deviations[i].x * deviations[i].x +
                        deviations[i].y * deviations[i].y +
                        deviations[i].z * deviations[i].z);
                    if (!std::isfinite(length) || length <= kTinyDeviation) {
                        continue;
                    }
                    const RE::NiPoint3 direction{
                        deviations[i].x / length,
                        deviations[i].y / length,
                        deviations[i].z / length,
                    };
                    const float needed =
                        length - (combined.x * direction.x + combined.y * direction.y + combined.z * direction.z);
                    if (needed <= kTinyDeviation) {
                        continue;
                    }
                    combined.x += direction.x * needed;
                    combined.y += direction.y * needed;
                    combined.z += direction.z * needed;
                    changed = true;
                }
                if (!changed) {
                    break;
                }
            }
            return combined;
        }

        /*
         * Exponential visual release. Active contacts can bypass this filter
         * and remain locked to the current solver result. A lost contact claim
         * decays through it instead of returning to the controller in one
         * frame. speed <= 0 disables the filter.
         */
        RE::NiPoint3 smoothAppliedDeviation(const RE::NiPoint3& applied, const RE::NiPoint3& target, float smoothingSpeed, float deltaSeconds)
        {
            if (!std::isfinite(smoothingSpeed) || smoothingSpeed <= 0.0f) {
                return target;
            }
            const float dt = std::clamp(
                std::isfinite(deltaSeconds) && deltaSeconds > 0.0f ?
                    deltaSeconds :
                    0.0f,
                0.0f,
                0.1f);
            const float alpha = std::clamp(1.0f - std::exp(-smoothingSpeed * dt), 0.0f, 1.0f);
            return RE::NiPoint3{
                applied.x + (target.x - applied.x) * alpha,
                applied.y + (target.y - applied.y) * alpha,
                applied.z + (target.z - applied.z) * alpha,
            };
        }
    }

    void DynamicHandCollisionRuntime::publishPhysicsTelemetry(ProxySlot& slot, const PhysicsTelemetrySample& sample)
    {
        auto& telemetry = slot.physicsTelemetry;
        telemetry.sequence.fetch_add(1, std::memory_order_acq_rel);  // odd: write in progress
        telemetry.requestedX.store(sample.requestedTargetWorldGame.x, std::memory_order_relaxed);
        telemetry.requestedY.store(sample.requestedTargetWorldGame.y, std::memory_order_relaxed);
        telemetry.requestedZ.store(sample.requestedTargetWorldGame.z, std::memory_order_relaxed);
        telemetry.commandedX.store(sample.commandedTargetWorldGame.x, std::memory_order_relaxed);
        telemetry.commandedY.store(sample.commandedTargetWorldGame.y, std::memory_order_relaxed);
        telemetry.commandedZ.store(sample.commandedTargetWorldGame.z, std::memory_order_relaxed);
        telemetry.liveX.store(sample.liveBodyWorldGame.x, std::memory_order_relaxed);
        telemetry.liveY.store(sample.liveBodyWorldGame.y, std::memory_order_relaxed);
        telemetry.liveZ.store(sample.liveBodyWorldGame.z, std::memory_order_relaxed);
        telemetry.targetVelocityX.store(sample.targetVelocityWorldGameUnitsPerSecond.x, std::memory_order_relaxed);
        telemetry.targetVelocityY.store(sample.targetVelocityWorldGameUnitsPerSecond.y, std::memory_order_relaxed);
        telemetry.targetVelocityZ.store(sample.targetVelocityWorldGameUnitsPerSecond.z, std::memory_order_relaxed);
        telemetry.approachSpeed.store(sample.approachSpeedGameUnitsPerSecond, std::memory_order_relaxed);
        telemetry.physicsDeltaSeconds.store(sample.physicsDeltaSeconds, std::memory_order_relaxed);
        telemetry.valid.store(sample.valid, std::memory_order_relaxed);
        telemetry.targetVelocityValid.store(sample.targetVelocityValid, std::memory_order_relaxed);
        telemetry.contactActive.store(sample.contactActive, std::memory_order_relaxed);
        telemetry.worldContactActive.store(sample.worldContactActive, std::memory_order_relaxed);
        telemetry.recoveryTeleport.store(sample.recoveryTeleport, std::memory_order_relaxed);
        telemetry.sequence.fetch_add(1, std::memory_order_release);  // even: complete sample
    }

    bool DynamicHandCollisionRuntime::readPhysicsTelemetry(
        const ProxySlot& slot,
        PhysicsTelemetrySample& outSample,
        std::uint64_t& outSequence)
    {
        constexpr int kMaxSnapshotAttempts = 4;
        const auto& telemetry = slot.physicsTelemetry;
        for (int attempt = 0; attempt < kMaxSnapshotAttempts; ++attempt) {
            const std::uint64_t begin = telemetry.sequence.load(std::memory_order_acquire);
            if ((begin & 1u) != 0) {
                continue;
            }

            PhysicsTelemetrySample sample{};
            sample.requestedTargetWorldGame = RE::NiPoint3{
                telemetry.requestedX.load(std::memory_order_relaxed),
                telemetry.requestedY.load(std::memory_order_relaxed),
                telemetry.requestedZ.load(std::memory_order_relaxed),
            };
            sample.commandedTargetWorldGame = RE::NiPoint3{
                telemetry.commandedX.load(std::memory_order_relaxed),
                telemetry.commandedY.load(std::memory_order_relaxed),
                telemetry.commandedZ.load(std::memory_order_relaxed),
            };
            sample.liveBodyWorldGame = RE::NiPoint3{
                telemetry.liveX.load(std::memory_order_relaxed),
                telemetry.liveY.load(std::memory_order_relaxed),
                telemetry.liveZ.load(std::memory_order_relaxed),
            };
            sample.targetVelocityWorldGameUnitsPerSecond = RE::NiPoint3{
                telemetry.targetVelocityX.load(std::memory_order_relaxed),
                telemetry.targetVelocityY.load(std::memory_order_relaxed),
                telemetry.targetVelocityZ.load(std::memory_order_relaxed),
            };
            sample.approachSpeedGameUnitsPerSecond = telemetry.approachSpeed.load(std::memory_order_relaxed);
            sample.physicsDeltaSeconds = telemetry.physicsDeltaSeconds.load(std::memory_order_relaxed);
            sample.valid = telemetry.valid.load(std::memory_order_relaxed);
            sample.targetVelocityValid = telemetry.targetVelocityValid.load(std::memory_order_relaxed);
            sample.contactActive = telemetry.contactActive.load(std::memory_order_relaxed);
            sample.worldContactActive = telemetry.worldContactActive.load(std::memory_order_relaxed);
            sample.recoveryTeleport = telemetry.recoveryTeleport.load(std::memory_order_relaxed);

            const std::uint64_t end = telemetry.sequence.load(std::memory_order_acquire);
            if (begin == end && (end & 1u) == 0) {
                outSample = sample;
                outSequence = end;
                return true;
            }
        }

        outSample = {};
        outSequence = 0;
        return false;
    }

    void DynamicHandCollisionRuntime::clearPhysicsContactState(ProxySlot& slot)
    {
        slot.droveThisSubstep = false;
        slot.lastPostSolveDeviationGame = {};
        slot.lastPostSolveDeviationValid = false;
        slot.lastPostSolveContact = false;
        slot.droveTargetVelocityGameUnitsPerSecond = {};
        slot.drovePhysicsDeltaSeconds = 0.0f;
        slot.droveTargetVelocityValid = false;
        slot.droveRecoveryTeleport = false;
        publishPhysicsTelemetry(slot, {});
    }

    void DynamicHandCollisionRuntime::updateHandHaptic(
        HandSlots& handSlots,
        dynamic_hand_collision_telemetry::HandSample& handTelemetry,
        bool authorityAllowsFeedback,
        float deltaSeconds)
    {
        handTelemetry.contactEntrySequence = handSlots.contactEntrySequenceAtomic.load(std::memory_order_acquire);
        handTelemetry.contactEntryApproachSpeedGameUnitsPerSecond =
            handSlots.contactEntryApproachSpeedAtomic.load(std::memory_order_relaxed);
        handTelemetry.entryContactMask = handSlots.contactEntryMaskAtomic.load(std::memory_order_relaxed);

        const dynamic_hand_collision_feedback::ContactPulseConfig config{
            .enabled = true,
            .baseIntensity = dynamic_hand_collision_policy::kHapticBaseIntensity,
            .maxIntensity = dynamic_hand_collision_policy::kHapticMaximumIntensity,
            .speedScale = dynamic_hand_collision_policy::kHapticSpeedScale,
            .minApproachSpeedGameUnitsPerSecond = dynamic_hand_collision_policy::kHapticMinimumApproachSpeedGameUnitsPerSecond,
            .cooldownSeconds = dynamic_hand_collision_policy::kHapticCooldownSeconds,
        };
        const auto decision = dynamic_hand_collision_feedback::updateContactPulse(
            handSlots.hapticState,
            handTelemetry.contactEntrySequence,
            handTelemetry.contactEntryApproachSpeedGameUnitsPerSecond,
            deltaSeconds,
            authorityAllowsFeedback,
            config);
        if (!decision.fire) {
            return;
        }

        auto& pulse = _pendingHapticEvents.hands[handTelemetry.isLeft ? 1u : 0u];
        pulse.fire = true;
        pulse.isLeft = handTelemetry.isLeft;
        pulse.intensity = decision.intensity;
        pulse.approachSpeedGameUnitsPerSecond = decision.approachSpeedGameUnitsPerSecond;
        pulse.contactEntrySequence = decision.entrySequence;
        pulse.contactMask = handTelemetry.entryContactMask;
    }

    bool DynamicHandCollisionRuntime::getTelemetrySnapshot(dynamic_hand_collision_telemetry::Snapshot& outSnapshot) const
    {
        outSnapshot = _telemetrySnapshot;
        return outSnapshot.updateSequence != 0;
    }

    bool DynamicHandCollisionRuntime::tryGetBodyTargetForDebug(
        const bool isLeft,
        const std::size_t bodyIndex,
        RE::NiTransform& outTarget) const
    {
        if (bodyIndex != 0) {
            return false;
        }

        const auto& slot = _hands[isLeft ? 1u : 0u].bodies[0];
        if (!slot.created) {
            return false;
        }
        std::unique_lock targetLock(
            slot.driveState.mutex,
            std::try_to_lock);
        if (!targetLock.owns_lock()) {
            return false;
        }
        if (slot.driveState.hasPendingTarget) {
            outTarget = slot.driveState.pendingTarget;
            return true;
        }
        if (slot.driveState.hasPreviousTarget) {
            outTarget = slot.driveState.previousTarget;
            return true;
        }
        return false;
    }

    dynamic_hand_collision_telemetry::HapticEvents DynamicHandCollisionRuntime::consumeHapticEvents()
    {
        const auto events = _pendingHapticEvents;
        _pendingHapticEvents = {};
        return events;
    }

    bool DynamicHandCollisionRuntime::tryClassifyDynamicBodyContactSourceAtomic(
        const std::uint32_t bodyId,
        const std::uint32_t shapeKey,
        DynamicBodyContactSource& outSource) const noexcept
    {
        outSource = {};
        if (bodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return false;
        }
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const auto& handState = _hands[hand];
            if (handState.bodies[0].bodyIdAtomic.load(
                    std::memory_order_acquire) != bodyId) {
                continue;
            }
            const auto childIndex =
                handState.compoundShape.tryResolveChildIndex(shapeKey);
            if (!childIndex || *childIndex >= kBodiesPerHand) {
                return false;
            }
            outSource.valid = true;
            outSource.isLeft = hand == 1;
            outSource.slot = static_cast<std::uint8_t>(*childIndex);
            outSource.bodyId = bodyId;
            return true;
        }
        return false;
    }

    void DynamicHandCollisionRuntime::recordDynamicBodyContactCallback(
        const DynamicBodyContactSource& source,
        const bool otherIsHand,
        const bool otherIsWeapon) noexcept
    {
        if (!source.valid || source.slot >= kBodiesPerHand) {
            return;
        }
        const std::uint32_t slotBit =
            1u << static_cast<std::uint32_t>(source.slot);
        auto& hand = _hands[source.isLeft ? 1u : 0u];
        hand.pendingSolverContactMaskAtomic.fetch_or(
            slotBit,
            std::memory_order_release);
        if (!otherIsHand && !otherIsWeapon) {
            hand.pendingWorldContactMaskAtomic.fetch_or(
                slotBit,
                std::memory_order_release);
        }
        if (otherIsHand) {
            hand.pendingOtherHandContactMaskAtomic.fetch_or(
                slotBit,
                std::memory_order_release);
        }
        if (otherIsWeapon) {
            hand.pendingWeaponContactMaskAtomic.fetch_or(
                slotBit,
                std::memory_order_release);
        }
    }

    bool DynamicHandCollisionRuntime::tryClassifySurfaceContactSourceAtomic(
        const std::uint32_t bodyId,
        const std::uint32_t shapeKey,
        dynamic_hand_surface_contact_state::ContactSource& outSource) const noexcept
    {
        outSource = {};
        if (bodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return false;
        }

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const auto& handState = _hands[hand];
            if (handState.bodies[0].bodyIdAtomic.load(
                    std::memory_order_acquire) != bodyId) {
                continue;
            }
            const auto childIndex =
                handState.compoundShape.tryResolveChildIndex(shapeKey);
            if (!childIndex || *childIndex >= kBodiesPerHand ||
                !dynamic_hand_collision_telemetry::
                    isSurfaceGrabSourceSlot(*childIndex)) {
                return false;
            }
            const std::size_t slot = *childIndex;
            const bool palm = slot == kPalmSlot;
            const std::size_t fingerIndex =
                dynamic_hand_collision_telemetry::
                    fingerIndexForBodyIndex(slot);
            const auto finger = palm ?
                hand_collider_semantics::HandFinger::None :
                static_cast<hand_collider_semantics::HandFinger>(fingerIndex);
            const auto segment = palm ?
                hand_collider_semantics::HandFingerSegment::None :
                hand_collider_semantics::HandFingerSegment::Tip;
            outSource.valid = true;
            outSource.isLeft = hand == 1;
            outSource.slot = palm ? 0u : fingerIndex + 1u;
            outSource.role = palm ?
                hand_collider_semantics::HandColliderRole::PalmAnchor :
                hand_collider_semantics::roleForFingerSegment(finger, segment);
            outSource.finger = finger;
            outSource.segment = segment;
            outSource.bodyId = bodyId;
            return true;
        }
        return false;
    }

    void DynamicHandCollisionRuntime::recordSurfaceContactCallback(
        const dynamic_hand_surface_contact_state::ContactSource& source,
        const std::uint32_t otherBodyId,
        const hand_semantic_contact_state::SemanticContactVector* contactPointGame,
        const hand_semantic_contact_state::SemanticContactVector* contactNormalGame) noexcept
    {
        _surfaceImpulsePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        _surfaceEligiblePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (_surfaceContacts.record(
                source,
                otherBodyId,
                contactPointGame,
                contactNormalGame)) {
            _surfaceContactPublishSequenceAtomic.fetch_add(1, std::memory_order_release);
        }
    }

    void DynamicHandCollisionRuntime::recordSurfaceManifoldProcessedCallback(
        const dynamic_hand_surface_contact_state::ContactSource& source,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer,
        const hand_semantic_contact_state::SemanticContactVector* contactPointGame,
        const hand_semantic_contact_state::SemanticContactVector* contactNormalGame) noexcept
    {
        _surfaceProcessedPairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead ||
            !collision_layer_policy::isDynamicHandProxySurfaceLayer(otherLayer)) {
            return;
        }
        _surfaceEligiblePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (_surfaceContacts.record(
                source,
                otherBodyId,
                contactPointGame,
                contactNormalGame)) {
            _surfaceContactPublishSequenceAtomic.fetch_add(1, std::memory_order_release);
        }
    }

    hand_semantic_contact_state::SemanticContactCollection
    DynamicHandCollisionRuntime::collectFreshSurfaceContacts(
        const bool isLeft,
        const std::uint32_t maximumAgeFrames,
        const float maximumAgeSeconds) const noexcept
    {
        return _surfaceContacts.collectFresh(isLeft, maximumAgeFrames, maximumAgeSeconds);
    }

    bool DynamicHandCollisionRuntime::beginSurfaceLatch(
        const dynamic_hand_surface_contact_state::ContactSource& source,
        const std::uint32_t targetBodyId,
        RE::hknpWorld* world,
        const SurfaceLatchPresentation* presentation,
        SurfaceLatchFailure* outFailure)
    {
        if (outFailure) {
            *outFailure = SurfaceLatchFailure::None;
        }
        const auto reject = [&](const SurfaceLatchFailure failure) {
            if (outFailure) {
                *outFailure = failure;
            }
            return false;
        };

        const bool isLeft = source.isLeft;
        const std::uint32_t sourceBodyId = source.bodyId;
        if (!frik_visual_authority::isAvailable() ||
            !world || targetBodyId == hand_semantic_contact_state::kInvalidBodyId ||
            sourceBodyId == hand_semantic_contact_state::kInvalidBodyId ||
            sourceBodyId == targetBodyId) {
            return reject(SurfaceLatchFailure::PrerequisiteUnavailable);
        }

        if (!source.valid ||
            source.slot >= dynamic_hand_surface_contact_state::
                kContactRolesPerHand) {
            return reject(SurfaceLatchFailure::ContactSourceMismatch);
        }

        auto& handSlots = _hands[handIndex(isLeft)];
        if (handSlots.surfaceLatch.active ||
            !handSlots.lastPresentedHandWorldValid ||
            !isFiniteTransform(handSlots.lastPresentedHandWorld)) {
            return reject(SurfaceLatchFailure::HandTransformUnavailable);
        }

        const auto targetSnapshot = havok_runtime::snapshotBody(
            world,
            RE::hknpBodyId{ targetBodyId });
        RE::NiTransform targetWorld{};
        if (!targetSnapshot.valid || !targetSnapshot.body ||
            !havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                RE::hknpBodyId{ targetBodyId },
                targetWorld) ||
            !isFiniteTransform(targetWorld)) {
            return reject(SurfaceLatchFailure::TargetTransformUnavailable);
        }

        HandSlots::SurfaceLatch candidate{};
        candidate.targetBodyId = targetBodyId;
        candidate.targetBodyIdentity = targetSnapshot.body;
        candidate.targetCollisionIdentity = targetSnapshot.collisionObject;
        const bool meshPresentationRequested =
            presentation && presentation->valid &&
            isFiniteTransform(presentation->handWorld) &&
            isFinitePoint(presentation->meshAnchorWorld) &&
            isFinitePoint(presentation->meshNormalWorld) &&
            std::isfinite(
                presentation->shellToMeshDistanceGameUnits) &&
            presentation->shellToMeshDistanceGameUnits >= 0.0f;
        candidate.lastHandWorld = meshPresentationRequested ?
            presentation->handWorld :
            handSlots.lastPresentedHandWorld;
        const auto targetInverse = transform_math::invertTransform(targetWorld);
        candidate.handInTargetBody = transform_math::composeTransforms(
            targetInverse,
            candidate.lastHandWorld);

        bool sourceCaptured = false;
        /*
         * Mesh authority moves only the presented hand. The compound children keep
         * their already-solved shell transforms relative to the target, so the
         * oversized Havok shell remains a collision guard without becoming the
         * visible/API grab anchor. Translating the twins into the render mesh
         * would require body-wide no-collide and would discard nearby-world
         * collision fidelity while latched.
         */
        const auto& compoundOwner = handSlots.bodies[0];
        RE::NiTransform liveCompoundWorld{};
        if (!compoundOwner.created || compoundOwner.createdWorld != world ||
            compoundOwner.body.getBodyId().value != sourceBodyId ||
            !havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                compoundOwner.body.getBodyId(),
                liveCompoundWorld) ||
            !isFiniteTransform(liveCompoundWorld)) {
            return reject(SurfaceLatchFailure::SourceProxyUnavailable);
        }
        std::array<RE::NiTransform, kBodiesPerHand> childFrames{};
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            childFrames = handSlots.consumedChildInCompound;
        }
        const RE::NiTransform liveCompoundSceneWorld =
            colliderFrameToSceneFrame(liveCompoundWorld);
        for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
            const RE::NiTransform proxyWorld =
                transform_math::composeTransforms(
                    liveCompoundSceneWorld,
                    childFrames[bodyIndex]);
            if (!isFiniteTransform(proxyWorld)) {
                continue;
            }
            candidate.lastProxyWorld[bodyIndex] = proxyWorld;
            candidate.proxyInTargetBody[bodyIndex] =
                transform_math::composeTransforms(
                    targetInverse,
                    proxyWorld);
            candidate.proxyRelationshipValid[bodyIndex] = true;
            sourceCaptured = true;
        }
        if (!sourceCaptured || !isFiniteTransform(candidate.handInTargetBody)) {
            return reject(SurfaceLatchFailure::SourceProxyUnavailable);
        }

        const bool meshAuthorityAccepted = meshPresentationRequested;

        if (meshAuthorityAccepted) {
            candidate.meshAuthoritative = true;
            candidate.meshAnchorWorld = presentation->meshAnchorWorld;
            candidate.meshNormalWorld = presentation->meshNormalWorld;
            candidate.meshAnchorInTargetBody =
                transform_math::worldPointToLocal(
                    targetWorld,
                    presentation->meshAnchorWorld);
            candidate.meshNormalInTargetBody =
                transform_math::worldVectorToLocal(
                    targetWorld,
                    presentation->meshNormalWorld);
            candidate.shellToMeshDistanceGameUnits =
                presentation->shellToMeshDistanceGameUnits;
            candidate.meshFingerJointValues =
                presentation->fingerJointValues;
            candidate.meshFingerContactMask =
                presentation->fingerContactMask;
            candidate.meshFingerPoseValid =
                presentation->fingerPoseValid;
            clearSurfaceFingerResponse(handSlots, isLeft);
        }

        candidate.active = true;
        handSlots.surfaceLatch = candidate;
        handSlots.appliedDeviation = {};
        handSlots.teleportRecoverySecondsRemaining = 0.0f;
        ROCK_LOG_INFO(
            Hand,
            "Dynamic surface latch acquired: hand={} sourceBody={} targetBody={} authority={} gap={:.2f}gu fingerPose={} contactMask=0x{:02X}",
            isLeft ? "left" : "right",
            sourceBodyId,
            targetBodyId,
            meshAuthorityAccepted ? "mesh" : "collision",
            candidate.shellToMeshDistanceGameUnits,
            candidate.meshFingerPoseValid ? "yes" : "no",
            candidate.meshFingerContactMask);
        return true;
    }

    void DynamicHandCollisionRuntime::endSurfaceLatch(
        const bool isLeft) noexcept
    {
        auto& handSlots = _hands[handIndex(isLeft)];
        if (!handSlots.surfaceLatch.active) {
            return;
        }
        const auto targetBodyId = handSlots.surfaceLatch.targetBodyId;
        const bool meshAuthoritative =
            handSlots.surfaceLatch.meshAuthoritative;
        clearSurfaceMeshPose(handSlots, isLeft);
        handSlots.surfaceLatch = {};
        handSlots.appliedDeviation = {};
        handSlots.teleportRecoverySecondsRemaining = 0.0f;
        if (handSlots.visualActive) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(
                dynamicHandTag(isLeft),
                frik_visual_authority::handFromBool(isLeft));
            handSlots.visualActive = false;
        }
        ROCK_LOG_INFO(
            Hand,
            "Dynamic surface latch released: hand={} targetBody={} authority={}",
            isLeft ? "left" : "right",
            targetBodyId,
            meshAuthoritative ? "mesh" : "collision");
    }

    bool DynamicHandCollisionRuntime::isSurfaceLatchActive(
        const bool isLeft) const noexcept
    {
        return _hands[handIndex(isLeft)].surfaceLatch.active;
    }

    bool DynamicHandCollisionRuntime::isSurfaceLatchMeshAuthoritative(
        const bool isLeft) const noexcept
    {
        const auto& latch = _hands[handIndex(isLeft)].surfaceLatch;
        return latch.active && latch.meshAuthoritative;
    }

    bool DynamicHandCollisionRuntime::isSurfaceLatchMeshFingerPoseActive(
        const bool isLeft) const noexcept
    {
        const auto& latch = _hands[handIndex(isLeft)].surfaceLatch;
        return latch.active && latch.meshAuthoritative &&
               latch.meshFingerPoseValid && latch.meshPosePublished &&
               frik_visual_authority::isHandPoseTagActive(
                   SURFACE_MESH_GRAB_POSE_TAG,
                   frik_visual_authority::handFromBool(isLeft));
    }

    bool DynamicHandCollisionRuntime::getLastPresentedHandWorld(
        const bool isLeft,
        RE::NiTransform& outHandWorld) const noexcept
    {
        const auto& handSlots = _hands[handIndex(isLeft)];
        if (!handSlots.lastPresentedHandWorldValid ||
            !isFiniteTransform(handSlots.lastPresentedHandWorld)) {
            return false;
        }
        outHandWorld = handSlots.lastPresentedHandWorld;
        return true;
    }

    bool DynamicHandCollisionRuntime::ensureHandCreated(
        HandSlots& handSlots,
        const bool isLeft,
        const PhysicsFrameContext& frame,
        const Hand& hand,
        const BodyBoneColliderSet& bodyBoneColliders,
        const std::array<const dynamic_hand_twin::TwinSlotFrame*,
            kBodiesPerHand>& twinFrames,
        const RE::NiTransform& compoundRootTarget,
        const std::array<RE::NiTransform, kBodiesPerHand>& driveTargets,
        const std::uint64_t geometryGeneration)
    {
        auto& slot = handSlots.bodies[0];
        if (slot.created) {
            if (slot.createdWorld == frame.hknpWorld) {
                if (_transitionCollisionSuppressed ||
                    (slot.createdGeometryGeneration == geometryGeneration &&
                        !slot.rebuildRequestedAtomic.load(
                            std::memory_order_acquire))) {
                    return true;
                }
            }
            retireHand(handSlots, frame.bhkWorld, isLeft);
        }

        if (!frame.hknpWorld || !frame.bhkWorld || geometryGeneration == 0 ||
            !isFiniteTransform(compoundRootTarget)) {
            return false;
        }
        for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
            if (!twinFrames[child] || !twinFrames[child]->valid ||
                !isFiniteTransform(driveTargets[child])) {
                return false;
            }
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        std::array<RE::hknpShape*, kBodiesPerHand> childShapes{};
        std::array<havok_compound_shape_builder::CompoundChild,
            kBodiesPerHand> compoundChildren{};
        const RE::NiTransform rootInverse =
            transform_math::invertTransform(
                colliderFrameToSceneFrame(compoundRootTarget));
        bool childrenValid = true;
        RE::NiPoint3 boundsMin{};
        RE::NiPoint3 boundsMax{};
        for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
            const auto& childFrame = *twinFrames[child];
            childShapes[child] = child >= kFirstForearmSlot ?
                bodyBoneColliders.buildDynamicForearmTwinShape(childFrame) :
                hand.buildDynamicTwinShape(childFrame, child == kPalmSlot);

            RE::NiTransform childLocal = transform_math::composeTransforms(
                rootInverse,
                colliderFrameToSceneFrame(driveTargets[child]));
            childLocal.scale = 1.0f;
            if (!childShapes[child] ||
                !makeCompoundChildTransform(
                    childLocal,
                    compoundChildren[child].transform)) {
                childrenValid = false;
                break;
            }
            compoundChildren[child].shape = childShapes[child];
            handSlots.childInCompound[child] = childLocal;

            const float childReach =
                (std::isfinite(childFrame.length) ?
                    std::max(childFrame.length, 0.0f) * 0.5f :
                    0.0f) +
                (std::isfinite(childFrame.radius) ?
                    std::max(childFrame.radius, 0.0f) :
                    0.0f) +
                (std::isfinite(childFrame.convexRadius) ?
                    std::max(childFrame.convexRadius, 0.0f) :
                    0.0f);
            const RE::NiPoint3 childMin{
                childLocal.translate.x - childReach,
                childLocal.translate.y - childReach,
                childLocal.translate.z - childReach,
            };
            const RE::NiPoint3 childMax{
                childLocal.translate.x + childReach,
                childLocal.translate.y + childReach,
                childLocal.translate.z + childReach,
            };
            if (child == 0) {
                boundsMin = childMin;
                boundsMax = childMax;
            } else {
                boundsMin.x = std::min(boundsMin.x, childMin.x);
                boundsMin.y = std::min(boundsMin.y, childMin.y);
                boundsMin.z = std::min(boundsMin.z, childMin.z);
                boundsMax.x = std::max(boundsMax.x, childMax.x);
                boundsMax.y = std::max(boundsMax.y, childMax.y);
                boundsMax.z = std::max(boundsMax.z, childMax.z);
            }
        }

        if (!childrenValid ||
            !handSlots.compoundShape.create(compoundChildren)) {
            for (auto* childShape : childShapes) {
                if (childShape) {
                    havok_ref_count::release(childShape);
                }
            }
            handSlots.compoundShape.reset();
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound child construction failed generation={}",
                isLeft ? "Left" : "Right",
                geometryGeneration);
            return false;
        }
        for (auto* childShape : childShapes) {
            havok_ref_count::release(childShape);
        }

        const std::uint32_t expectedFilterInfo = dynamicHandProxyFilterInfo(
            isLeft,
            _transitionCollisionSuppressed ||
                _weaponOwnershipCollisionSuppressed[handIndex(isLeft)]);
        if (!slot.body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                handSlots.compoundShape.get(),
                expectedFilterInfo,
                havok_material_registry::registerGeneratedBodyMaterial(
                    frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                isLeft ?
                    "ROCK_LeftHandDynamicCompound" :
                    "ROCK_RightHandDynamicCompound",
                kTrackedDynamicBodyCreationOptions)) {
            handSlots.compoundShape.reset();
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound body creation failed",
                isLeft ? "Left" : "Right");
            return false;
        }

        const RE::hknpBodyId bodyId = slot.body.getBodyId();
        const auto createdBody = havok_runtime::snapshotBody(
            frame.hknpWorld,
            bodyId);
        if (!createdBody.valid ||
            createdBody.collisionFilterInfo != expectedFilterInfo) {
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            return false;
        }

        const bool processedManifoldFlagEnabled =
            havok_runtime::enableBodyFlags(
                frame.hknpWorld,
                bodyId.value,
                kRaiseManifoldProcessedEvents,
                kRebuildBodyCollisionState);
        const auto flaggedBody = havok_runtime::snapshotBody(
            frame.hknpWorld,
            bodyId);
        if (!processedManifoldFlagEnabled || !flaggedBody.valid ||
            !flaggedBody.body ||
            (flaggedBody.body->flags & kRaiseManifoldProcessedEvents) !=
                kRaiseManifoldProcessedEvents) {
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand compound failed processed-manifold opt-in body={}",
                isLeft ? "Left" : "Right",
                bodyId.value);
            return false;
        }

        if (!applyHandCompoundMassProperties(
                frame.hknpWorld,
                bodyId,
                isLeft,
                boundsMin,
                boundsMax) ||
            !placeGeneratedKeyframedBodyImmediately(
                slot.body,
                compoundRootTarget)) {
            slot.body.retireDeferred(frame.bhkWorld);
            handSlots.compoundShape.reset();
            return false;
        }

        slot.shape = nullptr;
        slot.createdWorld = frame.hknpWorld;
        slot.createdBhkWorld = frame.bhkWorld;
        slot.createdGeometryGeneration = geometryGeneration;
        slot.created = true;
        for (auto& semanticSlot : handSlots.bodies) {
            semanticSlot.bodyIdAtomic.store(
                bodyId.value,
                std::memory_order_release);
            clearPhysicsContactState(semanticSlot);
        }
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
        initializeGeneratedKeyframedBodyDriveState(
            slot.driveState,
            compoundRootTarget);
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
                handSlots.pendingCompoundChildTransforms[child] =
                    compoundChildren[child].transform;
                handSlots.consumedChildInCompound[child] =
                    handSlots.childInCompound[child];
            }
            handSlots.queuedCompoundPoseSequence = 1;
            handSlots.consumedCompoundPoseSequence = 1;
        }
        handSlots.compoundGeometryGeneration = geometryGeneration;

        ROCK_LOG_INFO(
            Hand,
            "{} animated dynamic hand compound created: body={} children={} keyBits={} generation={} layer={} group={}",
            isLeft ? "Left" : "Right",
            bodyId.value,
            handSlots.compoundShape.childCount(),
            handSlots.compoundShape.shapeKeyBitCount(),
            geometryGeneration,
            collision_layer_policy::dynamicHandProxyLayerForHand(isLeft),
            expectedFilterInfo >> 16);
        return true;
    }

    bool DynamicHandCollisionRuntime::queueCompoundPose(
        HandSlots& handSlots,
        const RE::NiTransform& compoundRootTarget,
        const std::array<RE::NiTransform, kBodiesPerHand>& driveTargets)
    {
        if (!handSlots.bodies[0].created ||
            !isFiniteTransform(compoundRootTarget)) {
            return false;
        }

        const RE::NiTransform rootInverse =
            transform_math::invertTransform(
                colliderFrameToSceneFrame(compoundRootTarget));
        std::array<havok_compound_shape_builder::ChildTransform,
            kBodiesPerHand> pendingTransforms{};
        std::array<RE::NiTransform, kBodiesPerHand> childFrames{};
        for (std::size_t child = 0; child < kBodiesPerHand; ++child) {
            RE::NiTransform childLocal = transform_math::composeTransforms(
                rootInverse,
                colliderFrameToSceneFrame(driveTargets[child]));
            childLocal.scale = 1.0f;
            if (!makeCompoundChildTransform(
                    childLocal,
                    pendingTransforms[child])) {
                return false;
            }
            childFrames[child] = childLocal;
        }

        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            handSlots.pendingCompoundChildTransforms = pendingTransforms;
            handSlots.childInCompound = childFrames;
            ++handSlots.queuedCompoundPoseSequence;
        }
        return true;
    }

    void DynamicHandCollisionRuntime::retireSlot(ProxySlot& slot, void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        slot.bodyIdAtomic.store(
            hand_semantic_contact_state::kInvalidBodyId,
            std::memory_order_release);
        if (slot.created) {
            /*
             * Live-world teardown must go through the deferred-retirement queue:
             * the hknp broadphase can still reach this body until the next
             * physics step (see 2026-07-08 use-after-free lesson).
             */
            slot.body.retireDeferred(bhkWorld ? bhkWorld : slot.createdBhkWorld);
        }
        if (slot.shape) {
            havok_ref_count::release(slot.shape);
            slot.shape = nullptr;
        }
        clearGeneratedKeyframedBodyDriveState(slot.driveState);
        slot.created = false;
        slot.createdWorld = nullptr;
        slot.createdBhkWorld = nullptr;
        slot.createdGeometryGeneration = 0;
        slot.droveThisSubstep = false;
        slot.divergenceDwellSeconds = 0.0f;
        slot.commandedTargetWorld = {};
        slot.requestedTargetWorld = {};
        slot.commandedTargetGame = {};
        slot.requestedTargetGame = {};
        clearPhysicsContactState(slot);
        slot.teleportedAtomic.store(false, std::memory_order_release);
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
    }

    void DynamicHandCollisionRuntime::retireHand(HandSlots& handSlots, void* bhkWorld, bool isLeft)
    {
        clearSurfaceMeshPose(handSlots, isLeft);
        clearVisual(handSlots, isLeft);
        clearSurfaceFingerResponse(handSlots, isLeft);
        for (auto& slot : handSlots.bodies) {
            retireSlot(slot, bhkWorld);
        }
        {
            std::scoped_lock poseLock(handSlots.compoundPoseMutex);
            handSlots.compoundShape.reset();
            handSlots.pendingCompoundChildTransforms = {};
            handSlots.childInCompound = {};
            handSlots.consumedChildInCompound = {};
            handSlots.queuedCompoundPoseSequence = 0;
            handSlots.consumedCompoundPoseSequence = 0;
        }
        handSlots.compoundGeometryGeneration = 0;
        handSlots.pendingSolverContactMaskAtomic.store(
            0,
            std::memory_order_release);
        handSlots.pendingWorldContactMaskAtomic.store(
            0,
            std::memory_order_release);
        handSlots.retainedSolverContactMask = 0;
        handSlots.retainedWorldContactMask = 0;
        handSlots.solverContactRetentionSeconds.fill(0.0f);
        handSlots.worldContactRetentionSeconds.fill(0.0f);
        handSlots.physicsContactActive = false;
        handSlots.contactEntrySequenceAtomic.store(0, std::memory_order_release);
        handSlots.contactEntryApproachSpeedAtomic.store(0.0f, std::memory_order_relaxed);
        handSlots.contactEntryMaskAtomic.store(0, std::memory_order_relaxed);
        handSlots.hapticState = {};
        handSlots.lastPresentedHandWorld = {};
        handSlots.lastPresentedHandWorldValid = false;
        handSlots.surfaceLatch = {};
    }

    void DynamicHandCollisionRuntime::clearVisual(HandSlots& handSlots, bool isLeft)
    {
        handSlots.appliedDeviation = {};
        handSlots.teleportRecoverySecondsRemaining = 0.0f;
        if (!handSlots.visualActive) {
            return;
        }
        (void)frik_visual_authority::clearExternalHandWorldTransform(dynamicHandTag(isLeft), frik_visual_authority::handFromBool(isLeft));
        handSlots.visualActive = false;
    }

    void DynamicHandCollisionRuntime::clearSurfaceFingerResponse(
        HandSlots& handSlots,
        const bool isLeft)
    {
        if (handSlots.surfaceFingerResponse.posePublished) {
            (void)frik_visual_authority::clearHandPose(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft));
        }
        handSlots.surfaceFingerResponse = {};
    }

    void DynamicHandCollisionRuntime::clearSurfaceMeshPose(
        HandSlots& handSlots,
        const bool isLeft)
    {
        if (handSlots.surfaceLatch.meshPosePublished) {
            (void)frik_visual_authority::clearHandPose(
                SURFACE_MESH_GRAB_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft));
        }
        handSlots.surfaceLatch.meshPosePublished = false;
    }

    bool DynamicHandCollisionRuntime::captureSurfaceFingerResponse(
        HandSlots& handSlots,
        const bool isLeft,
        const HandState handState,
        const RE::NiTransform& rawHandWorld,
        const dynamic_hand_twin::TwinTargets& handTwins)
    {
        if (!isFiniteTransform(rawHandWorld) ||
            !frik_visual_authority::isAvailable()) {
            return false;
        }

        root_flattened_finger_skeleton_runtime::Snapshot liveSnapshot{};
        if (!root_flattened_finger_skeleton_runtime::
                resolveLiveFingerSkeletonSnapshot(isLeft, liveSnapshot) ||
            !liveSnapshot.valid) {
            return false;
        }

        HandSlots::SurfaceFingerResponse candidate{};
        const float selectedCloseValue = std::clamp(
            g_rockConfig.rockSelectedCloseFingerAnimValue,
            0.0f,
            1.0f);
        const float fallbackOpenValue = handState == HandState::SelectedClose ?
            selectedCloseValue :
            1.0f;
        candidate.baselineOpenValues.fill(fallbackOpenValue);
        for (std::size_t finger = 1;
             finger < hand_collider_semantics::kHandFingerCount;
             ++finger) {
            const auto& chain = liveSnapshot.fingers[finger];
            if (!chain.valid) {
                return false;
            }
            const float fingerLength =
                root_flattened_finger_skeleton_runtime::distance(
                    chain.points[0],
                    chain.points[1]) +
                root_flattened_finger_skeleton_runtime::distance(
                    chain.points[1],
                    chain.points[2]);
            const float chordLength =
                root_flattened_finger_skeleton_runtime::distance(
                    chain.points[0],
                    chain.points[2]);
            const auto estimate =
                grab_finger_pose_math::estimateCalibratedChainCurlFromChord(
                    finger,
                    isLeft,
                    liveSnapshot.inPowerArmor,
                    fingerLength,
                    chordLength);
            if (estimate.valid && std::isfinite(estimate.openValue)) {
                candidate.baselineOpenValues[finger] = std::clamp(
                    estimate.openValue,
                    0.0f,
                    1.0f);
            }
        }

        const auto policyConfig =
            surface_finger_collision_policy::sanitize(
                surface_finger_collision_policy::Config{
                    .probeDeltaOpenUnits =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerProbeDeltaOpenUnits,
                    .responseGain =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerResponseGain,
                    .maximumDeflectionOpenUnits =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerMaximumDeflectionOpenUnits,
                    .minimumHelpfulProbeTravelGameUnits =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerMinimumHelpfulTravelGameUnits,
                    .directionSwitchHysteresisFraction =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerDirectionSwitchHysteresisFraction,
                });
        auto closingProbeOpenValues = candidate.baselineOpenValues;
        auto openingProbeOpenValues = candidate.baselineOpenValues;
        for (std::size_t finger = 0;
             finger < hand_collider_semantics::kHandFingerCount;
             ++finger) {
            closingProbeOpenValues[finger] = std::max(
                0.0f,
                closingProbeOpenValues[finger] -
                    policyConfig.probeDeltaOpenUnits);
            openingProbeOpenValues[finger] = std::min(
                1.0f,
                openingProbeOpenValues[finger] +
                    policyConfig.probeDeltaOpenUnits);
        }

        const auto baselineJointValues =
            grab_finger_pose_math::expandFingerCurlsToJointValues(
                candidate.baselineOpenValues);
        const auto closingProbeJointValues =
            grab_finger_pose_math::expandFingerCurlsToJointValues(
                closingProbeOpenValues);
        const auto openingProbeJointValues =
            grab_finger_pose_math::expandFingerCurlsToJointValues(
                openingProbeOpenValues);
        frik_visual_authority::FingerLocalTransformOverride baselineLocals{};
        frik_visual_authority::FingerLocalTransformOverride
            closingProbeLocals{};
        frik_visual_authority::FingerLocalTransformOverride
            openingProbeLocals{};
        const auto hand = frik_visual_authority::handFromBool(isLeft);
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(
                hand,
                frik_visual_authority::makeHandPoseDataFromJointValues(
                    baselineJointValues),
                &baselineLocals) ||
            !frik_visual_authority::getHandPoseLocalTransformsForPose(
                hand,
                frik_visual_authority::makeHandPoseDataFromJointValues(
                    closingProbeJointValues),
                &closingProbeLocals) ||
            !frik_visual_authority::getHandPoseLocalTransformsForPose(
                hand,
                frik_visual_authority::makeHandPoseDataFromJointValues(
                    openingProbeJointValues),
                &openingProbeLocals)) {
            return false;
        }

        std::array<RE::NiPoint3,
            hand_collider_semantics::kHandFingerRoleCount>
            baselineCenters{};
        std::array<RE::NiPoint3,
            hand_collider_semantics::kHandFingerRoleCount>
            closingProbeCenters{};
        std::array<RE::NiPoint3,
            hand_collider_semantics::kHandFingerRoleCount>
            openingProbeCenters{};
        if (!buildPoseFingerSegmentCenters(
                rawHandWorld,
                handTwins,
                baselineLocals,
                baselineCenters) ||
            !buildPoseFingerSegmentCenters(
                rawHandWorld,
                handTwins,
                closingProbeLocals,
                closingProbeCenters) ||
            !buildPoseFingerSegmentCenters(
                rawHandWorld,
                handTwins,
                openingProbeLocals,
                openingProbeCenters)) {
            return false;
        }

        for (std::size_t finger = 0;
             finger < hand_collider_semantics::kHandFingerCount;
             ++finger) {
            for (std::size_t segment = 0;
                 segment < hand_collider_semantics::kHandFingerSegmentCount;
                 ++segment) {
                const std::size_t linearIndex =
                    finger * hand_collider_semantics::kHandFingerSegmentCount +
                    segment;
                const auto& twin = handTwins.fingers[finger][segment];
                if (!twin.valid || !isFiniteTransform(twin.target)) {
                    return false;
                }
                candidate.closingProbeTravelInHand[linearIndex] =
                    transform_math::worldVectorToLocal(
                        rawHandWorld,
                        closingProbeCenters[linearIndex] -
                            baselineCenters[linearIndex]);
                candidate.openingProbeTravelInHand[linearIndex] =
                    transform_math::worldVectorToLocal(
                        rawHandWorld,
                        openingProbeCenters[linearIndex] -
                            baselineCenters[linearIndex]);
                if (!isFinitePoint(
                        candidate.closingProbeTravelInHand[linearIndex]) ||
                    !isFinitePoint(
                        candidate.openingProbeTravelInHand[linearIndex])) {
                    return false;
                }
            }
        }

        candidate.currentOpenValues = candidate.baselineOpenValues;
        candidate.active = true;
        handSlots.surfaceFingerResponse = candidate;
        ROCK_LOG_SAMPLE_DEBUG(
            Hand,
            2000,
            "{} surface finger intent captured: open=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) powerArmor={}",
            isLeft ? "Left" : "Right",
            candidate.baselineOpenValues[0],
            candidate.baselineOpenValues[1],
            candidate.baselineOpenValues[2],
            candidate.baselineOpenValues[3],
            candidate.baselineOpenValues[4],
            liveSnapshot.inPowerArmor ? "yes" : "no");
        return true;
    }

    std::uint32_t DynamicHandCollisionRuntime::updateSurfaceFingerResponse(
        HandSlots& handSlots,
        const bool isLeft,
        const HandState handState,
        const RE::NiTransform& rawHandWorld,
        const dynamic_hand_twin::TwinTargets& handTwins,
        const dynamic_hand_collision_telemetry::HandSample& handTelemetry,
        const float deltaSeconds,
        const bool freezeCurrentPose)
    {
        bool anyFingerContact = false;
        for (std::size_t bodyIndex =
                 dynamic_hand_collision_telemetry::kFirstFingerSlot;
             bodyIndex < kFirstForearmSlot;
             ++bodyIndex) {
            anyFingerContact = anyFingerContact ||
                               handTelemetry.twins[bodyIndex].worldContactActive;
        }
        if (!handSlots.surfaceFingerResponse.active) {
            if (!anyFingerContact) {
                return 0;
            }
            if (!captureSurfaceFingerResponse(
                    handSlots,
                    isLeft,
                    handState,
                    rawHandWorld,
                    handTwins)) {
                ROCK_LOG_SAMPLE_WARN(
                    Hand,
                    2000,
                    "{} surface finger intent capture unavailable",
                    isLeft ? "Left" : "Right");
                return 0;
            }
        }

        auto& response = handSlots.surfaceFingerResponse;
        std::array<std::array<
                       surface_finger_collision_policy::SegmentContact,
                       surface_finger_collision_policy::kSegmentCount>,
            surface_finger_collision_policy::kFingerCount>
            contacts{};
        if (!freezeCurrentPose) {
            for (std::size_t finger = 0;
                 finger < hand_collider_semantics::kHandFingerCount;
                 ++finger) {
                for (std::size_t segment = 0;
                     segment < hand_collider_semantics::kHandFingerSegmentCount;
                     ++segment) {
                    const std::size_t bodyIndex =
                        dynamic_hand_collision_telemetry::
                            bodyIndexForFingerSegment(finger, segment);
                    const auto& twin = handTelemetry.twins[bodyIndex];
                    if (!twin.worldContactActive ||
                        !std::isfinite(twin.contactDeviationGameUnits) ||
                        twin.contactDeviationGameUnits <= 0.0f) {
                        continue;
                    }
                    const float inverseDepth =
                        1.0f / twin.contactDeviationGameUnits;
                    const RE::NiPoint3 safeDirection{
                        twin.contactDeviationWorldGame.x * inverseDepth,
                        twin.contactDeviationWorldGame.y * inverseDepth,
                        twin.contactDeviationWorldGame.z * inverseDepth,
                    };
                    const std::size_t linearIndex =
                        finger *
                            hand_collider_semantics::kHandFingerSegmentCount +
                        segment;
                    const RE::NiPoint3 closingProbeTravelWorld =
                        transform_math::localVectorToWorld(
                            rawHandWorld,
                            response.closingProbeTravelInHand[linearIndex]);
                    const RE::NiPoint3 openingProbeTravelWorld =
                        transform_math::localVectorToWorld(
                            rawHandWorld,
                            response.openingProbeTravelInHand[linearIndex]);
                    contacts[finger][segment] =
                        surface_finger_collision_policy::SegmentContact{
                            .blockedDepthGameUnits =
                                twin.contactDeviationGameUnits,
                            .closingProbeTravelGameUnits =
                                dotPoints(
                                    closingProbeTravelWorld,
                                    safeDirection),
                            .openingProbeTravelGameUnits =
                                dotPoints(
                                    openingProbeTravelWorld,
                                    safeDirection),
                            .active = true,
                        };
                }
            }
        }

        const auto solve = freezeCurrentPose ?
            surface_finger_collision_policy::SolveResult{
                .targetOpenValues = response.currentOpenValues,
            } :
            surface_finger_collision_policy::solve(
                response.baselineOpenValues,
                contacts,
                response.lastDirections,
                surface_finger_collision_policy::Config{
                    .probeDeltaOpenUnits =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerProbeDeltaOpenUnits,
                    .responseGain =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerResponseGain,
                    .maximumDeflectionOpenUnits =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerMaximumDeflectionOpenUnits,
                    .minimumHelpfulProbeTravelGameUnits =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerMinimumHelpfulTravelGameUnits,
                    .directionSwitchHysteresisFraction =
                        dynamic_hand_collision_policy::
                            kSurfaceFingerDirectionSwitchHysteresisFraction,
                });
        const auto previousDirections = response.lastDirections;
        if (!freezeCurrentPose) {
            response.lastDirections = solve.directions;
        }
        const auto& targetOpenValues = anyFingerContact || freezeCurrentPose ?
            solve.targetOpenValues :
            response.baselineOpenValues;
        constexpr std::uint32_t kFingerSlotMask =
            ((1u << static_cast<std::uint32_t>(kFirstForearmSlot)) - 1u) &
            ~((1u << static_cast<std::uint32_t>(
                   dynamic_hand_collision_telemetry::kFirstFingerSlot)) - 1u);
        const bool dynamicInteractionFingerContact =
            ((handTelemetry.otherHandContactMask |
                 handTelemetry.weaponContactMask) &
                kFingerSlotMask) != 0;
        response.currentOpenValues =
            surface_finger_collision_policy::advanceOpenValues(
                response.currentOpenValues,
                targetOpenValues,
                dynamicInteractionFingerContact ?
                    0.0f :
                    dynamic_hand_collision_policy::
                        kSurfaceFingerSmoothingSpeed,
                deltaSeconds);
        if (response.lastDirections != previousDirections) {
            ROCK_LOG_SAMPLE_DEBUG(
                Hand,
                1000,
                "{} surface finger directions: thumb={} index={} middle={} ring={} pinky={} (-1 close, +1 open)",
                isLeft ? "Left" : "Right",
                static_cast<int>(response.lastDirections[0]),
                static_cast<int>(response.lastDirections[1]),
                static_cast<int>(response.lastDirections[2]),
                static_cast<int>(response.lastDirections[3]),
                static_cast<int>(response.lastDirections[4]));
        }

        response.lastHelpfulDynamicSlotMask = 0;
        for (std::size_t linearIndex = 0;
             linearIndex < hand_collider_semantics::kHandFingerRoleCount;
             ++linearIndex) {
            if ((solve.helpfulSegmentMask & (1u << linearIndex)) != 0) {
                response.lastHelpfulDynamicSlotMask |=
                    1u << static_cast<std::uint32_t>(
                        dynamic_hand_collision_telemetry::kFirstFingerSlot +
                        linearIndex);
            }
        }

        if (!anyFingerContact && !freezeCurrentPose) {
            const float dt = std::clamp(
                std::isfinite(deltaSeconds) && deltaSeconds > 0.0f ?
                    deltaSeconds :
                    0.0f,
                0.0f,
                0.1f);
            response.noContactSeconds += dt;
            if (response.noContactSeconds >=
                dynamic_hand_collision_policy::
                    kSurfaceFingerReleaseDelaySeconds) {
                clearSurfaceFingerResponse(handSlots, isLeft);
                return 0;
            }
        } else {
            response.noContactSeconds = 0.0f;
        }

        const auto jointValues =
            grab_finger_pose_math::expandFingerCurlsToJointValues(
                response.currentOpenValues);
        if (!frik_visual_authority::setHandPoseCustomWithPriority(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft),
                frik_visual_authority::makeHandPoseDataFromJointValues(
                    jointValues),
                dynamic_hand_collision_policy::kVisualPriority)) {
            clearSurfaceFingerResponse(handSlots, isLeft);
            return 0;
        }
        response.posePublished = true;
        if (!frik_visual_authority::isHandPoseTagActive(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft))) {
            /*
             * Another tagged pose won hFRIK arbitration. Keep the response
             * state alive so the claim can recover when authority returns,
             * but do not report curl as physical resolution while it is not
             * driving the rendered fingers.
             */
            response.lastHelpfulDynamicSlotMask = 0;
        }
        return response.lastHelpfulDynamicSlotMask;
    }

    void DynamicHandCollisionRuntime::applyWeaponOwnershipCollisionSuppression(
        RE::hknpWorld* world,
        bool rightHandWeaponOwned,
        bool leftHandWeaponOwned)
    {
        const std::array<bool, 2> desiredSuppression{
            rightHandWeaponOwned,
            leftHandWeaponOwned,
        };
        if (_weaponOwnershipCollisionSuppressed == desiredSuppression) {
            return;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        for (std::size_t handIndexValue = 0;
             handIndexValue < _hands.size();
             ++handIndexValue) {
            if (_weaponOwnershipCollisionSuppressed[handIndexValue] ==
                desiredSuppression[handIndexValue]) {
                continue;
            }

            _weaponOwnershipCollisionSuppressed[handIndexValue] =
                desiredSuppression[handIndexValue];
            auto& handSlots = _hands[handIndexValue];
            auto& owner = handSlots.bodies[0];
            if (owner.created && owner.body.isValid() &&
                owner.createdWorld == world) {
                owner.body.setCollisionFilterInfo(
                    dynamicHandProxyFilterInfo(
                        handIndexValue == 1,
                        _transitionCollisionSuppressed ||
                            desiredSuppression[handIndexValue]),
                    1);
            }
            for (auto& slot : handSlots.bodies) {
                clearPhysicsContactState(slot);
            }
            handSlots.pendingOtherHandContactMaskAtomic.store(
                0,
                std::memory_order_release);
            handSlots.pendingWeaponContactMaskAtomic.store(
                0,
                std::memory_order_release);
            handSlots.otherHandContactMask = 0;
            handSlots.weaponContactMask = 0;
            handSlots.otherHandContactGraceFrames = 0;
            handSlots.weaponContactGraceFrames = 0;
        }

        ROCK_LOG_INFO(
            Hand,
            "Dynamic hand weapon ownership collision suppression: right={} left={}",
            desiredSuppression[0] ? "active" : "inactive",
            desiredSuppression[1] ? "active" : "inactive");
    }

    void DynamicHandCollisionRuntime::applyTransitionCollisionSuppression(
        RE::hknpWorld* world,
        bool suppressCollision)
    {
        if (_transitionCollisionSuppressed == suppressCollision) {
            return;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        for (std::size_t handIndexValue = 0;
             handIndexValue < _hands.size();
             ++handIndexValue) {
            auto& handSlots = _hands[handIndexValue];
            const auto filterInfo = dynamicHandProxyFilterInfo(
                handIndexValue == 1,
                suppressCollision ||
                    _weaponOwnershipCollisionSuppressed[handIndexValue]);
            auto& owner = handSlots.bodies[0];
            if (owner.created && owner.body.isValid() &&
                owner.createdWorld == world) {
                owner.body.setCollisionFilterInfo(filterInfo, 1);
            }
            for (auto& slot : handSlots.bodies) {
                clearPhysicsContactState(slot);
            }
        }
        _transitionCollisionSuppressed = suppressCollision;
        _transitionCollisionSuppressedAtomic.store(
            suppressCollision,
            std::memory_order_release);
        ROCK_LOG_INFO(
            Hand,
            "Dynamic hand collision animation transition {} retained bodies",
            suppressCollision ? "suspended for" : "resumed on");
    }

    void DynamicHandCollisionRuntime::retireAll(void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        retireHand(_hands[0], bhkWorld, false);
        retireHand(_hands[1], bhkWorld, true);
        _surfaceContacts.clear();
    }

    void DynamicHandCollisionRuntime::reset()
    {
        retireAll(nullptr);
        auto pairStateMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        _weaponPairLeases.abandonWorld();
        _telemetrySnapshot = {};
        _pendingHapticEvents = {};
        _surfaceImpulsePairSequenceAtomic.store(0, std::memory_order_release);
        _surfaceProcessedPairSequenceAtomic.store(0, std::memory_order_release);
        _surfaceEligiblePairSequenceAtomic.store(0, std::memory_order_release);
        _surfaceContactPublishSequenceAtomic.store(0, std::memory_order_release);
        _telemetryUpdateSequence = 0;
        _logCounter = 0;
        _transitionState = {};
        _transitionCollisionSuppressed = false;
        _transitionCollisionSuppressedAtomic.store(false, std::memory_order_release);
        _weaponOwnershipCollisionSuppressed = {};
        _desiredWeaponBodyIdAtomic.store(
            hand_semantic_contact_state::kInvalidBodyId,
            std::memory_order_release);
        _pairFilterReadyAtomic.store(false, std::memory_order_release);
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            _weaponOwnedAtomic[hand].store(false, std::memory_order_release);
            _suppressedWeaponPairCountAtomic[hand].store(
                0,
                std::memory_order_release);
            _hands[hand].pendingOtherHandContactMaskAtomic.store(
                0,
                std::memory_order_release);
            _hands[hand].pendingWeaponContactMaskAtomic.store(
                0,
                std::memory_order_release);
            _hands[hand].otherHandContactMask = 0;
            _hands[hand].weaponContactMask = 0;
            _hands[hand].otherHandContactGraceFrames = 0;
            _hands[hand].weaponContactGraceFrames = 0;
        }
    }

    void DynamicHandCollisionRuntime::updateFrame(const PhysicsFrameContext& frame,
        bool physicsWritesAllowed,
        const Hand& rightHand,
        const Hand& leftHand,
        const BodyBoneColliderSet& bodyBoneColliders,
        bool rightHandWeaponOwned,
        bool leftHandWeaponOwned,
        std::uint32_t dynamicWeaponBodyId,
        bool rightVisualReturnActive,
        bool leftVisualReturnActive)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionFrame);

        _surfaceContacts.advanceFrame(
            frame.timing.valid ? frame.timing.deltaSeconds : 0.0f);
        _pendingHapticEvents = {};
        dynamic_hand_collision_telemetry::Snapshot telemetry{};
        telemetry.updateSequence = ++_telemetryUpdateSequence;
        telemetry.surfaceImpulsePairSequence =
            _surfaceImpulsePairSequenceAtomic.load(std::memory_order_acquire);
        telemetry.surfaceProcessedPairSequence =
            _surfaceProcessedPairSequenceAtomic.load(std::memory_order_acquire);
        telemetry.surfaceEligiblePairSequence =
            _surfaceEligiblePairSequenceAtomic.load(std::memory_order_acquire);
        telemetry.surfaceContactPublishSequence =
            _surfaceContactPublishSequenceAtomic.load(std::memory_order_acquire);
        telemetry.runtimeEnabled = true;
        telemetry.worldReady = frame.worldReady;
        telemetry.menuBlocked = frame.menuBlocked;
        telemetry.physicsWritesAllowed = physicsWritesAllowed;
        telemetry.hands[0].isLeft = false;
        telemetry.hands[1].isLeft = true;

        _desiredWeaponBodyIdAtomic.store(
            dynamicWeaponBodyId,
            std::memory_order_release);
        _weaponOwnedAtomic[0].store(
            rightHandWeaponOwned,
            std::memory_order_release);
        _weaponOwnedAtomic[1].store(
            leftHandWeaponOwned,
            std::memory_order_release);

        constexpr std::uint8_t kInteractionContactGraceFrames = 2;
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            auto& slots = _hands[hand];
            const auto updateContactMask = [](
                                               std::atomic<std::uint32_t>& pending,
                                               std::uint32_t& current,
                                               std::uint8_t& grace) {
                const std::uint32_t observed = pending.exchange(
                    0,
                    std::memory_order_acq_rel);
                if (observed != 0) {
                    current = observed;
                    grace = kInteractionContactGraceFrames;
                } else if (grace > 0) {
                    --grace;
                } else {
                    current = 0;
                }
            };
            updateContactMask(
                slots.pendingOtherHandContactMaskAtomic,
                slots.otherHandContactMask,
                slots.otherHandContactGraceFrames);
            updateContactMask(
                slots.pendingWeaponContactMaskAtomic,
                slots.weaponContactMask,
                slots.weaponContactGraceFrames);
            auto& handTelemetry = telemetry.hands[hand];
            handTelemetry.dynamicInteractionsEnabled = true;
            handTelemetry.pairFilterReady =
                _pairFilterReadyAtomic.load(std::memory_order_acquire);
            handTelemetry.otherHandContactMask =
                slots.otherHandContactMask;
            handTelemetry.weaponContactMask = slots.weaponContactMask;
            handTelemetry.suppressedWeaponPairCount =
                _suppressedWeaponPairCountAtomic[hand].load(
                    std::memory_order_acquire);
            handTelemetry.weaponPairSuppressed =
                handTelemetry.suppressedWeaponPairCount != 0;
            handTelemetry.dynamicInteractionLayer =
                collision_layer_policy::dynamicHandProxyLayerForHand(
                    hand == 1);
        }

        if (!frame.worldReady || frame.menuBlocked || !physicsWritesAllowed) {
            clearVisual(_hands[0], false);
            clearVisual(_hands[1], true);
            clearSurfaceFingerResponse(_hands[0], false);
            clearSurfaceFingerResponse(_hands[1], true);
            telemetry.hands[0].visualAuthorityAvailable = frik_visual_authority::isAvailable();
            telemetry.hands[1].visualAuthorityAvailable = telemetry.hands[0].visualAuthorityAvailable;
            updateHandHaptic(_hands[0], telemetry.hands[0], false, frame.deltaSeconds);
            updateHandHaptic(_hands[1], telemetry.hands[1], false, frame.deltaSeconds);
            telemetry.transitionCollisionSuppressed =
                _transitionCollisionSuppressed;
            _telemetrySnapshot = telemetry;
            return;
        }

        applyWeaponOwnershipCollisionSuppression(
            frame.hknpWorld,
            rightHandWeaponOwned,
            leftHandWeaponOwned);

        if (++_logCounter >= 360) {
            _logCounter = 0;
            ROCK_LOG_DEBUG(Hand,
                "DynamicHandCollision active: twinsPerHand={} maxLinVelHk={:.1f} divergenceTeleport={:.1f} minDeviation={:.3f} smoothingSpeed={:.1f} haptics={} priority={} palmCreated R={} L={} surfaceCallbacks(impulse/manifold/eligible/published)={}/{}/{}/{}",
                kBodiesPerHand,
                dynamic_hand_collision_policy::kMaximumLinearVelocityHavok,
                dynamic_hand_collision_policy::kDivergenceTeleportDistanceGameUnits,
                dynamic_hand_collision_policy::kRenderFollowMinimumDeviationGameUnits,
                dynamic_hand_collision_policy::kRenderFollowSmoothingSpeed,
                "yes",
                dynamic_hand_collision_policy::kVisualPriority,
                _hands[0].bodies[kPalmSlot].created ? "yes" : "no",
                _hands[1].bodies[kPalmSlot].created ? "yes" : "no",
                telemetry.surfaceImpulsePairSequence,
                telemetry.surfaceProcessedPairSequence,
                telemetry.surfaceEligiblePairSequence,
                telemetry.surfaceContactPublishSequence);
        }

        const auto handTargetsStable = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand) {
            if (handInput.disabled) {
                return true;
            }
            const auto& handTwins = hand.dynamicTwinTargets();
            const auto& forearmTwins = bodyBoneColliders.dynamicForearmTwinTargets();
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                const auto* twinFrame = twinFrameForSlot(handTwins, forearmTwins, isLeft, bodyIndex);
                if (!twinFrame || !twinFrame->valid ||
                    !isFiniteTransform(twinFrame->target)) {
                    return false;
                }
            }
            return true;
        };
        const bool transitionTargetsStable =
            handTargetsStable(false, frame.right, rightHand) &&
            handTargetsStable(true, frame.left, leftHand);
        const auto transitionStep = dynamic_hand_collision_transition::advance(
            _transitionState,
            frame.reloadBoundaryActive,
            transitionTargetsStable);
        _transitionState = transitionStep.state;
        if (transitionStep.collisionStateChanged) {
            applyTransitionCollisionSuppression(frame.hknpWorld, transitionStep.suppressCollision);
        }

        auto updateHand = [&](bool isLeft, const HandFrameInput& handInput, const Hand& hand, bool weaponOwned, bool visualReturnActive) {
            const std::size_t index = handIndex(isLeft);
            auto& handSlots = _hands[index];
            auto& handTelemetry = telemetry.hands[index];
            handTelemetry.isLeft = isLeft;
            handTelemetry.handDisabled = handInput.disabled;
            handTelemetry.visualAuthorityAvailable = frik_visual_authority::isAvailable();

            if (!handSlots.surfaceLatch.active) {
                handSlots.lastPresentedHandWorld = handInput.rawHandWorld;
                handSlots.lastPresentedHandWorldValid =
                    isFiniteTransform(handInput.rawHandWorld);
            }

            if (handInput.disabled) {
                clearVisual(handSlots, isLeft);
                clearSurfaceFingerResponse(handSlots, isLeft);
                updateHandHaptic(handSlots, handTelemetry, false, frame.deltaSeconds);
                return;
            }

            const auto& handTwins = hand.dynamicTwinTargets();
            const auto& forearmTwins = bodyBoneColliders.dynamicForearmTwinTargets();
            std::array<RE::NiPoint3, kBodiesPerHand> deviations{};
            std::array<bool, kBodiesPerHand> deviationValid{};

            /*
             * A fixed-surface latch follows the target body's live rigid
             * transform. If identity/readback disappears, retain the last safe
             * transforms for this frame; TouchGrabRuntime owns invalidation and
             * releases the latch from its normal lifecycle pass.
             */
            if (handSlots.surfaceLatch.active) {
                auto& latch = handSlots.surfaceLatch;
                const auto targetSnapshot = havok_runtime::snapshotBody(
                    frame.hknpWorld,
                    RE::hknpBodyId{ latch.targetBodyId });
                RE::NiTransform targetWorld{};
                if (targetSnapshot.valid &&
                    targetSnapshot.body == latch.targetBodyIdentity &&
                    targetSnapshot.collisionObject == latch.targetCollisionIdentity &&
                    havok_runtime::tryResolveLiveBodyWorldTransform(
                        frame.hknpWorld,
                        RE::hknpBodyId{ latch.targetBodyId },
                        targetWorld) &&
                    isFiniteTransform(targetWorld)) {
                    latch.lastHandWorld = transform_math::composeTransforms(
                        targetWorld,
                        latch.handInTargetBody);
                    if (latch.meshAuthoritative) {
                        latch.meshAnchorWorld =
                            transform_math::localPointToWorld(
                                targetWorld,
                                latch.meshAnchorInTargetBody);
                        latch.meshNormalWorld =
                            transform_math::localVectorToWorld(
                                targetWorld,
                                latch.meshNormalInTargetBody);
                    }
                    for (std::size_t bodyIndex = 0;
                         bodyIndex < kBodiesPerHand;
                         ++bodyIndex) {
                        if (!latch.proxyRelationshipValid[bodyIndex]) {
                            continue;
                        }
                        latch.lastProxyWorld[bodyIndex] =
                            transform_math::composeTransforms(
                                targetWorld,
                                latch.proxyInTargetBody[bodyIndex]);
                    }
                }
            }

            /*
             * The one compound keeps chasing every current role frame even
             * when another system owns presentation. Only the visual authority
             * gate changes; physics tracking and compound animation continue.
             */
            std::array<const dynamic_hand_twin::TwinSlotFrame*,
                kBodiesPerHand> twinFrames{};
            std::array<RE::NiTransform, kBodiesPerHand> driveTargets{};
            bool allChildrenValid = true;
            for (std::size_t bodyIndex = 0;
                 bodyIndex < kBodiesPerHand;
                 ++bodyIndex) {
                auto& twinTelemetry = handTelemetry.twins[bodyIndex];
                twinTelemetry.role =
                    dynamic_hand_collision_telemetry::roleForBodyIndex(
                        bodyIndex);
                const auto* twinFrame = twinFrameForSlot(
                    handTwins,
                    forearmTwins,
                    isLeft,
                    bodyIndex);
                twinFrames[bodyIndex] = twinFrame;
                if (!twinFrame || !twinFrame->valid ||
                    !isFiniteTransform(twinFrame->target)) {
                    allChildrenValid = false;
                    continue;
                }

                twinTelemetry.lengthGameUnits = twinFrame->length;
                twinTelemetry.radiusGameUnits = twinFrame->radius;
                twinTelemetry.convexRadiusGameUnits =
                    twinFrame->convexRadius;
                twinTelemetry.handTargetResponseScale =
                    dynamic_hand_collision_kinematics::
                        sanitizeHandTargetResponseScale(
                            twinFrame->handTargetResponseScale);
                driveTargets[bodyIndex] =
                    handSlots.surfaceLatch.active &&
                        handSlots.surfaceLatch.
                            proxyRelationshipValid[bodyIndex] ?
                    handSlots.surfaceLatch.lastProxyWorld[bodyIndex] :
                    twinFrame->target;
                twinTelemetry.publishedTargetValid =
                    isFiniteTransform(driveTargets[bodyIndex]);
                twinTelemetry.publishedTargetWorld =
                    driveTargets[bodyIndex];
            }

            const std::uint64_t geometryGeneration =
                (handTwins.geometryGeneration * 0x9E3779B185EBCA87ull) ^
                (forearmTwins.geometryGeneration +
                    0xC2B2AE3D27D4EB4Full);
            const RE::NiTransform compoundRootTarget =
                driveTargets[kPalmSlot];
            if (!allChildrenValid ||
                !ensureHandCreated(
                    handSlots,
                    isLeft,
                    frame,
                    hand,
                    bodyBoneColliders,
                    twinFrames,
                    compoundRootTarget,
                    driveTargets,
                    geometryGeneration) ||
                !queueCompoundPose(
                    handSlots,
                    compoundRootTarget,
                    driveTargets)) {
                if (handSlots.bodies[0].created &&
                    !_transitionCollisionSuppressed) {
                    retireHand(handSlots, frame.bhkWorld, isLeft);
                }
                clearVisual(handSlots, isLeft);
                updateHandHaptic(
                    handSlots,
                    handTelemetry,
                    false,
                    frame.deltaSeconds);
                return;
            }

            auto& compoundOwner = handSlots.bodies[0];
            (void)queueGeneratedKeyframedBodyTarget(
                compoundOwner.driveState,
                compoundRootTarget,
                frame.deltaSeconds,
                dynamic_hand_collision_policy::kDivergenceTeleportDistanceGameUnits);

            for (std::size_t bodyIndex = 0;
                 bodyIndex < kBodiesPerHand;
                 ++bodyIndex) {
                auto& twinTelemetry = handTelemetry.twins[bodyIndex];
                twinTelemetry.bodyCreated = compoundOwner.created;
                twinTelemetry.bodyId = compoundOwner.created ?
                    compoundOwner.body.getBodyId().value :
                    dynamic_hand_collision_telemetry::kInvalidBodyId;

                PhysicsTelemetrySample physicsSample{};
                std::uint64_t physicsSampleSequence = 0;
                if (!readPhysicsTelemetry(
                        handSlots.bodies[bodyIndex],
                        physicsSample,
                        physicsSampleSequence) ||
                    !physicsSample.valid ||
                    !isFinitePoint(
                        physicsSample.requestedTargetWorldGame) ||
                    !isFinitePoint(
                        physicsSample.commandedTargetWorldGame) ||
                    !isFinitePoint(physicsSample.liveBodyWorldGame)) {
                    continue;
                }

                twinTelemetry.physicsSampleSequence =
                    physicsSampleSequence;
                twinTelemetry.physicsSampleValid = true;
                twinTelemetry.targetVelocityValid =
                    physicsSample.targetVelocityValid;
                twinTelemetry.contactActive =
                    physicsSample.contactActive;
                twinTelemetry.worldContactActive =
                    physicsSample.worldContactActive;
                twinTelemetry.recoveryTeleport =
                    physicsSample.recoveryTeleport;
                twinTelemetry.requestedTargetWorldGame =
                    physicsSample.requestedTargetWorldGame;
                twinTelemetry.commandedTargetWorldGame =
                    physicsSample.commandedTargetWorldGame;
                twinTelemetry.liveBodyWorldGame =
                    physicsSample.liveBodyWorldGame;
                twinTelemetry.targetVelocityWorldGameUnitsPerSecond =
                    physicsSample.
                        targetVelocityWorldGameUnitsPerSecond;
                twinTelemetry.approachSpeedGameUnitsPerSecond =
                    physicsSample.approachSpeedGameUnitsPerSecond;
                twinTelemetry.physicsDeltaSeconds =
                    physicsSample.physicsDeltaSeconds;
                twinTelemetry.solverResidualWorldGame = subtractPoints(
                    twinTelemetry.liveBodyWorldGame,
                    twinTelemetry.commandedTargetWorldGame);
                twinTelemetry.requestedGapWorldGame = subtractPoints(
                    twinTelemetry.liveBodyWorldGame,
                    twinTelemetry.requestedTargetWorldGame);
                twinTelemetry.solverResidualGameUnits = pointLength(
                    twinTelemetry.solverResidualWorldGame);
                twinTelemetry.requestedGapGameUnits = pointLength(
                    twinTelemetry.requestedGapWorldGame);

                if (!physicsSample.contactActive) {
                    continue;
                }

                twinTelemetry.contactDeviationWorldGame =
                    twinTelemetry.requestedGapWorldGame;
                twinTelemetry.contactDeviationGameUnits =
                    twinTelemetry.requestedGapGameUnits;
                twinTelemetry.handTargetCorrectionWorldGame = {
                    twinTelemetry.contactDeviationWorldGame.x *
                        twinTelemetry.handTargetResponseScale,
                    twinTelemetry.contactDeviationWorldGame.y *
                        twinTelemetry.handTargetResponseScale,
                    twinTelemetry.contactDeviationWorldGame.z *
                        twinTelemetry.handTargetResponseScale,
                };
                twinTelemetry.handTargetCorrectionGameUnits =
                    pointLength(
                        twinTelemetry.handTargetCorrectionWorldGame);
                deviations[bodyIndex] =
                    twinTelemetry.handTargetCorrectionWorldGame;
                deviationValid[bodyIndex] = true;
                handTelemetry.contactMask |=
                    1u << static_cast<std::uint32_t>(bodyIndex);
                ++handTelemetry.contactCount;
            }

            handTelemetry.anyContact = handTelemetry.contactCount > 0;
            const bool physicallyOwnedByStrongerSystem =
                suppressesGeneratedHandContactEvidence(hand.getState()) ||
                weaponOwned ||
                _transitionCollisionSuppressed;
            const bool visuallyOwnedByStrongerSystem =
                physicallyOwnedByStrongerSystem || visualReturnActive;
            const bool latchAuthorityBlocked =
                handSlots.surfaceLatch.active &&
                visuallyOwnedByStrongerSystem;
            std::uint32_t helpfulFingerSlotMask = 0;
            if (handSlots.surfaceLatch.meshAuthoritative) {
                clearSurfaceFingerResponse(handSlots, isLeft);
            } else if (!visuallyOwnedByStrongerSystem &&
                       handTelemetry.visualAuthorityAvailable) {
                helpfulFingerSlotMask = updateSurfaceFingerResponse(
                    handSlots,
                    isLeft,
                    hand.getState(),
                    handInput.rawHandWorld,
                    handTwins,
                    handTelemetry,
                    frame.deltaSeconds,
                    handSlots.surfaceLatch.active);
            } else {
                clearSurfaceFingerResponse(handSlots, isLeft);
            }
            handTelemetry.surfaceFingerResponseActive =
                handSlots.surfaceFingerResponse.active;
            handTelemetry.surfaceFingerHelpfulSlotMask =
                helpfulFingerSlotMask;
            if (handSlots.surfaceFingerResponse.active) {
                handTelemetry.surfaceFingerOpenValues =
                    handSlots.surfaceFingerResponse.currentOpenValues;
                handTelemetry.surfaceFingerDirections =
                    handSlots.surfaceFingerResponse.lastDirections;
            }

            /*
             * Palm and forearm preserve the established rigid-hand response.
             * A fingertip also preserves that legacy path whenever anatomical
             * flexion/extension is unavailable or would move the segment
             * farther into the surface. Base/middle phalanxes are finger-only
             * probes: adding them must not multiply whole-hand pushout.
             */
            for (std::size_t bodyIndex = 0;
                 bodyIndex < kBodiesPerHand;
                 ++bodyIndex) {
                if (!deviationValid[bodyIndex]) {
                    continue;
                }
                const bool rigidPrimary = bodyIndex == kPalmSlot ||
                                          bodyIndex >= kFirstForearmSlot;
                const bool unresolvedLegacyTip =
                    dynamic_hand_collision_telemetry::isFingerTipSlot(
                        bodyIndex) &&
                    (helpfulFingerSlotMask &
                        (1u << static_cast<std::uint32_t>(bodyIndex))) == 0;
                deviationValid[bodyIndex] =
                    rigidPrimary || unresolvedLegacyTip;
            }
            const RE::NiPoint3 combined =
                handTelemetry.anyContact ? combineTwinDeviations(deviations, deviationValid) : RE::NiPoint3{};
            handTelemetry.combinedContactDeviationWorldGame = combined;
            handTelemetry.combinedContactDeviationGameUnits = pointLength(combined);

            if (handSlots.surfaceLatch.active) {
                if (latchAuthorityBlocked) {
                    endSurfaceLatch(isLeft);
                    updateHandHaptic(
                        handSlots,
                        handTelemetry,
                        false,
                        frame.deltaSeconds);
                    handTelemetry.ownedByStrongerSystem = true;
                    handTelemetry.visualActive = false;
                    return;
                }
                handTelemetry.ownedByStrongerSystem = true;
                updateHandHaptic(
                    handSlots,
                    handTelemetry,
                    false,
                    frame.deltaSeconds);
                handSlots.appliedDeviation = {};
                handSlots.teleportRecoverySecondsRemaining = 0.0f;
                const auto& latchTarget =
                    handSlots.surfaceLatch.lastHandWorld;
                if (!handTelemetry.visualAuthorityAvailable ||
                    !isFiniteTransform(latchTarget)) {
                    endSurfaceLatch(isLeft);
                    handTelemetry.visualActive = handSlots.visualActive;
                    return;
                }
                auto& latch = handSlots.surfaceLatch;
                if (latch.meshAuthoritative &&
                    latch.meshFingerPoseValid) {
                    if (frik_visual_authority::setHandPoseCustomWithPriority(
                            SURFACE_MESH_GRAB_POSE_TAG,
                            frik_visual_authority::handFromBool(isLeft),
                            frik_visual_authority::
                                makeHandPoseDataFromJointValues(
                                    latch.meshFingerJointValues),
                            kSurfaceLatchVisualPriority)) {
                        latch.meshPosePublished = true;
                    } else {
                        ROCK_LOG_SAMPLE_WARN(
                            Hand,
                            2000,
                            "{} surface mesh finger pose apply failed",
                            isLeft ? "Left" : "Right");
                    }
                }
                if (frik_visual_authority::applyExternalHandWorldTransform(
                        dynamicHandTag(isLeft),
                        frik_visual_authority::handFromBool(isLeft),
                        latchTarget,
                        kSurfaceLatchVisualPriority)) {
                    handSlots.visualActive = true;
                    handSlots.lastPresentedHandWorld = latchTarget;
                    handSlots.lastPresentedHandWorldValid = true;
                } else {
                    ROCK_LOG_SAMPLE_WARN(
                        Hand,
                        2000,
                        "{} dynamic surface latch render apply failed",
                        isLeft ? "Left" : "Right");
                    endSurfaceLatch(isLeft);
                }
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            handTelemetry.ownedByStrongerSystem = visuallyOwnedByStrongerSystem;
            updateHandHaptic(
                handSlots,
                handTelemetry,
                !physicallyOwnedByStrongerSystem && handTelemetry.visualAuthorityAvailable,
                frame.deltaSeconds);
            if (visuallyOwnedByStrongerSystem || !handTelemetry.visualAuthorityAvailable) {
                clearVisual(handSlots, isLeft);
                handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
                handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            /*
             * A divergence teleport opens the recovery window: the applied
             * deviation glides home over the configured duration (speed ~3/T
             * reaches ~95% by the window's end) instead of snapping at the
             * normal visual-release rate.
             */
            bool teleportedThisFrame = false;
            for (auto& slot : handSlots.bodies) {
                if (slot.teleportedAtomic.exchange(false, std::memory_order_acq_rel)) {
                    teleportedThisFrame = true;
                }
            }
            constexpr float recoveryDuration = dynamic_hand_collision_policy::kTeleportRecoverySeconds;
            if (teleportedThisFrame && recoveryDuration > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = recoveryDuration;
            }
            const float frameDt = std::clamp(
                std::isfinite(frame.deltaSeconds) &&
                        frame.deltaSeconds > 0.0f ?
                    frame.deltaSeconds :
                    0.0f,
                0.0f,
                0.1f);
            float smoothingSpeed = handTelemetry.anyContact ? 0.0f : dynamic_hand_collision_policy::kRenderFollowSmoothingSpeed;
            if (handSlots.teleportRecoverySecondsRemaining > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = std::max(0.0f, handSlots.teleportRecoverySecondsRemaining - frameDt);
                const float recoverySpeed = 3.0f / std::max(recoveryDuration, 0.05f);
                smoothingSpeed = dynamic_hand_collision_policy::kRenderFollowSmoothingSpeed;
                smoothingSpeed = smoothingSpeed > 0.0f ? std::min(smoothingSpeed, recoverySpeed) : recoverySpeed;
            }

            handSlots.appliedDeviation = smoothAppliedDeviation(
                handSlots.appliedDeviation,
                combined,
                smoothingSpeed,
                frame.deltaSeconds);
            handTelemetry.teleportRecoverySecondsRemaining = handSlots.teleportRecoverySecondsRemaining;

            const auto& applied = handSlots.appliedDeviation;
            const float appliedLengthSq = applied.x * applied.x + applied.y * applied.y + applied.z * applied.z;
            constexpr float minDeviation = dynamic_hand_collision_policy::kRenderFollowMinimumDeviationGameUnits;
            if (!std::isfinite(appliedLengthSq) || appliedLengthSq <= minDeviation * minDeviation) {
                clearVisual(handSlots, isLeft);
                handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
                handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            handTelemetry.appliedVisualDeviationWorldGame = applied;
            handTelemetry.appliedVisualDeviationGameUnits = std::sqrt(appliedLengthSq);

            RE::NiTransform target = handInput.rawHandWorld;
            target.translate.x += applied.x;
            target.translate.y += applied.y;
            target.translate.z += applied.z;
            if (!isFiniteTransform(target)) {
                clearVisual(handSlots, isLeft);
                handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
                handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
                handTelemetry.visualActive = handSlots.visualActive;
                return;
            }

            handSlots.lastPresentedHandWorld = target;
            handSlots.lastPresentedHandWorldValid = true;

            if (frik_visual_authority::applyExternalHandWorldTransform(
                    dynamicHandTag(isLeft),
                    frik_visual_authority::handFromBool(isLeft),
                    target,
                    dynamic_hand_collision_policy::kVisualPriority)) {
                handSlots.visualActive = true;
            } else {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "{} dynamic hand render-follow apply failed", isLeft ? "Left" : "Right");
                clearVisual(handSlots, isLeft);
            }
            handTelemetry.appliedVisualDeviationWorldGame = handSlots.appliedDeviation;
            handTelemetry.appliedVisualDeviationGameUnits = pointLength(handSlots.appliedDeviation);
            handTelemetry.visualActive = handSlots.visualActive;
        };

        updateHand(false, frame.right, rightHand, rightHandWeaponOwned, rightVisualReturnActive);
        updateHand(true, frame.left, leftHand, leftHandWeaponOwned, leftVisualReturnActive);
        telemetry.transitionCollisionSuppressed =
            _transitionCollisionSuppressed;
        _telemetrySnapshot = telemetry;
    }

    void DynamicHandCollisionRuntime::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionPhysicsDrive);

        std::array<HavokPairCollisionLeaseSet::DesiredPair,
            HavokPairCollisionLeaseSet::kMaximumPairs>
            desiredPairs{};
        std::size_t desiredPairCount = 0;
        const std::uint32_t weaponBodyId =
            _desiredWeaponBodyIdAtomic.load(std::memory_order_acquire);
        if (world &&
            weaponBodyId != hand_semantic_contact_state::kInvalidBodyId) {
            for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
                if (!_weaponOwnedAtomic[hand].load(
                        std::memory_order_acquire)) {
                    continue;
                }
                const std::uint32_t handBodyId =
                    _hands[hand].bodies[0].bodyIdAtomic.load(
                        std::memory_order_acquire);
                if (handBodyId !=
                        hand_semantic_contact_state::kInvalidBodyId &&
                    desiredPairCount < desiredPairs.size()) {
                    desiredPairs[desiredPairCount++] = {
                        .bodyA = handBodyId,
                        .bodyB = weaponBodyId,
                        .ownerGroup = static_cast<std::uint8_t>(hand),
                    };
                }
            }
        }
        const auto pairResult = _weaponPairLeases.reconcile(
            world,
            desiredPairs.data(),
            desiredPairCount);
        _pairFilterReadyAtomic.store(
            pairResult.filterAvailable,
            std::memory_order_release);
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            _suppressedWeaponPairCountAtomic[hand].store(
                pairResult.activePairsByOwnerGroup[hand],
                std::memory_order_release);
        }

        if (!world) {
            return;
        }

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const bool isLeft = hand == 1;
            auto& handSlots = _hands[hand];
            auto& slot = handSlots.bodies[0];
            if (!slot.created || slot.createdWorld != world) {
                continue;
            }

            const auto clearSemanticTelemetry = [&]() {
                for (auto& semanticSlot : handSlots.bodies) {
                    clearPhysicsContactState(semanticSlot);
                }
            };

            {
                std::scoped_lock poseLock(handSlots.compoundPoseMutex);
                if (handSlots.queuedCompoundPoseSequence !=
                    handSlots.consumedCompoundPoseSequence) {
                    const auto updateResult =
                        handSlots.compoundShape.updateTransforms(
                            handSlots.pendingCompoundChildTransforms);
                    if (!updateResult.succeeded) {
                        slot.rebuildRequestedAtomic.store(
                            true,
                            std::memory_order_release);
                        clearSemanticTelemetry();
                        continue;
                    }
                    handSlots.consumedCompoundPoseSequence =
                        handSlots.queuedCompoundPoseSequence;
                    handSlots.consumedChildInCompound =
                        handSlots.childInCompound;
                }
            }

            constexpr float divergenceThreshold = dynamic_hand_collision_policy::kDivergenceTeleportDistanceGameUnits;
            const bool teleportArmed =
                slot.divergenceDwellSeconds >=
                dynamic_hand_collision_policy::kDivergenceTeleportDwellSeconds;
            GeneratedBodyDriveMode mode{
                .dynamicVelocity = true,
                .divergenceTeleportGameUnits =
                    teleportArmed ? divergenceThreshold : 0.0f,
            };

            constexpr float kPressCapActivationDeviationGameUnits = 0.25f;
            constexpr float pressCapHavok = dynamic_hand_collision_policy::kContactPressMaximumVelocityHavok;
            if (pressCapHavok > 0.0f &&
                slot.lastPostSolveDeviationValid) {
                const auto& deviation = slot.lastPostSolveDeviationGame;
                const float deviationLength = std::sqrt(
                    deviation.x * deviation.x +
                    deviation.y * deviation.y +
                    deviation.z * deviation.z);
                if (std::isfinite(deviationLength) &&
                    deviationLength >
                        kPressCapActivationDeviationGameUnits) {
                    mode.hasContactPressDirection = true;
                    mode.contactPressDirection[0] =
                        -deviation.x / deviationLength;
                    mode.contactPressDirection[1] =
                        -deviation.y / deviationLength;
                    mode.contactPressDirection[2] =
                        -deviation.z / deviationLength;
                    mode.contactPressMaxVelocityHavok = pressCapHavok;
                }
            }

            const auto result = driveGeneratedKeyframedBody(
                world,
                slot.body,
                slot.driveState,
                timing,
                isLeft ?
                    "LeftHandDynamicCompound" :
                    "RightHandDynamicCompound",
                0,
                dynamic_hand_collision_policy::kMaximumLinearVelocityHavok,
                0.0f,
                mode);

            if (result.shouldRequestRebuild()) {
                slot.rebuildRequestedAtomic.store(
                    true,
                    std::memory_order_release);
                clearSemanticTelemetry();
                continue;
            }
            if (!result.driven ||
                !result.hasRequestedTargetGameTransform ||
                !result.hasCommandedTargetGameTransform) {
                clearSemanticTelemetry();
                continue;
            }

            slot.requestedTargetWorld =
                result.requestedTargetGameTransform;
            slot.commandedTargetWorld =
                result.commandedTargetGameTransform;
            slot.requestedTargetGame =
                slot.requestedTargetWorld.translate;
            slot.commandedTargetGame =
                slot.commandedTargetWorld.translate;
            const float havokToGameScale = physics_scale::havokToGame();
            slot.droveTargetVelocityValid =
                result.hasSampledTargetLinearVelocityHavok &&
                physics_scale::isUsableScale(havokToGameScale);
            slot.droveTargetVelocityGameUnitsPerSecond =
                slot.droveTargetVelocityValid ?
                RE::NiPoint3{
                    result.sampledTargetLinearVelocityHavok.x *
                        havokToGameScale,
                    result.sampledTargetLinearVelocityHavok.y *
                        havokToGameScale,
                    result.sampledTargetLinearVelocityHavok.z *
                        havokToGameScale,
                } :
                RE::NiPoint3{};
            slot.drovePhysicsDeltaSeconds = result.driveDeltaSeconds;
            slot.droveRecoveryTeleport = result.teleported;
            slot.droveThisSubstep = true;

            const float requestedGapGameUnits =
                result.hasLiveBodyTransform ?
                pointLength(subtractPoints(
                    result.liveBodyGamePosition,
                    result.requestedTargetGamePosition)) :
                0.0f;
            if (result.teleported) {
                slot.divergenceDwellSeconds = 0.0f;
                slot.teleportedAtomic.store(
                    true,
                    std::memory_order_release);
            } else if (result.attempted &&
                       result.hasLiveBodyTransform &&
                       std::isfinite(requestedGapGameUnits) &&
                       divergenceThreshold > 0.0f &&
                       requestedGapGameUnits > divergenceThreshold) {
                slot.divergenceDwellSeconds += std::clamp(
                    result.driveDeltaSeconds,
                    0.0f,
                    0.1f);
            } else {
                slot.divergenceDwellSeconds = 0.0f;
            }
        }
    }

    void DynamicHandCollisionRuntime::samplePostSolveDeviations(
        RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(
            performance_profiler::Scope::DynamicHandCollisionPostSolve);

        if (!world) {
            return;
        }

        for (auto& handSlots : _hands) {
            auto& owner = handSlots.bodies[0];
            if (!owner.created || owner.createdWorld != world ||
                !owner.droveThisSubstep) {
                continue;
            }
            owner.droveThisSubstep = false;

            RE::NiTransform liveCompoundWorld{};
            if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    owner.body.getBodyId(),
                    liveCompoundWorld) ||
                !isFiniteTransform(liveCompoundWorld)) {
                for (auto& semanticSlot : handSlots.bodies) {
                    clearPhysicsContactState(semanticSlot);
                }
                continue;
            }

            constexpr std::uint32_t kAllChildBits =
                (1u << static_cast<std::uint32_t>(kBodiesPerHand)) - 1u;
            const std::uint32_t observedContactMask =
                handSlots.pendingSolverContactMaskAtomic.exchange(
                    0,
                    std::memory_order_acq_rel) &
                kAllChildBits;
            const std::uint32_t observedWorldContactMask =
                handSlots.pendingWorldContactMaskAtomic.exchange(
                    0,
                    std::memory_order_acq_rel) &
                kAllChildBits;
            const bool retentionTimingValid =
                timing.valid && !timing.usedFallback &&
                havok_physics_timing::isUsableDelta(
                    timing.substepDeltaSeconds);
            const float retentionDeltaSeconds =
                retentionTimingValid ?
                timing.substepDeltaSeconds :
                0.0f;
            std::uint32_t retainedSolverContactMask = 0;
            std::uint32_t retainedWorldContactMask = 0;
            for (std::size_t bodyIndex = 0;
                 bodyIndex < kBodiesPerHand;
                 ++bodyIndex) {
                const std::uint32_t childBit =
                    1u << static_cast<std::uint32_t>(bodyIndex);
                const bool solverObserved =
                    (observedContactMask & childBit) != 0;
                const bool worldObserved =
                    (observedWorldContactMask & childBit) != 0;

                auto& solverRetention =
                    handSlots.solverContactRetentionSeconds[bodyIndex];
                solverRetention = solverObserved ?
                    kCompoundContactRetentionSeconds :
                    (retentionTimingValid ?
                            std::max(
                                0.0f,
                                solverRetention - retentionDeltaSeconds) :
                            0.0f);
                if (solverRetention > 0.0f) {
                    retainedSolverContactMask |= childBit;
                }

                auto& worldRetention =
                    handSlots.worldContactRetentionSeconds[bodyIndex];
                if (worldObserved) {
                    worldRetention = kCompoundContactRetentionSeconds;
                } else if (solverObserved) {
                    // Fresh hand or weapon evidence supersedes stale world
                    // classification for this semantic child.
                    worldRetention = 0.0f;
                } else {
                    worldRetention = retentionTimingValid ?
                        std::max(
                            0.0f,
                            worldRetention - retentionDeltaSeconds) :
                        0.0f;
                }
                if (solverRetention > 0.0f &&
                    worldRetention > 0.0f) {
                    retainedWorldContactMask |= childBit;
                }
            }
            handSlots.retainedSolverContactMask =
                retainedSolverContactMask;
            handSlots.retainedWorldContactMask =
                retainedWorldContactMask;
            const std::uint32_t contactMask =
                handSlots.retainedSolverContactMask;
            const std::uint32_t worldContactMask =
                handSlots.retainedWorldContactMask;

            std::array<RE::NiTransform, kBodiesPerHand> childFrames{};
            {
                std::scoped_lock poseLock(handSlots.compoundPoseMutex);
                childFrames = handSlots.consumedChildInCompound;
            }

            const RE::NiTransform requestedSceneWorld =
                colliderFrameToSceneFrame(owner.requestedTargetWorld);
            const RE::NiTransform commandedSceneWorld =
                colliderFrameToSceneFrame(owner.commandedTargetWorld);
            const RE::NiTransform liveCompoundSceneWorld =
                colliderFrameToSceneFrame(liveCompoundWorld);

            float maxEntryApproachSpeed = 0.0f;
            for (std::size_t bodyIndex = 0;
                 bodyIndex < kBodiesPerHand;
                 ++bodyIndex) {
                const RE::NiTransform requestedChildWorld =
                    transform_math::composeTransforms(
                        requestedSceneWorld,
                        childFrames[bodyIndex]);
                const RE::NiTransform commandedChildWorld =
                    transform_math::composeTransforms(
                        commandedSceneWorld,
                        childFrames[bodyIndex]);
                const RE::NiTransform liveChildWorld =
                    transform_math::composeTransforms(
                        liveCompoundSceneWorld,
                        childFrames[bodyIndex]);
                if (!isFiniteTransform(requestedChildWorld) ||
                    !isFiniteTransform(commandedChildWorld) ||
                    !isFiniteTransform(liveChildWorld)) {
                    clearPhysicsContactState(
                        handSlots.bodies[bodyIndex]);
                    continue;
                }

                const bool contact =
                    (contactMask &
                        (1u << static_cast<std::uint32_t>(
                            bodyIndex))) != 0;
                const bool worldContact =
                    (worldContactMask &
                        (1u << static_cast<std::uint32_t>(
                            bodyIndex))) != 0;
                RE::NiPoint3 deviation{};
                float approachSpeedGameUnitsPerSecond = 0.0f;
                if (contact) {
                    deviation = subtractPoints(
                        liveChildWorld.translate,
                        requestedChildWorld.translate);
                    if (!owner.droveRecoveryTeleport &&
                        owner.droveTargetVelocityValid) {
                        approachSpeedGameUnitsPerSecond =
                            dynamic_hand_collision_feedback::
                                projectedApproachSpeedGameUnitsPerSecond(
                                    owner.
                                        droveTargetVelocityGameUnitsPerSecond,
                                    deviation);
                    }
                    maxEntryApproachSpeed = std::max(
                        maxEntryApproachSpeed,
                        approachSpeedGameUnitsPerSecond);
                }

                publishPhysicsTelemetry(
                    handSlots.bodies[bodyIndex],
                    PhysicsTelemetrySample{
                        .requestedTargetWorldGame =
                            requestedChildWorld.translate,
                        .commandedTargetWorldGame =
                            commandedChildWorld.translate,
                        .liveBodyWorldGame = liveChildWorld.translate,
                        .targetVelocityWorldGameUnitsPerSecond =
                            owner.
                                droveTargetVelocityGameUnitsPerSecond,
                        .approachSpeedGameUnitsPerSecond =
                            approachSpeedGameUnitsPerSecond,
                        .physicsDeltaSeconds =
                            owner.drovePhysicsDeltaSeconds,
                        .valid = true,
                        .targetVelocityValid =
                            owner.droveTargetVelocityValid,
                        .contactActive = contact,
                        .worldContactActive = worldContact,
                        .recoveryTeleport =
                            owner.droveRecoveryTeleport,
                    });
            }

            owner.lastPostSolveDeviationGame = subtractPoints(
                liveCompoundWorld.translate,
                owner.requestedTargetWorld.translate);
            owner.lastPostSolveDeviationValid = contactMask != 0;
            owner.lastPostSolveContact = contactMask != 0;

            const bool anyContact = contactMask != 0;
            constexpr float entryGateSpeed = dynamic_hand_collision_policy::kHapticMinimumApproachSpeedGameUnitsPerSecond;
            if (anyContact && !handSlots.physicsContactActive &&
                maxEntryApproachSpeed >= entryGateSpeed &&
                maxEntryApproachSpeed > 0.0f) {
                handSlots.contactEntryApproachSpeedAtomic.store(
                    maxEntryApproachSpeed,
                    std::memory_order_relaxed);
                handSlots.contactEntryMaskAtomic.store(
                    contactMask,
                    std::memory_order_relaxed);
                handSlots.contactEntrySequenceAtomic.fetch_add(
                    1,
                    std::memory_order_release);
                handSlots.physicsContactActive = true;
            } else if (!anyContact) {
                handSlots.physicsContactActive = false;
            }
        }
    }
}

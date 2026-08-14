#include "physics-interaction/hand/DynamicHandCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

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
        /*
         * Post-solve solver-residual thresholds separating "tracking freely"
         * (residual is integration noise, well under the 0.05 gu render
         * epsilon — proven by the twitch-free at-rest sessions) from "blocked
         * by the solver" (at least the press-cap penetration step, 1 Havok m/s
         * over the shortest substep ≈ 0.26 gu). Enter sits between the two;
         * stay is lower so a grazing contact does not flap per substep.
         */
        constexpr float kContactResidualEnterGameUnits = 0.15f;
        constexpr float kContactResidualStayGameUnits = 0.05f;

        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kRightTwinNames{
            "ROCK_DynHandTwin_R_Palm",
            "ROCK_DynHandTwin_R_ThumbBase",
            "ROCK_DynHandTwin_R_ThumbMiddle",
            "ROCK_DynHandTwin_R_ThumbTip",
            "ROCK_DynHandTwin_R_IndexBase",
            "ROCK_DynHandTwin_R_IndexMiddle",
            "ROCK_DynHandTwin_R_IndexTip",
            "ROCK_DynHandTwin_R_MiddleBase",
            "ROCK_DynHandTwin_R_MiddleMiddle",
            "ROCK_DynHandTwin_R_MiddleTip",
            "ROCK_DynHandTwin_R_RingBase",
            "ROCK_DynHandTwin_R_RingMiddle",
            "ROCK_DynHandTwin_R_RingTip",
            "ROCK_DynHandTwin_R_PinkyBase",
            "ROCK_DynHandTwin_R_PinkyMiddle",
            "ROCK_DynHandTwin_R_PinkyTip",
            "ROCK_DynHandTwin_R_Forearm",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kLeftTwinNames{
            "ROCK_DynHandTwin_L_Palm",
            "ROCK_DynHandTwin_L_ThumbBase",
            "ROCK_DynHandTwin_L_ThumbMiddle",
            "ROCK_DynHandTwin_L_ThumbTip",
            "ROCK_DynHandTwin_L_IndexBase",
            "ROCK_DynHandTwin_L_IndexMiddle",
            "ROCK_DynHandTwin_L_IndexTip",
            "ROCK_DynHandTwin_L_MiddleBase",
            "ROCK_DynHandTwin_L_MiddleMiddle",
            "ROCK_DynHandTwin_L_MiddleTip",
            "ROCK_DynHandTwin_L_RingBase",
            "ROCK_DynHandTwin_L_RingMiddle",
            "ROCK_DynHandTwin_L_RingTip",
            "ROCK_DynHandTwin_L_PinkyBase",
            "ROCK_DynHandTwin_L_PinkyMiddle",
            "ROCK_DynHandTwin_L_PinkyTip",
            "ROCK_DynHandTwin_L_Forearm",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kRightTwinOwnerNames{
            "DynHandTwinR.Palm",
            "DynHandTwinR.ThumbBase",
            "DynHandTwinR.ThumbMiddle",
            "DynHandTwinR.ThumbTip",
            "DynHandTwinR.IndexBase",
            "DynHandTwinR.IndexMiddle",
            "DynHandTwinR.IndexTip",
            "DynHandTwinR.MiddleBase",
            "DynHandTwinR.MiddleMiddle",
            "DynHandTwinR.MiddleTip",
            "DynHandTwinR.RingBase",
            "DynHandTwinR.RingMiddle",
            "DynHandTwinR.RingTip",
            "DynHandTwinR.PinkyBase",
            "DynHandTwinR.PinkyMiddle",
            "DynHandTwinR.PinkyTip",
            "DynHandTwinR.Forearm",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kLeftTwinOwnerNames{
            "DynHandTwinL.Palm",
            "DynHandTwinL.ThumbBase",
            "DynHandTwinL.ThumbMiddle",
            "DynHandTwinL.ThumbTip",
            "DynHandTwinL.IndexBase",
            "DynHandTwinL.IndexMiddle",
            "DynHandTwinL.IndexTip",
            "DynHandTwinL.MiddleBase",
            "DynHandTwinL.MiddleMiddle",
            "DynHandTwinL.MiddleTip",
            "DynHandTwinL.RingBase",
            "DynHandTwinL.RingMiddle",
            "DynHandTwinL.RingTip",
            "DynHandTwinL.PinkyBase",
            "DynHandTwinL.PinkyMiddle",
            "DynHandTwinL.PinkyTip",
            "DynHandTwinL.Forearm",
        };

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
            return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
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
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
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
         * Contact-noise smoothing for the rendered hand: the solver resolves a
         * driven-into-surface twin slightly differently each substep, and the
         * raw deviation twitch is visible while the hand rests still. The
         * exponential filter only shapes CONTACT deviations (free space is
         * exactly zero and gated before application), so tracking latency is
         * untouched. speed <= 0 disables.
         */
        RE::NiPoint3 smoothAppliedDeviation(const RE::NiPoint3& applied, const RE::NiPoint3& target, float smoothingSpeed, float deltaSeconds)
        {
            if (!std::isfinite(smoothingSpeed) || smoothingSpeed <= 0.0f) {
                return target;
            }
            const float dt = std::clamp(std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
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
        telemetry.physicsRawDeltaSeconds.store(sample.physicsRawDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsRemainderDeltaSeconds.store(sample.physicsRemainderDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsAccumulatedDeltaSeconds.store(sample.physicsAccumulatedDeltaSeconds, std::memory_order_relaxed);
        telemetry.physicsSubstepProgress.store(sample.physicsSubstepProgress, std::memory_order_relaxed);
        telemetry.sourceGameFrameIndex.store(sample.sourceGameFrameIndex, std::memory_order_relaxed);
        telemetry.sourceQueueSequence.store(sample.sourceQueueSequence, std::memory_order_relaxed);
        telemetry.solveSequence.store(sample.solveSequence, std::memory_order_relaxed);
        telemetry.physicsSubstepCount.store(sample.physicsSubstepCount, std::memory_order_relaxed);
        telemetry.physicsSubstepIndex.store(sample.physicsSubstepIndex, std::memory_order_relaxed);
        telemetry.valid.store(sample.valid, std::memory_order_relaxed);
        telemetry.targetVelocityValid.store(sample.targetVelocityValid, std::memory_order_relaxed);
        telemetry.contactActive.store(sample.contactActive, std::memory_order_relaxed);
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
            sample.physicsRawDeltaSeconds = telemetry.physicsRawDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsRemainderDeltaSeconds = telemetry.physicsRemainderDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsAccumulatedDeltaSeconds = telemetry.physicsAccumulatedDeltaSeconds.load(std::memory_order_relaxed);
            sample.physicsSubstepProgress = telemetry.physicsSubstepProgress.load(std::memory_order_relaxed);
            sample.sourceGameFrameIndex = telemetry.sourceGameFrameIndex.load(std::memory_order_relaxed);
            sample.sourceQueueSequence = telemetry.sourceQueueSequence.load(std::memory_order_relaxed);
            sample.solveSequence = telemetry.solveSequence.load(std::memory_order_relaxed);
            sample.physicsSubstepCount = telemetry.physicsSubstepCount.load(std::memory_order_relaxed);
            sample.physicsSubstepIndex = telemetry.physicsSubstepIndex.load(std::memory_order_relaxed);
            sample.valid = telemetry.valid.load(std::memory_order_relaxed);
            sample.targetVelocityValid = telemetry.targetVelocityValid.load(std::memory_order_relaxed);
            sample.contactActive = telemetry.contactActive.load(std::memory_order_relaxed);
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
        slot.droveSourceGameFrameIndex = 0;
        slot.droveSourceQueueSequence = 0;
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
            .enabled = g_rockConfig.rockHandCollisionDynamicHapticsEnabled,
            .baseIntensity = g_rockConfig.rockHandCollisionDynamicHapticBaseIntensity,
            .maxIntensity = g_rockConfig.rockHandCollisionDynamicHapticMaxIntensity,
            .speedScale = g_rockConfig.rockHandCollisionDynamicHapticSpeedScale,
            .minApproachSpeedGameUnitsPerSecond = g_rockConfig.rockHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond,
            .cooldownSeconds = g_rockConfig.rockHandCollisionDynamicHapticCooldownSeconds,
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

    dynamic_hand_collision_telemetry::HapticEvents DynamicHandCollisionRuntime::consumeHapticEvents()
    {
        const auto events = _pendingHapticEvents;
        _pendingHapticEvents = {};
        return events;
    }

    bool DynamicHandCollisionRuntime::tryClassifyDynamicBodyContactSourceAtomic(
        const std::uint32_t bodyId,
        DynamicBodyContactSource& outSource) const noexcept
    {
        outSource = {};
        if (bodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return false;
        }
        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            for (std::size_t slot = 0; slot < kBodiesPerHand; ++slot) {
                if (_hands[hand].bodies[slot].bodyIdAtomic.load(
                        std::memory_order_acquire) != bodyId) {
                    continue;
                }
                outSource.valid = true;
                outSource.isLeft = hand == 1;
                outSource.slot = static_cast<std::uint8_t>(slot);
                outSource.bodyId = bodyId;
                return true;
            }
        }
        return false;
    }

    void DynamicHandCollisionRuntime::recordDynamicBodyContactCallback(
        const DynamicBodyContactSource& source,
        const bool otherIsHand,
        const bool otherIsWeapon) noexcept
    {
        if (!source.valid || source.slot >= kBodiesPerHand ||
            !_dynamicInteractionsEnabledAtomic.load(
                std::memory_order_acquire)) {
            return;
        }
        const std::uint32_t slotBit =
            1u << static_cast<std::uint32_t>(source.slot);
        auto& hand = _hands[source.isLeft ? 1u : 0u];
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
        dynamic_hand_surface_contact_state::ContactSource& outSource) const noexcept
    {
        outSource = {};
        if (bodyId == hand_semantic_contact_state::kInvalidBodyId) {
            return false;
        }

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            for (std::size_t slot = 0; slot < kBodiesPerHand; ++slot) {
                if (!dynamic_hand_collision_telemetry::
                        isSurfaceGrabSourceSlot(slot)) {
                    continue;
                }
                if (_hands[hand].bodies[slot].bodyIdAtomic.load(
                        std::memory_order_acquire) != bodyId) {
                    continue;
                }
                const bool palm = slot == kPalmSlot;
                const std::size_t fingerIndex =
                    dynamic_hand_collision_telemetry::
                        fingerIndexForBodyIndex(slot);
                const auto finger = palm ?
                    hand_collider_semantics::HandFinger::None :
                    static_cast<hand_collider_semantics::HandFinger>(
                        fingerIndex);
                const auto segment = palm ?
                    hand_collider_semantics::HandFingerSegment::None :
                    hand_collider_semantics::HandFingerSegment::Tip;
                outSource.valid = true;
                outSource.isLeft = hand == 1;
                outSource.slot = slot;
                outSource.role = palm ?
                    hand_collider_semantics::HandColliderRole::PalmAnchor :
                    hand_collider_semantics::roleForFingerSegment(
                        finger,
                        segment);
                outSource.finger = finger;
                outSource.segment = segment;
                outSource.bodyId = bodyId;
                return true;
            }
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
        const std::uint32_t otherLayer) noexcept
    {
        _surfaceProcessedPairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead ||
            !collision_layer_policy::isDynamicHandProxySurfaceLayer(otherLayer)) {
            return;
        }
        _surfaceEligiblePairSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (_surfaceContacts.record(source, otherBodyId, nullptr, nullptr)) {
            _surfaceContactPublishSequenceAtomic.fetch_add(1, std::memory_order_release);
        }
    }

    hand_semantic_contact_state::SemanticContactCollection
    DynamicHandCollisionRuntime::collectFreshSurfaceContacts(
        const bool isLeft,
        const std::uint32_t maximumAgeFrames) const noexcept
    {
        return _surfaceContacts.collectFresh(isLeft, maximumAgeFrames);
    }

    bool DynamicHandCollisionRuntime::beginSurfaceLatch(
        const bool isLeft,
        const std::uint32_t sourceBodyId,
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

        if (!g_rockConfig.rockHandCollisionDynamicDrive ||
            !frik_visual_authority::isAvailable() ||
            !world || targetBodyId == hand_semantic_contact_state::kInvalidBodyId ||
            sourceBodyId == hand_semantic_contact_state::kInvalidBodyId ||
            sourceBodyId == targetBodyId) {
            return reject(SurfaceLatchFailure::PrerequisiteUnavailable);
        }

        dynamic_hand_surface_contact_state::ContactSource source{};
        if (!tryClassifySurfaceContactSourceAtomic(sourceBodyId, source) ||
            source.isLeft != isLeft) {
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
         * Mesh authority moves only the presented hand. The dynamic twins keep
         * their already-solved shell transforms relative to the target, so the
         * oversized Havok shell remains a collision guard without becoming the
         * visible/API grab anchor. Translating the twins into the render mesh
         * would require body-wide no-collide and would discard nearby-world
         * collision fidelity while latched.
         */
        for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
            const auto& slot = handSlots.bodies[bodyIndex];
            if (!slot.created || slot.createdWorld != world) {
                continue;
            }
            RE::NiTransform proxyWorld{};
            if (!havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    slot.body.getBodyId(),
                    proxyWorld) ||
                !isFiniteTransform(proxyWorld)) {
                continue;
            }
            candidate.lastProxyWorld[bodyIndex] = proxyWorld;
            candidate.proxyInTargetBody[bodyIndex] =
                transform_math::composeTransforms(
                    targetInverse,
                    proxyWorld);
            candidate.proxyRelationshipValid[bodyIndex] = true;
            sourceCaptured = sourceCaptured ||
                             slot.body.getBodyId().value == sourceBodyId;
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

    bool DynamicHandCollisionRuntime::ensureSlotCreated(ProxySlot& slot,
        bool isLeft,
        std::size_t bodyIndex,
        const PhysicsFrameContext& frame,
        const Hand& hand,
        const BodyBoneColliderSet& bodyBoneColliders,
        const dynamic_hand_twin::TwinSlotFrame& twinFrame,
        std::uint64_t geometryGeneration)
    {
        if (slot.created) {
            if (slot.createdWorld == frame.hknpWorld) {
                // Animation transitions retain the old body even if a real
                // geometry rebuild is queued. The latest generation commits
                // once stable targets have restored collision.
                if (_transitionCollisionSuppressed ||
                    (slot.createdGeometryGeneration == geometryGeneration &&
                        !slot.rebuildRequestedAtomic.load(std::memory_order_acquire))) {
                    return true;
                }
            }
            retireSlot(slot, frame.bhkWorld);
        }

        if (!frame.hknpWorld || !frame.bhkWorld || geometryGeneration == 0 ||
            !twinFrame.valid || !isFiniteTransform(twinFrame.target)) {
            return false;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        auto* shape = bodyIndex >= kFirstForearmSlot ?
            bodyBoneColliders.buildDynamicForearmTwinShape(twinFrame) :
            hand.buildDynamicTwinShape(twinFrame, bodyIndex == kPalmSlot);
        if (!shape) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "{} dynamic hand twin {}: shape creation failed",
                isLeft ? "Left" : "Right",
                bodyIndex);
            return false;
        }

        const auto expectedFilterInfo = dynamicHandProxyFilterInfo(
            isLeft,
            _transitionCollisionSuppressed);
        if (!slot.body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                shape,
                expectedFilterInfo,
                havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                (isLeft ? kLeftTwinNames : kRightTwinNames)[bodyIndex],
                kTrackedDynamicBodyCreationOptions)) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "{} dynamic hand twin {}: body creation failed",
                isLeft ? "Left" : "Right",
                bodyIndex);
            havok_ref_count::release(shape);
            return false;
        }

        const auto proxyBodyId = slot.body.getBodyId();
        const auto createdBody = havok_runtime::snapshotBody(frame.hknpWorld, proxyBodyId);
        if (!createdBody.valid || createdBody.collisionFilterInfo != expectedFilterInfo) {
            ROCK_LOG_ERROR(
                Hand,
                "{} dynamic hand twin {} filter publication failed: body={} requested=0x{:08X} observed=0x{:08X} readable={}",
                isLeft ? "Left" : "Right",
                bodyIndex,
                proxyBodyId.value,
                expectedFilterInfo,
                createdBody.collisionFilterInfo,
                createdBody.valid ? "yes" : "no");
            slot.body.retireDeferred(frame.bhkWorld);
            havok_ref_count::release(shape);
            return false;
        }
        const bool surfaceContactSource =
            dynamic_hand_collision_telemetry::isSurfaceGrabSourceSlot(
                bodyIndex);
        bool processedManifoldEventsEnabled = false;
        if (surfaceContactSource) {
            /*
             * FO4VR's manifold event producers at 0x1418028B0 and
             * 0x141802A40 both gate their key-2 record on bit 0x40. Publish
             * that verified body flag only on palm/fingertip twins; forearm
             * contacts remain collision feedback and never seed a grab.
             */
            const bool processedManifoldFlagEnabled = havok_runtime::enableBodyFlags(
                frame.hknpWorld,
                proxyBodyId.value,
                kRaiseManifoldProcessedEvents,
                kRebuildBodyCollisionState);
            const auto flaggedBody = havok_runtime::snapshotBody(
                frame.hknpWorld,
                proxyBodyId);
            const bool processedManifoldFlagPublished =
                processedManifoldFlagEnabled &&
                flaggedBody.valid &&
                flaggedBody.body &&
                (flaggedBody.body->flags & kRaiseManifoldProcessedEvents) ==
                    kRaiseManifoldProcessedEvents;
            processedManifoldEventsEnabled = processedManifoldFlagPublished;
            if (!processedManifoldFlagPublished) {
                ROCK_LOG_ERROR(
                    Hand,
                    "{} dynamic hand twin {} failed processed-manifold event opt-in: body={} enabled={} readable={} flags=0x{:08X}",
                    isLeft ? "Left" : "Right",
                    bodyIndex,
                    proxyBodyId.value,
                    processedManifoldFlagEnabled,
                    flaggedBody.valid && flaggedBody.body,
                    flaggedBody.body ? flaggedBody.body->flags : 0u);
            }
        }

        slot.shape = shape;
        slot.createdWorld = frame.hknpWorld;
        slot.createdBhkWorld = frame.bhkWorld;
        slot.createdGeometryGeneration = geometryGeneration;
        slot.created = true;
        slot.bodyIdAtomic.store(proxyBodyId.value, std::memory_order_release);
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
        clearPhysicsContactState(slot);

        initializeGeneratedKeyframedBodyDriveState(slot.driveState, twinFrame.target);
        (void)placeGeneratedKeyframedBodyImmediately(slot.body, twinFrame.target);

        ROCK_LOG_INFO(Hand,
            "{} dynamic hand twin created: slot={} bodyId={} length={:.2f} radius={:.2f} layer={} group={} manifoldEvents={}",
            isLeft ? "Left" : "Right",
            bodyIndex,
            slot.body.getBodyId().value,
            twinFrame.length,
            twinFrame.radius,
            collision_layer_policy::dynamicHandProxyLayerForHand(isLeft),
            expectedFilterInfo >> 16,
            processedManifoldEventsEnabled ? "yes" : "no");
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
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerProbeDeltaOpenUnits,
                    .responseGain =
                        g_rockConfig.rockHandCollisionSurfaceFingerResponseGain,
                    .maximumDeflectionOpenUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerMaximumDeflectionOpenUnits,
                    .minimumHelpfulProbeTravelGameUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits,
                    .directionSwitchHysteresisFraction =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerDirectionSwitchHysteresisFraction,
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

        const RE::NiTransform inverseHand =
            transform_math::invertTransform(rawHandWorld);
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
                candidate.intentFramesInHand[linearIndex] =
                    transform_math::composeTransforms(
                        inverseHand,
                        twin.target);
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
                if (!isFiniteTransform(
                        candidate.intentFramesInHand[linearIndex]) ||
                    !isFinitePoint(
                        candidate.closingProbeTravelInHand[linearIndex]) ||
                    !isFinitePoint(
                        candidate.openingProbeTravelInHand[linearIndex])) {
                    return false;
                }
                candidate.intentValid[linearIndex] = true;
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
        if (!g_rockConfig.rockHandCollisionSurfaceFingerResponseEnabled) {
            clearSurfaceFingerResponse(handSlots, isLeft);
            return 0;
        }

        bool anyFingerContact = false;
        for (std::size_t bodyIndex =
                 dynamic_hand_collision_telemetry::kFirstFingerSlot;
             bodyIndex < kFirstForearmSlot;
             ++bodyIndex) {
            anyFingerContact = anyFingerContact ||
                               handTelemetry.twins[bodyIndex].contactActive;
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
                    if (!twin.contactActive ||
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
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerProbeDeltaOpenUnits,
                    .responseGain =
                        g_rockConfig.rockHandCollisionSurfaceFingerResponseGain,
                    .maximumDeflectionOpenUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerMaximumDeflectionOpenUnits,
                    .minimumHelpfulProbeTravelGameUnits =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits,
                    .directionSwitchHysteresisFraction =
                        g_rockConfig.
                            rockHandCollisionSurfaceFingerDirectionSwitchHysteresisFraction,
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
                    g_rockConfig.
                        rockHandCollisionSurfaceFingerSmoothingSpeed,
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
                std::isfinite(deltaSeconds) ? deltaSeconds : (1.0f / 90.0f),
                0.0f,
                0.1f);
            response.noContactSeconds += dt;
            if (response.noContactSeconds >=
                g_rockConfig.
                    rockHandCollisionSurfaceFingerReleaseDelaySeconds) {
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
                g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
            clearSurfaceFingerResponse(handSlots, isLeft);
            return 0;
        }
        response.posePublished = true;
        if (!frik_visual_authority::isHandPoseTagActive(
                SURFACE_FINGER_POSE_TAG,
                frik_visual_authority::handFromBool(isLeft))) {
            /*
             * Another tagged pose won hFRIK arbitration. Keep the immutable
             * intent alive so this claim can recover when authority returns,
             * but do not report any curl as a physical resolution while it is
             * not actually driving the rendered fingers.
             */
            response.lastHelpfulDynamicSlotMask = 0;
        }
        return response.lastHelpfulDynamicSlotMask;
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
                suppressCollision);
            for (auto& slot : handSlots.bodies) {
                if (!slot.created || !slot.body.isValid() || slot.createdWorld != world) {
                    continue;
                }
                slot.body.setCollisionFilterInfo(filterInfo, 1);
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
        _dynamicInteractionsEnabledAtomic.store(false, std::memory_order_release);
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

        _surfaceContacts.advanceFrame();
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
        telemetry.runtimeEnabled = g_rockConfig.rockHandCollisionDynamicDrive;
        telemetry.worldReady = frame.worldReady;
        telemetry.menuBlocked = frame.menuBlocked;
        telemetry.physicsWritesAllowed = physicsWritesAllowed;
        telemetry.hands[0].isLeft = false;
        telemetry.hands[1].isLeft = true;

        const bool dynamicInteractionsEnabled =
            g_rockConfig.rockHandDynamicInteractionsEnabled &&
            g_rockConfig.rockHandCollisionDynamicDrive;
        _dynamicInteractionsEnabledAtomic.store(
            dynamicInteractionsEnabled,
            std::memory_order_release);
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
            handTelemetry.dynamicInteractionsEnabled =
                dynamicInteractionsEnabled;
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

        if (!g_rockConfig.rockHandCollisionDynamicDrive) {
            const auto hasCreatedTwin = [](const HandSlots& handSlots) {
                return std::any_of(handSlots.bodies.begin(), handSlots.bodies.end(), [](const ProxySlot& slot) {
                    return slot.created;
                });
            };
            if (hasCreatedTwin(_hands[0]) || hasCreatedTwin(_hands[1])) {
                retireAll(frame.bhkWorld);
            }
            _hands[0].hapticState = {};
            _hands[1].hapticState = {};
            _transitionState = {};
            _transitionCollisionSuppressed = false;
            _transitionCollisionSuppressedAtomic.store(
                false,
                std::memory_order_release);
            telemetry.transitionCollisionSuppressed = false;
            _telemetrySnapshot = telemetry;
            return;
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

        if (++_logCounter >= 360) {
            _logCounter = 0;
            ROCK_LOG_DEBUG(Hand,
                "DynamicHandCollision active: twinsPerHand={} maxLinVelHk={:.1f} divergenceTeleport={:.1f} minDeviation={:.3f} smoothingSpeed={:.1f} haptics={} priority={} palmCreated R={} L={} surfaceCallbacks(impulse/manifold/eligible/published)={}/{}/{}/{}",
                kBodiesPerHand,
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed,
                g_rockConfig.rockHandCollisionDynamicHapticsEnabled ? "yes" : "no",
                g_rockConfig.rockHandCollisionDynamicVisualPriority,
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
            const auto& handSlots = _hands[handIndex(isLeft)];
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                const auto* twinFrame = twinFrameForSlot(handTwins, forearmTwins, isLeft, bodyIndex);
                const bool targetRequired = bodyIndex == kPalmSlot || handSlots.bodies[bodyIndex].created;
                if (targetRequired &&
                    (!twinFrame || !twinFrame->valid || !isFiniteTransform(twinFrame->target))) {
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
             * The drive keeps chasing the published role frames even while
             * another system owns the hand pose: parked twins recover through
             * the divergence teleport, and continuing to track keeps the
             * unsuppress handoff seamless. Ownership gates only decide VISUAL
             * authority.
             */
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& slot = handSlots.bodies[bodyIndex];
                auto& twinTelemetry = handTelemetry.twins[bodyIndex];
                twinTelemetry.role = dynamic_hand_collision_telemetry::roleForBodyIndex(bodyIndex);
                const auto* twinFrame = twinFrameForSlot(handTwins, forearmTwins, isLeft, bodyIndex);
                if (!twinFrame || !twinFrame->valid) {
                    if (bodyIndex >= kFirstForearmSlot && slot.created && !_transitionCollisionSuppressed) {
                        retireSlot(slot, frame.bhkWorld);
                    }
                    continue;
                }
                twinTelemetry.lengthGameUnits = twinFrame->length;
                twinTelemetry.radiusGameUnits = twinFrame->radius;
                twinTelemetry.convexRadiusGameUnits = twinFrame->convexRadius;
                twinTelemetry.handTargetResponseScale =
                    dynamic_hand_collision_kinematics::sanitizeHandTargetResponseScale(twinFrame->handTargetResponseScale);
                const auto geometryGeneration = bodyIndex >= kFirstForearmSlot ?
                    forearmTwins.geometryGeneration :
                    handTwins.geometryGeneration;
                if (!ensureSlotCreated(
                        slot,
                        isLeft,
                        bodyIndex,
                        frame,
                        hand,
                        bodyBoneColliders,
                        *twinFrame,
                        geometryGeneration)) {
                    continue;
                }
                RE::NiTransform driveTarget =
                    handSlots.surfaceLatch.active &&
                        handSlots.surfaceLatch.proxyRelationshipValid[bodyIndex] ?
                    handSlots.surfaceLatch.lastProxyWorld[bodyIndex] :
                    twinFrame->target;
                if (!handSlots.surfaceLatch.active &&
                    handSlots.surfaceFingerResponse.active &&
                    dynamic_hand_collision_telemetry::isFingerSlot(bodyIndex)) {
                    const std::size_t linearIndex =
                        bodyIndex -
                        dynamic_hand_collision_telemetry::kFirstFingerSlot;
                    if (linearIndex <
                            handSlots.surfaceFingerResponse.intentFramesInHand
                                .size() &&
                        handSlots.surfaceFingerResponse
                            .intentValid[linearIndex]) {
                        driveTarget = transform_math::composeTransforms(
                            handInput.rawHandWorld,
                            handSlots.surfaceFingerResponse
                                .intentFramesInHand[linearIndex]);
                    }
                }
                twinTelemetry.publishedTargetValid =
                    isFiniteTransform(driveTarget);
                twinTelemetry.publishedTargetWorld = driveTarget;
                (void)queueGeneratedKeyframedBodyTarget(
                    slot.driveState,
                    driveTarget,
                    frame.deltaSeconds,
                    g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                    frame.gameFrameIndex);
                twinTelemetry.bodyCreated = slot.created;
                twinTelemetry.bodyId = slot.created ? slot.body.getBodyId().value : dynamic_hand_collision_telemetry::kInvalidBodyId;

                PhysicsTelemetrySample physicsSample{};
                std::uint64_t physicsSampleSequence = 0;
                if (!readPhysicsTelemetry(slot, physicsSample, physicsSampleSequence) || !physicsSample.valid ||
                    !isFinitePoint(physicsSample.requestedTargetWorldGame) ||
                    !isFinitePoint(physicsSample.commandedTargetWorldGame) ||
                    !isFinitePoint(physicsSample.liveBodyWorldGame)) {
                    continue;
                }

                twinTelemetry.physicsSampleSequence = physicsSampleSequence;
                twinTelemetry.sourceGameFrameIndex = physicsSample.sourceGameFrameIndex;
                twinTelemetry.sourceQueueSequence = physicsSample.sourceQueueSequence;
                twinTelemetry.solveSequence = physicsSample.solveSequence;
                twinTelemetry.physicsSampleValid = true;
                twinTelemetry.targetVelocityValid = physicsSample.targetVelocityValid;
                twinTelemetry.contactActive = physicsSample.contactActive;
                twinTelemetry.recoveryTeleport = physicsSample.recoveryTeleport;
                twinTelemetry.requestedTargetWorldGame = physicsSample.requestedTargetWorldGame;
                twinTelemetry.commandedTargetWorldGame = physicsSample.commandedTargetWorldGame;
                twinTelemetry.liveBodyWorldGame = physicsSample.liveBodyWorldGame;
                twinTelemetry.targetVelocityWorldGameUnitsPerSecond = physicsSample.targetVelocityWorldGameUnitsPerSecond;
                twinTelemetry.approachSpeedGameUnitsPerSecond = physicsSample.approachSpeedGameUnitsPerSecond;
                twinTelemetry.physicsDeltaSeconds = physicsSample.physicsDeltaSeconds;
                twinTelemetry.physicsRawDeltaSeconds = physicsSample.physicsRawDeltaSeconds;
                twinTelemetry.physicsRemainderDeltaSeconds = physicsSample.physicsRemainderDeltaSeconds;
                twinTelemetry.physicsAccumulatedDeltaSeconds = physicsSample.physicsAccumulatedDeltaSeconds;
                twinTelemetry.physicsSubstepProgress = physicsSample.physicsSubstepProgress;
                twinTelemetry.physicsSubstepCount = physicsSample.physicsSubstepCount;
                twinTelemetry.physicsSubstepIndex = physicsSample.physicsSubstepIndex;
                twinTelemetry.solverResidualWorldGame = subtractPoints(
                    twinTelemetry.liveBodyWorldGame,
                    twinTelemetry.commandedTargetWorldGame);
                twinTelemetry.requestedGapWorldGame = subtractPoints(
                    twinTelemetry.liveBodyWorldGame,
                    twinTelemetry.requestedTargetWorldGame);
                twinTelemetry.solverResidualGameUnits = pointLength(twinTelemetry.solverResidualWorldGame);
                twinTelemetry.requestedGapGameUnits = pointLength(twinTelemetry.requestedGapWorldGame);

                if (physicsSample.contactActive) {
                    twinTelemetry.contactDeviationWorldGame = twinTelemetry.requestedGapWorldGame;
                    twinTelemetry.contactDeviationGameUnits = twinTelemetry.requestedGapGameUnits;
                    // Palm/finger points are the hand target (scale 1). A
                    // forearm point has less shoulder-lever response to that
                    // target, so its source publication maps the blocked point
                    // displacement into the IK target displacement first.
                    twinTelemetry.handTargetCorrectionWorldGame = RE::NiPoint3{
                        twinTelemetry.contactDeviationWorldGame.x * twinTelemetry.handTargetResponseScale,
                        twinTelemetry.contactDeviationWorldGame.y * twinTelemetry.handTargetResponseScale,
                        twinTelemetry.contactDeviationWorldGame.z * twinTelemetry.handTargetResponseScale,
                    };
                    twinTelemetry.handTargetCorrectionGameUnits = pointLength(twinTelemetry.handTargetCorrectionWorldGame);
                    deviations[bodyIndex] = twinTelemetry.handTargetCorrectionWorldGame;
                    deviationValid[bodyIndex] = true;
                    handTelemetry.contactMask |= 1u << static_cast<std::uint32_t>(bodyIndex);
                    ++handTelemetry.contactCount;
                }
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
             * contact-smoothing rate.
             */
            bool teleportedThisFrame = false;
            for (auto& slot : handSlots.bodies) {
                if (slot.teleportedAtomic.exchange(false, std::memory_order_acq_rel)) {
                    teleportedThisFrame = true;
                }
            }
            const float recoveryDuration = g_rockConfig.rockHandCollisionDynamicTeleportRecoverySeconds;
            if (teleportedThisFrame && recoveryDuration > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = recoveryDuration;
            }
            const float frameDt = std::clamp(std::isfinite(frame.deltaSeconds) ? frame.deltaSeconds : (1.0f / 90.0f), 0.0f, 0.1f);
            float smoothingSpeed = 0.0f;
            if (handSlots.teleportRecoverySecondsRemaining > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = std::max(0.0f, handSlots.teleportRecoverySecondsRemaining - frameDt);
                const float recoverySpeed = 3.0f / std::max(recoveryDuration, 0.05f);
                smoothingSpeed = g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed;
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
            const float minDeviation = g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits;
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
                    g_rockConfig.rockHandCollisionDynamicVisualPriority)) {
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
        const bool interactionsEnabled =
            _dynamicInteractionsEnabledAtomic.load(
                std::memory_order_acquire);
        const std::uint32_t weaponBodyId =
            _desiredWeaponBodyIdAtomic.load(std::memory_order_acquire);
        if (interactionsEnabled && world &&
            weaponBodyId != hand_semantic_contact_state::kInvalidBodyId) {
            for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
                if (!_weaponOwnedAtomic[hand].load(
                        std::memory_order_acquire)) {
                    continue;
                }
                for (const auto& slot : _hands[hand].bodies) {
                    const std::uint32_t handBodyId =
                        slot.bodyIdAtomic.load(std::memory_order_acquire);
                    if (handBodyId ==
                            hand_semantic_contact_state::kInvalidBodyId ||
                        desiredPairCount >= desiredPairs.size()) {
                        continue;
                    }
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

        if (!g_rockConfig.rockHandCollisionDynamicDrive || !world) {
            return;
        }

        for (std::size_t hand = 0; hand < _hands.size(); ++hand) {
            const bool isLeft = hand == 1;
            for (std::size_t bodyIndex = 0; bodyIndex < kBodiesPerHand; ++bodyIndex) {
                auto& slot = _hands[hand].bodies[bodyIndex];
                if (!slot.created || slot.createdWorld != world) {
                    continue;
                }

                const float divergenceThreshold = g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits;
                const bool teleportArmed =
                    slot.divergenceDwellSeconds >= g_rockConfig.rockHandCollisionDynamicDivergenceTeleportDwellSeconds;
                GeneratedBodyDriveMode mode{
                    .dynamicVelocity = true,
                    .divergenceTeleportGameUnits = teleportArmed ? divergenceThreshold : 0.0f,
                };

                /*
                 * Established contact -> lean, don't slam: cap the commanded
                 * velocity component pressing INTO the contact (press direction
                 * = from body toward target = minus the deviation direction).
                 * Tangential slide and retreat keep full drive speed.
                 */
                constexpr float kPressCapActivationDeviationGameUnits = 0.25f;
                const float pressCapHavok = g_rockConfig.rockHandCollisionDynamicContactPressMaxVelocityHavok;
                if (pressCapHavok > 0.0f && slot.lastPostSolveDeviationValid) {
                    const auto& deviation = slot.lastPostSolveDeviationGame;
                    const float deviationLength = std::sqrt(
                        deviation.x * deviation.x + deviation.y * deviation.y + deviation.z * deviation.z);
                    if (std::isfinite(deviationLength) && deviationLength > kPressCapActivationDeviationGameUnits) {
                        mode.hasContactPressDirection = true;
                        mode.contactPressDirection[0] = -deviation.x / deviationLength;
                        mode.contactPressDirection[1] = -deviation.y / deviationLength;
                        mode.contactPressDirection[2] = -deviation.z / deviationLength;
                        mode.contactPressMaxVelocityHavok = pressCapHavok;
                    }
                }

                const auto result = driveGeneratedKeyframedBody(
                    world,
                    slot.body,
                    slot.driveState,
                    timing,
                    (isLeft ? kLeftTwinOwnerNames : kRightTwinOwnerNames)[bodyIndex],
                    static_cast<std::uint32_t>(bodyIndex),
                    g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                    0.0f,
                    mode);

                if (result.shouldRequestRebuild()) {
                    slot.rebuildRequestedAtomic.store(true, std::memory_order_release);
                    clearPhysicsContactState(slot);
                    continue;
                }

                /*
                 * The deviation itself is sampled POST-SOLVE against these
                 * exact targets (samplePostSolveDeviations); here we only
                 * record what was commanded and what was requested. Teleports
                 * sample the same way: the body was placed at the target
                 * pre-collide, so the post-solve read reports the solver's
                 * ejection (if any) and the rendered hand glides through the
                 * recovery instead of snapping to zero.
                 */
                if (result.driven) {
                    slot.commandedTargetGame = result.targetGamePosition;
                    slot.requestedTargetGame = result.requestedTargetGamePosition;
                    const float havokToGameScale = physics_scale::havokToGame();
                    slot.droveTargetVelocityValid =
                        result.hasSampledTargetLinearVelocityHavok && physics_scale::isUsableScale(havokToGameScale);
                    slot.droveTargetVelocityGameUnitsPerSecond = slot.droveTargetVelocityValid ?
                        RE::NiPoint3{
                            result.sampledTargetLinearVelocityHavok.x * havokToGameScale,
                            result.sampledTargetLinearVelocityHavok.y * havokToGameScale,
                            result.sampledTargetLinearVelocityHavok.z * havokToGameScale,
                        } :
                        RE::NiPoint3{};
                    slot.drovePhysicsDeltaSeconds = result.driveDeltaSeconds;
                    slot.droveSourceGameFrameIndex = result.sourceFrameIndex;
                    slot.droveSourceQueueSequence = result.sourceSequence;
                    slot.droveRecoveryTeleport = result.teleported;
                    slot.droveThisSubstep = true;
                } else {
                    clearPhysicsContactState(slot);
                }

                /*
                 * Dwell accounting runs on the REQUESTED-target gap. The
                 * commanded-target delta (bodyDeltaGameUnits) saturates at the
                 * velocity-limit distance, far below any useful divergence
                 * threshold, which silently turned the recovery teleport into
                 * dead code in earlier revisions.
                 */
                const float requestedGapGameUnits = result.hasLiveBodyTransform
                    ? std::sqrt(
                          (result.liveBodyGamePosition.x - result.requestedTargetGamePosition.x) * (result.liveBodyGamePosition.x - result.requestedTargetGamePosition.x) +
                          (result.liveBodyGamePosition.y - result.requestedTargetGamePosition.y) * (result.liveBodyGamePosition.y - result.requestedTargetGamePosition.y) +
                          (result.liveBodyGamePosition.z - result.requestedTargetGamePosition.z) * (result.liveBodyGamePosition.z - result.requestedTargetGamePosition.z))
                    : 0.0f;
                if (result.teleported) {
                    slot.divergenceDwellSeconds = 0.0f;
                    slot.teleportedAtomic.store(true, std::memory_order_release);
                } else if (result.attempted &&
                           result.hasLiveBodyTransform &&
                           std::isfinite(requestedGapGameUnits) &&
                           divergenceThreshold > 0.0f &&
                           requestedGapGameUnits > divergenceThreshold) {
                    slot.divergenceDwellSeconds += std::clamp(result.driveDeltaSeconds, 0.0f, 0.1f);
                } else {
                    slot.divergenceDwellSeconds = 0.0f;
                }
            }
        }
    }

    void DynamicHandCollisionRuntime::samplePostSolveDeviations(
        RE::hknpWorld* world,
        const std::uint64_t solveSequence,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionPostSolve);

        if (!g_rockConfig.rockHandCollisionDynamicDrive || !world) {
            return;
        }

        for (auto& handSlots : _hands) {
            std::uint32_t contactMask = 0;
            float maxEntryApproachSpeed = 0.0f;

            for (std::size_t bodyIndex = 0; bodyIndex < handSlots.bodies.size(); ++bodyIndex) {
                auto& slot = handSlots.bodies[bodyIndex];
                if (!slot.created || slot.createdWorld != world || !slot.droveThisSubstep) {
                    continue;
                }
                slot.droveThisSubstep = false;

                RE::NiTransform liveWorld{};
                if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, slot.body.getBodyId(), liveWorld) ||
                    !isFinitePoint(liveWorld.translate)) {
                    clearPhysicsContactState(slot);
                    continue;
                }

                /*
                 * Contact discriminator: an unobstructed hard-keyframe drive
                 * lands exactly on the COMMANDED (velocity-limited) target, so
                 * the solver residual against it is integration noise in free
                 * space and at least the press-cap penetration step (~0.26 gu
                 * at 270 Hz) when the solver blocked the body. Only in contact
                 * does the render deviation get published — measured against
                 * the REQUESTED target so it equals the true blocked depth
                 * instead of saturating at the dt-dependent limiter distance.
                 */
                const RE::NiPoint3 solverResidual{
                    liveWorld.translate.x - slot.commandedTargetGame.x,
                    liveWorld.translate.y - slot.commandedTargetGame.y,
                    liveWorld.translate.z - slot.commandedTargetGame.z,
                };
                const float residualLength = std::sqrt(
                    solverResidual.x * solverResidual.x +
                    solverResidual.y * solverResidual.y +
                    solverResidual.z * solverResidual.z);
                const float contactThreshold = slot.lastPostSolveContact
                    ? kContactResidualStayGameUnits
                    : kContactResidualEnterGameUnits;
                const bool contact = std::isfinite(residualLength) && residualLength > contactThreshold;
                slot.lastPostSolveContact = contact;

                RE::NiPoint3 deviation{};
                float approachSpeedGameUnitsPerSecond = 0.0f;
                if (contact) {
                    deviation = RE::NiPoint3{
                        liveWorld.translate.x - slot.requestedTargetGame.x,
                        liveWorld.translate.y - slot.requestedTargetGame.y,
                        liveWorld.translate.z - slot.requestedTargetGame.z,
                    };
                    if (!slot.droveRecoveryTeleport) {
                        if (slot.droveTargetVelocityValid) {
                            approachSpeedGameUnitsPerSecond =
                                dynamic_hand_collision_feedback::projectedApproachSpeedGameUnitsPerSecond(
                                    slot.droveTargetVelocityGameUnitsPerSecond,
                                    deviation);
                        }
                    }
                    contactMask |= 1u << static_cast<std::uint32_t>(bodyIndex);
                    maxEntryApproachSpeed = std::max(maxEntryApproachSpeed, approachSpeedGameUnitsPerSecond);
                }
                slot.lastPostSolveDeviationGame = deviation;
                slot.lastPostSolveDeviationValid = contact;

                publishPhysicsTelemetry(slot,
                    PhysicsTelemetrySample{
                        .requestedTargetWorldGame = slot.requestedTargetGame,
                        .commandedTargetWorldGame = slot.commandedTargetGame,
                        .liveBodyWorldGame = liveWorld.translate,
                        .targetVelocityWorldGameUnitsPerSecond = slot.droveTargetVelocityGameUnitsPerSecond,
                        .approachSpeedGameUnitsPerSecond = approachSpeedGameUnitsPerSecond,
                        .physicsDeltaSeconds = slot.drovePhysicsDeltaSeconds,
                        .physicsRawDeltaSeconds = timing.rawDeltaSeconds,
                        .physicsRemainderDeltaSeconds = timing.remainderDeltaSeconds,
                        .physicsAccumulatedDeltaSeconds = timing.accumulatedDeltaSeconds,
                        .physicsSubstepProgress = timing.substepProgress,
                        .sourceGameFrameIndex = slot.droveSourceGameFrameIndex,
                        .sourceQueueSequence = slot.droveSourceQueueSequence,
                        .solveSequence = solveSequence,
                        .physicsSubstepCount = timing.substepCount,
                        .physicsSubstepIndex = timing.substepIndex,
                        .valid = true,
                        .targetVelocityValid = slot.droveTargetVelocityValid,
                        .contactActive = contact,
                        .recoveryTeleport = slot.droveRecoveryTeleport,
                    });
            }

            const bool anyContact = contactMask != 0;
            if (anyContact && !handSlots.physicsContactActive) {
                handSlots.contactEntryApproachSpeedAtomic.store(maxEntryApproachSpeed, std::memory_order_relaxed);
                handSlots.contactEntryMaskAtomic.store(contactMask, std::memory_order_relaxed);
                handSlots.contactEntrySequenceAtomic.fetch_add(1, std::memory_order_release);
            }
            handSlots.physicsContactActive = anyContact;
        }
    }
}

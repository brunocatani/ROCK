#include "physics-interaction/hand/DynamicHandCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
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
        constexpr std::uint32_t kDynamicHandProxyCollisionGroup = 0x000C;
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
            "ROCK_DynHandTwin_R_ThumbTip",
            "ROCK_DynHandTwin_R_IndexTip",
            "ROCK_DynHandTwin_R_MiddleTip",
            "ROCK_DynHandTwin_R_RingTip",
            "ROCK_DynHandTwin_R_PinkyTip",
            "ROCK_DynHandTwin_R_Forearm",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kLeftTwinNames{
            "ROCK_DynHandTwin_L_Palm",
            "ROCK_DynHandTwin_L_ThumbTip",
            "ROCK_DynHandTwin_L_IndexTip",
            "ROCK_DynHandTwin_L_MiddleTip",
            "ROCK_DynHandTwin_L_RingTip",
            "ROCK_DynHandTwin_L_PinkyTip",
            "ROCK_DynHandTwin_L_Forearm",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kRightTwinOwnerNames{
            "DynHandTwinR.Palm",
            "DynHandTwinR.Thumb",
            "DynHandTwinR.Index",
            "DynHandTwinR.Middle",
            "DynHandTwinR.Ring",
            "DynHandTwinR.Pinky",
            "DynHandTwinR.Forearm",
        };
        constexpr std::array<const char*, DynamicHandCollisionRuntime::kBodiesPerHand> kLeftTwinOwnerNames{
            "DynHandTwinL.Palm",
            "DynHandTwinL.Thumb",
            "DynHandTwinL.Index",
            "DynHandTwinL.Middle",
            "DynHandTwinL.Ring",
            "DynHandTwinL.Pinky",
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

        std::uint32_t dynamicHandProxyFilterInfo(bool suppressCollision = false)
        {
            const auto baseFilter =
                (kDynamicHandProxyCollisionGroup << 16) |
                (collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY & collision_layer_policy::FO4_LAYER_FILTER_MASK);
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
                const std::size_t fingerIndex = bodyIndex - 1;
                return fingerIndex < handTwins.fingertips.size() ? &handTwins.fingertips[fingerIndex] : nullptr;
            }

            const std::size_t forearmIndex = bodyIndex - DynamicHandCollisionRuntime::kFirstForearmSlot;
            const auto& sideForearms = forearmTwins.forHand(isLeft);
            return forearmIndex < sideForearms.size() ? &sideForearms[forearmIndex] : nullptr;
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

        if (!slot.body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                shape,
                dynamicHandProxyFilterInfo(_transitionCollisionSuppressed),
                havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld),
                BethesdaMotionType::Dynamic,
                (isLeft ? kLeftTwinNames : kRightTwinNames)[bodyIndex])) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                5000,
                "{} dynamic hand twin {}: body creation failed",
                isLeft ? "Left" : "Right",
                bodyIndex);
            havok_ref_count::release(shape);
            return false;
        }

        slot.shape = shape;
        slot.createdWorld = frame.hknpWorld;
        slot.createdBhkWorld = frame.bhkWorld;
        slot.createdGeometryGeneration = geometryGeneration;
        slot.created = true;
        slot.rebuildRequestedAtomic.store(false, std::memory_order_release);
        clearPhysicsContactState(slot);

        initializeGeneratedKeyframedBodyDriveState(slot.driveState, twinFrame.target);
        (void)placeGeneratedKeyframedBodyImmediately(slot.body, twinFrame.target);

        ROCK_LOG_INFO(Hand,
            "{} dynamic hand twin created: slot={} bodyId={} length={:.2f} radius={:.2f} layer={}",
            isLeft ? "Left" : "Right",
            bodyIndex,
            slot.body.getBodyId().value,
            twinFrame.length,
            twinFrame.radius,
            collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY);
        return true;
    }

    void DynamicHandCollisionRuntime::retireSlot(ProxySlot& slot, void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
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
        clearVisual(handSlots, isLeft);
        for (auto& slot : handSlots.bodies) {
            retireSlot(slot, bhkWorld);
        }
        handSlots.physicsContactActive = false;
        handSlots.contactEntrySequenceAtomic.store(0, std::memory_order_release);
        handSlots.contactEntryApproachSpeedAtomic.store(0.0f, std::memory_order_relaxed);
        handSlots.contactEntryMaskAtomic.store(0, std::memory_order_relaxed);
        handSlots.hapticState = {};
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
        const auto filterInfo = dynamicHandProxyFilterInfo(suppressCollision);
        for (auto& handSlots : _hands) {
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
    }

    void DynamicHandCollisionRuntime::reset()
    {
        retireAll(nullptr);
        _telemetrySnapshot = {};
        _pendingHapticEvents = {};
        _telemetryUpdateSequence = 0;
        _logCounter = 0;
        _transitionState = {};
        _transitionCollisionSuppressed = false;
        _transitionCollisionSuppressedAtomic.store(false, std::memory_order_release);
    }

    void DynamicHandCollisionRuntime::updateFrame(const PhysicsFrameContext& frame,
        bool physicsWritesAllowed,
        const Hand& rightHand,
        const Hand& leftHand,
        const BodyBoneColliderSet& bodyBoneColliders,
        bool rightHandWeaponEquipped,
        bool leftSupportGripActive,
        bool rightVisualReturnActive,
        bool leftVisualReturnActive)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionFrame);

        _pendingHapticEvents = {};
        dynamic_hand_collision_telemetry::Snapshot telemetry{};
        telemetry.updateSequence = ++_telemetryUpdateSequence;
        telemetry.runtimeEnabled = g_rockConfig.rockHandCollisionDynamicDrive;
        telemetry.worldReady = frame.worldReady;
        telemetry.menuBlocked = frame.menuBlocked;
        telemetry.physicsWritesAllowed = physicsWritesAllowed;
        telemetry.hands[0].isLeft = false;
        telemetry.hands[1].isLeft = true;

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
                "DynamicHandCollision active: twinsPerHand={} maxLinVelHk={:.1f} divergenceTeleport={:.1f} minDeviation={:.3f} smoothingSpeed={:.1f} haptics={} priority={} palmCreated R={} L={}",
                kBodiesPerHand,
                g_rockConfig.rockHandCollisionDynamicMaxLinearVelocityHavok,
                g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowMinDeviationGameUnits,
                g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed,
                g_rockConfig.rockHandCollisionDynamicHapticsEnabled ? "yes" : "no",
                g_rockConfig.rockHandCollisionDynamicVisualPriority,
                _hands[0].bodies[kPalmSlot].created ? "yes" : "no",
                _hands[1].bodies[kPalmSlot].created ? "yes" : "no");
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

            if (handInput.disabled) {
                clearVisual(handSlots, isLeft);
                updateHandHaptic(handSlots, handTelemetry, false, frame.deltaSeconds);
                return;
            }

            const auto& handTwins = hand.dynamicTwinTargets();
            const auto& forearmTwins = bodyBoneColliders.dynamicForearmTwinTargets();
            std::array<RE::NiPoint3, kBodiesPerHand> deviations{};
            std::array<bool, kBodiesPerHand> deviationValid{};

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
                twinTelemetry.publishedTargetValid = true;
                twinTelemetry.publishedTargetWorld = twinFrame->target;
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
                (void)queueGeneratedKeyframedBodyTarget(
                    slot.driveState,
                    twinFrame->target,
                    frame.deltaSeconds,
                    g_rockConfig.rockHandCollisionDynamicDivergenceTeleportGameUnits);
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
            const RE::NiPoint3 combined =
                handTelemetry.anyContact ? combineTwinDeviations(deviations, deviationValid) : RE::NiPoint3{};
            handTelemetry.combinedContactDeviationWorldGame = combined;
            handTelemetry.combinedContactDeviationGameUnits = pointLength(combined);

            const bool physicallyOwnedByStrongerSystem =
                suppressesGeneratedHandContactEvidence(hand.getState()) ||
                weaponOwned ||
                _transitionCollisionSuppressed;
            const bool visuallyOwnedByStrongerSystem = physicallyOwnedByStrongerSystem || visualReturnActive;
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
            float smoothingSpeed = g_rockConfig.rockHandCollisionDynamicRenderFollowSmoothingSpeed;
            if (handSlots.teleportRecoverySecondsRemaining > 0.0f) {
                handSlots.teleportRecoverySecondsRemaining = std::max(0.0f, handSlots.teleportRecoverySecondsRemaining - frameDt);
                const float recoverySpeed = 3.0f / std::max(recoveryDuration, 0.05f);
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

        updateHand(false, frame.right, rightHand, rightHandWeaponEquipped, rightVisualReturnActive);
        updateHand(true, frame.left, leftHand, leftSupportGripActive, leftVisualReturnActive);
        telemetry.transitionCollisionSuppressed =
            _transitionCollisionSuppressed;
        _telemetrySnapshot = telemetry;
    }

    void DynamicHandCollisionRuntime::flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::DynamicHandCollisionPhysicsDrive);

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

    void DynamicHandCollisionRuntime::samplePostSolveDeviations(RE::hknpWorld* world)
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

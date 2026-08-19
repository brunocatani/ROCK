#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabMath.h"
#include "physics-interaction/hand/HandGrabTrace.h"
#include "physics-interaction/hand/HandGrabVisualDetail.h"
#include "physics-interaction/hand/HandGrabContactEvidence.h"
#include "physics-interaction/hand/HandGrabFingerPose.h"
#include "physics-interaction/hand/HandGrabSupportModel.h"
#include "physics-interaction/hand/HandGrabOffsetSources.h"
#include "physics-interaction/hand/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/HandGrabPivotAuthority.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/HavokOffsets.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/debug/GrabClockDebugFeed.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/native/SceneWriterProbe.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/PhysicsShapeCast.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <algorithm>
#include <array>
#include <atomic>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <xmmintrin.h>

namespace rock
{
    using namespace hand_grab_detail;

    namespace
    {
        RE::NiPoint3 rotationAxisProxyLocal(const RE::NiMatrix3& proxyWorldRotation, const RE::NiPoint3& axisWorld)
        {
            RE::NiTransform proxyWorld = transform_math::makeIdentityTransform<RE::NiTransform>();
            proxyWorld.rotate = proxyWorldRotation;
            return normalizeOrZero(hand_bone_collider_geometry_math::generatedColliderWorldVectorToLocal(proxyWorld, axisWorld));
        }

        bool computeHardKeyframeVelocityForTarget(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            const RE::NiTransform& targetWorld,
            float deltaTime,
            float outLinearVelocityHavok[4],
            float outAngularVelocityRadians[4])
        {
            /*
             * The generated keyframed proxy uses FO4VR's native hard-keyframe
             * helper for velocity telemetry when the live palm motion is not
             * directly readable. Held-object angular correction is solver-owned
             * by the grab constraint's ragdoll atom motor.
             */
            if (outLinearVelocityHavok) {
                outLinearVelocityHavok[0] = 0.0f;
                outLinearVelocityHavok[1] = 0.0f;
                outLinearVelocityHavok[2] = 0.0f;
                outLinearVelocityHavok[3] = 0.0f;
            }
            if (outAngularVelocityRadians) {
                outAngularVelocityRadians[0] = 0.0f;
                outAngularVelocityRadians[1] = 0.0f;
                outAngularVelocityRadians[2] = 0.0f;
                outAngularVelocityRadians[3] = 0.0f;
            }

            if (!world || bodyId.value == INVALID_BODY_ID || !havok_physics_timing::isUsableDelta(deltaTime) ||
                !outLinearVelocityHavok || !outAngularVelocityRadians ||
                !havok_runtime::getBody(world, bodyId)) {
                return false;
            }

            alignas(16) float targetPositionHavok[4]{
                targetWorld.translate.x * gameToHavokScale(),
                targetWorld.translate.y * gameToHavokScale(),
                targetWorld.translate.z * gameToHavokScale(),
                0.0f,
            };
            alignas(16) float targetRotationHavok[4]{};
            transform_math::niRowsToHavokQuaternion(targetWorld.rotate, targetRotationHavok);

            using ComputeHardKeyFrame_t = void (*)(RE::hknpWorld*, RE::hknpBodyId, float*, float*, float, float*, float*);
            static REL::Relocation<ComputeHardKeyFrame_t> compute{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            compute(world, bodyId, targetPositionHavok, targetRotationHavok, deltaTime, outLinearVelocityHavok, outAngularVelocityRadians);

            outLinearVelocityHavok[3] = 0.0f;
            outAngularVelocityRadians[3] = 0.0f;
            return havok_runtime::isFinite3(outLinearVelocityHavok) && havok_runtime::isFinite3(outAngularVelocityRadians);
        }
        grab_motion_controller::ContactSupportShape classifyContactSupportShapeFromGrabFrame(const CanonicalGrabFrame& frame)
        {
            if (frame.hasGripSupportModel) {
                switch (frame.gripSupportKind) {
                case grab_support_model_math::GripSupportKind::OpposedPinch:
                    return grab_motion_controller::ContactSupportShape::SphereLike;
                case grab_support_model_math::GripSupportKind::LongHandleAxis:
                    return grab_motion_controller::ContactSupportShape::LongHandle;
                case grab_support_model_math::GripSupportKind::PalmWrap:
                    return grab_motion_controller::ContactSupportShape::Wrap;
                case grab_support_model_math::GripSupportKind::SameSurface:
                    return frame.pivotAuthorityNormalTrusted ?
                        grab_motion_controller::ContactSupportShape::Surface :
                        grab_motion_controller::ContactSupportShape::ThinFace;
                case grab_support_model_math::GripSupportKind::SinglePoint:
                    return grab_motion_controller::ContactSupportShape::Point;
                default:
                    break;
                }
            }

            const auto fallback = grab_motion_controller::classifyContactSupportShape(
                grab_motion_controller::ContactSupportShape::Unknown,
                frame.pivotAuthorityNormalTrusted,
                frame.hasContactPatchEvidence,
                frame.contactPatchSampleCount,
                frame.multiFingerContactGroupCount,
                frame.multiFingerContactSpreadGameUnits,
                frame.longObjectLeverGameUnits,
                g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits,
                g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits);

            const std::uint32_t sampleCount = (std::min)(frame.contactPatchSampleCount, static_cast<std::uint32_t>(frame.contactPatchSamples.size()));
            if (!frame.hasContactPatchEvidence || sampleCount < 2) {
                return fallback;
            }

            std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> points{};
            std::uint32_t acceptedCount = 0;
            RE::NiPoint3 normalSum{};
            for (std::uint32_t i = 0; i < sampleCount; ++i) {
                const auto& sample = frame.contactPatchSamples[i];
                if (!sample.accepted) {
                    continue;
                }
                points[acceptedCount++] = sample.point;
                normalSum = normalSum + sample.normal;
            }
            if (acceptedCount < 2) {
                return fallback;
            }

            RE::NiPoint3 centroid{};
            for (std::uint32_t i = 0; i < acceptedCount; ++i) {
                centroid = centroid + points[i];
            }
            centroid = scalePoint(centroid, 1.0f / static_cast<float>(acceptedCount));

            RE::NiPoint3 normal = normalizeOrZero(frame.gripNormalLocal);
            if (lengthSquared(normal) <= 0.000001f) {
                normal = normalizeOrZero(normalSum);
            }
            if (lengthSquared(normal) <= 0.000001f) {
                return fallback;
            }

            RE::NiPoint3 tangent{};
            float tangentLengthSq = 0.0f;
            for (std::uint32_t i = 1; i < acceptedCount; ++i) {
                const RE::NiPoint3 candidate = points[i] - points[0];
                const RE::NiPoint3 projected = candidate - scalePoint(normal, dotProduct(candidate, normal));
                const float candidateLengthSq = lengthSquared(projected);
                if (candidateLengthSq > tangentLengthSq) {
                    tangent = projected;
                    tangentLengthSq = candidateLengthSq;
                }
            }
            tangent = normalizeOrZero(tangent);
            if (lengthSquared(tangent) <= 0.000001f) {
                return fallback;
            }
            const RE::NiPoint3 bitangent = normalizeOrZero(crossProduct(normal, tangent));
            if (lengthSquared(bitangent) <= 0.000001f) {
                return fallback;
            }

            float minT = (std::numeric_limits<float>::max)();
            float maxT = -(std::numeric_limits<float>::max)();
            float minB = (std::numeric_limits<float>::max)();
            float maxB = -(std::numeric_limits<float>::max)();
            float minDepth = (std::numeric_limits<float>::max)();
            float maxDepth = -(std::numeric_limits<float>::max)();
            for (std::uint32_t i = 0; i < acceptedCount; ++i) {
                const RE::NiPoint3 delta = points[i] - centroid;
                const float t = dotProduct(delta, tangent);
                const float b = dotProduct(delta, bitangent);
                const float depth = dotProduct(delta, normal);
                minT = (std::min)(minT, t);
                maxT = (std::max)(maxT, t);
                minB = (std::min)(minB, b);
                maxB = (std::max)(maxB, b);
                minDepth = (std::min)(minDepth, depth);
                maxDepth = (std::max)(maxDepth, depth);
            }

            const float spanT = maxT - minT;
            const float spanB = maxB - minB;
            const float majorSpan = (std::max)(spanT, spanB);
            const float minorSpan = (std::min)(spanT, spanB);
            const float depthSpan = maxDepth - minDepth;
            const float smallReference = (std::max)(1.0f, g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits);
            const float longReference = (std::max)(smallReference, g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits);
            const bool smallObject = frame.longObjectLeverGameUnits > 0.0f && frame.longObjectLeverGameUnits <= smallReference;
            const bool longObject = frame.longObjectLeverGameUnits >= longReference * 1.35f;
            const bool thinLine = majorSpan > 0.75f && minorSpan <= (std::max)(0.75f, majorSpan * 0.20f);
            const bool flatSurface = majorSpan > 0.75f && minorSpan > 0.75f && depthSpan <= (std::max)(0.75f, majorSpan * 0.20f);

            if (smallObject && acceptedCount >= 3) {
                return grab_motion_controller::ContactSupportShape::SphereLike;
            }
            if (longObject && (thinLine || acceptedCount <= 2)) {
                return grab_motion_controller::ContactSupportShape::LongHandle;
            }
            if (thinLine) {
                return grab_motion_controller::ContactSupportShape::ThinEdge;
            }
            if (flatSurface) {
                return frame.pivotAuthorityNormalTrusted ?
                    grab_motion_controller::ContactSupportShape::Surface :
                    grab_motion_controller::ContactSupportShape::ThinFace;
            }
            return fallback;
        }

        grab_motion_controller::AngularAuthorityInput makeAngularAuthorityInput(const CanonicalGrabFrame& frame)
        {
            return grab_motion_controller::AngularAuthorityInput{
                .enabled = g_rockConfig.rockGrabPivotQualityAngularScalingEnabled,
                .positionOnlyPivot = frame.pivotAuthorityPositionOnly,
                .normalTrusted = frame.pivotAuthorityNormalTrusted,
                .contactPatchEvidence = frame.hasContactPatchEvidence,
                .contactPatchSampleCount = frame.contactPatchSampleCount,
                .multiFingerContactGroupCount = frame.multiFingerContactGroupCount,
                .multiFingerContactSpreadGameUnits = frame.multiFingerContactSpreadGameUnits,
                .longObjectLeverGameUnits = frame.longObjectLeverGameUnits,
                .smallObjectReferenceLeverGameUnits = g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits,
                .positionOnlyAngularScale = g_rockConfig.rockGrabPositionOnlyAngularScale,
                .smallObjectAngularScale = g_rockConfig.rockGrabSmallObjectAngularScale,
                .lowContactSupportAngularScale = g_rockConfig.rockGrabLowContactSupportAngularScale,
                .minAngularAuthorityScale = g_rockConfig.rockGrabMinAngularAuthorityScale,
                .weakPivotTwistScale = g_rockConfig.rockGrabWeakPivotTwistScale,
                .contactSupportShape = classifyContactSupportShapeFromGrabFrame(frame),
                .longObjectReferenceLeverGameUnits = g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
            };
        }

        grab_motion_controller::HeldAuthorityState evaluateRuntimeHeldAuthority(
            const CanonicalGrabFrame& frame,
            bool heldBodyContactSoftening)
        {
            return grab_motion_controller::evaluateHeldAuthority(grab_motion_controller::HeldAuthorityInput{
                .angular = makeAngularAuthorityInput(frame),
                .heldBodyColliding = heldBodyContactSoftening,
            });
        }
        constexpr const char* kGrabObjectRotationReferenceName = "generatedProxyAuthorityLocal";
        constexpr float kGrabFrameMismatchRawProxyRotationWarnDegrees = 20.0f;
        constexpr float kGrabFrameMismatchProxyRotationWarnDegrees = 5.0f;
        constexpr float kGrabFrameMismatchObjectRotationWarnDegrees = 25.0f;
        constexpr float kGrabFrameMismatchGripErrorWarnGameUnits = 5.0f;
    }


    void Hand::flushPendingCustomGrabAuthority(
        RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing,
        const grab_authority_source_clock::ControllerRootFrameSample& consumptionControllerRoot,
        float controllerHavokToGameScale)
    {
        // ANCHOR_CLOCK probe: room/heldNode state as seen from the physics
        // clock (first substep only). Read-only diagnostic reads of
        // game-thread node state; ordering vs producer/preFrik stages comes
        // from log emission order.
        if (timing.substepIndex == 0 && isHoldingAtomic()) {
            const auto anchorProbeRoom = sampleAnchorClockRoom();
            auto* anchorProbeHeldNode = _grabFrame.heldNode;
            const RE::NiPoint3 anchorProbeHeldPos =
                anchorProbeHeldNode ? anchorProbeHeldNode->world.translate : RE::NiPoint3{};
            const float anchorProbeHeldVsLastWrite =
                (anchorProbeHeldNode && _hasGrabProbeLastAnchorWrite) ?
                pointDistanceGameUnits(anchorProbeHeldPos, _grabProbeLastAnchorWrite.translate) :
                -1.0f;
            ROCK_LOG_INFO(Hand,
                "{} ANCHOR_CLOCK stage=physics phase={} room=({:.2f},{:.2f},{:.2f}) roomYaw={:.3f} heldNode=({:.2f},{:.2f},{:.2f}) heldVsLastWrite={:.3f}gu",
                handName(),
                static_cast<int>(timing.phase),
                anchorProbeRoom.position.x,
                anchorProbeRoom.position.y,
                anchorProbeRoom.position.z,
                anchorProbeRoom.yawDegrees,
                anchorProbeHeldPos.x,
                anchorProbeHeldPos.y,
                anchorProbeHeldPos.z,
                anchorProbeHeldVsLastWrite);

            rock::debug::RockGrabClockStagePhysicsV1 grabClockPhysicsSample{};
            assignGrabClockFeedVec(grabClockPhysicsSample.roomPos, anchorProbeRoom.position);
            grabClockPhysicsSample.roomYawDegrees = anchorProbeRoom.yawDegrees;
            grabClockPhysicsSample.roomValid = anchorProbeRoom.valid ? 1u : 0u;
            assignGrabClockFeedVec(grabClockPhysicsSample.heldNodePos, anchorProbeHeldPos);
            grabClockPhysicsSample.heldNodeValid = anchorProbeHeldNode ? 1u : 0u;
            grabClockPhysicsSample.heldVsLastWriteGu = anchorProbeHeldVsLastWrite;
            {
                held_body_render_pose::MasqueradeStatus masqueradeStatus{};
                held_body_render_pose::copyStatus(masqueradeStatus);
                grabClockPhysicsSample.masqActiveNow = masqueradeStatus.activeNow;
                grabClockPhysicsSample.masqApplied = masqueradeStatus.appliedSteps;
                grabClockPhysicsSample.masqRestored = masqueradeStatus.restoredSteps;
                grabClockPhysicsSample.masqMismatch = masqueradeStatus.restoreMismatch;
                grabClockPhysicsSample.masqSkipped = masqueradeStatus.skippedContended +
                                                     masqueradeStatus.skippedInvalidBody +
                                                     masqueradeStatus.skippedImplausible;
            }
            rock::debug::publishGrabClockPhysicsStage(_isLeft, grabClockPhysicsSample);
        }

        GrabAuthorityProxyPendingTarget pending{};
        RE::NiTransform queuedRawHandWorld{};
        RE::NiTransform previousProxyWorld{};
        RE::hknpBodyId proxyBodyId{ INVALID_BODY_ID };
        bool proxyDriveOk = false;
        bool livePalmReferenceOk = false;
        bool targetUpdateOk = false;
        bool angularDriveOk = false;
        bool shouldLog = false;
        LivePalmAnchorReference livePalmReference{};
        GeneratedKeyframedBodyDriveResult proxyDriveResult{};
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiPoint3 desiredTargetPointWorld{};
        RE::NiPoint3 activePivotBBodyLocalGame{};
        float proxyLinearVelocityHavokMagnitude = 0.0f;
        float proxyAngularVelocityRadiansPerSecond = 0.0f;
        bool proxyVelocityTelemetryOk = false;
        RE::NiTransform proxyReadbackBetween{};
        body_frame::BodyFrameSource proxyReadbackSourceBetween = body_frame::BodyFrameSource::Fallback;
        std::uint32_t proxyReadbackMotionIndexBetween = body_frame::kFreeMotionIndex;
        bool proxyReadbackBetweenOk = false;
        float proxyReadbackBetweenPositionErrorGameUnits = -1.0f;
        float proxyReadbackBetweenRotationErrorDegrees = -1.0f;
        float angularMotorBudget = 0.0f;
        std::uint64_t queuedSequence = 0;
        std::uint64_t flushSequence = 0;
        grab_authority_source_clock::ResampleAction resampleAction = grab_authority_source_clock::ResampleAction::Hold;
        grab_authority_source_clock::ConsumptionFrameRebaseResult consumptionFrameRebase{};
        std::uint32_t resampleRebaseCount = 0;
        GrabAngularAuthority angularAuthority = GrabAngularAuthority::HknpRagdollMotorAtom;
        {
            // Guard the proxy lease and its queued target as one snapshot.
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            const bool hasAuthority = _grabAuthorityProxy.isValid() &&
                                      _grabAuthorityProxyHknpWorld == world &&
                                      _grabAuthorityPendingTarget.valid;
            if (!hasAuthority) {
                return;
            }
            performance_profiler::addEventCount(performance_profiler::Scope::GrabAuthorityFlush);
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAuthorityFlush);

            pending = _grabAuthorityPendingTarget;
            queuedRawHandWorld = pending.rawHandWorld;
            // Accept the game-frame sample exactly once; the queued-sequence
            // identity keeps multi-substep re-flushes of the same pending
            // target from advancing the source segment. The drive target is
            // then phase-locked to the game clock per substep below. See
            // GrabAuthoritySourceClockResampler.h for the contract.
            _grabAuthoritySourceClock.advanceSource(
                pending.proxyWorld.translate,
                pending.proxyWorld.rotate,
                pending.deltaTime,
                _grabAuthorityProxyQueuedSequence);
            livePalmReferenceOk = tryResolveLivePalmAnchorReference(world, livePalmReference);
            if (!livePalmReferenceOk) {
                ++_grabAuthorityProxyFailedFlushes;
                _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
            }
            /*
             * The game-frame held update owns pending.proxyWorld. Rebinding it
             * here to the live palm body reintroduced one-Havok-tick latency
             * during stick locomotion, even after collider targets were bridged.
             * Keep live palm as a fail-closed health/telemetry read only.
             */
            previousProxyWorld = _hasLastAppliedGrabAuthorityProxyWorld ? _lastAppliedGrabAuthorityProxyWorld : pending.proxyWorld;
            proxyBodyId = _grabAuthorityProxy.getBodyId();
            angularAuthority = _activeConstraint.angularAuthority;

            const float driveDelta = havok_physics_timing::driveDeltaSeconds(timing);
            // Game-clock phase lock: before the measured root rebase below, the
            // frame's last substep lands EXACTLY on the queued game-frame sample
            // and intra-frame substeps stay on its sampled path -- the
            // 2026-07-13 OVERLAY_POINT probe proved physics-clock playback put
            // v x (clock mismatch) between the held object and everything else
            // the eye tracks. Intra-frame substeps interpolate along the
            // sample segment. The consumption-frame rebase removes only the
            // common character-root displacement that occurred after sampling.
            // Local copy only: every downstream use in this
            // flush -- keyframe drive, constraint target, motors, readback
            // diagnostics, last-applied tracking -- sees the locked target
            // consistently, while the stored pending target stays the raw
            // sample. Rotation deliberately stays on the sampled path.
            pending.proxyWorld.translate = _grabAuthoritySourceClock.evaluate(timing.substepIndex, timing.substepCount, resampleAction);
            resampleRebaseCount = _grabAuthoritySourceClock.rebaseCount;
            consumptionFrameRebase = _grabAuthorityConsumptionFrameRebase.evaluate(
                pending.sourceControllerRoot,
                consumptionControllerRoot,
                controllerHavokToGameScale,
                _grabAuthorityProxyQueuedSequence,
                _grabAuthoritySourceClock.playedFraction);
            if (consumptionFrameRebase.valid) {
                // The proxy target may be an intra-frame interpolation between
                // two source samples, so it receives the matching interpolated
                // root shift. rawHandWorld is the newest endpoint and receives
                // that endpoint's exact measured shift for truthful telemetry.
                pending.proxyWorld.translate.x += consumptionFrameRebase.shiftGame.x;
                pending.proxyWorld.translate.y += consumptionFrameRebase.shiftGame.y;
                pending.proxyWorld.translate.z += consumptionFrameRebase.shiftGame.z;
                pending.rawHandWorld.translate.x += consumptionFrameRebase.currentEndpointShiftGame.x;
                pending.rawHandWorld.translate.y += consumptionFrameRebase.currentEndpointShiftGame.y;
                pending.rawHandWorld.translate.z += consumptionFrameRebase.currentEndpointShiftGame.z;
            }
            float linearVelocityHavok[4]{};
            float angularVelocityHavok[4]{};
            float nativeLinearVelocityIgnored[4]{};
            grab_authority_proxy::computeLinearVelocityHavok(previousProxyWorld, pending.proxyWorld, driveDelta, linearVelocityHavok);
            if (livePalmReference.hasMotionVelocity) {
                angularVelocityHavok[0] = livePalmReference.angularVelocityRadiansPerSecond.x;
                angularVelocityHavok[1] = livePalmReference.angularVelocityRadiansPerSecond.y;
                angularVelocityHavok[2] = livePalmReference.angularVelocityRadiansPerSecond.z;
                proxyVelocityTelemetryOk = true;
            } else if (livePalmReferenceOk) {
                proxyVelocityTelemetryOk = computeHardKeyframeVelocityForTarget(
                    world,
                    proxyBodyId,
                    pending.proxyWorld,
                    driveDelta,
                    nativeLinearVelocityIgnored,
                    angularVelocityHavok);
            }
            proxyLinearVelocityHavokMagnitude = std::sqrt(
                linearVelocityHavok[0] * linearVelocityHavok[0] + linearVelocityHavok[1] * linearVelocityHavok[1] +
                linearVelocityHavok[2] * linearVelocityHavok[2]);
            proxyAngularVelocityRadiansPerSecond = std::sqrt(
                angularVelocityHavok[0] * angularVelocityHavok[0] + angularVelocityHavok[1] * angularVelocityHavok[1] +
                angularVelocityHavok[2] * angularVelocityHavok[2]);

            if (livePalmReferenceOk) {
                queueGeneratedKeyframedBodyTarget(_grabAuthorityProxyDriveState, pending.proxyWorld, driveDelta, 1000.0f);
                proxyDriveResult = driveGeneratedKeyframedBody(
                    world,
                    _grabAuthorityProxy,
                    _grabAuthorityProxyDriveState,
                    timing,
                    "grab-authority-proxy",
                    0);
                proxyDriveOk = proxyDriveResult.driven;
            }
            if (proxyDriveOk) {
                proxyReadbackBetweenOk =
                    tryResolveLiveBodyWorldTransform(world, proxyBodyId, proxyReadbackBetween, &proxyReadbackSourceBetween, &proxyReadbackMotionIndexBetween);
                if (proxyReadbackBetweenOk) {
                    proxyReadbackBetweenPositionErrorGameUnits =
                        pointDistanceGameUnits(proxyReadbackBetween.translate, pending.proxyWorld.translate);
                    proxyReadbackBetweenRotationErrorDegrees =
                        rotationDeltaDegrees(proxyReadbackBetween.rotate, pending.proxyWorld.rotate);
                }
            }
            if (!proxyDriveOk) {
                ++_grabAuthorityProxyFailedFlushes;
                _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
            } else {
                // This call runs on the physics thread before the solver step.
                targetUpdateOk = updateProxyConstraintGrabDriveTarget(
                    world,
                    pending.proxyWorld,
                    desiredObjectWorld,
                    desiredBodyWorld,
                    desiredTargetPointWorld,
                    activePivotBBodyLocalGame);
                if (!targetUpdateOk) {
                    ++_grabAuthorityProxyFailedFlushes;
                    _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
                } else {
                    const auto pendingHeldAuthority = evaluateRuntimeHeldAuthority(
                        _grabFrame,
                        pending.heldBodyColliding);
                    updateConstraintGrabDriveMotors(
                        world,
                        driveDelta,
                        pending.forceFadeInTime,
                        pending.tauMin,
                        pending.authorityForceScale,
                        pending.heldBodyColliding,
                        pendingHeldAuthority);
                    angularDriveOk =
                        _activeConstraint.isValid() &&
                        _activeConstraint.usesRagdollAngularMotorAtom() &&
                        _activeConstraint.linearMotor &&
                        _activeConstraint.angularMotor;
                    if (_activeConstraint.angularMotor) {
                        angularMotorBudget = (std::max)(
                            std::fabs(_activeConstraint.angularMotor->minForce),
                            std::fabs(_activeConstraint.angularMotor->maxForce));
                    }
                    _ragdollAngularProbePreSolve = {};
                    if (angularDriveOk && _activeConstraint.constraintData) {
                        RE::NiTransform bodyWorldBeforeSolve{};
                        if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, bodyWorldBeforeSolve)) {
                            const auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
                            const auto* targetBRca = reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
                            const auto* transformARotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_A_COL0);
                            const auto* transformBRotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_COL0);
                            const auto* transformBTranslation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
                            const RE::NiTransform desiredBodyTransformHandSpace =
                                grab_frame_math::objectInGeneratedProxyLocalSpace(pending.proxyWorld, desiredBodyWorld);
                            const RE::NiTransform desiredBodyToHandSpace = invertTransform(desiredBodyTransformHandSpace);
                            const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(targetBRca);
                            const RE::NiMatrix3 targetAsHkColumns = matrixFromHkColumns(targetBRca);
                            const RE::NiMatrix3 transformAAsHkColumns = matrixFromHkColumns(transformARotation);
                            const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformBRotation);
                            std::array<float, 12> transformARaw{};
                            std::array<float, 12> transformBRaw{};
                            std::array<float, 12> targetBRcaRaw{};
                            std::memcpy(transformARaw.data(), transformARotation, sizeof(float) * transformARaw.size());
                            std::memcpy(transformBRaw.data(), transformBRotation, sizeof(float) * transformBRaw.size());
                            std::memcpy(targetBRcaRaw.data(), targetBRca, sizeof(float) * targetBRcaRaw.size());
                            float transformARawMaxDelta = 0.0f;
                            float transformBRawMaxDelta = 0.0f;
                            float targetBRcaRawMaxDelta = 0.0f;
                            if (_hasLastGrabProbeAtomBytes && _lastGrabProbeAtomTraceId == _grabFrame.traceId) {
                                transformARawMaxDelta = maxAbsDelta(transformARaw, _lastGrabProbeTransformARaw);
                                transformBRawMaxDelta = maxAbsDelta(transformBRaw, _lastGrabProbeTransformBRaw);
                                targetBRcaRawMaxDelta = maxAbsDelta(targetBRcaRaw, _lastGrabProbeTargetBRcaRaw);
                            }
                            _lastGrabProbeTransformARaw = transformARaw;
                            _lastGrabProbeTransformBRaw = transformBRaw;
                            _lastGrabProbeTargetBRcaRaw = targetBRcaRaw;
                            _lastGrabProbeAtomTraceId = _grabFrame.traceId;
                            _hasLastGrabProbeAtomBytes = true;

                            RE::NiTransform bodyAWorldBeforeSolve{};
                            bool bodyAWorldBeforeSolveOk = false;
                            if (proxyReadbackBetweenOk) {
                                bodyAWorldBeforeSolve = proxyReadbackBetween;
                                bodyAWorldBeforeSolveOk = true;
                            } else {
                                bodyAWorldBeforeSolveOk = tryResolveLiveBodyWorldTransform(world, proxyBodyId, bodyAWorldBeforeSolve);
                            }
                            const RE::NiPoint3 targetAnchorAWorld =
                                generatedProxyLocalPointToWorld(pending.proxyWorld, _grabFrame.pivotAHandBodyLocalGame);
                            RE::NiTransform targetRelationAWorld = makeGeneratedProxyAuthorityRelationFrame(pending.proxyWorld);
                            targetRelationAWorld.translate = targetAnchorAWorld;
                            RE::NiTransform targetConstraintAWorld =
                                transform_math::composeTransforms(pending.proxyWorld, rotationOnlyTransform(transformAAsHkColumns));
                            targetConstraintAWorld.translate = targetAnchorAWorld;
                            RE::NiTransform liveRelationAWorld = targetRelationAWorld;
                            RE::NiTransform liveConstraintAWorld = targetConstraintAWorld;
                            float targetProxyToLiveProxyDeltaGameUnits = -1.0f;
                            float targetProxyToLiveProxyDeltaDegrees = -1.0f;
                            float targetRelationAToLiveRelationADegrees = -1.0f;
                            float targetConstraintAToLiveConstraintADegrees = -1.0f;
                            float targetRelationAToTargetConstraintADegrees =
                                rotationDeltaDegrees(targetRelationAWorld.rotate, targetConstraintAWorld.rotate);
                            float liveRelationAToLiveConstraintADegrees = targetRelationAToTargetConstraintADegrees;
                            if (bodyAWorldBeforeSolveOk) {
                                const RE::NiPoint3 liveAnchorAWorld =
                                    generatedProxyLocalPointToWorld(bodyAWorldBeforeSolve, _grabFrame.pivotAHandBodyLocalGame);
                                liveRelationAWorld = makeGeneratedProxyAuthorityRelationFrame(bodyAWorldBeforeSolve);
                                liveRelationAWorld.translate = liveAnchorAWorld;
                                liveConstraintAWorld =
                                    transform_math::composeTransforms(bodyAWorldBeforeSolve, rotationOnlyTransform(transformAAsHkColumns));
                                liveConstraintAWorld.translate = liveAnchorAWorld;
                                targetProxyToLiveProxyDeltaGameUnits =
                                    pointDistanceGameUnits(pending.proxyWorld.translate, bodyAWorldBeforeSolve.translate);
                                targetProxyToLiveProxyDeltaDegrees =
                                    rotationDeltaDegrees(pending.proxyWorld.rotate, bodyAWorldBeforeSolve.rotate);
                                targetRelationAToLiveRelationADegrees =
                                    rotationDeltaDegrees(targetRelationAWorld.rotate, liveRelationAWorld.rotate);
                                targetConstraintAToLiveConstraintADegrees =
                                    rotationDeltaDegrees(targetConstraintAWorld.rotate, liveConstraintAWorld.rotate);
                                liveRelationAToLiveConstraintADegrees =
                                    rotationDeltaDegrees(liveRelationAWorld.rotate, liveConstraintAWorld.rotate);
                            }

                            float ragdollBRcaRowsErrorDegrees = -1.0f;
                            float ragdollBRcaColumnsErrorDegrees = -1.0f;
                            float ragdollARcbRowsInverseErrorDegrees = -1.0f;
                            float ragdollARcbColumnsInverseErrorDegrees = -1.0f;
                            const float targetToHiggsRelationDegrees =
                                rotationDeltaDegrees(targetAsHkRows, desiredBodyToHandSpace.rotate);
                            const float transformBFrozenDeltaDegrees =
                                rotationDeltaDegrees(transformBAsHkColumns, desiredBodyToHandSpace.rotate);
                            const RE::NiPoint3 relationPivotB =
                                grab_constraint_math::computeHiggsTransformBTranslationGame(
                                    desiredBodyTransformHandSpace,
                                    _grabFrame.pivotAHandBodyLocalGame);
                            const RE::NiPoint3 activeTransformBTranslationGame{
                                transformBTranslation[0] * havokToGameScale(),
                                transformBTranslation[1] * havokToGameScale(),
                                transformBTranslation[2] * havokToGameScale(),
                            };
                            const float pivotBRelationDeltaGameUnits =
                                pointDistanceGameUnits(activeTransformBTranslationGame, relationPivotB);
                            const RE::NiTransform proxyInBodyBeforeTargetWrite =
                                grab_constraint_math::proxyInBodyFromBodyInProxy(desiredBodyTransformHandSpace);
                            const RE::NiTransform relationInverseBodyWorld = reconstructBodyWorldFromProxyInBody(
                                pending.proxyWorld,
                                proxyInBodyBeforeTargetWrite.rotate,
                                relationPivotB,
                                _grabFrame.pivotAHandBodyLocalGame);
                            const RE::NiTransform atomRowsBodyWorld = reconstructBodyWorldFromProxyInBody(
                                pending.proxyWorld,
                                targetAsHkRows,
                                activeTransformBTranslationGame,
                                _grabFrame.pivotAHandBodyLocalGame);
                            const RE::NiTransform solverEffectiveBodyWorld = reconstructSolverEffectiveBodyWorld(
                                pending.proxyWorld,
                                transformAAsHkColumns,
                                transformBAsHkColumns,
                                targetAsHkRows,
                                activeTransformBTranslationGame,
                                targetAnchorAWorld,
                                desiredBodyWorld.scale);
                            const RE::NiTransform solverEffectiveLiveAWorld = reconstructSolverEffectiveBodyWorld(
                                bodyAWorldBeforeSolveOk ? bodyAWorldBeforeSolve : pending.proxyWorld,
                                transformAAsHkColumns,
                                transformBAsHkColumns,
                                targetAsHkRows,
                                activeTransformBTranslationGame,
                                liveConstraintAWorld.translate,
                                desiredBodyWorld.scale);
                            const float relationInverseBodyDeltaGameUnits =
                                translationDeltaGameUnits(relationInverseBodyWorld, desiredBodyWorld);
                            const float relationInverseBodyDeltaDegrees =
                                rotationDeltaDegrees(relationInverseBodyWorld.rotate, desiredBodyWorld.rotate);
                            const float atomRowsBodyDeltaGameUnits =
                                translationDeltaGameUnits(atomRowsBodyWorld, desiredBodyWorld);
                            const float atomRowsBodyDeltaDegrees =
                                rotationDeltaDegrees(atomRowsBodyWorld.rotate, desiredBodyWorld.rotate);
                            const float atomRowsToRelationInverseDegrees =
                                rotationDeltaDegrees(atomRowsBodyWorld.rotate, relationInverseBodyWorld.rotate);
                            const float targetRowsToProxyInBodyDegrees =
                                rotationDeltaDegrees(targetAsHkRows, proxyInBodyBeforeTargetWrite.rotate);
                            const float solverEffectiveBodyDeltaGameUnits =
                                translationDeltaGameUnits(solverEffectiveBodyWorld, desiredBodyWorld);
                            const float solverEffectiveBodyDeltaDegrees =
                                rotationDeltaDegrees(solverEffectiveBodyWorld.rotate, desiredBodyWorld.rotate);
                            const float solverEffectiveToAtomDegrees =
                                rotationDeltaDegrees(solverEffectiveBodyWorld.rotate, atomRowsBodyWorld.rotate);
                            const float solverEffectiveLiveABodyDeltaDegrees =
                                rotationDeltaDegrees(solverEffectiveLiveAWorld.rotate, desiredBodyWorld.rotate);
                            const float solverEffectiveTargetALiveADeltaDegrees =
                                rotationDeltaDegrees(solverEffectiveBodyWorld.rotate, solverEffectiveLiveAWorld.rotate);
                            float transformAPivotRoundTripDeltaGameUnits = -1.0f;
                            RE::NiPoint3 activePivotAWorld{};
                            if (resolveActiveGrabAuthorityPivotAWorld(pending.proxyWorld, activePivotAWorld)) {
                                const RE::NiPoint3 roundTripPivotAProxyLocalGame =
                                    grab_constraint_math::computeGeneratedProxyConstraintPivotLocalGame(pending.proxyWorld, activePivotAWorld);
                                if (std::isfinite(roundTripPivotAProxyLocalGame.x) &&
                                    std::isfinite(roundTripPivotAProxyLocalGame.y) &&
                                    std::isfinite(roundTripPivotAProxyLocalGame.z)) {
                                    transformAPivotRoundTripDeltaGameUnits =
                                        pointDistanceGameUnits(_grabAuthorityPivotAProxyLocalGame, roundTripPivotAProxyLocalGame);
                                }
                            }
                            if (bodyAWorldBeforeSolveOk) {
                                const RE::NiMatrix3 constraintAWorldRotation =
                                    multiplyTransforms(bodyAWorldBeforeSolve, rotationOnlyTransform(transformAAsHkColumns)).rotate;
                                const RE::NiMatrix3 constraintBWorldRotation =
                                    multiplyTransforms(bodyWorldBeforeSolve, rotationOnlyTransform(transformBAsHkColumns)).rotate;
                                const RE::NiMatrix3 currentBRca = frameToFrameRotation(constraintBWorldRotation, constraintAWorldRotation);
                                const RE::NiMatrix3 currentARcb = frameToFrameRotation(constraintAWorldRotation, constraintBWorldRotation);
                                const RE::NiMatrix3 targetRowsInverse = transform_math::transposeRotation(targetAsHkRows);
                                const RE::NiMatrix3 targetColumnsInverse = transform_math::transposeRotation(targetAsHkColumns);
                                ragdollBRcaRowsErrorDegrees = rotationDeltaDegrees(currentBRca, targetAsHkRows);
                                ragdollBRcaColumnsErrorDegrees = rotationDeltaDegrees(currentBRca, targetAsHkColumns);
                                ragdollARcbRowsInverseErrorDegrees = rotationDeltaDegrees(currentARcb, targetRowsInverse);
                                ragdollARcbColumnsInverseErrorDegrees = rotationDeltaDegrees(currentARcb, targetColumnsInverse);
                            }

                            const RE::NiPoint3 liveGripBeforeSolve =
                                transform_math::localPointToWorld(bodyWorldBeforeSolve, activePivotBBodyLocalGame);
                            /*
                             * The linear motor can rotate the object only through
                             * this off-center pivot witness. A small cross product
                             * means the angular atom must do almost all rotation.
                             */
                            const RE::NiPoint3 linearCorrectionWorld = desiredTargetPointWorld - liveGripBeforeSolve;
                            const RE::NiPoint3 linearLeverWorld = liveGripBeforeSolve - bodyWorldBeforeSolve.translate;
                            const RE::NiPoint3 linearTorqueWitnessWorld = crossProduct(linearLeverWorld, linearCorrectionWorld);
                            const RE::NiPoint3 linearTorqueAxisWorld = normalizeOrZero(linearTorqueWitnessWorld);
                            const float linearTorqueWitnessGameUnitsSquared = vectorMagnitude(linearTorqueWitnessWorld);
                            RE::NiPoint3 angularVelocityBeforeSolve{};
                            if (auto* motion = havok_runtime::getBodyMotion(world, _savedObjectState.bodyId)) {
                                angularVelocityBeforeSolve = RE::NiPoint3{
                                    motion->angularVelocity.x,
                                    motion->angularVelocity.y,
                                    motion->angularVelocity.z,
                                };
                            }

                            const float linearMotorBudget = _activeConstraint.linearMotor ?
                                (std::max)(
                                    std::fabs(_activeConstraint.linearMotor->minForce),
                                    std::fabs(_activeConstraint.linearMotor->maxForce)) :
                                0.0f;
                            const RE::NiPoint3 requiredAxisWorld = rotationCorrectionAxisWorld(bodyWorldBeforeSolve.rotate, desiredBodyWorld.rotate);
                            const RE::NiPoint3 requiredAxisProxyLocal = bodyAWorldBeforeSolveOk ?
                                rotationAxisProxyLocal(bodyAWorldBeforeSolve.rotate, requiredAxisWorld) :
                                RE::NiPoint3{};
                            const RE::NiPoint3 linearTorqueAxisProxyLocal =
                                bodyAWorldBeforeSolveOk ? rotationAxisProxyLocal(bodyAWorldBeforeSolve.rotate, linearTorqueAxisWorld) : RE::NiPoint3{};
                            const float linearTorqueAxisDotRequired =
                                (lengthSquared(linearTorqueAxisWorld) > 0.000001f && lengthSquared(requiredAxisWorld) > 0.000001f) ?
                                    std::clamp(dotProduct(linearTorqueAxisWorld, requiredAxisWorld), -1.0f, 1.0f) :
                                    0.0f;
                            _ragdollAngularProbePreSolve = RagdollAngularProbePreSolve{
                                .objectBodyId = _savedObjectState.bodyId,
                                .desiredBodyWorld = desiredBodyWorld,
                                .bodyAWorldBefore = bodyAWorldBeforeSolve,
                                .bodyWorldBefore = bodyWorldBeforeSolve,
                                .targetRelationAWorld = targetRelationAWorld,
                                .liveRelationAWorld = liveRelationAWorld,
                                .targetConstraintAWorld = targetConstraintAWorld,
                                .liveConstraintAWorld = liveConstraintAWorld,
                                .relationInverseBodyWorld = relationInverseBodyWorld,
                                .atomRowsBodyWorld = atomRowsBodyWorld,
                                .solverEffectiveBodyWorld = solverEffectiveBodyWorld,
                                .solverEffectiveLiveAWorld = solverEffectiveLiveAWorld,
                                .transformARotation = transformAAsHkColumns,
                                .transformBRotation = transformBAsHkColumns,
                                .targetBRcaRaw = targetBRcaRaw,
                                .requiredAxisWorld = requiredAxisWorld,
                                .requiredAxisProxyLocal = requiredAxisProxyLocal,
                                .linearCorrectionWorld = linearCorrectionWorld,
                                .linearLeverWorld = linearLeverWorld,
                                .linearTorqueWitnessWorld = linearTorqueWitnessWorld,
                                .linearTorqueAxisProxyLocal = linearTorqueAxisProxyLocal,
                                .angularVelocityBeforeRadians = angularVelocityBeforeSolve,
                                .beforeErrorDegrees = rotationDeltaDegrees(bodyWorldBeforeSolve.rotate, desiredBodyWorld.rotate),
                                .beforeGripErrorGameUnits = pointDistanceGameUnits(liveGripBeforeSolve, desiredTargetPointWorld),
                                .pivotLeverGameUnits = pointDistanceGameUnits(bodyWorldBeforeSolve.translate, liveGripBeforeSolve),
                                .linearTorqueWitnessGameUnitsSquared = linearTorqueWitnessGameUnitsSquared,
                                .linearTorqueAxisDotRequired = linearTorqueAxisDotRequired,
                                .angularMotorTau = _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : 0.0f,
                                .angularMotorDamping = _activeConstraint.angularMotor ? _activeConstraint.angularMotor->damping : 0.0f,
                                .angularMotorMaxForce = angularMotorBudget,
                                .linearMotorMaxForce = linearMotorBudget,
                                .targetToHiggsRelationDegrees = targetToHiggsRelationDegrees,
                                .transformBFrozenDeltaDegrees = transformBFrozenDeltaDegrees,
                                .pivotBRelationDeltaGameUnits = pivotBRelationDeltaGameUnits,
                                .transformAPivotRoundTripDeltaGameUnits = transformAPivotRoundTripDeltaGameUnits,
                                .targetProxyToLiveProxyDeltaGameUnits = targetProxyToLiveProxyDeltaGameUnits,
                                .targetProxyToLiveProxyDeltaDegrees = targetProxyToLiveProxyDeltaDegrees,
                                .targetRelationAToLiveRelationADegrees = targetRelationAToLiveRelationADegrees,
                                .targetConstraintAToLiveConstraintADegrees = targetConstraintAToLiveConstraintADegrees,
                                .targetRelationAToTargetConstraintADegrees = targetRelationAToTargetConstraintADegrees,
                                .liveRelationAToLiveConstraintADegrees = liveRelationAToLiveConstraintADegrees,
                                .transformARawMaxDelta = transformARawMaxDelta,
                                .transformBRawMaxDelta = transformBRawMaxDelta,
                                .targetBRcaRawMaxDelta = targetBRcaRawMaxDelta,
                                .relationInverseBodyDeltaGameUnits = relationInverseBodyDeltaGameUnits,
                                .relationInverseBodyDeltaDegrees = relationInverseBodyDeltaDegrees,
                                .atomRowsBodyDeltaGameUnits = atomRowsBodyDeltaGameUnits,
                                .atomRowsBodyDeltaDegrees = atomRowsBodyDeltaDegrees,
                                .atomRowsToRelationInverseDegrees = atomRowsToRelationInverseDegrees,
                                .targetRowsToProxyInBodyDegrees = targetRowsToProxyInBodyDegrees,
                                .solverEffectiveBodyDeltaGameUnits = solverEffectiveBodyDeltaGameUnits,
                                .solverEffectiveBodyDeltaDegrees = solverEffectiveBodyDeltaDegrees,
                                .solverEffectiveToAtomDegrees = solverEffectiveToAtomDegrees,
                                .solverEffectiveLiveABodyDeltaDegrees = solverEffectiveLiveABodyDeltaDegrees,
                                .solverEffectiveTargetALiveADeltaDegrees = solverEffectiveTargetALiveADeltaDegrees,
                                .ragdollBRcaRowsErrorDegrees = ragdollBRcaRowsErrorDegrees,
                                .ragdollBRcaColumnsErrorDegrees = ragdollBRcaColumnsErrorDegrees,
                                .ragdollARcbRowsInverseErrorDegrees = ragdollARcbRowsInverseErrorDegrees,
                                .ragdollARcbColumnsInverseErrorDegrees = ragdollARcbColumnsInverseErrorDegrees,
                                .traceId = _grabFrame.traceId,
                                .targetWriteSequence = _grabFrame.traceTargetWriteSequence,
                                .flushSequence = _grabAuthorityProxyFlushSequence + 1,
                                .bodyAWorldValid = bodyAWorldBeforeSolveOk,
                                .ragdollMotorEnabled = *(constraintData + ATOM_RAGDOLL_MOT + 0x02) != 0,
                                .valid = true,
                            };
                            if (grabTimelineTraceEnabled() && shouldLogGrabTimelineSequence(_grabFrame.traceTargetWriteSequence)) {
                                const float lastGrabPhysicsHz = _lastGrabPhysicsHz.load(std::memory_order_relaxed);
                                const float lastGrabPhysicsRateForceScale = _lastGrabPhysicsRateForceScale.load(std::memory_order_relaxed);
                                ROCK_LOG_INFO(Hand,
                                    "{} GRAB_TRACE stage=pre_solve trace={} writeSeq={} flushNext={} queued={} substep={}/{} constraint={} proxyBody={} objBody={} bodyA={} beforeErr={:.2f}deg gripBefore={:.2f}gu pivotLever={:.2f}gu linTorque={:.3f}gu2 linTorqueDotReq={:.2f} reqAxis=({:.3f},{:.3f},{:.3f}) reqAxisProxy=({:.3f},{:.3f},{:.3f}) forceA={:.0f} forceL={:.0f} physHz={:.1f} forceScale={:.3f} tau={:.3f} damp={:.2f} targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg pivotBRelationDelta={:.3f}gu bRcaRowsErr={:.2f}deg bRcaColsErr={:.2f}deg pivotARoundTrip={:.3f}gu",
                                    handName(),
                                    _grabFrame.traceId,
                                    _grabFrame.traceTargetWriteSequence,
                                    _grabAuthorityProxyFlushSequence + 1,
                                    _grabAuthorityProxyQueuedSequence,
                                    timing.substepIndex,
                                    timing.substepCount,
                                    _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                                    proxyBodyId.value,
                                    _savedObjectState.bodyId.value,
                                    bodyAWorldBeforeSolveOk ? "ok" : "fail",
                                    _ragdollAngularProbePreSolve.beforeErrorDegrees,
                                    _ragdollAngularProbePreSolve.beforeGripErrorGameUnits,
                                    _ragdollAngularProbePreSolve.pivotLeverGameUnits,
                                    _ragdollAngularProbePreSolve.linearTorqueWitnessGameUnitsSquared,
                                    _ragdollAngularProbePreSolve.linearTorqueAxisDotRequired,
                                    _ragdollAngularProbePreSolve.requiredAxisWorld.x,
                                    _ragdollAngularProbePreSolve.requiredAxisWorld.y,
                                    _ragdollAngularProbePreSolve.requiredAxisWorld.z,
                                    _ragdollAngularProbePreSolve.requiredAxisProxyLocal.x,
                                    _ragdollAngularProbePreSolve.requiredAxisProxyLocal.y,
                                    _ragdollAngularProbePreSolve.requiredAxisProxyLocal.z,
                                    _ragdollAngularProbePreSolve.angularMotorMaxForce,
                                    _ragdollAngularProbePreSolve.linearMotorMaxForce,
                                    lastGrabPhysicsHz,
                                    lastGrabPhysicsRateForceScale,
                                    _ragdollAngularProbePreSolve.angularMotorTau,
                                    _ragdollAngularProbePreSolve.angularMotorDamping,
                                    _ragdollAngularProbePreSolve.targetToHiggsRelationDegrees,
                                    _ragdollAngularProbePreSolve.transformBFrozenDeltaDegrees,
                                    _ragdollAngularProbePreSolve.pivotBRelationDeltaGameUnits,
                                    _ragdollAngularProbePreSolve.ragdollBRcaRowsErrorDegrees,
                                    _ragdollAngularProbePreSolve.ragdollBRcaColumnsErrorDegrees,
                                    _ragdollAngularProbePreSolve.transformAPivotRoundTripDeltaGameUnits);
                                ROCK_LOG_INFO(Hand,
                                    "{} GRAB_TRACE stage=pre_solve_relation trace={} writeSeq={} flushNext={} queued={} relationInvToDesired={:.3f}gu/{:.2f}deg atomRowsToDesired={:.3f}gu/{:.2f}deg atomRowsToRelationInv={:.2f}deg solverEffToDesired={:.3f}gu/{:.2f}deg solverEffToAtom={:.2f}deg targetRowsToProxyInBody={:.2f}deg tBAtomRelationDelta={:.3f}gu",
                                    handName(),
                                    _grabFrame.traceId,
                                    _grabFrame.traceTargetWriteSequence,
                                    _grabAuthorityProxyFlushSequence + 1,
                                    _grabAuthorityProxyQueuedSequence,
                                    _ragdollAngularProbePreSolve.relationInverseBodyDeltaGameUnits,
                                    _ragdollAngularProbePreSolve.relationInverseBodyDeltaDegrees,
                                    _ragdollAngularProbePreSolve.atomRowsBodyDeltaGameUnits,
                                    _ragdollAngularProbePreSolve.atomRowsBodyDeltaDegrees,
                                    _ragdollAngularProbePreSolve.atomRowsToRelationInverseDegrees,
                                    _ragdollAngularProbePreSolve.solverEffectiveBodyDeltaGameUnits,
                                    _ragdollAngularProbePreSolve.solverEffectiveBodyDeltaDegrees,
                                    _ragdollAngularProbePreSolve.solverEffectiveToAtomDegrees,
                                    _ragdollAngularProbePreSolve.targetRowsToProxyInBodyDegrees,
                                    _ragdollAngularProbePreSolve.pivotBRelationDeltaGameUnits);
                                ROCK_LOG_INFO(Hand,
                                    "{} GRAB_TRACE stage=pre_solve_a_frame trace={} writeSeq={} flushNext={} queued={} targetProxyLive={:.3f}gu/{:.2f}deg relA_targetLive={:.2f}deg conA_targetLive={:.2f}deg targetRelToCon={:.2f}deg liveRelToCon={:.2f}deg solverTargetA={:.3f}gu/{:.2f}deg solverLiveA={:.2f}deg solverTargetVsLive={:.2f}deg rawDelta tA={:.6f} tB={:.6f} target={:.6f}",
                                    handName(),
                                    _grabFrame.traceId,
                                    _grabFrame.traceTargetWriteSequence,
                                    _grabAuthorityProxyFlushSequence + 1,
                                    _grabAuthorityProxyQueuedSequence,
                                    _ragdollAngularProbePreSolve.targetProxyToLiveProxyDeltaGameUnits,
                                    _ragdollAngularProbePreSolve.targetProxyToLiveProxyDeltaDegrees,
                                    _ragdollAngularProbePreSolve.targetRelationAToLiveRelationADegrees,
                                    _ragdollAngularProbePreSolve.targetConstraintAToLiveConstraintADegrees,
                                    _ragdollAngularProbePreSolve.targetRelationAToTargetConstraintADegrees,
                                    _ragdollAngularProbePreSolve.liveRelationAToLiveConstraintADegrees,
                                    _ragdollAngularProbePreSolve.solverEffectiveBodyDeltaGameUnits,
                                    _ragdollAngularProbePreSolve.solverEffectiveBodyDeltaDegrees,
                                    _ragdollAngularProbePreSolve.solverEffectiveLiveABodyDeltaDegrees,
                                    _ragdollAngularProbePreSolve.solverEffectiveTargetALiveADeltaDegrees,
                                    _ragdollAngularProbePreSolve.transformARawMaxDelta,
                                    _ragdollAngularProbePreSolve.transformBRawMaxDelta,
                                    _ragdollAngularProbePreSolve.targetBRcaRawMaxDelta);
                                if (bodyAWorldBeforeSolveOk) {
                                    const auto bodyABasis = grab_transform_telemetry::makeOrientationBasis(bodyAWorldBeforeSolve);
                                    const auto liveBodyBasis = grab_transform_telemetry::makeOrientationBasis(bodyWorldBeforeSolve);
                                    const auto desiredBodyBasis = grab_transform_telemetry::makeOrientationBasis(desiredBodyWorld);
                                    const auto relationBodyBasis = grab_transform_telemetry::makeOrientationBasis(relationInverseBodyWorld);
                                    const auto atomRowsBodyBasis = grab_transform_telemetry::makeOrientationBasis(atomRowsBodyWorld);
                                    const auto solverEffectiveBasis = grab_transform_telemetry::makeOrientationBasis(solverEffectiveBodyWorld);
                                    const auto targetRelationABasis = grab_transform_telemetry::makeOrientationBasis(targetRelationAWorld);
                                    const auto liveRelationABasis = grab_transform_telemetry::makeOrientationBasis(liveRelationAWorld);
                                    const auto targetConstraintABasis = grab_transform_telemetry::makeOrientationBasis(targetConstraintAWorld);
                                    const auto liveConstraintABasis = grab_transform_telemetry::makeOrientationBasis(liveConstraintAWorld);
                                    const auto solverEffectiveLiveABasis = grab_transform_telemetry::makeOrientationBasis(solverEffectiveLiveAWorld);
                                    ROCK_LOG_INFO(Hand,
                                        "{} GRAB_TRACE stage=pre_solve_axismap trace={} writeSeq={} flushNext={} queued={} {} {} {}",
                                        handName(),
                                        _grabFrame.traceId,
                                        _grabFrame.traceTargetWriteSequence,
                                        _grabAuthorityProxyFlushSequence + 1,
                                        _grabAuthorityProxyQueuedSequence,
                                        grab_transform_telemetry::formatBasisCrossMap("liveBodyToA", liveBodyBasis, bodyABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("desiredBodyToA", desiredBodyBasis, bodyABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("atomRowsToA", atomRowsBodyBasis, bodyABasis));
                                    ROCK_LOG_INFO(Hand,
                                        "{} GRAB_TRACE stage=pre_solve_axismap2 trace={} writeSeq={} flushNext={} queued={} {} {} {}",
                                        handName(),
                                        _grabFrame.traceId,
                                        _grabFrame.traceTargetWriteSequence,
                                        _grabAuthorityProxyFlushSequence + 1,
                                        _grabAuthorityProxyQueuedSequence,
                                        grab_transform_telemetry::formatBasisCrossMap("relationInvToA", relationBodyBasis, bodyABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("solverEffToA", solverEffectiveBasis, bodyABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("liveBodyToDesiredBody", liveBodyBasis, desiredBodyBasis));
                                    ROCK_LOG_INFO(Hand,
                                        "{} GRAB_TRACE stage=pre_solve_a_axismap trace={} writeSeq={} flushNext={} queued={} {} {} {} {}",
                                        handName(),
                                        _grabFrame.traceId,
                                        _grabFrame.traceTargetWriteSequence,
                                        _grabAuthorityProxyFlushSequence + 1,
                                        _grabAuthorityProxyQueuedSequence,
                                        grab_transform_telemetry::formatBasisCrossMap("desiredToTargetRelA", desiredBodyBasis, targetRelationABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("solverToTargetRelA", solverEffectiveBasis, targetRelationABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("desiredToLiveRelA", desiredBodyBasis, liveRelationABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("solverLiveToLiveRelA", solverEffectiveLiveABasis, liveRelationABasis));
                                    ROCK_LOG_INFO(Hand,
                                        "{} GRAB_TRACE stage=pre_solve_constraint_a_axismap trace={} writeSeq={} flushNext={} queued={} {} {} {} {}",
                                        handName(),
                                        _grabFrame.traceId,
                                        _grabFrame.traceTargetWriteSequence,
                                        _grabAuthorityProxyFlushSequence + 1,
                                        _grabAuthorityProxyQueuedSequence,
                                        grab_transform_telemetry::formatBasisCrossMap("desiredToTargetConA", desiredBodyBasis, targetConstraintABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("solverToTargetConA", solverEffectiveBasis, targetConstraintABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("desiredToLiveConA", desiredBodyBasis, liveConstraintABasis),
                                        grab_transform_telemetry::formatBasisCrossMap("solverLiveToLiveConA", solverEffectiveLiveABasis, liveConstraintABasis));
                                }
                            }
                        }
                    }
                    if (!angularDriveOk) {
                        ++_grabAuthorityProxyFailedFlushes;
                        _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
                    }

                    _lastAppliedGrabAuthorityQueuedRawHandWorld = queuedRawHandWorld;
                    _lastAppliedGrabAuthorityProxyWorld = pending.proxyWorld;
                    _lastAppliedGrabAuthorityRawHandWorld = pending.rawHandWorld;
                    _lastAppliedGrabAuthorityConsumptionFrameShiftGame = consumptionFrameRebase.shiftGame;
                    _lastAppliedGrabAuthorityConsumptionFrameRebaseStatus = consumptionFrameRebase.status;
                    _lastAppliedGrabAuthoritySourceControllerRoot = pending.sourceControllerRoot;
                    _lastAppliedGrabAuthorityConsumptionControllerRoot = consumptionControllerRoot;
                    _lastAppliedGrabAuthoritySourceGameFrameIndex = pending.sourceGameFrameIndex;
                    _lastAppliedGrabAuthoritySourceQueueSequence = _grabAuthorityProxyQueuedSequence;
                    _grabAuthorityProxyLastFlushTiming = timing;
                    _hasLastAppliedGrabAuthorityProxyWorld = true;
                    _grabAuthorityProxyLastFlushDeltaSeconds = driveDelta;
                    ++_grabAuthorityProxyFlushSequence;
                    flushSequence = _grabAuthorityProxyFlushSequence;
                    queuedSequence = _grabAuthorityProxyQueuedSequence;
                    ++_grabAuthorityProxyLogCounter;
                    if (flushSequence <= 16 || _grabAuthorityProxyLogCounter >= 45 ||
                        !proxyReadbackBetweenOk ||
                        !angularDriveOk ||
                        resampleAction == grab_authority_source_clock::ResampleAction::Rebase ||
                        (!consumptionFrameRebase.valid &&
                            consumptionFrameRebase.status != grab_authority_source_clock::ConsumptionFrameRebaseStatus::Stationary) ||
                        proxyReadbackBetweenPositionErrorGameUnits > 1.0f ||
                        proxyReadbackBetweenRotationErrorDegrees > 1.0f) {
                        _grabAuthorityProxyLogCounter = 0;
                        shouldLog = true;
                    }
                }
            }
        }

        if (!proxyDriveOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab drive failed; release queued: proxyBody={} livePalm={} driven={} stale={} missing={} ownerMismatch={} substep={}/{} dt={:.6f}",
                handName(),
                proxyBodyId.value,
                livePalmReferenceOk ? "ok" : "fail",
                proxyDriveResult.driven ? "ok" : "fail",
                proxyDriveResult.skippedStale ? "yes" : "no",
                proxyDriveResult.missingBody ? "yes" : "no",
                proxyDriveResult.bodyCollisionObjectMismatch ? "yes" : "no",
                timing.substepIndex,
                timing.substepCount,
                havok_physics_timing::driveDeltaSeconds(timing));
            return;
        }

        if (!targetUpdateOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab target update failed; release queued: proxyBody={} constraint={} substep={}/{}",
                handName(),
                proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount);
            return;
        }

        if (!angularDriveOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab angular drive failed; release queued: proxyBody={} objBody={} constraint={} substep={}/{}",
                handName(),
                proxyBodyId.value,
                _savedObjectState.bodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount);
            return;
        }

        if (shouldLog && g_rockConfig.rockDebugGrabFrameLogging) {
            std::uint32_t filterInfo = 0;
            const bool filterReadOk = havok_runtime::tryReadFilterInfo(world, proxyBodyId, filterInfo);
            ROCK_LOG_DEBUG(Hand,
                "{} PROXY GRAB AUTHORITY: seq={}/{} diag=bodyFrameConstraint+queuedTarget+generatedKeyframedProxy proxyBody={} constraint={} substep={}/{} dt={:.6f} resample={} rebases={} consumptionRebase={} shift=({:.3f},{:.3f},{:.3f}) controller(src/now)={:016X}/{:016X} vtable(src/now)={:016X}/{:016X} scaleRev(src/now)={}/{} targetSrc={} target=({:.1f},{:.1f},{:.1f}) desiredBody=({:.1f},{:.1f},{:.1f}) angularAuthority={} angularRef={} solverAngular=ragdollAtom angularBudget={:.3f} pivotB=({:.2f},{:.2f},{:.2f}) err={:.2f}gu rotErr={:.2f}deg proxyDrive=driveToKeyFrame palmRef={} palmSrc={} palmMotion={} proxyVelSource={} proxyVel={:.3f}hk proxyAngVel={:.3f}rad/s longLever={:.1f}gu proxyRead={} proxySrc={} proxyMotion={} proxyErr={:.3f}gu/{:.2f}deg forceBudget={:.2f} colliding={} filterRead={} filter=0x{:08X} noContact={}",
                handName(),
                flushSequence,
                queuedSequence,
                proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount,
                havok_physics_timing::driveDeltaSeconds(timing),
                grab_authority_source_clock::resampleActionName(resampleAction),
                resampleRebaseCount,
                grab_authority_source_clock::consumptionFrameRebaseStatusName(consumptionFrameRebase.status),
                consumptionFrameRebase.shiftGame.x,
                consumptionFrameRebase.shiftGame.y,
                consumptionFrameRebase.shiftGame.z,
                pending.sourceControllerRoot.controllerIdentity,
                consumptionControllerRoot.controllerIdentity,
                pending.sourceControllerRoot.controllerVtable,
                consumptionControllerRoot.controllerVtable,
                pending.sourceControllerRoot.physicsScaleRevision,
                consumptionControllerRoot.physicsScaleRevision,
                pending.proxyFrameSource ? pending.proxyFrameSource : "unknown",
                pending.proxyWorld.translate.x,
                pending.proxyWorld.translate.y,
                pending.proxyWorld.translate.z,
                desiredBodyWorld.translate.x,
                desiredBodyWorld.translate.y,
                desiredBodyWorld.translate.z,
                grabAngularAuthorityName(angularAuthority),
                kGrabObjectRotationReferenceName,
                angularMotorBudget,
                activePivotBBodyLocalGame.x,
                activePivotBBodyLocalGame.y,
                activePivotBBodyLocalGame.z,
                pending.grabPositionErrorGameUnits,
                pending.grabRotationErrorDegrees,
                livePalmReferenceOk ? "ok" : "fail",
                body_frame::bodyFrameSourceCode(livePalmReference.source),
                livePalmReference.motionIndex,
                proxyVelocityTelemetryOk ? "palmMotion" : "computed",
                proxyLinearVelocityHavokMagnitude,
                proxyAngularVelocityRadiansPerSecond,
                _grabFrame.longObjectLeverGameUnits,
                proxyReadbackBetweenOk ? "ok" : "fail",
                body_frame::bodyFrameSourceCode(proxyReadbackSourceBetween),
                proxyReadbackMotionIndexBetween,
                proxyReadbackBetweenPositionErrorGameUnits,
                proxyReadbackBetweenRotationErrorDegrees,
                pending.authorityForceScale,
                pending.heldBodyColliding ? "yes" : "no",
                filterReadOk ? "ok" : "fail",
                filterInfo,
                grab_authority_proxy::hasNoContactFilterInfo(filterInfo) ? "yes" : "no");
        }
    }


    void Hand::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world) {
            return;
        }

        const bool debugGrabFrameLogging = g_rockConfig.rockDebugGrabFrameLogging;
        const bool timelineTraceLogging = grabTimelineTraceEnabled();

        RE::hknpBodyId proxyBodyId{ INVALID_BODY_ID };
        RE::hknpBodyId objectBodyId{ INVALID_BODY_ID };
        RE::NiTransform targetProxyWorld{};
        RE::NiTransform targetRawHandWorld{};
        RE::NiTransform proxyAuthorityHandSpace{};
        RE::NiTransform proxyAuthorityBodyHandSpace{};
        RE::NiPoint3 pivotBConstraintLocalGame{};
        std::uint64_t queuedSequence = 0;
        std::uint64_t flushSequence = 0;
        std::uint64_t afterSolveSequence = 0;
        std::uint32_t constraintId = 0x7FFF'FFFFu;
        GrabAngularAuthority angularAuthority = GrabAngularAuthority::HknpRagdollMotorAtom;
        RagdollAngularProbePreSolve ragdollAngularProbePreSolve{};
        {
            // Guard the proxy lease while the after-solve snapshot is copied.
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            const bool hasAuthority = _grabAuthorityProxy.isValid() &&
                                      _grabAuthorityProxyHknpWorld == world &&
                                      _hasLastAppliedGrabAuthorityProxyWorld;
            if (!hasAuthority) {
                return;
            }

            proxyBodyId = _grabAuthorityProxy.getBodyId();
            objectBodyId = _savedObjectState.bodyId;
            targetProxyWorld = _lastAppliedGrabAuthorityProxyWorld;
            targetRawHandWorld = _lastAppliedGrabAuthorityRawHandWorld;
            proxyAuthorityHandSpace = _grabFrame.proxyAuthorityHandSpace;
            proxyAuthorityBodyHandSpace = _grabFrame.proxyAuthorityBodyHandSpace;
            pivotBConstraintLocalGame = activeProxyConstraintPivotBLocalGame();
            queuedSequence = _grabAuthorityProxyQueuedSequence;
            flushSequence = _grabAuthorityProxyFlushSequence;
            afterSolveSequence = ++_grabAuthorityProxyAfterSolveLogCounter;
            _grabAuthorityProxyLastAfterSolveTiming = timing;
            constraintId = _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu;
            angularAuthority = _activeConstraint.angularAuthority;
            ragdollAngularProbePreSolve = _ragdollAngularProbePreSolve;
        }

        const bool shouldSampleForAnomaly = g_rockConfig.rockDebugGrabAfterSolveAnomalySampling &&
                                            (afterSolveSequence <= 16u || (afterSolveSequence % 30u) == 0u);
        /*
         * After-solve readback is diagnostic only. Keeping it behind explicit
         * grab diagnostics avoids paying body readback, constraint atom, and
         * basis math costs during normal two-hand held-object gameplay.
         */
        if (!debugGrabFrameLogging && !timelineTraceLogging && !shouldSampleForAnomaly) {
            return;
        }
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAuthorityAfterSolveDiagnostics);

        bool hasConstraintFrameMetrics = false;
        float pivotBRelationDeltaGameUnits = -1.0f;
        float targetToHiggsRelationDegrees = -1.0f;
        float transformBFrozenDeltaDegrees = -1.0f;
        {
            // Guard the live constraint data during the diagnostic read.
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            if (_activeConstraint.constraintData) {
                const auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
                const auto* targetBRca = reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
                const auto* transformBRotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_COL0);
                const auto* transformBTranslation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
                const RE::NiTransform desiredBodyWorldForConstraintMetrics =
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(targetProxyWorld, _grabFrame.proxyAuthorityBodyHandSpace);
                const RE::NiTransform desiredBodyTransformHandSpace =
                    grab_frame_math::objectInGeneratedProxyLocalSpace(targetProxyWorld, desiredBodyWorldForConstraintMetrics);
                const RE::NiTransform desiredBodyToHandSpace = invertTransform(desiredBodyTransformHandSpace);
                const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(targetBRca);
                const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformBRotation);
                const RE::NiPoint3 constraintTransformBLocalGame{
                    transformBTranslation[0] * havokToGameScale(),
                    transformBTranslation[1] * havokToGameScale(),
                    transformBTranslation[2] * havokToGameScale(),
                };
                const RE::NiPoint3 desiredTransformBLocalGame =
                    grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformHandSpace, _grabFrame.pivotAHandBodyLocalGame);
                pivotBRelationDeltaGameUnits =
                    pointDistanceGameUnits(constraintTransformBLocalGame, desiredTransformBLocalGame);
                targetToHiggsRelationDegrees = rotationDeltaDegrees(targetAsHkRows, desiredBodyToHandSpace.rotate);
                transformBFrozenDeltaDegrees = rotationDeltaDegrees(transformBAsHkColumns, desiredBodyToHandSpace.rotate);
                hasConstraintFrameMetrics = true;
            }
        }

        RE::NiTransform proxyReadback{};
        RE::NiTransform objectReadback{};
        body_frame::BodyFrameSource proxySource = body_frame::BodyFrameSource::Fallback;
        body_frame::BodyFrameSource objectSource = body_frame::BodyFrameSource::Fallback;
        std::uint32_t proxyMotionIndex = body_frame::kFreeMotionIndex;
        std::uint32_t objectMotionIndex = body_frame::kFreeMotionIndex;
        objectSource = body_frame::BodyFrameSource::BodyTransform;
        objectMotionIndex = body_frame::kFreeMotionIndex;
        const bool proxyOk = tryResolveLiveBodyWorldTransform(world, proxyBodyId, proxyReadback, &proxySource, &proxyMotionIndex);
        const bool objectOk = tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, objectReadback);
        if (!proxyOk) {
            proxySource = body_frame::BodyFrameSource::Fallback;
        }
        if (!objectOk) {
            objectSource = body_frame::BodyFrameSource::Fallback;
        }

        const RE::NiTransform desiredObjectFromTarget =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(targetProxyWorld, proxyAuthorityHandSpace);
        const RE::NiTransform desiredBodyFromTarget =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(targetProxyWorld, proxyAuthorityBodyHandSpace);
        const RE::NiTransform desiredObjectFromLiveProxy = proxyOk ?
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyReadback, proxyAuthorityHandSpace) :
            desiredObjectFromTarget;
        const RE::NiTransform desiredBodyFromLiveProxy = proxyOk ?
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyReadback, proxyAuthorityBodyHandSpace) :
            desiredBodyFromTarget;

        const float proxyTargetPositionErrorGameUnits =
            proxyOk ? pointDistanceGameUnits(proxyReadback.translate, targetProxyWorld.translate) : -1.0f;
        const float proxyTargetRotationErrorDegrees =
            proxyOk ? rotationDeltaDegrees(proxyReadback.rotate, targetProxyWorld.rotate) : -1.0f;
        const float objectTargetPositionErrorGameUnits =
            objectOk ? pointDistanceGameUnits(objectReadback.translate, desiredBodyFromTarget.translate) : -1.0f;
        const float objectTargetRotationErrorDegrees =
            objectOk ? rotationDeltaDegrees(objectReadback.rotate, desiredBodyFromTarget.rotate) : -1.0f;
        const float objectLiveProxyPositionErrorGameUnits =
            objectOk ? pointDistanceGameUnits(objectReadback.translate, desiredBodyFromLiveProxy.translate) : -1.0f;
        const float objectLiveProxyRotationErrorDegrees =
            objectOk ? rotationDeltaDegrees(objectReadback.rotate, desiredBodyFromLiveProxy.rotate) : -1.0f;
        const float rawToTargetProxyRotationDegrees = rotationDeltaDegrees(targetRawHandWorld.rotate, targetProxyWorld.rotate);
        const float rawToLiveProxyRotationDegrees =
            proxyOk ? rotationDeltaDegrees(targetRawHandWorld.rotate, proxyReadback.rotate) : -1.0f;
        const GrabPalmBasisDelta targetPalmBasisDelta = computeGrabPalmBasisDelta(targetRawHandWorld, targetProxyWorld);
        const GrabPalmBasisDelta livePalmBasisDelta =
            proxyOk ? computeGrabPalmBasisDelta(targetRawHandWorld, proxyReadback) : targetPalmBasisDelta;

        float gripTargetErrorGameUnits = -1.0f;
        float gripLiveProxyErrorGameUnits = -1.0f;
        if (objectOk) {
            const RE::NiPoint3 liveGripWorld = transform_math::localPointToWorld(objectReadback, pivotBConstraintLocalGame);
            const RE::NiPoint3 targetGripWorld = transform_math::localPointToWorld(desiredBodyFromTarget, pivotBConstraintLocalGame);
            const RE::NiPoint3 liveProxyGripWorld = transform_math::localPointToWorld(desiredBodyFromLiveProxy, pivotBConstraintLocalGame);
            gripTargetErrorGameUnits = pointDistanceGameUnits(liveGripWorld, targetGripWorld);
            gripLiveProxyErrorGameUnits = pointDistanceGameUnits(liveGripWorld, liveProxyGripWorld);
        }

        /*
         * Per-substep POST-SOLVE ripple probe (2026-07-13 stutter hunt): every
         * pre-solve metric in this file is blind to what the constraint motors
         * actually did to the object this substep — the signal the eye sees.
         * This line samples the object body and its solver-output velocities
         * AFTER the solve, once per substep, continuously while held. Offline
         * analysis differentiates obj/proxy positions per substep and checks
         * the velocity ripple against dt quantization. Debug-flag gated; it is
         * the replacement for trusting pre-solve telemetry.
         */
        if (debugGrabFrameLogging) {
            RE::NiPoint3 objectLinearVelocityHavok{};
            RE::NiPoint3 objectAngularVelocityRadians{};
            if (objectOk) {
                if (auto* motion = havok_runtime::getBodyMotion(world, objectBodyId)) {
                    objectLinearVelocityHavok = RE::NiPoint3{
                        motion->linearVelocity.x,
                        motion->linearVelocity.y,
                        motion->linearVelocity.z,
                    };
                    objectAngularVelocityRadians = RE::NiPoint3{
                        motion->angularVelocity.x,
                        motion->angularVelocity.y,
                        motion->angularVelocity.z,
                    };
                }
            }
            ROCK_LOG_DEBUG(Hand,
                "{} HELD_POSTSOLVE: afterSeq={} substep={}/{} dt={:.6f} objOk={} obj=({:.3f},{:.3f},{:.3f}) objVelHk=({:.4f},{:.4f},{:.4f}) objAngVel={:.4f} objErrTgt={:.3f}gu/{:.2f}deg objErrLive={:.3f}gu/{:.2f}deg gripErrLive={:.3f}gu proxyOk={} proxyErr={:.3f}gu/{:.2f}deg tgt=({:.3f},{:.3f},{:.3f})",
                handName(),
                afterSolveSequence,
                timing.substepIndex,
                timing.substepCount,
                havok_physics_timing::driveDeltaSeconds(timing),
                objectOk ? "y" : "n",
                objectReadback.translate.x,
                objectReadback.translate.y,
                objectReadback.translate.z,
                objectLinearVelocityHavok.x,
                objectLinearVelocityHavok.y,
                objectLinearVelocityHavok.z,
                vectorMagnitude(objectAngularVelocityRadians),
                objectTargetPositionErrorGameUnits,
                objectTargetRotationErrorDegrees,
                objectLiveProxyPositionErrorGameUnits,
                objectLiveProxyRotationErrorDegrees,
                gripLiveProxyErrorGameUnits,
                proxyOk ? "y" : "n",
                proxyTargetPositionErrorGameUnits,
                proxyTargetRotationErrorDegrees,
                desiredBodyFromTarget.translate.x,
                desiredBodyFromTarget.translate.y,
                desiredBodyFromTarget.translate.z);
        }

        const bool hasRagdollAngularProbe =
            ragdollAngularProbePreSolve.valid &&
            ragdollAngularProbePreSolve.objectBodyId.value == objectBodyId.value;
        if (hasRagdollAngularProbe && objectOk) {
            RE::NiPoint3 angularVelocityAfterSolve{};
            if (auto* motion = havok_runtime::getBodyMotion(world, objectBodyId)) {
                angularVelocityAfterSolve = RE::NiPoint3{
                    motion->angularVelocity.x,
                    motion->angularVelocity.y,
                    motion->angularVelocity.z,
                };
            }

            const RE::NiPoint3 velocityAxisAfterSolve = normalizeOrZero(angularVelocityAfterSolve);
            /*
             * Keep the response axis in the same proxy-local basis as the required
             * correction axis so left/right parity can be compared without a moving
             * world-space frame hiding mirrored sign errors.
             */
            const RE::NiPoint3 velocityAxisProxyLocal =
                ragdollAngularProbePreSolve.bodyAWorldValid ?
                rotationAxisProxyLocal(ragdollAngularProbePreSolve.bodyAWorldBefore.rotate, velocityAxisAfterSolve) :
                RE::NiPoint3{};
            const float axisDot = dotProduct(ragdollAngularProbePreSolve.requiredAxisWorld, velocityAxisAfterSolve);
            const float angularSpeedBefore = vectorMagnitude(ragdollAngularProbePreSolve.angularVelocityBeforeRadians);
            const float angularSpeedAfter = vectorMagnitude(angularVelocityAfterSolve);
            const float afterErrorDegrees =
                rotationDeltaDegrees(objectReadback.rotate, ragdollAngularProbePreSolve.desiredBodyWorld.rotate);
            const float errorReductionDegrees = ragdollAngularProbePreSolve.beforeErrorDegrees - afterErrorDegrees;
            const RE::NiPoint3 liveGripAfterSolve =
                transform_math::localPointToWorld(objectReadback, pivotBConstraintLocalGame);
            const RE::NiPoint3 targetGripFromProbe =
                transform_math::localPointToWorld(ragdollAngularProbePreSolve.desiredBodyWorld, pivotBConstraintLocalGame);
            const float afterGripErrorGameUnits = pointDistanceGameUnits(liveGripAfterSolve, targetGripFromProbe);
            const bool staleProbe = ragdollAngularProbePreSolve.flushSequence != flushSequence;
            const bool shouldLogRagdollProbe =
                !staleProbe &&
                (afterSolveSequence <= 16u ||
                    ragdollAngularProbePreSolve.beforeErrorDegrees > 10.0f ||
                    afterErrorDegrees > 10.0f ||
                    errorReductionDegrees < 0.25f ||
                    axisDot < 0.35f);

            if (!staleProbe &&
                grabTimelineTraceEnabled() &&
                shouldLogGrabTimelineSequence(ragdollAngularProbePreSolve.targetWriteSequence)) {
                ROCK_LOG_INFO(Hand,
                    "{} GRAB_TRACE stage=after_solve trace={} writeSeq={} flush={} queued={} afterSeq={} substep={}/{} constraint={} proxyBody={} objBody={} stale={} beforeErr={:.2f}deg afterErr={:.2f}deg reduce={:.2f}deg axisDot={:.3f} beforeAng={:.3f}rad/s afterAng={:.3f}rad/s gripBefore={:.2f}gu gripAfter={:.2f}gu proxyErr={:.3f}gu/{:.2f}deg objectErr={:.2f}gu/{:.2f}deg objectLiveProxyErr={:.2f}gu/{:.2f}deg pivotLever={:.2f}gu linTorque={:.3f}gu2 linTorqueDotReq={:.2f} reqAxisProxy=({:.3f},{:.3f},{:.3f}) velAxisProxy=({:.3f},{:.3f},{:.3f}) forceA={:.0f} forceL={:.0f} targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg pivotBRelationDelta={:.3f}gu",
                    handName(),
                    ragdollAngularProbePreSolve.traceId,
                    ragdollAngularProbePreSolve.targetWriteSequence,
                    flushSequence,
                    queuedSequence,
                    afterSolveSequence,
                    timing.substepIndex,
                    timing.substepCount,
                    constraintId,
                    proxyBodyId.value,
                    objectBodyId.value,
                    staleProbe ? "yes" : "no",
                    ragdollAngularProbePreSolve.beforeErrorDegrees,
                    afterErrorDegrees,
                    errorReductionDegrees,
                    std::isfinite(axisDot) ? axisDot : 0.0f,
                    angularSpeedBefore,
                    angularSpeedAfter,
                    ragdollAngularProbePreSolve.beforeGripErrorGameUnits,
                    afterGripErrorGameUnits,
                    proxyTargetPositionErrorGameUnits,
                    proxyTargetRotationErrorDegrees,
                    objectTargetPositionErrorGameUnits,
                    objectTargetRotationErrorDegrees,
                    objectLiveProxyPositionErrorGameUnits,
                    objectLiveProxyRotationErrorDegrees,
                    ragdollAngularProbePreSolve.pivotLeverGameUnits,
                    ragdollAngularProbePreSolve.linearTorqueWitnessGameUnitsSquared,
                    ragdollAngularProbePreSolve.linearTorqueAxisDotRequired,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.x,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.y,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.z,
                    velocityAxisProxyLocal.x,
                    velocityAxisProxyLocal.y,
                    velocityAxisProxyLocal.z,
                    ragdollAngularProbePreSolve.angularMotorMaxForce,
                    ragdollAngularProbePreSolve.linearMotorMaxForce,
                    ragdollAngularProbePreSolve.targetToHiggsRelationDegrees,
                    ragdollAngularProbePreSolve.transformBFrozenDeltaDegrees,
                    ragdollAngularProbePreSolve.pivotBRelationDeltaGameUnits);
            }

            if (shouldLogRagdollProbe) {
                ROCK_LOG_SAMPLE_WARN(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} RAGDOLL ANGULAR PROBE: seq={}/{} afterSeq={} probeSeq={} stale={} substep={}/{} beforeErr={:.1f}deg afterErr={:.1f}deg reduce={:.1f}deg axisDot={:.2f} reqAxis=({:.2f},{:.2f},{:.2f}) velAxis=({:.2f},{:.2f},{:.2f}) reqAxisProxy=({:.2f},{:.2f},{:.2f}) velAxisProxy=({:.2f},{:.2f},{:.2f}) beforeAng={:.2f}rad/s afterAng={:.2f}rad/s forceA={:.0f} forceL={:.0f} tau={:.3f} damp={:.2f} ragdoll={} targetToHiggsRelation={:.1f}deg transformBFrozenDelta={:.1f}deg pivotBRelationDelta={:.3f}gu pivotARoundTrip={:.3f}gu gripBefore={:.2f}gu gripAfter={:.2f}gu pivotLever={:.2f}gu linTorque={:.3f}gu2 linTorqueDotReq={:.2f} linTorqueAxisProxy=({:.2f},{:.2f},{:.2f}) angularRef={} phase={} body={} motion={}",
                    handName(),
                    flushSequence,
                    queuedSequence,
                    afterSolveSequence,
                    ragdollAngularProbePreSolve.flushSequence,
                    staleProbe ? "yes" : "no",
                    timing.substepIndex,
                    timing.substepCount,
                    ragdollAngularProbePreSolve.beforeErrorDegrees,
                    afterErrorDegrees,
                    errorReductionDegrees,
                    std::isfinite(axisDot) ? axisDot : 0.0f,
                    ragdollAngularProbePreSolve.requiredAxisWorld.x,
                    ragdollAngularProbePreSolve.requiredAxisWorld.y,
                    ragdollAngularProbePreSolve.requiredAxisWorld.z,
                    velocityAxisAfterSolve.x,
                    velocityAxisAfterSolve.y,
                    velocityAxisAfterSolve.z,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.x,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.y,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.z,
                    velocityAxisProxyLocal.x,
                    velocityAxisProxyLocal.y,
                    velocityAxisProxyLocal.z,
                    angularSpeedBefore,
                    angularSpeedAfter,
                    ragdollAngularProbePreSolve.angularMotorMaxForce,
                    ragdollAngularProbePreSolve.linearMotorMaxForce,
                    ragdollAngularProbePreSolve.angularMotorTau,
                    ragdollAngularProbePreSolve.angularMotorDamping,
                    ragdollAngularProbePreSolve.ragdollMotorEnabled ? "yes" : "no",
                    ragdollAngularProbePreSolve.targetToHiggsRelationDegrees,
                    ragdollAngularProbePreSolve.transformBFrozenDeltaDegrees,
                    ragdollAngularProbePreSolve.pivotBRelationDeltaGameUnits,
                    ragdollAngularProbePreSolve.transformAPivotRoundTripDeltaGameUnits,
                    ragdollAngularProbePreSolve.beforeGripErrorGameUnits,
                    afterGripErrorGameUnits,
                    ragdollAngularProbePreSolve.pivotLeverGameUnits,
                    ragdollAngularProbePreSolve.linearTorqueWitnessGameUnitsSquared,
                    ragdollAngularProbePreSolve.linearTorqueAxisDotRequired,
                    ragdollAngularProbePreSolve.linearTorqueAxisProxyLocal.x,
                    ragdollAngularProbePreSolve.linearTorqueAxisProxyLocal.y,
                    ragdollAngularProbePreSolve.linearTorqueAxisProxyLocal.z,
                    kGrabObjectRotationReferenceName,
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    objectBodyId.value,
                    objectMotionIndex);

                ROCK_LOG_SAMPLE_WARN(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} RAGDOLL FRAME RAW: seq={}/{} target_bRca_raw=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})] tA_rows=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})] tB_rows=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})]",
                    handName(),
                    flushSequence,
                    queuedSequence,
                    ragdollAngularProbePreSolve.targetBRcaRaw[0],
                    ragdollAngularProbePreSolve.targetBRcaRaw[1],
                    ragdollAngularProbePreSolve.targetBRcaRaw[2],
                    ragdollAngularProbePreSolve.targetBRcaRaw[4],
                    ragdollAngularProbePreSolve.targetBRcaRaw[5],
                    ragdollAngularProbePreSolve.targetBRcaRaw[6],
                    ragdollAngularProbePreSolve.targetBRcaRaw[8],
                    ragdollAngularProbePreSolve.targetBRcaRaw[9],
                    ragdollAngularProbePreSolve.targetBRcaRaw[10],
                    ragdollAngularProbePreSolve.transformARotation.entry[0][0],
                    ragdollAngularProbePreSolve.transformARotation.entry[0][1],
                    ragdollAngularProbePreSolve.transformARotation.entry[0][2],
                    ragdollAngularProbePreSolve.transformARotation.entry[1][0],
                    ragdollAngularProbePreSolve.transformARotation.entry[1][1],
                    ragdollAngularProbePreSolve.transformARotation.entry[1][2],
                    ragdollAngularProbePreSolve.transformARotation.entry[2][0],
                    ragdollAngularProbePreSolve.transformARotation.entry[2][1],
                    ragdollAngularProbePreSolve.transformARotation.entry[2][2],
                    ragdollAngularProbePreSolve.transformBRotation.entry[0][0],
                    ragdollAngularProbePreSolve.transformBRotation.entry[0][1],
                    ragdollAngularProbePreSolve.transformBRotation.entry[0][2],
                    ragdollAngularProbePreSolve.transformBRotation.entry[1][0],
                    ragdollAngularProbePreSolve.transformBRotation.entry[1][1],
                    ragdollAngularProbePreSolve.transformBRotation.entry[1][2],
                    ragdollAngularProbePreSolve.transformBRotation.entry[2][0],
                    ragdollAngularProbePreSolve.transformBRotation.entry[2][1],
                    ragdollAngularProbePreSolve.transformBRotation.entry[2][2]);

                ROCK_LOG_SAMPLE_WARN(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} RAGDOLL FRAME LIVE: seq={}/{} liveA={} liveA_rows=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})] liveB_rows=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})]",
                    handName(),
                    flushSequence,
                    queuedSequence,
                    ragdollAngularProbePreSolve.bodyAWorldValid ? "ok" : "fail",
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[0][0],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[0][1],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[0][2],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[1][0],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[1][1],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[1][2],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[2][0],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[2][1],
                    ragdollAngularProbePreSolve.bodyAWorldBefore.rotate.entry[2][2],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[0][0],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[0][1],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[0][2],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[1][0],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[1][1],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[1][2],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[2][0],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[2][1],
                    ragdollAngularProbePreSolve.bodyWorldBefore.rotate.entry[2][2]);

                ROCK_LOG_SAMPLE_WARN(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} RAGDOLL FRAME ERROR: seq={}/{} appearsSolve=bRcaRows bRcaRowsErr={:.1f}deg bRcaColsErr={:.1f}deg aRcbRowsInvErr={:.1f}deg aRcbColsInvErr={:.1f}deg intendedBodyErr={:.1f}->{:.1f}deg targetToHiggsRelation={:.1f}deg transformBFrozenDelta={:.1f}deg pivotBRelationDelta={:.3f}gu pivotARoundTrip={:.3f}gu relationInv={:.3f}gu/{:.1f}deg atomRows={:.3f}gu/{:.1f}deg atomRel={:.1f}deg solverEff={:.3f}gu/{:.1f}deg effAtom={:.1f}deg rowsProxyInBody={:.1f}deg linTorque={:.3f}gu2 linTorqueDotReq={:.2f} reqAxisProxy=({:.2f},{:.2f},{:.2f})",
                    handName(),
                    flushSequence,
                    queuedSequence,
                    ragdollAngularProbePreSolve.ragdollBRcaRowsErrorDegrees,
                    ragdollAngularProbePreSolve.ragdollBRcaColumnsErrorDegrees,
                    ragdollAngularProbePreSolve.ragdollARcbRowsInverseErrorDegrees,
                    ragdollAngularProbePreSolve.ragdollARcbColumnsInverseErrorDegrees,
                    ragdollAngularProbePreSolve.beforeErrorDegrees,
                    afterErrorDegrees,
                    ragdollAngularProbePreSolve.targetToHiggsRelationDegrees,
                    ragdollAngularProbePreSolve.transformBFrozenDeltaDegrees,
                    ragdollAngularProbePreSolve.pivotBRelationDeltaGameUnits,
                    ragdollAngularProbePreSolve.transformAPivotRoundTripDeltaGameUnits,
                    ragdollAngularProbePreSolve.relationInverseBodyDeltaGameUnits,
                    ragdollAngularProbePreSolve.relationInverseBodyDeltaDegrees,
                    ragdollAngularProbePreSolve.atomRowsBodyDeltaGameUnits,
                    ragdollAngularProbePreSolve.atomRowsBodyDeltaDegrees,
                    ragdollAngularProbePreSolve.atomRowsToRelationInverseDegrees,
                    ragdollAngularProbePreSolve.solverEffectiveBodyDeltaGameUnits,
                    ragdollAngularProbePreSolve.solverEffectiveBodyDeltaDegrees,
                    ragdollAngularProbePreSolve.solverEffectiveToAtomDegrees,
                    ragdollAngularProbePreSolve.targetRowsToProxyInBodyDegrees,
                    ragdollAngularProbePreSolve.linearTorqueWitnessGameUnitsSquared,
                    ragdollAngularProbePreSolve.linearTorqueAxisDotRequired,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.x,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.y,
                    ragdollAngularProbePreSolve.requiredAxisProxyLocal.z);
                ROCK_LOG_SAMPLE_WARN(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} RAGDOLL A FRAME: seq={}/{} targetProxyLive={:.3f}gu/{:.1f}deg relA_targetLive={:.1f}deg conA_targetLive={:.1f}deg targetRelToCon={:.1f}deg liveRelToCon={:.1f}deg solverTargetA={:.3f}gu/{:.1f}deg solverLiveA={:.1f}deg solverTargetVsLive={:.1f}deg rawDelta tA={:.6f} tB={:.6f} target={:.6f}",
                    handName(),
                    flushSequence,
                    queuedSequence,
                    ragdollAngularProbePreSolve.targetProxyToLiveProxyDeltaGameUnits,
                    ragdollAngularProbePreSolve.targetProxyToLiveProxyDeltaDegrees,
                    ragdollAngularProbePreSolve.targetRelationAToLiveRelationADegrees,
                    ragdollAngularProbePreSolve.targetConstraintAToLiveConstraintADegrees,
                    ragdollAngularProbePreSolve.targetRelationAToTargetConstraintADegrees,
                    ragdollAngularProbePreSolve.liveRelationAToLiveConstraintADegrees,
                    ragdollAngularProbePreSolve.solverEffectiveBodyDeltaGameUnits,
                    ragdollAngularProbePreSolve.solverEffectiveBodyDeltaDegrees,
                    ragdollAngularProbePreSolve.solverEffectiveLiveABodyDeltaDegrees,
                    ragdollAngularProbePreSolve.solverEffectiveTargetALiveADeltaDegrees,
                    ragdollAngularProbePreSolve.transformARawMaxDelta,
                    ragdollAngularProbePreSolve.transformBRawMaxDelta,
                    ragdollAngularProbePreSolve.targetBRcaRawMaxDelta);
                if (ragdollAngularProbePreSolve.bodyAWorldValid) {
                    const auto bodyABasis = grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.liveRelationAWorld);
                    const auto targetABasis = grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.targetRelationAWorld);
                    const auto targetConstraintABasis =
                        grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.targetConstraintAWorld);
                    const auto liveConstraintABasis =
                        grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.liveConstraintAWorld);
                    const auto liveBeforeBasis = grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.bodyWorldBefore);
                    const auto liveAfterBasis = grab_transform_telemetry::makeOrientationBasis(objectReadback);
                    const auto desiredBasis = grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.desiredBodyWorld);
                    const auto atomRowsBasis = grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.atomRowsBodyWorld);
                    const auto solverEffectiveBasis = grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.solverEffectiveBodyWorld);
                    const auto solverEffectiveLiveABasis =
                        grab_transform_telemetry::makeOrientationBasis(ragdollAngularProbePreSolve.solverEffectiveLiveAWorld);
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} RAGDOLL AXISMAP: seq={}/{} afterSeq={} ref=liveRelA {} {} {}",
                        handName(),
                        flushSequence,
                        queuedSequence,
                        afterSolveSequence,
                        grab_transform_telemetry::formatBasisCrossMap("liveBeforeToA", liveBeforeBasis, bodyABasis),
                        grab_transform_telemetry::formatBasisCrossMap("liveAfterToA", liveAfterBasis, bodyABasis),
                        grab_transform_telemetry::formatBasisCrossMap("desiredToA", desiredBasis, bodyABasis));
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} RAGDOLL AXISMAP TARGETS: seq={}/{} afterSeq={} {} {} {}",
                        handName(),
                        flushSequence,
                        queuedSequence,
                        afterSolveSequence,
                        grab_transform_telemetry::formatBasisCrossMap("atomRowsToA", atomRowsBasis, bodyABasis),
                        grab_transform_telemetry::formatBasisCrossMap("solverEffToA", solverEffectiveBasis, bodyABasis),
                        grab_transform_telemetry::formatBasisCrossMap("liveAfterToDesired", liveAfterBasis, desiredBasis));
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} RAGDOLL A AXISMAP: seq={}/{} afterSeq={} {} {} {} {}",
                        handName(),
                        flushSequence,
                        queuedSequence,
                        afterSolveSequence,
                        grab_transform_telemetry::formatBasisCrossMap("desiredToTargetRelA", desiredBasis, targetABasis),
                        grab_transform_telemetry::formatBasisCrossMap("solverToTargetRelA", solverEffectiveBasis, targetABasis),
                        grab_transform_telemetry::formatBasisCrossMap("desiredToLiveRelA", desiredBasis, bodyABasis),
                        grab_transform_telemetry::formatBasisCrossMap("solverLiveToLiveRelA", solverEffectiveLiveABasis, bodyABasis));
                    ROCK_LOG_SAMPLE_WARN(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} RAGDOLL CONSTRAINT A AXISMAP: seq={}/{} afterSeq={} {} {} {} {}",
                        handName(),
                        flushSequence,
                        queuedSequence,
                        afterSolveSequence,
                        grab_transform_telemetry::formatBasisCrossMap("desiredToTargetConA", desiredBasis, targetConstraintABasis),
                        grab_transform_telemetry::formatBasisCrossMap("solverToTargetConA", solverEffectiveBasis, targetConstraintABasis),
                        grab_transform_telemetry::formatBasisCrossMap("desiredToLiveConA", desiredBasis, liveConstraintABasis),
                        grab_transform_telemetry::formatBasisCrossMap("solverLiveToLiveConA", solverEffectiveLiveABasis, liveConstraintABasis));
                }
            }
        }

        const bool likelyRawProxyFrameMismatch =
            rawToTargetProxyRotationDegrees > kGrabFrameMismatchRawProxyRotationWarnDegrees &&
            ((proxyOk && proxyTargetRotationErrorDegrees > kGrabFrameMismatchProxyRotationWarnDegrees) ||
                (objectOk && objectTargetRotationErrorDegrees > kGrabFrameMismatchObjectRotationWarnDegrees) ||
                (objectOk && gripTargetErrorGameUnits > kGrabFrameMismatchGripErrorWarnGameUnits));
        if (likelyRawProxyFrameMismatch) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} PROXY GRAB FRAME MISMATCH: seq={}/{} afterSeq={} substep={}/{} rawToProxyTarget={:.1f}deg rawToProxyLive={:.1f}deg targetAxisDeg=({:.1f},{:.1f},{:.1f}) liveAxisDeg=({:.1f},{:.1f},{:.1f}) determinantTarget=({:.3f},{:.3f}) determinantLive=({:.3f},{:.3f}) proxyErr={:.2f}gu/{:.1f}deg objectErr={:.2f}gu/{:.1f}deg gripErr={:.2f}gu pivotBRelationDelta={:.2f}gu pivotARoundTrip={:.3f}gu linTorque={:.3f}gu2 linTorqueDotReq={:.2f} targetToHiggsRelation={:.1f}deg transformBFrozenDelta={:.1f}deg probePivotBRelationDelta={:.3f}gu reqAxisProxy=({:.2f},{:.2f},{:.2f}) angularRef={} proxySrc={} objectSrc={} phase={} pivotB=({:.2f},{:.2f},{:.2f})",
                handName(),
                flushSequence,
                queuedSequence,
                afterSolveSequence,
                timing.substepIndex,
                timing.substepCount,
                rawToTargetProxyRotationDegrees,
                rawToLiveProxyRotationDegrees,
                targetPalmBasisDelta.xAxisDegrees,
                targetPalmBasisDelta.yAxisDegrees,
                targetPalmBasisDelta.zAxisDegrees,
                livePalmBasisDelta.xAxisDegrees,
                livePalmBasisDelta.yAxisDegrees,
                livePalmBasisDelta.zAxisDegrees,
                targetPalmBasisDelta.rawDeterminant,
                targetPalmBasisDelta.proxyDeterminant,
                livePalmBasisDelta.rawDeterminant,
                livePalmBasisDelta.proxyDeterminant,
                proxyTargetPositionErrorGameUnits,
                proxyTargetRotationErrorDegrees,
                objectTargetPositionErrorGameUnits,
                objectTargetRotationErrorDegrees,
                gripTargetErrorGameUnits,
                hasConstraintFrameMetrics ? pivotBRelationDeltaGameUnits : -1.0f,
                ragdollAngularProbePreSolve.transformAPivotRoundTripDeltaGameUnits,
                ragdollAngularProbePreSolve.linearTorqueWitnessGameUnitsSquared,
                ragdollAngularProbePreSolve.linearTorqueAxisDotRequired,
                hasConstraintFrameMetrics ? targetToHiggsRelationDegrees : -1.0f,
                hasConstraintFrameMetrics ? transformBFrozenDeltaDegrees : -1.0f,
                ragdollAngularProbePreSolve.pivotBRelationDeltaGameUnits,
                ragdollAngularProbePreSolve.requiredAxisProxyLocal.x,
                ragdollAngularProbePreSolve.requiredAxisProxyLocal.y,
                ragdollAngularProbePreSolve.requiredAxisProxyLocal.z,
                kGrabObjectRotationReferenceName,
                body_frame::bodyFrameSourceCode(proxySource),
                body_frame::bodyFrameSourceCode(objectSource),
                grab_three_phase::phaseName(_grabAcquisitionPhase),
                pivotBConstraintLocalGame.x,
                pivotBConstraintLocalGame.y,
                pivotBConstraintLocalGame.z);
        }

        const bool shouldLogSequence = afterSolveSequence <= 16 || (afterSolveSequence % 45u) == 0u;
        const bool shouldLogAnomaly =
            !proxyOk ||
            !objectOk ||
            proxyTargetPositionErrorGameUnits > 1.0f ||
            proxyTargetRotationErrorDegrees > 1.0f ||
            objectTargetPositionErrorGameUnits > 5.0f ||
            objectTargetRotationErrorDegrees > 15.0f ||
            gripTargetErrorGameUnits > 5.0f ||
            likelyRawProxyFrameMismatch;
        if (!debugGrabFrameLogging || (!shouldLogSequence && !shouldLogAnomaly)) {
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} PROXY GRAB AFTER_SOLVE: seq={}/{} afterSeq={} diag=bodyFrameConstraint+{} proxyBody={} objBody={} constraint={} substep={}/{} proxyRead={} proxySrc={} proxyMotion={} objectRead={} objectSrc={} objectMotion={} proxyTargetErr={:.3f}gu/{:.2f}deg objectTargetErr={:.2f}gu/{:.1f}deg objectLiveProxyErr={:.2f}gu/{:.1f}deg gripTargetErr={:.2f}gu gripLiveProxyErr={:.2f}gu angularRef={} targetBody=({:.1f},{:.1f},{:.1f}) targetProxy=({:.1f},{:.1f},{:.1f}) liveProxy=({:.1f},{:.1f},{:.1f}) targetObj=({:.1f},{:.1f},{:.1f}) liveObj=({:.1f},{:.1f},{:.1f}) desiredObjectTarget=({:.1f},{:.1f},{:.1f}) desiredObjectLiveProxy=({:.1f},{:.1f},{:.1f})",
            handName(),
            flushSequence,
            queuedSequence,
            afterSolveSequence,
            grabAngularAuthorityName(angularAuthority),
            proxyBodyId.value,
            objectBodyId.value,
            constraintId,
            timing.substepIndex,
            timing.substepCount,
            proxyOk ? "ok" : "fail",
            body_frame::bodyFrameSourceCode(proxySource),
            proxyMotionIndex,
            objectOk ? "ok" : "fail",
            body_frame::bodyFrameSourceCode(objectSource),
            objectMotionIndex,
            proxyTargetPositionErrorGameUnits,
            proxyTargetRotationErrorDegrees,
            objectTargetPositionErrorGameUnits,
            objectTargetRotationErrorDegrees,
            objectLiveProxyPositionErrorGameUnits,
            objectLiveProxyRotationErrorDegrees,
            gripTargetErrorGameUnits,
            gripLiveProxyErrorGameUnits,
            kGrabObjectRotationReferenceName,
            desiredBodyFromTarget.translate.x,
            desiredBodyFromTarget.translate.y,
            desiredBodyFromTarget.translate.z,
            targetProxyWorld.translate.x,
            targetProxyWorld.translate.y,
            targetProxyWorld.translate.z,
            proxyReadback.translate.x,
            proxyReadback.translate.y,
            proxyReadback.translate.z,
            desiredBodyFromTarget.translate.x,
            desiredBodyFromTarget.translate.y,
            desiredBodyFromTarget.translate.z,
            objectReadback.translate.x,
            objectReadback.translate.y,
            objectReadback.translate.z,
            desiredObjectFromTarget.translate.x,
            desiredObjectFromTarget.translate.y,
            desiredObjectFromTarget.translate.z,
            desiredObjectFromLiveProxy.translate.x,
            desiredObjectFromLiveProxy.translate.y,
            desiredObjectFromLiveProxy.translate.z);
    }

}

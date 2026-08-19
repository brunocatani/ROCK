#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabInternal.h"
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
#include "physics-interaction/grab/saved/SavedGrabOffsetStore.h"
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



















    void Hand::captureHeldReleaseMotion(
        RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime)
    {
        if (!isHolding() || !world) {
            return;
        }

        recordHeldControllerMotionSample(handWorldTransform, deltaTime);
        recordHeldObjectVelocitySample(world);
    }

    void Hand::applyReleaseVelocitySnapshot(RE::hknpWorld* world, const GrabReleaseOutcome::VelocitySnapshot& snapshot) const
    {
        if (!world || !snapshot.available) {
            return;
        }

        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(snapshot.bodyCount);
        const auto count = (std::min<std::uint32_t>)(snapshot.bodyCount, static_cast<std::uint32_t>(snapshot.bodyIds.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            bodyIds.push_back(snapshot.bodyIds[i]);
        }

        const auto releaseActivation = activateHeldObjectBodySet(world, snapshot.primaryBodyId.value, bodyIds);
        if (releaseActivation.failedActivationCount > 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand pending-transfer release activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                handName(),
                snapshot.primaryBodyId.value,
                releaseActivation.bodyCount,
                releaseActivation.activatedCount,
                releaseActivation.failedActivationCount);
        }

        setHeldVelocity(world,
            snapshot.primaryBodyId,
            bodyIds,
            snapshot.linearVelocityHavok,
            snapshot.angularVelocityRadiansPerSecond,
            snapshot.overrideAngularVelocity);
    }

    namespace
    {
        /*
         * ROCK owns the held object's rendered node while the render-clock
         * anchor is engaged. The engine's post-physics body-to-node sync runs
         * earlier in the frame; this later game-thread write is what the
         * renderer consumes, and the engine re-syncs from the body again next
         * frame, so releasing ownership is automatic. Writing local relative
         * to the live parent keeps the scene graph consistent; updateDown
         * propagates to children.
         */
    }

    void Hand::clearAllGrabRuntimeState()
    {
        // "ROCK_Grab" belongs only to the grab finger pose.
        (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
        clearGrabExternalHandWorldTransform(_isLeft);
        _preFrikGrabVisualAuthority.clear();
        clearSelectedCloseFingerPose();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _activeConstraint.clear();
        clearGrabAuthorityProxyRuntime();
        _heldBodyIds.clear();
        _heldBodyIdsCount.store(0, std::memory_order_release);
        _grabFrame.clear();
        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        _grabObjectGripAtGrab = {};
        _heldDriveDecision = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        _grabDeviationExceededSeconds = 0.0f;
        _grabDeviationHistory = {};
        _grabDeviationHistoryCount = 0;
        _grabDeviationHistoryNext = 0;
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _lastPublishedGrabVisualHandTransform = {};
        _hasLastPublishedGrabVisualHandTransform = false;
        _grabVisualHandLerpStartTransform = {};
        _grabVisualHandLerpElapsedSeconds = 0.0f;
        _grabVisualHandLerpDurationSeconds = 0.0f;
        _grabVisualDeviationExceededSeconds = 0.0f;
        _grabVisualDeviationHistory = {};
        _grabVisualDeviationHistoryCount = 0;
        _grabVisualDeviationHistoryNext = 0;
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerSweepDebugCapture = {};
        _grabFingerSweepDebugObjectWorld = {};
        _hasGrabFingerSweepDebug = false;
        _grabFingerPadProbeStart = {};
        _grabFingerPadProbeEnd = {};
        _grabFingerPadProbeHit = {};
        _grabFingerPadProbeHitValid = {};
        _hasGrabFingerPadProbeDebug = false;
        _grabFingerSurfaceTarget = {};
        _grabFingerSurfaceTargetValid = {};
        _hasGrabFingerSurfaceTargetDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _hasGrabFingerPose = false;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _heldLocalHandVelocityHistory = {};
        _heldHandAngularVelocityHistory = {};
        _heldHandVelocityHistoryCount = 0;
        _heldHandVelocityHistoryNext = 0;
        _lastHeldObjectLocalLinearVelocityHavok = {};
        _hasLastHeldObjectLocalLinearVelocityHavok = false;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
        _currentSelection.clear();
        clearGrabAcquisitionCache("grab-runtime-cleared");
    }

    GrabReleaseOutcome Hand::releaseGrabbedObject(
        RE::hknpWorld* world,
        GrabReleaseCollisionRestoreMode collisionRestoreMode,
        const GrabReleaseContext& releaseContext)
    {
        GrabReleaseOutcome outcome{};
        outcome.finalObjectRelease = releaseContext.finalObjectRelease;
        if (!isHolding()) {
            return outcome;
        }

        outcome.released = true;
        outcome.retainedRef = _savedObjectState.retainedRef;
        outcome.refr = outcome.retainedRef.get();
        outcome.formID = outcome.refr ? outcome.refr->GetFormID() : 0;

        /*
         * Drop the render-pose masquerade WITHOUT restoring: the body sits at
         * the last rendered anchor pose, which is where the player saw the
         * object; restoring the solver pose here would snap the release back
         * by the masquerade delta.
         */
        held_body_render_pose::clearWithoutRestore(_isLeft);
        scene_writer_probe::clearHeldTarget(_isLeft);
        _sceneWriterProbeRegisteredTraceId = 0;

        if (grabTimelineTraceEnabled()) {
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=release trace={} hand={} formID={:08X} body={} proxyBody={} constraint={} queued={} flushed={} afterSeq={} writes={} finalObjectRelease={} disposition={} reason={} heldBodies={} looseWeapon={}",
                handName(),
                _grabFrame.traceId,
                _isLeft ? "left" : "right",
                outcome.formID,
                _savedObjectState.bodyId.value,
                _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                _grabAuthorityProxyQueuedSequence,
                _grabAuthorityProxyFlushSequence,
                _grabAuthorityProxyAfterSolveLogCounter,
                _grabFrame.traceTargetWriteSequence,
                releaseContext.finalObjectRelease ? "yes" : "no",
                releaseDispositionName(releaseContext.disposition),
                releaseContext.reason ? releaseContext.reason : "none",
                _heldBodyIds.size(),
                _heldObjectIsLooseWeapon ? "yes" : "no");
        }

        ROCK_LOG_INFO(Hand,
            "{} hand RELEASE: bodyId={} constraintId={} proxyBody={} finalObjectRelease={} disposition={} reason={}",
            handName(),
            _savedObjectState.bodyId.value,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            releaseContext.finalObjectRelease ? "yes" : "no",
            releaseDispositionName(releaseContext.disposition),
            releaseContext.reason ? releaseContext.reason : "none");

        nearby_grab_damping::restoreNearbyGrabDamping(world, _nearbyGrabDamping);

        const bool captureReleaseVelocity =
            (releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop ||
                releaseContext.disposition == GrabReleaseDisposition::PendingInventoryTransfer ||
                releaseContext.disposition == GrabReleaseDisposition::PendingConsumeTransfer) &&
            releaseContext.applyCapturedReleaseVelocity &&
            releaseContext.finalObjectRelease && world && (_heldLocalLinearVelocityHistoryCount > 0 || _heldHandVelocityHistoryCount > 0);
        if (captureReleaseVelocity) {
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedObjectHistory{};
            const std::size_t historySize = _heldLocalLinearVelocityHistory.size();
            const std::size_t firstIndex = (_heldLocalLinearVelocityHistoryNext + historySize - _heldLocalLinearVelocityHistoryCount) % historySize;
            for (std::size_t i = 0; i < _heldLocalLinearVelocityHistoryCount; ++i) {
                orderedObjectHistory[i] = _heldLocalLinearVelocityHistory[(firstIndex + i) % historySize];
            }

            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedHandHistory{};
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedAngularHistory{};
            const std::size_t handHistorySize = _heldLocalHandVelocityHistory.size();
            const std::size_t firstHandIndex = (_heldHandVelocityHistoryNext + handHistorySize - _heldHandVelocityHistoryCount) % handHistorySize;
            for (std::size_t i = 0; i < _heldHandVelocityHistoryCount; ++i) {
                const std::size_t sourceIndex = (firstHandIndex + i) % handHistorySize;
                orderedHandHistory[i] = _heldLocalHandVelocityHistory[sourceIndex];
                orderedAngularHistory[i] = _heldHandAngularVelocityHistory[sourceIndex];
            }

            const RE::NiPoint3 objectLocalReleaseVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedObjectHistory, _heldLocalLinearVelocityHistoryCount);
            const RE::NiPoint3 handLocalReleaseVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedHandHistory, _heldHandVelocityHistoryCount);
            const RE::NiPoint3 handAngularVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedAngularHistory, _heldHandVelocityHistoryCount);

            RE::NiPoint3 tangentialVelocityHavok{};
            bool hasTangentialVelocity = false;
            RE::NiPoint3 releaseLeverOriginHavok = _lastHeldHandPositionHavok;
            bool hasReleaseLeverOrigin = _hasLastHeldHandPositionHavok;
            RE::NiPoint3 releaseCenterOfMassHavok{};
            bool hasReleaseCenterOfMass = false;
            RE::NiTransform releaseBodyWorld{};
            bool hasReleaseBodyWorld = false;
            if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, releaseBodyWorld)) {
                hasReleaseBodyWorld = true;
                releaseLeverOriginHavok =
                    gamePointToHavokPoint(transform_math::localPointToWorld(releaseBodyWorld, activeProxyConstraintPivotBLocalGame()));
                hasReleaseLeverOrigin = true;
            }
            if (hasReleaseLeverOrigin && lengthSquared(handAngularVelocity) > 0.000001f) {
                float comX = 0.0f;
                float comY = 0.0f;
                float comZ = 0.0f;
                if (havok_runtime::getBodyCOMWorld(world, _savedObjectState.bodyId, comX, comY, comZ)) {
                    releaseCenterOfMassHavok = RE::NiPoint3{ comX, comY, comZ };
                    hasReleaseCenterOfMass = true;
                    tangentialVelocityHavok = grab_held_response::computeTangentialVelocityFromAngularSwing(
                        handAngularVelocity,
                        releaseLeverOriginHavok,
                        releaseCenterOfMassHavok);
                    hasTangentialVelocity = lengthSquared(tangentialVelocityHavok) > 0.000001f;
                }
            }

            const RE::NiPoint3 releaseVelocity =
                grab_held_response::composeControllerReleaseVelocity(grab_held_response::ReleaseVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandLocalVelocity = _heldHandVelocityHistoryCount > 0,
                    .hasObjectLocalVelocity = _heldLocalLinearVelocityHistoryCount > 0,
                    .hasTangentialVelocity = hasTangentialVelocity,
                    .handLocalVelocityHavok = handLocalReleaseVelocity,
                    .objectLocalVelocityHavok = objectLocalReleaseVelocity,
                    .tangentialVelocityHavok = tangentialVelocityHavok,
                    .objectVelocityBlend = g_rockConfig.rockGrabThrowObjectVelocityBlend,
                    .tangentialVelocityScale = g_rockConfig.rockGrabThrowTangentialVelocityScale,
                    .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                    .maxVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                });
            const RE::NiPoint3 rawReleaseAngularVelocity =
                grab_held_response::composeControllerReleaseAngularVelocity(grab_held_response::ReleaseAngularVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandAngularVelocity = _heldHandVelocityHistoryCount > 0,
                    .handAngularVelocityRadiansPerSecond = handAngularVelocity,
                    .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                    .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                });
            RE::NiPoint3 releaseAngularVelocity = rawReleaseAngularVelocity;
            const auto releaseContactSnapshot = readHeldBodyContactSnapshot();
            const bool releaseContactSoftening =
                releaseContactSnapshot.recent &&
                classifyHeldContactOtherMotion(world, releaseContactSnapshot.otherBodyId) != held_object_contact_policy::HeldContactOtherMotion::Dynamic;
            const auto releaseAuthority = evaluateRuntimeHeldAuthority(
                _grabFrame,
                releaseContactSoftening);
            const auto& angularAuthority = releaseAuthority.angular;
            const float releaseLongObjectAngularScale = grab_motion_controller::computeLongObjectAngularSpeedScale(
                g_rockConfig.rockGrabLongObjectAngularScalingEnabled,
                _grabFrame.longObjectLeverGameUnits,
                g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
                g_rockConfig.rockGrabLongObjectMinAngularScale);
            const float releaseAngularVelocityCap = grab_motion_controller::computeAuthorityScaledAngularVelocityCap(
                g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                releaseAuthority.releaseAngularVelocityScale,
                releaseLongObjectAngularScale);
            if ((angularAuthority.axisLimited || angularAuthority.weakPivotTwistScale < 0.999f) && hasReleaseLeverOrigin) {
                if (!hasReleaseCenterOfMass) {
                    float comX = 0.0f;
                    float comY = 0.0f;
                    float comZ = 0.0f;
                    if (havok_runtime::getBodyCOMWorld(world, _savedObjectState.bodyId, comX, comY, comZ)) {
                        releaseCenterOfMassHavok = RE::NiPoint3{ comX, comY, comZ };
                        hasReleaseCenterOfMass = true;
                    }
                }
                if (hasReleaseCenterOfMass) {
                    RE::NiPoint3 releaseContactNormalWorld{};
                    if (_grabFrame.pivotAuthorityNormalTrusted && hasReleaseBodyWorld) {
                        const RE::NiTransform releaseNodeWorld =
                            _grabFrame.heldNode ? _grabFrame.heldNode->world : heldNodeWorldFromBodyWorld(releaseBodyWorld);
                        releaseContactNormalWorld = gripEvidenceNormalWorld(_grabFrame, releaseNodeWorld);
                    }
                    releaseAngularVelocity = grab_motion_controller::scaleAngularVelocityByHeldAuthorityAxes(
                        rawReleaseAngularVelocity,
                        releaseCenterOfMassHavok - releaseLeverOriginHavok,
                        releaseContactNormalWorld,
                        angularAuthority);
                }
            }
            releaseAngularVelocity = clampAngularVelocityVector(releaseAngularVelocity, releaseAngularVelocityCap);
            const bool overrideAngularVelocity =
                g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled && lengthSquared(releaseAngularVelocity) > 0.000001f;
            outcome.velocity.available = true;
            outcome.velocity.primaryBodyId = _savedObjectState.bodyId;
            outcome.velocity.linearVelocityHavok = releaseVelocity;
            outcome.velocity.angularVelocityRadiansPerSecond = releaseAngularVelocity;
            outcome.velocity.overrideAngularVelocity = overrideAngularVelocity;
            if (_heldDriveDecision.includeConnectedLinearVelocity || _heldDriveDecision.includeConnectedAngularVelocity) {
                for (const auto bodyId : _heldBodyIds) {
                    if (outcome.velocity.bodyCount >= outcome.velocity.bodyIds.size()) {
                        break;
                    }
                    outcome.velocity.bodyIds[outcome.velocity.bodyCount++] = bodyId;
                }
            }

            const bool applyReleaseVelocity = releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop;
            if (applyReleaseVelocity) {
                setHeldVelocity(
                    world,
                    _savedObjectState.bodyId,
                    _heldBodyIds,
                    releaseVelocity,
                    releaseAngularVelocity,
                    overrideAngularVelocity,
                    1.0f,
                    _heldDriveDecision.includeConnectedLinearVelocity,
                    _heldDriveDecision.includeConnectedAngularVelocity);
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand RELEASE VELOCITY: applied={} driveMode={} linearScope={} angularScope={} authority={} shape={} angularScale={:.2f} angularCap={:.3f} longScale={:.2f} handLocal=({:.3f},{:.3f},{:.3f}) objectLocal=({:.3f},{:.3f},{:.3f}) tangent=({:.3f},{:.3f},{:.3f}) angularRaw=({:.3f},{:.3f},{:.3f}) angularFinal=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) lever=({:.3f},{:.3f},{:.3f}) objectHistory={} handHistory={} multiplier={:.2f}",
                handName(),
                applyReleaseVelocity ? "yes" : "no",
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                releaseAuthority.reason,
                grab_motion_controller::contactSupportShapeName(angularAuthority.contactSupportShape),
                releaseAuthority.releaseAngularVelocityScale,
                releaseAngularVelocityCap,
                releaseLongObjectAngularScale,
                handLocalReleaseVelocity.x,
                handLocalReleaseVelocity.y,
                handLocalReleaseVelocity.z,
                objectLocalReleaseVelocity.x,
                objectLocalReleaseVelocity.y,
                objectLocalReleaseVelocity.z,
                tangentialVelocityHavok.x,
                tangentialVelocityHavok.y,
                tangentialVelocityHavok.z,
                handAngularVelocity.x,
                handAngularVelocity.y,
                handAngularVelocity.z,
                releaseAngularVelocity.x,
                releaseAngularVelocity.y,
                releaseAngularVelocity.z,
                releaseVelocity.x,
                releaseVelocity.y,
                releaseVelocity.z,
                releaseLeverOriginHavok.x,
                releaseLeverOriginHavok.y,
                releaseLeverOriginHavok.z,
                _heldLocalLinearVelocityHistoryCount,
                _heldHandVelocityHistoryCount,
                g_rockConfig.rockThrowVelocityMultiplier);
        }

        if (releaseContext.finalObjectRelease) {
            restoreGrabbedInertia(world, _savedObjectState);
        }

        if (releaseContext.finalObjectRelease && world && _activeGrabLifecycle.size() > 0) {
            const auto releaseRestorePolicy =
                active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(_savedObjectState.targetKind);
            const auto releaseIntent = releaseIntentFromDisposition(releaseContext.disposition);
            const auto releasePlan = _activeGrabLifecycle.restorePlanForRelease(
                releaseRestorePolicy,
                _savedObjectState.targetKind,
                releaseIntent);
            restoreActiveGrabLifecycle(world,
                _activeGrabLifecycle,
                releasePlan,
                _savedObjectState.bodyId.value,
                handName(),
                "release");
            if (_activeGrabLifecycle.hasIncompleteNativeScan()) {
                auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;
                if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan, _savedObjectState.originalMotionPropsId)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand release: skipped recursive root restore for converted loose-object physical drop root='{}' motionProps={} preservedMotion={}",
                        handName(),
                        nodeDebugName(rootNode),
                        _savedObjectState.originalMotionPropsId,
                        releasePlan.preservedConvertedMotionCount);
                } else {
                    restoreIncompleteActivePrepRoot(rootNode, _savedObjectState.originalMotionPropsId, handName(), "release-incomplete-scan");
                }
            }
        }

        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        clearHeldBodyContactSnapshot();

        if (world) {
            const auto heldFlagReleases = releaseHeldObjectBodyFlagLeases(
                world,
                _savedObjectState.bodyId.value,
                _heldBodyIds,
                heldBodyFlagLeaseOwner(this),
                releaseContext.finalObjectRelease);
            if (heldFlagReleases.failedLeaseCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand RELEASE held body flag release incomplete: primaryBody={} bodies={} collision={} authority={} failed={} finalObjectRelease={}",
                    handName(),
                    _savedObjectState.bodyId.value,
                    heldFlagReleases.bodyCount,
                    heldFlagReleases.collisionLeaseCount,
                    heldFlagReleases.authorityLeaseCount,
                    heldFlagReleases.failedLeaseCount,
                    releaseContext.finalObjectRelease ? "yes" : "no");
            } else {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand RELEASE held body flag leases released: primaryBody={} bodies={} collision={} authority={} finalObjectRelease={}",
                    handName(),
                    _savedObjectState.bodyId.value,
                    heldFlagReleases.bodyCount,
                    heldFlagReleases.collisionLeaseCount,
                    heldFlagReleases.authorityLeaseCount,
                    releaseContext.finalObjectRelease ? "yes" : "no");
            }

            if (releaseContext.finalObjectRelease && releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop) {
                const auto releaseActivation = activateHeldObjectBodySet(world, _savedObjectState.bodyId.value, _heldBodyIds);
                if (releaseActivation.failedActivationCount > 0) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand RELEASE activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                        handName(),
                        _savedObjectState.bodyId.value,
                        releaseActivation.bodyCount,
                        releaseActivation.activatedCount,
                        releaseActivation.failedActivationCount);
                }
            }
        }

        {
            // Guard both proxy-owned handles during final constraint teardown.
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            if (_activeConstraint.isValid()) {
                destroyGrabConstraint(world, _activeConstraint);
            }
            destroyGrabAuthorityProxyLocked(nullptr);
        }

        const bool delayRestore = collisionRestoreMode == GrabReleaseCollisionRestoreMode::Delayed &&
                                  hand_collision_suppression_math::beginDelayedRestore(
                                      _grabHandCollisionDelayedRestore, _grabHandCollisionSuppression, g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        restoreBodyCollisionAfterHeldLooseWeapon(world);
        if (delayRestore) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand: grab hand collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                handName(),
                _grabHandCollisionDelayedRestore.bodyCount,
                _grabHandCollisionDelayedRestore.bodyId,
                _grabHandCollisionDelayedRestore.remainingSeconds);
        } else {
            restoreHandCollisionAfterGrab(world);
        }

        beginGrabVisualReturn();
        clearAllGrabRuntimeState();
        HandInteractionEvent releaseEvent = HandInteractionEvent::ReleaseRequested;
        if (releaseContext.disposition == GrabReleaseDisposition::TransferToInventory && _state == HandState::StashCandidate) {
            releaseEvent = HandInteractionEvent::CommitStash;
        }
        applyTransition(HandTransitionRequest{ .event = releaseEvent });

        ROCK_LOG_DEBUG(Hand, "{} hand: Idle", handName());
        return outcome;
    }
}

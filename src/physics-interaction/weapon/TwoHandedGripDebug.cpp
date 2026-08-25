#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace rock
{
    using namespace two_handed_grip_internal;

    void TwoHandedGrip::traceNativeScopeTransitionFinalState(RE::NiNode* weaponNode)
    {
        if (!_nativeScopeTransitionFinalTracePending) {
            return;
        }
        _nativeScopeTransitionFinalTracePending = false;

        struct HmdRelativeTrace
        {
            RE::NiPoint3 position{};
            bool valid{ false };
        };

        const auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform hmdWorld{};
        const bool hmdWorldValid =
            playerNodes && playerNodes->HmdNode &&
            isFiniteTransform(playerNodes->HmdNode->world);
        if (hmdWorldValid) {
            hmdWorld = playerNodes->HmdNode->world;
        }

        const auto captureWorld = [&hmdWorld, hmdWorldValid](
                                      const RE::NiTransform& world,
                                      const bool worldValid) {
            HmdRelativeTrace trace{};
            if (!hmdWorldValid || !worldValid) {
                return trace;
            }
            trace.position = transform_math::worldPointToLocal(
                hmdWorld,
                world.translate);
            trace.valid = std::isfinite(trace.position.x) &&
                          std::isfinite(trace.position.y) &&
                          std::isfinite(trace.position.z);
            return trace;
        };
        const auto captureNode = [&captureWorld](const RE::NiAVObject* node) {
            return captureWorld(
                node ? node->world : RE::NiTransform{},
                node && isFiniteTransform(node->world));
        };

        RE::NiTransform scopeCameraWorld{};
        bool scopeCameraWorldValid = false;
        if (playerNodes && playerNodes->primaryWeaponScopeCamera) {
            const auto* scopeCamera = playerNodes->primaryWeaponScopeCamera;
            scopeCameraWorld = scopeCamera->world;
            if (scopeCamera->parent &&
                isFiniteTransform(scopeCamera->parent->world) &&
                isFiniteTransform(scopeCamera->local)) {
                scopeCameraWorld = transform_math::composeTransforms(
                    scopeCamera->parent->world,
                    scopeCamera->local);
            }
            scopeCameraWorldValid = isFiniteTransform(scopeCameraWorld);
        }

        RE::NiTransform leftRootWorld{};
        RE::NiTransform rightRootWorld{};
        const bool leftRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(true, leftRootWorld);
        const bool rightRootWorldValid =
            tryGetRootFlattenedHandBoneTransform(false, rightRootWorld);
        const auto* playerCamera = f4vr::getPlayerCamera();
        const bool cameraValuesValid =
            playerCamera &&
            std::isfinite(playerCamera->zoomInput) &&
            std::isfinite(playerCamera->worldFOV) &&
            std::isfinite(playerCamera->firstPersonFOV) &&
            std::isfinite(playerCamera->fovAdjustCurrent) &&
            std::isfinite(playerCamera->fovAdjustTarget) &&
            std::isfinite(playerCamera->fovAdjustPerSec) &&
            std::isfinite(playerCamera->fovAnimatorAdjust);

        const HmdRelativeTrace weaponTrace = captureWorld(
            weaponNode ? weaponNode->world : RE::NiTransform{},
            weaponNode && isFiniteTransform(weaponNode->world));
        const HmdRelativeTrace leftRootTrace = captureWorld(
            leftRootWorld,
            leftRootWorldValid);
        const HmdRelativeTrace rightRootTrace = captureWorld(
            rightRootWorld,
            rightRootWorldValid);
        const HmdRelativeTrace scopeCameraTrace = captureWorld(
            scopeCameraWorld,
            scopeCameraWorldValid);
        const HmdRelativeTrace scopeParentTrace = captureNode(
            playerNodes ? playerNodes->ScopeParentNode : nullptr);
        const HmdRelativeTrace cameraRootTrace = captureNode(
            playerCamera ? playerCamera->cameraRoot.get() : nullptr);
        const HmdRelativeTrace roomTrace = captureNode(
            playerNodes ? playerNodes->roomnode : nullptr);
        const HmdRelativeTrace uprightHmdTrace = captureNode(
            playerNodes ? playerNodes->UprightHmdNode : nullptr);
        const HmdRelativeTrace skeletonRootTrace =
            captureNode(f4vr::getRootNode());

        ROCK_LOG_INFO(Weapon,
            "SCOPE-VIEW-FINAL seq={} sample={}/{} buttonRequested={} rendererActive={} menuOpen={} driverAuthority={} state={} hmdValid={} hmdWorld=({:.2f},{:.2f},{:.2f}) weaponHmdValid={} weaponHmd=({:.2f},{:.2f},{:.2f}) leftRootHmdValid={} leftRootHmd=({:.2f},{:.2f},{:.2f}) rightRootHmdValid={} rightRootHmd=({:.2f},{:.2f},{:.2f}) scopeCameraHmdValid={} scopeCameraHmd=({:.2f},{:.2f},{:.2f}) scopeParentHmdValid={} scopeParentHmd=({:.2f},{:.2f},{:.2f}) cameraRootHmdValid={} cameraRootHmd=({:.2f},{:.2f},{:.2f}) roomHmdValid={} roomHmd=({:.2f},{:.2f},{:.2f}) uprightHmdValid={} uprightHmd=({:.2f},{:.2f},{:.2f}) skeletonRootHmdValid={} skeletonRootHmd=({:.2f},{:.2f},{:.2f}) cameraValuesValid={} zoomInput={:.4f} worldFov={:.4f} firstPersonFov={:.4f} fovAdjust=({:.4f},{:.4f},{:.4f},{:.4f})",
            _nativeScopeTransitionFinalTraceSequence,
            _nativeScopeTransitionFinalTraceSample,
            SCOPE_TRANSITION_TRACE_FRAMES,
            _manualScopeActivationRequested ? "yes" : "no",
            _nativeScopeRequestActive ? "yes" : "no",
            _scopeMenuOpenThisFrame ? "yes" : "no",
            _scopeDriverFrameAuthorityActive ? "yes" : "no",
            static_cast<std::uint32_t>(_state),
            hmdWorldValid ? "yes" : "no",
            hmdWorld.translate.x,
            hmdWorld.translate.y,
            hmdWorld.translate.z,
            weaponTrace.valid ? "yes" : "no",
            weaponTrace.position.x,
            weaponTrace.position.y,
            weaponTrace.position.z,
            leftRootTrace.valid ? "yes" : "no",
            leftRootTrace.position.x,
            leftRootTrace.position.y,
            leftRootTrace.position.z,
            rightRootTrace.valid ? "yes" : "no",
            rightRootTrace.position.x,
            rightRootTrace.position.y,
            rightRootTrace.position.z,
            scopeCameraTrace.valid ? "yes" : "no",
            scopeCameraTrace.position.x,
            scopeCameraTrace.position.y,
            scopeCameraTrace.position.z,
            scopeParentTrace.valid ? "yes" : "no",
            scopeParentTrace.position.x,
            scopeParentTrace.position.y,
            scopeParentTrace.position.z,
            cameraRootTrace.valid ? "yes" : "no",
            cameraRootTrace.position.x,
            cameraRootTrace.position.y,
            cameraRootTrace.position.z,
            roomTrace.valid ? "yes" : "no",
            roomTrace.position.x,
            roomTrace.position.y,
            roomTrace.position.z,
            uprightHmdTrace.valid ? "yes" : "no",
            uprightHmdTrace.position.x,
            uprightHmdTrace.position.y,
            uprightHmdTrace.position.z,
            skeletonRootTrace.valid ? "yes" : "no",
            skeletonRootTrace.position.x,
            skeletonRootTrace.position.y,
            skeletonRootTrace.position.z,
            cameraValuesValid ? "yes" : "no",
            cameraValuesValid ? playerCamera->zoomInput : 0.0f,
            cameraValuesValid ? playerCamera->worldFOV : 0.0f,
            cameraValuesValid ? playerCamera->firstPersonFOV : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustCurrent : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustTarget : 0.0f,
            cameraValuesValid ? playerCamera->fovAdjustPerSec : 0.0f,
            cameraValuesValid ? playerCamera->fovAnimatorAdjust : 0.0f);
    }

    void TwoHandedGrip::reframeAuthoredSupportGripDebugSnapshot(
        const RE::NiTransform& finalWeaponWorld)
    {
        auto& snapshot = _authoredSupportGripDebugSnapshot;
        if (!snapshot.valid ||
            !isFiniteTransform(snapshot.weaponWorld) ||
            !isFiniteTransform(finalWeaponWorld)) {
            return;
        }

        const RE::NiTransform previousWeaponWorld = snapshot.weaponWorld;
        const auto reframePoint = [&](const RE::NiPoint3& pointWorld) {
            return transform_math::localPointToWorld(
                finalWeaponWorld,
                transform_math::worldPointToLocal(
                    previousWeaponWorld,
                    pointWorld));
        };
        const auto reframeDirection = [&](const RE::NiPoint3& directionWorld) {
            RE::NiPoint3 reframed{};
            (void)gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    finalWeaponWorld,
                    transform_math::worldVectorToLocal(
                        previousWeaponWorld,
                        directionWorld)),
                reframed);
            return reframed;
        };

        snapshot.authoredPalmSeatWorld =
            transform_math::localPointToWorld(
                finalWeaponWorld,
                snapshot.authoredPalmSeatWeaponLocal);
        snapshot.leftAxisWorld = reframeDirection(snapshot.leftAxisWorld);
        snapshot.downAxisWorld = reframeDirection(snapshot.downAxisWorld);
        snapshot.referenceAxisWorld =
            reframeDirection(snapshot.referenceAxisWorld);
        for (auto& landmark : snapshot.poseLandmarksWorld) {
            landmark = reframePoint(landmark);
        }
        for (std::size_t index = 0;
             index < snapshot.poseSurfaceWitnessWorld.size();
             ++index) {
            if ((snapshot.poseSurfaceWitnessMask &
                    static_cast<std::uint8_t>(1u << index)) != 0) {
                snapshot.poseSurfaceWitnessWorld[index] =
                    reframePoint(snapshot.poseSurfaceWitnessWorld[index]);
            }
        }

        snapshot.liveTouchProbeWeaponLocal =
            transform_math::worldPointToLocal(
                finalWeaponWorld,
                snapshot.liveTouchProbeWorld);
        const RE::NiPoint3 approach{
            snapshot.liveTouchProbeWorld.x -
                snapshot.authoredPalmSeatWorld.x,
            snapshot.liveTouchProbeWorld.y -
                snapshot.authoredPalmSeatWorld.y,
            snapshot.liveTouchProbeWorld.z -
                snapshot.authoredPalmSeatWorld.z,
        };
        snapshot.approachDirectionWorld = {};
        const bool approachValid =
            gunstock_alignment_policy::tryNormalizeDirection(
                approach,
                snapshot.approachDirectionWorld);
        const RE::NiPoint3 weaponLocalSeparation{
            snapshot.liveTouchProbeWeaponLocal.x -
                snapshot.authoredPalmSeatWeaponLocal.x,
            snapshot.liveTouchProbeWeaponLocal.y -
                snapshot.authoredPalmSeatWeaponLocal.y,
            snapshot.liveTouchProbeWeaponLocal.z -
                snapshot.authoredPalmSeatWeaponLocal.z,
        };
        snapshot.weaponRelativeDistanceGameUnits =
            std::sqrt(dot(
                weaponLocalSeparation,
                weaponLocalSeparation)) *
            std::abs(finalWeaponWorld.scale);
        snapshot.worldReadbackDistanceGameUnits =
            std::sqrt(dot(approach, approach));
        snapshot.frameAgreementErrorGameUnits = std::abs(
            snapshot.weaponRelativeDistanceGameUnits -
            snapshot.worldReadbackDistanceGameUnits);
        snapshot.insideTouchRadius =
            snapshot.weaponRelativeDistanceGameUnits <=
            snapshot.touchRadiusGameUnits;

        using ActivationVec3 =
            authored_weapon_grip_activation_policy::Vec3;
        const auto toActivationVector = [](const RE::NiPoint3& value) {
            return ActivationVec3{ value.x, value.y, value.z };
        };
        const auto gate =
            authored_weapon_grip_activation_policy::evaluateDirectionGate(
                authored_weapon_grip_activation_policy::DirectionGateInput{
                    .weaponFamily = snapshot.weaponFamily,
                    .authoredSeatWorld = toActivationVector(
                        snapshot.authoredPalmSeatWorld),
                    .liveProbeWorld = toActivationVector(
                        snapshot.liveTouchProbeWorld),
                    .leftAxisWorld = toActivationVector(
                        snapshot.leftAxisWorld),
                    .downAxisWorld = toActivationVector(
                        snapshot.downAxisWorld),
                    .lastStableDirectionWorld = toActivationVector(
                        snapshot.approachDirectionWorld),
                    .radialCapGameUnits = snapshot.radialCapGameUnits,
                    .lastStableDirectionValid = approachValid,
                    .rightFiringLeftSupportScope =
                        snapshot.canonicalAxesValid &&
                        !_firingHandIsLeft && snapshot.supportHandIsLeft,
                });
        snapshot.approachDirectionWorld = RE::NiPoint3{
            gate.approachDirectionWorld.x,
            gate.approachDirectionWorld.y,
            gate.approachDirectionWorld.z,
        };
        snapshot.leftDot = gate.leftDot;
        snapshot.downDot = gate.downDot;
        snapshot.selectedCone = gate.selectedCone;
        snapshot.classifierSupported = gate.familySupported;
        snapshot.directionUsedLastStableSample =
            gate.usedLastStableDirection;
        snapshot.radialPass = gate.radialPass;
        snapshot.directionPass = gate.directionPass;
        snapshot.scopePass = gate.scopePass;
        snapshot.activationSpatialPass = gate.spatialPass;
        snapshot.weaponWorld = finalWeaponWorld;
    }

    bool TwoHandedGrip::populateGunstockAlignmentDebugPrediction(
        const RE::NiPoint3& neutralHandLocal)
    {
        auto& snapshot = _gunstockAlignmentDebugSnapshot;
        snapshot.correctionValid = false;
        snapshot.correctionAxisValid = false;
        snapshot.correctionUsedAntiparallelFallback = false;
        snapshot.predictionValid = false;
        snapshot.correctionWorld = {};
        snapshot.predictedWeaponWorld = {};
        snapshot.predictedFireNodeWorld = {};
        snapshot.wristForwardWorld = {};
        snapshot.fineTunedTargetForwardWorld = {};
        snapshot.weaponRootForwardWorld = {};
        snapshot.unalignedLiveFireWorld = {};
        snapshot.neutralFireWorldBefore = {};
        snapshot.predictedNeutralFireWorld = {};
        snapshot.correctionAxisWorld = {};
        snapshot.unalignedAngleDegrees = 0.0f;
        snapshot.correctionAngleRadians = 0.0f;
        snapshot.correctionAngleDegrees = 0.0f;
        const auto fineTune = configuredGunstockFineTune();
        snapshot.fineTunePitchDegrees = fineTune.pitchDegrees;
        snapshot.fineTuneYawDegrees = fineTune.yawDegrees;
        snapshot.fineTuneRollDegrees = fineTune.rollDegrees;
        snapshot.fineTuneTargetOffsetDegrees = 0.0f;
        snapshot.fineTuneActive =
            gunstock_alignment_policy::hasFineTune(fineTune);
        snapshot.neutralResidualDegrees = 0.0f;

        if (!snapshot.firingHandValid ||
            !snapshot.weaponBeforeValid ||
            !snapshot.fireNodeBeforeValid) {
            return false;
        }

        const RE::NiPoint3 localForward{ 0.0f, 1.0f, 0.0f };
        const RE::NiPoint3 localWristForward{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 wristForward{};
        RE::NiPoint3 weaponRootForward{};
        RE::NiPoint3 unalignedLiveFire{};
        RE::NiPoint3 neutralLocal{};
        RE::NiPoint3 neutralFireWorld{};
        if (!gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.firingHandWorld,
                    localWristForward),
                wristForward) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.weaponWorldBefore,
                    localForward),
                weaponRootForward) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.fireNodeWorldBefore,
                    localForward),
                unalignedLiveFire) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                neutralHandLocal,
                neutralLocal) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.firingHandWorld,
                    neutralLocal),
                neutralFireWorld)) {
            return false;
        }

        float directionDot = 1.0f;
        RE::NiMatrix3 correction{};
        RE::NiPoint3 fineTunedTarget{};
        if (!gunstock_alignment_policy::tryBuildFineTunedWorldCorrection<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                snapshot.firingHandWorld,
                neutralLocal,
                wristForward,
                fineTune,
                correction,
                &fineTunedTarget,
                &directionDot)) {
            return false;
        }

        const RE::NiTransform predictedWeapon =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                snapshot.weaponWorldBefore,
                correction,
                snapshot.pivotWorld);
        const RE::NiTransform predictedFire =
            gunstock_alignment_policy::rotateRigidlyAroundPivot<
                RE::NiTransform,
                RE::NiMatrix3,
                RE::NiPoint3>(
                snapshot.fireNodeWorldBefore,
                correction,
                snapshot.pivotWorld);
        RE::NiPoint3 predictedNeutral{};
        if (!isFiniteTransform(predictedWeapon) ||
            !isFiniteTransform(predictedFire) ||
            !gunstock_alignment_policy::tryNormalizeDirection(
                weaponSolverApplyStoredWorldRotationToVector<
                    RE::NiMatrix3,
                    RE::NiPoint3>(
                    correction,
                    neutralFireWorld),
                predictedNeutral)) {
            return false;
        }

        const float clampedDot = (std::max)(
            -1.0f,
            (std::min)(1.0f, directionDot));
        const float correctionAngle = std::acos(clampedDot);
        RE::NiPoint3 correctionAxis =
            weaponSolverCross(neutralFireWorld, wristForward);
        if (weaponSolverLength(correctionAxis) >
            gunstock_alignment_policy::kMinimumDirectionLength) {
            correctionAxis = weaponSolverNormalize(correctionAxis);
            snapshot.correctionAxisValid = true;
        } else if (clampedDot < -0.9999f) {
            correctionAxis = weaponSolverOrthogonalAxis(neutralFireWorld);
            snapshot.correctionAxisValid = true;
            snapshot.correctionUsedAntiparallelFallback = true;
        }

        constexpr float radiansToDegrees =
            57.2957795130823208768f;
        const float neutralResidualDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    predictedNeutral,
                    fineTunedTarget)));
        const float fineTuneTargetOffsetDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    fineTunedTarget,
                    wristForward)));
        snapshot.correctionWorld = correction;
        snapshot.predictedWeaponWorld = predictedWeapon;
        snapshot.predictedFireNodeWorld = predictedFire;
        snapshot.wristForwardWorld = wristForward;
        snapshot.fineTunedTargetForwardWorld = fineTunedTarget;
        snapshot.weaponRootForwardWorld = weaponRootForward;
        snapshot.unalignedLiveFireWorld = unalignedLiveFire;
        snapshot.neutralFireWorldBefore = neutralFireWorld;
        snapshot.predictedNeutralFireWorld = predictedNeutral;
        snapshot.correctionAxisWorld = correctionAxis;
        snapshot.unalignedAngleDegrees =
            correctionAngle * radiansToDegrees;
        snapshot.correctionAngleRadians = correctionAngle;
        snapshot.correctionAngleDegrees =
            correctionAngle * radiansToDegrees;
        snapshot.fineTuneTargetOffsetDegrees =
            std::acos(fineTuneTargetOffsetDot) * radiansToDegrees;
        snapshot.neutralResidualDegrees =
            std::acos(neutralResidualDot) * radiansToDegrees;
        snapshot.correctionValid = true;
        snapshot.predictionValid = true;
        return true;
    }

    void TwoHandedGrip::prepareGunstockAlignmentDebugSnapshot(
        RE::NiNode* weaponNode,
        RE::NiAVObject* projectileNode,
        const std::uint32_t currentWeaponFormID,
        const std::uint64_t currentWeaponGenerationKey,
        const bool calibrationSampleBlocked,
        const bool authorityBlocked)
    {
        if (!g_rockConfig.rockDebugDrawGunstockAlignment) {
            _gunstockAlignmentDebugSnapshot = {};
            return;
        }

        ++_gunstockAlignmentDebugSequence;
        if (_gunstockAlignmentDebugSequence == 0) {
            ++_gunstockAlignmentDebugSequence;
        }

        auto& snapshot = _gunstockAlignmentDebugSnapshot;
        snapshot = {};
        snapshot.publicationSequence = _gunstockAlignmentDebugSequence;
        snapshot.weaponGenerationKey = currentWeaponGenerationKey;
        snapshot.canonicalCaptureSequence =
            _rightFiringHandCanonicalCaptureSequence;
        snapshot.weaponNodeIdentity =
            reinterpret_cast<std::uintptr_t>(weaponNode);
        snapshot.fireNodeIdentity =
            reinterpret_cast<std::uintptr_t>(projectileNode);
        snapshot.weaponFormID = currentWeaponFormID;
        snapshot.behaviorEnabled =
            g_rockConfig.rockGunstockModeEnabled;
        snapshot.weaponEligible = isGunstockWeaponEligible(
            weaponNode,
            currentWeaponGenerationKey);
        snapshot.firingHandIsLeft = _firingHandIsLeft;
        snapshot.gripState = _state;

        if (!snapshot.weaponEligible) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::WeaponNotEligible;
            return;
        }
        if (!weaponNode ||
            !projectileNode ||
            currentWeaponGenerationKey == 0 ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(projectileNode->world)) {
            snapshot.yieldReason = GunstockAlignmentDebugYieldReason::
                WeaponOrFireNodeUnavailable;
            return;
        }
        if (!runtime_state::isLocalSkeletonReady()) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::SkeletonUnavailable;
            return;
        }
        if (!f4vr::isNodeVisible(weaponNode)) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::WeaponHidden;
            return;
        }

        snapshot.weaponWorldBefore = weaponNode->world;
        snapshot.fireNodeWorldBefore = projectileNode->world;
        snapshot.weaponBeforeValid = true;
        snapshot.fireNodeBeforeValid = true;

        if (frik_visual_authority::isAvailable()) {
            RE::NiTransform firingHandWorld{};
            RE::NiTransform firingDriverWorld{};
            if (tryResolveGunstockPhysicalFiringFrame(
                    firingHandWorld,
                    firingDriverWorld)) {
                snapshot.dampedDriverWorld = firingDriverWorld;
                snapshot.firingHandWorld = firingHandWorld;
                snapshot.dampedDriverValid = true;
                snapshot.firingHandValid = true;
                snapshot.pivotWorld = firingDriverWorld.translate;

                if (_firingHandIsLeft) {
                    snapshot.leftHandWorld = firingHandWorld;
                    snapshot.leftHandValid = true;
                }
            }

            if (!_firingHandIsLeft) {
                RE::NiTransform leftHandWorld{};
                if (tryGetRootFlattenedHandBoneTransform(
                        true,
                        leftHandWorld)) {
                    snapshot.leftHandWorld = leftHandWorld;
                    snapshot.leftHandValid = true;
                }
            }
        }

        if (!snapshot.dampedDriverValid ||
            !snapshot.firingHandValid) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::
                    DampedDriverUnavailable;
            return;
        }

        RE::NiPoint3 previewNeutralHandLocal{};
        const bool runtimeIdentityMatches =
            snapshot.behaviorEnabled &&
            _gunstockAlignment.weaponNodeIdentity == weaponNode &&
            _gunstockAlignment.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            _gunstockAlignment.firingHandIsLeft == _firingHandIsLeft;
        if (runtimeIdentityMatches &&
            _gunstockAlignment.directionLatch.latched) {
            previewNeutralHandLocal =
                _gunstockAlignment.directionLatch.neutralHandLocal;
        } else if (!gunstock_alignment_policy::
                       tryCaptureHandLocalBore(
                           snapshot.firingHandWorld,
                           snapshot.fireNodeWorldBefore,
                           previewNeutralHandLocal)) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::FrikUnavailable;
            return;
        }
        (void)populateGunstockAlignmentDebugPrediction(
            previewNeutralHandLocal);

        if (!snapshot.behaviorEnabled) {
            snapshot.state =
                GunstockAlignmentDebugState::DisabledPreview;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::AlignmentDisabled;
            return;
        }
        if (!frik_visual_authority::isAvailable()) {
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::FrikUnavailable;
            return;
        }

        const bool neutralSampleBlocked =
            calibrationSampleBlocked ||
            _scopeMenuOpenThisFrame ||
            _scopeMenuClosedThisFrame;
        snapshot.yieldReason =
            currentGunstockAlignmentYieldReason(authorityBlocked);
        if (snapshot.yieldReason !=
            GunstockAlignmentDebugYieldReason::None) {
            snapshot.state = GunstockAlignmentDebugState::Yielded;
            return;
        }
        if (!runtimeIdentityMatches ||
            !_gunstockAlignment.directionLatch.latched) {
            snapshot.state = GunstockAlignmentDebugState::Waiting;
            snapshot.yieldReason = neutralSampleBlocked ?
                GunstockAlignmentDebugYieldReason::NeutralSampleBlocked :
                GunstockAlignmentDebugYieldReason::NeutralSampleCollecting;
            return;
        }
        snapshot.state = GunstockAlignmentDebugState::Latched;
    }

    void TwoHandedGrip::finalizeGunstockAlignmentDebugSnapshot(
        RE::NiNode* weaponNode,
        RE::NiAVObject* projectileNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!g_rockConfig.rockDebugDrawGunstockAlignment) {
            _gunstockAlignmentDebugSnapshot = {};
            return;
        }

        auto& snapshot = _gunstockAlignmentDebugSnapshot;
        if (snapshot.publicationSequence == 0 ||
            !snapshot.dampedDriverValid ||
            !snapshot.weaponBeforeValid ||
            !snapshot.fireNodeBeforeValid ||
            !weaponNode ||
            !projectileNode ||
            snapshot.weaponNodeIdentity !=
                reinterpret_cast<std::uintptr_t>(weaponNode) ||
            snapshot.fireNodeIdentity !=
                reinterpret_cast<std::uintptr_t>(projectileNode) ||
            snapshot.weaponGenerationKey != currentWeaponGenerationKey ||
            !runtime_state::isLocalSkeletonReady() ||
            !f4vr::isNodeVisible(weaponNode) ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(projectileNode->world)) {
            snapshot = {};
            return;
        }

        const bool runtimeIdentityMatches =
            _gunstockAlignment.weaponNodeIdentity == weaponNode &&
            _gunstockAlignment.weaponGenerationKey ==
                currentWeaponGenerationKey &&
            _gunstockAlignment.firingHandIsLeft == _firingHandIsLeft;
        if (snapshot.behaviorEnabled && runtimeIdentityMatches) {
            snapshot.candidateSamples =
                _gunstockAlignment.directionLatch.candidateSamples;
            RE::NiPoint3 runtimeNeutralHandLocal{};
            if (_gunstockAlignment.directionLatch.latched) {
                runtimeNeutralHandLocal =
                    _gunstockAlignment.directionLatch.
                        neutralHandLocal;
                (void)populateGunstockAlignmentDebugPrediction(
                    runtimeNeutralHandLocal);
            } else if (_gunstockAlignment.directionLatch.candidateSamples !=
                           0 &&
                       gunstock_alignment_policy::tryNormalizeDirection(
                           _gunstockAlignment.directionLatch.candidateSum,
                           runtimeNeutralHandLocal)) {
                (void)populateGunstockAlignmentDebugPrediction(
                    runtimeNeutralHandLocal);
            }
        }

        snapshot.finalWeaponWorld = weaponNode->world;
        snapshot.finalFireNodeWorld = projectileNode->world;
        snapshot.finalWeaponValid = true;
        snapshot.finalFireNodeValid = true;
        if (!snapshot.renderedFiringHandValid) {
            RE::NiTransform renderedFiringHandWorld{};
            if (tryGetRootFlattenedHandBoneTransform(
                    _firingHandIsLeft,
                    renderedFiringHandWorld)) {
                snapshot.renderedFiringHandWorld =
                    renderedFiringHandWorld;
                snapshot.renderedFiringHandValid = true;
            }
        }
        snapshot.alignmentAppliedThisFrame =
            _gunstockAlignmentAppliedThisFrame;

        const RE::NiPoint3 localForward{ 0.0f, 1.0f, 0.0f };
        RE::NiPoint3 finalLiveFire{};
        if (!gunstock_alignment_policy::tryNormalizeDirection(
                transform_math::localVectorToWorld(
                    snapshot.finalFireNodeWorld,
                    localForward),
                finalLiveFire)) {
            snapshot = {};
            return;
        }
        snapshot.finalLiveFireWorld = finalLiveFire;
        constexpr float radiansToDegrees =
            57.2957795130823208768f;
        const float liveDot = (std::max)(
            -1.0f,
            (std::min)(
                1.0f,
                weaponSolverDot(
                    snapshot.finalLiveFireWorld,
                    snapshot.predictionValid ?
                        snapshot.fineTunedTargetForwardWorld :
                        snapshot.wristForwardWorld)));
        snapshot.liveDeviationDegrees =
            std::acos(liveDot) * radiansToDegrees;
        if (snapshot.predictionValid) {
            snapshot.finalWeaponPredictionErrorGameUnits =
                weaponSolverLength(
                    weaponSolverSub(
                        snapshot.finalWeaponWorld.translate,
                        snapshot.predictedWeaponWorld.translate));
        }

        if (!snapshot.behaviorEnabled) {
            snapshot.state =
                GunstockAlignmentDebugState::DisabledPreview;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::AlignmentDisabled;
        } else if (snapshot.yieldReason ==
                       GunstockAlignmentDebugYieldReason::
                           FrikUnavailable ||
                   snapshot.yieldReason ==
                       GunstockAlignmentDebugYieldReason::
                           SkeletonUnavailable) {
            snapshot.state = GunstockAlignmentDebugState::Invalid;
        } else if (snapshot.yieldReason !=
                       GunstockAlignmentDebugYieldReason::None &&
                   snapshot.yieldReason !=
                       GunstockAlignmentDebugYieldReason::
                           NeutralSampleBlocked &&
                   snapshot.yieldReason !=
                       GunstockAlignmentDebugYieldReason::
                           NeutralSampleCollecting) {
            snapshot.state = GunstockAlignmentDebugState::Yielded;
        } else if (!runtimeIdentityMatches ||
                   !_gunstockAlignment.directionLatch.latched) {
            snapshot.state = GunstockAlignmentDebugState::Waiting;
            if (snapshot.yieldReason !=
                GunstockAlignmentDebugYieldReason::NeutralSampleBlocked) {
                snapshot.yieldReason =
                    GunstockAlignmentDebugYieldReason::
                        NeutralSampleCollecting;
            }
        } else if (_gunstockAlignmentAppliedThisFrame) {
            snapshot.state = GunstockAlignmentDebugState::Active;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::None;
            if (snapshot.recoilWitness ==
                GunstockAlignmentDebugRecoilWitness::None) {
                snapshot.recoilWitness =
                    GunstockAlignmentDebugRecoilWitness::Regular;
            }
        } else {
            snapshot.state = GunstockAlignmentDebugState::Latched;
            snapshot.yieldReason =
                GunstockAlignmentDebugYieldReason::
                    RuntimeAuthorityUnavailable;
        }
        snapshot.published = true;
    }
    bool TwoHandedGrip::getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const
    {
        const auto& leftGrip = partGrip(true);
        const auto& rightGrip = partGrip(false);
        if (!_hasSolvedWeaponTransform || !_activeWeaponNode) {
            return false;
        }
        if (!_hasFiringHandWeaponLocal && !leftGrip.active && !rightGrip.active) {
            return false;
        }

        outSnapshot.weaponWorld = _lastSolvedWeaponTransform;
        if (rightGrip.active) {
            outSnapshot.rightRequestedHandWorld = resolvePartGripHandWorld(rightGrip, _activeWeaponNode);
            outSnapshot.rightGripWorld = resolvePartGripWorld(rightGrip, _activeWeaponNode);
        } else {
            outSnapshot.rightRequestedHandWorld = transform_math::composeTransforms(_lastSolvedWeaponTransform, _primaryHandWeaponLocal);
            outSnapshot.rightGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        }
        if (leftGrip.active) {
            outSnapshot.leftRequestedHandWorld = resolvePartGripHandWorld(leftGrip, _activeWeaponNode);
            outSnapshot.leftGripWorld = resolvePartGripWorld(leftGrip, _activeWeaponNode);
        } else {
            outSnapshot.leftRequestedHandWorld = RE::NiTransform{};
            outSnapshot.leftGripWorld = RE::NiPoint3{};
        }
        return true;
    }

    bool TwoHandedGrip::getAuthoredSupportGripDebugSnapshot(
        AuthoredSupportGripDebugSnapshot& outSnapshot) const
    {
        outSnapshot = _authoredSupportGripDebugSnapshot;
        return outSnapshot.valid;
    }
}

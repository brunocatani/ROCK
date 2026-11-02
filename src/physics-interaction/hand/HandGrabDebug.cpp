#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabInternal.h"

#include <ctime>
#include <filesystem>
#include <format>
#include <fstream>

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/scene/HeldScenePresentation.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/native/havok/BodyCollisionControl.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "physics-interaction/native/havok/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/havok/PhysicsScale.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"
#include "physics-interaction/native/query/PhysicsShapeCast.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/authored/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/classification/WeaponTypePolicy.h"
#include "physics-interaction/weapon/grip/LooseWeaponGripZone.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <xmmintrin.h>

namespace rock
{
    using namespace hand_grab_internal;

    bool Hand::getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform objectBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, objectBodyWorld)) {
            return false;
        }

        if (!_activeConstraint.isValid() || !_activeConstraint.constraintData) {
            return false;
        }

        RE::NiTransform anchorBodyWorld{};
        if (!_grabAuthorityProxy.isValid() || _grabAuthorityProxy.getBodyId().value == INVALID_BODY_ID ||
            !tryGetGrabAuthorityBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), anchorBodyWorld)) {
            return false;
        }

        auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
        auto* pivotALocal = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_A_POS);
        auto* pivotBLocal = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
        const RE::NiPoint3 pivotALocalGame{ pivotALocal[0] * havokToGameScale(), pivotALocal[1] * havokToGameScale(),
            pivotALocal[2] * havokToGameScale() };
        const RE::NiPoint3 pivotBLocalGame{ pivotBLocal[0] * havokToGameScale(), pivotBLocal[1] * havokToGameScale(),
            pivotBLocal[2] * havokToGameScale() };
        out.handPivotWorld = transform_math::localPointToWorld(anchorBodyWorld, pivotALocalGame);
        out.objectPivotWorld = transform_math::localPointToWorld(objectBodyWorld, pivotBLocalGame);
        out.handBodyWorld = anchorBodyWorld.translate;
        out.objectBodyWorld = objectBodyWorld.translate;

        const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
        out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
        return true;
    }

    bool Hand::getGrabPresentationNodeDebugSnapshot(GrabPresentationNodeDebugSnapshot& out) const
    {
        out = {};
        if (!isHolding()) {
            return false;
        }

        const auto sampleNode = [](const RE::NiAVObject* node) {
            GrabPresentationNodeDebugPose pose{};
            if (!node) {
                return pose;
            }

            pose.node = node;
            pose.parent = node->parent;
            pose.local = node->local;
            pose.world = node->world;
            pose.previousWorld = node->previousWorld;
            pose.valid = grab_three_phase::isFinite(pose.local) && grab_three_phase::isFinite(pose.world) &&
                grab_three_phase::isFinite(pose.previousWorld);
            return pose;
        };

        const auto* referenceRoot = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;
        const auto* visibleGeometry = _grabFrame.gripEvidence.gripSourceNode;

        out.collisionOwner = sampleNode(_grabFrame.heldNode);
        out.referenceRoot = sampleNode(referenceRoot);
        out.visibleGeometry = sampleNode(visibleGeometry);
        out.visibleGeometryParent = sampleNode(visibleGeometry ? visibleGeometry->parent : nullptr);
        out.traceId = _grabFrame.traceId;
        out.targetKind = _savedObjectState.targetKind;
        out.driveMode = _heldDriveDecision.mode;
        out.looseWeapon = _heldObjectIsLooseWeapon;
        return out.collisionOwner.valid || out.referenceRoot.valid || out.visibleGeometry.valid ||
            out.visibleGeometryParent.valid;
    }

    bool Hand::getGrabPocketNormalDebugSnapshot(RE::hknpWorld* world, GrabPocketNormalDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.gripEvidence.hasGripPoint ||
            _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform grabBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
            return false;
        }
        const RE::NiTransform currentNodeWorld =
            deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.authority.bodyLocal);
        out.contactPointWorld = gripEvidencePointWorld(_grabFrame, currentNodeWorld);

        const RE::NiPoint3 normalWorld = gripEvidenceNormalWorld(_grabFrame, currentNodeWorld);
        if (normalWorld.x == 0.0f && normalWorld.y == 0.0f && normalWorld.z == 0.0f) {
            return false;
        }

        constexpr float kFrameAxisLengthGameUnits = 12.0f;
        out.normalEndWorld = out.contactPointWorld + normalWorld * kFrameAxisLengthGameUnits;

        return true;
    }

    bool Hand::getGrabSupportFrameDebugSnapshot(RE::hknpWorld* world, GrabSupportFrameDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.authority.hasFrozenPivotB ||
            _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform liveBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, liveBodyWorld)) {
            return false;
        }

        out.pivotWorld = transform_math::localPointToWorld(liveBodyWorld, activeProxyConstraintPivotBLocalGame());
        const RE::NiTransform currentNodeWorld =
            deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabFrame.authority.bodyLocal);

        RE::NiPoint3 normalWorld = _grabFrame.gripEvidence.hasGripPoint
            ? gripEvidenceNormalWorld(_grabFrame, currentNodeWorld)
            : RE::NiPoint3{};
        if (vector_math::lengthSquared(normalWorld) <= 0.000001f) {
            return false;
        }
        const RE::NiPoint3 axisWorld = stablePerpendicularAxis(normalWorld);
        const RE::NiPoint3 binormalWorld = normalizeOrZero(vector_math::cross(normalWorld, axisWorld));
        constexpr float axisLength = 10.0f;
        out.axisLengthGameUnits = axisLength;
        out.normalEndWorld = out.pivotWorld + normalWorld * axisLength;
        out.supportAxisEndWorld = out.pivotWorld + axisWorld * axisLength;
        out.binormalEndWorld = out.pivotWorld + binormalWorld * axisLength;
        out.hasNormal = true;
        out.hasSupportAxis = vector_math::lengthSquared(axisWorld) > 0.000001f;
        out.hasBinormal = vector_math::lengthSquared(binormalWorld) > 0.000001f;
        out.pivotAuthoritySource =
            grab_authority_frame_math::grabAuthorityPivotSourceName(_grabFrame.pivotAuthority.source);
        out.activeGrabPointMode = _grabFrame.seat.activeGrabPointMode ? _grabFrame.seat.activeGrabPointMode : "none";
        out.positionOnlyPivot = _grabFrame.pivotAuthority.positionOnly;
        out.normalTrusted = _grabFrame.pivotAuthority.normalTrusted;

        if (_grabFrame.gripEvidence.gripEvidenceTriangleIndex < _grabFrame.localMeshTriangles.size()) {
            const auto& triangle = _grabFrame.localMeshTriangles[_grabFrame.gripEvidence.gripEvidenceTriangleIndex];
            out.pivotTriangleWorld[0] = transform_math::localPointToWorld(currentNodeWorld, triangle.v0);
            out.pivotTriangleWorld[1] = transform_math::localPointToWorld(currentNodeWorld, triangle.v1);
            out.pivotTriangleWorld[2] = transform_math::localPointToWorld(currentNodeWorld, triangle.v2);
            out.hasPivotTriangle = true;
        }

        return out.hasNormal || out.hasSupportAxis || out.hasBinormal || out.hasPivotTriangle;
    }

    bool Hand::getGrabForceTorqueDebugSnapshot(RE::hknpWorld* world, GrabForceTorqueDebugSnapshot& out) const
    {
        /*
         * This view is intentionally built from the same BODY-local pivot and
         * generated/proxy authority relation used by the active ragdoll-motor grab. Its
         * job is to show whether the solver is being asked to pull an off-center
         * frozen pivot toward the hand, which can rotate a body even when the
         * selected mesh/contact point looked reasonable at acquisition time.
         */
        out = {};

        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID ||
            !_grabFrame.authority.hasFrozenPivotB) {
            return false;
        }

        RE::NiTransform liveBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, liveBodyWorld)) {
            return false;
        }

        RE::NiTransform proxyWorld{};
        const char* proxySource = "none";
        bool proxyFrameOk =
            resolveGrabAuthorityProxyFrame(world, proxyWorld, proxySource, GrabAuthorityProxyFramePolicy::LivePalmOnly);
        if (!proxyFrameOk && _grabAuthorityProxy.isValid() &&
            _grabAuthorityProxy.getBodyId().value != INVALID_BODY_ID) {
            proxyFrameOk = tryGetGrabAuthorityBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), proxyWorld);
            proxySource = proxyFrameOk ? "proxyReadbackFallback" : "none";
        }
        if (!proxyFrameOk) {
            return false;
        }

        const RE::NiTransform desiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, _grabFrame.proxyAuthorityBodyHandSpace);
        if (_grabFrame.hasTelemetryCapture &&
            grab_authority_frame_math::isFiniteTransform(_grabFrame.authority.bodyWorldAtGrab) &&
            grab_authority_frame_math::isFiniteTransform(_grabFrame.authority.desiredBodyWorldAtGrab) &&
            grab_authority_frame_math::isFiniteVector(_grabFrame.authority.grabPivotWorldAtGrab) &&
            grab_authority_frame_math::isFiniteVector(_grabFrame.gripEvidence.gripPointWorldAtGrab) &&
            grab_authority_frame_math::isFiniteVector(_grabFrame.authority.pivotBConstraintLocalGame)) {
            const RE::NiPoint3 pivotBeforeFreeze = transform_math::localPointToWorld(
                _grabFrame.authority.bodyWorldAtGrab, _grabFrame.authority.pivotBConstraintLocalGame);
            const RE::NiPoint3 pivotAfterFreeze = transform_math::localPointToWorld(
                _grabFrame.authority.desiredBodyWorldAtGrab, _grabFrame.authority.pivotBConstraintLocalGame);
            const RE::NiPoint3 freezeShift =
                _grabFrame.authority.desiredBodyWorldAtGrab.translate - _grabFrame.authority.bodyWorldAtGrab.translate;
            const RE::NiPoint3 pivotGap =
                _grabFrame.authority.grabPivotWorldAtGrab - _grabFrame.gripEvidence.gripPointWorldAtGrab;
            const float freezeShiftLen = vectorMagnitude(freezeShift);
            const float pivotGapLen = vectorMagnitude(pivotGap);
            out.captureFreezeBodyShiftGameUnits = translationDeltaGameUnits(_grabFrame.authority.desiredBodyWorldAtGrab,
                _grabFrame.authority.bodyWorldAtGrab);
            out.captureFreezeBodyRotationDegrees = rotationDeltaDegrees(
                _grabFrame.authority.desiredBodyWorldAtGrab.rotate, _grabFrame.authority.bodyWorldAtGrab.rotate);
            out.captureFreezePivotGapBeforeGameUnits =
                pointDistanceGameUnits(pivotBeforeFreeze, _grabFrame.authority.grabPivotWorldAtGrab);
            out.captureFreezePivotGapAfterGameUnits =
                pointDistanceGameUnits(pivotAfterFreeze, _grabFrame.authority.grabPivotWorldAtGrab);
            out.captureFreezeShiftDot = freezeShiftLen > 0.0001f && pivotGapLen > 0.0001f
                ? std::clamp(vector_math::dot(freezeShift, pivotGap) / (freezeShiftLen * pivotGapLen), -1.0f, 1.0f)
                : 0.0f;
            out.captureFreezePivotLeverGameUnits = pointDistanceGameUnits(_grabFrame.gripEvidence.gripPointWorldAtGrab,
                _grabFrame.authority.bodyWorldAtGrab.translate);
        }
        RE::NiPoint3 pivotBLocalGame = activeProxyConstraintPivotBLocalGame();
        RE::NiPoint3 atomTransformBLocalGame = pivotBLocalGame;
        bool hasAtomMotorFrames = false;

        if (_activeConstraint.constraintData && _grabAuthorityProxyFrameValid) {
            const auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
            const auto* transformARotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_A_COL0);
            const auto* transformBRotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_COL0);
            const auto* transformBTranslation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
            const auto* targetBRca =
                reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
            atomTransformBLocalGame = RE::NiPoint3{
                transformBTranslation[0] * havokToGameScale(),
                transformBTranslation[1] * havokToGameScale(),
                transformBTranslation[2] * havokToGameScale(),
            };

            const RE::NiPoint3 pivotAProxyLocalGame = _grabAuthorityPivotAProxyLocalGame;
            if (std::isfinite(atomTransformBLocalGame.x) && std::isfinite(atomTransformBLocalGame.y) &&
                std::isfinite(atomTransformBLocalGame.z) && std::isfinite(pivotAProxyLocalGame.x) &&
                std::isfinite(pivotAProxyLocalGame.y) && std::isfinite(pivotAProxyLocalGame.z)) {
                out.motorAnchorAWorld = generatedProxyLocalPointToWorld(proxyWorld, pivotAProxyLocalGame);
                out.motorAnchorBWorld = transform_math::localPointToWorld(liveBodyWorld, atomTransformBLocalGame);

                out.motorConstraintAWorld = makeGeneratedProxyAuthorityRelationFrame(proxyWorld);
                out.motorConstraintAWorld.translate = out.motorAnchorAWorld;

                const RE::NiMatrix3 transformAAsHkColumns = matrixFromHkColumns(transformARotation);
                const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformBRotation);
                const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(targetBRca);

                RE::NiTransform transformBLocal = makeIdentityTransform();
                transformBLocal.rotate = transformBAsHkColumns;
                transformBLocal.translate = atomTransformBLocalGame;
                out.motorConstraintBWorld = transform_math::composeTransforms(liveBodyWorld, transformBLocal);

                const RE::NiTransform bodyInProxyBeforeInversion =
                    grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorld, desiredBodyWorld);
                const RE::NiTransform proxyInBodyBeforeTargetWrite =
                    grab_constraint_math::proxyInBodyFromBodyInProxy(bodyInProxyBeforeInversion);
                const RE::NiPoint3 relationTransformBLocalGame =
                    grab_constraint_math::computeHiggsTransformBTranslationGameFromProxyInBody(
                        proxyInBodyBeforeTargetWrite, pivotAProxyLocalGame);

                out.motorRelationInputBodyWorld =
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, bodyInProxyBeforeInversion);
                out.motorRelationInverseBodyWorld = reconstructBodyWorldFromProxyInBody(proxyWorld,
                    proxyInBodyBeforeTargetWrite.rotate, relationTransformBLocalGame, pivotAProxyLocalGame);
                out.motorRelationPivotWorld =
                    transform_math::localPointToWorld(out.motorRelationInverseBodyWorld, relationTransformBLocalGame);

                out.motorAtomTargetBodyWorld = reconstructBodyWorldFromProxyInBody(proxyWorld, targetAsHkRows,
                    atomTransformBLocalGame, pivotAProxyLocalGame);
                out.motorSolverEffectiveBodyWorld =
                    reconstructSolverEffectiveBodyWorld(proxyWorld, transformAAsHkColumns, transformBAsHkColumns,
                        targetAsHkRows, atomTransformBLocalGame, out.motorAnchorAWorld, desiredBodyWorld.scale);
                RE::NiTransform liveProxyWorld{};
                const bool liveProxyOk = _grabAuthorityProxy.isValid() &&
                    tryGetGrabAuthorityBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), liveProxyWorld);
                if (liveProxyOk) {
                    const RE::NiPoint3 liveAnchorAWorld =
                        generatedProxyLocalPointToWorld(liveProxyWorld, pivotAProxyLocalGame);
                    const RE::NiTransform solverEffectiveLiveAWorld = reconstructSolverEffectiveBodyWorld(
                        liveProxyWorld, transformAAsHkColumns, transformBAsHkColumns, targetAsHkRows,
                        atomTransformBLocalGame, liveAnchorAWorld, desiredBodyWorld.scale);
                    out.motorSolverEffectiveLiveABodyDeltaDegrees =
                        rotationDeltaDegrees(solverEffectiveLiveAWorld.rotate, desiredBodyWorld.rotate);
                    out.motorSolverEffectiveTargetALiveADeltaDegrees = rotationDeltaDegrees(
                        out.motorSolverEffectiveBodyWorld.rotate, solverEffectiveLiveAWorld.rotate);
                    out.motorTargetProxyToLiveProxyDeltaGameUnits =
                        pointDistanceGameUnits(proxyWorld.translate, liveProxyWorld.translate);
                    out.motorTargetProxyToLiveProxyDeltaDegrees =
                        rotationDeltaDegrees(proxyWorld.rotate, liveProxyWorld.rotate);
                } else {
                    out.motorSolverEffectiveLiveABodyDeltaDegrees = -1.0f;
                    out.motorSolverEffectiveTargetALiveADeltaDegrees = -1.0f;
                    out.motorTargetProxyToLiveProxyDeltaGameUnits = -1.0f;
                    out.motorTargetProxyToLiveProxyDeltaDegrees = -1.0f;
                }
                out.motorAtomTargetPivotWorld =
                    transform_math::localPointToWorld(out.motorAtomTargetBodyWorld, atomTransformBLocalGame);
                out.motorTargetBodyDeltaGameUnits =
                    translationDeltaGameUnits(out.motorAtomTargetBodyWorld, desiredBodyWorld);
                out.motorTargetBodyDeltaDegrees =
                    rotationDeltaDegrees(out.motorAtomTargetBodyWorld.rotate, desiredBodyWorld.rotate);
                out.motorRelationInverseBodyDeltaGameUnits =
                    translationDeltaGameUnits(out.motorRelationInverseBodyWorld, desiredBodyWorld);
                out.motorRelationInverseBodyDeltaDegrees =
                    rotationDeltaDegrees(out.motorRelationInverseBodyWorld.rotate, desiredBodyWorld.rotate);
                out.motorAtomToRelationInverseDeltaDegrees =
                    rotationDeltaDegrees(out.motorAtomTargetBodyWorld.rotate, out.motorRelationInverseBodyWorld.rotate);
                out.motorSolverEffectiveBodyDeltaGameUnits =
                    translationDeltaGameUnits(out.motorSolverEffectiveBodyWorld, desiredBodyWorld);
                out.motorSolverEffectiveBodyDeltaDegrees =
                    rotationDeltaDegrees(out.motorSolverEffectiveBodyWorld.rotate, desiredBodyWorld.rotate);
                out.motorSolverEffectiveToAtomDeltaDegrees =
                    rotationDeltaDegrees(out.motorSolverEffectiveBodyWorld.rotate, out.motorAtomTargetBodyWorld.rotate);
                out.motorTransformBRelationLocalDeltaGameUnits =
                    pointDistanceGameUnits(atomTransformBLocalGame, relationTransformBLocalGame);
                out.motorTransformBPivotToAnchorAGameUnits =
                    pointDistanceGameUnits(out.motorAtomTargetPivotWorld, out.motorAnchorAWorld);
                out.motorTargetBodyDeltaEndWorld = out.motorAtomTargetBodyWorld.translate;
                out.hasMotorConstraintFrames = true;
                out.hasMotorRelationFrames = true;
                out.hasMotorSolverEffectiveBody = true;
                out.hasMotorTargetBodyDelta =
                    out.motorTargetBodyDeltaGameUnits > 0.01f || out.motorTargetBodyDeltaDegrees > 0.01f;

                const RE::NiPoint3 angularAxis =
                    rotationCorrectionAxisWorld(liveBodyWorld.rotate, out.motorAtomTargetBodyWorld.rotate);
                if (vector_math::lengthSquared(angularAxis) > 0.000001f) {
                    const float axisLength = std::clamp(out.motorTargetBodyDeltaDegrees * 0.18f, 6.0f, 34.0f);
                    out.motorAngularAxisEndWorld = liveBodyWorld.translate + angularAxis * axisLength;
                    out.hasMotorAngularCommand = true;
                }

                hasAtomMotorFrames = true;
            }
        }

        pivotBLocalGame = hasAtomMotorFrames ? atomTransformBLocalGame : pivotBLocalGame;
        const RE::NiPoint3 livePivotWorld = transform_math::localPointToWorld(liveBodyWorld, pivotBLocalGame);
        const RE::NiPoint3 targetPivotWorld = transform_math::localPointToWorld(desiredBodyWorld, pivotBLocalGame);
        const RE::NiPoint3 correction = targetPivotWorld - livePivotWorld;
        const RE::NiPoint3 lever = livePivotWorld - liveBodyWorld.translate;
        const RE::NiPoint3 torqueWitness = vector_math::cross(lever, correction);
        const float correctionLength = vectorMagnitude(correction);
        const float leverLength = vectorMagnitude(lever);
        const float torqueWitnessLength = vectorMagnitude(torqueWitness);

        if (!std::isfinite(correctionLength) || !std::isfinite(leverLength) || !std::isfinite(torqueWitnessLength)) {
            return false;
        }

        out.pivotSourceBodyId = _savedObjectState.bodyId;
        out.liveBodyWorld = liveBodyWorld;
        out.desiredBodyWorld = desiredBodyWorld;
        out.livePivotWorld = livePivotWorld;
        out.targetPivotWorld = targetPivotWorld;
        out.activePivotBLiveBodyWorld = livePivotWorld;
        out.activePivotBDesiredBodyWorld = targetPivotWorld;
        out.correctionEndWorld = targetPivotWorld;
        out.leverArmEndWorld = livePivotWorld;
        out.pivotErrorGameUnits = correctionLength;
        out.pivotTrackingErrorGameUnits = correctionLength;
        out.correctionLengthGameUnits = correctionLength;
        out.leverLengthGameUnits = leverLength;
        out.torqueWitnessGameUnitsSquared = torqueWitnessLength;
        out.rotationErrorDegrees = rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorld.rotate);
        out.pocketDistanceGameUnits = _grabFrame.pivotAuthority.pocketDistanceGameUnits;
        out.selectionDistanceGameUnits = _grabFrame.pivotAuthority.selectionDistanceGameUnits;
        out.longLeverGameUnits = _grabFrame.pivotAuthority.longLeverGameUnits;
        out.positionConfidence = _grabFrame.pivotAuthority.positionConfidence;
        out.pivotAuthoritySource =
            grab_authority_frame_math::grabAuthorityPivotSourceName(_grabFrame.pivotAuthority.source);
        out.activeGrabPointMode = _grabFrame.seat.activeGrabPointMode ? _grabFrame.seat.activeGrabPointMode : "none";
        out.authorityFrameSource = proxySource ? proxySource : "none";
        out.acquisitionPhase = grab_three_phase::phaseName(_grabAcquisitionPhase);
        out.capturePivotAuthoritySource =
            grab_authority_frame_math::grabAuthorityPivotSourceName(_grabFrame.captureTelemetry.pivotAuthority.source);
        out.captureGrabPointMode = _grabFrame.captureTelemetry.seat.activeGrabPointMode
            ? _grabFrame.captureTelemetry.seat.activeGrabPointMode
            : "none";
        out.positionOnlyPivot = _grabFrame.pivotAuthority.positionOnly;
        out.normalTrusted = _grabFrame.pivotAuthority.normalTrusted;

        if (torqueWitnessLength > 0.001f) {
            const RE::NiPoint3 torqueAxis = torqueWitness * (1.0f / torqueWitnessLength);
            const float torqueAxisLength = std::clamp(std::sqrt(torqueWitnessLength), 6.0f, 28.0f);
            out.torqueAxisEndWorld = liveBodyWorld.translate + torqueAxis * torqueAxisLength;
            out.hasTorqueAxis = true;
        }

        const RE::NiTransform currentNodeWorld =
            deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabFrame.authority.bodyLocal);
        /*
         * Active pivot-B markers answer a different question than mesh evidence:
         * mesh points show what ROCK selected visually, while this converts the
         * exact BODY-local pivot consumed by the constraint back through the
         * rendered node. If this visual equivalent separates from the live BODY
         * pivot, the solver and rendered mesh disagree even when the blue mesh
         * selection point itself looks correct.
         */
        if (_grabFrame.heldNode) {
            const RE::NiPoint3 activePivotBNodeLocal =
                transform_math::worldPointToLocal(currentNodeWorld, livePivotWorld);
            out.activePivotBVisualNodeWorld =
                transform_math::localPointToWorld(_grabFrame.heldNode->world, activePivotBNodeLocal);
            out.activePivotBVisualLockErrorGameUnits =
                pointDistanceGameUnits(livePivotWorld, out.activePivotBVisualNodeWorld);
            out.hasActivePivotBVisualNode = true;
        }

        if (_grabFrame.gripEvidence.hasGripPoint) {
            out.meshGripPointWorld = gripEvidencePointWorld(_grabFrame, currentNodeWorld);
            out.hasMeshGripPoint = true;
            if (_grabFrame.gripEvidence.gripSourceNode || _grabFrame.gripEvidence.hasGripSourceNodePoint) {
                out.visualMeshGripPointWorld = out.meshGripPointWorld;
                out.hasVisualMeshGripPoint = true;
                out.bodyVisualMeshLockErrorGameUnits = pointDistanceGameUnits(livePivotWorld, out.meshGripPointWorld);
            } else if (_grabFrame.heldNode) {
                out.visualMeshGripPointWorld = transform_math::localPointToWorld(_grabFrame.heldNode->world,
                    _grabFrame.gripEvidence.gripPointLocal);
                out.hasVisualMeshGripPoint = true;
                out.bodyVisualMeshLockErrorGameUnits =
                    pointDistanceGameUnits(out.meshGripPointWorld, out.visualMeshGripPointWorld);
            }
        }

        if (_grabFrame.captureTelemetry.valid && _grabFrame.captureTelemetry.gripEvidence.hasGripPoint) {
            const auto& capture = _grabFrame.captureTelemetry;
            const RE::NiTransform captureNodeWorld =
                deriveNodeWorldFromBodyWorld(liveBodyWorld, capture.authority.bodyLocal);
            if (capture.gripEvidence.hasGripSourceNodePoint) {
                out.captureMeshGripPointBodyWorld = transform_math::localPointToWorld(
                    gripEvidenceWorldFrame(capture, captureNodeWorld), capture.gripEvidence.gripPointSourceNodeLocal);
            } else {
                out.captureMeshGripPointBodyWorld =
                    transform_math::localPointToWorld(captureNodeWorld, capture.gripEvidence.gripPointLocal);
            }
            out.hasCaptureMeshGripPoint = true;
            if (capture.gripEvidence.hasGripSourceNodePoint) {
                out.captureMeshGripPointVisualWorld = out.captureMeshGripPointBodyWorld;
            } else if (_grabFrame.heldNode) {
                out.captureMeshGripPointVisualWorld =
                    transform_math::localPointToWorld(_grabFrame.heldNode->world, capture.gripEvidence.gripPointLocal);
            } else {
                out.captureMeshGripPointVisualWorld = out.captureMeshGripPointBodyWorld;
            }

            const RE::NiPoint3 localDelta =
                _grabFrame.gripEvidence.gripPointLocal - capture.gripEvidence.gripPointLocal;
            const float scale =
                std::isfinite(currentNodeWorld.scale) && currentNodeWorld.scale > 0.0f ? currentNodeWorld.scale : 1.0f;
            out.captureGripLocalDeltaGameUnits = vectorMagnitude(localDelta) * scale;
            out.gripPointMutatedAfterCapture = out.captureGripLocalDeltaGameUnits > 0.001f;
        }

        if (_grabFrame.gripEvidence.gripEvidenceTriangleIndex < _grabFrame.localMeshTriangles.size()) {
            const auto& triangle = _grabFrame.localMeshTriangles[_grabFrame.gripEvidence.gripEvidenceTriangleIndex];
            out.pivotTriangleWorld[0] = transform_math::localPointToWorld(currentNodeWorld, triangle.v0);
            out.pivotTriangleWorld[1] = transform_math::localPointToWorld(currentNodeWorld, triangle.v1);
            out.pivotTriangleWorld[2] = transform_math::localPointToWorld(currentNodeWorld, triangle.v2);
            out.hasPivotTriangle = true;
        }

        return true;
    }

    namespace
    {
        grab_pose_replay::Vec3 toReplay(const RE::NiPoint3& p) { return { p.x, p.y, p.z }; }

        grab_pose_replay::Transform toReplay(const RE::NiTransform& t)
        {
            grab_pose_replay::Transform out{};
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column)
                    out.rotate[row * 3 + column] = t.rotate.entry[row][column];
            }
            out.translate = toReplay(t.translate);
            out.scale = t.scale;
            return out;
        }

        grab_pose_replay::FingerSkeleton toReplay(const root_flattened_finger_skeleton_runtime::Snapshot& s)
        {
            grab_pose_replay::FingerSkeleton out{};
            for (std::size_t f = 0; f < out.fingers.size(); ++f) {
                const auto& in = s.fingers[f];
                auto& chain = out.fingers[f];
                for (std::size_t p = 0; p < 3; ++p)
                    chain.points[p] = toReplay(in.points[p]);
                chain.tipSegmentCenter = toReplay(in.tipSegmentCenterWorld);
                chain.tipDirection = toReplay(in.tipDirectionWorld);
                chain.tipGeometryValid = in.tipGeometryValid;
                chain.valid = in.valid;
            }
            out.palmNormal = toReplay(s.palmNormalWorld);
            out.palmNormalValid = s.palmNormalValid;
            out.inPowerArmor = s.inPowerArmor;
            out.valid = s.valid;
            return out;
        }
    }

    // Bench only: the solve half of the replay record, taken from the exact inputs and result of the capsule solve.
    void Hand::beginGrabPoseReplay(const GrabFingerSolveFrame& frame,
        const std::vector<GrabLocalTriangle>& localTriangles,
        const root_flattened_finger_skeleton_runtime::Snapshot& solveSkeleton,
        const grab_finger_capsule_sweep::Result& solve, const grab_finger_capsule_sweep::Options& options)
    {
        auto& r = _grabPoseReplay.emplace();
        r.isLeft = _isLeft;
        r.trianglesObjectLocal.reserve(localTriangles.size() * 9);
        for (const auto& triangle : localTriangles) {
            for (const auto* v : { &triangle.v0, &triangle.v1, &triangle.v2 }) {
                r.trianglesObjectLocal.insert(r.trianglesObjectLocal.end(), { v->x, v->y, v->z });
            }
        }
        r.solveObjectWorld = toReplay(frame.objectWorld);
        r.solveHandWorld = toReplay(frame.solveHandWorld);
        r.liveHandWorld = toReplay(frame.liveHandWorld);
        r.grabAnchorWorld = toReplay(frame.pivotWorld);
        r.solveSkeleton = toReplay(solveSkeleton);
        r.options = grab_pose_replay::CapsuleOptions{
            .minValue = options.minValue,
            .fingerMaxOpenValue = options.fingerMaxOpenValue,
            .thumbMaxOpenValue = options.thumbMaxOpenValue,
            .step = options.step,
        };
        for (std::size_t f = 0; f < grab_pose_replay::kFingerCount; ++f) {
            r.solvedValues[f] = solve.values[f];
            r.solvedContact[f] = solve.fingers[f].contact ? 1 : 0;
            r.solvedOpenedToClear[f] = solve.fingers[f].openedToClear ? 1 : 0;
            r.solvedBlocked[f] = solve.fingers[f].blocked ? 1 : 0;
        }

        std::array<RE::NiTransform, 15> closedLocal{};
        std::array<RE::NiTransform, 15> openLocal{};
        r.frikRigValid = grab_finger_capsule_sweep::captureFrikRigLocals(_isLeft, closedLocal, openLocal);
        if (r.frikRigValid) {
            for (std::size_t bone = 0; bone < grab_pose_replay::kFingerBoneCount; ++bone) {
                r.frikClosedLocal[bone] = toReplay(closedLocal[bone]);
                r.frikOpenLocal[bone] = toReplay(openLocal[bone]);
            }
        }
    }

    /*
     * Bench pose metric, logged once per grab 60 held updates (FRIK eases bones ~7.8%/frame, so ~99% settled) after the final pose
     * publishes. Everything comes from what is drawn: the rendered finger
     * bones and the held node's world transform, against the grab's cached
     * mesh index (object space). Distances are from bone centrelines to the
     * nearest mesh point, signed by the triangle's outward winding: negative
     * means the bone line is inside the object. Skin sits about 0.7 gu off the
     * finger centreline, so anything under +0.7 visibly penetrates. Curl
     * values and the open caps show whether a finger could have opened
     * further. Object-local palm and finger directions record how the hand sat
     * on the object, the input for deriving grasp rules from good grabs.
     */
    void Hand::logGrabPoseQuality()
    {
        if (!_grabFrame.heldNode || _grabFingerTriangleIndex.empty() || !_hasGrabVisualHandTransform) {
            ROCK_LOG_INFO(Hand, "{} GRAB POSE QUALITY: skipped heldNode={} meshIndex={} visualHand={}", handName(),
                _grabFrame.heldNode != nullptr, !_grabFingerTriangleIndex.empty(), _hasGrabVisualHandTransform);
            return;
        }
        /*
         * The drawn fingers: the in-frame flattened array may still hold pre-IK bones
         * (bench: 4.2 gu off the drawn wrist), so take the chain relative to its own
         * wrist (Controller capture, transported onto the raw hand) and place it at the
         * hand ROCK published, which is the drawn hand bone (AfterWorldFinal: 0.00 gu).
         */
        root_flattened_finger_skeleton_runtime::Snapshot rendered{};
        RE::NiTransform rawHand{};
        if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, rendered) ||
            !frik_hand_world_authority::tryGetRawHandWorld(_isLeft, rawHand) ||
            !grab_finger_capsule_sweep::rebaseSkeleton(rendered, rawHand, _grabVisualHandTransform) ||
            !rendered.palmNormalValid) {
            ROCK_LOG_INFO(Hand, "{} GRAB POSE QUALITY: skipped drawnSkeleton=no", handName());
            return;
        }

        const RE::NiTransform objectWorld = _grabFrame.heldNode->world;
        constexpr float kSearchGameUnits = 12.0f;
        constexpr float kNoSurface = 99.0f;
        const auto signedDistance = [&](const RE::NiPoint3& point, bool& inside) {
            RE::NiPoint3 nearest{};
            RE::NiPoint3 outward{};
            inside = _grabFingerTriangleIndex.queryPointInsideWorld(objectWorld, point);
            if (!_grabFingerTriangleIndex.querySphereWorld(objectWorld, point, kSearchGameUnits, &nearest, &outward,
                    nullptr, false)) {
                return kNoSurface;
            }
            const RE::NiPoint3 offset = point - nearest;
            const float distance = std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z);
            return offset.x * outward.x + offset.y * outward.y + offset.z * outward.z < 0.0f ? -distance : distance;
        };
        const auto lerp = [](const RE::NiPoint3& a, const RE::NiPoint3& b, float t) { return a + (b - a) * t; };
        const auto unitLocal = [&](const RE::NiPoint3& world) {
            RE::NiPoint3 local = transform_math::worldVectorToLocal(objectWorld, world);
            const float length = std::sqrt(local.x * local.x + local.y * local.y + local.z * local.z);
            return length > 0.000001f ? local * (1.0f / length) : RE::NiPoint3{};
        };

        static constexpr std::array<const char*, 3> kSegment{ "prox", "mid", "tip" };
        std::string fingers;
        for (std::size_t finger = 0; finger < rendered.fingers.size(); ++finger) {
            const auto& chain = rendered.fingers[finger];
            if (!chain.valid) {
                fingers += std::format(" f{}=invalid", finger);
                continue;
            }
            const RE::NiPoint3 tipEnd = chain.tipGeometryValid
                ? chain.points[2] + (chain.tipSegmentCenterWorld - chain.points[2]) * 2.0f
                : chain.points[2] + (chain.points[2] - chain.points[1]) * 0.8f;
            const std::array<RE::NiPoint3, 4> joints{ chain.points[0], chain.points[1], chain.points[2], tipEnd };
            float worst = kNoSurface;
            std::size_t worstSegment = 0;
            int insideSamples = 0;
            for (std::size_t segment = 0; segment < 3; ++segment) {
                for (int sample = 0; sample < 4; ++sample) {
                    bool inside = false;
                    const float distance =
                        signedDistance(lerp(joints[segment], joints[segment + 1], (sample + 0.5f) / 4.0f), inside);
                    insideSamples += inside ? 1 : 0;
                    if (distance < worst) {
                        worst = distance;
                        worstSegment = segment;
                    }
                }
            }
            const float openCap = finger == 0 ? g_rockConfig.rockGrabThumbSweepMaxOpenValue
                                              : g_rockConfig.rockGrabFingerSweepMaxOpenValue;
            const float curl = _grabFingerPose.values[finger];
            fingers += std::format(" f{}=[min={:.2f}@{} insideShell={}/12 curl={:.2f}{} contact={}]", finger, worst,
                kSegment[worstSegment], insideSamples, curl, curl >= openCap - 0.01f ? "(openCap)" : "",
                (_grabFingerPose.contactValidMask >> finger) & 1u);
        }

        // Palm centreline: wrist (visual hand origin) toward the knuckle midpoint.
        RE::NiPoint3 knuckles{};
        for (std::size_t finger = 1; finger < 5; ++finger)
            knuckles = knuckles + rendered.fingers[finger].points[0] * 0.25f;
        const RE::NiPoint3 wrist = _grabVisualHandTransform.translate;
        float palmMin = kNoSurface;
        int palmInside = 0;
        for (const float t : { 0.35f, 0.6f, 0.85f }) {
            bool inside = false;
            palmMin = (std::min)(palmMin, signedDistance(lerp(wrist, knuckles, t), inside));
            palmInside += inside ? 1 : 0;
        }
        const RE::NiPoint3 palmNormalLocal = unitLocal(rendered.palmNormalWorld);
        const RE::NiPoint3 fingerDirLocal = unitLocal(knuckles - wrist);
        ROCK_LOG_INFO(Hand,
            "{} GRAB POSE QUALITY: source={} palm=[min={:.2f} insideShell={}/3 normalLocal=({:.2f},{:.2f},{:.2f}) "
            "fingerDirLocal=({:.2f},{:.2f},{:.2f})] "
            "contactMask=0x{:02X}{}",
            handName(), _grabObjectGripAtGrab.source ? _grabObjectGripAtGrab.source : "none", palmMin, palmInside,
            palmNormalLocal.x, palmNormalLocal.y, palmNormalLocal.z, fingerDirLocal.x, fingerDirLocal.y,
            fingerDirLocal.z, _grabFingerPose.contactValidMask, fingers);
        writeGrabPoseReplay(rendered, objectWorld);
    }

    // Bench only: completes the replay record with what was published and rendered, and writes it once.
    void Hand::writeGrabPoseReplay(const root_flattened_finger_skeleton_runtime::Snapshot& rendered,
        const RE::NiTransform& renderedObjectWorld)
    {
        if (!_grabPoseReplay) {
            return;
        }
        auto record = std::move(*_grabPoseReplay);
        _grabPoseReplay.reset();

        const std::uint32_t formId = _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0;
        record.object = std::format("0x{:08X}", formId);
        if (_savedObjectState.refr) {
            if (const char* name = _savedObjectState.refr->GetDisplayFullName(); name && *name)
                record.objectName = name;
        }
        record.acquisition = _grabFrame.seat.diagnostics.acquisitionMode;
        record.gripSource = _grabObjectGripAtGrab.source ? _grabObjectGripAtGrab.source : "none";
        const std::time_t now = std::time(nullptr);
        std::tm local{};
        char stamp[32]{};
        if (localtime_s(&local, &now) == 0)
            std::strftime(stamp, sizeof(stamp), "%Y-%m-%d_%H-%M-%S", &local);
        record.capturedLocal = stamp;

        for (std::size_t f = 0; f < grab_pose_replay::kFingerCount; ++f)
            record.finalValues[f] = _grabFingerPose.values[f];
        record.finalJointValues = _grabFingerPose.jointValues;
        record.publishedJointValues = _grabFingerJointPose;
        record.renderedObjectWorld = toReplay(renderedObjectWorld);
        record.renderedVisualHandWorld = toReplay(_grabVisualHandTransform);
        record.rendered = toReplay(rendered);
        RE::NiTransform presented{};
        record.presentedHandValid = frik_hand_world_authority::tryGetPresentedHandWorld(_isLeft, presented);
        record.presentedHandWorld = toReplay(presented);

        // One small file per grab, bench only, written after the pose settled (never on the solve frame).
        try {
            auto directory = F4SE::log::log_directory();
            if (!directory) {
                ROCK_LOG_WARN(Hand, "{} GRAB POSE REPLAY: no F4SE log directory", handName());
                return;
            }
            const auto folder = *directory / "ROCK_GrabReplay";
            std::filesystem::create_directories(folder);
            const auto path =
                folder / std::format("{}_{:08X}_{}.json", record.capturedLocal, formId, _isLeft ? "left" : "right");
            std::ofstream out(path, std::ios::binary | std::ios::trunc);
            out << grab_pose_replay::serialize(record);
            ROCK_LOG_INFO(Hand, "{} GRAB POSE REPLAY: wrote {} ({} triangles){}", handName(), path.string(),
                record.trianglesObjectLocal.size() / 9, out.good() ? "" : " WRITE FAILED");
        } catch (const std::exception& error) {
            ROCK_LOG_WARN(Hand, "{} GRAB POSE REPLAY: write failed: {}", handName(), error.what());
        }
    }

}

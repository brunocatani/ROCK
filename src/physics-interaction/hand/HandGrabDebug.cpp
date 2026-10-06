#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandGrabInternal.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HeldScenePresentation.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugMath.h"
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
#include "physics-interaction/VectorMath.h"
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
        const RE::NiPoint3 pivotALocalGame{ pivotALocal[0] * havokToGameScale(), pivotALocal[1] * havokToGameScale(), pivotALocal[2] * havokToGameScale() };
        const RE::NiPoint3 pivotBLocalGame{ pivotBLocal[0] * havokToGameScale(), pivotBLocal[1] * havokToGameScale(), pivotBLocal[2] * havokToGameScale() };
        out.handPivotWorld = transform_math::localPointToWorld(anchorBodyWorld, pivotALocalGame);
        out.objectPivotWorld = transform_math::localPointToWorld(objectBodyWorld, pivotBLocalGame);
        out.handBodyWorld = anchorBodyWorld.translate;
        out.objectBodyWorld = objectBodyWorld.translate;

        const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
        out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
        return true;
    }

    bool Hand::getGrabPresentationNodeDebugSnapshot(
        GrabPresentationNodeDebugSnapshot& out) const
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
            pose.valid = grab_three_phase::isFinite(pose.local) &&
                         grab_three_phase::isFinite(pose.world) &&
                         grab_three_phase::isFinite(pose.previousWorld);
            return pose;
        };

        const auto* referenceRoot =
            _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;
        const auto* visibleGeometry = _grabFrame.gripEvidence.gripSourceNode;

        out.collisionOwner = sampleNode(_grabFrame.heldNode);
        out.referenceRoot = sampleNode(referenceRoot);
        out.visibleGeometry = sampleNode(visibleGeometry);
        out.visibleGeometryParent = sampleNode(
            visibleGeometry ? visibleGeometry->parent : nullptr);
        out.traceId = _grabFrame.traceId;
        out.targetKind = _savedObjectState.targetKind;
        out.driveMode = _heldDriveDecision.mode;
        out.looseWeapon = _heldObjectIsLooseWeapon;
        return out.collisionOwner.valid || out.referenceRoot.valid ||
               out.visibleGeometry.valid || out.visibleGeometryParent.valid;
    }

    bool Hand::getGrabPocketNormalDebugSnapshot(RE::hknpWorld* world, GrabPocketNormalDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.gripEvidence.hasGripPoint || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform grabBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
            return false;
        }
        const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.authority.bodyLocal);
        out.contactPointWorld = gripEvidencePointWorld(_grabFrame, currentNodeWorld);

        const RE::NiPoint3 normalWorld = gripEvidenceNormalWorld(_grabFrame, currentNodeWorld);
        if (normalWorld.x == 0.0f && normalWorld.y == 0.0f && normalWorld.z == 0.0f) {
            return false;
        }

        constexpr float kFrameAxisLengthGameUnits = 12.0f;
        out.normalEndWorld = out.contactPointWorld + normalWorld * kFrameAxisLengthGameUnits;

        return true;
    }

    bool Hand::getGrabContactPatchDebugSnapshot(RE::hknpWorld* world, GrabContactPatchDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.hasContactPatch || _grabFrame.contactPatchSampleCount == 0 || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform grabBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
            return false;
        }
        const std::uint32_t count = (std::min)(_grabFrame.contactPatchSampleCount, static_cast<std::uint32_t>(out.samplePointsWorld.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            out.samplePointsWorld[i] = transform_math::localPointToWorld(grabBodyWorld, _grabFrame.contactPatchSamples[i].point);
        }
        out.sampleCount = count;
        return count > 0;
    }

    bool Hand::getGrabSupportFrameDebugSnapshot(RE::hknpWorld* world, GrabSupportFrameDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.authority.hasFrozenPivotB || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform liveBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, liveBodyWorld)) {
            return false;
        }

        out.pivotWorld = transform_math::localPointToWorld(liveBodyWorld, activeProxyConstraintPivotBLocalGame());
        const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabFrame.authority.bodyLocal);

        RE::NiPoint3 normalWorld = _grabFrame.support.hasNormal ?
            normalizeOrZero(transform_math::localVectorToWorld(liveBodyWorld, _grabFrame.support.normalBodyLocal)) :
            RE::NiPoint3{};
        if (vector_math::lengthSquared(normalWorld) <= 0.000001f && _grabFrame.gripEvidence.hasGripPoint) {
            normalWorld = gripEvidenceNormalWorld(_grabFrame, currentNodeWorld);
        }
        if (vector_math::lengthSquared(normalWorld) <= 0.000001f) {
            return false;
        }

        RE::NiPoint3 axisWorld = _grabFrame.support.hasAxis ?
            normalizeOrZero(transform_math::localVectorToWorld(liveBodyWorld, _grabFrame.support.axisBodyLocal)) :
            RE::NiPoint3{};
        if (vector_math::lengthSquared(axisWorld) <= 0.000001f) {
            axisWorld = stablePerpendicularAxis(normalWorld);
        }

        RE::NiPoint3 binormalWorld = _grabFrame.support.hasBinormal ?
            normalizeOrZero(transform_math::localVectorToWorld(liveBodyWorld, _grabFrame.support.binormalBodyLocal)) :
            RE::NiPoint3{};
        if (vector_math::lengthSquared(binormalWorld) <= 0.000001f) {
            binormalWorld = normalizeOrZero(vector_math::cross(normalWorld, axisWorld));
        }

        const float supportHalfSpan = std::isfinite(_grabFrame.support.spanGameUnits) && _grabFrame.support.spanGameUnits > 0.0f ?
            _grabFrame.support.spanGameUnits * 0.5f :
            0.0f;
        const float axisLength = std::clamp((std::max)(supportHalfSpan, 10.0f), 6.0f, 24.0f);
        out.axisLengthGameUnits = axisLength;
        out.normalEndWorld = out.pivotWorld + normalWorld * axisLength;
        out.supportAxisEndWorld = out.pivotWorld + axisWorld * axisLength;
        out.binormalEndWorld = out.pivotWorld + binormalWorld * axisLength;
        out.hasNormal = true;
        out.hasSupportAxis = vector_math::lengthSquared(axisWorld) > 0.000001f;
        out.hasBinormal = vector_math::lengthSquared(binormalWorld) > 0.000001f;
        out.pivotAuthoritySource = grab_authority_frame_math::grabAuthorityPivotSourceName(
            _grabFrame.pivotAuthority.source);
        out.activeGrabPointMode = _grabFrame.seat.activeGrabPointMode ? _grabFrame.seat.activeGrabPointMode : "none";
        out.supportKind = grab_support_model_math::gripSupportKindName(_grabFrame.support.kind);
        out.supportReason = _grabFrame.support.reason ? _grabFrame.support.reason : "none";
        out.authoredSupportPivot = _grabFrame.support.authoredPivot;
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

    bool Hand::getGrabForceTorqueDebugSnapshot(RE::hknpWorld* world, const RE::NiTransform& rawHandWorld, GrabForceTorqueDebugSnapshot& out) const
    {
        /*
         * This view is intentionally built from the same BODY-local pivot and
         * generated/proxy authority relation used by the active ragdoll-motor grab. Its
         * job is to show whether the solver is being asked to pull an off-center
         * frozen pivot toward the hand, which can rotate a body even when the
         * selected mesh/contact point looked reasonable at acquisition time.
         */
        out = {};

        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID || !_grabFrame.authority.hasFrozenPivotB) {
            return false;
        }

        RE::NiTransform liveBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, liveBodyWorld)) {
            return false;
        }

        RE::NiTransform proxyWorld{};
        const char* proxySource = "none";
        bool proxyFrameOk = resolveGrabAuthorityProxyFrame(
            world,
            rawHandWorld,
            nullptr,
            proxyWorld,
            proxySource,
            GrabAuthorityProxyFramePolicy::LivePalmOnly);
        if (!proxyFrameOk && _grabAuthorityProxy.isValid() && _grabAuthorityProxy.getBodyId().value != INVALID_BODY_ID) {
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
            const RE::NiPoint3 pivotBeforeFreeze =
                transform_math::localPointToWorld(_grabFrame.authority.bodyWorldAtGrab, _grabFrame.authority.pivotBConstraintLocalGame);
            const RE::NiPoint3 pivotAfterFreeze =
                transform_math::localPointToWorld(_grabFrame.authority.desiredBodyWorldAtGrab, _grabFrame.authority.pivotBConstraintLocalGame);
            const RE::NiPoint3 freezeShift =
                _grabFrame.authority.desiredBodyWorldAtGrab.translate - _grabFrame.authority.bodyWorldAtGrab.translate;
            const RE::NiPoint3 pivotGap =
                _grabFrame.authority.grabPivotWorldAtGrab - _grabFrame.gripEvidence.gripPointWorldAtGrab;
            const float freezeShiftLen = vectorMagnitude(freezeShift);
            const float pivotGapLen = vectorMagnitude(pivotGap);
            out.captureFreezeBodyShiftGameUnits =
                translationDeltaGameUnits(_grabFrame.authority.desiredBodyWorldAtGrab, _grabFrame.authority.bodyWorldAtGrab);
            out.captureFreezeBodyRotationDegrees =
                rotationDeltaDegrees(_grabFrame.authority.desiredBodyWorldAtGrab.rotate, _grabFrame.authority.bodyWorldAtGrab.rotate);
            out.captureFreezePivotGapBeforeGameUnits =
                pointDistanceGameUnits(pivotBeforeFreeze, _grabFrame.authority.grabPivotWorldAtGrab);
            out.captureFreezePivotGapAfterGameUnits =
                pointDistanceGameUnits(pivotAfterFreeze, _grabFrame.authority.grabPivotWorldAtGrab);
            out.captureFreezeShiftDot =
                freezeShiftLen > 0.0001f && pivotGapLen > 0.0001f ?
                    std::clamp(vector_math::dot(freezeShift, pivotGap) / (freezeShiftLen * pivotGapLen), -1.0f, 1.0f) :
                    0.0f;
            out.captureFreezePivotLeverGameUnits =
                pointDistanceGameUnits(_grabFrame.gripEvidence.gripPointWorldAtGrab, _grabFrame.authority.bodyWorldAtGrab.translate);
        }
        RE::NiPoint3 pivotBLocalGame = activeProxyConstraintPivotBLocalGame();
        RE::NiPoint3 atomTransformBLocalGame = pivotBLocalGame;
        bool hasAtomMotorFrames = false;

        if (_activeConstraint.constraintData && _grabAuthorityProxyFrameValid) {
            const auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
            const auto* transformARotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_A_COL0);
            const auto* transformBRotation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_COL0);
            const auto* transformBTranslation = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
            const auto* targetBRca = reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
            atomTransformBLocalGame = RE::NiPoint3{
                transformBTranslation[0] * havokToGameScale(),
                transformBTranslation[1] * havokToGameScale(),
                transformBTranslation[2] * havokToGameScale(),
            };

            const RE::NiPoint3 pivotAProxyLocalGame = _grabAuthorityPivotAProxyLocalGame;
            if (std::isfinite(atomTransformBLocalGame.x) &&
                std::isfinite(atomTransformBLocalGame.y) &&
                std::isfinite(atomTransformBLocalGame.z) &&
                std::isfinite(pivotAProxyLocalGame.x) &&
                std::isfinite(pivotAProxyLocalGame.y) &&
                std::isfinite(pivotAProxyLocalGame.z)) {
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
                        proxyInBodyBeforeTargetWrite,
                        pivotAProxyLocalGame);

                out.motorRelationInputBodyWorld =
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, bodyInProxyBeforeInversion);
                out.motorRelationInverseBodyWorld = reconstructBodyWorldFromProxyInBody(
                    proxyWorld,
                    proxyInBodyBeforeTargetWrite.rotate,
                    relationTransformBLocalGame,
                    pivotAProxyLocalGame);
                out.motorRelationPivotWorld =
                    transform_math::localPointToWorld(out.motorRelationInverseBodyWorld, relationTransformBLocalGame);

                out.motorAtomTargetBodyWorld = reconstructBodyWorldFromProxyInBody(
                    proxyWorld,
                    targetAsHkRows,
                    atomTransformBLocalGame,
                    pivotAProxyLocalGame);
                out.motorSolverEffectiveBodyWorld = reconstructSolverEffectiveBodyWorld(
                    proxyWorld,
                    transformAAsHkColumns,
                    transformBAsHkColumns,
                    targetAsHkRows,
                    atomTransformBLocalGame,
                    out.motorAnchorAWorld,
                    desiredBodyWorld.scale);
                RE::NiTransform liveProxyWorld{};
                const bool liveProxyOk =
                    _grabAuthorityProxy.isValid() &&
                    tryGetGrabAuthorityBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), liveProxyWorld);
                if (liveProxyOk) {
                    const RE::NiPoint3 liveAnchorAWorld =
                        generatedProxyLocalPointToWorld(liveProxyWorld, pivotAProxyLocalGame);
                    const RE::NiTransform solverEffectiveLiveAWorld = reconstructSolverEffectiveBodyWorld(
                        liveProxyWorld,
                        transformAAsHkColumns,
                        transformBAsHkColumns,
                        targetAsHkRows,
                        atomTransformBLocalGame,
                        liveAnchorAWorld,
                        desiredBodyWorld.scale);
                    out.motorSolverEffectiveLiveABodyDeltaDegrees =
                        rotationDeltaDegrees(solverEffectiveLiveAWorld.rotate, desiredBodyWorld.rotate);
                    out.motorSolverEffectiveTargetALiveADeltaDegrees =
                        rotationDeltaDegrees(out.motorSolverEffectiveBodyWorld.rotate, solverEffectiveLiveAWorld.rotate);
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

                const RE::NiPoint3 angularAxis = rotationCorrectionAxisWorld(liveBodyWorld.rotate, out.motorAtomTargetBodyWorld.rotate);
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
        out.pivotAuthoritySource = grab_authority_frame_math::grabAuthorityPivotSourceName(
            _grabFrame.pivotAuthority.source);
        out.activeGrabPointMode = _grabFrame.seat.activeGrabPointMode ? _grabFrame.seat.activeGrabPointMode : "none";
        out.authorityFrameSource = proxySource ? proxySource : "none";
        out.acquisitionPhase = grab_three_phase::phaseName(_grabAcquisitionPhase);
        out.capturePivotAuthoritySource =
            grab_authority_frame_math::grabAuthorityPivotSourceName(
                _grabFrame.captureTelemetry.pivotAuthority.source);
        out.captureGrabPointMode =
            _grabFrame.captureTelemetry.seat.activeGrabPointMode ? _grabFrame.captureTelemetry.seat.activeGrabPointMode : "none";
        out.positionOnlyPivot = _grabFrame.pivotAuthority.positionOnly;
        out.normalTrusted = _grabFrame.pivotAuthority.normalTrusted;

        if (torqueWitnessLength > 0.001f) {
            const RE::NiPoint3 torqueAxis = torqueWitness * (1.0f / torqueWitnessLength);
            const float torqueAxisLength = std::clamp(std::sqrt(torqueWitnessLength), 6.0f, 28.0f);
            out.torqueAxisEndWorld = liveBodyWorld.translate + torqueAxis * torqueAxisLength;
            out.hasTorqueAxis = true;
        }

        const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabFrame.authority.bodyLocal);
        /*
         * Active pivot-B markers answer a different question than mesh evidence:
         * mesh points show what ROCK selected visually, while this converts the
         * exact BODY-local pivot consumed by the constraint back through the
         * rendered node. If this visual equivalent separates from the live BODY
         * pivot, the solver and rendered mesh disagree even when the blue mesh
         * selection point itself looks correct.
         */
        if (_grabFrame.heldNode) {
            const RE::NiPoint3 activePivotBNodeLocal = transform_math::worldPointToLocal(currentNodeWorld, livePivotWorld);
            out.activePivotBVisualNodeWorld = transform_math::localPointToWorld(_grabFrame.heldNode->world, activePivotBNodeLocal);
            out.activePivotBVisualLockErrorGameUnits = pointDistanceGameUnits(livePivotWorld, out.activePivotBVisualNodeWorld);
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
                out.visualMeshGripPointWorld = transform_math::localPointToWorld(_grabFrame.heldNode->world, _grabFrame.gripEvidence.gripPointLocal);
                out.hasVisualMeshGripPoint = true;
                out.bodyVisualMeshLockErrorGameUnits = pointDistanceGameUnits(out.meshGripPointWorld, out.visualMeshGripPointWorld);
            }
        }

        if (_grabFrame.captureTelemetry.valid && _grabFrame.captureTelemetry.gripEvidence.hasGripPoint) {
            const auto& capture = _grabFrame.captureTelemetry;
            const RE::NiTransform captureNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, capture.authority.bodyLocal);
            if (capture.gripEvidence.hasGripSourceNodePoint) {
                out.captureMeshGripPointBodyWorld =
                    transform_math::localPointToWorld(gripEvidenceWorldFrame(capture, captureNodeWorld), capture.gripEvidence.gripPointSourceNodeLocal);
            } else {
                out.captureMeshGripPointBodyWorld = transform_math::localPointToWorld(captureNodeWorld, capture.gripEvidence.gripPointLocal);
            }
            out.hasCaptureMeshGripPoint = true;
            if (capture.gripEvidence.hasGripSourceNodePoint) {
                out.captureMeshGripPointVisualWorld = out.captureMeshGripPointBodyWorld;
            } else if (_grabFrame.heldNode) {
                out.captureMeshGripPointVisualWorld = transform_math::localPointToWorld(_grabFrame.heldNode->world, capture.gripEvidence.gripPointLocal);
            } else {
                out.captureMeshGripPointVisualWorld = out.captureMeshGripPointBodyWorld;
            }

            const RE::NiPoint3 localDelta = _grabFrame.gripEvidence.gripPointLocal - capture.gripEvidence.gripPointLocal;
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

        if (_grabFrame.hasContactPatch && _grabFrame.contactPatchSampleCount > 0) {
            const std::uint32_t count = (std::min)(_grabFrame.contactPatchSampleCount, static_cast<std::uint32_t>(out.contactSamplePointsWorld.size()));
            RE::NiPoint3 average{};
            for (std::uint32_t i = 0; i < count; ++i) {
                out.contactSamplePointsWorld[i] = transform_math::localPointToWorld(liveBodyWorld, _grabFrame.contactPatchSamples[i].point);
                average = average + out.contactSamplePointsWorld[i];
            }
            out.contactSampleCount = count;
            if (count > 0) {
                out.contactPatchPointWorld = average * (1.0f / static_cast<float>(count));
                out.hasContactPatchPoint = true;
            }
        }

        return true;
    }

}

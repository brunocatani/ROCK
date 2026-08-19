#include "physics-interaction/hand/Hand.h"

/*
 * Read-only debug snapshot readers for the overlay and the logs. Nothing here
 * changes grab state, and nothing here may become load-bearing for gameplay.
 *
 * These run on the game thread while the physics thread can be flushing, so the
 * proxy-owned reads take _grabAuthorityProxyMutex. Every getter fails CLOSED: an
 * unavailable snapshot returns false rather than stale values.
 */

#include "physics-interaction/hand/grab/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/grab/HandGrabFingerPose.h"
#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/SceneWriterProbe.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace rock
{
    using namespace hand_grab_detail;

    bool Hand::tryGetHeldObjectGrabPivotWorld(RE::hknpWorld* world, RE::NiPoint3& outPivotWorld) const
    {
        outPivotWorld = {};
    
        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }
    
        RE::NiTransform objectBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, objectBodyWorld)) {
            return false;
        }
    
        const RE::NiPoint3 pivotBLocal = activeProxyConstraintPivotBLocalGame();
        outPivotWorld = transform_math::localPointToWorld(objectBodyWorld, pivotBLocal);
        return std::isfinite(outPivotWorld.x) && std::isfinite(outPivotWorld.y) && std::isfinite(outPivotWorld.z);
    }
    
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
    
    bool Hand::getGrabPocketNormalDebugSnapshot(RE::hknpWorld* world, GrabPocketNormalDebugSnapshot& out) const
    {
        out = {};
    
        if (!world || !isHolding() || !_grabFrame.hasGripPoint || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }
    
        RE::NiTransform grabBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
            return false;
        }
        const RE::NiTransform currentNodeWorld = heldNodeWorldFromBodyWorld(grabBodyWorld);
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
    
        if (!world || !isHolding() || !_grabFrame.hasFrozenPivotB || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }
    
        RE::NiTransform liveBodyWorld{};
        if (!tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, liveBodyWorld)) {
            return false;
        }
    
        out.pivotWorld = transform_math::localPointToWorld(liveBodyWorld, activeProxyConstraintPivotBLocalGame());
        const RE::NiTransform currentNodeWorld = heldNodeWorldFromBodyWorld(liveBodyWorld);
    
        RE::NiPoint3 normalWorld = _grabFrame.hasSupportFrameNormal ?
            normalizeOrZero(transform_math::localVectorToWorld(liveBodyWorld, _grabFrame.supportFrameNormalBodyLocal)) :
            RE::NiPoint3{};
        if (lengthSquared(normalWorld) <= 0.000001f && _grabFrame.hasGripPoint) {
            normalWorld = gripEvidenceNormalWorld(_grabFrame, currentNodeWorld);
        }
        if (lengthSquared(normalWorld) <= 0.000001f) {
            return false;
        }
    
        RE::NiPoint3 axisWorld = _grabFrame.hasSupportFrameAxis ?
            normalizeOrZero(transform_math::localVectorToWorld(liveBodyWorld, _grabFrame.supportFrameAxisBodyLocal)) :
            RE::NiPoint3{};
        if (lengthSquared(axisWorld) <= 0.000001f) {
            axisWorld = stablePerpendicularAxis(normalWorld);
        }
    
        RE::NiPoint3 binormalWorld = _grabFrame.hasSupportFrameBinormal ?
            normalizeOrZero(transform_math::localVectorToWorld(liveBodyWorld, _grabFrame.supportFrameBinormalBodyLocal)) :
            RE::NiPoint3{};
        if (lengthSquared(binormalWorld) <= 0.000001f) {
            binormalWorld = normalizeOrZero(crossProduct(normalWorld, axisWorld));
        }
    
        const float supportHalfSpan = std::isfinite(_grabFrame.gripSupportSpanGameUnits) && _grabFrame.gripSupportSpanGameUnits > 0.0f ?
            _grabFrame.gripSupportSpanGameUnits * 0.5f :
            0.0f;
        const float axisLength = std::clamp((std::max)(supportHalfSpan, 10.0f), 6.0f, 24.0f);
        out.axisLengthGameUnits = axisLength;
        out.normalEndWorld = out.pivotWorld + normalWorld * axisLength;
        out.supportAxisEndWorld = out.pivotWorld + axisWorld * axisLength;
        out.binormalEndWorld = out.pivotWorld + binormalWorld * axisLength;
        out.hasNormal = true;
        out.hasSupportAxis = lengthSquared(axisWorld) > 0.000001f;
        out.hasBinormal = lengthSquared(binormalWorld) > 0.000001f;
        out.pivotAuthoritySource = _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none";
        out.activeGrabPointMode = _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none";
        out.supportKind = grab_support_model_math::gripSupportKindName(_grabFrame.gripSupportKind);
        out.supportReason = _grabFrame.gripSupportReason ? _grabFrame.gripSupportReason : "none";
        out.authoredSupportPivot = _grabFrame.gripSupportAuthoredPivot;
        out.positionOnlyPivot = _grabFrame.pivotAuthorityPositionOnly;
        out.normalTrusted = _grabFrame.pivotAuthorityNormalTrusted;
    
        if (_grabFrame.gripEvidenceTriangleIndex < _grabFrame.localMeshTriangles.size()) {
            const auto& triangle = _grabFrame.localMeshTriangles[_grabFrame.gripEvidenceTriangleIndex];
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
    
        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID || !_grabFrame.hasFrozenPivotB) {
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
            grab_authority_frame_math::isFiniteTransform(_grabFrame.bodyWorldAtGrab) &&
            grab_authority_frame_math::isFiniteTransform(_grabFrame.desiredBodyWorldAtGrab) &&
            grab_authority_frame_math::isFiniteVector(_grabFrame.grabPivotWorldAtGrab) &&
            grab_authority_frame_math::isFiniteVector(_grabFrame.gripPointWorldAtGrab) &&
            grab_authority_frame_math::isFiniteVector(_grabFrame.pivotBConstraintLocalGame)) {
            const RE::NiPoint3 pivotBeforeFreeze =
                transform_math::localPointToWorld(_grabFrame.bodyWorldAtGrab, _grabFrame.pivotBConstraintLocalGame);
            const RE::NiPoint3 pivotAfterFreeze =
                transform_math::localPointToWorld(_grabFrame.desiredBodyWorldAtGrab, _grabFrame.pivotBConstraintLocalGame);
            const RE::NiPoint3 freezeShift =
                _grabFrame.desiredBodyWorldAtGrab.translate - _grabFrame.bodyWorldAtGrab.translate;
            const RE::NiPoint3 pivotGap =
                _grabFrame.grabPivotWorldAtGrab - _grabFrame.gripPointWorldAtGrab;
            const float freezeShiftLen = vectorMagnitude(freezeShift);
            const float pivotGapLen = vectorMagnitude(pivotGap);
            out.captureFreezeBodyShiftGameUnits =
                translationDeltaGameUnits(_grabFrame.desiredBodyWorldAtGrab, _grabFrame.bodyWorldAtGrab);
            out.captureFreezeBodyRotationDegrees =
                rotationDeltaDegrees(_grabFrame.desiredBodyWorldAtGrab.rotate, _grabFrame.bodyWorldAtGrab.rotate);
            out.captureFreezePivotGapBeforeGameUnits =
                pointDistanceGameUnits(pivotBeforeFreeze, _grabFrame.grabPivotWorldAtGrab);
            out.captureFreezePivotGapAfterGameUnits =
                pointDistanceGameUnits(pivotAfterFreeze, _grabFrame.grabPivotWorldAtGrab);
            out.captureFreezeShiftDot =
                freezeShiftLen > 0.0001f && pivotGapLen > 0.0001f ?
                    std::clamp(dotProduct(freezeShift, pivotGap) / (freezeShiftLen * pivotGapLen), -1.0f, 1.0f) :
                    0.0f;
            out.captureFreezePivotLeverGameUnits =
                pointDistanceGameUnits(_grabFrame.gripPointWorldAtGrab, _grabFrame.bodyWorldAtGrab.translate);
        }
        RE::NiPoint3 pivotBLocalGame = activeProxyConstraintPivotBLocalGame();
        RE::NiPoint3 atomTransformBLocalGame = pivotBLocalGame;
        bool hasAtomMotorFrames = false;
    
        if (_activeConstraint.constraintData && _grabAuthorityProxyFrameValid) {
            const auto atoms = decodeGrabConstraintAtoms(_activeConstraint.constraintData);
            atomTransformBLocalGame = atoms.transformBTranslationGame;
    
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
    
                RE::NiTransform transformBLocal = makeIdentityTransform();
                transformBLocal.rotate = atoms.transformBColumns;
                transformBLocal.translate = atomTransformBLocalGame;
                out.motorConstraintBWorld = transform_math::composeTransforms(liveBodyWorld, transformBLocal);
    
                const RE::NiTransform bodyInProxyBeforeInversion =
                    grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorld, desiredBodyWorld);
                const auto relation = buildGrabConstraintRelationDiagnostics(
                    atoms,
                    proxyWorld,
                    bodyInProxyBeforeInversion,
                    pivotAProxyLocalGame);
    
                out.motorRelationInputBodyWorld =
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, bodyInProxyBeforeInversion);
                out.motorRelationInverseBodyWorld = relation.relationInverseBodyWorld;
                out.motorRelationPivotWorld =
                    transform_math::localPointToWorld(out.motorRelationInverseBodyWorld, relation.relationTransformBLocalGame);
    
                out.motorAtomTargetBodyWorld = relation.atomRowsBodyWorld;
                out.motorSolverEffectiveBodyWorld = reconstructSolverEffectiveBodyWorld(
                    proxyWorld,
                    atoms.transformAColumns,
                    atoms.transformBColumns,
                    atoms.targetRows,
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
                        atoms.transformAColumns,
                        atoms.transformBColumns,
                        atoms.targetRows,
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
                const bool hasMatchingPhysicsProbe =
                    _ragdollAngularProbePreSolve.valid &&
                    _ragdollAngularProbePreSolve.objectBodyId.value == _savedObjectState.bodyId.value;
                if (hasMatchingPhysicsProbe) {
                    out.motorPhysicsProxyToLiveProxyDeltaGameUnits =
                        _ragdollAngularProbePreSolve.targetProxyToLiveProxyDeltaGameUnits;
                    out.motorPhysicsProxyToLiveProxyDeltaDegrees =
                        _ragdollAngularProbePreSolve.targetProxyToLiveProxyDeltaDegrees;
                    out.motorRelationToConstraintATargetDegrees =
                        _ragdollAngularProbePreSolve.targetRelationAToTargetConstraintADegrees;
                    out.motorRelationToConstraintALiveDegrees =
                        _ragdollAngularProbePreSolve.liveRelationAToLiveConstraintADegrees;
                    out.motorTransformARawMaxDelta = _ragdollAngularProbePreSolve.transformARawMaxDelta;
                    out.motorTransformBRawMaxDelta = _ragdollAngularProbePreSolve.transformBRawMaxDelta;
                    out.motorTargetBRcaRawMaxDelta = _ragdollAngularProbePreSolve.targetBRcaRawMaxDelta;
                } else {
                    out.motorPhysicsProxyToLiveProxyDeltaGameUnits = -1.0f;
                    out.motorPhysicsProxyToLiveProxyDeltaDegrees = -1.0f;
                    out.motorRelationToConstraintATargetDegrees = -1.0f;
                    out.motorRelationToConstraintALiveDegrees = -1.0f;
                    out.motorTransformARawMaxDelta = -1.0f;
                    out.motorTransformBRawMaxDelta = -1.0f;
                    out.motorTargetBRcaRawMaxDelta = -1.0f;
                }
                out.motorTransformBRelationLocalDeltaGameUnits =
                    relation.transformBRelationDeltaGameUnits;
                out.motorTransformBPivotToAnchorAGameUnits =
                    pointDistanceGameUnits(out.motorAtomTargetPivotWorld, out.motorAnchorAWorld);
                out.motorTargetBodyDeltaEndWorld = out.motorAtomTargetBodyWorld.translate;
                out.hasMotorConstraintFrames = true;
                out.hasMotorRelationFrames = true;
                out.hasMotorSolverEffectiveBody = true;
                out.hasMotorTargetBodyDelta =
                    out.motorTargetBodyDeltaGameUnits > 0.01f || out.motorTargetBodyDeltaDegrees > 0.01f;
    
                const RE::NiPoint3 angularAxis = rotationCorrectionAxisWorld(liveBodyWorld.rotate, out.motorAtomTargetBodyWorld.rotate);
                if (lengthSquared(angularAxis) > 0.000001f) {
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
        const RE::NiPoint3 torqueWitness = crossProduct(lever, correction);
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
        out.pocketDistanceGameUnits = _grabFrame.pocketToGripDistanceGameUnits;
        out.selectionDistanceGameUnits = _grabFrame.selectionToGripEvidenceDistanceGameUnits;
        out.longLeverGameUnits = _grabFrame.longObjectLeverGameUnits;
        out.positionConfidence = _grabFrame.pivotAuthorityPositionConfidence;
        out.pivotAuthoritySource = _grabFrame.pivotAuthoritySource ? _grabFrame.pivotAuthoritySource : "none";
        out.activeGrabPointMode = _grabFrame.activeGrabPointMode ? _grabFrame.activeGrabPointMode : "none";
        out.authorityFrameSource = proxySource ? proxySource : "none";
        out.acquisitionPhase = grab_three_phase::phaseName(_grabAcquisitionPhase);
        out.capturePivotAuthoritySource =
            _grabFrame.captureTelemetry.pivotAuthoritySource ? _grabFrame.captureTelemetry.pivotAuthoritySource : "none";
        out.captureGrabPointMode =
            _grabFrame.captureTelemetry.activeGrabPointMode ? _grabFrame.captureTelemetry.activeGrabPointMode : "none";
        out.lastSeatedPivotReacquireReason =
            _grabFrame.lastSeatedPivotReacquireReason ? _grabFrame.lastSeatedPivotReacquireReason : "none";
        out.seatedPivotReacquireCount = _grabFrame.seatedPivotReacquireCount;
        out.positionOnlyPivot = _grabFrame.pivotAuthorityPositionOnly;
        out.normalTrusted = _grabFrame.pivotAuthorityNormalTrusted;
    
        if (torqueWitnessLength > 0.001f) {
            const RE::NiPoint3 torqueAxis = torqueWitness * (1.0f / torqueWitnessLength);
            const float torqueAxisLength = std::clamp(std::sqrt(torqueWitnessLength), 6.0f, 28.0f);
            out.torqueAxisEndWorld = liveBodyWorld.translate + torqueAxis * torqueAxisLength;
            out.hasTorqueAxis = true;
        }
    
        const RE::NiTransform currentNodeWorld = heldNodeWorldFromBodyWorld(liveBodyWorld);
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
    
        if (_grabFrame.hasGripPoint) {
            out.meshGripPointWorld = gripEvidencePointWorld(_grabFrame, currentNodeWorld);
            out.hasMeshGripPoint = true;
            if (_grabFrame.gripSourceNode || _grabFrame.hasGripSourceNodePoint) {
                out.visualMeshGripPointWorld = out.meshGripPointWorld;
                out.hasVisualMeshGripPoint = true;
                out.bodyVisualMeshLockErrorGameUnits = pointDistanceGameUnits(livePivotWorld, out.meshGripPointWorld);
            } else if (_grabFrame.heldNode) {
                out.visualMeshGripPointWorld = transform_math::localPointToWorld(_grabFrame.heldNode->world, _grabFrame.gripPointLocal);
                out.hasVisualMeshGripPoint = true;
                out.bodyVisualMeshLockErrorGameUnits = pointDistanceGameUnits(out.meshGripPointWorld, out.visualMeshGripPointWorld);
            }
        }
    
        if (_grabFrame.captureTelemetry.valid && _grabFrame.captureTelemetry.hasGripPoint) {
            const auto& capture = _grabFrame.captureTelemetry;
            const RE::NiTransform captureNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, capture.bodyLocal);
            if (capture.hasGripSourceNodePoint) {
                out.captureMeshGripPointBodyWorld =
                    transform_math::localPointToWorld(gripEvidenceWorldFrame(capture, captureNodeWorld), capture.gripPointSourceNodeLocal);
            } else {
                out.captureMeshGripPointBodyWorld = transform_math::localPointToWorld(captureNodeWorld, capture.gripPointLocal);
            }
            out.hasCaptureMeshGripPoint = true;
            if (capture.hasGripSourceNodePoint) {
                out.captureMeshGripPointVisualWorld = out.captureMeshGripPointBodyWorld;
            } else if (_grabFrame.heldNode) {
                out.captureMeshGripPointVisualWorld = transform_math::localPointToWorld(_grabFrame.heldNode->world, capture.gripPointLocal);
            } else {
                out.captureMeshGripPointVisualWorld = out.captureMeshGripPointBodyWorld;
            }
    
            const RE::NiPoint3 localDelta = _grabFrame.gripPointLocal - capture.gripPointLocal;
            const float scale =
                std::isfinite(currentNodeWorld.scale) && currentNodeWorld.scale > 0.0f ? currentNodeWorld.scale : 1.0f;
            out.captureGripLocalDeltaGameUnits = vectorMagnitude(localDelta) * scale;
            out.gripPointMutatedAfterCapture = out.captureGripLocalDeltaGameUnits > 0.001f;
        }
    
        if (_grabFrame.gripEvidenceTriangleIndex < _grabFrame.localMeshTriangles.size()) {
            const auto& triangle = _grabFrame.localMeshTriangles[_grabFrame.gripEvidenceTriangleIndex];
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
    
    bool Hand::getGrabTransformTelemetrySnapshot(RE::hknpWorld* world,
        const RE::NiTransform& rawHandWorld,
        grab_transform_telemetry::RuntimeSample& out) const
    {
        out = {};
        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }
    
        out.valid = true;
        out.isLeft = _isLeft;
        out.rawHandWorld = rawHandWorld;
        out.nativeFlattenedHandWorld = rawHandWorld;
        out.rawHandBasis = grab_transform_telemetry::makeOrientationBasis(out.rawHandWorld);
        out.nativeFlattenedHandBasis = out.rawHandBasis;
        out.heldFormId = _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0;
        out.heldBodyId = _savedObjectState.bodyId.value;
        out.handBodyId = _handBody.isValid() ? _handBody.getBodyId().value : INVALID_BODY_ID;
        out.legacyPalmPivotAWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(out.nativeFlattenedHandWorld, _isLeft);
        out.hasLegacyPalmPivotAWorld = true;
        out.runtimePivotSource = "rawHandOriginFallback";
    
        LivePalmAnchorReference palmReference{};
        if (tryResolveLivePalmAnchorReference(world, palmReference)) {
            out.runtimePivotSource = palmReference.source == body_frame::BodyFrameSource::MotionCenterOfMass ?
                                         "livePalmAnchorMotion" :
                                         "livePalmAnchorBody";
            out.handBodyWorld = palmReference.world;
            out.handBodyBasis = grab_transform_telemetry::makeOrientationBasis(out.handBodyWorld);
            out.handBodySource = palmReference.source;
            out.handMotionIndex = palmReference.motionIndex;
            out.hasHandBodyWorld = true;
            out.rawToHandBody = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.handBodyWorld);
            const RE::NiTransform palmAnchorGrabAuthorityBase =
                hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(out.handBodyWorld);
            out.palmAnchorGrabAuthorityWorld =
                applyGrabAuthorityProxyLocalOffsetToFrame(palmAnchorGrabAuthorityBase, _isLeft);
            out.palmAnchorGrabAuthorityBasis = grab_transform_telemetry::makeOrientationBasis(out.palmAnchorGrabAuthorityWorld);
            out.hasPalmAnchorGrabAuthority = true;
            out.nativeFlattenedHandToGrabAuthority =
                grab_transform_telemetry::measureTransformDelta(out.nativeFlattenedHandWorld, out.palmAnchorGrabAuthorityWorld);
            out.legacyPalmPivotAToGrabAuthority =
                grab_transform_telemetry::measurePointPair(out.legacyPalmPivotAWorld, out.palmAnchorGrabAuthorityWorld.translate);
        }
    
        if (_boneColliders.tryGetPalmAnchorTarget(out.palmAnchorTargetWorld)) {
            out.hasPalmAnchorTarget = true;
            out.palmAnchorTargetBasis = grab_transform_telemetry::makeOrientationBasis(out.palmAnchorTargetWorld);
            out.rawToPalmAnchorTarget =
                grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.palmAnchorTargetWorld);
            out.nativeFlattenedHandToPalmAnchorTarget =
                grab_transform_telemetry::measureTransformDelta(out.nativeFlattenedHandWorld, out.palmAnchorTargetWorld);
            out.legacyPalmPivotAToPalmAnchor =
                grab_transform_telemetry::measurePointPair(out.legacyPalmPivotAWorld, out.palmAnchorTargetWorld.translate);
            if (out.hasPalmAnchorGrabAuthority) {
                out.palmAnchorTargetToGrabAuthority =
                    grab_transform_telemetry::measureTransformDelta(out.palmAnchorTargetWorld, out.palmAnchorGrabAuthorityWorld);
            }
        }
    
        root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
        if (root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, fingerSnapshot)) {
            RE::NiPoint3 baseSum{};
            RE::NiPoint3 tipSum{};
            std::uint32_t validFingers = 0;
            for (const auto& finger : fingerSnapshot.fingers) {
                if (!finger.valid) {
                    continue;
                }
                baseSum = baseSum + finger.points[0];
                tipSum = tipSum + finger.points[2];
                ++validFingers;
            }
    
            if (validFingers > 0) {
                const float inv = 1.0f / static_cast<float>(validFingers);
                out.rootFingerBaseCenterWorld = baseSum * inv;
                out.rootFingerTipCenterWorld = tipSum * inv;
                out.rootFingerBaseLineWorld =
                    grab_transform_telemetry::normalizeDirectionOrFallback(out.rootFingerBaseCenterWorld - out.rawHandWorld.translate, out.rawHandBasis.x);
                out.handBodyFingerBaseLineWorld =
                    out.hasHandBodyWorld ?
                    grab_transform_telemetry::normalizeDirectionOrFallback(out.rootFingerBaseCenterWorld - out.handBodyWorld.translate, out.handBodyBasis.x) :
                    out.rootFingerBaseLineWorld;
                out.palmAnchorFingerBaseLineWorld =
                    out.hasPalmAnchorTarget ?
                    grab_transform_telemetry::normalizeDirectionOrFallback(out.rootFingerBaseCenterWorld - out.palmAnchorTargetWorld.translate, out.palmAnchorTargetBasis.x) :
                    out.rootFingerBaseLineWorld;
                out.rootFingerOpenLineWorld =
                    grab_transform_telemetry::normalizeDirectionOrFallback(out.rootFingerTipCenterWorld - out.rootFingerBaseCenterWorld, out.rootFingerBaseLineWorld);
                out.rootPalmNormalWorld =
                    grab_transform_telemetry::normalizeDirectionOrFallback(fingerSnapshot.palmNormalWorld, out.rawHandBasis.z);
                out.hasRootFingerLandmarks = true;
            }
        }
    
        if (_grabAuthorityProxy.isValid() && _grabAuthorityProxy.getBodyId().value != INVALID_BODY_ID) {
            RE::NiTransform proxyWorld{};
            if (tryResolveLiveBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), proxyWorld)) {
                out.proxyReadbackWorld = proxyWorld;
                out.proxyReadbackBasis = grab_transform_telemetry::makeOrientationBasis(out.proxyReadbackWorld);
                out.hasProxyReadback = true;
                if (out.hasPalmAnchorGrabAuthority) {
                    out.grabAuthorityToProxyReadback =
                        grab_transform_telemetry::measureTransformDelta(out.palmAnchorGrabAuthorityWorld, out.proxyReadbackWorld);
                }
                out.nativeFlattenedHandToProxyReadback =
                    grab_transform_telemetry::measureTransformDelta(out.nativeFlattenedHandWorld, out.proxyReadbackWorld);
                out.legacyPalmPivotAToProxyReadback =
                    grab_transform_telemetry::measurePointPair(out.legacyPalmPivotAWorld, out.proxyReadbackWorld.translate);
            }
        }
    
        const auto heldBodyFrame = resolveLiveBodyWorldTransform(world, _savedObjectState.bodyId);
        out.heldBodyWorld = heldBodyFrame.transform;
        out.heldBodyBasis = grab_transform_telemetry::makeOrientationBasis(out.heldBodyWorld);
        out.heldBodySource = heldBodyFrame.source;
        out.heldMotionIndex = heldBodyFrame.motionIndex;
        out.hasHeldBodyWorld = heldBodyFrame.valid;
        out.heldBodyMass = readHeldBodyMassSummary(
            world,
            _savedObjectState.bodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedMass).motorMass();
    
        RE::NiTransform heldNativeBodyWorld{};
        if (tryGetBodyArrayWorldTransform(world, _savedObjectState.bodyId, heldNativeBodyWorld)) {
            out.heldNativeBodyWorld = heldNativeBodyWorld;
            out.heldNativeBodyBasis = grab_transform_telemetry::makeOrientationBasis(out.heldNativeBodyWorld);
            out.hasHeldNativeBodyWorld = true;
            if (heldBodyFrame.valid) {
                out.heldNativeBodyToHeldBody = grab_transform_telemetry::measureTransformDelta(out.heldNativeBodyWorld, out.heldBodyWorld);
            }
        }
    
        if (heldBodyFrame.valid || out.hasHeldNativeBodyWorld) {
            const RE::NiTransform& bodyLocalAuthorityFrame = out.hasHeldNativeBodyWorld ? out.heldNativeBodyWorld : out.heldBodyWorld;
            out.heldBodyDerivedNodeWorld = heldNodeWorldFromBodyWorld(bodyLocalAuthorityFrame);
            out.heldBodyDerivedNodeBasis = grab_transform_telemetry::makeOrientationBasis(out.heldBodyDerivedNodeWorld);
            out.hasHeldBodyDerivedNodeWorld = true;
        }
    
        /*
         * Telemetry keeps the same split as runtime authority: the held visual
         * node is the scene graph source, while body-derived node shows what the
         * native BODY authority predicts from the frozen frame. In a correct
         * native dynamic grab those should be nearly identical; MOTION is logged
         * separately as COM/weight diagnostics.
         */
        RE::NiAVObject* heldVisualNode = nullptr;
        if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted() && !_savedObjectState.refr->IsDisabled()) {
            heldVisualNode = _grabFrame.heldNode ? _grabFrame.heldNode : _savedObjectState.refr->Get3D();
        }
        if (heldVisualNode) {
            out.heldNodeWorld = heldVisualNode->world;
            out.heldNodeBasis = grab_transform_telemetry::makeOrientationBasis(out.heldNodeWorld);
            out.hasHeldNodeWorld = true;
        } else if (out.hasHeldBodyDerivedNodeWorld) {
            out.heldNodeWorld = out.heldBodyDerivedNodeWorld;
            out.heldNodeBasis = grab_transform_telemetry::makeOrientationBasis(out.heldNodeWorld);
            out.hasHeldNodeWorld = true;
        }
        if (out.hasHeldNodeWorld && out.hasHeldBodyDerivedNodeWorld) {
            out.bodyDerivedNodeToHeldNode = grab_transform_telemetry::measureTransformDelta(out.heldBodyDerivedNodeWorld, out.heldNodeWorld);
        }
        if (_grabFrame.hasTelemetryCapture) {
            out.hasGrabStartFrames = true;
            out.liveHandWorldAtGrab = _grabFrame.liveHandWorldAtGrab;
            out.handBodyWorldAtGrab = _grabFrame.handBodyWorldAtGrab;
            out.objectNodeWorldAtGrab = _grabFrame.objectNodeWorldAtGrab;
            out.desiredObjectWorldAtGrab = _grabFrame.desiredObjectWorldAtGrab;
            out.rawHandSpace = _grabFrame.rawHandSpace;
            out.handBodyToRawHandAtGrab = _grabFrame.handBodyToRawHandAtGrab;
            out.bodyLocal = _grabFrame.bodyLocal;
            out.liveHandWorldAtGrabBasis = grab_transform_telemetry::makeOrientationBasis(out.liveHandWorldAtGrab);
            out.handBodyWorldAtGrabBasis = grab_transform_telemetry::makeOrientationBasis(out.handBodyWorldAtGrab);
            out.objectNodeWorldAtGrabBasis = grab_transform_telemetry::makeOrientationBasis(out.objectNodeWorldAtGrab);
            out.desiredObjectWorldAtGrabBasis = grab_transform_telemetry::makeOrientationBasis(out.desiredObjectWorldAtGrab);
            out.rawHandSpaceBasis = grab_transform_telemetry::makeOrientationBasis(out.rawHandSpace);
            out.bodyLocalBasis = grab_transform_telemetry::makeOrientationBasis(out.bodyLocal);
            out.currentRawDesiredObjectWorld =
                grab_transform_telemetry::computeCurrentDesiredObjectFromFrame(out.rawHandWorld, _grabFrame.rawHandSpace);
            out.currentRawDesiredObjectWorldBasis = grab_transform_telemetry::makeOrientationBasis(out.currentRawDesiredObjectWorld);
            out.currentRawDesiredBodyWorld = multiplyTransforms(out.currentRawDesiredObjectWorld, _grabFrame.bodyLocal);
            out.currentRawDesiredBodyWorldBasis = grab_transform_telemetry::makeOrientationBasis(out.currentRawDesiredBodyWorld);
            {
                const RE::NiPoint3 currentPalmAnchorWorld = computeGrabPivotAWorld(world, out.rawHandWorld);
                const RE::NiPoint3 currentDesiredGripWorld =
                    transform_math::localPointToWorld(out.currentRawDesiredBodyWorld, activeProxyConstraintPivotBLocalGame());
                out.relationPivotErr =
                    pointDistanceGameUnits(currentPalmAnchorWorld, currentDesiredGripWorld);
                out.rotationPreservedDeg =
                    rotationDeltaDegrees(_grabFrame.objectNodeWorldAtGrab.rotate, _grabFrame.desiredObjectWorldAtGrab.rotate);
                out.normalAuthority = false;
                out.authoredRotationAuthority = false;
                out.hasGrabRelationInvariants = true;
            }
            if (out.hasHeldNodeWorld) {
                out.heldNodeToDesiredObjectAtGrab =
                    grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.desiredObjectWorldAtGrab);
                out.heldNodeToRawDesiredObject =
                    grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.currentRawDesiredObjectWorld);
            }
            if (out.hasHeldBodyWorld) {
                out.heldBodyToRawDesiredBody =
                    grab_transform_telemetry::measureTransformDelta(out.heldBodyWorld, out.currentRawDesiredBodyWorld);
            }
            out.bodyTargetNodeErr = grab_transform_telemetry::measureTransformDelta(
                heldNodeWorldFromBodyWorld(out.currentRawDesiredBodyWorld),
                out.currentRawDesiredObjectWorld);
        }
        if (out.hasHeldNodeWorld) {
            out.heldRelativeHandTargetWorld = grab_transform_telemetry::computeHeldRelativeHandTarget(out.heldNodeWorld, _grabFrame.rawHandSpace);
            out.heldRelativeHandTargetBasis = grab_transform_telemetry::makeOrientationBasis(out.heldRelativeHandTargetWorld);
            out.hasHeldRelativeHandTarget = true;
            out.rawToHeldRelativeHandTarget = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.heldRelativeHandTargetWorld);
            out.rawToHeldRelativeHandTargetAxes = grab_transform_telemetry::axisAlignmentDots(out.rawHandWorld.rotate, out.heldRelativeHandTargetWorld.rotate);
        }
    
        GrabPivotDebugSnapshot pivot{};
        if (getGrabPivotDebugSnapshot(world, pivot)) {
            out.pivotAWorld = pivot.handPivotWorld;
            out.pivotBWorld = pivot.objectPivotWorld;
            const auto pivotDelta = grab_transform_telemetry::measurePointPair(pivot.handPivotWorld, pivot.objectPivotWorld);
            out.pivotDeltaWorld = pivotDelta.delta;
            out.pivotErrorGameUnits = pivotDelta.distance;
            out.legacyPalmPivotAToRuntimePivotA =
                grab_transform_telemetry::measurePointPair(out.legacyPalmPivotAWorld, out.pivotAWorld);
        }
    
        if (_activeConstraint.constraintData) {
            const auto atoms = decodeGrabConstraintAtoms(_activeConstraint.constraintData);
    
            // The ragdoll motor target is seeded and updated from the generated
            // proxy local BODY relation. The proxy parent uses the generated
            // collider column convention, so telemetry derives the local relation
            // through that same boundary before comparing the atom bytes.
            const RE::NiTransform constraintProxyWorld =
                out.hasPalmAnchorGrabAuthority ? out.palmAnchorGrabAuthorityWorld : _grabFrame.handBodyWorldAtGrab;
            const RE::NiTransform desiredBodyWorldForConstraintTelemetry =
                grab_frame_math::objectFromGeneratedProxyLocalSpace(constraintProxyWorld, _grabFrame.proxyAuthorityBodyHandSpace);
            const RE::NiTransform desiredBodyTransformHandSpace =
                grab_frame_math::objectInGeneratedProxyLocalSpace(constraintProxyWorld, desiredBodyWorldForConstraintTelemetry);
            const auto relation = buildGrabConstraintRelationDiagnostics(
                atoms,
                constraintProxyWorld,
                desiredBodyTransformHandSpace,
                _grabFrame.pivotAHandBodyLocalGame);
            const RE::NiTransform desiredBodyToHandSpace = invertTransform(desiredBodyTransformHandSpace);
    
            out.constraintTransformBLocalGame = atoms.transformBTranslationGame;
            out.desiredTransformBLocalGame =
                grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformHandSpace, _grabFrame.pivotAHandBodyLocalGame);
            out.transformBLocalDelta = grab_transform_telemetry::measurePointPair(out.constraintTransformBLocalGame, out.desiredTransformBLocalGame);
            out.targetColumnsToConstraintInverseDegrees = rotationDeltaDegrees(atoms.targetColumns, desiredBodyToHandSpace.rotate);
            out.targetToHiggsRelationDegrees = relation.targetToHiggsRelationDegrees;
            out.targetColumnsToConstraintForwardDegrees = rotationDeltaDegrees(atoms.targetColumns, desiredBodyTransformHandSpace.rotate);
            out.transformBFrozenDeltaDegrees = relation.transformBFrozenDeltaDegrees;
            out.ragdollMotorEnabled = atoms.ragdollMotorEnabled;
            if (_activeConstraint.angularMotor) {
                out.angularMotorTau = _activeConstraint.angularMotor->tau;
                out.angularMotorDamping = _activeConstraint.angularMotor->damping;
                out.angularMotorMaxForce = (std::max)(std::fabs(_activeConstraint.angularMotor->minForce), std::fabs(_activeConstraint.angularMotor->maxForce));
            }
            if (_activeConstraint.linearMotor) {
                out.linearMotorTau = _activeConstraint.linearMotor->tau;
                out.linearMotorMaxForce = (std::max)(std::fabs(_activeConstraint.linearMotor->minForce), std::fabs(_activeConstraint.linearMotor->maxForce));
            }
            out.hasConstraintAngularTelemetry = true;
        }
    
        return true;
    }
    
    bool Hand::tryGetGrabOverlayPointProbeSample(RE::hknpWorld* world, GrabOverlayPointProbeSample& out)
    {
        out = {};
        if (!world) {
            return false;
        }
    
        // Guard the proxy handle and its last applied world transform.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if (!_grabAuthorityProxy.isValid() || _grabAuthorityProxyHknpWorld != world || !_hasLastAppliedGrabAuthorityProxyWorld) {
            return false;
        }
    
        out.queuedRawHandWorld = _lastAppliedGrabAuthorityQueuedRawHandWorld;
        out.appliedProxyTargetWorld = _lastAppliedGrabAuthorityProxyWorld;
        out.appliedRawHandWorld = _lastAppliedGrabAuthorityRawHandWorld;
        out.consumptionFrameShiftGame = _lastAppliedGrabAuthorityConsumptionFrameShiftGame;
        out.consumptionFrameRebaseStatus = _lastAppliedGrabAuthorityConsumptionFrameRebaseStatus;
        out.sourceControllerIdentity = _lastAppliedGrabAuthoritySourceControllerRoot.controllerIdentity;
        out.consumptionControllerIdentity = _lastAppliedGrabAuthorityConsumptionControllerRoot.controllerIdentity;
        out.sourceControllerVtable = _lastAppliedGrabAuthoritySourceControllerRoot.controllerVtable;
        out.consumptionControllerVtable = _lastAppliedGrabAuthorityConsumptionControllerRoot.controllerVtable;
        out.sourcePhysicsScaleRevision = _lastAppliedGrabAuthoritySourceControllerRoot.physicsScaleRevision;
        out.consumptionPhysicsScaleRevision = _lastAppliedGrabAuthorityConsumptionControllerRoot.physicsScaleRevision;
        out.proxyBodyId = _grabAuthorityProxy.getBodyId();
        out.objectBodyId = _savedObjectState.bodyId;
        out.sourceGameFrameIndex = _lastAppliedGrabAuthoritySourceGameFrameIndex;
        out.sourceQueueSequence = _lastAppliedGrabAuthoritySourceQueueSequence;
        out.pendingQueueSequence = _grabAuthorityProxyQueuedSequence;
        out.flushSequence = _grabAuthorityProxyFlushSequence;
        out.afterSolveSequence = _grabAuthorityProxyAfterSolveLogCounter;
        out.flushTiming = _grabAuthorityProxyLastFlushTiming;
        out.afterSolveTiming = _grabAuthorityProxyLastAfterSolveTiming;
        return true;
    }
}

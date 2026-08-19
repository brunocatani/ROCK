#include "physics-interaction/hand/Hand.h"

#include "physics-interaction/hand/grab/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace rock
{
    using namespace hand_grab_detail;

    bool Hand::tryGetGrabDriveObjectWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform) const
    {
        /*
         * Active grab drive reads use the rigid BODY frame. FO4VR also exposes a
         * motion/COM frame, but runtime testing rejected using that as the custom
         * constraint body-B local frame: the proxy target remained stable while
         * the object settled 90-180 degrees away from the hand. The selected
         * contact point stays frozen in BODY space for constraint, visual, and
         * release reconstruction; MOTION stays diagnostics/weight data only.
         */
        outTransform = makeIdentityTransform();
        return tryGetGrabAuthorityBodyWorldTransform(world, bodyId, outTransform);
    }
    
    void Hand::clearGrabAuthorityProxyRuntimeLocked()
    {
        _grabAuthorityProxyBhkWorld = nullptr;
        _grabAuthorityProxyHknpWorld = nullptr;
        _grabAuthorityPivotAProxyLocalGame = {};
        _grabAuthorityPivotBConstraintLocalGame = {};
        _grabAuthorityProxyFrameValid = false;
        _grabAuthorityPendingTarget = {};
        _grabAuthoritySourceClock.reset();
        _grabAuthorityConsumptionFrameRebase.reset();
        _lastAppliedGrabAuthorityQueuedRawHandWorld = {};
        _lastAppliedGrabAuthorityProxyWorld = {};
        _lastAppliedGrabAuthorityRawHandWorld = {};
        _lastAppliedGrabAuthorityConsumptionFrameShiftGame = {};
        _lastAppliedGrabAuthorityConsumptionFrameRebaseStatus =
            grab_authority_source_clock::ConsumptionFrameRebaseStatus::Unavailable;
        _lastAppliedGrabAuthoritySourceControllerRoot = {};
        _lastAppliedGrabAuthorityConsumptionControllerRoot = {};
        _lastAppliedGrabAuthoritySourceGameFrameIndex = 0;
        _lastAppliedGrabAuthoritySourceQueueSequence = 0;
        _grabAuthorityProxyLastFlushTiming = {};
        _grabAuthorityProxyLastAfterSolveTiming = {};
        _hasLastAppliedGrabAuthorityProxyWorld = false;
        clearGeneratedKeyframedBodyDriveState(_grabAuthorityProxyDriveState);
        _grabAuthorityProxyQueuedSequence = 0;
        _grabAuthorityProxyFlushSequence = 0;
        _grabAuthorityProxyFailedFlushes = 0;
        _grabAuthorityProxyLastFlushDeltaSeconds = 0.0f;
        _grabAuthorityProxyLogCounter = 0;
        _grabAuthorityProxyAfterSolveLogCounter = 0;
        _ragdollAngularProbePreSolve = {};
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
    }
    
    void Hand::clearGrabAuthorityProxyRuntime()
    {
        // Guard all proxy handles and cached proxy-world state during reset.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        clearGrabAuthorityProxyRuntimeLocked();
    }
    
    void Hand::destroyGrabAuthorityProxyLocked(RE::bhkWorld* bhkWorld)
    {
        auto* destroyWorld = bhkWorld ? bhkWorld : _grabAuthorityProxyBhkWorld;
        if (_grabAuthorityProxy.isValid()) {
            _grabAuthorityProxy.retireDeferred(destroyWorld);
        } else {
            _grabAuthorityProxy.reset();
        }
        clearGrabAuthorityProxyRuntimeLocked();
    }
    
    void Hand::destroyGrabAuthorityProxy(RE::bhkWorld* bhkWorld)
    {
        // Guard proxy destruction and the associated runtime-state reset.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        destroyGrabAuthorityProxyLocked(bhkWorld);
    }
    
    void Hand::abandonGrabAuthorityProxyLocked()
    {
        _grabAuthorityProxy.reset();
        clearGrabAuthorityProxyRuntimeLocked();
    }
    
    void Hand::abandonGrabAuthorityProxy()
    {
        // Guard proxy ownership while the invalid world is abandoned.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        abandonGrabAuthorityProxyLocked();
    }
    
    bool Hand::createProxyConstraintGrabDrive(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        RE::hknpBodyId objectBodyId,
        const RE::NiTransform& proxyWorldTransform,
        const RE::NiTransform& rawHandWorldTransform,
        const RE::NiPoint3& grabPivotAWorld,
        float tau,
        float damping,
        float maxForce,
        float authorityForceScale,
        float proportionalRecovery,
        float constantRecovery,
        bool looseWeaponGrab,
        const char* reason)
    {
        /*
         * The proxy path keeps the working ROCK contact/palm relation but moves
         * the solver anchor off the semantic hand collider. Body A is a hidden
         * keyframed no-contact proxy driven from the root-flattened hand frame
         * in the physics between phase; body B remains the dynamic held object.
         * This preserves the non-COM pivot while letting finite motors express
         * mass, collision, angular lag, and loose-weapon weight.
         */
        if (!bhkWorld || !world || objectBodyId.value == INVALID_BODY_ID) {
            return false;
        }
    
        if (_grabAuthorityProxy.isValid()) {
            destroyGrabAuthorityProxy(bhkWorld);
        }
    
        auto* proxyShape = grab_authority_proxy::buildProxyShape();
        if (!proxyShape) {
            ROCK_LOG_ERROR(Hand, "{} hand proxy constraint grab failed: proxy shape creation failed reason={}", handName(), reason ? reason : "unknown");
            return false;
        }
    
        const std::uint32_t proxyFilterInfo = grab_authority_proxy::noContactFilterInfo();
        const auto material = havok_material_registry::registerGeneratedBodyMaterial(world);
        const char* proxyName = _isLeft ? "ROCK_LeftGrabAuthorityProxy" : "ROCK_RightGrabAuthorityProxy";
        if (!_grabAuthorityProxy.create(
                world,
                bhkWorld,
                proxyShape,
                proxyFilterInfo,
                material,
                BethesdaMotionType::Keyframed,
                proxyName)) {
            havok_ref_count::release(proxyShape);
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: proxy body creation failed policy={} filter=0x{:08X} reason={}",
                handName(),
                grab_authority_proxy::filterPolicyName(),
                proxyFilterInfo,
                reason ? reason : "unknown");
            return false;
        }
        havok_ref_count::release(proxyShape);
    
        const RE::hkTransformf initialProxyHavok = grab_authority_proxy::makeHavokTransform(proxyWorldTransform);
        float zeroVelocity[4]{};
        const bool setTransformOk = _grabAuthorityProxy.setTransform(initialProxyHavok);
        const bool setVelocityOk = _grabAuthorityProxy.setVelocity(zeroVelocity, zeroVelocity);
        if (!setTransformOk || !setVelocityOk) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: initial proxy drive failed setTransform={} setVelocity={} proxyBody={} reason={}",
                handName(),
                setTransformOk ? "ok" : "fail",
                setVelocityOk ? "ok" : "fail",
                _grabAuthorityProxy.getBodyId().value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
        initializeGeneratedKeyframedBodyDriveState(_grabAuthorityProxyDriveState, proxyWorldTransform);
    
        std::uint32_t actualFilterInfo = 0;
        const bool filterReadOk = havok_runtime::tryReadFilterInfo(world, _grabAuthorityProxy.getBodyId(), actualFilterInfo);
        if (!filterReadOk || !grab_authority_proxy::hasNoContactFilterInfo(actualFilterInfo)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: proxy no-contact filter invalid read={} filter=0x{:08X} expectedPolicy={} proxyBody={} reason={}",
                handName(),
                filterReadOk ? "ok" : "fail",
                actualFilterInfo,
                grab_authority_proxy::filterPolicyName(),
                _grabAuthorityProxy.getBodyId().value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
    
        /*
         * Keep the hidden proxy at its configured seat offset. The selected
         * grab point is a real local pivot on body A; rebinding the proxy world
         * origin to pivot A would discard the runtime seat offset that moved the
         * solver anchor out of the palm interior.
         */
        const RE::NiPoint3 constraintPivotAWorld = grabPivotAWorld;
        const RE::NiPoint3 pivotAProxyLocalGame =
            grab_constraint_math::computeGeneratedProxyConstraintPivotLocalGame(proxyWorldTransform, constraintPivotAWorld);
        if (!std::isfinite(pivotAProxyLocalGame.x) ||
            !std::isfinite(pivotAProxyLocalGame.y) ||
            !std::isfinite(pivotAProxyLocalGame.z)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: transform-A local pivot is invalid proxyBody={} objBody={} reason={}",
                handName(),
                _grabAuthorityProxy.getBodyId().value,
                objectBodyId.value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
        // Constraint creation must seed the ragdoll motor with the generated
        // proxy local BODY relation that held updates keep writing. This parent
        // frame is the same column-authored generated-collider frame used for
        // pivot A, so derive it through the collider local conversion instead
        // of a normal NiTransform inverse. Transform-B stays on the selected
        // BODY-local grip pivot captured by the authority freeze.
        const RE::NiTransform desiredBodyWorldAtCreation =
            _grabFrame.hasTelemetryCapture ?
                _grabFrame.desiredBodyWorldAtGrab :
                grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, _grabFrame.proxyAuthorityBodyHandSpace);
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyWorldAtCreation);
        const RE::NiPoint3 relationPivotBConstraintLocalGame =
            grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformProxySpace, _grabFrame.pivotAHandBodyLocalGame);
        const RE::NiPoint3 solverPivotBConstraintLocalGame = _grabFrame.pivotBConstraintLocalGame;
        if (!std::isfinite(solverPivotBConstraintLocalGame.x) ||
            !std::isfinite(solverPivotBConstraintLocalGame.y) ||
            !std::isfinite(solverPivotBConstraintLocalGame.z)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: transform-B relation pivot is invalid proxyBody={} objBody={} reason={}",
                handName(),
                _grabAuthorityProxy.getBodyId().value,
                objectBodyId.value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
        const float selectionPivotRelationDeltaGameUnits =
            pointDistanceGameUnits(solverPivotBConstraintLocalGame, relationPivotBConstraintLocalGame);
    
        const float sanitizedAuthorityForceScale = std::clamp(
            std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f,
            0.05f,
            1.0f);
        const auto massSummaryAtCreation = readHeldBodyMassSummary(
            world,
            objectBodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedMass);
        const float effectiveMassAtCreation = effectiveGrabMotorMass(massSummaryAtCreation.motorMass());
        const GrabConstraintMotorTuning motorTuning = buildProxyConstraintMotorTuning(tau,
            damping,
            maxForce,
            sanitizedAuthorityForceScale,
            proportionalRecovery,
            constantRecovery,
            looseWeaponGrab,
            effectiveMassAtCreation,
            g_rockConfig.rockGrabMaxForceToMassRatio);
    
        _activeConstraint = createGrabConstraint(world,
            _grabAuthorityProxy.getBodyId(),
            objectBodyId,
            proxyWorldTransform,
            constraintPivotAWorld,
            desiredBodyTransformProxySpace,
            motorTuning);
        if (!_activeConstraint.isValid()) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: constraint creation failed proxyBody={} objBody={} reason={}",
                handName(),
                _grabAuthorityProxy.getBodyId().value,
                objectBodyId.value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
    
        if (grabTimelineTraceEnabled()) {
            std::array<float, 12> traceTransformBRotation{};
            std::array<float, 4> traceTransformBTranslation{};
            std::array<float, 12> traceTargetBRca{};
            grab_constraint_math::writeGrabConstraintCreationAtoms(
                traceTransformBRotation.data(),
                traceTransformBTranslation.data(),
                traceTargetBRca.data(),
                desiredBodyTransformProxySpace,
                pivotAProxyLocalGame,
                gameToHavokScale());
            const float* traceTransformBRotationData = traceTransformBRotation.data();
            const float* traceTransformBTranslationData = traceTransformBTranslation.data();
            const float* traceTargetBRcaData = traceTargetBRca.data();
            bool actualConstraintBytes = false;
            if (_activeConstraint.constraintData) {
                const auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
                traceTransformBRotationData = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_COL0);
                traceTransformBTranslationData = reinterpret_cast<const float*>(constraintData + GRAB_TRANSFORM_B_POS);
                traceTargetBRcaData = reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
                actualConstraintBytes = true;
            }
            const auto atoms = decodeGrabConstraintAtoms(
                nullptr,
                traceTransformBRotationData,
                traceTransformBTranslationData,
                traceTargetBRcaData,
                _activeConstraint.usesRagdollAngularMotorAtom());
            const auto relation = buildGrabConstraintRelationDiagnostics(
                atoms,
                proxyWorldTransform,
                desiredBodyTransformProxySpace,
                pivotAProxyLocalGame);
    
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=constraint_create trace={} constraint={} proxyBody={} objBody={} bytes={} angular={} ragdoll={} pivotAProxy=({:.2f},{:.2f},{:.2f}) pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) selectedPivotRelationDelta={:.3f}gu pivotBRelationDelta={:.3f}gu pivotBBody=({:.2f},{:.2f},{:.2f}) linearTau={:.3f} angularTau={:.3f} linearForce={:.0f} angularForce={:.0f} motorMass={:.3f} effectiveMass={:.3f} forceBudget={:.2f} targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg reason={}",
                handName(),
                _grabFrame.traceId,
                _activeConstraint.constraintId,
                _grabAuthorityProxy.getBodyId().value,
                objectBodyId.value,
                actualConstraintBytes ? "actual" : "computed",
                grabAngularAuthorityName(_activeConstraint.angularAuthority),
                _activeConstraint.usesRagdollAngularMotorAtom() ? "yes" : "no",
                pivotAProxyLocalGame.x,
                pivotAProxyLocalGame.y,
                pivotAProxyLocalGame.z,
                solverPivotBConstraintLocalGame.x,
                solverPivotBConstraintLocalGame.y,
                solverPivotBConstraintLocalGame.z,
                relationPivotBConstraintLocalGame.x,
                relationPivotBConstraintLocalGame.y,
                relationPivotBConstraintLocalGame.z,
                selectionPivotRelationDeltaGameUnits,
                relation.transformBRelationDeltaGameUnits,
                _grabFrame.pivotBBodyLocalGame.x,
                _grabFrame.pivotBBodyLocalGame.y,
                _grabFrame.pivotBBodyLocalGame.z,
                motorTuning.linearTau,
                motorTuning.angularTau,
                motorTuning.linearMaxForce,
                motorTuning.angularMaxForce,
                massSummaryAtCreation.motorMass(),
                effectiveMassAtCreation,
                sanitizedAuthorityForceScale,
                relation.targetToHiggsRelationDegrees,
                relation.transformBFrozenDeltaDegrees,
                reason ? reason : "unknown");
    
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=constraint_atoms trace={} target_bRca=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})] transformBCols=[({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f}) ({:.3f},{:.3f},{:.3f})]",
                handName(),
                _grabFrame.traceId,
                traceTargetBRcaData[0],
                traceTargetBRcaData[1],
                traceTargetBRcaData[2],
                traceTargetBRcaData[4],
                traceTargetBRcaData[5],
                traceTargetBRcaData[6],
                traceTargetBRcaData[8],
                traceTargetBRcaData[9],
                traceTargetBRcaData[10],
                traceTransformBRotationData[0],
                traceTransformBRotationData[1],
                traceTransformBRotationData[2],
                traceTransformBRotationData[4],
                traceTransformBRotationData[5],
                traceTransformBRotationData[6],
                traceTransformBRotationData[8],
                traceTransformBRotationData[9],
                traceTransformBRotationData[10]);
    
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=constraint_relation trace={} bytes={} bodyInProxyToDesired={:.3f}gu/{:.2f}deg relationInvToDesired={:.3f}gu/{:.2f}deg atomRowsToDesired={:.3f}gu/{:.2f}deg atomRowsToRelationInv={:.2f}deg targetRowsToProxyInBody={:.2f}deg tBAtomRelationDelta={:.3f}gu relationPivotB=({:.2f},{:.2f},{:.2f}) atomPivotB=({:.2f},{:.2f},{:.2f})",
                handName(),
                _grabFrame.traceId,
                actualConstraintBytes ? "actual" : "computed",
                translationDeltaGameUnits(
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyTransformProxySpace),
                    desiredBodyWorldAtCreation),
                rotationDeltaDegrees(
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyTransformProxySpace).rotate,
                    desiredBodyWorldAtCreation.rotate),
                translationDeltaGameUnits(relation.relationInverseBodyWorld, desiredBodyWorldAtCreation),
                rotationDeltaDegrees(relation.relationInverseBodyWorld.rotate, desiredBodyWorldAtCreation.rotate),
                translationDeltaGameUnits(relation.atomRowsBodyWorld, desiredBodyWorldAtCreation),
                rotationDeltaDegrees(relation.atomRowsBodyWorld.rotate, desiredBodyWorldAtCreation.rotate),
                rotationDeltaDegrees(relation.atomRowsBodyWorld.rotate, relation.relationInverseBodyWorld.rotate),
                relation.targetToHiggsRelationDegrees,
                relation.transformBRelationDeltaGameUnits,
                relation.relationTransformBLocalGame.x,
                relation.relationTransformBLocalGame.y,
                relation.relationTransformBLocalGame.z,
                atoms.transformBTranslationGame.x,
                atoms.transformBTranslationGame.y,
                atoms.transformBTranslationGame.z);
        }
    
        const auto grabStartControllerRoot = samplePlayerControllerRootFrame();
        {
            // Guard proxy creation, world binding, and initial authority state.
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            _grabAuthorityProxyBhkWorld = bhkWorld;
            _grabAuthorityProxyHknpWorld = world;
            _grabAuthorityPivotAProxyLocalGame = pivotAProxyLocalGame;
            // Preserve the selected BODY-local pivot for grab/release semantics;
            // the constraint atom's transform-B translation is relation-derived.
            _grabAuthorityPivotBConstraintLocalGame = solverPivotBConstraintLocalGame;
            _grabAuthorityProxyFrameValid = true;
            _grabAuthorityPendingTarget = GrabAuthorityProxyPendingTarget{
                .proxyWorld = proxyWorldTransform,
                .rawHandWorld = rawHandWorldTransform,
                .proxyFrameSource = "grabStartLivePalmAnchor",
                .sourceGameFrameIndex = runtime_state::currentFrame().frameIndex,
                .sourceControllerRoot = grabStartControllerRoot,
                .deltaTime = 1.0f / 90.0f,
                .forceFadeInTime = g_rockConfig.rockGrabForceFadeInTime,
                .tauMin = g_rockConfig.rockGrabTauMin,
                .grabPositionErrorGameUnits = 0.0f,
                .grabRotationErrorDegrees = 0.0f,
                .authorityForceScale = sanitizedAuthorityForceScale,
                .heldBodyColliding = false,
                .valid = true,
            };
            _lastAppliedGrabAuthorityQueuedRawHandWorld = rawHandWorldTransform;
            _lastAppliedGrabAuthorityProxyWorld = proxyWorldTransform;
            _lastAppliedGrabAuthorityRawHandWorld = rawHandWorldTransform;
            _lastAppliedGrabAuthorityConsumptionFrameShiftGame = {};
            _lastAppliedGrabAuthorityConsumptionFrameRebaseStatus =
                grab_authority_source_clock::ConsumptionFrameRebaseStatus::Unavailable;
            _lastAppliedGrabAuthoritySourceControllerRoot = grabStartControllerRoot;
            _lastAppliedGrabAuthorityConsumptionControllerRoot = {};
            _lastAppliedGrabAuthoritySourceGameFrameIndex = runtime_state::currentFrame().frameIndex;
            _lastAppliedGrabAuthoritySourceQueueSequence = 1;
            _hasLastAppliedGrabAuthorityProxyWorld = true;
            _grabAuthoritySourceClock.reset();
            _grabAuthorityConsumptionFrameRebase.reset();
            _grabAuthorityProxyQueuedSequence = 1;
            _grabAuthorityProxyFlushSequence = 0;
            _grabAuthorityProxyFailedFlushes = 0;
            _grabAuthorityProxyLastFlushDeltaSeconds = 0.0f;
            _grabAuthorityProxyLogCounter = 0;
            _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        }
    
        ROCK_LOG_DEBUG(Hand,
            "{} hand proxy constraint grab drive: constraint={} looseWeapon={} proxyBody={} objBody={} filter=0x{:08X} pivotAProxy=({:.2f},{:.2f},{:.2f}) pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) selectedPivotRelationDelta={:.3f}gu pivotBBody=({:.2f},{:.2f},{:.2f}) linearTau={:.3f} angularTau={:.3f} linearForce={:.0f} angularForce={:.0f} motorMass={:.3f} forceBudget={:.2f} reason={}",
            handName(),
            _activeConstraint.constraintId,
            looseWeaponGrab ? "yes" : "no",
            _grabAuthorityProxy.getBodyId().value,
            objectBodyId.value,
            actualFilterInfo,
            pivotAProxyLocalGame.x,
            pivotAProxyLocalGame.y,
            pivotAProxyLocalGame.z,
            solverPivotBConstraintLocalGame.x,
            solverPivotBConstraintLocalGame.y,
            solverPivotBConstraintLocalGame.z,
            relationPivotBConstraintLocalGame.x,
            relationPivotBConstraintLocalGame.y,
            relationPivotBConstraintLocalGame.z,
            selectionPivotRelationDeltaGameUnits,
            _grabFrame.pivotBBodyLocalGame.x,
            _grabFrame.pivotBBodyLocalGame.y,
            _grabFrame.pivotBBodyLocalGame.z,
            motorTuning.linearTau,
            motorTuning.angularTau,
            motorTuning.linearMaxForce,
            motorTuning.angularMaxForce,
            massSummaryAtCreation.motorMass(),
            sanitizedAuthorityForceScale,
            reason ? reason : "unknown");
        return true;
    }
    
    bool Hand::updateProxyConstraintGrabDriveTarget(RE::hknpWorld* world,
        const RE::NiTransform& proxyWorldTransform,
        RE::NiTransform& outDesiredObjectWorld,
        RE::NiTransform& outDesiredBodyWorld,
        RE::NiPoint3& outDesiredTargetPointWorld,
        RE::NiPoint3& outActivePivotBBodyLocalGame)
    {
        const RE::NiTransform desiredBodyRelationProxySpace = _grabFrame.proxyAuthorityBodyHandSpace;
        outDesiredObjectWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, _grabFrame.proxyAuthorityHandSpace);
        outDesiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyRelationProxySpace);
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorldTransform, outDesiredBodyWorld);
        const RE::NiPoint3 selectedPivotBBodyLocalGame = activeProxyConstraintPivotBLocalGame();
        const RE::NiPoint3 relationPivotB =
            grab_constraint_math::computeHiggsTransformBTranslationGame(
                desiredBodyTransformProxySpace,
                _grabFrame.pivotAHandBodyLocalGame);
        outActivePivotBBodyLocalGame = relationPivotB;
        outDesiredTargetPointWorld = transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);
    
        if (!world || !_activeConstraint.isValid() || !_activeConstraint.constraintData || !_grabAuthorityProxy.isValid() ||
            !_grabAuthorityProxyFrameValid || _grabAuthorityProxy.getBodyId().value == INVALID_BODY_ID) {
            return false;
        }
    
        /*
         * Transform-A is part of the frozen local constraint frame and is not
         * rewritten while held. The frozen capture chooses the ragdoll
         * transform-B decomposition once, while the current proxy-in-BODY
         * relation keeps updating target_bRca and transform-B translation.
         */
        const RE::NiPoint3 pivotAProxyLocalGame = _grabAuthorityPivotAProxyLocalGame;
        if (!std::isfinite(pivotAProxyLocalGame.x) ||
            !std::isfinite(pivotAProxyLocalGame.y) ||
            !std::isfinite(pivotAProxyLocalGame.z)) {
            return false;
        }
    
        auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
        const float gameToHkScale = gameToHavokScale();
    
        auto* transformBRotation = reinterpret_cast<float*>(constraintData + GRAB_TRANSFORM_B_COL0);
        auto* transformBTranslation = reinterpret_cast<float*>(constraintData + GRAB_TRANSFORM_B_POS);
        auto* targetBRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
        grab_constraint_math::writeGrabConstraintHeldTargetAtoms(
            transformBRotation,
            transformBTranslation,
            targetBRca,
            desiredBodyTransformProxySpace,
            pivotAProxyLocalGame,
            gameToHkScale);
        outDesiredTargetPointWorld = transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);
    
        const std::uint64_t targetWriteSequence = ++_grabFrame.traceTargetWriteSequence;
        if (grabTimelineTraceEnabled() && shouldLogGrabTimelineSequence(targetWriteSequence)) {
            const auto atoms = decodeGrabConstraintAtoms(
                nullptr,
                transformBRotation,
                transformBTranslation,
                targetBRca,
                true);
            const auto relation = buildGrabConstraintRelationDiagnostics(
                atoms,
                proxyWorldTransform,
                desiredBodyTransformProxySpace,
                pivotAProxyLocalGame);
            const float selectedPivotRelationDeltaGameUnits =
                pointDistanceGameUnits(selectedPivotBBodyLocalGame, relationPivotB);
    
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=target_write trace={} writeSeq={} flushNext={} queued={} constraint={} proxyBody={} objBody={} proxyPos=({:.2f},{:.2f},{:.2f}) desiredBodyPos=({:.2f},{:.2f},{:.2f}) targetPoint=({:.2f},{:.2f},{:.2f}) pivotAProxy=({:.2f},{:.2f},{:.2f}) pivotBSelected=({:.2f},{:.2f},{:.2f}) relationPivotB=({:.2f},{:.2f},{:.2f}) selectedPivotRelationDelta={:.3f}gu pivotBRelationDelta={:.3f}gu targetToHiggsRelation={:.2f}deg transformBFrozenDelta={:.2f}deg",
                handName(),
                _grabFrame.traceId,
                targetWriteSequence,
                _grabAuthorityProxyFlushSequence + 1,
                _grabAuthorityProxyQueuedSequence,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
                _savedObjectState.bodyId.value,
                proxyWorldTransform.translate.x,
                proxyWorldTransform.translate.y,
                proxyWorldTransform.translate.z,
                outDesiredBodyWorld.translate.x,
                outDesiredBodyWorld.translate.y,
                outDesiredBodyWorld.translate.z,
                outDesiredTargetPointWorld.x,
                outDesiredTargetPointWorld.y,
                outDesiredTargetPointWorld.z,
                pivotAProxyLocalGame.x,
                pivotAProxyLocalGame.y,
                pivotAProxyLocalGame.z,
                selectedPivotBBodyLocalGame.x,
                selectedPivotBBodyLocalGame.y,
                selectedPivotBBodyLocalGame.z,
                relationPivotB.x,
                relationPivotB.y,
                relationPivotB.z,
                selectedPivotRelationDeltaGameUnits,
                relation.transformBRelationDeltaGameUnits,
                relation.targetToHiggsRelationDegrees,
                relation.transformBFrozenDeltaDegrees);
    
            ROCK_LOG_INFO(Hand,
                "{} GRAB_TRACE stage=target_relation trace={} writeSeq={} flushNext={} queued={} bodyInProxyToDesired={:.3f}gu/{:.2f}deg relationInvToDesired={:.3f}gu/{:.2f}deg atomRowsToDesired={:.3f}gu/{:.2f}deg atomRowsToRelationInv={:.2f}deg targetRowsToProxyInBody={:.2f}deg tBAtomRelationDelta={:.3f}gu relationPivotB=({:.2f},{:.2f},{:.2f}) atomPivotB=({:.2f},{:.2f},{:.2f})",
                handName(),
                _grabFrame.traceId,
                targetWriteSequence,
                _grabAuthorityProxyFlushSequence + 1,
                _grabAuthorityProxyQueuedSequence,
                translationDeltaGameUnits(
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyTransformProxySpace),
                    outDesiredBodyWorld),
                rotationDeltaDegrees(
                    grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyTransformProxySpace).rotate,
                    outDesiredBodyWorld.rotate),
                translationDeltaGameUnits(relation.relationInverseBodyWorld, outDesiredBodyWorld),
                rotationDeltaDegrees(relation.relationInverseBodyWorld.rotate, outDesiredBodyWorld.rotate),
                translationDeltaGameUnits(relation.atomRowsBodyWorld, outDesiredBodyWorld),
                rotationDeltaDegrees(relation.atomRowsBodyWorld.rotate, outDesiredBodyWorld.rotate),
                rotationDeltaDegrees(relation.atomRowsBodyWorld.rotate, relation.relationInverseBodyWorld.rotate),
                relation.targetToHiggsRelationDegrees,
                relation.transformBRelationDeltaGameUnits,
                relationPivotB.x,
                relationPivotB.y,
                relationPivotB.z,
                atoms.transformBTranslationGame.x,
                atoms.transformBTranslationGame.y,
                atoms.transformBTranslationGame.z);
        }
        return true;
    }
    
    bool Hand::resolveGrabAuthorityProxyFrame(RE::hknpWorld* world,
        const RE::NiTransform& rawHandWorld,
        const RE::NiTransform* fallbackPalmAnchorWorld,
        RE::NiTransform& outProxyWorld,
        const char*& outSource,
        Hand::GrabAuthorityProxyFramePolicy policy) const
    {
        (void)rawHandWorld;
        (void)fallbackPalmAnchorWorld;
    
        auto isFiniteProxyFrameInput = [](const RE::NiTransform& transform) {
            bool rotationFinite = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite && std::isfinite(transform.rotate.entry[row][column]);
                }
            }
            return rotationFinite &&
                   std::isfinite(transform.translate.x) &&
                   std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) &&
                   std::isfinite(transform.scale) &&
                   transform.scale > 0.0001f;
        };
    
        if (policy == GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget) {
            RE::NiTransform queuedPalmAnchorTarget{};
            if (tryGetPalmAnchorTarget(queuedPalmAnchorTarget) && isFiniteProxyFrameInput(queuedPalmAnchorTarget)) {
                const RE::NiTransform proxyBaseWorld =
                    hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(queuedPalmAnchorTarget);
                outProxyWorld = applyGrabAuthorityProxyLocalOffsetToFrame(proxyBaseWorld, _isLeft);
                outSource = "queuedPalmAnchorTargetGrabFrame";
                return true;
            }
        }
    
        LivePalmAnchorReference palmReference{};
        if (tryResolveLivePalmAnchorReference(world, palmReference)) {
            const RE::NiTransform proxyBaseWorld =
                hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(palmReference.world);
            outProxyWorld = applyGrabAuthorityProxyLocalOffsetToFrame(proxyBaseWorld, _isLeft);
            switch (palmReference.source) {
            case body_frame::BodyFrameSource::MotionCenterOfMass:
                outSource = "livePalmAnchorMotionGrabFrame";
                break;
            case body_frame::BodyFrameSource::BodyTransform:
                outSource = "livePalmAnchorBodyGrabFrame";
                break;
            default:
                outSource = "livePalmAnchorResolvedGrabFrame";
                break;
            }
            return true;
        }
    
        outProxyWorld = transform_math::makeIdentityTransform<RE::NiTransform>();
        outSource = "livePalmAnchorUnavailable";
        return false;
    }
    
    bool Hand::resolveActiveGrabAuthorityPivotAWorld(
        const RE::NiTransform& proxyWorldTransform,
        RE::NiPoint3& outPivotWorld) const
    {
        outPivotWorld = {};
        if (!_grabFrame.hasTelemetryCapture || !_grabFrame.hasFrozenPivotB ||
            !std::isfinite(_grabFrame.pivotAHandBodyLocalGame.x) ||
            !std::isfinite(_grabFrame.pivotAHandBodyLocalGame.y) ||
            !std::isfinite(_grabFrame.pivotAHandBodyLocalGame.z)) {
            return false;
        }
    
        /*
         * Pivot A is frozen as a generated/proxy local point at grab commit.
         * Held updates replay that local point through the current proxy body
         * frame instead of recomputing palm or pinch seats from raw hand space.
         */
        outPivotWorld = generatedProxyLocalPointToWorld(proxyWorldTransform, _grabFrame.pivotAHandBodyLocalGame);
        return std::isfinite(outPivotWorld.x) &&
               std::isfinite(outPivotWorld.y) &&
               std::isfinite(outPivotWorld.z);
    }
    
    void Hand::updateConstraintGrabDriveMotors(RE::hknpWorld* world,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        float authorityForceScale,
        bool heldBodyColliding,
        const grab_motion_controller::HeldAuthorityState& heldAuthority)
    {
        if (!_activeConstraint.isValid() || !_activeConstraint.linearMotor || !_activeConstraint.angularMotor) {
            return;
        }
    
        const float looseLinearTauMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier);
        const float looseAngularTauMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier);
        const float looseCollisionTauMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier);
        const float looseLinearDampingMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier);
        const float looseAngularDampingMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier);
        const float looseMaxForceMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier);
        const float looseAngularForceMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier);
        const float looseLinearRecoveryMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
        const float looseAngularRecoveryMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);
        const float sharedBaseMaxForce = scaleDriveValue(g_rockConfig.rockGrabConstraintMaxForce, looseMaxForceMultiplier);
    
        const auto massSummary = readHeldBodyMassSummary(
            world,
            _savedObjectState.bodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedMass);
        const auto motorInput = grab_motion_controller::MotorInput{
            .heldBodyColliding = heldBodyColliding,
            .baseLinearTau = scaleDriveValue(g_rockConfig.rockGrabLinearTau, looseLinearTauMultiplier),
            .baseAngularTau = scaleDriveValue(g_rockConfig.rockGrabAngularTau, looseAngularTauMultiplier),
            .collisionTau = scaleDriveValue(tauMin, looseCollisionTauMultiplier),
            .currentLinearTau = _activeConstraint.linearMotor->tau,
            .currentAngularTau = _activeConstraint.angularMotor->tau,
            .tauLerpSpeed = g_rockConfig.rockGrabTauLerpSpeed,
            .deltaTime = deltaTime,
            .physicsRateForceScalingEnabled = g_rockConfig.rockGrabPhysicsRateForceScalingEnabled,
            .physicsDeltaSeconds = deltaTime,
            .physicsRateReferenceHz = g_rockConfig.rockGrabPhysicsRateReferenceHz,
            .physicsRateForceScaleExponent = g_rockConfig.rockGrabPhysicsRateForceScaleExponent,
            .physicsRateMinForceScale = g_rockConfig.rockGrabPhysicsRateMinForceScale,
            .physicsRateMaxForceScale = g_rockConfig.rockGrabPhysicsRateMaxForceScale,
            .baseMaxForce = sharedBaseMaxForce,
            .authorityForceScale = authorityForceScale,
            .angularForceMultiplier = looseAngularForceMultiplier,
            .mass = massSummary.motorMass(),
            .forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio,
            .effectiveMotorMassFloorEnabled = g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
            .effectiveMotorMassFloor = g_rockConfig.rockGrabEffectiveMotorMassFloor,
            .fadeInEnabled = _grabFrame.fadeInGrabConstraint,
            .fadeElapsed = _grabStartTime,
            .fadeDuration = forceFadeInTime,
        };
        const auto output = grab_motion_controller::solveMotorTargetsWithAuthority(motorInput, heldAuthority);
        _lastGrabPhysicsHz.store(output.physicsHz, std::memory_order_relaxed);
        _lastGrabPhysicsRateForceScale.store(output.physicsRateForceScale, std::memory_order_relaxed);
    
        _activeConstraint.linearMotor->tau = output.linearTau;
        _activeConstraint.linearMotor->damping = scaleDriveValue(g_rockConfig.rockGrabLinearDamping, looseLinearDampingMultiplier);
        _activeConstraint.linearMotor->proportionalRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabLinearProportionalRecovery, looseLinearRecoveryMultiplier);
        _activeConstraint.linearMotor->constantRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabLinearConstantRecovery, looseLinearRecoveryMultiplier);
        _activeConstraint.linearMotor->minForce = -output.linearMaxForce;
        _activeConstraint.linearMotor->maxForce = output.linearMaxForce;
    
        _activeConstraint.angularMotor->tau = output.angularTau;
        _activeConstraint.angularMotor->damping =
            scaleDriveValue(g_rockConfig.rockGrabAngularDamping, looseAngularDampingMultiplier);
        _activeConstraint.angularMotor->proportionalRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabAngularProportionalRecovery, looseAngularRecoveryMultiplier);
        _activeConstraint.angularMotor->constantRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabAngularConstantRecovery, looseAngularRecoveryMultiplier);
        _activeConstraint.angularMotor->minForce = -output.angularMaxForce;
        _activeConstraint.angularMotor->maxForce = output.angularMaxForce;
    
        _activeConstraint.currentTau = output.linearTau;
        _activeConstraint.currentMaxForce = output.linearMaxForce;
        _activeConstraint.targetMaxForce = output.linearMaxForce;
    }
    
    void Hand::queueProxyGrabAuthorityTarget(const RE::NiTransform& proxyWorldTransform,
        const RE::NiTransform& rawHandWorldTransform,
        const char* proxyFrameSource,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        float grabPositionErrorGameUnits,
        float grabRotationErrorDegrees,
        float authorityForceScale,
        bool heldBodyColliding)
    {
        // The authority flush calls this method on the physics thread.
        const auto sourceControllerRoot = samplePlayerControllerRootFrame();
        // Guard the live proxy handle and every target-write diagnostic.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if (!_grabAuthorityProxy.isValid()) {
            return;
        }
    
        _grabAuthorityPendingTarget.proxyWorld = proxyWorldTransform;
        _grabAuthorityPendingTarget.rawHandWorld = rawHandWorldTransform;
        _grabAuthorityPendingTarget.proxyFrameSource = proxyFrameSource ? proxyFrameSource : "unknown";
        _grabAuthorityPendingTarget.sourceGameFrameIndex = runtime_state::currentFrame().frameIndex;
        _grabAuthorityPendingTarget.sourceControllerRoot = sourceControllerRoot;
        _grabAuthorityPendingTarget.deltaTime = deltaTime;
        _grabAuthorityPendingTarget.forceFadeInTime = forceFadeInTime;
        _grabAuthorityPendingTarget.tauMin = tauMin;
        _grabAuthorityPendingTarget.grabPositionErrorGameUnits = grabPositionErrorGameUnits;
        _grabAuthorityPendingTarget.grabRotationErrorDegrees = grabRotationErrorDegrees;
        _grabAuthorityPendingTarget.authorityForceScale = std::clamp(
            std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f,
            0.05f,
            1.0f);
        _grabAuthorityPendingTarget.heldBodyColliding = heldBodyColliding;
        _grabAuthorityPendingTarget.valid = true;
        ++_grabAuthorityProxyQueuedSequence;
    }
    
    bool Hand::promoteHeldObjectToConstraintDrive(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        const char* reason)
    {
        /*
         * Dynamic loose-object grab has only one authority path now: the hidden
         * no-contact proxy plus finite linear/angular constraint. Peer-hand join
         * only tightens the shared force budget for the already-active proxy.
         */
        if (!isHolding() || !bhkWorld || !world || !_savedObjectState.isValid()) {
            return false;
        }
        (void)handWorldTransform;
        (void)tau;
        (void)damping;
        (void)maxForce;
        (void)proportionalRecovery;
        (void)constantRecovery;
    
        if (_activeConstraint.isValid() && _grabAuthorityProxy.isValid()) {
            // Guard the proxy handle while held state adopts constraint drive.
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            _grabAuthorityPendingTarget.authorityForceScale =
                held_object_drive_policy::sanitizeMotorAuthorityScale(sharedGrabAuthorityForceScale(true));
            ROCK_LOG_DEBUG(Hand,
                "{} hand peer promotion kept existing proxy constraint drive: formID={:08X} body={} forceBudget={:.2f} driveMode={} reason={}",
                handName(),
                _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
                _savedObjectState.bodyId.value,
                _grabAuthorityPendingTarget.authorityForceScale,
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                reason ? reason : "peer-joined-held-object");
            return true;
        }
    
        ROCK_LOG_WARN(Hand,
            "{} hand peer promotion failed: held object has no proxy constraint authority formID={:08X} body={} reason={}",
            handName(),
            _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
            _savedObjectState.bodyId.value,
            reason ? reason : "peer-joined-held-object");
        return false;
    }

    RE::NiPoint3 Hand::activeProxyConstraintPivotBLocalGame() const
    {
        if (_grabAuthorityProxyFrameValid) {
            return _grabAuthorityPivotBConstraintLocalGame;
        }
        return _grabFrame.pivotBConstraintLocalGame;
    }
}

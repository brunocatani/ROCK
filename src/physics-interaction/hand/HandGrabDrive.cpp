// The held object's drive: a keyframed grab authority proxy body follows the
// hand and a finite-force constraint pulls the held body toward it. Creates
// and updates that constraint and its motors, resolves the proxy frame, and
// flushes and observes the custom grab authority around each Havok step.
#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock
{

    namespace
    {
        bool computeHardKeyframeVelocityForTarget(RE::hknpWorld* world, RE::hknpBodyId bodyId,
            const RE::NiTransform& targetWorld, float deltaTime, float outLinearVelocityHavok[4],
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
                !outLinearVelocityHavok || !outAngularVelocityRadians || !havok_runtime::getBody(world, bodyId)) {
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

            // Native hard-keyframing uses aligned stores; caller outputs need only float alignment.
            alignas(16) float linearVelocityHavok[4]{};
            alignas(16) float angularVelocityRadians[4]{};
            using ComputeHardKeyFrame_t =
                void (*)(RE::hknpWorld*, RE::hknpBodyId, float*, float*, float, float*, float*);
            static REL::Relocation<ComputeHardKeyFrame_t> compute{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            compute(world, bodyId, targetPositionHavok, targetRotationHavok, deltaTime, linearVelocityHavok,
                angularVelocityRadians);

            havok_runtime::copyVector4(linearVelocityHavok, outLinearVelocityHavok);
            havok_runtime::copyVector4(angularVelocityRadians, outAngularVelocityRadians);
            outLinearVelocityHavok[3] = 0.0f;
            outAngularVelocityRadians[3] = 0.0f;
            return havok_runtime::isFinite3(outLinearVelocityHavok) &&
                havok_runtime::isFinite3(outAngularVelocityRadians);
        }

        /*
         * A live NPC limb is one link of a jointed body anchored by his legs. At
         * an object's force the hold beat the ragdoll's joint solver: a pull
         * opened his neck joints 4.4 cm and forced them past their limits, a
         * visible tear in his skin, and lifted his feet off the ground.
         */
        constexpr float LIVE_LIMB_AUTHORITY_SCALE = 0.2f;
        /*
         * The hand holds a point on a live limb, not its orientation: the
         * angular motor turned a grabbed head 40 degrees to match the palm
         * before any pull, past what the neck's skin allows.
         */
        constexpr float LIVE_LIMB_ANGULAR_SHARE = 0.1f;

        float looseWeaponMultiplier(bool looseWeaponGrab, float multiplier)
        {
            return looseWeaponGrab ? (std::isfinite(multiplier) ? multiplier : 1.0f) : 1.0f;
        }

        float scaleDriveValue(float value, float multiplier)
        {
            return (std::isfinite(value) ? value : 0.0f) * (std::isfinite(multiplier) ? multiplier : 1.0f);
        }

        GrabConstraintMotorTuning buildProxyConstraintMotorTuning(float tau, float damping, float maxForce,
            float authorityForceScale, float proportionalRecovery, float constantRecovery, bool looseWeaponGrab,
            float mass, float forceToMassRatio)
        {
            const float linearTauMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier);
            const float angularTauMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier);
            const float linearDampingMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier);
            const float angularDampingMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier);
            const float maxForceMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier);
            const float angularForceMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier);
            const float linearRecoveryMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
            const float angularRecoveryMultiplier = looseWeaponMultiplier(looseWeaponGrab,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);

            const float linearBudget = (std::max)(0.0f, scaleDriveValue(maxForce, maxForceMultiplier));
            const float sanitizedAuthorityForceScale = std::clamp(
                std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f, 0.05f,
                1.0f);
            const float linearMaxForce = grab_motion_controller::capForceByMass(linearBudget, mass, forceToMassRatio) *
                sanitizedAuthorityForceScale;
            const float angularMaxForce = linearMaxForce * angularForceMultiplier;

            return GrabConstraintMotorTuning{
                .linearTau = scaleDriveValue(tau, linearTauMultiplier),
                .linearDamping = scaleDriveValue(damping, linearDampingMultiplier),
                .linearProportionalRecovery = scaleDriveValue(proportionalRecovery, linearRecoveryMultiplier),
                .linearConstantRecovery = scaleDriveValue(constantRecovery, linearRecoveryMultiplier),
                .linearMaxForce = linearMaxForce,
                .angularTau = scaleDriveValue(g_rockConfig.rockGrabAngularTau, angularTauMultiplier),
                .angularDamping = scaleDriveValue(g_rockConfig.rockGrabAngularDamping, angularDampingMultiplier),
                .angularProportionalRecovery =
                    scaleDriveValue(g_rockConfig.rockGrabAngularProportionalRecovery, angularRecoveryMultiplier),
                .angularConstantRecovery =
                    scaleDriveValue(g_rockConfig.rockGrabAngularConstantRecovery, angularRecoveryMultiplier),
                .angularMaxForce = angularMaxForce,
            };
        }
    }

    bool Hand::tryGetGrabDriveObjectWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform) const
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

    RE::NiPoint3 Hand::activeProxyConstraintPivotBLocalGame() const
    {
        if (_grabAuthorityProxyFrameValid) {
            return _grabAuthorityPivotBConstraintLocalGame;
        }
        return _grabFrame.authority.pivotBConstraintLocalGame;
    }

    void Hand::clearGrabAuthorityProxyRuntimeLocked()
    {
        _consumerPairCount = 0;
        _releaseObjectSample = {};
        _grabAuthorityProxyBhkWorld = nullptr;
        _grabAuthorityProxyHknpWorld = nullptr;
        _grabAuthorityPivotAProxyLocalGame = {};
        _grabAuthorityPivotBConstraintLocalGame = {};
        _grabAuthorityProxyFrameValid = false;
        _grabAuthorityPendingTarget = {};
        _grabAuthoritySourceClock.reset();
        _lastAppliedGrabAuthorityProxyWorld = {};
        _hasLastAppliedGrabAuthorityProxyWorld = false;
        clearGeneratedKeyframedBodyDriveState(_grabAuthorityProxyDriveState);
        _grabAuthorityProxyQueuedSequence = 0;
        _grabAuthorityProxyFlushSequence = 0;
        _grabAuthorityProxyFailedFlushes = 0;
        _grabAuthorityProxyLastFlushDeltaSeconds = 0.0f;
        _grabAuthorityProxyLogCounter = 0;
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
    }

    void Hand::clearGrabAuthorityProxyRuntime()
    {
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
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        destroyGrabAuthorityProxyLocked(bhkWorld);
    }

    void Hand::abandonGrabAuthorityProxyLocked()
    {
        _consumerPairLeases.abandonWorld();
        _grabAuthorityProxy.reset();
        clearGrabAuthorityProxyRuntimeLocked();
    }

    void Hand::abandonGrabAuthorityProxy()
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        abandonGrabAuthorityProxyLocked();
    }

    bool Hand::getHeldBodyContactMesh(RE::hknpWorld* world, std::span<const GrabLocalTriangle>& triangles,
        RE::NiTransform& meshWorld, const HeldContactMeshCache*& cache) const
    {
        triangles = {};
        cache = nullptr;
        RE::NiTransform bodyWorld{};
        if (!world || !isHolding() || !_savedObjectState.isValid() || _grabFrame.localMeshTriangles.empty() ||
            !tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, bodyWorld)) {
            return false;
        }
        meshWorld = deriveNodeWorldFromBodyWorld(bodyWorld, _grabFrame.authority.bodyLocal);
        if (!isUsableGrabVisualTransform(meshWorld)) {
            return false;
        }
        triangles = _grabFrame.localMeshTriangles;
        _grabFrame.contactMeshCache.prepare(triangles);
        cache = &_grabFrame.contactMeshCache;
        return true;
    }

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
        return vector_math::hasFiniteComponents(outPivotWorld);
    }

    bool Hand::createProxyConstraintGrabDrive(RE::bhkWorld* bhkWorld, RE::hknpWorld* world, RE::hknpBodyId objectBodyId,
        const RE::NiTransform& proxyWorldTransform, const RE::NiTransform& rawHandWorldTransform,
        const RE::NiPoint3& grabPivotAWorld, float tau, float damping, float maxForce, float authorityForceScale,
        float proportionalRecovery, float constantRecovery, bool looseWeaponGrab, const char* reason)
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
            ROCK_LOG_ERROR(Hand, "{} hand proxy constraint grab failed: proxy shape creation failed reason={}",
                handName(), reason ? reason : "unknown");
            return false;
        }

        const std::uint32_t proxyFilterInfo = grab_authority_proxy::noContactFilterInfo();
        const auto material = havok_material_registry::registerGeneratedBodyMaterial(world);
        const char* proxyName = _isLeft ? "ROCK_LeftGrabAuthorityProxy" : "ROCK_RightGrabAuthorityProxy";
        if (!_grabAuthorityProxy.create(world, bhkWorld, proxyShape, proxyFilterInfo, material,
                BethesdaMotionType::Keyframed, proxyName)) {
            havok_ref_count::release(proxyShape);
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: proxy body creation failed policy={} filter=0x{:08X} reason={}",
                handName(), grab_authority_proxy::filterPolicyName(), proxyFilterInfo, reason ? reason : "unknown");
            return false;
        }
        havok_ref_count::release(proxyShape);

        const RE::hkTransformf initialProxyHavok = grab_authority_proxy::makeHavokTransform(proxyWorldTransform);
        float zeroVelocity[4]{};
        const bool setTransformOk = _grabAuthorityProxy.setTransform(initialProxyHavok);
        const bool setVelocityOk = _grabAuthorityProxy.setVelocity(zeroVelocity, zeroVelocity);
        if (!setTransformOk || !setVelocityOk) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: initial proxy drive failed setTransform={} setVelocity={} "
                "proxyBody={} reason={}",
                handName(), setTransformOk ? "ok" : "fail", setVelocityOk ? "ok" : "fail",
                _grabAuthorityProxy.getBodyId().value, reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
        initializeGeneratedKeyframedBodyDriveState(_grabAuthorityProxyDriveState, proxyWorldTransform);

        std::uint32_t actualFilterInfo = 0;
        const bool filterReadOk =
            havok_runtime::tryReadFilterInfo(world, _grabAuthorityProxy.getBodyId(), actualFilterInfo);
        if (!filterReadOk || !grab_authority_proxy::hasNoContactFilterInfo(actualFilterInfo)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: proxy no-contact filter invalid read={} filter=0x{:08X} "
                "expectedPolicy={} proxyBody={} reason={}",
                handName(), filterReadOk ? "ok" : "fail", actualFilterInfo, grab_authority_proxy::filterPolicyName(),
                _grabAuthorityProxy.getBodyId().value, reason ? reason : "unknown");
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
        const RE::NiPoint3 pivotAProxyLocalGame = grab_constraint_math::computeGeneratedProxyConstraintPivotLocalGame(
            proxyWorldTransform, constraintPivotAWorld);
        if (!std::isfinite(pivotAProxyLocalGame.x) || !std::isfinite(pivotAProxyLocalGame.y) ||
            !std::isfinite(pivotAProxyLocalGame.z)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: transform-A local pivot is invalid proxyBody={} objBody={} "
                "reason={}",
                handName(), _grabAuthorityProxy.getBodyId().value, objectBodyId.value, reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
        // Constraint creation must seed the ragdoll motor with the generated
        // proxy local BODY relation that held updates keep writing. This parent
        // frame is the same column-authored generated-collider frame used for
        // pivot A, so derive it through the collider local conversion instead
        // of a normal NiTransform inverse. Transform-B stays on the selected
        // BODY-local grip pivot captured by the authority freeze.
        const RE::NiTransform desiredBodyWorldAtCreation = grab_frame_math::objectFromGeneratedProxyLocalSpace(
            proxyWorldTransform, _grabFrame.proxyAuthorityBodyHandSpace);
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyWorldAtCreation);
        const RE::NiPoint3 relationPivotBConstraintLocalGame =
            grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformProxySpace,
                _grabFrame.authority.pivotAHandBodyLocalGame);
        const RE::NiPoint3 solverPivotBConstraintLocalGame = _grabFrame.authority.pivotBConstraintLocalGame;
        if (!std::isfinite(solverPivotBConstraintLocalGame.x) || !std::isfinite(solverPivotBConstraintLocalGame.y) ||
            !std::isfinite(solverPivotBConstraintLocalGame.z)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: transform-B relation pivot is invalid proxyBody={} objBody={} "
                "reason={}",
                handName(), _grabAuthorityProxy.getBodyId().value, objectBodyId.value, reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }
        const float selectionPivotRelationDeltaGameUnits =
            pointDistanceGameUnits(solverPivotBConstraintLocalGame, relationPivotBConstraintLocalGame);

        const float sanitizedAuthorityForceScale = std::clamp(
            std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f, 0.05f, 1.0f) *
            (_savedObjectState.liveLimb ? LIVE_LIMB_AUTHORITY_SCALE : 1.0f);
        const auto massSummaryAtCreation =
            readHeldBodyMassSummary(world, objectBodyId, _heldBodyIds, _heldDriveDecision.includeConnectedMass);
        const float effectiveMassAtCreation = effectiveGrabMotorMass(massSummaryAtCreation.motorMass());
        GrabConstraintMotorTuning motorTuning =
            buildProxyConstraintMotorTuning(tau, damping, maxForce, sanitizedAuthorityForceScale, proportionalRecovery,
                constantRecovery, looseWeaponGrab, effectiveMassAtCreation, g_rockConfig.rockGrabMaxForceToMassRatio);
        if (_savedObjectState.liveLimb)
            motorTuning.angularMaxForce *= LIVE_LIMB_ANGULAR_SHARE;

        _activeConstraint = createGrabConstraint(world, _grabAuthorityProxy.getBodyId(), objectBodyId,
            proxyWorldTransform, constraintPivotAWorld, desiredBodyTransformProxySpace, motorTuning);
        if (!_activeConstraint.isValid()) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: constraint creation failed proxyBody={} objBody={} reason={}",
                handName(), _grabAuthorityProxy.getBodyId().value, objectBodyId.value, reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }

        {
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
                // The first sample is rebase-only in the source clock (no
                // segment exists yet), so it carries no fabricated interval.
                .deltaTime = 0.0f,
                .forceFadeInTime = g_rockConfig.rockGrabForceFadeInTime,
                .tauMin = g_rockConfig.rockGrabTauMin,
                .grabPositionErrorGameUnits = 0.0f,
                .grabRotationErrorDegrees = 0.0f,
                .authorityForceScale = sanitizedAuthorityForceScale,
                .heldBodyColliding = false,
                .valid = true,
            };
            _lastAppliedGrabAuthorityProxyWorld = proxyWorldTransform;
            _lastAppliedGrabAuthorityRawHandWorld = rawHandWorldTransform;
            _hasLastAppliedGrabAuthorityProxyWorld = true;
            _grabAuthoritySourceClock.reset();
            _grabAuthorityProxyQueuedSequence = 1;
            _grabAuthorityProxyFlushSequence = 0;
            _grabAuthorityProxyFailedFlushes = 0;
            _grabAuthorityProxyLastFlushDeltaSeconds = 0.0f;
            _grabAuthorityProxyLogCounter = 0;
            _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand proxy constraint grab drive: constraint={} looseWeapon={} proxyBody={} objBody={} filter=0x{:08X} "
            "pivotAProxy=({:.2f},{:.2f},{:.2f}) pivotBSelected=({:.2f},{:.2f},{:.2f}) "
            "relationPivotB=({:.2f},{:.2f},{:.2f}) selectedPivotRelationDelta={:.3f}gu "
            "pivotBBody=({:.2f},{:.2f},{:.2f}) linearTau={:.3f} angularTau={:.3f} linearForce={:.0f} "
            "angularForce={:.0f} motorMass={:.3f} forceBudget={:.2f} reason={}",
            handName(), _activeConstraint.constraintId, looseWeaponGrab ? "yes" : "no",
            _grabAuthorityProxy.getBodyId().value, objectBodyId.value, actualFilterInfo, pivotAProxyLocalGame.x,
            pivotAProxyLocalGame.y, pivotAProxyLocalGame.z, solverPivotBConstraintLocalGame.x,
            solverPivotBConstraintLocalGame.y, solverPivotBConstraintLocalGame.z, relationPivotBConstraintLocalGame.x,
            relationPivotBConstraintLocalGame.y, relationPivotBConstraintLocalGame.z,
            selectionPivotRelationDeltaGameUnits, _grabFrame.authority.pivotBBodyLocalGame.x,
            _grabFrame.authority.pivotBBodyLocalGame.y, _grabFrame.authority.pivotBBodyLocalGame.z,
            motorTuning.linearTau, motorTuning.angularTau, motorTuning.linearMaxForce, motorTuning.angularMaxForce,
            massSummaryAtCreation.motorMass(), sanitizedAuthorityForceScale, reason ? reason : "unknown");
        return true;
    }

    bool Hand::updateProxyConstraintGrabDriveTarget(RE::hknpWorld* world, const RE::NiTransform& proxyWorldTransform,
        RE::NiTransform& outDesiredObjectWorld, RE::NiTransform& outDesiredBodyWorld,
        RE::NiPoint3& outDesiredTargetPointWorld, RE::NiPoint3& outActivePivotBBodyLocalGame)
    {
        const RE::NiTransform desiredBodyRelationProxySpace = _grabFrame.proxyAuthorityBodyHandSpace;
        outDesiredObjectWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform,
            _grabFrame.proxyAuthorityHandSpace);
        outDesiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyRelationProxySpace);
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorldTransform, outDesiredBodyWorld);
        const RE::NiPoint3 selectedPivotBBodyLocalGame = activeProxyConstraintPivotBLocalGame();
        const RE::NiPoint3 relationPivotB = grab_constraint_math::computeHiggsTransformBTranslationGame(
            desiredBodyTransformProxySpace, _grabFrame.authority.pivotAHandBodyLocalGame);
        outActivePivotBBodyLocalGame = relationPivotB;
        outDesiredTargetPointWorld =
            transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);

        if (!world || !_activeConstraint.isValid() || !_activeConstraint.constraintData ||
            !_grabAuthorityProxy.isValid() || !_grabAuthorityProxyFrameValid ||
            _grabAuthorityProxy.getBodyId().value == INVALID_BODY_ID) {
            return false;
        }

        /*
         * Transform-A is part of the frozen local constraint frame and is not
         * rewritten while held. The frozen capture chooses the ragdoll
         * transform-B decomposition once, while the current proxy-in-BODY
         * relation keeps updating target_bRca and transform-B translation.
         */
        const RE::NiPoint3 pivotAProxyLocalGame = _grabAuthorityPivotAProxyLocalGame;
        if (!std::isfinite(pivotAProxyLocalGame.x) || !std::isfinite(pivotAProxyLocalGame.y) ||
            !std::isfinite(pivotAProxyLocalGame.z)) {
            return false;
        }

        auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
        const float gameToHkScale = gameToHavokScale();

        auto* transformBRotation = reinterpret_cast<float*>(constraintData + GRAB_TRANSFORM_B_COL0);
        auto* transformBTranslation = reinterpret_cast<float*>(constraintData + GRAB_TRANSFORM_B_POS);
        auto* targetBRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + RAGDOLL_MOTOR_TARGET_BRCA);
        grab_constraint_math::writeGrabConstraintHeldTargetAtoms(transformBRotation, transformBTranslation, targetBRca,
            desiredBodyTransformProxySpace, pivotAProxyLocalGame, gameToHkScale);
        outDesiredTargetPointWorld =
            transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);

        return true;
    }

    bool Hand::resolveGrabAuthorityProxyFrame(RE::hknpWorld* world, RE::NiTransform& outProxyWorld,
        const char*& outSource, Hand::GrabAuthorityProxyFramePolicy policy) const
    {
        auto isFiniteProxyFrameInput = [](const RE::NiTransform& transform) {
            bool rotationFinite = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite && std::isfinite(transform.rotate.entry[row][column]);
                }
            }
            return rotationFinite && std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
                std::isfinite(transform.translate.z) && std::isfinite(transform.scale) && transform.scale > 0.0001f;
        };

        if (policy == GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget) {
            RE::NiTransform queuedPalmAnchorTarget{};
            if (tryGetPalmAnchorTarget(queuedPalmAnchorTarget) && isFiniteProxyFrameInput(queuedPalmAnchorTarget)) {
                const RE::NiTransform proxyBaseWorld =
                    hand_bone_collider_geometry_math::generatedColliderFrameToGrabAuthorityFrame(
                        queuedPalmAnchorTarget);
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

    bool Hand::resolveActiveGrabAuthorityPivotAWorld(const RE::NiTransform& proxyWorldTransform,
        RE::NiPoint3& outPivotWorld) const
    {
        outPivotWorld = {};
        if (!_grabFrame.hasTelemetryCapture || !_grabFrame.authority.hasFrozenPivotB ||
            !std::isfinite(_grabFrame.authority.pivotAHandBodyLocalGame.x) ||
            !std::isfinite(_grabFrame.authority.pivotAHandBodyLocalGame.y) ||
            !std::isfinite(_grabFrame.authority.pivotAHandBodyLocalGame.z)) {
            return false;
        }

        /*
         * Pivot A is frozen as a generated/proxy local point at grab commit.
         * Held updates replay that local point through the current proxy body
         * frame instead of recomputing palm or pinch seats from raw hand space.
         */
        outPivotWorld =
            generatedProxyLocalPointToWorld(proxyWorldTransform, _grabFrame.authority.pivotAHandBodyLocalGame);
        return std::isfinite(outPivotWorld.x) && std::isfinite(outPivotWorld.y) && std::isfinite(outPivotWorld.z);
    }

    bool Hand::updateConstraintGrabDriveMotors(RE::hknpWorld* world, float deltaTime, float forceFadeInTime,
        float tauMin, float authorityForceScale, bool heldBodyColliding,
        const grab_motion_controller::HeldAuthorityState& heldAuthority)
    {
        if (!_activeConstraint.isValid() || !_activeConstraint.linearMotor || !_activeConstraint.angularMotor) {
            return false;
        }

        const float looseLinearTauMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier);
        const float looseAngularTauMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier);
        const float looseCollisionTauMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier);
        const float looseLinearDampingMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier);
        const float looseAngularDampingMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier);
        const float looseMaxForceMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier);
        const float looseAngularForceMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier);
        const float looseLinearRecoveryMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
        const float looseAngularRecoveryMultiplier = looseWeaponMultiplier(_heldObjectIsLooseWeapon,
            g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);
        const float sharedBaseMaxForce =
            scaleDriveValue(g_rockConfig.rockGrabConstraintMaxForce, looseMaxForceMultiplier);

        const auto massSummary = readHeldBodyMassSummary(world, _savedObjectState.bodyId, _heldBodyIds,
            _heldDriveDecision.includeConnectedMass);
        _activeConstraint.motorBodyProperties =
            readGrabMotorBodyProperties(world, _savedObjectState.bodyId, activeProxyConstraintPivotBLocalGame());
        const auto& properties = _activeConstraint.motorBodyProperties;
        const auto motorInput = grab_motion_controller::MotorInput{
            .heldBodyColliding = heldBodyColliding,
            .baseLinearTau = scaleDriveValue(g_rockConfig.rockGrabLinearTau, looseLinearTauMultiplier),
            .baseAngularTau = scaleDriveValue(g_rockConfig.rockGrabAngularTau, looseAngularTauMultiplier),
            .collisionTau = scaleDriveValue(tauMin, looseCollisionTauMultiplier),
            .currentLinearTau = _activeConstraint.linearMotor->tau,
            .currentAngularTau = _activeConstraint.angularMotor->tau,
            .tauLerpSpeed = g_rockConfig.rockGrabTauLerpSpeed,
            .deltaTime = deltaTime,
            .physicsDeltaSeconds = deltaTime,
            .baseMaxForce = sharedBaseMaxForce,
            .authorityForceScale = authorityForceScale * (_savedObjectState.liveLimb ? LIVE_LIMB_AUTHORITY_SCALE : 1.0f),
            .angularForceMultiplier = looseAngularForceMultiplier,
            .mass = massSummary.motorMass(),
            .maximumInertia = properties.valid ? properties.maximumInertia : 0.0f,
            .gripRadiusHavok = properties.gripRadiusHavok,
            .freeLinearAcceleration = g_rockConfig.rockGrabFreeLinearAcceleration,
            .freeAngularAcceleration = g_rockConfig.rockGrabFreeAngularAcceleration,
            .forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio,
            .effectiveMotorMassFloorEnabled = g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
            .effectiveMotorMassFloor = g_rockConfig.rockGrabEffectiveMotorMassFloor,
            .fadeInEnabled = _grabFrame.fadeInGrabConstraint,
            .fadeElapsed = _grabStartTime,
            .fadeDuration = forceFadeInTime,
        };
        auto output = grab_motion_controller::solveMotorTargetsWithAuthority(motorInput, heldAuthority);
        if (_savedObjectState.liveLimb)
            output.angularMaxForce *= LIVE_LIMB_ANGULAR_SHARE;
        if (!output.valid) {
            _activeConstraint.linearMotor->minForce = _activeConstraint.linearMotor->maxForce = 0.0f;
            _activeConstraint.angularMotor->minForce = _activeConstraint.angularMotor->maxForce = 0.0f;
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "{} grab motor properties unavailable; release queued body={}", handName(),
                _savedObjectState.bodyId.value);
            return false;
        }
        _lastGrabPhysicsHz.store(output.physicsHz, std::memory_order_relaxed);

        _activeConstraint.linearMotor->tau = output.linearTau;
        _activeConstraint.linearMotor->damping =
            scaleDriveValue(g_rockConfig.rockGrabLinearDamping, looseLinearDampingMultiplier);
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
        return true;
    }

    void Hand::queueProxyGrabAuthorityTarget(const RE::NiTransform& proxyWorldTransform,
        const RE::NiTransform& rawHandWorldTransform, const char* proxyFrameSource, float deltaTime,
        float forceFadeInTime, float tauMin, float grabPositionErrorGameUnits, float grabRotationErrorDegrees,
        float authorityForceScale, bool heldBodyColliding)
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if (!_grabAuthorityProxy.isValid()) {
            return;
        }

        _grabAuthorityPendingTarget.proxyWorld = proxyWorldTransform;
        _grabAuthorityPendingTarget.rawHandWorld = rawHandWorldTransform;
        _grabAuthorityPendingTarget.proxyFrameSource = proxyFrameSource ? proxyFrameSource : "unknown";
        _grabAuthorityPendingTarget.deltaTime = deltaTime;
        _grabAuthorityPendingTarget.forceFadeInTime = forceFadeInTime;
        _grabAuthorityPendingTarget.tauMin = tauMin;
        _grabAuthorityPendingTarget.grabPositionErrorGameUnits = grabPositionErrorGameUnits;
        _grabAuthorityPendingTarget.grabRotationErrorDegrees = grabRotationErrorDegrees;
        _grabAuthorityPendingTarget.authorityForceScale = std::clamp(
            std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f, 0.05f, 1.0f);
        _grabAuthorityPendingTarget.heldBodyColliding = heldBodyColliding;
        _grabAuthorityPendingTarget.valid = true;
        ++_grabAuthorityProxyQueuedSequence;
    }

    bool Hand::promoteHeldObjectToConstraintDrive(RE::bhkWorld* bhkWorld, RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform, float tau, float damping, float maxForce, float proportionalRecovery,
        float constantRecovery, const char* reason)
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
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            _grabAuthorityPendingTarget.authorityForceScale =
                held_object_drive_policy::sanitizeMotorAuthorityScale(sharedGrabAuthorityForceScale(true));
            ROCK_LOG_DEBUG(Hand,
                "{} hand peer promotion kept existing proxy constraint drive: formID={:08X} body={} forceBudget={:.2f} "
                "driveMode={} reason={}",
                handName(), _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
                _savedObjectState.bodyId.value, _grabAuthorityPendingTarget.authorityForceScale,
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                reason ? reason : "peer-joined-held-object");
            return true;
        }

        ROCK_LOG_WARN(Hand,
            "{} hand peer promotion failed: held object has no proxy constraint authority formID={:08X} body={} "
            "reason={}",
            handName(), _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
            _savedObjectState.bodyId.value, reason ? reason : "peer-joined-held-object");
        return false;
    }

    void Hand::flushPendingCustomGrabAuthority(RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        GrabAuthorityProxyPendingTarget pending{};
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
        std::uint32_t resampleRebaseCount = 0;
        GrabAngularAuthority angularAuthority = GrabAngularAuthority::HknpRagdollMotorAtom;
        float driveDelta = 0.0f;
        {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            const bool hasAuthority = _grabAuthorityProxy.isValid() && _grabAuthorityProxyHknpWorld == world &&
                _grabAuthorityPendingTarget.valid;
            if (!hasAuthority) {
                return;
            }
            if (!havok_physics_timing::tryGetDriveDeltaSeconds(timing, driveDelta)) {
                /*
                 * No measured native physics time this substep: skip the whole
                 * flush. The pending target and its queued sequence stay
                 * intact, so the next measured substep consumes the same
                 * game-source sample and the phase lock is preserved.
                 */
                ROCK_LOG_SAMPLE_WARN(Hand, 1000,
                    "{} hand grab authority flush skipped unmeasured physics timing substep={}/{}", handName(),
                    timing.substepIndex, timing.substepCount);
                return;
            }
            performance_profiler::addEventCount(performance_profiler::Scope::GrabAuthorityFlush);
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAuthorityFlush);

            pending = _grabAuthorityPendingTarget;
            // Accept the game-frame sample exactly once; the queued-sequence
            // identity keeps multi-substep re-flushes of the same pending
            // target from advancing the source segment. The drive target is
            // then phase-locked to the game clock per substep below. See
            // GrabAuthoritySourceClockResampler.h for the contract.
            _grabAuthoritySourceClock.advanceSource(pending.proxyWorld.translate, pending.proxyWorld.rotate,
                pending.deltaTime, _grabAuthorityProxyQueuedSequence);
            livePalmReferenceOk = tryResolveLivePalmAnchorReference(world, livePalmReference);
            /*
             * The game-frame held update owns pending.proxyWorld. Rebinding it
             * here to the live palm body reintroduced one-Havok-tick latency
             * during stick locomotion, even after collider targets were bridged.
             * The live palm body supplies only the proxy's angular velocity and
             * telemetry. A palm collider that is missing or being rebuilt must
             * not stop the drive: that released the held object.
             */
            previousProxyWorld =
                _hasLastAppliedGrabAuthorityProxyWorld ? _lastAppliedGrabAuthorityProxyWorld : pending.proxyWorld;
            proxyBodyId = _grabAuthorityProxy.getBodyId();
            angularAuthority = _activeConstraint.angularAuthority;

            // Game-clock phase lock: the frame's last substep commands EXACTLY
            // the queued game-frame sample, so frame-end proxy positions lie on
            // the sampled wand path the same way the hand collider's do -- the
            // 2026-07-13 OVERLAY_POINT probe proved physics-clock playback put
            // v x (clock mismatch) between the held object and everything else
            // the eye tracks. Intra-frame substeps interpolate along the
            // sample segment. Local copy only: every downstream use in this
            // flush -- keyframe drive, constraint target, motors, readback
            // diagnostics, last-applied tracking -- sees the locked target
            // consistently, while the stored pending target stays the raw
            // sample. Rotation deliberately stays on the sampled path.
            pending.proxyWorld.translate =
                _grabAuthoritySourceClock.evaluate(timing.substepIndex, timing.substepCount, resampleAction);
            resampleRebaseCount = _grabAuthoritySourceClock.rebaseCount;
            _lastGrabSourceIntervalSeconds.store(_grabAuthoritySourceClock.filteredSourceIntervalSeconds,
                std::memory_order_relaxed);
            float linearVelocityHavok[4]{};
            float angularVelocityHavok[4]{};
            float nativeLinearVelocityIgnored[4]{};
            grab_authority_proxy::computeLinearVelocityHavok(previousProxyWorld, pending.proxyWorld, driveDelta,
                linearVelocityHavok);
            if (livePalmReference.hasMotionVelocity) {
                angularVelocityHavok[0] = livePalmReference.angularVelocityRadiansPerSecond.x;
                angularVelocityHavok[1] = livePalmReference.angularVelocityRadiansPerSecond.y;
                angularVelocityHavok[2] = livePalmReference.angularVelocityRadiansPerSecond.z;
                proxyVelocityTelemetryOk = true;
            } else {
                proxyVelocityTelemetryOk = computeHardKeyframeVelocityForTarget(world, proxyBodyId, pending.proxyWorld,
                    driveDelta, nativeLinearVelocityIgnored, angularVelocityHavok);
            }
            proxyLinearVelocityHavokMagnitude = std::sqrt(linearVelocityHavok[0] * linearVelocityHavok[0] +
                linearVelocityHavok[1] * linearVelocityHavok[1] + linearVelocityHavok[2] * linearVelocityHavok[2]);
            proxyAngularVelocityRadiansPerSecond = std::sqrt(angularVelocityHavok[0] * angularVelocityHavok[0] +
                angularVelocityHavok[1] * angularVelocityHavok[1] + angularVelocityHavok[2] * angularVelocityHavok[2]);

            queueGeneratedKeyframedBodyTarget(_grabAuthorityProxyDriveState, pending.proxyWorld, driveDelta, 1000.0f);
            proxyDriveResult = driveGeneratedKeyframedBody(world, _grabAuthorityProxy, _grabAuthorityProxyDriveState,
                timing, "grab-authority-proxy", 0, g_rockConfig.rockHandBoneColliderMaxLinearVelocity,
                g_rockConfig.rockHandBoneColliderMaxAngularVelocity);
            proxyDriveOk = proxyDriveResult.driven;
            if (proxyDriveOk) {
                proxyReadbackBetweenOk = tryResolveLiveBodyWorldTransform(world, proxyBodyId, proxyReadbackBetween,
                    &proxyReadbackSourceBetween, &proxyReadbackMotionIndexBetween);
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
                targetUpdateOk = updateProxyConstraintGrabDriveTarget(world, pending.proxyWorld, desiredObjectWorld,
                    desiredBodyWorld, desiredTargetPointWorld, activePivotBBodyLocalGame);
                if (!targetUpdateOk) {
                    ++_grabAuthorityProxyFailedFlushes;
                    _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
                } else {
                    const auto pendingHeldAuthority =
                        grab_motion_controller::evaluateHeldAuthority(pending.heldBodyColliding);
                    const bool motorsReady = updateConstraintGrabDriveMotors(world, driveDelta, pending.forceFadeInTime,
                        pending.tauMin, pending.authorityForceScale, pending.heldBodyColliding, pendingHeldAuthority);
                    angularDriveOk = motorsReady && _activeConstraint.isValid() &&
                        _activeConstraint.usesRagdollAngularMotorAtom() && _activeConstraint.linearMotor &&
                        _activeConstraint.angularMotor;
                    if (_activeConstraint.angularMotor) {
                        angularMotorBudget = (std::max)(std::fabs(_activeConstraint.angularMotor->minForce),
                            std::fabs(_activeConstraint.angularMotor->maxForce));
                    }
                    if (!angularDriveOk) {
                        ++_grabAuthorityProxyFailedFlushes;
                        _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
                    }

                    _lastAppliedGrabAuthorityProxyWorld = pending.proxyWorld;
                    _lastAppliedGrabAuthorityRawHandWorld = pending.rawHandWorld;
                    _hasLastAppliedGrabAuthorityProxyWorld = true;
                    _grabAuthorityProxyLastFlushDeltaSeconds = driveDelta;
                    ++_grabAuthorityProxyFlushSequence;
                    flushSequence = _grabAuthorityProxyFlushSequence;
                    queuedSequence = _grabAuthorityProxyQueuedSequence;
                    ++_grabAuthorityProxyLogCounter;
                    if (flushSequence <= 16 || _grabAuthorityProxyLogCounter >= 45 || !proxyReadbackBetweenOk ||
                        !angularDriveOk || resampleAction == grab_authority_source_clock::ResampleAction::Rebase ||
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
                "{} hand proxy dynamic grab drive failed; release queued: proxyBody={} livePalm={} driven={} stale={} "
                "missing={} ownerMismatch={} substep={}/{} dt={:.6f}",
                handName(), proxyBodyId.value, livePalmReferenceOk ? "ok" : "fail",
                proxyDriveResult.driven ? "ok" : "fail", proxyDriveResult.skippedStale ? "yes" : "no",
                proxyDriveResult.missingBody ? "yes" : "no",
                proxyDriveResult.bodyCollisionObjectMismatch ? "yes" : "no", timing.substepIndex, timing.substepCount,
                timing.substepDeltaSeconds);
            return;
        }

        if (!targetUpdateOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab target update failed; release queued: proxyBody={} constraint={} "
                "substep={}/{}",
                handName(), proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu, timing.substepIndex,
                timing.substepCount);
            return;
        }

        if (!angularDriveOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab angular drive failed; release queued: proxyBody={} objBody={} "
                "constraint={} substep={}/{}",
                handName(), proxyBodyId.value, _savedObjectState.bodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu, timing.substepIndex,
                timing.substepCount);
            return;
        }
    }

    bool Hand::tryGetGrabOverlayPointProbeSample(RE::hknpWorld* world, GrabOverlayPointProbeSample& out)
    {
        out = {};
        if (!world) {
            return false;
        }

        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if (!_grabAuthorityProxy.isValid() || _grabAuthorityProxyHknpWorld != world ||
            !_hasLastAppliedGrabAuthorityProxyWorld) {
            return false;
        }

        out.appliedProxyTargetWorld = _lastAppliedGrabAuthorityProxyWorld;
        out.appliedRawHandWorld = _lastAppliedGrabAuthorityRawHandWorld;
        out.proxyBodyId = _grabAuthorityProxy.getBodyId();
        out.objectBodyId = _savedObjectState.bodyId;
        out.flushSequence = _grabAuthorityProxyFlushSequence;
        return true;
    }

    void Hand::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world) {
            return;
        }

        // Post-solve object velocity for release momentum. It is sampled under
        // the proxy mutex so release never reads a half-written sample.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        const bool hasAuthority = _grabAuthorityProxy.isValid() && _grabAuthorityProxyHknpWorld == world &&
            _hasLastAppliedGrabAuthorityProxyWorld;
        _releaseObjectSample = {};
        if (!hasAuthority || !timing.valid || timing.usedFallback) {
            return;
        }

        const auto* body = havok_runtime::getBody(world, _savedObjectState.bodyId);
        const auto* motion = body && body_frame::hasUsableMotionIndex(body->motionIndex)
            ? havok_runtime::getMotion(world, body->motionIndex)
            : nullptr;
        if (motion) {
            _releaseObjectSample = { world, _savedObjectState.bodyId.value, _grabFrame.traceId, timing.solveSequence,
                release_velocity::sampleTimeSeconds(),
                { motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z } };
        }
    }
}

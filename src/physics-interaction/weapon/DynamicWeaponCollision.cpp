#include "physics-interaction/weapon/DynamicWeaponCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/native/HavokConvexShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/WeaponCollision.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
        constexpr std::uint32_t kDynamicWeaponCollisionGroup = 0x000Du;
        constexpr std::uint32_t kRaiseManifoldProcessedEvents = 0x40u;
        constexpr std::uint32_t kRebuildBodyCollisionState = 0u;
        constexpr std::uint32_t kContactGraceSolves = 3;
        constexpr float kMaxVisualCorrectionRotationDegrees = 85.0f;

        std::uint32_t dynamicWeaponProxyFilterInfo()
        {
            return (kDynamicWeaponCollisionGroup << 16) |
                   (collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY & collision_layer_policy::FO4_LAYER_FILTER_MASK);
        }

        float scaleFiniteValue(const float value, const float multiplier)
        {
            return (std::isfinite(value) ? value : 0.0f) * (std::isfinite(multiplier) ? multiplier : 1.0f);
        }

        GrabConstraintMotorTuning buildWeaponGripConstraintTuning(const float bodyMass)
        {
            const float effectiveMotorMass = grab_motion_controller::effectiveMotorMass(
                bodyMass,
                g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
                g_rockConfig.rockGrabEffectiveMotorMassFloor);
            const float linearBudget = (std::max)(0.0f, scaleFiniteValue(
                g_rockConfig.rockGrabConstraintMaxForce,
                g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier));
            const float linearMaxForce = grab_motion_controller::capForceByMass(
                linearBudget,
                effectiveMotorMass,
                g_rockConfig.rockGrabMaxForceToMassRatio);

            return GrabConstraintMotorTuning{
                .linearTau = scaleFiniteValue(
                    g_rockConfig.rockGrabLinearTau,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier),
                .linearDamping = scaleFiniteValue(
                    g_rockConfig.rockGrabLinearDamping,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier),
                .linearProportionalRecovery = scaleFiniteValue(
                    g_rockConfig.rockGrabLinearProportionalRecovery,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier),
                .linearConstantRecovery = scaleFiniteValue(
                    g_rockConfig.rockGrabLinearConstantRecovery,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier),
                .linearMaxForce = linearMaxForce,
                .angularTau = scaleFiniteValue(
                    g_rockConfig.rockGrabAngularTau,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier),
                .angularDamping = scaleFiniteValue(
                    g_rockConfig.rockGrabAngularDamping,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier),
                .angularProportionalRecovery = scaleFiniteValue(
                    g_rockConfig.rockGrabAngularProportionalRecovery,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier),
                .angularConstantRecovery = scaleFiniteValue(
                    g_rockConfig.rockGrabAngularConstantRecovery,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier),
                .angularMaxForce = linearMaxForce * (std::isfinite(g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier) ?
                        g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier :
                        1.0f),
            };
        }

        bool applyWeaponBoxMassProperties(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            const dynamic_weapon_collision_policy::BoxGeometry& geometry,
            const float weaponScale,
            const float paddingGameUnits,
            const float bodyMass)
        {
            const auto boxMassProperties = dynamic_weapon_collision_policy::makeBoxMassProperties(
                geometry,
                weaponScale,
                paddingGameUnits,
                physics_scale::gameToHavok(),
                bodyMass);
            if (!boxMassProperties.valid) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon box mass properties invalid: body={} scale={:.3f} padding={:.3f} mass={:.3f}",
                    bodyId.value,
                    weaponScale,
                    paddingGameUnits,
                    bodyMass);
                return false;
            }

            const auto normalizedInertia = grab_inertia_policy::normalizeInverseInertiaAxesForGrab(
                boxMassProperties.inverseInertia.x,
                boxMassProperties.inverseInertia.y,
                boxMassProperties.inverseInertia.z,
                g_rockConfig.rockGrabMaxInertiaRatio,
                g_rockConfig.rockGrabMinInertia);
            if (!normalizedInertia.valid) {
                ROCK_LOG_ERROR(Weapon, "Dynamic weapon box inertia normalization failed: body={}", bodyId.value);
                return false;
            }

            const auto initialMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!initialMotion.valid || !initialMotion.motion) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon box motion unavailable for mass properties: body={} readable={} motion={}",
                    bodyId.value,
                    initialMotion.valid,
                    initialMotion.motion != nullptr);
                return false;
            }

            auto* initialPacked = reinterpret_cast<std::int16_t*>(
                reinterpret_cast<char*>(initialMotion.motion) + MOTION_PACKED_INERTIA_OFFSET);
            const std::int16_t desiredPackedInertia[3] = {
                repackBfloat16(normalizedInertia.normalized[0]),
                repackBfloat16(normalizedInertia.normalized[1]),
                repackBfloat16(normalizedInertia.normalized[2]),
            };
            const std::int16_t desiredPackedMass = initialPacked[3];
            if (desiredPackedInertia[0] <= 0 || desiredPackedInertia[1] <= 0 || desiredPackedInertia[2] <= 0 ||
                desiredPackedMass <= 0) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon box packed mass properties invalid: body={} inertia=[{},{},{}] inverseMass={}",
                    bodyId.value,
                    desiredPackedInertia[0],
                    desiredPackedInertia[1],
                    desiredPackedInertia[2],
                    desiredPackedMass);
                return false;
            }

            initialPacked[0] = desiredPackedInertia[0];
            initialPacked[1] = desiredPackedInertia[1];
            initialPacked[2] = desiredPackedInertia[2];
            if (!havok_runtime::rebuildMotionMassProperties(world, initialMotion.motionIndex)) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon box mass-properties rebuild failed: body={} motion={}",
                    bodyId.value,
                    initialMotion.motionIndex);
                return false;
            }

            const auto rebuiltMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!rebuiltMotion.valid || !rebuiltMotion.motion || rebuiltMotion.motionIndex != initialMotion.motionIndex) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon box motion changed during mass-properties rebuild: body={} before={} after={} readable={}",
                    bodyId.value,
                    initialMotion.motionIndex,
                    rebuiltMotion.motionIndex,
                    rebuiltMotion.valid && rebuiltMotion.motion);
                return false;
            }

            auto* rebuiltPacked = reinterpret_cast<std::int16_t*>(
                reinterpret_cast<char*>(rebuiltMotion.motion) + MOTION_PACKED_INERTIA_OFFSET);
            rebuiltPacked[0] = desiredPackedInertia[0];
            rebuiltPacked[1] = desiredPackedInertia[1];
            rebuiltPacked[2] = desiredPackedInertia[2];
            rebuiltPacked[3] = desiredPackedMass;

            ROCK_LOG_INFO(
                Weapon,
                "Dynamic weapon box mass properties: body={} motion={} halfHavok=({:.4f},{:.4f},{:.4f}) physicalInverseInertia=({:.6f},{:.6f},{:.6f}) appliedInverseInertia=({:.6f},{:.6f},{:.6f}) packed=[{},{},{}] inverseMass={:.6f} ratio={:.2f}->{:.2f}",
                bodyId.value,
                rebuiltMotion.motionIndex,
                boxMassProperties.halfExtentsHavok.x,
                boxMassProperties.halfExtentsHavok.y,
                boxMassProperties.halfExtentsHavok.z,
                boxMassProperties.inverseInertia.x,
                boxMassProperties.inverseInertia.y,
                boxMassProperties.inverseInertia.z,
                unpackBfloat16(rebuiltPacked[0]),
                unpackBfloat16(rebuiltPacked[1]),
                unpackBfloat16(rebuiltPacked[2]),
                rebuiltPacked[0],
                rebuiltPacked[1],
                rebuiltPacked[2],
                unpackBfloat16(rebuiltPacked[3]),
                normalizedInertia.originalRatio,
                normalizedInertia.normalizedRatio);
            return true;
        }
    }

    void DynamicWeaponCollisionRuntime::beginFrame(
        const std::uint64_t frameIndex,
        RE::hknpWorld* world,
        void* bhkWorld,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const bool enabled,
        const bool suppressDefaultNativeIntent)
    {
        _frameIndex = frameIndex;
        _frameWorld = world;
        _frameBhkWorld = bhkWorld;
        _frameWeaponNode = weaponNode;
        _frameGenerationKey = weaponGenerationKey;
        _frameRequestedWeaponWorld = {};
        _frameAcceptingIntent = enabled && world && bhkWorld && weaponNode && weaponGenerationKey != 0;
        _frameHasIntent =
            _frameAcceptingIntent &&
            !suppressDefaultNativeIntent &&
            dynamic_weapon_collision_policy::isFiniteTransform(weaponNode->world);
        if (_frameHasIntent) {
            // Native/F4SE visual authority is the collision-free intent when
            // ROCK does not publish a later manual-grip or gunstock pose.
            _frameRequestedWeaponWorld = weaponNode->world;
        }
        _enabledAtomic.store(_frameAcceptingIntent, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::observeWeaponVisualIntent(
        void* context,
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const std::uint64_t weaponGenerationKey)
    {
        auto* runtime = static_cast<DynamicWeaponCollisionRuntime*>(context);
        if (runtime) {
            runtime->captureVisualIntent(weaponNode, requestedWeaponWorld, weaponGenerationKey);
        }
    }

    void DynamicWeaponCollisionRuntime::captureVisualIntent(
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const std::uint64_t weaponGenerationKey)
    {
        if (!_frameAcceptingIntent || weaponNode != _frameWeaponNode || weaponGenerationKey != _frameGenerationKey ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeaponWorld)) {
            return;
        }
        _frameRequestedWeaponWorld = requestedWeaponWorld;
        _frameHasIntent = true;
    }

    DynamicWeaponCollisionRuntime::FrameResult DynamicWeaponCollisionRuntime::finishFrame(
        const PhysicsFrameContext& frame,
        const bool physicsWritesAllowed,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        FrameResult result{};
        result.requestedWeaponWorld = _frameRequestedWeaponWorld;
        result.resolvedWeaponWorld = _frameRequestedWeaponWorld;

        const bool frameMatches =
            _frameAcceptingIntent &&
            _frameHasIntent &&
            physicsWritesAllowed &&
            frame.worldReady &&
            frame.hknpWorld == _frameWorld &&
            frame.bhkWorld == _frameBhkWorld &&
            weaponNode == _frameWeaponNode &&
            weaponGenerationKey == _frameGenerationKey &&
            !frame.menuBlocked;
        if (!frameMatches || !ensureProxyBody(frame, weaponCollision, _frameRequestedWeaponWorld)) {
            if (_created) {
                retireAll(frame.bhkWorld);
            }
            _debugSnapshot = {};
            return result;
        }

        result.proxyActive = true;
        const RE::NiTransform requestedAuthorityTarget =
            dynamic_weapon_collision_policy::makeGripAuthorityTarget(_frameRequestedWeaponWorld);
        const auto queueResult = queueGeneratedKeyframedBodyTarget(
            _authorityDriveState,
            requestedAuthorityTarget,
            frame.deltaSeconds,
            g_rockConfig.rockWeaponCollisionDynamicDivergenceTeleportGameUnits);
        if (!queueResult.queued) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
        }

        PhysicsSnapshot snapshot{};
        const bool snapshotReadable = readPhysicsSnapshot(snapshot);
        const bool snapshotIdentityCurrent =
            snapshotReadable &&
            snapshot.valid &&
            snapshot.world == reinterpret_cast<std::uintptr_t>(frame.hknpWorld) &&
            snapshot.bodyId == _body.getBodyId().value &&
            snapshot.generationKey == _createdGenerationKey;
        const bool snapshotCurrent =
            snapshotIdentityCurrent &&
            snapshot.contactActive &&
            !snapshot.teleported;

        _debugSnapshot = {};
        _debugSnapshot.valid = true;
        _debugSnapshot.physicsSnapshotReadable = snapshotReadable;
        _debugSnapshot.physicsSnapshotValid = snapshotReadable && snapshot.valid;
        _debugSnapshot.physicsSnapshotIdentityCurrent = snapshotIdentityCurrent;
        _debugSnapshot.physicsSnapshotContactActive = snapshotReadable && snapshot.contactActive;
        _debugSnapshot.physicsSnapshotTeleported = snapshotReadable && snapshot.teleported;
        _debugSnapshot.bodyId = _body.getBodyId().value;
        _debugSnapshot.authorityBodyId = _authorityProxy.getBodyId().value;
        _debugSnapshot.constraintId = _authorityConstraint.constraintId;
        _debugSnapshot.generationKey = _createdGenerationKey;
        _debugSnapshot.proxyPairCallbackSequence = _proxyPairCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.worldSurfaceCallbackSequence = _worldSurfaceCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.rawPointCallbackSequence = _rawPointCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.processedManifoldCallbackSequence = _processedManifoldCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.admittedContactSequence = _contactSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.centerWeaponLocal = _createdCenterWeaponLocal;
        _debugSnapshot.halfExtentsWeaponLocal = _createdHalfExtentsWeaponLocal;
        _debugSnapshot.requestedWeaponWorld = _frameRequestedWeaponWorld;

        const auto logPipelineStage = [&](const char* stage) {
            if (!g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
                return;
            }
            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                500,
                "DWC pipeline: stage={} contactBody={} authorityBody={} constraint={} callbacks(pair/world/raw/manifold/admit)={}/{}/{}/{}/{} snapshot(read/valid/identity/contact/teleport)={}/{}/{}/{}/{} gripPivotError={:.2f}gu angularYield={:.2f}deg visual={}",
                stage,
                _debugSnapshot.bodyId,
                _debugSnapshot.authorityBodyId,
                _debugSnapshot.constraintId,
                _debugSnapshot.proxyPairCallbackSequence,
                _debugSnapshot.worldSurfaceCallbackSequence,
                _debugSnapshot.rawPointCallbackSequence,
                _debugSnapshot.processedManifoldCallbackSequence,
                _debugSnapshot.admittedContactSequence,
                _debugSnapshot.physicsSnapshotReadable,
                _debugSnapshot.physicsSnapshotValid,
                _debugSnapshot.physicsSnapshotIdentityCurrent,
                _debugSnapshot.physicsSnapshotContactActive,
                _debugSnapshot.physicsSnapshotTeleported,
                result.translationCorrectionGameUnits,
                result.rotationCorrectionDegrees,
                result.applyVisualCorrection);
        };

        if (!snapshotCurrent) {
            logPipelineStage("snapshot-gate");
            return result;
        }

        const RE::NiTransform sampledRequestedWeaponWorld = dynamic_weapon_collision_policy::reconstructWeaponRoot(
            snapshot.requestedProxyBodyWorld,
            _createdCenterWeaponLocal,
            snapshot.weaponScale);
        const RE::NiTransform sampledLiveWeaponWorld = dynamic_weapon_collision_policy::reconstructWeaponRoot(
            snapshot.liveProxyBodyWorld,
            _createdCenterWeaponLocal,
            snapshot.weaponScale);
        const RE::NiTransform resolvedWeaponWorld = dynamic_weapon_collision_policy::resolveCurrentIntentFromSample(
            snapshot.requestedProxyBodyWorld,
            snapshot.liveProxyBodyWorld,
            _createdCenterWeaponLocal,
            snapshot.weaponScale,
            _frameRequestedWeaponWorld);
        if (!dynamic_weapon_collision_policy::isFiniteTransform(sampledRequestedWeaponWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(sampledLiveWeaponWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(resolvedWeaponWorld)) {
            logPipelineStage("transform-gate");
            return result;
        }

        result.translationCorrectionGameUnits = dynamic_weapon_collision_policy::translationDeltaGameUnits(
            resolvedWeaponWorld,
            _frameRequestedWeaponWorld);
        result.rotationCorrectionDegrees = dynamic_weapon_collision_policy::rotationDeltaDegrees(
            resolvedWeaponWorld,
            _frameRequestedWeaponWorld);
        _debugSnapshot.contactActive = true;
        _debugSnapshot.otherBodyId = snapshot.otherBodyId;
        _debugSnapshot.otherLayer = snapshot.otherLayer;
        _debugSnapshot.contactGraceSolves = snapshot.contactGraceSolves;
        _debugSnapshot.solveSequence = snapshot.solveSequence;
        _debugSnapshot.liveWeaponWorld = sampledLiveWeaponWorld;
        _debugSnapshot.resolvedWeaponWorld = resolvedWeaponWorld;
        _debugSnapshot.translationCorrectionGameUnits = result.translationCorrectionGameUnits;
        _debugSnapshot.rotationCorrectionDegrees = result.rotationCorrectionDegrees;

        const bool correctionWithinSafetyEnvelope =
            std::isfinite(result.translationCorrectionGameUnits) &&
            std::isfinite(result.rotationCorrectionDegrees) &&
            result.translationCorrectionGameUnits <= g_rockConfig.rockWeaponCollisionDynamicMaxVisualCorrectionGameUnits &&
            result.rotationCorrectionDegrees <= kMaxVisualCorrectionRotationDegrees;
        if (!correctionWithinSafetyEnvelope) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "Dynamic weapon visual correction rejected: body={} translation={:.2f}gu rotation={:.2f}deg",
                snapshot.bodyId,
                result.translationCorrectionGameUnits,
                result.rotationCorrectionDegrees);
            logPipelineStage("safety-gate");
            return result;
        }

        const bool correctionVisible =
            result.translationCorrectionGameUnits >= g_rockConfig.rockWeaponCollisionDynamicRenderMinTranslationGameUnits ||
            result.rotationCorrectionDegrees >= g_rockConfig.rockWeaponCollisionDynamicRenderMinRotationDegrees;
        if (correctionVisible) {
            result.applyVisualCorrection = true;
            result.resolvedWeaponWorld = resolvedWeaponWorld;
            _debugSnapshot.visualCorrectionActive = true;
        }
        logPipelineStage(correctionVisible ? "publish-requested" : "visibility-gate");
        return result;
    }

    bool DynamicWeaponCollisionRuntime::ensureProxyBody(
        const PhysicsFrameContext& frame,
        const WeaponCollision& weaponCollision,
        const RE::NiTransform& requestedWeaponWorld)
    {
        WeaponCollision::ApproximateBoundsSnapshot bounds{};
        if (!weaponCollision.getApproximateBoundsSnapshot(bounds) || !bounds.valid ||
            bounds.generationKey != _frameGenerationKey ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeaponWorld)) {
            return false;
        }
        const auto geometry = dynamic_weapon_collision_policy::makeBoxGeometry(bounds.minWeaponLocal, bounds.maxWeaponLocal);
        if (!geometry.valid) {
            return false;
        }

        const float scale = std::abs(requestedWeaponWorld.scale);
        const float padding = g_rockConfig.rockWeaponCollisionDynamicBoxPaddingGameUnits;
        const bool bodyMatches =
            _created &&
            _body.isValid() &&
            _authorityProxy.isValid() &&
            _authorityConstraint.isValid() &&
            _createdWorld == frame.hknpWorld &&
            _createdBhkWorld == frame.bhkWorld &&
            _createdGenerationKey == bounds.generationKey &&
            std::abs(_createdWeaponScale - scale) <= 0.0001f &&
            std::abs(_createdPaddingGameUnits - padding) <= 0.0001f &&
            !_rebuildRequestedAtomic.load(std::memory_order_acquire);
        if (bodyMatches) {
            return true;
        }

        const auto weaponIdentity = weaponCollision.getEquippedWeaponClassification();
        if (!weaponIdentity.hasEquippedWeapon) {
            return false;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_created) {
            retireProxyLocked(frame.bhkWorld);
        }

        const auto corners = dynamic_weapon_collision_policy::makeBoxCornerPointsHavok(
            geometry,
            scale,
            padding,
            physics_scale::gameToHavok());
        std::vector<RE::NiPoint3> pointCloud(corners.begin(), corners.end());
        auto* shape = havok_convex_shape_builder::buildConvexShapeFromLocalHavokPoints(
            pointCloud,
            (std::max)(0.0f, g_rockConfig.rockWeaponCollisionConvexRadius));
        if (!shape) {
            return false;
        }

        RE::NiTransform initialContactTarget = dynamic_weapon_collision_policy::makeProxyBodyTarget(
            requestedWeaponWorld,
            geometry.centerWeaponLocal);
        initialContactTarget.scale = 1.0f;
        const RE::NiTransform initialAuthorityTarget =
            dynamic_weapon_collision_policy::makeGripAuthorityTarget(requestedWeaponWorld);
        const auto generatedMaterial = havok_material_registry::registerGeneratedBodyMaterial(frame.hknpWorld);
        if (!_body.create(
                frame.hknpWorld,
                frame.bhkWorld,
                shape,
                dynamicWeaponProxyFilterInfo(),
                generatedMaterial,
                BethesdaMotionType::Dynamic,
                "ROCK_DynamicWeaponBox")) {
            havok_ref_count::release(shape);
            return false;
        }

        const auto proxyBodyId = _body.getBodyId();
        _bodyIdAtomic.store(proxyBodyId.value, std::memory_order_release);
        const bool processedManifoldFlagEnabled = havok_runtime::enableBodyFlags(
            frame.hknpWorld,
            proxyBodyId.value,
            kRaiseManifoldProcessedEvents,
            kRebuildBodyCollisionState);
        const auto flaggedBody = havok_runtime::snapshotBody(frame.hknpWorld, proxyBodyId);
        const bool processedManifoldFlagPublished =
            processedManifoldFlagEnabled &&
            flaggedBody.valid &&
            flaggedBody.body &&
            (flaggedBody.body->flags & kRaiseManifoldProcessedEvents) == kRaiseManifoldProcessedEvents;
        if (!processedManifoldFlagPublished) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon box {} failed processed-manifold event opt-in: enabled={} readable={} flags=0x{:08X}",
                proxyBodyId.value,
                processedManifoldFlagEnabled,
                flaggedBody.valid && flaggedBody.body,
                flaggedBody.body ? flaggedBody.body->flags : 0u);
            _bodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
            _body.retireDeferred(frame.bhkWorld);
            havok_ref_count::release(shape);
            return false;
        }

        _shape = shape;
        _createdWorld = frame.hknpWorld;
        _createdBhkWorld = frame.bhkWorld;
        _createdGenerationKey = bounds.generationKey;
        _createdCenterWeaponLocal = geometry.centerWeaponLocal;
        _createdHalfExtentsWeaponLocal = geometry.halfExtentsWeaponLocal;
        _createdWeaponScale = scale;
        _createdPaddingGameUnits = padding;
        _created = true;
        _rebuildRequestedAtomic.store(false, std::memory_order_release);
        const float bodyMass = dynamic_weapon_collision_policy::sanitizeWeaponMass(weaponIdentity.weightGame);
        _body.setMass(bodyMass);
        if (!applyWeaponBoxMassProperties(
                frame.hknpWorld,
                _body.getBodyId(),
                geometry,
                scale,
                padding,
                bodyMass)) {
            retireProxyLocked(frame.bhkWorld);
            return false;
        }
        if (!placeGeneratedKeyframedBodyImmediately(_body, initialContactTarget)) {
            retireProxyLocked(frame.bhkWorld);
            return false;
        }

        auto* authorityShape = grab_authority_proxy::buildProxyShape();
        if (!authorityShape) {
            ROCK_LOG_ERROR(Weapon, "Dynamic weapon grip authority creation failed: anchor shape unavailable contactBody={}", proxyBodyId.value);
            retireProxyLocked(frame.bhkWorld);
            return false;
        }
        const bool authorityCreated = _authorityProxy.create(
            frame.hknpWorld,
            frame.bhkWorld,
            authorityShape,
            grab_authority_proxy::noContactFilterInfo(),
            generatedMaterial,
            BethesdaMotionType::Keyframed,
            "ROCK_WeaponGripAuthorityProxy");
        havok_ref_count::release(authorityShape);
        if (!authorityCreated) {
            ROCK_LOG_ERROR(Weapon, "Dynamic weapon grip authority creation failed: anchor body unavailable contactBody={}", proxyBodyId.value);
            retireProxyLocked(frame.bhkWorld);
            return false;
        }
        if (!placeGeneratedKeyframedBodyImmediately(_authorityProxy, initialAuthorityTarget)) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon grip authority creation failed: initial anchor placement failed contactBody={} authorityBody={}",
                proxyBodyId.value,
                _authorityProxy.getBodyId().value);
            retireProxyLocked(frame.bhkWorld);
            return false;
        }
        initializeGeneratedKeyframedBodyDriveState(_authorityDriveState, initialAuthorityTarget);

        std::uint32_t authorityFilterInfo = 0;
        const bool authorityFilterReadable = havok_runtime::tryReadFilterInfo(
            frame.hknpWorld,
            _authorityProxy.getBodyId(),
            authorityFilterInfo);
        if (!authorityFilterReadable || !grab_authority_proxy::hasNoContactFilterInfo(authorityFilterInfo)) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon grip authority creation failed: no-contact policy invalid read={} filter=0x{:08X} authorityBody={}",
                authorityFilterReadable,
                authorityFilterInfo,
                _authorityProxy.getBodyId().value);
            retireProxyLocked(frame.bhkWorld);
            return false;
        }

        // Both generated bodies are authored with the same physical rotation.
        // Their constraint relation therefore contains only the weapon-local
        // grip-to-box-center offset. Passing these two generated-column frames
        // through the visual-object/proxy adapter invents a world-dependent
        // relative rotation and twists the box sideways when the motor engages.
        const RE::NiTransform desiredBodyTransformAuthoritySpace =
            dynamic_weapon_collision_policy::makeContactBodyInGripAuthoritySpace(
                geometry.centerWeaponLocal,
                scale);
        const auto motorTuning = buildWeaponGripConstraintTuning(bodyMass);
        _authorityConstraint = createGrabConstraint(
            frame.hknpWorld,
            _authorityProxy.getBodyId(),
            _body.getBodyId(),
            initialAuthorityTarget,
            requestedWeaponWorld.translate,
            desiredBodyTransformAuthoritySpace,
            motorTuning);
        if (!_authorityConstraint.isValid()) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon grip authority creation failed: constraint unavailable contactBody={} authorityBody={}",
                proxyBodyId.value,
                _authorityProxy.getBodyId().value);
            retireProxyLocked(frame.bhkWorld);
            return false;
        }

        ROCK_LOG_INFO(
            Weapon,
            "Dynamic weapon grip-constrained box created: contactBody={} authorityBody={} constraint={} generation={:016X} authority=grip center=({:.2f},{:.2f},{:.2f}) half=({:.2f},{:.2f},{:.2f}) scale={:.3f} mass={:.2f} forces=({:.1f},{:.1f}) padding={:.2f} layer={}",
            _body.getBodyId().value,
            _authorityProxy.getBodyId().value,
            _authorityConstraint.constraintId,
            _createdGenerationKey,
            _createdCenterWeaponLocal.x,
            _createdCenterWeaponLocal.y,
            _createdCenterWeaponLocal.z,
            _createdHalfExtentsWeaponLocal.x,
            _createdHalfExtentsWeaponLocal.y,
            _createdHalfExtentsWeaponLocal.z,
            _createdWeaponScale,
            bodyMass,
            motorTuning.linearMaxForce,
            motorTuning.angularMaxForce,
            _createdPaddingGameUnits,
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        return true;
    }

    void DynamicWeaponCollisionRuntime::flushPendingPhysicsDrive(
        RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world ||
            !_body.isValid() || !_authorityProxy.isValid() || !_authorityConstraint.isValid()) {
            return;
        }

        const auto driveResult = driveGeneratedKeyframedBody(
            world,
            _authorityProxy,
            _authorityDriveState,
            timing,
            "DynamicWeaponGripAuthority",
            0,
            g_rockConfig.rockWeaponCollisionDynamicMaxLinearVelocityHavok,
            g_rockConfig.rockWeaponCollisionDynamicMaxAngularVelocityRadians);
        if (driveResult.shouldRequestRebuild()) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            return;
        }

        _droveThisSubstep =
            driveResult.driven &&
            driveResult.hasRequestedTargetGameTransform;
        _physicsRequestedTargetValid = driveResult.hasRequestedTargetGameTransform;
        if (_physicsRequestedTargetValid) {
            _physicsRequestedTarget = dynamic_weapon_collision_policy::makeContactBodyTargetFromGripAuthority(
                driveResult.requestedTargetGameTransform,
                _createdCenterWeaponLocal,
                _createdWeaponScale);
        }
        _physicsDriveTeleported = driveResult.teleported;
        if (driveResult.teleported) {
            _contactGraceSolves = 0;
        }
    }

    void DynamicWeaponCollisionRuntime::samplePostSolve(RE::hknpWorld* world, const std::uint64_t solveSequence)
    {
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world ||
            !_body.isValid() || !_droveThisSubstep || !_physicsRequestedTargetValid) {
            return;
        }
        _droveThisSubstep = false;

        RE::NiTransform liveBodyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, _body.getBodyId(), liveBodyWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(liveBodyWorld)) {
            _contactGraceSolves = 0;
            clearPublishedPhysicsSnapshot();
            return;
        }
        const auto contactSequence = _contactSequenceAtomic.load(std::memory_order_acquire);
        bool newMatchingContact = false;
        std::uint32_t otherBodyId = kInvalidBodyId;
        std::uint32_t otherLayer = 0;
        if (contactSequence != _consumedContactSequence) {
            _consumedContactSequence = contactSequence;
            const auto contactWorld = _contactWorldAtomic.load(std::memory_order_relaxed);
            const auto contactProxy = _contactProxyBodyIdAtomic.load(std::memory_order_relaxed);
            otherBodyId = _contactOtherBodyIdAtomic.load(std::memory_order_relaxed);
            otherLayer = _contactOtherLayerAtomic.load(std::memory_order_relaxed);
            newMatchingContact =
                contactWorld == reinterpret_cast<std::uintptr_t>(world) &&
                contactProxy == _body.getBodyId().value &&
                collision_layer_policy::isWorldSurfaceLayer(otherLayer);
        }
        if (_physicsDriveTeleported) {
            _contactGraceSolves = 0;
        } else if (newMatchingContact) {
            _contactGraceSolves = kContactGraceSolves;
        } else if (_contactGraceSolves > 0) {
            --_contactGraceSolves;
        }

        PhysicsSnapshot snapshot{};
        snapshot.valid = true;
        snapshot.contactActive = _contactGraceSolves > 0;
        snapshot.teleported = _physicsDriveTeleported;
        snapshot.world = reinterpret_cast<std::uintptr_t>(world);
        snapshot.bodyId = _body.getBodyId().value;
        snapshot.otherBodyId = newMatchingContact ? otherBodyId : _snapshotOtherBodyIdAtomic.load(std::memory_order_relaxed);
        snapshot.otherLayer = newMatchingContact ? otherLayer : _snapshotOtherLayerAtomic.load(std::memory_order_relaxed);
        snapshot.contactGraceSolves = _contactGraceSolves;
        snapshot.generationKey = _createdGenerationKey;
        snapshot.solveSequence = solveSequence;
        snapshot.weaponScale = _createdWeaponScale;
        snapshot.requestedProxyBodyWorld = _physicsRequestedTarget;
        snapshot.liveProxyBodyWorld = liveBodyWorld;
        publishPhysicsSnapshot(snapshot);
        _physicsDriveTeleported = false;
    }

    bool DynamicWeaponCollisionRuntime::isProxyBodyIdAtomic(const std::uint32_t bodyId) const
    {
        return bodyId != kInvalidBodyId && _bodyIdAtomic.load(std::memory_order_acquire) == bodyId;
    }

    void DynamicWeaponCollisionRuntime::recordWorldSurfaceContactCallback(
        RE::hknpWorld* world,
        const std::uint32_t proxyBodyId,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer,
        const bool rawContactPointValid)
    {
        if (!world || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isWorldSurfaceLayer(otherLayer)) {
            return;
        }
        _worldSurfaceCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!rawContactPointValid) {
            return;
        }
        _rawPointCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _contactWorldAtomic.store(reinterpret_cast<std::uintptr_t>(world), std::memory_order_relaxed);
        _contactProxyBodyIdAtomic.store(proxyBodyId, std::memory_order_relaxed);
        _contactOtherBodyIdAtomic.store(otherBodyId, std::memory_order_relaxed);
        _contactOtherLayerAtomic.store(otherLayer, std::memory_order_relaxed);
        _contactSequenceAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::recordWorldSurfaceManifoldProcessedCallback(
        RE::hknpWorld* world,
        const std::uint32_t proxyBodyId,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer)
    {
        if (!world || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isWorldSurfaceLayer(otherLayer)) {
            return;
        }
        _worldSurfaceCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _contactWorldAtomic.store(reinterpret_cast<std::uintptr_t>(world), std::memory_order_relaxed);
        _contactProxyBodyIdAtomic.store(proxyBodyId, std::memory_order_relaxed);
        _contactOtherBodyIdAtomic.store(otherBodyId, std::memory_order_relaxed);
        _contactOtherLayerAtomic.store(otherLayer, std::memory_order_relaxed);
        _contactSequenceAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::retireProxyLocked(void* bhkWorld)
    {
        const auto bodyId = _bodyIdAtomic.exchange(kInvalidBodyId, std::memory_order_acq_rel);
        const auto authorityBodyId = _authorityProxy.isValid() ? _authorityProxy.getBodyId().value : kInvalidBodyId;
        const auto constraintId = _authorityConstraint.constraintId;
        const bool liveOwnerMatches =
            _created &&
            bhkWorld &&
            bhkWorld == _createdBhkWorld;
        if (liveOwnerMatches) {
            destroyGrabConstraint(_createdWorld, _authorityConstraint);
            if (_body.isValid()) {
                _body.retireDeferred(bhkWorld);
            }
            if (_authorityProxy.isValid()) {
                _authorityProxy.retireDeferred(bhkWorld);
            }
        } else {
            destroyGrabConstraint(nullptr, _authorityConstraint);
            if (_created && (_body.isValid() || _authorityProxy.isValid())) {
                ROCK_LOG_WARN(
                    Weapon,
                    "Dynamic weapon authority owner changed before retirement; abandoning native state without mutating an unverified world: contactBody={} authorityBody={} constraint={}",
                    bodyId,
                    authorityBodyId,
                    constraintId);
            }
            _body.reset();
            _authorityProxy.reset();
        }
        clearLocalProxyStateLocked();
        if (bodyId != kInvalidBodyId && liveOwnerMatches) {
            ROCK_LOG_INFO(
                Weapon,
                "Dynamic weapon grip-constrained box retired: contactBody={} authorityBody={} constraint={}",
                bodyId,
                authorityBodyId,
                constraintId);
        }
    }

    void DynamicWeaponCollisionRuntime::clearLocalProxyStateLocked()
    {
        if (_shape) {
            havok_ref_count::release(_shape);
        }
        _shape = nullptr;
        _createdWorld = nullptr;
        _createdBhkWorld = nullptr;
        _createdGenerationKey = 0;
        _createdCenterWeaponLocal = {};
        _createdHalfExtentsWeaponLocal = {};
        _createdWeaponScale = 1.0f;
        _createdPaddingGameUnits = 0.0f;
        _created = false;
        _droveThisSubstep = false;
        _physicsRequestedTargetValid = false;
        _physicsDriveTeleported = false;
        _physicsRequestedTarget = {};
        _contactGraceSolves = 0;
        _consumedContactSequence = 0;
        _proxyPairCallbackSequenceAtomic.store(0, std::memory_order_release);
        _worldSurfaceCallbackSequenceAtomic.store(0, std::memory_order_release);
        _rawPointCallbackSequenceAtomic.store(0, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.store(0, std::memory_order_release);
        _contactSequenceAtomic.store(0, std::memory_order_release);
        _contactWorldAtomic.store(0, std::memory_order_release);
        _contactProxyBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _contactOtherBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _contactOtherLayerAtomic.store(0, std::memory_order_release);
        _rebuildRequestedAtomic.store(false, std::memory_order_release);
        clearGeneratedKeyframedBodyDriveState(_authorityDriveState);
        clearPublishedPhysicsSnapshot();
    }

    void DynamicWeaponCollisionRuntime::retireAll(void* bhkWorld)
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        retireProxyLocked(bhkWorld);
        _enabledAtomic.store(false, std::memory_order_release);
        _frameAcceptingIntent = false;
        _frameHasIntent = false;
        _debugSnapshot = {};
    }

    void DynamicWeaponCollisionRuntime::abandonHavokStateAfterWorldLoss()
    {
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        const auto bodyId = _bodyIdAtomic.exchange(kInvalidBodyId, std::memory_order_acq_rel);
        const auto authorityBodyId = _authorityProxy.isValid() ? _authorityProxy.getBodyId().value : kInvalidBodyId;
        const auto constraintId = _authorityConstraint.constraintId;
        destroyGrabConstraint(nullptr, _authorityConstraint);
        _body.reset();
        _authorityProxy.reset();
        clearLocalProxyStateLocked();
        _enabledAtomic.store(false, std::memory_order_release);
        _frameAcceptingIntent = false;
        _frameHasIntent = false;
        _frameIndex = 0;
        _frameGenerationKey = 0;
        _frameWorld = nullptr;
        _frameBhkWorld = nullptr;
        _frameWeaponNode = nullptr;
        _frameRequestedWeaponWorld = {};
        _debugSnapshot = {};
        if (bodyId != kInvalidBodyId) {
            ROCK_LOG_WARN(
                Weapon,
                "Dynamic weapon grip-constrained box abandoned after Havok world loss: contactBody={} authorityBody={} constraint={}",
                bodyId,
                authorityBodyId,
                constraintId);
        }
    }

    RE::hknpBodyId DynamicWeaponCollisionRuntime::proxyBodyIdForDebug() const
    {
        return RE::hknpBodyId{ _bodyIdAtomic.load(std::memory_order_acquire) };
    }

    bool DynamicWeaponCollisionRuntime::getDebugSnapshot(DebugSnapshot& outSnapshot) const
    {
        outSnapshot = _debugSnapshot;
        return outSnapshot.valid;
    }

    void DynamicWeaponCollisionRuntime::storeAtomicTransform(AtomicTransform& target, const RE::NiTransform& value)
    {
        std::size_t index = 0;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                target.rotation[index++].store(value.rotate.entry[row][column], std::memory_order_relaxed);
            }
        }
        target.translation[0].store(value.translate.x, std::memory_order_relaxed);
        target.translation[1].store(value.translate.y, std::memory_order_relaxed);
        target.translation[2].store(value.translate.z, std::memory_order_relaxed);
        target.scale.store(value.scale, std::memory_order_relaxed);
    }

    RE::NiTransform DynamicWeaponCollisionRuntime::loadAtomicTransform(const AtomicTransform& source)
    {
        RE::NiTransform result{};
        std::size_t index = 0;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                result.rotate.entry[row][column] = source.rotation[index++].load(std::memory_order_relaxed);
            }
        }
        result.translate = RE::NiPoint3{
            source.translation[0].load(std::memory_order_relaxed),
            source.translation[1].load(std::memory_order_relaxed),
            source.translation[2].load(std::memory_order_relaxed),
        };
        result.scale = source.scale.load(std::memory_order_relaxed);
        return result;
    }

    void DynamicWeaponCollisionRuntime::publishPhysicsSnapshot(const PhysicsSnapshot& snapshot)
    {
        _snapshotVersionAtomic.fetch_add(1, std::memory_order_acq_rel);
        _snapshotValidAtomic.store(snapshot.valid, std::memory_order_relaxed);
        _snapshotContactActiveAtomic.store(snapshot.contactActive, std::memory_order_relaxed);
        _snapshotTeleportedAtomic.store(snapshot.teleported, std::memory_order_relaxed);
        _snapshotWorldAtomic.store(snapshot.world, std::memory_order_relaxed);
        _snapshotBodyIdAtomic.store(snapshot.bodyId, std::memory_order_relaxed);
        _snapshotOtherBodyIdAtomic.store(snapshot.otherBodyId, std::memory_order_relaxed);
        _snapshotOtherLayerAtomic.store(snapshot.otherLayer, std::memory_order_relaxed);
        _snapshotContactGraceAtomic.store(snapshot.contactGraceSolves, std::memory_order_relaxed);
        _snapshotGenerationKeyAtomic.store(snapshot.generationKey, std::memory_order_relaxed);
        _snapshotSolveSequenceAtomic.store(snapshot.solveSequence, std::memory_order_relaxed);
        _snapshotWeaponScaleAtomic.store(snapshot.weaponScale, std::memory_order_relaxed);
        storeAtomicTransform(_snapshotRequestedProxyBodyWorld, snapshot.requestedProxyBodyWorld);
        storeAtomicTransform(_snapshotLiveProxyBodyWorld, snapshot.liveProxyBodyWorld);
        _snapshotVersionAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::clearPublishedPhysicsSnapshot()
    {
        PhysicsSnapshot snapshot{};
        publishPhysicsSnapshot(snapshot);
    }

    bool DynamicWeaponCollisionRuntime::readPhysicsSnapshot(PhysicsSnapshot& outSnapshot) const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto before = _snapshotVersionAtomic.load(std::memory_order_acquire);
            if ((before & 1u) != 0) {
                continue;
            }
            PhysicsSnapshot candidate{};
            candidate.valid = _snapshotValidAtomic.load(std::memory_order_relaxed);
            candidate.contactActive = _snapshotContactActiveAtomic.load(std::memory_order_relaxed);
            candidate.teleported = _snapshotTeleportedAtomic.load(std::memory_order_relaxed);
            candidate.world = _snapshotWorldAtomic.load(std::memory_order_relaxed);
            candidate.bodyId = _snapshotBodyIdAtomic.load(std::memory_order_relaxed);
            candidate.otherBodyId = _snapshotOtherBodyIdAtomic.load(std::memory_order_relaxed);
            candidate.otherLayer = _snapshotOtherLayerAtomic.load(std::memory_order_relaxed);
            candidate.contactGraceSolves = _snapshotContactGraceAtomic.load(std::memory_order_relaxed);
            candidate.generationKey = _snapshotGenerationKeyAtomic.load(std::memory_order_relaxed);
            candidate.solveSequence = _snapshotSolveSequenceAtomic.load(std::memory_order_relaxed);
            candidate.weaponScale = _snapshotWeaponScaleAtomic.load(std::memory_order_relaxed);
            candidate.requestedProxyBodyWorld = loadAtomicTransform(_snapshotRequestedProxyBodyWorld);
            candidate.liveProxyBodyWorld = loadAtomicTransform(_snapshotLiveProxyBodyWorld);
            const auto after = _snapshotVersionAtomic.load(std::memory_order_acquire);
            if (before == after && (after & 1u) == 0) {
                outSnapshot = candidate;
                return candidate.valid;
            }
        }
        outSnapshot = {};
        return false;
    }
}

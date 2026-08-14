#include "physics-interaction/weapon/DynamicWeaponCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
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

        const char* compoundSnapshotFailureName(const WeaponCollision::CompoundGeometrySnapshotFailure failure)
        {
            using Failure = WeaponCollision::CompoundGeometrySnapshotFailure;
            switch (failure) {
            case Failure::None:
                return "none";
            case Failure::NoGeneration:
                return "no-generation";
            case Failure::NoActiveBodies:
                return "no-active-bodies";
            case Failure::MissingShape:
                return "missing-shape";
            case Failure::MissingPointCloud:
                return "missing-point-cloud";
            case Failure::NonFinitePoint:
                return "nonfinite-point";
            case Failure::DegeneratePointCloud:
                return "degenerate-point-cloud";
            case Failure::SourceTransformUnavailable:
                return "source-transform-unavailable";
            case Failure::BodyCountChanged:
                return "body-count-changed";
            case Failure::GenerationChanged:
                return "generation-changed";
            case Failure::InvalidBounds:
                return "invalid-bounds";
            }
            return "unknown";
        }

        bool makeCompoundChildTransform(
            const RE::NiTransform& shapeInWeapon,
            const RE::NiPoint3& aggregateCenterWeaponLocal,
            const float weaponScale,
            havok_compound_shape_builder::ChildTransform& outTransform)
        {
            outTransform = {};
            if (!dynamic_weapon_collision_policy::isFiniteTransform(shapeInWeapon) ||
                std::abs(shapeInWeapon.scale - 1.0f) > 0.0001f) {
                return false;
            }

            const auto childFrame = dynamic_weapon_collision_policy::makeCompoundChildFrame(
                shapeInWeapon.translate,
                aggregateCenterWeaponLocal,
                weaponScale,
                physics_scale::gameToHavok());
            if (!childFrame.valid) {
                return false;
            }

            // NiTransform stores each local basis axis as a row. hkTransformf
            // stores those same axes as columns, so each Ni row maps directly
            // to one native child-transform column.
            outTransform.column0 = {
                shapeInWeapon.rotate.entry[0][0],
                shapeInWeapon.rotate.entry[0][1],
                shapeInWeapon.rotate.entry[0][2],
                0.0f,
            };
            outTransform.column1 = {
                shapeInWeapon.rotate.entry[1][0],
                shapeInWeapon.rotate.entry[1][1],
                shapeInWeapon.rotate.entry[1][2],
                0.0f,
            };
            outTransform.column2 = {
                shapeInWeapon.rotate.entry[2][0],
                shapeInWeapon.rotate.entry[2][1],
                shapeInWeapon.rotate.entry[2][2],
                0.0f,
            };
            outTransform.translation = {
                childFrame.translationHavok.x,
                childFrame.translationHavok.y,
                childFrame.translationHavok.z,
                1.0f,
            };
            return true;
        }

        std::uint32_t dynamicWeaponProxyFilterInfo()
        {
            return (kDynamicWeaponCollisionGroup << 16) |
                   (collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY & collision_layer_policy::FO4_LAYER_FILTER_MASK);
        }

        float scaleFiniteValue(const float value, const float multiplier)
        {
            return (std::isfinite(value) ? value : 0.0f) * (std::isfinite(multiplier) ? multiplier : 1.0f);
        }

        float signedTranslationStepTowardContactError(
            const RE::NiTransform& previousRequested,
            const RE::NiTransform& currentRequested,
            const RE::NiTransform& liveBody)
        {
            const RE::NiPoint3 contactError{
                currentRequested.translate.x - liveBody.translate.x,
                currentRequested.translate.y - liveBody.translate.y,
                currentRequested.translate.z - liveBody.translate.z,
            };
            const float contactErrorLength = std::sqrt(
                contactError.x * contactError.x +
                contactError.y * contactError.y +
                contactError.z * contactError.z);
            if (!std::isfinite(contactErrorLength) || contactErrorLength <= 0.0001f) {
                return 0.0f;
            }

            const RE::NiPoint3 requestedStep{
                currentRequested.translate.x - previousRequested.translate.x,
                currentRequested.translate.y - previousRequested.translate.y,
                currentRequested.translate.z - previousRequested.translate.z,
            };
            return
                (requestedStep.x * contactError.x +
                    requestedStep.y * contactError.y +
                    requestedStep.z * contactError.z) /
                contactErrorLength;
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

        void updateWeaponGripConstraintContactTau(
            ActiveConstraint& constraint,
            const bool contactActive,
            const float deltaTime)
        {
            if (!constraint.linearMotor || !constraint.angularMotor) {
                return;
            }

            const float baseLinearTau = grab_motion_controller::safePositive(
                scaleFiniteValue(
                    g_rockConfig.rockGrabLinearTau,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier),
                0.03f);
            const float baseAngularTau = grab_motion_controller::safePositive(
                scaleFiniteValue(
                    g_rockConfig.rockGrabAngularTau,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier),
                baseLinearTau);
            const float collisionTau = grab_motion_controller::safePositive(
                scaleFiniteValue(
                    g_rockConfig.rockGrabTauMin,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier),
                baseLinearTau);
            const float linearTarget = contactActive ? collisionTau : baseLinearTau;
            const float angularTarget = contactActive ? collisionTau : baseAngularTau;

            constraint.linearMotor->tau = grab_motion_controller::advanceToward(
                constraint.linearMotor->tau,
                linearTarget,
                g_rockConfig.rockGrabTauLerpSpeed,
                deltaTime);
            constraint.angularMotor->tau = grab_motion_controller::advanceToward(
                constraint.angularMotor->tau,
                angularTarget,
                g_rockConfig.rockGrabTauLerpSpeed,
                deltaTime);
            constraint.currentTau = constraint.linearMotor->tau;
        }

        bool applyWeaponEnvelopeMassProperties(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            const dynamic_weapon_collision_policy::BoundingBoxGeometry& geometry,
            const float weaponScale,
            const float paddingGameUnits,
            const float bodyMass)
        {
            const auto envelopeMassProperties = dynamic_weapon_collision_policy::makeBoundingBoxMassProperties(
                geometry,
                weaponScale,
                paddingGameUnits,
                physics_scale::gameToHavok(),
                bodyMass);
            if (!envelopeMassProperties.valid) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon compound envelope mass properties invalid: body={} scale={:.3f} padding={:.3f} mass={:.3f}",
                    bodyId.value,
                    weaponScale,
                    paddingGameUnits,
                    bodyMass);
                return false;
            }

            const float inverseInertiaMultiplier =
                g_rockConfig.rockWeaponCollisionDynamicInverseInertiaMultiplier;
            if (!std::isfinite(inverseInertiaMultiplier) || inverseInertiaMultiplier <= 0.0f) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon inverse-inertia multiplier invalid: body={} multiplier={}",
                    bodyId.value,
                    inverseInertiaMultiplier);
                return false;
            }

            const auto normalizedInertia = grab_inertia_policy::normalizeInverseInertiaAxesForGrab(
                envelopeMassProperties.inverseInertia.x * inverseInertiaMultiplier,
                envelopeMassProperties.inverseInertia.y * inverseInertiaMultiplier,
                envelopeMassProperties.inverseInertia.z * inverseInertiaMultiplier,
                g_rockConfig.rockGrabMaxInertiaRatio,
                g_rockConfig.rockGrabMinInertia);
            if (!normalizedInertia.valid) {
                ROCK_LOG_ERROR(Weapon, "Dynamic weapon compound envelope inertia normalization failed: body={}", bodyId.value);
                return false;
            }

            const auto initialMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!initialMotion.valid || !initialMotion.motion) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon compound motion unavailable for mass properties: body={} readable={} motion={}",
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
                    "Dynamic weapon compound packed mass properties invalid: body={} inertia=[{},{},{}] inverseMass={}",
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
                    "Dynamic weapon compound mass-properties rebuild failed: body={} motion={}",
                    bodyId.value,
                    initialMotion.motionIndex);
                return false;
            }

            const auto rebuiltMotion = havok_runtime::snapshotBody(world, bodyId);
            if (!rebuiltMotion.valid || !rebuiltMotion.motion || rebuiltMotion.motionIndex != initialMotion.motionIndex) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon compound motion changed during mass-properties rebuild: body={} before={} after={} readable={}",
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
                "Dynamic weapon compound envelope mass properties: body={} motion={} halfHavok=({:.4f},{:.4f},{:.4f}) physicalInverseInertia=({:.6f},{:.6f},{:.6f}) multiplier={:.3f} appliedInverseInertia=({:.6f},{:.6f},{:.6f}) packed=[{},{},{}] inverseMass={:.6f} ratio={:.2f}->{:.2f}",
                bodyId.value,
                rebuiltMotion.motionIndex,
                envelopeMassProperties.halfExtentsHavok.x,
                envelopeMassProperties.halfExtentsHavok.y,
                envelopeMassProperties.halfExtentsHavok.z,
                envelopeMassProperties.inverseInertia.x,
                envelopeMassProperties.inverseInertia.y,
                envelopeMassProperties.inverseInertia.z,
                inverseInertiaMultiplier,
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

        if (!queueCompoundChildTransforms(
                weaponCollision,
                weaponNode,
                std::abs(_frameRequestedWeaponWorld.scale))) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "Dynamic weapon live compound pose rejected: generation={:016X} children={}",
                _createdGenerationKey,
                _createdCompoundChildCount);
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
        _debugSnapshot.obstacleCallbackSequence = _obstacleCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.rawPointCallbackSequence = _rawPointCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.processedManifoldCallbackSequence = _processedManifoldCallbackSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.admittedContactSequence = _contactSequenceAtomic.load(std::memory_order_acquire);
        _debugSnapshot.compoundChildCount = _createdCompoundChildCount;
        _debugSnapshot.compoundPointCount = _createdCompoundPointCount;
        _debugSnapshot.centerWeaponLocal = _createdCenterWeaponLocal;
        _debugSnapshot.halfExtentsWeaponLocal = _createdHalfExtentsWeaponLocal;
        _debugSnapshot.requestedWeaponWorld = _frameRequestedWeaponWorld;

        ContactDiagnosticSnapshot contactDiagnostic{};
        const bool contactDiagnosticCurrent =
            g_rockConfig.rockDebugDrawDynamicWeaponColliders &&
            readContactDiagnosticSnapshot(contactDiagnostic) &&
            contactDiagnostic.valid &&
            contactDiagnostic.world == reinterpret_cast<std::uintptr_t>(frame.hknpWorld) &&
            contactDiagnostic.bodyId == _body.getBodyId().value &&
            contactDiagnostic.generationKey == _createdGenerationKey;
        if (contactDiagnosticCurrent &&
            contactDiagnostic.contactEpisode > _reportedContactEpisode) {
            _reportedContactEpisode = contactDiagnostic.contactEpisode;
            result.contactEpisodeStarted = true;
            result.rawContactPointValid = contactDiagnostic.rawContactPointValid;
            result.rawContactProxyWasBodyA = contactDiagnostic.rawContactProxyWasBodyA;
            result.otherBodyWorldValid = contactDiagnostic.otherBodyWorldValid;
            result.otherBodyId = contactDiagnostic.otherBodyId;
            result.otherLayer = contactDiagnostic.otherLayer;
            result.otherMotionIndex = contactDiagnostic.otherMotionIndex;
            result.rawContactPointCount = contactDiagnostic.rawContactPointCount;
            result.rawContactPointIndex = contactDiagnostic.rawContactPointIndex;
            result.contactEpisode = contactDiagnostic.contactEpisode;
            result.contactSolveAge = contactDiagnostic.contactSolveAge;
            result.otherCollisionObject = contactDiagnostic.otherCollisionObject;
            result.otherOwnerNode = contactDiagnostic.otherOwnerNode;
            result.rawContactPointWeightSum = contactDiagnostic.rawContactPointWeightSum;
            result.rawContactPointGame = contactDiagnostic.rawContactPointGame;
            result.rawContactNormalHavok = contactDiagnostic.rawContactNormalHavok;
            result.requestedContactBodyWorld = contactDiagnostic.requestedProxyBodyWorld;
            result.liveContactBodyWorld = contactDiagnostic.liveProxyBodyWorld;
            result.otherBodyWorld = contactDiagnostic.otherBodyWorld;
        }

        const auto logPipelineStage = [&](const char* stage) {
            if (!g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
                return;
            }
            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                500,
                "DWC pipeline: stage={} contactBody={} authorityBody={} constraint={} callbacks(pair/obstacle/raw/manifold/admit)={}/{}/{}/{}/{} snapshot(read/valid/identity/contact/teleport)={}/{}/{}/{}/{} retention={:.3f}s gripPivotError={:.2f}gu angularYield={:.2f}deg visual={}",
                stage,
                _debugSnapshot.bodyId,
                _debugSnapshot.authorityBodyId,
                _debugSnapshot.constraintId,
                _debugSnapshot.proxyPairCallbackSequence,
                _debugSnapshot.obstacleCallbackSequence,
                _debugSnapshot.rawPointCallbackSequence,
                _debugSnapshot.processedManifoldCallbackSequence,
                _debugSnapshot.admittedContactSequence,
                _debugSnapshot.physicsSnapshotReadable,
                _debugSnapshot.physicsSnapshotValid,
                _debugSnapshot.physicsSnapshotIdentityCurrent,
                _debugSnapshot.physicsSnapshotContactActive,
                _debugSnapshot.physicsSnapshotTeleported,
                _debugSnapshot.contactRetentionSeconds,
                result.translationCorrectionGameUnits,
                result.rotationCorrectionDegrees,
                result.publishVisualAuthority);
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
        _debugSnapshot.contactActive = snapshot.contactActive;
        _debugSnapshot.otherBodyId = snapshot.otherBodyId;
        _debugSnapshot.otherLayer = snapshot.otherLayer;
        _debugSnapshot.contactRetentionSeconds = snapshot.contactRetentionSeconds;
        _debugSnapshot.solveSequence = snapshot.solveSequence;
        _debugSnapshot.liveWeaponWorld = sampledLiveWeaponWorld;
        _debugSnapshot.resolvedWeaponWorld = resolvedWeaponWorld;
        _debugSnapshot.translationCorrectionGameUnits = result.translationCorrectionGameUnits;
        _debugSnapshot.rotationCorrectionDegrees = result.rotationCorrectionDegrees;

        const bool correctionFinite =
            std::isfinite(result.translationCorrectionGameUnits) &&
            std::isfinite(result.rotationCorrectionDegrees);
        if (!correctionFinite) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "Dynamic weapon visual correction rejected as non-finite: body={} translation={:.2f}gu rotation={:.2f}deg",
                snapshot.bodyId,
                result.translationCorrectionGameUnits,
                result.rotationCorrectionDegrees);
            logPipelineStage("finite-gate");
            return result;
        }

        const bool correctionVisible =
            (result.translationCorrectionGameUnits >= g_rockConfig.rockWeaponCollisionDynamicRenderMinTranslationGameUnits ||
                result.rotationCorrectionDegrees >= g_rockConfig.rockWeaponCollisionDynamicRenderMinRotationDegrees);
        /*
         * FRIK V2 consumes tagged hand transforms during its next skeleton
         * frame. Keep one persistent presentation owner for every current
         * post-solve sample, but expose the compliant body's displacement only
         * while a positive-point world/hand manifold is retained. In free
         * space the body naturally has a soft-motor residual; publishing that
         * residual makes locomotion look like weapon wobble. Publishing the
         * collision-free intent through the same tag avoids both that residual
         * and the former clear/re-add owner switch.
         */
        const bool publishResolvedContact =
            snapshot.contactActive && correctionVisible;
        result.publishVisualAuthority = true;
        result.resolvedWeaponWorld = publishResolvedContact ?
            resolvedWeaponWorld :
            _frameRequestedWeaponWorld;
        _debugSnapshot.visualCorrectionActive = publishResolvedContact;
        logPipelineStage(
            publishResolvedContact ?
                "publish-contact" :
                "publish-intent");
        return result;
    }

    bool DynamicWeaponCollisionRuntime::queueCompoundChildTransforms(
        const WeaponCollision& weaponCollision,
        const RE::NiAVObject* weaponNode,
        const float weaponScale)
    {
        if (!_created || !weaponNode || !static_cast<bool>(_compoundShape) ||
            !std::isfinite(weaponScale) || weaponScale <= 0.0001f) {
            return false;
        }

        std::scoped_lock poseLock(_compoundPoseMutex);
        if (_compoundPoseScratch.size() != _createdCompoundChildCount ||
            _pendingCompoundChildTransforms.size() != _createdCompoundChildCount) {
            return false;
        }

        std::size_t childCount = 0;
        if (!weaponCollision.getCompoundChildPoseSnapshot(
                weaponNode,
                _createdGenerationKey,
                _compoundPoseScratch,
                childCount) ||
            childCount != _createdCompoundChildCount) {
            return false;
        }

        for (std::size_t i = 0; i < childCount; ++i) {
            if (!makeCompoundChildTransform(
                    _compoundPoseScratch[i].shapeInWeapon,
                    _createdCenterWeaponLocal,
                    weaponScale,
                    _pendingCompoundChildTransforms[i])) {
                return false;
            }
        }
        ++_queuedCompoundPoseSequence;
        return true;
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
        const auto approximateGeometry = dynamic_weapon_collision_policy::makeBoundingBoxGeometry(bounds.minWeaponLocal, bounds.maxWeaponLocal);
        if (!approximateGeometry.valid) {
            return false;
        }

        const float scale = std::abs(requestedWeaponWorld.scale);
        // The serialized key retains its original Box name so the active test
        // configuration is not silently disabled. It now affects only the
        // already-qualified bounding-envelope inertia, never child geometry.
        const float inertiaEnvelopePadding = g_rockConfig.rockWeaponCollisionDynamicBoxPaddingGameUnits;
        const bool bodyMatches =
            _created &&
            _body.isValid() &&
            _authorityProxy.isValid() &&
            _authorityConstraint.isValid() &&
            static_cast<bool>(_compoundShape) &&
            _createdWorld == frame.hknpWorld &&
            _createdBhkWorld == frame.bhkWorld &&
            _createdGenerationKey == bounds.generationKey &&
            std::abs(_createdWeaponScale - scale) <= 0.0001f &&
            std::abs(_createdInertiaEnvelopePaddingGameUnits - inertiaEnvelopePadding) <= 0.0001f &&
            !_rebuildRequestedAtomic.load(std::memory_order_acquire);
        if (bodyMatches) {
            return true;
        }

        const auto weaponIdentity = weaponCollision.getEquippedWeaponClassification();
        if (!weaponIdentity.hasEquippedWeapon) {
            return false;
        }

        WeaponCollision::CompoundGeometrySnapshot compoundGeometry{};
        if (!weaponCollision.getCompoundGeometrySnapshot(compoundGeometry) ||
            !compoundGeometry.valid ||
            compoundGeometry.generationKey != bounds.generationKey ||
            compoundGeometry.generationKey != _frameGenerationKey) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "Dynamic weapon compound snapshot rejected: stage={} generation={:016X} expected={:016X} source={}/{} body={}",
                compoundSnapshotFailureName(compoundGeometry.failure),
                compoundGeometry.generationKey,
                _frameGenerationKey,
                compoundGeometry.failedSourceIndex,
                compoundGeometry.sourceBodyCount,
                compoundGeometry.failedBodyId);
            return false;
        }
        const auto geometry = dynamic_weapon_collision_policy::makeBoundingBoxGeometry(
            compoundGeometry.minWeaponLocal,
            compoundGeometry.maxWeaponLocal);
        if (!geometry.valid || compoundGeometry.children.empty()) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "Dynamic weapon compound snapshot rejected: stage=invalid-envelope generation={:016X} children={} points={}",
                compoundGeometry.generationKey,
                compoundGeometry.children.size(),
                compoundGeometry.sourcePointCount);
            return false;
        }

        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};

        std::vector<havok_compound_shape_builder::CompoundChild> compoundChildren;
        compoundChildren.reserve(compoundGeometry.children.size());
        for (std::size_t childIndex = 0; childIndex < compoundGeometry.children.size(); ++childIndex) {
            const auto& sourceChild = compoundGeometry.children[childIndex];
            havok_compound_shape_builder::CompoundChild compoundChild{};
            compoundChild.shape = sourceChild.shape;
            if (!makeCompoundChildTransform(
                    sourceChild.shapeInWeapon,
                    geometry.centerWeaponLocal,
                    scale,
                    compoundChild.transform)) {
                ROCK_LOG_ERROR(
                    Weapon,
                    "Dynamic weapon compound build failed: stage=child-pose generation={:016X} child={} center=({:.3f},{:.3f},{:.3f})",
                    compoundGeometry.generationKey,
                    childIndex,
                    sourceChild.shapeInWeapon.translate.x,
                    sourceChild.shapeInWeapon.translate.y,
                    sourceChild.shapeInWeapon.translate.z);
                return false;
            }
            compoundChildren.push_back(compoundChild);
        }

        havok_compound_shape_builder::DynamicCompoundShape pendingCompoundShape;
        if (!pendingCompoundShape.create(compoundChildren)) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon compound build failed: stage=compound-constructor generation={:016X} children={} points={}",
                compoundGeometry.generationKey,
                compoundGeometry.children.size(),
                compoundGeometry.sourcePointCount);
            return false;
        }
        if (weaponCollision.getCurrentWeaponGenerationKey() != compoundGeometry.generationKey) {
            ROCK_LOG_WARN(
                Weapon,
                "Dynamic weapon compound build discarded: stage=generation-changed built={:016X} current={:016X}",
                compoundGeometry.generationKey,
                weaponCollision.getCurrentWeaponGenerationKey());
            return false;
        }

        if (_created) {
            retireProxyLocked(frame.bhkWorld);
        }

        _compoundShape = std::move(pendingCompoundShape);
        auto* shape = _compoundShape.get();

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
                "ROCK_DynamicWeaponCompound",
                kTrackedDynamicBodyCreationOptions)) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon compound build failed: stage=body-create generation={:016X} children={} points={}",
                compoundGeometry.generationKey,
                compoundGeometry.children.size(),
                compoundGeometry.sourcePointCount);
            _compoundShape.reset();
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
                "Dynamic weapon compound {} failed processed-manifold event opt-in: enabled={} readable={} flags=0x{:08X}",
                proxyBodyId.value,
                processedManifoldFlagEnabled,
                flaggedBody.valid && flaggedBody.body,
                flaggedBody.body ? flaggedBody.body->flags : 0u);
            _bodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
            _body.retireDeferred(frame.bhkWorld);
            _compoundShape.reset();
            return false;
        }

        _createdWorld = frame.hknpWorld;
        _createdBhkWorld = frame.bhkWorld;
        _createdGenerationKey = compoundGeometry.generationKey;
        _createdCenterWeaponLocal = geometry.centerWeaponLocal;
        _createdHalfExtentsWeaponLocal = geometry.halfExtentsWeaponLocal;
        _createdWeaponScale = scale;
        _createdInertiaEnvelopePaddingGameUnits = inertiaEnvelopePadding;
        _createdCompoundChildCount = static_cast<std::uint32_t>(compoundGeometry.children.size());
        _createdCompoundPointCount = compoundGeometry.sourcePointCount;
        _created = true;
        {
            std::scoped_lock poseLock(_compoundPoseMutex);
            _compoundPoseScratch.resize(compoundGeometry.children.size());
            _pendingCompoundChildTransforms.resize(compoundGeometry.children.size());
            for (std::size_t i = 0; i < compoundGeometry.children.size(); ++i) {
                _compoundPoseScratch[i].shapeInWeapon = compoundGeometry.children[i].shapeInWeapon;
                _pendingCompoundChildTransforms[i] = compoundChildren[i].transform;
            }
            _queuedCompoundPoseSequence = 1;
            _consumedCompoundPoseSequence = 1;
        }
        _rebuildRequestedAtomic.store(false, std::memory_order_release);
        const float bodyMass = dynamic_weapon_collision_policy::sanitizeWeaponMass(weaponIdentity.weightGame);
        _body.setMass(bodyMass);
        if (!applyWeaponEnvelopeMassProperties(
                frame.hknpWorld,
                _body.getBodyId(),
                geometry,
                scale,
                inertiaEnvelopePadding,
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
        // grip-to-contact-body-center offset. Passing these two generated-column frames
        // through the visual-object/proxy adapter invents a world-dependent
        // relative rotation and twists the compound sideways when the motor engages.
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
            "Dynamic weapon grip-constrained compound created: contactBody={} authorityBody={} constraint={} generation={:016X} children={} points={} authority=grip center=({:.2f},{:.2f},{:.2f}) envelopeHalf=({:.2f},{:.2f},{:.2f}) scale={:.3f} mass={:.2f} forces=({:.1f},{:.1f}) inertiaPadding={:.2f} layer={}",
            _body.getBodyId().value,
            _authorityProxy.getBodyId().value,
            _authorityConstraint.constraintId,
            _createdGenerationKey,
            _createdCompoundChildCount,
            _createdCompoundPointCount,
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
            _createdInertiaEnvelopePaddingGameUnits,
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

        {
            std::scoped_lock poseLock(_compoundPoseMutex);
            if (_queuedCompoundPoseSequence != _consumedCompoundPoseSequence) {
                const auto updateResult = _compoundShape.updateTransforms(_pendingCompoundChildTransforms);
                if (!updateResult.succeeded) {
                    _rebuildRequestedAtomic.store(true, std::memory_order_release);
                    _droveThisSubstep = false;
                    clearPublishedPhysicsSnapshot();
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        1000,
                        "Dynamic weapon live compound update failed: generation={:016X} children={}",
                        _createdGenerationKey,
                        _createdCompoundChildCount);
                    return;
                }
                _consumedCompoundPoseSequence = _queuedCompoundPoseSequence;
                if (updateResult.changedChildCount > 0 && g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
                    ROCK_LOG_SAMPLE_INFO(
                        Weapon,
                        500,
                        "Dynamic weapon live compound updated: body={} generation={:016X} changed={}/{}",
                        _body.getBodyId().value,
                        _createdGenerationKey,
                        updateResult.changedChildCount,
                        _createdCompoundChildCount);
                }
            }
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

        // The hidden authority remains exact, while active world contact uses
        // the same shared-authority motor policy as ROCK's loose held weapons.
        // Only tau changes; targets, recovery velocities, and force limits stay
        // owned by the existing grip constraint.
        updateWeaponGripConstraintContactTau(
            _authorityConstraint,
            _contactRetentionSeconds > 0.0f,
            driveResult.driveDeltaSeconds);

        _droveThisSubstep =
            driveResult.driven &&
            driveResult.hasRequestedTargetGameTransform;
        _physicsRequestedTargetValid = driveResult.hasRequestedTargetGameTransform;
        if (_physicsRequestedTargetValid) {
            _physicsRequestedAuthorityTarget = driveResult.requestedTargetGameTransform;
            _physicsRequestedTarget = dynamic_weapon_collision_policy::makeContactBodyTargetFromGripAuthority(
                driveResult.requestedTargetGameTransform,
                _createdCenterWeaponLocal,
                _createdWeaponScale);
        }

        bool contactBodyRecovered = false;
        if (_physicsRequestedTargetValid) {
            RE::NiTransform liveContactBodyWorld{};
            const bool liveContactBodyReadable =
                havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    _body.getBodyId(),
                    liveContactBodyWorld) &&
                dynamic_weapon_collision_policy::isFiniteTransform(
                    liveContactBodyWorld);
            const float translationGapGameUnits = liveContactBodyReadable ?
                dynamic_weapon_collision_policy::translationDeltaGameUnits(
                    liveContactBodyWorld,
                    _physicsRequestedTarget) :
                0.0f;
            const auto recovery =
                dynamic_weapon_collision_policy::advanceFreeSpaceDivergenceRecovery(
                    _divergenceDwellSeconds,
                    _contactRetentionSeconds > 0.0f,
                    driveResult.teleported,
                    translationGapGameUnits,
                    g_rockConfig.rockWeaponCollisionDynamicDivergenceTeleportGameUnits,
                    g_rockConfig.rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds,
                    driveResult.driveDeltaSeconds);
            _divergenceDwellSeconds = recovery.dwellSeconds;
            if (recovery.recover) {
                if (!placeGeneratedKeyframedBodyImmediately(
                        _body,
                        _physicsRequestedTarget)) {
                    _rebuildRequestedAtomic.store(true, std::memory_order_release);
                    _droveThisSubstep = false;
                    clearPublishedPhysicsSnapshot();
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        1000,
                        "Dynamic weapon free-space divergence recovery failed: body={} gap={:.2f}gu dwell={:.3f}s",
                        _body.getBodyId().value,
                        translationGapGameUnits,
                        _divergenceDwellSeconds);
                    return;
                }
                contactBodyRecovered = true;
                _divergenceDwellSeconds = 0.0f;
                ROCK_LOG_SAMPLE_WARN(
                    Weapon,
                    1000,
                    "Dynamic weapon free-space divergence recovered: body={} gap={:.2f}gu authorityTeleport={}",
                    _body.getBodyId().value,
                    translationGapGameUnits,
                    driveResult.teleported);
            }
        } else {
            _divergenceDwellSeconds = 0.0f;
        }

        _physicsDriveTeleported = driveResult.teleported || contactBodyRecovered;
        if (_physicsDriveTeleported) {
            _contactRetentionSeconds = 0.0f;
        }
    }

    void DynamicWeaponCollisionRuntime::samplePostSolve(
        RE::hknpWorld* world,
        const std::uint64_t solveSequence,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world ||
            !_body.isValid() || !_droveThisSubstep || !_physicsRequestedTargetValid) {
            return;
        }
        _droveThisSubstep = false;

        RE::NiTransform liveBodyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, _body.getBodyId(), liveBodyWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(liveBodyWorld)) {
            _contactRetentionSeconds = 0.0f;
            clearPublishedPhysicsSnapshot();
            return;
        }
        ++_postSolveSamplesSinceCreate;
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
                collision_layer_policy::isDynamicWeaponProxySolverObstacleLayer(otherLayer);
        }
        const bool contactWasActive = _contactRetentionSeconds > 0.0f;
        const bool contactEpisodeStarted =
            newMatchingContact &&
            (!contactWasActive || _activeContactOtherBodyId != otherBodyId);
        if (contactEpisodeStarted) {
            ++_contactEpisode;
            _activeContactOtherBodyId = otherBodyId;
        }
        _contactRetentionSeconds =
            dynamic_weapon_collision_policy::advanceProcessedManifoldContactRetention(
                _contactRetentionSeconds,
                newMatchingContact,
                _physicsDriveTeleported,
                havok_physics_timing::driveDeltaSeconds(timing));

        PhysicsSnapshot snapshot{};
        snapshot.valid = true;
        snapshot.contactActive = _contactRetentionSeconds > 0.0f;
        snapshot.teleported = _physicsDriveTeleported;
        snapshot.world = reinterpret_cast<std::uintptr_t>(world);
        snapshot.bodyId = _body.getBodyId().value;
        snapshot.otherBodyId = newMatchingContact ? otherBodyId : _snapshotOtherBodyIdAtomic.load(std::memory_order_relaxed);
        snapshot.otherLayer = newMatchingContact ? otherLayer : _snapshotOtherLayerAtomic.load(std::memory_order_relaxed);
        snapshot.contactRetentionSeconds = _contactRetentionSeconds;
        snapshot.generationKey = _createdGenerationKey;
        snapshot.solveSequence = solveSequence;
        snapshot.weaponScale = _createdWeaponScale;
        snapshot.requestedProxyBodyWorld = _physicsRequestedTarget;
        snapshot.liveProxyBodyWorld = liveBodyWorld;
        publishPhysicsSnapshot(snapshot);

        if (contactEpisodeStarted &&
            g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
            ContactDiagnosticSnapshot diagnostic{};
            diagnostic.valid = true;
            diagnostic.world = reinterpret_cast<std::uintptr_t>(world);
            diagnostic.bodyId = _body.getBodyId().value;
            diagnostic.otherBodyId = otherBodyId;
            diagnostic.otherLayer = otherLayer;
            diagnostic.generationKey = _createdGenerationKey;
            diagnostic.contactEpisode = _contactEpisode;
            diagnostic.contactSolveAge = _postSolveSamplesSinceCreate;
            diagnostic.requestedProxyBodyWorld = _physicsRequestedTarget;
            diagnostic.liveProxyBodyWorld = liveBodyWorld;

            const auto otherBodySnapshot = havok_runtime::snapshotBody(
                world,
                RE::hknpBodyId{ otherBodyId });
            diagnostic.otherMotionIndex = otherBodySnapshot.motionIndex;
            diagnostic.otherCollisionObject = reinterpret_cast<std::uintptr_t>(
                otherBodySnapshot.collisionObject);
            diagnostic.otherOwnerNode = reinterpret_cast<std::uintptr_t>(
                otherBodySnapshot.ownerNode);
            diagnostic.otherBodyWorldValid =
                havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    RE::hknpBodyId{ otherBodyId },
                    diagnostic.otherBodyWorld) &&
                dynamic_weapon_collision_policy::isFiniteTransform(
                    diagnostic.otherBodyWorld);

            const auto rawWitnessSequence =
                _rawContactWitnessSequenceAtomic.load(std::memory_order_acquire);
            const auto rawOtherBodyId =
                _rawContactOtherBodyIdAtomic.load(std::memory_order_relaxed);
            diagnostic.rawContactPointValid =
                rawWitnessSequence != 0 &&
                rawWitnessSequence != _lastEpisodeRawWitnessSequence &&
                rawOtherBodyId == otherBodyId;
            _lastEpisodeRawWitnessSequence = rawWitnessSequence;
            if (diagnostic.rawContactPointValid) {
                diagnostic.rawContactPointCount =
                    _rawContactPointCountAtomic.load(std::memory_order_relaxed);
                diagnostic.rawContactPointIndex =
                    _rawContactPointIndexAtomic.load(std::memory_order_relaxed);
                diagnostic.rawContactPointWeightSum =
                    _rawContactPointWeightSumAtomic.load(std::memory_order_relaxed);
                diagnostic.rawContactProxyWasBodyA =
                    _rawContactProxyWasBodyAAtomic.load(std::memory_order_relaxed);
                const float pointScale = physics_scale::havokToGame();
                diagnostic.rawContactPointGame = RE::NiPoint3{
                    _rawContactPointHavokAtomic[0].load(std::memory_order_relaxed) * pointScale,
                    _rawContactPointHavokAtomic[1].load(std::memory_order_relaxed) * pointScale,
                    _rawContactPointHavokAtomic[2].load(std::memory_order_relaxed) * pointScale,
                };
                diagnostic.rawContactNormalHavok = RE::NiPoint3{
                    _rawContactNormalHavokAtomic[0].load(std::memory_order_relaxed),
                    _rawContactNormalHavokAtomic[1].load(std::memory_order_relaxed),
                    _rawContactNormalHavokAtomic[2].load(std::memory_order_relaxed),
                };
            }
            publishContactDiagnosticSnapshot(diagnostic);
        }

        if (g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
            const float requestedStepTranslation = _physicsPreviousRequestedTargetValid ?
                dynamic_weapon_collision_policy::translationDeltaGameUnits(
                    _physicsPreviousRequestedTarget,
                    _physicsRequestedTarget) :
                0.0f;
            const float requestedStepRotation = _physicsPreviousRequestedTargetValid ?
                dynamic_weapon_collision_policy::rotationDeltaDegrees(
                    _physicsPreviousRequestedTarget,
                    _physicsRequestedTarget) :
                0.0f;
            const float signedPressStep = _physicsPreviousRequestedTargetValid ?
                signedTranslationStepTowardContactError(
                    _physicsPreviousRequestedTarget,
                    _physicsRequestedTarget,
                    liveBodyWorld) :
                0.0f;
            const float contactTranslationError = dynamic_weapon_collision_policy::translationDeltaGameUnits(
                _physicsRequestedTarget,
                liveBodyWorld);
            const float contactRotationError = dynamic_weapon_collision_policy::rotationDeltaDegrees(
                _physicsRequestedTarget,
                liveBodyWorld);

            RE::NiTransform liveAuthorityWorld{};
            const bool authorityReadable = havok_runtime::tryResolveLiveBodyWorldTransform(
                world,
                _authorityProxy.getBodyId(),
                liveAuthorityWorld);
            const float authorityTranslationError = authorityReadable ?
                dynamic_weapon_collision_policy::translationDeltaGameUnits(
                    _physicsRequestedAuthorityTarget,
                    liveAuthorityWorld) :
                -1.0f;
            const float authorityRotationError = authorityReadable ?
                dynamic_weapon_collision_policy::rotationDeltaDegrees(
                    _physicsRequestedAuthorityTarget,
                    liveAuthorityWorld) :
                -1.0f;
            const auto* linearMotor = _authorityConstraint.linearMotor;
            const auto* angularMotor = _authorityConstraint.angularMotor;

            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                500,
                "DWC motor trace: contact={} newCallback={} intentStep=({:.3f}gu,{:.2f}deg) signedPress={:.3f}gu contactError=({:.2f}gu,{:.2f}deg) authority(read/error)={}/({:.3f}gu,{:.2f}deg) tau=({:.4f},{:.4f}) recovery=({:.2f}/{:.2f},{:.2f}/{:.2f}) force=({:.1f},{:.1f})",
                snapshot.contactActive,
                newMatchingContact,
                requestedStepTranslation,
                requestedStepRotation,
                signedPressStep,
                contactTranslationError,
                contactRotationError,
                authorityReadable,
                authorityTranslationError,
                authorityRotationError,
                linearMotor ? linearMotor->tau : -1.0f,
                angularMotor ? angularMotor->tau : -1.0f,
                linearMotor ? linearMotor->proportionalRecoveryVelocity : -1.0f,
                linearMotor ? linearMotor->constantRecoveryVelocity : -1.0f,
                angularMotor ? angularMotor->proportionalRecoveryVelocity : -1.0f,
                angularMotor ? angularMotor->constantRecoveryVelocity : -1.0f,
                linearMotor ? linearMotor->maxForce : -1.0f,
                angularMotor ? angularMotor->maxForce : -1.0f);
        }
        _physicsPreviousRequestedTarget = _physicsRequestedTarget;
        _physicsPreviousRequestedTargetValid = true;
        _physicsDriveTeleported = false;
    }

    bool DynamicWeaponCollisionRuntime::isProxyBodyIdAtomic(const std::uint32_t bodyId) const
    {
        return bodyId != kInvalidBodyId && _bodyIdAtomic.load(std::memory_order_acquire) == bodyId;
    }

    void DynamicWeaponCollisionRuntime::recordObstacleContactCallback(
        RE::hknpWorld* world,
        const std::uint32_t proxyBodyId,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer,
        const bool proxyWasBodyA,
        const havok_runtime::ContactSignalPointResult* rawContactPoint)
    {
        if (!world || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isDynamicWeaponProxySolverObstacleLayer(otherLayer)) {
            return;
        }
        _obstacleCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!rawContactPoint || !rawContactPoint->valid) {
            return;
        }
        _rawPointCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (g_rockConfig.rockDebugDrawDynamicWeaponColliders) {
            _rawContactOtherBodyIdAtomic.store(otherBodyId, std::memory_order_relaxed);
            _rawContactPointCountAtomic.store(rawContactPoint->pointCount, std::memory_order_relaxed);
            _rawContactPointIndexAtomic.store(rawContactPoint->selectedPointIndex, std::memory_order_relaxed);
            _rawContactPointWeightSumAtomic.store(rawContactPoint->contactPointWeightSum, std::memory_order_relaxed);
            for (std::size_t axis = 0; axis < 3; ++axis) {
                _rawContactPointHavokAtomic[axis].store(rawContactPoint->contactPointHavok[axis], std::memory_order_relaxed);
                _rawContactNormalHavokAtomic[axis].store(rawContactPoint->contactNormalHavok[axis], std::memory_order_relaxed);
            }
            _rawContactProxyWasBodyAAtomic.store(proxyWasBodyA, std::memory_order_relaxed);
            _rawContactWitnessSequenceAtomic.fetch_add(1, std::memory_order_release);
        }
        _contactWorldAtomic.store(reinterpret_cast<std::uintptr_t>(world), std::memory_order_relaxed);
        _contactProxyBodyIdAtomic.store(proxyBodyId, std::memory_order_relaxed);
        _contactOtherBodyIdAtomic.store(otherBodyId, std::memory_order_relaxed);
        _contactOtherLayerAtomic.store(otherLayer, std::memory_order_relaxed);
        _contactSequenceAtomic.fetch_add(1, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::recordObstacleManifoldProcessedCallback(
        RE::hknpWorld* world,
        const std::uint32_t proxyBodyId,
        const std::uint32_t otherBodyId,
        const bool otherLayerRead,
        const std::uint32_t otherLayer,
        const std::int32_t manifoldPointCount)
    {
        if (!world || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isDynamicWeaponProxySolverObstacleLayer(otherLayer)) {
            return;
        }
        _obstacleCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!dynamic_weapon_collision_policy::hasSolvedProcessedManifoldContact(
                manifoldPointCount)) {
            return;
        }
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
                "Dynamic weapon grip-constrained compound retired: contactBody={} authorityBody={} constraint={}",
                bodyId,
                authorityBodyId,
                constraintId);
        }
    }

    void DynamicWeaponCollisionRuntime::clearLocalProxyStateLocked()
    {
        {
            std::scoped_lock poseLock(_compoundPoseMutex);
            _compoundPoseScratch.clear();
            _pendingCompoundChildTransforms.clear();
            _queuedCompoundPoseSequence = 0;
            _consumedCompoundPoseSequence = 0;
            _compoundShape.reset();
        }
        _createdWorld = nullptr;
        _createdBhkWorld = nullptr;
        _createdGenerationKey = 0;
        _createdCenterWeaponLocal = {};
        _createdHalfExtentsWeaponLocal = {};
        _createdWeaponScale = 1.0f;
        _createdInertiaEnvelopePaddingGameUnits = 0.0f;
        _createdCompoundChildCount = 0;
        _createdCompoundPointCount = 0;
        _created = false;
        _droveThisSubstep = false;
        _physicsRequestedTargetValid = false;
        _physicsDriveTeleported = false;
        _physicsRequestedAuthorityTarget = {};
        _physicsRequestedTarget = {};
        _physicsPreviousRequestedTarget = {};
        _physicsPreviousRequestedTargetValid = false;
        _divergenceDwellSeconds = 0.0f;
        _contactRetentionSeconds = 0.0f;
        _consumedContactSequence = 0;
        _contactEpisode = 0;
        _reportedContactEpisode = 0;
        _postSolveSamplesSinceCreate = 0;
        _lastEpisodeRawWitnessSequence = 0;
        _activeContactOtherBodyId = kInvalidBodyId;
        _proxyPairCallbackSequenceAtomic.store(0, std::memory_order_release);
        _obstacleCallbackSequenceAtomic.store(0, std::memory_order_release);
        _rawPointCallbackSequenceAtomic.store(0, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.store(0, std::memory_order_release);
        _contactSequenceAtomic.store(0, std::memory_order_release);
        _contactWorldAtomic.store(0, std::memory_order_release);
        _contactProxyBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _contactOtherBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _contactOtherLayerAtomic.store(0, std::memory_order_release);
        _rawContactOtherBodyIdAtomic.store(kInvalidBodyId, std::memory_order_release);
        _rawContactPointCountAtomic.store(0, std::memory_order_release);
        _rawContactPointIndexAtomic.store(0, std::memory_order_release);
        _rawContactPointWeightSumAtomic.store(0.0f, std::memory_order_release);
        for (std::size_t axis = 0; axis < 3; ++axis) {
            _rawContactPointHavokAtomic[axis].store(0.0f, std::memory_order_release);
            _rawContactNormalHavokAtomic[axis].store(0.0f, std::memory_order_release);
        }
        _rawContactProxyWasBodyAAtomic.store(false, std::memory_order_release);
        _rawContactWitnessSequenceAtomic.store(0, std::memory_order_release);
        _rebuildRequestedAtomic.store(false, std::memory_order_release);
        clearGeneratedKeyframedBodyDriveState(_authorityDriveState);
        clearPublishedPhysicsSnapshot();
        clearContactDiagnosticSnapshot();
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
                "Dynamic weapon grip-constrained compound abandoned after Havok world loss: contactBody={} authorityBody={} constraint={}",
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
        _snapshotContactRetentionSecondsAtomic.store(snapshot.contactRetentionSeconds, std::memory_order_relaxed);
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
            candidate.contactRetentionSeconds = _snapshotContactRetentionSecondsAtomic.load(std::memory_order_relaxed);
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

    void DynamicWeaponCollisionRuntime::clearContactDiagnosticSnapshot()
    {
        ContactDiagnosticSnapshot snapshot{};
        publishContactDiagnosticSnapshot(snapshot);
    }

    void DynamicWeaponCollisionRuntime::publishContactDiagnosticSnapshot(
        const ContactDiagnosticSnapshot& snapshot)
    {
        _contactDiagnosticVersionAtomic.fetch_add(1, std::memory_order_acq_rel);
        _contactDiagnosticValidAtomic.store(snapshot.valid, std::memory_order_relaxed);
        _contactDiagnosticRawPointValidAtomic.store(snapshot.rawContactPointValid, std::memory_order_relaxed);
        _contactDiagnosticProxyWasBodyAAtomic.store(snapshot.rawContactProxyWasBodyA, std::memory_order_relaxed);
        _contactDiagnosticOtherWorldValidAtomic.store(snapshot.otherBodyWorldValid, std::memory_order_relaxed);
        _contactDiagnosticWorldAtomic.store(snapshot.world, std::memory_order_relaxed);
        _contactDiagnosticBodyIdAtomic.store(snapshot.bodyId, std::memory_order_relaxed);
        _contactDiagnosticOtherBodyIdAtomic.store(snapshot.otherBodyId, std::memory_order_relaxed);
        _contactDiagnosticOtherLayerAtomic.store(snapshot.otherLayer, std::memory_order_relaxed);
        _contactDiagnosticOtherMotionIndexAtomic.store(snapshot.otherMotionIndex, std::memory_order_relaxed);
        _contactDiagnosticRawPointCountAtomic.store(snapshot.rawContactPointCount, std::memory_order_relaxed);
        _contactDiagnosticRawPointIndexAtomic.store(snapshot.rawContactPointIndex, std::memory_order_relaxed);
        _contactDiagnosticGenerationKeyAtomic.store(snapshot.generationKey, std::memory_order_relaxed);
        _contactDiagnosticEpisodeAtomic.store(snapshot.contactEpisode, std::memory_order_relaxed);
        _contactDiagnosticSolveAgeAtomic.store(snapshot.contactSolveAge, std::memory_order_relaxed);
        _contactDiagnosticOtherCollisionObjectAtomic.store(snapshot.otherCollisionObject, std::memory_order_relaxed);
        _contactDiagnosticOtherOwnerNodeAtomic.store(snapshot.otherOwnerNode, std::memory_order_relaxed);
        _contactDiagnosticRawPointWeightSumAtomic.store(snapshot.rawContactPointWeightSum, std::memory_order_relaxed);
        _contactDiagnosticRawPointGameAtomic[0].store(snapshot.rawContactPointGame.x, std::memory_order_relaxed);
        _contactDiagnosticRawPointGameAtomic[1].store(snapshot.rawContactPointGame.y, std::memory_order_relaxed);
        _contactDiagnosticRawPointGameAtomic[2].store(snapshot.rawContactPointGame.z, std::memory_order_relaxed);
        _contactDiagnosticRawNormalHavokAtomic[0].store(snapshot.rawContactNormalHavok.x, std::memory_order_relaxed);
        _contactDiagnosticRawNormalHavokAtomic[1].store(snapshot.rawContactNormalHavok.y, std::memory_order_relaxed);
        _contactDiagnosticRawNormalHavokAtomic[2].store(snapshot.rawContactNormalHavok.z, std::memory_order_relaxed);
        storeAtomicTransform(_contactDiagnosticRequestedProxyBodyWorld, snapshot.requestedProxyBodyWorld);
        storeAtomicTransform(_contactDiagnosticLiveProxyBodyWorld, snapshot.liveProxyBodyWorld);
        storeAtomicTransform(_contactDiagnosticOtherBodyWorld, snapshot.otherBodyWorld);
        _contactDiagnosticVersionAtomic.fetch_add(1, std::memory_order_release);
    }

    bool DynamicWeaponCollisionRuntime::readContactDiagnosticSnapshot(
        ContactDiagnosticSnapshot& outSnapshot) const
    {
        for (int attempt = 0; attempt < 4; ++attempt) {
            const auto before = _contactDiagnosticVersionAtomic.load(std::memory_order_acquire);
            if ((before & 1u) != 0) {
                continue;
            }

            ContactDiagnosticSnapshot candidate{};
            candidate.valid = _contactDiagnosticValidAtomic.load(std::memory_order_relaxed);
            candidate.rawContactPointValid = _contactDiagnosticRawPointValidAtomic.load(std::memory_order_relaxed);
            candidate.rawContactProxyWasBodyA = _contactDiagnosticProxyWasBodyAAtomic.load(std::memory_order_relaxed);
            candidate.otherBodyWorldValid = _contactDiagnosticOtherWorldValidAtomic.load(std::memory_order_relaxed);
            candidate.world = _contactDiagnosticWorldAtomic.load(std::memory_order_relaxed);
            candidate.bodyId = _contactDiagnosticBodyIdAtomic.load(std::memory_order_relaxed);
            candidate.otherBodyId = _contactDiagnosticOtherBodyIdAtomic.load(std::memory_order_relaxed);
            candidate.otherLayer = _contactDiagnosticOtherLayerAtomic.load(std::memory_order_relaxed);
            candidate.otherMotionIndex = _contactDiagnosticOtherMotionIndexAtomic.load(std::memory_order_relaxed);
            candidate.rawContactPointCount = _contactDiagnosticRawPointCountAtomic.load(std::memory_order_relaxed);
            candidate.rawContactPointIndex = _contactDiagnosticRawPointIndexAtomic.load(std::memory_order_relaxed);
            candidate.generationKey = _contactDiagnosticGenerationKeyAtomic.load(std::memory_order_relaxed);
            candidate.contactEpisode = _contactDiagnosticEpisodeAtomic.load(std::memory_order_relaxed);
            candidate.contactSolveAge = _contactDiagnosticSolveAgeAtomic.load(std::memory_order_relaxed);
            candidate.otherCollisionObject = _contactDiagnosticOtherCollisionObjectAtomic.load(std::memory_order_relaxed);
            candidate.otherOwnerNode = _contactDiagnosticOtherOwnerNodeAtomic.load(std::memory_order_relaxed);
            candidate.rawContactPointWeightSum = _contactDiagnosticRawPointWeightSumAtomic.load(std::memory_order_relaxed);
            candidate.rawContactPointGame = RE::NiPoint3{
                _contactDiagnosticRawPointGameAtomic[0].load(std::memory_order_relaxed),
                _contactDiagnosticRawPointGameAtomic[1].load(std::memory_order_relaxed),
                _contactDiagnosticRawPointGameAtomic[2].load(std::memory_order_relaxed),
            };
            candidate.rawContactNormalHavok = RE::NiPoint3{
                _contactDiagnosticRawNormalHavokAtomic[0].load(std::memory_order_relaxed),
                _contactDiagnosticRawNormalHavokAtomic[1].load(std::memory_order_relaxed),
                _contactDiagnosticRawNormalHavokAtomic[2].load(std::memory_order_relaxed),
            };
            candidate.requestedProxyBodyWorld = loadAtomicTransform(_contactDiagnosticRequestedProxyBodyWorld);
            candidate.liveProxyBodyWorld = loadAtomicTransform(_contactDiagnosticLiveProxyBodyWorld);
            candidate.otherBodyWorld = loadAtomicTransform(_contactDiagnosticOtherBodyWorld);

            const auto after = _contactDiagnosticVersionAtomic.load(std::memory_order_acquire);
            if (before == after && (after & 1u) == 0) {
                outSnapshot = candidate;
                return candidate.valid;
            }
        }
        outSnapshot = {};
        return false;
    }
}

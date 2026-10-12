#include "physics-interaction/weapon/DynamicWeaponCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/core/PhysicsFrameContext.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabInertiaPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponRestSupport.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>
#include <utility>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;
        constexpr std::uint32_t kDynamicWeaponCollisionGroup = 0x000Du;
        // FO4VR 141551F00 installs the contact-impulse creator at bundle+18;
        // 14154BB41..14154BB51 registers it for body flag 0x80, and its
        // 1418012D0 solver callback emits key-3 events consumed by native audio.
        // The separate 0x40/key-2 opt-in only reports ongoing manifold contact.
        // Both flags belong to this generated body's lifetime, including rebuilds.
        constexpr std::uint32_t kRequiredContactEventFlags = 0x40u | 0x80u;
        constexpr std::uint32_t kRebuildBodyCollisionState = 0u;

        [[nodiscard]] bool dynamicWeaponDebugEnabled()
        {
            return g_rockConfig.rockDebugShowColliders &&
                   g_rockConfig.rockDebugDrawDynamicWeaponColliders;
        }

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

        std::uint32_t dynamicWeaponProxyFilterInfo(bool equippedDrop = false)
        {
            return (kDynamicWeaponCollisionGroup << 16) |
                   (equippedDrop ? collision_layer_policy::ROCK_LAYER_EQUIPPED_DROP_PROXY :
                       collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        }

        float scaleFiniteValue(const float value, const float multiplier)
        {
            return (std::isfinite(value) ? value : 0.0f) * (std::isfinite(multiplier) ? multiplier : 1.0f);
        }

        bool setOwnedWeaponMotionProfile(RE::hknpWorld* world, BethesdaPhysicsBody& body,
            std::uint16_t profile, const char*& stage)
        {
            stage = "motion-profile-owner";
            if (!world || !body.isValid() || profile == equipped_drop_motion_profile::kInvalid) return false;
            const auto before = havok_runtime::snapshotBody(world, body.getBodyId());
            if (!before.valid || !before.motion || before.motionIndex == 0 ||
                before.collisionObject != body.getCollisionObject()) return false;
            stage = "motion-profile-library";
            havok_runtime::MotionPropertiesLibrarySnapshot library{};
            if (!havok_runtime::snapshotMotionPropertiesLibrary(world, library) || !library.valid || profile >= library.count) return false;

            // FO4VR 14153B35B..14153B379 writes motion+38 and publishes the
            // native change signal. 1417E1EE4..1417E1EFB independently writes
            // the same uint16 from motion-cinfo+0 during initialization.
            using Setter = void (*)(RE::hknpWorld*, RE::hknpBodyId, RE::hknpMotionPropertiesId_Handle);
            static const Setter setProfile = []() -> Setter {
                const auto address = REL::Module::get().base() + offsets::kFunc_SetBodyMotionProperties;
                constexpr std::array<std::uint8_t, 20> expected{
                    0x48, 0x89, 0x5C, 0x24, 0x08, 0x48, 0x89, 0x74, 0x24, 0x20,
                    0x66, 0x44, 0x89, 0x44, 0x24, 0x18, 0x89, 0x54, 0x24, 0x10 };
                std::array<std::uint8_t, expected.size()> actual{};
                return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), actual.data(), actual.size()) && actual == expected ?
                    reinterpret_cast<Setter>(address) : nullptr;
            }();
            stage = "motion-profile-native-entry";
            if (!setProfile) return false;
            RE::hknpMotionPropertiesId_Handle target{};
            target.value = profile;
            setProfile(world, body.getBodyId(), target);
            stage = "motion-profile-readback";
            const auto after = havok_runtime::snapshotBody(world, body.getBodyId());
            std::uint16_t actual = equipped_drop_motion_profile::kInvalid;
            return after.valid && after.motion && after.motionIndex == before.motionIndex &&
                after.collisionObject == body.getCollisionObject() &&
                havok_runtime::tryReadMotionPropertiesId(after.motion, actual) && actual == profile;
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

        bool updateWeaponGripConstraintContactResponse(
            ActiveConstraint& constraint,
            const GrabConstraintMotorTuning& baseTuning,
            const GrabMotorBodyProperties& properties,
            const bool contactActive,
            const weapon_physics_time_scale::HandlingScale& timeScale)
        {
            if (!constraint.linearMotor || !constraint.angularMotor) {
                return false;
            }

            const float baseLinearTau = grab_motion_controller::safePositive(
                baseTuning.linearTau,
                0.03f);
            const float baseAngularTau = grab_motion_controller::safePositive(
                baseTuning.angularTau,
                baseLinearTau);
            const float collisionTau = grab_motion_controller::safePositive(
                scaleFiniteValue(
                    g_rockConfig.rockGrabTauMin,
                    g_rockConfig.rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier),
                baseLinearTau);
            const auto input = grab_motion_controller::MotorInput{
                .heldBodyColliding = contactActive,
                .baseLinearTau = baseLinearTau,
                .baseAngularTau = baseAngularTau,
                .collisionTau = collisionTau,
                .currentLinearTau = constraint.linearMotor->tau,
                .currentAngularTau = constraint.angularMotor->tau,
                .tauLerpSpeed = g_rockConfig.rockGrabTauLerpSpeed,
                .deltaTime = timeScale.responseDeltaSeconds,
                .physicsDeltaSeconds = timeScale.responseDeltaSeconds / timeScale.velocity,
                .baseMaxForce = baseTuning.linearMaxForce,
                .angularForceMultiplier = g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier,
                .mass = properties.mass,
                .maximumInertia = properties.valid ? properties.maximumInertia : 0.0f,
                .gripRadiusHavok = properties.gripRadiusHavok,
                .freeLinearAcceleration = g_rockConfig.rockGrabFreeLinearAcceleration,
                .freeAngularAcceleration = g_rockConfig.rockGrabFreeAngularAcceleration,
                .forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio,
                .effectiveMotorMassFloorEnabled = g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
                .effectiveMotorMassFloor = g_rockConfig.rockGrabEffectiveMotorMassFloor,
                .fadeInEnabled = false,
            };
            grab_motion_controller::HeldAuthorityState authority{};
            authority.softenForContact = contactActive;
            const auto output = grab_motion_controller::solveMotorTargetsWithAuthority(input, authority);
            if (!output.valid) return false;
            constraint.linearMotor->tau = output.linearTau;
            constraint.angularMotor->tau = output.angularTau;
            const auto linearRecovery = dynamic_weapon_collision_policy::resolveContactMotorRecovery(
                grab_motion_controller::safePositive(baseTuning.linearDamping, 0.8f),
                grab_motion_controller::safePositive(baseTuning.linearConstantRecovery, 1.0f),
                baseLinearTau, collisionTau, constraint.linearMotor->tau, contactActive);
            const auto angularRecovery = dynamic_weapon_collision_policy::resolveContactMotorRecovery(
                grab_motion_controller::safePositive(baseTuning.angularDamping, 0.8f),
                grab_motion_controller::safePositive(baseTuning.angularConstantRecovery, 1.0f),
                baseAngularTau, collisionTau, constraint.angularMotor->tau, contactActive);
            constraint.linearMotor->damping = linearRecovery.damping;
            constraint.linearMotor->constantRecoveryVelocity = linearRecovery.constantRecoveryVelocity * timeScale.velocity;
            constraint.angularMotor->damping = angularRecovery.damping;
            constraint.angularMotor->constantRecoveryVelocity = angularRecovery.constantRecoveryVelocity * timeScale.velocity;
            // FO4VR 141AFD739..141AFD749 multiplies both recovery rates by
            // solver dt. 141AFD998..141AFD9B2 converts force bounds to impulse
            // bounds. Preserve recovery per real second and acceleration per
            // real second squared, keeping dimensionless tau/damping intact.
            constraint.linearMotor->proportionalRecoveryVelocity =
                grab_motion_controller::safePositive(baseTuning.linearProportionalRecovery, 2.0f) * timeScale.velocity;
            constraint.angularMotor->proportionalRecoveryVelocity =
                grab_motion_controller::safePositive(baseTuning.angularProportionalRecovery, 2.0f) * timeScale.velocity;
            constraint.linearMotor->maxForce = output.linearMaxForce * timeScale.force;
            constraint.linearMotor->minForce = -constraint.linearMotor->maxForce;
            constraint.angularMotor->maxForce = output.angularMaxForce * timeScale.force;
            constraint.angularMotor->minForce = -constraint.angularMotor->maxForce;
            constraint.currentTau = constraint.linearMotor->tau;
            constraint.currentMaxForce = constraint.linearMotor->maxForce;
            constraint.targetMaxForce = constraint.linearMotor->maxForce;
            return true;
        }

        bool rebaseWeaponVelocity(RE::hknpWorld* world, BethesdaPhysicsBody& body, float ratio, const char*& stage)
        {
            if (ratio == 1.0f) return true;
            stage = "ratio";
            if (!std::isfinite(ratio) || ratio <= 0.0f) return false;
            stage = "motion-owner";
            const auto snapshot = havok_runtime::snapshotBody(world, body.getBodyId());
            if (!snapshot.valid || !snapshot.motion || snapshot.collisionObject != body.getCollisionObject()) {
                return false;
            }
            const auto q = snapshot.motion->orientation;
            const float quaternion[4]{q.x, q.y, q.z, q.w};
            const auto linear = snapshot.motion->linearVelocity;
            const auto angular = snapshot.motion->angularVelocity;
            // The native setter at 14153A0A8..14153A10F rotates world angular
            // velocity into the motion's local frame. Invert that rotation
            // before calling the existing setter; never pass local omega as
            // world omega. 1417D37D0/1417D3829 independently consume these
            // linear/angular fields for swept collision bounds.
            RE::NiPoint3 worldAngular{};
            stage = "angular-frame";
            if (!weapon_physics_time_scale::rebaseWorldAngularVelocity(quaternion,
                    RE::NiPoint3{angular.x, angular.y, angular.z}, ratio, worldAngular)) return false;
            alignas(16) const float linearWorld[4]{linear.x * ratio, linear.y * ratio, linear.z * ratio, 0.0f};
            alignas(16) const float angularWorld[4]{worldAngular.x, worldAngular.y, worldAngular.z, 0.0f};
            stage = "velocity-components";
            if (!havok_runtime::isFinite3(linearWorld) || !havok_runtime::isFinite3(angularWorld)) return false;
            // The native writer also refreshes swept bounds and activation.
            // Do not retain the borrowed motion across this native mutation.
            stage = "native-velocity-writer";
            return body.setVelocity(linearWorld, angularWorld);
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

            constexpr float inverseInertiaMultiplier =
                dynamic_weapon_collision_policy::kInverseInertiaMultiplier;

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
        const bool enabled)
    {
        _frameIndex = frameIndex;
        _frameWorld = world;
        _frameBhkWorld = bhkWorld;
        _frameWeaponNode = weaponNode;
        _frameGenerationKey = weaponGenerationKey;
        if (_surfaceSupport.ownsPose() &&
            (_surfaceSupport.contact.world != reinterpret_cast<std::uintptr_t>(world) ||
                _surfaceSupport.contact.generation != weaponGenerationKey || !weaponNode)) {
            ROCK_LOG_INFO(Weapon, "Weapon surface support cleared: reason=weapon-or-world-changed");
            _surfaceSupport = {};
        }
        _frameRequestedWeaponWorld = {};
        _frameAcceptingIntent = enabled && world && bhkWorld && weaponNode && weaponGenerationKey != 0;
        // Only explicit collision-free intent publications can drive the body.
        // Never alternate between the rendered weapon and a reconstructed hand
        // according to whether the previous frame had a visible correction.
        _frameHasIntent = false;
        _frameIntentDriverValid = false;
        _gripRecoveryDistanceGameUnitsAtomic.store(
            g_rockConfig.rockWeaponCollisionGripRecoveryDistanceGameUnits,
            std::memory_order_release);
        _enabledAtomic.store(_frameAcceptingIntent, std::memory_order_release);
    }

    void DynamicWeaponCollisionRuntime::observeWeaponVisualIntent(
        void* context,
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const std::uint64_t weaponGenerationKey,
        const dynamic_weapon_collision_policy::VisualIntentSource,
        const RE::NiTransform* physicalDriverWorld)
    {
        auto* runtime = static_cast<DynamicWeaponCollisionRuntime*>(context);
        if (runtime) {
            runtime->captureVisualIntent(weaponNode, requestedWeaponWorld, weaponGenerationKey, physicalDriverWorld);
        }
    }

    void DynamicWeaponCollisionRuntime::captureVisualIntent(
        RE::NiNode* weaponNode,
        const RE::NiTransform& requestedWeaponWorld,
        const std::uint64_t weaponGenerationKey,
        const RE::NiTransform* physicalDriverWorld)
    {
        if (!_frameAcceptingIntent || weaponNode != _frameWeaponNode || weaponGenerationKey != _frameGenerationKey ||
            !dynamic_weapon_collision_policy::isFiniteTransform(requestedWeaponWorld)) {
            return;
        }
        _frameRequestedWeaponWorld = requestedWeaponWorld;
        _frameHasIntent = true;
        _frameIntentDriverValid = physicalDriverWorld && dynamic_weapon_collision_policy::isFiniteTransform(*physicalDriverWorld);
        if (_frameIntentDriverValid) {
            _frameIntentDriverWorld = *physicalDriverWorld;
        }
    }

    bool DynamicWeaponCollisionRuntime::setAuthorityPivot(
        const PhysicsFrameContext& frame, const RE::NiPoint3& pivotWeaponLocal,
        const RE::NiTransform& weaponWorld)
    {
        if (_authorityConstraint.isValid() &&
            weaponSolverLength(weaponSolverSub(pivotWeaponLocal, _authorityPivotWeaponLocal)) < 0.0001f) {
            return true;
        }
        auto mutation = _physicsCallbackGate ? _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (!_created || _createdWorld != frame.hknpWorld || !_body.isValid() || !_authorityProxy.isValid()) return false;

        const auto target = dynamic_weapon_collision_policy::makeGripAuthorityTarget(weaponWorld, pivotWeaponLocal);
        destroyGrabConstraint(frame.hknpWorld, _authorityConstraint);
        if (!placeGeneratedKeyframedBodyImmediately(_authorityProxy, target)) return false;
        const auto relation = dynamic_weapon_collision_policy::makeContactBodyInGripAuthoritySpace(
            _createdCenterWeaponLocal, _createdWeaponScale, pivotWeaponLocal);
        _authorityConstraint = createGrabConstraint(frame.hknpWorld,
            _authorityProxy.getBodyId(), _body.getBodyId(), target, target.translate,
            relation, buildWeaponGripConstraintTuning(_createdMass));
        if (!_authorityConstraint.isValid()) return false;
        _authorityPivotWeaponLocal = pivotWeaponLocal;
        initializeGeneratedKeyframedBodyDriveState(_authorityDriveState, target);
        _physicsRequestedTargetValid = false;
        _droveThisSubstep = false;
        _divergenceDwellSeconds = 0.0f;
        clearPublishedPhysicsSnapshot();
        return true;
    }

    bool DynamicWeaponCollisionRuntime::beginEquippedDrop(const PhysicsFrameContext& frame,
        RE::NiNode* weapon, const std::uint64_t generation, RE::NiTransform& pose)
    {
        auto mutation = _physicsCallbackGate ? _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_equippedDrop || !_created || !_body.isValid() || !_authorityConstraint.isValid() ||
            _createdWorld != frame.hknpWorld || _createdBhkWorld != frame.bhkWorld ||
            weapon != _frameWeaponNode || generation == 0 || generation != _createdGenerationKey ||
            native_interaction_service::weaponControlled() || _rebuildRequestedAtomic.load(std::memory_order_acquire)) return false;
        RE::NiTransform live{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(frame.hknpWorld, _body.getBodyId(), live) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(live)) return false;
        pose = dynamic_weapon_collision_policy::reconstructWeaponRoot(live, _createdCenterWeaponLocal, _createdWeaponScale);
        if (!dynamic_weapon_collision_policy::isFiniteTransform(pose)) return false;
        // The reusable no-contact anchor must not coast on its last keyframe
        // velocity while no constraint owns it.
        if (!placeGeneratedKeyframedBodyImmediately(_authorityProxy,
                dynamic_weapon_collision_policy::makeGripAuthorityTarget(pose, _authorityPivotWeaponLocal))) return false;
        const char* stage = "unchanged";
        // A released object follows the native physics clock. Undo only ROCK's
        // held/VATS velocity multiplier; retain the solver's linear and angular momentum.
        if (!rebaseWeaponVelocity(frame.hknpWorld, _body,
                weapon_physics_time_scale::velocityRebase(_handlingScale.velocity, 1.0f), stage)) {
            ROCK_LOG_WARN(Weapon, "Equipped drop refused: momentum stage={}", stage);
            return false;
        }
        // The wrapper path maps motion-cinfo's default FFFF to profile 2
        // (141565041..1415650B9), regardless of the body's dynamic flags.
        // Profile 1 differs from 2 by the gravity multiplier: 141841DEB sets
        // it to 1; 141841DFF..141841E03 clears it for 2. Keep held behavior
        // intact and lease the native gravity profile only for free flight.
        std::uint16_t heldProfile = equipped_drop_motion_profile::kInvalid;
        if (!havok_runtime::tryReadBodyMotionPropertiesId(frame.hknpWorld, _body.getBodyId(), heldProfile) ||
            !_equippedDropMotionProfile.acquire(heldProfile, [&](std::uint16_t target) {
                return setOwnedWeaponMotionProfile(frame.hknpWorld, _body, target, stage);
            })) {
            ROCK_LOG_WARN(Weapon, "Equipped drop refused: gravity stage={} heldProfile={}", stage, heldProfile);
            // A failed readback may follow a native write. Remove this owned
            // body rather than returning an uncertain profile to the held path.
            retireProxyLocked(frame.bhkWorld);
            return false;
        }
        destroyGrabConstraint(frame.hknpWorld, _authorityConstraint);
        _body.setCollisionFilterInfo(dynamicWeaponProxyFilterInfo(true), kRebuildBodyCollisionState);
        _body.activateBody();
        _equippedDrop = true;
        resetEquippedDropRestWatch(true);
        _equippedDropGravitySamplePending = true;
        _handlingScale = {};
        _surfaceSupport = {};
        _surfaceContacts.clear();
        _surfaceInputReserved = false;
        _droveThisSubstep = false;
        _surfaceClickRequested = false;
        _surfaceToggle = {};
        _physicsRequestedTargetValid = false;
        _physicsDriveTeleported = false;
        _divergenceDwellSeconds = 0.0f;
        clearPublishedPhysicsSnapshot();
        ROCK_LOG_INFO(Weapon, "Equipped drop physics released: body={} generation={:016X} motionProfile={}->{} position=({:.3f},{:.3f},{:.3f})",
            _body.getBodyId().value, generation, heldProfile, equipped_drop_motion_profile::kGravityEnabled,
            pose.translate.x, pose.translate.y, pose.translate.z);
        return true;
    }

    bool DynamicWeaponCollisionRuntime::restoreEquippedHold(const PhysicsFrameContext& frame, const char* reason)
    {
        if (!_equippedDrop) return true;
        auto mutation = _physicsCallbackGate ? _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        const auto heldProfile = _equippedDropMotionProfile.saved();
        const char* stage = "motion-profile-restore";
        if (_createdWorld != frame.hknpWorld || _createdBhkWorld != frame.bhkWorld ||
            !_equippedDropMotionProfile.restore([&](std::uint16_t target) {
                return setOwnedWeaponMotionProfile(frame.hknpWorld, _body, target, stage);
            }) || !setAuthorityPivot(frame, {}, _frameRequestedWeaponWorld)) {
            ROCK_LOG_WARN(Weapon, "Equipped drop held physics restore failed: body={} profile={} stage={} reason={}; retiring owned body",
                _body.getBodyId().value, heldProfile, stage, reason);
            retireProxyLocked(frame.bhkWorld);
            return false;
        }
        _body.setCollisionFilterInfo(dynamicWeaponProxyFilterInfo(), kRebuildBodyCollisionState);
        _body.activateBody();
        _equippedDrop = false;
        resetEquippedDropRestWatch(false);
        cancelEquippedDropPull();
        _equippedDropGravitySamplePending = false;
        ROCK_LOG_INFO(Weapon, "Equipped drop held physics restored: body={} generation={:016X} restoredMotionProfile={} reason={}",
            _body.getBodyId().value, _createdGenerationKey, heldProfile, reason);
        return true;
    }

    bool DynamicWeaponCollisionRuntime::tryGetEquippedDropPose(RE::NiNode* weapon,
        const std::uint64_t generation, RE::NiTransform& pose) const
    {
        if (!_equippedDrop || !_created || weapon != _frameWeaponNode ||
            generation == 0 || generation != _createdGenerationKey) return false;
        PhysicsSnapshot snapshot{};
        if (readPhysicsSnapshot(snapshot) && snapshot.valid &&
            snapshot.world == reinterpret_cast<std::uintptr_t>(_createdWorld) &&
            snapshot.bodyId == _body.getBodyId().value && snapshot.generationKey == generation) {
            const auto sampled = dynamic_weapon_collision_policy::reconstructWeaponRoot(
                snapshot.liveProxyBodyWorld, _createdCenterWeaponLocal, _createdWeaponScale);
            if (!dynamic_weapon_collision_policy::isFiniteTransform(sampled)) return false;
            pose = sampled;
        } else {
            // The game-thread owner retains the last presented pose. A missing
            // sample must never send a moving weapon back to its release point.
            return false;
        }
        return true;
    }

    bool DynamicWeaponCollisionRuntime::startEquippedDropPull(const PhysicsFrameContext& frame,
        const RE::NiPoint3& gripWeaponLocal, const RE::NiPoint3& handWorld)
    {
        if (!_equippedDrop || _equippedDropPull.active || !_created || _createdWorld != frame.hknpWorld ||
            !dynamic_weapon_collision_policy::isFinitePoint(gripWeaponLocal) ||
            !dynamic_weapon_collision_policy::isFinitePoint(handWorld)) return false;
        RE::NiTransform pose{};
        if (!tryGetEquippedDropPose(_frameWeaponNode, _createdGenerationKey, pose)) return false;
        const auto grip = transform_math::localPointToWorld(pose, gripWeaponLocal);
        const float distance = (handWorld - grip).Length() * physics_scale::gameToHavok();
        if (!std::isfinite(distance)) return false;
        _equippedDropPull = { .active = true, .gripWeaponLocal = gripWeaponLocal,
            .duration = pull_motion_math::computePullDurationSeconds(distance) };
        _equippedDropRest.reset();
        ROCK_LOG_INFO(Weapon, "Equipped drop pull started body={} generation={:016X} distanceMeters={:.3f}",
            _body.getBodyId().value, _createdGenerationKey, distance);
        return true;
    }

    void DynamicWeaponCollisionRuntime::cancelEquippedDropPull() { _equippedDropPull = {}; }

    DynamicWeaponCollisionRuntime::DropPullProgress DynamicWeaponCollisionRuntime::updateEquippedDropPull(
        const PhysicsFrameContext& frame, const RE::NiPoint3& handWorld)
    {
        if (!_equippedDropPull.active || !_equippedDrop || !_created || _createdWorld != frame.hknpWorld ||
            !dynamic_weapon_collision_policy::isFinitePoint(handWorld)) {
            cancelEquippedDropPull();
            return DropPullProgress::Unavailable;
        }
        auto mutation = _physicsCallbackGate ? _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        RE::NiTransform bodyWorld{};
        const auto body = havok_runtime::snapshotBody(frame.hknpWorld, _body.getBodyId());
        if (!body.valid || !body.motion || body.collisionObject != _body.getCollisionObject() ||
            !havok_runtime::tryResolveLiveBodyWorldTransform(frame.hknpWorld, _body.getBodyId(), bodyWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(bodyWorld)) {
            ROCK_LOG_WARN(Weapon, "Equipped drop pull cancelled: body unavailable");
            cancelEquippedDropPull();
            return DropPullProgress::Unavailable;
        }
        const auto pose = dynamic_weapon_collision_policy::reconstructWeaponRoot(bodyWorld, _createdCenterWeaponLocal, _createdWeaponScale);
        const auto grip = transform_math::localPointToWorld(pose, _equippedDropPull.gripWeaponLocal);
        const float scale = physics_scale::gameToHavok();
        if (!dynamic_weapon_collision_policy::isFinitePoint(grip) || !std::isfinite(scale) || scale <= 0.0f) {
            ROCK_LOG_WARN(Weapon, "Equipped drop pull cancelled: invalid grip or physics scale");
            cancelEquippedDropPull();
            return DropPullProgress::Unavailable;
        }
        _equippedDropPull.elapsed += std::isfinite(frame.deltaSeconds) ? (std::max)(0.0f, frame.deltaSeconds) : 0.0f;
        const auto motion = pull_motion_math::computePullMotion(pull_motion_math::PullMotionInput<RE::NiPoint3>{
            .handHavok = handWorld * scale, .objectPointHavok = grip * scale,
            .previousTargetHavok = _equippedDropPull.previousTargetHavok,
            .elapsedSeconds = _equippedDropPull.elapsed, .durationSeconds = _equippedDropPull.duration,
            .hasPreviousTarget = _equippedDropPull.hasTarget });
        if (motion.expired) {
            ROCK_LOG_INFO(Weapon, "Equipped drop pull expired body={} distance={:.3f}", _body.getBodyId().value, (handWorld - grip).Length());
            cancelEquippedDropPull();
            return DropPullProgress::Expired;
        }
        if ((handWorld - grip).Length() <= pull_motion_math::kAutoGrabDistanceGameUnits) return DropPullProgress::Arrived;
        _equippedDropPull.previousTargetHavok = motion.targetHavok;
        _equippedDropPull.hasTarget = true;
        if (motion.applyVelocity) {
            const auto q = body.motion->orientation;
            const auto w = body.motion->angularVelocity;
            const float quaternion[4]{q.x, q.y, q.z, q.w};
            RE::NiPoint3 angular{};
            if (!weapon_physics_time_scale::rebaseWorldAngularVelocity(quaternion, {w.x, w.y, w.z},
                    pull_motion_math::angularVelocityKeepForDamping(frame.deltaSeconds), angular)) {
                ROCK_LOG_WARN(Weapon, "Equipped drop pull cancelled: invalid angular motion");
                cancelEquippedDropPull();
                return DropPullProgress::Unavailable;
            }
            const float linear[4]{motion.velocityHavok.x, motion.velocityHavok.y, motion.velocityHavok.z, 0};
            const float omega[4]{angular.x, angular.y, angular.z, 0};
            if (!havok_runtime::isFinite3(linear) || !havok_runtime::isFinite3(omega) || !_body.setVelocity(linear, omega)) {
                ROCK_LOG_WARN(Weapon, "Equipped drop pull cancelled: native velocity write rejected");
                cancelEquippedDropPull();
                return DropPullProgress::Unavailable;
            }
        }
        return DropPullProgress::Flying;
    }

    DynamicWeaponCollisionRuntime::FrameResult DynamicWeaponCollisionRuntime::finishFrame(
        const PhysicsFrameContext& frame,
        const bool physicsWritesAllowed,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const WeaponCollision& weaponCollision,
        const RE::NiPoint3* primaryGripWeaponLocal, const bool handsFree, const bool equippedDrop)
    {
        FrameResult result{};
        result.requestedWeaponWorld = _frameRequestedWeaponWorld;
        result.resolvedWeaponWorld = _frameRequestedWeaponWorld;
        _compoundSourcesUnavailable = false;

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
                retireAll(frame.bhkWorld, frame.menuBlocked);
            }
            _debugSnapshot = {};
            return result;
        }

        result.proxyActive = true;
        if (_equippedDrop) {
            if (equippedDrop) {
                updateEquippedDropRest(frame);
                _debugSnapshot = {}; // A free body has no hand target or held-contact residual.
                // Free motion is absolute, including frames without contact.
                // Never apply a hand-relative residual or a grip recovery here.
                result.applyVisualCorrection = tryGetEquippedDropPose(weaponNode, weaponGenerationKey, result.resolvedWeaponWorld);
                return result;
            }
            if (!restoreEquippedHold(frame, "regrab")) {
                result.proxyActive = false;
                return result;
            }
        }
        updateSurfaceSupport(frame, primaryGripWeaponLocal, handsFree);
        if (!_authorityConstraint.isValid() || _rebuildRequestedAtomic.load(std::memory_order_acquire)) return result;
        if (!native_interaction_service::updateWeapon(frame, weaponCollision, weaponNode, _body,
                _createdCenterWeaponLocal, weaponGenerationKey,
                _frameRequestedWeaponWorld, _surfaceSupport.ownsPose(), primaryGripWeaponLocal)) {
            retireAll(frame.bhkWorld);
            return result;
        }
        result.surfaceSupportOwnsPose = _surfaceSupport.ownsPose();
        result.requestedWeaponWorld = _frameRequestedWeaponWorld;
        result.resolvedWeaponWorld = _frameRequestedWeaponWorld;
        // Support still publishes when resting contact has no fresh residual,
        // or while the rebound pivot awaits its first post-solve sample.
        result.applyVisualCorrection = result.surfaceSupportOwnsPose;
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
        const bool debugEnabled = dynamicWeaponDebugEnabled();

        _debugSnapshot = {};
        if (debugEnabled) {
            _debugSnapshot.valid = true;
            _debugSnapshot.physicsSnapshotReadable = snapshotReadable;
            _debugSnapshot.physicsSnapshotValid =
                snapshotReadable && snapshot.valid;
            _debugSnapshot.physicsSnapshotIdentityCurrent =
                snapshotIdentityCurrent;
            _debugSnapshot.physicsSnapshotContactActive =
                snapshotReadable && snapshot.contactActive;
            _debugSnapshot.physicsSnapshotTeleported =
                snapshotReadable && snapshot.teleported;
            _debugSnapshot.bodyId = _body.getBodyId().value;
            _debugSnapshot.authorityBodyId = _authorityProxy.getBodyId().value;
            _debugSnapshot.constraintId = _authorityConstraint.constraintId;
            _debugSnapshot.generationKey = _createdGenerationKey;
            _debugSnapshot.proxyPairCallbackSequence =
                _proxyPairCallbackSequenceAtomic.load(
                    std::memory_order_acquire);
            _debugSnapshot.obstacleCallbackSequence =
                _obstacleCallbackSequenceAtomic.load(
                    std::memory_order_acquire);
            _debugSnapshot.rawPointCallbackSequence =
                _rawPointCallbackSequenceAtomic.load(
                    std::memory_order_acquire);
            _debugSnapshot.processedManifoldCallbackSequence =
                _processedManifoldCallbackSequenceAtomic.load(
                    std::memory_order_acquire);
            _debugSnapshot.admittedContactSequence =
                _contactSequenceAtomic.load(std::memory_order_acquire);
            _debugSnapshot.compoundChildCount = _createdCompoundChildCount;
            _debugSnapshot.compoundPointCount = _createdCompoundPointCount;
            _debugSnapshot.centerWeaponLocal = _createdCenterWeaponLocal;
            _debugSnapshot.halfExtentsWeaponLocal =
                _createdHalfExtentsWeaponLocal;
            _debugSnapshot.requestedWeaponWorld = _frameRequestedWeaponWorld;
        }

        ContactDiagnosticSnapshot contactDiagnostic{};
        const bool contactDiagnosticCurrent =
            debugEnabled &&
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
            if (!debugEnabled) {
                return;
            }
            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                500,
                "DWC pipeline: stage={} contactBody={} authorityBody={} constraint={} callbacks(pair/obstacle/raw/manifold/admit)={}/{}/{}/{}/{} snapshot(read/valid/identity/contact/teleport)={}/{}/{}/{}/{} gripPivotError={:.2f}gu angularYield={:.2f}deg visual={}",
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
        const RE::NiTransform resolvedWeaponWorld = native_interaction_service::weaponControlled() ? native_interaction_service::weaponPresentation() :
            dynamic_weapon_collision_policy::resolveCurrentIntentFromSample(
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

        const auto correction = dynamic_weapon_collision_policy::evaluateVisualCorrection(_frameRequestedWeaponWorld, resolvedWeaponWorld);
        result.translationCorrectionGameUnits = correction.translationGameUnits;
        result.rotationCorrectionDegrees = correction.rotationDegrees;
        if (debugEnabled) {
            _debugSnapshot.contactActive = true;
            _debugSnapshot.otherBodyId = snapshot.otherBodyId;
            _debugSnapshot.otherLayer = snapshot.otherLayer;
            _debugSnapshot.contactRetentionSeconds = snapshot.contactRetentionSeconds;
            _debugSnapshot.solveSequence = snapshot.solveSequence;
            _debugSnapshot.liveWeaponWorld = sampledLiveWeaponWorld;
            _debugSnapshot.resolvedWeaponWorld = resolvedWeaponWorld;
            _debugSnapshot.translationCorrectionGameUnits =
                result.translationCorrectionGameUnits;
            _debugSnapshot.rotationCorrectionDegrees =
                result.rotationCorrectionDegrees;
        }

        if (!correction.apply) {
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

        // Even a zero residual stays on the same presentation/hand authority.
        // A perceptual cutoff must not switch the next frame's input ownership.
        result.applyVisualCorrection = correction.apply;
        result.resolvedWeaponWorld = resolvedWeaponWorld;
        if (_surfaceSupport.ownsPose()) _surfaceSupport.lastWorld = resolvedWeaponWorld;
        if (debugEnabled) {
            _debugSnapshot.visualCorrectionActive = true;
        }
        logPipelineStage("publish-requested");

        return result;
    }

    void DynamicWeaponCollisionRuntime::finalizeCompoundPose(const WeaponCollision& weaponCollision,
        RE::NiNode* weaponNode, const PhysicsFrameContext& frame, std::uint64_t generation)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponCompoundPoseUpdate);
        const bool hasIntent = std::exchange(_frameHasIntent, false);
        if (!_created || !hasIntent || _frameIndex != frame.timing.sequence || _createdWorld != frame.hknpWorld ||
            _frameWeaponNode != weaponNode || _createdGenerationKey != generation) return;
        // Only articulation is read back. The free-space root intent remains
        // independent of the collision-corrected rendered weapon.
        if (!queueCompoundChildTransforms(weaponCollision, weaponNode, std::abs(_frameRequestedWeaponWorld.scale))) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "Dynamic weapon final compound pose rejected: generation={:016X} children={}",
                generation, _createdCompoundChildCount);
            return;
        }
        if (_equippedDrop) return; // Articulation follows the mesh; the root belongs only to Havok.
        const auto target = dynamic_weapon_collision_policy::makeGripAuthorityTarget(_frameRequestedWeaponWorld, _authorityPivotWeaponLocal);
        const auto queued = queueGeneratedKeyframedBodyTarget(_authorityDriveState, target, frame.deltaSeconds,
            dynamic_weapon_collision_policy::kDivergenceTeleportDistanceGameUnits);
        if (!queued.queued) _rebuildRequestedAtomic.store(true, std::memory_order_release);
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

        if (_compoundPoseScratch.size() != _createdCompoundChildCount ||
            _preparedCompoundChildTransforms.size() != _createdCompoundChildCount) {
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
                    _preparedCompoundChildTransforms[i])) {
                return false;
            }
        }
        std::scoped_lock poseLock(_compoundPoseMutex);
        if (_pendingCompoundChildTransforms == _preparedCompoundChildTransforms) return true;
        std::copy(_preparedCompoundChildTransforms.begin(), _preparedCompoundChildTransforms.end(),
            _pendingCompoundChildTransforms.begin());
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
        constexpr float inertiaEnvelopePadding =
            dynamic_weapon_collision_policy::kInertiaEnvelopePaddingGameUnits;
        const bool bodyMatches =
            _created &&
            _body.isValid() &&
            _authorityProxy.isValid() &&
            (_equippedDrop || _authorityConstraint.isValid()) &&
            static_cast<bool>(_compoundShape) &&
            _createdWorld == frame.hknpWorld &&
            _createdBhkWorld == frame.bhkWorld &&
            _createdGenerationKey == bounds.generationKey &&
            std::abs(_createdWeaponScale - scale) <= 0.0001f &&
            !_rebuildRequestedAtomic.load(std::memory_order_acquire);
        if (bodyMatches) {
            return true;
        }
        // A free body cannot be silently rebuilt at a hand target. The owner
        // retires the retained item if this physical representation is lost.
        if (_equippedDrop) return false;

        const auto weaponIdentity = weaponCollision.getEquippedWeaponClassification();
        if (!weaponIdentity.hasEquippedWeapon) {
            return false;
        }

        WeaponCollision::CompoundGeometrySnapshot compoundGeometry{};
        if (!weaponCollision.getCompoundGeometrySnapshot(compoundGeometry) ||
            !compoundGeometry.valid ||
            compoundGeometry.generationKey != bounds.generationKey ||
            compoundGeometry.generationKey != _frameGenerationKey) {
            _compoundSourcesUnavailable =
                compoundGeometry.failure == WeaponCollision::CompoundGeometrySnapshotFailure::SourceTransformUnavailable &&
                compoundGeometry.generationKey == _frameGenerationKey;
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

        std::size_t taggedCollisionSoundChildren = 0;
        std::uint32_t compoundCollisionSoundMaterialId = 0;
        bool collisionSoundMaterialUniform = !compoundChildren.empty();
        for (std::size_t childIndex = 0;
             childIndex < compoundChildren.size();
             ++childIndex) {
            const auto* childShape = compoundChildren[childIndex].shape;
            const auto materialId = childShape ?
                static_cast<std::uint32_t>(childShape->userData) :
                0u;
            if (materialId != 0) {
                ++taggedCollisionSoundChildren;
            }
            if (childIndex == 0) {
                compoundCollisionSoundMaterialId = materialId;
            } else if (materialId != compoundCollisionSoundMaterialId) {
                collisionSoundMaterialUniform = false;
            }
        }
        if (!collisionSoundMaterialUniform ||
            compoundCollisionSoundMaterialId == 0) {
            compoundCollisionSoundMaterialId = 0;
        } else if (auto* pendingShape = pendingCompoundShape.get()) {
            // Contact shape keys preserve per-child material IDs. The
            // top-level tag is the native fallback for contacts that do not
            // publish a leaf key, and is valid only when every child agrees.
            pendingShape->userData = static_cast<std::uintptr_t>(
                compoundCollisionSoundMaterialId);
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
        const bool contactEventFlagsEnabled = havok_runtime::enableBodyFlags(
            frame.hknpWorld,
            proxyBodyId.value,
            kRequiredContactEventFlags,
            kRebuildBodyCollisionState);
        const auto flaggedBody = havok_runtime::snapshotBody(frame.hknpWorld, proxyBodyId);
        const bool contactEventFlagsPublished =
            contactEventFlagsEnabled &&
            flaggedBody.valid &&
            flaggedBody.body &&
            (flaggedBody.body->flags & kRequiredContactEventFlags) == kRequiredContactEventFlags;
        if (!contactEventFlagsPublished) {
            ROCK_LOG_ERROR(
                Weapon,
                "Dynamic weapon compound {} failed manifold/impact event opt-in: enabled={} readable={} flags=0x{:08X}",
                proxyBodyId.value,
                contactEventFlagsEnabled,
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
        _createdCompoundChildCount = static_cast<std::uint32_t>(compoundGeometry.children.size());
        _authorityPivotWeaponLocal = {};
        _createdCompoundPointCount = compoundGeometry.sourcePointCount;
        _created = true;
        {
            std::scoped_lock poseLock(_compoundPoseMutex);
            _compoundPoseScratch.resize(compoundGeometry.children.size());
            _preparedCompoundChildTransforms.resize(compoundGeometry.children.size());
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
        _createdMass = bodyMass;
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
            "Dynamic weapon grip-constrained compound created: contactBody={} authorityBody={} constraint={} generation={:016X} children={} points={} soundMaterial=0x{:08X} taggedChildren={}/{} authority=grip center=({:.2f},{:.2f},{:.2f}) envelopeHalf=({:.2f},{:.2f},{:.2f}) scale={:.3f} mass={:.2f} forces=({:.1f},{:.1f}) inertiaPadding={:.2f} layer={}",
            _body.getBodyId().value,
            _authorityProxy.getBodyId().value,
            _authorityConstraint.constraintId,
            _createdGenerationKey,
            _createdCompoundChildCount,
            _createdCompoundPointCount,
            compoundCollisionSoundMaterialId,
            taggedCollisionSoundChildren,
            compoundChildren.size(),
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
            inertiaEnvelopePadding,
            collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY);
        return true;
    }

    void DynamicWeaponCollisionRuntime::flushPendingPhysicsDrive(
        RE::hknpWorld* world,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponDynamicDrive);
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world ||
            !_body.isValid() || !_authorityProxy.isValid() || (!_equippedDrop && !_authorityConstraint.isValid())) {
            return;
        }

        if (_equippedDrop) {
            (void)flushCompoundChildTransforms();
            return; // No motors, hand transport, velocity caps or grip recovery in free flight.
        }

        const auto timeScale = weapon_physics_time_scale::resolve(g_rockConfig.rockVatsPhysicsFixes, timing);
        const auto motorTuning = buildWeaponGripConstraintTuning(_createdMass);
        if (!timeScale.valid ||
            !std::isfinite(motorTuning.linearMaxForce * timeScale.force) ||
            !std::isfinite(motorTuning.angularMaxForce * timeScale.force)) {
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "VATS_PHYSICS drive skipped: body={} multiplier={} physicsDt={} fallback={} stage=timing-or-budget",
                _body.getBodyId().value, timing.timeMultiplier, timing.substepDeltaSeconds, timing.usedFallback);
            return;
        }
        const float velocityRatio = weapon_physics_time_scale::velocityRebase(_handlingScale.velocity, timeScale.velocity);
        const char* rebaseStage = "unchanged";
        if (!rebaseWeaponVelocity(world, _body, velocityRatio, rebaseStage)) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "VATS_PHYSICS velocity rebase failed: body={} ratio={} stage={}",
                _body.getBodyId().value, velocityRatio, rebaseStage);
            return;
        }
        _handlingScale = timeScale;

        if (!native_interaction_service::preCollideWeapon(world, timing)) {
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            return;
        }

        if (!flushCompoundChildTransforms()) return;

        const auto driveResult = driveGeneratedKeyframedBody(
            world,
            _authorityProxy,
            _authorityDriveState,
            timing,
            "DynamicWeaponGripAuthority",
            0,
            dynamic_weapon_collision_policy::kMaximumLinearVelocityHavok * timeScale.velocity,
            dynamic_weapon_collision_policy::kMaximumAngularVelocityRadiansPerSecond * timeScale.velocity);
        if (driveResult.shouldRequestRebuild()) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            return;
        }

        // Use the actual constraint pivot and live COM properties. The shared
        // free/contact budget is converted to the weapon's solver clock once.
        const RE::NiPoint3 pivotBodyLocalGame{
            (_authorityPivotWeaponLocal.x - _createdCenterWeaponLocal.x) * _createdWeaponScale,
            (_authorityPivotWeaponLocal.y - _createdCenterWeaponLocal.y) * _createdWeaponScale,
            (_authorityPivotWeaponLocal.z - _createdCenterWeaponLocal.z) * _createdWeaponScale,
        };
        _authorityConstraint.motorBodyProperties = readGrabMotorBodyProperties(world, _body.getBodyId(), pivotBodyLocalGame);
        if (!updateWeaponGripConstraintContactResponse(
            _authorityConstraint,
            motorTuning,
            _authorityConstraint.motorBodyProperties,
            _contactRetentionSeconds > 0.0f || native_interaction_service::weaponControlled(),
            timeScale)) {
            if (_authorityConstraint.linearMotor)
                _authorityConstraint.linearMotor->minForce = _authorityConstraint.linearMotor->maxForce = 0.0f;
            if (_authorityConstraint.angularMotor)
                _authorityConstraint.angularMotor->minForce = _authorityConstraint.angularMotor->maxForce = 0.0f;
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Dynamic weapon motor properties unavailable; rebuild requested body={}",
                _body.getBodyId().value);
            return;
        }

        _droveThisSubstep =
            driveResult.driven &&
            driveResult.hasRequestedTargetGameTransform;
        _physicsRequestedTargetValid = driveResult.hasRequestedTargetGameTransform;
        _physicsSourceSequence = driveResult.sourceSequence;
        bool contactBodyRecovered = false;
        if (_physicsRequestedTargetValid) {
            _physicsRequestedAuthorityTarget = driveResult.requestedTargetGameTransform;
            _physicsRequestedTarget = dynamic_weapon_collision_policy::makeContactBodyTargetFromGripAuthority(
                driveResult.requestedTargetGameTransform,
                _createdCenterWeaponLocal,
                _createdWeaponScale, _authorityPivotWeaponLocal);

            RE::NiTransform liveContactBodyWorld{};
            const bool hasLiveContactBody =
                havok_runtime::tryResolveLiveBodyWorldTransform(
                    world,
                    _body.getBodyId(),
                    liveContactBodyWorld) &&
                dynamic_weapon_collision_policy::isFiniteTransform(liveContactBodyWorld);
            const float requestedGapGameUnits = hasLiveContactBody ?
                dynamic_weapon_collision_policy::translationDeltaGameUnits(
                    liveContactBodyWorld,
                    _physicsRequestedTarget) :
                0.0f;
            const auto gripRecovery = hasLiveContactBody && !native_interaction_service::weaponControlled() ?
                dynamic_weapon_collision_policy::evaluateGripRecovery(
                    liveContactBodyWorld,
                    _physicsRequestedAuthorityTarget,
                    _createdCenterWeaponLocal,
                    _createdWeaponScale,
                    _gripRecoveryDistanceGameUnitsAtomic.load(
                        std::memory_order_acquire), _authorityPivotWeaponLocal) :
                dynamic_weapon_collision_policy::GripRecoveryDecision{};
            if (gripRecovery.resetNow) {
                // This is an independent catastrophic-failure boundary. It
                // does not wait for normal contact convergence or its dwell.
                _divergenceDwellSeconds = 0.0f;
                contactBodyRecovered = placeGeneratedKeyframedBodyImmediately(
                    _body,
                    _physicsRequestedTarget);
                if (contactBodyRecovered) {
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        1000,
                        "Dynamic weapon contact body reset after catastrophic grip separation: body={} gripGap={:.2f} threshold={:.2f}",
                        _body.getBodyId().value,
                        gripRecovery.distanceGameUnits,
                        _gripRecoveryDistanceGameUnitsAtomic.load(
                            std::memory_order_relaxed));
                } else {
                    _rebuildRequestedAtomic.store(true, std::memory_order_release);
                    _droveThisSubstep = false;
                    clearPublishedPhysicsSnapshot();
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        1000,
                        "Dynamic weapon catastrophic grip reset failed; requesting collider rebuild: body={} gripGap={:.2f} threshold={:.2f}",
                        _body.getBodyId().value,
                        gripRecovery.distanceGameUnits,
                        _gripRecoveryDistanceGameUnitsAtomic.load(
                            std::memory_order_relaxed));
                    return;
                }
            } else if (!native_interaction_service::weaponControlled()) {
                const auto dwell = dynamic_weapon_collision_policy::advanceDivergenceDwell(
                    _divergenceDwellSeconds,
                    requestedGapGameUnits,
                    timeScale.responseDeltaSeconds);
                _divergenceDwellSeconds = dwell.elapsedSeconds;
                if (dwell.recoverNow) {
                    // Bound recovery attempts even if the engine rejects a body
                    // placement. Persistent divergence must accrue a new dwell
                    // interval before another attempt.
                    _divergenceDwellSeconds = 0.0f;
                    contactBodyRecovered = placeGeneratedKeyframedBodyImmediately(
                        _body,
                        _physicsRequestedTarget);
                    if (contactBodyRecovered) {
                        ROCK_LOG_SAMPLE_WARN(
                            Weapon,
                            1000,
                            "Dynamic weapon contact body recovered after persistent divergence: body={} gap={:.2f} threshold={:.2f} dwell={:.3f}s",
                            _body.getBodyId().value,
                            requestedGapGameUnits,
                            dynamic_weapon_collision_policy::kDivergenceTeleportDistanceGameUnits,
                            dynamic_weapon_collision_policy::kDivergenceTeleportDwellSeconds);
                    }
                }
            }
        } else {
            _divergenceDwellSeconds = 0.0f;
        }
        _physicsDriveTeleported = driveResult.teleported || contactBodyRecovered;
        if (_physicsDriveTeleported) {
            _contactRetentionSeconds = 0.0f;
        }
    }

    bool DynamicWeaponCollisionRuntime::flushCompoundChildTransforms()
    {
        std::scoped_lock poseLock(_compoundPoseMutex);
        if (_queuedCompoundPoseSequence == _consumedCompoundPoseSequence) return true;
        const auto result = _compoundShape.updateTransforms(_pendingCompoundChildTransforms);
        if (!result.succeeded) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _droveThisSubstep = false;
            clearPublishedPhysicsSnapshot();
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Dynamic weapon live compound update failed: generation={:016X} children={}",
                _createdGenerationKey, _createdCompoundChildCount);
            return false;
        }
        _consumedCompoundPoseSequence = _queuedCompoundPoseSequence;
        if (result.changedChildCount > 0 && dynamicWeaponDebugEnabled()) {
            ROCK_LOG_SAMPLE_INFO(Weapon, 500, "Dynamic weapon live compound updated: body={} generation={:016X} changed={}/{}",
                _body.getBodyId().value, _createdGenerationKey, result.changedChildCount, _createdCompoundChildCount);
        }
        return true;
    }

    void DynamicWeaponCollisionRuntime::samplePostSolve(RE::hknpWorld* world, const std::uint64_t solveSequence,
        const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!_enabledAtomic.load(std::memory_order_acquire) || !world || !_created || _createdWorld != world ||
            !_body.isValid() || (!_equippedDrop && (!_droveThisSubstep || !_physicsRequestedTargetValid))) {
            return;
        }
        _droveThisSubstep = false;

        RE::NiTransform liveBodyWorld{};
        if (!havok_runtime::tryResolveLiveBodyWorldTransform(world, _body.getBodyId(), liveBodyWorld) ||
            !dynamic_weapon_collision_policy::isFiniteTransform(liveBodyWorld)) {
            if (_equippedDrop) _rebuildRequestedAtomic.store(true, std::memory_order_release);
            _contactRetentionSeconds = 0.0f;
            clearPublishedPhysicsSnapshot();
            return;
        }
        if (_equippedDrop) {
            if (_equippedDropGravitySamplePending) {
                _equippedDropGravitySamplePending = false;
                const auto body = havok_runtime::snapshotBody(world, _body.getBodyId());
                std::uint16_t profile = equipped_drop_motion_profile::kInvalid;
                const bool readable = body.valid && body.motion && body.collisionObject == _body.getCollisionObject() &&
                    havok_runtime::tryReadMotionPropertiesId(body.motion, profile);
                if (readable) {
                    ROCK_LOG_INFO(Weapon, "Equipped drop first solve: body={} motionProfile={} z={:.3f} velocityZ={:.5f} physicsDt={:.6f}",
                        _body.getBodyId().value, profile, liveBodyWorld.translate.z,
                        body.motion->linearVelocity.z, timing.substepDeltaSeconds);
                } else {
                    ROCK_LOG_WARN(Weapon, "Equipped drop first solve: motion unavailable body={}", _body.getBodyId().value);
                }
            }
            PhysicsSnapshot snapshot{};
            snapshot.valid = true;
            snapshot.world = reinterpret_cast<std::uintptr_t>(world);
            snapshot.bodyId = _body.getBodyId().value;
            snapshot.generationKey = _createdGenerationKey;
            snapshot.solveSequence = solveSequence;
            snapshot.weaponScale = _createdWeaponScale;
            snapshot.requestedProxyBodyWorld = liveBodyWorld;
            snapshot.liveProxyBodyWorld = liveBodyWorld;
            publishPhysicsSnapshot(snapshot);
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
                collision_layer_policy::isDynamicWeaponProxyObstacleLayer(otherLayer);
        }
        const bool contactWasActive = _contactRetentionSeconds > 0.0f;
        const bool contactEpisodeStarted =
            newMatchingContact &&
            (!contactWasActive || _activeContactOtherBodyId != otherBodyId);
        if (contactEpisodeStarted) {
            ++_contactEpisode;
            _activeContactOtherBodyId = otherBodyId;
        }
        _contactRetentionSeconds = dynamic_weapon_collision_policy::advanceContactRetention(
            _contactRetentionSeconds, newMatchingContact, _physicsDriveTeleported, timing, _handlingScale.velocity);

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
        snapshot.sourceSequence = _physicsSourceSequence;
        snapshot.weaponScale = _createdWeaponScale;
        snapshot.requestedProxyBodyWorld = _physicsRequestedTarget;
        snapshot.liveProxyBodyWorld = liveBodyWorld;
        publishPhysicsSnapshot(snapshot);

        if (contactEpisodeStarted &&
            dynamicWeaponDebugEnabled()) {
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

        _physicsDriveTeleported = false;
    }

    void DynamicWeaponCollisionRuntime::refreshCollisionFilter(RE::hknpWorld* world)
    {
        auto mutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        if (_created && _createdWorld == world && !_body.refreshCollisionFilter(world)) {
            _rebuildRequestedAtomic.store(true, std::memory_order_release);
            ROCK_LOG_WARN(Weapon, "Dynamic weapon collision filter refresh failed: body={}",
                _body.getBodyId().value);
        }
    }

    void DynamicWeaponCollisionRuntime::resetEquippedDropRestWatch(const bool active)
    {
        _dropRestWorldAtomic.store(nullptr, std::memory_order_release);
        _dropRestBodyAtomic.store(kInvalidBodyId, std::memory_order_release);
        _dropRestEpochAtomic.fetch_add(1, std::memory_order_acq_rel);
        _equippedDropRest.reset();
        if (!_dropRestSupportBusy.test_and_set(std::memory_order_acquire)) {
            _dropRestSupport = {};
            _dropRestSupportBusy.clear(std::memory_order_release);
        }
        if (active) {
            _dropRestBodyAtomic.store(_body.getBodyId().value, std::memory_order_release);
            _dropRestWorldAtomic.store(_createdWorld, std::memory_order_release);
        }
    }

    void DynamicWeaponCollisionRuntime::recordEquippedDropRestContact(RE::hknpWorld* world,
        const std::uint32_t a, const std::uint32_t b, const RE::NiPoint3& pointGame, const RE::NiPoint3& normal) noexcept
    {
        const auto epoch = _dropRestEpochAtomic.load(std::memory_order_acquire);
        if (!world || world != _dropRestWorldAtomic.load(std::memory_order_acquire) ||
            !std::isfinite(normal.z) || std::abs(normal.z) < 0.5f) return;
        const bool mineA = watchesEquippedDropRest(a), mineB = watchesEquippedDropRest(b);
        if (mineA == mineB) return;
        DropRestSupport witness{};
        witness.world = reinterpret_cast<std::uintptr_t>(world);
        witness.generation = epoch;
        witness.weaponBody = mineA ? a : b;
        witness.surfaceBody = mineA ? b : a;
        const auto surface = havok_runtime::snapshotBody(world, RE::hknpBodyId{witness.surfaceBody});
        if (!weapon_rest_support::parkingSurface(surface)) return;
        witness.surfaceCollision = reinterpret_cast<std::uintptr_t>(surface.collisionObject);
        float comX{}, comY{}, comZ{};
        if (!havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{witness.weaponBody}, witness.weaponPose) ||
            !havok_runtime::tryGetBodyArrayWorldTransform(world, RE::hknpBodyId{witness.surfaceBody}, witness.surfacePose) ||
            !havok_runtime::getBodyCOMWorld(world, RE::hknpBodyId{witness.weaponBody}, comX, comY, comZ)) return;
        if (!std::isfinite(pointGame.z) || !std::isfinite(comZ) || pointGame.z > comZ * physics_scale::havokToGame() + 1.0f) return;
        witness.valid = true;
        if (!_dropRestSupportBusy.test_and_set(std::memory_order_acquire)) {
            if (epoch == _dropRestEpochAtomic.load(std::memory_order_acquire) &&
                world == _dropRestWorldAtomic.load(std::memory_order_acquire) && watchesEquippedDropRest(witness.weaponBody))
                _dropRestSupport = witness;
            _dropRestSupportBusy.clear(std::memory_order_release);
        }
    }

    void DynamicWeaponCollisionRuntime::updateEquippedDropRest(const PhysicsFrameContext& frame)
    {
        if (!_equippedDrop || _equippedDropPull.active || !_created || !frame.worldReady ||
            frame.menuBlocked || frame.hknpWorld != _createdWorld || !_body.isValid()) {
            _equippedDropRest.reset();
            return;
        }
        DropRestSupport support{};
        if (!_dropRestSupportBusy.test_and_set(std::memory_order_acquire)) {
            support = _dropRestSupport;
            _dropRestSupportBusy.clear(std::memory_order_release);
        }
        RE::NiTransform pose{};
        bool validPose = false, supported = false;
        {
            havok_world_lock::ScopedWorldReadLock lock(frame.hknpWorld);
            const auto body = havok_runtime::snapshotBody(frame.hknpWorld, _body.getBodyId());
            validPose = body.valid && body.body && body.motion && body.collisionObject == _body.getCollisionObject() &&
                havok_runtime::tryGetBodyArrayWorldTransform(frame.hknpWorld, _body.getBodyId(), pose);
            if (validPose && support.valid && support.world == reinterpret_cast<std::uintptr_t>(frame.hknpWorld) &&
                support.generation == _dropRestEpochAtomic.load(std::memory_order_acquire) && support.weaponBody == _body.getBodyId().value) {
                const auto surface = havok_runtime::snapshotBody(frame.hknpWorld, RE::hknpBodyId{support.surfaceBody});
                RE::NiTransform surfacePose{};
                // Retain sleeping contacts only while BOTH witnessed poses and
                // the surface identity remain current, exactly as loose weapons.
                supported = weapon_rest_support::parkingSurface(surface) &&
                    reinterpret_cast<std::uintptr_t>(surface.collisionObject) == support.surfaceCollision &&
                    havok_runtime::tryGetBodyArrayWorldTransform(frame.hknpWorld, RE::hknpBodyId{support.surfaceBody}, surfacePose) &&
                    loose_weapon_collision_policy::poseDelta(pose, support.weaponPose, physics_scale::gameToHavok()).withinRestTolerance() &&
                    weapon_rest_support::sameSurfacePose(surfacePose, support.surfacePose);
            }
        }
        const bool previouslyReady = _equippedDropRest.ready();
        const bool ready = _equippedDropRest.update(false, supported, validPose ? &pose : nullptr,
            physics_scale::gameToHavok(), frame.deltaSeconds);
        if (ready && !previouslyReady) {
            ROCK_LOG_INFO(Weapon, "Equipped drop resting: body={} generation={:016X} supportedRest={:.3f}",
                _body.getBodyId().value, _createdGenerationKey, _equippedDropRest.seconds());
        }
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
        if (!world || _equippedDrop || !isProxyBodyIdAtomic(proxyBodyId)) {
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
        if (dynamicWeaponDebugEnabled()) {
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
        const RE::NiPoint3* contactPointGame, const RE::NiPoint3& targetNormalGame, std::uint32_t pointCount)
    {
        if (!world || _equippedDrop || !isProxyBodyIdAtomic(proxyBodyId)) {
            return;
        }
        _proxyPairCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (!otherLayerRead || !collision_layer_policy::isDynamicWeaponProxySolverObstacleLayer(otherLayer)) {
            return;
        }
        _obstacleCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        _processedManifoldCallbackSequenceAtomic.fetch_add(1, std::memory_order_release);
        if (contactPointGame && collision_layer_policy::isWorldSurfaceLayer(otherLayer)) {
            recordSurfaceSupportContact(world, otherBodyId, *contactPointGame);
        }
        if (contactPointGame) {
            native_interaction_service::weaponContact(world, proxyBodyId, otherBodyId, otherLayer, *contactPointGame, targetNormalGame, pointCount);
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
            native_interaction_service::retireWeapon(_createdWorld, bhkWorld);
            destroyGrabConstraint(_createdWorld, _authorityConstraint);
            if (_body.isValid()) {
                _body.retireDeferred(bhkWorld);
            }
            if (_authorityProxy.isValid()) {
                _authorityProxy.retireDeferred(bhkWorld);
            }
        } else {
            native_interaction_service::retireWeapon(nullptr, nullptr);
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
        _equippedDrop = false;
        resetEquippedDropRestWatch(false);
        _equippedDropMotionProfile.reset();
        cancelEquippedDropPull();
        _equippedDropGravitySamplePending = false;
        _surfaceContacts.clear();
        _authorityPivotWeaponLocal = {};
        {
            std::scoped_lock poseLock(_compoundPoseMutex);
            _compoundPoseScratch.clear();
            _preparedCompoundChildTransforms.clear();
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
        _createdMass = 0.0f;
        _handlingScale = {};
        _createdCompoundChildCount = 0;
        _createdCompoundPointCount = 0;
        _created = false;
        _droveThisSubstep = false;
        _physicsRequestedTargetValid = false;
        _physicsDriveTeleported = false;
        _physicsRequestedAuthorityTarget = {};
        _physicsRequestedTarget = {};
        _physicsSourceSequence = 0;
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

    void DynamicWeaponCollisionRuntime::retireAll(void* bhkWorld, const bool preserveSurfaceSupport)
    {
        _surfaceInputReserved = false;
        auto structuralMutation = _physicsCallbackGate ?
            _physicsCallbackGate->pauseForMutation() :
            PhysicsCallbackQuiescenceGate::MutationLease{};
        retireProxyLocked(bhkWorld);
        if (!preserveSurfaceSupport) _surfaceSupport = {};
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
        native_interaction_service::retireWeapon(nullptr, nullptr);
        destroyGrabConstraint(nullptr, _authorityConstraint);
        _body.reset();
        _authorityProxy.reset();
        clearLocalProxyStateLocked();
        _surfaceSupport = {};
        _enabledAtomic.store(false, std::memory_order_release);
        _surfaceInputReserved = false;
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

    bool DynamicWeaponCollisionRuntime::tryGetContactState(RE::NiNode* weapon, std::uint64_t generation, bool& contact) const
    {
        contact = false;
        if (!_created || !weapon || weapon != _frameWeaponNode || generation == 0 ||
            generation != _frameGenerationKey || generation != _createdGenerationKey) return false;
        PhysicsSnapshot snapshot{};
        if (!readPhysicsSnapshot(snapshot) || !snapshot.valid || snapshot.teleported ||
            snapshot.world != reinterpret_cast<std::uintptr_t>(_frameWorld) ||
            snapshot.bodyId != _body.getBodyId().value || snapshot.generationKey != generation) return false;
        contact = snapshot.contactActive;
        return true;
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
        _snapshotSourceSequenceAtomic.store(snapshot.sourceSequence, std::memory_order_relaxed);
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
            candidate.sourceSequence = _snapshotSourceSequenceAtomic.load(std::memory_order_relaxed);
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

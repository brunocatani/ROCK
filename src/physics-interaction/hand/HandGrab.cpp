#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"
#include "physics-interaction/hand/HandGrabInternal.h"
#include "api/ProviderRuntimeServices.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HeldScenePresentation.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/weapon/WeaponSupport.h"
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
#include "physics-interaction/consume/ImmersiveAid.h"
#include "physics-interaction/consume/ImmersiveAidPose.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabLocalGraspSolve.h"
#include "physics-interaction/grab/GrabPalmSeat.h"
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
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
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

#include <chrono>
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
#include <utility>
#include <xmmintrin.h>

namespace rock
{
    namespace
    {
        hand_visual_lerp_math::VisualReturnConfig looseWeaponAcquireConfig()
        {
            return {
                .minSeconds = g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance,
                .minAngleDegrees = hand_visual_lerp_math::kEquippedWeaponReturnConfig.minAngleDegrees,
                .maxAngleDegrees = hand_visual_lerp_math::kEquippedWeaponReturnConfig.maxAngleDegrees,
            };
        }
        template <class Rollback>
        class GrabPreparationTransaction
        {
        public:
            explicit GrabPreparationTransaction(Rollback rollback) : _rollback(std::move(rollback)) {}

            GrabPreparationTransaction(const GrabPreparationTransaction&) = delete;
            GrabPreparationTransaction& operator=(const GrabPreparationTransaction&) = delete;
            GrabPreparationTransaction(GrabPreparationTransaction&&) = delete;
            GrabPreparationTransaction& operator=(GrabPreparationTransaction&&) = delete;

            ~GrabPreparationTransaction() noexcept
            {
                rollback();
            }

            void rollback() noexcept
            {
                if (_active) {
                    _active = false;
                    _rollback();
                }
            }

            void commit() noexcept
            {
                _active = false;
            }

        private:
            Rollback _rollback;
            bool _active = true;
        };

        struct GrabRollbackAction
        {
            void* context = nullptr;
            void (*invoke)(void*) noexcept = nullptr;

            void rollback() const noexcept
            {
                if (context && invoke) {
                    invoke(context);
                }
            }
        };

        static_assert(kGrabCollisionSuppressionArmBodyCountPerHand == kBodyBoneGrabSuppressionArmBodyCountPerSide,
            "Normal grab arm-collider suppression capacity must match the body collider arm-chain query.");

        std::uint64_t nextGrabTraceId() noexcept
        {
            static std::atomic<std::uint64_t> nextTraceId{ 1 };
            return nextTraceId.fetch_add(1, std::memory_order_relaxed);
        }

        const char* releaseDispositionName(GrabReleaseDisposition disposition) noexcept
        {
            switch (disposition) {
            case GrabReleaseDisposition::PhysicalDrop:
                return "physical-drop";
            case GrabReleaseDisposition::PendingInventoryTransfer:
                return "pending-inventory-transfer";
            case GrabReleaseDisposition::TransferToInventory:
                return "transfer-to-inventory";
            case GrabReleaseDisposition::PendingConsumeTransfer:
                return "pending-consume-transfer";
            case GrabReleaseDisposition::OwnershipHandoff:
                return "ownership-handoff";
            }
            return "unknown";
        }

        RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lengthSquared <= 1.0e-8f) {
                return RE::NiPoint3{};
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            return RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
        }

        float lengthSquared(const RE::NiPoint3& value)
        {
            return vector_math::lengthSquared(value);
        }

        RE::NiPoint3 crossProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return vector_math::cross(lhs, rhs);
        }

        float dotProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return vector_math::dot(lhs, rhs);
        }

        RE::NiPoint3 scalePoint(const RE::NiPoint3& value, float scalar)
        {
            return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
        }

        RE::NiPoint3 stablePerpendicularAxis(const RE::NiPoint3& normal)
        {
            const RE::NiPoint3 normalizedNormal = normalizeOrZero(normal);
            if (lengthSquared(normalizedNormal) <= 0.000001f) {
                return {};
            }

            const RE::NiPoint3 reference = std::fabs(normalizedNormal.z) < 0.75f ? RE::NiPoint3{ 0.0f, 0.0f, 1.0f } : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
            RE::NiPoint3 axis = normalizeOrZero(crossProduct(reference, normalizedNormal));
            if (lengthSquared(axis) <= 0.000001f) {
                axis = normalizeOrZero(crossProduct(RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, normalizedNormal));
            }
            return axis;
        }

        RE::NiPoint3 gamePointToHavokPoint(const RE::NiPoint3& value)
        {
            const float scale = physics_scale::gameToHavok();
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        RE::NiMatrix3 storedRotationFromConventionalRows(const RE::NiPoint3 rows[3])
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = rows[0].x;
            result.entry[0][1] = rows[1].x;
            result.entry[0][2] = rows[2].x;
            result.entry[1][0] = rows[0].y;
            result.entry[1][1] = rows[1].y;
            result.entry[1][2] = rows[2].y;
            result.entry[2][0] = rows[0].z;
            result.entry[2][1] = rows[1].z;
            result.entry[2][2] = rows[2].z;
            return result;
        }

        RE::NiMatrix3 axisAngleStored(const RE::NiPoint3& axisRaw, float angle)
        {
            const RE::NiPoint3 axis = normalizeOrZero(axisRaw);
            if (lengthSquared(axis) <= 0.000001f || !std::isfinite(angle)) {
                return transform_math::makeIdentityRotation<RE::NiMatrix3>();
            }

            const float x = axis.x;
            const float y = axis.y;
            const float z = axis.z;
            const float cosTheta = std::cos(angle);
            const float sinTheta = std::sin(angle);
            const float oneMinusCos = 1.0f - cosTheta;

            const RE::NiPoint3 rows[3]{
                RE::NiPoint3{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
                RE::NiPoint3{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
                RE::NiPoint3{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
            };
            return storedRotationFromConventionalRows(rows);
        }

        RE::NiMatrix3 applyWorldRotationToStoredBasis(const RE::NiMatrix3& worldRotationStored, const RE::NiMatrix3& baseRotation)
        {
            RE::NiMatrix3 result{};
            for (int axis = 0; axis < 3; ++axis) {
                const RE::NiPoint3 basis{ baseRotation.entry[axis][0], baseRotation.entry[axis][1], baseRotation.entry[axis][2] };
                const RE::NiPoint3 rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis);
                result.entry[axis][0] = rotated.x;
                result.entry[axis][1] = rotated.y;
                result.entry[axis][2] = rotated.z;
            }
            return result;
        }

        RE::NiPoint3 angularVelocityFromRotationDelta(const RE::NiMatrix3& previous, const RE::NiMatrix3& current, float deltaTime)
        {
            return held_object_physics_math::angularVelocityFromRotationDelta<RE::NiMatrix3, RE::NiPoint3>(previous, current, deltaTime);
        }

        RE::NiPoint3 rotationCorrectionAxisWorld(const RE::NiMatrix3& current, const RE::NiMatrix3& target)
        {
            return normalizeOrZero(angularVelocityFromRotationDelta(current, target, 1.0f));
        }

        bool computeHardKeyframeVelocityForTarget(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            const RE::NiTransform& targetWorld,
            float deltaTime,
            float outLinearVelocityHavok[4],
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
                !outLinearVelocityHavok || !outAngularVelocityRadians ||
                !havok_runtime::getBody(world, bodyId)) {
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
            using ComputeHardKeyFrame_t = void (*)(RE::hknpWorld*, RE::hknpBodyId, float*, float*, float, float*, float*);
            static REL::Relocation<ComputeHardKeyFrame_t> compute{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            compute(world, bodyId, targetPositionHavok, targetRotationHavok, deltaTime, linearVelocityHavok, angularVelocityRadians);

            havok_runtime::copyVector4(linearVelocityHavok, outLinearVelocityHavok);
            havok_runtime::copyVector4(angularVelocityRadians, outAngularVelocityRadians);
            outLinearVelocityHavok[3] = 0.0f;
            outAngularVelocityRadians[3] = 0.0f;
            return havok_runtime::isFinite3(outLinearVelocityHavok) && havok_runtime::isFinite3(outAngularVelocityRadians);
        }

        const char* nodeDebugName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "(null)";
            }

            const char* name = node->name.c_str();
            return name ? name : "(unnamed)";
        }

        const char* primaryBodyChoiceReasonName(object_physics_body_set::PrimaryBodyChoiceReason reason)
        {
            using object_physics_body_set::PrimaryBodyChoiceReason;
            switch (reason) {
            case PrimaryBodyChoiceReason::PreferredHitAccepted:
                return "preferredHitAccepted";
            case PrimaryBodyChoiceReason::SurfaceOwnerAccepted:
                return "surfaceOwnerAccepted";
            case PrimaryBodyChoiceReason::NearestAcceptedFallback:
                return "nearestAcceptedFallback";
            case PrimaryBodyChoiceReason::NoAcceptedBody:
                return "noAcceptedBody";
            case PrimaryBodyChoiceReason::None:
            default:
                return "none";
            }
        }

        float pointDistanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const RE::NiPoint3 delta = a - b;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        float looseWeaponMultiplier(bool looseWeaponGrab, float multiplier)
        {
            return looseWeaponGrab ? (std::isfinite(multiplier) ? multiplier : 1.0f) : 1.0f;
        }

        float scaleDriveValue(float value, float multiplier)
        {
            return (std::isfinite(value) ? value : 0.0f) * (std::isfinite(multiplier) ? multiplier : 1.0f);
        }

        float vectorMagnitude(const RE::NiPoint3& value)
        {
            return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
        }

        RE::NiPoint3 clampAngularVelocityVector(const RE::NiPoint3& value, float maxRadiansPerSecond)
        {
            if (!std::isfinite(maxRadiansPerSecond) || maxRadiansPerSecond <= 0.0f) {
                return RE::NiPoint3{};
            }

            const float magnitude = vectorMagnitude(value);
            if (!std::isfinite(magnitude) || magnitude <= 0.000001f) {
                return RE::NiPoint3{};
            }
            if (magnitude <= maxRadiansPerSecond) {
                return value;
            }

            const float scale = maxRadiansPerSecond / magnitude;
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        float sharedGrabAuthorityForceScale(bool peerHandStillHolding)
        {
            /*
             * Two ROCK proxy constraints on one loose object must share the
             * finite force budget instead of each hand receiving a full HIGGS-
             * style mass-capped motor. The selected grip points remain
             * independent; only the total per-object authority is budgeted.
             */
            return peerHandStillHolding ? 0.5f : 1.0f;
        }

        std::uint32_t bodySetRejectCount(
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            physics_body_classifier::BodyRejectReason reason)
        {
            const auto index = static_cast<std::size_t>(reason);
            if (index >= bodySet.diagnostics.rejectCounts.size()) {
                return 0;
            }
            return bodySet.diagnostics.rejectCounts[index];
        }

        const char* bodyMotionTypeName(physics_body_classifier::BodyMotionType motionType)
        {
            using physics_body_classifier::BodyMotionType;
            switch (motionType) {
            case BodyMotionType::Static:
                return "Static";
            case BodyMotionType::Dynamic:
                return "Dynamic";
            case BodyMotionType::Keyframed:
                return "Keyframed";
            case BodyMotionType::Other:
                return "Other";
            case BodyMotionType::Unknown:
            default:
                return "Unknown";
            }
        }

        const object_physics_body_set::ObjectPhysicsBodyRecord* diagnosticRejectedBodyRecord(
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            std::uint32_t preferredBodyId)
        {
            if (const auto* preferred = bodySet.findRecord(preferredBodyId); preferred && !preferred->accepted) {
                return preferred;
            }
            for (const auto& record : bodySet.records) {
                if (!record.accepted) {
                    return &record;
                }
            }
            return nullptr;
        }

        held_object_drive_policy::HeldBodySetDriveDecision classifyHeldBodySetDrive(
            const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
            const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet,
            bool incompleteNativeScan)
        {
            const auto uniqueMotionCount = preparedBodySet.forEachUniqueAcceptedMotion([](const auto&) {});
            return held_object_drive_policy::evaluateHeldBodySetDrive(held_object_drive_policy::HeldBodySetDriveInput{
                .acceptedBodyCount = static_cast<std::uint32_t>(preparedBodySet.acceptedCount()),
                .uniqueMotionCount = static_cast<std::uint32_t>(uniqueMotionCount),
                .rejectedFixedOrNonDynamicCount =
                    bodySetRejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::StaticMotion) +
                    bodySetRejectCount(preparedBodySet, physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep),
                .scanFailureCount = beforePrepBodySet.diagnostics.scanFailures + preparedBodySet.diagnostics.scanFailures,
                .invalidPhysicsSystemCount = beforePrepBodySet.diagnostics.invalidPhysicsSystems + preparedBodySet.diagnostics.invalidPhysicsSystems,
                .incompleteNativeScan = incompleteNativeScan,
            });
        }

        template <std::size_t N>
        float recordDeviationAverage(std::array<float, N>& history, std::size_t& count, std::size_t& next, float sample)
        {
            if constexpr (N == 0) {
                return std::isfinite(sample) ? sample : 0.0f;
            } else {
                const float sanitizedSample = std::isfinite(sample) && sample > 0.0f ? sample : 0.0f;
                history[next] = sanitizedSample;
                next = (next + 1) % N;
                if (count < N) {
                    ++count;
                }

                float total = 0.0f;
                for (std::size_t i = 0; i < count; ++i) {
                    total += history[i];
                }
                return count > 0 ? total / static_cast<float>(count) : sanitizedSample;
            }
        }

        float effectiveGrabMotorMass(float mass)
        {
            return grab_motion_controller::effectiveMotorMass(
                mass,
                g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
                g_rockConfig.rockGrabEffectiveMotorMassFloor);
        }

        GrabConstraintMotorTuning buildProxyConstraintMotorTuning(
            float tau,
            float damping,
            float maxForce,
            float authorityForceScale,
            float proportionalRecovery,
            float constantRecovery,
            bool looseWeaponGrab,
            float mass,
            float forceToMassRatio)
        {
            const float linearTauMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier);
            const float angularTauMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier);
            const float linearDampingMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier);
            const float angularDampingMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier);
            const float maxForceMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier);
            const float angularForceMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier);
            const float linearRecoveryMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
            const float angularRecoveryMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);

            const float linearBudget = (std::max)(0.0f, scaleDriveValue(maxForce, maxForceMultiplier));
            const float sanitizedAuthorityForceScale =
                std::clamp(std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f, 0.05f, 1.0f);
            const float linearMaxForce =
                grab_motion_controller::capForceByMass(linearBudget, mass, forceToMassRatio) * sanitizedAuthorityForceScale;
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

        bool sharedContextMatchesSelection(const GrabSharedObjectContext& sharedContext, const SelectedObject& selection)
        {
            if (!sharedContext.hasPeerState() || !selection.refr || sharedContext.peerSavedObjectState->refr != selection.refr) return false;
            if (!grab_target::isRagdoll(selection.targetKind)) return true;
            auto* cell = selection.refr->GetParentCell();
            auto* world = cell ? havok_runtime::getHknpWorldFromBhk(cell->GetbhkWorld()) : nullptr;
            const auto component = ragdoll::readComponent(world, selection.bodyId.value);
            return component.valid && component.contains(sharedContext.peerSavedObjectState->bodyId.value);
        }

        bool isLooseWeaponGrabTarget(const SelectedObject& selection)
        {
            if (!selection.refr || !grab_target::canUseRockActiveGrab(selection.targetKind)) {
                return false;
            }

            auto* selectedBase = selection.refr->GetObjectReference();
            return selectedBase && selectedBase->Is(RE::ENUM_FORM_ID::kWEAP);
        }

        bool isFiniteNiTransform(const RE::NiTransform& value)
        {
            bool rotationFinite = true;
            for (std::uint32_t row = 0; row < 3; ++row) {
                for (std::uint32_t column = 0; column < 3; ++column) {
                    rotationFinite = rotationFinite && std::isfinite(value.rotate.entry[row][column]);
                }
            }
            return rotationFinite &&
                   std::isfinite(value.translate.x) &&
                   std::isfinite(value.translate.y) &&
                   std::isfinite(value.translate.z) &&
                   std::isfinite(value.scale) &&
                   value.scale > 0.0001f;
        }

        bool nodeIsOrDescendsFrom(const RE::NiAVObject* root, const RE::NiAVObject* node);
        RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child);
        RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld, const RE::NiTransform& bodyLocalTransform);

        const RE::TESObjectWEAP* looseWeaponFormFromRef(RE::TESObjectREFR* refr)
        {
            auto* selectedBase = refr ? refr->GetObjectReference() : nullptr;
            return selectedBase ? selectedBase->As<RE::TESObjectWEAP>() : nullptr;
        }

        const RE::TESObjectWEAP* selectedLooseWeaponForm(const SelectedObject& selection)
        {
            return looseWeaponFormFromRef(selection.refr);
        }

        bool isThrowableLooseWeapon(const RE::TESObjectWEAP* weapon)
        {
            if (!weapon) {
                return false;
            }
            /*
             * WEAPON_TYPE is stored as a single enum value in FO4VR. Keep this
             * as direct equality instead of EnumSet::any so guns cannot alias
             * thrown types through bit-style tests.
             */
            return weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                   weapon->weaponData.type == RE::WEAPON_TYPE::kMine;
        }

        bool publishLooseWeaponPrimaryAttachHandPose(bool isLeft, RE::TESObjectREFR* refr,
            const AuthoredWeaponGripPose& pose)
        {
            const auto* weapon = looseWeaponFormFromRef(refr);
            if (!pose.valid() || !weapon || pose.isLeft != isLeft || pose.weaponFormId != weapon->formID) return false;
            frik_visual_authority::FingerLocalTransformOverride exact{};
            exact.enabledMask = pose.fingerMask;
            std::copy(pose.fingerLocals.begin(), pose.fingerLocals.end(), std::begin(exact.localTransforms));
            constexpr const char* tag = "ROCK_Grab";
            constexpr int priority = 100;
            const char* blockTag = isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight";
            const bool blocked = frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, true);
            const bool scalar = blocked && frik_visual_authority::setHandPoseCustom(
                tag, handFromBool(isLeft), frik_visual_authority::HandPoseData{}, priority);
            if (scalar && frik_visual_authority::setHandPoseCustomLocalTransforms(tag, handFromBool(isLeft), &exact, priority)) {
                ROCK_LOG_INFO(Hand, "{} loose authored pose published role={} form={:08X}",
                    isLeft ? "left" : "right", static_cast<unsigned>(pose.role), pose.weaponFormId);
                return true;
            }
            (void)frik_visual_authority::clearHandPose(tag, handFromBool(isLeft));
            if (blocked) (void)frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, false);
            return false;
        }
        struct LooseWeaponPrimaryAttachFrame
        {
            bool valid = false;
            bool supportGrip = false;
            AuthoredWeaponGripPose pose{};
            RE::NiTransform desiredRootWorld{};
            bool sourceVisible = false;
            RE::NiTransform desiredObjectWorld{};
            RE::NiTransform desiredBodyWorld{};
            RE::NiPoint3 gripPointWorld{};
            const char* reason = "notEvaluated";
        };

        // Cache-only disk lookup; false when this object+hand has no saved
        // offset (normal - most objects never had one saved).
        bool tryLoadSavedGrabOffsetHandOffset(RE::TESObjectREFR* refr, bool isLeft, saved_grab_offset::HandOffset& out)
        {
            if (!refr) {
                return false;
            }
            auto* baseForm = refr->GetObjectReference();
            if (!baseForm) {
                return false;
            }
            const auto formRef = saved_grab_offset::formRefFromRuntimeId(baseForm->GetFormID());
            saved_grab_offset::SavedGrabOffsetFile file{};
            if (!formRef.empty() && saved_grab_offset::load(formRef, file, nullptr)) {
                const auto& handOffset = isLeft ? file.left : file.right;
                if (handOffset.present) {
                    if (handOffset.hasExactFingerRotations && fo4vr::isInPowerArmor()) {
                        ROCK_LOG_WARN(Hand, "{} saved exact grab pose skipped: normal-hand rig does not match power armor",
                            isLeft ? "Left" : "Right");
                        return false;
                    }
                    out = handOffset;
                    return true;
                }
            }
            return saved_grab_offset::loadDefaultGrabPose(baseForm->GetFormID(), isLeft, out);
        }

        /*
         * proxyWorld/proxyWorldValid are resolved by the caller (a live
         * GrabAuthorityProxy read, Hand::tryComputeGrabProxyLocalPalmPocketFrameWorld)
         * since this file's helpers are free functions with no Hand access.
         */
        bool tryResolveSavedGrabOffsetAttach(
            const RE::NiTransform& proxyWorld,
            bool proxyWorldValid,
            const saved_grab_offset::HandOffset& handOffset,
            RE::NiTransform& desiredRootWorld)
        {
            if (!proxyWorldValid) {
                return false;
            }

            RE::NiTransform objectProxyLocal = transform_math::makeIdentityTransform<RE::NiTransform>();
            objectProxyLocal.translate = { handOffset.translateGame[0], handOffset.translateGame[1], handOffset.translateGame[2] };
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    objectProxyLocal.rotate.entry[row][column] = handOffset.rotate[row * 3 + column];
                }
            }

            desiredRootWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, objectProxyLocal);
            return isFiniteNiTransform(desiredRootWorld);
        }

        /*
         * A resolved offset's finger pose is only meaningful for the loose-
         * weapon synthetic primary attach (pull-catch/force-grab): that path
         * has no mesh contact to solve fingers from, so it otherwise falls
         * back to a generic canned named pose (see
         * publishLooseWeaponPrimaryAttachHandPose / grabSelectedObject).
         */
        bool tryBuildSavedGrabOffsetFingerPose(
            const saved_grab_offset::HandOffset& handOffset,
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose)
        {
            if (!handOffset.hasFingerPose) {
                return false;
            }

            pose.solved = true;
            pose.values = { handOffset.fingerValues[0], handOffset.fingerValues[1], handOffset.fingerValues[2],
                handOffset.fingerValues[3], handOffset.fingerValues[4] };
            pose.hasJointValues = handOffset.hasFingerJointValues;
            if (handOffset.hasFingerJointValues) {
                for (std::size_t i = 0; i < pose.jointValues.size(); ++i) {
                    pose.jointValues[i] = handOffset.fingerJointValues[i];
                }
            }
            return true;
        }

        LooseWeaponPrimaryAttachFrame resolveLooseWeaponPrimaryAttachFrame(
            bool looseWeaponGrab,
            bool grabbedFromPullCatch,
            bool isLeft,
            const SelectedObject& selection,
            const RE::NiAVObject* rootNode,
            const RE::NiTransform& rootBodyLocalAtGrab,
            const RE::NiTransform& objectToBodyAtGrab,
            const RE::NiTransform& grabBodyWorldAtGrab,
            const RE::NiPoint3& grabPivotAWorld,
            bool savedGrabOffsetAttachValid,
            const RE::NiTransform& savedGrabOffsetRootWorld,
            const loose_weapon_grip_zone::NearGrab* nearGrip)
        {
            LooseWeaponPrimaryAttachFrame frame{};
            // The acquisition policy has already selected exact authored
            // placement or a real touching mesh grip.
            const bool nearAuthored = looseWeaponGrab && nearGrip && nearGrip->pose.valid();
            if (!nearAuthored && !savedGrabOffsetAttachValid) {
                frame.reason = "dynamicTouchHold";
                return frame;
            }

            if (savedGrabOffsetAttachValid) {
                /*
                 * A per-object saved offset is explicit hand-placement
                 * authority, so it overrides both the generic FRIK weapon
                 * offset and the throwable live-pose default.
                 */
                frame.desiredRootWorld = savedGrabOffsetRootWorld;
                frame.sourceVisible = false;
                frame.reason = "savedGrabOffset";
            } else {
                if (!looseWeaponGrab) {
                    frame.reason = "notLooseWeapon";
                    return frame;
                }
                /*
                 * Non-throwable weapons use the selected authored relation.
                 * Grenades, mines, and Molotov variants are
                 * hand-thrown objects: force-grab and pull-catch commits keep the
                 * normal mesh/body relation so the object is translated into the
                 * pocket without forcing a root rotation from FRIK or the live hand.
                 */
                const auto* looseWeapon = selectedLooseWeaponForm(selection);
                const bool throwableArrival = isThrowableLooseWeapon(looseWeapon) && (grabbedFromPullCatch || selection.forcedArrival);
                if (throwableArrival) {
                    frame.reason = selection.forcedArrival ? "throwableForcedArrivalPreservePose" : "throwablePullCatchPreservePose";
                    return frame;
                }
                if (!rootNode || !isFiniteNiTransform(rootNode->world)) {
                    frame.reason = "missingWeaponRoot";
                    return frame;
                }

                frame.pose = nearGrip->pose;
                frame.supportGrip = frame.pose.role == loose_weapon_authored_grab_policy::Role::Support;
                frame.reason = frame.supportGrip ? "authoredSupportGrip" : "authoredFiringGrip";
            }

            frame.desiredRootWorld.scale =
                rootNode && std::isfinite(rootNode->world.scale) && rootNode->world.scale > 0.0001f ? rootNode->world.scale : 1.0f;
            if (nearAuthored) {
                const auto palmInHand = authoredHandspaceToRawHandspaceForHand(
                    computeGrabLegacyPalmPivotAHandspacePosition(isLeft), isLeft);
                frame.desiredRootWorld = nearGrip->pose.placementWorld(
                    nearGrip->handWorld, frame.desiredRootWorld.scale, palmInHand);
                const auto proxyDelta = grabPivotAWorld -
                    transform_math::localPointToWorld(nearGrip->handWorld, palmInHand);
                ROCK_LOG_DEBUG(Hand, "{} authored loose placement uses equipped palm form={:08X} proxyDelta=({:.4f},{:.4f},{:.4f})gu",
                    isLeft ? "left" : "right", nearGrip->pose.weaponFormId, proxyDelta.x, proxyDelta.y, proxyDelta.z);
            }
            if (!isFiniteNiTransform(frame.desiredRootWorld)) {
                frame.reason = "nonFiniteDesiredRoot";
                return frame;
            }

            frame.desiredBodyWorld = multiplyTransforms(frame.desiredRootWorld, rootBodyLocalAtGrab);
            frame.desiredObjectWorld = deriveNodeWorldFromBodyWorld(frame.desiredBodyWorld, objectToBodyAtGrab);
            // Preserve the generated proxy's physical pivot. Its weapon/body
            // anchor is derived AFTER placement, so it cannot offset the hand
            // and weapon from the equipped palm seat (0.86gu in the capture).
            const RE::NiPoint3 desiredPivotBodyLocal = transform_math::worldPointToLocal(frame.desiredBodyWorld, grabPivotAWorld);
            frame.gripPointWorld = transform_math::localPointToWorld(grabBodyWorldAtGrab, desiredPivotBodyLocal);
            if (!isFiniteNiTransform(frame.desiredBodyWorld) ||
                !isFiniteNiTransform(frame.desiredObjectWorld) ||
                !std::isfinite(frame.gripPointWorld.x) ||
                !std::isfinite(frame.gripPointWorld.y) ||
                !std::isfinite(frame.gripPointWorld.z)) {
                frame.reason = "nonFiniteDesiredBody";
                return frame;
            }

            frame.valid = true;
            return frame;
        }

        bool nodeIsOrDescendsFrom(const RE::NiAVObject* root, const RE::NiAVObject* node)
        {
            if (!root || !node) {
                return false;
            }

            for (unsigned depth = 0; node && depth < 64; ++depth, node = node->parent) {
                if (node == root) {
                    return true;
                }
            }
            return false;
        }

        bool acceptsSelectedOwnerlessVisualMesh(const SelectedObject& selection,
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            std::uint32_t resolvedBodyId,
            RE::NiAVObject* surfaceOwnerNode,
            const object_physics_body_set::ObjectPhysicsBodyRecord* surfaceOwnerRecord)
        {
            if (!selection.refr || selection.refr != bodySet.rootRef ||
                resolvedBodyId == object_physics_body_set::INVALID_BODY_ID ||
                resolvedBodyId != selection.bodyId.value || !surfaceOwnerNode ||
                surfaceOwnerRecord || !nodeIsOrDescendsFrom(bodySet.rootNode, surfaceOwnerNode)) {
                return false;
            }
            const auto* selectedBody = bodySet.findRecord(resolvedBodyId);
            if (!selectedBody || !selectedBody->accepted) return false;

            // Preserve the established selected-body authority for multipart refs.
            if (bodySet.acceptedCount() > 1) return true;

            // Single-body modular weapons can place the handguard/magazine beside
            // the receiver's collision branch. A seeded body is valid evidence
            // even when its native system cannot be enumerated (benignScanSkips),
            // but the seed alone must not stand in for verified reference ownership.
            if (selection.targetKind != grab_target::Kind::LooseObject ||
                !looseWeaponFormFromRef(selection.refr) ||
                selection.refr->Get3D() != bodySet.rootNode || bodySet.records.size() != 1 ||
                !selectedBody->refResolutionKnown || selectedBody->resolvedRef != selection.refr ||
                bodySet.diagnostics.scanFailures || bodySet.diagnostics.invalidPhysicsSystems ||
                bodySet.diagnostics.depthLimitSkips || bodySet.diagnostics.foreignRefBodySkips ||
                bodySet.diagnostics.unresolvedRefBodySkips) {
                return false;
            }
            return grab_contact_source_policy::isSingleBodyOwnerlessVisualBranch(
                bodySet.rootNode, selectedBody->owningNode, surfaceOwnerNode);
        }

        constexpr const char* kHeldObjectDriveName = "proxyConstraint";
        constexpr std::uint32_t heldCollisionParticipationFlags(grab_target::Kind targetKind)
        {
            // Key-2 manifold events (0x40, also used by hands/bipod) report a
            // loose prop's sustained surface contact even without an impact.
            // Other grab kinds keep their existing notification contract.
            return 0x80u | (targetKind == grab_target::Kind::LooseObject ? 0x40u : 0u);
        }
        constexpr std::uint32_t kHeldCollisionParticipationFlagMode = 0u;
        constexpr std::uint32_t kHeldAuthorityBodyFlags = 0x08000000u;
        constexpr std::uint32_t kHeldAuthorityBodyFlagMode = 1u;

        std::uintptr_t heldBodyFlagLeaseOwner(const Hand* hand)
        {
            return reinterpret_cast<std::uintptr_t>(hand) ^ 0x524F434B48454C44ull;
        }

        struct HeldBodyActivationSummary
        {
            std::uint32_t bodyCount = 0;
            std::uint32_t activatedCount = 0;
            std::uint32_t failedActivationCount = 0;
        };

        struct HeldBodyFlagLeaseSummary
        {
            std::uint32_t bodyCount = 0;
            std::uint32_t collisionLeaseCount = 0;
            std::uint32_t authorityLeaseCount = 0;
            std::uint32_t failedLeaseCount = 0;
        };

        HeldBodyActivationSummary activateHeldObjectBodySet(
            RE::hknpWorld* world,
            std::uint32_t primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds)
        {
            HeldBodyActivationSummary summary{};
            if (!world) {
                return summary;
            }

            const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
            summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
            for (const auto bodyId : bodyIds) {
                if (physics_recursive_wrappers::activateBody(world, bodyId)) {
                    ++summary.activatedCount;
                } else {
                    ++summary.failedActivationCount;
                }
            }
            return summary;
        }

        HeldBodyFlagLeaseSummary acquireHeldObjectBodyFlagLeases(
            RE::hknpWorld* world,
            std::uint32_t primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            grab_target::Kind targetKind,
            std::uintptr_t ownerToken)
        {
            /*
             * Proxy-constraint grab replaced the native held-object action, but the
             * old path owned two different flag contracts: collision flags were
             * leased across the accepted held body set by ROCK, while the native action leased
             * 0x08000000 only on its selected primary body. Keeping that split is
             * important for multipart weapons because secondary collision bodies
             * should participate in the hold without all becoming grab-authority
             * bodies.
             */
            HeldBodyFlagLeaseSummary summary{};
            if (!world || ownerToken == 0) {
                return summary;
            }

            const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
            summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
            for (const auto bodyId : bodyIds) {
                if (havok_runtime::acquireBodyFlagLease(
                        world,
                        bodyId,
                        heldCollisionParticipationFlags(targetKind),
                        kHeldCollisionParticipationFlagMode,
                        ownerToken)) {
                    ++summary.collisionLeaseCount;
                } else {
                    ++summary.failedLeaseCount;
                }
            }

            if (primaryBodyId != INVALID_BODY_ID) {
                if (havok_runtime::acquireBodyFlagLease(
                        world,
                        primaryBodyId,
                        kHeldAuthorityBodyFlags,
                        kHeldAuthorityBodyFlagMode,
                        ownerToken)) {
                    ++summary.authorityLeaseCount;
                } else {
                    ++summary.failedLeaseCount;
                }
            }
            return summary;
        }

        HeldBodyFlagLeaseSummary releaseHeldObjectBodyFlagLeases(
            RE::hknpWorld* world,
            std::uint32_t primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            grab_target::Kind targetKind,
            std::uintptr_t ownerToken,
            bool restoreOnFinalLease)
        {
            HeldBodyFlagLeaseSummary summary{};
            if (!world || ownerToken == 0) {
                return summary;
            }

            const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
            summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
            for (const auto bodyId : bodyIds) {
                if (havok_runtime::releaseBodyFlagLease(
                        world,
                        bodyId,
                        heldCollisionParticipationFlags(targetKind),
                        kHeldCollisionParticipationFlagMode,
                        ownerToken,
                        restoreOnFinalLease)) {
                    ++summary.collisionLeaseCount;
                } else {
                    ++summary.failedLeaseCount;
                }
            }

            if (primaryBodyId != INVALID_BODY_ID) {
                if (havok_runtime::releaseBodyFlagLease(
                        world,
                        primaryBodyId,
                        kHeldAuthorityBodyFlags,
                        kHeldAuthorityBodyFlagMode,
                        ownerToken,
                        restoreOnFinalLease)) {
                    ++summary.authorityLeaseCount;
                } else {
                    ++summary.failedLeaseCount;
                }
            }
            return summary;
        }

        void copyPeerInertiaSnapshot(SavedObjectState& target, const SavedObjectState& peer)
        {
            target.savedPackedInertia[0] = peer.savedPackedInertia[0];
            target.savedPackedInertia[1] = peer.savedPackedInertia[1];
            target.savedPackedInertia[2] = peer.savedPackedInertia[2];
            target.savedPackedMass = peer.savedPackedMass;
            target.inertiaModified = peer.inertiaModified;
            target.motionInertiaStates = peer.motionInertiaStates;
        }

        std::vector<std::uint32_t> buildCommittedHeldBodyIds(
            std::uint32_t primaryBodyId,
            const std::vector<std::uint32_t>& mechanicalScopeBodyIds,
            const GrabSharedObjectContext& sharedContext,
            bool& adoptedPeerHeldBodyIds)
        {
            /*
             * The second hand has its own selected primary body for drive-frame
             * capture, but the object body set is already owned by the first hand.
             * Reusing the peer's committed body list keeps multipart activation,
             * flag leases, release velocity, and final restoration on the same
             * bodies instead of depending on a second close-selection rescan.
             */
            adoptedPeerHeldBodyIds = false;
            std::vector<std::uint32_t> sourceBodyIds;
            if (sharedContext.hasPeerState() && sharedContext.peerHeldBodyIds && !sharedContext.peerHeldBodyIds->empty()) {
                sourceBodyIds = *sharedContext.peerHeldBodyIds;
                adoptedPeerHeldBodyIds = true;
            } else {
                sourceBodyIds = mechanicalScopeBodyIds;
            }

            return held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, sourceBodyIds);
        }

        GrabSurfaceHit makeCollisionQueryGrabSurfaceHit(const SelectedObject& selection, RE::NiAVObject* fallbackOwnerNode)
        {
            GrabSurfaceHit result{};
            if (!selection.hasHitPoint || !selection.hasHitNormal) {
                return result;
            }

            result.position = selection.hitPointWorld;
            result.normal = normalizeOrZero(selection.hitNormalWorld);
            result.triangleIndex = -1;
            result.distance = selection.distance;
            result.sourceNode = selection.hitNode ? selection.hitNode : fallbackOwnerNode;
            result.sourceShape = nullptr;
            result.sourceKind = GrabSurfaceSourceKind::CollisionQuery;
            result.shapeKey = selection.hitShapeKey;
            result.shapeCollisionFilterInfo = selection.hitShapeCollisionFilterInfo;
            result.hitFraction = selection.hitFraction;
            result.hasSelectionHit = true;
            result.selectionToMeshDistanceGameUnits = 0.0f;
            result.signedAlongPalmDistanceGameUnits = selection.signedAlongDistance;
            result.lateralPalmDistanceGameUnits = selection.lateralDistance;
            result.hasShapeKey = selection.hasHitShapeKey;
            result.hasTriangle = false;
            result.valid = lengthSquared(result.normal) > 0.0f;
            return result;
        }

        grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimeFingerPoseTargets(
            const RE::NiPoint3& seatPointWorld,
            const RE::NiPoint3& seatNormalWorld)
        {
            auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(seatPointWorld, seatNormalWorld);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = true;
            return targets;
        }

        struct RuntimePinchPocketCandidate
        {
            GrabSurfaceHit surfaceHit{};
            grab_pinch_pocket_policy::MeshExtentMetrics meshExtents{};
            grab_pinch_pocket_policy::ObjectDecision decision{};
            RE::NiPoint3 thumbPadWorld{};
            RE::NiPoint3 indexPadWorld{};
            RE::NiPoint3 pinchPocketWorld{};
            RE::NiPoint3 pinchAxisWorld{ 1.0f, 0.0f, 0.0f };
            RE::NiPoint3 pinchDetectionDirectionWorld{ 1.0f, 0.0f, 0.0f };
            float thumbIndexGapGameUnits = 0.0f;
            float pocketToSurfaceDistanceGameUnits = std::numeric_limits<float>::infinity();
            grab_pinch_pocket_policy::ClosureSolution closure{};
            bool valid = false;
        };

        grab_pinch_pocket_policy::Config currentPinchPocketConfig()
        {
            return grab_pinch_pocket_policy::sanitizeConfig(grab_pinch_pocket_policy::Config{
                .enabled = g_rockConfig.rockGrabPinchPocketEnabled,
                .maxVolumeCubicGameUnits = g_rockConfig.rockGrabPinchMaxVolumeCubicGameUnits,
                .maxPocketDistanceGameUnits = g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits,
                .minFingerGapGameUnits = g_rockConfig.rockGrabPinchMinFingerGapGameUnits,
                .maxFingerGapGameUnits = g_rockConfig.rockGrabPinchMaxFingerGapGameUnits,
                .thumbIndexMaxOpenValue = g_rockConfig.rockGrabPinchThumbIndexMaxOpenValue,
                .otherFingerCurlValue = g_rockConfig.rockGrabPinchOtherFingerCurlValue,
                .detectionDirectionHandspace = g_rockConfig.rockGrabPinchDetectionDirectionHandspace,
                .detectionAxisBlend = g_rockConfig.rockGrabPinchDetectionAxisBlend,
            });
        }

        bool rebaseFingerSkeletonSnapshot(
            root_flattened_finger_skeleton_runtime::Snapshot& snapshot,
            const RE::NiTransform& sourceHandWorld,
            const RE::NiTransform& targetHandWorld)
        {
            if (!snapshot.valid ||
                !std::isfinite(sourceHandWorld.scale) || std::abs(sourceHandWorld.scale) <= 0.000001f ||
                !std::isfinite(targetHandWorld.scale) || std::abs(targetHandWorld.scale) <= 0.000001f) {
                return false;
            }

            for (auto& finger : snapshot.fingers) {
                if (!finger.valid) {
                    return false;
                }
                for (auto& point : finger.points) {
                    point = transform_math::localPointToWorld(
                        targetHandWorld,
                        transform_math::worldPointToLocal(sourceHandWorld, point));
                }
                if (finger.tipGeometryValid) {
                    finger.tipSegmentCenterWorld = transform_math::localPointToWorld(targetHandWorld,
                        transform_math::worldPointToLocal(sourceHandWorld, finger.tipSegmentCenterWorld));
                    finger.tipDirectionWorld = normalizeOrZero(transform_math::localVectorToWorld(targetHandWorld,
                        transform_math::worldVectorToLocal(sourceHandWorld, finger.tipDirectionWorld)));
                }
            }
            if (snapshot.palmNormalValid) {
                snapshot.palmNormalWorld = normalizeOrZero(
                    transform_math::localVectorToWorld(
                        targetHandWorld,
                        transform_math::worldVectorToLocal(sourceHandWorld, snapshot.palmNormalWorld)));
                if (lengthSquared(snapshot.palmNormalWorld) <= 0.000001f) {
                    return false;
                }
            }
            return true;
        }

        bool solveRuntimePinchClosure(RuntimePinchPocketCandidate& candidate,
            const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectWorld, const RE::NiTransform& handWorld,
            bool isLeft, const Hand& hand);

        RuntimePinchPocketCandidate buildRuntimePinchPocketCandidate(
            const SelectedObject& selection,
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            std::uint32_t resolvedBodyId,
            const RE::NiTransform& objectWorldTransform,
            const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
            const GrabSurfaceQueryIndex& queryIndex,
            const std::vector<GrabLocalTriangle>& localMeshTriangles,
            const RE::NiTransform& handWorldTransform,
            bool isLeft,
            const Hand& hand,
            bool closeGrab,
            bool handPocketOnlyGrab,
            bool looseWeaponGrab)
        {
            performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabPinchPocket);
            RuntimePinchPocketCandidate candidate{};
            const auto config = currentPinchPocketConfig();
            candidate.meshExtents = grab_pinch_pocket_policy::computeMeshExtents(localMeshTriangles, objectWorldTransform.scale);

            grab_pinch_pocket_policy::FingerFrame pinchFrame{};
            const bool hasFingerSnapshot = hand.tryGetPinchFingerFrame(pinchFrame);

            bool hasPinchSurface = false;
            bool ownerMatchesResolvedBody = false;
            if (hasFingerSnapshot && !surfaceTriangles.empty()) {
                candidate.thumbPadWorld = pinchFrame.thumbTip;
                candidate.indexPadWorld = pinchFrame.indexTip;
                candidate.thumbIndexGapGameUnits = pinchFrame.gapGameUnits;
                candidate.pinchAxisWorld = pinchFrame.axis;
                candidate.pinchPocketWorld = pinchFrame.center;
                const RE::NiPoint3 configuredDetectionWorld =
                    transformHandspaceDirection(handWorldTransform, config.detectionDirectionHandspace, isLeft);
                candidate.pinchDetectionDirectionWorld = grab_pinch_pocket_policy::detectionDirection(
                    pinchFrame, configuredDetectionWorld, config.detectionAxisBlend);

                GrabSurfaceHit surfaceHit{};
                hasPinchSurface = findClosestGrabSurfaceHitToPointPositionOnly(
                    surfaceTriangles,
                    candidate.pinchPocketWorld,
                    candidate.pinchDetectionDirectionWorld,
                    config.maxPocketDistanceGameUnits,
                    surfaceHit, &queryIndex);
                if (hasPinchSurface) {
                    candidate.surfaceHit = surfaceHit;
                    candidate.pocketToSurfaceDistanceGameUnits =
                        grab_pinch_pocket_policy::distance(candidate.pinchPocketWorld, surfaceHit.position);
                    if (surfaceHit.sourceNode) {
                        const auto* ownerRecord = bodySet.findAcceptedRecordByOwnerNode(surfaceHit.sourceNode);
                        ownerMatchesResolvedBody =
                            (ownerRecord && ownerRecord->bodyId == resolvedBodyId) ||
                            acceptsSelectedOwnerlessVisualMesh(selection,
                                bodySet,
                                resolvedBodyId,
                                surfaceHit.sourceNode,
                                ownerRecord);
                    }
                }
            }

            candidate.decision = grab_pinch_pocket_policy::evaluateObject(grab_pinch_pocket_policy::ObjectDecisionInput{
                .config = config,
                .mesh = candidate.meshExtents,
                .closeGrab = closeGrab,
                .handPocketOnlyGrab = handPocketOnlyGrab,
                .looseWeaponGrab = looseWeaponGrab,
                .ownerMatchesResolvedBody = ownerMatchesResolvedBody,
                .hasFingerSnapshot = hasFingerSnapshot,
                .hasPinchSurface = hasPinchSurface,
                .multipleAcceptedBodies = bodySet.acceptedCount() > 1,
                .thumbIndexGapGameUnits = candidate.thumbIndexGapGameUnits,
                .pocketToSurfaceDistanceGameUnits = candidate.pocketToSurfaceDistanceGameUnits,
            });
            candidate.valid = candidate.decision.accept;
            if (candidate.valid && !solveRuntimePinchClosure(candidate, localMeshTriangles, objectWorldTransform, handWorldTransform, isLeft, hand)) {
                candidate.valid = false;
                candidate.decision.accept = false;
                candidate.decision.retryable = true;
                candidate.decision.reason = "pinchPoseUnavailable";
            }
            return candidate;
        }

        struct LocalMeshLongAxis
        {
            RE::NiPoint3 axisWorld{};
            float spanGameUnits = 0.0f;
            bool valid = false;
        };

        LocalMeshLongAxis computeLocalMeshLongAxis(
            const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectWorldTransform)
        {
            LocalMeshLongAxis result{};
            if (localTriangles.empty() || !grab_three_phase::isFinite(objectWorldTransform)) {
                return result;
            }

            RE::NiPoint3 minPoint{
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)(),
            };
            RE::NiPoint3 maxPoint{
                -(std::numeric_limits<float>::max)(),
                -(std::numeric_limits<float>::max)(),
                -(std::numeric_limits<float>::max)(),
            };

            auto visit = [&](const RE::NiPoint3& point) {
                minPoint.x = (std::min)(minPoint.x, point.x);
                minPoint.y = (std::min)(minPoint.y, point.y);
                minPoint.z = (std::min)(minPoint.z, point.z);
                maxPoint.x = (std::max)(maxPoint.x, point.x);
                maxPoint.y = (std::max)(maxPoint.y, point.y);
                maxPoint.z = (std::max)(maxPoint.z, point.z);
            };
            for (const auto& triangle : localTriangles) {
                visit(triangle.v0);
                visit(triangle.v1);
                visit(triangle.v2);
            }

            const RE::NiPoint3 extents = maxPoint - minPoint;
            if (!std::isfinite(extents.x) || !std::isfinite(extents.y) || !std::isfinite(extents.z)) {
                return result;
            }

            RE::NiPoint3 localAxis{ 1.0f, 0.0f, 0.0f };
            float spanLocal = extents.x;
            if (extents.y > spanLocal) {
                localAxis = RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
                spanLocal = extents.y;
            }
            if (extents.z > spanLocal) {
                localAxis = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
                spanLocal = extents.z;
            }

            const float objectScale =
                std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
            result.axisWorld = normalizeOrZero(transform_math::localVectorToWorld(objectWorldTransform, localAxis));
            result.spanGameUnits = spanLocal * objectScale;
            result.valid = result.spanGameUnits > 0.001f && lengthSquared(result.axisWorld) > 0.000001f;
            return result;
        }

        /*
         * Thumb and index keep the mesh solve, so they stop on the object's
         * surface instead of closing through it onto each other. The free
         * fingers tuck to the pinch curl unless the mesh stops them first.
         */
        void applyPinchFingerPosePolicy(
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const CanonicalGrabFrame& frame)
        {
            const auto& tucked = frame.seat.pinchFingerPose;
            for (std::size_t finger = 2; finger < pose.values.size(); ++finger) {
                if (tucked.values[finger] <= pose.values[finger]) {
                    continue;
                }
                pose.values[finger] = tucked.values[finger];
                if (pose.hasJointValues) {
                    for (std::size_t segment = 0; segment < 3; ++segment) {
                        pose.jointValues[finger * 3 + segment] = tucked.jointValues[finger * 3 + segment];
                    }
                }
            }
        }

        void storeFingerPoseTargetsInGrabFrame(CanonicalGrabFrame& frame,
            const grab_finger_pose_runtime::GrabFingerPoseTargetSet& targets,
            const RE::NiTransform& objectWorldTransform)
        {
            frame.fingerPoseTargetLocal = {};
            frame.fingerPoseTargetNormalLocal = {};
            frame.fingerPoseTargetValid = {};
            frame.fingerPoseTargetNormalValid = {};
            frame.fingerPoseTargetCount = 0;
            for (std::size_t finger = 0; finger < targets.targets.size(); ++finger) {
                if (!targets.targetValid[finger]) {
                    continue;
                }
                frame.fingerPoseTargetLocal[finger] = transform_math::worldPointToLocal(objectWorldTransform, targets.targets[finger]);
                frame.fingerPoseTargetValid[finger] = 1;
                if (targets.targetNormalValid[finger]) {
                    frame.fingerPoseTargetNormalLocal[finger] = transform_math::worldVectorToLocal(objectWorldTransform, targets.targetNormals[finger]);
                    frame.fingerPoseTargetNormalValid[finger] = 1;
                }
                ++frame.fingerPoseTargetCount;
            }
        }

        void storeGripSourceEvidence(CanonicalGrabFrame& frame,
            RE::NiAVObject* sourceNode,
            const RE::NiTransform& fallbackObjectWorld,
            const RE::NiPoint3& gripPointWorld,
            const RE::NiPoint3& gripNormalWorld,
            bool normalValid)
        {
            /*
             * Mesh triangles choose a world-space position only. A rendered
             * triangle can live under a child node whose native X/Y/Z does not
             * match the collidable node or the hknp BODY, so its local point is
             * stored only as source-node evidence. Solver authority is the
             * separate BODY-local pivot B captured from the same world point.
             */
            const RE::NiTransform& evidenceWorld = sourceNode ? sourceNode->world : fallbackObjectWorld;
            frame.gripEvidence.gripSourceNode = sourceNode;
            frame.gripEvidence.gripSourceNodeWorldAtGrab = evidenceWorld;
            frame.gripEvidence.gripPointSourceNodeLocal = transform_math::worldPointToLocal(evidenceWorld, gripPointWorld);
            frame.gripEvidence.hasGripSourceNodePoint = true;
            if (normalValid) {
                frame.gripEvidence.gripNormalSourceNodeLocal = transform_math::worldVectorToLocal(evidenceWorld, gripNormalWorld);
                frame.gripEvidence.hasGripSourceNodeNormal = true;
            } else {
                frame.gripEvidence.gripNormalSourceNodeLocal = {};
                frame.gripEvidence.hasGripSourceNodeNormal = false;
            }
        }

        RE::NiTransform gripEvidenceWorldFrame(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
        {
            if (frame.gripEvidence.gripSourceNode) {
                return frame.gripEvidence.gripSourceNode->world;
            }
            if (frame.gripEvidence.hasGripSourceNodePoint) {
                return frame.gripEvidence.gripSourceNodeWorldAtGrab;
            }
            return fallbackWorld;
        }

        RE::NiTransform gripEvidenceWorldFrame(const ImmutableGrabCaptureTelemetry& capture, const RE::NiTransform& fallbackWorld)
        {
            if (capture.gripEvidence.gripSourceNode) {
                return capture.gripEvidence.gripSourceNode->world;
            }
            if (capture.gripEvidence.hasGripSourceNodePoint) {
                return capture.gripEvidence.gripSourceNodeWorldAtGrab;
            }
            return fallbackWorld;
        }

        RE::NiPoint3 gripEvidencePointWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
        {
            if (frame.gripEvidence.hasGripSourceNodePoint) {
                return transform_math::localPointToWorld(gripEvidenceWorldFrame(frame, fallbackWorld), frame.gripEvidence.gripPointSourceNodeLocal);
            }
            return transform_math::localPointToWorld(fallbackWorld, frame.gripEvidence.gripPointLocal);
        }

        RE::NiPoint3 gripEvidenceNormalWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
        {
            if (frame.gripEvidence.hasGripSourceNodeNormal) {
                return normalizeOrZero(transform_math::localVectorToWorld(gripEvidenceWorldFrame(frame, fallbackWorld), frame.gripEvidence.gripNormalSourceNodeLocal));
            }
            return normalizeOrZero(transform_math::localVectorToWorld(fallbackWorld, frame.gripEvidence.gripNormalLocal));
        }

        grab_finger_pose_runtime::GrabFingerPoseTargetSet rebuildFingerPoseTargetsFromGrabFrame(
            const CanonicalGrabFrame& frame,
            const RE::NiTransform& currentNodeWorld)
        {
            const RE::NiPoint3 seatNormalWorld = frame.gripEvidence.hasGripPoint ? gripEvidenceNormalWorld(frame, currentNodeWorld) : RE::NiPoint3{};
            auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(gripEvidencePointWorld(frame, currentNodeWorld), seatNormalWorld);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = true;
            for (std::size_t finger = 0; finger < frame.fingerPoseTargetLocal.size(); ++finger) {
                if (!frame.fingerPoseTargetValid[finger]) {
                    continue;
                }
                targets.targets[finger] = transform_math::localPointToWorld(currentNodeWorld, frame.fingerPoseTargetLocal[finger]);
                targets.targetValid[finger] = 1;
                if (frame.fingerPoseTargetNormalValid[finger]) {
                    targets.targetNormals[finger] = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, frame.fingerPoseTargetNormalLocal[finger]));
                    targets.targetNormalValid[finger] = 1;
                }
                ++targets.targetCount;
            }
            return targets;
        }

        std::vector<TriangleData> rebuildFingerPoseWorldTrianglesFromGrabFrame(
            const CanonicalGrabFrame& frame,
            const RE::NiTransform& currentNodeWorld)
        {
            std::vector<TriangleData> worldTriangles;
            const auto& localPoseTriangles = !frame.fingerPoseLocalMeshTriangles.empty() ?
                frame.fingerPoseLocalMeshTriangles :
                frame.localMeshTriangles;
            worldTriangles.reserve(localPoseTriangles.size());
            for (const auto& localTriangle : localPoseTriangles) {
                worldTriangles.push_back(TriangleData{
                    transform_math::localPointToWorld(currentNodeWorld, localTriangle.v0),
                    transform_math::localPointToWorld(currentNodeWorld, localTriangle.v1),
                    transform_math::localPointToWorld(currentNodeWorld, localTriangle.v2),
                });
            }
            return worldTriangles;
        }

        float translationDeltaGameUnits(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            const RE::NiPoint3 delta = a.translate - b.translate;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            const RE::NiMatrix3 delta = a.Transpose() * b;
            float cosTheta = (delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f;
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        struct GrabCaptureTransformRefreshSample
        {
            const char* role = "unknown";
            RE::NiAVObject* node = nullptr;
            bool validBefore = true;
            bool validAfter = true;
            float positionDeltaGameUnits = 0.0f;
            float rotationDeltaDegrees = 0.0f;
        };

        struct GrabCaptureTransformRefreshResult
        {
            std::array<GrabCaptureTransformRefreshSample, 6> samples{};
            std::uint32_t count = 0;
            bool ok = true;
        };

        bool grabCaptureRefreshAlreadyVisited(const GrabCaptureTransformRefreshResult& result, const RE::NiAVObject* node)
        {
            for (std::uint32_t i = 0; i < result.count; ++i) {
                if (result.samples[i].node == node) {
                    return true;
                }
            }
            return false;
        }

        void refreshGrabCaptureNodeTransform(GrabCaptureTransformRefreshResult& result, const char* role, RE::NiAVObject* node, bool refreshScene = true)
        {
            if (!node || result.count >= result.samples.size() || grabCaptureRefreshAlreadyVisited(result, node)) {
                return;
            }

            auto& sample = result.samples[result.count++];
            sample.role = role ? role : "unknown";
            sample.node = node;

            const RE::NiTransform before = node->world;
            sample.validBefore = grab_three_phase::isFinite(before);

            // Ragdoll world transforms are the engine's current physics/skin
            // output. Acquisition must sample that pose without recalculating it.
            if (refreshScene) {
                RE::NiUpdateData update{};
                node->UpdateTransforms(update);
            }

            const RE::NiTransform after = node->world;
            sample.validAfter = grab_three_phase::isFinite(after);
            sample.positionDeltaGameUnits = translationDeltaGameUnits(before, after);
            sample.rotationDeltaDegrees = rotationDeltaDegrees(before.rotate, after.rotate);
            result.ok = result.ok && sample.validAfter;
        }

        GrabCaptureTransformRefreshResult refreshGrabCaptureTransforms(
            RE::NiAVObject* rootNode,
            RE::NiAVObject* meshSourceNode,
            RE::NiAVObject* collidableNode,
            bool refreshScene = true)
        {
            GrabCaptureTransformRefreshResult result{};
            refreshGrabCaptureNodeTransform(result, "root", rootNode, refreshScene);
            refreshGrabCaptureNodeTransform(result, "mesh", meshSourceNode, refreshScene);
            refreshGrabCaptureNodeTransform(result, "collidable", collidableNode, refreshScene);
            return result;
        }

        float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
            const float lenA = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
            const float lenB = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
            if (lenA < 0.0001f || lenB < 0.0001f) {
                return -1.0f;
            }

            float cosTheta = dot / (lenA * lenB);
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                   matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                   matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        struct GrabPalmBasisDelta
        {
            float rotationDegrees = -1.0f;
            float xAxisDegrees = -1.0f;
            float yAxisDegrees = -1.0f;
            float zAxisDegrees = -1.0f;
            float rawDeterminant = 0.0f;
            float proxyDeterminant = 0.0f;
        };

        RE::NiPoint3 frameAxisWorld(const RE::NiTransform& transform, const RE::NiPoint3& localAxis)
        {
            return normalizeOrZero(transform_math::localVectorToWorld(transform, localAxis));
        }

        RE::NiPoint3 generatedFrameAxisWorld(const RE::NiTransform& transform, const RE::NiPoint3& localAxis)
        {
            return normalizeOrZero(hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(transform, localAxis));
        }

        GrabPalmBasisDelta computeGrabPalmBasisDelta(const RE::NiTransform& rawHandWorld, const RE::NiTransform& proxyWorld)
        {
            GrabPalmBasisDelta result{};
            result.rotationDegrees = rotationDeltaDegrees(rawHandWorld.rotate, proxyWorld.rotate);
            result.xAxisDegrees = axisDeltaDegrees(
                frameAxisWorld(rawHandWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }));
            result.yAxisDegrees = axisDeltaDegrees(
                frameAxisWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
                generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }));
            result.zAxisDegrees = axisDeltaDegrees(
                frameAxisWorld(rawHandWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }),
                generatedFrameAxisWorld(proxyWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }));
            result.rawDeterminant = matrixDeterminant(rawHandWorld.rotate);
            result.proxyDeterminant = matrixDeterminant(proxyWorld.rotate);
            return result;
        }

        RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = hkMatrix[0];
            result.entry[1][0] = hkMatrix[1];
            result.entry[2][0] = hkMatrix[2];
            result.entry[0][1] = hkMatrix[4];
            result.entry[1][1] = hkMatrix[5];
            result.entry[2][1] = hkMatrix[6];
            result.entry[0][2] = hkMatrix[8];
            result.entry[1][2] = hkMatrix[9];
            result.entry[2][2] = hkMatrix[10];
            return result;
        }

        RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = hkMatrix[0];
            result.entry[0][1] = hkMatrix[1];
            result.entry[0][2] = hkMatrix[2];
            result.entry[1][0] = hkMatrix[4];
            result.entry[1][1] = hkMatrix[5];
            result.entry[1][2] = hkMatrix[6];
            result.entry[2][0] = hkMatrix[8];
            result.entry[2][1] = hkMatrix[9];
            result.entry[2][2] = hkMatrix[10];
            return result;
        }

        RE::NiTransform rotationOnlyTransform(const RE::NiMatrix3& rotation)
        {
            RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
            result.rotate = rotation;
            return result;
        }

        RE::NiMatrix3 frameToFrameRotation(const RE::NiMatrix3& fromWorldRotation, const RE::NiMatrix3& toWorldRotation)
        {
            const RE::NiTransform fromWorld = rotationOnlyTransform(fromWorldRotation);
            const RE::NiTransform toWorld = rotationOnlyTransform(toWorldRotation);
            return transform_math::composeTransforms(transform_math::invertTransform(fromWorld), toWorld).rotate;
        }

        RE::NiTransform makeIdentityTransform()
        {
            return transform_math::makeIdentityTransform<RE::NiTransform>();
        }

        RE::NiTransform reconstructBodyWorldFromProxyInBody(const RE::NiTransform& proxyWorld,
            const RE::NiMatrix3& proxyInBodyRotation,
            const RE::NiPoint3& transformBLocalGame,
            const RE::NiPoint3& pivotAProxyLocalGame)
        {
            RE::NiTransform proxyInBody = makeIdentityTransform();
            proxyInBody.rotate = proxyInBodyRotation;
            const RE::NiPoint3 rotatedPivotA = transform_math::localVectorToWorld(proxyInBody, pivotAProxyLocalGame);
            proxyInBody.translate = transformBLocalGame - rotatedPivotA;
            const RE::NiTransform bodyInProxy = transform_math::invertTransform(proxyInBody);
            return grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, bodyInProxy);
        }

        RE::NiTransform reconstructSolverEffectiveBodyWorld(const RE::NiTransform& proxyWorld,
            const RE::NiMatrix3& transformARotation,
            const RE::NiMatrix3& transformBRotation,
            const RE::NiMatrix3& targetBRcaRotation,
            const RE::NiPoint3& transformBLocalGame,
            const RE::NiPoint3& anchorAWorld,
            float bodyScale)
        {
            const RE::NiMatrix3 constraintAWorldRotation =
                transform_math::composeTransforms(proxyWorld, rotationOnlyTransform(transformARotation)).rotate;
            const RE::NiMatrix3 desiredConstraintBWorldRotation =
                transform_math::composeTransforms(
                    rotationOnlyTransform(constraintAWorldRotation),
                    rotationOnlyTransform(transform_math::transposeRotation(targetBRcaRotation)))
                    .rotate;
            const RE::NiMatrix3 desiredBodyRotation =
                transform_math::composeTransforms(
                    rotationOnlyTransform(desiredConstraintBWorldRotation),
                    rotationOnlyTransform(transform_math::transposeRotation(transformBRotation)))
                    .rotate;

            RE::NiTransform result = makeIdentityTransform();
            result.rotate = desiredBodyRotation;
            result.scale = std::isfinite(bodyScale) && bodyScale > 0.0f ? bodyScale : 1.0f;
            result.translate = anchorAWorld - transform_math::localVectorToWorld(result, transformBLocalGame);
            return result;
        }

        physics_recursive_wrappers::MotionPreset motionPresetFromMotionType(
            physics_body_classifier::BodyMotionType motionType,
            std::uint16_t fallbackMotionPropertiesId)
        {
            switch (motionType) {
            case physics_body_classifier::BodyMotionType::Static:
                return physics_recursive_wrappers::MotionPreset::Static;
            case physics_body_classifier::BodyMotionType::Keyframed:
                return physics_recursive_wrappers::MotionPreset::Keyframed;
            case physics_body_classifier::BodyMotionType::Dynamic:
                return physics_recursive_wrappers::MotionPreset::Dynamic;
            default:
                break;
            }

            switch (fallbackMotionPropertiesId & 0xFF) {
            case 0:
                return physics_recursive_wrappers::MotionPreset::Static;
            case 2:
                return physics_recursive_wrappers::MotionPreset::Keyframed;
            case 1:
            default:
                return physics_recursive_wrappers::MotionPreset::Dynamic;
            }
        }

        active_grab_body_lifecycle::BodyLifecycleAudit restoreActiveGrabLifecycle(RE::hknpWorld* world,
            const active_grab_body_lifecycle::BodyLifecycleSnapshot& snapshot,
            const active_grab_body_lifecycle::BodyRestorePlan& plan,
            std::uint32_t primaryBodyId,
            const char* handName,
            const char* context)
        {
            auto audit = active_grab_body_lifecycle::makeLifecycleAudit(snapshot, plan, primaryBodyId);
            if (!world) {
                return audit;
            }

            for (const auto& entry : plan.entries) {
                const auto bodyId = entry.record.bodyId;
                if (bodyId == INVALID_BODY_ID) {
                    continue;
                }

                if (entry.restoreFilter) {
                    body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, entry.record.filterInfo);
                }
            }

            for (const auto& command : snapshot.makeMotionRestoreCommands(plan)) {
                auto* ownerNode = reinterpret_cast<RE::NiAVObject*>(command.ownerKey);
                if (!ownerNode) {
                    continue;
                }
                physics_recursive_wrappers::setMotionRecursive(
                    ownerNode,
                    motionPresetFromMotionType(command.motionType, command.motionPropertiesId),
                    command.recursive,
                    command.force,
                    command.activate);
            }

            ROCK_LOG_DEBUG(Hand,
                "{} hand grab lifecycle audit {}: targetKind={} intent={} bodies={} converted={} restoredMotion={} restoredFilter={} preservedMotion={} preservedFilter={} dampingSnapshots={} inertiaSnapshots={} latePrepared={} incompleteScan={} primaryBody={}",
                handName ? handName : "?",
                context ? context : "",
                grab_target::name(audit.targetKind),
                active_grab_body_lifecycle::releaseIntentName(audit.intent),
                audit.bodyCount,
                audit.convertedCount,
                audit.restoredMotionCount,
                audit.restoredFilterCount,
                audit.preservedConvertedMotionCount,
                audit.preservedConvertedFilterCount,
                audit.dampingSnapshotCount,
                audit.inertiaSnapshotCount,
                audit.latePreparedBodyCount,
                audit.incompleteNativeScan ? "yes" : "no",
                audit.primaryBodyId);
            return audit;
        }

        bool restoreIncompleteActivePrepRoot(
            RE::NiAVObject* rootNode,
            std::uint16_t originalMotionPropsId,
            const char* handName,
            const char* context)
        {
            /*
             * Late-discovered bodies have already been touched by recursive
             * active prep, but their original per-body state was not captured.
             * Individual restore would manufacture state. When a scan is known
             * incomplete, restore the object root with the selected body's
             * original motion preset so the whole Fallout-owned system returns
             * to one coherent motion mode.
             */
            if (!rootNode) {
                return false;
            }

            const auto motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(originalMotionPropsId);
            const bool restored = physics_recursive_wrappers::setMotionRecursive(
                rootNode,
                motionPresetFromMotionType(motionType, originalMotionPropsId),
                true,
                true,
                false);

            ROCK_LOG_WARN(Hand,
                "{} hand {}: recursive root restore after incomplete object scan root='{}' motionProps={} result={}",
                handName ? handName : "?",
                context ? context : "incomplete-scan",
                nodeDebugName(rootNode),
                originalMotionPropsId,
                restored ? "ok" : "failed");
            return restored;
        }

        RE::NiTransform invertTransform(const RE::NiTransform& transform) { return transform_math::invertTransform(transform); }

        RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child) { return transform_math::composeTransforms(parent, child); }

        RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld, const RE::NiTransform& bodyLocalTransform)
        {
            return multiplyTransforms(bodyWorld, invertTransform(bodyLocalTransform));
        }

        std::vector<GrabLocalTriangle> cacheTrianglesInLocalSpace(const std::vector<TriangleData>& worldTriangles, const RE::NiTransform& nodeWorld)
        {
            performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabLocalTriangleCapture);
            std::vector<GrabLocalTriangle> localTriangles;
            localTriangles.reserve(worldTriangles.size());
            for (const auto& triangle : worldTriangles) {
                localTriangles.push_back(GrabLocalTriangle{
                    transform_math::worldPointToLocal(nodeWorld, triangle.v0),
                    transform_math::worldPointToLocal(nodeWorld, triangle.v1),
                    transform_math::worldPointToLocal(nodeWorld, triangle.v2),
                });
            }
            return localTriangles;
        }

        constexpr std::size_t kMaxGrabRuntimeSurfaceContactTriangles = 2048;
        constexpr std::size_t kMaxGrabRuntimeFingerPoseTriangles = 2048;

        std::vector<GrabSurfaceTriangleData> selectNearestGrabSurfaceTriangles(
            const std::vector<GrabSurfaceTriangleData>& sourceTriangles,
            const RE::NiPoint3& centerWorld, std::size_t maxTriangles,
            const GrabSurfaceQueryIndex& queryIndex)
        {
            if (sourceTriangles.size() <= maxTriangles || maxTriangles == 0 || !grab_three_phase::isFinite(centerWorld)) {
                return sourceTriangles;
            }
            const auto& selected = queryIndex.nearest(sourceTriangles, centerWorld, maxTriangles);
            std::vector<GrabSurfaceTriangleData> result;
            result.reserve(selected.size());
            for (const auto& triangle : selected) result.push_back(sourceTriangles[triangle.index]);
            return result;
        }

        std::vector<TriangleData> selectNearestGrabFingerPoseTriangles(
            const std::vector<GrabSurfaceTriangleData>& sourceTriangles,
            const RE::NiPoint3& centerWorld, std::size_t maxTriangles,
            const GrabSurfaceQueryIndex& queryIndex)
        {
            std::vector<TriangleData> result;
            if (sourceTriangles.size() <= maxTriangles || maxTriangles == 0 || !grab_three_phase::isFinite(centerWorld)) {
                result.reserve(sourceTriangles.size());
                for (const auto& triangle : sourceTriangles) result.push_back(triangle.triangle);
            } else {
                const auto& selected = queryIndex.nearest(sourceTriangles, centerWorld, maxTriangles);
                result.reserve(selected.size());
                for (const auto& triangle : selected) result.push_back(sourceTriangles[triangle.index].triangle);
            }
            return result;
        }

        RE::NiTransform getLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryResolveLiveBodyWorldTransform(world, bodyId, result);
            return result;
        }

        bool tryGetGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
        {
            /*
             * Dynamic grab object-side state is measured from the hknp BODY slot.
             * The hand side is ROCK's hidden no-contact proxy. The held object's
             * contact pivot and visual node relation stay in BODY space; MOTION is
             * COM/weight/diagnostic data only and never grip authority.
             */
            outTransform = makeIdentityTransform();
            return tryGetBodyArrayWorldTransform(world, bodyId, outTransform);
        }

        RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld, const RE::NiTransform& bodyWorld)
        {
            return multiplyTransforms(invertTransform(nodeWorld), bodyWorld);
        }

        constexpr const char* kGrabObjectRotationReferenceName = "generatedProxyAuthorityLocal";
        constexpr float kGrabFrameMismatchRawProxyRotationWarnDegrees = 20.0f;

        float computeLocalMeshMaxDistanceFromPoint(const std::vector<GrabLocalTriangle>& localTriangles, const RE::NiPoint3& originLocal)
        {
            if (localTriangles.empty()) {
                return 0.0f;
            }

            float maxDistanceSquared = 0.0f;
            auto visit = [&](const RE::NiPoint3& point) {
                const RE::NiPoint3 delta = point - originLocal;
                const float distanceSquared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
                if (std::isfinite(distanceSquared)) {
                    maxDistanceSquared = (std::max)(maxDistanceSquared, distanceSquared);
                }
            };
            for (const auto& triangle : localTriangles) {
                visit(triangle.v0);
                visit(triangle.v1);
                visit(triangle.v2);
            }
            return maxDistanceSquared > 0.0f ? std::sqrt(maxDistanceSquared) : 0.0f;
        }

        using GrabPivotAuthoritySource =
            grab_authority_frame_math::GrabAuthorityPivotSource;

        const char* grabPivotAuthoritySourceName(GrabPivotAuthoritySource source)
        {
            return grab_authority_frame_math::grabAuthorityPivotSourceName(source);
        }

        void applyFrozenGrabAuthorityFrameToGrabFrame(
            CanonicalGrabFrame& frame,
            const grab_authority_frame_math::FrozenGrabAuthorityFrame<RE::NiTransform>& frozen)
        {
            if (!frozen.valid) {
                return;
            }

            frame.rawHandSpace = frozen.rawHandSpace;
            frame.handBodyToRawHandAtGrab = frozen.handBodyToRawHandAtGrab;
            frame.proxyAuthorityHandSpace = frozen.proxyAuthorityHandSpace;
            frame.proxyAuthorityBodyHandSpace = frozen.proxyAuthorityBodyHandSpace;
            frame.authority.bodyLocal = frozen.bodyLocal;
            frame.authority.bodyWorldAtGrab = frozen.bodyWorldAtGrab;
            frame.rootBodyLocal = frozen.rootBodyLocal;
            frame.ownerBodyLocal = frozen.ownerBodyLocal;
            frame.gripEvidence.gripPointLocal = frozen.gripPointLocal;
            frame.authority.gripPointBodyLocalGame = frozen.pivotBBodyLocalGame;
            frame.authority.pivotBBodyLocalGame = frozen.pivotBBodyLocalGame;
            frame.authority.pivotBConstraintLocalGame = frozen.pivotBConstraintLocalGame;
            frame.authority.pivotAHandBodyLocalGame = frozen.pivotAHandBodyLocalGame;
            frame.authority.grabPivotWorldAtGrab = frozen.grabPivotWorldAtGrab;
            frame.gripEvidence.gripPointWorldAtGrab = frozen.gripPointWorldAtGrab;
            frame.authority.desiredObjectWorldAtGrab = frozen.desiredObjectWorld;
            frame.authority.desiredBodyWorldAtGrab = frozen.desiredBodyWorld;
            frame.authority.hasFrozenPivotB = true;
            frame.gripEvidence.hasGripPoint = true;
        }

        float finitePositiveOr(float value, float fallback)
        {
            return std::isfinite(value) && value > 0.0f ? value : fallback;
        }

        using GrabSeatDepthStopResult = grab_palm_seat::DepthResult;

        GrabSeatDepthStopResult computeGrabPalmSeatDepthStop(
            const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectNodeWorld,
            const RE::NiPoint3& depthOriginWorld,
            const grab_three_phase::GrabPocketFrame& pocket,
            const HandBoneColliderSet::PublishedSegmentFrames& segments,
            float maxFootprintRadiusGameUnits,
            float maxDepthGameUnits)
        {
            GrabSeatDepthStopResult invalid{};
            invalid.reason = "missingPalmFootprint";
            if (!pocket.valid || !isFiniteNiTransform(objectNodeWorld) ||
                !grab_three_phase::isFinite(depthOriginWorld) ||
                !std::isfinite(maxFootprintRadiusGameUnits) || maxFootprintRadiusGameUnits <= 0.0f) {
                invalid.reason = "invalidPalmSeatFrame";
                return invalid;
            }
            for (const auto& segment : segments) {
                if (segment.role != hand_collider_semantics::HandColliderRole::PalmFace || !segment.valid ||
                    !isFiniteNiTransform(segment.target) || !grab_three_phase::isFinite(segment.palmHalfExtents) ||
                    segment.palmHalfExtents.x <= 0.0f || segment.palmHalfExtents.y <= 0.0f || segment.palmHalfExtents.z <= 0.0f ||
                    !std::isfinite(segment.convexRadius) || segment.convexRadius < 0.0f) {
                    continue;
                }
                // Project the created palm hull onto the pocket plane. Its
                // generated-body axes are columns; mesh axes use Ni row storage.
                const auto axisX = hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(segment.target, RE::NiPoint3{ 1, 0, 0 });
                const auto axisY = hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(segment.target, RE::NiPoint3{ 0, 1, 0 });
                const auto axisZ = hand_bone_collider_geometry_math::generatedColliderLocalVectorToWorld(segment.target, RE::NiPoint3{ 0, 0, 1 });
                auto projectedHalfExtent = [&](const RE::NiPoint3& axis) {
                    return std::abs(dotProduct(axisX, axis)) * segment.palmHalfExtents.x +
                           std::abs(dotProduct(axisY, axis)) * segment.palmHalfExtents.y +
                           std::abs(dotProduct(axisZ, axis)) * segment.palmHalfExtents.z +
                           segment.convexRadius * std::abs(segment.target.scale);
                };
                float halfAlong = projectedHalfExtent(pocket.fingerForwardWorld);
                float halfAcross = projectedHalfExtent(pocket.crossPalmWorld);
                const float footprintRadius = std::hypot(halfAlong, halfAcross);
                if (footprintRadius > maxFootprintRadiusGameUnits) {
                    const float scale = maxFootprintRadiusGameUnits / footprintRadius;
                    halfAlong *= scale;
                    halfAcross *= scale;
                }
                // The mesh will translate from depthOrigin to the palm anchor.
                // Carry only that translation into the footprint so side tabs
                // are tested against the palm they would actually overlap.
                const auto footprintCenterWorld = depthOriginWorld + (segment.target.translate - pocket.palmCenterWorld);
                const auto centerLocal = transform_math::worldPointToLocal(objectNodeWorld, footprintCenterWorld);
                const auto originLocal = transform_math::worldPointToLocal(objectNodeWorld, depthOriginWorld);
                const auto alongLocal = transform_math::worldVectorToLocal(objectNodeWorld, pocket.fingerForwardWorld);
                const auto acrossLocal = transform_math::worldVectorToLocal(objectNodeWorld, pocket.crossPalmWorld);
                const auto inwardLocal = transform_math::worldVectorToLocal(objectNodeWorld, pocket.palmNormalWorld * -1.0f);
                const float objectScale = std::abs(objectNodeWorld.scale);
                return grab_palm_seat::computeDepth(localTriangles, halfAlong, halfAcross, maxDepthGameUnits,
                    [&](const RE::NiPoint3& vertex) {
                        const auto lateralDelta = vertex - centerLocal;
                        return grab_palm_seat::Point{
                            dotProduct(lateralDelta, alongLocal) * objectScale,
                            dotProduct(lateralDelta, acrossLocal) * objectScale,
                            dotProduct(vertex - originLocal, inwardLocal) * objectScale,
                        };
                    });
            }
            return invalid;
        }

        /*
         * Pinch thickness uses the existing circular pad footprint on both
         * sides of the pinch axis. Palm seating uses the created palm hull
         * footprint above; finger reach must not widen that support area.
         */
        GrabSeatDepthStopResult computeGrabSeatDepthStop(
            const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectNodeWorld,
            const RE::NiPoint3& gripPointWorld,
            const RE::NiPoint3& palmNormalWorld,
            float footprintRadiusGameUnits,
            float maxDepthGameUnits)
        {
            GrabSeatDepthStopResult result{};
            if (!std::isfinite(maxDepthGameUnits) || maxDepthGameUnits <= 0.0f) {
                result.reason = "seatDepthDisabled";
                return result;
            }
            if (localTriangles.empty()) {
                result.reason = "noLocalTriangles";
                return result;
            }
            if (!grab_three_phase::isFinite(objectNodeWorld) ||
                !grab_three_phase::isFinite(gripPointWorld) ||
                !grab_three_phase::isFinite(palmNormalWorld)) {
                result.reason = "nonFiniteSeatFrame";
                return result;
            }
            const RE::NiPoint3 inwardLocal = normalizeOrZero(transform_math::worldVectorToLocal(
                objectNodeWorld,
                RE::NiPoint3{ -palmNormalWorld.x, -palmNormalWorld.y, -palmNormalWorld.z }));
            if (lengthSquared(inwardLocal) <= 0.000001f) {
                result.reason = "degeneratePalmNormal";
                return result;
            }

            const float objectScale = finitePositiveOr(objectNodeWorld.scale, 1.0f);
            const float footprintRadiusLocal = (std::max)(0.1f, finitePositiveOr(footprintRadiusGameUnits, 10.0f)) / objectScale;
            const float footprintRadiusLocalSquared = footprintRadiusLocal * footprintRadiusLocal;
            const float maxDepthLocal = maxDepthGameUnits / objectScale;
            const RE::NiPoint3 gripLocal = transform_math::worldPointToLocal(objectNodeWorld, gripPointWorld);
            if (!grab_three_phase::isFinite(gripLocal)) {
                result.reason = "nonFiniteGripLocal";
                return result;
            }

            float bestDepthLocal = 0.0f;
            std::uint32_t footprintSampleCount = 0;
            auto considerLocalPoint = [&](const RE::NiPoint3& pointLocal) {
                const RE::NiPoint3 delta = pointLocal - gripLocal;
                const float depthLocal = dotProduct(delta, inwardLocal);
                if (!std::isfinite(depthLocal) || depthLocal <= 0.0f) {
                    return;
                }
                const RE::NiPoint3 lateral = delta - inwardLocal * depthLocal;
                const float lateralSquared = lengthSquared(lateral);
                if (!std::isfinite(lateralSquared) || lateralSquared > footprintRadiusLocalSquared) {
                    return;
                }
                ++footprintSampleCount;
                bestDepthLocal = (std::max)(bestDepthLocal, (std::min)(depthLocal, maxDepthLocal));
            };
            const std::array<float, 3> axisProbeDepthsLocal{ 0.0f, maxDepthLocal * 0.5f, maxDepthLocal };
            for (const auto& localTriangle : localTriangles) {
                considerLocalPoint(localTriangle.v0);
                considerLocalPoint(localTriangle.v1);
                considerLocalPoint(localTriangle.v2);
                const TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
                for (const float axisDepthLocal : axisProbeDepthsLocal) {
                    const RE::NiPoint3 axisPointLocal = gripLocal + inwardLocal * axisDepthLocal;
                    float distanceSquared = 0.0f;
                    considerLocalPoint(closestPointOnTriangleToPoint(axisPointLocal, triangle, distanceSquared));
                }
            }

            result.depthGameUnits = bestDepthLocal * objectScale;
            result.footprintSampleCount = footprintSampleCount;
            result.reason = footprintSampleCount > 0 ? "meshSupportDepth" : "noMeshInsideFootprint";
            result.valid = true;
            return result;
        }

        bool solveRuntimePinchClosure(RuntimePinchPocketCandidate& candidate,
            const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& objectWorld, const RE::NiTransform& handWorld,
            bool isLeft, const Hand& hand)
        {
            namespace pinch = grab_pinch_pocket_policy;
            const auto config = currentPinchPocketConfig();
            constexpr auto tipSegment = static_cast<std::size_t>(hand_collider_semantics::HandFingerSegment::Tip);
            const auto& colliders = hand.dynamicTwinTargets();
            const float maxDepth = candidate.meshExtents.maxExtentGameUnits * 1.73205081f;
            const char* failureStage = "mesh-span";
            auto measure = [&](pinch::ClosureSample sample) {
                if (!sample.fingers.valid) return pinch::ClosureSample{};
                constexpr float footprintRadius = 2.5f;
                const auto towardIndex = computeGrabSeatDepthStop(localTriangles, objectWorld,
                    candidate.surfaceHit.position, sample.fingers.axis * -1.0f, footprintRadius, maxDepth);
                const auto towardThumb = computeGrabSeatDepthStop(localTriangles, objectWorld,
                    candidate.surfaceHit.position, sample.fingers.axis, footprintRadius, maxDepth);
                if (!towardIndex.valid || !towardThumb.valid) {
                    failureStage = "mesh-span";
                    return pinch::ClosureSample{};
                }
                sample.thicknessGameUnits = towardIndex.depthGameUnits + towardThumb.depthGameUnits;
                sample.centerOffsetGameUnits = (towardIndex.depthGameUnits - towardThumb.depthGameUnits) * 0.5f;
                return sample;
            };
            auto evaluate = [&](float opening) {
                pinch::ClosureSample sample{};
                auto poseConfig = config;
                poseConfig.thumbIndexMaxOpenValue = opening;
                sample.pose = pinch::buildStablePinchFingerPose(poseConfig, g_rockConfig.rockGrabFingerMinValue);
                const auto handPose = frik_visual_authority::makeHandPoseDataFromJointValues(sample.pose.jointValues.data());
                frik_visual_authority::FingerLocalTransformOverride locals{};
                if (!frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(isLeft), handPose, &locals) ||
                    (locals.enabledMask & 0x003Fu) != 0x003Fu) {
                    failureStage = "provider-pose";
                    return sample;
                }
                std::array<RE::NiPoint3, 2> tips{};
                for (std::size_t finger = 0; finger < tips.size(); ++finger) {
                    RE::NiTransform joint = handWorld;
                    hand_bone_collider_geometry_math::BoneColliderFrameInput<RE::NiTransform, RE::NiPoint3> tipInput{};
                    tipInput.extrapolateFromPrevious = true;
                    tipInput.extrapolateAlongStartBoneAxis = true;
                    for (std::size_t segment = 0; segment < 3; ++segment) {
                        const auto& local = locals.localTransforms[finger * 3 + segment];
                        if (!grab_finger_pose_runtime::isFiniteTransformForFingerPadProbe(local)) {
                            failureStage = "joint-transform";
                            return pinch::ClosureSample{};
                        }
                        joint = transform_math::composeTransforms(joint, local);
                        if (segment == 1) tipInput.previous = joint;
                        if (segment == 2) tipInput.start = joint;
                    }
                    const auto tipFrame = hand_bone_collider_geometry_math::buildSegmentColliderFrame(tipInput);
                    const auto& collider = colliders.fingers[finger][tipSegment];
                    if (!tipFrame.valid || !collider.valid || !pinch::colliderTipEndpoint(tipFrame.transform.translate,
                            tipFrame.xAxis, collider.length, collider.convexRadius, tips[finger])) {
                        failureStage = "tip-collider";
                        return pinch::ClosureSample{};
                    }
                }
                sample.fingers = pinch::makeFingerFrame(tips[0], tips[1]);
                return measure(sample);
            };
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
                candidate.closure = pinch::solveClosure(g_rockConfig.rockGrabFingerMinValue,
                    config.thumbIndexMaxOpenValue, evaluate);
            } else {
                // With automatic finger posing disabled, only the live tips
                // can define the seat; no uncommanded closing pose is predicted.
                pinch::ClosureSample live{};
                live.fingers = pinch::makeFingerFrame(candidate.thumbPadWorld, candidate.indexPadWorld);
                candidate.closure.sample = measure(live);
                candidate.closure.valid = candidate.closure.sample.fingers.valid;
                candidate.closure.bracketed = true;
            }
            if (!candidate.closure.valid) {
                ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} pinch closure unavailable: stage={}", isLeft ? "Left" : "Right", failureStage);
                return false;
            }
            const auto& solved = candidate.closure.sample;
            candidate.thumbPadWorld = solved.fingers.thumbTip;
            candidate.indexPadWorld = solved.fingers.indexTip;
            candidate.pinchPocketWorld = solved.fingers.center;
            candidate.pinchAxisWorld = solved.fingers.axis;
            candidate.thumbIndexGapGameUnits = solved.fingers.gapGameUnits;
            candidate.pinchDetectionDirectionWorld = pinch::detectionDirection(solved.fingers,
                transformHandspaceDirection(handWorld, config.detectionDirectionHandspace, isLeft), config.detectionAxisBlend);
            if (!candidate.closure.bracketed) {
                ROCK_LOG_SAMPLE_WARN(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} pinch closure reached curl limit: opening={:.3f} range={:.3f}/{:.3f} gap={:.3f} thickness={:.3f} error={:.3f}",
                    isLeft ? "Left" : "Right", solved.opening, g_rockConfig.rockGrabFingerMinValue,
                    config.thumbIndexMaxOpenValue, solved.fingers.gapGameUnits, solved.thicknessGameUnits, candidate.closure.gapErrorGameUnits);
            }
            return true;
        }

        struct GrabMeshLongAxisResult
        {
            RE::NiPoint3 axisWorld{};
            RE::NiPoint3 secondAxisWorld{};
            float elongationRatio = 0.0f;
            float secondElongationRatio = 0.0f;
            std::uint32_t triangleCount = 0;
            const char* reason = "notEvaluated";
            bool valid = false;
        };

        /*
         * Principal axis of the rendered mesh via area-weighted PCA over the
         * extracted world-space triangles. NIF axes are not authored
         * consistently across props, so long-object orientation must come from
         * the geometry itself. elongationRatio = sqrt(lambda1/lambda2), the RMS
         * extent ratio between the dominant and second axis: ~1 for compact
         * objects, >2 for bottle/broom shapes. secondAxisWorld is the second
         * principal axis and secondElongationRatio = sqrt(lambda2/lambda3)
         * (lambda3 recovered from the covariance trace): ~1 for round cross
         * sections where roll about the long axis is meaningless, high for
         * plank/board shapes with a well-defined flat face. Both axis signs
         * are arbitrary; callers must align to the nearest hemisphere of
         * their target axis. Double accumulators because world coordinates
         * are large; covariance is built about the area-weighted mean.
         */
        GrabMeshLongAxisResult computeGrabMeshLongAxis(const std::vector<TriangleData>& worldTriangles)
        {
            GrabMeshLongAxisResult result{};
            result.triangleCount = static_cast<std::uint32_t>(worldTriangles.size());
            if (worldTriangles.empty()) {
                result.reason = "noTriangles";
                return result;
            }

            double weightSum = 0.0;
            double meanAccum[3] = {};
            auto isFinitePoint = [](const RE::NiPoint3& p) {
                return vector_math::hasFiniteComponents(p);
            };
            auto triangleArea = [](const TriangleData& tri) {
                const RE::NiPoint3 e0 = tri.v1 - tri.v0;
                const RE::NiPoint3 e1 = tri.v2 - tri.v0;
                const RE::NiPoint3 n = vector_math::cross(e0, e1);
                return 0.5f * std::sqrt((std::max)(0.0f, vector_math::lengthSquared(n)));
            };
            for (const auto& tri : worldTriangles) {
                if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                    continue;
                }
                const float area = triangleArea(tri);
                if (!std::isfinite(area) || area <= 0.000001f) {
                    continue;
                }
                weightSum += area;
                meanAccum[0] += static_cast<double>(area) * (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0;
                meanAccum[1] += static_cast<double>(area) * (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0;
                meanAccum[2] += static_cast<double>(area) * (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
            }
            if (weightSum <= 0.000001) {
                result.reason = "degenerateMeshArea";
                return result;
            }
            const double mean[3] = { meanAccum[0] / weightSum, meanAccum[1] / weightSum, meanAccum[2] / weightSum };

            // Symmetric covariance: [xx, xy, xz, yy, yz, zz]
            double cov[6] = {};
            for (const auto& tri : worldTriangles) {
                if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                    continue;
                }
                const float area = triangleArea(tri);
                if (!std::isfinite(area) || area <= 0.000001f) {
                    continue;
                }
                const double vertexWeight = static_cast<double>(area) / 3.0;
                const RE::NiPoint3* vertices[3] = { &tri.v0, &tri.v1, &tri.v2 };
                for (const auto* vertex : vertices) {
                    const double d[3] = { vertex->x - mean[0], vertex->y - mean[1], vertex->z - mean[2] };
                    cov[0] += vertexWeight * d[0] * d[0];
                    cov[1] += vertexWeight * d[0] * d[1];
                    cov[2] += vertexWeight * d[0] * d[2];
                    cov[3] += vertexWeight * d[1] * d[1];
                    cov[4] += vertexWeight * d[1] * d[2];
                    cov[5] += vertexWeight * d[2] * d[2];
                }
            }
            for (double& entry : cov) {
                entry /= weightSum;
            }

            auto covMultiply = [](const double m[6], const double v[3], double out[3]) {
                out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
                out[1] = m[1] * v[0] + m[3] * v[1] + m[4] * v[2];
                out[2] = m[2] * v[0] + m[4] * v[1] + m[5] * v[2];
            };
            auto dominantEigen = [&covMultiply](const double m[6], double outAxis[3]) {
                // Deterministic start: the coordinate axis with the largest diagonal.
                double v[3] = {};
                if (m[0] >= m[3] && m[0] >= m[5]) {
                    v[0] = 1.0;
                } else if (m[3] >= m[5]) {
                    v[1] = 1.0;
                } else {
                    v[2] = 1.0;
                }
                for (int iteration = 0; iteration < 48; ++iteration) {
                    double next[3];
                    covMultiply(m, v, next);
                    const double lenSq = next[0] * next[0] + next[1] * next[1] + next[2] * next[2];
                    if (!(lenSq > 1e-18)) {
                        break;
                    }
                    const double invLen = 1.0 / std::sqrt(lenSq);
                    v[0] = next[0] * invLen;
                    v[1] = next[1] * invLen;
                    v[2] = next[2] * invLen;
                }
                double mv[3];
                covMultiply(m, v, mv);
                const double eigenvalue = mv[0] * v[0] + mv[1] * v[1] + mv[2] * v[2];
                outAxis[0] = v[0];
                outAxis[1] = v[1];
                outAxis[2] = v[2];
                return eigenvalue;
            };

            double axis1[3];
            const double lambda1 = dominantEigen(cov, axis1);
            if (!(lambda1 > 1e-9)) {
                result.reason = "degenerateCovariance";
                return result;
            }
            double deflated[6] = {
                cov[0] - lambda1 * axis1[0] * axis1[0],
                cov[1] - lambda1 * axis1[0] * axis1[1],
                cov[2] - lambda1 * axis1[0] * axis1[2],
                cov[3] - lambda1 * axis1[1] * axis1[1],
                cov[4] - lambda1 * axis1[1] * axis1[2],
                cov[5] - lambda1 * axis1[2] * axis1[2],
            };
            double axis2[3];
            const double lambda2 = (std::max)(0.0, dominantEigen(deflated, axis2));

            result.axisWorld = RE::NiPoint3{
                static_cast<float>(axis1[0]),
                static_cast<float>(axis1[1]),
                static_cast<float>(axis1[2]),
            };
            result.elongationRatio = static_cast<float>((std::min)(100.0, std::sqrt(lambda1 / (std::max)(lambda2, lambda1 * 1e-4))));
            if (lambda2 > 1e-9) {
                // Re-orthogonalize against axis1: deflation leaves numerical drift.
                const double axis12dot = axis2[0] * axis1[0] + axis2[1] * axis1[1] + axis2[2] * axis1[2];
                double axis2Ortho[3] = {
                    axis2[0] - axis12dot * axis1[0],
                    axis2[1] - axis12dot * axis1[1],
                    axis2[2] - axis12dot * axis1[2],
                };
                const double axis2LenSq =
                    axis2Ortho[0] * axis2Ortho[0] + axis2Ortho[1] * axis2Ortho[1] + axis2Ortho[2] * axis2Ortho[2];
                if (axis2LenSq > 1e-12) {
                    const double invLen = 1.0 / std::sqrt(axis2LenSq);
                    result.secondAxisWorld = RE::NiPoint3{
                        static_cast<float>(axis2Ortho[0] * invLen),
                        static_cast<float>(axis2Ortho[1] * invLen),
                        static_cast<float>(axis2Ortho[2] * invLen),
                    };
                    // Eigenvalues of a symmetric matrix sum to its trace, so
                    // lambda3 needs no third power iteration.
                    const double trace = cov[0] + cov[3] + cov[5];
                    const double lambda3 = (std::max)(0.0, trace - lambda1 - lambda2);
                    result.secondElongationRatio =
                        static_cast<float>((std::min)(100.0, std::sqrt(lambda2 / (std::max)(lambda3, lambda2 * 1e-4))));
                }
            }
            result.reason = "meshPrincipalAxis";
            result.valid = true;
            return result;
        }

        /*
         * World-side rigid rotation of an NiTransform about a world pivot
         * point. Stored NiMatrix3 rows are the world images of the local axes
         * (hand_frame::transformHandspaceLocalToWorld documents the engine
         * convention), so a world rotation applies vector Rodrigues to each
         * stored row and to the pivot-relative translation. The pivot point
         * itself is the fixed point: worldPointToLocal(rotated, pivotWorld)
         * equals worldPointToLocal(original, pivotWorld).
         */
        RE::NiTransform rotateTransformWorldAboutPoint(
            const RE::NiTransform& transform,
            const RE::NiPoint3& unitAxisWorld,
            float angleRadians,
            const RE::NiPoint3& pivotWorld)
        {
            RE::NiTransform rotated = transform;
            for (int row = 0; row < 3; ++row) {
                const RE::NiPoint3 rowWorld{
                    transform.rotate.entry[row][0],
                    transform.rotate.entry[row][1],
                    transform.rotate.entry[row][2],
                };
                const RE::NiPoint3 rotatedRow = grab_finger_pose_math::rotateAroundUnitAxis(rowWorld, unitAxisWorld, angleRadians);
                rotated.rotate.entry[row][0] = rotatedRow.x;
                rotated.rotate.entry[row][1] = rotatedRow.y;
                rotated.rotate.entry[row][2] = rotatedRow.z;
            }
            const RE::NiPoint3 pivotOffset = transform.translate - pivotWorld;
            const RE::NiPoint3 rotatedOffset = grab_finger_pose_math::rotateAroundUnitAxis(pivotOffset, unitAxisWorld, angleRadians);
            rotated.translate = pivotWorld + rotatedOffset;
            return rotated;
        }

        void setHeldVelocity(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const RE::NiPoint3& linearVelocity,
            const RE::NiPoint3& angularVelocity,
            bool overrideAngularVelocity,
            float angularVelocityKeep = 1.0f,
            bool includeConnectedLinearVelocity = true,
            bool includeConnectedAngularVelocity = true)
        {
            if (!world) {
                return;
            }

            constexpr std::size_t kMaxVelocityMotionSlots = 96;
            std::array<std::uint32_t, kMaxVelocityMotionSlots> updatedMotionSlots{};
            std::size_t updatedMotionSlotCount = 0;

            auto alreadyUpdated = [&updatedMotionSlots, &updatedMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < updatedMotionSlotCount; ++i) {
                    if (updatedMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto setBody = [&](std::uint32_t bodyId, bool primaryBody) {
                if (bodyId == INVALID_BODY_ID) {
                    return;
                }

                auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
                if (!body) {
                    return;
                }

                const std::uint32_t motionIndex = body->motionIndex;
                if (!body_frame::hasUsableMotionIndex(motionIndex) || alreadyUpdated(motionIndex)) {
                    return;
                }

                if (updatedMotionSlotCount >= updatedMotionSlots.size()) {
                    return;
                }

                auto* motion = havok_runtime::getMotion(world, motionIndex);
                if (!motion) {
                    return;
                }

                updatedMotionSlots[updatedMotionSlotCount++] = motionIndex;
                const bool applyLinearForBody = primaryBody || includeConnectedLinearVelocity;
                const float angularKeep = std::clamp(std::isfinite(angularVelocityKeep) ? angularVelocityKeep : 1.0f, 0.0f, 1.0f);
                const bool overrideAngularForBody = overrideAngularVelocity && (primaryBody || includeConnectedAngularVelocity);
                const RE::hkVector4f linearHavok = applyLinearForBody ?
                    RE::hkVector4f{ linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f } :
                    RE::hkVector4f{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z, 0.0f };
                const RE::hkVector4f angularHavok = overrideAngularForBody ?
                    RE::hkVector4f{ angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f } :
                    RE::hkVector4f{ motion->angularVelocity.x * angularKeep, motion->angularVelocity.y * angularKeep, motion->angularVelocity.z * angularKeep, 0.0f };
                havok_runtime::setBodyVelocityDeferred(world,
                    bodyId,
                    linearHavok,
                    angularHavok);
            };

            setBody(primaryBodyId.value, true);
            if (includeConnectedLinearVelocity || includeConnectedAngularVelocity) {
                for (const auto bodyId : heldBodyIds) {
                    setBody(bodyId, false);
                }
            }
        }

        void setHeldLinearVelocity(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const RE::NiPoint3& linearVelocity,
            float angularVelocityKeep = 1.0f,
            bool includeConnectedBodies = true)
        {
            setHeldVelocity(world, primaryBodyId, heldBodyIds, linearVelocity, RE::NiPoint3{}, false, angularVelocityKeep, includeConnectedBodies, false);
        }

        float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            if (!world || bodyId.value == INVALID_BODY_ID) {
                return 0.0f;
            }

            auto* motion = havok_runtime::getBodyMotion(world, bodyId);
            if (!motion) {
                return 0.0f;
            }

            const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
            if (packedInvMass == 0) {
                return 0.0f;
            }

            return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
        }

        struct HeldBodyMassSummary
        {
            float primaryMass = 0.0f;
            float aggregateMass = 0.0f;
            std::uint32_t sampledBodies = 0;
            std::uint32_t uniqueMotions = 0;

            [[nodiscard]] float motorMass() const noexcept
            {
                if (std::isfinite(aggregateMass) && aggregateMass > 0.0f) {
                    return aggregateMass;
                }
                return (std::isfinite(primaryMass) && primaryMass > 0.0f) ? primaryMass : 0.0f;
            }
        };

        HeldBodyMassSummary readHeldBodyMassSummary(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            bool includeConnectedBodies = true)
        {
            /*
             * Dynamic grab owns one held object even when FO4VR exposes that
             * object as several hknp bodies. Lifecycle, inertia normalization,
             * release velocity, and nearby damping already operate on the whole
             * accepted body set. The motor mass budget must use the same object
             * scope, with unique-motion dedupe, so multipart loose weapons are
             * not budgeted from whichever child body happened to be selected.
             */
            HeldBodyMassSummary summary{};
            summary.primaryMass = readBodyMass(world, primaryBodyId);
            if (!world) {
                summary.aggregateMass = summary.primaryMass;
                return summary;
            }

            constexpr std::size_t kMaxMassMotionSlots = 96;
            std::array<std::uint32_t, kMaxMassMotionSlots> sampledMotionSlots{};
            std::size_t sampledMotionSlotCount = 0;

            auto motionAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                    if (sampledMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto sampleBody = [&](std::uint32_t rawBodyId) {
                if (rawBodyId == INVALID_BODY_ID) {
                    return;
                }

                auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ rawBodyId });
                if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) || motionAlreadySampled(body->motionIndex)) {
                    return;
                }
                if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                    return;
                }

                const float mass = readBodyMass(world, RE::hknpBodyId{ rawBodyId });
                if (!std::isfinite(mass) || mass <= 0.0f) {
                    return;
                }

                sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
                summary.aggregateMass += mass;
                ++summary.sampledBodies;
                summary.uniqueMotions = static_cast<std::uint32_t>(sampledMotionSlotCount);
            };

            sampleBody(primaryBodyId.value);
            if (includeConnectedBodies) {
                for (const auto bodyId : heldBodyIds) {
                    sampleBody(bodyId);
                }
            }

            if (!(std::isfinite(summary.aggregateMass) && summary.aggregateMass > 0.0f)) {
                summary.aggregateMass = summary.primaryMass;
            }
            return summary;
        }

        held_object_contact_policy::HeldContactOtherMotion classifyHeldContactOtherMotion(RE::hknpWorld* world, std::uint32_t bodyId)
        {
            if (!world || bodyId == INVALID_BODY_ID) {
                return held_object_contact_policy::HeldContactOtherMotion::Unknown;
            }

            auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
            if (!motion) {
                return held_object_contact_policy::HeldContactOtherMotion::Unknown;
            }

            const float mass = readBodyMass(world, RE::hknpBodyId{ bodyId });
            return mass > 0.0f ?
                held_object_contact_policy::HeldContactOtherMotion::Dynamic :
                held_object_contact_policy::HeldContactOtherMotion::FixedOrStatic;
        }

        struct FingerPadPublishDebug
        {
            std::array<RE::NiPoint3, 5> padProbeStart{};
            std::array<RE::NiPoint3, 5> padProbeEnd{};
            std::array<RE::NiPoint3, 5> padProbeHit{};
            std::array<std::uint8_t, 5> padProbeHitValid{};
            bool hasPadProbeDebug = false;
            std::array<RE::NiPoint3, 5> surfaceTarget{};
            std::array<std::uint8_t, 5> surfaceTargetValid{};
            bool hasSurfaceTargetDebug = false;
        };

        FingerPadPublishDebug makeFingerPadPublishDebug(
            const grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5>& padEvidence)
        {
            FingerPadPublishDebug debug{};
            for (std::size_t finger = 0; finger < padEvidence.size(); ++finger) {
                const auto& evidence = padEvidence[finger];
                debug.padProbeStart[finger] = evidence.startWorld;
                debug.padProbeEnd[finger] = evidence.endWorld;
                debug.padProbeHit[finger] = evidence.hitPointWorld;
                debug.padProbeHitValid[finger] = evidence.hit ? 1 : 0;
                debug.hasPadProbeDebug = debug.hasPadProbeDebug || grab_finger_pose_runtime::hasFingerPadProbeLine(evidence);

                if (pose.surfaceAimTargetValid[finger]) {
                    debug.surfaceTarget[finger] = pose.surfaceAimTarget[finger];
                    debug.surfaceTargetValid[finger] = 1;
                    debug.hasSurfaceTargetDebug = true;
                }
            }
            return debug;
        }

        void applyRockGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose, std::array<float, 15>& currentJointPose,
            bool& hasCurrentJointPose, std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask, bool& hasCurrentLocalTransforms,
            float deltaTime, bool publishLocalTransforms = true, bool snapJointPoseToTarget = false)
        {
            if (!frik_visual_authority::isAvailable()) {
                return;
            }

            const auto hand = handFromBool(isLeft);
            grab_finger_local_transform_runtime::State localTransformState{
                .currentTransforms = currentLocalTransforms,
                .currentMask = currentLocalTransformMask,
                .hasCurrentTransforms = hasCurrentLocalTransforms,
            };
            auto syncLocalTransformState = [&]() {
                currentLocalTransforms = localTransformState.currentTransforms;
                currentLocalTransformMask = localTransformState.currentMask;
                hasCurrentLocalTransforms = localTransformState.hasCurrentTransforms;
            };

            if (!g_rockConfig.rockGrabMeshFingerPoseEnabled) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                syncLocalTransformState();
                (void)frik_visual_authority::clearHandPose("ROCK_Grab", hand);
                return;
            }

            if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved) {
                const auto targetJointPose = fingerPose.hasJointValues ? fingerPose.jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(fingerPose.values);
                if (!hasCurrentJointPose || snapJointPoseToTarget) {
                    currentJointPose = targetJointPose;
                    hasCurrentJointPose = true;
                } else {
                    currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose, g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
                }

                std::array<float, 5> currentSplayRadians{};
                (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
                const auto currentHandPose = frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose, currentSplayRadians);
                if (!frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, currentHandPose, 100)) {
                    hasCurrentJointPose = false;
                    grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                    syncLocalTransformState();
                    return;
                }
                (void)grab_finger_local_transform_runtime::publishLocalTransformPose("ROCK_Grab",
                    hand,
                    isLeft,
                    fingerPose,
                    currentHandPose,
                    grab_finger_local_transform_runtime::Options{
                        .enabled = publishLocalTransforms && g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                        .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                        .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                        .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                        .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                        .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                        .thumbSurfaceSafetyEnabled = g_rockConfig.rockGrabThumbSurfaceSafetyEnabled,
                        .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
                    },
                    deltaTime,
                    100,
                    localTransformState);
                syncLocalTransformState();
                return;
            }

            if (fingerPose.solved) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                syncLocalTransformState();
                std::array<float, 5> currentSplayRadians{};
                (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
                const auto handPose = frik_visual_authority::makeUniformHandPoseData(
                    fingerPose.values[0],
                    fingerPose.values[1],
                    fingerPose.values[2],
                    fingerPose.values[3],
                    fingerPose.values[4],
                    currentSplayRadians);
                (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, handPose, 100);
                return;
            }

            hasCurrentJointPose = false;
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
            syncLocalTransformState();
            const float fallbackMin =
                std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
            const float configuredFallback =
                std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
            const float fallbackValue = std::clamp(configuredFallback, fallbackMin, 1.0f);
            const auto fallbackPose = frik_visual_authority::makeUniformHandPoseData(
                fallbackValue,
                fallbackValue,
                fallbackValue,
                fallbackValue,
                fallbackValue);
            (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, fallbackPose, 100);
        }

        grab_finger_pose_runtime::SolvedGrabFingerPose buildAcquisitionFingerPose(
            const grab_finger_pose_runtime::SolvedGrabFingerPose& targetPose,
            float progress)
        {
            auto pose = targetPose;
            const float t = std::clamp(std::isfinite(progress) ? progress : 0.0f, 0.0f, 1.0f);
            const float precloseValue =
                std::clamp(std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f, 0.0f, 1.0f);

            for (std::size_t finger = 0; finger < pose.values.size(); ++finger) {
                const float solvedValue =
                    targetPose.solved ? std::clamp(targetPose.values[finger], 0.0f, 1.0f) : precloseValue;
                const float closingTarget = (std::min)(precloseValue, solvedValue);
                pose.values[finger] = precloseValue + (closingTarget - precloseValue) * t;
            }

            if (targetPose.solved && targetPose.hasJointValues) {
                std::array<float, 5> precloseValues{ precloseValue, precloseValue, precloseValue, precloseValue, precloseValue };
                const auto precloseJoints = grab_finger_pose_math::expandFingerCurlsToJointValues(precloseValues);
                for (std::size_t joint = 0; joint < pose.jointValues.size(); ++joint) {
                    const float targetJoint = std::clamp(targetPose.jointValues[joint], 0.0f, 1.0f);
                    pose.jointValues[joint] = precloseJoints[joint] + (targetJoint - precloseJoints[joint]) * t;
                }
            } else {
                pose.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(pose.values);
            }
            pose.solved = true;
            pose.hasJointValues = true;
            return pose;
        }

        constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";
        constexpr int GRAB_EXTERNAL_HAND_PRIORITY = 90;
        constexpr const char* GRAB_RETURN_HAND_TAG = "ROCK_GrabReturn";
        constexpr int GRAB_RETURN_HAND_PRIORITY = 85;

        bool applyGrabExternalHandWorldTransform(bool isLeft, const RE::NiTransform& adjustedHandTransform)
        {
            return frik_visual_authority::publishHandWorld(
                GRAB_EXTERNAL_HAND_TAG,
                handFromBool(isLeft),
                adjustedHandTransform,
                GRAB_EXTERNAL_HAND_PRIORITY);
        }

        void clearGrabExternalHandWorldTransform(bool isLeft)
        {
            (void)frik_visual_authority::clearHandWorld(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
        }

        bool applyGrabReturnHandWorldTransform(bool isLeft, const RE::NiTransform& handTransform)
        {
            return frik_visual_authority::publishHandWorld(
                GRAB_RETURN_HAND_TAG,
                handFromBool(isLeft),
                handTransform,
                GRAB_RETURN_HAND_PRIORITY);
        }

        void clearGrabReturnHandWorldTransform(bool isLeft)
        {
            (void)frik_visual_authority::clearHandWorld(GRAB_RETURN_HAND_TAG, handFromBool(isLeft));
        }

        bool isUsableGrabVisualTransform(const RE::NiTransform& transform)
        {
            if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
                !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale) ||
                std::abs(transform.scale) <= 0.0001f) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(transform.rotate.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        void logRuntimeScaleIfChanged(bool isLeft, const char* handName, const RE::NiTransform& handWorldTransform, const RE::NiAVObject* collidableNode)
        {
            struct RuntimeScaleLogState
            {
                bool initialized = false;
                float handScale = 0.0f;
                float collidableScale = 0.0f;
                float vrScale = 0.0f;
            };

            static std::array<RuntimeScaleLogState, 2> s_scaleStates{};
            auto& state = s_scaleStates[isLeft ? 1 : 0];
            auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
            const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;
            const float collidableScale = collidableNode ? collidableNode->world.scale : -1.0f;
            auto changedEnough = [](float a, float b) {
                return std::fabs(a - b) > 0.01f;
            };

            if (!state.initialized || changedEnough(state.handScale, handWorldTransform.scale) || changedEnough(state.collidableScale, collidableScale) ||
                changedEnough(state.vrScale, vrScale)) {
                ROCK_LOG_DEBUG(Hand,
                    "Runtime scale {}: handScale={:.3f} collidableScale={:.3f} vrScale={:.3f} previous=({:.3f},{:.3f},{:.3f})",
                    handName,
                    handWorldTransform.scale,
                    collidableScale,
                    vrScale,
                    state.handScale,
                    state.collidableScale,
                    state.vrScale);
                state.initialized = true;
                state.handScale = handWorldTransform.scale;
                state.collidableScale = collidableScale;
                state.vrScale = vrScale;
            }
        }
    }

    namespace hand_grab_internal
    {
        RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value) { return ::rock::normalizeOrZero(value); }
        RE::NiPoint3 stablePerpendicularAxis(const RE::NiPoint3& normal) { return ::rock::stablePerpendicularAxis(normal); }
        float vectorMagnitude(const RE::NiPoint3& value) { return ::rock::vectorMagnitude(value); }
        float pointDistanceGameUnits(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) { return ::rock::pointDistanceGameUnits(lhs, rhs); }
        float translationDeltaGameUnits(const RE::NiTransform& lhs, const RE::NiTransform& rhs) { return ::rock::translationDeltaGameUnits(lhs, rhs); }
        float rotationDeltaDegrees(const RE::NiMatrix3& lhs, const RE::NiMatrix3& rhs) { return ::rock::rotationDeltaDegrees(lhs, rhs); }
        RE::NiPoint3 rotationCorrectionAxisWorld(const RE::NiMatrix3& current, const RE::NiMatrix3& target)
        {
            return ::rock::rotationCorrectionAxisWorld(current, target);
        }

        RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix) { return ::rock::matrixFromHkColumns(hkMatrix); }
        RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix) { return ::rock::matrixFromHkRows(hkMatrix); }
        RE::NiTransform makeIdentityTransform() { return ::rock::makeIdentityTransform(); }
        RE::NiTransform invertTransform(const RE::NiTransform& transform) { return ::rock::invertTransform(transform); }
        RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child)
        {
            return ::rock::multiplyTransforms(parent, child);
        }
        RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld, const RE::NiTransform& bodyLocalTransform)
        {
            return ::rock::deriveNodeWorldFromBodyWorld(bodyWorld, bodyLocalTransform);
        }
        RE::NiTransform reconstructBodyWorldFromProxyInBody(
            const RE::NiTransform& proxyWorld,
            const RE::NiMatrix3& proxyInBodyRotation,
            const RE::NiPoint3& transformBLocalGame,
            const RE::NiPoint3& pivotAProxyLocalGame)
        {
            return ::rock::reconstructBodyWorldFromProxyInBody(
                proxyWorld,
                proxyInBodyRotation,
                transformBLocalGame,
                pivotAProxyLocalGame);
        }
        RE::NiTransform reconstructSolverEffectiveBodyWorld(
            const RE::NiTransform& proxyWorld,
            const RE::NiMatrix3& transformARotation,
            const RE::NiMatrix3& transformBRotation,
            const RE::NiMatrix3& targetBRcaRotation,
            const RE::NiPoint3& transformBLocalGame,
            const RE::NiPoint3& anchorAWorld,
            float bodyScale)
        {
            return ::rock::reconstructSolverEffectiveBodyWorld(
                proxyWorld,
                transformARotation,
                transformBRotation,
                targetBRcaRotation,
                transformBLocalGame,
                anchorAWorld,
                bodyScale);
        }

        bool tryGetGrabAuthorityBodyWorldTransform(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            RE::NiTransform& outTransform)
        {
            return ::rock::tryGetGrabAuthorityBodyWorldTransform(world, bodyId, outTransform);
        }
        RE::NiTransform gripEvidenceWorldFrame(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
        {
            return ::rock::gripEvidenceWorldFrame(frame, fallbackWorld);
        }
        RE::NiTransform gripEvidenceWorldFrame(const ImmutableGrabCaptureTelemetry& capture, const RE::NiTransform& fallbackWorld)
        {
            return ::rock::gripEvidenceWorldFrame(capture, fallbackWorld);
        }
        RE::NiPoint3 gripEvidencePointWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
        {
            return ::rock::gripEvidencePointWorld(frame, fallbackWorld);
        }
        RE::NiPoint3 gripEvidenceNormalWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
        {
            return ::rock::gripEvidenceNormalWorld(frame, fallbackWorld);
        }
        HeldBodyMassSummary readHeldBodyMassSummary(
            RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            bool includeConnectedBodies)
        {
            const auto summary = ::rock::readHeldBodyMassSummary(
                world,
                primaryBodyId,
                heldBodyIds,
                includeConnectedBodies);
            return HeldBodyMassSummary{
                .primaryMass = summary.primaryMass,
                .aggregateMass = summary.aggregateMass,
                .sampledBodies = summary.sampledBodies,
                .uniqueMotions = summary.uniqueMotions,
            };
        }
    }

    void Hand::beginGrabVisualReturn()
    {
        const bool weaponReturn = isHoldingLooseWeapon();
        if (!(weaponReturn ? hand_visual_lerp_math::kEquippedWeaponReturnEnabled : g_rockConfig.rockGrabHandReturnEnabled) ||
            !_hasLastPublishedGrabVisualHandTransform ||
            !isUsableGrabVisualTransform(_lastPublishedGrabVisualHandTransform) ||
            !frik_visual_authority::isAvailable()) {
            clearGrabVisualReturn("release-not-eligible", false);
            return;
        }

        // A submitted IK target is not necessarily the rendered wrist (reach,
        // final arm solve and locomotion can move it). Release from what was
        // actually shown, before discarding the held-object authority.
        auto start = _lastPublishedGrabVisualHandTransform;
        RE::NiTransform presented{};
        const bool hasPresentedStart = weaponReturn &&
            frik_hand_world_authority::tryGetPresentedHandWorld(_isLeft, presented) && isUsableGrabVisualTransform(presented);
        if (hasPresentedStart) start = presented;
        _grabVisualReturnConfig = weaponReturn ? hand_visual_lerp_math::kEquippedWeaponReturnConfig :
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockGrabHandReturnTimeMin,
                .maxSeconds = g_rockConfig.rockGrabHandReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockGrabHandReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockGrabHandReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockGrabHandReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockGrabHandReturnMaxAngleDegrees,
            };
        _grabVisualReturn.begin(start);
        if (!applyGrabReturnHandWorldTransform(_isLeft, _grabVisualReturn.start)) {
            _grabVisualReturn.clear();
            clearGrabReturnHandWorldTransform(_isLeft);
            ROCK_LOG_WARN(Hand, "{} hand visual return start failed; restoring tracked authority immediately", handName());
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand visual return started from=({:.2f},{:.2f},{:.2f}) weapon={} source={} publishedGap={:.3f}gu/{:.2f}deg",
            handName(),
            _grabVisualReturn.start.translate.x,
            _grabVisualReturn.start.translate.y,
            _grabVisualReturn.start.translate.z, weaponReturn, hasPresentedStart ? "rendered" : "published",
            hand_visual_lerp_math::distanceGameUnits(start.translate, _lastPublishedGrabVisualHandTransform.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(start, _lastPublishedGrabVisualHandTransform));
    }

    void Hand::updateGrabVisualReturn(const RE::NiTransform& trackedHandWorld, float deltaTime)
    {
        if (!_grabVisualReturn.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() ||
            !frik_visual_authority::isAvailable() ||
            !isUsableGrabVisualTransform(trackedHandWorld)) {
            clearGrabVisualReturn("tracked-hand-unavailable", true);
            return;
        }

        const auto result = hand_visual_lerp_math::driveVisualReturn(
            _grabVisualReturn,
            trackedHandWorld,
            deltaTime,
            _grabVisualReturnConfig,
            [](const RE::NiTransform& transform) {
                return isUsableGrabVisualTransform(transform);
            },
            [this](const RE::NiTransform& transform) {
                return applyGrabReturnHandWorldTransform(_isLeft, transform);
            });
        if (result.timingInitializedThisFrame) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand visual return timing distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                handName(),
                result.initialDistanceGameUnits,
                result.initialAngleDegrees,
                result.durationSeconds);
        }
        if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::InvalidTransform ||
            result.status == hand_visual_lerp_math::VisualReturnDriveStatus::PublishFailed) {
            clearGrabVisualReturn("publish-failed", true);
            return;
        }

        if (result.status != hand_visual_lerp_math::VisualReturnDriveStatus::Completed) {
            return;
        }

        const float completedDuration = result.durationSeconds;
        clearGrabReturnHandWorldTransform(_isLeft);
        _grabVisualReturn.clear();
        ROCK_LOG_DEBUG(Hand, "{} hand visual return completed duration={:.3f}s", handName(), completedDuration);
    }

    void Hand::clearGrabVisualReturn(const char* reason, bool logCancellation)
    {
        const bool wasActive = _grabVisualReturn.active;
        clearGrabReturnHandWorldTransform(_isLeft);
        _grabVisualReturn.clear();
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Hand, "{} hand visual return cancelled reason={}", handName(), reason ? reason : "unknown");
        }
    }

    void Hand::cancelGrabVisualReturn(const char* reason)
    {
        clearGrabVisualReturn(reason, true);
    }

    static void nativeVRGrabDrop(void* playerChar, int handIndex)
    {
        typedef void func_t(void*, int, std::uint64_t);
        static REL::Relocation<func_t> func{ REL::Offset(offsets::kFunc_NativeVRGrabDrop) };
        func(playerChar, handIndex, 0);
    }

    void Hand::clearPullPrepTracking()
    {
        _pullActiveLifecycle.clear();
        _pullPrepHknpWorld = nullptr;
        _pullPrepRootNode = nullptr;
        _pullPrepRefr = nullptr;
        _pullPrepTargetKind = grab_target::Kind::LooseObject;
        _pullPrepOriginalMotionPropsId = 1;
        _pullPrepRestoreArmed = false;
    }

    void Hand::restorePullPrepIfActive(const char* context)
    {
        if (!_pullPrepRestoreArmed) {
            return;
        }

        const std::uint32_t primaryBodyId =
            _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : active_grab_body_lifecycle::kInvalidBodyId;

        restoreActiveGrabLifecycle(_pullPrepHknpWorld,
            _pullActiveLifecycle,
            _pullActiveLifecycle.restorePlanForFailure(),
            primaryBodyId,
            handName(),
            context ? context : "pull-prep-abandoned");
        if (_pullActiveLifecycle.hasIncompleteNativeScan()) {
            restoreIncompleteActivePrepRoot(
                _pullPrepRootNode,
                _pullPrepOriginalMotionPropsId,
                handName(),
                context ? context : "pull-prep-abandoned-incomplete-scan");
        }

        clearPullPrepTracking();
    }

    void Hand::finishPullPrepAsPhysicalDropIfActive(const char* context)
    {
        if (!_pullPrepRestoreArmed) {
            return;
        }

        const std::uint32_t primaryBodyId =
            _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : active_grab_body_lifecycle::kInvalidBodyId;
        const auto releaseRestorePolicy =
            active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(_pullPrepTargetKind);
        const auto releasePlan = _pullActiveLifecycle.restorePlanForRelease(
            releaseRestorePolicy,
            _pullPrepTargetKind,
            active_grab_body_lifecycle::BodyReleaseIntent::PhysicalDrop);

        restoreActiveGrabLifecycle(_pullPrepHknpWorld,
            _pullActiveLifecycle,
            releasePlan,
            primaryBodyId,
            handName(),
            context ? context : "pull-physical-drop");

        if (_pullActiveLifecycle.hasIncompleteNativeScan()) {
            if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan, _pullPrepOriginalMotionPropsId)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand {}: skipped recursive root restore for converted loose-object physical pull drop root='{}' motionProps={} preservedMotion={}",
                    handName(),
                    context ? context : "pull-physical-drop",
                    nodeDebugName(_pullPrepRootNode),
                    _pullPrepOriginalMotionPropsId,
                    releasePlan.preservedConvertedMotionCount);
            } else {
                restoreIncompleteActivePrepRoot(
                    _pullPrepRootNode,
                    _pullPrepOriginalMotionPropsId,
                    handName(),
                    context ? context : "pull-physical-drop-incomplete-scan");
            }
        }

        if (_pullPrepHknpWorld && primaryBodyId != INVALID_BODY_ID) {
            const auto releaseActivation = activateHeldObjectBodySet(_pullPrepHknpWorld, primaryBodyId, _pulledBodyIds);
            if (releaseActivation.failedActivationCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand PULL physical-drop activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                    handName(),
                    primaryBodyId,
                    releaseActivation.bodyCount,
                    releaseActivation.activatedCount,
                    releaseActivation.failedActivationCount);
            }
        }

        clearPullPrepTracking();
    }

    bool Hand::consumePullPrepLifecycleForActiveGrab(RE::TESObjectREFR* refr, active_grab_body_lifecycle::BodyLifecycleSnapshot& outLifecycle)
    {
        if (!_pullPrepRestoreArmed || !refr || refr != _pullPrepRefr) {
            return false;
        }

        outLifecycle = _pullActiveLifecycle;
        ROCK_LOG_DEBUG(Hand,
            "{} hand consumed pull prep lifecycle for held grab: formID={:08X} bodies={} incompleteScan={}",
            handName(),
            refr->GetFormID(),
            outLifecycle.size(),
            outLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
        clearPullPrepTracking();
        return true;
    }

    void Hand::clearGrabHandCollisionSuppressionState()
    {
        _grabHandCollisionSuppression.clearTracking();
    }

    void Hand::clearHeldLooseWeaponBodyCollisionSuppressionState()
    {
        _heldLooseWeaponBodyCollisionSuppression.clearTracking();
    }

    void Hand::suppressHandCollisionForGrab(RE::hknpWorld* world, const BodyBoneColliderSet* bodyBoneColliders)
    {
        /*
         * Normal held-object grabs suppress the grabbing hand's immediate collision
         * authority while the object is constrained to that hand. The generated
         * hand suite covers palm/fingers, and BodyBoneColliderSet owns the adjacent
         * same-side forearm/wrist chain. Leasing both sets prevents held objects
         * from solving against their own driving arm without touching the separate
         * two-handed equipped-weapon suppression path.
         */
        _grabHandCollisionSuppression.cancelDelayedRestore();

        if (!world || !hasCollisionBody())
            return;

        auto suppressBody = [&](std::uint32_t bodyId, const char* context) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }

            if (!_grabHandCollisionSuppression.contains(bodyId) &&
                _grabHandCollisionSuppression.full()) {
                ROCK_LOG_WARN(Hand,
                    "{} hand: grab collision suppression set full; bodyId={} context={} left active",
                    handName(),
                    bodyId,
                    context ? context : "unknown");
                return;
            }

            const auto registryResult = _grabHandCollisionSuppression.acquire(
                world,
                bodyId,
                context);

            if (registryResult.valid && (registryResult.filterChanged || registryResult.firstLeaseForBody)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: grab collision lease acquired bodyId={} context={} filter=0x{:08X}->0x{:08X} wasDisabledBeforeGrab={} leases={}",
                    handName(),
                    bodyId,
                    context ? context : "unknown",
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _boneColliders.getBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_boneColliders.getBodyIdAtomic(i), "held-grab-hand-suite");
            }
        } else {
            suppressBody(_handBody.getBodyId().value, "held-grab-hand-anchor");
        }

        if (bodyBoneColliders) {
            std::array<std::uint32_t, kGrabCollisionSuppressionArmBodyCountPerHand> armBodyIds{};
            const auto armBodyCount =
                bodyBoneColliders->copyGrabSuppressionArmBodyIdsAtomic(_isLeft, armBodyIds.data(), armBodyIds.size());
            for (std::uint32_t i = 0; i < armBodyCount && i < armBodyIds.size(); ++i) {
                suppressBody(armBodyIds[i], "held-grab-arm-chain");
            }
        }
    }

    void Hand::restoreHandCollisionAfterGrab(RE::hknpWorld* world)
    {
        if (_grabHandCollisionSuppression.empty())
            return;

        if (!world) {
            ROCK_LOG_WARN(Hand,
                "{} hand: cannot restore grab hand collision yet (world={}); preserving suppression state",
                handName(),
                static_cast<const void*>(world));
            return;
        }

        const bool restored = _grabHandCollisionSuppression.releaseAll(
            world,
            "held-grab-hand",
            [this](
                std::uint32_t bodyId,
                const collision_suppression_registry::RuntimeSuppressionResult& releaseResult) {
                if (releaseResult.readFailed) {
                    return;
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: grab hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                    handName(),
                    bodyId,
                    releaseResult.filterBefore,
                    releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            });
        if (!restored) {
            ROCK_LOG_WARN(Hand, "{} hand: grab hand collision restore deferred; suppression leases preserved", handName());
            return;
        }
        clearGrabHandCollisionSuppressionState();
    }

    void Hand::suppressBodyCollisionForHeldLooseWeapon(RE::hknpWorld* world, const BodyBoneColliderSet* bodyBoneColliders)
    {
        if (!world || !bodyBoneColliders || !bodyBoneColliders->hasBodies()) {
            return;
        }

        auto keepBodyColliderEnabled = [&](const BodyBoneColliderMetadata& metadata) {
            const auto otherSide = _isLeft ? body_zone::BodyZoneSide::Right : body_zone::BodyZoneSide::Left;
            if (metadata.side != otherSide) {
                return false;
            }
            return metadata.role == skeleton_bone_debug_math::BoneColliderRole::ForearmSegment ||
                   metadata.role == skeleton_bone_debug_math::BoneColliderRole::HandSegment;
        };

        auto roleName = [](skeleton_bone_debug_math::BoneColliderRole role) {
            using skeleton_bone_debug_math::BoneColliderRole;
            switch (role) {
            case BoneColliderRole::UpperArmSegment:
                return "UpperArmSegment";
            case BoneColliderRole::ForearmSegment:
                return "ForearmSegment";
            case BoneColliderRole::HandSegment:
                return "HandSegment";
            case BoneColliderRole::FingerSegment:
                return "FingerSegment";
            case BoneColliderRole::TorsoSegment:
                return "TorsoSegment";
            case BoneColliderRole::LegSegment:
                return "LegSegment";
            case BoneColliderRole::FootSegment:
                return "FootSegment";
            }
            return "Unknown";
        };

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }

            BodyBoneColliderMetadata metadata{};
            if (!bodyBoneColliders->tryGetBodyMetadataAtomic(bodyId, metadata) || keepBodyColliderEnabled(metadata)) {
                return;
            }

            if (!_heldLooseWeaponBodyCollisionSuppression.contains(bodyId) &&
                _heldLooseWeaponBodyCollisionSuppression.full()) {
                ROCK_LOG_WARN(Hand,
                    "{} hand: held loose weapon body suppression set full; bodyId={} role={} zone={} side={} left active",
                    handName(),
                    bodyId,
                    roleName(metadata.role),
                    body_zone::bodyZoneName(metadata.zone),
                    body_zone::bodyZoneSideName(metadata.side));
                return;
            }

            const auto registryResult = _heldLooseWeaponBodyCollisionSuppression.acquire(
                world,
                bodyId,
                "held-loose-weapon-body");

            if (registryResult.valid && (registryResult.filterChanged || registryResult.firstLeaseForBody)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: held loose weapon body collision lease acquired bodyId={} role={} zone={} side={} filter=0x{:08X}->0x{:08X} wasDisabledBefore={} leases={}",
                    handName(),
                    bodyId,
                    roleName(metadata.role),
                    body_zone::bodyZoneName(metadata.zone),
                    body_zone::bodyZoneSideName(metadata.side),
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t bodyCount = bodyBoneColliders->getBodyCount();
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            suppressBody(bodyBoneColliders->getBodyIdAtomic(i));
        }
    }

    void Hand::restoreBodyCollisionAfterHeldLooseWeapon(RE::hknpWorld* world)
    {
        if (_heldLooseWeaponBodyCollisionSuppression.empty()) {
            return;
        }

        if (!world) {
            ROCK_LOG_WARN(Hand,
                "{} hand: cannot restore held loose weapon body collision yet (world={}); preserving suppression state",
                handName(),
                static_cast<const void*>(world));
            return;
        }

        const bool restored = _heldLooseWeaponBodyCollisionSuppression.releaseAll(
            world,
            "held-loose-weapon-body",
            [this](
                std::uint32_t bodyId,
                const collision_suppression_registry::RuntimeSuppressionResult& releaseResult) {
                if (releaseResult.readFailed) {
                    return;
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: held loose weapon body collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                    handName(),
                    bodyId,
                    releaseResult.filterBefore,
                    releaseResult.filterAfter,
                    releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    releaseResult.bodyFullyReleased ? "yes" : "no");
            });

        if (!restored) {
            ROCK_LOG_WARN(Hand, "{} hand: held loose weapon body collision restore deferred; suppression leases preserved", handName());
            return;
        }

        clearHeldLooseWeaponBodyCollisionSuppressionState();
    }

    void Hand::updateDelayedGrabHandCollisionRestore(RE::hknpWorld* world, float deltaTime)
    {
        if (!_grabHandCollisionSuppression.advanceDelayedRestore(deltaTime)) {
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand: delayed grab hand collision restore ready bodies={} firstBodyId={} delayRemaining={:.3f}",
            handName(),
            _grabHandCollisionSuppression.size(),
            _grabHandCollisionSuppression.firstBodyId(),
            _grabHandCollisionSuppression.delayedRestoreRemainingSeconds());
        restoreHandCollisionAfterGrab(world);
    }

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

    RE::NiPoint3 Hand::activeProxyConstraintPivotBLocalGame() const
    {
        if (_grabAuthorityProxyFrameValid) {
            return _grabAuthorityPivotBConstraintLocalGame;
        }
        return _grabFrame.authority.pivotBConstraintLocalGame;
    }

    void Hand::clearGrabAuthorityProxyRuntimeLocked()
    {
        _consumerPairCount=0;
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

    bool Hand::getHeldBodyContactMesh(RE::hknpWorld* world,
        std::span<const GrabLocalTriangle>& triangles, RE::NiTransform& meshWorld, const HeldContactMeshCache*& cache) const
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
            grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorldTransform, _grabFrame.proxyAuthorityBodyHandSpace);
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorldTransform, desiredBodyWorldAtCreation);
        const RE::NiPoint3 relationPivotBConstraintLocalGame =
            grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformProxySpace, _grabFrame.authority.pivotAHandBodyLocalGame);
        const RE::NiPoint3 solverPivotBConstraintLocalGame = _grabFrame.authority.pivotBConstraintLocalGame;
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
            _grabFrame.authority.pivotBBodyLocalGame.x,
            _grabFrame.authority.pivotBBodyLocalGame.y,
            _grabFrame.authority.pivotBBodyLocalGame.z,
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
                _grabFrame.authority.pivotAHandBodyLocalGame);
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
        outPivotWorld = generatedProxyLocalPointToWorld(proxyWorldTransform, _grabFrame.authority.pivotAHandBodyLocalGame);
        return std::isfinite(outPivotWorld.x) &&
               std::isfinite(outPivotWorld.y) &&
               std::isfinite(outPivotWorld.z);
    }

    bool Hand::updateConstraintGrabDriveMotors(RE::hknpWorld* world,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        float authorityForceScale,
        bool heldBodyColliding,
        const grab_motion_controller::HeldAuthorityState& heldAuthority)
    {
        if (!_activeConstraint.isValid() || !_activeConstraint.linearMotor || !_activeConstraint.angularMotor) {
            return false;
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
        _activeConstraint.motorBodyProperties = readGrabMotorBodyProperties(
            world, _savedObjectState.bodyId, activeProxyConstraintPivotBLocalGame());
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
            .authorityForceScale = authorityForceScale,
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
        const auto output = grab_motion_controller::solveMotorTargetsWithAuthority(motorInput, heldAuthority);
        if (!output.valid) {
            _activeConstraint.linearMotor->minForce = _activeConstraint.linearMotor->maxForce = 0.0f;
            _activeConstraint.angularMotor->minForce = _activeConstraint.angularMotor->maxForce = 0.0f;
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "{} grab motor properties unavailable; release queued body={}",
                handName(), _savedObjectState.bodyId.value);
            return false;
        }
        _lastGrabPhysicsHz.store(output.physicsHz, std::memory_order_relaxed);

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
        return true;
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

    bool Hand::startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform)
    {
        /*
         * Long-range pull remains a dynamic-object operation. ROCK promotes the
         * selected object tree through FO4VR's recursive wrappers, scans the
         * resulting dynamic body set, and then applies short-lived predicted
         * velocity to the accepted motions. It avoids a keyframed object path
         * that would conflict with the dynamic grab constraint used after arrival.
         */
        if (!world || _state != HandState::SelectionLocked || !_currentSelection.isValid() || !_currentSelection.isFarSelection) {
            return false;
        }
        _pullDriveDecision = {};

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            clearSelectionState(true);
            return false;
        }

        if (!grab_target::canUseRockDynamicPull(_currentSelection.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL blocked: targetKind={} formID={:08X}; actor targets require the loot/SCISSORS path",
                handName(),
                grab_target::name(_currentSelection.targetKind),
                selectedRef->GetFormID());
            clearSelectionState(true);
            return false;
        }

        auto* rootNode = selectedRef->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no 3D root", handName());
            clearSelectionState(true);
            return false;
        }

        restorePullPrepIfActive("new-pull");

        auto* ownerCell = selectedRef->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no bhkWorld", handName());
            clearSelectionState(true);
            return false;
        }

        auto* selectedBase = selectedRef->GetObjectReference();
        const char* selectedType = selectedBase ? selectedBase->GetFormTypeString() : "???";
        auto selectedNameView = selectedBase ? RE::TESFullName::GetFullName(*selectedBase, false) : std::string_view{};
        const std::string selectedName = selectedNameView.empty() ? std::string("(unnamed)") : std::string(selectedNameView);
        const bool selectedIsWeapon = selectedType && std::string_view(selectedType) == "WEAP";

        std::uint16_t selectedOriginalMotionPropsId = 1;
        auto selectedOriginalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        if (_currentSelection.bodyId.value != INVALID_BODY_ID) {
            havok_runtime::tryReadBodyMotionPropertiesId(world, _currentSelection.bodyId, selectedOriginalMotionPropsId);
            if (auto* selectedBody = havok_runtime::getBody(world, _currentSelection.bodyId)) {
                selectedOriginalMotionType = physics_body_classifier::motionTypeFromBodyFlags(selectedBody->flags);
            }
        }

        const auto scanOptions = makeActiveGrabBodyScanOptions(_currentSelection);

        object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet;
        bool beforePrepScanCacheHit = tryUseGrabAcquisitionBeforePrepCache(bhkWorld, world, _currentSelection, scanOptions, beforePrepBodySet);
        if (!beforePrepScanCacheHit) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        }
        active_grab_body_lifecycle::BodyLifecycleSnapshot pullLifecycle;
        pullLifecycle.captureBeforeActivePrep(beforePrepBodySet);
        bool motionConverted = false;
        bool collisionEnabled = false;
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionActivePrep);
            motionConverted =
                physics_recursive_wrappers::setMotionRecursive(rootNode, physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
            collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);
        }
        auto restoreFailedPullPrep = [&]() {
            restoreActiveGrabLifecycle(world,
                pullLifecycle,
                pullLifecycle.restorePlanForFailure(),
                _pulledPrimaryBodyId != INVALID_BODY_ID ? _pulledPrimaryBodyId : _currentSelection.bodyId.value,
                handName(),
                "failed-pull-setup");
            if (pullLifecycle.hasIncompleteNativeScan()) {
                restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(), "failed-pull-setup-incomplete-scan");
            } else if (active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(motionConverted, selectedOriginalMotionPropsId)) {
                physics_recursive_wrappers::setMotionRecursive(
                    rootNode,
                    motionPresetFromMotionType(selectedOriginalMotionType, selectedOriginalMotionPropsId),
                    true,
                    true,
                    false);
            }
        };
        object_physics_body_set::ObjectPhysicsBodySet preparedBodySet;
        bool preparedBodySetPostPrepComplete = false;
        bool preparedScanCacheHit =
            tryBuildGrabAcquisitionPreparedBodySetFromCache(bhkWorld, world, _currentSelection, scanOptions, preparedBodySet, preparedBodySetPostPrepComplete);
        if (!preparedScanCacheHit || preparedBodySet.acceptedCount() == 0) {
            preparedScanCacheHit = false;
            preparedBodySetPostPrepComplete = true;
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        }
        pullLifecycle.markPreparedBodies(preparedBodySet);
        if (preparedScanCacheHit && !preparedBodySetPostPrepComplete) {
            pullLifecycle.markIncompleteNativeScan();
        }
        _pullDriveDecision = classifyHeldBodySetDrive(beforePrepBodySet, preparedBodySet, pullLifecycle.hasIncompleteNativeScan());
        ROCK_LOG_DEBUG(Hand,
            "{} hand PULL scan: type={} weapon={} name='{}' formID={:08X} seedBody={} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seeded={} scanSource={}/{} preparedComplete={} cachedBodyIds={} cacheHits={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} collisionObjects={} visitedNodes={} driveMode={} driveReason={} linearScope={} angularScope={}",
            handName(),
            selectedType ? selectedType : "???",
            selectedIsWeapon ? "yes" : "no",
            selectedName,
            selectedRef->GetFormID(),
            scanOptions.seedBodyId,
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            preparedBodySet.diagnostics.seedBodiesAdded,
            beforePrepScanCacheHit ? "cache" : "direct",
            preparedScanCacheHit ? "cache" : "direct",
            preparedBodySetPostPrepComplete ? "yes" : "no",
            preparedBodySet.diagnostics.cachedBodyIds,
            preparedBodySet.diagnostics.cachedScanHits,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips,
            preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes,
            held_object_drive_policy::modeName(_pullDriveDecision.mode),
            _pullDriveDecision.reason,
            _pullDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _pullDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly");
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);
        const RE::NiPoint3 primaryChoiceTarget = _currentSelection.hasHitPoint ? _currentSelection.hitPointWorld : grabPivotAForPrimaryChoice;
        const auto primaryChoice = preparedBodySet.choosePrimaryBody(_currentSelection.bodyId.value, object_physics_body_set::PurePoint3{ primaryChoiceTarget });

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            const auto* rejectedBody = diagnosticRejectedBodyRecord(preparedBodySet, _currentSelection.bodyId.value);
            ROCK_LOG_WARN(Hand,
                "{} hand PULL failed: no accepted dynamic body after recursive prep formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
                "seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} rejectReason={} rejectBody={} rejectLayer={} rejectMotion={} rejectFlags=0x{:08X} rejectMotionProps={} setMotion={} enableCollision={}",
                handName(),
                selectedRef->GetFormID(),
                beforePrepBodySet.records.size(),
                preparedBodySet.records.size(),
                preparedBodySet.acceptedCount(),
                preparedBodySet.rejectedCount(),
                preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                rejectedBody ? physics_body_classifier::rejectReasonName(rejectedBody->rejectReason) : "none",
                rejectedBody ? rejectedBody->bodyId : INVALID_BODY_ID,
                rejectedBody ? rejectedBody->collisionLayer : 0,
                rejectedBody ? bodyMotionTypeName(rejectedBody->motionType) : "none",
                rejectedBody ? rejectedBody->bodyFlags : 0,
                rejectedBody ? rejectedBody->motionPropertiesId : 0,
                motionConverted ? "ok" : "failed",
                collisionEnabled ? "ok" : "failed");
            restoreFailedPullPrep();
            clearSelectionState(true);
            _pullDriveDecision = {};
            return false;
        }

        _pulledPrimaryBodyId = primaryChoice.bodyId;
        _pulledBodyIds = preparedBodySet.acceptedBodyIds();
        if (_pulledBodyIds.empty()) {
            _pulledBodyIds.push_back(_pulledPrimaryBodyId);
        }
        armPullCatchIntent(selectedRef, _pulledPrimaryBodyId, _currentSelection.targetKind);

        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        _currentSelection.bodyId = RE::hknpBodyId{ _pulledPrimaryBodyId };
        if (!_currentSelection.visualNode) {
            _currentSelection.visualNode = rootNode;
        }
        if (!_currentSelection.hitNode) {
            _currentSelection.hitNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: primary dynamic body has no motion bodyId={}", handName(), _pulledPrimaryBodyId);
            restoreFailedPullPrep();
            clearPullRuntimeState();
            clearSelectionState(true);
            return false;
        }

        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavok = niPointToHkVector(grabPivotWorld);
        RE::NiTransform primaryBodyWorld{};
        const bool hasPrimaryBodyWorld = tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, primaryBodyWorld);
        RE::NiPoint3 ownerNodePosition{};
        bool hasOwnerNode = false;
        if (auto* ownerNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId })) {
            ownerNodePosition = ownerNode->world.translate;
            hasOwnerNode = true;
        }
        const auto selectedPointAnchor = body_frame::chooseSelectionDistanceAnchor(_currentSelection.hasHitPoint,
            _currentSelection.hitPointWorld,
            hasPrimaryBodyWorld,
            primaryBodyWorld.translate,
            hasOwnerNode,
            ownerNodePosition,
            true,
            hkVectorToNiPoint(motion->position),
            grabPivotWorld);
        const auto selectedPointWorld = selectedPointAnchor.position;
        const auto selectedPointHavok = niPointToHkVector(selectedPointWorld);
        _pullPointOffsetHavok = RE::NiPoint3{
            selectedPointHavok.x - motion->position.x,
            selectedPointHavok.y - motion->position.y,
            selectedPointHavok.z - motion->position.z,
        };
        /*
         * Far grabs pull the object CENTER: a ray hit at selection range is aim
         * noise, not grip intent, and a fixed world offset cannot rotate with
         * the presentation servo anyway. Tracking the COM (zero offset) drifts
         * with nothing. Arrival then overwrites the selection hit point with
         * the tracked center, so capture seeds from the middle of the object
         * and the seat (mesh point nearest the palm, seat depth stop) settles
         * the surface onto the palm.
         */
        if (g_rockConfig.rockPullToObjectCenterEnabled) {
            _pullPointOffsetHavok = {};
        }

        /*
         * Long-object presentation capture: one mesh extraction + PCA at pull
         * start. The axis is frozen in primary-body local space so the flight
         * servo in updateDynamicPull can re-derive it from the live body pose
         * without touching nodes per frame.
         */
        _pullPresentationAxisBodyLocal = {};
        _pullPresentationElongationRatio = 0.0f;
        _pullPresentationValid = false;
        if (g_rockConfig.rockPullLongAxisPresentationEnabled && hasPrimaryBodyWorld) {
            RE::NiAVObject* presentationMeshNode = _currentSelection.visualNode ? _currentSelection.visualNode : rootNode;
            if (presentationMeshNode) {
                std::vector<TriangleData> presentationTriangles;
                std::vector<GrabSurfaceTriangleData> presentationSurfaceTriangles;
                MeshExtractionStats presentationMeshStats;
                extractAllSurfaceTriangles(presentationMeshNode,
                    presentationTriangles,
                    presentationSurfaceTriangles,
                    (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth),
                    &presentationMeshStats,
                    false);
                const auto longAxis = computeGrabMeshLongAxis(presentationTriangles);
                if (longAxis.valid && longAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio) {
                    const RE::NiPoint3 axisBodyLocal = normalizeOrZero(
                        transform_math::worldVectorToLocal(primaryBodyWorld, longAxis.axisWorld));
                    if (lengthSquared(axisBodyLocal) > 0.000001f) {
                        _pullPresentationAxisBodyLocal = axisBodyLocal;
                        _pullPresentationElongationRatio = longAxis.elongationRatio;
                        _pullPresentationValid = true;
                    }
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand PULL presentation axis: valid={} elongation={:.2f} tris={} reason={} axisLocal=({:.3f},{:.3f},{:.3f})",
                    handName(),
                    _pullPresentationValid ? "yes" : "no",
                    longAxis.elongationRatio,
                    longAxis.triangleCount,
                    longAxis.reason,
                    _pullPresentationAxisBodyLocal.x,
                    _pullPresentationAxisBodyLocal.y,
                    _pullPresentationAxisBodyLocal.z);
            }
        }

        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };
        const RE::NiPoint3 grabPivotPointHavok{ grabPivotHavok.x, grabPivotHavok.y, grabPivotHavok.z };
        const float pullDistance = (grabPivotPointHavok - objectPointHavok).Length();

        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = pull_motion_math::computePullDurationSeconds(pullDistance);
        _pullTargetHavok = {};
        _pullHasTarget = false;
        const auto transition = applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::BeginPull });
        if (!transition.accepted) {
            clearPullRuntimeState(false, "begin-pull-rejected");
            clearPullCatchIntent("beginPullRejected");
            restoreFailedPullPrep();
            return false;
        }
        _pullActiveLifecycle = pullLifecycle;
        _pullPrepHknpWorld = world;
        _pullPrepRootNode = rootNode;
        _pullPrepRefr = selectedRef;
        _pullPrepTargetKind = _currentSelection.targetKind;
        _pullPrepOriginalMotionPropsId = selectedOriginalMotionPropsId;
        _pullPrepRestoreArmed = true;
        stopSelectionHighlight();

        ROCK_LOG_INFO(Hand,
            "{} hand PULL start: type={} weapon={} formID={:08X} primaryBody={} bodyCount={} driveMode={} linearScope={} angularScope={} seeded={} scanFailures={} invalidSystems={} benignSkips={} unresolvedAccepted={} distanceHk={:.3f} duration={:.3f}s setMotion={} enableCollision={}",
            handName(),
            selectedType ? selectedType : "???",
            selectedIsWeapon ? "yes" : "no",
            selectedRef->GetFormID(),
            _pulledPrimaryBodyId,
            _pulledBodyIds.size(),
            held_object_drive_policy::modeName(_pullDriveDecision.mode),
            _pullDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            _pullDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            preparedBodySet.diagnostics.seedBodiesAdded,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            pullDistance,
            _pullDurationSeconds,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed");
        return true;
    }

    bool Hand::updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime)
    {
        if (_state != HandState::Pulled || !world || _pulledPrimaryBodyId == INVALID_BODY_ID || !_currentSelection.isValid()) {
            return false;
        }

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            ROCK_LOG_DEBUG(Hand, "{} hand PULL invalidated before arrival: selected ref unavailable", handName());
            clearSelectionState(true);
            return false;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL invalidated before arrival: primary motion missing bodyId={}",
                handName(),
                _pulledPrimaryBodyId);
            clearSelectionState(true);
            return false;
        }

        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavokVector = niPointToHkVector(grabPivotWorld);
        const RE::NiPoint3 grabPivotHavok{ grabPivotHavokVector.x, grabPivotHavokVector.y, grabPivotHavokVector.z };
        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };

        const float distanceGameUnits = (grabPivotHavok - objectPointHavok).Length() * havokToGameScale();
        _currentSelection.distance = distanceGameUnits;
        const float configuredAutoGrabDistance = pull_motion_math::kAutoGrabDistanceGameUnits;
        const float nearConvergeDistance = (std::max)(configuredAutoGrabDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
        const float pocketBand = (std::max)(0.0f, g_rockConfig.rockGrabPocketRadiusGameUnits);
        const float arrivalDistance = (std::max)(configuredAutoGrabDistance, (std::min)(nearConvergeDistance, configuredAutoGrabDistance + pocketBand));
        if (distanceGameUnits <= arrivalDistance) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = RE::NiPoint3{
                objectPointHavok.x * havokToGameScale(),
                objectPointHavok.y * havokToGameScale(),
                objectPointHavok.z * havokToGameScale(),
            };
            _currentSelection.hasHitPoint = true;
            markPullCatchIntentArrived();
            _pullTargetHavok = {};
            _pullElapsedSeconds = 0.0f;
            _pullDurationSeconds = 0.0f;
            _pullHasTarget = false;
            applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::PullArrivedClose });
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL arrived -> close grab window dist={:.1f} arrival={:.1f} configuredAuto={:.1f} near={:.1f} pocketBand={:.1f}",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                configuredAutoGrabDistance,
                nearConvergeDistance,
                pocketBand);
            return true;
        }

        const auto motionResult = pull_motion_math::computePullMotion<RE::NiPoint3>(
            pull_motion_math::PullMotionInput<RE::NiPoint3>{
                .handHavok = grabPivotHavok,
                .objectPointHavok = objectPointHavok,
                .previousTargetHavok = _pullTargetHavok,
                .elapsedSeconds = _pullElapsedSeconds,
                .durationSeconds = _pullDurationSeconds,
                .hasPreviousTarget = _pullHasTarget,
            });

        _pullElapsedSeconds += (std::max)(0.0f, deltaTime);
        if (motionResult.refreshTarget) {
            _pullTargetHavok = motionResult.targetHavok;
            _pullHasTarget = true;
        }

        if (motionResult.expired) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL owner expired before arrival dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s ownerGrace={:.3f}s",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                _pullElapsedSeconds,
                _pullDurationSeconds,
                pull_motion_math::kOwnerGraceSeconds);
            finishPullPrepAsPhysicalDropIfActive("pull-owner-expired");
            clearSelectionState(true);
            return false;
        }

        if (!motionResult.applyVelocity) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} hand PULL holding owner after velocity window dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s ownerGrace={:.3f}s",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                _pullElapsedSeconds,
                _pullDurationSeconds,
                pull_motion_math::kOwnerGraceSeconds);
            for (const auto bodyId : _pulledBodyIds) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
            return false;
        }

        /*
         * Long-object presentation (flight only): servo the mesh principal
         * axis toward the hand's cross-palm (thumb->pinky) line while the pull
         * drive owns the object, so long props arrive oriented for the grab.
         * The angular velocity is SET each frame from the remaining angle
         * (kinematic servo with exponential decay), so it cannot overshoot.
         * This path ends at pull arrival, before capture freezes the relation;
         * held objects keep the no-rotate rule.
         */
        RE::NiPoint3 presentationAngularVelocity{};
        bool presentationActive = false;
        if (_pullPresentationValid && g_rockConfig.rockPullLongAxisPresentationEnabled) {
            RE::NiTransform pulledBodyWorld{};
            if (tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, pulledBodyWorld)) {
                const RE::NiPoint3 currentAxisWorld =
                    normalizeOrZero(transform_math::localVectorToWorld(pulledBodyWorld, _pullPresentationAxisBodyLocal));
                /*
                 * The grip line is not pure cross-palm Z: the thumb sits in the
                 * way, so the natural long-object hold tilts a few degrees
                 * toward the fingers-forward X axis (Bruno-tuned via INI).
                 */
                const RE::NiPoint3 crossPalmWorld =
                    transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 0.0f, 0.0f, 1.0f }, _isLeft);
                const RE::NiPoint3 fingerForwardWorld =
                    transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft);
                RE::NiPoint3 targetAxisWorld = grab_three_phase::buildGripPresentationAxisTowardFingertips(
                    crossPalmWorld,
                    fingerForwardWorld,
                    g_rockConfig.rockPullPresentationGripAxisTiltDegrees);
                if (lengthSquared(currentAxisWorld) > 0.000001f && lengthSquared(targetAxisWorld) > 0.000001f) {
                    // The mesh axis has no sign: always rotate toward the nearest hemisphere.
                    if (dotProduct(currentAxisWorld, targetAxisWorld) < 0.0f) {
                        targetAxisWorld = RE::NiPoint3{ -targetAxisWorld.x, -targetAxisWorld.y, -targetAxisWorld.z };
                    }
                    const RE::NiPoint3 rotationAxis = crossProduct(currentAxisWorld, targetAxisWorld);
                    const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxis)));
                    const float cosAngle = std::clamp(dotProduct(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                    const float angleRadians = std::atan2(sinAngle, cosAngle);
                    if (sinAngle > 0.000001f && angleRadians > 0.005f) {
                        const float angularSpeed = (std::min)(
                            angleRadians * (std::max)(0.0f, g_rockConfig.rockPullPresentationAngularGainPerSecond),
                            (std::max)(0.0f, g_rockConfig.rockPullPresentationMaxAngularSpeedRadiansPerSecond));
                        const float invSin = 1.0f / sinAngle;
                        presentationAngularVelocity = RE::NiPoint3{
                            rotationAxis.x * invSin * angularSpeed,
                            rotationAxis.y * invSin * angularSpeed,
                            rotationAxis.z * invSin * angularSpeed,
                        };
                        presentationActive = angularSpeed > 0.0f;
                    }
                }
            }
        }
        if (presentationActive) {
            setHeldVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds, motionResult.velocityHavok,
                presentationAngularVelocity,
                true,
                1.0f,
                _pullDriveDecision.includeConnectedLinearVelocity,
                _pullDriveDecision.includeConnectedAngularVelocity);
        } else {
            setHeldLinearVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds, motionResult.velocityHavok,
                pull_motion_math::angularVelocityKeepForDamping(deltaTime),
                _pullDriveDecision.includeConnectedLinearVelocity);
        }
        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        return false;
    }

    struct Hand::ValidatedGrabSelection
    {
        RE::NiPointer<RE::TESObjectREFR> retainedRef;
        RE::hknpBodyId bodyId{};
        RE::NiAVObject* rootNode = nullptr;
        RE::bhkWorld* bhkWorld = nullptr;
        RE::TESBoundObject* baseObject = nullptr;
        std::string objectName{ "(unnamed)" };
        bool joiningPeerHeldObject = false;
        bool grabbedFromPullCatch = false;
        bool looseWeaponGrab = false;
        bool handPocketOnlyGrab = false;
    };

    bool Hand::validateSelectedGrab(
        RE::hknpWorld* world,
        const GrabSharedObjectContext& sharedContext,
        ValidatedGrabSelection& outSelection)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabSelectionValidation);
        outSelection = {};
        if (!hasSelection() || !world) {
            return false;
        }
        if (!hasCollisionBody()) {
            return false;
        }

        const auto& selection = _currentSelection;
        outSelection.retainedRef = selection.retainedRef;
        if (!outSelection.retainedRef || outSelection.retainedRef.get() != selection.refr || selection.bodyId.value == INVALID_BODY_ID) {
            return false;
        }
        if (selection.refr->IsDeleted() || selection.refr->IsDisabled()) {
            return false;
        }

        if (!grab_target::canUseRockActiveGrab(selection.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB blocked: targetKind={} formID={:08X}; actor targets are not normal ROCK physical grabs",
                handName(),
                grab_target::name(selection.targetKind),
                selection.refr ? selection.refr->GetFormID() : 0);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        outSelection.bodyId = selection.bodyId;
        outSelection.rootNode = selection.refr->Get3D();
        if (!outSelection.rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no 3D root", handName());
            return false;
        }

        auto* ownerCell = selection.refr->GetParentCell();
        outSelection.bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!outSelection.bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no bhkWorld for object-tree scan", handName());
            return false;
        }

        if (!havok_runtime::getBody(world, outSelection.bodyId)) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: selected body no longer readable bodyId={}",
                handName(),
                outSelection.bodyId.value);
            return false;
        }

        auto* baseObj = selection.refr->GetObjectReference();
        outSelection.baseObject = baseObj;
        const bool selectedObjectIsCar = fo4vr::isExplodableCar(baseObj);
        const auto carGrabDecision = car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = selectedObjectIsCar,
            .playerInPowerArmor = selectedObjectIsCar && fo4vr::isInPowerArmor(),
        });
        if (!carGrabDecision.allowed) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB blocked: formID={:08X} reason={}",
                handName(),
                selection.refr->GetFormID(),
                carGrabDecision.reason);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        if (outSelection.baseObject) {
            const auto nameView = RE::TESFullName::GetFullName(*outSelection.baseObject, false);
            if (!nameView.empty()) {
                outSelection.objectName = std::string(nameView);
            }
        }

        outSelection.joiningPeerHeldObject = sharedContextMatchesSelection(sharedContext, selection);
        outSelection.grabbedFromPullCatch = pullCatchIntentMatchesSelection();
        outSelection.looseWeaponGrab = isLooseWeaponGrabTarget(selection);
        outSelection.handPocketOnlyGrab = grab_target::requiresHandPocketGrab(selection.targetKind);
        return true;
    }

    struct Hand::GrabBodyPreparation
    {
        object_physics_body_set::BodySetScanOptions scanOptions{};
        active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle{};
        object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet{};
        object_physics_body_set::ObjectPhysicsBodySet preparedBodySet{};
        bool consumedPullPrepLifecycle = false;
        bool beforePrepScanCacheHit = false;
        bool motionConverted = true;
        bool collisionEnabled = true;
        bool preparedScanCacheHit = false;
        bool preparedBodySetPostPrepComplete = false;
    };

    void Hand::prepareSelectedGrabBodies(
        RE::hknpWorld* world,
        const GrabSharedObjectContext& sharedContext,
        const ValidatedGrabSelection& selection,
        GrabBodyPreparation& outPreparation)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabBodyPreparation);
        outPreparation = {};
        const auto& selectedObject = _currentSelection;
        outPreparation.scanOptions = makeActiveGrabBodyScanOptions(selectedObject);
        const auto& scanOptions = outPreparation.scanOptions;
        if (grab_target::isRagdoll(selectedObject.targetKind)) {
            outPreparation.beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(
                selection.bhkWorld, world, selectedObject.refr, scanOptions);
            outPreparation.preparedBodySet = outPreparation.beforePrepBodySet;
            outPreparation.preparedBodySetPostPrepComplete = true;
            return;
        }


        outPreparation.consumedPullPrepLifecycle =
            !selection.joiningPeerHeldObject &&
            selection.grabbedFromPullCatch &&
            consumePullPrepLifecycleForActiveGrab(selectedObject.refr, outPreparation.activeLifecycle);
        outPreparation.beforePrepScanCacheHit = tryUseGrabAcquisitionBeforePrepCache(
            selection.bhkWorld,
            world,
            selectedObject,
            scanOptions,
            outPreparation.beforePrepBodySet);
        if (!outPreparation.beforePrepScanCacheHit) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
            outPreparation.beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(
                selection.bhkWorld,
                world,
                selectedObject.refr,
                scanOptions);
        }
        if (!selection.joiningPeerHeldObject && !outPreparation.consumedPullPrepLifecycle) {
            outPreparation.activeLifecycle.captureBeforeActivePrep(outPreparation.beforePrepBodySet);
        }

        if (!selection.joiningPeerHeldObject) {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionActivePrep);
            outPreparation.motionConverted = physics_recursive_wrappers::setMotionRecursive(
                selection.rootNode,
                physics_recursive_wrappers::MotionPreset::Dynamic,
                true,
                true,
                true);
            outPreparation.collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(
                selection.rootNode,
                true,
                true,
                true);
        }

        outPreparation.preparedScanCacheHit = selection.joiningPeerHeldObject;
        outPreparation.preparedBodySetPostPrepComplete = selection.joiningPeerHeldObject;
        if (selection.joiningPeerHeldObject) {
            outPreparation.preparedBodySet = outPreparation.beforePrepBodySet;
        } else {
            outPreparation.preparedScanCacheHit = tryBuildGrabAcquisitionPreparedBodySetFromCache(
                selection.bhkWorld,
                world,
                selectedObject,
                scanOptions,
                outPreparation.preparedBodySet,
                outPreparation.preparedBodySetPostPrepComplete);
            if (!outPreparation.preparedScanCacheHit || outPreparation.preparedBodySet.acceptedCount() == 0) {
                outPreparation.preparedScanCacheHit = false;
                outPreparation.preparedBodySetPostPrepComplete = true;
                performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabAcquisitionBodyScan);
                outPreparation.preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(
                    selection.bhkWorld,
                    world,
                    selectedObject.refr,
                    scanOptions);
            }
        }

        if (!selection.joiningPeerHeldObject) {
            outPreparation.activeLifecycle.markPreparedBodies(outPreparation.preparedBodySet);
            if (outPreparation.preparedScanCacheHit && !outPreparation.preparedBodySetPostPrepComplete) {
                outPreparation.activeLifecycle.markIncompleteNativeScan();
            }
        } else if (sharedContext.peerActiveGrabLifecycle) {
            outPreparation.activeLifecycle = *sharedContext.peerActiveGrabLifecycle;
        }
    }

    struct Hand::GrabProxyPreparation
    {
        RE::NiTransform handBodyWorldAtGrab{};
        RE::NiTransform proxyFrameWorldAtGrab{};
        RE::NiTransform proxyAuthorityFrameWorldAtGrab{};
        RE::NiPoint3 grabAuthorityPivotAWorld{};
        RE::NiPoint3 palmPocketPivotAWorld{};
        RE::NiPoint3 grabPivotAForPrimaryChoice{};
        GrabPalmBasisDelta palmBasisDelta{};
        const char* proxyFrameSourceAtGrab = "unresolved";
        float palmPocketToProxyDeltaGameUnits = 0.0f;
        bool hasPalmProxyFrameAtGrab = false;
    };

    bool Hand::prepareGrabProxyAuthority(
        RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        GrabProxyPreparation& outPreparation)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabProxyPreparation);
        outPreparation = {};
        outPreparation.handBodyWorldAtGrab = getLiveBodyWorldTransform(world, _handBody.getBodyId());
        outPreparation.proxyFrameWorldAtGrab = outPreparation.handBodyWorldAtGrab;
        if (!resolveGrabAuthorityProxyFrame(
                world,
                handWorldTransform,
                &outPreparation.handBodyWorldAtGrab,
                outPreparation.proxyFrameWorldAtGrab,
                outPreparation.proxyFrameSourceAtGrab,
                GrabAuthorityProxyFramePolicy::LivePalmOnly)) {
            return false;
        }

        outPreparation.grabAuthorityPivotAWorld = outPreparation.proxyFrameWorldAtGrab.translate;
        outPreparation.palmPocketPivotAWorld = outPreparation.proxyFrameWorldAtGrab.translate;
        outPreparation.palmPocketToProxyDeltaGameUnits = pointDistanceGameUnits(
            outPreparation.grabAuthorityPivotAWorld,
            outPreparation.palmPocketPivotAWorld);
        outPreparation.palmBasisDelta = computeGrabPalmBasisDelta(handWorldTransform, outPreparation.proxyFrameWorldAtGrab);
        outPreparation.grabPivotAForPrimaryChoice = outPreparation.palmPocketPivotAWorld;
        outPreparation.proxyAuthorityFrameWorldAtGrab = makeGeneratedProxyAuthorityRelationFrame(outPreparation.proxyFrameWorldAtGrab);
        outPreparation.hasPalmProxyFrameAtGrab = true;
        return true;
    }

    struct Hand::GrabMeshCaptureSetup
    {
        RE::NiAVObject* collidableNode = nullptr;
        RE::NiAVObject* meshSourceNode = nullptr;
        RE::NiTransform objectWorldTransform{};
    };

    bool Hand::prepareGrabMeshCapture(
        const RE::NiTransform& handWorldTransform,
        const ValidatedGrabSelection& selection,
        const std::string& objectName,
        GrabMeshCaptureSetup& outSetup)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabMeshCapturePreparation);
        outSetup = {};
        const auto& selectedObject = _currentSelection;
        outSetup.collidableNode = selectedObject.hitNode ? selectedObject.hitNode : selection.rootNode;
        outSetup.meshSourceNode = selectedObject.visualNode ? selectedObject.visualNode : selection.rootNode;
        if (!outSetup.meshSourceNode) {
            outSetup.meshSourceNode = outSetup.collidableNode;
        }

        const auto captureRefresh = refreshGrabCaptureTransforms(
            selection.rootNode,
            outSetup.meshSourceNode,
            outSetup.collidableNode,
            !grab_target::isRagdoll(selectedObject.targetKind));
        if (!captureRefresh.ok) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: capture transform refresh produced a non-finite node transform for '{}' formID={:08X}; root='{}' mesh='{}' collidable='{}'",
                handName(),
                objectName,
                selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                nodeDebugName(selection.rootNode),
                nodeDebugName(outSetup.meshSourceNode),
                nodeDebugName(outSetup.collidableNode));
            return false;
        }

        outSetup.objectWorldTransform = outSetup.collidableNode ? outSetup.collidableNode->world : handWorldTransform;
        if (!grab_three_phase::isFinite(outSetup.objectWorldTransform)) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: refreshed object transform is non-finite for '{}' formID={:08X}; collidable='{}'",
                handName(),
                objectName,
                selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                nodeDebugName(outSetup.collidableNode));
            return false;
        }
        return true;
    }

    struct Hand::GrabMeshExtraction
    {
        RE::NiAVObject* meshSourceNode = nullptr;
        MeshExtractionStats stats{};
        std::vector<TriangleData> meshTriangles{};
        std::vector<GrabSurfaceTriangleData> surfaceTriangles{};
        GrabSurfaceQueryIndex queryIndex{};
    };

    void Hand::extractGrabMeshEvidence(
        RE::hknpWorld* world,
        RE::hknpBodyId objectBodyId,
        RE::NiAVObject* rootNode,
        RE::NiAVObject* collidableNode,
        RE::NiAVObject* meshSourceNode,
        bool handPocketOnlyGrab,
        bool surfaceEvidenceRequired,
        GrabMeshExtraction& outExtraction)
    {
        outExtraction = {};
        outExtraction.meshSourceNode = meshSourceNode;
        if (!meshSourceNode) {
            return;
        }

        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabMeshExtraction);
        const int meshExtractionDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
        std::array<RE::NiAVObject*, 3> attemptedRoots{};
        std::uint32_t attemptCount = 0;
        const char* extractionSource = "visual";
        attemptedRoots[attemptCount++] = meshSourceNode;
        const auto extract = [&](RE::NiAVObject* source) {
            if (surfaceEvidenceRequired) {
                extractAllSurfaceTriangles(source, outExtraction.meshTriangles, outExtraction.surfaceTriangles,
                    meshExtractionDepth, &outExtraction.stats, handPocketOnlyGrab);
                return;
            }
            // Authored placement needs no per-triangle contact/skin ownership
            // records. Keep the same vertex positions for release spin limiting.
            const auto before = outExtraction.meshTriangles.size();
            extractAllTriangles(source, outExtraction.meshTriangles, meshExtractionDepth, &outExtraction.stats);
            if (source && outExtraction.meshTriangles.size() == before) {
                RE::BSVisit::TraverseScenegraphGeometries(source, [&](RE::BSGeometry* geometry) {
                    if (auto* shape = geometry ? geometry->IsTriShape() : nullptr) {
                        extractAllTriangles(shape, outExtraction.meshTriangles, 1, &outExtraction.stats);
                    }
                    return RE::BSVisit::BSVisitControl::kContinue;
                });
            }
        };
        extract(meshSourceNode);

        auto tryAlternateRoot = [&](RE::NiAVObject* candidateRoot, const char* sourceName) {
            if (!candidateRoot) {
                return false;
            }
            for (std::uint32_t i = 0; i < attemptCount; ++i) {
                if (attemptedRoots[i] == candidateRoot) {
                    return false;
                }
            }
            if (attemptCount < attemptedRoots.size()) {
                attemptedRoots[attemptCount] = candidateRoot;
            }
            ++attemptCount;
            const auto beforeTriangles = outExtraction.meshTriangles.size();
            extract(candidateRoot);
            if (outExtraction.meshTriangles.size() == beforeTriangles) {
                return false;
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand mesh extraction recovered from {} node: meshNode='{}' previousMeshNode='{}' ownerNode='{}' rootNode='{}' addedTris={} totalTris={}",
                handName(),
                sourceName,
                nodeDebugName(candidateRoot),
                nodeDebugName(outExtraction.meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode),
                outExtraction.meshTriangles.size() - beforeTriangles,
                outExtraction.meshTriangles.size());
            outExtraction.meshSourceNode = candidateRoot;
            extractionSource = sourceName;
            return true;
        };

        if (outExtraction.meshTriangles.empty()) {
            if (!tryAlternateRoot(collidableNode, "owner")) {
                (void)tryAlternateRoot(rootNode, "root");
            }
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} source={} attempts={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} totalTris={}",
            handName(),
            nodeDebugName(outExtraction.meshSourceNode),
            nodeDebugName(collidableNode),
            nodeDebugName(rootNode),
            outExtraction.stats.visitedShapes,
            extractionSource,
            attemptCount,
            outExtraction.stats.staticShapes,
            outExtraction.stats.staticTriangles,
            outExtraction.stats.dynamicShapes,
            outExtraction.stats.dynamicTriangles,
            outExtraction.stats.skinnedShapes,
            outExtraction.stats.skinnedTriangles,
            outExtraction.stats.dynamicSkinnedSkipped,
            outExtraction.stats.emptyShapes,
            outExtraction.stats.totalTriangles());
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::GrabMeshTriangles,
            outExtraction.stats.totalTriangles());
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshStaticTriangles, outExtraction.stats.staticTriangles);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshDynamicTriangles, outExtraction.stats.dynamicTriangles);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshSkinnedTriangles, outExtraction.stats.skinnedTriangles);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshCaptureAttempts, attemptCount);
        performance_profiler::observeValue(performance_profiler::ValueMetric::GrabMeshPayloadBytes,
            outExtraction.meshTriangles.size() * sizeof(TriangleData) + outExtraction.surfaceTriangles.size() * sizeof(GrabSurfaceTriangleData));

        RE::BSTriShape* firstTriShape = outExtraction.meshSourceNode->IsTriShape();
        if (!firstTriShape) {
            if (auto* meshNode = outExtraction.meshSourceNode->IsNode()) {
                auto& children = meshNode->GetRuntimeData().children;
                const auto childCount = children.size();
                for (auto index = decltype(childCount){ 0 }; index < childCount; ++index) {
                    auto* child = children[index].get();
                    if (child && child->IsTriShape()) {
                        firstTriShape = child->IsTriShape();
                        break;
                    }
                }
            }
        }
        if (firstTriShape) {
            auto* triShapeBytes = reinterpret_cast<char*>(firstTriShape);
            const std::uint64_t vertexDescriptor = *reinterpret_cast<std::uint64_t*>(triShapeBytes + VROffset::vertexDesc);
            const std::uint32_t stride = static_cast<std::uint32_t>(vertexDescriptor & 0xF) * 4;
            const std::uint32_t positionOffset = static_cast<std::uint32_t>((vertexDescriptor >> 2) & 0x3C);
            const bool fullPrecision = ((vertexDescriptor >> 54) & 1) != 0;
            const std::uint8_t geometryType = *reinterpret_cast<std::uint8_t*>(triShapeBytes + 0x198);
            void* skinInstance = *reinterpret_cast<void**>(triShapeBytes + VROffset::skinInstance);
            ROCK_LOG_TRACE(MeshGrab,
                "VertexDiag '{}': vtxDesc=0x{:016X} stride={} posOffset={} fullPrec={} geomType={} skinned={}",
                firstTriShape->name.c_str() ? firstTriShape->name.c_str() : "(null)",
                vertexDescriptor,
                stride,
                positionOffset,
                fullPrecision ? 1 : 0,
                geometryType,
                skinInstance ? 1 : 0);
        }

        if (!outExtraction.meshTriangles.empty()) {
            const auto& triangle = outExtraction.meshTriangles.front();
            const float cx = (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0f;
            const float cy = (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0f;
            const float cz = (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0f;
            if (auto* liveBody = havok_runtime::getBody(world, objectBodyId)) {
                auto* bodyValues = reinterpret_cast<float*>(liveBody);
                const float scale = havokToGameScale();
                const float distance = std::sqrt(
                    (cx - bodyValues[12] * scale) * (cx - bodyValues[12] * scale) +
                    (cy - bodyValues[13] * scale) * (cy - bodyValues[13] * scale) +
                    (cz - bodyValues[14] * scale) * (cz - bodyValues[14] * scale));
                ROCK_LOG_TRACE(MeshGrab,
                    "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToBody={:.1f}gu",
                    cx,
                    cy,
                    cz,
                    distance);
            }
        }
    }

    struct Hand::GrabSurfaceEvidence
    {
        RE::NiPoint3 gripPoint{};
        float selectionToMeshDistanceGameUnits = 0.0f;
        bool meshGrabFound = false;
        GrabSurfaceHit surfaceHit{};
        RE::NiAVObject* surfaceOwnerNode = nullptr;
        const char* pointMode = "noContactPointPending";
        GrabPivotAuthoritySource pointAuthoritySource = GrabPivotAuthoritySource::None;
        const char* fallbackReason = "noMeshSourceNode";
    };

    void Hand::resolveGrabSurfaceEvidence(
        const ValidatedGrabSelection& selection,
        const GrabProxyPreparation& proxy,
        const GrabMeshCaptureSetup& capture,
        const GrabMeshExtraction& mesh,
        bool meshContactOnly,
        GrabSurfaceEvidence& outEvidence)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabSurfaceResolution);
        const auto& sel = _currentSelection;
        auto* rootNode = selection.rootNode;
        auto* collidableNode = capture.collidableNode;
        auto* meshSourceNode = mesh.meshSourceNode;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        const auto& grabSurfaceTriangles = mesh.surfaceTriangles;
        const auto& meshStats = mesh.stats;
        const bool handPocketOnlyGrab = selection.handPocketOnlyGrab;
        const auto& palmPocketPivotAWorld = proxy.palmPocketPivotAWorld;
        const auto& proxyAuthorityFrameWorldAtGrab = proxy.proxyAuthorityFrameWorldAtGrab;
        auto& grabGripPoint = outEvidence.gripPoint;
        auto& selectionToMeshDistanceGameUnits = outEvidence.selectionToMeshDistanceGameUnits;
        auto& meshGrabFound = outEvidence.meshGrabFound;
        auto& grabSurfaceHit = outEvidence.surfaceHit;
        auto& surfaceOwnerNode = outEvidence.surfaceOwnerNode;
        auto& grabPointMode = outEvidence.pointMode;
        auto& grabPointAuthoritySource = outEvidence.pointAuthoritySource;
        auto& grabFallbackReason = outEvidence.fallbackReason;
        grabGripPoint = sel.hasHitPoint ? sel.hitPointWorld : proxy.grabPivotAForPrimaryChoice;
        grabPointMode = sel.hasHitPoint ? "selectionHitPointFallback" : "palmPivotFallback";
        grabPointAuthoritySource = sel.hasHitPoint ? GrabPivotAuthoritySource::CollisionFallback : GrabPivotAuthoritySource::None;
        grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";
            /*
             * hknp selection identifies the object/body. In mesh-authoritative mode
             * the rendered mesh supplies the grabbed point when it is found; the
             * physics collision point is the fallback below, never a refusal.
             */
            GrabSurfaceHit collisionSurfaceHit{};
            if (sel.hasHitPoint && sel.hasHitNormal) {
                collisionSurfaceHit = makeCollisionQueryGrabSurfaceHit(sel, collidableNode);
                if (collisionSurfaceHit.valid) {
                    if (!meshContactOnly && !handPocketOnlyGrab) {
                        grabSurfaceHit = collisionSurfaceHit;
                        grabGripPoint = grabSurfaceHit.position;
                        surfaceOwnerNode = grabSurfaceHit.sourceNode;
                        meshGrabFound = true;
                        grabPointMode = "collisionSurface";
                        grabPointAuthoritySource = GrabPivotAuthoritySource::CollisionFallback;
                        grabFallbackReason = "none";
                    }
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand COLLISION SELECTION HIT: body={} activeGrabPoint={} point=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) "
                        "fraction={:.4f} shapeKey=0x{:08X} filter=0x{:08X} owner='{}'",
                        handName(),
                        sel.bodyId.value,
                        meshContactOnly ? "no" : "yes",
                        collisionSurfaceHit.position.x,
                        collisionSurfaceHit.position.y,
                        collisionSurfaceHit.position.z,
                        collisionSurfaceHit.normal.x,
                        collisionSurfaceHit.normal.y,
                        collisionSurfaceHit.normal.z,
                        collisionSurfaceHit.hitFraction,
                        collisionSurfaceHit.shapeKey,
                        collisionSurfaceHit.shapeCollisionFilterInfo,
                        nodeDebugName(collisionSurfaceHit.sourceNode));
                }
            }

            if (meshSourceNode) {
                const bool closeGrabNeedsPalmPocketMeshAuthority =
                    !grabSurfaceTriangles.empty() &&
                    (handPocketOnlyGrab ||
                        (!sel.isFarSelection &&
                            (!meshGrabFound || grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::CollisionQuery)));
                if (closeGrabNeedsPalmPocketMeshAuthority) {
                    /*
                     * One grab point, chosen in order: the mesh point nearest
                     * the palm centre (close grabs), the mesh point at the
                     * selection hit, the physics collision point, the palm.
                     */
                    const RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
                    const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(proxyAuthorityFrameWorldAtGrab, _isLeft);
                    const auto closePocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                        proxyAuthorityFrameWorldAtGrab,
                        _isLeft,
                        grabPivotAWorld,
                        g_rockConfig.rockGrabPocketDepthGameUnits,
                        g_rockConfig.rockGrabPocketRadiusGameUnits);
                    const RE::NiPoint3 pocketAuthorityPoint = closePocket.valid ? closePocket.palmCenterWorld : grabPivotAWorld;
                    const RE::NiPoint3 pocketAuthorityNormal = closePocket.valid ? closePocket.palmNormalWorld : palmDir;
                    const float palmPocketSnapDistance = (std::max)(
                        finitePositiveOr(g_rockConfig.rockGrabPocketRadiusGameUnits, 6.0f),
                        (std::max)(
                            finitePositiveOr(g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance, 8.0f),
                            finitePositiveOr(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, 10.0f)));

                    if (findClosestGrabSurfaceHitToPointPositionOnly(
                            grabSurfaceTriangles,
                            pocketAuthorityPoint,
                            pocketAuthorityNormal,
                            palmPocketSnapDistance,
                            grabSurfaceHit, &mesh.queryIndex)) {
                        grabGripPoint = grabSurfaceHit.position;
                        selectionToMeshDistanceGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                        grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                        grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        grabSurfaceHit.shapeKey = sel.hitShapeKey;
                        grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
                        grabSurfaceHit.hitFraction = sel.hitFraction;
                        grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
                        surfaceOwnerNode = grabSurfaceHit.sourceNode;
                        meshGrabFound = true;
                        grabPointMode = "palmPocketMeshSurface";
                        grabPointAuthoritySource = GrabPivotAuthoritySource::PalmPocketMeshPoint;
                        grabFallbackReason = "closePalmPocketMeshAuthority";
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand MESH GRAB: mode={} tris={} closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' shape='{}' "
                            "pocket=({:.1f},{:.1f},{:.1f}) snapLimit={:.1f} selectionHit={} selectionDelta={:.1f}gu surfaceAlong={:.1f} surfaceLateral={:.1f}",
                            handName(),
                            grabPointMode,
                            grabMeshTriangles.size(),
                            grabSurfaceHit.position.x,
                            grabSurfaceHit.position.y,
                            grabSurfaceHit.position.z,
                            grabSurfaceHit.triangleIndex,
                            grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                            nodeDebugName(grabSurfaceHit.sourceNode),
                            nodeDebugName(grabSurfaceHit.sourceShape),
                            pocketAuthorityPoint.x,
                            pocketAuthorityPoint.y,
                            pocketAuthorityPoint.z,
                            palmPocketSnapDistance,
                            sel.hasHitPoint ? "yes" : "no",
                            sel.hasHitPoint ? selectionToMeshDistanceGameUnits : -1.0f,
                            grabSurfaceHit.signedAlongPalmDistanceGameUnits,
                            grabSurfaceHit.lateralPalmDistanceGameUnits);
                    }
                }

                if (!meshGrabFound && !grabSurfaceTriangles.empty() && sel.hasHitPoint) {
                    const RE::NiPoint3 grabPivotAWorld = palmPocketPivotAWorld;
                    RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(proxyAuthorityFrameWorldAtGrab, _isLeft);
                    const RE::NiPoint3 expectedNormal =
                        sel.hasHitNormal && lengthSquared(sel.hitNormalWorld) > 0.0f ? sel.hitNormalWorld : palmDir;

                    if (findClosestGrabSurfaceHitToPoint(grabSurfaceTriangles,
                            sel.hitPointWorld,
                            expectedNormal,
                            g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
                            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees,
                            grabSurfaceHit, &mesh.queryIndex)) {
                        grabGripPoint = grabSurfaceHit.position;
                        selectionToMeshDistanceGameUnits = pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint);
                        grabSurfaceHit.hasSelectionHit = true;
                        grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                        grabSurfaceHit.signedAlongPalmDistanceGameUnits = sel.signedAlongDistance;
                        grabSurfaceHit.lateralPalmDistanceGameUnits = sel.lateralDistance;
                        grabSurfaceHit.shapeKey = sel.hitShapeKey;
                        grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
                        grabSurfaceHit.hitFraction = sel.hitFraction;
                        grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
                        surfaceOwnerNode = grabSurfaceHit.sourceNode;
                        meshGrabFound = true;
                        grabPointMode = "selectionHitMeshSnap";
                        grabPointAuthoritySource = GrabPivotAuthoritySource::SelectionHitMeshSnap;
                        grabFallbackReason = "selectionHitMeshSnap";
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand MESH GRAB: mode={} tris={} closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' shape='{}' selectionDelta={:.1f}gu surfaceAlong={:.1f} surfaceLateral={:.1f}",
                            handName(),
                            grabPointMode,
                            grabMeshTriangles.size(),
                            grabSurfaceHit.position.x,
                            grabSurfaceHit.position.y,
                            grabSurfaceHit.position.z,
                            grabSurfaceHit.triangleIndex,
                            grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                            nodeDebugName(grabSurfaceHit.sourceNode),
                            nodeDebugName(grabSurfaceHit.sourceShape),
                            selectionToMeshDistanceGameUnits,
                            grabSurfaceHit.signedAlongPalmDistanceGameUnits,
                            grabSurfaceHit.lateralPalmDistanceGameUnits);
                    }
                }

                if (!meshGrabFound) {
                    grabFallbackReason = grabSurfaceTriangles.empty() ? "noTriangles" : "noMeshNearPalmOrSelection";
                }
            }
            if (!meshGrabFound && collisionSurfaceHit.valid) {
                // Object collision can sit off the visible surface; it is still the
                // object's own physical surface and a valid place to hold it.
                grabSurfaceHit = collisionSurfaceHit;
                grabGripPoint = grabSurfaceHit.position;
                surfaceOwnerNode = grabSurfaceHit.sourceNode;
                meshGrabFound = true;
                grabPointMode = "collisionSurfaceFallback";
                grabPointAuthoritySource = GrabPivotAuthoritySource::CollisionFallback;
            }
            if (!meshGrabFound) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB POINT FALLBACK: mode={} reason={} meshNode='{}' ownerNode='{}' rootNode='{}' "
                    "shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} "
                    "fallbackPoint=({:.1f},{:.1f},{:.1f})",
                    handName(), grabPointMode, grabFallbackReason, nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes,
                    meshStats.staticShapes, meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                    meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
            }

    }

    struct Hand::GrabBodyResolution
    {
        object_physics_body_set::PrimaryBodyChoice primaryChoice{};
        mechanical_connected_body_set::MechanicalScope mechanicalScope{};
        RE::NiPoint3 primaryChoiceTarget{};
        bool surfaceOwnerMatchesResolvedBody = true;
        bool relaxedArticulatedAuthority = false;
    };

    void Hand::resolveGrabBodyAndContactPolicy(
        const ValidatedGrabSelection& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
        const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& activeLifecycle,
        const GrabProxyPreparation& proxy,
        GrabSurfaceEvidence& surface,
        GrabBodyResolution& outResolution)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabBodyResolution);
        outResolution = {};
        const auto& selectedObject = _currentSelection;
        outResolution.primaryChoiceTarget = (surface.meshGrabFound ||
            surface.pointAuthoritySource == GrabPivotAuthoritySource::LooseWeaponPrimaryAttach) ?
            surface.gripPoint :
            (selectedObject.hasHitPoint ? selectedObject.hitPointWorld : proxy.grabPivotAForPrimaryChoice);
        const auto nearestPrimaryChoice = preparedBodySet.choosePrimaryBody(
            object_physics_body_set::INVALID_BODY_ID,
            object_physics_body_set::PurePoint3{ outResolution.primaryChoiceTarget });
        const bool ragdollGrab = grab_target::isRagdoll(selectedObject.targetKind);
        const auto* surfaceOwnerRecord = ragdollGrab ? preparedBodySet.findAcceptedRagdollOwner(surface.surfaceOwnerNode) :
            preparedBodySet.findAcceptedRecordByOwnerNode(surface.surfaceOwnerNode);
        const auto skinnedResolution = skinned_body_resolver::resolvePrimaryBody(skinned_body_resolver::ResolutionInput{
            .targetKind = selectedObject.targetKind,
            .surfaceOwnerBodyId = surfaceOwnerRecord ? surfaceOwnerRecord->bodyId : object_physics_body_set::INVALID_BODY_ID,
            .selectedBodyId = selectedObject.bodyId.value,
            .nearestBodyId = nearestPrimaryChoice.bodyId,
            .surfaceOwnerUsable = surfaceOwnerRecord != nullptr,
            .selectedUsable = preparedBodySet.containsAcceptedBody(selectedObject.bodyId.value),
            .nearestUsable = nearestPrimaryChoice.bodyId != object_physics_body_set::INVALID_BODY_ID,
            .surfaceIsSkinned = surface.surfaceHit.valid && surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned,
            .hasSkinInfluences = surface.surfaceHit.valid && surface.surfaceHit.hasSkinInfluences,
        });
        outResolution.primaryChoice.bodyId = skinnedResolution.bodyId;
        outResolution.primaryChoice.reason =
            skinnedResolution.source == skinned_body_resolver::ResolutionSource::WeightedSkinOwner ||
                skinnedResolution.source == skinned_body_resolver::ResolutionSource::TriangleOwner ?
            object_physics_body_set::PrimaryBodyChoiceReason::SurfaceOwnerAccepted :
            (skinnedResolution.source == skinned_body_resolver::ResolutionSource::SelectedBody ?
                    object_physics_body_set::PrimaryBodyChoiceReason::PreferredHitAccepted :
                    (skinnedResolution.source == skinned_body_resolver::ResolutionSource::NearestAccepted ?
                            object_physics_body_set::PrimaryBodyChoiceReason::NearestAcceptedFallback :
                            object_physics_body_set::PrimaryBodyChoiceReason::NoAcceptedBody));

        if (surface.surfaceHit.valid) {
            const bool handPocketPositionOnlySkinnedSurface =
                selection.handPocketOnlyGrab &&
                surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned &&
                !surface.surfaceHit.hasSkinInfluences;
            if (surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::CollisionQuery) {
                outResolution.surfaceOwnerMatchesResolvedBody =
                    outResolution.primaryChoice.bodyId == selectedObject.bodyId.value;
            } else if (handPocketPositionOnlySkinnedSurface) {
                outResolution.surfaceOwnerMatchesResolvedBody =
                    outResolution.primaryChoice.bodyId == selectedObject.bodyId.value &&
                    preparedBodySet.containsAcceptedBody(selectedObject.bodyId.value);
            } else {
                outResolution.surfaceOwnerMatchesResolvedBody =
                    (surfaceOwnerRecord && surfaceOwnerRecord->bodyId == outResolution.primaryChoice.bodyId) ||
                    acceptsSelectedOwnerlessVisualMesh(
                        selectedObject,
                        preparedBodySet,
                        outResolution.primaryChoice.bodyId,
                        surface.surfaceOwnerNode,
                        surfaceOwnerRecord);
                if (outResolution.surfaceOwnerMatchesResolvedBody && !surfaceOwnerRecord &&
                    preparedBodySet.records.size() == 1) {
                    performance_profiler::addCounter(performance_profiler::Counter::GrabSingleBodyVisualOwnerAccepted);
                }
            }
            surface.surfaceHit.resolvedOwnerMatchesBody = outResolution.surfaceOwnerMatchesResolvedBody;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB BODY RESOLUTION: selectedBody={} resolvedBody={} reason={} resolver={} resolverReason={} skin={} sourceNode='{}' sourceKind={} ownerMatch={} target=({:.1f},{:.1f},{:.1f})",
            handName(),
            selectedObject.bodyId.value,
            outResolution.primaryChoice.bodyId,
            primaryBodyChoiceReasonName(outResolution.primaryChoice.reason),
            skinned_body_resolver::sourceName(skinnedResolution.source),
            skinnedResolution.reason,
            skinnedResolution.usedSkinInfluences ? "weighted" :
                (surface.surfaceHit.valid && surface.surfaceHit.sourceKind == GrabSurfaceSourceKind::Skinned ? "positionOnly" : "no"),
            nodeDebugName(surface.surfaceOwnerNode),
            surface.surfaceHit.valid ? grabSurfaceSourceKindName(surface.surfaceHit.sourceKind) : "fallback",
            outResolution.surfaceOwnerMatchesResolvedBody ? "yes" : "no",
            outResolution.primaryChoiceTarget.x,
            outResolution.primaryChoiceTarget.y,
            outResolution.primaryChoiceTarget.z);

        if (ragdollGrab && (!surface.meshGrabFound || !surface.surfaceHit.valid || !surfaceOwnerRecord)) {
            outResolution.primaryChoice.bodyId = INVALID_BODY_ID;
            outResolution.surfaceOwnerMatchesResolvedBody = false;
            ROCK_LOG_WARN(Hand, "{} ragdoll grab rejected: no posed surface with exact limb owner selected={}", handName(), selectedObject.bodyId.value);
            return;
        }
        if (outResolution.primaryChoice.bodyId != INVALID_BODY_ID) {
            if (!ragdollGrab) {
                outResolution.mechanicalScope = mechanical_connected_body_set::buildFromPreparedBodySet(
                    beforePrepBodySet,
                    preparedBodySet,
                    outResolution.primaryChoice.bodyId,
                    selectedObject.targetKind,
                    activeLifecycle.hasIncompleteNativeScan());
                outResolution.relaxedArticulatedAuthority = outResolution.mechanicalScope.strictPocketAuthorityRelaxed;
            }
            if (ragdollGrab) {
                const auto component = ragdoll::readComponent(havok_runtime::getHknpWorldFromBhk(selection.bhkWorld), outResolution.primaryChoice.bodyId);
                if (!component.valid || component.count > MAX_HELD_BODIES) {
                    ROCK_LOG_WARN(Hand, "{} ragdoll grab rejected: topology stage={} body={}", handName(), component.reason, outResolution.primaryChoice.bodyId);
                    outResolution.primaryChoice.bodyId = INVALID_BODY_ID;
                    return;
                }
                auto& scope = outResolution.mechanicalScope;
                scope.primaryBodyId = outResolution.primaryChoice.bodyId;
                for (std::size_t i = 0; i < component.count; ++i) {
                    scope.committedBodyIds.push_back(component.bodies[i].body.id);
                    bool uniqueMotion = true;
                    for (std::size_t j = 0; j < i; ++j) {
                        uniqueMotion = uniqueMotion && component.bodies[j].body.motion != component.bodies[i].body.motion;
                    }
                    if (uniqueMotion) ++scope.uniqueMotionCount;
                }
                scope.acceptedBodyCount = static_cast<std::uint32_t>(component.count);
                scope.targetPrefersMechanical = true;
                scope.strictPocketAuthorityRelaxed = true;
                scope.kind = component.fixedAttached ? mechanical_connected_body_set::ScopeKind::FixedAttached :
                    mechanical_connected_body_set::ScopeKind::ArticulatedMechanical;
                scope.driveDecision.mode = component.fixedAttached ? held_object_drive_policy::HeldBodySetDriveMode::FixedAttached :
                    held_object_drive_policy::HeldBodySetDriveMode::ComplexArticulated;
                scope.driveDecision.reason = component.reason;
                scope.driveDecision.includeConnectedLinearVelocity = false;
                scope.driveDecision.includeConnectedAngularVelocity = false;
                scope.driveDecision.includeConnectedMass = !component.fixedAttached;
                scope.reason = component.reason;
                outResolution.surfaceOwnerMatchesResolvedBody = true;
                surface.surfaceHit.resolvedOwnerMatchesBody = true;
                // Corpse support may be position-only, but its posed surface
                // and native body identity above are mandatory.
                outResolution.relaxedArticulatedAuthority = true;
            }
        }
    }

    struct Hand::ResolvedGrabBodyCapture
    {
        RE::hknpBodyId bodyId{};
        RE::NiAVObject* collidableNode = nullptr;
        RE::NiTransform objectWorldTransform{};
        std::vector<GrabLocalTriangle> localMeshTriangles{};
    };

    bool Hand::captureResolvedGrabBody(
        RE::hknpWorld* world,
        const GrabMeshCaptureSetup& meshCapture,
        const GrabBodyResolution& resolution,
        const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
        const std::vector<TriangleData>& meshTriangles,
        const RE::NiPointer<RE::TESObjectREFR>& selectedRef,
        std::uint16_t selectedOriginalMotionPropsId,
        const std::string& objectName,
        ResolvedGrabBodyCapture& outCapture)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabResolvedBodyCapture);
        outCapture = {};
        outCapture.bodyId = RE::hknpBodyId{ resolution.primaryChoice.bodyId };
        outCapture.collidableNode = meshCapture.collidableNode;
        outCapture.objectWorldTransform = meshCapture.objectWorldTransform;

        auto* preparedBody = havok_runtime::getBody(world, outCapture.bodyId);
        if (!preparedBody) {
            ROCK_LOG_ERROR(Hand,
                "{} grabSelectedObject: prepared primary body {} is not readable after object prep",
                handName(),
                outCapture.bodyId.value);
            return false;
        }

        const auto& selectedObject = _currentSelection;
        _savedObjectState.bodyId = outCapture.bodyId;
        _savedObjectState.setReference(selectedRef);
        _savedObjectState.targetKind = selectedObject.targetKind;
        _savedObjectState.originalFilterInfo = preparedBody->collisionFilterInfo;
        _savedObjectState.originalMotionPropsId = selectedOriginalMotionPropsId;
        if (const auto* originalPrimaryRecord = beforePrepBodySet.findRecord(outCapture.bodyId.value)) {
            _savedObjectState.originalMotionPropsId = originalPrimaryRecord->motionPropertiesId;
        }

        auto* ownerCell = selectedObject.refr ? selectedObject.refr->GetParentCell() : nullptr;
        auto* ownerWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        auto* bodyCollisionObject = ownerWorld ? RE::bhkNPCollisionObject::Getbhk(ownerWorld, outCapture.bodyId) : nullptr;
        if (auto* resolvedOwnerNode = bodyCollisionObject ? bodyCollisionObject->sceneObject : nullptr) {
            GrabCaptureTransformRefreshResult ownerRefresh{};
            refreshGrabCaptureNodeTransform(ownerRefresh, "resolvedOwner", resolvedOwnerNode, !grab_target::isRagdoll(selectedObject.targetKind));
            if (!ownerRefresh.ok) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB failed: resolved owner transform refresh produced a non-finite node transform for '{}' formID={:08X}; owner='{}'",
                    handName(),
                    objectName,
                    selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                    nodeDebugName(resolvedOwnerNode));
                return false;
            }
            outCapture.collidableNode = resolvedOwnerNode;
            outCapture.objectWorldTransform = resolvedOwnerNode->world;
            if (!grab_three_phase::isFinite(outCapture.objectWorldTransform)) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB failed: resolved owner transform is non-finite for '{}' formID={:08X}; owner='{}'",
                    handName(),
                    objectName,
                    selectedObject.refr ? selectedObject.refr->GetFormID() : 0,
                    nodeDebugName(resolvedOwnerNode));
                return false;
            }
        }

        if (!meshTriangles.empty()) {
            outCapture.localMeshTriangles = cacheTrianglesInLocalSpace(meshTriangles, outCapture.objectWorldTransform);
        }
        return true;
    }

    struct Hand::GrabPivotEvidence
    {
        hand_semantic_contact_state::SemanticContactCollection semanticContacts{};
        GrabPivotAuthoritySource authoritySource = GrabPivotAuthoritySource::None;
        bool authorityNormalTrusted = false;
        bool authorityPositionOnly = false;
        float authorityPositionConfidence = 0.0f;
        float authorityPocketDistanceGameUnits = std::numeric_limits<float>::max();
        float authoritySelectionDeltaGameUnits = std::numeric_limits<float>::max();
        float authorityLongLeverGameUnits = 0.0f;
    };

    void Hand::resolveGrabPivotEvidence(
        const GrabProxyPreparation& proxy,
        const ResolvedGrabBodyCapture& bodyCapture,
        const GrabSurfaceEvidence& surface,
        GrabPivotEvidence& outEvidence)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabPivotEvidence);
        outEvidence = {};
        const auto& sel = _currentSelection;
        /*
         * The surface resolver already chose the single grab point. This stage
         * only records the hand contacts touching the object (they decide
         * whether a pulled object counts as touching on arrival) and describes
         * that point for the seat and held drive.
         */
        outEvidence.semanticContacts = collectFreshSemanticContactsForBodyWithinSeconds(
            bodyCapture.bodyId.value,
            (std::max)(0.0f, g_rockConfig.rockGrabOppositionContactMaxAgeSeconds));
        const auto pocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
            proxy.proxyAuthorityFrameWorldAtGrab,
            _isLeft,
            proxy.palmPocketPivotAWorld,
            g_rockConfig.rockGrabPocketDepthGameUnits,
            g_rockConfig.rockGrabPocketRadiusGameUnits);
        outEvidence.authoritySource = surface.pointAuthoritySource;
        outEvidence.authorityNormalTrusted = surface.surfaceHit.valid;
        outEvidence.authorityPositionConfidence = surface.surfaceHit.valid ? 1.0f : 0.0f;
        outEvidence.authorityPocketDistanceGameUnits =
            pocket.valid ? pointDistanceGameUnits(surface.gripPoint, pocket.palmCenterWorld) : std::numeric_limits<float>::max();
        outEvidence.authoritySelectionDeltaGameUnits =
            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, surface.gripPoint) : std::numeric_limits<float>::max();
    }

    struct Hand::GrabCommitPreparationInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        RE::NiAVObject* collidableNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        const std::string* objectName = nullptr;
        const mechanical_connected_body_set::MechanicalScope* mechanicalScope = nullptr;
        const GrabSharedObjectContext* sharedContext = nullptr;
        const object_physics_body_set::ObjectPhysicsBodySet* preparedBodySet = nullptr;
        bool joiningPeerHeldObject = false;
    };

    void Hand::beginResolvedGrabCommit(const GrabCommitPreparationInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabCommitPreparation);
        const auto& handWorldTransform = *input.handWorldTransform;
        auto* collidableNode = input.collidableNode;
        auto objectBodyId = input.objectBodyId;
        const auto& objectName = *input.objectName;
        const auto& mechanicalScope = *input.mechanicalScope;
        const auto& sharedContext = *input.sharedContext;
        const auto& preparedBodySet = *input.preparedBodySet;
        const bool joiningPeerHeldObject = input.joiningPeerHeldObject;
        logRuntimeScaleIfChanged(_isLeft, handName(), handWorldTransform, collidableNode);
        ROCK_LOG_INFO(Hand,
            "{} hand GRAB: '{}' formID={:08X} bodyId={}",
            handName(),
            objectName,
            _currentSelection.refr->GetFormID(),
            objectBodyId.value);


        bool adoptedPeerHeldBodySet = false;
        _heldBodyIds = grab_target::isRagdoll(_currentSelection.targetKind) ?
            held_object_body_set_policy::makePrimaryFirstUniqueBodyList(objectBodyId.value, mechanicalScope.committedBodyIds) :
            buildCommittedHeldBodyIds(objectBodyId.value, mechanicalScope.committedBodyIds, sharedContext, adoptedPeerHeldBodySet);
        if (_heldBodyIds.empty()) {
            _heldBodyIds.push_back(objectBodyId.value);
        }
        _heldDriveDecision = mechanicalScope.driveDecision;
        if (adoptedPeerHeldBodySet) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand SHARED held body set adopted: primaryBody={} peerBodies={} committedBodies={} scanAccepted={} mechanicalKind={} driveMode={} driveReason={}",
                handName(),
                objectBodyId.value,
                sharedContext.peerHeldBodyIds ? sharedContext.peerHeldBodyIds->size() : 0,
                _heldBodyIds.size(),
                preparedBodySet.acceptedCount(),
                mechanical_connected_body_set::scopeKindName(mechanicalScope.kind),
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.reason);
        }

        if (auto* player = RE::PlayerCharacter::GetSingleton(); player && !joiningPeerHeldObject) {
            nativeVRGrabDrop(player, 0);
            nativeVRGrabDrop(player, 1);
        }

        _grabStartTime = 0.0f;
        _heldObjectUpdateFrame = 0;
        _grabConvergeStableInsidePocketSeconds = 0.0f;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
    }

    struct Hand::GrabBodyFrameCaptureInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        GrabSurfaceEvidence* surface = nullptr;
        const GrabPivotEvidence* pivotEvidence = nullptr;
        const GrabBodyResolution* bodyResolution = nullptr;
        const std::vector<GrabLocalTriangle>* localMeshTriangles = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        std::uint64_t traceId = 0;
        const std::string* objectName = nullptr;
        bool authoredSeatPrepared = false;
    };

    struct Hand::GrabBodyFrameCapture
    {
        RE::NiPoint3 palmPosition{};
        RE::NiTransform grabBodyWorld{};
        RE::NiTransform motionBodyWorld{};
        body_frame::BodyFrameSource motionBodySource = body_frame::BodyFrameSource::Fallback;
        bool hasMotionBodyWorld = false;
        bool constraintUsesMotionBody = false;
        RE::NiTransform constraintBodyWorld{};
        RE::NiPoint3 grabPivotAWorld{};
        RE::bhkNPCollisionObject* bodyCollisionObject = nullptr;
        RE::NiAVObject* ownerNode = nullptr;
        RE::NiTransform objectToBody{};
        RE::NiTransform ownerBodyLocal{};
        RE::NiTransform rootBodyLocal{};
        RE::NiPoint3 selectedGripPointLocal{};
        RE::NiPoint3 selectedPivotBBodyLocalGame{};
        std::vector<GrabLocalTriangle> fingerPoseLocalMeshTriangles{};
        bool clearExternalOnFailure = false;
    };

    bool Hand::captureGrabBodyFrame(
        RE::hknpWorld* world,
        const GrabBodyFrameCaptureInput& input,
        GrabBodyFrameCapture& outCapture)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabBodyFrameCapture);
        outCapture = {};
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& mesh = *input.mesh;
        auto& surface = *input.surface;
        const auto& pivotEvidence = *input.pivotEvidence;
        const auto& bodyResolution = *input.bodyResolution;
        const auto& grabLocalMeshTriangles = *input.localMeshTriangles;
        const auto& sel = _currentSelection;
        auto* collidableNode = meshCapture.collidableNode;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        auto& grabSurfaceHit = surface.surfaceHit;
        auto& grabGripPoint = surface.gripPoint;
        const auto pivotAuthoritySource = pivotEvidence.authoritySource;
        const bool pivotAuthorityPositionOnly = pivotEvidence.authorityPositionOnly;
        const bool pivotAuthorityNormalTrusted = pivotEvidence.authorityNormalTrusted;
        const float pivotAuthorityPositionConfidence = pivotEvidence.authorityPositionConfidence;
        const auto& primaryChoice = bodyResolution.primaryChoice;
        const auto& palmPocketPivotAWorld = proxy.palmPocketPivotAWorld;
        auto* rootNode = input.rootNode;
        auto objectBodyId = input.objectBodyId;
        const auto& objName = *input.objectName;
        const char* grabPointMode = surface.pointMode;
        auto& palmPos = outCapture.palmPosition;
        auto& grabBodyWorldAtGrab = outCapture.grabBodyWorld;
        auto& motionBodyWorldAtGrab = outCapture.motionBodyWorld;
        auto& motionBodySourceAtGrab = outCapture.motionBodySource;
        auto& hasMotionBodyWorldAtGrab = outCapture.hasMotionBodyWorld;
        auto& constraintUsesMotionBodyAtGrab = outCapture.constraintUsesMotionBody;
        auto& constraintBodyWorldAtGrab = outCapture.constraintBodyWorld;
        auto& grabPivotAWorld = outCapture.grabPivotAWorld;
        RE::TESObjectCELL* ownerCellAtGrab = nullptr;
        RE::bhkWorld* bhkWorldAtGrab = nullptr;
        auto*& bodyCollisionObjectAtGrab = outCapture.bodyCollisionObject;
        auto*& ownerNodeAtGrab = outCapture.ownerNode;
        auto& objectToBodyAtGrab = outCapture.objectToBody;
        auto& ownerBodyLocalAtGrab = outCapture.ownerBodyLocal;
        auto& rootBodyLocalAtGrab = outCapture.rootBodyLocal;
        auto& selectedGripPointLocal = outCapture.selectedGripPointLocal;
        auto& selectedPivotBBodyLocalGame = outCapture.selectedPivotBBodyLocalGame;
        std::vector<TriangleData> grabFingerPoseMeshTriangles;
        auto& grabFingerPoseLocalMeshTriangles = outCapture.fingerPoseLocalMeshTriangles;
                palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
                if (!tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, grabBodyWorldAtGrab)) {
                    ROCK_LOG_ERROR(Hand,
                        "{} hand GRAB FAILED: native grab BODY frame unreadable bodyId={} formID={:08X}",
                        handName(),
                        objectBodyId.value,
                        sel.refr ? sel.refr->GetFormID() : 0);
                    _grabFrame.clear();
                    _heldBodyIds.clear();
                    _heldDriveDecision = {};
                    _heldBodyIdsCount.store(0, std::memory_order_release);
                    return false;
                }
                hasMotionBodyWorldAtGrab =
                    tryResolveLiveBodyWorldTransform(world, objectBodyId, motionBodyWorldAtGrab, &motionBodySourceAtGrab);
                /*
                 * Custom constraint body-B data is authored in the hknp BODY frame.
                 * HIGGS does the same kind of thing on Skyrim's hkp side: it freezes
                 * pivot B from the rigid body transform and treats COM/motion as mass
                 * data, not as a local-frame owner. A runtime MOTION-frame experiment
                 * made the object settle at the wrong rotation even when the proxy
                 * target was stable, so the live motion transform remains diagnostic
                 * evidence only.
                 */
                constraintUsesMotionBodyAtGrab = false;
                constraintBodyWorldAtGrab = grabBodyWorldAtGrab;
                /*
                 * The hidden proxy is body A for dynamic grab. The close-grab
                 * pocket pivot is captured from the same generated/proxy-local
                 * seat frame. The proxy body keeps its seat offset, and transform A
                 * carries the selected pivot as an explicit local point on body A.
                 */
                grabPivotAWorld = palmPocketPivotAWorld;
                ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
                bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
                bodyCollisionObjectAtGrab = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
                ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
                if (ownerNodeAtGrab && ownerNodeAtGrab != collidableNode) {
                    GrabCaptureTransformRefreshResult ownerAtGrabRefresh{};
                    refreshGrabCaptureNodeTransform(ownerAtGrabRefresh, "ownerAtGrab", ownerNodeAtGrab, !grab_target::isRagdoll(sel.targetKind));
                    if (!ownerAtGrabRefresh.ok) {
                        ROCK_LOG_WARN(Hand,
                            "{} hand GRAB failed: owner-at-grab transform refresh produced a non-finite node transform for '{}' formID={:08X}; owner='{}'",
                            handName(),
                            objName,
                            sel.refr ? sel.refr->GetFormID() : 0,
                            nodeDebugName(ownerNodeAtGrab));
                        outCapture.clearExternalOnFailure = true;
                        return false;
                    }
                }
                _grabFrame.heldNode = collidableNode;
                objectToBodyAtGrab = computeRuntimeBodyLocalTransform(objectWorldTransform, grabBodyWorldAtGrab);
                /*
                 * ROCK dynamic grab has one production authority convention:
                 * the generated/proxy palm frame seats the selected BODY-local grip
                 * point and owns the object angular relation through a row-view of
                 * its generated local axes. The custom constraint stores body-B local
                 * data in the rigid BODY frame. MOTION and COM are mass/diagnostic
                 * data only.
                 */
                ownerBodyLocalAtGrab =
                    ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, grabBodyWorldAtGrab) : makeIdentityTransform();
                rootBodyLocalAtGrab =
                    rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, grabBodyWorldAtGrab) : makeIdentityTransform();
                selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
                selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
                if (!input.authoredSeatPrepared && !grabMeshTriangles.empty()) {
                    grabFingerPoseMeshTriangles = selectNearestGrabFingerPoseTriangles(
                        mesh.surfaceTriangles,
                        grabGripPoint,
                        kMaxGrabRuntimeFingerPoseTriangles, mesh.queryIndex);
                    grabFingerPoseLocalMeshTriangles = cacheTrianglesInLocalSpace(grabFingerPoseMeshTriangles, objectWorldTransform);
                    if (grabFingerPoseMeshTriangles.size() != grabMeshTriangles.size()) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand MESH FINGER POSE TRIANGLES: sourceTris={} localTris={} center=({:.1f},{:.1f},{:.1f})",
                            handName(),
                            grabMeshTriangles.size(),
                            grabFingerPoseMeshTriangles.size(),
                            grabGripPoint.x,
                            grabGripPoint.y,
                            grabGripPoint.z);
                    }
                }
                _grabFrame.localMeshTriangles.clear();
                _grabFrame.contactMeshCache.clear();
                _grabFrame.fingerPoseLocalMeshTriangles.clear();
                _grabFrame.gripEvidence.gripEvidenceLocal = selectedGripPointLocal;
                _grabFrame.gripEvidence.gripNormalLocal = grabSurfaceHit.valid ? transform_math::worldVectorToLocal(objectWorldTransform, grabSurfaceHit.normal) : RE::NiPoint3{};
                storeGripSourceEvidence(_grabFrame,
                    grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
                    objectWorldTransform,
                    grabGripPoint,
                    grabSurfaceHit.normal,
                    grabSurfaceHit.valid && lengthSquared(grabSurfaceHit.normal) > 0.000001f);
                _grabFrame.gripEvidence.gripEvidenceTriangleIndex = grabSurfaceHit.valid && grabSurfaceHit.hasTriangle ? static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex) : 0xFFFF'FFFF;
                _grabFrame.gripEvidence.gripEvidenceShapeKey = grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
                _grabFrame.gripEvidence.gripEvidenceShapeCollisionFilterInfo = grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
                _grabFrame.gripEvidence.gripEvidenceHitFraction = grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
                _grabFrame.gripEvidence.hasGripEvidenceShapeKey = grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
                _grabFrame.seat.palmSeatPointWorldAtGrab = grabGripPoint;
                _grabFrame.seat.hasPalmSeatPoint = input.authoredSeatPrepared || surface.meshGrabFound;
                _grabFrame.seat.activeGrabPointMode = grabPointMode;
                _grabFrame.pivotAuthority.source = pivotAuthoritySource;
                _grabFrame.pivotAuthority.positionOnly = pivotAuthorityPositionOnly;
                _grabFrame.pivotAuthority.normalTrusted = pivotAuthorityNormalTrusted;
                _grabFrame.pivotAuthority.positionConfidence = pivotAuthorityPositionConfidence;
                _grabFrame.seat.palmSeatPointMode = grabPointMode;
                _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(primaryChoice.reason);
                _grabFrame.pivotAuthority.pocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                _grabFrame.pivotAuthority.selectionDistanceGameUnits =
                    grabSurfaceHit.valid && grabSurfaceHit.hasSelectionHit ? grabSurfaceHit.selectionToMeshDistanceGameUnits : (sel.hasHitPoint ? 0.0f : std::numeric_limits<float>::max());
                if (grabSurfaceHit.valid) {
                    grabSurfaceHit.pivotToSurfaceDistanceGameUnits = _grabFrame.pivotAuthority.pocketDistanceGameUnits;
                }
                _grabFrame.hasMeshPoseData = false;
                if (!grabLocalMeshTriangles.empty()) {
                    _grabFrame.localMeshTriangles = grabLocalMeshTriangles;
                    _grabFrame.contactMeshCache.clear();
                }
                if (!grabFingerPoseLocalMeshTriangles.empty()) {
                    _grabFrame.fingerPoseLocalMeshTriangles = grabFingerPoseLocalMeshTriangles;
                }
                _grabFrame.hasMeshPoseData =
                    !_grabFrame.localMeshTriangles.empty() ||
                    !_grabFrame.fingerPoseLocalMeshTriangles.empty();

        return true;
    }

    struct Hand::GrabSeatCaptureInput
    {
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        GrabSurfaceEvidence* surface = nullptr;
        GrabPivotEvidence* pivotEvidence = nullptr;
        GrabBodyFrameCapture* bodyFrame = nullptr;
        const RuntimePinchPocketCandidate* pinchPocketCandidate = nullptr;
        saved_grab_offset::HandOffset* savedGrabOffset = nullptr;
        bool* hasSavedGrabOffset = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::NiAVObject* collidableNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        bool grabbedFromPullCatch = false;
        bool looseWeaponGrab = false;
        bool joiningPeerHeldObject = false;
        const AuthoredWeaponGripPose* transferPose = nullptr;
        const loose_weapon_grip_zone::NearGrab* preparedAuthoredGrip = nullptr;
        RE::NiPoint3 canonicalPivotNormalWorld{};
        GrabRollbackAction rollback{};
    };

    struct Hand::GrabSeatCaptureResult
    {
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform authoredHandWeaponLocal{};
        bool hasAuthoredHandRelation = false;
        GrabPivotAuthoritySource resolvedAuthoritySource = GrabPivotAuthoritySource::None;
        const char* resolvedAuthorityReason = "notResolved";
    };

    bool Hand::resolveGrabSeatCapture(
        RE::hknpWorld* world,
        const GrabSeatCaptureInput& input,
        GrabSeatCaptureResult& outCapture)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabSeatCapture);
        outCapture = {};
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& mesh = *input.mesh;
        auto& surface = *input.surface;
        auto& pivotEvidence = *input.pivotEvidence;
        auto& bodyFrame = *input.bodyFrame;
        const auto& pinchPocketCandidate = *input.pinchPocketCandidate;
        auto& savedGrabOffset = *input.savedGrabOffset;
        auto& hasSavedGrabOffset = *input.hasSavedGrabOffset;
        const auto& sel = _currentSelection;
        const auto& proxyAuthorityFrameWorldAtGrab = proxy.proxyAuthorityFrameWorldAtGrab;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        const auto& grabLocalMeshTriangles = _grabFrame.localMeshTriangles;
        auto& grabGripPoint = surface.gripPoint;
        auto& selectionToMeshDistanceGameUnits = surface.selectionToMeshDistanceGameUnits;
        auto& grabSurfaceHit = surface.surfaceHit;
        const char*& grabPointMode = surface.pointMode;
        auto& grabPointAuthoritySource = surface.pointAuthoritySource;
        const char*& grabFallbackReason = surface.fallbackReason;
        auto& pivotAuthoritySource = pivotEvidence.authoritySource;
        auto& pivotAuthorityNormalTrusted = pivotEvidence.authorityNormalTrusted;
        auto& pivotAuthorityPositionOnly = pivotEvidence.authorityPositionOnly;
        auto& pivotAuthorityPositionConfidence = pivotEvidence.authorityPositionConfidence;
        auto& pivotAuthorityPocketDistanceGameUnits = pivotEvidence.authorityPocketDistanceGameUnits;
        auto& pivotAuthoritySelectionDeltaGameUnits = pivotEvidence.authoritySelectionDeltaGameUnits;
        auto& pivotAuthorityLongLeverGameUnits = pivotEvidence.authorityLongLeverGameUnits;
        const auto& semanticContacts = pivotEvidence.semanticContacts;
        auto& grabBodyWorldAtGrab = bodyFrame.grabBodyWorld;
        auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        auto& selectedGripPointLocal = bodyFrame.selectedGripPointLocal;
        auto& selectedPivotBBodyLocalGame = bodyFrame.selectedPivotBBodyLocalGame;
        const auto& objectToBodyAtGrab = bodyFrame.objectToBody;
        const auto& rootBodyLocalAtGrab = bodyFrame.rootBodyLocal;
        const auto objectBodyId = input.objectBodyId;
        auto* rootNode = input.rootNode;
        auto* collidableNode = input.collidableNode;
        const bool grabbedFromPullCatch = input.grabbedFromPullCatch;
        const bool looseWeaponGrab = input.looseWeaponGrab;
        loose_weapon_grip_zone::NearGrab nearGrip{};
        const auto pocketAtAcquisition = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
            proxyAuthorityFrameWorldAtGrab, _isLeft, grabPivotAWorld,
            g_rockConfig.rockGrabPocketDepthGameUnits, g_rockConfig.rockGrabPocketRadiusGameUnits);
        const auto pocketDelta = grabGripPoint - pocketAtAcquisition.palmCenterWorld;
        const bool touchingPocket = pocketAtAcquisition.valid && semanticContacts.count != 0 &&
            pointDistanceGameUnits(grabGripPoint, pocketAtAcquisition.palmCenterWorld) <= pocketAtAcquisition.pocketRadiusGameUnits &&
            dot(pocketDelta, pocketAtAcquisition.palmNormalWorld) >= -g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits;
        const bool authoredWeapon = looseWeaponGrab && !isThrowableLooseWeapon(selectedLooseWeaponForm(sel));
        bool requestAuthoredPose = false;
        bool nearAuthored = false;
        if (input.preparedAuthoredGrip) {
            nearGrip = *input.preparedAuthoredGrip;
            requestAuthoredPose = true;
            const auto* weapon = selectedLooseWeaponForm(sel);
            nearAuthored = authoredWeapon && weapon && sel.refr->Get3D() == rootNode &&
                nearGrip.handWorldValid && nearGrip.pose.valid() &&
                nearGrip.pose.weaponFormId == weapon->formID && nearGrip.pose.isLeft == _isLeft &&
                nearGrip.pose.role == nearGrip.role;
            if (!nearAuthored) {
                performance_profiler::addCounter(performance_profiler::Counter::GrabAuthoredPreparationInvalidated);
            }
        } else if (authoredWeapon) {
            namespace policy = loose_weapon_authored_grab_policy;
            (void)loose_weapon_grip_zone::tryResolveNearGrab(_isLeft, sel.refr, nearGrip, input.joiningPeerHeldObject);
            const auto role = policy::acquisitionRole(touchingPocket && !grabbedFromPullCatch && !sel.forcedArrival,
                input.transferPose != nullptr, input.transferPose ? input.transferPose->role : policy::Role::None,
                nearGrip.arrangement, input.joiningPeerHeldObject, nearGrip.role);
            const bool dynamicOffhand = policy::dynamicSupportAcquisition(
                role, input.joiningPeerHeldObject, nearGrip.arrangement, input.transferPose != nullptr);
            ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                "{} loose weapon role selection ref={:08X} peerHolding={} touchingPocket={} pull={} forced={} transfer={} arrangement={} zoneRole={} selectedRole={} dynamicSupport={}",
                handName(), sel.refr ? sel.refr->GetFormID() : 0, input.joiningPeerHeldObject,
                touchingPocket, grabbedFromPullCatch, sel.forcedArrival, input.transferPose != nullptr,
                static_cast<unsigned>(nearGrip.arrangement), static_cast<unsigned>(nearGrip.role),
                static_cast<unsigned>(role), dynamicOffhand);
            requestAuthoredPose = (role != policy::Role::None || input.transferPose != nullptr) && !dynamicOffhand;
            nearGrip.role = role;
            // Retain the controller-derived frame resolved by the zone runtime.
            // The incoming grab wrist can already be presented by ROCK.
            if (input.transferPose) {
                nearGrip.pose = *input.transferPose;
                const auto* weapon = selectedLooseWeaponForm(sel);
                nearAuthored = nearGrip.pose.valid() && weapon && nearGrip.pose.weaponFormId == weapon->formID &&
                    nearGrip.pose.isLeft == _isLeft;
            } else if (requestAuthoredPose) {
                nearAuthored = loose_weapon_grip_zone::tryResolveAuthoredGrabPose(_isLeft, sel.refr, role, nearGrip.pose);
            }
            nearAuthored = nearAuthored && nearGrip.handWorldValid;
        }
        const RE::NiPoint3 canonicalPivotNormalWorld = input.canonicalPivotNormalWorld;
        auto& desiredObjectWorld = outCapture.desiredObjectWorld;
        auto& desiredBodyWorld = outCapture.desiredBodyWorld;
        auto& resolvedAuthorityPivotSourceForFreeze = outCapture.resolvedAuthoritySource;
        auto& resolvedAuthorityPivotReasonForFreeze = outCapture.resolvedAuthorityReason;
        auto abortCapture = [&]() {
            _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
            _grabObjectGripAtGrab = {};
            _heldObjectIsLooseWeapon = false;
            _grabFrame.clear();
            _heldBodyIds.clear();
            _heldDriveDecision = {};
            _heldBodyIdsCount.store(0, std::memory_order_release);
            _grabFingerPosePublished = false;
            (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
            clearGrabExternalHandWorldTransform(_isLeft);
            input.rollback.rollback();
            return false;
        };
                desiredObjectWorld = objectWorldTransform;
                if (input.preparedAuthoredGrip && !nearAuthored) {
                    // The authored pose is unavailable now; hold the weapon dynamically.
                    ROCK_LOG_SAMPLE_INFO(Hand, 1000, "{} authored weapon pose unavailable; dynamic grab ref={:08X} role={} transfer={}",
                        handName(), sel.refr ? sel.refr->GetFormID() : 0, static_cast<unsigned>(nearGrip.role), input.transferPose != nullptr);
                }
                desiredBodyWorld = grabBodyWorldAtGrab;
                bool looseWeaponPrimaryAttachApplied = false;
                bool looseWeaponPrimaryAttachSourceVisible = false;
                const char* looseWeaponPrimaryAttachReason = "notEvaluated";
                const auto oppositionContacts = hand_semantic_contact_state::selectThumbOppositionContacts(semanticContacts);
                resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::None;
                resolvedAuthorityPivotReasonForFreeze = "notResolved";
                /*
                 * Runtime grab authority is intentionally single-source. Mesh hits,
                 * authored nodes, semantic contacts, contact patches, and multi-finger
                 * contacts are evidence until the pivot resolver chooses one BODY-local
                 * point to freeze. Final dynamic authority is strict: only a pinch
                 * pocket or an authorable support group may create the frozen frame.
                 */
                {
                    auto pocket = pocketAtAcquisition;
                    auto gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                        .objectBodyWorld = grabBodyWorldAtGrab,
                        .contactSeedWorld = grabGripPoint,
                        .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                        .interiorDirectionWorld = RE::NiPoint3{},
                        .preferredInsetGameUnits = 0.0f,
                        .maxInsetGameUnits = 0.0f,
                        .centerOfMassValid = false,
                        .interiorDirectionValid = false,
                        .source = grabPointMode,
                    });
                    if (!nearAuthored && pinchPocketCandidate.valid) {
                        gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                            .objectBodyWorld = grabBodyWorldAtGrab,
                            .contactSeedWorld = pinchPocketCandidate.surfaceHit.position,
                            .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                            .interiorDirectionWorld = RE::NiPoint3{},
                            .preferredInsetGameUnits = 0.0f,
                            .maxInsetGameUnits = 0.0f,
                            .centerOfMassValid = false,
                            .interiorDirectionValid = false,
                            .source = "pinchPocket",
                        });
                    }
                    const RE::NiPoint3 phaseGripSeed = gripArea.valid ? gripArea.contactSeedWorld : grabGripPoint;
                    const RE::NiPoint3 palmSeatAnchorWorld = pocket.valid ? pocket.palmCenterWorld : grabPivotAWorld;
                    const RE::NiPoint3 gripToPocket = pocket.valid ? (phaseGripSeed - palmSeatAnchorWorld) : RE::NiPoint3{};
                    const float gripToPocketDistance =
                        pocket.valid ? std::sqrt(gripToPocket.x * gripToPocket.x + gripToPocket.y * gripToPocket.y + gripToPocket.z * gripToPocket.z) :
                                       std::numeric_limits<float>::max();
                    const float stableTouchEnvelope = (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, pocket.pocketRadiusGameUnits);
                    const bool hasStablePocketTouchContact =
                        oppositionContacts.valid && gripToPocketDistance <= stableTouchEnvelope;
                    const bool meshSurfaceAuthorityEvidence =
                        grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
                    const bool programmaticArrival = grabbedFromPullCatch || sel.forcedArrival;
                    const bool pullArrivalTouchHeldAuthorityEvidence =
                        hasStablePocketTouchContact ||
                        meshSurfaceAuthorityEvidence;
                    /*
                     * Mesh/support evidence proves that ROCK found an object-side
                     * seat; it does not prove that a forced arrival has reached the
                     * hand. Pull catch may retain its flight evidence, but a direct
                     * forced arrival may enter TouchHeld immediately only from real
                     * stable hand contact. Otherwise it must converge to the seated
                     * relation before final hand-pose publication.
                     */
                    const bool programmaticTouchHeldAuthorityEvidence =
                        grabbedFromPullCatch ?
                            pullArrivalTouchHeldAuthorityEvidence :
                            (!sel.forcedArrival || hasStablePocketTouchContact);
                    const auto phaseDecision = grab_three_phase::classifyAcquisitionPhase(grab_three_phase::PhaseClassificationInput{
                        .pocket = pocket,
                        .gripSeedWorld = phaseGripSeed,
                        .hasFreshTouchContact = hasStablePocketTouchContact,
                        .isFarSelection = sel.isFarSelection,
                        .programmaticArrival = programmaticArrival,
                        .requireEvidenceForTouchHeld = programmaticArrival,
                        .hasTouchHeldAuthorityEvidence = programmaticTouchHeldAuthorityEvidence,
                        .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                        .touchContactMaxDistanceGameUnits = stableTouchEnvelope,
                        .nearConvergeDistanceGameUnits = g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                        .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                    });

                    const bool usingPinchPocket = !nearAuthored && pinchPocketCandidate.valid && gripArea.valid;
                    const bool captureAccepted = usingPinchPocket || (pocket.valid && gripArea.valid && (nearAuthored || phaseDecision.accepted));
                    if (captureAccepted) {
                        _grabObjectGripAtGrab = gripArea;
                        _grabAcquisitionPhase = nearAuthored ? grab_three_phase::AcquisitionPhase::NearConverging :
                            (usingPinchPocket ? grab_three_phase::AcquisitionPhase::TouchHeld : phaseDecision.phase);
                        if (usingPinchPocket) {
                            _grabObjectGripAtGrab.source = "pinchPocket";
                            _grabObjectGripAtGrab.fallbackReason = pinchPocketCandidate.decision.reason;
                            _grabObjectGripAtGrab.confidence = 0.95f;
                        }
                        const char* relationMode = usingPinchPocket ? "pinchPocket" : "rockPointToPalm";
                        const char* captureReason = usingPinchPocket ? pinchPocketCandidate.decision.reason : phaseDecision.reason;

                        if (usingPinchPocket) {
                            grabPivotAWorld = pinchPocketCandidate.pinchPocketWorld;
                        } else {
                            grabPivotAWorld = pocket.palmCenterWorld;
                        }
                        grabGripPoint = gripArea.contactSeedWorld;
                        grabPointMode = relationMode;
                        grabPointAuthoritySource = usingPinchPocket ?
                            GrabPivotAuthoritySource::PinchPocketMeshPoint :
                            GrabPivotAuthoritySource::PalmPocketMeshPoint;
                        grabFallbackReason = captureReason;
                        if (usingPinchPocket) {
                            grabSurfaceHit = pinchPocketCandidate.surfaceHit;
                            grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                            grabSurfaceHit.selectionToMeshDistanceGameUnits =
                                sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                            grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                            grabSurfaceHit.resolvedOwnerMatchesBody = true;
                            selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                            pivotAuthoritySource = GrabPivotAuthoritySource::PinchPocketMeshPoint;
                            pivotAuthorityPositionOnly = false;
                            pivotAuthorityNormalTrusted = true;
                            pivotAuthorityPositionConfidence = 0.95f;
                            pivotAuthorityPocketDistanceGameUnits = grabSurfaceHit.pivotToSurfaceDistanceGameUnits;
                            pivotAuthoritySelectionDeltaGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                        }

                        auto firstValidNormal = [](std::initializer_list<RE::NiPoint3> candidates) {
                            for (const auto& candidate : candidates) {
                                const RE::NiPoint3 normal = normalizeOrZero(candidate);
                                if (lengthSquared(normal) > 0.000001f) {
                                    return normal;
                                }
                            }
                            return RE::NiPoint3{};
                        };

                        /*
                         * Pivot B may be chosen from hand/proxy proximity, but normal
                         * authority must come from the object surface that produced the
                         * pivot. The palm/proxy normal is only a final fallback and
                         * tangent/sign guide; promoting it to face authority makes the
                         * frozen support frame follow the hand instead of the mesh.
                         */
                        RE::NiPoint3 gripNormalWorld = firstValidNormal({
                            grabSurfaceHit.valid ? grabSurfaceHit.normal : RE::NiPoint3{},
                            canonicalPivotNormalWorld,
                            usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
                            pocket.palmNormalWorld,
                        });

                        RE::NiPoint3 gripEvidencePointWorld = grabGripPoint;
                        RE::NiPoint3 gripEvidenceNormalWorld = gripNormalWorld;
                        const float gripObjectScale =
                            std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
                        // Furthest mesh point from the grip: the release spin cap
                        // for long objects reads this lever.
                        const float gripLeverGameUnits =
                            !grabLocalMeshTriangles.empty() && grab_three_phase::isFinite(objectWorldTransform) ?
                                computeLocalMeshMaxDistanceFromPoint(
                                    grabLocalMeshTriangles,
                                    transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint)) *
                                    gripObjectScale :
                                0.0f;
                        if (usingPinchPocket) {
                            resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::PinchPocketMeshPoint;
                            resolvedAuthorityPivotReasonForFreeze = pinchPocketCandidate.decision.reason ? pinchPocketCandidate.decision.reason : "none";
                        } else {
                            resolvedAuthorityPivotSourceForFreeze = pivotAuthoritySource;
                            resolvedAuthorityPivotReasonForFreeze = grabFallbackReason ? grabFallbackReason : "none";
                        }
                        pivotAuthorityPositionOnly = false;
                        pivotAuthorityNormalTrusted = lengthSquared(gripNormalWorld) > 0.000001f;
                        pivotAuthorityLongLeverGameUnits = gripLeverGameUnits;
                        pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        pivotAuthoritySelectionDeltaGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                        gripEvidencePointWorld = grabGripPoint;
                        gripEvidenceNormalWorld = gripNormalWorld;
                        _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                        _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                        _grabObjectGripAtGrab.source = grabPointMode;
                        _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;

                        const RE::NiPoint3 finalGripToPocketVector =
                            pocket.valid ? (grabGripPoint - pocket.palmCenterWorld) : RE::NiPoint3{};
                        const float finalGripToPocketDistance =
                            pocket.valid ?
                                std::sqrt(finalGripToPocketVector.x * finalGripToPocketVector.x +
                                          finalGripToPocketVector.y * finalGripToPocketVector.y +
                                          finalGripToPocketVector.z * finalGripToPocketVector.z) :
                                std::numeric_limits<float>::max();
                        const float finalSignedPalmDistance =
                            pocket.valid ? grab_three_phase::dot(finalGripToPocketVector, pocket.palmNormalWorld) : 0.0f;
                        const auto arrivalSeatSafety =
                            grab_three_phase::evaluateProgrammaticArrivalSeatSafety(
                                grab_three_phase::ProgrammaticArrivalSeatSafetyInput{
                                    .programmaticArrival = programmaticArrival,
                                    .usingPinchPocket = usingPinchPocket,
                                    .capturePhase = _grabAcquisitionPhase,
                                    .pocketValid = pocket.valid,
                                    .stablePocketTouchContact = hasStablePocketTouchContact,
                                    .pivotAuthorityNormalTrusted = pivotAuthorityNormalTrusted,
                                    .pivotAuthorityPositionOnly = pivotAuthorityPositionOnly,
                                    .gripToPocketDistanceGameUnits = finalGripToPocketDistance,
                                    .signedPalmDistanceGameUnits = finalSignedPalmDistance,
                                    .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                                    .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                                    .pocketRadiusGameUnits = pocket.valid ? pocket.pocketRadiusGameUnits : g_rockConfig.rockGrabPocketRadiusGameUnits,
                                    .palmNormalWorld = pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                                    .gripNormalWorld = gripNormalWorld,
                                });
                        if (programmaticArrival &&
                            !usingPinchPocket &&
                            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld &&
                            !arrivalSeatSafety.allowImmediateTouchHeld) {
                            _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                            captureReason = arrivalSeatSafety.reason;
                            grabFallbackReason = captureReason;
                            _grabObjectGripAtGrab.fallbackReason = captureReason;
                        }

                        desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                            grabBodyWorldAtGrab,
                            grabPivotAWorld,
                            grabGripPoint);
                        desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                        /*
                         * Guns and melee never seat from saved offsets: the FRIK
                         * weapon-offset attach below is the only weapon authority
                         * (saved_grab_offset::participatesInSavedGrabOffsets).
                         * Leaving the source empty here also keeps a saved finger
                         * pose from overriding the FRIK weapon hand pose.
                         */
                        bool fixedAidPose = g_rockConfig.rockImmersiveAidEnabled &&
                            immersive_aid::usesStimpakPose(immersive_aid::classify(sel.refr ? sel.refr->GetObjectReference() : nullptr));
                        /*
                         * Owner rule (2026-09-30, behavior library "Vanilla grip templates"):
                         * saved offsets apply only when the object arrives by a far pull
                         * (or a forced arrival); every close grab, pinch pocket included,
                         * stays dynamic.
                         */
                        if (fixedAidPose) {
                            savedGrabOffset = immersive_aid::stimpakPose(_isLeft);
                            hasSavedGrabOffset = true;
                        } else if (programmaticArrival && !authoredWeapon &&
                            saved_grab_offset::participatesInSavedGrabOffsets(
                                looseWeaponGrab,
                                isThrowableLooseWeapon(selectedLooseWeaponForm(sel)))) {
                            hasSavedGrabOffset = tryLoadSavedGrabOffsetHandOffset(sel.refr, _isLeft, savedGrabOffset);
                        }
                        RE::NiTransform grabProxyWorldForOffset{};
                        const bool grabProxyWorldValidForOffset =
                            (programmaticArrival || fixedAidPose) && tryComputeGrabProxyLocalPalmPocketFrameWorld(world, grabProxyWorldForOffset);
                        RE::NiTransform savedGrabOffsetRootWorld{};
                        const bool savedGrabOffsetAttachValid = hasSavedGrabOffset &&
                            tryResolveSavedGrabOffsetAttach(
                                grabProxyWorldForOffset,
                                grabProxyWorldValidForOffset,
                                savedGrabOffset,
                                savedGrabOffsetRootWorld);
                        if (fixedAidPose && !savedGrabOffsetAttachValid) {
                            // Without the authored proxy frame the aid is held like any loose item.
                            ROCK_LOG_SAMPLE_INFO(Hand, 1000, "{} hand immersive aid pose unavailable; dynamic grab", handName());
                            fixedAidPose = false;
                            hasSavedGrabOffset = false;
                        }
                        const auto looseWeaponPrimaryAttachFrame = resolveLooseWeaponPrimaryAttachFrame(
                            looseWeaponGrab,
                            grabbedFromPullCatch,
                            _isLeft,
                            sel,
                            rootNode,
                            rootBodyLocalAtGrab,
                            objectToBodyAtGrab,
                            grabBodyWorldAtGrab,
                            grabPivotAWorld,
                            savedGrabOffsetAttachValid,
                            savedGrabOffsetRootWorld,
                            nearAuthored ? &nearGrip : nullptr);
                        looseWeaponPrimaryAttachReason = looseWeaponPrimaryAttachFrame.reason;
                        if (nearAuthored && !looseWeaponPrimaryAttachFrame.valid) {
                            // The authored seat cannot be resolved; hold the weapon dynamically.
                            ROCK_LOG_SAMPLE_INFO(Hand, 1000, "{} hand authored loose seat unavailable; dynamic grab ref={:08X} reason={}",
                                handName(), sel.refr ? sel.refr->GetFormID() : 0, looseWeaponPrimaryAttachReason);
                            nearAuthored = false;
                        }
                        if (looseWeaponPrimaryAttachFrame.valid) {
                            desiredObjectWorld = looseWeaponPrimaryAttachFrame.desiredObjectWorld;
                            desiredBodyWorld = looseWeaponPrimaryAttachFrame.desiredBodyWorld;
                            grabGripPoint = looseWeaponPrimaryAttachFrame.gripPointWorld;
                            gripArea.contactSeedWorld = grabGripPoint;
                            gripEvidencePointWorld = grabGripPoint;
                            gripNormalWorld = firstValidNormal({
                                pocket.valid ? pocket.palmNormalWorld : RE::NiPoint3{},
                                gripNormalWorld,
                            });
                            gripEvidenceNormalWorld = gripNormalWorld;
                            _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                            _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                            grabPointMode = "looseWeaponPrimaryAttach";
                            grabPointAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
                            relationMode = grabPointMode;
                            grabFallbackReason = looseWeaponPrimaryAttachFrame.reason;
                            _grabObjectGripAtGrab.source = grabPointMode;
                            _grabObjectGripAtGrab.fallbackReason = grabFallbackReason;
                            if (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
                                _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::NearConverging;
                                captureReason = "looseWeaponPrimaryAttachSettle";
                            }
                            grabSurfaceHit = GrabSurfaceHit{};
                            pivotAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
                            pivotAuthorityPositionOnly = false;
                            pivotAuthorityNormalTrusted = true;
                            pivotAuthorityPositionConfidence = 1.0f;
                            pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                            pivotAuthoritySelectionDeltaGameUnits = sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                            resolvedAuthorityPivotSourceForFreeze = grab_authority_frame_math::GrabAuthorityPivotSource::LooseWeaponPrimaryAttach;
                            resolvedAuthorityPivotReasonForFreeze = looseWeaponPrimaryAttachFrame.reason;
                            looseWeaponPrimaryAttachApplied = true;
                            _grabFrame.authoredLooseWeaponSupportGrip = looseWeaponPrimaryAttachFrame.supportGrip;
                            _grabFrame.authoredWeaponPose = looseWeaponPrimaryAttachFrame.pose;
                            outCapture.hasAuthoredHandRelation = nearAuthored;
                            if (nearAuthored) {
                                outCapture.authoredHandWeaponLocal = nearGrip.pose.handWeaponLocal;
                            }
                            ROCK_LOG_INFO(Hand, "{} hand loose authored grab ref={:08X} role={} source={}",
                                handName(), sel.refr ? sel.refr->GetFormID() : 0,
                                looseWeaponPrimaryAttachFrame.supportGrip ? "support" : "firing",
                                looseWeaponPrimaryAttachFrame.reason);
                            looseWeaponPrimaryAttachSourceVisible = looseWeaponPrimaryAttachFrame.sourceVisible;
                        }
                        /*
                         * Seat-time self-alignment. Forced arrivals (grenade menu,
                         * provider API) commit without a pull flight, so the
                         * long-axis presentation servo never gets to run: rotate
                         * the SEAT pose instead - the minimal rotation taking the
                         * mesh principal axis onto the pocket's cross-palm
                         * (thumb->pinky) line, about the grip point so the pivot
                         * pair is untouched. Pull-catch arrivals already aligned
                         * their long axis in flight and keep it.
                         *
                         * Face-to-palm alignment is a separate correction: a
                         * minimal-arc swing (flight servo or seat rotation) adds
                         * zero twist, so an object that arrived tilted keeps its
                         * arrival roll and its flat face can cut diagonally into
                         * the palm - the depth stop below only translates and
                         * cannot fix a tilted face. Which rotation achieves it
                         * depends on the shape class, read from the same PCA
                         * spectrum:
                         *
                         *   ROD/PLANK (elongationRatio gate passes): the long axis
                         *   is already committed to the cross-palm line, so the
                         *   only free DOF is a TWIST about it - roll until the
                         *   thinnest-extent direction faces the palm. Round cross
                         *   sections skip; any roll is equivalent for them.
                         *
                         *   PLATE (elongation gate fails but the minor axis is
                         *   well separated - books, trays, boards): there is no
                         *   meaningful long axis to twist about, but the face
                         *   normal is exact. Swing that normal onto the palm
                         *   normal directly. Nearest-hemisphere keeps the swing
                         *   under 90 degrees and never flips the object over. The
                         *   free in-plane spin is deliberately left alone: it
                         *   cannot affect clipping, and for near-square plates the
                         *   in-plane axes are numerically interchangeable.
                         *
                         *   Otherwise (compact/isotropic): no alignment.
                         *
                         * Rotations survive the freeze (pivot alignment only
                         * rewrites translation).
                         */
                        RE::NiTransform seatBodyWorld = grabBodyWorldAtGrab;
                        RE::NiTransform seatObjectWorld = objectWorldTransform;
                        float seatAlignmentAngleDegrees = 0.0f;
                        const char* seatAlignmentReason = "inactive";
                        float seatRollAngleDegrees = 0.0f;
                        const char* seatRollReason = "inactive";
                        bool seatPoseChanged = false;
                        const bool seatSwingAlignmentWanted =
                            sel.forcedArrival && g_rockConfig.rockForceGrabSeatAlignmentEnabled;
                        const bool seatRollAlignmentWanted =
                            programmaticArrival && g_rockConfig.rockGrabSeatRollAlignmentEnabled;
                        /*
                         * The shape class also selects the penetration backstop
                         * footprint below, and that backstop runs on EVERY arrival
                         * (close grabs never rotate the object), so the PCA is
                         * resolved outside the alignment gates. One pass at
                         * capture, never per frame.
                         */
                        const bool seatShapeEvaluable =
                            !looseWeaponPrimaryAttachApplied &&
                            !usingPinchPocket &&
                            pocket.valid &&
                            !grabMeshTriangles.empty();
                        const auto seatLongAxis =
                            seatShapeEvaluable ? computeGrabMeshLongAxis(grabMeshTriangles) : GrabMeshLongAxisResult{};
                        const bool seatRodShape =
                            seatLongAxis.valid &&
                            seatLongAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio;
                        const bool seatFlatFace =
                            seatLongAxis.valid &&
                            seatLongAxis.secondElongationRatio >= g_rockConfig.rockGrabSeatRollMinSecondElongationRatio;
                        const bool seatPlateShape = seatLongAxis.valid && !seatRodShape && seatFlatFace;
                        if ((seatSwingAlignmentWanted || seatRollAlignmentWanted) && seatShapeEvaluable) {
                            if (seatSwingAlignmentWanted) {
                                seatAlignmentReason = seatLongAxis.reason;
                            }
                            if (seatRollAlignmentWanted) {
                                seatRollReason = seatLongAxis.reason;
                            }
                            RE::NiPoint3 currentAxisWorld = normalizeOrZero(seatLongAxis.axisWorld);
                            RE::NiPoint3 currentSecondAxisWorld = normalizeOrZero(seatLongAxis.secondAxisWorld);
                            if (seatRodShape) {
                                if (seatSwingAlignmentWanted) {
                                    // Same hand-neutral, +X-toward-fingertips tilt as the flight servo.
                                    RE::NiPoint3 targetAxisWorld =
                                        grab_three_phase::buildGripPresentationAxisTowardFingertips(
                                            pocket.crossPalmWorld,
                                            pocket.fingerForwardWorld,
                                            g_rockConfig.rockPullPresentationGripAxisTiltDegrees);
                                    if (lengthSquared(currentAxisWorld) > 0.000001f && lengthSquared(targetAxisWorld) > 0.000001f) {
                                        if (dotProduct(currentAxisWorld, targetAxisWorld) < 0.0f) {
                                            targetAxisWorld = RE::NiPoint3{ -targetAxisWorld.x, -targetAxisWorld.y, -targetAxisWorld.z };
                                        }
                                        const RE::NiPoint3 rotationAxisRaw = crossProduct(currentAxisWorld, targetAxisWorld);
                                        const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxisRaw)));
                                        const float cosAngle = std::clamp(dotProduct(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                                        const float angleRadians = std::atan2(sinAngle, cosAngle);
                                        if (sinAngle > 0.000001f && angleRadians > 0.01f) {
                                            const float invSin = 1.0f / sinAngle;
                                            const RE::NiPoint3 rotationAxis{
                                                rotationAxisRaw.x * invSin,
                                                rotationAxisRaw.y * invSin,
                                                rotationAxisRaw.z * invSin,
                                            };
                                            seatBodyWorld = rotateTransformWorldAboutPoint(
                                                grabBodyWorldAtGrab, rotationAxis, angleRadians, grabGripPoint);
                                            seatObjectWorld = rotateTransformWorldAboutPoint(
                                                objectWorldTransform, rotationAxis, angleRadians, grabGripPoint);
                                            // Carry the mesh axes into the swung pose so the roll below runs in seat space.
                                            currentAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(
                                                currentAxisWorld, rotationAxis, angleRadians);
                                            currentSecondAxisWorld = grab_finger_pose_math::rotateAroundUnitAxis(
                                                currentSecondAxisWorld, rotationAxis, angleRadians);
                                            seatAlignmentAngleDegrees = angleRadians * 57.29577951308232f;
                                            seatAlignmentReason = "longAxisSeatAligned";
                                            seatPoseChanged = true;
                                        } else {
                                            seatAlignmentReason = "alreadyAligned";
                                        }
                                    } else {
                                        seatAlignmentReason = "degenerateAxes";
                                    }
                                }
                                if (seatRollAlignmentWanted) {
                                    if (seatFlatFace &&
                                        lengthSquared(currentAxisWorld) > 0.000001f &&
                                        lengthSquared(currentSecondAxisWorld) > 0.000001f) {
                                        // Thinnest-extent direction = the flat face's normal.
                                        const RE::NiPoint3 faceNormalWorld = crossProduct(currentAxisWorld, currentSecondAxisWorld);
                                        const RE::NiPoint3 rollTargetRaw =
                                            pocket.palmNormalWorld - currentAxisWorld * dotProduct(pocket.palmNormalWorld, currentAxisWorld);
                                        const RE::NiPoint3 rollCurrentRaw =
                                            faceNormalWorld - currentAxisWorld * dotProduct(faceNormalWorld, currentAxisWorld);
                                        RE::NiPoint3 rollTargetWorld = normalizeOrZero(rollTargetRaw);
                                        const RE::NiPoint3 rollCurrentWorld = normalizeOrZero(rollCurrentRaw);
                                        if (lengthSquared(rollTargetWorld) > 0.000001f && lengthSquared(rollCurrentWorld) > 0.000001f) {
                                            if (dotProduct(rollCurrentWorld, rollTargetWorld) < 0.0f) {
                                                // A face has two sides; twist toward the nearer one.
                                                rollTargetWorld = RE::NiPoint3{ -rollTargetWorld.x, -rollTargetWorld.y, -rollTargetWorld.z };
                                            }
                                            const float rollSin = dotProduct(crossProduct(rollCurrentWorld, rollTargetWorld), currentAxisWorld);
                                            const float rollCos = std::clamp(dotProduct(rollCurrentWorld, rollTargetWorld), -1.0f, 1.0f);
                                            const float rollAngleRadians = std::atan2(rollSin, rollCos);
                                            if (std::fabs(rollAngleRadians) > 0.01f) {
                                                seatBodyWorld = rotateTransformWorldAboutPoint(
                                                    seatBodyWorld, currentAxisWorld, rollAngleRadians, grabGripPoint);
                                                seatObjectWorld = rotateTransformWorldAboutPoint(
                                                    seatObjectWorld, currentAxisWorld, rollAngleRadians, grabGripPoint);
                                                seatRollAngleDegrees = rollAngleRadians * 57.29577951308232f;
                                                seatRollReason = "faceRollSeatAligned";
                                                seatPoseChanged = true;
                                            } else {
                                                seatRollReason = "alreadyRollAligned";
                                            }
                                        } else {
                                            seatRollReason = "degenerateRollAxes";
                                        }
                                    } else {
                                        seatRollReason = "belowSecondElongationGate";
                                    }
                                }
                            } else if (seatLongAxis.valid) {
                                if (seatSwingAlignmentWanted) {
                                    // Cross-palm swing is a rod correction only.
                                    seatAlignmentReason = "belowElongationGate";
                                }
                                if (seatRollAlignmentWanted) {
                                    /*
                                     * Plates deliberately get NO face alignment.
                                     * The intuition that a flat object should lie
                                     * face-down on the palm is simply wrong for
                                     * this hand: across 31 user-verified plate
                                     * holds (2026-07-25 ground-truth captures) the
                                     * mesh face normal sits 63 degrees off the palm
                                     * normal, IQR 57..72 - plates are held by an
                                     * EDGE or CORNER, and that is the intended
                                     * grip, not an artifact. A face swing fought
                                     * that preference on every plate, so the branch
                                     * was removed rather than retuned. The plate
                                     * CLASS is still resolved above because the
                                     * penetration backstop uses it.
                                     */
                                    seatRollReason = seatPlateShape ? "plateEdgeHoldNoFaceAlign" : "belowSecondElongationGate";
                                }
                            }
                        }

                        /*
                         * Close-grab seat mode 1 (iGrabCloseSeatMode): local grasp
                         * solve. The current seat keeps the object's rotation and
                         * slides the mesh point nearest the palm onto it, so a mug
                         * handle held in front of the palm is seated like any other
                         * point and the fingers stop on the mug body behind it. The
                         * whole-mesh shape class cannot help: a mug is compact as a
                         * whole and its handle is a bar only locally.
                         *
                         * This mode classifies the mesh inside a hand-sized window in
                         * front of the palm (GrabLocalGraspSolve.h). When it finds a
                         * bar, rim or round body there, that part's own point becomes
                         * pivot B and the object turns about it onto the hand's grip
                         * line. Pivot A moves from the palm centre toward the finger
                         * bases by what the part's width allows, because closed
                         * fingers trap a handle under their first segments and never
                         * reach one lying on the palm centre. The depth stop below
                         * then rests the turned surface on the palm exactly as it
                         * does for any other seat.
                         *
                         * Plain close grabs of loose objects only. Pulled, forced,
                         * pinched, authored, weapon and shared grabs keep their own
                         * seat authority, and an unrecognised window keeps the current
                         * seat: the mode changes how an object is placed, never whether
                         * it is grabbed. One bounded pass at capture.
                         */
                        grab_local_grasp::Result localGrasp{};
                        const char* localGraspReason = "modeOff";
                        bool localGraspApplied = false;
                        // In-plane distance of pivot A from the palm centre. Zero
                        // for every seat except a local grasp seat.
                        RE::NiPoint3 seatPalmAnchorOffsetWorld{};
                        float localGraspSeatForwardGameUnits = 0.0f;
                        if (g_rockConfig.rockGrabCloseSeatMode == 1) {
                            const auto localGraspStart = std::chrono::steady_clock::now();
                            const std::size_t fingerTrianglesBefore = _grabFrame.fingerPoseLocalMeshTriangles.size();
                            grab_local_grasp::NearestPoint localGraspSeed{};
                            const bool plainCloseGrab =
                                !programmaticArrival &&
                                !nearAuthored &&
                                !looseWeaponGrab &&
                                !input.joiningPeerHeldObject &&
                                sel.targetKind == grab_target::Kind::LooseObject;
                            if (!plainCloseGrab) {
                                localGraspReason = "notPlainCloseGrab";
                            } else if (!seatShapeEvaluable || grabLocalMeshTriangles.empty()) {
                                localGraspReason = "seatNotEvaluable";
                            } else {
                                // Pocket frame: x along the fingers, y across the palm,
                                // z out of the palm, origin at the palm centre.
                                const auto vectorToPocket = [&](const RE::NiPoint3& vectorWorld) {
                                    return grab_local_grasp::Vec3{
                                        dotProduct(vectorWorld, pocket.fingerForwardWorld),
                                        dotProduct(vectorWorld, pocket.crossPalmWorld),
                                        dotProduct(vectorWorld, pocket.palmNormalWorld),
                                    };
                                };
                                const auto vectorToWorld = [&](const grab_local_grasp::Vec3& vectorPocket) {
                                    return pocket.fingerForwardWorld * vectorPocket.x +
                                           pocket.crossPalmWorld * vectorPocket.y +
                                           pocket.palmNormalWorld * vectorPocket.z;
                                };
                                /*
                                 * Node-local to pocket is affine, so it is sampled
                                 * once instead of composing two transforms per
                                 * vertex. The long basis keeps the float error of
                                 * far-from-origin world coordinates out of the map.
                                 */
                                constexpr float kBasisLength = 64.0f;
                                const RE::NiPoint3 nodeOriginWorld =
                                    transform_math::localPointToWorld(objectWorldTransform, RE::NiPoint3{ 0.0f, 0.0f, 0.0f });
                                const auto pocketOrigin = vectorToPocket(nodeOriginWorld - pocket.palmCenterWorld);
                                const auto pocketColumn = [&](const RE::NiPoint3& axisLocal) {
                                    const auto image = vectorToPocket(
                                        transform_math::localPointToWorld(objectWorldTransform, axisLocal * kBasisLength) - nodeOriginWorld);
                                    return grab_local_grasp::Vec3{ image.x / kBasisLength, image.y / kBasisLength, image.z / kBasisLength };
                                };
                                const auto pocketFromX = pocketColumn(RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                                const auto pocketFromY = pocketColumn(RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
                                const auto pocketFromZ = pocketColumn(RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
                                const auto localToPocket = [&](const RE::NiPoint3& pointLocal) {
                                    return grab_local_grasp::Vec3{
                                        pocketOrigin.x + pocketFromX.x * pointLocal.x + pocketFromY.x * pointLocal.y + pocketFromZ.x * pointLocal.z,
                                        pocketOrigin.y + pocketFromX.y * pointLocal.x + pocketFromY.y * pointLocal.y + pocketFromZ.y * pointLocal.z,
                                        pocketOrigin.z + pocketFromX.z * pointLocal.x + pocketFromY.z * pointLocal.y + pocketFromZ.z * pointLocal.z,
                                    };
                                };

                                // Same hand-neutral grip line the pull presentation seats rods on.
                                const auto gripLinePocket = vectorToPocket(grab_three_phase::buildGripPresentationAxisTowardFingertips(
                                    pocket.crossPalmWorld,
                                    pocket.fingerForwardWorld,
                                    g_rockConfig.rockPullPresentationGripAxisTiltDegrees));
                                localGraspSeed = grab_local_grasp::nearestPointToOrigin(grabLocalMeshTriangles, localToPocket);
                                const float behindPalmTolerance = (std::max)(0.0f, g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
                                if (!localGraspSeed.found) {
                                    localGraspReason = "noFiniteMesh";
                                } else if (localGraspSeed.point.z < -behindPalmTolerance ||
                                           std::hypot(localGraspSeed.point.x, localGraspSeed.point.y) > pocket.pocketRadiusGameUnits) {
                                    // The window is read along the palm normal, which
                                    // only describes mesh the palm actually faces.
                                    localGraspReason = "nearestMeshNotInFrontOfPalm";
                                } else {
                                    localGrasp = grab_local_grasp::solve(
                                        grabLocalMeshTriangles,
                                        localGraspSeed.point,
                                        gripLinePocket,
                                        grab_local_grasp::Config{},
                                        localToPocket);
                                    localGraspReason = localGrasp.reason;
                                }

                                const RE::NiPoint3 localGraspPivotWorld = pocket.palmCenterWorld + vectorToWorld(localGrasp.pivot);
                                if (localGrasp.valid && localGrasp.kind != grab_local_grasp::Kind::None &&
                                    grab_three_phase::isFinite(localGraspPivotWorld)) {
                                    grabGripPoint = localGraspPivotWorld;
                                    if (localGrasp.hasAlignment) {
                                        const RE::NiPoint3 fromWorld = normalizeOrZero(vectorToWorld(localGrasp.alignFrom));
                                        const RE::NiPoint3 toWorld = normalizeOrZero(vectorToWorld(localGrasp.alignTo));
                                        const RE::NiPoint3 rotationAxisRaw = crossProduct(fromWorld, toWorld);
                                        const float sinAngle = std::sqrt((std::max)(0.0f, lengthSquared(rotationAxisRaw)));
                                        const float cosAngle = std::clamp(dotProduct(fromWorld, toWorld), -1.0f, 1.0f);
                                        const float angleRadians = std::atan2(sinAngle, cosAngle);
                                        if (sinAngle > 0.000001f && angleRadians > 0.01f) {
                                            const float invSin = 1.0f / sinAngle;
                                            const RE::NiPoint3 rotationAxis{
                                                rotationAxisRaw.x * invSin,
                                                rotationAxisRaw.y * invSin,
                                                rotationAxisRaw.z * invSin,
                                            };
                                            // About the new pivot, so the pivot pair is untouched.
                                            seatBodyWorld = rotateTransformWorldAboutPoint(seatBodyWorld, rotationAxis, angleRadians, grabGripPoint);
                                            seatObjectWorld = rotateTransformWorldAboutPoint(seatObjectWorld, rotationAxis, angleRadians, grabGripPoint);
                                            seatAlignmentAngleDegrees = angleRadians * 57.29577951308232f;
                                        }
                                    }
                                    seatAlignmentReason = localGrasp.reason;

                                    /*
                                     * Finger base joints from the created hand
                                     * colliders. A base joint does not move when its
                                     * finger curls, so the pose at grip press does
                                     * not matter. The joint is the first segment's
                                     * end away from the second segment; segment
                                     * centres are used so no collider axis
                                     * convention is involved.
                                     */
                                    {
                                        using hand_collider_semantics::HandColliderRole;
                                        constexpr std::array<std::pair<HandColliderRole, HandColliderRole>, 4> kFingerBaseSegments{ {
                                            { HandColliderRole::IndexBase, HandColliderRole::IndexMiddle },
                                            { HandColliderRole::MiddleBase, HandColliderRole::MiddleMiddle },
                                            { HandColliderRole::RingBase, HandColliderRole::RingMiddle },
                                            { HandColliderRole::PinkyBase, HandColliderRole::PinkyMiddle },
                                        } };
                                        const auto findSegment = [&](HandColliderRole role) -> const HandBoneColliderSet::PublishedSegmentFrame* {
                                            for (const auto& segment : _boneColliders.segmentColliderFrames()) {
                                                if (segment.valid && segment.role == role && grab_three_phase::isFinite(segment.target) &&
                                                    std::isfinite(segment.length) && segment.length > 0.0f &&
                                                    std::isfinite(segment.radius) && segment.radius > 0.0f) {
                                                    return &segment;
                                                }
                                            }
                                            return nullptr;
                                        };
                                        RE::NiPoint3 baseJointSumWorld{};
                                        float fingerRadiusSum = 0.0f;
                                        int fingerCount = 0;
                                        for (const auto& [baseRole, middleRole] : kFingerBaseSegments) {
                                            const auto* base = findSegment(baseRole);
                                            const auto* middle = findSegment(middleRole);
                                            if (!base || !middle) {
                                                continue;
                                            }
                                            const RE::NiPoint3 towardPalm = normalizeOrZero(base->target.translate - middle->target.translate);
                                            if (lengthSquared(towardPalm) <= 0.000001f) {
                                                continue;
                                            }
                                            baseJointSumWorld = baseJointSumWorld + base->target.translate + towardPalm * (base->length * 0.5f);
                                            fingerRadiusSum += base->radius;
                                            ++fingerCount;
                                        }
                                        if (fingerCount > 0) {
                                            const float inverseCount = 1.0f / static_cast<float>(fingerCount);
                                            localGraspSeatForwardGameUnits = grab_local_grasp::fingerBaseSeat(
                                                localGrasp,
                                                vectorToPocket(baseJointSumWorld * inverseCount - pocket.palmCenterWorld),
                                                fingerRadiusSum * inverseCount,
                                                gripLinePocket);
                                            seatPalmAnchorOffsetWorld = pocket.fingerForwardWorld * localGraspSeatForwardGameUnits;
                                            grabPivotAWorld = pocket.palmCenterWorld + seatPalmAnchorOffsetWorld;
                                        }
                                    }
                                    gripArea.contactSeedWorld = grabGripPoint;
                                    gripEvidencePointWorld = grabGripPoint;
                                    _grabObjectGripAtGrab.contactSeedWorld = grabGripPoint;
                                    _grabObjectGripAtGrab.gripCenterWorld = grabGripPoint;
                                    pivotAuthorityPocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                                    pivotAuthoritySelectionDeltaGameUnits =
                                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                                    desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                        seatBodyWorld,
                                        grabPivotAWorld,
                                        grabGripPoint);
                                    desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                                    seatPoseChanged = true;
                                    localGraspApplied = true;

                                    if (localGrasp.threadFingers) {
                                        /*
                                         * Finger closure stops at first contact. With
                                         * the body behind a handle loop still in its
                                         * mesh, the fingers stop on that body on their
                                         * way in and never reach the handle. Drop the
                                         * triangles wholly behind the gap from closure
                                         * only; seating and collision keep the full
                                         * mesh. An emptied list would make closure fall
                                         * back to that full mesh, so keep it populated.
                                         */
                                        auto& fingerTriangles = _grabFrame.fingerPoseLocalMeshTriangles;
                                        const auto behindGap = [&](const GrabLocalTriangle& triangle) {
                                            return grab_local_grasp::behindThreadGap(localGrasp,
                                                localToPocket(triangle.v0), localToPocket(triangle.v1), localToPocket(triangle.v2));
                                        };
                                        const auto droppedCount = static_cast<std::size_t>(
                                            std::count_if(fingerTriangles.begin(), fingerTriangles.end(), behindGap));
                                        constexpr std::size_t kMinFingerClosureTriangles = 8;
                                        if (droppedCount > 0 && fingerTriangles.size() - droppedCount >= kMinFingerClosureTriangles) {
                                            std::erase_if(fingerTriangles, behindGap);
                                        }
                                    }
                                }
                            }
                            const float localGraspMicroseconds = static_cast<float>(
                                std::chrono::duration_cast<std::chrono::microseconds>(
                                    std::chrono::steady_clock::now() - localGraspStart).count());
                            // Applied-mode status: one line per grab while the mode is on.
                            ROCK_LOG_INFO(Hand,
                                "{} CLOSE GRAB SEAT: mode=1 localGrasp applied={} part={} reason={} turn={:.1f}deg towardFingers={:.2f}gu seed=({:.2f},{:.2f},{:.2f}) "
                                "pivot=({:.2f},{:.2f},{:.2f}) width={:.2f} thickness={:.2f} length={:.2f} gapBehind={:.2f} slide={:.2f} "
                                "elongation={:.2f} thread={} fingerTris={}->{} samples={} near={} meshTris={} us={:.0f} ref={:08X}",
                                handName(),
                                localGraspApplied ? "yes" : "no",
                                grab_local_grasp::kindName(localGrasp.kind),
                                localGraspReason,
                                localGraspApplied ? seatAlignmentAngleDegrees : 0.0f,
                                localGraspSeatForwardGameUnits,
                                localGraspSeed.point.x,
                                localGraspSeed.point.y,
                                localGraspSeed.point.z,
                                localGrasp.pivot.x,
                                localGrasp.pivot.y,
                                localGrasp.pivot.z,
                                localGrasp.widthGameUnits,
                                localGrasp.thicknessGameUnits,
                                localGrasp.lengthGameUnits,
                                localGrasp.gapBehindGameUnits,
                                localGrasp.stationShiftGameUnits,
                                localGrasp.elongation,
                                localGrasp.threadFingers ? "yes" : "no",
                                fingerTrianglesBefore,
                                _grabFrame.fingerPoseLocalMeshTriangles.size(),
                                localGrasp.sampleCount,
                                localGrasp.nearSampleCount,
                                grabLocalMeshTriangles.size(),
                                localGraspMicroseconds,
                                sel.refr ? sel.refr->GetFormID() : 0);
                        }

                        /*
                         * Seat depth stop. The freeze aligns the grip point onto
                         * pivot A, which can pull mesh behind that point through
                         * the palm. Measure the final seat pose from the palm
                         * plane itself: any mesh past the plane inside the palm
                         * footprint is penetration, and one push along the palm
                         * normal removes it exactly (a translation lowers every
                         * sample depth by the same amount). Plates use their
                         * wider edge footprint. Weapon attach frames and pinch
                         * pockets keep their own seat.
                         */
                        GrabSeatDepthStopResult seatDepthStop{};
                        float seatDepthOffsetGameUnits = 0.0f;
                        if (!looseWeaponPrimaryAttachApplied && !usingPinchPocket && pocket.valid) {
                            if (seatPoseChanged) {
                                desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                    seatBodyWorld,
                                    grabPivotAWorld,
                                    grabGripPoint);
                                desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                            }
                            const float seatFootprintRadiusGameUnits =
                                seatPlateShape ? g_rockConfig.rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits
                                               : g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits;
                            seatDepthStop = computeGrabPalmSeatDepthStop(
                                grabLocalMeshTriangles,
                                desiredObjectWorld,
                                pocket.palmCenterWorld,
                                pocket,
                                _boneColliders.segmentColliderFrames(),
                                seatFootprintRadiusGameUnits,
                                g_rockConfig.rockGrabSeatDepthMaxGameUnits);
                            if (seatDepthStop.valid && seatDepthStop.depthGameUnits > 0.01f) {
                                seatDepthOffsetGameUnits =
                                    seatDepthStop.depthGameUnits + (std::max)(0.0f, g_rockConfig.rockGrabSeatDepthSkinGameUnits);
                                grabPivotAWorld = grabPivotAWorld + pocket.palmNormalWorld * seatDepthOffsetGameUnits;
                                desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                    seatBodyWorld,
                                    grabPivotAWorld,
                                    grabGripPoint);
                                desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                            }
                        }

                        // The object and the commanded finger pose converge to the
                        // same solved fingertip midpoint. Keep the source mesh hit
                        // as pivot B and compensate its measured mid-thickness offset.
                        float pinchCenterOffsetGameUnits = 0.0f;
                        if (usingPinchPocket && !looseWeaponPrimaryAttachApplied) {
                            pinchCenterOffsetGameUnits = pinchPocketCandidate.closure.sample.centerOffsetGameUnits;
                            grabPivotAWorld = pinchPocketCandidate.pinchPocketWorld -
                                pinchPocketCandidate.pinchAxisWorld * pinchCenterOffsetGameUnits;
                            desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                                grabBodyWorldAtGrab, grabPivotAWorld, grabGripPoint);
                            desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                        }

                        selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
                        selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);
                        const bool effectivePinchPocket = usingPinchPocket && !looseWeaponPrimaryAttachApplied;
                        /*
                         * Seat diagnostics: the classification and every
                         * correction outcome, kept past the log line so a saved
                         * ground-truth capture can record the seat ROCK produced
                         * for this grab next to the pose the user corrected it to.
                         */
                        _grabFrame.seat.diagnostics = GrabSeatDiagnostics{
                            .acquisitionMode = sel.forcedArrival ? "forceGrab" : (grabbedFromPullCatch ? "pullCatch" : "closeGrab"),
                            .shapeClass = seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                            .elongationRatio = seatLongAxis.elongationRatio,
                            .secondElongationRatio = seatLongAxis.secondElongationRatio,
                            .alignmentAngleDegrees = seatAlignmentAngleDegrees,
                            .alignmentReason = seatAlignmentReason,
                            .rollAngleDegrees = seatRollAngleDegrees,
                            .rollReason = seatRollReason,
                            .depthGameUnits = seatDepthStop.depthGameUnits,
                            .depthOffsetGameUnits = seatDepthOffsetGameUnits,
                            .depthReason = seatDepthStop.reason,
                        };
                        _grabFrame.gripEvidence.gripEvidenceLocal = transform_math::worldPointToLocal(objectWorldTransform, gripEvidencePointWorld);
                        _grabFrame.gripEvidence.gripNormalLocal = transform_math::worldVectorToLocal(objectWorldTransform, gripNormalWorld);
                        storeGripSourceEvidence(_grabFrame,
                            grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
                            objectWorldTransform,
                            gripEvidencePointWorld,
                            gripEvidenceNormalWorld,
                            lengthSquared(gripEvidenceNormalWorld) > 0.000001f);
                        if (looseWeaponPrimaryAttachApplied) {
                            _grabFrame.gripEvidence.gripSourceNode = nullptr;
                            _grabFrame.gripEvidence.gripPointSourceNodeLocal = {};
                            _grabFrame.gripEvidence.gripNormalSourceNodeLocal = {};
                            _grabFrame.gripEvidence.hasGripSourceNodePoint = false;
                            _grabFrame.gripEvidence.hasGripSourceNodeNormal = false;
                            _grabFrame.gripEvidence.gripEvidenceTriangleIndex = 0xFFFF'FFFF;
                            _grabFrame.gripEvidence.gripEvidenceShapeKey = 0xFFFF'FFFF;
                            _grabFrame.gripEvidence.gripEvidenceShapeCollisionFilterInfo = 0;
                            _grabFrame.gripEvidence.gripEvidenceHitFraction = 1.0f;
                            _grabFrame.gripEvidence.hasGripEvidenceShapeKey = false;
                        }
                        _grabFrame.pivotAuthority.pocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                        _grabFrame.pivotAuthority.selectionDistanceGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, gripArea.contactSeedWorld) : std::numeric_limits<float>::max();
                        _grabFrame.authority.grabPivotWorldAtGrab = grabPivotAWorld;
                        _grabFrame.gripEvidence.gripPointWorldAtGrab = grabGripPoint;
                        _grabFrame.seat.activeGrabPointMode = grabPointMode;
                        _grabFrame.seat.mode = usingPinchPocket ? GrabSeatMode::PinchPocket : GrabSeatMode::SupportGroup;
                        _grabFrame.seat.programmaticArrival = programmaticArrival;
                        _grabFrame.seat.localGraspSeat = localGraspApplied;
                        if (looseWeaponPrimaryAttachApplied) {
                            _grabFrame.seat.mode = GrabSeatMode::SupportGroup;
                        }
                        _grabFrame.seat.hasPinchPocket = effectivePinchPocket;
                        _grabFrame.seat.pinchFingerPose = pinchPocketCandidate.closure.sample.pose;
                        _grabFrame.seat.pinchPocketWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : RE::NiPoint3{};
                        _grabFrame.seat.pinchAxisWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
                        _grabFrame.seat.palmSeatPointWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : pocket.palmCenterWorld;
                        _grabFrame.seat.hasPalmSeatPoint = true;
                        _grabFrame.seat.palmSeatPointMode = effectivePinchPocket ? "pinchPocket" : "threePhasePocket";
                        _grabFrame.pivotAuthority.source = pivotAuthoritySource;
                        _grabFrame.pivotAuthority.positionOnly = pivotAuthorityPositionOnly;
                        _grabFrame.pivotAuthority.normalTrusted = pivotAuthorityNormalTrusted;
                        _grabFrame.pivotAuthority.positionConfidence = pivotAuthorityPositionConfidence;
                        _grabFrame.syntheticLooseWeaponPrimaryAttach = looseWeaponPrimaryAttachApplied;
                        _grabFrame.authoredWeaponArrangement = authoredWeapon ? nearGrip.arrangement :
                            loose_weapon_authored_grab_policy::Arrangement::Pending;
                        if (looseWeaponPrimaryAttachApplied) {
                            _grabFrame.seat.hasSettledVisualHandRelation = false;
                        }
                        _grabFrame.seat.requiresSettledVisualHandRelation = looseWeaponPrimaryAttachApplied ?
                            true :
                            (!effectivePinchPocket && arrivalSeatSafety.requireSettledVisualRelation);
                        _grabFrame.fingerPoseAimValid = true;
                        _grabFrame.fingerPoseAimReason = effectivePinchPocket ? "pinchPocketThumbIndexTargets" : "rockPointToPalmEvidence";
                        /*
                         * An arrival that must settle first solves its final
                         * fingers at TouchHeld, against the arrived pose. Other
                         * captures solve against the frozen target before the
                         * first approach frame.
                         */
                        const bool fullHeldAuthorityAtCapture =
                            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;

                        const auto captureFingerTargets = buildRuntimeFingerPoseTargets(gripEvidencePointWorld, gripEvidenceNormalWorld);
                        storeFingerPoseTargetsInGrabFrame(_grabFrame, captureFingerTargets, objectWorldTransform);

                        ROCK_LOG_DEBUG(Hand,
                            "{} THREE-PHASE GRAB CAPTURE: relation={} seat={} rotation={} phase={} reason={} touchContact={} stableTouch={} pocket=({:.1f},{:.1f},{:.1f}) "
                            "palm=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) seed=({:.1f},{:.1f},{:.1f}) "
                            "grip=({:.1f},{:.1f},{:.1f}) gripLocal=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) dist={:.1f} signedPalm={:.1f} "
                            "fullHeldAuthority={} pivotAuthoritySource={} positionOnlyPatch={} normalTrusted={} settledVisualRequired={} arrivalSeatSafety={} arrivalSeatDot={:.3f} arrivalSeatSigned={:.1f} arrivalSeatDist={:.1f} seatDepth={:.2f} seatDepthOffset={:.2f} seatDepthSamples={} seatDepthReason={} seatAlignDeg={:.1f} seatAlignReason={} seatRollDeg={:.1f} seatRollReason={} seatShape={} seatRatio12={:.2f} seatRatio23={:.2f} pinchCenter={:.2f} inset={:.2f} insetSource={} looseWeaponPrimaryAttach={} attachReason={} attachVisible={}",
                            handName(),
                            relationMode,
                            grabSeatModeName(_grabFrame.seat.mode),
                            looseWeaponPrimaryAttachApplied ? "primaryWeaponAttach" : (localGraspApplied ? "localGrasp" : "preserve"),
                            grab_three_phase::phaseName(_grabAcquisitionPhase),
                            captureReason,
                            semanticContacts.count > 0 ? "yes" : "no",
                            hasStablePocketTouchContact ? "yes" : "no",
                            pocket.pocketCenterWorld.x,
                            pocket.pocketCenterWorld.y,
                            pocket.pocketCenterWorld.z,
                            pocket.palmCenterWorld.x,
                            pocket.palmCenterWorld.y,
                            pocket.palmCenterWorld.z,
                            gripNormalWorld.x,
                            gripNormalWorld.y,
                            gripNormalWorld.z,
                            gripArea.contactSeedWorld.x,
                            gripArea.contactSeedWorld.y,
                            gripArea.contactSeedWorld.z,
                            grabGripPoint.x,
                            grabGripPoint.y,
                            grabGripPoint.z,
                            selectedGripPointLocal.x,
                            selectedGripPointLocal.y,
                            selectedGripPointLocal.z,
                            selectedPivotBBodyLocalGame.x,
                            selectedPivotBBodyLocalGame.y,
                            selectedPivotBBodyLocalGame.z,
                            usingPinchPocket ? pinchPocketCandidate.pocketToSurfaceDistanceGameUnits : finalGripToPocketDistance,
                            usingPinchPocket ? 0.0f : finalSignedPalmDistance,
                            fullHeldAuthorityAtCapture ? "yes" : "no",
                            grabPivotAuthoritySourceName(_grabFrame.pivotAuthority.source),
                            _grabFrame.pivotAuthority.positionOnly ? "yes" : "no",
                            _grabFrame.pivotAuthority.normalTrusted ? "yes" : "no",
                            _grabFrame.seat.requiresSettledVisualHandRelation ? "yes" : "no",
                            arrivalSeatSafety.reason,
                            arrivalSeatSafety.normalDotPalm,
                            finalSignedPalmDistance,
                            finalGripToPocketDistance,
                            seatDepthStop.depthGameUnits,
                            seatDepthOffsetGameUnits,
                            seatDepthStop.footprintSampleCount,
                            seatDepthStop.reason,
                            seatAlignmentAngleDegrees,
                            seatAlignmentReason,
                            seatRollAngleDegrees,
                            seatRollReason,
                            seatPlateShape ? "plate" : (seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none")),
                            seatLongAxis.elongationRatio,
                            seatLongAxis.secondElongationRatio,
                            pinchCenterOffsetGameUnits,
                            gripArea.seedInsetGameUnits,
                            gripArea.fallbackReason,
                            looseWeaponPrimaryAttachApplied ? "yes" : "no",
                            looseWeaponPrimaryAttachReason,
                            looseWeaponPrimaryAttachSourceVisible ? "yes" : "no");
                    } else {
                        ROCK_LOG_WARN(Hand,
                            "{} THREE-PHASE GRAB ABORT: pocketValid={} gripValid={} accepted={} reason={} dist={:.2f}gu stableTouch={}",
                            handName(),
                            pocket.valid ? "yes" : "no",
                            gripArea.valid ? "yes" : "no",
                            phaseDecision.accepted ? "yes" : "no",
                            phaseDecision.reason,
                            gripToPocketDistance,
                            hasStablePocketTouchContact ? "yes" : "no");
                        return abortCapture();
                    }
                }

        return true;
    }

    struct Hand::GrabFrozenCommitInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabBodyFrameCapture* bodyFrame = nullptr;
        GrabSeatCaptureResult* seatCapture = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::hknpBodyId objectBodyId{};
        std::uint64_t traceId = 0;
        GrabRollbackAction rollback{};
    };

    bool Hand::commitFrozenGrabAuthority(const GrabFrozenCommitInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabFrozenCommit);
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyFrame = *input.bodyFrame;
        auto& seatCapture = *input.seatCapture;
        const auto& surface = *input.surface;
        const auto& proxyFrameWorldAtGrab = proxy.proxyFrameWorldAtGrab;
        const auto& proxyAuthorityFrameWorldAtGrab = proxy.proxyAuthorityFrameWorldAtGrab;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabBodyWorldAtGrab = bodyFrame.grabBodyWorld;
        const auto& constraintBodyWorldAtGrab = bodyFrame.constraintBodyWorld;
        const auto& rootBodyLocalAtGrab = bodyFrame.rootBodyLocal;
        const auto& ownerBodyLocalAtGrab = bodyFrame.ownerBodyLocal;
        const auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        const auto& grabGripPoint = surface.gripPoint;
        const auto* grabPointMode = surface.pointMode;
        auto& desiredObjectWorld = seatCapture.desiredObjectWorld;
        auto& desiredBodyWorld = seatCapture.desiredBodyWorld;
        const auto resolvedAuthorityPivotSourceForFreeze = seatCapture.resolvedAuthoritySource;
        const auto visualHandObjectLocal = seatCapture.hasAuthoredHandRelation ?
            transform_math::composeTransforms(bodyFrame.objectToBody,
                transform_math::composeTransforms(transform_math::invertTransform(rootBodyLocalAtGrab),
                    seatCapture.authoredHandWeaponLocal)) : transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto objectBodyId = input.objectBodyId;
        const auto grabTraceId = input.traceId;
        grab_authority_frame_math::FrozenGrabAuthorityFrame<RE::NiTransform> frozenAuthorityFrame{};
                const RE::NiPoint3 frozenVisualNormalWorld = gripEvidenceNormalWorld(_grabFrame, objectWorldTransform);
                frozenAuthorityFrame = grab_authority_frame_math::freezeGrabAuthorityFrame<RE::NiTransform>(
                    grab_authority_frame_math::GrabAuthorityFrameFreezeInput<RE::NiTransform>{
                        .rawHandWorld = handWorldTransform,
                        .proxyWorld = proxyFrameWorldAtGrab,
                        .proxyAuthorityFrameWorld = proxyAuthorityFrameWorldAtGrab,
                        .objectWorld = objectWorldTransform,
                        .bodyWorld = grabBodyWorldAtGrab,
                        .constraintBodyWorld = constraintBodyWorldAtGrab,
                        .rootBodyLocal = rootBodyLocalAtGrab,
                        .ownerBodyLocal = ownerBodyLocalAtGrab,
                        .desiredObjectWorld = desiredObjectWorld,
                        .desiredBodyWorld = desiredBodyWorld,
                        .visualHandObjectLocal = visualHandObjectLocal,
                        .pivotAWorld = grabPivotAWorld,
                        .gripPointWorld = grabGripPoint,
                        .visualNormalWorld = frozenVisualNormalWorld,
                        .source = resolvedAuthorityPivotSourceForFreeze,
                        .hasDesiredObjectWorld = true,
                        .hasDesiredBodyWorld = true,
                        .hasVisualHandObjectLocal = seatCapture.hasAuthoredHandRelation,
                        .visualNormalValid = lengthSquared(frozenVisualNormalWorld) > 0.000001f,
                    });
                if (!frozenAuthorityFrame.valid) {
                    ROCK_LOG_ERROR(Hand,
                        "{} GRAB FAILED: unable to freeze coherent authority frame point=({:.2f},{:.2f},{:.2f}) pivotA=({:.2f},{:.2f},{:.2f}) mode={} bodyId={}",
                        handName(),
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z,
                        grabPivotAWorld.x,
                        grabPivotAWorld.y,
                        grabPivotAWorld.z,
                        grabPointMode,
                        objectBodyId.value);
                    _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
                    _grabObjectGripAtGrab = {};
                    _heldObjectIsLooseWeapon = false;
                    _grabFrame.clear();
                    _heldBodyIds.clear();
                    _heldDriveDecision = {};
                    _heldBodyIdsCount.store(0, std::memory_order_release);
                    _grabFingerPosePublished = false;
                    (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
                    clearGrabExternalHandWorldTransform(_isLeft);
                    input.rollback.rollback();
                    return false;
                }
                applyFrozenGrabAuthorityFrameToGrabFrame(_grabFrame, frozenAuthorityFrame);
                desiredObjectWorld = frozenAuthorityFrame.desiredObjectWorld;
                desiredBodyWorld = frozenAuthorityFrame.desiredBodyWorld;
                const float objectScaleForLever =
                    std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
                _grabFrame.pivotAuthority.longLeverGameUnits =
                    computeLocalMeshMaxDistanceFromPoint(_grabFrame.localMeshTriangles, _grabFrame.gripEvidence.gripPointLocal) * objectScaleForLever;
                _grabFrame.authority.liveHandWorldAtGrab = handWorldTransform;
                _grabFrame.authority.handBodyWorldAtGrab = proxyFrameWorldAtGrab;
                _grabFrame.authority.objectNodeWorldAtGrab = objectWorldTransform;
                _grabFrame.hasTelemetryCapture = true;
                _grabFrame.handScaleAtGrab = handWorldTransform.scale;
                _grabFrame.traceId = grabTraceId;
                _grabFrame.freezeCaptureTelemetry(objectBodyId.value);
        return true;
    }

    struct Hand::GrabPostFreezeInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        const GrabBodyFrameCapture* bodyFrame = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::hknpBodyId objectBodyId{};
        const BodyBoneColliderSet* bodyBoneColliders = nullptr;
        const GrabSharedObjectContext* sharedContext = nullptr;
        bool joiningPeerHeldObject = false;
        bool looseWeaponGrab = false;
    };

    void Hand::initializePostFreezeGrab(
        RE::hknpWorld* world,
        const GrabPostFreezeInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabPostFreeze);
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyFrame = *input.bodyFrame;
        const auto& surface = *input.surface;
        const auto& sel = _currentSelection;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& palmPos = bodyFrame.palmPosition;
        const auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        const auto& grabGripPoint = surface.gripPoint;
        const auto objectBodyId = input.objectBodyId;
        const auto* bodyBoneColliders = input.bodyBoneColliders;
        const auto& sharedContext = *input.sharedContext;
        const bool joiningPeerHeldObject = input.joiningPeerHeldObject;
        const bool looseWeaponGrab = input.looseWeaponGrab;
                clearGrabExternalHandWorldTransform(_isLeft);
                _grabVisualHandTransform = handWorldTransform;
                _hasGrabVisualHandTransform = false;
                _lastPublishedGrabVisualHandTransform = {};
                _hasLastPublishedGrabVisualHandTransform = false;
                _grabVisualHandLerpStartTransform = handWorldTransform;
                _grabVisualHandLerpElapsedSeconds = 0.0f;
                _grabVisualHandLerpDurationSeconds = 0.0f;
                const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabGripPoint;
                const float initialGrabDistance =
                    std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
                const bool needsLargeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
                _grabFrame.fadeInGrabConstraint = needsLargeInitialSync;
                if (needsLargeInitialSync) {
                    _grabFrame.motorFadeReason = "largeInitialSync";
                } else {
                    _grabFrame.motorFadeReason = "none";
                }
                _controllerReleaseHistory.clear();
                _objectReleaseHistory.clear();
                _releaseObjectNotBefore = release_velocity::sampleTimeSeconds();
                _releaseControllerFrame = 0;
                _previousHeldRawHandWorld = {};
                _previousHeldHandPositionHavok = {};
                _lastHeldHandPositionHavok = {};
                _hasPreviousHeldRawHandWorld = false;
                _hasLastHeldHandPositionHavok = false;
                _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);


                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
                    "palmPos=({:.1f},{:.1f},{:.1f}) pivotA=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
                    handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, palmPos.x, palmPos.y, palmPos.z, grabPivotAWorld.x,
                    grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
                ROCK_LOG_DEBUG(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}", handName(), _grabFrame.authority.bodyLocal.translate.x, _grabFrame.authority.bodyLocal.translate.y,
                    _grabFrame.authority.bodyLocal.translate.z, _grabFrame.authority.bodyLocal.scale);

            {
                RE::NiTransform handBodyDiag{};
                RE::NiTransform objectBodyDiag{};
                const bool hasLiveDiag = tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyDiag) &&
                                         tryResolveLiveBodyWorldTransform(world, objectBodyId, objectBodyDiag);
                if (hasLiveDiag) {
                    ROCK_LOG_TRACE(Hand,
                        "{} DIAG: handBodyLive pos=({:.1f},{:.1f},{:.1f}) objBodyLive pos=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        handBodyDiag.translate.x,
                        handBodyDiag.translate.y,
                        handBodyDiag.translate.z,
                        objectBodyDiag.translate.x,
                        objectBodyDiag.translate.y,
                        objectBodyDiag.translate.z);
                }
                ROCK_LOG_TRACE(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})", handName(), handWorldTransform.translate.x,
                    handWorldTransform.translate.y, handWorldTransform.translate.z, objectWorldTransform.translate.x, objectWorldTransform.translate.y,
                    objectWorldTransform.translate.z);

                float comX, comY, comZ;
                if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ) && hasLiveDiag) {
                    ROCK_LOG_TRACE(Hand,
                        "{} B8 COM LIVE: comHk=({:.3f},{:.3f},{:.3f}) objBodyLive=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        comX,
                        comY,
                        comZ,
                        objectBodyDiag.translate.x,
                        objectBodyDiag.translate.y,
                        objectBodyDiag.translate.z);
                }
            }

            if (!grab_target::isRagdoll(sel.targetKind)) {
                const RE::hkVector4f zeroVel{ 0.0f, 0.0f, 0.0f, 0.0f };
                havok_runtime::setBodyVelocityDeferred(world, objectBodyId.value, zeroVel, zeroVel);

                for (auto bid : _heldBodyIds) {
                    if (bid != objectBodyId.value) {
                        havok_runtime::setBodyVelocityDeferred(world, bid, zeroVel, zeroVel);
                    }
                }
            }

            const auto grabActivation = activateHeldObjectBodySet(world, objectBodyId.value, _heldBodyIds);
            if (grabActivation.failedActivationCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                    handName(),
                    objectBodyId.value,
                    grabActivation.bodyCount,
                    grabActivation.activatedCount,
                    grabActivation.failedActivationCount);
            }

            suppressHandCollisionForGrab(world, bodyBoneColliders);

            // Native ragdoll inertia belongs to its joints/solver.
            if (!grab_target::isRagdoll(sel.targetKind)) {
                if (joiningPeerHeldObject && sharedContext.peerSavedObjectState) {
                    copyPeerInertiaSnapshot(_savedObjectState, *sharedContext.peerSavedObjectState);
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand joined peer-held object inertia snapshot: formID={:08X} peerMotions={} inertiaModified={}",
                        handName(),
                        sel.refr ? sel.refr->GetFormID() : 0,
                        sharedContext.peerSavedObjectState->motionInertiaStates.size(),
                        sharedContext.peerSavedObjectState->inertiaModified ? "yes" : "no");
                } else {
                    normalizeGrabbedInertiaForBodies(world, objectBodyId, _heldBodyIds, _savedObjectState, looseWeaponGrab);
                }
            }

    }

    struct Hand::GrabConstraintCommitInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::bhkWorld* bhkWorld = nullptr;
        RE::hknpBodyId objectBodyId{};
        const BodyBoneColliderSet* bodyBoneColliders = nullptr;
        active_grab_body_lifecycle::BodyLifecycleSnapshot* activeLifecycle = nullptr;
        const saved_grab_offset::HandOffset* savedGrabOffset = nullptr;
        bool hasSavedGrabOffset = false;
        bool joiningPeerHeldObject = false;
        bool looseWeaponGrab = false;
        float tau = 0.0f;
        float damping = 0.0f;
        float maxForce = 0.0f;
        float proportionalRecovery = 0.0f;
        float constantRecovery = 0.0f;
        GrabRollbackAction rollback{};
    };

    /*
     * The one mesh finger solve. Commit calls it when the seat is already
     * final; otherwise arrival at TouchHeld does. The live finger skeleton is
     * rebased from the live hand onto the hand pose being solved for.
     */
    void Hand::solveAndPublishGrabFingerPose(const GrabFingerSolveFrame& frame)
    {
        const auto mode = _grabFingerPose.mode;
        const bool legacyFingerPose = mode == grab_finger_pose_runtime::FingerPoseMode::June2;
        const bool pinchFingerPose = _grabFrame.seat.mode == GrabSeatMode::PinchPocket;
        const auto& localFingerPoseTriangles = !_grabFrame.fingerPoseLocalMeshTriangles.empty() ?
            _grabFrame.fingerPoseLocalMeshTriangles :
            _grabFrame.localMeshTriangles;
        const auto fingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, frame.objectWorld);

        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
        const bool liveFingerSnapshotValid =
            root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshot) &&
            rebaseFingerSkeletonSnapshot(liveFingerSnapshot, frame.liveHandWorld, frame.solveHandWorld);
        if (!liveFingerSnapshotValid) {
            liveFingerSnapshot = {};
        }

        std::vector<TriangleData> worldTriangles;
        grab_finger_pose_runtime::FingerSweepDebugCapture sweepDebug{};
        bool spatialIndexBuilt = false;
        bool commandedOpenDirectionsValid = false;
        if (legacyFingerPose) {
            worldTriangles = rebuildFingerPoseWorldTrianglesFromGrabFrame(_grabFrame, frame.objectWorld);
            _grabFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(worldTriangles, frame.solveHandWorld, _isLeft, frame.pivotWorld,
                fingerPoseTargets, g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, true,
                liveFingerSnapshotValid ? &liveFingerSnapshot : nullptr,
                g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits, _grabFrame.fingerPoseAimValid,
                g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits, -1.0f, g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                nullptr, nullptr, nullptr, nullptr,
                grab_finger_pose_runtime::FingerPoseMeshRelation::CurrentMeshRequiresVirtualSeat,
                mode);
        } else {
            const auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPoseBase(
                localFingerPoseTriangles,
                frame.objectWorld,
                frame.solveHandWorld,
                _isLeft,
                frame.pivotWorld,
                fingerPoseTargets,
                _grabFingerTriangleIndex,
                worldTriangles,
                grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                    .minValue = g_rockConfig.rockGrabFingerMinValue,
                    .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                    .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                    .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                    .allowSurfaceAimTargets = _grabFrame.fingerPoseAimValid,
                    .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                    .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                    .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                    .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                    .captureSweepDebug = g_rockConfig.rockDebugShowGrabFingerSweptArc,
                    .mode = mode,
                },
                &liveFingerSnapshot);
            _grabFingerPose = frozenSolve.pose;
            sweepDebug = frozenSolve.sweepDebug;
            spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
            commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
        }
        _grabFingerPose.mode = mode;
        if (pinchFingerPose) {
            applyPinchFingerPosePolicy(_grabFingerPose, _grabFrame);
        }
        grab_finger_pose_runtime::useThumbIndexCurveOnlyPose(_grabFingerPose);
        const auto* padIndex = spatialIndexBuilt ? &_grabFingerTriangleIndex : nullptr;
        std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padCaptureEvidence{};
        (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(_grabFingerPose, worldTriangles, fingerPoseTargets, liveFingerSnapshot,
            frame.objectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padCaptureEvidence, true, {}, padIndex);
        grab_finger_pose_runtime::captureSurfaceAimObjectLocal(_grabFingerPose, frame.objectWorld);

        _grabFingerSweepDebugCapture = sweepDebug;
        _grabFingerSweepDebugObjectWorld = frame.objectWorld;
        _hasGrabFingerSweepDebug = sweepDebug.valid;
        _grabFingerProbeStart = _grabFingerPose.probeStart;
        _grabFingerProbeEnd = _grabFingerPose.probeEnd;
        _hasGrabFingerProbeDebug = _grabFingerPose.candidateTriangleCount > 0;
        auto publishFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, frame.objectWorld);
        if (g_rockConfig.rockDebugShowGrabFingerProbes) {
            std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padEvidence{};
            (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(publishFingerPose, worldTriangles, fingerPoseTargets, liveFingerSnapshot,
                frame.objectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padEvidence, false, {}, padIndex);
            const auto padDebug = makeFingerPadPublishDebug(publishFingerPose, padEvidence);
            _grabFingerPadProbeStart = padDebug.padProbeStart;
            _grabFingerPadProbeEnd = padDebug.padProbeEnd;
            _grabFingerPadProbeHit = padDebug.padProbeHit;
            _grabFingerPadProbeHitValid = padDebug.padProbeHitValid;
            _hasGrabFingerPadProbeDebug = padDebug.hasPadProbeDebug;
            _grabFingerSurfaceTarget = padDebug.surfaceTarget;
            _grabFingerSurfaceTargetValid = padDebug.surfaceTargetValid;
            _hasGrabFingerSurfaceTargetDebug = padDebug.hasSurfaceTargetDebug;
        } else {
            _hasGrabFingerPadProbeDebug = false;
            _hasGrabFingerSurfaceTargetDebug = false;
        }
        if (!frame.touchHeld) {
            publishFingerPose = buildAcquisitionFingerPose(publishFingerPose, 0.0f);
        }
        applyRockGrabHandPose(_isLeft, publishFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
            _hasGrabFingerLocalTransforms, 0.0f, /*publishLocalTransforms=*/legacyFingerPose, frame.touchHeld);
        _grabFingerPosePublished = true;
        _grabFingerLocalTransformFinalizePending = !legacyFingerPose && frame.touchHeld && _grabFingerPose.solved;
        ROCK_LOG_DEBUG(Hand,
            "{} THREE-PHASE GRAB POSE SOLVED: stage={} phase={} pinch={} solved={} hits={} triangles={} spatial={} nodes={} tests={} commandedAnchors={}",
            handName(), frame.stage, grab_three_phase::phaseName(_grabAcquisitionPhase), pinchFingerPose ? "yes" : "no",
            _grabFingerPose.solved ? "yes" : "no", _grabFingerPose.hitCount, _grabFingerPose.candidateTriangleCount,
            _grabFingerPose.usedSpatialIndex ? "yes" : "no", _grabFingerPose.spatialNodeVisitCount, _grabFingerPose.spatialTriangleTestCount,
            commandedOpenDirectionsValid ? "yes" : "no");
    }

    bool Hand::commitGrabConstraintAndPose(
        RE::hknpWorld* world,
        const GrabConstraintCommitInput& input)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabConstraintCommit);
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& surface = *input.surface;
        auto* bhkWorld = input.bhkWorld;
        const auto objectBodyId = input.objectBodyId;
        const auto* bodyBoneColliders = input.bodyBoneColliders;
        auto& activeLifecycle = *input.activeLifecycle;
        const auto& savedGrabOffset = *input.savedGrabOffset;
        const bool hasSavedGrabOffset = input.hasSavedGrabOffset;
        const bool joiningPeerHeldObject = input.joiningPeerHeldObject;
        const bool looseWeaponGrab = input.looseWeaponGrab;
        const float tau = input.tau;
        const float damping = input.damping;
        const float maxForce = input.maxForce;
        const float proportionalRecovery = input.proportionalRecovery;
        const float constantRecovery = input.constantRecovery;
        const auto& sel = _currentSelection;
        const auto& proxyFrameWorldAtGrab = proxy.proxyFrameWorldAtGrab;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabGripPoint = surface.gripPoint;
        const bool meshGrabFound = surface.meshGrabFound;
        const char* grabPointMode = surface.pointMode;
        const char* grabFallbackReason = surface.fallbackReason;
        _grabOffsetAcquisition = {};
        _looseSharedAcquisition = {};
        _looseSharedAcquisitionPeer = 0;
        _looseSharedAcquisitionFrame = 0;
        _looseWeaponReturnStartFrame = 0;
        _looseSharedHandBlendAlpha = -1.0f;
        _grabOffsetMaximumGripError = 0.0f;
        _grabOffsetMaximumRotationError = 0.0f;
        const bool authoredPoseReady = !_grabFrame.authoredWeaponPose.valid() ||
            publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr, _grabFrame.authoredWeaponPose);
            if (!authoredPoseReady) {
                ROCK_LOG_WARN(Hand, "{} authored weapon pose rejected before constraint commit", handName());
            } else if (grab_target::isRagdoll(sel.targetKind) &&
                (!surface.surfaceHit.valid || !surface.surfaceHit.sourceShape ||
                    surface.surfaceHit.sourceNode != _grabFrame.heldNode)) {
                // Pocket refinement can choose another surface after primary
                // resolution. Never commit that point to a different limb.
                ROCK_LOG_WARN(Hand, "{} ragdoll final surface rejected: body={} source='{}' owner='{}' valid={}",
                    handName(), objectBodyId.value, nodeDebugName(surface.surfaceHit.sourceNode),
                    nodeDebugName(_grabFrame.heldNode), surface.surfaceHit.valid);
            } else {
                RE::NiPoint3 legacyPalmPivotAWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
                RE::NiPoint3 grabPivotAWorld =
                    _grabFrame.hasTelemetryCapture ? _grabFrame.authority.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
                const float gameToHkScale = gameToHavokScale();
                const RE::NiTransform initialDesiredBodyWorld = _grabFrame.authority.desiredBodyWorldAtGrab;

                float pivotAHk[4];
                pivotAHk[0] = grabPivotAWorld.x * gameToHkScale;
                pivotAHk[1] = grabPivotAWorld.y * gameToHkScale;
                pivotAHk[2] = grabPivotAWorld.z * gameToHkScale;
                pivotAHk[3] = 0.0f;

                float gripWorldHk[4];
                gripWorldHk[0] = grabGripPoint.x * gameToHkScale;
                gripWorldHk[1] = grabGripPoint.y * gameToHkScale;
                gripWorldHk[2] = grabGripPoint.z * gameToHkScale;
                gripWorldHk[3] = 0.0f;

                {
                    float pivotAToGrab = std::sqrt(
                        (pivotAHk[0] - gripWorldHk[0]) * (pivotAHk[0] - gripWorldHk[0]) + (pivotAHk[1] - gripWorldHk[1]) * (pivotAHk[1] - gripWorldHk[1]) +
                        (pivotAHk[2] - gripWorldHk[2]) * (pivotAHk[2] - gripWorldHk[2]));
                    float legacyPalmPivotAToHandOrigin = std::sqrt((legacyPalmPivotAWorld.x - handWorldTransform.translate.x) * (legacyPalmPivotAWorld.x - handWorldTransform.translate.x) +
                        (legacyPalmPivotAWorld.y - handWorldTransform.translate.y) * (legacyPalmPivotAWorld.y - handWorldTransform.translate.y) +
                        (legacyPalmPivotAWorld.z - handWorldTransform.translate.z) * (legacyPalmPivotAWorld.z - handWorldTransform.translate.z));
                    float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) * (grabPivotAWorld.x - handWorldTransform.translate.x) +
                        (grabPivotAWorld.y - handWorldTransform.translate.y) * (grabPivotAWorld.y - handWorldTransform.translate.y) +
                        (grabPivotAWorld.z - handWorldTransform.translate.z) * (grabPivotAWorld.z - handWorldTransform.translate.z));
                    ROCK_LOG_DEBUG(Hand,
                        "GRAB DIAG {}: legacyPalmPivotAWorld=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
                        "pocket=({:.1f},{:.1f},{:.1f}) grip=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
                        "frozenPivotB=({:.2f},{:.2f},{:.2f}) activePoint={} palmSeatPoint={} "
                        "pivotAToGrab_hk={:.4f} ({:.1f} game units) legacyPalmPivotAToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units "
                        "selectionToGripEvidence={:.1f} fingerPoseAim={} fingerPoseAimReason={}",
                        handName(), legacyPalmPivotAWorld.x, legacyPalmPivotAWorld.y, legacyPalmPivotAWorld.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                        grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                        _grabFrame.authority.pivotBBodyLocalGame.x, _grabFrame.authority.pivotBBodyLocalGame.y, _grabFrame.authority.pivotBBodyLocalGame.z,
                        _grabFrame.seat.activeGrabPointMode, _grabFrame.seat.palmSeatPointMode, pivotAToGrab,
                        pivotAToGrab * havokToGameScale(), legacyPalmPivotAToHandOrigin, pivotAToHandOrigin, _grabFrame.pivotAuthority.selectionDistanceGameUnits,
                        _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);
                }

                const char* driveReason = joiningPeerHeldObject ? "joining-peer-held-loose-object" : "ordinary-dynamic-loose-object";
                RE::NiTransform initialProxyWorld = proxyFrameWorldAtGrab;
                RE::NiPoint3 initialConstraintPivotWorld = grabPivotAWorld;
                // A local grasp seat can turn the object up to a quarter turn
                // about its grip. Ramp the target there like the other seats
                // that rotate at capture; the visual hand stays on the
                // controller until the ramp ends.
                if (!joiningPeerHeldObject &&
                    sel.targetKind == grab_target::Kind::LooseObject &&
                    (_grabFrame.syntheticLooseWeaponPrimaryAttach || sel.forcedArrival || _grabFrame.seat.localGraspSeat)) {
                    _grabOffsetAcquisition = looseWeaponGrab ?
                        (g_rockConfig.rockWeaponSupportGripHandLerpEnabled ?
                            grab_offset_acquisition::beginWeapon(proxyFrameWorldAtGrab, _grabFrame.authority.bodyWorldAtGrab,
                                _grabFrame.proxyAuthorityBodyHandSpace, _grabFrame.authority.pivotBConstraintLocalGame,
                                looseWeaponAcquireConfig()) : grab_offset_acquisition::Transition<RE::NiTransform>{}) :
                        grab_offset_acquisition::begin(proxyFrameWorldAtGrab, _grabFrame.authority.bodyWorldAtGrab,
                            _grabFrame.proxyAuthorityBodyHandSpace, _grabFrame.authority.pivotBConstraintLocalGame);
                    if (_grabOffsetAcquisition.active) {
                        initialProxyWorld = grab_offset_acquisition::advance(_grabOffsetAcquisition,
                            proxyFrameWorldAtGrab, _grabFrame.proxyAuthorityBodyHandSpace,
                            _grabFrame.authority.pivotBConstraintLocalGame, 0.0f);
                        initialConstraintPivotWorld = generatedProxyLocalPointToWorld(initialProxyWorld,
                            _grabFrame.authority.pivotAHandBodyLocalGame);
                        // The initial constraint is already satisfied. Ramp its
                        // target, not its force budget, to avoid letting the
                        // body sag while a second startup fade is running.
                        _grabFrame.fadeInGrabConstraint = false;
                        _grabFrame.motorFadeReason = "offsetAcquisitionTarget";
                        const auto initialGrip = transform_math::localPointToWorld(
                            _grabFrame.authority.bodyWorldAtGrab, _grabFrame.authority.pivotBConstraintLocalGame);
                        ROCK_LOG_INFO(Hand,
                            "{} OFFSET_ACQUIRE begin trace={} ref={:08X} source={} gripCorrection={:.3f}gu rotationCorrection={:.3f}deg duration={:.3f}s",
                            handName(), _grabFrame.traceId, sel.refr ? sel.refr->GetFormID() : 0,
                            grabFallbackReason, pointDistanceGameUnits(initialGrip, grabPivotAWorld),
                            grab_offset_acquisition::rotationAngleRadians(_grabFrame.authority.bodyWorldAtGrab.rotate,
                                initialDesiredBodyWorld.rotate) * 57.295779513f,
                            _grabOffsetAcquisition.durationSeconds);
                    }
                }
                if (!createProxyConstraintGrabDrive(
                        bhkWorld,
                        world,
                        objectBodyId,
                        initialProxyWorld,
                        handWorldTransform,
                        initialConstraintPivotWorld,
                        tau,
                        damping,
                        maxForce,
                        held_object_drive_policy::sanitizeMotorAuthorityScale(sharedGrabAuthorityForceScale(joiningPeerHeldObject)),
                        proportionalRecovery,
                        constantRecovery,
                        looseWeaponGrab,
                        driveReason)) {
                    ROCK_LOG_ERROR(Hand,
                        "{} hand GRAB FAILED: proxy constraint creation failed bodyId={} targetBody=({:.2f},{:.2f},{:.2f}) pivotBConstraint=({:.2f},{:.2f},{:.2f}) joiningPeer={}",
                        handName(),
                        objectBodyId.value,
                        initialDesiredBodyWorld.translate.x,
                        initialDesiredBodyWorld.translate.y,
                        initialDesiredBodyWorld.translate.z,
                        activeProxyConstraintPivotBLocalGame().x,
                        activeProxyConstraintPivotBLocalGame().y,
                        activeProxyConstraintPivotBLocalGame().z,
                        joiningPeerHeldObject ? "yes" : "no");
                }
            }

            const bool driveCreated = _activeConstraint.isValid() && _grabAuthorityProxy.isValid();
            if (!driveCreated) {
                if (_grabFrame.authoredWeaponPose.valid()) {
                    (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
                    (void)frik_visual_authority::blockPrimaryHandWeaponPose(
                        _isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight", false);
                }
                ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: proxy-constraint dynamic grab creation failed", handName());
                destroyGrabAuthorityProxy(bhkWorld);
                clearGrabExternalHandWorldTransform(_isLeft);
                if (!joiningPeerHeldObject) {
                    restoreGrabbedInertia(world, _savedObjectState);
                }
                input.rollback.rollback();
                restoreHandCollisionAfterGrab(world);
                restoreBodyCollisionAfterHeldLooseWeapon(world);
                _savedObjectState.clear();
                _heldBodyIds.clear();
                _heldDriveDecision = {};
                _heldObjectIsLooseWeapon = false;
                return false;
            }

            _ragdollGrabOwner.reset(grab_target::isRagdoll(sel.targetKind) ?
                havok_runtime::getCollisionObjectFromBody(world, _savedObjectState.bodyId) : nullptr);
            _ragdollGrabSurface.reset(grab_target::isRagdoll(sel.targetKind) ? input.surface->surfaceHit.sourceShape : nullptr);
            _ragdollGrabTriangle = input.surface->surfaceHit.sourceTriangleIndex;
            _heldObjectIsLooseWeapon = looseWeaponGrab;
            if (_heldObjectIsLooseWeapon) {
                suppressBodyCollisionForHeldLooseWeapon(world, bodyBoneColliders);
            }
            const auto massSummaryAtGrab = readHeldBodyMassSummary(
                world,
                _savedObjectState.bodyId,
                _heldBodyIds,
                _heldDriveDecision.includeConnectedMass);
            const float effectiveMassAtGrab = effectiveGrabMotorMass(massSummaryAtGrab.motorMass());
            const float sharedLinearForce = _activeConstraint.linearMotor ?
                (std::max)(std::fabs(_activeConstraint.linearMotor->minForce), std::fabs(_activeConstraint.linearMotor->maxForce)) :
                maxForce;
            const float sharedAngularForce = _activeConstraint.angularMotor ?
                (std::max)(std::fabs(_activeConstraint.angularMotor->minForce), std::fabs(_activeConstraint.angularMotor->maxForce)) :
                0.0f;
            ROCK_LOG_DEBUG(Hand,
                "{} hand dynamic grab created: drive={} bodyDriveMode={} driveReason={} forceShare={:.2f} linearScope={} angularScope={} massScope={} looseWeapon={} constraint={} proxyBody={} handBody={} objBody={} heldBodies={} mass={:.2f} effectiveMotorMass={:.2f} primaryMass={:.2f} massBodies={} motions={} longLever={:.1f}gu linearTau={:.3f} angularTau={:.3f} linearDamping={:.2f} angularDamping={:.2f} linearForce={:.0f} angularForce={:.0f} propRecov={:.1f} constRecov={:.1f} rotRef={}",
                handName(),
                kHeldObjectDriveName,
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.reason,
                sharedGrabAuthorityForceScale(joiningPeerHeldObject),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
                _heldObjectIsLooseWeapon ? "yes" : "no",
                _activeConstraint.constraintId,
                _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
                _handBody.getBodyId().value,
                objectBodyId.value,
                _heldBodyIds.size(),
                massSummaryAtGrab.motorMass(),
                effectiveMassAtGrab,
                massSummaryAtGrab.primaryMass,
                massSummaryAtGrab.sampledBodies,
                massSummaryAtGrab.uniqueMotions,
                _grabFrame.pivotAuthority.longLeverGameUnits,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : tau,
                _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : g_rockConfig.rockGrabAngularTau,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->damping : damping,
                _activeConstraint.angularMotor ? _activeConstraint.angularMotor->damping : g_rockConfig.rockGrabAngularDamping,
                sharedLinearForce,
                sharedAngularForce,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->proportionalRecoveryVelocity : proportionalRecovery,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->constantRecoveryVelocity : constantRecovery,
                kGrabObjectRotationReferenceName);

            const auto heldFlagLeases =
                acquireHeldObjectBodyFlagLeases(world, _savedObjectState.bodyId.value, _heldBodyIds, _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this));
            if (heldFlagLeases.failedLeaseCount > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB held body flag lease incomplete: primaryBody={} bodies={} collision={} authority={} failed={}",
                    handName(),
                    _savedObjectState.bodyId.value,
                    heldFlagLeases.bodyCount,
                    heldFlagLeases.collisionLeaseCount,
                    heldFlagLeases.authorityLeaseCount,
                    heldFlagLeases.failedLeaseCount);
            } else {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand GRAB held body flag leases acquired: primaryBody={} bodies={} collision={} authority={}",
                    handName(),
                    _savedObjectState.bodyId.value,
                    heldFlagLeases.bodyCount,
                    heldFlagLeases.collisionLeaseCount,
                    heldFlagLeases.authorityLeaseCount);
            }
            clearHeldBodyContactSnapshot();
            _activeGrabLifecycle = std::move(activeLifecycle);

            publishHeldBodyScope(world);
            _isHoldingFlag.store(true, std::memory_order_release);


            stopSelectionHighlight();
            clearSelectedCloseFingerPose();
            _grabFingerPose = {};
            _grabFingerPose.mode = grab_finger_pose_runtime::fingerPoseMode(g_rockConfig.rockGrabFingerPoseMode);
            _legacyFingerPoseUpdates = 0;
            _legacyFingerPoseDeltaTime = 0.0f;
            _grabFingerTriangleIndex.clear();
            const bool useLooseWeaponPrimaryAttachHandPose = _grabFrame.syntheticLooseWeaponPrimaryAttach;
            _hasGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled && !useLooseWeaponPrimaryAttachHandPose;
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
            _grabFingerPosePublished = false;
            _grabFingerLocalTransformFinalizePending = false;
            if (useLooseWeaponPrimaryAttachHandPose) {
                grab_finger_pose_runtime::SolvedGrabFingerPose savedGrabFingerPose{};
                const bool fixedAidPose = g_rockConfig.rockImmersiveAidEnabled &&
                    immersive_aid::usesStimpakPose(immersive_aid::classify(sel.refr ? sel.refr->GetObjectReference() : nullptr));
                const bool hasSavedGrabFingerPose = (fixedAidPose || g_rockConfig.rockGrabMeshFingerPoseEnabled) &&
                    hasSavedGrabOffset &&
                    tryBuildSavedGrabOffsetFingerPose(savedGrabOffset, savedGrabFingerPose);
                if (_grabFrame.authoredWeaponPose.valid()) {
                    _grabFingerPosePublished = authoredPoseReady;
                } else if (hasSavedGrabFingerPose) {
                    /*
                     * Reuse the same saved offset that established the object
                     * attach. This synthetic attach has no mesh contact
                     * to solve fingers from, so without the resolved snapshot it
                     * would fall back to a generic canned pose. This is a one-time
                     * publish; mesh-resolve paths must not mutate it afterward.
                     */
                    _grabFingerPose = savedGrabFingerPose;
                    applyRockGrabHandPose(_isLeft,
                        _grabFingerPose,
                        _grabFingerJointPose,
                        _hasGrabFingerJointPose,
                        _grabFingerLocalTransforms,
                        _grabFingerLocalTransformMask,
                        _hasGrabFingerLocalTransforms,
                        0.0f,
                        /*publishLocalTransforms=*/false);
                    _grabFingerPosePublished = true;
                    if (savedGrabOffset.hasExactFingerRotations) {
                        frik_visual_authority::FingerLocalTransformOverride exact{};
                        const auto handPose = frik_visual_authority::makeHandPoseDataFromJointValues(savedGrabOffset.fingerJointValues);
                        bool applied = frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(_isLeft), handPose, &exact) &&
                            exact.enabledMask == 0x7fff;
                        if (applied) {
                            for (std::size_t joint = 0; joint < 15; ++joint) {
                                for (std::size_t row = 0; row < 3; ++row) {
                                    for (std::size_t column = 0; column < 3; ++column) {
                                        exact.localTransforms[joint].rotate.entry[row][column] =
                                            savedGrabOffset.exactFingerRotations[joint][row * 3 + column];
                                    }
                                }
                            }
                            applied = frik_visual_authority::setHandPoseCustomLocalTransforms("ROCK_Grab", handFromBool(_isLeft), &exact, 100);
                        }
                        if (applied) {
                            for (std::size_t joint = 0; joint < 15; ++joint) _grabFingerLocalTransforms[joint] = exact.localTransforms[joint];
                            _grabFingerLocalTransformMask = exact.enabledMask;
                            _hasGrabFingerLocalTransforms = true;
                            ROCK_LOG_INFO(Hand, "{} saved exact grab pose applied: 15 normal-rig joints", handName());
                        } else {
                            // Never retain the scalar seed as if the exact pose
                            // succeeded. Existing release cleanup owns this tag.
                            (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
                            _grabFingerPosePublished = false;
                            ROCK_LOG_WARN(Hand, "{} saved exact grab pose unavailable: FRIK rejected the normal-rig transform override", handName());
                        }
                    }
                    ROCK_LOG_INFO(Hand,
                        "{} hand loose weapon attach: applying savedGrabOffset finger pose",
                        handName());
                } else {
                    _grabFingerPosePublished = publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr, _grabFrame.authoredWeaponPose);
                    if (!_grabFingerPosePublished) {
                        ROCK_LOG_WARN(Hand, "{} hand loose weapon attach: failed to publish FRIK weapon hand pose", handName());
                    }
                }
            } else if (_hasGrabFingerPose) {
                const bool legacyFingerPose = _grabFingerPose.mode == grab_finger_pose_runtime::FingerPoseMode::June2;
                ROCK_LOG_INFO(Hand, "{} dynamic finger pose mode={} trace={}", handName(),
                    legacyFingerPose ? "2 June2" : "1 Current", _grabFrame.traceId);
                const bool pinchFingerPose = _grabFrame.seat.mode == GrabSeatMode::PinchPocket;
                const bool touchHeldAtCommit = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld;
                const bool deferFingerPoseUntilSettledTouch =
                    !pinchFingerPose &&
                    !touchHeldAtCommit &&
                    _grabFrame.seat.requiresSettledVisualHandRelation;
                if ((pinchFingerPose || legacyFingerPose) && !touchHeldAtCommit) {
                    ROCK_LOG_DEBUG(Hand, "{} THREE-PHASE GRAB POSE: contact-time solve deferred until TouchHeld phase={} cachedTriangles={} poseTargets={}", handName(),
                        grab_three_phase::phaseName(_grabAcquisitionPhase),
                        _grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.localMeshTriangles.size() : _grabFrame.fingerPoseLocalMeshTriangles.size(),
                        _grabFrame.fingerPoseTargetCount);
                } else if (deferFingerPoseUntilSettledTouch) {
                    /*
                     * A programmatic arrival whose pivot still needs seated
                     * promotion has no final object-hand relation yet. Publish
                     * only the neutral pre-close pose during approach. The
                     * promotion path solves once against the final live seat,
                     * after the object reaches the hand.
                     */
                    const grab_finger_pose_runtime::SolvedGrabFingerPose pendingTarget{};
                    const auto acquisitionFingerPose = buildAcquisitionFingerPose(pendingTarget, 0.0f);
                    applyRockGrabHandPose(_isLeft,
                        acquisitionFingerPose,
                        _grabFingerJointPose,
                        _hasGrabFingerJointPose,
                        _grabFingerLocalTransforms,
                        _grabFingerLocalTransformMask,
                        _hasGrabFingerLocalTransforms,
                        0.0f,
                        /*publishLocalTransforms=*/false,
                        /*snapJointPoseToTarget=*/true);
                    ROCK_LOG_DEBUG(Hand,
                        "{} THREE-PHASE GRAB POSE: final mesh solve deferred until seated TouchHeld phase={} cachedTriangles={} poseTargets={}",
                        handName(),
                        grab_three_phase::phaseName(_grabAcquisitionPhase),
                        _grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.localMeshTriangles.size() : _grabFrame.fingerPoseLocalMeshTriangles.size(),
                        _grabFrame.fingerPoseTargetCount);
                } else {
                    // The seat is final: solve once against the commanded relation.
                    solveAndPublishGrabFingerPose(GrabFingerSolveFrame{
                        .objectWorld = legacyFingerPose ? objectWorldTransform : _grabFrame.authority.desiredObjectWorldAtGrab,
                        .liveHandWorld = handWorldTransform,
                        .solveHandWorld = handWorldTransform,
                        .pivotWorld = _grabFrame.hasTelemetryCapture ? _grabFrame.authority.grabPivotWorldAtGrab :
                                                                       computeGrabPivotAWorld(world, handWorldTransform),
                        .touchHeld = touchHeldAtCommit,
                        .stage = "commit",
                    });
                }
            }

        return true;
    }

    GrabAttemptResult Hand::grabSelectedObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        const BodyBoneColliderSet* bodyBoneColliders,
        const GrabSharedObjectContext& sharedContext,
        const AuthoredWeaponGripPose* transferPose)
    {
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabAcquisition);
        ValidatedGrabSelection validatedSelection{};
        if (!validateSelectedGrab(world, sharedContext, validatedSelection)) {
            return GrabAttemptResult::Rejected;
        }

        const auto& sel = _currentSelection;
        const auto selectedRef = validatedSelection.retainedRef;
        const bool joiningPeerHeldObject = validatedSelection.joiningPeerHeldObject;
        if (joiningPeerHeldObject) performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionPeerHeld);
        if (sel.equippedWeaponTransfer) performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionEquippedTransfer);
        const bool grabbedFromPullCatch = validatedSelection.grabbedFromPullCatch;
        const bool looseWeaponGrab = validatedSelection.looseWeaponGrab;
        const bool handPocketOnlyGrab = validatedSelection.handPocketOnlyGrab;
        saved_grab_offset::HandOffset savedGrabOffset{};
        bool hasSavedGrabOffset = false;

        auto objectBodyId = validatedSelection.bodyId;
        auto* rootNode = validatedSelection.rootNode;
        auto* bhkWorld = validatedSelection.bhkWorld;
        std::string objName = std::move(validatedSelection.objectName);

        const char* motionTypeStr = "UNKNOWN";
        std::uint16_t selectedOriginalMotionPropsId = 1;
        {
            auto* body = havok_runtime::getBody(world, objectBodyId);
            auto* objMotion = havok_runtime::getBodyMotion(world, objectBodyId);
            if (body && objMotion) {
                std::uint32_t bodyFlags = body->flags;
                std::uint8_t bodyMotionPropsId = static_cast<std::uint8_t>(body->motionPropertiesId);
                std::uint16_t motionPropsId = selectedOriginalMotionPropsId;
                if (havok_runtime::tryReadMotionPropertiesId(objMotion, motionPropsId)) {
                    selectedOriginalMotionPropsId = motionPropsId;
                }

                havok_runtime::MotionVelocityCaps velocityCaps{};
                const bool hasVelocityCaps = havok_runtime::tryReadMotionVelocityCaps(objMotion, velocityCaps);
                const float maxLinVel = hasVelocityCaps ? velocityCaps.maxLinearVelocity : 0.0f;
                const float maxAngVel = hasVelocityCaps ? velocityCaps.maxAngularVelocity : 0.0f;

                bool isDynamicInit = (bodyFlags & 0x2) != 0;
                bool isKeyframed = (bodyFlags & 0x4) != 0;

                switch (motionPropsId & 0xFF) {
                case 0:
                    motionTypeStr = "STATIC";
                    break;
                case 1:
                    motionTypeStr = "DYNAMIC";
                    break;
                case 2:
                    motionTypeStr = "KEYFRAMED";
                    break;
                default:
                    motionTypeStr = "OTHER";
                    break;
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand GRAB MOTION: body={} motionPropsId={} ({}) "
                    "bodyFlags=0x{:08X} dynInit={} keyfr={} bodyPropsId={} "
                    "maxLinVel={:.1f} maxAngVel={:.1f}",
                    handName(), objectBodyId.value, motionPropsId, motionTypeStr, bodyFlags, isDynamicInit, isKeyframed, bodyMotionPropsId, maxLinVel, maxAngVel);

                {
                    static bool libraryDumped = false;
                    if (!libraryDumped) {
                        libraryDumped = true;
                        havok_runtime::MotionPropertiesLibrarySnapshot motionProperties{};
                        if (havok_runtime::snapshotMotionPropertiesLibrary(world, motionProperties)) {
                            ROCK_LOG_TRACE(Hand, "Motion properties library: {} entries, stride=0x40", motionProperties.count);
                            for (std::uint32_t i = 0; i < motionProperties.copiedCount; i++) {
                                const auto& record = motionProperties.records[i];
                                const auto* f = record.values;
                                const auto* u = record.words;
                                ROCK_LOG_TRACE(Hand,
                                    "  [{}] +00: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+10: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+20: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                    "+30: {:.4f} {:.4f} {:.4f} {:.4f}",
                                    i, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]);
                                ROCK_LOG_TRACE(Hand,
                                    "  [{}] hex: {:08X} {:08X} {:08X} {:08X} | "
                                    "{:08X} {:08X} {:08X} {:08X}",
                                    i, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                            }
                        }
                    }
                }
            }
        }

        const std::uint64_t grabTraceId = nextGrabTraceId();

        GrabBodyPreparation bodyPreparation{};
        prepareSelectedGrabBodies(world, sharedContext, validatedSelection, bodyPreparation);
        auto& activeLifecycle = bodyPreparation.activeLifecycle;
        const bool consumedPullPrepLifecycle = bodyPreparation.consumedPullPrepLifecycle;
        const auto& beforePrepBodySet = bodyPreparation.beforePrepBodySet;
        const bool beforePrepScanCacheHit = bodyPreparation.beforePrepScanCacheHit;
        const bool motionConverted = bodyPreparation.motionConverted;
        const bool collisionEnabled = bodyPreparation.collisionEnabled;
        auto& preparedBodySet = bodyPreparation.preparedBodySet;
        const bool preparedScanCacheHit = bodyPreparation.preparedScanCacheHit;
        const bool preparedBodySetPostPrepComplete = bodyPreparation.preparedBodySetPostPrepComplete;
        const auto& scanOptions = bodyPreparation.scanOptions;

        auto restoreFailedGrabPrep = [&]() {
            if (!joiningPeerHeldObject) {
                if (consumedPullPrepLifecycle) {
                    const auto releaseRestorePolicy =
                        active_grab_body_lifecycle::releaseRestorePolicyForTargetKind(sel.targetKind);
                    const auto releasePlan = activeLifecycle.restorePlanForRelease(
                        releaseRestorePolicy,
                        sel.targetKind,
                        active_grab_body_lifecycle::BodyReleaseIntent::PhysicalDrop);
                    restoreActiveGrabLifecycle(world,
                        activeLifecycle,
                        releasePlan,
                        objectBodyId.value,
                        handName(),
                        "failed-pull-catch-setup-physical-drop");
                    if (activeLifecycle.hasIncompleteNativeScan()) {
                        if (active_grab_body_lifecycle::shouldSkipIncompleteScanRootRestore(releasePlan, selectedOriginalMotionPropsId)) {
                            ROCK_LOG_DEBUG(Hand,
                                "{} hand failed pull-catch setup: skipped recursive root restore for converted loose-object physical drop root='{}' motionProps={} preservedMotion={}",
                                handName(),
                                nodeDebugName(rootNode),
                                selectedOriginalMotionPropsId,
                                releasePlan.preservedConvertedMotionCount);
                        } else {
                            restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(), "failed-pull-catch-setup-incomplete-scan");
                        }
                    }
                    if (world && objectBodyId.value != INVALID_BODY_ID) {
                        const auto releaseActivation = activateHeldObjectBodySet(world, objectBodyId.value, _pulledBodyIds);
                        if (releaseActivation.failedActivationCount > 0) {
                            ROCK_LOG_WARN(Hand,
                                "{} hand failed pull-catch setup activation incomplete: primaryBody={} bodies={} activated={} failed={}",
                                handName(),
                                objectBodyId.value,
                                releaseActivation.bodyCount,
                                releaseActivation.activatedCount,
                                releaseActivation.failedActivationCount);
                        }
                    }
                } else {
                    restoreActiveGrabLifecycle(world,
                        activeLifecycle,
                        activeLifecycle.restorePlanForFailure(),
                        objectBodyId.value,
                        handName(),
                        "failed-setup");
                    if (activeLifecycle.hasIncompleteNativeScan()) {
                        restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(), "failed-setup-incomplete-scan");
                    }
                }
            }
            /*
             * Some setup failures occur after body resolution has staged a saved
             * object state but before ROCK owns a live grab constraint. Clear it
             * here so a rejected strict-authority grab cannot look held or block
             * later reset cleanup.
             */
            _savedObjectState.clear();
        };
        GrabPreparationTransaction grabPreparationTransaction{ restoreFailedGrabPrep };

        GrabProxyPreparation proxyPreparation{};
        if (!prepareGrabProxyAuthority(world, handWorldTransform, proxyPreparation)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand GRAB FAILED: live palm anchor frame unavailable before grab evidence capture bodyId={} handBody={} source={} formID={:08X}",
                handName(),
                objectBodyId.value,
                _handBody.isValid() ? _handBody.getBodyId().value : INVALID_BODY_ID,
                proxyPreparation.proxyFrameSourceAtGrab,
                sel.refr ? sel.refr->GetFormID() : 0);
            _grabFrame.clear();
            _heldBodyIds.clear();
            _heldBodyIdsCount.store(0, std::memory_order_release);
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }
        const auto& grabAuthorityPivotAWorld = proxyPreparation.grabAuthorityPivotAWorld;
        const auto& palmPocketPivotAWorld = proxyPreparation.palmPocketPivotAWorld;
        const auto& grabPalmBasisDelta = proxyPreparation.palmBasisDelta;
        const char* proxyFrameSourceAtGrab = proxyPreparation.proxyFrameSourceAtGrab;
        const float palmPocketToProxyDeltaGameUnits = proxyPreparation.palmPocketToProxyDeltaGameUnits;
        /*
         * Live proxy motion and close-grab pocket acquisition both resolve the
         * configured seat offset in generated/proxy local space. The generated
         * collision body path and hidden proxy seat now agree at startup.
         */
        if (grabPalmBasisDelta.rotationDegrees > kGrabFrameMismatchRawProxyRotationWarnDegrees) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} PROXY GRAB PALM BASIS MISMATCH: ref='{}' formID={:08X} source={} rawToProxy={:.1f}deg axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f}) proxyPivot=({:.1f},{:.1f},{:.1f}) palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f}gu phase={}",
                handName(),
                objName,
                sel.refr ? sel.refr->GetFormID() : 0,
                proxyFrameSourceAtGrab,
                grabPalmBasisDelta.rotationDegrees,
                grabPalmBasisDelta.xAxisDegrees,
                grabPalmBasisDelta.yAxisDegrees,
                grabPalmBasisDelta.zAxisDegrees,
                grabPalmBasisDelta.rawDeterminant,
                grabPalmBasisDelta.proxyDeterminant,
                grabAuthorityPivotAWorld.x,
                grabAuthorityPivotAWorld.y,
                grabAuthorityPivotAWorld.z,
                palmPocketPivotAWorld.x,
                palmPocketPivotAWorld.y,
                palmPocketPivotAWorld.z,
                palmPocketToProxyDeltaGameUnits,
                grab_three_phase::phaseName(_grabAcquisitionPhase));
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand object-tree prep: ref='{}' formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seedBody={} seeded={} scanSource={}/{} preparedComplete={} cachedBodyIds={} cacheHits={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} "
            "latePrepared={} incompleteScan={} collisionObjects={} visitedNodes={} setMotion={} enableCollision={} sharedPeer={} "
            "proxyPivot=({:.1f},{:.1f},{:.1f}) palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f} palmBasis={:.1f}deg axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f})",
            handName(),
            objName,
            sel.refr->GetFormID(),
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            scanOptions.seedBodyId,
            preparedBodySet.diagnostics.seedBodiesAdded,
            beforePrepScanCacheHit ? "cache" : "direct",
            preparedScanCacheHit ? "cache" : "direct",
            preparedBodySetPostPrepComplete ? "yes" : "no",
            preparedBodySet.diagnostics.cachedBodyIds,
            preparedBodySet.diagnostics.cachedScanHits,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips,
            activeLifecycle.latePreparedBodyCount(),
            activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no",
            preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed",
            joiningPeerHeldObject ? "yes" : "no",
            grabAuthorityPivotAWorld.x,
            grabAuthorityPivotAWorld.y,
            grabAuthorityPivotAWorld.z,
            palmPocketPivotAWorld.x,
            palmPocketPivotAWorld.y,
            palmPocketPivotAWorld.z,
            palmPocketToProxyDeltaGameUnits,
            grabPalmBasisDelta.rotationDegrees,
            grabPalmBasisDelta.xAxisDegrees,
            grabPalmBasisDelta.yAxisDegrees,
            grabPalmBasisDelta.zAxisDegrees,
            grabPalmBasisDelta.rawDeterminant,
            grabPalmBasisDelta.proxyDeterminant);

        GrabMeshCaptureSetup meshCaptureSetup{};
        if (!prepareGrabMeshCapture(handWorldTransform, validatedSelection, objName, meshCaptureSetup)) {
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }
        auto* collidableNode = meshCaptureSetup.collidableNode;
        auto* meshSourceNode = meshCaptureSetup.meshSourceNode;
        RE::NiTransform objectWorldTransform = meshCaptureSetup.objectWorldTransform;

        namespace authoredPolicy = loose_weapon_authored_grab_policy;
        const auto acquisitionSource = authoredPolicy::acquisitionSource(
            grabbedFromPullCatch, sel.forcedArrival, transferPose != nullptr);
        const bool authoredWeapon = looseWeaponGrab && !isThrowableLooseWeapon(selectedLooseWeaponForm(sel));
        loose_weapon_grip_zone::NearGrab preparedAuthoredGrip{};
        const auto prepareAuthoredGrip = [&](bool arrival) {
            performance_profiler::ScopedTimer preparationTimer(performance_profiler::Scope::GrabAuthoredPreparation);
            preparedAuthoredGrip = {};
            (void)loose_weapon_grip_zone::tryResolveNearGrab(_isLeft, sel.refr,
                preparedAuthoredGrip, joiningPeerHeldObject);
            const auto role = authoredPolicy::surfaceIndependentAcquisitionRole(
                arrival, transferPose != nullptr,
                transferPose ? transferPose->role : authoredPolicy::Role::None,
                preparedAuthoredGrip.arrangement, joiningPeerHeldObject, preparedAuthoredGrip.role);
            if (role == authoredPolicy::Role::None) return false;
            preparedAuthoredGrip.role = role;
            if (transferPose) preparedAuthoredGrip.pose = *transferPose;
            else (void)loose_weapon_grip_zone::tryResolveAuthoredGrabPose(_isLeft, sel.refr, role, preparedAuthoredGrip.pose);
            const auto* weapon = selectedLooseWeaponForm(sel);
            return preparedAuthoredGrip.handWorldValid && preparedAuthoredGrip.pose.valid() && weapon &&
                preparedAuthoredGrip.pose.weaponFormId == weapon->formID && preparedAuthoredGrip.pose.isLeft == _isLeft &&
                preparedAuthoredGrip.pose.role == role;
        };
        const bool authoredArrivalRequested = authoredWeapon && !handPocketOnlyGrab &&
            authoredPolicy::useAuthoredArrival(acquisitionSource, true, false, sel.pinchCloseSelectionFallback);
        const bool arrivalPoseAvailable = authoredArrivalRequested && prepareAuthoredGrip(true);
        bool authoredSeatPrepared = authoredPolicy::useAuthoredArrival(acquisitionSource, arrivalPoseAvailable,
            !authoredWeapon || handPocketOnlyGrab, sel.pinchCloseSelectionFallback);
        if (authoredArrivalRequested && !authoredSeatPrepared) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAuthoredArrivalDynamicFallback);
        }
        const auto* selectedBodyRecord = preparedBodySet.findRecord(objectBodyId.value);
        const auto& bodyScan = preparedBodySet.diagnostics;
        const bool authoredBodyOwnerVerified = authoredSeatPrepared &&
            preparedBodySet.rootRef == sel.refr && preparedBodySet.rootNode == rootNode &&
            sel.refr->Get3D() == rootNode && preparedBodySet.records.size() == 1 &&
            selectedBodyRecord && selectedBodyRecord->accepted && selectedBodyRecord->refResolutionKnown &&
            selectedBodyRecord->resolvedRef == sel.refr && nodeIsOrDescendsFrom(rootNode, selectedBodyRecord->owningNode) &&
            !activeLifecycle.hasIncompleteNativeScan() && !bodyScan.scanFailures && !bodyScan.invalidPhysicsSystems &&
            !bodyScan.depthLimitSkips && !bodyScan.foreignRefBodySkips && !bodyScan.unresolvedRefBodySkips;
        if (authoredBodyOwnerVerified) {
            performance_profiler::addCounter(performance_profiler::Counter::GrabAuthoredPositionsOnly);
        }
        const auto seedAuthoredPoint = [&](GrabSurfaceEvidence& surface) {
            const auto seatLocal = computeGrabLegacyPalmPivotAWorldFromHandBasis(preparedAuthoredGrip.pose.handWeaponLocal, _isLeft);
            surface.gripPoint = transform_math::localPointToWorld(rootNode->world, seatLocal);
            surface.pointMode = "authoredWeaponSeat";
            surface.pointAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
            surface.fallbackReason = "authoredArrival";
            // No mesh hit is fabricated: the authored pose is the position authority.
        };

        GrabMeshExtraction meshExtraction{};
        auto& grabMeshTriangles = meshExtraction.meshTriangles;
        auto& grabSurfaceTriangles = meshExtraction.surfaceTriangles;
        std::vector<GrabLocalTriangle> grabLocalMeshTriangles;
        std::vector<GrabLocalTriangle> grabFingerPoseLocalMeshTriangles;
        const bool meshContactOnly = g_rockConfig.rockGrabMeshContactOnly;
        extractGrabMeshEvidence(world, objectBodyId, rootNode, collidableNode, meshSourceNode, handPocketOnlyGrab, !authoredBodyOwnerVerified, meshExtraction);
        meshSourceNode = meshExtraction.meshSourceNode;
        if (grab_target::isRagdoll(sel.targetKind)) {
            const auto selectedComponent = ragdoll::readComponent(world, sel.bodyId.value);
            if (!selectedComponent.valid) {
                ROCK_LOG_WARN(Hand, "{} ragdoll surface rejected: selected body={} topology={}", handName(), sel.bodyId.value, selectedComponent.reason);
                grabPreparationTransaction.rollback();
                clearGrabExternalHandWorldTransform(_isLeft);
                return GrabAttemptResult::Rejected;
            }
            std::unordered_map<RE::NiAVObject*, RE::NiAVObject*> ownerCache;
            auto resolveOwner = [&](RE::NiAVObject* node) {
                if (auto it = ownerCache.find(node); it != ownerCache.end()) return it->second;
                const auto* record = preparedBodySet.findAcceptedRagdollOwner(node);
                auto* owner = record && selectedComponent.contains(record->bodyId) ? record->owningNode : nullptr;
                ownerCache.emplace(node, owner);
                return owner;
            };
            auto mapSurfaceOwners = [&] {
                for (auto& triangle : grabSurfaceTriangles) {
                    bool completeOwners = true;
                    for (auto& vertex : triangle.skinInfluences) for (auto& influence : vertex) {
                        influence.bone = resolveOwner(influence.bone);
                        if (influence.weight > 0.0f && !influence.bone) completeOwners = false;
                    }
                    if (triangle.sourceKind == GrabSurfaceSourceKind::Skinned) {
                        triangle.hasSkinInfluences = completeOwners && hasAnySkinInfluence(triangle.skinInfluences);
                        if (!triangle.hasSkinInfluences) triangle.sourceNode = nullptr;
                    } else triangle.sourceNode = resolveOwner(triangle.sourceNode);
                }
                std::erase_if(grabSurfaceTriangles, [](const auto& triangle) {
                    return !triangle.hasSkinInfluences && !triangle.sourceNode;
                });
            };
            mapSurfaceOwners();
            if (grabSurfaceTriangles.empty() && sel.targetKind == grab_target::Kind::DetachedGore) {
                // The actor's 3D is not necessarily the detached part's render root.
                // Choose the nearest bounded ancestor with surfaces owned by this
                // exact native component. Never accept the actor's unrelated mesh.
                auto* candidate = sel.hitNode;
                for (int level = 0; candidate && level < 8 && grabSurfaceTriangles.empty(); ++level, candidate = candidate->parent) {
                    if (candidate == meshSourceNode) continue;
                    grabMeshTriangles.clear();
                    const auto extraction = extractBoundedSurfaceTriangles(candidate, grabMeshTriangles, grabSurfaceTriangles,
                        (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth), 64, kMaxMeshExtractionTriangles, true);
                    if (extraction.nodeBudgetExceeded || extraction.shapeBudgetExceeded || extraction.triangleBudgetExceeded) {
                        grabSurfaceTriangles.clear();
                        ROCK_LOG_WARN(Hand, "{} detached part render search exceeded budget at '{}'", handName(), nodeDebugName(candidate));
                        break;
                    }
                    mapSurfaceOwners();
                    if (!grabSurfaceTriangles.empty()) {
                        meshSourceNode = meshExtraction.meshSourceNode = meshCaptureSetup.meshSourceNode = candidate;
                        ROCK_LOG_INFO(Hand, "{} detached part render root: '{}' body={} triangles={}",
                            handName(), nodeDebugName(candidate), sel.bodyId.value, grabSurfaceTriangles.size());
                    }
                }
            }
            grabMeshTriangles.clear();
            for (const auto& triangle : grabSurfaceTriangles) grabMeshTriangles.push_back(triangle.triangle);
        }
        GrabSurfaceEvidence surfaceEvidence{};
        if (authoredBodyOwnerVerified) {
            seedAuthoredPoint(surfaceEvidence);
        } else {
            meshExtraction.queryIndex.build(grabSurfaceTriangles);
            resolveGrabSurfaceEvidence(validatedSelection, proxyPreparation, meshCaptureSetup,
                meshExtraction, meshContactOnly, surfaceEvidence);
            if (authoredSeatPrepared && !surfaceEvidence.meshGrabFound) seedAuthoredPoint(surfaceEvidence);
        }
        auto& meshGrabFound = surfaceEvidence.meshGrabFound;
        auto& grabSurfaceHit = surfaceEvidence.surfaceHit;
        auto*& surfaceOwnerNode = surfaceEvidence.surfaceOwnerNode;
        const bool hasMeshSurfaceContact =
            meshGrabFound && grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
        GrabBodyResolution bodyResolution{};
        resolveGrabBodyAndContactPolicy(
            validatedSelection,
            beforePrepBodySet,
            preparedBodySet,
            activeLifecycle,
            proxyPreparation,
            surfaceEvidence,
            bodyResolution);
        const auto& primaryChoice = bodyResolution.primaryChoice;
        bool& surfaceOwnerMatchesResolvedBody = bodyResolution.surfaceOwnerMatchesResolvedBody;
        const auto& mechanicalScope = bodyResolution.mechanicalScope;
        const bool relaxedArticulatedAuthority = bodyResolution.relaxedArticulatedAuthority;

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            const auto* rejectedBody = diagnosticRejectedBodyRecord(preparedBodySet, sel.bodyId.value);
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no accepted dynamic body after recursive prep for '{}' formID={:08X} visitedNodes={} collisionObjects={} seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} rejectReason={} rejectBody={} rejectLayer={} rejectMotion={} rejectFlags=0x{:08X} rejectMotionProps={} latePrepared={} incompleteScan={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                preparedBodySet.diagnostics.visitedNodes,
                preparedBodySet.diagnostics.collisionObjects,
                preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                rejectedBody ? physics_body_classifier::rejectReasonName(rejectedBody->rejectReason) : "none",
                rejectedBody ? rejectedBody->bodyId : INVALID_BODY_ID,
                rejectedBody ? rejectedBody->collisionLayer : 0,
                rejectedBody ? bodyMotionTypeName(rejectedBody->motionType) : "none",
                rejectedBody ? rejectedBody->bodyFlags : 0,
                rejectedBody ? rejectedBody->motionPropertiesId : 0,
                activeLifecycle.latePreparedBodyCount(),
                activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand MECHANICAL SCOPE: targetKind={} kind={} reason={} primaryBody={} bodies={} accepted={} motions={} fixedRejects={} incomplete={} driveMode={} linearScope={} angularScope={} massScope={}",
            handName(),
            grab_target::name(sel.targetKind),
            mechanical_connected_body_set::scopeKindName(mechanicalScope.kind),
            mechanicalScope.reason,
            mechanicalScope.primaryBodyId,
            mechanicalScope.committedBodyIds.size(),
            mechanicalScope.acceptedBodyCount,
            mechanicalScope.uniqueMotionCount,
            mechanicalScope.rejectedFixedOrNonDynamicCount,
            mechanicalScope.incompleteDiscovery ? "yes" : "no",
            held_object_drive_policy::modeName(mechanicalScope.driveDecision.mode),
            mechanicalScope.driveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
            mechanicalScope.driveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
            mechanicalScope.driveDecision.includeConnectedMass ? "bodySet" : "primaryOnly");

        if (hasMeshSurfaceContact && !surfaceOwnerMatchesResolvedBody && !relaxedArticulatedAuthority) {
            // A mesh point on a sibling branch still lies on this object; hold it
            // there instead of refusing the grab.
            ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                "{} hand GRAB mesh owner differs from resolved body for '{}' formID={:08X}; selectedBody={} resolvedBody={} sourceNode='{}'",
                handName(), objName, sel.refr->GetFormID(), sel.bodyId.value, primaryChoice.bodyId, nodeDebugName(surfaceOwnerNode));
        }

        const bool visualMeshPivotAvailable = hasMeshSurfaceContact;
        const RE::NiPoint3 canonicalPivotNormalWorld = grabSurfaceHit.valid ? grabSurfaceHit.normal : RE::NiPoint3{};

        if (grab_target::isRagdoll(sel.targetKind)) {
            auto* owner = preparedBodySet.findRecord(primaryChoice.bodyId)->owningNode;
            std::erase_if(grabSurfaceTriangles, [owner](const auto& triangle) {
                if (!triangle.hasSkinInfluences) return triangle.sourceNode != owner;
                for (const auto& vertex : triangle.skinInfluences) for (const auto& influence : vertex) {
                    if (influence.bone == owner && influence.weight > 0.0f) return false;
                }
                return true;
            });
            grabMeshTriangles.clear();
            for (const auto& triangle : grabSurfaceTriangles) grabMeshTriangles.push_back(triangle.triangle);
            meshExtraction.queryIndex.build(grabSurfaceTriangles);
        }
        ResolvedGrabBodyCapture resolvedBodyCapture{};
        if (!captureResolvedGrabBody(
                world,
                meshCaptureSetup,
                bodyResolution,
                beforePrepBodySet,
                grabMeshTriangles,
                selectedRef,
                selectedOriginalMotionPropsId,
                objName,
                resolvedBodyCapture)) {
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptResult::Rejected;
        }
        objectBodyId = resolvedBodyCapture.bodyId;
        collidableNode = resolvedBodyCapture.collidableNode;
        objectWorldTransform = resolvedBodyCapture.objectWorldTransform;
        grabLocalMeshTriangles = std::move(resolvedBodyCapture.localMeshTriangles);
        if (grab_target::isRagdoll(sel.targetKind)) {
            // Skin weights may resolve to a different bone than the cast hit.
            // Later frame/constraint capture reads this setup, so it must use
            // the resolved owner's frame just like the local mesh cache above.
            meshCaptureSetup.collidableNode = collidableNode;
            meshCaptureSetup.objectWorldTransform = objectWorldTransform;
            ROCK_LOG_SAMPLE_DEBUG(Hand, 2000,
                "{} ragdoll resolved frame: selectedBody={} resolvedBody={} selectedNode='{}' captureNode='{}' surfaceNode='{}'",
                handName(), sel.bodyId.value, objectBodyId.value, nodeDebugName(sel.hitNode),
                nodeDebugName(collidableNode), nodeDebugName(surfaceEvidence.surfaceOwnerNode));
        }

        if (authoredWeapon) {
            const char* preparationReason = authoredSeatPrepared ? "authored-arrival-ready" :
                (authoredArrivalRequested ? "authored-unavailable-dynamic" : "surface-evidence-required");
            if (!authoredSeatPrepared && !authoredArrivalRequested && !handPocketOnlyGrab &&
                !sel.pinchCloseSelectionFallback && sel.refr->Get3D() == rootNode &&
                preparedBodySet.acceptedCount() == 1 && !activeLifecycle.hasIncompleteNativeScan() &&
                surfaceOwnerMatchesResolvedBody && visualMeshPivotAvailable) {
                authoredSeatPrepared = prepareAuthoredGrip(false);
                if (authoredSeatPrepared) preparationReason = "authored-close-seat-ready";
            }
            performance_profiler::addCounter(authoredSeatPrepared ?
                performance_profiler::Counter::GrabAuthoredSurfaceWorkSkipped :
                performance_profiler::Counter::GrabAuthoredSurfaceWorkRequired);
            if (performance_profiler::enabled()) {
                ROCK_LOG_SAMPLE_INFO(Hand, 1000,
                    "{} authored pickup preparation: ref={:08X} body={} source={} role={} skipSurfaceWork={} positionsOnly={} reason={} triangles={} surfaceRecords={}",
                    handName(), sel.refr ? sel.refr->GetFormID() : 0, objectBodyId.value,
                    static_cast<unsigned>(acquisitionSource), static_cast<unsigned>(preparedAuthoredGrip.role),
                    authoredSeatPrepared, authoredBodyOwnerVerified, preparationReason,
                    grabMeshTriangles.size(), grabSurfaceTriangles.size());
            }
        }

        GrabPivotEvidence pivotEvidence{};
        resolveGrabPivotEvidence(proxyPreparation, resolvedBodyCapture, surfaceEvidence, pivotEvidence);

        const RuntimePinchPocketCandidate pinchPocketCandidate = authoredSeatPrepared ?
            RuntimePinchPocketCandidate{} : buildRuntimePinchPocketCandidate(
            sel,
            preparedBodySet,
            objectBodyId.value,
            objectWorldTransform,
            grabSurfaceTriangles,
            meshExtraction.queryIndex,
            grabLocalMeshTriangles,
            handWorldTransform,
            _isLeft,
            *this,
            !sel.isFarSelection && !grabbedFromPullCatch,
            handPocketOnlyGrab,
            looseWeaponGrab);
        if (pinchPocketCandidate.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PINCH POCKET candidate accepted: reason={} pocket=({:.1f},{:.1f},{:.1f}) point=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) gap={:.2f}gu dist={:.2f}gu extents=({:.2f},{:.2f},{:.2f}) boundsVolume={:.2f}/{:.2f}gu^3",
                handName(),
                pinchPocketCandidate.decision.reason,
                pinchPocketCandidate.pinchPocketWorld.x,
                pinchPocketCandidate.pinchPocketWorld.y,
                pinchPocketCandidate.pinchPocketWorld.z,
                pinchPocketCandidate.surfaceHit.position.x,
                pinchPocketCandidate.surfaceHit.position.y,
                pinchPocketCandidate.surfaceHit.position.z,
                pinchPocketCandidate.pinchDetectionDirectionWorld.x,
                pinchPocketCandidate.pinchDetectionDirectionWorld.y,
                pinchPocketCandidate.pinchDetectionDirectionWorld.z,
                pinchPocketCandidate.thumbIndexGapGameUnits,
                pinchPocketCandidate.pocketToSurfaceDistanceGameUnits,
                pinchPocketCandidate.meshExtents.minExtentGameUnits,
                pinchPocketCandidate.meshExtents.middleExtentGameUnits,
                pinchPocketCandidate.meshExtents.maxExtentGameUnits,
                pinchPocketCandidate.meshExtents.boundsVolumeCubicGameUnits,
                g_rockConfig.rockGrabPinchMaxVolumeCubicGameUnits);
        }

        const GrabCommitPreparationInput commitPreparationInput{
            .handWorldTransform = &handWorldTransform,
            .collidableNode = collidableNode,
            .objectBodyId = objectBodyId,
            .objectName = &objName,
            .mechanicalScope = &mechanicalScope,
            .sharedContext = &sharedContext,
            .preparedBodySet = &preparedBodySet,
            .joiningPeerHeldObject = joiningPeerHeldObject,
        };
        beginResolvedGrabCommit(commitPreparationInput);

        {
            GrabBodyFrameCaptureInput bodyFrameInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .surface = &surfaceEvidence,
                .pivotEvidence = &pivotEvidence,
                .bodyResolution = &bodyResolution,
                .localMeshTriangles = &grabLocalMeshTriangles,
                .rootNode = rootNode,
                .objectBodyId = objectBodyId,
                .traceId = grabTraceId,
                .objectName = &objName,
                .authoredSeatPrepared = authoredSeatPrepared,
            };
            GrabBodyFrameCapture bodyFrameCapture{};
            if (!captureGrabBodyFrame(world, bodyFrameInput, bodyFrameCapture)) {
                grabPreparationTransaction.rollback();
                if (bodyFrameCapture.clearExternalOnFailure) {
                    clearGrabExternalHandWorldTransform(_isLeft);
                }
                return GrabAttemptResult::Rejected;
            }
            grabFingerPoseLocalMeshTriangles = std::move(bodyFrameCapture.fingerPoseLocalMeshTriangles);

            const GrabRollbackAction seatRollback{
                .context = &grabPreparationTransaction,
                .invoke = [](void* context) noexcept {
                    static_cast<decltype(grabPreparationTransaction)*>(context)->rollback();
                },
            };
            GrabSeatCaptureInput seatCaptureInput{
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .surface = &surfaceEvidence,
                .pivotEvidence = &pivotEvidence,
                .bodyFrame = &bodyFrameCapture,
                .pinchPocketCandidate = &pinchPocketCandidate,
                .savedGrabOffset = &savedGrabOffset,
                .hasSavedGrabOffset = &hasSavedGrabOffset,
                .rootNode = rootNode,
                .collidableNode = collidableNode,
                .objectBodyId = objectBodyId,
                .grabbedFromPullCatch = grabbedFromPullCatch,
                .looseWeaponGrab = looseWeaponGrab,
                .joiningPeerHeldObject = joiningPeerHeldObject,
                .transferPose = transferPose,
                .preparedAuthoredGrip = authoredSeatPrepared ? &preparedAuthoredGrip : nullptr,
                .canonicalPivotNormalWorld = canonicalPivotNormalWorld,
                .rollback = seatRollback,
            };
            GrabSeatCaptureResult seatCapture{};
            if (!resolveGrabSeatCapture(world, seatCaptureInput, seatCapture)) {
                return GrabAttemptResult::Rejected;
            }

            GrabFrozenCommitInput frozenCommitInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .bodyFrame = &bodyFrameCapture,
                .seatCapture = &seatCapture,
                .surface = &surfaceEvidence,
                .objectBodyId = objectBodyId,
                .traceId = grabTraceId,
                .rollback = seatRollback,
            };
            if (!commitFrozenGrabAuthority(frozenCommitInput)) {
                return GrabAttemptResult::Rejected;
            }

            const GrabPostFreezeInput postFreezeInput{
                .handWorldTransform = &handWorldTransform,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .bodyFrame = &bodyFrameCapture,
                .surface = &surfaceEvidence,
                .objectBodyId = objectBodyId,
                .bodyBoneColliders = bodyBoneColliders,
                .sharedContext = &sharedContext,
                .joiningPeerHeldObject = joiningPeerHeldObject,
                .looseWeaponGrab = looseWeaponGrab,
            };
            initializePostFreezeGrab(world, postFreezeInput);
        }

        const GrabRollbackAction constraintRollback{
            .context = &grabPreparationTransaction,
            .invoke = [](void* context) noexcept {
                static_cast<decltype(grabPreparationTransaction)*>(context)->rollback();
            },
        };
        const GrabConstraintCommitInput constraintCommitInput{
            .handWorldTransform = &handWorldTransform,
            .proxy = &proxyPreparation,
            .meshCapture = &meshCaptureSetup,
            .surface = &surfaceEvidence,
            .bhkWorld = bhkWorld,
            .objectBodyId = objectBodyId,
            .bodyBoneColliders = bodyBoneColliders,
            .activeLifecycle = &activeLifecycle,
            .savedGrabOffset = &savedGrabOffset,
            .hasSavedGrabOffset = hasSavedGrabOffset,
            .joiningPeerHeldObject = joiningPeerHeldObject,
            .looseWeaponGrab = looseWeaponGrab,
            .tau = tau,
            .damping = damping,
            .maxForce = maxForce,
            .proportionalRecovery = proportionalRecovery,
            .constantRecovery = constantRecovery,
            .rollback = constraintRollback,
        };
        if (!commitGrabConstraintAndPose(world, constraintCommitInput)) {
            return GrabAttemptResult::Rejected;
        }

        grabPreparationTransaction.commit();
        performance_profiler::addCounter(performance_profiler::Counter::GrabAcquisitionSucceeded);
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::GrabCommitSucceeded });
        clearPullRuntimeState();
        clearPullCatchIntent(grabbedFromPullCatch ? "pullCatchGrabbed" : "grabbed");

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), objectBodyId.value);
        _grabFrame.equippedWeaponTransfer = sel.equippedWeaponTransfer;
        if (sel.equippedWeaponTransfer && rootNode) {
            // Seed presentation from the constraint's initial physical target.
            // The same-frame held update can then carry the measured body pose
            // onto the queued hand target instead of discarding its first sample.
            const auto& initialTarget = _grabOffsetAcquisition.active ?
                _grabFrame.authority.bodyWorldAtGrab : _grabFrame.authority.desiredBodyWorldAtGrab;
            (void)held_scene_presentation::publishTargetTransport(_isLeft, world, objectBodyId.value,
                _grabFrame.traceId, initialTarget, _grabFrame.authority.bodyWorldAtGrab,
                rootNode, _grabFrame.rootBodyLocal);
        }
        return GrabAttemptResult::Grabbed;
    }

    Hand::HeldHandMotionSample Hand::recordHeldControllerMotionSample(
        const RE::NiTransform& handWorldTransform,
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        HeldHandMotionSample handMotion{};
        const auto admission = release_velocity::admitControllerSample(_releaseControllerFrame, timing);
        if (admission.duplicate) return handMotion;
        _releaseControllerFrame = timing.sequence;
        if (admission.rebase) {
            if (_hasPreviousHeldRawHandWorld && timing.discontinuity && timing.valid) {
                ROCK_LOG_SAMPLE_WARN(Hand, 1000, "{} throw history rebased after timing discontinuity rawDt={:.4f}",
                    handName(), timing.rawDeltaSeconds);
            }
            _controllerReleaseHistory.clear();
            _objectReleaseHistory.clear();
            _releaseObjectNotBefore = release_velocity::sampleTimeSeconds();
            _hasPreviousHeldRawHandWorld = false;
        }
        const float deltaTime = admission.deltaSeconds;
        const bool usableDeltaTime = std::isfinite(deltaTime) && deltaTime > 0.000001f;
        const RE::NiPoint3 currentHandPositionHavok = gamePointToHavokPoint(handWorldTransform.translate);
        _lastHeldHandPositionHavok = currentHandPositionHavok;
        _hasLastHeldHandPositionHavok = true;

        if (_hasPreviousHeldRawHandWorld && usableDeltaTime) {
            handMotion.localLinearVelocityHavok = scalePoint(currentHandPositionHavok - _previousHeldHandPositionHavok, 1.0f / deltaTime);
            handMotion.hasLocalLinearVelocity = true;

            handMotion.angularVelocityRadiansPerSecond =
                angularVelocityFromRotationDelta(_previousHeldRawHandWorld.rotate, handWorldTransform.rotate, deltaTime);
            handMotion.hasAngularVelocity = lengthSquared(handMotion.angularVelocityRadiansPerSecond) > 0.000001f;

            _controllerReleaseHistory.append(timing.sequence, timing.elapsedGameSeconds,
                handMotion.localLinearVelocityHavok, handMotion.angularVelocityRadiansPerSecond);
        }

        _previousHeldRawHandWorld = handWorldTransform;
        _previousHeldHandPositionHavok = currentHandPositionHavok;
        _hasPreviousHeldRawHandWorld = true;
        return handMotion;
    }

    void Hand::recordHeldObjectVelocitySample(RE::hknpWorld* world)
    {
        const auto& timing = runtime_state::currentFrame().timing;
        if (!timing.valid || timing.discontinuity || timing.menuPaused) return;
        ReleaseObjectSample sample{};
        {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            sample = _releaseObjectSample;
        }
        if (sample.world != world || sample.bodyId != _savedObjectState.bodyId.value ||
            sample.grabTrace != _grabFrame.traceId || !_objectReleaseHistory.needsSource(sample.solve) ||
            !release_velocity::usablePhysicsSample(sample.solve, sample.capturedAt,
                _releaseObjectNotBefore, release_velocity::sampleTimeSeconds())) return;
        _objectReleaseHistory.append(sample.solve, sample.capturedAt, sample.velocity);
    }

    void Hand::captureHeldReleaseMotion(
        RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        const game_frame_timing_policy::GameFrameTiming& timing)
    {
        if (!isHolding() || !world) {
            return;
        }

        recordHeldControllerMotionSample(handWorldTransform, timing);
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

    void Hand::publishHeldBodyScope(RE::hknpWorld* world)
    {
        _heldBodyIdsSequence.fetch_add(1, std::memory_order_acq_rel);
        {
            int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
            for (int i = 0; i < count; i++) {
                _heldBodyIdsSnapshot[i].store(_heldBodyIds[i], std::memory_order_release);
            }
            _heldBodyIdsCount.store(count, std::memory_order_release);

        }

        _heldBodyIdsSequence.fetch_add(1, std::memory_order_release);

        held_scene_presentation::Registration sceneRegistration{};
        sceneRegistration.traceId = _grabFrame.traceId;
        for (const std::uint32_t heldBodyId : _heldBodyIds) {
            if (sceneRegistration.count >=
                held_scene_presentation::kMaxRegisteredBodies) {
                sceneRegistration.complete = false;
                break;
            }

            auto* collisionObject =
                havok_runtime::getCollisionObjectFromBody(
                    world,
                    RE::hknpBodyId{ heldBodyId });
            if (!collisionObject) {
                sceneRegistration.complete = false;
                continue;
            }

            sceneRegistration.bodies[sceneRegistration.count++] =
                held_scene_presentation::RegisteredBody{
                    .collisionObject = collisionObject,
                    .world = world,
                    .bodyId = heldBodyId,
                };
        }
        held_scene_presentation::publishHeldBodies(
            _isLeft,
            sceneRegistration);
        if (sceneRegistration.count == 0) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB could not publish held-body scene presentation identity: trace={} heldBodies={}",
                handName(),
                _grabFrame.traceId,
                _heldBodyIds.size());
        }

    }

    bool Hand::refreshRagdollBodyScope(RE::hknpWorld* world, const GrabReleaseContext& releaseContext)
    {
        if (!grab_target::isRagdoll(_savedObjectState.targetKind)) return true;
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::RagdollBodyRefresh);
        const auto component = ragdoll::readComponent(world, _savedObjectState.bodyId.value);
        const auto* liveOwner = component.valid ? havok_runtime::getCollisionObjectFromBody(world, _savedObjectState.bodyId) : nullptr;
        bool sameNode = false;
        for (std::size_t i = 0; i < component.count; ++i) {
            if (component.bodies[i].body.id == _savedObjectState.bodyId.value) sameNode = component.bodies[i].node == _grabFrame.heldNode;
        }
        segment_visibility::VisibleRanges visible{};
        std::uint32_t triangles = 0;
        const bool surfaceVisible = _ragdollGrabSurface && (_ragdollGrabSurface->flags.flags & 1) == 0 &&
            native_memory::tryReadField(_ragdollGrabSurface.get(), VROffset::numTriangles, triangles) &&
            readVisibleTriangleRanges(_ragdollGrabSurface.get(), triangles, visible) && visible.contains(_ragdollGrabTriangle);
        if (!component.valid || !sameNode || !surfaceVisible || liveOwner != _ragdollGrabOwner.get() || component.count > MAX_HELD_BODIES) {
            ROCK_LOG_WARN(Hand, "{} ragdoll hold ended: body={} stage={} ownerMatch={} surfaceVisible={}", handName(),
                _savedObjectState.bodyId.value, component.reason, liveOwner == _ragdollGrabOwner.get(), surfaceVisible);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        bool changed = component.count != _heldBodyIds.size();
        for (const auto id : _heldBodyIds) changed = changed || !component.contains(id);
        _heldDriveDecision.includeConnectedMass = !component.fixedAttached;
        if (!changed) return true;

        // Native topology changes are rare. Allocation and lease reconciliation
        // occur only on this transition; unchanged frames read bounded arrays.
        std::vector<std::uint32_t> removed, added;
        for (const auto id : _heldBodyIds) if (!component.contains(id)) removed.push_back(id);
        for (std::size_t i = 0; i < component.count; ++i) {
            const auto id = component.bodies[i].body.id;
            if (!held_object_body_set_policy::containsBody(_heldBodyIds, id)) added.push_back(id);
        }
        const auto released = releaseHeldObjectBodyFlagLeases(world, INVALID_BODY_ID, removed, _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this), true);
        const auto acquired = acquireHeldObjectBodyFlagLeases(world, INVALID_BODY_ID, added, _savedObjectState.targetKind, heldBodyFlagLeaseOwner(this));
        // Include newly acquired leases even on failure so normal release owns all cleanup.
        std::erase_if(_heldBodyIds, [&](auto id) { return !component.contains(id); });
        _heldBodyIds.insert(_heldBodyIds.end(), added.begin(), added.end());
        publishHeldBodyScope(world);
        clearHeldBodyContactSnapshot();
        ROCK_LOG_INFO(Hand, "{} ragdoll component changed: primary={} bodies={} removed={} added={} leaseFailures={}",
            handName(), _savedObjectState.bodyId.value, _heldBodyIds.size(), removed.size(), added.size(),
            released.failedLeaseCount + acquired.failedLeaseCount);
        if (acquired.failedLeaseCount) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        return true;
    }

    bool Hand::validateHeldObjectUpdate(RE::hknpWorld* world, const GrabReleaseContext& releaseContext)
    {
        if (!refreshRagdollBodyScope(world, releaseContext)) return false;
        if (_grabAuthorityProxyReleasePending.load(std::memory_order_acquire) || !_activeConstraint.isValid() || !_grabAuthorityProxy.isValid()) {
            ROCK_LOG_WARN(Hand, "{} hand release: proxy constraint authority marked grab invalid", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            ROCK_LOG_WARN(Hand,
                "{} hand held reference invalid: grab={} ref={:08X} missing={} deleted={} disabled={}",
                handName(), _grabFrame.traceId, _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
                !_savedObjectState.refr, _savedObjectState.refr && _savedObjectState.refr->IsDeleted(),
                _savedObjectState.refr && _savedObjectState.refr->IsDisabled());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        return true;
    }

    std::uint64_t Hand::runtimeFrameForObjectTarget() const {return runtime_state::currentFrame().frameIndex;}
    bool Hand::readHeldObjectRoot(RE::hknpWorld* world,RE::NiTransform& root) const
    {
        RE::NiTransform body;
        if(!isHolding()||!_savedObjectState.isValid()||
            !tryGetGrabDriveObjectWorldTransform(world,_savedObjectState.bodyId,body))return false;
        root=multiplyTransforms(body,invertTransform(_grabFrame.rootBodyLocal));
        return isFiniteNiTransform(root);
    }

    api::Status Hand::setHeldObjectTarget(api::OwnerToken owner,const api::grab::v1_3::HeldObjectTarget& request,std::span<const std::uint32_t> weaponBodies)
    {
        if(!isHolding()||!getHeldRef()||getHeldRef()->GetFormID()!=request.referenceFormId)return api::Status::HandUnavailable;
        if(_savedObjectState.targetKind!=grab_target::Kind::LooseObject||_heldObjectIsLooseWeapon||
            _grabFrame.looseObjectSharedPeerTrace||!_activeConstraint.isValid())return api::Status::TargetUnavailable;
        auto* root=getHeldRef()->Get3D();
        if(!root||!std::isfinite(request.rootWorld.scale)||std::abs(request.rootWorld.scale-root->world.scale)>.001f)return api::Status::InvalidArgument;
        for(const auto value:request.rootWorld.translate)if(!std::isfinite(value))return api::Status::InvalidArgument;
        for(int row=0;row<3;++row)for(int other=0;other<3;++other) {
            float dot=0;for(int c=0;c<3;++c)dot+=request.rootWorld.rotate[row*3+c]*request.rootWorld.rotate[other*3+c];
            if(!std::isfinite(dot)||std::abs(dot-(row==other?1.f:0.f))>.001f)return api::Status::InvalidArgument;
        }
        const auto* r=request.rootWorld.rotate;
        const float determinant=r[0]*(r[4]*r[8]-r[5]*r[7])-r[1]*(r[3]*r[8]-r[5]*r[6])+r[2]*(r[3]*r[7]-r[4]*r[6]);
        if(determinant<.999f)return api::Status::InvalidArgument;
        if(_heldBodyIds.size()*weaponBodies.size()>_consumerPairs.size())return api::Status::CapacityFull;
        const auto status=_consumerObjectTarget.set(owner,heldGrabIdentity(),runtimeFrameForObjectTarget(),request);
        if(status!=api::Status::Ok)return status;
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        _consumerPairCount=0;_consumerPairsExpire=_consumerObjectTarget.expires;
        for(const auto held:_heldBodyIds)for(const auto weapon:weaponBodies)_consumerPairs[_consumerPairCount++]={held,weapon,0};
        return api::Status::Ok;
    }

    api::Status Hand::clearHeldObjectTarget(api::OwnerToken owner)
    {
        const auto result=_consumerObjectTarget.clear(owner,heldGrabIdentity(),runtimeFrameForObjectTarget());
        if(result==api::Status::Ok){std::scoped_lock lock(_grabAuthorityProxyMutex);_consumerPairCount=0;}
        return result;
    }
    void Hand::flushConsumerObjectPairs(RE::hknpWorld* world)
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if(world!=_grabAuthorityProxyHknpWorld||provider::runtime::currentGameFrameIndex()>_consumerPairsExpire)_consumerPairCount=0;
        const auto result=_consumerPairLeases.reconcile(world,_consumerPairs.data(),_consumerPairCount);
        if(_consumerPairCount&&(!result.filterAvailable||result.activePairCount!=_consumerPairCount)) {
            _grabAuthorityProxyReleasePending.store(true,std::memory_order_release);
            ROCK_LOG_SAMPLE_WARN(Hand,1000,"Held object connection pairs unavailable; releasing guided object");
        }
    }

    struct Hand::HeldDriveUpdate
    {
        RE::NiTransform proxyAuthorityWorld{};
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform solvedBodyWorld{};
        /*
         * BODY-frame pose the scene writer presents for the held node this
         * frame (current target composed with the previous solved residual).
         * Valid only when the target transport accepted this frame's sample.
         */
        RE::NiTransform presentedBodyWorld{};
        RE::NiPoint3 activePivotBBodyLocalGame{};
        RE::NiPoint3 desiredTargetPointWorld{};
        RE::NiPoint3 liveGripWorldForAuthority{};
        const char* proxyAuthoritySource = "notProxy";
        const char* heldMotorContactReason = "no-recent-contact";
        float pivotTrackingErrorGameUnits = 0.0f;
        float grabRotationErrorDegrees = 0.0f;
        float authorityForceScale = 1.0f;
        float averageGrabDeviationGameUnits = 0.0f;
        bool hasProxyAuthorityFrame = false;
        bool hasPivotTrackingError = false;
        bool hasPresentedBodyWorld = false;
        bool heldBodyColliding = false;
        bool heldMotorContactSoftening = false;
    };

    bool Hand::coordinateLooseObjectProxy(RE::hknpWorld* world, Hand* peer,
        const RE::NiTransform* peerHandWorld, RE::NiTransform& proxyWorld)
    {
        const bool sameObject = isHolding() && peer && peerHandWorld && peer->isHolding() &&
            getHeldRef() && getHeldRef() == peer->getHeldRef();
        const bool sameBody = sameObject && _savedObjectState.bodyId.value == peer->_savedObjectState.bodyId.value;
        // Detached pieces can retain the same actor ref without sharing a
        // rigid root. Only two grips on the same ragdoll body may share a pivot.
        const bool sameAssembly = sameObject && (sameBody ||
            (!grab_target::isRagdoll(_savedObjectState.targetKind) && !grab_target::isRagdoll(peer->_savedObjectState.targetKind)));
        const bool shared = sameAssembly && held_object_drive_policy::canShareTwoHandPivot(
            _heldDriveDecision.mode, peer->_heldDriveDecision.mode, sameBody,
            isHoldingLooseWeapon() && peer->isHoldingLooseWeapon());
        const auto finishOldPair = [](Hand& hand, std::uint64_t peerTrace) {
            auto& frame = hand._grabFrame;
            if (frame.looseObjectSharedPeerTrace && frame.looseObjectSharedPeerTrace != peerTrace) {
                // A remaining firing grip returns to its solo aim. A support
                // carry or prop retains its acquired orientation.
                const bool preserveSharedPose = !hand.isHoldingFiringGrip();
                frame.looseObjectSoloProxyCorrection = preserveSharedPose ? frame.looseObjectSharedProxyCorrection :
                    transform_math::makeIdentityTransform<RE::NiTransform>();
                frame.hasLooseObjectSoloProxyCorrection = preserveSharedPose;
                frame.looseObjectSharedPeerTrace = 0;
                ROCK_LOG_INFO(Hand, "{} hand loose two-hand pivot ended; {}", hand.handName(),
                    preserveSharedPose ? "preserving remaining grip" : "restoring solo weapon grip");
            }
        };
        const auto correctedProxy = [](const Hand& hand, RE::NiTransform proxy) {
            if (hand._grabFrame.hasLooseObjectSoloProxyCorrection) {
                proxy.rotate = transform_math::transposeRotation(proxy.rotate);
                proxy = transform_math::composeTransforms(proxy, hand._grabFrame.looseObjectSoloProxyCorrection);
                proxy.rotate = transform_math::transposeRotation(proxy.rotate);
            }
            return proxy;
        };
        const auto readPresentedBody = [world](const Hand& hand, RE::NiTransform& body) {
            if (hand._grabFrame.heldNode && isFiniteNiTransform(hand._grabFrame.heldNode->world)) {
                body = transform_math::composeTransforms(hand._grabFrame.heldNode->world, hand._grabFrame.authority.bodyLocal);
                return isFiniteNiTransform(body);
            }
            return hand.tryGetGrabDriveObjectWorldTransform(world, hand._savedObjectState.bodyId, body) && isFiniteNiTransform(body);
        };
        const bool leavingPair = _grabFrame.looseObjectSharedPeerTrace != 0 && !shared;
        finishOldPair(*this, shared ? peer->_grabFrame.traceId : 0);
        if (!shared) {
            _looseSharedAcquisition = {};
            _looseSharedAcquisitionPeer = 0;
            _looseSharedAcquisitionFrame = 0;
            _looseSharedHandBlendAlpha = -1.0f;
            proxyWorld = correctedProxy(*this, proxyWorld);
            if (leavingPair && isHoldingLooseWeapon()) {
                RE::NiTransform presentedBody{};
                if (!readPresentedBody(*this, presentedBody)) return false;
                if (!isHoldingFiringGrip()) {
                    // When the firing hand leaves, keep the support carry as
                    // displayed. Reattaching a firing grip will acquire it anew.
                    const auto carriedProxy = grab_frame_math::generatedProxyFromObjectWorld(presentedBody,
                        _grabFrame.proxyAuthorityBodyHandSpace);
                    auto physical = proxyWorld;
                    physical.rotate = transform_math::transposeRotation(physical.rotate);
                    if (_grabFrame.hasLooseObjectSoloProxyCorrection)
                        physical = transform_math::composeTransforms(physical,
                            transform_math::invertTransform(_grabFrame.looseObjectSoloProxyCorrection));
                    auto carried = carriedProxy;
                    carried.rotate = transform_math::transposeRotation(carried.rotate);
                    _grabFrame.looseObjectSoloProxyCorrection = transform_math::composeTransforms(
                        transform_math::invertTransform(physical), carried);
                    _grabFrame.hasLooseObjectSoloProxyCorrection = true;
                    proxyWorld = carriedProxy;
                }
                _grabOffsetAcquisition = grab_offset_acquisition::beginWeapon(proxyWorld, presentedBody,
                    _grabFrame.proxyAuthorityBodyHandSpace, _grabFrame.authority.pivotBConstraintLocalGame,
                    hand_visual_lerp_math::kEquippedWeaponReturnConfig);
                _looseWeaponReturnStartFrame = runtime_state::currentFrame().timing.sequence;
            }
            return isFiniteNiTransform(proxyWorld);
        }
        finishOldPair(*peer, _grabFrame.traceId);
        RE::NiTransform peerProxy{};
        const char* peerSource = "unavailable";
        if (!peer->resolveGrabAuthorityProxyFrame(world, *peerHandWorld, nullptr, peerProxy,
                peerSource, GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget)) {
            ROCK_LOG_WARN(Hand, "Loose two-hand pivot missing peer palm source={}", peerSource);
            return false;
        }
        // A firing seat owns the pivot. For two arbitrary mesh seats, keep
        // the first grab as the pivot until that hand releases.
        const auto firingSeat = [](const Hand& hand) {
            return hand._grabFrame.syntheticLooseWeaponPrimaryAttach && !hand._grabFrame.authoredLooseWeaponSupportGrip;
        };
        const bool thisPrimary = firingSeat(*this) != firingSeat(*peer) ? firingSeat(*this) :
            _grabFrame.traceId < peer->_grabFrame.traceId;
        Hand& primary = thisPrimary ? *this : *peer;
        Hand& support = thisPrimary ? *peer : *this;
        const auto primaryProxy = correctedProxy(primary, thisPrimary ? proxyWorld : peerProxy);
        const auto supportProxy = correctedProxy(support, thisPrimary ? peerProxy : proxyWorld);
        // One driven body is its own common frame, including two seats on a
        // jointed object's same part. Different rigid bodies use the assembly
        // root; independent articulated parts keep their existing joint motion.
        const auto primaryBodyLocal = sameBody ? transform_math::makeIdentityTransform<RE::NiTransform>() : primary._grabFrame.rootBodyLocal;
        const auto supportBodyLocal = sameBody ? transform_math::makeIdentityTransform<RE::NiTransform>() : support._grabFrame.rootBodyLocal;
        const auto primaryBody = grab_frame_math::objectFromGeneratedProxyLocalSpace(
            primaryProxy, primary._grabFrame.proxyAuthorityBodyHandSpace);
        const auto root = transform_math::composeTransforms(primaryBody,
            transform_math::invertTransform(primaryBodyLocal));
        const auto primaryLocal = transform_math::localPointToWorld(primaryBodyLocal,
            primary.activeProxyConstraintPivotBLocalGame());
        const auto supportLocal = transform_math::localPointToWorld(supportBodyLocal,
            support.activeProxyConstraintPivotBLocalGame());
        const auto primaryTarget = generatedProxyLocalPointToWorld(primaryProxy,
            primary._grabFrame.authority.pivotAHandBodyLocalGame);
        const auto supportTarget = generatedProxyLocalPointToWorld(supportProxy,
            support._grabFrame.authority.pivotAHandBodyLocalGame);
        const auto supportBase = transform_math::localPointToWorld(root, supportLocal);
        const float separation = pointDistanceGameUnits(transform_math::localPointToWorld(root, primaryLocal), supportBase);
        const auto lockedSupport = makeLockedSupportGripTarget(primaryTarget, supportTarget, supportBase, separation, 0.001f);
        const bool sharedFiringStation = loose_weapon_authored_grab_policy::sharedFiringZone(primary._grabFrame.authoredWeaponArrangement) ||
            loose_weapon_authored_grab_policy::sharedFiringZone(support._grabFrame.authoredWeaponArrangement);
        const auto solved = sharedFiringStation ? WeaponTwoHandedSolverResult<RE::NiTransform>{} :
            solveTwoHandedWeaponTransformFrikPivot(WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3>{
            .weaponWorldTransform = root,
            .primaryGripLocal = primaryLocal,
            .supportGripLocal = supportLocal,
            .primaryTargetWorld = primaryTarget,
            .supportTargetWorld = lockedSupport,
        });
        // Coincident seats cannot define an aim axis. Both constraints then
        // share the primary carry target until separation defines that axis.
        if (!sharedFiringStation && !solved.solved) {
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Loose two-hand pivot degenerate axis: separation={:.4f}; sharing primary target", separation);
        }
        auto sharedRoot = solved.solved ? solved.weaponWorldTransform : root;
        if (isHoldingLooseWeapon() && peer->isHoldingLooseWeapon()) {
            const auto targetBody = transform_math::composeTransforms(sharedRoot, primaryBodyLocal);
            const auto targetInProxy = grab_frame_math::objectInGeneratedProxyLocalSpace(primaryProxy, targetBody);
            const auto pivot = primary.activeProxyConstraintPivotBLocalGame();
            auto& acquisition = primary._looseSharedAcquisition;
            const auto sequence = runtime_state::currentFrame().timing.sequence;
            if (primary._looseSharedAcquisitionPeer != support._grabFrame.traceId) {
                RE::NiTransform liveBody{};
                if (!readPresentedBody(primary, liveBody)) return false;
                acquisition = grab_offset_acquisition::begin(primaryProxy, liveBody, targetInProxy, pivot);
                if (acquisition.active) {
                    const auto liveRoot = transform_math::composeTransforms(liveBody, transform_math::invertTransform(primaryBodyLocal));
                    const float distance = (std::max)(
                        pointDistanceGameUnits(transform_math::localPointToWorld(liveRoot, primaryLocal), primaryTarget),
                        pointDistanceGameUnits(transform_math::localPointToWorld(liveRoot, supportLocal),
                            transform_math::localPointToWorld(sharedRoot, supportLocal)));
                    const float linearDuration = hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(distance,
                        g_rockConfig.rockWeaponSupportGripHandLerpTimeMin, g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                        g_rockConfig.rockWeaponSupportGripHandLerpMinDistance, g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
                    const float angularDuration = hand_visual_lerp_math::computeAngleMappedDurationDegrees(
                        hand_visual_lerp_math::rotationDistanceDegrees(liveRoot, sharedRoot),
                        g_rockConfig.rockWeaponSupportGripHandLerpTimeMin, g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                        hand_visual_lerp_math::kEquippedWeaponReturnConfig.minAngleDegrees,
                        hand_visual_lerp_math::kEquippedWeaponReturnConfig.maxAngleDegrees);
                    acquisition.durationSeconds = g_rockConfig.rockWeaponSupportGripHandLerpEnabled ?
                        (std::max)(linearDuration, angularDuration) : 0.0f;
                }
                primary._looseSharedAcquisitionPeer = support._grabFrame.traceId;
                primary._looseSharedAcquisitionFrame = sequence;
            }
            const auto blendedProxy = grab_offset_acquisition::advanceShared(acquisition,
                primary._looseSharedAcquisitionFrame, sequence, primaryProxy, targetInProxy, pivot,
                runtime_state::currentFrame().deltaSeconds);
            primary._looseSharedHandBlendAlpha = -1.0f;
            support._looseSharedHandBlendAlpha = -1.0f;
            const float alpha = hand_visual_lerp_math::timedBlendAlpha(acquisition.elapsedSeconds, acquisition.durationSeconds);
            Hand& joining = primary._grabFrame.traceId > support._grabFrame.traceId ? primary : support;
            joining._looseSharedHandBlendAlpha = !sharedFiringStation && acquisition.durationSeconds > 0.0f ?
                alpha * alpha * (3.0f - 2.0f * alpha) : -1.0f;
            sharedRoot = transform_math::composeTransforms(
                grab_frame_math::objectFromGeneratedProxyLocalSpace(blendedProxy, targetInProxy),
                transform_math::invertTransform(primaryBodyLocal));
        }
        const auto sharedBody = transform_math::composeTransforms(sharedRoot, thisPrimary ? primaryBodyLocal : supportBodyLocal);
        const auto targetProxy = grab_frame_math::generatedProxyFromObjectWorld(sharedBody, _grabFrame.proxyAuthorityBodyHandSpace);
        if (!isFiniteNiTransform(targetProxy) || targetProxy.scale <= 0.0001f) {
            return false;
        }
        auto physicalNi = proxyWorld;
        physicalNi.rotate = transform_math::transposeRotation(physicalNi.rotate);
        auto targetNi = targetProxy;
        targetNi.rotate = transform_math::transposeRotation(targetNi.rotate);
        _grabFrame.looseObjectSharedProxyCorrection = transform_math::composeTransforms(
            transform_math::invertTransform(physicalNi), targetNi);
        if (!_grabFrame.looseObjectSharedPeerTrace) {
            ROCK_LOG_INFO(Hand, "{} hand loose two-hand pivot started ref={:08X} primary={} separation={:.3f}",
                handName(), getHeldRef()->GetFormID(), primary.handName(), separation);
        }
        _grabFrame.looseObjectSharedPeerTrace = peer->_grabFrame.traceId;
        _grabFrame.looseObjectSharedPrimaryIsLeft = primary._isLeft;
        proxyWorld = targetProxy;
        return true;
    }

    bool Hand::updateHeldDrive(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        const GrabReleaseContext& releaseContext,
        HeldDriveUpdate& update,
        Hand* peerHand,
        const RE::NiTransform* peerHandWorld)
    {
        update.proxyAuthorityWorld = handWorldTransform;
        update.hasProxyAuthorityFrame = resolveGrabAuthorityProxyFrame(
            world,
            handWorldTransform,
            nullptr,
            update.proxyAuthorityWorld,
            update.proxyAuthoritySource,
            GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget);
        if (!update.hasProxyAuthorityFrame) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: palm anchor proxy frame unavailable while held source={}",
                handName(),
                update.proxyAuthoritySource);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        if (!coordinateLooseObjectProxy(world, peerHand, peerHandWorld, update.proxyAuthorityWorld)) {
            ROCK_LOG_WARN(Hand, "{} hand release: shared object pivot target unavailable", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        if (_grabFrame.looseObjectSharedPeerTrace) {
            _looseWeaponReturnStartFrame = 0;
            update.proxyAuthoritySource = "sharedLooseObjectPalmPivot";
            if (_grabOffsetAcquisition.active) {
                ROCK_LOG_INFO(Hand, "{} OFFSET_ACQUIRE ended trace={} reason=shared-grip", handName(), _grabFrame.traceId);
                _grabOffsetAcquisition = {};
            }
        }
        const bool consumerTarget=_consumerObjectTarget.current(heldGrabIdentity(),runtimeFrameForObjectTarget())&&
            !_grabFrame.looseObjectSharedPeerTrace&&provider::runtime::authorize(_consumerObjectTarget.owner,api::grab::kInterfaceId,2,true)==api::Status::Ok;
        if(!consumerTarget&&_consumerObjectTarget.owner) {
            _consumerObjectTarget={};
            std::scoped_lock lock(_grabAuthorityProxyMutex);_consumerPairCount=0;
        }
        if(consumerTarget) {
            // Change only the queued proxy target. Frozen body/constraint
            // relations, finite-force motors and collision stay ROCK-owned.
            const auto& source=_consumerObjectTarget.target;
            RE::NiTransform target;
            for(int row=0;row<3;++row)for(int col=0;col<3;++col)target.rotate.entry[row][col]=source.rotate[row*3+col];
            target.translate={source.translate[0],source.translate[1],source.translate[2]};target.scale=source.scale;
            const auto bodyWorld=multiplyTransforms(target,_grabFrame.rootBodyLocal);
            update.proxyAuthorityWorld=grab_frame_math::generatedProxyFromObjectWorld(bodyWorld,_grabFrame.proxyAuthorityBodyHandSpace);
            update.proxyAuthoritySource="consumerHeldObjectTarget";
            _grabOffsetAcquisition={};
        }
        const bool advancingOffset = _grabOffsetAcquisition.active;
        if (advancingOffset) {
            const bool synchronizeAcquiringWeaponHand = isHoldingLooseWeapon() &&
                !_grabFrame.looseObjectSharedPeerTrace && _looseWeaponReturnStartFrame == 0;
            // Both endpoints ride the current physical hand frame. The fixed
            // body-local grip follows a straight segment while rotation uses
            // the same fraction; no offset is applied around the body origin.
            const float acquisitionDelta = held_object_physics_math::shouldQueueGrabAuthorityTargetForDelta(deltaTime) &&
                _looseWeaponReturnStartFrame != runtime_state::currentFrame().timing.sequence ? deltaTime : 0.0f;
            update.proxyAuthorityWorld = grab_offset_acquisition::advance(_grabOffsetAcquisition,
                update.proxyAuthorityWorld, _grabFrame.proxyAuthorityBodyHandSpace,
                _grabFrame.authority.pivotBConstraintLocalGame, acquisitionDelta);
            if (synchronizeAcquiringWeaponHand) {
                const float alpha = hand_visual_lerp_math::timedBlendAlpha(
                    _grabOffsetAcquisition.elapsedSeconds, _grabOffsetAcquisition.durationSeconds);
                _looseSharedHandBlendAlpha = alpha * alpha * (3.0f - 2.0f * alpha);
            }
            if (!_grabOffsetAcquisition.active) _looseWeaponReturnStartFrame = 0;
            update.proxyAuthoritySource = "offsetAcquisitionGripPath";
            if (!isFiniteNiTransform(update.proxyAuthorityWorld)) {
                ROCK_LOG_WARN(Hand, "{} hand release: invalid offset acquisition target trace={}", handName(), _grabFrame.traceId);
                releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                return false;
            }
        }
        update.desiredObjectWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(update.proxyAuthorityWorld, _grabFrame.proxyAuthorityHandSpace);
        if (_hasGrabFingerSweepDebug) {
            _grabFingerSweepDebugObjectWorld = update.desiredObjectWorld;
        }
        update.desiredBodyWorld =
            grab_frame_math::objectFromGeneratedProxyLocalSpace(update.proxyAuthorityWorld, _grabFrame.proxyAuthorityBodyHandSpace);
        update.activePivotBBodyLocalGame = activeProxyConstraintPivotBLocalGame();
        update.desiredTargetPointWorld = transform_math::localPointToWorld(update.desiredBodyWorld, update.activePivotBBodyLocalGame);

        if (tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, update.solvedBodyWorld)) {
            update.liveGripWorldForAuthority =
                transform_math::localPointToWorld(update.solvedBodyWorld, update.activePivotBBodyLocalGame);
            update.pivotTrackingErrorGameUnits =
                pointDistanceGameUnits(update.liveGripWorldForAuthority, update.desiredTargetPointWorld);
            update.hasPivotTrackingError = true;
            update.grabRotationErrorDegrees = _grabFrame.heldNode ?
                rotationDeltaDegrees(_grabFrame.heldNode->world.rotate, update.desiredObjectWorld.rotate) :
                rotationDeltaDegrees(update.solvedBodyWorld.rotate, update.desiredBodyWorld.rotate);
        }
        if (!update.hasPivotTrackingError) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object drive body readback failed before queuing grab authority bodyId={} phase={}",
                handName(),
                _savedObjectState.bodyId.value,
                grab_three_phase::phaseName(_grabAcquisitionPhase));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        if (advancingOffset) {
            _grabOffsetMaximumGripError = (std::max)(_grabOffsetMaximumGripError, update.pivotTrackingErrorGameUnits);
            _grabOffsetMaximumRotationError = (std::max)(_grabOffsetMaximumRotationError, update.grabRotationErrorDegrees);
            if (!_grabOffsetAcquisition.active) {
                ROCK_LOG_INFO(Hand,
                    "{} OFFSET_ACQUIRE target-complete trace={} gripError={:.3f}gu rotationError={:.3f}deg maxGripError={:.3f}gu maxRotationError={:.3f}deg",
                    handName(), _grabFrame.traceId, update.pivotTrackingErrorGameUnits, update.grabRotationErrorDegrees,
                    _grabOffsetMaximumGripError, _grabOffsetMaximumRotationError);
            }
        }
        // A lagging held object stays held; only a broken (non-finite) pivot
        // relation releases it.
        if (!std::isfinite(update.pivotTrackingErrorGameUnits)) {
            ROCK_LOG_WARN(Hand, "{} hand release: held object pivot error is non-finite", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return false;
        }

        // A free loose reference owns the whole model, including branches
        // outside its selected collision owner (e.g. a bottle's opaque mesh).
        // Actor/gore and fixed mechanical scopes retain their per-body path.
        const bool presentReferenceRoot = _heldObjectIsLooseWeapon ||
            (_savedObjectState.targetKind == grab_target::Kind::LooseObject &&
                _heldDriveDecision.mode != held_object_drive_policy::HeldBodySetDriveMode::FixedAttached &&
                _heldDriveDecision.mode != held_object_drive_policy::HeldBodySetDriveMode::IncompleteNativeScan);
        const auto presentation = held_scene_presentation::publishTargetTransport(
            _isLeft,
            world,
            _savedObjectState.bodyId.value,
            _grabFrame.traceId,
            update.desiredBodyWorld,
            update.solvedBodyWorld,
            presentReferenceRoot && _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr,
            _grabFrame.rootBodyLocal);
        update.hasPresentedBodyWorld = presentation.applied;
        update.presentedBodyWorld = presentation.presentedBodyWorld;

        update.heldBodyColliding = isHeldBodyColliding();
        const auto heldContactSnapshot = readHeldBodyContactSnapshot();
        update.heldMotorContactSoftening = update.heldBodyColliding;
        update.heldMotorContactReason = update.heldBodyColliding ? "legacy-recent-contact" : "no-recent-contact";
        if (heldContactSnapshot.recent) {
            const RE::NiPoint3 correctionGame = update.desiredTargetPointWorld - update.liveGripWorldForAuthority;
            const RE::NiPoint3 correctionHavok = gamePointToHavokPoint(correctionGame);
            RE::NiTransform heldContactBodyWorld{};
            RE::NiTransform otherContactBodyWorld{};
            const bool hasHeldContactBody =
                heldContactSnapshot.heldBodyId != INVALID_BODY_ID &&
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ heldContactSnapshot.heldBodyId }, heldContactBodyWorld);
            const bool hasOtherContactBody =
                heldContactSnapshot.otherBodyId != INVALID_BODY_ID &&
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ heldContactSnapshot.otherBodyId }, otherContactBodyWorld);
            const RE::NiPoint3 heldToOtherHavok =
                (hasHeldContactBody && hasOtherContactBody) ?
                    gamePointToHavokPoint(otherContactBodyWorld.translate - heldContactBodyWorld.translate) :
                    RE::NiPoint3{};
            const auto contactSoftening = held_object_contact_policy::evaluateHeldContactMotorSoftening(
                held_object_contact_policy::HeldContactMotorSofteningInput<RE::NiPoint3>{
                    .recentContact = true,
                    .hasCorrectionVector = update.hasPivotTrackingError,
                    .hasHeldToOtherVector = hasHeldContactBody && hasOtherContactBody,
                    .hasContactNormal = heldContactSnapshot.hasNormal,
                    .otherMotion = classifyHeldContactOtherMotion(world, heldContactSnapshot.otherBodyId),
                    .correctionTowardTarget = correctionHavok,
                    .heldToOther = heldToOtherHavok,
                    .contactNormal = heldContactSnapshot.contactNormalHavok,
                });
            update.heldMotorContactSoftening = contactSoftening.soften;
            update.heldMotorContactReason = contactSoftening.reason;
        }

        update.authorityForceScale = held_object_drive_policy::sanitizeMotorAuthorityScale(
            sharedGrabAuthorityForceScale(releaseContext.peerHandStillHolding));
        if (held_object_physics_math::shouldQueueGrabAuthorityTargetForDelta(deltaTime)) {
            queueProxyGrabAuthorityTarget(
                update.proxyAuthorityWorld,
                handWorldTransform,
                update.proxyAuthoritySource,
                deltaTime,
                forceFadeInTime,
                tauMin,
                update.pivotTrackingErrorGameUnits,
                update.grabRotationErrorDegrees,
                update.authorityForceScale,
                update.heldMotorContactSoftening);
        } else {
            ROCK_LOG_SAMPLE_WARN(Hand,
                500,
                "{} hand skipped grab authority target after stutter delta dt={:.6f}s threshold={:.3f}s; holding last proxy target",
                handName(),
                std::isfinite(deltaTime) ? deltaTime : -1.0f,
                held_object_physics_math::kMaxGrabAuthorityTargetDeltaSeconds);
        }

        update.averageGrabDeviationGameUnits = recordDeviationAverage(
            _grabDeviationHistory,
            _grabDeviationHistoryCount,
            _grabDeviationHistoryNext,
            update.pivotTrackingErrorGameUnits);

        return true;
    }

    bool Hand::updateHeldVisualPresentation(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        const HeldDriveUpdate& driveUpdate,
        const GrabReleaseContext& releaseContext,
        bool& outConvergingAcquisitionPhase)
    {
        const float pivotTrackingErrorGameUnits = driveUpdate.pivotTrackingErrorGameUnits;
        const bool hasPivotTrackingError = driveUpdate.hasPivotTrackingError;
        const bool heldMotorContactSoftening = driveUpdate.heldMotorContactSoftening;
        tickHeldBodyContact();
        outConvergingAcquisitionPhase =
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::NearConverging ||
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::GravityPulling;
        const float acquisitionVisualEnvelope =
            grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(
                g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                g_rockConfig.rockGrabAcquisitionVisualStartDistanceGameUnits);
        const float acquisitionVisualAttachEnvelope =
            (std::min)(
                acquisitionVisualEnvelope,
                (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, g_rockConfig.rockGrabPocketRadiusGameUnits));
        const bool acquisitionVisualEligible =
            outConvergingAcquisitionPhase &&
            hasPivotTrackingError &&
            _grabObjectGripAtGrab.valid &&
            pivotTrackingErrorGameUnits <= acquisitionVisualAttachEnvelope;
        const bool preserveTransferredHand = _grabFrame.equippedWeaponTransfer && _grabFrame.authoredWeaponPose.valid();
        const auto visualPublishDecision = grab_motion_controller::evaluateVisualHandPublishGate(
            grab_motion_controller::VisualHandPublishInput{
                .hasTelemetryCapture = _grabFrame.hasTelemetryCapture,
                .touchHeldPhase = _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld,
                .acquisitionVisualEligible = acquisitionVisualEligible,
                .hasPivotTrackingError = hasPivotTrackingError,
                .motorContactSoftening = heldMotorContactSoftening,
                .hasSettledVisualRelation = _grabFrame.seat.hasSettledVisualHandRelation,
                .requiresSettledVisualRelation = _grabFrame.seat.requiresSettledVisualHandRelation,
                .transferredAuthoredGrip = preserveTransferredHand,
            });
        // A transferred grip already owns the hand. Its small constraint-seat
        // correction must carry that hand with the presented weapon, rather than
        // returning the arm to the controller until acquisition finishes.
        const bool weaponVisual = isHoldingLooseWeapon();
        if (_grabFrame.hasTelemetryCapture &&
            (weaponVisual || ((!_grabOffsetAcquisition.active || preserveTransferredHand) && visualPublishDecision.apply))) {
            RE::NiTransform heldVisualNodeWorld{};
            bool hasHeldVisualNodeWorld = false;
            bool heldVisualNodeFromPresentedPose = false;
            if (driveUpdate.hasPresentedBodyWorld) {
                /*
                 * Pose the hand on the pose the scene writer presents THIS
                 * frame. heldNode->world still holds the previous frame's
                 * presented pose here (the writer runs after physics), so
                 * reading it would put the rendered hand one locomotion step
                 * behind the held mesh and the skeleton -- the per-hand arm
                 * stutter measured during stick locomotion.
                 */
                heldVisualNodeWorld =
                    deriveNodeWorldFromBodyWorld(driveUpdate.presentedBodyWorld, _grabFrame.authority.bodyLocal);
                hasHeldVisualNodeWorld = true;
                heldVisualNodeFromPresentedPose = true;
            } else if (_grabFrame.heldNode) {
                heldVisualNodeWorld = _grabFrame.heldNode->world;
                hasHeldVisualNodeWorld = true;
            } else {
                RE::NiTransform grabBodyWorld{};
                if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
                    heldVisualNodeWorld = deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.authority.bodyLocal);
                    hasHeldVisualNodeWorld = true;
                }
            }

            if (hasHeldVisualNodeWorld) {
                /*
                 * ROCK visual hand update:
                 *     adjustedHand = heldObjectWorld * inverse(frozenObjectHandSpace)
                 *
                 * This is intentionally visual-only. The active dynamic grab
                 * drive remains the only object motor authority, so the rendered
                 * hand can settle to the object without feeding wrist/object
                 * rotation back into the grab relation.
                 */
                RE::NiTransform targetVisualHandWorld =
                    hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(heldVisualNodeWorld, _grabFrame.rawHandSpace);
                targetVisualHandWorld.scale = handWorldTransform.scale;

                RE::NiTransform nextVisualHandWorld = targetVisualHandWorld;
                const bool smoothVisualHand = !preserveTransferredHand && hand_visual_lerp_math::shouldSmoothHeldObjectRelativeHand(
                    isHoldingLooseWeapon() ? g_rockConfig.rockWeaponSupportGripHandLerpEnabled : g_rockConfig.rockGrabHandLerpEnabled,
                    _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld,
                    visualPublishDecision.acquisition, weaponVisual);
                if (!_hasGrabVisualHandTransform) {
                    RE::NiTransform acquisitionStart =
                        _grabVisualReturn.active && isUsableGrabVisualTransform(_grabVisualReturn.lastApplied) ?
                        _grabVisualReturn.lastApplied :
                        handWorldTransform;
                    if (weaponVisual && !_grabVisualReturn.active) {
                        RE::NiTransform presented{};
                        if (frik_hand_world_authority::tryGetPresentedHandWorld(_isLeft, presented) &&
                            isUsableGrabVisualTransform(presented)) acquisitionStart = presented;
                    }
                    _grabVisualHandTransform = acquisitionStart;
                    _grabVisualHandLerpStartTransform = acquisitionStart;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = smoothVisualHand ?
                        (isHoldingLooseWeapon() ? hand_visual_lerp_math::computeVisualReturnDuration(
                            acquisitionStart, targetVisualHandWorld, looseWeaponAcquireConfig()) : hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                            hand_visual_lerp_math::distanceGameUnits(acquisitionStart.translate, targetVisualHandWorld.translate),
                            g_rockConfig.rockGrabHandLerpTimeMin,
                            g_rockConfig.rockGrabHandLerpTimeMax,
                            g_rockConfig.rockGrabHandLerpMinDistance,
                            g_rockConfig.rockGrabHandLerpMaxDistance)) :
                        0.0f;
                    _hasGrabVisualHandTransform = true;
                }

                float visualHandLerpAlpha = 1.0f;
                if (smoothVisualHand) {
                    _grabVisualHandLerpElapsedSeconds = hand_visual_lerp_math::advanceTimedBlendElapsed(
                        _grabVisualHandLerpElapsedSeconds,
                        deltaTime,
                        _grabVisualHandLerpDurationSeconds);
                    const auto advancedVisual = hand_visual_lerp_math::blendTransformOverDuration(
                        _grabVisualHandLerpStartTransform,
                        targetVisualHandWorld,
                        _grabVisualHandLerpElapsedSeconds,
                        _grabVisualHandLerpDurationSeconds);
                    nextVisualHandWorld = advancedVisual.transform;
                    visualHandLerpAlpha = hand_visual_lerp_math::timedBlendAlpha(
                        _grabVisualHandLerpElapsedSeconds,
                        _grabVisualHandLerpDurationSeconds);
                    if (weaponVisual && _looseSharedHandBlendAlpha >= 0.0f) {
                        // Initial wrist acquisition and paired joining use the
                        // weapon's exact eased progress, including its last frame.
                        // Visual-only support retains its independent wrist blend.
                        visualHandLerpAlpha = _looseSharedHandBlendAlpha;
                        nextVisualHandWorld = hand_visual_lerp_math::interpolateTransform(
                            _grabVisualHandLerpStartTransform, targetVisualHandWorld, visualHandLerpAlpha);
                        if (visualHandLerpAlpha >= 1.0f)
                            _grabVisualHandLerpElapsedSeconds = _grabVisualHandLerpDurationSeconds;
                        if (visualHandLerpAlpha < 1.0f) {
                            ROCK_LOG_SAMPLE_DEBUG(Hand, 100,
                                "{} loose grip slurp trace={} paired={} sharedAlpha={:.3f} wristDistance={:.3f}gu wristAngle={:.2f}deg",
                                handName(), _grabFrame.traceId, _grabFrame.looseObjectSharedPeerTrace != 0,
                                visualHandLerpAlpha,
                                pointDistanceGameUnits(nextVisualHandWorld.translate, targetVisualHandWorld.translate),
                                hand_visual_lerp_math::rotationDistanceDegrees(nextVisualHandWorld,targetVisualHandWorld));
                        }
                    }
                } else {
                    _grabVisualHandLerpStartTransform = targetVisualHandWorld;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = 0.0f;
                }

                const float visualHandDeviationGameUnits =
                    pointDistanceGameUnits(nextVisualHandWorld.translate, handWorldTransform.translate);
                if (!std::isfinite(visualHandDeviationGameUnits)) {
                    ROCK_LOG_WARN(Hand, "{} hand release: visual held-object hand target is non-finite", handName());
                    clearGrabExternalHandWorldTransform(_isLeft);
                    releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
                    return false;
                }
                _grabVisualHandTransform = nextVisualHandWorld;
                if (applyGrabExternalHandWorldTransform(_isLeft, _grabVisualHandTransform)) {
                    _lastPublishedGrabVisualHandTransform = _grabVisualHandTransform;
                    _hasLastPublishedGrabVisualHandTransform = true;
                    clearGrabVisualReturn("active-grab-authority-acquired", false);
                    if (!_grabOffsetAcquisition.active && _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld)
                        _grabFrame.equippedWeaponTransfer = false;
                }

                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} GRAB VISUAL HAND: relation=heldRelative phase={} visualOnly=yes authority={} follow={} lerpAlpha={:.2f} lerpDuration={:.3f}s target=({:.1f},{:.1f},{:.1f}) applied=({:.1f},{:.1f},{:.1f}) live=({:.1f},{:.1f},{:.1f}) deviation={:.2f}gu normalAuthority=false",
                    handName(),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    visualPublishDecision.reason,
                    smoothVisualHand ? "smoothedAcquisition" : "immediateHeldObject",
                    visualHandLerpAlpha,
                    _grabVisualHandLerpDurationSeconds,
                    targetVisualHandWorld.translate.x,
                    targetVisualHandWorld.translate.y,
                    targetVisualHandWorld.translate.z,
                    _grabVisualHandTransform.translate.x,
                    _grabVisualHandTransform.translate.y,
                    _grabVisualHandTransform.translate.z,
                    handWorldTransform.translate.x,
                    handWorldTransform.translate.y,
                    handWorldTransform.translate.z,
                    visualHandDeviationGameUnits);
            } else {
                if (_hasGrabVisualHandTransform) {
                    clearGrabExternalHandWorldTransform(_isLeft);
                    _hasGrabVisualHandTransform = false;
                    _lastPublishedGrabVisualHandTransform = {};
                    _hasLastPublishedGrabVisualHandTransform = false;
                }
                _grabVisualHandLerpStartTransform = {};
                _grabVisualHandLerpElapsedSeconds = 0.0f;
                _grabVisualHandLerpDurationSeconds = 0.0f;
            }
        } else {
            if (_hasGrabVisualHandTransform) {
                clearGrabExternalHandWorldTransform(_isLeft);
                _hasGrabVisualHandTransform = false;
                _lastPublishedGrabVisualHandTransform = {};
                _hasLastPublishedGrabVisualHandTransform = false;
            }
            _grabVisualHandLerpStartTransform = {};
            _grabVisualHandLerpElapsedSeconds = 0.0f;
            _grabVisualHandLerpDurationSeconds = 0.0f;
        }


        return true;
    }

    void Hand::updateHeldAcquisition(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        const HeldDriveUpdate& driveUpdate,
        bool convergingAcquisitionPhase)
    {
        const auto& desiredObjectWorld = driveUpdate.desiredObjectWorld;
        const auto& proxyAuthorityWorld = driveUpdate.proxyAuthorityWorld;
        const auto& desiredTargetPointWorld = driveUpdate.desiredTargetPointWorld;
        const auto& activePivotBBodyLocalGame = driveUpdate.activePivotBBodyLocalGame;
        const bool heldBodyColliding = driveUpdate.heldBodyColliding;

        /*
         * FRIK publishes joint values before the root-flattened scene reflects
         * them. Surface-local corrections built in that same update would use
         * the previous finger skeleton and can lock a closed or displaced pose.
         * Finish the local-transform layer once, on the next TouchHeld update,
         * after the solved joint pose is the live skeleton source.
         */
        if (_grabFingerLocalTransformFinalizePending &&
            _grabFingerPosePublished &&
            _grabFingerPose.solved &&
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
            const auto finalFingerPose =
                grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, desiredObjectWorld);
            applyRockGrabHandPose(_isLeft,
                finalFingerPose,
                _grabFingerJointPose,
                _hasGrabFingerJointPose,
                _grabFingerLocalTransforms,
                _grabFingerLocalTransformMask,
                _hasGrabFingerLocalTransforms,
                deltaTime,
                /*publishLocalTransforms=*/true,
                /*snapJointPoseToTarget=*/false);
            _grabFingerLocalTransformFinalizePending = false;
            ROCK_LOG_DEBUG(Hand,
                "{} THREE-PHASE GRAB FINAL POSE: local transforms finalized after settled joint publication",
                handName());
        }

        if (convergingAcquisitionPhase && !_grabOffsetAcquisition.active && _grabObjectGripAtGrab.valid) {
            const auto previousAcquisitionPhase = _grabAcquisitionPhase;
            RE::NiTransform grabBodyWorld{};
            const bool hasGrabBody = tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld);
            const RE::NiPoint3 liveGripWorld =
                hasGrabBody ? transform_math::localPointToWorld(grabBodyWorld, activePivotBBodyLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 targetGripWorld = desiredTargetPointWorld;
            const RE::NiPoint3 gripError = targetGripWorld - liveGripWorld;
            const float gripErrorGameUnits = hasGrabBody ? std::sqrt(gripError.x * gripError.x + gripError.y * gripError.y + gripError.z * gripError.z) :
                                                           std::numeric_limits<float>::max();
            const float touchDistance = (std::max)(0.1f, g_rockConfig.rockGrabTouchAcquireDistanceGameUnits);
            const auto convergenceDecision =
                grab_three_phase::evaluateConvergencePromotion(grab_three_phase::ConvergencePromotionInput{
                    .hasGrabBody = hasGrabBody,
                    .heldBodyColliding = heldBodyColliding,
                    .gripErrorGameUnits = gripErrorGameUnits,
                    .previousGripErrorGameUnits = _grabConvergePreviousGripErrorGameUnits,
                    .deltaSeconds = deltaTime,
                    .elapsedSeconds = _grabStartTime,
                    .maxTimeSeconds = g_rockConfig.rockGrabConvergeMaxTimeSeconds,
                    .touchDistanceGameUnits = touchDistance,
                    .pocketRadiusGameUnits = g_rockConfig.rockGrabPocketRadiusGameUnits,
                    .stableInsidePocketSeconds = _grabConvergeStableInsidePocketSeconds,
                    .requiredStableInsidePocketSeconds = g_rockConfig.rockGrabConvergeStableSeconds,
                    .maxSeparatingSpeedGameUnitsPerSecond = g_rockConfig.rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond,
                });
            _grabConvergeStableInsidePocketSeconds = convergenceDecision.nextStableInsidePocketSeconds;
            _grabConvergePreviousGripErrorGameUnits = gripErrorGameUnits;
            const bool reachedTouchRange = convergenceDecision.reachedTouchRange;
            const bool convergenceTimedOutInsidePocket = convergenceDecision.timedOutInsidePocket;
            if (convergenceTimedOutInsidePocket) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} THREE-PHASE GRAB CONVERGE PROMOTION READY: phase={} gripErr={:.2f}gu touch={:.2f}gu pocket={:.2f}gu elapsed={:.3f}s colliding={} stableDwell={:.3f}s sepSpeed={:.2f}gu/s",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    gripErrorGameUnits,
                    touchDistance,
                    g_rockConfig.rockGrabPocketRadiusGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no",
                    _grabConvergeStableInsidePocketSeconds,
                    convergenceDecision.separatingSpeedGameUnitsPerSecond);
            }

            if (hasGrabBody && !reachedTouchRange && !convergenceTimedOutInsidePocket && !_grabFrame.syntheticLooseWeaponPrimaryAttach &&
                g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose && _grabFingerPosePublished) {
                const float nearDistance = (std::max)(touchDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
                const float progressDenominator = (std::max)(0.001f, nearDistance - touchDistance);
                const float acquisitionProgress = 1.0f - std::clamp((gripErrorGameUnits - touchDistance) / progressDenominator, 0.0f, 1.0f);
                const auto resolvedTargetPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, desiredObjectWorld);
                const auto acquisitionFingerPose = buildAcquisitionFingerPose(resolvedTargetPose, acquisitionProgress);
                applyRockGrabHandPose(_isLeft, acquisitionFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms, deltaTime, false);
            }

            if (reachedTouchRange || convergenceTimedOutInsidePocket) {
                const char* promotionReason =
                    _grabFrame.syntheticLooseWeaponPrimaryAttach ? "looseWeaponPrimaryAttachSettled" :
                    (reachedTouchRange ? "threePhaseTouchReachedFrozenRelation" : "threePhaseTimeoutInsidePocket");
                /*
                 * The seat was chosen once at capture against the pose the
                 * object rests in, so arrival never re-picks it: it only lets
                 * the visual hand lock onto that frozen relation.
                 */
                if (_grabFrame.syntheticLooseWeaponPrimaryAttach || _grabFrame.seat.requiresSettledVisualHandRelation) {
                    _grabFrame.seat.hasSettledVisualHandRelation = true;
                }
                _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::TouchHeld;
                _grabFrame.fadeInGrabConstraint = false;
                _grabFrame.motorFadeReason = promotionReason;
                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB TRANSITION: {} -> TouchHeld relation=frozenRockPointToPalm reason={} gripErr={:.2f}gu elapsed={:.3f}s colliding={} posePublished={}",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    promotionReason,
                    gripErrorGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no",
                    _grabFingerPosePublished ? "yes" : "no");

                if (!_grabFrame.syntheticLooseWeaponPrimaryAttach && g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose) {
                    if (!_grabFingerPosePublished) {
                        const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.authority.bodyLocal);
                        RE::NiTransform finalPoseHandWorld =
                            hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(currentNodeWorld, _grabFrame.rawHandSpace);
                        finalPoseHandWorld.scale = handWorldTransform.scale;
                        const bool legacyFingerPose = _grabFingerPose.mode == grab_finger_pose_runtime::FingerPoseMode::June2;
                        if (legacyFingerPose) {
                            finalPoseHandWorld = handWorldTransform;
                        }
                        if (!_grabFrame.fingerPoseAimValid && _grabFrame.pivotAuthority.normalTrusted) {
                            _grabFrame.fingerPoseAimValid = true;
                            _grabFrame.fingerPoseAimReason = "touchHeldNormalTrusted";
                        }
                        RE::NiPoint3 fingerPosePivotWorld = _grabFrame.hasTelemetryCapture ? _grabFrame.authority.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
                        RE::NiPoint3 livePivotAWorld{};
                        if (tryComputeGrabProxyLocalPalmPocketPivotAWorld(world, livePivotAWorld)) {
                            fingerPosePivotWorld = livePivotAWorld;
                        }
                        solveAndPublishGrabFingerPose(GrabFingerSolveFrame{
                            .objectWorld = currentNodeWorld,
                            .liveHandWorld = handWorldTransform,
                            .solveHandWorld = finalPoseHandWorld,
                            .pivotWorld = fingerPosePivotWorld,
                            .touchHeld = true,
                            .stage = "touchHeld",
                        });
                    } else {
                        const RE::NiTransform finalPoseObjectWorld = grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyAuthorityWorld, _grabFrame.proxyAuthorityHandSpace);
                        const auto finalFingerPose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, finalPoseObjectWorld);
                        applyRockGrabHandPose(_isLeft, finalFingerPose, _grabFingerJointPose, _hasGrabFingerJointPose, _grabFingerLocalTransforms, _grabFingerLocalTransformMask,
                            _hasGrabFingerLocalTransforms, 0.0f, /*publishLocalTransforms=*/false, true);
                        _grabFingerLocalTransformFinalizePending = _grabFingerPose.solved;
                    }
                }
            }
        }

        if (_grabFingerPose.mode == grab_finger_pose_runtime::FingerPoseMode::June2 &&
            _hasGrabFingerPose && _grabFingerPosePublished &&
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld) {
            _legacyFingerPoseDeltaTime += std::isfinite(deltaTime) ? (std::max)(0.0f, deltaTime) : 0.0f;
            if (++_legacyFingerPoseUpdates >= 3) {
                auto pose = _grabFingerPose;
                RE::NiTransform currentGrabBodyWorld{};
                if (tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, currentGrabBodyWorld)) {
                    const auto currentNodeWorld = deriveNodeWorldFromBodyWorld(currentGrabBodyWorld, _grabFrame.authority.bodyLocal);
                    pose = grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, currentNodeWorld);
                }
                applyRockGrabHandPose(_isLeft, pose, _grabFingerJointPose, _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms, _grabFingerLocalTransformMask, _hasGrabFingerLocalTransforms,
                    _legacyFingerPoseDeltaTime);
                _legacyFingerPoseUpdates = 0;
                _legacyFingerPoseDeltaTime = 0.0f;
            }
        }
    }

    void Hand::finalizeHeldObjectUpdate(RE::hknpWorld* world,
        float forceFadeInTime,
        const HeldDriveUpdate& driveUpdate)
    {
        const auto& desiredBodyWorld = driveUpdate.desiredBodyWorld;
        const auto& desiredTargetPointWorld = driveUpdate.desiredTargetPointWorld;
        const auto& activePivotBBodyLocalGame = driveUpdate.activePivotBBodyLocalGame;
        const char* proxyAuthoritySource = driveUpdate.proxyAuthoritySource;
        const char* heldMotorContactReason = driveUpdate.heldMotorContactReason;
        const float grabRotationErrorDegrees = driveUpdate.grabRotationErrorDegrees;
        const float authorityForceScale = driveUpdate.authorityForceScale;
        const float averageGrabDeviationGameUnits = driveUpdate.averageGrabDeviationGameUnits;
        const bool hasProxyAuthorityFrame = driveUpdate.hasProxyAuthorityFrame;
        const bool heldBodyColliding = driveUpdate.heldBodyColliding;
        const bool heldMotorContactSoftening = driveUpdate.heldMotorContactSoftening;

        const float fadeDuration = std::max(forceFadeInTime, 0.0001f);
        const float grabFadeFactor = _grabFrame.fadeInGrabConstraint ? std::clamp(_grabStartTime / fadeDuration, 0.0f, 1.0f) : 1.0f;

        if (_state == HandState::HeldInit) {
            if (grabFadeFactor >= 0.999f) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::HeldFadeComplete });
                ROCK_LOG_DEBUG(Hand, "{} hand: HeldInit -> HeldBody ({} dynamic grab fade complete, {:.2f}s)", handName(), kHeldObjectDriveName, _grabStartTime);
            }
        }

        recordHeldObjectVelocitySample(world);

        _heldLogCounter++;
        if (_heldLogCounter >= 45) {
            _heldLogCounter = 0;

            std::uint64_t driveQueuedTargets = 0;
            std::uint64_t driveFlushedTargets = 0;
            std::uint64_t driveFailedFlushes = 0;
            float driveLastFlushDeltaSeconds = 0.0f;
            {
                std::scoped_lock lock(_grabAuthorityProxyMutex);
                driveQueuedTargets = _grabAuthorityProxyQueuedSequence;
                driveFlushedTargets = _grabAuthorityProxyFlushSequence;
                driveFailedFlushes = _grabAuthorityProxyFailedFlushes;
                driveLastFlushDeltaSeconds = _grabAuthorityProxyLastFlushDeltaSeconds;
            }
            RE::NiTransform grabDriveObjectWorld{};
            const bool hasGrabDriveObject = tryGetGrabDriveObjectWorldTransform(world, _savedObjectState.bodyId, grabDriveObjectWorld);

            const bool hasGrabPivotFrame = hasGrabDriveObject;
            const RE::NiPoint3 pivotAWorld = desiredTargetPointWorld;
            const RE::NiPoint3 pivotBWorld = hasGrabPivotFrame ? transform_math::localPointToWorld(grabDriveObjectWorld, activePivotBBodyLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 pivotError = pivotAWorld - pivotBWorld;
            const float pivotErrGame = hasGrabPivotFrame ? std::sqrt(pivotError.x * pivotError.x + pivotError.y * pivotError.y + pivotError.z * pivotError.z) : 0.0f;

            const RE::NiPoint3 bodyDelta = hasGrabPivotFrame ? (grabDriveObjectWorld.translate - desiredBodyWorld.translate) : RE::NiPoint3{};
            const float bodyDistGame = hasGrabPivotFrame ? std::sqrt(bodyDelta.x * bodyDelta.x + bodyDelta.y * bodyDelta.y + bodyDelta.z * bodyDelta.z) : 0.0f;

            float objVelMag = 0.0f;
            {
                auto* objMotion = havok_runtime::getBodyMotion(world, _savedObjectState.bodyId);
                if (objMotion) {
                    const auto& velocity = objMotion->linearVelocity;
                    objVelMag = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
                }
            }

            const std::uint32_t heldFormId = _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0;
            const float lastGrabPhysicsHz = _lastGrabPhysicsHz.load(std::memory_order_relaxed);

            ROCK_LOG_DEBUG(Hand,
                "{} HELD dynamic: drive={} bodyDriveMode={} linearScope={} angularScope={} massScope={} looseWeapon={} formID={:08X} constraint={} queued={} flushed={} failedFlushes={} lastDt={:.6f} proxyFrame={}/{} "
                "phase={} posePublished={} fade={:.2f}/{} reason={} colliding={} motorContact={} contactReason={} forceBudget={:.2f} physHz={:.1f} longLever={:.1f}gu pivotTrack={:.1f}gu avgTrack={:.1f}gu rotErr={:.1f}deg bDist={:.1f}gu objVel={:.3f} "
                "paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
                "targetBody=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
                handName(),
                kHeldObjectDriveName,
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedMass ? "bodySet" : "primaryOnly",
                _heldObjectIsLooseWeapon ? "yes" : "no",
                heldFormId,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                driveQueuedTargets,
                driveFlushedTargets,
                driveFailedFlushes,
                driveLastFlushDeltaSeconds,
                proxyAuthoritySource,
                hasProxyAuthorityFrame ? "ok" : "fallback",
                grab_three_phase::phaseName(_grabAcquisitionPhase),
                _grabFingerPosePublished ? "yes" : "no",
                grabFadeFactor,
                _grabFrame.fadeInGrabConstraint ? "on" : "off",
                _grabFrame.motorFadeReason,
                heldBodyColliding ? "yes" : "no",
                heldMotorContactSoftening ? "soften" : "preserve",
                heldMotorContactReason,
                authorityForceScale,
                lastGrabPhysicsHz,
                _grabFrame.pivotAuthority.longLeverGameUnits,
                pivotErrGame,
                averageGrabDeviationGameUnits,
                grabRotationErrorDegrees,
                bodyDistGame,
                objVelMag,
                pivotAWorld.x, pivotAWorld.y, pivotAWorld.z, pivotBWorld.x, pivotBWorld.y, pivotBWorld.z,
                desiredBodyWorld.translate.x, desiredBodyWorld.translate.y, desiredBodyWorld.translate.z,
                grabDriveObjectWorld.translate.x, grabDriveObjectWorld.translate.y, grabDriveObjectWorld.translate.z);

        }

        {
            const auto wakeBodies = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(_savedObjectState.bodyId.value, _heldBodyIds);
            for (const auto bodyId : wakeBodies) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
        }

    }

    void Hand::updateHeldObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        const BodyBoneColliderSet* bodyBoneColliders,
        const GrabReleaseContext& releaseContext,
        Hand* peerHand,
        const RE::NiTransform* peerHandWorld)
    {
        if (!isHolding() || !world)
            return;
        // A transfer may refresh the assembly after its owning hand already
        // ran earlier in the bilateral input pass. Never tick timers or consume
        // presentation history twice; a new grab resets this frame identity.
        const auto frame = runtime_state::currentFrame().frameIndex;
        if (frame != 0 && _heldObjectUpdateFrame == frame) return;
        _heldObjectUpdateFrame = frame;
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabHeldObjectUpdate);
        if (!validateHeldObjectUpdate(world, releaseContext)) {
            return;
        }


        suppressHandCollisionForGrab(world, bodyBoneColliders);
        if (_heldObjectIsLooseWeapon) {
            suppressBodyCollisionForHeldLooseWeapon(world, bodyBoneColliders);
        }

        _grabStartTime += held_object_physics_math::finitePositiveOrZero(deltaTime);

        const HeldHandMotionSample handMotion = recordHeldControllerMotionSample(handWorldTransform, runtime_state::currentFrame().timing);
        (void)handMotion;

        /*
         * ROCK freezes the visible object/node relation in generated/proxy
         * authority space, then composes it with the rigid-body local transform
         * for the driven body target. BODY remains object-space authority and
         * MOTION remains COM/weight/diagnostic data only.
         */
        HeldDriveUpdate driveUpdate{};
        if (!updateHeldDrive(
                world,
                handWorldTransform,
                deltaTime,
                forceFadeInTime,
                tauMin,
                releaseContext,
                driveUpdate,
                peerHand,
                peerHandWorld)) {
            return;
        }
        bool convergingAcquisitionPhase = false;
        if (!updateHeldVisualPresentation(
                world,
                handWorldTransform,
                deltaTime,
                driveUpdate,
                releaseContext,
                convergingAcquisitionPhase)) {
            return;
        }
        updateHeldAcquisition(
            world,
            handWorldTransform,
            deltaTime,
            driveUpdate,
            convergingAcquisitionPhase);
        finalizeHeldObjectUpdate(world, forceFadeInTime, driveUpdate);
    }

    void Hand::flushPendingCustomGrabAuthority(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
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
            const bool hasAuthority = _grabAuthorityProxy.isValid() &&
                                      _grabAuthorityProxyHknpWorld == world &&
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
                ROCK_LOG_SAMPLE_WARN(Hand,
                    1000,
                    "{} hand grab authority flush skipped unmeasured physics timing substep={}/{}",
                    handName(),
                    timing.substepIndex,
                    timing.substepCount);
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
            _grabAuthoritySourceClock.advanceSource(
                pending.proxyWorld.translate,
                pending.proxyWorld.rotate,
                pending.deltaTime,
                _grabAuthorityProxyQueuedSequence);
            livePalmReferenceOk = tryResolveLivePalmAnchorReference(world, livePalmReference);
            if (!livePalmReferenceOk) {
                ++_grabAuthorityProxyFailedFlushes;
                _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
            }
            /*
             * The game-frame held update owns pending.proxyWorld. Rebinding it
             * here to the live palm body reintroduced one-Havok-tick latency
             * during stick locomotion, even after collider targets were bridged.
             * Keep live palm as a fail-closed health/telemetry read only.
             */
            previousProxyWorld = _hasLastAppliedGrabAuthorityProxyWorld ? _lastAppliedGrabAuthorityProxyWorld : pending.proxyWorld;
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
            pending.proxyWorld.translate = _grabAuthoritySourceClock.evaluate(timing.substepIndex, timing.substepCount, resampleAction);
            resampleRebaseCount = _grabAuthoritySourceClock.rebaseCount;
            _lastGrabSourceIntervalSeconds.store(
                _grabAuthoritySourceClock.filteredSourceIntervalSeconds,
                std::memory_order_relaxed);
            float linearVelocityHavok[4]{};
            float angularVelocityHavok[4]{};
            float nativeLinearVelocityIgnored[4]{};
            grab_authority_proxy::computeLinearVelocityHavok(previousProxyWorld, pending.proxyWorld, driveDelta, linearVelocityHavok);
            if (livePalmReference.hasMotionVelocity) {
                angularVelocityHavok[0] = livePalmReference.angularVelocityRadiansPerSecond.x;
                angularVelocityHavok[1] = livePalmReference.angularVelocityRadiansPerSecond.y;
                angularVelocityHavok[2] = livePalmReference.angularVelocityRadiansPerSecond.z;
                proxyVelocityTelemetryOk = true;
            } else if (livePalmReferenceOk) {
                proxyVelocityTelemetryOk = computeHardKeyframeVelocityForTarget(
                    world,
                    proxyBodyId,
                    pending.proxyWorld,
                    driveDelta,
                    nativeLinearVelocityIgnored,
                    angularVelocityHavok);
            }
            proxyLinearVelocityHavokMagnitude = std::sqrt(
                linearVelocityHavok[0] * linearVelocityHavok[0] + linearVelocityHavok[1] * linearVelocityHavok[1] +
                linearVelocityHavok[2] * linearVelocityHavok[2]);
            proxyAngularVelocityRadiansPerSecond = std::sqrt(
                angularVelocityHavok[0] * angularVelocityHavok[0] + angularVelocityHavok[1] * angularVelocityHavok[1] +
                angularVelocityHavok[2] * angularVelocityHavok[2]);

            if (livePalmReferenceOk) {
                queueGeneratedKeyframedBodyTarget(_grabAuthorityProxyDriveState, pending.proxyWorld, driveDelta, 1000.0f);
                proxyDriveResult = driveGeneratedKeyframedBody(
                    world,
                    _grabAuthorityProxy,
                    _grabAuthorityProxyDriveState,
                    timing,
                    "grab-authority-proxy",
                    0,
                    g_rockConfig.rockHandBoneColliderMaxLinearVelocity,
                    g_rockConfig.rockHandBoneColliderMaxAngularVelocity);
                proxyDriveOk = proxyDriveResult.driven;
            }
            if (proxyDriveOk) {
                proxyReadbackBetweenOk =
                    tryResolveLiveBodyWorldTransform(world, proxyBodyId, proxyReadbackBetween, &proxyReadbackSourceBetween, &proxyReadbackMotionIndexBetween);
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
                targetUpdateOk = updateProxyConstraintGrabDriveTarget(
                    world,
                    pending.proxyWorld,
                    desiredObjectWorld,
                    desiredBodyWorld,
                    desiredTargetPointWorld,
                    activePivotBBodyLocalGame);
                if (!targetUpdateOk) {
                    ++_grabAuthorityProxyFailedFlushes;
                    _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
                } else {
                    const auto pendingHeldAuthority =
                        grab_motion_controller::evaluateHeldAuthority(pending.heldBodyColliding);
                    const bool motorsReady = updateConstraintGrabDriveMotors(
                        world,
                        driveDelta,
                        pending.forceFadeInTime,
                        pending.tauMin,
                        pending.authorityForceScale,
                        pending.heldBodyColliding,
                        pendingHeldAuthority);
                    angularDriveOk =
                        motorsReady &&
                        _activeConstraint.isValid() &&
                        _activeConstraint.usesRagdollAngularMotorAtom() &&
                        _activeConstraint.linearMotor &&
                        _activeConstraint.angularMotor;
                    if (_activeConstraint.angularMotor) {
                        angularMotorBudget = (std::max)(
                            std::fabs(_activeConstraint.angularMotor->minForce),
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
                    if (flushSequence <= 16 || _grabAuthorityProxyLogCounter >= 45 ||
                        !proxyReadbackBetweenOk ||
                        !angularDriveOk ||
                        resampleAction == grab_authority_source_clock::ResampleAction::Rebase ||
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
                "{} hand proxy dynamic grab drive failed; release queued: proxyBody={} livePalm={} driven={} stale={} missing={} ownerMismatch={} substep={}/{} dt={:.6f}",
                handName(),
                proxyBodyId.value,
                livePalmReferenceOk ? "ok" : "fail",
                proxyDriveResult.driven ? "ok" : "fail",
                proxyDriveResult.skippedStale ? "yes" : "no",
                proxyDriveResult.missingBody ? "yes" : "no",
                proxyDriveResult.bodyCollisionObjectMismatch ? "yes" : "no",
                timing.substepIndex,
                timing.substepCount,
                timing.substepDeltaSeconds);
            return;
        }

        if (!targetUpdateOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab target update failed; release queued: proxyBody={} constraint={} substep={}/{}",
                handName(),
                proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount);
            return;
        }

        if (!angularDriveOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab angular drive failed; release queued: proxyBody={} objBody={} constraint={} substep={}/{}",
                handName(),
                proxyBodyId.value,
                _savedObjectState.bodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
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
        if (!_grabAuthorityProxy.isValid() || _grabAuthorityProxyHknpWorld != world || !_hasLastAppliedGrabAuthorityProxyWorld) {
            return false;
        }

        out.appliedProxyTargetWorld = _lastAppliedGrabAuthorityProxyWorld;
        out.appliedRawHandWorld = _lastAppliedGrabAuthorityRawHandWorld;
        out.proxyBodyId = _grabAuthorityProxy.getBodyId();
        out.objectBodyId = _savedObjectState.bodyId;
        out.flushSequence = _grabAuthorityProxyFlushSequence;
        return true;
    }

    void Hand::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world) {
            return;
        }

        // Post-solve object velocity for release momentum. It is sampled under
        // the proxy mutex so release never reads a half-written sample.
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        const bool hasAuthority = _grabAuthorityProxy.isValid() &&
                                  _grabAuthorityProxyHknpWorld == world &&
                                  _hasLastAppliedGrabAuthorityProxyWorld;
        _releaseObjectSample = {};
        if (!hasAuthority || !timing.valid || timing.usedFallback) {
            return;
        }

        const auto* body = havok_runtime::getBody(world, _savedObjectState.bodyId);
        const auto* motion = body && body_frame::hasUsableMotionIndex(body->motionIndex) ?
            havok_runtime::getMotion(world, body->motionIndex) : nullptr;
        if (motion) {
            _releaseObjectSample = { world, _savedObjectState.bodyId.value, _grabFrame.traceId,
                timing.solveSequence, release_velocity::sampleTimeSeconds(),
                { motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z } };
        }
    }

    GrabReleaseOutcome Hand::releaseGrabbedObject(
        RE::hknpWorld* world,
        GrabReleaseCollisionRestoreMode collisionRestoreMode,
        const GrabReleaseContext& releaseContext,
        const std::source_location& caller)
    {
        GrabReleaseOutcome outcome{};
        outcome.finalObjectRelease = releaseContext.finalObjectRelease;
        if (!isHolding()) {
            return outcome;
        }

        if (releaseContext.finalObjectRelease && _heldObjectIsLooseWeapon && world) {
            auto* physics = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            if (physics && !physics->prepareLooseWeaponRelease(getHeldRef(), world,
                releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop)) {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000, "Weapon release waiting for native collision restoration");
                return outcome;
            }
        }

        held_scene_presentation::clearHeldBodies(_isLeft);

        outcome.released = true;
        outcome.retainedRef = _savedObjectState.retainedRef;
        outcome.refr = outcome.retainedRef.get();
        outcome.formID = outcome.refr ? outcome.refr->GetFormID() : 0;


        const std::string_view callerPath{ caller.file_name() };
        const auto separator = callerPath.find_last_of("/\\");
        const auto callerFile = separator == std::string_view::npos ? callerPath : callerPath.substr(separator + 1);
        ROCK_LOG_INFO(Hand,
            "{} hand RELEASE: bodyId={} constraintId={} proxyBody={} finalObjectRelease={} disposition={} reason={} grab={} caller={}:{}",
            handName(),
            _savedObjectState.bodyId.value,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            releaseContext.finalObjectRelease ? "yes" : "no",
            releaseDispositionName(releaseContext.disposition),
            releaseContext.reason ? releaseContext.reason : "none",
            _grabFrame.traceId, callerFile, caller.line());


        const auto releaseTime = runtime_state::currentFrame().timing.elapsedGameSeconds;
        const auto objectHistory = _objectReleaseHistory.peaks(release_velocity::sampleTimeSeconds());
        const auto handHistory = _controllerReleaseHistory.peaks(releaseTime);
        const bool captureReleaseVelocity =
            !grab_target::isRagdoll(_savedObjectState.targetKind) &&
            (releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop ||
                releaseContext.disposition == GrabReleaseDisposition::PendingInventoryTransfer ||
                releaseContext.disposition == GrabReleaseDisposition::PendingConsumeTransfer) &&
            releaseContext.applyCapturedReleaseVelocity &&
            releaseContext.finalObjectRelease && world && (objectHistory.count > 0 || handHistory.count > 0);
        if (captureReleaseVelocity) {
            const RE::NiPoint3 objectLocalReleaseVelocity = objectHistory.linear;
            const RE::NiPoint3 handLocalReleaseVelocity = handHistory.linear;
            const RE::NiPoint3 handAngularVelocity = handHistory.angular;

            RE::NiPoint3 releaseLeverOriginHavok = _lastHeldHandPositionHavok;
            RE::NiTransform releaseBodyWorld{};
            bool hasReleaseBodyWorld = false;
            if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, releaseBodyWorld)) {
                hasReleaseBodyWorld = true;
                releaseLeverOriginHavok =
                    gamePointToHavokPoint(transform_math::localPointToWorld(releaseBodyWorld, activeProxyConstraintPivotBLocalGame()));
            }

            // Both histories contain world-space movement, including player
            // translation. Do not rotate the velocity by the hand's facing.
            const RE::NiPoint3 releaseVelocity =
                grab_held_response::composeControllerReleaseVelocity(grab_held_response::ReleaseVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandLocalVelocity = handHistory.count > 0,
                    .hasObjectLocalVelocity = objectHistory.count > 0,
                    .handLocalVelocityHavok = handLocalReleaseVelocity,
                    .objectLocalVelocityHavok = objectLocalReleaseVelocity,
                    .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                    .maxVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                });
            const RE::NiPoint3 rawReleaseAngularVelocity =
                grab_held_response::composeControllerReleaseAngularVelocity(grab_held_response::ReleaseAngularVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandAngularVelocity = handHistory.count > 0,
                    .handAngularVelocityRadiansPerSecond = handAngularVelocity,
                    .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                    .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                });
            RE::NiPoint3 releaseAngularVelocity = rawReleaseAngularVelocity;
            const auto releaseContactSnapshot = readHeldBodyContactSnapshot();
            const bool releaseContactSoftening =
                releaseContactSnapshot.recent &&
                classifyHeldContactOtherMotion(world, releaseContactSnapshot.otherBodyId) != held_object_contact_policy::HeldContactOtherMotion::Dynamic;
            const auto releaseAuthority = grab_motion_controller::evaluateHeldAuthority(releaseContactSoftening);
            const float releaseLongObjectAngularScale = grab_motion_controller::computeLongObjectAngularSpeedScale(
                g_rockConfig.rockGrabLongObjectAngularScalingEnabled,
                _grabFrame.pivotAuthority.longLeverGameUnits,
                g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
                g_rockConfig.rockGrabLongObjectMinAngularScale);
            const float releaseAngularVelocityCap = grab_motion_controller::computeAuthorityScaledAngularVelocityCap(
                g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                releaseAuthority.releaseAngularVelocityScale,
                releaseLongObjectAngularScale);
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
                "{} hand RELEASE VELOCITY: applied={} driveMode={} linearScope={} angularScope={} authority={} angularScale={:.2f} angularCap={:.3f} longScale={:.2f} handLocal=({:.3f},{:.3f},{:.3f}) objectLocal=({:.3f},{:.3f},{:.3f}) angularRaw=({:.3f},{:.3f},{:.3f}) angularFinal=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) lever=({:.3f},{:.3f},{:.3f}) objectHistory={} handHistory={} multiplier={:.2f}",
                handName(),
                applyReleaseVelocity ? "yes" : "no",
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                releaseAuthority.reason,
                releaseAuthority.releaseAngularVelocityScale,
                releaseAngularVelocityCap,
                releaseLongObjectAngularScale,
                handLocalReleaseVelocity.x,
                handLocalReleaseVelocity.y,
                handLocalReleaseVelocity.z,
                objectLocalReleaseVelocity.x,
                objectLocalReleaseVelocity.y,
                objectLocalReleaseVelocity.z,
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
                objectHistory.count,
                handHistory.count,
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
                _savedObjectState.targetKind,
                heldBodyFlagLeaseOwner(this),
                grab_target::isRagdoll(_savedObjectState.targetKind) || releaseContext.finalObjectRelease);
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

            // A pending pickup still owns a loose world object. Wake it even
            // without throw history, so rejection leaves an ordinary drop.
            if (releaseContext.finalObjectRelease && shouldActivateReleasedBodies(releaseContext.disposition)) {
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
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            if (_activeConstraint.isValid()) {
                destroyGrabConstraint(world, _activeConstraint);
            }
            destroyGrabAuthorityProxyLocked(nullptr);
        }

        const bool delayRestore = collisionRestoreMode == GrabReleaseCollisionRestoreMode::Delayed &&
                                  _grabHandCollisionSuppression.beginDelayedRestore(
                                      g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        restoreBodyCollisionAfterHeldLooseWeapon(world);
        if (delayRestore) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand: grab hand collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                handName(),
                _grabHandCollisionSuppression.size(),
                _grabHandCollisionSuppression.firstBodyId(),
                _grabHandCollisionSuppression.delayedRestoreRemainingSeconds());
        } else {
            restoreHandCollisionAfterGrab(world);
        }

        beginGrabVisualReturn();
        (void)frik_visual_authority::clearHandPose("ROCK_Grab", handFromBool(_isLeft));
        clearGrabExternalHandWorldTransform(_isLeft);
        clearSelectedCloseFingerPose();
        _ragdollGrabOwner.reset();
        _ragdollGrabSurface.reset();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _activeConstraint.clear();
        clearGrabAuthorityProxyRuntime();
        _heldBodyIds.clear();
        _grabFrame.clear();
        _grabOffsetAcquisition = {};
        _looseSharedAcquisition = {};
        _looseSharedAcquisitionPeer = 0;
        _looseSharedAcquisitionFrame = 0;
        _looseWeaponReturnStartFrame = 0;
        _looseSharedHandBlendAlpha = -1.0f;
        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        _grabObjectGripAtGrab = {};
        _heldDriveDecision = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabConvergeStableInsidePocketSeconds = 0.0f;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
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
        _grabFingerLocalTransformFinalizePending = false;
        _hasGrabFingerPose = false;
        _controllerReleaseHistory.clear();
        _objectReleaseHistory.clear();
        _releaseObjectNotBefore = release_velocity::sampleTimeSeconds();
        _releaseControllerFrame = 0;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
        _currentSelection.clear();
        clearGrabAcquisitionCache("release-cleared-selection");
        HandInteractionEvent releaseEvent = HandInteractionEvent::ReleaseRequested;
        if (releaseContext.disposition == GrabReleaseDisposition::TransferToInventory && _state == HandState::StashCandidate) {
            releaseEvent = HandInteractionEvent::CommitStash;
        }
        applyTransition(HandTransitionRequest{ .event = releaseEvent });

        ROCK_LOG_DEBUG(Hand, "{} hand: Idle", handName());
        return outcome;
    }
}

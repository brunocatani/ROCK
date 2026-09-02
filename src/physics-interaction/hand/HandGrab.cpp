#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"
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

        /*
         * Per-grab identity for the held scene presentation registration and
         * its target transport history. Zero means no grab.
         */
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

        active_grab_body_lifecycle::BodyReleaseIntent releaseIntentFromDisposition(GrabReleaseDisposition disposition) noexcept
        {
            using active_grab_body_lifecycle::BodyReleaseIntent;
            switch (disposition) {
            case GrabReleaseDisposition::PhysicalDrop:
                return BodyReleaseIntent::PhysicalDrop;
            case GrabReleaseDisposition::OwnershipHandoff:
                return BodyReleaseIntent::OwnershipHandoff;
            case GrabReleaseDisposition::PendingInventoryTransfer:
            case GrabReleaseDisposition::TransferToInventory:
            case GrabReleaseDisposition::PendingConsumeTransfer:
                return BodyReleaseIntent::NonPhysicalTransfer;
            }
            return BodyReleaseIntent::NonPhysicalTransfer;
        }

        RE::NiPoint3 getMatrixColumn(const RE::NiMatrix3& matrix, int column) { return RE::NiPoint3(matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column]); }

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

        RE::NiPoint3 projectOntoPlane(const RE::NiPoint3& value, const RE::NiPoint3& normal)
        {
            const RE::NiPoint3 normalizedNormal = normalizeOrZero(normal);
            if (lengthSquared(normalizedNormal) <= 0.000001f) {
                return {};
            }
            return value - scalePoint(normalizedNormal, dotProduct(value, normalizedNormal));
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

            using ComputeHardKeyFrame_t = void (*)(RE::hknpWorld*, RE::hknpBodyId, float*, float*, float, float*, float*);
            static REL::Relocation<ComputeHardKeyFrame_t> compute{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            compute(world, bodyId, targetPositionHavok, targetRotationHavok, deltaTime, outLinearVelocityHavok, outAngularVelocityRadians);

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

        float grabAngularToLinearForceRatio(bool looseWeaponGrab)
        {
            return looseWeaponGrab ?
                looseWeaponMultiplier(true, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier) :
                grab_motion_controller::kGrabAngularToLinearForceRatio;
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
            const auto uniqueMotionRecords = preparedBodySet.uniqueAcceptedMotionRecords();
            return held_object_drive_policy::evaluateHeldBodySetDrive(held_object_drive_policy::HeldBodySetDriveInput{
                .acceptedBodyCount = static_cast<std::uint32_t>(preparedBodySet.acceptedCount()),
                .uniqueMotionCount = static_cast<std::uint32_t>(uniqueMotionRecords.size()),
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
            const float linearRecoveryMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
            const float angularRecoveryMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);

            const float linearBudget = (std::max)(0.0f, scaleDriveValue(maxForce, maxForceMultiplier));
            const float sanitizedAuthorityForceScale =
                std::clamp(std::isfinite(authorityForceScale) && authorityForceScale > 0.0f ? authorityForceScale : 1.0f, 0.05f, 1.0f);
            const float linearMaxForce =
                grab_motion_controller::capForceByMass(linearBudget, mass, forceToMassRatio) * sanitizedAuthorityForceScale;
            const float angularMaxForce = linearMaxForce * grabAngularToLinearForceRatio(looseWeaponGrab);

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
            return sharedContext.hasPeerState() && selection.refr && sharedContext.peerSavedObjectState->refr == selection.refr;
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

        frik_visual_authority::HandPoseKind looseWeaponPrimaryAttachPoseKind(const RE::TESObjectWEAP* weapon)
        {
            return weapon && weapon_type_policy::isMelee(weapon->weaponData.type.get()) ?
                       frik_visual_authority::HandPoseKind::HoldingMelee :
                       frik_visual_authority::HandPoseKind::HoldingGun;
        }

        bool publishLooseWeaponPrimaryAttachHandPose(bool isLeft, RE::TESObjectREFR* refr)
        {
            const auto* weapon = looseWeaponFormFromRef(refr);
            auto* weaponRoot = refr ? refr->Get3D() : nullptr;
            if (weapon && weaponRoot) {
                const auto authored = authored_weapon_grip_library::find(weapon, weaponRoot, f4vr::isInPowerArmor());
                if (authored.found && authored.rightFiringFingerPose.complete()) {
                    frik_visual_authority::FingerLocalTransformOverride exactRightPose{};
                    exactRightPose.enabledMask = authored.rightFiringFingerPose.enabledMask;
                    for (std::size_t index = 0; index < authored.rightFiringFingerPose.localTransforms.size(); ++index) {
                        exactRightPose.localTransforms[index] = authored.rightFiringFingerPose.localTransforms[index];
                    }

                    frik_visual_authority::FingerLocalTransformOverride exactPose = exactRightPose;
                    const bool exactPoseReady =
                        !isLeft ||
                        hand_finger_mirror_math::mirrorFingerLocalsAcrossHands<RE::NiTransform>(
                            std::span<const RE::NiTransform>(exactRightPose.localTransforms),
                            std::span<RE::NiTransform>(exactPose.localTransforms));
                    if (exactPoseReady) {
                        constexpr const char* tag = "ROCK_Grab";
                        constexpr int priority = 100;
                        const char* blockTag = isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight";
                        const bool blockedNativePose = frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, true);
                        const bool scalarPublished =
                            blockedNativePose && frik_visual_authority::setHandPoseCustomWithPriority(tag, handFromBool(isLeft), frik_visual_authority::HandPoseData{}, priority);
                        const bool localsPublished =
                            scalarPublished && frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(tag, handFromBool(isLeft), &exactPose, priority);
                        if (localsPublished) {
                            ROCK_LOG_INFO(Hand, "{} hand loose weapon attach: applying exact native-idle firing pose source={} mask=0x{:04X}", isLeft ? "left" : "right",
                                authored.reason, exactPose.enabledMask);
                            return true;
                        }

                        (void)frik_visual_authority::clearHandPose(tag, handFromBool(isLeft));
                        if (blockedNativePose) {
                            (void)frik_visual_authority::blockPrimaryHandWeaponPose(blockTag, false);
                        }
                    }
                }
            }

            return frik_visual_authority::setHandPoseWithPriority(
                "ROCK_Grab",
                handFromBool(isLeft),
                looseWeaponPrimaryAttachPoseKind(weapon),
                100);
        }

        struct LooseWeaponPrimaryAttachFrame
        {
            bool valid = false;
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
            if (formRef.empty()) {
                return false;
            }
            saved_grab_offset::SavedGrabOffsetFile file{};
            if (!saved_grab_offset::load(formRef, file, nullptr)) {
                return false;
            }
            const auto& handOffset = isLeft ? file.left : file.right;
            if (!handOffset.present) {
                return false;
            }
            out = handOffset;
            return true;
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
            const RE::NiTransform& handWorldAtGrab,
            bool savedGrabOffsetAttachValid,
            const RE::NiTransform& savedGrabOffsetRootWorld)
        {
            LooseWeaponPrimaryAttachFrame frame{};
            /*
             * A close grab is a free mesh hold on either hand regardless of
             * source; the firing-grip transition happens later through the
             * grip-zone equip path (loose_weapon_grip_zone), not by forcing
             * the attach at grab.
             */
            if (!grabbedFromPullCatch && !selection.forcedArrival) {
                frame.reason = "closeGrabFreeHold";
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
                 * Only non-throwable programmatic loose-weapon arrivals snap to a
                 * canonical attach pose. Grenades, mines, and Molotov variants are
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

                /*
                 * Both firing hands use the same weapon-relative authority resolver.
                 * It enforces custom hFRIK > learned authored > embedded hFRIK and
                 * performs no filesystem work on this grab path. Explicit hFRIK
                 * keeps its complete correction. Authored loose placement always
                 * derives from the native carrier, so a prior equip cannot replace
                 * the first-grab weapon orientation.
                 */
                RE::NiTransform handWorld{};
                RE::NiTransform handWeaponLocal{};
                const char* holdReason = "canonicalHoldUnavailable";
                const bool haveDesiredRoot = loose_weapon_grip_zone::tryResolveLooseWeaponFiringHandHold(
                    isLeft,
                    selection.refr,
                    handWorld,
                    handWeaponLocal,
                    &holdReason);
                if (haveDesiredRoot) {
                    frame.desiredRootWorld = multiplyTransforms(
                        handWorld,
                        transform_math::invertTransform(handWeaponLocal));
                    frame.sourceVisible = false;
                    frame.reason = holdReason;
                } else if (!selection.forcedArrival) {
                    frame.reason = holdReason;
                    return frame;
                }

                if (!haveDesiredRoot) {
                    /*
                     * Palm-anchored fallback for non-throwable forced arrivals
                     * without a usable FRIK offset: root axes follow the live hand
                     * basis and the root origin sits on the hand grab pivot. Any
                     * fixed choice is correct here -- the goal is a deterministic
                     * commit pose, not a per-weapon tuned grip.
                     */
                    if (!isFiniteNiTransform(handWorldAtGrab)) {
                        frame.reason = "nonFiniteHandWorld";
                        return frame;
                    }
                    frame.desiredRootWorld.rotate = handWorldAtGrab.rotate;
                    frame.desiredRootWorld.translate = grabPivotAWorld;
                    frame.sourceVisible = false;
                    frame.reason = "forcedArrivalPalmPose";
                }
            }

            frame.desiredRootWorld.scale =
                rootNode && std::isfinite(rootNode->world.scale) && rootNode->world.scale > 0.0001f ? rootNode->world.scale : 1.0f;
            if (!isFiniteNiTransform(frame.desiredRootWorld)) {
                frame.reason = "nonFiniteDesiredRoot";
                return frame;
            }

            frame.desiredBodyWorld = multiplyTransforms(frame.desiredRootWorld, rootBodyLocalAtGrab);
            frame.desiredObjectWorld = deriveNodeWorldFromBodyWorld(frame.desiredBodyWorld, objectToBodyAtGrab);
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

            for (auto* current = node; current; current = current->parent) {
                if (current == root) {
                    return true;
                }
            }
            return false;
        }

        bool acceptsSelectedMultibodyOwnerlessVisualMesh(const SelectedObject& selection,
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            std::uint32_t resolvedBodyId,
            RE::NiAVObject* surfaceOwnerNode,
            const object_physics_body_set::ObjectPhysicsBodyRecord* surfaceOwnerRecord)
        {
            /*
             * Multipart refs can expose visible geometry and hknp collision
             * owners as sibling nodes under the same selected reference. When
             * the visible mesh has no accepted collision owner record, the
             * selected body remains the acquisition authority; a concrete
             * mismatched owner record still fails closed.
             */
            return selection.refr == bodySet.rootRef &&
                   bodySet.acceptedCount() > 1 &&
                   resolvedBodyId != object_physics_body_set::INVALID_BODY_ID &&
                   resolvedBodyId == selection.bodyId.value &&
                   bodySet.containsAcceptedBody(selection.bodyId.value) &&
                   surfaceOwnerNode &&
                   !surfaceOwnerRecord &&
                   nodeIsOrDescendsFrom(bodySet.rootNode, surfaceOwnerNode);
        }

        constexpr const char* kHeldObjectDriveName = "proxyConstraint";
        constexpr std::uint32_t kHeldCollisionParticipationFlags = 0x80u;
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
            std::uintptr_t ownerToken)
        {
            /*
             * Proxy-constraint grab replaced the native held-object action, but the
             * old path owned two different flag contracts: 0x80 was leased across
             * the accepted held body set by ROCK, while the native action leased
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
                        kHeldCollisionParticipationFlags,
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
                        kHeldCollisionParticipationFlags,
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
            bool valid = false;
        };

        grab_pinch_pocket_policy::Config currentPinchPocketConfig()
        {
            return grab_pinch_pocket_policy::sanitizeConfig(grab_pinch_pocket_policy::Config{
                .enabled = g_rockConfig.rockGrabPinchPocketEnabled,
                .compactMaxExtentGameUnits = g_rockConfig.rockGrabPinchCompactMaxExtentGameUnits,
                .thinRodMaxLengthGameUnits = g_rockConfig.rockGrabPinchThinRodMaxLengthGameUnits,
                .thinRodMaxCrossSectionGameUnits = g_rockConfig.rockGrabPinchThinRodMaxCrossSectionGameUnits,
                .maxPocketDistanceGameUnits = g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits,
                .minFingerGapGameUnits = g_rockConfig.rockGrabPinchMinFingerGapGameUnits,
                .maxFingerGapGameUnits = g_rockConfig.rockGrabPinchMaxFingerGapGameUnits,
                .thumbIndexMaxOpenValue = g_rockConfig.rockGrabPinchThumbIndexMaxOpenValue,
                .otherFingerCurlValue = g_rockConfig.rockGrabPinchOtherFingerCurlValue,
                .surfaceInsetGameUnits = g_rockConfig.rockGrabPinchSurfaceInsetGameUnits,
                .detectionDirectionHandspace = g_rockConfig.rockGrabPinchDetectionDirectionHandspace,
                .detectionAxisBlend = g_rockConfig.rockGrabPinchDetectionAxisBlend,
            });
        }

        RE::NiPoint3 pinchPadPointFromSnapshot(const root_flattened_finger_skeleton_runtime::FingerChain& chain)
        {
            return chain.points[2];
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

        RuntimePinchPocketCandidate buildRuntimePinchPocketCandidate(
            const SelectedObject& selection,
            const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
            std::uint32_t resolvedBodyId,
            const RE::NiTransform& objectWorldTransform,
            const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
            const std::vector<GrabLocalTriangle>& localMeshTriangles,
            const RE::NiPoint3& currentObjectPointWorld,
            const RE::NiTransform& handWorldTransform,
            bool isLeft,
            bool closeGrab,
            bool handPocketOnlyGrab,
            bool looseWeaponGrab)
        {
            RuntimePinchPocketCandidate candidate{};
            const auto config = currentPinchPocketConfig();
            const float objectScale =
                std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
            candidate.meshExtents = grab_pinch_pocket_policy::computeMeshExtents(localMeshTriangles, objectScale);

            root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
            const bool hasFingerSnapshot =
                root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, fingerSnapshot) &&
                fingerSnapshot.valid &&
                fingerSnapshot.fingers[0].valid &&
                fingerSnapshot.fingers[1].valid;

            bool hasPinchSurface = false;
            bool ownerMatchesResolvedBody = false;
            if (hasFingerSnapshot && !surfaceTriangles.empty()) {
                candidate.thumbPadWorld = pinchPadPointFromSnapshot(fingerSnapshot.fingers[0]);
                candidate.indexPadWorld = pinchPadPointFromSnapshot(fingerSnapshot.fingers[1]);
                candidate.thumbIndexGapGameUnits =
                    grab_pinch_pocket_policy::distance(candidate.thumbPadWorld, candidate.indexPadWorld);
                candidate.pinchAxisWorld =
                    grab_pinch_pocket_policy::normalizeOrFallback(candidate.indexPadWorld - candidate.thumbPadWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                candidate.pinchPocketWorld =
                    grab_pinch_pocket_policy::closestPointOnSegment(candidate.thumbPadWorld, candidate.indexPadWorld, currentObjectPointWorld);
                const RE::NiPoint3 configuredDetectionWorld =
                    transformHandspaceDirection(handWorldTransform, config.detectionDirectionHandspace, isLeft);
                const RE::NiPoint3 configuredDetectionNormal =
                    grab_pinch_pocket_policy::normalizeOrFallback(configuredDetectionWorld, candidate.pinchAxisWorld);
                candidate.pinchDetectionDirectionWorld =
                    grab_pinch_pocket_policy::normalizeOrFallback(candidate.pinchAxisWorld * config.detectionAxisBlend +
                                                                      configuredDetectionNormal * (1.0f - config.detectionAxisBlend),
                        candidate.pinchAxisWorld);

                GrabSurfaceHit surfaceHit{};
                hasPinchSurface = findClosestGrabSurfaceHitToPointPositionOnly(
                    surfaceTriangles,
                    candidate.pinchPocketWorld,
                    candidate.pinchDetectionDirectionWorld,
                    config.maxPocketDistanceGameUnits,
                    surfaceHit);
                if (hasPinchSurface) {
                    candidate.surfaceHit = surfaceHit;
                    candidate.pocketToSurfaceDistanceGameUnits =
                        grab_pinch_pocket_policy::distance(candidate.pinchPocketWorld, surfaceHit.position);
                    if (surfaceHit.sourceNode) {
                        const auto* ownerRecord = bodySet.findAcceptedRecordByOwnerNode(surfaceHit.sourceNode);
                        ownerMatchesResolvedBody =
                            (ownerRecord && ownerRecord->bodyId == resolvedBodyId) ||
                            acceptsSelectedMultibodyOwnerlessVisualMesh(selection,
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
            return candidate;
        }

        grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimePinchFingerPoseTargets(
            const RuntimePinchPocketCandidate& candidate)
        {
            auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(candidate.surfaceHit.position, candidate.surfaceHit.normal);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = false;

            const auto config = currentPinchPocketConfig();
            const float halfWidth =
                grab_pinch_pocket_policy::oppositionHalfWidthGameUnits(candidate.meshExtents, config.surfaceInsetGameUnits);
            const RE::NiPoint3 thumbNormal{
                -candidate.pinchAxisWorld.x,
                -candidate.pinchAxisWorld.y,
                -candidate.pinchAxisWorld.z,
            };
            const RE::NiPoint3 indexNormal = candidate.pinchAxisWorld;

            targets.targets[0] = candidate.surfaceHit.position + thumbNormal * halfWidth;
            targets.targetNormals[0] = thumbNormal;
            targets.targetValid[0] = 1;
            targets.targetNormalValid[0] = 1;
            targets.targets[1] = candidate.surfaceHit.position + indexNormal * halfWidth;
            targets.targetNormals[1] = indexNormal;
            targets.targetValid[1] = 1;
            targets.targetNormalValid[1] = 1;
            targets.targetCount = 2;
            return targets;
        }

        void applyPinchFingerPosePolicy(
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const CanonicalGrabFrame& frame,
            float minFingerValue)
        {
            const auto config = currentPinchPocketConfig();
            const auto stablePose = grab_pinch_pocket_policy::buildStablePinchFingerPose(config, minFingerValue);

            pose.values = stablePose.values;
            pose.usedAlternateThumbCurve = false;
            pose.usedAlternateThumbSurfaceHit = false;
            pose.selectedThumbLane = grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
            pose.selectedThumbLaneNormalBlend = 0.0f;
            pose.selectedThumbLaneLocalCorrectionStrength = 0.0f;
            pose.hasThumbAlternateCurveFrame = false;
            pose.thumbAlternateCurveBaseWorld = {};
            pose.thumbAlternateCurveOpenDirectionWorld = {};
            pose.thumbAlternateCurveNormalWorld = {};
            pose.thumbAlternateCurveMaxCurlAngleRadians = 0.0f;
            pose.hasThumbCurveDiagnostics = false;
            pose.thumbPrimaryCurve = {};
            pose.thumbAlternateCurve = {};
            pose.thumbSidePadCurve = {};
            pose.poseTargetCount = (std::max)(pose.poseTargetCount, static_cast<int>(frame.fingerPoseTargetCount));

            for (std::size_t finger = 0; finger < 2 && finger < pose.surfaceAimTargetValid.size(); ++finger) {
                pose.surfaceAimTargetValid[finger] = 0;
                pose.surfaceAimNormalValid[finger] = 0;
            }

            pose.solved = true;
            pose.hasJointValues = true;
            pose.jointValues = stablePose.jointValues;
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

        void refreshGrabCaptureNodeTransform(GrabCaptureTransformRefreshResult& result, const char* role, RE::NiAVObject* node)
        {
            if (!node || result.count >= result.samples.size() || grabCaptureRefreshAlreadyVisited(result, node)) {
                return;
            }

            auto& sample = result.samples[result.count++];
            sample.role = role ? role : "unknown";
            sample.node = node;

            const RE::NiTransform before = node->world;
            sample.validBefore = grab_three_phase::isFinite(before);

            RE::NiUpdateData update{};
            node->UpdateTransforms(update);

            const RE::NiTransform after = node->world;
            sample.validAfter = grab_three_phase::isFinite(after);
            sample.positionDeltaGameUnits = translationDeltaGameUnits(before, after);
            sample.rotationDeltaDegrees = rotationDeltaDegrees(before.rotate, after.rotate);
            result.ok = result.ok && sample.validAfter;
        }

        GrabCaptureTransformRefreshResult refreshGrabCaptureTransforms(
            RE::NiAVObject* rootNode,
            RE::NiAVObject* meshSourceNode,
            RE::NiAVObject* collidableNode)
        {
            GrabCaptureTransformRefreshResult result{};
            refreshGrabCaptureNodeTransform(result, "root", rootNode);
            refreshGrabCaptureNodeTransform(result, "mesh", meshSourceNode);
            refreshGrabCaptureNodeTransform(result, "collidable", collidableNode);
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

        RE::NiTransform makeIdentityTransform()
        {
            return transform_math::makeIdentityTransform<RE::NiTransform>();
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

        // Pull flight: cached-mesh arrival scan bound and the pocket margin that
        // absorbs one frame of hand motion between arrival and the capture gate.
        constexpr std::size_t kMaxPullArrivalTriangles = 4096;
        constexpr float kPullArrivalPocketMarginGameUnits = 1.5f;
        // Close press whose selection hit is this far outside the pocket radius
        // starts the pull without running the grab capture first.
        constexpr float kCloseSelectionPullPreCheckMarginGameUnits = 3.0f;

        std::vector<GrabLocalTriangle> cacheBoundedTrianglesInLocalSpace(
            const std::vector<TriangleData>& worldTriangles,
            const RE::NiTransform& nodeWorld,
            std::size_t maxTriangles)
        {
            std::vector<GrabLocalTriangle> localTriangles;
            if (worldTriangles.empty() || maxTriangles == 0) {
                return localTriangles;
            }
            // Uniform stride subsample keeps the per-frame arrival scan bounded.
            const std::size_t stride = (worldTriangles.size() + maxTriangles - 1) / maxTriangles;
            localTriangles.reserve((worldTriangles.size() + stride - 1) / stride);
            for (std::size_t i = 0; i < worldTriangles.size(); i += stride) {
                const auto& triangle = worldTriangles[i];
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

        float triangleDistanceSquaredToPoint(const TriangleData& triangle, const RE::NiPoint3& point)
        {
            const RE::NiPoint3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
            return (std::min)({
                lengthSquared(centroid - point),
                lengthSquared(triangle.v0 - point),
                lengthSquared(triangle.v1 - point),
                lengthSquared(triangle.v2 - point),
            });
        }

        struct RankedGrabTriangle
        {
            float distanceSquared = 0.0f;
            std::size_t index = 0;
        };

        bool rankedGrabTriangleLess(const RankedGrabTriangle& lhs, const RankedGrabTriangle& rhs)
        {
            if (lhs.distanceSquared == rhs.distanceSquared) {
                return lhs.index < rhs.index;
            }
            return lhs.distanceSquared < rhs.distanceSquared;
        }

        std::vector<GrabSurfaceTriangleData> selectNearestGrabSurfaceTriangles(
            const std::vector<GrabSurfaceTriangleData>& sourceTriangles,
            const RE::NiPoint3& centerWorld,
            std::size_t maxTriangles)
        {
            if (sourceTriangles.size() <= maxTriangles || maxTriangles == 0 || !grab_three_phase::isFinite(centerWorld)) {
                return sourceTriangles;
            }

            std::vector<RankedGrabTriangle> rankedTriangles;
            rankedTriangles.reserve(sourceTriangles.size());
            for (std::size_t i = 0; i < sourceTriangles.size(); ++i) {
                rankedTriangles.push_back(RankedGrabTriangle{
                    triangleDistanceSquaredToPoint(sourceTriangles[i].triangle, centerWorld),
                    i,
                });
            }

            const auto selectedEnd = rankedTriangles.begin() + maxTriangles;
            std::nth_element(rankedTriangles.begin(), selectedEnd, rankedTriangles.end(), rankedGrabTriangleLess);
            std::sort(rankedTriangles.begin(), selectedEnd, rankedGrabTriangleLess);

            std::vector<GrabSurfaceTriangleData> selectedTriangles;
            selectedTriangles.reserve(maxTriangles);
            for (auto it = rankedTriangles.begin(); it != selectedEnd; ++it) {
                selectedTriangles.push_back(sourceTriangles[it->index]);
            }
            return selectedTriangles;
        }

        std::vector<TriangleData> selectNearestGrabFingerPoseTriangles(
            const std::vector<TriangleData>& sourceTriangles,
            const RE::NiPoint3& centerWorld,
            std::size_t maxTriangles)
        {
            if (sourceTriangles.size() <= maxTriangles || maxTriangles == 0 || !grab_three_phase::isFinite(centerWorld)) {
                return sourceTriangles;
            }

            std::vector<RankedGrabTriangle> rankedTriangles;
            rankedTriangles.reserve(sourceTriangles.size());
            for (std::size_t i = 0; i < sourceTriangles.size(); ++i) {
                rankedTriangles.push_back(RankedGrabTriangle{
                    triangleDistanceSquaredToPoint(sourceTriangles[i], centerWorld),
                    i,
                });
            }

            const auto selectedEnd = rankedTriangles.begin() + maxTriangles;
            std::nth_element(rankedTriangles.begin(), selectedEnd, rankedTriangles.end(), rankedGrabTriangleLess);
            std::sort(rankedTriangles.begin(), selectedEnd, rankedGrabTriangleLess);

            std::vector<TriangleData> selectedTriangles;
            selectedTriangles.reserve(maxTriangles);
            for (auto it = rankedTriangles.begin(); it != selectedEnd; ++it) {
                selectedTriangles.push_back(sourceTriangles[it->index]);
            }
            return selectedTriangles;
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

        RE::NiTransform getGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryGetGrabAuthorityBodyWorldTransform(world, bodyId, result);
            return result;
        }

        RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld, const RE::NiTransform& bodyWorld)
        {
            return multiplyTransforms(invertTransform(nodeWorld), bodyWorld);
        }

        constexpr const char* kGrabObjectRotationReferenceName = "generatedProxyAuthorityLocal";
        constexpr float kGrabFrameMismatchRawProxyRotationWarnDegrees = 20.0f;
        constexpr float kGrabFrameMismatchProxyRotationWarnDegrees = 5.0f;
        constexpr float kGrabFrameMismatchObjectRotationWarnDegrees = 25.0f;
        constexpr float kGrabFrameMismatchGripErrorWarnGameUnits = 5.0f;

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

        /*
         * Closest point of a cached object-local mesh to a world point, returned
         * in world space. The scan is bounded by the cached triangle count; the
         * pull caches at most kMaxPullArrivalTriangles.
         */
        bool findClosestLocalMeshPointToWorldPoint(
            const std::vector<GrabLocalTriangle>& localTriangles,
            const RE::NiTransform& nodeWorld,
            const RE::NiPoint3& pointWorld,
            RE::NiPoint3& outPointWorld)
        {
            outPointWorld = {};
            if (localTriangles.empty() || !grab_three_phase::isFinite(nodeWorld) || !grab_three_phase::isFinite(pointWorld)) {
                return false;
            }

            const RE::NiPoint3 pointLocal = transform_math::worldPointToLocal(nodeWorld, pointWorld);
            float bestDistanceSquared = std::numeric_limits<float>::max();
            RE::NiPoint3 bestPointLocal{};
            bool found = false;
            for (const auto& localTriangle : localTriangles) {
                TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
                float distanceSquared = 0.0f;
                const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(pointLocal, triangle, distanceSquared);
                if (!std::isfinite(distanceSquared) || distanceSquared >= bestDistanceSquared) {
                    continue;
                }
                bestDistanceSquared = distanceSquared;
                bestPointLocal = candidate;
                found = true;
            }

            if (!found) {
                return false;
            }
            outPointWorld = transform_math::localPointToWorld(nodeWorld, bestPointLocal);
            return grab_three_phase::isFinite(outPointWorld);
        }

        /*
         * Palm seat selection. The one object-side authority of a palm grab
         * is the closest palm-facing surface point to the pocket centre. The
         * scan is bounded by the extracted surface set and runs once at
         * capture. Facing uses the mesh winding; its global sign is voted
         * once against the mesh centroid so a mirrored or inside-out export
         * cannot flip every face, and the per-triangle threshold keeps
         * edge-on faces (plate edges) grabbable. Owner filtering keeps the
         * seat on the resolved body of a multi-body object; a mesh with no
         * acceptable point fails closed.
         */
        constexpr float kPalmSeatMaxFacingDot = 0.3f;

        struct PalmSeatPointSelection
        {
            GrabSurfaceHit hit{};
            std::uint32_t evaluatedCandidates = 0;
            std::uint32_t rejectedFacing = 0;
            std::uint32_t rejectedOwner = 0;
            const char* reason = "noSurfaceTriangles";
            bool valid = false;
        };

        struct PalmSeatOwnerFilter
        {
            const SelectedObject* selection = nullptr;
            const object_physics_body_set::ObjectPhysicsBodySet* bodySet = nullptr;
            std::uint32_t resolvedBodyId = object_physics_body_set::INVALID_BODY_ID;
            bool handPocketOnlyGrab = false;
            bool relaxedArticulatedAuthority = false;
        };

        bool palmSeatOwnerAccepts(const PalmSeatOwnerFilter& filter,
            const GrabSurfaceTriangleData& surfaceTriangle,
            RE::NiAVObject* ownerNode)
        {
            if (!filter.selection || !filter.bodySet ||
                filter.resolvedBodyId == object_physics_body_set::INVALID_BODY_ID ||
                filter.relaxedArticulatedAuthority) {
                return true;
            }
            const auto* ownerRecord = filter.bodySet->findAcceptedRecordByOwnerNode(ownerNode);
            if (ownerRecord) {
                return ownerRecord->bodyId == filter.resolvedBodyId;
            }
            const bool positionOnlySkinnedSurface =
                filter.handPocketOnlyGrab &&
                surfaceTriangle.sourceKind == GrabSurfaceSourceKind::Skinned &&
                !surfaceTriangle.hasSkinInfluences;
            if (positionOnlySkinnedSurface) {
                return filter.resolvedBodyId == filter.selection->bodyId.value &&
                       filter.bodySet->containsAcceptedBody(filter.selection->bodyId.value);
            }
            return acceptsSelectedMultibodyOwnerlessVisualMesh(
                *filter.selection,
                *filter.bodySet,
                filter.resolvedBodyId,
                ownerNode,
                ownerRecord);
        }

        PalmSeatPointSelection selectPalmSeatPoint(
            const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
            const grab_three_phase::GrabPocketFrame& pocket,
            const PalmSeatOwnerFilter& ownerFilter)
        {
            PalmSeatPointSelection result{};
            if (surfaceTriangles.empty() || !pocket.valid) {
                result.reason = surfaceTriangles.empty() ? "noSurfaceTriangles" : "invalidPocket";
                return result;
            }

            RE::NiPoint3 centroidSum{};
            for (const auto& surfaceTriangle : surfaceTriangles) {
                const auto& tri = surfaceTriangle.triangle;
                centroidSum = centroidSum + tri.v0 + tri.v1 + tri.v2;
            }
            const RE::NiPoint3 meshCentroid = scalePoint(centroidSum, 1.0f / static_cast<float>(surfaceTriangles.size() * 3));
            // Area-weighted vote: does the winding normal point away from the centroid?
            float windingVote = 0.0f;
            for (const auto& surfaceTriangle : surfaceTriangles) {
                const auto& tri = surfaceTriangle.triangle;
                const RE::NiPoint3 windingNormal = crossProduct(tri.v1 - tri.v0, tri.v2 - tri.v0);
                const RE::NiPoint3 triangleCenter = scalePoint(tri.v0 + tri.v1 + tri.v2, 1.0f / 3.0f);
                windingVote += dotProduct(windingNormal, triangleCenter - meshCentroid);
            }
            const float windingSign = windingVote < 0.0f ? -1.0f : 1.0f;

            const RE::NiPoint3 palmNormal = normalizeOrZero(pocket.palmNormalWorld);
            float bestDistSq = (std::numeric_limits<float>::max)();
            int bestIndex = -1;
            RE::NiPoint3 bestPoint{};
            RE::NiPoint3 bestNormal{};
            RE::NiAVObject* bestOwner = nullptr;
            for (int i = 0; i < static_cast<int>(surfaceTriangles.size()); ++i) {
                const auto& surfaceTriangle = surfaceTriangles[static_cast<std::size_t>(i)];
                const auto& tri = surfaceTriangle.triangle;
                float distSq = 0.0f;
                const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(pocket.pocketCenterWorld, tri, distSq);
                if (distSq >= bestDistSq) {
                    continue;
                }
                ++result.evaluatedCandidates;
                RE::NiPoint3 normal = normalizeOrZero(crossProduct(tri.v1 - tri.v0, tri.v2 - tri.v0));
                if (lengthSquared(normal) <= 0.000001f) {
                    continue;
                }
                normal = scalePoint(normal, windingSign);
                if (dotProduct(normal, palmNormal) > kPalmSeatMaxFacingDot) {
                    ++result.rejectedFacing;
                    continue;
                }
                RE::NiAVObject* ownerNode = resolveDominantSurfaceOwnerNode(surfaceTriangle, candidate);
                if (!palmSeatOwnerAccepts(ownerFilter, surfaceTriangle, ownerNode)) {
                    ++result.rejectedOwner;
                    continue;
                }
                bestDistSq = distSq;
                bestIndex = i;
                bestPoint = candidate;
                bestNormal = normal;
                bestOwner = ownerNode;
            }

            if (bestIndex < 0) {
                result.reason = result.rejectedOwner > 0 ? "noOwnerMatchedPalmFacingPoint" : "noPalmFacingPoint";
                return result;
            }

            const auto& surfaceTriangle = surfaceTriangles[static_cast<std::size_t>(bestIndex)];
            auto& hit = result.hit;
            hit.position = bestPoint;
            hit.normal = bestNormal;
            hit.triangleIndex = bestIndex;
            hit.distance = bestDistSq;
            hit.sourceNode = bestOwner;
            hit.sourceShape = surfaceTriangle.sourceShape;
            hit.triangle = surfaceTriangle.triangle;
            hit.sourceKind = surfaceTriangle.sourceKind;
            hit.hasSkinInfluences = surfaceTriangle.hasSkinInfluences;
            hit.hasTriangle = true;
            const RE::NiPoint3 toCandidate = bestPoint - pocket.palmCenterWorld;
            hit.signedAlongPalmDistanceGameUnits = dotProduct(toCandidate, palmNormal);
            const RE::NiPoint3 lateral = toCandidate - scalePoint(palmNormal, hit.signedAlongPalmDistanceGameUnits);
            hit.lateralPalmDistanceGameUnits = std::sqrt((std::max)(0.0f, lengthSquared(lateral)));
            hit.valid = true;
            result.reason = "closestPalmFacingPoint";
            result.valid = true;
            return result;
        }

        struct GrabSeatDepthStopResult
        {
            float depthGameUnits = 0.0f;
            std::uint32_t footprintSampleCount = 0;
            const char* reason = "notEvaluated";
            bool valid = false;
        };

        /*
         * Seat depth stop: the frozen authority frame seats pivot B exactly onto
         * pivot A, so any mesh that extends past the grip point toward the palm
         * ends up inside the hand. Measure that extent as a support distance:
         * the farthest the cached object-local mesh reaches along -palmNormal
         * from the grip point, counting only geometry inside a lateral footprint
         * around the palm axis (mesh far to the side clears the palm and must
         * not push the seat out). Pushing pivot A out by this distance seats the
         * object's SURFACE on the palm. Triangle vertices alone under-sample
         * coarse meshes (a crate face keeps its vertices at corners, outside the
         * footprint), so each triangle is also probed with closest-point queries
         * against samples along the palm axis.
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

        struct HeldMotionCompensationResult
        {
            RE::NiPoint3 primaryLocalLinearVelocity{};
            bool hasPrimaryVelocity = false;
        };

        HeldMotionCompensationResult applyHeldMotionCompensation(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            bool includeConnectedBodies = true)
        {
            HeldMotionCompensationResult result{};
            if (!world) {
                return result;
            }

            constexpr std::size_t kMaxSampledMotionSlots = 96;
            std::array<std::uint32_t, kMaxSampledMotionSlots> sampledMotionSlots{};
            std::size_t sampledMotionSlotCount = 0;

            auto motionSlotAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                    if (sampledMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto sampleBody = [&](std::uint32_t bodyId) {
                if (bodyId == INVALID_BODY_ID) {
                    return;
                }

                auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
                if (!body) {
                    return;
                }

                const std::uint32_t motionIndex = body->motionIndex;
                if (!body_frame::hasUsableMotionIndex(motionIndex) || motionSlotAlreadySampled(motionIndex)) {
                    return;
                }

                if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                    return;
                }

                auto* motion = havok_runtime::getMotion(world, motionIndex);
                if (!motion) {
                    return;
                }

                sampledMotionSlots[sampledMotionSlotCount++] = motionIndex;

                const RE::NiPoint3 localLinearVelocity{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z };

                if (bodyId == primaryBodyId.value) {
                    result.primaryLocalLinearVelocity = localLinearVelocity;
                    result.hasPrimaryVelocity = true;
                }
            };

            sampleBody(primaryBodyId.value);
            if (includeConnectedBodies) {
                for (const auto bodyId : heldBodyIds) {
                    sampleBody(bodyId);
                }
            }

            return result;
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

        void applyRockGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose, std::array<float, 15>& currentJointPose,
            bool& hasCurrentJointPose, std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask, bool& hasCurrentLocalTransforms,
            float deltaTime, bool publishLocalTransforms = true)
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
                if (!hasCurrentJointPose) {
                    currentJointPose = targetJointPose;
                    hasCurrentJointPose = true;
                } else {
                    currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose, g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
                }

                std::array<float, 5> currentSplayRadians{};
                (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
                const auto currentHandPose = frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose, currentSplayRadians);
                if (!frik_visual_authority::setHandPoseCustomWithPriority("ROCK_Grab", hand, currentHandPose, 100)) {
                    hasCurrentJointPose = false;
                    grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                    syncLocalTransformState();
                    return;
                }
                const bool publishedLocalTransforms = grab_finger_local_transform_runtime::publishLocalTransformPose("ROCK_Grab",
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
                if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand FINGER JOINT POSE: thumb=({:.2f},{:.2f},{:.2f}) index=({:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={} thumbLane={} localTransforms={} mask=0x{:04X}",
                        isLeft ? "Left" : "Right", currentJointPose[0], currentJointPose[1], currentJointPose[2], currentJointPose[3], currentJointPose[4],
                        currentJointPose[5], fingerPose.hitCount, fingerPose.candidateTriangleCount, fingerPose.usedAlternateThumbCurve ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(fingerPose.selectedThumbLane),
                        publishedLocalTransforms ? "yes" : "no", currentLocalTransformMask);
                }
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
                const bool published = frik_visual_authority::setHandPoseCustomWithPriority("ROCK_Grab", hand, handPose, 100);
                if (published && g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand FINGER POSE: mesh values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={} thumbLane={}",
                        isLeft ? "Left" : "Right", fingerPose.values[0], fingerPose.values[1], fingerPose.values[2], fingerPose.values[3], fingerPose.values[4],
                        fingerPose.hitCount, fingerPose.candidateTriangleCount, fingerPose.usedAlternateThumbCurve ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(fingerPose.selectedThumbLane));
                }
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
            (void)frik_visual_authority::setHandPoseCustomWithPriority("ROCK_Grab", hand, fallbackPose, 100);
            if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand FINGER POSE: using selected-close fallback value={:.2f} solved={} hits={} candidateTris={}",
                    isLeft ? "Left" : "Right",
                    fallbackValue,
                    fingerPose.solved ? "yes" : "no",
                    fingerPose.hitCount,
                    fingerPose.candidateTriangleCount);
            }
        }

        constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";
        constexpr int GRAB_EXTERNAL_HAND_PRIORITY = 90;
        constexpr const char* GRAB_RETURN_HAND_TAG = "ROCK_GrabReturn";
        constexpr int GRAB_RETURN_HAND_PRIORITY = 85;

        bool applyGrabExternalHandWorldTransform(bool isLeft, const RE::NiTransform& adjustedHandTransform)
        {
            return frik_visual_authority::applyExternalHandWorldTransform(
                GRAB_EXTERNAL_HAND_TAG,
                handFromBool(isLeft),
                adjustedHandTransform,
                GRAB_EXTERNAL_HAND_PRIORITY);
        }

        void clearGrabExternalHandWorldTransform(bool isLeft)
        {
            (void)frik_visual_authority::clearExternalHandWorldTransform(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
        }

        bool applyGrabReturnHandWorldTransform(bool isLeft, const RE::NiTransform& handTransform)
        {
            return frik_visual_authority::applyExternalHandWorldTransform(
                GRAB_RETURN_HAND_TAG,
                handFromBool(isLeft),
                handTransform,
                GRAB_RETURN_HAND_PRIORITY);
        }

        void clearGrabReturnHandWorldTransform(bool isLeft)
        {
            (void)frik_visual_authority::clearExternalHandWorldTransform(GRAB_RETURN_HAND_TAG, handFromBool(isLeft));
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
        bool tryGetGrabAuthorityBodyWorldTransform(
            RE::hknpWorld* world,
            RE::hknpBodyId bodyId,
            RE::NiTransform& outTransform)
        {
            return ::rock::tryGetGrabAuthorityBodyWorldTransform(world, bodyId, outTransform);
        }
    }

    void Hand::beginGrabVisualReturn()
    {
        if (!g_rockConfig.rockGrabHandReturnEnabled ||
            !_hasLastPublishedGrabVisualHandTransform ||
            !isUsableGrabVisualTransform(_lastPublishedGrabVisualHandTransform) ||
            !frik_visual_authority::isAvailable()) {
            clearGrabVisualReturn("release-not-eligible", false);
            return;
        }

        _grabVisualReturn.begin(_lastPublishedGrabVisualHandTransform);
        if (!applyGrabReturnHandWorldTransform(_isLeft, _grabVisualReturn.start)) {
            _grabVisualReturn.clear();
            clearGrabReturnHandWorldTransform(_isLeft);
            ROCK_LOG_WARN(Hand, "{} hand visual return start failed; restoring tracked authority immediately", handName());
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand visual return started from=({:.2f},{:.2f},{:.2f})",
            handName(),
            _grabVisualReturn.start.translate.x,
            _grabVisualReturn.start.translate.y,
            _grabVisualReturn.start.translate.z);
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
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockGrabHandReturnTimeMin,
                .maxSeconds = g_rockConfig.rockGrabHandReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockGrabHandReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockGrabHandReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockGrabHandReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockGrabHandReturnMaxAngleDegrees,
            },
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
        _grabAuthorityProxy.reset();
        clearGrabAuthorityProxyRuntimeLocked();
    }

    void Hand::abandonGrabAuthorityProxy()
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        abandonGrabAuthorityProxyLocked();
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
            _grabFrame.hasTelemetryCapture ?
                _grabFrame.authority.desiredBodyWorldAtGrab :
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
        RE::NiTransform& outProxyWorld,
        const char*& outSource,
        Hand::GrabAuthorityProxyFramePolicy policy) const
    {
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

    void Hand::updateConstraintGrabDriveMotors(RE::hknpWorld* world,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        float authorityForceScale,
        bool heldBodyColliding)
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
            .angularToLinearForceRatio = grabAngularToLinearForceRatio(_heldObjectIsLooseWeapon),
            .mass = massSummary.motorMass(),
            .forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio,
            .effectiveMotorMassFloorEnabled = g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
            .effectiveMotorMassFloor = g_rockConfig.rockGrabEffectiveMotorMassFloor,
            .fadeInEnabled = _grabFrame.fadeInGrabConstraint,
            .fadeElapsed = _grabStartTime,
            .fadeDuration = forceFadeInTime,
        };
        const auto output = grab_motion_controller::solveMotorTargets(motorInput, heldBodyColliding);
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

    bool Hand::isCloseSelectionClearlyOutsidePocket(RE::hknpWorld* world) const
    {
        /*
         * Cheap pre-check for a close press: when the selection hit is well
         * outside the pocket the grab capture would only prepare the body set
         * and roll it back, so the input layer starts the pull directly.
         */
        if (!world || !_currentSelection.isValid() || _currentSelection.isFarSelection ||
            !_currentSelection.hasHitPoint || _currentSelection.pinchCloseSelectionFallback) {
            return false;
        }
        RE::NiTransform proxyFrameWorld{};
        if (!tryComputeGrabProxyLocalPalmPocketFrameWorld(world, proxyFrameWorld)) {
            return false;
        }
        const float clearlyOutsideDistance = g_rockConfig.rockGrabPocketRadiusGameUnits + kCloseSelectionPullPreCheckMarginGameUnits;
        return pointDistanceGameUnits(_currentSelection.hitPointWorld, proxyFrameWorld.translate) > clearlyOutsideDistance;
    }

    bool Hand::startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform)
    {
        /*
         * The pull is the only way an object reaches the palm pocket, for far
         * selections and for close presses outside the pocket alike. ROCK promotes the
         * selected object tree through FO4VR's recursive wrappers, scans the
         * resulting dynamic body set, and then applies short-lived predicted
         * velocity to the accepted motions. It avoids a keyframed object path
         * that would conflict with the dynamic grab constraint used after arrival.
         */
        if (!world || _state != HandState::SelectionLocked || !_currentSelection.isValid()) {
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
         * and the seat machinery (support model, seated reacquire, seat depth
         * stop) settles the surface onto the palm.
         */
        if (g_rockConfig.rockPullToObjectCenterEnabled) {
            _pullPointOffsetHavok = {};
        }

        /*
         * One mesh extraction at pull start, cached in primary-body local space
         * for two flight-time readers: the long-object presentation servo
         * (principal axis) and the arrival test in updateDynamicPull (closest
         * mesh point to the palm centre). Both re-derive world data from the
         * live body pose; neither touches scene nodes per frame.
         */
        _pullPresentationAxisBodyLocal = {};
        _pullPresentationElongationRatio = 0.0f;
        _pullPresentationValid = false;
        _pullLocalMeshTriangles.clear();
        if (hasPrimaryBodyWorld) {
            RE::NiAVObject* pullMeshNode = _currentSelection.visualNode ? _currentSelection.visualNode : rootNode;
            std::vector<TriangleData> pullTriangles;
            std::vector<GrabSurfaceTriangleData> pullSurfaceTriangles;
            MeshExtractionStats pullMeshStats;
            extractAllSurfaceTriangles(pullMeshNode,
                pullTriangles,
                pullSurfaceTriangles,
                (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth),
                &pullMeshStats,
                false);
            _pullLocalMeshTriangles = cacheBoundedTrianglesInLocalSpace(pullTriangles, primaryBodyWorld, kMaxPullArrivalTriangles);
            const auto longAxis = computeGrabMeshLongAxis(pullTriangles);
            if (g_rockConfig.rockPullLongAxisPresentationEnabled &&
                longAxis.valid &&
                longAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio) {
                const RE::NiPoint3 axisBodyLocal = normalizeOrZero(
                    transform_math::worldVectorToLocal(primaryBodyWorld, longAxis.axisWorld));
                if (lengthSquared(axisBodyLocal) > 0.000001f) {
                    _pullPresentationAxisBodyLocal = axisBodyLocal;
                    _pullPresentationElongationRatio = longAxis.elongationRatio;
                    _pullPresentationValid = true;
                }
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL mesh cached: arrivalTris={}/{} presentation={} elongation={:.2f} reason={} axisLocal=({:.3f},{:.3f},{:.3f})",
                handName(),
                _pullLocalMeshTriangles.size(),
                pullTriangles.size(),
                _pullPresentationValid ? "yes" : "no",
                longAxis.elongationRatio,
                longAxis.reason,
                _pullPresentationAxisBodyLocal.x,
                _pullPresentationAxisBodyLocal.y,
                _pullPresentationAxisBodyLocal.z);
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

        /*
         * Arrival is the pocket gate applied to the closest cached mesh point of
         * the pulled object. The pocket is built from the same live palm frame
         * the capture uses one frame later, so the two gates agree up to one
         * frame of hand motion; the radius margin absorbs that frame. Without a
         * cached mesh the primary body centre stands in for the surface.
         */
        RE::NiTransform pullProxyFrameWorld{};
        const bool hasPullPocketFrame = tryComputeGrabProxyLocalPalmPocketFrameWorld(world, pullProxyFrameWorld);
        const grab_three_phase::GrabPocketFrame pullPocket = hasPullPocketFrame ?
            grab_three_phase::buildGrabPocketFrameWithPalmCenter(
                makeGeneratedProxyAuthorityRelationFrame(pullProxyFrameWorld),
                _isLeft,
                pullProxyFrameWorld.translate,
                g_rockConfig.rockGrabPocketDepthGameUnits,
                g_rockConfig.rockGrabPocketRadiusGameUnits) :
            grab_three_phase::GrabPocketFrame{};
        const RE::NiPoint3 grabPivotWorld = pullPocket.valid ? pullPocket.palmCenterWorld : handWorldTransform.translate;
        const auto grabPivotHavokVector = niPointToHkVector(grabPivotWorld);
        const RE::NiPoint3 grabPivotHavok{ grabPivotHavokVector.x, grabPivotHavokVector.y, grabPivotHavokVector.z };
        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };

        const float distanceGameUnits = (grabPivotHavok - objectPointHavok).Length() * havokToGameScale();
        _currentSelection.distance = distanceGameUnits;
        const float arrivalDistance = (std::max)(0.0f, pullPocket.pocketRadiusGameUnits - kPullArrivalPocketMarginGameUnits);

        RE::NiPoint3 arrivalPointWorld{
            objectPointHavok.x * havokToGameScale(),
            objectPointHavok.y * havokToGameScale(),
            objectPointHavok.z * havokToGameScale(),
        };
        const char* arrivalPointSource = "primaryBodyCenter";
        RE::NiTransform arrivalBodyWorld{};
        if (!_pullLocalMeshTriangles.empty() &&
            tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, arrivalBodyWorld)) {
            RE::NiPoint3 closestMeshPointWorld{};
            if (findClosestLocalMeshPointToWorldPoint(_pullLocalMeshTriangles, arrivalBodyWorld, grabPivotWorld, closestMeshPointWorld)) {
                arrivalPointWorld = closestMeshPointWorld;
                arrivalPointSource = "closestMeshPoint";
            }
        }
        const auto arrivalGate = grab_three_phase::evaluatePocketGate(
            pullPocket,
            arrivalPointWorld,
            g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
            kPullArrivalPocketMarginGameUnits);
        if (arrivalGate.inside) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = arrivalPointWorld;
            _currentSelection.hasHitPoint = true;
            markPullCatchIntentArrived();
            _pullTargetHavok = {};
            _pullElapsedSeconds = 0.0f;
            _pullDurationSeconds = 0.0f;
            _pullHasTarget = false;
            applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::PullArrivedClose });
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL arrived -> close grab commit source={} pocketDist={:.1f} signedPalm={:.1f} radius={:.1f} margin={:.1f} centerDist={:.1f}",
                handName(),
                arrivalPointSource,
                arrivalGate.gripToPalmDistanceGameUnits,
                arrivalGate.signedPalmDistanceGameUnits,
                pullPocket.pocketRadiusGameUnits,
                kPullArrivalPocketMarginGameUnits,
                distanceGameUnits);
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
         * axis toward the hand Weapon node handle axis while the pull drive
         * owns the object, so long props arrive oriented the way a weapon
         * sits in the hand; the seat then finishes the same alignment.
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
                RE::NiPoint3 targetAxisWorld{};
                if (!tryGetWeaponHandleAxisWorld(targetAxisWorld)) {
                    targetAxisWorld = RE::NiPoint3{};
                }
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
        outPreparation = {};
        const auto& selectedObject = _currentSelection;
        outPreparation.scanOptions = makeActiveGrabBodyScanOptions(selectedObject);
        const auto& scanOptions = outPreparation.scanOptions;

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
        outPreparation = {};
        outPreparation.handBodyWorldAtGrab = getLiveBodyWorldTransform(world, _handBody.getBodyId());
        outPreparation.proxyFrameWorldAtGrab = outPreparation.handBodyWorldAtGrab;
        if (!resolveGrabAuthorityProxyFrame(
                world,
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
            outSetup.collidableNode);
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
    };

    void Hand::extractGrabMeshEvidence(
        RE::hknpWorld* world,
        RE::hknpBodyId objectBodyId,
        RE::NiAVObject* rootNode,
        RE::NiAVObject* collidableNode,
        RE::NiAVObject* meshSourceNode,
        bool handPocketOnlyGrab,
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
        extractAllSurfaceTriangles(
            meshSourceNode,
            outExtraction.meshTriangles,
            outExtraction.surfaceTriangles,
            meshExtractionDepth,
            &outExtraction.stats,
            handPocketOnlyGrab);

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
            extractAllSurfaceTriangles(
                candidateRoot,
                outExtraction.meshTriangles,
                outExtraction.surfaceTriangles,
                meshExtractionDepth,
                &outExtraction.stats,
                handPocketOnlyGrab);
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
        grab_three_phase::GrabPocketFrame pocket{};
        RE::NiPoint3 gripPoint{};
        float selectionToMeshDistanceGameUnits = std::numeric_limits<float>::max();
        bool meshGrabFound = false;
        GrabSurfaceHit surfaceHit{};
        RE::NiAVObject* surfaceOwnerNode = nullptr;
        const char* pointMode = "noPalmSeatPoint";
        GrabPivotAuthoritySource pointAuthoritySource = GrabPivotAuthoritySource::None;
        const char* fallbackReason = "noSurfaceTriangles";
    };

    void Hand::resolveGrabSurfaceEvidence(
        const GrabProxyPreparation& proxy,
        const GrabMeshExtraction& mesh,
        GrabSurfaceEvidence& outEvidence)
    {
        /*
         * Provisional palm seat, used only to resolve the primary body; the
         * owner-filtered pass in resolveGrabSeat re-selects the point against
         * that body from the same pocket frame. A collision-query hit is never
         * a seat, so without surface triangles nothing is proposed here.
         */
        const auto& sel = _currentSelection;
        outEvidence = {};
        outEvidence.gripPoint = sel.hasHitPoint ? sel.hitPointWorld : proxy.grabPivotAForPrimaryChoice;
        outEvidence.pocket = grab_three_phase::buildGrabPocketFrameWithPalmCenter(
            proxy.proxyAuthorityFrameWorldAtGrab,
            _isLeft,
            proxy.palmPocketPivotAWorld,
            g_rockConfig.rockGrabPocketDepthGameUnits,
            g_rockConfig.rockGrabPocketRadiusGameUnits);
        const auto palmSeat = selectPalmSeatPoint(mesh.surfaceTriangles, outEvidence.pocket, PalmSeatOwnerFilter{});
        outEvidence.fallbackReason = palmSeat.reason;
        if (!palmSeat.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PALM SEAT provisional point unavailable: reason={} tris={} evaluated={} rejectedFacing={} meshNode='{}'",
                handName(),
                palmSeat.reason,
                mesh.surfaceTriangles.size(),
                palmSeat.evaluatedCandidates,
                palmSeat.rejectedFacing,
                nodeDebugName(mesh.meshSourceNode));
            return;
        }
        outEvidence.surfaceHit = palmSeat.hit;
        outEvidence.gripPoint = palmSeat.hit.position;
        outEvidence.surfaceOwnerNode = palmSeat.hit.sourceNode;
        outEvidence.meshGrabFound = true;
        outEvidence.selectionToMeshDistanceGameUnits =
            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, outEvidence.gripPoint) : std::numeric_limits<float>::max();
        outEvidence.surfaceHit.hasSelectionHit = sel.hasHitPoint;
        outEvidence.surfaceHit.selectionToMeshDistanceGameUnits = outEvidence.selectionToMeshDistanceGameUnits;
        outEvidence.pointMode = "palmPocketMeshPoint";
        outEvidence.pointAuthoritySource = GrabPivotAuthoritySource::PalmPocketMeshPoint;
        ROCK_LOG_DEBUG(Hand,
            "{} hand PALM SEAT provisional: point=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) tri={} source={} owner='{}' shape='{}' "
            "pocket=({:.1f},{:.1f},{:.1f}) signedPalm={:.2f}gu lateral={:.2f}gu selectionDelta={:.1f}gu evaluated={} rejectedFacing={} tris={}",
            handName(),
            palmSeat.hit.position.x,
            palmSeat.hit.position.y,
            palmSeat.hit.position.z,
            palmSeat.hit.normal.x,
            palmSeat.hit.normal.y,
            palmSeat.hit.normal.z,
            palmSeat.hit.triangleIndex,
            grabSurfaceSourceKindName(palmSeat.hit.sourceKind),
            nodeDebugName(palmSeat.hit.sourceNode),
            nodeDebugName(palmSeat.hit.sourceShape),
            outEvidence.pocket.pocketCenterWorld.x,
            outEvidence.pocket.pocketCenterWorld.y,
            outEvidence.pocket.pocketCenterWorld.z,
            palmSeat.hit.signedAlongPalmDistanceGameUnits,
            palmSeat.hit.lateralPalmDistanceGameUnits,
            sel.hasHitPoint ? outEvidence.selectionToMeshDistanceGameUnits : -1.0f,
            palmSeat.evaluatedCandidates,
            palmSeat.rejectedFacing,
            mesh.surfaceTriangles.size());
    }

    struct Hand::GrabBodyResolution
    {
        object_physics_body_set::PrimaryBodyChoice primaryChoice{};
        mechanical_connected_body_set::MechanicalScope mechanicalScope{};
        RE::NiPoint3 primaryChoiceTarget{};
        bool relaxedArticulatedAuthority = false;
    };

    void Hand::resolveGrabBodyAndContactPolicy(
        const object_physics_body_set::ObjectPhysicsBodySet& beforePrepBodySet,
        const object_physics_body_set::ObjectPhysicsBodySet& preparedBodySet,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& activeLifecycle,
        const GrabProxyPreparation& proxy,
        const GrabSurfaceEvidence& surface,
        GrabBodyResolution& outResolution)
    {
        outResolution = {};
        const auto& selectedObject = _currentSelection;
        outResolution.primaryChoiceTarget = surface.meshGrabFound ?
            surface.gripPoint :
            (selectedObject.hasHitPoint ? selectedObject.hitPointWorld : proxy.grabPivotAForPrimaryChoice);
        const auto nearestPrimaryChoice = preparedBodySet.choosePrimaryBody(
            object_physics_body_set::INVALID_BODY_ID,
            object_physics_body_set::PurePoint3{ outResolution.primaryChoiceTarget });
        const auto* surfaceOwnerRecord = preparedBodySet.findAcceptedRecordByOwnerNode(surface.surfaceOwnerNode);
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

        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB BODY RESOLUTION: selectedBody={} resolvedBody={} reason={} resolver={} resolverReason={} skin={} sourceNode='{}' sourceKind={} target=({:.1f},{:.1f},{:.1f})",
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
            outResolution.primaryChoiceTarget.x,
            outResolution.primaryChoiceTarget.y,
            outResolution.primaryChoiceTarget.z);

        if (outResolution.primaryChoice.bodyId != INVALID_BODY_ID) {
            outResolution.mechanicalScope = mechanical_connected_body_set::buildFromPreparedBodySet(
                beforePrepBodySet,
                preparedBodySet,
                outResolution.primaryChoice.bodyId,
                selectedObject.targetKind,
                activeLifecycle.hasIncompleteNativeScan());
            outResolution.relaxedArticulatedAuthority =
                outResolution.mechanicalScope.strictPocketAuthorityRelaxed &&
                outResolution.primaryChoice.bodyId != object_physics_body_set::INVALID_BODY_ID;
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
            refreshGrabCaptureNodeTransform(ownerRefresh, "resolvedOwner", resolvedOwnerNode);
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
        _heldBodyIds = buildCommittedHeldBodyIds(
            objectBodyId.value,
            mechanicalScope.committedBodyIds,
            sharedContext,
            adoptedPeerHeldBodySet);
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
    }

    struct Hand::GrabBodyFrameCaptureInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabBodyResolution* bodyResolution = nullptr;
        const std::vector<GrabLocalTriangle>* localMeshTriangles = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        const std::string* objectName = nullptr;
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
        std::vector<TriangleData> fingerPoseMeshTriangles{};
        bool clearExternalOnFailure = false;
    };

    bool Hand::captureGrabBodyFrame(
        RE::hknpWorld* world,
        const GrabBodyFrameCaptureInput& input,
        GrabBodyFrameCapture& outCapture)
    {
        outCapture = {};
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyResolution = *input.bodyResolution;
        const auto& grabLocalMeshTriangles = *input.localMeshTriangles;
        const auto& sel = _currentSelection;
        auto* collidableNode = meshCapture.collidableNode;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        auto* rootNode = input.rootNode;
        auto objectBodyId = input.objectBodyId;
        const auto& objName = *input.objectName;

        outCapture.palmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(handWorldTransform, _isLeft);
        if (!tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, outCapture.grabBodyWorld)) {
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
        const auto& grabBodyWorldAtGrab = outCapture.grabBodyWorld;
        outCapture.hasMotionBodyWorld =
            tryResolveLiveBodyWorldTransform(world, objectBodyId, outCapture.motionBodyWorld, &outCapture.motionBodySource);
        /*
         * Custom constraint body-B data is authored in the hknp BODY frame.
         * HIGGS does the same kind of thing on Skyrim's hkp side: it freezes
         * pivot B from the rigid body transform and treats COM/motion as mass
         * data, not as a local-frame owner. A runtime MOTION-frame experiment
         * made the object settle at the wrong rotation even when the proxy
         * target was stable, so the live motion transform remains diagnostic
         * evidence only.
         */
        outCapture.constraintUsesMotionBody = false;
        outCapture.constraintBodyWorld = grabBodyWorldAtGrab;
        /*
         * The hidden proxy is body A for dynamic grab. The close-grab
         * pocket pivot is captured from the same generated/proxy-local
         * seat frame. The proxy body keeps its seat offset, and transform A
         * carries the selected pivot as an explicit local point on body A.
         */
        outCapture.grabPivotAWorld = proxy.palmPocketPivotAWorld;
        auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
        auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
        outCapture.bodyCollisionObject = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
        outCapture.ownerNode = outCapture.bodyCollisionObject ? outCapture.bodyCollisionObject->sceneObject : nullptr;
        auto* ownerNodeAtGrab = outCapture.ownerNode;
        if (ownerNodeAtGrab && ownerNodeAtGrab != collidableNode) {
            GrabCaptureTransformRefreshResult ownerAtGrabRefresh{};
            refreshGrabCaptureNodeTransform(ownerAtGrabRefresh, "ownerAtGrab", ownerNodeAtGrab);
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
        outCapture.objectToBody = computeRuntimeBodyLocalTransform(objectWorldTransform, grabBodyWorldAtGrab);
        /*
         * ROCK dynamic grab has one production authority convention:
         * the generated/proxy palm frame seats the selected BODY-local grip
         * point and owns the object angular relation through a row-view of
         * its generated local axes. The custom constraint stores body-B local
         * data in the rigid BODY frame. MOTION and COM are mass/diagnostic
         * data only.
         */
        outCapture.ownerBodyLocal =
            ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, grabBodyWorldAtGrab) : makeIdentityTransform();
        outCapture.rootBodyLocal =
            rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, grabBodyWorldAtGrab) : makeIdentityTransform();
        _grabFrame.localMeshTriangles.clear();
        _grabFrame.fingerPoseLocalMeshTriangles.clear();
        if (!grabLocalMeshTriangles.empty()) {
            _grabFrame.localMeshTriangles = grabLocalMeshTriangles;
        }
        _grabFrame.hasMeshPoseData = !_grabFrame.localMeshTriangles.empty();
        _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(bodyResolution.primaryChoice.reason);
        return true;
    }

    struct Hand::GrabSeatInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        GrabSurfaceEvidence* surface = nullptr;
        GrabBodyFrameCapture* bodyFrame = nullptr;
        const object_physics_body_set::ObjectPhysicsBodySet* preparedBodySet = nullptr;
        const RuntimePinchPocketCandidate* pinchPocketCandidate = nullptr;
        saved_grab_offset::HandOffset* savedGrabOffset = nullptr;
        bool* hasSavedGrabOffset = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::NiAVObject* collidableNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        bool grabbedFromPullCatch = false;
        bool looseWeaponGrab = false;
        bool handPocketOnlyGrab = false;
        bool relaxedArticulatedAuthority = false;
        GrabRollbackAction rollback{};
    };

    struct Hand::GrabSeatResult
    {
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        GrabPivotAuthoritySource resolvedAuthoritySource = GrabPivotAuthoritySource::None;
        const char* resolvedAuthorityReason = "notResolved";
        // Forced arrival moved every held body onto the seat before the commit.
        bool warpedToSeat = false;
        // Refused only by the palm-pocket gate; the caller may pull and retry.
        bool refusedOutsidePocket = false;
    };

    /*
     * The one seat function. A dynamic grab seats exactly one object-side
     * point on exactly one hand-side point:
     *   pinch pocket        - thumb/index pad contact on a small object;
     *   palm pocket         - the closest palm-facing mesh point to the pocket
     *                         centre, owner-checked against the held body;
     *   loose weapon attach - the authored FRIK weapon offset.
     * A palm seat is then pushed out along the palm normal by the mesh
     * support depth so the SURFACE rests on the palm; a pinch seat is centred
     * between the pads. Nothing else rotates or corrects the pose: what the
     * pocket gate accepted is what the freeze commits.
     */
    bool Hand::resolveGrabSeat(
        RE::hknpWorld* world,
        const GrabSeatInput& input,
        GrabSeatResult& outSeat)
    {
        outSeat = {};
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& meshCapture = *input.meshCapture;
        const auto& mesh = *input.mesh;
        auto& surface = *input.surface;
        auto& bodyFrame = *input.bodyFrame;
        const auto& preparedBodySet = *input.preparedBodySet;
        const auto& pinchPocketCandidate = *input.pinchPocketCandidate;
        auto& savedGrabOffset = *input.savedGrabOffset;
        auto& hasSavedGrabOffset = *input.hasSavedGrabOffset;
        const auto& sel = _currentSelection;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabSurfaceTriangles = mesh.surfaceTriangles;
        const auto& grabMeshTriangles = mesh.meshTriangles;
        const auto& grabLocalMeshTriangles = _grabFrame.localMeshTriangles;
        const auto& pocket = surface.pocket;
        auto& grabGripPoint = surface.gripPoint;
        auto& grabSurfaceHit = surface.surfaceHit;
        const auto& grabBodyWorldAtGrab = bodyFrame.grabBodyWorld;
        auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        const auto& objectToBodyAtGrab = bodyFrame.objectToBody;
        const auto& rootBodyLocalAtGrab = bodyFrame.rootBodyLocal;
        const auto objectBodyId = input.objectBodyId;
        auto* rootNode = input.rootNode;
        auto* collidableNode = input.collidableNode;
        const bool grabbedFromPullCatch = input.grabbedFromPullCatch;
        const bool looseWeaponGrab = input.looseWeaponGrab;
        auto& desiredObjectWorld = outSeat.desiredObjectWorld;
        auto& desiredBodyWorld = outSeat.desiredBodyWorld;
        desiredObjectWorld = objectWorldTransform;
        desiredBodyWorld = grabBodyWorldAtGrab;

        auto abortSeat = [&](bool refusedOutsidePocket) {
            outSeat.refusedOutsidePocket = refusedOutsidePocket;
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

        if (!pocket.valid) {
            ROCK_LOG_WARN(Hand, "{} GRAB ABORT: palm pocket frame invalid", handName());
            return abortSeat(false);
        }

        const bool usingPinchPocket = pinchPocketCandidate.valid;
        PalmSeatPointSelection palmSeat{};
        if (!usingPinchPocket) {
            palmSeat = selectPalmSeatPoint(
                grabSurfaceTriangles,
                pocket,
                PalmSeatOwnerFilter{
                    .selection = &sel,
                    .bodySet = &preparedBodySet,
                    .resolvedBodyId = objectBodyId.value,
                    .handPocketOnlyGrab = input.handPocketOnlyGrab,
                    .relaxedArticulatedAuthority = input.relaxedArticulatedAuthority,
                });
            if (!palmSeat.valid) {
                ROCK_LOG_WARN(Hand,
                    "{} hand GRAB failed: no palm-facing mesh point on body {} formID={:08X} reason={} tris={} evaluated={} rejectedFacing={} rejectedOwner={} meshNode='{}'",
                    handName(),
                    objectBodyId.value,
                    sel.refr ? sel.refr->GetFormID() : 0,
                    palmSeat.reason,
                    grabSurfaceTriangles.size(),
                    palmSeat.evaluatedCandidates,
                    palmSeat.rejectedFacing,
                    palmSeat.rejectedOwner,
                    nodeDebugName(mesh.meshSourceNode));
                return abortSeat(false);
            }
        }
        const GrabSurfaceHit seatHit = usingPinchPocket ? pinchPocketCandidate.surfaceHit : palmSeat.hit;

        /*
         * The one acquisition rule: the proxy and constraint are created only
         * when the seat point is already inside the palm pocket. A valid pinch
         * candidate has proven finger-pad contact and passes on its own. A
         * forced arrival (grenade quick-draw, provider force grab) has nothing
         * that moves the object into the hand, so it is warped onto its seat
         * below and then committed like an inside-pocket grab. Everything
         * else outside the pocket is refused with OutsidePocket, which the
         * input layer turns into a short pull.
         */
        const auto pocketGate = grab_three_phase::evaluatePocketGate(
            pocket,
            seatHit.position,
            g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
        const bool programmaticArrival = grabbedFromPullCatch || sel.forcedArrival;
        const bool warpToSeat = !usingPinchPocket && !pocketGate.inside && sel.forcedArrival;
        outSeat.warpedToSeat = warpToSeat;
        if (!usingPinchPocket && !pocketGate.inside && !warpToSeat) {
            ROCK_LOG_DEBUG(Hand,
                "{} GRAB refused outside pocket: reason={} dist={:.2f}gu signedPalm={:.2f}gu radius={:.2f}gu forced={} pullCatch={}",
                handName(),
                pocketGate.reason,
                pocketGate.gripToPalmDistanceGameUnits,
                pocketGate.signedPalmDistanceGameUnits,
                pocket.pocketRadiusGameUnits,
                sel.forcedArrival ? "yes" : "no",
                grabbedFromPullCatch ? "yes" : "no");
            return abortSeat(true);
        }

        const char* captureReason = usingPinchPocket ?
            pinchPocketCandidate.decision.reason :
            (warpToSeat ? "forcedArrivalWarpedToSeat" : pocketGate.reason);
        grabPivotAWorld = usingPinchPocket ? pinchPocketCandidate.pinchPocketWorld : pocket.palmCenterWorld;
        grabGripPoint = seatHit.position;
        grabSurfaceHit = seatHit;
        grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
        grabSurfaceHit.selectionToMeshDistanceGameUnits =
            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
        grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
        grabSurfaceHit.resolvedOwnerMatchesBody = true;
        grabSurfaceHit.shapeKey = sel.hitShapeKey;
        grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
        grabSurfaceHit.hitFraction = sel.hitFraction;
        grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
        surface.selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
        surface.surfaceOwnerNode = grabSurfaceHit.sourceNode;
        surface.meshGrabFound = true;
        surface.pointMode = usingPinchPocket ? "pinchPocket" : "palmPocketMeshPoint";
        surface.pointAuthoritySource = usingPinchPocket ?
            GrabPivotAuthoritySource::PinchPocketMeshPoint :
            GrabPivotAuthoritySource::PalmPocketMeshPoint;
        surface.fallbackReason = captureReason;
        GrabPivotAuthoritySource pivotAuthoritySource = surface.pointAuthoritySource;
        outSeat.resolvedAuthoritySource = pivotAuthoritySource;
        outSeat.resolvedAuthorityReason = captureReason;

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
         * Normal authority comes from the object surface that produced the
         * seat point; the pinch axis and the palm normal are fallbacks only.
         */
        RE::NiPoint3 gripNormalWorld = firstValidNormal({
            grabSurfaceHit.normal,
            usingPinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{},
            pocket.palmNormalWorld,
        });

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
        bool looseWeaponPrimaryAttachApplied = false;
        bool looseWeaponPrimaryAttachSourceVisible = false;
        const char* looseWeaponPrimaryAttachReason = "notEvaluated";
        if (programmaticArrival &&
            saved_grab_offset::participatesInSavedGrabOffsets(
                looseWeaponGrab,
                isThrowableLooseWeapon(selectedLooseWeaponForm(sel)))) {
            hasSavedGrabOffset = tryLoadSavedGrabOffsetHandOffset(sel.refr, _isLeft, savedGrabOffset);
        }
        RE::NiTransform grabProxyWorldForOffset{};
        const bool grabProxyWorldValidForOffset =
            programmaticArrival && tryComputeGrabProxyLocalPalmPocketFrameWorld(world, grabProxyWorldForOffset);
        RE::NiTransform savedGrabOffsetRootWorld{};
        const bool savedGrabOffsetAttachValid = hasSavedGrabOffset &&
            tryResolveSavedGrabOffsetAttach(
                grabProxyWorldForOffset,
                grabProxyWorldValidForOffset,
                savedGrabOffset,
                savedGrabOffsetRootWorld);
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
            handWorldTransform,
            savedGrabOffsetAttachValid,
            savedGrabOffsetRootWorld);
        looseWeaponPrimaryAttachReason = looseWeaponPrimaryAttachFrame.reason;
        if (looseWeaponPrimaryAttachFrame.valid) {
            desiredObjectWorld = looseWeaponPrimaryAttachFrame.desiredObjectWorld;
            desiredBodyWorld = looseWeaponPrimaryAttachFrame.desiredBodyWorld;
            grabGripPoint = looseWeaponPrimaryAttachFrame.gripPointWorld;
            gripNormalWorld = firstValidNormal({
                pocket.palmNormalWorld,
                gripNormalWorld,
            });
            surface.pointMode = "looseWeaponPrimaryAttach";
            surface.pointAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
            surface.fallbackReason = looseWeaponPrimaryAttachFrame.reason;
            grabSurfaceHit = GrabSurfaceHit{};
            pivotAuthoritySource = GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
            outSeat.resolvedAuthoritySource = pivotAuthoritySource;
            outSeat.resolvedAuthorityReason = looseWeaponPrimaryAttachFrame.reason;
            looseWeaponPrimaryAttachApplied = true;
            looseWeaponPrimaryAttachSourceVisible = looseWeaponPrimaryAttachFrame.sourceVisible;
        }
        const bool effectivePinchPocket = usingPinchPocket && !looseWeaponPrimaryAttachApplied;
        const bool palmSeatApplied = !usingPinchPocket && !looseWeaponPrimaryAttachApplied;

        /*
         * Shape class from the mesh PCA, capture diagnostics only: the saved
         * ground-truth capture records which class the seat was produced
         * under. One pass at capture, never per frame.
         */
        const auto seatLongAxis =
            palmSeatApplied && !grabMeshTriangles.empty() ? computeGrabMeshLongAxis(grabMeshTriangles) : GrabMeshLongAxisResult{};
        const bool seatRodShape =
            seatLongAxis.valid &&
            seatLongAxis.elongationRatio >= g_rockConfig.rockPullPresentationMinElongationRatio;
        const char* seatShapeClass = seatRodShape ? "rod" : (seatLongAxis.valid ? "compact" : "none");

        /*
         * Rod alignment: an elongated object seats the way a weapon sits in
         * the hand. Rotate the seat pose about the seat point so the mesh
         * long axis meets the hand Weapon node handle axis (nearest
         * hemisphere; the PCA axis has no sign). Close grabs align too; the
         * pull flight only pre-aligns so the object arrives near this pose.
         * The rotation survives the freeze (pivot alignment rewrites
         * translation only) and a large one fades the motor in. The axis is
         * the latest published collider-frame value, the same game frame as
         * the pocket read; the pocket itself still reads the live palm body.
         */
        RE::NiTransform seatBodyWorld = grabBodyWorldAtGrab;
        RE::NiTransform seatObjectWorld = objectWorldTransform;
        float seatAlignmentAngleDegrees = 0.0f;
        const char* seatAlignmentReason = "inactive";
        if (seatRodShape) {
            RE::NiPoint3 targetAxisWorld{};
            const RE::NiPoint3 currentAxisWorld = normalizeOrZero(seatLongAxis.axisWorld);
            if (!tryGetWeaponHandleAxisWorld(targetAxisWorld)) {
                seatAlignmentReason = "noWeaponHandleAxis";
            } else if (lengthSquared(currentAxisWorld) <= 0.000001f || lengthSquared(targetAxisWorld) <= 0.000001f) {
                seatAlignmentReason = "degenerateAxes";
            } else {
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
                    seatBodyWorld = rotateTransformWorldAboutPoint(grabBodyWorldAtGrab, rotationAxis, angleRadians, grabGripPoint);
                    seatObjectWorld = rotateTransformWorldAboutPoint(objectWorldTransform, rotationAxis, angleRadians, grabGripPoint);
                    desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                        seatBodyWorld,
                        grabPivotAWorld,
                        grabGripPoint);
                    desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                    seatAlignmentAngleDegrees = angleRadians * 57.29577951308232f;
                    seatAlignmentReason = "rodAlignedToWeaponHandleAxis";
                } else {
                    seatAlignmentReason = "alreadyAligned";
                }
            }
        } else if (seatLongAxis.valid) {
            seatAlignmentReason = "belowElongationGate";
        }

        /*
         * Seat depth stop: the freeze re-aligns the seat point exactly onto
         * pivot A, which pulls any mesh behind the seat point through the
         * palm. Push pivot A out along the palm normal by the mesh support
         * depth so the object's surface rests ON the palm instead of its
         * interior. Measured against the SEAT orientation, after the rod
         * alignment. Weapon attach frames and pinch pockets keep their own
         * seat authority and are excluded.
         */
        GrabSeatDepthStopResult seatDepthStop{};
        float seatDepthOffsetGameUnits = 0.0f;
        if (palmSeatApplied) {
            seatDepthStop = computeGrabSeatDepthStop(
                grabLocalMeshTriangles,
                seatObjectWorld,
                grabGripPoint,
                pocket.palmNormalWorld,
                g_rockConfig.rockGrabSeatDepthFootprintRadiusGameUnits,
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

        /*
         * Pinch seat centering: the freeze puts the pinch SURFACE hit on the
         * pocket point, which parks the object's near face at the pocket and
         * shifts its body toward one finger pad by its full local thickness.
         * Measure the mesh extents both ways along the pinch axis from the
         * seat point (small footprint - only the material actually between
         * the pads matters) and offset pivot A so the object's MID-THICKNESS
         * sits exactly at the pocket middle. Same pivot-A mechanism as the
         * depth stop; the correction is zero for a surface hit already
         * centered.
         */
        float pinchCenterOffsetGameUnits = 0.0f;
        if (effectivePinchPocket) {
            const RE::NiPoint3 pinchAxisWorld = normalizeOrZero(pinchPocketCandidate.pinchAxisWorld);
            if (lengthSquared(pinchAxisWorld) > 0.000001f) {
                // Finger-pad scale; pinch objects are small by classification.
                constexpr float kPinchCenterFootprintRadiusGameUnits = 2.5f;
                constexpr float kPinchCenterMaxExtentGameUnits = 8.0f;
                const auto extentTowardIndex = computeGrabSeatDepthStop(
                    grabLocalMeshTriangles,
                    objectWorldTransform,
                    grabGripPoint,
                    RE::NiPoint3{ -pinchAxisWorld.x, -pinchAxisWorld.y, -pinchAxisWorld.z },
                    kPinchCenterFootprintRadiusGameUnits,
                    kPinchCenterMaxExtentGameUnits);
                const auto extentTowardThumb = computeGrabSeatDepthStop(
                    grabLocalMeshTriangles,
                    objectWorldTransform,
                    grabGripPoint,
                    pinchAxisWorld,
                    kPinchCenterFootprintRadiusGameUnits,
                    kPinchCenterMaxExtentGameUnits);
                if (extentTowardIndex.valid && extentTowardThumb.valid) {
                    pinchCenterOffsetGameUnits =
                        (extentTowardIndex.depthGameUnits - extentTowardThumb.depthGameUnits) * 0.5f;
                    if (std::fabs(pinchCenterOffsetGameUnits) > 0.05f) {
                        grabPivotAWorld = grabPivotAWorld - pinchAxisWorld * pinchCenterOffsetGameUnits;
                        desiredBodyWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                            grabBodyWorldAtGrab,
                            grabPivotAWorld,
                            grabGripPoint);
                        desiredObjectWorld = deriveNodeWorldFromBodyWorld(desiredBodyWorld, objectToBodyAtGrab);
                    }
                }
            }
        }

        if (warpToSeat) {
            /*
             * Forced arrival: nothing moves this object into the hand, so
             * move it. Every accepted body takes the rigid delta that carries
             * the primary body onto its final seat, and the commit below then
             * behaves exactly like an inside-pocket grab (zero motor error on
             * the first substep; velocities are zeroed in
             * initializePostFreezeGrab).
             */
            const RE::NiTransform warpDelta = multiplyTransforms(desiredBodyWorld, invertTransform(grabBodyWorldAtGrab));
            const auto warpBodies = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(objectBodyId.value, _heldBodyIds);
            std::uint32_t warpedBodies = 0;
            std::uint32_t failedWarps = 0;
            for (const auto bodyId : warpBodies) {
                RE::NiTransform warpedBodyWorld{};
                if (bodyId == objectBodyId.value) {
                    warpedBodyWorld = desiredBodyWorld;
                } else if (tryGetBodyWorldTransform(world, RE::hknpBodyId{ bodyId }, warpedBodyWorld)) {
                    warpedBodyWorld = multiplyTransforms(warpDelta, warpedBodyWorld);
                } else {
                    ++failedWarps;
                    continue;
                }
                if (havok_runtime::setBodyTransformDeferred(world, bodyId, warpedBodyWorld)) {
                    ++warpedBodies;
                } else {
                    ++failedWarps;
                }
            }
            ROCK_LOG_DEBUG(Hand,
                "{} GRAB forced arrival warped to seat: bodies={} failed={} shift={:.2f}gu rotation={:.1f}deg gripDist={:.2f}gu",
                handName(),
                warpedBodies,
                failedWarps,
                translationDeltaGameUnits(desiredBodyWorld, grabBodyWorldAtGrab),
                rotationDeltaDegrees(desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate),
                pocketGate.gripToPalmDistanceGameUnits);
            if (failedWarps > 0) {
                ROCK_LOG_WARN(Hand,
                    "{} GRAB forced arrival warp incomplete: {} of {} bodies not moved",
                    handName(),
                    failedWarps,
                    warpBodies.size());
            }
        }

        auto& selectedGripPointLocal = bodyFrame.selectedGripPointLocal;
        auto& selectedPivotBBodyLocalGame = bodyFrame.selectedPivotBBodyLocalGame;
        selectedGripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
        selectedPivotBBodyLocalGame = transform_math::worldPointToLocal(grabBodyWorldAtGrab, grabGripPoint);

        /*
         * Seat diagnostics, kept past the log line so a saved ground-truth
         * capture can record the seat ROCK produced for this grab next to
         * the pose the user corrected it to.
         */
        _grabFrame.seat.diagnostics = GrabSeatDiagnostics{
            .acquisitionMode = sel.forcedArrival ? "forceGrab" : (grabbedFromPullCatch ? "pullCatch" : "closeGrab"),
            .shapeClass = seatShapeClass,
            .elongationRatio = seatLongAxis.elongationRatio,
            .secondElongationRatio = seatLongAxis.secondElongationRatio,
            .alignmentAngleDegrees = seatAlignmentAngleDegrees,
            .alignmentReason = seatAlignmentReason,
            .depthGameUnits = seatDepthStop.depthGameUnits,
            .depthOffsetGameUnits = seatDepthOffsetGameUnits,
            .depthReason = seatDepthStop.reason,
        };
        _grabFrame.gripEvidence.gripEvidenceLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
        _grabFrame.gripEvidence.gripNormalLocal = transform_math::worldVectorToLocal(objectWorldTransform, gripNormalWorld);
        storeGripSourceEvidence(_grabFrame,
            grabSurfaceHit.sourceNode ? grabSurfaceHit.sourceNode : collidableNode,
            objectWorldTransform,
            grabGripPoint,
            gripNormalWorld,
            lengthSquared(gripNormalWorld) > 0.000001f);
        _grabFrame.gripEvidence.gripEvidenceTriangleIndex =
            grabSurfaceHit.valid && grabSurfaceHit.hasTriangle ? static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex) : 0xFFFF'FFFF;
        _grabFrame.gripEvidence.gripEvidenceShapeKey = grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
        _grabFrame.gripEvidence.gripEvidenceShapeCollisionFilterInfo = grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
        _grabFrame.gripEvidence.gripEvidenceHitFraction = grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
        _grabFrame.gripEvidence.hasGripEvidenceShapeKey = grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
        if (looseWeaponPrimaryAttachApplied) {
            _grabFrame.gripEvidence.gripSourceNode = nullptr;
            _grabFrame.gripEvidence.gripPointSourceNodeLocal = {};
            _grabFrame.gripEvidence.gripNormalSourceNodeLocal = {};
            _grabFrame.gripEvidence.hasGripSourceNodePoint = false;
            _grabFrame.gripEvidence.hasGripSourceNodeNormal = false;
        }
        auto& grabFingerPoseMeshTriangles = bodyFrame.fingerPoseMeshTriangles;
        if (!grabMeshTriangles.empty()) {
            grabFingerPoseMeshTriangles = selectNearestGrabFingerPoseTriangles(
                grabMeshTriangles,
                grabGripPoint,
                kMaxGrabRuntimeFingerPoseTriangles);
            _grabFrame.fingerPoseLocalMeshTriangles = cacheTrianglesInLocalSpace(grabFingerPoseMeshTriangles, objectWorldTransform);
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
        _grabFrame.hasMeshPoseData =
            !_grabFrame.localMeshTriangles.empty() ||
            !_grabFrame.fingerPoseLocalMeshTriangles.empty();
        _grabFrame.pivotAuthority.source = pivotAuthoritySource;
        _grabFrame.pivotAuthority.pocketDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
        _grabFrame.pivotAuthority.selectionDistanceGameUnits =
            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, seatHit.position) : std::numeric_limits<float>::max();
        _grabFrame.authority.grabPivotWorldAtGrab = grabPivotAWorld;
        _grabFrame.gripEvidence.gripPointWorldAtGrab = grabGripPoint;
        _grabFrame.seat.activeGrabPointMode = surface.pointMode;
        _grabFrame.seat.mode = effectivePinchPocket ? GrabSeatMode::PinchPocket : GrabSeatMode::PalmPocket;
        _grabFrame.seat.hasPinchPocket = effectivePinchPocket;
        _grabFrame.seat.pinchPocketWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : RE::NiPoint3{};
        _grabFrame.seat.pinchAxisWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchAxisWorld : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        _grabFrame.seat.palmSeatPointWorldAtGrab = effectivePinchPocket ? pinchPocketCandidate.pinchPocketWorld : pocket.palmCenterWorld;
        _grabFrame.seat.hasPalmSeatPoint = true;
        _grabFrame.seat.palmSeatPointMode =
            effectivePinchPocket ? "pinchPocket" : (looseWeaponPrimaryAttachApplied ? "looseWeaponPrimaryAttach" : "palmPocket");
        _grabFrame.syntheticLooseWeaponPrimaryAttach = looseWeaponPrimaryAttachApplied;
        _grabFrame.fingerPoseAimValid = true;
        _grabFrame.fingerPoseAimReason = effectivePinchPocket ? "pinchPocketThumbIndexTargets" : "seatPointEvidence";

        const auto captureFingerTargets = effectivePinchPocket ?
            buildRuntimePinchFingerPoseTargets(pinchPocketCandidate) :
            buildRuntimeFingerPoseTargets(grabGripPoint, gripNormalWorld);
        storeFingerPoseTargetsInGrabFrame(_grabFrame, captureFingerTargets, objectWorldTransform);

        ROCK_LOG_DEBUG(Hand,
            "{} GRAB SEAT: mode={} seat={} reason={} grip=({:.1f},{:.1f},{:.1f}) pivotB=({:.2f},{:.2f},{:.2f}) dist={:.2f} source={} "
            "shape={} seatAlign={:.1f}deg/{} seatDepth={:.2f}/{} warped={} looseWeaponPrimaryAttach={}/{}",
            handName(),
            surface.pointMode,
            grabSeatModeName(_grabFrame.seat.mode),
            captureReason,
            grabGripPoint.x,
            grabGripPoint.y,
            grabGripPoint.z,
            selectedPivotBBodyLocalGame.x,
            selectedPivotBBodyLocalGame.y,
            selectedPivotBBodyLocalGame.z,
            usingPinchPocket ? pinchPocketCandidate.pocketToSurfaceDistanceGameUnits : pocketGate.gripToPalmDistanceGameUnits,
            grabPivotAuthoritySourceName(_grabFrame.pivotAuthority.source),
            seatShapeClass,
            seatAlignmentAngleDegrees,
            seatAlignmentReason,
            seatDepthStop.depthGameUnits,
            seatDepthStop.reason,
            warpToSeat ? "yes" : "no",
            looseWeaponPrimaryAttachApplied ? "yes" : "no",
            looseWeaponPrimaryAttachReason);
        return true;
    }

    struct Hand::GrabFrozenCommitInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabBodyFrameCapture* bodyFrame = nullptr;
        GrabSeatResult* seatCapture = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::hknpBodyId objectBodyId{};
        std::uint64_t traceId = 0;
        const std::string* objectName = nullptr;
        GrabRollbackAction rollback{};
    };

    bool Hand::commitFrozenGrabAuthority(const GrabFrozenCommitInput& input)
    {
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyFrame = *input.bodyFrame;
        auto& seatCapture = *input.seatCapture;
        const auto& surface = *input.surface;
        const auto& sel = _currentSelection;
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
        const auto objectBodyId = input.objectBodyId;
        const auto grabTraceId = input.traceId;
        const auto& objName = *input.objectName;
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
                        .pivotAWorld = grabPivotAWorld,
                        .gripPointWorld = grabGripPoint,
                        .visualNormalWorld = frozenVisualNormalWorld,
                        .source = resolvedAuthorityPivotSourceForFreeze,
                        .hasDesiredObjectWorld = true,
                        .hasDesiredBodyWorld = true,
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
                if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} GRAB FREEZE AUTHORITY: formID={:08X} name='{}' body={} mode={} seat={} pivotAuthority={} frozenSource={} "
                        "pivotA=({:.2f},{:.2f},{:.2f}) grip=({:.2f},{:.2f},{:.2f}) pivotBBody=({:.2f},{:.2f},{:.2f}) lever={:.2f}gu pocket={:.2f}gu",
                        handName(),
                        sel.refr ? sel.refr->GetFormID() : 0,
                        objName,
                        objectBodyId.value,
                        _grabFrame.seat.activeGrabPointMode ? _grabFrame.seat.activeGrabPointMode : "none",
                        grabSeatModeName(_grabFrame.seat.mode),
                        grabPivotAuthoritySourceName(_grabFrame.pivotAuthority.source),
                        grab_authority_frame_math::grabAuthorityPivotSourceName(frozenAuthorityFrame.source),
                        frozenAuthorityFrame.pivotAWorld.x,
                        frozenAuthorityFrame.pivotAWorld.y,
                        frozenAuthorityFrame.pivotAWorld.z,
                        frozenAuthorityFrame.gripPointWorldAtGrab.x,
                        frozenAuthorityFrame.gripPointWorldAtGrab.y,
                        frozenAuthorityFrame.gripPointWorldAtGrab.z,
                        frozenAuthorityFrame.pivotBBodyLocalGame.x,
                        frozenAuthorityFrame.pivotBBodyLocalGame.y,
                        frozenAuthorityFrame.pivotBBodyLocalGame.z,
                        pointDistanceGameUnits(frozenAuthorityFrame.gripPointWorldAtGrab, grabBodyWorldAtGrab.translate),
                        _grabFrame.pivotAuthority.pocketDistanceGameUnits);
                }
        return true;
    }

    struct Hand::GrabPostFreezeInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        const GrabBodyFrameCapture* bodyFrame = nullptr;
        const GrabSeatResult* seatCapture = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::NiAVObject* collidableNode = nullptr;
        RE::NiAVObject* meshSourceNode = nullptr;
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
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& meshCapture = *input.meshCapture;
        const auto& bodyFrame = *input.bodyFrame;
        const auto& seatCapture = *input.seatCapture;
        const auto& surface = *input.surface;
        const auto& sel = _currentSelection;
        const auto& objectWorldTransform = meshCapture.objectWorldTransform;
        const auto& grabBodyWorldAtGrab = bodyFrame.grabBodyWorld;
        const auto& palmPos = bodyFrame.palmPosition;
        const auto& grabPivotAWorld = bodyFrame.grabPivotAWorld;
        const auto& grabGripPoint = surface.gripPoint;
        const auto& desiredBodyWorld = seatCapture.desiredBodyWorld;
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
                _grabDeviationExceededSeconds = 0.0f;
                const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabGripPoint;
                const float initialGrabDistance =
                    std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
                const float seatRotationDegrees = rotationDeltaDegrees(desiredBodyWorld.rotate, grabBodyWorldAtGrab.rotate);
                /*
                 * Fade the motors in only when the commit still has to move the
                 * object onto its seat: a grip point farther from pivot A than
                 * the hand-lerp minimum, or a seat rotation the object did not
                 * arrive with. A warped forced arrival already sits on its seat.
                 */
                constexpr float kSeatRotationFadeDegrees = 15.0f;
                const bool largeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
                const bool seatRotationSync = seatRotationDegrees >= kSeatRotationFadeDegrees;
                _grabFrame.fadeInGrabConstraint = !seatCapture.warpedToSeat && (largeInitialSync || seatRotationSync);
                _grabFrame.motorFadeReason = seatCapture.warpedToSeat ?
                    "warpedToSeat" :
                    (largeInitialSync ? "largeInitialSync" : (seatRotationSync ? "seatRotation" : "none"));
                _heldLocalLinearVelocityHistory = {};
                _heldLocalLinearVelocityHistoryCount = 0;
                _heldLocalLinearVelocityHistoryNext = 0;
                _heldLocalHandVelocityHistory = {};
                _heldHandAngularVelocityHistory = {};
                _heldHandVelocityHistoryCount = 0;
                _heldHandVelocityHistoryNext = 0;
                _lastHeldObjectLocalLinearVelocityHavok = {};
                _hasLastHeldObjectLocalLinearVelocityHavok = false;
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

            {
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
        const std::vector<TriangleData>* fingerPoseMeshTriangles = nullptr;
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

    bool Hand::commitGrabConstraintAndPose(
        RE::hknpWorld* world,
        const GrabConstraintCommitInput& input)
    {
        const auto& handWorldTransform = *input.handWorldTransform;
        const auto& proxy = *input.proxy;
        const auto& meshCapture = *input.meshCapture;
        const auto& surface = *input.surface;
        auto* bhkWorld = input.bhkWorld;
        const auto objectBodyId = input.objectBodyId;
        const auto* bodyBoneColliders = input.bodyBoneColliders;
        auto& activeLifecycle = *input.activeLifecycle;
        const auto& grabFingerPoseMeshTriangles = *input.fingerPoseMeshTriangles;
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
            {
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
                        "frozenPivotB=({:.2f},{:.2f},{:.2f}) "
                        "activePoint={} palmSeatPoint={} "
                        "pivotAToGrab_hk={:.4f} ({:.1f} game units) legacyPalmPivotAToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units "
                        "selectionToGripEvidence={:.1f} fingerPoseAim={} fingerPoseAimReason={}",
                        handName(), legacyPalmPivotAWorld.x, legacyPalmPivotAWorld.y, legacyPalmPivotAWorld.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                        grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                        _grabFrame.authority.pivotBBodyLocalGame.x, _grabFrame.authority.pivotBBodyLocalGame.y, _grabFrame.authority.pivotBBodyLocalGame.z,
                        _grabFrame.seat.activeGrabPointMode,
                        _grabFrame.seat.palmSeatPointMode, pivotAToGrab,
                        pivotAToGrab * havokToGameScale(), legacyPalmPivotAToHandOrigin, pivotAToHandOrigin, _grabFrame.pivotAuthority.selectionDistanceGameUnits,
                        _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);
                }

                const char* driveReason = joiningPeerHeldObject ? "joining-peer-held-loose-object" : "ordinary-dynamic-loose-object";
                if (!createProxyConstraintGrabDrive(
                        bhkWorld,
                        world,
                        objectBodyId,
                        proxyFrameWorldAtGrab,
                        handWorldTransform,
                        grabPivotAWorld,
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
                acquireHeldObjectBodyFlagLeases(world, _savedObjectState.bodyId.value, _heldBodyIds, heldBodyFlagLeaseOwner(this));
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

            {
                int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
                for (int i = 0; i < count; i++) {
                    _heldBodyIdsSnapshot[i] = _heldBodyIds[i];
                }
                _heldBodyIdsCount.store(count, std::memory_order_release);
                _isHoldingFlag.store(true, std::memory_order_release);
            }

            held_scene_presentation::Registration sceneRegistration{};
            sceneRegistration.traceId = _grabFrame.traceId;
            for (const std::uint32_t heldBodyId : _heldBodyIds) {
                if (sceneRegistration.count >=
                    held_scene_presentation::kMaxRegisteredBodies) {
                    break;
                }

                auto* collisionObject =
                    havok_runtime::getCollisionObjectFromBody(
                        world,
                        RE::hknpBodyId{ heldBodyId });
                if (!collisionObject) {
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

            if (g_rockConfig.rockGrabNearbyDampingEnabled) {
                object_physics_body_set::BodySetScanOptions dampingOptions{};
                dampingOptions.mode = physics_body_classifier::InteractionMode::PassivePush;
                dampingOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
                dampingOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
                dampingOptions.heldBySameHand = &_heldBodyIds;
                dampingOptions.maxDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                {
                    performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabNearbyDampingBegin);
                    _nearbyGrabDamping = nearby_grab_damping::beginNearbyGrabDamping(bhkWorld,
                        world,
                        sel.refr,
                        _heldBodyIds,
                        grabGripPoint,
                        g_rockConfig.rockGrabNearbyDampingRadius,
                        g_rockConfig.rockGrabNearbyDampingSeconds,
                        g_rockConfig.rockGrabNearbyLinearDamping,
                        g_rockConfig.rockGrabNearbyAngularDamping,
                        dampingOptions);
                }
                performance_profiler::observeValue(performance_profiler::ValueMetric::GrabNearbyDampingMotions, _nearbyGrabDamping.motions.size());
            } else {
                _nearbyGrabDamping.clear();
            }

            stopSelectionHighlight();
            clearSelectedCloseFingerPose();
            _grabFingerPose = {};
            _grabFingerTriangleIndex.clear();
            const bool useLooseWeaponPrimaryAttachHandPose = _grabFrame.syntheticLooseWeaponPrimaryAttach;
            _hasGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled && !useLooseWeaponPrimaryAttachHandPose;
            _grabFingerSweepDebugCapture = {};
            _grabFingerSweepDebugObjectWorld = {};
            _hasGrabFingerSweepDebug = false;
            _grabFingerPosePublished = false;
            _grabFingerLocalTransformFinalizePending = false;
            if (useLooseWeaponPrimaryAttachHandPose) {
                grab_finger_pose_runtime::SolvedGrabFingerPose savedGrabFingerPose{};
                const bool hasSavedGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled &&
                    hasSavedGrabOffset &&
                    tryBuildSavedGrabOffsetFingerPose(savedGrabOffset, savedGrabFingerPose);
                if (hasSavedGrabFingerPose) {
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
                    ROCK_LOG_INFO(Hand,
                        "{} hand loose weapon attach: applying savedGrabOffset finger pose",
                        handName());
                } else {
                    _grabFingerPosePublished = publishLooseWeaponPrimaryAttachHandPose(_isLeft, sel.refr);
                    if (!_grabFingerPosePublished) {
                        ROCK_LOG_WARN(Hand, "{} hand loose weapon attach: failed to publish FRIK weapon hand pose", handName());
                    }
                }
            } else if (_hasGrabFingerPose) {
                /*
                 * One finger solve at commit against the frozen commanded seat.
                 * The seat is final for every acquisition path (pinch, palm
                 * pocket, loose-weapon attach, forced-arrival warp).
                 * finalizeHeldObjectUpdate publishes this pose every held frame
                 * and starts the local-transform layer one update after the
                 * first joint publish.
                 */
                const bool pinchFingerPose = _grabFrame.seat.mode == GrabSeatMode::PinchPocket;
                const RE::NiTransform& targetObjectWorld = pinchFingerPose ? objectWorldTransform : _grabFrame.authority.desiredObjectWorldAtGrab;
                std::vector<TriangleData> targetFingerPoseWorldTriangles = pinchFingerPose ? grabFingerPoseMeshTriangles : std::vector<TriangleData>{};
                const auto& localFingerPoseTriangles = !_grabFrame.fingerPoseLocalMeshTriangles.empty() ? _grabFrame.fingerPoseLocalMeshTriangles : _grabFrame.localMeshTriangles;

                const RE::NiTransform& targetFingerHandTransform = handWorldTransform;
                root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshotAtGrab{};
                const RE::NiPoint3 fingerPosePivotWorld =
                    _grabFrame.hasTelemetryCapture ? _grabFrame.authority.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, targetFingerHandTransform);
                const auto targetFingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, targetObjectWorld);
                grab_finger_pose_runtime::FingerSweepDebugCapture sweepDebugCapture{};
                grab_finger_pose_runtime::SolvedGrabFingerPose fingerPose{};
                bool spatialIndexBuilt = false;
                bool commandedOpenDirectionsValid = false;
                if (pinchFingerPose) {
                    const auto* liveFingerSnapshotAtGrabPtr =
                        root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshotAtGrab) ? &liveFingerSnapshotAtGrab : nullptr;
                    fingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(targetFingerPoseWorldTriangles, targetFingerHandTransform, _isLeft, fingerPosePivotWorld,
                        targetFingerPoseTargets, g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, !pinchFingerPose, liveFingerSnapshotAtGrabPtr,
                        g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits, _grabFrame.fingerPoseAimValid,
                        g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits, -1.0f, g_rockConfig.rockGrabThumbSweepMaxOpenValue, g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                        nullptr, nullptr, nullptr, nullptr, grab_finger_pose_runtime::FingerPoseMeshRelation::AlreadyAtCommandedSeat);
                    applyPinchFingerPosePolicy(fingerPose, _grabFrame, g_rockConfig.rockGrabFingerMinValue);
                    grab_finger_pose_runtime::useThumbIndexCurveOnlyPose(fingerPose);
                    std::array<grab_finger_pose_runtime::FingerPadSurfaceEvidence, 5> padCaptureEvidence{};
                    (void)grab_finger_pose_runtime::refineGrabFingerPoseWithPadProbes(fingerPose, targetFingerPoseWorldTriangles, targetFingerPoseTargets,
                        liveFingerSnapshotAtGrab, targetObjectWorld, g_rockConfig.rockGrabMeshFingerPoseEnabled, true, padCaptureEvidence, true);
                    grab_finger_pose_runtime::captureSurfaceAimObjectLocal(fingerPose, targetObjectWorld);
                } else {
                    const auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPose(
                        localFingerPoseTriangles,
                        targetObjectWorld,
                        targetFingerHandTransform,
                        _isLeft,
                        fingerPosePivotWorld,
                        targetFingerPoseTargets,
                        _grabFingerTriangleIndex,
                        targetFingerPoseWorldTriangles,
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
                        });
                    fingerPose = frozenSolve.pose;
                    sweepDebugCapture = frozenSolve.sweepDebug;
                    liveFingerSnapshotAtGrab = frozenSolve.liveFingerSnapshot;
                    spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
                    commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
                }
                _grabFingerSweepDebugCapture = sweepDebugCapture;
                _grabFingerSweepDebugObjectWorld = targetObjectWorld;
                _hasGrabFingerSweepDebug = sweepDebugCapture.valid;
                _grabFingerPose = fingerPose;
                _grabFingerLocalTransformFinalizePending = _grabFingerPose.solved;
                ROCK_LOG_DEBUG(Hand, "{} GRAB POSE TARGET: seat={} solved={} hits={} triangles={} spatial={} nodes={} tests={} commandedAnchors={}",
                    handName(), grabSeatModeName(_grabFrame.seat.mode), _grabFingerPose.solved ? "yes" : "no", _grabFingerPose.hitCount,
                    _grabFingerPose.candidateTriangleCount, _grabFingerPose.usedSpatialIndex ? "yes" : "no", _grabFingerPose.spatialNodeVisitCount,
                    _grabFingerPose.spatialTriangleTestCount, commandedOpenDirectionsValid ? "yes" : "no");
            }

        return true;
    }

    Hand::GrabAttemptOutcome Hand::grabSelectedObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        const BodyBoneColliderSet* bodyBoneColliders,
        const GrabSharedObjectContext& sharedContext)
    {
        ValidatedGrabSelection validatedSelection{};
        if (!validateSelectedGrab(world, sharedContext, validatedSelection)) {
            return GrabAttemptOutcome::Refused;
        }

        const auto& sel = _currentSelection;
        const auto selectedRef = validatedSelection.retainedRef;
        const bool joiningPeerHeldObject = validatedSelection.joiningPeerHeldObject;
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
            return GrabAttemptOutcome::Refused;
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
                "{} PROXY GRAB PALM BASIS MISMATCH: ref='{}' formID={:08X} source={} rawToProxy={:.1f}deg axisDeg=({:.1f},{:.1f},{:.1f}) determinant=({:.3f},{:.3f}) proxyPivot=({:.1f},{:.1f},{:.1f}) palmPocketPivot=({:.1f},{:.1f},{:.1f}) pocketProxyDelta={:.2f}gu",
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
                palmPocketToProxyDeltaGameUnits);
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
            return GrabAttemptOutcome::Refused;
        }
        auto* collidableNode = meshCaptureSetup.collidableNode;
        auto* meshSourceNode = meshCaptureSetup.meshSourceNode;
        RE::NiTransform objectWorldTransform = meshCaptureSetup.objectWorldTransform;

        GrabMeshExtraction meshExtraction{};
        auto& meshStats = meshExtraction.stats;
        auto& grabMeshTriangles = meshExtraction.meshTriangles;
        std::vector<TriangleData> grabFingerPoseMeshTriangles;
        auto& grabSurfaceTriangles = meshExtraction.surfaceTriangles;
        std::vector<GrabLocalTriangle> grabLocalMeshTriangles;
        extractGrabMeshEvidence(world, objectBodyId, rootNode, collidableNode, meshSourceNode, handPocketOnlyGrab, meshExtraction);
        meshSourceNode = meshExtraction.meshSourceNode;
        GrabSurfaceEvidence surfaceEvidence{};
        resolveGrabSurfaceEvidence(proxyPreparation, meshExtraction, surfaceEvidence);
        auto& grabGripPoint = surfaceEvidence.gripPoint;
        if (grabSurfaceTriangles.empty()) {
            /*
             * A collision-query hit is never a seat: without rendered surface
             * triangles there is no object-side point to seat, so the grab
             * fails closed instead of pivoting on the selection hit.
             */
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no surface mesh for '{}' formID={:08X}; reason={} meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} totalTris={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                surfaceEvidence.fallbackReason,
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode),
                meshStats.visitedShapes,
                meshStats.totalTriangles());
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptOutcome::Refused;
        }

        GrabBodyResolution bodyResolution{};
        resolveGrabBodyAndContactPolicy(
            beforePrepBodySet,
            preparedBodySet,
            activeLifecycle,
            proxyPreparation,
            surfaceEvidence,
            bodyResolution);
        const auto& primaryChoice = bodyResolution.primaryChoice;
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
            return GrabAttemptOutcome::Refused;
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
            return GrabAttemptOutcome::Refused;
        }
        objectBodyId = resolvedBodyCapture.bodyId;
        collidableNode = resolvedBodyCapture.collidableNode;
        objectWorldTransform = resolvedBodyCapture.objectWorldTransform;
        grabLocalMeshTriangles = std::move(resolvedBodyCapture.localMeshTriangles);

        const RuntimePinchPocketCandidate pinchPocketCandidate = buildRuntimePinchPocketCandidate(
            sel,
            preparedBodySet,
            objectBodyId.value,
            objectWorldTransform,
            grabSurfaceTriangles,
            grabLocalMeshTriangles,
            grabGripPoint,
            handWorldTransform,
            _isLeft,
            !sel.isFarSelection && !grabbedFromPullCatch,
            handPocketOnlyGrab,
            looseWeaponGrab);
        if (pinchPocketCandidate.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PINCH POCKET candidate accepted: reason={} pocket=({:.1f},{:.1f},{:.1f}) point=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) gap={:.2f}gu dist={:.2f}gu extents=({:.2f},{:.2f},{:.2f})",
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
                pinchPocketCandidate.meshExtents.maxExtentGameUnits);
        } else if (g_rockConfig.rockDebugGrabFrameLogging) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PINCH POCKET candidate rejected: reason={} gap={:.2f}gu dist={:.2f}gu extentsValid={} extents=({:.2f},{:.2f},{:.2f}) close={} bodies={}",
                handName(),
                pinchPocketCandidate.decision.reason,
                pinchPocketCandidate.thumbIndexGapGameUnits,
                pinchPocketCandidate.pocketToSurfaceDistanceGameUnits,
                pinchPocketCandidate.meshExtents.valid ? "yes" : "no",
                pinchPocketCandidate.meshExtents.minExtentGameUnits,
                pinchPocketCandidate.meshExtents.middleExtentGameUnits,
                pinchPocketCandidate.meshExtents.maxExtentGameUnits,
                (!sel.isFarSelection && !grabbedFromPullCatch) ? "yes" : "no",
                preparedBodySet.acceptedCount());
        }
        if (sel.pinchCloseSelectionFallback && !pinchPocketCandidate.valid) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB rejected: pinch-direction close selection did not qualify for pinch pocket reason={} formID={:08X}",
                handName(),
                pinchPocketCandidate.decision.reason,
                sel.refr ? sel.refr->GetFormID() : 0);
            grabPreparationTransaction.rollback();
            clearGrabExternalHandWorldTransform(_isLeft);
            return GrabAttemptOutcome::Refused;
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
                .bodyResolution = &bodyResolution,
                .localMeshTriangles = &grabLocalMeshTriangles,
                .rootNode = rootNode,
                .objectBodyId = objectBodyId,
                .objectName = &objName,
            };
            GrabBodyFrameCapture bodyFrameCapture{};
            if (!captureGrabBodyFrame(world, bodyFrameInput, bodyFrameCapture)) {
                grabPreparationTransaction.rollback();
                if (bodyFrameCapture.clearExternalOnFailure) {
                    clearGrabExternalHandWorldTransform(_isLeft);
                }
                return GrabAttemptOutcome::Refused;
            }

            const GrabRollbackAction seatRollback{
                .context = &grabPreparationTransaction,
                .invoke = [](void* context) noexcept {
                    static_cast<decltype(grabPreparationTransaction)*>(context)->rollback();
                },
            };
            GrabSeatInput seatInput{
                .handWorldTransform = &handWorldTransform,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .surface = &surfaceEvidence,
                .bodyFrame = &bodyFrameCapture,
                .preparedBodySet = &preparedBodySet,
                .pinchPocketCandidate = &pinchPocketCandidate,
                .savedGrabOffset = &savedGrabOffset,
                .hasSavedGrabOffset = &hasSavedGrabOffset,
                .rootNode = rootNode,
                .collidableNode = collidableNode,
                .objectBodyId = objectBodyId,
                .grabbedFromPullCatch = grabbedFromPullCatch,
                .looseWeaponGrab = looseWeaponGrab,
                .handPocketOnlyGrab = handPocketOnlyGrab,
                .relaxedArticulatedAuthority = relaxedArticulatedAuthority,
                .rollback = seatRollback,
            };
            GrabSeatResult seatResult{};
            if (!resolveGrabSeat(world, seatInput, seatResult)) {
                return seatResult.refusedOutsidePocket ? GrabAttemptOutcome::OutsidePocket : GrabAttemptOutcome::Refused;
            }
            grabFingerPoseMeshTriangles = std::move(bodyFrameCapture.fingerPoseMeshTriangles);

            GrabFrozenCommitInput frozenCommitInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .bodyFrame = &bodyFrameCapture,
                .seatCapture = &seatResult,
                .surface = &surfaceEvidence,
                .objectBodyId = objectBodyId,
                .traceId = grabTraceId,
                .objectName = &objName,
                .rollback = seatRollback,
            };
            if (!commitFrozenGrabAuthority(frozenCommitInput)) {
                return GrabAttemptOutcome::Refused;
            }

            const GrabPostFreezeInput postFreezeInput{
                .handWorldTransform = &handWorldTransform,
                .proxy = &proxyPreparation,
                .meshCapture = &meshCaptureSetup,
                .mesh = &meshExtraction,
                .bodyFrame = &bodyFrameCapture,
                .seatCapture = &seatResult,
                .surface = &surfaceEvidence,
                .rootNode = rootNode,
                .collidableNode = collidableNode,
                .meshSourceNode = meshSourceNode,
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
            .fingerPoseMeshTriangles = &grabFingerPoseMeshTriangles,
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
            return GrabAttemptOutcome::Refused;
        }

        grabPreparationTransaction.commit();
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::GrabCommitSucceeded });
        clearPullRuntimeState();
        clearPullCatchIntent(grabbedFromPullCatch ? "pullCatchGrabbed" : "grabbed");

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), objectBodyId.value);
        return GrabAttemptOutcome::Grabbed;
    }

    Hand::HeldHandMotionSample Hand::recordHeldControllerMotionSample(
        const RE::NiTransform& handWorldTransform,
        float deltaTime)
    {
        HeldHandMotionSample handMotion{};
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

            _heldLocalHandVelocityHistory[_heldHandVelocityHistoryNext] = handMotion.localLinearVelocityHavok;
            _heldHandAngularVelocityHistory[_heldHandVelocityHistoryNext] = handMotion.angularVelocityRadiansPerSecond;
            _heldHandVelocityHistoryNext = (_heldHandVelocityHistoryNext + 1) % _heldLocalHandVelocityHistory.size();
            if (_heldHandVelocityHistoryCount < _heldLocalHandVelocityHistory.size()) {
                ++_heldHandVelocityHistoryCount;
            }
        }

        _previousHeldRawHandWorld = handWorldTransform;
        _previousHeldHandPositionHavok = currentHandPositionHavok;
        _hasPreviousHeldRawHandWorld = true;
        return handMotion;
    }

    void Hand::recordHeldObjectVelocitySample(RE::hknpWorld* world)
    {
        const auto compensationResult = applyHeldMotionCompensation(
            world,
            _savedObjectState.bodyId,
            _heldBodyIds,
            _heldDriveDecision.includeConnectedLinearVelocity);
        if (compensationResult.hasPrimaryVelocity) {
            _heldLocalLinearVelocityHistory[_heldLocalLinearVelocityHistoryNext] = compensationResult.primaryLocalLinearVelocity;
            _heldLocalLinearVelocityHistoryNext = (_heldLocalLinearVelocityHistoryNext + 1) % _heldLocalLinearVelocityHistory.size();
            if (_heldLocalLinearVelocityHistoryCount < _heldLocalLinearVelocityHistory.size()) {
                ++_heldLocalLinearVelocityHistoryCount;
            }
            _lastHeldObjectLocalLinearVelocityHavok = compensationResult.primaryLocalLinearVelocity;
            _hasLastHeldObjectLocalLinearVelocityHavok = true;
        }
    }

    void Hand::captureHeldReleaseMotion(
        RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime)
    {
        if (!isHolding() || !world) {
            return;
        }

        recordHeldControllerMotionSample(handWorldTransform, deltaTime);
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

    bool Hand::validateHeldObjectUpdate(RE::hknpWorld* world, const GrabReleaseContext& releaseContext)
    {
        if (_grabAuthorityProxyReleasePending.load(std::memory_order_acquire) || !_activeConstraint.isValid() || !_grabAuthorityProxy.isValid()) {
            ROCK_LOG_WARN(Hand, "{} hand release: proxy constraint authority marked grab invalid", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        const bool finalSeatMode =
            _grabFrame.seat.mode == GrabSeatMode::PinchPocket ||
            _grabFrame.seat.mode == GrabSeatMode::PalmPocket;
        const bool finalPivotSource = _grabFrame.pivotAuthority.source != GrabPivotAuthoritySource::None;
        if (!finalSeatMode || !finalPivotSource) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held grab has non-final authority seat={} source={}",
                handName(),
                grabSeatModeName(_grabFrame.seat.mode),
                grabPivotAuthoritySourceName(_grabFrame.pivotAuthority.source));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }

        return true;
    }

    struct Hand::HeldDriveUpdate
    {
        RE::NiTransform proxyAuthorityWorld{};
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform solvedBodyWorld{};
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
        bool heldBodyColliding = false;
        bool heldMotorContactSoftening = false;
    };

    bool Hand::updateHeldDrive(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        const GrabReleaseContext& releaseContext,
        HeldDriveUpdate& update)
    {
        update.proxyAuthorityWorld = handWorldTransform;
        update.hasProxyAuthorityFrame = resolveGrabAuthorityProxyFrame(
            world,
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
                "{} hand release: held object drive body readback failed before queuing grab authority bodyId={}",
                handName(),
                _savedObjectState.bodyId.value);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return false;
        }
        if (held_object_physics_math::instantDeviationExceeded(
                update.pivotTrackingErrorGameUnits, g_rockConfig.rockGrabMaxDeviation)) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object instant pivot deviation exceeded ({:.1f}gu > {:.1f}gu)",
                handName(),
                update.pivotTrackingErrorGameUnits,
                held_object_physics_math::instantDeviationReleaseThreshold(g_rockConfig.rockGrabMaxDeviation));
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return false;
        }

        held_scene_presentation::publishTargetTransport(
            _isLeft,
            world,
            _savedObjectState.bodyId.value,
            _grabFrame.traceId,
            update.desiredBodyWorld,
            update.solvedBodyWorld);

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
        _grabDeviationExceededSeconds = held_object_physics_math::advanceDeviationSeconds(
            _grabDeviationExceededSeconds,
            update.averageGrabDeviationGameUnits,
            g_rockConfig.rockGrabMaxDeviation,
            deltaTime);
        if (held_object_physics_math::deviationExceeded(
                _grabDeviationExceededSeconds, g_rockConfig.rockGrabMaxDeviationTime)) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object exceeded max deviation average ({:.1f}gu > {:.1f}gu for {:.2f}s)",
                handName(),
                update.averageGrabDeviationGameUnits,
                g_rockConfig.rockGrabMaxDeviation,
                _grabDeviationExceededSeconds);
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return false;
        }

        return true;
    }

    void Hand::updateHeldVisualPresentation(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float deltaTime,
        bool hasPivotTrackingError)
    {
        tickHeldBodyContact();
        /*
         * The rendered hand follows the held object from the first held frame:
         *     adjustedHand = heldObjectWorld * inverse(frozenObjectHandSpace)
         * This is visual-only; the dynamic grab drive stays the only object
         * motor authority. The first frame starts a short distance-mapped blend
         * from the tracked hand (or the visual-return pose) so the hand is not
         * snapped onto the object; once the blend completes it tracks exactly.
         */
        const bool publishVisualHand = _grabFrame.hasTelemetryCapture && hasPivotTrackingError;
        if (publishVisualHand) {
            RE::NiTransform heldVisualNodeWorld{};
            bool hasHeldVisualNodeWorld = false;
            if (_grabFrame.heldNode) {
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
                RE::NiTransform targetVisualHandWorld =
                    hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(heldVisualNodeWorld, _grabFrame.rawHandSpace);
                targetVisualHandWorld.scale = handWorldTransform.scale;

                if (!_hasGrabVisualHandTransform) {
                    const RE::NiTransform acquisitionStart =
                        _grabVisualReturn.active && isUsableGrabVisualTransform(_grabVisualReturn.lastApplied) ?
                        _grabVisualReturn.lastApplied :
                        handWorldTransform;
                    _grabVisualHandTransform = acquisitionStart;
                    _grabVisualHandLerpStartTransform = acquisitionStart;
                    _grabVisualHandLerpElapsedSeconds = 0.0f;
                    _grabVisualHandLerpDurationSeconds = g_rockConfig.rockGrabHandLerpEnabled ?
                        hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                            hand_visual_lerp_math::distanceGameUnits(acquisitionStart.translate, targetVisualHandWorld.translate),
                            g_rockConfig.rockGrabHandLerpTimeMin,
                            g_rockConfig.rockGrabHandLerpTimeMax,
                            g_rockConfig.rockGrabHandLerpMinDistance,
                            g_rockConfig.rockGrabHandLerpMaxDistance) :
                        0.0f;
                    _hasGrabVisualHandTransform = true;
                }

                _grabVisualHandLerpElapsedSeconds = hand_visual_lerp_math::advanceTimedBlendElapsed(
                    _grabVisualHandLerpElapsedSeconds,
                    deltaTime,
                    _grabVisualHandLerpDurationSeconds);
                const auto advancedVisual = hand_visual_lerp_math::blendTransformOverDuration(
                    _grabVisualHandLerpStartTransform,
                    targetVisualHandWorld,
                    _grabVisualHandLerpElapsedSeconds,
                    _grabVisualHandLerpDurationSeconds);
                const RE::NiTransform nextVisualHandWorld = advancedVisual.transform;
                const float visualHandLerpAlpha = hand_visual_lerp_math::timedBlendAlpha(
                    _grabVisualHandLerpElapsedSeconds,
                    _grabVisualHandLerpDurationSeconds);

                const float visualHandDeviationGameUnits =
                    pointDistanceGameUnits(nextVisualHandWorld.translate, handWorldTransform.translate);
                _grabVisualHandTransform = nextVisualHandWorld;
                if (applyGrabExternalHandWorldTransform(_isLeft, _grabVisualHandTransform)) {
                    _lastPublishedGrabVisualHandTransform = _grabVisualHandTransform;
                    _hasLastPublishedGrabVisualHandTransform = true;
                    clearGrabVisualReturn("active-grab-authority-acquired", false);
                }

                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} GRAB VISUAL HAND: relation=heldRelative visualOnly=yes lerpAlpha={:.2f} lerpDuration={:.3f}s target=({:.1f},{:.1f},{:.1f}) applied=({:.1f},{:.1f},{:.1f}) live=({:.1f},{:.1f},{:.1f}) deviation={:.2f}gu",
                    handName(),
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
    }

    void Hand::finalizeHeldObjectUpdate(RE::hknpWorld* world,
        float deltaTime,
        float forceFadeInTime,
        const HeldDriveUpdate& driveUpdate)
    {
        /*
         * One finger publish path: the pose solved at commit animates toward
         * its target every held frame. FRIK publishes joint values before the
         * root-flattened scene reflects them, so the local-transform layer
         * starts one held update after the first joint publish; surface-local
         * corrections built in the same update would read the previous finger
         * skeleton and could lock a closed or displaced pose. An unsolved pose
         * publishes its fallback once.
         */
        if (_hasGrabFingerPose && (_grabFingerPose.solved || !_grabFingerPosePublished)) {
            const bool publishLocalTransforms = _grabFingerPosePublished;
            const auto heldFingerPose =
                grab_finger_pose_runtime::resolveSurfaceAimObjectLocal(_grabFingerPose, driveUpdate.desiredObjectWorld);
            applyRockGrabHandPose(_isLeft,
                heldFingerPose,
                _grabFingerJointPose,
                _hasGrabFingerJointPose,
                _grabFingerLocalTransforms,
                _grabFingerLocalTransformMask,
                _hasGrabFingerLocalTransforms,
                deltaTime,
                publishLocalTransforms);
            _grabFingerPosePublished = true;
            if (publishLocalTransforms && _grabFingerLocalTransformFinalizePending) {
                _grabFingerLocalTransformFinalizePending = false;
                ROCK_LOG_DEBUG(Hand, "{} GRAB FINAL POSE: local transforms finalized after settled joint publication", handName());
            }
        }

        const float fadeDuration = std::max(forceFadeInTime, 0.0001f);
        const float grabFadeFactor = _grabFrame.fadeInGrabConstraint ? std::clamp(_grabStartTime / fadeDuration, 0.0f, 1.0f) : 1.0f;

        if (_state == HandState::HeldInit) {
            if (grabFadeFactor >= 0.999f) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::HeldFadeComplete });
                ROCK_LOG_DEBUG(Hand, "{} hand: HeldInit -> HeldBody ({} dynamic grab fade complete, {:.2f}s)", handName(), kHeldObjectDriveName, _grabStartTime);
            }
        }

        recordHeldObjectVelocitySample(world);

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
        const GrabReleaseContext& releaseContext)
    {
        if (!isHolding() || !world)
            return;
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GrabHeldObjectUpdate);
        if (!validateHeldObjectUpdate(world, releaseContext)) {
            return;
        }

        nearby_grab_damping::tickNearbyGrabDamping(world, _nearbyGrabDamping, deltaTime);

        suppressHandCollisionForGrab(world, bodyBoneColliders);
        if (_heldObjectIsLooseWeapon) {
            suppressBodyCollisionForHeldLooseWeapon(world, bodyBoneColliders);
        }

        _grabStartTime += held_object_physics_math::finitePositiveOrZero(deltaTime);

        const HeldHandMotionSample handMotion = recordHeldControllerMotionSample(handWorldTransform, deltaTime);
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
                driveUpdate)) {
            return;
        }
        const bool hasPivotTrackingError = driveUpdate.hasPivotTrackingError;
        updateHeldVisualPresentation(world, handWorldTransform, deltaTime, hasPivotTrackingError);
        finalizeHeldObjectUpdate(world, deltaTime, forceFadeInTime, driveUpdate);
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
                    updateConstraintGrabDriveMotors(
                        world,
                        driveDelta,
                        pending.forceFadeInTime,
                        pending.tauMin,
                        pending.authorityForceScale,
                        pending.heldBodyColliding);
                    angularDriveOk =
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

        if (shouldLog && g_rockConfig.rockDebugGrabFrameLogging) {
            std::uint32_t filterInfo = 0;
            const bool filterReadOk = havok_runtime::tryReadFilterInfo(world, proxyBodyId, filterInfo);
            ROCK_LOG_DEBUG(Hand,
                "{} PROXY GRAB AUTHORITY: seq={}/{} diag=bodyFrameConstraint+queuedTarget+generatedKeyframedProxy proxyBody={} constraint={} substep={}/{} dt={:.6f} resample={} rebases={} targetSrc={} target=({:.1f},{:.1f},{:.1f}) desiredBody=({:.1f},{:.1f},{:.1f}) angularAuthority={} angularRef={} solverAngular=ragdollAtom angularBudget={:.3f} pivotB=({:.2f},{:.2f},{:.2f}) err={:.2f}gu rotErr={:.2f}deg proxyDrive=driveToKeyFrame palmRef={} palmSrc={} palmMotion={} proxyVelSource={} proxyVel={:.3f}hk proxyAngVel={:.3f}rad/s longLever={:.1f}gu proxyRead={} proxySrc={} proxyMotion={} proxyErr={:.3f}gu/{:.2f}deg forceBudget={:.2f} colliding={} filterRead={} filter=0x{:08X} noContact={}",
                handName(),
                flushSequence,
                queuedSequence,
                proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount,
                timing.substepDeltaSeconds,
                grab_authority_source_clock::resampleActionName(resampleAction),
                resampleRebaseCount,
                pending.proxyFrameSource ? pending.proxyFrameSource : "unknown",
                pending.proxyWorld.translate.x,
                pending.proxyWorld.translate.y,
                pending.proxyWorld.translate.z,
                desiredBodyWorld.translate.x,
                desiredBodyWorld.translate.y,
                desiredBodyWorld.translate.z,
                grabAngularAuthorityName(angularAuthority),
                kGrabObjectRotationReferenceName,
                angularMotorBudget,
                activePivotBBodyLocalGame.x,
                activePivotBBodyLocalGame.y,
                activePivotBBodyLocalGame.z,
                pending.grabPositionErrorGameUnits,
                pending.grabRotationErrorDegrees,
                livePalmReferenceOk ? "ok" : "fail",
                body_frame::bodyFrameSourceCode(livePalmReference.source),
                livePalmReference.motionIndex,
                proxyVelocityTelemetryOk ? "palmMotion" : "computed",
                proxyLinearVelocityHavokMagnitude,
                proxyAngularVelocityRadiansPerSecond,
                _grabFrame.pivotAuthority.longLeverGameUnits,
                proxyReadbackBetweenOk ? "ok" : "fail",
                body_frame::bodyFrameSourceCode(proxyReadbackSourceBetween),
                proxyReadbackMotionIndexBetween,
                proxyReadbackBetweenPositionErrorGameUnits,
                proxyReadbackBetweenRotationErrorDegrees,
                pending.authorityForceScale,
                pending.heldBodyColliding ? "yes" : "no",
                filterReadOk ? "ok" : "fail",
                filterInfo,
                grab_authority_proxy::hasNoContactFilterInfo(filterInfo) ? "yes" : "no");
        }
    }

    GrabReleaseOutcome Hand::releaseGrabbedObject(
        RE::hknpWorld* world,
        GrabReleaseCollisionRestoreMode collisionRestoreMode,
        const GrabReleaseContext& releaseContext)
    {
        GrabReleaseOutcome outcome{};
        outcome.finalObjectRelease = releaseContext.finalObjectRelease;
        if (!isHolding()) {
            return outcome;
        }

        held_scene_presentation::clearHeldBodies(_isLeft);

        outcome.released = true;
        outcome.retainedRef = _savedObjectState.retainedRef;
        outcome.refr = outcome.retainedRef.get();
        outcome.formID = outcome.refr ? outcome.refr->GetFormID() : 0;

        ROCK_LOG_INFO(Hand,
            "{} hand RELEASE: bodyId={} constraintId={} proxyBody={} finalObjectRelease={} disposition={} reason={}",
            handName(),
            _savedObjectState.bodyId.value,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            releaseContext.finalObjectRelease ? "yes" : "no",
            releaseDispositionName(releaseContext.disposition),
            releaseContext.reason ? releaseContext.reason : "none");

        nearby_grab_damping::restoreNearbyGrabDamping(world, _nearbyGrabDamping);

        const bool captureReleaseVelocity =
            (releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop ||
                releaseContext.disposition == GrabReleaseDisposition::PendingInventoryTransfer ||
                releaseContext.disposition == GrabReleaseDisposition::PendingConsumeTransfer) &&
            releaseContext.applyCapturedReleaseVelocity &&
            releaseContext.finalObjectRelease && world && (_heldLocalLinearVelocityHistoryCount > 0 || _heldHandVelocityHistoryCount > 0);
        if (captureReleaseVelocity) {
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedObjectHistory{};
            const std::size_t historySize = _heldLocalLinearVelocityHistory.size();
            const std::size_t firstIndex = (_heldLocalLinearVelocityHistoryNext + historySize - _heldLocalLinearVelocityHistoryCount) % historySize;
            for (std::size_t i = 0; i < _heldLocalLinearVelocityHistoryCount; ++i) {
                orderedObjectHistory[i] = _heldLocalLinearVelocityHistory[(firstIndex + i) % historySize];
            }

            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedHandHistory{};
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedAngularHistory{};
            const std::size_t handHistorySize = _heldLocalHandVelocityHistory.size();
            const std::size_t firstHandIndex = (_heldHandVelocityHistoryNext + handHistorySize - _heldHandVelocityHistoryCount) % handHistorySize;
            for (std::size_t i = 0; i < _heldHandVelocityHistoryCount; ++i) {
                const std::size_t sourceIndex = (firstHandIndex + i) % handHistorySize;
                orderedHandHistory[i] = _heldLocalHandVelocityHistory[sourceIndex];
                orderedAngularHistory[i] = _heldHandAngularVelocityHistory[sourceIndex];
            }

            const RE::NiPoint3 objectLocalReleaseVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedObjectHistory, _heldLocalLinearVelocityHistoryCount);
            const RE::NiPoint3 handLocalReleaseVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedHandHistory, _heldHandVelocityHistoryCount);
            const RE::NiPoint3 handAngularVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedAngularHistory, _heldHandVelocityHistoryCount);

            RE::NiPoint3 tangentialVelocityHavok{};
            bool hasTangentialVelocity = false;
            RE::NiPoint3 releaseLeverOriginHavok = _lastHeldHandPositionHavok;
            bool hasReleaseLeverOrigin = _hasLastHeldHandPositionHavok;
            RE::NiPoint3 releaseCenterOfMassHavok{};
            bool hasReleaseCenterOfMass = false;
            RE::NiTransform releaseBodyWorld{};
            if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, releaseBodyWorld)) {
                releaseLeverOriginHavok =
                    gamePointToHavokPoint(transform_math::localPointToWorld(releaseBodyWorld, activeProxyConstraintPivotBLocalGame()));
                hasReleaseLeverOrigin = true;
            }
            if (hasReleaseLeverOrigin && lengthSquared(handAngularVelocity) > 0.000001f) {
                float comX = 0.0f;
                float comY = 0.0f;
                float comZ = 0.0f;
                if (havok_runtime::getBodyCOMWorld(world, _savedObjectState.bodyId, comX, comY, comZ)) {
                    releaseCenterOfMassHavok = RE::NiPoint3{ comX, comY, comZ };
                    hasReleaseCenterOfMass = true;
                    tangentialVelocityHavok = grab_held_response::computeTangentialVelocityFromAngularSwing(
                        handAngularVelocity,
                        releaseLeverOriginHavok,
                        releaseCenterOfMassHavok);
                    hasTangentialVelocity = lengthSquared(tangentialVelocityHavok) > 0.000001f;
                }
            }

            const RE::NiPoint3 releaseVelocity =
                grab_held_response::composeControllerReleaseVelocity(grab_held_response::ReleaseVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandLocalVelocity = _heldHandVelocityHistoryCount > 0,
                    .hasObjectLocalVelocity = _heldLocalLinearVelocityHistoryCount > 0,
                    .hasTangentialVelocity = hasTangentialVelocity,
                    .handLocalVelocityHavok = handLocalReleaseVelocity,
                    .objectLocalVelocityHavok = objectLocalReleaseVelocity,
                    .tangentialVelocityHavok = tangentialVelocityHavok,
                    .objectVelocityBlend = g_rockConfig.rockGrabThrowObjectVelocityBlend,
                    .tangentialVelocityScale = g_rockConfig.rockGrabThrowTangentialVelocityScale,
                    .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                    .maxVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                });
            const RE::NiPoint3 rawReleaseAngularVelocity =
                grab_held_response::composeControllerReleaseAngularVelocity(grab_held_response::ReleaseAngularVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandAngularVelocity = _heldHandVelocityHistoryCount > 0,
                    .handAngularVelocityRadiansPerSecond = handAngularVelocity,
                    .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                    .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                });
            const float releaseLongObjectAngularScale = grab_motion_controller::computeLongObjectAngularSpeedScale(
                g_rockConfig.rockGrabLongObjectAngularScalingEnabled,
                _grabFrame.pivotAuthority.longLeverGameUnits,
                g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits,
                g_rockConfig.rockGrabLongObjectMinAngularScale);
            const float releaseAngularVelocityCap = grab_motion_controller::computeAuthorityScaledAngularVelocityCap(
                g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                releaseLongObjectAngularScale);
            const RE::NiPoint3 releaseAngularVelocity =
                clampAngularVelocityVector(rawReleaseAngularVelocity, releaseAngularVelocityCap);
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
                "{} hand RELEASE VELOCITY: applied={} driveMode={} linearScope={} angularScope={} angularCap={:.3f} longScale={:.2f} handLocal=({:.3f},{:.3f},{:.3f}) objectLocal=({:.3f},{:.3f},{:.3f}) tangent=({:.3f},{:.3f},{:.3f}) angularRaw=({:.3f},{:.3f},{:.3f}) angularFinal=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) lever=({:.3f},{:.3f},{:.3f}) objectHistory={} handHistory={} multiplier={:.2f}",
                handName(),
                applyReleaseVelocity ? "yes" : "no",
                held_object_drive_policy::modeName(_heldDriveDecision.mode),
                _heldDriveDecision.includeConnectedLinearVelocity ? "bodySet" : "primaryOnly",
                _heldDriveDecision.includeConnectedAngularVelocity ? "bodySet" : "primaryOnly",
                releaseAngularVelocityCap,
                releaseLongObjectAngularScale,
                handLocalReleaseVelocity.x,
                handLocalReleaseVelocity.y,
                handLocalReleaseVelocity.z,
                objectLocalReleaseVelocity.x,
                objectLocalReleaseVelocity.y,
                objectLocalReleaseVelocity.z,
                tangentialVelocityHavok.x,
                tangentialVelocityHavok.y,
                tangentialVelocityHavok.z,
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
                _heldLocalLinearVelocityHistoryCount,
                _heldHandVelocityHistoryCount,
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
                heldBodyFlagLeaseOwner(this),
                releaseContext.finalObjectRelease);
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

            if (releaseContext.finalObjectRelease && releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop) {
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
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _activeConstraint.clear();
        clearGrabAuthorityProxyRuntime();
        _heldBodyIds.clear();
        _grabFrame.clear();
        _heldDriveDecision = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabDeviationExceededSeconds = 0.0f;
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
        _grabFingerSweepDebugCapture = {};
        _grabFingerSweepDebugObjectWorld = {};
        _hasGrabFingerSweepDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _grabFingerTriangleIndex.clear();
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _grabFingerLocalTransformFinalizePending = false;
        _hasGrabFingerPose = false;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _heldLocalHandVelocityHistory = {};
        _heldHandAngularVelocityHistory = {};
        _heldHandVelocityHistoryCount = 0;
        _heldHandVelocityHistoryNext = 0;
        _lastHeldObjectLocalLinearVelocityHavok = {};
        _hasLastHeldObjectLocalLinearVelocityHavok = false;
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

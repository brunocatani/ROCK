#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "api/ROCKProviderApi.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string_view>

namespace rock
{
    namespace
    {
        constexpr const char* PRIMARY_GRIP_TAG = "ROCK_WeaponPrimaryGrip";
        constexpr const char* PRIMARY_DETACH_TAG = "ROCK_WeaponPrimaryDetach";
        constexpr const char* SUPPORT_GRIP_TAG = "ROCK_WeaponSupportGrip";
        constexpr const char* WEAPON_NODE_OWNERSHIP_TAG = "ROCK_LeftFiringCarry";
        constexpr int GRIP_HAND_POSE_PRIORITY = 100;
        constexpr float SUPPORT_NORMAL_TWIST_FACTOR = 0.5f;
        constexpr std::uint32_t SCOPE_DRIVER_MISS_GRACE_FRAMES = 3;
        constexpr float SCOPE_ROOT_REBASE_DURATION_SECONDS = 0.075f;

        constexpr std::array<float, 15> BARREL_WRAP_POSE = { 0.85f, 0.80f, 0.75f, 0.35f, 0.30f, 0.25f, 0.30f, 0.25f, 0.20f, 0.35f, 0.30f, 0.25f, 0.40f, 0.35f, 0.30f };
        constexpr std::array<float, 15> HANDGUARD_CLAMP_POSE = { 0.75f, 0.72f, 0.68f, 0.45f, 0.42f, 0.38f, 0.46f, 0.42f, 0.38f, 0.48f, 0.44f, 0.40f, 0.54f, 0.48f, 0.42f };
        constexpr std::array<float, 15> FOREGRIP_POSE = { 0.90f, 0.86f, 0.82f, 0.70f, 0.66f, 0.60f, 0.74f, 0.68f, 0.62f, 0.72f, 0.66f, 0.60f, 0.66f, 0.58f, 0.50f };
        constexpr std::array<float, 15> PUMP_GRIP_POSE = { 0.82f, 0.78f, 0.72f, 0.58f, 0.54f, 0.48f, 0.60f, 0.56f, 0.50f, 0.62f, 0.56f, 0.50f, 0.58f, 0.50f, 0.44f };
        constexpr std::array<float, 15> MAGWELL_HOLD_POSE = { 0.58f, 0.52f, 0.46f, 0.40f, 0.36f, 0.32f, 0.42f, 0.38f, 0.34f, 0.42f, 0.38f, 0.34f, 0.44f, 0.38f, 0.32f };
        constexpr std::array<float, 15> RECEIVER_SUPPORT_POSE = { 0.46f, 0.40f, 0.34f, 0.34f, 0.30f, 0.26f, 0.36f, 0.32f, 0.28f, 0.36f, 0.32f, 0.28f, 0.36f, 0.30f, 0.24f };

        const std::array<float, 15>& poseValuesForGrip(WeaponGripPoseId poseId)
        {
            switch (poseId) {
            case WeaponGripPoseId::HandguardClamp:
                return HANDGUARD_CLAMP_POSE;
            case WeaponGripPoseId::VerticalForegrip:
            case WeaponGripPoseId::AngledForegrip:
                return FOREGRIP_POSE;
            case WeaponGripPoseId::PumpGrip:
                return PUMP_GRIP_POSE;
            case WeaponGripPoseId::MagwellHold:
                return MAGWELL_HOLD_POSE;
            case WeaponGripPoseId::ReceiverSupport:
                return RECEIVER_SUPPORT_POSE;
            case WeaponGripPoseId::BarrelWrap:
            case WeaponGripPoseId::None:
            default:
                return BARREL_WRAP_POSE;
            }
        }

        /*
         * The part-carry two-anchor solve feeds its own rotation back as the
         * next frame's base, chaining several float matrix products per frame.
         * Without re-orthonormalization the rotation's row norms decay and the
         * matrix acquires shear, which visibly stretches the weapon mesh and
         * collapses the grip geometry (telemetry: rigid grip separation decayed
         * ~0.05% per frame). Rows are the stored local axes.
         */
        RE::NiMatrix3 orthonormalizeStoredRotation(const RE::NiMatrix3& rotation)
        {
            const RE::NiPoint3 row0{ rotation.entry[0][0], rotation.entry[0][1], rotation.entry[0][2] };
            const RE::NiPoint3 row1{ rotation.entry[1][0], rotation.entry[1][1], rotation.entry[1][2] };

            const RE::NiPoint3 axis0 = weaponSolverNormalize(row0);
            RE::NiPoint3 axis2 = weaponSolverCross(axis0, row1);
            axis2 = weaponSolverNormalize(axis2);
            const RE::NiPoint3 axis1 = weaponSolverCross(axis2, axis0);

            RE::NiMatrix3 result = rotation;
            result.entry[0][0] = axis0.x;
            result.entry[0][1] = axis0.y;
            result.entry[0][2] = axis0.z;
            result.entry[1][0] = axis1.x;
            result.entry[1][1] = axis1.y;
            result.entry[1][2] = axis1.z;
            result.entry[2][0] = axis2.x;
            result.entry[2][1] = axis2.y;
            result.entry[2][2] = axis2.z;
            return result;
        }

        /*
         * Ambidextrous firing grip (left-hand fire) needs BOTH hFRIK blockers:
         * the finger-pose block (right hand must stop receiving FRIK's weapon
         * pose) and the weapon-node ownership block (FRIK must stop gluing the
         * weapon to the right hand). Fail closed to right-only behavior when
         * either is missing (older FRIK build) or the feature is disabled.
         */
        bool ambidextrousFiringGripTakeoverAvailable()
        {
            // The hFRIK cooperation reuses the native left-handed node
            // topology; combining it with the game's own left-handed setting
            // is untested/undefined, so the feature stands down there.
            return g_rockConfig.rockAmbidextrousFiringGripEnabled &&
                !f4vr::isLeftHandedMode() &&
                frik_visual_authority::canBlockPrimaryHandWeaponPose() &&
                frik_visual_authority::canBlockPrimaryWeaponNodeOwnership();
        }

        RE::NiMatrix3 makeMatrixFromRows(const RE::NiPoint3& row0, const RE::NiPoint3& row1, const RE::NiPoint3& row2)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = row0.x;
            result.entry[0][1] = row0.y;
            result.entry[0][2] = row0.z;
            result.entry[1][0] = row1.x;
            result.entry[1][1] = row1.y;
            result.entry[1][2] = row1.z;
            result.entry[2][0] = row2.x;
            result.entry[2][1] = row2.y;
            result.entry[2][2] = row2.z;
            return result;
        }

        RE::NiMatrix3 transposeMatrix(const RE::NiMatrix3& m)
        {
            RE::NiMatrix3 result{};
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    result.entry[row][col] = m.entry[col][row];
                }
            }
            return result;
        }

        RE::NiMatrix3 mulRowMatrices(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            RE::NiMatrix3 result{};
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    result.entry[row][col] =
                        a.entry[row][0] * b.entry[0][col] +
                        a.entry[row][1] * b.entry[1][col] +
                        a.entry[row][2] * b.entry[2][col];
                }
            }
            return result;
        }

        // Rows-as-axes convention: a hand-local vector maps to parent space
        // as u.x*row0 + u.y*row1 + u.z*row2.
        RE::NiPoint3 rotateLocalVectorThroughRows(const RE::NiMatrix3& m, const RE::NiPoint3& v)
        {
            return RE::NiPoint3{
                v.x * m.entry[0][0] + v.y * m.entry[1][0] + v.z * m.entry[2][0],
                v.x * m.entry[0][1] + v.y * m.entry[1][1] + v.z * m.entry[2][1],
                v.x * m.entry[0][2] + v.y * m.entry[1][2] + v.z * m.entry[2][2],
            };
        }

        RE::NiNode* sourceRootNodeOrFallback(RE::NiAVObject* sourceRoot, RE::NiNode* fallback)
        {
            if (sourceRoot) {
                if (auto* sourceNode = sourceRoot->IsNode()) {
                    return sourceNode;
                }
            }
            return fallback;
        }

        RE::NiPoint3 lerpPoint(const RE::NiPoint3& from, const RE::NiPoint3& to, float alpha)
        {
            const float t = (std::max)(0.0f, (std::min)(1.0f, alpha));
            return RE::NiPoint3{ from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t };
        }

        constexpr std::uint16_t SUPPORT_THUMB_LOCAL_TRANSFORM_MASK = 0x0007;
        constexpr float MIN_THUMB_OPPOSITION_DISTANCE = 0.001f;

        bool isFiniteRotation(const RE::NiMatrix3& rotation)
        {
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (!std::isfinite(rotation.entry[row][column])) {
                        return false;
                    }
                }
            }
            return true;
        }

        bool isFiniteTransform(const RE::NiTransform& transform)
        {
            return isFiniteRotation(transform.rotate) && std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
        }

        bool isUsableHandAuthorityTransform(const RE::NiTransform& transform)
        {
            // hFRIK uses 0.00001 as its intentional ScopeMenu hide scale.
            return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f;
        }

        struct NativeScopeCameraFollowCapture
        {
            RE::NiNode* camera{ nullptr };
            RE::NiTransform weaponWorldBefore{};
            RE::NiTransform cameraWorldBefore{};
            bool valid{ false };
        };

        struct ScopeHandAuthorityCleanupVisualSnapshot
        {
            RE::NiNode* weapon{ nullptr };
            RE::NiTransform weaponWorld{};
            RE::NiNode* scopeCamera{ nullptr };
            RE::NiTransform scopeCameraWorld{};
            bool weaponValid{ false };
            bool scopeCameraValid{ false };
        };

        ScopeHandAuthorityCleanupVisualSnapshot captureScopeHandAuthorityCleanupVisuals(RE::NiNode* weaponNode)
        {
            ScopeHandAuthorityCleanupVisualSnapshot snapshot{};
            if (weaponNode && isFiniteTransform(weaponNode->world)) {
                snapshot.weapon = weaponNode;
                snapshot.weaponWorld = weaponNode->world;
                snapshot.weaponValid = true;
            }

            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
            if (scopeCamera) {
                RE::NiTransform scopeCameraWorld = scopeCamera->world;
                if (scopeCamera->parent) {
                    scopeCameraWorld = transform_math::composeTransforms(scopeCamera->parent->world, scopeCamera->local);
                }
                if (isFiniteTransform(scopeCameraWorld)) {
                    snapshot.scopeCamera = scopeCamera;
                    snapshot.scopeCameraWorld = scopeCameraWorld;
                    snapshot.scopeCameraValid = true;
                }
            }
            return snapshot;
        }

        void restoreScopeHandAuthorityCleanupVisuals(const ScopeHandAuthorityCleanupVisualSnapshot& snapshot)
        {
            if (snapshot.weaponValid && snapshot.weapon) {
                if (snapshot.weapon->parent) {
                    snapshot.weapon->local = weapon_visual_authority_math::worldTargetToParentLocal(
                        snapshot.weapon->parent->world,
                        snapshot.weaponWorld);
                    f4vr::updateTransformsDown(snapshot.weapon, true);
                } else {
                    snapshot.weapon->local = snapshot.weaponWorld;
                    snapshot.weapon->world = snapshot.weaponWorld;
                    f4vr::updateTransformsDown(snapshot.weapon, false);
                }
            }

            if (snapshot.scopeCameraValid && snapshot.scopeCamera) {
                if (snapshot.scopeCamera->parent) {
                    snapshot.scopeCamera->local = weapon_visual_authority_math::worldTargetToParentLocal(
                        snapshot.scopeCamera->parent->world,
                        snapshot.scopeCameraWorld);
                    f4vr::updateTransforms(snapshot.scopeCamera);
                } else {
                    snapshot.scopeCamera->local = snapshot.scopeCameraWorld;
                    snapshot.scopeCamera->world = snapshot.scopeCameraWorld;
                }
            }
        }

        NativeScopeCameraFollowCapture captureNativeScopeCameraFollow(const RE::NiNode* weaponNode)
        {
            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
            if (!weaponNode || !scopeCamera || !isFiniteTransform(weaponNode->world)) {
                return {};
            }

            RE::NiTransform cameraWorld = scopeCamera->world;
            if (scopeCamera->parent) {
                cameraWorld = transform_math::composeTransforms(scopeCamera->parent->world, scopeCamera->local);
            }
            if (!isFiniteTransform(cameraWorld)) {
                return {};
            }

            return NativeScopeCameraFollowCapture{
                .camera = scopeCamera,
                .weaponWorldBefore = weaponNode->world,
                .cameraWorldBefore = cameraWorld,
                .valid = true,
            };
        }

        void applyNativeScopeCameraFollow(
            const NativeScopeCameraFollowCapture& capture,
            const RE::NiTransform& weaponWorldAfter)
        {
            if (!capture.valid || !capture.camera || !isFiniteTransform(weaponWorldAfter)) {
                return;
            }

            const RE::NiTransform targetCameraWorld = native_scope_camera_follow_math::followWeaponWorldChange(
                capture.weaponWorldBefore,
                weaponWorldAfter,
                capture.cameraWorldBefore);
            if (!isFiniteTransform(targetCameraWorld)) {
                return;
            }

            auto* scopeCamera = capture.camera;
            if (scopeCamera->parent) {
                const RE::NiTransform targetCameraLocal = weapon_visual_authority_math::worldTargetToParentLocal(
                    scopeCamera->parent->world,
                    targetCameraWorld);
                if (!isFiniteTransform(targetCameraLocal)) {
                    return;
                }
                scopeCamera->local = targetCameraLocal;
                f4vr::updateTransforms(scopeCamera);
                return;
            }

            scopeCamera->local = targetCameraWorld;
            scopeCamera->world = targetCameraWorld;
        }

        float lengthSquared(const RE::NiPoint3& value)
        {
            return value.x * value.x + value.y * value.y + value.z * value.z;
        }

        RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
        {
            const float valueLengthSquared = lengthSquared(value);
            if (std::isfinite(valueLengthSquared) && valueLengthSquared > 0.000001f) {
                const float invLength = 1.0f / std::sqrt(valueLengthSquared);
                return RE::NiPoint3{ value.x * invLength, value.y * invLength, value.z * invLength };
            }

            const float fallbackLengthSquared = lengthSquared(fallback);
            if (std::isfinite(fallbackLengthSquared) && fallbackLengthSquared > 0.000001f) {
                const float invLength = 1.0f / std::sqrt(fallbackLengthSquared);
                return RE::NiPoint3{ fallback.x * invLength, fallback.y * invLength, fallback.z * invLength };
            }

            return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        }

        struct LiveThumbTransform
        {
            RE::NiTransform world{};
            RE::NiTransform parentWorld{};
            RE::NiTransform local{};
            bool valid = false;
        };

        DirectSkeletonBoneReader& rootFlattenedTwoHandedReader()
        {
            static DirectSkeletonBoneReader reader;
            return reader;
        }

        const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        }

        const DirectSkeletonBoneEntry* findSnapshotBoneByTreeIndex(const DirectSkeletonBoneSnapshot& snapshot, int treeIndex)
        {
            if (treeIndex < 0) {
                return nullptr;
            }

            for (const auto& bone : snapshot.bones) {
                if (bone.treeIndex == treeIndex) {
                    return &bone;
                }
            }
            return nullptr;
        }

        bool resolveLiveThumbTransforms(bool isLeft, std::array<LiveThumbTransform, 3>& outNodes)
        {
            outNodes = {};

            DirectSkeletonBoneSnapshot snapshot{};
            if (!rootFlattenedTwoHandedReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                    snapshot)) {
                return false;
            }

            for (std::size_t segment = 0; segment < outNodes.size(); ++segment) {
                const char* boneName = root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, 0, segment);
                const auto* node = boneName ? findSnapshotBone(snapshot, boneName) : nullptr;
                const auto* parent = node ? findSnapshotBoneByTreeIndex(snapshot, node->parentTreeIndex) : nullptr;
                if (!node || !parent || !isFiniteTransform(node->world) || !isFiniteTransform(parent->world)) {
                    return false;
                }

                const RE::NiTransform local = transform_math::composeTransforms(transform_math::invertTransform(parent->world), node->world);
                if (!isFiniteTransform(local)) {
                    return false;
                }

                outNodes[segment] = LiveThumbTransform{
                    .world = node->world,
                    .parentWorld = parent->world,
                    .local = local,
                    .valid = true,
                };
            }
            return true;
        }

        bool buildAlternateThumbLocalTransforms(
            bool isLeft,
            const RE::NiPoint3& supportGripPivotWorldPoint,
            const RE::NiPoint3& gripWorldPoint,
            float thumbScalarValue,
            std::array<RE::NiTransform, 15>& outLocalTransforms,
            std::uint16_t& outMask)
        {
            /*
             * ROCK switches the support thumb to an alternate local-transform
             * target when the weapon mesh solve requires thumb opposition. FRIK
             * only exposes scalar curls by default, so derive local thumb targets
             * from the root-flattened chain and publish them through the local
             * pose API.
             */
            outLocalTransforms = {};
            outMask = 0;

            std::array<LiveThumbTransform, 3> thumbNodes{};
            if (!resolveLiveThumbTransforms(isLeft, thumbNodes)) {
                return false;
            }

            const float sanitizedThumbValue = std::isfinite(thumbScalarValue) ? std::clamp(thumbScalarValue, 0.0f, 1.0f) : 1.0f;
            const float oppositionStrength = std::clamp(0.45f + (1.0f - sanitizedThumbValue) * 0.55f, 0.45f, 1.0f);

            for (std::size_t segment = 0; segment < thumbNodes.size(); ++segment) {
                const auto& node = thumbNodes[segment];
                if (!node.valid) {
                    return false;
                }

                const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                    transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 toGrip = weapon_support_thumb_pose_policy::vectorToGripFromPredictedThumbNode(
                    node.world.translate,
                    supportGripPivotWorldPoint,
                    gripWorldPoint);
                if (lengthSquared(toGrip) <= MIN_THUMB_OPPOSITION_DISTANCE * MIN_THUMB_OPPOSITION_DISTANCE) {
                    return false;
                }

                const RE::NiPoint3 targetAxisWorld = normalizeOrFallback(toGrip, currentAxisWorld);
                const float dotToTarget = std::clamp(weaponSolverDot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                const float angle = std::acos(dotToTarget) * oppositionStrength;
                if (!std::isfinite(angle)) {
                    return false;
                }

                RE::NiMatrix3 rotationDelta = transform_math::makeIdentityRotation<RE::NiMatrix3>();
                if (angle > 0.0001f) {
                    RE::NiPoint3 axis = weaponSolverCross(currentAxisWorld, targetAxisWorld);
                    if (lengthSquared(axis) <= 0.000001f) {
                        axis = weaponSolverOrthogonalAxis(currentAxisWorld);
                    }
                    rotationDelta = weaponSolverAxisAngleStored<RE::NiMatrix3, RE::NiPoint3>(axis, angle);
                }

                const RE::NiMatrix3 targetWorldRotation =
                    weaponSolverApplyWorldRotationToStoredBasis<RE::NiMatrix3, RE::NiPoint3>(rotationDelta, node.world.rotate);
                RE::NiTransform localTransform = node.local;
                localTransform.rotate = transform_math::multiplyStoredRotations(targetWorldRotation, transform_math::transposeRotation(node.parentWorld.rotate));
                if (!isFiniteTransform(localTransform)) {
                    return false;
                }

                outLocalTransforms[segment] = localTransform;
                outMask = static_cast<std::uint16_t>(outMask | (1U << segment));
            }

            return outMask == SUPPORT_THUMB_LOCAL_TRANSFORM_MASK;
        }

        bool buildFullHandLocalTransformsForMeshPose(
            bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose& meshFingerPose,
            const frik_visual_authority::HandPoseData& handPose,
            std::array<RE::NiTransform, 15>& outLocalTransforms,
            std::uint16_t& outMask)
        {
            auto* api = frik_visual_authority::api();
            const bool canPublish =
                grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                    meshFingerPose.solved,
                    true,
                    api && api->getHandPoseLocalTransformsForPose != nullptr,
                    api && api->setHandPoseCustomLocalTransformsWithPriority != nullptr);
            if (!canPublish) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override skipped hand={} enabled={} api={} baselineApi={} publishApi={}",
                    isLeft ? "left" : "right",
                    g_rockConfig.rockGrabMeshLocalTransformPoseEnabled ? "yes" : "no",
                    api ? "yes" : "no",
                    (api && api->getHandPoseLocalTransformsForPose) ? "yes" : "no",
                    (api && api->setHandPoseCustomLocalTransformsWithPriority) ? "yes" : "no");
                return false;
            }

            frik_visual_authority::FingerLocalTransformOverride baseline{};
            if (!frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(isLeft), handPose, &baseline)) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: full-hand local transform override failed hand={} reason=baseline-query", isLeft ? "left" : "right");
                return false;
            }

            frik_visual_authority::FingerLocalTransformOverride corrected{};
            const char* failureReason = "unknown";
            if (!grab_finger_local_transform_runtime::buildSurfaceCorrectedLocalTransforms(isLeft,
                    meshFingerPose,
                    baseline,
                    grab_finger_local_transform_runtime::Options{
                        .enabled = g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                        .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                        .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                        .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                        .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                        .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                        .thumbSurfaceSafetyEnabled = g_rockConfig.rockGrabThumbSurfaceSafetyEnabled,
                        .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
                    },
                    corrected,
                    &failureReason)) {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: full-hand local transform override failed hand={} reason={}",
                    isLeft ? "left" : "right",
                    failureReason ? failureReason : "unknown");
                return false;
            }

            outMask = corrected.enabledMask;
            for (std::size_t i = 0; i < outLocalTransforms.size(); ++i) {
                outLocalTransforms[i] = corrected.localTransforms[i];
            }
            return outMask == grab_finger_local_transform_math::kFullFingerLocalTransformMask;
        }

    }

    static bool tryGetRootFlattenedHandBoneTransform(bool isLeft, RE::NiTransform& outTransform)
    {
        outTransform = {};
        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedTwoHandedReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            return false;
        }

        const auto* handBone = findSnapshotBone(snapshot, isLeft ? "LArm_Hand" : "RArm_Hand");
        if (!handBone || !isUsableHandAuthorityTransform(handBone->world)) {
            return false;
        }

        outTransform = handBone->world;
        return true;
    }

    bool TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(bool isLeft, RE::NiPoint3& outPalmWorld, RE::NiTransform& outHandWorld)
    {
        outPalmWorld = {};
        if (!tryGetRootFlattenedHandBoneTransform(isLeft, outHandWorld)) {
            return false;
        }
        outPalmWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(outHandWorld, isLeft);
        return true;
    }

    void TwoHandedGrip::refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        const bool scopeWasOpen = _scopeMenuOpenThisFrame;
        const bool scopeStateChanged = _scopeMenuOpenThisFrame != frameInput.scopeMenuOpen;
        _scopeMenuOpenThisFrame = frameInput.scopeMenuOpen;
        const bool scopeClosedThisFrame = scopeStateChanged && scopeWasOpen && !_scopeMenuOpenThisFrame;

        if (scopeStateChanged) {
            // Never resume a pre-menu visual interpolation after hFRIK restores
            // its visible body. The weapon solver itself remains continuous.
            resetLockedHandVisualLerp();
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: native scope hand-frame mode={} leftCache={} rightCache={}",
                _scopeMenuOpenThisFrame ? "frik-driver" : "root-flattened",
                _scopeSafeHandFrames[0].hasDriverToHandLocal ? "ready" : "missing",
                _scopeSafeHandFrames[1].hasDriverToHandLocal ? "ready" : "missing");
            if (_scopeMenuOpenThisFrame) {
                // hFRIK's clear call cannot restore a hand after its root has
                // already been collapsed. Defer removal of the persistent API
                // authority entries until the visible root returns.
                _scopeHandAuthorityCleanupPending = true;
            }
        }

        const float frameDeltaSeconds = std::isfinite(dt) && dt > 0.0f ? (std::min)(dt, 0.1f) : (1.0f / 90.0f);
        const auto refreshHand = [this, scopeClosedThisFrame, frameDeltaSeconds](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
            ScopeSafeHandFrameState& state = _scopeSafeHandFrames[isLeft ? 0u : 1u];
            state.currentHandWorldValid = false;

            RE::NiTransform rootHandWorld{};
            const bool rootHandValid = !_scopeMenuOpenThisFrame &&
                                       tryGetRootFlattenedHandBoneTransform(isLeft, rootHandWorld);
            const bool driverValid = driverFrame.valid &&
                                     isUsableHandAuthorityTransform(driverFrame.world);
            RE::NiTransform reconstructedHandWorld{};
            bool reconstructedHandValid = false;
            if (driverValid && state.hasDriverToHandLocal) {
                reconstructedHandWorld = scope_safe_hand_frame_math::resolveHandWorld(
                    driverFrame.world,
                    state.driverToHandLocal);
                reconstructedHandValid = isUsableHandAuthorityTransform(reconstructedHandWorld);
            }
            const auto resolutionMode = scope_safe_hand_frame_math::resolveMode(
                _scopeMenuOpenThisFrame,
                rootHandValid,
                reconstructedHandValid,
                state.hasLastHandWorld,
                state.consecutiveDriverMissFrames,
                SCOPE_DRIVER_MISS_GRACE_FRAMES);

            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::RootFlattened) {
                const bool recentScopedHandAvailable = state.hasLastHandWorld &&
                                                       state.consecutiveDriverMissFrames < SCOPE_DRIVER_MISS_GRACE_FRAMES;
                if (scopeClosedThisFrame && (reconstructedHandValid || recentScopedHandAvailable)) {
                    // The previous ROCK output is the continuity authority.
                    // hFRIK may resume non-scope damping from a stale internal
                    // sample on this exact edge even though its driver is finite.
                    const RE::NiTransform& continuityHandWorld = recentScopedHandAvailable ?
                                                                      state.lastHandWorld :
                                                                      reconstructedHandWorld;
                    const RE::NiTransform rootRebaseLocalStart = transform_math::composeTransforms(
                        transform_math::invertTransform(rootHandWorld),
                        continuityHandWorld);
                    if (isUsableHandAuthorityTransform(rootRebaseLocalStart)) {
                        state.rootRebaseLocalStart = rootRebaseLocalStart;
                        state.rootRebaseElapsedSeconds = 0.0f;
                        state.rootRebaseActive = true;
                    }
                }
                state.consecutiveDriverMissFrames = 0;

                RE::NiTransform resolvedHandWorld = rootHandWorld;
                if (state.rootRebaseActive) {
                    const RE::NiTransform identity = transform_math::makeIdentityTransform<RE::NiTransform>();
                    const float rebaseAlpha = scope_safe_hand_frame_math::rebaseAlpha(
                        state.rootRebaseElapsedSeconds,
                        SCOPE_ROOT_REBASE_DURATION_SECONDS);
                    const RE::NiTransform rebase = scope_safe_hand_frame_math::interpolateRebaseTransform(
                        state.rootRebaseLocalStart,
                        identity,
                        rebaseAlpha);
                    const RE::NiTransform rebasedHandWorld = transform_math::composeTransforms(rootHandWorld, rebase);
                    if (isUsableHandAuthorityTransform(rebasedHandWorld)) {
                        resolvedHandWorld = rebasedHandWorld;
                    } else {
                        state.rootRebaseActive = false;
                    }
                    if (rebaseAlpha >= 1.0f) {
                        state.rootRebaseActive = false;
                    } else {
                        state.rootRebaseElapsedSeconds = (std::min)(
                            SCOPE_ROOT_REBASE_DURATION_SECONDS,
                            state.rootRebaseElapsedSeconds + frameDeltaSeconds);
                    }
                }

                state.currentHandWorld = resolvedHandWorld;
                state.currentHandWorldValid = true;
                state.lastHandWorld = resolvedHandWorld;
                state.hasLastHandWorld = true;

                if (driverValid) {
                    const RE::NiTransform driverToHandLocal =
                        scope_safe_hand_frame_math::captureDriverToHandLocal(driverFrame.world, resolvedHandWorld);
                    if (isUsableHandAuthorityTransform(driverToHandLocal)) {
                        state.driverToHandLocal = driverToHandLocal;
                        state.hasDriverToHandLocal = true;
                    }
                }
                return;
            }

            state.rootRebaseActive = false;
            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::DriverReconstructed) {
                state.consecutiveDriverMissFrames = 0;
                state.currentHandWorld = reconstructedHandWorld;
                state.currentHandWorldValid = true;
                state.lastHandWorld = reconstructedHandWorld;
                state.hasLastHandWorld = true;
                return;
            }

            // A transient hFRIK arm-driver miss must not become an ownership
            // release/reacquire loop. Hold only a few frames: a real driver
            // loss must still fail closed instead of pinning the weapon in the
            // world indefinitely.
            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::LastKnown) {
                ++state.consecutiveDriverMissFrames;
                state.currentHandWorld = state.lastHandWorld;
                state.currentHandWorldValid = true;
            } else if (_scopeMenuOpenThisFrame) {
                state.consecutiveDriverMissFrames = SCOPE_DRIVER_MISS_GRACE_FRAMES;
            }
        };

        refreshHand(true, frameInput.leftHandDriverFrame);
        refreshHand(false, frameInput.rightHandDriverFrame);

        // Capture the fully adjusted hFRIK root hands first. Clearing an API
        // wrist tag asks hFRIK to restore its tracked arm but does not rerun the
        // later weapon-position adjustment pass, so cleanup must not mutate the
        // canonical frames used by this exit-frame rebase/weapon solve.
        if (!_scopeMenuOpenThisFrame &&
            _scopeHandAuthorityCleanupPending &&
            frik_visual_authority::isAvailable()) {
            const ScopeHandAuthorityCleanupVisualSnapshot visualSnapshot =
                captureScopeHandAuthorityCleanupVisuals(weaponNode);
            bool cleared = true;
            for (const bool isLeft : { true, false }) {
                cleared &= frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, handFromBool(isLeft));
                cleared &= frik_visual_authority::clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, handFromBool(isLeft));
                cleared &= frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_DETACH_TAG, handFromBool(isLeft));
            }
            restoreScopeHandAuthorityCleanupVisuals(visualSnapshot);
            if (cleared) {
                _scopeHandAuthorityCleanupPending = false;
                ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: cleared deferred native-scope hand authority entries");
            }
        }
    }

    bool TwoHandedGrip::tryGetSolverHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        const ScopeSafeHandFrameState& state = _scopeSafeHandFrames[isLeft ? 0u : 1u];
        if (!state.currentHandWorldValid) {
            outTransform = {};
            return false;
        }
        outTransform = state.currentHandWorld;
        return true;
    }

    RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }

    RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::localPointToWorld(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, localPos);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::localPointToWorld(supportAttachmentRoot->world, grip.gripSourceLocal);
        }
        return weaponLocalToWorld(grip.gripLocal, weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        return worldToWeaponLocal(resolvePartGripWorld(grip, weaponNode), weaponNode);
    }

    RE::NiPoint3 TwoHandedGrip::resolvePartGripNormalWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            const RE::NiPoint3 supportNormalWorld = transform_math::localVectorToWorld(supportAttachmentRoot->world, grip.normalSourceLocal);
            return transform_math::worldVectorToLocal(weaponNode->world, supportNormalWorld);
        }
        return grip.normalLocal;
    }

    RE::NiTransform TwoHandedGrip::resolvePartGripHandWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::composeTransforms(supportAttachmentRoot->world, grip.handSourceLocal);
        }
        if (!weaponNode) {
            return RE::NiTransform{};
        }
        return weapon_support_authority_policy::buildVisualOnlySupportHandWorld(weaponNode->world, grip.handWeaponLocal);
    }

    RE::NiAVObject* TwoHandedGrip::resolveCurrentSupportAttachmentRoot(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (!grip.hasSourceFrames || !grip.attachmentRoot || !weaponNode) {
            return nullptr;
        }
        return actor_equipment_grab::nodeContainsNode(weaponNode, grip.attachmentRoot, 64) ? grip.attachmentRoot : nullptr;
    }

    void TwoHandedGrip::update(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const EquippedWeaponGripFrameInput& frameInput,
        float dt,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool sidearmHybridEligible,
        bool primaryDetachEnabled)
    {
        _hasSolvedWeaponTransform = false;
        _firingGripReattachHoverInsideRadius = false;
        _firingGripReattachHoverHandIsLeft = _firingHandIsLeft;
        _leftFiringTriggerAxis = std::isfinite(frameInput.leftTriggerAxis) ? std::clamp(frameInput.leftTriggerAxis, 0.0f, 1.0f) : 0.0f;

        refreshScopeSafeHandFrames(weaponNode, frameInput, dt);

        if (!runtime_state::isLocalSkeletonReady() || !weaponNode) {
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            return;
        }

        EquippedWeaponGripFrameInput stableFrameInput = frameInput;
        const auto primaryReleaseDecision = equipped_weapon_manual_ownership_policy::debouncePrimaryGripRelease(
            _primaryReleaseDebounce,
            frameInput.primaryGripInput.held);
        stableFrameInput.primaryGripInput.held = primaryReleaseDecision.retained;
        stableFrameInput.primaryGripInput.released = primaryReleaseDecision.releaseConfirmed;

        if (isManualOwnershipActive() &&
            !reconcileCollisionGeneration(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponCollision)) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: clearing authority because equipped weapon instance changed active={:016X} current={:016X}",
                _activeEquippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey);
            transitionToInactive(false);
            return;
        }

        /*
         * Support-side routing follows the CURRENT firing hand: the support
         * hand is whichever physical hand does not own the firing grip. All
         * grip math below is weapon-relative; the hands only choose roles.
         */
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const WeaponInteractionContact& supportWeaponContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionDecision decision = routeWeaponInteraction(supportWeaponContact, supportRuntimeState);
        const bool supportTouchingSupport = decision.kind == WeaponInteractionKind::SupportGrip;
        RE::NiNode* interactionWeaponNode = sourceRootNodeOrFallback(decision.interactionRoot, weaponNode);
        const bool supportGripHeld = supportHandIsLeft ? stableFrameInput.leftGripHeld : stableFrameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? stableFrameInput.leftHandHoldingObject : stableFrameInput.rightHandHoldingObject;
        const EquippedWeaponPrimaryGripInput& primaryGripInput = stableFrameInput.primaryGripInput;

        switch (_state) {
        case TwoHandedState::Inactive:
            if (supportTouchingSupport && !supportHandHoldingObject) {
                transitionToTouching(interactionWeaponNode, decision);
            }
            break;

        case TwoHandedState::Touching:
            if (supportHandHoldingObject) {
                _state = TwoHandedState::Inactive;
                break;
            }
            if (supportTouchingSupport) {
                _touchFrames = 0;
            } else {
                _touchFrames++;
                if (_touchFrames > TOUCH_TIMEOUT_FRAMES) {
                    _state = TwoHandedState::Inactive;
                    break;
                }
            }
            if (weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    sidearmHybridEligible,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            }
            break;

        case TwoHandedState::Gripping:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because weapon generation changed during support grip");
                transitionToInactive(false);
            } else if (!providerPartAuthorityStillCurrent(partGrip(supportHandIsLeft), currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because provider weapon-part target is no longer current");
                transitionToInactive(false);
            } else if (providerPartTargetNewlyMatchesGrip(partGrip(supportHandIsLeft), currentWeaponGenerationKey)) {
                // The still-held grab recaptures next frame under the new
                // provider resolution (e.g. an AttachOnly whitelist armed
                // mid-hold).
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: releasing support grip to recapture under newly matched provider weapon-part target");
                transitionToInactive(ownsWeaponTransform());
            } else if (!supportRuntimeState.supportGripAllowed) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing authority because offhand reservation disabled support grip");
                transitionToInactive(false);
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                const auto releaseAction = weapon_two_handed_grip_math::resolveSupportReleaseManualAction(primaryDetachEnabled, primaryGripInput.held);
                if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::KeepPrimaryOwnership) {
                    transitionToPrimaryOnly(
                        _activeWeaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "support-released-primary-held");
                } else if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::DropEquippedWeapon) {
                    requestEquippedWeaponDrop(
                        "support-released-primary-not-held",
                        equipped_weapon_drop_policy::sourceForSupportRelease(primaryGripInput.released));
                } else {
                    transitionToInactive(ownsWeaponTransform());
                }
            } else if (primaryDetachEnabled && !primaryGripInput.held && tryPromoteSupportGripToFiringGrip(_activeWeaponNode)) {
                // The support hand was wrapped over the firing grip when the
                // firing hand opened: it takes over the SAME weapon-relative
                // grip in place (seamless hand switch, pistol shooting-cup
                // flow). State is PrimaryOnly under the new firing hand.
            } else if (primaryDetachEnabled &&
                       _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver &&
                       !primaryGripInput.held) {
                if (transitionToPartCarry()) {
                    updatePartCarryGrip(
                        _activeWeaponNode,
                        dt,
                        stableFrameInput,
                        leftWeaponContact,
                        rightWeaponContact,
                        weaponCollision,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        leftRuntimeState,
                        rightRuntimeState);
                }
            } else {
                updateGripping(_activeWeaponNode, dt);
            }
            break;

        case TwoHandedState::PartCarry:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(_activeWeaponGenerationKey, currentWeaponGenerationKey)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because weapon generation changed");
                transitionToInactive(false);
            } else if (!primaryDetachEnabled) {
                transitionToInactive(ownsWeaponTransform());
            } else {
                updatePartCarryGrip(
                    _activeWeaponNode,
                    dt,
                    stableFrameInput,
                    leftWeaponContact,
                    rightWeaponContact,
                    weaponCollision,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    leftRuntimeState,
                    rightRuntimeState);
            }
            break;

        case TwoHandedState::PrimaryOnly:
            if (!_activeWeaponNode) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing primary-only authority because active weapon source root is unavailable");
                transitionToInactive(false);
            } else if (!primaryDetachEnabled) {
                transitionToInactive(false);
            } else if (supportTouchingSupport && weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    sidearmHybridEligible,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            } else {
                updatePrimaryOnlyGrip(_activeWeaponNode, currentEquippedWeaponOwnershipKey, primaryGripInput);
            }
            break;
        }

        // Enforce the left-firing weapon-node ownership contract after every
        // state/role transition this frame (idempotent; also the parent
        // watchdog for engine-side re-attach).
        syncFiringHandWeaponNodeOwnership(weaponNode);
    }

    void TwoHandedGrip::reset()
    {
        _equippedWeaponDropRequest = {};
        _hapticEvents = {};
        _firingGripReattachHoverInsideRadius = false;
        _scopeSafeHandFrames = {};
        _scopeHandAuthorityCleanupPending = _scopeHandAuthorityCleanupPending || _scopeMenuOpenThisFrame;
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikPrimaryWeaponPose();
        _hasRightFiringHandCanonicalWeaponLocal = false;
        _rightFiringHandCanonicalGenerationKey = 0;
        _rightFiringHandCanonicalWeaponLocal = {};
        if (_state != TwoHandedState::Inactive) {
            transitionToInactive(false);
            _scopeMenuOpenThisFrame = false;
            return;
        }
        _scopeMenuOpenThisFrame = false;
        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryGripSeparationWorld = 0.0f;
        _primaryGripLocal = {};
        _lockedGripSeparationWorld = 0.0f;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = false;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        resetLockedHandVisualLerp();
    }

    bool TwoHandedGrip::ownsWeaponTransform() const
    {
        return (_state == TwoHandedState::Gripping || _state == TwoHandedState::PartCarry) &&
               weapon_support_authority_policy::supportGripOwnsWeaponTransform(_authorityMode);
    }

    bool TwoHandedGrip::getSolvedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasSolvedWeaponTransform) {
            return false;
        }
        outTransform = _lastSolvedWeaponTransform;
        return true;
    }

    bool TwoHandedGrip::getDebugAuthoritySnapshot(TwoHandedGripDebugSnapshot& outSnapshot) const
    {
        const auto& leftGrip = partGrip(true);
        const auto& rightGrip = partGrip(false);
        if (!_hasSolvedWeaponTransform || !_activeWeaponNode) {
            return false;
        }
        if (!_hasFiringHandWeaponLocal && !leftGrip.active && !rightGrip.active) {
            return false;
        }

        outSnapshot.weaponWorld = _lastSolvedWeaponTransform;
        if (rightGrip.active) {
            outSnapshot.rightRequestedHandWorld = resolvePartGripHandWorld(rightGrip, _activeWeaponNode);
            outSnapshot.rightGripWorld = resolvePartGripWorld(rightGrip, _activeWeaponNode);
        } else {
            outSnapshot.rightRequestedHandWorld = transform_math::composeTransforms(_lastSolvedWeaponTransform, _primaryHandWeaponLocal);
            outSnapshot.rightGripWorld = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        }
        if (leftGrip.active) {
            outSnapshot.leftRequestedHandWorld = resolvePartGripHandWorld(leftGrip, _activeWeaponNode);
            outSnapshot.leftGripWorld = resolvePartGripWorld(leftGrip, _activeWeaponNode);
        } else {
            outSnapshot.leftRequestedHandWorld = RE::NiTransform{};
            outSnapshot.leftGripWorld = RE::NiPoint3{};
        }
        return true;
    }

    void TwoHandedGrip::resetLockedHandVisualLerp()
    {
        _primaryHandVisualLerp = {};
        partGrip(true).visualLerp = {};
        partGrip(false).visualLerp = {};
    }

    RE::NiTransform TwoHandedGrip::resolveLockedHandVisualTarget(
        const RE::NiTransform& targetWorld,
        const RE::NiTransform* liveHandWorld,
        float dt,
        LockedHandVisualLerpState& state)
    {
        /*
         * Two-handed weapon smoothing is visual-only. The weapon solver keeps
         * immediate aim authority; this only eases the FRIK external hand target
         * into the locked hand-to-weapon relation captured at grip start.
         */
        if (!g_rockConfig.rockWeaponSupportGripHandLerpEnabled) {
            state = {};
            return targetWorld;
        }

        if (!state.active) {
            const RE::NiTransform startWorld = (liveHandWorld && isFiniteTransform(*liveHandWorld)) ? *liveHandWorld : targetWorld;
            const float initialDistance =
                hand_visual_lerp_math::distanceGameUnits(startWorld.translate, targetWorld.translate);
            const float durationSeconds =
                hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                    initialDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                    g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
            if (durationSeconds <= 0.0f) {
                state = {};
                state.lastAlpha = 1.0f;
                return targetWorld;
            }

            state.active = true;
            state.startWorld = startWorld;
            state.elapsedSeconds = 0.0f;
            state.durationSeconds = durationSeconds;
            state.lastAlpha = 0.0f;
        }

        state.elapsedSeconds =
            hand_visual_lerp_math::advanceTimedBlendElapsed(state.elapsedSeconds, dt, state.durationSeconds);
        const auto blended =
            hand_visual_lerp_math::blendTransformOverDuration(state.startWorld, targetWorld, state.elapsedSeconds, state.durationSeconds);
        state.lastAlpha = hand_visual_lerp_math::timedBlendAlpha(state.elapsedSeconds, state.durationSeconds);
        return blended.transform;
    }

    void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode, const WeaponInteractionDecision& decision)
    {
        if (!weaponNode) {
            _state = TwoHandedState::Inactive;
            return;
        }

        _state = TwoHandedState::Touching;
        _touchFrames = 0;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: touching weapon='{}' bodyId={} partKind={} pose={} interactionRoot={:x} sourceRoot={:x} generation={:016X}",
            weaponNode->name.c_str(),
            decision.bodyId,
            static_cast<int>(decision.partKind),
            static_cast<int>(decision.gripPose),
            reinterpret_cast<std::uintptr_t>(decision.interactionRoot),
            reinterpret_cast<std::uintptr_t>(decision.sourceRoot),
            decision.weaponGenerationKey);
    }

    bool TwoHandedGrip::capturePartGrip(
        bool isLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        const WeaponProviderPartAuthority& providerPartAuthority)
    {
        if (!weaponNode) {
            return false;
        }
        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(decision.weaponGenerationKey, _activeWeaponGenerationKey)) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: part grip capture skipped because contact generation is stale hand={}", isLeft ? "left" : "right");
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(isLeft, handTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: part grip capture skipped because authoritative hand transforms are unavailable hand={}", isLeft ? "left" : "right");
            return false;
        }

        WeaponPartGrip& grip = partGrip(isLeft);
        grip = {};
        RE::NiAVObject* supportAttachmentRoot = decision.sourceRoot ? decision.sourceRoot : static_cast<RE::NiAVObject*>(weaponNode);
        grip.gripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
        grip.partKind = decision.partKind;
        grip.attachmentRoot = supportAttachmentRoot;
        grip.providerPartAuthority = providerPartAuthority.active ? providerPartAuthority : WeaponProviderPartAuthority{};
        grip.attachOnly = weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
            grip.providerPartAuthority.active,
            grip.providerPartAuthority.grabMode);
        grip.contactBodyId = decision.bodyId;
        grip.reloadRole = decision.reloadRole;
        grip.socketRole = decision.socketRole;
        grip.actionRole = decision.actionRole;
        grip.weaponGenerationKey = decision.weaponGenerationKey;
        grip.gripSequence = ++_gripCaptureSequence;
        {
            // The routing decision carries no support role or authored source
            // name; both come from the evidence descriptor keyed by the
            // contact body, matching the provider target-query construction.
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* descriptorSourceNode = nullptr;
            if (weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(decision.bodyId, descriptor, descriptorSourceNode) &&
                descriptor.weaponGenerationKey == decision.weaponGenerationKey) {
                grip.supportRole = descriptor.semantic.supportGripRole;
                grip.omodFormId = descriptor.omodFormId;
                grip.attachPointFormId = descriptor.semantic.attachPointFormId;
                grip.classificationSource = descriptor.semantic.classificationSource;
                const std::size_t copyLength = (std::min)(descriptor.sourceName.size(), grip.sourceName.size() - 1);
                std::memcpy(grip.sourceName.data(), descriptor.sourceName.data(), copyLength);
                grip.sourceName[copyLength] = '\0';
            } else if (grip.providerPartAuthority.active) {
                grip.supportRole = static_cast<WeaponSupportGripRole>(grip.providerPartAuthority.supportRole);
                grip.sourceName = grip.providerPartAuthority.sourceName;
                grip.sourceName[grip.sourceName.size() - 1] = '\0';
            }
        }

        RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, isLeft);
        RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handTransform, isLeft);

        std::vector<TriangleData> triangles;
        const bool cachedTrianglesFound = weaponCollision.tryBuildSupportGripEvidenceTriangles(decision.bodyId, weaponNode, triangles);

        GrabPoint grabPoint;
        bool meshFound = false;
        if (!triangles.empty()) {
            meshFound = findClosestGrabPoint(triangles,
                palmPos,
                palmDir,
                g_rockConfig.rockGrabLateralWeight,
                g_rockConfig.rockGrabDirectionalWeight,
                grabPoint,
                g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
        }

        if (meshFound) {
            grip.gripLocal = worldToWeaponLocal(grabPoint.position, weaponNode);
            grip.grabNormalWorld = grabPoint.normal;
        } else {
            grip.gripLocal = worldToWeaponLocal(palmPos, weaponNode);
            grip.grabNormalWorld = palmDir;
        }
        const RE::NiPoint3 gripWorldPoint = meshFound ? grabPoint.position : palmPos;
        const RE::NiTransform adjustedHandTransform =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palmPos, gripWorldPoint);
        grip.handWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        grip.hasHandWeaponLocal = true;
        grip.normalLocal = transform_math::worldVectorToLocal(weaponNode->world, palmDir);
        if (supportAttachmentRoot) {
            grip.gripSourceLocal = transform_math::worldPointToLocal(supportAttachmentRoot->world, gripWorldPoint);
            grip.normalSourceLocal = transform_math::worldVectorToLocal(supportAttachmentRoot->world, palmDir);
            grip.handSourceLocal = transform_math::composeTransforms(transform_math::invertTransform(supportAttachmentRoot->world), adjustedHandTransform);
            grip.attachmentWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), supportAttachmentRoot->world);
            grip.hasSourceFrames = true;
            grip.hasAttachmentWeaponLocal = true;
        }

        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPosePtr = nullptr;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            auto fingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(gripWorldPoint, grip.grabNormalWorld);
            fingerPoseTargets.useSeatPointForMissingTargets = false;
            fingerPoseTargets.useWholeMeshForMissingTargets = true;
            root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
            const auto* liveFingerSnapshotPtr =
                root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, liveFingerSnapshot) ? &liveFingerSnapshot : nullptr;
            const auto solvedFingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                triangles, handTransform, isLeft, palmPos, fingerPoseTargets, g_rockConfig.rockGrabFingerMinValue,
                g_rockConfig.rockGrabMaxTriangleDistance, true, liveFingerSnapshotPtr,
                g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits);
            if (solvedFingerPose.solved) {
                meshFingerPose = solvedFingerPose;
                meshFingerPosePtr = &meshFingerPose;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    solvedFingerPose.hitCount,
                    solvedFingerPose.candidateTriangleCount,
                    solvedFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(solvedFingerPose.selectedThumbLane));
                if (solvedFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        solvedFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        solvedFingerPose.thumbPrimaryCurve.value,
                        solvedFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        solvedFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        solvedFingerPose.thumbAlternateCurve.value,
                        solvedFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        solvedFingerPose.thumbSidePadCurve.hit ? "yes" : "no",
                        solvedFingerPose.thumbSidePadCurve.value,
                        solvedFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(solvedFingerPose.selectedThumbLane));
                }

                const bool canPublishAlternateThumb = !g_rockConfig.rockGrabMeshLocalTransformPoseEnabled &&
                    weapon_support_thumb_pose_policy::shouldPublishAlternateThumbLocalOverride(
                    solvedFingerPose.solved,
                    solvedFingerPose.usedAlternateThumbCurve,
                    frik_visual_authority::api() && frik_visual_authority::api()->setHandPoseCustomLocalTransformsWithPriority);
                if (canPublishAlternateThumb) {
                    std::array<RE::NiTransform, 15> localTransforms{};
                    std::uint16_t localTransformMask = 0;
                    if (buildAlternateThumbLocalTransforms(isLeft, palmPos, gripWorldPoint, meshFingerPose.values[0], localTransforms, localTransformMask)) {
                        grip.fingerLocalTransforms = localTransforms;
                        grip.fingerLocalTransformMask = localTransformMask;
                        grip.hasFingerLocalTransforms = true;
                        ROCK_LOG_DEBUG(Weapon,
                            "TwoHandedGrip: alternate thumb local transform override prepared hand={} mask=0x{:04X}",
                            isLeft ? "left" : "right",
                            grip.fingerLocalTransformMask);
                    } else {
                        ROCK_LOG_WARN(Weapon, "TwoHandedGrip: alternate thumb selected but local transform override could not be built");
                    }
                }
            }
        }

        setSupportGripPose(isLeft, grip.gripPose, meshFingerPosePtr);
        if (meshFingerPosePtr && grip.hasFingerPose) {
            std::array<RE::NiTransform, 15> localTransforms{};
            std::uint16_t localTransformMask = 0;
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            if (buildFullHandLocalTransformsForMeshPose(
                    isLeft,
                    *meshFingerPosePtr,
                    handPose,
                    localTransforms,
                    localTransformMask)) {
                grip.fingerLocalTransforms = localTransforms;
                grip.fingerLocalTransformMask = localTransformMask;
                grip.hasFingerLocalTransforms = true;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: full-hand local transform override prepared hand={} mask=0x{:04X}",
                    isLeft ? "left" : "right",
                    grip.fingerLocalTransformMask);
            }
        }

        grip.visualLerp = {};
        grip.active = true;
        if (isLeft) {
            _hapticEvents.leftPartGripCaptured = true;
        } else {
            _hapticEvents.rightPartGripCaptured = true;
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: part grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) meshGrab={} triangles={} cachedTriangles={} partKind={} pose={} generation={:016X}",
            isLeft ? "left" : "right",
            weaponNode->name.c_str(),
            grip.gripLocal.x,
            grip.gripLocal.y,
            grip.gripLocal.z,
            meshFound ? "YES" : "FALLBACK",
            triangles.size(),
            cachedTrianglesFound ? "yes" : "no",
            static_cast<int>(grip.partKind),
            static_cast<int>(grip.gripPose),
            _activeWeaponGenerationKey);
        return true;
    }

    void TwoHandedGrip::lockPartGripToWeaponRoot(bool isLeft)
    {
        /*
         * Part-carry feeds its own solved weapon transform back as the next
         * frame's base, so grips must resolve exclusively through the captured
         * weapon-root frames while it is active. Following live part-node
         * chains lets any per-frame part animation integrate into a steady
         * carry drift and pulls the locked hand visuals apart (verified by
         * telemetry: rigid-weapon grip separation grew frame over frame).
         */
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.hasSourceFrames = false;
        grip.hasAttachmentWeaponLocal = false;
    }

    void TwoHandedGrip::releasePartGrip(bool isLeft, const char* reason)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active) {
            return;
        }
        clearSupportGripPose(isLeft);
        grip = {};
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: part grip released hand={} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }

    void TwoHandedGrip::transitionToGripping(
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode,
        bool sidearmHybridEligible,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponProviderPartAuthority& providerPartAuthority)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::TwoHandedGripStart);

        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0) {
            transitionToInactive(false);
            return;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool primaryHandIsLeft = _firingHandIsLeft;

        _authorityMode = supportAuthorityMode;
        _activeWeaponNode = weaponNode;
        _activeWeaponGenerationKey = decision.weaponGenerationKey;
        _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
        _weaponNodeLocalBaseline = weaponNode->local;
        _hasWeaponNodeLocalBaseline = true;
        _primaryGripConfidence = 0.0f;
        _hasFiringHandWeaponLocal = false;
        resetLockedHandVisualLerp();
        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);

        killFrikOffhandGrip();

        RE::NiTransform primaryTransform{};
        if (!tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: support grip start skipped because authoritative hand transforms are unavailable");
            restoreFrikOffhandGrip();
            return;
        }

        const RE::NiPoint3 primaryPalmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        _primaryGripLocal = worldToWeaponLocal(primaryPalmPos, weaponNode);
        _primaryGripConfidence = 1.0f;
        _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), primaryTransform);
        _hasFiringHandWeaponLocal = true;
        _firingGripSequence = ++_gripCaptureSequence;
        // A right-hand capture here rides FRIK's authored carry: snapshot it
        // as the canonical hold that left takeovers apply mirrored.
        rememberRightFiringHandCanonicalFrame();

        /*
         * Sidearm hybrid: at capture the firing grip point is the primary palm,
         * so support-palm-to-firing-grip distance decides whether this grab is
         * a shooting cup (visual-only) or a manipulation grip (full two-handed
         * authority). Missing support hand transforms fail closed to
         * visual-only, the pre-hybrid sidearm behavior.
         */
        if (sidearmHybridEligible &&
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
            RE::NiTransform supportTransform{};
            if (tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
                const RE::NiPoint3 supportPalmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);
                const RE::NiPoint3 supportToGrip = sub(primaryPalmPos, supportPalmPos);
                const float supportPalmToGripDistance = std::sqrt(dot(supportToGrip, supportToGrip));
                if (std::isfinite(supportPalmToGripDistance)) {
                    _authorityMode = weapon_support_authority_policy::resolveSidearmHybridSupportAuthorityMode(
                        supportPalmToGripDistance,
                        g_rockConfig.rockSidearmVisualOnlySupportGripRadius);
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: sidearm hybrid support grip distance={:.2f} radius={:.2f} mode={}",
                        supportPalmToGripDistance,
                        g_rockConfig.rockSidearmVisualOnlySupportGripRadius,
                        _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                            "visual-only" :
                            "full-authority");
                }
            }
        }

        if (!capturePartGrip(supportHandIsLeft, weaponNode, decision, weaponCollision, providerPartAuthority)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: support grip start skipped because part grip capture failed");
            restoreFrikOffhandGrip();
            return;
        }

        const RE::NiPoint3 supportGripWorldPoint = resolvePartGripWorld(partGrip(supportHandIsLeft), weaponNode);
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryPalmPos);
        _lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        _state = TwoHandedState::Gripping;
        _rotationBlend = 0.0f;
        _gripLogCounter = 0;

        if (_firingHandIsLeft) {
            // clearPrimaryGripPose above dropped the left firing pose; the
            // left hand still occupies the firing grip in Gripping.
            publishLeftFiringHandPose();
        }

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, partKind={}, pose={}, authorityMode={}, generation={:016X}",
            weaponNode->name.c_str(),
            _primaryGripLocal.x,
            _primaryGripLocal.y,
            _primaryGripLocal.z,
            supportGrip.gripLocal.x,
            supportGrip.gripLocal.y,
            supportGrip.gripLocal.z,
            _lockedGripSeparationWorld,
            _scopeMenuOpenThisFrame ? "frik-driver-reconstructed" : "root-flattened",
            _primaryGripConfidence,
            static_cast<int>(supportGrip.partKind),
            static_cast<int>(supportGrip.gripPose),
            static_cast<int>(_authorityMode),
            _activeWeaponGenerationKey);
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        // Return equipped weapon-node ownership to FRIK/native before FRIK
        // pose restoration; must run while _activeWeaponNode is still known.
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikOffhandGrip();
        restoreFrikPrimaryWeaponPose();
        bool restoredWeaponTransformAvailable = false;
        RE::NiTransform restoredWeaponTransform{};
        if (publishRestoredWeaponTransform && _hasWeaponNodeLocalBaseline && _activeWeaponNode) {
            if (_activeWeaponNode->parent) {
                restoredWeaponTransform = transform_math::composeTransforms(_activeWeaponNode->parent->world, _weaponNodeLocalBaseline);
            } else {
                restoredWeaponTransform = _weaponNodeLocalBaseline;
            }
            restoredWeaponTransformAvailable = true;
        }

        _state = TwoHandedState::Inactive;
        _touchFrames = 0;
        _rotationBlend = 0.0f;
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryGripSeparationWorld = 0.0f;
        _primaryGripLocal = {};
        _lockedGripSeparationWorld = 0.0f;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = publishRestoredWeaponTransform && restoredWeaponTransformAvailable;
        if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = restoredWeaponTransform;
        }
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        resetLockedHandVisualLerp();
        // The firing-hand role is grip-session state: outside manual
        // ownership the weapon is FRIK/native-carried by the right hand.
        _firingHandIsLeft = false;

        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: grip released");
    }

    void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode, float dt)
    {
        if (_authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport) {
            updateVisualOnlySupportGrip(weaponNode, dt);
            return;
        }

        updateFullWeaponAuthorityGrip(weaponNode, dt);
    }

    bool TwoHandedGrip::providerPartAuthorityStillCurrent(WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey)
    {
        if (!grip.providerPartAuthority.active) {
            return true;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.providerPartAuthority.weaponGenerationKey) {
            return false;
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
        query.weaponGenerationKey = grip.providerPartAuthority.weaponGenerationKey;
        query.bodyId = grip.providerPartAuthority.bodyId;
        query.partKind = grip.providerPartAuthority.partKind;
        query.reloadRole = grip.providerPartAuthority.reloadRole;
        query.supportRole = grip.providerPartAuthority.supportRole;
        query.socketRole = grip.providerPartAuthority.socketRole;
        query.actionRole = grip.providerPartAuthority.actionRole;
        query.sourceRoot = grip.providerPartAuthority.sourceRoot;
        std::memcpy(query.sourceName, grip.providerPartAuthority.sourceName.data(), grip.providerPartAuthority.sourceName.size());
        query.sourceName[sizeof(query.sourceName) - 1] = '\0';

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        if (!::rock::provider::resolveWeaponPartTargetV1(query, resolution)) {
            return false;
        }
        return resolution.matched != 0 &&
               resolution.ownerToken == grip.providerPartAuthority.ownerToken &&
               resolution.groupId == grip.providerPartAuthority.groupId &&
               static_cast<std::uint32_t>(resolution.grabMode) == grip.providerPartAuthority.grabMode;
    }

    bool TwoHandedGrip::providerPartTargetNewlyMatchesGrip(const WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey) const
    {
        /*
         * Upgrade twin of providerPartAuthorityStillCurrent: a support grip
         * captured WITHOUT provider authority whose own part NOW resolves to
         * a matched provider target — a consumer armed its whitelist while
         * the hand was already holding the part (PAPER_Redux: pulling the
         * trigger mid-hold switches an authority grab to attach-only). The
         * caller releases the grip; the still-held grab recaptures within a
         * couple of frames under the new resolution, through the same
         * re-resolve path the downgrade direction uses when a target
         * disappears mid-grip. The query is built from the grip's own
         * captured contact identity, not the live contact, so a flickering
         * contact cannot convert against the wrong part.
         */
        if (!grip.active || grip.providerPartAuthority.active) {
            return false;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.weaponGenerationKey) {
            return false;
        }

        ::rock::provider::RockProviderWeaponPartTargetQueryV1 query{};
        query.weaponGenerationKey = grip.weaponGenerationKey;
        query.bodyId = grip.contactBodyId;
        query.partKind = static_cast<std::uint32_t>(grip.partKind);
        query.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
        query.supportRole = static_cast<std::uint32_t>(grip.supportRole);
        query.socketRole = static_cast<std::uint32_t>(grip.socketRole);
        query.actionRole = static_cast<std::uint32_t>(grip.actionRole);
        std::memcpy(query.sourceName, grip.sourceName.data(), grip.sourceName.size());
        query.sourceName[sizeof(query.sourceName) - 1] = '\0';

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        return ::rock::provider::resolveWeaponPartTargetV1(query, resolution) && resolution.matched != 0;
    }

    bool TwoHandedGrip::tryRebindPartGripToCurrentGeneration(
        WeaponPartGrip& grip,
        std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        if (!grip.active) {
            return true;
        }

        WeaponCollisionProfileEvidenceDescriptor bestDescriptor{};
        RE::NiAVObject* bestSourceNode = nullptr;
        int bestScore = 0;
        float bestDistanceSquared = (std::numeric_limits<float>::max)();
        bool bestAmbiguous = false;
        const std::string_view capturedSourceName{ grip.sourceName.data() };
        const auto distanceSquaredToBounds = [&grip](const WeaponEvidenceBounds3& bounds) {
            if (!bounds.valid) {
                return (std::numeric_limits<float>::max)();
            }
            const auto axisDistance = [](float value, float minimum, float maximum) {
                if (value < minimum) {
                    return minimum - value;
                }
                if (value > maximum) {
                    return value - maximum;
                }
                return 0.0f;
            };
            const float dx = axisDistance(grip.gripLocal.x, bounds.min.x, bounds.max.x);
            const float dy = axisDistance(grip.gripLocal.y, bounds.min.y, bounds.max.y);
            const float dz = axisDistance(grip.gripLocal.z, bounds.min.z, bounds.max.z);
            return dx * dx + dy * dy + dz * dz;
        };
        const auto bodyCount = weaponCollision.getWeaponBodyCount();
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const auto bodyId = weaponCollision.getWeaponBodyIdAtomic(i);
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* sourceNode = nullptr;
            if (!weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode) ||
                !descriptor.valid || descriptor.weaponGenerationKey != currentWeaponGenerationKey) {
                continue;
            }

            const bool sourcePointerMatches = sourceNode && sourceNode == grip.attachmentRoot;
            const bool sourceNameMatches = !capturedSourceName.empty() && descriptor.sourceName == capturedSourceName;
            if ((!sourcePointerMatches && !sourceNameMatches) || descriptor.semantic.partKind != grip.partKind) {
                continue;
            }
            if (grip.omodFormId != 0 && descriptor.omodFormId != grip.omodFormId) {
                continue;
            }
            if (grip.attachPointFormId != 0 && descriptor.semantic.attachPointFormId != grip.attachPointFormId) {
                continue;
            }

            const int score = sourcePointerMatches ? 2 : 1;
            const float distanceSquared = distanceSquaredToBounds(descriptor.localBoundsGame);
            constexpr float kDistanceTieEpsilon = 0.0001f;
            if (score > bestScore ||
                (score == bestScore && distanceSquared + kDistanceTieEpsilon < bestDistanceSquared)) {
                bestScore = score;
                bestDistanceSquared = distanceSquared;
                bestAmbiguous = false;
                bestDescriptor = descriptor;
                bestSourceNode = sourceNode;
            } else if (score == bestScore &&
                       (distanceSquared == bestDistanceSquared ||
                           (std::isfinite(distanceSquared) && std::isfinite(bestDistanceSquared) &&
                               std::fabs(distanceSquared - bestDistanceSquared) <= kDistanceTieEpsilon))) {
                bestAmbiguous = true;
            }
        }

        if (bestScore == 0 || bestAmbiguous) {
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: part grip rebind failed closed hand={} generation={:016X} source='{}' part={} omod={:08X} attachPoint={:08X} reason={}",
                (&grip == &_partGrips[0]) ? "left" : "right",
                currentWeaponGenerationKey,
                capturedSourceName,
                static_cast<std::uint32_t>(grip.partKind),
                grip.omodFormId,
                grip.attachPointFormId,
                bestScore == 0 ? "missing" : "ambiguous");
            return false;
        }

        grip.weaponGenerationKey = currentWeaponGenerationKey;
        grip.contactBodyId = bestDescriptor.bodyId;
        grip.attachmentRoot = bestSourceNode ? bestSourceNode : grip.attachmentRoot;
        grip.partKind = bestDescriptor.semantic.partKind;
        grip.reloadRole = bestDescriptor.semantic.reloadRole;
        grip.supportRole = bestDescriptor.semantic.supportGripRole;
        grip.socketRole = bestDescriptor.semantic.socketRole;
        grip.actionRole = bestDescriptor.semantic.actionRole;
        grip.omodFormId = bestDescriptor.omodFormId;
        grip.attachPointFormId = bestDescriptor.semantic.attachPointFormId;
        grip.classificationSource = bestDescriptor.semantic.classificationSource;
        const auto copyLength = (std::min)(bestDescriptor.sourceName.size(), grip.sourceName.size() - 1);
        std::memcpy(grip.sourceName.data(), bestDescriptor.sourceName.data(), copyLength);
        grip.sourceName[copyLength] = '\0';

        if (grip.providerPartAuthority.active) {
            grip.providerPartAuthority.weaponGenerationKey = currentWeaponGenerationKey;
            grip.providerPartAuthority.bodyId = bestDescriptor.bodyId;
            grip.providerPartAuthority.sourceRoot = reinterpret_cast<std::uintptr_t>(bestSourceNode);
            grip.providerPartAuthority.partKind = static_cast<std::uint32_t>(grip.partKind);
            grip.providerPartAuthority.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
            grip.providerPartAuthority.supportRole = static_cast<std::uint32_t>(grip.supportRole);
            grip.providerPartAuthority.socketRole = static_cast<std::uint32_t>(grip.socketRole);
            grip.providerPartAuthority.actionRole = static_cast<std::uint32_t>(grip.actionRole);
            std::memcpy(
                grip.providerPartAuthority.sourceName.data(),
                grip.sourceName.data(),
                grip.providerPartAuthority.sourceName.size());
        }
        return true;
    }

    bool TwoHandedGrip::reconcileCollisionGeneration(
        RE::NiNode* currentWeaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision)
    {
        if (!equipped_weapon_manual_ownership_policy::canPreserveManualOwnership(
                _activeEquippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey,
                currentWeaponGenerationKey,
                _state != TwoHandedState::PrimaryOnly)) {
            return false;
        }
        if (currentWeaponGenerationKey == 0) {
            // PrimaryOnly rides the native firing-hand attach and can retain
            // ownership while the complete collider set is still building.
            _activeWeaponNode = currentWeaponNode;
            _activeWeaponGenerationKey = 0;
            _weaponNodeLocalBaseline = currentWeaponNode->local;
            _hasWeaponNodeLocalBaseline = true;
            return true;
        }

        const bool generationChanged = _activeWeaponGenerationKey != currentWeaponGenerationKey;
        const bool weaponRootChanged = _activeWeaponNode != currentWeaponNode;
        if (generationChanged || weaponRootChanged) {
            const auto previousGeneration = _activeWeaponGenerationKey;
            _activeWeaponNode = currentWeaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            if (weaponRootChanged) {
                _weaponNodeLocalBaseline = currentWeaponNode->local;
                _hasWeaponNodeLocalBaseline = true;
            }
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: preserving manual ownership across collision rebuild oldGeneration={:016X} newGeneration={:016X} ownership={:016X} rootChanged={}",
                previousGeneration,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponRootChanged ? "yes" : "no");
        }

        if (generationChanged || weaponRootChanged) {
            for (auto& grip : _partGrips) {
                if (grip.active && !tryRebindPartGripToCurrentGeneration(grip, currentWeaponGenerationKey, weaponCollision)) {
                    return false;
                }
            }
        }
        return true;
    }

    void TwoHandedGrip::updateFullWeaponAuthorityGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool primaryHandIsLeft = _firingHandIsLeft;
        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);

        _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

        RE::NiTransform primaryTransform{};
        RE::NiTransform supportTransform{};
        if (!tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform) || !tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return;
        }

        RE::NiPoint3 primaryController = computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        RE::NiPoint3 supportController = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);

        const RE::NiPoint3 currentSupportWorld = resolvePartGripWorld(supportGrip, weaponNode);
        const RE::NiPoint3 currentPrimaryGripWorld = transform_math::localPointToWorld(weaponNode->world, _primaryGripLocal);
        const float currentGripSeparationWorld = std::sqrt(dot(sub(currentSupportWorld, currentPrimaryGripWorld), sub(currentSupportWorld, currentPrimaryGripWorld)));
        const float lockedGripSeparationWorld = supportGrip.hasSourceFrames ? currentGripSeparationWorld : _lockedGripSeparationWorld;
        const RE::NiPoint3 supportGripLocal = resolvePartGripWeaponLocal(supportGrip, weaponNode);
        const RE::NiPoint3 lockedSupportControllerTarget = makeLockedSupportGripTarget(
            primaryController,
            supportController,
            currentSupportWorld,
            lockedGripSeparationWorld,
            0.001f);
        const RE::NiPoint3 blendedSupportTarget = lerpPoint(currentSupportWorld, lockedSupportControllerTarget, _rotationBlend);

        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
        solverInput.weaponWorldTransform = weaponNode->world;
        solverInput.primaryGripLocal = _primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryController;
        solverInput.supportTargetWorld = blendedSupportTarget;
        solverInput.supportNormalLocal = resolvePartGripNormalWeaponLocal(supportGrip, weaponNode);
        solverInput.supportNormalTargetWorld = computePalmNormalFromHandBasis(supportTransform, supportHandIsLeft);
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;

        const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        if (!solved.solved) {
            return;
        }

        if (!applyWeaponVisualAuthority(weaponNode, solved.weaponWorldTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return;
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);
        if (_firingHandIsLeft) {
            publishLeftFiringHandPose();
        }

        const bool applyPrimaryHandAuthority = weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
        if (!applyLockedHandVisualAuthority(weaponNode, applyPrimaryHandAuthority, true, dt, &primaryTransform, &supportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing support grip because ROCK locked hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        RE::NiPoint3 primaryGripFinal = transform_math::localPointToWorld(_lastSolvedWeaponTransform, _primaryGripLocal);
        RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal), sub(primaryGripFinal, offhandGripFinal)));
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: blend={:.2f}, separation={:.1f}gu, "
                "primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                _rotationBlend,
                separation,
                primaryGripFinal.x,
                primaryGripFinal.y,
                primaryGripFinal.z,
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                _primaryHandVisualLerp.lastAlpha,
                _primaryHandVisualLerp.durationSeconds,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }
    }

    bool TwoHandedGrip::transitionToPartCarry()
    {
        if (_state == TwoHandedState::PartCarry) {
            return true;
        }

        clearPrimaryGripPose(_firingHandIsLeft);
        if (!blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: primary detach skipped because hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }
        _primaryHandVisualLerp = {};
        partGrip(!_firingHandIsLeft).visualLerp = {};
        lockPartGripToWeaponRoot(!_firingHandIsLeft);
        _rotationBlend = 1.0f;
        _partCarryPivotIsLeft = !_firingHandIsLeft;
        _partCarryGripSeparationWorld = 0.0f;
        _state = TwoHandedState::PartCarry;
        _hapticEvents.firingGripDetached = true;
        _hapticEvents.firingGripDetachedHandIsLeft = _firingHandIsLeft;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand detached; part grips own equipped weapon authority");
        return true;
    }

    bool TwoHandedGrip::republishPartCarryWeaponTransform(RE::NiNode* weaponNode)
    {
        if (_state != TwoHandedState::PartCarry || !_hasSolvedWeaponTransform || !weaponNode) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, _lastSolvedWeaponTransform);
    }

    bool TwoHandedGrip::beginPrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0 || _state != TwoHandedState::Inactive) {
            return false;
        }

        if (!transitionToPrimaryOnly(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                "primary-grip-start")) {
            return false;
        }
        // Only a fresh grab pulses; transitionToPrimaryOnly is also reached
        // from support-release paths where the firing grip never changed.
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        _firingGripSequence = ++_gripCaptureSequence;
        return true;
    }

    EquippedWeaponManualDropRequest TwoHandedGrip::consumeEquippedWeaponDropRequest()
    {
        const EquippedWeaponManualDropRequest request = _equippedWeaponDropRequest;
        _equippedWeaponDropRequest = {};
        return request;
    }

    void TwoHandedGrip::getHandGripReport(bool isLeft, HandGripReport& outReport) const
    {
        outReport = {};
        const bool isFiringHand = isLeft == _firingHandIsLeft;
        const WeaponPartGrip& grip = partGrip(isLeft);
        const auto kind = weapon_part_grip_report_policy::resolveHandGripKind(
            _state == TwoHandedState::Gripping,
            _state == TwoHandedState::PartCarry,
            _state == TwoHandedState::PrimaryOnly,
            isFiringHand,
            grip.active,
            grip.attachOnly,
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport);
        outReport.kind = kind;
        if (kind == weapon_part_grip_report_policy::HandGripKind::None) {
            return;
        }

        outReport.active = true;
        if (kind == weapon_part_grip_report_policy::HandGripKind::FiringGrip) {
            // In PrimaryOnly the weapon rides the FRIK-native hand attach and
            // ROCK holds no captured hand-to-weapon frame; hasHandPartLocal
            // stays false there by design.
            outReport.gripSequence = _firingGripSequence;
            outReport.weaponGenerationKey = _activeWeaponGenerationKey;
            outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(_activeWeaponNode);
            outReport.hasHandPartLocal = _hasFiringHandWeaponLocal;
            outReport.handPartLocal = _primaryHandWeaponLocal;
            return;
        }

        outReport.attachOnly = grip.attachOnly;
        outReport.gripSequence = grip.gripSequence;
        outReport.weaponGenerationKey = grip.weaponGenerationKey != 0 ? grip.weaponGenerationKey : _activeWeaponGenerationKey;
        outReport.bodyId = grip.contactBodyId;
        outReport.partKind = static_cast<std::uint32_t>(grip.partKind);
        outReport.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
        outReport.supportRole = static_cast<std::uint32_t>(grip.supportRole);
        outReport.socketRole = static_cast<std::uint32_t>(grip.socketRole);
        outReport.actionRole = static_cast<std::uint32_t>(grip.actionRole);
        outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(grip.attachmentRoot);
        if (grip.providerPartAuthority.active) {
            outReport.providerOwnerToken = grip.providerPartAuthority.ownerToken;
            outReport.providerGroupId = grip.providerPartAuthority.groupId;
            outReport.providerGrabMode = grip.providerPartAuthority.grabMode;
        }
        outReport.hasHandPartLocal = grip.hasSourceFrames || grip.hasHandWeaponLocal;
        outReport.handPartLocalIsSourceLocal = grip.hasSourceFrames;
        outReport.handPartLocal = grip.hasSourceFrames ? grip.handSourceLocal : grip.handWeaponLocal;
        outReport.sourceName = grip.sourceName;
        outReport.omodFormId = grip.omodFormId;
        outReport.attachPointFormId = grip.attachPointFormId;
        outReport.classificationSource = static_cast<std::uint32_t>(grip.classificationSource);
    }

    TwoHandedGripHapticEvents TwoHandedGrip::consumeHapticEvents()
    {
        const TwoHandedGripHapticEvents events = _hapticEvents;
        _hapticEvents = {};
        return events;
    }

    bool TwoHandedGrip::transitionToPrimaryOnly(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const char* reason)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0) {
            return false;
        }

        const bool primaryHandIsLeft = _firingHandIsLeft;
        const bool supportHandIsLeft = !_firingHandIsLeft;

        if (_state == TwoHandedState::Inactive) {
            _activeWeaponNode = weaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
            _weaponNodeLocalBaseline = weaponNode->local;
            _hasWeaponNodeLocalBaseline = true;

            RE::NiTransform primaryTransform{};
            if (tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform)) {
                _primaryGripLocal = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft), weaponNode);
                _primaryGripConfidence = 1.0f;
            } else {
                _primaryGripLocal = {};
                _primaryGripConfidence = 0.0f;
            }
        }
        _activeWeaponGenerationKey = currentWeaponGenerationKey;
        _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;

        clearPrimaryGripPose(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);
        clearSupportGripPose(primaryHandIsLeft);
        clearPrimaryDetachVisualAuthority(primaryHandIsLeft);
        restoreFrikOffhandGrip();
        if (_firingHandIsLeft) {
            // FRIK's primary weapon pose targets the game-primary RIGHT hand;
            // while the LEFT hand fires it must stay blocked and ROCK poses
            // the left hand itself.
            publishLeftFiringHandPose();
        } else {
            restoreFrikPrimaryWeaponPose();
        }
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryGripSeparationWorld = 0.0f;
        _hasSolvedWeaponTransform = false;
        if (!_firingHandIsLeft) {
            /*
             * Right firing hand: PrimaryOnly is FRIK-native carry, so ROCK
             * deliberately holds no hand-to-weapon frame. A LEFT firing hand
             * has no native carry - its captured frame IS the carry solve and
             * must survive this transition (wiping it here was the "weapon
             * snaps back to the right hand" takeover regression).
             */
            _primaryHandWeaponLocal = {};
            _hasFiringHandWeaponLocal = false;
        }
        _primaryHandVisualLerp = {};
        _state = TwoHandedState::PrimaryOnly;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: primary-only equipped weapon ownership active reason={} generation={:016X} ownership={:016X} provisional={}",
            reason ? reason : "unknown",
            _activeWeaponGenerationKey,
            _activeEquippedWeaponOwnershipKey,
            _activeWeaponGenerationKey == 0 ? "yes" : "no");
        return true;
    }

    void TwoHandedGrip::requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (_equippedWeaponDropRequest.requested) {
            transitionToInactive(false);
            return;
        }

        _equippedWeaponDropRequest = EquippedWeaponManualDropRequest{
            .requested = true,
            .sourceHand = sourceHand,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: equipped weapon drop requested reason={} sourceHand={} generation={:016X}",
            reason ? reason : "unknown",
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            _activeWeaponGenerationKey);
        transitionToInactive(false);
    }

    void TwoHandedGrip::updatePrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const EquippedWeaponPrimaryGripInput& primaryGripInput)
    {
        equipped_weapon_manual_ownership_policy::RuntimeState manualState{
            .active = true,
            .ownershipKey = _activeEquippedWeaponOwnershipKey,
        };
        const auto manualDecision = equipped_weapon_manual_ownership_policy::update(manualState,
            equipped_weapon_manual_ownership_policy::Input{
                .weaponEquipped = weaponNode != nullptr,
                .ownershipKey = currentEquippedWeaponOwnershipKey,
                .startRequested = false,
                .primaryGripRetained = primaryGripInput.held,
                .supportGripRetained = false,
            });

        if (manualDecision.dropRequested) {
            requestEquippedWeaponDrop("primary-only-grip-released",
                _firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right);
            return;
        }

        if (manualDecision.cleared) {
            transitionToInactive(false);
            return;
        }

        if (!_firingHandIsLeft) {
            // Right firing hand: FRIK-native carry, ROCK bookkeeping only.
            _hasSolvedWeaponTransform = false;
            return;
        }

        // Left firing hand: FRIK cannot carry (its weapon glue targets the
        // right hand and is blocked); ROCK drives the weapon rigidly from the
        // left hand through the captured weapon-relative firing-grip frame.
        if (solveLeftFiringWeaponCarry(weaponNode)) {
            publishLeftFiringHandPose();
        }
    }

    bool TwoHandedGrip::solveLeftFiringWeaponCarry(RE::NiNode* weaponNode)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the captured firing-grip frame is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform firingHandTransform{};
        if (!tryGetSolverHandTransform(true, firingHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }

        const RE::NiTransform solvedWeaponWorld =
            transform_math::composeTransforms(firingHandTransform, transform_math::invertTransform(_primaryHandWeaponLocal));
        if (!isFiniteTransform(solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the weapon solve produced an invalid transform");
            transitionToInactive(false);
            return false;
        }

        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return false;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;
        return true;
    }

    bool TwoHandedGrip::firingGripContactMatchesCapturedGrip(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& handWeaponContact,
        const RE::NiTransform& handTransform,
        const bool handIsLeft) const
    {
        if (!weaponNode || !handWeaponContact.valid) {
            return false;
        }

        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(handWeaponContact.weaponGenerationKey, _activeWeaponGenerationKey)) {
            return false;
        }

        const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, handIsLeft);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 delta = sub(palm, firingGripWorld);
        const float distance = std::sqrt(dot(delta, delta));
        return std::isfinite(distance) && distance <= g_rockConfig.rockWeaponFiringGripReattachRadius;
    }

    bool TwoHandedGrip::tryComputePalmToGripDistanceForHand(RE::NiNode* weaponNode, const bool handIsLeft, float& outDistance) const
    {
        if (!weaponNode) {
            return false;
        }
        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(handIsLeft, handTransform)) {
            return false;
        }
        const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, handIsLeft);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 delta = sub(palm, firingGripWorld);
        const float distance = std::sqrt(dot(delta, delta));
        if (!std::isfinite(distance)) {
            return false;
        }
        outDistance = distance;
        return true;
    }

    bool TwoHandedGrip::tryReattachFiringGrip(const bool handIsLeft, RE::NiNode* weaponNode, const WeaponInteractionContact& handWeaponContact)
    {
        if (!weaponNode) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(handIsLeft, handTransform)) {
            return false;
        }

        if (!firingGripContactMatchesCapturedGrip(weaponNode, handWeaponContact, handTransform, handIsLeft)) {
            return false;
        }

        // Validated: commit. A takeover by the other hand flips the firing
        // role here and reuses the SAME captured weapon-relative grip point.
        if (handIsLeft != _firingHandIsLeft) {
            setFiringHand(handIsLeft, "firing-grip-reattach-other-hand");
        }

        /*
         * The LEFT firing hand takes the canonical right-hand hold MIRRORED
         * (same authored offsets, adapted to the left bone basis) instead of
         * freezing the live squeeze orientation, which contorted the arm.
         * Fallback to the live capture only when no canonical exists for the
         * current weapon generation.
         */
        bool usedMirroredCanonicalHold = false;
        if (handIsLeft) {
            RE::NiTransform mirroredHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(mirroredHandWeaponLocal)) {
                _primaryHandWeaponLocal = mirroredHandWeaponLocal;
                usedMirroredCanonicalHold = true;
            }
        }
        if (!usedMirroredCanonicalHold) {
            const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, handIsLeft);
            const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
            const RE::NiTransform adjustedHandTransform =
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palm, firingGripWorld);
            _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        }
        _hasFiringHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _firingGripSequence = ++_gripCaptureSequence;
        _primaryHandVisualLerp = {};
        clearPrimaryDetachVisualAuthority(handIsLeft);
        if (_firingHandIsLeft) {
            publishLeftFiringHandPose();
        } else {
            restoreFrikPrimaryWeaponPose();
        }
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = handIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: firing hand reattached at configured grip hand={} hold={}",
            handIsLeft ? "left" : "right",
            usedMirroredCanonicalHold ? "mirrored-canonical" : "live-capture");
        return true;
    }

    void TwoHandedGrip::updatePartCarryGrip(
        RE::NiNode* weaponNode,
        float dt,
        const EquippedWeaponGripFrameInput& frameInput,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const WeaponCollision& weaponCollision,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
        const bool firingHandIsLeft = _firingHandIsLeft;
        const WeaponInteractionContact& firingHandContact = firingHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionContact& supportHandContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& firingRuntimeState = firingHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const bool supportGripHeld = supportHandIsLeft ? frameInput.leftGripHeld : frameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool firingHandHoldingObject = firingHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;

        /*
         * Firing-grip reattach is the squeeze gesture: a held grab with a
         * free palm inside the reattach radius re-takes the grip. Nothing
         * attaches to an open hand, and the gesture cannot re-capture a fresh
         * detach because the detach requires the grab to be open. A hand
         * already part-gripping is never converted; open it first, then
         * squeeze the grip. With ambidextrous takeover available, EITHER free
         * hand can squeeze the firing grip - whichever hand takes it becomes
         * the firing hand (the grip point itself stays weapon-relative). The
         * current firing hand is tested first so same-frame ties keep today's
         * behavior.
         */
        struct FiringGripReattachCandidate
        {
            bool isLeft;
            bool eligible;
            bool gripHeld;
            const WeaponInteractionContact* contact;
        };
        const FiringGripReattachCandidate reattachCandidates[2] = {
            { firingHandIsLeft,
                firingHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                frameInput.primaryGripInput.held,
                &firingHandContact },
            { supportHandIsLeft,
                supportHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                supportGripHeld,
                &supportHandContact },
        };
        for (const FiringGripReattachCandidate& candidate : reattachCandidates) {
            if (candidate.isLeft != firingHandIsLeft && !ambidextrousFiringGripTakeoverAvailable()) {
                continue;
            }
            if (!candidate.eligible || partGrip(candidate.isLeft).active) {
                continue;
            }
            float palmToGripDistance = 0.0f;
            if (!tryComputePalmToGripDistanceForHand(weaponNode, candidate.isLeft, palmToGripDistance)) {
                continue;
            }
            const bool reattachRequested = weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab(
                candidate.gripHeld,
                palmToGripDistance,
                g_rockConfig.rockWeaponFiringGripReattachRadius);
            if (!_firingGripReattachHoverInsideRadius &&
                weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate(
                    candidate.gripHeld,
                    palmToGripDistance,
                    g_rockConfig.rockWeaponFiringGripReattachRadius)) {
                _firingGripReattachHoverInsideRadius = true;
                _firingGripReattachHoverHandIsLeft = candidate.isLeft;
            }
            if (reattachRequested && tryReattachFiringGrip(candidate.isLeft, weaponNode, *candidate.contact)) {
                const bool newSupportHandIsLeft = !_firingHandIsLeft;
                if (partGrip(newSupportHandIsLeft).active) {
                    // Re-lock the two-hand separation against the (possibly
                    // swapped) support grip and re-blend the support target in.
                    const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(partGrip(newSupportHandIsLeft), weaponNode);
                    const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
                    const RE::NiPoint3 separationDelta = sub(supportGripWorld, firingGripWorld);
                    const float separation = std::sqrt(dot(separationDelta, separationDelta));
                    if (std::isfinite(separation)) {
                        _lockedGripSeparationWorld = separation;
                    }
                    if (candidate.isLeft != firingHandIsLeft) {
                        _rotationBlend = 0.0f;
                    }
                    _state = TwoHandedState::Gripping;
                    updateFullWeaponAuthorityGrip(weaponNode, dt);
                } else {
                    transitionToPrimaryOnly(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "part-carry-reattached-firing-grip");
                }
                return;
            }
        }

        bool lastReleaseWasSupportHand = true;

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        if (supportGrip.active) {
            if (!providerPartAuthorityStillCurrent(supportGrip, currentWeaponGenerationKey) || !supportRuntimeState.supportGripAllowed) {
                /*
                 * Provider revocation and offhand reservation are policy
                 * changes, not a player release: return the weapon to
                 * FRIK-native carry instead of dropping it.
                 */
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because provider revoked the support part grip");
                transitionToInactive(false);
                return;
            }
            if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                releasePartGrip(supportHandIsLeft, "support-grip-released");
                lastReleaseWasSupportHand = true;
            }
        }

        WeaponPartGrip& freeHandGrip = partGrip(firingHandIsLeft);
        if (freeHandGrip.active) {
            if (!providerPartAuthorityStillCurrent(freeHandGrip, currentWeaponGenerationKey)) {
                releasePartGrip(firingHandIsLeft, "provider-part-authority-lost");
                lastReleaseWasSupportHand = false;
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(frameInput.primaryGripInput.held, firingHandHoldingObject)) {
                releasePartGrip(firingHandIsLeft, "free-hand-grip-released");
                lastReleaseWasSupportHand = false;
            }
        }

        /*
         * Mid-hold conversion, part-carry flavor: a part grip captured
         * WITHOUT provider authority whose own part now resolves to a
         * matched provider target (consumer armed its whitelist while the
         * hand was already holding — per-hand trigger arming) recaptures
         * immediately under the new resolution. Gated on the OTHER grip
         * holding carry authority: converting the last carry grip to
         * attach-only glue would drop the weapon through the fail-closed
         * all-grips check below. The free hand recaptures in the same
         * update rather than release-to-recapture because its capture path
         * is press-edged; the support hand gets the same treatment for
         * symmetry (no one-frame glue gap).
         */
        if (freeHandGrip.active && !freeHandGrip.providerPartAuthority.active &&
            firingRuntimeState.providerPartAuthority.active &&
            firingRuntimeState.providerPartAuthority.bodyId == freeHandGrip.contactBodyId &&
            weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly)) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (freeHandDecision.kind == WeaponInteractionKind::SupportGrip) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: recapturing free-hand part grip under newly matched provider weapon-part target");
                releasePartGrip(firingHandIsLeft, "provider-part-target-newly-matched");
                (void)capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority);
            }
        }
        if (supportGrip.active && !supportGrip.providerPartAuthority.active &&
            supportRuntimeState.providerPartAuthority.active &&
            supportRuntimeState.providerPartAuthority.bodyId == supportGrip.contactBodyId &&
            weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
            const WeaponInteractionDecision supportDecision = routeWeaponInteraction(supportHandContact, supportRuntimeState);
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: recapturing support part grip under newly matched provider weapon-part target");
                releasePartGrip(supportHandIsLeft, "provider-part-target-newly-matched");
                (void)capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority);
            }
        }

        if (!freeHandGrip.active && frameInput.primaryGripInput.pressed) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (weapon_two_handed_grip_math::canStartFreeHandPartGrip(
                    freeHandDecision.kind == WeaponInteractionKind::SupportGrip,
                    frameInput.primaryGripInput.pressed,
                    firingHandHoldingObject,
                    freeHandGrip.active)) {
                if (capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority)) {
                    /*
                     * AttachOnly keeps its authored source frames so the glued
                     * hand follows provider-driven part motion; it never joins
                     * the carry solve, so it cannot feed part animation back
                     * into the carry (the drift lockPartGripToWeaponRoot
                     * prevents). Separation/blend only matter for a two-anchor
                     * carry, which needs both grips to hold carry authority.
                     */
                    if (freeHandGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: free-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(firingHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly)) {
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        if (!supportGrip.active) {
            const WeaponInteractionDecision supportDecision = routeWeaponInteraction(supportHandContact, supportRuntimeState);
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip &&
                weapon_two_handed_grip_math::canStartSupportGrip(true, supportGripHeld, supportHandHoldingObject)) {
                if (capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority)) {
                    // Symmetric to the free-hand capture above: attach-only
                    // glue keeps source frames and stays out of the carry.
                    if (supportGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: support-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(supportHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        /*
         * Only carry-authority grips can hold the weapon. When the last carry
         * grip releases, a remaining AttachOnly glue cannot inherit pivot
         * authority (never upgrade), so it releases with the carry and the
         * normal manual-drop request proceeds (fail closed).
         */
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly) &&
            !weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
            releasePartGrip(supportHandIsLeft, "carry-authority-lost");
            releasePartGrip(firingHandIsLeft, "carry-authority-lost");
            requestEquippedWeaponDrop(
                "part-carry-all-grips-released",
                lastReleaseWasSupportHand ?
                    (supportHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right) :
                    (firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right));
            return;
        }

        // The pivot must always be a carry-authority grip; after the check
        // above, the other hand is guaranteed to hold one.
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(
                partGrip(_partCarryPivotIsLeft).active,
                partGrip(_partCarryPivotIsLeft).attachOnly)) {
            _partCarryPivotIsLeft = !_partCarryPivotIsLeft;
        }

        (void)solvePartCarryWeaponAuthority(weaponNode, dt);
    }

    float TwoHandedGrip::partCarryGripSeparation(RE::NiNode* weaponNode) const
    {
        const RE::NiPoint3 leftGripWorld = resolvePartGripWorld(partGrip(true), weaponNode);
        const RE::NiPoint3 rightGripWorld = resolvePartGripWorld(partGrip(false), weaponNode);
        const RE::NiPoint3 delta = sub(leftGripWorld, rightGripWorld);
        const float separation = std::sqrt(dot(delta, delta));
        return std::isfinite(separation) ? separation : 0.0f;
    }

    bool TwoHandedGrip::solvePartCarryWeaponAuthority(RE::NiNode* weaponNode, float dt)
    {
        const bool pivotIsLeft = _partCarryPivotIsLeft;
        const WeaponPartGrip& pivotGrip = partGrip(pivotIsLeft);
        const WeaponPartGrip& aimGrip = partGrip(!pivotIsLeft);
        // An AttachOnly glue never aims the weapon; the carry solves
        // single-anchor around the pivot and the glue publishes afterwards.
        const bool aimGripCarries = weapon_part_grip_report_policy::partGripCountsAsCarry(aimGrip.active, aimGrip.attachOnly);
        if (!pivotGrip.active || !pivotGrip.hasHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because captured hand frames are unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform pivotHandTransform{};
        if (!tryGetSolverHandTransform(pivotIsLeft, pivotHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }

        if (aimGripCarries) {
            RE::NiTransform aimHandTransform{};
            if (!tryGetSolverHandTransform(!pivotIsLeft, aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because aim hand transform is unavailable");
                transitionToInactive(false);
                return false;
            }

            _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

            /*
             * The two-anchor solve must be closed over the captured
             * weapon-root-local grip points. Part-carry feeds its own solved
             * transform back as the next frame's base, so re-resolving grips
             * through live part-node chains lets any per-frame part animation
             * integrate into a steady carry drift (verified by telemetry:
             * rigid-weapon grip separation grew frame over frame). The
             * trade-off is that a two-anchor carry does not follow externally
             * driven part motion; provider part revocation releases the grip
             * in that case.
             */
            const RE::NiPoint3 pivotPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(pivotHandTransform, pivotIsLeft);
            const RE::NiPoint3 aimPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(aimHandTransform, !pivotIsLeft);
            const RE::NiPoint3 currentAimGripWorld = weaponLocalToWorld(aimGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentPivotGripWorld = weaponLocalToWorld(pivotGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentSeparationDelta = sub(currentAimGripWorld, currentPivotGripWorld);
            const float currentSeparation = std::sqrt(dot(currentSeparationDelta, currentSeparationDelta));
            const float lockedSeparation = _partCarryGripSeparationWorld > 0.0f ? _partCarryGripSeparationWorld : currentSeparation;
            const RE::NiPoint3 lockedAimTarget = makeLockedSupportGripTarget(
                pivotPalm,
                aimPalm,
                currentAimGripWorld,
                lockedSeparation,
                0.001f);
            const RE::NiPoint3 blendedAimTarget = lerpPoint(currentAimGripWorld, lockedAimTarget, _rotationBlend);

            WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
            solverInput.weaponWorldTransform = weaponNode->world;
            solverInput.primaryGripLocal = pivotGrip.gripLocal;
            solverInput.supportGripLocal = aimGrip.gripLocal;
            solverInput.primaryTargetWorld = pivotPalm;
            solverInput.supportTargetWorld = blendedAimTarget;
            solverInput.supportNormalLocal = aimGrip.normalLocal;
            solverInput.supportNormalTargetWorld = computePalmNormalFromHandBasis(aimHandTransform, !pivotIsLeft);
            solverInput.useSupportNormalTwist = true;
            solverInput.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;

            const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
            if (!solved.solved) {
                return true;
            }

            // Break the rotation feedback loop's orthonormality decay before
            // the solved transform becomes next frame's base.
            RE::NiTransform stabilizedWeaponWorld = solved.weaponWorldTransform;
            stabilizedWeaponWorld.rotate = orthonormalizeStoredRotation(stabilizedWeaponWorld.rotate);

            if (!applyWeaponVisualAuthority(weaponNode, stabilizedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            publishGripHandPoses(!pivotIsLeft);

            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform) ||
                !applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, &aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        } else {
            RE::NiTransform solvedWeaponWorld{};
            if (pivotGrip.hasSourceFrames && pivotGrip.hasAttachmentWeaponLocal && resolveCurrentSupportAttachmentRoot(pivotGrip, weaponNode)) {
                const RE::NiTransform solvedSourceWorld =
                    transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handSourceLocal));
                solvedWeaponWorld = transform_math::composeTransforms(solvedSourceWorld, transform_math::invertTransform(pivotGrip.attachmentWeaponLocal));
            } else {
                solvedWeaponWorld = transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handWeaponLocal));
            }
            if (!isFiniteTransform(solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because single-anchor weapon solve produced invalid transform");
                transitionToInactive(false);
                return false;
            }

            if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        }

        /*
         * AttachOnly glue publishes after the weapon solve so it composes from
         * this frame's part transforms (including provider part drives applied
         * earlier in the frame). A glue visual failure only loses the attach;
         * the carry pivot must survive it.
         */
        if (aimGrip.active && aimGrip.attachOnly) {
            RE::NiTransform attachHandTransform{};
            const RE::NiTransform* liveAttachHandWorld =
                tryGetSolverHandTransform(!pivotIsLeft, attachHandTransform) ? &attachHandTransform : nullptr;
            publishGripHandPoses(!pivotIsLeft);
            if (!applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, liveAttachHandWorld)) {
                releasePartGrip(!pivotIsLeft, "attach-only-visual-authority-failed");
            }
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 pivotGripFinal = resolvePartGripWorld(pivotGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: part-carry authority pivot={} anchors={} pivotGrip=({:.1f},{:.1f},{:.1f}) handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                pivotIsLeft ? "left" : "right",
                aimGrip.active ? 2 : 1,
                pivotGripFinal.x,
                pivotGripFinal.y,
                pivotGripFinal.z,
                pivotGrip.visualLerp.lastAlpha,
                pivotGrip.visualLerp.durationSeconds,
                aimGrip.visualLerp.lastAlpha,
                aimGrip.visualLerp.durationSeconds);
        }
        return true;
    }

    void TwoHandedGrip::updateVisualOnlySupportGrip(RE::NiNode* weaponNode, float dt)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;

        if (_firingHandIsLeft) {
            // Visual-only support never steers aim, but with a LEFT firing
            // hand the weapon itself must still be ROCK-carried (FRIK's glue
            // is blocked); the shooting-cup right hand stays visual-only.
            if (!solveLeftFiringWeaponCarry(weaponNode)) {
                return;
            }
            publishLeftFiringHandPose();
        }

        static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
        publishGripHandPoses(supportHandIsLeft);

        RE::NiTransform supportTransform{};
        const RE::NiTransform* liveSupportTransform = tryGetSolverHandTransform(supportHandIsLeft, supportTransform) ? &supportTransform : nullptr;
        if (!applyLockedHandVisualAuthority(weaponNode, false, true, dt, nullptr, liveSupportTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing visual-only support grip because ROCK support hand authority failed");
            transitionToInactive(false);
            return;
        }

        _lastSolvedWeaponTransform = weaponNode ? weaponNode->world : RE::NiTransform{};
        _hasSolvedWeaponTransform = _firingHandIsLeft && _hasSolvedWeaponTransform;

        if (weaponNode && ++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
            const RE::NiPoint3 offhandGripFinal = resolvePartGripWorld(supportGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: visual-only support follows weapon='{}', offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp={:.2f}/{:.3f}s",
                weaponNode->name.c_str(),
                offhandGripFinal.x,
                offhandGripFinal.y,
                offhandGripFinal.z,
                supportGrip.visualLerp.lastAlpha,
                supportGrip.visualLerp.durationSeconds);
        }
    }

    void TwoHandedGrip::setSupportGripPose(bool isLeft, WeaponGripPoseId poseId, const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (meshFingerPose && meshFingerPose->solved) {
            grip.fingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            grip.fingerSplayRadians = {};
            grip.hasFingerSplay = grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, *meshFingerPose, grip.fingerSplayRadians);
            grip.hasFingerPose = true;
            return;
        }

        const auto& poseValues = poseValuesForGrip(poseId);
        grip.fingerPose = poseValues;
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = true;
        grip.hasFingerSplay = false;
    }

    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.fingerPose = {};
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = false;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;

        (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
        if (_scopeMenuOpenThisFrame) {
            _scopeHandAuthorityCleanupPending = true;
        } else {
            (void)frik_visual_authority::clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, handFromBool(isLeft));
        }
    }

    bool TwoHandedGrip::applyWeaponVisualAuthority(RE::NiNode* weaponNode, const RE::NiTransform& solvedWeaponWorld)
    {
        if (!weaponNode) {
            return false;
        }

        // hFRIK has already calibrated the native activation camera for the
        // current one-hand weapon frame. Capture that relationship before ROCK
        // publishes its final physical grip frame, then move both together.
        const NativeScopeCameraFollowCapture scopeCameraFollow = captureNativeScopeCameraFollow(weaponNode);

        if (weaponNode->parent) {
            weaponNode->local = weapon_visual_authority_math::worldTargetToParentLocal(weaponNode->parent->world, solvedWeaponWorld);
            f4vr::updateTransformsDown(weaponNode, true);
        } else {
            weaponNode->local = solvedWeaponWorld;
            weaponNode->world = solvedWeaponWorld;
            f4vr::updateTransformsDown(weaponNode, false);
        }

        applyNativeScopeCameraFollow(scopeCameraFollow, weaponNode->world);
        return true;
    }

    bool TwoHandedGrip::applyFiringHandLockedVisual(RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            return false;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform firingHandWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _primaryHandWeaponLocal);
        const RE::NiTransform appliedFiringHandWorld =
            resolveLockedHandVisualTarget(firingHandWorld, liveHandWorld, dt, _primaryHandVisualLerp);
        return frik_visual_authority::applyExternalHandWorldTransform(
            PRIMARY_GRIP_TAG, handFromBool(_firingHandIsLeft), appliedFiringHandWorld, GRIP_HAND_POSE_PRIORITY);
    }

    bool TwoHandedGrip::applyPartGripLockedVisual(bool isLeft, RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!weaponNode || !grip.active || !grip.hasHandWeaponLocal) {
            return false;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform partGripHandWorld = resolvePartGripHandWorld(grip, weaponNode);
        const RE::NiTransform appliedHandWorld =
            resolveLockedHandVisualTarget(partGripHandWorld, liveHandWorld, dt, grip.visualLerp);
        return frik_visual_authority::applyExternalHandWorldTransform(
            SUPPORT_GRIP_TAG, handFromBool(isLeft), appliedHandWorld, GRIP_HAND_POSE_PRIORITY);
    }

    bool TwoHandedGrip::applyLockedHandVisualAuthority(
        RE::NiNode* weaponNode,
        bool applyPrimaryHand,
        bool applySupportHand,
        float dt,
        const RE::NiTransform* livePrimaryHandWorld,
        const RE::NiTransform* liveSupportHandWorld)
    {
        if (!weaponNode) {
            return false;
        }

        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scopeMenuOpenThisFrame)) {
            return true;
        }

        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        if (!applyPrimaryHand && !applySupportHand) {
            return true;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        bool primaryApplied = true;
        bool supportApplied = true;
        if (applyPrimaryHand) {
            primaryApplied = applyFiringHandLockedVisual(weaponNode, dt, livePrimaryHandWorld);
        }
        if (applySupportHand) {
            supportApplied = applyPartGripLockedVisual(supportHandIsLeft, weaponNode, dt, liveSupportHandWorld);
        }
        if (primaryApplied && supportApplied) {
            return true;
        }

        if (applyPrimaryHand && primaryApplied) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, handFromBool(_firingHandIsLeft));
        }
        if (applySupportHand && supportApplied) {
            (void)frik_visual_authority::clearExternalHandWorldTransform(SUPPORT_GRIP_TAG, handFromBool(supportHandIsLeft));
        }
        return false;
    }

    void TwoHandedGrip::publishGripHandPoses(bool isLeft)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }

        const WeaponPartGrip& grip = partGrip(isLeft);
        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && grip.hasFingerPose) {
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            (void)frik_visual_authority::setHandPoseCustomWithPriority(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft),
                handPose,
                GRIP_HAND_POSE_PRIORITY);
        }

        if (grip.hasFingerLocalTransforms) {
            frik_visual_authority::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = grip.fingerLocalTransformMask;
            for (std::size_t i = 0; i < grip.fingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = grip.fingerLocalTransforms[i];
            }
            (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(SUPPORT_GRIP_TAG, handFromBool(isLeft), &overrideData, GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripPose(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        if (_scopeMenuOpenThisFrame) {
            _scopeHandAuthorityCleanupPending = true;
        } else {
            (void)frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        }
    }

    void TwoHandedGrip::clearPrimaryDetachVisualAuthority(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        if (_scopeMenuOpenThisFrame) {
            _scopeHandAuthorityCleanupPending = true;
        } else {
            (void)frik_visual_authority::clearExternalHandWorldTransform(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        }
    }

    void TwoHandedGrip::killFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip suppressed");
        }
    }

    void TwoHandedGrip::restoreFrikOffhandGrip()
    {
        if (frik_visual_authority::blockOffHandWeaponGripping("ROCK_TwoHanded", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK offhand grip restored");
        }
    }

    bool TwoHandedGrip::blockFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose suppressed");
            return true;
        }
        return false;
    }

    void TwoHandedGrip::restoreFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose restored");
        }
    }

    void TwoHandedGrip::rememberRightFiringHandCanonicalFrame()
    {
        if (_firingHandIsLeft || !_hasFiringHandWeaponLocal || _activeWeaponGenerationKey == 0) {
            return;
        }
        _rightFiringHandCanonicalWeaponLocal = _primaryHandWeaponLocal;
        _rightFiringHandCanonicalGenerationKey = _activeWeaponGenerationKey;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    bool TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal(RE::NiTransform& outHandWeaponLocal) const
    {
        if (!_hasRightFiringHandCanonicalWeaponLocal ||
            _rightFiringHandCanonicalGenerationKey == 0 ||
            _rightFiringHandCanonicalGenerationKey != _activeWeaponGenerationKey) {
            return false;
        }

        /*
         * Semantic mirror built from ROCK's own handed palm math, NOT from
         * bone-basis algebra: the skeleton's left/right hand bone conventions
         * are not mirror images (a row-negation recipe derived from hFRIK's
         * left-handed-mode constants pointed the barrel backward in-game).
         * The canonical right hold is decomposed into weapon-space palm
         * semantics (palm pivot, palm normal, wrist-to-palm direction),
         * mirrored across the weapon's side symmetry plane (weapon +Y =
         * barrel, +X = side; FRIK's own gripping code relies on Y-forward),
         * then re-solved into a LEFT hand rotation through the left hand's
         * own palm-basis mapping. Every handed constant comes from the same
         * palm helpers the grip captures already trust in production.
         */
        const RE::NiTransform& rightHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;

        RE::NiTransform identityTransform{};
        identityTransform.MakeIdentity();

        // Hand-local palm semantics (helpers applied to identity => local).
        const RE::NiPoint3 leftPalmNormalLocal = computePalmNormalFromHandBasis(identityTransform, true);
        const RE::NiPoint3 leftPalmPivotLocal = computeGrabLegacyPalmPivotAWorldFromHandBasis(identityTransform, true);

        // Canonical right hold as weapon-space palm semantics.
        const RE::NiPoint3 rightPalmNormalWeapon = computePalmNormalFromHandBasis(rightHandWeaponLocal, false);
        const RE::NiPoint3 rightPalmPivotWeapon = computeGrabLegacyPalmPivotAWorldFromHandBasis(rightHandWeaponLocal, false);
        const RE::NiPoint3 rightWristToPalmWeapon = sub(rightPalmPivotWeapon, rightHandWeaponLocal.translate);

        const auto mirrorAcrossWeaponSidePlane = [](const RE::NiPoint3& v) {
            return RE::NiPoint3{ -v.x, v.y, v.z };
        };
        const RE::NiPoint3 mirroredPalmNormal = mirrorAcrossWeaponSidePlane(rightPalmNormalWeapon);
        const RE::NiPoint3 mirroredWristToPalm = mirrorAcrossWeaponSidePlane(rightWristToPalmWeapon);
        const RE::NiPoint3 mirroredPalmPivot = mirrorAcrossWeaponSidePlane(rightPalmPivotWeapon);

        constexpr float kMinAxisSeparation = 0.05f;
        const auto orthonormalPair = [](const RE::NiPoint3& primary, const RE::NiPoint3& secondary, RE::NiPoint3& outPrimary, RE::NiPoint3& outSecondary) {
            outPrimary = weaponSolverNormalize(primary);
            const RE::NiPoint3 secondaryUnit = weaponSolverNormalize(secondary);
            const float alignment = dot(secondaryUnit, outPrimary);
            const RE::NiPoint3 rejected = sub(secondaryUnit, RE::NiPoint3{ outPrimary.x * alignment, outPrimary.y * alignment, outPrimary.z * alignment });
            const float rejectedLength = std::sqrt(dot(rejected, rejected));
            if (!std::isfinite(rejectedLength) || rejectedLength < kMinAxisSeparation) {
                return false;
            }
            outSecondary = weaponSolverNormalize(rejected);
            return true;
        };

        RE::NiPoint3 nLocal{};
        RE::NiPoint3 pLocal{};
        RE::NiPoint3 nTarget{};
        RE::NiPoint3 pTarget{};
        if (!orthonormalPair(leftPalmNormalLocal, leftPalmPivotLocal, nLocal, pLocal) ||
            !orthonormalPair(mirroredPalmNormal, mirroredWristToPalm, nTarget, pTarget)) {
            return false;
        }
        const RE::NiPoint3 tLocal = weaponSolverCross(nLocal, pLocal);
        const RE::NiPoint3 tTarget = weaponSolverCross(nTarget, pTarget);

        /*
         * Rows-as-axes: the hand rotation's rows are the hand basis images in
         * weapon space, so the rotation mapping each left-local semantic axis
         * onto its mirrored weapon-space image is A^T * B (A rows = local
         * semantic frame, B rows = target semantic frame; both right-handed
         * orthonormal triples, so the product is a proper rotation).
         */
        const RE::NiMatrix3 localFrame = makeMatrixFromRows(nLocal, pLocal, tLocal);
        const RE::NiMatrix3 targetFrame = makeMatrixFromRows(nTarget, pTarget, tTarget);

        RE::NiTransform mirroredHandWeaponLocal{};
        mirroredHandWeaponLocal.rotate = mulRowMatrices(transposeMatrix(localFrame), targetFrame);
        // The left palm pivot lands on the mirrored right palm pivot.
        mirroredHandWeaponLocal.translate = sub(mirroredPalmPivot, rotateLocalVectorThroughRows(mirroredHandWeaponLocal.rotate, leftPalmPivotLocal));
        mirroredHandWeaponLocal.scale = rightHandWeaponLocal.scale;

        if (!isFiniteTransform(mirroredHandWeaponLocal)) {
            return false;
        }
        outHandWeaponLocal = mirroredHandWeaponLocal;
        return true;
    }

    void TwoHandedGrip::setFiringHand(const bool isLeft, const char* reason)
    {
        if (_firingHandIsLeft == isLeft) {
            return;
        }

        // Drop the old hand's role-tagged FRIK publications; the new hand's
        // grip-frame capture and pose publication are owned by the caller.
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        _primaryHandVisualLerp = {};
        _primaryReleaseDebounce = {};
        _firingHandIsLeft = isLeft;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand switched to {} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }

    bool TwoHandedGrip::tryPromoteSupportGripToFiringGrip(RE::NiNode* weaponNode)
    {
        if (!weaponNode || !ambidextrousFiringGripTakeoverAvailable()) {
            return false;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        // AttachOnly glue never inherits the firing grip; the distance gate
        // below keeps promotion to hands physically wrapped over the grip.
        if (!supportGrip.active || supportGrip.attachOnly) {
            return false;
        }

        /*
         * Promotion distance uses the support GRIP POINT (where the hand
         * actually grabbed the weapon), not the palm pivot: a shooting-cup
         * palm sits a hand-width away from the grip center and the tight
         * reattach radius silently declined every takeover. The dedicated
         * promotion radius keeps handguard/foregrip support grips out.
         */
        const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(supportGrip, weaponNode);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 gripDelta = sub(supportGripWorld, firingGripWorld);
        const float supportGripToFiringGripDistance = std::sqrt(dot(gripDelta, gripDelta));
        if (!std::isfinite(supportGripToFiringGripDistance) ||
            supportGripToFiringGripDistance > g_rockConfig.rockFiringGripPromotionRadius) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, handTransform)) {
            return false;
        }

        // A left firing hand needs FRIK's right-hand weapon pose blocked for
        // the whole left-firing tenure; abort the promotion if that fails.
        if (supportHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            return false;
        }

        /*
         * Commit: the support hand takes over the SAME weapon-relative firing
         * grip in place. A LEFT takeover applies the canonical right-hand
         * hold mirrored (authored offsets adapted to the left bone basis);
         * the live squeeze orientation is only the no-canonical fallback.
         */
        RE::NiTransform newFiringHandWeaponLocal{};
        bool usedMirroredCanonicalHold = false;
        if (supportHandIsLeft && tryComputeMirroredLeftFiringHandWeaponLocal(newFiringHandWeaponLocal)) {
            usedMirroredCanonicalHold = true;
        } else {
            const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, supportHandIsLeft);
            const RE::NiTransform adjustedHandTransform =
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palm, firingGripWorld);
            newFiringHandWeaponLocal =
                transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        }

        setFiringHand(supportHandIsLeft, "support-grip-promotion");
        if (!transitionToPrimaryOnly(weaponNode, _activeWeaponGenerationKey, _activeEquippedWeaponOwnershipKey, "firing-grip-hand-promotion")) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: firing-grip promotion failed to enter primary-only; clearing authority");
            transitionToInactive(false);
            return true;
        }

        _primaryHandWeaponLocal = newFiringHandWeaponLocal;
        _hasFiringHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _firingGripSequence = ++_gripCaptureSequence;
        _primaryHandVisualLerp = {};
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: support hand promoted to firing grip hand={} gripToGrip={:.2f} hold={}",
            _firingHandIsLeft ? "left" : "right",
            supportGripToFiringGripDistance,
            usedMirroredCanonicalHold ? "mirrored-canonical" : "live-capture");
        return true;
    }

    RE::NiNode* TwoHandedGrip::resolveFirstPersonHandNode(const bool isLeft)
    {
        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        if (!firstPersonSkeleton) {
            return nullptr;
        }
        return f4vr::findNode(firstPersonSkeleton, isLeft ? "LArm_Hand" : "RArm_Hand");
    }

    void TwoHandedGrip::publishLeftFiringHandPose()
    {
        /*
         * Trigger-articulated firing pose for the LEFT hand: the game's own
         * fire animation only ever plays on the game-primary right hand, so
         * ROCK poses the left hand itself and drives the index finger from
         * the physical trigger pull. Published every left-firing frame;
         * quantizing the trigger keeps the bridge's pose cache effective
         * between changes. Curl scale: 0 = bent, 1 = straight.
         */
        constexpr float kIndexRelaxed = 0.60f;
        constexpr float kIndexPulled = 0.16f;
        const float quantizedTrigger = std::round(std::clamp(_leftFiringTriggerAxis, 0.0f, 1.0f) * 20.0f) / 20.0f;
        const float index = kIndexRelaxed + (kIndexPulled - kIndexRelaxed) * quantizedTrigger;

        const std::array<float, 15> firingCurls = {
            0.42f, 0.38f, 0.34f, // thumb wrapped over the grip back
            index, index, index * 0.9f, // index rides the trigger
            0.30f, 0.26f, 0.22f, // middle wrapped on the grip
            0.30f, 0.26f, 0.22f, // ring
            0.32f, 0.28f, 0.24f, // pinky
        };
        if (!frik_visual_authority::setHandPoseCustomWithPriority(
                PRIMARY_GRIP_TAG,
                frik_visual_authority::Hand::Left,
                frik_visual_authority::makeHandPoseDataFromJointValues(firingCurls),
                GRIP_HAND_POSE_PRIORITY)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left firing-hand pose publish failed");
        }
    }

    void TwoHandedGrip::syncFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        const bool wantLeftFiringCarry = _firingHandIsLeft &&
            (_state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryOnly);

        if (!wantLeftFiringCarry) {
            releaseFiringHandWeaponNodeOwnership(weaponNode);
            return;
        }

        if (!_weaponNodeOwnershipBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, true)) {
                // Fail closed: without the FRIK block the weapon node would
                // fight two per-frame owners.
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left-firing carry aborted because the FRIK weapon-node ownership block is unavailable");
                transitionToInactive(false);
                return;
            }
            _weaponNodeOwnershipBlockEngaged = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership blocked for left-firing carry");
        }

        if (!weaponNode) {
            return;
        }

        RE::NiNode* leftHand = resolveFirstPersonHandNode(true);
        if (!leftHand) {
            return;
        }
        if (weaponNode->parent == leftHand) {
            _weaponNodeReparentedToLeftHand = true;
            return;
        }

        /*
         * Re-parent under LArm_Hand preserving world so the scene graph keeps
         * the weapon riding the firing hand at every point in the frame
         * (native fire/aim sampling included). Same operation FRIK performs
         * for the game's own left-handed mode, minus the mirrored offsets.
         */
        const RE::NiTransform worldBefore = weaponNode->world;
        RE::NiPointer<RE::NiAVObject> detached;
        if (weaponNode->parent) {
            weaponNode->parent->DetachChild(weaponNode, detached);
        }
        leftHand->AttachChild(weaponNode, true);
        weaponNode->local = weapon_visual_authority_math::worldTargetToParentLocal(leftHand->world, worldBefore);
        f4vr::updateTransformsDown(weaponNode, true);
        _weaponNodeReparentedToLeftHand = true;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented under LArm_Hand for left-firing carry");
    }

    void TwoHandedGrip::releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        if (_weaponNodeReparentedToLeftHand) {
            RE::NiNode* node = weaponNode ? weaponNode : _activeWeaponNode;
            RE::NiNode* rightHand = resolveFirstPersonHandNode(false);
            if (node && rightHand && node->parent != rightHand) {
                const RE::NiTransform worldBefore = node->world;
                RE::NiPointer<RE::NiAVObject> detached;
                if (node->parent) {
                    node->parent->DetachChild(node, detached);
                }
                rightHand->AttachChild(node, true);
                node->local = weapon_visual_authority_math::worldTargetToParentLocal(rightHand->world, worldBefore);
                f4vr::updateTransformsDown(node, true);
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented back under RArm_Hand");
            }
            _weaponNodeReparentedToLeftHand = false;
        }

        if (_weaponNodeOwnershipBlockEngaged) {
            // FRIK also force-reattaches native weapon-node parenting once the
            // block releases (belt and braces for teardown without nodes).
            (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, false);
            _weaponNodeOwnershipBlockEngaged = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership restored");
        }
    }

}

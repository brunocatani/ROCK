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
        constexpr const char* RETURN_HAND_TAG = "ROCK_WeaponReturn";
        constexpr const char* WEAPON_NODE_OWNERSHIP_TAG = "ROCK_LeftFiringCarry";
        constexpr int GRIP_HAND_POSE_PRIORITY = 100;
        constexpr int RETURN_HAND_VISUAL_PRIORITY = 85;
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

        bool areTransformsNearlyEqual(const RE::NiTransform& lhs, const RE::NiTransform& rhs, const float epsilon = 0.001f)
        {
            if (std::abs(lhs.translate.x - rhs.translate.x) > epsilon ||
                std::abs(lhs.translate.y - rhs.translate.y) > epsilon ||
                std::abs(lhs.translate.z - rhs.translate.z) > epsilon ||
                std::abs(lhs.scale - rhs.scale) > epsilon) {
                return false;
            }
            for (int row = 0; row < 3; ++row) {
                for (int column = 0; column < 3; ++column) {
                    if (std::abs(lhs.rotate.entry[row][column] - rhs.rotate.entry[row][column]) > epsilon) {
                        return false;
                    }
                }
            }
            return true;
        }

        bool tryGetComposedNodeWorld(const RE::NiAVObject* node, RE::NiTransform& outWorld)
        {
            if (!node) {
                return false;
            }
            outWorld = node->parent ?
                transform_math::composeTransforms(node->parent->world, node->local) :
                node->world;
            return isFiniteTransform(outWorld);
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

        struct NativeScopeCameraFollowResult
        {
            RE::NiTransform targetCameraWorld{};
            RE::NiTransform immediateCameraWorldAfter{};
            bool targetValid{ false };
            bool writeApplied{ false };
            bool immediateReadbackValid{ false };
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

        NativeScopeCameraFollowResult applyNativeScopeCameraWorldTarget(const NativeScopeCameraFollowCapture& capture, const RE::NiTransform& targetCameraWorld)
        {
            NativeScopeCameraFollowResult result{};
            if (!capture.valid || !capture.camera || !isFiniteTransform(targetCameraWorld)) {
                return result;
            }
            result.targetCameraWorld = targetCameraWorld;
            result.targetValid = true;

            auto* scopeCamera = capture.camera;
            if (scopeCamera->parent) {
                const RE::NiTransform targetCameraLocal = weapon_visual_authority_math::worldTargetToParentLocal(
                    scopeCamera->parent->world,
                    targetCameraWorld);
                if (!isFiniteTransform(targetCameraLocal)) {
                    return result;
                }
                scopeCamera->local = targetCameraLocal;
                f4vr::updateTransforms(scopeCamera);
                result.writeApplied = true;
                const RE::NiTransform immediateCameraWorld = scopeCamera->world;
                if (isFiniteTransform(immediateCameraWorld)) {
                    result.immediateCameraWorldAfter = immediateCameraWorld;
                    result.immediateReadbackValid = true;
                }
                return result;
            }

            scopeCamera->local = targetCameraWorld;
            scopeCamera->world = targetCameraWorld;
            result.writeApplied = true;
            result.immediateCameraWorldAfter = scopeCamera->world;
            result.immediateReadbackValid = isFiniteTransform(result.immediateCameraWorldAfter);
            return result;
        }

        NativeScopeCameraFollowResult applyNativeScopeCameraFollow(const NativeScopeCameraFollowCapture& capture, const RE::NiTransform& weaponWorldAfter,
            const RE::NiPoint3* sightAnchorWeaponLocal)
        {
            if (!capture.valid || !isFiniteTransform(weaponWorldAfter)) {
                return {};
            }
            const RE::NiTransform targetCameraWorld = sightAnchorWeaponLocal
                ? native_scope_camera_follow_math::followWeaponWorldChangeFromSightAnchor(capture.weaponWorldBefore, weaponWorldAfter, capture.cameraWorldBefore,
                      *sightAnchorWeaponLocal)
                : native_scope_camera_follow_math::followWeaponWorldChange(capture.weaponWorldBefore, weaponWorldAfter, capture.cameraWorldBefore);
            return applyNativeScopeCameraWorldTarget(capture, targetCameraWorld);
        }

        NativeScopeCameraDebugSnapshot makeNativeScopeCameraDebugSnapshot(const NativeScopeCameraDebugSnapshot& previous, const std::uint64_t weaponGenerationKey,
            const NativeScopeCameraWriteSource writeSource, const NativeScopeCameraFollowCapture& capture, const NativeScopeCameraFollowResult& result, const bool usedSightAnchor)
        {
            NativeScopeCameraDebugSnapshot snapshot{};
            snapshot.applySequence = previous.applySequence + 1;
            snapshot.weaponGenerationKey = weaponGenerationKey;
            snapshot.framesSinceApply = 0;
            snapshot.writeSource = writeSource;
            snapshot.captureValid = capture.valid;
            snapshot.targetValid = result.targetValid;
            snapshot.writeApplied = result.writeApplied;
            snapshot.immediateReadbackValid = result.immediateReadbackValid;
            snapshot.usedSightAnchor = usedSightAnchor;
            if (capture.valid) {
                snapshot.cameraWorldBefore = capture.cameraWorldBefore;
            }
            if (result.targetValid) {
                snapshot.targetCameraWorld = result.targetCameraWorld;
            }
            if (result.immediateReadbackValid) {
                snapshot.immediateCameraWorldAfter = result.immediateCameraWorldAfter;
            }
            return snapshot;
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

    void TwoHandedGrip::clearNativeScopeRigidFrame() { _nativeScopeRigidFrame = {}; }

    bool TwoHandedGrip::captureNativeScopeRigidFrame(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey, RE::NiNode* scopeCamera,
        const RE::NiTransform& nativeCameraWorld)
    {
        if (_nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey == currentWeaponGenerationKey && _nativeScopeRigidFrame.weaponNodeIdentity == weaponNode &&
            _nativeScopeRigidFrame.scopeCameraIdentity == scopeCamera) {
            return true;
        }

        clearNativeScopeRigidFrame();
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_nativeScopeSightAnchorValid || _nativeScopeSightAnchorWeaponNode != weaponNode ||
            _nativeScopeSightAnchorGenerationKey != currentWeaponGenerationKey || !scopeCamera || !isFiniteTransform(weaponNode->world) || !isFiniteTransform(nativeCameraWorld)) {
            return false;
        }

        const RE::NiTransform cameraWeaponLocal =
            native_scope_camera_follow_math::captureRigidSightFrameWeaponLocal(weaponNode->world, nativeCameraWorld, _nativeScopeSightAnchorWeaponLocal);
        if (!isFiniteTransform(cameraWeaponLocal) || std::abs(cameraWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeRigidFrame = NativeScopeRigidFrameState{
            .weaponGenerationKey = currentWeaponGenerationKey,
            .weaponNodeIdentity = weaponNode,
            .scopeCameraIdentity = scopeCamera,
            .cameraWeaponLocal = cameraWeaponLocal,
            .valid = true,
        };
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: native scope rigid frame captured generation={:016X} cameraLocal=({:.2f},{:.2f},{:.2f}) scale={:.3f}", currentWeaponGenerationKey,
            cameraWeaponLocal.translate.x, cameraWeaponLocal.translate.y, cameraWeaponLocal.translate.z, cameraWeaponLocal.scale);
        return true;
    }

    void TwoHandedGrip::synchronizeNativeScopePresentationAfterFrikUpdate(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_nativeScopeSightAnchorValid || _nativeScopeSightAnchorWeaponNode != weaponNode ||
            _nativeScopeSightAnchorGenerationKey != currentWeaponGenerationKey) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            return;
        }

        if ((_nativeScopeOverlayCalibration.valid && _nativeScopeOverlayCalibration.weaponGenerationKey != currentWeaponGenerationKey) ||
            (_nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey != currentWeaponGenerationKey)) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
        }

        /*
         * PlayerCharacter's native scope gate ran earlier in the frame. hFRIK
         * has now authored its engine-specific camera axis calibration; capture
         * that calibration once, then publish the complete rigid weapon-local
         * scope frame before FO4VR's later mono render. ROCK's two-hand solve
         * republishes this same frame from its final weapon transform below.
         */
        const NativeScopeCameraFollowCapture capture = captureNativeScopeCameraFollow(weaponNode);
        if (!capture.valid || !captureNativeScopeRigidFrame(weaponNode, currentWeaponGenerationKey, capture.camera, capture.cameraWorldBefore)) {
            return;
        }

        const bool overlayCalibrationReady = captureNativeScopeOverlayCalibration(capture.cameraWorldBefore, currentWeaponGenerationKey);
        const RE::NiTransform targetCameraWorld = native_scope_camera_follow_math::resolveRigidSightFrameWorld(weaponNode->world, _nativeScopeRigidFrame.cameraWeaponLocal);
        const NativeScopeCameraFollowResult result = applyNativeScopeCameraWorldTarget(capture, targetCameraWorld);
        if (overlayCalibrationReady && result.targetValid && result.writeApplied) {
            (void)applyNativeScopeOverlayTarget(result.targetCameraWorld, currentWeaponGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, currentWeaponGenerationKey,
                NativeScopeCameraWriteSource::PostFrikPresentationSync, capture, result, true);
        }
    }

    void TwoHandedGrip::clearNativeScopeOverlayAuthority(const bool restoreNativeLocal)
    {
        if (restoreNativeLocal && _nativeScopeOverlayCalibration.valid && _nativeScopeOverlayCalibration.hasAppliedLocal && runtime_state::isLocalSkeletonReady() &&
            RE::PlayerCharacter::GetSingleton()) {
            const auto* playerNodes = f4vr::getPlayerNodes();
            auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
            if (scopeParent == _nativeScopeOverlayCalibration.scopeParentIdentity &&
                areTransformsNearlyEqual(scopeParent->local, _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal)) {
                scopeParent->local = _nativeScopeOverlayCalibration.nativeScopeParentLocal;
                if (scopeParent->parent) {
                    f4vr::updateTransformsDown(scopeParent, true);
                } else {
                    scopeParent->world = scopeParent->local;
                    f4vr::updateTransformsDown(scopeParent, false);
                }
            }
        }

        _nativeScopeOverlayCalibration = {};
    }

    bool TwoHandedGrip::captureNativeScopeOverlayCalibration(
        const RE::NiTransform& nativeCameraWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (currentWeaponGenerationKey == 0 || !isFiniteTransform(nativeCameraWorld) ||
            std::abs(nativeCameraWorld.scale) <= 0.0001f || !RE::PlayerCharacter::GetSingleton()) {
            return false;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
        if (!scopeParent || !scopeParent->parent || !isFiniteTransform(scopeParent->local) ||
            std::abs(scopeParent->parent->world.scale) <= 0.0001f) {
            return false;
        }

        auto* scopeModelRoot = f4vr::find1StChildNode(scopeParent, "world_scope.nif");
        if (!scopeModelRoot || scopeModelRoot->parent != scopeParent ||
            !isFiniteTransform(scopeModelRoot->local) || std::abs(scopeModelRoot->local.scale) <= 0.0001f) {
            return false;
        }

        if (_nativeScopeOverlayCalibration.valid) {
            const bool sameOwner =
                _nativeScopeOverlayCalibration.weaponGenerationKey == currentWeaponGenerationKey &&
                _nativeScopeOverlayCalibration.scopeParentIdentity == scopeParent &&
                _nativeScopeOverlayCalibration.scopeModelRootIdentity == scopeModelRoot &&
                areTransformsNearlyEqual(scopeModelRoot->local, _nativeScopeOverlayCalibration.scopeModelRootLocal);
            const bool engineStillHasRockLocal =
                !_nativeScopeOverlayCalibration.hasAppliedLocal ||
                areTransformsNearlyEqual(scopeParent->local, _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal);
            if (sameOwner && engineStillHasRockLocal) {
                return true;
            }

            /*
             * A changed local transform is an engine re-authoring event (for
             * example an equip/OMOD change), not ours to restore. A changed
             * generation/node with our last local still present is restored
             * before the new native baseline is captured.
             */
            clearNativeScopeOverlayAuthority(!sameOwner && engineStillHasRockLocal);
        }

        RE::NiTransform nativeScopeModelRootWorld{};
        if (!tryGetComposedNodeWorld(scopeModelRoot, nativeScopeModelRootWorld)) {
            return false;
        }
        const RE::NiTransform modelRootCalibrationInCameraLocal =
            native_scope_overlay_follow_math::captureModelRootCalibrationInCameraLocal(
                nativeCameraWorld,
                nativeScopeModelRootWorld);
        if (!isFiniteTransform(modelRootCalibrationInCameraLocal) ||
            std::abs(modelRootCalibrationInCameraLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeOverlayCalibration = NativeScopeOverlayCalibrationState{
            .weaponGenerationKey = currentWeaponGenerationKey,
            .scopeParentIdentity = scopeParent,
            .scopeModelRootIdentity = scopeModelRoot,
            .scopeModelRootLocal = scopeModelRoot->local,
            .scopeModelRootCalibrationInCameraLocal = modelRootCalibrationInCameraLocal,
            .nativeScopeParentLocal = scopeParent->local,
            .lastAppliedScopeParentLocal = {},
            .valid = true,
            .hasAppliedLocal = false,
        };
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: native scope overlay calibrated generation={:016X} modelRootLocal=({:.2f},{:.2f},{:.2f}) cameraCalibrationScale={:.3f} nativeParentLocal=({:.2f},{:.2f},{:.2f})",
            currentWeaponGenerationKey,
            scopeModelRoot->local.translate.x,
            scopeModelRoot->local.translate.y,
            scopeModelRoot->local.translate.z,
            modelRootCalibrationInCameraLocal.scale,
            scopeParent->local.translate.x,
            scopeParent->local.translate.y,
            scopeParent->local.translate.z);
        return true;
    }

    bool TwoHandedGrip::applyNativeScopeOverlayTarget(
        const RE::NiTransform& correctedCameraWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!_nativeScopeOverlayCalibration.valid ||
            _nativeScopeOverlayCalibration.weaponGenerationKey != currentWeaponGenerationKey ||
            !isFiniteTransform(correctedCameraWorld) || !RE::PlayerCharacter::GetSingleton()) {
            return false;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeParent = playerNodes ? playerNodes->ScopeParentNode : nullptr;
        if (!scopeParent || scopeParent != _nativeScopeOverlayCalibration.scopeParentIdentity ||
            !scopeParent->parent || std::abs(scopeParent->parent->world.scale) <= 0.0001f) {
            return false;
        }

        auto* scopeModelRoot = f4vr::find1StChildNode(scopeParent, "world_scope.nif");
        if (!scopeModelRoot || scopeModelRoot != _nativeScopeOverlayCalibration.scopeModelRootIdentity ||
            scopeModelRoot->parent != scopeParent ||
            !areTransformsNearlyEqual(scopeModelRoot->local, _nativeScopeOverlayCalibration.scopeModelRootLocal)) {
            return false;
        }

        if (_nativeScopeOverlayCalibration.hasAppliedLocal && !areTransformsNearlyEqual(scopeParent->local, _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal)) {
            // FO4VR reclaimed the node after our calibration. The next
            // post-hFRIK presentation sync captures its new native baseline.
            return false;
        }

        const RE::NiTransform modelRootFineTuneLocal =
            native_scope_overlay_follow_math::makeModelRootFineTuneLocal<RE::NiTransform>(
                g_rockConfig.rockNativeScopeOverlayOffsetXGameUnits,
                g_rockConfig.rockNativeScopeOverlayOffsetYGameUnits,
                g_rockConfig.rockNativeScopeOverlayOffsetZGameUnits,
                g_rockConfig.rockNativeScopeOverlayPitchDegrees,
                g_rockConfig.rockNativeScopeOverlayYawDegrees,
                g_rockConfig.rockNativeScopeOverlayRollDegrees);
        const RE::NiTransform targetScopeModelRootWorld =
            native_scope_overlay_follow_math::resolveScopeModelRootWorld(
                correctedCameraWorld,
                _nativeScopeOverlayCalibration.scopeModelRootCalibrationInCameraLocal,
                modelRootFineTuneLocal);
        if (!isFiniteTransform(targetScopeModelRootWorld)) {
            return false;
        }

        const RE::NiTransform targetScopeParentWorld =
            native_scope_overlay_follow_math::resolveScopeParentWorldForModelRoot(
                targetScopeModelRootWorld,
                _nativeScopeOverlayCalibration.scopeModelRootLocal);
        if (!isFiniteTransform(targetScopeParentWorld)) {
            return false;
        }

        const RE::NiTransform targetScopeParentLocal =
            weapon_visual_authority_math::worldTargetToParentLocal(
                scopeParent->parent->world,
                targetScopeParentWorld);
        if (!isFiniteTransform(targetScopeParentLocal)) {
            return false;
        }

        scopeParent->local = targetScopeParentLocal;
        f4vr::updateTransformsDown(scopeParent, true);
        _nativeScopeOverlayCalibration.lastAppliedScopeParentLocal = targetScopeParentLocal;
        _nativeScopeOverlayCalibration.hasAppliedLocal = true;

        RE::NiTransform immediateScopeModelRootWorld{};
        return tryGetComposedNodeWorld(scopeModelRoot, immediateScopeModelRootWorld) &&
               areTransformsNearlyEqual(immediateScopeModelRootWorld, targetScopeModelRootWorld, 0.01f);
    }

    bool TwoHandedGrip::tryResolveNativeScopeGeometryDecision(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey, RE::NiNode* hmdNode,
        const RE::NiPoint3& hmdSampleOffsetLocal, const native_scope_activation_geometry::ConeThresholds& thresholds, const bool nativeScopeAlreadyActive,
        const bool nativeGeometryDecision, bool& outRockGeometryDecision)
    {
        outRockGeometryDecision = nativeGeometryDecision;
        if (!weaponNode || !hmdNode || currentWeaponGenerationKey == 0 || !_nativeScopeSightAnchorValid || _nativeScopeSightAnchorWeaponNode != weaponNode ||
            _nativeScopeSightAnchorGenerationKey != currentWeaponGenerationKey || !isFiniteTransform(weaponNode->world) || !isFiniteTransform(hmdNode->world)) {
            return false;
        }

        const native_scope_activation_geometry::ConeSample sample =
            native_scope_activation_geometry::sample(weaponNode->world, _nativeScopeSightAnchorWeaponLocal, hmdNode->world, hmdSampleOffsetLocal, thresholds);
        if (!sample.valid) {
            return false;
        }

        constexpr std::uint32_t kNativeScopeExitConfirmationFrames = 3;
        if (_nativeScopeExitDebounceGenerationKey != currentWeaponGenerationKey) {
            _nativeScopeExitDebounceGenerationKey = currentWeaponGenerationKey;
            _nativeScopeExitOutsideFrames = 0;
        }
        const bool insideCone = native_scope_activation_geometry::isInsideCone(sample, nativeScopeAlreadyActive, thresholds);
        const native_scope_activation_geometry::ExitDebounceResult stabilizedDecision = native_scope_activation_geometry::stabilizeExitDecision(
            insideCone,
            nativeScopeAlreadyActive,
            _nativeScopeExitOutsideFrames,
            kNativeScopeExitConfirmationFrames);
        _nativeScopeExitOutsideFrames = stabilizedDecision.consecutiveOutsideFrames;
        outRockGeometryDecision = stabilizedDecision.decision;
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeActivationDebugSnapshot = NativeScopeActivationDebugSnapshot{
                .evaluationSequence = _nativeScopeActivationDebugSnapshot.evaluationSequence + 1,
                .weaponGenerationKey = currentWeaponGenerationKey,
                .nativeGeometryDecision = nativeGeometryDecision,
                .rockGeometryDecision = outRockGeometryDecision,
                .nativeScopeAlreadyActive = nativeScopeAlreadyActive,
                .sample = sample,
                .thresholds = thresholds,
            };
        }
        return true;
    }

    void TwoHandedGrip::refreshNativeScopeSightAnchor(RE::NiNode* weaponNode, std::uint64_t currentWeaponGenerationKey, const WeaponCollision& weaponCollision)
    {
        if (_nativeScopeSightAnchorWeaponNode == weaponNode && _nativeScopeSightAnchorGenerationKey == currentWeaponGenerationKey) {
            return;
        }

        clearNativeScopeOverlayAuthority(true);
        clearNativeScopeRigidFrame();
        _nativeScopeExitDebounceGenerationKey = 0;
        _nativeScopeExitOutsideFrames = 0;

        _nativeScopeSightAnchorWeaponNode = weaponNode;
        _nativeScopeSightAnchorGenerationKey = currentWeaponGenerationKey;
        _nativeScopeSightAnchorWeaponLocal = {};
        _nativeScopeSightAnchorValid = false;

        if (!weaponNode || currentWeaponGenerationKey == 0) {
            return;
        }

        const WeaponCollision::NativeScopeSightAnchorSnapshot snapshot = weaponCollision.getNativeScopeSightAnchorSnapshot();
        if (snapshot.weaponGenerationKey != currentWeaponGenerationKey) {
            // Publication changed between the caller's generation read and
            // this snapshot. Leave the cache key unmatched so the next frame
            // retries instead of retaining geometry from another weapon.
            _nativeScopeSightAnchorWeaponNode = nullptr;
            _nativeScopeSightAnchorGenerationKey = 0;
            clearNativeScopeRigidFrame();
            return;
        }

        if (!snapshot.valid) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: native scope sight anchor unavailable generation={:016X}; preserving calibrated camera delta", currentWeaponGenerationKey);
            return;
        }

        _nativeScopeSightAnchorWeaponLocal = snapshot.anchorWeaponLocal;
        _nativeScopeSightAnchorValid = true;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: native scope sight anchor generation={:016X} bodies={} local=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f}) "
            "policy=rear-center",
            currentWeaponGenerationKey, snapshot.sightBodyCount, snapshot.anchorWeaponLocal.x, snapshot.anchorWeaponLocal.y, snapshot.anchorWeaponLocal.z,
            snapshot.sightBoundsMinWeaponLocal.x, snapshot.sightBoundsMinWeaponLocal.y, snapshot.sightBoundsMinWeaponLocal.z, snapshot.sightBoundsMaxWeaponLocal.x,
            snapshot.sightBoundsMaxWeaponLocal.y, snapshot.sightBoundsMaxWeaponLocal.z);
    }

    void TwoHandedGrip::refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        const bool scopeStateChanged = _scopeMenuOpenThisFrame != frameInput.scopeMenuOpen;
        _scopeMenuOpenThisFrame = frameInput.scopeMenuOpen;
        const bool driverFrameAuthorityWasActive = _scopeDriverFrameAuthorityActive;
        _scopeDriverFrameAuthorityActive = scope_safe_hand_frame_math::retainDriverFrameAuthority(
            _scopeMenuOpenThisFrame,
            isManualOwnershipActive(),
            driverFrameAuthorityWasActive);
        const bool driverFrameAuthorityStoppedThisFrame =
            driverFrameAuthorityWasActive && !_scopeDriverFrameAuthorityActive;

        if (scopeStateChanged) {
            // Never resume a pre-menu visual interpolation after hFRIK restores
            // its visible body. The weapon solver itself remains continuous.
            resetLockedHandVisualLerp();
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: native scope hand-frame menu={} solver={} leftCache={} rightCache={}",
                _scopeMenuOpenThisFrame ? "open" : "closed",
                _scopeDriverFrameAuthorityActive ?
                    (_scopeMenuOpenThisFrame ? "frik-driver" : "frik-driver-latched") :
                    "root-flattened",
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
        const auto refreshHand = [this, driverFrameAuthorityStoppedThisFrame, frameDeltaSeconds](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
            ScopeSafeHandFrameState& state = _scopeSafeHandFrames[isLeft ? 0u : 1u];
            state.currentHandWorldValid = false;

            RE::NiTransform rootHandWorld{};
            const bool rootHandValid = !_scopeDriverFrameAuthorityActive &&
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
                _scopeDriverFrameAuthorityActive,
                rootHandValid,
                reconstructedHandValid,
                state.hasLastHandWorld,
                state.consecutiveDriverMissFrames,
                SCOPE_DRIVER_MISS_GRACE_FRAMES);

            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::RootFlattened) {
                const bool recentScopedHandAvailable = state.hasLastHandWorld &&
                                                       state.consecutiveDriverMissFrames < SCOPE_DRIVER_MISS_GRACE_FRAMES;
                if (driverFrameAuthorityStoppedThisFrame && (reconstructedHandValid || recentScopedHandAvailable)) {
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
            } else if (_scopeDriverFrameAuthorityActive) {
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
        bool firingGripProximityAuthorityEnabled,
        const EquippedWeaponGripMode& gripMode)
    {
        _hasSolvedWeaponTransform = false;
        _firingGripReattachHoverInsideRadius = false;
        _firingGripReattachHoverHandIsLeft = _firingHandIsLeft;
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            _nativeScopeCameraDebugSnapshot.framesSinceApply != (std::numeric_limits<std::uint32_t>::max)()) {
            ++_nativeScopeCameraDebugSnapshot.framesSinceApply;
        }

        refreshNativeScopeSightAnchor(weaponNode, currentWeaponGenerationKey, weaponCollision);
        refreshScopeSafeHandFrames(weaponNode, frameInput, dt);

        if (!runtime_state::isLocalSkeletonReady() || !weaponNode) {
            clearAllVisualReturns("skeleton-or-weapon-unavailable", true, true);
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            return;
        }

        updateWeaponVisualReturn(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            dt);

        EquippedWeaponGripFrameInput stableFrameInput = frameInput;
        if (_persistentEquippedCarryActive && isManualOwnershipActive()) {
            /*
             * A Pip-Boy equip has no grab press to retain PrimaryOnly. Keep
             * the firing grip virtually closed until the player physically
             * holds it once; only that armed hand's later release is allowed
             * through the normal debounce/drop machinery. This preserves all
             * existing two-hand, detach, stash, and handoff gestures without
             * an immediate phantom drop on the first post-menu frame.
             */
            if (frameInput.primaryGripInput.held || frameInput.primaryGripInput.pressed) {
                _persistentEquippedCarryDetachArmed = true;
            }
            if (!_persistentEquippedCarryDetachArmed) {
                stableFrameInput.primaryGripInput.held = true;
                stableFrameInput.primaryGripInput.pressed = false;
                stableFrameInput.primaryGripInput.released = false;
            }
        }
        const auto primaryReleaseDecision = equipped_weapon_manual_ownership_policy::debouncePrimaryGripRelease(
            _primaryReleaseDebounce,
            stableFrameInput.primaryGripInput.held);
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
            clearAllVisualReturns("equipped-weapon-identity-changed", true, true);
            transitionToInactive(false);
            return;
        }

        /*
         * Left-firing feed-forward pre-write: while ROCK owns the weapon node
         * (left-firing topology), FRIK's earlier skeleton pass has already
         * rewritten the node to its OFFHAND GLUE pose, so at this point
         * weaponNode->world is glue space, not the real carried pose. Every
         * world<->weapon-local conversion below (part-grip captures, mesh
         * grab points, promotion distances, the two-hand solver base) would
         * silently mix real-space palm/contact points with that glue frame -
         * the round-4 corrupted captures. Publishing the canonical
         * feed-forward pose FIRST makes the node a real-space basis for all
         * existing math with no per-call-site special cases; the state
         * handlers below re-publish their final solved pose as before.
         * Right-firing reads FRIK's authored carry and is untouched.
         * (PhysicsInteraction additionally publishes this before the frame's
         * weapon interaction probes - see the header note.)
         */
        (void)publishLeftFiringFeedForwardWeaponPose(weaponNode);

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
                    firingGripProximityAuthorityEnabled,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            }
            break;

        case TwoHandedState::Gripping:
            if (_supportGripAgeFrames < (std::numeric_limits<std::uint32_t>::max)()) {
                ++_supportGripAgeFrames;
            }
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
                const auto releaseAction = weapon_two_handed_grip_math::resolveSupportReleaseManualAction(
                    weapon_two_handed_grip_math::SupportReleaseOwnershipInput{
                        .firingGripOwnershipEnabled = gripMode.firingGripOwnershipEnabled,
                        .primaryDetachEnabled = gripMode.primaryDetachEnabled,
                        .primaryGripHeld = primaryGripInput.held,
                    });
                if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::KeepPrimaryOwnership) {
                    beginHandVisualReturn(supportHandIsLeft, "support-released-primary-held");
                    if (ownsWeaponTransform()) {
                        beginHandVisualReturn(_firingHandIsLeft, "two-hand-primary-return-to-native-carry");
                        if (!_firingHandIsLeft) {
                            beginWeaponVisualReturn("support-released-primary-held");
                        }
                    }
                    transitionToPrimaryOnly(
                        _activeWeaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "support-released-primary-held");
                } else if (releaseAction == weapon_two_handed_grip_math::SupportReleaseManualAction::DropEquippedWeapon) {
                    beginHandVisualReturn(supportHandIsLeft, "support-released-drop");
                    beginHandVisualReturn(_firingHandIsLeft, "primary-released-drop");
                    requestEquippedWeaponDrop(
                        "support-released-primary-not-held",
                        equipped_weapon_drop_policy::sourceForSupportRelease(primaryGripInput.released));
                } else {
                    beginHandVisualReturn(supportHandIsLeft, "support-released");
                    beginHandVisualReturn(_firingHandIsLeft, "primary-authority-cleared");
                    if (ownsWeaponTransform()) {
                        beginWeaponVisualReturn("support-released");
                    }
                    transitionToInactive(ownsWeaponTransform());
                }
            } else if ((gripMode.primaryDetachEnabled || gripMode.ambidextrousHandoffEnabled) && !primaryGripInput.held &&
                       equipped_weapon_manual_ownership_policy::shouldDeferPrimaryReleaseActionForFreshSupportGrip(_supportGripAgeFrames)) {
                /*
                 * The firing-grip release confirmed while the support grab is
                 * only a few frames old: same physical gesture or a
                 * grab-synchronized grip flicker, never an independent
                 * release. Hold the two-handed grip unchanged; a re-pressed
                 * grip resumes normally, and promotion/detach run below once
                 * the grab has aged. leftGripHeld/rightGripHeld in the log
                 * discriminate a physical flicker (both pipelines open) from
                 * an input-path divergence (normal pipeline still held).
                 */
                if (!_freshSupportGripDeferLogged) {
                    _freshSupportGripDeferLogged = true;
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: deferring firing-grip release action while support grip is fresh age={} firingHand={} leftGripHeld={} rightGripHeld={}",
                        _supportGripAgeFrames,
                        _firingHandIsLeft ? "left" : "right",
                        stableFrameInput.leftGripHeld ? "yes" : "no",
                        stableFrameInput.rightGripHeld ? "yes" : "no");
                }
                updateGripping(_activeWeaponNode, dt);
            } else if (gripMode.ambidextrousHandoffEnabled && !primaryGripInput.held && tryPromoteSupportGripToFiringGrip(_activeWeaponNode)) {
                // The support hand was wrapped over the firing grip when the
                // firing hand opened: it takes over the SAME weapon-relative
                // grip in place (seamless hand switch, pistol shooting-cup
                // flow). State is PrimaryOnly under the new firing hand.
            } else if (gripMode.primaryDetachEnabled &&
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
            } else if (!gripMode.primaryDetachEnabled) {
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
            } else if (!gripMode.firingGripOwnershipEnabled) {
                transitionToInactive(false);
            } else if (supportTouchingSupport && weapon_two_handed_grip_math::canStartSupportGrip(supportTouchingSupport, supportGripHeld, supportHandHoldingObject)) {
                transitionToGripping(interactionWeaponNode,
                    decision,
                    weaponCollision,
                    supportAuthorityMode,
                    firingGripProximityAuthorityEnabled,
                    currentEquippedWeaponOwnershipKey,
                    supportRuntimeState.providerPartAuthority);
            } else {
                updatePrimaryOnlyGrip(
                    _activeWeaponNode,
                    currentEquippedWeaponOwnershipKey,
                    primaryGripInput,
                    gripMode.primaryDetachEnabled);
            }
            break;
        }

        refreshRightNativeCanonicalFrame(weaponNode, currentWeaponGenerationKey);

        // Enforce the left-firing weapon-node ownership contract after every
        // state/role transition this frame (idempotent; also the parent
        // watchdog for engine-side re-attach).
        syncFiringHandWeaponNodeOwnership(weaponNode);
        updateHandVisualReturns(dt);
    }

    void TwoHandedGrip::reset()
    {
        clearAllVisualReturns("reset", false, true);
        clearNativeScopeOverlayAuthority(true);
        _equippedWeaponDropRequest = {};
        _hapticEvents = {};
        _firingGripReattachHoverInsideRadius = false;
        _nativeScopeSightAnchorWeaponNode = nullptr;
        _nativeScopeSightAnchorGenerationKey = 0;
        _nativeScopeSightAnchorWeaponLocal = {};
        _nativeScopeSightAnchorValid = false;
        _nativeScopeExitDebounceGenerationKey = 0;
        _nativeScopeExitOutsideFrames = 0;
        _nativeScopeCameraDebugSnapshot = {};
        _nativeScopeActivationDebugSnapshot = {};
        clearNativeScopeRigidFrame();
        _scopeSafeHandFrames = {};
        _scopeDriverFrameAuthorityActive = false;
        _scopeHandAuthorityCleanupPending = _scopeHandAuthorityCleanupPending || _scopeMenuOpenThisFrame;
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikPrimaryWeaponPose();
        _hasRightFiringHandCanonicalWeaponLocal = false;
        _rightFiringHandCanonicalGenerationKey = 0;
        _rightFiringHandCanonicalWeaponLocal = {};
        _rightFiringGripCanonicalWeaponLocal = {};
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
        _supportGripAgeFrames = 0;
        _freshSupportGripDeferLogged = false;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = false;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        _lastPublishedHandWorld = {};
        _hasLastPublishedHandWorld = {};
        _lastRenderedWeaponWorld = {};
        _hasLastRenderedWeaponWorld = false;
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

    bool TwoHandedGrip::isHandVisualReturnActive(const bool isLeft) const
    {
        return _returningHandVisuals[isLeft ? 0u : 1u].transition.active;
    }

    bool TwoHandedGrip::hasVisualAuthorityForHand(const bool isLeft) const
    {
        if (isHandVisualReturnActive(isLeft) || partGrip(isLeft).active) {
            return true;
        }
        return isLeft == _firingHandIsLeft &&
            _state == TwoHandedState::Gripping &&
            weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode);
    }

    void TwoHandedGrip::recordPublishedHandWorld(const bool isLeft, const RE::NiTransform& appliedWorld)
    {
        if (!isUsableHandAuthorityTransform(appliedWorld)) {
            return;
        }
        const std::size_t index = isLeft ? 0u : 1u;
        _lastPublishedHandWorld[index] = appliedWorld;
        _hasLastPublishedHandWorld[index] = true;
    }

    void TwoHandedGrip::beginHandVisualReturn(const bool isLeft, const char* reason)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _returningHandVisuals[index].transition;
        if (!g_rockConfig.rockWeaponVisualReturnEnabled ||
            !_hasLastPublishedHandWorld[index] ||
            !isUsableHandAuthorityTransform(_lastPublishedHandWorld[index]) ||
            !frik_visual_authority::isAvailable()) {
            clearHandVisualReturn(isLeft, "not-eligible", false);
            return;
        }

        state.begin(_lastPublishedHandWorld[index]);
        if (!frik_visual_authority::applyExternalHandWorldTransform(
                RETURN_HAND_TAG,
                handFromBool(isLeft),
                state.start,
                RETURN_HAND_VISUAL_PRIORITY)) {
            state.clear();
            (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: hand return start failed hand={}", isLeft ? "left" : "right");
            return;
        }

        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: hand return started hand={} reason={} from=({:.2f},{:.2f},{:.2f})",
            isLeft ? "left" : "right",
            reason ? reason : "unknown",
            state.start.translate.x,
            state.start.translate.y,
            state.start.translate.z);
    }

    void TwoHandedGrip::updateHandVisualReturns(const float dt)
    {
        if (_scopeMenuOpenThisFrame) {
            return;
        }

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            auto& state = _returningHandVisuals[index].transition;
            if (!state.active) {
                continue;
            }

            RE::NiTransform targetWorld{};
            if (!frik_visual_authority::isAvailable() ||
                !tryGetSolverHandTransform(isLeft, targetWorld) ||
                !isUsableHandAuthorityTransform(targetWorld)) {
                clearHandVisualReturn(isLeft, "tracked-hand-unavailable", true);
                continue;
            }

            const bool timingPending = !state.durationInitialized;
            const float initialDistance = timingPending ?
                hand_visual_lerp_math::distanceGameUnits(state.start.translate, targetWorld.translate) :
                0.0f;
            const float initialAngleDegrees = timingPending ?
                hand_visual_lerp_math::rotationDistanceDegrees(state.start, targetWorld) :
                0.0f;
            const auto result = hand_visual_lerp_math::advanceVisualReturn(
                state,
                targetWorld,
                dt,
                hand_visual_lerp_math::VisualReturnConfig{
                    .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                    .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                    .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                    .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                    .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                    .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
                });
            if (timingPending) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return timing hand={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                    isLeft ? "left" : "right",
                    initialDistance,
                    initialAngleDegrees,
                    state.durationSeconds);
            }
            if (!isUsableHandAuthorityTransform(result.transform) ||
                !frik_visual_authority::applyExternalHandWorldTransform(
                    RETURN_HAND_TAG,
                    handFromBool(isLeft),
                    result.transform,
                    RETURN_HAND_VISUAL_PRIORITY)) {
                clearHandVisualReturn(isLeft, "publish-failed", true);
                continue;
            }

            if (result.reachedTarget) {
                const float completedDuration = state.durationSeconds;
                (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
                state.clear();
                _hasLastPublishedHandWorld[index] = false;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return completed hand={} duration={:.3f}s",
                    isLeft ? "left" : "right",
                    completedDuration);
            }
        }
    }

    void TwoHandedGrip::clearHandVisualReturn(const bool isLeft, const char* reason, const bool logCancellation)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _returningHandVisuals[index].transition;
        const bool wasActive = state.active;
        (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
        state.clear();
        _hasLastPublishedHandWorld[index] = false;
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: hand return cancelled hand={} reason={}",
                isLeft ? "left" : "right",
                reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::cancelHandVisualReturn(const bool isLeft, const char* reason)
    {
        clearHandVisualReturn(isLeft, reason, true);
    }

    void TwoHandedGrip::beginWeaponVisualReturn(const char* reason)
    {
        if (!g_rockConfig.rockWeaponVisualReturnEnabled ||
            _returningWeaponVisual.localTransition.active ||
            !_activeWeaponNode ||
            !_hasWeaponNodeLocalBaseline ||
            _activeWeaponGenerationKey == 0 ||
            _activeEquippedWeaponOwnershipKey == 0) {
            return;
        }

        RE::NiTransform startWorld = _hasLastRenderedWeaponWorld ? _lastRenderedWeaponWorld : _activeWeaponNode->world;
        if (!isFiniteTransform(startWorld) || !isFiniteTransform(_weaponNodeLocalBaseline)) {
            return;
        }

        RE::NiNode* nativeParent = _activeWeaponNode->parent;
        if (_weaponNodeReparentedToLeftHand) {
            nativeParent = resolveFirstPersonHandNode(false);
            if (!nativeParent) {
                return;
            }
        }
        if (!nativeParent) {
            return;
        }

        const RE::NiTransform startLocal = weapon_visual_authority_math::worldTargetToParentLocal(nativeParent->world, startWorld);
        if (!isFiniteTransform(startLocal)) {
            return;
        }

        /*
         * blockPrimaryWeaponNodeOwnership is hFRIK's external LEFT-carry
         * topology switch, not a transform-write-only blocker. Retaining it
         * here makes hFRIK reparent the weapon back under LArm_Hand on the next
         * frame, which invalidates this right-parent-local return and snaps the
         * weapon immediately. Release left-carry topology before beginning the
         * overlay. ROCK runs after hFRIK and republishes the interpolated node
         * every frame, so hFRIK's earlier native write cannot reach rendering;
         * at the exact endpoint both writers already agree on the baseline.
         */
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        if (_activeWeaponNode->parent != nativeParent) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: weapon return skipped because native right-hand parenting could not be restored");
            return;
        }

        ReturningWeaponVisualState returnState{};
        returnState.weaponNode = _activeWeaponNode;
        returnState.nativeParent = nativeParent;
        returnState.weaponGenerationKey = _activeWeaponGenerationKey;
        returnState.equippedWeaponOwnershipKey = _activeEquippedWeaponOwnershipKey;
        returnState.nativeBaselineLocal = _weaponNodeLocalBaseline;
        returnState.retainPrimaryPoseBlocker = _firingHandIsLeft;
        returnState.localTransition.begin(startLocal);
        returnState.localTransition.durationSeconds = hand_visual_lerp_math::computeVisualReturnDuration(
            startLocal,
            returnState.nativeBaselineLocal,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
            });
        returnState.localTransition.durationInitialized = true;
        _returningWeaponVisual = returnState;
        _activeWeaponNode->local = startLocal;
        f4vr::updateTransformsDown(_activeWeaponNode, true);
        _lastRenderedWeaponWorld = _activeWeaponNode->world;
        _hasLastRenderedWeaponWorld = true;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: weapon return started reason={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
            reason ? reason : "unknown",
            hand_visual_lerp_math::distanceGameUnits(startLocal.translate, returnState.nativeBaselineLocal.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(startLocal, returnState.nativeBaselineLocal),
            _returningWeaponVisual.localTransition.durationSeconds);
    }

    void TwoHandedGrip::updateWeaponVisualReturn(
        RE::NiNode* currentWeaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const float dt)
    {
        auto& state = _returningWeaponVisual;
        if (!state.localTransition.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() ||
            !currentWeaponNode ||
            currentWeaponNode != state.weaponNode ||
            currentWeaponGenerationKey != state.weaponGenerationKey ||
            currentEquippedWeaponOwnershipKey != state.equippedWeaponOwnershipKey ||
            !state.nativeParent ||
            !isFiniteTransform(state.nativeParent->world) ||
            currentWeaponNode->parent != state.nativeParent) {
            clearAllVisualReturns("weapon-identity-or-parent-changed", true, true);
            return;
        }

        const auto result = hand_visual_lerp_math::advanceVisualReturn(
            state.localTransition,
            state.nativeBaselineLocal,
            dt,
            hand_visual_lerp_math::VisualReturnConfig{
                .minSeconds = g_rockConfig.rockWeaponVisualReturnTimeMin,
                .maxSeconds = g_rockConfig.rockWeaponVisualReturnTimeMax,
                .minDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMinDistance,
                .maxDistanceGameUnits = g_rockConfig.rockWeaponVisualReturnMaxDistance,
                .minAngleDegrees = g_rockConfig.rockWeaponVisualReturnMinAngleDegrees,
                .maxAngleDegrees = g_rockConfig.rockWeaponVisualReturnMaxAngleDegrees,
            });
        if (!isFiniteTransform(result.transform)) {
            clearWeaponVisualReturn("non-finite-return-transform", true, true);
            return;
        }

        const RE::NiTransform returnedWeaponWorld =
            transform_math::composeTransforms(state.nativeParent->world, result.transform);
        if (!applyWeaponVisualAuthority(currentWeaponNode, returnedWeaponWorld, state.weaponGenerationKey)) {
            clearWeaponVisualReturn("weapon-return-publish-failed", true, true);
            return;
        }
        _lastSolvedWeaponTransform = currentWeaponNode->world;
        _hasSolvedWeaponTransform = true;
        if (result.reachedTarget) {
            const float completedDuration = state.localTransition.durationSeconds;
            clearWeaponVisualReturn("completed", false, true);
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return completed duration={:.3f}s", completedDuration);
        }
    }

    void TwoHandedGrip::clearWeaponVisualReturn(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        const bool wasActive = _returningWeaponVisual.localTransition.active;
        const bool retainedPrimaryPoseBlocker = _returningWeaponVisual.retainPrimaryPoseBlocker;
        RE::NiNode* returnNode = _returningWeaponVisual.weaponNode;
        _returningWeaponVisual = {};
        if (restoreBlockers) {
            releaseFiringHandWeaponNodeOwnership(returnNode);
            if (retainedPrimaryPoseBlocker) {
                restoreFrikPrimaryWeaponPose();
            }
        }
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return cancelled reason={}", reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::clearAllVisualReturns(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        clearHandVisualReturn(true, reason, logCancellation);
        clearHandVisualReturn(false, reason, logCancellation);
        clearWeaponVisualReturn(reason, logCancellation, restoreBlockers);
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
                g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                true, g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits, -1.0f,
                g_rockConfig.rockGrabThumbSweepMaxOpenValue, g_rockConfig.rockGrabFingerSweepMaxOpenValue);
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

    void TwoHandedGrip::releasePartGrip(bool isLeft, const char* reason, const bool smoothHandReturn)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active) {
            return;
        }
        if (smoothHandReturn) {
            beginHandVisualReturn(isLeft, reason);
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
        bool firingGripProximityAuthorityEnabled,
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
        RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
        if (_returningWeaponVisual.localTransition.active && _returningWeaponVisual.weaponNode == weaponNode) {
            nativeWeaponLocalBaseline = _returningWeaponVisual.nativeBaselineLocal;
            clearWeaponVisualReturn("new-two-hand-acquisition", true, true);
        }

        /*
         * A LEFT firing hand entering a two-handed grip KEEPS its captured
         * firing-grip frames: they hold the mirrored canonical hold
         * (takeover-committed), and recapturing from the live hand both
         * replaced that authored hold with the momentary squeeze orientation
         * (round-2 arm break) and rebased the promotion grip point onto
         * whatever pose the node carried at grab time (round-4 role theft).
         * The right hand recaptures as before - its frames deliberately ride
         * FRIK's authored carry and feed the canonical snapshot.
         */
        const bool keepLeftFiringHold = _firingHandIsLeft && _hasFiringHandWeaponLocal;

        _authorityMode = supportAuthorityMode;
        _activeWeaponNode = weaponNode;
        _activeWeaponGenerationKey = decision.weaponGenerationKey;
        _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
        _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
        _hasWeaponNodeLocalBaseline = true;
        if (!keepLeftFiringHold) {
            _primaryGripConfidence = 0.0f;
            _hasFiringHandWeaponLocal = false;
        }
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

        const bool reuseRightFiringCanonicalGrip = scope_safe_hand_frame_math::shouldReuseRightFiringCanonicalGrip(_scopeMenuOpenThisFrame, _firingHandIsLeft,
            _hasRightFiringHandCanonicalWeaponLocal, _rightFiringHandCanonicalGenerationKey, decision.weaponGenerationKey);
        if (_scopeMenuOpenThisFrame && !_firingHandIsLeft && !reuseRightFiringCanonicalGrip) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "TwoHandedGrip: scoped support grip start deferred because the matching pre-scope firing grip is unavailable generation={:016X} canonicalGeneration={:016X}",
                decision.weaponGenerationKey, _rightFiringHandCanonicalGenerationKey);
            restoreFrikOffhandGrip();
            return;
        }
        if (reuseRightFiringCanonicalGrip) {
            _primaryHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
            _primaryGripLocal = _rightFiringGripCanonicalWeaponLocal;
        }

        const RE::NiPoint3 primaryPalmPos =
            reuseRightFiringCanonicalGrip ? weaponLocalToWorld(_primaryGripLocal, weaponNode) : computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft);
        if (!keepLeftFiringHold) {
            if (!reuseRightFiringCanonicalGrip) {
                _primaryGripLocal = worldToWeaponLocal(primaryPalmPos, weaponNode);
                _primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), primaryTransform);
            }
            _primaryGripConfidence = 1.0f;
            _hasFiringHandWeaponLocal = true;
            _firingGripSequence = ++_gripCaptureSequence;
            // A right-hand capture here rides FRIK's authored carry: snapshot it
            // as the canonical hold that left takeovers apply mirrored.
            rememberRightFiringHandCanonicalFrame();
        }

        /*
         * At capture the firing grip point is the primary palm, so the support
         * palm distance selects visual-only attachment near the firing grip or
         * full two-handed manipulation farther out. This applies uniformly to
         * equipped weapons and is bypassed by explicit provider grab modes.
         * If the distance cannot be measured, retain full authority rather
         * than assuming the hand is inside the proximity radius.
         */
        if (firingGripProximityAuthorityEnabled) {
            RE::NiTransform supportTransform{};
            if (tryGetSolverHandTransform(supportHandIsLeft, supportTransform)) {
                const RE::NiPoint3 supportPalmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(supportTransform, supportHandIsLeft);
                const RE::NiPoint3 supportToGrip = sub(primaryPalmPos, supportPalmPos);
                const float supportPalmToGripDistance = std::sqrt(dot(supportToGrip, supportToGrip));
                if (std::isfinite(supportPalmToGripDistance)) {
                    _authorityMode = weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                        supportPalmToGripDistance,
                        g_rockConfig.rockFiringGripProximitySupportRadius);
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: firing-grip proximity support distance={:.2f} radius={:.2f} mode={}",
                        supportPalmToGripDistance,
                        g_rockConfig.rockFiringGripProximitySupportRadius,
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
        _supportGripAgeFrames = 0;
        _freshSupportGripDeferLogged = false;

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, partKind={}, pose={}, authorityMode={}, generation={:016X}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, supportGrip.gripLocal.x, supportGrip.gripLocal.y, supportGrip.gripLocal.z,
            _lockedGripSeparationWorld, reuseRightFiringCanonicalGrip ? "pre-scope-canonical" : (_scopeMenuOpenThisFrame ? "frik-driver-reconstructed" : "root-flattened"),
            _primaryGripConfidence, static_cast<int>(supportGrip.partKind), static_cast<int>(supportGrip.gripPose), static_cast<int>(_authorityMode), _activeWeaponGenerationKey);
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        const bool weaponReturnActive = _returningWeaponVisual.localTransition.active;
        // Weapon-node topology always returns to native immediately. A visual
        // return owns only ROCK's later transform publication, never hFRIK's
        // external-left-carry topology switch.
        releaseFiringHandWeaponNodeOwnership(_activeWeaponNode);
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikOffhandGrip();
        if (!weaponReturnActive) {
            restoreFrikPrimaryWeaponPose();
        }
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
        _supportGripAgeFrames = 0;
        _freshSupportGripDeferLogged = false;
        _authorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        _hasSolvedWeaponTransform = weaponReturnActive || (publishRestoredWeaponTransform && restoredWeaponTransformAvailable);
        if (weaponReturnActive && _hasLastRenderedWeaponWorld) {
            _lastSolvedWeaponTransform = _lastRenderedWeaponWorld;
        } else if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = restoredWeaponTransform;
        }
        _primaryHandWeaponLocal = {};
        _hasFiringHandWeaponLocal = false;
        _primaryGripConfidence = 0.0f;
        _activeWeaponNode = nullptr;
        _activeWeaponGenerationKey = 0;
        _activeEquippedWeaponOwnershipKey = 0;
        _primaryReleaseDebounce = {};
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        _weaponNodeLocalBaseline = {};
        _hasWeaponNodeLocalBaseline = false;
        resetLockedHandVisualLerp();
        if (!isHandVisualReturnActive(true)) {
            _hasLastPublishedHandWorld[0] = false;
        }
        if (!isHandVisualReturnActive(false)) {
            _hasLastPublishedHandWorld[1] = false;
        }
        if (!weaponReturnActive) {
            _hasLastRenderedWeaponWorld = false;
        }
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

        if (!blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: primary detach skipped because hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }
        beginHandVisualReturn(_firingHandIsLeft, "primary-detach-part-carry");
        clearPrimaryGripPose(_firingHandIsLeft);
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

    bool TwoHandedGrip::canBeginPrimaryOnlyGripForHand(const bool isLeft)
    {
        return !isLeft || ambidextrousFiringGripTakeoverAvailable();
    }

    bool TwoHandedGrip::beginPrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool firingHandIsLeft,
        const RE::NiTransform* capturedFiringHandWeaponLocal,
        const RE::NiPoint3* capturedFiringGripWeaponLocal)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0 || _state != TwoHandedState::Inactive ||
            !canBeginPrimaryOnlyGripForHand(firingHandIsLeft)) {
            return false;
        }
        if (firingHandIsLeft &&
            (!capturedFiringHandWeaponLocal || !isFiniteTransform(*capturedFiringHandWeaponLocal) ||
                !capturedFiringGripWeaponLocal ||
                !std::isfinite(capturedFiringGripWeaponLocal->x) ||
                !std::isfinite(capturedFiringGripWeaponLocal->y) ||
                !std::isfinite(capturedFiringGripWeaponLocal->z))) {
            return false;
        }
        if (firingHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: left primary-grip start skipped because the hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }

        setFiringHand(firingHandIsLeft, "primary-grip-start-hand");
        if (firingHandIsLeft) {
            _primaryHandWeaponLocal = *capturedFiringHandWeaponLocal;
            _hasFiringHandWeaponLocal = true;
        }

        if (!transitionToPrimaryOnly(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                "primary-grip-start")) {
            _primaryHandWeaponLocal = {};
            _hasFiringHandWeaponLocal = false;
            setFiringHand(false, "primary-grip-start-failed");
            restoreFrikPrimaryWeaponPose();
            return false;
        }
        if (firingHandIsLeft) {
            // The newly equipped node inherits the exact loose-model firing
            // grip; transitionToPrimaryOnly must not recapture it from the
            // left palm against FRIK's still-right-native first frame.
            _primaryGripLocal = *capturedFiringGripWeaponLocal;
            _primaryGripConfidence = 1.0f;
        }
        // Only a fresh grab pulses; transitionToPrimaryOnly is also reached
        // from support-release paths where the firing grip never changed.
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        _firingGripSequence = ++_gripCaptureSequence;
        return true;
    }

    bool TwoHandedGrip::beginPersistentEquippedCarry(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || currentEquippedWeaponOwnershipKey == 0 ||
            _state != TwoHandedState::Inactive || !canBeginPrimaryOnlyGripForHand(true) ||
            !_hasRightFiringHandCanonicalWeaponLocal ||
            _rightFiringHandCanonicalGenerationKey != currentWeaponGenerationKey) {
            return false;
        }

        RE::NiTransform rightHandWorld{};
        RE::NiTransform leftHandWorld{};
        RE::NiTransform mirroredLeftHold{};
        if (!tryGetSolverHandTransform(false, rightHandWorld) ||
            !tryGetSolverHandTransform(true, leftHandWorld) ||
            !tryBuildMirroredLeftFiringHandWeaponLocal(
                _rightFiringHandCanonicalWeaponLocal,
                _rightFiringGripCanonicalWeaponLocal,
                rightHandWorld,
                leftHandWorld,
                mirroredLeftHold,
                true)) {
            return false;
        }

        if (!beginPrimaryOnlyGrip(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                true,
                &mirroredLeftHold,
                &_rightFiringGripCanonicalWeaponLocal)) {
            return false;
        }

        _persistentEquippedCarryActive = true;
        _persistentEquippedCarryDetachArmed = false;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: persistent Pip-Boy left-hand carry active generation={:016X} ownership={:016X}",
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);
        return true;
    }

    void TwoHandedGrip::clearPersistentEquippedCarry(const char* reason)
    {
        if (!_persistentEquippedCarryActive) {
            return;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: clearing persistent Pip-Boy carry reason={}",
            reason ? reason : "unknown");
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        if (isManualOwnershipActive()) {
            transitionToInactive(false);
        }
    }

    bool TwoHandedGrip::publishLeftFiringFeedForwardWeaponPose(RE::NiNode* weaponNode)
    {
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            (_state != TwoHandedState::Gripping && _state != TwoHandedState::PrimaryOnly) ||
            !_firingHandIsLeft || !_hasFiringHandWeaponLocal) {
            return false;
        }

        RE::NiTransform leftFiringHandTransform{};
        if (!tryGetSolverHandTransform(true, leftFiringHandTransform)) {
            return false;
        }

        const RE::NiTransform feedForwardWeaponWorld = transform_math::composeTransforms(
            leftFiringHandTransform, transform_math::invertTransform(_primaryHandWeaponLocal));
        if (!isFiniteTransform(feedForwardWeaponWorld)) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, feedForwardWeaponWorld);
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
            RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
            if (_returningWeaponVisual.localTransition.active && _returningWeaponVisual.weaponNode == weaponNode) {
                nativeWeaponLocalBaseline = _returningWeaponVisual.nativeBaselineLocal;
                clearWeaponVisualReturn("new-primary-acquisition", true, true);
            }
            _activeWeaponNode = weaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            _activeEquippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
            _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
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
        if (!_firingHandIsLeft) {
            // FRIK's primary weapon pose targets the game-primary RIGHT hand.
            // While the LEFT hand fires it stays blocked; hFRIK poses the
            // left hand itself from the weapon-node ownership block state
            // (mirrored copy of the animated right weapon hand).
            restoreFrikPrimaryWeaponPose();
        }
        _partGrips = {};
        _partCarryPivotIsLeft = true;
        _partCarryGripSeparationWorld = 0.0f;
        _hasSolvedWeaponTransform = _returningWeaponVisual.localTransition.active && _hasLastRenderedWeaponWorld;
        if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = _lastRenderedWeaponWorld;
        }
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
        clearWeaponVisualReturn("equipped-weapon-drop", true, true);
        transitionToInactive(false);
    }

    void TwoHandedGrip::updatePrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const EquippedWeaponPrimaryGripInput& primaryGripInput,
        const bool primaryDetachEnabled)
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
                .primaryGripRetained = equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership(
                    primaryDetachEnabled,
                    primaryGripInput.held),
                .supportGripRetained = false,
            });

        if (manualDecision.dropRequested) {
            beginHandVisualReturn(_firingHandIsLeft, "primary-only-drop");
            requestEquippedWeaponDrop("primary-only-grip-released",
                _firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right);
            return;
        }

        if (manualDecision.cleared) {
            beginHandVisualReturn(_firingHandIsLeft, "primary-only-released");
            if (_firingHandIsLeft) {
                beginWeaponVisualReturn("left-primary-only-released");
            }
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
        // The left hand's finger pose is hFRIK's mirrored weapon-hand copy,
        // driven by the same ownership block.
        (void)solveLeftFiringWeaponCarry(weaponNode);
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

        /*
         * Carry-time aim diagnostic (~3s cadence): the barrel direction in
         * LEFT-wand coordinates during the live carry. Matching the
         * takeover's barrelInLeftWand proves the carry chain is faithful to
         * the committed hold (residual cant then lives in the left wand /
         * melee driver chain and the aim trim is the right knob); a drift
         * from the takeover value means the left bone-in-wand relationship
         * changed after the topology swap and the hold must be resampled.
         */
        if (++_leftFiringAimLogCounter >= 270) {
            _leftFiringAimLogCounter = 0;
            auto* playerNodes = f4vr::getPlayerNodes();
            if (playerNodes && playerNodes->SecondaryWandNode && isFiniteTransform(playerNodes->SecondaryWandNode->world)) {
                const RE::NiTransform weaponInLeftWandNow = transform_math::composeTransforms(
                    transform_math::invertTransform(playerNodes->SecondaryWandNode->world), weaponNode->world);
                const RE::NiPoint3 barrelNow = sub(
                    transform_math::localPointToWorld(weaponInLeftWandNow, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
                    weaponInLeftWandNow.translate);
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: left-firing carry aim barrelInLeftWand=({:.3f},{:.3f},{:.3f})",
                    barrelNow.x,
                    barrelNow.y,
                    barrelNow.z);
            }
        }
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
         * Reattach forces the CANONICAL per-hand hold instead of freezing the
         * live squeeze orientation: the LEFT hand takes the canonical
         * right-hand hold MIRRORED (authored offsets adapted to the left bone
         * basis), the RIGHT hand re-takes its canonical native hold directly.
         * A live squeeze capture both fired with the weapon crooked and, for
         * the right hand, poisoned the canonical itself through the snapshot
         * below. The live capture remains only as the no-canonical fallback.
         */
        bool usedCanonicalHold = false;
        const char* holdSource = "live-capture";
        if (handIsLeft) {
            RE::NiTransform mirroredHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(mirroredHandWeaponLocal)) {
                _primaryHandWeaponLocal = mirroredHandWeaponLocal;
                usedCanonicalHold = true;
                holdSource = "mirrored-canonical";
            }
        } else if (_hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalGenerationKey == _activeWeaponGenerationKey) {
            _primaryHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
            usedCanonicalHold = true;
            holdSource = "native-canonical";
        }
        if (!usedCanonicalHold) {
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
        if (!_firingHandIsLeft) {
            restoreFrikPrimaryWeaponPose();
        }
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = handIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: firing hand reattached at configured grip hand={} hold={}",
            handIsLeft ? "left" : "right",
            holdSource);
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
                    // Fresh two-hand configuration: the just-taken firing grip
                    // gets the same release-defer window as a fresh support grab.
                    _supportGripAgeFrames = 0;
                    _freshSupportGripDeferLogged = false;
                    updateFullWeaponAuthorityGrip(weaponNode, dt);
                } else {
                    if (!_firingHandIsLeft && ownsWeaponTransform()) {
                        beginWeaponVisualReturn("part-carry-reattached-primary-only");
                    }
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
                releasePartGrip(supportHandIsLeft, "support-grip-released", true);
                lastReleaseWasSupportHand = true;
            }
        }

        WeaponPartGrip& freeHandGrip = partGrip(firingHandIsLeft);
        if (freeHandGrip.active) {
            if (!providerPartAuthorityStillCurrent(freeHandGrip, currentWeaponGenerationKey)) {
                releasePartGrip(firingHandIsLeft, "provider-part-authority-lost");
                lastReleaseWasSupportHand = false;
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(frameInput.primaryGripInput.held, firingHandHoldingObject)) {
                releasePartGrip(firingHandIsLeft, "free-hand-grip-released", true);
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
            releasePartGrip(supportHandIsLeft, "carry-authority-lost", true);
            releasePartGrip(firingHandIsLeft, "carry-authority-lost", true);
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
        _hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
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

    bool TwoHandedGrip::applyWeaponVisualAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t authorityGenerationKey)
    {
        if (!weaponNode) {
            return false;
        }

        const std::uint64_t effectiveGenerationKey = authorityGenerationKey != 0 ? authorityGenerationKey : _activeWeaponGenerationKey;
        const bool scopeAnchorMatchesAuthority =
            _nativeScopeSightAnchorValid && _nativeScopeSightAnchorWeaponNode == weaponNode && _nativeScopeSightAnchorGenerationKey == effectiveGenerationKey;

        // Capture hFRIK's engine-specific camera axis only before changing the
        // weapon. Once captured, every hand mode resolves the same immutable
        // generation-bound weapon-local sight frame.
        const NativeScopeCameraFollowCapture scopeCameraFollow = captureNativeScopeCameraFollow(weaponNode);
        if (scopeAnchorMatchesAuthority && scopeCameraFollow.valid) {
            (void)captureNativeScopeRigidFrame(weaponNode, effectiveGenerationKey, scopeCameraFollow.camera, scopeCameraFollow.cameraWorldBefore);
            (void)captureNativeScopeOverlayCalibration(scopeCameraFollow.cameraWorldBefore, effectiveGenerationKey);
        }

        if (weaponNode->parent) {
            weaponNode->local = weapon_visual_authority_math::worldTargetToParentLocal(weaponNode->parent->world, solvedWeaponWorld);
            f4vr::updateTransformsDown(weaponNode, true);
        } else {
            weaponNode->local = solvedWeaponWorld;
            weaponNode->world = solvedWeaponWorld;
            f4vr::updateTransformsDown(weaponNode, false);
        }

        const bool rigidFrameMatchesAuthority = _nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey == effectiveGenerationKey &&
            _nativeScopeRigidFrame.weaponNodeIdentity == weaponNode && _nativeScopeRigidFrame.scopeCameraIdentity == scopeCameraFollow.camera;
        const NativeScopeCameraFollowResult scopeCameraResult = rigidFrameMatchesAuthority
            ? applyNativeScopeCameraWorldTarget(scopeCameraFollow,
                  native_scope_camera_follow_math::resolveRigidSightFrameWorld(weaponNode->world, _nativeScopeRigidFrame.cameraWeaponLocal))
            : applyNativeScopeCameraFollow(scopeCameraFollow, weaponNode->world, nullptr);
        if (scopeCameraResult.targetValid && scopeCameraResult.writeApplied) {
            (void)applyNativeScopeOverlayTarget(scopeCameraResult.targetCameraWorld, effectiveGenerationKey);
        }
        _lastRenderedWeaponWorld = weaponNode->world;
        _hasLastRenderedWeaponWorld = isFiniteTransform(_lastRenderedWeaponWorld);
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, effectiveGenerationKey,
                NativeScopeCameraWriteSource::WeaponVisualAuthority, scopeCameraFollow, scopeCameraResult, rigidFrameMatchesAuthority);
        }
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
        const auto& returningHand = _returningHandVisuals[_firingHandIsLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const RE::NiTransform appliedFiringHandWorld =
            resolveLockedHandVisualTarget(firingHandWorld, acquisitionStart, dt, _primaryHandVisualLerp);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            PRIMARY_GRIP_TAG, handFromBool(_firingHandIsLeft), appliedFiringHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied) {
            clearHandVisualReturn(_firingHandIsLeft, "firing-grip-authority-acquired", false);
            recordPublishedHandWorld(_firingHandIsLeft, appliedFiringHandWorld);
        }
        return applied;
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
        const auto& returningHand = _returningHandVisuals[isLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const RE::NiTransform appliedHandWorld =
            resolveLockedHandVisualTarget(partGripHandWorld, acquisitionStart, dt, grip.visualLerp);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            SUPPORT_GRIP_TAG, handFromBool(isLeft), appliedHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied) {
            clearHandVisualReturn(isLeft, "part-grip-authority-acquired", false);
            recordPublishedHandWorld(isLeft, appliedHandWorld);
        }
        return applied;
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
        _hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
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
        _rightFiringGripCanonicalWeaponLocal = _primaryGripLocal;
        _rightFiringHandCanonicalGenerationKey = _activeWeaponGenerationKey;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    void TwoHandedGrip::refreshRightNativeCanonicalFrame(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey)
    {
        /*
         * Passive canonical capture: whenever the equipped weapon rides the
         * native RIGHT hand (no ROCK transform ownership), the live weapon
         * pose already carries FRIK's authored per-weapon offsets, so the
         * canonical right hold and its weapon-in-wand frame can refresh
         * continuously. Without this, a weapon that was never
         * right-firing-gripped in the session had no canonical, and a LEFT
         * takeover fell back to the raw squeeze capture - the per-weapon
         * offsets (e.g. the UMP's large forward offset) silently missing
         * from the mirrored left hold ("worked before by coincidence").
         */
        if (isManualOwnershipActive() || _weaponNodeOwnershipBlockEngaged ||
            !scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(_scopeMenuOpenThisFrame, _scopeSafeHandFrames[1].rootRebaseActive) || !weaponNode ||
            currentWeaponGenerationKey == 0 || !isFiniteTransform(weaponNode->world)) {
            return;
        }
        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform rightHandWorld{};
        if (!playerNodes || !playerNodes->primaryWandNode ||
            !isFiniteTransform(playerNodes->primaryWandNode->world) ||
            !tryGetSolverHandTransform(false, rightHandWorld)) {
            return;
        }
        const RE::NiTransform boneInRightWand = transform_math::composeTransforms(
            transform_math::invertTransform(playerNodes->primaryWandNode->world), rightHandWorld);
        // Same wrist-range gate as the mirror's wand-map sampling, plus a
        // loose weapon-to-hand bound so a mid-equip/mid-teleport frame never
        // poisons the canonical.
        constexpr float kMaxBoneToWandDistance = 30.0f;
        constexpr float kMaxWeaponToHandDistance = 100.0f;
        const RE::NiPoint3 weaponToHand = sub(weaponNode->world.translate, rightHandWorld.translate);
        if (!isFiniteTransform(boneInRightWand) ||
            std::sqrt(dot(boneInRightWand.translate, boneInRightWand.translate)) > kMaxBoneToWandDistance ||
            std::sqrt(dot(weaponToHand, weaponToHand)) > kMaxWeaponToHandDistance) {
            return;
        }
        const RE::NiTransform canonicalHold = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), rightHandWorld);
        const RE::NiPoint3 canonicalGrip = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(rightHandWorld, false), weaponNode);
        if (!isFiniteTransform(canonicalHold) || !std::isfinite(canonicalGrip.x) || !std::isfinite(canonicalGrip.y) || !std::isfinite(canonicalGrip.z)) {
            return;
        }
        _rightFiringHandCanonicalWeaponLocal = canonicalHold;
        _rightFiringGripCanonicalWeaponLocal = canonicalGrip;
        _rightFiringHandCanonicalGenerationKey = currentWeaponGenerationKey;
        _hasRightFiringHandCanonicalWeaponLocal = true;
        // Native carry is the only state where the right bone is guaranteed
        // to ride the wand naturally; snapshot the relation for the mirror's
        // locked-right-hand substitution (see _rightNaturalBoneInWand).
        _rightNaturalBoneInWand = boneInRightWand;
        _hasRightNaturalBoneInWand = true;
    }

    bool TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal(RE::NiTransform& outHandWeaponLocal) const
    {
        if (!_hasRightFiringHandCanonicalWeaponLocal ||
            _rightFiringHandCanonicalGenerationKey == 0 ||
            _rightFiringHandCanonicalGenerationKey != _activeWeaponGenerationKey) {
            return false;
        }

        RE::NiTransform leftHandWorld{};
        if (!tryGetSolverHandTransform(true, leftHandWorld)) {
            return false;
        }

        /*
         * A part-gripping right hand is visually locked to the weapon part,
         * so its live bone no longer expresses the natural bone-in-wand
         * relation the wand conjugation depends on - a left reattach from a
         * right offhand carry came out at whatever angle the lock left the
         * bone. Replay the natural relation (snapshotted during native right
         * carry) onto the live right wand instead; every unlocked case keeps
         * the live sample the confirmed takeover path uses.
         */
        RE::NiTransform rightHandWorld{};
        if (partGrip(false).active && _hasRightNaturalBoneInWand) {
            auto* playerNodes = f4vr::getPlayerNodes();
            if (!playerNodes || !playerNodes->primaryWandNode || !isFiniteTransform(playerNodes->primaryWandNode->world)) {
                return false;
            }
            rightHandWorld = transform_math::composeTransforms(playerNodes->primaryWandNode->world, _rightNaturalBoneInWand);
        } else if (!tryGetSolverHandTransform(false, rightHandWorld)) {
            return false;
        }

        return tryBuildMirroredLeftFiringHandWeaponLocal(
            _rightFiringHandCanonicalWeaponLocal,
            _primaryGripLocal,
            rightHandWorld,
            leftHandWorld,
            outHandWeaponLocal,
            true);
    }

    bool TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
        const RE::NiTransform& canonicalRightHandWeaponLocal,
        const RE::NiPoint3& firingGripWeaponLocal,
        const RE::NiTransform& rightHandWorld,
        const RE::NiTransform& leftHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const bool logDiagnostic)
    {
        if (!isFiniteTransform(canonicalRightHandWeaponLocal) ||
            !std::isfinite(firingGripWeaponLocal.x) ||
            !std::isfinite(firingGripWeaponLocal.y) ||
            !std::isfinite(firingGripWeaponLocal.z)) {
            return false;
        }

        /*
         * WAND-CONJUGATION MIRROR. The aim requirement is controller-
         * relative: the tuned right-hand offsets align the barrel with the
         * RIGHT controller's forward, so the mirrored hold must align it
         * with the LEFT controller's forward with the lateral components
         * negated ("2 degrees left of the right wand" becomes "2 degrees
         * right of the left wand"). Left/right WAND device frames are the
         * physically exact mirror pair; the hand BONE conventions are not
         * mirrors (previous semantic-palm and bone-anchor mirrors both left
         * a residual yaw/side bias in-game). Conjugating the canonical hold
         * through the wand pair cancels every per-hand bone convention
         * inside the live-sampled bone-in-wand transforms:
         *
         *   weaponInLeftWand = Msag o weaponInRightWand o Mside
         *
         * with two reflections keeping the result a proper rotation: Msag
         * mirrors across the wand's sagittal plane (wand-local X lateral -
         * same axis family as the weapon frame the wand chain parents) and
         * Mside across the weapon's own side plane (+Y barrel, +X side),
         * which maps the grip from the weapon's right flank to its left.
         * Effect on the tuned offsets: yaw and roll negate, pitch and
         * fore/aft/vertical placement are preserved.
         *
         * The native first-person arm sync drags each hand bone to its wand
         * with a fixed per-hand map, so bone-in-wand is constant and
         * sampling it at takeover time is exact.
         */
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return false;
        }
        // Ambidextrous stands down in game-left-handed mode, so primary is
        // always the physical RIGHT wand here.
        RE::NiNode* rightWand = playerNodes->primaryWandNode;
        RE::NiNode* leftWand = playerNodes->SecondaryWandNode;
        if (!rightWand || !leftWand ||
            !isFiniteTransform(rightWand->world) || !isFiniteTransform(leftWand->world)) {
            return false;
        }

        const RE::NiTransform boneInRightWand =
            transform_math::composeTransforms(transform_math::invertTransform(rightWand->world), rightHandWorld);
        const RE::NiTransform boneInLeftWand =
            transform_math::composeTransforms(transform_math::invertTransform(leftWand->world), leftHandWorld);
        // A hand bone rides its wand at wrist range; a large offset means a
        // stale or foreign frame - fail closed to the live-capture fallback.
        constexpr float kMaxBoneToWandDistance = 30.0f;
        const auto transformOffsetLength = [](const RE::NiTransform& transform) {
            return std::sqrt(dot(transform.translate, transform.translate));
        };
        if (!isFiniteTransform(boneInRightWand) || !isFiniteTransform(boneInLeftWand) ||
            transformOffsetLength(boneInRightWand) > kMaxBoneToWandDistance ||
            transformOffsetLength(boneInLeftWand) > kMaxBoneToWandDistance) {
            return false;
        }

        // Reflections are involutions with symmetric matrices, so the
        // diagonal form is convention-proof; composed in pairs they keep
        // every final rotation proper.
        RE::NiTransform lateralMirror{};
        lateralMirror.MakeIdentity();
        lateralMirror.rotate.entry[0][0] = -1.0f;

        const RE::NiTransform weaponInRightWand = transform_math::composeTransforms(
            boneInRightWand, transform_math::invertTransform(canonicalRightHandWeaponLocal));
        RE::NiTransform weaponInLeftWand = transform_math::composeTransforms(
            lateralMirror, transform_math::composeTransforms(weaponInRightWand, lateralMirror));

        /*
         * Global left-hold trim, applied on the WAND side of the conjugation
         * (PRE-composed in the LEFT WAND frame), never on the weapon side.
         * The error it corrects is the fixed frame-convention delta between
         * the two wand device frames, which sits to the LEFT of the
         * conjugated hold. A weapon-side (post-composed) trim conjugates
         * through each weapon's own hold and therefore acts along different
         * axes per weapon: only the calibration weapon looked right, and
         * weapons with large authored holds (UMP forward offset, hunting
         * rifle) showed the trim rotated into unrelated directions
         * (2026-07-12 regression). Pre-composing makes one calibration exact
         * for every weapon, and if the mirror is fully correct these trims
         * converge to zero.
         *
         * Axes are the left wand's hand-anatomical basis (user-calibrated
         * in-game): X = palm normal, Y = fingers forward, Z = thumb up.
         * Yaw rotates about Z (thumb), pitch about X (palm normal); if a
         * value moves the aim opposite to its documented direction, the
         * user flips its sign once. The trim is ROTATION-ONLY: position is
         * anchored per weapon below, so a translation here would fight it.
         */
        constexpr float kDegreesToRadiansLocal = 0.017453292519943295769f;
        const float aimYawRadians = g_rockConfig.rockLeftFiringAimYawDegrees * kDegreesToRadiansLocal;
        const float aimPitchRadians = g_rockConfig.rockLeftFiringAimPitchDegrees * kDegreesToRadiansLocal;
        if (aimYawRadians != 0.0f || aimPitchRadians != 0.0f) {
            RE::NiTransform yawTrim{};
            yawTrim.MakeIdentity();
            if (aimYawRadians != 0.0f) {
                const float yawCos = std::cos(aimYawRadians);
                const float yawSin = std::sin(aimYawRadians);
                // yaw about wand +Z (thumb axis)
                yawTrim.rotate.entry[0][0] = yawCos;
                yawTrim.rotate.entry[0][1] = -yawSin;
                yawTrim.rotate.entry[1][0] = yawSin;
                yawTrim.rotate.entry[1][1] = yawCos;
            }
            RE::NiTransform pitchTrim{};
            pitchTrim.MakeIdentity();
            if (aimPitchRadians != 0.0f) {
                const float pitchCos = std::cos(aimPitchRadians);
                const float pitchSin = std::sin(aimPitchRadians);
                // pitch about wand +X (palm-normal axis)
                pitchTrim.rotate.entry[1][1] = pitchCos;
                pitchTrim.rotate.entry[1][2] = pitchSin;
                pitchTrim.rotate.entry[2][1] = -pitchSin;
                pitchTrim.rotate.entry[2][2] = pitchCos;
            }
            const RE::NiTransform wandTrim = transform_math::composeTransforms(yawTrim, pitchTrim);
            weaponInLeftWand = transform_math::composeTransforms(wandTrim, weaponInLeftWand);
        }

        const RE::NiTransform weaponInLeftHand = transform_math::composeTransforms(
            transform_math::invertTransform(boneInLeftWand), weaponInLeftWand);
        RE::NiTransform mirroredHandWeaponLocal = transform_math::invertTransform(weaponInLeftHand);

        if (!isFiniteTransform(mirroredHandWeaponLocal)) {
            return false;
        }

        /*
         * PALM-ANCHORED POSITION: the wand conjugation is the ORIENTATION
         * authority only. Deriving the translation through frame mirroring
         * left per-weapon height errors that no global knob can fix (UMP
         * too low while the P226 sits too high - weapons with authored
         * FRIK rotations/offsets each landed differently, because any
         * residual rotation-convention error displaces a hold by an amount
         * proportional to that weapon's own offsets). Instead the FIRING
         * GRIP POINT is pinned per weapon: it must sit at the same place in
         * the left palm as it does in the right palm. ROCK's hand bases
         * correspond anatomically with only Z flipped - empirical, from the
         * user-tuned palm pivots R(6.0,-2.0,+0.2) / L(6.0,-2.0,-0.2) - so
         * the target is simply (x, y, -z) of the grip's right-hand-local
         * position, plus the global offset knobs as palm-space nudges.
         * Per-weapon exact by construction; residuals are global-only.
         */
        const RE::NiPoint3 gripInRightHand = transform_math::localPointToWorld(
            transform_math::invertTransform(canonicalRightHandWeaponLocal), firingGripWeaponLocal);
        const RE::NiPoint3 gripTargetInLeftHand{
            gripInRightHand.x + g_rockConfig.rockLeftFiringAimOffsetXGameUnits,
            gripInRightHand.y + g_rockConfig.rockLeftFiringAimOffsetYGameUnits,
            -gripInRightHand.z + g_rockConfig.rockLeftFiringAimOffsetZGameUnits
        };
        if (std::isfinite(gripTargetInLeftHand.x) && std::isfinite(gripTargetInLeftHand.y) && std::isfinite(gripTargetInLeftHand.z)) {
            RE::NiTransform anchoredWeaponInLeftHand = transform_math::invertTransform(mirroredHandWeaponLocal);
            const RE::NiPoint3 gripRotatedOnly = sub(
                transform_math::localPointToWorld(anchoredWeaponInLeftHand, firingGripWeaponLocal),
                anchoredWeaponInLeftHand.translate);
            anchoredWeaponInLeftHand.translate = sub(gripTargetInLeftHand, gripRotatedOnly);
            const RE::NiTransform anchoredHold = transform_math::invertTransform(anchoredWeaponInLeftHand);
            if (isFiniteTransform(anchoredHold)) {
                mirroredHandWeaponLocal = anchoredHold;
            }
        }

        // Takeover-event diagnostic: barrel (+Y weapon) direction in each
        // wand frame. A correct mirror negates x and preserves y/z; a wand
        // axis-convention mismatch shows up here as a different component
        // flipping.
        if (logDiagnostic) {
            const RE::NiPoint3 barrelInRightWand =
                sub(transform_math::localPointToWorld(weaponInRightWand, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), weaponInRightWand.translate);
            const RE::NiPoint3 barrelInLeftWand =
                sub(transform_math::localPointToWorld(weaponInLeftWand, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), weaponInLeftWand.translate);
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: wand-conjugated left hold barrelInRightWand=({:.3f},{:.3f},{:.3f}) barrelInLeftWand=({:.3f},{:.3f},{:.3f}) boneWandDist=({:.2f},{:.2f})",
                barrelInRightWand.x,
                barrelInRightWand.y,
                barrelInRightWand.z,
                barrelInLeftWand.x,
                barrelInLeftWand.y,
                barrelInLeftWand.z,
                transformOffsetLength(boneInRightWand),
                transformOffsetLength(boneInLeftWand));
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
        if (_persistentEquippedCarryActive) {
            _persistentEquippedCarryDetachArmed = false;
        }
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
         * grip in place, forcing the CANONICAL per-hand hold: a LEFT takeover
         * applies the canonical right-hand hold mirrored (authored offsets
         * adapted to the left bone basis), a RIGHT takeover re-takes its
         * canonical native hold directly - the promoted hand's live bone is
         * still part-grip-locked here, so a live capture froze that locked
         * angle and (for the right) poisoned the canonical snapshot below.
         * The live capture remains only as the no-canonical fallback.
         */
        RE::NiTransform newFiringHandWeaponLocal{};
        bool usedCanonicalHold = false;
        const char* holdSource = "live-capture";
        if (supportHandIsLeft) {
            if (tryComputeMirroredLeftFiringHandWeaponLocal(newFiringHandWeaponLocal)) {
                usedCanonicalHold = true;
                holdSource = "mirrored-canonical";
            }
        } else if (_hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalGenerationKey == _activeWeaponGenerationKey) {
            newFiringHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
            usedCanonicalHold = true;
            holdSource = "native-canonical";
        }
        if (!usedCanonicalHold) {
            const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, supportHandIsLeft);
            const RE::NiTransform adjustedHandTransform =
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palm, firingGripWorld);
            newFiringHandWeaponLocal =
                transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        }

        beginHandVisualReturn(_firingHandIsLeft, "ambidextrous-firing-hand-promotion");
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
            holdSource);
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

#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>
#include <vector>

namespace rock
{
    namespace
    {
        constexpr const char* PRIMARY_GRIP_TAG = "ROCK_WeaponPrimaryGrip";
        constexpr const char* AUTHORED_PRIMARY_POSE_BLOCK_TAG = "ROCK_AuthoredPrimaryPose";
        constexpr const char* PRIMARY_DETACH_TAG = "ROCK_WeaponPrimaryDetach";
        constexpr const char* SUPPORT_GRIP_TAG = "ROCK_WeaponSupportGrip";
        constexpr const char* RETURN_HAND_TAG = "ROCK_WeaponReturn";
        constexpr const char* WEAPON_NODE_OWNERSHIP_TAG = "ROCK_LeftFiringCarry";
        constexpr const char* WEAPON_RECOIL_CONTROLLER_TAG = "ROCK_LeftFiringRecoil";
        constexpr int GRIP_HAND_POSE_PRIORITY = 100;
        constexpr int RETURN_HAND_VISUAL_PRIORITY = 85;
        constexpr float SUPPORT_NORMAL_TWIST_FACTOR = 0.5f;
        constexpr float DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS =
            0.5f * 0.01745329251994329577f;
        constexpr float RADIANS_TO_DEGREES = 57.295779513082320876f;
        constexpr std::uint32_t SCOPE_DRIVER_MISS_GRACE_FRAMES = 3;
        constexpr float SCOPE_ROOT_REBASE_DURATION_SECONDS = 0.075f;

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
         * either is missing (older FRIK build).
         */
        bool leftFiringInfrastructureAvailable()
        {
            return frik_visual_authority::canBlockPrimaryHandWeaponPose() &&
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

        bool arePointsNearlyEqual(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs, const float epsilon = 0.001f)
        {
            return std::abs(lhs.x - rhs.x) <= epsilon &&
                   std::abs(lhs.y - rhs.y) <= epsilon &&
                   std::abs(lhs.z - rhs.z) <= epsilon;
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

        struct AuthoredSupportPalmSeatProximity
        {
            RE::NiTransform authoredHandWorld{};
            RE::NiPoint3 authoredPalmSeatWeaponLocal{};
            RE::NiPoint3 authoredPalmSeatWorld{};
            RE::NiPoint3 liveTouchProbeWeaponLocal{};
            RE::NiPoint3 liveTouchProbeWorld{};
            float weaponRelativeDistanceGameUnits{ 0.0f };
            float worldReadbackDistanceGameUnits{ 0.0f };
            float frameAgreementErrorGameUnits{ 0.0f };
        };

        [[nodiscard]] bool resolveAuthoredSupportPalmSeatProximity(
            const RE::NiTransform& weaponWorld,
            const RE::NiTransform& liveHandWorld,
            const RE::NiTransform& authoredHandWeaponLocal,
            const bool isLeft,
            AuthoredSupportPalmSeatProximity& out)
        {
            out = {};
            if (!isFiniteTransform(weaponWorld) ||
                std::abs(weaponWorld.scale) <= 0.0001f ||
                !isUsableHandAuthorityTransform(liveHandWorld) ||
                !isFiniteTransform(authoredHandWeaponLocal) ||
                std::abs(authoredHandWeaponLocal.scale) <= 0.0001f) {
                return false;
            }

            out.authoredHandWorld = transform_math::composeTransforms(
                weaponWorld,
                authoredHandWeaponLocal);
            out.authoredPalmSeatWeaponLocal =
                computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    authoredHandWeaponLocal,
                    isLeft);
            out.authoredPalmSeatWorld = transform_math::localPointToWorld(
                weaponWorld,
                out.authoredPalmSeatWeaponLocal);
            out.liveTouchProbeWorld =
                computeGrabLegacyPalmPivotAWorldFromHandBasis(
                    liveHandWorld,
                    isLeft);
            out.liveTouchProbeWeaponLocal = transform_math::worldPointToLocal(
                weaponWorld,
                out.liveTouchProbeWorld);
            if (!isFiniteTransform(out.authoredHandWorld) ||
                !std::isfinite(out.authoredPalmSeatWeaponLocal.x) ||
                !std::isfinite(out.authoredPalmSeatWeaponLocal.y) ||
                !std::isfinite(out.authoredPalmSeatWeaponLocal.z) ||
                !std::isfinite(out.authoredPalmSeatWorld.x) ||
                !std::isfinite(out.authoredPalmSeatWorld.y) ||
                !std::isfinite(out.authoredPalmSeatWorld.z) ||
                !std::isfinite(out.liveTouchProbeWeaponLocal.x) ||
                !std::isfinite(out.liveTouchProbeWeaponLocal.y) ||
                !std::isfinite(out.liveTouchProbeWeaponLocal.z) ||
                !std::isfinite(out.liveTouchProbeWorld.x) ||
                !std::isfinite(out.liveTouchProbeWorld.y) ||
                !std::isfinite(out.liveTouchProbeWorld.z)) {
                return false;
            }

            // The touch probe and yellow authored palm seat are points, not
            // wrist bones. Compare them in one current Weapon frame so a ROCK
            // physical-left reparent or world translation cannot contaminate
            // acquisition.
            const float localDeltaX =
                out.liveTouchProbeWeaponLocal.x -
                out.authoredPalmSeatWeaponLocal.x;
            const float localDeltaY =
                out.liveTouchProbeWeaponLocal.y -
                out.authoredPalmSeatWeaponLocal.y;
            const float localDeltaZ =
                out.liveTouchProbeWeaponLocal.z -
                out.authoredPalmSeatWeaponLocal.z;
            const float weaponLocalDistance = std::sqrt(
                localDeltaX * localDeltaX +
                localDeltaY * localDeltaY +
                localDeltaZ * localDeltaZ);
            out.weaponRelativeDistanceGameUnits =
                weaponLocalDistance * std::abs(weaponWorld.scale);

            // Readback is diagnostic only. The Weapon-relative value above is
            // the acquisition gate.
            const float worldDeltaX =
                out.liveTouchProbeWorld.x - out.authoredPalmSeatWorld.x;
            const float worldDeltaY =
                out.liveTouchProbeWorld.y - out.authoredPalmSeatWorld.y;
            const float worldDeltaZ =
                out.liveTouchProbeWorld.z - out.authoredPalmSeatWorld.z;
            out.worldReadbackDistanceGameUnits = std::sqrt(
                worldDeltaX * worldDeltaX +
                worldDeltaY * worldDeltaY +
                worldDeltaZ * worldDeltaZ);
            out.frameAgreementErrorGameUnits = std::abs(
                out.weaponRelativeDistanceGameUnits -
                out.worldReadbackDistanceGameUnits);
            return std::isfinite(out.weaponRelativeDistanceGameUnits) &&
                   std::isfinite(out.worldReadbackDistanceGameUnits) &&
                   std::isfinite(out.frameAgreementErrorGameUnits);
        }

        struct RankedSupportGripTriangle
        {
            float distanceSquared = 0.0f;
            std::size_t sourceIndex = 0;
        };

        bool rankedSupportGripTriangleLess(const RankedSupportGripTriangle& lhs, const RankedSupportGripTriangle& rhs)
        {
            if (lhs.distanceSquared == rhs.distanceSquared) {
                return lhs.sourceIndex < rhs.sourceIndex;
            }
            return lhs.distanceSquared < rhs.distanceSquared;
        }

        struct TransformedSupportGripTriangleView
        {
            std::span<const TriangleData> localTriangles{};
            RE::NiTransform localToWorld{};

            [[nodiscard]] std::size_t size() const noexcept { return localTriangles.size(); }

            [[nodiscard]] TriangleData operator[](std::size_t index) const
            {
                const auto& triangle = localTriangles[index];
                return TriangleData{
                    transform_math::localPointToWorld(localToWorld, triangle.v0),
                    transform_math::localPointToWorld(localToWorld, triangle.v1),
                    transform_math::localPointToWorld(localToWorld, triangle.v2),
                };
            }
        };

        void selectNearestSupportGripFingerTriangles(
            std::span<const TriangleData> sourceTriangles,
            const RE::NiPoint3& gripPointLocal,
            std::size_t maxTriangles,
            std::vector<RankedSupportGripTriangle>& rankingScratch,
            std::vector<TriangleData>& outTriangles)
        {
            rankingScratch.clear();
            outTriangles.clear();
            const std::size_t boundedLimit = (std::min)(maxTriangles, grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles);
            if (boundedLimit == 0 || sourceTriangles.empty()) {
                return;
            }

            if (sourceTriangles.size() <= boundedLimit) {
                outTriangles.reserve(sourceTriangles.size());
                for (const auto& triangle : sourceTriangles) {
                    if (grab_finger_pose_runtime::isFinitePoint(triangle.v0) &&
                        grab_finger_pose_runtime::isFinitePoint(triangle.v1) &&
                        grab_finger_pose_runtime::isFinitePoint(triangle.v2)) {
                        outTriangles.push_back(triangle);
                    }
                }
                return;
            }

            rankingScratch.reserve(boundedLimit);
            for (std::size_t sourceIndex = 0; sourceIndex < sourceTriangles.size(); ++sourceIndex) {
                const auto& triangle = sourceTriangles[sourceIndex];
                if (!grab_finger_pose_runtime::isFinitePoint(triangle.v0) ||
                    !grab_finger_pose_runtime::isFinitePoint(triangle.v1) ||
                    !grab_finger_pose_runtime::isFinitePoint(triangle.v2)) {
                    continue;
                }

                float distanceSquared = 0.0f;
                (void)closestPointOnTriangleToPoint(gripPointLocal, triangle, distanceSquared);
                if (!std::isfinite(distanceSquared)) {
                    continue;
                }

                const RankedSupportGripTriangle candidate{ distanceSquared, sourceIndex };
                if (rankingScratch.size() < boundedLimit) {
                    rankingScratch.push_back(candidate);
                    std::push_heap(rankingScratch.begin(), rankingScratch.end(), rankedSupportGripTriangleLess);
                    continue;
                }

                if (rankedSupportGripTriangleLess(candidate, rankingScratch.front())) {
                    std::pop_heap(rankingScratch.begin(), rankingScratch.end(), rankedSupportGripTriangleLess);
                    rankingScratch.back() = candidate;
                    std::push_heap(rankingScratch.begin(), rankingScratch.end(), rankedSupportGripTriangleLess);
                }
            }

            std::sort(rankingScratch.begin(), rankingScratch.end(), rankedSupportGripTriangleLess);
            outTriangles.reserve(rankingScratch.size());
            for (const auto& ranked : rankingScratch) {
                outTriangles.push_back(sourceTriangles[ranked.sourceIndex]);
            }
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

        NativeScopeCameraDebugSnapshot makeNativeScopeCameraDebugSnapshot(const NativeScopeCameraDebugSnapshot& previous, const std::uint64_t weaponGenerationKey,
            const NativeScopeCameraWriteSource writeSource, const NativeScopeCameraFollowCapture& capture, const NativeScopeCameraFollowResult& result,
            const native_scope_sight_anchor_policy::AnchorSource anchorSource)
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
            snapshot.anchorSource = anchorSource;
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

    struct TwoHandedGrip::FingerPoseSolveScratch
    {
        struct HandScratch
        {
            std::vector<RankedSupportGripTriangle> ranking;
            std::vector<TriangleData> localTriangles;
            std::vector<TriangleData> worldTriangles;
            grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex spatialIndex;
        };

        std::array<HandScratch, 2> hands{};
    };

    TwoHandedGrip::TwoHandedGrip() :
        _fingerPoseSolveScratch(std::make_unique<FingerPoseSolveScratch>())
    {
        _recoilControllerRegistered =
            frik_visual_authority::registerWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG,
                &TwoHandedGrip::controlWeaponHandRecoil,
                this,
                GRIP_HAND_POSE_PRIORITY);
        if (!_recoilControllerRegistered) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: FRIK weapon-hand recoil controller registration failed; regular FRIK recoil remains active");
        }
    }

    TwoHandedGrip::~TwoHandedGrip()
    {
        if (_recoilControllerRegistered) {
            (void)frik_visual_authority::unregisterWeaponHandRecoilController(
                WEAPON_RECOIL_CONTROLLER_TAG);
            _recoilControllerRegistered = false;
        }
    }

    bool FRIK_CALL TwoHandedGrip::controlWeaponHandRecoil(
        const frik::api::FRIKApi::RecoilSample* const sample,
        frik::api::FRIKApi::RecoilResponse* const outResponse,
        void* const userData) noexcept
    {
        const auto* const self = static_cast<const TwoHandedGrip*>(userData);
        if (!self ||
            !sample ||
            sample->structSize < sizeof(frik::api::FRIKApi::RecoilSample) ||
            !outResponse) {
            return false;
        }

        if (!self->_weaponNodeOwnershipBlockEngaged ||
            !self->_firingHandIsLeft ||
            !self->isManualOwnershipActive()) {
            return false;
        }

        *outResponse = {};
        outResponse->structSize = sizeof(frik::api::FRIKApi::RecoilResponse);
        outResponse->handMask = static_cast<std::uint32_t>(
            frik::api::FRIKApi::RecoilHandMask::Primary);
        outResponse->delivery = frik::api::FRIKApi::RecoilDelivery::Direct;
        outResponse->controlledKickLocal = sample->nativeKickLocal;
        return true;
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

    bool TwoHandedGrip::rebuildNativeScopeRigidFrameTarget()
    {
        if (!_nativeScopeRigidFrame.valid ||
            !_nativeScopeAnchorValid ||
            _nativeScopeRigidFrame.weaponNodeIdentity !=
                _nativeScopeAnchorWeaponNode ||
            _nativeScopeRigidFrame.weaponGenerationKey !=
                _nativeScopeAnchorGenerationKey) {
            return false;
        }

        RE::NiTransform targetCameraWeaponLocal =
            _nativeScopeRigidFrame.nativeCameraWeaponLocal;
        targetCameraWeaponLocal.translate = _nativeScopeAnchorWeaponLocal;
        if (_nativeScopeAnchorSource ==
            native_scope_sight_anchor_policy::AnchorSource::
                FiringGripFallback) {
            targetCameraWeaponLocal =
                native_scope_camera_follow_math::
                    applyWeaponLocalRotationOffset(
                        targetCameraWeaponLocal,
                        _nativeScopeFallbackRotationDegrees.x,
                        _nativeScopeFallbackRotationDegrees.y,
                        _nativeScopeFallbackRotationDegrees.z);
        }
        if (!isFiniteTransform(targetCameraWeaponLocal) ||
            std::abs(targetCameraWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeRigidFrame.cameraWeaponLocal =
            targetCameraWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::captureNativeScopeRigidFrame(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey, RE::NiNode* scopeCamera,
        const RE::NiTransform& nativeCameraWorld)
    {
        if (_nativeScopeRigidFrame.valid && _nativeScopeRigidFrame.weaponGenerationKey == currentWeaponGenerationKey && _nativeScopeRigidFrame.weaponNodeIdentity == weaponNode &&
            _nativeScopeRigidFrame.scopeCameraIdentity == scopeCamera) {
            return true;
        }

        clearNativeScopeRigidFrame();
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_nativeScopeAnchorValid || _nativeScopeAnchorWeaponNode != weaponNode ||
            _nativeScopeAnchorGenerationKey != currentWeaponGenerationKey || !scopeCamera || !isFiniteTransform(weaponNode->world) || !isFiniteTransform(nativeCameraWorld)) {
            return false;
        }

        const RE::NiTransform nativeCameraWeaponLocal =
            native_scope_camera_follow_math::captureRigidAnchorFrameWeaponLocal(weaponNode->world, nativeCameraWorld, _nativeScopeAnchorWeaponLocal);
        if (!isFiniteTransform(nativeCameraWeaponLocal) ||
            std::abs(nativeCameraWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        _nativeScopeRigidFrame = NativeScopeRigidFrameState{
            .weaponGenerationKey = currentWeaponGenerationKey,
            .weaponNodeIdentity = weaponNode,
            .scopeCameraIdentity = scopeCamera,
            .nativeCameraWeaponLocal = nativeCameraWeaponLocal,
            .cameraWeaponLocal = nativeCameraWeaponLocal,
            .valid = true,
        };
        if (!rebuildNativeScopeRigidFrameTarget()) {
            clearNativeScopeRigidFrame();
            return false;
        }
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: native scope rigid frame captured generation={:016X} cameraLocal=({:.2f},{:.2f},{:.2f}) scale={:.3f}", currentWeaponGenerationKey,
            _nativeScopeRigidFrame.cameraWeaponLocal.translate.x, _nativeScopeRigidFrame.cameraWeaponLocal.translate.y,
            _nativeScopeRigidFrame.cameraWeaponLocal.translate.z, _nativeScopeRigidFrame.cameraWeaponLocal.scale);
        return true;
    }

    void TwoHandedGrip::synchronizeNativeScopePresentationAfterFrikUpdate(RE::NiNode* weaponNode, const std::uint64_t currentWeaponGenerationKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || !_nativeScopeAnchorValid || _nativeScopeAnchorWeaponNode != weaponNode ||
            _nativeScopeAnchorGenerationKey != currentWeaponGenerationKey) {
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
        const RE::NiTransform targetCameraWorld = native_scope_camera_follow_math::resolveRigidAnchorFrameWorld(weaponNode->world, _nativeScopeRigidFrame.cameraWeaponLocal);
        const NativeScopeCameraFollowResult result = applyNativeScopeCameraWorldTarget(capture, targetCameraWorld);
        if (overlayCalibrationReady && result.targetValid && result.writeApplied) {
            (void)applyNativeScopeOverlayTarget(result.targetCameraWorld, currentWeaponGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, currentWeaponGenerationKey,
                NativeScopeCameraWriteSource::PostFrikPresentationSync, capture, result, _nativeScopeAnchorSource);
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
        if (!weaponNode || !hmdNode || currentWeaponGenerationKey == 0 || !_nativeScopeAnchorValid || _nativeScopeAnchorWeaponNode != weaponNode ||
            _nativeScopeAnchorGenerationKey != currentWeaponGenerationKey || !isFiniteTransform(weaponNode->world) || !isFiniteTransform(hmdNode->world)) {
            return false;
        }

        const native_scope_activation_geometry::ConeSample sample =
            native_scope_activation_geometry::sample(weaponNode->world, _nativeScopeAnchorWeaponLocal, hmdNode->world, hmdSampleOffsetLocal, thresholds);
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
        _nativeScopeActivationDebugSnapshot = NativeScopeActivationDebugSnapshot{
            .evaluationSequence = _nativeScopeActivationDebugSnapshot.evaluationSequence + 1,
            .weaponGenerationKey = currentWeaponGenerationKey,
            .anchorSource = _nativeScopeAnchorSource,
            .nativeGeometryDecision = nativeGeometryDecision,
            .rockGeometryDecision = outRockGeometryDecision,
            .nativeScopeAlreadyActive = nativeScopeAlreadyActive,
            .sample = sample,
            .thresholds = thresholds,
        };
        return true;
    }

    void TwoHandedGrip::refreshNativeScopeAnchor(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        std::uint32_t currentEquippedWeaponFormID,
        const WeaponCollision& weaponCollision)
    {
        const bool forceFiringGripFallback =
            g_rockConfig.rockNativeScopeForceFiringGripFallback;
        const RE::NiPoint3 fallbackOffsetWeaponLocal{
            g_rockConfig.rockNativeScopeFiringGripFallbackOffsetXGameUnits,
            g_rockConfig.rockNativeScopeFiringGripFallbackOffsetYGameUnits,
            g_rockConfig.rockNativeScopeFiringGripFallbackOffsetZGameUnits,
        };
        const RE::NiPoint3 fallbackRotationDegrees{
            g_rockConfig.rockNativeScopeFiringGripFallbackPitchDegrees,
            g_rockConfig.rockNativeScopeFiringGripFallbackYawDegrees,
            g_rockConfig.rockNativeScopeFiringGripFallbackRollDegrees,
        };

        RE::NiPoint3 firingGripWeaponLocal{};
        bool firingGripFromCanonical = false;
        bool firingGripValid = hasRightFiringHandCanonicalFrame(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);
        if (firingGripValid) {
            firingGripWeaponLocal = _rightFiringGripCanonicalWeaponLocal;
            firingGripFromCanonical = true;
        } else if (isManualOwnershipActive() &&
                   _activeWeaponNode == weaponNode &&
                   _activeWeaponGenerationKey == currentWeaponGenerationKey &&
                   _activeEquippedWeaponOwnershipKey ==
                       currentEquippedWeaponOwnershipKey &&
                   _primaryGripConfidence > 0.0f &&
                   native_scope_sight_anchor_policy::isFinitePoint(
                       _primaryGripLocal)) {
            firingGripWeaponLocal = _primaryGripLocal;
            firingGripValid = true;
        }

        const bool sameIdentity =
            _nativeScopeAnchorWeaponNode == weaponNode &&
            _nativeScopeAnchorGenerationKey == currentWeaponGenerationKey &&
            _nativeScopeAnchorOwnershipKey ==
                currentEquippedWeaponOwnershipKey &&
            _nativeScopeAnchorWeaponFormID == currentEquippedWeaponFormID;
        if (sameIdentity &&
            _nativeScopeAnchorSource ==
                native_scope_sight_anchor_policy::AnchorSource::GeneratedSight &&
            !forceFiringGripFallback) {
            return;
        }

        const bool identityChanged = !sameIdentity;
        if (identityChanged) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            _nativeScopeExitDebounceGenerationKey = 0;
            _nativeScopeExitOutsideFrames = 0;

            _nativeScopeAnchorWeaponNode = weaponNode;
            _nativeScopeAnchorGenerationKey = currentWeaponGenerationKey;
            _nativeScopeAnchorOwnershipKey =
                currentEquippedWeaponOwnershipKey;
            _nativeScopeAnchorWeaponFormID = currentEquippedWeaponFormID;
            _nativeScopeAnchorWeaponLocal = {};
            _nativeScopeAnchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _nativeScopeAnchorValid = false;
            _nativeScopeFallbackRotationDegrees = fallbackRotationDegrees;
        }
        if (!weaponNode || currentWeaponGenerationKey == 0 ||
            currentEquippedWeaponOwnershipKey == 0 ||
            currentEquippedWeaponFormID == 0) {
            return;
        }

        WeaponCollision::NativeScopeSightAnchorSnapshot snapshot{};
        if (!forceFiringGripFallback) {
            snapshot = weaponCollision.getNativeScopeSightAnchorSnapshot();
            const native_scope_sight_anchor_policy::PublicationIdentity
                publishedIdentity{
                    .weaponGenerationKey = snapshot.weaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        snapshot.equippedWeaponOwnershipKey,
                    .weaponFormID = snapshot.weaponFormID,
                };
            const native_scope_sight_anchor_policy::PublicationIdentity
                currentIdentity{
                    .weaponGenerationKey = currentWeaponGenerationKey,
                    .equippedWeaponOwnershipKey =
                        currentEquippedWeaponOwnershipKey,
                    .weaponFormID = currentEquippedWeaponFormID,
                };
            if (snapshot.weaponGenerationKey !=
                    currentWeaponGenerationKey ||
                !native_scope_sight_anchor_policy::
                    matchesCurrentEquippedWeapon(
                        publishedIdentity,
                        currentIdentity)) {
                // A racing publication is never allowed to select either its
                // geometry or a sticky fallback. Clear the identity so the
                // next frame retries against one coherent body-set snapshot.
                _nativeScopeAnchorWeaponNode = nullptr;
                _nativeScopeAnchorGenerationKey = 0;
                _nativeScopeAnchorOwnershipKey = 0;
                _nativeScopeAnchorWeaponFormID = 0;
                _nativeScopeAnchorWeaponLocal = {};
                _nativeScopeAnchorSource =
                    native_scope_sight_anchor_policy::AnchorSource::None;
                _nativeScopeAnchorValid = false;
                _nativeScopeFallbackRotationDegrees = {};
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
                _nativeScopeExitDebounceGenerationKey = 0;
                _nativeScopeExitOutsideFrames = 0;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: rejected stale native scope sight anchor generation={:016X}->{:016X} ownership={:016X}->{:016X} form={:08X}->{:08X}",
                    snapshot.weaponGenerationKey,
                    currentWeaponGenerationKey,
                    snapshot.equippedWeaponOwnershipKey,
                    currentEquippedWeaponOwnershipKey,
                    snapshot.weaponFormID,
                    currentEquippedWeaponFormID);
                return;
            }
        }

        const auto resolved = native_scope_sight_anchor_policy::resolve(
            forceFiringGripFallback,
            snapshot.valid,
            snapshot.anchorWeaponLocal,
            firingGripValid,
            firingGripWeaponLocal,
            fallbackOffsetWeaponLocal);
        if (!resolved.valid) {
            if (_nativeScopeAnchorValid) {
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
                _nativeScopeExitDebounceGenerationKey = 0;
                _nativeScopeExitOutsideFrames = 0;
            }
            _nativeScopeAnchorWeaponLocal = {};
            _nativeScopeAnchorSource =
                native_scope_sight_anchor_policy::AnchorSource::None;
            _nativeScopeAnchorValid = false;
            _nativeScopeFallbackRotationDegrees =
                fallbackRotationDegrees;
            ROCK_LOG_SAMPLE_DEBUG(Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: native scope anchor unavailable generation={:016X} generatedSight={} firingGrip={} forced={}; leaving native camera untouched",
                currentWeaponGenerationKey,
                snapshot.valid,
                firingGripValid,
                forceFiringGripFallback);
            return;
        }

        const bool sameResolvedAnchor =
            !identityChanged &&
            _nativeScopeAnchorValid &&
            _nativeScopeAnchorSource == resolved.source &&
            arePointsNearlyEqual(
                _nativeScopeAnchorWeaponLocal,
                resolved.weaponLocal);
        const bool fallbackRotationChanged =
            resolved.source ==
                native_scope_sight_anchor_policy::AnchorSource::
                    FiringGripFallback &&
            !arePointsNearlyEqual(
                _nativeScopeFallbackRotationDegrees,
                fallbackRotationDegrees);
        if (sameResolvedAnchor && !fallbackRotationChanged) {
            return;
        }

        if (sameResolvedAnchor) {
            _nativeScopeFallbackRotationDegrees =
                fallbackRotationDegrees;
            if (_nativeScopeRigidFrame.valid &&
                !rebuildNativeScopeRigidFrameTarget()) {
                clearNativeScopeOverlayAuthority(true);
                clearNativeScopeRigidFrame();
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: rejected invalid native scope fallback rotation pitch={:.2f} yaw={:.2f} roll={:.2f}",
                    fallbackRotationDegrees.x,
                    fallbackRotationDegrees.y,
                    fallbackRotationDegrees.z);
                return;
            }
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: native scope firing-grip fallback rotation refreshed generation={:016X} pitch={:.2f} yaw={:.2f} roll={:.2f}",
                currentWeaponGenerationKey,
                fallbackRotationDegrees.x,
                fallbackRotationDegrees.y,
                fallbackRotationDegrees.z);
            return;
        }

        if (!identityChanged) {
            // Keep the immutable native camera/model calibration for this
            // equipped instance. Rebuild only the derived target so live
            // position/rotation tuning cannot compound ROCK's previous write.
            _nativeScopeExitDebounceGenerationKey = 0;
            _nativeScopeExitOutsideFrames = 0;
        }

        _nativeScopeAnchorWeaponLocal = resolved.weaponLocal;
        _nativeScopeAnchorSource = resolved.source;
        _nativeScopeAnchorValid = true;
        _nativeScopeFallbackRotationDegrees =
            fallbackRotationDegrees;
        if (_nativeScopeRigidFrame.valid &&
            !rebuildNativeScopeRigidFrameTarget()) {
            clearNativeScopeOverlayAuthority(true);
            clearNativeScopeRigidFrame();
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: native scope target rebuild failed generation={:016X} source={}",
                currentWeaponGenerationKey,
                resolved.source ==
                        native_scope_sight_anchor_policy::AnchorSource::
                            FiringGripFallback ?
                    "firing-grip-fallback" :
                    "generated-sight");
        }
        if (resolved.source ==
            native_scope_sight_anchor_policy::AnchorSource::GeneratedSight) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: native scope anchor generation={:016X} source=generated-sight bodies={} local=({:.2f},{:.2f},{:.2f}) boundsMin=({:.2f},{:.2f},{:.2f}) boundsMax=({:.2f},{:.2f},{:.2f}) policy=rear-center",
                currentWeaponGenerationKey,
                snapshot.sightBodyCount,
                snapshot.anchorWeaponLocal.x,
                snapshot.anchorWeaponLocal.y,
                snapshot.anchorWeaponLocal.z,
                snapshot.sightBoundsMinWeaponLocal.x,
                snapshot.sightBoundsMinWeaponLocal.y,
                snapshot.sightBoundsMinWeaponLocal.z,
                snapshot.sightBoundsMaxWeaponLocal.x,
                snapshot.sightBoundsMaxWeaponLocal.y,
                snapshot.sightBoundsMaxWeaponLocal.z);
        } else {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: native scope anchor generation={:016X} source=firing-grip-fallback origin={} gripLocal=({:.2f},{:.2f},{:.2f}) offset=({:.2f},{:.2f},{:.2f}) rotation=({:.2f},{:.2f},{:.2f}) anchorLocal=({:.2f},{:.2f},{:.2f}) forced={}",
                currentWeaponGenerationKey,
                firingGripFromCanonical ? "canonical" : "active-grip",
                firingGripWeaponLocal.x,
                firingGripWeaponLocal.y,
                firingGripWeaponLocal.z,
                fallbackOffsetWeaponLocal.x,
                fallbackOffsetWeaponLocal.y,
                fallbackOffsetWeaponLocal.z,
                fallbackRotationDegrees.x,
                fallbackRotationDegrees.y,
                fallbackRotationDegrees.z,
                _nativeScopeAnchorWeaponLocal.x,
                _nativeScopeAnchorWeaponLocal.y,
                _nativeScopeAnchorWeaponLocal.z,
                forceFiringGripFallback);
        }
    }

    void TwoHandedGrip::refreshScopeSafeHandFrames(const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        const bool scopeStateChanged = _scopeMenuOpenThisFrame != frameInput.scopeMenuOpen;
        _scopeMenuOpenThisFrame = frameInput.scopeMenuOpen;
        _scopeMenuClosedThisFrame = scopeStateChanged && !_scopeMenuOpenThisFrame;
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
        const EquippedWeaponHandlingSettings& handlingSettings)
    {
        _handlingSettings = handlingSettings;
        setGrabbedObjectHandPoseOwnership(
            frameInput.leftHandHoldingObject,
            frameInput.rightHandHoldingObject);
        _hasSolvedWeaponTransform = false;
        _scopeHandAuthorityPublishedThisFrame = {};
        _firingGripReattachHoverInsideRadius = false;
        _firingGripReattachHoverHandIsLeft = _firingHandIsLeft;
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            _nativeScopeCameraDebugSnapshot.framesSinceApply != (std::numeric_limits<std::uint32_t>::max)()) {
            ++_nativeScopeCameraDebugSnapshot.framesSinceApply;
        }

        refreshNativeScopeAnchor(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            weaponCollision.getCurrentObservedEquippedWeaponFormID(),
            weaponCollision);
        refreshScopeSafeHandFrames(frameInput, dt);

        if (!runtime_state::isLocalSkeletonReady() || !weaponNode) {
            clearAllVisualReturns("skeleton-or-weapon-unavailable", true, true);
            if (_state != TwoHandedState::Inactive) {
                transitionToInactive(false);
            }
            reconcileDeferredScopeHandAuthority(weaponNode);
            return;
        }

        refreshNaturalHandInWandFrames();
        refreshAuthoredSupportRightMirror();

        updateWeaponVisualReturn(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            dt);

        EquippedWeaponGripFrameInput stableFrameInput = frameInput;
        if (_persistentEquippedCarryActive && isManualOwnershipActive()) {
            /*
             * A persistent fixed/selected-hand carry has no grab press to
             * retain PrimaryOnly. Keep
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
            reconcileDeferredScopeHandAuthority(weaponNode);
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
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: support release predicate firingHand={} supportHand={} gripHeld={} holdingObject={} scopeMenu={}",
                    _firingHandIsLeft ? "left" : "right",
                    supportHandIsLeft ? "left" : "right",
                    supportGripHeld ? "yes" : "no",
                    supportHandHoldingObject ? "yes" : "no",
                    _scopeMenuOpenThisFrame ? "open" : "closed");
                const auto releaseAction = weapon_two_handed_grip_math::resolveSupportReleaseManualAction(
                    weapon_two_handed_grip_math::SupportReleaseOwnershipInput{
                        .firingGripOwnershipEnabled = handlingSettings.firingGripOwnershipEnabled,
                        .primaryDetachEnabled = handlingSettings.primaryDetachEnabled,
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
            } else if ((handlingSettings.primaryDetachEnabled || handlingSettings.ambidextrousHandoffEnabled) && !primaryGripInput.held &&
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
            } else if (handlingSettings.ambidextrousHandoffEnabled && !primaryGripInput.held && tryPromoteSupportGripToFiringGrip(_activeWeaponNode)) {
                // The support hand was wrapped over the firing grip when the
                // firing hand opened: it takes over the SAME weapon-relative
                // grip in place (seamless hand switch, pistol shooting-cup
                // flow). State is PrimaryOnly under the new firing hand.
            } else if (handlingSettings.primaryDetachEnabled &&
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
            } else if (!handlingSettings.primaryDetachEnabled) {
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
            } else if (!handlingSettings.firingGripOwnershipEnabled) {
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
                    handlingSettings.primaryDetachEnabled);
            }
            break;
        }

        refreshRightNativeCanonicalFrame(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);

        // Enforce the left-firing weapon-node ownership contract after every
        // state/role transition this frame (idempotent; also the parent
        // watchdog for engine-side re-attach).
        syncFiringHandWeaponNodeOwnership(weaponNode);
        updateHandVisualReturns(dt);
        // State transitions and their replacement publications must finish
        // before stale scoped roles are removed. This keeps hFRIK under one
        // continuous ROCK authority selection across scope and role edges.
        reconcileDeferredScopeHandAuthority(weaponNode);
    }

    void TwoHandedGrip::reset()
    {
        clearDynamicSupportAcquisition("reset", true);
        clearAuthoredSupportGripCandidate();
        clearAllVisualReturns("reset", false, true);
        clearNativeScopeOverlayAuthority(true);
        _equippedWeaponDropRequest = {};
        _hapticEvents = {};
        _firingGripReattachHoverInsideRadius = false;
        _nativeScopeAnchorWeaponNode = nullptr;
        _nativeScopeAnchorGenerationKey = 0;
        _nativeScopeAnchorOwnershipKey = 0;
        _nativeScopeAnchorWeaponFormID = 0;
        _nativeScopeAnchorWeaponLocal = {};
        _nativeScopeAnchorSource =
            native_scope_sight_anchor_policy::AnchorSource::None;
        _nativeScopeAnchorValid = false;
        _nativeScopeFallbackRotationDegrees = {};
        _nativeScopeExitDebounceGenerationKey = 0;
        _nativeScopeExitOutsideFrames = 0;
        _nativeScopeCameraDebugSnapshot = {};
        _nativeScopeActivationDebugSnapshot = {};
        clearNativeScopeRigidFrame();
        _scopeSafeHandFrames = {};
        _scopeDriverFrameAuthorityActive = false;
        _scopeHandAuthorityPublishedThisFrame = {};
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        clearSupportGripPose(true);
        clearSupportGripPose(false);
        restoreFrikPrimaryWeaponPose();
        clearRightFiringHandCanonicalFrame();
        _rightNaturalBoneInWand = {};
        _leftNaturalBoneInWand = {};
        _hasRightNaturalBoneInWand = false;
        _hasLeftNaturalBoneInWand = false;
        _authoredPrimaryFingerPoseSuppressed = false;
        _leftHandHoldingObjectForPose = false;
        _rightHandHoldingObjectForPose = false;
        if (_state != TwoHandedState::Inactive) {
            transitionToInactive(false);
            _scopeMenuOpenThisFrame = false;
            _scopeMenuClosedThisFrame = false;
            return;
        }
        _scopeMenuOpenThisFrame = false;
        _scopeMenuClosedThisFrame = false;
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

    bool TwoHandedGrip::blocksAuthoredPrimaryGripWeaponAlignment() const
    {
        /*
         * Right-firing PrimaryOnly is lifecycle/input ownership only: hFRIK
         * still publishes the native Weapon transform every frame. Treating
         * that state as a competing transform owner made the authored
         * calibration disappear immediately after a support-hand return.
         * Left-firing carry always remains ROCK-owned even in PrimaryOnly or
         * visual-only support mode, and the topology blocker is included as a
         * fail-closed witness if state and bridge cleanup ever diverge.
         */
        return _firingHandIsLeft ||
               _weaponNodeOwnershipBlockEngaged ||
               ownsWeaponTransform();
    }

    bool TwoHandedGrip::isWeaponVisualReturnActive() const
    {
        return _returningWeaponVisual.localTransition.active;
    }

    bool TwoHandedGrip::getSolvedWeaponTransform(RE::NiTransform& outTransform) const
    {
        if (!_hasSolvedWeaponTransform) {
            return false;
        }
        outTransform = _lastSolvedWeaponTransform;
        return true;
    }

    bool TwoHandedGrip::getManualCycleRockGripBaselines(
        RE::NiTransform& outRightHandInWeapon,
        RE::NiTransform& outLeftHandInWeapon) const
    {
        outRightHandInWeapon = {};
        outLeftHandInWeapon = {};

        const WeaponPartGrip& supportGrip = partGrip(true);
        if (_state != TwoHandedState::Gripping ||
            _firingHandIsLeft ||
            !ownsWeaponTransform() ||
            !_hasSolvedWeaponTransform ||
            !_activeWeaponNode ||
            !_hasFiringHandWeaponLocal ||
            !supportGrip.active ||
            !supportGrip.hasHandWeaponLocal ||
            !isFiniteTransform(_lastSolvedWeaponTransform) ||
            !isFiniteTransform(_primaryHandWeaponLocal)) {
            return false;
        }

        const RE::NiTransform supportHandWorld =
            resolvePartGripHandWorld(supportGrip, _activeWeaponNode);
        if (!isFiniteTransform(supportHandWorld)) {
            return false;
        }

        outRightHandInWeapon = _primaryHandWeaponLocal;
        outLeftHandInWeapon = transform_math::composeTransforms(
            transform_math::invertTransform(_lastSolvedWeaponTransform),
            supportHandWorld);
        if (!isFiniteTransform(outRightHandInWeapon) ||
            !isFiniteTransform(outLeftHandInWeapon)) {
            outRightHandInWeapon = {};
            outLeftHandInWeapon = {};
            return false;
        }
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

    bool TwoHandedGrip::getAuthoredSupportGripDebugSnapshot(
        AuthoredSupportGripDebugSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        const auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid ||
            !candidate.weaponNode ||
            candidate.weaponGenerationKey == 0 ||
            candidate.captureSequence == 0 ||
            !isFiniteTransform(candidate.weaponNode->world)) {
            return false;
        }

        const bool supportHandIsLeft = !_firingHandIsLeft;
        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask = 0;
        if (!tryResolveAuthoredSupportGripCandidateForHand(
                supportHandIsLeft,
                candidate.weaponNode,
                candidate.weaponGenerationKey,
                authoredSupportHandWeaponLocal,
                authoredSupportFingerLocalTransforms,
                authoredSupportFingerLocalTransformMask)) {
            return false;
        }

        RE::NiTransform liveSupportHandWorld{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, liveSupportHandWorld)) {
            return false;
        }

        AuthoredSupportPalmSeatProximity proximity{};
        if (!resolveAuthoredSupportPalmSeatProximity(
                candidate.weaponNode->world,
                liveSupportHandWorld,
                authoredSupportHandWeaponLocal,
                supportHandIsLeft,
                proximity)) {
            return false;
        }

        outSnapshot.weaponWorld = candidate.weaponNode->world;
        outSnapshot.authoredPalmSeatWeaponLocal =
            proximity.authoredPalmSeatWeaponLocal;
        outSnapshot.authoredPalmSeatWorld =
            proximity.authoredPalmSeatWorld;
        outSnapshot.liveTouchProbeWeaponLocal =
            proximity.liveTouchProbeWeaponLocal;
        outSnapshot.liveTouchProbeWorld =
            proximity.liveTouchProbeWorld;
        outSnapshot.weaponRelativeDistanceGameUnits =
            proximity.weaponRelativeDistanceGameUnits;
        outSnapshot.worldReadbackDistanceGameUnits =
            proximity.worldReadbackDistanceGameUnits;
        outSnapshot.frameAgreementErrorGameUnits =
            proximity.frameAgreementErrorGameUnits;
        outSnapshot.touchRadiusGameUnits =
            g_rockConfig.rockWeaponInteractionTouchRadius;
        outSnapshot.weaponGenerationKey = candidate.weaponGenerationKey;
        outSnapshot.captureSequence = candidate.captureSequence;
        outSnapshot.supportHandIsLeft = supportHandIsLeft;
        outSnapshot.mirroredForRightSupport = !supportHandIsLeft;
        outSnapshot.insideTouchRadius =
            proximity.weaponRelativeDistanceGameUnits <=
            outSnapshot.touchRadiusGameUnits;
        return true;
    }

    void TwoHandedGrip::resetLockedHandVisualLerp()
    {
        _primaryHandVisualLerp = {};
        partGrip(true).visualLerp = {};
        partGrip(false).visualLerp = {};
    }

    void TwoHandedGrip::beginDynamicSupportAcquisition(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip,
        const RE::NiTransform& primaryStartWorld,
        const RE::NiTransform& supportStartWorld)
    {
        if (_dynamicSupportAcquisition.active) {
            clearDynamicSupportAcquisition(
                "replaced-by-new-dynamic-support-grip",
                true);
        }

        _dynamicSupportAcquisition = {};
        _dynamicSupportAcquisition.active = true;
        _dynamicSupportAcquisition.supportHandIsLeft = supportHandIsLeft;
        _dynamicSupportAcquisition.weaponGenerationKey =
            supportGrip.weaponGenerationKey;
        _dynamicSupportAcquisition.gripSequence = supportGrip.gripSequence;
        _dynamicSupportAcquisition.primaryStartWorld = primaryStartWorld;
        _dynamicSupportAcquisition.supportStartWorld = supportStartWorld;
    }

    void TwoHandedGrip::clearDynamicSupportAcquisition(
        const char* reason,
        const bool logCancellation)
    {
        if (!_dynamicSupportAcquisition.active) {
            _dynamicSupportAcquisition = {};
            return;
        }

        if (logCancellation) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: dynamic support acquisition event=cancel reason={} hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s rawAlpha={:.3f} easedAlpha={:.3f} fullCorrection={:.2f}deg appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                reason ? reason : "unknown",
                _dynamicSupportAcquisition.supportHandIsLeft ?
                    "left" :
                    "right",
                _dynamicSupportAcquisition.gripSequence,
                _dynamicSupportAcquisition.weaponGenerationKey,
                _dynamicSupportAcquisition.elapsedSeconds,
                _dynamicSupportAcquisition.durationSeconds,
                _dynamicSupportAcquisition.rawAlpha,
                _dynamicSupportAcquisition.easedAlpha,
                _dynamicSupportAcquisition.fullCorrectionRadians *
                    RADIANS_TO_DEGREES,
                _dynamicSupportAcquisition.lastAppliedRotationRadians *
                    RADIANS_TO_DEGREES,
                _dynamicSupportAcquisition.lastPrimaryPivotError,
                _dynamicSupportAcquisition.lastSupportTargetError);
        }
        _dynamicSupportAcquisition = {};
    }

    bool TwoHandedGrip::dynamicSupportAcquisitionMatches(
        const bool supportHandIsLeft,
        const WeaponPartGrip& supportGrip) const
    {
        return _dynamicSupportAcquisition.active &&
               _dynamicSupportAcquisition.supportHandIsLeft ==
                   supportHandIsLeft &&
               _dynamicSupportAcquisition.weaponGenerationKey != 0 &&
               _dynamicSupportAcquisition.weaponGenerationKey ==
                   _activeWeaponGenerationKey &&
               _dynamicSupportAcquisition.weaponGenerationKey ==
                   supportGrip.weaponGenerationKey &&
               _dynamicSupportAcquisition.gripSequence != 0 &&
               _dynamicSupportAcquisition.gripSequence ==
                   supportGrip.gripSequence;
    }

    RE::NiTransform
    TwoHandedGrip::resolveDynamicSupportAcquisitionHandTarget(
        const RE::NiTransform& targetWorld,
        const bool primaryHand,
        LockedHandVisualLerpState& visualState)
    {
        const RE::NiTransform& startWorld =
            primaryHand ?
                _dynamicSupportAcquisition.primaryStartWorld :
                _dynamicSupportAcquisition.supportStartWorld;
        visualState.active = true;
        visualState.startWorld = startWorld;
        visualState.elapsedSeconds =
            _dynamicSupportAcquisition.elapsedSeconds;
        visualState.durationSeconds =
            _dynamicSupportAcquisition.durationSeconds;
        visualState.lastAlpha =
            _dynamicSupportAcquisition.easedAlpha;
        return hand_visual_lerp_math::interpolateTransform(
            startWorld,
            targetWorld,
            _dynamicSupportAcquisition.easedAlpha);
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
         * Authored, provider-owned, and visual-only support paths retain their
         * independent external-hand transition. Normal dynamic full-authority
         * acquisition is intercepted by resolveDynamicSupportAcquisitionHandTarget
         * so both hands share the pivot-preserving weapon correction alpha.
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
        const WeaponProviderPartAuthority& providerPartAuthority,
        const bool firingGripProximityAuthorityEnabled)
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

        performance_profiler::ScopedTimer fingerPoseCaptureTimer(performance_profiler::Scope::EquippedWeaponFingerPoseCapture);

        const RE::NiPoint3 palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, isLeft);
        const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handTransform, isLeft);

        /*
         * Acquisition-only authored priority. A pure proximity probe snaps to
         * an eligible authored support relation. Eligibility requires the
         * resolved palm seat itself to have a current generated-mesh surface
         * witness, so an animation-zero/default support hand cannot escape to
         * an unrelated world-space pose. A physical/recent touch substitutes
         * authored only when its small live palm probe is inside that authored
         * palm-seat radius; every rejected authored candidate continues into
         * the unrestricted dynamic mesh grab below. The final authored seat
         * also selects visual-only versus full weapon authority. Provider
         * AttachOnly remains PAPER/consumer glue. Once selected, the exact
         * hand/weapon relation and 15 finger locals are latched; later
         * candidate changes cannot move it.
         */
        const bool authoredWeaponIdentityMatches =
            _authoredSupportGripCandidate.weaponNode == weaponNode;
        const bool authoredGenerationMatches =
            _authoredSupportGripCandidate.weaponGenerationKey ==
            decision.weaponGenerationKey;
        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15> authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask = 0;
        const bool authoredSupportCandidateForHandValid =
            tryResolveAuthoredSupportGripCandidateForHand(
                isLeft,
                weaponNode,
                decision.weaponGenerationKey,
                authoredSupportHandWeaponLocal,
                authoredSupportFingerLocalTransforms,
                authoredSupportFingerLocalTransformMask);
        AuthoredSupportPalmSeatProximity authoredSupportProximity{};
        RE::NiTransform authoredSupportHandWorld{};
        RE::NiPoint3 authoredSupportPalmWeaponLocal{};
        RE::NiPoint3 authoredSupportPalmNormalWorld{};
        float authoredSupportTouchProbeDistance =
            (std::numeric_limits<float>::infinity)();
        float authoredSupportPalmToFiringGripDistance =
            (std::numeric_limits<float>::infinity)();
        bool authoredSupportFrameValid = false;
        if (authoredSupportCandidateForHandValid &&
            resolveAuthoredSupportPalmSeatProximity(
                weaponNode->world,
                handTransform,
                authoredSupportHandWeaponLocal,
                isLeft,
                authoredSupportProximity)) {
            authoredSupportHandWorld = authoredSupportProximity.authoredHandWorld;
            authoredSupportPalmWeaponLocal =
                authoredSupportProximity.authoredPalmSeatWeaponLocal;
            authoredSupportPalmNormalWorld =
                computePalmNormalFromHandBasis(
                    authoredSupportHandWorld,
                    isLeft);
            authoredSupportTouchProbeDistance =
                authoredSupportProximity.weaponRelativeDistanceGameUnits;
            authoredSupportFrameValid =
                std::isfinite(authoredSupportTouchProbeDistance) &&
                std::isfinite(authoredSupportPalmNormalWorld.x) &&
                std::isfinite(authoredSupportPalmNormalWorld.y) &&
                std::isfinite(authoredSupportPalmNormalWorld.z);
        }

        WeaponCollision::WeaponSurfaceProximityWitness
            authoredSupportSurfaceWitness{};
        const bool authoredSeatWeaponSurfaceValid =
            authoredSupportFrameValid &&
            authoredWeaponIdentityMatches &&
            authoredGenerationMatches &&
            weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(
                weaponNode,
                authoredSupportProximity.authoredPalmSeatWorld,
                g_rockConfig.rockWeaponInteractionTouchRadius,
                authoredSupportSurfaceWitness) &&
            authoredSupportSurfaceWitness.weaponGenerationKey ==
                decision.weaponGenerationKey &&
            authoredSupportSurfaceWitness.weaponGenerationKey ==
                _activeWeaponGenerationKey;
        const float authoredSupportSurfaceDistance =
            authoredSeatWeaponSurfaceValid ?
            authoredSupportSurfaceWitness.distanceGameUnits :
            (std::numeric_limits<float>::infinity)();

        bool authoredSupportAuthorityGateValid =
            !firingGripProximityAuthorityEnabled;
        auto authoredSupportAuthorityMode =
            weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
        if (authoredSupportFrameValid &&
            firingGripProximityAuthorityEnabled &&
            std::isfinite(weaponNode->world.scale)) {
            const RE::NiPoint3 authoredSeatToFiringGripLocal =
                sub(authoredSupportPalmWeaponLocal, _primaryGripLocal);
            const float authoredSeatToFiringGripLocalDistance = std::sqrt(
                dot(authoredSeatToFiringGripLocal,
                    authoredSeatToFiringGripLocal));
            authoredSupportPalmToFiringGripDistance =
                authoredSeatToFiringGripLocalDistance *
                std::abs(weaponNode->world.scale);
            if (std::isfinite(authoredSupportPalmToFiringGripDistance)) {
                authoredSupportAuthorityMode =
                    weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                        authoredSupportPalmToFiringGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits);
                authoredSupportAuthorityGateValid = true;
            }
        }

        const bool authoredSeatTouchAcquisition =
            decision.acquisitionSource ==
                WeaponInteractionAcquisitionSource::PhysicalContact &&
            authoredSupportFrameValid &&
            authoredSupportTouchProbeDistance <=
                g_rockConfig.rockWeaponInteractionTouchRadius;

        constexpr std::uint16_t kCompleteAuthoredFingerMask = 0x7FFFu;
        const bool useAuthoredSupportGrip =
            authored_weapon_grip_capture_policy::shouldUseAuthoredSupportGrip(
                authored_weapon_grip_capture_policy::AuthoredSupportGripCandidateInput{
                    .proximityProbeAcquisition =
                        decision.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe,
                    .authoredSeatTouchAcquisition =
                        authoredSeatTouchAcquisition,
                    .providerAuthorityActive = providerPartAuthority.active,
                    .attachOnly = grip.attachOnly,
                    .captureValid =
                        authoredSupportCandidateForHandValid &&
                        authoredSupportFrameValid &&
                        authoredSupportAuthorityGateValid,
                    .weaponIdentityMatches = authoredWeaponIdentityMatches,
                    .generationMatches = authoredGenerationMatches,
                    .authoredSeatWeaponSurfaceValid =
                        authoredSeatWeaponSurfaceValid,
                    .completeFingerPose =
                        authoredSupportFingerLocalTransformMask ==
                        kCompleteAuthoredFingerMask,
                });
        if (useAuthoredSupportGrip) {
            if (firingGripProximityAuthorityEnabled) {
                _authorityMode = authoredSupportAuthorityMode;
            }
            grip.authoredSupportGrip = true;
            grip.authoredSupportCaptureSequence =
                _authoredSupportGripCandidate.captureSequence;
            grip.attachmentRoot = weaponNode;
            grip.gripLocal = authoredSupportPalmWeaponLocal;
            grip.grabNormalWorld = authoredSupportPalmNormalWorld;
            grip.normalLocal = transform_math::worldVectorToLocal(
                weaponNode->world,
                authoredSupportPalmNormalWorld);
            grip.handWeaponLocal =
                authoredSupportHandWeaponLocal;
            grip.hasHandWeaponLocal = true;
            grip.hasSourceFrames = false;
            grip.hasAttachmentWeaponLocal = false;

            // hFRIK requires the role-tagged numeric pose to exist before the
            // exact per-joint local override can win at the same priority.
            setSupportGripPose(isLeft, nullptr, nullptr);
            grip.fingerLocalTransforms =
                authoredSupportFingerLocalTransforms;
            grip.fingerLocalTransformMask =
                authoredSupportFingerLocalTransformMask;
            grip.hasFingerLocalTransforms = true;
            grip.visualLerp = {};
            grip.active = true;

            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles,
                0);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles,
                0);
            if (isLeft) {
                _hapticEvents.leftPartGripCaptured = true;
            } else {
                _hapticEvents.rightPartGripCaptured = true;
            }

            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: authored support grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) touchToSeat={:.3f} touchRadius={:.3f} surfaceDistance={:.3f} surfaceBody={} surfaceSourceCurrent={} authoredSeatToFiringGrip={:.3f} seatLocal=({:.3f},{:.3f},{:.3f}) touchLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} capture={} generation={:016X} acquisition={} authority={} priority=provider>authored>dynamic",
                isLeft ? "left" : "right",
                weaponNode->name.c_str(),
                grip.gripLocal.x,
                grip.gripLocal.y,
                grip.gripLocal.z,
                authoredSupportTouchProbeDistance,
                g_rockConfig.rockWeaponInteractionTouchRadius,
                authoredSupportSurfaceDistance,
                authoredSupportSurfaceWitness.bodyId,
                authoredSupportSurfaceWitness.sourceNodeCurrent ? "yes" : "no",
                authoredSupportPalmToFiringGripDistance,
                authoredSupportPalmWeaponLocal.x,
                authoredSupportPalmWeaponLocal.y,
                authoredSupportPalmWeaponLocal.z,
                authoredSupportProximity.liveTouchProbeWeaponLocal.x,
                authoredSupportProximity.liveTouchProbeWeaponLocal.y,
                authoredSupportProximity.liveTouchProbeWeaponLocal.z,
                authoredSupportProximity.frameAgreementErrorGameUnits,
                grip.authoredSupportCaptureSequence,
                _activeWeaponGenerationKey,
                decision.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::PhysicalContact ?
                    "authored-seat-touch" :
                    "probe",
                _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                    "visual-only" :
                    "full");
            return true;
        }

        auto& fingerScratch = _fingerPoseSolveScratch->hands[isLeft ? 0u : 1u];
        fingerScratch.ranking.clear();
        fingerScratch.localTriangles.clear();
        fingerScratch.worldTriangles.clear();
        fingerScratch.spatialIndex.clear();

        WeaponCollision::SupportGripEvidenceView evidenceView{};
        const bool cachedTrianglesFound = weaponCollision.tryGetSupportGripEvidenceView(decision.bodyId, weaponNode, evidenceView) &&
            evidenceView.weaponGenerationKey == decision.weaponGenerationKey &&
            evidenceView.weaponGenerationKey == _activeWeaponGenerationKey;
        const std::size_t sourceTriangleCount = cachedTrianglesFound ? evidenceView.localTriangles.size() : 0u;
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles,
            static_cast<std::uint64_t>(sourceTriangleCount));

        GrabPoint grabPoint{};
        bool meshFound = false;
        if (cachedTrianglesFound) {
            const TransformedSupportGripTriangleView worldEvidence{
                .localTriangles = evidenceView.localTriangles,
                .localToWorld = evidenceView.localToWorld,
            };
            meshFound = findClosestGrabPoint(
                worldEvidence,
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
        if (cachedTrianglesFound && g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            const RE::NiPoint3 triangleSelectionPointLocal = transform_math::worldPointToLocal(evidenceView.localToWorld, gripWorldPoint);
            selectNearestSupportGripFingerTriangles(
                evidenceView.localTriangles,
                triangleSelectionPointLocal,
                grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles,
                fingerScratch.ranking,
                fingerScratch.localTriangles);
        }
        performance_profiler::observeValue(
            performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles,
            static_cast<std::uint64_t>(fingerScratch.localTriangles.size()));

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
        std::array<float, 5> capturedFingerSplayRadians{};
        const std::array<float, 5>* capturedFingerSplayRadiansPtr = nullptr;
        bool spatialIndexBuilt = false;
        bool commandedOpenDirectionsValid = false;
        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && !fingerScratch.localTriangles.empty()) {
            const RE::NiTransform frozenMeshWorld = weapon_two_handed_grip_math::virtualizeMeshForTranslatedHandSeat(
                evidenceView.localToWorld,
                palmPos,
                gripWorldPoint);
            const RE::NiPoint3 frozenGripPoint = weapon_two_handed_grip_math::virtualizeGripPointForTranslatedHandSeat(
                palmPos,
                gripWorldPoint);
            auto fingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(frozenGripPoint, grip.grabNormalWorld);
            fingerPoseTargets.useSeatPointForMissingTargets = false;
            fingerPoseTargets.useWholeMeshForMissingTargets = true;
            const auto frozenSolve = grab_finger_pose_runtime::solveFrozenMeshFingerPose(
                fingerScratch.localTriangles,
                frozenMeshWorld,
                handTransform,
                isLeft,
                frozenGripPoint,
                fingerPoseTargets,
                fingerScratch.spatialIndex,
                fingerScratch.worldTriangles,
                grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                    .minValue = g_rockConfig.rockGrabFingerMinValue,
                    .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                    .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                    .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                    .allowSurfaceAimTargets = true,
                    .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                    .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                    .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                    .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                    .captureSweepDebug = false,
                });
            spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
            commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
            meshFingerPose = frozenSolve.pose;
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits,
                meshFingerPose.spatialNodeVisitCount);
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::EquippedWeaponFingerPoseTriangleTests,
                meshFingerPose.spatialTriangleTestCount);
            if (meshFingerPose.solved) {
                meshFingerPosePtr = &meshFingerPose;
                if (frozenSolve.liveFingerSnapshotValid &&
                    grab_finger_pose_runtime::buildSurfaceContactSplayValues(
                        meshFingerPose,
                        frozenSolve.liveFingerSnapshot,
                        capturedFingerSplayRadians)) {
                    capturedFingerSplayRadiansPtr = &capturedFingerSplayRadians;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} sourceTris={} candidateTris={} spatial={} nodes={} tests={} commandedAnchors={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    meshFingerPose.hitCount,
                    sourceTriangleCount,
                    meshFingerPose.candidateTriangleCount,
                    spatialIndexBuilt ? "yes" : "no",
                    meshFingerPose.spatialNodeVisitCount,
                    meshFingerPose.spatialTriangleTestCount,
                    commandedOpenDirectionsValid ? "yes" : "no",
                    meshFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                if (meshFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        meshFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbPrimaryCurve.value,
                        meshFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.value,
                        meshFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.value,
                        meshFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                }
            }
        }

        setSupportGripPose(isLeft, meshFingerPosePtr, capturedFingerSplayRadiansPtr);
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
            "TwoHandedGrip: part grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) meshGrab={} sourceTriangles={} fingerTriangles={} cachedTriangles={} sourceNodeCurrent={} authoredSupport=NO acquisition={} authority={} provider={} attachOnly={} authoredCandidate={} authoredFrame={} authoredIdentity={} authoredGeneration={} authoredSurface={} surfaceDistance={:.3f} surfaceRadius={:.3f} authoredFingerMask=0x{:04X} touchToAuthoredSeat={:.3f} authoredSeatLocal=({:.3f},{:.3f},{:.3f}) touchProbeLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} partKind={} pose={} generation={:016X}",
            isLeft ? "left" : "right",
            weaponNode->name.c_str(),
            grip.gripLocal.x,
            grip.gripLocal.y,
            grip.gripLocal.z,
            meshFound ? "YES" : "FALLBACK",
            sourceTriangleCount,
            fingerScratch.localTriangles.size(),
            cachedTrianglesFound ? "yes" : "no",
            cachedTrianglesFound && evidenceView.sourceNodeCurrent ? "yes" : "no",
            decision.acquisitionSource == WeaponInteractionAcquisitionSource::PhysicalContact ?
                "contact" :
                (decision.acquisitionSource == WeaponInteractionAcquisitionSource::ProximityProbe ? "probe" : "none"),
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                "visual-only" :
                "full",
            providerPartAuthority.active ? "yes" : "no",
            grip.attachOnly ? "yes" : "no",
            authoredSupportCandidateForHandValid ? "yes" : "no",
            authoredSupportFrameValid ? "yes" : "no",
            authoredWeaponIdentityMatches ? "yes" : "no",
            authoredGenerationMatches ? "yes" : "no",
            authoredSeatWeaponSurfaceValid ? "yes" : "no",
            authoredSupportSurfaceDistance,
            g_rockConfig.rockWeaponInteractionTouchRadius,
            authoredSupportFingerLocalTransformMask,
            authoredSupportTouchProbeDistance,
            authoredSupportPalmWeaponLocal.x,
            authoredSupportPalmWeaponLocal.y,
            authoredSupportPalmWeaponLocal.z,
            authoredSupportProximity.liveTouchProbeWeaponLocal.x,
            authoredSupportProximity.liveTouchProbeWeaponLocal.y,
            authoredSupportProximity.liveTouchProbeWeaponLocal.z,
            authoredSupportProximity.frameAgreementErrorGameUnits,
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
        if (dynamicSupportAcquisitionMatches(isLeft, grip)) {
            clearDynamicSupportAcquisition(reason, true);
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
        clearDynamicSupportAcquisition(
            "new-two-hand-acquisition",
            true);

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
            hasRightFiringHandCanonicalFrame(
                weaponNode,
                decision.weaponGenerationKey,
                currentEquippedWeaponOwnershipKey),
            _rightFiringHandCanonicalGenerationKey,
            decision.weaponGenerationKey);
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
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits);
                    ROCK_LOG_INFO(Weapon,
                        "TwoHandedGrip: firing-grip proximity support distance={:.2f} radius={:.2f} mode={}",
                        supportPalmToGripDistance,
                        _handlingSettings.firingGripProximitySupportRadiusGameUnits,
                        _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                            "visual-only" :
                            "full-authority");
                }
            }
        }

        if (!capturePartGrip(
                supportHandIsLeft,
                weaponNode,
                decision,
                weaponCollision,
                providerPartAuthority,
                firingGripProximityAuthorityEnabled)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: support grip start skipped because part grip capture failed");
            restoreFrikOffhandGrip();
            return;
        }

        const RE::NiPoint3 supportGripWorldPoint = resolvePartGripWorld(partGrip(supportHandIsLeft), weaponNode);
        const RE::NiPoint3 primaryToSupportWorld = sub(supportGripWorldPoint, primaryPalmPos);
        _lockedGripSeparationWorld = std::sqrt(dot(primaryToSupportWorld, primaryToSupportWorld));

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        const bool useDynamicSupportAcquisition =
            weapon_support_authority_policy::
                shouldUseDynamicSupportAcquisition(
                    _authorityMode,
                    supportGrip.authoredSupportGrip,
                    supportGrip.providerPartAuthority.active,
                    supportGrip.attachOnly);
        RE::NiTransform dynamicPrimaryStartWorld{};
        RE::NiTransform dynamicSupportStartWorld{};
        if (useDynamicSupportAcquisition) {
            RE::NiTransform supportTransform{};
            if (!tryGetSolverHandTransform(
                    supportHandIsLeft,
                    supportTransform)) {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: dynamic support acquisition skipped because the support hand frame disappeared after capture");
                transitionToInactive(false);
                return;
            }

            const auto& primaryReturn =
                _returningHandVisuals[
                    primaryHandIsLeft ? 0u : 1u]
                    .transition;
            const auto& supportReturn =
                _returningHandVisuals[
                    supportHandIsLeft ? 0u : 1u]
                    .transition;
            dynamicPrimaryStartWorld =
                primaryReturn.active &&
                    isUsableHandAuthorityTransform(
                        primaryReturn.lastApplied) ?
                primaryReturn.lastApplied :
                primaryTransform;
            dynamicSupportStartWorld =
                supportReturn.active &&
                    isUsableHandAuthorityTransform(
                        supportReturn.lastApplied) ?
                supportReturn.lastApplied :
                supportTransform;
        }

        _state = TwoHandedState::Gripping;
        _rotationBlend = 0.0f;
        _gripLogCounter = 0;
        _supportGripAgeFrames = 0;
        _freshSupportGripDeferLogged = false;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: grip active weapon='{}', "
            "primaryLocal=({:.3f},{:.3f},{:.3f}), supportLocal=({:.3f},{:.3f},{:.3f}), "
            "gripSeparation={:.3f}, primaryGripSource={}, primaryGripConfidence={:.2f}, partKind={}, pose={}, authorityMode={}, generation={:016X}",
            weaponNode->name.c_str(), _primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z, supportGrip.gripLocal.x, supportGrip.gripLocal.y, supportGrip.gripLocal.z,
            _lockedGripSeparationWorld, reuseRightFiringCanonicalGrip ? "pre-scope-canonical" : (_scopeMenuOpenThisFrame ? "frik-driver-reconstructed" : "root-flattened"),
            _primaryGripConfidence, static_cast<int>(supportGrip.partKind), static_cast<int>(supportGrip.gripPose), static_cast<int>(_authorityMode), _activeWeaponGenerationKey);

        if (useDynamicSupportAcquisition) {
            beginDynamicSupportAcquisition(
                supportHandIsLeft,
                supportGrip,
                dynamicPrimaryStartWorld,
                dynamicSupportStartWorld);
            /*
             * Capture and first publication are one transaction. dt=0 keeps
             * alpha exactly zero while publishing the frozen finger pose, the
             * pivot-preserving one-hand weapon frame, and both live hand roots
             * before this update returns to PhysicsInteraction.
             */
            updateFullWeaponAuthorityGrip(weaponNode, 0.0f);
        }
    }

    void TwoHandedGrip::transitionToInactive(bool publishRestoredWeaponTransform)
    {
        clearDynamicSupportAcquisition(
            "transition-to-inactive",
            true);
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
         * the hand was already holding the part (PAPER_Toolkit: pulling the
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

        bool dynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                supportHandIsLeft,
                supportGrip);
        if (_dynamicSupportAcquisition.active &&
            !dynamicAcquisition) {
            clearDynamicSupportAcquisition(
                "grip-or-generation-witness-changed",
                true);
        }
        if (!dynamicAcquisition) {
            _rotationBlend = (std::min)(
                1.0f,
                _rotationBlend +
                    (std::isfinite(dt) && dt > 0.0f ? dt : 0.0f) *
                        ROTATION_BLEND_SPEED);
        }

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

        WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
        solverInput.weaponWorldTransform = weaponNode->world;
        solverInput.primaryGripLocal = _primaryGripLocal;
        solverInput.supportGripLocal = supportGripLocal;
        solverInput.primaryTargetWorld = primaryController;
        solverInput.supportTargetWorld =
            dynamicAcquisition ?
                lockedSupportControllerTarget :
                lerpPoint(
                    currentSupportWorld,
                    lockedSupportControllerTarget,
                    _rotationBlend);
        solverInput.supportNormalLocal = resolvePartGripNormalWeaponLocal(supportGrip, weaponNode);
        solverInput.supportNormalTargetWorld = computePalmNormalFromHandBasis(supportTransform, supportHandIsLeft);
        solverInput.useSupportNormalTwist = true;
        solverInput.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;

        const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
        if (!solved.solved) {
            if (dynamicAcquisition) {
                clearDynamicSupportAcquisition(
                    "full-target-solve-degenerate",
                    true);
                transitionToInactive(false);
            }
            return;
        }

        RE::NiTransform appliedWeaponWorld =
            solved.weaponWorldTransform;
        if (dynamicAcquisition) {
            auto& acquisition = _dynamicSupportAcquisition;
            if (!acquisition.durationInitialized) {
                auto axisOnlyInput = solverInput;
                axisOnlyInput.useSupportNormalTwist = false;
                axisOnlyInput.supportNormalTwistFactor = 0.0f;
                const auto axisOnlySolved =
                    solveTwoHandedWeaponTransformFrikPivot(
                        axisOnlyInput);

                acquisition.fullCorrectionRadians =
                    weapon_support_acquisition_math::
                        rotationAngleRadians(solved.rotationDelta);
                acquisition.axisCorrectionRadians =
                    axisOnlySolved.solved ?
                    weapon_support_acquisition_math::
                        rotationAngleRadians(
                            axisOnlySolved.rotationDelta) :
                    0.0f;
                acquisition.twistContributionRadians =
                    axisOnlySolved.solved ?
                    weapon_support_acquisition_math::
                        rotationDistanceRadians(
                            axisOnlySolved.rotationDelta,
                            solved.rotationDelta) :
                    0.0f;
                if (!std::isfinite(
                        acquisition.fullCorrectionRadians) ||
                    !std::isfinite(
                        acquisition.axisCorrectionRadians) ||
                    !std::isfinite(
                        acquisition.twistContributionRadians)) {
                    clearDynamicSupportAcquisition(
                        "non-finite-correction-angle",
                        true);
                    transitionToInactive(false);
                    return;
                }

                const RE::NiTransform fullPrimaryHandWorld =
                    weapon_visual_authority_math::
                        weaponLocalFrameToWorld(
                            solved.weaponWorldTransform,
                            _primaryHandWeaponLocal);
                const RE::NiTransform fullSupportHandWorld =
                    weapon_visual_authority_math::
                        weaponLocalFrameToWorld(
                            solved.weaponWorldTransform,
                            supportGrip.handWeaponLocal);
                const float primarySeatDistance =
                    hand_visual_lerp_math::
                        distanceGameUnits(
                            acquisition.primaryStartWorld.translate,
                            fullPrimaryHandWorld.translate);
                const float supportSeatDistance =
                    hand_visual_lerp_math::
                        distanceGameUnits(
                            acquisition.supportStartWorld.translate,
                            fullSupportHandWorld.translate);
                acquisition.initialSeatDistanceGameUnits =
                    (std::max)(
                        primarySeatDistance,
                        supportSeatDistance);
                acquisition.durationSeconds = 0.0f;
                if (g_rockConfig
                        .rockWeaponSupportGripHandLerpEnabled) {
                    acquisition.durationSeconds =
                        hand_visual_lerp_math::
                            computeDistanceMappedDurationGameUnits(
                                acquisition
                                    .initialSeatDistanceGameUnits,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpTimeMin,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpTimeMax,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpMinDistance,
                                g_rockConfig
                                    .rockWeaponSupportGripHandLerpMaxDistance);
                    if (acquisition.durationSeconds <= 0.0f &&
                        acquisition.fullCorrectionRadians >=
                            DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS) {
                        acquisition.durationSeconds =
                            1.0f / ROTATION_BLEND_SPEED;
                    }
                }
                acquisition.durationInitialized = true;
            }

            acquisition.elapsedSeconds =
                hand_visual_lerp_math::
                    advanceTimedBlendElapsed(
                        acquisition.elapsedSeconds,
                        dt,
                        acquisition.durationSeconds);
            acquisition.rawAlpha =
                hand_visual_lerp_math::timedBlendAlpha(
                    acquisition.elapsedSeconds,
                    acquisition.durationSeconds);
            acquisition.easedAlpha =
                weapon_support_acquisition_math::
                    timedSmoothStepAlpha(
                        acquisition.elapsedSeconds,
                        acquisition.durationSeconds);
            _rotationBlend = acquisition.easedAlpha;

            const auto acquisitionSolve =
                weapon_support_acquisition_math::
                    applyRotationAroundPrimaryPivot<
                        RE::NiTransform,
                        RE::NiPoint3>(
                        solverInput.weaponWorldTransform,
                        solved.rotationDelta,
                        _primaryGripLocal,
                        primaryController,
                        acquisition.easedAlpha);
            if (!acquisitionSolve.valid) {
                clearDynamicSupportAcquisition(
                    "pivot-preserving-slerp-invalid",
                    true);
                transitionToInactive(false);
                return;
            }
            appliedWeaponWorld =
                acquisitionSolve.weaponWorldTransform;
            acquisition.lastAppliedRotationRadians =
                acquisitionSolve.appliedRotationRadians;
            acquisition.lastPrimaryPivotError =
                acquisitionSolve.primaryError;
            const RE::NiPoint3 appliedSupportWorld =
                transform_math::localPointToWorld(
                    appliedWeaponWorld,
                    supportGripLocal);
            acquisition.lastSupportTargetError =
                weaponSolverLength(
                    weaponSolverSub(
                        appliedSupportWorld,
                        lockedSupportControllerTarget));
        }

        if (!applyWeaponVisualAuthority(weaponNode, appliedWeaponWorld)) {
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

        if (dynamicAcquisition) {
            auto& acquisition = _dynamicSupportAcquisition;
            if (!acquisition.firstPublicationRecorded) {
                acquisition.firstPublicationRecorded = true;
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic support acquisition event=start hand={} authored=no provider=no attachOnly=no grip={} generation={:016X} captureToFirstPublicationFrames=0 seatDistance={:.3f} duration={:.3f}s fullCorrection={:.2f}deg axisCorrection={:.2f}deg twistContribution={:.2f}deg firstRawAlpha={:.3f} firstEasedAlpha={:.3f} firstAppliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                    supportHandIsLeft ? "left" : "right",
                    acquisition.gripSequence,
                    acquisition.weaponGenerationKey,
                    acquisition.initialSeatDistanceGameUnits,
                    acquisition.durationSeconds,
                    acquisition.fullCorrectionRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.axisCorrectionRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.twistContributionRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.rawAlpha,
                    acquisition.easedAlpha,
                    acquisition.lastAppliedRotationRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
            }

            if (acquisition.easedAlpha >= 1.0f) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: dynamic support acquisition event=complete hand={} grip={} generation={:016X} elapsed={:.3f}s duration={:.3f}s appliedCorrection={:.2f}deg primaryPivotError={:.4f} supportTargetError={:.4f}",
                    supportHandIsLeft ? "left" : "right",
                    acquisition.gripSequence,
                    acquisition.weaponGenerationKey,
                    acquisition.elapsedSeconds,
                    acquisition.durationSeconds,
                    acquisition.lastAppliedRotationRadians *
                        RADIANS_TO_DEGREES,
                    acquisition.lastPrimaryPivotError,
                    acquisition.lastSupportTargetError);
                _rotationBlend = 1.0f;
                clearDynamicSupportAcquisition(
                    "completed",
                    false);
                dynamicAcquisition = false;
            }
        }

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal), sub(primaryGripFinal, offhandGripFinal)));
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: blend={:.2f}, dynamicAcquisition={}, separation={:.1f}gu, "
                "primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f}), handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                _rotationBlend,
                dynamicAcquisition ? "active" : "inactive",
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
        clearDynamicSupportAcquisition(
            "transition-to-part-carry",
            true);
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
        return !isLeft || leftFiringInfrastructureAvailable();
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
            !hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
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
        const bool usedAuthoredCanonical =
            _rightFiringHandCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: persistent left-hand carry active generation={:016X} ownership={:016X} source={} capture={}",
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            usedAuthoredCanonical ? "authored-animation" : "native-carry",
            _rightFiringHandCanonicalCaptureSequence);
        return true;
    }

    void TwoHandedGrip::clearPersistentEquippedCarry(const char* reason)
    {
        if (!_persistentEquippedCarryActive) {
            return;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: clearing persistent left-hand carry reason={}",
            reason ? reason : "unknown");
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
        if (isManualOwnershipActive()) {
            transitionToInactive(false);
        }
    }

    void TwoHandedGrip::restoreNativeRightEquippedCarry(const char* reason)
    {
        clearPersistentEquippedCarry(reason);
        if (!isManualOwnershipActive()) {
            return;
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: restoring native right-hand carry reason={}",
            reason ? reason : "unknown");
        transitionToInactive(false);
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
        outReport.authoredSupportGrip = grip.authoredSupportGrip;
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

        clearDynamicSupportAcquisition(
            reason ? reason : "transition-to-primary-only",
            true);
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

        (void)publishAuthoredPrimaryFiringGripFingerPose(true);
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
        return std::isfinite(distance) &&
               distance <= _handlingSettings.firingGripReattachRadiusGameUnits;
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

    bool TwoHandedGrip::tryResolveAuthoredFiringHandCanonicalForProbe(
        const bool handIsLeft,
        RE::NiTransform& outHandWeaponLocal,
        const char*& outSource) const
    {
        outHandWeaponLocal = {};
        outSource = "unavailable";
        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        if (!hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) ||
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        if (handIsLeft) {
            if (_leftFiringFingerLocalTransformMask !=
                kCompleteFingerLocalTransformMask) {
                return false;
            }
            bool usedAuthoredCanonical = false;
            if (!tryComputeMirroredLeftFiringHandWeaponLocal(
                    outHandWeaponLocal,
                    &usedAuthoredCanonical) ||
                !usedAuthoredCanonical) {
                return false;
            }
            outSource = "authored-mirror-probe";
            return true;
        }

        if (_rightFiringFingerLocalTransformMask !=
            kCompleteFingerLocalTransformMask) {
            return false;
        }
        outHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
        outSource = "authored-canonical-probe";
        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::tryReattachFiringGrip(
        const bool handIsLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& handWeaponContact,
        const bool authoredProviderAuthorityActive,
        const bool authoredAttachOnlyAuthorityActive)
    {
        if (!weaponNode ||
            !handWeaponContact.valid ||
            !weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(
                handWeaponContact.weaponGenerationKey,
                _activeWeaponGenerationKey)) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(handIsLeft, handTransform)) {
            return false;
        }

        RE::NiTransform authoredProbeCanonical{};
        const char* authoredProbeCanonicalSource = "unavailable";
        const bool authoredProbeCanonicalAvailable =
            tryResolveAuthoredFiringHandCanonicalForProbe(
                handIsLeft,
                authoredProbeCanonical,
                authoredProbeCanonicalSource);
        const bool useAuthoredProbeCanonical =
            authored_weapon_grip_capture_policy::shouldUseAuthoredFiringGripProbe(
                authored_weapon_grip_capture_policy::AuthoredFiringGripProbeInput{
                    .proximityProbeAcquisition =
                        handWeaponContact.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe,
                    .providerAuthorityActive = authoredProviderAuthorityActive,
                    .attachOnly = authoredAttachOnlyAuthorityActive,
                    .authoredCanonicalAvailable =
                        authoredProbeCanonicalAvailable,
                });
        if (!useAuthoredProbeCanonical &&
            !firingGripContactMatchesCapturedGrip(
                weaponNode,
                handWeaponContact,
                handTransform,
                handIsLeft)) {
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
        bool usedAuthoredCanonical = false;
        const char* holdSource = "live-capture";
        if (useAuthoredProbeCanonical) {
            _primaryHandWeaponLocal = authoredProbeCanonical;
            usedCanonicalHold = true;
            usedAuthoredCanonical = true;
            holdSource = authoredProbeCanonicalSource;
        } else if (handIsLeft) {
            RE::NiTransform mirroredHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    mirroredHandWeaponLocal,
                    &usedAuthoredCanonical)) {
                _primaryHandWeaponLocal = mirroredHandWeaponLocal;
                usedCanonicalHold = true;
                holdSource = usedAuthoredCanonical ?
                    "authored-mirror" :
                    "native-mirror";
            }
        } else if (hasRightFiringHandCanonicalFrame(
                       _activeWeaponNode,
                       _activeWeaponGenerationKey,
                       _activeEquippedWeaponOwnershipKey)) {
            _primaryHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
            usedCanonicalHold = true;
            holdSource = _rightFiringHandCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" :
                "native-canonical";
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
        const bool authoredProviderAuthorityActive =
            leftRuntimeState.providerPartAuthority.active ||
            rightRuntimeState.providerPartAuthority.active;
        const bool authoredAttachOnlyAuthorityActive =
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                leftRuntimeState.providerPartAuthority.active,
                leftRuntimeState.providerPartAuthority.grabMode) ||
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                rightRuntimeState.providerPartAuthority.active,
                rightRuntimeState.providerPartAuthority.grabMode) ||
            (partGrip(true).active && partGrip(true).attachOnly) ||
            (partGrip(false).active && partGrip(false).attachOnly);

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
            if (candidate.isLeft != firingHandIsLeft &&
                (!_handlingSettings.ambidextrousHandoffEnabled ||
                    !canBeginPrimaryOnlyGripForHand(candidate.isLeft))) {
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
                _handlingSettings.firingGripReattachRadiusGameUnits) ||
                (candidate.gripHeld &&
                    candidate.contact->acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe &&
                    !authoredProviderAuthorityActive &&
                    !authoredAttachOnlyAuthorityActive);
            if (!_firingGripReattachHoverInsideRadius &&
                weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate(
                    candidate.gripHeld,
                    palmToGripDistance,
                    _handlingSettings.firingGripReattachRadiusGameUnits)) {
                _firingGripReattachHoverInsideRadius = true;
                _firingGripReattachHoverHandIsLeft = candidate.isLeft;
            }
            if (reattachRequested &&
                tryReattachFiringGrip(
                    candidate.isLeft,
                    weaponNode,
                    *candidate.contact,
                    authoredProviderAuthorityActive,
                    authoredAttachOnlyAuthorityActive)) {
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
                (void)capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority, false);
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
                (void)capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority, false);
            }
        }

        if (!freeHandGrip.active && frameInput.primaryGripInput.pressed) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (weapon_two_handed_grip_math::canStartFreeHandPartGrip(
                    freeHandDecision.kind == WeaponInteractionKind::SupportGrip,
                    frameInput.primaryGripInput.pressed,
                    firingHandHoldingObject,
                    freeHandGrip.active)) {
                if (capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority, false)) {
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
                if (capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority, false)) {
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

    void TwoHandedGrip::setSupportGripPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose,
        const std::array<float, 5>* capturedSplayRadians)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (meshFingerPose && meshFingerPose->solved) {
            grip.fingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            grip.fingerSplayRadians = capturedSplayRadians ? *capturedSplayRadians : std::array<float, 5>{};
            grip.hasFingerSplay = capturedSplayRadians != nullptr;
            grip.hasFingerPose = true;
            return;
        }

        const float fallbackMin =
            std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
        const float configuredFallback =
            std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
        const float fallbackValue = std::clamp(configuredFallback, fallbackMin, 1.0f);
        const std::array<float, 5> fallbackCurls{
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
        };
        grip.fingerPose = grab_finger_pose_math::expandFingerCurlsToJointValues(fallbackCurls);
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = true;
        grip.hasFingerSplay = false;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: using selected-close finger fallback hand={} value={:.2f}",
            isLeft ? "left" : "right",
            fallbackValue);
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
        if (_scopeMenuOpenThisFrame || _scopeMenuClosedThisFrame) {
            deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
        } else {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
        }
    }

    void TwoHandedGrip::deferScopeHandAuthorityClear(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        _scopeDeferredHandAuthorityClears[isLeft ? 0u : 1u] |= scope_safe_hand_frame_math::roleMask(role);
    }

    void TwoHandedGrip::recordScopeHandAuthorityPublication(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scopeHandAuthorityPublishedThisFrame[index] |= roleBit;
        // A successfully republished role is live again; a clear requested for
        // the same role while ScopeMenu was open is obsolete.
        _scopeDeferredHandAuthorityClears[index] &= static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
    }

    bool TwoHandedGrip::clearHandAuthorityRoleNow(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const char* tag = nullptr;
        switch (role) {
        case scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip:
            tag = PRIMARY_GRIP_TAG;
            break;
        case scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip:
            tag = SUPPORT_GRIP_TAG;
            break;
        case scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach:
            tag = PRIMARY_DETACH_TAG;
            break;
        }

        if (!tag || !frik_visual_authority::clearExternalHandWorldTransform(tag, handFromBool(isLeft))) {
            return false;
        }

        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scopeDeferredHandAuthorityClears[isLeft ? 0u : 1u] &=
            static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
        return true;
    }

    void TwoHandedGrip::reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode)
    {
        if (_scopeMenuOpenThisFrame || !frik_visual_authority::isAvailable()) {
            return;
        }

        const auto pendingBefore = _scopeDeferredHandAuthorityClears;
        if (pendingBefore[0] == 0 && pendingBefore[1] == 0) {
            return;
        }

        const scope_safe_hand_frame_math::DesiredHandAuthorityInput ownership{
            .gripping = _state == TwoHandedState::Gripping,
            .primaryHandAuthorityEnabled =
                weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode),
            .firingHandIsLeft = _firingHandIsLeft,
            .leftPartGripActive = partGrip(true).active,
            .rightPartGripActive = partGrip(false).active,
        };
        const ScopeHandAuthorityCleanupVisualSnapshot visualSnapshot =
            captureScopeHandAuthorityCleanupVisuals(weaponNode);

        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> cleared{};
        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> retained{};
        bool clearAttempted = false;
        constexpr std::array roles{
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
        };

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            const auto desiredRoles = scope_safe_hand_frame_math::desiredRolesForHand(ownership, isLeft);
            for (const auto role : roles) {
                if (!scope_safe_hand_frame_math::hasRole(_scopeDeferredHandAuthorityClears[index], role)) {
                    continue;
                }

                switch (scope_safe_hand_frame_math::resolveDeferredClearAction(
                    role,
                    desiredRoles,
                    _scopeHandAuthorityPublishedThisFrame[index])) {
                case scope_safe_hand_frame_math::DeferredClearAction::RetainLiveRole:
                    _scopeDeferredHandAuthorityClears[index] &=
                        static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(
                            ~scope_safe_hand_frame_math::roleMask(role));
                    retained[index] |= scope_safe_hand_frame_math::roleMask(role);
                    break;
                case scope_safe_hand_frame_math::DeferredClearAction::ClearStaleRole:
                    clearAttempted = true;
                    if (clearHandAuthorityRoleNow(role, isLeft)) {
                        cleared[index] |= scope_safe_hand_frame_math::roleMask(role);
                    }
                    break;
                case scope_safe_hand_frame_math::DeferredClearAction::WaitForReplacementPublication:
                    break;
                }
            }
        }

        // A stale-tag clear can ask hFRIK to restore an arm. Preserve the
        // already-solved weapon/scope frame; live replacement hand authority
        // was published before this reconciliation and remains selected.
        if (clearAttempted) {
            restoreScopeHandAuthorityCleanupVisuals(visualSnapshot);
        }
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: reconciled deferred native-scope hand authority "
            "left(clear=0x{:02X},retain=0x{:02X},pending=0x{:02X}) "
            "right(clear=0x{:02X},retain=0x{:02X},pending=0x{:02X})",
            static_cast<unsigned>(cleared[0]),
            static_cast<unsigned>(retained[0]),
            static_cast<unsigned>(_scopeDeferredHandAuthorityClears[0]),
            static_cast<unsigned>(cleared[1]),
            static_cast<unsigned>(retained[1]),
            static_cast<unsigned>(_scopeDeferredHandAuthorityClears[1]));
    }

    void TwoHandedGrip::clearAuthoredSupportGripCandidate()
    {
        _authoredSupportGripCandidate = {};
    }

    bool TwoHandedGrip::setAuthoredSupportGripCandidate(
        RE::NiNode* weaponNode,
        const RE::NiTransform& handWeaponLocal,
        const std::array<RE::NiTransform, 15>& fingerLocalTransforms,
        const std::uint16_t fingerLocalTransformMask,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t captureSequence)
    {
        clearAuthoredSupportGripCandidate();
        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            captureSequence == 0 ||
            fingerLocalTransformMask != kCompleteFingerLocalTransformMask ||
            !isFiniteTransform(handWeaponLocal) ||
            std::abs(handWeaponLocal.scale) <= 0.0001f) {
            return false;
        }
        for (const auto& fingerLocal : fingerLocalTransforms) {
            if (!isFiniteTransform(fingerLocal) ||
                std::abs(fingerLocal.scale) <= 0.0001f) {
                return false;
            }
        }

        AuthoredSupportGripCandidate candidate{
            .weaponNode = weaponNode,
            .leftHandWeaponLocal = handWeaponLocal,
            .leftFingerLocalTransforms = fingerLocalTransforms,
            .leftFingerLocalTransformMask = fingerLocalTransformMask,
            .weaponGenerationKey = weaponGenerationKey,
            .captureSequence = captureSequence,
            .valid = true,
        };

        _authoredSupportGripCandidate = candidate;
        refreshAuthoredSupportRightMirror();
        return true;
    }

    void TwoHandedGrip::refreshAuthoredSupportRightMirror()
    {
        auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid || candidate.rightMirrorValid) {
            return;
        }

        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        RE::NiTransform mirroredRightHandWeaponLocal{};
        frik_visual_authority::FingerLocalTransformOverride leftFingerLocals{};
        leftFingerLocals.enabledMask = candidate.leftFingerLocalTransformMask;
        for (std::size_t index = 0;
             index < candidate.leftFingerLocalTransforms.size();
             ++index) {
            leftFingerLocals.localTransforms[index] =
                candidate.leftFingerLocalTransforms[index];
        }

        frik_visual_authority::FingerLocalTransformOverride mirroredRightFingerLocals{};
        const bool rightHandTransformMirrored =
            tryBuildMirroredRightSupportHandWeaponLocal(
                candidate.leftHandWeaponLocal,
                mirroredRightHandWeaponLocal);
        const bool rightFingerPoseMirrored =
            frik_visual_authority::mirrorFingerLocalTransforms(
                frik_visual_authority::Hand::Left,
                leftFingerLocals,
                mirroredRightFingerLocals) &&
            mirroredRightFingerLocals.enabledMask ==
                kCompleteFingerLocalTransformMask;
        bool rightFingerPoseFinite = rightFingerPoseMirrored;
        if (rightFingerPoseFinite) {
            for (const auto& fingerLocal : mirroredRightFingerLocals.localTransforms) {
                if (!isFiniteTransform(fingerLocal) ||
                    std::abs(fingerLocal.scale) <= 0.0001f) {
                    rightFingerPoseFinite = false;
                    break;
                }
            }
        }

        if (rightHandTransformMirrored && rightFingerPoseFinite) {
            candidate.rightHandWeaponLocal = mirroredRightHandWeaponLocal;
            for (std::size_t index = 0;
                 index < candidate.rightFingerLocalTransforms.size();
                 ++index) {
                candidate.rightFingerLocalTransforms[index] =
                    mirroredRightFingerLocals.localTransforms[index];
            }
            candidate.rightFingerLocalTransformMask =
                mirroredRightFingerLocals.enabledMask;
            candidate.rightMirrorValid = true;
            return;
        }

        if (_firingHandIsLeft) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "TwoHandedGrip: authored right-support mirror unavailable transform={} fingers={} naturalFrames=({}, {})",
                rightHandTransformMirrored ? "ready" : "missing",
                rightFingerPoseFinite ? "ready" : "missing",
                _hasLeftNaturalBoneInWand ? "left" : "no-left",
                _hasRightNaturalBoneInWand ? "right" : "no-right");
        }
    }

    bool TwoHandedGrip::tryResolveAuthoredSupportGripCandidateForHand(
        const bool isLeft,
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        RE::NiTransform& outHandWeaponLocal,
        std::array<RE::NiTransform, 15>& outFingerLocalTransforms,
        std::uint16_t& outFingerLocalTransformMask) const
    {
        outHandWeaponLocal = {};
        outFingerLocalTransforms = {};
        outFingerLocalTransformMask = 0;

        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        const auto& candidate = _authoredSupportGripCandidate;
        if (!candidate.valid ||
            !weaponNode ||
            candidate.weaponNode != weaponNode ||
            weaponGenerationKey == 0 ||
            candidate.weaponGenerationKey != weaponGenerationKey) {
            return false;
        }

        if (isLeft) {
            if (candidate.leftFingerLocalTransformMask !=
                kCompleteFingerLocalTransformMask) {
                return false;
            }
            outHandWeaponLocal = candidate.leftHandWeaponLocal;
            outFingerLocalTransforms = candidate.leftFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.leftFingerLocalTransformMask;
        } else {
            if (!candidate.rightMirrorValid ||
                candidate.rightFingerLocalTransformMask !=
                    kCompleteFingerLocalTransformMask) {
                return false;
            }
            outHandWeaponLocal = candidate.rightHandWeaponLocal;
            outFingerLocalTransforms = candidate.rightFingerLocalTransforms;
            outFingerLocalTransformMask = candidate.rightFingerLocalTransformMask;
        }

        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::setAuthoredPrimaryFiringGripCanonical(
        RE::NiNode* weaponNode,
        const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        const std::uint64_t captureSequence, const authored_weapon_grip_library::FiringFingerPose* rightFingerPose,
        const authored_weapon_grip_library::FiringFingerPose* leftFingerPose)
    {
        const auto validFingerPose = [](const authored_weapon_grip_library::FiringFingerPose* pose) {
            if (!pose) {
                return true;
            }
            if (!pose->complete()) {
                return false;
            }
            return std::ranges::all_of(pose->localTransforms, [](const RE::NiTransform& transform) { return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f; });
        };
        if (!weaponNode ||
            weaponGenerationKey == 0 ||
            weaponOwnershipKey == 0 ||
            captureSequence == 0 ||
            !isFiniteTransform(rightHandWeaponLocal) ||
            std::abs(rightHandWeaponLocal.scale) <= 0.0001f || !validFingerPose(rightFingerPose) || !validFingerPose(leftFingerPose) || (leftFingerPose && !rightFingerPose)) {
            return false;
        }

        // HandFrame helpers are frame-agnostic: feeding Hand-in-Weapon yields
        // the configured right palm seat directly in Weapon coordinates. The
        // mirror needs this authored seat, not _primaryGripLocal (which can be
        // a live squeeze or an older native-offset capture).
        const RE::NiPoint3 authoredGripWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                rightHandWeaponLocal,
                false);
        if (!std::isfinite(authoredGripWeaponLocal.x) ||
            !std::isfinite(authoredGripWeaponLocal.y) ||
            !std::isfinite(authoredGripWeaponLocal.z)) {
            return false;
        }

        const std::uint16_t incomingRightFingerMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        const std::uint16_t incomingLeftFingerMask = leftFingerPose ? leftFingerPose->enabledMask : 0;
        const bool fingerPoseBoundary =
            _rightFiringFingerLocalTransformMask != incomingRightFingerMask ||
            _leftFiringFingerLocalTransformMask != incomingLeftFingerMask;
        const bool sourceBoundary =
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            _rightFiringHandCanonicalWeaponNode != weaponNode ||
            _rightFiringHandCanonicalGenerationKey != weaponGenerationKey ||
            _rightFiringHandCanonicalOwnershipKey != weaponOwnershipKey ||
            fingerPoseBoundary;

        _rightFiringHandCanonicalWeaponLocal = rightHandWeaponLocal;
        _rightFiringGripCanonicalWeaponLocal = authoredGripWeaponLocal;
        _rightFiringHandCanonicalWeaponNode = weaponNode;
        _rightFiringHandCanonicalGenerationKey = weaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = weaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = captureSequence;
        _rightFiringHandCanonicalSource =
            RightFiringCanonicalSource::AuthoredAnimation;
        _hasRightFiringHandCanonicalWeaponLocal = true;
        _rightFiringFingerLocalTransforms = rightFingerPose ? rightFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _rightFiringFingerLocalTransformMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        _leftFiringFingerLocalTransforms = leftFingerPose ? leftFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _leftFiringFingerLocalTransformMask = leftFingerPose ? leftFingerPose->enabledMask : 0;

        if (sourceBoundary) {
            ROCK_LOG_INFO(Animation,
                "TwoHandedGrip: authored firing canonical active generation={:016X} ownership={:016X} capture={} handWeaponT=({:.3f},{:.3f},{:.3f}) "
                "gripWeapon=({:.3f},{:.3f},{:.3f}) rightFingerMask=0x{:04X} leftFingerMask=0x{:04X} leftSource=wand-and-anatomy-mirror",
                weaponGenerationKey,
                weaponOwnershipKey,
                captureSequence,
                rightHandWeaponLocal.translate.x,
                rightHandWeaponLocal.translate.y,
                rightHandWeaponLocal.translate.z,
                authoredGripWeaponLocal.x,
                authoredGripWeaponLocal.y,
                authoredGripWeaponLocal.z, _rightFiringFingerLocalTransformMask, _leftFiringFingerLocalTransformMask);
        }
        return true;
    }

    bool TwoHandedGrip::publishAuthoredPrimaryFiringGripFingerPose(const bool isLeft)
    {
        const bool targetHandHoldingObject =
            isLeft ? _leftHandHoldingObjectForPose : _rightHandHoldingObjectForPose;
        if (_authoredPrimaryFingerPoseSuppressed ||
            !authored_weapon_grip_capture_policy::shouldPublishAuthoredFiringFingerPose(
                targetHandHoldingObject) ||
            _rightFiringHandCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        const auto& transforms = isLeft ? _leftFiringFingerLocalTransforms : _rightFiringFingerLocalTransforms;
        const std::uint16_t mask = isLeft ? _leftFiringFingerLocalTransformMask : _rightFiringFingerLocalTransformMask;
        if (mask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }

        if (_authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft != isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }

        if (!_authoredPrimaryFingerPoseBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, true)) {
                return false;
            }
            _authoredPrimaryFingerPoseBlockEngaged = true;
        }

        const auto hand = handFromBool(isLeft);
        _publishedFiringFingerPoseIsLeft = isLeft;
        if (!frik_visual_authority::setHandPoseCustomWithPriority(PRIMARY_GRIP_TAG, hand, frik_visual_authority::HandPoseData{}, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride overrideData{};
        overrideData.enabledMask = mask;
        for (std::size_t index = 0; index < transforms.size(); ++index) {
            overrideData.localTransforms[index] = transforms[index];
        }
        if (!frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(PRIMARY_GRIP_TAG, hand, &overrideData, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        _publishedFiringFingerPoseIsLeft = isLeft;
        _authoredPrimaryFingerPosePublished = true;
        return true;
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripFingerPose()
    {
        if (_authoredPrimaryFingerPosePublished || _authoredPrimaryFingerPoseBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(_publishedFiringFingerPoseIsLeft));
        }
        if (_authoredPrimaryFingerPoseBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, false);
        }
        _publishedFiringFingerPoseIsLeft = false;
        _authoredPrimaryFingerPosePublished = false;
        _authoredPrimaryFingerPoseBlockEngaged = false;
    }

    void TwoHandedGrip::setAuthoredPrimaryFiringGripFingerPoseSuppressed(const bool suppressed)
    {
        _authoredPrimaryFingerPoseSuppressed = suppressed;
        if (suppressed) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::setGrabbedObjectHandPoseOwnership(
        const bool leftHandHoldingObject,
        const bool rightHandHoldingObject)
    {
        _leftHandHoldingObjectForPose = leftHandHoldingObject;
        _rightHandHoldingObjectForPose = rightHandHoldingObject;

        if (!_authoredPrimaryFingerPosePublished) {
            return;
        }

        const bool publishedHandHoldingObject =
            _publishedFiringFingerPoseIsLeft ?
                _leftHandHoldingObjectForPose :
                _rightHandHoldingObjectForPose;
        if (publishedHandHoldingObject) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripCanonical(
        const char* reason)
    {
        if (_rightFiringHandCanonicalSource !=
            RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        ROCK_LOG_DEBUG(Animation,
            "TwoHandedGrip: clearing authored firing canonical reason={} generation={:016X} ownership={:016X} capture={}",
            reason ? reason : "unknown",
            _rightFiringHandCanonicalGenerationKey,
            _rightFiringHandCanonicalOwnershipKey,
            _rightFiringHandCanonicalCaptureSequence);
        clearAuthoredPrimaryFiringGripFingerPose();
        clearRightFiringHandCanonicalFrame();
    }

    bool TwoHandedGrip::applyAuthoredPrimaryGripWeaponAlignment(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (blocksAuthoredPrimaryGripWeaponAlignment() || isWeaponVisualReturnActive()) {
            return false;
        }
        return applyWeaponVisualAuthority(
            weaponNode,
            solvedWeaponWorld,
            currentWeaponGenerationKey);
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
            _nativeScopeAnchorValid && _nativeScopeAnchorWeaponNode == weaponNode && _nativeScopeAnchorGenerationKey == effectiveGenerationKey;

        // Capture hFRIK's engine-specific camera axis only before changing the
        // weapon. Once captured, every hand mode resolves the same immutable
        // generation-bound weapon-local scope frame.
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
        const NativeScopeCameraFollowResult scopeCameraResult =
            rigidFrameMatchesAuthority ?
                applyNativeScopeCameraWorldTarget(
                    scopeCameraFollow,
                    native_scope_camera_follow_math::
                        resolveRigidAnchorFrameWorld(
                            weaponNode->world,
                            _nativeScopeRigidFrame.cameraWeaponLocal)) :
                NativeScopeCameraFollowResult{};
        if (scopeCameraResult.targetValid && scopeCameraResult.writeApplied) {
            (void)applyNativeScopeOverlayTarget(scopeCameraResult.targetCameraWorld, effectiveGenerationKey);
        }
        _lastRenderedWeaponWorld = weaponNode->world;
        _hasLastRenderedWeaponWorld = isFiniteTransform(_lastRenderedWeaponWorld);
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _nativeScopeCameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_nativeScopeCameraDebugSnapshot, effectiveGenerationKey,
                NativeScopeCameraWriteSource::WeaponVisualAuthority, scopeCameraFollow, scopeCameraResult,
                rigidFrameMatchesAuthority ? _nativeScopeAnchorSource : native_scope_sight_anchor_policy::AnchorSource::None);
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
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                !_firingHandIsLeft,
                partGrip(!_firingHandIsLeft));
        const RE::NiTransform appliedFiringHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                firingHandWorld,
                true,
                _primaryHandVisualLerp) :
            resolveLockedHandVisualTarget(
                firingHandWorld,
                acquisitionStart,
                dt,
                _primaryHandVisualLerp);
        (void)publishAuthoredPrimaryFiringGripFingerPose(_firingHandIsLeft);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            PRIMARY_GRIP_TAG, handFromBool(_firingHandIsLeft), appliedFiringHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied) {
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, _firingHandIsLeft);
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
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(isLeft, grip);
        const RE::NiTransform appliedHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                partGripHandWorld,
                false,
                grip.visualLerp) :
            resolveLockedHandVisualTarget(
                partGripHandWorld,
                acquisitionStart,
                dt,
                grip.visualLerp);
        const bool applied = frik_visual_authority::applyExternalHandWorldTransform(
            SUPPORT_GRIP_TAG, handFromBool(isLeft), appliedHandWorld, GRIP_HAND_POSE_PRIORITY);
        if (applied) {
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
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

        ROCK_LOG_WARN(Weapon,
            "TwoHandedGrip: locked hand authority publication failed primary={} support={} "
            "firingHand={} supportHand={} state={} scopeMenu={} primaryFrame={} supportGrip={} supportFrame={}",
            primaryApplied ? "ok" : "failed",
            supportApplied ? "ok" : "failed",
            _firingHandIsLeft ? "left" : "right",
            supportHandIsLeft ? "left" : "right",
            static_cast<int>(_state),
            _scopeMenuOpenThisFrame ? "open" : "closed",
            _hasFiringHandWeaponLocal ? "ready" : "missing",
            partGrip(supportHandIsLeft).active ? "active" : "inactive",
            partGrip(supportHandIsLeft).hasHandWeaponLocal ? "ready" : "missing");

        if (applyPrimaryHand && primaryApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, _firingHandIsLeft);
        }
        if (applySupportHand && supportApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, supportHandIsLeft);
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
        if (_authoredPrimaryFingerPosePublished && _publishedFiringFingerPoseIsLeft == isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        } else {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        }
        if (_scopeMenuOpenThisFrame || _scopeMenuClosedThisFrame) {
            deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, isLeft);
        } else {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, isLeft);
        }
    }

    void TwoHandedGrip::clearPrimaryDetachVisualAuthority(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        if (_scopeMenuOpenThisFrame || _scopeMenuClosedThisFrame) {
            deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach, isLeft);
        } else {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach, isLeft);
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

    void TwoHandedGrip::clearRightFiringHandCanonicalFrame()
    {
        _rightFiringHandCanonicalWeaponLocal = {};
        _rightFiringGripCanonicalWeaponLocal = {};
        _rightFiringHandCanonicalWeaponNode = nullptr;
        _rightFiringHandCanonicalGenerationKey = 0;
        _rightFiringHandCanonicalOwnershipKey = 0;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::None;
        _hasRightFiringHandCanonicalWeaponLocal = false;
        _rightFiringFingerLocalTransforms = {};
        _leftFiringFingerLocalTransforms = {};
        _rightFiringFingerLocalTransformMask = 0;
        _leftFiringFingerLocalTransformMask = 0;
    }

    bool TwoHandedGrip::hasRightFiringHandCanonicalFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode &&
               weaponGenerationKey != 0 &&
               _hasRightFiringHandCanonicalWeaponLocal &&
               _rightFiringHandCanonicalWeaponNode == weaponNode &&
               _rightFiringHandCanonicalGenerationKey == weaponGenerationKey &&
               _rightFiringHandCanonicalOwnershipKey == weaponOwnershipKey &&
               _rightFiringHandCanonicalSource != RightFiringCanonicalSource::None;
    }

    void TwoHandedGrip::rememberRightFiringHandCanonicalFrame()
    {
        if (_firingHandIsLeft || !_activeWeaponNode ||
            !_hasFiringHandWeaponLocal || _activeWeaponGenerationKey == 0) {
            return;
        }
        if (hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }
        _rightFiringHandCanonicalWeaponLocal = _primaryHandWeaponLocal;
        _rightFiringGripCanonicalWeaponLocal = _primaryGripLocal;
        _rightFiringHandCanonicalWeaponNode = _activeWeaponNode;
        _rightFiringHandCanonicalGenerationKey = _activeWeaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = _activeEquippedWeaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    void TwoHandedGrip::refreshNaturalHandInWandFrames()
    {
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return;
        }

        constexpr float kMaxBoneToWandDistance = 30.0f;
        const auto refreshHand = [&](const bool isLeft, RE::NiNode* wandNode, RE::NiTransform& outBoneInWand, bool& outValid) {
            if (!wandNode || hasVisualAuthorityForHand(isLeft) || !isFiniteTransform(wandNode->world)) {
                return;
            }

            RE::NiTransform handWorld{};
            if (!tryGetSolverHandTransform(isLeft, handWorld)) {
                return;
            }
            const RE::NiTransform boneInWand = transform_math::composeTransforms(
                transform_math::invertTransform(wandNode->world), handWorld);
            if (!isFiniteTransform(boneInWand) ||
                std::sqrt(dot(boneInWand.translate, boneInWand.translate)) > kMaxBoneToWandDistance) {
                return;
            }

            outBoneInWand = boneInWand;
            outValid = true;
        };

        refreshHand(false, playerNodes->primaryWandNode, _rightNaturalBoneInWand, _hasRightNaturalBoneInWand);
        refreshHand(true, playerNodes->SecondaryWandNode, _leftNaturalBoneInWand, _hasLeftNaturalBoneInWand);
    }

    void TwoHandedGrip::refreshRightNativeCanonicalFrame(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
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
        // The animation capture is a more direct authority than a later
        // presentation sample. Preserve it for this exact weapon identity,
        // generation, and ownership.
        if (hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        _rightFiringHandCanonicalWeaponLocal = canonicalHold;
        _rightFiringGripCanonicalWeaponLocal = canonicalGrip;
        _rightFiringHandCanonicalWeaponNode = weaponNode;
        _rightFiringHandCanonicalGenerationKey = currentWeaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = currentEquippedWeaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    bool TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal(
        RE::NiTransform& outHandWeaponLocal,
        bool* outUsedAuthoredCanonical,
        const bool logDiagnostic) const
    {
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = false;
        }
        if (!hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey)) {
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

        const bool mirrored = tryBuildMirroredLeftFiringHandWeaponLocal(
            _rightFiringHandCanonicalWeaponLocal,
            _rightFiringGripCanonicalWeaponLocal,
            rightHandWorld,
            leftHandWorld,
            outHandWeaponLocal,
            logDiagnostic);
        if (!mirrored) {
            return false;
        }

        const bool usedAuthoredCanonical =
            _rightFiringHandCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = usedAuthoredCanonical;
        }
        if (logDiagnostic) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: left firing hold resolved source={} generation={:016X} ownership={:016X} capture={} mirroredHandWeaponT=({:.3f},{:.3f},{:.3f})",
                usedAuthoredCanonical ? "authored-animation" : "native-carry",
                _rightFiringHandCanonicalGenerationKey,
                _rightFiringHandCanonicalOwnershipKey,
                _rightFiringHandCanonicalCaptureSequence,
                outHandWeaponLocal.translate.x,
                outHandWeaponLocal.translate.y,
                outHandWeaponLocal.translate.z);
        }
        return true;
    }

    bool TwoHandedGrip::tryBuildMirroredRightSupportHandWeaponLocal(
        const RE::NiTransform& leftHandWeaponLocal,
        RE::NiTransform& outRightHandWeaponLocal) const
    {
        outRightHandWeaponLocal = {};
        if (!_hasLeftNaturalBoneInWand || !_hasRightNaturalBoneInWand ||
            !isFiniteTransform(leftHandWeaponLocal) ||
            !isFiniteTransform(_leftNaturalBoneInWand) ||
            !isFiniteTransform(_rightNaturalBoneInWand)) {
            return false;
        }

        /*
         * Mirror only ORIENTATION through the physical wand pair. The cached
         * bone-in-wand transforms remove hFRIK's asymmetric hand-bone
         * conventions, but their translations/scales are presentation state
         * and can be collapsed while ROCK owns left-primary carry. Feeding
         * those affine values through inverse composition produced enormous
         * intermediate translations and a catastrophically cancelled right
         * support seat. Position is anchored independently below, so rigid
         * zero-origin frames are the complete source authority here.
         */
        const auto orientationFrame = [](const RE::NiTransform& source) {
            RE::NiTransform result = source;
            result.translate = {};
            result.scale = 1.0f;
            return result;
        };
        const RE::NiTransform leftBoneInWandOrientation =
            orientationFrame(_leftNaturalBoneInWand);
        const RE::NiTransform rightBoneInWandOrientation =
            orientationFrame(_rightNaturalBoneInWand);
        const RE::NiTransform leftHandWeaponOrientation =
            orientationFrame(leftHandWeaponLocal);

        RE::NiTransform lateralMirror{};
        lateralMirror.MakeIdentity();
        lateralMirror.rotate.entry[0][0] = -1.0f;

        const RE::NiTransform weaponInLeftWand = transform_math::composeTransforms(
            leftBoneInWandOrientation,
            transform_math::invertTransform(leftHandWeaponOrientation));
        const RE::NiTransform weaponInRightWand = transform_math::composeTransforms(
            lateralMirror,
            transform_math::composeTransforms(weaponInLeftWand, lateralMirror));
        const RE::NiTransform weaponInRightHand = transform_math::composeTransforms(
            transform_math::invertTransform(rightBoneInWandOrientation),
            weaponInRightWand);
        RE::NiTransform mirroredRightHandWeaponLocal = transform_math::invertTransform(weaponInRightHand);
        if (!isFiniteTransform(mirroredRightHandWeaponLocal)) {
            return false;
        }

        /*
         * Position is anchored directly by the actual solver palm seat.
         * Reflect the authored left seat across weapon-local X, clear the
         * orientation solve's translation, then place the right hand from its
         * own palm offset. This avoids subtracting two huge nearly-equal
         * floats and keeps the result weapon-relative and controller-free.
         */
        const RE::NiPoint3 leftPalmWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(leftHandWeaponLocal, true);
        const RE::NiPoint3 desiredRightPalmWeaponLocal{
            -leftPalmWeaponLocal.x,
            leftPalmWeaponLocal.y,
            leftPalmWeaponLocal.z,
        };
        mirroredRightHandWeaponLocal.translate = {};
        mirroredRightHandWeaponLocal.scale = leftHandWeaponLocal.scale;
        const RE::NiPoint3 rightPalmOffsetWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(mirroredRightHandWeaponLocal, false);
        if (!std::isfinite(desiredRightPalmWeaponLocal.x) ||
            !std::isfinite(desiredRightPalmWeaponLocal.y) ||
            !std::isfinite(desiredRightPalmWeaponLocal.z) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.x) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.y) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.z)) {
            return false;
        }
        mirroredRightHandWeaponLocal.translate =
            sub(desiredRightPalmWeaponLocal, rightPalmOffsetWeaponLocal);
        if (!isFiniteTransform(mirroredRightHandWeaponLocal)) {
            return false;
        }

        const RE::NiPoint3 anchoredRightPalmWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                mirroredRightHandWeaponLocal,
                false);
        const RE::NiPoint3 anchorError =
            sub(anchoredRightPalmWeaponLocal, desiredRightPalmWeaponLocal);
        constexpr float kMaxPalmAnchorErrorGameUnits = 0.01f;
        if (!std::isfinite(anchorError.x) ||
            !std::isfinite(anchorError.y) ||
            !std::isfinite(anchorError.z) ||
            std::sqrt(dot(anchorError, anchorError)) >
                kMaxPalmAnchorErrorGameUnits) {
            return false;
        }

        outRightHandWeaponLocal = mirroredRightHandWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
        const RE::NiTransform& canonicalRightHandWeaponLocal,
        const RE::NiPoint3& firingGripWeaponLocal,
        const RE::NiTransform& rightHandWorld,
        const RE::NiTransform& leftHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const bool logDiagnostic)
    {
        const auto& handlingSettings =
            equipped_weapon_handling_runtime::current();
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
        const float aimYawRadians =
            handlingSettings.leftFiringAimYawDegrees * kDegreesToRadiansLocal;
        const float aimPitchRadians =
            handlingSettings.leftFiringAimPitchDegrees * kDegreesToRadiansLocal;
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
            gripInRightHand.x + handlingSettings.leftFiringAimOffsetXGameUnits,
            gripInRightHand.y + handlingSettings.leftFiringAimOffsetYGameUnits,
            -gripInRightHand.z + handlingSettings.leftFiringAimOffsetZGameUnits
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
        const bool supportHandIsLeft = !_firingHandIsLeft;
        if (!weaponNode || !_handlingSettings.ambidextrousHandoffEnabled ||
            !canBeginPrimaryOnlyGripForHand(supportHandIsLeft)) {
            return false;
        }

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        // AttachOnly glue never inherits the firing grip. Likewise, an
        // authored non-touch seat captured under VisualOnlySupport stays
        // presentation-only; a close dynamic touch retains the handoff path.
        // The distance gate below keeps other support grips physically tied
        // to the firing grip before promotion.
        if (!supportGrip.active ||
            supportGrip.attachOnly ||
            !weapon_support_authority_policy::canPromoteSupportGripToFiringGrip(
                _authorityMode,
                supportGrip.authoredSupportGrip)) {
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
            supportGripToFiringGripDistance >
                _handlingSettings.firingGripPromotionRadiusGameUnits) {
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
        bool usedAuthoredCanonical = false;
        const char* holdSource = "live-capture";
        if (supportHandIsLeft) {
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    newFiringHandWeaponLocal,
                    &usedAuthoredCanonical)) {
                usedCanonicalHold = true;
                holdSource = usedAuthoredCanonical ?
                    "authored-mirror" :
                    "native-mirror";
            }
        } else if (hasRightFiringHandCanonicalFrame(
                       _activeWeaponNode,
                       _activeWeaponGenerationKey,
                       _activeEquippedWeaponOwnershipKey)) {
            newFiringHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
            usedCanonicalHold = true;
            holdSource = _rightFiringHandCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" :
                "native-canonical";
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

    bool TwoHandedGrip::getSelectedAuthoredGripPoseSnapshot(
        SelectedAuthoredGripPoseSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        const auto generationKey = _activeWeaponGenerationKey != 0 ?
            _activeWeaponGenerationKey :
            _rightFiringHandCanonicalGenerationKey;
        if (generationKey == 0) {
            return false;
        }

        const bool canonicalCurrent =
            _hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _rightFiringHandCanonicalWeaponNode == _activeWeaponNode);
        const bool supportCurrent =
            _authoredSupportGripCandidate.valid &&
            _authoredSupportGripCandidate.weaponGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _authoredSupportGripCandidate.weaponNode == _activeWeaponNode);
        if (!canonicalCurrent && !supportCurrent) {
            return false;
        }

        outSnapshot.weaponGenerationKey = generationKey;
        if (canonicalCurrent) {
            outSnapshot.rightHandWeaponLocal =
                _rightFiringHandCanonicalWeaponLocal;
            outSnapshot.rightHandValid = true;
            outSnapshot.rightFingerLocalTransforms =
                _rightFiringFingerLocalTransforms;
            outSnapshot.rightFingerLocalTransformMask =
                _rightFiringFingerLocalTransformMask;
            outSnapshot.captureSequence =
                _rightFiringHandCanonicalCaptureSequence;
            outSnapshot.source =
                _rightFiringHandCanonicalSource ==
                        RightFiringCanonicalSource::AuthoredAnimation ?
                    SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest :
                    SelectedAuthoredGripPoseSnapshot::Source::RuntimeCanonical;
        }

        if (supportCurrent) {
            outSnapshot.leftHandWeaponLocal =
                _authoredSupportGripCandidate.leftHandWeaponLocal;
            outSnapshot.leftHandValid = true;
            outSnapshot.leftFingerLocalTransforms =
                _authoredSupportGripCandidate.leftFingerLocalTransforms;
            outSnapshot.leftFingerLocalTransformMask =
                _authoredSupportGripCandidate.leftFingerLocalTransformMask;
            outSnapshot.captureSequence = (std::max)(
                outSnapshot.captureSequence,
                _authoredSupportGripCandidate.captureSequence);
            if (!canonicalCurrent) {
                outSnapshot.rightHandWeaponLocal =
                    _authoredSupportGripCandidate.rightHandWeaponLocal;
                outSnapshot.rightHandValid =
                    _authoredSupportGripCandidate.rightMirrorValid;
                outSnapshot.rightFingerLocalTransforms =
                    _authoredSupportGripCandidate.rightFingerLocalTransforms;
                outSnapshot.rightFingerLocalTransformMask =
                    _authoredSupportGripCandidate.rightFingerLocalTransformMask;
            }
            outSnapshot.source =
                SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest;
        } else if (canonicalCurrent) {
            RE::NiTransform leftHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    leftHandWeaponLocal,
                    nullptr,
                    false)) {
                outSnapshot.leftHandWeaponLocal = leftHandWeaponLocal;
                outSnapshot.leftHandValid = true;
                outSnapshot.leftFingerLocalTransforms =
                    _leftFiringFingerLocalTransforms;
                outSnapshot.leftFingerLocalTransformMask =
                    _leftFiringFingerLocalTransformMask;
            }
        }

        outSnapshot.variantKey = outSnapshot.captureSequence != 0 ?
            outSnapshot.captureSequence :
            generationKey;
        outSnapshot.valid =
            outSnapshot.rightHandValid || outSnapshot.leftHandValid;
        return outSnapshot.valid;
    }

}

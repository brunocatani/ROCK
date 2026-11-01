#pragma once

#include "physics-interaction/hand/SkeletonBoneNameIndex.h"

#include "physics-interaction/weapon/TwoHandedGrip.h"

#include "api/ProviderRuntime.h"
#include "physics-interaction/native/SceneRelation.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponScenePresentation.h"
#include "physics-interaction/weapon/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <limits>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

/*
 * Shared internals of the TwoHandedGrip implementation, split across the
 * grip/, scope/, and telemetry/ translation units. These helpers lived in
 * the anonymous namespace of the former single TwoHandedGrip.cpp; the
 * using-directive below preserves unqualified name lookup for the moved
 * method bodies. Include only from TwoHandedGrip implementation files.
 */

namespace rock
{
    namespace two_handed_grip_internal
    {
        inline constexpr const char* PRIMARY_GRIP_TAG = "ROCK_WeaponPrimaryGrip";
        inline constexpr const char* AUTHORED_PRIMARY_POSE_BLOCK_TAG = "ROCK_AuthoredPrimaryPose";
        inline constexpr const char* PRIMARY_DETACH_TAG = "ROCK_WeaponPrimaryDetach";
        inline constexpr const char* SUPPORT_GRIP_TAG = "ROCK_WeaponSupportGrip";
        inline constexpr const char* RETURN_HAND_TAG = "ROCK_WeaponReturn";
        inline constexpr const char* WEAPON_COLLISION_HAND_TAG =
            "ROCK_WeaponCollisionHand";
        inline constexpr const char* WEAPON_NODE_OWNERSHIP_TAG = "ROCK_LeftFiringCarry";
        // FRIK v2.3 pure write block held while ROCK owns the weapon transform.
        inline constexpr const char* WEAPON_NODE_WRITE_TAG = "ROCK_WeaponAuthority";
        // FRIK v2.3 two-handed grip report (Pip-Boy guards, isOffHandGrippingWeapon).
        inline constexpr const char* TWO_HANDED_GRIP_REPORT_TAG = "ROCK_TwoHandedGrip";
        inline constexpr const char* ONE_HAND_RECOIL_TAG = "ROCK_OneHandRecoil";
        inline constexpr const char* WEAPON_RECOIL_CONTROLLER_TAG = "ROCK_FiringGripRecoil";
        inline constexpr int GRIP_HAND_POSE_PRIORITY = 100;
        // The right hand's native-physical base (ROCK_WeaponFiltering). It
        // stays below the equip bridge and drop visual poses (99): a tie goes
        // to the newest claim, and on a right trigger equip's first grip frame
        // the base re-registered and replaced the bridge pose for one frame.
        inline constexpr int NATIVE_PHYSICAL_HAND_PRIORITY = 98;
        inline constexpr int WEAPON_COLLISION_HAND_PRIORITY = 110;
        inline constexpr int RETURN_HAND_VISUAL_PRIORITY = 85;
        inline constexpr float SUPPORT_NORMAL_TWIST_FACTOR = 0.5f;
        inline constexpr float DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS =
            0.5f * 0.01745329251994329577f;
        inline constexpr float RADIANS_TO_DEGREES = 57.295779513082320876f;
        /*
         * Clock domain: a consecutive-publication count by design. The hFRIK
         * driver either publishes each frame or it does not; the tolerance is
         * a number of missed publications, not an elapsed duration.
         */
        inline constexpr std::uint32_t SCOPE_DRIVER_MISS_GRACE_FRAMES = 3;
        inline constexpr float SCOPE_ROOT_REBASE_DURATION_SECONDS = 0.075f;

        inline const char* twoHandedStateDiagnosticName(
            const TwoHandedState state)
        {
            switch (state) {
            case TwoHandedState::Inactive:
                return "inactive";
            case TwoHandedState::Touching:
                return "touching";
            case TwoHandedState::Gripping:
                return "gripping";
            case TwoHandedState::PartCarry:
                return "part-carry";
            case TwoHandedState::PrimaryOnly:
                return "primary-only";
            }
            return "unknown";
        }

        inline const char* supportAuthorityDiagnosticName(
            const weapon_support_authority_policy::
                WeaponSupportAuthorityMode mode)
        {
            switch (mode) {
            case weapon_support_authority_policy::
                WeaponSupportAuthorityMode::FullTwoHandedSolver:
                return "full";
            case weapon_support_authority_policy::
                WeaponSupportAuthorityMode::VisualOnlySupport:
                return "visual-only";
            }
            return "unknown";
        }

        inline const char* weaponInteractionAcquisitionSourceName(
            const WeaponInteractionAcquisitionSource source)
        {
            switch (source) {
            case WeaponInteractionAcquisitionSource::PhysicalContact:
                return "contact";
            case WeaponInteractionAcquisitionSource::ProximityProbe:
                return "probe";
            case WeaponInteractionAcquisitionSource::AuthoredSeat:
                return "authored-seat";
            case WeaponInteractionAcquisitionSource::FiringGripZone:
                return "firing-grip-zone";
            case WeaponInteractionAcquisitionSource::None:
                return "none";
            }
            return "unknown";
        }

        /*
         * The part-carry two-anchor solve feeds its own rotation back as the
         * next frame's base, chaining several float matrix products per frame.
         * Without re-orthonormalization the rotation's row norms decay and the
         * matrix acquires shear, which visibly stretches the weapon mesh and
         * collapses the grip geometry (telemetry: rigid grip separation decayed
         * ~0.05% per frame). Rows are the stored local axes.
         */
        inline RE::NiMatrix3 orthonormalizeStoredRotation(const RE::NiMatrix3& rotation)
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
         * Ambidextrous firing grip (left-hand fire) needs the finger-pose
         * block (right hand must stop receiving FRIK's weapon pose), the
         * weapon-node write block (FRIK must stop writing the weapon node)
         * and, since FRIK API v2.3, the explicit parent-hand request (FRIK
         * re-parents the node under LArm_Hand and keeps its own bookkeeping).
         * Fail closed to right-only behavior when any is missing.
         */
        inline bool leftFiringInfrastructureAvailable()
        {
            return frik_visual_authority::canBlockPrimaryHandWeaponPose() &&
                frik_visual_authority::canBlockPrimaryWeaponNodeOwnership() &&
                frik_visual_authority::canSetWeaponNodeParentHand();
        }

        inline RE::NiNode* sourceRootNodeOrFallback(RE::NiAVObject* sourceRoot, RE::NiNode* fallback)
        {
            if (sourceRoot) {
                if (auto* sourceNode = sourceRoot->IsNode()) {
                    return sourceNode;
                }
            }
            return fallback;
        }

        inline RE::NiPoint3 lerpPoint(const RE::NiPoint3& from, const RE::NiPoint3& to, float alpha)
        {
            const float t = (std::max)(0.0f, (std::min)(1.0f, alpha));
            return RE::NiPoint3{ from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t, from.z + (to.z - from.z) * t };
        }

        inline bool isFiniteRotation(const RE::NiMatrix3& rotation)
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

        inline bool isFiniteTransform(const RE::NiTransform& transform)
        {
            return isFiniteRotation(transform.rotate) && std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
                   std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
        }

        inline bool isInvertibleTransform(const RE::NiTransform& transform)
        {
            return isFiniteTransform(transform) &&
                std::abs(transform.scale) > 0.0001f;
        }

        inline RE::NiTransform makeLeftFiringWandAimTrim(
            const EquippedWeaponHandlingSettings& handlingSettings)
        {
            return left_firing_position_only_math::leftAimTrim<RE::NiTransform>(
                handlingSettings.leftFiringAimYawDegrees, handlingSettings.leftFiringAimPitchDegrees);
        }

        using weapon_scene_presentation::restoreWeaponRootPreservingPresentedDescendants;
        using weapon_scene_presentation::moveWeaponPresentationRigidly;

        inline bool areTransformsNearlyEqual(const RE::NiTransform& lhs, const RE::NiTransform& rhs, const float epsilon = 0.001f)
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

        inline bool arePointsNearlyEqual(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs, const float epsilon = 0.001f)
        {
            return std::abs(lhs.x - rhs.x) <= epsilon &&
                   std::abs(lhs.y - rhs.y) <= epsilon &&
                   std::abs(lhs.z - rhs.z) <= epsilon;
        }

        inline bool tryGetComposedNodeWorld(const RE::NiAVObject* node, RE::NiTransform& outWorld)
        {
            if (!node) {
                return false;
            }
            outWorld = node->parent ?
                transform_math::composeTransforms(node->parent->world, node->local) :
                node->world;
            return isFiniteTransform(outWorld);
        }

        inline bool isUsableHandAuthorityTransform(const RE::NiTransform& transform)
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

        [[nodiscard]] inline bool resolveAuthoredSupportPalmSeatProximity(
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
                computeGrabPalmPointWorldFromHandBasis(
                    authoredHandWeaponLocal,
                    isLeft);
            out.authoredPalmSeatWorld = transform_math::localPointToWorld(
                weaponWorld,
                out.authoredPalmSeatWeaponLocal);
            out.liveTouchProbeWorld =
                computeGrabPalmPointWorldFromHandBasis(
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

        /*
         * Capsule-sweep index over the equipped weapon's own mesh parts. Each
         * part already carries a triangle index built at equip time, off the
         * frame path, so a support grip queries the whole weapon mesh with no
         * triangle copy, no per-grab index build and no triangle cap (detailed
         * rifles hold 90k+ triangles within the hand's reach). build() keeps
         * only parts whose bounds reach the sweep sphere. The parts are
         * non-owning views into the weapon body bank: valid only inside the
         * capture call that built them, which clears them before returning.
         */
        class SupportGripWeaponPartIndex
        {
        public:
            void clear()
            {
                _parts.clear();
                _triangleCount = 0;
            }

            // Returns the number of parts skipped because they carry no index.
            std::size_t build(std::span<const WeaponCollision::SupportGripEvidenceView> views, const RE::NiTransform& weaponWorld,
                const RE::NiPoint3& reachCenterWorld, float reachRadiusWorld)
            {
                clear();
                if (!weapon_support_acquisition_math::isUsableTransform(weaponWorld) || !grab_finger_pose_runtime::isFinitePoint(reachCenterWorld) ||
                    !std::isfinite(reachRadiusWorld) || reachRadiusWorld <= 0.0f) {
                    return 0;
                }
                const RE::NiTransform worldToWeapon = transform_math::invertTransform(weaponWorld);
                const RE::NiPoint3 reachCenter = transform_math::localPointToWorld(worldToWeapon, reachCenterWorld);
                const float reachRadius = reachRadiusWorld / std::abs(weaponWorld.scale);
                std::size_t unindexed = 0;
                for (const auto& view : views) {
                    if (view.localTriangles.empty() || !weapon_support_acquisition_math::isUsableTransform(view.localToWorld)) {
                        continue;
                    }
                    RE::NiPoint3 boundsMin{};
                    RE::NiPoint3 boundsMax{};
                    if (!view.localTriangleIndex) {
                        ++unindexed;
                        continue;
                    }
                    if (!view.localTriangleIndex->bounds(boundsMin, boundsMax)) {
                        continue;
                    }
                    const RE::NiTransform partToWeapon = transform_math::composeTransforms(worldToWeapon, view.localToWorld);
                    if (!weapon_support_acquisition_math::isUsableTransform(partToWeapon)) {
                        continue;
                    }
                    const RE::NiPoint3 halfExtent = (boundsMax - boundsMin) * 0.5f;
                    const RE::NiPoint3 boundsCenter = transform_math::localPointToWorld(partToWeapon, (boundsMin + boundsMax) * 0.5f);
                    const float boundsRadius = std::sqrt(halfExtent.x * halfExtent.x + halfExtent.y * halfExtent.y + halfExtent.z * halfExtent.z) *
                                               std::abs(partToWeapon.scale);
                    const RE::NiPoint3 offset = boundsCenter - reachCenter;
                    if (!(std::sqrt(offset.x * offset.x + offset.y * offset.y + offset.z * offset.z) <= boundsRadius + reachRadius)) {
                        continue;
                    }
                    Part part{};
                    part.triangles = view.localTriangles;
                    part.index = view.localTriangleIndex;
                    part.weaponToPart = transform_math::invertTransform(partToWeapon);
                    part.boundsCenter = boundsCenter;
                    part.boundsRadius = boundsRadius;
                    _parts.push_back(part);
                    _triangleCount += view.localTriangles.size();
                }
                return unindexed;
            }

            // The capsule sweep's Index contract. Only the yes/no answer is
            // supported: the closest point and normal outputs must be null.
            [[nodiscard]] bool querySphereWorld(const RE::NiTransform& objectWorld, const RE::NiPoint3& centerWorld, float radiusWorld, std::nullptr_t,
                std::nullptr_t, grab_finger_pose_runtime::FingerPoseSpatialQueryStats* stats = nullptr) const
            {
                if (_parts.empty() || !grab_finger_pose_runtime::isFinitePoint(centerWorld) || !std::isfinite(radiusWorld) || radiusWorld <= 0.0f ||
                    !std::isfinite(objectWorld.scale) || std::abs(objectWorld.scale) <= 0.000001f) {
                    return false;
                }
                const RE::NiPoint3 center = transform_math::worldPointToLocal(objectWorld, centerWorld);
                const float radius = radiusWorld / std::abs(objectWorld.scale);
                const auto distanceSquared = [](const RE::NiPoint3& point, const TriangleData& triangle) {
                    float result = 0.0f;
                    (void)closestPointOnTriangleToPoint(point, triangle, result);
                    return result;
                };
                for (const auto& part : _parts) {
                    const RE::NiPoint3 offset = part.boundsCenter - center;
                    const float reach = part.boundsRadius + radius;
                    if (offset.x * offset.x + offset.y * offset.y + offset.z * offset.z > reach * reach) {
                        continue;
                    }
                    const float partRadius = radius * std::abs(part.weaponToPart.scale);
                    WeaponTriangleIndex::QueryStats partStats{};
                    const bool hit = part.index->anyTriangleWithin(part.triangles, transform_math::localPointToWorld(part.weaponToPart, center),
                        partRadius * partRadius, distanceSquared, stats ? &partStats : nullptr);
                    if (stats) {
                        stats->nodeVisits += partStats.nodes;
                        stats->triangleTests += partStats.triangles;
                    }
                    if (hit) {
                        return true;
                    }
                }
                return false;
            }

            [[nodiscard]] std::size_t partCount() const noexcept { return _parts.size(); }
            [[nodiscard]] std::size_t triangleCount() const noexcept { return _triangleCount; }

        private:
            struct Part
            {
                std::span<const TriangleData> triangles{};
                const WeaponTriangleIndex* index{ nullptr };
                RE::NiTransform weaponToPart{};
                RE::NiPoint3 boundsCenter{};
                float boundsRadius{ 0.0f };
            };

            std::vector<Part> _parts;
            std::size_t _triangleCount{ 0 };
        };

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

        inline ScopeHandAuthorityCleanupVisualSnapshot captureScopeHandAuthorityCleanupVisuals(RE::NiNode* weaponNode)
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

        inline void restoreScopeHandAuthorityCleanupVisuals(const ScopeHandAuthorityCleanupVisualSnapshot& snapshot)
        {
            if (snapshot.weaponValid && snapshot.weapon) {
                (void)restoreWeaponRootPreservingPresentedDescendants(
                    snapshot.weapon,
                    snapshot.weaponWorld);
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

        inline NativeScopeCameraFollowCapture captureNativeScopeCameraFollow(const RE::NiNode* weaponNode)
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

        inline NativeScopeCameraFollowResult applyNativeScopeCameraWorldTarget(const NativeScopeCameraFollowCapture& capture, const RE::NiTransform& targetCameraWorld, NativeScopeRigidFrameState& authority)
        {
            NativeScopeCameraFollowResult result{};
            if (!g_rockConfig.rockEnableImmersiveScopes || !capture.valid || !capture.camera ||
                capture.camera != authority.scopeCameraIdentity || !isFiniteTransform(targetCameraWorld)) {
                return result;
            }
            result.targetCameraWorld = targetCameraWorld;
            result.targetValid = true;

            auto* scopeCamera = capture.camera;
            if (!authority.hasAppliedLocal || authority.cameraParentIdentity != scopeCamera->parent ||
                !areTransformsNearlyEqual(scopeCamera->local, authority.lastAppliedCameraLocal)) {
                authority.nativeCameraLocal = scopeCamera->local;
                authority.cameraParentIdentity = scopeCamera->parent;
            }
            if (scopeCamera->parent) {
                const RE::NiTransform targetCameraLocal = weapon_visual_authority_math::worldTargetToParentLocal(
                    scopeCamera->parent->world,
                    targetCameraWorld);
                if (!isFiniteTransform(targetCameraLocal)) {
                    return result;
                }
                scopeCamera->local = targetCameraLocal;
                authority.lastAppliedCameraLocal = scopeCamera->local;
                authority.hasAppliedLocal = true;
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
            authority.lastAppliedCameraLocal = scopeCamera->local;
            authority.hasAppliedLocal = true;
            scopeCamera->world = targetCameraWorld;
            result.writeApplied = true;
            result.immediateCameraWorldAfter = scopeCamera->world;
            result.immediateReadbackValid = isFiniteTransform(result.immediateCameraWorldAfter);
            return result;
        }

        inline NativeScopeCameraDebugSnapshot makeNativeScopeCameraDebugSnapshot(const NativeScopeCameraDebugSnapshot& previous, const std::uint64_t weaponGenerationKey,
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

        inline DirectSkeletonBoneReader& rootFlattenedTwoHandedReader()
        {
            static DirectSkeletonBoneReader reader;
            return reader;
        }

        inline const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
        {
            for (const auto& bone : snapshot.bones) {
                if (bone.name == name) {
                    return &bone;
                }
            }
            return nullptr;
        }

        /*
         * The controller hand for this frame. Under FRIK API v2 the root
         * flattened bone shows ROCK's previous claim while one is active, so
         * physical-intent readers take the isolated controller hand the
         * PhysicsInteraction frame resolved before the grip update.
         */
        inline bool tryGetRootFlattenedHandBoneTransform(bool isLeft, RE::NiTransform& outTransform)
        {
            outTransform = {};
            RE::NiTransform rawHandWorld{};
            if (!frik_hand_world_authority::tryGetRawHandWorld(isLeft, rawHandWorld) ||
                !isUsableHandAuthorityTransform(rawHandWorld)) {
                return false;
            }
            outTransform = rawHandWorld;
            return true;
        }
    }

    // Implementation TUs resolve the helper names unqualified, exactly as
    // they did inside the original anonymous namespace.
    using namespace two_handed_grip_internal;

    struct TwoHandedGrip::FingerPoseSolveScratch
    {
        struct HandScratch
        {
            DirectSkeletonBoneSnapshot capturedBones;
            SkeletonBoneNameIndex capturedBoneNames;
            SupportGripWeaponPartIndex partIndex;
            // Triangles of the weapon parts the last support solve could reach; for the capture log.
            std::size_t reachTriangleCount{ 0 };
        };

        std::array<HandScratch, 2> hands{};
    };
}

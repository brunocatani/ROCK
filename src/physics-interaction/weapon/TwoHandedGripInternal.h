#pragma once

/*
 * Helpers shared by the TwoHandedGrip translation units.
 *
 * Keep one-file helpers in that file's anonymous namespace. Promote a helper
 * here only when more than one translation unit owns part of its flow.
 *
 * INTERNAL. Never include this file from a public include tree.
 */

#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/WeaponCollision.h"

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace rock::provider
{
    struct RockProviderWeaponPartTargetQueryV1;
}

namespace rock::two_handed_grip_detail
{
    inline constexpr const char* PRIMARY_GRIP_TAG =
        "ROCK_WeaponPrimaryGrip";
    inline constexpr const char* AUTHORED_PRIMARY_POSE_BLOCK_TAG =
        "ROCK_AuthoredPrimaryPose";
    inline constexpr const char* PRIMARY_DETACH_TAG =
        "ROCK_WeaponPrimaryDetach";
    inline constexpr const char* SUPPORT_GRIP_TAG =
        "ROCK_WeaponSupportGrip";
    inline constexpr const char* GUNSTOCK_ALIGNMENT_TAG =
        "ROCK_GunstockAlignment";
    inline constexpr const char* RETURN_HAND_TAG =
        "ROCK_WeaponReturn";
    inline constexpr const char* WEAPON_COLLISION_HAND_TAG =
        "ROCK_WeaponCollisionHand";
    inline constexpr const char* WEAPON_NODE_OWNERSHIP_TAG =
        "ROCK_LeftFiringCarry";
    inline constexpr const char* WEAPON_RECOIL_CONTROLLER_TAG =
        "ROCK_FiringGripRecoil";

    inline constexpr int GRIP_HAND_POSE_PRIORITY = 100;
    inline constexpr int WEAPON_COLLISION_HAND_PRIORITY = 110;
    inline constexpr int RETURN_HAND_VISUAL_PRIORITY = 85;

    inline constexpr float kDegreesToRadians =
        0.01745329251994329577f;
    inline constexpr float kRadiansToDegrees =
        57.295779513082320876f;
    inline constexpr std::size_t kSupportGripFingerLaneCount = 5;
    inline constexpr std::size_t kSupportGripFingerLaneReferenceCapacity = 10;
    inline constexpr std::size_t kSupportGripGlobalRankingIndex =
        kSupportGripFingerLaneCount;

    // Reject a hand frame that cannot belong to its current VR driver.
    inline constexpr float kMaximumBoneToDriverDistanceGameUnits = 30.0f;
    inline constexpr float SUPPORT_NORMAL_TWIST_FACTOR = 0.5f;
    inline constexpr float DYNAMIC_SUPPORT_MINIMUM_SMOOTHED_ROTATION_RADIANS =
        0.5f * kDegreesToRadians;
    inline constexpr std::uint32_t SCOPE_DRIVER_MISS_GRACE_FRAMES = 3;
    inline constexpr float SCOPE_ROOT_REBASE_DURATION_SECONDS = 0.075f;
    inline constexpr std::uint32_t SCOPE_TRANSITION_TRACE_FRAMES = 6;
    inline constexpr float WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS =
        24.0f;
    inline constexpr float WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS =
        0.5f;
    inline constexpr std::size_t WEAPON_PRESENTATION_MAX_DEPTH = 128;
    inline constexpr std::size_t WEAPON_PRESENTATION_MAX_OBJECTS = 8192;

    // A stable ordinal makes equal-distance triangle selection deterministic.
    struct RankedSupportGripTriangle
    {
        float distanceSquared = 0.0f;
        std::uint64_t deterministicOrdinal = 0;
        TriangleData weaponLocalTriangle{};
    };

    // Each finger lane keeps its own world-space solve references.
    struct SupportGripFingerReferenceSet
    {
        RE::NiPoint3 seatPointWorld{};
        std::array<
            std::array<
                RE::NiPoint3,
                kSupportGripFingerLaneReferenceCapacity>,
            kSupportGripFingerLaneCount>
            lanePointsWorld{};
        std::array<std::size_t, kSupportGripFingerLaneCount>
            lanePointCounts{};
        bool seatPointValid{ false };
    };

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

    [[nodiscard]] grab_pinch_pocket_policy::Config
        currentWeaponOppositionPocketConfig();
    [[nodiscard]] gunstock_alignment_policy::FineTuneDegrees
        configuredGunstockFineTune();
    [[nodiscard]] RE::NiMatrix3 orthonormalizeStoredRotation(
        const RE::NiMatrix3& rotation);
    [[nodiscard]] bool leftFiringInfrastructureAvailable();
    [[nodiscard]] provider::RockProviderWeaponPartTargetQueryV1
        buildProviderPartTargetQuery(
            std::uint64_t weaponGenerationKey,
            std::uint32_t bodyId,
            std::uint32_t partKind,
            std::uint32_t reloadRole,
            std::uint32_t supportRole,
            std::uint32_t socketRole,
            std::uint32_t actionRole,
            std::uintptr_t sourceRoot,
            std::span<const char> sourceName);

    [[nodiscard]] RE::NiPoint3 lerpPoint(
        const RE::NiPoint3& from,
        const RE::NiPoint3& to,
        float alpha);
    [[nodiscard]] bool isFiniteRotation(const RE::NiMatrix3& rotation);
    [[nodiscard]] bool isFiniteTransform(const RE::NiTransform& transform);
    [[nodiscard]] bool isInvertibleTransform(
        const RE::NiTransform& transform);
    [[nodiscard]] bool areTransformsNearlyEqual(
        const RE::NiTransform& lhs,
        const RE::NiTransform& rhs,
        float epsilon = 0.001f);
    [[nodiscard]] bool arePointsNearlyEqual(
        const RE::NiPoint3& lhs,
        const RE::NiPoint3& rhs,
        float epsilon = 0.001f);
    [[nodiscard]] bool isUsableHandAuthorityTransform(
        const RE::NiTransform& transform);

    [[nodiscard]] bool validateWeaponPresentationSubtree(
        RE::NiAVObject* object,
        const RE::NiTransform& presentationWorldDelta,
        std::size_t depth,
        std::size_t& objectCount);
    void applyWeaponPresentationDeltaToDescendants(
        RE::NiNode* parent,
        const RE::NiTransform& presentationWorldDelta);
    [[nodiscard]] bool tryResolveWeaponRootLocal(
        const RE::NiNode* parent,
        const RE::NiTransform& weaponWorld,
        RE::NiTransform& outWeaponLocal);
    [[nodiscard]] bool restoreWeaponRootPreservingPresentedDescendants(
        RE::NiNode* weaponNode,
        const RE::NiTransform& weaponWorld);
    [[nodiscard]] bool moveWeaponPresentationRigidly(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld);

    [[nodiscard]] bool resolveAuthoredSupportPalmSeatProximityFromPoints(
        const RE::NiTransform& weaponWorld,
        const RE::NiPoint3& liveTouchProbeWorld,
        const RE::NiPoint3& authoredPalmSeatWeaponLocal,
        AuthoredSupportPalmSeatProximity& out);
    [[nodiscard]] bool resolveAuthoredSupportPalmSeatProximity(
        const RE::NiTransform& weaponWorld,
        const RE::NiTransform& liveHandWorld,
        const RE::NiTransform& authoredHandWeaponLocal,
        bool isLeft,
        AuthoredSupportPalmSeatProximity& out);
    [[nodiscard]] authored_weapon_grip_activation_policy::DirectionGateResult
        evaluateAuthoredSupportGripDirectionGate(
            AuthoredSupportGripDebugSnapshot& snapshot,
            const RE::NiPoint3& lastStableDirectionWorld,
            bool lastStableDirectionValid,
            bool rightFiringLeftSupportScope);

    void selectNearestSupportGripFingerTriangles(
        std::span<const WeaponCollision::SupportGripEvidenceView> evidenceViews,
        const RE::NiTransform& weaponWorld,
        const SupportGripFingerReferenceSet& referenceSet,
        std::size_t maxTriangles,
        std::array<std::vector<RankedSupportGripTriangle>,
            kSupportGripFingerLaneCount + 1>& rankingScratch,
        std::vector<TriangleData>& outTriangles);

    [[nodiscard]] ScopeHandAuthorityCleanupVisualSnapshot
        captureScopeHandAuthorityCleanupVisuals(RE::NiNode* weaponNode);
    void restoreScopeHandAuthorityCleanupVisuals(
        const ScopeHandAuthorityCleanupVisualSnapshot& snapshot);
    [[nodiscard]] NativeScopeCameraFollowCapture
        captureNativeScopeCameraFollow(const RE::NiNode* weaponNode);
    [[nodiscard]] NativeScopeCameraFollowResult
        applyNativeScopeCameraWorldTarget(
            const NativeScopeCameraFollowCapture& capture,
            const RE::NiTransform& targetCameraWorld);
    [[nodiscard]] NativeScopeCameraDebugSnapshot
        makeNativeScopeCameraDebugSnapshot(
            const NativeScopeCameraDebugSnapshot& previous,
            std::uint64_t weaponGenerationKey,
            NativeScopeCameraWriteSource writeSource,
            const NativeScopeCameraFollowCapture& capture,
            const NativeScopeCameraFollowResult& result,
            native_scope_sight_anchor_policy::AnchorSource anchorSource);

    [[nodiscard]] DirectSkeletonBoneReader& rootFlattenedTwoHandedReader();
    [[nodiscard]] bool tryGetRootFlattenedHandBoneTransform(
        bool isLeft,
        RE::NiTransform& outTransform);
    [[nodiscard]] bool buildFullHandLocalTransformsForMeshPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& meshFingerPose,
        const frik_visual_authority::HandPoseData& handPose,
        const DirectSkeletonBoneSnapshot* capturedFingerSnapshot,
        std::array<RE::NiTransform, 15>& outLocalTransforms,
        std::uint16_t& outMask);
}

namespace rock
{
    // Capture owns this bounded scratch across re-grabs to avoid hot-path churn.
    struct TwoHandedGrip::FingerPoseSolveScratch
    {
        struct HandScratch
        {
            std::array<std::vector<two_handed_grip_detail::RankedSupportGripTriangle>,
                two_handed_grip_detail::kSupportGripFingerLaneCount + 1>
                rankings;
            std::vector<TriangleData> localTriangles;
            std::vector<TriangleData> worldTriangles;
            grab_finger_pose_runtime::FingerPoseTriangleSpatialIndex spatialIndex;
        };

        std::array<HandScratch, 2> hands{};
    };
}

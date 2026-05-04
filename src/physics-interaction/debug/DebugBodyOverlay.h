#pragma once

#include <array>
#include <cstdint>

#include "physics-interaction/weapon/WeaponTypes.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/hand/HandColliderTypes.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class hknpWorld;
}

namespace frik::rock::debug
{
    enum class BodyOverlayRole : std::uint8_t
    {
        RightHand,
        LeftHand,
        RightHandSegment,
        LeftHandSegment,
        Weapon,
        Target
    };

    enum class AxisOverlayRole : std::uint8_t
    {
        RightHandRaw,
        LeftHandRaw,
        RightHandBody,
        LeftHandBody,
        TargetBody,
        WeaponAuthority,
        RightWeaponPrimaryGrip,
        LeftWeaponSupportGrip,
        RightFrikAppliedHand,
        LeftFrikAppliedHand,
        RightGrabHiggsReverse,
        LeftGrabHiggsReverse,
        RightGrabConstraintReverse,
        LeftGrabConstraintReverse,
        RightGrabRockVisualTarget,
        LeftGrabRockVisualTarget,
        RightGrabDesiredObject,
        LeftGrabDesiredObject,
        RightGrabHeldNode,
        LeftGrabHeldNode
    };

    enum class AxisOverlaySource : std::uint8_t
    {
        Transform,
        Body
    };

    enum class MarkerOverlayRole : std::uint8_t
    {
        RightGrabAnchor,
        LeftGrabAnchor,
        RightPalmNormal,
        LeftPalmNormal,
        RightPointing,
        LeftPointing,
        RightGrabPivotA,
        LeftGrabPivotA,
        RightGrabPivotB,
        LeftGrabPivotB,
        RightGrabPivotError,
        LeftGrabPivotError,
        RightGrabSurfacePoint,
        LeftGrabSurfacePoint,
        RightGrabSurfaceNormal,
        LeftGrabSurfaceNormal,
        RightGrabSurfaceTangent,
        LeftGrabSurfaceTangent,
        RightGrabSurfaceBitangent,
        LeftGrabSurfaceBitangent,
        RightGrabContactPatchSample,
        LeftGrabContactPatchSample,
        RightGrabFingerProbe,
        LeftGrabFingerProbe,
        RightHandBoneContact,
        LeftHandBoneContact,
        RightWeaponPrimaryGrip,
        LeftWeaponSupportGrip,
        RightWeaponAuthorityMismatch,
        LeftWeaponAuthorityMismatch,
        RightRootFlattenedFingerSkeleton,
        LeftRootFlattenedFingerSkeleton,
        TargetVisualOrigin,
        TargetRawBodyOrigin,
        TargetBodyTransformOrigin,
        TargetMotionOrigin,
        TargetBestOriginCandidate,
        TargetOriginErrorLine,
        RightGrabHiggsReverseError,
        LeftGrabHiggsReverseError,
        RightGrabConstraintReverseError,
        LeftGrabConstraintReverseError,
        RightGrabRockVisualError,
        LeftGrabRockVisualError,
        RightGrabHeldDesiredError,
        LeftGrabHeldDesiredError,
        RightGrabTelemetryLabelAnchor,
        LeftGrabTelemetryLabelAnchor
    };

    enum class SkeletonOverlayRole : std::uint8_t
    {
        Core,
        Head,
        RightArm,
        LeftArm,
        RightFinger,
        LeftFinger,
        RightLeg,
        LeftLeg
    };

    struct BodyOverlayEntry
    {
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        BodyOverlayRole role{ BodyOverlayRole::Target };
    };

    struct AxisOverlayEntry
    {
        AxisOverlaySource source{ AxisOverlaySource::Transform };
        AxisOverlayRole role{ AxisOverlayRole::TargetBody };
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        RE::NiTransform transform{};
        RE::NiPoint3 translationStart{};
        bool drawTranslationLine{ false };
    };

    struct MarkerOverlayEntry
    {
        MarkerOverlayRole role{ MarkerOverlayRole::RightGrabAnchor };
        RE::NiPoint3 position{};
        RE::NiPoint3 lineEnd{};
        float size{ 2.0f };
        bool drawPoint{ true };
        bool drawLine{ false };
    };

    struct SkeletonOverlayEntry
    {
        SkeletonOverlayRole role{ SkeletonOverlayRole::Core };
        RE::NiTransform transform{};
        RE::NiPoint3 parentPosition{};
        float pointSize{ 1.4f };
        float axisLength{ 4.0f };
        bool hasParent{ false };
        bool drawPoint{ true };
        bool drawAxis{ true };
        bool inPowerArmor{ false };
    };

    struct TextOverlayEntry
    {
        char text[128]{};
        float x{ 18.0f };
        float y{ 18.0f };
        float size{ 2.0f };
        float color[4]{ 0.90f, 1.0f, 0.95f, 0.92f };
        RE::NiPoint3 worldAnchor{};
        bool worldAnchored{ false };
    };

    struct BodyOverlayFrame
    {
        RE::hknpWorld* world{ nullptr };
        std::array<BodyOverlayEntry, MAX_WEAPON_COLLISION_BODIES + (hand_collider_semantics::kHandColliderBodyCountPerHand * 2) + 8> entries{};
        std::array<AxisOverlayEntry, 24> axisEntries{};
        std::array<MarkerOverlayEntry, 192> markerEntries{};
        std::array<SkeletonOverlayEntry, skeleton_bone_debug_math::skeletonOverlayBudget()> skeletonEntries{};
        std::array<TextOverlayEntry, 24> textEntries{};
        std::uint32_t count{ 0 };
        std::uint32_t axisCount{ 0 };
        std::uint32_t markerCount{ 0 };
        std::uint32_t skeletonCount{ 0 };
        std::uint32_t textCount{ 0 };
        bool drawRockBodies{ false };
        bool drawTargetBodies{ false };
        bool drawAxes{ false };
        bool drawMarkers{ false };
        bool drawSkeleton{ false };
        bool drawText{ false };
    };

    void Install();
    bool IsInstalled();
    void PublishFrame(const BodyOverlayFrame& frame);
    void ClearFrame();
    void ClearShapeCache();
}

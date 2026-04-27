#pragma once

#include <array>
#include <cstdint>

#include "WeaponCollisionLimits.h"

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
        Weapon,
        Target
    };

    enum class AxisOverlayRole : std::uint8_t
    {
        RightHandRaw,
        LeftHandRaw,
        RightHandCollider,
        LeftHandCollider,
        RightHandBody,
        LeftHandBody,
        TargetBody
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
        RightGrabFingerProbe,
        LeftGrabFingerProbe
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

    struct BodyOverlayFrame
    {
        RE::hknpWorld* world{ nullptr };
        std::array<BodyOverlayEntry, MAX_WEAPON_COLLISION_BODIES + 8> entries{};
        std::array<AxisOverlayEntry, 16> axisEntries{};
        std::array<MarkerOverlayEntry, 32> markerEntries{};
        std::uint32_t count{ 0 };
        std::uint32_t axisCount{ 0 };
        std::uint32_t markerCount{ 0 };
        bool drawRockBodies{ false };
        bool drawTargetBodies{ false };
        bool drawAxes{ false };
        bool drawMarkers{ false };
    };

    void Install();
    bool IsInstalled();
    void PublishFrame(const BodyOverlayFrame& frame);
    void ClearFrame();
    void ClearShapeCache();
}

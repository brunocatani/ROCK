#pragma once

#include <cstdint>
#include <string_view>

namespace rock::body_zone
{
    /*
     * Body zones are gameplay-facing identities for generated body colliders.
     * Descriptor indices are implementation details and can diverge between the
     * standard and power-armor profiles; stash, holster, backpack, and provider
     * queries need stable names that survive descriptor table reshaping.
     */
    enum class BodyZoneSide : std::uint32_t
    {
        Center = 0,
        Left = 1,
        Right = 2,
    };

    enum class BodyZoneKind : std::uint32_t
    {
        Unknown = 0,
        Pelvis = 1,
        SpineLower = 2,
        SpineUpper = 3,
        Chest = 4,
        NeckHead = 5,
        LeftShoulder = 6,
        LeftUpperArm = 7,
        LeftForearmUpper = 8,
        LeftForearmLower = 9,
        LeftHand = 10,
        RightShoulder = 11,
        RightUpperArm = 12,
        RightForearmUpper = 13,
        RightForearmLower = 14,
        RightHand = 15,
        LeftHip = 16,
        LeftThigh = 17,
        LeftCalf = 18,
        LeftFoot = 19,
        RightHip = 20,
        RightThigh = 21,
        RightCalf = 22,
        RightFoot = 23,
    };

    inline constexpr std::string_view bodyZoneName(BodyZoneKind zone)
    {
        switch (zone) {
        case BodyZoneKind::Pelvis:
            return "Pelvis";
        case BodyZoneKind::SpineLower:
            return "SpineLower";
        case BodyZoneKind::SpineUpper:
            return "SpineUpper";
        case BodyZoneKind::Chest:
            return "Chest";
        case BodyZoneKind::NeckHead:
            return "NeckHead";
        case BodyZoneKind::LeftShoulder:
            return "LeftShoulder";
        case BodyZoneKind::LeftUpperArm:
            return "LeftUpperArm";
        case BodyZoneKind::LeftForearmUpper:
            return "LeftForearmUpper";
        case BodyZoneKind::LeftForearmLower:
            return "LeftForearmLower";
        case BodyZoneKind::LeftHand:
            return "LeftHand";
        case BodyZoneKind::RightShoulder:
            return "RightShoulder";
        case BodyZoneKind::RightUpperArm:
            return "RightUpperArm";
        case BodyZoneKind::RightForearmUpper:
            return "RightForearmUpper";
        case BodyZoneKind::RightForearmLower:
            return "RightForearmLower";
        case BodyZoneKind::RightHand:
            return "RightHand";
        case BodyZoneKind::LeftHip:
            return "LeftHip";
        case BodyZoneKind::LeftThigh:
            return "LeftThigh";
        case BodyZoneKind::LeftCalf:
            return "LeftCalf";
        case BodyZoneKind::LeftFoot:
            return "LeftFoot";
        case BodyZoneKind::RightHip:
            return "RightHip";
        case BodyZoneKind::RightThigh:
            return "RightThigh";
        case BodyZoneKind::RightCalf:
            return "RightCalf";
        case BodyZoneKind::RightFoot:
            return "RightFoot";
        case BodyZoneKind::Unknown:
            break;
        }
        return "Unknown";
    }

    inline constexpr std::string_view bodyZoneSideName(BodyZoneSide side)
    {
        switch (side) {
        case BodyZoneSide::Center:
            return "Center";
        case BodyZoneSide::Left:
            return "Left";
        case BodyZoneSide::Right:
            return "Right";
        }
        return "Center";
    }

    inline constexpr bool isLeftSide(BodyZoneSide side) { return side == BodyZoneSide::Left; }
    inline constexpr bool isRightSide(BodyZoneSide side) { return side == BodyZoneSide::Right; }
    inline constexpr bool isCentered(BodyZoneSide side) { return side == BodyZoneSide::Center; }
}

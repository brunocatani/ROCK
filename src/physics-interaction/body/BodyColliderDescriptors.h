#pragma once

#include "physics-interaction/body/BodyZone.h"

#include <array>
#include <string_view>

namespace rock::body_collider
{
    /*
     * The generated body collider set: one capsule-like hull per bone segment,
     * from a start bone to an end bone. Roles name the segment kind; zones
     * (BodyZone.h) are the stable gameplay identities. Role values reach the
     * provider's BodyContact and player collider records.
     */
    enum class Role : int
    {
        UpperArmSegment = 0,
        ForearmSegment = 1,
        HandSegment = 2,
        // 3 was a finger segment the body set never used; never reuse it.
        TorsoSegment = 4,
    };

    inline constexpr std::string_view roleName(Role role)
    {
        switch (role) {
        case Role::UpperArmSegment:
            return "UpperArm";
        case Role::ForearmSegment:
            return "Forearm";
        case Role::HandSegment:
            return "Hand";
        case Role::TorsoSegment:
            return "Torso";
        }
        return "Unknown";
    }

    struct Descriptor
    {
        Role role = Role::TorsoSegment;
        body_zone::BodyZoneKind zone = body_zone::BodyZoneKind::Unknown;
        body_zone::BodyZoneSide side = body_zone::BodyZoneSide::Center;
        std::string_view startBone{};
        std::string_view endBone{};
        float radiusGameUnits = 1.0f;
        float convexRadiusGameUnits = 0.25f;
        // The part of the start-to-end bone span the hull covers.
        float startFraction = 0.0f;
        float endFraction = 1.0f;
    };

    template <class Vector>
    inline void applyEndpointRange(const Descriptor& descriptor, Vector& start, Vector& end)
    {
        if (descriptor.startFraction == 0.0f && descriptor.endFraction == 1.0f) {
            return;
        }
        const Vector origin = start;
        const Vector span{ end.x - origin.x, end.y - origin.y, end.z - origin.z };
        start = Vector{ origin.x + span.x * descriptor.startFraction,
            origin.y + span.y * descriptor.startFraction,
            origin.z + span.z * descriptor.startFraction };
        end = Vector{ origin.x + span.x * descriptor.endFraction,
            origin.y + span.y * descriptor.endFraction,
            origin.z + span.z * descriptor.endFraction };
    }

    inline constexpr std::array<Descriptor, 15> kStandardDescriptors{
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::Pelvis, body_zone::BodyZoneSide::Center, "Pelvis", "SPINE1", 4.5f, 0.50f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::SpineLower, body_zone::BodyZoneSide::Center, "SPINE1", "SPINE2", 5.0f, 0.50f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::SpineUpper, body_zone::BodyZoneSide::Center, "SPINE2", "Chest", 5.5f, 0.50f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::Chest, body_zone::BodyZoneSide::Center, "Chest", "Neck", 3.5f, 0.40f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::NeckHead, body_zone::BodyZoneSide::Center, "Neck", "Head", 3.0f, 0.35f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::LeftShoulder, body_zone::BodyZoneSide::Left, "LArm_Collarbone", "LArm_UpperArm", 2.5f, 0.30f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::LeftUpperArm, body_zone::BodyZoneSide::Left, "LArm_UpperArm", "LArm_ForeArm1", 2.7f, 0.30f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::LeftForearmUpper, body_zone::BodyZoneSide::Left, "LArm_ForeArm1", "LArm_ForeArm2", 2.2f, 0.25f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::LeftForearmLower, body_zone::BodyZoneSide::Left, "LArm_ForeArm2", "LArm_ForeArm3", 2.0f, 0.25f },
        Descriptor{ Role::HandSegment, body_zone::BodyZoneKind::LeftHand, body_zone::BodyZoneSide::Left, "LArm_ForeArm3", "LArm_Hand", 2.1f, 0.25f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::RightShoulder, body_zone::BodyZoneSide::Right, "RArm_Collarbone", "RArm_UpperArm", 2.5f, 0.30f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::RightUpperArm, body_zone::BodyZoneSide::Right, "RArm_UpperArm", "RArm_ForeArm1", 2.7f, 0.30f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::RightForearmUpper, body_zone::BodyZoneSide::Right, "RArm_ForeArm1", "RArm_ForeArm2", 2.2f, 0.25f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::RightForearmLower, body_zone::BodyZoneSide::Right, "RArm_ForeArm2", "RArm_ForeArm3", 2.0f, 0.25f },
        Descriptor{ Role::HandSegment, body_zone::BodyZoneKind::RightHand, body_zone::BodyZoneSide::Right, "RArm_ForeArm3", "RArm_Hand", 2.1f, 0.25f }
    };

    // The armor rig has coincident ForeArm1/ForeArm2 positions (runtime 2026-09-08).
    // Split the nonzero ForeArm1->ForeArm3 span into adjacent upper/lower zones;
    // keep the wrist segment separate and the merged forearm's total length intact.
    inline constexpr std::array<Descriptor, 15> kPowerArmorDescriptors{
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::Pelvis, body_zone::BodyZoneSide::Center, "Pelvis", "SPINE1", 6.5f, 0.65f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::SpineLower, body_zone::BodyZoneSide::Center, "SPINE1", "SPINE2", 7.0f, 0.65f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::SpineUpper, body_zone::BodyZoneSide::Center, "SPINE2", "Chest", 7.5f, 0.65f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::Chest, body_zone::BodyZoneSide::Center, "Chest", "Neck", 5.0f, 0.50f },
        Descriptor{ Role::TorsoSegment, body_zone::BodyZoneKind::NeckHead, body_zone::BodyZoneSide::Center, "Neck", "Head", 4.0f, 0.45f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::LeftShoulder, body_zone::BodyZoneSide::Left, "LArm_Collarbone", "LArm_UpperArm", 4.2f, 0.45f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::LeftUpperArm, body_zone::BodyZoneSide::Left, "LArm_UpperArm", "LArm_ForeArm1", 4.5f, 0.45f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::LeftForearmUpper, body_zone::BodyZoneSide::Left, "LArm_ForeArm1", "LArm_ForeArm3", 3.8f, 0.40f, 0.0f, 0.5f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::LeftForearmLower, body_zone::BodyZoneSide::Left, "LArm_ForeArm1", "LArm_ForeArm3", 3.4f, 0.40f, 0.5f, 1.0f },
        Descriptor{ Role::HandSegment, body_zone::BodyZoneKind::LeftHand, body_zone::BodyZoneSide::Left, "LArm_ForeArm3", "LArm_Hand", 3.5f, 0.35f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::RightShoulder, body_zone::BodyZoneSide::Right, "RArm_Collarbone", "RArm_UpperArm", 4.2f, 0.45f },
        Descriptor{ Role::UpperArmSegment, body_zone::BodyZoneKind::RightUpperArm, body_zone::BodyZoneSide::Right, "RArm_UpperArm", "RArm_ForeArm1", 4.5f, 0.45f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::RightForearmUpper, body_zone::BodyZoneSide::Right, "RArm_ForeArm1", "RArm_ForeArm3", 3.8f, 0.40f, 0.0f, 0.5f },
        Descriptor{ Role::ForearmSegment, body_zone::BodyZoneKind::RightForearmLower, body_zone::BodyZoneSide::Right, "RArm_ForeArm1", "RArm_ForeArm3", 3.4f, 0.40f, 0.5f, 1.0f },
        Descriptor{ Role::HandSegment, body_zone::BodyZoneKind::RightHand, body_zone::BodyZoneSide::Right, "RArm_ForeArm3", "RArm_Hand", 3.5f, 0.35f }
    };
}

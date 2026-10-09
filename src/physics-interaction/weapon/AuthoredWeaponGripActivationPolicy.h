#pragma once

#include "physics-interaction/weapon/WeaponClassificationPolicy.h"
#include "physics-interaction/VectorMath.h"

#include <cmath>
#include <cstdint>

namespace rock::authored_weapon_grip_activation_policy
{
    inline constexpr std::uint32_t kRightHandEquipSlotFormID =
        weapon_classification_policy::kRightHandEquipSlotFormID;
    inline constexpr std::uint32_t kBothHandsLeftOptionalEquipSlotFormID =
        weapon_classification_policy::kBothHandsLeftOptionalEquipSlotFormID;
    inline constexpr std::uint32_t kBothHandsEquipSlotFormID =
        weapon_classification_policy::kBothHandsEquipSlotFormID;
    enum class WeaponFamily : std::uint8_t
    {
        Unknown,
        OneHandGun,
        TwoHandGun,
        Unsupported,
    };

    /*
     * Authored support data originates from Bethesda's native
     * right-firing/left-support graph, but ambidextrous carry has two distinct
     * runtime topologies. Keep them explicit: collapsing this back to a single
     * "authored scope" previously made every valid mirrored right-support pose
     * fall through to dynamic grab.
     */
    enum class HandTopology : std::uint8_t
    {
        Invalid,
        RightFiringLeftSupport,
        LeftFiringRightSupport,
    };


    [[nodiscard]] constexpr HandTopology resolveHandTopology(
        const bool firingHandIsLeft,
        const bool supportHandIsLeft)
    {
        if (!firingHandIsLeft && supportHandIsLeft) {
            return HandTopology::RightFiringLeftSupport;
        }
        if (firingHandIsLeft && !supportHandIsLeft) {
            return HandTopology::LeftFiringRightSupport;
        }
        return HandTopology::Invalid;
    }

    struct WeaponFamilyInput
    {
        std::uint32_t effectiveEquipSlotFormID{ 0 };
        bool equippedWeaponPresent{ false };
        bool meleeOrUnarmed{ false };
    };

    [[nodiscard]] constexpr WeaponFamily resolveWeaponFamily(
        const WeaponFamilyInput& input)
    {
        if (!input.equippedWeaponPresent) {
            return WeaponFamily::Unknown;
        }
        if (input.meleeOrUnarmed) {
            return WeaponFamily::Unsupported;
        }
        /*
         * Support entry follows the effective BGSEquipType behavior, not the
         * weapon's size class. Heavy guns with a verified player-hand slot and
         * a complete authored pose use the same spatial contract; the later
         * identity, generation, surface, and finger-pose gates still fail
         * closed when that runtime evidence is unavailable.
         */
        if (weapon_classification_policy::isOneHandGunBehaviorSlot(
                input.effectiveEquipSlotFormID)) {
            return WeaponFamily::OneHandGun;
        }
        if (weapon_classification_policy::isTwoHandGunBehaviorSlot(
                input.effectiveEquipSlotFormID)) {
            return WeaponFamily::TwoHandGun;
        }
        return WeaponFamily::Unknown;
    }

    struct Vec3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    /*
     * Axis input is weapon-local and authored for the native right-firing
     * topology. The support-pose mirror reflects weapon-local X, so activation
     * axes must use the identical reflection: lateral changes LEFT to RIGHT,
     * while every non-lateral component (including the DOWN endpoint) remains
     * geometrically mirrored with the authored seat.
     */
    [[nodiscard]] constexpr Vec3 orientRightFiringAxisForTopology(
        const Vec3& rightFiringAxisWeaponLocal,
        const HandTopology topology)
    {
        switch (topology) {
        case HandTopology::RightFiringLeftSupport:
            return rightFiringAxisWeaponLocal;
        case HandTopology::LeftFiringRightSupport:
            return Vec3{
                -rightFiringAxisWeaponLocal.x,
                rightFiringAxisWeaponLocal.y,
                rightFiringAxisWeaponLocal.z,
            };
        case HandTopology::Invalid:
        default:
            return {};
        }
    }

    [[nodiscard]] constexpr Vec3 subtract(const Vec3& lhs, const Vec3& rhs)
    {
        return Vec3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    [[nodiscard]] constexpr float dot(const Vec3& lhs, const Vec3& rhs)
    {
        return vector_math::dot(lhs, rhs);
    }

    [[nodiscard]] inline float length(const Vec3& value)
    {
        const float lengthSquared = dot(value, value);
        return lengthSquared > 0.0f && std::isfinite(lengthSquared) ?
            std::sqrt(lengthSquared) : 0.0f;
    }

    [[nodiscard]] inline bool tryNormalize(const Vec3& value, Vec3& outValue)
    {
        outValue = {};
        const float valueLength = length(value);
        if (!std::isfinite(valueLength) || valueLength <= 0.000001f) {
            return false;
        }
        const float inverseLength = 1.0f / valueLength;
        outValue = Vec3{
            value.x * inverseLength,
            value.y * inverseLength,
            value.z * inverseLength,
        };
        return std::isfinite(outValue.x) &&
               std::isfinite(outValue.y) &&
               std::isfinite(outValue.z);
    }

    struct RadialGateInput
    {
        WeaponFamily weaponFamily{WeaponFamily::Unknown};
        HandTopology handTopology{HandTopology::Invalid};
        Vec3 authoredSeatWorld{}, liveProbeWorld{};
        float radialCapGameUnits{};
    };
    struct RadialGateResult
    {
        float radialDistanceGameUnits{};
        bool familySupported{}, radialPass{}, topologyPass{}, spatialPass{};
    };
    inline RadialGateResult evaluateRadialGate(const RadialGateInput& input)
    {
        RadialGateResult result{};
        const auto delta = subtract(input.liveProbeWorld, input.authoredSeatWorld);
        const float squared = dot(delta, delta);
        if (!std::isfinite(squared) || squared < 0.0f) return result;
        result.radialDistanceGameUnits = std::sqrt(squared);
        result.familySupported = input.weaponFamily == WeaponFamily::OneHandGun || input.weaponFamily == WeaponFamily::TwoHandGun;
        result.topologyPass = input.handTopology != HandTopology::Invalid;
        result.radialPass = std::isfinite(input.radialCapGameUnits) && input.radialCapGameUnits > 0.0f && result.radialDistanceGameUnits <= input.radialCapGameUnits;
        result.spatialPass = result.familySupported && result.topologyPass && result.radialPass;
        return result;
    }

    struct IndicatorInput
    {
        WeaponFamily weaponFamily{ WeaponFamily::Unknown };
        Vec3 authoredSeatWorld{};
        Vec3 supportSideAxisWorld{};
        Vec3 downAxisWorld{};
        bool activationStateValid{ false };
        bool activationSpatialPass{ false };
        bool supportGripAllowed{ false };
        bool providerPartAuthorityActive{ false };
        bool supportHandHoldingObject{ false };
        bool supportHandWeaponEngaged{ false };
    };

    struct IndicatorResult
    {
        Vec3 markerWorld{};
        bool visible{ false };
    };

    [[nodiscard]] inline IndicatorResult evaluateIndicator(
        const IndicatorInput& input)
    {
        if (!input.activationStateValid ||
            !input.activationSpatialPass ||
            !input.supportGripAllowed ||
            input.providerPartAuthorityActive ||
            input.supportHandHoldingObject ||
            input.supportHandWeaponEngaged) {
            return {};
        }

        const auto& seat = input.authoredSeatWorld;
        if (!std::isfinite(seat.x) || !std::isfinite(seat.y) || !std::isfinite(seat.z)) return {};
        return IndicatorResult{seat, true};
    }

    [[nodiscard]] constexpr const char* weaponFamilyName(const WeaponFamily family)
    {
        switch (family) {
        case WeaponFamily::OneHandGun:
            return "ONE_HAND_GUN";
        case WeaponFamily::TwoHandGun:
            return "TWO_HAND_GUN";
        case WeaponFamily::Unsupported:
            return "UNSUPPORTED";
        case WeaponFamily::Unknown:
        default:
            return "UNKNOWN";
        }
    }

    [[nodiscard]] constexpr const char* handTopologyName(
        const HandTopology topology)
    {
        switch (topology) {
        case HandTopology::RightFiringLeftSupport:
            return "RIGHT_FIRE_LEFT_SUPPORT";
        case HandTopology::LeftFiringRightSupport:
            return "LEFT_FIRE_RIGHT_SUPPORT";
        case HandTopology::Invalid:
        default:
            return "INVALID";
        }
    }
}

#pragma once

#include "physics-interaction/weapon/WeaponClassificationPolicy.h"
#include "physics-interaction/VectorMath.h"

#include <cmath>
#include <cstdint>

namespace rock::authored_weapon_grip_activation_policy
{
    inline constexpr std::uint32_t kRightHandEquipSlotFormID =
        weapon_classification_policy::kRightHandEquipSlotFormID;
    inline constexpr std::uint32_t kBothHandsEquipSlotFormID =
        weapon_classification_policy::kBothHandsEquipSlotFormID;
    inline constexpr float kActivationHalfAngleDegrees = 45.0f;
    inline constexpr float kActivationConeMinimumDot = 0.70710678118654752440f;
    inline constexpr float kMinimumDirectionDistanceGameUnits = 0.25f;

    enum class WeaponFamily : std::uint8_t
    {
        Unknown,
        OneHandGun,
        TwoHandGun,
        Unsupported,
    };

    enum class AllowedCone : std::uint8_t
    {
        None,
        Left,
        Down,
    };

    struct WeaponFamilyInput
    {
        std::uint32_t effectiveEquipSlotFormID{ 0 };
        bool equippedWeaponPresent{ false };
        bool meleeOrUnarmed{ false };
        bool heavyGun{ false };
    };

    [[nodiscard]] constexpr WeaponFamily resolveWeaponFamily(
        const WeaponFamilyInput& input)
    {
        if (!input.equippedWeaponPresent) {
            return WeaponFamily::Unknown;
        }
        if (input.meleeOrUnarmed || input.heavyGun) {
            return WeaponFamily::Unsupported;
        }
        if (input.effectiveEquipSlotFormID == kRightHandEquipSlotFormID) {
            return WeaponFamily::OneHandGun;
        }
        if (input.effectiveEquipSlotFormID == kBothHandsEquipSlotFormID) {
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

    struct DirectionGateInput
    {
        WeaponFamily weaponFamily{ WeaponFamily::Unknown };
        Vec3 authoredSeatWorld{};
        Vec3 liveProbeWorld{};
        Vec3 leftAxisWorld{};
        Vec3 downAxisWorld{};
        Vec3 lastStableDirectionWorld{};
        float radialCapGameUnits{ 0.0f };
        bool lastStableDirectionValid{ false };
        bool rightFiringLeftSupportScope{ true };
    };

    struct DirectionGateResult
    {
        Vec3 approachDirectionWorld{};
        float radialDistanceGameUnits{ 0.0f };
        float leftDot{ -1.0f };
        float downDot{ -1.0f };
        AllowedCone selectedCone{ AllowedCone::None };
        bool directionValid{ false };
        bool usedLastStableDirection{ false };
        bool familySupported{ false };
        bool radialPass{ false };
        bool directionPass{ false };
        bool scopePass{ false };
        bool spatialPass{ false };
    };

    [[nodiscard]] inline DirectionGateResult evaluateDirectionGate(
        const DirectionGateInput& input)
    {
        DirectionGateResult result{};
        const Vec3 radialVector = subtract(input.liveProbeWorld, input.authoredSeatWorld);
        result.radialDistanceGameUnits = length(radialVector);
        result.familySupported =
            input.weaponFamily == WeaponFamily::OneHandGun ||
            input.weaponFamily == WeaponFamily::TwoHandGun;
        result.radialPass =
            std::isfinite(input.radialCapGameUnits) &&
            input.radialCapGameUnits >= 0.0f &&
            result.radialDistanceGameUnits <= input.radialCapGameUnits;
        result.scopePass = input.rightFiringLeftSupportScope;

        if (result.radialDistanceGameUnits >= kMinimumDirectionDistanceGameUnits) {
            result.directionValid = tryNormalize(radialVector, result.approachDirectionWorld);
        } else if (input.lastStableDirectionValid) {
            result.directionValid =
                tryNormalize(input.lastStableDirectionWorld, result.approachDirectionWorld);
            result.usedLastStableDirection = result.directionValid;
        }

        Vec3 normalizedLeft{};
        Vec3 normalizedDown{};
        const bool leftValid = tryNormalize(input.leftAxisWorld, normalizedLeft);
        const bool downValid = tryNormalize(input.downAxisWorld, normalizedDown);
        if (result.directionValid && leftValid) {
            result.leftDot = dot(result.approachDirectionWorld, normalizedLeft);
        }
        if (result.directionValid && downValid) {
            result.downDot = dot(result.approachDirectionWorld, normalizedDown);
        }

        if (input.weaponFamily == WeaponFamily::OneHandGun &&
            result.directionValid && leftValid &&
            result.leftDot >= kActivationConeMinimumDot) {
            result.directionPass = true;
            result.selectedCone = AllowedCone::Left;
        } else if (input.weaponFamily == WeaponFamily::TwoHandGun &&
                   result.directionValid) {
            const bool leftPass = leftValid &&
                result.leftDot >= kActivationConeMinimumDot;
            const bool downPass = downValid &&
                result.downDot >= kActivationConeMinimumDot;
            result.directionPass = leftPass || downPass;
            if (leftPass && (!downPass || result.leftDot >= result.downDot)) {
                result.selectedCone = AllowedCone::Left;
            } else if (downPass) {
                result.selectedCone = AllowedCone::Down;
            }
        }

        result.spatialPass =
            result.familySupported &&
            result.radialPass &&
            result.directionPass &&
            result.scopePass;
        return result;
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

    [[nodiscard]] constexpr const char* allowedConeName(const AllowedCone cone)
    {
        switch (cone) {
        case AllowedCone::Left:
            return "LEFT";
        case AllowedCone::Down:
            return "DOWN";
        case AllowedCone::None:
        default:
            return "NONE";
        }
    }
}

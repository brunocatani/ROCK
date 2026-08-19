#pragma once

#include <cmath>
#include <cstdint>

namespace rock::authored_weapon_grip_activation_policy
{
    inline constexpr std::uint32_t kRightHandEquipSlotFormID = 0x00013F42u;
    inline constexpr std::uint32_t kBothHandsEquipSlotFormID = 0x00013F45u;
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
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
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

    struct ConeBoundaryDimensions
    {
        float axialGameUnits{ 0.0f };
        float rimRadiusGameUnits{ 0.0f };
        bool valid{ false };
    };

    /*
     * The enforced direction gate is a cone clipped by a sphere centered on
     * the authored seat. The debug wire must end where the conical side meets
     * that sphere; using the radial cap as both axial length and rim radius
     * would draw a boundary sqrt(2) times farther away for the 45-degree cone.
     */
    [[nodiscard]] inline ConeBoundaryDimensions resolveConeBoundaryDimensions(
        const float radialCapGameUnits)
    {
        if (!std::isfinite(radialCapGameUnits) ||
            radialCapGameUnits <= 0.0f) {
            return {};
        }
        const float rimFactorSquared =
            1.0f -
            kActivationConeMinimumDot * kActivationConeMinimumDot;
        if (!std::isfinite(rimFactorSquared) ||
            rimFactorSquared < 0.0f) {
            return {};
        }
        const float rimFactor = std::sqrt(rimFactorSquared);
        const float axialGameUnits =
            radialCapGameUnits * kActivationConeMinimumDot;
        const float rimRadiusGameUnits =
            radialCapGameUnits * rimFactor;
        return {
            .axialGameUnits = axialGameUnits,
            .rimRadiusGameUnits = rimRadiusGameUnits,
            .valid = std::isfinite(axialGameUnits) &&
                     std::isfinite(rimRadiusGameUnits),
        };
    }

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

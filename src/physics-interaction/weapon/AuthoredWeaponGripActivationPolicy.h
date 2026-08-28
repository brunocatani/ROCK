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
    inline constexpr float kActivationHalfAngleDegrees = 45.0f;
    inline constexpr float kActivationConeMinimumDot = 0.70710678118654752440f;
    inline constexpr float kActivationConeMinimumDotSquared =
        kActivationConeMinimumDot * kActivationConeMinimumDot;
    inline constexpr float kSweptArcAxisOrthogonalityTolerance = 0.001f;
    inline constexpr float kMinimumDirectionDistanceGameUnits = 0.25f;
    inline constexpr float kIndicatorOffsetGameUnits = 5.0f;

    enum class WeaponFamily : std::uint8_t
    {
        Unknown,
        OneHandGun,
        TwoHandGun,
        Unsupported,
    };

    enum class ActivationRegion : std::uint8_t
    {
        None,
        Left,
        Arc,
        Down,
    };

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

    struct ActivationBoundaryDimensions
    {
        float axialGameUnits{ 0.0f };
        float rimRadiusGameUnits{ 0.0f };
        bool valid{ false };
    };

    /*
     * The angular gate is clipped by a sphere centered on the authored seat.
     * These dimensions put the debug boundary on that sphere instead of using
     * the radial cap as both cone height and rim radius.
     */
    [[nodiscard]] inline ActivationBoundaryDimensions
        resolveActivationBoundaryDimensions(const float radialExtentGameUnits)
    {
        if (!std::isfinite(radialExtentGameUnits) ||
            radialExtentGameUnits <= 0.0f) {
            return {};
        }
        const float rimFactorSquared =
            1.0f - kActivationConeMinimumDotSquared;
        if (!std::isfinite(rimFactorSquared) || rimFactorSquared < 0.0f) {
            return {};
        }
        const float axialGameUnits =
            radialExtentGameUnits * kActivationConeMinimumDot;
        const float rimRadiusGameUnits =
            radialExtentGameUnits * std::sqrt(rimFactorSquared);
        return ActivationBoundaryDimensions{
            .axialGameUnits = axialGameUnits,
            .rimRadiusGameUnits = rimRadiusGameUnits,
            .valid = std::isfinite(axialGameUnits) &&
                     std::isfinite(rimRadiusGameUnits),
        };
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
        float sweptArcDot{ -1.0f };
        ActivationRegion selectedRegion{ ActivationRegion::None };
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
            result.selectedRegion = ActivationRegion::Left;
        } else if (input.weaponFamily == WeaponFamily::TwoHandGun &&
                   result.directionValid && leftValid && downValid) {
            /*
             * LEFT and DOWN are an orthogonal pair derived from the canonical
             * firing-hand frame. The nearest axis on their quarter-circle is
             * the normalized planar projection while both components are
             * positive; outside that interval, the nearest endpoint wins.
             * Comparing that closest-axis dot against the original 45-degree
             * threshold produces the exact union of cones swept along the arc.
             */
            const float axisDot = dot(normalizedLeft, normalizedDown);
            if (std::isfinite(axisDot) &&
                std::abs(axisDot) <=
                    kSweptArcAxisOrthogonalityTolerance) {
                ActivationRegion nearestRegion = ActivationRegion::None;
                if (result.leftDot > 0.0f && result.downDot > 0.0f) {
                    const float sweptArcDotSquared =
                        result.leftDot * result.leftDot +
                        result.downDot * result.downDot;
                    if (std::isfinite(sweptArcDotSquared) &&
                        sweptArcDotSquared >= 0.0f) {
                        result.sweptArcDot = std::sqrt(sweptArcDotSquared);
                        nearestRegion = ActivationRegion::Arc;
                    }
                } else if (result.leftDot >= result.downDot) {
                    result.sweptArcDot = result.leftDot;
                    nearestRegion = ActivationRegion::Left;
                } else {
                    result.sweptArcDot = result.downDot;
                    nearestRegion = ActivationRegion::Down;
                }

                if (result.sweptArcDot >= kActivationConeMinimumDot) {
                    result.directionPass = true;
                    result.selectedRegion = nearestRegion;
                }
            }
        }

        result.spatialPass =
            result.familySupported &&
            result.radialPass &&
            result.directionPass &&
            result.scopePass;
        return result;
    }

    struct IndicatorInput
    {
        WeaponFamily weaponFamily{ WeaponFamily::Unknown };
        Vec3 authoredSeatWorld{};
        Vec3 leftAxisWorld{};
        Vec3 downAxisWorld{};
        bool activationStateValid{ false };
        bool activationSpatialPass{ false };
        bool interactionCandidateValid{ false };
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
            !input.interactionCandidateValid ||
            !input.supportGripAllowed ||
            input.providerPartAuthorityActive ||
            input.supportHandHoldingObject ||
            input.supportHandWeaponEngaged) {
            return {};
        }

        Vec3 indicatorAxis{};
        switch (input.weaponFamily) {
        case WeaponFamily::OneHandGun:
            indicatorAxis = input.leftAxisWorld;
            break;
        case WeaponFamily::TwoHandGun:
            indicatorAxis = input.downAxisWorld;
            break;
        case WeaponFamily::Unsupported:
        case WeaponFamily::Unknown:
        default:
            return {};
        }

        Vec3 normalizedAxis{};
        if (!tryNormalize(indicatorAxis, normalizedAxis) ||
            !std::isfinite(input.authoredSeatWorld.x) ||
            !std::isfinite(input.authoredSeatWorld.y) ||
            !std::isfinite(input.authoredSeatWorld.z)) {
            return {};
        }

        const Vec3 markerWorld{
            input.authoredSeatWorld.x +
                normalizedAxis.x * kIndicatorOffsetGameUnits,
            input.authoredSeatWorld.y +
                normalizedAxis.y * kIndicatorOffsetGameUnits,
            input.authoredSeatWorld.z +
                normalizedAxis.z * kIndicatorOffsetGameUnits,
        };
        if (!std::isfinite(markerWorld.x) ||
            !std::isfinite(markerWorld.y) ||
            !std::isfinite(markerWorld.z)) {
            return {};
        }

        return IndicatorResult{
            .markerWorld = markerWorld,
            .visible = true,
        };
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

    [[nodiscard]] constexpr const char* activationRegionName(
        const ActivationRegion region)
    {
        switch (region) {
        case ActivationRegion::Left:
            return "LEFT";
        case ActivationRegion::Arc:
            return "ARC";
        case ActivationRegion::Down:
            return "DOWN";
        case ActivationRegion::None:
        default:
            return "NONE";
        }
    }
}

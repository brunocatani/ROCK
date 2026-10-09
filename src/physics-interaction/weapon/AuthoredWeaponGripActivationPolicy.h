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
    inline constexpr float kIndicatorOffsetGameUnits = 3.0f;

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

    enum class ActivationRegion : std::uint8_t
    {
        None,
        Left,
        Right,
        Arc,
        Down,
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

    [[nodiscard]] constexpr ActivationRegion supportSideRegion(
        const HandTopology topology)
    {
        switch (topology) {
        case HandTopology::RightFiringLeftSupport:
            return ActivationRegion::Left;
        case HandTopology::LeftFiringRightSupport:
            return ActivationRegion::Right;
        case HandTopology::Invalid:
        default:
            return ActivationRegion::None;
        }
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
        HandTopology handTopology{ HandTopology::Invalid };
        Vec3 authoredSeatWorld{};
        Vec3 liveProbeWorld{};
        Vec3 supportSideAxisWorld{};
        Vec3 downAxisWorld{};
        Vec3 lastStableDirectionWorld{};
        float radialCapGameUnits{ 0.0f };
        bool lastStableDirectionValid{ false };
    };

    struct DirectionGateResult
    {
        Vec3 approachDirectionWorld{};
        float radialDistanceGameUnits{ 0.0f };
        float supportSideDot{ -1.0f };
        float downDot{ -1.0f };
        float sweptArcDot{ -1.0f };
        ActivationRegion selectedRegion{ ActivationRegion::None };
        bool directionValid{ false };
        bool usedLastStableDirection{ false };
        bool familySupported{ false };
        bool radialPass{ false };
        bool directionPass{ false };
        bool topologyPass{ false };
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
        const ActivationRegion lateralRegion =
            supportSideRegion(input.handTopology);
        result.topologyPass = lateralRegion != ActivationRegion::None;

        if (result.radialDistanceGameUnits >= kMinimumDirectionDistanceGameUnits) {
            result.directionValid = tryNormalize(radialVector, result.approachDirectionWorld);
        } else if (input.lastStableDirectionValid) {
            result.directionValid =
                tryNormalize(input.lastStableDirectionWorld, result.approachDirectionWorld);
            result.usedLastStableDirection = result.directionValid;
        }

        Vec3 normalizedSupportSide{};
        Vec3 normalizedDown{};
        const bool supportSideValid = tryNormalize(
            input.supportSideAxisWorld,
            normalizedSupportSide);
        const bool downValid = tryNormalize(input.downAxisWorld, normalizedDown);
        if (result.directionValid && supportSideValid) {
            result.supportSideDot = dot(
                result.approachDirectionWorld,
                normalizedSupportSide);
        }
        if (result.directionValid && downValid) {
            result.downDot = dot(result.approachDirectionWorld, normalizedDown);
        }

        if (input.weaponFamily == WeaponFamily::OneHandGun &&
            result.topologyPass && result.directionValid && supportSideValid &&
            result.supportSideDot >= kActivationConeMinimumDot) {
            result.directionPass = true;
            result.selectedRegion = lateralRegion;
        } else if (input.weaponFamily == WeaponFamily::TwoHandGun &&
                   result.topologyPass && result.directionValid &&
                   supportSideValid && downValid) {
            /*
             * The topology-specific support side (LEFT or RIGHT) and DOWN are
             * an orthogonal pair derived from the same canonical frame. The
             * nearest axis on their quarter-circle is the normalized planar
             * projection while both components are positive; outside that
             * interval, the nearest endpoint wins. Comparing that closest-axis
             * dot against the original 45-degree threshold produces the exact
             * union of cones swept along the arc.
             */
            const float axisDot = dot(normalizedSupportSide, normalizedDown);
            if (std::isfinite(axisDot) &&
                std::abs(axisDot) <=
                    kSweptArcAxisOrthogonalityTolerance) {
                ActivationRegion nearestRegion = ActivationRegion::None;
                if (result.supportSideDot > 0.0f && result.downDot > 0.0f) {
                    const float sweptArcDotSquared =
                        result.supportSideDot * result.supportSideDot +
                        result.downDot * result.downDot;
                    if (std::isfinite(sweptArcDotSquared) &&
                        sweptArcDotSquared >= 0.0f) {
                        result.sweptArcDot = std::sqrt(sweptArcDotSquared);
                        nearestRegion = ActivationRegion::Arc;
                    }
                } else if (result.supportSideDot >= result.downDot) {
                    result.sweptArcDot = result.supportSideDot;
                    nearestRegion = lateralRegion;
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
            result.topologyPass;
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

        Vec3 indicatorAxis{};
        switch (input.weaponFamily) {
        case WeaponFamily::OneHandGun:
            indicatorAxis = input.supportSideAxisWorld;
            break;
        case WeaponFamily::TwoHandGun: {
            // Bisect the swept quarter-arc independently of axis magnitudes.
            Vec3 normalizedSupportSide{};
            Vec3 normalizedDown{};
            if (!tryNormalize(
                    input.supportSideAxisWorld,
                    normalizedSupportSide) ||
                !tryNormalize(input.downAxisWorld, normalizedDown)) {
                return {};
            }
            indicatorAxis = Vec3{
                normalizedSupportSide.x + normalizedDown.x,
                normalizedSupportSide.y + normalizedDown.y,
                normalizedSupportSide.z + normalizedDown.z,
            };
            break;
        }
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
        case ActivationRegion::Right:
            return "RIGHT";
        case ActivationRegion::Arc:
            return "ARC";
        case ActivationRegion::Down:
            return "DOWN";
        case ActivationRegion::None:
        default:
            return "NONE";
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

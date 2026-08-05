#pragma once

#include "api/ROCKProviderApi.h"
#include "physics-interaction/weapon/WeaponTypes.h"

#include <cstdint>

namespace rock::physical_melee
{
    struct WeaponImpactGeometry
    {
        float longitudinalCoordinate{ 0.5f };
        std::uint32_t majorAxis{ 0 };
        std::uint32_t nearestAxis{ 0 };
        std::uint32_t closeFaceCount{ 1 };
    };

    struct WeaponImpactProfile
    {
        provider::RockProviderImpactSurfaceRegionV1 region{
            provider::RockProviderImpactSurfaceRegionV1::Unknown
        };
        float damageCoefficient{ 0.0f };
        std::uint32_t confidencePermille{ 0 };
        bool damaging{ false };
    };

    [[nodiscard]] inline constexpr bool isUnsafeImpactPart(WeaponPartKind part) noexcept
    {
        switch (part) {
        case WeaponPartKind::Shell:
        case WeaponPartKind::Round:
        case WeaponPartKind::LaserCell:
        case WeaponPartKind::Sight:
        case WeaponPartKind::Accessory:
        case WeaponPartKind::CosmeticAmmo:
        case WeaponPartKind::LaserSight:
        case WeaponPartKind::Flashlight:
        case WeaponPartKind::LaserFlashlightCombo:
        case WeaponPartKind::Scope:
        case WeaponPartKind::Bipod:
        case WeaponPartKind::Count:
            return true;
        default:
            return false;
        }
    }

    [[nodiscard]] inline constexpr WeaponImpactProfile classifyWeaponImpact(
        WeaponPartKind part,
        WeaponSizeClass sizeClass,
        const WeaponImpactGeometry& geometry) noexcept
    {
        using Region = provider::RockProviderImpactSurfaceRegionV1;
        if (isUnsafeImpactPart(part)) {
            return { Region::Flat, 0.0f, 1000, false };
        }

        switch (part) {
        case WeaponPartKind::Grip:
        case WeaponPartKind::Foregrip:
            return { Region::Grip, 0.25f, 950, true };
        case WeaponPartKind::Stock:
            return { Region::Pommel, 0.65f, 900, true };
        case WeaponPartKind::Handguard:
        case WeaponPartKind::Pump:
            return { Region::Flat, sizeClass == WeaponSizeClass::Melee ? 0.70f : 0.40f, 850, true };
        case WeaponPartKind::Magazine:
        case WeaponPartKind::Magwell:
            return { Region::Flat, 0.30f, 850, true };
        case WeaponPartKind::Bolt:
        case WeaponPartKind::Slide:
        case WeaponPartKind::ChargingHandle:
        case WeaponPartKind::BreakAction:
        case WeaponPartKind::Cylinder:
        case WeaponPartKind::Chamber:
        case WeaponPartKind::Lever:
            return { Region::Flat, 0.35f, 800, true };
        default:
            break;
        }

        const bool longitudinalEnd = geometry.nearestAxis == geometry.majorAxis &&
            (geometry.longitudinalCoordinate <= 0.15f || geometry.longitudinalCoordinate >= 0.85f);
        if (longitudinalEnd) {
            float coefficient = sizeClass == WeaponSizeClass::Melee ? 1.25f : 0.85f;
            if (part == WeaponPartKind::MuzzleDevice) {
                coefficient = 0.75f;
            }
            return { Region::Tip, coefficient, 750, true };
        }
        if (geometry.closeFaceCount >= 2) {
            return { Region::Edge, sizeClass == WeaponSizeClass::Melee ? 1.0f : 0.60f, 750, true };
        }
        return { Region::Flat, sizeClass == WeaponSizeClass::Melee ? 0.55f : 0.50f, 750, true };
    }
}

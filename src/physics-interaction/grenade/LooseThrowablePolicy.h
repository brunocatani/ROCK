#pragma once

#include <cmath>
#include <cstdint>

namespace rock::loose_throwable_policy
{
    enum class DetonationMode : std::uint8_t
    {
        Unsupported,
        TimedFuse,
        Impact,
        Proximity,
    };

    // PROJ DATA flag 0x40 is the authored "Can Be Picked Up" contract.
    inline constexpr std::uint32_t kProjectileCanBePickedUp = 0x40u;

    template <class WeaponType>
    [[nodiscard]] constexpr bool isSupportedWeaponType(
        WeaponType type,
        WeaponType grenadeType,
        WeaponType mineType) noexcept
    {
        return type == grenadeType || type == mineType;
    }

    template <class WeaponType>
    [[nodiscard]] inline DetonationMode classifyDetonationMode(
        WeaponType type,
        WeaponType grenadeType,
        WeaponType mineType,
        bool molotov,
        bool hasExplosion,
        float projectileProximity) noexcept
    {
        if (!isSupportedWeaponType(type, grenadeType, mineType) || !hasExplosion) {
            return DetonationMode::Unsupported;
        }
        if (type == mineType) {
            if (!std::isfinite(projectileProximity)) {
                return DetonationMode::Unsupported;
            }
            return projectileProximity > 0.0f ?
                       DetonationMode::Proximity :
                       DetonationMode::Impact;
        }
        return molotov ? DetonationMode::Impact : DetonationMode::TimedFuse;
    }

    [[nodiscard]] constexpr bool preservesReferenceAfterDetonation(
        DetonationMode mode,
        std::uint32_t projectileFlags,
        bool explosionSpawnsPlacedObject) noexcept
    {
        return mode == DetonationMode::Impact &&
               (projectileFlags & kProjectileCanBePickedUp) != 0 &&
               !explosionSpawnsPlacedObject;
    }

}

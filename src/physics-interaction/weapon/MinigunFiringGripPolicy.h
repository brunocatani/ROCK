#pragma once

#include "physics-interaction/weapon/WeaponTypes.h"

#include <array>
#include <cstdint>

namespace rock::minigun_firing_grip_policy
{
    struct FixedWeaponLocalTransform
    {
        std::array<float, 9> rotation{};
        std::array<float, 3> translation{};
        float scale{ 1.0f };
    };

    /*
     * User-approved Avenger Minigun hFRIK Weapon offset, promoted to ROCK's
     * compiled minigun firing-seat contract. hFRIK stores Weapon-in-primary-
     * hand; the runtime inverts this value before publishing Hand-in-Weapon.
     * Power armor intentionally uses this same calibration.
     */
    inline constexpr FixedWeaponLocalTransform kWeaponInFiringHand{
        .rotation = {
            -0.12200000137090683f,
            0.9900000095367432f,
            0.0689999982714653f,
            0.9869999885559082f,
            0.11400000005960464f,
            0.10899999737739563f,
            0.10000000149011612f,
            0.08100000023841858f,
            -0.9919999837875366f,
        },
        .translation = {
            6.887054443359375f,
            5.837783336639404f,
            8.01791000366211f,
        },
        .scale = 0.9999999403953552f,
    };

    [[nodiscard]] constexpr bool usesCompiledFiringSeat(
        const std::uint64_t keywordFlags) noexcept
    {
        return hasWeaponKeywordFlag(
            keywordFlags,
            WeaponKeywordFlag::Minigun);
    }

    [[nodiscard]] constexpr const FixedWeaponLocalTransform&
        weaponInFiringHand(const bool /* inPowerArmor */) noexcept
    {
        return kWeaponInFiringHand;
    }
}

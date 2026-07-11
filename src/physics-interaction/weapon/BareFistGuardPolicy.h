#pragma once

#include <cstdint>

namespace rock::bare_fist_guard_policy
{
    struct Witness
    {
        bool rockEnabled{ true };
        bool weaponDrawn{ false };
        bool actorUsingMelee{ false };
        bool realMeleeWeaponEquipped{ false };
    };

    [[nodiscard]] inline constexpr bool shouldHolster(const Witness& witness) noexcept
    {
        return witness.rockEnabled && witness.weaponDrawn && witness.actorUsingMelee && !witness.realMeleeWeaponEquipped;
    }

    struct RecheckState
    {
        bool initialized{ false };
        bool weaponDrawn{ false };
        std::uint32_t equippedWeaponFormId{ 0 };
        bool actorUsingMelee{ false };
        bool realMeleeWeaponEquipped{ false };
    };

    [[nodiscard]] inline constexpr bool shouldRefreshWitness(
        const RecheckState& state,
        bool forceRecheck,
        bool weaponDrawn,
        std::uint32_t equippedWeaponFormId,
        bool actorUsingMelee) noexcept
    {
        return forceRecheck || !state.initialized || state.weaponDrawn != weaponDrawn ||
               state.equippedWeaponFormId != equippedWeaponFormId ||
               state.actorUsingMelee != actorUsingMelee;
    }
}

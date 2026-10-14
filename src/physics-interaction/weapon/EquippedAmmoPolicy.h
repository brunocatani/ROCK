#pragma once

#include <cstdint>

/*
 * Loaded and reserve rounds of one equipped weapon slot, computed by the rule
 * of the native HUD ammo producer (FO4VR 1.2.72, 0x140A193B0, raw
 * disassembly), applied to that slot's equipment index instead of the
 * default index:
 * - no counter without a ranged weapon (base type >= 7) or for a weapon
 *   flagged kNotPlayable;
 * - no ammo: 0 loaded, 0 reserve;
 * - the count is the loaded rounds for the two special ammo forms (0x2F30E0),
 *   the weapon's capacity for non-playable ammo (ammo flag bit 1), otherwise
 *   the inventory count;
 * - charged ammo (TESAmmo data.health > 0) shows the count as reserve;
 *   otherwise reserve is the count less the loaded rounds.
 *
 * One correction: the inventory count also holds the rounds loaded in the
 * other slot's weapon when both use the same uncharged ammo, so they are not
 * reserve. The native producer misses this because it reads one index.
 */
namespace rock::equipped_ammo
{
    struct Facts
    {
        // An item at the index whose base weapon type is ranged.
        bool weapon{ false };
        bool notPlayable{ false };
        bool hasAmmo{ false };
        std::uint32_t loaded{ 0 };
        std::uint32_t charge{ 0 };
        bool countAsLoaded{ false };
        bool nonPlayableAmmo{ false };
        std::uint32_t capacity{ 0 };
        std::int32_t inventoryCount{ 0 };
        // Loaded rounds of the other slot's weapon when it uses the same ammo.
        std::uint32_t otherLoadedSameAmmo{ 0 };
    };

    struct Counts
    {
        bool known{ false };
        std::uint32_t loaded{ 0 };
        std::uint32_t reserve{ 0 };
    };

    [[nodiscard]] constexpr Counts resolve(const Facts& facts) noexcept
    {
        if (!facts.weapon || facts.notPlayable) return {};
        if (!facts.hasAmmo) return { .known = true };
        const bool inventory = !facts.countAsLoaded && !facts.nonPlayableAmmo;
        const std::uint32_t count = facts.countAsLoaded ? facts.loaded :
            facts.nonPlayableAmmo ? facts.capacity :
            facts.inventoryCount > 0 ? static_cast<std::uint32_t>(facts.inventoryCount) : 0u;
        if (facts.charge != 0) return { .known = true, .loaded = facts.loaded, .reserve = count };
        const std::uint64_t held = static_cast<std::uint64_t>(facts.loaded) + (inventory ? facts.otherLoadedSameAmmo : 0u);
        return { .known = true, .loaded = facts.loaded,
            .reserve = count > held ? static_cast<std::uint32_t>(count - held) : 0u };
    }
}

#pragma once
#include <cstdint>

namespace RE { class TESObjectREFR; class NiAVObject; class EquippedItem; }

namespace rock::carried_weapon_projectile
{
    struct Identity
    {
        std::uintptr_t weapon{}, instance{};
        std::uint32_t shooter{}, index{};
        constexpr bool operator==(const Identity&) const = default;
    };
    constexpr bool isOwnHeldWeapon(std::uint32_t heldReference, const Identity& owner,
        std::uint32_t hitReference, const Identity& projectile) noexcept
    {
        return heldReference && heldReference == hitReference && owner.weapon && owner.shooter && owner == projectile;
    }
    bool install() noexcept;
    // Values only. Pointer-sized identities are compared, never dereferenced.
    void publish(std::uint32_t heldReference, std::uint32_t shooterHandle,
        std::uintptr_t weapon, std::uintptr_t instance, std::uint32_t index) noexcept;
    // Frame-thread publication. Native initialization on that same thread
    // leases the exact item/model for its deferred muzzle-effect work.
    void publishEffects(RE::TESObjectREFR* reference, RE::NiAVObject* root,
        RE::NiAVObject* muzzle, const RE::EquippedItem& item) noexcept;
    void clearEffects() noexcept;
    void clear() noexcept;
}

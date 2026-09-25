#pragma once
#include <cstdint>
#include <span>
namespace RE { class AIProcess; class EquippedItem; class TESObjectREFR; class NiAVObject; }

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
    void publishBodies(std::uint32_t slot, std::span<const std::uint32_t> bodies) noexcept;
    // Game-thread publication. Native queued projectile initialization copies
    // strong leases before entering its exact private indexed context.
    void publishContext(std::uint32_t slot, RE::AIProcess* process, const RE::EquippedItem& item,
        RE::TESObjectREFR* reference, RE::NiAVObject* muzzle);
    void withdrawContext(std::uint32_t slot) noexcept;
    void clear(std::uint32_t slot) noexcept;
}

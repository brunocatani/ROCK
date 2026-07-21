#pragma once

#include <cstdint>

namespace rock::native_scope_sight_anchor_policy
{
    struct PublicationIdentity
    {
        std::uint64_t weaponGenerationKey{ 0 };
        std::uint64_t equippedWeaponOwnershipKey{ 0 };
        std::uint32_t weaponFormID{ 0 };
    };

    /*
     * Generated bodies can remain published while a replacement weapon is
     * staged. The reusable first-person Weapon node is therefore not an
     * ownership witness. Scope geometry is usable only when its body-set
     * publication still belongs to the exact equipped instance and form.
     */
    [[nodiscard]] inline constexpr bool matchesCurrentEquippedWeapon(
        const PublicationIdentity& published,
        const PublicationIdentity& current) noexcept
    {
        return published.weaponGenerationKey != 0 &&
               published.equippedWeaponOwnershipKey != 0 &&
               published.weaponFormID != 0 &&
               published.weaponGenerationKey == current.weaponGenerationKey &&
               published.equippedWeaponOwnershipKey == current.equippedWeaponOwnershipKey &&
               published.weaponFormID == current.weaponFormID;
    }
}

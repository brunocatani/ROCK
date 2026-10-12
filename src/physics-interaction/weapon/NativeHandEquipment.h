#pragma once

#include <array>
#include <cstdint>

namespace RE
{
    class TESObjectWEAP;
    class TBO_InstanceData;
    class NiAVObject;
}

namespace rock::native_hand_equipment
{
    struct EquippedView
    {
        RE::TESObjectWEAP* weapon = nullptr;
        RE::TBO_InstanceData* instance = nullptr;
        RE::NiAVObject* root = nullptr;
        RE::NiAVObject* bodyRoot = nullptr;
        std::uintptr_t data = 0;
        std::uint32_t slot = 0;
    };
    // Main-frame borrowed views, indexed by native physical right/left slot.
    // No caller may retain these pointers across native calls or frames.
    [[nodiscard]] std::array<EquippedView, 2> equippedViews() noexcept;
    [[nodiscard]] bool hasSecondaryGun() noexcept;
    // Borrowed for this synchronous native call. Uses the native bool biped
    // getter; avoids the incorrectly ordered CommonLib virtual overloads.
    [[nodiscard]] const void* playerBiped(bool firstPerson) noexcept;
    // Native race/equip-index attachment, borrowed from the first-person biped.
    // This is the complete Weapon/WeaponLeft frame, not one model record.
    [[nodiscard]] RE::NiAVObject* resolveAttachment(std::uint32_t index) noexcept;

    // Native Pip-Boy input/transaction hooks. No firearm or world-reference
    // execution is introduced here; equipment remains on the player actor.
    [[nodiscard]] bool install() noexcept;

    // Frame owner only. Records changes in the two native equipment/model
    // identities after a hand-directed transaction, without retaining objects.
    void observe() noexcept;
}

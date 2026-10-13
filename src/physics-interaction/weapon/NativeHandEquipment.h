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
    // Main-frame borrowed views, indexed by native equipment index (0: the
    // player's own graph; 1: ROCK's second graph). An index is not a hand.
    // No caller may retain these pointers across native calls or frames.
    [[nodiscard]] std::array<EquippedView, 2> equippedViews() noexcept;
    [[nodiscard]] bool hasSecondaryGun() noexcept;
    // Physical hand holding the weapon at native index 1. Frame owner only.
    // The hand chosen by a Pip-Boy equip or cross-hand move is kept for that
    // exact item; an index-1 weapon without one (save load, script equip)
    // takes the hand that does not fire index 0.
    [[nodiscard]] bool secondaryHandIsLeft() noexcept;
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

    // The last accepted Pip-Boy equip or cross-hand move that gave the weapon
    // at native index 0 a physical hand. Numeric identity only; the frame
    // owner compares it with the observed primary and moves its firing hand.
    struct DirectedEquip
    {
        std::uint32_t form = 0;
        std::uintptr_t instance = 0;
        bool left = false;
        std::uint64_t sequence = 0; // 0: none since load.
    };
    [[nodiscard]] DirectedEquip lastDirectedPrimaryEquip() noexcept;
    // Pip-Boy display (UI thread): the physical hand holding this exact worn
    // weapon item, or None when it is neither equipped weapon.
    enum class WornHand : std::uint8_t { None, Right, Left };
    [[nodiscard]] WornHand wornWeaponHand(std::uint32_t form, std::uintptr_t instance) noexcept;

    // Frame owner: the primary now fires from the directed hand. Until then a
    // later Pip-Boy press decides with the directed hand, not the old one.
    void noteDirectedPrimaryApplied(std::uint64_t sequence) noexcept;
}

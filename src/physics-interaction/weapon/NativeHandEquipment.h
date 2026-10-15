#pragma once

#include <array>
#include <cstdint>

namespace RE
{
    class Actor;
    class BGSEquipSlot;
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
    // Equips one exact inventory item into native index 1 for the physical
    // hand `left`: `equip` performs the caller's native equip call with the
    // LeftHand leaf it receives, inside the same execution-slot scope as a
    // Pip-Boy equip. Synchronous; true when `equip` reported acceptance.
    using SecondaryEquipCall = bool (*)(const RE::BGSEquipSlot* slot, void* context);
    [[nodiscard]] bool equipIntoSecondary(RE::Actor* actor, RE::TESObjectWEAP* weapon, RE::TBO_InstanceData* instance,
        bool left, SecondaryEquipCall equip, void* context) noexcept;

    // True on the thread running one of ROCK's synchronous native equips
    // into equipment index 1 (LeftHand leaf) for `actor`.
    [[nodiscard]] bool secondaryEquipInProgress(const RE::Actor* actor) noexcept;

    // Pip-Boy display (UI thread): the physical hand holding the weapon of
    // base form `form` in the row's native slot (0 or 1). The other index
    // is used only when the slot wears a different form; None when neither
    // index wears that form.
    enum class WornHand : std::uint8_t { None, Right, Left };
    [[nodiscard]] WornHand wornRowHand(std::uint32_t form, int slot) noexcept;

    // The hand holding an index-0 item: its firing hand, or with `firing`
    // false the hand carrying it by a part grip.
    struct PrimaryHand
    {
        std::uint32_t form = 0;
        std::uintptr_t instance = 0;
        bool left = false;
        bool firing = true;
    };
    // Frame owner, outside menus: the settled hand holding this index-0
    // item. Menus release the live grip; this record keeps the hand for
    // Pip-Boy display and allocation.
    void recordPrimaryHand(std::uint32_t form, std::uintptr_t instance, bool left, bool firing) noexcept;

    // Frame owner: the item now at index 0 fires from `left`'s hand, as a
    // Pip-Boy directed equip would (used when the second weapon is promoted).
    void directPrimaryHand(std::uint32_t form, std::uintptr_t instance, bool left) noexcept;

    // Frame owner: the primary now fires from the directed hand. Until then a
    // later Pip-Boy press decides with the directed hand, not the old one.
    void noteDirectedPrimaryApplied(std::uint64_t sequence) noexcept;
}

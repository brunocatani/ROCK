#pragma once

namespace RE { class TESObjectREFR; class BGSEquipSlot; }
namespace rock::native_inventory_hand_slots
{
    // Synchronous native inventory-owner scope. Never changes a form, an
    // instance's authored equip slot, or a serialized inventory flag directly.
    class Scope
    {
        bool _previous;
    public:
        explicit Scope(const RE::TESObjectREFR* owner) noexcept;
        ~Scope();
        Scope(const Scope&)=delete;
        Scope& operator=(const Scope&)=delete;
    };
    bool install() noexcept;
    // A multi-hand firearm slot (EitherHand, BothHands, BothHandsLeftOptional
    // or a mod's own) that names the right hand leaf. A firearm on such a
    // slot is held on the right leaf: left as authored, its left leaf (direct
    // or through LeftHandOptional) makes a second weapon's left-leaf equip
    // unequip it in the native conflict pass.
    [[nodiscard]] bool heldOnRightLeaf(const RE::BGSEquipSlot* slot) noexcept;
    // Existing ROCK drop/transfer callers validate this same native entry.
    // Accept only our exact forwarding hook after its original was validated.
    bool ownsNativeUnequipEntry() noexcept;
}

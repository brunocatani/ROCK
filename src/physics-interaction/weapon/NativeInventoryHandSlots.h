#pragma once

namespace RE { class TESObjectREFR; }
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
    // Existing ROCK drop/transfer callers validate this same native entry.
    // Accept only our exact forwarding hook after its original was validated.
    bool ownsNativeUnequipEntry() noexcept;
}

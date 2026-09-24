#pragma once

#include "RE/Bethesda/Actor.h"

namespace rock::native_carried_weapon_context
{
    // Only an explicit synchronous carried action exposes its private record
    // to native indexed accessors. Player equip/animation processing never
    // sees a second firearm in its persistent equipment array.
    bool install() noexcept;
    RE::NiPointer<RE::EquippedItemData> create(RE::Actor* actor,
        const RE::BGSObjectInstance& weapon, std::uint32_t index) noexcept;

    class Scope
    {
    public:
        Scope(RE::AIProcess* process, const RE::EquippedItem& item) noexcept;
        ~Scope();
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
    private:
        RE::AIProcess* _previousProcess{};
        const RE::EquippedItem* _previousItem{};
    };

    const RE::EquippedItem* current(RE::AIProcess* process, std::uint32_t index) noexcept;
}

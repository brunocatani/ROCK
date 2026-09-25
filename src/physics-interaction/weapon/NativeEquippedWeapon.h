#pragma once

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::native_equipped_weapon
{
    // A native inventory equipment record. Hands are deliberately absent from
    // this identity: a handoff must not replace the item or its magazine.
    struct Identity
    {
        std::uint32_t form{}, index{};
        std::uintptr_t instance{}, data{};
        bool operator==(const Identity&) const = default;
    };

    struct Snapshot
    {
        RE::EquippedItem item{RE::BGSObjectInstance(nullptr, nullptr)};
        RE::BSTSmartPointer<RE::BipedAnim> biped{};
        RE::NiPointer<RE::NiAVObject> model{};
        RE::NiPointer<RE::NiNode> node{};
        std::uint32_t modelSlot{UINT32_MAX};
        Identity identity{};
        bool equipped{}, attached{};
    };

    [[nodiscard]] bool ready() noexcept;
    [[nodiscard]] bool read(std::uint32_t index, Snapshot& result) noexcept;
    [[nodiscard]] bool matches(const Identity& identity) noexcept;
    [[nodiscard]] bool slotEmpty(std::uint32_t index) noexcept;
    [[nodiscard]] RE::BGSEquipSlot* handSlot(std::uint32_t index) noexcept;
    [[nodiscard]] std::uint16_t inventorySlotMask(const RE::BGSObjectInstance& item,
        const RE::BGSEquipSlot* slot) noexcept;
    [[nodiscard]] bool requestAttach(const Identity& identity) noexcept;
    [[nodiscard]] bool restoreMagazine(const Identity& identity, std::uint32_t ammo,
        std::uint32_t loaded) noexcept;
}

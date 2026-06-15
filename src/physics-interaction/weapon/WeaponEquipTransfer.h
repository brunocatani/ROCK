#pragma once

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include <cstdint>

namespace RE
{
    class TESObjectREFR;
    class TESObjectWEAP;
}

namespace rock::weapon_equip_transfer
{
    enum class EquipReason : std::uint8_t
    {
        NotAttempted = 0,
        MissingRef,
        MissingPlayer,
        MissingEquipManager,
        DeletedOrDisabled,
        PlayerRef,
        MissingBaseForm,
        UnsupportedBaseForm,
        MissingEquipSlot,
        ActivateRefFailed,
        MissingInventoryList,
        InventoryStackNotFound,
        EquipObjectFailed,
        ActivateRefThenEquipObject,
    };

    enum class DropReason : std::uint8_t
    {
        NotAttempted = 0,
        MissingPlayer,
        MissingEquippedWeapon,
        UnsupportedEquippedForm,
        MissingInventoryList,
        InventoryStackNotFound,
        RemoveItemFailed,
        DroppedReferenceUnavailable,
        Dropped,
    };

    struct EquipInput
    {
        RE::TESObjectREFR* heldRef = nullptr;
        bool playSounds = true;
    };

    struct EquipResult
    {
        bool attempted = false;
        bool transferredToInventory = false;
        bool success = false;
        bool matchedInstanceData = false;
        EquipReason reason = EquipReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        std::uint32_t stackID = 0;
        RE::TESObjectWEAP* weapon = nullptr;
    };

    struct EquippedDropInput
    {
        RE::NiPoint3 dropLoc{};
        bool hasDropLoc{ false };
    };

    struct EquippedDropResult
    {
        bool attempted{ false };
        bool success{ false };
        bool matchedInstanceData{ false };
        DropReason reason{ DropReason::NotAttempted };
        std::int32_t count{ 1 };
        std::uint32_t formID{ 0 };
        std::uint32_t stackID{ 0 };
        std::uint32_t droppedFormID{ 0 };
        RE::TESObjectWEAP* weapon{ nullptr };
        RE::ObjectRefHandle handle{};
        RE::NiPointer<RE::TESObjectREFR> droppedRef{};
    };

    [[nodiscard]] const char* equipReasonName(EquipReason reason) noexcept;
    [[nodiscard]] const char* dropReasonName(DropReason reason) noexcept;
    [[nodiscard]] EquipResult transferHeldWeaponToPlayerAndEquip(const EquipInput& input) noexcept;
    [[nodiscard]] EquippedDropResult dropEquippedWeaponFromPlayer(const EquippedDropInput& input) noexcept;
}

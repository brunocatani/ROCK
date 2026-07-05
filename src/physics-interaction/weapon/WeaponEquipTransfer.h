#pragma once

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiAVObject.h"
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
        EquippedWeaponMismatch,
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
        bool usedImmediateEquip = false;
        EquipReason reason = EquipReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        std::uint32_t observedEquippedFormID = 0;
        std::uint32_t stackID = 0;
        RE::TESObjectWEAP* weapon = nullptr;
        /*
         * Loose weapon 3D captured before ActivateRef. The engine detaches it
         * from the scene graph synchronously during pickup but only releases
         * its own reference; this pointer keeps the assembled model alive so
         * the caller can bridge the visual gap until the equipped 3D attaches.
         */
        RE::NiPointer<RE::NiAVObject> detachedWorldModel{};
    };

    enum class UnequipReason : std::uint8_t
    {
        NotAttempted = 0,
        MissingPlayer,
        MissingEquipManager,
        MissingEquippedWeapon,
        MissingInventoryList,
        InventoryStackNotFound,
        MissingEquipSlot,
        UnequipObjectFailed,
        Unequipped,
    };

    struct EquippedDropInput
    {
        RE::NiPoint3 dropLoc{};
        // Reference Euler radians (nifskope convention) for the spawned ref.
        RE::NiPoint3 dropRot{};
        bool hasDropLoc{ false };
        bool hasDropRot{ false };
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

    struct EquippedUnequipInput
    {
        bool playSounds{ true };
    };

    /*
     * Unequip stows the weapon into the already-owned inventory stack, so
     * unlike drop it never creates a world reference. On failure the weapon
     * simply stays equipped -- callers fail closed by doing nothing.
     */
    struct EquippedUnequipResult
    {
        bool attempted{ false };
        bool success{ false };
        bool matchedInstanceData{ false };
        UnequipReason reason{ UnequipReason::NotAttempted };
        std::uint32_t formID{ 0 };
        std::uint32_t stackID{ 0 };
        RE::TESObjectWEAP* weapon{ nullptr };
    };

    [[nodiscard]] const char* equipReasonName(EquipReason reason) noexcept;
    [[nodiscard]] const char* dropReasonName(DropReason reason) noexcept;
    [[nodiscard]] const char* unequipReasonName(UnequipReason reason) noexcept;
    [[nodiscard]] EquipResult transferHeldWeaponToPlayerAndEquip(const EquipInput& input) noexcept;
    [[nodiscard]] EquippedDropResult dropEquippedWeaponFromPlayer(const EquippedDropInput& input) noexcept;
    [[nodiscard]] EquippedUnequipResult unequipEquippedWeaponFromPlayer(const EquippedUnequipInput& input) noexcept;
}

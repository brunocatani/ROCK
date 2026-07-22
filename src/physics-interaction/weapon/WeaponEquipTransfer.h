#pragma once

#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiSmartPointer.h"

#include "physics-interaction/native/HeldWeaponInstantTransition.h"
#include "physics-interaction/weapon/WeaponInventoryStackSelectionPolicy.h"

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
        InstantTransitionUnavailable,
        InvalidNativeActionTrace,
        EquippedIdentityMismatch,
        EquippedStackMismatch,
        ActivateRefThenInstantEquip,
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
        // Sole ROCK ownership of the released world reference. The transfer
        // consumes this lease immediately after ActivateRef succeeds so the
        // native equip/draw path never runs with the picked ref artificially
        // alive.
        RE::NiPointer<RE::TESObjectREFR> heldRef{};
        held_weapon_instant_transition::RequestReason transitionReason{
            held_weapon_instant_transition::RequestReason::SameHandTrigger
        };
    };

    struct EquipResult
    {
        bool attempted = false;
        bool transferredToInventory = false;
        // The one immediate manager call was accepted, its exact equipped
        // identity/stack was observed synchronously, and its intercepted
        // native action trace authorizes the caller's final draw completion.
        bool success = false;
        bool committed = false;
        bool matchedInstanceData = false;
        bool matchedEquippedStack = false;
        bool usedImmediateEquip = false;
        EquipReason reason = EquipReason::NotAttempted;
        std::int32_t count = 1;
        std::uint32_t formID = 0;
        std::uint32_t previousEquippedFormID = 0;
        std::uint32_t observedEquippedFormID = 0;
        std::uintptr_t observedEquippedInstanceData = 0;
        std::uint32_t observedEquipIndex = 0;
        std::uint32_t stackID = 0;
        std::uint32_t preTransferStackCount = 0;
        std::uint32_t postTransferStackCount = 0;
        std::uint32_t stackMutationCandidateCount = 0;
        std::uintptr_t previousEquippedInstanceData = 0;
        std::uintptr_t requestedInstanceData = 0;
        weapon_inventory_stack_selection_policy::Evidence stackSelectionEvidence =
            weapon_inventory_stack_selection_policy::Evidence::None;
        held_weapon_instant_transition::ImmediateEquipResult instantTransition{};
        RE::TESObjectWEAP* weapon = nullptr;
        // Present only when native pickup did not acquire the released world
        // reference. This keeps failure recovery and release events safe.
        RE::NiPointer<RE::TESObjectREFR> untransferredRef{};
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
    [[nodiscard]] EquipResult transferHeldWeaponToPlayerAndEquip(EquipInput input) noexcept;
    [[nodiscard]] EquippedDropResult dropEquippedWeaponFromPlayer(const EquippedDropInput& input) noexcept;
    [[nodiscard]] EquippedUnequipResult unequipEquippedWeaponFromPlayer(const EquippedUnequipInput& input) noexcept;
}

#include "physics-interaction/weapon/WeaponEquipTransfer.h"

#include "physics-interaction/stash/ShoulderStashTransfer.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include "rock_support/Fo4VrRuntime.h"

#include <utility>

namespace rock::weapon_equip_transfer
{
    namespace
    {
        struct InventoryWeaponStack
        {
            bool found = false;
            bool matchedInstanceData = false;
            std::uint32_t stackID = 0;
            std::uint32_t count = 0;
            RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
            RE::BGSEquipSlot* equipSlot = nullptr;
        };

        struct EquippedWeaponSnapshot
        {
            RE::TESObjectWEAP* weapon = nullptr;
            RE::TBO_InstanceData* instanceData = nullptr;
        };

        [[nodiscard]] RE::TESObjectWEAP* asWeaponForm(RE::TESForm* form) noexcept
        {
            if (!form || form->formType != RE::ENUM_FORM_ID::kWEAP) {
                return nullptr;
            }

            return form->As<RE::TESObjectWEAP>();
        }

        [[nodiscard]] EquippedWeaponSnapshot readEquippedWeaponSnapshot() noexcept
        {
            EquippedWeaponSnapshot snapshot{};
            auto* equipData = f4vr::getEquippedItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            snapshot.weapon = asWeaponForm(weaponForm);
            snapshot.instanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            return snapshot;
        }

        [[nodiscard]] RE::BSTSmartPointer<RE::TBO_InstanceData> resolveReferenceInstanceData(RE::TESObjectREFR* refr) noexcept
        {
            if (!refr || !refr->extraList) {
                return {};
            }

            const auto* instanceExtra = refr->extraList->GetByType<RE::ExtraInstanceData>();
            return instanceExtra ? instanceExtra->data : RE::BSTSmartPointer<RE::TBO_InstanceData>{};
        }

        [[nodiscard]] InventoryWeaponStack findTransferredWeaponStack(
            RE::PlayerCharacter* player,
            RE::TESObjectWEAP* weapon,
            const RE::BSTSmartPointer<RE::TBO_InstanceData>& expectedInstanceData) noexcept
        {
            InventoryWeaponStack firstCandidate{};
            InventoryWeaponStack fallback{};
            std::uint32_t candidateCount = 0;
            if (!player || !weapon || !player->inventoryList) {
                return fallback;
            }

            const RE::BSAutoReadLock inventoryLock{ player->inventoryList->rwLock };
            for (auto& inventoryItem : player->inventoryList->data) {
                if (inventoryItem.object != weapon) {
                    continue;
                }

                std::uint32_t stackID = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackID) {
                    RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
                    if (stack->extra) {
                        if (const auto* instanceExtra = stack->extra->GetByType<RE::ExtraInstanceData>()) {
                            instanceData = instanceExtra->data;
                        }
                    }
                    auto* equipSlot = weapon->GetEquipSlot(instanceData.get());
                    if (!equipSlot) {
                        equipSlot = weapon->GetEquipSlot(nullptr);
                    }
                    InventoryWeaponStack candidate{
                        .found = true,
                        .matchedInstanceData = expectedInstanceData && instanceData.get() == expectedInstanceData.get(),
                        .stackID = stackID,
                        .count = stack->GetCount(),
                        .instanceData = instanceData,
                        .equipSlot = equipSlot,
                    };

                    if (candidate.matchedInstanceData) {
                        return candidate;
                    }

                    ++candidateCount;
                    if (!firstCandidate.found) {
                        firstCandidate = candidate;
                    }
                    const bool fallbackMatches = expectedInstanceData || !instanceData;
                    if (fallbackMatches && !fallback.found) {
                        fallback = candidate;
                    }
                }
            }

            if (expectedInstanceData && candidateCount != 1u) {
                return {};
            }
            if (!fallback.found && candidateCount == 1u) {
                return firstCandidate;
            }
            return fallback;
        }

        [[nodiscard]] InventoryWeaponStack findEquippedWeaponStack(
            RE::PlayerCharacter* player,
            RE::TESObjectWEAP* weapon,
            const RE::TBO_InstanceData* expectedInstanceData) noexcept
        {
            InventoryWeaponStack fallback{};
            std::uint32_t equippedCandidateCount = 0;
            if (!player || !weapon || !player->inventoryList) {
                return fallback;
            }

            const RE::BSAutoReadLock inventoryLock{ player->inventoryList->rwLock };
            for (auto& inventoryItem : player->inventoryList->data) {
                if (inventoryItem.object != weapon) {
                    continue;
                }

                std::uint32_t stackID = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackID) {
                    if (!stack->IsEquipped()) {
                        continue;
                    }

                    ++equippedCandidateCount;
                    RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
                    if (stack->extra) {
                        if (const auto* instanceExtra = stack->extra->GetByType<RE::ExtraInstanceData>()) {
                            instanceData = instanceExtra->data;
                        }
                    }

                    InventoryWeaponStack candidate{
                        .found = true,
                        .matchedInstanceData = expectedInstanceData && instanceData.get() == expectedInstanceData,
                        .stackID = stackID,
                        .count = stack->GetCount(),
                        .instanceData = instanceData,
                        .equipSlot = weapon->GetEquipSlot(instanceData.get()),
                    };
                    if (!candidate.equipSlot) {
                        candidate.equipSlot = weapon->GetEquipSlot(nullptr);
                    }

                    if (candidate.matchedInstanceData) {
                        return candidate;
                    }
                    if (!fallback.found) {
                        fallback = candidate;
                    }
                }
            }

            if (expectedInstanceData && equippedCandidateCount != 1u) {
                return {};
            }
            return fallback;
        }

    }

    const char* equipReasonName(EquipReason reason) noexcept
    {
        switch (reason) {
        case EquipReason::MissingRef:
            return "missing-ref";
        case EquipReason::MissingPlayer:
            return "missing-player";
        case EquipReason::MissingEquipManager:
            return "missing-equip-manager";
        case EquipReason::DeletedOrDisabled:
            return "deleted-or-disabled";
        case EquipReason::PlayerRef:
            return "player-ref";
        case EquipReason::MissingBaseForm:
            return "missing-base-form";
        case EquipReason::UnsupportedBaseForm:
            return "unsupported-base-form";
        case EquipReason::MissingEquipSlot:
            return "missing-equip-slot";
        case EquipReason::ActivateRefFailed:
            return "activate-ref-failed";
        case EquipReason::MissingInventoryList:
            return "missing-inventory-list";
        case EquipReason::InventoryStackNotFound:
            return "inventory-stack-not-found";
        case EquipReason::EquipObjectFailed:
            return "equip-object-failed";
        case EquipReason::EquippedWeaponMismatch:
            return "equipped-weapon-mismatch";
        case EquipReason::ActivateRefThenEquipObject:
            return "activate-ref-equip-object";
        default:
            return "not-attempted";
        }
    }

    const char* dropReasonName(DropReason reason) noexcept
    {
        switch (reason) {
        case DropReason::MissingPlayer:
            return "missing-player";
        case DropReason::MissingEquippedWeapon:
            return "missing-equipped-weapon";
        case DropReason::UnsupportedEquippedForm:
            return "unsupported-equipped-form";
        case DropReason::MissingInventoryList:
            return "missing-inventory-list";
        case DropReason::InventoryStackNotFound:
            return "inventory-stack-not-found";
        case DropReason::RemoveItemFailed:
            return "remove-item-failed";
        case DropReason::DroppedReferenceUnavailable:
            return "dropped-reference-unavailable";
        case DropReason::Dropped:
            return "dropped";
        default:
            return "not-attempted";
        }
    }

    const char* unequipReasonName(UnequipReason reason) noexcept
    {
        switch (reason) {
        case UnequipReason::MissingPlayer:
            return "missing-player";
        case UnequipReason::MissingEquipManager:
            return "missing-equip-manager";
        case UnequipReason::MissingEquippedWeapon:
            return "missing-equipped-weapon";
        case UnequipReason::MissingInventoryList:
            return "missing-inventory-list";
        case UnequipReason::InventoryStackNotFound:
            return "inventory-stack-not-found";
        case UnequipReason::MissingEquipSlot:
            return "missing-equip-slot";
        case UnequipReason::UnequipObjectFailed:
            return "unequip-object-failed";
        case UnequipReason::Unequipped:
            return "unequipped";
        default:
            return "not-attempted";
        }
    }

    EquipResult transferHeldWeaponToPlayerAndEquip(EquipInput input) noexcept
    {
        EquipResult result{};
        auto* heldRef = input.heldRef.get();
        result.untransferredRef = std::move(input.heldRef);
        if (!heldRef) {
            result.reason = EquipReason::MissingRef;
            return result;
        }

        result.formID = heldRef->GetFormID();
        result.count = shoulder_stash::resolveReferenceStackCount(heldRef);
        if (heldRef->IsDeleted() || heldRef->IsDisabled()) {
            result.reason = EquipReason::DeletedOrDisabled;
            return result;
        }

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = EquipReason::MissingPlayer;
            return result;
        }

        if (heldRef == player) {
            result.reason = EquipReason::PlayerRef;
            return result;
        }

        auto* baseForm = heldRef->GetObjectReference();
        if (!baseForm) {
            result.reason = EquipReason::MissingBaseForm;
            return result;
        }

        result.weapon = baseForm->As<RE::TESObjectWEAP>();
        if (!result.weapon) {
            result.reason = EquipReason::UnsupportedBaseForm;
            return result;
        }

        auto* equipManager = RE::ActorEquipManager::GetSingleton();
        if (!equipManager) {
            result.reason = EquipReason::MissingEquipManager;
            return result;
        }

        if (!player->inventoryList) {
            result.reason = EquipReason::MissingInventoryList;
            return result;
        }

        const auto expectedInstanceData = resolveReferenceInstanceData(heldRef);
        result.attempted = true;
        /*
         * Capture the loose 3D before ActivateRef. The pickup path detaches it
         * from the scene graph synchronously inside this call (DetachHavok +
         * Set3D(nullptr) via the inline-processed remove task; see
         * docs/research/2026-07-04-loose-to-equipped-weapon-visual-gap.md),
         * but that teardown only releases the ref's own ownership — this
         * NiPointer keeps the assembled model alive for the visual bridge.
         */
        result.detachedWorldModel.reset(heldRef->Get3D());
        const bool activated = heldRef->ActivateRef(player, nullptr, result.count, false, false, false);
        if (!activated) {
            result.reason = EquipReason::ActivateRefFailed;
            return result;
        }
        result.transferredToInventory = true;
        /*
         * ActivateRef has synchronously acquired the item for player
         * inventory. Drop ROCK's last TESObjectREFR lease before stack lookup,
         * EquipObject, draw actions, or visual-bridge setup. Retaining the
         * consumed world reference into those stages can postpone Bethesda's
         * pickup teardown and leave the inventory stack equipped without a
         * durable first-person scene instance.
         */
        result.untransferredRef.reset();

        const auto stack = findTransferredWeaponStack(player, result.weapon, expectedInstanceData);
        if (!stack.found) {
            result.reason = EquipReason::InventoryStackNotFound;
            return result;
        }
        if (!stack.equipSlot) {
            result.reason = EquipReason::MissingEquipSlot;
            return result;
        }

        result.stackID = stack.stackID;
        result.matchedInstanceData = stack.matchedInstanceData;
        RE::BGSObjectInstance objectInstance(result.weapon, stack.instanceData.get());
        /*
         * a_queueEquip=false takes the engine's immediate DoEquip inside this
         * call (raw disasm: EquipObject 0x140e6fea0 stores the flag at
         * params+0x18, 0x140e71920 branches on it). Weapons on the player
         * otherwise always defer through the middleProcess pending-equip list,
         * adding visible frames before the draw can start. A keyword-gated
         * engine pre-check can veto the immediate path for special items, so
         * fall back to the legacy queued call when the immediate one fails.
         */
        bool equipped = equipManager->EquipObject(player,
            objectInstance,
            stack.stackID,
            1,
            stack.equipSlot,
            false,
            false,
            input.playSounds,
            true,
            false);
        result.usedImmediateEquip = equipped;
        if (!equipped) {
            equipped = equipManager->EquipObject(player,
                objectInstance,
                stack.stackID,
                1,
                stack.equipSlot,
                true,
                false,
                input.playSounds,
                true,
                false);
        }
        if (!equipped) {
            result.reason = EquipReason::EquipObjectFailed;
            return result;
        }

        const auto equippedAfter = readEquippedWeaponSnapshot();
        result.observedEquippedFormID = equippedAfter.weapon ? equippedAfter.weapon->GetFormID() : 0;
        if (equippedAfter.weapon != result.weapon) {
            result.reason = EquipReason::EquippedWeaponMismatch;
            return result;
        }

        result.success = true;
        result.reason = EquipReason::ActivateRefThenEquipObject;
        return result;
    }

    EquippedDropResult dropEquippedWeaponFromPlayer(const EquippedDropInput& input) noexcept
    {
        EquippedDropResult result{};

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = DropReason::MissingPlayer;
            return result;
        }

        const auto equipped = readEquippedWeaponSnapshot();
        result.weapon = equipped.weapon;
        result.formID = equipped.weapon ? equipped.weapon->GetFormID() : 0;
        if (!equipped.weapon) {
            result.reason = DropReason::MissingEquippedWeapon;
            return result;
        }

        if (!player->inventoryList) {
            result.reason = DropReason::MissingInventoryList;
            return result;
        }

        const auto stack = findEquippedWeaponStack(player, equipped.weapon, equipped.instanceData);
        if (!stack.found || stack.count == 0) {
            result.reason = DropReason::InventoryStackNotFound;
            return result;
        }

        result.attempted = true;
        result.count = 1;
        result.stackID = stack.stackID;
        result.matchedInstanceData = stack.matchedInstanceData;

        RE::TESObjectREFR::RemoveItemData removeData(equipped.weapon, result.count);
        removeData.reason = RE::ITEM_REMOVE_REASON::KDropping;
        if (input.hasDropLoc) {
            removeData.dropLoc = &input.dropLoc;
        }
        if (input.hasDropRot) {
            removeData.rotate = &input.dropRot;
        }
        removeData.stackData.push_back(stack.stackID);

        result.handle = player->RemoveItem(removeData);
        if (!result.handle) {
            result.reason = DropReason::RemoveItemFailed;
            return result;
        }

        result.droppedRef = result.handle.get();
        if (!result.droppedRef) {
            result.reason = DropReason::DroppedReferenceUnavailable;
            return result;
        }

        result.success = true;
        result.reason = DropReason::Dropped;
        result.droppedFormID = result.droppedRef->GetFormID();
        return result;
    }

    EquippedUnequipResult unequipEquippedWeaponFromPlayer(const EquippedUnequipInput& input) noexcept
    {
        EquippedUnequipResult result{};

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = UnequipReason::MissingPlayer;
            return result;
        }

        auto* equipManager = RE::ActorEquipManager::GetSingleton();
        if (!equipManager) {
            result.reason = UnequipReason::MissingEquipManager;
            return result;
        }

        const auto equipped = readEquippedWeaponSnapshot();
        result.weapon = equipped.weapon;
        result.formID = equipped.weapon ? equipped.weapon->GetFormID() : 0;
        if (!equipped.weapon) {
            result.reason = UnequipReason::MissingEquippedWeapon;
            return result;
        }

        if (!player->inventoryList) {
            result.reason = UnequipReason::MissingInventoryList;
            return result;
        }

        const auto stack = findEquippedWeaponStack(player, equipped.weapon, equipped.instanceData);
        if (!stack.found || stack.count == 0) {
            result.reason = UnequipReason::InventoryStackNotFound;
            return result;
        }
        if (!stack.equipSlot) {
            result.reason = UnequipReason::MissingEquipSlot;
            return result;
        }

        result.attempted = true;
        result.stackID = stack.stackID;
        result.matchedInstanceData = stack.matchedInstanceData;

        RE::BGSObjectInstance objectInstance(equipped.weapon, stack.instanceData.get());
        const bool unequipped = equipManager->UnequipObject(player,
            &objectInstance,
            1,
            stack.equipSlot,
            stack.stackID,
            true,
            false,
            input.playSounds,
            true,
            nullptr);
        if (!unequipped) {
            result.reason = UnequipReason::UnequipObjectFailed;
            return result;
        }

        result.success = true;
        result.reason = UnequipReason::Unequipped;
        return result;
    }
}

#include "physics-interaction/weapon/WeaponEquipTransfer.h"

#include "physics-interaction/stash/ShoulderStashTransfer.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

namespace rock::weapon_equip_transfer
{
    namespace
    {
        struct InventoryWeaponStack
        {
            bool found = false;
            bool matchedInstanceData = false;
            std::uint32_t stackID = 0;
            RE::TBO_InstanceData* instanceData = nullptr;
            RE::BGSEquipSlot* equipSlot = nullptr;
        };

        [[nodiscard]] RE::TBO_InstanceData* resolveReferenceInstanceData(RE::TESObjectREFR* refr) noexcept
        {
            if (!refr || !refr->extraList) {
                return nullptr;
            }

            const auto* instanceExtra = refr->extraList->GetByType<RE::ExtraInstanceData>();
            return instanceExtra ? instanceExtra->data.get() : nullptr;
        }

        [[nodiscard]] InventoryWeaponStack findTransferredWeaponStack(
            RE::PlayerCharacter* player,
            RE::TESObjectWEAP* weapon,
            RE::TBO_InstanceData* expectedInstanceData) noexcept
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
                    auto* instanceData = inventoryItem.GetInstanceData(stackID);
                    auto* equipSlot = weapon->GetEquipSlot(instanceData);
                    InventoryWeaponStack candidate{
                        .found = true,
                        .matchedInstanceData = expectedInstanceData && instanceData == expectedInstanceData,
                        .stackID = stackID,
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
                    const bool fallbackMatches = expectedInstanceData || instanceData == nullptr;
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
        case EquipReason::ActivateRefThenEquipObject:
            return "activate-ref-equip-object";
        default:
            return "not-attempted";
        }
    }

    EquipResult transferHeldWeaponToPlayerAndEquip(const EquipInput& input) noexcept
    {
        EquipResult result{};
        auto* heldRef = input.heldRef;
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

        auto* expectedInstanceData = resolveReferenceInstanceData(heldRef);
        result.attempted = true;
        const bool activated = heldRef->ActivateRef(player, nullptr, result.count, false, false, false);
        if (!activated) {
            result.reason = EquipReason::ActivateRefFailed;
            return result;
        }
        result.transferredToInventory = true;

        if (!player->inventoryList) {
            result.reason = EquipReason::MissingInventoryList;
            return result;
        }

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
        RE::BGSObjectInstance objectInstance(result.weapon, stack.instanceData);
        const bool equipped = equipManager->EquipObject(player,
            objectInstance,
            stack.stackID,
            1,
            stack.equipSlot,
            true,
            false,
            true,
            true,
            false);
        if (!equipped) {
            result.reason = EquipReason::EquipObjectFailed;
            return result;
        }

        result.success = true;
        result.reason = EquipReason::ActivateRefThenEquipObject;
        return result;
    }
}

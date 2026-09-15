#include "physics-interaction/weapon/WeaponEquipTransfer.h"

#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/NativeMemory.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/BGSInventoryItem.h"
#include "RE/Bethesda/BSExtraData.h"
#include "RE/Bethesda/BSLock.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESObjectREFRs.h"

#include "rock_support/Fo4VrRuntime.h"
#include <REL/Relocation.h>

#include <array>
#include <utility>

namespace rock::weapon_equip_transfer
{
    namespace
    {
        [[nodiscard]] bool clearPreviousWeaponRestore(RE::PlayerCharacter* player) noexcept
        {
            // FO4VR 1.2.72: Unequip at 140E745F0 snapshots the player's saved
            // one-hand items, clears them at 140E7482D, then re-equips each via
            // 140E7486D -> 140E714A0 -> EquipObject. Intentional ROCK detach
            // must invalidate that restoration before RemoveItem can unequip.
            // Use the native destructor/clear routine (140F7CF70), not the
            // incompatible CommonLib PlayerCharacter::lastOneHandItems layout.
            constexpr std::array<std::uint8_t, 17> expected{
                0x48, 0x89, 0x5C, 0x24, 0x18, 0x56, 0x48, 0x83, 0xEC,
                0x20, 0x48, 0x8D, 0xB1, 0x08, 0x11, 0x00, 0x00,
            };
            if (!player || !REL::Module::IsVR() ||
                REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
            const auto address = REL::Offset(0xF7CF70).address();
            std::array<std::uint8_t, expected.size()> actual{};
            if (!native_memory::guardedCopyFromMemory(
                    reinterpret_cast<const void*>(address), actual.data(), actual.size()) ||
                actual != expected) {
                ROCK_LOG_WARN(Weapon, "Equipped detach refused: previous-weapon restore reset entry validation failed");
                return false;
            }
            using ClearLastOneHandItems = void (*)(RE::PlayerCharacter*);
            reinterpret_cast<ClearLastOneHandItems>(address)(player);
            return true;
        }

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
            std::uint32_t equipIndex = 0;
        };

        struct InventoryWeaponStackSnapshot
        {
            weapon_inventory_stack_selection_policy::Snapshot witnesses{};
            std::array<InventoryWeaponStack,
                weapon_inventory_stack_selection_policy::kMaximumObservedStacks>
                stacks{};
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
            auto* equipData = f4vr::getEquippedWeaponItem();
            auto* weaponForm = equipData ? equipData->item.object : nullptr;
            snapshot.weapon = asWeaponForm(weaponForm);
            snapshot.instanceData = equipData ? equipData->item.instanceData.get() : nullptr;
            snapshot.equipIndex = equipData ? equipData->equipIndex.index : 0;
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

        [[nodiscard]] InventoryWeaponStackSnapshot captureWeaponStacks(
            RE::PlayerCharacter* player,
            RE::TESObjectWEAP* weapon,
            const bool retainRuntimeStacks) noexcept
        {
            InventoryWeaponStackSnapshot snapshot{};
            if (!player || !weapon || !player->inventoryList) {
                return snapshot;
            }

            const RE::BSAutoReadLock inventoryLock{ player->inventoryList->rwLock };
            for (auto& inventoryItem : player->inventoryList->data) {
                if (inventoryItem.object != weapon) {
                    continue;
                }

                std::uint32_t stackID = 0;
                for (auto* stack = inventoryItem.stackData.get(); stack; stack = stack->nextStack.get(), ++stackID) {
                    if (snapshot.witnesses.count >= snapshot.stacks.size()) {
                        snapshot.witnesses.complete = false;
                        return snapshot;
                    }

                    RE::TBO_InstanceData* instanceData = nullptr;
                    if (stack->extra) {
                        if (const auto* instanceExtra = stack->extra->GetByType<RE::ExtraInstanceData>()) {
                            instanceData = instanceExtra->data.get();
                        }
                    }
                    auto* equipSlot = weapon->GetEquipSlot(instanceData);
                    if (!equipSlot) {
                        equipSlot = weapon->GetEquipSlot(nullptr);
                    }
                    const auto snapshotIndex = snapshot.witnesses.count++;
                    snapshot.witnesses.stacks[snapshotIndex] =
                        weapon_inventory_stack_selection_policy::StackWitness{
                            .stackAddress = reinterpret_cast<std::uintptr_t>(stack),
                            .instanceDataAddress = reinterpret_cast<std::uintptr_t>(instanceData),
                            .count = stack->GetCount(),
                        };
                    if (retainRuntimeStacks) {
                        snapshot.stacks[snapshotIndex] = InventoryWeaponStack{
                            .found = true,
                            .stackID = stackID,
                            .count = stack->GetCount(),
                            .instanceData = RE::BSTSmartPointer<RE::TBO_InstanceData>{ instanceData },
                            .equipSlot = equipSlot,
                        };
                    }
                }
            }
            return snapshot;
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
        case EquipReason::InstantTransitionUnavailable:
            return "instant-transition-unavailable";
        case EquipReason::InvalidNativeActionTrace:
            return "invalid-native-action-trace";
        case EquipReason::EquippedIdentityMismatch:
            return "equipped-identity-mismatch";
        case EquipReason::EquippedStackMismatch:
            return "equipped-stack-mismatch";
        case EquipReason::ActivateRefThenInstantEquip:
            return "activate-ref-instant-equip";
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
        case DropReason::PreviousWeaponRestoreResetUnavailable:
            return "previous-weapon-restore-reset-unavailable";
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
        const auto inventoryBeforeTransfer = captureWeaponStacks(
            player,
            result.weapon,
            false);
        result.attempted = true;
        const auto equippedBeforeTransfer = readEquippedWeaponSnapshot();
        result.previousEquippedFormID = equippedBeforeTransfer.weapon ?
            equippedBeforeTransfer.weapon->GetFormID() :
            0;
        result.previousEquippedInstanceData = reinterpret_cast<std::uintptr_t>(
            equippedBeforeTransfer.instanceData);
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

        const auto inventoryAfterTransfer = captureWeaponStacks(
            player,
            result.weapon,
            true);
        const auto stackSelection =
            weapon_inventory_stack_selection_policy::selectTransferredStack(
                inventoryBeforeTransfer.witnesses,
                inventoryAfterTransfer.witnesses,
                reinterpret_cast<std::uintptr_t>(expectedInstanceData.get()));
        result.preTransferStackCount = static_cast<std::uint32_t>(
            inventoryBeforeTransfer.witnesses.count);
        result.postTransferStackCount = static_cast<std::uint32_t>(
            inventoryAfterTransfer.witnesses.count);
        result.stackMutationCandidateCount = static_cast<std::uint32_t>(
            stackSelection.mutationCandidateCount);
        result.stackSelectionEvidence = stackSelection.evidence;
        const auto stack = stackSelection.found ?
            inventoryAfterTransfer.stacks[stackSelection.postIndex] :
            InventoryWeaponStack{};
        if (!stack.found) {
            result.reason = EquipReason::InventoryStackNotFound;
            return result;
        }
        if (!stack.equipSlot) {
            result.reason = EquipReason::MissingEquipSlot;
            return result;
        }

        result.stackID = stack.stackID;
        result.matchedInstanceData = expectedInstanceData &&
            stack.instanceData.get() == expectedInstanceData.get();
        result.requestedInstanceData = reinterpret_cast<std::uintptr_t>(
            stack.instanceData.get());
        RE::BGSObjectInstance objectInstance(result.weapon, stack.instanceData.get());
        /*
         * The native wrapper owns one immediate manager call and suppresses
         * only the verified sheathe/draw action submissions made inside that
         * synchronous transaction. A queued retry cannot rescue an immediate
         * validation failure and would escape the scoped interceptor, so it is
         * deliberately unsupported here.
         */
        result.instantTransition =
            held_weapon_instant_transition::equipImmediatelyWithoutActions(
                held_weapon_instant_transition::ImmediateEquipInput{
                    .manager = equipManager,
                    .player = player,
                    .object = &objectInstance,
                    .stackID = stack.stackID,
                    .equipSlot = stack.equipSlot,
                    .reason = input.transitionReason,
                });
        result.usedImmediateEquip = result.instantTransition.managerAccepted;
        if (!result.instantTransition.managerAccepted) {
            result.reason = result.instantTransition.code ==
                    held_weapon_instant_transition::ImmediateEquipCode::CapabilityUnavailable ?
                EquipReason::InstantTransitionUnavailable :
                EquipReason::EquipObjectFailed;
            return result;
        }
        if (!result.instantTransition.success()) {
            result.reason = EquipReason::InvalidNativeActionTrace;
            return result;
        }

        const auto equippedAfter = readEquippedWeaponSnapshot();
        result.observedEquippedFormID = equippedAfter.weapon ? equippedAfter.weapon->GetFormID() : 0;
        result.observedEquippedInstanceData = reinterpret_cast<std::uintptr_t>(
            equippedAfter.instanceData);
        result.observedEquipIndex = equippedAfter.equipIndex;
        result.committed = equippedAfter.weapon == result.weapon &&
            (!stack.instanceData || equippedAfter.instanceData == stack.instanceData.get());
        if (!result.committed) {
            result.reason = EquipReason::EquippedIdentityMismatch;
            return result;
        }

        const auto equippedStack = findEquippedWeaponStack(
            player,
            result.weapon,
            stack.instanceData.get());
        result.matchedEquippedStack = equippedStack.found &&
            equippedStack.stackID == stack.stackID &&
            (!stack.instanceData ||
                equippedStack.instanceData.get() == stack.instanceData.get());
        if (!result.matchedEquippedStack) {
            result.reason = EquipReason::EquippedStackMismatch;
            return result;
        }
        result.success = true;
        result.reason = EquipReason::ActivateRefThenInstantEquip;
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

        if (!clearPreviousWeaponRestore(player)) {
            result.reason = DropReason::PreviousWeaponRestoreResetUnavailable;
            return result;
        }
        result.handle = player->RemoveItem(removeData);
        const auto equippedAfterDrop = readEquippedWeaponSnapshot();
        ROCK_LOG_INFO(Weapon,
            "Equipped detach native removal: weapon={:08X} stack={} previousRestoreCleared=yes remainingEquipped={:08X} handleValid={}",
            result.formID, result.stackID,
            equippedAfterDrop.weapon ? equippedAfterDrop.weapon->GetFormID() : 0,
            static_cast<bool>(result.handle));
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

}

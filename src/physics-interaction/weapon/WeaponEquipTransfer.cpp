#include "physics-interaction/weapon/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/NativeEquippedWeapon.h"

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
        using UnequipObject = bool (*)(RE::ActorEquipManager*, RE::Actor*, const RE::BGSObjectInstance*,
            std::uint32_t, const RE::BGSEquipSlot*, std::uint32_t, bool, bool, bool, bool, const RE::BGSEquipSlot*);

        [[nodiscard]] UnequipObject validatedUnequipObject() noexcept
        {
            if (!REL::Module::IsVR() ||
                REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return nullptr;
            // VR raw witnesses: 140E70280 builds the synchronous request to
            // 140E745F0; caller 1409CB31F supplies the same stack/slot ABI.
            constexpr std::array<std::uint8_t, 15> expected{
                0x48, 0x8B, 0xC4, 0x48, 0x89, 0x58, 0x18, 0x55,
                0x57, 0x41, 0x56, 0x48, 0x83, 0xEC, 0x70,
            };
            const auto target = REL::Offset(0xE70280).address();
            std::array<std::uint8_t, expected.size()> actual{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(target), actual.data(), actual.size()) ||
                actual != expected) return nullptr;
            return reinterpret_cast<UnequipObject>(target);
        }

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
            std::uintptr_t stackAddress = 0;
            RE::BSTSmartPointer<RE::TBO_InstanceData> instanceData{};
            const RE::BGSEquipSlot* equipSlot = nullptr;
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

        [[nodiscard]] EquippedWeaponSnapshot readEquippedWeaponSnapshot(std::uint32_t nativeIndex = UINT32_MAX) noexcept
        {
            EquippedWeaponSnapshot snapshot{};
            if (nativeIndex != UINT32_MAX) {
                native_equipped_weapon::Snapshot current;
                if (native_equipped_weapon::read(nativeIndex, current)) {
                    snapshot = {asWeaponForm(current.item.item.object), current.item.item.instanceData.get(), nativeIndex};
                }
                return snapshot;
            }
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
                            .stackAddress = reinterpret_cast<std::uintptr_t>(stack),
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
                        .stackAddress = reinterpret_cast<std::uintptr_t>(stack),
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

        InventoryWeaponStack findIndexedEquippedStack(const native_equipped_weapon::Snapshot& equipped) noexcept
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* slot = equipped.item.equipSlot;
            const auto mask = native_equipped_weapon::inventorySlotMask(equipped.item.item, slot);
            if (!player || !player->inventoryList || !mask) return {};
            const RE::BSAutoReadLock lock{player->inventoryList->rwLock};
            InventoryWeaponStack result{};
            std::size_t scanned{};
            for (const auto& entry : player->inventoryList->data) {
                if (++scanned > 16384) return {};
                if (entry.object != equipped.item.item.object) continue;
                std::uint32_t index{};
                auto* stack = entry.stackData.get();
                for (; stack && index < weapon_inventory_stack_selection_policy::kMaximumObservedStacks;
                    stack = stack->nextStack.get(), ++index) {
                    if ((stack->flags.underlying() & mask) == 0 || !stack->GetCount()) continue;
                    RE::BSTSmartPointer<RE::TBO_InstanceData> instance{};
                    if (stack->extra) {
                        if (const auto* extra = stack->extra->GetByType<RE::ExtraInstanceData>()) instance = extra->data;
                    }
                    if (instance.get() != equipped.item.item.instanceData.get()) continue;
                    if (result.found) return {}; // No arbitrary choice among identical equipped copies.
                    result = {.found = true, .matchedInstanceData = true, .stackID = index,
                        .count = stack->GetCount(), .stackAddress = reinterpret_cast<std::uintptr_t>(stack),
                        .instanceData = std::move(instance), .equipSlot = slot};
                }
                return stack ? InventoryWeaponStack{} : result;
            }
            return {};
        }

        EquipResult equipSelectedStack(RE::PlayerCharacter* player, RE::ActorEquipManager* equipManager,
            const InventoryWeaponStack& stack, held_weapon_instant_transition::RequestReason reason,
            EquipResult result, std::uint32_t nativeIndex = UINT32_MAX) noexcept
        {
            auto* slot = nativeIndex == UINT32_MAX ? stack.equipSlot : native_equipped_weapon::handSlot(nativeIndex);
            if (!slot) { result.reason = EquipReason::MissingEquipSlot; return result; }
            result.stackID = stack.stackID;
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
                        .equipSlot = slot,
                        .reason = reason,
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

            auto equippedAfter = readEquippedWeaponSnapshot();
            native_equipped_weapon::Snapshot indexed;
            if (nativeIndex != UINT32_MAX) {
                equippedAfter = {};
                if (native_equipped_weapon::read(nativeIndex, indexed)) {
                    equippedAfter = {asWeaponForm(indexed.item.item.object), indexed.item.item.instanceData.get(), nativeIndex};
                }
            }
            result.observedEquippedFormID = equippedAfter.weapon ? equippedAfter.weapon->GetFormID() : 0;
            result.observedEquippedInstanceData = reinterpret_cast<std::uintptr_t>(
                equippedAfter.instanceData);
            result.observedEquipIndex = equippedAfter.equipIndex;
            result.committed = equippedAfter.weapon == result.weapon &&
                (nativeIndex == UINT32_MAX ? (!stack.instanceData || equippedAfter.instanceData == stack.instanceData.get()) :
                    equippedAfter.instanceData == stack.instanceData.get() && indexed.item.equipSlot == slot);
            if (!result.committed) {
                result.reason = EquipReason::EquippedIdentityMismatch;
                return result;
            }

            const auto equippedStack = nativeIndex != UINT32_MAX ? findIndexedEquippedStack(indexed) : findEquippedWeaponStack(
                player,
                result.weapon,
                stack.instanceData.get());
            result.matchedEquippedStack = equippedStack.found &&
                weapon_inventory_stack_selection_policy::matchesEquippedStack(
                    { stack.stackAddress, reinterpret_cast<std::uintptr_t>(stack.instanceData.get()), stack.count },
                    { equippedStack.stackAddress, reinterpret_cast<std::uintptr_t>(equippedStack.instanceData.get()), equippedStack.count });
            ROCK_LOG_INFO(Weapon,
                "Held equip stack validation weapon={:08X} requestedIndex={} equippedIndex={} found={} identityMatch={} requestedNode=0x{:X} equippedNode=0x{:X} requestedInstance=0x{:X} equippedInstance=0x{:X}",
                result.weapon->formID, stack.stackID, equippedStack.stackID,
                equippedStack.found, result.matchedEquippedStack,
                stack.stackAddress, equippedStack.stackAddress,
                reinterpret_cast<std::uintptr_t>(stack.instanceData.get()),
                reinterpret_cast<std::uintptr_t>(equippedStack.instanceData.get()));
            if (!result.matchedEquippedStack) {
                result.reason = EquipReason::EquippedStackMismatch;
                return result;
            }
            result.success = true;
            result.reason = EquipReason::ActivateRefThenInstantEquip;
            return result;
        }

        InventoryWeaponStack findSelectedInventoryStack(const InventorySelection& selection, bool allowEquipped = false) noexcept
        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            auto* weapon = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(selection.formID);
            if (!selection.stack || !player || !player->inventoryList || !weapon) return {};
            const RE::BSAutoReadLock lock{player->inventoryList->rwLock};
            std::size_t scanned = 0;
            for (const auto& entry : player->inventoryList->data) {
                if (++scanned > 16384) return {};
                if (entry.object != weapon) continue;
                std::uint32_t index = 0;
                for (auto* stack = entry.stackData.get(); stack && index <
                    weapon_inventory_stack_selection_policy::kMaximumObservedStacks; stack = stack->nextStack.get(), ++index) {
                    if (stack != selection.stack.get()) continue;
                    if (!stack->GetCount() || (!allowEquipped && stack->IsEquipped())) return {};
                    RE::BSTSmartPointer<RE::TBO_InstanceData> instance{};
                    if (stack->extra) {
                        if (const auto* extra = stack->extra->GetByType<RE::ExtraInstanceData>()) instance = extra->data;
                    }
                    if (instance.get() != selection.instance.get()) return {};
                    auto* slot = weapon->GetEquipSlot(instance.get());
                    if (!slot) slot = weapon->GetEquipSlot(nullptr);
                    return {.found = true, .matchedInstanceData = true, .stackID = index,
                        .count = stack->GetCount(), .stackAddress = reinterpret_cast<std::uintptr_t>(stack),
                        .instanceData = std::move(instance), .equipSlot = slot};
                }
                return {};
            }
            return {};
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
        case EquipReason::InventoryInstantEquip:
            return "inventory-instant-equip";
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
        case DropReason::UnequipUnavailable:
            return "unequip-unavailable";
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

        result.matchedInstanceData = expectedInstanceData &&
            stack.instanceData.get() == expectedInstanceData.get();
        return equipSelectedStack(player, equipManager, stack, input.transitionReason, std::move(result), input.nativeIndex);
    }

    InventorySelection captureInventoryWeapon(std::uint32_t formID, std::uint32_t stackIndex) noexcept
    {
        return captureInventoryWeapon(formID, stackIndex, false);
    }

    InventorySelection captureInventoryWeapon(std::uint32_t formID, std::uint32_t stackIndex, bool allowEquippedStack) noexcept
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* weapon = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(formID);
        if (!player || !player->inventoryList || !weapon || stackIndex >=
            weapon_inventory_stack_selection_policy::kMaximumObservedStacks) return {};
        const RE::BSAutoReadLock lock{player->inventoryList->rwLock};
        std::size_t scanned = 0;
        for (const auto& entry : player->inventoryList->data) {
            if (++scanned > 16384) return {};
            if (entry.object != weapon) continue;
            auto stack = entry.stackData;
            for (std::uint32_t i = 0; stack && i < stackIndex; ++i) stack = stack->nextStack;
            if (!stack || !stack->GetCount() || (!allowEquippedStack && stack->IsEquipped())) return {};
            RE::BSTSmartPointer<RE::TBO_InstanceData> instance{};
            if (stack->extra) {
                if (const auto* extra = stack->extra->GetByType<RE::ExtraInstanceData>()) instance = extra->data;
            }
            return {formID, stackIndex, std::move(stack), std::move(instance)};
        }
        return {};
    }

    bool inventoryWeaponCurrent(const InventorySelection& selection) noexcept
    {
        return findSelectedInventoryStack(selection).found;
    }

    bool resolveInventoryWeaponStack(const InventorySelection& selection, std::uint32_t& stackIndex) noexcept
    {
        const auto current = findSelectedInventoryStack(selection);
        if (!current.found) return false;
        stackIndex = current.stackID;
        return true;
    }

    InventorySelection captureEquippedInventoryWeapon(std::uint32_t nativeIndex) noexcept
    {
        native_equipped_weapon::Snapshot current;
        if (!native_equipped_weapon::read(nativeIndex, current)) return {};
        const auto stack = findIndexedEquippedStack(current);
        if (!stack.found || stack.instanceData.get() != current.item.item.instanceData.get()) return {};
        return captureInventoryWeapon(current.identity.form, stack.stackID, true);
    }

    namespace
    {
    EquipResult equipInventorySelection(const InventorySelection& selection, std::uint32_t nativeIndex, bool allowEquipped) noexcept
    {
        EquipResult result{};
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* manager = RE::ActorEquipManager::GetSingleton();
        result.weapon = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(selection.formID);
        const auto stack = findSelectedInventoryStack(selection, allowEquipped);
        if (!player || !manager || !result.weapon || !stack.found || !stack.equipSlot) {
            result.reason = EquipReason::InventoryStackNotFound;
            return result;
        }
        const auto before = readEquippedWeaponSnapshot();
        result.previousEquippedFormID = before.weapon ? before.weapon->GetFormID() : 0;
        result.previousEquippedInstanceData = reinterpret_cast<std::uintptr_t>(before.instanceData);
        result.formID = selection.formID;
        result.attempted = true;
        result.matchedInstanceData = stack.instanceData.get() == selection.instance.get();
        result = equipSelectedStack(player, manager, stack,
            held_weapon_instant_transition::RequestReason::InventoryEquip, std::move(result), nativeIndex);
        if (result.success) result.reason = EquipReason::InventoryInstantEquip;
        return result;
    }
    }

    EquipResult equipInventoryWeapon(const InventorySelection& selection) noexcept
    {
        return equipInventorySelection(selection, UINT32_MAX, false);
    }

    EquipResult equipInventoryWeapon(const InventorySelection& selection, std::uint32_t nativeIndex) noexcept
    {
        return equipInventorySelection(selection, nativeIndex, true);
    }

    bool replaceHolsteredWeaponWithUnarmed() noexcept
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* manager = RE::ActorEquipManager::GetSingleton();
        if (!player || !manager || f4vr::getNativeWeaponState(player) != 0) return false;
        const auto equipped = readEquippedWeaponSnapshot();
        if (!equipped.weapon) return true;
        const auto stack = findEquippedWeaponStack(player, equipped.weapon, equipped.instanceData);
        if (!stack.found || !stack.equipSlot ||
            (equipped.instanceData && stack.instanceData.get() != equipped.instanceData)) {
            ROCK_LOG_WARN(Weapon, "Bare fists refused: holstered inventory stack unavailable weapon={:08X}", equipped.weapon->GetFormID());
            return false;
        }
        const auto unequip = validatedUnequipObject();
        if (!unequip || !clearPreviousWeaponRestore(player)) {
            ROCK_LOG_WARN(Weapon, "Bare fists refused: native unequip/previous-weapon validation failed");
            return false;
        }
        RE::BGSObjectInstance object(equipped.weapon, stack.instanceData.get());
        const bool accepted = unequip(manager, player, &object, 1, stack.equipSlot,
            stack.stackID, false, true, false, true, nullptr);
        const auto remaining = readEquippedWeaponSnapshot();
        ROCK_LOG_INFO(Weapon, "Bare fists equipment replacement: previous={:08X} stack={} accepted={} remaining={:08X}",
            equipped.weapon->GetFormID(), stack.stackID, accepted, remaining.weapon ? remaining.weapon->GetFormID() : 0);
        return accepted && !remaining.weapon;
    }

    bool canRecoverHeldEquip() noexcept
    {
        return RE::PlayerCharacter::GetSingleton() && RE::ActorEquipManager::GetSingleton() && validatedUnequipObject();
    }

    bool unequipExactCurrentWeapon(const std::uint32_t formID, const std::uintptr_t instanceData) noexcept
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* manager = RE::ActorEquipManager::GetSingleton();
        const auto equipped = readEquippedWeaponSnapshot();
        if (!equipped.weapon) return true;
        if (!player || !manager || equipped.weapon->GetFormID() != formID ||
            reinterpret_cast<std::uintptr_t>(equipped.instanceData) != instanceData) return false;
        const auto stack = findEquippedWeaponStack(player, equipped.weapon, equipped.instanceData);
        const auto unequip = validatedUnequipObject();
        if (!stack.found || !stack.equipSlot || !stack.count || !unequip ||
            stack.instanceData.get() != equipped.instanceData || !clearPreviousWeaponRestore(player)) return false;
        RE::BGSObjectInstance object(equipped.weapon, stack.instanceData.get());
        const bool accepted = unequip(manager, player, &object, 1, stack.equipSlot, stack.stackID, false, true, false, true, nullptr);
        const auto remaining = readEquippedWeaponSnapshot();
        ROCK_LOG_INFO(Weapon, "Held transfer recovery unequip form={:08X} instance={:#x} accepted={} remaining={:08X}",
            formID, instanceData, accepted, remaining.weapon ? remaining.weapon->GetFormID() : 0u);
        return !remaining.weapon;
    }

    bool unequipExactIndexedWeapon(std::uint32_t nativeIndex, std::uint32_t formID, std::uintptr_t instanceData) noexcept
    {
        native_equipped_weapon::Snapshot current;
        if (!native_equipped_weapon::read(nativeIndex, current)) return native_equipped_weapon::slotEmpty(nativeIndex);
        if (current.identity.form != formID || current.identity.instance != instanceData) return false;
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* manager = RE::ActorEquipManager::GetSingleton();
        const auto stack = findIndexedEquippedStack(current);
        const auto unequip = validatedUnequipObject();
        if (!player || !manager || !unequip || !stack.found || !clearPreviousWeaponRestore(player)) return false;
        const auto accepted = unequip(manager, player, &current.item.item, 1, current.item.equipSlot,
            stack.stackID, false, true, false, true, nullptr);
        const bool empty = native_equipped_weapon::slotEmpty(nativeIndex);
        ROCK_LOG_INFO(Weapon, "Native equipped release index={} form={:08X} stack={} accepted={} empty={}",
            nativeIndex, formID, stack.stackID, accepted, empty);
        return empty;
    }

    EquippedDropResult dropEquippedWeaponFromPlayer(const EquippedDropInput& input) noexcept
    {
        EquippedDropResult result{};

        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) {
            result.reason = DropReason::MissingPlayer;
            return result;
        }

        const auto equipped = readEquippedWeaponSnapshot(input.nativeIndex);
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

        native_equipped_weapon::Snapshot indexed;
        if (input.nativeIndex != UINT32_MAX && !native_equipped_weapon::read(input.nativeIndex, indexed)) {
            result.reason = DropReason::MissingEquippedWeapon;
            return result;
        }
        const auto stack = input.nativeIndex != UINT32_MAX ? findIndexedEquippedStack(indexed) :
            findEquippedWeaponStack(player, equipped.weapon, equipped.instanceData);
        if (!stack.found || stack.count == 0) {
            result.reason = DropReason::InventoryStackNotFound;
            return result;
        }

        // Validate the duplicate-release capability before removing anything.
        auto* equipManager = RE::ActorEquipManager::GetSingleton();
        const auto unequip = validatedUnequipObject();
        if (!equipManager || !unequip || !stack.equipSlot) {
            result.reason = DropReason::UnequipUnavailable;
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
        const auto equippedAfterDrop = readEquippedWeaponSnapshot(input.nativeIndex);
        bool duplicateUnequipAccepted = false;
        if (result.handle && equippedAfterDrop.weapon == equipped.weapon) {
            // RemoveItem (1403E1C80/1403E1DC7) skips unequip for a partial
            // stack, or an equivalent surviving stack. Drop the original
            // equipped stack first: unequipping first can merge/reorder its
            // extra data. Then resolve only the surviving equipped copy;
            // never reuse the pre-removal stack index for this second call.
            native_equipped_weapon::Snapshot remainingIndexed;
            if (input.nativeIndex != UINT32_MAX) (void)native_equipped_weapon::read(input.nativeIndex, remainingIndexed);
            const auto remainingStack = input.nativeIndex != UINT32_MAX ? findIndexedEquippedStack(remainingIndexed) :
                findEquippedWeaponStack(player, equippedAfterDrop.weapon, equippedAfterDrop.instanceData);
            // Native removal can move equipped flags to an equivalent stack
            // with different instance data. The selector's sole-equipped-stack
            // witness is sufficient for unequip; this copy is never dropped.
            if (remainingStack.found && remainingStack.count != 0 && remainingStack.equipSlot &&
                clearPreviousWeaponRestore(player)) {
                RE::BGSObjectInstance remainingObject(equippedAfterDrop.weapon, remainingStack.instanceData.get());
                duplicateUnequipAccepted = unequip(equipManager, player, &remainingObject, 1,
                    remainingStack.equipSlot, remainingStack.stackID, false, true, false, true, nullptr);
            }
        }
        const auto equippedAfterRelease = readEquippedWeaponSnapshot(input.nativeIndex);
        result.equippedSlotReleased = input.nativeIndex != UINT32_MAX ? native_equipped_weapon::slotEmpty(input.nativeIndex) :
            !equippedAfterRelease.weapon;
        ROCK_LOG_INFO(Weapon,
            "Equipped detach native removal: weapon={:08X} stack={} countBefore={} previousRestoreCleared=yes afterRemoval={:08X} duplicateUnequipAccepted={} remainingEquipped={:08X} handleValid={}",
            result.formID, result.stackID, stack.count,
            equippedAfterDrop.weapon ? equippedAfterDrop.weapon->GetFormID() : 0,
            duplicateUnequipAccepted,
            equippedAfterRelease.weapon ? equippedAfterRelease.weapon->GetFormID() : 0,
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

#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/EquipSlotAllocationPolicy.h"
#include "physics-interaction/weapon/InventoryWeaponEquipPolicy.h"
#include "physics-interaction/weapon/NativeEquippedModelLifetime.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/NativeSecondaryActions.h"

#include <array>

namespace rock
{
    namespace
    {
        namespace equip_api = api::weapon;
        using Failure = equip_api::EquipFailure;
        using State = equip_api::EquipState;
        using Flag = equip_api::EquipResultFlag;

        constexpr std::uint32_t bit(Flag flag) { return static_cast<std::uint32_t>(flag); }
    }

    api::Status PhysicsInteraction::captureProviderInventoryWeapon(std::uint32_t form, std::uint32_t stack,
        equip_api::InventoryWeapon& output) const
    {
        if (!isProviderReady() || !runtime_state::isLocalSkeletonReady())
            return api::Status::NotReady;
        auto* weapon = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(form);
        if (!weapon || loose_grenade_runtime::isThrowableWeapon(weapon))
            return api::Status::TargetInvalid;
        const auto selection = weapon_equip_transfer::captureInventoryWeapon(form, stack);
        if (!selection.stack)
            return api::Status::TargetUnavailable;
        output.baseFormId = form;
        output.stackIndex = stack;
        output.count = selection.stack->GetCount();
        output.stackKey = reinterpret_cast<std::uintptr_t>(selection.stack.get());
        output.instanceKey = reinterpret_cast<std::uintptr_t>(selection.instance.get());
        output.frameIndex = provider::runtime::currentGameFrameIndex();
        output.worldGeneration = _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        output.skeletonGeneration = _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire);
        output.providerGeneration = _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire);
        return api::Status::Ok;
    }

    api::Status PhysicsInteraction::setProviderWarmInventoryWeapons(api::OwnerToken owner,
        const equip_api::InventoryWeapon* items, std::uint32_t count)
    {
        static_assert(equip_api::kMaxWarmInventoryWeapons == native_idle_grip_preharvest::kConsumerWarmCapacity);
        if (count > equip_api::kMaxWarmInventoryWeapons || (count && !items))
            return api::Status::InvalidArgument;
        // Resolve every item before changing anything; the selections keep
        // the instances alive until the preharvest retains them.
        std::array<weapon_equip_transfer::InventorySelection, equip_api::kMaxWarmInventoryWeapons> selections{};
        std::array<native_idle_grip_preharvest::ConsumerWarmRequest, equip_api::kMaxWarmInventoryWeapons> requests{};
        for (std::uint32_t i = 0; i < count; ++i) {
            equip_api::InventoryWeapon current{};
            const auto status = captureProviderInventoryWeapon(items[i].baseFormId, items[i].stackIndex, current);
            if (status != api::Status::Ok)
                return status == api::Status::NotReady ? status : api::Status::TargetUnavailable;
            if (current.stackKey != items[i].stackKey || current.instanceKey != items[i].instanceKey)
                return api::Status::TargetUnavailable;
            selections[i] = weapon_equip_transfer::captureInventoryWeapon(current.baseFormId, current.stackIndex);
            if (!selections[i].stack)
                return api::Status::TargetUnavailable;
            requests[i] = { RE::TESForm::GetFormByID<RE::TESObjectWEAP>(current.baseFormId),
                selections[i].instance.get() };
        }
        return native_idle_grip_preharvest::setConsumerWarmSet(owner, std::span(requests.data(), count))
            ? api::Status::Ok
            : api::Status::CapacityFull;
    }

    api::Status PhysicsInteraction::requestProviderInventoryEquip(api::OwnerToken owner,
        const equip_api::EquipRequest& request, std::uint64_t& command, const bool presentedAtFiringAim)
    {
        command = 0;
        if (request.hand != api::Hand::Right && request.hand != api::Hand::Left)
            return api::Status::InvalidArgument;
        if (request.item.size != sizeof(request.item) || request.item.version != 1 || !request.item.stackKey ||
            !request.item.count)
            return api::Status::InvalidArgument;
        if (_inventoryEquip.active() || _equipped.transition.heldTransfer().active())
            return api::Status::Busy;
        equip_api::InventoryWeapon current{};
        const auto status = captureProviderInventoryWeapon(request.item.baseFormId, request.item.stackIndex, current);
        if (status != api::Status::Ok)
            return status;
        if (const auto captured = inventory_weapon_equip_policy::validateCapture(request.item, current);
            captured != api::Status::Ok)
            return captured;
        if (runtime_state::isPhysicsMenuBlocked() || runtime_state::isCompatibilityConfigBlocked())
            return api::Status::NotReady;
        auto selection = weapon_equip_transfer::captureInventoryWeapon(current.baseFormId, current.stackIndex);
        if (!selection.stack || reinterpret_cast<std::uintptr_t>(selection.stack.get()) != current.stackKey ||
            reinterpret_cast<std::uintptr_t>(selection.instance.get()) != current.instanceKey)
            return api::Status::TargetUnavailable;
        auto& pending = _inventoryEquip;
        pending.owner = owner;
        pending.request = request;
        pending.selection = std::move(selection);
        command = pending.nextCommandId++;
        if (!command)
            command = pending.nextCommandId++;
        // Firearms only: melee seats on the physical wrist, with no aim pose
        // for the consumer to show.
        auto* incoming = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(current.baseFormId);
        if (presentedAtFiringAim && incoming && incoming->weaponData.type == RE::WEAPON_TYPE::kGun) {
            _twoHandedGrip.armDrawPresentationBlend(current.baseFormId, request.hand == api::Hand::Left);
        }
        pending.result = { .commandId = command,
            .state = State::Queued,
            .hand = request.hand,
            .incomingFormId = current.baseFormId,
            .worldGeneration = current.worldGeneration,
            .skeletonGeneration = current.skeletonGeneration,
            .providerGeneration = current.providerGeneration };
        return api::Status::RequestQueued;
    }

    api::Status PhysicsInteraction::getProviderInventoryEquipResult(api::OwnerToken owner, std::uint64_t command,
        equip_api::EquipResult& output) const
    {
        if (!command)
            return api::Status::InvalidArgument;
        if (_inventoryEquip.active() && _inventoryEquip.owner == owner && _inventoryEquip.result.commandId == command) {
            output = _inventoryEquip.result;
            return api::Status::Ok;
        }
        for (const auto& entry : _inventoryEquip.completed) {
            if (entry.owner == owner && entry.result.commandId == command) {
                output = entry.result;
                return api::Status::Ok;
            }
        }
        return api::Status::RequestNotFound;
    }

    api::Status PhysicsInteraction::cancelProviderInventoryEquip(api::OwnerToken owner, std::uint64_t command)
    {
        if (!_inventoryEquip.active() || _inventoryEquip.owner != owner || _inventoryEquip.result.commandId != command)
            return api::Status::RequestNotFound;
        const auto& transfer = _equipped.transition.heldTransfer();
        if (_inventoryEquip.result.transferSequence && transfer.inventoryCommitted)
            return api::Status::AlreadyCommitted;
        _inventoryEquip.cancelRequested = true;
        return api::Status::Ok;
    }

    api::Status PhysicsInteraction::unequipProviderWeapon(std::uint32_t weaponSlot, std::uint32_t expectedFormId)
    {
        if (weaponSlot >= api::kMaxWeaponSlots || !expectedFormId)
            return api::Status::InvalidArgument;
        if (!isProviderReady() || !runtime_state::isLocalSkeletonReady() || runtime_state::isPhysicsMenuBlocked() ||
            runtime_state::isCompatibilityConfigBlocked())
            return api::Status::NotReady;
        // A switch owns both native indices until its terminal result.
        if (_inventoryEquip.active() || _equipped.transition.heldTransfer().active())
            return api::Status::Busy;
        const auto view = native_hand_equipment::equippedViews()[weaponSlot];
        if (!view.weapon)
            return api::Status::TargetUnavailable;
        if (view.weapon->GetFormID() != expectedFormId)
            return api::Status::TargetInvalid;
        /*
         * The same exact-index unequip as a second-weapon promotion: the
         * stack's own hand leaf, so the other index keeps its weapon. Slot 0
         * emptied this way is followed by the next frame's promotion of a
         * slot-1 weapon in its own hand, as after a Pip-Boy unequip.
         */
        const auto instance = reinterpret_cast<std::uintptr_t>(view.instance);
        // As the shoulder stash and the second weapon's drop: the second
        // graph retires with its model, so its actions stop first, and the
        // equipment change completes the survivor's visibility handoff.
        if (weaponSlot == 1)
            native_secondary_actions::suspend();
        const bool unequipped = weapon_equip_transfer::unequipEquippedIndex(weaponSlot, expectedFormId, instance);
        ROCK_LOG_INFO(Weapon, "Provider weapon unequip slot={} form={:08X} instance={:#x} unequipped={}", weaponSlot,
            expectedFormId, instance, unequipped);
        if (!unequipped)
            return api::Status::InternalError;
        native_equipped_model_lifetime::noteEquipmentChange();
        if (weaponSlot == 0) {
            _equipped.transition.pendingGrip() = {};
            clearEquippedWeaponFiringGripInputState();
            enforceNoBareFistState(true);
        } else {
            sampleEquippedSlotOne();
        }
        return api::Status::Ok;
    }

    void PhysicsInteraction::clearProviderInventoryEquip()
    {
        if (_inventoryEquip.active())
            _inventoryEquip.finish(State::Cancelled, Failure::LifecycleLost);
    }

    void PhysicsInteraction::processProviderInventoryEquip(const PhysicsFrameContext& frame)
    {
        auto& pending = _inventoryEquip;
        if (!pending.active())
            return;
        auto& result = pending.result;
        auto& coordinator = _equipped.transition;
        const auto* observedTransfer = result.transferSequence
            ? coordinator.inventoryTransferResult(result.transferSequence)
            : &coordinator.heldTransfer();
        const auto finish = [&](State state, Failure failure) {
            ROCK_LOG_INFO(Weapon,
                "Inventory equip command={} hand={} incoming={:08X} outgoing={:08X} state={} failure={} flags={:X}",
                result.commandId, result.hand == api::Hand::Left ? "left" : "right", result.incomingFormId,
                result.outgoingFormId, static_cast<unsigned>(state), static_cast<unsigned>(failure), result.flags);
            pending.finish(state, failure);
        };
        if (result.worldGeneration != _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire) ||
            result.skeletonGeneration != _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire) ||
            result.providerGeneration != _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire)) {
            finish(State::Cancelled, Failure::LifecycleLost);
            return;
        }
        const bool started = result.transferSequence != 0;
        if (!observedTransfer) {
            finish(State::Failed, Failure::EquippedWeaponChanged);
            return;
        }
        const auto& transfer = *observedTransfer;
        if (started) {
            if (transfer.gripAcquired)
                result.flags |= bit(Flag::IncomingGripAcquired);
            if (transfer.presentationAcquired)
                result.flags |= bit(Flag::IncomingPresented);
            if (transfer.phase == held_weapon_transfer::Phase::Terminal) {
                const bool succeeded = transfer.outcome == held_weapon_transfer::Outcome::Completed;
                const bool cancelled = transfer.outcome == held_weapon_transfer::Outcome::Cancelled;
                const auto failure =
                    result.failure != Failure::None ? result.failure : Failure::GripOrPresentationFailed;
                finish(succeeded    ? State::Succeeded
                        : cancelled ? State::Cancelled
                                    : State::Failed,
                    succeeded ? Failure::None : failure);
                return;
            }
        }
        const bool committed = started && transfer.inventoryCommitted;
        const bool ownerLost = provider::runtime::authorize(pending.owner, api::InterfaceId::Weapon, 2, true,
                                   provider::OwnerAccess::Active) != api::Status::Ok;
        if (!committed && (pending.cancelRequested || ownerLost || frame.menuBlocked)) {
            const auto failure = ownerLost ? Failure::OwnerLost
                : frame.menuBlocked        ? Failure::MenuBlocked
                                           : Failure::CancelledBeforeCommit;
            if (started)
                coordinator.cancelHeldRequest("inventory-equip-cancelled", held_weapon_transfer::Outcome::Cancelled);
            finish(State::Cancelled, failure);
            return;
        }
        if (!frame.worldReady || !frame.hknpWorld || frame.menuBlocked || frame.reloadBoundaryActive ||
            !runtime_state::currentFrame().visualAuthorityAvailable || runtime_state::isCompatibilityConfigBlocked()) {
            if (!started)
                finish(State::Failed, Failure::InfrastructureUnavailable);
            return;
        }
        if (started && transfer.phase != held_weapon_transfer::Phase::AwaitEquip)
            return;

        const bool left = pending.request.hand == api::Hand::Left;
        const auto& hand = left ? _leftHand : _rightHand;
        const auto& handFrame = left ? frame.left : frame.right;
        const auto reject = [&](Failure failure, const char* reason) {
            if (result.transferSequence) {
                result.failure = failure;
                coordinator.cancelHeldRequest(reason);
            } else
                finish(State::Failed, failure);
        };
        // The destination hand's own weapon is replaced below, so it does not
        // make the hand busy; anything else the hand holds or does does.
        constexpr auto ownWeapon = static_cast<std::uint32_t>(force_grab_policy::HandBlocker::EquippedWeapon) |
            static_cast<std::uint32_t>(force_grab_policy::HandBlocker::OffhandWeapon);
        const auto blockers = forceGrabHandBlockerMask(hand, left, handFrame.disabled, true) & ~ownWeapon;
        // The hand's own auto drop (its released weapon, left in the world)
        // never takes the hand; it holds the hand's commit slot only until the
        // native placement solve, bounded by its commit limit. It finishes
        // first and the command stays queued, as for another weapon transfer:
        // a holster draw on the let-go that dropped a support-carried weapon
        // failed HandBusy 10 ms before that drop completed (runtime 2 Nov
        // 12:44:25).
        const auto& handCommit = _forceGrab.pendingCommits[left ? 1u : 0u];
        if (blockers == static_cast<std::uint32_t>(force_grab_policy::HandBlocker::PendingForceGrab) &&
            handCommit.active && handCommit.equippedWeaponDropMode == equipped_weapon_drop_policy::Mode::AutoDrop) {
            ROCK_LOG_SAMPLE_INFO(Weapon, 500, "Inventory equip command={} waiting for the {} hand's auto drop",
                result.commandId, left ? "left" : "right");
            return;
        }
        if (blockers != 0) {
            ROCK_LOG_INFO(Weapon, "Inventory equip command={} refused: {} hand busy blockers={:X}", result.commandId,
                left ? "left" : "right", blockers);
            reject(handFrame.disabled ? Failure::HandUnavailable : Failure::HandBusy, "inventory-destination-busy");
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto handling = resolveEquippedWeaponDetachDecision(_equipped.handlingSettings);
        if (!handling.firingGripOwnershipEnabled || !frik_visual_authority::canBlockPrimaryHandWeaponPose() ||
            !TwoHandedGrip::canBeginPrimaryOnlyGripForHand(left) || !weapon_equip_transfer::canRecoverHeldEquip() ||
            !held_weapon_instant_transition::readinessFor(player).ready) {
            reject(Failure::InfrastructureUnavailable, "inventory-equip-infrastructure-unavailable");
            return;
        }
        if (!started) {
            // Another weapon transfer into a hand finishes first (it is short and
            // bounded by its own recovery); the command stays queued. An equip
            // transition that is only presenting the current weapon (menu exit,
            // observed equip) is superseded: beginHeldTransition replaces it,
            // as a native equip during it would.
            if (transfer.active()) {
                ROCK_LOG_SAMPLE_INFO(Weapon, 500, "Inventory equip command={} waiting for weapon transfer seq={}",
                    result.commandId, transfer.sequence);
                return;
            }
            // The weapon lands in the slot the shared equip rule picks: the
            // slot whose weapon the destination hand fires or carries is
            // replaced, and a weapon the other hand holds stays equipped.
            // Captured stacks are never equipped, so the rule's same-item
            // cases (cross-hand move, support toggle) cannot arise here.
            const auto roster = equippedWeaponRoster();
            const auto& slotZero = roster.facts().slot0;
            const auto& slotOne = roster.facts().slot1;
            using RosterHand = equipped_weapon_roster::Hand;
            const auto acting = equipped_weapon_roster::handFor(left);
            const auto carrier = !slotZero.firingGripDetached ? RosterHand::None
                : slotZero.partGrips[0].active                ? RosterHand::Right
                : slotZero.partGrips[1].active                ? RosterHand::Left
                                                              : RosterHand::None;
            const auto decision = equip_slot_allocation::decide(acting,
                { {
                    { .worn = slotZero.holdsHand,
                        .firingHand = slotZero.firingGripDetached ? RosterHand::None : slotZero.firingHand,
                        .holdingHand = carrier },
                    { .worn = slotOne.equipped, .firingHand = slotOne.hand },
                } });
            if (decision.slot == 1) {
                // Slot 1 is ROCK's private graph; its presentation takes the
                // weapon into this hand once the native model exists. Native
                // conflict resolution returns a replaced slot-1 weapon to the
                // inventory, as a Pip-Boy equip into that hand does; a slot-1
                // weapon in the other hand is never evicted.
                if (slotOne.equipped && slotOne.hand != acting) {
                    finish(State::Failed, Failure::HandBusy);
                    return;
                }
                result.outgoingFormId = slotOne.equipped ? slotOne.formId : 0;
                if (!held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(player)))
                    return;
                const auto equipped = weapon_equip_transfer::equipInventoryWeapon(pending.selection, true, left);
                if (!equipped.success) {
                    finish(State::Failed, Failure::NativeEquipFailed);
                    return;
                }
                result.flags |= bit(Flag::IncomingEquipAccepted);
                result.weaponSlot = 1;
                armTransitionSeamTrace("inventory-equip-slot1");
                pending.selection = {};
                sampleEquippedSlotOne();
                finish(State::Succeeded, Failure::None);
                return;
            }
            const auto previousForm = currentEquippedWeaponFormId();
            const auto previousInstance =
                reinterpret_cast<std::uintptr_t>(currentEquippedWeaponInstanceData(currentEquippedWeaponForm()));
            pending.previousNativeNode = previousForm
                ? reinterpret_cast<std::uintptr_t>(equipped_weapon_visual_state::observe(previousForm).exactInstance)
                : 0;
            if (!coordinator.beginHeldRequest({ .world = result.worldGeneration,
                    .skeleton = result.skeletonGeneration,
                    .isLeft = left,
                    .previousForm = previousForm,
                    .previousInstance = previousInstance,
                    .inventorySource = true })) {
                finish(State::Failed, Failure::InfrastructureUnavailable);
                return;
            }
            result.state = State::Switching;
            result.outgoingFormId = previousForm;
            result.transferSequence = transfer.sequence;
        }
        if (!held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(player)))
            return;
        if (!weapon_equip_transfer::inventoryWeaponCurrent(pending.selection)) {
            reject(Failure::InventoryChanged, "inventory-selection-changed");
            return;
        }
        // equipInventoryWeapon resolves the retained exact stack again and
        // never substitutes another instance.
        const auto equipped = weapon_equip_transfer::equipInventoryWeapon(pending.selection);
        if (equipped.instantTransition.managerAccepted) {
            coordinator.recordInventoryCommit(equipped.weapon ? equipped.weapon->GetFormID() : 0,
                equipped.requestedInstanceData, equipped.success, equipped.observedEquippedInstanceData);
        }
        if (!equipped.success) {
            result.failure = Failure::NativeEquipFailed;
            coordinator.cancelHeldRequest("inventory-native-equip-failed");
            return;
        }
        result.flags |= bit(Flag::IncomingEquipAccepted);
        coordinator.pendingGrip() = {
            .pending = true,
            .isLeft = left,
            .targetWeaponFormID = equipped.weapon->GetFormID(),
            .targetWeaponInstanceData = equipped.observedEquippedInstanceData,
            .previousWeaponFormID = equipped.previousEquippedFormID,
            .previousWeaponInstanceData = equipped.previousEquippedInstanceData,
            .remainingSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
            .source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::InventoryEquip,
        };
        armTransitionSeamTrace("inventory-equip");
        coordinator.beginHeldTransition(
            {
                .formID = equipped.weapon->GetFormID(),
                .instanceData = equipped.observedEquippedInstanceData,
                .previousFormID = equipped.previousEquippedFormID,
                .previousInstanceData = equipped.previousEquippedInstanceData,
                .previousNativeInstanceNode = pending.previousNativeNode,
            },
            EquippedWeaponTransitionCoordinator::Source::InventoryEquip,
            {
                .weaponFormID = equipped.weapon->GetFormID(),
                .isLeftHand = left,
                .weapon = equipped.weapon,
                .timeoutSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
                .blendSeconds = _equipped.handlingSettings.equipVisualBridgeBlendSeconds,
            });
        pending.selection = {};
    }
}

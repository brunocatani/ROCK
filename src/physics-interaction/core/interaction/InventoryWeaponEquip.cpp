#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/weapon/InventoryWeaponEquipPolicy.h"

namespace rock {
    namespace {
        namespace equip_api = api::weapon::v1_1;
        using Failure = equip_api::EquipFailure;
        using State = equip_api::EquipState;
        using Flag = equip_api::EquipResultFlag;
        constexpr std::uint32_t bit(Flag flag) { return static_cast<std::uint32_t>(flag); }
    }

    api::Status PhysicsInteraction::captureProviderInventoryWeapon(std::uint32_t form, std::uint32_t stack,
        equip_api::InventoryWeapon& output) const
    {
        if (!isProviderReady() || !runtime_state::isLocalSkeletonReady()) return api::Status::NotReady;
        auto* weapon = RE::TESForm::GetFormByID<RE::TESObjectWEAP>(form);
        if (!weapon || loose_grenade_runtime::isThrowableWeapon(weapon)) return api::Status::TargetInvalid;
        const auto selection = weapon_equip_transfer::captureInventoryWeapon(form, stack);
        if (!selection.stack) return api::Status::TargetUnavailable;
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

    api::Status PhysicsInteraction::requestProviderInventoryEquip(api::OwnerToken owner,
        const equip_api::EquipRequest& request, std::uint64_t& command)
    {
        command = 0;
        if (request.hand != api::Hand::Right && request.hand != api::Hand::Left) return api::Status::InvalidArgument;
        if (request.item.size != sizeof(request.item) || request.item.reserved || !request.item.stackKey || !request.item.count)
            return api::Status::InvalidArgument;
        if (_inventoryEquip.active() || _equipped.transition.heldTransfer().active()) return api::Status::Busy;
        equip_api::InventoryWeapon current{};
        const auto status = captureProviderInventoryWeapon(request.item.baseFormId, request.item.stackIndex, current);
        if (status != api::Status::Ok) return status;
        if (const auto captured = inventory_weapon_equip_policy::validateCapture(request.item, current);
            captured != api::Status::Ok) return captured;
        if (runtime_state::isPhysicsMenuBlocked() || runtime_state::isCompatibilityConfigBlocked()) return api::Status::NotReady;
        auto selection = weapon_equip_transfer::captureInventoryWeapon(current.baseFormId, current.stackIndex);
        if (!selection.stack || reinterpret_cast<std::uintptr_t>(selection.stack.get()) != current.stackKey ||
            reinterpret_cast<std::uintptr_t>(selection.instance.get()) != current.instanceKey) return api::Status::TargetUnavailable;
        auto& pending = _inventoryEquip;
        pending.owner = owner;
        pending.request = request;
        pending.selection = std::move(selection);
        command = pending.nextCommandId++;
        if (!command) command = pending.nextCommandId++;
        pending.result = {.state = State::Queued, .commandId = command, .hand = request.hand,
            .incomingFormId = current.baseFormId, .worldGeneration = current.worldGeneration,
            .skeletonGeneration = current.skeletonGeneration, .providerGeneration = current.providerGeneration};
        return api::Status::RequestQueued;
    }

    api::Status PhysicsInteraction::getProviderInventoryEquipResult(api::OwnerToken owner, std::uint64_t command,
        equip_api::EquipResult& output) const
    {
        if (!command) return api::Status::InvalidArgument;
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
        if (_inventoryEquip.result.transferSequence && (transfer.outgoingRemoved || transfer.inventoryCommitted))
            return api::Status::AlreadyCommitted;
        _inventoryEquip.cancelRequested = true;
        return api::Status::Ok;
    }

    void PhysicsInteraction::clearProviderInventoryEquip()
    {
        if (_inventoryEquip.active()) _inventoryEquip.finish(State::Cancelled, Failure::LifecycleLost);
    }

    void PhysicsInteraction::processProviderInventoryEquip(const PhysicsFrameContext& frame)
    {
        auto& pending = _inventoryEquip;
        if (!pending.active()) return;
        auto& result = pending.result;
        auto& coordinator = _equipped.transition;
        const auto* observedTransfer = result.transferSequence ? coordinator.inventoryTransferResult(result.transferSequence) :
            &coordinator.heldTransfer();
        const auto finish = [&](State state, Failure failure) {
            ROCK_LOG_INFO(Weapon, "Inventory equip command={} hand={} incoming={:08X} outgoing={:08X} state={} failure={} flags={:X}",
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
            if (transfer.outgoingRemoved) result.flags |= bit(Flag::OutgoingRemoved);
            if (transfer.outgoingSucceeded) result.flags |= bit(Flag::OutgoingRetained);
            if (transfer.gripAcquired) result.flags |= bit(Flag::IncomingGripAcquired);
            if (transfer.presentationAcquired) result.flags |= bit(Flag::IncomingPresented);
            result.outgoingReferenceFormId = transfer.outgoingReference;
            if (transfer.phase == held_weapon_transfer::Phase::Terminal) {
                const bool succeeded = transfer.outcome == held_weapon_transfer::Outcome::Completed;
                const bool cancelled = transfer.outcome == held_weapon_transfer::Outcome::Cancelled;
                const auto failure = result.failure != Failure::None ? result.failure :
                    transfer.outgoingRemoved && !transfer.outgoingSucceeded ? Failure::OutgoingTransferFailed :
                    Failure::GripOrPresentationFailed;
                finish(succeeded ? State::Succeeded : cancelled ? State::Cancelled : State::Failed,
                    succeeded ? Failure::None : failure);
                return;
            }
        }
        const bool committed = started && (transfer.outgoingRemoved || transfer.inventoryCommitted);
        const bool ownerLost = provider::runtime::authorize(pending.owner, api::InterfaceId::Weapon, 2, true,
            provider::OwnerAccess::Active) != api::Status::Ok;
        if (!committed && (pending.cancelRequested || ownerLost || frame.menuBlocked)) {
            const auto failure = ownerLost ? Failure::OwnerLost : frame.menuBlocked ? Failure::MenuBlocked : Failure::CancelledBeforeCommit;
            if (started) coordinator.cancelHeldRequest("inventory-equip-cancelled", held_weapon_transfer::Outcome::Cancelled);
            finish(State::Cancelled, failure);
            return;
        }
        if (!frame.worldReady || !frame.hknpWorld || frame.menuBlocked || frame.reloadBoundaryActive ||
            !runtime_state::currentFrame().visualAuthorityAvailable || runtime_state::isCompatibilityConfigBlocked()) {
            if (!started) finish(State::Failed, Failure::InfrastructureUnavailable);
            return;
        }
        if (started && transfer.phase != held_weapon_transfer::Phase::AwaitEquip) return;

        const bool left = pending.request.hand == api::Hand::Left;
        const auto& hand = left ? _leftHand : _rightHand;
        const auto& handFrame = left ? frame.left : frame.right;
        const auto reject = [&](Failure failure, const char* reason) {
            if (result.transferSequence) {
                result.failure = failure;
                coordinator.cancelHeldRequest(reason);
            } else finish(State::Failed, failure);
        };
        if (forceGrabHandBlockerMask(hand, left, handFrame.disabled, true) != 0) {
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
            if (transfer.active() || coordinator.getPublicSnapshot().active) {
                finish(State::Failed, Failure::EquippedWeaponChanged);
                return;
            }
            const auto occupancy = _twoHandedGrip.getGripOccupancy();
            const auto destination = inventory_weapon_equip_policy::destination(left, false,
                occupancy.right.carriesWeapon(), occupancy.left.carriesWeapon(), currentEquippedWeaponOccupiesHand());
            // An occupied equipped weapon must have exactly the other carrier;
            // never let ordinary native replacement consume a held weapon.
            if (destination == inventory_weapon_equip_policy::Destination::Unavailable) {
                finish(State::Failed, Failure::HandBusy);
                return;
            }
            const auto previousForm = currentEquippedWeaponFormId();
            const auto previousInstance = reinterpret_cast<std::uintptr_t>(currentEquippedWeaponInstanceData(currentEquippedWeaponForm()));
            pending.previousNativeNode = previousForm ? reinterpret_cast<std::uintptr_t>(
                equipped_weapon_visual_state::observe(previousForm).exactInstance) : 0;
            if (!coordinator.beginHeldRequest({.world = result.worldGeneration, .skeleton = result.skeletonGeneration,
                    .isLeft = left, .retainOutgoing = destination == inventory_weapon_equip_policy::Destination::RetainOtherHand,
                    .previousForm = previousForm,
                    .previousInstance = previousInstance, .inventorySource = true})) {
                finish(State::Failed, Failure::InfrastructureUnavailable);
                return;
            }
            result.state = State::Switching;
            result.outgoingFormId = previousForm;
            result.transferSequence = transfer.sequence;
        }
        if (!held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(player))) return;
        if (!weapon_equip_transfer::inventoryWeaponCurrent(pending.selection)) {
            reject(Failure::InventoryChanged, "inventory-selection-changed");
            return;
        }
        if (transfer.request.retainOutgoing && !transfer.outgoingRemoved) {
            if (!retainEquippedWeaponForReplacement(frame, left)) {
                result.failure = Failure::OutgoingTransferFailed;
                return;
            }
            result.flags |= bit(Flag::OutgoingRemoved);
            result.outgoingReferenceFormId = transfer.outgoingReference;
            if (!held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(player))) return;
        }
        // The outgoing drop may change stack indices, including another copy of
        // the same base weapon. equipInventoryWeapon resolves the retained exact
        // stack again and never substitutes another instance.
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
            .pending = true, .isLeft = left,
            .targetWeaponFormID = equipped.weapon->GetFormID(),
            .targetWeaponInstanceData = equipped.observedEquippedInstanceData,
            .previousWeaponFormID = equipped.previousEquippedFormID,
            .previousWeaponInstanceData = equipped.previousEquippedInstanceData,
            .remainingSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
            .source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::InventoryEquip,
        };
        coordinator.beginHeldTransition({
            .formID = equipped.weapon->GetFormID(), .instanceData = equipped.observedEquippedInstanceData,
            .previousFormID = equipped.previousEquippedFormID, .previousInstanceData = equipped.previousEquippedInstanceData,
            .previousNativeInstanceNode = pending.previousNativeNode,
        }, EquippedWeaponTransitionCoordinator::Source::InventoryEquip, {
            .weaponFormID = equipped.weapon->GetFormID(), .isLeftHand = left, .weapon = equipped.weapon,
            .timeoutSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
            .blendSeconds = _equipped.handlingSettings.equipVisualBridgeBlendSeconds,
        });
        pending.selection = {};
    }
}

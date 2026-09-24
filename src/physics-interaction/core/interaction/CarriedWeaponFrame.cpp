#include "physics-interaction/core/PhysicsInteractionInternal.h"

namespace rock
{
    namespace
    {
        RE::NiAVObject* carriedMuzzle(RE::NiAVObject* root) noexcept
        {
            if (!root) return nullptr;
            std::array<RE::NiAVObject*, 512> pending{};
            std::size_t count = 1, remaining = pending.size();
            pending[0] = root;
            while (count && remaining--) {
                auto* object = pending[--count];
                if (object->name.c_str() && _stricmp(object->name.c_str(), "ProjectileNode") == 0) return object;
                auto* node = object->IsNode();
                if (!node) continue;
                if (node->children.size() > pending.size() - count) return nullptr;
                for (const auto& child : node->children) if (child) pending[count++] = child.get();
            }
            return nullptr;
        }
    }

    bool PhysicsInteraction::beforeNativeWeaponEquip(const RE::BGSObjectInstance& item, void* request)
    {
        if (!CarriedWeaponRuntime::isInteractionThread() ||
            !CarriedWeaponRuntime::ready() || !_lifecycle.initialized || !item.object ||
            item.object->formType != RE::ENUM_FORM_ID::kWEAP) return true;
        _nativeAkimboEquip = {};
        // Native replacement may remove an overlapping borrowed context. Keep
        // its weapon-owned count while the normal inventory transaction runs.
        if (!_carriedWeapon.suspend()) return false;
        if (_inventoryEquip.active() || held_weapon_instant_transition::transactionActive() ||
            runtime_state::isPhysicsMenuBlocked() || runtime_state::isCompatibilityConfigBlocked() ||
            !_twoHandedGrip.isManualOwnershipActive()) return true;
        auto* current = currentEquippedWeaponForm();
        auto* currentInstance = currentEquippedWeaponInstanceData(current);
        if (!current || (current == item.object && currentInstance == item.instanceData.get())) return true;
        const auto occupancy = _twoHandedGrip.getGripOccupancy();
        const bool right = occupancy.right.carriesWeapon(), left = occupancy.left.carriesWeapon();
        if (right == left) return true;
        const bool destinationLeft = right;
        const auto& destination = destinationLeft ? _leftHand : _rightHand;
        if (destination.isHolding() || _equipped.transition.heldTransfer().active()) return false;
        auto* bhk = getPlayerBhkWorld();
        auto* world = bhk ? getHknpWorld(bhk) : nullptr;
        if (!bhk || bhk != _lifecycle.cachedBhkWorld || !world || world != _lifecycle.cachedHknpWorld ||
            !physicsWritesAllowedForWorld(world)) return false;
        std::uint32_t stack{};
        // E6FEA0 constructs request+0 from the supplied stack ID; E0F920
        // consumes that same ID. No other request fields are changed here.
        if (!native_memory::tryReadField(request, 0, stack)) return false;
        auto selection = weapon_equip_transfer::captureInventoryWeapon(item.object->formID, stack, true);
        if (!selection.stack || selection.instance.get() != item.instanceData.get()) return false;
        auto frame = buildFrameContext(bhk, world);
        if (!frame.worldReady || frame.menuBlocked) return false;
        if (!_equipped.transition.beginHeldRequest({
                .world = _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
                .skeleton = _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire),
                .isLeft = destinationLeft, .retainOutgoing = true,
                .previousForm = current->formID,
                .previousInstance = reinterpret_cast<std::uintptr_t>(currentInstance), .inventorySource = true})) return false;
        _nativeAkimboEquip.incoming = std::move(selection);
        _nativeAkimboEquip.previousForm = current->formID;
        _nativeAkimboEquip.previousInstance = reinterpret_cast<std::uintptr_t>(currentInstance);
        _nativeAkimboEquip.previousNode = reinterpret_cast<std::uintptr_t>(
            equipped_weapon_visual_state::observe(current->formID).exactInstance);
        _nativeAkimboEquip.isLeft = destinationLeft;
        _nativeAkimboEquip.pending = true;
        if (!retainEquippedWeaponForReplacement(frame, destinationLeft)) return false;
        // Removing the outgoing item can renumber another stack of the same
        // base form. Resolve the retained exact stack before native apply.
        if (!weapon_equip_transfer::resolveInventoryWeaponStack(_nativeAkimboEquip.incoming, stack) ||
            !native_memory::guardedCopyToMemory(request, &stack, sizeof(stack))) return false;
        return true;
    }

    void PhysicsInteraction::afterNativeWeaponEquip(const RE::BGSObjectInstance& item, bool success)
    {
        if (!CarriedWeaponRuntime::isInteractionThread() || !_nativeAkimboEquip.pending) return;
        const auto pending = std::exchange(_nativeAkimboEquip, NativeAkimboEquip{});
        auto* equipped = currentEquippedWeaponForm();
        auto* instance = currentEquippedWeaponInstanceData(equipped);
        if (!success || equipped != item.object || instance != item.instanceData.get()) {
            _equipped.transition.cancelHeldRequest("akimbo-incoming-native-equip-failed");
            ROCK_LOG_WARN(Weapon, "Akimbo inventory transfer declined incoming={:08X}; outgoing remains in its physical transfer",
                item.object ? item.object->formID : 0);
            return;
        }
        _equipped.transition.recordInventoryCommit(equipped->formID, reinterpret_cast<std::uintptr_t>(instance),
            true, reinterpret_cast<std::uintptr_t>(instance));
        _equipped.transition.pendingGrip() = {
            .pending = true, .isLeft = pending.isLeft,
            .targetWeaponFormID = equipped->formID,
            .targetWeaponInstanceData = reinterpret_cast<std::uintptr_t>(instance),
            .previousWeaponFormID = pending.previousForm,
            .previousWeaponInstanceData = pending.previousInstance,
            .remainingSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
            .source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::InventoryEquip,
        };
        _equipped.transition.beginHeldTransition({
            .formID = equipped->formID, .instanceData = reinterpret_cast<std::uintptr_t>(instance),
            .previousFormID = pending.previousForm, .previousInstanceData = pending.previousInstance,
            .previousNativeInstanceNode = pending.previousNode,
        }, EquippedWeaponTransitionCoordinator::Source::InventoryEquip, {
            .weaponFormID = equipped->formID, .isLeftHand = pending.isLeft, .weapon = equipped,
            .timeoutSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
            .blendSeconds = _equipped.handlingSettings.equipVisualBridgeBlendSeconds,
        });
        ROCK_LOG_INFO(Weapon, "Akimbo inventory transfer committed incoming={:08X} hand={} retained={:08X}",
            equipped->formID, pending.isLeft ? "left" : "right", pending.previousForm);
    }

    void PhysicsInteraction::updateCarriedWeapon(const PhysicsFrameContext& frame, bool prepareOnly)
    {
        CarriedWeaponRuntime::noteInteractionThread();
        CarriedWeaponRuntime::Input input{};
        for (const bool left : {false, true}) {
            auto& hand = left ? _leftHand : _rightHand;
            const auto& tracking = left ? frame.left : frame.right;
            if (!hand.isHoldingLooseWeapon() || !hand.getHeldRef()) continue;
            if (!_carriedWeapon.owns(hand.getHeldRef()) && !_carriedWeapon.retains(hand.getHeldRef())) continue;
            const auto& peer = left ? _rightHand : _leftHand;
            if (peer.isHoldingLooseWeapon() && peer.getHeldRef() == hand.getHeldRef() &&
                !weapon_grip_transfer::requesterIsPrimary(hand.isHoldingFiringGrip(), peer.isHoldingFiringGrip(),
                    hand.heldGrabIdentity(), peer.heldGrabIdentity())) continue;
            auto* reference = hand.getHeldRef();
            input.reference = reference;
            input.hand = left ? akimbo::Hand::Left : akimbo::Hand::Right;
            input.grip = hand.isHoldingFiringGrip() ? akimbo::Grip::Firing : akimbo::Grip::Support;
            input.deltaSeconds = frame.deltaSeconds;
            input.inputAllowed = frame.worldReady && !frame.menuBlocked && !tracking.disabled &&
                !input_remap_runtime::isMenuInputActive() && !runtime_state::isCompatibilityConfigBlocked() &&
                !input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(left);
            // Resolve only inside this held item's current model. No global
            // Weapon/WeaponLeft lookup can substitute the other gun's muzzle.
            if (!prepareOnly) {
                input.muzzle = carriedMuzzle(reference->Get3D());
                const auto trigger = input_remap_runtime::peekRawButtonState(left, 33);
                input.triggerHeld = trigger.available && trigger.held;
                input.inputAllowed = input.inputAllowed && trigger.available;
                input.reloadPressed = input_remap_runtime::consumeCarriedWeaponReload(left,
                    _carriedWeapon.sessionId(), _carriedWeapon.bindingId());
            }
            break;
        }
        if (prepareOnly) _carriedWeapon.prepare(input);
        else _carriedWeapon.update(input);
        if (!prepareOnly) {
            for (const bool left : {false, true}) {
                const bool owner = _carriedWeapon.owns(input.reference) && input.grip == akimbo::Grip::Firing &&
                    input.hand == (left ? akimbo::Hand::Left : akimbo::Hand::Right);
                input_remap_runtime::setCarriedWeaponInputOwner(left,
                    owner ? _carriedWeapon.sessionId() : 0, owner ? _carriedWeapon.bindingId() : 0);
            }
        }
    }
}

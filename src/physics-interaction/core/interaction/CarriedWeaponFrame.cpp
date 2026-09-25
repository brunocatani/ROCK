#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/CarriedWeaponProjectile.h"
#include "physics-interaction/weapon/PhysicalWeaponPairPolicy.h"

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
        if (_carriedWeapon.sessions[0].hasSession() && _carriedWeapon.sessions[1].hasSession()) return false;
        _nativeAkimboEquip = {};
        // Cancel pending input during the native inventory transaction. The
        // retained weapon's private data is never part of native equipment.
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

    bool PhysicsInteraction::activatePhysicalWeapon(const PhysicsFrameContext& frame, bool isLeft)
    {
        if (!frame.worldReady || frame.menuBlocked) return false;
        auto& hand = isLeft ? _leftHand : _rightHand;
        auto& peer = isLeft ? _rightHand : _leftHand;
        auto* reference = hand.getHeldRef();
        if (!reference || !hand.isHoldingLooseWeapon() || !CarriedWeaponRuntime::ready()) return false;
        const bool nativePresent = currentEquippedWeaponFormId() != 0;
        const bool peerGun = peer.isHoldingLooseWeapon() && peer.getHeldRef() != reference;
        if (!_carriedWeapon.hasSession() && !nativePresent && !peerGun) return false;
        const auto inputFor = [](Hand& held) {
            CarriedWeaponRuntime::Input input{};
            input.reference = held.getHeldRef();
            input.hand = held.isLeft() ? akimbo::Hand::Left : akimbo::Hand::Right;
            input.grip = held.isHoldingFiringGrip() ? akimbo::Grip::Firing : akimbo::Grip::Support;
            return input;
        };
        if (auto* session = _carriedWeapon.find(reference); session && session->owns(reference)) {
            if (!session->active()) {
                (void)session->activate(inputFor(hand));
                input_remap_runtime::blockWeaponTriggerUntilRelease(isLeft);
                ROCK_LOG_INFO(Weapon, "Physical weapon reactivated session={} ref={:08X} hand={} magazine={}",
                    session->sessionId(), reference->formID, isLeft ? "left" : "right", session->data()->ammoCount);
            }
            return true;
        }
        if (nativePresent) {
            const auto occupancy = _twoHandedGrip.getGripOccupancy();
            if (!(isLeft ? occupancy.right.carriesWeapon() : occupancy.left.carriesWeapon()) ||
                (isLeft ? occupancy.left.carriesWeapon() : occupancy.right.carriesWeapon())) return false;
            if (!_carriedWeapon.captureTransfer()) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Physical dual activation declined: original firearm or magazine cannot enter an independent session");
                return true;
            }
        }
        if (!_carriedWeapon.activate(inputFor(hand))) {
            if (nativePresent) _carriedWeapon.cancelTransfer();
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Physical weapon activation declined ref={:08X}: firearm data, native mapping or capacity unavailable; other weapon retained",
                reference->formID);
            return true;
        }
        if (!nativePresent && peerGun && !_carriedWeapon.activate(inputFor(peer))) {
            if (auto* session = _carriedWeapon.find(reference)) session->clear(true);
            ROCK_LOG_WARN(Weapon, "Physical dual activation declined: second item unavailable; both held items retained");
            return true;
        }
        if (nativePresent) {
            _physicalWeaponEntry = {reference->GetHandle(), {}, hand.heldGrabIdentity(), isLeft};
        }
        input_remap_runtime::blockWeaponTriggerUntilRelease(isLeft);
        ROCK_LOG_INFO(Weapon, "Physical weapon activation requested ref={:08X} hand={} nativeConversion={}",
            reference->formID, isLeft ? "left" : "right", nativePresent);
        return true;
    }

    DynamicWeaponCollisionRuntime* PhysicsInteraction::weaponProxyForBody(std::uint32_t bodyId) noexcept
    {
        if (_dynamicWeaponCollision.isProxyBodyIdAtomic(bodyId)) return &_dynamicWeaponCollision;
        for (auto& session : _carriedWeapon.sessions)
            if (session.physics.dynamic.isProxyBodyIdAtomic(bodyId)) return &session.physics.dynamic;
        return nullptr;
    }

    void PhysicsInteraction::updateCarriedWeapon(const PhysicsFrameContext& frame, bool prepareOnly)
    {
        CarriedWeaponRuntime::noteInteractionThread();
        std::array<CarriedWeaponRuntime::Input, 2> inputs{};
        std::array<Hand*, 2> owners{}, supports{};
        for (unsigned slot = 0; slot < _carriedWeapon.sessions.size(); ++slot) {
            auto& session = _carriedWeapon.sessions[slot];
            auto& input = inputs[slot];
            for (auto* hand : {&_rightHand, &_leftHand}) {
                if (!hand->isHoldingLooseWeapon() || !hand->getHeldRef()) continue;
                if (!session.owns(hand->getHeldRef()) && !session.retains(hand->getHeldRef())) continue;
                auto* peer = hand->isLeft() ? &_rightHand : &_leftHand;
                if (peer->isHoldingLooseWeapon() && peer->getHeldRef() == hand->getHeldRef()) {
                    if (!weapon_grip_transfer::requesterIsPrimary(hand->isHoldingFiringGrip(), peer->isHoldingFiringGrip(),
                            hand->heldGrabIdentity(), peer->heldGrabIdentity())) continue;
                    supports[slot] = peer;
                }
                owners[slot] = hand;
                input.reference = hand->getHeldRef();
                input.hand = hand->isLeft() ? akimbo::Hand::Left : akimbo::Hand::Right;
                input.grip = hand->isHoldingFiringGrip() ? akimbo::Grip::Firing : akimbo::Grip::Support;
                input.deltaSeconds = frame.deltaSeconds;
                const auto& tracking = hand->isLeft() ? frame.left : frame.right;
                input.inputAllowed = frame.worldReady && !frame.menuBlocked && !tracking.disabled &&
                    !input_remap_runtime::isMenuInputActive() && !runtime_state::isCompatibilityConfigBlocked() &&
                    !input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(hand->isLeft());
                break;
            }
            session.prepare(input);
            if (session.hasSession()) _carriedWeapon.transferAdmitted(session);
        }
        if (prepareOnly) return;

        if (_physicalWeaponEntry.incoming && !_physicalWeaponEntry.rollback) {
            const auto* incoming = _physicalWeaponEntry.incomingLeft ? &_leftHand : &_rightHand;
            const bool incomingHeld = incoming->isHolding() && incoming->getHeldRef() &&
                incoming->getHeldRef()->GetHandle() == _physicalWeaponEntry.incoming &&
                incoming->heldGrabIdentity() == _physicalWeaponEntry.incomingGrab;
            if (!incomingHeld && !_physicalWeaponEntry.converted) {
                if (const auto candidate = _physicalWeaponEntry.incoming.get())
                    if (auto* session = _carriedWeapon.find(candidate.get())) session->clear(true);
                _carriedWeapon.cancelTransfer();
                _physicalWeaponEntry = {};
                ROCK_LOG_INFO(Weapon, "Physical dual entry canceled before native conversion; original equipped gun retained");
            }
        }

        bool allReady = true, anyHeld = false;
        for (unsigned slot = 0; slot < inputs.size(); ++slot) {
            auto& session = _carriedWeapon.sessions[slot];
            if (owners[slot] && session.owns(inputs[slot].reference)) {
                anyHeld = true;
                _dynamicHandCollision.claimWeaponHandCollision(frame.hknpWorld, owners[slot]->isLeft());
                if (supports[slot]) _dynamicHandCollision.claimWeaponHandCollision(frame.hknpWorld, supports[slot]->isLeft());
                const bool physicsReady = session.physics.update(frame, inputs[slot].reference, session.data(), *owners[slot], supports[slot]);
                allReady = physicsReady && session.presentationReady() && allReady;
                if (session.presentationFailed() && currentEquippedWeaponFormId() && !_physicalWeaponEntry.converted) {
                    session.clear(true);
                    _carriedWeapon.cancelTransfer();
                    _physicalWeaponEntry = {};
                    ROCK_LOG_WARN(Weapon, "Physical dual entry declined before conversion: mechanical presentation unavailable; native gun retained");
                }
            }
        }
        auto* native = currentEquippedWeaponForm();
        if (native && anyHeld && _carriedWeapon.hasSession() && allReady && !_physicalWeaponEntry.rollback &&
            held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(RE::PlayerCharacter::GetSingleton()))) {
            const auto occupancy = _twoHandedGrip.getGripOccupancy();
            const bool left = occupancy.left.carriesWeapon(), right = occupancy.right.carriesWeapon();
            if (left != right && !_forceGrab.pendingCommits[left ? 1u : 0u].active) {
                if (retainEquippedWeaponForReplacement(frame, !left)) {
                    _physicalWeaponEntry.converted = true;
                    _physicalWeaponEntry.outgoing = _forceGrab.pendingCommits[left ? 1u : 0u].targetHandle;
                    ROCK_LOG_INFO(Weapon, "Physical dual representation committed native={:08X} sourceHand={}; incoming stays in its existing hold",
                        native->formID, left ? "left" : "right");
                }
            }
            allReady = false;
        }
        const bool nativeStillPresent = currentEquippedWeaponFormId() != 0;
        const bool transferPending = _forceGrab.pendingCommits[0].isEquippedWeaponTransfer() ||
            _forceGrab.pendingCommits[1].isEquippedWeaponTransfer();
        allReady = allReady && !nativeStillPresent && !transferPending;

        if (_physicalWeaponEntry.incoming && !_physicalWeaponEntry.rollback) {
            const auto* incoming = _physicalWeaponEntry.incomingLeft ? &_leftHand : &_rightHand;
            const auto isHeld = [&](RE::ObjectRefHandle reference) {
                return reference && ((_leftHand.getHeldRef() && _leftHand.getHeldRef()->GetHandle() == reference && _leftHand.isHolding()) ||
                    (_rightHand.getHeldRef() && _rightHand.getHeldRef()->GetHandle() == reference && _rightHand.isHolding()));
            };
            using namespace physical_weapon_pair_policy;
            bool failed = false;
            for (auto& session : _carriedWeapon.sessions) {
                failed = failed || session.presentationFailed();
                if (const auto retained = _physicalWeaponEntry.outgoing.get()) failed = failed || session.admissionFailed(retained.get());
            }
            const auto action = advance({
                .converted = _physicalWeaponEntry.converted,
                .incomingHeld = isHeld(_physicalWeaponEntry.incoming) && incoming->heldGrabIdentity() == _physicalWeaponEntry.incomingGrab,
                .outgoingHeld = isHeld(_physicalWeaponEntry.outgoing),
                .nativeOriginalPresent = nativeStillPresent,
                .placementPending = transferPending,
                .allReady = allReady,
                .failed = failed,
            });
            if (action == EntryAction::RestoreOriginal) {
                _physicalWeaponEntry.rollback = true;
                if (const auto candidate = _physicalWeaponEntry.incoming.get())
                    if (auto* session = _carriedWeapon.find(candidate.get())) session->clear(true);
                allReady = false;
                ROCK_LOG_INFO(Weapon, "Physical dual entry interrupted; restoring original gun from its exact retained reference");
            } else if (action == EntryAction::RestoreIncoming) {
                _physicalWeaponEntry.outgoing = _physicalWeaponEntry.incoming;
                _physicalWeaponEntry.incoming = {};
                _physicalWeaponEntry.rollback = true;
                _physicalDualEstablished = true;
                allReady = false;
                ROCK_LOG_INFO(Weapon, "Physical dual entry lost outgoing hold; restoring the remaining held gun");
            } else if (action == EntryAction::Cancel || action == EntryAction::Complete) {
                if (action == EntryAction::Cancel && _physicalWeaponEntry.converted) _physicalDualEstablished = true;
                _physicalWeaponEntry = {};
            }
        }
        unsigned liveCount = 0;
        PhysicalWeaponSession* survivor = nullptr;
        for (auto& session : _carriedWeapon.sessions) if (session.hasSession()) { ++liveCount; survivor = &session; }
        if (liveCount == 2 && allReady) {
            if (!_physicalDualEstablished) ROCK_LOG_INFO(Weapon, "Physical dual ready refs=({:08X},{:08X}) magazines=({},{}) native=absent",
                _carriedWeapon.sessions[0].reference()->formID, _carriedWeapon.sessions[1].reference()->formID,
                _carriedWeapon.sessions[0].data()->ammoCount, _carriedWeapon.sessions[1].data()->ammoCount);
            _physicalDualEstablished = true;
            if (!_physicalWeaponEntry.incoming && !_physicalWeaponEntry.rollback) _physicalWeaponEntry = {};
        }
        const bool twoDistinctHeldWeapons = _leftHand.isHoldingLooseWeapon() && _rightHand.isHoldingLooseWeapon() &&
            _leftHand.getHeldRef() != _rightHand.getHeldRef();
        const auto* survivorOwner = survivor ? owners[survivor->slotNumber()] : nullptr;
        const bool survivorLeft = survivorOwner && survivorOwner->isLeft();
        const auto survivorTrigger = input_remap_runtime::peekRawButtonState(survivorLeft, 33);
        if (_physicalDualEstablished && liveCount == 1 && !_physicalWeaponEntry.incoming && !_physicalWeaponEntry.rollback &&
            !transferPending && !nativeStillPresent && !twoDistinctHeldWeapons && survivorOwner && survivor->active() && survivor->canReturnToNative() &&
            survivorTrigger.available && !survivorTrigger.held) {
            _physicalWeaponEntry.outgoing = survivor->reference()->GetHandle();
            _physicalWeaponEntry.rollback = true;
            ROCK_LOG_INFO(Weapon, "Physical dual exit: returning surviving gun to ordinary single-weapon handling ref={:08X}", survivor->reference()->formID);
        }
        allReady = allReady && !_physicalWeaponEntry.rollback;
        if (!liveCount && !transferPending) {
            _physicalDualEstablished = false;
            _physicalWeaponEntry = {};
        }
        for (unsigned slot = 0; slot < inputs.size(); ++slot) {
            auto& session = _carriedWeapon.sessions[slot];
            auto& input = inputs[slot];
            if (!owners[slot] || !session.owns(input.reference)) continue;
            input.inputAllowed = input.inputAllowed && session.active() && !nativeStillPresent && !transferPending && !_physicalWeaponEntry.rollback &&
                session.presentationReady() && (_physicalDualEstablished || allReady);
            input.muzzle = carriedMuzzle(input.reference->Get3D());
            const bool left = input.hand == akimbo::Hand::Left;
            const auto trigger = input_remap_runtime::peekRawButtonState(left, 33);
            input.triggerHeld = trigger.available && trigger.held;
            input.inputAllowed = input.inputAllowed && trigger.available;
            input.reloadPressed = input_remap_runtime::consumeCarriedWeaponReload(left, session.sessionId(), session.bindingId());
            const auto bank = session.physics.collision.getWeaponBodySnapshotAtomic();
            std::array<std::uint32_t, MAX_WEAPON_COLLISION_BODIES + 1> bodies{};
            const auto count = (std::min)(static_cast<std::size_t>(bank.count), bank.bodyIds.size());
            std::copy_n(bank.bodyIds.begin(), count, bodies.begin());
            bodies[count] = session.physics.dynamic.proxyBodyIdForDebug().value;
            carried_weapon_projectile::publishBodies(slot, std::span(bodies).first(count + 1));
            session.update(input);
            session.physics.finishParts(frame);
        }
        for (auto* hand : {&_rightHand, &_leftHand}) {
            auto* session = _carriedWeapon.find(hand->getHeldRef());
            // Support holds also reserve the native trigger: a support grip
            // must not fire fists or a different actor weapon.
            const auto id = session && session->hasSession() ? session->sessionId() :
                (_carriedWeapon.hasSession() || _physicalWeaponEntry.incoming || transferPending) &&
                    (nativeStillPresent || transferPending || _physicalWeaponEntry.rollback) ? UINT64_MAX : 0;
            input_remap_runtime::setCarriedWeaponInputOwner(hand->isLeft(), id,
                session && session->hasSession() ? session->bindingId() : 0);
        }
    }
}

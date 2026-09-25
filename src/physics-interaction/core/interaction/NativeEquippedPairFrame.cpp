#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/NativeEquippedModelSlot.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"

namespace rock
{
    DynamicWeaponCollisionRuntime* PhysicsInteraction::weaponProxyForBody(std::uint32_t bodyId) noexcept
    {
        if (_dynamicWeaponCollision.isProxyBodyIdAtomic(bodyId)) return &_dynamicWeaponCollision;
        if (_secondaryEquipped.dynamic.isProxyBodyIdAtomic(bodyId)) return &_secondaryEquipped.dynamic;
        return nullptr;
    }

    bool PhysicsInteraction::promoteSecondaryEquippedWeapon(const PhysicsFrameContext& frame)
    {
        if (!frame.worldReady || frame.menuBlocked || !native_equipped_weapon::slotEmpty(0) ||
            !_secondaryEquipped.ready() || !_nativeEquippedActions[1].quiescent()) return false;
        const auto carry = _secondaryEquipped.captureContinuity();
        const auto& source = _secondaryEquipped.snapshot();
        if (!carry.valid || input_remap_runtime::peekRawButtonState(carry.isLeft,33).held ||
            _secondaryPromotionAttempted == source.identity) return false;
        const auto selection = weapon_equip_transfer::captureEquippedInventoryWeapon(1);
        if (!selection.stack) return false;
        const auto* data = static_cast<RE::EquippedWeaponData*>(source.item.data.get());
        if (!data->ammo) return false;
        const auto ammo = data->ammo->formID, loaded = data->ammoCount;
        _secondaryPromotionAttempted = source.identity;
        // One manager transaction moves the exact occupied stack back to its
        // ordinary authored slot. Never unequip first and let that stack merge.
        const auto result = weapon_equip_transfer::equipInventoryWeapon(selection,UINT32_MAX);
        native_equipped_weapon::Snapshot promoted;
        const auto observedStack = weapon_equip_transfer::captureEquippedInventoryWeapon(0);
        if (!result.success || !native_equipped_weapon::read(0,promoted) || !native_equipped_weapon::slotEmpty(1) ||
            promoted.identity.form != carry.form || promoted.identity.instance != carry.instance ||
            observedStack.stack != selection.stack || !native_equipped_weapon::restoreMagazine(promoted.identity,ammo,loaded)) {
            ROCK_LOG_ERROR(Weapon,"Secondary equipped promotion incomplete form={:08X} reason={}; no repeated equip retry",carry.form,
                weapon_equip_transfer::equipReasonName(result.reason));
            return false;
        }
        EquippedWeaponTransitionCoordinator::PendingGrip resume;
        resume.pending = true;
        resume.isLeft = carry.isLeft;
        resume.targetWeaponFormID = carry.form;
        resume.targetWeaponInstanceData = carry.instance;
        resume.pairedGrips = carry.paired;
        resume.supportGrip = carry.support;
        resume.secondSupportGrip = carry.secondSupport;
        resume.hasFiringHandWeaponLocal = !carry.support.validCarry();
        resume.hasFiringGripWeaponLocal = resume.hasFiringHandWeaponLocal;
        resume.firingHandWeaponLocal = carry.firing.handWeaponLocal;
        resume.firingGripWeaponLocal = carry.firing.gripWeaponLocal;
        resume.toggleAcquisitionCommitted = !carry.support.validCarry();
        resume.source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::HeldWeaponEquip;
        for (auto& actions : _nativeEquippedActions) actions.clear(true);
        _secondaryEquipped.clear(true);
        _twoHandedGrip.reset();
        restoreEquippedWeaponContinuity(resume);
        _nativeEquippedPairActive = false;
        (void)frik_visual_authority::clearWeaponNodeParentHand("ROCK_EquippedPair");
        input_remap_runtime::blockWeaponTriggerUntilRelease(carry.isLeft);
        (void)native_equipped_weapon::requestAttach(promoted.identity);
        ROCK_LOG_INFO(Weapon,"Secondary equipped weapon returned to primary slot form={:08X} hand={} loaded={} exactStack=retained",
            carry.form,carry.isLeft ? "left":"right",loaded);
        return true;
    }

    bool PhysicsInteraction::equipSecondNativeWeapon(const PhysicsFrameContext& frame, bool isLeft)
    {
        if (!frame.worldReady || frame.menuBlocked) return false;
        auto& hand = isLeft ? _leftHand : _rightHand;
        auto& peer = isLeft ? _rightHand : _leftHand;
        auto* ref = hand.isHoldingLooseWeapon() ? hand.getHeldRef() : nullptr;
        const auto occupied = _twoHandedGrip.getGripOccupancy();
        if (!ref || !(isLeft ? occupied.right : occupied.left).carriesWeapon() ||
            (isLeft ? occupied.left : occupied.right).carriesWeapon() || peer.isHolding()) return false;
        auto* weapon = ref->GetObjectReference() ? ref->GetObjectReference()->As<RE::TESObjectWEAP>() : nullptr;
        if (!weapon || weapon->weaponData.type != RE::WEAPON_TYPE::kGun) return false;
        // A second firearm request is consumed even when preparation fails.
        // It must never fall through into the old replacement/drop transaction.
        if (!native_equipped_actions::ready() || _nativeEquippedAdmission.pending() ||
            !native_equipped_weapon::slotEmpty(1)) return true;
        native_equipped_weapon::Snapshot original;
        const auto* incomingExtra = ref->extraList ? ref->extraList->GetByType<RE::ExtraInstanceData>() : nullptr;
        if (!native_equipped_weapon::read(0,original) ||
            !native_equipped_actions::supports(weapon,incomingExtra ? incomingExtra->data.get() : nullptr) ||
            !native_equipped_actions::supports(static_cast<RE::TESObjectWEAP*>(original.item.item.object),original.item.item.instanceData.get()) ||
            input_remap_runtime::peekRawButtonState(!isLeft,33).held ||
            f4vr::getNativeGunState(f4vr::getPlayer()) == static_cast<std::uint32_t>(RE::GUN_STATE::kReloading)) {
            ROCK_LOG_SAMPLE_WARN(Weapon,1000,"Native second equip waiting: exact weapon timing or original action unavailable");
            return true;
        }
        SecondaryEquippedWeapon::Transfer incoming;
        RE::NiTransform physical{}, presented{};
        if (!hand.captureWeaponGripTransfer(incoming.firing) || !ref->Get3D() ||
            !frik_hand_world_authority::tryGetRawHandWorld(isLeft, physical) ||
            !frik_hand_world_authority::tryGetPublishedHandWorld(isLeft, presented) ||
            !vanilla_weapon_grip_frame::resolveModelTranslation(weapon->formID, ref->Get3D(), incoming.sourceModelTranslation)) {
            ROCK_LOG_SAMPLE_WARN(Weapon,1000,"Native second equip deferred ref={:08X}: exact grip/model capture unavailable",ref->formID);
            return true;
        }
        incoming.firing.handWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(ref->Get3D()->world), presented);
        incoming.presentedHandInPhysical = transform_math::composeTransforms(transform_math::invertTransform(physical),presented);
        incoming.form = weapon->formID;
        incoming.isLeft = isLeft;
        incoming.valid = incoming.firing.valid();
        if (!hand.isHoldingFiringGrip()) {
            RE::NiTransform driver{};
            if (!hand.isHoldingAuthoredSupportGrip() || !frik_hand_world_authority::tryGetInputDriverWorld(isLeft,driver)) return true;
            incoming.support = {.grip=incoming.firing,
                .weaponInDriver=transform_math::composeTransforms(transform_math::invertTransform(driver),ref->Get3D()->world),
                .sourceModelTranslation=incoming.sourceModelTranslation,.weaponFormID=incoming.form,.isLeft=isLeft};
            incoming.valid = incoming.support.validCarry();
        }
        if (!incoming.valid) return true;
        captureEquippedWeaponContinuity();
        const auto originalGrip = equippedWeaponContinuity();
        const auto prepared = _nativeEquippedAdmission.begin(weapon);
        if (prepared != NativeEquippedAdmission::Result::Ready) {
            ROCK_LOG_WARN(Weapon,"Native second equip refused form={:08X} admission={}",weapon->formID,static_cast<unsigned>(prepared));
            restoreEquippedWeaponContinuity(originalGrip);
            return true;
        }
        const auto sourceForm = ref->formID;
        const auto sourceBody = hand.getSavedObjectState().bodyId.value;
        hand.captureHeldReleaseMotion(frame.hknpWorld, (isLeft ? frame.left : frame.right).rawHandWorld,frame.timing);
        auto release = makeGrabReleaseContext(hand,isLeft);
        release.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
        release.reason = "second-native-equipped-weapon";
        hand.stopSelectionHighlight();
        auto outcome = hand.releaseGrabbedObject(frame.hknpWorld,GrabReleaseCollisionRestoreMode::Immediate,release);
        if (!outcome.released) {
            (void)_nativeEquippedAdmission.recover();
            restoreEquippedWeaponContinuity(originalGrip);
            return true;
        }
        releaseObject(ref,isLeft ? PhysicsObjectClaimOwner::LeftHand : PhysicsObjectClaimOwner::RightHand);
        const auto result = weapon_equip_transfer::transferHeldWeaponToPlayerAndEquip({
            .heldRef = outcome.takeRetainedReference(),
            .transitionReason = held_weapon_instant_transition::RequestReason::SameHandTrigger,
            .nativeIndex = 1,
        });
        const auto admitted = _nativeEquippedAdmission.finish(result.success);
        restoreEquippedWeaponContinuity(originalGrip);
        if (admitted == NativeEquippedAdmission::Result::Ready) {
            incoming.instance = result.observedEquippedInstanceData;
            _secondaryEquipped.adopt(incoming);
            _secondaryEquipped.bridge(result.detachedWorldModel);
            _nativeEquippedPairActive = true;
            native_equipped_weapon::Snapshot second;
            if (native_equipped_weapon::read(1,second)) (void)native_equipped_weapon::requestAttach(second.identity);
            input_remap_runtime::blockWeaponTriggerUntilRelease(isLeft);
            ROCK_LOG_INFO(Weapon,"Native second equip committed first={:08X} second={:08X} firingHand={} originalMagazine={}",
                _nativeEquippedAdmission.original().identity.form,incoming.form,isLeft ? "left":"right",_nativeEquippedAdmission.originalLoaded());
            _nativeEquippedAdmission.abandonAfterGameLoad();
        } else {
            ROCK_LOG_ERROR(Weapon,"Native second equip incomplete pickup={} native={} recovery={} reason={}",
                result.transferredToInventory,result.success,static_cast<unsigned>(admitted),weapon_equip_transfer::equipReasonName(result.reason));
        }
        auto* untransferred = result.untransferredRef.get();
        if (!result.transferredToInventory) hand.applyReleaseVelocitySnapshot(frame.hknpWorld,outcome.velocity);
        dispatchPhysicsMessage(kPhysMsg_OnRelease,isLeft,untransferred,sourceForm,0);
        GrabEventData released{};
        released.type = GrabEventType::Released;
        released.sourceKind = GrabEventSourceKind::HeldObject;
        released.isLeft = isLeft;
        released.refr = untransferred;
        released.formID = sourceForm;
        released.primaryBodyId = sourceBody;
        dispatchGrabEvent(released);
        input_remap_runtime::setHandHeldWeapon(isLeft,false);
        clearGameplayCandidatesForHand(hand,isLeft);
        return true;
    }

    void PhysicsInteraction::updateNativeEquippedPair(const PhysicsFrameContext& frame, bool prepareOnly)
    {
        if (prepareOnly && frame.worldReady && !frame.menuBlocked && _nativeEquippedAdmission.pending()) {
            const auto recovered = _nativeEquippedAdmission.recover();
            if (recovered != NativeEquippedAdmission::Result::RecoveryRequired) _nativeEquippedAdmission.abandonAfterGameLoad();
            else ROCK_LOG_SAMPLE_WARN(Weapon,2000,"Native equipped admission still requires exact inventory recovery");
        }
        if (prepareOnly) (void)promoteSecondaryEquippedWeapon(frame);
        native_equipped_weapon::Snapshot first, second;
        const bool firstPresent = native_equipped_weapon::read(0,first);
        const bool secondPresent = native_equipped_weapon::read(1,second);
        if (!secondPresent) {
            if (_nativeEquippedPairActive) {
                for (auto& actions : _nativeEquippedActions) actions.clear(frame.worldReady);
                _secondaryEquipped.clear(frame.worldReady);
                _nativeEquippedPairActive = false;
                (void)frik_visual_authority::clearWeaponNodeParentHand("ROCK_EquippedPair");
            }
            for (const bool left : {false,true}) input_remap_runtime::setCarriedWeaponInputOwner(left,0,0);
            input_remap_runtime::setWeaponTransferPending(_nativeEquippedAdmission.pending() || _equipped.transition.heldTransfer().active());
            _weaponCollision.bindEquippedSource(UINT32_MAX);
            (void)native_equipped_model_slot::releaseIfUnused();
            return;
        }
        _nativeEquippedPairActive = true;
        _weaponCollision.bindEquippedSource(0);
        auto handling = _equipped.handlingSettings;
        const auto detach = resolveEquippedWeaponDetachDecision(handling);
        handling.firingGripOwnershipEnabled = true;
        handling.primaryDetachEnabled = detach.primaryDetachEnabled;
        handling.detachAuthority = detach.authority;
        handling.preserveWeaponPoseOnDetach = detach.preserveWeaponPoseOnDetach;
        handling.firingGripReattachRadiusGameUnits = detach.reattachRadiusGameUnits;
        handling.weaponGripHapticDurationSeconds = detach.gripHapticDurationSeconds;
        handling.firingGripAttachHapticIntensity = detach.gripAttachHapticIntensity;
        handling.firingGripDetachHapticIntensity = detach.gripDetachHapticIntensity;
        const auto originalOccupancy = _twoHandedGrip.getGripOccupancy();
        const bool originalLeft = originalOccupancy.left.carriesWeapon() && !originalOccupancy.right.carriesWeapon();
        (void)frik_visual_authority::setWeaponNodeParentHand("ROCK_EquippedPair",frik_visual_authority::handFromBool(originalLeft));
        const bool secondaryReady = _secondaryEquipped.prepare(frame,handling);
        std::array<const native_equipped_weapon::Snapshot*,2> native{&first,&second};
        std::array<TwoHandedGrip*,2> grips{&_twoHandedGrip,&_secondaryEquipped.grip};
        struct InputOwner { std::uint64_t session{}, binding{}; };
        std::array<InputOwner,2> inputOwners{};
        bool ready = secondaryReady;
        for (unsigned index=0;index<2;++index) {
            auto& actions = _nativeEquippedActions[index];
            if (!native[index]->equipped) { actions.clear(frame.worldReady); continue; }
            const auto content = index ? _secondaryEquipped.collision.getCurrentEquippedWeaponInstanceContentKey() :
                _weaponCollision.getCurrentEquippedWeaponInstanceContentKey();
            const bool presentation = actions.prepare(*native[index],content,prepareOnly ? 0.0f : frame.deltaSeconds);
            ready = presentation && ready;
            const auto occupancy = grips[index]->getGripOccupancy();
            const bool firing = grips[index]->isFiringGripOccupied();
            const auto hand = occupancy.left.carriesWeapon() ? akimbo::Hand::Left : occupancy.right.carriesWeapon() ? akimbo::Hand::Right : akimbo::Hand::None;
            actions.bind(hand,firing ? akimbo::Grip::Firing : hand != akimbo::Hand::None ? akimbo::Grip::Support : akimbo::Grip::None);
            if (hand != akimbo::Hand::None) inputOwners[hand==akimbo::Hand::Left ? 1u : 0u] = {actions.sessionId(),actions.bindingId()};
        }
        for (unsigned hand=0;hand<2;++hand) input_remap_runtime::setCarriedWeaponInputOwner(hand==1,inputOwners[hand].session,inputOwners[hand].binding);
        input_remap_runtime::setWeaponTransferPending(!ready || _nativeEquippedAdmission.pending() || _equipped.transition.heldTransfer().active());
        if (prepareOnly && secondaryReady && !frame.menuBlocked) {
            using namespace equipped_weapon_toggle_grab_policy;
            const auto raw = [](bool left) {
                const auto b = input_remap_runtime::peekRawButtonState(left,input_remap_policy::kGrabButtonId);
                return ButtonState{b.held,b.pressed,b.released};
            };
            const auto left = raw(true), right = raw(false);
            const auto ownership = _secondaryEquipped.collision.getCurrentEquippedWeaponOwnershipKey();
            auto decision = prepare(_secondaryEquipped.toggle,{.weaponGrabMode=_equipped.handlingSettings.weaponGrabMode,
                .inputAllowed=true,.weaponOwnershipKey=ownership,.occupancy=_secondaryEquipped.grip.getGrabInputOccupancy(),.left=left,.right=right});
            const bool firingLeft = _secondaryEquipped.grip.isFiringHandLeft();
            auto primary = firingLeft ? decision.left : decision.right;
            if (_secondaryEquipped.consumeTransferredRelease()) {
                (void)_secondaryEquipped.grip.commitPersistentEquippedCarryInputAcquisition(firingLeft);
                primary = {.released=true};
                (firingLeft ? decision.left : decision.right) = primary;
                _secondaryEquipped.toggle.hands[handIndex(firingLeft)] = equipped_weapon_toggle_grab_policy::HandState::ReleasePending;
            }
            EquippedWeaponGripFrameInput input{
                .leftGripHeld=decision.left.held,.rightGripHeld=decision.right.held,
                .leftHandHoldingObject=_leftHand.isHolding() || originalOccupancy.left.carriesWeapon(),
                .rightHandHoldingObject=_rightHand.isHolding() || originalOccupancy.right.carriesWeapon(),
                .leftHandAvailableForAcquisition=!frame.left.disabled && !_leftHand.isHolding() && !originalOccupancy.left.carriesWeapon(),
                .rightHandAvailableForAcquisition=!frame.right.disabled && !_rightHand.isHolding() && !originalOccupancy.right.carriesWeapon(),
                .leftReattachEligible=!originalOccupancy.left.carriesWeapon(),.rightReattachEligible=!originalOccupancy.right.carriesWeapon(),
                .primaryGripInput={primary.held,primary.pressed,primary.released},
                .leftPhysicalGripInput={left.held,left.pressed,left.released},.rightPhysicalGripInput={right.held,right.pressed,right.released},
                .hmdPositionWorld=frame.hmdPositionWorld,.weaponGrabMode=_equipped.handlingSettings.weaponGrabMode,.hasHmdFrame=frame.hasHmdFrame,
            };
            const auto update = _secondaryEquipped.update(frame,input,handling);
            (void)reconcile(_secondaryEquipped.toggle,_equipped.handlingSettings.weaponGrabMode,ownership,update.after,
                {.left=update.releaseRetained.left,.right=update.releaseRetained.right});
            const auto pulses = _secondaryEquipped.grip.consumeHapticEvents();
            const auto pulse = [&](bool left, float intensity) {
                (void)_feedbackHaptics.queue(left ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    handling.weaponGripHapticDurationSeconds,intensity);
            };
            if (pulses.firingGripAttached) pulse(pulses.firingGripAttachedHandIsLeft,handling.firingGripAttachHapticIntensity);
            if (pulses.firingGripDetached) pulse(pulses.firingGripDetachedHandIsLeft,handling.firingGripDetachHapticIntensity);
            if (pulses.leftPartGripCaptured) pulse(true,handling.supportGripHapticIntensity);
            if (pulses.rightPartGripCaptured) pulse(false,handling.supportGripHapticIntensity);
            for (const bool handLeft : {false,true}) {
                const auto& held = handLeft ? update.after.left : update.after.right;
                if (held.weaponEngaged()) {
                    (void)input_remap_runtime::consumeRawButtonState(handLeft,input_remap_policy::kGrabButtonId);
                    (handLeft ? _leftHand : _rightHand).cancelGrabVisualReturn("secondary-equipped-owner");
                }
            }
            const auto drop = _secondaryEquipped.grip.consumeEquippedWeaponDropRequest();
            if (drop.requested) {
                _nativeEquippedActions[1].suspend();
                const bool committed = dropEquippedWeaponToWorld(frame,drop,equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode),1);
                _secondaryEquipped.grip.completeEquippedWeaponDrop(drop,committed);
                if (committed) { _secondaryEquipped.clear(true); return; }
            }
        }
        if (prepareOnly) return;
        for (unsigned index=0;index<2;++index) {
            auto& actions = _nativeEquippedActions[index];
            const auto occupancy = grips[index]->getGripOccupancy();
            const bool firing = grips[index]->isFiringGripOccupied();
            const bool left = grips[index]->isFiringHandLeft();
            const auto hand = firing ? (left ? akimbo::Hand::Left : akimbo::Hand::Right) :
                occupancy.left.carriesWeapon() ? akimbo::Hand::Left : occupancy.right.carriesWeapon() ? akimbo::Hand::Right : akimbo::Hand::None;
            native_equipped_actions::Input input{
                .hand=hand,.grip=firing ? akimbo::Grip::Firing : hand != akimbo::Hand::None ? akimbo::Grip::Support : akimbo::Grip::None,
                .deltaSeconds=frame.deltaSeconds,
                .allowed=ready && !frame.menuBlocked && runtime_state::currentFrame().weaponDrawn &&
                    !(left ? frame.left : frame.right).disabled && !input_remap_runtime::isMenuInputActive() &&
                    !input_remap_runtime::isProviderOpenVrGameInputSuppressedForHand(left),
                .triggerHeld=input_remap_runtime::peekRawButtonState(left,33).held,
                .reloadPressed=input_remap_runtime::consumeCarriedWeaponReload(left,actions.sessionId(),actions.bindingId()),
            };
            const auto events = actions.update(*native[index],input);
            if (events.reloadStarted || events.reloadFinished)
                (void)_feedbackHaptics.queue(left ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                    events.reloadStarted ? 0.10f : 0.04f,events.reloadStarted ? 0.35f : 0.7f);
        }
        _secondaryEquipped.finishPresentation(frame);
    }
}

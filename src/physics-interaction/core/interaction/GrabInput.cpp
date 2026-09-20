#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/grab/GlobalSurfaceGrabPolicy.h"
#include "physics-interaction/consume/ImmersiveAid.h"
#include "physics-interaction/native/HeldScenePresentation.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include "physics-interaction/weapon/telemetry/VanillaWeaponAlignmentTelemetry.h"

// Grab input pipeline: hand preludes, touch grab, grab intent and commit, and per-frame grab input update.

namespace rock
{
    void PhysicsInteraction::prepareDynamicWorldCarCollisionForGrab(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* ref)
    {
        if (!ref) {
            return;
        }
        auto* baseObject = ref->GetObjectReference();
        const bool targetIsCar = fo4vr::isExplodableCar(baseObject);
        if (!targetIsCar) {
            return;
        }
        const auto decision = car_interaction_policy::evaluateGrab(car_interaction_policy::GrabPolicyInput{
            .targetIsCar = targetIsCar,
            .playerInPowerArmor = fo4vr::isInPowerArmor(),
        });
        if (decision.allowed) {
            _dynamicWorldCarCollision.restoreReference(bhkWorld, hknpWorld, ref, "grab-commit");
        }
    }

    GrabReleaseContext PhysicsInteraction::makeGrabReleaseContext(const Hand& hand, bool isLeft) const
    {
        const Hand& peer = isLeft ? _rightHand : _leftHand;
        auto* heldRef = hand.getHeldRef();
        const bool peerStillHoldingSameObject = heldRef && peer.isHolding() && peer.getHeldRef() == heldRef;
        return GrabReleaseContext{
            .finalObjectRelease = !peerStillHoldingSameObject,
            .peerHandStillHolding = peerStillHoldingSameObject,
            .reason = peerStillHoldingSameObject ? "peer-hand-still-holding-object" : "last-hand-release",
        };
    }

    GrabSharedObjectContext PhysicsInteraction::makeGrabSharedObjectContext(const Hand& hand, bool isLeft) const
    {
        const Hand& peer = isLeft ? _rightHand : _leftHand;
        auto* selectedRef = hand.hasSelection() ? hand.getSelection().refr : nullptr;
        if (!selectedRef || !peer.isHolding() || peer.getHeldRef() != selectedRef) {
            return {};
        }

        return GrabSharedObjectContext{
            .joiningPeerHeldObject = true,
            .peerSavedObjectState = &peer.getSavedObjectState(),
            .peerActiveGrabLifecycle = &peer.getActiveGrabLifecycle(),
            .peerHeldBodyIds = &peer.getHeldBodyIds(),
        };
    }

    void PhysicsInteraction::enforceNoBareFistState(bool forceRecheck)
    {
        if (_equipped.transition.heldTransfer().ownsEmptySlot(currentEquippedWeaponFormId())) return;
        if (input_remap_runtime::isBareFistDrawPermitted() && currentEquippedWeaponFormId() == 0) return;
        auto* player = RE::PlayerCharacter::GetSingleton();
        auto* legacyPlayer = f4vr::getPlayer();
        if (!player || !legacyPlayer) {
            _grabInput.bareFistGuardState = {};
            return;
        }

        const auto nativeState = f4vr::getNativeWeaponState(player);
        if (_grabInput.bareFistDrawOwned && _grabInput.bareFistHolsterRequested &&
            (nativeState == 4 || nativeState == 5)) {
            // The owned exit already reached WantToSheathe/Sheathing. Let it
            // finish instead of dispatching the same holster every frame.
            return;
        }

        const bool weaponDrawn = f4vr::IsWeaponDrawn();
        const std::uint32_t equippedWeaponFormId = currentEquippedWeaponFormId();
        const bool actorUsingMelee = weaponDrawn && f4vr::CombatUtilities_IsActorUsingMelee(legacyPlayer);
        if (!bare_fist_guard_policy::shouldRefreshWitness(
                _grabInput.bareFistGuardState,
                forceRecheck,
                weaponDrawn,
                equippedWeaponFormId,
                actorUsingMelee)) {
            return;
        }

        // Inventory stack scanning is transition-only; never run it every
        // frame while a legitimate melee weapon remains drawn.
        const bool realMeleeWeaponEquipped = actorUsingMelee && f4vr::isMeleeWeaponEquipped();

        _grabInput.bareFistGuardState = bare_fist_guard_policy::RecheckState{
            .initialized = true,
            .weaponDrawn = weaponDrawn,
            .equippedWeaponFormId = equippedWeaponFormId,
            .actorUsingMelee = actorUsingMelee,
            .realMeleeWeaponEquipped = realMeleeWeaponEquipped,
        };
        if (!weaponDrawn) {
            return;
        }

        if (!bare_fist_guard_policy::shouldHolster(bare_fist_guard_policy::Witness{
                .rockEnabled = true,
                .weaponDrawn = weaponDrawn,
                .actorUsingMelee = actorUsingMelee,
                .realMeleeWeaponEquipped = realMeleeWeaponEquipped,
            })) {
            return;
        }

        /*
         * FO4 represents an unarmed fallback as drawn melee even though no
         * inventory weapon owns the hand. ROCK has no manual unequip action,
         * so close that state centrally without filtering real hand-to-hand
         * weapons such as knuckles or power fists.
         */
        player->DrawWeaponMagicHands(false);
        _grabInput.bareFistGuardState.weaponDrawn = false;
        ROCK_LOG_SAMPLE_INFO(Weapon,
            g_rockConfig.rockLogSampleMilliseconds,
            "Bare-fist draw state blocked: holstering unarmed fallback (equippedForm={:08X})",
            equippedWeaponFormId);
    }

    void PhysicsInteraction::saveGrabOffsetForHand(Hand& hand, bool isLeft, RE::hknpWorld* hknpWorld)
    {
        if (!hknpWorld) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, no hknpWorld this frame ({} hand)", isLeft ? "left" : "right");
            return;
        }
        if (!hand.isHolding()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, {} hand is not holding anything", isLeft ? "left" : "right");
            return;
        }

        auto* heldRef = hand.getHeldRef();
        auto* rootNode = heldRef ? heldRef->Get3D() : nullptr;
        auto* baseForm = heldRef ? heldRef->GetObjectReference() : nullptr;
        if (!rootNode || !baseForm) {
            ROCK_LOG_WARN(Hand,
                "Saved grab offset: aborted, held ref missing 3D root or base form ({} hand, refr={:08X})",
                isLeft ? "left" : "right",
                heldRef ? heldRef->GetFormID() : 0);
            return;
        }

        const auto* weaponForm = baseForm->As<RE::TESObjectWEAP>();
        const bool throwableWeapon = weaponForm &&
            (weaponForm->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                weaponForm->weaponData.type == RE::WEAPON_TYPE::kMine);
        if (!saved_grab_offset::participatesInSavedGrabOffsets(weaponForm != nullptr, throwableWeapon)) {
            ROCK_LOG_INFO(Hand,
                "Saved grab offset: skipped for {} hand, '{}' ({:08X}) is a weapon and weapons seat through FRIK weapon offsets only",
                isLeft ? "left" : "right",
                heldRef->GetDisplayFullName() ? heldRef->GetDisplayFullName() : "",
                baseForm->GetFormID());
            return;
        }

        const auto formRef = saved_grab_offset::formRefFromRuntimeId(baseForm->GetFormID());
        if (formRef.empty()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: aborted, could not resolve load-order-independent identity for base form {:08X}",
                baseForm->GetFormID());
            return;
        }

        RE::NiTransform proxyWorld{};
        if (!hand.tryComputeGrabProxyLocalPalmPocketFrameWorld(hknpWorld, proxyWorld)) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: could not resolve live proxy frame for {} hand", isLeft ? "left" : "right");
            return;
        }

        const RE::NiTransform objectProxyLocal = grab_frame_math::objectInGeneratedProxyLocalSpace(proxyWorld, rootNode->world);

        saved_grab_offset::SavedGrabOffsetFile file{};
        std::string loadError;
        if (!saved_grab_offset::load(formRef, file, &loadError) && !loadError.empty()) {
            ROCK_LOG_WARN(Hand, "Saved grab offset: existing file for {:08X} unreadable ({}), overwriting", baseForm->GetFormID(), loadError);
        }
        file.object = formRef;
        file.objectName = heldRef->GetDisplayFullName() ? heldRef->GetDisplayFullName() : std::string{};
        file.formatVersion = saved_grab_offset::kFormatVersion;

        auto& handOffset = isLeft ? file.left : file.right;
        handOffset.present = true;
        handOffset.translateGame[0] = objectProxyLocal.translate.x;
        handOffset.translateGame[1] = objectProxyLocal.translate.y;
        handOffset.translateGame[2] = objectProxyLocal.translate.z;
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                handOffset.rotate[row * 3 + column] = objectProxyLocal.rotate.entry[row][column];
            }
        }

        /*
         * Finger pose is only captured from a live organic mesh-curl grab
         * (see Hand::tryGetLiveGrabFingerPoseSnapshot). If this hold never
         * ran the mesh solve - e.g. re-saving position only while the object
         * is already attached via a previously-saved offset (pull-catch/
         * force-grab) - there is no fresh finger data this frame, so any
         * finger pose already on disk for this hand is left untouched
         * instead of being cleared.
         */
        Hand::GrabFingerPoseSnapshot fingerSnapshot{};
        if (hand.tryGetLiveGrabFingerPoseSnapshot(fingerSnapshot)) {
            handOffset.hasFingerPose = true;
            handOffset.fingerValues[0] = fingerSnapshot.values[0];
            handOffset.fingerValues[1] = fingerSnapshot.values[1];
            handOffset.fingerValues[2] = fingerSnapshot.values[2];
            handOffset.fingerValues[3] = fingerSnapshot.values[3];
            handOffset.fingerValues[4] = fingerSnapshot.values[4];
            handOffset.hasFingerJointValues = fingerSnapshot.hasJointValues;
            if (fingerSnapshot.hasJointValues) {
                for (std::size_t i = 0; i < fingerSnapshot.jointValues.size(); ++i) {
                    handOffset.fingerJointValues[i] = fingerSnapshot.jointValues[i];
                }
            }
        }

        saved_grab_offset::save(file);

        /*
         * Ground-truth capture companion. The offset records WHERE the object
         * ends up; this records what it was posed against - the mesh ROCK
         * scored, the hand colliders it was posed relative to, the physics,
         * the contacts the pose actually makes, and the seat ROCK itself
         * committed before the user corrected it. That is what makes a saved
         * pose replayable and scorable offline instead of only reproducible
         * in-game. Physics and identity are filled here because this scope
         * owns the body-mass reader and the form identity.
         */
        saved_grab_capture::SavedGrabCaptureFile capture{};
        if (hand.tryBuildSavedGrabCapture(hknpWorld, proxyWorld, capture.capture)) {
            capture.object = formRef;
            capture.objectName = file.objectName;
            capture.hand = isLeft ? "left" : "right";
            capture.rockVersion = std::string(Version::NAME);

            const std::time_t capturedAt = std::time(nullptr);
            std::tm capturedUtc{};
            if (gmtime_s(&capturedUtc, &capturedAt) == 0) {
                char timeText[32]{};
                if (std::strftime(timeText, sizeof(timeText), "%Y-%m-%dT%H:%M:%SZ", &capturedUtc) > 0) {
                    capture.capturedUtc = timeText;
                }
            }

            const std::uint32_t heldBodyId = hand.getSavedObjectState().bodyId.value;
            capture.capture.physics.bodyId = heldBodyId;
            const float heldMass = readGrabEventBodyMass(hknpWorld, heldBodyId);
            if (std::isfinite(heldMass) && heldMass > 0.0f) {
                capture.capture.physics.mass = heldMass;
                capture.capture.physics.valid = true;
            }

            saved_grab_offset::saveCapture(capture);
            ROCK_LOG_INFO(Hand,
                "Saved grab capture for {:08X} ({} hand): triangles={} colliders={} mass={:.2f} com={} shape={} seatReasons=[align={} roll={} depth={} backstop={}]",
                baseForm->GetFormID(),
                isLeft ? "left" : "right",
                capture.capture.mesh.triangleCount,
                capture.capture.fingerSegments.size() + (capture.capture.palm.valid ? 1u : 0u),
                capture.capture.physics.mass,
                capture.capture.physics.hasCenterOfMass ? (capture.capture.physics.comTrusted ? "trusted" : "UNTRUSTED") : "none",
                capture.capture.seat.shapeClass,
                capture.capture.seat.alignmentReason,
                capture.capture.seat.rollReason,
                capture.capture.seat.depthReason,
                capture.capture.seat.penetrationBackstopReason);
        } else {
            ROCK_LOG_WARN(Hand,
                "Saved grab offset: ground-truth capture unavailable for {} hand ({:08X}); the offset itself was still saved",
                isLeft ? "left" : "right",
                baseForm->GetFormID());
        }

        ROCK_LOG_INFO(Hand,
            "Saved grab offset for {:08X} ({} hand, finger pose {})",
            baseForm->GetFormID(),
            isLeft ? "left" : "right",
            handOffset.hasFingerPose ? "captured" : "unchanged");

        const char* itemName = heldRef->GetDisplayFullName();
        f4vr::showNotification(std::string("Saved grab offset: ") + (itemName && *itemName ? itemName : "item"));
    }

    void PhysicsInteraction::updateSavedGrabOffsetGesture(const PhysicsFrameContext& frame)
    {
        /*
         * The actual press detection (developer mode, Activate/WandAccept
         * edge, which hand is engaged) lives in InputRemapRuntime, which
         * already tracks per-hand held-object state and native-event
         * dispatch; this just consumes the resulting per-hand request.
         */
        if (input_remap_runtime::consumePendingSavedGrabOffsetRequest(false)) {
            saveGrabOffsetForHand(_rightHand, false, frame.hknpWorld);
        }
        if (input_remap_runtime::consumePendingSavedGrabOffsetRequest(true)) {
            saveGrabOffsetForHand(_leftHand, true, frame.hknpWorld);
        }
    }

    struct PhysicsInteraction::GrabInputHandContext
    {
        RE::hknpWorld* hknp = nullptr;
        int grabButton = input_remap_policy::kGrabButtonId;
        bool ambidextrousHandoffAvailable = false;
        equipped_weapon_manual_ownership_policy::FiringGripModeAvailability firingGripModes{};
        bool gripZoneSettleEquipEnabled = false;
        FarSelectionHmdConeGate farHmdConeGate{};
        std::uint32_t worldGeneration = 0;
        std::uint32_t skeletonGeneration = 0;
        std::uint32_t providerGeneration = 0;
        std::uint32_t collisionGeneration = 0;
    };

    void PhysicsInteraction::clearShoulderStashForHand(Hand& hand, bool isLeft)
    {
        shoulder_stash::resetRuntime(_grabInput.shoulderStashStates[isLeft ? 1u : 0u]);
        hand.cancelStashCandidate();
    }

    void PhysicsInteraction::clearMouthConsumeForHand(Hand& hand, bool isLeft)
    {
        mouth_consume::resetRuntime(_grabInput.mouthConsumeStates[isLeft ? 1u : 0u]);
        hand.cancelConsumeCandidate();
    }

    void PhysicsInteraction::clearGameplayCandidatesForHand(Hand& hand, bool isLeft)
    {
        clearShoulderStashForHand(hand, isLeft);
        clearMouthConsumeForHand(hand, isLeft);
    }

    void PhysicsInteraction::publishHandInputOwnership(const Hand& hand, bool isLeft)
    {
        auto* heldRef = hand.isHolding() ? hand.getHeldRef() : nullptr;
        const bool pendingEquippedGripOwnership =
            _equipped.transition.pendingGrip().pending &&
            _equipped.transition.pendingGrip().isLeft == isLeft;
        input_remap_runtime::setHandHeldWeapon(isLeft, hand.isHoldingLooseWeapon());
        input_remap_runtime::setHandInteractionEngaged(
            isLeft,
            hand.isHolding() ||
                _touchGrabRuntime.isHandActive(isLeft) ||
                _twoHandedGrip.isHandPartGripping(isLeft) ||
                pendingEquippedGripOwnership);
        input_remap_runtime::setHeldObjectFormId(isLeft, heldRef ? heldRef->GetFormID() : 0u);
    }

    void PhysicsInteraction::releaseSuppressedHeldObject(
        RE::hknpWorld* world,
        Hand& hand,
        bool isLeft,
        const char* reason)
    {
        auto* heldRef = hand.getHeldRef();
        const auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
        auto releaseContext = makeGrabReleaseContext(hand, isLeft);
        releaseContext.reason = reason ? reason : "normal-grab-suppressed";
        hand.releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
        if (heldRef) {
            releaseObject(heldRef, claimOwnerForHand(isLeft));
        }
        ROCK_LOG_DEBUG(Hand,
            "{} hand: released held object because normal grab input is suppressed ({})",
            hand.handName(),
            reason ? reason : "unknown");
        dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
        dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
    }

    struct PhysicsInteraction::GrabInputHandPrelude
    {
        GrabButtonState grabInput{};
        GrabButtonState rawGrabInput{};
        bool heldWeaponAtFrameStart = false;
        bool heldWeaponEquipTriggerPressedEdge = false;
    };

    void PhysicsInteraction::cancelPeerHeldJoinRetry(
        Hand& hand,
        peer_held_join_retry_policy::RuntimeState& retryState,
        const char* reason,
        bool logCancellation)
    {
        if (!retryState.active) {
            return;
        }
        const auto peerFormId = retryState.peerFormId;
        const auto attempts = retryState.attempts;
        const char* lastRefusal = retryState.lastRefusalReason ? retryState.lastRefusalReason : "none";
        peer_held_join_retry_policy::reset(retryState);
        if (logCancellation) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand peer-held join retry cancelled: reason={} peerFormID={:08X} attempts={} lastRefusal={}",
                hand.handName(),
                reason ? reason : "unknown",
                peerFormId,
                attempts,
                lastRefusal);
        }
    }

    bool PhysicsInteraction::prepareGrabInputHand(
        const PhysicsFrameContext& frame,
        Hand& hand,
        bool isLeft,
        const GrabInputHandContext& context,
        GrabInputHandPrelude& outPrelude)
    {
        const int grabButton = context.grabButton;
        const auto collisionGeneration = context.collisionGeneration;

        const auto& handInput = isLeft ? frame.left : frame.right;
        auto& inputIntentState = _grabInput.intentStates[isLeft ? 1u : 0u];
        auto& peerHeldJoinRetryState = _grabInput.peerHeldJoinRetryStates[isLeft ? 1u : 0u];
        auto& inputSuppressionState = _grabInput.providerHandInputSuppressionStates[isLeft ? 1u : 0u];
        if (input_remap_runtime::ownsBareFistInput()) {
            static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, context.grabButton));
            static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, 33));
            grab_input_intent_policy::reset(inputIntentState);
            _grabInput.heldWeaponTriggerEquipIntents[isLeft ? 1u : 0u] = {};
            peer_held_join_retry_policy::reset(peerHeldJoinRetryState);
            inputSuppressionState.deferredGrabRelease = false;
            clearGameplayCandidatesForHand(hand, isLeft);
            if (hand.hasSelection()) hand.clearSelectionState(false);
            return false;
        }
        const bool heldWeaponAtFrameStart = hand.isHoldingLooseWeapon();
        const auto providerHand = isLeft ? provider::RockProviderHand::Left : provider::RockProviderHand::Right;
        const std::uint32_t providerInputSuppressionFlags = provider::currentHandInputSuppressionFlagsV1(providerHand);
        auto providerSuppresses = [&](provider::RockProviderHandInputSuppressionFlagV1 flag) {
            return provider::hasHandInputSuppressionFlagV1(providerInputSuppressionFlags, flag);
        };
        const bool providerSuppressesNormalGrabPress =
            providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressNormalGrabPress);
        const bool providerSuppressesGrabRelease =
            providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressGrabRelease);
        const bool providerSuppressesHeldWeaponTriggerEquip =
            providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressHeldWeaponTriggerEquip);
        const bool providerSuppressesGameplayCandidates =
            providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressGameplayCandidates);
        const bool providerSuppressesOpenVrGameInput =
            providerSuppresses(provider::RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput);
        input_remap_runtime::setProviderOpenVrGameInputSuppressed(isLeft, providerSuppressesOpenVrGameInput);
        auto cancelPeerHeldJoinRetry = [&](const char* reason, bool logCancellation) {
            if (!peerHeldJoinRetryState.active) {
                return;
            }
            const auto peerFormId = peerHeldJoinRetryState.peerFormId;
            const auto attempts = peerHeldJoinRetryState.attempts;
            const char* lastRefusal = peerHeldJoinRetryState.lastRefusalReason ? peerHeldJoinRetryState.lastRefusalReason : "none";
            peer_held_join_retry_policy::reset(peerHeldJoinRetryState);
            if (logCancellation) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand peer-held join retry cancelled: reason={} peerFormID={:08X} attempts={} lastRefusal={}",
                    hand.handName(),
                    reason ? reason : "unknown",
                    peerFormId,
                    attempts,
                    lastRefusal);
            }
        };

        const auto handIndex = isLeft ? 1u : 0u;
        auto& retainedWeapon = _forceGrab.retainedWeaponGrabs[handIndex];
        const auto& pendingTransfer = _forceGrab.pendingCommits[handIndex];
        const bool retainedWeaponInput = transferred_weapon_grab_policy::ownsInput(
            pendingTransfer.active && pendingTransfer.isEquippedWeaponTransfer(),
            retainedWeapon.grabIdentity, hand.heldGrabIdentity());
        if (retainedWeapon.grabIdentity != 0 && !retainedWeaponInput) {
            ROCK_LOG_INFO(Weapon,
                "Transferred weapon retention ended with its grab: hand={} state={} holding={} expectedGrab={} actualGrab={} ref={:08X}",
                isLeft ? "left" : "right", static_cast<unsigned>(retainedWeapon.inputState),
                hand.isHolding(), retainedWeapon.grabIdentity, hand.heldGrabIdentity(),
                hand.getHeldRef() ? hand.getHeldRef()->GetFormID() : 0);
            retainedWeapon = {};
        }
        const auto consumeHandGrabInput = [&](bool releaseAllowed = true) {
            GrabButtonState physical{};
            if (_grabInput.firingHandButtonFrame.valid && _grabInput.firingHandButtonFrame.isLeft == isLeft) {
                physical = {
                    .held = _grabInput.firingHandButtonFrame.held,
                    .pressed = _grabInput.firingHandButtonFrame.pressed,
                    .released = _grabInput.firingHandButtonFrame.released,
                };
                _grabInput.firingHandButtonFrame.valid = false;
            } else {
                physical = readGrabButtonState(isLeft, grabButton);
            }
            if (retainedWeaponInput) {
                const auto previous = retainedWeapon.inputState;
                (void)transferred_weapon_grab_policy::advance(retainedWeapon.inputState,
                    physical.held, physical.pressed, physical.released, releaseAllowed);
                if (previous != retainedWeapon.inputState) {
                    ROCK_LOG_INFO(Weapon,
                        "Transferred weapon grab input: hand={} state={}->{} held={} pressed={} released={} allowed={}",
                        isLeft ? "left" : "right", static_cast<unsigned>(previous),
                        static_cast<unsigned>(retainedWeapon.inputState), physical.held, physical.pressed, physical.released, releaseAllowed);
                }
            }
            return physical;
        };

        if (_equipped.shoulderGestureConsumedThisFrame[handIndex]) {
            // The equipped-weapon shoulder transaction owns this complete
            // physical button cycle. Never reuse any part of it for world,
            // surface, touch, or peer-held selection.
            static_cast<void>(consumeHandGrabInput(false));
            inputSuppressionState.deferredGrabRelease = false;
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry(
                "equipped-weapon-shoulder-gesture-this-frame",
                true);
            clearGameplayCandidatesForHand(hand, isLeft);
            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            return false;
        }
        if (_equipped.toggleGrabReleasePressConsumedThisFrame[handIndex] ||
            _equipped.holsterInputConsumedThisFrame[handIndex]) {
            // The second press belongs only to the equipped-weapon latch. Do
            // not let the same edge start a loose-object, surface, or touch
            // grab after the weapon state releases this hand. Virtual Holsters
            // likewise owns its claimed physical cycle through release.
            static_cast<void>(consumeHandGrabInput(false));
            inputSuppressionState.deferredGrabRelease = false;
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry(
                _equipped.holsterInputConsumedThisFrame[handIndex] ?
                    "virtual-holsters-gesture-this-frame" :
                    "equipped-weapon-toggle-release-this-frame",
                true);
            clearGameplayCandidatesForHand(hand, isLeft);
            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            return false;
        }
        if (_forceGrab.committedThisFrame[handIndex]) {
            /*
             * Consume, but do not apply, the physical button edges from
             * before this programmatic attachment. Otherwise a stale
             * release from the Pip-Boy/API frame can drop the object in
             * the same update that reported a successful force-grab.
             */
            static_cast<void>(consumeHandGrabInput());
            if (retainedWeaponInput) {
                static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, input_remap_policy::kOpenVrSteamVrTriggerButtonId));
                _grabInput.heldWeaponTriggerEquipIntents[handIndex] = {};
            }
            inputSuppressionState.deferredGrabRelease = false;
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("force-grab-committed-this-frame", true);
            clearGameplayCandidatesForHand(hand, isLeft);
            return false;
        }
        if (_forceGrab.pendingCommits[handIndex].active) {
            static_cast<void>(consumeHandGrabInput());
            if (_forceGrab.pendingCommits[handIndex].isEquippedWeaponTransfer()) {
                // An edge from the outgoing equipped representation cannot
                // become a fresh trigger-equip after its loose grab arrives.
                static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, input_remap_policy::kOpenVrSteamVrTriggerButtonId));
                _grabInput.heldWeaponTriggerEquipIntents[handIndex] = {};
            }
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("pending-force-grab-reservation", true);
            clearGameplayCandidatesForHand(hand, isLeft);
            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            return false;
        }

        const auto& peerCommit = _forceGrab.pendingCommits[isLeft ? 0u : 1u];
        if (peerCommit.active && hand.hasSelection()) {
            const auto peerTargetPtr = peerCommit.targetHandle.get();
            if (peerTargetPtr && hand.getSelection().refr == peerTargetPtr.get()) {
                hand.clearSelectionState(false);
            }
        }
        if (handInput.disabled) {
            grab_input_intent_policy::reset(inputIntentState);
            _touchGrabRuntime.releaseHand(
                isLeft,
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                collisionGeneration);
            cancelPeerHeldJoinRetry("hand-input-disabled", false);
            clearGameplayCandidatesForHand(hand, isLeft);
            return false;
        }
        if (providerSuppressesGameplayCandidates) {
            clearGameplayCandidatesForHand(hand, isLeft);
        }
        const bool providerHoldsCurrentGrabState =
            providerSuppressesGrabRelease &&
            (hand.isHolding() ||
                _touchGrabRuntime.isHandActive(isLeft) ||
                hand.getState() == HandState::SelectionLocked ||
                hand.getState() == HandState::Pulled);
        const bool providerBlocksNewGrabPress =
            providerSuppressesNormalGrabPress &&
            !hand.isHolding() &&
            !_touchGrabRuntime.isHandActive(isLeft) &&
            hand.getState() != HandState::SelectionLocked &&
            hand.getState() != HandState::Pulled;
        if (providerHoldsCurrentGrabState || providerBlocksNewGrabPress) {
            static_cast<void>(consumeHandGrabInput(false));
            if (providerSuppressesHeldWeaponTriggerEquip) {
                static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, 33));
            }
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("provider-hand-input-suppressed", true);
            if (providerHoldsCurrentGrabState &&
                !readGrabButtonHeld(isLeft, grabButton)) {
                inputSuppressionState.deferredGrabRelease = true;
            }
            return false;
        }

        const bool heldWeaponEquipTriggerPressedEdge =
            !providerSuppressesHeldWeaponTriggerEquip && readHeldWeaponEquipTriggerPressedEdge(isLeft);
        const bool firingHandIsLeft = equippedWeaponFiringHandForGrabIsLeft();
        const bool handIsFiringHand = isLeft == firingHandIsLeft;
        // The skeleton's Weapon node/drawn flag can outlive unequip. Use the
        // same item authority as force grab, sampled for each hand after any
        // earlier hand's equip/transfer instead of caching scene occupancy.
        const bool equippedWeaponPresent = currentEquippedWeaponOccupiesHand();
        const bool supportTransferPending = _equipped.transition.pendingGrip().supportGrip.validCarry();
        if ((_equipped.transition.pendingGrip().pairedGrips.valid() || _equipped.transition.pendingGrip().secondSupportGrip.validCarry()) ||
            (supportTransferPending ? isLeft == _equipped.transition.pendingGrip().isLeft :
            !weapon_two_handed_grip_math::canProcessNormalGrabInput(
                handIsFiringHand,
                equippedWeaponPresent,
                _twoHandedGrip.isHandPartGripping(isLeft),
                _twoHandedGrip.isPartCarryActive() && !_twoHandedGrip.isHandPartGripping(isLeft)))) {
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("normal-grab-suppressed", true);
            clearGameplayCandidatesForHand(hand, isLeft);
            if (_touchGrabRuntime.isHandActive(isLeft)) {
                _touchGrabRuntime.releaseHand(
                    isLeft,
                    frame.bhkWorld,
                    frame.hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        HandUnavailable,
                    collisionGeneration);
            }
            if (hand.isHolding()) {
                releaseSuppressedHeldObject(context.hknp, hand, isLeft, handIsFiringHand ? "firing-hand weapon equipped" : "equipped weapon support grip active");
            } else if (hand.hasActivePullCatchIntent()) {
                auto* pullCatchRef = hand.getPullCatchIntentRef();
                hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-normal-grab-suppressed");
                hand.clearSelectionState(true);
                releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull catch because normal grab input is suppressed", hand.handName());
            } else if (hand.hasPendingActorEquipmentDropHandoff()) {
                hand.clearSelectionState(true);
                ROCK_LOG_DEBUG(Hand, "{} hand: cleared actor-equipment drop handoff because normal grab input is suppressed", hand.handName());
            } else if (hand.getState() == HandState::Pulled || hand.getState() == HandState::SelectionLocked) {
                auto* selectedRef = hand.getSelection().refr;
                if (hand.getState() == HandState::SelectionLocked) {
                    dispatchSimpleGrabEvent(GrabEventType::SelectionUnlocked, isLeft, selectedRef, hand.getSelection().bodyId.value);
                } else {
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-normal-grab-suppressed");
                }
                hand.clearSelectionState(true);
                releaseObject(selectedRef, claimOwnerForHand(isLeft));
                ROCK_LOG_DEBUG(Hand, "{} hand: cleared pull/locked selection because normal grab input is suppressed", hand.handName());
            }
            return false;
        }

        if (hand.isHolding() && !handIsFiringHand &&
            firingHandIsLeft != _twoHandedGrip.isFiringHandLeft()) {
            ROCK_LOG_SAMPLE_INFO(Weapon, 1000,
                "Held object preserved through pending equipped hand transfer: hand={} ref={:08X} target={:08X} firingHand={}->{}",
                hand.handName(), hand.getHeldRef() ? hand.getHeldRef()->GetFormID() : 0u,
                _equipped.transition.pendingGrip().targetWeaponFormID,
                _twoHandedGrip.isFiringHandLeft() ? "left" : "right", firingHandIsLeft ? "left" : "right");
        }

        /*
         * The equipped-weapon manual ownership path consumes the firing
         * hand's grab edges earlier this frame. Reuse that single consumed
         * snapshot for the same physical hand; re-reading would see
         * cleared edges and starve free-hand world grabs of press/release
         * input.
         */
        GrabButtonState grabInput = consumeHandGrabInput();
        if (inputSuppressionState.deferredGrabRelease) {
            if (grabInput.held) {
                inputSuppressionState.deferredGrabRelease = false;
            } else {
                if (!grabInput.released &&
                    (hand.isHolding() ||
                        _touchGrabRuntime.isHandActive(isLeft) ||
                        hand.getState() == HandState::SelectionLocked ||
                        hand.getState() == HandState::Pulled)) {
                    grabInput.released = true;
                }
                inputSuppressionState.deferredGrabRelease = false;
            }
        }
        const auto rawGrabInput = grabInput;
        if (retainedWeaponInput && hand.isHolding()) {
            // Keep raw input for other gestures. Only loose-grab release is
            // latched, and its second press cannot leak into acquisition.
            const bool release = retainedWeapon.inputState ==
                transferred_weapon_grab_policy::State::ReleaseRequested;
            grabInput.held = !release;
            grabInput.pressed = false;
            grabInput.released = release;
        }

        /*
         * Provider-registered touch targets consume the same physical
         * grip edge as ordinary grabs, but live in a separate runtime so
         * ROCK's loose-object selection policy continues to reject static
         * and keyframed bodies. An explicit body registration always wins
         * over a wildcard fixed-surface registration when one press has
         * contact evidence for both.
         */
        if (_touchGrabRuntime.isHandActive(isLeft)) {
            TouchGrabRuntime::HandReport touchReport{};
            (void)_touchGrabRuntime.getHandReport(isLeft, touchReport);
            if (global_surface_grab_policy::releaseOnInput(
                    touchReport.commandOwned, rawGrabInput.held, rawGrabInput.released)) {
                _touchGrabRuntime.releaseHand(
                    isLeft,
                    frame.bhkWorld,
                    frame.hknpWorld,
                    provider::RockProviderTouchGrabReleaseReasonV1::
                        GripReleased,
                    collisionGeneration);
            }
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry(
                "touch-grab-active",
                true);
            clearGameplayCandidatesForHand(hand, isLeft);
            hand.cancelGrabVisualReturn(
                "touch-grab-active");
            return false;
        }


        outPrelude.grabInput = grabInput;
        outPrelude.rawGrabInput = rawGrabInput;
        outPrelude.heldWeaponAtFrameStart = heldWeaponAtFrameStart;
        outPrelude.heldWeaponEquipTriggerPressedEdge = heldWeaponEquipTriggerPressedEdge;
        return true;
    }

    bool PhysicsInteraction::processTouchGrabInput(
        const PhysicsFrameContext& frame,
        Hand& hand,
        bool isLeft,
        const GrabInputHandContext& context,
        const GrabInputHandPrelude& prelude)
    {
        const auto rawGrabInput = prelude.rawGrabInput;
        const auto handIndex = isLeft ? 1u : 0u;
        const auto worldGeneration = context.worldGeneration;
        const auto skeletonGeneration = context.skeletonGeneration;
        const auto providerGeneration = context.providerGeneration;
        const auto collisionGeneration = context.collisionGeneration;
        auto& inputIntentState = _grabInput.intentStates[handIndex];
        auto& peerHeldJoinRetryState = _grabInput.peerHeldJoinRetryStates[handIndex];

        const auto handState = hand.getState();
        if (rawGrabInput.pressed) {
            const auto& probe = _powerArmorProbeDiagnostics[handIndex];
            const auto& candidate = _powerArmorCandidates[handIndex];
            ROCK_LOG_INFO(Hand, "PA proximity probe: hand={} frame={} stage={} hits={} refs={} paRefs={} bones={} nearest={:.2f}gu candidate={} form={:08X} point={} heldInput={} visualAuthority={}",
                isLeft ? "left" : "right", _powerArmorCandidateFrame, probe.stage, probe.hits, probe.references,
                probe.armorReferences, probe.bones, probe.nearestDistanceGame, candidate.valid, candidate.referenceFormId,
                static_cast<std::uint32_t>(candidate.point), rawGrabInput.held, runtime_state::currentFrame().visualAuthorityAvailable);
        }
        const bool touchGrabStateAvailable =
            handState == HandState::Idle ||
            handState == HandState::SelectedClose ||
            handState == HandState::SelectedFar;
        const bool touchGrabPhysicsWritesAllowed =
            physicsWritesAllowedForWorld(frame.hknpWorld);
        /*
         * One grip edge has a strict candidate order: explicit provider touch
         * targets, a nearby native PA point, an eligible close-selected object,
         * then provider wildcard or built-in world surfaces. The close selection was classified as
         * grabbable by the selection path; a far selection does not suppress an
         * intentional surface grab at the hand.
         */
        const bool closeObjectCandidate =
            handState == HandState::SelectedClose &&
            hand.hasSelection() &&
            !hand.getSelection().isFarSelection;
        const bool canTryTouchGrab =
            rawGrabInput.pressed &&
            rawGrabInput.held &&
            frame.worldReady &&
            !frame.menuBlocked &&
            !input_remap_runtime::isMenuInputActive() &&
            !hand.isHolding() &&
            touchGrabStateAvailable &&
            !hand.hasActivePullCatchIntent() &&
            !hand.hasPendingActorEquipmentDropHandoff() &&
            !_forceGrab.pendingCommits[handIndex].active &&
            touchGrabPhysicsWritesAllowed;
        if (rawGrabInput.pressed && !canTryTouchGrab) {
            ROCK_LOG_INFO(
                Hand,
                "Touch grab edge gated: hand={} held={} worldReady={} menuBlocked={} menuInput={} holding={} handState={} stateAvailable={} pullCatch={} actorDrop={} forceCommit={} physicsWrites={}",
                isLeft ? "left" : "right",
                rawGrabInput.held,
                frame.worldReady,
                frame.menuBlocked,
                input_remap_runtime::isMenuInputActive(),
                hand.isHolding(),
                static_cast<std::uint32_t>(handState),
                touchGrabStateAvailable,
                hand.hasActivePullCatchIntent(),
                hand.hasPendingActorEquipmentDropHandoff(),
                _forceGrab.pendingCommits[handIndex].active,
                touchGrabPhysicsWritesAllowed);
        }
        if (canTryTouchGrab) {
            _touchGrabRuntime.beginAttemptDiagnostics();
            /*
             * Elapsed-time contact freshness for touch-grab candidacy:
             * the historical 4-frame window at the 90 Hz tuning baseline,
             * now identical at every game frame rate.
             */
            constexpr float
                kTouchGrabContactFreshnessSeconds = 4.0f / 90.0f;
            const auto contacts =
                hand.collectFreshSemanticContactsWithinSeconds(
                    kTouchGrabContactFreshnessSeconds);
            const auto surfaceContacts =
                _dynamicHandCollision.collectFreshSurfaceContacts(
                    isLeft,
                    0xFFFF'FFFFu,
                    kTouchGrabContactFreshnessSeconds);
            const auto tryTargetClass =
                [&](const TouchGrabRuntime::TargetClass
                        targetClass) {
                    const auto tryContacts =
                        [&](const hand_semantic_contact_state::
                                SemanticContactCollection& candidates,
                            const TouchGrabRuntime::ContactSource source) {
                            for (std::size_t index = 0;
                                 index < candidates.count;
                                 ++index) {
                                if (_touchGrabRuntime.tryAcquire(
                                        isLeft,
                                        candidates.records[index],
                                        frame.bhkWorld,
                                        frame.hknpWorld,
                                        worldGeneration,
                                        skeletonGeneration,
                                        providerGeneration,
                                        collisionGeneration,
                                        targetClass,
                                        source,
                                        closeObjectCandidate)) {
                                    return true;
                                }
                            }
                            return false;
                        };
                    if (tryContacts(
                            contacts,
                            TouchGrabRuntime::ContactSource::
                                SemanticHand)) {
                        return true;
                    }
                    return tryContacts(
                        surfaceContacts,
                        TouchGrabRuntime::ContactSource::
                            DynamicSurface);
                };
            const bool touchGrabAcquired =
                tryTargetClass(
                    TouchGrabRuntime::TargetClass::
                        Explicit) ||
                _touchGrabRuntime.tryAcquirePowerArmor(isLeft, _powerArmorCandidates[handIndex],
                    frame.bhkWorld, frame.hknpWorld, worldGeneration, skeletonGeneration,
                    providerGeneration, collisionGeneration) ||
                tryTargetClass(
                    TouchGrabRuntime::TargetClass::
                        Wildcard) ||
                tryTargetClass(TouchGrabRuntime::TargetClass::Fallback);
            if (touchGrabAcquired) {
                TouchGrabRuntime::HandReport touchGrabReport{};
                if (g_rockConfig.rockSurfaceGrabHapticsEnabled &&
                    _touchGrabRuntime.getHandReport(
                        isLeft,
                        touchGrabReport) &&
                    touchGrabReport.kind ==
                        provider::RockProviderTouchGrabKindV1::
                            FixedAnchor) {
                    (void)_feedbackHaptics.queue(
                        isLeft ?
                            feedback_haptics::FeedbackHand::Left :
                            feedback_haptics::FeedbackHand::Right,
                        g_rockConfig.
                            rockSurfaceGrabHapticDurationSeconds,
                        g_rockConfig.
                            rockSurfaceGrabHapticIntensity);
                }
                if (hand.hasSelection()) {
                    hand.clearSelectionState(false);
                }
                grab_input_intent_policy::reset(
                    inputIntentState);
                cancelPeerHeldJoinRetry(
                    hand,
                    peerHeldJoinRetryState,
                    "touch-grab-acquired",
                    true);
                clearGameplayCandidatesForHand(
                    hand,
                    isLeft);
                _twoHandedGrip.cancelHandVisualReturn(
                    isLeft,
                    "touch-grab-acquired");
                hand.cancelGrabVisualReturn(
                    "touch-grab-acquired");
                return false;
            }
            const auto attempt =
                _touchGrabRuntime.getAttemptReport();
            ROCK_LOG_INFO(
                Hand,
                "Touch grab attempt rejected: hand={} semanticContacts={} surfaceContacts={} failure={} targetClass={} source={} body={} layer={} motionClass={} motionProperties={} motionIndex={} latchFailure={} meshFailure={}",
                isLeft ? "left" : "right",
                contacts.count,
                surfaceContacts.count,
                static_cast<std::uint32_t>(attempt.failure),
                static_cast<std::uint32_t>(attempt.targetClass),
                static_cast<std::uint32_t>(attempt.contactSource),
                attempt.bodyId,
                attempt.collisionLayer,
                static_cast<std::uint32_t>(attempt.motionClass),
                attempt.motionPropertiesId,
                attempt.motionIndex,
                attempt.surfaceLatchFailure,
                attempt.surfaceMeshFailure);
        }


        return true;
    }

    void PhysicsInteraction::processGrabIntentAndCommit(
        const PhysicsFrameContext& frame,
        Hand& hand,
        bool isLeft,
        const GrabInputHandContext& context,
        const GrabInputHandPrelude& prelude)
    {
        auto* hknp = context.hknp;
        const bool ambidextrousHandoffAvailable = context.ambidextrousHandoffAvailable;
        const auto& firingGripModes = context.firingGripModes;
        const bool gripZoneSettleEquipEnabled = context.gripZoneSettleEquipEnabled;
        const auto& farHmdConeGate = context.farHmdConeGate;
        const auto& handInput = isLeft ? frame.left : frame.right;
        auto& inputIntentState = _grabInput.intentStates[isLeft ? 1u : 0u];
        auto& peerHeldJoinRetryState = _grabInput.peerHeldJoinRetryStates[isLeft ? 1u : 0u];
        auto& triggerEquipIntent = _grabInput.heldWeaponTriggerEquipIntents[isLeft ? 1u : 0u];
        auto& shoulderStashState = _grabInput.shoulderStashStates[isLeft ? 1u : 0u];
        auto& mouthConsumeState = _grabInput.mouthConsumeStates[isLeft ? 1u : 0u];
        const bool heldWeaponAtFrameStart = prelude.heldWeaponAtFrameStart;
        const bool heldWeaponEquipTriggerPressedEdge = prelude.heldWeaponEquipTriggerPressedEdge;
        auto grabInput = prelude.grabInput;
        const auto rawGrabInput = prelude.rawGrabInput;

        if (triggerEquipIntent.pending) {
            triggerEquipIntent.remainingSeconds -= (std::max)(0.0f, frame.deltaSeconds);
            if (triggerEquipIntent.remainingSeconds <= 0.0f ||
                (triggerEquipIntent.grabIdentity != 0 &&
                    (!hand.isHolding() || triggerEquipIntent.grabIdentity != hand.heldGrabIdentity()))) {
                triggerEquipIntent = {};
            }
        }
        /*
         * Input and grab commit are sampled in the same frame but the
         * held-weapon equip block runs before the selected-object commit.
         * Retain a same-hand trigger edge only for the selected weapon and
         * replay it after that exact ref becomes held. Without this, a
         * quick grip+trigger gesture lost the left trigger edge forever;
         * the right hand appeared more reliable only because its legacy
         * settle auto-equip could mask the loss.
         */
        if (heldWeaponEquipTriggerPressedEdge && !heldWeaponAtFrameStart && rawGrabInput.held && hand.hasSelection()) {
            auto* selectedRef = hand.getSelection().refr;
            auto* selectedBase = selectedRef ? selectedRef->GetObjectReference() : nullptr;
            const auto* selectedWeapon = selectedBase ? selectedBase->As<RE::TESObjectWEAP>() : nullptr;
            const bool throwable = selectedWeapon &&
                (selectedWeapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
                    selectedWeapon->weaponData.type == RE::WEAPON_TYPE::kMine);
            if (selectedWeapon && !throwable) {
                triggerEquipIntent = HeldWeaponTriggerEquipIntent{
                    .pending = true,
                    .formID = selectedRef->GetFormID(),
                    .remainingSeconds = 0.35f,
                };
            }
        }
        if (grabInput.pressed &&
            selection_state_policy::canProcessSelectedState(hand.getState()) &&
            hand.hasSelection() &&
            hand.getSelection().isFarSelection &&
            !hand.hasPendingActorEquipmentDropHandoff() &&
            !hand.hasPendingPullCatchCommit()) {
            float hmdConeDot = -1.0f;
            if (!selectedObjectPassesFarHmdCone(hknp, hand.getSelection(), farHmdConeGate, &hmdConeDot)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: far grab press ignored outside HMD cone formID={:08X} hmdDot={:.3f} minDot={:.3f}",
                    hand.handName(),
                    hand.getSelection().refr ? hand.getSelection().refr->GetFormID() : 0,
                    hmdConeDot,
                    farHmdConeGate.minDot);
                hand.clearSelectionState(true);
                grab_input_intent_policy::reset(inputIntentState);
                return;
            }
        }

        if (hand.hasArrivedPullCatchIntent() && !hand.hasPendingPullCatchCommit()) {
            auto* pullCatchRef = hand.getPullCatchIntentRef();
            if (pull_motion_math::kCatchWideReacquireEnabled &&
                hand.reacquirePullCatchCloseSelection(frame.bhkWorld,
                    frame.hknpWorld,
                    handInput.grabAnchorWorld,
                    handInput.closeSelectionDirectionWorld,
                    pull_motion_math::kCatchWideReacquireRadiusGameUnits,
                    pull_motion_math::kCatchWideReacquireMaximumBodyDistanceGameUnits)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand restored stale pull catch commit with target-specific wide close reacquire",
                    hand.handName());
            } else {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand cancelled stale pull catch commit because selected close ref/body no longer matches the pull owner",
                    hand.handName());
                hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-stale-reacquire-failed");
                hand.clearSelectionState(true);
                releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                return;
            }
        }

        const Hand& peerForInputIntent = isLeft ? _rightHand : _leftHand;
        auto* peerHeldRefForInput = peerForInputIntent.getHeldRef();
        const auto& peerSavedObjectStateForInput = peerForInputIntent.getSavedObjectState();
        const bool peerHoldingLooseObject =
            !hand.isHolding() &&
            peerForInputIntent.isHolding() &&
            peerHeldRefForInput &&
            peerSavedObjectStateForInput.isValid() &&
            peerSavedObjectStateForInput.targetKind == grab_target::Kind::LooseObject;
        const bool peerHeldCloseSelectionReady =
            peerHoldingLooseObject &&
            hand.hasSelection() &&
            !hand.getSelection().isFarSelection &&
            hand.getSelection().refr == peerHeldRefForInput;
        const bool selectedPressCandidate =
            !hand.isHolding() &&
            hand.hasSelection() &&
            selection_state_policy::canProcessSelectedState(hand.getState());
        const bool pullCatchPressCandidate = !hand.isHolding() && hand.hasPendingPullCatchCommit();
        const auto closeRetryTarget = [&]() -> grab_input_intent_policy::Target {
            if (!selectedPressCandidate || pullCatchPressCandidate || peerHeldCloseSelectionReady) {
                return {};
            }
            const auto& selection = hand.getSelection();
            if (selection.isFarSelection || selection.targetKind != grab_target::Kind::LooseObject || !selection.refr) {
                return {};
            }
            return { selection.refr->GetFormID(), selection.bodyId.value };
        }();
        const auto intentDecision = grab_input_intent_policy::update(
            inputIntentState,
            grab_input_intent_policy::RawButtonState{
                .held = grabInput.held,
                .pressed = grabInput.pressed,
                .released = grabInput.released,
            },
            selectedPressCandidate || pullCatchPressCandidate || peerHeldCloseSelectionReady,
            hand.isHolding(),
            frame.deltaSeconds,
            grab_input_intent_policy::Config{
                .enabled = g_rockConfig.rockGrabInputIntentStateEnabled,
                .leewaySeconds = g_rockConfig.rockGrabInputLeewaySeconds,
                .forceSeconds = g_rockConfig.rockGrabInputForceSeconds,
            }, closeRetryTarget);
        grabInput.held = intentDecision.held;
        grabInput.pressed = intentDecision.pressed;
        grabInput.released = intentDecision.released;
        grabInput.syntheticPressed = intentDecision.syntheticPressed;
        if (intentDecision.syntheticPressed) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} hand delivered latched grab input intent state={} reason={}",
                hand.handName(),
                grab_input_intent_policy::stateName(intentDecision.state),
                intentDecision.reason);
        }

        auto attemptPeerHeldCloseJoinSelection = [&](const char** outRefusalReason = nullptr) {
            auto setRefusal = [&](const char* reason) {
                if (outRefusalReason) {
                    *outRefusalReason = reason ? reason : "unknown";
                }
                return false;
            };

            if (hand.isHolding()) {
                return setRefusal("hand-already-holding");
            }

            if (outRefusalReason) {
                *outRefusalReason = "not-evaluated";
            }

            const Hand& peer = isLeft ? _rightHand : _leftHand;
            if (!peer.isHolding() || !peer.getHeldRef()) {
                return setRefusal("peer-not-holding");
            }
            if (!peer.getSavedObjectState().isValid() || peer.getSavedObjectState().targetKind != grab_target::Kind::LooseObject) {
                return setRefusal("peer-target-not-loose-object");
            }

            if (hand.hasSelection() && hand.getSelection().refr != peer.getHeldRef()) {
                return setRefusal(hand.getSelection().isFarSelection ? "unrelated-far-selection" : "unrelated-close-selection");
            }

            const bool hadPeerHeldCloseSelection =
                hand.hasSelection() && hand.getSelection().refr == peer.getHeldRef() && !hand.getSelection().isFarSelection;
            const bool refreshedPeerHeldSelection = hand.acquirePeerHeldCloseSelection(frame.bhkWorld,
                frame.hknpWorld,
                peer.getSavedObjectState(),
                peer.getHeldBodyIds(),
                handInput.grabAnchorWorld,
                handInput.closeSelectionDirectionWorld,
                outRefusalReason);
            if (!refreshedPeerHeldSelection && hadPeerHeldCloseSelection) {
                hand.clearSelectionState(false);
            }
            return refreshedPeerHeldSelection;
        };

        const bool unrelatedSelectionForPeerJoin =
            peerHoldingLooseObject &&
            hand.hasSelection() &&
            hand.getSelection().refr != peerHeldRefForInput;
        const std::uint32_t peerHeldFormIdForRetry = peerHeldRefForInput ? peerHeldRefForInput->GetFormID() : 0u;
        const bool peerStillHoldingRetryObject =
            peerHoldingLooseObject &&
            (!peerHeldJoinRetryState.active ||
                (peerHeldJoinRetryState.peerFormId != 0 && peerHeldJoinRetryState.peerFormId == peerHeldFormIdForRetry));
        const char* retryLastRefusalBeforeUpdate =
            peerHeldJoinRetryState.lastRefusalReason ? peerHeldJoinRetryState.lastRefusalReason : "none";
        const auto retryAttemptsBeforeUpdate = peerHeldJoinRetryState.attempts;
        const auto peerHeldRetryDecision = peer_held_join_retry_policy::update(
            peerHeldJoinRetryState,
            peer_held_join_retry_policy::Input{
                .rawHeld = rawGrabInput.held,
                .rawPressed = rawGrabInput.pressed,
                .rawReleased = rawGrabInput.released,
                .normalGrabSuppressed = false,
                .handHolding = hand.isHolding(),
                .peerHoldingLooseObject = peerHoldingLooseObject,
                .peerStillHoldingSameObject = peerStillHoldingRetryObject,
                .unrelatedSelection = unrelatedSelectionForPeerJoin,
                .grabSucceeded = false,
                .peerFormId = peerHeldFormIdForRetry,
                .deltaSeconds = frame.deltaSeconds,
                .config = peer_held_join_retry_policy::Config{
                    .enabled = g_rockConfig.rockGrabInputIntentStateEnabled,
                    .leewaySeconds = g_rockConfig.rockGrabInputLeewaySeconds,
                    .forceSeconds = g_rockConfig.rockGrabInputForceSeconds,
                },
            });
        if (peerHeldRetryDecision.started) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand peer-held join retry started: peerFormID={:08X} window={:.3f}s interval={:.3f}s",
                hand.handName(),
                peerHeldJoinRetryState.peerFormId,
                peerHeldJoinRetryState.windowSeconds,
                peer_held_join_retry_policy::retryIntervalSeconds(peer_held_join_retry_policy::Config{
                    .enabled = g_rockConfig.rockGrabInputIntentStateEnabled,
                    .leewaySeconds = g_rockConfig.rockGrabInputLeewaySeconds,
                    .forceSeconds = g_rockConfig.rockGrabInputForceSeconds,
                }));
        }
        if (peerHeldRetryDecision.cancelled) {
            grab_input_intent_policy::reset(inputIntentState);
            if (!peerHeldRetryDecision.success &&
                hand.hasSelection() &&
                !hand.getSelection().isFarSelection &&
                hand.getSelection().refr == peerHeldRefForInput) {
                grabInput.pressed = false;
                grabInput.syntheticPressed = false;
            }
            if (peerHeldRetryDecision.success) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand peer-held join retry succeeded: reason={} peerFormID={:08X} attempts={}",
                    hand.handName(),
                    peerHeldRetryDecision.reason,
                    peerHeldFormIdForRetry,
                    retryAttemptsBeforeUpdate);
            } else {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand peer-held join retry cancelled: reason={} peerFormID={:08X} attempts={} lastRefusal={}",
                    hand.handName(),
                    peerHeldRetryDecision.reason,
                    peerHeldFormIdForRetry,
                    retryAttemptsBeforeUpdate,
                    retryLastRefusalBeforeUpdate);
            }
        }

        bool peerHeldRetryAttemptDue = peerHeldRetryDecision.attempt && peerHeldJoinRetryState.active;
        bool peerHeldRetryRefreshedSelection = false;
        if (peerHeldRetryAttemptDue) {
            const char* refusalReason = "not-attempted";
            peerHeldRetryRefreshedSelection = attemptPeerHeldCloseJoinSelection(&refusalReason);
            if (peerHeldJoinRetryState.active) {
                peerHeldJoinRetryState.lastRefusalReason = peerHeldRetryRefreshedSelection ? "selection-acquired" : (refusalReason ? refusalReason : "unknown");
            }
            if (peerHeldRetryRefreshedSelection) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} hand peer-held join retry refreshed close selection: peerFormID={:08X} attempt={} source={}",
                    hand.handName(),
                    peerHeldJoinRetryState.peerFormId,
                    peerHeldJoinRetryState.attempts,
                    refusalReason ? refusalReason : "unknown");
            }
        }

        const bool peerHeldRetryCommitIntent =
            peerHeldRetryAttemptDue &&
            peerHeldJoinRetryState.active &&
            rawGrabInput.held &&
            !hand.isHolding() &&
            hand.hasSelection() &&
            !hand.getSelection().isFarSelection &&
            hand.getSelection().refr == peerHeldRefForInput;

        auto selectedObjectInteractionBlocked = [&]() {
            const auto& sel = hand.getSelection();
            auto* selRef = sel.refr;
            if (!selRef) {
                return false;
            }

            auto* baseObj = selRef->GetObjectReference();
            if (!baseObj) {
                return false;
            }

            const char* typeStr = baseObj->GetFormTypeString();
            const std::string_view formType = typeStr ? std::string_view(typeStr) : std::string_view{};

            bool hasMotionProps = false;
            std::uint16_t motionProps = 0;
            if (formType == "ACTI" && sel.bodyId.value != 0x7FFF'FFFF && hknp) {
                hasMotionProps = havok_runtime::tryReadBodyMotionPropertiesId(hknp, sel.bodyId, motionProps);
            }

            const bool isLiveNpc = formType == "NPC_" && !selRef->IsDead(false);
            const bool blocked = grab_interaction_policy::shouldBlockSelectedObjectInteractionForTarget(sel.targetKind, formType, isLiveNpc, hasMotionProps, motionProps);
            if (blocked) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: selected object interaction blocked targetKind={} formType={} formID={:08X} far={} motionProps={} hasMotionProps={}",
                    hand.handName(),
                    grab_target::name(sel.targetKind),
                    formType.empty() ? "???" : typeStr,
                    selRef->GetFormID(),
                    sel.isFarSelection ? "yes" : "no",
                    motionProps,
                    hasMotionProps ? "yes" : "no");
            }
            return blocked;
        };
        auto attemptSelectedGrab = [&]() {
            const auto& transform = handInput.rawHandWorld;

            const auto sharedContext = makeGrabSharedObjectContext(hand, isLeft);
            const bool grabbedFromPullCatchCommit = hand.hasPendingPullCatchCommit();
            prepareDynamicWorldCarCollisionForGrab(frame.bhkWorld, hknp, hand.getSelection().refr);
            const auto grabResult = hand.grabSelectedObject(hknp,
                transform,
                g_rockConfig.rockGrabLinearTau,
                g_rockConfig.rockGrabLinearDamping,
                g_rockConfig.rockGrabConstraintMaxForce,
                g_rockConfig.rockGrabLinearProportionalRecovery,
                g_rockConfig.rockGrabLinearConstantRecovery,
                &_bodyBoneColliders,
                sharedContext);

            const bool grabbed = grabResult == GrabAttemptResult::Grabbed;
            if (grabResult == GrabAttemptResult::ContactUnavailable &&
                g_rockConfig.rockGrabInputIntentStateEnabled &&
                rawGrabInput.held && !rawGrabInput.released && closeRetryTarget.valid()) {
                grab_input_intent_policy::retainContactRetry(inputIntentState, closeRetryTarget);
                ROCK_LOG_SAMPLE_DEBUG(Hand, g_rockConfig.rockLogSampleMilliseconds,
                    "{} hand retained close grab intent after contact refusal: formID={:08X} body={}",
                    hand.handName(), closeRetryTarget.formId, closeRetryTarget.bodyId);
            }

            if (grabbed) {
                if (sharedContext.joiningPeerHeldObject) {
                    Hand& peer = isLeft ? _rightHand : _leftHand;
                    const auto& peerInput = isLeft ? frame.right : frame.left;
                    if (!peer.promoteHeldObjectToConstraintDrive(frame.bhkWorld,
                            hknp,
                            peerInput.rawHandWorld,
                            g_rockConfig.rockGrabLinearTau,
                            g_rockConfig.rockGrabLinearDamping,
                            g_rockConfig.rockGrabConstraintMaxForce,
                            g_rockConfig.rockGrabLinearProportionalRecovery,
                            g_rockConfig.rockGrabLinearConstantRecovery,
                            "peer-hand-joined-loose-object")) {
                        auto* joinedRef = hand.getHeldRef();
                        ROCK_LOG_WARN(Hand,
                            "{} hand: rolling back shared grab because peer hand could not promote to constraint drive formID={:08X}",
                            hand.handName(),
                            joinedRef ? joinedRef->GetFormID() : 0);
                        hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, makeGrabReleaseContext(hand, isLeft));
                        return false;
                    }
                }
                auto* heldRef = hand.getHeldRef();
                claimObject(heldRef, claimOwnerForHand(isLeft));
                dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef, heldRef ? heldRef->GetFormID() : 0, 0);
                dispatchGrabCommittedEvent(isLeft, heldRef, hand.getSavedObjectState().bodyId.value, hknp);
                if (grabbedFromPullCatchCommit) {
                    dispatchSimpleGrabEvent(GrabEventType::PullCatchSucceeded, isLeft, heldRef, hand.getSavedObjectState().bodyId.value);
                }
            }
            return grabbed;
        };
        auto dispatchHeldObjectEventByFormID =
            [&](GrabEventType type, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t primaryBodyId, bool eventIsLeft) {
                GrabEventData eventData{};
                eventData.type = type;
                eventData.sourceKind = GrabEventSourceKind::HeldObject;
                eventData.isLeft = eventIsLeft;
                eventData.refr = refr;
                eventData.formID = formID != 0 ? formID : (refr ? refr->GetFormID() : 0);
                eventData.primaryBodyId = primaryBodyId;
                dispatchGrabEvent(eventData);
            };
        auto dispatchShoulderStashEvent = [&](GrabEventType type,
                                               RE::TESObjectREFR* refr,
                                               std::uint32_t formID,
                                              std::uint32_t primaryBodyId,
                                              const shoulder_stash::Decision& decision) {
            GrabEventData eventData{};
            eventData.type = type;
            eventData.sourceKind = GrabEventSourceKind::HeldObject;
            eventData.isLeft = isLeft;
            eventData.refr = refr;
            eventData.formID = formID != 0 ? formID : (refr ? refr->GetFormID() : 0);
            eventData.primaryBodyId = primaryBodyId;
            eventData.secondaryBodyId = decision.shoulderBodyId;
            eventData.positionGame[0] = decision.nearestPointGame.x;
            eventData.positionGame[1] = decision.nearestPointGame.y;
            eventData.positionGame[2] = decision.nearestPointGame.z;
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_POSITION_VALID;
            if (std::isfinite(decision.speedGameUnitsPerSecond)) {
                eventData.speedGameUnitsPerSecond = decision.speedGameUnitsPerSecond;
                eventData.flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
            }
            eventData.intensityHint = std::clamp(std::isfinite(decision.confidence) ? decision.confidence : 0.0f, 0.0f, 1.0f);
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID;
            dispatchGrabEvent(eventData);
        };
        auto dispatchMouthConsumeEvent = [&](GrabEventType type,
                                             RE::TESObjectREFR* refr,
                                             std::uint32_t formID,
                                             std::uint32_t primaryBodyId,
                                             const mouth_consume::Decision& decision) {
            GrabEventData eventData{};
            eventData.type = type;
            eventData.sourceKind = GrabEventSourceKind::HeldObject;
            eventData.isLeft = isLeft;
            eventData.refr = refr;
            eventData.formID = formID != 0 ? formID : (refr ? refr->GetFormID() : 0);
            eventData.primaryBodyId = primaryBodyId;
            eventData.positionGame[0] = decision.mouthCenterGame.x;
            eventData.positionGame[1] = decision.mouthCenterGame.y;
            eventData.positionGame[2] = decision.mouthCenterGame.z;
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_POSITION_VALID;
            if (std::isfinite(decision.speedGameUnitsPerSecond)) {
                eventData.speedGameUnitsPerSecond = decision.speedGameUnitsPerSecond;
                eventData.flags |= ROCK_GRAB_EVENT_FLAG_SPEED_VALID;
            }
            eventData.intensityHint = std::clamp(std::isfinite(decision.confidence) ? decision.confidence : 0.0f, 0.0f, 1.0f);
            eventData.flags |= ROCK_GRAB_EVENT_FLAG_INTENSITY_VALID;
            dispatchGrabEvent(eventData);
        };

        /*
         * Native idle-grip harvesting is acquisition preparation, not a
         * hover-haptic side effect. Offer the retained held loose weapon or
         * open-hand selection before grab input is committed below. The
         * retained reference crosses native asynchronous progress safely;
         * the visual equip bridge alone owns only a scene model. A null
         * candidate still advances an in-flight load, so pull travel can
         * hide the load without blocking the frame thread.
         */
        RE::NiPointer<RE::TESObjectREFR> nativeIdleGripCandidate{};
        if (hand.isHoldingLooseWeapon()) {
            nativeIdleGripCandidate = hand.getSavedObjectState().retainedRef;
        } else if (!hand.isHolding() && hand.hasSelection() && !input_remap_runtime::isMenuInputActive()) {
            nativeIdleGripCandidate = hand.getSelection().retainedRef;
        }
        // Evaluate both physical hands independently, including a support-only
        // first grab. Cold native sampling is already driven by this candidate.
        loose_weapon_grip_zone::updateNearGrabCandidate(isLeft,
            input_remap_runtime::isMenuInputActive() ? nullptr : nativeIdleGripCandidate.get(),
            nativeIdleGripCandidate && (isLeft ? _rightHand : _leftHand).isHolding() &&
                (isLeft ? _rightHand : _leftHand).getHeldRef() == nativeIdleGripCandidate.get());
        native_idle_grip_preharvest::observeCandidate(std::move(nativeIdleGripCandidate));

        const bool heldWeaponEquipOwnershipEligible =
            equipped_weapon_manual_ownership_policy::shouldStartHeldWeaponEquipOwnership(
                equipped_weapon_manual_ownership_policy::HeldWeaponEquipOwnershipInput{
                    .modes = firingGripModes,
                    .handIsLeft = isLeft,
                    .holdingLooseWeapon = hand.isHoldingLooseWeapon(),
                });
        /*
         * The canonical firing-grip frame serves two independent consumers:
         * optional grip-zone settle detection and hand-preserving trigger
         * equip. Keep producing it whenever either consumer can use it.
         */
        loose_weapon_grip_zone::updateHeldLooseWeapon(
            isLeft,
            equipped_weapon_manual_ownership_policy::shouldTrackHeldWeaponGripFrame(
                hand.isHoldingLooseWeapon(),
                gripZoneSettleEquipEnabled,
                heldWeaponEquipOwnershipEligible),
            hand.getHeldRef(),
            hand.getState() == HandState::HeldBody,
            frame.deltaSeconds,
            _equipped.handlingSettings.gripZoneEquipRadiusGameUnits,
            !hand.isHoldingAuthoredSupportGrip());

        /*
         * Grip-zone hover probe: while either OPEN hand's selection
         * candidate is a loose weapon, feel out whether grabbing right now
         * would land the palm inside the firing-grip zone (and therefore
         * equip after the settle into that same physical hand while the
         * addon authority is active). Both hands
         * use the same projected FRIK firing-grip radius; grenades never
         * reach the equip path so they never hum. Vibration stops on grab
         * because the hover candidate goes null while holding.
         */
        RE::TESObjectREFR* gripZoneHoverCandidate = nullptr;
        if (_equipped.handlingSettings.gripZoneHoverHapticsEnabled &&
            gripZoneSettleEquipEnabled &&
            !hand.isHolding() &&
            hand.hasSelection() &&
            !input_remap_runtime::isMenuInputActive()) {
            auto* selectionRef = hand.getSelection().refr;
            if (selectionRef && !loose_grenade_runtime::isThrowableRef(selectionRef)) {
                gripZoneHoverCandidate = selectionRef;
            }
        }
        loose_weapon_grip_zone::updateHoverCandidateWeapon(
            isLeft,
            gripZoneHoverCandidate,
            _equipped.handlingSettings.gripZoneEquipRadiusGameUnits);
        if (loose_weapon_grip_zone::isGripZoneHoverInsideRadius(isLeft)) {
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                _equipped.handlingSettings.gripZoneHoverHapticIntensity);
        }

        if (hand.isHolding()) {
            const Hand& peer = isLeft ? _rightHand : _leftHand;
            auto* heldRefForGameplay = hand.getHeldRef();
            const bool heldLooseGrenade = loose_grenade_runtime::isThrowableRef(heldRefForGameplay);
            const bool peerHoldingSameObject =
                heldRefForGameplay && peer.isHolding() && peer.getHeldRef() == heldRefForGameplay;
            const bool replayedSameHandTrigger = triggerEquipIntent.pending &&
                heldRefForGameplay && triggerEquipIntent.formID == heldRefForGameplay->GetFormID();
            const bool acceptedTransfer = _equipped.transition.heldTransfer().wantsEquip(
                isLeft, heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u, hand.heldGrabIdentity());
            const bool heldWeaponEquipTriggerPressed = heldWeaponEquipTriggerPressedEdge || replayedSameHandTrigger || acceptedTransfer;
            if (replayedSameHandTrigger || (heldWeaponEquipTriggerPressedEdge && hand.isHoldingLooseWeapon())) {
                triggerEquipIntent = {};
            }
            const bool heldWeaponGripZoneEquipSettled = !heldLooseGrenade &&
                loose_weapon_grip_zone::isGripZoneEquipSettled(
                    isLeft,
                    _equipped.handlingSettings.gripZoneEquipSettleSeconds);
            const bool heldWeaponEquipRequested = input_remap_policy::shouldRequestHeldWeaponEquip(input_remap_policy::HeldWeaponEquipInput{
                .remapEnabled = true,
                .gameplayInputAllowed = true,
                .menuInputActive = input_remap_runtime::isMenuInputActive(),
                .heldWeaponAtFrameStart = heldWeaponAtFrameStart,
                .heldWeaponNow = hand.isHoldingLooseWeapon(),
                .heldWeaponHand = isLeft ? input_remap_policy::Hand::Left : input_remap_policy::Hand::Right,
                .triggerInputHand = isLeft ? input_remap_policy::Hand::Left : input_remap_policy::Hand::Right,
                .triggerPressedEdge = heldWeaponEquipTriggerPressed,
                .gripZoneEquipEnabled = gripZoneSettleEquipEnabled,
                .gripZoneEquipSettled = heldWeaponGripZoneEquipSettled,
            });

            auto equipHeldWeaponFromHand = [&](const bool triggeredByInput, const char* requestReason, const char* logAction) {
                if (acceptedTransfer && grabInput.released) {
                    _equipped.transition.cancelHeldRequest("incoming-release-before-pickup", held_weapon_transfer::Outcome::Cancelled);
                    return false;
                }
                const auto transitionReason = triggeredByInput ?
                    held_weapon_instant_transition::RequestReason::SameHandTrigger :
                    held_weapon_instant_transition::RequestReason::GripZoneSettle;
                auto* player = RE::PlayerCharacter::GetSingleton();
                const std::uint32_t nativeStateBeforeEquip =
                    f4vr::getNativeWeaponState(player);
                if (grabInput.released && !held_weapon_equip_state_policy::canBeginEquip(nativeStateBeforeEquip)) return false;

                const auto instantReadiness =
                    held_weapon_instant_transition::readinessFor(player);
                if (!instantReadiness.ready) {
                    ROCK_LOG_SAMPLE_WARN(
                        Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand {} held weapon equip blocked before release formID={:08X} instantTransition={}",
                        hand.handName(),
                        logAction ? logAction : "requested",
                        heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u,
                        held_weapon_instant_transition::readinessReasonName(
                            instantReadiness.reason));
                    return false;
                }

                bool equipIsLeft = isLeft;
                const bool supportEquipRequested = triggeredByInput && !peerHoldingSameObject && hand.isHoldingAuthoredSupportGrip();
                if (supportEquipRequested &&
                    (!resolveEquippedWeaponDetachDecision(_equipped.handlingSettings).primaryDetachEnabled ||
                        !frik_visual_authority::canBlockPrimaryHandWeaponPose() ||
                        !TwoHandedGrip::canBeginPrimaryOnlyGripForHand(!isLeft))) {
                    ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Support held equip deferred: support-only carry unavailable hand={}", hand.handName());
                    return false;
                }
                weapon_grip_transfer::Pair pairedGrips{};
                if (peerHoldingSameObject) {
                    const bool requesterPrimary = weapon_grip_transfer::requesterIsPrimary(
                        hand.isHoldingFiringGrip(), peer.isHoldingFiringGrip(), hand.heldGrabIdentity(), peer.heldGrabIdentity());
                    equipIsLeft = requesterPrimary ? isLeft : !isLeft;
                    const auto& primary = equipIsLeft ? _leftHand : _rightHand;
                    const auto& support = equipIsLeft ? _rightHand : _leftHand;
                    const auto* base = heldRefForGameplay ? heldRefForGameplay->GetObjectReference() : nullptr;
                    pairedGrips.weaponFormID = base ? base->GetFormID() : 0;
                    pairedGrips.firingHandIsLeft = equipIsLeft;
                    pairedGrips.arrangement = primary.heldWeaponArrangement();
                    if (!equipped_weapon_manual_ownership_policy::firingGripOwnershipEnabled(firingGripModes) ||
                        !primary.captureWeaponGripTransfer(pairedGrips.primary) || !support.captureWeaponGripTransfer(pairedGrips.support) ||
                        !vanilla_weapon_grip_frame::resolveModelTranslation(pairedGrips.weaponFormID,
                            heldRefForGameplay->Get3D(), pairedGrips.sourceModelTranslation) || !pairedGrips.valid() ||
                        !TwoHandedGrip::canBeginPrimaryOnlyGripForHand(equipIsLeft)) {
                        ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Paired held equip deferred: grip capture or equipped ownership unavailable ref={:08X}",
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0);
                        return false;
                    }
                    ROCK_LOG_INFO(Hand, "Paired held equip captured ref={:08X} triggerHand={} firingHand={} roles=({},{})",
                        heldRefForGameplay->GetFormID(), isLeft ? "left" : "right", equipIsLeft ? "left" : "right",
                        static_cast<unsigned>(pairedGrips.primary.authoredRole), static_cast<unsigned>(pairedGrips.support.authoredRole));
                }
                Hand& equipHand = equipIsLeft ? _leftHand : _rightHand;
                Hand& supportHand = equipIsLeft ? _rightHand : _leftHand;
                const auto& equipHandInput = equipIsLeft ? frame.left : frame.right;
                const bool equipOwnershipEligible = equipped_weapon_manual_ownership_policy::shouldStartHeldWeaponEquipOwnership({
                    .modes = firingGripModes, .handIsLeft = equipIsLeft, .holdingLooseWeapon = equipHand.isHoldingLooseWeapon() });
                PendingEquippedWeaponPrimaryOnlyGripStart pendingGripStart{};
                pendingGripStart.pending = supportEquipRequested || pairedGrips.valid() || equipOwnershipEligible;
                pendingGripStart.pairedGrips = pairedGrips;
                pendingGripStart.isLeft = equipIsLeft;
                pendingGripStart.toggleAcquisitionCommitted =
                    pendingGripStart.pending && !supportEquipRequested;
                if (pendingGripStart.pending && !supportEquipRequested && !pairedGrips.valid()) {
                    pendingGripStart.pairedRelease[equipped_weapon_toggle_grab_policy::handIndex(equipIsLeft)].observe(
                        _equipped.handlingSettings.weaponGrabMode, true,
                        { .held = rawGrabInput.held, .pressed = false, .released = false }, true);
                }
                const bool handCarryAvailable =
                    TwoHandedGrip::canBeginPrimaryOnlyGripForHand(equipIsLeft);
                const bool capturedLooseHold = pairedGrips.valid() || (!supportEquipRequested && pendingGripStart.pending &&
                    handCarryAvailable &&
                    loose_weapon_grip_zone::tryGetFiringHandWeaponLocal(
                        equipIsLeft,
                        pendingGripStart.firingHandWeaponLocal,
                        pendingGripStart.firingGripWeaponLocal));
                if (pairedGrips.valid()) {
                    pendingGripStart.firingHandWeaponLocal = pairedGrips.primary.handWeaponLocal;
                    pendingGripStart.firingGripWeaponLocal = pairedGrips.primary.gripWeaponLocal;
                }
                pendingGripStart.hasFiringHandWeaponLocal = capturedLooseHold;
                pendingGripStart.hasFiringGripWeaponLocal = capturedLooseHold;
                if (equipIsLeft && !supportEquipRequested && !capturedLooseHold) {
                    ROCK_LOG_SAMPLE_WARN(
                        Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "left hand {} held weapon equip blocked: weapon='{}' formID={:08X} ownershipEligible={} canonical weapon-relative left carry unavailable addonAuthority={} integratedDetach={} ambidextrousFiring={} grabHeld={} hFRIKBlockers={} gripFrame={}",
                        logAction ? logAction : "requested",
                        heldRefForGameplay && heldRefForGameplay->GetObjectReference() ?
                            RE::TESFullName::GetFullName(*heldRefForGameplay->GetObjectReference(), false) : "unknown",
                        heldRefForGameplay && heldRefForGameplay->GetObjectReference() ?
                            heldRefForGameplay->GetObjectReference()->GetFormID() : 0u,
                        pendingGripStart.pending,
                        _equipped.handlingSettings.externalAuthorityActive ? "yes" : "no",
                        firingGripModes.integratedDetachEnabled ? "yes" : "no",
                        ambidextrousHandoffAvailable ? "yes" : "no",
                        rawGrabInput.held ? "yes" : "no",
                        handCarryAvailable ? "yes" : "no",
                        pendingGripStart.hasFiringHandWeaponLocal ? "yes" : "no");
                    return false;
                }

                auto* heldRef = equipHand.getHeldRef();
                vanilla_weapon_alignment_telemetry::beginTransferTrace(
                    vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip,
                    equipIsLeft, heldRef ? heldRef->GetFormID() : 0, heldRef ? heldRef->Get3D() : nullptr);
                if (pairedGrips.valid()) {
                    vanilla_weapon_alignment_telemetry::beginTransferTrace(
                        vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip,
                        !equipIsLeft, heldRef ? heldRef->GetFormID() : 0, heldRef ? heldRef->Get3D() : nullptr);
                }
                const auto refreshTransferHand = [&](bool left) {
                    Hand& carryingHand = left ? _leftHand : _rightHand;
                    const auto reference = carryingHand.getSavedObjectState().retainedRef;
                    if (!reference || !carryingHand.isHolding()) return false;
                    carryingHand.updateHeldObject(hknp, (left ? frame.left : frame.right).rawHandWorld,
                        frame.deltaSeconds, g_rockConfig.rockGrabForceFadeInTime, g_rockConfig.rockGrabTauMin,
                        &_bodyBoneColliders, makeGrabReleaseContext(carryingHand, left),
                        left ? &_rightHand : &_leftHand, &(left ? frame.right : frame.left).rawHandWorld);
                    if (carryingHand.isHolding() && carryingHand.getHeldRef() == reference.get()) return true;
                    if (!carryingHand.isHolding()) {
                        ROCK_LOG_WARN(Hand, "Held equip cancelled by current-frame hold validation: hand={} ref={:08X}",
                            left ? "left" : "right", reference->GetFormID());
                        releaseObject(reference.get(), claimOwnerForHand(left));
                        dispatchPhysicsMessage(kPhysMsg_OnRelease, left, reference.get(), reference->GetFormID(), 0);
                        dispatchSimpleGrabEvent(GrabEventType::Released, left, reference.get());
                        clearGameplayCandidatesForHand(carryingHand, left);
                    }
                    return false;
                };
                // Equip bypasses the normal held-update branch below. Advance
                // the outgoing presentation before pickup or bridge-local capture.
                if (peerHoldingSameObject) {
                    const bool leftFirst = held_scene_presentation::leftOwnsSharedAssembly();
                    const bool firstReady = refreshTransferHand(leftFirst);
                    const bool secondReady = refreshTransferHand(!leftFirst);
                    if (!firstReady || !secondReady) return true;
                } else if (!refreshTransferHand(equipIsLeft)) {
                    return true;
                }
                heldRef = equipHand.getHeldRef();
                weapon_grip_transfer::HandGrip singleGrip{};
                RE::NiTransform looseHandWorld{};
                if (supportEquipRequested) {
                    AuthoredWeaponGripPose authored{};
                    RE::NiTransform driverWorld{};
                    auto& support = pendingGripStart.supportGrip;
                    support.isLeft = equipIsLeft;
                    if (!frik_hand_world_authority::tryGetInputDriverWorld(equipIsLeft, driverWorld) ||
                        !weapon_grip_transfer::validFrame(driverWorld) ||
                        !loose_weapon_grip_zone::tryResolveAuthoredGrabPose(equipIsLeft, heldRef,
                            loose_weapon_authored_grab_policy::Role::Support, authored) ||
                        !vanilla_weapon_grip_frame::resolveModelTranslation(authored.weaponFormId,
                            heldRef->Get3D(), support.sourceModelTranslation)) {
                        ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Support held equip deferred: authored support pose or physical driver unavailable hand={}", hand.handName());
                        return false;
                    }
                    support.weaponFormID = authored.weaponFormId;
                    support.weaponInDriver = transform_math::composeTransforms(
                        transform_math::invertTransform(driverWorld), heldRef->Get3D()->world);
                    singleGrip.handWeaponLocal = authored.handWeaponLocal;
                    singleGrip.gripWeaponLocal = computeGrabLegacyPalmPivotAWorldFromHandBasis(authored.handWeaponLocal, equipIsLeft);
                    singleGrip.fingerLocals = authored.fingerLocals;
                    singleGrip.fingerMask = authored.fingerMask;
                    singleGrip.authoredRole = authored.role;
                    singleGrip.hasFingerPose = true;
                    support.grip = singleGrip;
                    if (!support.valid()) return false;
                    pendingGripStart.pairedRelease[equipped_weapon_toggle_grab_policy::handIndex(equipIsLeft)].observe(
                        _equipped.handlingSettings.weaponGrabMode, false,
                        { .held = rawGrabInput.held, .pressed = rawGrabInput.pressed, .released = rawGrabInput.released });
                } else if (!pairedGrips.valid() && equipHand.captureWeaponGripTransfer(singleGrip)) {
                    // Preserve the wrist actually presented by the loose grab,
                    // including a saved/altered hold, before release clears its tag.
                    if (heldRef && heldRef->Get3D() &&
                        frik_hand_world_authority::tryGetPublishedHandWorld(equipIsLeft, looseHandWorld)) {
                        singleGrip.handWeaponLocal = transform_math::composeTransforms(
                            transform_math::invertTransform(heldRef->Get3D()->world), looseHandWorld);
                        if (!singleGrip.valid()) singleGrip = {};
                    } else {
                        singleGrip = {};
                    }
                }
                vanilla_weapon_alignment_telemetry::recordTransferTrace(
                    vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip,
                    equipIsLeft, "equip-source-refreshed", heldRef ? heldRef->Get3D() : nullptr);
                equipHand.captureHeldReleaseMotion(hknp, equipHandInput.rawHandWorld, frame.timing);
                const auto previousEquippedWeaponFormID =
                    currentEquippedWeaponFormId();
                const auto previousNativeInstanceNode =
                    previousEquippedWeaponFormID != 0 ?
                    equipped_weapon_visual_state::observe(
                        previousEquippedWeaponFormID).exactInstance :
                    nullptr;
                const auto transferRole = supportEquipRequested ? held_weapon_transfer::Role::Support :
                    pairedGrips.valid() ? held_weapon_transfer::Role::Paired : held_weapon_transfer::Role::Firing;
                auto& transition = _equipped.transition;
                if (transition.heldTransfer().phase == held_weapon_transfer::Phase::AwaitEquip &&
                    transition.heldTransfer().request.reference == (heldRef ? heldRef->GetFormID() : 0u) &&
                    (transition.heldTransfer().request.role != transferRole || transition.heldTransfer().request.isLeft != equipIsLeft)) {
                    transition.cancelHeldRequest("incoming-grip-arrangement-changed", held_weapon_transfer::Outcome::Cancelled);
                    return true;
                }
                const bool continuingTransfer = transition.heldTransfer().wantsEquip(equipIsLeft,
                    heldRef ? heldRef->GetFormID() : 0u, equipHand.heldGrabIdentity());
                const bool retainOutgoing = continuingTransfer ? transition.heldTransfer().request.retainOutgoing :
                    triggeredByInput && g_rockConfig.rockKeepPreviousWeaponInHandOnEquip &&
                        !peerHoldingSameObject && currentEquippedWeaponOccupiesHand();
                if (!transition.heldTransfer().wantsEquip(equipIsLeft, heldRef ? heldRef->GetFormID() : 0u, equipHand.heldGrabIdentity())) {
                    if (!transition.beginHeldRequest({
                        .reference = heldRef ? heldRef->GetFormID() : 0u,
                        .grab = equipHand.heldGrabIdentity(), .world = context.worldGeneration,
                        .skeleton = context.skeletonGeneration, .isLeft = equipIsLeft,
                        .role = transferRole, .retainOutgoing = retainOutgoing,
                        .previousForm = previousEquippedWeaponFormID,
                        .previousInstance = reinterpret_cast<std::uintptr_t>(currentEquippedWeaponInstanceData(currentEquippedWeaponForm())),
                    })) return false;
                }
                if (!held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(player))) {
                    ROCK_LOG_SAMPLE_INFO(Weapon, 1000,
                        "Weapon transfer seq={} waiting for native readiness hand={} state={} heldUpdate=current",
                        transition.heldTransfer().sequence, equipIsLeft ? "left" : "right", f4vr::getNativeWeaponState(player));
                    // refreshTransferHand already advanced this exact held item
                    // once; do not integrate it a second time below.
                    return true;
                }
                if (retainOutgoing && !transition.heldTransfer().outgoingRemoved) {
                    const bool retainedIsLeft = !equipIsLeft;
                    const auto occupancy = _twoHandedGrip.getGripOccupancy();
                    const bool retainedHandCarries = retainedIsLeft ?
                        occupancy.left.carriesWeapon() : occupancy.right.carriesWeapon();
                    const bool receivingHandCarries = equipIsLeft ?
                        occupancy.left.carriesWeapon() : occupancy.right.carriesWeapon();
                    const auto sourceHand = retainedIsLeft ?
                        equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right;
                    if (!retainedHandCarries || receivingHandCarries ||
                        !_twoHandedGrip.requestEquippedWeaponDrop("trigger-equip-retention", sourceHand, frame.deltaSeconds)) {
                        ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                            "Trigger equip retained both weapons: previous={:08X} hand={} carry={} receivingCarry={} drop pose unavailable",
                            previousEquippedWeaponFormID, retainedIsLeft ? "left" : "right",
                            retainedHandCarries, receivingHandCarries);
                        transition.cancelHeldRequest("outgoing-carrier-unavailable");
                        return true;
                    }
                    const auto dropRequest = _twoHandedGrip.consumeEquippedWeaponDropRequest();
                    // Keep the old scene alive through cleanup of its grip authorities.
                    RE::NiPointer<RE::NiNode> transferSourceNode(resolveEquippedWeaponInteractionNode());
                    const bool dropped = dropEquippedWeaponToWorld(frame, dropRequest,
                        equipped_weapon_drop_policy::Mode::ToggleDrop);
                    _twoHandedGrip.completeEquippedWeaponDrop(dropRequest, dropped);
                    if (dropped) {
                        _equipped.transition.pendingGrip() = {};
                        clearEquippedWeaponFiringGripInputState();
                    }
                    const auto& retainedTransfer = _forceGrab.pendingCommits[retainedIsLeft ? 1u : 0u];
                    if (!dropped || !retainedTransfer.active ||
                        retainedTransfer.phase != PendingForceGrabCommitPhase::WaitingForNativePlacement) {
                        transition.cancelHeldRequest("outgoing-transfer-rejected");
                        return true;
                    }
                    ROCK_LOG_INFO(Weapon,
                        "Trigger equip keeping previous weapon in hand: previous={:08X} retainedHand={} incoming={:08X} equipHand={}",
                        previousEquippedWeaponFormID, retainedIsLeft ? "left" : "right",
                        heldRef ? heldRef->GetFormID() : 0u, equipIsLeft ? "left" : "right");
                    if (!held_weapon_equip_state_policy::canBeginEquip(f4vr::getNativeWeaponState(player))) {
                        // The accepted transfer owns this wait. Its original
                        // reference keeps updating; no short trigger replay owns it.
                        return true;
                    }
                }
                equipHand.stopSelectionHighlight();
                std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                const std::uint32_t primaryBodyId = equipHand.getSavedObjectState().bodyId.value;
                const auto supportBodyId = peerHoldingSameObject ? supportHand.getSavedObjectState().bodyId.value : 0;
                if (peerHoldingSameObject) {
                    auto supportRelease = makeGrabReleaseContext(supportHand, !equipIsLeft);
                    supportRelease.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
                    supportRelease.reason = "paired-held-weapon-equip";
                    supportHand.stopSelectionHighlight();
                    const auto supportOutcome = supportHand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, supportRelease);
                    if (!supportOutcome.released) {
                        ROCK_LOG_ERROR(Hand, "Paired held equip blocked: support hand did not release ref={:08X} hand={} state={}",
                            heldFormID, supportHand.handName(), static_cast<unsigned>(supportHand.getState()));
                        transition.cancelHeldRequest("paired-support-release-failed");
                        return true;
                    }
                    releaseObject(heldRef, claimOwnerForHand(!equipIsLeft));
                    input_remap_runtime::setHandHeldWeapon(!equipIsLeft, false);
                    clearGameplayCandidatesForHand(supportHand, !equipIsLeft);
                }
                // A shared hold must release its constraint and pose before
                // selection cleanup or native inventory transfer can retire it.
                if (heldRef && supportHand.hasSelection() && supportHand.getSelection().refr == heldRef) {
                    supportHand.clearSelectionState(false);
                }
                auto releaseContext = makeGrabReleaseContext(equipHand, equipIsLeft);
                releaseContext.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
                releaseContext.reason = requestReason ? requestReason : "held-weapon-equip";
                auto releaseOutcome = equipHand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                if (!releaseOutcome.released) {
                    transition.cancelHeldRequest("incoming-constraint-release-failed");
                    return true;
                }
                if (heldRef) {
                    releaseObject(heldRef, claimOwnerForHand(equipIsLeft));
                }

                const auto equipResult = weapon_equip_transfer::transferHeldWeaponToPlayerAndEquip(weapon_equip_transfer::EquipInput{
                    .heldRef = releaseOutcome.takeRetainedReference(),
                    .transitionReason = transitionReason,
                });
                if (equipResult.transferredToInventory) {
                    transition.recordInventoryCommit(equipResult.weapon ? equipResult.weapon->formID : 0u,
                        equipResult.requestedInstanceData, equipResult.success, equipResult.observedEquippedInstanceData);
                }
                if (!equipResult.success) transition.cancelHeldRequest("incoming-native-equip-failed");
                const std::uint32_t nativeStateAfterEquip =
                    f4vr::getNativeWeaponState(player);
                const auto immediateVisual = equipResult.committed && equipResult.weapon ?
                    equipped_weapon_visual_state::observe(
                        equipResult.weapon->formID,
                        reinterpret_cast<std::uintptr_t>(previousNativeInstanceNode)) :
                    equipped_weapon_visual_state::Snapshot{};
                bool equipBridgeStarted = false;
                if (equipResult.success) {
                    const auto transitionSource = triggeredByInput ?
                        EquippedWeaponTransitionCoordinator::Source::HeldTriggerEquip :
                        EquippedWeaponTransitionCoordinator::Source::HeldGripZoneEquip;
                    equipBridgeStarted = _equipped.transition.beginHeldTransition(
                        EquippedWeaponTransitionCoordinator::ExpectedIdentity{
                            .formID = equipResult.weapon ? equipResult.weapon->formID : equipResult.observedEquippedFormID,
                            .instanceData = equipResult.requestedInstanceData,
                            .previousFormID = equipResult.previousEquippedFormID,
                            .previousInstanceData = equipResult.previousEquippedInstanceData,
                            .previousNativeInstanceNode =
                                reinterpret_cast<std::uintptr_t>(
                                    previousNativeInstanceNode),
                        },
                        transitionSource,
                        EquipVisualBridge::BeginInput{
                        .worldModel = equipResult.detachedWorldModel,
                        .weaponFormID = equipResult.weapon ? equipResult.weapon->formID : equipResult.observedEquippedFormID,
                        .isLeftHand = equipIsLeft,
                        .pairedGrips = pairedGrips.valid() ? &pairedGrips : nullptr,
                        .singleGrip = singleGrip.valid() ? &singleGrip : nullptr,
                        .supportOnly = supportEquipRequested,
                        .weapon = equipResult.weapon,
                        .hasFiringHandWeaponLocal = pendingGripStart.hasFiringHandWeaponLocal,
                        .firingHandWeaponLocal = pendingGripStart.firingHandWeaponLocal,
                        .timeoutSeconds = _equipped.handlingSettings.equipVisualBridgeTimeoutSeconds,
                        .blendSeconds = _equipped.handlingSettings.equipVisualBridgeBlendSeconds,
                    });
                    if (equipBridgeStarted && _equipped.transition.hasCapturedHandPoseHandoff()) {
                        equipHand.cancelGrabVisualReturn("equip-pose-handoff");
                        if (pairedGrips.valid()) supportHand.cancelGrabVisualReturn("paired-equip-pose-handoff");
                    }
                }
                if (heldFormID == 0 && equipResult.formID != 0) {
                    heldFormID = equipResult.formID;
                }

                auto* postEquipRef = equipResult.untransferredRef.get();
                dispatchPhysicsMessage(kPhysMsg_OnRelease, equipIsLeft, postEquipRef, heldFormID, 0);
                if (!equipResult.success && !equipResult.transferredToInventory) {
                    equipHand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                }
                dispatchHeldObjectEventByFormID(GrabEventType::Released, postEquipRef, heldFormID, primaryBodyId, equipIsLeft);
                if (peerHoldingSameObject) {
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, !equipIsLeft, postEquipRef, heldFormID, 0);
                    dispatchHeldObjectEventByFormID(GrabEventType::Released, postEquipRef, heldFormID, supportBodyId, !equipIsLeft);
                }
                const bool physicalCarryPendingArmed =
                    equipResult.success && pendingGripStart.pending;
                if (physicalCarryPendingArmed) {
                    pendingGripStart.targetWeaponFormID = equipResult.weapon ?
                        equipResult.weapon->formID :
                        equipResult.observedEquippedFormID;
                    pendingGripStart.targetWeaponInstanceData =
                        equipResult.requestedInstanceData;
                    pendingGripStart.previousWeaponFormID =
                        equipResult.previousEquippedFormID;
                    pendingGripStart.previousWeaponInstanceData =
                        equipResult.previousEquippedInstanceData;
                    pendingGripStart.remainingSeconds = 10.0f;
                    pendingGripStart.source =
                        equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::HeldWeaponEquip;
                    // A solo left carry reacquires its canonical seat. A paired
                    // transfer retains both visual seats; its native aim frame
                    // remains separately owned by the equipped carry solver.
                    if (equipIsLeft && !pairedGrips.valid() && !supportEquipRequested) {
                        pendingGripStart.hasFiringHandWeaponLocal = false;
                        pendingGripStart.hasFiringGripWeaponLocal = false;
                    }
                    _equipped.transition.pendingGrip() = pendingGripStart;
                } else if (equipResult.success && !equipIsLeft && !supportEquipRequested) {
                    transition.recordGripAcquired(equipResult.weapon->formID, equipResult.observedEquippedInstanceData,
                        false, held_weapon_transfer::Role::Firing);
                }
                const auto& actionTrace = equipResult.instantTransition.actionTrace;
                ROCK_LOG_INFO(Hand,
                    "{} hand {} held weapon equip formID={:08X} weapon='{}' success={} managerAccepted={} committed={} equippedStackMatch={} equipReason={} requestReason={} transition={} readiness={} count={} stack={} stackEvidence={} stacks={}->{} mutations={} instanceMatch={} requestedInstance={:#x} observedInstance={:#x} transferred={} observedEquipped={:08X} equipIndex={} weaponState={}({})->{}({}) traceCount={} traceSheathe={} traceDraw={} traceFaults=0x{:02X} nativeInstance={} nativeAncestorsVisible={} nativeLocalVisible={} immediateEquip={} visualBridge={} physicalCarryPending={} physicalCarryHand={}",
                    equipHand.handName(),
                    logAction ? logAction : "requested",
                    heldFormID,
                    equipResult.weapon ? RE::TESFullName::GetFullName(*equipResult.weapon, false) : "unknown",
                    equipResult.success ? "yes" : "no",
                    equipResult.instantTransition.managerAccepted ? "yes" : "no",
                    equipResult.committed ? "yes" : "no",
                    equipResult.matchedEquippedStack ? "yes" : "no",
                    weapon_equip_transfer::equipReasonName(equipResult.reason),
                    held_weapon_instant_transition::requestReasonName(
                        transitionReason),
                    held_weapon_instant_transition::immediateEquipCodeName(
                        equipResult.instantTransition.code),
                    held_weapon_instant_transition::readinessReasonName(
                        instantReadiness.reason),
                    equipResult.count,
                    equipResult.stackID,
                    weapon_inventory_stack_selection_policy::evidenceName(
                        equipResult.stackSelectionEvidence),
                    equipResult.preTransferStackCount,
                    equipResult.postTransferStackCount,
                    equipResult.stackMutationCandidateCount,
                    equipResult.matchedInstanceData ? "yes" : "no",
                    equipResult.requestedInstanceData,
                    equipResult.observedEquippedInstanceData,
                    equipResult.transferredToInventory ? "yes" : "no",
                    equipResult.observedEquippedFormID,
                    equipResult.observedEquipIndex,
                    nativeStateBeforeEquip,
                    held_weapon_equip_state_policy::nativeWeaponStateName(nativeStateBeforeEquip),
                    nativeStateAfterEquip,
                    held_weapon_equip_state_policy::nativeWeaponStateName(nativeStateAfterEquip),
                    actionTrace.count,
                    held_weapon_instant_transition_policy::actionCount(
                        actionTrace,
                        held_weapon_instant_transition_policy::NativeAction::Sheathe),
                    held_weapon_instant_transition_policy::actionCount(
                        actionTrace,
                        held_weapon_instant_transition_policy::NativeAction::Draw),
                    static_cast<unsigned int>(
                        (actionTrace.playerMismatch ? 0x01u : 0u) |
                        (actionTrace.unexpectedCaller ? 0x02u : 0u) |
                        (actionTrace.nestedScope ? 0x04u : 0u) |
                        (actionTrace.overflow ? 0x08u : 0u)),
                    immediateVisual.exactInstance ? "yes" : "no",
                    immediateVisual.ancestorPathVisible ? "yes" : "no",
                    immediateVisual.instanceLocallyVisible ? "yes" : "no",
                    equipResult.usedImmediateEquip ? "yes" : "no",
                    equipBridgeStarted ? "yes" : "no",
                    physicalCarryPendingArmed ? "yes" : "no",
                    physicalCarryPendingArmed ? (equipIsLeft ? "left" : "right") : "none");
                input_remap_runtime::setHandHeldWeapon(equipIsLeft, false);
                clearGameplayCandidatesForHand(equipHand, equipIsLeft);
                return true;
            };

            if (heldLooseGrenade) {
                if (heldWeaponEquipTriggerPressed) {
                    static_cast<void>(armHeldLooseGrenade(hand, frame));
                }
            } else if (heldWeaponEquipRequested) {
                const bool triggeredByInput = heldWeaponEquipTriggerPressed;
                const char* requestReason = triggeredByInput ? "same-hand-trigger-held-weapon-equip" :
                                                               "grip-zone-held-weapon-equip";
                const char* logAction = triggeredByInput ? "trigger" : "grip-zone";
                if (equipHeldWeaponFromHand(triggeredByInput, requestReason, logAction)) {
                    return;
                }
            }

            const bool injectionMode = g_rockConfig.rockImmersiveAidEnabled &&
                immersive_aid::classify(heldRefForGameplay ? heldRefForGameplay->GetObjectReference() : nullptr) != immersive_aid::Injector::None;
            const auto consumeEligibility = mouth_consume::evaluateEligibility(mouth_consume::EligibilityInput{
                .enabled = injectionMode || g_rockConfig.rockMouthConsumeEnabled,
                .allowPoison = g_rockConfig.rockMouthConsumeAllowPoison,
                .peerHoldingSameObject = peerHoldingSameObject,
                .heldRef = heldRefForGameplay,
                .savedState = &hand.getSavedObjectState(),
            });

            mouth_consume::Decision consumeDecision{};
            if (consumeEligibility.eligible && injectionMode) {
                consumeDecision = immersive_aid::evaluate(hknp, _bodyBoneColliders, hand, isLeft,
                    _lifecycle.collisionGenerationAtomic.load(std::memory_order_acquire), frame.timing, mouthConsumeState);
            } else if (consumeEligibility.eligible) {
                if (mouthConsumeState.injection.grabIdentity != 0) {
                    clearMouthConsumeForHand(hand, isLeft);
                }
                consumeDecision = mouth_consume::evaluate(mouth_consume::DetectorInput{
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .objectProbe = makeMouthConsumeObjectProbe(hknp, hand, handInput),
                        .hasObjectProbe = true,
                        .handProbe = makeMouthConsumeHandProbe(handInput),
                        .hasHandProbe = true,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = makeMouthConsumeDetectorConfig(),
                    },
                    mouthConsumeState);
            } else {
                clearMouthConsumeForHand(hand, isLeft);
            }

            bool consumeCandidateActive = false;
            if (consumeEligibility.eligible && consumeDecision.candidate) {
                if (hand.getState() == HandState::StashCandidate) {
                    hand.cancelStashCandidate();
                }
                if (hand.getState() == HandState::HeldBody) {
                    hand.beginConsumeCandidate();
                }
                if (hand.getState() == HandState::ConsumeCandidate) {
                    consumeCandidateActive = true;
                    const bool pulseDue = _contacts.dynamicPushElapsedSeconds >= mouthConsumeState.nextCandidatePulseTimeSeconds;
                    if (consumeDecision.enteredCandidate || consumeDecision.changedCandidate || pulseDue) {
                        dispatchMouthConsumeEvent(
                            GrabEventType::ConsumeCandidate,
                            heldRefForGameplay,
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
                            hand.getSavedObjectState().bodyId.value,
                            consumeDecision);
                        mouthConsumeState.nextCandidatePulseTimeSeconds =
                            _contacts.dynamicPushElapsedSeconds + (std::max)(0.02f, g_rockConfig.rockMouthConsumeCandidateHapticIntervalSeconds);
                    }
                }
            } else {
                hand.cancelConsumeCandidate();
            }

            const auto stashEligibility = !consumeCandidateActive ?
                shoulder_stash::evaluateEligibility(shoulder_stash::EligibilityInput{
                    .enabled = g_rockConfig.rockShoulderStashEnabled,
                    .peerHoldingSameObject = peerHoldingSameObject,
                    .heldRef = heldRefForGameplay,
                    .savedState = &hand.getSavedObjectState(),
                }) :
                shoulder_stash::EligibilityResult{ .eligible = false, .reason = shoulder_stash::EligibilityReason::SharedHeldObject };

            shoulder_stash::Decision stashDecision{};
            if (!consumeCandidateActive && stashEligibility.eligible) {
                stashDecision = shoulder_stash::evaluate(shoulder_stash::DetectorInput{
                        .world = hknp,
                        .bodyColliders = &_bodyBoneColliders,
                        .bodyContacts = &_contacts.bodyRuntime,
                        .heldBodyIds = &hand.getHeldBodyIds(),
                        .contactElapsedSeconds = _contacts.handActivity.currentElapsedSeconds(),
                        .isLeftHand = isLeft,
                        .probe = makeShoulderStashObjectProbe(hknp, hand, handInput),
                        .hmdProbe = makeShoulderStashHmdProbe(handInput),
                        .hasHmdProbe = true,
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = makeShoulderStashDetectorConfig(),
                    },
                    shoulderStashState);
            } else {
                clearShoulderStashForHand(hand, isLeft);
            }

            if (!consumeCandidateActive && stashEligibility.eligible && stashDecision.candidate) {
                if (hand.getState() == HandState::HeldBody) {
                    hand.beginStashCandidate();
                }
                if (hand.getState() == HandState::StashCandidate) {
                    const bool pulseDue = _contacts.dynamicPushElapsedSeconds >= shoulderStashState.nextCandidatePulseTimeSeconds;
                    if (stashDecision.enteredCandidate || stashDecision.changedCandidate || pulseDue) {
                        dispatchShoulderStashEvent(
                            GrabEventType::StashCandidate,
                            heldRefForGameplay,
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
                            hand.getSavedObjectState().bodyId.value,
                            stashDecision);
                        shoulderStashState.nextCandidatePulseTimeSeconds =
                            _contacts.dynamicPushElapsedSeconds + (std::max)(0.02f, g_rockConfig.rockShoulderStashCandidateHapticIntervalSeconds);
                    }
                }
            } else {
                hand.cancelStashCandidate();
            }

            const bool injectionCommit = injectionMode && consumeEligibility.eligible &&
                consumeDecision.confirmedForCommit && hand.getState() == HandState::ConsumeCandidate;
            if (grabInput.released || injectionCommit) {
                triggerEquipIntent = {};
                ROCK_LOG_INFO(Hand,
                    "Held release input: hand={} grab={} physical=({},{},{}) logical=({},{},{}) injection={}",
                    isLeft ? "left" : "right", hand.heldGrabIdentity(),
                    rawGrabInput.held, rawGrabInput.pressed, rawGrabInput.released,
                    grabInput.held, grabInput.pressed, grabInput.released, injectionCommit);
                hand.captureHeldReleaseMotion(hknp, handInput.rawHandWorld, frame.timing);
                auto* heldRef = hand.getHeldRef();
                std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                if (consumeEligibility.eligible && consumeDecision.confirmedForCommit && hand.getState() == HandState::ConsumeCandidate) {
                    /*
                     * Mouth release and automatic injection share the two-phase transfer:
                     * detach the grab without throw velocity first, then let the
                     * native consume/activation path take ownership. Only failures
                     * that leave a world ref behind get the captured throw velocity.
                     */
                    auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                    releaseContext.disposition = GrabReleaseDisposition::PendingConsumeTransfer;
                    releaseContext.reason = injectionMode ? "aid-injection-pending-transfer" : "mouth-consume-pending-transfer";
                    const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                    auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                    if (heldRef) {
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                    }
                    const auto consumeResult = mouth_consume::transferToPlayerConsume(mouth_consume::ConsumeInput{
                        .heldRef = releaseOutcome.takeRetainedReference(),
                        .allowPoison = g_rockConfig.rockMouthConsumeAllowPoison,
                    });
                    if (heldFormID == 0 && consumeResult.formID != 0) {
                        heldFormID = consumeResult.formID;
                    }

                    const bool failedBeforeOwnershipTransfer =
                        !consumeResult.attempted || consumeResult.reason == mouth_consume::ConsumeReason::ActivateRefFailed;
                    auto* postConsumeRef = consumeResult.untransferredRef.get();
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, postConsumeRef, heldFormID, 0);
                    if (consumeResult.success) {
                        dispatchMouthConsumeEvent(GrabEventType::Consumed, nullptr, heldFormID, primaryBodyId, consumeDecision);
                    } else {
                        if (failedBeforeOwnershipTransfer) {
                            hand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                        }
                        dispatchHeldObjectEventByFormID(GrabEventType::Released, postConsumeRef, heldFormID, primaryBodyId, isLeft);
                    }
                    ROCK_LOG_INFO(Hand,
                        "{} hand {} formID={:08X} success={} consumeReason={} count={} confidence={:.2f} distance={:.1f} speed={:.1f}",
                        hand.handName(),
                        injectionMode ? "aid injection" : "mouth consume release",
                        heldFormID,
                        consumeResult.success ? "yes" : "no",
                        mouth_consume::consumeReasonName(consumeResult.reason),
                        consumeResult.count,
                        consumeDecision.confidence,
                        consumeDecision.distanceGameUnits,
                        consumeDecision.speedGameUnitsPerSecond);
                    clearGameplayCandidatesForHand(hand, isLeft);
                    return;
                }
                if (stashEligibility.eligible && stashDecision.confirmedForCommit && hand.getState() == HandState::StashCandidate) {
                    /*
                     * Shoulder stash uses a two-phase release because native
                     * pickup can still refuse the reference. ROCK detaches the
                     * grab as a pending transfer, captures the physical release
                     * velocity, then applies that velocity only when transfer
                     * fails so successful stash stays quiet and failed stash is
                     * an honest drop.
                     */
                    auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                    releaseContext.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
                    releaseContext.reason = "shoulder-stash-pending-transfer";
                    const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                    auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                    if (heldRef) {
                        releaseObject(heldRef, claimOwnerForHand(isLeft));
                    }
                    const auto transferResult = shoulder_stash::transferToPlayerInventory(shoulder_stash::TransferInput{
                        .heldRef = releaseOutcome.takeRetainedReference(),
                    });
                    if (heldFormID == 0 && transferResult.formID != 0) {
                        heldFormID = transferResult.formID;
                    }

                    auto* postTransferRef = transferResult.untransferredRef.get();
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, postTransferRef, heldFormID, 0);
                    if (transferResult.success) {
                        dispatchShoulderStashEvent(GrabEventType::Stashed, nullptr, heldFormID, primaryBodyId, stashDecision);
                        showShoulderStashCollectedNotification(transferResult, heldFormID);
                    } else {
                        hand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                        dispatchHeldObjectEventByFormID(GrabEventType::Released, postTransferRef, heldFormID, primaryBodyId, isLeft);
                    }
                    ROCK_LOG_INFO(Hand,
                        "{} hand shoulder stash release formID={:08X} success={} transferReason={} stashSource={} zone={} count={} confidence={:.2f} speed={:.1f}",
                        hand.handName(),
                        heldFormID,
                        transferResult.success ? "yes" : "no",
                        shoulder_stash::transferReasonName(transferResult.reason),
                        shoulder_stash::evidenceSourceName(stashDecision.source),
                        body_zone::bodyZoneName(stashDecision.zone),
                        transferResult.count,
                        stashDecision.confidence,
                        stashDecision.speedGameUnitsPerSecond);
                    clearGameplayCandidatesForHand(hand, isLeft);
                    return;
                }
                hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(hand, isLeft));
                if (heldRef)
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                clearGameplayCandidatesForHand(hand, isLeft);
            } else {
                const auto& transform = handInput.rawHandWorld;
                auto* heldRef = hand.getHeldRef();
                auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                logPalmClockSampleForHand("game-before-held-update",
                    hand,
                    hknp,
                    &transform,
                    _frame.palmClockGameFrameIndex.load(std::memory_order_acquire),
                    _frame.palmClockGameDeltaSeconds.load(std::memory_order_acquire),
                    nullptr);
                hand.updateHeldObject(hknp,
                    transform,
                    frame.deltaSeconds,
                    g_rockConfig.rockGrabForceFadeInTime,
                    g_rockConfig.rockGrabTauMin,
                    &_bodyBoneColliders,
                    makeGrabReleaseContext(hand, isLeft),
                    isLeft ? &_rightHand : &_leftHand,
                    &(isLeft ? frame.right : frame.left).rawHandWorld);
                if (triggerEquipIntent.pending) {
                    // Existing opt-in, bounded transfer telemetry shows the
                    // live hand/model relationship during native readiness waits.
                    const auto* currentHeldRef = hand.getHeldRef();
                    vanilla_weapon_alignment_telemetry::recordTransferTrace(
                        vanilla_weapon_alignment_telemetry::TransferKind::HeldEquip,
                        isLeft, "equip-wait-held-update", currentHeldRef ? currentHeldRef->Get3D() : nullptr);
                }
                if (heldRef && !hand.isHolding()) {
                    triggerEquipIntent = {};
                    ROCK_LOG_WARN(Hand, "Held update ended grab without an input release: hand={} ref={:08X}",
                        isLeft ? "left" : "right", heldFormID);
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                    dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
                    dispatchSimpleGrabEvent(GrabEventType::Released, isLeft, heldRef);
                    clearGameplayCandidatesForHand(hand, isLeft);
                }
            }
        } else {
            clearGameplayCandidatesForHand(hand, isLeft);
            if (grabInput.pressed && !peerHeldRetryAttemptDue) {
                const char* refusalReason = "not-attempted";
                (void)attemptPeerHeldCloseJoinSelection(&refusalReason);
            }
        }

        if (!hand.isHolding() && selection_state_policy::canProcessSelectedState(hand.getState()) && hand.hasSelection()) {
            const bool pullCatchCommitPending = hand.hasPendingPullCatchCommit();
            auto* pullCatchRef = pullCatchCommitPending ? hand.getPullCatchIntentRef() : nullptr;
            bool actorEquipmentDropHandoffReady = false;
            if (pullCatchCommitPending) {
                if (grabInput.released || !grabInput.held) {
                    ROCK_LOG_DEBUG(Hand, "{} hand cancelled pull catch commit because grip was released", hand.handName());
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-release");
                    hand.clearSelectionState(true);
                    releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                    return;
                }
                if (!hand.advancePullCatchCommit(frame.deltaSeconds, pull_motion_math::kCatchRetryMaximumTimeSeconds)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand cancelled pull catch commit because retry window expired ({:.3f}s)",
                        hand.handName(),
                        pull_motion_math::kCatchRetryMaximumTimeSeconds);
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-retry-expired");
                    hand.clearSelectionState(true);
                    releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                    return;
                }
            }

            if (hand.hasPendingActorEquipmentDropHandoff()) {
                if (grabInput.released || !grabInput.held) {
                    ROCK_LOG_DEBUG(Hand, "{} hand cancelled actor-equipment drop handoff because grip was released", hand.handName());
                    hand.clearSelectionState(true);
                    return;
                }

                const auto handoffStatus = hand.advanceActorEquipmentDropHandoff(
                    frame.bhkWorld,
                    hknp,
                    frame.deltaSeconds,
                    pull_motion_math::kActorEquipmentHandoffMaximumTimeSeconds);
                switch (handoffStatus) {
                case Hand::ActorEquipmentDropHandoffStatus::Ready:
                    actorEquipmentDropHandoffReady = true;
                    break;
                case Hand::ActorEquipmentDropHandoffStatus::Pending:
                    return;
                case Hand::ActorEquipmentDropHandoffStatus::None:
                    break;
                case Hand::ActorEquipmentDropHandoffStatus::InvalidSelection:
                case Hand::ActorEquipmentDropHandoffStatus::MissingDroppedReference:
                case Hand::ActorEquipmentDropHandoffStatus::TimedOut:
                default:
                    ROCK_LOG_WARN(Hand,
                        "{} hand actor-equipment drop handoff failed status={} actorSelection={:08X}",
                        hand.handName(),
                        static_cast<int>(handoffStatus),
                        hand.getSelection().refr ? hand.getSelection().refr->GetFormID() : 0);
                    hand.clearSelectionState(true);
                    return;
                }
            }

            if (grabInput.pressed || peerHeldRetryCommitIntent || (pullCatchCommitPending && grabInput.held) || (actorEquipmentDropHandoffReady && grabInput.held)) {
                if (!grab_interaction_policy::canAttemptSelectedObjectGrab(
                        hand.getSelection().isFarSelection, hand.getSelection().distance, selection_query_policy::kFarDetectionRangeGameUnits)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand: far grab blocked (dist={:.1f}, configuredFarRange={:.1f})",
                        hand.handName(),
                        hand.getSelection().distance,
                        selection_query_policy::kFarDetectionRangeGameUnits);
                    return;
                }

                if (hand.getSelection().isFarSelection) {
                    float hmdConeDot = -1.0f;
                    if (!selectedObjectPassesFarHmdCone(hknp, hand.getSelection(), farHmdConeGate, &hmdConeDot)) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand: far grab blocked outside HMD cone formID={:08X} hmdDot={:.3f} minDot={:.3f}",
                            hand.handName(),
                            hand.getSelection().refr ? hand.getSelection().refr->GetFormID() : 0,
                            hmdConeDot,
                            farHmdConeGate.minDot);
                        hand.clearSelectionState(true);
                        return;
                    }
                }

                if (!pullCatchCommitPending &&
                    hand.getSelection().isFarSelection &&
                    hand.getSelection().targetKind == grab_target::Kind::ActorEquipment) {
                    const auto actorSelection = hand.getSelection();
                    const auto dropResult = actor_equipment_grab::dropFarActorEquipmentSelection(
                        actorSelection.refr,
                        actorSelection.actorEquipment,
                        actor_equipment_grab::kDefaultAttachedDropZOffsetGameUnits);
                    if (dropResult.status != actor_equipment_grab::DropStatus::Success || !dropResult.droppedRef) {
                        ROCK_LOG_WARN(Hand,
                            "{} hand actor-equipment far pull failed before drop handoff: status={} actor={:08X} item={:08X}",
                            hand.handName(),
                            actor_equipment_grab::dropStatusName(dropResult.status),
                            dropResult.actorFormId,
                            dropResult.itemFormId);
                        hand.clearSelectionState(true);
                        return;
                    }

                    if (!hand.beginActorEquipmentDropHandoff(
                            dropResult,
                            actorSelection.hasHitPoint ? actorSelection.hitPointWorld : actorSelection.actorEquipment.hitPointWorld)) {
                        ROCK_LOG_WARN(Hand,
                            "{} hand actor-equipment far pull failed to arm drop handoff: dropped={:08X} actor={:08X} item={:08X}",
                            hand.handName(),
                            dropResult.droppedFormId,
                            dropResult.actorFormId,
                            dropResult.itemFormId);
                        hand.clearSelectionState(true);
                        return;
                    }

                    const auto handoffStatus = hand.advanceActorEquipmentDropHandoff(
                        frame.bhkWorld,
                        hknp,
                        0.0f,
                        pull_motion_math::kActorEquipmentHandoffMaximumTimeSeconds);
                    if (handoffStatus == Hand::ActorEquipmentDropHandoffStatus::Ready) {
                        actorEquipmentDropHandoffReady = true;
                    } else if (handoffStatus == Hand::ActorEquipmentDropHandoffStatus::Pending) {
                        return;
                    } else {
                        ROCK_LOG_WARN(Hand,
                            "{} hand actor-equipment far pull failed after arming handoff: status={} dropped={:08X} actor={:08X} item={:08X}",
                            hand.handName(),
                            static_cast<int>(handoffStatus),
                            dropResult.droppedFormId,
                            dropResult.actorFormId,
                            dropResult.itemFormId);
                        hand.clearSelectionState(true);
                        return;
                    }
                }

                if (selectedObjectInteractionBlocked()) {
                    if (pullCatchCommitPending) {
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-blocked");
                        hand.clearSelectionState(true);
                        releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                    }
                    return;
                }

                if (hand.getSelection().isFarSelection) {
                    if (pullCatchCommitPending) {
                        ROCK_LOG_WARN(Hand,
                            "{} hand cancelled pull catch commit because pending catch unexpectedly resolved to far selection",
                            hand.handName());
                        hand.finishPullPrepAsPhysicalDropIfActive("pull-catch-far-selection");
                        hand.clearSelectionState(true);
                        releaseObject(pullCatchRef, claimOwnerForHand(isLeft));
                        return;
                    }
                    auto* selectedRef = hand.getSelection().refr;
                    const auto selectedBodyId = hand.getSelection().bodyId.value;
                    const auto& transform = handInput.rawHandWorld;
                    /*
                     * Far-pull startup publishes the lock before dynamic body conversion because
                     * startDynamicPull owns failure cleanup and may clear selection internally.
                     * Deferring SelectionLocked fixed haptic overwrite but exposed impossible
                     * lock/unlock ordering to API consumers, so the event stays ordered and only
                     * the selection haptic is suppressed on this pull-start path.
                     */
                    const bool lockedSelection = hand.lockFarSelection();
                    if (lockedSelection) {
                        dispatchSimpleGrabEvent(
                            GrabEventType::SelectionLocked,
                            isLeft,
                            selectedRef,
                            selectedBodyId,
                            ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC);
                    }
                    const bool pullStarted = lockedSelection && hand.startDynamicPull(hknp, transform);
                    if (pullStarted) {
                        claimObject(selectedRef, claimOwnerForHand(isLeft));
                        dispatchSimpleGrabEvent(GrabEventType::PullStarted, isLeft, selectedRef, selectedBodyId);
                    } else {
                        if (lockedSelection) {
                            dispatchSimpleGrabEvent(GrabEventType::SelectionUnlocked, isLeft, selectedRef, selectedBodyId);
                        }
                        releaseObject(selectedRef, claimOwnerForHand(isLeft));
                    }
                    return;
                }

                if (pullCatchCommitPending) {
                    dispatchSimpleGrabEvent(GrabEventType::PullCatchAttempt, isLeft, pullCatchRef, hand.getSelection().bodyId.value);
                }
                const bool peerHeldRetryWasActiveForCommit =
                    peerHeldJoinRetryState.active &&
                    rawGrabInput.held &&
                    !hand.isHolding() &&
                    hand.hasSelection() &&
                    !hand.getSelection().isFarSelection &&
                    hand.getSelection().refr == peerHeldRefForInput;
                const bool grabbed = attemptSelectedGrab();
                if (grabbed && peerHeldRetryWasActiveForCommit) {
                    const auto peerFormId = peerHeldJoinRetryState.peerFormId;
                    const auto attempts = peerHeldJoinRetryState.attempts;
                    peer_held_join_retry_policy::reset(peerHeldJoinRetryState);
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand peer-held join retry succeeded: peerFormID={:08X} attempts={}",
                        hand.handName(),
                        peerFormId,
                        attempts);
                } else if (!grabbed && peerHeldRetryWasActiveForCommit && peerHeldJoinRetryState.active) {
                    peerHeldJoinRetryState.lastRefusalReason = "grab-commit-refused";
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand retaining peer-held join retry after grab commit refused; grip still held",
                        hand.handName());
                }
                if (!grabbed && pullCatchCommitPending) {
                    hand.notePullCatchCommitAttemptFailed();
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand retaining pull catch commit after grab attempt failed; grip still held",
                        hand.handName());
                }
            }
        } else if (hand.getState() == HandState::SelectionLocked) {
            if (grabInput.released) {
                auto* selectedRef = hand.getSelection().refr;
                ROCK_LOG_DEBUG(Hand, "{} hand released locked far selection", hand.handName());
                dispatchSimpleGrabEvent(GrabEventType::SelectionUnlocked, isLeft, selectedRef, hand.getSelection().bodyId.value);
                hand.clearSelectionState(true);
                releaseObject(selectedRef, claimOwnerForHand(isLeft));
            }
        } else if (hand.getState() == HandState::Pulled) {
            auto* pulledRef = hand.getSelection().refr;
            if (grabInput.released) {
                ROCK_LOG_DEBUG(Hand, "{} hand released dynamic pull", hand.handName());
                hand.finishPullPrepAsPhysicalDropIfActive("pull-release");
                hand.clearSelectionState(true);
                releaseObject(pulledRef, claimOwnerForHand(isLeft));
                return;
            }

            const auto& transform = handInput.rawHandWorld;
            const bool readyToGrab = hand.updateDynamicPull(hknp, transform, frame.deltaSeconds);
            if (!hand.hasSelection() || hand.getState() == HandState::Idle) {
                releaseObject(pulledRef, claimOwnerForHand(isLeft));
                return;
            }

            if (readyToGrab) {
                dispatchSimpleGrabEvent(GrabEventType::PullArrived, isLeft, pulledRef, hand.getSelection().bodyId.value);
                if (selectedObjectInteractionBlocked()) {
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-arrived-blocked");
                    hand.clearSelectionState(true);
                    releaseObject(pulledRef, claimOwnerForHand(isLeft));
                    return;
                }

                dispatchSimpleGrabEvent(GrabEventType::PullCatchAttempt, isLeft, pulledRef, hand.getSelection().bodyId.value);
                const bool grabbed = attemptSelectedGrab();
                if (!grabbed && (!hand.hasSelection() || !hand.hasPendingPullCatchCommit())) {
                    hand.finishPullPrepAsPhysicalDropIfActive("pull-grab-refused");
                    hand.clearSelectionState(true);
                    releaseObject(pulledRef, claimOwnerForHand(isLeft));
                } else if (!grabbed) {
                    hand.notePullCatchCommitAttemptFailed();
                    ROCK_LOG_SAMPLE_DEBUG(Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand pull arrived but grab commit did not accept yet; retaining catch intent while grip is held",
                        hand.handName());
                }
            }
        }


    }

    void PhysicsInteraction::processGrabInputHand(
        const PhysicsFrameContext& frame,
        Hand& hand,
        bool isLeft,
        const GrabInputHandContext& context)
    {
        GrabInputHandPrelude prelude{};
        if (!prepareGrabInputHand(frame, hand, isLeft, context, prelude)) {
            return;
        }

        if (!processTouchGrabInput(frame, hand, isLeft, context, prelude)) {
            return;
        }
        processGrabIntentAndCommit(frame, hand, isLeft, context, prelude);
    }

    void PhysicsInteraction::updateGrabInput(const PhysicsFrameContext& frame)
    {
        _forceGrab.committedThisFrame = {};

        /*
         * Loose-hold solves mirror from the physical right hand. Publish the
         * weapon-authority frame before any hand can commit a grab this frame
         * so a support lock or part carry on RArm_Hand never leaks ROCK's
         * presented bone into a loose weapon placement.
         */
        {
            loose_weapon_grip_zone::CanonicalPrimaryHandFrame canonicalPrimaryHand{};
            canonicalPrimaryHand.valid =
                _twoHandedGrip.tryGetPhysicalHandWorld(false, canonicalPrimaryHand.handWorld);
            canonicalPrimaryHand.presentedByRock =
                _twoHandedGrip.hasVisualAuthorityForHand(false);
            loose_weapon_grip_zone::publishCanonicalPrimaryHandFrame(canonicalPrimaryHand);
            loose_weapon_grip_zone::CanonicalPrimaryHandFrame physicalLeftHand{};
            physicalLeftHand.valid = _twoHandedGrip.tryGetPhysicalHandWorld(true, physicalLeftHand.handWorld);
            physicalLeftHand.presentedByRock = _twoHandedGrip.hasVisualAuthorityForHand(true);
            loose_weapon_grip_zone::publishPhysicalLeftHandFrame(physicalLeftHand);
        }

        if (!runtime_state::isLocalSkeletonReady()) {
            _touchGrabRuntime.releaseAll(
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                _lifecycle.collisionGenerationAtomic.load(
                    std::memory_order_acquire));
            _rightHand.cancelGrabVisualReturn("skeleton-not-ready");
            _leftHand.cancelGrabVisualReturn("skeleton-not-ready");
            provider::clearInteractionCommandsForProviderLossV1(provider::RockProviderInteractionFailureV1::ProviderNotReady);
            clearPendingForceGrabCommits();
            input_remap_runtime::setHandHeldWeapon(false, false);
            input_remap_runtime::setHandHeldWeapon(true, false);
            input_remap_runtime::setHandInteractionEngaged(false, false);
            input_remap_runtime::setHandInteractionEngaged(true, false);
            input_remap_runtime::setHeldObjectFormId(false, 0u);
            input_remap_runtime::setHeldObjectFormId(true, 0u);
            input_remap_runtime::setEquippedWeaponFiringGripInputActive(false);
            input_remap_runtime::setEquippedWeaponPrimaryDetached(false);
            input_remap_runtime::setProviderOpenVrGameInputSuppressed(false, false);
            input_remap_runtime::setProviderOpenVrGameInputSuppressed(true, false);
            _grabInput.heldWeaponTriggerEquipIntents = {};
            _grabInput.intentStates = {};
            clearGameplayCandidatesForHand(_rightHand, false);
            clearGameplayCandidatesForHand(_leftHand, true);
            return;
        }

        auto* hknp = frame.hknpWorld;
        const auto worldGeneration =
            _lifecycle.worldGenerationAtomic.load(
                std::memory_order_acquire);
        const auto skeletonGeneration =
            _lifecycle.skeletonGenerationAtomic.load(
                std::memory_order_acquire);
        const auto providerGeneration =
            _lifecycle.providerGenerationAtomic.load(
                std::memory_order_acquire);
        const auto collisionGeneration =
            _lifecycle.collisionGenerationAtomic.load(
                std::memory_order_acquire);
        _touchGrabRuntime.setGlobalSurfaceGrabEnabled(
            g_rockConfig.rockGlobalSurfaceGrabEnabled);
        _touchGrabRuntime.service(
            frame.bhkWorld,
            frame.hknpWorld,
            frame.deltaSeconds,
            worldGeneration,
            skeletonGeneration,
            providerGeneration,
            collisionGeneration);
        if (frame.menuBlocked) {
            _grabInput.intentStates = {};
            _touchGrabRuntime.releaseAll(
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                collisionGeneration);
        }
        constexpr int grabButton = input_remap_policy::kGrabButtonId;
        const bool ambidextrousHandoffAvailable =
            _equipped.handlingSettings.ambidextrousHandoffEnabled &&
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        const equipped_weapon_manual_ownership_policy::FiringGripModeAvailability firingGripModes{
            .primaryDetachEnabled = _equipped.handlingSettings.primaryDetachEnabled,
            .integratedDetachEnabled = _equipped.handlingSettings.
                immersiveWeapon.firingGripDetachEnabled,
            .ambidextrousHandoffAvailable = ambidextrousHandoffAvailable,
        };
        const bool gripZoneSettleEquipEnabled =
            equipped_weapon_manual_ownership_policy::canSettleEquipInGripZone(
                _equipped.handlingSettings.gripZoneEquipEnabled);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);
        const GrabInputHandContext handContext{
            .hknp = hknp,
            .grabButton = grabButton,
            .ambidextrousHandoffAvailable = ambidextrousHandoffAvailable,
            .firingGripModes = firingGripModes,
            .gripZoneSettleEquipEnabled = gripZoneSettleEquipEnabled,
            .farHmdConeGate = farHmdConeGate,
            .worldGeneration = worldGeneration,
            .skeletonGeneration = skeletonGeneration,
            .providerGeneration = providerGeneration,
            .collisionGeneration = collisionGeneration,
        };
        publishHandInputOwnership(_rightHand, false);
        publishHandInputOwnership(_leftHand, true);
        _powerArmorCandidates = {};
        _powerArmorProbeDiagnostics = {};
        _powerArmorCandidateFrame = runtime_state::currentFrame().frameIndex;
        for (const bool isLeft : {false, true}) {
            const auto& input = isLeft ? frame.left : frame.right;
            const auto& hand = isLeft ? _leftHand : _rightHand;
            RE::NiTransform presented{};
            if (frame.worldReady && !frame.menuBlocked && !input.disabled &&
                forceGrabHandBlockerMask(hand, isLeft, false, true) == 0 &&
                _dynamicHandCollision.getLastPresentedHandWorld(isLeft, presented)) {
                _powerArmorCandidates[isLeft ? 1u : 0u] =
                    _touchGrabRuntime.findPowerArmorCandidate(frame.hknpWorld, presented.translate, 0, {},
                        TouchGrabRuntime::kPowerArmorProximityRadiusGame, &_powerArmorProbeDiagnostics[isLeft ? 1u : 0u]);
            }
        }
        processProviderInteractionCommands(frame);
        serviceLooseGrenadeQuickDraw(frame);
        serviceEquippedWeaponNativeHandoff(frame);
        servicePendingForceGrabCommits(frame);
        updateEquippedWeaponDropVisuals(frame);
        updateSavedGrabOffsetGesture(frame);
        updateLooseGrenadeFuses(frame);
        publishHandInputOwnership(_rightHand, false);
        publishHandInputOwnership(_leftHand, true);


        // A shared hold presents the entire assembly on the first grab's
        // clock. Its update must precede the peer's visual-hand readback.
        if (held_scene_presentation::leftOwnsSharedAssembly()) {
            processGrabInputHand(frame, _leftHand, true, handContext);
            publishHandInputOwnership(_leftHand, true);
            processGrabInputHand(frame, _rightHand, false, handContext);
            publishHandInputOwnership(_rightHand, false);
        } else {
            processGrabInputHand(frame, _rightHand, false, handContext);
            publishHandInputOwnership(_rightHand, false);
            processGrabInputHand(frame, _leftHand, true, handContext);
            publishHandInputOwnership(_leftHand, true);
        }

        if (_rightHand.isHolding() ||
            _touchGrabRuntime.isHandActive(false)) {
            _twoHandedGrip.cancelHandVisualReturn(
                false,
                _rightHand.isHolding() ?
                    "generic-grab-acquired" :
                    "touch-grab-active");
        }
        if (frame.right.disabled ||
            _touchGrabRuntime.isHandActive(false)) {
            _rightHand.cancelGrabVisualReturn(
                frame.right.disabled ?
                    "hand-disabled" :
                    "touch-grab-active");
        } else {
            _rightHand.updateGrabVisualReturn(frame.right.rawHandWorld, frame.deltaSeconds);
        }
        if (_leftHand.isHolding() ||
            _touchGrabRuntime.isHandActive(true)) {
            _twoHandedGrip.cancelHandVisualReturn(
                true,
                _leftHand.isHolding() ?
                    "generic-grab-acquired" :
                    "touch-grab-active");
        }
        if (frame.left.disabled ||
            _touchGrabRuntime.isHandActive(true)) {
            _leftHand.cancelGrabVisualReturn(
                frame.left.disabled ?
                    "hand-disabled" :
                    "touch-grab-active");
        } else {
            _leftHand.updateGrabVisualReturn(frame.left.rawHandWorld, frame.deltaSeconds);
        }
    }
}

#include "api/AuthoredInteractionService.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/TriggerEquipReadinessTrace.h"
#include "physics-interaction/visual/GripZoneIndicators.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/weapon/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "api/ProviderRuntimeServices.h"

// Equipped-weapon frame: transitions, the per-frame equipped weapon update, authored primary grip runtime, handling settings, and shoulder sheath/retrieve.

namespace rock
{
    namespace
    {
        void maskProviderWeaponGrabInput(bool isLeft, bool holding, GrabButtonState& button)
        {
            if (input_remap_runtime::ownsBareFistInput()) {
                button = {};
                return;
            }
            using Flag = ::rock::api::input::HandInputSuppressionFlag;
            const auto flags = provider::currentHandInputSuppressionFlags(
                isLeft ? api::Hand::Left : api::Hand::Right);
            const bool press = provider::hasHandInputSuppressionFlag(flags, Flag::SuppressNormalGrabPress);
            const bool release = provider::hasHandInputSuppressionFlag(flags, Flag::SuppressGrabRelease);
            if (!press && !release) return;
            const auto filtered = input_remap_policy::suppressGrabInput(
                {.grabHeld = button.held, .grabPressed = button.pressed, .grabReleased = button.released},
                press, release, holding);
            button = {.held = filtered.grabHeld, .pressed = filtered.grabPressed, .released = filtered.grabReleased};
            // The UI reads raw levels. Drain gameplay edges so opening the
            // wheel cannot become a delayed detach after its lease is cleared.
            static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, input_remap_policy::kGrabButtonId));
        }
    }

    bool PhysicsInteraction::tryGetManualScopePresentationTarget(
        std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex,
        bool& outDirectTransitionRequired,
        const void* expectedWeapon,
        const void* expectedInstance) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        outDirectTransitionRequired = false;
        if (!g_rockConfig.rockEnableImmersiveScopes ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto snapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
        if (!snapshot.scopeEligible) {
            // Native classification also runs while idle. Ordinary sights
            // are an expected negative result, not a failed scope anchor.
            return false;
        }
        if (expectedWeapon && !manual_scope_target_policy::matchesNativeIdentity(
                snapshot.scopeWeaponIdentity, snapshot.scopeInstanceIdentity,
                reinterpret_cast<std::uintptr_t>(expectedWeapon), reinterpret_cast<std::uintptr_t>(expectedInstance))) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "Native scope admission identity rejected: publishedWeapon=0x{:X} instance=0x{:X} nativeWeapon=0x{:X} instance=0x{:X}",
                snapshot.scopeWeaponIdentity, snapshot.scopeInstanceIdentity,
                reinterpret_cast<std::uintptr_t>(expectedWeapon), reinterpret_cast<std::uintptr_t>(expectedInstance));
            return false;
        }
        const native_scope_sight_anchor_policy::PublicationIdentity publishedIdentity{
            .weaponGenerationKey = snapshot.weaponGenerationKey,
            .equippedWeaponOwnershipKey = snapshot.equippedWeaponOwnershipKey,
            .weaponFormID = snapshot.weaponFormID,
        };
        const native_scope_sight_anchor_policy::PublicationIdentity currentIdentity{
            .weaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey(),
            .equippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey(),
            .weaponFormID = _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
        };
        const NativeScopeResolvedAnchorSnapshot resolvedAnchor =
            _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity
            resolvedIdentity{
                .weaponGenerationKey = resolvedAnchor.weaponGenerationKey,
                .equippedWeaponOwnershipKey =
                    resolvedAnchor.equippedWeaponOwnershipKey,
                .weaponFormID = resolvedAnchor.weaponFormID,
            };
        if (!resolvedAnchor.valid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(
                resolvedIdentity,
                currentIdentity) ||
            !snapshot.nativeScopeOverlayValid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(publishedIdentity, currentIdentity)) {
            if (expectedWeapon) {
                ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                    "Native scope admission target rejected: anchorValid={} eligible={} overlayValid={} publishedGeneration={:016X} resolvedGeneration={:016X} currentGeneration={:016X}",
                    resolvedAnchor.valid, snapshot.scopeEligible, snapshot.nativeScopeOverlayValid,
                    snapshot.weaponGenerationKey, resolvedAnchor.weaponGenerationKey, currentIdentity.weaponGenerationKey);
            }
            return false;
        }
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        outDirectTransitionRequired = snapshot.manualDirectTransitionRequired;
        return true;
    }

    EquippedWeaponTransitionCoordinator::PendingGrip PhysicsInteraction::equippedWeaponContinuity() const
    {
        return _equipped.continuityGrip;
    }

    void PhysicsInteraction::restoreEquippedWeaponContinuity(const EquippedWeaponTransitionCoordinator::PendingGrip& grip)
    {
        _equipped.continuityGrip = grip;
        _equipped.gripResumePending = grip.pending;
        if (grip.pending) {
            ROCK_LOG_INFO(Weapon, "Equipped grip suspended form={:08X} hand={} paired={} support={} secondSupport={}",
                grip.targetWeaponFormID, grip.isLeft ? "left" : "right", grip.pairedGrips.valid(),
                grip.supportGrip.validCarry(), grip.secondSupportGrip.validCarry());
        }
    }

    void PhysicsInteraction::captureEquippedWeaponContinuity()
    {
        if (_equipped.gripResumePending) return;
        auto snapshot = _equipped.transition.pendingGrip();
        if (snapshot.pending) {
            _equipped.continuityGrip = snapshot;
            return;
        }
        if (!_twoHandedGrip.isManualOwnershipActive() || _twoHandedGrip.isHandsFreeRetained()) {
            _equipped.continuityGrip = {};
            _equipped.resumeAwaitingHold = {};
            return;
        }
        auto* weapon = currentEquippedWeaponForm();
        snapshot.targetWeaponFormID = weapon ? weapon->formID : 0u;
        snapshot.targetWeaponInstanceData = reinterpret_cast<std::uintptr_t>(currentEquippedWeaponInstanceData(weapon));
        if (!_equipped.transition.observedItemMatches(snapshot.targetWeaponFormID, snapshot.targetWeaponInstanceData)) {
            _equipped.continuityGrip = {};
            return;
        }
        weapon_grip_transfer::HandGrip firing{};
        bool carrierLeft = _twoHandedGrip.isFiringHandLeft();
        const char* captureFailure = nullptr;
        snapshot.pending = _twoHandedGrip.captureMenuCarry(firing, snapshot.pairedGrips,
            snapshot.supportGrip, snapshot.secondSupportGrip, carrierLeft, &captureFailure);
        snapshot.isLeft = carrierLeft;
        snapshot.firingHandWeaponLocal = firing.handWeaponLocal;
        snapshot.firingGripWeaponLocal = firing.gripWeaponLocal;
        snapshot.hasFiringHandWeaponLocal = firing.valid();
        snapshot.hasFiringGripWeaponLocal = firing.valid();
        snapshot.toggleAcquisitionCommitted = !snapshot.supportGrip.validCarry();
        snapshot.source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::HeldWeaponEquip;
        // Keep no scene pointers or world transforms. This is sampled after
        // grip decisions, while the outgoing nodes and controller frame live.
        _equipped.continuityGrip = snapshot;
        if (!snapshot.pending) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000, "Equipped menu-carry continuity capture unavailable form={:08X} hand={} reason={} instance={:#x}",
                snapshot.targetWeaponFormID, carrierLeft ? "left" : "right", captureFailure ? captureFailure : "unknown", snapshot.targetWeaponInstanceData);
        }
    }

    void PhysicsInteraction::updateEquippedWeaponTransition()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeGunState =
            f4vr::getNativeGunState(player);
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool currentIdentityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        weapon_transition_animation_acceleration::service(
            weapon_transition_animation_acceleration::ServiceInput{
                .player = player,
                .identity = currentIdentityCaptured ?
                    weapon_transition_animation_acceleration::Identity{
                        .formID = currentIdentity.formID,
                        .instanceData = currentIdentity.instanceData,
                        .equipIndex = currentIdentity.equipIndex,
                    } :
                    weapon_transition_animation_acceleration::Identity{},
                .nativeWeaponState = nativeWeaponState,
                .runtimeAllowed =
                    runtime.visualAuthorityAvailable &&
                    runtime.localSkeletonReady &&
                    !runtime.localMenuBlocking &&
                    !runtime.compatibilityConfigBlocking,
            });
        const bool nativeWeaponAnimationActive =
            provider::currentNativeAnimationAuthorityFlags() != 0 ||
            nativeGunState ==
                static_cast<std::uint32_t>(RE::GUN_STATE::kReloading);
        RE::NiTransform leftCarrySolvedWeaponWorld{};
        if (runtime.localMenuBlocking && !_equipped.menuReconcilePending) {
            restoreEquippedWeaponContinuity(_equipped.continuityGrip);
            cancelEquippedWeaponTransfersForMenu();
        }
        const auto& transfer = _equipped.transition.heldTransfer();
        const auto& incomingHand = transfer.request.isLeft ? _leftHand : _rightHand;
        auto* incomingReference = incomingHand.getHeldRef();
        _equipped.transition.validateHeldSource(incomingHand.isHolding(), incomingReference ? incomingReference->GetFormID() : 0u,
            incomingHand.heldGrabIdentity(), _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire));
        _equipped.transition.update(
            EquippedWeaponTransitionCoordinator::FrameInput{
                .deltaSeconds = runtime.deltaSeconds,
                .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
                .localSkeletonReady = runtime.localSkeletonReady,
                .menuBlocking = runtime.localMenuBlocking,
                .compatibilityBlocking = runtime.compatibilityConfigBlocking,
                .gripSuspended = _equipped.gripResumePending,
                .nativeWeaponState = nativeWeaponState,
                .intentionalShoulderSheathActive =
                    _equipped.shoulderSheath.active,
                .shoulderSheathFormID =
                    _equipped.shoulderSheath.weaponFormID,
                .shoulderSheathInstanceData =
                    _equipped.shoulderSheath.weaponInstanceData,
                .shoulderSheathEquipIndex =
                    _equipped.shoulderSheath.equipIndex,
                .nativeWeaponAnimationActive = nativeWeaponAnimationActive,
                // Read one frame behind the grip solve by design: this update
                // runs before TwoHandedGrip::update each frame, so the bridge
                // receives the previous frame's solved carry pose - the pose
                // that was actually rendered.
                .leftCarrySolvedWeaponWorldValid =
                    _twoHandedGrip.tryGetSolvedLeftFiringWeaponWorld(
                        leftCarrySolvedWeaponWorld),
                .leftCarrySolvedWeaponWorld = leftCarrySolvedWeaponWorld,
            });
        // Seam diagnostic: every menu close (a repeated exit can rebind the
        // same transition) and every native equip the coordinator observes.
        // Held and inventory equips arm it at their own call sites.
        static bool s_seamMenuBlocking = false;
        if (s_seamMenuBlocking && !runtime.localMenuBlocking) {
            armTransitionSeamTrace("menu-exit");
            // Fire readiness after a menu exit: the native draw/attack gates
            // per frame and each trigger edge, for 5 s.
            trigger_equip_readiness_trace::begin(currentEquippedWeaponFormId(), 0);
        }
        s_seamMenuBlocking = runtime.localMenuBlocking;
        static std::uint64_t s_seamTransitionSequence = 0;
        const auto transition = _equipped.transition.getPublicSnapshot();
        if (transition.transitionSequence != s_seamTransitionSequence) {
            s_seamTransitionSequence = transition.transitionSequence;
            if (transition.source == EquippedWeaponTransitionCoordinator::Source::ObservedEquip)
                armTransitionSeamTrace("observed-equip");
        }
    }

    /*
     * The equipped-weapon frame runs in two parts around the collider update.
     * beginEquippedWeaponFrame leases the hands that own the weapon, so hand
     * bodies the collider update recreates are re-leased before the next
     * physics step. updateEquippedWeaponFrame then runs the weapon stages in
     * order; they share one EquippedWeaponFrameWork that lives for the call.
     */
    struct PhysicsInteraction::EquippedWeaponFrameWork
    {
        const PhysicsFrameContext& frame;
        RE::bhkWorld* bhk = nullptr;
        RE::hknpWorld* hknp = nullptr;
        const EquippedWeaponFramePreparation& prepared;

        // Weapon identity, read once after the weapon collision update.
        weapon_recoil_policy::WeaponEvidence recoilWeapon{};
        std::uint64_t currentWeaponGenerationKey = 0;
        std::uint64_t currentEquippedWeaponOwnershipKey = 0;
        std::uint64_t currentAuthoredGripGenerationKey = 0;
        RE::TESObjectWEAP* observedEquippedWeapon = nullptr;
        std::uint32_t observedEquippedWeaponFormID = 0;
        std::uintptr_t observedEquippedWeaponInstanceData = 0;
        bool inputBlockingMenuActive = false;

        // Firing hand and the shoulder gesture.
        EquippedWeaponShoulderFrameResult shoulderFrame{};
        bool pendingPrimaryStartMatchesCurrentWeapon = false;
        bool firingHandIsLeft = false;
        bool supportHandIsLeft = true;
        immersive_weapon_policy::Decision firingGripDecision{};

        // Each hand's weapon contact.
        WeaponInteractionContact leftWeaponContact{};
        WeaponInteractionContact rightWeaponContact{};

        // Grab buttons after resume rearm and provider, holster and shoulder
        // masking. The firing hand's state is read lazily, once.
        GrabButtonState leftPhysicalGripState{};
        GrabButtonState rightPhysicalGripState{};
        bool leftGripHeld = false;
        bool rightGripHeld = false;
        EquippedWeaponGripOccupancy providerGripOccupancy{};
        virtual_holsters::Decision leftHolsterInput{};
        virtual_holsters::Decision rightHolsterInput{};
        GrabButtonState primaryGrabState{};
        bool primaryGrabStateRead = false;

        // Provider part authority for each hand.
        WeaponInteractionRuntimeState leftHandInteractionState{};
        WeaponInteractionRuntimeState rightHandInteractionState{};
        weapon_support_authority_policy::WeaponSupportAuthorityMode supportAuthorityMode{
            weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver
        };
        bool firingGripProximityAuthorityEnabled = false;
        bool leftHandHoldingObject = false;

        // Grip starts and the grip update.
        EquippedWeaponPrimaryGripInput primaryGripInput{};
        bool primaryPoseBlockerAvailable = false;
        bool ambidextrousHandoffAvailable = false;
        bool firingGripOwnershipFeatureAvailable = false;
        bool primaryDetachFeatureAvailable = false;
        bool pendingToggleGripRetained = false;
        EquippedWeaponGripOccupancy toggleOccupancyBefore{};
        bool primaryOnlyGripStartedThisFrame = false;
        bool pairedGripStartedThisFrame = false;
        bool supportGripStartedThisFrame = false;
        bool transferredSupportIsLeft = false;
        bool transferredBothSupports = false;
        std::array<equipped_weapon_toggle_grab_policy::TransferReleaseState, 2> pairedRelease{};
        bool nativeFiringGripTransfer = false;
        TwoHandedGripUpdateResult gripUpdateResult{};
    };

    void PhysicsInteraction::sampleEquippedSlotOne()
    {
        // Slot 1's equipment view is a guarded native read, so the frame owner
        // samples it rather than every roster query.
        const auto* weapon = native_hand_equipment::equippedViews()[1].weapon;
        const bool equipped = weapon != nullptr;
        _equippedSlotOne = {
            .formId = equipped ? weapon->formID : 0u,
            .equipped = equipped,
            .holdsHand = equipped &&
                fo4vr_actor_state_policy::equippedWeaponOccupiesHand(true, f4vr::getNativeWeaponState(f4vr::getPlayer())),
            .hand = equipped ? equipped_weapon_roster::handFor(native_hand_equipment::secondaryHandIsLeft()) :
                               equipped_weapon_roster::Hand::None,
        };
    }

    equipped_weapon_roster::Roster PhysicsInteraction::equippedWeaponRoster() const
    {
        using equipped_weapon_roster::handFor;
        const auto& pending = _equipped.transition.pendingGrip();
        const bool sessionFiringLeft = _twoHandedGrip.isFiringHandLeft();
        bool firingLeft = sessionFiringLeft;
        if (pending.pending) {
            auto* weapon = currentEquippedWeaponForm();
            firingLeft = equipped_weapon_transition_policy::resolveFiringHandIsLeft(
                sessionFiringLeft, weapon ? weapon->GetFormID() : 0u,
                reinterpret_cast<std::uintptr_t>(currentEquippedWeaponInstanceData(weapon)),
                {
                    .pending = pending.pending,
                    .isLeft = pending.supportGrip.validCarry() ? !pending.isLeft : pending.isLeft,
                    .formID = pending.targetWeaponFormID,
                    .instanceData = pending.targetWeaponInstanceData,
                    .previousFormID = pending.previousWeaponFormID,
                    .previousInstanceData = pending.previousWeaponInstanceData,
                });
        }
        const auto grips = _twoHandedGrip.getGripOccupancy();
        equipped_weapon_roster::Facts facts{};
        facts.slot0 = {
            .holdsHand = currentEquippedWeaponOccupiesHand(),
            .sessionFiringHand = handFor(sessionFiringLeft),
            .firingHand = handFor(firingLeft),
            .firingGripConfirmed = _twoHandedGrip.isFiringGripOccupied(),
            .firingGripDetached = _twoHandedGrip.isFiringGripDetached(),
            .partGrips = { {
                { .active = grips.right.partGripActive, .attachOnly = grips.right.partGripAttachOnly },
                { .active = grips.left.partGripActive, .attachOnly = grips.left.partGripAttachOnly },
            } },
        };
        facts.slot1 = _equippedSlotOne;
        facts.pending = {
            .bothHands = pending.pairedGrips.valid() || pending.secondSupportGrip.validCarry(),
            .supportHand = pending.supportGrip.validCarry() ? handFor(pending.isLeft) : equipped_weapon_roster::Hand::None,
        };
        return equipped_weapon_roster::Roster(facts);
    }

    PhysicsInteraction::EquippedWeaponFramePreparation PhysicsInteraction::beginEquippedWeaponFrame(
        const PhysicsFrameContext& frame,
        RE::hknpWorld* hknp)
    {
        EquippedWeaponFramePreparation prepared{};
        sampleEquippedSlotOne();
        prepared.secondaryEquipped = _equippedSlotOne.equipped;
        prepared.secondaryLeft = _equippedSlotOne.hand == equipped_weapon_roster::Hand::Left;
        prepared.secondaryRight = _equippedSlotOne.hand == equipped_weapon_roster::Hand::Right;
        // The hand holding the second weapon must not push that weapon's
        // dynamic proxy; its colliders drop only the second weapon's pairs.
        for (const bool isLeft : { false, true }) {
            setWeaponHandSuppression(hknp, isLeft, WeaponHandReason::SecondWeapon,
                _equippedSlotOne.holdsHand && _equippedSlotOne.hand == equipped_weapon_roster::handFor(isLeft),
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        }

        // The second weapon's hand cannot also hold the primary, by its firing
        // grip or a part grip. In the right hand it also ends ROCK's right
        // firing ownership; the primary moves to the left below. A
        // support-held (detached) primary keeps its support hand.
        const bool leftHandTaken = prepared.secondaryLeft &&
            (_twoHandedGrip.isFiringHandLeft() || _twoHandedGrip.isHandPartGripping(true));
        const bool rightHandTaken = prepared.secondaryRight &&
            (_twoHandedGrip.isHandPartGripping(false) ||
                (_twoHandedGrip.isManualOwnershipActive() && !_twoHandedGrip.isFiringHandLeft() &&
                    !_twoHandedGrip.isFiringGripDetached()));
        if (leftHandTaken || rightHandTaken) {
            ROCK_LOG_INFO(Weapon, "AKIMBO primary released from the second weapon's hand={}", prepared.secondaryLeft ? "left" : "right");
            _twoHandedGrip.reset();
            clearEquippedWeaponFiringGripInputState();
        }

        prepared.leftHandAvailableForAcquisition = !prepared.secondaryLeft &&
            force_grab_policy::availableForEquippedGrip(forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true));
        prepared.rightHandAvailableForAcquisition = !prepared.secondaryRight &&
            force_grab_policy::availableForEquippedGrip(forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true));
        if (_equipped.transition.heldTransfer().phase == held_weapon_transfer::Phase::Recovering &&
            _twoHandedGrip.isManualOwnershipActive()) {
            _twoHandedGrip.reset();
            clearEquippedWeaponFiringGripInputState();
        }

        prepared.weaponNode = resolveEquippedWeaponInteractionNode();
        /*
         * The weapon node does not keep ROCK's carry pose across frames, even
         * in part-carry. Republish ROCK's solved carry transform first so
         * weapon-part probes, firing-grip zone checks, and grip capture frames
         * all read the weapon where the player sees it - the same frame the
         * generated colliders follow.
         */
        (void)_twoHandedGrip.republishPartCarryWeaponTransform(prepared.weaponNode);
        prepared.rightHandWeaponEquipped = prepared.weaponNode != nullptr;
        prepared.retainedWeaponCollisionActive =
            _weaponCollision.hasWeaponBody() && _weaponCollision.getCurrentWeaponGenerationKey() != 0;
        /*
         * Reload can temporarily remove the first-person weapon node while ROCK
         * deliberately retains the generated weapon body set. Keep the dominant
         * hand under weapon authority until those retained bodies are gone.
         */
        bool rightHandWeaponAuthorityActive = prepared.rightHandWeaponEquipped || prepared.retainedWeaponCollisionActive;
        /*
         * A visible part-carry (no hand at the firing grip) frees the right hand
         * even while generated weapon bodies exist: the free hand needs live
         * colliders for offhand-parity interaction, the same layer 43 vs 44
         * coexistence the left hand already has. Reload-retained bodies with no
         * visible weapon node keep the dominant-hand suppression because the
         * part-carry state cannot survive a missing weapon node anyway.
         */
        if (prepared.rightHandWeaponEquipped && _twoHandedGrip.isFiringGripDetached()) {
            rightHandWeaponAuthorityActive = false;
        }
        /*
         * Left-firing carry frees the right hand the same way: the LEFT hand
         * owns the firing grip and the weapon transform, so the right hand has
         * support/free parity (its own part grip is leased separately below).
         */
        if (prepared.rightHandWeaponEquipped && _twoHandedGrip.isFiringHandLeft() && _twoHandedGrip.isFiringGripOccupied()) {
            rightHandWeaponAuthorityActive = false;
        }
        prepared.rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActive;

        const auto occupancy = equippedWeaponRoster().slotZeroGrips();
        prepared.leftSupportGripActive = occupancy.left.partGripActive;
        prepared.rightPartGripActive = occupancy.right.partGripActive;
        setWeaponHandSuppression(hknp, false, WeaponHandReason::Dominant, prepared.rightHandWeaponAuthorityActive);
        // A part-gripping free hand is a transform driver like the support hand
        // and must not also solve contacts against the weapon package.
        setWeaponHandSuppression(hknp, false, WeaponHandReason::Grip, prepared.rightPartGripActive);
        return prepared;
    }

    PhysicsInteraction::EquippedWeaponFrameResult PhysicsInteraction::updateEquippedWeaponFrame(
        const PhysicsFrameContext& frame,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp,
        const EquippedWeaponFramePreparation& prepared)
    {
        performance_profiler::ScopedTimer equippedFrameTimer(performance_profiler::Scope::EquippedWeaponInteraction);
        EquippedWeaponFrameWork w{
            .frame = frame,
            .bhk = bhk,
            .hknp = hknp,
            .prepared = prepared,
        };

        observeEquippedWeapon(w);
        routeEquippedWeaponPrimaryHand(w);
        resolveEquippedWeaponFiringHand(w);
        probeEquippedWeaponContacts(w);
        readEquippedWeaponGripButtons(w);
        resolveEquippedWeaponPartAuthority(w);
        startEquippedWeaponGrips(w);
        updateEquippedWeaponGrips(w);
        queueEquippedWeaponGripHaptics();
        commitEquippedWeaponSheathAndDrop(w);
        const EquippedWeaponFrameResult result = publishEquippedWeaponGripOwnership(w);
        finishDynamicWeaponCollision(w);

        // Final source transforms are queued after all animation/FRIK writers.
        if (retireHandsFreeWeapon(frame, prepared.weaponNode)) {
            return {};
        }
        if (f4vr::isNodeVisible(prepared.weaponNode)) {
            applyFinalWeaponMuzzleAuthority();
        }
        return result;
    }

    // Weapon collision, classification and identity keys, then the dynamic
    // weapon collision frame and the grip's equipped ownership.
    void PhysicsInteraction::observeEquippedWeapon(EquippedWeaponFrameWork& w)
    {
        const auto& frame = w.frame;
        const auto& runtime = runtime_state::currentFrame();
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);

            if (g_rockConfig.rockDebugVerboseLogging && ++_diagnostics.wpnNodeLogCounter >= 90) {
                _diagnostics.wpnNodeLogCounter = 0;
                if (weaponNode) {
                    ROCK_LOG_DEBUG(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}",
                        weaponNode->name.c_str(), weaponNode->world.translate.x, weaponNode->world.translate.y,
                        weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                }
            }
            _weaponCollision.update(w.hknp, weaponNode, frame.deltaSeconds, runtime.weaponDrawn);
            const auto weaponClassification = _weaponCollision.getEquippedWeaponClassification();
            w.recoilWeapon = {
                .formID = weaponClassification.formID,
                .keywordFlags = weaponClassification.keywordFlags,
                .sizeClass = weaponClassification.sizeClass,
                .source = weaponClassification.classificationSource,
                .resolved = weaponClassification.hasEquippedWeapon && weaponClassification.classificationResolved,
            };
            const bool realMeleeWeaponEquipped = weaponClassification.hasEquippedWeapon &&
                weaponClassification.classificationResolved && weaponClassification.sizeClass == WeaponSizeClass::Melee;
            input_remap_runtime::setRealMeleeWeaponEquipped(realMeleeWeaponEquipped);
        }

        w.currentWeaponGenerationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        w.currentEquippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        w.currentAuthoredGripGenerationKey = authored_support_grab_policy::resolveAuthoredGenerationKey(
            w.currentWeaponGenerationKey, w.currentEquippedWeaponOwnershipKey);
        _dynamicWeaponCollision.beginFrame(runtime.frameIndex, w.hknp, w.bhk, weaponNode, w.currentWeaponGenerationKey,
            dynamic_weapon_collision_policy::kDynamicCompoundEnabled && runtime.weaponDrawn && !frame.menuBlocked &&
                physicsWritesAllowedForWorld(frame.hknpWorld));
        reconcileEquippedWeaponHandlingMode();
        // WeaponCollision has just read the actual drawn inventory instance;
        // its observed identity exists before geometry/grab-session readiness.
        const auto transferPhase = _equipped.transition.heldTransfer().phase;
        _twoHandedGrip.observeEquippedOwnership(
            runtime.weaponDrawn && _weaponCollision.getCurrentObservedEquippedWeaponFormID() != 0 ?
                w.currentEquippedWeaponOwnershipKey : 0,
            w.currentAuthoredGripGenerationKey,
            transferPhase == held_weapon_transfer::Phase::AwaitGrip || transferPhase == held_weapon_transfer::Phase::Recovering);

        // A loose-weapon equip carries the originating physical hand into
        // the first equipped frame; use it immediately so input/contact
        // routing never spends a frame under the default right-hand role.
        w.observedEquippedWeapon = currentEquippedWeaponForm();
        w.observedEquippedWeaponFormID = w.observedEquippedWeapon ? w.observedEquippedWeapon->formID : 0;
        w.observedEquippedWeaponInstanceData =
            reinterpret_cast<std::uintptr_t>(currentEquippedWeaponInstanceData(w.observedEquippedWeapon));
        w.inputBlockingMenuActive = input_remap_runtime::isMenuInputActive();
    }

    // Which hand the primary weapon's firing grip belongs to when no physical
    // grab chose one: queues the default-hand equip and expires stale
    // hand-specific equip requests.
    void PhysicsInteraction::routeEquippedWeaponPrimaryHand(EquippedWeaponFrameWork& w)
    {
        using StartSource = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource;
        const auto& runtime = runtime_state::currentFrame();
        const auto& prepared = w.prepared;
        const auto observedEquippedWeaponFormID = w.observedEquippedWeaponFormID;
        const auto observedEquippedWeaponInstanceData = w.observedEquippedWeaponInstanceData;
        auto& equipGrip = _equipped.transition.pendingGrip();

        const bool pendingGripMatches = equipGrip.pending &&
            (equipGrip.targetWeaponFormID == 0 ||
                equipped_weapon_transition_policy::matchesExpectedIdentity(
                    observedEquippedWeaponFormID, observedEquippedWeaponInstanceData,
                    equipGrip.targetWeaponFormID, equipGrip.targetWeaponInstanceData,
                    equipGrip.previousWeaponFormID, equipGrip.previousWeaponInstanceData));
        // Primary firing hand when no physical grab chose one: opposite
        // the second weapon; else the hand of the Pip-Boy equip or move
        // that gave this exact item a hand; else the default hand.
        const auto directedPrimary = native_hand_equipment::lastDirectedPrimaryEquip();
        const bool directedPrimaryMatches = directedPrimary.sequence != 0 &&
            directedPrimary.form == observedEquippedWeaponFormID &&
            directedPrimary.instance == observedEquippedWeaponInstanceData;
        // A Pip-Boy hand choice stays pending until it can be applied:
        // the Pip-Boy closed, the weapon drawn and this exact item equipped.
        const bool directedPrimaryPending = directedPrimary.sequence != _equipped.directedPrimarySequence;
        const bool primaryDefaultLeft = prepared.secondaryEquipped ? prepared.secondaryRight :
            directedPrimaryMatches ? directedPrimary.left : g_rockConfig.rockLeftHandedMode;
        if (equipGrip.pending && equipGrip.source == StartSource::DefaultEquip &&
            (!primaryDefaultLeft || !pendingGripMatches || !runtime.weaponDrawn)) {
            equipGrip = {};
        }
        const bool handSpecificOwnership = _twoHandedGrip.isManualOwnershipActive() ||
            _equipped.transition.heldTransfer().active() || _equipped.gripResumePending ||
            _equipped.shoulderSheath.active ||
            (pendingGripMatches && (equipGrip.menuResume ||
                equipGrip.source == StartSource::HeldWeaponEquip ||
                equipGrip.source == StartSource::InventoryEquip ||
                equipGrip.source == StartSource::ShoulderRetrieval));
        bool queueDefaultLeft = equipped_weapon_transition_policy::observeDefaultLeftEquip(
            _equipped.defaultEquipObservedOwnershipKey,
            runtime.weaponDrawn && observedEquippedWeaponFormID != 0 ? w.currentEquippedWeaponOwnershipKey : 0,
            primaryDefaultLeft, handSpecificOwnership, w.inputBlockingMenuActive);
        // The primary takes the hand chosen in the Pip-Boy (equip or
        // cross-hand move), or the left when the second weapon holds the
        // right hand. A support-held primary keeps its support hand.
        if (!queueDefaultLeft && !w.inputBlockingMenuActive && runtime.weaponDrawn && observedEquippedWeaponFormID != 0 &&
            !equipGrip.pending && !_twoHandedGrip.isFiringGripDetached()) {
            const bool firingLeft = _twoHandedGrip.isFiringHandLeft();
            const bool moved = directedPrimaryPending && directedPrimaryMatches;
            if (moved) {
                _equipped.directedPrimarySequence = directedPrimary.sequence;
                native_hand_equipment::noteDirectedPrimaryApplied(directedPrimary.sequence);
                ROCK_LOG_INFO(Weapon, "AKIMBO primary hand request formID={:08X} hand={} firing={}",
                    observedEquippedWeaponFormID, primaryDefaultLeft ? "left" : "right", firingLeft ? "left" : "right");
            }
            if (primaryDefaultLeft && !firingLeft && (prepared.secondaryRight || moved)) {
                if (_twoHandedGrip.isManualOwnershipActive()) {
                    _twoHandedGrip.reset();
                    clearEquippedWeaponFiringGripInputState();
                }
                queueDefaultLeft = true;
            } else if (!primaryDefaultLeft && firingLeft && moved) {
                // Back to the right hand: release ROCK's left carry and
                // the native right carry resumes.
                ROCK_LOG_INFO(Weapon, "AKIMBO primary moved to hand=right formID={:08X}", observedEquippedWeaponFormID);
                _twoHandedGrip.reset();
                clearEquippedWeaponFiringGripInputState();
            }
        }
        if (queueDefaultLeft) {
            equipGrip = PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = true,
                .isLeft = true,
                .targetWeaponFormID = observedEquippedWeaponFormID,
                .targetWeaponInstanceData = observedEquippedWeaponInstanceData,
                .previousWeaponFormID = observedEquippedWeaponFormID,
                .previousWeaponInstanceData = observedEquippedWeaponInstanceData,
                .source = StartSource::DefaultEquip,
            };
            ROCK_LOG_INFO(Weapon, "Default weapon equip queued formID={:08X} instance={:#x} hand=left",
                observedEquippedWeaponFormID, observedEquippedWeaponInstanceData);
        }

        // An ordinary equip has no physical grab to time out. Keep its
        // exact-instance request until the hand/pose is ready or the item
        // changes; use the existing readiness diagnostics below.
        auto& pendingGrip = _equipped.transition.pendingGrip();
        if (pendingGrip.pending && !_equipped.transition.heldTransfer().active() && pendingGrip.source != StartSource::DefaultEquip) {
            pendingGrip.remainingSeconds -= (std::max)(0.0f, w.frame.deltaSeconds);
            if (pendingGrip.remainingSeconds <= 0.0f) {
                ROCK_LOG_WARN(Weapon,
                    "Held weapon manual ownership handoff expired targetForm={:08X} targetInstance={:#x} hand={} lastStartFailure={} supportReadiness={}",
                    pendingGrip.targetWeaponFormID,
                    pendingGrip.targetWeaponInstanceData,
                    pendingGrip.isLeft ? "left" : "right",
                    pendingGrip.lastStartFailureReason ? pendingGrip.lastStartFailureReason : "none",
                    authored_support_grab_policy::leftFiringTakeoverReadinessName(pendingGrip.takeoverWitness.last));
                pendingGrip = {};
            }
        }
    }

    // The shoulder sheath/retrieve gesture, then the firing and support hands
    // for this frame: a pending equip's hand when it matches this weapon,
    // else the grip's current firing hand.
    void PhysicsInteraction::resolveEquippedWeaponFiringHand(EquippedWeaponFrameWork& w)
    {
        const auto& pendingGrip = _equipped.transition.pendingGrip();
        const auto pendingFiringHandIsLeft = [&]() {
            const bool matches = pendingGrip.pending &&
                (pendingGrip.targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::matchesExpectedIdentity(
                        w.observedEquippedWeaponFormID, w.observedEquippedWeaponInstanceData,
                        pendingGrip.targetWeaponFormID, pendingGrip.targetWeaponInstanceData,
                        pendingGrip.previousWeaponFormID, pendingGrip.previousWeaponInstanceData));
            const bool firingLeft = matches ?
                (pendingGrip.supportGrip.validCarry() ? !pendingGrip.isLeft : pendingGrip.isLeft) :
                _twoHandedGrip.isFiringHandLeft();
            return std::pair{ matches, firingLeft };
        };

        const bool equippedWeaponShoulderStashActive =
            equipped_weapon_drop_policy::equippedWeaponShoulderStashAvailable(
                _equipped.handlingSettings.equippedWeaponShoulderStashEnabled);
        const bool shoulderFiringHandIsLeft = pendingFiringHandIsLeft().second;
        w.shoulderFrame = advanceEquippedWeaponShoulderCoordinator(
            w.frame,
            equippedWeaponShoulderStashActive,
            w.inputBlockingMenuActive,
            w.observedEquippedWeaponFormID,
            w.observedEquippedWeaponInstanceData,
            w.prepared.weaponNode,
            w.currentEquippedWeaponOwnershipKey,
            shoulderFiringHandIsLeft);

        // The shoulder coordinator may have changed the pending equip.
        const auto [pendingMatches, firingHandIsLeft] = pendingFiringHandIsLeft();
        w.pendingPrimaryStartMatchesCurrentWeapon = pendingMatches;
        w.firingHandIsLeft = firingHandIsLeft;
        w.supportHandIsLeft = !firingHandIsLeft;
        w.firingGripDecision = resolveEquippedWeaponDetachDecision(_equipped.handlingSettings);
    }

    // Each hand's weapon contact: operating-pose and touch overlaps first,
    // then the wider grab-anchor probe where allowed.
    void PhysicsInteraction::probeEquippedWeaponContacts(EquippedWeaponFrameWork& w)
    {
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        /*
         * While the LEFT hand carries the weapon, the node does not hold
         * last frame's carry pose here; the ranked part probes below
         * convert real palm points into node-local space, so a stale frame
         * made a forend grab select the wrong part. Publish the canonical
         * carry pose first so both hands probe the weapon where it
         * actually is.
         */
        (void)_twoHandedGrip.publishLeftFiringFeedForwardWeaponPose(weaponNode);

        const bool weaponContactQueriesRequired = weaponNode && w.currentWeaponGenerationKey != 0 &&
            _twoHandedGrip.requiresWeaponContactQueries(
                weaponNode, w.currentAuthoredGripGenerationKey,
                _equipped.handlingSettings.authoredOnlySupportGrabsEnabled,
                provider::hasWeaponPartTargetsForGeneration(w.currentWeaponGenerationKey));
        if (!weaponContactQueriesRequired) {
            performance_profiler::addCounter(performance_profiler::Counter::WeaponProbeFramesSkipped);
            // Drain contact notifications and reset acquisition provenance
            // immediately when leases end or the weapon returns to authored
            // seats. Authored activation and firing-grip zones run below.
            clearLeftWeaponContact();
            clearRightWeaponContact();
            return;
        }

        performance_profiler::addCounter(performance_profiler::Counter::WeaponProbeFramesEnabled);
        WeaponCollision::InteractionQueryBatch interactionQueries;
        std::optional<weapon_drive_pose::NativeFrame> operatingPose;
        if (provider::runtime::hasGrabPoints(w.currentWeaponGenerationKey)) {
            operatingPose.emplace(captureProviderPartQueryPose(weaponNode, w.currentWeaponGenerationKey));
        }
        const auto* operatingPosePointer = operatingPose ? &*operatingPose : nullptr;
        consumeWeaponContactForHand(weaponNode, true, w.frame.left, true, w.leftWeaponContact, interactionQueries, operatingPosePointer);
        const bool rightWeaponContactProbeAllowed = _twoHandedGrip.isFiringGripDetached() || w.firingHandIsLeft;
        consumeWeaponContactForHand(weaponNode, false, w.frame.right, rightWeaponContactProbeAllowed, w.rightWeaponContact,
            interactionQueries, operatingPosePointer);
    }

    void PhysicsInteraction::publishWeaponInteractionContact(bool isLeft, WeaponInteractionContact& contact)
    {
        auto& hand = isLeft ? _weaponContact.left : _weaponContact.right;
        hand.partKind.store(static_cast<std::uint32_t>(contact.partKind), std::memory_order_release);
        hand.reloadRole.store(static_cast<std::uint32_t>(contact.reloadRole), std::memory_order_release);
        hand.supportRole.store(static_cast<std::uint32_t>(contact.supportGripRole), std::memory_order_release);
        hand.socketRole.store(static_cast<std::uint32_t>(contact.socketRole), std::memory_order_release);
        hand.actionRole.store(static_cast<std::uint32_t>(contact.actionRole), std::memory_order_release);
        hand.gripPose.store(static_cast<std::uint32_t>(contact.fallbackGripPose), std::memory_order_release);
        contact.sequence = hand.sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        hand.missedFrames.store(0, std::memory_order_release);
    }

    void PhysicsInteraction::consumeWeaponContactForHand(
        RE::NiNode* weaponNode,
        bool isLeft,
        const HandFrameInput& handInput,
        bool probeAllowed,
        WeaponInteractionContact& outContact,
        WeaponCollision::InteractionQueryBatch& interactionQueries,
        const weapon_drive_pose::NativeFrame* operatingPose)
    {
        auto& handContact = isLeft ? _weaponContact.left : _weaponContact.right;
        auto& acquisitionState = _equipped.weaponInteractionAcquisitionStates[isLeft ? 0u : 1u];
        const auto apiHand = isLeft ? api::Hand::Left : api::Hand::Right;
        Hand& hand = isLeft ? _leftHand : _rightHand;
        const auto missFrame = [&]() {
            const auto missed = handContact.missedFrames.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (missed > WEAPON_CONTACT_TIMEOUT_FRAMES) {
                isLeft ? clearLeftWeaponContact() : clearRightWeaponContact();
            }
        };

        // Drain the physics-thread notification, but do not use an
        // arbitrary finger/body callback as palm-touch provenance.
        // Touch is the deterministic overlap below for both physical
        // hands and for either firing/support role.
        (void)handContact.bodyId.exchange(INVALID_CONTACT_BODY_ID, std::memory_order_acquire);

        const RE::NiPoint3 palmPivotWorld = computeGrabPalmPointWorldFromHandBasis(handInput.rawHandWorld, isLeft);

        // Operating probes: thumb and index tips, the pinky probes, and the
        // grab anchor when the wider probe is allowed.
        std::array<std::array<float, 3>, 3> extraProbes{};
        std::size_t extraCount = 0;
        std::array<std::array<float, 3>, 2> pinkyProbes{};
        std::size_t pinkyCount = 0;
        if (operatingPose && !handInput.disabled) {
            grab_pinch_pocket_policy::FingerFrame fingers;
            if (hand.tryGetPinchFingerFrame(fingers)) {
                extraProbes[extraCount++] = { fingers.thumbTip.x, fingers.thumbTip.y, fingers.thumbTip.z };
                extraProbes[extraCount++] = { fingers.indexTip.x, fingers.indexTip.y, fingers.indexTip.z };
            }
            std::array<RE::NiPoint3, 2> pinky{};
            pinkyCount = hand.getPinkyOperatingProbes(pinky);
            for (std::size_t i = 0; i < pinkyCount; ++i) {
                pinkyProbes[i] = { pinky[i].x, pinky[i].y, pinky[i].z };
            }
            if (probeAllowed) {
                extraProbes[extraCount++] = { handInput.grabAnchorWorld.x, handInput.grabAnchorWorld.y, handInput.grabAnchorWorld.z };
            }
        }

        const bool operatingObserved = operatingPose && !handInput.disabled &&
            _weaponCollision.tryFindInteractionContactNearPoint(weaponNode, palmPivotWorld,
                probeAllowed ? g_rockConfig.rockWeaponInteractionProbeRadius : g_rockConfig.rockWeaponInteractionTouchRadius,
                outContact, interactionQueries, apiHand, operatingPose, std::span(extraProbes).first(extraCount), true,
                std::span(pinkyProbes).first(pinkyCount));
        const bool touchObserved = !operatingObserved && weaponNode &&
            _weaponCollision.tryFindInteractionContactNearPoint(weaponNode, palmPivotWorld,
                g_rockConfig.rockWeaponInteractionTouchRadius, outContact, interactionQueries, apiHand, operatingPose);
        if (operatingObserved || touchObserved) {
            publishWeaponInteractionContact(isLeft, outContact);
        } else if (weaponNode && probeAllowed) {
            if (_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, handInput.grabAnchorWorld,
                    g_rockConfig.rockWeaponInteractionProbeRadius, outContact, interactionQueries, apiHand, operatingPose)) {
                publishWeaponInteractionContact(isLeft, outContact);
                if (g_rockConfig.rockDebugVerboseLogging && ++_diagnostics.weaponInteractionProbeLogCounter >= 90) {
                    _diagnostics.weaponInteractionProbeLogCounter = 0;
                    ROCK_LOG_DEBUG(Weapon,
                        "WeaponInteractionProbe: hand={} bodyId={} partKind={} supportRole={} reloadRole={} actionRole={} radius={:.1f}",
                        isLeft ? "left" : "right",
                        outContact.bodyId,
                        static_cast<int>(outContact.partKind),
                        static_cast<int>(outContact.supportGripRole),
                        static_cast<int>(outContact.reloadRole),
                        static_cast<int>(outContact.actionRole),
                        g_rockConfig.rockWeaponInteractionProbeRadius);
                }
            } else {
                missFrame();
            }
        } else {
            missFrame();
        }

        outContact.acquisitionSource =
            weapon_interaction_acquisition_policy::resolve(acquisitionState, touchObserved, outContact.valid);
    }

    void PhysicsInteraction::rearmResumedGripHold(bool isLeft, GrabButtonState& button)
    {
        const auto rearmed = equipped_weapon_toggle_grab_policy::rearmResumedHold(
            _equipped.resumeAwaitingHold[equipped_weapon_toggle_grab_policy::handIndex(isLeft)],
            { .held = button.held, .pressed = button.pressed, .released = button.released });
        button = { .held = rearmed.held, .pressed = rearmed.pressed, .released = rearmed.released };
    }

    void PhysicsInteraction::maskHolsterGripInput(const EquippedWeaponFrameWork& w, bool isLeft, GrabButtonState& button) const
    {
        const auto& decision = isLeft ? w.leftHolsterInput : w.rightHolsterInput;
        if (decision.consumeInput) {
            // Preserve an existing grip before any detach/toggle/drop
            // decision. Empty hands cannot acquire from this gesture.
            button = GrabButtonState{ .held = decision.retainGrip };
        }
    }

    void PhysicsInteraction::maskShoulderGestureGripInput(const EquippedWeaponFrameWork& w, bool isLeft, GrabButtonState& button) const
    {
        const auto& decision = w.shoulderFrame.decision;
        const bool consumed = isLeft ? decision.consumeLeftInput : decision.consumeRightInput;
        if (!consumed) {
            return;
        }

        if (decision.gestureAction == equipped_weapon_shoulder::Action::SubmitRetrieve) {
            // Retrieval starts a committed carry without forwarding
            // that same button cycle into detach, toggle, or ordinary
            // grab handling.
            button = {};
        } else if (decision.gestureAction == equipped_weapon_shoulder::Action::SubmitSheath) {
            // The coordinator consumed the physical tap/release.
            // Keep the current carry logically closed until native
            // sheathing captures its transfer and clears ownership.
            button = GrabButtonState{ .held = true };
        }
    }

    // Both hands' grab buttons, with menu-resume rearm, provider suppression,
    // Virtual Holsters release deferral and the shoulder gesture applied.
    void PhysicsInteraction::readEquippedWeaponGripButtons(EquippedWeaponFrameWork& w)
    {
        auto& leftPhysicalGripState = w.leftPhysicalGripState;
        auto& rightPhysicalGripState = w.rightPhysicalGripState;
        leftPhysicalGripState = peekGrabButtonState(true, input_remap_policy::kGrabButtonId);
        rightPhysicalGripState = peekGrabButtonState(false, input_remap_policy::kGrabButtonId);
        if (_equipped.transition.pendingGrip().menuResume) {
            leftPhysicalGripState.held = input_remap_runtime::isRawButtonPhysicallyHeld(true, input_remap_policy::kGrabButtonId);
            rightPhysicalGripState.held = input_remap_runtime::isRawButtonPhysicallyHeld(false, input_remap_policy::kGrabButtonId);
        }
        rearmResumedGripHold(true, leftPhysicalGripState);
        rearmResumedGripHold(false, rightPhysicalGripState);
        w.providerGripOccupancy = equippedWeaponRoster().slotZeroGrips();
        maskProviderWeaponGrabInput(true, w.providerGripOccupancy.left.weaponEngaged(), leftPhysicalGripState);
        maskProviderWeaponGrabInput(false, w.providerGripOccupancy.right.weaponEngaged(), rightPhysicalGripState);

        const auto holsterSnapshot = w.inputBlockingMenuActive || w.frame.menuBlocked ?
            virtual_holsters::Snapshot{} : virtual_holsters::readSnapshot();
        const auto holsterOccupancy = equippedWeaponRoster().slotZeroGrips();
        const auto advanceHolsterInput = [&](const bool isLeft, const GrabButtonState& button) {
            const auto handIndex = equipped_weapon_toggle_grab_policy::handIndex(isLeft);
            const auto& occupancy = isLeft ? holsterOccupancy.left : holsterOccupancy.right;
            const bool toggleGrab = occupancy.usesToggleGrab(_equipped.handlingSettings.weaponGrabMode);
            const auto decision = virtual_holsters::advance(
                _equipped.holsterInputStates[handIndex],
                virtual_holsters::Input{
                    .holster = holsterSnapshot,
                    .ownershipKey = w.currentEquippedWeaponOwnershipKey,
                    .grabButtonId = input_remap_policy::kGrabButtonId,
                    .isLeft = isLeft,
                    .weaponEngaged = occupancy.carriesWeapon(),
                    .toggleGrab = toggleGrab,
                    .held = button.held,
                    .pressed = button.pressed,
                    .released = button.released,
                });
            _equipped.holsterInputConsumedThisFrame[handIndex] = decision.consumeInput;
            if (decision.started) {
                ROCK_LOG_INFO(Weapon,
                    "VirtualHolsters weapon release deferred: hand={} slot={} mode={} held={} pressed={} released={} ownership={:016X}",
                    isLeft ? "left" : "right", holsterSnapshot.slot, toggleGrab ? "toggle" : "hold",
                    button.held, button.pressed, button.released, w.currentEquippedWeaponOwnershipKey);
            }
            return decision;
        };
        w.leftHolsterInput = advanceHolsterInput(true, leftPhysicalGripState);
        w.rightHolsterInput = advanceHolsterInput(false, rightPhysicalGripState);
        maskHolsterGripInput(w, true, leftPhysicalGripState);
        maskHolsterGripInput(w, false, rightPhysicalGripState);
        maskShoulderGestureGripInput(w, true, leftPhysicalGripState);
        maskShoulderGestureGripInput(w, false, rightPhysicalGripState);
        w.leftGripHeld = leftPhysicalGripState.held;
        w.rightGripHeld = rightPhysicalGripState.held;
    }

    // Provider part targets for each hand's contact, the support-role
    // reservation, the part indicators, and the support authority mode.
    void PhysicsInteraction::resolveEquippedWeaponPartAuthority(EquippedWeaponFrameWork& w)
    {
        const auto resolvePart = [&](const WeaponInteractionContact& contact, WeaponInteractionRuntimeState& state,
                                     weapon_part_runtime::Resolution& resolution) {
            const auto query = makeProviderWeaponPartTargetQuery(contact, _weaponCollision);
            const bool resolved = contact.valid && ::rock::provider::resolveWeaponPartTarget(query, resolution);
            const bool whitelistActive = resolved && resolution.whitelistActive != 0;
            const bool matched = resolved && resolution.matched != 0;
            if (whitelistActive && !matched) {
                state.supportGripAllowed = false;
            } else if (matched) {
                state.providerPartAuthority = makeWeaponProviderPartAuthority(query, resolution);
            }
            return matched;
        };
        weapon_part_runtime::Resolution leftWeaponPartResolution{};
        weapon_part_runtime::Resolution rightWeaponPartResolution{};
        const bool leftWeaponPartMatched = resolvePart(w.leftWeaponContact, w.leftHandInteractionState, leftWeaponPartResolution);
        const bool rightWeaponPartMatched = resolvePart(w.rightWeaponContact, w.rightHandInteractionState, rightWeaponPartResolution);

        /*
         * The offhand reservation is a SUPPORT-ROLE gate, not a physical
         * left-hand gate: it constrains whichever hand currently plays the
         * support role. Part grips by the free firing hand stay gated by
         * the provider part whitelist alone so PAPER reload sessions still
         * constrain which parts the free hand may take.
         */
        const auto offhandReservation = offhand_interaction_reservation::fromProvider(
            ::rock::provider::currentHandReservation(w.supportHandIsLeft ? api::Hand::Left : api::Hand::Right));
        if (!offhand_interaction_reservation::allowsSupportGrip(offhandReservation)) {
            (w.supportHandIsLeft ? w.leftHandInteractionState : w.rightHandInteractionState).supportGripAllowed = false;
        }

        const auto indicatorBodyId = [&](const WeaponInteractionContact& contact, const WeaponInteractionRuntimeState& state) {
            return contact.valid && state.supportGripAllowed && contact.weaponGenerationKey == w.currentWeaponGenerationKey ?
                contact.bodyId : weapon_part_runtime::kInvalidBodyId;
        };
        _equipped.partIndicatorBodyIds = {
            !w.prepared.leftHandAvailableForAcquisition ? weapon_part_runtime::kInvalidBodyId :
                indicatorBodyId(w.leftWeaponContact, w.leftHandInteractionState),
            !w.prepared.rightHandAvailableForAcquisition ? weapon_part_runtime::kInvalidBodyId :
                indicatorBodyId(w.rightWeaponContact, w.rightHandInteractionState),
        };
        _equipped.partIndicatorPoints = { w.leftWeaponContact.grabPoint, w.rightWeaponContact.grabPoint };
        _equipped.partIndicatorFrame = runtime_state::currentFrame().frameIndex;
        _equipped.partIndicatorGeneration = w.currentWeaponGenerationKey;

        w.leftHandHoldingObject = _leftHand.isHolding();
        // The grab-mode override follows the SUPPORT-ROLE hand's provider
        // resolution: that is the hand whose grip the mode describes.
        bool supportAuthorityProviderOverride = false;
        const bool supportWeaponPartMatched = w.supportHandIsLeft ? leftWeaponPartMatched : rightWeaponPartMatched;
        const auto& supportWeaponPartResolution = w.supportHandIsLeft ? leftWeaponPartResolution : rightWeaponPartResolution;
        if (supportWeaponPartMatched) {
            if (supportWeaponPartResolution.grabMode == weapon_part_runtime::GrabMode::FullTwoHandAuthority) {
                w.supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
                supportAuthorityProviderOverride = true;
            } else if (supportWeaponPartResolution.grabMode == weapon_part_runtime::GrabMode::AttachOnly) {
                w.supportAuthorityMode = weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport;
                supportAuthorityProviderOverride = true;
            }
        }
        w.firingGripProximityAuthorityEnabled =
            weapon_support_authority_policy::canApplyFiringGripProximityAuthority(supportAuthorityProviderOverride);
    }

    // Reads the firing hand's grab button once per frame. Menu rearm masks
    // gameplay edges, but firing-grip ownership follows the physical hand
    // state after the menu closes. The consumed snapshot is published so the
    // normal grab pipeline sees the same edges instead of re-consuming them.
    const GrabButtonState& PhysicsInteraction::readPrimaryGrabState(EquippedWeaponFrameWork& w)
    {
        if (w.primaryGrabStateRead) {
            return w.primaryGrabState;
        }
        const bool firingHandIsLeft = w.firingHandIsLeft;
        auto& primaryGrabState = w.primaryGrabState;
        primaryGrabState = readGrabButtonState(firingHandIsLeft, input_remap_policy::kGrabButtonId);
        primaryGrabState.held = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, input_remap_policy::kGrabButtonId);
        const auto physicalPrimaryGrabState = primaryGrabState;
        rearmResumedGripHold(firingHandIsLeft, primaryGrabState);
        maskProviderWeaponGrabInput(firingHandIsLeft,
            (firingHandIsLeft ? w.providerGripOccupancy.left : w.providerGripOccupancy.right).weaponEngaged(), primaryGrabState);
        maskHolsterGripInput(w, firingHandIsLeft, primaryGrabState);
        maskShoulderGestureGripInput(w, firingHandIsLeft, primaryGrabState);
        w.primaryGrabStateRead = true;
        (firingHandIsLeft ? w.leftPhysicalGripState : w.rightPhysicalGripState) = primaryGrabState;
        _grabInput.firingHandButtonFrame = SharedGrabButtonFrameState{
            .valid = true,
            .isLeft = firingHandIsLeft,
            .held = physicalPrimaryGrabState.held,
            .pressed = physicalPrimaryGrabState.pressed,
            .released = physicalPrimaryGrabState.released,
        };
        return primaryGrabState;
    }

    // Feature availability for this frame, then the grips a pending equip,
    // menu resume or grip press starts: a transferred support grip, a
    // transferred two-hand grip, or the firing grip alone.
    void PhysicsInteraction::startEquippedWeaponGrips(EquippedWeaponFrameWork& w)
    {
        using StartSource = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource;
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        const bool firingHandIsLeft = w.firingHandIsLeft;
        const bool inputBlockingMenuActive = w.inputBlockingMenuActive;
        const bool leftHandAvailableForAcquisition = w.prepared.leftHandAvailableForAcquisition;
        const bool rightHandAvailableForAcquisition = w.prepared.rightHandAvailableForAcquisition;
        const auto currentAuthoredGripGenerationKey = w.currentAuthoredGripGenerationKey;
        const auto currentEquippedWeaponOwnershipKey = w.currentEquippedWeaponOwnershipKey;
        const auto observedEquippedWeaponFormID = w.observedEquippedWeaponFormID;
        const auto& firingGripDecision = w.firingGripDecision;

        _grabInput.firingHandButtonFrame = {};
        w.primaryPoseBlockerAvailable = frik_visual_authority::canBlockPrimaryHandWeaponPose();
        w.ambidextrousHandoffAvailable = !w.prepared.secondaryEquipped && _equipped.handlingSettings.ambidextrousHandoffEnabled &&
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        w.firingGripOwnershipFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
            !_equipped.shoulderSheath.active && firingGripDecision.firingGripOwnershipEnabled,
            w.primaryPoseBlockerAvailable, weaponNode != nullptr, currentEquippedWeaponOwnershipKey);
        w.primaryDetachFeatureAvailable = equipped_weapon_manual_ownership_policy::featureAvailable(
            !_equipped.shoulderSheath.active && firingGripDecision.primaryDetachEnabled,
            w.primaryPoseBlockerAvailable, weaponNode != nullptr, currentEquippedWeaponOwnershipKey);

        // A pending transferred grip watches each hand for the release that
        // should apply once the grip exists.
        auto& pendingPrimaryStart = _equipped.transition.pendingGrip();
        const auto observeRelease = [&](bool isLeft, bool firingRole, bool menuResume) {
            const auto& physical = isLeft ? w.leftPhysicalGripState : w.rightPhysicalGripState;
            pendingPrimaryStart.pairedRelease[equipped_weapon_toggle_grab_policy::handIndex(isLeft)].observe(
                _equipped.handlingSettings.weaponGrabMode, firingRole,
                { .held = physical.held, .pressed = physical.pressed, .released = physical.released }, menuResume);
        };
        if (pendingPrimaryStart.pairedGrips.valid()) {
            observeRelease(false, !pendingPrimaryStart.isLeft, pendingPrimaryStart.menuResume);
            observeRelease(true, pendingPrimaryStart.isLeft, pendingPrimaryStart.menuResume);
        } else if (pendingPrimaryStart.supportGrip.validCarry()) {
            for (const bool isLeft : { false, true }) {
                if (isLeft != pendingPrimaryStart.isLeft && !pendingPrimaryStart.secondSupportGrip.validCarry()) {
                    continue;
                }
                observeRelease(isLeft, false, pendingPrimaryStart.menuResume);
            }
        } else if (pendingPrimaryStart.toggleAcquisitionCommitted) {
            observeRelease(firingHandIsLeft, true, true);
        }

        w.pendingToggleGripRetained = pendingPrimaryStart.toggleAcquisitionCommitted;
        if (inputBlockingMenuActive && pendingPrimaryStart.source != StartSource::DefaultEquip) {
            pendingPrimaryStart = {};
        } else if (pendingPrimaryStart.pending && pendingPrimaryStart.source != StartSource::DefaultEquip &&
                   !equipped_weapon_manual_ownership_policy::shouldKeepPendingPrimaryOnlyStart(
                       equipped_weapon_manual_ownership_policy::PendingPrimaryOnlyStartInput{
                           .pending = pendingPrimaryStart.pending,
                           .gripHeld = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, input_remap_policy::kGrabButtonId) ||
                               w.pendingToggleGripRetained || pendingPrimaryStart.supportGrip.validCarry(),
                           .source = pendingPrimaryStart.source,
                           .ownershipModeEnabled = firingGripDecision.firingGripOwnershipEnabled,
                           .primaryPoseBlockerAvailable = w.primaryPoseBlockerAvailable,
                       })) {
            pendingPrimaryStart = {};
        }

        const input_remap_policy::EquippedWeaponFiringGripInputGate firingGripInputGate{
            .featureAvailable = w.firingGripOwnershipFeatureAvailable,
            .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
            .menuInputActive = inputBlockingMenuActive,
        };
        if (input_remap_policy::shouldConsumeEquippedWeaponFiringGripInput(firingGripInputGate)) {
            const auto& primaryState = readPrimaryGrabState(w);
            if (input_remap_policy::shouldUseEquippedWeaponFiringGripInput(firingGripInputGate)) {
                w.primaryGripInput = EquippedWeaponPrimaryGripInput{
                    .held = primaryState.held,
                    .pressed = primaryState.pressed,
                    .released = primaryState.released,
                };
            }
        }
        (void)readPrimaryGrabState(w);

        w.toggleOccupancyBefore = _twoHandedGrip.getGrabInputOccupancy();
        w.transferredSupportIsLeft = pendingPrimaryStart.isLeft;
        w.transferredBothSupports = pendingPrimaryStart.secondSupportGrip.validCarry();
        w.pairedRelease = pendingPrimaryStart.pairedRelease;
        const bool transferredSupportIsLeft = w.transferredSupportIsLeft;
        const bool firingHandAvailableForAcquisition = firingHandIsLeft ? leftHandAvailableForAcquisition : rightHandAvailableForAcquisition;
        const bool pairedHandsAvailable = !pendingPrimaryStart.pairedGrips.valid() ||
            (leftHandAvailableForAcquisition && rightHandAvailableForAcquisition);
        if (pendingPrimaryStart.pending && pendingPrimaryStart.source == StartSource::DefaultEquip &&
            (!firingHandAvailableForAcquisition || !w.firingGripOwnershipFeatureAvailable)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "Default left equip waiting formID={:08X} handAvailable={} ownershipAvailable={} poseBlocker={} weaponNode={} ownership={:016X}",
                observedEquippedWeaponFormID, firingHandAvailableForAcquisition, w.firingGripOwnershipFeatureAvailable,
                w.primaryPoseBlockerAvailable, weaponNode != nullptr, currentEquippedWeaponOwnershipKey);
        }

        // Transferred support grip (one or both hands on support stations).
        if (w.pendingPrimaryStartMatchesCurrentWeapon && pendingPrimaryStart.supportGrip.validCarry() &&
            w.primaryDetachFeatureAvailable && !inputBlockingMenuActive && !_twoHandedGrip.isManualOwnershipActive() &&
            (transferredSupportIsLeft ? leftHandAvailableForAcquisition : rightHandAvailableForAcquisition) &&
            (!pendingPrimaryStart.secondSupportGrip.validCarry() || (leftHandAvailableForAcquisition && rightHandAvailableForAcquisition)) &&
            (transferredSupportIsLeft ? w.leftHandInteractionState : w.rightHandInteractionState).supportGripAllowed &&
            (!w.transferredBothSupports || (w.leftHandInteractionState.supportGripAllowed && w.rightHandInteractionState.supportGripAllowed))) {
            const char* failure = nullptr;
            if (_twoHandedGrip.beginTransferredSupportGrip(weaponNode, currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
                    pendingPrimaryStart.supportGrip, &failure,
                    pendingPrimaryStart.secondSupportGrip.validCarry() ? &pendingPrimaryStart.secondSupportGrip : nullptr)) {
                _equipped.transition.recordGripAcquired(observedEquippedWeaponFormID, w.observedEquippedWeaponInstanceData,
                    transferredSupportIsLeft, held_weapon_transfer::Role::Support);
                w.supportGripStartedThisFrame = true;
                w.toggleOccupancyBefore = _twoHandedGrip.getGrabInputOccupancy();
                pendingPrimaryStart = {};
            } else {
                pendingPrimaryStart.lastStartFailureReason = failure;
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Support equipped grip transfer waiting form={:08X} hand={} reason={}",
                    observedEquippedWeaponFormID, transferredSupportIsLeft ? "left" : "right", failure ? failure : "unknown");
            }
        }

        // Firing grip alone, or a transferred two-hand grip.
        if (pendingPrimaryStart.supportGrip.validCarry() || !w.firingGripOwnershipFeatureAvailable || inputBlockingMenuActive ||
            !firingHandAvailableForAcquisition || !pairedHandsAvailable || _twoHandedGrip.isManualOwnershipActive()) {
            if (inputBlockingMenuActive && pendingPrimaryStart.source != StartSource::DefaultEquip) {
                pendingPrimaryStart = {};
            }
            return;
        }

        const auto& primaryState = readPrimaryGrabState(w);
        if (pendingPrimaryStart.pending && !primaryState.held && pendingPrimaryStart.source == StartSource::GripInput &&
            !w.pendingToggleGripRetained) {
            pendingPrimaryStart = {};
        }

        const bool pendingMatches = w.pendingPrimaryStartMatchesCurrentWeapon;
        const bool pendingPrimaryOnlyStartRequested = equipped_weapon_manual_ownership_policy::shouldStartPendingPrimaryOnlyGrip(
            pendingMatches, primaryState.held || w.pendingToggleGripRetained, pendingPrimaryStart.source);
        const bool primaryOnlyStartRequested = weaponNode != nullptr && currentEquippedWeaponOwnershipKey != 0 &&
            ((w.primaryDetachFeatureAvailable && primaryState.held && primaryState.pressed) || pendingPrimaryOnlyStartRequested);
        /*
         * The whole grip subsystem (session keys, reconcile/rebind,
         * authored canonical, support capability and candidate) runs
         * on the AUTHORED generation key, which exists the moment the
         * weapon node does. The takeover entry must use the same key:
         * keying it to the raw collision generation made every
         * left-hand equip wait for the collider build and present the
         * weapon at the native right-hand attach until takeover.
         */
        if (pendingMatches && pendingPrimaryStart.isLeft &&
            (!pendingPrimaryStart.hasFiringHandWeaponLocal || !pendingPrimaryStart.hasFiringGripWeaponLocal)) {
            pendingPrimaryStart.hasFiringHandWeaponLocal = _twoHandedGrip.tryBuildCurrentLeftFiringGripCapture(weaponNode,
                currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
                pendingPrimaryStart.firingHandWeaponLocal, pendingPrimaryStart.firingGripWeaponLocal);
            pendingPrimaryStart.hasFiringGripWeaponLocal = pendingPrimaryStart.hasFiringHandWeaponLocal;
        }
        const RE::NiTransform* capturedFiringHandWeaponLocal =
            pendingMatches && pendingPrimaryStart.hasFiringHandWeaponLocal ? &pendingPrimaryStart.firingHandWeaponLocal : nullptr;
        const RE::NiPoint3* capturedFiringGripWeaponLocal =
            pendingMatches && pendingPrimaryStart.hasFiringGripWeaponLocal ? &pendingPrimaryStart.firingGripWeaponLocal : nullptr;
        const bool retainUntilPhysicalGrip = pendingMatches &&
            (pendingPrimaryStart.source == StartSource::ShoulderRetrieval ||
                pendingPrimaryStart.source == StartSource::DefaultEquip ||
                pendingPrimaryStart.source == StartSource::InventoryEquip ||
                w.pendingToggleGripRetained);
        const auto leftTakeoverReadiness = _twoHandedGrip.getLeftFiringTakeoverReadiness(weaponNode,
            currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
            _equipped.handlingSettings.authoredOnlySupportGrabsEnabled);
        const bool leftTakeoverBlocked = pendingMatches && primaryOnlyStartRequested && firingHandIsLeft &&
            !pendingPrimaryStart.pairedGrips.valid() &&
            !authored_support_grab_policy::leftFiringTakeoverReady(leftTakeoverReadiness);
        const auto observedWeaponName = [&]() {
            return w.observedEquippedWeapon ? RE::TESFullName::GetFullName(*w.observedEquippedWeapon, false) : std::string_view{ "unknown" };
        };
        if (leftTakeoverBlocked) {
            if (pendingPrimaryStart.takeoverWitness.observe(leftTakeoverReadiness)) {
                ROCK_LOG_WARN(Weapon,
                    "Left firing-grip start waiting weapon='{}' formID={:08X} authored support readiness={} collisionGeneration={:016X} authoredGeneration={:016X} ownership={:016X} firingCapture={} rightHolding={}",
                    observedWeaponName(),
                    pendingPrimaryStart.targetWeaponFormID,
                    authored_support_grab_policy::leftFiringTakeoverReadinessName(leftTakeoverReadiness),
                    w.currentWeaponGenerationKey,
                    currentAuthoredGripGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    pendingPrimaryStart.hasFiringHandWeaponLocal && pendingPrimaryStart.hasFiringGripWeaponLocal,
                    _rightHand.isHolding());
            }
            return;
        }
        if (!primaryOnlyStartRequested) {
            return;
        }

        const char* startFailureReason = nullptr;
        const bool pairedTransfer = pendingMatches && pendingPrimaryStart.pairedGrips.valid();
        const bool started = pairedTransfer ?
            _twoHandedGrip.beginTransferredTwoHandGrip(weaponNode, currentAuthoredGripGenerationKey,
                currentEquippedWeaponOwnershipKey, pendingPrimaryStart.pairedGrips, &startFailureReason) :
            _twoHandedGrip.beginPrimaryOnlyGrip(weaponNode, currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
                firingHandIsLeft, capturedFiringHandWeaponLocal, capturedFiringGripWeaponLocal, retainUntilPhysicalGrip,
                !pendingPrimaryStart.menuResume && pendingPrimaryStart.source != StartSource::DefaultEquip, &startFailureReason);
        if (!started) {
            pendingPrimaryStart.lastStartFailureReason = startFailureReason;
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "Equipped hand transfer blocked weapon='{}' formID={:08X} hand={} reason={} generation={:016X} ownership={:016X} physicalHeld={} toggleRetained={}",
                observedWeaponName(),
                observedEquippedWeaponFormID,
                firingHandIsLeft ? "left" : "right",
                startFailureReason ? startFailureReason : "unknown",
                currentAuthoredGripGenerationKey,
                currentEquippedWeaponOwnershipKey,
                primaryState.held,
                w.pendingToggleGripRetained);
            return;
        }

        _equipped.transition.recordGripAcquired(observedEquippedWeaponFormID, w.observedEquippedWeaponInstanceData,
            firingHandIsLeft, pairedTransfer ? held_weapon_transfer::Role::Paired : held_weapon_transfer::Role::Firing);
        w.pairedGripStartedThisFrame = pairedTransfer;
        if (pairedTransfer) {
            (void)_twoHandedGrip.commitPersistentEquippedCarryInputAcquisition(firingHandIsLeft);
            w.toggleOccupancyBefore = _twoHandedGrip.getGrabInputOccupancy();
        }
        if (pendingPrimaryStart.pending) {
            ROCK_LOG_INFO(Weapon, "Equipped hand transfer completed weapon='{}' formID={:08X} hand={} source={}",
                observedWeaponName(), observedEquippedWeaponFormID, firingHandIsLeft ? "left" : "right",
                static_cast<unsigned>(pendingPrimaryStart.source));
        }
        w.primaryOnlyGripStartedThisFrame = true;
        w.nativeFiringGripTransfer = !pendingPrimaryOnlyStartRequested &&
            equipped_weapon_toggle_grab_policy::firingUsesToggle(_equipped.handlingSettings.weaponGrabMode) &&
            _equipped.handlingSettings.lastGripReleaseDropEnabled;
        if (w.nativeFiringGripTransfer) {
            (firingHandIsLeft ? w.toggleOccupancyBefore.left : w.toggleOccupancyBefore.right).firingGripActive = true;
        }
        pendingPrimaryStart = {};
        w.primaryGripInput = EquippedWeaponPrimaryGripInput{
            .held = primaryState.held,
            .pressed = primaryState.pressed,
            .released = primaryState.released,
        };
    }

    // Provider part drives, the toggle-grab decision, the two-handed grip
    // update itself, and the toggle reconcile that follows it.
    void PhysicsInteraction::updateEquippedWeaponGrips(EquippedWeaponFrameWork& w)
    {
        const auto& frame = w.frame;
        const auto& runtime = runtime_state::currentFrame();
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        const bool firingHandIsLeft = w.firingHandIsLeft;
        const bool inputBlockingMenuActive = w.inputBlockingMenuActive;
        const auto currentEquippedWeaponOwnershipKey = w.currentEquippedWeaponOwnershipKey;
        const auto& firingGripDecision = w.firingGripDecision;

        std::array<const RE::NiAVObject*, api::weaponparts::kMaxDrives> drivenSourceNodes{};
        if (weaponNode) {
            (void)applyProviderWeaponPartDrives(weaponNode, w.currentWeaponGenerationKey, frame, drivenSourceNodes);
        } else {
            _providerDrives.resultCount = 0;
        }

        /*
         * Firing-grip reattach is the squeeze gesture (grab held with the
         * palm on the grip); the reattach zone is evaluated by
         * TwoHandedGrip. This only gates whether each free hand may be
         * captured at all - either hand can take the firing grip when
         * ambidextrous takeover is available.
         */
        bool leftReattachEligible = false;
        bool rightReattachEligible = false;
        if (_twoHandedGrip.isFiringGripDetached() && (w.primaryDetachFeatureAvailable || _twoHandedGrip.isHandsFreeRetained())) {
            const auto reattachEligible = [&](bool handAvailable, const Hand& hand) {
                return handAvailable && weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                    weapon_two_handed_grip_math::FiringGripReattachInput{
                        .partCarryActive = true,
                        .menuInputActive = inputBlockingMenuActive,
                        .handHoldingObject = hand.isHolding(),
                    });
            };
            leftReattachEligible = reattachEligible(w.prepared.leftHandAvailableForAcquisition, _leftHand);
            rightReattachEligible = reattachEligible(w.prepared.rightHandAvailableForAcquisition, _rightHand);
        }

        // Toggle grab turns each hand's button into a logical held state.
        const auto toToggleButtonState = [](const GrabButtonState& state) {
            return equipped_weapon_toggle_grab_policy::ButtonState{
                .held = state.held,
                .pressed = state.pressed,
                .released = state.released,
            };
        };
        auto toggleGrabDecision = equipped_weapon_toggle_grab_policy::prepare(
            _equipped.toggleGrabState,
            equipped_weapon_toggle_grab_policy::Input{
                .weaponGrabMode = _equipped.handlingSettings.weaponGrabMode,
                .inputAllowed = !inputBlockingMenuActive,
                .weaponOwnershipKey = currentEquippedWeaponOwnershipKey,
                .occupancy = w.toggleOccupancyBefore,
                .nativeFiringGripTransfer = w.nativeFiringGripTransfer,
                .left = toToggleButtonState(w.leftPhysicalGripState),
                .right = toToggleButtonState(w.rightPhysicalGripState),
            });
        if (w.pairedGripStartedThisFrame || w.supportGripStartedThisFrame) {
            equipped_weapon_toggle_grab_policy::adoptTransferredGrips(_equipped.toggleGrabState,
                _equipped.handlingSettings.weaponGrabMode, currentEquippedWeaponOwnershipKey, _twoHandedGrip.getGrabInputOccupancy());
            for (const bool isLeft : { false, true }) {
                if (w.supportGripStartedThisFrame && !w.transferredBothSupports && isLeft != w.transferredSupportIsLeft) {
                    continue;
                }
                const auto index = equipped_weapon_toggle_grab_policy::handIndex(isLeft);
                const bool released = w.pairedRelease[index].releaseRequested;
                (isLeft ? toggleGrabDecision.left : toggleGrabDecision.right) = { .held = !released, .released = released };
                (isLeft ? toggleGrabDecision.leftReleasePressConsumed : toggleGrabDecision.rightReleasePressConsumed) = released;
                if (released) {
                    _equipped.toggleGrabState.hands[index] = equipped_weapon_toggle_grab_policy::HandState::ReleasePending;
                }
            }
        }
        const auto firingHandIndex = equipped_weapon_toggle_grab_policy::handIndex(firingHandIsLeft);
        if (w.primaryOnlyGripStartedThisFrame && !w.pairedGripStartedThisFrame && w.pairedRelease[firingHandIndex].releaseRequested) {
            // Inventory already owns the weapon. Apply the saved release
            // through its requested hand and configured drop mode.
            (void)_twoHandedGrip.commitPersistentEquippedCarryInputAcquisition(firingHandIsLeft);
            (firingHandIsLeft ? toggleGrabDecision.left : toggleGrabDecision.right) = { .released = true };
            (firingHandIsLeft ? toggleGrabDecision.leftReleasePressConsumed : toggleGrabDecision.rightReleasePressConsumed) = true;
            _equipped.toggleGrabState.hands[firingHandIndex] = equipped_weapon_toggle_grab_policy::HandState::ReleasePending;
        }
        // A release may already be pending when the sphere becomes active.
        // Override that logical open state too, then reconcile it below so
        // leaving the sphere cannot replay the refused toggle release.
        if (w.leftHolsterInput.consumeInput) {
            toggleGrabDecision.left = { .held = w.leftHolsterInput.retainGrip };
        }
        if (w.rightHolsterInput.consumeInput) {
            toggleGrabDecision.right = { .held = w.rightHolsterInput.retainGrip };
        }
        w.leftGripHeld = toggleGrabDecision.left.held;
        w.rightGripHeld = toggleGrabDecision.right.held;
        const auto& logicalPrimaryGrip = firingHandIsLeft ? toggleGrabDecision.left : toggleGrabDecision.right;
        w.primaryGripInput = EquippedWeaponPrimaryGripInput{
            .held = logicalPrimaryGrip.held,
            .pressed = logicalPrimaryGrip.pressed,
            .released = logicalPrimaryGrip.released,
        };
        auto& releasePressConsumed = _equipped.toggleGrabReleasePressConsumedThisFrame;
        releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(true)] =
            releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(true)] || toggleGrabDecision.leftReleasePressConsumed;
        releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(false)] =
            releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(false)] || toggleGrabDecision.rightReleasePressConsumed;

        // The grip update.
        const auto captureScopeHandDriverFrame = [](const bool isLeft) {
            EquippedWeaponScopeHandDriverFrame result{};
            result.nodeAvailable = frik_hand_world_authority::tryGetInputDriverWorld(isLeft, result.world);
            result.worldFinite = result.nodeAvailable && finiteNiTransform(result.world);
            result.valid = result.worldFinite;
            return result;
        };
        const EquippedWeaponScopeHandDriverFrame leftHandDriverFrame = captureScopeHandDriverFrame(true);
        const EquippedWeaponScopeHandDriverFrame rightHandDriverFrame = captureScopeHandDriverFrame(false);
        bool nativeScopeRequestActive = false;
        const bool nativeScopeRequestStateValid = tryReadNativeScopeRequestState(nativeScopeRequestActive);
        const bool manualScopeActivationRequested = input_remap_runtime::isManualScopeActivationRequested();
        const bool pairedGripPending = _equipped.transition.pendingGrip().pairedGrips.valid() ||
            _equipped.transition.pendingGrip().supportGrip.validCarry();
        const auto toGripInput = [](const GrabButtonState& state) {
            return EquippedWeaponPrimaryGripInput{ .held = state.held, .pressed = state.pressed, .released = state.released };
        };
        const EquippedWeaponGripFrameInput gripFrameInput{
            .leftGripHeld = !pairedGripPending && w.leftGripHeld,
            .rightGripHeld = !pairedGripPending && w.rightGripHeld,
            .leftHandHoldingObject = w.leftHandHoldingObject,
            .rightHandHoldingObject = _rightHand.isHolding(),
            .leftHandAvailableForAcquisition = w.prepared.leftHandAvailableForAcquisition,
            .rightHandAvailableForAcquisition = w.prepared.rightHandAvailableForAcquisition,
            .leftReattachEligible = leftReattachEligible,
            .rightReattachEligible = rightReattachEligible,
            .scopeMenuOpen = runtime.localScopeMenuOpen,
            .manualScopeActivationRequested = manualScopeActivationRequested,
            .nativeScopeRequestStateValid = nativeScopeRequestStateValid,
            .nativeScopeRequestActive = nativeScopeRequestActive,
            .leftHandDriverFrame = leftHandDriverFrame,
            .rightHandDriverFrame = rightHandDriverFrame,
            .primaryGripInput = w.primaryGripInput,
            .leftPhysicalGripInput = toGripInput(w.leftPhysicalGripState),
            .rightPhysicalGripInput = toGripInput(w.rightPhysicalGripState),
            .hmdPositionWorld = frame.hmdPositionWorld,
            .weaponGrabMode = _equipped.handlingSettings.weaponGrabMode,
            .animationBoundaryActive = frame.reloadBoundaryActive,
            .hasHmdFrame = frame.hasHmdFrame,
            .recoilWeapon = w.recoilWeapon,
        };
        auto effectiveHandlingSettings = _equipped.handlingSettings;
        effectiveHandlingSettings.firingGripOwnershipEnabled = w.firingGripOwnershipFeatureAvailable;
        effectiveHandlingSettings.ambidextrousHandoffEnabled = w.ambidextrousHandoffAvailable;
        effectiveHandlingSettings.primaryDetachEnabled = w.primaryDetachFeatureAvailable;
        effectiveHandlingSettings.detachAuthority = w.primaryDetachFeatureAvailable ?
            firingGripDecision.authority : immersive_weapon_policy::DetachAuthority::None;
        effectiveHandlingSettings.firingGripReattachRadiusGameUnits = firingGripDecision.reattachRadiusGameUnits;
        effectiveHandlingSettings.weaponGripHapticDurationSeconds = firingGripDecision.gripHapticDurationSeconds;
        effectiveHandlingSettings.firingGripAttachHapticIntensity = firingGripDecision.gripAttachHapticIntensity;
        effectiveHandlingSettings.firingGripDetachHapticIntensity = firingGripDecision.gripDetachHapticIntensity;
        w.gripUpdateResult = _twoHandedGrip.update(
            weaponNode,
            w.leftWeaponContact,
            w.rightWeaponContact,
            gripFrameInput,
            frame.deltaSeconds,
            w.currentAuthoredGripGenerationKey,
            currentEquippedWeaponOwnershipKey,
            _weaponCollision,
            w.leftHandInteractionState,
            w.rightHandInteractionState,
            w.supportAuthorityMode,
            w.firingGripProximityAuthorityEnabled,
            effectiveHandlingSettings);

        const auto toggleReconcileDecision = equipped_weapon_toggle_grab_policy::reconcile(
            _equipped.toggleGrabState,
            _equipped.handlingSettings.weaponGrabMode,
            currentEquippedWeaponOwnershipKey,
            _twoHandedGrip.getGrabInputOccupancy(),
            equipped_weapon_toggle_grab_policy::GripReleaseRetention{
                .left = w.gripUpdateResult.releaseRetained.left || w.leftHolsterInput.retainGrip,
                .right = w.gripUpdateResult.releaseRetained.right || w.rightHolsterInput.retainGrip,
            });
        // A successful weapon grip owns the press that acquired it. Drain
        // that edge before normal-grab suppression can leave it pending;
        // only a later physical press may toggle this grip open.
        consumeToggleAcquisitionPress(true, toggleReconcileDecision.leftGripAcquired || w.pairedGripStartedThisFrame);
        consumeToggleAcquisitionPress(false, toggleReconcileDecision.rightGripAcquired || w.pairedGripStartedThisFrame);
        if (_twoHandedGrip.hasVisualAuthorityForHand(false)) {
            _rightHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
        }
        if (_twoHandedGrip.hasVisualAuthorityForHand(true)) {
            _leftHand.cancelGrabVisualReturn("equipped-weapon-visual-authority");
        }
        if (w.primaryOnlyGripStartedThisFrame) {
            ROCK_LOG_DEBUG(Weapon, "Equipped weapon firing-grip ownership started from grip input or held-weapon equip");
        }
    }

    void PhysicsInteraction::consumeToggleAcquisitionPress(bool isLeft, bool acquired)
    {
        if (!acquired) {
            return;
        }

        (void)_twoHandedGrip.commitPersistentEquippedCarryInputAcquisition(isLeft);

        GrabButtonState acquisitionInput{};
        if (_grabInput.firingHandButtonFrame.valid && _grabInput.firingHandButtonFrame.isLeft == isLeft) {
            acquisitionInput = GrabButtonState{
                .held = _grabInput.firingHandButtonFrame.held,
                .pressed = _grabInput.firingHandButtonFrame.pressed,
                .released = _grabInput.firingHandButtonFrame.released,
            };
            _grabInput.firingHandButtonFrame.valid = false;
        } else {
            acquisitionInput = readGrabButtonState(isLeft, input_remap_policy::kGrabButtonId);
        }

        ROCK_LOG_DEBUG(Weapon,
            "Equipped weapon toggle grab armed hand={} acquisitionPress={} physicalHeld={} physicalReleased={}",
            isLeft ? "left" : "right",
            acquisitionInput.pressed ? "yes" : "no",
            acquisitionInput.held ? "yes" : "no",
            acquisitionInput.released ? "yes" : "no");
    }

    // Attach/detach pulses from the grip update, and the continuous hover
    // pulse while a free palm sits in the firing-grip reattach zone.
    void PhysicsInteraction::queueEquippedWeaponGripHaptics()
    {
        const auto& settings = _equipped.handlingSettings;
        const auto gripHapticEvents = _twoHandedGrip.consumeHapticEvents();
        const auto queueGripHaptic = [this](const bool isLeft, const float durationSeconds, const float intensity) {
            (void)_feedbackHaptics.queue(isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                durationSeconds, intensity);
        };
        const auto queueFiringGripHaptic = [&](const bool isLeft, const bool attached) {
            const auto eventDecision = resolveEquippedWeaponDetachDecision(settings);
            if (eventDecision.authority == immersive_weapon_policy::DetachAuthority::IntegratedImmersive) {
                queueGripHaptic(isLeft, eventDecision.gripHapticDurationSeconds,
                    attached ? eventDecision.gripAttachHapticIntensity : eventDecision.gripDetachHapticIntensity);
                return;
            }
            if (settings.externalAuthorityActive) {
                queueGripHaptic(isLeft, settings.weaponGripHapticDurationSeconds,
                    attached ? settings.firingGripAttachHapticIntensity : settings.firingGripDetachHapticIntensity);
            }
        };
        if (gripHapticEvents.firingGripAttached) {
            queueFiringGripHaptic(gripHapticEvents.firingGripAttachedHandIsLeft, true);
        }
        if (gripHapticEvents.firingGripDetached) {
            queueFiringGripHaptic(gripHapticEvents.firingGripDetachedHandIsLeft, false);
        }
        if (settings.externalAuthorityActive) {
            if (gripHapticEvents.leftPartGripCaptured) {
                queueGripHaptic(true, settings.weaponGripHapticDurationSeconds, settings.supportGripHapticIntensity);
            }
            if (gripHapticEvents.rightPartGripCaptured) {
                queueGripHaptic(false, settings.weaponGripHapticDurationSeconds, settings.supportGripHapticIntensity);
            }
        }
        /*
         * Continuous hover feedback while an open free palm sits inside
         * the firing-grip reattach zone during part carry: re-queued
         * every frame so the vibration holds until the squeeze reattaches
         * (which flips the state and hands off to the firingGripAttached
         * pulse above).
         */
        if (settings.gripZoneHoverHapticsEnabled && _twoHandedGrip.isFiringGripReattachHoverInsideZone()) {
            (void)_feedbackHaptics.queue(
                _twoHandedGrip.isFiringGripReattachHoverHandLeft() ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                settings.gripZoneHoverHapticIntensity);
        }
    }

    // A shoulder sheath the gesture selected, then a drop the grip update
    // requested. A stash commit replaces the physical drop.
    void PhysicsInteraction::commitEquippedWeaponSheathAndDrop(EquippedWeaponFrameWork& w)
    {
        const auto& frame = w.frame;
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        const auto& shoulderDecision = w.shoulderFrame.decision;

        const bool nativeShoulderSheathSelected = shoulderDecision.action == equipped_weapon_shoulder::Action::SubmitSheath;
        if (nativeShoulderSheathSelected) {
            bool sheathAccepted = false;
            if (w.shoulderFrame.sourceHand != equipped_weapon_drop_policy::SourceHand::None) {
                sheathAccepted = submitEquippedWeaponShoulderSheath(
                    w.observedEquippedWeaponFormID,
                    w.observedEquippedWeaponInstanceData,
                    w.shoulderFrame.sourceHand,
                    w.shoulderFrame.detectorDecision,
                    weaponNode,
                    w.currentWeaponGenerationKey,
                    w.currentEquippedWeaponOwnershipKey);
            } else {
                ROCK_LOG_ERROR(Weapon, "Equipped shoulder coordinator selected a sheath without a source hand");
            }
            equipped_weapon_shoulder::reportExecutionResult(
                _equipped.shoulderCoordinator, equipped_weapon_shoulder::Action::SubmitSheath, sheathAccepted);
        }

        const auto equippedWeaponDropRequest = _twoHandedGrip.consumeEquippedWeaponDropRequest();
        if (!equippedWeaponDropRequest.requested) {
            return;
        }
        bool transferCommitted = false;
        // Keep the old scene alive until its presentation authorities
        // are cleared after the synchronous inventory mutation.
        RE::NiPointer<RE::NiNode> transferSourceNode(weaponNode);
        const auto sourceHand = equippedWeaponDropRequest.sourceHand;
        const bool sourceHandKnown = sourceHand == equipped_weapon_drop_policy::SourceHand::Right ||
            sourceHand == equipped_weapon_drop_policy::SourceHand::Left;
        const RE::NiPoint3 dropLoc = sourceHandKnown ?
            (equipped_weapon_drop_policy::isLeft(sourceHand) ? frame.left.grabAnchorWorld : frame.right.grabAnchorWorld) :
            (weaponNode ? weaponNode->world.translate : frame.right.grabAnchorWorld);
        if (w.inputBlockingMenuActive) {
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon manual release suppressed because an input-blocking menu is active sourceHand={} releaseLoc=({:.1f},{:.1f},{:.1f})",
                equipped_weapon_drop_policy::sourceHandName(sourceHand), dropLoc.x, dropLoc.y, dropLoc.z);
        } else {
            const bool stashCommitSelected = nativeShoulderSheathSelected || shoulderDecision.suppressEquippedDrop;
            if (equipped_weapon_drop_policy::shouldAttemptPhysicalDrop(stashCommitSelected)) {
                transferCommitted = dropEquippedWeaponToWorld(frame, equippedWeaponDropRequest, equippedWeaponDropRequest.mode);
            }
            transferCommitted = transferCommitted || stashCommitSelected;
        }
        _twoHandedGrip.completeEquippedWeaponDrop(equippedWeaponDropRequest, transferCommitted);
        w.gripUpdateResult.after = _twoHandedGrip.getGripOccupancy();
        if (transferCommitted) {
            _equipped.transition.pendingGrip() = {};
            clearEquippedWeaponFiringGripInputState();
        } else {
            (void)equipped_weapon_toggle_grab_policy::reconcile(_equipped.toggleGrabState,
                _equipped.handlingSettings.weaponGrabMode, w.currentEquippedWeaponOwnershipKey,
                _twoHandedGrip.getGrabInputOccupancy(), {
                    .left = sourceHand == equipped_weapon_drop_policy::SourceHand::Left,
                    .right = sourceHand == equipped_weapon_drop_policy::SourceHand::Right,
                });
        }
    }

    // Publishes the grip result to input remapping and the provider, and
    // updates which hands' colliders the weapon suppresses.
    PhysicsInteraction::EquippedWeaponFrameResult PhysicsInteraction::publishEquippedWeaponGripOwnership(EquippedWeaponFrameWork& w)
    {
        const auto& prepared = w.prepared;
        const auto& gripUpdateResult = w.gripUpdateResult;
        const input_remap_policy::EquippedWeaponFiringGripInputGate updatedFiringGripInputGate{
            .featureAvailable = w.firingGripOwnershipFeatureAvailable,
            .canUseFiringGripInput = _twoHandedGrip.canUseFiringGripInput(),
            .menuInputActive = w.inputBlockingMenuActive,
        };
        input_remap_runtime::setEquippedWeaponFiringGripInputActive(
            input_remap_policy::shouldUseEquippedWeaponFiringGripInput(updatedFiringGripInputGate));
        input_remap_runtime::setEquippedWeaponPrimaryDetached(_twoHandedGrip.isFiringGripDetached());
        /*
         * Left-hand fire publication: while the LEFT hand occupies the
         * firing grip, the OpenVR-level trigger remap presents the left
         * trigger to the game as the primary (right) wand's trigger.
         */
        const bool leftHandFiringActiveAfterGrip = gripUpdateResult.after.left.firingGripActive;
        input_remap_runtime::setEquippedWeaponLeftHandFiringActive(leftHandFiringActiveAfterGrip);
        ::rock::provider::setEquippedWeaponFiringHandIsLeft(_twoHandedGrip.isFiringHandLeft());
        // Seam diagnostic: every change of the primary's grip ownership.
        {
            const std::uint8_t gripState = static_cast<std::uint8_t>(_twoHandedGrip.isFiringGripOccupied()) |
                (static_cast<std::uint8_t>(_twoHandedGrip.isManualOwnershipActive()) << 1) |
                (static_cast<std::uint8_t>(_twoHandedGrip.isFiringHandLeft()) << 2) |
                (static_cast<std::uint8_t>(_twoHandedGrip.isPartCarryActive()) << 3) |
                (static_cast<std::uint8_t>(_equipped.transition.pendingGrip().pending) << 4);
            static std::uint8_t s_seamGripState = 0;
            if (gripState != s_seamGripState) {
                s_seamGripState = gripState;
                armTransitionSeamTrace("primary-grip-change");
            }
        }
        // The settled hand of the drawn primary, for readers inside menus
        // (which release the live grip). Native carry is the right hand;
        // ROCK's own ownership names its firing hand or part-carry anchor.
        // A pending equip, resume or transfer has not settled yet.
        if (!w.inputBlockingMenuActive && runtime_state::currentFrame().weaponDrawn && w.observedEquippedWeaponFormID &&
            !_equipped.transition.pendingGrip().pending && !_equipped.gripResumePending &&
            !_equipped.transition.heldTransfer().active() && !_twoHandedGrip.isHandsFreeRetained()) {
            const bool partCarry = _twoHandedGrip.isPartCarryActive();
            native_hand_equipment::recordPrimaryHand(w.observedEquippedWeaponFormID, w.observedEquippedWeaponInstanceData,
                partCarry ? _twoHandedGrip.weaponCarrierIsLeft() :
                    _twoHandedGrip.isManualOwnershipActive() && _twoHandedGrip.isFiringHandLeft(),
                !partCarry);
        }

        bool rightHandWeaponAuthorityActive = prepared.rightHandWeaponEquipped || prepared.retainedWeaponCollisionActive;
        // A visible part-carry frees the right hand even while weapon bodies exist (see beginEquippedWeaponFrame).
        if (prepared.rightHandWeaponEquipped && _twoHandedGrip.isFiringGripDetached()) {
            rightHandWeaponAuthorityActive = false;
        }
        // Left-firing carry frees the right hand the same way.
        if (prepared.rightHandWeaponEquipped && leftHandFiringActiveAfterGrip) {
            rightHandWeaponAuthorityActive = false;
        }
        if (rightHandWeaponAuthorityActive != prepared.rightHandWeaponAuthorityActive) {
            setWeaponHandSuppression(w.hknp, false, WeaponHandReason::Dominant, rightHandWeaponAuthorityActive);
        }
        const bool rightPartGripActive = gripUpdateResult.after.right.partGripActive;
        if (rightPartGripActive != gripUpdateResult.before.right.partGripActive) {
            setWeaponHandSuppression(w.hknp, false, WeaponHandReason::Grip, rightPartGripActive,
                g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        }

        /*
         * A LEFT hand occupying the firing grip is weapon-engaged exactly
         * like a support hand from the collision standpoint: its generated
         * colliders must not become a second physical owner while the
         * weapon rides the hand. Reuses the per-hand support lease.
         */
        const bool weaponSupportGripActive = gripUpdateResult.after.left.partGripActive;
        const bool leftWeaponGripActiveAfterUpdate = weaponSupportGripActive || leftHandFiringActiveAfterGrip;
        setWeaponHandSuppression(w.hknp, true, WeaponHandReason::Grip, leftWeaponGripActiveAfterUpdate,
            gripUpdateResult.before.left.weaponEngaged() ? g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds : 0.0f);

        return EquippedWeaponFrameResult{
            .rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActive,
            .leftSupportGripActive = weaponSupportGripActive,
            .rightPartGripActive = rightPartGripActive,
        };
    }

    // The dynamic weapon collision frame: its contact witness log, and the
    // visual correction it requests for the weapon.
    void PhysicsInteraction::finishDynamicWeaponCollision(EquippedWeaponFrameWork& w)
    {
        const auto& frame = w.frame;
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        const auto currentWeaponGenerationKey = w.currentWeaponGenerationKey;

        RE::NiPoint3 surfaceSupportPrimaryGripLocal{};
        const bool surfaceSupportPrimaryValid =
            _twoHandedGrip.tryGetSurfaceSupportPrimaryGripLocal(weaponNode, currentWeaponGenerationKey, surfaceSupportPrimaryGripLocal);
        const auto dynamicWeaponFrame = _dynamicWeaponCollision.finishFrame(
            frame,
            physicsWritesAllowedForWorld(frame.hknpWorld),
            weaponNode,
            currentWeaponGenerationKey,
            _weaponCollision,
            surfaceSupportPrimaryValid ? &surfaceSupportPrimaryGripLocal : nullptr,
            _twoHandedGrip.isSurfaceSupported());
        if (_dynamicWeaponCollision.compoundSourcesUnavailable()) {
            _weaponCollision.requestRebuildForReplacedSources();
        }
        const bool dynamicWeaponDebugEnabled = g_rockConfig.rockDebugShowColliders && g_rockConfig.rockDebugDrawDynamicWeaponColliders;
        if (dynamicWeaponFrame.contactEpisodeStarted && dynamicWeaponDebugEnabled) {
            auto* otherRef = resolveBodyToRef(frame.bhkWorld, frame.hknpWorld, RE::hknpBodyId{ dynamicWeaponFrame.otherBodyId });
            const auto* otherBase = otherRef ? otherRef->GetObjectReference() : nullptr;
            const auto otherNameView = otherBase ? RE::TESFullName::GetFullName(*otherBase, false) : std::string_view{};
            const std::string otherName = otherNameView.empty() ? std::string("(unresolved)") : std::string(otherNameView);
            const char* otherType = otherBase ? otherBase->GetFormTypeString() : "unresolved";

            WeaponCollision::WeaponSurfaceProximityWitness partWitness{};
            WeaponInteractionDebugInfo partInfo{};
            constexpr float kContactPartSearchRadiusGameUnits = 24.0f;
            const bool partWitnessValid = dynamicWeaponFrame.rawContactPointValid &&
                _weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(
                    weaponNode, dynamicWeaponFrame.rawContactPointGame, kContactPartSearchRadiusGameUnits, partWitness);
            const bool partInfoValid = partWitnessValid && _weaponCollision.tryGetWeaponContactDebugInfo(partWitness.bodyId, partInfo);

            ROCK_LOG_INFO(
                Weapon,
                "DWC contact witness: episode={} solveAge={} generation={:016X} weaponForm={:08X} other(body/layer/motion/ref/form/type/name)=({}/{}/{}/{:p}/{:08X}/{}/{}) native(collObj/owner)=({:p}/{:p}) raw(valid/proxyWasA/points/index/weight)={}/{}/{}/{}/{:.3f} pointGame=({:.2f},{:.2f},{:.2f}) normalRaw=({:.3f},{:.3f},{:.3f}) nearestPart(valid/body/source/distance/current)={}/{}/{}/{:.3f}/{}",
                dynamicWeaponFrame.contactEpisode,
                dynamicWeaponFrame.contactSolveAge,
                currentWeaponGenerationKey,
                _weaponCollision.getCurrentObservedEquippedWeaponFormID(),
                dynamicWeaponFrame.otherBodyId,
                dynamicWeaponFrame.otherLayer,
                dynamicWeaponFrame.otherMotionIndex,
                static_cast<void*>(otherRef),
                otherRef ? otherRef->GetFormID() : 0,
                otherType,
                otherName,
                reinterpret_cast<void*>(dynamicWeaponFrame.otherCollisionObject),
                reinterpret_cast<void*>(dynamicWeaponFrame.otherOwnerNode),
                dynamicWeaponFrame.rawContactPointValid,
                dynamicWeaponFrame.rawContactProxyWasBodyA,
                dynamicWeaponFrame.rawContactPointCount,
                dynamicWeaponFrame.rawContactPointIndex,
                dynamicWeaponFrame.rawContactPointWeightSum,
                dynamicWeaponFrame.rawContactPointGame.x,
                dynamicWeaponFrame.rawContactPointGame.y,
                dynamicWeaponFrame.rawContactPointGame.z,
                dynamicWeaponFrame.rawContactNormalHavok.x,
                dynamicWeaponFrame.rawContactNormalHavok.y,
                dynamicWeaponFrame.rawContactNormalHavok.z,
                partWitnessValid,
                partWitnessValid ? partWitness.bodyId : 0x7FFF'FFFFu,
                partInfoValid ? partInfo.sourceName : std::string("(unresolved)"),
                partWitnessValid ? partWitness.distanceGameUnits : -1.0f,
                partWitnessValid && partWitness.sourceNodeCurrent);
            ROCK_LOG_INFO(
                Weapon,
                "DWC contact transforms: episode={} requestedBody=({:.2f},{:.2f},{:.2f}) liveBody=({:.2f},{:.2f},{:.2f}) otherReadable={} otherBody=({:.2f},{:.2f},{:.2f}) correction=({:.2f}gu,{:.2f}deg)",
                dynamicWeaponFrame.contactEpisode,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.x,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.y,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.z,
                dynamicWeaponFrame.liveContactBodyWorld.translate.x,
                dynamicWeaponFrame.liveContactBodyWorld.translate.y,
                dynamicWeaponFrame.liveContactBodyWorld.translate.z,
                dynamicWeaponFrame.otherBodyWorldValid,
                dynamicWeaponFrame.otherBodyWorld.translate.x,
                dynamicWeaponFrame.otherBodyWorld.translate.y,
                dynamicWeaponFrame.otherBodyWorld.translate.z,
                dynamicWeaponFrame.translationCorrectionGameUnits,
                dynamicWeaponFrame.rotationCorrectionDegrees);
        }
        if (!dynamicWeaponFrame.applyVisualCorrection) {
            return;
        }
        const bool visualPublishSucceeded = _twoHandedGrip.applyWeaponCollisionResolvedAuthority(
            weaponNode, dynamicWeaponFrame.resolvedWeaponWorld, currentWeaponGenerationKey);
        if (dynamicWeaponDebugEnabled) {
            const bool measured = visualPublishSucceeded && weaponNode;
            const float immediateTranslationError = measured ?
                dynamic_weapon_collision_policy::translationDeltaGameUnits(weaponNode->world, dynamicWeaponFrame.resolvedWeaponWorld) : -1.0f;
            const float immediateRotationError = measured ?
                dynamic_weapon_collision_policy::rotationDeltaDegrees(weaponNode->world, dynamicWeaponFrame.resolvedWeaponWorld) : -1.0f;
            ROCK_LOG_SAMPLE_INFO(
                Weapon,
                500,
                "DWC visual publication: bodyActive={} publishSucceeded={} immediateError=({:.3f}gu,{:.3f}deg) requestedCorrection=({:.3f}gu,{:.3f}deg)",
                dynamicWeaponFrame.proxyActive,
                visualPublishSucceeded,
                immediateTranslationError,
                immediateRotationError,
                dynamicWeaponFrame.translationCorrectionGameUnits,
                dynamicWeaponFrame.rotationCorrectionDegrees);
        }
    }

    void PhysicsInteraction::publishGripZoneIndicatorRenderFrame(
        const std::uint64_t gameFrameIndex)
    {
        const auto& runtime = runtime_state::currentFrame();
        if (!_lifecycle.initialized.load(std::memory_order_acquire) ||
            !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking ||
            runtime.compatibilityConfigBlocking || gameFrameIndex == 0 ||
            gameFrameIndex != runtime.frameIndex ||
            _frame.gripZoneIndicatorFrameIndex != gameFrameIndex) {
            grip_zone_indicators::Clear();
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        auto* hknp = bhk ? getHknpWorld(bhk) : nullptr;
        if (!bhk || bhk != _lifecycle.cachedBhkWorld || !hknp || hknp != _lifecycle.cachedHknpWorld) {
            grip_zone_indicators::Clear();
            return;
        }

        grip_zone_indicators::Frame overlayFrame{};
        overlayFrame.gameFrameIndex = gameFrameIndex;
        overlayFrame.diameterGameUnits =
            g_rockConfig.rockGripZoneIndicatorDiameterGameUnits;
        overlayFrame.count += authored_interaction::appendIndicators(
            std::span(overlayFrame.positions).subspan(overlayFrame.count));

        if (_powerArmorCandidateFrame == gameFrameIndex) {
            for (const bool isLeft : {false, true}) {
                const auto& candidate = _powerArmorCandidates[isLeft ? 1u : 0u];
                if (candidate.valid && !_touchGrabRuntime.isHandActive(isLeft) &&
                    !(isLeft ? _leftHand : _rightHand).isHolding()) {
                    overlayFrame.positions[overlayFrame.count++] = candidate.positionGame;
                }
            }
        }

        for (const bool isLeft : {false, true}) {
            const auto& hand = isLeft ? _leftHand : _rightHand;
            if (!hand.isHolding() && hand.hasSelection() && !_touchGrabRuntime.isHandActive(isLeft)) {
                overlayFrame.count += static_cast<std::uint32_t>(loose_weapon_grip_zone::collectIndicators(
                    isLeft, hand.getSelection().refr,
                    std::span<RE::NiPoint3>(overlayFrame.positions).subspan(overlayFrame.count)));
            }
        }

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        const std::uint64_t currentWeaponGenerationKey =
            _weaponCollision.getCurrentWeaponGenerationKey();
        if (!weaponNode || !f4vr::isNodeVisible(weaponNode) ||
            currentWeaponGenerationKey == 0 ||
            !dynamic_weapon_collision_policy::isFiniteTransform(
                weaponNode->world) ||
            std::abs(weaponNode->world.scale) <= 0.0001f) {
            grip_zone_indicators::Publish(overlayFrame);
            return;
        }

        const auto appendIndicator = [&](const auto& indicatorFrame,
                                         const Hand& hand) {
            if (!indicatorFrame.visible || hand.isHolding() ||
                !indicatorFrame.weaponLocalValid ||
                indicatorFrame.weaponGenerationKey !=
                    currentWeaponGenerationKey ||
                overlayFrame.count >= overlayFrame.positions.size()) {
                return;
            }

            const RE::NiPoint3 positionWorld =
                transform_math::localPointToWorld(
                    weaponNode->world,
                    indicatorFrame.positionWeaponLocal);
            if (!std::isfinite(positionWorld.x) ||
                !std::isfinite(positionWorld.y) ||
                !std::isfinite(positionWorld.z)) {
                return;
            }
            overlayFrame.positions[overlayFrame.count++] = positionWorld;
        };

        for (const bool isLeft : { true, false }) {
            appendIndicator(_twoHandedGrip.getAuthoredSupportGripIndicatorFrame(isLeft),
                isLeft ? _leftHand : _rightHand);
        }

        const auto firingReattachFrame =
            _twoHandedGrip.getFiringGripReattachIndicatorFrame();
        appendIndicator(
            firingReattachFrame,
            firingReattachFrame.handIsLeft ? _leftHand : _rightHand);

        if (_equipped.partIndicatorFrame == gameFrameIndex &&
            _equipped.partIndicatorGeneration == currentWeaponGenerationKey) {
            auto candidateBodyIds = _equipped.partIndicatorBodyIds;
            const auto roster = equippedWeaponRoster();
            const auto& slotZero = roster.facts().slot0;
            for (const bool isLeft : { true, false }) {
                const auto& hand = isLeft ? _leftHand : _rightHand;
                if (hand.isHolding() || _touchGrabRuntime.isHandActive(isLeft) ||
                    roster.slotZeroGrips(isLeft).weaponEngaged() ||
                    (slotZero.sessionFiringHand == equipped_weapon_roster::handFor(isLeft) &&
                        !slotZero.firingGripDetached)) {
                    candidateBodyIds[isLeft ? 0u : 1u] = weapon_part_runtime::kInvalidBodyId;
                }
            }
            if (candidateBodyIds[0] != weapon_part_runtime::kInvalidBodyId ||
                candidateBodyIds[1] != weapon_part_runtime::kInvalidBodyId) {
                std::array<weapon_part_runtime::Target,
                    api::weaponparts::kMaxTargets> partTargets{};
                const auto targetCount = provider::copyWeaponPartTargets(partTargets);
                overlayFrame.count += static_cast<std::uint32_t>(
                    _weaponCollision.collectAttachOnlyGripIndicators(weaponNode,
                        std::span(partTargets).first(targetCount), candidateBodyIds,
                        std::span(overlayFrame.positions).subspan(overlayFrame.count),_equipped.partIndicatorPoints));
            }
        }

        grip_zone_indicators::Publish(overlayFrame);
    }

    /*
     * AfterWeaponPosition. FRIK's weapon pass writes the whole Weapon local
     * (offset and scale) after ROCK's frame, so the presentation baseline is
     * re-applied here: the render sees it, and the offset latch taken next
     * carries it into ROCK's next frame. Animation graph scale and an explicit
     * Weapon animation owner remain authoritative over this default. A
     * transition can remain active after the exact model is visible; waiting
     * for completion leaves its first equipped frames undersized.
     */
    void PhysicsInteraction::normalizeWeaponPresentationScaleAfterFrikWeaponPass()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        auto* equippedWeapon = currentEquippedWeaponForm();
        if (!weaponNode || !equippedWeapon ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking ||
            !runtime.weaponDrawn) {
            return;
        }
        // Same freshness boundary as the authored grip: the richer
        // stack/instance key when generated collision is on, the equipped form otherwise.
        std::uint64_t weaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        if (weaponOwnershipKey == 0) {
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }
        const std::uint64_t weaponGenerationKey =
            authored_support_grab_policy::resolveAuthoredGenerationKey(
                _weaponCollision.getCurrentWeaponGenerationKey(),
                weaponOwnershipKey);
        const bool equippedGenerationMatchesForm =
            weaponGenerationKey != 0 &&
            weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;
        const auto nativeAuthorityFlags = provider::currentNativeAnimationAuthorityFlags();
        if (!equippedGenerationMatchesForm || !weaponNode->parent || !f4vr::isNodeVisible(weaponNode) ||
            (nativeAuthorityFlags & authored_weapon_grip_capture_policy::kWeapon) != 0) {
            return;
        }
        const float parentScale = weaponNode->parent->world.scale;
        float animationScale = 1.0f;
        const bool animationAvailable = authored_weapon_grip_capture::tryGetAnimationWeaponScale(
            weaponNode, equippedWeapon->formID, animationScale);
        const float desiredScale = authored_weapon_grip_capture_policy::resolveWeaponPresentationScale(animationAvailable, animationScale);
        if (std::isfinite(parentScale) && std::abs(parentScale) > 0.0001f &&
            std::abs(weaponNode->world.scale - desiredScale) > 0.00001f) {
            const float previousScale = weaponNode->world.scale;
            weaponNode->local.scale = desiredScale / parentScale;
            f4vr::updateDown(weaponNode, true);
            ROCK_LOG_SAMPLE_INFO(Animation, 2000,
                "Weapon presentation scale form={:08X} incoming={:.6f} target={:.6f} result={:.6f} source={}",
                equippedWeapon->formID, previousScale, desiredScale, weaponNode->world.scale,
                animationAvailable ? "animation-graph" : "unit-default");
        }
    }

    void PhysicsInteraction::synchronizeEquipVisualBridgeAfterFrikWeaponPass()
    {
        const auto& runtime = runtime_state::currentFrame();
        if (!_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking) return;
        // The native Weapon may still be hidden by draw animation or the
        // bridge itself. Its accepted grip solve nevertheless owns both hands.
        _equipped.transition.synchronizeEquippedPresentation(
            resolveEquippedWeaponInteractionNode(), currentEquippedWeaponFormId());
    }

    void PhysicsInteraction::finalizeEquipVisualBridgePresentation()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* player = f4vr::getPlayer();
        if (!player || !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking ||
            provider::currentNativeAnimationAuthorityFlags() != 0 ||
            f4vr::getNativeGunState(player) == static_cast<std::uint32_t>(RE::GUN_STATE::kReloading)) return;
        _equipped.transition.finalizeVisualPresentation();
    }

    void PhysicsInteraction::updateAuthoredPrimaryFiringGrip()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        // The authored/native canonical weapon frame is always ROCK's
        // physical-right primary controller, independent of FO4VR settings.
        const bool leftHandHoldingObject = _leftHand.isHolding();
        const bool rightHandHoldingObject = _rightHand.isHolding();
        _twoHandedGrip.setGrabbedObjectHandPoseOwnership(
            leftHandHoldingObject,
            rightHandHoldingObject);
        const auto nativeAuthorityFlags =
            provider::currentNativeAnimationAuthorityFlags();
        auto* equippedWeapon = currentEquippedWeaponForm();
        const auto weaponClassification =
            _weaponCollision.getEquippedWeaponClassification();
        const std::uint64_t weaponKeywordFlags =
            equippedWeapon &&
                weaponClassification.hasEquippedWeapon &&
                weaponClassification.formID == equippedWeapon->formID ?
            weaponClassification.keywordFlags :
            0;
        std::uint64_t weaponOwnershipKey =
            weaponNode ? _weaponCollision.getCurrentEquippedWeaponOwnershipKey() : 0;
        if (weaponNode && weaponOwnershipKey == 0) {
            // Keep authored grip alignment independent of generated weapon
            // collision.
            // The richer stack/instance key wins when available; the equipped
            // form remains a stable freshness boundary when collision is off.
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }
        const std::uint64_t collisionGenerationKey =
            weaponNode ? _weaponCollision.getCurrentWeaponGenerationKey() : 0;
        const std::uint64_t weaponGenerationKey =
            authored_support_grab_policy::resolveAuthoredGenerationKey(
                collisionGenerationKey,
                weaponOwnershipKey);

        const bool equippedGenerationMatchesForm =
            weaponNode &&
            equippedWeapon &&
            weaponGenerationKey != 0 &&
            weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;
        const bool equippedWeaponTransitionActive =
            _equipped.transition.getPublicSnapshot().active;

        native_idle_grip_preharvest::observeEquippedWeapon(
            equippedGenerationMatchesForm ? equippedWeapon : nullptr,
            equippedGenerationMatchesForm ? weaponNode : nullptr,
            equippedGenerationMatchesForm ? currentEquippedWeaponInstanceData(equippedWeapon) : nullptr,
            equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0);

        _twoHandedGrip.beginAuthoredPrimaryFiringGripFrame();
        const auto* equippedInstance = currentEquippedWeaponInstanceData(equippedWeapon);
        const auto& pendingGrip = _equipped.transition.pendingGrip();
        // Prepare the accepted left firing seat before acquisition. Waiting
        // for the live firing role would require right-hand alignment first,
        // which an outgoing loose weapon correctly prevents.
        const bool pendingLeftFiringGrip = pendingGrip.pending && pendingGrip.isLeft &&
            !pendingGrip.supportGrip.validCarry() &&
            equipped_weapon_transition_policy::matchesExpectedIdentity(
                equippedWeapon ? equippedWeapon->formID : 0u,
                reinterpret_cast<std::uintptr_t>(equippedInstance),
                pendingGrip.targetWeaponFormID, pendingGrip.targetWeaponInstanceData,
                pendingGrip.previousWeaponFormID, pendingGrip.previousWeaponInstanceData);
        const auto* effectiveWeaponData = equippedInstance ?
            static_cast<const RE::TESObjectWEAP::InstanceData*>(equippedInstance) :
            equippedWeapon ? &equippedWeapon->weaponData : nullptr;
        frik_hand_world_authority::prepareWeaponInputs(weaponOwnershipKey, weaponGenerationKey,
            effectiveWeaponData && effectiveWeaponData->type.get() == RE::WEAPON_TYPE::kGun &&
            !(nativeAuthorityFlags & authored_weapon_grip_capture_policy::kWeapon));
        if (frik_hand_world_authority::weaponFilteringActive()) {
            // Authored grip alignment precedes the ordinary interaction pass.
            // Resolve this frame's isolated input before its first filter read.
            refreshHandBoneCache();
        }
        _equipped.authoredPrimaryFiringGrip.update(AuthoredPrimaryFiringGripFrameInput{
            .weaponNode = weaponNode,
            .weapon = equippedWeapon,
            .weaponOwnershipKey = weaponOwnershipKey,
            .weaponGenerationKey = weaponGenerationKey,
            .weaponInstanceContentKey = equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0,
            .weaponKeywordFlags = weaponKeywordFlags,
            .weaponInstanceContentKnown = equippedGenerationMatchesForm,
            .meleeWeapon = (effectiveWeaponData && weapon_type_policy::isEquippedMelee(effectiveWeaponData->type.get())) ||
                (equippedWeapon && weaponClassification.formID == equippedWeapon->formID &&
                    weaponClassification.classificationResolved && weaponClassification.sizeClass == WeaponSizeClass::Melee),
            .runtimeInitialized = _lifecycle.initialized.load(std::memory_order_acquire),
            .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
            .localSkeletonReady = runtime.localSkeletonReady,
            .menuBlocking = runtime.localMenuBlocking,
            .compatibilityBlocking = runtime.compatibilityConfigBlocking,
            .weaponDrawn = runtime.weaponDrawn,
            .weaponVisible = weaponNode && f4vr::isNodeVisible(weaponNode),
            // Arms/hands-only manual cycling must retain ROCK's authored
            // weapon-to-controller alignment. Only a native Weapon transform
            // lease (the full reload path) suspends that owner.
            .nativeReloadAuthorityActive =
                (nativeAuthorityFlags &
                    authored_weapon_grip_capture_policy::kWeapon) != 0,
            .conflictingWeaponTransformAuthorityActive =
                _equipped.gripResumePending || _equipped.transition.pendingGrip().menuResume ||
                _twoHandedGrip.blocksAuthoredPrimaryGripWeaponAlignment(),
            .weaponVisualReturnActive = _twoHandedGrip.isWeaponVisualReturnActive(),
            .equippedWeaponTransitionActive =
                equippedWeaponTransitionActive,
            .primaryHandHoldingObject = rightHandHoldingObject,
            .rockFiringHandIsLeft = _twoHandedGrip.isFiringHandLeft(),
            .pendingLeftFiringGrip = pendingLeftFiringGrip,
            .inPowerArmor = f4vr::isInPowerArmor(),
        }, _twoHandedGrip);
        _twoHandedGrip.finishAuthoredPrimaryFiringGripFrame();
        if (_equipped.transition.isHandPoseHandoffActive()) {
            const bool handoffHandIsLeft = _equipped.transition.handPoseHandoffIsLeft();
            const bool pairedSupportOwned = !_twoHandedGrip.isFiringGripDetached() &&
                _twoHandedGrip.isHandPartGripping(!handoffHandIsLeft);
            if (nativeAuthorityFlags != 0 ||
                runtime.localMenuBlocking ||
                runtime.compatibilityConfigBlocking) {
                _equipped.transition.completeHandPoseHandoff("authored-pose-unavailable");
            } else if (equippedWeapon && equippedWeapon->formID != _equipped.transition.bridgeWeaponBaseFormID()) {
                _equipped.transition.completeHandPoseHandoff("equipped-weapon-changed");
            } else if (_equipped.transition.hasPairedHandPoseHandoff() &&
                !_equipped.transition.pendingGrip().pairedGrips.valid() && !pairedSupportOwned) {
                _equipped.transition.completeHandPoseHandoff("paired-transfer-ended");
            } else if (_equipped.transition.hasSupportOnlyHandPoseHandoff() &&
                !_equipped.transition.pendingGrip().supportGrip.validCarry() &&
                !_twoHandedGrip.isHandPartGripping(handoffHandIsLeft)) {
                _equipped.transition.completeHandPoseHandoff("support-transfer-ended");
            }
        }
    }

    void PhysicsInteraction::finishEquippedWeaponHandPoseHandoff()
    {
        if (!_equipped.transition.isHandPoseHandoffActive()) return;
        const bool handIsLeft = _equipped.transition.handPoseHandoffIsLeft();
        const bool pairedSupportOwned = !_twoHandedGrip.isFiringGripDetached() &&
            _twoHandedGrip.isHandPartGripping(!handIsLeft);
        const bool supportGripOwned = _equipped.transition.hasSupportOnlyHandPoseHandoff() &&
            _twoHandedGrip.isHandPartGripping(handIsLeft);
        if ((_twoHandedGrip.hasPublishedAuthoredPrimaryFiringGripFingerPose(handIsLeft) || supportGripOwned) &&
            _twoHandedGrip.hasVisualAuthorityForHand(handIsLeft) &&
            !_twoHandedGrip.isHandVisualReturnActive(handIsLeft) &&
            (!_equipped.transition.hasPairedHandPoseHandoff() || pairedSupportOwned)) {
            _equipped.transition.tryCompleteHandPoseHandoff();
            if (!_equipped.transition.isHandPoseHandoffActive()) _equipped.transition.recordGripPresentation();
        }
    }

    void PhysicsInteraction::refreshEquippedWeaponHandlingSettings()
    {
        // Frame updates begin after GameLoaded, when F4SE has loaded every
        // plugin. Cache module presence for this process without polling or
        // changing the user's configuration on reloads or new game sessions.
        const bool virtualHolstersLoaded = virtual_holsters::isLoaded();
        // Virtual Holsters owns weapon stowing when it is loaded.
        if (virtualHolstersLoaded && g_rockConfig.rockEquippedWeaponShoulderStashEnabled &&
            !_equipped.virtualHolstersStashOverrideLogged) {
            _equipped.virtualHolstersStashOverrideLogged = true;
            ROCK_LOG_INFO(Weapon, "Virtual Holsters is loaded; equipped-weapon shoulder stash is off and Virtual Holsters stows weapons");
        }

        api::weapon::HandlingRequest request{};
        const bool externalAuthorityActive =
            ::rock::provider::getHandlingAuthorityRequest(request);
        const auto weaponClassification = _weaponCollision.getEquippedWeaponClassification();
        const RockEquippedWeaponHandlingBaseline rockBaseline{
            .leftHandedModeEnabled = g_rockConfig.rockLeftHandedMode,
            // Firing belongs to whichever hand holds the firing grip; the
            // handoff is the general rule, not an option.
            .ambidextrousHandoffEnabled = true,
            .authoredOnlySupportGrabsEnabled =
                !g_rockConfig.rockGrabAnywhereOnWeapon,
            .meleeWeapon = weaponClassification.classificationResolved &&
                weaponClassification.sizeClass == WeaponSizeClass::Melee,
            .weaponGrabMode =
                equipped_weapon_toggle_grab_policy::fromSetting(g_rockConfig.rockWeaponGrabMode),
            .equippedWeaponShoulderStashEnabled =
                g_rockConfig.rockEquippedWeaponShoulderStashEnabled &&
                !virtualHolstersLoaded,
            .lastGripReleaseDropEnabled =
                equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode) != equipped_weapon_drop_policy::Mode::Off,
            .immersiveWeapon = {
                .firingGripDetachEnabled =
                    g_rockConfig.
                        rockDetachEitherHand,
                .firingGripReattachRadiusGameUnits =
                    g_rockConfig.
                        rockFiringGripReattachRadiusGameUnits,
                .firingGripHapticDurationSeconds =
                    g_rockConfig.
                        rockFiringGripHapticDurationSeconds,
                .firingGripAttachHapticIntensity =
                    g_rockConfig.
                        rockFiringGripAttachHapticIntensity,
                .firingGripDetachHapticIntensity =
                    g_rockConfig.
                        rockFiringGripDetachHapticIntensity,
            },
            .firingGripReattachCylinderRadiusGameUnits =
                g_rockConfig.rockFiringGripReattachCylinderRadiusGameUnits,
            .firingGripProximitySupportRadiusGameUnits =
                g_rockConfig.rockFiringGripProximitySupportRadius,
            .leftFiringAimYawDegrees =
                g_rockConfig.rockLeftFiringAimYawDegrees,
            .leftFiringAimPitchDegrees =
                g_rockConfig.rockLeftFiringAimPitchDegrees,
            .leftFiringAimOffsetXGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetXGameUnits,
            .leftFiringAimOffsetYGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetYGameUnits,
            .leftFiringAimOffsetZGameUnits =
                g_rockConfig.rockLeftFiringAimOffsetZGameUnits,
        };
        auto settings = makeEquippedWeaponHandlingSettings(
            rockBaseline,
            externalAuthorityActive ? &request : nullptr);

        if (_equipped.handlingModeInitialized) {
            if (requiresEquippedWeaponHandlingModeReconcile(
                    _equipped.handlingSettings,
                    settings)) {
                _equipped.handlingModeReconcilePending = true;
            }
        }

        _equipped.handlingSettings = settings;
        _equipped.handlingModeInitialized = true;
        equipped_weapon_handling_runtime::publish(settings);
    }

    void PhysicsInteraction::reconcileEquippedWeaponHandlingMode()
    {
        if (!_equipped.handlingModeReconcilePending) {
            return;
        }

        _twoHandedGrip.restoreNativeRightEquippedCarry(
            "equipped-weapon-handling-mode-changed");
        _equipped.transition.pendingGrip() = {};
        _equipped.handlingModeReconcilePending = false;
    }

    bool PhysicsInteraction::submitEquippedWeaponShoulderSheath(
        const std::uint32_t observedWeaponFormID,
        const std::uintptr_t observedWeaponInstanceData,
        const equipped_weapon_drop_policy::SourceHand sourceHand,
        const shoulder_stash::Decision& stashDecision,
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        const bool stashHandIsLeft =
            equipped_weapon_drop_policy::isLeft(sourceHand);
        const std::size_t stashHandIndex = stashHandIsLeft ? 1u : 0u;
        native_equipped_weapon_draw::Identity sheathIdentity{};
        const bool identityCaptured =
            native_equipped_weapon_draw::captureCurrentIdentity(
                sheathIdentity);
        const bool identityMatchesObserved =
            identityCaptured &&
            sheathIdentity.formID == observedWeaponFormID &&
            sheathIdentity.instanceData == observedWeaponInstanceData;
        RE::NiTransform leftFiringHandWeaponLocal{};
        RE::NiPoint3 leftFiringGripWeaponLocal{};
        const bool hasLeftFiringGripTransfer =
            identityMatchesObserved &&
            _twoHandedGrip.tryCaptureLeftFiringGripTransfer(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                leftFiringHandWeaponLocal,
                leftFiringGripWeaponLocal);
        native_equipped_weapon_draw::Result sheathResult{};
        sheathResult.result = identityCaptured ?
            native_equipped_weapon_draw::SubmitResult::IdentityChanged :
            native_equipped_weapon_draw::SubmitResult::MissingEquippedWeapon;
        if (identityMatchesObserved) {
            sheathResult = native_equipped_weapon_draw::
                submitSheatheExactCurrent(sheathIdentity);
        }
        const bool sheathAccepted =
            (sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::Submitted ||
                sheathResult.result ==
                    native_equipped_weapon_draw::SubmitResult::
                        AlreadySheathingOrSheathed) &&
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(
                    sheathResult.stateAfter);
        if (sheathAccepted) {
            equipped_weapon_toggle_grab_policy::reset(
                _equipped.toggleGrabState);
            _twoHandedGrip.restoreNativeRightEquippedCarry(
                "shoulder-weapon-sheathed");
            _equipped.transition.pendingGrip() = {};
            _equipped.shoulderSheath =
                EquippedWeaponShoulderSheathState{
                    .active = true,
                    .stashedByLeftHand = stashHandIsLeft,
                    .weaponFormID = sheathIdentity.formID,
                    .weaponInstanceData = sheathIdentity.instanceData,
                    .equipIndex = sheathIdentity.equipIndex,
                    .weaponOwnershipKey =
                        currentEquippedWeaponOwnershipKey,
                    .zone = stashDecision.zone,
                    .hasLeftFiringGripTransfer =
                        hasLeftFiringGripTransfer,
                    .leftFiringHandWeaponLocal =
                        leftFiringHandWeaponLocal,
                    .leftFiringGripWeaponLocal =
                        leftFiringGripWeaponLocal,
                };
            input_remap_runtime::setEquippedWeaponShoulderSheathActive(true);
            _equipped.sheathRetrievalStates = {};
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon shoulder sheathed formID={:08X} instance={:#x} equipIndex={} sourceHand={} zone={} confidence={:.2f} leftTransfer={} state={}({})->{}({}) result={}",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                sheathIdentity.equipIndex,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                body_zone::bodyZoneName(stashDecision.zone),
                stashDecision.confidence,
                hasLeftFiringGripTransfer ? "captured" : "unavailable",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
            if (g_rockConfig.rockShoulderStashHapticsEnabled) {
                (void)_feedbackHaptics.queue(
                    stashHandIsLeft ? feedback_haptics::FeedbackHand::Left :
                                      feedback_haptics::FeedbackHand::Right,
                    g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                    g_rockConfig.rockShoulderStashCommitHapticIntensity);
            }
            if (g_rockConfig.rockShoulderStashShowCollectedNotifications) {
                f4vr::showNotification(
                    shoulder_stash_notification_policy::
                        formatStowedNotification(
                            shoulderStashItemName(
                                currentEquippedWeaponForm()),
                            sheathIdentity.formID));
            }
        } else {
            ROCK_LOG_WARN(Weapon,
                "Equipped weapon shoulder sheathe failed formID={:08X} instance={:#x} sourceHand={} identityMatch={} state={}({})->{}({}) result={} -- weapon stays equipped and physical drop is suppressed",
                sheathIdentity.formID,
                sheathIdentity.instanceData,
                equipped_weapon_drop_policy::sourceHandName(sourceHand),
                identityMatchesObserved ? "yes" : "no",
                sheathResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateBefore),
                sheathResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    sheathResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    sheathResult.result));
        }

        shoulder_stash::resetRuntime(
            _equipped.stashStates[stashHandIndex]);
        return sheathAccepted;
    }

    void PhysicsInteraction::clearEquippedWeaponShoulderSheath(
        const char* reason,
        const bool resetCoordinator)
    {
        if (_equipped.shoulderSheath.active) {
            ROCK_LOG_INFO(
                Weapon,
                "Equipped weapon shoulder sheath cleared reason={} formID={:08X} instance={:#x} zone={}",
                reason ? reason : "unknown",
                _equipped.shoulderSheath.weaponFormID,
                _equipped.shoulderSheath.weaponInstanceData,
                body_zone::bodyZoneName(
                    _equipped.shoulderSheath.zone));
        }
        _equipped.shoulderSheath = {};
        input_remap_runtime::setEquippedWeaponShoulderSheathActive(false);
        _equipped.sheathRetrievalStates = {};
        if (resetCoordinator) {
            equipped_weapon_shoulder::reset(
                _equipped.shoulderCoordinator);
        }
    }

    PhysicsInteraction::EquippedWeaponShoulderFrameResult
        PhysicsInteraction::advanceEquippedWeaponShoulderCoordinator(
            const PhysicsFrameContext& frame,
            const bool handlingEnabled,
            const bool menuInputActive,
            const std::uint32_t observedWeaponFormID,
            const std::uintptr_t observedWeaponInstanceData,
            RE::NiNode* weaponNode,
            const std::uint64_t currentEquippedWeaponOwnershipKey,
            const bool firingHandIsLeft)
    {
        EquippedWeaponShoulderFrameResult result{};
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeWeaponState =
            f4vr::getNativeWeaponState(player);
        const auto nativePresentation =
            shoulderNativePresentation(nativeWeaponState);

        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool capturedCurrentIdentity =
            native_equipped_weapon_draw::captureCurrentIdentity(
                currentIdentity);
        if (_equipped.shoulderSheath.active) {
            const bool identityMatches = capturedCurrentIdentity &&
                currentIdentity.formID ==
                    _equipped.shoulderSheath.weaponFormID &&
                currentIdentity.instanceData ==
                    _equipped.shoulderSheath.weaponInstanceData &&
                currentIdentity.equipIndex ==
                    _equipped.shoulderSheath.equipIndex &&
                observedWeaponFormID == currentIdentity.formID &&
                observedWeaponInstanceData == currentIdentity.instanceData;
            if (!handlingEnabled) {
                clearEquippedWeaponShoulderSheath(
                    "shoulder-stash-authority-lost");
            } else if (!identityMatches) {
                clearEquippedWeaponShoulderSheath(
                    "equipped-weapon-identity-changed");
            } else if (nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::Invalid ||
                       nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::WantDraw ||
                       nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::Drawing ||
                       nativePresentation ==
                           equipped_weapon_shoulder::NativePresentation::StableDrawn) {
                clearEquippedWeaponShoulderSheath(
                    nativePresentation ==
                            equipped_weapon_shoulder::NativePresentation::Invalid ?
                        "invalid-native-weapon-state" :
                        "native-weapon-no-longer-sheathed");
            }
        }

        auto leftPhysicalGrip = peekGrabButtonState(
            true,
            input_remap_policy::kGrabButtonId);
        auto rightPhysicalGrip = peekGrabButtonState(
            false,
            input_remap_policy::kGrabButtonId);
        leftPhysicalGrip.held =
            input_remap_runtime::isRawButtonPhysicallyHeld(
                true,
                input_remap_policy::kGrabButtonId);
        rightPhysicalGrip.held =
            input_remap_runtime::isRawButtonPhysicallyHeld(
                false,
                input_remap_policy::kGrabButtonId);
        const auto providerGripOccupancy = equippedWeaponRoster().slotZeroGrips();
        maskProviderWeaponGrabInput(true, providerGripOccupancy.left.weaponEngaged(), leftPhysicalGrip);
        maskProviderWeaponGrabInput(false, providerGripOccupancy.right.weaponEngaged(), rightPhysicalGrip);
        const std::uint64_t shoulderOwnershipKey =
            _equipped.shoulderSheath.active ?
            _equipped.shoulderSheath.weaponOwnershipKey :
            _equipped.shoulderCoordinator.activeAction !=
                    equipped_weapon_shoulder::Action::None ?
                _equipped.shoulderCoordinator.weaponOwnershipKey :
                currentEquippedWeaponOwnershipKey;

        const auto sheathModeForHand = [&](bool isLeft) {
            const bool firingRole = _equipped.shoulderSheath.active || isLeft == firingHandIsLeft;
            return equipped_weapon_shoulder::resolveSheathInputMode(
                _equipped.handlingSettings.immersiveWeapon.firingGripDetachEnabled,
                equipped_weapon_toggle_grab_policy::usesToggleForRole(_equipped.handlingSettings.weaponGrabMode, firingRole));
        };
        equipped_weapon_shoulder::FrameInput coordinatorInput{
            .enabled = handlingEnabled,
            .inputAllowed = !menuInputActive,
            .storedActive = _equipped.shoulderSheath.active,
            .stashedByLeftHand =
                _equipped.shoulderSheath.stashedByLeftHand,
            .weaponOwnershipKey = shoulderOwnershipKey,
            .presentation = nativePresentation,
            .storedZone = _equipped.shoulderSheath.zone,
            .right = {
                .sheathInputMode = sheathModeForHand(false),
                .button = {
                    .held = rightPhysicalGrip.held,
                    .pressed = rightPhysicalGrip.pressed,
                    .released = rightPhysicalGrip.released,
                },
            },
            .left = {
                .sheathInputMode = sheathModeForHand(true),
                .button = {
                    .held = leftPhysicalGrip.held,
                    .pressed = leftPhysicalGrip.pressed,
                    .released = leftPhysicalGrip.released,
                },
            },
        };

        std::array<shoulder_stash::Decision, 2> detectorDecisions{};
        const auto detectorConfig =
            makeEquippedWeaponStashDetectorConfig(handlingEnabled);
        const auto queueCandidateHaptic = [&](const bool isLeft,
                                              const bool sameStoredZone,
                                              const shoulder_stash::Decision& decision,
                                              shoulder_stash::RuntimeState& detectorState) {
            if (!decision.candidate || !sameStoredZone ||
                !g_rockConfig.rockShoulderStashHapticsEnabled) {
                return;
            }
            const bool pulseDue = _contacts.hapticClockSeconds >=
                detectorState.nextCandidatePulseTimeSeconds;
            if (!decision.enteredCandidate &&
                !decision.changedCandidate && !pulseDue) {
                return;
            }
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left :
                         feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
                shoulder_stash_haptic_policy::computeCandidatePulseIntensity(
                    decision.confidence,
                    shoulder_stash_haptic_policy::CandidatePulseConfig{
                        .enabled = true,
                        .baseIntensity = g_rockConfig.
                            rockShoulderStashCandidateHapticBaseIntensity,
                        .maxIntensity = g_rockConfig.
                            rockShoulderStashCandidateHapticIntensity,
                    }));
            detectorState.nextCandidatePulseTimeSeconds =
                _contacts.hapticClockSeconds +
                (std::max)(0.02f,
                    g_rockConfig.
                        rockShoulderStashCandidateHapticIntervalSeconds);
        };
        const auto publishDetectorDecision = [&](const bool isLeft,
                                                 const shoulder_stash::Decision& decision) {
            auto& handInput = isLeft ?
                coordinatorInput.left : coordinatorInput.right;
            handInput.detector = {
                .candidate = decision.candidate,
                .confirmed = decision.confirmedForCommit,
                .zone = decision.zone,
                .confidence = decision.confidence,
            };
        };

        if (_equipped.shoulderSheath.active) {
            _equipped.stashStates = {};
            const bool roleNeutralFiringGripOwnership =
                resolveEquippedWeaponDetachDecision(
                    _equipped.handlingSettings).
                    firingGripOwnershipEnabled;
            for (const bool isLeft : { false, true }) {
                const std::size_t handIndex = isLeft ? 1u : 0u;
                Hand& hand = isLeft ? _leftHand : _rightHand;
                const HandFrameInput& handInput =
                    isLeft ? frame.left : frame.right;
                auto& detectorState =
                    _equipped.sheathRetrievalStates[handIndex];
                const bool handEmpty = !hand.isHolding() &&
                    !_touchGrabRuntime.isHandActive(isLeft) &&
                    !_forceGrab.pendingCommits[handIndex].active &&
                    !hand.hasActivePullCatchIntent() &&
                    !hand.hasPendingExtractedItemHandoff() &&
                    !equippedWeaponRoster().slotZeroGrips(isLeft).weaponEngaged();
                const bool handAllowedByHandlingMode =
                    roleNeutralFiringGripOwnership || !isLeft;
                const bool handCanOwnFiringGrip =
                    handAllowedByHandlingMode &&
                    TwoHandedGrip::canBeginPrimaryOnlyGripForHand(isLeft);
                auto& policyHand = isLeft ?
                    coordinatorInput.left : coordinatorInput.right;
                policyHand.disabled = handInput.disabled;
                policyHand.eligible = !storage_interaction::blocksShoulder(isLeft) && !handInput.disabled && handEmpty &&
                    handCanOwnFiringGrip;
                if (!policyHand.eligible || menuInputActive) {
                    shoulder_stash::resetRuntime(detectorState);
                    continue;
                }

                const auto decision = shoulder_stash::evaluate(
                    shoulder_stash::DetectorInput{
                        .isLeftHand = isLeft,
                        .probe = shoulder_stash::Probe{
                            .pointGame = handInput.grabAnchorWorld,
                        },
                        .hmdProbe = makeShoulderStashHmdProbe(handInput),
                        .hasHmdProbe = true,
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = detectorConfig,
                    },
                    detectorState);
                detectorDecisions[handIndex] = decision;
                publishDetectorDecision(isLeft, decision);
                queueCandidateHaptic(
                    isLeft,
                    decision.zone ==
                        _equipped.shoulderSheath.zone,
                    decision,
                    detectorState);
            }
        } else {
            _equipped.sheathRetrievalStates = {};
            const auto manualStashCarryHand =
                equipped_weapon_drop_policy::
                    resolveEquippedWeaponStashCarryHand(
                        _twoHandedGrip.isPrimaryOnlyActive(),
                        _twoHandedGrip.isPartCarryActive(),
                        _twoHandedGrip.isHandPartCarryGripping(true),
                        _twoHandedGrip.isHandPartCarryGripping(false),
                        _twoHandedGrip.isFiringHandLeft());
            const auto attachedShoulderGestureHand = firingHandIsLeft ?
                equipped_weapon_drop_policy::SourceHand::Left :
                equipped_weapon_drop_policy::SourceHand::Right;
            const bool manualCarryActive =
                _twoHandedGrip.isPrimaryOnlyActive() ||
                _twoHandedGrip.isPartCarryActive();
            const auto stashCarryHand = manualStashCarryHand !=
                    equipped_weapon_drop_policy::SourceHand::None ?
                manualStashCarryHand :
                handlingEnabled && !manualCarryActive ?
                    attachedShoulderGestureHand :
                    equipped_weapon_drop_policy::SourceHand::None;

            for (const bool isLeft : { false, true }) {
                const std::size_t handIndex = isLeft ? 1u : 0u;
                const bool selectedCarryHand =
                    stashCarryHand !=
                        equipped_weapon_drop_policy::SourceHand::None &&
                    equipped_weapon_drop_policy::isLeft(stashCarryHand) ==
                        isLeft;
                auto& detectorState =
                    _equipped.stashStates[handIndex];
                auto& policyHand = isLeft ?
                    coordinatorInput.left : coordinatorInput.right;
                policyHand.carriesWeapon = selectedCarryHand;
                if (!selectedCarryHand || menuInputActive || storage_interaction::blocksShoulder(isLeft)) {
                    shoulder_stash::resetRuntime(detectorState);
                    continue;
                }

                Hand& hand = isLeft ? _leftHand : _rightHand;
                const HandFrameInput& handInput =
                    isLeft ? frame.left : frame.right;
                const bool handEmpty = !hand.isHolding() &&
                    !_touchGrabRuntime.isHandActive(isLeft) &&
                    !_forceGrab.pendingCommits[handIndex].active &&
                    !hand.hasActivePullCatchIntent() &&
                    !hand.hasPendingExtractedItemHandoff();
                policyHand.disabled = handInput.disabled;
                policyHand.eligible = handlingEnabled &&
                    !handInput.disabled && handEmpty && weaponNode != nullptr &&
                    currentEquippedWeaponOwnershipKey != 0;
                if (!policyHand.eligible) {
                    shoulder_stash::resetRuntime(detectorState);
                    continue;
                }

                const auto decision = shoulder_stash::evaluate(
                    shoulder_stash::DetectorInput{
                        .isLeftHand = isLeft,
                        .probe = shoulder_stash::Probe{
                            .pointGame = handInput.grabAnchorWorld,
                        },
                        .hmdProbe = makeShoulderStashHmdProbe(handInput),
                        .hasHmdProbe = true,
                        .hasHmdFrame = frame.hasHmdFrame,
                        .hmdPositionWorld = frame.hmdPositionWorld,
                        .hmdForwardWorld = frame.hmdForwardWorld,
                        .deltaSeconds = frame.deltaSeconds,
                        .config = detectorConfig,
                    },
                    detectorState);
                detectorDecisions[handIndex] = decision;
                publishDetectorDecision(isLeft, decision);
                queueCandidateHaptic(
                    isLeft,
                    true,
                    decision,
                    detectorState);
            }
        }

        const auto previousPhase =
            _equipped.shoulderCoordinator.phase;
        result.decision = equipped_weapon_shoulder::advance(
            _equipped.shoulderCoordinator,
            coordinatorInput);
        _equipped.shoulderGestureConsumedThisFrame[0] =
            result.decision.consumeRightInput;
        _equipped.shoulderGestureConsumedThisFrame[1] =
            result.decision.consumeLeftInput;

        if (result.decision.action !=
                equipped_weapon_shoulder::Action::None) {
            const auto& selectedDetector = detectorDecisions[
                result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                    1u : 0u];
            ROCK_LOG_INFO(
                Weapon,
                "Equipped shoulder coordinator action={} reason={} phase={}->{} hand={} gesture={} zone={} candidate={} confirmed={} source={} confidence={:.2f} speed={:.1f} nativeState={}({}) sheathInput={} immersive={} grabMode={}",
                equipped_weapon_shoulder::actionName(
                    result.decision.action),
                equipped_weapon_shoulder::reasonName(
                    result.decision.reason),
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(
                    _equipped.shoulderCoordinator.phase),
                equipped_weapon_shoulder::handName(result.decision.hand),
                result.decision.gestureSerial,
                body_zone::bodyZoneName(result.decision.zone),
                (result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                        coordinatorInput.left.detector.candidate :
                        coordinatorInput.right.detector.candidate) ?
                    "yes" : "no",
                (result.decision.hand ==
                        equipped_weapon_shoulder::Hand::Left ?
                        coordinatorInput.left.detector.confirmed :
                        coordinatorInput.right.detector.confirmed) ?
                    "yes" : "no",
                shoulder_stash::evidenceSourceName(
                    selectedDetector.source),
                result.decision.confidence,
                selectedDetector.speedGameUnitsPerSecond,
                nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    nativeWeaponState),
                equipped_weapon_shoulder::sheathInputModeName(
                    result.decision.hand == equipped_weapon_shoulder::Hand::Left ?
                        coordinatorInput.left.sheathInputMode : coordinatorInput.right.sheathInputMode),
                _equipped.handlingSettings.immersiveWeapon.
                        firingGripDetachEnabled ?
                    "yes" : "no",
                equipped_weapon_toggle_grab_policy::modeName(_equipped.handlingSettings.weaponGrabMode));
        } else if (previousPhase !=
                   _equipped.shoulderCoordinator.phase) {
            ROCK_LOG_DEBUG(
                Weapon,
                "Equipped shoulder coordinator phase={}->{} reason={} gestureHand={} gesture={} nativeState={}({}) stored={}",
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(
                    _equipped.shoulderCoordinator.phase),
                equipped_weapon_shoulder::reasonName(
                    result.decision.reason),
                equipped_weapon_shoulder::handName(
                    _equipped.shoulderCoordinator.actionHand),
                _equipped.shoulderCoordinator.actionGestureSerial,
                nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    nativeWeaponState),
                _equipped.shoulderSheath.active ? "yes" : "no");
        }

        const auto toSourceHand = [](const equipped_weapon_shoulder::Hand hand) {
            return hand == equipped_weapon_shoulder::Hand::Left ?
                equipped_weapon_drop_policy::SourceHand::Left :
                hand == equipped_weapon_shoulder::Hand::Right ?
                equipped_weapon_drop_policy::SourceHand::Right :
                equipped_weapon_drop_policy::SourceHand::None;
        };
        result.sourceHand = toSourceHand(result.decision.hand);
        if (result.decision.hand !=
            equipped_weapon_shoulder::Hand::None) {
            result.detectorDecision = detectorDecisions[
                equipped_weapon_shoulder::isLeft(result.decision.hand) ?
                    1u : 0u];
        }

        if (result.decision.action !=
            equipped_weapon_shoulder::Action::SubmitRetrieve) {
            return result;
        }

        const bool retrieveWithLeftHand =
            result.decision.hand ==
            equipped_weapon_shoulder::Hand::Left;
        const std::size_t handIndex = retrieveWithLeftHand ? 1u : 0u;
        if (!capturedCurrentIdentity ||
            !_equipped.shoulderSheath.active) {
            equipped_weapon_shoulder::reportExecutionResult(
                _equipped.shoulderCoordinator,
                result.decision.action,
                false);
            ROCK_LOG_WARN(
                Weapon,
                "Equipped weapon shoulder retrieval rejected after coordinator selection because exact stored identity is unavailable");
            return result;
        }

        const auto storedZone = _equipped.shoulderSheath.zone;
        const auto drawResult =
            native_equipped_weapon_draw::submitExactCurrent(
                currentIdentity);
        const bool drawAccepted =
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::Submitted ||
            drawResult.result ==
                native_equipped_weapon_draw::SubmitResult::
                    AlreadyDrawingOrDrawn;
        equipped_weapon_shoulder::reportExecutionResult(
            _equipped.shoulderCoordinator,
            result.decision.action,
            drawAccepted);
        if (!drawAccepted) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "Equipped weapon shoulder unsheath failed formID={:08X} hand={} gesture={} zone={} state={}({})->{}({}) result={}",
                currentIdentity.formID,
                equipped_weapon_shoulder::handName(result.decision.hand),
                result.decision.gestureSerial,
                body_zone::bodyZoneName(storedZone),
                drawResult.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateBefore),
                drawResult.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    drawResult.stateAfter),
                native_equipped_weapon_draw::submitResultName(
                    drawResult.result));
            shoulder_stash::resetRuntime(
                _equipped.sheathRetrievalStates[handIndex]);
            return result;
        }

        _equipped.transition.pendingGrip() =
            PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = true,
                .isLeft = retrieveWithLeftHand,
                .targetWeaponFormID = currentIdentity.formID,
                .targetWeaponInstanceData = currentIdentity.instanceData,
                .remainingSeconds = 10.0f,
                .source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::ShoulderRetrieval,
                .hasFiringHandWeaponLocal = retrieveWithLeftHand &&
                    _equipped.shoulderSheath.
                        hasLeftFiringGripTransfer,
                .firingHandWeaponLocal =
                    _equipped.shoulderSheath.
                        leftFiringHandWeaponLocal,
                .hasFiringGripWeaponLocal = retrieveWithLeftHand &&
                    _equipped.shoulderSheath.
                        hasLeftFiringGripTransfer,
                .firingGripWeaponLocal =
                    _equipped.shoulderSheath.
                        leftFiringGripWeaponLocal,
            };
        if (g_rockConfig.rockShoulderStashHapticsEnabled) {
            (void)_feedbackHaptics.queue(
                retrieveWithLeftHand ?
                    feedback_haptics::FeedbackHand::Left :
                    feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                g_rockConfig.rockShoulderStashCommitHapticIntensity);
        }
        ROCK_LOG_INFO(
            Weapon,
            "Equipped weapon shoulder unsheathed formID={:08X} hand={} gesture={} zone={} confidence={:.2f} state={}({})->{}({}) result={}",
            currentIdentity.formID,
            equipped_weapon_shoulder::handName(result.decision.hand),
            result.decision.gestureSerial,
            body_zone::bodyZoneName(storedZone),
            result.detectorDecision.confidence,
            drawResult.stateBefore,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateBefore),
            drawResult.stateAfter,
            held_weapon_equip_state_policy::nativeWeaponStateName(
                drawResult.stateAfter),
            native_equipped_weapon_draw::submitResultName(
                drawResult.result));
        clearEquippedWeaponShoulderSheath(
            "physical-hand-unsheath-committed",
            false);
        return result;
    }
}

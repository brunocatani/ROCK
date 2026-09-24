#include "physics-interaction/weapon/EquippedWeaponTransitionCoordinator.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/NativeEquippedWeaponAttach.h"
#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"
#include "physics-interaction/weapon/WeaponEquipTransfer.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"

#include <algorithm>

namespace rock
{
    namespace
    {
        constexpr float kTransitionWatchdogSeconds = 10.0f;

        [[nodiscard]] const char* sourceName(const EquippedWeaponTransitionCoordinator::Source source) noexcept
        {
            switch (source) {
            case EquippedWeaponTransitionCoordinator::Source::InventoryEquip:
                return "inventory-equip";
            case EquippedWeaponTransitionCoordinator::Source::ObservedEquip:
                return "observed-equip";
            case EquippedWeaponTransitionCoordinator::Source::HeldTriggerEquip:
                return "held-trigger-equip";
            case EquippedWeaponTransitionCoordinator::Source::HeldGripZoneEquip:
                return "held-grip-zone-equip";
            case EquippedWeaponTransitionCoordinator::Source::MenuExit:
                return "menu-exit";
            case EquippedWeaponTransitionCoordinator::Source::WorkbenchExit:
                return "workbench-exit";
            default:
                return "unknown";
            }
        }

    }

    void EquippedWeaponTransitionCoordinator::traceHeldTransfer(const char* event) const
    {
        const auto& t = _heldTransfer;
        ROCK_LOG_INFO(Weapon, "Weapon transfer seq={} event={} phase={} outcome={} incoming={:08X}/{} hand={} role={} outgoing={:08X} pending={} target={:08X}/{:#x} inventory={} grip={} presentation={} reason={}",
            t.sequence, event, static_cast<unsigned>(t.phase), static_cast<unsigned>(t.outcome),
            t.request.reference, t.request.grab, t.request.isLeft ? "left" : "right", static_cast<unsigned>(t.request.role),
            t.outgoingReference, t.outgoingPending, t.targetForm, t.targetInstance,
            t.inventoryCommitted, t.gripAcquired, t.presentationAcquired, _heldFailureReason ? _heldFailureReason : "none");
    }

    bool EquippedWeaponTransitionCoordinator::beginHeldRequest(const held_weapon_transfer::Request& request)
    {
        held_weapon_transfer::rememberInventoryCompletion(_heldTransfer, _completedInventoryTransfer);
        if (!weapon_equip_transfer::canRecoverHeldEquip() || !held_weapon_transfer::admit(_heldTransfer, request)) return false;
        _heldWaitSeconds = 0.0f;
        _heldBridgeStarted = false;
        _heldRecoveryAttempts = 0;
        _heldRecoveryHolsterRequested = false;
        _heldFailureReason = nullptr;
        input_remap_runtime::setWeaponTransferPending(true);
        input_remap_runtime::blockWeaponTriggerUntilRelease(request.isLeft);
        traceHeldTransfer("admitted");
        return true;
    }

    void EquippedWeaponTransitionCoordinator::cancelHeldRequest(const char* reason, held_weapon_transfer::Outcome outcome)
    {
        if (!_heldTransfer.active() || _heldTransfer.phase == held_weapon_transfer::Phase::Recovering) return;
        _heldFailureReason = reason;
        held_weapon_transfer::cancel(_heldTransfer, outcome);
        _pendingGrip = {};
        if (_heldTransfer.restoringEquippedGrip) releasePendingNativeCull(true);
        traceHeldTransfer("cancelled");
        input_remap_runtime::setWeaponTransferPending(_heldTransfer.blocksFire());
    }

    void EquippedWeaponTransitionCoordinator::recordOutgoingRemoval(std::uint32_t reference)
    {
        if (held_weapon_transfer::outgoingRemoved(_heldTransfer, reference)) traceHeldTransfer("outgoing-removed");
    }

    void EquippedWeaponTransitionCoordinator::recordOutgoingResult(std::uint64_t sequence, bool succeeded)
    {
        if (held_weapon_transfer::outgoingFinished(_heldTransfer, sequence, succeeded)) traceHeldTransfer(succeeded ? "outgoing-complete" : "outgoing-recovered-or-failed");
    }

    void EquippedWeaponTransitionCoordinator::recordInventoryCommit(std::uint32_t form, std::uintptr_t instance,
        bool accepted, std::uintptr_t observedInstance)
    {
        if (held_weapon_transfer::inventoryCommitted(_heldTransfer, form, instance)) {
            if (accepted) {
                _heldTransfer.observedInstance = observedInstance;
                _heldTransfer.identityBound = true;
            }
            _heldWaitSeconds = 0.0f;
            traceHeldTransfer("inventory-committed");
        }
    }

    void EquippedWeaponTransitionCoordinator::recordGripAcquired(std::uint32_t form, std::uintptr_t instance,
        bool left, held_weapon_transfer::Role role)
    {
        if (held_weapon_transfer::acquireGrip(_heldTransfer, form, instance, left, role)) {
            releasePendingNativeCull(true);
            input_remap_runtime::blockWeaponTriggerUntilRelease(left);
            traceHeldTransfer("grip-acquired");
            input_remap_runtime::setWeaponTransferPending(false);
        }
    }

    void EquippedWeaponTransitionCoordinator::recordGripPresentation()
    {
        if (!_heldTransfer.active() || !_heldTransfer.gripAcquired || _heldTransfer.presentationAcquired) return;
        held_weapon_transfer::presentationAcquired(_heldTransfer);
        traceHeldTransfer("presentation-acquired");
    }

    void EquippedWeaponTransitionCoordinator::validateHeldSource(bool held, std::uint32_t reference, std::uint64_t grab,
        std::uint32_t world, std::uint32_t skeleton)
    {
        if (!_heldTransfer.active()) return;
        const auto& request = _heldTransfer.request;
        if (world != request.world || skeleton != request.skeleton) {
            // No old handles or inventory compensation may cross a lifecycle.
            traceHeldTransfer("lifecycle-abandoned");
            abandonSceneGraph();
        } else if (!held_weapon_transfer::sourceCurrent(_heldTransfer, held, reference, grab, world, skeleton)) {
            cancelHeldRequest("incoming-grab-ended", held_weapon_transfer::Outcome::Cancelled);
        }
    }

    void EquippedWeaponTransitionCoordinator::updateHeldRecovery()
    {
        if (_heldTransfer.phase != held_weapon_transfer::Phase::Recovering) return;
        input_remap_runtime::setWeaponTransferPending(true);
        const auto current = readCurrentIdentity();
        if (!_heldTransfer.matchesTarget(current.formID, current.instanceData)) {
            const bool ownsPresentation = _heldTransfer.matchesTarget(_boundIdentity.formID, _boundIdentity.instanceData) ||
                (!_boundIdentity.valid() && _expectedIdentity.formID == _heldTransfer.targetForm);
            held_weapon_transfer::recovered(_heldTransfer);
            releasePendingNativeCull(true);
            // A menu or another provider may already own presentation of a
            // different item. Retiring this request cannot cancel that work.
            if (ownsPresentation) finish(TerminalResult::IdentityLost, "held-transfer-identity-retired", true);
            traceHeldTransfer("recovered-or-superseded");
            input_remap_runtime::setWeaponTransferPending(_heldTransfer.blocksFire());
            return;
        }
        auto* player = RE::PlayerCharacter::GetSingleton();
        if (!player) return;
        auto visual = equipped_weapon_visual_state::observe(current.formID);
        RE::NiPointer<RE::NiAVObject> exact(visual.exactInstance);
        _bridge.release("held-transfer-recovery");
        if (exact && equipped_weapon_visual_state::isLocallyVisible(exact.get())) {
            if (_pendingNativeCull.get() != exact.get()) releasePendingNativeCull(true);
            _pendingNativeCull = exact;
            equipped_weapon_visual_state::setLocallyVisible(exact.get(), false);
        }
        const auto state = f4vr::getNativeWeaponState(player);
        if (_heldRecoveryAttempts == 0 || (state == 0 && _heldRecoveryAttempts == 1)) {
            ++_heldRecoveryAttempts;
            if (weapon_equip_transfer::unequipExactCurrentWeapon(current.formID, current.instanceData)) {
                held_weapon_transfer::recovered(_heldTransfer);
                releasePendingNativeCull(true);
                finish(TerminalResult::RecoveryExhausted, "held-transfer-returned-to-inventory", true);
                traceHeldTransfer("incoming-recovered-to-inventory");
                input_remap_runtime::setWeaponTransferPending(_heldTransfer.blocksFire());
                return;
            }
        }
        if (!_heldRecoveryHolsterRequested) {
            _heldRecoveryHolsterRequested = true;
            player->DrawWeaponMagicHands(false);
            ROCK_LOG_ERROR(Weapon, "Weapon transfer seq={} exact unequip failed; holstering target={:08X} and blocking fire until recovery", _heldTransfer.sequence, current.formID);
        }
        if (state == 0 && _heldRecoveryAttempts >= 2) {
            // Native holster is the safe terminal presentation if exact
            // inventory compensation was refused. Keep the identity-bound
            // fire guard until an explicit equip supersedes this item.
            releasePendingNativeCull(true);
        }
    }

    void EquippedWeaponTransitionCoordinator::releasePendingNativeCull(bool restore)
    {
        if (_pendingNativeCull && restore) equipped_weapon_visual_state::setLocallyVisible(_pendingNativeCull.get(), true);
        _pendingNativeCull.reset();
    }

    void EquippedWeaponTransitionCoordinator::resumeMenuGrip(const PendingGrip& grip, std::uint32_t world, std::uint32_t skeleton)
    {
        held_weapon_transfer::rememberInventoryCompletion(_heldTransfer, _completedInventoryTransfer);
        if (!grip.pending) return;
        const auto current = readCurrentIdentity();
        if (!held_weapon_transfer::sameMenuItem(grip.targetWeaponFormID, grip.targetWeaponInstanceData,
                current.formID, current.instanceData)) {
            ROCK_LOG_INFO(Weapon, "Equipped grip resume cancelled: equipped instance changed saved={:08X}/{:#x} current={:08X}/{:#x}",
                grip.targetWeaponFormID, grip.targetWeaponInstanceData, current.formID, current.instanceData);
            return;
        }
        if (!held_weapon_transfer::resumeMenu(_heldTransfer, current.formID, current.instanceData,
                { .world = world, .skeleton = skeleton, .isLeft = grip.isLeft,
                    .role = grip.supportGrip.validCarry() ? held_weapon_transfer::Role::Support :
                        grip.pairedGrips.valid() ? held_weapon_transfer::Role::Paired : held_weapon_transfer::Role::Firing })) {
            ROCK_LOG_WARN(Weapon, "Equipped grip resume rejected: another weapon transfer owns the item form={:08X}", current.formID);
            return;
        }
        _pendingGrip = grip;
        _pendingGrip.menuResume = true;
        _pendingGrip.pairedRelease = {};
        _heldWaitSeconds = 0.0f;
        _heldBridgeStarted = false;
        _heldRecoveryAttempts = 0;
        _heldRecoveryHolsterRequested = false;
        input_remap_runtime::setWeaponTransferPending(true);
        traceHeldTransfer("menu-role-restored");
        if (!_pendingGrip.pending) cancelHeldRequest("menu-carry-capture-unavailable");
    }

    bool EquippedWeaponTransitionCoordinator::beginHeldTransition(
        const ExpectedIdentity& expected,
        const Source source,
        const EquipVisualBridge::BeginInput& bridgeInput)
    {
        if (expected.formID == 0) {
            return false;
        }

        if (_bridge.isActive()) {
            _bridge.shutdown();
        }
        const bool bridgeStarted = _bridge.begin(bridgeInput);
        _heldBridgeStarted = bridgeStarted && _heldTransfer.active();
        _expectedIdentity = expected;
        _boundIdentity = {};
        _policyState = {};
        _source = source;
        _activeSeconds = 0.0f;
        resetDrawRecoveryClock(false);
        _active = true;
        _waitingForExpectedIdentity = true;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
        ++_transitionSequence;

        const auto current = readCurrentIdentity();
        _observedIdentity = current;
        _observationInitialized = true;
        if (expectedMatches(current)) {
            bindCurrentIdentity(current, source, "held-equip-current");
            auto immediateVisual = equipped_weapon_visual_state::observe(
                current.formID,
                _supersededNativeInstanceNode);
            _bridge.update(EquipVisualBridge::UpdateInput{
                .deltaSeconds = 0.0f,
                .advanceLifetime = false,
                .presentModel = _bridge.hasVisualModel(),
                .nativeVisual = &immediateVisual,
            });
        }

        ROCK_LOG_INFO(Weapon,
            "Equipped weapon transition armed source={} expectedForm={:08X} expectedInstance={:#x} previousForm={:08X} previousInstance={:#x} bridge={}",
            sourceName(source),
            expected.formID,
            expected.instanceData,
            expected.previousFormID,
            expected.previousInstanceData,
            bridgeStarted ? "yes" : "no");
        return bridgeStarted;
    }

    void EquippedWeaponTransitionCoordinator::requestCurrentWeaponReconcile(const Source source) noexcept
    {
        _requestedCurrentSource = source;
        _requestCurrentPending = true;
    }

    void EquippedWeaponTransitionCoordinator::update(const FrameInput& input)
    {
        _presentationKnown = false;
        _presentationWeaponFormID = 0;
        _nativeRenderable = false;
        if (!input.localSkeletonReady) {
            if (_active || _observationInitialized || _requestCurrentPending) {
                abandonSceneGraph();
            }
            return;
        }

        if (_heldTransfer.gripAcquired || !_heldTransfer.active()) releasePendingNativeCull(true);

        const auto current = readCurrentIdentity();
        if (_heldTransfer.active() && !input.gripSuspended) {
            if (_heldTransfer.phase == held_weapon_transfer::Phase::AwaitEquip && input.menuBlocking) {
                cancelHeldRequest("menu-before-pickup", held_weapon_transfer::Outcome::Cancelled);
            }
            if (!input.menuBlocking && !input.compatibilityBlocking && input.visualAuthorityAvailable) {
                _heldWaitSeconds += (std::max)(0.0f, input.deltaSeconds);
                if (_heldTransfer.restoringEquippedGrip && !_heldTransfer.matchesTarget(current.formID, current.instanceData)) {
                    cancelHeldRequest("resumed-equipped-instance-changed", held_weapon_transfer::Outcome::Cancelled);
                } else if (!held_weapon_transfer::equippedSourceCurrent(_heldTransfer, current.formID, current.instanceData)) {
                    cancelHeldRequest("equipped-item-changed-before-pickup", held_weapon_transfer::Outcome::Cancelled);
                } else if (_heldTransfer.inventoryCommitted && !_heldTransfer.matchesTarget(current.formID, current.instanceData)) {
                    cancelHeldRequest("incoming-identity-replaced");
                } else if (_heldTransfer.phase == held_weapon_transfer::Phase::AwaitGrip && !_pendingGrip.pending) {
                    cancelHeldRequest("requested-grip-cancelled");
                } else if ((_heldTransfer.phase == held_weapon_transfer::Phase::AwaitEquip && _heldWaitSeconds >= 10.0f) ||
                    (_heldTransfer.phase == held_weapon_transfer::Phase::AwaitGrip && !_heldTransfer.restoringEquippedGrip &&
                        (_heldWaitSeconds >= 1.0f || (_heldBridgeStarted && _bridge.presentationLeaseWouldExpire(input.deltaSeconds))))) {
                    cancelHeldRequest("native-or-grip-readiness-exhausted");
                }
                if (_heldTransfer.phase == held_weapon_transfer::Phase::Recovering) {
                    updateHeldRecovery();
                    return;
                }
            }
            input_remap_runtime::setWeaponTransferPending(_heldTransfer.blocksFire());
        }
        const bool currentMatchesIntentionalShoulderSheath =
            input.intentionalShoulderSheathActive &&
            current.valid() &&
            current.formID == input.shoulderSheathFormID &&
            current.instanceData == input.shoulderSheathInstanceData &&
            current.equipIndex == input.shoulderSheathEquipIndex &&
            held_weapon_equip_state_policy::
                isShoulderStashedPresentationState(
                    input.nativeWeaponState);
        if (!_observationInitialized) {
            _observedIdentity = current;
            _observationInitialized = true;
            _wasMenuBlocking = input.menuBlocking;
            if (input.menuBlocking) {
                _menuEntryIdentity = current;
                _menuEntryNativeInstanceNode = current.valid() ?
                    reinterpret_cast<std::uintptr_t>(
                        equipped_weapon_visual_state::observe(current.formID).exactInstance) :
                    0;
                _menuEntryCaptured = true;
            }
            if (current.valid() &&
                !currentMatchesIntentionalShoulderSheath) {
                bindCurrentIdentity(
                    current,
                    Source::ObservedEquip,
                    "initial-equipped-identity");
            }
        }

        const bool menuOpened = !_wasMenuBlocking && input.menuBlocking;
        const bool menuClosed = _wasMenuBlocking && !input.menuBlocking;
        if (menuOpened) {
            _menuEntryIdentity = current;
            _menuEntryNativeInstanceNode = current.valid() ?
                reinterpret_cast<std::uintptr_t>(
                    equipped_weapon_visual_state::observe(current.formID).exactInstance) :
                0;
            _menuEntryCaptured = true;
        }

        const bool identityChanged = current != _observedIdentity;
        if (identityChanged) {
            const auto previous = _observedIdentity;
            _observedIdentity = current;
            if (_waitingForExpectedIdentity && expectedMatches(current)) {
                bindCurrentIdentity(
                    current,
                    _source,
                    "expected-identity-observed",
                    {},
                    0);
            } else if (!_waitingForExpectedIdentity && current.valid() &&
                !currentMatchesIntentionalShoulderSheath) {
                if (_bridge.isActive() && _boundIdentity.valid() && current != _boundIdentity) {
                    _bridge.release("equipped-weapon-changed");
                }
                const auto previousNativeInstanceNode =
                    _menuEntryCaptured && previous == _menuEntryIdentity ?
                    _menuEntryNativeInstanceNode :
                    0;
                bindCurrentIdentity(
                    current,
                    Source::ObservedEquip,
                    "equipped-identity-changed",
                    previous,
                    previousNativeInstanceNode);
            } else if (!_waitingForExpectedIdentity && !current.valid() && previous == _boundIdentity) {
                finish(TerminalResult::WeaponUnequipped, "weapon-unequipped", true);
            }
        }

        _wasMenuBlocking = input.menuBlocking;
        if (currentMatchesIntentionalShoulderSheath) {
            _requestCurrentPending = false;
            if (menuClosed) {
                _menuEntryIdentity = {};
                _menuEntryNativeInstanceNode = 0;
                _menuEntryCaptured = false;
            }
            if (_active) {
                ROCK_LOG_INFO(Weapon,
                    "Equipped weapon transition yielded to exact shoulder sheath formID={:08X} instance={:#x} equipIndex={}",
                    current.formID,
                    current.instanceData,
                    current.equipIndex);
                finish(
                    TerminalResult::IntentionalShoulderSheathe,
                    "intentional-shoulder-sheathe",
                    true);
            }
            return;
        }
        if (menuClosed && !_waitingForExpectedIdentity && current.valid()) {
            bindCurrentIdentity(
                current,
                Source::MenuExit,
                "menu-closed",
                _menuEntryCaptured ? _menuEntryIdentity : Identity{},
                _menuEntryCaptured ? _menuEntryNativeInstanceNode : 0);
        }
        if (_requestCurrentPending) {
            _requestCurrentPending = false;
            if (current.valid() && !_waitingForExpectedIdentity) {
                bindCurrentIdentity(
                    current,
                    _requestedCurrentSource,
                    "explicit-reconcile",
                    _menuEntryCaptured ? _menuEntryIdentity : Identity{},
                    _menuEntryCaptured ? _menuEntryNativeInstanceNode : 0);
            }
        }
        if (menuClosed) {
            _menuEntryIdentity = {};
            _menuEntryNativeInstanceNode = 0;
            _menuEntryCaptured = false;
        }

        // Observe current presentation even after the transition watchdog
        // completes. Reuse the same bounded weapon observation for recovery.
        auto visual = equipped_weapon_visual_state::Snapshot{};
        if (current.valid() && input.visualAuthorityAvailable && (_active || input.localSkeletonReady) &&
            !input.menuBlocking && !input.compatibilityBlocking) {
            visual = equipped_weapon_visual_state::observe(current.formID,
                _active ? _supersededNativeInstanceNode : 0, &_visualCache);
            _presentationWeaponFormID = current.formID;
            _presentationKnown = input.localSkeletonReady;
            _nativeRenderable = visual.exactInstance && visual.ancestorPathVisible && visual.instanceLocallyVisible;
        }
        if (_heldTransfer.active() && _heldTransfer.gripAcquired && !_bridge.isHandPoseHandoffActive() &&
            _heldTransfer.matchesTarget(current.formID, current.instanceData) && _nativeRenderable) recordGripPresentation();
        if (!_active) {
            return;
        }

        const float deltaSeconds = (std::max)(0.0f, input.deltaSeconds);
        const float drawRecoveryWallDelta =
            sampleDrawRecoveryWallDelta();
        if (!input.visualAuthorityAvailable || input.menuBlocking || input.compatibilityBlocking) {
            _bridge.advancePresentationLease(deltaSeconds);
            return;
        }

        _activeSeconds += deltaSeconds;
        if (!_waitingForExpectedIdentity &&
            !input.nativeWeaponAnimationActive) {
            _drawRecoveryElapsedSeconds += (std::max)(
                deltaSeconds,
                drawRecoveryWallDelta);
        }

        if (_waitingForExpectedIdentity) {
            _bridge.update(EquipVisualBridge::UpdateInput{
                .deltaSeconds = deltaSeconds,
                .advanceLifetime = true,
                .presentModel = _bridge.hasVisualModel(),
                .nativeVisual = nullptr,
            });
            if (_activeSeconds >= kTransitionWatchdogSeconds) {
                finish(
                    TerminalResult::ExpectedIdentityTimeout,
                    "expected-identity-timeout",
                    true);
            }
            return;
        }

        if (!current.valid() || current != _boundIdentity) {
            finish(TerminalResult::IdentityLost, "bound-identity-lost", true);
            return;
        }

        if (input.nativeWeaponAnimationActive) {
            // Reload/bolt owners deliberately replace or hide the same Weapon
            // graph. Yield both the exact-child cull and the temporary hand
            // pose; a completed equip needs no further watchdog after this
            // explicit ownership transfer.
            _bridge.completeHandPoseHandoff("native-weapon-animation");
            if (_policyState.nativeHandoffObserved) {
                finish(
                    TerminalResult::NativeAnimationHandoff,
                    "native-weapon-animation-after-handoff",
                    true);
                return;
            }
            _bridge.update(EquipVisualBridge::UpdateInput{
                .deltaSeconds = deltaSeconds,
                .advanceLifetime = true,
                .presentModel = false,
                .nativeVisual = &visual,
            });
            if (_activeSeconds >= kTransitionWatchdogSeconds) {
                finish(
                    TerminalResult::NativeAnimationHandoff,
                    "native-weapon-animation-timeout",
                    true);
            }
            return;
        }

        const bool weaponExactlyDrawn =
            input.nativeWeaponState == static_cast<std::uint32_t>(
                held_weapon_equip_state_policy::NativeWeaponState::Drawn);
        if (_policyState.nativeHandoffObserved && !weaponExactlyDrawn) {
            finish(
                TerminalResult::WeaponNoLongerDrawn,
                "weapon-no-longer-drawn",
                true);
            return;
        }

        const bool exactNativeInstanceIsCurrent =
            visual.exactInstance &&
            reinterpret_cast<std::uintptr_t>(visual.exactInstance) !=
                _supersededNativeInstanceNode;
        const bool nativeRenderableBeforeAdvance =
            exactNativeInstanceIsCurrent &&
            visual.ancestorPathVisible &&
            (visual.instanceLocallyVisible ||
                _bridge.ownsNativeInstanceCull(visual.exactInstance));
        const bool lateNativeLoss =
            _policyState.nativeHandoffObserved &&
            _policyState.missingFrames == 0 &&
            !nativeRenderableBeforeAdvance;
        if (lateNativeLoss && !_lateRecoveryWindowGranted) {
            // Give the exact attach repair its own bounded window even when a
            // stale detach arrives on the original watchdog's final frame.
            _activeSeconds = 0.0f;
            _lateRecoveryWindowGranted = true;
        }

        const bool bridgeOwnsCull = _bridge.ownsNativeInstanceCull(visual.exactInstance);
        const auto presentationLocalAttemptsBeforeAdvance =
            _policyState.localVisibilityAttempts;
        const auto presentationAttachAttemptsBeforeAdvance =
            _policyState.attachAttempts;
        auto decision = equipped_weapon_transition_policy::advance(
            _policyState,
            equipped_weapon_transition_policy::FrameInput{
                .drawRecoveryElapsedSeconds =
                    _drawRecoveryElapsedSeconds,
                .mutationAllowed = true,
                .identityMatches = true,
                .weaponExactlyDrawn = weaponExactlyDrawn,
                .nativeWeaponState = input.nativeWeaponState,
                .bridgeModelAvailable = _bridge.hasVisualModel(),
                .nativeInstanceFound = exactNativeInstanceIsCurrent,
                .nativeAncestorPathVisible = visual.ancestorPathVisible,
                .nativeInstanceLocallyVisible = visual.instanceLocallyVisible,
                .bridgeOwnsNativeInstanceCull = bridgeOwnsCull,
                .gripHandoffPending = _pendingGrip.pending ||
                    (_heldTransfer.active() && !_heldTransfer.gripAcquired) || _bridge.isHandPoseHandoffActive(),
            });

        using NativeWeaponState =
            held_weapon_equip_state_policy::NativeWeaponState;
        const auto nativeWeaponState =
            static_cast<NativeWeaponState>(input.nativeWeaponState);
        const bool nativeDrawAcknowledged =
            weaponExactlyDrawn ||
            nativeWeaponState == NativeWeaponState::WantToDraw ||
            nativeWeaponState == NativeWeaponState::Drawing;
        if (_drawExhaustionLogged &&
            nativeDrawAcknowledged &&
            !_policyState.drawRecoveryExhausted) {
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition draw recovery resumed source={} formID={:08X} instance={:#x} state={}({}) requests={} elapsed={:.3f}s",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                input.nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    input.nativeWeaponState),
                _policyState.drawRequests,
                _drawRecoveryElapsedSeconds);
            _drawExhaustionLogged = false;
        }
        if (_repairExhaustionLogged && nativeRenderableBeforeAdvance) {
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition presentation recovery resumed source={} formID={:08X} instance={:#x} localAttempts={} attachAttempts={} elapsed={:.3f}s",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                presentationLocalAttemptsBeforeAdvance,
                presentationAttachAttemptsBeforeAdvance,
                _drawRecoveryElapsedSeconds);
            _repairExhaustionLogged = false;
        }

        const auto applyDrawSubmissionOutcome =
            [this](const native_equipped_weapon_draw::Result& result) noexcept {
                if (result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            PartialActionRecovered &&
                    result.stateAfter == static_cast<std::uint32_t>(
                        held_weapon_equip_state_policy::NativeWeaponState::
                            Drawing)) {
                    _policyState.partialDrawRecoveryStartedAtSeconds =
                        _drawRecoveryElapsedSeconds;
                    _policyState.partialDrawRecoveryActive = true;
                    _policyState.drawRecoveryExhausted = false;
                }

                if (result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            InvalidWeaponState ||
                    result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            MissingPlayer ||
                    result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            MissingEquippedWeapon ||
                    result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            RecoveryPreparationUnavailable ||
                    result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            PartialRecoveryUnavailable ||
                    result.result ==
                        native_equipped_weapon_draw::SubmitResult::
                            PartialRecoveryRejected) {
                    _policyState.drawRecoveryExhausted = true;
                }
            };

        switch (decision.repair) {
        case equipped_weapon_transition_policy::RepairAction::RequestDraw: {
            const auto result = native_equipped_weapon_draw::submitExactCurrent(
                native_equipped_weapon_draw::Identity{
                    .formID = _boundIdentity.formID,
                    .instanceData = _boundIdentity.instanceData,
                    .equipIndex = _boundIdentity.equipIndex,
                });
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition draw recovery source={} formID={:08X} instance={:#x} request={} elapsed={:.3f}s state={}({})->{}({}) result={} evidenceSequence={} matchedActivations={} registeredUpdates={} activeClips={} updatedActiveClips={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                _policyState.drawRequests,
                _drawRecoveryElapsedSeconds,
                result.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(result.stateBefore),
                result.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(result.stateAfter),
                native_equipped_weapon_draw::submitResultName(result.result),
                result.evidenceSequence,
                result.matchedActivations,
                result.registeredUpdates,
                result.activeClips,
                result.updatedActiveClips);
            applyDrawSubmissionOutcome(result);
            break;
        }
        case equipped_weapon_transition_policy::RepairAction::RequestPreparedDraw: {
            const auto result =
                native_equipped_weapon_draw::submitPreparedExactCurrent(
                    native_equipped_weapon_draw::Identity{
                        .formID = _boundIdentity.formID,
                        .instanceData = _boundIdentity.instanceData,
                        .equipIndex = _boundIdentity.equipIndex,
                    });
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition prepared draw recovery source={} formID={:08X} instance={:#x} request={} elapsed={:.3f}s state={}({})->{}({}) result={} evidenceSequence={} matchedActivations={} registeredUpdates={} activeClips={} updatedActiveClips={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                _policyState.drawRequests,
                _drawRecoveryElapsedSeconds,
                result.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    result.stateBefore),
                result.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    result.stateAfter),
                native_equipped_weapon_draw::submitResultName(result.result),
                result.evidenceSequence,
                result.matchedActivations,
                result.registeredUpdates,
                result.activeClips,
                result.updatedActiveClips);
            applyDrawSubmissionOutcome(result);
            break;
        }
        case equipped_weapon_transition_policy::RepairAction::FinalizePartialDraw: {
            const auto result =
                native_equipped_weapon_draw::finalizePartialExactCurrent(
                    native_equipped_weapon_draw::Identity{
                        .formID = _boundIdentity.formID,
                        .instanceData = _boundIdentity.instanceData,
                        .equipIndex = _boundIdentity.equipIndex,
                    });
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition partial draw finalization source={} formID={:08X} instance={:#x} elapsed={:.3f}s state={}({})->{}({}) result={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                _drawRecoveryElapsedSeconds,
                result.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    result.stateBefore),
                result.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    result.stateAfter),
                native_equipped_weapon_draw::submitResultName(result.result));
            equipped_weapon_transition_policy::resetPartialDrawRecovery(
                _policyState);
            if (result.result !=
                    native_equipped_weapon_draw::SubmitResult::
                        PartialActionFinalized &&
                result.result !=
                    native_equipped_weapon_draw::SubmitResult::
                        RecoveryStateChanged) {
                _policyState.drawRecoveryWindowStartedAtSeconds =
                    _drawRecoveryElapsedSeconds -
                    equipped_weapon_transition_policy::
                        kDrawRecoveryDeadlineSeconds;
                _policyState.drawRecoveryWindowActive = true;
                _policyState.drawRecoveryExhausted = true;
            }
            break;
        }
        case equipped_weapon_transition_policy::RepairAction::DrawExhausted:
            if (!_drawExhaustionLogged) {
                _drawExhaustionLogged = true;
                const float drawRecoveryWindowSeconds =
                    (std::max)(
                        0.0f,
                        _drawRecoveryElapsedSeconds -
                            _policyState.drawRecoveryWindowStartedAtSeconds);
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon transition draw recovery exhausted source={} formID={:08X} instance={:#x} weaponState={}({}) requests={} elapsed={:.3f}s window={:.3f}s deadline={:.3f}s",
                    sourceName(_source),
                    _boundIdentity.formID,
                    _boundIdentity.instanceData,
                    input.nativeWeaponState,
                    held_weapon_equip_state_policy::nativeWeaponStateName(
                        input.nativeWeaponState),
                    _policyState.drawRequests,
                    _drawRecoveryElapsedSeconds,
                    drawRecoveryWindowSeconds,
                    equipped_weapon_transition_policy::
                        kDrawRecoveryDeadlineSeconds);
            }
            break;
        case equipped_weapon_transition_policy::RepairAction::RestoreLocalVisibility: {
            const bool restored = equipped_weapon_visual_state::restoreExactInstancePathVisibility(visual);
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition local visibility repair source={} formID={:08X} instance={:#x} restored={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                restored ? "yes" : "no");
            visual = equipped_weapon_visual_state::observe(
                _boundIdentity.formID,
                _supersededNativeInstanceNode);
            break;
        }
        case equipped_weapon_transition_policy::RepairAction::QueueNativeAttach: {
            const auto result = native_equipped_weapon_attach::submitExactCurrent(
                native_equipped_weapon_attach::Identity{
                    .formID = _boundIdentity.formID,
                    .instanceData = _boundIdentity.instanceData,
                    .equipIndex = _boundIdentity.equipIndex,
                });
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition native attach source={} formID={:08X} instance={:#x} attempt={}/{} result={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                _policyState.attachAttempts,
                equipped_weapon_transition_policy::kMaximumAttachAttempts,
                native_equipped_weapon_attach::submitResultName(result));
            if (result == native_equipped_weapon_attach::SubmitResult::UnsupportedRuntime ||
                result == native_equipped_weapon_attach::SubmitResult::ContractMismatch) {
                _policyState.attachAttempts = equipped_weapon_transition_policy::kMaximumAttachAttempts;
            }
            break;
        }
        case equipped_weapon_transition_policy::RepairAction::Exhausted:
            if (!_repairExhaustionLogged) {
                _repairExhaustionLogged = true;
                const float presentationRecoveryWindowSeconds =
                    (std::max)(
                        0.0f,
                        _drawRecoveryElapsedSeconds -
                            _policyState.
                                presentationRecoveryWindowStartedAtSeconds);
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon transition presentation recovery exhausted source={} formID={:08X} instance={:#x} exactInstance={} ancestorsVisible={} localVisible={} localAttempts={} attachAttempts={} elapsed={:.3f}s window={:.3f}s deadline={:.3f}s",
                    sourceName(_source),
                    _boundIdentity.formID,
                    _boundIdentity.instanceData,
                    exactNativeInstanceIsCurrent ? "yes" : "no",
                    visual.ancestorPathVisible ? "yes" : "no",
                    visual.instanceLocallyVisible ? "yes" : "no",
                    _policyState.localVisibilityAttempts,
                    _policyState.attachAttempts,
                    _drawRecoveryElapsedSeconds,
                    presentationRecoveryWindowSeconds,
                    equipped_weapon_transition_policy::
                        kPresentationRecoveryDeadlineSeconds);
            }
            break;
        case equipped_weapon_transition_policy::RepairAction::None:
        default:
            break;
        }

        _bridge.update(EquipVisualBridge::UpdateInput{
            .deltaSeconds = deltaSeconds,
            .advanceLifetime = true,
            .presentModel = decision.presentBridgeModel && !decision.handoffBridgeToNative,
            .nativeVisual = &visual,
            .leftCarrySolvedWeaponWorldValid =
                input.leftCarrySolvedWeaponWorldValid,
            .leftCarrySolvedWeaponWorld = input.leftCarrySolvedWeaponWorld,
        });
        if (_heldTransfer.phase == held_weapon_transfer::Phase::AwaitGrip && !_bridge.isActive() &&
            visual.exactInstance && equipped_weapon_visual_state::isLocallyVisible(visual.exactInstance)) {
            // Native geometry may arrive after the model bridge has gone
            // (notably menu recovery). Hide only until actual hand acquisition.
            if (_pendingNativeCull.get() != visual.exactInstance) releasePendingNativeCull(true);
            _pendingNativeCull.reset(visual.exactInstance);
            equipped_weapon_visual_state::setLocallyVisible(visual.exactInstance, false);
            _nativeRenderable = false;
        }
        if (_activeSeconds >= kTransitionWatchdogSeconds) {
            finish(
                (_pendingGrip.pending || (_heldTransfer.active() && !_heldTransfer.gripAcquired)) ?
                    TerminalResult::RecoveryExhausted : TerminalResult::Completed,
                "watchdog-complete",
                true);
        }
    }

    void EquippedWeaponTransitionCoordinator::shutdown()
    {
        if (_heldTransfer.active()) traceHeldTransfer("lifecycle-ended");
        const auto sequence = _heldTransfer.sequence;
        _heldTransfer = { .sequence = sequence };
        _completedInventoryTransfer = {};
        _pendingGrip = {};
        releasePendingNativeCull(true);
        input_remap_runtime::setWeaponTransferPending(false);
        _visualCache = {};
        _presentationKnown = false;
        _presentationWeaponFormID = 0;
        _nativeRenderable = false;
        if (_active) {
            _lastTerminalWeaponFormID = _boundIdentity.valid() ?
                _boundIdentity.formID :
                _expectedIdentity.formID;
            _lastTerminalSource = _source;
            _lastTerminalResult = TerminalResult::Shutdown;
            ++_terminalSequence;
        }
        _bridge.shutdown();
        _policyState = {};
        _observedIdentity = {};
        _boundIdentity = {};
        _menuEntryIdentity = {};
        _expectedIdentity = {};
        _menuEntryNativeInstanceNode = 0;
        _supersededNativeInstanceNode = 0;
        _activeSeconds = 0.0f;
        resetDrawRecoveryClock(false);
        _observationInitialized = false;
        _active = false;
        _waitingForExpectedIdentity = false;
        _requestCurrentPending = false;
        _wasMenuBlocking = false;
        _menuEntryCaptured = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
    }

    void EquippedWeaponTransitionCoordinator::abandonSceneGraph()
    {
        if (_heldTransfer.active()) traceHeldTransfer("lifecycle-ended");
        const auto sequence = _heldTransfer.sequence;
        _heldTransfer = { .sequence = sequence };
        _completedInventoryTransfer = {};
        _pendingGrip = {};
        releasePendingNativeCull(false);
        input_remap_runtime::setWeaponTransferPending(false);
        _visualCache = {};
        _presentationKnown = false;
        _presentationWeaponFormID = 0;
        _nativeRenderable = false;
        if (_active) {
            _lastTerminalWeaponFormID = _boundIdentity.valid() ?
                _boundIdentity.formID :
                _expectedIdentity.formID;
            _lastTerminalSource = _source;
            _lastTerminalResult = TerminalResult::ProviderLost;
            ++_terminalSequence;
        }
        _bridge.abandonSceneGraph();
        _policyState = {};
        _observedIdentity = {};
        _boundIdentity = {};
        _menuEntryIdentity = {};
        _expectedIdentity = {};
        _menuEntryNativeInstanceNode = 0;
        _supersededNativeInstanceNode = 0;
        _activeSeconds = 0.0f;
        resetDrawRecoveryClock(false);
        _observationInitialized = false;
        _active = false;
        _waitingForExpectedIdentity = false;
        _requestCurrentPending = false;
        _wasMenuBlocking = false;
        _menuEntryCaptured = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
    }

    EquippedWeaponTransitionCoordinator::Identity
    EquippedWeaponTransitionCoordinator::readCurrentIdentity() noexcept
    {
        auto* equipped = f4vr::getEquippedWeaponItem();
        auto* object = equipped ? equipped->item.object : nullptr;
        if (!object || object->formType != RE::ENUM_FORM_ID::kWEAP) {
            return {};
        }
        return Identity{
            .formID = object->formID,
            .instanceData = reinterpret_cast<std::uintptr_t>(
                equipped->item.instanceData.get()),
            .equipIndex = equipped->equipIndex.index,
        };
    }

    bool EquippedWeaponTransitionCoordinator::expectedMatches(const Identity& identity) const noexcept
    {
        return equipped_weapon_transition_policy::matchesExpectedIdentity(
            identity.formID,
            identity.instanceData,
            _expectedIdentity.formID,
            _expectedIdentity.instanceData,
            _expectedIdentity.previousFormID,
            _expectedIdentity.previousInstanceData);
    }

    void EquippedWeaponTransitionCoordinator::bindCurrentIdentity(
        const Identity& identity,
        const Source source,
        const char* reason,
        Identity previousIdentity,
        std::uintptr_t previousNativeInstanceNode)
    {
        if (!identity.valid()) {
            return;
        }
        _visualCache = {};
        const bool completesSuppressedHeldDraw =
            _waitingForExpectedIdentity &&
            (source == Source::HeldTriggerEquip ||
                source == Source::HeldGripZoneEquip || source == Source::InventoryEquip);
        const bool startsNewTransition = !_active ||
            !_boundIdentity.valid() ||
            _boundIdentity != identity ||
            _source != source;
        if (startsNewTransition) {
            ++_transitionSequence;
        }
        if (_waitingForExpectedIdentity) {
            previousIdentity = Identity{
                .formID = _expectedIdentity.previousFormID,
                .instanceData = _expectedIdentity.previousInstanceData,
            };
            previousNativeInstanceNode =
                _expectedIdentity.previousNativeInstanceNode;
        }
        const bool logicalIdentityChanged =
            previousIdentity.valid() &&
            (identity.formID != previousIdentity.formID ||
                identity.instanceData != previousIdentity.instanceData);
        _supersededNativeInstanceNode =
            logicalIdentityChanged ?
            previousNativeInstanceNode :
            0;
        _boundIdentity = identity;
        _expectedIdentity = ExpectedIdentity{
            .formID = identity.formID,
            .instanceData = identity.instanceData,
            .previousFormID = identity.formID,
            .previousInstanceData = identity.instanceData,
        };
        _policyState = {};
        _source = source;
        _activeSeconds = 0.0f;
        resetDrawRecoveryClock(true);
        _active = true;
        _waitingForExpectedIdentity = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon transition bound source={} reason={} formID={:08X} instance={:#x} equipIndex={} presentation=required",
            sourceName(source),
            reason ? reason : "unknown",
            identity.formID,
            identity.instanceData,
            identity.equipIndex);

        if (completesSuppressedHeldDraw) {
            /*
             * The scoped EquipManager transaction deliberately suppressed
             * its synchronous draw action after committing this exact held
             * weapon. Submit the coordinator-owned replacement now, while
             * the identity is still exact, so clip acceleration is armed at
             * the real draw boundary. The normal timed policy remains only
             * as bounded recovery if native state does not acknowledge it.
             */
            const auto result =
                native_equipped_weapon_draw::submitExactCurrent(
                    native_equipped_weapon_draw::Identity{
                        .formID = _boundIdentity.formID,
                        .instanceData = _boundIdentity.instanceData,
                        .equipIndex = _boundIdentity.equipIndex,
                    });
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition initial held draw source={} formID={:08X} instance={:#x} state={}({})->{}({}) result={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                result.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    result.stateBefore),
                result.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(
                    result.stateAfter),
                native_equipped_weapon_draw::submitResultName(result.result));
        }
    }

    void EquippedWeaponTransitionCoordinator::resetDrawRecoveryClock(
        const bool armed) noexcept
    {
        _drawRecoveryElapsedSeconds = 0.0f;
        _drawRecoveryLastUpdateAt =
            std::chrono::steady_clock::now();
        _drawRecoveryClockArmed = armed;
    }

    float EquippedWeaponTransitionCoordinator::sampleDrawRecoveryWallDelta() noexcept
    {
        const auto now = std::chrono::steady_clock::now();
        if (!_drawRecoveryClockArmed) {
            _drawRecoveryLastUpdateAt = now;
            return 0.0f;
        }

        const float elapsedSeconds =
            std::chrono::duration<float>(
                now - _drawRecoveryLastUpdateAt)
                .count();
        _drawRecoveryLastUpdateAt = now;
        return (std::max)(0.0f, elapsedSeconds);
    }

    void EquippedWeaponTransitionCoordinator::finish(
        const TerminalResult result,
        const char* reason,
        const bool releaseSceneGraph)
    {
        const auto terminalWeaponFormID = _boundIdentity.valid() ?
            _boundIdentity.formID :
            _expectedIdentity.formID;
        const auto terminalSource = _source;
        const auto terminalResult =
            (_drawExhaustionLogged || _repairExhaustionLogged) ?
                TerminalResult::RecoveryExhausted :
                result;
        if (releaseSceneGraph) {
            _bridge.release(reason);
        } else {
            _bridge.abandonSceneGraph();
        }
        _policyState = {};
        _boundIdentity = {};
        _expectedIdentity = {};
        _supersededNativeInstanceNode = 0;
        _activeSeconds = 0.0f;
        resetDrawRecoveryClock(false);
        _active = false;
        _waitingForExpectedIdentity = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
        _lastTerminalWeaponFormID = terminalWeaponFormID;
        _lastTerminalSource = terminalSource;
        _lastTerminalResult = terminalResult;
        ++_terminalSequence;
    }

    EquippedWeaponTransitionCoordinator::PublicSnapshot
    EquippedWeaponTransitionCoordinator::getPublicSnapshot() const noexcept
    {
        PublicSnapshot snapshot{};
        snapshot.transitionSequence = _transitionSequence;
        snapshot.terminalSequence = _terminalSequence;
        snapshot.weaponFormID = _active ?
            (_boundIdentity.valid() ? _boundIdentity.formID :
                                     _expectedIdentity.formID) :
            _lastTerminalWeaponFormID;
        snapshot.source = _active ? _source : _lastTerminalSource;
        snapshot.terminalResult = _lastTerminalResult;
        snapshot.active = _active;
        snapshot.identityPending = _waitingForExpectedIdentity;
        snapshot.drawPending = _active &&
            !_waitingForExpectedIdentity &&
            !_policyState.nativeHandoffObserved;
        snapshot.bridgePresented = _bridge.isModelPresented();
        snapshot.presentationKnown = _presentationKnown;
        snapshot.presentationWeaponFormID = _presentationWeaponFormID;
        snapshot.nativeRenderable = _presentationKnown && _nativeRenderable;
        snapshot.handPoseHandoffComplete =
            snapshot.nativeRenderable &&
            !_pendingGrip.pending && (!_heldTransfer.active() || _heldTransfer.gripAcquired) &&
            !_bridge.isHandPoseHandoffActive();
        snapshot.terminalWeaponFormID = _lastTerminalWeaponFormID;
        snapshot.terminalSource = _lastTerminalSource;
        snapshot.recoveryExhausted =
            _drawExhaustionLogged || _repairExhaustionLogged ||
            (!_active &&
                _lastTerminalResult == TerminalResult::RecoveryExhausted);
        return snapshot;
    }
}

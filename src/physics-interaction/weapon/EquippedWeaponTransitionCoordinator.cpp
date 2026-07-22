#include "physics-interaction/weapon/EquippedWeaponTransitionCoordinator.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/weapon/EquippedWeaponVisualState.h"
#include "physics-interaction/weapon/NativeEquippedWeaponAttach.h"
#include "physics-interaction/weapon/NativeEquippedWeaponDraw.h"
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
        _expectedIdentity = expected;
        _boundIdentity = {};
        _policyState = {};
        _source = source;
        _activeSeconds = 0.0f;
        _active = true;
        _presentationExpected = true;
        _lastNonMenuPresentationExpected = true;
        _waitingForExpectedIdentity = true;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;

        const auto current = readCurrentIdentity();
        _observedIdentity = current;
        _observationInitialized = true;
        if (expectedMatches(current)) {
            bindCurrentIdentity(current, source, "held-equip-current", {}, 0, true);
            auto immediateVisual = equipped_weapon_visual_state::observe(
                current.formID,
                _supersededNativeInstanceNode);
            _bridge.update(EquipVisualBridge::UpdateInput{
                .deltaSeconds = 0.0f,
                .advanceLifetime = false,
                .presentModel = _bridge.hasStandbyModel(),
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
        const auto current = readCurrentIdentity();
        const bool currentStatePresentationExpected =
            equipped_weapon_transition_policy::presentationExpectedFromNativeState(
                input.nativeWeaponState,
                _lastNonMenuPresentationExpected);
        if (!_observationInitialized) {
            _observedIdentity = current;
            _observationInitialized = true;
            _wasMenuBlocking = input.menuBlocking;
            _lastNonMenuPresentationExpected = currentStatePresentationExpected;
            if (input.menuBlocking) {
                _menuEntryIdentity = current;
                _menuEntryNativeInstanceNode = current.valid() ?
                    reinterpret_cast<std::uintptr_t>(
                        equipped_weapon_visual_state::observe(current.formID).exactInstance) :
                    0;
                _menuEntryCaptured = true;
                _menuEntryPresentationExpected = currentStatePresentationExpected ||
                    (_active && _presentationExpected);
                _menuWeaponIdentityMutated = false;
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
            _menuEntryPresentationExpected = _lastNonMenuPresentationExpected ||
                (_active && _presentationExpected);
            _menuWeaponIdentityMutated = false;
        }
        if (!input.menuBlocking) {
            _lastNonMenuPresentationExpected =
                equipped_weapon_transition_policy::presentationExpectedFromNativeState(
                    input.nativeWeaponState,
                    _lastNonMenuPresentationExpected);
        }

        const bool identityChanged = current != _observedIdentity;
        if (identityChanged && input.menuBlocking && _menuEntryCaptured) {
            _menuWeaponIdentityMutated = true;
        }
        if (identityChanged) {
            const auto previous = _observedIdentity;
            _observedIdentity = current;
            if (_waitingForExpectedIdentity && expectedMatches(current)) {
                bindCurrentIdentity(
                    current,
                    _source,
                    "expected-identity-observed",
                    {},
                    0,
                    true);
            } else if (!_waitingForExpectedIdentity && current.valid()) {
                if (_bridge.isActive() && _boundIdentity.valid() && current != _boundIdentity) {
                    _bridge.releaseStandbyModel("equipped-weapon-changed");
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
                    previousNativeInstanceNode,
                    input.menuBlocking && _menuEntryCaptured ?
                        equipped_weapon_transition_policy::presentationExpectedAfterMenu(
                            _menuEntryPresentationExpected,
                            _menuWeaponIdentityMutated,
                            current != _menuEntryIdentity,
                            input.nativeWeaponState,
                            _lastNonMenuPresentationExpected) :
                        currentStatePresentationExpected);
            } else if (!_waitingForExpectedIdentity && !current.valid() && previous == _boundIdentity) {
                finish("weapon-unequipped", true);
            }
        }

        _wasMenuBlocking = input.menuBlocking;
        if (menuClosed && !_waitingForExpectedIdentity && current.valid()) {
            const bool presentationExpected = _menuEntryCaptured ?
                equipped_weapon_transition_policy::presentationExpectedAfterMenu(
                    _menuEntryPresentationExpected,
                    _menuWeaponIdentityMutated,
                    current != _menuEntryIdentity,
                    input.nativeWeaponState,
                    _lastNonMenuPresentationExpected) :
                currentStatePresentationExpected;
            bindCurrentIdentity(
                current,
                Source::MenuExit,
                "menu-closed",
                _menuEntryCaptured ? _menuEntryIdentity : Identity{},
                _menuEntryCaptured ? _menuEntryNativeInstanceNode : 0,
                presentationExpected);
        }
        if (_requestCurrentPending) {
            _requestCurrentPending = false;
            if (current.valid()) {
                const bool presentationExpected = _menuEntryCaptured ?
                    equipped_weapon_transition_policy::presentationExpectedAfterMenu(
                        _menuEntryPresentationExpected,
                        _menuWeaponIdentityMutated,
                        current != _menuEntryIdentity,
                        input.nativeWeaponState,
                        _lastNonMenuPresentationExpected) :
                    currentStatePresentationExpected;
                bindCurrentIdentity(
                    current,
                    _requestedCurrentSource,
                    "explicit-reconcile",
                    _menuEntryCaptured ? _menuEntryIdentity : Identity{},
                    _menuEntryCaptured ? _menuEntryNativeInstanceNode : 0,
                    presentationExpected);
            }
        }
        if (menuClosed) {
            _menuEntryIdentity = {};
            _menuEntryNativeInstanceNode = 0;
            _menuEntryCaptured = false;
            _menuEntryPresentationExpected = false;
            _menuWeaponIdentityMutated = false;
        }

        if (!_active) {
            return;
        }

        if (!input.localSkeletonReady) {
            abandonSceneGraph();
            return;
        }
        if (!input.visualAuthorityAvailable || input.menuBlocking || input.compatibilityBlocking) {
            return;
        }

        const float deltaSeconds = (std::max)(0.0f, input.deltaSeconds);
        _activeSeconds += deltaSeconds;

        if (_waitingForExpectedIdentity) {
            _bridge.update(EquipVisualBridge::UpdateInput{
                .deltaSeconds = deltaSeconds,
                .advanceLifetime = true,
                .presentModel = _bridge.hasStandbyModel(),
                .nativeVisual = nullptr,
            });
            if (_activeSeconds >= kTransitionWatchdogSeconds) {
                finish("expected-identity-timeout", true);
            }
            return;
        }

        if (!current.valid() || current != _boundIdentity) {
            finish("bound-identity-lost", true);
            return;
        }
        if (!_presentationExpected) {
            finish("holstered-presentation-preserved", true);
            return;
        }

        auto visual = equipped_weapon_visual_state::observe(
            _boundIdentity.formID,
            _supersededNativeInstanceNode);
        if (input.nativeWeaponAnimationActive) {
            // Reload/bolt owners deliberately replace or hide the same Weapon
            // graph. Yield both the exact-child cull and the temporary hand
            // pose; a completed equip needs no further watchdog after this
            // explicit ownership transfer.
            _bridge.completeHandPoseHandoff("native-weapon-animation");
            if (_policyState.nativeHandoffObserved) {
                finish("native-weapon-animation-after-handoff", true);
                return;
            }
            _bridge.update(EquipVisualBridge::UpdateInput{
                .deltaSeconds = deltaSeconds,
                .advanceLifetime = true,
                .presentModel = false,
                .nativeVisual = &visual,
            });
            if (_activeSeconds >= kTransitionWatchdogSeconds) {
                finish("native-weapon-animation-timeout", true);
            }
            return;
        }

        const bool weaponExactlyDrawn =
            input.nativeWeaponState == static_cast<std::uint32_t>(
                held_weapon_equip_state_policy::NativeWeaponState::Drawn);
        if (_policyState.nativeHandoffObserved && !weaponExactlyDrawn) {
            finish("weapon-no-longer-drawn", true);
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
        auto decision = equipped_weapon_transition_policy::advance(
            _policyState,
            equipped_weapon_transition_policy::FrameInput{
                .mutationAllowed = true,
                .identityMatches = true,
                .presentationExpected = _presentationExpected,
                .weaponExactlyDrawn = weaponExactlyDrawn,
                .nativeWeaponState = input.nativeWeaponState,
                .bridgeModelAvailable = _bridge.hasStandbyModel(),
                .nativeInstanceFound = exactNativeInstanceIsCurrent,
                .nativeAncestorPathVisible = visual.ancestorPathVisible,
                .nativeInstanceLocallyVisible = visual.instanceLocallyVisible,
                .bridgeOwnsNativeInstanceCull = bridgeOwnsCull,
            });

        switch (decision.repair) {
        case equipped_weapon_transition_policy::RepairAction::RequestDraw: {
            const auto result = native_equipped_weapon_draw::submitExactCurrent(
                native_equipped_weapon_draw::Identity{
                    .formID = _boundIdentity.formID,
                    .instanceData = _boundIdentity.instanceData,
                    .equipIndex = _boundIdentity.equipIndex,
                });
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon transition draw recovery source={} formID={:08X} instance={:#x} attempt={}/{} state={}({})->{}({}) result={}",
                sourceName(_source),
                _boundIdentity.formID,
                _boundIdentity.instanceData,
                _policyState.drawAttempts,
                equipped_weapon_transition_policy::kMaximumDrawAttempts,
                result.stateBefore,
                held_weapon_equip_state_policy::nativeWeaponStateName(result.stateBefore),
                result.stateAfter,
                held_weapon_equip_state_policy::nativeWeaponStateName(result.stateAfter),
                native_equipped_weapon_draw::submitResultName(result.result));
            if (result.result == native_equipped_weapon_draw::SubmitResult::InvalidWeaponState ||
                result.result == native_equipped_weapon_draw::SubmitResult::MissingPlayer ||
                result.result == native_equipped_weapon_draw::SubmitResult::MissingEquippedWeapon) {
                _policyState.drawAttempts =
                    equipped_weapon_transition_policy::kMaximumDrawAttempts;
            }
            break;
        }
        case equipped_weapon_transition_policy::RepairAction::DrawExhausted:
            if (!_drawExhaustionLogged) {
                _drawExhaustionLogged = true;
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon transition draw recovery exhausted source={} formID={:08X} instance={:#x} weaponState={}({})",
                    sourceName(_source),
                    _boundIdentity.formID,
                    _boundIdentity.instanceData,
                    input.nativeWeaponState,
                    held_weapon_equip_state_policy::nativeWeaponStateName(
                        input.nativeWeaponState));
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
                ROCK_LOG_WARN(Weapon,
                    "Equipped weapon transition recovery exhausted source={} formID={:08X} instance={:#x}",
                    sourceName(_source),
                    _boundIdentity.formID,
                    _boundIdentity.instanceData);
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
        });
        if (_activeSeconds >= kTransitionWatchdogSeconds) {
            finish("watchdog-complete", true);
        }
    }

    void EquippedWeaponTransitionCoordinator::shutdown()
    {
        _bridge.shutdown();
        _policyState = {};
        _observedIdentity = {};
        _boundIdentity = {};
        _menuEntryIdentity = {};
        _expectedIdentity = {};
        _menuEntryNativeInstanceNode = 0;
        _supersededNativeInstanceNode = 0;
        _activeSeconds = 0.0f;
        _observationInitialized = false;
        _active = false;
        _presentationExpected = false;
        _waitingForExpectedIdentity = false;
        _requestCurrentPending = false;
        _wasMenuBlocking = false;
        _menuEntryCaptured = false;
        _menuEntryPresentationExpected = false;
        _menuWeaponIdentityMutated = false;
        _lastNonMenuPresentationExpected = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
    }

    void EquippedWeaponTransitionCoordinator::abandonSceneGraph()
    {
        _bridge.abandonSceneGraph();
        _policyState = {};
        _boundIdentity = {};
        _menuEntryIdentity = {};
        _expectedIdentity = {};
        _menuEntryNativeInstanceNode = 0;
        _supersededNativeInstanceNode = 0;
        _activeSeconds = 0.0f;
        _active = false;
        _presentationExpected = false;
        _waitingForExpectedIdentity = false;
        _requestCurrentPending = false;
        _wasMenuBlocking = false;
        _menuEntryCaptured = false;
        _menuEntryPresentationExpected = false;
        _menuWeaponIdentityMutated = false;
        _lastNonMenuPresentationExpected = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
    }

    EquippedWeaponTransitionCoordinator::Identity
    EquippedWeaponTransitionCoordinator::readCurrentIdentity() noexcept
    {
        auto* equipped = f4vr::getEquippedItem();
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
        std::uintptr_t previousNativeInstanceNode,
        const bool presentationExpected)
    {
        if (!identity.valid()) {
            return;
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
        _active = true;
        _presentationExpected = presentationExpected;
        _waitingForExpectedIdentity = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon transition bound source={} reason={} formID={:08X} instance={:#x} equipIndex={} presentationExpected={}",
            sourceName(source),
            reason ? reason : "unknown",
            identity.formID,
            identity.instanceData,
            identity.equipIndex,
            presentationExpected ? "yes" : "no");
    }

    void EquippedWeaponTransitionCoordinator::finish(
        const char* reason,
        const bool releaseSceneGraph)
    {
        if (releaseSceneGraph) {
            _bridge.releaseStandbyModel(reason);
        } else {
            _bridge.abandonSceneGraph();
        }
        _policyState = {};
        _boundIdentity = {};
        _expectedIdentity = {};
        _supersededNativeInstanceNode = 0;
        _activeSeconds = 0.0f;
        _active = false;
        _presentationExpected = false;
        _waitingForExpectedIdentity = false;
        _lateRecoveryWindowGranted = false;
        _drawExhaustionLogged = false;
        _repairExhaustionLogged = false;
    }
}

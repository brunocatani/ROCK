#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/weapon/WeaponEquipTransfer.h"

namespace rock
{
    const char* PhysicsInteraction::bareFistHandBlockReason(const PhysicsFrameContext& frame) const
    {
        if (!frame.worldReady) return "world-unavailable";
        if (frame.menuBlocked) return "menu";
        if (frame.reloadBoundaryActive) return "reload-boundary";
        if (frame.left.disabled || frame.right.disabled) return "hand-disabled";
        if (_equipped.transition.getPublicSnapshot().active) return "weapon-transition";
        if (_equipped.transition.pendingGrip().pending) return "pending-grip";
        if (provider::currentNativeAnimationAuthorityFlagsV1() != 0) return "native-animation-owner";
        const auto grips = _twoHandedGrip.getGripOccupancy();
        if (grips.left.weaponEngaged() || grips.right.weaponEngaged()) return "weapon-hand-occupied";
        const auto* frik = frik_visual_authority::api();
        if (!frik || !frik->getHandPoseSetTagState) return "frik-pose-api";
        for (const bool left : { false, true }) {
            const Hand& hand = left ? _leftHand : _rightHand;
            const auto index = left ? 1u : 0u;
            const auto state = hand.getState();
            if (hand.isHolding()) return "object-held";
            if (hand.hasActivePullCatchIntent()) return "pull-active";
            if (hand.hasPendingActorEquipmentDropHandoff()) return "equipment-drop-handoff";
            if (hand.hasPendingPullCatchCommit()) return "pull-commit";
            if (_forceGrab.pendingCommits[index].active) return "force-grab-pending";
            if (_forceGrab.retainedWeaponGrabs[index].grabIdentity != 0) return "retained-weapon";
            if (_touchGrabRuntime.isHandActive(left)) return "touch-grab";
            if (state != HandState::Idle && state != HandState::SelectedClose && state != HandState::SelectedFar)
                return "hand-state";
            const auto physicalHand = left ? provider::RockProviderHand::Left : provider::RockProviderHand::Right;
            constexpr auto inputMask = static_cast<std::uint32_t>(provider::RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
                static_cast<std::uint32_t>(provider::RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput);
            if ((provider::currentHandInputSuppressionFlagsV1(physicalHand) & inputMask) != 0) return "provider-suppression";
            // Immersive Flashlight publishes this tag while actually carrying
            // its mesh. Both active and overridden tags still own the object.
            const auto frikHand = left ? frik_visual_authority::Hand::Left : frik_visual_authority::Hand::Right;
            if (frik->getHandPoseSetTagState("ImFl_Hold", frikHand) != frik_visual_authority::HandPoseTagState::None)
                return "flashlight-held";
            if (frik->getHandPoseSetTagState("InFl_Config", frikHand) != frik_visual_authority::HandPoseTagState::None)
                return "flashlight-config";
        }
        return nullptr;
    }

    void PhysicsInteraction::cancelBareFistMode(const char* reason)
    {
        input_remap_runtime::setBareFistAdmission(false);
        const auto previous = _grabInput.bareFistGesture.phase;
        _grabInput.bareFistGesture = {};
        if (previous != bare_fist_gesture::Phase::Idle) {
            _grabInput.bareFistGuardState = {};
            ROCK_LOG_INFO(Weapon, "Bare fists ended: reason={} phase={}", reason, static_cast<unsigned>(previous));
        }
        if (_grabInput.bareFistDrawOwned) {
            auto* player = RE::PlayerCharacter::GetSingleton();
            const bool sameWorld = _grabInput.bareFistWorldGeneration ==
                _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
            if (!sameWorld || currentEquippedWeaponFormId() != 0 ||
                (player && f4vr::getNativeWeaponState(player) == 0)) {
                _grabInput.bareFistDrawOwned = false;
                _grabInput.bareFistHolsterRequested = false;
            } else if (player && !_grabInput.bareFistHolsterRequested) {
                _grabInput.bareFistHolsterRequested = true;
                player->DrawWeaponMagicHands(false);
            }
        }
        input_remap_runtime::setBareFistDrawState(0, _grabInput.bareFistDrawOwned, false);
    }

    void PhysicsInteraction::updateBareFistMode(const PhysicsFrameContext& frame)
    {
        auto* player = RE::PlayerCharacter::GetSingleton();
        const auto& runtime = runtime_state::currentFrame();
        const bool gestureStarted = _grabInput.bareFistGesture.phase != bare_fist_gesture::Phase::Idle;
        const auto nativeState = f4vr::getNativeWeaponState(player);
        const auto weaponForm = currentEquippedWeaponFormId();
        const bool ownsDraw = _grabInput.bareFistDrawOwned &&
            _grabInput.bareFistWorldGeneration == _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
        const char* denial = !g_rockConfig.rockRockyModeEnabled ? "mode-disabled" :
            !player ? "player-unavailable" : player->IsDead(false) ? "player-dead" :
            !g_rockConfig.rockEnableVanillaMelee ? "vanilla-melee-disabled" :
            runtime.compatibilityConfigBlocking ? "config-mode" :
            runtime.localGameStopped ? "game-stopped" : !runtime.localSkeletonReady ? "skeleton-unavailable" :
            !(nativeState == 0 || (ownsDraw && weaponForm == 0)) ? "native-weapon-state" :
            !input_remap_runtime::bareFistHooksReady() ? "input-hooks" :
            !areNativeMeleeHooksInstalled() ? "melee-hooks" :
            isNativeMeleeSuppressionActive() ? "melee-suppressed" :
            bareFistHandBlockReason(frame);
        if (!denial && _grabInput.bareFistDrawOwned && !gestureStarted) denial = "fist-holster-pending";
        const bool eligible = denial == nullptr;
        input_remap_runtime::setBareFistAdmission(eligible);
        if (g_rockConfig.rockRockyModeEnabled) {
            const auto input = input_remap_runtime::readBareFistInputSnapshot();
            const auto& buttons = input.buttons;
            const bool physicalChord = (buttons.leftBits & 3u) == 3u && (buttons.rightBits & 3u) == 3u;
            if ((!eligible || _grabInput.bareFistGesture.phase != bare_fist_gesture::Phase::Active) &&
                (physicalChord || gestureStarted)) {
                const auto capture = bare_fist_gesture::capture(input.cycle);
                const auto* reason = denial ? denial : !buttons.fresh ? "controller-sample-stale" :
                    buttons.blocked ? "ui-or-button-rearm" : !buttons.held ? "chord-released" :
                    (capture == bare_fist_gesture::Capture::Idle || capture == bare_fist_gesture::Capture::ReadyToRetry) ? "waiting-for-capture" :
                    capture != bare_fist_gesture::Capture::Holding ? "waiting-for-chord-break" :
                    "qualifying";
                ROCK_LOG_SAMPLE_INFO(Weapon, 2000,
                    "Rocky activation: reason={} phase={} capture={} cycle={} bitsL/R={}/{} ageMsL/R={}/{} admissionFresh={} hold={:.3f}/{:.3f} nativeState={} equipped={:08X} handsL/R={}/{}",
                    reason, static_cast<unsigned>(_grabInput.bareFistGesture.phase),
                    static_cast<unsigned>(bare_fist_gesture::capture(input.cycle)), input.cycle,
                    static_cast<unsigned>(buttons.leftBits), static_cast<unsigned>(buttons.rightBits),
                    buttons.leftAgeMilliseconds, buttons.rightAgeMilliseconds, input.admissionFresh,
                    _grabInput.bareFistGesture.seconds,
                    gestureStarted ? _grabInput.bareFistGesture.requiredHoldSeconds : g_rockConfig.rockRockyModeHoldSeconds,
                    nativeState, weaponForm,
                    static_cast<unsigned>(_leftHand.getState()), static_cast<unsigned>(_rightHand.getState()));
            }
        }
        if (!eligible) {
            cancelBareFistMode("eligibility-lost");
            return;
        }
        const auto previous = _grabInput.bareFistGesture.phase;
        const bool drawnUnarmed = ownsDraw && weaponForm == 0 && nativeState == 3 &&
            f4vr::CombatUtilities_IsActorUsingMelee(player);
        const auto action = bare_fist_gesture::update(_grabInput.bareFistGesture, {
            .cycle = input_remap_runtime::bareFistInputCycle(),
            .eligible = input_remap_runtime::bareFistChordValid(),
            .drawnUnarmed = drawnUnarmed,
            .deltaSeconds = frame.deltaSeconds,
            .holdSeconds = g_rockConfig.rockRockyModeHoldSeconds,
        });
        if (action == bare_fist_gesture::Action::Cancel) {
            // The policy clears its state on cancellation; preserve its prior
            // phase only for the transition diagnostic and common cleanup.
            _grabInput.bareFistGesture.phase = previous;
            cancelBareFistMode("release-or-draw-failure");
            return;
        }
        if (action == bare_fist_gesture::Action::Draw) {
            if (!weapon_equip_transfer::replaceHolsteredWeaponWithUnarmed() ||
                !input_remap_runtime::bareFistChordValid()) {
                cancelBareFistMode("equipment-replacement-refused");
                return;
            }
            _grabInput.bareFistDrawOwned = true;
            _grabInput.bareFistHolsterRequested = false;
            _grabInput.bareFistWorldGeneration = _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire);
            input_remap_runtime::setBareFistDrawState(_grabInput.bareFistGesture.cycle, true, false);
            player->DrawWeaponMagicHands(true);
        }
        const auto phase = _grabInput.bareFistGesture.phase;
        if (previous != phase) {
            _grabInput.bareFistGuardState = {};
            ROCK_LOG_INFO(Weapon, "Bare fists transition: {} -> {} cycle={} nativeState={} equipped={:08X}",
                static_cast<unsigned>(previous), static_cast<unsigned>(phase),
                _grabInput.bareFistGesture.cycle, nativeState, weaponForm);
        }
        input_remap_runtime::setBareFistDrawState(_grabInput.bareFistGesture.cycle,
            _grabInput.bareFistDrawOwned, phase == bare_fist_gesture::Phase::Active);
    }
}

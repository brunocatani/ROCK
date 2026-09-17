#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/weapon/WeaponEquipTransfer.h"

namespace rock
{
    bool PhysicsInteraction::bareFistHandsAvailable(const PhysicsFrameContext& frame) const
    {
        if (!frame.worldReady || frame.menuBlocked || frame.reloadBoundaryActive ||
            frame.left.disabled || frame.right.disabled ||
            _equipped.transition.getPublicSnapshot().active ||
            _equipped.pendingPrimaryOnlyGripStart.pending ||
            provider::currentNativeAnimationAuthorityFlagsV1() != 0) return false;
        const auto grips = _twoHandedGrip.getGripOccupancy();
        if (grips.left.weaponEngaged() || grips.right.weaponEngaged()) return false;
        const auto* frik = frik_visual_authority::api();
        if (!frik || !frik->getHandPoseSetTagState) return false;
        for (const bool left : { false, true }) {
            const Hand& hand = left ? _leftHand : _rightHand;
            const auto index = left ? 1u : 0u;
            const auto state = hand.getState();
            if ((state != HandState::Idle && state != HandState::SelectedClose && state != HandState::SelectedFar) ||
                hand.isHolding() || hand.hasActivePullCatchIntent() ||
                hand.hasPendingActorEquipmentDropHandoff() || hand.hasPendingPullCatchCommit() ||
                _forceGrab.pendingCommits[index].active ||
                _forceGrab.retainedWeaponGrabs[index].grabIdentity != 0 ||
                _touchGrabRuntime.isHandActive(left)) return false;
            const auto physicalHand = left ? provider::RockProviderHand::Left : provider::RockProviderHand::Right;
            constexpr auto inputMask = static_cast<std::uint32_t>(provider::RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord) |
                static_cast<std::uint32_t>(provider::RockProviderHandInputSuppressionFlagV1::SuppressOpenVrGameInput);
            if ((provider::currentHandInputSuppressionFlagsV1(physicalHand) & inputMask) != 0) return false;
            // Immersive Flashlight publishes this tag while actually carrying
            // its mesh. Both active and overridden tags still own the object.
            const auto frikHand = left ? frik_visual_authority::Hand::Left : frik_visual_authority::Hand::Right;
            if (frik->getHandPoseSetTagState("ImFl_Hold", frikHand) != frik_visual_authority::HandPoseTagState::None ||
                frik->getHandPoseSetTagState("InFl_Config", frikHand) != frik_visual_authority::HandPoseTagState::None) return false;
        }
        return true;
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
        const bool eligible = player && !player->IsDead(false) && g_rockConfig.rockEnableVanillaMelee &&
            !runtime.compatibilityConfigBlocking && !runtime.localGameStopped && runtime.localSkeletonReady &&
            (nativeState == 0 || (ownsDraw && weaponForm == 0)) &&
            input_remap_runtime::bareFistHooksReady() && areNativeMeleeHooksInstalled() &&
            !isNativeMeleeSuppressionActive() && bareFistHandsAvailable(frame) &&
            (!_grabInput.bareFistDrawOwned || gestureStarted);
        input_remap_runtime::setBareFistAdmission(eligible);
        if (!eligible) {
            if (input_remap_runtime::isRawButtonPhysicallyHeld(true, 2) &&
                input_remap_runtime::isRawButtonPhysicallyHeld(true, 33) &&
                input_remap_runtime::isRawButtonPhysicallyHeld(false, 2) &&
                input_remap_runtime::isRawButtonPhysicallyHeld(false, 33)) {
                ROCK_LOG_SAMPLE_INFO(Weapon, 2000,
                    "Bare fists admission denied: enabled={} nativeState={} equipped={:08X} hands={}/{} touch={}/{} transition={} pendingGrip={} menu={} skeleton={} hooks={}/{}",
                    g_rockConfig.rockEnableVanillaMelee, nativeState, weaponForm,
                    static_cast<unsigned>(_leftHand.getState()), static_cast<unsigned>(_rightHand.getState()),
                    _touchGrabRuntime.isHandActive(true), _touchGrabRuntime.isHandActive(false),
                    _equipped.transition.getPublicSnapshot().active, _equipped.pendingPrimaryOnlyGripStart.pending,
                    frame.menuBlocked || runtime.compatibilityConfigBlocking, runtime.localSkeletonReady,
                    input_remap_runtime::bareFistHooksReady(), areNativeMeleeHooksInstalled());
            }
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

// Equipped-weapon frame: transitions, the per-frame equipped weapon update,
// authored primary grip runtime, handling settings, and the shoulder stash.
#include "api/AuthoredInteractionService.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapture.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/input/PhysicalReloadInput.h"
#include "physics-interaction/input/SupportTriggerInput.h"
#include "physics-interaction/visual/GripZoneIndicators.h"
#include "physics-interaction/weapon/authored/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/authored/HoldingGripRecords.h"
#include "physics-interaction/weapon/authored/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/authored/VanillaWeaponGripFrame.h"
#include "physics-interaction/weapon/classification/WeaponInstanceClassification.h"
#include "physics-interaction/weapon/classification/WeaponTypePolicy.h"
#include "physics-interaction/weapon/equip/EquipVisualBridgePolicy.h"
#include "physics-interaction/weapon/grip/KnifeReverseGrip.h"
#include "physics-interaction/weapon/grip/WeaponGripSeat.h"
#include "physics-interaction/weapon/handling/WeaponInputFilter.h"
#include "physics-interaction/weapon/native/NativeEquippedModelLifetime.h"
#include "physics-interaction/weapon/native/NativeHandEquipment.h"
#include "physics-interaction/weapon/native/TriggerEquipReadinessTrace.h"
#include "physics-interaction/weapon/scene/WeaponScenePresentation.h"
#include "physics-interaction/weapon/scope/ManualScopeTargetPolicy.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryActions.h"
#include "physics-interaction/weapon/secondary/NativeSecondaryPresentation.h"

namespace rock
{
    namespace
    {
        void maskProviderWeaponGrabInput(bool isLeft, bool holding, GrabButtonState& button)
        {
            using Flag = ::rock::api::input::HandInputSuppressionFlag;
            const auto flags = provider::currentHandInputSuppressionFlags(isLeft ? api::Hand::Left : api::Hand::Right);
            const bool press = provider::hasHandInputSuppressionFlag(flags, Flag::SuppressNormalGrabPress);
            const bool release = provider::hasHandInputSuppressionFlag(flags, Flag::SuppressGrabRelease);
            if (!press && !release)
                return;
            const auto filtered = input_remap_policy::suppressGrabInput(
                { .grabHeld = button.held, .grabPressed = button.pressed, .grabReleased = button.released }, press,
                release, holding);
            button = { .held = filtered.grabHeld, .pressed = filtered.grabPressed, .released = filtered.grabReleased };
            // The UI reads raw levels. Drain gameplay edges so opening the
            // wheel cannot become a delayed detach after its lease is cleared.
            static_cast<void>(input_remap_runtime::consumeRawButtonState(isLeft, input_remap_policy::kGrabButtonId));
        }

        // The nodes below `object` with their locals, bounded for a modded model.
        template <class Parts>
        void captureMenuHoldParts(RE::NiAVObject* object, Parts& parts, std::uint32_t depth = 0)
        {
            constexpr std::uint32_t kMaxDepth = 32;
            constexpr std::size_t kMaxParts = 256;
            auto* node = object ? object->IsNode() : nullptr;
            if (!node || depth >= kMaxDepth)
                return;
            for (const auto& child : node->children) {
                if (!child || !child->IsNode() || parts.size() >= kMaxParts)
                    continue;
                parts.push_back({ RE::NiPointer<RE::NiAVObject>(child.get()), child->local });
                captureMenuHoldParts(child.get(), parts, depth + 1);
            }
        }
    }

    bool PhysicsInteraction::tryGetManualScopePresentationTarget(std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex, bool& outDirectTransitionRequired, const void* expectedWeapon,
        const void* expectedInstance) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        outDirectTransitionRequired = false;
        if (!g_rockConfig.rockEnableImmersiveScopes || !_lifecycle.initialized.load(std::memory_order_acquire) ||
            !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto snapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
        if (!snapshot.scopeEligible) {
            // Native classification also runs while idle. Ordinary sights
            // are an expected negative result, not a failed scope anchor.
            return false;
        }
        if (expectedWeapon &&
            !manual_scope_target_policy::matchesNativeIdentity(snapshot.scopeWeaponIdentity,
                snapshot.scopeInstanceIdentity, reinterpret_cast<std::uintptr_t>(expectedWeapon),
                reinterpret_cast<std::uintptr_t>(expectedInstance))) {
            ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                "Native scope admission identity rejected: publishedWeapon=0x{:X} instance=0x{:X} nativeWeapon=0x{:X} "
                "instance=0x{:X}",
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
        const NativeScopeResolvedAnchorSnapshot resolvedAnchor = _twoHandedGrip.getNativeScopeResolvedAnchorSnapshot();
        const native_scope_sight_anchor_policy::PublicationIdentity resolvedIdentity{
            .weaponGenerationKey = resolvedAnchor.weaponGenerationKey,
            .equippedWeaponOwnershipKey = resolvedAnchor.equippedWeaponOwnershipKey,
            .weaponFormID = resolvedAnchor.weaponFormID,
        };
        if (!resolvedAnchor.valid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(resolvedIdentity, currentIdentity) ||
            !snapshot.nativeScopeOverlayValid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(publishedIdentity, currentIdentity)) {
            if (expectedWeapon) {
                ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000,
                    "Native scope admission target rejected: anchorValid={} eligible={} overlayValid={} "
                    "publishedGeneration={:016X} resolvedGeneration={:016X} currentGeneration={:016X}",
                    resolvedAnchor.valid, snapshot.scopeEligible, snapshot.nativeScopeOverlayValid,
                    snapshot.weaponGenerationKey, resolvedAnchor.weaponGenerationKey,
                    currentIdentity.weaponGenerationKey);
            }
            return false;
        }
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        outDirectTransitionRequired = snapshot.manualDirectTransitionRequired;
        return true;
    }

    bool PhysicsInteraction::tryGetSecondaryScopeTarget(std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex, float& outMagnification, RE::NiPoint3* outAnchorModelLocal) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        outMagnification = 0.0f;
        if (!g_rockConfig.rockEnableImmersiveScopes || !_lifecycle.initialized.load(std::memory_order_acquire) ||
            !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto view = native_hand_equipment::equippedViews()[1];
        if (!view.weapon)
            return false;
        // The index-1 collision generates from the presented model, so its
        // sight anchor is local to that model.
        const auto snapshot = _secondaryWeaponCollision.getNativeScopeSightAnchorSnapshot();
        if (!snapshot.valid || !snapshot.scopeEligible || !snapshot.nativeScopeOverlayValid ||
            snapshot.weaponGenerationKey == 0 ||
            snapshot.weaponGenerationKey != _secondaryWeaponCollision.getCurrentWeaponGenerationKey() ||
            !manual_scope_target_policy::matchesNativeIdentity(snapshot.scopeWeaponIdentity,
                snapshot.scopeInstanceIdentity, reinterpret_cast<std::uintptr_t>(view.weapon),
                reinterpret_cast<std::uintptr_t>(view.instance))) {
            return false;
        }
        // The same ZOOM the mono renderer reads: instance first, then base.
        const auto* instance =
            view.instance ? static_cast<const RE::TESObjectWEAP::InstanceData*>(view.instance) : nullptr;
        const RE::BGSZoomData* zoom = instance ? instance->zoomData : nullptr;
        if (!zoom)
            zoom = view.weapon->weaponData.zoomData;
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        outMagnification = zoom ? zoom->zoomData.fovMult : 0.0f;
        if (outAnchorModelLocal)
            *outAnchorModelLocal = snapshot.anchorWeaponLocal;
        return true;
    }

    bool PhysicsInteraction::isScopeEligible(const std::int32_t slot) const
    {
        if (slot == 1) {
            std::uint64_t generation = 0;
            std::uint32_t overlay = 0;
            float magnification = 0.0f;
            return tryGetSecondaryScopeTarget(generation, overlay, magnification);
        }
        return slot == 0 && _weaponCollision.getNativeScopeSightAnchorSnapshot().scopeEligible;
    }

    void PhysicsInteraction::synchronizeSecondaryScopePresentation()
    {
        if (!_lifecycle.initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady())
            return;
        RE::NiPoint3 anchor{};
        std::uint64_t generation = 0;
        std::uint32_t overlay = 0;
        float magnification = 0.0f;
        auto* model = native_secondary_presentation::presentedModel();
        const bool held = input_remap_runtime::manualScopeSlot() == 1 && model && model->IsNode() &&
            tryGetSecondaryScopeTarget(generation, overlay, magnification, &anchor);
        _twoHandedGrip.synchronizeSecondaryScopePresentation(held ? model->IsNode() : nullptr, generation, anchor);
    }

    EquippedWeaponTransitionCoordinator::PendingGrip PhysicsInteraction::equippedWeaponContinuity() const
    {
        return _equipped.continuityGrip;
    }

    void PhysicsInteraction::restoreEquippedWeaponContinuity(
        const EquippedWeaponTransitionCoordinator::PendingGrip& grip)
    {
        _equipped.continuityGrip = grip;
        _equipped.gripResumePending = grip.pending;
        if (grip.pending) {
            ROCK_LOG_INFO(Weapon, "Equipped grip suspended form={:08X} hand={} paired={} support={} secondSupport={}",
                grip.targetWeaponFormID, grip.isLeft ? "left" : "right", grip.pairedGrips.valid(),
                grip.supportGrip.validCarry(), grip.secondSupportGrip.validCarry());
        }
    }

    void PhysicsInteraction::applyPipboySupportRequest()
    {
        // Covers a promotion's re-equip and draw after the menu closes.
        constexpr float kDeferredAttachSeconds = 5.0f;
        const auto request = native_hand_equipment::pendingSupportRequest();
        if (!request.sequence)
            return;
        const auto& frame = runtime_state::currentFrame();
        const bool menu = frame.localMenuBlocking;
        const auto sameItem = [&](const EquippedWeaponTransitionCoordinator::PendingGrip& grip) {
            return grip.targetWeaponFormID != 0 && grip.targetWeaponFormID == request.form &&
                (!grip.targetWeaponInstanceData || !request.instance ||
                    grip.targetWeaponInstanceData == request.instance);
        };
        auto& grip = _equipped.continuityGrip;
        const bool manual = _twoHandedGrip.isManualOwnershipActive();
        const char* refused = nullptr;
        const char* applied = menu ? "applied-on-menu-exit" : "applied";
        if (request.attach) {
            // In a menu: the carry saved when it opened. After a menu that
            // equipped the weapon: the carry ROCK started for it since.
            auto carry = menu ? (grip.pending ? grip : _equipped.nativeCarryCapture)
                : manual      ? grip
                              : _equipped.nativeCarryCapture;
            const bool settling = !menu && (_equipped.gripResumePending || _equipped.transition.pendingGrip().pending);
            if (!carry.pending || !sameItem(carry) || settling) {
                // The weapon may still be moving: equipped in this menu, or
                // promoted from the second slot once it closes.
                if (_equipped.deferredSupportSequence != request.sequence) {
                    _equipped.deferredSupportSequence = request.sequence;
                    _equipped.deferredSupportSeconds = 0.0f;
                    ROCK_LOG_INFO(Weapon, "Pip-Boy support attach weapon={:08X} deferred until its carry starts",
                        request.form);
                }
                if (!menu)
                    _equipped.deferredSupportSeconds += frame.deltaSeconds;
                if (menu || _equipped.deferredSupportSeconds < kDeferredAttachSeconds)
                    return;
                auto* weapon = currentEquippedWeaponForm();
                refused = weapon && weapon->formID == request.form ? "carry-did-not-start" : "weapon-not-equipped";
            } else if (carry.supportGrip.validCarry()) {
                refused = "weapon-support-held";
            } else if (!menu && carry.pairedGrips.valid()) {
                applied = "already-supported";
            } else if (!carry.pairedGrips.valid() && !carry.supportAttach.valid()) {
                refused = "authored-support-grip-unavailable";
            } else {
                if (!carry.pairedGrips.valid())
                    carry.pairedGrips = carry.supportAttach;
                // Outside a menu the paired resume starts its own grip session.
                if (!menu && manual)
                    _twoHandedGrip.reset();
                grip = carry;
                _equipped.gripResumePending = true;
            }
        } else if (!menu || !grip.pending || !sameItem(grip) || !grip.pairedGrips.valid()) {
            refused = "no-support-to-detach";
        } else {
            // The firing-only resume keeps the firing grip captured with it.
            grip.supportAttach = grip.pairedGrips;
            grip.pairedGrips = {};
        }
        _equipped.deferredSupportSequence = 0;
        native_hand_equipment::noteSupportRequestApplied(request.sequence, refused ? !request.attach : request.attach);
        ROCK_LOG_INFO(Weapon, "Pip-Boy support {} weapon={:08X} result={}", request.attach ? "attach" : "detach",
            request.form, refused ? refused : applied);
    }

    void PhysicsInteraction::holdEquippedWeaponInMenu()
    {
        constexpr const char* kTag = "ROCK.MenuWeaponHold";
        constexpr int kPriority = 100;
        // Above the live grip claims a menu frame may not have cleared yet.
        constexpr int kHandPriority = 120;
        const auto releaseHand = [this]() {
            if (_equipped.menuHoldHandClaim < 0)
                return;
            (void)frik_visual_authority::clearHandWorld(kTag,
                frik_visual_authority::handFromBool(_equipped.menuHoldHandClaim == 1));
            _equipped.menuHoldHandClaim = -1;
        };
        const auto releaseSupport = [this]() {
            if (_equipped.menuHoldSupportHand < 0)
                return;
            const auto supportHand = frik_visual_authority::handFromBool(_equipped.menuHoldSupportHand == 1);
            (void)frik_visual_authority::clearHandWorld(kTag, supportHand);
            (void)frik_visual_authority::clearHandPose(kTag, supportHand);
            _equipped.menuHoldSupportHand = -1;
        };
        const auto release = [this, &releaseHand, &releaseSupport]() {
            releaseSupport();
            releaseHand();
            if (_equipped.menuHoldFingerHand < 0)
                return;
            (void)frik_visual_authority::clearHandPose(kTag,
                frik_visual_authority::handFromBool(_equipped.menuHoldFingerHand == 1));
            _equipped.menuHoldFingerHand = -1;
        };
        // A menu resets ROCK's grip session and its FRIK blocks. Without them
        // FRIK poses the hand for the previous weapon and returns the weapon
        // node to the native glue, which turns the hand to the authored grip.
        // The hold keeps both blocked so the hand stays on its controller.
        const auto setPoseBlocked = [this](bool blocked) {
            if (_equipped.menuHoldPoseBlocked == blocked)
                return;
            const bool pose = frik_visual_authority::blockPrimaryHandWeaponPose(kTag, blocked);
            const bool node = frik_visual_authority::blockPrimaryWeaponNodeOwnership(kTag, blocked);
            if (pose && node)
                _equipped.menuHoldPoseBlocked = blocked;
        };
        auto& reveal = _equipped.menuEquipReveal;
        const auto endReveal = [&reveal]() {
            if (reveal.culled)
                equipped_weapon_visual_state::setLocallyVisible(reveal.culled.get(), true);
            reveal = {};
        };
        const auto& frame = runtime_state::currentFrame();
        const auto view = native_hand_equipment::equippedViews()[0];
        // A firearm aims by the controller basis; a melee weapon follows the
        // wrist, as each does in the live carry.
        const bool melee = view.weapon && !view.firearm;
        /*
         * After a menu the hold lets go only once the weapon is drawn and its
         * live grip has the hand. A weapon first equipped in the menu has no
         * grip for the first drawn frames (its capture needs the identified
         * instance); without a claim the hand and weapon stepped 17 degrees for
         * those frames. The hold and the live grip seat the hand identically,
         * so the handoff shows nothing.
         */
        constexpr float kLiveGripHandoffMaxSeconds = 0.5f;
        const bool firstHoldThisFrame = _equipped.menuHoldDecidedFrame != frame.frameIndex;
        if (firstHoldThisFrame) {
            _equipped.menuHoldDecidedFrame = frame.frameIndex;
            bool holding = frame.menuHoldsWeapons();
            if (holding) {
                _equipped.menuHoldHandoffSeconds = 0.0f;
            } else if (_equipped.menuHolding && view.weapon && frame.weaponDrawn &&
                !frame.compatibilityConfigBlocking && !_twoHandedGrip.isLiveGripPresenting() &&
                // A native reload or an object in the hand keeps the live grip away.
                !(provider::currentNativeAnimationAuthorityFlags() & authored_weapon_grip_capture_policy::kWeapon) &&
                !(_equipped.menuHoldHandClaim == 1 ? _leftHand : _rightHand).isHolding()) {
                _equipped.menuHoldHandoffSeconds += frame.deltaSeconds;
                holding = _equipped.menuHoldHandoffSeconds <= kLiveGripHandoffMaxSeconds;
                if (!holding)
                    ROCK_LOG_WARN(Weapon, "MENU_WEAPON_HOLD weapon={:08X} live grip absent after {:.2f}s; hold released",
                        view.weapon->formID, _equipped.menuHoldHandoffSeconds);
            }
            if (_equipped.menuHolding && !holding && _equipped.menuHoldHandoffSeconds > 0.0f)
                ROCK_LOG_INFO(Weapon, "MENU_WEAPON_HOLD weapon={:08X} handed to the live grip after {:.3f}s",
                    view.weapon ? view.weapon->formID : 0u, _equipped.menuHoldHandoffSeconds);
            _equipped.menuHolding = holding;
        }
        if (!_equipped.menuHolding) {
            endReveal();
            release();
            setPoseBlocked(false);
            _equipped.menuHoldParts.clear();
            _equipped.menuHoldPartsRoot = 0;
            return;
        }
        setPoseBlocked(view.weapon != nullptr);
        // The native drawn state can drop inside menus; the equipped item decides.
        auto* node = view.weapon ? resolveEquippedWeaponInteractionNodeDirect() : nullptr;
        const auto& carry = _equipped.continuityGrip.pending ? _equipped.continuityGrip : _equipped.nativeCarryCapture;
        const bool carried = carry.pending && view.weapon && view.weapon->formID == carry.targetWeaponFormID;
        /*
         * A weapon equipped inside this menu while the game runs (the wrist
         * Pip-Boy) stays hidden until its draw has posed it. Its graph runs at
         * once and, before the equip clip's first firearm pose, outputs the
         * part bones at their origin; shown then, the parts assembled on
         * screen (runtime 2 Nov 11:30:38: shown 20 ms before that pose). The
         * native equip hide covers this only when native applies it, never
         * when one of two worn weapons changes, so the hold hides the node
         * itself if needed. A paused game (the projected Pip-Boy) never
         * evaluates that graph and shows the model's assembled parts at once,
         * as does a weapon carried into the menu. A melee model has no parts
         * to assemble. The exit's settling keeps a reveal still running; it
         * starts none.
         */
        bool awaitingPose = false;
        const bool revealApplies =
            frame.localMenuBlocking ? !frame.localGamePaused : reveal.formID != 0 && !reveal.revealed;
        if (node && !carried && !melee && revealApplies) {
            const auto instance = reinterpret_cast<std::uintptr_t>(view.instance);
            if (reveal.formID != view.weapon->formID || reveal.instance != instance) {
                endReveal();
                reveal.formID = view.weapon->formID;
                reveal.instance = instance;
                reveal.seenAt = std::chrono::steady_clock::now();
            }
            if (!reveal.revealed) {
                auto* player = f4vr::getPlayer();
                const bool drawn = f4vr::getNativeWeaponState(player) ==
                    static_cast<std::uint32_t>(held_weapon_equip_state_policy::NativeWeaponState::Drawn);
                reveal.drawStarted |= !drawn;
                const float waited =
                    std::chrono::duration<float>(std::chrono::steady_clock::now() - reveal.seenAt).count();
                const bool posed = weapon_transition_animation_acceleration::hasGeneratedFirearmPose(player,
                    { reveal.formID, reveal.instance, 0 });
                const bool drawComplete = reveal.drawStarted && drawn;
                // Backstop for a draw that never runs: logged, never silent.
                const bool timedOut = waited >= weapon_transition_animation_acceleration_policy::kLeaseTimeoutSeconds;
                if (posed || drawComplete || timedOut) {
                    if (posed || drawComplete) {
                        ROCK_LOG_INFO(Weapon, "Menu equip reveal formID={:08X} by={} after={:.3f}s hiddenByHold={}",
                            reveal.formID, posed ? "firearm-pose" : "draw-complete", waited,
                            static_cast<bool>(reveal.culled));
                    } else {
                        ROCK_LOG_WARN(Weapon,
                            "Menu equip reveal formID={:08X} by=timeout after={:.3f}s drawStarted={}; shown without a "
                            "posed draw",
                            reveal.formID, waited, reveal.drawStarted);
                    }
                    reveal.revealed = true;
                    // The hold's show below takes over the node it hid.
                    reveal.culled.reset();
                } else {
                    awaitingPose = true;
                    if (equipped_weapon_visual_state::isLocallyVisible(node)) {
                        equipped_weapon_visual_state::setLocallyVisible(node, false);
                        reveal.culled.reset(node);
                    }
                }
            }
        } else if (reveal.formID) {
            endReveal();
        }
        const bool supportCarry = carried && carry.supportGrip.validCarry();
        // A two-handed carry, held when the menu opened or attached in it.
        const bool paired = carried && !supportCarry && carry.pairedGrips.valid();
        const auto& firingGrip = carry.firingGrip.valid() || !paired ? carry.firingGrip : carry.pairedGrips.primary;
        bool isLeft = carry.isLeft;
        RE::NiPoint3 gripLocal{};
        // Support carry: the weapon rides the hand. Firing: the hand rides the weapon.
        RE::NiTransform supportHandWeaponLocal{}, firingHandWeaponLocal{};
        frik_visual_authority::FingerLocalTransformOverride fingers{};
        const char* stage = !node ? "weapon-node" : nullptr;
        if (!stage && supportCarry) {
            supportHandWeaponLocal = carry.supportGrip.grip.handWeaponLocal;
            fingers.enabledMask = carry.supportGrip.grip.fingerMask;
            std::copy(carry.supportGrip.grip.fingerLocals.begin(), carry.supportGrip.grip.fingerLocals.end(),
                fingers.localTransforms);
        } else if (!stage && carried && firingGrip.valid() && (!melee || paired)) {
            gripLocal = firingGrip.gripWeaponLocal;
            firingHandWeaponLocal = firingGrip.handWeaponLocal;
            fingers.enabledMask = firingGrip.fingerMask;
            std::copy(firingGrip.fingerLocals.begin(), firingGrip.fingerLocals.end(), fingers.localTransforms);
        } else if (!stage) {
            // Equipped inside this menu: its own firing seat in the hand it
            // went to. A one-handed melee carry takes the same seat: the capture
            // holds the authored seat, not the reverse grip the player sees.
            if (!carried) {
                bool firing = true, supported = false;
                isLeft = native_hand_equipment::wornRowHand(view.weapon->formID, 0, firing, supported) ==
                    native_hand_equipment::WornHand::Left;
            }
            const bool powerArmor = f4vr::isInPowerArmor();
            // Starts the off-screen idle harvest a never-drawn weapon needs.
            native_idle_grip_preharvest::observeInventoryDisplay(view.weapon, node, view.instance);
            const auto native = weapon_grip_seat::resolveNative({
                .weapon = view.weapon,
                .weaponRoot = node,
                .variant = authored_weapon_grip_library::identifyWeaponVariant(node),
                .keywordFlags = weapon_instance_classification::computeWeaponKeywordFlags(view.weapon, view.instance),
                .inPowerArmor = powerArmor,
            });
            weapon_grip_seat::FiringSeat seat{};
            if (!weapon_grip_seat::firingSeat(native,
                    holding_grips::find(view.weapon, node, isLeft, view.data, powerArmor), isLeft,
                    _weaponCollision.holdingMirrorOffsets().plane(
                        weapon_type_policy::isEquippedMelee(view.weapon->weaponData.type.get())),
                    seat)) {
                stage = native.modelRegistered ? "authored-seat" : "model-frame";
            } else {
                firingHandWeaponLocal = melee ? knife_reverse_grip::presentedSeat(seat.handWeaponLocal, isLeft,
                                                    view.weapon->formID, _weaponCollision.holdingMirrorOffsets().z)
                                              : seat.handWeaponLocal;
                gripLocal = computeGrabPalmPointWorldFromHandBasis(firingHandWeaponLocal, isLeft);
                fingers.enabledMask = seat.fingers.enabledMask;
                std::copy(std::begin(seat.fingers.localTransforms), std::end(seat.fingers.localTransforms),
                    fingers.localTransforms);
            }
        }
        const auto hand = frik_visual_authority::handFromBool(isLeft);
        // The isolated controller hand, as the live carry reads it; never the
        // rendered hand, which is this hold's own claim.
        RE::NiTransform handWorld{}, weaponWorld{};
        if (!stage &&
            (!frik_hand_world_authority::tryGetRawHandWorld(isLeft, handWorld) ||
                !weapon_input_filter::usable(handWorld)))
            stage = "hand";
        if (!stage && !supportCarry &&
            !(melee ? TwoHandedGrip::tryGetMeleeWeaponAimWorld(isLeft, handWorld, firingHandWeaponLocal,
                          node->world.scale, weaponWorld)
                    : isLeft ? TwoHandedGrip::tryGetLeftWeaponAimWorld(node->world.scale, weaponWorld)
                             : TwoHandedGrip::tryGetRightWeaponAimWorld(node->world.scale, weaponWorld)))
            stage = "controller-aim";
        // Offhand-only carry: the weapon keeps the placement in the controller
        // hand captured when the menu opened. Not the live carry's input
        // driver: the right one is the native weapon offset node, which a menu
        // moves under the hand.
        RE::NiPoint3 modelTranslation{};
        if (!stage && supportCarry && !carry.supportGrip.weaponInRawHand)
            stage = "raw-hand-relation";
        if (!stage && supportCarry &&
            !vanilla_weapon_grip_frame::resolveModelTranslation(view.weapon->formID, node, modelTranslation))
            stage = "model-registration";
        // Diagnostic: how far the input driver would have put the weapon.
        float driverDeltaUnits = -1.0f, driverDeltaDegrees = -1.0f;
        RE::NiTransform driverWorld{};
        if (!stage && supportCarry && frik_hand_world_authority::tryGetInputDriverWorld(isLeft, driverWorld) &&
            weapon_input_filter::usable(driverWorld)) {
            const auto byDriver =
                transform_math::composeTransforms(transform_math::orthonormalizedTransform(driverWorld),
                    carry.supportGrip.registeredWeaponInDriver(modelTranslation));
            const auto byHand = transform_math::composeTransforms(transform_math::orthonormalizedTransform(handWorld),
                carry.supportGrip.registeredWeaponInRawHand(modelTranslation));
            driverDeltaUnits = dynamic_weapon_collision_policy::translationDeltaGameUnits(byDriver, byHand);
            driverDeltaDegrees = dynamic_weapon_collision_policy::rotationDeltaDegrees(byDriver, byHand);
        }
        ROCK_LOG_SAMPLE_INFO(Weapon, 1000,
            "MENU_WEAPON_HOLD weapon=first stage={} source={} hand={} melee={} paired={} settling={} visible={} "
            "driverVsHand={:.2f}gu/{:.2f}deg",
            stage ? stage : "placed",
            supportCarry  ? "support-carry"
                : carried ? "menu-carry"
                          : "native-seat",
            isLeft ? "left" : "right", melee, paired, frame.localMenuExitSettling,
            node ? equipped_weapon_visual_state::isLocallyVisible(node) : false, driverDeltaUnits, driverDeltaDegrees);
        if (stage) {
            release();
            return;
        }
        if (_equipped.menuHoldFingerHand >= 0 && _equipped.menuHoldFingerHand != (isLeft ? 1 : 0))
            release();
        if (frik_visual_authority::setAuthoredFingerPose(kTag, hand, fingers, kPriority))
            _equipped.menuHoldFingerHand = isLeft ? 1 : 0;
        // A grip captured on this model, moved into its current registration.
        const auto registered = [&](RE::NiTransform handWeaponLocal, const RE::NiPoint3& sourceModelTranslation) {
            handWeaponLocal.translate += modelTranslation - sourceModelTranslation;
            return handWeaponLocal;
        };
        if (supportCarry) {
            weaponWorld = transform_math::composeTransforms(transform_math::orthonormalizedTransform(handWorld),
                carry.supportGrip.registeredWeaponInRawHand(modelTranslation));
            if (_equipped.menuHoldHandClaim >= 0 && _equipped.menuHoldHandClaim != (isLeft ? 1 : 0))
                releaseHand();
            if (frik_visual_authority::publishHandWorld(kTag, hand,
                    transform_math::composeTransforms(weaponWorld,
                        registered(supportHandWeaponLocal, carry.supportGrip.sourceModelTranslation)),
                    kHandPriority))
                _equipped.menuHoldHandClaim = isLeft ? 1 : 0;
        } else {
            // As the live carry: aim from the controller, firing grip at the
            // palm, and the hand seated on the grip of that weapon.
            weaponWorld.translate += computeGrabPalmPointWorldFromHandBasis(handWorld, isLeft) -
                transform_math::localPointToWorld(weaponWorld, gripLocal);
            // Two-handed: the live paired solve pivots the weapon on the firing
            // palm toward the support palm (or the gunstock virtual aim).
            const auto& pair = carry.pairedGrips;
            if (paired && pair.support.authoredRole != loose_weapon_authored_grab_policy::Role::Firing &&
                !loose_weapon_authored_grab_policy::sharedFiringZone(pair.arrangement)) {
                WeaponTwoHandedSolverResult<RE::NiTransform> solved{};
                RE::NiTransform supportHandWorld{}, primaryWand{}, supportWand{};
                if (pair.gunstockAim) {
                    if (frik_hand_world_authority::tryGetWeaponWandWorld(isLeft, primaryWand) &&
                        frik_hand_world_authority::tryGetWeaponWandWorld(!isLeft, supportWand))
                        solved = equip_visual_bridge_policy::solvePairedGunstockBridge(weaponWorld, gripLocal,
                            primaryWand, supportWand);
                } else if (frik_hand_world_authority::tryGetRawHandWorld(!isLeft, supportHandWorld) &&
                    weapon_input_filter::usable(supportHandWorld)) {
                    solved = equip_visual_bridge_policy::solvePairedBridge(weaponWorld, gripLocal,
                        pair.support.gripWeaponLocal,
                        computeGrabPalmPointWorldFromHandBasis(supportHandWorld, !isLeft));
                }
                if (solved.solved)
                    weaponWorld = solved.weaponWorldTransform;
                else
                    ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "MENU_WEAPON_HOLD paired solve unavailable gunstock={}",
                        pair.gunstockAim);
            }
            if (_equipped.menuHoldHandClaim >= 0 && _equipped.menuHoldHandClaim != (isLeft ? 1 : 0))
                releaseHand();
            if (frik_visual_authority::publishHandWorld(kTag, hand,
                    transform_math::composeTransforms(weaponWorld, firingHandWeaponLocal), kHandPriority))
                _equipped.menuHoldHandClaim = isLeft ? 1 : 0;
        }
        // The other hand stays on its grip of that weapon: the support grip of
        // a two-handed carry, or the second part grip of an offhand carry.
        const bool secondPart = supportCarry && carry.secondSupportGrip.validCarry();
        const weapon_grip_transfer::HandGrip* other = paired ? &carry.pairedGrips.support
            : secondPart                                     ? &carry.secondSupportGrip.grip
                                                             : nullptr;
        const int supportSide = other ? (isLeft ? 0 : 1) : -1;
        if (_equipped.menuHoldSupportHand != supportSide)
            releaseSupport();
        if (other) {
            const auto supportHand = frik_visual_authority::handFromBool(supportSide == 1);
            const auto otherHandWeaponLocal = secondPart
                ? registered(other->handWeaponLocal, carry.secondSupportGrip.sourceModelTranslation)
                : other->handWeaponLocal;
            frik_visual_authority::FingerLocalTransformOverride supportFingers{};
            supportFingers.enabledMask = other->fingerMask;
            std::copy(other->fingerLocals.begin(), other->fingerLocals.end(), supportFingers.localTransforms);
            const bool posed =
                frik_visual_authority::setAuthoredFingerPose(kTag, supportHand, supportFingers, kPriority);
            const bool placed = frik_visual_authority::publishHandWorld(kTag, supportHand,
                transform_math::composeTransforms(weaponWorld, otherHandWeaponLocal), kHandPriority);
            if (posed || placed)
                _equipped.menuHoldSupportHand = supportSide;
        }
        // Opening a menu or equipping culls the first weapon's node natively
        // (the second is ROCK's own model); the native show after the menu
        // matches. A weapon still awaiting its posed draw stays hidden.
        if (!awaitingPose && !equipped_weapon_visual_state::isLocallyVisible(node))
            equipped_weapon_visual_state::setLocallyVisible(node, true);
        /*
         * The exit's draw starts on a clip of the outgoing subgraph, whose
         * first pose throws the parts of a weapon equipped in the menu off for
         * one frame (bolt 8 gu on the AK-74). The parts keep the pose last
         * shown until the draw ends. Captured each menu frame: this node stays
         * while an equip replaces the model below it. A weapon still awaiting
         * its pose has shown none.
         */
        const auto partsRoot = reinterpret_cast<std::uintptr_t>(node);
        if (frame.localMenuBlocking) {
            if (firstHoldThisFrame) {
                _equipped.menuHoldParts.clear();
                if (!awaitingPose)
                    captureMenuHoldParts(node, _equipped.menuHoldParts);
                _equipped.menuHoldPartsRoot = partsRoot;
            }
        } else if (frame.localMenuExitSettling && _equipped.menuHoldPartsRoot == partsRoot) {
            for (const auto& part : _equipped.menuHoldParts)
                part.node->local = part.local;
            f4vr::updateTransformsDown(node, false);
        }
        (void)weapon_scene_presentation::moveWeaponPresentationRigidly(node, weaponWorld);
    }

    void PhysicsInteraction::captureEquippedWeaponContinuity()
    {
        if (_equipped.gripResumePending || runtime_state::currentFrame().localMenuBlocking)
            return;
        auto snapshot = _equipped.transition.pendingGrip();
        if (snapshot.pending) {
            _equipped.continuityGrip = snapshot;
            _equipped.nativeCarryCapture = {};
            return;
        }
        if (_twoHandedGrip.isHandsFreeRetained()) {
            _equipped.continuityGrip = {};
            _equipped.nativeCarryCapture = {};
            _equipped.resumeAwaitingHold = {};
            return;
        }
        const bool manualCarry = _twoHandedGrip.isManualOwnershipActive();
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
        snapshot.pending = _twoHandedGrip.captureMenuCarry(firing, snapshot.pairedGrips, snapshot.supportGrip,
            snapshot.secondSupportGrip, carrierLeft, &captureFailure, &snapshot.supportAttach);
        snapshot.isLeft = carrierLeft;
        snapshot.firingHandWeaponLocal = firing.handWeaponLocal;
        snapshot.firingGripWeaponLocal = firing.gripWeaponLocal;
        snapshot.hasFiringHandWeaponLocal = firing.valid();
        snapshot.hasFiringGripWeaponLocal = firing.valid();
        snapshot.firingGrip = firing;
        snapshot.toggleAcquisitionCommitted = !snapshot.supportGrip.validCarry();
        snapshot.source = equipped_weapon_manual_ownership_policy::PrimaryOnlyStartSource::HeldWeaponEquip;
        // Keep no scene pointers or world transforms. This is sampled after
        // grip decisions, while the outgoing nodes and controller frame live.
        if (!manualCarry) {
            _equipped.continuityGrip = {};
            _equipped.resumeAwaitingHold = {};
            _equipped.nativeCarryCapture =
                snapshot.pending ? snapshot : EquippedWeaponTransitionCoordinator::PendingGrip{};
            return;
        }
        _equipped.nativeCarryCapture = {};
        _equipped.continuityGrip = snapshot;
        if (!snapshot.pending) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "Equipped menu-carry continuity capture unavailable form={:08X} hand={} reason={} instance={:#x}",
                snapshot.targetWeaponFormID, carrierLeft ? "left" : "right",
                captureFailure ? captureFailure : "unknown", snapshot.targetWeaponInstanceData);
        }
    }

    void PhysicsInteraction::updateEquippedWeaponTransition()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeGunState = f4vr::getNativeGunState(player);
        const std::uint32_t nativeWeaponState = f4vr::getNativeWeaponState(player);
        native_equipped_weapon_draw::Identity currentIdentity{};
        const bool currentIdentityCaptured = native_equipped_weapon_draw::captureCurrentIdentity(currentIdentity);
        /*
         * The lease ends when the engine pauses the game, not on every input
         * menu or FRIK compatibility block: the wrist Pip-Boy blocks input
         * and holds FRIK's compatibility block (ROCK leaves the hands to
         * FRIK), but the game keeps running and its equips play their draw
         * clip while it stays open. The lease only speeds that clip in the
         * animation graph and never touches the hands; ending it there let
         * the clip run at native speed, so the parts assembled on screen.
         * The projected Pip-Boy pauses the game; its clip plays after the
         * menu closes, under the menu-exit lease.
         */
        const char* accelerationBlockedBy = !runtime.visualAuthorityAvailable ? "visual-authority-unavailable"
            : !runtime.localSkeletonReady                                     ? "skeleton-not-ready"
            : runtime.localGamePaused                                         ? "game-paused"
                                                                              : nullptr;
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
                .runtimeAllowed = accelerationBlockedBy == nullptr,
                .runtimeBlockedBy = accelerationBlockedBy,
            });
        if (nativeWeaponState != _equipped.tracedNativeWeaponState) {
            // Logged only on a change, with how long the previous state
            // lasted and whether an inventory equip is waiting on it.
            const auto now = std::chrono::steady_clock::now();
            const bool first = _equipped.tracedNativeWeaponState == 0xFFFFFFFFu;
            ROCK_LOG_INFO(Weapon,
                "Native weapon state {}->{} previousLastedMs={:.0f} form={:08X} inventoryEquipPending={}",
                first ? "none"
                      : held_weapon_equip_state_policy::nativeWeaponStateName(_equipped.tracedNativeWeaponState),
                held_weapon_equip_state_policy::nativeWeaponStateName(nativeWeaponState),
                first ? 0.0
                      : std::chrono::duration<double, std::milli>(now - _equipped.tracedNativeWeaponStateAt).count(),
                currentIdentityCaptured ? currentIdentity.formID : 0u, _inventoryEquip.active());
            _equipped.tracedNativeWeaponState = nativeWeaponState;
            _equipped.tracedNativeWeaponStateAt = now;
        }
        const bool nativeWeaponAnimationActive = provider::currentNativeAnimationAuthorityFlags() != 0 ||
            nativeGunState == static_cast<std::uint32_t>(RE::GUN_STATE::kReloading);
        RE::NiTransform leftCarrySolvedWeaponWorld{};
        if (runtime.localMenuBlocking && !_equipped.menuReconcilePending) {
            restoreEquippedWeaponContinuity(_equipped.continuityGrip);
            cancelEquippedWeaponTransfersForMenu();
        }
        applyPipboySupportRequest();
        const auto& transfer = _equipped.transition.heldTransfer();
        const auto& incomingHand = transfer.request.isLeft ? _leftHand : _rightHand;
        auto* incomingReference = incomingHand.getHeldRef();
        _equipped.transition.validateHeldSource(incomingHand.isHolding(),
            incomingReference ? incomingReference->GetFormID() : 0u, incomingHand.heldGrabIdentity(),
            _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
            _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire));
        _equipped.transition.update(EquippedWeaponTransitionCoordinator::FrameInput{
            .deltaSeconds = runtime.deltaSeconds,
            .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
            .localSkeletonReady = runtime.localSkeletonReady,
            .menuBlocking = runtime.localMenuBlocking,
            .compatibilityBlocking = runtime.compatibilityConfigBlocking,
            .gripSuspended = _equipped.gripResumePending,
            .nativeWeaponState = nativeWeaponState,
            .nativeWeaponAnimationActive = nativeWeaponAnimationActive,
            // Read one frame behind the grip solve by design: this update
            // runs before TwoHandedGrip::update each frame, so the bridge
            // receives the previous frame's solved carry pose - the pose
            // that was actually rendered.
            .leftCarrySolvedWeaponWorldValid =
                _twoHandedGrip.tryGetSolvedLeftFiringWeaponWorld(leftCarrySolvedWeaponWorld),
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

        // Grab buttons after resume rearm and provider and shoulder
        // masking. The firing hand's state is read lazily, once.
        GrabButtonState leftPhysicalGripState{};
        GrabButtonState rightPhysicalGripState{};
        bool leftGripHeld = false;
        bool rightGripHeld = false;
        EquippedWeaponGripOccupancy providerGripOccupancy{};
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
                fo4vr_actor_state_policy::equippedWeaponOccupiesHand(true,
                    f4vr::getNativeWeaponState(f4vr::getPlayer())),
            .hand = equipped ? equipped_weapon_roster::handFor(native_hand_equipment::secondaryHandIsLeft())
                             : equipped_weapon_roster::Hand::None,
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
            firingLeft = equipped_weapon_transition_policy::resolveFiringHandIsLeft(sessionFiringLeft,
                weapon ? weapon->GetFormID() : 0u,
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
        const bool pendingBothHands = pending.pairedGrips.valid() || pending.secondSupportGrip.validCarry();
        const bool pendingSupport = pending.supportGrip.validCarry();
        facts.pending = {
            .bothHands = pendingBothHands,
            .supportHand = pendingSupport ? handFor(pending.isLeft) : equipped_weapon_roster::Hand::None,
            .firingHand = pending.pending && !pendingBothHands && !pendingSupport ? handFor(pending.isLeft)
                                                                                  : equipped_weapon_roster::Hand::None,
        };
        return equipped_weapon_roster::Roster(facts);
    }

    PhysicsInteraction::EquippedWeaponFramePreparation PhysicsInteraction::beginEquippedWeaponFrame(
        const PhysicsFrameContext& frame, RE::hknpWorld* hknp)
    {
        EquippedWeaponFramePreparation prepared{};
        sampleEquippedSlotOne();
        /*
         * Index 1 never stands alone. When index 0 empties by any path (a
         * Pip-Boy unequip or cross-hand move, a script), the second weapon
         * moves to index 0 in its own hand, as on a first-weapon drop.
         * Alone in index 1 the player is natively unarmed: the weapon state
         * sheathes and the second weapon stays hidden (runtime 14 Oct
         * 21:39:03). A held transfer or an equip awaiting its identity owns
         * index 0 and is left to finish.
         */
        if (_equippedSlotOne.equipped && !frame.menuBlocked && !_equipped.transition.heldTransfer().active() &&
            !_equipped.transition.getPublicSnapshot().identityPending) {
            const auto views = native_hand_equipment::equippedViews();
            const auto instance = reinterpret_cast<std::uintptr_t>(views[1].instance);
            const bool failedBefore = views[1].weapon && _failedPromotion.formID == views[1].weapon->formID &&
                _failedPromotion.instance == instance;
            if (views[0].weapon || !failedBefore)
                _failedPromotion = {};
            if (!views[0].weapon && views[1].weapon && !failedBefore) {
                ROCK_LOG_INFO(Weapon, "AKIMBO index 0 empty; promoting the second weapon form={:08X}",
                    views[1].weapon->formID);
                if (!promoteSecondaryWeapon(views[1].weapon, instance, native_hand_equipment::secondaryHandIsLeft())) {
                    ROCK_LOG_WARN(Weapon,
                        "AKIMBO promotion failed form={:08X} instance={:#x}; retried when either slot changes",
                        views[1].weapon->formID, instance);
                }
                sampleEquippedSlotOne();
            }
        }
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
        const bool leftHandTaken =
            prepared.secondaryLeft && (_twoHandedGrip.isFiringHandLeft() || _twoHandedGrip.isHandPartGripping(true));
        const bool rightHandTaken = prepared.secondaryRight &&
            (_twoHandedGrip.isHandPartGripping(false) ||
                (_twoHandedGrip.isManualOwnershipActive() && !_twoHandedGrip.isFiringHandLeft() &&
                    !_twoHandedGrip.isFiringGripDetached()));
        if (leftHandTaken || rightHandTaken) {
            ROCK_LOG_INFO(Weapon, "AKIMBO primary released from the second weapon's hand={}",
                prepared.secondaryLeft ? "left" : "right");
            _twoHandedGrip.reset();
            clearEquippedWeaponFiringGripInputState();
        }

        prepared.leftHandAvailableForAcquisition = !prepared.secondaryLeft &&
            force_grab_policy::availableForEquippedGrip(
                forceGrabHandBlockerMask(_leftHand, true, frame.left.disabled, true));
        prepared.rightHandAvailableForAcquisition = !prepared.secondaryRight &&
            force_grab_policy::availableForEquippedGrip(
                forceGrabHandBlockerMask(_rightHand, false, frame.right.disabled, true));
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
        bool rightHandWeaponAuthorityActive =
            prepared.rightHandWeaponEquipped || prepared.retainedWeaponCollisionActive;
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
        if (prepared.rightHandWeaponEquipped && _twoHandedGrip.isFiringHandLeft() &&
            _twoHandedGrip.isFiringGripOccupied()) {
            rightHandWeaponAuthorityActive = false;
        }
        prepared.rightHandWeaponAuthorityActive = rightHandWeaponAuthorityActive;

        const auto occupancy = equippedWeaponRoster().slotZeroGrips();
        prepared.leftSupportGripActive = occupancy.left.partGripActive;
        prepared.rightPartGripActive = occupancy.right.partGripActive;
        setRightFiringWeaponOwnership(hknp, prepared.rightHandWeaponAuthorityActive, prepared.rightHandWeaponEquipped);
        // A part-gripping free hand is a transform driver like the support hand
        // and must not also solve contacts against the weapon package.
        setWeaponHandSuppression(hknp, false, WeaponHandReason::Grip, prepared.rightPartGripActive);
        return prepared;
    }

    PhysicsInteraction::EquippedWeaponFrameResult PhysicsInteraction::updateEquippedWeaponFrame(
        const PhysicsFrameContext& frame, RE::bhkWorld* bhk, RE::hknpWorld* hknp,
        const EquippedWeaponFramePreparation& prepared)
    {
        performance_profiler::ScopedTimer equippedFrameTimer(performance_profiler::Scope::EquippedWeaponInteraction);
        EquippedWeaponFrameWork w{
            .frame = frame,
            .bhk = bhk,
            .hknp = hknp,
            .prepared = prepared,
        };

        using performance_profiler::Scope;
        using StageTimer = performance_profiler::ScopedTimer;
        {
            StageTimer stage(Scope::EquippedObserve);
            observeEquippedWeapon(w);
        }
        {
            StageTimer stage(Scope::EquippedRouting);
            routeEquippedWeaponPrimaryHand(w);
            resolveEquippedWeaponFiringHand(w);
        }
        {
            StageTimer stage(Scope::EquippedContacts);
            probeEquippedWeaponContacts(w);
            readEquippedWeaponGripButtons(w);
            resolveEquippedWeaponPartAuthority(w);
        }
        {
            StageTimer stage(Scope::EquippedGripStart);
            startEquippedWeaponGrips(w);
        }
        bool placingWeapon = false;
        {
            StageTimer stage(Scope::EquippedGripUpdate);
            updateEquippedWeaponGrips(w);
            placingWeapon = beginEquippedWeaponPlacement(frame, prepared.weaponNode);
            queueEquippedWeaponGripHaptics();
        }
        {
            StageTimer stage(Scope::EquippedStashDrop);
            commitEquippedWeaponStashAndDrop(w);
        }
        StageTimer publishStage(Scope::EquippedPublish);
        const EquippedWeaponFrameResult result = publishEquippedWeaponGripOwnership(w);
        finishDynamicWeaponCollision(w);

        // Final source transforms are queued after all animation/FRIK writers.
        const bool retired = placingWeapon ? retireEquippedWeaponInPlace(frame, prepared.weaponNode, "held-placement")
                                           : retireHandsFreeWeapon(frame, prepared.weaponNode);
        if (placingWeapon) {
            // The native side reports the end of a retirement that did start
            // (serviceEquippedWeaponNativeHandoff).
            _placementEquippedRetiring = retired;
            _placementLastResult =
                retired ? api::grab::PlacementResult::Pending : api::grab::PlacementResult::Rejected;
        }
        if (retired) {
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
                        weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(),
                        _weaponCollision.getWeaponBodyCount());
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
        // The colliders arriving change the key, not the weapon: what consumers hold on
        // it follows, this frame, before the grips below read it. A grip already on a
        // grab point stays through it (tryRebindPartGripToCurrentGeneration).
        if (_equipped.weaponInteractionKey != w.currentAuthoredGripGenerationKey) {
            if (_equipped.weaponInteractionKey != 0 && w.currentWeaponGenerationKey != 0 &&
                _equipped.weaponInteractionKey == w.currentEquippedWeaponOwnershipKey) {
                ::rock::provider::adoptWeaponPartGeneration(_equipped.weaponInteractionKey,
                    w.currentAuthoredGripGenerationKey);
                support_trigger_input::adoptWeaponKey(_equipped.weaponInteractionKey,
                    w.currentAuthoredGripGenerationKey);
                physical_reload_input::adoptWeaponKey(_equipped.weaponInteractionKey,
                    w.currentAuthoredGripGenerationKey);
                if (_providerDrives.generationKey == _equipped.weaponInteractionKey) {
                    _providerDrives.generationKey = w.currentAuthoredGripGenerationKey;
                }
                refreshProviderWeaponSources();
            }
            _equipped.weaponInteractionKey = w.currentAuthoredGripGenerationKey;
        }
        _dynamicWeaponCollision.beginFrame(runtime.frameIndex, w.hknp, w.bhk, weaponNode, w.currentWeaponGenerationKey,
            dynamic_weapon_collision_policy::kDynamicCompoundEnabled && runtime.weaponDrawn && !frame.menuBlocked &&
                physicsWritesAllowedForWorld(frame.hknpWorld));
        reconcileEquippedWeaponHandlingMode();
        // WeaponCollision has just read the actual drawn inventory instance;
        // its observed identity exists before geometry/grab-session readiness.
        const auto transferPhase = _equipped.transition.heldTransfer().phase;
        _twoHandedGrip.observeEquippedOwnership(runtime.weaponDrawn &&
                    _weaponCollision.getCurrentObservedEquippedWeaponFormID() != 0
                ? w.currentEquippedWeaponOwnershipKey
                : 0,
            w.currentAuthoredGripGenerationKey,
            transferPhase == held_weapon_transfer::Phase::AwaitGrip ||
                transferPhase == held_weapon_transfer::Phase::Recovering);

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
                equipped_weapon_transition_policy::matchesExpectedIdentity(observedEquippedWeaponFormID,
                    observedEquippedWeaponInstanceData, equipGrip.targetWeaponFormID,
                    equipGrip.targetWeaponInstanceData, equipGrip.previousWeaponFormID,
                    equipGrip.previousWeaponInstanceData));
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
        const bool primaryDefaultLeft = prepared.secondaryEquipped ? prepared.secondaryRight
            : directedPrimaryMatches                               ? directedPrimary.left
                                                                   : g_rockConfig.rockDefaultHandLeft;
        if (equipGrip.pending && equipGrip.source == StartSource::DefaultEquip &&
            (!primaryDefaultLeft || !pendingGripMatches || !runtime.weaponDrawn)) {
            equipGrip = {};
        }
        const bool handSpecificOwnership = _twoHandedGrip.isManualOwnershipActive() ||
            _equipped.transition.heldTransfer().active() || _equipped.gripResumePending ||
            (pendingGripMatches &&
                (equipGrip.menuResume || equipGrip.source == StartSource::HeldWeaponEquip ||
                    equipGrip.source == StartSource::InventoryEquip));
        bool queueDefaultLeft =
            equipped_weapon_transition_policy::observeDefaultLeftEquip(_equipped.defaultEquipObservedOwnershipKey,
                runtime.weaponDrawn && observedEquippedWeaponFormID != 0 ? w.currentEquippedWeaponOwnershipKey : 0,
                primaryDefaultLeft, handSpecificOwnership, w.inputBlockingMenuActive);
        // The primary takes the hand chosen in the Pip-Boy (equip or
        // cross-hand move), or the left when the second weapon holds the
        // right hand. A support-held primary keeps its support hand.
        // The hand that last fired this exact item. A re-draw of the same item
        // (a promotion's draw recovery) releases its grip; it keeps that hand.
        auto& memory = _equipped.firingHandMemory;
        const auto ownershipKey = w.currentEquippedWeaponOwnershipKey;
        if (_twoHandedGrip.isManualOwnershipActive() && ownershipKey != 0 && !_twoHandedGrip.isFiringGripDetached())
            memory = { .ownershipKey = ownershipKey, .left = _twoHandedGrip.isFiringHandLeft() };
        if (!queueDefaultLeft && !w.inputBlockingMenuActive && runtime.weaponDrawn &&
            observedEquippedWeaponFormID != 0 && !equipGrip.pending && !_twoHandedGrip.isFiringGripDetached()) {
            const bool firingLeft = _twoHandedGrip.isFiringHandLeft();
            const bool moved = directedPrimaryPending && directedPrimaryMatches;
            if (moved) {
                memory = {};
                _equipped.directedPrimarySequence = directedPrimary.sequence;
                native_hand_equipment::noteDirectedPrimaryApplied(directedPrimary.sequence);
                ROCK_LOG_INFO(Weapon, "AKIMBO primary hand request formID={:08X} hand={} firing={}",
                    observedEquippedWeaponFormID, primaryDefaultLeft ? "left" : "right", firingLeft ? "left" : "right");
            }
            const bool restored = !moved && !handSpecificOwnership && ownershipKey != 0 &&
                memory.ownershipKey == ownershipKey && memory.left != firingLeft;
            if (restored) {
                ROCK_LOG_INFO(Weapon, "AKIMBO primary hand restored after its grip restarted formID={:08X} hand={}",
                    observedEquippedWeaponFormID, memory.left ? "left" : "right");
            }
            const bool targetLeft = restored ? memory.left : primaryDefaultLeft;
            if (targetLeft && !firingLeft && (prepared.secondaryRight || moved || restored)) {
                if (_twoHandedGrip.isManualOwnershipActive()) {
                    _twoHandedGrip.reset();
                    clearEquippedWeaponFiringGripInputState();
                }
                queueDefaultLeft = true;
            } else if (!targetLeft && firingLeft && (moved || restored)) {
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
        if (pendingGrip.pending && !_equipped.transition.heldTransfer().active() &&
            pendingGrip.source != StartSource::DefaultEquip) {
            pendingGrip.remainingSeconds -= (std::max)(0.0f, w.frame.deltaSeconds);
            if (pendingGrip.remainingSeconds <= 0.0f) {
                ROCK_LOG_WARN(Weapon,
                    "Held weapon manual ownership handoff expired targetForm={:08X} targetInstance={:#x} hand={} "
                    "lastStartFailure={}",
                    pendingGrip.targetWeaponFormID, pendingGrip.targetWeaponInstanceData,
                    pendingGrip.isLeft ? "left" : "right",
                    pendingGrip.lastStartFailureReason ? pendingGrip.lastStartFailureReason : "none");
                pendingGrip = {};
            }
        }
    }

    // The shoulder stash gesture, then the firing and support hands
    // for this frame: a pending equip's hand when it matches this weapon,
    // else the grip's current firing hand.
    void PhysicsInteraction::resolveEquippedWeaponFiringHand(EquippedWeaponFrameWork& w)
    {
        const auto& pendingGrip = _equipped.transition.pendingGrip();
        const auto pendingFiringHandIsLeft = [&]() {
            const bool matches = pendingGrip.pending &&
                (pendingGrip.targetWeaponFormID == 0 ||
                    equipped_weapon_transition_policy::matchesExpectedIdentity(w.observedEquippedWeaponFormID,
                        w.observedEquippedWeaponInstanceData, pendingGrip.targetWeaponFormID,
                        pendingGrip.targetWeaponInstanceData, pendingGrip.previousWeaponFormID,
                        pendingGrip.previousWeaponInstanceData));
            const bool firingLeft = matches
                ? (pendingGrip.supportGrip.validCarry() ? !pendingGrip.isLeft : pendingGrip.isLeft)
                : _twoHandedGrip.isFiringHandLeft();
            return std::pair{ matches, firingLeft };
        };

        const bool equippedWeaponShoulderStashActive =
            equipped_weapon_drop_policy::equippedWeaponShoulderStashAvailable(
                _equipped.handlingSettings.equippedWeaponShoulderStashEnabled);
        const bool shoulderFiringHandIsLeft = pendingFiringHandIsLeft().second;
        w.shoulderFrame = advanceEquippedWeaponShoulderCoordinator(w.frame, equippedWeaponShoulderStashActive,
            w.inputBlockingMenuActive, w.prepared.weaponNode, w.currentEquippedWeaponOwnershipKey,
            shoulderFiringHandIsLeft);

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

        const bool weaponContactQueriesRequired = weaponNode && w.currentAuthoredGripGenerationKey != 0 &&
            _twoHandedGrip.requiresWeaponContactQueries(weaponNode, w.currentAuthoredGripGenerationKey,
                _equipped.handlingSettings.authoredOnlySupportGrabsEnabled,
                provider::hasWeaponPartTargetsForGeneration(w.currentAuthoredGripGenerationKey));
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
        if (provider::runtime::hasGrabPoints(w.currentAuthoredGripGenerationKey)) {
            operatingPose.emplace(captureProviderPartQueryPose(weaponNode, w.currentAuthoredGripGenerationKey));
        }
        const auto* operatingPosePointer = operatingPose ? &*operatingPose : nullptr;
        consumeWeaponContactForHand(weaponNode, true, w.frame.left, true, w.leftWeaponContact, interactionQueries,
            operatingPosePointer);
        const bool rightWeaponContactProbeAllowed = _twoHandedGrip.isFiringGripDetached() || w.firingHandIsLeft;
        consumeWeaponContactForHand(weaponNode, false, w.frame.right, rightWeaponContactProbeAllowed,
            w.rightWeaponContact, interactionQueries, operatingPosePointer);
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

    void PhysicsInteraction::consumeWeaponContactForHand(RE::NiNode* weaponNode, bool isLeft,
        const HandFrameInput& handInput, bool probeAllowed, WeaponInteractionContact& outContact,
        WeaponCollision::InteractionQueryBatch& interactionQueries, const weapon_drive_pose::NativeFrame* operatingPose)
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
                extraProbes[extraCount++] = { handInput.grabAnchorWorld.x, handInput.grabAnchorWorld.y,
                    handInput.grabAnchorWorld.z };
            }
        }

        const bool operatingObserved = operatingPose && !handInput.disabled &&
            _weaponCollision.tryFindInteractionContactNearPoint(weaponNode, palmPivotWorld,
                probeAllowed ? g_rockConfig.rockWeaponInteractionProbeRadius
                             : g_rockConfig.rockWeaponInteractionTouchRadius,
                outContact, interactionQueries, apiHand, operatingPose, std::span(extraProbes).first(extraCount), true,
                std::span(pinkyProbes).first(pinkyCount));
        const bool touchObserved = !operatingObserved && weaponNode &&
            _weaponCollision.tryFindInteractionContactNearPoint(weaponNode, palmPivotWorld,
                g_rockConfig.rockWeaponInteractionTouchRadius, outContact, interactionQueries, apiHand, operatingPose);
        if (operatingObserved || touchObserved) {
            publishWeaponInteractionContact(isLeft, outContact);
        } else if (weaponNode && probeAllowed) {
            if (_weaponCollision.tryFindInteractionContactNearPoint(weaponNode, handInput.grabAnchorWorld,
                    g_rockConfig.rockWeaponInteractionProbeRadius, outContact, interactionQueries, apiHand,
                    operatingPose)) {
                publishWeaponInteractionContact(isLeft, outContact);
                if (g_rockConfig.rockDebugVerboseLogging && ++_diagnostics.weaponInteractionProbeLogCounter >= 90) {
                    _diagnostics.weaponInteractionProbeLogCounter = 0;
                    ROCK_LOG_DEBUG(Weapon,
                        "WeaponInteractionProbe: hand={} bodyId={} partKind={} supportRole={} reloadRole={} "
                        "actionRole={} radius={:.1f}",
                        isLeft ? "left" : "right", outContact.bodyId, static_cast<int>(outContact.partKind),
                        static_cast<int>(outContact.supportGripRole), static_cast<int>(outContact.reloadRole),
                        static_cast<int>(outContact.actionRole), g_rockConfig.rockWeaponInteractionProbeRadius);
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

    void PhysicsInteraction::maskShoulderGestureGripInput(const EquippedWeaponFrameWork& w, bool isLeft,
        GrabButtonState& button) const
    {
        const auto& decision = w.shoulderFrame.decision;
        const bool consumed = isLeft ? decision.consumeLeftInput : decision.consumeRightInput;
        if (!consumed) {
            return;
        }

        if (decision.gestureAction == equipped_weapon_shoulder::Action::SubmitStash) {
            // The coordinator consumed the physical tap/release. Keep the
            // carry logically closed so the stash never becomes a drop.
            button = GrabButtonState{ .held = true };
        }
    }

    // Both hands' grab buttons, with menu-resume rearm, provider suppression
    // and the shoulder gesture applied.
    void PhysicsInteraction::readEquippedWeaponGripButtons(EquippedWeaponFrameWork& w)
    {
        auto& leftPhysicalGripState = w.leftPhysicalGripState;
        auto& rightPhysicalGripState = w.rightPhysicalGripState;
        leftPhysicalGripState = peekGrabButtonState(true, input_remap_policy::kGrabButtonId);
        rightPhysicalGripState = peekGrabButtonState(false, input_remap_policy::kGrabButtonId);
        if (_equipped.transition.pendingGrip().menuResume) {
            leftPhysicalGripState.held =
                input_remap_runtime::isRawButtonPhysicallyHeld(true, input_remap_policy::kGrabButtonId);
            rightPhysicalGripState.held =
                input_remap_runtime::isRawButtonPhysicallyHeld(false, input_remap_policy::kGrabButtonId);
        }
        rearmResumedGripHold(true, leftPhysicalGripState);
        rearmResumedGripHold(false, rightPhysicalGripState);
        w.providerGripOccupancy = equippedWeaponRoster().slotZeroGrips();
        maskProviderWeaponGrabInput(true, w.providerGripOccupancy.left.weaponEngaged(), leftPhysicalGripState);
        maskProviderWeaponGrabInput(false, w.providerGripOccupancy.right.weaponEngaged(), rightPhysicalGripState);

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
        const bool leftWeaponPartMatched =
            resolvePart(w.leftWeaponContact, w.leftHandInteractionState, leftWeaponPartResolution);
        const bool rightWeaponPartMatched =
            resolvePart(w.rightWeaponContact, w.rightHandInteractionState, rightWeaponPartResolution);

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

        const auto indicatorCandidate = [&](const WeaponInteractionContact& contact,
                                            const WeaponInteractionRuntimeState& state) {
            return contact.valid && state.supportGripAllowed &&
                contact.weaponGenerationKey == w.currentAuthoredGripGenerationKey;
        };
        const bool leftIndicator = w.prepared.leftHandAvailableForAcquisition &&
            indicatorCandidate(w.leftWeaponContact, w.leftHandInteractionState);
        const bool rightIndicator = w.prepared.rightHandAvailableForAcquisition &&
            indicatorCandidate(w.rightWeaponContact, w.rightHandInteractionState);
        _equipped.partIndicatorBodyIds = {
            leftIndicator ? w.leftWeaponContact.bodyId : weapon_part_runtime::kInvalidBodyId,
            rightIndicator ? w.rightWeaponContact.bodyId : weapon_part_runtime::kInvalidBodyId,
        };
        _equipped.partIndicatorPoints = { w.leftWeaponContact.grabPoint, w.rightWeaponContact.grabPoint };
        _equipped.partIndicatorPointCandidates = { leftIndicator, rightIndicator };
        _equipped.partIndicatorFrame = runtime_state::currentFrame().frameIndex;
        _equipped.partIndicatorGeneration = w.currentAuthoredGripGenerationKey;

        w.leftHandHoldingObject = _leftHand.isHolding();
        // The grab-mode override follows the SUPPORT-ROLE hand's provider
        // resolution: that is the hand whose grip the mode describes.
        bool supportAuthorityProviderOverride = false;
        const bool supportWeaponPartMatched = w.supportHandIsLeft ? leftWeaponPartMatched : rightWeaponPartMatched;
        const auto& supportWeaponPartResolution =
            w.supportHandIsLeft ? leftWeaponPartResolution : rightWeaponPartResolution;
        if (supportWeaponPartMatched) {
            if (supportWeaponPartResolution.grabMode == weapon_part_runtime::GrabMode::FullTwoHandAuthority) {
                w.supportAuthorityMode =
                    weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
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
        primaryGrabState.held =
            input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft, input_remap_policy::kGrabButtonId);
        const auto physicalPrimaryGrabState = primaryGrabState;
        rearmResumedGripHold(firingHandIsLeft, primaryGrabState);
        maskProviderWeaponGrabInput(firingHandIsLeft,
            (firingHandIsLeft ? w.providerGripOccupancy.left : w.providerGripOccupancy.right).weaponEngaged(),
            primaryGrabState);
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
        w.ambidextrousHandoffAvailable = !w.prepared.secondaryEquipped &&
            _equipped.handlingSettings.ambidextrousHandoffEnabled &&
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        w.firingGripOwnershipFeatureAvailable =
            equipped_weapon_manual_ownership_policy::featureAvailable(firingGripDecision.firingGripOwnershipEnabled,
                w.primaryPoseBlockerAvailable, weaponNode != nullptr, currentEquippedWeaponOwnershipKey);
        w.primaryDetachFeatureAvailable =
            equipped_weapon_manual_ownership_policy::featureAvailable(firingGripDecision.primaryDetachEnabled,
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
                    .gripHeld = input_remap_runtime::isRawButtonPhysicallyHeld(firingHandIsLeft,
                                    input_remap_policy::kGrabButtonId) ||
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
        const bool firingHandAvailableForAcquisition =
            firingHandIsLeft ? leftHandAvailableForAcquisition : rightHandAvailableForAcquisition;
        const bool pairedHandsAvailable = !pendingPrimaryStart.pairedGrips.valid() ||
            (leftHandAvailableForAcquisition && rightHandAvailableForAcquisition);
        if (pendingPrimaryStart.pending && pendingPrimaryStart.source == StartSource::DefaultEquip &&
            (!firingHandAvailableForAcquisition || !w.firingGripOwnershipFeatureAvailable)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 2000,
                "Default left equip waiting formID={:08X} handAvailable={} ownershipAvailable={} poseBlocker={} "
                "weaponNode={} ownership={:016X}",
                observedEquippedWeaponFormID, firingHandAvailableForAcquisition, w.firingGripOwnershipFeatureAvailable,
                w.primaryPoseBlockerAvailable, weaponNode != nullptr, currentEquippedWeaponOwnershipKey);
        }

        // Transferred support grip (one or both hands on support stations).
        if (w.pendingPrimaryStartMatchesCurrentWeapon && pendingPrimaryStart.supportGrip.validCarry() &&
            w.primaryDetachFeatureAvailable && !inputBlockingMenuActive && !_twoHandedGrip.isManualOwnershipActive() &&
            (transferredSupportIsLeft ? leftHandAvailableForAcquisition : rightHandAvailableForAcquisition) &&
            (!pendingPrimaryStart.secondSupportGrip.validCarry() ||
                (leftHandAvailableForAcquisition && rightHandAvailableForAcquisition)) &&
            (transferredSupportIsLeft ? w.leftHandInteractionState : w.rightHandInteractionState).supportGripAllowed &&
            (!w.transferredBothSupports ||
                (w.leftHandInteractionState.supportGripAllowed && w.rightHandInteractionState.supportGripAllowed))) {
            const char* failure = nullptr;
            if (_twoHandedGrip.beginTransferredSupportGrip(weaponNode, currentAuthoredGripGenerationKey,
                    currentEquippedWeaponOwnershipKey, pendingPrimaryStart.supportGrip, &failure,
                    pendingPrimaryStart.secondSupportGrip.validCarry() ? &pendingPrimaryStart.secondSupportGrip
                                                                       : nullptr)) {
                _equipped.transition.recordGripAcquired(observedEquippedWeaponFormID,
                    w.observedEquippedWeaponInstanceData, transferredSupportIsLeft,
                    held_weapon_transfer::Role::Support);
                w.supportGripStartedThisFrame = true;
                w.toggleOccupancyBefore = _twoHandedGrip.getGrabInputOccupancy();
                pendingPrimaryStart = {};
            } else {
                pendingPrimaryStart.lastStartFailureReason = failure;
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                    "Support equipped grip transfer waiting form={:08X} hand={} reason={}",
                    observedEquippedWeaponFormID, transferredSupportIsLeft ? "left" : "right",
                    failure ? failure : "unknown");
            }
        }

        // Firing grip alone, or a transferred two-hand grip.
        if (pendingPrimaryStart.supportGrip.validCarry() || !w.firingGripOwnershipFeatureAvailable ||
            inputBlockingMenuActive || !firingHandAvailableForAcquisition || !pairedHandsAvailable ||
            _twoHandedGrip.isManualOwnershipActive()) {
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
        const bool pendingPrimaryOnlyStartRequested =
            equipped_weapon_manual_ownership_policy::shouldStartPendingPrimaryOnlyGrip(pendingMatches,
                primaryState.held || w.pendingToggleGripRetained, pendingPrimaryStart.source);
        const bool primaryOnlyStartRequested = weaponNode != nullptr && currentEquippedWeaponOwnershipKey != 0 &&
            ((w.primaryDetachFeatureAvailable && primaryState.held && primaryState.pressed) ||
                pendingPrimaryOnlyStartRequested);
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
            pendingPrimaryStart.hasFiringHandWeaponLocal = _twoHandedGrip.tryBuildCurrentLeftFiringGripCapture(
                weaponNode, currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
                pendingPrimaryStart.firingHandWeaponLocal, pendingPrimaryStart.firingGripWeaponLocal);
            pendingPrimaryStart.hasFiringGripWeaponLocal = pendingPrimaryStart.hasFiringHandWeaponLocal;
        }
        const RE::NiTransform* capturedFiringHandWeaponLocal =
            pendingMatches && pendingPrimaryStart.hasFiringHandWeaponLocal ? &pendingPrimaryStart.firingHandWeaponLocal
                                                                           : nullptr;
        const RE::NiPoint3* capturedFiringGripWeaponLocal =
            pendingMatches && pendingPrimaryStart.hasFiringGripWeaponLocal ? &pendingPrimaryStart.firingGripWeaponLocal
                                                                           : nullptr;
        const bool retainUntilPhysicalGrip = pendingMatches &&
            (pendingPrimaryStart.source == StartSource::DefaultEquip ||
                pendingPrimaryStart.source == StartSource::InventoryEquip || w.pendingToggleGripRetained);
        const auto observedWeaponName = [&]() {
            return w.observedEquippedWeapon ? RE::TESFullName::GetFullName(*w.observedEquippedWeapon, false)
                                            : std::string_view{ "unknown" };
        };
        // The left firing grip starts as soon as its own seat exists. It never
        // waits for the other hand's support candidate: that wait left the
        // weapon at the native right-hand pose for frames after a menu exit,
        // overlapping the right hand's weapon (runtime 22 Oct 11:39:07).
        if (!primaryOnlyStartRequested) {
            return;
        }

        const char* startFailureReason = nullptr;
        const bool pairedTransfer = pendingMatches && pendingPrimaryStart.pairedGrips.valid();
        const bool started = pairedTransfer
            ? _twoHandedGrip.beginTransferredTwoHandGrip(weaponNode, currentAuthoredGripGenerationKey,
                  currentEquippedWeaponOwnershipKey, pendingPrimaryStart.pairedGrips, &startFailureReason)
            : _twoHandedGrip.beginPrimaryOnlyGrip(weaponNode, currentAuthoredGripGenerationKey,
                  currentEquippedWeaponOwnershipKey, firingHandIsLeft, capturedFiringHandWeaponLocal,
                  capturedFiringGripWeaponLocal, retainUntilPhysicalGrip,
                  !pendingPrimaryStart.menuResume && pendingPrimaryStart.source != StartSource::DefaultEquip,
                  &startFailureReason);
        if (!started) {
            pendingPrimaryStart.lastStartFailureReason = startFailureReason;
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "Equipped hand transfer blocked weapon='{}' formID={:08X} hand={} reason={} generation={:016X} "
                "ownership={:016X} physicalHeld={} toggleRetained={}",
                observedWeaponName(), observedEquippedWeaponFormID, firingHandIsLeft ? "left" : "right",
                startFailureReason ? startFailureReason : "unknown", currentAuthoredGripGenerationKey,
                currentEquippedWeaponOwnershipKey, primaryState.held, w.pendingToggleGripRetained);
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
            (void)applyProviderWeaponPartDrives(weaponNode, w.currentAuthoredGripGenerationKey, frame,
                drivenSourceNodes);
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
        if (_twoHandedGrip.isFiringGripDetached() &&
            (w.primaryDetachFeatureAvailable || _twoHandedGrip.isHandsFreeRetained())) {
            const auto reattachEligible = [&](bool handAvailable, const Hand& hand) {
                return handAvailable &&
                    weapon_two_handed_grip_math::canAttemptFiringGripReattach(
                        weapon_two_handed_grip_math::FiringGripReattachInput{
                            .partCarryActive = true,
                            .menuInputActive = inputBlockingMenuActive,
                            .handHoldingObject = hand.isHolding(),
                        });
            };
            leftReattachEligible = reattachEligible(w.prepared.leftHandAvailableForAcquisition, _leftHand);
            rightReattachEligible = reattachEligible(w.prepared.rightHandAvailableForAcquisition, _rightHand);
            // Diagnostic: why a squeezing hand is not free to re-take the
            // firing grip (left reattach refused after a menu, 14 Oct).
            for (const bool isLeft : { false, true }) {
                const auto& hand = isLeft ? _leftHand : _rightHand;
                if (!input_remap_runtime::isRawButtonPhysicallyHeld(isLeft, input_remap_policy::kGrabButtonId) ||
                    (isLeft ? leftReattachEligible : rightReattachEligible))
                    continue;
                ROCK_LOG_SAMPLE_INFO(Weapon, 250,
                    "REATTACH ineligible hand={} available={} blockers={:#x} holding={} state={} menu={} "
                    "secondaryLeft={} secondaryRight={}",
                    isLeft ? "left" : "right",
                    isLeft ? w.prepared.leftHandAvailableForAcquisition : w.prepared.rightHandAvailableForAcquisition,
                    forceGrabHandBlockerMask(hand, isLeft, (isLeft ? frame.left : frame.right).disabled, true),
                    hand.isHolding(), static_cast<unsigned>(hand.getState()), inputBlockingMenuActive,
                    w.prepared.secondaryLeft, w.prepared.secondaryRight);
            }
        }

        // Toggle grab turns each hand's button into a logical held state.
        const auto toToggleButtonState = [](const GrabButtonState& state) {
            return equipped_weapon_toggle_grab_policy::ButtonState{
                .held = state.held,
                .pressed = state.pressed,
                .released = state.released,
            };
        };
        auto toggleGrabDecision = equipped_weapon_toggle_grab_policy::prepare(_equipped.toggleGrabState,
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
                _equipped.handlingSettings.weaponGrabMode, currentEquippedWeaponOwnershipKey,
                _twoHandedGrip.getGrabInputOccupancy());
            for (const bool isLeft : { false, true }) {
                if (w.supportGripStartedThisFrame && !w.transferredBothSupports &&
                    isLeft != w.transferredSupportIsLeft) {
                    continue;
                }
                const auto index = equipped_weapon_toggle_grab_policy::handIndex(isLeft);
                const bool released = w.pairedRelease[index].releaseRequested;
                (isLeft ? toggleGrabDecision.left : toggleGrabDecision.right) = { .held = !released,
                    .released = released };
                (isLeft ? toggleGrabDecision.leftReleasePressConsumed : toggleGrabDecision.rightReleasePressConsumed) =
                    released;
                if (released) {
                    _equipped.toggleGrabState.hands[index] =
                        equipped_weapon_toggle_grab_policy::HandState::ReleasePending;
                }
            }
        }
        const auto firingHandIndex = equipped_weapon_toggle_grab_policy::handIndex(firingHandIsLeft);
        if (w.primaryOnlyGripStartedThisFrame && !w.pairedGripStartedThisFrame &&
            w.pairedRelease[firingHandIndex].releaseRequested) {
            // Inventory already owns the weapon. Apply the saved release
            // through its requested hand and configured drop mode.
            (void)_twoHandedGrip.commitPersistentEquippedCarryInputAcquisition(firingHandIsLeft);
            (firingHandIsLeft ? toggleGrabDecision.left : toggleGrabDecision.right) = { .released = true };
            (firingHandIsLeft ? toggleGrabDecision.leftReleasePressConsumed
                              : toggleGrabDecision.rightReleasePressConsumed) = true;
            _equipped.toggleGrabState.hands[firingHandIndex] =
                equipped_weapon_toggle_grab_policy::HandState::ReleasePending;
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
            releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(true)] ||
            toggleGrabDecision.leftReleasePressConsumed;
        releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(false)] =
            releasePressConsumed[equipped_weapon_toggle_grab_policy::handIndex(false)] ||
            toggleGrabDecision.rightReleasePressConsumed;

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
        // Index 0's grip only: a held second-weapon scope is not this weapon's.
        const bool manualScopeActivationRequested = input_remap_runtime::manualScopeSlot() == 0;
        const bool pairedGripPending = _equipped.transition.pendingGrip().pairedGrips.valid() ||
            _equipped.transition.pendingGrip().supportGrip.validCarry();
        const auto toGripInput = [](const GrabButtonState& state) {
            return EquippedWeaponPrimaryGripInput{ .held = state.held,
                .pressed = state.pressed,
                .released = state.released };
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
        effectiveHandlingSettings.detachAuthority = w.primaryDetachFeatureAvailable
            ? firingGripDecision.authority
            : immersive_weapon_policy::DetachAuthority::None;
        effectiveHandlingSettings.firingGripReattachRadiusGameUnits = firingGripDecision.reattachRadiusGameUnits;
        effectiveHandlingSettings.weaponGripHapticDurationSeconds = firingGripDecision.gripHapticDurationSeconds;
        effectiveHandlingSettings.firingGripAttachHapticIntensity = firingGripDecision.gripAttachHapticIntensity;
        effectiveHandlingSettings.firingGripDetachHapticIntensity = firingGripDecision.gripDetachHapticIntensity;
        // What a squeeze of the detached firing hand found, one line per press.
        if (_grabInput.firingHandButtonFrame.valid && _grabInput.firingHandButtonFrame.pressed &&
            _twoHandedGrip.isFiringGripDetached()) {
            const auto& contact = firingHandIsLeft ? w.leftWeaponContact : w.rightWeaponContact;
            const auto& state = firingHandIsLeft ? w.leftHandInteractionState : w.rightHandInteractionState;
            ROCK_LOG_INFO(Weapon,
                "FREE_HAND_PRESS hand={} logical(held={} pressed={}) available={} holding={} contact(valid={} body={} "
                "point={} distance={:.2f} generation={:016X}) allowed={} provider={} key={:016X}",
                firingHandIsLeft ? "left" : "right", w.primaryGripInput.held, w.primaryGripInput.pressed,
                firingHandIsLeft ? w.prepared.leftHandAvailableForAcquisition
                                 : w.prepared.rightHandAvailableForAcquisition,
                (firingHandIsLeft ? _leftHand : _rightHand).isHolding(), contact.valid, contact.bodyId,
                contact.grabPoint.pointId, contact.probeDistanceGame, contact.weaponGenerationKey,
                state.supportGripAllowed, state.providerPartAuthority.active, w.currentAuthoredGripGenerationKey);
        }
        w.gripUpdateResult = _twoHandedGrip.update(weaponNode, w.leftWeaponContact, w.rightWeaponContact,
            gripFrameInput, frame.deltaSeconds, w.currentAuthoredGripGenerationKey, currentEquippedWeaponOwnershipKey,
            _weaponCollision, w.leftHandInteractionState, w.rightHandInteractionState, w.supportAuthorityMode,
            w.firingGripProximityAuthorityEnabled, effectiveHandlingSettings);

        const auto toggleReconcileDecision = equipped_weapon_toggle_grab_policy::reconcile(_equipped.toggleGrabState,
            _equipped.handlingSettings.weaponGrabMode, currentEquippedWeaponOwnershipKey,
            _twoHandedGrip.getGrabInputOccupancy(),
            equipped_weapon_toggle_grab_policy::GripReleaseRetention{
                .left = w.gripUpdateResult.releaseRetained.left,
                .right = w.gripUpdateResult.releaseRetained.right,
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
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon firing-grip ownership started from grip input or held-weapon equip");
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
            isLeft ? "left" : "right", acquisitionInput.pressed ? "yes" : "no", acquisitionInput.held ? "yes" : "no",
            acquisitionInput.released ? "yes" : "no");
    }

    // Attach/detach pulses from the grip update, and the continuous hover
    // pulse while a free palm sits in the firing-grip reattach zone.
    void PhysicsInteraction::queueEquippedWeaponGripHaptics()
    {
        const auto& settings = _equipped.handlingSettings;
        const auto gripHapticEvents = _twoHandedGrip.consumeHapticEvents();
        const auto queueGripHaptic = [this](const bool isLeft, const float durationSeconds, const float intensity) {
            (void)_feedbackHaptics.queue(isLeft ? feedback_haptics::FeedbackHand::Left
                                                : feedback_haptics::FeedbackHand::Right,
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
            (void)_feedbackHaptics.queue(_twoHandedGrip.isFiringGripReattachHoverHandLeft()
                    ? feedback_haptics::FeedbackHand::Left
                    : feedback_haptics::FeedbackHand::Right,
                grip_zone_hover_haptic_policy::kContinuousQueueSeconds, settings.gripZoneHoverHapticIntensity);
        }
    }

    // A shoulder stash the gesture selected, then a drop the grip update
    // requested. A stash commit replaces the physical drop.
    void PhysicsInteraction::commitEquippedWeaponStashAndDrop(EquippedWeaponFrameWork& w)
    {
        const auto& frame = w.frame;
        RE::NiNode* weaponNode = w.prepared.weaponNode;
        const auto& shoulderDecision = w.shoulderFrame.decision;

        const bool shoulderStashSelected = shoulderDecision.action == equipped_weapon_shoulder::Action::SubmitStash;
        if (shoulderStashSelected) {
            if (w.shoulderFrame.sourceHand != equipped_weapon_drop_policy::SourceHand::None) {
                const bool left = equipped_weapon_drop_policy::isLeft(w.shoulderFrame.sourceHand);
                // The carry ends before the weapon leaves, as a drop's would.
                equipped_weapon_toggle_grab_policy::reset(_equipped.toggleGrabState);
                _twoHandedGrip.restoreNativeRightEquippedCarry("shoulder-weapon-stashed");
                _equipped.transition.pendingGrip() = {};
                (void)stashEquippedWeaponAtShoulder(frame, 0, left, w.shoulderFrame.detectorDecision);
                shoulder_stash::resetRuntime(_equipped.stashStates[left ? 1u : 0u]);
            } else {
                ROCK_LOG_ERROR(Weapon, "Equipped shoulder coordinator selected a stash without a source hand");
            }
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
        const RE::NiPoint3 dropLoc = sourceHandKnown
            ? (equipped_weapon_drop_policy::isLeft(sourceHand) ? frame.left.grabAnchorWorld
                                                               : frame.right.grabAnchorWorld)
            : (weaponNode ? weaponNode->world.translate : frame.right.grabAnchorWorld);
        if (w.inputBlockingMenuActive) {
            ROCK_LOG_INFO(Weapon,
                "Equipped weapon manual release suppressed because an input-blocking menu is active sourceHand={} "
                "releaseLoc=({:.1f},{:.1f},{:.1f})",
                equipped_weapon_drop_policy::sourceHandName(sourceHand), dropLoc.x, dropLoc.y, dropLoc.z);
        } else {
            const bool stashCommitSelected = shoulderStashSelected || shoulderDecision.suppressEquippedDrop;
            if (equipped_weapon_drop_policy::shouldAttemptPhysicalDrop(stashCommitSelected)) {
                transferCommitted =
                    dropEquippedWeaponToWorld(frame, equippedWeaponDropRequest, equippedWeaponDropRequest.mode);
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
                _twoHandedGrip.getGrabInputOccupancy(),
                {
                    .left = sourceHand == equipped_weapon_drop_policy::SourceHand::Left,
                    .right = sourceHand == equipped_weapon_drop_policy::SourceHand::Right,
                });
        }
    }

    // Publishes the grip result to input remapping and the provider, and
    // updates which hands' colliders the weapon suppresses.
    PhysicsInteraction::EquippedWeaponFrameResult PhysicsInteraction::publishEquippedWeaponGripOwnership(
        EquippedWeaponFrameWork& w)
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
            const bool firingLeft = _twoHandedGrip.isManualOwnershipActive() && _twoHandedGrip.isFiringHandLeft();
            native_hand_equipment::recordPrimaryHand(w.observedEquippedWeaponFormID,
                w.observedEquippedWeaponInstanceData, partCarry ? _twoHandedGrip.weaponCarrierIsLeft() : firingLeft,
                !partCarry,
                !partCarry && _twoHandedGrip.isManualOwnershipActive() &&
                    _twoHandedGrip.isHandPartCarryGripping(!firingLeft));
        }

        bool rightHandWeaponAuthorityActive =
            prepared.rightHandWeaponEquipped || prepared.retainedWeaponCollisionActive;
        // A visible part-carry frees the right hand even while weapon bodies exist (see beginEquippedWeaponFrame).
        if (prepared.rightHandWeaponEquipped && _twoHandedGrip.isFiringGripDetached()) {
            rightHandWeaponAuthorityActive = false;
        }
        // Left-firing carry frees the right hand the same way.
        if (prepared.rightHandWeaponEquipped && leftHandFiringActiveAfterGrip) {
            rightHandWeaponAuthorityActive = false;
        }
        if (rightHandWeaponAuthorityActive != prepared.rightHandWeaponAuthorityActive) {
            setRightFiringWeaponOwnership(w.hknp, rightHandWeaponAuthorityActive, prepared.rightHandWeaponEquipped);
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
            gripUpdateResult.before.left.weaponEngaged() ? g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds
                                                         : 0.0f);

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
        const bool surfaceSupportPrimaryValid = _twoHandedGrip.tryGetSurfaceSupportPrimaryGripLocal(weaponNode,
            currentWeaponGenerationKey, surfaceSupportPrimaryGripLocal);
        const auto dynamicWeaponFrame = _dynamicWeaponCollision.finishFrame(frame,
            physicsWritesAllowedForWorld(frame.hknpWorld), weaponNode, currentWeaponGenerationKey, _weaponCollision,
            surfaceSupportPrimaryValid ? &surfaceSupportPrimaryGripLocal : nullptr,
            _twoHandedGrip.isSurfaceSupported());
        if (_dynamicWeaponCollision.compoundSourcesUnavailable()) {
            _weaponCollision.requestRebuildForReplacedSources();
        }
        const bool dynamicWeaponDebugEnabled =
            g_rockConfig.rockDebugShowColliders && g_rockConfig.rockDebugDrawDynamicWeaponColliders;
        if (dynamicWeaponFrame.contactEpisodeStarted && dynamicWeaponDebugEnabled) {
            auto* otherRef =
                resolveBodyToRef(frame.bhkWorld, frame.hknpWorld, RE::hknpBodyId{ dynamicWeaponFrame.otherBodyId });
            const auto* otherBase = otherRef ? otherRef->GetObjectReference() : nullptr;
            const auto otherNameView = otherBase ? RE::TESFullName::GetFullName(*otherBase, false) : std::string_view{};
            const std::string otherName =
                otherNameView.empty() ? std::string("(unresolved)") : std::string(otherNameView);
            const char* otherType = otherBase ? otherBase->GetFormTypeString() : "unresolved";

            WeaponCollision::WeaponSurfaceProximityWitness partWitness{};
            WeaponInteractionDebugInfo partInfo{};
            constexpr float kContactPartSearchRadiusGameUnits = 24.0f;
            const bool partWitnessValid = dynamicWeaponFrame.rawContactPointValid &&
                _weaponCollision.tryFindCurrentWeaponSurfaceNearPoint(weaponNode,
                    dynamicWeaponFrame.rawContactPointGame, kContactPartSearchRadiusGameUnits, partWitness);
            const bool partInfoValid =
                partWitnessValid && _weaponCollision.tryGetWeaponContactDebugInfo(partWitness.bodyId, partInfo);

            ROCK_LOG_INFO(Weapon,
                "DWC contact witness: episode={} solveAge={} generation={:016X} weaponForm={:08X} "
                "other(body/layer/motion/ref/form/type/name)=({}/{}/{}/{:p}/{:08X}/{}/{}) "
                "native(collObj/owner)=({:p}/{:p}) raw(valid/proxyWasA/points/index/weight)={}/{}/{}/{}/{:.3f} "
                "pointGame=({:.2f},{:.2f},{:.2f}) normalRaw=({:.3f},{:.3f},{:.3f}) "
                "nearestPart(valid/body/source/distance/current)={}/{}/{}/{:.3f}/{}",
                dynamicWeaponFrame.contactEpisode, dynamicWeaponFrame.contactSolveAge, currentWeaponGenerationKey,
                _weaponCollision.getCurrentObservedEquippedWeaponFormID(), dynamicWeaponFrame.otherBodyId,
                dynamicWeaponFrame.otherLayer, dynamicWeaponFrame.otherMotionIndex, static_cast<void*>(otherRef),
                otherRef ? otherRef->GetFormID() : 0, otherType, otherName,
                reinterpret_cast<void*>(dynamicWeaponFrame.otherCollisionObject),
                reinterpret_cast<void*>(dynamicWeaponFrame.otherOwnerNode), dynamicWeaponFrame.rawContactPointValid,
                dynamicWeaponFrame.rawContactProxyWasBodyA, dynamicWeaponFrame.rawContactPointCount,
                dynamicWeaponFrame.rawContactPointIndex, dynamicWeaponFrame.rawContactPointWeightSum,
                dynamicWeaponFrame.rawContactPointGame.x, dynamicWeaponFrame.rawContactPointGame.y,
                dynamicWeaponFrame.rawContactPointGame.z, dynamicWeaponFrame.rawContactNormalHavok.x,
                dynamicWeaponFrame.rawContactNormalHavok.y, dynamicWeaponFrame.rawContactNormalHavok.z,
                partWitnessValid, partWitnessValid ? partWitness.bodyId : 0x7FFF'FFFFu,
                partInfoValid ? partInfo.sourceName : std::string("(unresolved)"),
                partWitnessValid ? partWitness.distanceGameUnits : -1.0f,
                partWitnessValid && partWitness.sourceNodeCurrent);
            ROCK_LOG_INFO(Weapon,
                "DWC contact transforms: episode={} requestedBody=({:.2f},{:.2f},{:.2f}) "
                "liveBody=({:.2f},{:.2f},{:.2f}) otherReadable={} otherBody=({:.2f},{:.2f},{:.2f}) "
                "correction=({:.2f}gu,{:.2f}deg)",
                dynamicWeaponFrame.contactEpisode, dynamicWeaponFrame.requestedContactBodyWorld.translate.x,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.y,
                dynamicWeaponFrame.requestedContactBodyWorld.translate.z,
                dynamicWeaponFrame.liveContactBodyWorld.translate.x,
                dynamicWeaponFrame.liveContactBodyWorld.translate.y,
                dynamicWeaponFrame.liveContactBodyWorld.translate.z, dynamicWeaponFrame.otherBodyWorldValid,
                dynamicWeaponFrame.otherBodyWorld.translate.x, dynamicWeaponFrame.otherBodyWorld.translate.y,
                dynamicWeaponFrame.otherBodyWorld.translate.z, dynamicWeaponFrame.translationCorrectionGameUnits,
                dynamicWeaponFrame.rotationCorrectionDegrees);
        }
        if (!dynamicWeaponFrame.applyVisualCorrection) {
            return;
        }
        const bool visualPublishSucceeded = _twoHandedGrip.applyWeaponCollisionResolvedAuthority(weaponNode,
            dynamicWeaponFrame.resolvedWeaponWorld, currentWeaponGenerationKey);
        if (dynamicWeaponDebugEnabled) {
            const bool measured = visualPublishSucceeded && weaponNode;
            const float immediateTranslationError = measured
                ? dynamic_weapon_collision_policy::translationDeltaGameUnits(weaponNode->world,
                      dynamicWeaponFrame.resolvedWeaponWorld)
                : -1.0f;
            const float immediateRotationError = measured
                ? dynamic_weapon_collision_policy::rotationDeltaDegrees(weaponNode->world,
                      dynamicWeaponFrame.resolvedWeaponWorld)
                : -1.0f;
            ROCK_LOG_SAMPLE_INFO(Weapon, 500,
                "DWC visual publication: bodyActive={} publishSucceeded={} immediateError=({:.3f}gu,{:.3f}deg) "
                "requestedCorrection=({:.3f}gu,{:.3f}deg)",
                dynamicWeaponFrame.proxyActive, visualPublishSucceeded, immediateTranslationError,
                immediateRotationError, dynamicWeaponFrame.translationCorrectionGameUnits,
                dynamicWeaponFrame.rotationCorrectionDegrees);
        }
    }

    void PhysicsInteraction::publishGripZoneIndicatorRenderFrame(const std::uint64_t gameFrameIndex)
    {
        const auto& runtime = runtime_state::currentFrame();
        if (!_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking ||
            gameFrameIndex == 0 || gameFrameIndex != runtime.frameIndex ||
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
        overlayFrame.diameterGameUnits = g_rockConfig.rockGripZoneIndicatorDiameterGameUnits;
        overlayFrame.count +=
            authored_interaction::appendIndicators(std::span(overlayFrame.positions).subspan(overlayFrame.count));

        if (_powerArmorCandidateFrame == gameFrameIndex) {
            for (const bool isLeft : { false, true }) {
                const auto& candidate = _powerArmorCandidates[isLeft ? 1u : 0u];
                if (candidate.valid && !_touchGrabRuntime.isHandActive(isLeft) &&
                    !(isLeft ? _leftHand : _rightHand).isHolding()) {
                    overlayFrame.positions[overlayFrame.count++] = candidate.positionGame;
                }
            }
        }

        for (const bool isLeft : { false, true }) {
            const auto& hand = isLeft ? _leftHand : _rightHand;
            if (!hand.isHolding() && hand.hasSelection() && !_touchGrabRuntime.isHandActive(isLeft)) {
                overlayFrame.count += static_cast<std::uint32_t>(
                    loose_weapon_grip_zone::collectIndicators(isLeft, hand.getSelection().refr,
                        std::span<RE::NiPoint3>(overlayFrame.positions).subspan(overlayFrame.count)));
            }
        }

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        // The key the grips run on, so an authored seat or a grab point can be shown
        // from the moment the weapon is drawn, as it can be taken.
        const std::uint64_t currentWeaponGenerationKey = _weaponCollision.getCurrentWeaponInteractionKey();
        if (!weaponNode || !f4vr::isNodeVisible(weaponNode) || currentWeaponGenerationKey == 0 ||
            !dynamic_weapon_collision_policy::isFiniteTransform(weaponNode->world) ||
            std::abs(weaponNode->world.scale) <= 0.0001f) {
            grip_zone_indicators::Publish(overlayFrame);
            return;
        }

        const auto appendIndicator = [&](const auto& indicatorFrame, const Hand& hand) {
            if (!indicatorFrame.visible || hand.isHolding() || !indicatorFrame.weaponLocalValid ||
                indicatorFrame.weaponGenerationKey != currentWeaponGenerationKey ||
                overlayFrame.count >= overlayFrame.positions.size()) {
                return;
            }

            const RE::NiPoint3 positionWorld =
                transform_math::localPointToWorld(weaponNode->world, indicatorFrame.positionWeaponLocal);
            if (!std::isfinite(positionWorld.x) || !std::isfinite(positionWorld.y) || !std::isfinite(positionWorld.z)) {
                return;
            }
            overlayFrame.positions[overlayFrame.count++] = positionWorld;
        };

        for (const bool isLeft : { true, false }) {
            appendIndicator(_twoHandedGrip.getAuthoredSupportGripIndicatorFrame(isLeft),
                isLeft ? _leftHand : _rightHand);
        }

        const auto firingReattachFrame = _twoHandedGrip.getFiringGripReattachIndicatorFrame();
        appendIndicator(firingReattachFrame, firingReattachFrame.handIsLeft ? _leftHand : _rightHand);

        if (_equipped.partIndicatorFrame == gameFrameIndex &&
            _equipped.partIndicatorGeneration == currentWeaponGenerationKey) {
            auto candidateBodyIds = _equipped.partIndicatorBodyIds;
            auto pointCandidates = _equipped.partIndicatorPointCandidates;
            const auto roster = equippedWeaponRoster();
            const auto& slotZero = roster.facts().slot0;
            for (const bool isLeft : { true, false }) {
                const auto& hand = isLeft ? _leftHand : _rightHand;
                if (hand.isHolding() || _touchGrabRuntime.isHandActive(isLeft) ||
                    roster.slotZeroGrips(isLeft).weaponEngaged() || roster.slotOneHolds(isLeft) ||
                    (slotZero.sessionFiringHand == equipped_weapon_roster::handFor(isLeft) &&
                        !slotZero.firingGripDetached)) {
                    candidateBodyIds[isLeft ? 0u : 1u] = weapon_part_runtime::kInvalidBodyId;
                    pointCandidates[isLeft ? 0u : 1u] = false;
                }
            }
            if (pointCandidates[0] || pointCandidates[1]) {
                std::array<weapon_part_runtime::Target, api::weaponparts::kMaxTargets> partTargets{};
                const auto targetCount = provider::copyWeaponPartTargets(partTargets);
                overlayFrame.count += static_cast<std::uint32_t>(_weaponCollision.collectAttachOnlyGripIndicators(
                    weaponNode, std::span(partTargets).first(targetCount), candidateBodyIds,
                    std::span(overlayFrame.positions).subspan(overlayFrame.count), _equipped.partIndicatorPoints,
                    pointCandidates));
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
        applyWeaponPresentationScale(false);
    }

    /*
     * userOffsetOnly: the pre-solve pass. The native graph resets the Weapon
     * scale every frame before ROCK's solves, which read the node's scale for
     * the hand seat and collision; a player FRIK offset's scale must already
     * be on the node then, not only re-applied for the render.
     */
    void PhysicsInteraction::applyWeaponPresentationScale(const bool userOffsetOnly)
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        auto* equippedWeapon = currentEquippedWeaponForm();
        if (!weaponNode || !equippedWeapon || !_lifecycle.initialized.load(std::memory_order_acquire) ||
            !runtime.visualAuthorityAvailable || !runtime.localSkeletonReady || runtime.localMenuBlocking ||
            runtime.compatibilityConfigBlocking || !runtime.weaponDrawn) {
            return;
        }
        // Same freshness boundary as the authored grip: the richer
        // stack/instance key when generated collision is on, the equipped form otherwise.
        std::uint64_t weaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        if (weaponOwnershipKey == 0) {
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }
        const std::uint64_t weaponGenerationKey = authored_support_grab_policy::resolveAuthoredGenerationKey(
            _weaponCollision.getCurrentWeaponGenerationKey(), weaponOwnershipKey);
        const bool equippedGenerationMatchesForm = weaponGenerationKey != 0 && weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;
        const auto nativeAuthorityFlags = provider::currentNativeAnimationAuthorityFlags();
        if (!equippedGenerationMatchesForm || !weaponNode->parent || !f4vr::isNodeVisible(weaponNode) ||
            (nativeAuthorityFlags & authored_weapon_grip_capture_policy::kWeapon) != 0) {
            return;
        }
        const float parentScale = weaponNode->parent->world.scale;
        float animationScale = 1.0f;
        const bool animationAvailable = authored_weapon_grip_capture::tryGetAnimationWeaponScale(weaponNode,
            equippedWeapon->formID, animationScale);
        // A player FRIK weapon offset resizes the weapon relative to this
        // baseline; the scale carries into every solve that reads the node.
        const auto userOffset = frik_weapon_offsets::find(equippedWeapon, weaponNode, f4vr::isInPowerArmor());
        if (userOffsetOnly && !userOffset) {
            return;
        }
        const float userScale = userOffset ? userOffset->weaponScale() : 1.0f;
        const float desiredScale =
            authored_weapon_grip_capture_policy::resolveWeaponPresentationScale(animationAvailable, animationScale) *
            userScale;
        if (std::isfinite(parentScale) && std::abs(parentScale) > 0.0001f &&
            std::abs(weaponNode->world.scale - desiredScale) > 0.00001f) {
            const float previousScale = weaponNode->world.scale;
            weaponNode->local.scale = desiredScale / parentScale;
            f4vr::updateDown(weaponNode, true);
            ROCK_LOG_SAMPLE_INFO(Animation, 2000,
                "Weapon presentation scale form={:08X} incoming={:.6f} target={:.6f} result={:.6f} source={} "
                "frikUserScale={:.4f} pass={}",
                equippedWeapon->formID, previousScale, desiredScale, weaponNode->world.scale,
                animationAvailable ? "animation-graph" : "unit-default", userScale,
                userOffsetOnly ? "pre-solve" : "after-weapon-position");
        }
    }

    void PhysicsInteraction::synchronizeEquipVisualBridgeAfterFrikWeaponPass()
    {
        const auto& runtime = runtime_state::currentFrame();
        if (!_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking)
            return;
        // The native Weapon may still be hidden by draw animation or the
        // bridge itself. Its accepted grip solve nevertheless owns both hands.
        _equipped.transition.synchronizeEquippedPresentation(resolveEquippedWeaponInteractionNode(),
            currentEquippedWeaponFormId());
    }

    void PhysicsInteraction::finalizeEquipVisualBridgePresentation()
    {
        const auto& runtime = runtime_state::currentFrame();
        auto* player = f4vr::getPlayer();
        if (!player || !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking ||
            provider::currentNativeAnimationAuthorityFlags() != 0 ||
            f4vr::getNativeGunState(player) == static_cast<std::uint32_t>(RE::GUN_STATE::kReloading))
            return;
        _equipped.transition.finalizeVisualPresentation();
    }

    void PhysicsInteraction::updateAuthoredPrimaryFiringGrip()
    {
        applyWeaponPresentationScale(true);
        const auto& runtime = runtime_state::currentFrame();
        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        // The authored/native canonical weapon frame is always ROCK's
        // physical-right primary controller, independent of FO4VR settings.
        const bool leftHandHoldingObject = _leftHand.isHolding();
        const bool rightHandHoldingObject = _rightHand.isHolding();
        _twoHandedGrip.setGrabbedObjectHandPoseOwnership(leftHandHoldingObject, rightHandHoldingObject);
        const auto nativeAuthorityFlags = provider::currentNativeAnimationAuthorityFlags();
        auto* equippedWeapon = currentEquippedWeaponForm();
        const auto weaponClassification = _weaponCollision.getEquippedWeaponClassification();
        const std::uint64_t weaponKeywordFlags = equippedWeapon && weaponClassification.hasEquippedWeapon &&
                weaponClassification.formID == equippedWeapon->formID
            ? weaponClassification.keywordFlags
            : 0;
        const bool classificationCurrent = equippedWeapon && weaponClassification.hasEquippedWeapon &&
            weaponClassification.formID == equippedWeapon->formID;
        std::uint64_t weaponOwnershipKey = weaponNode ? _weaponCollision.getCurrentEquippedWeaponOwnershipKey() : 0;
        if (weaponNode && weaponOwnershipKey == 0) {
            /*
             * The collision update publishes the key later in this frame, and
             * clears it while the weapon is not drawn. This frame's
             * classification gives the same key now. A session started on a
             * stand-in key met the real one a frame later as a false weapon
             * boundary, which dropped the grip for a frame on the first drawn
             * frames after every menu draw. The equipped form is left for an
             * item with no instance witness.
             */
            if (classificationCurrent)
                weaponOwnershipKey = weapon_generation_identity_policy::makeEquippedWeaponOwnershipKey(weaponClassification);
            if (weaponOwnershipKey == 0)
                weaponOwnershipKey = currentEquippedWeaponFormId();
        }
        const std::uint64_t collisionGenerationKey = weaponNode ? _weaponCollision.getCurrentWeaponGenerationKey() : 0;
        const std::uint64_t weaponGenerationKey =
            authored_support_grab_policy::resolveAuthoredGenerationKey(collisionGenerationKey, weaponOwnershipKey);

        // Drawn frames only: the collision's observed identity is cleared otherwise.
        const bool equippedGenerationMatchesForm = weaponNode && classificationCurrent && runtime.weaponDrawn &&
            weaponGenerationKey != 0 && weaponOwnershipKey != 0;
        const std::uint64_t weaponInstanceContentKey =
            equippedGenerationMatchesForm ? weaponClassification.instanceContentKey : 0;
        const bool equippedWeaponTransitionActive = _equipped.transition.getPublicSnapshot().active;

        native_idle_grip_preharvest::observeEquippedWeapon(equippedGenerationMatchesForm ? equippedWeapon : nullptr,
            equippedGenerationMatchesForm ? weaponNode : nullptr,
            equippedGenerationMatchesForm ? currentEquippedWeaponInstanceData(equippedWeapon) : nullptr,
            weaponInstanceContentKey);
        // After the equipped weapon's own harvest, which keeps priority.
        native_idle_grip_preharvest::observeConsumerWarmups();

        _twoHandedGrip.beginAuthoredPrimaryFiringGripFrame();
        const auto* equippedInstance = currentEquippedWeaponInstanceData(equippedWeapon);
        const auto& pendingGrip = _equipped.transition.pendingGrip();
        // Prepare the accepted left firing seat before acquisition. Waiting
        // for the live firing role would require right-hand alignment first,
        // which an outgoing loose weapon correctly prevents.
        const bool pendingLeftFiringGrip = pendingGrip.pending && pendingGrip.isLeft &&
            !pendingGrip.supportGrip.validCarry() &&
            equipped_weapon_transition_policy::matchesExpectedIdentity(equippedWeapon ? equippedWeapon->formID : 0u,
                reinterpret_cast<std::uintptr_t>(equippedInstance), pendingGrip.targetWeaponFormID,
                pendingGrip.targetWeaponInstanceData, pendingGrip.previousWeaponFormID,
                pendingGrip.previousWeaponInstanceData);
        const auto* effectiveWeaponData = equippedInstance
            ? static_cast<const RE::TESObjectWEAP::InstanceData*>(equippedInstance)
            : equippedWeapon ? &equippedWeapon->weaponData
                             : nullptr;
        frik_hand_world_authority::prepareWeaponInputs(weaponOwnershipKey, weaponGenerationKey,
            effectiveWeaponData && effectiveWeaponData->type.get() == RE::WEAPON_TYPE::kGun &&
                !(nativeAuthorityFlags & authored_weapon_grip_capture_policy::kWeapon));
        if (frik_hand_world_authority::weaponFilteringActive()) {
            // Authored grip alignment precedes the ordinary interaction pass.
            // Resolve this frame's isolated input before its first filter read.
            refreshHandBoneCache();
        }
        _equipped.authoredPrimaryFiringGrip.update(
            AuthoredPrimaryFiringGripFrameInput{
                .weaponNode = weaponNode,
                .weapon = equippedWeapon,
                .weaponOwnershipKey = weaponOwnershipKey,
                .weaponGenerationKey = weaponGenerationKey,
                .weaponInstanceContentKey = weaponInstanceContentKey,
                .weaponKeywordFlags = weaponKeywordFlags,
                .weaponInstanceContentKnown = equippedGenerationMatchesForm,
                .meleeWeapon =
                    (effectiveWeaponData && weapon_type_policy::isEquippedMelee(effectiveWeaponData->type.get())) ||
                    (equippedWeapon && weaponClassification.formID == equippedWeapon->formID &&
                        weaponClassification.classificationResolved &&
                        weaponClassification.sizeClass == WeaponSizeClass::Melee),
                .meleeFlatPlaneZ = _weaponCollision.holdingMirrorOffsets().z,
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
                    (nativeAuthorityFlags & authored_weapon_grip_capture_policy::kWeapon) != 0,
                .conflictingWeaponTransformAuthorityActive = _equipped.gripResumePending ||
                    _equipped.transition.pendingGrip().menuResume ||
                    _twoHandedGrip.blocksAuthoredPrimaryGripWeaponAlignment(),
                .weaponVisualReturnActive = _twoHandedGrip.isWeaponVisualReturnActive(),
                .equippedWeaponTransitionActive = equippedWeaponTransitionActive,
                .primaryHandHoldingObject = rightHandHoldingObject,
                .rockFiringHandIsLeft = _twoHandedGrip.isFiringHandLeft(),
                .pendingLeftFiringGrip = pendingLeftFiringGrip,
                .inPowerArmor = f4vr::isInPowerArmor(),
            },
            _twoHandedGrip);
        _twoHandedGrip.finishAuthoredPrimaryFiringGripFrame();
        if (_equipped.transition.isHandPoseHandoffActive()) {
            const bool handoffHandIsLeft = _equipped.transition.handPoseHandoffIsLeft();
            const bool pairedSupportOwned =
                !_twoHandedGrip.isFiringGripDetached() && _twoHandedGrip.isHandPartGripping(!handoffHandIsLeft);
            if (nativeAuthorityFlags != 0 || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking) {
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
        if (!_equipped.transition.isHandPoseHandoffActive())
            return;
        const bool handIsLeft = _equipped.transition.handPoseHandoffIsLeft();
        const bool pairedSupportOwned =
            !_twoHandedGrip.isFiringGripDetached() && _twoHandedGrip.isHandPartGripping(!handIsLeft);
        const bool supportGripOwned =
            _equipped.transition.hasSupportOnlyHandPoseHandoff() && _twoHandedGrip.isHandPartGripping(handIsLeft);
        if ((_twoHandedGrip.hasPublishedAuthoredPrimaryFiringGripFingerPose(handIsLeft) || supportGripOwned) &&
            _twoHandedGrip.hasVisualAuthorityForHand(handIsLeft) &&
            !_twoHandedGrip.isHandVisualReturnActive(handIsLeft) &&
            (!_equipped.transition.hasPairedHandPoseHandoff() || pairedSupportOwned)) {
            _equipped.transition.tryCompleteHandPoseHandoff();
            if (!_equipped.transition.isHandPoseHandoffActive())
                _equipped.transition.recordGripPresentation();
        }
    }

    void PhysicsInteraction::refreshEquippedWeaponHandlingSettings()
    {
        api::weapon::HandlingRequest request{};
        const bool externalAuthorityActive = ::rock::provider::getHandlingAuthorityRequest(request);
        const auto weaponClassification = _weaponCollision.getEquippedWeaponClassification();
        const RockEquippedWeaponHandlingBaseline rockBaseline{
            .leftHandedModeEnabled = g_rockConfig.rockDefaultHandLeft,
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
                g_rockConfig.rockEquippedWeaponShoulderStashEnabled,
            .lastGripReleaseDropEnabled =
                equipped_weapon_drop_policy::fromEquipMode(g_rockConfig.rockWeaponEquipMode) != equipped_weapon_drop_policy::Mode::Off,
            .autoEquipEnabled = equipped_weapon_drop_policy::autoEquipFromEquipMode(g_rockConfig.rockWeaponEquipMode),
            .autoEquipSettleSeconds = g_rockConfig.rockAutoEquipSettleSeconds,
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
        auto settings = makeEquippedWeaponHandlingSettings(rockBaseline, externalAuthorityActive ? &request : nullptr);

        if (_equipped.handlingModeInitialized) {
            if (requiresEquippedWeaponHandlingModeReconcile(_equipped.handlingSettings, settings)) {
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

        _twoHandedGrip.restoreNativeRightEquippedCarry("equipped-weapon-handling-mode-changed");
        _equipped.transition.pendingGrip() = {};
        _equipped.handlingModeReconcilePending = false;
    }

    bool PhysicsInteraction::stashEquippedWeaponAtShoulder(const PhysicsFrameContext& frame, const std::uint32_t index,
        const bool left, const shoulder_stash::Decision& stashDecision)
    {
        // Borrowed for this call only; the identity is copied before the unequip.
        const auto view = native_hand_equipment::equippedViews()[index];
        if (!view.weapon) {
            ROCK_LOG_WARN(Weapon, "Equipped weapon shoulder stash refused index={} hand={}: no weapon at that index",
                index, left ? "left" : "right");
            return false;
        }
        const auto formID = view.weapon->GetFormID();
        const auto instance = reinterpret_cast<std::uintptr_t>(view.instance);
        const std::string itemName(shoulderStashItemName(view.weapon));
        // The second graph retires with its model; stop its actions first,
        // as the second weapon's drop does.
        if (index == 1)
            native_secondary_actions::suspend();
        const bool stashed = weapon_equip_transfer::unequipEquippedIndex(index, formID, instance);
        if (stashed) {
            native_equipped_model_lifetime::noteEquipmentChange();
            if (index == 1) {
                _secondaryDynamicWeaponCollision.retireAll(frame.bhkWorld);
                _secondaryWeaponCollision.destroyWeaponBody(frame.hknpWorld);
            }
        }
        ROCK_LOG_INFO(Weapon,
            "Equipped weapon shoulder stash index={} formID={:08X} instance={:#x} hand={} zone={} confidence={:.2f} "
            "stashed={}",
            index, formID, instance, left ? "left" : "right", body_zone::bodyZoneName(stashDecision.zone),
            stashDecision.confidence, stashed);
        if (!stashed)
            return false;
        if (g_rockConfig.rockShoulderStashHapticsEnabled) {
            (void)_feedbackHaptics.queue(left ? feedback_haptics::FeedbackHand::Left
                                              : feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockShoulderStashCommitHapticDurationSeconds,
                g_rockConfig.rockShoulderStashCommitHapticIntensity);
        }
        if (g_rockConfig.rockShoulderStashShowCollectedNotifications) {
            f4vr::showNotification(shoulder_stash_notification_policy::formatStashedNotification(itemName, formID));
        }
        return true;
    }

    void PhysicsInteraction::queueShoulderStashCandidateHaptic(const bool isLeft,
        const shoulder_stash::Decision& decision, shoulder_stash::RuntimeState& detectorState)
    {
        if (!decision.candidate || !g_rockConfig.rockShoulderStashHapticsEnabled) {
            return;
        }
        const bool pulseDue = _contacts.hapticClockSeconds >= detectorState.nextCandidatePulseTimeSeconds;
        if (!decision.enteredCandidate && !decision.changedCandidate && !pulseDue) {
            return;
        }
        (void)_feedbackHaptics.queue(isLeft ? feedback_haptics::FeedbackHand::Left
                                            : feedback_haptics::FeedbackHand::Right,
            g_rockConfig.rockShoulderStashCandidateHapticDurationSeconds,
            shoulder_stash_haptic_policy::computeCandidatePulseIntensity(decision.confidence,
                shoulder_stash_haptic_policy::CandidatePulseConfig{
                    .enabled = true,
                    .baseIntensity = g_rockConfig.rockShoulderStashCandidateHapticBaseIntensity,
                    .maxIntensity = g_rockConfig.rockShoulderStashCandidateHapticIntensity,
                }));
        detectorState.nextCandidatePulseTimeSeconds = _contacts.hapticClockSeconds +
            (std::max)(0.02f, g_rockConfig.rockShoulderStashCandidateHapticIntervalSeconds);
    }

    void PhysicsInteraction::updateSecondWeaponShoulderStash(const PhysicsFrameContext& frame, const bool left,
        const bool handlingEnabled)
    {
        auto& detectorState = _equipped.secondWeaponStashStates[left ? 1u : 0u];
        auto& lastDecision = _equipped.secondWeaponStashDecisions[left ? 1u : 0u];
        const HandFrameInput& handInput = left ? frame.left : frame.right;
        const bool eligible =
            handlingEnabled && !frame.menuBlocked && !handInput.disabled && !storage_interaction::blocksShoulder(left);
        if (!eligible) {
            shoulder_stash::resetRuntime(detectorState);
            lastDecision = {};
            return;
        }
        const auto decision = shoulder_stash::evaluate(
            shoulder_stash::DetectorInput{
                .isLeftHand = left,
                .probe = shoulder_stash::Probe{ .pointGame = handInput.grabAnchorWorld },
                .hmdProbe = makeShoulderStashHmdProbe(handInput),
                .hasHmdProbe = true,
                .hasHmdFrame = frame.hasHmdFrame,
                .hmdPositionWorld = frame.hmdPositionWorld,
                .hmdForwardWorld = frame.hmdForwardWorld,
                .deltaSeconds = frame.deltaSeconds,
                .config = makeEquippedWeaponStashDetectorConfig(handlingEnabled),
            },
            detectorState);
        lastDecision = decision;
        queueShoulderStashCandidateHaptic(left, decision, detectorState);
    }

    bool PhysicsInteraction::trySecondWeaponShoulderStash(const PhysicsFrameContext& frame, const bool left)
    {
        auto& detectorState = _equipped.secondWeaponStashStates[left ? 1u : 0u];
        const auto decision = _equipped.secondWeaponStashDecisions[left ? 1u : 0u];
        // Tap modes release on the press, so the hand only needs to be in the
        // volume; hold mode releases on button release after dwelling there.
        const bool holdMode =
            _equipped.handlingSettings.weaponGrabMode == equipped_weapon_toggle_grab_policy::Mode::HoldBoth;
        if (!(holdMode ? decision.confirmedForCommit : decision.candidate))
            return false;
        const bool stashed = stashEquippedWeaponAtShoulder(frame, 1, left, decision);
        shoulder_stash::resetRuntime(detectorState);
        _equipped.secondWeaponStashDecisions[left ? 1u : 0u] = {};
        return stashed;
    }

    PhysicsInteraction::EquippedWeaponShoulderFrameResult PhysicsInteraction::advanceEquippedWeaponShoulderCoordinator(
        const PhysicsFrameContext& frame, const bool handlingEnabled, const bool menuInputActive,
        RE::NiNode* weaponNode, const std::uint64_t currentEquippedWeaponOwnershipKey, const bool firingHandIsLeft)
    {
        EquippedWeaponShoulderFrameResult result{};
        auto* player = f4vr::getPlayer();
        const std::uint32_t nativeWeaponState = f4vr::getNativeWeaponState(player);
        const auto nativePresentation = shoulderNativePresentation(nativeWeaponState);

        // The second weapon's hand stashes on its own release (GrabInput);
        // its detector runs here with the same volume and haptics.
        const auto roster = equippedWeaponRoster();
        for (const bool isLeft : { false, true }) {
            if (roster.slotOneHolds(isLeft)) {
                updateSecondWeaponShoulderStash(frame, isLeft, handlingEnabled && !menuInputActive);
            } else {
                shoulder_stash::resetRuntime(_equipped.secondWeaponStashStates[isLeft ? 1u : 0u]);
                _equipped.secondWeaponStashDecisions[isLeft ? 1u : 0u] = {};
            }
        }

        auto leftPhysicalGrip = peekGrabButtonState(true, input_remap_policy::kGrabButtonId);
        auto rightPhysicalGrip = peekGrabButtonState(false, input_remap_policy::kGrabButtonId);
        leftPhysicalGrip.held = input_remap_runtime::isRawButtonPhysicallyHeld(true, input_remap_policy::kGrabButtonId);
        rightPhysicalGrip.held =
            input_remap_runtime::isRawButtonPhysicallyHeld(false, input_remap_policy::kGrabButtonId);
        const auto providerGripOccupancy = roster.slotZeroGrips();
        maskProviderWeaponGrabInput(true, providerGripOccupancy.left.weaponEngaged(), leftPhysicalGrip);
        maskProviderWeaponGrabInput(false, providerGripOccupancy.right.weaponEngaged(), rightPhysicalGrip);
        const auto stashModeForHand = [&](bool isLeft) {
            return equipped_weapon_shoulder::resolveStashInputMode(
                _equipped.handlingSettings.immersiveWeapon.firingGripDetachEnabled,
                equipped_weapon_toggle_grab_policy::usesToggleForRole(_equipped.handlingSettings.weaponGrabMode,
                    isLeft == firingHandIsLeft));
        };
        equipped_weapon_shoulder::FrameInput coordinatorInput{
            .enabled = handlingEnabled,
            .inputAllowed = !menuInputActive,
            .weaponOwnershipKey = currentEquippedWeaponOwnershipKey,
            .presentation = nativePresentation,
            .right = {
                .stashInputMode = stashModeForHand(false),
                .button = {
                    .held = rightPhysicalGrip.held,
                    .pressed = rightPhysicalGrip.pressed,
                    .released = rightPhysicalGrip.released,
                },
            },
            .left = {
                .stashInputMode = stashModeForHand(true),
                .button = {
                    .held = leftPhysicalGrip.held,
                    .pressed = leftPhysicalGrip.pressed,
                    .released = leftPhysicalGrip.released,
                },
            },
        };

        std::array<shoulder_stash::Decision, 2> detectorDecisions{};
        const auto detectorConfig = makeEquippedWeaponStashDetectorConfig(handlingEnabled);
        const auto manualStashCarryHand =
            equipped_weapon_drop_policy::resolveEquippedWeaponStashCarryHand(_twoHandedGrip.isPrimaryOnlyActive(),
                _twoHandedGrip.isPartCarryActive(), _twoHandedGrip.isHandPartCarryGripping(true),
                _twoHandedGrip.isHandPartCarryGripping(false), _twoHandedGrip.isFiringHandLeft());
        const auto attachedShoulderGestureHand = firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left
                                                                  : equipped_weapon_drop_policy::SourceHand::Right;
        const bool manualCarryActive = _twoHandedGrip.isPrimaryOnlyActive() || _twoHandedGrip.isPartCarryActive();
        const auto stashCarryHand = manualStashCarryHand != equipped_weapon_drop_policy::SourceHand::None
            ? manualStashCarryHand
            : handlingEnabled && !manualCarryActive ? attachedShoulderGestureHand
                                                    : equipped_weapon_drop_policy::SourceHand::None;

        for (const bool isLeft : { false, true }) {
            const std::size_t handIndex = isLeft ? 1u : 0u;
            const bool selectedCarryHand = stashCarryHand != equipped_weapon_drop_policy::SourceHand::None &&
                equipped_weapon_drop_policy::isLeft(stashCarryHand) == isLeft;
            auto& detectorState = _equipped.stashStates[handIndex];
            auto& policyHand = isLeft ? coordinatorInput.left : coordinatorInput.right;
            policyHand.carriesWeapon = selectedCarryHand;
            if (!selectedCarryHand || menuInputActive || storage_interaction::blocksShoulder(isLeft)) {
                shoulder_stash::resetRuntime(detectorState);
                continue;
            }

            Hand& hand = isLeft ? _leftHand : _rightHand;
            const HandFrameInput& handInput = isLeft ? frame.left : frame.right;
            const bool handEmpty = !hand.isHolding() && !_touchGrabRuntime.isHandActive(isLeft) &&
                !_forceGrab.pendingCommits[handIndex].active && !hand.hasActivePullCatchIntent() &&
                !hand.hasPendingExtractedItemHandoff();
            policyHand.disabled = handInput.disabled;
            policyHand.eligible = handlingEnabled && !handInput.disabled && handEmpty && weaponNode != nullptr &&
                currentEquippedWeaponOwnershipKey != 0;
            if (!policyHand.eligible) {
                shoulder_stash::resetRuntime(detectorState);
                continue;
            }

            const auto decision = shoulder_stash::evaluate(
                shoulder_stash::DetectorInput{
                    .isLeftHand = isLeft,
                    .probe =
                        shoulder_stash::Probe{
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
            policyHand.detector = {
                .candidate = decision.candidate,
                .confirmed = decision.confirmedForCommit,
                .zone = decision.zone,
                .confidence = decision.confidence,
            };
            queueShoulderStashCandidateHaptic(isLeft, decision, detectorState);
        }

        const auto previousPhase = _equipped.shoulderCoordinator.phase;
        result.decision = equipped_weapon_shoulder::advance(_equipped.shoulderCoordinator, coordinatorInput);
        _equipped.shoulderGestureConsumedThisFrame[0] = result.decision.consumeRightInput;
        _equipped.shoulderGestureConsumedThisFrame[1] = result.decision.consumeLeftInput;

        if (result.decision.action != equipped_weapon_shoulder::Action::None) {
            const auto& selectedDetector =
                detectorDecisions[result.decision.hand == equipped_weapon_shoulder::Hand::Left ? 1u : 0u];
            ROCK_LOG_INFO(Weapon,
                "Equipped shoulder coordinator action={} reason={} phase={}->{} hand={} gesture={} zone={} source={} "
                "confidence={:.2f} speed={:.1f} nativeState={}({}) stashInput={} immersive={} grabMode={}",
                equipped_weapon_shoulder::actionName(result.decision.action),
                equipped_weapon_shoulder::reasonName(result.decision.reason),
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(_equipped.shoulderCoordinator.phase),
                equipped_weapon_shoulder::handName(result.decision.hand), result.decision.gestureSerial,
                body_zone::bodyZoneName(result.decision.zone),
                shoulder_stash::evidenceSourceName(selectedDetector.source), result.decision.confidence,
                selectedDetector.speedGameUnitsPerSecond, nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(nativeWeaponState),
                equipped_weapon_shoulder::stashInputModeName(
                    result.decision.hand == equipped_weapon_shoulder::Hand::Left
                        ? coordinatorInput.left.stashInputMode
                        : coordinatorInput.right.stashInputMode),
                _equipped.handlingSettings.immersiveWeapon.firingGripDetachEnabled ? "yes" : "no",
                equipped_weapon_toggle_grab_policy::modeName(_equipped.handlingSettings.weaponGrabMode));
        } else if (previousPhase != _equipped.shoulderCoordinator.phase) {
            ROCK_LOG_DEBUG(Weapon,
                "Equipped shoulder coordinator phase={}->{} reason={} gestureHand={} gesture={} nativeState={}({})",
                equipped_weapon_shoulder::phaseName(previousPhase),
                equipped_weapon_shoulder::phaseName(_equipped.shoulderCoordinator.phase),
                equipped_weapon_shoulder::reasonName(result.decision.reason),
                equipped_weapon_shoulder::handName(_equipped.shoulderCoordinator.actionHand),
                _equipped.shoulderCoordinator.actionGestureSerial, nativeWeaponState,
                held_weapon_equip_state_policy::nativeWeaponStateName(nativeWeaponState));
        }

        result.sourceHand = result.decision.hand == equipped_weapon_shoulder::Hand::Left
            ? equipped_weapon_drop_policy::SourceHand::Left
            : result.decision.hand == equipped_weapon_shoulder::Hand::Right
            ? equipped_weapon_drop_policy::SourceHand::Right
            : equipped_weapon_drop_policy::SourceHand::None;
        if (result.decision.hand != equipped_weapon_shoulder::Hand::None) {
            result.detectorDecision =
                detectorDecisions[equipped_weapon_shoulder::isLeft(result.decision.hand) ? 1u : 0u];
        }
        return result;
    }
}

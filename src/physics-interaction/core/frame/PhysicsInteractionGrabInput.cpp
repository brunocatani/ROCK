/*
 * GRAB INPUT: turns one frame of controller state into grab and release decisions.
 *
 * updateGrabInput builds one GrabInputFrame for the frame, then calls
 * processGrabInputHand once per hand. processGrabInputHand is a member, not a
 * lambda, so the per-hand flow is readable and greppable on its own.
 *
 * Also owns the two context factories, makeGrabReleaseContext and
 * makeGrabSharedObjectContext, that the force-grab flow declares and reuses.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"

#include "api/ProviderColliderVisualizationRuntime.h"
#include "api/ProviderDebugOverlayRuntime.h"
#include "api/ROCKProviderApiInternal.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <numbers>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/api/InteractionCommandQueue.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/collision/ContactPipelinePolicy.h"
#include "physics-interaction/collision/ContactSignalSubscriptionPolicy.h"
#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/consume/MouthConsumePolicy.h"
#include "physics-interaction/consume/MouthConsumeTransfer.h"
#include "physics-interaction/feedback/FeedbackHaptics.h"
#include "physics-interaction/hand/skeleton/HandSkeleton.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/debug/overlay/DebugBodyOverlay.h"
#include "physics-interaction/debug/overlay/DebugOverlayPolicy.h"
#include "physics-interaction/feedback/HapticPolicy.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/CustomOGA.h"
#include "physics-interaction/grab/GrabEvent.h"
#include "physics-interaction/grab/GrabTelemetry.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/HeldMassMovement.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/native/havok/HavokRuntime.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/native/HeldWeaponInstantTransition.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/input/GrabInputIntentPolicy.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/stash/ShoulderStashDetector.h"
#include "physics-interaction/stash/ShoulderStashPolicy.h"
#include "physics-interaction/stash/ShoulderStashTransfer.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/collision/DynamicWeaponCollisionPolicy.h"
#include "physics-interaction/weapon/equip/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/native_anim/NativeScopeSightAnchorPolicy.h"
#include "physics-interaction/weapon/native_anim/NativeIdleGripPreharvest.h"
#include "physics-interaction/weapon/native_anim/NativeEquippedWeaponDraw.h"
#include "physics-interaction/weapon/equip/WeaponTransitionAnimationAcceleration.h"
#include "physics-interaction/weapon/equip/PipboyEquipRuntime.h"
#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/weapon/equip/WeaponEquipTransfer.h"
#include "physics-interaction/weapon/WeaponInteraction.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsRayCast.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/debug/PhysicsWorldOriginDiagnostics.h"
#include "physics-interaction/collision/PushAssist.h"
#include "physics-interaction/hand/selection/HandSelection.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"

#include "RE/Bethesda/ActorValueInfo.h"
#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/Events.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/UI.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"

#include "ROCKMain.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrActorStatePolicy.h"
#include "rock_support/Fo4VrRuntime.h"
#include "rock_support/VRControllers.h"
#include <windows.h>

namespace rock
{
    using namespace physics_interaction_detail;

    void PhysicsInteraction::updateSelection(const PhysicsFrameContext& frame)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            _rightHand.stopSelectionBeam();
            _leftHand.stopSelectionBeam();
            return;
        }

        const auto rightPendingTargetPtr = _pendingForceGrabCommits[0].targetHandle.get();
        const auto leftPendingTargetPtr = _pendingForceGrabCommits[1].targetHandle.get();

        auto selectionContextForOtherHand = [](const Hand& hand, RE::TESObjectREFR* pendingTarget) {
            OtherHandSelectionContext context{};
            if (pendingTarget) {
                context.exclusiveRef = pendingTarget;
                return context;
            }
            if (hand.isHolding() && hand.getHeldRef()) {
                context.shareableHeldRef = hand.getHeldRef();
                return context;
            }
            if (hand.hasActivePullCatchIntent()) {
                context.exclusiveRef = hand.getPullCatchIntentRef();
                return context;
            }
            if (hand.hasSelection() && selection_state_policy::hasExclusiveObjectSelection(hand.getState())) {
                context.exclusiveRef = hand.getSelection().refr;
            }
            return context;
        };

        const auto rightHandContext = selectionContextForOtherHand(
            _rightHand,
            _pendingForceGrabCommits[0].active ? rightPendingTargetPtr.get() : nullptr);
        const auto leftHandContext = selectionContextForOtherHand(
            _leftHand,
            _pendingForceGrabCommits[1].active ? leftPendingTargetPtr.get() : nullptr);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);

        if (_pendingForceGrabCommits[0].active) {
            if (_rightHand.hasSelection()) {
                _rightHand.clearSelectionState(false);
            }
            _rightHand.stopSelectionBeam();
        } else if (!frame.right.disabled) {
            _rightHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.right.grabAnchorWorld,
                frame.right.closeSelectionDirectionWorld,
                frame.right.farSelectionDirectionWorld,
                frame.right.pinchPocketWorld,
                frame.right.pinchDirectionWorld,
                frame.right.hasPinchPocketWorld,
                farHmdConeGate,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                leftHandContext);
            _rightHand.updateSelectionBeam(frame.hknpWorld, frame.right.grabAnchorWorld);
        } else {
            _rightHand.stopSelectionBeam();
        }

        if (_pendingForceGrabCommits[1].active) {
            if (_leftHand.hasSelection()) {
                _leftHand.clearSelectionState(false);
            }
            _leftHand.stopSelectionBeam();
        } else if (!frame.left.disabled) {
            _leftHand.updateSelection(frame.bhkWorld,
                frame.hknpWorld,
                frame.left.grabAnchorWorld,
                frame.left.closeSelectionDirectionWorld,
                frame.left.farSelectionDirectionWorld,
                frame.left.pinchPocketWorld,
                frame.left.pinchDirectionWorld,
                frame.left.hasPinchPocketWorld,
                farHmdConeGate,
                g_rockConfig.rockNearDetectionRange,
                g_rockConfig.rockFarDetectionRange,
                frame.deltaSeconds,
                rightHandContext);
            _leftHand.updateSelectionBeam(frame.hknpWorld, frame.left.grabAnchorWorld);
        } else {
            _leftHand.stopSelectionBeam();
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

    void PhysicsInteraction::updateGrabInput(const PhysicsFrameContext& frame)
    {
        _forceGrabCommittedThisFrame = {};

        auto clearShoulderStashForHand = [&](Hand& hand, bool isLeft) {
            shoulder_stash::resetRuntime(_shoulderStashStates[isLeft ? 1u : 0u]);
            hand.cancelStashCandidate();
        };

        auto clearMouthConsumeForHand = [&](Hand& hand, bool isLeft) {
            mouth_consume::resetRuntime(_mouthConsumeStates[isLeft ? 1u : 0u]);
            hand.cancelConsumeCandidate();
        };

        auto clearGameplayCandidatesForHand = [&](Hand& hand, bool isLeft) {
            clearShoulderStashForHand(hand, isLeft);
            clearMouthConsumeForHand(hand, isLeft);
        };

        if (!runtime_state::isLocalSkeletonReady()) {
            _touchGrabRuntime.releaseAll(
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                _collisionGenerationAtomic.load(
                    std::memory_order_acquire));
            _rightHand.cancelGrabVisualReturn("skeleton-not-ready");
            _leftHand.cancelGrabVisualReturn("skeleton-not-ready");
            provider::clearInteractionCommandsForProviderLossV1(provider::RockProviderInteractionFailureV1::ProviderNotReady);
            clearPendingForceGrabCommitsForOrigin(PendingForceGrabCommitOrigin::ProviderForceGrabCommand);
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
            _heldWeaponTriggerEquipIntents = {};
            clearGameplayCandidatesForHand(_rightHand, false);
            clearGameplayCandidatesForHand(_leftHand, true);
            return;
        }

        const auto worldGeneration =
            _worldGenerationAtomic.load(
                std::memory_order_acquire);
        const auto skeletonGeneration =
            _skeletonGenerationAtomic.load(
                std::memory_order_acquire);
        const auto providerGeneration =
            _providerGenerationAtomic.load(
                std::memory_order_acquire);
        const auto collisionGeneration =
            _collisionGenerationAtomic.load(
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
            _touchGrabRuntime.releaseAll(
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                collisionGeneration);
        }
        int grabButton = g_rockConfig.rockGrabButtonID;
        const bool rightHandWeaponEquipped = resolveEquippedWeaponInteractionNode() != nullptr;
        const bool ambidextrousHandoffAvailable =
            _equippedWeaponHandlingSettings.ambidextrousHandoffEnabled &&
            TwoHandedGrip::canBeginPrimaryOnlyGripForHand(true);
        const equipped_weapon_manual_ownership_policy::FiringGripModeAvailability firingGripModes{
            .primaryDetachEnabled = _equippedWeaponHandlingSettings.primaryDetachEnabled,
            .ambidextrousHandoffAvailable = ambidextrousHandoffAvailable,
        };
        const bool gripZoneSettleEquipEnabled =
            equipped_weapon_manual_ownership_policy::canSettleEquipInGripZone(
                _equippedWeaponHandlingSettings.gripZoneEquipEnabled);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);
        auto publishHandInputOwnership = [&](const Hand& hand, const bool isLeft) {
            auto* heldRef = hand.isHolding() ? hand.getHeldRef() : nullptr;
            const bool pendingEquippedGripOwnership =
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending &&
                _pendingEquippedWeaponPrimaryOnlyGripStart.isLeft == isLeft;
            input_remap_runtime::setHandHeldWeapon(isLeft, hand.isHoldingLooseWeapon());
            // Engaged = holding a ROCK object or gripping the equipped weapon (support/two-hand, part carry while primary detached, attach-only glue).
            input_remap_runtime::setHandInteractionEngaged(
                isLeft,
                hand.isHolding() ||
                    _touchGrabRuntime.isHandActive(isLeft) ||
                    _twoHandedGrip.isHandPartGripping(isLeft) ||
                    pendingEquippedGripOwnership);
            input_remap_runtime::setHeldObjectFormId(isLeft, heldRef ? heldRef->GetFormID() : 0u);
        };
        publishHandInputOwnership(_rightHand, false);
        publishHandInputOwnership(_leftHand, true);
        processProviderInteractionCommands(frame);
        serviceLooseGrenadeQuickDraw(frame);
        servicePendingForceGrabCommits(frame);
        updateSavedGrabOffsetGesture(frame);
        serviceEquippedWeaponDropMomentumHandoff(frame);
        updateLooseGrenadeFuses(frame);
        publishHandInputOwnership(_rightHand, false);
        publishHandInputOwnership(_leftHand, true);

        const GrabInputFrame input{
            .frame = &frame,
            .farHmdConeGate = farHmdConeGate,
            .worldGeneration = worldGeneration,
            .skeletonGeneration = skeletonGeneration,
            .providerGeneration = providerGeneration,
            .collisionGeneration = collisionGeneration,
            .grabButton = grabButton,
            .rightHandWeaponEquipped = rightHandWeaponEquipped,
            .ambidextrousHandoffAvailable =
                ambidextrousHandoffAvailable,
            .primaryDetachEnabled =
                firingGripModes.primaryDetachEnabled,
            .gripZoneSettleEquipEnabled =
                gripZoneSettleEquipEnabled,
        };

        processGrabInputHand(_rightHand, false, input);
        publishHandInputOwnership(_rightHand, false);
        processGrabInputHand(_leftHand, true, input);
        publishHandInputOwnership(_leftHand, true);

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
            _rightHand.updateGrabVisualReturn(
                frame.right.rawHandWorld,
                frame.deltaSeconds,
                _currentPreFrikSchedulerSequence);
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
            _leftHand.updateGrabVisualReturn(
                frame.left.rawHandWorld,
                frame.deltaSeconds,
                _currentPreFrikSchedulerSequence);
        }
    }

    void PhysicsInteraction::processGrabInputHand(
        Hand& hand,
        bool isLeft,
        const GrabInputFrame& input)
    {
        const auto& frame = *input.frame;
        auto* hknp = frame.hknpWorld;
        const auto worldGeneration = input.worldGeneration;
        const auto skeletonGeneration = input.skeletonGeneration;
        const auto providerGeneration = input.providerGeneration;
        const auto collisionGeneration = input.collisionGeneration;
        const int grabButton = input.grabButton;
        const bool rightHandWeaponEquipped =
            input.rightHandWeaponEquipped;
        const bool ambidextrousHandoffAvailable =
            input.ambidextrousHandoffAvailable;
        const bool gripZoneSettleEquipEnabled =
            input.gripZoneSettleEquipEnabled;
        const auto& farHmdConeGate = input.farHmdConeGate;
        const equipped_weapon_manual_ownership_policy::
            FiringGripModeAvailability firingGripModes{
                .primaryDetachEnabled =
                    input.primaryDetachEnabled,
                .ambidextrousHandoffAvailable =
                    ambidextrousHandoffAvailable,
            };

        const auto clearShoulderStashForHand =
            [&](Hand& targetHand, bool targetIsLeft) {
                shoulder_stash::resetRuntime(
                    _shoulderStashStates[targetIsLeft ? 1u : 0u]);
                targetHand.cancelStashCandidate();
            };
        const auto clearMouthConsumeForHand =
            [&](Hand& targetHand, bool targetIsLeft) {
                mouth_consume::resetRuntime(
                    _mouthConsumeStates[targetIsLeft ? 1u : 0u]);
                targetHand.cancelConsumeCandidate();
            };
        const auto clearGameplayCandidatesForHand =
            [&](Hand& targetHand, bool targetIsLeft) {
                clearShoulderStashForHand(
                    targetHand,
                    targetIsLeft);
                clearMouthConsumeForHand(
                    targetHand,
                    targetIsLeft);
            };
        const auto releaseSuppressedHeldObject =
            [&](Hand& targetHand,
                bool targetIsLeft,
                const char* reason) {
                auto* heldRef = targetHand.getHeldRef();
                const auto heldFormID =
                    heldRef ? heldRef->GetFormID() : 0u;
                targetHand.releaseGrabbedObject(
                    hknp,
                    GrabReleaseCollisionRestoreMode::Delayed,
                    makeGrabReleaseContext(
                        targetHand,
                        targetIsLeft));
                if (heldRef) {
                    releaseObject(
                        heldRef,
                        claimOwnerForHand(targetIsLeft));
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: released held object because normal grab input is suppressed ({})",
                    targetHand.handName(),
                    reason ? reason : "unknown");
                dispatchPhysicsMessage(
                    kPhysMsg_OnRelease,
                    targetIsLeft,
                    heldRef,
                    heldFormID,
                    0);
                dispatchSimpleGrabEvent(
                    GrabEventType::Released,
                    targetIsLeft,
                    heldRef);
            };

        const auto& handInput = isLeft ? frame.left : frame.right;
        auto& inputIntentState = _grabInputIntentStates[isLeft ? 1u : 0u];
        auto& peerHeldJoinRetryState = _peerHeldJoinRetryStates[isLeft ? 1u : 0u];
        auto& triggerEquipIntent = _heldWeaponTriggerEquipIntents[isLeft ? 1u : 0u];
        auto& shoulderStashState = _shoulderStashStates[isLeft ? 1u : 0u];
        auto& mouthConsumeState = _mouthConsumeStates[isLeft ? 1u : 0u];
        auto& inputSuppressionState = _providerHandInputSuppressionStates[isLeft ? 1u : 0u];
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
        if (_equippedWeaponSheathCommittedThisFrame[handIndex] ||
            _equippedWeaponUnsheathCommittedThisFrame[handIndex]) {
            // Consume the shoulder gesture once. The equipped-weapon path
            // already used this edge to sheath, or used the squeeze to
            // draw and claim this hand.
            if (_firingHandGrabButtonFrameState.valid &&
                _firingHandGrabButtonFrameState.isLeft == isLeft) {
                _firingHandGrabButtonFrameState.valid = false;
            } else {
                static_cast<void>(
                    readGrabButtonState(isLeft, grabButton));
            }
            inputSuppressionState.deferredGrabRelease = false;
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry(
                "equipped-weapon-shoulder-gesture-this-frame",
                true);
            clearGameplayCandidatesForHand(hand, isLeft);
            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            return;
        }
        if (_forceGrabCommittedThisFrame[handIndex]) {
            /*
             * Consume, but do not apply, the physical button edges from
             * before this programmatic attachment. Otherwise a stale
             * release from the Pip-Boy/API frame can drop the object in
             * the same update that reported a successful force-grab.
             */
            if (_firingHandGrabButtonFrameState.valid && _firingHandGrabButtonFrameState.isLeft == isLeft) {
                _firingHandGrabButtonFrameState.valid = false;
            } else {
                static_cast<void>(readGrabButtonState(isLeft, grabButton));
            }
            inputSuppressionState.deferredGrabRelease = false;
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("force-grab-committed-this-frame", true);
            clearGameplayCandidatesForHand(hand, isLeft);
            return;
        }
        if (_pendingForceGrabCommits[handIndex].active) {
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("pending-force-grab-reservation", true);
            clearGameplayCandidatesForHand(hand, isLeft);
            if (hand.hasSelection()) {
                hand.clearSelectionState(false);
            }
            return;
        }

        const auto& peerCommit = _pendingForceGrabCommits[isLeft ? 0u : 1u];
        if (peerCommit.active && hand.hasSelection()) {
            const auto peerTargetPtr = peerCommit.targetHandle.get();
            if (peerTargetPtr && hand.getSelection().refr == peerTargetPtr.get()) {
                hand.clearSelectionState(false);
            }
        }
        if (handInput.disabled) {
            _touchGrabRuntime.releaseHand(
                isLeft,
                frame.bhkWorld,
                frame.hknpWorld,
                provider::RockProviderTouchGrabReleaseReasonV1::
                    HandUnavailable,
                collisionGeneration);
            cancelPeerHeldJoinRetry("hand-input-disabled", false);
            clearGameplayCandidatesForHand(hand, isLeft);
            return;
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
            grab_input_intent_policy::reset(inputIntentState);
            cancelPeerHeldJoinRetry("provider-hand-input-suppressed", true);
            if (providerHoldsCurrentGrabState &&
                !readGrabButtonHeld(isLeft, grabButton)) {
                inputSuppressionState.deferredGrabRelease = true;
            }
            return;
        }

        const bool heldWeaponEquipTriggerPressedEdge =
            !providerSuppressesHeldWeaponTriggerEquip && readHeldWeaponEquipTriggerPressedEdge(isLeft);
        const bool handIsFiringHand = isLeft == _twoHandedGrip.isFiringHandLeft();
        if (!weapon_two_handed_grip_math::canProcessNormalGrabInput(
                handIsFiringHand,
                rightHandWeaponEquipped,
                _twoHandedGrip.isHandPartGripping(isLeft),
                _twoHandedGrip.isPartCarryActive() && !_twoHandedGrip.isHandPartGripping(isLeft))) {
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
                releaseSuppressedHeldObject(hand, isLeft, handIsFiringHand ? "firing-hand weapon equipped" : "equipped weapon support grip active");
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
            return;
        }

        /*
         * The equipped-weapon manual ownership path consumes the firing
         * hand's grab edges earlier this frame. Reuse that single consumed
         * snapshot for the same physical hand; re-reading would see
         * cleared edges and starve free-hand world grabs of press/release
         * input.
         */
        GrabButtonState grabInput{};
        if (_firingHandGrabButtonFrameState.valid && _firingHandGrabButtonFrameState.isLeft == isLeft) {
            grabInput = GrabButtonState{
                .held = _firingHandGrabButtonFrameState.held,
                .pressed = _firingHandGrabButtonFrameState.pressed,
                .released = _firingHandGrabButtonFrameState.released,
            };
            _firingHandGrabButtonFrameState.valid = false;
        } else {
            grabInput = readGrabButtonState(isLeft, grabButton);
        }
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

        /*
         * Provider-registered touch targets consume the same physical
         * grip edge as ordinary grabs, but live in a separate runtime so
         * ROCK's loose-object selection policy continues to reject static
         * and keyframed bodies. An explicit body registration always wins
         * over a wildcard fixed-surface registration when one press has
         * contact evidence for both.
         */
        if (_touchGrabRuntime.isHandActive(isLeft)) {
            if (rawGrabInput.released ||
                !rawGrabInput.held) {
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
            return;
        }

        const auto handState = hand.getState();
        const bool touchGrabStateAvailable =
            handState == HandState::Idle ||
            handState == HandState::SelectedClose ||
            handState == HandState::SelectedFar;
        const bool touchGrabPhysicsWritesAllowed =
            physicsWritesAllowedForWorld(frame.hknpWorld);
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
            !_pendingForceGrabCommits[handIndex].active &&
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
                _pendingForceGrabCommits[handIndex].active,
                touchGrabPhysicsWritesAllowed);
        }
        if (canTryTouchGrab) {
            _touchGrabRuntime.beginAttemptDiagnostics();
            constexpr std::uint32_t
                kTouchGrabContactFreshnessFrames = 4;
            const auto contacts =
                hand.collectFreshSemanticContacts(
                    kTouchGrabContactFreshnessFrames);
            const auto surfaceContacts =
                _dynamicHandCollision.collectFreshSurfaceContacts(
                    isLeft,
                    kTouchGrabContactFreshnessFrames);
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
                                        source)) {
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
                tryTargetClass(
                    TouchGrabRuntime::TargetClass::
                        Wildcard);
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
                return;
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

        if (triggerEquipIntent.pending) {
            triggerEquipIntent.remainingSeconds -= (std::max)(0.0f, frame.deltaSeconds);
            if (triggerEquipIntent.remainingSeconds <= 0.0f) {
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
            if (g_rockConfig.rockPullCatchWideReacquireEnabled &&
                hand.reacquirePullCatchCloseSelection(frame.bhkWorld,
                    frame.hknpWorld,
                    handInput.grabAnchorWorld,
                    handInput.closeSelectionDirectionWorld,
                    g_rockConfig.rockPullCatchWideReacquireRadiusGameUnits,
                    g_rockConfig.rockPullCatchWideReacquireMaxBodyDistanceGameUnits)) {
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
            });
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
                g_rockConfig.rockNearDetectionRange,
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
            bool grabbed = hand.grabSelectedObject(hknp,
                transform,
                g_rockConfig.rockGrabLinearTau,
                g_rockConfig.rockGrabLinearDamping,
                g_rockConfig.rockGrabConstraintMaxForce,
                g_rockConfig.rockGrabLinearProportionalRecovery,
                g_rockConfig.rockGrabLinearConstantRecovery,
                &_bodyBoneColliders,
                sharedContext);

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
            [&](GrabEventType type, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t primaryBodyId) {
                GrabEventData eventData{};
                eventData.type = type;
                eventData.sourceKind = GrabEventSourceKind::HeldObject;
                eventData.isLeft = isLeft;
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
        native_idle_grip_preharvest::observeCandidate(std::move(nativeIdleGripCandidate));

        loose_weapon_grip_zone::updateHeldLooseWeapon(
            isLeft,
            gripZoneSettleEquipEnabled && hand.isHoldingLooseWeapon(),
            hand.getHeldRef(),
            hand.getState() == HandState::HeldBody,
            frame.deltaSeconds,
            _equippedWeaponHandlingSettings.gripZoneEquipRadiusGameUnits);

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
        if (_equippedWeaponHandlingSettings.gripZoneHoverHapticsEnabled &&
            gripZoneSettleEquipEnabled &&
            g_rockConfig.rockInputRemapEnabled &&
            !hand.isHolding() &&
            hand.hasSelection() &&
            !input_remap_runtime::isMenuInputActive()) {
            auto* selectionRef = hand.getSelection().refr;
            if (selectionRef && !loose_grenade_runtime::isGrenadeRef(selectionRef)) {
                gripZoneHoverCandidate = selectionRef;
            }
        }
        loose_weapon_grip_zone::updateHoverCandidateWeapon(
            isLeft,
            gripZoneHoverCandidate,
            _equippedWeaponHandlingSettings.gripZoneEquipRadiusGameUnits);
        if (loose_weapon_grip_zone::isGripZoneHoverInsideRadius(isLeft)) {
            (void)_feedbackHaptics.queue(
                isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                grip_zone_hover_haptic_policy::kContinuousQueueSeconds,
                _equippedWeaponHandlingSettings.gripZoneHoverHapticIntensity);
        }

        if (hand.isHolding()) {
            const Hand& peer = isLeft ? _rightHand : _leftHand;
            auto* heldRefForGameplay = hand.getHeldRef();
            const bool heldLooseGrenade = loose_grenade_runtime::isGrenadeRef(heldRefForGameplay);
            const bool peerHoldingSameObject =
                heldRefForGameplay && peer.isHolding() && peer.getHeldRef() == heldRefForGameplay;
            const bool replayedSameHandTrigger = triggerEquipIntent.pending &&
                heldRefForGameplay && triggerEquipIntent.formID == heldRefForGameplay->GetFormID();
            const bool heldWeaponEquipTriggerPressed = heldWeaponEquipTriggerPressedEdge || replayedSameHandTrigger;
            if (replayedSameHandTrigger || (heldWeaponEquipTriggerPressedEdge && hand.isHoldingLooseWeapon())) {
                triggerEquipIntent = {};
            }
            const bool heldWeaponGripZoneEquipSettled = !heldLooseGrenade &&
                loose_weapon_grip_zone::isGripZoneEquipSettled(
                    isLeft,
                    _equippedWeaponHandlingSettings.gripZoneEquipSettleSeconds);
            const bool heldWeaponEquipRequested = input_remap_policy::shouldRequestHeldWeaponEquip(input_remap_policy::HeldWeaponEquipInput{
                .remapEnabled = g_rockConfig.rockInputRemapEnabled,
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
                const auto transitionReason = triggeredByInput ?
                    held_weapon_instant_transition::RequestReason::SameHandTrigger :
                    held_weapon_instant_transition::RequestReason::GripZoneSettle;
                if (peerHoldingSameObject) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand {} held weapon equip blocked: peer hand still holding formID={:08X}",
                        hand.handName(),
                        logAction ? logAction : "requested",
                        heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u);
                    return true;
                }

                auto* player = RE::PlayerCharacter::GetSingleton();
                const std::uint32_t nativeStateBeforeEquip =
                    f4vr::getNativeWeaponState(player);
                if (!held_weapon_equip_state_policy::canBeginEquip(nativeStateBeforeEquip)) {
                    const bool triggerIntentRearmed =
                        held_weapon_equip_state_policy::shouldRearmTrigger(nativeStateBeforeEquip, triggeredByInput) &&
                        heldRefForGameplay;
                    if (triggerIntentRearmed) {
                        triggerEquipIntent = HeldWeaponTriggerEquipIntent{
                            .pending = true,
                            .formID = heldRefForGameplay->GetFormID(),
                            .remainingSeconds = 0.35f,
                        };
                    }
                    ROCK_LOG_SAMPLE_WARN(
                        Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "{} hand {} held weapon equip deferred/blocked before release formID={:08X} weaponState={}({}) triggerIntentRearmed={}",
                        hand.handName(),
                        logAction ? logAction : "requested",
                        heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0u,
                        nativeStateBeforeEquip,
                        held_weapon_equip_state_policy::nativeWeaponStateName(nativeStateBeforeEquip),
                        triggerIntentRearmed ? "yes" : "no");
                    return true;
                }

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
                    return true;
                }

                PendingEquippedWeaponPrimaryOnlyGripStart pendingGripStart{};
                pendingGripStart.pending = equipped_weapon_manual_ownership_policy::shouldStartHeldWeaponEquipOwnership(
                    equipped_weapon_manual_ownership_policy::HeldWeaponEquipOwnershipInput{
                        .modes = firingGripModes,
                        .handIsLeft = isLeft,
                        .gripHeld = rawGrabInput.held,
                    });
                pendingGripStart.isLeft = isLeft;
                const bool handCarryAvailable = !isLeft || ambidextrousHandoffAvailable;
                const bool capturedLooseHold = pendingGripStart.pending &&
                    handCarryAvailable &&
                    loose_weapon_grip_zone::tryGetFiringHandWeaponLocal(
                        isLeft,
                        pendingGripStart.firingHandWeaponLocal,
                        pendingGripStart.firingGripWeaponLocal);
                pendingGripStart.hasFiringHandWeaponLocal = capturedLooseHold;
                pendingGripStart.hasFiringGripWeaponLocal = capturedLooseHold;
                if (pendingGripStart.pending && isLeft && !capturedLooseHold) {
                    ROCK_LOG_SAMPLE_WARN(
                        Hand,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "left hand {} held weapon equip blocked: canonical weapon-relative left carry unavailable addonAuthority={} ambidextrousFiring={} grabHeld={} hFRIKBlockers={} gripFrame={}",
                        logAction ? logAction : "requested",
                        _equippedWeaponHandlingSettings.externalAuthorityActive ? "yes" : "no",
                        ambidextrousHandoffAvailable ? "yes" : "no",
                        rawGrabInput.held ? "yes" : "no",
                        handCarryAvailable ? "yes" : "no",
                        pendingGripStart.hasFiringHandWeaponLocal ? "yes" : "no");
                    return true;
                }

                hand.captureHeldReleaseMotion(hknp, handInput.rawHandWorld, frame.deltaSeconds);
                auto* heldRef = hand.getHeldRef();
                const auto previousEquippedWeaponFormID =
                    currentEquippedWeaponFormId();
                const auto previousNativeInstanceNode =
                    previousEquippedWeaponFormID != 0 ?
                    equipped_weapon_visual_state::observe(
                        previousEquippedWeaponFormID).exactInstance :
                    nullptr;
                hand.stopSelectionHighlight();
                Hand& peerHandForVisualState = isLeft ? _rightHand : _leftHand;
                if (heldRef && peerHandForVisualState.hasSelection() && peerHandForVisualState.getSelection().refr == heldRef) {
                    peerHandForVisualState.clearSelectionState(false);
                }
                std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                const std::uint32_t primaryBodyId = hand.getSavedObjectState().bodyId.value;
                auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                releaseContext.disposition = GrabReleaseDisposition::PendingInventoryTransfer;
                releaseContext.reason = requestReason ? requestReason : "held-weapon-equip";
                auto releaseOutcome = hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                if (heldRef) {
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                }

                const auto equipResult = weapon_equip_transfer::transferHeldWeaponToPlayerAndEquip(weapon_equip_transfer::EquipInput{
                    .heldRef = releaseOutcome.takeRetainedReference(),
                    .transitionReason = transitionReason,
                });
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
                    equipBridgeStarted = _equippedWeaponTransition.beginHeldTransition(
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
                        .isLeftHand = isLeft,
                        .weapon = equipResult.weapon,
                        .hasFiringHandWeaponLocal = pendingGripStart.hasFiringHandWeaponLocal,
                        .firingHandWeaponLocal = pendingGripStart.firingHandWeaponLocal,
                        .sourceSchedulerSequence =
                            _currentPreFrikSchedulerSequence,
                        .timeoutSeconds = _equippedWeaponHandlingSettings.equipVisualBridgeTimeoutSeconds,
                        .blendSeconds = _equippedWeaponHandlingSettings.equipVisualBridgeBlendSeconds,
                    });
                }
                if (heldFormID == 0 && equipResult.formID != 0) {
                    heldFormID = equipResult.formID;
                }

                auto* postEquipRef = equipResult.untransferredRef.get();
                dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, postEquipRef, heldFormID, 0);
                if (!equipResult.success && !equipResult.transferredToInventory) {
                    hand.applyReleaseVelocitySnapshot(hknp, releaseOutcome.velocity);
                }
                dispatchHeldObjectEventByFormID(GrabEventType::Released, postEquipRef, heldFormID, primaryBodyId);
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
                    _pendingEquippedWeaponPrimaryOnlyGripStart = pendingGripStart;
                }
                const auto& actionTrace = equipResult.instantTransition.actionTrace;
                ROCK_LOG_INFO(Hand,
                    "{} hand {} held weapon equip formID={:08X} success={} managerAccepted={} committed={} equippedStackMatch={} equipReason={} requestReason={} transition={} readiness={} count={} stack={} stackEvidence={} stacks={}->{} mutations={} instanceMatch={} requestedInstance={:#x} observedInstance={:#x} transferred={} observedEquipped={:08X} equipIndex={} weaponState={}({})->{}({}) traceCount={} traceSheathe={} traceDraw={} traceFaults=0x{:02X} nativeInstance={} nativeAncestorsVisible={} nativeLocalVisible={} immediateEquip={} visualBridge={} physicalCarryPending={} physicalCarryHand={}",
                    hand.handName(),
                    logAction ? logAction : "requested",
                    heldFormID,
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
                    physicalCarryPendingArmed ? (isLeft ? "left" : "right") : "none");
                input_remap_runtime::setHandHeldWeapon(isLeft, false);
                clearGameplayCandidatesForHand(hand, isLeft);
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

            const auto consumeEligibility = mouth_consume::evaluateEligibility(mouth_consume::EligibilityInput{
                .enabled = g_rockConfig.rockMouthConsumeEnabled,
                .allowPoison = g_rockConfig.rockMouthConsumeAllowPoison,
                .peerHoldingSameObject = peerHoldingSameObject,
                .heldRef = heldRefForGameplay,
                .savedState = &hand.getSavedObjectState(),
            });

            mouth_consume::Decision consumeDecision{};
            if (consumeEligibility.eligible) {
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
                    const bool pulseDue = _dynamicPushElapsedSeconds >= mouthConsumeState.nextCandidatePulseTimeSeconds;
                    if (consumeDecision.enteredCandidate || consumeDecision.changedCandidate || pulseDue) {
                        dispatchMouthConsumeEvent(
                            GrabEventType::ConsumeCandidate,
                            heldRefForGameplay,
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
                            hand.getSavedObjectState().bodyId.value,
                            consumeDecision);
                        mouthConsumeState.nextCandidatePulseTimeSeconds =
                            _dynamicPushElapsedSeconds + (std::max)(0.02f, g_rockConfig.rockMouthConsumeCandidateHapticIntervalSeconds);
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
                        .bodyContacts = &_bodyContactRuntime,
                        .heldBodyIds = &hand.getHeldBodyIds(),
                        .contactFrame = _handContactActivity.currentFrame(),
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
                    if (shouldEmitShoulderStashCandidatePulse(
                            stashDecision,
                            shoulderStashState,
                            _dynamicPushElapsedSeconds)) {
                        dispatchShoulderStashEvent(
                            GrabEventType::StashCandidate,
                            heldRefForGameplay,
                            heldRefForGameplay ? heldRefForGameplay->GetFormID() : 0,
                            hand.getSavedObjectState().bodyId.value,
                            stashDecision);
                    }
                }
            } else {
                hand.cancelStashCandidate();
            }

            if (grabInput.released) {
                hand.captureHeldReleaseMotion(hknp, handInput.rawHandWorld, frame.deltaSeconds);
                auto* heldRef = hand.getHeldRef();
                std::uint32_t heldFormID = heldRef ? heldRef->GetFormID() : 0u;
                if (consumeEligibility.eligible && consumeDecision.confirmedForCommit && hand.getState() == HandState::ConsumeCandidate) {
                    /*
                     * Mouth consume mirrors shoulder stash's two-phase release:
                     * detach the grab without throw velocity first, then let the
                     * native consume/activation path take ownership. Only failures
                     * that leave a world ref behind get the captured throw velocity.
                     */
                    auto releaseContext = makeGrabReleaseContext(hand, isLeft);
                    releaseContext.disposition = GrabReleaseDisposition::PendingConsumeTransfer;
                    releaseContext.reason = "mouth-consume-pending-transfer";
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
                        dispatchHeldObjectEventByFormID(GrabEventType::Released, postConsumeRef, heldFormID, primaryBodyId);
                    }
                    ROCK_LOG_INFO(Hand,
                        "{} hand mouth consume release formID={:08X} success={} consumeReason={} count={} confidence={:.2f} distance={:.1f} speed={:.1f}",
                        hand.handName(),
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
                        dispatchHeldObjectEventByFormID(GrabEventType::Released, postTransferRef, heldFormID, primaryBodyId);
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
                    _palmClockGameFrameIndex.load(std::memory_order_acquire),
                    _palmClockGameDeltaSeconds.load(std::memory_order_acquire),
                    nullptr);
                hand.updateHeldObject(hknp,
                    transform,
                    frame.deltaSeconds,
                    g_rockConfig.rockGrabForceFadeInTime,
                    g_rockConfig.rockGrabTauMin,
                    &_bodyBoneColliders,
                    _currentPreFrikSchedulerSequence,
                    makeGrabReleaseContext(hand, isLeft));
                if (heldRef && !hand.isHolding()) {
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

        auto actorEquipmentHandoffMaxSeconds = []() -> float {
            return (std::isfinite(g_rockConfig.rockPullCatchRetryMaxTimeSeconds) && g_rockConfig.rockPullCatchRetryMaxTimeSeconds > 0.0f) ?
                       g_rockConfig.rockPullCatchRetryMaxTimeSeconds :
                       0.65f;
        };

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
                if (!hand.advancePullCatchCommit(frame.deltaSeconds, g_rockConfig.rockPullCatchRetryMaxTimeSeconds)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand cancelled pull catch commit because retry window expired ({:.3f}s)",
                        hand.handName(),
                        g_rockConfig.rockPullCatchRetryMaxTimeSeconds);
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
                    actorEquipmentHandoffMaxSeconds());
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
                        hand.getSelection().isFarSelection, hand.getSelection().distance, g_rockConfig.rockFarDetectionRange)) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand: far grab blocked (dist={:.1f}, configuredFarRange={:.1f})",
                        hand.handName(),
                        hand.getSelection().distance,
                        g_rockConfig.rockFarDetectionRange);
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
                        actorEquipmentHandoffMaxSeconds());
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
}

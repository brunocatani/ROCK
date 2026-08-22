/*
 * The per-frame ORCHESTRATOR. This is the top of the main-thread call tree.
 *
 * Read top-down and you read one game frame in order:
 *   buildFrameContext  - sample wands, HMD, worlds, and clocks once
 *   update             - the frame table of contents; guards first, then phases
 *   the phase methods  - each phase, in the order update calls it
 *   the finalize pass  - overlay publication and post-FRIK presentation fixups
 *
 * Every phase method takes the frame context by const reference. Nothing here may
 * allocate, log per frame, or dispatch virtually: this runs every frame.
 */

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/core/FrikSkeletonProfile.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/core/PhysicsInteractionTransformValidation.h"
#include "physics-interaction/weapon/presentation/PresentationTraceRuntime.h"

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


    PhysicsFrameContext PhysicsInteraction::buildFrameContext(RE::bhkWorld* bhk, RE::hknpWorld* hknp, float deltaSeconds)
    {
        /*
         * Frame-context construction is separated from the main update loop so
         * lifecycle, collision, grab, weapon, and debug phases consume one coherent
         * snapshot of ROCK/FO4VR hand state. This keeps future frame inputs from
         * being added as scattered global reads throughout PhysicsInteraction::update().
         */
        PhysicsFrameContext frame{};
        frame.gameFrameIndex = runtime_state::currentFrame().frameIndex;
        frame.preFrikSchedulerSequence = _currentPreFrikSchedulerSequence;
        frame.bhkWorld = bhk;
        frame.hknpWorld = hknp;
        frame.deltaSeconds = (deltaSeconds > 0.0f && deltaSeconds <= 0.1f) ? deltaSeconds : (1.0f / 90.0f);
        frame.worldReady = bhk && hknp;
        frame.menuBlocked = runtime_state::isPhysicsMenuBlocked();
        frame.nativeReloadHandAuthorityActive =
            _nativeReloadHandAuthorityActive;
    
        if (auto* player = RE::PlayerCharacter::GetSingleton()) {
            (void)player;
            if (auto* playerNodes = f4vr::getPlayerNodes()) {
                const auto captureTrackedNode = [](const RE::NiNode* node) {
                    TrackedNodeFrame sample{};
                    if (node && finiteNiTransform(node->world)) {
                        sample.world = node->world;
                        sample.valid = true;
                    }
                    return sample;
                };
                frame.rightWand = captureTrackedNode(playerNodes->primaryWandNode);
                frame.leftWand = captureTrackedNode(playerNodes->SecondaryWandNode);
                frame.rightWeaponDriver = captureTrackedNode(playerNodes->primaryWeaponOffsetNOde);
                frame.leftWeaponDriver = captureTrackedNode(playerNodes->SecondaryMeleeWeaponOffsetNode2);
                if (playerNodes->HmdNode) {
                    frame.hmdPositionWorld = playerNodes->HmdNode->world.translate;
                    const RE::NiPoint3 rawHmdForwardWorld = playerNodes->HmdNode->world.rotate.Transpose() * RE::NiPoint3(0.0f, 1.0f, 0.0f);
                    frame.hasHmdFrame = selection_query_policy::tryNormalizeVectorForHmdCone(rawHmdForwardWorld, frame.hmdForwardWorld);
                }
            }
        }
    
        auto buildHandInput = [&](bool isLeft, Hand& hand) {
            HandFrameInput input{};
            input.isLeft = isLeft;
            const bool rootHandReady = _handBoneCache.isReady();
            const auto interactionHandFrame =
                rootHandReady ?
                getInteractionHandFrame(isLeft) :
                HandFrame{};
            input.disabled =
                (isLeft ?
                        s_leftHandDisabled.load(std::memory_order_acquire) :
                        s_rightHandDisabled.load(std::memory_order_acquire)) ||
                !interactionHandFrame.valid;
            if (input.disabled) {
                collision_isolated_hand_frame_runtime::clear(isLeft);
                return input;
            }
    
            input.rawHandWorld = interactionHandFrame.transform;
            input.handNode = interactionHandFrame.node;
            collision_isolated_hand_frame_runtime::publish(
                isLeft,
                input.rawHandWorld,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                frik_visual_authority::hasPublishedExternalHandWorldTransform(
                    frik_visual_authority::handFromBool(isLeft)));
            input.grabAnchorWorld = input.rawHandWorld.translate;
            RE::NiTransform closeSelectionBasisWorld = input.rawHandWorld;
            if (frame.worldReady) {
                RE::NiTransform proxyFrameWorld{};
                if (hand.tryComputeGrabProxyLocalPalmPocketFrameWorld(hknp, proxyFrameWorld)) {
                    input.grabAnchorWorld = proxyFrameWorld.translate;
                    closeSelectionBasisWorld = makeGeneratedProxyAuthorityRelationFrame(proxyFrameWorld);
                }
            }
            input.palmNormalWorld = computePalmNormalFromHandBasis(closeSelectionBasisWorld, isLeft);
            input.pointingWorld = computePointingVectorFromHandBasis(input.rawHandWorld, isLeft);
            input.closeSelectionDirectionWorld = computeCloseSelectionDirectionFromHandBasis(closeSelectionBasisWorld, isLeft);
            input.farSelectionDirectionWorld = computeFarSelectionDirectionFromHandBasis(input.rawHandWorld, isLeft);
            input.pinchDirectionWorld = computePinchDetectionDirectionFromHandBasis(closeSelectionBasisWorld, isLeft);
            if (g_rockConfig.rockDebugDrawGrabPockets) {
                root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
                if (root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, fingerSnapshot) &&
                    fingerSnapshot.valid &&
                    fingerSnapshot.fingers[0].valid &&
                    fingerSnapshot.fingers[1].valid) {
                    input.thumbPadWorld = fingerSnapshot.fingers[0].points[2];
                    input.indexPadWorld = fingerSnapshot.fingers[1].points[2];
                    input.pinchPocketWorld = (input.thumbPadWorld + input.indexPadWorld) * 0.5f;
                    input.hasPinchPocketWorld = true;
                }
            }
            return input;
        };
    
        frame.right = buildHandInput(false, _rightHand);
        frame.left = buildHandInput(true, _leftHand);
        return frame;
    }

    // Retire the dynamic weapon bodies when a frame stops early. It keeps the
    // Havok state consistent: the bodies go back only while the cached worlds
    // are still the live ones, otherwise the state is abandoned, not touched.
    void PhysicsInteraction::retireDynamicWeaponForInterruptedFrame()
    {
        if (!_initialized.load(std::memory_order_acquire)) {
            return;
        }
        auto* currentBhk = getPlayerBhkWorld();
        auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
        if (currentBhk && currentBhk == _cachedBhkWorld &&
            currentHknp && currentHknp == _cachedHknpWorld) {
            _dynamicWeaponCollision.retireAll(currentBhk);
        } else {
            _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
        }
    }

    // Frame entry gate. It runs every guard that can stop a frame, and each
    // rejection path does its own teardown before it returns false.
    //
    // Returns true only when the frame may proceed; frame is filled then.
    // Nothing here may be skipped: a missed teardown leaves the player with a
    // stale movement penalty, a frozen Havok state, or a leaked lease.
    bool PhysicsInteraction::tryBeginFrame(PhysicsFrameContext& frame)
    {
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        _equippedWeaponSheathCommittedThisFrame = {};
        _equippedWeaponUnsheathCommittedThisFrame = {};
        const auto& runtime = runtime_state::currentFrame();
        // A publication is valid only when this invocation reaches the single
        // completed-frame handoff below. Early returns must never let the main
        // loop freeze a previous frame's Havok state again.
        _pendingDebugOverlayFrame = {};
        refreshEquippedWeaponHandlingSettings();
        if (!runtime.visualAuthorityAvailable) {
            retireDynamicWeaponForInterruptedFrame();
            restoreHeldMassMovementSlowdown("frik-unavailable");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            return false;
        }

        // ROCK always binds raw controller identity physically: right is the
        // primary wand and left is the secondary wand. Weapon handedness is a
        // separate ROCK role and never remaps buttons/controllers.
        vrcf::VRControllers.update(false);

        // Before any early return below: a skipped consume would let a stale
        // accept-button press replay as a reload frames later (see the API doc).
        input_remap_runtime::updateFiringHandReloadInput(runtime.deltaSeconds);

        _deltaTime = runtime.deltaSeconds;

        if (_deltaTime <= 0.0f || _deltaTime > 0.1f) {
            _deltaTime = 1.0f / 90.0f;
        }
        enforceNativeGrabHapticRuntimeSuppression();
        _dynamicPushElapsedSeconds += _deltaTime;
        if (_dynamicPushCooldownUntil.size() > 512) {
            for (auto it = _dynamicPushCooldownUntil.begin(); it != _dynamicPushCooldownUntil.end();) {
                if (it->second <= _dynamicPushElapsedSeconds) {
                    it = _dynamicPushCooldownUntil.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (!runtime.localSkeletonReady) {
            if (_initialized) {
                ROCK_LOG_WARN(Update, "Local skeleton no longer ready — shutting down");
                shutdown();
            }
            return false;
        }

        const bool menuBlocking = runtime.localMenuBlocking;
        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                menuBlocking,
                false,
                false)) {
            retireDynamicWeaponForInterruptedFrame();
            _equippedWeaponMenuReconcilePending = true;
            if (_initialized) {
                _twoHandedGrip.reset();
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
                auto* bhkMenu = getPlayerBhkWorld();
                if (bhkMenu) {
                    auto* hknpMenu = getHknpWorld(bhkMenu);
                    if (hknpMenu) {
                        restoreAllHandCollisionLeases(hknpMenu);
                        releaseHeldObjectsForTeardown(
                            hknpMenu,
                            GrabReleaseCollisionRestoreMode::Delayed);
                    }
                } else {
                    clearAllHandCollisionSuppressionState();
                }
            }
            debug::ClearFrame();
            auto* snapshotBhk = getPlayerBhkWorld();
            auto* snapshotHknp = snapshotBhk ? getHknpWorld(snapshotBhk) : nullptr;
            if (snapshotBhk && snapshotHknp) {
                _dynamicWorldCarCollision.restoreAll(snapshotBhk, snapshotHknp, "menu-blocked");
            } else {
                _dynamicWorldCarCollision.abandon();
            }
            observeLifecycleFrame(snapshotBhk, snapshotHknp, ::rock::provider::RockProviderLifecycleReason::MenuBlocked);
            restoreHeldMassMovementSlowdown("menu-blocked");
            yieldFrameAndDispatch(false);
            return false;
        }

        if (weapon_authority_lifecycle_policy::shouldClearWeaponAuthorityForUpdateInterruption(
                false,
                !g_rockConfig.rockEnabled,
                false)) {
            if (_initialized) {
                shutdown();
            }
            debug::ClearFrame();
            return false;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            _dynamicWorldCarCollision.abandon();
            if (_initialized) {
                ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
                shutdown();
            }
            return false;
        }

        if (_initialized && bhk != _cachedBhkWorld) {
            ROCK_LOG_INFO(Update, "bhkWorld changed (cell transition) — reinitializing");

            shutdown();
        }

        if (!_initialized) {
            init();
            if (!_initialized) {
                return false;
            }
        }

        _cachedBhkWorld = bhk;

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            _dynamicWorldCarCollision.abandon();
            _cachedHknpWorld = nullptr;
            observeLifecycleFrame(bhk, nullptr, ::rock::provider::RockProviderLifecycleReason::WorldUnavailable);
            debug::ClearFrame();
            restoreHeldMassMovementSlowdown("world-unavailable");
            yieldFrameAndDispatch(true);
            return false;
        }
        _cachedHknpWorld = hknp;

        if (physics_scale::refreshAndLogIfChanged()) {
            ROCK_LOG_WARN(Config, "Authoritative Havok scale changed; invalidating ROCK-generated collision bodies");
            /*
             * A live scale change is a physics-frame convention change, not a
             * cosmetic setting reload. Existing constraints and ROCK-owned shapes
             * were authored with the previous conversion, so active interactions
             * must yield before generated bodies are destroyed and rebuilt.
             */
            releaseHeldObjectsForTeardown(
                hknp,
                GrabReleaseCollisionRestoreMode::Immediate);
            restoreAllHandCollisionLeases(hknp);
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _bodyContactRuntime.reset();
            clearWeaponContact(true);
            clearWeaponContact(false);

            destroyHandCollisions(bhk);
            destroyBodyBoneCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
            markGeneratedBodiesInvalidated();
            clearAllHandCollisionSuppressionState();
            restoreNativePlayerCollisionSuppression(hknp, "scale-change");
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        refreshHandBoneCache();
        sampleHandTransformParity();
        frame = buildFrameContext(bhk, hknp, _deltaTime);
        _palmClockGameFrameIndex.store(runtime.frameIndex, std::memory_order_release);
        _palmClockGameDeltaSeconds.store(frame.deltaSeconds, std::memory_order_release);
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::None);
        if (!generatedBodiesMatchLifecycle(bhk, hknp)) {
            const bool rebuilt =
                rebuildGeneratedBodiesForLifecycle(
                    bhk,
                    hknp,
                    "epoch-mismatch");
            if (rebuilt) {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesRebuilt);
            } else {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesInvalidated);
                ROCK_LOG_SAMPLE_DEBUG(Update,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "ROCK lifecycle generated-body rebuild pending: flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                    _lifecycleFlagsAtomic.load(std::memory_order_acquire),
                    _lastLifecycleReasonAtomic.load(std::memory_order_acquire),
                    _worldGenerationAtomic.load(std::memory_order_acquire),
                    _skeletonGenerationAtomic.load(std::memory_order_acquire),
                    _providerGenerationAtomic.load(std::memory_order_acquire),
                    _stableFrameCountAtomic.load(std::memory_order_acquire));
                debug::ClearFrame();
                yieldFrameAndDispatch(true);
                return false;
            }
        }

        if (!physicsWritesAllowedForWorld(hknp)) {
            ROCK_LOG_SAMPLE_DEBUG(Update,
                g_rockConfig.rockLogSampleMilliseconds,
                "ROCK lifecycle gate closed frame: flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                _lifecycleFlagsAtomic.load(std::memory_order_acquire),
                _lastLifecycleReasonAtomic.load(std::memory_order_acquire),
                _worldGenerationAtomic.load(std::memory_order_acquire),
                _skeletonGenerationAtomic.load(std::memory_order_acquire),
                _providerGenerationAtomic.load(std::memory_order_acquire),
                _stableFrameCountAtomic.load(std::memory_order_acquire));
            debug::ClearFrame();
            yieldFrameAndDispatch(true);
            return false;
        }

        return true;
    }

    void PhysicsInteraction::update()
    {
        PhysicsFrameContext frame{};
        if (!tryBeginFrame(frame)) {
            return;
        }

        presentation_trace::beginFrame(
            frame.gameFrameIndex,
            frame.preFrikSchedulerSequence,
            0);

        // The frame runs in this order. Each phase is a member below.
        EquippedWeaponFrame weaponFrame{};
        serviceFrameEntryReconcile(frame, weaponFrame);
        updateFrameColliders(frame);
        beginEquippedWeaponFrame(frame, weaponFrame);
        // Acquire hand evidence before any grip can change ownership.
        serviceWeaponContactAcquisition(frame, weaponFrame);
        // Resolve stash, grip, drop, and hand assignment in input order.
        serviceEquippedWeaponGripFrame(frame, weaponFrame);
        // Publish dynamic weapon authority after the grip solve is final.
        finishDynamicWeaponFrame(frame, weaponFrame);
        updateInteractionFrame(frame, weaponFrame);
        completeFrame(frame);
    }

    weapon_presentation_warm_up_policy::BlockReason
        PhysicsInteraction::weaponCollisionPresentationWarmUpBlockReason() const
    {
        const auto transition = _equippedWeaponTransition.getPublicSnapshot();
        const auto attachedHands =
            _twoHandedGrip.weaponCollisionAttachedHands();
        weapon_presentation_warm_up_policy::Inputs inputs{
            .handAttached = { attachedHands.left, attachedHands.right },
            .nativeRenderable = transition.nativeRenderable,
            .handPoseHandoffComplete = transition.handPoseHandoffComplete,
            .pairFilterReady = _dynamicHandCollision.isPairFilterReady(),
        };
        for (std::size_t index = 0; index < inputs.handAttached.size();
             ++index) {
            const bool isLeft = index == 0u;
            inputs.handPublicationReady[index] =
                frik_visual_authority::isExternalHandWorldPublicationReady(
                    frik_visual_authority::handFromBool(isLeft));
            inputs.handWeaponPairSuppressed[index] =
                _dynamicHandCollision.isWeaponPairSuppressedForHand(isLeft);
        }
        return weapon_presentation_warm_up_policy::blockReason(inputs);
    }

    presentation_transaction_policy::TransactionIdentity
        PhysicsInteraction::makeWeaponPresentationIdentity(
            const std::uint64_t weaponGenerationKey) const
    {
        const auto attachedHands =
            _twoHandedGrip.weaponCollisionAttachedHands();
        std::uint8_t attachedHandMask = 0;
        if (attachedHands.left) {
            attachedHandMask |= presentation_transaction_policy::handBit(true);
        }
        if (attachedHands.right) {
            attachedHandMask |= presentation_transaction_policy::handBit(false);
        }
        /*
         * The per-step physics solve counter is deliberately NOT part of the
         * identity. It advances every frame, and the identity must stay equal
         * from the staging frame to the next frame's readback; sample
         * staleness is already refused where the proposal is admitted.
         */
        return presentation_transaction_policy::TransactionIdentity{
            .weaponGenerationKey = weaponGenerationKey,
            .worldGeneration =
                _worldGenerationAtomic.load(std::memory_order_acquire),
            .skeletonGeneration =
                _skeletonGenerationAtomic.load(std::memory_order_acquire),
            .providerGeneration =
                _providerGenerationAtomic.load(std::memory_order_acquire),
            .attachedHandMask = attachedHandMask,
        };
    }

    void PhysicsInteraction::finalizePresentationTraceFrame()
    {
        presentation_trace::recordTransactionOutcome(
            _weaponPresentationCoordinator.deepestStageReached(),
            _weaponPresentationCoordinator.abortReason());
        auto* const weaponNode = resolveEquippedWeaponInteractionNode();
        presentation_trace::finalizeFrame(
            weaponNode != nullptr,
            weaponNode ? weaponNode->world : RE::NiTransform{});
    }

    // Phase 1. Settle anything a menu left half-done, then decide who owns the
    // weapon this frame. Every later phase reads the answer out of weaponFrame.
    void PhysicsInteraction::serviceFrameEntryReconcile(const PhysicsFrameContext& frame, EquippedWeaponFrame& weaponFrame)
    {
        const bool forceBareFistRecheck = _equippedWeaponMenuReconcilePending;
        if (_equippedWeaponMenuReconcilePending) {
            const bool firingHandIsLeft = _twoHandedGrip.isFiringGripOccupied() ?
                _twoHandedGrip.isFiringHandLeft() :
                _fixedFiringHandIsLeft;
            const bool primaryGrabHeld = input_remap_runtime::isRawButtonPhysicallyHeld(
                firingHandIsLeft,
                g_rockConfig.rockGrabButtonID);
            _pendingEquippedWeaponPrimaryOnlyGripStart = PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = _equippedWeaponHandlingSettings.primaryDetachEnabled &&
                    primaryGrabHeld,
                .isLeft = firingHandIsLeft,
            };
            _equippedWeaponMenuReconcilePending = false;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon ownership reconciled after menu: primaryGrabHeld={} pendingPrimaryOnlyStart={}",
                primaryGrabHeld ? "yes" : "no",
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending ? "yes" : "no");
        }
        enforceNoBareFistState(forceBareFistRecheck);

        // Keep the shared collision matrix at the configured policy.
        serviceCollisionLayerDrift(frame.hknpWorld);

        weaponFrame.weaponNode = resolveEquippedWeaponInteractionNode();
        /*
         * FRIK re-attaches the weapon node to the firing hand every frame
         * before ROCK runs, even in part-carry. Republish ROCK's solved carry
         * transform first so weapon-part probes, firing-grip zone checks, and
         * grip capture frames all read the weapon where the player sees it —
         * the same frame the generated colliders follow.
         */
        (void)_twoHandedGrip.republishPartCarryWeaponTransform(weaponFrame.weaponNode);
        weaponFrame.rightHandWeaponEquipped = weaponFrame.weaponNode != nullptr;
        weaponFrame.retainedWeaponCollisionActive =
            _weaponCollision.hasWeaponBody() && _weaponCollision.getCurrentWeaponGenerationKey() != 0;
        /*
         * Reload can temporarily remove the first-person weapon node while ROCK
         * deliberately retains the generated weapon body set. Keep the dominant
         * hand under weapon authority until those retained bodies are gone.
         */
        weaponFrame.rightHandWeaponAuthorityActive = weaponFrame.rightHandWeaponEquipped || weaponFrame.retainedWeaponCollisionActive;
        /*
         * A visible part-carry (no hand at the firing grip) frees the right hand
         * even while generated weapon bodies exist: the free hand needs live
         * colliders for offhand-parity interaction, the same layer 43 vs 44
         * coexistence the left hand already has. Reload-retained bodies with no
         * visible weapon node keep the dominant-hand suppression because the
         * part-carry state cannot survive a missing weapon node anyway.
         */
        if (weaponFrame.rightHandWeaponEquipped && _twoHandedGrip.isPartCarryActive()) {
            weaponFrame.rightHandWeaponAuthorityActive = false;
        }
        /*
         * Left-firing carry frees the right hand the same way: the LEFT hand
         * owns the firing grip and the weapon transform, so the right hand has
         * support/free parity (its own part grip is leased separately below).
         */
        if (weaponFrame.rightHandWeaponEquipped && _twoHandedGrip.isFiringHandLeft() && _twoHandedGrip.isFiringGripOccupied()) {
            weaponFrame.rightHandWeaponAuthorityActive = false;
        }
        weaponFrame.rightHandWeaponAuthorityActiveBeforeGrip = weaponFrame.rightHandWeaponAuthorityActive;
        weaponFrame.leftSupportGripActive = false;
        weaponFrame.rightPartGripActive = _twoHandedGrip.isHandPartGripping(false);
        if (weaponFrame.rightHandWeaponAuthorityActive) {
            suppressRightHandCollisionForDominantWeapon(frame.hknpWorld);
        } else {
            restoreRightHandCollisionAfterDominantWeapon(frame.hknpWorld);
        }
        // A part-gripping free hand is a transform driver like the support hand
        // and must not also solve contacts against the weapon package.
        if (weaponFrame.rightPartGripActive) {
            suppressHandCollisionForWeaponSupport(frame.hknpWorld, false);
        } else {
            restoreHandCollisionAfterWeaponSupport(frame.hknpWorld, false);
        }
    }

    // Phase 2. Push this frame's hand and body colliders at the physics world.
    // It runs after authority is known, so a suppressed hand queues no collider.
    void PhysicsInteraction::updateFrameColliders(const PhysicsFrameContext& frame)
    {
        const auto& runtime = runtime_state::currentFrame();

        updateHandCollisions(frame);
        logPalmClockSampleForHand("game-after-hand-collider-queue",
            _rightHand,
            frame.hknpWorld,
            frame.right.disabled ? nullptr : &frame.right.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        logPalmClockSampleForHand("game-after-hand-collider-queue",
            _leftHand,
            frame.hknpWorld,
            frame.left.disabled ? nullptr : &frame.left.rawHandWorld,
            runtime.frameIndex,
            frame.deltaSeconds,
            nullptr);
        updateBodyBoneCollisions(frame);
        updateNativePlayerCollisionSuppression(frame.bhkWorld, frame.hknpWorld);
    }

    // Phase 3. Open the equipped-weapon frame: refresh the weapon body, stamp the
    // generation and ownership keys, and route the weapon to a hand.
    void PhysicsInteraction::beginEquippedWeaponFrame(const PhysicsFrameContext& frame, EquippedWeaponFrame& weaponFrame)
    {
        const auto& runtime = runtime_state::currentFrame();

        {
            performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::WeaponCollision);

            if (g_rockConfig.rockDebugVerboseLogging) {
                if (++_wpnNodeLogCounter >= 90) {
                    _wpnNodeLogCounter = 0;
                    if (weaponFrame.weaponNode) {
                        ROCK_LOG_DEBUG(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}", weaponFrame.weaponNode->name.c_str(), weaponFrame.weaponNode->world.translate.x,
                            weaponFrame.weaponNode->world.translate.y, weaponFrame.weaponNode->world.translate.z, _weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
                    } else {
                    }
                }
            }
            _weaponCollision.update(frame.hknpWorld, weaponFrame.weaponNode, frame.deltaSeconds, runtime.weaponDrawn);
        }

        weaponFrame.generationKey = _weaponCollision.getCurrentWeaponGenerationKey();
        weaponFrame.ownershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        presentation_trace::recordWeaponGeneration(weaponFrame.generationKey);
        _weaponPresentationCoordinator.beginFrame(
            makeWeaponPresentationIdentity(weaponFrame.generationKey),
            presentation_transaction_policy::FrameStamp{
                .frameIndex = frame.gameFrameIndex,
                .schedulerSequence = frame.preFrikSchedulerSequence,
            });
        /*
         * Measure the weapon against its controller driver, before any ROCK
         * authority writes this frame. Player locomotion cancels out of that
         * local relation, so a moving value means the engine is still flying
         * the weapon into place after an equip or rebuilding its graph.
         */
        _weaponIntentStabilitySample = weapon_intent_stability_policy::update(
            _weaponIntentStabilityState,
            frame.rightWeaponDriver.valid,
            frame.rightWeaponDriver.world,
            weaponFrame.weaponNode != nullptr,
            weaponFrame.weaponNode ?
                weaponFrame.weaponNode->world :
                RE::NiTransform{},
            weaponFrame.generationKey);
        presentation_trace::recordIntentStability(
            _weaponIntentStabilitySample);
        _twoHandedGrip.beginWeaponCollisionPresentationFrame(
            weaponFrame.generationKey);
        const bool suppressDefaultNativeWeaponIntent =
            _twoHandedGrip.previousWeaponCollisionPresentationWasLive();
        _dynamicWeaponCollision.beginFrame(
            runtime.frameIndex,
            frame.hknpWorld,
            frame.bhkWorld,
            weaponFrame.weaponNode,
            weaponFrame.generationKey,
            g_rockConfig.rockWeaponCollisionEnabled &&
                g_rockConfig.rockWeaponCollisionDynamicBoxEnabled &&
                runtime.weaponDrawn &&
                !frame.menuBlocked &&
                physicsWritesAllowedForWorld(frame.hknpWorld) &&
                /*
                 * An unsettled weapon is still flying to its attach point.
                 * Driving a collision body along that flight sweeps the proxy
                 * through the world and manufactures contacts, which is how a
                 * plain equip produced a large correction against a static
                 * surface the player never touched.
                 */
                weapon_intent_stability_policy::isAdmissibleCollisionIntent(
                    _weaponIntentStabilitySample),
            suppressDefaultNativeWeaponIntent);
        reconcileEquippedWeaponHandlingMode();
        serviceEquippedWeaponHandAssignment(
            weaponFrame.weaponNode,
            weaponFrame.generationKey,
            weaponFrame.ownershipKey,
            input_remap_runtime::isMenuInputActive(),
            _equippedWeaponHandlingSettings);
        serviceFixedWeaponHand(
            weaponFrame.weaponNode,
            weaponFrame.generationKey,
            weaponFrame.ownershipKey,
            input_remap_runtime::isMenuInputActive());
    }

    // Phase 7. The hand-interaction pass: selection, grab input, world car
    // collision, contact resolution, and the haptics they produce.
    void PhysicsInteraction::updateInteractionFrame(const PhysicsFrameContext& frame, const EquippedWeaponFrame& weaponFrame)
    {
        refreshGeneratedBodyContactRegistry();
        updateSelection(frame);

        updateGrabInput(frame);
        auto selectedCloseCarTarget = [&](const Hand& hand, const HandFrameInput& handInput) {
            DynamicWorldCarTarget target{};
            if (handInput.disabled || hand.isHolding() || !hand.hasSelection()) {
                return target;
            }
            const auto& selection = hand.getSelection();
            if (selection.isFarSelection || !selection.refr || !fo4vr::isExplodableCar(selection.refr->GetObjectReference())) {
                return target;
            }
            target.ref = selection.refr;
            target.seedBodyId = selection.bodyId.value;
            return target;
        };
        _dynamicWorldCarCollision.update(
            frame.bhkWorld,
            frame.hknpWorld,
            std::array<DynamicWorldCarTarget, 2>{
                selectedCloseCarTarget(_rightHand, frame.right),
                selectedCloseCarTarget(_leftHand, frame.left),
            });
        updateHeldMassMovementSlowdown(frame.hknpWorld, frame.deltaSeconds);
        synchronizeContactEvidenceOwnership(weaponFrame.rightHandWeaponAuthorityActive, weaponFrame.leftSupportGripActive, weaponFrame.rightPartGripActive);

        /*
         * Dynamic hand collision runs after normal grab input so the final
         * grab, pull, support-grip, or weapon owner for this frame can gate its
         * lower-priority visual authority without delaying proxy tracking.
         */
        _dynamicHandCollision.updateFrame(
            frame,
            physicsWritesAllowedForWorld(frame.hknpWorld),
            _rightHand,
            _leftHand,
            _bodyBoneColliders,
            weaponFrame.rightHandWeaponAuthorityActive || weaponFrame.rightPartGripActive,
            weaponFrame.leftSupportGripActive ||
                (_twoHandedGrip.isFiringHandLeft() &&
                    _twoHandedGrip.isFiringGripOccupied()),
            _dynamicWeaponCollision.proxyBodyIdForDebug().value,
            _rightHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(false),
            _leftHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(true));
        logColliderClockTrace(frame, weaponFrame.weaponNode);
        const auto dynamicHandHapticEvents = _dynamicHandCollision.consumeHapticEvents();
        for (const auto& pulse : dynamicHandHapticEvents.hands) {
            if (!pulse.fire) {
                continue;
            }
            TouchGrabRuntime::HandReport touchGrabReport{};
            const bool surfaceGrabOwnsFeedback =
                g_rockConfig.rockSurfaceGrabHapticsEnabled &&
                _touchGrabRuntime.getHandReport(
                    pulse.isLeft,
                    touchGrabReport) &&
                touchGrabReport.kind ==
                    provider::RockProviderTouchGrabKindV1::FixedAnchor;
            if (surfaceGrabOwnsFeedback) {
                continue;
            }
            (void)_feedbackHaptics.queue(
                pulse.isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                g_rockConfig.rockHandCollisionDynamicHapticDurationSeconds,
                pulse.intensity);
        }
        updateFeedbackHaptics(frame.deltaSeconds);

        resolveContacts(frame);

        bool wasTouchingR = _rightHand.isTouching();
        bool wasTouchingL = _leftHand.isTouching();
        _rightHand.tickTouchState();
        _leftHand.tickTouchState();
        _rightHand.tickSemanticContactState();
        _leftHand.tickSemanticContactState();
        _handContactActivity.advanceFrame();
        if (wasTouchingR && !_rightHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, false, _rightHand.getLastTouchedRef(), _rightHand.getLastTouchedFormID(), _rightHand.getLastTouchedLayer());
        }
        if (wasTouchingL && !_leftHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, true, _leftHand.getLastTouchedRef(), _leftHand.getLastTouchedFormID(), _leftHand.getLastTouchedLayer());
        }

    }

    // Phase 8. The single completed-frame handoff. Reaching this point is what
    // makes a debug-overlay publication valid; every early return above skips it.
    void PhysicsInteraction::completeFrame(const PhysicsFrameContext& frame)
    {
        _deltaLogCounter++;
        if (g_rockConfig.rockDebugVerboseLogging && _deltaLogCounter >= 90) {
            _deltaLogCounter = 0;

            const auto& playerSpace = runtime_state::currentFrame().playerSpace;
            if (playerSpace.valid) {
                const auto smoothPos = playerSpace.world.translate;
                const bool moving = playerSpace.moving;

                if (_hasPrevPositions && moving) {
                    const auto smoothDelta = smoothPos - _prevSmoothedPos;

                    ROCK_LOG_DEBUG(Update, "PlayerSpace: smoothDelta=({:.2f},{:.2f},{:.2f}) moving={}", smoothDelta.x, smoothDelta.y, smoothDelta.z, moving);
                }

                _prevSmoothedPos = smoothPos;
                _hasPrevPositions = true;
            }
        }

        ::rock::provider::dispatchFrameCallbacks(*this);
        /*
         * Callback ownership is still published only after every main-thread
         * collider mutation and target update for this frame has committed.
         * The last of those moved out of this phase: generated weapon
         * collider targets are now published from the main hook, once the
         * gunstock finalize and provider AfterRock have finished moving the
         * weapon. So the registration moves with them rather than running
         * here, one step ahead of the mutation it is meant to follow.
         */
        _pendingStepDriveRegistration = PendingStepDriveRegistration{
            .bhkWorld = frame.bhkWorld,
            .hknpWorld = frame.hknpWorld,
            .frameIndex = frame.gameFrameIndex,
            .valid = true,
        };

        // Defer immutable overlay construction until the outer frame hook has
        // also completed native-animation finalization and every provider
        // animation phase. The context is consumed once in that same hook.
        _pendingDebugOverlayFrame = PendingDebugOverlayFrame{
            .context = frame,
            .valid = true,
        };
    }





    void PhysicsInteraction::publishDebugOverlayAfterFrameCallbacks()
    {
        if (!_pendingDebugOverlayFrame.valid) {
            return;
        }

        const auto frame = _pendingDebugOverlayFrame.context;
        _pendingDebugOverlayFrame = {};

        if (!_initialized.load(std::memory_order_acquire) ||
            !frame.worldReady || !frame.bhkWorld || !frame.hknpWorld ||
            frame.gameFrameIndex != runtime_state::currentFrame().frameIndex ||
            frame.bhkWorld != _cachedBhkWorld ||
            frame.hknpWorld != _cachedHknpWorld) {
            debug::ClearFrame();
            return;
        }

        auto* currentBhk = getPlayerBhkWorld();
        auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
        if (currentBhk != frame.bhkWorld || currentHknp != frame.hknpWorld) {
            debug::ClearFrame();
            return;
        }

        publishDebugBodyOverlay(frame);
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
            provider::currentNativeAnimationAuthorityFlagsV1();
        auto* equippedWeapon = currentEquippedWeaponForm();
        const std::uint64_t weaponGenerationKey =
            weaponNode ? _weaponCollision.getCurrentWeaponGenerationKey() : 0;
        std::uint64_t weaponOwnershipKey =
            weaponNode ? _weaponCollision.getCurrentEquippedWeaponOwnershipKey() : 0;
        if (weaponNode && weaponOwnershipKey == 0) {
            // Keep authored grip alignment independent of generated weapon
            // collision.
            // The richer stack/instance key wins when available; the equipped
            // form remains a stable freshness boundary when collision is off.
            weaponOwnershipKey = currentEquippedWeaponFormId();
        }

        const bool equippedGenerationMatchesForm =
            weaponNode &&
            equippedWeapon &&
            weaponGenerationKey != 0 &&
            weaponOwnershipKey != 0 &&
            _weaponCollision.getCurrentObservedEquippedWeaponFormID() == equippedWeapon->formID;

        native_idle_grip_preharvest::observeEquippedWeapon(
            equippedGenerationMatchesForm ? equippedWeapon : nullptr,
            equippedGenerationMatchesForm ? weaponNode : nullptr,
            equippedGenerationMatchesForm ? currentEquippedWeaponInstanceData(equippedWeapon) : nullptr,
            equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0);

        const bool rockFiringHandIsLeft =
            _twoHandedGrip.isFiringHandLeft();
        RE::NiTransform controllerHandWorld{};
        RE::NiNode* const controllerWand = rockFiringHandIsLeft ?
            f4vr::getLeftHandNode() :
            f4vr::getRightHandNode();
        const bool controllerHandWorldValid =
            controllerWand &&
            finiteNiTransform(controllerWand->world) &&
            _handFrameResolver.tryReconstructCalibratedHand(
                rockFiringHandIsLeft,
                _handBoneCache.getSkeleton(),
                _handBoneCache.getBoneTree(),
                controllerWand->world,
                controllerHandWorld);
        frik_visual_authority::HandWorldAuthoritySnapshot
            firingHandWorldWinner{};
        const bool weaponCoupledProviderWorldAuthorityActive =
            frik_visual_authority::tryGetPublishedExternalHandWorldWinner(
                frik_visual_authority::handFromBool(
                    rockFiringHandIsLeft),
                firingHandWorldWinner) &&
            firingHandWorldWinner.role ==
                frik_visual_authority::HandWorldAuthorityRole::
                    ProviderWeaponCoupled;

        _authoredPrimaryFiringGrip.update(AuthoredPrimaryFiringGripFrameInput{
            .weaponNode = weaponNode,
            .weapon = equippedWeapon,
            .weaponOwnershipKey = weaponOwnershipKey,
            .weaponGenerationKey = weaponGenerationKey,
            .weaponInstanceContentKey = equippedGenerationMatchesForm ? _weaponCollision.getCurrentEquippedWeaponInstanceContentKey() : 0,
            .weaponInstanceContentKnown = equippedGenerationMatchesForm,
            .controllerHandWorld = controllerHandWorld,
            .controllerHandWorldValid = controllerHandWorldValid,
            .weaponCoupledProviderWorldAuthorityActive =
                weaponCoupledProviderWorldAuthorityActive,
            .runtimeInitialized = _initialized.load(std::memory_order_acquire),
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
                _twoHandedGrip.blocksAuthoredPrimaryGripWeaponAlignment(),
            .weaponVisualReturnActive = _twoHandedGrip.isWeaponVisualReturnActive(),
            .primaryHandHoldingObject = rightHandHoldingObject,
            .rockFiringHandIsLeft = rockFiringHandIsLeft,
            .inPowerArmor = frik_skeleton_profile::effectiveInPowerArmor(),
        }, _twoHandedGrip);

        if (_equippedWeaponTransition.isHandPoseHandoffActive()) {
            const bool handoffHandIsLeft = _equippedWeaponTransition.handPoseHandoffIsLeft();
            if (nativeAuthorityFlags != 0 ||
                runtime.localMenuBlocking ||
                runtime.compatibilityConfigBlocking) {
                _equippedWeaponTransition.completeHandPoseHandoff("authored-pose-unavailable");
            } else if (equippedWeapon && equippedWeapon->formID != _equippedWeaponTransition.bridgeWeaponBaseFormID()) {
                _equippedWeaponTransition.completeHandPoseHandoff("equipped-weapon-changed");
            } else if (_twoHandedGrip.hasPublishedAuthoredPrimaryFiringGripFingerPose(handoffHandIsLeft)) {
                _equippedWeaponTransition.completeHandPoseHandoff("equipped-authored-pose-acquired");
            }
        }
    }

    void PhysicsInteraction::clearWeaponContact(bool isLeft)
    {
        auto& bodyId = isLeft ?
            _leftWeaponContactBodyId : _rightWeaponContactBodyId;
        auto& partKind = isLeft ?
            _leftWeaponContactPartKind : _rightWeaponContactPartKind;
        auto& reloadRole = isLeft ?
            _leftWeaponContactReloadRole : _rightWeaponContactReloadRole;
        auto& supportRole = isLeft ?
            _leftWeaponContactSupportRole : _rightWeaponContactSupportRole;
        auto& socketRole = isLeft ?
            _leftWeaponContactSocketRole : _rightWeaponContactSocketRole;
        auto& actionRole = isLeft ?
            _leftWeaponContactActionRole : _rightWeaponContactActionRole;
        auto& gripPose = isLeft ?
            _leftWeaponContactGripPose : _rightWeaponContactGripPose;
        auto& missedFrames = isLeft ?
            _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;

        // Publish the invalid body first so readers stop using this bank.
        bodyId.store(INVALID_CONTACT_BODY_ID, std::memory_order_release);
        partKind.store(
            static_cast<std::uint32_t>(WeaponPartKind::Other),
            std::memory_order_release);
        reloadRole.store(
            static_cast<std::uint32_t>(WeaponReloadRole::None),
            std::memory_order_release);
        supportRole.store(
            static_cast<std::uint32_t>(WeaponSupportGripRole::None),
            std::memory_order_release);
        socketRole.store(
            static_cast<std::uint32_t>(WeaponSocketRole::None),
            std::memory_order_release);
        actionRole.store(
            static_cast<std::uint32_t>(WeaponActionRole::None),
            std::memory_order_release);
        gripPose.store(
            static_cast<std::uint32_t>(WeaponGripPoseId::None),
            std::memory_order_release);
        missedFrames.store(
            WEAPON_CONTACT_TIMEOUT_FRAMES + 1,
            std::memory_order_release);
        _weaponInteractionAcquisitionStates[isLeft ? 0u : 1u] = {};
    }

    void PhysicsInteraction::publishWeaponInteractionContact(
        bool isLeft,
        WeaponInteractionContact& contact)
    {
        auto& partKind = isLeft ?
            _leftWeaponContactPartKind : _rightWeaponContactPartKind;
        auto& reloadRole = isLeft ?
            _leftWeaponContactReloadRole : _rightWeaponContactReloadRole;
        auto& supportRole = isLeft ?
            _leftWeaponContactSupportRole : _rightWeaponContactSupportRole;
        auto& socketRole = isLeft ?
            _leftWeaponContactSocketRole : _rightWeaponContactSocketRole;
        auto& actionRole = isLeft ?
            _leftWeaponContactActionRole : _rightWeaponContactActionRole;
        auto& gripPose = isLeft ?
            _leftWeaponContactGripPose : _rightWeaponContactGripPose;
        auto& sequence = isLeft ?
            _leftWeaponContactSequence : _rightWeaponContactSequence;
        auto& missedFrames = isLeft ?
            _leftWeaponContactMissedFrames : _rightWeaponContactMissedFrames;

        partKind.store(
            static_cast<std::uint32_t>(contact.partKind),
            std::memory_order_release);
        reloadRole.store(
            static_cast<std::uint32_t>(contact.reloadRole),
            std::memory_order_release);
        supportRole.store(
            static_cast<std::uint32_t>(contact.supportGripRole),
            std::memory_order_release);
        socketRole.store(
            static_cast<std::uint32_t>(contact.socketRole),
            std::memory_order_release);
        actionRole.store(
            static_cast<std::uint32_t>(contact.actionRole),
            std::memory_order_release);
        gripPose.store(
            static_cast<std::uint32_t>(contact.fallbackGripPose),
            std::memory_order_release);
        contact.sequence =
            sequence.fetch_add(1, std::memory_order_acq_rel) + 1;
        missedFrames.store(0, std::memory_order_release);
    }

    bool PhysicsInteraction::isHandContactEvidenceSuppressed(bool isLeft) const
    {
        /*
         * Native hknp contact callbacks can run on the physics boundary while
         * game-frame ownership is changing. Use only atomic state here: the
         * physics thread needs ROCK's "hand collision disabled while owned"
         * answer without reading Hand::_state directly.
         */
        const Hand& hand = isLeft ? _leftHand : _rightHand;
        return hand.hasContactEvidenceSuppressedAtomic() ||
               (!isLeft && _rightDominantWeaponCollisionSuppressed.load(std::memory_order_acquire)) ||
               (!isLeft && _rightWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) ||
               (isLeft && _leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire));
    }

    void PhysicsInteraction::clearContactEvidenceForHand(bool isLeft)
    {
        if (isLeft) {
            _leftHand.clearSemanticContactEvidence();
        } else {
            _rightHand.clearSemanticContactEvidence();
        }
    }

    void PhysicsInteraction::synchronizeContactEvidenceOwnership(bool rightHandWeaponAuthorityActive, bool leftSupportGripActive, bool rightPartGripActive)
    {
        /*
         * ROCK disables generated hand collision when a grab or two-hand/tool
         * owner has the hand. Clear semantic contact state at the same
         * authority transition so callbacks cannot leave a stale touch owner.
         */
        if (_rightHand.hasContactEvidenceSuppressedAtomic() || rightHandWeaponAuthorityActive || rightPartGripActive ||
            _rightDominantWeaponCollisionSuppressed.load(std::memory_order_acquire) ||
            _rightWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(false);
        }

        if (_leftHand.hasContactEvidenceSuppressedAtomic() || leftSupportGripActive ||
            _leftWeaponSupportCollisionSuppressed.load(std::memory_order_acquire)) {
            clearContactEvidenceForHand(true);
        }
    }


    void PhysicsInteraction::synchronizeNativeScopePresentationAfterFrikUpdate()
    {
        if (!_initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* weaponNode = f4vr::getWeaponNode();
        _twoHandedGrip.synchronizeNativeScopePresentationAfterFrikUpdate(weaponNode, _weaponCollision.getCurrentWeaponGenerationKey());
    }

    void PhysicsInteraction::finalizeGunstockPresentationAfterNativeAnimation()
    {
        if (!_initialized.load(std::memory_order_acquire) ||
            !g_rockConfig.rockGunstockModeEnabled ||
            !runtime_state::isLocalSkeletonReady() ||
            (provider::currentNativeAnimationAuthorityFlagsV1() &
                authored_weapon_grip_capture_policy::kWeapon) == 0) {
            return;
        }

        auto* weaponNode = resolveEquippedWeaponInteractionNode();
        (void)_twoHandedGrip.
            finalizeGunstockPresentationAfterNativeWeaponAnimation(
                weaponNode,
                _weaponCollision.getCurrentWeaponGenerationKey());
    }

    bool PhysicsInteraction::tryGetManualScopeDirectTransitionTarget(
        std::uint64_t& outWeaponGenerationKey,
        std::uint32_t& outNativeOverlayIndex) const
    {
        outWeaponGenerationKey = 0;
        outNativeOverlayIndex = 0;
        if (!_initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return false;
        }
        const auto snapshot = _weaponCollision.getNativeScopeSightAnchorSnapshot();
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
            !snapshot.manualDirectTransitionRequired || !snapshot.nativeScopeOverlayValid ||
            !native_scope_sight_anchor_policy::matchesCurrentEquippedWeapon(publishedIdentity, currentIdentity)) {
            return false;
        }
        outWeaponGenerationKey = snapshot.weaponGenerationKey;
        outNativeOverlayIndex = snapshot.nativeScopeOverlayIndex;
        return true;
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
            (provider::currentNativeAnimationAuthorityFlagsV1() &
                authored_weapon_grip_capture_policy::kWeapon) != 0 ||
            nativeGunState ==
                static_cast<std::uint32_t>(RE::GUN_STATE::kReloading);
        _equippedWeaponTransition.update(
            EquippedWeaponTransitionCoordinator::FrameInput{
                .deltaSeconds = runtime.deltaSeconds,
                .visualAuthorityAvailable = runtime.visualAuthorityAvailable,
                .localSkeletonReady = runtime.localSkeletonReady,
                .menuBlocking = runtime.localMenuBlocking,
                .compatibilityBlocking = runtime.compatibilityConfigBlocking,
                .nativeWeaponState = nativeWeaponState,
                .intentionalShoulderSheathActive =
                    _equippedWeaponShoulderSheath.active,
                .shoulderSheathFormID =
                    _equippedWeaponShoulderSheath.weaponFormID,
                .shoulderSheathInstanceData =
                    _equippedWeaponShoulderSheath.weaponInstanceData,
                .shoulderSheathEquipIndex =
                    _equippedWeaponShoulderSheath.equipIndex,
                .nativeWeaponAnimationActive = nativeWeaponAnimationActive,
                .sourceSchedulerSequence =
                    _currentPreFrikSchedulerSequence,
            });
    }

}

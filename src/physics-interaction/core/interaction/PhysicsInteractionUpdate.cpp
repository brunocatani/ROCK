#include "api/GrabEventCallbacks.h"
#include "api/AuthoredInteractionService.h"
#include "api/ProviderRuntimeServices.h"
#include "api/StorageService.h"
#include "api/WeaponPartCollisionService.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/native/NativePlayerCollisionFilter.h"
#include "physics-interaction/weapon/NativeHandEquipment.h"
#include "physics-interaction/weapon/NativeSecondaryPresentation.h"

// Per-frame orchestration: update(), interaction frame finalization, hand transform sampling, physics substep callbacks, held-mass slowdown, and the frame/debug-overlay implementation includes.

namespace rock
{
    void PhysicsInteraction::requestWeaponCollisionRebuildAfterWorkbenchExit(const char* sourceMenuName)
    {
        if (!_lifecycle.initialized.load(std::memory_order_acquire)) {
            return;
        }

        _weaponCollision.requestWorkbenchExitRebuild();
        _equipped.transition.requestCurrentWeaponReconcile(
            EquippedWeaponTransitionCoordinator::Source::WorkbenchExit);
        ROCK_LOG_DEBUG(Weapon,
            "Weapon collision workbench-exit rebuild gate armed by {} close",
            sourceMenuName ? sourceMenuName : "<unknown>");
    }

    bool PhysicsInteraction::tryGetRootFlattenedHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        outTransform = {};
        return _handBoneCache.isReady() &&
               frik_hand_world_authority::tryGetRawHandWorld(isLeft, outTransform);
    }

    bool PhysicsInteraction::refreshHandBoneCache()
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::HandFrameResolve);
        const bool resolved = _handBoneCache.resolve();

        /*
         * Isolate the controller hand once per frame, before any consumer
         * reads it. While a ROCK claim was solved this frame the rendered
         * hand bone is ROCK's own target, not the controller; the service
         * reconstructs the controller hand from FRIK's first-person hand
         * input. FRIK's solve result marks an unreachable claim.
         */
        frik_hand_world_authority::FrameHandSamples samples{};
        const auto sampleHand = [this, resolved](bool isLeft, frik_hand_world_authority::RawHandSample& outSample) {
            if (!resolved) {
                return;
            }
            outSample.flattenedHandWorld = _handBoneCache.getWorldTransform(isLeft);
            outSample.flattenedHandValid = true;
            outSample.bodyHandNodeValid = _handBoneCache.tryGetNodeWorldTransform(isLeft, outSample.bodyHandNodeWorld);
        };
        sampleHand(false, samples.right);
        sampleHand(true, samples.left);

        const std::uint64_t kickSequence = _twoHandedGrip.nativeRecoilKickSequence();
        samples.recoilKickThisFrame = kickSequence != _observedNativeRecoilKickSequence;
        _observedNativeRecoilKickSequence = kickSequence;

        frik_hand_world_authority::resolveRawHands(samples);

        return resolved;
    }

    void PhysicsInteraction::captureRenderedHands()
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::RenderedHandCapture);
        // The bone array was rebuilt by FRIK's world final: both hands' rendered
        // flattened bones and their nodes are this frame's final values.
        const bool resolved = _handBoneCache.resolve();
        frik_hand_world_authority::FrameHandSamples samples{};
        const auto sampleHand = [this, resolved](bool isLeft, frik_hand_world_authority::RawHandSample& outSample) {
            if (!resolved) {
                return;
            }
            outSample.flattenedHandWorld = _handBoneCache.getWorldTransform(isLeft);
            outSample.flattenedHandValid = true;
            outSample.bodyHandNodeValid = _handBoneCache.tryGetNodeWorldTransform(isLeft, outSample.bodyHandNodeWorld);
        };
        sampleHand(false, samples.right);
        sampleHand(true, samples.left);
        frik_hand_world_authority::captureRenderedFrame(samples);
    }

    void PhysicsInteraction::discardUnfinishedFramePose()
    {
        const auto pending = std::exchange(_frame.poseFrameIndex, 0);
        if (!pending || !_lifecycle.initialized.load(std::memory_order_acquire)) return;
        auto* bhk = getPlayerBhkWorld();
        auto* world = bhk ? getHknpWorld(bhk) : nullptr;
        if (!bhk || bhk != _lifecycle.cachedBhkWorld || !world || world != _lifecycle.cachedHknpWorld ||
            !physicsWritesAllowedForWorld(world)) return;
        auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
        _bodyBoneColliders.invalidatePose(world);
        _rightHand.invalidateCollisionPose(world);
        _leftHand.invalidateCollisionPose(world);
        _dynamicHandCollision.retireAll(bhk);
        _dynamicWeaponCollision.retireAll(bhk);
        _secondaryDynamicWeaponCollision.retireAll(bhk);
        ROCK_LOG_SAMPLE_WARN(Physics, 1000, "Collision pose publication interrupted frame={}; old collider targets invalidated", pending);
    }

    void PhysicsInteraction::finalizeFramePose()
    {
        const auto& runtime = runtime_state::currentFrame();
        const auto sourceFrame = _frame.poseFrameIndex;
        if (sourceFrame == 0 || sourceFrame != runtime.frameIndex ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.localSkeletonReady ||
            runtime.localMenuBlocking || runtime.compatibilityConfigBlocking) return;
        auto* bhk = getPlayerBhkWorld();
        auto* world = bhk ? getHknpWorld(bhk) : nullptr;
        if (!bhk || bhk != _lifecycle.cachedBhkWorld || !world || world != _lifecycle.cachedHknpWorld ||
            !physicsWritesAllowedForWorld(world)) return;

        // No native callback can consume half of this final pose publication.
        // The existing mutation gate resumes on every exit, including faults.
        {
            auto mutation = _generatedBodyStepDrive.callbackGate().pauseForMutation();
            auto frame = buildFrameContext(bhk, world);
            const bool bodyPoseValid = _bodyBoneColliders.finalizePose(world, frame.deltaSeconds, _finalPoseBoneSnapshot);
            // Presented poses already copied this final array. Transport that
            // same snapshot once; do not reread or blend animation phases.
            const bool handPoseValid = transportControllerHands(_finalPoseBoneSnapshot);
            for (const bool isLeft : { false, true }) {
                auto& input = isLeft ? frame.left : frame.right;
                auto& hand = isLeft ? _leftHand : _rightHand;
                if (!handPoseValid || input.disabled) {
                    hand.invalidateCollisionPose(world);
                    input.disabled = true;
                } else if (!hand.finalizeCollisionPose(world, input.rawHandWorld,
                        frame.deltaSeconds, _finalPoseBoneSnapshot)) input.disabled = true;
            }
            if (!bodyPoseValid) { frame.right.disabled = true; frame.left.disabled = true; }
            if (!bodyPoseValid || !handPoseValid) {
                ROCK_LOG_SAMPLE_WARN(Hand, 2000,
                    "Final collision pose unavailable frame={} body={} hands={}; invalid targets were not published",
                    runtime.frameIndex, bodyPoseValid, handPoseValid);
            }
            _dynamicHandCollision.finalizePose(frame, _rightHand, _leftHand, _bodyBoneColliders);
            finalizeSecondaryWeaponCollisionPose(frame, world, runtime.weaponDrawn);
            if (auto* weapon = resolveEquippedWeaponInteractionNode()) {
                performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponCollisionTransforms);
                _weaponCollision.updateBodiesFromCurrentSourceTransforms(world, weapon, frame.deltaSeconds, nullptr, 0);
                _dynamicWeaponCollision.finalizeCompoundPose(_weaponCollision, weapon,
                    frame, _weaponCollision.getCurrentWeaponGenerationKey());
            }
        }
        _frame.poseFrameIndex = 0;
        _generatedBodyStepDrive.registerForNextStep(bhk, world);
    }

    bool PhysicsInteraction::resolveSecondaryWeaponCollision(void* context,
        const RE::NiTransform& modelIntentWorld, RE::NiTransform& resolvedModelWorld)
    {
        resolvedModelWorld = modelIntentWorld;
        auto* self = static_cast<PhysicsInteraction*>(context);
        if (!self || !self->_lifecycle.initialized.load(std::memory_order_acquire)) return false;
        auto* bhk = self->getPlayerBhkWorld();
        auto* world = bhk ? getHknpWorld(bhk) : nullptr;
        if (!world || bhk != self->_lifecycle.cachedBhkWorld || world != self->_lifecycle.cachedHknpWorld) return false;
        return self->_secondaryDynamicWeaponCollision.resolvePresentedIntent(world, modelIntentWorld, resolvedModelWorld);
    }

    void PhysicsInteraction::finalizeSecondaryWeaponCollisionPose(const PhysicsFrameContext& frame,
        RE::hknpWorld* world, bool weaponDrawn)
    {
        /*
         * native_secondary_presentation places the second weapon through
         * AfterWorldFinal, so its colliders are built and posed here, after
         * that placement. The keyframed hulls (53) follow the presented gun;
         * the dynamic proxy (56) is driven toward the collision-free intent,
         * and its blocked deviation is what the presentation applies next frame.
         */
        performance_profiler::ScopedTimer timer(performance_profiler::Scope::WeaponCollisionTransforms);
        auto* model = native_secondary_presentation::presentedModel();
        const bool enabled = weaponDrawn && native_hand_equipment::hasSecondaryGun();
        _secondaryWeaponCollision.update(world, model, frame.deltaSeconds, enabled);
        native_secondary_presentation::setMirrorPlaneX(_secondaryWeaponCollision.holdingMirrorPlaneX());

        auto* modelNode = model ? model->IsNode() : nullptr;
        RE::NiTransform intent{};
        const bool hasIntent = modelNode && native_secondary_presentation::tryGetModelIntentWorld(intent);
        const auto generation = _secondaryWeaponCollision.getCurrentWeaponGenerationKey();
        _secondaryDynamicWeaponCollision.beginFrame(frame.timing.sequence, world, frame.bhkWorld, modelNode, generation,
            dynamic_weapon_collision_policy::kDynamicCompoundEnabled && enabled && hasIntent && !frame.menuBlocked);
        if (hasIntent) {
            _secondaryDynamicWeaponCollision.captureIntent(modelNode, intent, generation);
        }
        // Retires the proxy itself when this frame has no current intent.
        (void)_secondaryDynamicWeaponCollision.finishFrame(frame, true, modelNode, generation, _secondaryWeaponCollision, nullptr);
        if (_secondaryDynamicWeaponCollision.compoundSourcesUnavailable()) {
            _secondaryWeaponCollision.requestRebuildForReplacedSources();
        }
        if (model) {
            _secondaryWeaponCollision.updateBodiesFromCurrentSourceTransforms(world, model, frame.deltaSeconds, nullptr, 0);
        }
        if (modelNode) {
            _secondaryDynamicWeaponCollision.finalizeCompoundPose(_secondaryWeaponCollision, modelNode, frame, generation);
        }
    }


    RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft) const
    {
        RE::NiTransform rawHandWorld{};
        const bool cacheReady = _handBoneCache.isReady() &&
                                frik_hand_world_authority::tryGetRawHandWorld(isLeft, rawHandWorld);
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, rawHandWorld);
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    RE::NiNode* PhysicsInteraction::getInteractionHandNode(bool isLeft) const
    {
        RE::NiTransform rawHandWorld{};
        const bool cacheReady = _handBoneCache.isReady() &&
                                frik_hand_world_authority::tryGetRawHandWorld(isLeft, rawHandWorld);
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, rawHandWorld);
        if (frame.valid) {
            return frame.node;
        }

        return nullptr;
    }

#include "physics-interaction/core/PhysicsInteractionFrame.inl"

    void PhysicsInteraction::synchronizeNativeScopePresentationAfterFrikUpdate()
    {
        if (!_lifecycle.initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* weaponNode = f4vr::getWeaponNode();
        _twoHandedGrip.synchronizeNativeScopePresentationAfterFrikUpdate(weaponNode, _weaponCollision.getCurrentWeaponGenerationKey());
    }

    void PhysicsInteraction::finalizeFrikWeaponOwnershipForFrame()
    {
        // Same identity as the authored grip: the stack/instance key when
        // generated collision is on, the equipped form otherwise.
        std::uint64_t equippedWeaponOwnershipKey = _weaponCollision.getCurrentEquippedWeaponOwnershipKey();
        if (equippedWeaponOwnershipKey == 0) {
            equippedWeaponOwnershipKey = currentEquippedWeaponFormId();
        }
        _twoHandedGrip.finalizeFrikWeaponOwnershipForFrame(equippedWeaponOwnershipKey);
    }

    void PhysicsInteraction::syncFrikOffHandGripReport()
    {
        _twoHandedGrip.syncFrikOffHandGripReport();
    }

    void PhysicsInteraction::publishDebugRenderFrame()
    {
        const auto& runtime = runtime_state::currentFrame();
        const auto sourceFrameIndex = std::exchange(_frame.debugOverlayFrameIndex, 0);
        if (sourceFrameIndex == 0 || sourceFrameIndex != runtime.frameIndex ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.visualAuthorityAvailable ||
            !runtime.localSkeletonReady || runtime.localMenuBlocking || runtime.compatibilityConfigBlocking) {
            debug::ClearFrame();
            return;
        }
        auto* bhk = getPlayerBhkWorld();
        auto* hknp = bhk ? getHknpWorld(bhk) : nullptr;
        if (!bhk || bhk != _lifecycle.cachedBhkWorld || !hknp || hknp != _lifecycle.cachedHknpWorld) {
            debug::ClearFrame();
            return;
        }
        // AfterWorldFinal: provider animation, FRIK's claim re-solve and its
        // flattened bone rebuild have completed. Re-sample on the main thread;
        // PublishFrame copies values before the render thread consumes them.
        _twoHandedGrip.publishGunstockRenderDebug(resolveEquippedWeaponInteractionNode());
        const auto context=buildFrameContext(bhk,hknp);
        publishDebugBodyOverlay(context);
    }

    void PhysicsInteraction::finalizeInteractionFrame(
        const PhysicsFrameContext& frame,
        RE::hknpWorld* hknp,
        const EquippedWeaponFrameResult& equippedWeaponFrame)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::InteractionFinalize);
        const bool rightHandWeaponAuthorityActive = equippedWeaponFrame.rightHandWeaponAuthorityActive;
        const bool leftSupportGripActive = equippedWeaponFrame.leftSupportGripActive;
        const bool rightPartGripActive = equippedWeaponFrame.rightPartGripActive;

        refreshGeneratedBodyContactRegistry();
        updateSelection(frame);

        /*
         * ROCK applies player/room-space compensation before held-object grab
         * constraints are updated. That keeps the constraint target from solving
         * against a stale body velocity and removes the apparent held-object
         * teleport/stutter caused by compensating after the grab loop has already
         * written the frame target.
         */

        updateGrabInput(frame);
        _looseWeaponCollision.update(frame,
            _rightHand.isHolding() ? _rightHand.getHeldRef() : nullptr,
            _leftHand.isHolding() ? _leftHand.getHeldRef() : nullptr,
            _generatedBodyStepDrive.callbackGate());
        updateObjectRest(frame);
        commitHeldPlacement(frame);
        publishHeldCarExclusion(frame.hknpWorld);
        publishHeldBodyExclusion(frame.hknpWorld);
        updateHeldMassMovementSlowdown(hknp, frame.deltaSeconds);
        synchronizeContactEvidenceOwnership(rightHandWeaponAuthorityActive, leftSupportGripActive, rightPartGripActive);

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
            rightHandWeaponAuthorityActive || rightPartGripActive ||
                weaponHandSuppressedFor(false, WeaponHandReason::Grip),
            leftSupportGripActive ||
                equippedWeaponRoster().slotZeroGrips(true).firingGripActive ||
                weaponHandSuppressedFor(true, WeaponHandReason::Grip),
            _dynamicWeaponCollision.proxyBodyIdForDebug().value,
            _rightHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(false),
            _leftHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(true),
            _secondaryDynamicWeaponCollision.proxyBodyIdForDebug().value,
            _equippedSlotOne.hand == equipped_weapon_roster::Hand::Left);
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
                    ::rock::api::touch::TouchGrabKind::FixedAnchor;
            if (surfaceGrabOwnsFeedback) {
                continue;
            }
            (void)_feedbackHaptics.queue(
                pulse.isLeft ? feedback_haptics::FeedbackHand::Left : feedback_haptics::FeedbackHand::Right,
                dynamic_hand_collision_policy::kHapticDurationSeconds,
                pulse.intensity);
        }
        updateFeedbackHaptics(frame.deltaSeconds);

        _frame.debugOverlayFrameIndex = runtime_state::currentFrame().frameIndex;

        resolveContacts(frame);

        const float measuredFrameDeltaSeconds =
            frame.timing.valid ? frame.timing.deltaSeconds : 0.0f;
        _contacts.clock.advance(measuredFrameDeltaSeconds);

        /*
         * Bounded timing telemetry: one rate-limited line that explains the
         * active game and physics schedule (sequence identities, measured
         * deltas and effective rates, validity, fallback use, and the grab
         * source-clock state). Debug-gated; never per-frame in production.
         */
        if (g_rockConfig.rockDebugVerboseLogging) {
            const auto gameTelemetry = game_timing::telemetry();
            const auto physicsTelemetry = _generatedBodyStepDrive.physicsTimingTelemetry();
            const auto rightGrabClock = _rightHand.getGrabClockTelemetry();
            const auto leftGrabClock = _leftHand.getGrabClockTelemetry();
            const float gameHz = gameTelemetry.deltaSeconds > 0.0f ? 1.0f / gameTelemetry.deltaSeconds : 0.0f;
            const float physicsHz = physicsTelemetry.substepDeltaSeconds > 0.0f ? 1.0f / physicsTelemetry.substepDeltaSeconds : 0.0f;
            ROCK_LOG_SAMPLE_DEBUG(Update,
                g_rockConfig.rockLogSampleMilliseconds,
                "TIMING game: seq={} dt={:.6f} hz={:.1f} valid={} paused={} elapsed={:.2f}s disc={} invalid={} | physics: step={} solve={} rawDt={:.6f} subDt={:.6f} substeps={} hz={:.1f} fallback={} fallbackCount={} simulated={:.2f}s | phaseIdentity={} | grabR: hz={:.1f} srcInt={:.4f} | grabL: hz={:.1f} srcInt={:.4f}",
                gameTelemetry.sequence,
                gameTelemetry.deltaSeconds,
                gameHz,
                gameTelemetry.valid ? "y" : "n",
                gameTelemetry.menuPaused ? "y" : "n",
                gameTelemetry.elapsedGameSeconds,
                gameTelemetry.discontinuityCount,
                gameTelemetry.invalidSampleCount,
                physicsTelemetry.stepSequence,
                physicsTelemetry.solveSequence,
                physicsTelemetry.rawDeltaSeconds,
                physicsTelemetry.substepDeltaSeconds,
                physicsTelemetry.substepCount,
                physicsHz,
                physicsTelemetry.lastSampleUsedFallback ? "y" : "n",
                physicsTelemetry.fallbackSampleCount,
                physicsTelemetry.elapsedSimulatedSeconds,
                frame.timing.sequence,
                rightGrabClock.physicsHz,
                rightGrabClock.sourceIntervalSeconds,
                leftGrabClock.physicsHz,
                leftGrabClock.sourceIntervalSeconds);
        }

        _diagnostics.deltaLogCounter++;
        if (g_rockConfig.rockDebugVerboseLogging && _diagnostics.deltaLogCounter >= 90) {
            _diagnostics.deltaLogCounter = 0;

            const auto& playerSpace = runtime_state::currentFrame().playerSpace;
            if (playerSpace.valid) {
                const auto smoothPos = playerSpace.world.translate;
                const bool moving = playerSpace.moving;

                if (_frame.hasPrevPositions && moving) {
                    const auto smoothDelta = smoothPos - _frame.prevSmoothedPos;

                    ROCK_LOG_DEBUG(Update, "PlayerSpace: smoothDelta=({:.2f},{:.2f},{:.2f}) moving={}", smoothDelta.x, smoothDelta.y, smoothDelta.z, moving);
                }

                _frame.prevSmoothedPos = smoothPos;
                _frame.hasPrevPositions = true;
            }
        }

        updateNativeGrenadeCollisionSuppression(hknp, 0.0f);
        ::rock::provider::dispatchFrameCallbacks(*this);
        // Consumers can create and register an ejected reference in this callback.
        // Apply its pair exclusion in the same frame, before the next solve.
        updateEjectionContacts(frame);
        storage_service::update(*this, frame);
        storage_interaction::update(*this,frame,_dynamicHandCollision,_generatedBodyStepDrive.callbackGate());
        weapon_part_collision::update(*this, frame);
        // Equip identity/grip reconciliation can clear the early authored pose.
        // Yield the bridge only after every hand owner has finished this frame.
        finishEquippedWeaponHandPoseHandoff();
        captureEquippedWeaponContinuity();
        // Final pose publication consumes this completed decision frame and
        // registers the callback only after all collider targets are committed.
        _frame.poseFrameIndex = frame.timing.sequence;
        _frame.gripZoneIndicatorFrameIndex = frame.timing.sequence;
    }

    void PhysicsInteraction::interruptInteractionFrame(const char* reason)
    {
        debug::ClearFrame();
        _twoHandedGrip.reset();
        _equipped.transition.pendingGrip() = {};
        clearEquippedWeaponFiringGripInputState();
        restoreHeldMassMovementSlowdown(reason);
        _grabInput.shoulderStashStates = {};
        _grabInput.mouthConsumeStates = {};
        _feedbackHaptics.reset();
        ::rock::provider::dispatchFrameCallbacks(*this);
    }

    void PhysicsInteraction::update()
    {
        auto authoredCollision = F4SE::stl::scope_exit([this] {
            const auto& runtime = runtime_state::currentFrame();
            auto* bhk = getPlayerBhkWorld();
            auto* world = bhk ? getHknpWorld(bhk) : nullptr;
            const bool live = world && bhk == _lifecycle.cachedBhkWorld && world == _lifecycle.cachedHknpWorld;
            std::array<native_player_collision::BodyIdentity, 2> bodies{};
            std::size_t count = 0;
            if (live && _frame.poseFrameIndex == runtime.frameIndex && runtime.visualAuthorityAvailable &&
                runtime.localSkeletonReady && !runtime.localMenuBlocking && !runtime.compatibilityConfigBlocking) {
                for (bool left : {false, true}) if (authored_interaction::isAttached(left)) {
                    const auto body = havok_runtime::snapshotBody(world, _dynamicHandCollision.proxyBodyIdForDebug(left, 0));
                    if (body.valid) bodies[count++] = {body.bodyId.value, body.motionIndex,
                        reinterpret_cast<std::uintptr_t>(body.collisionObject), reinterpret_cast<std::uintptr_t>(body.ownerNode)};
                }
            } else authored_interaction::reset();
            native_player_collision::publishAuthoredHands(live ? world : nullptr, std::span(bodies).first(count));
        });
        auto cancelInterruptedFists = F4SE::stl::scope_exit([this] {
            cancelBareFistMode("interaction-frame-interrupted");
        });
        _frame.debugOverlayFrameIndex = 0;
        _frame.gripZoneIndicatorFrameIndex = 0;
        _frame.poseFrameIndex = 0;
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        _equipped.shoulderGestureConsumedThisFrame = {};
        _equipped.toggleGrabReleasePressConsumedThisFrame = {};
        _equipped.holsterInputConsumedThisFrame = {};
        const auto& runtime = runtime_state::currentFrame();
        if (!runtime.visualAuthorityAvailable || !runtime.localSkeletonReady ||
            runtime.localMenuBlocking || runtime.compatibilityConfigBlocking) {
            for (auto& visual : _drop.visuals) visual.release("interaction-interrupted");
        }
        if (!_suppression.nativeGrenadeLeases.empty()) {
            auto* bhk = getPlayerBhkWorld();
            auto* world = bhk ? getHknpWorld(bhk) : nullptr;
            if (bhk == _lifecycle.cachedBhkWorld && world == _lifecycle.cachedHknpWorld)
                updateNativeGrenadeCollisionSuppression(world, runtime.deltaSeconds);
        }
        beginHeldPlacementFrame();
        _dynamicWeaponCollision.updateSurfaceSupportInput(_placementClickReserved);
        const auto retireDynamicWeaponForInterruptedFrame = [this](bool preserveSurfaceSupport = false) {
            if (!_lifecycle.initialized.load(std::memory_order_acquire)) {
                return;
            }
            auto* currentBhk = getPlayerBhkWorld();
            auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
            native_interaction_service::clear(currentHknp, currentBhk);
            if (currentBhk && currentBhk == _lifecycle.cachedBhkWorld &&
                currentHknp && currentHknp == _lifecycle.cachedHknpWorld) {
                _dynamicWeaponCollision.retireAll(currentBhk, preserveSurfaceSupport);
                _secondaryDynamicWeaponCollision.retireAll(currentBhk);
            } else {
                _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
                _secondaryDynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            }
        };
        refreshEquippedWeaponHandlingSettings();
        // Sampled before any early return so input-thread grenade gating never
        // reads a stale pairing.
        sampleEquippedSlotOne();
        input_remap_runtime::setBothHandsHoldWeapons(equippedWeaponRoster().bothHandsHoldWeapons());
        if (!runtime.visualAuthorityAvailable) {
            retireDynamicWeaponForInterruptedFrame();
            restoreHeldMassMovementSlowdown("frik-unavailable");
            _grabInput.shoulderStashStates = {};
            _grabInput.mouthConsumeStates = {};
            _feedbackHaptics.reset();
            return;
        }

        _twoHandedGrip.beginWeaponCollisionPresentationFrame();

        // ROCK always binds raw controller identity physically: right is the
        // primary wand and left is the secondary wand. Weapon handedness is a
        // separate ROCK role and never remaps buttons/controllers.
        vrcf::VRControllers.update(false,
            g_rockConfig.rockFarGrabMode == static_cast<int>(far_pull_gesture::Mode::Gesture) ||
            _rightHand.getState() == HandState::SelectionLocked || _leftHand.getState() == HandState::SelectionLocked);

        // Before any early return below: a skipped consume would let a stale
        // accept-button press replay as a reload frames later (see the API doc).
        input_remap_runtime::updateFiringHandReloadInput(runtime.deltaSeconds);
        native_hand_equipment::observe();

        /*
         * runtime.deltaSeconds is the central sanitized game delta; no local
         * resanitization. Consumers migrate to runtime.timing individually
         * with explicit invalid-sample handling.
         */
        _frame.deltaTime = runtime.deltaSeconds;
        enforceNativeGrabHapticRuntimeSuppression();
        _contacts.hapticClockSeconds += _frame.deltaTime;

        if (!runtime.localSkeletonReady) {
            if (_lifecycle.initialized) {
                ROCK_LOG_WARN(Update, "Local skeleton no longer ready — shutting down");
                shutdown();
            }
            return;
        }

        if (runtime.localMenuBlocking) {
            retireDynamicWeaponForInterruptedFrame(true);
            _equipped.menuReconcilePending = true;
            auto* bhkMenu = getPlayerBhkWorld();
            auto* hknpMenu = bhkMenu ? getHknpWorld(bhkMenu) : nullptr;
            if (_lifecycle.initialized) {
                if (bhkMenu && bhkMenu == _lifecycle.cachedBhkWorld && hknpMenu && hknpMenu == _lifecycle.cachedHknpWorld) {
                    // Weapon grips yield to the menu and resume through the
                    // equip continuity path. Held objects stay held: no menu
                    // drops what a hand is holding.
                    clearWeaponHandSuppression(hknpMenu, false);
                    clearWeaponHandSuppression(hknpMenu, true);
                } else {
                    shutdown();
                }
            }
            storage_service::shutdown(hknpMenu);
            storage_interaction::shutdown(hknpMenu);
            observeLifecycleFrame(bhkMenu, hknpMenu, api::core::LifecycleReason::MenuBlocked);
            interruptInteractionFrame("menu-blocked");
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            if (_lifecycle.initialized) {
                ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
                shutdown();
            }
            return;
        }

        if (_lifecycle.initialized && (bhk != _lifecycle.cachedBhkWorld || getHknpWorld(bhk) != _lifecycle.cachedHknpWorld)) {
            ROCK_LOG_INFO(Update, "Physics world changed (cell transition) — reinitializing");

            shutdown();
        }

        if (!_lifecycle.initialized) {
            init();
            if (!_lifecycle.initialized) {
                return;
            }
        }

        _lifecycle.cachedBhkWorld = bhk;

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            native_interaction_service::clear(nullptr, nullptr);
            _lifecycle.cachedHknpWorld = nullptr;
            observeLifecycleFrame(bhk, nullptr, api::core::LifecycleReason::WorldUnavailable);
            _equipped.gripResumePending = _equipped.continuityGrip.pending;
            interruptInteractionFrame("world-unavailable");
            return;
        }
        _lifecycle.cachedHknpWorld = hknp;

        refreshHandBoneCache();
        const auto frame = buildFrameContext(bhk, hknp);
        _frame.palmClockGameFrameIndex.store(runtime.frameIndex, std::memory_order_release);
        _frame.palmClockGameDeltaSeconds.store(frame.deltaSeconds, std::memory_order_release);
        observeLifecycleFrame(bhk, hknp, api::core::LifecycleReason::None);

        if (!physicsWritesAllowedForWorld(hknp)) {
            native_interaction_service::clear(hknp, bhk);
            _equipped.gripResumePending = _equipped.continuityGrip.pending;
            interruptInteractionFrame("physics-gate-closed");
            return;
        }

        if (_equipped.gripResumePending &&
            frik_hand_world_authority::hasCalibratedRawHandFrame(false) &&
            frik_hand_world_authority::hasCalibratedRawHandFrame(true)) {
            _equipped.transition.resumeMenuGrip(_equipped.continuityGrip,
                _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
                _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire));
            const auto& resumed = _equipped.transition.pendingGrip();
            if (resumed.menuResume) {
                for (const bool left : { false, true }) {
                    const bool firing = !resumed.supportGrip.validCarry() && left == resumed.isLeft;
                    const bool occupied = left == resumed.isLeft || resumed.pairedGrips.valid() || resumed.secondSupportGrip.validCarry();
                    _equipped.resumeAwaitingHold[equipped_weapon_toggle_grab_policy::handIndex(left)] = occupied &&
                        !equipped_weapon_toggle_grab_policy::usesToggleForRole(_equipped.handlingSettings.weaponGrabMode, firing);
                }
            }
            _equipped.gripResumePending = false;
        }

        const bool forceBareFistRecheck = _equipped.menuReconcilePending;
        if (_equipped.menuReconcilePending && !_equipped.transition.pendingGrip().menuResume &&
            !_equipped.gripResumePending && !_equipped.transition.heldTransfer().active()) {
            const bool firingHandIsLeft = equippedWeaponRoster().slotZeroGrips(true).firingGripActive;
            const auto detachDecision =
                resolveEquippedWeaponDetachDecision(
                    _equipped.handlingSettings);
            const bool primaryGrabHeld = input_remap_runtime::isRawButtonPhysicallyHeld(
                firingHandIsLeft,
                input_remap_policy::kGrabButtonId);
            _equipped.transition.pendingGrip() = PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = detachDecision.primaryDetachEnabled &&
                    primaryGrabHeld,
                .isLeft = firingHandIsLeft,
                .toggleAcquisitionCommitted =
                    primaryGrabHeld,
            };
            _equipped.menuReconcilePending = false;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon ownership reconciled after menu: primaryGrabHeld={} pendingPrimaryOnlyStart={}",
                primaryGrabHeld ? "yes" : "no",
                _equipped.transition.pendingGrip().pending ? "yes" : "no");
        }
        _equipped.menuReconcilePending = false;
        updateBareFistMode(frame);
        enforceNoBareFistState(forceBareFistRecheck);

        native_interaction_service::update(frame);
        if (_layers.handStaticWorld != g_rockConfig.rockHandCollisionStaticWorldEnabled ||
            _layers.weaponBlocksProjectiles != g_rockConfig.rockWeaponCollisionBlocksProjectiles ||
            _layers.weaponBlocksSpells != g_rockConfig.rockWeaponCollisionBlocksSpells ||
            _layers.extensionObstacleLayers != native_interaction_service::dynamicObstacleLayers()) {
            ROCK_LOG_INFO(Config, "ROCK collision layer config changed; re-registering matrix policy");
            registerCollisionLayer(hknp);
        }


        // The hands that own the equipped weapon are leased before the
        // colliders update, so recreated hand bodies are re-leased before
        // the next physics step.
        const auto equippedWeaponPreparation = beginEquippedWeaponFrame(frame, hknp);
        updateHandCollisions(frame);
        const auto frameIndex = runtime_state::currentFrame().frameIndex;
        logPalmClockSampleForHand("game-after-hand-collider-prepare", _rightHand, hknp,
            frame.right.disabled ? nullptr : &frame.right.rawHandWorld, frameIndex, frame.deltaSeconds, nullptr);
        logPalmClockSampleForHand("game-after-hand-collider-prepare", _leftHand, hknp,
            frame.left.disabled ? nullptr : &frame.left.rawHandWorld, frameIndex, frame.deltaSeconds, nullptr);
        updateBodyBoneCollisions(frame);
        updateNativePlayerCollisionFilter(bhk, hknp, frame.deltaSeconds);
        const auto equippedWeaponFrame = updateEquippedWeaponFrame(frame, bhk, hknp, equippedWeaponPreparation);
        finalizeInteractionFrame(frame, hknp, equippedWeaponFrame);
        if (input_remap_runtime::ownsBareFistInput()) {
            if (const auto* reason = bareFistHandBlockReason(frame)) cancelBareFistMode(reason);
        }
        cancelInterruptedFists.release();
    }

    void PhysicsInteraction::onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->driveGeneratedCollidersFromPhysicsSubstep(world, timing);
    }

    void PhysicsInteraction::onCustomGrabAuthorityBetweenStep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->driveCustomGrabAuthorityFromBetweenStep(world, timing);
    }

    void PhysicsInteraction::onCustomGrabAuthorityAfterSolve(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        self->observeCustomGrabAuthorityAfterSolve(world, timing);
    }

    void PhysicsInteraction::driveGeneratedCollidersFromPhysicsSubstep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        performance_profiler::ScopedTimer profilerTimer(performance_profiler::Scope::GeneratedColliderPhysicsFlush);

        if (!world || !_lifecycle.initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        native_interaction_service::preCollide(world, timing);
        _rightHand.flushPendingCollisionPhysicsDrive(world, timing);
        _leftHand.flushPendingCollisionPhysicsDrive(world, timing);
        _bodyBoneColliders.flushPendingPhysicsDrive(world, timing);
        _weaponCollision.flushPendingPhysicsDrive(world, timing);
        _secondaryWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _secondaryDynamicWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicHandCollision.flushPendingPhysicsDrive(world, timing);
        storage_interaction::flush(world,timing);
        _rightHand.flushConsumerObjectPairs(world);
        _leftHand.flushConsumerObjectPairs(world);
        if (performance_profiler::enabled()) {
            performance_profiler::observeValue(performance_profiler::ValueMetric::GeneratedHandBodies,
                _rightHand.getHandColliderBodyCount() + _leftHand.getHandColliderBodyCount());
            performance_profiler::observeValue(performance_profiler::ValueMetric::GeneratedBodyBodies, _bodyBoneColliders.getBodyCount());
            performance_profiler::observeValue(performance_profiler::ValueMetric::GeneratedWeaponBodies, _weaponCollision.getWeaponBodyCount());
        }
        const auto gameFrameIndex = _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _frame.palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-collider-drive", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-collider-drive", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
    }

    void PhysicsInteraction::driveCustomGrabAuthorityFromBetweenStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_lifecycle.initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        const auto gameFrameIndex = _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _frame.palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-between-before-grab-flush", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-between-before-grab-flush", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        _rightHand.flushPendingCustomGrabAuthority(world, timing);
        _leftHand.flushPendingCustomGrabAuthority(world, timing);
    }

    void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_lifecycle.initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        const auto completedSolveSequence =
            _frame.completedPhysicsSolveSequence.fetch_add(
                1,
                std::memory_order_release) +
            1;
        _rightHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _leftHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _dynamicWeaponCollision.samplePostSolve(
            world,
            completedSolveSequence,
            timing);
        _secondaryDynamicWeaponCollision.samplePostSolve(world, completedSolveSequence, timing);
        _dynamicHandCollision.samplePostSolveDeviations(world, timing);
        const auto gameFrameIndex = _frame.palmClockGameFrameIndex.load(std::memory_order_acquire);
        debug::CapturePostSolveBodyPhases(
            world,
            timing,
            gameFrameIndex,
            completedSolveSequence);
        const auto gameDeltaSeconds = _frame.palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-solve", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-solve", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        serviceRetiredGrabConstraintPayloads();
        _weaponCollision.serviceRetiredWeaponBodies(world);
        _secondaryWeaponCollision.serviceRetiredWeaponBodies(world);
        // Neutralizes hand/body and grab-authority wrappers removed on the main
        // thread after the broadphase grace, while retaining their addresses for
        // native late readers. All generated body owners share this post-solve
        // cadence and exact-world witness.
        BethesdaPhysicsBody::serviceRetiredDeferredPayloads(world);
    }

#include "physics-interaction/core/PhysicsInteractionDebugOverlay.inl"
    void PhysicsInteraction::updateSelection(const PhysicsFrameContext& frame)
    {
        if (!runtime_state::isLocalSkeletonReady()) {
            return;
        }

        const auto rightPendingTargetPtr = _forceGrab.pendingCommits[0].targetHandle.get();
        const auto leftPendingTargetPtr = _forceGrab.pendingCommits[1].targetHandle.get();

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
            _forceGrab.pendingCommits[0].active ? rightPendingTargetPtr.get() : nullptr);
        const auto leftHandContext = selectionContextForOtherHand(
            _leftHand,
            _forceGrab.pendingCommits[1].active ? leftPendingTargetPtr.get() : nullptr);
        const auto farHmdConeGate = makeFarSelectionHmdConeGate(frame);

        if (_forceGrab.pendingCommits[0].active) {
            if (_rightHand.hasSelection()) {
                _rightHand.clearSelectionState(false);
            }
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
                frame.deltaSeconds,
                leftHandContext);
        }

        if (_forceGrab.pendingCommits[1].active) {
            if (_leftHand.hasSelection()) {
                _leftHand.clearSelectionState(false);
            }
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
                frame.deltaSeconds,
                rightHandContext);
        }
    }

    void PhysicsInteraction::restoreHeldMassMovementSlowdown(const char* reason)
    {
        if (_frame.heldMassSpeedReduction <= 0.0f) {
            return;
        }

        const float previousReduction = _frame.heldMassSpeedReduction;
        if (applyPlayerSpeedReduction(previousReduction, 0.0f)) {
            _frame.heldMassSpeedReduction = 0.0f;
            _frame.heldMassFadeStartReduction = 0.0f;
            _frame.heldMassFadeElapsedSeconds = 0.0f;
            ROCK_LOG_DEBUG(Hand,
                "Held mass movement slowdown restored: previousReduction={:.2f} reason={}",
                previousReduction,
                reason ? reason : "restore");
        } else {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown restore delayed: previousReduction={:.2f} reason={}",
                previousReduction,
                reason ? reason : "restore");
        }
    }

    void PhysicsInteraction::updateHeldMassMovementSlowdown(RE::hknpWorld* hknp, float deltaSeconds)
    {
        if (!g_rockConfig.rockGrabHeldMassMovementSlowdownEnabled) {
            restoreHeldMassMovementSlowdown("disabled");
            return;
        }

        float heldMass = 0.0f;
        if (hknp) {
            constexpr std::size_t kMaxMovementMassMotionSlots = 160;
            std::array<std::uint32_t, kMaxMovementMassMotionSlots> sampledMotionSlots{};
            std::size_t sampledMotionSlotCount = 0;

            auto motionAlreadySampled = [&](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                    if (sampledMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto sampleBody = [&](std::uint32_t bodyId) {
                if (isInvalidGrabBodyId(bodyId)) {
                    return;
                }

                auto* body = havok_runtime::getBody(hknp, RE::hknpBodyId{ bodyId });
                if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) || motionAlreadySampled(body->motionIndex)) {
                    return;
                }
                if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                    return;
                }

                const float mass = readGrabEventBodyMass(hknp, bodyId);
                if (!std::isfinite(mass) || mass <= 0.0f) {
                    return;
                }

                sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
                heldMass += mass;
            };

            auto sampleHand = [&](const Hand& hand) {
                if (!hand.isHolding()) {
                    return;
                }

                const auto& savedState = hand.getSavedObjectState();
                sampleBody(savedState.bodyId.value);
                for (const auto bodyId : hand.getHeldBodyIds()) {
                    sampleBody(bodyId);
                }
            };

            sampleHand(_rightHand);
            sampleHand(_leftHand);
        }

        const held_mass_movement::Config movementConfig{
            .enabled = g_rockConfig.rockGrabHeldMassMovementSlowdownEnabled,
            .massProportion = g_rockConfig.rockGrabHeldMassMovementMassProportion,
            .massExponent = g_rockConfig.rockGrabHeldMassMovementMassExponent,
            .maxReduction = g_rockConfig.rockGrabHeldMassMovementMaxReduction,
            .fadeOutSeconds = g_rockConfig.rockGrabHeldMassMovementFadeOutSeconds,
        };
        const float heldMassReduction = held_mass_movement::computeHeldMassReduction(heldMass, movementConfig);
        float targetReduction = heldMassReduction;
        if (heldMassReduction > 0.0f) {
            _frame.heldMassFadeStartReduction = heldMassReduction;
            _frame.heldMassFadeElapsedSeconds = 0.0f;
        } else if (_frame.heldMassSpeedReduction > 0.0f) {
            if (_frame.heldMassFadeStartReduction <= 0.0f) {
                _frame.heldMassFadeStartReduction = _frame.heldMassSpeedReduction;
                _frame.heldMassFadeElapsedSeconds = 0.0f;
            }
            _frame.heldMassFadeElapsedSeconds += std::isfinite(deltaSeconds) ? (std::max)(0.0f, deltaSeconds) : 0.0f;
            targetReduction = held_mass_movement::computeFadeOutReduction(
                _frame.heldMassFadeStartReduction,
                _frame.heldMassFadeElapsedSeconds,
                movementConfig.fadeOutSeconds);
        } else {
            _frame.heldMassFadeStartReduction = 0.0f;
            _frame.heldMassFadeElapsedSeconds = 0.0f;
        }

        if (std::fabs(targetReduction - _frame.heldMassSpeedReduction) <= 0.001f &&
            (targetReduction > 0.0f || _frame.heldMassSpeedReduction <= 0.0f)) {
            return;
        }

        const float previousReduction = _frame.heldMassSpeedReduction;
        if (!applyPlayerSpeedReduction(previousReduction, targetReduction)) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown skipped: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                heldMass,
                previousReduction,
                targetReduction);
            return;
        }

        _frame.heldMassSpeedReduction = targetReduction;
        if (targetReduction <= 0.0f) {
            _frame.heldMassFadeStartReduction = 0.0f;
            _frame.heldMassFadeElapsedSeconds = 0.0f;
        }
    }
}

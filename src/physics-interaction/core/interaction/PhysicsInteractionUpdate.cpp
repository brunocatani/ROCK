#include "physics-interaction/core/PhysicsInteractionInternal.h"

// Per-frame orchestration: update(), interaction frame finalization, hand transform sampling, physics substep callbacks, held-mass slowdown, and the frame/debug-overlay implementation includes.

namespace rock
{
    void PhysicsInteraction::requestWeaponCollisionRebuildAfterWorkbenchExit(const char* sourceMenuName)
    {
        if (!_initialized.load(std::memory_order_acquire)) {
            return;
        }

        _weaponCollision.requestWorkbenchExitRebuild();
        _equippedWeaponTransition.requestCurrentWeaponReconcile(
            EquippedWeaponTransitionCoordinator::Source::WorkbenchExit);
        ROCK_LOG_DEBUG(Weapon,
            "Weapon collision workbench-exit rebuild gate armed by {} close",
            sourceMenuName ? sourceMenuName : "<unknown>");
    }

    bool PhysicsInteraction::tryGetRootFlattenedHandTransform(bool isLeft, RE::NiTransform& outTransform) const
    {
        outTransform = {};
        if (!_handBoneCache.isReady()) {
            return false;
        }

        outTransform = _handBoneCache.getWorldTransform(isLeft);
        return true;
    }

    bool PhysicsInteraction::refreshHandBoneCache()
    {
        if (_handBoneCache.resolve()) {
            _handCacheResolveLogCounter = 0;
            return true;
        }

        if (g_rockConfig.rockDebugHandTransformParity) {
            if (++_handCacheResolveLogCounter == 1 || _handCacheResolveLogCounter % 90 == 0) {
                ROCK_LOG_WARN(Hand, "HandBoneCache unresolved; raw parity sampling skipped this frame");
            }
        }

        return false;
    }

    RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft) const
    {
        const bool cacheReady = _handBoneCache.isReady();
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, cacheReady ? _handBoneCache.getWorldTransform(isLeft) : RE::NiTransform());
        if (frame.valid) {
            return frame.transform;
        }

        return RE::NiTransform();
    }

    RE::NiNode* PhysicsInteraction::getInteractionHandNode(bool isLeft) const
    {
        const bool cacheReady = _handBoneCache.isReady();
        const auto frame = _handFrameResolver.resolve(isLeft, cacheReady, cacheReady ? _handBoneCache.getWorldTransform(isLeft) : RE::NiTransform());
        if (frame.valid) {
            return frame.node;
        }

        return nullptr;
    }

    void PhysicsInteraction::sampleHandTransformParity()
    {
        if (!g_rockConfig.rockDebugHandTransformParity) {
            _parityEnabledLogged = false;
            _paritySummaryCounter = 0;
            return;
        }

        if (!frik_visual_authority::isAvailable() || !_handBoneCache.isReady()) {
            return;
        }

        if (!_parityEnabledLogged) {
            ROCK_LOG_INFO(Init, "Hand-transform parity enabled (root flattened cache vs FRIK API, pre-write sampling)");
            _parityEnabledLogged = true;
        }

        const bool playerMoving = runtime_state::currentFrame().playerSpace.moving;
        const bool emitSummary = (++_paritySummaryCounter >= kRawParitySummaryFrames);

        auto sampleHand = [&](bool isLeft) {
            auto& state = _rawHandParityStates[isLeft ? 1 : 0];
            const auto handEnum = handFromBool(isLeft);
            const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
            RE::NiTransform apiTransform{};
            if (!frik_visual_authority::tryGetHandWorldTransform(
                    handEnum,
                    apiTransform)) {
                state = {};
                return;
            }
            const auto delta = measureTransformDelta(localTransform, apiTransform);
            const auto localPalmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(localTransform, isLeft);
            const auto apiPalmPosition = computeGrabLegacyPalmPivotAWorldFromHandBasis(apiTransform, isLeft);
            const auto localPalmNormal = computePalmNormalFromHandBasis(localTransform, isLeft);
            const auto apiPalmNormal = computePalmNormalFromHandBasis(apiTransform, isLeft);
            const auto localPointing = computePointingVectorFromHandBasis(localTransform, isLeft);
            const auto apiPointing = computePointingVectorFromHandBasis(apiTransform, isLeft);
            state.lastPositionDelta = delta.position;
            state.lastRotationDeltaDegrees = delta.rotationDegrees;

            const bool warnExceeded = delta.position > kRawParityWarnPosition || delta.rotationDegrees > kRawParityWarnRotationDegrees;
            const bool failExceeded = delta.position > kRawParityFailPosition || delta.rotationDegrees > kRawParityFailRotationDegrees;

            state.warnFrames = warnExceeded ? state.warnFrames + 1 : 0;
            state.failFrames = failExceeded ? state.failFrames + 1 : 0;

            const char* handLabel = isLeft ? "Left" : "Right";
            if (state.warnFrames == kRawParityWarnFrames) {
                ROCK_LOG_WARN(Hand, "{} raw hand parity warning: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (state.failFrames == kRawParityFailFrames) {
                ROCK_LOG_ERROR(Hand, "{} raw hand parity failure: posDelta={:.3f} rotDelta={:.3f}deg", handLabel, delta.position, delta.rotationDegrees);
            }

            if (playerMoving && state.hasPreviousApiTransform) {
                const auto prevApiDelta = measureTransformDelta(localTransform, state.previousApiTransform);
                if (prevApiDelta.position + kRawParityLagSlack < delta.position) {
                    state.lagFrames++;
                    if (state.lagFrames == kRawParityLagFrames) {
                        ROCK_LOG_WARN(Hand, "{} hand parity suggests possible one-frame lag: currentDelta={:.3f} prevApiDelta={:.3f}", handLabel, delta.position,
                            prevApiDelta.position);
                    }
                } else {
                    state.lagFrames = 0;
                }
            } else {
                state.lagFrames = 0;
            }

            state.previousApiTransform = apiTransform;
            state.hasPreviousApiTransform = true;

            if (emitSummary) {
                const char* summaryHandLabel = isLeft ? "L" : "R";
                ROCK_LOG_DEBUG(Hand, "{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)", summaryHandLabel,
                    delta.position, delta.rotationDegrees, measurePointDelta(localPalmPosition, apiPalmPosition), measureDirectionDeltaDegrees(localPalmNormal, apiPalmNormal),
                    measureDirectionDeltaDegrees(localPointing, apiPointing));
            }
        };

        sampleHand(false);
        sampleHand(true);

        if (emitSummary) {
            _paritySummaryCounter = 0;
            const auto& right = _rawHandParityStates[0];
            const auto& left = _rawHandParityStates[1];
            ROCK_LOG_DEBUG(Hand, "Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)", right.lastPositionDelta, right.lastRotationDeltaDegrees,
                left.lastPositionDelta, left.lastRotationDeltaDegrees);
        }
    }

#include "physics-interaction/core/PhysicsInteractionFrame.inl"

    void PhysicsInteraction::synchronizeNativeScopePresentationAfterFrikUpdate()
    {
        if (!_initialized.load(std::memory_order_acquire) || !runtime_state::isLocalSkeletonReady()) {
            return;
        }

        auto* weaponNode = f4vr::getWeaponNode();
        _twoHandedGrip.synchronizeNativeScopePresentationAfterFrikUpdate(weaponNode, _weaponCollision.getCurrentWeaponGenerationKey());
    }

    void PhysicsInteraction::finalizeInteractionFrame(
        const PhysicsFrameContext& frame,
        RE::bhkWorld* bhk,
        RE::hknpWorld* hknp,
        const EquippedWeaponFrameResult& equippedWeaponFrame)
    {
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
                _rightWeaponSupportCollisionSuppressed.load(
                    std::memory_order_acquire),
            leftSupportGripActive ||
                (_twoHandedGrip.isFiringHandLeft() &&
                    _twoHandedGrip.isFiringGripOccupied()) ||
                _leftWeaponSupportCollisionSuppressed.load(
                    std::memory_order_acquire),
            _dynamicWeaponCollision.proxyBodyIdForDebug().value,
            _rightHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(false),
            _leftHand.isGrabVisualReturnActive() || _twoHandedGrip.isHandVisualReturnActive(true));
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
                dynamic_hand_collision_policy::kHapticDurationSeconds,
                pulse.intensity);
        }
        updateFeedbackHaptics(frame.deltaSeconds);

        updateAuthoredSupportGripIndicator();
        publishDebugBodyOverlay(frame);

        resolveContacts(frame);

        bool wasTouchingR = _rightHand.isTouching();
        bool wasTouchingL = _leftHand.isTouching();
        const float measuredFrameDeltaSeconds =
            frame.timing.valid ? frame.timing.deltaSeconds : 0.0f;
        _rightHand.tickTouchState(measuredFrameDeltaSeconds);
        _leftHand.tickTouchState(measuredFrameDeltaSeconds);
        _rightHand.tickSemanticContactState(measuredFrameDeltaSeconds);
        _leftHand.tickSemanticContactState(measuredFrameDeltaSeconds);
        _handContactActivity.advanceFrame(measuredFrameDeltaSeconds);
        if (wasTouchingR && !_rightHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, false, _rightHand.getLastTouchedRef(), _rightHand.getLastTouchedFormID(), _rightHand.getLastTouchedLayer());
        }
        if (wasTouchingL && !_leftHand.isTouching()) {
            dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, true, _leftHand.getLastTouchedRef(), _leftHand.getLastTouchedFormID(), _leftHand.getLastTouchedLayer());
        }

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
                "TIMING game: seq={} dt={:.6f} hz={:.1f} valid={} paused={} elapsed={:.2f}s disc={} invalid={} | physics: step={} solve={} rawDt={:.6f} subDt={:.6f} substeps={} hz={:.1f} fallback={} fallbackCount={} simulated={:.2f}s | phaseIdentity={} | grabR: hz={:.1f} scale={:.3f} srcInt={:.4f} | grabL: hz={:.1f} scale={:.3f} srcInt={:.4f}",
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
                rightGrabClock.physicsRateForceScale,
                rightGrabClock.sourceIntervalSeconds,
                leftGrabClock.physicsHz,
                leftGrabClock.physicsRateForceScale,
                leftGrabClock.sourceIntervalSeconds);
        }

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
        // Publish callback ownership only after every main-thread collider
        // mutation and target update for this frame has committed.
        _generatedBodyStepDrive.registerForNextStep(bhk, hknp);
    }

    void PhysicsInteraction::update()
    {
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        _equippedWeaponShoulderGestureConsumedThisFrame = {};
        _equippedWeaponToggleGrabReleasePressConsumedThisFrame = {};
        const auto& runtime = runtime_state::currentFrame();
        const auto retireDynamicWeaponForInterruptedFrame = [this]() {
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
        };
        refreshEquippedWeaponHandlingSettings();
        if (!runtime.visualAuthorityAvailable) {
            retireDynamicWeaponForInterruptedFrame();
            _authoredSupportGripIndicator.hide();
            restoreHeldMassMovementSlowdown("frik-unavailable");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            return;
        }

        _twoHandedGrip.beginWeaponCollisionPresentationFrame();

        // ROCK always binds raw controller identity physically: right is the
        // primary wand and left is the secondary wand. Weapon handedness is a
        // separate ROCK role and never remaps buttons/controllers.
        vrcf::VRControllers.update(false);

        // Before any early return below: a skipped consume would let a stale
        // accept-button press replay as a reload frames later (see the API doc).
        input_remap_runtime::updateFiringHandReloadInput(runtime.deltaSeconds);

        /*
         * runtime.deltaSeconds is the central sanitized game delta; no local
         * resanitization. Consumers migrate to runtime.timing individually
         * with explicit invalid-sample handling.
         */
        _deltaTime = runtime.deltaSeconds;
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
            return;
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
                        restoreRightHandCollisionAfterDominantWeapon(hknpMenu);
                        restoreHandCollisionAfterWeaponSupport(hknpMenu, true, true);
                        restoreHandCollisionAfterWeaponSupport(hknpMenu, false, true);
                        restoreHandCollisionAfterEquippedWeaponDrop(hknpMenu, false);
                        restoreHandCollisionAfterEquippedWeaponDrop(hknpMenu, true);
                        if (_rightHand.isHolding()) {
                            auto* r = _rightHand.getHeldRef();
                            _rightHand.releaseGrabbedObject(hknpMenu, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(_rightHand, false));
                            if (r)
                                releaseObject(r, PhysicsObjectClaimOwner::RightHand);
                        }
                        if (_leftHand.isHolding()) {
                            auto* r = _leftHand.getHeldRef();
                            _leftHand.releaseGrabbedObject(hknpMenu, GrabReleaseCollisionRestoreMode::Delayed, makeGrabReleaseContext(_leftHand, true));
                            if (r)
                                releaseObject(r, PhysicsObjectClaimOwner::LeftHand);
                        }
                    }
                } else {
                    _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
                    _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
                    _rightWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
                    _rightDominantWeaponCollisionSuppression.clearTracking();
                    _leftWeaponSupportCollisionSuppression.clearTracking();
                    _rightWeaponSupportCollisionSuppression.clearTracking();
                    clearEquippedWeaponPostDropCollisionSuppressionState();
                }
            }
            debug::ClearFrame();
            _authoredSupportGripIndicator.hide();
            clearEquippedWeaponFiringGripInputState();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            auto* snapshotBhk = getPlayerBhkWorld();
            auto* snapshotHknp = snapshotBhk ? getHknpWorld(snapshotBhk) : nullptr;
            if (snapshotBhk && snapshotHknp) {
                _dynamicWorldCarCollision.restoreAll(snapshotBhk, snapshotHknp, "menu-blocked");
            } else {
                _dynamicWorldCarCollision.abandon();
            }
            observeLifecycleFrame(snapshotBhk, snapshotHknp, ::rock::provider::RockProviderLifecycleReason::MenuBlocked);
            restoreHeldMassMovementSlowdown("menu-blocked");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            _dynamicWorldCarCollision.abandon();
            if (_initialized) {
                ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
                shutdown();
            }
            return;
        }

        if (_initialized && bhk != _cachedBhkWorld) {
            ROCK_LOG_INFO(Update, "bhkWorld changed (cell transition) — reinitializing");

            shutdown();
        }

        if (!_initialized) {
            init();
            if (!_initialized) {
                return;
            }
        }

        _cachedBhkWorld = bhk;

        auto* hknp = getHknpWorld(bhk);
        if (!hknp) {
            _dynamicWorldCarCollision.abandon();
            _cachedHknpWorld = nullptr;
            observeLifecycleFrame(bhk, nullptr, ::rock::provider::RockProviderLifecycleReason::WorldUnavailable);
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            debug::ClearFrame();
            _authoredSupportGripIndicator.hide();
            restoreHeldMassMovementSlowdown("world-unavailable");
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
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
            auto releaseHeldForScaleChange = [&](Hand& hand, bool isLeft) {
                if (!hand.isHolding()) {
                    return;
                }

                auto* heldRef = hand.getHeldRef();
                hand.releaseGrabbedObject(hknp, GrabReleaseCollisionRestoreMode::Immediate, makeGrabReleaseContext(hand, isLeft));
                if (heldRef) {
                    releaseObject(heldRef, claimOwnerForHand(isLeft));
                }
            };
            releaseHeldForScaleChange(_rightHand, false);
            releaseHeldForScaleChange(_leftHand, true);

            restoreRightHandCollisionAfterDominantWeapon(hknp);
            restoreHandCollisionAfterWeaponSupport(hknp, true, true);
            restoreHandCollisionAfterWeaponSupport(hknp, false, true);
            restoreHandCollisionAfterEquippedWeaponDrop(hknp, false);
            restoreHandCollisionAfterEquippedWeaponDrop(hknp, true);
            _twoHandedGrip.reset();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _bodyContactRuntime.reset();
            clearLeftWeaponContact();
            clearRightWeaponContact();

            destroyHandCollisions(bhk);
            destroyBodyBoneCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
            markGeneratedBodiesInvalidated();
            _rightDominantWeaponCollisionSuppression.clearTracking();
            _leftWeaponSupportCollisionSuppression.clearTracking();
            _rightWeaponSupportCollisionSuppression.clearTracking();
            _rightDominantWeaponCollisionSuppressed.store(false, std::memory_order_release);
            _leftWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            _rightWeaponSupportCollisionSuppressed.store(false, std::memory_order_release);
            clearEquippedWeaponPostDropCollisionSuppressionState();
            restoreNativePlayerCollisionSuppression(hknp, "scale-change");
            _nativePlayerCollisionSuppressionRefreshFrames = 0;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        refreshHandBoneCache();
        sampleHandTransformParity();
        const auto frame = buildFrameContext(bhk, hknp);
        _palmClockGameFrameIndex.store(runtime.frameIndex, std::memory_order_release);
        _palmClockGameDeltaSeconds.store(frame.deltaSeconds, std::memory_order_release);
        observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::None);
        if (!generatedBodiesMatchLifecycle(bhk, hknp)) {
            const bool rebuilt =
                !frame.reloadBoundaryActive &&
                rebuildGeneratedBodiesForLifecycle(bhk, hknp, "epoch-mismatch");
            if (rebuilt) {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesRebuilt);
            } else {
                observeLifecycleFrame(bhk, hknp, ::rock::provider::RockProviderLifecycleReason::GeneratedBodiesInvalidated);
                ROCK_LOG_SAMPLE_DEBUG(Update,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "ROCK lifecycle generated-body rebuild pending: animationBoundary={} flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                    frame.reloadBoundaryActive ? "yes" : "no",
                    _lifecycleFlagsAtomic.load(std::memory_order_acquire),
                    _lastLifecycleReasonAtomic.load(std::memory_order_acquire),
                    _worldGenerationAtomic.load(std::memory_order_acquire),
                    _skeletonGenerationAtomic.load(std::memory_order_acquire),
                    _providerGenerationAtomic.load(std::memory_order_acquire),
                    _stableFrameCountAtomic.load(std::memory_order_acquire));
                debug::ClearFrame();
                _twoHandedGrip.reset();
                _authoredSupportGripIndicator.hide();
                _pendingEquippedWeaponPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
                _shoulderStashStates = {};
                _mouthConsumeStates = {};
                _feedbackHaptics.reset();
                ::rock::provider::dispatchFrameCallbacks(*this);
                return;
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
            _twoHandedGrip.reset();
            _authoredSupportGripIndicator.hide();
            _pendingEquippedWeaponPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _shoulderStashStates = {};
            _mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        const bool forceBareFistRecheck = _equippedWeaponMenuReconcilePending;
        if (_equippedWeaponMenuReconcilePending) {
            const bool firingHandIsLeft =
                _twoHandedGrip.isFiringGripOccupied() &&
                _twoHandedGrip.isFiringHandLeft();
            const auto detachDecision =
                resolveEquippedWeaponDetachDecision(
                    _equippedWeaponHandlingSettings);
            const bool primaryGrabHeld = input_remap_runtime::isRawButtonPhysicallyHeld(
                firingHandIsLeft,
                input_remap_policy::kGrabButtonId);
            _pendingEquippedWeaponPrimaryOnlyGripStart = PendingEquippedWeaponPrimaryOnlyGripStart{
                .pending = detachDecision.primaryDetachEnabled &&
                    primaryGrabHeld,
                .isLeft = firingHandIsLeft,
                .toggleAcquisitionCommitted =
                    _equippedWeaponHandlingSettings.toggleGrabEnabled &&
                    primaryGrabHeld,
            };
            _equippedWeaponMenuReconcilePending = false;
            ROCK_LOG_DEBUG(Weapon,
                "Equipped weapon ownership reconciled after menu: primaryGrabHeld={} pendingPrimaryOnlyStart={}",
                primaryGrabHeld ? "yes" : "no",
                _pendingEquippedWeaponPrimaryOnlyGripStart.pending ? "yes" : "no");
        }
        enforceNoBareFistState(forceBareFistRecheck);

        if (_collisionLayerRegistered &&
            (_expectedHandLayerMask != 0 || _expectedWeaponLayerMask != 0 || _expectedReloadLayerMask != 0 || _expectedBodyLayerMask != 0 ||
                _expectedDynamicHandProxyLayerMask != 0 || _expectedDynamicWeaponProxyLayerMask != 0 ||
                _expectedDynamicWorldCarClutterLayerMask != 0 || _expectedDynamicWorldCarLargeClutterLayerMask != 0 ||
                _nativeCharacterControllerLayerPolicyCaptured)) {
            const auto desiredHandMask = collision_layer_policy::buildRockHandExpectedMask(true, g_rockConfig.rockHandCollisionStaticWorldEnabled);
            const auto desiredWeaponMask = collision_layer_policy::buildRockWeaponExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                true);
            const auto desiredReloadMask = collision_layer_policy::buildRockReloadExpectedMask(
                g_rockConfig.rockWeaponCollisionBlocksProjectiles,
                g_rockConfig.rockWeaponCollisionBlocksSpells,
                g_rockConfig.rockHandCollisionStaticWorldEnabled);
            const auto desiredBodyMask = collision_layer_policy::buildRockBodyExpectedMask();
            const auto desiredDynamicRightHandProxyMask =
                collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                    false);
            const auto desiredDynamicLeftHandProxyMask =
                collision_layer_policy::buildRockDynamicHandProxyExpectedMask(
                    true);
            const auto desiredDynamicWeaponProxyMask =
                collision_layer_policy::buildRockDynamicWeaponProxyExpectedMask();
            const bool desiredNativeControllerPolicyEnabled = g_rockConfig.rockNativeCharacterControllerObjectContactFilterEnabled;
            const bool nativeControllerPolicyModeChanged =
                _nativeCharacterControllerLayerPolicyCaptured &&
                _nativeCharacterControllerLayerPolicyEnabled != desiredNativeControllerPolicyEnabled;
            if (!collision_layer_policy::matrixLayerMaskMatches(_expectedHandLayerMask, desiredHandMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedWeaponLayerMask, desiredWeaponMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedReloadLayerMask, desiredReloadMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_expectedBodyLayerMask, desiredBodyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicHandProxyLayerMask,
                    desiredDynamicRightHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicLeftHandProxyLayerMask,
                    desiredDynamicLeftHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _expectedDynamicWeaponProxyLayerMask,
                    desiredDynamicWeaponProxyMask) ||
                nativeControllerPolicyModeChanged) {
                ROCK_LOG_INFO(Config, "ROCK collision layer config changed; re-registering matrix policy");
                _collisionLayerRegistered = false;
                registerCollisionLayer(hknp);
            }

            if (auto* matrix = havok_runtime::getCollisionFilterMatrix(hknp)) {
                const auto currentHandMask = matrix[collision_layer_policy::ROCK_LAYER_HAND];
                const auto currentWeaponMask = matrix[collision_layer_policy::ROCK_LAYER_WEAPON];
                const auto currentReloadMask = matrix[collision_layer_policy::ROCK_LAYER_RELOAD];
                const auto currentBodyMask = matrix[collision_layer_policy::ROCK_LAYER_BODY];
                const auto currentDynamicHandProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_HAND_PROXY];
                const auto currentDynamicLeftHandProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY];
                const auto currentDynamicWeaponProxyMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WEAPON_PROXY];
                const auto currentDynamicWorldCarClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER];
                const auto currentDynamicWorldCarLargeClutterMask = matrix[collision_layer_policy::ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER];
                const bool handMaskDrifted = _expectedHandLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentHandMask, _expectedHandLayerMask);
                const bool weaponMaskDrifted = _expectedWeaponLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentWeaponMask, _expectedWeaponLayerMask);
                const bool reloadMaskDrifted = _expectedReloadLayerMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentReloadMask, _expectedReloadLayerMask);
                const bool bodyMaskDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::bodyManagedLayerMaskMatches(currentBodyMask, _expectedBodyLayerMask);
                const bool dynamicHandProxyMaskDrifted = _expectedDynamicHandProxyLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicHandProxyMask, _expectedDynamicHandProxyLayerMask);
                const bool dynamicLeftHandProxyMaskDrifted =
                    _expectedDynamicLeftHandProxyLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(
                        currentDynamicLeftHandProxyMask,
                        _expectedDynamicLeftHandProxyLayerMask);
                const bool dynamicWeaponProxyMaskDrifted = _expectedDynamicWeaponProxyLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWeaponProxyMask, _expectedDynamicWeaponProxyLayerMask);
                const bool dynamicWorldCarClutterMaskDrifted = _expectedDynamicWorldCarClutterLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarClutterMask, _expectedDynamicWorldCarClutterLayerMask);
                const bool dynamicWorldCarLargeClutterMaskDrifted = _expectedDynamicWorldCarLargeClutterLayerMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarLargeClutterMask, _expectedDynamicWorldCarLargeClutterLayerMask);
                const bool actorToolPairsDrifted =
                    _expectedHandLayerMask != 0 && _expectedWeaponLayerMask != 0 &&
                    !collision_layer_policy::rockToolActorPairsMatch(matrix, _expectedHandLayerMask, _expectedWeaponLayerMask);
                const bool bodyPairsDrifted = _expectedBodyLayerMask != 0 && !collision_layer_policy::rockBodyManagedPairsMatch(matrix, _expectedBodyLayerMask);
                const bool nativeControllerObjectPairsDrifted =
                    _nativeCharacterControllerLayerPolicyCaptured &&
                    !collision_layer_policy::nativeCharacterControllerObjectPairsMatch(matrix, _expectedNativeCharacterControllerLayerMask);
                if (handMaskDrifted || weaponMaskDrifted || reloadMaskDrifted || bodyMaskDrifted || dynamicHandProxyMaskDrifted || dynamicLeftHandProxyMaskDrifted || dynamicWeaponProxyMaskDrifted ||
                    dynamicWorldCarClutterMaskDrifted || dynamicWorldCarLargeClutterMaskDrifted || actorToolPairsDrifted || bodyPairsDrifted ||
                    nativeControllerObjectPairsDrifted) {
                    const auto currentNativeCharacterControllerMask =
                        _nativeCharacterControllerLayerPolicyCaptured ? matrix[collision_layer_policy::FO4_LAYER_CHARCONTROLLER] : 0;
                    ROCK_LOG_WARN(Config,
                        "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}, reload expected=0x{:016X} current=0x{:016X}, body expected=0x{:016X} current=0x{:016X}, dynamicRightHandProxy expected=0x{:016X} current=0x{:016X}, dynamicLeftHandProxy expected=0x{:016X} current=0x{:016X}, dynamicWeaponProxy expected=0x{:016X} current=0x{:016X}, carClutter expected=0x{:016X} current=0x{:016X}, carLarge expected=0x{:016X} current=0x{:016X}, nativeController expected=0x{:016X} current=0x{:016X}, actorToolPairs={}, bodyManagedPairs={}, nativeControllerObjects={}; re-registering",
                        collision_layer_policy::matrixAddressableMask(_expectedHandLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentHandMask),
                        collision_layer_policy::matrixAddressableMask(_expectedWeaponLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentWeaponMask),
                        collision_layer_policy::matrixAddressableMask(_expectedReloadLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentReloadMask),
                        collision_layer_policy::matrixAddressableMask(_expectedBodyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentBodyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicHandProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicLeftHandProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicLeftHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWeaponProxyLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWeaponProxyMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWorldCarClutterLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarClutterMask),
                        collision_layer_policy::matrixAddressableMask(_expectedDynamicWorldCarLargeClutterLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarLargeClutterMask),
                        collision_layer_policy::matrixAddressableMask(_expectedNativeCharacterControllerLayerMask),
                        collision_layer_policy::matrixAddressableMask(currentNativeCharacterControllerMask),
                        actorToolPairsDrifted ? "drifted" : "ok",
                        bodyPairsDrifted ? "drifted" : "ok",
                        nativeControllerObjectPairsDrifted ? "drifted" : "ok");
                    _collisionLayerRegistered = false;
                    registerCollisionLayer(hknp);
                }
            }
        }

        const auto equippedWeaponFrame = updateEquippedWeaponFrame(frame, bhk, hknp);
        finalizeInteractionFrame(frame, bhk, hknp, equippedWeaponFrame);
    }

    void PhysicsInteraction::dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft, RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
    {
        PhysicsEventData data{ isLeft, refr, formID, layer };

        if (auto* m = ::rock::getROCKMessaging()) {
            m->Dispatch(msgType, &data, sizeof(data), nullptr);
        }
    }

    void PhysicsInteraction::onGeneratedColliderPhysicsSubstep(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        auto* self = static_cast<PhysicsInteraction*>(userData);
        if (!self) {
            return;
        }

        /*
         * This pre-collide callback runs inside the same native Havok step path
         * used for generated body writes. It reasserts bit 14 only while the
         * cached native identity still owns the lease. This callback only reads
         * the cache; game-frame refresh owns stale-lease eviction under the
         * callback quiescence gate.
         */
        self->refreshNativePlayerCollisionSuppressionFromPhysicsSubstep(world, "native-player-body-pre-collide");

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

        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        _rightHand.flushPendingCollisionPhysicsDrive(world, timing);
        _leftHand.flushPendingCollisionPhysicsDrive(world, timing);
        _bodyBoneColliders.flushPendingPhysicsDrive(world, timing);
        _weaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicHandCollision.flushPendingPhysicsDrive(world, timing);
        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-collider-drive", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-collider-drive", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
    }

    void PhysicsInteraction::driveCustomGrabAuthorityFromBetweenStep(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-between-before-grab-flush", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-between-before-grab-flush", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        _rightHand.flushPendingCustomGrabAuthority(world, timing);
        _leftHand.flushPendingCustomGrabAuthority(world, timing);
    }

    void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!world || !_initialized.load(std::memory_order_acquire) || !physicsWritesAllowedForWorld(world)) {
            return;
        }

        const auto completedSolveSequence =
            _completedPhysicsSolveSequence.fetch_add(
                1,
                std::memory_order_release) +
            1;
        _rightHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _leftHand.observeCustomGrabAuthorityAfterSolve(world, timing);
        _dynamicWeaponCollision.samplePostSolve(
            world,
            completedSolveSequence);
        _dynamicHandCollision.samplePostSolveDeviations(world, timing);
        const auto gameFrameIndex = _palmClockGameFrameIndex.load(std::memory_order_acquire);
        debug::CapturePostSolveBodyPhases(
            world,
            timing,
            gameFrameIndex,
            completedSolveSequence);
        const auto gameDeltaSeconds = _palmClockGameDeltaSeconds.load(std::memory_order_acquire);
        logPalmClockSampleForHand("physics-after-solve", _rightHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        logPalmClockSampleForHand("physics-after-solve", _leftHand, world, nullptr, gameFrameIndex, gameDeltaSeconds, &timing);
        serviceRetiredGrabConstraintPayloads();
        _weaponCollision.serviceRetiredWeaponBodies(world);
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
                frame.deltaSeconds,
                rightHandContext);
            _leftHand.updateSelectionBeam(frame.hknpWorld, frame.left.grabAnchorWorld);
        } else {
            _leftHand.stopSelectionBeam();
        }
    }

    void PhysicsInteraction::restoreHeldMassMovementSlowdown(const char* reason)
    {
        if (_heldMassMovementSpeedReduction <= 0.0f) {
            return;
        }

        const float previousReduction = _heldMassMovementSpeedReduction;
        if (applyPlayerSpeedReduction(previousReduction, 0.0f)) {
            _heldMassMovementSpeedReduction = 0.0f;
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
            _heldMassMovementLogCounter = 0;
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
            _heldMassMovementFadeStartReduction = heldMassReduction;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        } else if (_heldMassMovementSpeedReduction > 0.0f) {
            if (_heldMassMovementFadeStartReduction <= 0.0f) {
                _heldMassMovementFadeStartReduction = _heldMassMovementSpeedReduction;
                _heldMassMovementFadeElapsedSeconds = 0.0f;
            }
            _heldMassMovementFadeElapsedSeconds += std::isfinite(deltaSeconds) ? (std::max)(0.0f, deltaSeconds) : 0.0f;
            targetReduction = held_mass_movement::computeFadeOutReduction(
                _heldMassMovementFadeStartReduction,
                _heldMassMovementFadeElapsedSeconds,
                movementConfig.fadeOutSeconds);
        } else {
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        }

        if (std::fabs(targetReduction - _heldMassMovementSpeedReduction) <= 0.001f &&
            (targetReduction > 0.0f || _heldMassMovementSpeedReduction <= 0.0f)) {
            return;
        }

        const float previousReduction = _heldMassMovementSpeedReduction;
        if (!applyPlayerSpeedReduction(previousReduction, targetReduction)) {
            ROCK_LOG_SAMPLE_WARN(Hand,
                300,
                "Held mass movement slowdown skipped: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                heldMass,
                previousReduction,
                targetReduction);
            return;
        }

        _heldMassMovementSpeedReduction = targetReduction;
        if (targetReduction <= 0.0f) {
            _heldMassMovementFadeStartReduction = 0.0f;
            _heldMassMovementFadeElapsedSeconds = 0.0f;
        }
        if (g_rockConfig.rockDebugGrabFrameLogging) {
            ++_heldMassMovementLogCounter;
            if (_heldMassMovementLogCounter >= 90 || heldMass <= 0.0f || previousReduction <= 0.0f) {
                _heldMassMovementLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held mass movement slowdown: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                    heldMass,
                    previousReduction,
                    targetReduction);
            }
        }
    }
}

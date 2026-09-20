#include "api/EventStreams.h"
#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteractionInternal.h"
#include "physics-interaction/weapon/telemetry/NativeScopeShotDiagnostics.h"
#include "physics-interaction/telemetry/DynamicColliderTrace.h"
#include "physics-interaction/telemetry/HeldRenderTrace.h"

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
        if (resolved) {
            _diagnostics.handCacheResolveLogCounter = 0;
        } else if (g_rockConfig.rockDebugHandTransformParity) {
            if (++_diagnostics.handCacheResolveLogCounter == 1 || _diagnostics.handCacheResolveLogCounter % 90 == 0) {
                ROCK_LOG_WARN(Hand, "HandBoneCache unresolved; raw parity sampling skipped this frame");
            }
        }

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
        if (g_rockConfig.rockDebugGrabFrameLogging) {
            _dynamicWeaponCollision.tracePresentedWeapon(resolveEquippedWeaponInteractionNode(), runtime_state::currentFrame().frameIndex);
        }
    }

    void PhysicsInteraction::traceHeldPresentationPhase(const char* phase)
    {
        const auto& runtime = runtime_state::currentFrame();
        // Before the next ROCK tick, the owner still contains the engine's
        // late scene write for the preceding publication. Pair it with that
        // frame's after-rock / after-world-final samples.
        const bool beforeRock = std::string_view(phase) == "before-rock";
        held_render_trace::recordPhase(beforeRock ? held_render_trace::Phase::BeforeRock :
            std::string_view(phase) == "after-rock" ? held_render_trace::Phase::AfterRock : held_render_trace::Phase::AfterWorldFinal,
            runtime.frameIndex);
        const auto presentationFrame = runtime.frameIndex > 0 ? runtime.frameIndex - (beforeRock ? 1u : 0u) : 0;
        if (!dynamic_collider_trace::presentationEnabled() ||
            !held_render_trace::sampleFrame(presentationFrame) ||
            !_lifecycle.initialized.load(std::memory_order_acquire) || !runtime.localSkeletonReady) return;
        auto* bhk = getPlayerBhkWorld();
        auto* world = bhk ? getHknpWorld(bhk) : nullptr;
        if (!world || world != _lifecycle.cachedHknpWorld || bhk != _lifecycle.cachedBhkWorld) return;

        // Borrow scene nodes only in this game-thread callback. The logs carry
        // values from the same phase, including the actual physics target age.
        for (Hand* hand : { &_rightHand, &_leftHand }) {
            if (!hand->isHolding()) continue;
            const bool left = hand->isLeft();
            GrabPresentationNodeDebugSnapshot nodes{};
            GrabAuthorityProxyClockDebugSnapshot clock{};
            GrabOverlayPointProbeSample applied{};
            const bool nodesValid = hand->getGrabPresentationNodeDebugSnapshot(nodes);
            const bool clockValid = hand->tryGetGrabAuthorityProxyClockDebugSnapshot(world, clock);
            const bool appliedValid = hand->tryGetGrabOverlayPointProbeSample(world, applied);
            RE::NiTransform raw{}, claim{}, presented{}, solved{};
            const bool rawValid = frik_hand_world_authority::tryGetRawHandWorld(left, raw);
            const bool claimValid = frik_hand_world_authority::tryGetPublishedHandWorld(left, claim);
            const bool presentationValid = appliedValid && held_scene_presentation::tryGetPresentedBodyWorld(
                world, applied.objectBodyId.value, presentationFrame, presented);
            const bool solvedValid = appliedValid && havok_runtime::tryGetBodyArrayWorldTransform(world, applied.objectBodyId, solved);
            frik_visual_authority::ArmChainTransforms arm{};
            const bool wristValid = frik_visual_authority::tryGetArmChain(frik_visual_authority::handFromBool(left), arm) &&
                (arm.validMask & (1u << 6)) != 0;
            dynamic_collider_trace::write(
                "HELD_PHASE phase={} frame={} presentationFrame={} trace={} hand={} body={} moving={} room=({:.3f},{:.3f},{:.3f}) roomStep=({:.3f},{:.3f},{:.3f}) nodes={} clock={} queued={} applied={} flush={} raw={} claim={} wrist={} presented={} solved={} contact={} kind={} drive={} looseWeapon={}",
                phase, runtime.frameIndex, presentationFrame, nodes.traceId, left ? "left" : "right", applied.objectBodyId.value,
                runtime.playerSpace.moving, runtime.playerSpace.world.translate.x, runtime.playerSpace.world.translate.y, runtime.playerSpace.world.translate.z,
                runtime.playerSpace.deltaGameUnits.x, runtime.playerSpace.deltaGameUnits.y, runtime.playerSpace.deltaGameUnits.z,
                nodesValid, clockValid, clock.queuedSequence, clock.hasAppliedTarget, clock.flushSequence,
                rawValid, claimValid, wristValid, presentationValid, solvedValid, hand->isHeldBodyColliding(),
                grab_target::name(nodes.targetKind), held_object_drive_policy::modeName(nodes.driveMode), nodes.looseWeapon);
            const auto pose = [&](const char* label, bool valid, const RE::NiTransform& value) {
                if (!valid) return;
                const auto& t = value.translate;
                const auto& r = value.rotate.entry;
                dynamic_collider_trace::write(
                    "HELD_PHASE_POSE phase={} frame={} trace={} hand={} label={} T=({:.4f},{:.4f},{:.4f}) S={:.6f} R=({:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f};{:.6f},{:.6f},{:.6f})",
                    phase, runtime.frameIndex, nodes.traceId, left ? "left" : "right", label, t.x, t.y, t.z, value.scale,
                    r[0][0], r[0][1], r[0][2], r[1][0], r[1][1], r[1][2], r[2][0], r[2][1], r[2][2]);
            };
            pose("raw", rawValid, raw);
            pose("claim", claimValid, claim);
            pose("wrist", wristValid, arm.hand);
            pose("queued-proxy", clockValid && clock.hasQueuedTarget, clock.queuedProxyTargetWorld);
            pose("applied-proxy", clockValid && clock.hasAppliedTarget, clock.appliedProxyTargetWorld);
            pose("presented-body", presentationValid, presented);
            pose("solved-body", solvedValid, solved);
            pose("owner", nodes.collisionOwner.valid, nodes.collisionOwner.world);
            pose("root", nodes.referenceRoot.valid, nodes.referenceRoot.world);
            pose("root-previous", nodes.referenceRoot.valid, nodes.referenceRoot.previousWorld);
            pose("mesh", nodes.visibleGeometry.valid, nodes.visibleGeometry.world);
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

    void PhysicsInteraction::sampleHandTransformParity()
    {
        if (!g_rockConfig.rockDebugHandTransformParity) {
            _diagnostics.parityEnabledLogged = false;
            _diagnostics.paritySummaryCounter = 0;
            return;
        }

        if (!frik_visual_authority::isAvailable() || !_handBoneCache.isReady()) {
            return;
        }

        if (!_diagnostics.parityEnabledLogged) {
            ROCK_LOG_INFO(Init, "Hand-transform parity enabled (root flattened cache vs FRIK API, pre-write sampling)");
            _diagnostics.parityEnabledLogged = true;
        }

        const bool playerMoving = runtime_state::currentFrame().playerSpace.moving;
        const bool emitSummary = (++_diagnostics.paritySummaryCounter >= kRawParitySummaryFrames);

        auto sampleHand = [&](bool isLeft) {
            auto& state = _diagnostics.rawHandParityStates[isLeft ? 1 : 0];
            const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
            RE::NiTransform apiTransform{};
            if (!frik_visual_authority::tryGetPresentedHandWorldTransform(isLeft, apiTransform)) {
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
            _diagnostics.paritySummaryCounter = 0;
            const auto& right = _diagnostics.rawHandParityStates[0];
            const auto& left = _diagnostics.rawHandParityStates[1];
            ROCK_LOG_DEBUG(Hand, "Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)", right.lastPositionDelta, right.lastRotationDeltaDegrees,
                left.lastPositionDelta, left.lastRotationDeltaDegrees);
        }
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
            native_scope_shot_diagnostics::clearPresentation();
            return;
        }
        auto* bhk = getPlayerBhkWorld();
        auto* hknp = bhk ? getHknpWorld(bhk) : nullptr;
        if (!bhk || bhk != _lifecycle.cachedBhkWorld || !hknp || hknp != _lifecycle.cachedHknpWorld) {
            debug::ClearFrame();
            native_scope_shot_diagnostics::clearPresentation();
            return;
        }
        // AfterWorldFinal: provider animation, FRIK's claim re-solve and its
        // flattened bone rebuild have completed. Re-sample on the main thread;
        // PublishFrame copies values before the render thread consumes them.
        if (g_rockConfig.rockDebugNativeScopeShotAlignment) {
            auto* weapon = resolveEquippedWeaponInteractionNode();
            const auto generation = _weaponCollision.getCurrentWeaponGenerationKey();
            bool contact = false;
            const bool contactKnown = _dynamicWeaponCollision.tryGetContactState(weapon, generation, contact);
            native_scope_shot_diagnostics::publishPresentation(weapon, generation,
                _weaponCollision.getCurrentObservedEquippedWeaponFormID(), contactKnown, contact,
                _dynamicWeaponCollision.hasLatchedSurfaceSupport(reinterpret_cast<std::uintptr_t>(weapon), generation));
        }
        publishDebugBodyOverlay(buildFrameContext(bhk, hknp));
    }

    void PhysicsInteraction::finalizeInteractionFrame(
        const PhysicsFrameContext& frame,
        RE::bhkWorld* bhk,
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
                _suppression.rightWeaponSupportSuppressed.load(
                    std::memory_order_acquire),
            leftSupportGripActive ||
                (_twoHandedGrip.isFiringHandLeft() &&
                    _twoHandedGrip.isFiringGripOccupied()) ||
                _suppression.leftWeaponSupportSuppressed.load(
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

        _frame.debugOverlayFrameIndex = runtime_state::currentFrame().frameIndex;

        resolveContacts(frame);

        bool wasTouchingR = _rightHand.isTouching();
        bool wasTouchingL = _leftHand.isTouching();
        const float measuredFrameDeltaSeconds =
            frame.timing.valid ? frame.timing.deltaSeconds : 0.0f;
        _rightHand.tickTouchState(measuredFrameDeltaSeconds);
        _leftHand.tickTouchState(measuredFrameDeltaSeconds);
        _rightHand.tickSemanticContactState(measuredFrameDeltaSeconds);
        _leftHand.tickSemanticContactState(measuredFrameDeltaSeconds);
        _contacts.handActivity.advanceFrame(measuredFrameDeltaSeconds);
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
        // Publish callback ownership only after every main-thread collider
        // mutation and target update for this frame has committed.
        _generatedBodyStepDrive.registerForNextStep(bhk, hknp);
    }

    void PhysicsInteraction::update()
    {
        auto cancelInterruptedFists = F4SE::stl::scope_exit([this] {
            cancelBareFistMode("interaction-frame-interrupted");
        });
        _frame.debugOverlayFrameIndex = 0;
        ensureWeaponCollisionWorkbenchExitMenuSinkRegistered();

        _equipped.shoulderGestureConsumedThisFrame = {};
        _equipped.toggleGrabReleasePressConsumedThisFrame = {};
        _equipped.holsterInputConsumedThisFrame = {};
        const auto& runtime = runtime_state::currentFrame();
        if (!_suppression.nativeGrenadeLeases.empty()) {
            auto* bhk = getPlayerBhkWorld();
            auto* world = bhk ? getHknpWorld(bhk) : nullptr;
            if (bhk == _lifecycle.cachedBhkWorld && world == _lifecycle.cachedHknpWorld)
                updateNativeGrenadeCollisionSuppression(world, runtime.deltaSeconds);
        }
        _dynamicWeaponCollision.updateSurfaceSupportInput();
        const auto retireDynamicWeaponForInterruptedFrame = [this](bool preserveSurfaceSupport = false) {
            if (!_lifecycle.initialized.load(std::memory_order_acquire)) {
                return;
            }
            auto* currentBhk = getPlayerBhkWorld();
            auto* currentHknp = currentBhk ? getHknpWorld(currentBhk) : nullptr;
            if (currentBhk && currentBhk == _lifecycle.cachedBhkWorld &&
                currentHknp && currentHknp == _lifecycle.cachedHknpWorld) {
                _dynamicWeaponCollision.retireAll(currentBhk, preserveSurfaceSupport);
            } else {
                _dynamicWeaponCollision.abandonHavokStateAfterWorldLoss();
            }
        };
        refreshEquippedWeaponHandlingSettings();
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
        vrcf::VRControllers.update(false);

        // Before any early return below: a skipped consume would let a stale
        // accept-button press replay as a reload frames later (see the API doc).
        input_remap_runtime::updateFiringHandReloadInput(runtime.deltaSeconds);

        /*
         * runtime.deltaSeconds is the central sanitized game delta; no local
         * resanitization. Consumers migrate to runtime.timing individually
         * with explicit invalid-sample handling.
         */
        _frame.deltaTime = runtime.deltaSeconds;
        enforceNativeGrabHapticRuntimeSuppression();
        _contacts.dynamicPushElapsedSeconds += _frame.deltaTime;
        if (_contacts.dynamicPushCooldownUntil.size() > 512) {
            for (auto it = _contacts.dynamicPushCooldownUntil.begin(); it != _contacts.dynamicPushCooldownUntil.end();) {
                if (it->second <= _contacts.dynamicPushElapsedSeconds) {
                    it = _contacts.dynamicPushCooldownUntil.erase(it);
                } else {
                    ++it;
                }
            }
        }

        if (!runtime.localSkeletonReady) {
            if (_lifecycle.initialized) {
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
            retireDynamicWeaponForInterruptedFrame(true);
            _equipped.menuReconcilePending = true;
            if (_lifecycle.initialized) {
                _twoHandedGrip.reset();
                _equipped.pendingPrimaryOnlyGripStart = {};
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
                            auto release = makeGrabReleaseContext(_rightHand, false);
                            release.reason = "blocking-menu-opened";
                            _rightHand.releaseGrabbedObject(hknpMenu, GrabReleaseCollisionRestoreMode::Delayed, release);
                            if (r)
                                releaseObject(r, PhysicsObjectClaimOwner::RightHand);
                        }
                        if (_leftHand.isHolding()) {
                            auto* r = _leftHand.getHeldRef();
                            auto release = makeGrabReleaseContext(_leftHand, true);
                            release.reason = "blocking-menu-opened";
                            _leftHand.releaseGrabbedObject(hknpMenu, GrabReleaseCollisionRestoreMode::Delayed, release);
                            if (r)
                                releaseObject(r, PhysicsObjectClaimOwner::LeftHand);
                        }
                    }
                } else {
                    _suppression.rightDominantSuppressed.store(false, std::memory_order_release);
                    _suppression.leftWeaponSupportSuppressed.store(false, std::memory_order_release);
                    _suppression.rightWeaponSupportSuppressed.store(false, std::memory_order_release);
                    _suppression.rightDominantLeases.clearTracking();
                    _suppression.leftWeaponSupportLeases.clearTracking();
                    _suppression.rightWeaponSupportLeases.clearTracking();
                    clearEquippedWeaponPostDropCollisionSuppressionState();
                }
            }
            debug::ClearFrame();
            clearEquippedWeaponFiringGripInputState();
            _equipped.pendingPrimaryOnlyGripStart = {};
            auto* snapshotBhk = getPlayerBhkWorld();
            auto* snapshotHknp = snapshotBhk ? getHknpWorld(snapshotBhk) : nullptr;
            if (snapshotBhk && snapshotHknp) {
                _dynamicWorldCarCollision.restoreAll(snapshotBhk, snapshotHknp, "menu-blocked");
            } else {
                _dynamicWorldCarCollision.abandon();
            }
            observeLifecycleFrame(snapshotBhk, snapshotHknp, ::rock::provider::RockProviderLifecycleReason::MenuBlocked);
            restoreHeldMassMovementSlowdown("menu-blocked");
            _grabInput.shoulderStashStates = {};
            _grabInput.mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        auto* bhk = getPlayerBhkWorld();
        if (!bhk) {
            _dynamicWorldCarCollision.abandon();
            if (_lifecycle.initialized) {
                ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
                shutdown();
            }
            return;
        }

        if (_lifecycle.initialized && bhk != _lifecycle.cachedBhkWorld) {
            ROCK_LOG_INFO(Update, "bhkWorld changed (cell transition) — reinitializing");

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
            _dynamicWorldCarCollision.abandon();
            _lifecycle.cachedHknpWorld = nullptr;
            observeLifecycleFrame(bhk, nullptr, ::rock::provider::RockProviderLifecycleReason::WorldUnavailable);
            _twoHandedGrip.reset();
            _equipped.pendingPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            debug::ClearFrame();
            restoreHeldMassMovementSlowdown("world-unavailable");
            _grabInput.shoulderStashStates = {};
            _grabInput.mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }
        _lifecycle.cachedHknpWorld = hknp;

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
            _equipped.pendingPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _contacts.bodyRuntime.reset();
            clearLeftWeaponContact();
            clearRightWeaponContact();

            destroyHandCollisions(bhk);
            destroyBodyBoneCollisions(bhk);
            _weaponCollision.invalidateForScaleChange(hknp);
            markGeneratedBodiesInvalidated();
            _suppression.rightDominantLeases.clearTracking();
            _suppression.leftWeaponSupportLeases.clearTracking();
            _suppression.rightWeaponSupportLeases.clearTracking();
            _suppression.rightDominantSuppressed.store(false, std::memory_order_release);
            _suppression.leftWeaponSupportSuppressed.store(false, std::memory_order_release);
            _suppression.rightWeaponSupportSuppressed.store(false, std::memory_order_release);
            clearEquippedWeaponPostDropCollisionSuppressionState();
            clearNativePlayerCollisionFilter(hknp);
            _suppression.nativePlayerRefreshFrames = 0;
            collision_suppression_registry::globalCollisionSuppressionRegistry().clear();
        }

        refreshHandBoneCache();
        sampleHandTransformParity();
        const auto frame = buildFrameContext(bhk, hknp);
        _frame.palmClockGameFrameIndex.store(runtime.frameIndex, std::memory_order_release);
        _frame.palmClockGameDeltaSeconds.store(frame.deltaSeconds, std::memory_order_release);
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
                    _lifecycle.flagsAtomic.load(std::memory_order_acquire),
                    _lifecycle.lastReasonAtomic.load(std::memory_order_acquire),
                    _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
                    _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire),
                    _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire),
                    _lifecycle.stableFrameCountAtomic.load(std::memory_order_acquire));
                debug::ClearFrame();
                _twoHandedGrip.reset();
                _equipped.pendingPrimaryOnlyGripStart = {};
                clearEquippedWeaponFiringGripInputState();
                _grabInput.shoulderStashStates = {};
                _grabInput.mouthConsumeStates = {};
                _feedbackHaptics.reset();
                ::rock::provider::dispatchFrameCallbacks(*this);
                return;
            }
        }

        if (!physicsWritesAllowedForWorld(hknp)) {
            ROCK_LOG_SAMPLE_DEBUG(Update,
                g_rockConfig.rockLogSampleMilliseconds,
                "ROCK lifecycle gate closed frame: flags=0x{:08X} reason={} worldGen={} skeletonGen={} providerGen={} stableFrames={}",
                _lifecycle.flagsAtomic.load(std::memory_order_acquire),
                _lifecycle.lastReasonAtomic.load(std::memory_order_acquire),
                _lifecycle.worldGenerationAtomic.load(std::memory_order_acquire),
                _lifecycle.skeletonGenerationAtomic.load(std::memory_order_acquire),
                _lifecycle.providerGenerationAtomic.load(std::memory_order_acquire),
                _lifecycle.stableFrameCountAtomic.load(std::memory_order_acquire));
            debug::ClearFrame();
            _twoHandedGrip.reset();
            _equipped.pendingPrimaryOnlyGripStart = {};
            clearEquippedWeaponFiringGripInputState();
            _grabInput.shoulderStashStates = {};
            _grabInput.mouthConsumeStates = {};
            _feedbackHaptics.reset();
            ::rock::provider::dispatchFrameCallbacks(*this);
            return;
        }

        const bool forceBareFistRecheck = _equipped.menuReconcilePending;
        if (_equipped.menuReconcilePending) {
            const bool firingHandIsLeft =
                _twoHandedGrip.isFiringGripOccupied() &&
                _twoHandedGrip.isFiringHandLeft();
            const auto detachDecision =
                resolveEquippedWeaponDetachDecision(
                    _equipped.handlingSettings);
            const bool primaryGrabHeld = input_remap_runtime::isRawButtonPhysicallyHeld(
                firingHandIsLeft,
                input_remap_policy::kGrabButtonId);
            _equipped.pendingPrimaryOnlyGripStart = PendingEquippedWeaponPrimaryOnlyGripStart{
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
                _equipped.pendingPrimaryOnlyGripStart.pending ? "yes" : "no");
        }
        updateBareFistMode(frame);
        enforceNoBareFistState(forceBareFistRecheck);

        if (_layers.registered &&
            (_layers.expectedHandMask != 0 || _layers.expectedWeaponMask != 0 || _layers.expectedReloadMask != 0 || _layers.expectedBodyMask != 0 ||
                _layers.expectedDynamicHandProxyMask != 0 || _layers.expectedDynamicWeaponProxyMask != 0 ||
                _layers.expectedDynamicWorldCarClutterMask != 0 || _layers.expectedDynamicWorldCarLargeClutterMask != 0)) {
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
            if (!collision_layer_policy::matrixLayerMaskMatches(_layers.expectedHandMask, desiredHandMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_layers.expectedWeaponMask, desiredWeaponMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_layers.expectedReloadMask, desiredReloadMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(_layers.expectedBodyMask, desiredBodyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _layers.expectedDynamicHandProxyMask,
                    desiredDynamicRightHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _layers.expectedDynamicLeftHandProxyMask,
                    desiredDynamicLeftHandProxyMask) ||
                !collision_layer_policy::matrixLayerMaskMatches(
                    _layers.expectedDynamicWeaponProxyMask,
                    desiredDynamicWeaponProxyMask)) {
                ROCK_LOG_INFO(Config, "ROCK collision layer config changed; re-registering matrix policy");
                _layers.registered = false;
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
                const bool handMaskDrifted = _layers.expectedHandMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentHandMask, _layers.expectedHandMask);
                const bool weaponMaskDrifted = _layers.expectedWeaponMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentWeaponMask, _layers.expectedWeaponMask);
                const bool reloadMaskDrifted = _layers.expectedReloadMask != 0 && !collision_layer_policy::matrixLayerMaskMatches(currentReloadMask, _layers.expectedReloadMask);
                const bool bodyMaskDrifted = _layers.expectedBodyMask != 0 && !collision_layer_policy::bodyManagedLayerMaskMatches(currentBodyMask, _layers.expectedBodyMask);
                const bool dynamicHandProxyMaskDrifted = _layers.expectedDynamicHandProxyMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicHandProxyMask, _layers.expectedDynamicHandProxyMask);
                const bool dynamicLeftHandProxyMaskDrifted =
                    _layers.expectedDynamicLeftHandProxyMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(
                        currentDynamicLeftHandProxyMask,
                        _layers.expectedDynamicLeftHandProxyMask);
                const bool dynamicWeaponProxyMaskDrifted = _layers.expectedDynamicWeaponProxyMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWeaponProxyMask, _layers.expectedDynamicWeaponProxyMask);
                const bool dynamicWorldCarClutterMaskDrifted = _layers.expectedDynamicWorldCarClutterMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarClutterMask, _layers.expectedDynamicWorldCarClutterMask);
                const bool dynamicWorldCarLargeClutterMaskDrifted = _layers.expectedDynamicWorldCarLargeClutterMask != 0 &&
                    !collision_layer_policy::matrixLayerMaskMatches(currentDynamicWorldCarLargeClutterMask, _layers.expectedDynamicWorldCarLargeClutterMask);
                const bool actorToolPairsDrifted =
                    _layers.expectedHandMask != 0 && _layers.expectedWeaponMask != 0 &&
                    !collision_layer_policy::rockToolActorPairsMatch(matrix, _layers.expectedHandMask, _layers.expectedWeaponMask);
                const bool bodyPairsDrifted = _layers.expectedBodyMask != 0 && !collision_layer_policy::rockBodyManagedPairsMatch(matrix, _layers.expectedBodyMask);
                if (handMaskDrifted || weaponMaskDrifted || reloadMaskDrifted || bodyMaskDrifted || dynamicHandProxyMaskDrifted || dynamicLeftHandProxyMaskDrifted || dynamicWeaponProxyMaskDrifted ||
                    dynamicWorldCarClutterMaskDrifted || dynamicWorldCarLargeClutterMaskDrifted || actorToolPairsDrifted || bodyPairsDrifted) {
                    ROCK_LOG_WARN(Config,
                        "ROCK configured layer mask drift detected; hand expected=0x{:016X} current=0x{:016X}, weapon expected=0x{:016X} current=0x{:016X}, reload expected=0x{:016X} current=0x{:016X}, body expected=0x{:016X} current=0x{:016X}, dynamicRightHandProxy expected=0x{:016X} current=0x{:016X}, dynamicLeftHandProxy expected=0x{:016X} current=0x{:016X}, dynamicWeaponProxy expected=0x{:016X} current=0x{:016X}, carClutter expected=0x{:016X} current=0x{:016X}, carLarge expected=0x{:016X} current=0x{:016X}, actorToolPairs={}, bodyManagedPairs={}; re-registering",
                        collision_layer_policy::matrixAddressableMask(_layers.expectedHandMask),
                        collision_layer_policy::matrixAddressableMask(currentHandMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedWeaponMask),
                        collision_layer_policy::matrixAddressableMask(currentWeaponMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedReloadMask),
                        collision_layer_policy::matrixAddressableMask(currentReloadMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedBodyMask),
                        collision_layer_policy::matrixAddressableMask(currentBodyMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedDynamicHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedDynamicLeftHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicLeftHandProxyMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedDynamicWeaponProxyMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWeaponProxyMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedDynamicWorldCarClutterMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarClutterMask),
                        collision_layer_policy::matrixAddressableMask(_layers.expectedDynamicWorldCarLargeClutterMask),
                        collision_layer_policy::matrixAddressableMask(currentDynamicWorldCarLargeClutterMask),
                        actorToolPairsDrifted ? "drifted" : "ok",
                        bodyPairsDrifted ? "drifted" : "ok");
                    _layers.registered = false;
                    registerCollisionLayer(hknp);
                }
            }
        }

        const auto equippedWeaponFrame = updateEquippedWeaponFrame(frame, bhk, hknp);
        finalizeInteractionFrame(frame, bhk, hknp, equippedWeaponFrame);
        if (input_remap_runtime::ownsBareFistInput() && !bareFistHandsAvailable(frame)) {
            cancelBareFistMode("hand-owner-changed");
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

        _rightHand.flushPendingCollisionPhysicsDrive(world, timing);
        _leftHand.flushPendingCollisionPhysicsDrive(world, timing);
        _bodyBoneColliders.flushPendingPhysicsDrive(world, timing);
        _weaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicWeaponCollision.flushPendingPhysicsDrive(world, timing);
        _dynamicHandCollision.flushPendingPhysicsDrive(world, timing);
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
            _diagnostics.heldMassLogCounter = 0;
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
        if (g_rockConfig.rockDebugGrabFrameLogging) {
            ++_diagnostics.heldMassLogCounter;
            if (_diagnostics.heldMassLogCounter >= 90 || heldMass <= 0.0f || previousReduction <= 0.0f) {
                _diagnostics.heldMassLogCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Held mass movement slowdown: heldMass={:.3f} previousReduction={:.2f} targetReduction={:.2f}",
                    heldMass,
                    previousReduction,
                    targetReduction);
            }
        }
    }
}

#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// ScopeMenu-safe hand frames and deferred scope hand-authority role clears.

namespace rock
{
    void TwoHandedGrip::refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        // A final trace is valid only for the same update that produced its
        // pre-solve sample. Early-return frames deliberately remain pre-only.
        _scope.transitionFinalTracePending = false;
        const bool activationStateChanged =
            frameInput.manualScopeActivationRequested !=
                _scope.manualActivationRequested ||
            frameInput.nativeScopeRequestStateValid != _scope.nativeRequestStateValid ||
            (frameInput.nativeScopeRequestStateValid &&
                frameInput.nativeScopeRequestActive != _scope.nativeRequestActive);
        _scope.manualActivationRequested =
            frameInput.manualScopeActivationRequested;
        _scope.nativeRequestStateValid = frameInput.nativeScopeRequestStateValid;
        _scope.nativeRequestActive = frameInput.nativeScopeRequestStateValid &&
                                    frameInput.nativeScopeRequestActive;
        _scope.activationDebugSnapshot = NativeScopeActivationDebugSnapshot{
            .publicationSequence =
                _scope.activationDebugSnapshot.publicationSequence + 1,
            .weaponGenerationKey = _scope.anchorGenerationKey,
            .anchorSource = _scope.anchorSource,
            .manualInputRequested = _scope.manualActivationRequested,
            .rendererStateValid = _scope.nativeRequestStateValid,
            .rendererActive = _scope.nativeRequestActive,
        };
        if (activationStateChanged) {
            ++_scope.transitionTraceSequence;
            _scope.transitionTraceFramesRemaining =
                SCOPE_TRANSITION_TRACE_FRAMES;
        }

        const bool scopeStateChanged = _scope.menuOpenThisFrame != frameInput.scopeMenuOpen;
        _scope.menuOpenThisFrame = frameInput.scopeMenuOpen;
        _scope.menuClosedThisFrame = scopeStateChanged && !_scope.menuOpenThisFrame;
        const bool driverFrameAuthorityWasActive = _scope.driverFrameAuthorityActive;
        _scope.driverFrameAuthorityActive = scope_safe_hand_frame_math::retainDriverFrameAuthority(
            _scope.menuOpenThisFrame,
            _scope.manualActivationRequested,
            isManualOwnershipActive(),
            driverFrameAuthorityWasActive);
        const bool driverFrameAuthorityStoppedThisFrame =
            driverFrameAuthorityWasActive && !_scope.driverFrameAuthorityActive;

        if (scopeStateChanged) {
            // Never resume a pre-menu visual interpolation after hFRIK restores
            // its visible body. The weapon solver itself remains continuous.
            resetLockedHandVisualLerp();
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: native scope hand-frame menu={} solver={} leftCache={} rightCache={}",
                _scope.menuOpenThisFrame ? "open" : "closed",
                _scope.driverFrameAuthorityActive ?
                    (_scope.menuOpenThisFrame ? "frik-driver" : "frik-driver-latched") :
                    "root-flattened",
                _scope.safeHandFrames[0].hasDriverToHandLocal ? "ready" : "missing",
                _scope.safeHandFrames[1].hasDriverToHandLocal ? "ready" : "missing");
        }

        // Central game delta; an unmeasurable frame holds the rebase
        // interpolation instead of advancing it by fabricated time.
        const float frameDeltaSeconds = std::isfinite(dt) && dt > 0.0f ? (std::min)(dt, 0.1f) : 0.0f;
        const auto refreshHand = [this, weaponNode, driverFrameAuthorityStoppedThisFrame, frameDeltaSeconds](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
            const std::size_t handIndex = isLeft ? 0u : 1u;
            ScopeSafeHandFrameState& state = _scope.safeHandFrames[handIndex];
            ScopeSafeHandFrameDiagnostic& diagnostic = state.diagnostic;
            diagnostic = {};
            state.currentHandWorldValid = false;

            RE::NiTransform rootHandWorld{};
            const bool rootSampleAllowed =
                !_scope.driverFrameAuthorityActive;
            const bool rootHandValid = !_scope.driverFrameAuthorityActive &&
                                       tryGetRootFlattenedHandBoneTransform(isLeft, rootHandWorld);
            const bool driverValid = driverFrame.valid &&
                                     isUsableHandAuthorityTransform(driverFrame.world);
            RE::NiTransform reconstructedHandWorld{};
            bool reconstructedHandValid = false;
            if (driverValid && state.hasDriverToHandLocal) {
                reconstructedHandWorld = scope_safe_hand_frame_math::resolveHandWorld(
                    driverFrame.world,
                    state.driverToHandLocal);
                reconstructedHandValid = isUsableHandAuthorityTransform(reconstructedHandWorld);
            }
            const auto resolutionMode = scope_safe_hand_frame_math::resolveCollisionIsolatedMode(
                _visuals.weaponCollisionHandPresentationFromPreviousFrame[handIndex],
                _scope.driverFrameAuthorityActive,
                rootHandValid,
                reconstructedHandValid,
                state.hasLastHandWorld,
                state.consecutiveDriverMissFrames,
                SCOPE_DRIVER_MISS_GRACE_FRAMES);

            diagnostic.rootHandWorld = rootHandWorld;
            diagnostic.driverWorld = driverFrame.world;
            diagnostic.reconstructedHandWorld = reconstructedHandWorld;
            diagnostic.resolutionMode = resolutionMode;
            diagnostic.consecutiveDriverMissFramesBefore =
                state.consecutiveDriverMissFrames;
            diagnostic.rootSampleAllowed = rootSampleAllowed;
            diagnostic.rootHandValid = rootHandValid;
            diagnostic.driverNodeAvailable = driverFrame.nodeAvailable;
            diagnostic.driverWorldFinite = driverFrame.worldFinite;
            diagnostic.driverFrameValid = driverFrame.valid;
            diagnostic.driverWorldUsable = driverValid;
            diagnostic.driverToHandLocalAvailable =
                state.hasDriverToHandLocal;
            diagnostic.reconstructedHandValid = reconstructedHandValid;
            diagnostic.lastHandWorldAvailable = state.hasLastHandWorld;
            diagnostic.collisionPresentationWasLive =
                _visuals.weaponCollisionHandPresentationFromPreviousFrame[handIndex];
            diagnostic.scopeDriverFrameAuthorityActive =
                _scope.driverFrameAuthorityActive;

            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::RootFlattened) {
                const bool recentScopedHandAvailable = state.hasLastHandWorld &&
                                                       state.consecutiveDriverMissFrames < SCOPE_DRIVER_MISS_GRACE_FRAMES;
                if (scope_safe_hand_frame_math::shouldStartRootRebase(
                        _scope.manualActivationRequested,
                        driverFrameAuthorityStoppedThisFrame,
                        reconstructedHandValid,
                        recentScopedHandAvailable)) {
                    // The previous ROCK output is the continuity authority.
                    // hFRIK may resume non-scope damping from a stale internal
                    // sample on this exact edge even though its driver is finite.
                    const RE::NiTransform& continuityHandWorld = recentScopedHandAvailable ?
                                                                      state.lastHandWorld :
                                                                      reconstructedHandWorld;
                    const RE::NiTransform rootRebaseLocalStart = transform_math::composeTransforms(
                        transform_math::invertTransform(rootHandWorld),
                        continuityHandWorld);
                    if (isUsableHandAuthorityTransform(rootRebaseLocalStart)) {
                        state.rootRebaseLocalStart = rootRebaseLocalStart;
                        state.rootRebaseElapsedSeconds = 0.0f;
                        state.rootRebaseActive = true;
                    }
                }
                state.consecutiveDriverMissFrames = 0;

                RE::NiTransform resolvedHandWorld = rootHandWorld;
                if (state.rootRebaseActive) {
                    const RE::NiTransform identity = transform_math::makeIdentityTransform<RE::NiTransform>();
                    const float rebaseAlpha = scope_safe_hand_frame_math::rebaseAlpha(
                        state.rootRebaseElapsedSeconds,
                        SCOPE_ROOT_REBASE_DURATION_SECONDS);
                    const RE::NiTransform rebase = scope_safe_hand_frame_math::interpolateRebaseTransform(
                        state.rootRebaseLocalStart,
                        identity,
                        rebaseAlpha);
                    const RE::NiTransform rebasedHandWorld = transform_math::composeTransforms(rootHandWorld, rebase);
                    if (isUsableHandAuthorityTransform(rebasedHandWorld)) {
                        resolvedHandWorld = rebasedHandWorld;
                    } else {
                        state.rootRebaseActive = false;
                    }
                    if (rebaseAlpha >= 1.0f) {
                        state.rootRebaseActive = false;
                    } else {
                        state.rootRebaseElapsedSeconds = (std::min)(
                            SCOPE_ROOT_REBASE_DURATION_SECONDS,
                            state.rootRebaseElapsedSeconds + frameDeltaSeconds);
                    }
                }

                state.currentHandWorld = resolvedHandWorld;
                state.currentHandWorldValid = true;
                state.lastHandWorld = resolvedHandWorld;
                state.hasLastHandWorld = true;

                const bool preserveAuthoredPhysicalRelation =
                    !isLeft &&
                    _firing.authoredHandWorldActive &&
                    _firing.hasRightNaturalBoneInDampedDriver;
                if (driverValid && !preserveAuthoredPhysicalRelation) {
                    const RE::NiTransform driverToHandLocal =
                        scope_safe_hand_frame_math::captureDriverToHandLocal(driverFrame.world, resolvedHandWorld);
                    if (isUsableHandAuthorityTransform(driverToHandLocal)) {
                        state.driverToHandLocal = driverToHandLocal;
                        state.hasDriverToHandLocal = true;
                    }
                }
                return;
            }

            state.rootRebaseActive = false;
            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::DriverReconstructed) {
                state.consecutiveDriverMissFrames = 0;
                state.currentHandWorld = reconstructedHandWorld;
                state.currentHandWorldValid = true;
                state.lastHandWorld = reconstructedHandWorld;
                state.hasLastHandWorld = true;
                return;
            }

            // A transient hFRIK arm-driver miss must not become an ownership
            // release/reacquire loop. Hold only a few frames: a real driver
            // loss must still fail closed instead of pinning the weapon in the
            // world indefinitely.
            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::LastKnown) {
                ++state.consecutiveDriverMissFrames;
                state.currentHandWorld = state.lastHandWorld;
                state.currentHandWorldValid = true;
            } else if (_scope.driverFrameAuthorityActive) {
                state.consecutiveDriverMissFrames = SCOPE_DRIVER_MISS_GRACE_FRAMES;
            }
        };

        refreshHand(true, frameInput.leftHandDriverFrame);
        refreshHand(false, frameInput.rightHandDriverFrame);

        for (const bool isLeft : { true, false }) {
            const bool positionOnlyHandWorldActive =
                isLeft ?
                _firing.leftHandWorldActive :
                _firing.authoredHandWorldActive;
            if (!positionOnlyHandWorldActive) {
                continue;
            }

            // Position-only presentation owns the rendered firing hand. Every
            // solver consumer must keep seeing controller intent, so replay
            // the cached physical wrist/driver relation instead of reading
            // ROCK's previous authored hand output from the flattened tree.
            RE::NiTransform physicalHandWorld{};
            RE::NiTransform driverWorld{};
            if (tryResolvePhysicalHandFrame(
                    isLeft,
                    physicalHandWorld,
                    driverWorld)) {
                auto& firingState =
                    _scope.safeHandFrames[isLeft ? 0u : 1u];
                firingState.currentHandWorld = physicalHandWorld;
                firingState.currentHandWorldValid = true;
                firingState.lastHandWorld = physicalHandWorld;
                firingState.hasLastHandWorld = true;
                firingState.diagnostic.physicalFrameOverrideApplied = true;
            }
        }

        for (ScopeSafeHandFrameState& state : _scope.safeHandFrames) {
            state.diagnostic.currentHandWorld = state.currentHandWorld;
            state.diagnostic.currentHandWorldValid =
                state.currentHandWorldValid;
            state.diagnostic.consecutiveDriverMissFramesAfter =
                state.consecutiveDriverMissFrames;
        }

        if (_scope.transitionTraceFramesRemaining > 0) {
            struct HandTrace
            {
                RE::NiTransform rootWorld{};
                RE::NiTransform driverWorld{};
                RE::NiTransform reconstructedWorld{};
                RE::NiTransform solverWorld{};
                bool rootValid{ false };
                bool driverValid{ false };
                bool reconstructedValid{ false };
                bool solverValid{ false };
                float rootToReconstructedDistance{ -1.0f };
                float rootToSolverDistance{ -1.0f };
            };

            const auto distanceBetween = [](const RE::NiPoint3& left, const RE::NiPoint3& right) {
                const RE::NiPoint3 delta = left - right;
                const float distance = delta.Length();
                return std::isfinite(distance) ? distance : -1.0f;
            };
            const auto captureHandTrace = [this, &distanceBetween](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
                HandTrace trace{};
                const ScopeSafeHandFrameState& state =
                    _scope.safeHandFrames[isLeft ? 0u : 1u];
                trace.rootValid =
                    tryGetRootFlattenedHandBoneTransform(isLeft, trace.rootWorld);
                trace.driverValid = driverFrame.valid &&
                                    isUsableHandAuthorityTransform(driverFrame.world);
                if (trace.driverValid) {
                    trace.driverWorld = driverFrame.world;
                }
                if (trace.driverValid && state.hasDriverToHandLocal) {
                    trace.reconstructedWorld =
                        scope_safe_hand_frame_math::resolveHandWorld(
                            driverFrame.world,
                            state.driverToHandLocal);
                    trace.reconstructedValid =
                        isUsableHandAuthorityTransform(trace.reconstructedWorld);
                }
                trace.solverValid = state.currentHandWorldValid;
                if (trace.solverValid) {
                    trace.solverWorld = state.currentHandWorld;
                }
                if (trace.rootValid && trace.reconstructedValid) {
                    trace.rootToReconstructedDistance = distanceBetween(
                        trace.rootWorld.translate,
                        trace.reconstructedWorld.translate);
                }
                if (trace.rootValid && trace.solverValid) {
                    trace.rootToSolverDistance = distanceBetween(
                        trace.rootWorld.translate,
                        trace.solverWorld.translate);
                }
                return trace;
            };

            const HandTrace leftTrace =
                captureHandTrace(true, frameInput.leftHandDriverFrame);
            const HandTrace rightTrace =
                captureHandTrace(false, frameInput.rightHandDriverFrame);
            RE::NiPoint3 weaponWorldPosition{};
            const bool weaponWorldValid = weaponNode &&
                                          isFiniteTransform(weaponNode->world);
            if (weaponWorldValid) {
                weaponWorldPosition = weaponNode->world.translate;
            }
            RE::NiPoint3 playerWorldOffset{};
            bool playerWorldOffsetValid = false;
            if (const auto* playerNodes = f4vr::getPlayerNodes();
                playerNodes && playerNodes->playerworldnode &&
                isFiniteTransform(playerNodes->playerworldnode->local)) {
                playerWorldOffset =
                    playerNodes->playerworldnode->local.translate;
                playerWorldOffsetValid = true;
            }

            const std::uint32_t sampleIndex =
                SCOPE_TRANSITION_TRACE_FRAMES -
                _scope.transitionTraceFramesRemaining;
            ROCK_LOG_INFO(Weapon,
                "SCOPE-TRANSITION seq={} sample={}/{} buttonRequested={} rendererValid={} rendererActive={} menuOpen={} manual={} state={} driverAuthority={} weaponValid={} weapon=({:.2f},{:.2f},{:.2f}) playerOffsetValid={} playerOffset=({:.2f},{:.2f},{:.2f}) left[root={} driver={} reconstructed={} solver={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) solverT=({:.2f},{:.2f},{:.2f}) rootToReconstructed={:.2f} rootToSolver={:.2f}] right[root={} driver={} reconstructed={} solver={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) solverT=({:.2f},{:.2f},{:.2f}) rootToReconstructed={:.2f} rootToSolver={:.2f}]",
                _scope.transitionTraceSequence,
                sampleIndex,
                SCOPE_TRANSITION_TRACE_FRAMES,
                _scope.manualActivationRequested ? "yes" : "no",
                frameInput.nativeScopeRequestStateValid ? "yes" : "no",
                _scope.nativeRequestActive ? "yes" : "no",
                _scope.menuOpenThisFrame ? "yes" : "no",
                isManualOwnershipActive() ? "yes" : "no",
                static_cast<std::uint32_t>(_session.state),
                _scope.driverFrameAuthorityActive ? "yes" : "no",
                weaponWorldValid ? "yes" : "no",
                weaponWorldPosition.x,
                weaponWorldPosition.y,
                weaponWorldPosition.z,
                playerWorldOffsetValid ? "yes" : "no",
                playerWorldOffset.x,
                playerWorldOffset.y,
                playerWorldOffset.z,
                leftTrace.rootValid ? "yes" : "no",
                leftTrace.driverValid ? "yes" : "no",
                leftTrace.reconstructedValid ? "yes" : "no",
                leftTrace.solverValid ? "yes" : "no",
                leftTrace.rootWorld.translate.x,
                leftTrace.rootWorld.translate.y,
                leftTrace.rootWorld.translate.z,
                leftTrace.driverWorld.translate.x,
                leftTrace.driverWorld.translate.y,
                leftTrace.driverWorld.translate.z,
                leftTrace.reconstructedWorld.translate.x,
                leftTrace.reconstructedWorld.translate.y,
                leftTrace.reconstructedWorld.translate.z,
                leftTrace.solverWorld.translate.x,
                leftTrace.solverWorld.translate.y,
                leftTrace.solverWorld.translate.z,
                leftTrace.rootToReconstructedDistance,
                leftTrace.rootToSolverDistance,
                rightTrace.rootValid ? "yes" : "no",
                rightTrace.driverValid ? "yes" : "no",
                rightTrace.reconstructedValid ? "yes" : "no",
                rightTrace.solverValid ? "yes" : "no",
                rightTrace.rootWorld.translate.x,
                rightTrace.rootWorld.translate.y,
                rightTrace.rootWorld.translate.z,
                rightTrace.driverWorld.translate.x,
                rightTrace.driverWorld.translate.y,
                rightTrace.driverWorld.translate.z,
                rightTrace.reconstructedWorld.translate.x,
                rightTrace.reconstructedWorld.translate.y,
                rightTrace.reconstructedWorld.translate.z,
                rightTrace.solverWorld.translate.x,
                rightTrace.solverWorld.translate.y,
                rightTrace.solverWorld.translate.z,
                rightTrace.rootToReconstructedDistance,
                rightTrace.rootToSolverDistance);
            _scope.transitionFinalTraceSequence =
                _scope.transitionTraceSequence;
            _scope.transitionFinalTraceSample = sampleIndex;
            _scope.transitionFinalTracePending = true;
            --_scope.transitionTraceFramesRemaining;
        }
    }

    void TwoHandedGrip::deferScopeHandAuthorityClear(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        _scope.deferredHandAuthorityClears[isLeft ? 0u : 1u] |= scope_safe_hand_frame_math::roleMask(role);
    }

    void TwoHandedGrip::recordScopeHandAuthorityPublication(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scope.handAuthorityPublishedThisFrame[index] |= roleBit;
        // A successfully republished role is live again; a clear requested for
        // the same role while ScopeMenu was open is obsolete.
        _scope.deferredHandAuthorityClears[index] &= static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
    }

    bool TwoHandedGrip::clearHandAuthorityRoleNow(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const char* tag = nullptr;
        switch (role) {
        case scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip:
            tag = PRIMARY_GRIP_TAG;
            break;
        case scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip:
            tag = SUPPORT_GRIP_TAG;
            break;
        case scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach:
            tag = PRIMARY_DETACH_TAG;
            break;
        }

        if (!tag || !frik_visual_authority::clearExternalHandWorldTransform(tag, handFromBool(isLeft))) {
            return false;
        }

        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scope.deferredHandAuthorityClears[isLeft ? 0u : 1u] &=
            static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
        return true;
    }

    void TwoHandedGrip::reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode)
    {
        if (_scope.menuOpenThisFrame || !frik_visual_authority::isAvailable()) {
            return;
        }

        const auto pendingBefore = _scope.deferredHandAuthorityClears;
        if (pendingBefore[0] == 0 && pendingBefore[1] == 0) {
            return;
        }

        const scope_safe_hand_frame_math::DesiredHandAuthorityInput ownership{
            .gripping = _session.state == TwoHandedState::Gripping,
            .primaryHandAuthorityEnabled =
                weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_session.authorityMode),
            .firingHandIsLeft = isFiringHandLeft(),
            .leftPartGripActive = partGrip(true).active,
            .rightPartGripActive = partGrip(false).active,
        };
        const ScopeHandAuthorityCleanupVisualSnapshot visualSnapshot =
            captureScopeHandAuthorityCleanupVisuals(weaponNode);

        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> cleared{};
        std::array<scope_safe_hand_frame_math::HandAuthorityRoleMask, 2> retained{};
        bool clearAttempted = false;
        constexpr std::array roles{
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach,
        };

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            const auto desiredRoles = scope_safe_hand_frame_math::desiredRolesForHand(ownership, isLeft);
            for (const auto role : roles) {
                if (!scope_safe_hand_frame_math::hasRole(_scope.deferredHandAuthorityClears[index], role)) {
                    continue;
                }

                switch (scope_safe_hand_frame_math::resolveDeferredClearAction(
                    role,
                    desiredRoles,
                    _scope.handAuthorityPublishedThisFrame[index])) {
                case scope_safe_hand_frame_math::DeferredClearAction::RetainLiveRole:
                    _scope.deferredHandAuthorityClears[index] &=
                        static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(
                            ~scope_safe_hand_frame_math::roleMask(role));
                    retained[index] |= scope_safe_hand_frame_math::roleMask(role);
                    break;
                case scope_safe_hand_frame_math::DeferredClearAction::ClearStaleRole:
                    clearAttempted = true;
                    if (clearHandAuthorityRoleNow(role, isLeft)) {
                        cleared[index] |= scope_safe_hand_frame_math::roleMask(role);
                    }
                    break;
                case scope_safe_hand_frame_math::DeferredClearAction::WaitForReplacementPublication:
                    break;
                }
            }
        }

        // A stale-tag clear can ask hFRIK to restore an arm. Preserve the
        // already-solved weapon/scope frame; live replacement hand authority
        // was published before this reconciliation and remains selected.
        if (clearAttempted) {
            restoreScopeHandAuthorityCleanupVisuals(visualSnapshot);
        }
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: reconciled deferred native-scope hand authority "
            "left(clear=0x{:02X},retain=0x{:02X},pending=0x{:02X}) "
            "right(clear=0x{:02X},retain=0x{:02X},pending=0x{:02X})",
            static_cast<unsigned>(cleared[0]),
            static_cast<unsigned>(retained[0]),
            static_cast<unsigned>(_scope.deferredHandAuthorityClears[0]),
            static_cast<unsigned>(cleared[1]),
            static_cast<unsigned>(retained[1]),
            static_cast<unsigned>(_scope.deferredHandAuthorityClears[1]));
    }
}

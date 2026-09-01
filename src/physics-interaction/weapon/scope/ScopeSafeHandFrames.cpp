#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// ScopeMenu-safe hand frames and deferred scope hand-authority role clears.

namespace rock
{
    void TwoHandedGrip::refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        // A final trace is valid only for the same update that produced its
        // pre-solve sample. Early-return frames deliberately remain pre-only.
        _nativeScopeTransitionFinalTracePending = false;
        const bool activationStateChanged =
            frameInput.manualScopeActivationRequested !=
                _manualScopeActivationRequested ||
            frameInput.nativeScopeRequestStateValid != _nativeScopeRequestStateValid ||
            (frameInput.nativeScopeRequestStateValid &&
                frameInput.nativeScopeRequestActive != _nativeScopeRequestActive);
        _manualScopeActivationRequested =
            frameInput.manualScopeActivationRequested;
        _nativeScopeRequestStateValid = frameInput.nativeScopeRequestStateValid;
        _nativeScopeRequestActive = frameInput.nativeScopeRequestStateValid &&
                                    frameInput.nativeScopeRequestActive;
        _nativeScopeActivationDebugSnapshot = NativeScopeActivationDebugSnapshot{
            .publicationSequence =
                _nativeScopeActivationDebugSnapshot.publicationSequence + 1,
            .weaponGenerationKey = _nativeScopeAnchorGenerationKey,
            .anchorSource = _nativeScopeAnchorSource,
            .manualInputRequested = _manualScopeActivationRequested,
            .rendererStateValid = _nativeScopeRequestStateValid,
            .rendererActive = _nativeScopeRequestActive,
        };
        if (activationStateChanged) {
            ++_nativeScopeTransitionTraceSequence;
            _nativeScopeTransitionTraceFramesRemaining =
                SCOPE_TRANSITION_TRACE_FRAMES;
        }

        const bool scopeStateChanged = _scopeMenuOpenThisFrame != frameInput.scopeMenuOpen;
        _scopeMenuOpenThisFrame = frameInput.scopeMenuOpen;
        _scopeMenuClosedThisFrame = scopeStateChanged && !_scopeMenuOpenThisFrame;
        const bool driverFrameAuthorityWasActive = _scopeDriverFrameAuthorityActive;
        _scopeDriverFrameAuthorityActive = scope_safe_hand_frame_math::retainDriverFrameAuthority(
            _scopeMenuOpenThisFrame,
            _manualScopeActivationRequested,
            isManualOwnershipActive(),
            driverFrameAuthorityWasActive);
        const bool driverFrameAuthorityStoppedThisFrame =
            driverFrameAuthorityWasActive && !_scopeDriverFrameAuthorityActive;

        if (scopeStateChanged) {
            // Never resume a pre-menu visual interpolation after hFRIK restores
            // its visible body. The weapon solver itself remains continuous.
            resetLockedHandVisualLerp();
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: native scope hand-frame menu={} solver={} leftCache={} rightCache={}",
                _scopeMenuOpenThisFrame ? "open" : "closed",
                _scopeDriverFrameAuthorityActive ?
                    (_scopeMenuOpenThisFrame ? "frik-driver" : "frik-driver-latched") :
                    "root-flattened",
                _scopeSafeHandFrames[0].hasDriverToHandLocal ? "ready" : "missing",
                _scopeSafeHandFrames[1].hasDriverToHandLocal ? "ready" : "missing");
        }

        // Central game delta; an unmeasurable frame holds the rebase
        // interpolation instead of advancing it by fabricated time.
        const float frameDeltaSeconds = std::isfinite(dt) && dt > 0.0f ? (std::min)(dt, 0.1f) : 0.0f;
        const auto refreshHand = [this, weaponNode, driverFrameAuthorityStoppedThisFrame, frameDeltaSeconds](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
            const std::size_t handIndex = isLeft ? 0u : 1u;
            ScopeSafeHandFrameState& state = _scopeSafeHandFrames[handIndex];
            ScopeSafeHandFrameDiagnostic& diagnostic = state.diagnostic;
            diagnostic = {};
            state.currentHandWorldValid = false;

            RE::NiTransform rootHandWorld{};
            const bool rootSampleAllowed =
                !_scopeDriverFrameAuthorityActive;
            const bool rootHandValid = !_scopeDriverFrameAuthorityActive &&
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
                _weaponCollisionHandPresentationFromPreviousFrame[handIndex],
                _scopeDriverFrameAuthorityActive,
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
                _weaponCollisionHandPresentationFromPreviousFrame[handIndex];
            diagnostic.scopeDriverFrameAuthorityActive =
                _scopeDriverFrameAuthorityActive;

            if (resolutionMode == scope_safe_hand_frame_math::ResolutionMode::RootFlattened) {
                const bool recentScopedHandAvailable = state.hasLastHandWorld &&
                                                       state.consecutiveDriverMissFrames < SCOPE_DRIVER_MISS_GRACE_FRAMES;
                if (scope_safe_hand_frame_math::shouldStartRootRebase(
                        _manualScopeActivationRequested,
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
                    _authoredPrimaryFiringHandWorldActive &&
                    _hasRightNaturalBoneInDampedDriver;
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
            } else if (_scopeDriverFrameAuthorityActive) {
                state.consecutiveDriverMissFrames = SCOPE_DRIVER_MISS_GRACE_FRAMES;
            }
        };

        refreshHand(true, frameInput.leftHandDriverFrame);
        refreshHand(false, frameInput.rightHandDriverFrame);

        for (const bool isLeft : { true, false }) {
            const bool positionOnlyHandWorldActive =
                isLeft ?
                _leftFiringHandWorldActive :
                _authoredPrimaryFiringHandWorldActive;
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
                    _scopeSafeHandFrames[isLeft ? 0u : 1u];
                firingState.currentHandWorld = physicalHandWorld;
                firingState.currentHandWorldValid = true;
                firingState.lastHandWorld = physicalHandWorld;
                firingState.hasLastHandWorld = true;
                firingState.diagnostic.physicalFrameOverrideApplied = true;
            }
        }

        for (ScopeSafeHandFrameState& state : _scopeSafeHandFrames) {
            state.diagnostic.currentHandWorld = state.currentHandWorld;
            state.diagnostic.currentHandWorldValid =
                state.currentHandWorldValid;
            state.diagnostic.consecutiveDriverMissFramesAfter =
                state.consecutiveDriverMissFrames;
        }

        if (_nativeScopeTransitionTraceFramesRemaining > 0) {
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
                    _scopeSafeHandFrames[isLeft ? 0u : 1u];
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
                _nativeScopeTransitionTraceFramesRemaining;
            ROCK_LOG_INFO(Weapon,
                "SCOPE-TRANSITION seq={} sample={}/{} buttonRequested={} rendererValid={} rendererActive={} menuOpen={} manual={} state={} driverAuthority={} weaponValid={} weapon=({:.2f},{:.2f},{:.2f}) playerOffsetValid={} playerOffset=({:.2f},{:.2f},{:.2f}) left[root={} driver={} reconstructed={} solver={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) solverT=({:.2f},{:.2f},{:.2f}) rootToReconstructed={:.2f} rootToSolver={:.2f}] right[root={} driver={} reconstructed={} solver={} rootT=({:.2f},{:.2f},{:.2f}) driverT=({:.2f},{:.2f},{:.2f}) reconstructedT=({:.2f},{:.2f},{:.2f}) solverT=({:.2f},{:.2f},{:.2f}) rootToReconstructed={:.2f} rootToSolver={:.2f}]",
                _nativeScopeTransitionTraceSequence,
                sampleIndex,
                SCOPE_TRANSITION_TRACE_FRAMES,
                _manualScopeActivationRequested ? "yes" : "no",
                frameInput.nativeScopeRequestStateValid ? "yes" : "no",
                _nativeScopeRequestActive ? "yes" : "no",
                _scopeMenuOpenThisFrame ? "yes" : "no",
                isManualOwnershipActive() ? "yes" : "no",
                static_cast<std::uint32_t>(_state),
                _scopeDriverFrameAuthorityActive ? "yes" : "no",
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
            _nativeScopeTransitionFinalTraceSequence =
                _nativeScopeTransitionTraceSequence;
            _nativeScopeTransitionFinalTraceSample = sampleIndex;
            _nativeScopeTransitionFinalTracePending = true;
            --_nativeScopeTransitionTraceFramesRemaining;
        }
    }

    void TwoHandedGrip::deferScopeHandAuthorityClear(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        _scopeDeferredHandAuthorityClears[isLeft ? 0u : 1u] |= scope_safe_hand_frame_math::roleMask(role);
    }

    void TwoHandedGrip::recordScopeHandAuthorityPublication(
        const scope_safe_hand_frame_math::HandAuthorityRole role,
        const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scopeHandAuthorityPublishedThisFrame[index] |= roleBit;
        // A successfully republished role is live again; a clear requested for
        // the same role while ScopeMenu was open is obsolete.
        _scopeDeferredHandAuthorityClears[index] &= static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
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
        _scopeDeferredHandAuthorityClears[isLeft ? 0u : 1u] &=
            static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
        return true;
    }

    void TwoHandedGrip::reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode)
    {
        if (_scopeMenuOpenThisFrame || !frik_visual_authority::isAvailable()) {
            return;
        }

        const auto pendingBefore = _scopeDeferredHandAuthorityClears;
        if (pendingBefore[0] == 0 && pendingBefore[1] == 0) {
            return;
        }

        const scope_safe_hand_frame_math::DesiredHandAuthorityInput ownership{
            .gripping = _state == TwoHandedState::Gripping,
            .primaryHandAuthorityEnabled =
                weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_authorityMode),
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
                if (!scope_safe_hand_frame_math::hasRole(_scopeDeferredHandAuthorityClears[index], role)) {
                    continue;
                }

                switch (scope_safe_hand_frame_math::resolveDeferredClearAction(
                    role,
                    desiredRoles,
                    _scopeHandAuthorityPublishedThisFrame[index])) {
                case scope_safe_hand_frame_math::DeferredClearAction::RetainLiveRole:
                    _scopeDeferredHandAuthorityClears[index] &=
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
            static_cast<unsigned>(_scopeDeferredHandAuthorityClears[0]),
            static_cast<unsigned>(cleared[1]),
            static_cast<unsigned>(retained[1]),
            static_cast<unsigned>(_scopeDeferredHandAuthorityClears[1]));
    }
}

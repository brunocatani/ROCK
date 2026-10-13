#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// ScopeMenu-safe hand frames and deferred scope hand-authority role clears.

namespace rock
{
    void TwoHandedGrip::refreshScopeSafeHandFrames(RE::NiNode* weaponNode, const EquippedWeaponGripFrameInput& frameInput, float dt)
    {
        performance_profiler::ScopedTimer gripStageTimer(performance_profiler::Scope::EquippedGripScopeFrames);
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
            // FRIK 2.3 keeps solving the arms while scoped. A menu edge does
            // not acquire a new grip: preserve its interpolation state so a
            // seated hand cannot jump back to controller intent and re-seat.
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: native scope hand-frame menu={} solver={} leftCache={} rightCache={} gripAlpha=({:.3f},{:.3f},{:.3f})",
                _scope.menuOpenThisFrame ? "open" : "closed",
                _scope.driverFrameAuthorityActive ?
                    (_scope.menuOpenThisFrame ? "frik-driver" : "frik-driver-latched") :
                    "root-flattened",
                _scope.safeHandFrames[0].hasDriverToHandLocal ? "ready" : "missing",
                _scope.safeHandFrames[1].hasDriverToHandLocal ? "ready" : "missing",
                _visuals.primaryHandLerp.lastAlpha, partGrip(true).visualLerp.lastAlpha, partGrip(false).visualLerp.lastAlpha);
        }

        // Central game delta; an unmeasurable frame holds the rebase
        // interpolation instead of advancing it by fabricated time.
        const float frameDeltaSeconds = std::isfinite(dt) && dt > 0.0f ? (std::min)(dt, 0.1f) : 0.0f;
        const auto refreshHand = [this, weaponNode, driverFrameAuthorityStoppedThisFrame, frameDeltaSeconds](bool isLeft, const EquippedWeaponScopeHandDriverFrame& driverFrame) {
            const std::size_t handIndex = isLeft ? 0u : 1u;
            ScopeSafeHandFrameState& state = _scope.safeHandFrames[handIndex];
            state.currentHandWorldValid = false;

            RE::NiTransform rootHandWorld{};
            const bool rootSampleAllowed =
                scope_safe_hand_frame_math::canSampleScopeRootHand(
                    _scope.driverFrameAuthorityActive,
                    state.hasDriverToHandLocal,
                    frik_hand_world_authority::hasCalibratedRawHandFrame(isLeft));
            bool rootHandValid = rootSampleAllowed &&
                                 tryGetRootFlattenedHandBoneTransform(isLeft, rootHandWorld);
            const bool driverValid = driverFrame.valid &&
                                     isUsableHandAuthorityTransform(driverFrame.world);
            if (rootHandValid && _scope.driverFrameAuthorityActive) {
                // A scoped sample is cached as the driver relation and never
                // re-taken while the menu is held, so refuse an unplaced arm
                // rather than latch it for the rest of the hold.
                const float dx = rootHandWorld.translate.x - driverFrame.world.translate.x;
                const float dy = rootHandWorld.translate.y - driverFrame.world.translate.y;
                const float dz = rootHandWorld.translate.z - driverFrame.world.translate.z;
                rootHandValid = driverValid &&
                                scope_safe_hand_frame_math::isPlausibleScopeSeedHand(
                                    std::sqrt(dx * dx + dy * dy + dz * dz));
            }
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
                    frik_hand_world_authority::hasCalibratedRawHandFrame(false);
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
            }
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

        if (!tag || !frik_visual_authority::clearHandWorld(tag, handFromBool(isLeft))) {
            return false;
        }

        const auto roleBit = scope_safe_hand_frame_math::roleMask(role);
        _scope.deferredHandAuthorityClears[isLeft ? 0u : 1u] &=
            static_cast<scope_safe_hand_frame_math::HandAuthorityRoleMask>(~roleBit);
        return true;
    }

    void TwoHandedGrip::reconcileDeferredScopeHandAuthority(RE::NiNode* weaponNode)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }
        /*
         * The deferral keeps hFRIK from restoring the tracked arm between two
         * ROCK roles while the ScopeMenu is open. A hand no role wants any
         * more has no next role to wait for, and a scope provider can keep
         * the vanilla ScopeMenu open long after the player stops aiming, so
         * such a hand is released now; hands with a live role still wait for
         * the menu to close.
         */
        const bool scopeMenuOpen = _scope.menuOpenThisFrame;

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
            if (scopeMenuOpen && desiredRoles != 0) {
                continue;
            }
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
    }
}

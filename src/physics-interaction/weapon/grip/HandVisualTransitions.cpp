#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Hand and weapon visual transitions: visual returns, locked-hand visuals, grip hand pose publication, weapon visual/collision-resolved authority application, and FRIK primary pose blocking.

namespace rock
{
    void TwoHandedGrip::publishCollisionIsolatedRightNativeWeaponIntent(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!_visuals.weaponCollisionHandPresentationFromPreviousFrame[1] ||
            !_visuals.weaponIntentObserver ||
            !weaponNode ||
            currentWeaponGenerationKey == 0 ||
            usesLeftFiringCarry() ||
            ownsWeaponTransform() ||
            !isFiniteTransform(weaponNode->local)) {
            return;
        }

        if (_firing.rightCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            /*
             * The authored pre-pass already rebuilt this frame's clean
             * position-only intent from the physical driver. Replacing its
             * presented-hand parent with the physical hand here would rotate
             * the weapon through the authored hand-in-weapon relation and
             * violate the position-only contract after a collision pulse.
             */
            if (isFiniteTransform(weaponNode->world)) {
                _visuals.weaponIntentObserver(
                    _visuals.weaponIntentObserverContext,
                    weaponNode,
                    weaponNode->world,
                    currentWeaponGenerationKey);
            }
            return;
        }

        RE::NiNode* rightHand = resolveFirstPersonHandNode(false);
        if (!rightHand || weaponNode->parent != rightHand) {
            return;
        }

        RE::NiTransform physicalRightHandWorld{};
        if (!tryGetSolverHandTransform(false, physicalRightHandWorld) ||
            !isUsableHandAuthorityTransform(physicalRightHandWorld)) {
            return;
        }

        const RE::NiTransform requestedWeaponWorld =
            transform_math::composeTransforms(
                physicalRightHandWorld,
                weaponNode->local);
        if (!isFiniteTransform(requestedWeaponWorld)) {
            return;
        }

        /*
         * FRIK has already authored this frame's weapon-local animation, but
         * its parent hand still contains the previous collision presentation.
         * Preserve the native local animation while replacing only that parent
         * basis with the collision-isolated physical hand. Later ROCK-owned
         * grip and return publications naturally supersede this
         * default intent through the same observer.
         */
        _visuals.weaponIntentObserver(
            _visuals.weaponIntentObserverContext,
            weaponNode,
            requestedWeaponWorld,
            currentWeaponGenerationKey);
    }

    void TwoHandedGrip::resetLockedHandVisualLerp()
    {
        _visuals.primaryHandLerp = {};
        partGrip(true).visualLerp = {};
        partGrip(false).visualLerp = {};
    }

    bool TwoHandedGrip::isHandVisualReturnActive(const bool isLeft) const
    {
        return _visuals.returningHands[isLeft ? 0u : 1u].transition.active;
    }

    bool TwoHandedGrip::hasVisualAuthorityForHand(const bool isLeft) const
    {
        if ((!isLeft && _firing.authoredHandWorldActive) ||
            (isLeft && _firing.leftHandWorldActive) ||
            isHandVisualReturnActive(isLeft) ||
            partGrip(isLeft).active) {
            return true;
        }
        return isFiringHand(isLeft) &&
            _session.state == TwoHandedState::Gripping &&
            weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_session.authorityMode);
    }

    void TwoHandedGrip::recordPublishedHandWorld(const bool isLeft, const RE::NiTransform& appliedWorld)
    {
        if (!isUsableHandAuthorityTransform(appliedWorld)) {
            return;
        }
        const std::size_t index = isLeft ? 0u : 1u;
        _visuals.lastPublishedHandWorld[index] = appliedWorld;
        _visuals.hasLastPublishedHandWorld[index] = true;
    }

    void TwoHandedGrip::beginHandVisualReturn(const bool isLeft, const char* reason)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _visuals.returningHands[index].transition;
        if (!hand_visual_lerp_math::kEquippedWeaponReturnEnabled ||
            !_visuals.hasLastPublishedHandWorld[index] ||
            !isUsableHandAuthorityTransform(_visuals.lastPublishedHandWorld[index]) ||
            !frik_visual_authority::isAvailable()) {
            clearHandVisualReturn(isLeft, "not-eligible", false);
            return;
        }

        state.begin(_visuals.lastPublishedHandWorld[index]);
        if (!frik_visual_authority::applyExternalHandWorldTransform(
                RETURN_HAND_TAG,
                handFromBool(isLeft),
                state.start,
                RETURN_HAND_VISUAL_PRIORITY)) {
            state.clear();
            (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: hand return start failed hand={}", isLeft ? "left" : "right");
            return;
        }

        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: hand return started hand={} reason={} from=({:.2f},{:.2f},{:.2f})",
            isLeft ? "left" : "right",
            reason ? reason : "unknown",
            state.start.translate.x,
            state.start.translate.y,
            state.start.translate.z);
    }

    void TwoHandedGrip::updateHandVisualReturns(const float dt)
    {
        if (_scope.menuOpenThisFrame) {
            return;
        }

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            auto& state = _visuals.returningHands[index].transition;
            if (!state.active) {
                continue;
            }

            RE::NiTransform targetWorld{};
            if (!frik_visual_authority::isAvailable() ||
                !tryGetSolverHandTransform(isLeft, targetWorld) ||
                !isUsableHandAuthorityTransform(targetWorld)) {
                clearHandVisualReturn(isLeft, "tracked-hand-unavailable", true);
                continue;
            }

            /*
             * Position-only carry re-seats the right hand at the authored
             * grip the moment the session resumes. Returning to the physical
             * wrist first showed the un-authored pose for a split second and
             * then snapped. Target the authored seat on the live (possibly
             * still returning) weapon so the return lands where the session
             * resumes. The physical target stays the fallback for drops and
             * for an implausibly distant seat.
             */
            if (!isLeft) {
                RE::NiTransform seatTargetWorld{};
                if (tryResolveAuthoredPositionOnlySeatWorld(seatTargetWorld) &&
                    hand_visual_lerp_math::distanceGameUnits(
                        seatTargetWorld.translate,
                        targetWorld.translate) <= 50.0f) {
                    targetWorld = seatTargetWorld;
                }
            }

            const auto result = hand_visual_lerp_math::driveVisualReturn(
                state,
                targetWorld,
                dt,
                hand_visual_lerp_math::kEquippedWeaponReturnConfig,
                [](const RE::NiTransform& transform) {
                    return isUsableHandAuthorityTransform(transform);
                },
                [isLeft](const RE::NiTransform& transform) {
                    return frik_visual_authority::applyExternalHandWorldTransform(
                        RETURN_HAND_TAG,
                        handFromBool(isLeft),
                        transform,
                        RETURN_HAND_VISUAL_PRIORITY);
                });
            if (result.timingInitializedThisFrame) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return timing hand={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
                    isLeft ? "left" : "right",
                    result.initialDistanceGameUnits,
                    result.initialAngleDegrees,
                    result.durationSeconds);
            }
            if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::InvalidTransform ||
                result.status == hand_visual_lerp_math::VisualReturnDriveStatus::PublishFailed) {
                clearHandVisualReturn(isLeft, "publish-failed", true);
                continue;
            }

            if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::Completed) {
                const float completedDuration = result.durationSeconds;
                (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
                state.clear();
                _visuals.hasLastPublishedHandWorld[index] = false;
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: hand return completed hand={} duration={:.3f}s",
                    isLeft ? "left" : "right",
                    completedDuration);
            }
        }
    }

    void TwoHandedGrip::clearHandVisualReturn(const bool isLeft, const char* reason, const bool logCancellation)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        auto& state = _visuals.returningHands[index].transition;
        const bool wasActive = state.active;
        (void)frik_visual_authority::clearExternalHandWorldTransform(RETURN_HAND_TAG, handFromBool(isLeft));
        state.clear();
        _visuals.hasLastPublishedHandWorld[index] = false;
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: hand return cancelled hand={} reason={}",
                isLeft ? "left" : "right",
                reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::cancelHandVisualReturn(const bool isLeft, const char* reason)
    {
        clearHandVisualReturn(isLeft, reason, true);
    }

    void TwoHandedGrip::beginWeaponVisualReturn(const char* reason)
    {
        if (!hand_visual_lerp_math::kEquippedWeaponReturnEnabled ||
            _visuals.returningWeapon.localTransition.active ||
            !_session.weaponNode ||
            !_hasWeaponNodeLocalBaseline ||
            _session.weaponGenerationKey == 0 ||
            _session.equippedWeaponOwnershipKey == 0) {
            return;
        }

        RE::NiTransform startWorld = _visuals.hasLastRenderedWeaponWorld ? _visuals.lastRenderedWeaponWorld : _session.weaponNode->world;
        if (!isFiniteTransform(startWorld) || !isFiniteTransform(_weaponNodeLocalBaseline)) {
            return;
        }

        RE::NiNode* nativeParent = _session.weaponNode->parent;
        if (_leftCarry.weaponNodeReparented) {
            nativeParent = resolveFirstPersonHandNode(false);
            if (!nativeParent) {
                return;
            }
        }
        if (!nativeParent) {
            return;
        }

        const RE::NiTransform startLocal = weapon_visual_authority_math::worldTargetToParentLocal(nativeParent->world, startWorld);
        if (!isFiniteTransform(startLocal)) {
            return;
        }

        /*
         * blockPrimaryWeaponNodeOwnership is hFRIK's external LEFT-carry
         * topology switch, not a transform-write-only blocker. Retaining it
         * here makes hFRIK reparent the weapon back under LArm_Hand on the next
         * frame, which invalidates this right-parent-local return and snaps the
         * weapon immediately. Release left-carry topology before beginning the
         * overlay. ROCK runs after hFRIK and republishes the interpolated node
         * every frame, so hFRIK's earlier native write cannot reach rendering;
         * at the exact endpoint both writers already agree on the baseline.
         */
        releaseFiringHandWeaponNodeOwnership(_session.weaponNode);
        if (_session.weaponNode->parent != nativeParent) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: weapon return skipped because native right-hand parenting could not be restored");
            return;
        }

        RE::NiTransform returnTargetLocal = _weaponNodeLocalBaseline;
        const bool followsAuthoredPrimaryGrip =
            usesNativeRightCarry() &&
            tryResolveAuthoredPrimaryWeaponReturnTargetLocal(
                _session.weaponNode,
                nativeParent,
                _weaponNodeLocalBaseline,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey,
                returnTargetLocal);

        ReturningWeaponVisualState returnState{};
        returnState.weaponNode = _session.weaponNode;
        returnState.nativeParent = nativeParent;
        returnState.weaponGenerationKey = _session.weaponGenerationKey;
        returnState.equippedWeaponOwnershipKey = _session.equippedWeaponOwnershipKey;
        returnState.nativeBaselineLocal = _weaponNodeLocalBaseline;
        returnState.lastTargetLocal = returnTargetLocal;
        returnState.retainPrimaryPoseBlocker = usesLeftFiringCarry();
        returnState.followsAuthoredPrimaryGrip = followsAuthoredPrimaryGrip;
        returnState.localTransition.begin(startLocal);
        returnState.localTransition.durationSeconds = hand_visual_lerp_math::computeVisualReturnDuration(
            startLocal,
            returnState.lastTargetLocal,
            hand_visual_lerp_math::kEquippedWeaponReturnConfig);
        returnState.localTransition.durationInitialized = true;
        if (!moveWeaponPresentationRigidly(_session.weaponNode, startWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: weapon return rejected an invalid presentation subtree");
            return;
        }
        _visuals.returningWeapon = returnState;
        _visuals.lastRenderedWeaponWorld = _session.weaponNode->world;
        _visuals.hasLastRenderedWeaponWorld = true;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: weapon return started reason={} target={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
            reason ? reason : "unknown",
            followsAuthoredPrimaryGrip ? "authored-primary" : "native-baseline",
            hand_visual_lerp_math::distanceGameUnits(startLocal.translate, returnState.lastTargetLocal.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(startLocal, returnState.lastTargetLocal),
            _visuals.returningWeapon.localTransition.durationSeconds);
    }

    void TwoHandedGrip::updateWeaponVisualReturn(
        RE::NiNode* currentWeaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const float dt)
    {
        auto& state = _visuals.returningWeapon;
        if (!state.localTransition.active) {
            return;
        }
        if (!runtime_state::isLocalSkeletonReady() ||
            !currentWeaponNode ||
            currentWeaponNode != state.weaponNode ||
            currentWeaponGenerationKey != state.weaponGenerationKey ||
            currentEquippedWeaponOwnershipKey != state.equippedWeaponOwnershipKey ||
            !state.nativeParent ||
            !isFiniteTransform(state.nativeParent->world) ||
            currentWeaponNode->parent != state.nativeParent) {
            clearAllVisualReturns("weapon-identity-or-parent-changed", true, true);
            return;
        }

        RE::NiTransform targetLocal = state.nativeBaselineLocal;
        if (state.followsAuthoredPrimaryGrip) {
            RE::NiTransform liveAuthoredTargetLocal{};
            if (tryResolveAuthoredPrimaryWeaponReturnTargetLocal(
                    currentWeaponNode,
                    state.nativeParent,
                    state.nativeBaselineLocal,
                    state.weaponGenerationKey,
                    state.equippedWeaponOwnershipKey,
                    liveAuthoredTargetLocal)) {
                state.lastTargetLocal = liveAuthoredTargetLocal;
            }
            targetLocal = state.lastTargetLocal;
            (void)retainAuthoredPrimaryFiringGripFingerPoseForHandoff(
                currentWeaponNode,
                state.weaponGenerationKey,
                state.equippedWeaponOwnershipKey);
        }

        const auto result = hand_visual_lerp_math::driveVisualReturn(
            state.localTransition,
            targetLocal,
            dt,
            hand_visual_lerp_math::kEquippedWeaponReturnConfig,
            [](const RE::NiTransform& transform) {
                return isFiniteTransform(transform);
            },
            [this, currentWeaponNode, &state](const RE::NiTransform& transform) {
                const RE::NiTransform returnedWeaponWorld =
                    transform_math::composeTransforms(
                        state.nativeParent->world,
                        transform);
                return applyWeaponVisualAuthority(
                    currentWeaponNode,
                    returnedWeaponWorld,
                    state.weaponGenerationKey);
            });
        if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::InvalidTransform) {
            clearWeaponVisualReturn("non-finite-return-transform", true, true);
            return;
        }
        if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::PublishFailed) {
            clearWeaponVisualReturn("weapon-return-publish-failed", true, true);
            return;
        }
        _lastSolvedWeaponTransform = currentWeaponNode->world;
        _hasSolvedWeaponTransform = true;
        if (result.status == hand_visual_lerp_math::VisualReturnDriveStatus::Completed) {
            const float completedDuration = result.durationSeconds;
            const bool preserveAuthoredPrimaryPose =
                state.followsAuthoredPrimaryGrip;
            clearWeaponVisualReturn(
                "completed",
                false,
                true,
                preserveAuthoredPrimaryPose);
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return completed duration={:.3f}s", completedDuration);
        }
    }

    void TwoHandedGrip::clearWeaponVisualReturn(
        const char* reason,
        const bool logCancellation,
        const bool restoreBlockers,
        const bool preserveAuthoredPrimaryPose)
    {
        const bool wasActive = _visuals.returningWeapon.localTransition.active;
        const bool retainedPrimaryPoseBlocker = _visuals.returningWeapon.retainPrimaryPoseBlocker;
        const bool retainedAuthoredPrimaryPose =
            _visuals.returningWeapon.followsAuthoredPrimaryGrip;
        RE::NiNode* returnNode = _visuals.returningWeapon.weaponNode;
        _visuals.returningWeapon = {};
        if (restoreBlockers) {
            releaseFiringHandWeaponNodeOwnership(returnNode);
            if (retainedPrimaryPoseBlocker) {
                restoreFrikPrimaryWeaponPose();
            }
        }
        if (wasActive && retainedAuthoredPrimaryPose &&
            !preserveAuthoredPrimaryPose) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
        if (wasActive && logCancellation) {
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: weapon return cancelled reason={}", reason ? reason : "unknown");
        }
    }

    void TwoHandedGrip::clearAllVisualReturns(const char* reason, const bool logCancellation, const bool restoreBlockers)
    {
        clearHandVisualReturn(true, reason, logCancellation);
        clearHandVisualReturn(false, reason, logCancellation);
        clearWeaponVisualReturn(reason, logCancellation, restoreBlockers);
    }

    bool TwoHandedGrip::tryResolveAuthoredPrimaryWeaponReturnTargetLocal(
        RE::NiNode* weaponNode,
        RE::NiNode* nativeParent,
        const RE::NiTransform& nativeBaselineLocal,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t equippedWeaponOwnershipKey,
        RE::NiTransform& outTargetLocal) const
    {
        outTargetLocal = {};
        if (!weaponNode || !nativeParent ||
            !hasRightFiringHandCanonicalFrame(
                weaponNode,
                weaponGenerationKey,
                equippedWeaponOwnershipKey) ||
            _firing.rightCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            !isFiniteTransform(nativeParent->world) ||
            !isFiniteTransform(nativeBaselineLocal)) {
            return false;
        }

        // The return reads the same physical-frame chain as the authored
        // session, so the slerp endpoint and the resumed session agree on the
        // tracked hand without reading ROCK's presented-hand output back.
        RE::NiTransform trackedRightHandWorld{};
        if (!tryGetAuthoredPrimaryTrackedFiringHandWorld(
                trackedRightHandWorld) ||
            !isFiniteTransform(trackedRightHandWorld)) {
            return false;
        }

        /*
         * The native reference must be the LIVE weapon local: hFRIK
         * republishes the native Weapon transform earlier in this same frame,
         * before ROCK writes, and the resumed session realigns from that live
         * pose. The grab-time _weaponNodeLocalBaseline is stale here - it
         * carries the session's own palm shift and, for grabs committed around
         * ScopeMenu, the scope-driven local - so a return targeted at it lands
         * away from the session pose and the weapon snaps on completion.
         */
        if (weaponNode->parent != nativeParent) {
            return false;
        }
        const RE::NiTransform nativeWeaponWorld =
            transform_math::composeTransforms(
                nativeParent->world,
                weaponNode->local);
        if (!isFiniteTransform(nativeWeaponWorld)) {
            return false;
        }
        const RE::NiPoint3 trackedPalmWorld =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                trackedRightHandWorld,
                false);
        const RE::NiTransform authoredWeaponWorld =
            authored_weapon_grip_capture_policy::
                resolveAuthoredPrimaryWeaponWorldPositionOnly(
                    nativeWeaponWorld,
                    _firing.rightCanonicalGripWeaponLocal,
                    trackedPalmWorld,
                    [](const RE::NiTransform& transform,
                        const RE::NiPoint3& point) {
                        return transform_math::localPointToWorld(
                            transform,
                            point);
                    });
        outTargetLocal =
            weapon_visual_authority_math::worldTargetToParentLocal(
                nativeParent->world,
                authoredWeaponWorld);
        return isFiniteTransform(authoredWeaponWorld) &&
               isFiniteTransform(outTargetLocal);
    }

    RE::NiTransform TwoHandedGrip::resolveLockedHandVisualTarget(
        const RE::NiTransform& targetWorld,
        const RE::NiTransform* liveHandWorld,
        float dt,
        LockedHandVisualLerpState& state)
    {
        /*
         * Authored, provider-owned, and visual-only support paths retain their
         * independent external-hand transition. Normal dynamic full-authority
         * acquisition is intercepted by resolveDynamicSupportAcquisitionHandTarget
         * so both hands share the pivot-preserving weapon correction alpha.
         */
        if (!g_rockConfig.rockWeaponSupportGripHandLerpEnabled) {
            state = {};
            return targetWorld;
        }

        if (!state.initialized) {
            const RE::NiTransform startWorld = (liveHandWorld && isFiniteTransform(*liveHandWorld)) ? *liveHandWorld : targetWorld;
            const float initialDistance =
                hand_visual_lerp_math::distanceGameUnits(startWorld.translate, targetWorld.translate);
            const float durationSeconds =
                hand_visual_lerp_math::computeDistanceMappedDurationGameUnits(
                    initialDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                    g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                    g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                    g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance);
            if (durationSeconds <= 0.0f) {
                state = {};
                state.lastAlpha = 1.0f;
                return targetWorld;
            }

            state.initialized = true;
            state.startWorld = startWorld;
            state.elapsedSeconds = 0.0f;
            state.durationSeconds = durationSeconds;
            state.lastAlpha = 0.0f;
        }

        state.elapsedSeconds =
            hand_visual_lerp_math::advanceTimedBlendElapsed(state.elapsedSeconds, dt, state.durationSeconds);
        const auto blended =
            hand_visual_lerp_math::blendTransformOverDuration(state.startWorld, targetWorld, state.elapsedSeconds, state.durationSeconds);
        state.lastAlpha = hand_visual_lerp_math::timedBlendAlpha(state.elapsedSeconds, state.durationSeconds);
        return blended.transform;
    }

    bool TwoHandedGrip::applyWeaponVisualAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const std::uint64_t authorityGenerationKey,
        const bool notifyVisualIntentObserver)
    {
        if (!weaponNode) {
            return false;
        }

        const std::uint64_t effectiveGenerationKey = authorityGenerationKey != 0 ? authorityGenerationKey : _session.weaponGenerationKey;
        if (notifyVisualIntentObserver && _visuals.weaponIntentObserver) {
            _visuals.weaponIntentObserver(
                _visuals.weaponIntentObserverContext,
                weaponNode,
                solvedWeaponWorld,
                effectiveGenerationKey);
        }
        const bool scopeAnchorMatchesAuthority =
            _scope.anchorValid && _scope.anchorWeaponNode == weaponNode && _scope.anchorGenerationKey == effectiveGenerationKey;

        // Capture hFRIK's engine-specific camera axis only before changing the
        // weapon. Once captured, every hand mode resolves the same immutable
        // generation-bound weapon-local scope frame.
        const NativeScopeCameraFollowCapture scopeCameraFollow = captureNativeScopeCameraFollow(weaponNode);
        if (scopeAnchorMatchesAuthority && scopeCameraFollow.valid) {
            (void)captureNativeScopeRigidFrame(weaponNode, effectiveGenerationKey, scopeCameraFollow.camera, scopeCameraFollow.cameraWorldBefore);
            (void)captureNativeScopeOverlayCalibration(scopeCameraFollow.cameraWorldBefore, effectiveGenerationKey);
        }

        if (!moveWeaponPresentationRigidly(weaponNode, solvedWeaponWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: rejected weapon visual authority because the root or bounded presentation subtree was invalid");
            return false;
        }

        const bool rigidFrameMatchesAuthority = _scope.rigidFrame.valid && _scope.rigidFrame.weaponGenerationKey == effectiveGenerationKey &&
            _scope.rigidFrame.weaponNodeIdentity == weaponNode && _scope.rigidFrame.scopeCameraIdentity == scopeCameraFollow.camera;
        const bool scopeTargetReady =
            rigidFrameMatchesAuthority &&
            isFiniteTransform(
                _scope.rigidFrame.cameraWeaponLocal) &&
            std::abs(_scope.rigidFrame.cameraWeaponLocal.scale) >
                0.0001f;
        const NativeScopeCameraFollowResult scopeCameraResult =
            scopeTargetReady ?
                applyNativeScopeCameraWorldTarget(
                    scopeCameraFollow,
                    native_scope_camera_follow_math::
                        resolveRigidAnchorFrameWorld(
                            weaponNode->world,
                            _scope.rigidFrame.cameraWeaponLocal)) :
                NativeScopeCameraFollowResult{};
        if (scopeCameraResult.targetValid && scopeCameraResult.writeApplied) {
            (void)applyNativeScopeOverlayTarget(scopeCameraResult.targetCameraWorld, effectiveGenerationKey);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation &&
            scopeTargetReady &&
            scopeCameraResult.immediateReadbackValid) {
            const RE::NiTransform immediateCameraWeaponLocal =
                transform_math::composeTransforms(
                    transform_math::invertTransform(weaponNode->world),
                    scopeCameraResult.immediateCameraWorldAfter);
            const float rigidPositionError = weaponSolverLength(
                weaponSolverSub(
                    immediateCameraWeaponLocal.translate,
                    _scope.rigidFrame.cameraWeaponLocal.translate));
            const float rigidRotationError =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    _scope.rigidFrame.cameraWeaponLocal,
                    immediateCameraWeaponLocal);
            ROCK_LOG_SAMPLE_DEBUG(
                Weapon,
                2000,
                "TwoHandedGrip: native scope retained weapon-local frame relationError=({:.4f}gu,{:.3f}deg) generation={:016X}",
                rigidPositionError,
                rigidRotationError,
                effectiveGenerationKey);
        }
        _visuals.lastRenderedWeaponWorld = weaponNode->world;
        _visuals.hasLastRenderedWeaponWorld = isFiniteTransform(_visuals.lastRenderedWeaponWorld);
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _scope.cameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_scope.cameraDebugSnapshot, effectiveGenerationKey,
                NativeScopeCameraWriteSource::WeaponVisualAuthority, scopeCameraFollow, scopeCameraResult,
                scopeTargetReady ? _scope.anchorSource : native_scope_sight_anchor_policy::AnchorSource::None);
        }
        return true;
    }

    bool TwoHandedGrip::clearWeaponCollisionHandAuthority(const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        if (!_visuals.weaponCollisionHandAuthorityLive[index]) {
            return true;
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::clearExternalHandWorldTransform(
                WEAPON_COLLISION_HAND_TAG,
                handFromBool(isLeft))) {
            return false;
        }
        _visuals.weaponCollisionHandAuthorityLive[index] = false;
        return true;
    }

    void TwoHandedGrip::beginWeaponCollisionPresentationFrame()
    {
        _visuals.weaponCollisionHandPresentationFromPreviousFrame =
            _visuals.weaponCollisionHandAuthorityLive;
        const bool leftCleared = clearWeaponCollisionHandAuthority(true);
        const bool rightCleared = clearWeaponCollisionHandAuthority(false);
        if (!leftCleared || !rightCleared) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: previous-frame dynamic weapon collision hand authority clear failed left={} right={} live(L/R)={}/{}",
                leftCleared ? "ok" : "failed",
                rightCleared ? "ok" : "failed",
                _visuals.weaponCollisionHandAuthorityLive[0],
                _visuals.weaponCollisionHandAuthorityLive[1]);
        }
    }

    bool TwoHandedGrip::applyWeaponCollisionResolvedAuthority(
        RE::NiNode* weaponNode,
        const RE::NiTransform& resolvedWeaponWorld,
        const std::uint64_t authorityGenerationKey)
    {
        if (!weaponNode ||
            !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(resolvedWeaponWorld)) {
            return false;
        }

        const RE::NiTransform requestedWeaponWorld = weaponNode->world;
        const auto attachedHands =
            dynamic_weapon_collision_policy::selectAttachedHands(
                _session.state == TwoHandedState::PartCarry,
                _session.state == TwoHandedState::Gripping ||
                    _session.state == TwoHandedState::PrimaryOnly,
                isFiringHandLeft(),
                partGrip(true).active,
                partGrip(false).active);

        struct CollisionHandPulse
        {
            RE::NiTransform targetWorld{};
            bool isLeft{ false };
            bool requested{ false };
            bool targetValid{ false };
            bool applied{ false };
            bool retained{ false };
        };
        std::array<CollisionHandPulse, 2> pulses{
            CollisionHandPulse{
                .isLeft = true,
                .requested = attachedHands.left,
            },
            CollisionHandPulse{
                .isLeft = false,
                .requested = attachedHands.right,
            },
        };

        const bool anyHandRequested = attachedHands.left || attachedHands.right;
        bool handTargetsReady =
            !anyHandRequested || frik_visual_authority::isAvailable();
        for (auto& pulse : pulses) {
            if (!pulse.requested) {
                continue;
            }
            RE::NiTransform requestedHandWorld{};
            const bool requestedHandValid =
                tryGetRootFlattenedHandBoneTransform(
                    pulse.isLeft,
                    requestedHandWorld);
            pulse.targetWorld =
                dynamic_weapon_collision_policy::reframeAttachedHand(
                    requestedWeaponWorld,
                    resolvedWeaponWorld,
                    requestedHandWorld);
            pulse.targetValid =
                requestedHandValid &&
                isUsableHandAuthorityTransform(requestedHandWorld) &&
                isUsableHandAuthorityTransform(pulse.targetWorld);
            handTargetsReady = handTargetsReady && pulse.targetValid;
        }

        bool handPulsesSucceeded = handTargetsReady;
        if (handTargetsReady) {
            for (auto& pulse : pulses) {
                if (!pulse.requested) {
                    continue;
                }
                const auto hand = handFromBool(pulse.isLeft);
                pulse.applied =
                    frik_visual_authority::applyExternalHandWorldTransform(
                        WEAPON_COLLISION_HAND_TAG,
                        hand,
                        pulse.targetWorld,
                        WEAPON_COLLISION_HAND_PRIORITY);
                /*
                 * Retain the high-priority result through rendering. Clearing
                 * it here synchronously reselects the live priority-100 firing
                 * and support targets, erasing the collision correction. The
                 * next PhysicsInteraction frame clears this tag, then uses the
                 * retained witness to reconstruct physical intent from the
                 * unaffected hand driver. FRIK's current root was produced
                 * before ROCK's clear and therefore cannot be sampled here.
                 */
                pulse.retained = pulse.applied;
                if (pulse.retained) {
                    _visuals.weaponCollisionHandAuthorityLive[
                        pulse.isLeft ? 0u : 1u] = true;
                }
                handPulsesSucceeded =
                    handPulsesSucceeded && pulse.applied && pulse.retained;
            }
        } else {
            for (const auto& pulse : pulses) {
                if (pulse.requested) {
                    handPulsesSucceeded =
                        clearWeaponCollisionHandAuthority(pulse.isLeft) &&
                        handPulsesSucceeded;
                }
            }
        }

        /*
         * A firing-hand pulse can propagate through the weapon's native parent
         * chain. Publish the solver-authoritative weapon last so the final
         * rendered weapon pose is exact while both hands keep the rigid
         * pre-collision weapon-local relationship captured above.
         */
        const bool weaponPublished = applyWeaponVisualAuthority(
            weaponNode,
            resolvedWeaponWorld,
            authorityGenerationKey,
            false);
        if (!weaponPublished || !handPulsesSucceeded) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                1000,
                "TwoHandedGrip: dynamic weapon collision group publication incomplete weapon={} hands={} left(req/target/apply/live)={}/{}/{}/{} right(req/target/apply/live)={}/{}/{}/{} state={} firingHand={}",
                weaponPublished ? "ok" : "failed",
                handPulsesSucceeded ? "ok" : "failed",
                pulses[0].requested,
                pulses[0].targetValid,
                pulses[0].applied,
                pulses[0].retained,
                pulses[1].requested,
                pulses[1].targetValid,
                pulses[1].applied,
                pulses[1].retained,
                static_cast<int>(_session.state),
                firingHandName());
        }
        return weaponPublished && handPulsesSucceeded;
    }

    bool TwoHandedGrip::applyFiringHandLockedVisual(RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        if (!weaponNode || !_firing.hasPrimaryHandWeaponLocal) {
            return false;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scope.menuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform firingHandWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _firing.primaryHandWeaponLocal);
        const auto& returningHand = _visuals.returningHands[isFiringHandLeft() ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(
                isSupportHandLeft(),
                supportPartGrip());
        const RE::NiTransform appliedFiringHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                firingHandWorld,
                true,
                _visuals.primaryHandLerp) :
            resolveLockedHandVisualTarget(
                firingHandWorld,
                acquisitionStart,
                dt,
                _visuals.primaryHandLerp);
        (void)publishAuthoredPrimaryFiringGripFingerPose(isFiringHandLeft());
        RE::NiTransform requestedFiringHandWorld = appliedFiringHandWorld;
        if (usesLeftFiringCarry() &&
            _leftCarry.recoilSupportConstrainedThisUpdate) {
            /*
             * hFRIK applies the accepted Direct recoil delta to every external
             * primary-hand target. The full two-hand solver has already
             * consumed that delta above, so pre-remove it from this request;
             * hFRIK's publication composes it back to the exact constrained
             * hand seat instead of kicking the hand a second time.
             */
            requestedFiringHandWorld = transform_math::composeTransforms(
                transform_math::invertTransform(
                    _leftCarry.recoilWorldDelta),
                appliedFiringHandWorld);
            if (!isUsableHandAuthorityTransform(
                    requestedFiringHandWorld)) {
                ROCK_LOG_SAMPLE_WARN(
                    Weapon,
                    1000,
                    "TwoHandedGrip: left supported recoil hand precompensation was invalid");
                return false;
            }
        }
        const bool applied =
            frik_visual_authority::applyExternalHandWorldTransform(
                PRIMARY_GRIP_TAG,
                handFromBool(isFiringHandLeft()),
                requestedFiringHandWorld,
                GRIP_HAND_POSE_PRIORITY);
        recordLockedHandAuthorityAttempt(
            isFiringHandLeft(),
            LockedHandAuthorityRole::PrimaryGrip,
            requestedFiringHandWorld,
            liveHandWorld,
            true,
            applied);
        if (applied) {
            if (usesLeftFiringCarry()) {
                _firing.leftHandWorldActive = true;
            }
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, isFiringHandLeft());
            clearHandVisualReturn(isFiringHandLeft(), "firing-grip-authority-acquired", false);
            recordPublishedHandWorld(isFiringHandLeft(), appliedFiringHandWorld);
        }
        return applied;
    }

    bool TwoHandedGrip::applyPartGripLockedVisual(bool isLeft, RE::NiNode* weaponNode, float dt, const RE::NiTransform* liveHandWorld)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!weaponNode || !grip.active || !grip.hasHandWeaponLocal) {
            return false;
        }
        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scope.menuOpenThisFrame)) {
            return true;
        }
        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        const RE::NiTransform partGripHandWorld = resolvePartGripHandWorld(grip, weaponNode);
        const auto& returningHand = _visuals.returningHands[isLeft ? 0u : 1u].transition;
        const RE::NiTransform* acquisitionStart =
            returningHand.active && isUsableHandAuthorityTransform(returningHand.lastApplied) ?
            &returningHand.lastApplied :
            liveHandWorld;
        const bool synchronizedDynamicAcquisition =
            dynamicSupportAcquisitionMatches(isLeft, grip);
        const RE::NiTransform appliedHandWorld =
            synchronizedDynamicAcquisition ?
            resolveDynamicSupportAcquisitionHandTarget(
                partGripHandWorld,
                false,
                grip.visualLerp) :
            resolveLockedHandVisualTarget(
                partGripHandWorld,
                acquisitionStart,
                dt,
                grip.visualLerp);
        const bool applied =
            frik_visual_authority::applyExternalHandWorldTransform(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft),
                appliedHandWorld,
                GRIP_HAND_POSE_PRIORITY);
        recordLockedHandAuthorityAttempt(
            isLeft,
            LockedHandAuthorityRole::SupportGrip,
            appliedHandWorld,
            liveHandWorld,
            true,
            applied);
        if (applied) {
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, isLeft);
            clearHandVisualReturn(isLeft, "part-grip-authority-acquired", false);
            recordPublishedHandWorld(isLeft, appliedHandWorld);
        }
        return applied;
    }

    bool TwoHandedGrip::applyLockedHandVisualAuthority(
        RE::NiNode* weaponNode,
        bool applyPrimaryHand,
        bool applySupportHand,
        float dt,
        const RE::NiTransform* livePrimaryHandWorld,
        const RE::NiTransform* liveSupportHandWorld)
    {
        if (!weaponNode) {
            return false;
        }

        if (!scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scope.menuOpenThisFrame)) {
            return true;
        }

        if (!frik_visual_authority::isAvailable()) {
            return false;
        }

        if (!applyPrimaryHand && !applySupportHand) {
            return true;
        }

        const bool supportHandIsLeft = isSupportHandLeft();
        bool primaryApplied = true;
        bool supportApplied = true;
        if (applyPrimaryHand) {
            primaryApplied = applyFiringHandLockedVisual(weaponNode, dt, livePrimaryHandWorld);
        }
        if (applySupportHand) {
            supportApplied = applyPartGripLockedVisual(supportHandIsLeft, weaponNode, dt, liveSupportHandWorld);
        }
        if (primaryApplied && supportApplied) {
            return true;
        }

        ROCK_LOG_WARN(Weapon,
            "TwoHandedGrip: locked hand authority publication failed primary={} support={} "
            "firingHand={} supportHand={} state={} scopeMenu={} primaryFrame={} supportGrip={} supportFrame={}",
            primaryApplied ? "ok" : "failed",
            supportApplied ? "ok" : "failed",
            firingHandName(),
            supportHandIsLeft ? "left" : "right",
            static_cast<int>(_session.state),
            _scope.menuOpenThisFrame ? "open" : "closed",
            _firing.hasPrimaryHandWeaponLocal ? "ready" : "missing",
            partGrip(supportHandIsLeft).active ? "active" : "inactive",
            partGrip(supportHandIsLeft).hasHandWeaponLocal ? "ready" : "missing");

        if (applyPrimaryHand && primaryApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, isFiringHandLeft());
        }
        if (applySupportHand && supportApplied) {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip, supportHandIsLeft);
        }
        return false;
    }

    void TwoHandedGrip::publishGripHandPoses(bool isLeft)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }

        const WeaponPartGrip& grip = partGrip(isLeft);
        if (weapon_visual_authority_math::shouldPublishTwoHandedGripPose(weapon_visual_authority_math::LockedHandRole::Support) && grip.hasFingerPose) {
            const auto handPose = grip.hasFingerSplay ?
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
            (void)frik_visual_authority::setHandPoseCustomWithPriority(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft),
                handPose,
                GRIP_HAND_POSE_PRIORITY);
        }

        if (grip.hasFingerLocalTransforms) {
            frik_visual_authority::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = grip.fingerLocalTransformMask;
            for (std::size_t i = 0; i < grip.fingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = grip.fingerLocalTransforms[i];
            }
            (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(SUPPORT_GRIP_TAG, handFromBool(isLeft), &overrideData, GRIP_HAND_POSE_PRIORITY);
        }
    }

    void TwoHandedGrip::clearPrimaryGripFingerPose(
        const bool isLeft,
        const bool preserveAuthoredFingerPose)
    {
        const bool authoredPoseMatchesHand =
            _firing.authoredFingerPosePublished &&
            _firing.publishedFingerPoseIsLeft == isLeft;
        if (authoredPoseMatchesHand && !preserveAuthoredFingerPose) {
            clearAuthoredPrimaryFiringGripFingerPose();
        } else if (!authoredPoseMatchesHand) {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(isLeft));
        }
    }

    void TwoHandedGrip::clearPrimaryGripWorldAuthority(
        const bool isLeft)
    {
        if (isLeft) {
            _firing.leftHandWorldActive = false;
        } else {
            _firing.authoredHandWorldActive = false;
            _firing.authoredHandWorldRefreshed = false;
        }
        _visuals.hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
        if (_scope.menuOpenThisFrame || _scope.menuClosedThisFrame) {
            deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, isLeft);
        } else {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, isLeft);
        }
    }

    void TwoHandedGrip::clearPrimaryDetachVisualAuthority(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(PRIMARY_DETACH_TAG, handFromBool(isLeft));
        if (_scope.menuOpenThisFrame || _scope.menuClosedThisFrame) {
            deferScopeHandAuthorityClear(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach, isLeft);
        } else {
            (void)clearHandAuthorityRoleNow(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryDetach, isLeft);
        }
    }

    bool TwoHandedGrip::blockFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", true)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose suppressed");
            return true;
        }
        return false;
    }

    void TwoHandedGrip::restoreFrikPrimaryWeaponPose()
    {
        if (frik_visual_authority::blockPrimaryHandWeaponPose("ROCK_PrimaryDetach", false)) {
            ROCK_LOG_DEBUG(Weapon, "FRIK primary weapon pose restored");
        }
    }
}

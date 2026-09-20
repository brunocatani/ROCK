#include "physics-interaction/weapon/TwoHandedGripInternal.h"
#include "physics-interaction/weapon/WeaponAimBasis.h"

// Hand and weapon visual transitions: visual returns, locked-hand visuals, grip hand pose publication, weapon visual/collision-resolved authority application, and FRIK primary pose blocking.

namespace rock
{
    void TwoHandedGrip::publishPhysicalRightNativeWeaponIntent(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 ||
            usesLeftFiringCarry() || ownsWeaponTransform()) {
            return;
        }

        // Only a successful authored alignment from THIS frame owns this
        // position-only pose. A retained canonical is not a fresh publication.
        if (_firing.authoredHandWorldRefreshed) {
            RE::NiTransform driverWorld{};
            const bool driverValid = tryGetAuthoredPrimaryTrackedFiringHandWorld(driverWorld);
            _recoil.rightWeaponBase = weaponNode->world;
            _recoil.rightHandBase = _visuals.lastPublishedHandWorld[1];
            _recoil.rightBaseValid = _visuals.hasLastPublishedHandWorld[1] &&
                isFiniteTransform(_recoil.rightWeaponBase) &&
                isUsableHandAuthorityTransform(_recoil.rightHandBase);
            _recoil.rightBaseSource = dynamic_weapon_collision_policy::VisualIntentSource::AuthoredPrimary;
            if (_visuals.weaponIntentObserver && isFiniteTransform(weaponNode->world)) {
                _visuals.weaponIntentObserver(_visuals.weaponIntentObserverContext, weaponNode,
                    weaponNode->world, currentWeaponGenerationKey,
                    dynamic_weapon_collision_policy::VisualIntentSource::AuthoredPrimary, driverValid ? &driverWorld : nullptr);
            }
            return;
        }

        RE::NiTransform physicalRightHandWorld{};
        RE::NiTransform requestedWeaponWorld{};
        auto* rightHand = resolveFirstPersonHandNode(false);
        if (!tryGetSolverHandTransform(false, physicalRightHandWorld) ||
            !isUsableHandAuthorityTransform(physicalRightHandWorld) ||
            !dynamic_weapon_collision_policy::reconstructNativeIntent<RE::NiAVObject>(
                weaponNode, rightHand, physicalRightHandWorld, requestedWeaponWorld)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "DWC intent unavailable: generation={:016X} source=native-physical-hand reason=driver-or-native-ancestry",
                currentWeaponGenerationKey);
            return;
        }

        // This source is identical whether the last collision residual was
        // zero, sub-threshold, or blocked by a wall. Later managed grip/return
        // publications may replace it as part of their existing ownership.
        _recoil.rightWeaponBase = requestedWeaponWorld;
        _recoil.rightHandBase = physicalRightHandWorld;
        _recoil.rightBaseValid = true;
        _recoil.rightBaseSource = dynamic_weapon_collision_policy::VisualIntentSource::NativePhysicalHand;
        if (_visuals.weaponIntentObserver) {
            _visuals.weaponIntentObserver(_visuals.weaponIntentObserverContext, weaponNode,
                requestedWeaponWorld, currentWeaponGenerationKey,
                dynamic_weapon_collision_policy::VisualIntentSource::NativePhysicalHand, &physicalRightHandWorld);
        }
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
        if ((!isLeft && (_firing.authoredHandWorldActive || _recoil.rightHandClaimActive)) ||
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
        if (!frik_visual_authority::publishHandWorld(
                RETURN_HAND_TAG,
                handFromBool(isLeft),
                state.start,
                RETURN_HAND_VISUAL_PRIORITY)) {
            state.clear();
            (void)frik_visual_authority::clearHandWorld(RETURN_HAND_TAG, handFromBool(isLeft));
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
        // A return that starts at the grip pose pins the hand there until it
        // advances, and a scope provider can hold the vanilla ScopeMenu open
        // long after the player stops aiming. Under FRIK API v2.3 the scoped
        // solver hand frame is a valid target, so a hand no role wants keeps
        // returning while the menu is open.
        const scope_safe_hand_frame_math::DesiredHandAuthorityInput ownership{
            .gripping = _session.state == TwoHandedState::Gripping,
            .primaryHandAuthorityEnabled =
                weapon_support_authority_policy::supportGripAppliesPrimaryHandAuthority(_session.authorityMode),
            .firingHandIsLeft = isFiringHandLeft(),
            .leftPartGripActive = partGrip(true).active,
            .rightPartGripActive = partGrip(false).active,
        };

        for (const bool isLeft : { true, false }) {
            const std::size_t index = isLeft ? 0u : 1u;
            auto& state = _visuals.returningHands[index].transition;
            if (!state.active) {
                continue;
            }
            if (_scope.menuOpenThisFrame &&
                scope_safe_hand_frame_math::desiredRolesForHand(ownership, isLeft) != 0) {
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
                    return frik_visual_authority::publishHandWorld(
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
                (void)frik_visual_authority::clearHandWorld(RETURN_HAND_TAG, handFromBool(isLeft));
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
        (void)frik_visual_authority::clearHandWorld(RETURN_HAND_TAG, handFromBool(isLeft));
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

    void TwoHandedGrip::beginWeaponVisualReturn(const char* reason, const bool keepFiringHandAttached)
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
        const bool awaitingParentRestore = _leftCarry.weaponNodeReparented;
        // The parent the carry leaves the node under until FRIK restores the game's hand.
        RE::NiNode* const carryParent = awaitingParentRestore ? _session.weaponNode->parent : nullptr;
        if (awaitingParentRestore) {
            // FRIK restores the game's own handedness setting, not the right hand.
            bool gameLeftHanded = false;
            if (!frik_visual_authority::tryResolveHandIsLeft(frik_visual_authority::Hand::Primary, gameLeftHanded)) {
                ROCK_LOG_SAMPLE_WARN(Weapon,
                    5000,
                    "TwoHandedGrip: bLeftHandedMode:VR unavailable; the weapon return assumes FRIK restores the right-hand parent");
            }
            nativeParent = resolveFirstPersonHandNode(gameLeftHanded);
        }
        if (!nativeParent) {
            return;
        }

        const RE::NiTransform startLocal = weapon_visual_authority_math::worldTargetToParentLocal(nativeParent->world, startWorld);
        if (!isFiniteTransform(startLocal)) {
            return;
        }

        /*
         * Release the left-carry parent request before beginning the overlay:
         * FRIK restores the game's parent hand in its next skeleton pass,
         * before this return's first advance, so the parent-local return
         * below lands on the restored parent. The return writes world
         * transforms, so the frames the node still hangs under the carry
         * parent render the same pose. The weapon-node write block stays held
         * for the return (finalizeFrikWeaponOwnershipForFrame).
         */
        releaseFiringHandWeaponNodeOwnership(_session.weaponNode);

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
        returnState.keepFiringHandAttached = keepFiringHandAttached &&
            usesNativeRightCarry() && _firing.hasPrimaryHandWeaponLocal &&
            isFiniteTransform(_firing.primaryHandWeaponLocal);
        if (returnState.keepFiringHandAttached) {
            returnState.firingHandWeaponLocal = _firing.primaryHandWeaponLocal;
        }
        returnState.carryParent = carryParent;
        returnState.localTransition.begin(startLocal);
        returnState.localTransition.durationSeconds = hand_visual_lerp_math::computeVisualReturnDuration(
            startLocal,
            returnState.lastTargetLocal,
            hand_visual_lerp_math::kEquippedWeaponReturnConfig);
        returnState.localTransition.durationInitialized = true;
        // The grip solver stops publishing on this release frame; the normal
        // return update has already run. Publish the initial return pose now
        // so collision authority retains its target and any latched bipod.
        if (!applyWeaponReturnVisualAuthority(returnState, startWorld)) {
            return;
        }
        _visuals.returningWeapon = returnState;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: weapon return started reason={} target={} distance={:.2f}gu angle={:.1f}deg duration={:.3f}s",
            reason ? reason : "unknown",
            followsAuthoredPrimaryGrip ? "authored-primary" : "native-baseline",
            hand_visual_lerp_math::distanceGameUnits(startLocal.translate, returnState.lastTargetLocal.translate),
            hand_visual_lerp_math::rotationDistanceDegrees(startLocal, returnState.lastTargetLocal),
            _visuals.returningWeapon.localTransition.durationSeconds);
    }

    bool TwoHandedGrip::applyWeaponReturnVisualAuthority(
        const ReturningWeaponVisualState& state,
        const RE::NiTransform& weaponWorld)
    {
        if (state.keepFiringHandAttached) {
            const RE::NiTransform handWorld = transform_math::composeTransforms(
                weaponWorld, state.firingHandWeaponLocal);
            if (!isUsableHandAuthorityTransform(handWorld) ||
                !frik_visual_authority::publishHandWorld(
                    PRIMARY_GRIP_TAG, handFromBool(false), handWorld,
                    GRIP_HAND_POSE_PRIORITY)) {
                clearPrimaryGripWorldAuthority(false);
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                    "TwoHandedGrip: firing hand could not follow weapon return");
                return false;
            }
            // Retain the seat through the completed return's render frame.
            // The next authored-primary frame replaces it or clears it via
            // finishAuthoredPrimaryFiringGripFrame when native carry resumes.
            _firing.authoredHandWorldActive = true;
            _firing.authoredHandWorldRefreshed = true;
            recordScopeHandAuthorityPublication(
                scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, false);
            clearHandVisualReturn(false, "firing-hand-follows-weapon-return", false);
            recordPublishedHandWorld(false, handWorld);
        }
        // Either hand's synchronous arm solve can move the weapon through
        // its parent. Publish the common weapon pose after the hand claim.
        if (!applyWeaponVisualAuthority(state.weaponNode, weaponWorld, state.weaponGenerationKey)) {
            if (state.keepFiringHandAttached) {
                clearPrimaryGripWorldAuthority(false);
            }
            return false;
        }
        return true;
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
        // Native intent was sampled before this return publishes. It cannot
        // become a one-hand recoil base on the frame the return completes;
        // authored carry supplies a fresh base on the following frame.
        _recoil.rightBaseValid = false;
        // After a left carry FRIK restores the game's parent hand in its next
        // skeleton pass; until then the node legitimately hangs under the
        // parent the carry left it with. Any other parent ends the return.
        const bool parentChanged = currentWeaponNode &&
            currentWeaponNode->parent != state.nativeParent &&
            !(state.carryParent && currentWeaponNode->parent == state.carryParent);
        if (!runtime_state::isLocalSkeletonReady() ||
            !currentWeaponNode ||
            currentWeaponNode != state.weaponNode ||
            currentWeaponGenerationKey != state.weaponGenerationKey ||
            currentEquippedWeaponOwnershipKey != state.equippedWeaponOwnershipKey ||
            !state.nativeParent ||
            !isFiniteTransform(state.nativeParent->world) ||
            parentChanged) {
            clearAllVisualReturns("weapon-identity-or-parent-changed", true, true);
            return;
        }
        if (state.carryParent && currentWeaponNode->parent == state.carryParent) {
            if (state.framesUnderCarryParent != 0xFF) {
                ++state.framesUnderCarryParent;
            }
            if (state.framesUnderCarryParent >= 3) {
                // Diagnostic only: FRIK restores the parent within two frames.
                ROCK_LOG_SAMPLE_DEBUG(Weapon,
                    2000,
                    "TwoHandedGrip: weapon return still under carry parent after {} frames",
                    state.framesUnderCarryParent);
            }
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
            [this, &state](const RE::NiTransform& transform) {
                const RE::NiTransform returnedWeaponWorld =
                    transform_math::composeTransforms(
                        state.nativeParent->world,
                        transform);
                return applyWeaponReturnVisualAuthority(state, returnedWeaponWorld);
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
                state.followsAuthoredPrimaryGrip || state.keepFiringHandAttached;
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
        const bool keptFiringHandAttached = _visuals.returningWeapon.keepFiringHandAttached;
        RE::NiNode* returnNode = _visuals.returningWeapon.weaponNode;
        _visuals.returningWeapon = {};
        if (keptFiringHandAttached && !preserveAuthoredPrimaryPose) {
            clearPrimaryGripWorldAuthority(false);
        }
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
        clearWeaponPoseHandoffBlend(reason, logCancellation);
    }

    void TwoHandedGrip::armWeaponPoseHandoffBlend(const char* reason)
    {
        auto& state = _visuals.weaponHandoff;
        state = {};
        if (!hand_visual_lerp_math::kEquippedWeaponReturnEnabled ||
            _session.weaponGenerationKey == 0 ||
            !_visuals.hasLastRenderedWeaponWorld ||
            !isInvertibleTransform(_visuals.lastRenderedWeaponWorld)) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: weapon handoff blend not armed reason={} renderedFrame={}",
                reason ? reason : "unknown",
                _visuals.hasLastRenderedWeaponWorld ? "available" : "missing");
            return;
        }
        state.armed = true;
        state.weaponGenerationKey = _session.weaponGenerationKey;
        ROCK_LOG_DEBUG(Weapon,
            "TwoHandedGrip: weapon handoff blend armed reason={} generation={:016X}",
            reason ? reason : "unknown",
            state.weaponGenerationKey);
    }

    RE::NiTransform TwoHandedGrip::resolveWeaponPoseHandoffBlend(
        const RE::NiTransform& solvedWeaponWorld,
        const float dt)
    {
        auto& state = _visuals.weaponHandoff;
        if (!state.armed && !state.residual.active) {
            return solvedWeaponWorld;
        }
        if (state.weaponGenerationKey != _session.weaponGenerationKey ||
            !isInvertibleTransform(solvedWeaponWorld)) {
            clearWeaponPoseHandoffBlend("weapon-identity-or-solve-invalid", true);
            return solvedWeaponWorld;
        }

        const RE::NiTransform identity =
            transform_math::makeIdentityTransform<RE::NiTransform>();
        if (state.armed) {
            state.armed = false;
            const RE::NiTransform residualLocal =
                hand_visual_lerp_math::captureHandoffResidualLocal(
                    solvedWeaponWorld,
                    _visuals.lastRenderedWeaponWorld);
            if (!isFiniteTransform(residualLocal)) {
                clearWeaponPoseHandoffBlend("non-finite-residual", true);
                return solvedWeaponWorld;
            }
            const float residualDistance =
                hand_visual_lerp_math::distanceGameUnits(
                    residualLocal.translate,
                    identity.translate);
            const float residualAngle =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    residualLocal,
                    identity);
            state.residual.begin(residualLocal);
            state.residual.durationSeconds =
                hand_visual_lerp_math::computeVisualReturnDuration(
                    residualLocal,
                    identity,
                    hand_visual_lerp_math::kEquippedWeaponReturnConfig);
            state.residual.durationInitialized = true;
            if (state.residual.durationSeconds <= 0.0f) {
                // Inside the exact-handoff tolerance (the calibrated detach
                // pose handoff): publish the solve unchanged.
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: weapon handoff blend skipped residual=({:.3f}gu,{:.2f}deg)",
                    residualDistance,
                    residualAngle);
                state = {};
                return solvedWeaponWorld;
            }
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: weapon handoff blend started residual=({:.2f}gu,{:.1f}deg) duration={:.3f}s",
                residualDistance,
                residualAngle,
                state.residual.durationSeconds);
            // First publication continues the rendered pose exactly; the
            // residual starts decaying on the next update.
            const RE::NiTransform continued =
                hand_visual_lerp_math::applyHandoffResidual(
                    solvedWeaponWorld,
                    residualLocal,
                    0.0f);
            if (!isFiniteTransform(continued)) {
                clearWeaponPoseHandoffBlend("non-finite-continued-pose", true);
                return solvedWeaponWorld;
            }
            return continued;
        }

        const auto advanced = hand_visual_lerp_math::advanceVisualReturn(
            state.residual,
            identity,
            dt,
            hand_visual_lerp_math::kEquippedWeaponReturnConfig);
        if (advanced.reachedTarget) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: weapon handoff blend completed duration={:.3f}s",
                state.residual.durationSeconds);
            state = {};
            return solvedWeaponWorld;
        }
        const RE::NiTransform blended =
            transform_math::composeTransforms(
                solvedWeaponWorld,
                advanced.transform);
        if (!isFiniteTransform(blended)) {
            clearWeaponPoseHandoffBlend("non-finite-blended-pose", true);
            return solvedWeaponWorld;
        }
        return blended;
    }

    void TwoHandedGrip::clearWeaponPoseHandoffBlend(const char* reason, const bool logCancellation)
    {
        auto& state = _visuals.weaponHandoff;
        const bool wasPending = state.armed || state.residual.active;
        state = {};
        if (wasPending && logCancellation) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: weapon handoff blend cancelled reason={}",
                reason ? reason : "unknown");
        }
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

        // Use the same controller aim as AuthoredPrimaryFiringGripRuntime.
        // At AfterArmSolve the node can contain graph output or our previous
        // return, carried through FRIK's arm update. Neither is the endpoint
        // that authored carry will publish when this transition completes.
        if (weaponNode->parent != nativeParent) {
            return false;
        }
        RE::NiTransform nativeWeaponWorld{};
        const bool meleeWeapon = _recoil.weaponEvidence.resolved && _recoil.weaponEvidence.sizeClass == WeaponSizeClass::Melee;
        const bool aimAvailable = meleeWeapon ?
            weapon_aim_basis::tryResolveMeleeWorld(trackedRightHandWorld, _firing.rightCanonicalHandWeaponLocal,
                weaponNode->world.scale, nativeWeaponWorld) :
            tryGetRightWeaponAimWorld(weaponNode->world.scale, nativeWeaponWorld);
        if (!aimAvailable) {
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
         * so the support hand shares the weapon correction alpha. An already
         * seated firing hand has a completed lerp and rides that weapon pose.
         */
        if (!g_rockConfig.rockWeaponSupportGripHandLerpEnabled) {
            state = {};
            return targetWorld;
        }

        if (!state.initialized) {
            const RE::NiTransform startWorld = (liveHandWorld && isFiniteTransform(*liveHandWorld)) ? *liveHandWorld : targetWorld;
            /*
             * The INI keys map the seat distance. The angle mapping shares the
             * equipped-weapon return bounds so a hand that is already at the
             * grip point but twisted (a firing-grip reattach with the palm on
             * the grip) still slerps into the seat instead of snapping.
             */
            const float durationSeconds =
                hand_visual_lerp_math::computeVisualReturnDuration(
                    startWorld,
                    targetWorld,
                    hand_visual_lerp_math::VisualReturnConfig{
                        .minSeconds = g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
                        .maxSeconds = g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
                        .minDistanceGameUnits = g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
                        .maxDistanceGameUnits = g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance,
                        .minAngleDegrees = hand_visual_lerp_math::kEquippedWeaponReturnConfig.minAngleDegrees,
                        .maxAngleDegrees = hand_visual_lerp_math::kEquippedWeaponReturnConfig.maxAngleDegrees,
                    });
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
        const bool notifyVisualIntentObserver,
        const bool recordRenderedWeaponWorld,
        const dynamic_weapon_collision_policy::VisualIntentSource intentSource)
    {
        if (!weaponNode) {
            return false;
        }

        const std::uint64_t effectiveGenerationKey = authorityGenerationKey != 0 ? authorityGenerationKey : _session.weaponGenerationKey;
        if (notifyVisualIntentObserver && _visuals.weaponIntentObserver) {
            RE::NiTransform driverWorld{};
            const bool driverValid = intentSource == dynamic_weapon_collision_policy::VisualIntentSource::AuthoredPrimary ?
                tryGetAuthoredPrimaryTrackedFiringHandWorld(driverWorld) :
                tryGetSolverHandTransform(weaponCarrierIsLeft(), driverWorld);
            _visuals.weaponIntentObserver(
                _visuals.weaponIntentObserverContext,
                weaponNode,
                solvedWeaponWorld,
                effectiveGenerationKey,
                intentSource,
                driverValid ? &driverWorld : nullptr);
        }
        const bool scopeAnchorMatchesAuthority =
            g_rockConfig.rockEnableImmersiveScopes && _scope.anchorValid &&
            _scope.anchorWeaponNode == weaponNode && _scope.anchorGenerationKey == effectiveGenerationKey;

        // This solve runs before FRIK aligns its scope camera. Only reuse the
        // calibration captured at AfterWeaponPosition; an early capture would
        // freeze a camera/weapon relationship from different frame phases.
        const NativeScopeCameraFollowCapture scopeCameraFollow = scopeAnchorMatchesAuthority ?
            captureNativeScopeCameraFollow(weaponNode) : NativeScopeCameraFollowCapture{};

        if (!moveWeaponPresentationRigidly(weaponNode, solvedWeaponWorld)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: rejected weapon visual authority because the root or bounded presentation subtree was invalid");
            return false;
        }

        const bool rigidFrameMatchesAuthority = scopeAnchorMatchesAuthority && _scope.rigidFrame.valid && _scope.rigidFrame.weaponGenerationKey == effectiveGenerationKey &&
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
                            _scope.rigidFrame.cameraWeaponLocal), _scope.rigidFrame) :
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
        if (recordRenderedWeaponWorld) {
            _visuals.lastRenderedWeaponWorld = weaponNode->world;
            _visuals.hasLastRenderedWeaponWorld = isFiniteTransform(_visuals.lastRenderedWeaponWorld);
        }
        if (g_rockConfig.rockDebugDrawNativeScopeActivation) {
            _scope.cameraDebugSnapshot = makeNativeScopeCameraDebugSnapshot(_scope.cameraDebugSnapshot, effectiveGenerationKey,
                NativeScopeCameraWriteSource::WeaponVisualAuthority, scopeCameraFollow, scopeCameraResult,
                scopeTargetReady ? _scope.anchorSource : native_scope_sight_anchor_policy::AnchorSource::None);
        }
        // FRIK's re-glue and weapon pass must skip this node from now on.
        noteFrikWeaponNodeWrite();
        return true;
    }

    bool TwoHandedGrip::clearWeaponCollisionHandAuthority(const bool isLeft)
    {
        const std::size_t index = isLeft ? 0u : 1u;
        if (!_visuals.weaponCollisionHandAuthorityLive[index]) {
            return true;
        }
        if (!frik_visual_authority::isAvailable() ||
            !frik_visual_authority::clearHandWorld(
                WEAPON_COLLISION_HAND_TAG,
                handFromBool(isLeft))) {
            return false;
        }
        _visuals.weaponCollisionHandAuthorityLive[index] = false;
        return true;
    }

    void TwoHandedGrip::beginWeaponCollisionPresentationFrame()
    {
        // Scope reconstruction and canonical capture still need to know which
        // rendered roots are contaminated. This witness never selects between
        // a raw weapon-world input and the collision-free intent publisher.
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

    bool TwoHandedGrip::tryGetSurfaceSupportPrimaryGripLocal(
        RE::NiNode* weaponNode, const std::uint64_t generation, RE::NiPoint3& outLocal) const
    {
        if (isManualOwnershipActive() && _session.state != TwoHandedState::PartCarry &&
            _session.weaponNode == weaponNode && _session.weaponGenerationKey == generation &&
            _firing.hasPrimaryHandWeaponLocal) {
            outLocal = _firing.primaryGripLocal;
        } else if (_firing.hasRightCanonicalHandWeaponLocal &&
            _firing.rightCanonicalWeaponNode == weaponNode &&
            _firing.rightCanonicalGenerationKey == generation) {
            outLocal = _firing.rightCanonicalGripWeaponLocal;
        } else {
            return false;
        }
        return dynamic_weapon_collision_policy::isFinitePoint(outLocal);
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
            /*
             * The hand ROCK requested against the requested weapon is this
             * frame's grip claim (published by the grip update earlier in
             * the frame), not the rendered bone: under FRIK API v2 the bone
             * still shows last frame's target. A hand without a claim is the
             * controller hand.
             * Only lower-layer grip targets are inputs. PAPER's animation is
             * already anchored to the resolved weapon; reframing it again
             * subtracts controller movement from a stationary bipod pose.
             */
            RE::NiTransform requestedHandWorld{};
            const bool requestedHandValid =
                frik_visual_authority::tryGetPublishedHandWorld(
                    handFromBool(pulse.isLeft),
                    requestedHandWorld,
                    WEAPON_COLLISION_HAND_TAG,
                    WEAPON_COLLISION_HAND_PRIORITY - 1) ||
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
                    frik_visual_authority::publishHandWorld(
                        WEAPON_COLLISION_HAND_TAG,
                        hand,
                        pulse.targetWorld,
                        WEAPON_COLLISION_HAND_PRIORITY);
                /*
                 * Retain the high-priority result through rendering. Clearing
                 * it here synchronously reselects the live priority-100 firing
                 * and support targets, erasing the collision correction. The
                 * next PhysicsInteraction frame clears this tag. Every frame's
                 * weapon intent comes from its collision-isolated driver;
                 * rendered hand roots never select that intent source.
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
        const RE::NiTransform appliedFiringHandWorld =
            resolveLockedHandVisualTarget(
                firingHandWorld,
                acquisitionStart,
                dt,
                _visuals.primaryHandLerp);
        (void)publishAuthoredPrimaryFiringGripFingerPose(isFiringHandLeft());
        const bool applied =
            frik_visual_authority::publishHandWorld(
                PRIMARY_GRIP_TAG,
                handFromBool(isFiringHandLeft()),
                appliedFiringHandWorld,
                GRIP_HAND_POSE_PRIORITY);
        recordLockedHandAuthorityAttempt(
            isFiringHandLeft(),
            LockedHandAuthorityRole::PrimaryGrip,
            appliedFiringHandWorld,
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
                grip.visualLerp) :
            resolveLockedHandVisualTarget(
                partGripHandWorld,
                acquisitionStart,
                dt,
                grip.visualLerp);
        const bool applied =
            frik_visual_authority::publishHandWorld(
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
        const bool leftCarryArmProbe =
            g_rockConfig.rockDebugGrabFrameLogging &&
            usesLeftFiringCarry() &&
            applyPrimaryHand &&
            applySupportHand &&
            _firing.hasPrimaryHandWeaponLocal;
        const LeftCarryArmSample armBeforePrimary =
            leftCarryArmProbe ? sampleLeftCarryArm() : LeftCarryArmSample{};
        const RE::NiTransform probeFiringHandWorld =
            leftCarryArmProbe ?
            weapon_visual_authority_math::weaponLocalFrameToWorld(weaponNode->world, _firing.primaryHandWeaponLocal) :
            RE::NiTransform{};
        bool primaryApplied = true;
        bool supportApplied = true;
        if (applyPrimaryHand) {
            primaryApplied = applyFiringHandLockedVisual(weaponNode, dt, livePrimaryHandWorld);
        }
        if (leftCarryArmProbe) {
            const LeftCarryArmSample armAfterPrimary = sampleLeftCarryArm();
            ROCK_LOG_DEBUG(Weapon,
                "LEFT_CARRY_ARM: frame={} applied={} target={} {} {}",
                runtime_state::currentFrame().frameIndex,
                primaryApplied ? 1 : 0,
                formatLeftCarryArmFrame(probeFiringHandWorld),
                formatLeftCarryArmSample("s0", armBeforePrimary),
                formatLeftCarryArmSample("s1", armAfterPrimary));
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
            if (!frik_visual_authority::setHandPoseCustom(
                SUPPORT_GRIP_TAG,
                handFromBool(isLeft),
                handPose,
                GRIP_HAND_POSE_PRIORITY)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                    "TwoHandedGrip: finger pose publication failed hand={} stage=joint-pose",
                    isLeft ? "left" : "right");
                (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
                return;
            }
        }

        if (grip.hasFingerLocalTransforms) {
            frik_visual_authority::FingerLocalTransformOverride overrideData{};
            overrideData.enabledMask = grip.fingerLocalTransformMask;
            for (std::size_t i = 0; i < grip.fingerLocalTransforms.size(); ++i) {
                overrideData.localTransforms[i] = grip.fingerLocalTransforms[i];
            }
            std::size_t failureIndex = grab_finger_local_transform_runtime::kInvalidFingerLocalTransformIndex;
            grab_finger_local_transform_math::FingerLocalTransformSafetyFailure failure{};
            if (!grab_finger_local_transform_runtime::fingerLocalTransformOverrideIsSafeForPublication(
                    overrideData, &failureIndex, &failure)) {
                if (failureIndex < grip.fingerLocalTransforms.size()) {
                    grab_finger_local_transform_runtime::logRejectedFingerTransform(
                        isLeft, failureIndex, "support-publication", overrideData.localTransforms[failureIndex], failure);
                }
                (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
                return;
            }
            if (!frik_visual_authority::setHandPoseCustomLocalTransforms(SUPPORT_GRIP_TAG, handFromBool(isLeft), &overrideData, GRIP_HAND_POSE_PRIORITY)) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                    "TwoHandedGrip: finger pose publication failed hand={} stage=local-transforms",
                    isLeft ? "left" : "right");
                (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
            }
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

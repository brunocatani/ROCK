#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Left-firing equipped carry: position-only carry solve, weapon node ownership/reparenting, feed-forward weapon publish, and owned recoil delivery.

namespace rock
{
    bool TwoHandedGrip::solveLeftFiringWeaponCarry(RE::NiNode* weaponNode, const float dt)
    {
        if (!weaponNode || !_firing.hasPrimaryHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the captured firing-grip frame is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform presentedHandWorld{};
        RE::NiTransform solvedWeaponWorld{};
        RE::NiTransform dampedAimCarrierWorld{};
        if (!tryResolveLeftPositionOnlyCarryFrames(
                weaponNode,
                physicalHandWorld,
                presentedHandWorld,
                solvedWeaponWorld,
                dt,
                &dampedAimCarrierWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: clearing left-firing carry because the physical hand, native aim frame, or authored seat is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform recoilDelta{};
        const bool recoilConsumed = consumeOwnedWeaponRecoil(recoilDelta);
        if (recoilConsumed) {
            solvedWeaponWorld = transform_math::composeTransforms(recoilDelta, solvedWeaponWorld);
            presentedHandWorld = transform_math::composeTransforms(solvedWeaponWorld, _firing.primaryHandWeaponLocal);
        }

        if (scope_safe_hand_frame_math::
                shouldPublishLockedHandVisualAuthority(
                    _scope.menuOpenThisFrame)) {
            (void)publishAuthoredPrimaryFiringGripFingerPose(true);
            if (!frik_visual_authority::
                    publishHandWorld(
                        PRIMARY_GRIP_TAG,
                        frik_visual_authority::Hand::Left,
                        presentedHandWorld,
                        GRIP_HAND_POSE_PRIORITY,
                        frik_visual_authority::RebaseDriver::LeftHand)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(
                    Weapon,
                    "TwoHandedGrip: clearing left-firing carry because authored left-hand presentation failed");
                transitionToInactive(false);
                return false;
            }
            _firing.leftHandWorldActive = true;
            recordScopeHandAuthorityPublication(
                scope_safe_hand_frame_math::HandAuthorityRole::
                    PrimaryGrip,
                true);
            clearHandVisualReturn(
                true,
                "left-position-only-hand-acquired",
                false);
            recordPublishedHandWorld(true, presentedHandWorld);
        }

        // The weapon is parented under LArm_Hand during left firing. Publish
        // it after the authored wrist so the native/mirrored weapon rotation
        // remains the final frame instead of inheriting the wrist correction.
        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return false;
        }

        if (recoilConsumed) {
            traceRecoilPresentation("one-hand-left");
        }
        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        return true;
    }

    bool TwoHandedGrip::tryResolveAuthoredPositionOnlySeatWorld(
        RE::NiTransform& outHandWorld) const
    {
        outHandWorld = {};
        // Carry states only: on drop/holster the session will not resume,
        // so the return keeps its physical endpoint.
        if (usesLeftFiringCarry() ||
            _firing.rightCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            (_session.state != TwoHandedState::PrimaryOnly &&
                _session.state != TwoHandedState::Gripping) ||
            !_session.weaponNode ||
            !hasRightFiringHandCanonicalFrame(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) ||
            !isFiniteTransform(_session.weaponNode->world)) {
            return false;
        }
        outHandWorld = transform_math::composeTransforms(
            _session.weaponNode->world,
            _firing.rightCanonicalHandWeaponLocal);
        return isUsableHandAuthorityTransform(outHandWorld);
    }

    bool TwoHandedGrip::captureLeftFiringDampedFollowFrame(
        RE::NiNode* weaponNode,
        const RE::NiTransform& leftWandWorld,
        const RE::NiTransform& physicalLeftHandWorld)
    {
        if (!weaponNode || weaponNode != _session.weaponNode ||
            !usesLeftFiringCarry() ||
            _session.equippedWeaponOwnershipKey == 0 ||
            !isInvertibleTransform(leftWandWorld) ||
            !isUsableHandAuthorityTransform(physicalLeftHandWorld)) {
            return false;
        }

        const RE::NiTransform handInWand =
            transform_math::composeTransforms(
                transform_math::invertTransform(leftWandWorld),
                physicalLeftHandWorld);
        constexpr float kMaxHandToWandDistance = 30.0f;
        if (!isFiniteTransform(handInWand) ||
            std::sqrt(dot(
                handInWand.translate,
                handInWand.translate)) > kMaxHandToWandDistance) {
            return false;
        }

        _firing.leftDampedFollowFrame = LeftFiringDampedFollowFrame{
            .handInWandOrientation =
                left_firing_position_only_math::orientationOnly(
                    handInWand),
            .weaponNodeIdentity = weaponNode,
            .weaponGenerationKey = _session.weaponGenerationKey,
            .weaponOwnershipKey = _session.equippedWeaponOwnershipKey,
            .valid = true,
        };
        ROCK_LOG_INFO(
            Weapon,
            "TwoHandedGrip: left firing damped-follow reference captured generation={:016X} ownership={:016X}",
            _session.weaponGenerationKey,
            _session.equippedWeaponOwnershipKey);
        return true;
    }

    bool TwoHandedGrip::hasLeftFiringDampedFollowFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode && weaponOwnershipKey != 0 &&
               _firing.leftDampedFollowFrame.valid &&
               _firing.leftDampedFollowFrame.weaponNodeIdentity == weaponNode &&
               _firing.leftDampedFollowFrame.weaponGenerationKey ==
                   weaponGenerationKey &&
               _firing.leftDampedFollowFrame.weaponOwnershipKey ==
                   weaponOwnershipKey &&
               isFiniteTransform(
                   _firing.leftDampedFollowFrame.handInWandOrientation);
    }

    bool TwoHandedGrip::tryResolveLeftPositionOnlyCarryFrames(
        RE::NiNode* weaponNode,
        RE::NiTransform& outPhysicalHandWorld,
        RE::NiTransform& outPresentedHandWorld,
        RE::NiTransform& outWeaponWorld,
        const float supportReleaseReturnAdvanceSeconds,
        RE::NiTransform* const outDampedAimCarrierWorld)
    {
        outPhysicalHandWorld = {};
        outPresentedHandWorld = {};
        outWeaponWorld = {};
        if (outDampedAimCarrierWorld) {
            *outDampedAimCarrierWorld = {};
        }
        if (!weaponNode || weaponNode != _session.weaponNode ||
            !usesLeftFiringCarry() || !_firing.hasPrimaryHandWeaponLocal ||
            !hasRightNativeWeaponAimFrame(
                weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) ||
            !isFiniteTransform(_firing.primaryHandWeaponLocal) ||
            !std::isfinite(_firing.primaryGripLocal.x) ||
            !std::isfinite(_firing.primaryGripLocal.y) ||
            !std::isfinite(_firing.primaryGripLocal.z) ||
            !isFiniteTransform(weaponNode->world)) {
            return false;
        }

        RE::NiTransform physicalDriverWorld{};
        if (!tryResolvePhysicalHandFrame(
                true,
                outPhysicalHandWorld,
                physicalDriverWorld)) {
            // Before the first left-hand publication, the scope-safe frame is
            // still a valid fallback only if ROCK owns no left visual output.
            if (_firing.leftHandWorldActive ||
                hasVisualAuthorityForHand(true) ||
                !tryGetSolverHandTransform(true, outPhysicalHandWorld) ||
                !isUsableHandAuthorityTransform(outPhysicalHandWorld)) {
                return false;
            }
        }

        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* leftWand =
            playerNodes ? playerNodes->SecondaryWandNode : nullptr;
        if (!leftWand || !isFiniteTransform(leftWand->world)) {
            return false;
        }

        if (!hasLeftFiringDampedFollowFrame(
                weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) &&
            !captureLeftFiringDampedFollowFrame(
                weaponNode,
                leftWand->world,
                outPhysicalHandWorld)) {
            return false;
        }
        const RE::NiTransform dampedAimCarrierWorld =
            left_firing_position_only_math::
                resolveDampedAimCarrierWorld(
                    leftWand->world,
                    _firing.leftDampedFollowFrame.
                        handInWandOrientation,
                    outPhysicalHandWorld);
        if (!isFiniteTransform(dampedAimCarrierWorld)) {
            return false;
        }
        if (outDampedAimCarrierWorld) {
            *outDampedAimCarrierWorld = dampedAimCarrierWorld;
        }

        RE::NiTransform leftWeaponInWand =
            left_firing_position_only_math::
                mirrorRightWeaponInWandOrientation(
                    _firing.rightNativeWeaponAimFrame.
                        weaponInWandOrientation);
        const RE::NiTransform aimTrim =
            makeLeftFiringWandAimTrim(_handlingSettings);
        leftWeaponInWand = transform_math::composeTransforms(
            aimTrim,
            leftWeaponInWand);

        RE::NiPoint3 firingGripInLeftHand =
            transform_math::localPointToWorld(
                transform_math::invertTransform(
                    _firing.primaryHandWeaponLocal),
                _firing.primaryGripLocal);
        firingGripInLeftHand.x +=
            _handlingSettings.leftFiringAimOffsetXGameUnits;
        firingGripInLeftHand.y +=
            _handlingSettings.leftFiringAimOffsetYGameUnits;
        firingGripInLeftHand.z +=
            _handlingSettings.leftFiringAimOffsetZGameUnits;
        const RE::NiPoint3 physicalGripTargetWorld =
            transform_math::localPointToWorld(
                outPhysicalHandWorld,
                firingGripInLeftHand);

        outWeaponWorld =
            left_firing_position_only_math::
                resolveWeaponWorldPositionOnly(
                    dampedAimCarrierWorld,
                    leftWeaponInWand,
                    weaponNode->world,
                    _firing.primaryGripLocal,
                    physicalGripTargetWorld);
        // The support-release return eases the rendered two-hand pose into
        // this wand-aimed pose; the authored hand rides the eased weapon.
        outWeaponWorld = resolveLeftFiringSupportReleaseReturn(
            outPhysicalHandWorld,
            outWeaponWorld,
            supportReleaseReturnAdvanceSeconds);
        outPresentedHandWorld = transform_math::composeTransforms(
            outWeaponWorld,
            _firing.primaryHandWeaponLocal);
        return isFiniteTransform(outWeaponWorld) &&
               isUsableHandAuthorityTransform(outPhysicalHandWorld) &&
               isUsableHandAuthorityTransform(outPresentedHandWorld);
    }

    void TwoHandedGrip::beginLeftFiringSupportReleaseReturn(const char* reason)
    {
        auto& state = _leftCarry.supportReleaseReturn;
        state.clear();
        if (!hand_visual_lerp_math::kEquippedWeaponReturnEnabled ||
            !usesLeftFiringCarry() ||
            !_visuals.hasLastRenderedWeaponWorld ||
            !isFiniteTransform(_visuals.lastRenderedWeaponWorld)) {
            return;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform physicalDriverWorld{};
        if (!tryResolvePhysicalHandFrame(
                true,
                physicalHandWorld,
                physicalDriverWorld) ||
            !isInvertibleTransform(physicalHandWorld)) {
            return;
        }

        // The rendered record is the previous frame's two-hand pose; the node
        // itself already holds this frame's wand-aimed basis pre-write.
        const RE::NiTransform startHandLocal =
            left_firing_position_only_math::weaponWorldToPhysicalHandLocal(
                physicalHandWorld,
                _visuals.lastRenderedWeaponWorld);
        if (!isFiniteTransform(startHandLocal)) {
            return;
        }
        state.begin(startHandLocal);
        ROCK_LOG_DEBUG(
            Weapon,
            "TwoHandedGrip: left carry weapon return started reason={}",
            reason ? reason : "unknown");
    }

    void TwoHandedGrip::clearLeftFiringSupportReleaseReturn(const char* reason)
    {
        auto& state = _leftCarry.supportReleaseReturn;
        if (!state.active) {
            return;
        }
        state.clear();
        ROCK_LOG_DEBUG(
            Weapon,
            "TwoHandedGrip: left carry weapon return cancelled reason={}",
            reason ? reason : "unknown");
    }

    RE::NiTransform TwoHandedGrip::resolveLeftFiringSupportReleaseReturn(
        const RE::NiTransform& physicalHandWorld,
        const RE::NiTransform& positionOnlyWeaponWorld,
        const float advanceSeconds)
    {
        auto& state = _leftCarry.supportReleaseReturn;
        if (!state.active) {
            return positionOnlyWeaponWorld;
        }
        if (!isInvertibleTransform(physicalHandWorld)) {
            clearLeftFiringSupportReleaseReturn("physical-hand-unavailable");
            return positionOnlyWeaponWorld;
        }

        const RE::NiTransform targetHandLocal =
            left_firing_position_only_math::weaponWorldToPhysicalHandLocal(
                physicalHandWorld,
                positionOnlyWeaponWorld);
        RE::NiTransform blendedHandLocal{};
        bool completed = false;
        if (advanceSeconds > 0.0f) {
            const auto advanced = hand_visual_lerp_math::advanceVisualReturn(
                state,
                targetHandLocal,
                advanceSeconds,
                hand_visual_lerp_math::kEquippedWeaponReturnConfig);
            blendedHandLocal = advanced.transform;
            completed = advanced.reachedTarget;
        } else if (state.durationInitialized) {
            // Basis pre-write: show this frame's blend without advancing it.
            blendedHandLocal =
                hand_visual_lerp_math::blendTransformOverDuration(
                    state.start,
                    targetHandLocal,
                    state.elapsedSeconds,
                    state.durationSeconds)
                    .transform;
        } else {
            blendedHandLocal = state.start;
        }

        const RE::NiTransform blendedWeaponWorld =
            left_firing_position_only_math::physicalHandLocalToWeaponWorld(
                physicalHandWorld,
                blendedHandLocal,
                positionOnlyWeaponWorld.scale);
        if (!isFiniteTransform(blendedWeaponWorld)) {
            clearLeftFiringSupportReleaseReturn("non-finite-return-transform");
            return positionOnlyWeaponWorld;
        }
        if (completed) {
            const float durationSeconds = state.durationSeconds;
            state.clear();
            ROCK_LOG_DEBUG(
                Weapon,
                "TwoHandedGrip: left carry weapon return completed duration={:.3f}s",
                durationSeconds);
            return positionOnlyWeaponWorld;
        }
        return blendedWeaponWorld;
    }

    RE::NiNode* TwoHandedGrip::resolveFirstPersonHandNode(const bool isLeft)
    {
        auto* firstPersonSkeleton = f4vr::getFirstPersonSkeleton();
        if (!firstPersonSkeleton) {
            return nullptr;
        }
        return f4vr::findNode(firstPersonSkeleton, isLeft ? "LArm_Hand" : "RArm_Hand");
    }

    void TwoHandedGrip::syncFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        (void)weaponNode;
        const bool wantLeftFiringCarry = usesLeftFiringCarry() &&
            (_session.state == TwoHandedState::Gripping || _session.state == TwoHandedState::PrimaryOnly);

        if (!wantLeftFiringCarry) {
            releaseFiringHandWeaponNodeOwnership(weaponNode);
            return;
        }

        if (!_leftCarry.weaponNodeOwnershipBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, true)) {
                // Fail closed: without the FRIK block the weapon node would
                // fight two per-frame owners.
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left-firing carry aborted because the FRIK weapon-node ownership block is unavailable");
                transitionToInactive(false);
                return;
            }
            _leftCarry.weaponNodeOwnershipBlockEngaged = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership blocked for left-firing carry");
        }

        /*
         * FRIK API v2.3: FRIK re-parents the weapon node under LArm_Hand and
         * keeps its own bookkeeping (first-person arm source, off-side hand
         * pose copy, recoil hand) for this request; the same operation it
         * performs for the game's own left-handed mode. The request is
         * recorded now and applied in FRIK's next skeleton pass, before any
         * frame phase, so the parent seen in this callback is still the right
         * hand. FRIK restores the game's setting when the request clears or
         * the skeleton rebuilds.
         */
        if (!_leftCarry.weaponNodeReparented) {
            if (!frik_visual_authority::setWeaponNodeParentHand(WEAPON_NODE_OWNERSHIP_TAG, frik_visual_authority::Hand::Left)) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left-firing carry aborted because FRIK refused the weapon-node parent request");
                releaseFiringHandWeaponNodeOwnership(weaponNode);
                transitionToInactive(false);
                return;
            }
            _leftCarry.weaponNodeReparented = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node parent requested under LArm_Hand for left-firing carry");
        }
    }

    void TwoHandedGrip::releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        (void)weaponNode;
        if (_leftCarry.weaponNodeReparented) {
            // FRIK restores the game's parent hand in its next skeleton pass.
            (void)frik_visual_authority::clearWeaponNodeParentHand(WEAPON_NODE_OWNERSHIP_TAG);
            _leftCarry.weaponNodeReparented = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node parent request cleared; FRIK restores RArm_Hand");
        }

        if (_leftCarry.weaponNodeOwnershipBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, false);
            _leftCarry.weaponNodeOwnershipBlockEngaged = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership restored");
        }
    }

    bool TwoHandedGrip::rebindLeftCarryFramesToWeapon(
        RE::NiNode* currentWeaponNode,
        const std::uint64_t targetWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool logMissingAimFrame)
    {
        if (usesNativeRightCarry()) {
            return true;
        }
        if (!_firing.rightNativeWeaponAimFrame.valid ||
            _firing.rightNativeWeaponAimFrame.weaponOwnershipKey !=
                currentEquippedWeaponOwnershipKey ||
            !isFiniteTransform(
                _firing.rightNativeWeaponAimFrame.weaponInWandOrientation)) {
            if (logMissingAimFrame) {
                ROCK_LOG_WARN(
                    Weapon,
                    "TwoHandedGrip: left carry cannot rebind missing native weapon aim frame generation={:016X} ownership={:016X}",
                    targetWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey);
            }
            return false;
        }
        _firing.rightNativeWeaponAimFrame.weaponNodeIdentity =
            currentWeaponNode;
        _firing.rightNativeWeaponAimFrame.weaponGenerationKey =
            targetWeaponGenerationKey;
        if (_firing.leftDampedFollowFrame.valid) {
            if (_firing.leftDampedFollowFrame.weaponOwnershipKey !=
                currentEquippedWeaponOwnershipKey) {
                _firing.leftDampedFollowFrame = {};
            } else {
                _firing.leftDampedFollowFrame.weaponNodeIdentity =
                    currentWeaponNode;
                _firing.leftDampedFollowFrame.weaponGenerationKey =
                    targetWeaponGenerationKey;
            }
        }
        return true;
    }
}

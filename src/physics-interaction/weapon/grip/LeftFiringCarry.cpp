// Left-firing equipped carry: position-only carry solve, weapon node
// ownership/reparenting, feed-forward weapon publish, and owned recoil
// delivery.
#include "physics-interaction/weapon/grip/TwoHandedGripInternal.h"

#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/weapon/grip/KnifeReverseGrip.h"
#include "physics-interaction/weapon/handling/WeaponAimBasis.h"

namespace rock
{
    bool TwoHandedGrip::publishLeftFiringFeedForwardWeaponPose(RE::NiNode* weaponNode)
    {
        if (!weaponNode || weaponNode != _session.weaponNode ||
            (_session.state != TwoHandedState::Gripping && _session.state != TwoHandedState::PrimaryOnly) ||
            !usesLeftFiringCarry() || !_firing.hasPrimaryHandWeaponLocal) {
            return false;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform presentedHandWorld{};
        RE::NiTransform feedForwardWeaponWorld{};
        if (!tryResolveLeftPositionOnlyCarryFrames(weaponNode, physicalHandWorld, presentedHandWorld,
                feedForwardWeaponWorld, 0.0f)) {
            return false;
        }
        /*
         * Basis pre-write, not a rendered frame: the state handler publishes
         * the final pose later this frame (the two-hand solve while both
         * hands hold). Recording it as rendered handed the part-carry
         * baseline the wand-aimed pose instead of the pose on screen, so the
         * weapon jumped to it when the firing hand let go.
         */
        return applyWeaponVisualAuthority(weaponNode, feedForwardWeaponWorld, 0, true, false);
    }

    bool TwoHandedGrip::solveLeftFiringWeaponCarry(RE::NiNode* weaponNode, const float dt)
    {
        if (!weaponNode || !_firing.hasPrimaryHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: clearing left-firing carry because the captured firing-grip frame is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform presentedHandWorld{};
        RE::NiTransform solvedWeaponWorld{};
        RE::NiTransform dampedAimCarrierWorld{};
        if (!tryResolveLeftPositionOnlyCarryFrames(weaponNode, physicalHandWorld, presentedHandWorld, solvedWeaponWorld,
                dt, &dampedAimCarrierWorld)) {
            _hasSolvedWeaponTransform = false;
            // A grip start retries every frame; one line per second is enough.
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "TwoHandedGrip: clearing left-firing carry because the physical hand, weapon aim, or authored seat is "
                "unavailable stage={}",
                _leftCarry.framesFailure);
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform recoilDelta{};
        const bool recoilConsumed = consumeOwnedWeaponRecoil(recoilDelta);
        if (recoilConsumed) {
            // The kick carries the hand with the weapon. Rebuilding the hand
            // from the raw seat would drop a takeover's sliding seat.
            solvedWeaponWorld = transform_math::composeTransforms(recoilDelta, solvedWeaponWorld);
            presentedHandWorld = transform_math::composeTransforms(recoilDelta, presentedHandWorld);
        }

        // A consumer draw continues from where the consumer showed the
        // weapon (the published left firing aim) and the tracked hand into
        // this seat. The left carry is always the slot-0 weapon.
        auto* equipped = f4vr::getEquippedWeaponItem();
        const std::uint32_t equippedFormId = equipped && equipped->item.object ? equipped->item.object->GetFormID() : 0;
        RE::NiTransform leftAimWorld{};
        if (equippedFormId && tryGetLeftWeaponAimWorld(weaponNode->world.scale, leftAimWorld)) {
            (void)resolveDrawPresentationBlend(equippedFormId, true, leftAimWorld, physicalHandWorld, solvedWeaponWorld,
                presentedHandWorld);
        }

        if (scope_safe_hand_frame_math::shouldPublishLockedHandVisualAuthority(_scope.menuOpenThisFrame)) {
            (void)publishAuthoredPrimaryFiringGripFingerPose(true);
            if (!frik_visual_authority::publishHandWorld(PRIMARY_GRIP_TAG, frik_visual_authority::Hand::Left,
                    presentedHandWorld, GRIP_HAND_POSE_PRIORITY)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: clearing left-firing carry because authored left-hand presentation failed");
                transitionToInactive(false);
                return false;
            }
            _firing.leftHandWorldActive = true;
            recordScopeHandAuthorityPublication(scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip, true);
            clearHandVisualReturn(true, "left-position-only-hand-acquired", false);
            recordPublishedHandWorld(true, presentedHandWorld);
        }

        // The weapon is parented under LArm_Hand during left firing. Publish
        // it after the authored wrist so the native/mirrored weapon rotation
        // remains the final frame instead of inheriting the wrist correction.
        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: clearing left-firing carry because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return false;
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        return true;
    }

    bool TwoHandedGrip::presentedPrimaryHandWeaponLocal(RE::NiTransform& outSeat) const
    {
        outSeat = _firing.primaryHandWeaponLocal;
        const bool meleeWeapon =
            _recoil.weaponEvidence.resolved && _recoil.weaponEvidence.sizeClass == WeaponSizeClass::Melee;
        const bool left = isFiringHandLeft();
        if (!meleeWeapon || !knife_reverse_grip::reversed(left, _holdingMirrorFormID))
            return false;
        // The melee mirror offset is the model's flat-side centre.
        outSeat = knife_reverse_grip::reverseSeat(_firing.primaryHandWeaponLocal, left, _holdingMirrorPlane.offset);
        return true;
    }

    bool TwoHandedGrip::tryResolveAuthoredPositionOnlySeatWorld(RE::NiTransform& outHandWorld) const
    {
        outHandWorld = {};
        // Carry states only: on drop/holster the session will not resume,
        // so the return keeps its physical endpoint.
        if (usesLeftFiringCarry() || _firing.rightCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation ||
            (_session.state != TwoHandedState::PrimaryOnly && _session.state != TwoHandedState::Gripping) ||
            !_session.weaponNode ||
            !hasRightFiringHandCanonicalFrame(_session.weaponNode, _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) ||
            !isFiniteTransform(_session.weaponNode->world)) {
            return false;
        }
        outHandWorld =
            transform_math::composeTransforms(_session.weaponNode->world, _firing.rightCanonicalHandWeaponLocal);
        return isUsableHandAuthorityTransform(outHandWorld);
    }

    bool TwoHandedGrip::captureLeftFiringDampedFollowFrame(RE::NiNode* weaponNode, const RE::NiTransform& leftWandWorld,
        const RE::NiTransform& physicalLeftHandWorld)
    {
        auto& failure = _leftCarry.framesFailure;
        if (!weaponNode || weaponNode != _session.weaponNode || !usesLeftFiringCarry() ||
            _session.equippedWeaponOwnershipKey == 0) {
            failure = "follow-frame:session";
            return false;
        }
        if (!isInvertibleTransform(leftWandWorld)) {
            failure = "follow-frame:wand";
            return false;
        }
        if (!isUsableHandAuthorityTransform(physicalLeftHandWorld)) {
            failure = "follow-frame:hand";
            return false;
        }

        const RE::NiTransform handInWand =
            transform_math::composeTransforms(transform_math::invertTransform(leftWandWorld), physicalLeftHandWorld);
        constexpr float kMaxHandToWandDistance = 30.0f;
        const float handToWand = std::sqrt(dot(handInWand.translate, handInWand.translate));
        if (!isFiniteTransform(handInWand) || handToWand > kMaxHandToWandDistance) {
            failure = "follow-frame:hand-far-from-wand";
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "TwoHandedGrip: left firing damped-follow reference refused handToWand={:.1f}gu (max {:.0f}) "
                "hand=({:.1f},{:.1f},{:.1f}) wand=({:.1f},{:.1f},{:.1f})",
                handToWand, kMaxHandToWandDistance, physicalLeftHandWorld.translate.x,
                physicalLeftHandWorld.translate.y, physicalLeftHandWorld.translate.z, leftWandWorld.translate.x,
                leftWandWorld.translate.y, leftWandWorld.translate.z);
            return false;
        }

        _firing.leftDampedFollowFrame = LeftFiringDampedFollowFrame{
            .handInWandOrientation = left_firing_position_only_math::orientationOnly(handInWand),
            .weaponNodeIdentity = weaponNode,
            .weaponGenerationKey = _session.weaponGenerationKey,
            .weaponOwnershipKey = _session.equippedWeaponOwnershipKey,
            .valid = true,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: left firing damped-follow reference captured generation={:016X} ownership={:016X}",
            _session.weaponGenerationKey, _session.equippedWeaponOwnershipKey);
        return true;
    }

    bool TwoHandedGrip::hasLeftFiringDampedFollowFrame(const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey, const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode && weaponOwnershipKey != 0 && _firing.leftDampedFollowFrame.valid &&
            _firing.leftDampedFollowFrame.weaponNodeIdentity == weaponNode &&
            _firing.leftDampedFollowFrame.weaponGenerationKey == weaponGenerationKey &&
            _firing.leftDampedFollowFrame.weaponOwnershipKey == weaponOwnershipKey &&
            isFiniteTransform(_firing.leftDampedFollowFrame.handInWandOrientation);
    }

    bool TwoHandedGrip::tryResolveLeftPositionOnlyCarryFrames(RE::NiNode* weaponNode,
        RE::NiTransform& outPhysicalHandWorld, RE::NiTransform& outPresentedHandWorld, RE::NiTransform& outWeaponWorld,
        const float supportReleaseReturnAdvanceSeconds, RE::NiTransform* const outDampedAimCarrierWorld)
    {
        outPhysicalHandWorld = {};
        outPresentedHandWorld = {};
        outWeaponWorld = {};
        if (outDampedAimCarrierWorld) {
            *outDampedAimCarrierWorld = {};
        }
        RE::NiTransform rightWeaponAimInWand{};
        const bool meleeWeapon =
            _recoil.weaponEvidence.resolved && _recoil.weaponEvidence.sizeClass == WeaponSizeClass::Melee;
        auto& failure = _leftCarry.framesFailure;
        const auto fail = [&failure](const char* stage) {
            failure = stage;
            return false;
        };
        failure = "none";
        if (!weaponNode || weaponNode != _session.weaponNode || !usesLeftFiringCarry() ||
            !_firing.hasPrimaryHandWeaponLocal) {
            return fail("session");
        }
        if (!meleeWeapon &&
            !tryResolveRightWeaponAimInWand(weaponNode, _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey, rightWeaponAimInWand)) {
            return fail("right-aim");
        }
        if (!isFiniteTransform(_firing.primaryHandWeaponLocal) || !std::isfinite(_firing.primaryGripLocal.x) ||
            !std::isfinite(_firing.primaryGripLocal.y) || !std::isfinite(_firing.primaryGripLocal.z) ||
            !isFiniteTransform(weaponNode->world)) {
            return fail("non-finite-seat");
        }
        RE::NiTransform seat{};
        const bool reversedGrip = presentedPrimaryHandWeaponLocal(seat);
        const RE::NiPoint3 gripLocal =
            reversedGrip ? computeGrabPalmPointWorldFromHandBasis(seat, true) : _firing.primaryGripLocal;

        RE::NiTransform physicalDriverWorld{};
        const char* physicalHandSource = "driver-relation";
        if (!tryResolvePhysicalHandFrame(true, outPhysicalHandWorld, physicalDriverWorld)) {
            // Before the first left-hand publication, the scope-safe frame is
            // still a valid fallback only if ROCK owns no left visual output.
            if (_firing.leftHandWorldActive)
                return fail("physical-hand:left-published");
            if (hasVisualAuthorityForHand(true))
                return fail("physical-hand:left-visual-authority");
            if (!tryGetSolverHandTransform(true, outPhysicalHandWorld) ||
                !isUsableHandAuthorityTransform(outPhysicalHandWorld)) {
                return fail("physical-hand:solver");
            }
            physicalHandSource = "solver-fallback";
        }

        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* leftWand = playerNodes ? playerNodes->SecondaryWandNode : nullptr;
        if (!leftWand || !isFiniteTransform(leftWand->world)) {
            return fail("left-wand");
        }

        if (!hasLeftFiringDampedFollowFrame(weaponNode, _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey)) {
            RE::NiTransform referenceHand = outPhysicalHandWorld;
            if (frik_hand_world_authority::weaponFilteringActive() &&
                !frik_hand_world_authority::tryGetRawHandWorld(true, referenceHand))
                return fail("follow-frame:raw-hand");
            if (!captureLeftFiringDampedFollowFrame(weaponNode, leftWand->world, referenceHand)) {
                return false; // captureLeftFiringDampedFollowFrame names the failure.
            }
            // Probe: the reference the carry's damping correction is measured
            // against, and which hand basis supplied it.
            RE::NiTransform firstPersonLeftHand{};
            const bool firstPersonValid =
                frik_visual_authority::tryGetHandWorldTransform(frik_visual_authority::Hand::Left, firstPersonLeftHand);
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: left carry reference captured physicalSource={} handInWand={:.1f}deg "
                "physicalVsFirstPerson={:.1f}deg/{:.2f}gu",
                physicalHandSource,
                tracked_hand_isolation_policy::rotationDegrees(_firing.leftDampedFollowFrame.handInWandOrientation,
                    transform_math::makeIdentityTransform<RE::NiTransform>()),
                firstPersonValid
                    ? tracked_hand_isolation_policy::rotationDegrees(outPhysicalHandWorld, firstPersonLeftHand)
                    : -1.0f,
                firstPersonValid
                    ? tracked_hand_isolation_policy::translationGameUnits(outPhysicalHandWorld, firstPersonLeftHand)
                    : -1.0f);
        }
        RE::NiTransform dampedAimCarrierWorld = left_firing_position_only_math::resolveDampedAimCarrierWorld(
            leftWand->world, _firing.leftDampedFollowFrame.handInWandOrientation, outPhysicalHandWorld);
        if (frik_hand_world_authority::weaponFilteringActive() &&
            !frik_hand_world_authority::tryGetWeaponWandWorld(true, dampedAimCarrierWorld)) {
            return fail("aim-carrier:weapon-wand");
        }
        if (!isFiniteTransform(dampedAimCarrierWorld)) {
            return fail("aim-carrier:non-finite");
        }
        if (outDampedAimCarrierWorld) {
            *outDampedAimCarrierWorld = dampedAimCarrierWorld;
        }

        RE::NiTransform leftWeaponInWand{};
        if (meleeWeapon) {
            // A melee carry follows the left hand's own natural wrist, as the
            // index-1 melee solve does. Mirroring the right hand's live wrist
            // let a weapon presented in the right hand steer this one.
            RE::NiTransform naturalLeftHandWorld{}, meleeWorld{};
            if (!tryResolveNaturalWandHandOrientationFrame(true, naturalLeftHandWorld) ||
                !weapon_aim_basis::tryResolveMeleeWorld(naturalLeftHandWorld, seat, 1.0f, meleeWorld)) {
                return fail("melee-natural-hand");
            }
            leftWeaponInWand = left_firing_position_only_math::orientationOnly(
                transform_math::composeTransforms(transform_math::invertTransform(leftWand->world), meleeWorld));
            leftWeaponInWand = transform_math::composeTransforms(
                weapon_aim_basis::meleePitchInController(g_rockConfig.rockMeleeGripPitchDegrees), leftWeaponInWand);
        } else {
            leftWeaponInWand = left_firing_position_only_math::mirrorRightWeaponInWandOrientation(rightWeaponAimInWand);
        }
        const RE::NiTransform aimTrim = makeLeftFiringWandAimTrim(_handlingSettings);
        leftWeaponInWand = transform_math::composeTransforms(aimTrim, leftWeaponInWand);

        RE::NiPoint3 firingGripInLeftHand =
            transform_math::localPointToWorld(transform_math::invertTransform(seat), gripLocal);
        firingGripInLeftHand.x += _handlingSettings.leftFiringAimOffsetXGameUnits;
        firingGripInLeftHand.y += _handlingSettings.leftFiringAimOffsetYGameUnits;
        firingGripInLeftHand.z += _handlingSettings.leftFiringAimOffsetZGameUnits;
        const RE::NiPoint3 physicalGripTargetWorld =
            transform_math::localPointToWorld(outPhysicalHandWorld, firingGripInLeftHand);

        outWeaponWorld = left_firing_position_only_math::resolveWeaponWorldPositionOnly(dampedAimCarrierWorld,
            leftWeaponInWand, weaponNode->world, gripLocal, physicalGripTargetWorld);
        // The support-release return eases the rendered two-hand pose into
        // this wand-aimed pose; the authored hand rides the eased weapon.
        outWeaponWorld = resolveLeftFiringSupportReleaseReturn(outPhysicalHandWorld, outWeaponWorld,
            supportReleaseReturnAdvanceSeconds);
        outPresentedHandWorld = transform_math::composeTransforms(outWeaponWorld, seat);
        // A firing-grip takeover continues from the pose on screen: the
        // weapon eases into this carry while the hand slides onto the seat.
        (void)resolveFiringHandoffBlend(true, outWeaponWorld, outPresentedHandWorld);

        /*
         * Probe: the carry orientation is wandL o damping o mirror(aim). Log
         * each factor so a rotated carry names the input that rotated it.
         */
        {
            RE::NiTransform firstPersonLeftHand{};
            const bool firstPersonValid =
                frik_visual_authority::tryGetHandWorldTransform(frik_visual_authority::Hand::Left, firstPersonLeftHand);
            const RE::NiPoint3 muzzle{ 0.0f, 1.0f, 0.0f };
            const RE::NiPoint3 barrelWorld = transform_math::rotateLocalVectorToWorld(outWeaponWorld.rotate, muzzle);
            const RE::NiPoint3 barrelInLeftWand =
                transform_math::rotateWorldVectorToLocal(leftWand->world.rotate, barrelWorld);
            const RE::NiPoint3 aimBarrelInRightWand =
                transform_math::rotateLocalVectorToWorld(rightWeaponAimInWand.rotate, muzzle);
            ROCK_LOG_SAMPLE_INFO(Weapon, 1000,
                "TwoHandedGrip: left carry frame physicalSource={} damping={:.1f}deg "
                "physicalVsFirstPerson={:.1f}deg/{:.2f}gu barrelWorld=({:.2f},{:.2f},{:.2f}) "
                "barrelInLeftWand=({:.2f},{:.2f},{:.2f}) aimBarrelInRightWand=({:.2f},{:.2f},{:.2f})",
                physicalHandSource,
                tracked_hand_isolation_policy::rotationDegrees(dampedAimCarrierWorld, leftWand->world),
                firstPersonValid
                    ? tracked_hand_isolation_policy::rotationDegrees(outPhysicalHandWorld, firstPersonLeftHand)
                    : -1.0f,
                firstPersonValid
                    ? tracked_hand_isolation_policy::translationGameUnits(outPhysicalHandWorld, firstPersonLeftHand)
                    : -1.0f,
                barrelWorld.x, barrelWorld.y, barrelWorld.z, barrelInLeftWand.x, barrelInLeftWand.y, barrelInLeftWand.z,
                aimBarrelInRightWand.x, aimBarrelInRightWand.y, aimBarrelInRightWand.z);
        }
        if (!isFiniteTransform(outWeaponWorld))
            return fail("weapon-world");
        if (!isUsableHandAuthorityTransform(outPhysicalHandWorld))
            return fail("physical-hand:unusable");
        if (!isUsableHandAuthorityTransform(outPresentedHandWorld))
            return fail("presented-hand:unusable");
        return true;
    }

    void TwoHandedGrip::beginLeftFiringSupportReleaseReturn(const char* reason)
    {
        auto& state = _leftCarry.supportReleaseReturn;
        state.clear();
        if (!hand_visual_lerp_math::kEquippedWeaponReturnEnabled || !usesLeftFiringCarry() ||
            !_visuals.hasLastRenderedWeaponWorld || !isFiniteTransform(_visuals.lastRenderedWeaponWorld)) {
            return;
        }

        RE::NiTransform physicalHandWorld{};
        RE::NiTransform physicalDriverWorld{};
        if (!tryResolvePhysicalHandFrame(true, physicalHandWorld, physicalDriverWorld) ||
            !isInvertibleTransform(physicalHandWorld)) {
            return;
        }

        // The rendered record is the previous frame's two-hand pose; the node
        // itself already holds this frame's wand-aimed basis pre-write.
        const RE::NiTransform startHandLocal = left_firing_position_only_math::weaponWorldToPhysicalHandLocal(
            physicalHandWorld, _visuals.lastRenderedWeaponWorld);
        if (!isFiniteTransform(startHandLocal)) {
            return;
        }
        state.begin(startHandLocal);
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: left carry weapon return started reason={}",
            reason ? reason : "unknown");
    }

    void TwoHandedGrip::clearLeftFiringSupportReleaseReturn(const char* reason)
    {
        auto& state = _leftCarry.supportReleaseReturn;
        if (!state.active) {
            return;
        }
        state.clear();
        ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: left carry weapon return cancelled reason={}",
            reason ? reason : "unknown");
    }

    RE::NiTransform TwoHandedGrip::resolveLeftFiringSupportReleaseReturn(const RE::NiTransform& physicalHandWorld,
        const RE::NiTransform& positionOnlyWeaponWorld, const float advanceSeconds)
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
            left_firing_position_only_math::weaponWorldToPhysicalHandLocal(physicalHandWorld, positionOnlyWeaponWorld);
        RE::NiTransform blendedHandLocal{};
        bool completed = false;
        if (advanceSeconds > 0.0f) {
            const auto advanced = hand_visual_lerp_math::advanceVisualReturn(state, targetHandLocal, advanceSeconds,
                hand_visual_lerp_math::kEquippedWeaponReturnConfig);
            blendedHandLocal = advanced.transform;
            completed = advanced.reachedTarget;
        } else if (state.durationInitialized) {
            // Basis pre-write: show this frame's blend without advancing it.
            blendedHandLocal = hand_visual_lerp_math::blendTransformOverDuration(state.start, targetHandLocal,
                state.elapsedSeconds, state.durationSeconds)
                                   .transform;
        } else {
            blendedHandLocal = state.start;
        }

        const RE::NiTransform blendedWeaponWorld = left_firing_position_only_math::physicalHandLocalToWeaponWorld(
            physicalHandWorld, blendedHandLocal, positionOnlyWeaponWorld.scale);
        if (!isFiniteTransform(blendedWeaponWorld)) {
            clearLeftFiringSupportReleaseReturn("non-finite-return-transform");
            return positionOnlyWeaponWorld;
        }
        if (completed) {
            const float durationSeconds = state.durationSeconds;
            state.clear();
            ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: left carry weapon return completed duration={:.3f}s",
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
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: left-firing carry aborted because the FRIK weapon-node ownership block is "
                    "unavailable");
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
            if (!frik_visual_authority::setWeaponNodeParentHand(WEAPON_NODE_OWNERSHIP_TAG,
                    frik_visual_authority::Hand::Left)) {
                ROCK_LOG_WARN(Weapon,
                    "TwoHandedGrip: left-firing carry aborted because FRIK refused the weapon-node parent request");
                releaseFiringHandWeaponNodeOwnership(weaponNode);
                transitionToInactive(false);
                return;
            }
            _leftCarry.weaponNodeReparented = true;
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: equipped weapon node parent requested under LArm_Hand for left-firing carry");
        }
    }

    void TwoHandedGrip::releaseFiringHandWeaponNodeOwnership(RE::NiNode* /*weaponNode*/)
    {
        if (_leftCarry.weaponNodeReparented) {
            // FRIK restores the game's parent hand in its next skeleton pass.
            (void)frik_visual_authority::clearWeaponNodeParentHand(WEAPON_NODE_OWNERSHIP_TAG);
            _leftCarry.weaponNodeReparented = false;
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: equipped weapon node parent request cleared; FRIK restores the game's parent hand");
        }

        if (_leftCarry.weaponNodeOwnershipBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, false);
            _leftCarry.weaponNodeOwnershipBlockEngaged = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership restored");
        }
    }

    void TwoHandedGrip::rebindCarryFramesToWeapon(RE::NiNode* currentWeaponNode,
        const std::uint64_t targetWeaponGenerationKey, const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        if (_firing.leftDampedFollowFrame.valid) {
            if (_firing.leftDampedFollowFrame.weaponOwnershipKey != currentEquippedWeaponOwnershipKey) {
                _firing.leftDampedFollowFrame = {};
            } else {
                _firing.leftDampedFollowFrame.weaponNodeIdentity = currentWeaponNode;
                _firing.leftDampedFollowFrame.weaponGenerationKey = targetWeaponGenerationKey;
            }
        }
    }
}

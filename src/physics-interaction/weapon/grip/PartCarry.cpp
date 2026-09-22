#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Part-carry sessions: baseline construction, transition, per-frame update, and part-carry weapon authority solve.

namespace rock
{
    bool TwoHandedGrip::hasAuthoredSupportCarryPair() const noexcept
    {
        const auto describe = [](const WeaponPartGrip& grip) {
            return weapon_part_grip_report_policy::CarryGripInput{
                .active = grip.active,
                .authoredSupportSeat = grip.authoredRole == loose_weapon_authored_grab_policy::Role::Support,
                .providerAuthorityActive = grip.providerPartAuthority.active,
                .attachOnly = grip.attachOnly,
            };
        };
        return isPartCarryActive() && weapon_part_grip_report_policy::usesAuthoredSupportCarryPair(
            describe(partGrip(true)), describe(partGrip(false)));
    }

    bool TwoHandedGrip::handoffAuthoredSupportCarry(const bool releasingHandIsLeft)
    {
        if (!hasAuthoredSupportCarryPair() || _partCarry.pivotIsLeft != releasingHandIsLeft) {
            return true;
        }

        const bool nextCarrierIsLeft = !releasingHandIsLeft;
        SupportInputBaselineState baseline{};
        const char* failure = nullptr;
        if (!tryBuildPartCarryInputBaseline(nextCarrierIsLeft, baseline, failure)) {
            // Do not release the owner or fall back to the visual wrist's
            // orientation when the new physical driver cannot be calibrated.
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "TwoHandedGrip: authored support carry handoff deferred hand={} reason={}",
                nextCarrierIsLeft ? "left" : "right", failure);
            return false;
        }

        clearSupportInputBaselines();
        partGrip(nextCarrierIsLeft).supportInputBaseline = baseline;
        _partCarry.pivotIsLeft = nextCarrierIsLeft;
        _partCarry.gripSeparationWorld = 0.0f;
        // The new baseline already reproduces the last rendered pose exactly.
        // An earlier transition blend must not offset that calibrated result.
        clearWeaponPoseHandoffBlend("authored-support-carry-handoff", true);
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: authored support carry handoff from={} to={} poseSource=last-rendered grip={} generation={:016X}",
            releasingHandIsLeft ? "left" : "right", nextCarrierIsLeft ? "left" : "right",
            partGrip(nextCarrierIsLeft).gripSequence, _session.weaponGenerationKey);
        return true;
    }

    bool TwoHandedGrip::tryBuildPartCarryInputBaseline(
        const bool carryHandIsLeft,
        SupportInputBaselineState& outBaseline,
        const char*& outFailureReason) const
    {
        outBaseline = {};
        outFailureReason = "carry-identity-unavailable";

        const WeaponPartGrip& carryGrip = partGrip(carryHandIsLeft);
        if (!_session.weaponNode ||
            _session.weaponGenerationKey == 0 ||
            _session.equippedWeaponOwnershipKey == 0 ||
            !carryGrip.active ||
            !carryGrip.hasHandWeaponLocal) {
            return false;
        }
        if (carryGrip.weaponGenerationKey == 0 ||
            carryGrip.weaponGenerationKey !=
                _session.weaponGenerationKey ||
            carryGrip.gripSequence == 0) {
            outFailureReason = "carry-grip-identity-stale";
            return false;
        }
        if (!_visuals.hasLastRenderedWeaponWorld ||
            !isInvertibleTransform(_visuals.lastRenderedWeaponWorld)) {
            outFailureReason = "no-rendered-frame";
            return false;
        }

        const auto& carryDriver =
            _scope.currentHandDriverFrames[carryHandIsLeft ? 0u : 1u];
        if (!carryDriver.valid ||
            !isInvertibleTransform(carryDriver.world)) {
            outFailureReason = "no-raw-carry-driver";
            return false;
        }
        RE::NiTransform calibratedCarryDriverWorld = carryDriver.world;
        calibratedCarryDriverWorld.rotate =
            orthonormalizeStoredRotation(
                calibratedCarryDriverWorld.rotate);
        if (!isInvertibleTransform(calibratedCarryDriverWorld)) {
            outFailureReason = "invalid-raw-carry-driver";
            return false;
        }

        /*
         * The raw driver and authored grip target are intentionally different
         * frames. A gunstock may keep the controller upright while the authored
         * support hand lies sideways under the weapon. Capture both the
         * driver-to-authored-target and driver-to-weapon relations from ROCK's
         * final rendered two-hand pose. PartCarry then applies only later raw
         * driver deltas without rewriting the authored hand-in-weapon frame.
         */
        const RE::NiTransform authoredGripTargetWorld =
            weapon_visual_authority_math::weaponLocalFrameToWorld(
                _visuals.lastRenderedWeaponWorld,
                carryGrip.handWeaponLocal);
        RE::NiTransform driverToGripTargetLocal{};
        RE::NiTransform driverToWeaponLocal{};
        if (!isInvertibleTransform(authoredGripTargetWorld) ||
            !weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    calibratedCarryDriverWorld,
                    authoredGripTargetWorld,
                    driverToGripTargetLocal) ||
            !weapon_support_acquisition_math::
                tryCaptureSupportInputBaseline(
                    calibratedCarryDriverWorld,
                    _visuals.lastRenderedWeaponWorld,
                    driverToWeaponLocal)) {
            outFailureReason = "driver-baseline-capture-failed";
            return false;
        }

        outBaseline = {
            .inputToGripTargetLocal = driverToGripTargetLocal,
            .inputToWeaponLocal = driverToWeaponLocal,
            .weaponWorldAtCapture = _visuals.lastRenderedWeaponWorld,
            .weaponGenerationKey = carryGrip.weaponGenerationKey,
            .equippedWeaponOwnershipKey =
                _session.equippedWeaponOwnershipKey,
            .gripSequence = carryGrip.gripSequence,
            .supportHandIsLeft = carryHandIsLeft,
            .kind = SupportInputBaselineKind::PartCarry,
            .active = true,
            .firstPublicationPending = true,
        };
        outFailureReason = "driver-calibrated";
        return true;
    }

    bool TwoHandedGrip::transitionToPartCarry()
    {
        if (_session.state == TwoHandedState::PartCarry) {
            return true;
        }

        const bool carryHandIsLeft = isSupportHandLeft();
        const bool integratedImmersiveDetach =
            _handlingSettings.detachAuthority ==
                immersive_weapon_policy::DetachAuthority::
                    IntegratedImmersive;
        const bool posePreservationRequested =
            integratedImmersiveDetach &&
            _handlingSettings.preserveWeaponPoseOnDetach;
        bool poseHandoffReady = false;
        const char* poseHandoffReason =
            integratedImmersiveDetach ?
                "config-disabled" :
                "not-integrated-immersive";
        SupportInputBaselineState partCarryBaseline{};

        if (posePreservationRequested) {
            poseHandoffReady =
                tryBuildPartCarryInputBaseline(
                    carryHandIsLeft,
                    partCarryBaseline,
                    poseHandoffReason);
        }

        if (!blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: primary detach skipped because hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }
        clearDynamicSupportAcquisition(
            "transition-to-part-carry",
            true);
        // PartCarry has no firing/support role split. Clear the Gripping
        // calibration, then install only the freshly captured integrated
        // driver baseline below. A later return to Gripping recaptures its own
        // paired support calibration.
        clearSupportInputBaselines();
        beginHandVisualReturn(isFiringHandLeft(), "primary-detach-part-carry");
        clearPrimaryGripFingerPose(isFiringHandLeft());
        clearPrimaryGripWorldAuthority(isFiringHandLeft());
        if (usesLeftFiringCarry()) {
            _firing.leftDampedFollowFrame = {};
            clearLeftFiringSupportReleaseReturn("primary-detach-part-carry");
        }
        _visuals.primaryHandLerp = {};
        WeaponPartGrip& carryGrip = partGrip(carryHandIsLeft);
        carryGrip.visualLerp = {};
        lockPartGripToWeaponRoot(carryHandIsLeft);
        if (poseHandoffReady) {
            carryGrip.supportInputBaseline = partCarryBaseline;
        } else if (posePreservationRequested) {
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: integrated detach pose handoff unavailable; using legacy part-carry relation hand={} reason={}",
                carryHandIsLeft ? "left" : "right",
                poseHandoffReason);
        }
        _support.rotationBlend = 1.0f;
        _partCarry.pivotIsLeft = carryHandIsLeft;
        _partCarry.detachAuthority = _handlingSettings.detachAuthority;
        _partCarry.gripSeparationWorld = 0.0f;
        // The calibrated pose handoff yields a zero residual and no blend;
        // the legacy relation eases from the rendered two-hand pose instead
        // of jumping to the support hand's captured relation.
        armWeaponPoseHandoffBlend("primary-detach-part-carry");
        _session.state = TwoHandedState::PartCarry;
        recordFiringGripDetachedHaptic();
        if (poseHandoffReady) {
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: firing hand detached; part grips own equipped weapon authority source={} poseHandoff=driver-calibrated driver={}-raw gripTarget=authored poseSource=last-rendered",
                immersive_weapon_policy::authorityName(
                    _handlingSettings.detachAuthority),
                carryHandIsLeft ? "left" : "right");
        } else {
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: firing hand detached; part grips own equipped weapon authority source={} poseHandoff=legacy reason={}",
                immersive_weapon_policy::authorityName(
                    _handlingSettings.detachAuthority),
                poseHandoffReason);
        }
        return true;
    }

    bool TwoHandedGrip::republishPartCarryWeaponTransform(RE::NiNode* weaponNode)
    {
        if (_session.state != TwoHandedState::PartCarry || !_hasSolvedWeaponTransform || !weaponNode) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, _lastSolvedWeaponTransform);
    }

    void TwoHandedGrip::updatePartCarryGrip(
        RE::NiNode* weaponNode,
        float dt,
        const EquippedWeaponGripFrameInput& frameInput,
        const WeaponInteractionContact& leftWeaponContact,
        const WeaponInteractionContact& rightWeaponContact,
        const WeaponCollision& weaponCollision,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponInteractionRuntimeState& leftRuntimeState,
        const WeaponInteractionRuntimeState& rightRuntimeState,
        const WeaponInteractionDecision& supportAcquisitionDecision)
    {
        performance_profiler::ScopedTimer gripStageTimer(performance_profiler::Scope::EquippedGripSolve);
        const bool hadAuthoredSupportPair = hasAuthoredSupportCarryPair();
        const bool supportHandIsLeft = isSupportHandLeft();
        const bool firingHandIsLeft = isFiringHandLeft();
        const WeaponInteractionContact& firingHandContact = firingHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionContact& supportHandContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& firingRuntimeState = firingHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const bool supportGripHeld = supportHandIsLeft ? frameInput.leftGripHeld : frameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool firingHandHoldingObject = firingHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool supportHandAvailableForAcquisition = supportHandIsLeft ?
            frameInput.leftHandAvailableForAcquisition : frameInput.rightHandAvailableForAcquisition;
        const bool firingHandAvailableForAcquisition = firingHandIsLeft ?
            frameInput.leftHandAvailableForAcquisition : frameInput.rightHandAvailableForAcquisition;
        const bool authoredProviderAuthorityActive =
            leftRuntimeState.providerPartAuthority.active ||
            rightRuntimeState.providerPartAuthority.active;
        const bool authoredAttachOnlyAuthorityActive =
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                leftRuntimeState.providerPartAuthority.active,
                leftRuntimeState.providerPartAuthority.grabMode) ||
            weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                rightRuntimeState.providerPartAuthority.active,
                rightRuntimeState.providerPartAuthority.grabMode) ||
            (partGrip(true).active && partGrip(true).attachOnly) ||
            (partGrip(false).active && partGrip(false).attachOnly);

        /*
         * Firing-grip reattach is the squeeze gesture: a held grab with a
         * free palm inside the reattach zone (the lateral cylinders from the
         * grip point) re-takes the grip. Nothing attaches to an open
         * hand, and the gesture cannot re-capture a fresh detach because the
         * detach requires the grab to be open. A hand
         * already part-gripping is never converted; open it first, then
         * squeeze the grip. With ambidextrous takeover available, EITHER free
         * hand can squeeze the firing grip - whichever hand takes it becomes
         * the firing hand (the grip point itself stays weapon-relative). The
         * current firing hand is tested first so same-frame ties keep today's
         * behavior.
         */
        struct FiringGripReattachCandidate
        {
            bool isLeft;
            bool eligible;
            bool gripHeld;
            const WeaponInteractionContact* contact;
        };
        const FiringGripReattachCandidate reattachCandidates[2] = {
            { firingHandIsLeft,
                firingHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                frameInput.primaryGripInput.held,
                &firingHandContact },
            { supportHandIsLeft,
                supportHandIsLeft ? frameInput.leftReattachEligible : frameInput.rightReattachEligible,
                supportGripHeld,
                &supportHandContact },
        };
        firing_grip_reattach_zone_policy::ZoneInput reattachInput{};
        const bool reattachInputValid = tryBuildFiringGripZoneInput(
            weaponNode, currentWeaponGenerationKey,
            _session.equippedWeaponOwnershipKey,
            _handlingSettings.firingGripReattachRadiusGameUnits, reattachInput);
        for (const FiringGripReattachCandidate& candidate : reattachCandidates) {
            if (candidate.isLeft != firingHandIsLeft &&
                (!_handlingSettings.ambidextrousHandoffEnabled ||
                    !canBeginPrimaryOnlyGripForHand(candidate.isLeft))) {
                continue;
            }
            if (!candidate.eligible || partGrip(candidate.isLeft).active) {
                continue;
            }
            firing_grip_reattach_zone_policy::ZoneResult reattachZone{};
            if (!reattachInputValid || !tryEvaluateFiringGripZoneForHand(
                    candidate.isLeft,
                    reattachInput,
                    reattachZone)) {
                continue;
            }
            _firing.reattachDebugSnapshot.hands[candidate.isLeft ? 0u : 1u]
                .gripHeld = candidate.gripHeld;
            /*
             * The zone is the only gate. A probe-acquired contact used to
             * re-take the grip from anywhere inside the weapon probe; that
             * acquisition now only selects the authored hold inside
             * tryReattachFiringGrip and never bypasses the cylinders.
             */
            const bool reattachRequested = weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab(
                candidate.gripHeld,
                reattachZone.inside);
            if (!_firing.reattachHoverInsideZone &&
                weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate(
                    candidate.gripHeld,
                    reattachZone.inside)) {
                _firing.reattachHoverInsideZone = true;
                _firing.reattachHoverHandIsLeft = candidate.isLeft;
                updateFiringGripZoneIndicator(weaponNode,
                    currentWeaponGenerationKey, candidate.isLeft, reattachZone);
            }
            if (reattachRequested &&
                tryReattachFiringGrip(
                    candidate.isLeft,
                    weaponNode,
                    *candidate.contact,
                    authoredProviderAuthorityActive,
                    authoredAttachOnlyAuthorityActive)) {
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: firing-grip reattach zone hand={} inside={} side={} along={:.2f} perp={:.2f} reach={:.2f} radius={:.2f}",
                    candidate.isLeft ? "left" : "right",
                    reattachZone.inside ? "yes" : "no",
                    firing_grip_reattach_zone_policy::sideName(reattachZone.side),
                    reattachZone.alongAxisGameUnits,
                    reattachZone.perpendicularDistanceGameUnits,
                    _handlingSettings.firingGripReattachRadiusGameUnits,
                    _handlingSettings.firingGripReattachCylinderRadiusGameUnits);
                const bool newSupportHandIsLeft = isSupportHandLeft();
                if (partGrip(newSupportHandIsLeft).active) {
                    // Re-lock the two-hand separation against the (possibly
                    // swapped) support grip and re-blend the support target in.
                    const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(partGrip(newSupportHandIsLeft), weaponNode);
                    const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
                    const RE::NiPoint3 separationDelta = sub(supportGripWorld, firingGripWorld);
                    const float separation = std::sqrt(dot(separationDelta, separationDelta));
                    if (std::isfinite(separation)) {
                        _support.lockedGripSeparationWorld = separation;
                    }
                    if (candidate.isLeft != firingHandIsLeft) {
                        _support.rotationBlend = 0.0f;
                    }
                    _session.state = TwoHandedState::Gripping;
                    _partCarry.detachAuthority =
                        immersive_weapon_policy::DetachAuthority::None;
                    // Fresh two-hand configuration: the just-taken firing grip
                    // gets the same release-defer window as a fresh support grab.
                    WeaponPartGrip& reattachedSupportGrip =
                        partGrip(newSupportHandIsLeft);
                    if (weapon_support_authority_policy::
                            shouldUseDynamicSupportAcquisition(
                                _session.authorityMode,
                                reattachedSupportGrip.authoredSupportGrip,
                                reattachedSupportGrip.providerPartAuthority.active,
                                reattachedSupportGrip.attachOnly) &&
                        !initializeDynamicSupportBaseline(
                            weaponNode,
                            newSupportHandIsLeft,
                            "part-carry-firing-grip-reattach")) {
                        ROCK_LOG_WARN(
                            Weapon,
                            "TwoHandedGrip: part-carry reattach failed closed because the zero-delta dynamic baseline could not be captured");
                        transitionToInactive(false);
                        return;
                    }
                    armWeaponPoseHandoffBlend("firing-grip-reattach");
                    updateFullWeaponAuthorityGrip(weaponNode, dt);
                } else {
                    if (usesNativeRightCarry() && ownsWeaponTransform()) {
                        beginWeaponVisualReturn("part-carry-reattached-primary-only");
                    }
                    const bool primaryOnlyActive = transitionToPrimaryOnly(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        "part-carry-reattached-firing-grip");
                    if (primaryOnlyActive && usesLeftFiringCarry()) {
                        (void)solveLeftFiringWeaponCarry(weaponNode, dt);
                    }
                }
                return;
            }
        }

        const auto occupancy = getGripOccupancy();
        const bool allowLastHandDrop = _handlingSettings.lastGripReleaseDropEnabled &&
            equipped_weapon_drop_policy::canStartAutoDrop(
                occupancy.left.carriesWeapon(), occupancy.right.carriesWeapon(), false);

        /*
         * Open-hand releases are evaluated support hand first, so a same-frame
         * double release honors the support hand and the free hand becomes
         * the last carrier. A last carry grip releases only when that release
         * may drop the weapon; otherwise the hand keeps its grip.
         */
        const auto gripCarries = [](const WeaponPartGrip& grip) {
            return weapon_part_grip_report_policy::partGripCountsAsCarry(grip.active, grip.attachOnly);
        };

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        WeaponPartGrip& freeHandGrip = partGrip(firingHandIsLeft);
        // A reservation may remove either authored seat after a carry handoff.
        // Preserve the other seat when it can still own the weapon safely.
        if (hasAuthoredSupportCarryPair()) {
            for (const bool isLeft : { supportHandIsLeft, firingHandIsLeft }) {
                const auto& runtime = isLeft ? leftRuntimeState : rightRuntimeState;
                if (runtime.supportGripAllowed) continue;
                const auto& remainingRuntime = isLeft ? rightRuntimeState : leftRuntimeState;
                if (!remainingRuntime.supportGripAllowed || !handoffAuthoredSupportCarry(isLeft)) {
                    transitionToInactive(false);
                    return;
                }
                releasePartGrip(isLeft, "authored-support-hand-reserved", true);
            }
        }
        if (supportGripHeld) supportGrip.releaseRequiresNewHold = false;
        if (frameInput.primaryGripInput.held) freeHandGrip.releaseRequiresNewHold = false;
        if (supportGrip.active) {
            if (!providerPartAuthorityStillCurrent(supportGrip, currentWeaponGenerationKey) || !supportRuntimeState.supportGripAllowed) {
                /*
                 * Provider revocation and offhand reservation are policy
                 * changes, not a player release: return the weapon to
                 * FRIK-native carry instead of dropping it.
                 */
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: clearing part-carry authority because provider revoked the support part grip");
                transitionToInactive(false);
                return;
            }
            if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(supportGripHeld, supportHandHoldingObject)) {
                if (weapon_two_handed_grip_math::canReleaseCarryGrip(
                        gripCarries(supportGrip),
                        gripCarries(freeHandGrip),
                        allowLastHandDrop, supportGrip.releaseRequiresNewHold)) {
                    if (gripCarries(supportGrip) && !gripCarries(freeHandGrip)) {
                        (void)requestEquippedWeaponDrop("support-carry-released",
                            supportHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right, dt);
                        return;
                    }
                    if (handoffAuthoredSupportCarry(supportHandIsLeft)) {
                        releasePartGrip(supportHandIsLeft, "support-grip-released", true);
                    } else {
                        recordGripReleaseRetained(supportHandIsLeft, "authored-support-handoff-unavailable");
                    }
                } else {
                    recordGripReleaseRetained(supportHandIsLeft, "part-carry-last-carrier");
                }
            }
        }

        if (freeHandGrip.active) {
            if (!providerPartAuthorityStillCurrent(freeHandGrip, currentWeaponGenerationKey)) {
                releasePartGrip(firingHandIsLeft, "provider-part-authority-lost");
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(frameInput.primaryGripInput.held, firingHandHoldingObject)) {
                if (weapon_two_handed_grip_math::canReleaseCarryGrip(
                        gripCarries(freeHandGrip),
                        gripCarries(supportGrip),
                        allowLastHandDrop, freeHandGrip.releaseRequiresNewHold)) {
                    if (gripCarries(freeHandGrip) && !gripCarries(supportGrip)) {
                        (void)requestEquippedWeaponDrop("part-carry-released",
                            firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right, dt);
                        return;
                    }
                    if (handoffAuthoredSupportCarry(firingHandIsLeft)) {
                        releasePartGrip(firingHandIsLeft, "free-hand-grip-released", true);
                    } else {
                        recordGripReleaseRetained(firingHandIsLeft, "authored-support-handoff-unavailable");
                    }
                } else {
                    recordGripReleaseRetained(firingHandIsLeft, "part-carry-last-carrier");
                }
            }
        }

        /*
         * Mid-hold conversion, part-carry flavor: a part grip captured
         * WITHOUT provider authority whose own part now resolves to a
         * matched provider target (consumer armed its whitelist while the
         * hand was already holding — per-hand trigger arming) recaptures
         * immediately under the new resolution. Gated on the OTHER grip
         * holding carry authority: converting the last carry grip to
         * attach-only glue would drop the weapon through the fail-closed
         * all-grips check below. The free hand recaptures in the same
         * update rather than release-to-recapture because its capture path
         * is press-edged; the support hand gets the same treatment for
         * symmetry (no one-frame glue gap).
         */
        if (freeHandGrip.active && !freeHandGrip.providerPartAuthority.active &&
            firingRuntimeState.providerPartAuthority.active &&
            firingRuntimeState.providerPartAuthority.bodyId == freeHandGrip.contactBodyId &&
            weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly)) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (freeHandDecision.kind == WeaponInteractionKind::SupportGrip &&
                handoffAuthoredSupportCarry(firingHandIsLeft)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: recapturing free-hand part grip under newly matched provider weapon-part target");
                releasePartGrip(firingHandIsLeft, "provider-part-target-newly-matched");
                (void)capturePartGrip(firingHandIsLeft, weaponNode, freeHandDecision, weaponCollision, firingRuntimeState.providerPartAuthority, false, false);
            }
        }
        if (supportGrip.active && !supportGrip.providerPartAuthority.active &&
            supportRuntimeState.providerPartAuthority.active &&
            supportRuntimeState.providerPartAuthority.bodyId == supportGrip.contactBodyId &&
            weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
            const WeaponInteractionDecision supportDecision = routeWeaponInteraction(supportHandContact, supportRuntimeState);
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip &&
                handoffAuthoredSupportCarry(supportHandIsLeft)) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: recapturing support part grip under newly matched provider weapon-part target");
                releasePartGrip(supportHandIsLeft, "provider-part-target-newly-matched");
                (void)capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority, false, false);
            }
        }

        const bool authoredFreeHandSeatAvailable = authoredSupportSeatAvailable(
            firingHandIsLeft, currentWeaponGenerationKey, firingRuntimeState);
        const bool freeHandGrabRequested = frameInput.primaryGripInput.pressed ||
            (authoredFreeHandSeatAvailable && frameInput.primaryGripInput.held);
        if (!freeHandGrip.active && firingHandAvailableForAcquisition && freeHandGrabRequested) {
            WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (authoredFreeHandSeatAvailable) {
                freeHandDecision = {
                    .kind = WeaponInteractionKind::SupportGrip,
                    .partKind = WeaponPartKind::Other,
                    .gripPose = WeaponGripPoseId::ReceiverSupport,
                    .bodyId = 0x7FFF'FFFFu,
                    .interactionRoot = weaponNode,
                    .sourceRoot = weaponNode,
                    .weaponGenerationKey = currentWeaponGenerationKey,
                    .acquisitionSource = WeaponInteractionAcquisitionSource::AuthoredSeat,
                };
            }
            if (weapon_two_handed_grip_math::canStartFreeHandPartGrip(
                    freeHandDecision.kind == WeaponInteractionKind::SupportGrip,
                    freeHandGrabRequested,
                    firingHandHoldingObject,
                    freeHandGrip.active)) {
                const auto partGrabSelection =
                    immersive_weapon_policy::
                        resolveDetachedFiringHandPartGrab(
                            immersive_weapon_policy::
                                DetachedFiringHandPartGrabInput{
                                    .partCarryAuthority =
                                        _partCarry.detachAuthority,
                                    .authoredOnlySupportGrabsEnabled =
                                        _handlingSettings.
                                            authoredOnlySupportGrabsEnabled,
                                    .exactProviderPartTargetActive =
                                        firingRuntimeState.
                                            providerPartAuthority.active,
                                    .authoredSupportSeatAvailable = authoredFreeHandSeatAvailable,
                                });
                if (partGrabSelection ==
                    immersive_weapon_policy::
                        DetachedFiringHandPartGrabSelection::Reject) {
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: detached firing-hand part grip rejected hand={} reason=authored-seat-or-exact-provider-target-required bodyId={} generation={:016X}",
                        firingHandIsLeft ? "left" : "right",
                        freeHandDecision.bodyId,
                        freeHandDecision.weaponGenerationKey);
                } else if (authored_support_grab_policy::captured(
                        capturePartGrip(
                            firingHandIsLeft,
                            weaponNode,
                            freeHandDecision,
                            weaponCollision,
                            firingRuntimeState.providerPartAuthority,
                            false,
                            false))) {
                    /*
                     * AttachOnly keeps its authored source frames so the glued
                     * hand follows provider-driven part motion; it never joins
                     * the carry solve, so it cannot feed part animation back
                     * into the carry (the drift lockPartGripToWeaponRoot
                     * prevents). Separation/blend only matter for a two-anchor
                     * carry, which needs both grips to hold carry authority.
                     */
                    if (freeHandGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: free-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(firingHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly)) {
                            _partCarry.gripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _support.rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        if (!supportGrip.active && supportHandAvailableForAcquisition) {
            // Reuse the controller's authored-zone admission here as well;
            // an authored seat does not require a mesh contact while carrying.
            const auto& supportDecision = supportAcquisitionDecision;
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip &&
                weapon_two_handed_grip_math::canStartSupportGrip(true, supportGripHeld, supportHandHoldingObject)) {
                if (authored_support_grab_policy::captured(
                        capturePartGrip(
                            supportHandIsLeft,
                            weaponNode,
                            supportDecision,
                            weaponCollision,
                            supportRuntimeState.providerPartAuthority,
                            false,
                            false))) {
                    // Symmetric to the free-hand capture above: attach-only
                    // glue keeps source frames and stays out of the carry.
                    if (supportGrip.attachOnly) {
                        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: support-hand part grip captured as provider attach-only glue");
                    } else {
                        lockPartGripToWeaponRoot(supportHandIsLeft);
                        if (weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
                            _partCarry.gripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _support.rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        // Player last-hand releases were handled while their pose still
        // existed. Revoked carry authority returns to native ownership;
        // provider AttachOnly grips cannot inherit it or cause an auto-drop.
        if (!gripCarries(supportGrip) && !gripCarries(freeHandGrip)) {
            releasePartGrip(supportHandIsLeft, "carry-authority-lost", true);
            releasePartGrip(firingHandIsLeft, "carry-authority-lost", true);
            transitionToInactive(false);
            return;
        }

        // The pivot must always be a carry-authority grip; after the check
        // above, the other hand is guaranteed to hold one.
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(
                partGrip(_partCarry.pivotIsLeft).active,
                partGrip(_partCarry.pivotIsLeft).attachOnly)) {
            _partCarry.pivotIsLeft = !_partCarry.pivotIsLeft;
        }

        if (!hadAuthoredSupportPair && hasAuthoredSupportCarryPair()) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: authored handguard pair carrier={} visualSupport={} firingGrip=vacant generation={:016X}",
                _partCarry.pivotIsLeft ? "left" : "right",
                _partCarry.pivotIsLeft ? "right" : "left", currentWeaponGenerationKey);
        }
        (void)solvePartCarryWeaponAuthority(weaponNode, dt);
    }

    float TwoHandedGrip::partCarryGripSeparation(RE::NiNode* weaponNode) const
    {
        const RE::NiPoint3 leftGripWorld = resolvePartGripWorld(partGrip(true), weaponNode);
        const RE::NiPoint3 rightGripWorld = resolvePartGripWorld(partGrip(false), weaponNode);
        const RE::NiPoint3 delta = sub(leftGripWorld, rightGripWorld);
        const float separation = std::sqrt(dot(delta, delta));
        return std::isfinite(separation) ? separation : 0.0f;
    }

    bool TwoHandedGrip::solvePartCarryWeaponAuthority(RE::NiNode* weaponNode, float dt)
    {
        const bool pivotIsLeft = _partCarry.pivotIsLeft;
        WeaponPartGrip& pivotGrip = partGrip(pivotIsLeft);
        const WeaponPartGrip& aimGrip = partGrip(!pivotIsLeft);
        // Provider glue and the second authored handguard seat follow the
        // pivot's result; neither contributes an aim axis or wrist rotation.
        const bool authoredVisualSupport = hasAuthoredSupportCarryPair();
        const bool aimGripCarries = !authoredVisualSupport &&
            weapon_part_grip_report_policy::partGripCountsAsCarry(aimGrip.active, aimGrip.attachOnly);
        if (!pivotGrip.active || !pivotGrip.hasHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because captured hand frames are unavailable");
            transitionToInactive(false);
            return false;
        }

        const bool partCarryBaselineDeclared =
            pivotGrip.supportInputBaseline.active &&
            pivotGrip.supportInputBaseline.kind ==
                SupportInputBaselineKind::PartCarry;
        const bool partCarryBaselineActive =
            isPartCarryInputBaselineActive(pivotIsLeft, pivotGrip);
        if (partCarryBaselineDeclared && !partCarryBaselineActive) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(
                Weapon,
                "TwoHandedGrip: clearing calibrated part-carry grip because its weapon, ownership, pivot, generation, or grip identity became stale");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform pivotHandTransform{};
        RE::NiTransform calibratedPartCarryWeaponWorld{};
        if (partCarryBaselineActive) {
            const auto& pivotDriver =
                _scope.currentHandDriverFrames[pivotIsLeft ? 0u : 1u];
            RE::NiTransform calibratedPivotDriverWorld{};
            if (pivotDriver.valid) {
                calibratedPivotDriverWorld = pivotDriver.world;
                calibratedPivotDriverWorld.rotate =
                    orthonormalizeStoredRotation(
                        calibratedPivotDriverWorld.rotate);
            }
            if (!pivotDriver.valid ||
                !isInvertibleTransform(calibratedPivotDriverWorld) ||
                !weapon_support_acquisition_math::
                    tryResolveSupportInputTarget(
                        calibratedPivotDriverWorld,
                        pivotGrip.supportInputBaseline.
                            inputToGripTargetLocal,
                        pivotHandTransform) ||
                !weapon_support_acquisition_math::
                    tryResolveSupportInputTarget(
                        calibratedPivotDriverWorld,
                        pivotGrip.supportInputBaseline.inputToWeaponLocal,
                        calibratedPartCarryWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(
                    Weapon,
                    "TwoHandedGrip: clearing calibrated part-carry grip because the raw pivot driver or its captured targets are unavailable hand={} driver={}",
                    pivotIsLeft ? "left" : "right",
                    pivotDriver.valid ? "valid" : "missing");
                transitionToInactive(false);
                return false;
            }
        } else if (!tryGetSolverHandTransform(
                       pivotIsLeft,
                       pivotHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }

        if (aimGripCarries) {
            RE::NiTransform aimHandTransform{};
            if (!tryGetSolverHandTransform(!pivotIsLeft, aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because aim hand transform is unavailable");
                transitionToInactive(false);
                return false;
            }

            _support.rotationBlend = (std::min)(1.0f, _support.rotationBlend + dt * ROTATION_BLEND_SPEED);

            /*
             * The two-anchor solve must be closed over the captured
             * weapon-root-local grip points. Part-carry feeds its own solved
             * transform back as the next frame's base, so re-resolving grips
             * through live part-node chains lets any per-frame part animation
             * integrate into a steady carry drift (verified by telemetry:
             * rigid-weapon grip separation grew frame over frame). The
             * trade-off is that a two-anchor carry does not follow externally
             * driven part motion; provider part revocation releases the grip
             * in that case.
             */
            const RE::NiPoint3 pivotPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(pivotHandTransform, pivotIsLeft);
            const RE::NiPoint3 aimPalm = computeGrabLegacyPalmPivotAWorldFromHandBasis(aimHandTransform, !pivotIsLeft);
            const RE::NiPoint3 currentAimGripWorld = weaponLocalToWorld(aimGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentPivotGripWorld = weaponLocalToWorld(pivotGrip.gripLocal, weaponNode);
            const RE::NiPoint3 currentSeparationDelta = sub(currentAimGripWorld, currentPivotGripWorld);
            const float currentSeparation = std::sqrt(dot(currentSeparationDelta, currentSeparationDelta));
            const float lockedSeparation = _partCarry.gripSeparationWorld > 0.0f ? _partCarry.gripSeparationWorld : currentSeparation;
            const RE::NiPoint3 lockedAimTarget = makeLockedSupportGripTarget(
                pivotPalm,
                aimPalm,
                currentAimGripWorld,
                lockedSeparation,
                0.001f);
            const RE::NiPoint3 blendedAimTarget = lerpPoint(currentAimGripWorld, lockedAimTarget, _support.rotationBlend);

            WeaponTwoHandedSolverInput<RE::NiTransform, RE::NiPoint3> solverInput{};
            solverInput.weaponWorldTransform = weaponNode->world;
            solverInput.primaryGripLocal = pivotGrip.gripLocal;
            solverInput.supportGripLocal = aimGrip.gripLocal;
            solverInput.primaryTargetWorld = pivotPalm;
            solverInput.supportTargetWorld = blendedAimTarget;
            solverInput.supportNormalLocal = aimGrip.normalLocal;
            solverInput.supportNormalTargetWorld = computePalmNormalFromHandBasis(aimHandTransform, !pivotIsLeft);
            solverInput.useSupportNormalTwist = true;
            solverInput.supportNormalTwistFactor = SUPPORT_NORMAL_TWIST_FACTOR;

            const auto solved = solveTwoHandedWeaponTransformFrikPivot(solverInput);
            if (!solved.solved) {
                return true;
            }

            // Break the rotation feedback loop's orthonormality decay before
            // the solved transform becomes next frame's base.
            RE::NiTransform stabilizedWeaponWorld = solved.weaponWorldTransform;
            stabilizedWeaponWorld.rotate = orthonormalizeStoredRotation(stabilizedWeaponWorld.rotate);
            stabilizedWeaponWorld = resolveWeaponPoseHandoffBlend(stabilizedWeaponWorld, dt);

            if (!applyWeaponVisualAuthority(weaponNode, stabilizedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            static_assert(weapon_visual_authority_math::weaponVisualPrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            publishGripHandPoses(!pivotIsLeft);

            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform) ||
                !applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, &aimHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        } else {
            RE::NiTransform solvedWeaponWorld{};
            if (partCarryBaselineActive) {
                solvedWeaponWorld = calibratedPartCarryWeaponWorld;
            } else if (pivotGrip.hasSourceFrames && pivotGrip.hasAttachmentWeaponLocal && resolveCurrentSupportAttachmentRoot(pivotGrip, weaponNode)) {
                const RE::NiTransform solvedSourceWorld =
                    transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handSourceLocal));
                solvedWeaponWorld = transform_math::composeTransforms(solvedSourceWorld, transform_math::invertTransform(pivotGrip.attachmentWeaponLocal));
            } else {
                solvedWeaponWorld = transform_math::composeTransforms(pivotHandTransform, transform_math::invertTransform(pivotGrip.handWeaponLocal));
            }
            if (!isFiniteTransform(solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because single-anchor weapon solve produced invalid transform");
                transitionToInactive(false);
                return false;
            }
            // Firing-grip detach: ease the rendered two-hand pose into the
            // support hand's carry relation.
            solvedWeaponWorld = resolveWeaponPoseHandoffBlend(solvedWeaponWorld, dt);

            if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK visual weapon authority failed");
                transitionToInactive(false);
                return false;
            }

            static_assert(weapon_visual_authority_math::handPosePrecedesLockedHandAuthority());
            publishGripHandPoses(pivotIsLeft);
            if (!applyPartGripLockedVisual(pivotIsLeft, weaponNode, dt, &pivotHandTransform)) {
                _hasSolvedWeaponTransform = false;
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing part-carry grip because ROCK part grip hand authority failed");
                transitionToInactive(false);
                return false;
            }
        }

        /*
         * Visual followers publish after the weapon solve. Provider glue also
         * follows this frame's part drives. Failure only loses the follower;
         * the carry pivot survives it.
         */
        if (aimGrip.active && (aimGrip.attachOnly || authoredVisualSupport)) {
            RE::NiTransform attachHandTransform{};
            const RE::NiTransform* liveAttachHandWorld =
                tryGetSolverHandTransform(!pivotIsLeft, attachHandTransform) ? &attachHandTransform : nullptr;
            publishGripHandPoses(!pivotIsLeft);
            if (!applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, liveAttachHandWorld)) {
                releasePartGrip(!pivotIsLeft, "carry-support-visual-authority-failed");
            }
        }

        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        if (partCarryBaselineActive &&
            pivotGrip.supportInputBaseline.firstPublicationPending) {
            const auto& baseline = pivotGrip.supportInputBaseline;
            const float weaponTranslationDeltaGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    baseline.weaponWorldAtCapture.translate,
                    weaponNode->world.translate);
            const float weaponRotationDeltaDegrees =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    baseline.weaponWorldAtCapture,
                    weaponNode->world);
            const RE::NiTransform publishedGripTargetWorld =
                resolvePartGripHandWorld(pivotGrip, weaponNode);
            const float handTargetTranslationDeltaGameUnits =
                hand_visual_lerp_math::distanceGameUnits(
                    pivotHandTransform.translate,
                    publishedGripTargetWorld.translate);
            const float handTargetRotationDeltaDegrees =
                hand_visual_lerp_math::rotationDistanceDegrees(
                    pivotHandTransform,
                    publishedGripTargetWorld);
            constexpr float kTranslationWarningGameUnits = 0.01f;
            constexpr float kRotationWarningDegrees = 0.05f;
            const bool firstPublicationExact =
                std::isfinite(weaponTranslationDeltaGameUnits) &&
                std::isfinite(weaponRotationDeltaDegrees) &&
                std::isfinite(handTargetTranslationDeltaGameUnits) &&
                std::isfinite(handTargetRotationDeltaDegrees) &&
                weaponTranslationDeltaGameUnits <=
                    kTranslationWarningGameUnits &&
                weaponRotationDeltaDegrees <= kRotationWarningDegrees &&
                handTargetTranslationDeltaGameUnits <=
                    kTranslationWarningGameUnits &&
                handTargetRotationDeltaDegrees <=
                    kRotationWarningDegrees;
            if (firstPublicationExact) {
                ROCK_LOG_INFO(
                    Weapon,
                    "TwoHandedGrip: calibrated part-carry first publication poseHandoff=driver-calibrated weaponDelta=({:.5f}gu,{:.5f}deg) authoredHandDelta=({:.5f}gu,{:.5f}deg)",
                    weaponTranslationDeltaGameUnits,
                    weaponRotationDeltaDegrees,
                    handTargetTranslationDeltaGameUnits,
                    handTargetRotationDeltaDegrees);
            } else {
                ROCK_LOG_WARN(
                    Weapon,
                    "TwoHandedGrip: calibrated part-carry first publication exceeded zero-delta witness weaponDelta=({:.5f}gu,{:.5f}deg) authoredHandDelta=({:.5f}gu,{:.5f}deg)",
                    weaponTranslationDeltaGameUnits,
                    weaponRotationDeltaDegrees,
                    handTargetTranslationDeltaGameUnits,
                    handTargetRotationDeltaDegrees);
            }
            pivotGrip.supportInputBaseline.firstPublicationPending = false;
        }

        if (++_gripLogCounter >= 90) {
            _gripLogCounter = 0;
            const RE::NiPoint3 pivotGripFinal = resolvePartGripWorld(pivotGrip, weaponNode);
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: part-carry authority pivot={} anchors={} pivotGrip=({:.1f},{:.1f},{:.1f}) handLerp=({:.2f}/{:.3f}s,{:.2f}/{:.3f}s)",
                pivotIsLeft ? "left" : "right",
                aimGripCarries ? 2 : 1,
                pivotGripFinal.x,
                pivotGripFinal.y,
                pivotGripFinal.z,
                pivotGrip.visualLerp.lastAlpha,
                pivotGrip.visualLerp.durationSeconds,
                aimGrip.visualLerp.lastAlpha,
                aimGrip.visualLerp.durationSeconds);
        }
        return true;
    }
}

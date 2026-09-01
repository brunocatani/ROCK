#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Part-carry sessions: baseline construction, transition, per-frame update, and part-carry weapon authority solve.

namespace rock
{
    bool TwoHandedGrip::tryBuildIntegratedDetachPartCarryBaseline(
        const bool carryHandIsLeft,
        SupportInputBaselineState& outBaseline,
        const char*& outFailureReason) const
    {
        outBaseline = {};
        outFailureReason = "carry-identity-unavailable";

        const WeaponPartGrip& carryGrip = partGrip(carryHandIsLeft);
        if (!_activeWeaponNode ||
            _activeWeaponGenerationKey == 0 ||
            _activeEquippedWeaponOwnershipKey == 0 ||
            !carryGrip.active ||
            !carryGrip.hasHandWeaponLocal) {
            return false;
        }
        if (carryGrip.weaponGenerationKey == 0 ||
            carryGrip.weaponGenerationKey !=
                _activeWeaponGenerationKey ||
            carryGrip.gripSequence == 0) {
            outFailureReason = "carry-grip-identity-stale";
            return false;
        }
        if (!_hasLastRenderedWeaponWorld ||
            !isInvertibleTransform(_lastRenderedWeaponWorld)) {
            outFailureReason = "no-rendered-frame";
            return false;
        }

        const auto& carryDriver =
            _currentHandDriverFrames[carryHandIsLeft ? 0u : 1u];
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
                _lastRenderedWeaponWorld,
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
                    _lastRenderedWeaponWorld,
                    driverToWeaponLocal)) {
            outFailureReason = "driver-baseline-capture-failed";
            return false;
        }

        outBaseline = {
            .inputToGripTargetLocal = driverToGripTargetLocal,
            .inputToWeaponLocal = driverToWeaponLocal,
            .weaponWorldAtCapture = _lastRenderedWeaponWorld,
            .weaponGenerationKey = carryGrip.weaponGenerationKey,
            .equippedWeaponOwnershipKey =
                _activeEquippedWeaponOwnershipKey,
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
        if (_state == TwoHandedState::PartCarry) {
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
                tryBuildIntegratedDetachPartCarryBaseline(
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
            _leftFiringDampedFollowFrame = {};
        }
        _primaryHandVisualLerp = {};
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
        _rotationBlend = 1.0f;
        _partCarryPivotIsLeft = carryHandIsLeft;
        _partCarryDetachAuthority = _handlingSettings.detachAuthority;
        _partCarryGripSeparationWorld = 0.0f;
        _state = TwoHandedState::PartCarry;
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
        if (_state != TwoHandedState::PartCarry || !_hasSolvedWeaponTransform || !weaponNode) {
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
        const WeaponInteractionRuntimeState& rightRuntimeState)
    {
        const bool supportHandIsLeft = isSupportHandLeft();
        const bool firingHandIsLeft = isFiringHandLeft();
        const WeaponInteractionContact& firingHandContact = firingHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionContact& supportHandContact = supportHandIsLeft ? leftWeaponContact : rightWeaponContact;
        const WeaponInteractionRuntimeState& firingRuntimeState = firingHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const WeaponInteractionRuntimeState& supportRuntimeState = supportHandIsLeft ? leftRuntimeState : rightRuntimeState;
        const bool supportGripHeld = supportHandIsLeft ? frameInput.leftGripHeld : frameInput.rightGripHeld;
        const bool supportHandHoldingObject = supportHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
        const bool firingHandHoldingObject = firingHandIsLeft ? frameInput.leftHandHoldingObject : frameInput.rightHandHoldingObject;
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
         * free palm inside the reattach radius re-takes the grip. Nothing
         * attaches to an open hand, and the gesture cannot re-capture a fresh
         * detach because the detach requires the grab to be open. A hand
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
        for (const FiringGripReattachCandidate& candidate : reattachCandidates) {
            if (candidate.isLeft != firingHandIsLeft &&
                (!_handlingSettings.ambidextrousHandoffEnabled ||
                    !canBeginPrimaryOnlyGripForHand(candidate.isLeft))) {
                continue;
            }
            if (!candidate.eligible || partGrip(candidate.isLeft).active) {
                continue;
            }
            float palmToGripDistance = 0.0f;
            if (!tryComputePalmToGripDistanceForHand(weaponNode, candidate.isLeft, palmToGripDistance)) {
                continue;
            }
            const bool reattachRequested = weapon_two_handed_grip_math::shouldReattachFiringGripOnGrab(
                candidate.gripHeld,
                palmToGripDistance,
                _handlingSettings.firingGripReattachRadiusGameUnits) ||
                (candidate.gripHeld &&
                    candidate.contact->acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe &&
                    !authoredProviderAuthorityActive &&
                    !authoredAttachOnlyAuthorityActive);
            if (!_firingGripReattachHoverInsideRadius &&
                weapon_two_handed_grip_math::isFiringGripReattachHoverCandidate(
                    candidate.gripHeld,
                    palmToGripDistance,
                    _handlingSettings.firingGripReattachRadiusGameUnits)) {
                _firingGripReattachHoverInsideRadius = true;
                _firingGripReattachHoverHandIsLeft = candidate.isLeft;
            }
            if (reattachRequested &&
                tryReattachFiringGrip(
                    candidate.isLeft,
                    weaponNode,
                    *candidate.contact,
                    authoredProviderAuthorityActive,
                    authoredAttachOnlyAuthorityActive)) {
                const bool newSupportHandIsLeft = isSupportHandLeft();
                if (partGrip(newSupportHandIsLeft).active) {
                    // Re-lock the two-hand separation against the (possibly
                    // swapped) support grip and re-blend the support target in.
                    const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(partGrip(newSupportHandIsLeft), weaponNode);
                    const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
                    const RE::NiPoint3 separationDelta = sub(supportGripWorld, firingGripWorld);
                    const float separation = std::sqrt(dot(separationDelta, separationDelta));
                    if (std::isfinite(separation)) {
                        _lockedGripSeparationWorld = separation;
                    }
                    if (candidate.isLeft != firingHandIsLeft) {
                        _rotationBlend = 0.0f;
                    }
                    _state = TwoHandedState::Gripping;
                    _partCarryDetachAuthority =
                        immersive_weapon_policy::DetachAuthority::None;
                    // Fresh two-hand configuration: the just-taken firing grip
                    // gets the same release-defer window as a fresh support grab.
                    _supportGripAgeSeconds = 0.0f;
                    _freshSupportGripDeferLogged = false;
                    WeaponPartGrip& reattachedSupportGrip =
                        partGrip(newSupportHandIsLeft);
                    if (weapon_support_authority_policy::
                            shouldUseDynamicSupportAcquisition(
                                _authorityMode,
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
                        (void)solveLeftFiringWeaponCarry(weaponNode);
                    }
                }
                return;
            }
        }

        bool lastReleaseWasSupportHand = true;

        WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
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
                releasePartGrip(supportHandIsLeft, "support-grip-released", true);
                lastReleaseWasSupportHand = true;
            }
        }

        WeaponPartGrip& freeHandGrip = partGrip(firingHandIsLeft);
        if (freeHandGrip.active) {
            if (!providerPartAuthorityStillCurrent(freeHandGrip, currentWeaponGenerationKey)) {
                releasePartGrip(firingHandIsLeft, "provider-part-authority-lost");
                lastReleaseWasSupportHand = false;
            } else if (!weapon_two_handed_grip_math::shouldContinueSupportGrip(frameInput.primaryGripInput.held, firingHandHoldingObject)) {
                releasePartGrip(firingHandIsLeft, "free-hand-grip-released", true);
                lastReleaseWasSupportHand = false;
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
            if (freeHandDecision.kind == WeaponInteractionKind::SupportGrip) {
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
            if (supportDecision.kind == WeaponInteractionKind::SupportGrip) {
                ROCK_LOG_INFO(Weapon, "TwoHandedGrip: recapturing support part grip under newly matched provider weapon-part target");
                releasePartGrip(supportHandIsLeft, "provider-part-target-newly-matched");
                (void)capturePartGrip(supportHandIsLeft, weaponNode, supportDecision, weaponCollision, supportRuntimeState.providerPartAuthority, false, false);
            }
        }

        if (!freeHandGrip.active && frameInput.primaryGripInput.pressed) {
            const WeaponInteractionDecision freeHandDecision = routeWeaponInteraction(firingHandContact, firingRuntimeState);
            if (weapon_two_handed_grip_math::canStartFreeHandPartGrip(
                    freeHandDecision.kind == WeaponInteractionKind::SupportGrip,
                    frameInput.primaryGripInput.pressed,
                    firingHandHoldingObject,
                    freeHandGrip.active)) {
                const auto partGrabSelection =
                    immersive_weapon_policy::
                        resolveDetachedFiringHandPartGrab(
                            immersive_weapon_policy::
                                DetachedFiringHandPartGrabInput{
                                    .partCarryAuthority =
                                        _partCarryDetachAuthority,
                                    .authoredOnlySupportGrabsEnabled =
                                        _handlingSettings.
                                            authoredOnlySupportGrabsEnabled,
                                    .exactProviderPartTargetActive =
                                        firingRuntimeState.
                                            providerPartAuthority.active,
                                });
                if (partGrabSelection ==
                    immersive_weapon_policy::
                        DetachedFiringHandPartGrabSelection::Reject) {
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: detached firing-hand part grip rejected hand={} reason=authored-firing-grip-or-exact-provider-target-required bodyId={} generation={:016X}",
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
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        if (!supportGrip.active) {
            const WeaponInteractionDecision supportDecision = routeWeaponInteraction(supportHandContact, supportRuntimeState);
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
                            _partCarryGripSeparationWorld = partCarryGripSeparation(weaponNode);
                            _rotationBlend = 0.0f;
                        }
                    }
                }
            }
        }

        /*
         * Only carry-authority grips can hold the weapon. When the last carry
         * grip releases, a remaining AttachOnly glue cannot inherit pivot
         * authority (never upgrade), so it releases with the carry and the
         * normal manual-drop request proceeds (fail closed).
         */
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(supportGrip.active, supportGrip.attachOnly) &&
            !weapon_part_grip_report_policy::partGripCountsAsCarry(freeHandGrip.active, freeHandGrip.attachOnly)) {
            releasePartGrip(supportHandIsLeft, "carry-authority-lost", true);
            releasePartGrip(firingHandIsLeft, "carry-authority-lost", true);
            requestEquippedWeaponDrop(
                "part-carry-all-grips-released",
                lastReleaseWasSupportHand ?
                    (supportHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right) :
                    (firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right));
            return;
        }

        // The pivot must always be a carry-authority grip; after the check
        // above, the other hand is guaranteed to hold one.
        if (!weapon_part_grip_report_policy::partGripCountsAsCarry(
                partGrip(_partCarryPivotIsLeft).active,
                partGrip(_partCarryPivotIsLeft).attachOnly)) {
            _partCarryPivotIsLeft = !_partCarryPivotIsLeft;
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
        const bool pivotIsLeft = _partCarryPivotIsLeft;
        WeaponPartGrip& pivotGrip = partGrip(pivotIsLeft);
        const WeaponPartGrip& aimGrip = partGrip(!pivotIsLeft);
        // An AttachOnly glue never aims the weapon; the carry solves
        // single-anchor around the pivot and the glue publishes afterwards.
        const bool aimGripCarries = weapon_part_grip_report_policy::partGripCountsAsCarry(aimGrip.active, aimGrip.attachOnly);
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
                _currentHandDriverFrames[pivotIsLeft ? 0u : 1u];
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

            _rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

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
            const float lockedSeparation = _partCarryGripSeparationWorld > 0.0f ? _partCarryGripSeparationWorld : currentSeparation;
            const RE::NiPoint3 lockedAimTarget = makeLockedSupportGripTarget(
                pivotPalm,
                aimPalm,
                currentAimGripWorld,
                lockedSeparation,
                0.001f);
            const RE::NiPoint3 blendedAimTarget = lerpPoint(currentAimGripWorld, lockedAimTarget, _rotationBlend);

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
         * AttachOnly glue publishes after the weapon solve so it composes from
         * this frame's part transforms (including provider part drives applied
         * earlier in the frame). A glue visual failure only loses the attach;
         * the carry pivot must survive it.
         */
        if (aimGrip.active && aimGrip.attachOnly) {
            RE::NiTransform attachHandTransform{};
            const RE::NiTransform* liveAttachHandWorld =
                tryGetSolverHandTransform(!pivotIsLeft, attachHandTransform) ? &attachHandTransform : nullptr;
            publishGripHandPoses(!pivotIsLeft);
            if (!applyPartGripLockedVisual(!pivotIsLeft, weaponNode, dt, liveAttachHandWorld)) {
                releasePartGrip(!pivotIsLeft, "attach-only-visual-authority-failed");
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
                aimGrip.active ? 2 : 1,
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

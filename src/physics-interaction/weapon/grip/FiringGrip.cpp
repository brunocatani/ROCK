#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Firing-hand grip: canonical right frame, native aim-frame capture, left/right mirroring, authored primary firing grip canonical and finger pose, reattach, support-to-firing promotion, and PrimaryOnly (persistent equipped carry) sessions.

namespace rock
{
    bool TwoHandedGrip::canBeginPrimaryOnlyGripForHand(const bool isLeft)
    {
        return !isLeft || leftFiringInfrastructureAvailable();
    }

    bool TwoHandedGrip::tryBuildCurrentLeftFiringGripCapture(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        RE::NiTransform& outFiringHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal)
    {
        outFiringHandWeaponLocal = {};
        outFiringGripWeaponLocal = {};
        if (!canBeginPrimaryOnlyGripForHand(true) ||
            !hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            return false;
        }

        if (!hasRightNativeWeaponAimFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            !captureRightNativeWeaponAimFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            return false;
        }

        /*
         * The damped weapon-offset drivers ride the draw/equip transient: an
         * instant left takeover at equip time sampled that swing into the
         * conjugation and baked a rotated (visibly inverted) left seat and
         * hand for the whole first carry. The conjugation needs only the
         * fixed per-hand bone-in-wand map, so use the cached natural frames
         * captured while each hand was settled and authority-free.
         */
        RE::NiTransform rightHandWorld{};
        RE::NiTransform leftHandWorld{};
        if (!tryResolveNaturalWandHandOrientationFrame(
                false,
                rightHandWorld) ||
            !tryResolveNaturalWandHandOrientationFrame(
                true,
                leftHandWorld) ||
            !tryBuildMirroredLeftFiringHandWeaponLocalImpl(
                _firing.rightCanonicalHandWeaponLocal,
                _firing.rightCanonicalGripWeaponLocal,
                rightHandWorld,
                leftHandWorld,
                outFiringHandWeaponLocal,
                false,
                true)) {
            return false;
        }

        outFiringGripWeaponLocal = _firing.rightCanonicalGripWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryResolveNaturalWandHandOrientationFrame(
        const bool isLeft,
        RE::NiTransform& outHandWorld) const
    {
        outHandWorld = {};
        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* wand = playerNodes ?
            (isLeft ?
                    playerNodes->SecondaryWandNode :
                    playerNodes->primaryWandNode) :
            nullptr;
        const bool relationValid = isLeft ?
            _firing.hasLeftNaturalBoneInWand :
            _firing.hasRightNaturalBoneInWand;
        const RE::NiTransform& boneInWand = isLeft ?
            _firing.leftNaturalBoneInWand :
            _firing.rightNaturalBoneInWand;
        if (!wand || !relationValid ||
            !isFiniteTransform(wand->world) ||
            !isFiniteTransform(boneInWand)) {
            return false;
        }

        // Orientation authority only: cached translate/scale are presentation
        // state and may be collapsed (see the right-support mirror). The seat
        // builders anchor position independently through the palm pin.
        RE::NiTransform orientationOnly = boneInWand;
        orientationOnly.translate = {};
        orientationOnly.scale = 1.0f;
        outHandWorld = transform_math::composeTransforms(
            wand->world,
            orientationOnly);
        return isFiniteTransform(outHandWorld);
    }

    bool TwoHandedGrip::tryCaptureLeftFiringGripTransfer(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        RE::NiTransform& outFiringHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal)
    {
        outFiringHandWeaponLocal = {};
        outFiringGripWeaponLocal = {};

        const bool activeLeftCaptureCurrent =
            isManualOwnershipActive() &&
            usesLeftFiringCarry() &&
            _session.weaponNode == weaponNode &&
            currentEquippedWeaponOwnershipKey != 0 &&
            _session.equippedWeaponOwnershipKey ==
                currentEquippedWeaponOwnershipKey &&
            _firing.hasPrimaryHandWeaponLocal &&
            isFiniteTransform(_firing.primaryHandWeaponLocal) &&
            _firing.primaryGripConfidence > 0.0f &&
            std::isfinite(_firing.primaryGripLocal.x) &&
            std::isfinite(_firing.primaryGripLocal.y) &&
            std::isfinite(_firing.primaryGripLocal.z);
        if (activeLeftCaptureCurrent) {
            outFiringHandWeaponLocal = _firing.primaryHandWeaponLocal;
            outFiringGripWeaponLocal = _firing.primaryGripLocal;
            return true;
        }

        return tryBuildCurrentLeftFiringGripCapture(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            outFiringHandWeaponLocal,
            outFiringGripWeaponLocal);
    }

    bool TwoHandedGrip::beginPrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const bool firingHandIsLeft,
        const RE::NiTransform* capturedFiringHandWeaponLocal,
        const RE::NiPoint3* capturedFiringGripWeaponLocal,
        const bool retainUntilPhysicalGrip,
        const bool emitAttachHaptic)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0 || _session.state != TwoHandedState::Inactive ||
            !canBeginPrimaryOnlyGripForHand(firingHandIsLeft)) {
            return false;
        }

        RE::NiTransform resolvedLeftHandWeaponLocal{};
        RE::NiPoint3 resolvedLeftFiringGripWeaponLocal{};
        if (firingHandIsLeft) {
            const bool currentCanonicalResolved =
                tryBuildCurrentLeftFiringGripCapture(
                    weaponNode,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    resolvedLeftHandWeaponLocal,
                    resolvedLeftFiringGripWeaponLocal);
            if (!currentCanonicalResolved) {
                // Only a committed equipped-weapon transfer may reuse a
                // captured left seat. Loose-model holds include their legacy
                // aim trim in the hand relation; accepting one here would
                // apply that trim again to the separate native weapon aim.
                if (!retainUntilPhysicalGrip ||
                    !capturedFiringHandWeaponLocal ||
                    !isFiniteTransform(
                        *capturedFiringHandWeaponLocal) ||
                    !capturedFiringGripWeaponLocal ||
                    !std::isfinite(
                        capturedFiringGripWeaponLocal->x) ||
                    !std::isfinite(
                        capturedFiringGripWeaponLocal->y) ||
                    !std::isfinite(
                        capturedFiringGripWeaponLocal->z) ||
                    (!hasRightNativeWeaponAimFrame(
                         weaponNode,
                         currentWeaponGenerationKey,
                         currentEquippedWeaponOwnershipKey) &&
                        !captureRightNativeWeaponAimFrame(
                            weaponNode,
                            currentWeaponGenerationKey,
                            currentEquippedWeaponOwnershipKey))) {
                    return false;
                }
                resolvedLeftHandWeaponLocal =
                    *capturedFiringHandWeaponLocal;
                resolvedLeftFiringGripWeaponLocal =
                    *capturedFiringGripWeaponLocal;
            }
        }
        if (firingHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: left primary-grip start skipped because the hFRIK primary weapon-pose blocker is unavailable");
            return false;
        }

        setFiringHand(firingHandIsLeft, "primary-grip-start-hand");
        if (firingHandIsLeft) {
            _firing.primaryHandWeaponLocal = resolvedLeftHandWeaponLocal;
            _firing.hasPrimaryHandWeaponLocal = true;
        }

        if (!transitionToPrimaryOnly(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                "primary-grip-start")) {
            _firing.primaryHandWeaponLocal = {};
            _firing.hasPrimaryHandWeaponLocal = false;
            setFiringHand(false, "primary-grip-start-failed");
            restoreFrikPrimaryWeaponPose();
            return false;
        }
        if (firingHandIsLeft) {
            // Install the normalized authored/transfer grip after the state
            // transition's provisional live sample. The first right-native
            // frame must not redefine the selected left firing seat.
            _firing.primaryGripLocal = resolvedLeftFiringGripWeaponLocal;
            _firing.primaryGripConfidence = 1.0f;
        }
        // Only a fresh grab pulses; transitionToPrimaryOnly is also reached
        // from support-release paths where the firing grip never changed.
        if (emitAttachHaptic) {
            _hapticEvents.firingGripAttached = true;
            _hapticEvents.firingGripAttachedHandIsLeft =
                isFiringHandLeft();
        }
        _session.firingGripSequence = ++_session.gripCaptureSequence;
        if (retainUntilPhysicalGrip) {
            _firing.persistentCarryActive = true;
            _firing.persistentCarryDetachArmed = false;
            _firing.persistentCarryInputAcquisitionPending = false;
        }
        return true;
    }

    bool TwoHandedGrip::commitPersistentEquippedCarryInputAcquisition(
        const bool handIsLeft) noexcept
    {
        if (!_firing.persistentCarryActive ||
            !isManualOwnershipActive() ||
            !isFiringHand(handIsLeft)) {
            return false;
        }

        if (_firing.persistentCarryInputAcquisitionPending) {
            _firing.persistentCarryInputAcquisitionPending = false;
            _hapticEvents.firingGripAttached = true;
            _hapticEvents.firingGripAttachedHandIsLeft = handIsLeft;
        }
        _firing.persistentCarryDetachArmed = true;
        return true;
    }

    void TwoHandedGrip::clearPersistentEquippedCarry(const char* reason)
    {
        if (!_firing.persistentCarryActive) {
            return;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: clearing persistent left-hand carry reason={}",
            reason ? reason : "unknown");
        _firing.persistentCarryActive = false;
        _firing.persistentCarryDetachArmed = false;
        _firing.persistentCarryInputAcquisitionPending = false;
        if (isManualOwnershipActive()) {
            transitionToInactive(false);
        }
    }

    void TwoHandedGrip::restoreNativeRightEquippedCarry(const char* reason)
    {
        clearPersistentEquippedCarry(reason);
        if (!isManualOwnershipActive()) {
            return;
        }

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: restoring native right-hand carry reason={}",
            reason ? reason : "unknown");
        transitionToInactive(false);
    }

    bool TwoHandedGrip::transitionToPrimaryOnly(
        RE::NiNode* weaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const char* reason)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0) {
            return false;
        }

        clearDynamicSupportAcquisition(
            reason ? reason : "transition-to-primary-only",
            true);
        const bool primaryHandIsLeft = isFiringHandLeft();
        const bool supportHandIsLeft = isSupportHandLeft();

        if (_session.state == TwoHandedState::Inactive) {
            RE::NiTransform nativeWeaponLocalBaseline = weaponNode->local;
            if (_visuals.returningWeapon.localTransition.active && _visuals.returningWeapon.weaponNode == weaponNode) {
                nativeWeaponLocalBaseline = _visuals.returningWeapon.nativeBaselineLocal;
                clearWeaponVisualReturn("new-primary-acquisition", true, true);
            }
            _session.weaponNode = weaponNode;
            _session.weaponGenerationKey = currentWeaponGenerationKey;
            _session.equippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
            _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
            _hasWeaponNodeLocalBaseline = true;

            RE::NiTransform primaryTransform{};
            if (tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform)) {
                _firing.primaryGripLocal = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(primaryTransform, primaryHandIsLeft), weaponNode);
                _firing.primaryGripConfidence = 1.0f;
            } else {
                _firing.primaryGripLocal = {};
                _firing.primaryGripConfidence = 0.0f;
            }
        }
        _session.weaponGenerationKey = currentWeaponGenerationKey;
        _session.equippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;

        clearPrimaryGripFingerPose(
            primaryHandIsLeft,
            _visuals.returningWeapon.localTransition.active &&
                _visuals.returningWeapon.followsAuthoredPrimaryGrip);
        clearPrimaryGripWorldAuthority(primaryHandIsLeft);
        clearSupportGripPose(supportHandIsLeft);
        clearSupportGripPose(primaryHandIsLeft);
        clearPrimaryDetachVisualAuthority(primaryHandIsLeft);
        if (usesNativeRightCarry()) {
            // FRIK's primary weapon pose targets the game-primary RIGHT hand.
            // During RIGHT carry that native pose remains authoritative.
            // LEFT carry keeps it blocked and publishes its authored wrist,
            // fingers, and normalized weapon frame through ROCK instead.
            restoreFrikPrimaryWeaponPose();
        }
        _support.partGrips = {};
        _partCarry.pivotIsLeft = true;
        _partCarry.detachAuthority =
            immersive_weapon_policy::DetachAuthority::None;
        _partCarry.gripSeparationWorld = 0.0f;
        _hasSolvedWeaponTransform = _visuals.returningWeapon.localTransition.active && _visuals.hasLastRenderedWeaponWorld;
        if (_hasSolvedWeaponTransform) {
            _lastSolvedWeaponTransform = _visuals.lastRenderedWeaponWorld;
        }
        if (usesNativeRightCarry()) {
            /*
             * Right firing hand: PrimaryOnly is FRIK-native carry, so ROCK
             * deliberately holds no hand-to-weapon frame. A LEFT firing hand
             * has no native carry - its captured frame IS the carry solve and
             * must survive this transition (wiping it here was the "weapon
             * snaps back to the right hand" takeover regression).
             */
            _firing.primaryHandWeaponLocal = {};
            _firing.hasPrimaryHandWeaponLocal = false;
        }
        _visuals.primaryHandLerp = {};
        _session.state = TwoHandedState::PrimaryOnly;

        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: primary-only equipped weapon ownership active reason={} generation={:016X} ownership={:016X} provisional={}",
            reason ? reason : "unknown",
            _session.weaponGenerationKey,
            _session.equippedWeaponOwnershipKey,
            _session.weaponGenerationKey == 0 ? "yes" : "no");
        return true;
    }

    void TwoHandedGrip::updatePrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const EquippedWeaponPrimaryGripInput& primaryGripInput,
        const bool primaryDetachEnabled,
        const float dt)
    {
        equipped_weapon_manual_ownership_policy::RuntimeState manualState{
            .active = true,
            .ownershipKey = _session.equippedWeaponOwnershipKey,
        };
        const auto manualDecision = equipped_weapon_manual_ownership_policy::update(manualState,
            equipped_weapon_manual_ownership_policy::Input{
                .weaponEquipped = weaponNode != nullptr,
                .ownershipKey = currentEquippedWeaponOwnershipKey,
                .startRequested = false,
                .primaryGripRetained = equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership(
                    primaryDetachEnabled,
                    _handlingSettings.toggleGrabEnabled,
                    primaryGripInput.held),
                .supportGripRetained = false,
            });

        if (manualDecision.dropRequested) {
            beginHandVisualReturn(isFiringHandLeft(), "primary-only-drop");
            if (_handlingSettings.detachAuthority ==
                immersive_weapon_policy::DetachAuthority::
                    IntegratedImmersive) {
                recordFiringGripDetachedHaptic();
            }
            requestEquippedWeaponDrop("primary-only-grip-released",
                isFiringHandLeft() ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right);
            return;
        }

        if (manualDecision.cleared) {
            beginHandVisualReturn(isFiringHandLeft(), "primary-only-released");
            if (usesLeftFiringCarry()) {
                beginWeaponVisualReturn("left-primary-only-released");
            }
            transitionToInactive(false);
            return;
        }

        if (usesNativeRightCarry()) {
            // Right firing hand: FRIK-native carry, ROCK bookkeeping only.
            _hasSolvedWeaponTransform = false;
            return;
        }

        // Left firing hand: mirror the native right weapon aim into the left
        // wand, translate its authored grip point onto the physical palm, and
        // publish the authored left wrist/fingers as separate presentation.
        (void)solveLeftFiringWeaponCarry(weaponNode, dt);
    }

    bool TwoHandedGrip::firingGripContactMatchesCapturedGrip(
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& handWeaponContact,
        const RE::NiTransform& handTransform,
        const bool handIsLeft) const
    {
        if (!weaponNode || !handWeaponContact.valid) {
            return false;
        }

        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(handWeaponContact.weaponGenerationKey, _session.weaponGenerationKey)) {
            return false;
        }

        const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, handIsLeft);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
        const RE::NiPoint3 delta = sub(palm, firingGripWorld);
        const float distance = std::sqrt(dot(delta, delta));
        return std::isfinite(distance) &&
               distance <= _handlingSettings.firingGripReattachRadiusGameUnits;
    }

    bool TwoHandedGrip::tryEvaluateFiringGripReattachZoneForHand(
        RE::NiNode* weaponNode,
        const bool handIsLeft,
        firing_grip_reattach_zone_policy::ZoneResult& outZone)
    {
        outZone = {};
        auto& approach = _firing.reattachApproach[handIsLeft ? 0u : 1u];
        RE::NiTransform handTransform{};
        if (!weaponNode ||
            !isFiniteTransform(weaponNode->world) ||
            !tryGetSolverHandTransform(handIsLeft, handTransform)) {
            approach = {};
            return false;
        }
        /*
         * The lateral axis is the seated canonical RIGHT palm normal on this
         * weapon (weapon LEFT), the same axis the authored support activation
         * cone is built from. The zone is symmetric in its sign, so one axis
         * serves both hands and both sides. Part carry always follows a
         * captured firing grip, so the canonical hold is expected; without it
         * the zone fails closed instead of guessing a weapon axis.
         */
        if (!hasRightFiringHandCanonicalFrame(
                weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey)) {
            approach = {};
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                g_rockConfig.rockLogSampleMilliseconds,
                "TwoHandedGrip: firing-grip reattach zone unavailable hand={} reason=noCanonicalRightHold",
                handIsLeft ? "left" : "right");
            return false;
        }
        const RE::NiTransform seatedRightHandWorld =
            transform_math::composeTransforms(
                weaponNode->world,
                _firing.rightCanonicalHandWeaponLocal);
        if (!isFiniteTransform(seatedRightHandWorld)) {
            approach = {};
            return false;
        }

        using firing_grip_reattach_zone_policy::Vec3;
        const auto toZoneVector = [](const RE::NiPoint3& value) {
            return Vec3{ value.x, value.y, value.z };
        };
        outZone = firing_grip_reattach_zone_policy::evaluateZone(
            firing_grip_reattach_zone_policy::ZoneInput{
                .gripWorld = toZoneVector(
                    weaponLocalToWorld(_firing.primaryGripLocal, weaponNode)),
                .palmWorld = toZoneVector(
                    computeGrabLegacyPalmPivotAWorldFromHandBasis(
                        handTransform,
                        handIsLeft)),
                .weaponLeftAxisWorld = toZoneVector(
                    computePalmNormalFromHandBasis(seatedRightHandWorld, false)),
                .lastStableDirectionWorld = toZoneVector(
                    approach.lastStableDirectionWorld),
                .radialCapGameUnits =
                    _handlingSettings.firingGripReattachRadiusGameUnits,
                .lastStableDirectionValid = approach.valid,
            });
        if (outZone.directionValid && !outZone.usedLastStableDirection) {
            approach.lastStableDirectionWorld = RE::NiPoint3{
                outZone.approachDirectionWorld.x,
                outZone.approachDirectionWorld.y,
                outZone.approachDirectionWorld.z,
            };
            approach.valid = true;
        }
        return true;
    }

    bool TwoHandedGrip::tryResolveAuthoredFiringHandCanonicalForProbe(
        const bool handIsLeft,
        RE::NiTransform& outHandWeaponLocal,
        const char*& outSource) const
    {
        outHandWeaponLocal = {};
        outSource = "unavailable";
        constexpr std::uint16_t kCompleteFingerLocalTransformMask = 0x7FFFu;
        if (!hasRightFiringHandCanonicalFrame(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) ||
            _firing.rightCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        if (handIsLeft) {
            if (_firing.leftFingerLocalTransformMask !=
                kCompleteFingerLocalTransformMask) {
                return false;
            }
            bool usedAuthoredCanonical = false;
            if (!tryComputeMirroredLeftFiringHandWeaponLocal(
                    outHandWeaponLocal,
                    &usedAuthoredCanonical) ||
                !usedAuthoredCanonical) {
                return false;
            }
            outSource = "authored-mirror-probe";
            return true;
        }

        if (_firing.rightFingerLocalTransformMask !=
            kCompleteFingerLocalTransformMask) {
            return false;
        }
        outHandWeaponLocal = _firing.rightCanonicalHandWeaponLocal;
        outSource = "authored-canonical-probe";
        return isFiniteTransform(outHandWeaponLocal);
    }

    bool TwoHandedGrip::tryReattachFiringGrip(
        const bool handIsLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionContact& handWeaponContact,
        const bool authoredProviderAuthorityActive,
        const bool authoredAttachOnlyAuthorityActive)
    {
        if (!weaponNode ||
            !handWeaponContact.valid ||
            !weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(
                handWeaponContact.weaponGenerationKey,
                _session.weaponGenerationKey)) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(handIsLeft, handTransform)) {
            return false;
        }

        RE::NiTransform authoredProbeCanonical{};
        const char* authoredProbeCanonicalSource = "unavailable";
        const bool authoredProbeCanonicalAvailable =
            tryResolveAuthoredFiringHandCanonicalForProbe(
                handIsLeft,
                authoredProbeCanonical,
                authoredProbeCanonicalSource);
        const bool useAuthoredProbeCanonical =
            authored_weapon_grip_capture_policy::shouldUseAuthoredFiringGripProbe(
                authored_weapon_grip_capture_policy::AuthoredFiringGripProbeInput{
                    .proximityProbeAcquisition =
                        handWeaponContact.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe,
                    .providerAuthorityActive = authoredProviderAuthorityActive,
                    .attachOnly = authoredAttachOnlyAuthorityActive,
                    .authoredCanonicalAvailable =
                        authoredProbeCanonicalAvailable,
                });
        if (!useAuthoredProbeCanonical &&
            !firingGripContactMatchesCapturedGrip(
                weaponNode,
                handWeaponContact,
                handTransform,
                handIsLeft)) {
            return false;
        }

        /*
         * Reattach forces the CANONICAL per-hand hold instead of freezing the
         * live squeeze orientation: the LEFT hand takes the canonical
         * right-hand hold MIRRORED (authored offsets adapted to the left bone
         * basis), the RIGHT hand re-takes its canonical native hold directly.
         * A live squeeze capture both fired with the weapon crooked and, for
         * the right hand, poisoned the canonical itself through the snapshot
         * below. The live capture remains only as the no-canonical fallback.
         */
        bool usedCanonicalHold = false;
        bool usedAuthoredCanonical = false;
        const char* holdSource = "live-capture";
        if (useAuthoredProbeCanonical) {
            _firing.primaryHandWeaponLocal = authoredProbeCanonical;
            usedCanonicalHold = true;
            usedAuthoredCanonical = true;
            holdSource = authoredProbeCanonicalSource;
        } else if (handIsLeft) {
            RE::NiTransform mirroredHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    mirroredHandWeaponLocal,
                    &usedAuthoredCanonical)) {
                _firing.primaryHandWeaponLocal = mirroredHandWeaponLocal;
                usedCanonicalHold = true;
                holdSource = usedAuthoredCanonical ?
                    "authored-mirror" :
                    "native-mirror";
            }
        } else if (hasRightFiringHandCanonicalFrame(
                       _session.weaponNode,
                       _session.weaponGenerationKey,
                       _session.equippedWeaponOwnershipKey)) {
            _firing.primaryHandWeaponLocal = _firing.rightCanonicalHandWeaponLocal;
            usedCanonicalHold = true;
            holdSource = _firing.rightCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" :
                "native-canonical";
        }
        if (handIsLeft && !usedCanonicalHold) {
            // Left carry has no safe live-capture fallback: it requires both
            // the authored left wrist seat and the generation-bound native
            // right weapon-in-wand aim frame.
            return false;
        }
        if (!usedCanonicalHold) {
            const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, handIsLeft);
            const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
            const RE::NiTransform adjustedHandTransform =
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palm, firingGripWorld);
            _firing.primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        }

        // Validated: commit. A takeover by the other hand flips the firing
        // role only after its complete normalized hold is ready.
        if (!isFiringHand(handIsLeft)) {
            setFiringHand(handIsLeft, "firing-grip-reattach-other-hand");
        }
        _firing.hasPrimaryHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _session.firingGripSequence = ++_session.gripCaptureSequence;
        _visuals.primaryHandLerp = {};
        clearPrimaryDetachVisualAuthority(handIsLeft);
        if (usesNativeRightCarry()) {
            restoreFrikPrimaryWeaponPose();
        }
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = handIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: firing hand reattached at configured grip hand={} hold={} detachSource={}",
            handIsLeft ? "left" : "right",
            holdSource,
            immersive_weapon_policy::authorityName(
                _handlingSettings.detachAuthority));
        return true;
    }

    bool TwoHandedGrip::setAuthoredPrimaryFiringGripCanonical(
        RE::NiNode* weaponNode,
        const RE::NiTransform& rightHandWeaponLocal,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        const std::uint64_t captureSequence,
        const authored_weapon_grip_library::FiringFingerPose* rightFingerPose,
        const authored_weapon_grip_library::FiringFingerPose* leftFingerPose)
    {
        const auto validFingerPose = [](const authored_weapon_grip_library::FiringFingerPose* pose) {
            if (!pose) {
                return true;
            }
            if (!pose->complete()) {
                return false;
            }
            return std::ranges::all_of(pose->localTransforms, [](const RE::NiTransform& transform) { return isFiniteTransform(transform) && std::abs(transform.scale) > 0.0001f; });
        };
        if (!weaponNode ||
            weaponOwnershipKey == 0 ||
            captureSequence == 0 ||
            !isFiniteTransform(rightHandWeaponLocal) ||
            std::abs(rightHandWeaponLocal.scale) <= 0.0001f || !validFingerPose(rightFingerPose) || !validFingerPose(leftFingerPose) || (leftFingerPose && !rightFingerPose)) {
            return false;
        }

        // HandFrame helpers are frame-agnostic: feeding Hand-in-Weapon yields
        // the configured right palm seat directly in Weapon coordinates. The
        // mirror needs this authored seat, not _firing.primaryGripLocal (which can be
        // a live squeeze or an older native-offset capture).
        const RE::NiPoint3 authoredGripWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                rightHandWeaponLocal,
                false);
        if (!std::isfinite(authoredGripWeaponLocal.x) ||
            !std::isfinite(authoredGripWeaponLocal.y) ||
            !std::isfinite(authoredGripWeaponLocal.z)) {
            return false;
        }

        const std::uint16_t incomingRightFingerMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        const std::uint16_t incomingLeftFingerMask = leftFingerPose ? leftFingerPose->enabledMask : 0;
        const bool fingerPoseBoundary =
            _firing.rightFingerLocalTransformMask != incomingRightFingerMask ||
            _firing.leftFingerLocalTransformMask != incomingLeftFingerMask;
        const bool sourceBoundary =
            _firing.rightCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation ||
            _firing.rightCanonicalWeaponNode != weaponNode ||
            _firing.rightCanonicalGenerationKey != weaponGenerationKey ||
            _firing.rightCanonicalOwnershipKey != weaponOwnershipKey ||
            fingerPoseBoundary;

        _firing.rightCanonicalHandWeaponLocal = rightHandWeaponLocal;
        _firing.rightCanonicalGripWeaponLocal = authoredGripWeaponLocal;
        _firing.rightCanonicalWeaponNode = weaponNode;
        _firing.rightCanonicalGenerationKey = weaponGenerationKey;
        _firing.rightCanonicalOwnershipKey = weaponOwnershipKey;
        _firing.rightCanonicalCaptureSequence = captureSequence;
        _firing.rightCanonicalSource =
            RightFiringCanonicalSource::AuthoredAnimation;
        _firing.hasRightCanonicalHandWeaponLocal = true;
        _firing.rightFingerLocalTransforms = rightFingerPose ? rightFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _firing.rightFingerLocalTransformMask = rightFingerPose ? rightFingerPose->enabledMask : 0;
        _firing.leftFingerLocalTransforms = leftFingerPose ? leftFingerPose->localTransforms : std::array<RE::NiTransform, 15>{};
        _firing.leftFingerLocalTransformMask = leftFingerPose ? leftFingerPose->enabledMask : 0;

        if (sourceBoundary) {
            ROCK_LOG_INFO(Animation,
                "TwoHandedGrip: authored firing canonical active generation={:016X} ownership={:016X} capture={} handWeaponT=({:.3f},{:.3f},{:.3f}) "
                "gripWeapon=({:.3f},{:.3f},{:.3f}) mode=position-only rightFingerMask=0x{:04X} leftFingerMask=0x{:04X} leftSource=authored-seat-mirror",
                weaponGenerationKey,
                weaponOwnershipKey,
                captureSequence,
                rightHandWeaponLocal.translate.x,
                rightHandWeaponLocal.translate.y,
                rightHandWeaponLocal.translate.z,
                authoredGripWeaponLocal.x,
                authoredGripWeaponLocal.y,
                authoredGripWeaponLocal.z,
                _firing.rightFingerLocalTransformMask,
                _firing.leftFingerLocalTransformMask);
        }
        return true;
    }

    bool TwoHandedGrip::retainAuthoredPrimaryFiringGripFingerPoseForHandoff(
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey)
    {
        const bool generationCompatible =
            _firing.rightCanonicalGenerationKey == weaponGenerationKey ||
            _firing.rightCanonicalGenerationKey == 0 ||
            weaponGenerationKey == 0;
        if (!weaponNode || weaponOwnershipKey == 0 ||
            !_firing.hasRightCanonicalHandWeaponLocal ||
            _firing.rightCanonicalWeaponNode != weaponNode ||
            _firing.rightCanonicalOwnershipKey != weaponOwnershipKey ||
            !generationCompatible ||
            _firing.rightCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }
        return publishAuthoredPrimaryFiringGripFingerPose(false);
    }

    bool TwoHandedGrip::publishAuthoredPrimaryFiringGripFingerPose(const bool isLeft)
    {
        const bool targetHandHoldingObject =
            isLeft ? _firing.leftHandHoldingObjectForPose : _firing.rightHandHoldingObjectForPose;
        const bool firingHandDetached = _session.state == TwoHandedState::PartCarry;
        if (firingHandDetached && _firing.authoredFingerPosePublished) {
            // Fail closed: a detached firing hand never keeps a grip pose,
            // whichever hand published it.
            clearAuthoredPrimaryFiringGripFingerPose();
        }
        if (_firing.authoredFingerPoseSuppressed ||
            !authored_weapon_grip_capture_policy::shouldPublishAuthoredFiringFingerPose(
                targetHandHoldingObject,
                firingHandDetached) ||
            _firing.rightCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        const auto& transforms = isLeft ? _firing.leftFingerLocalTransforms : _firing.rightFingerLocalTransforms;
        const std::uint16_t mask = isLeft ? _firing.leftFingerLocalTransformMask : _firing.rightFingerLocalTransformMask;
        if (mask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }

        if (_firing.authoredFingerPosePublished && _firing.publishedFingerPoseIsLeft != isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }

        if (!_firing.authoredFingerPoseBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, true)) {
                return false;
            }
            _firing.authoredFingerPoseBlockEngaged = true;
        }

        const auto hand = handFromBool(isLeft);
        _firing.publishedFingerPoseIsLeft = isLeft;
        if (!frik_visual_authority::setHandPoseCustomWithPriority(PRIMARY_GRIP_TAG, hand, frik_visual_authority::HandPoseData{}, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride overrideData{};
        overrideData.enabledMask = mask;
        for (std::size_t index = 0; index < transforms.size(); ++index) {
            overrideData.localTransforms[index] = transforms[index];
        }
        if (!frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(PRIMARY_GRIP_TAG, hand, &overrideData, GRIP_HAND_POSE_PRIORITY)) {
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        _firing.publishedFingerPoseIsLeft = isLeft;
        _firing.authoredFingerPosePublished = true;
        return true;
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripFingerPose()
    {
        if (_firing.authoredFingerPosePublished || _firing.authoredFingerPoseBlockEngaged) {
            (void)frik_visual_authority::clearHandPose(PRIMARY_GRIP_TAG, handFromBool(_firing.publishedFingerPoseIsLeft));
        }
        if (_firing.authoredFingerPoseBlockEngaged) {
            (void)frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, false);
        }
        _firing.publishedFingerPoseIsLeft = false;
        _firing.authoredFingerPosePublished = false;
        _firing.authoredFingerPoseBlockEngaged = false;
    }

    void TwoHandedGrip::setAuthoredPrimaryFiringGripFingerPoseSuppressed(const bool suppressed)
    {
        _firing.authoredFingerPoseSuppressed = suppressed;
        if (suppressed) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::setGrabbedObjectHandPoseOwnership(
        const bool leftHandHoldingObject,
        const bool rightHandHoldingObject)
    {
        _firing.leftHandHoldingObjectForPose = leftHandHoldingObject;
        _firing.rightHandHoldingObjectForPose = rightHandHoldingObject;

        if (!_firing.authoredFingerPosePublished) {
            return;
        }

        const bool publishedHandHoldingObject =
            _firing.publishedFingerPoseIsLeft ?
                _firing.leftHandHoldingObjectForPose :
                _firing.rightHandHoldingObjectForPose;
        if (publishedHandHoldingObject) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringGripCanonical(
        const char* reason)
    {
        if (_firing.authoredHandWorldActive) {
            clearAuthoredPrimaryFiringHandWorldAuthority();
        }
        if (_firing.rightCanonicalSource !=
            RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        ROCK_LOG_DEBUG(Animation,
            "TwoHandedGrip: clearing authored firing canonical reason={} generation={:016X} ownership={:016X} capture={}",
            reason ? reason : "unknown",
            _firing.rightCanonicalGenerationKey,
            _firing.rightCanonicalOwnershipKey,
            _firing.rightCanonicalCaptureSequence);
        clearAuthoredPrimaryFiringGripFingerPose();
        clearRightFiringHandCanonicalFrame();
    }

    bool TwoHandedGrip::applyAuthoredPrimaryGripWeaponAlignment(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld,
        const RE::NiTransform& solvedFiringHandWorld,
        const std::uint64_t currentWeaponGenerationKey)
    {
        if (blocksAuthoredPrimaryGripWeaponAlignment() || isWeaponVisualReturnActive()) {
            return false;
        }

        if (!isUsableHandAuthorityTransform(solvedFiringHandWorld) ||
            !frik_visual_authority::applyExternalHandWorldTransform(
                PRIMARY_GRIP_TAG,
                frik_visual_authority::Hand::Right,
                solvedFiringHandWorld,
                GRIP_HAND_POSE_PRIORITY)) {
            if (_firing.authoredHandWorldActive) {
                clearAuthoredPrimaryFiringHandWorldAuthority();
            }
            return false;
        }
        // Publish the fixed weapon after the hand: hFRIK's arm solve excludes
        // the weapon subtree, but the weapon world must remain the final
        // authored frame in this presentation pass.
        if (!applyWeaponVisualAuthority(
                weaponNode,
                solvedWeaponWorld,
                currentWeaponGenerationKey)) {
            clearAuthoredPrimaryFiringHandWorldAuthority();
            return false;
        }

        _firing.authoredHandWorldActive = true;
        _firing.authoredHandWorldRefreshed = true;
        recordScopeHandAuthorityPublication(
            scope_safe_hand_frame_math::HandAuthorityRole::PrimaryGrip,
            false);
        clearHandVisualReturn(
            false,
            "authored-position-only-hand-acquired",
            false);
        recordPublishedHandWorld(false, solvedFiringHandWorld);
        return true;
    }

    void TwoHandedGrip::beginAuthoredPrimaryFiringGripFrame()
    {
        _firing.authoredHandWorldRefreshed = false;
    }

    void TwoHandedGrip::finishAuthoredPrimaryFiringGripFrame()
    {
        if (_firing.authoredHandWorldActive &&
            !_firing.authoredHandWorldRefreshed) {
            clearAuthoredPrimaryFiringHandWorldAuthority();
        }
    }

    void TwoHandedGrip::clearAuthoredPrimaryFiringHandWorldAuthority()
    {
        // clearPrimaryGripWorldAuthority(false) resets the position-only
        // state itself so every existing PrimaryGrip clear path releases
        // this presentation through the same role machinery.
        clearPrimaryGripWorldAuthority(false);
    }

    bool TwoHandedGrip::tryGetAuthoredPrimaryTrackedFiringHandWorld(
        RE::NiTransform& outHandWorld) const
    {
        RE::NiTransform driverWorld{};
        if (tryResolvePhysicalHandFrame(false, outHandWorld, driverWorld)) {
            return true;
        }
        // While ROCK presents the right firing hand, the rendered hand is
        // ROCK's own output; reading it back would freeze the solve. The
        // natural relation refreshes again as soon as authority releases.
        if (_firing.authoredHandWorldActive) {
            outHandWorld = {};
            return false;
        }
        return tryGetSolverHandTransform(false, outHandWorld) &&
               isUsableHandAuthorityTransform(outHandWorld);
    }

    void TwoHandedGrip::clearRightFiringHandCanonicalFrame()
    {
        _firing.rightCanonicalHandWeaponLocal = {};
        _firing.rightCanonicalGripWeaponLocal = {};
        _firing.rightCanonicalWeaponNode = nullptr;
        _firing.rightCanonicalGenerationKey = 0;
        _firing.rightCanonicalOwnershipKey = 0;
        _firing.rightCanonicalCaptureSequence = 0;
        _firing.rightCanonicalSource = RightFiringCanonicalSource::None;
        _firing.hasRightCanonicalHandWeaponLocal = false;
        _firing.rightFingerLocalTransforms = {};
        _firing.leftFingerLocalTransforms = {};
        _firing.rightFingerLocalTransformMask = 0;
        _firing.leftFingerLocalTransformMask = 0;
    }

    bool TwoHandedGrip::hasRightFiringHandCanonicalFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode &&
               weaponGenerationKey != 0 &&
               _firing.hasRightCanonicalHandWeaponLocal &&
               _firing.rightCanonicalWeaponNode == weaponNode &&
               _firing.rightCanonicalGenerationKey == weaponGenerationKey &&
               _firing.rightCanonicalOwnershipKey == weaponOwnershipKey &&
               _firing.rightCanonicalSource != RightFiringCanonicalSource::None;
    }

    void TwoHandedGrip::rememberRightFiringHandCanonicalFrame()
    {
        if (usesLeftFiringCarry() || !_session.weaponNode ||
            !_firing.hasPrimaryHandWeaponLocal || _session.weaponGenerationKey == 0) {
            return;
        }
        if (hasRightFiringHandCanonicalFrame(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) &&
            _firing.rightCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }
        _firing.rightCanonicalHandWeaponLocal = _firing.primaryHandWeaponLocal;
        _firing.rightCanonicalGripWeaponLocal = _firing.primaryGripLocal;
        _firing.rightCanonicalWeaponNode = _session.weaponNode;
        _firing.rightCanonicalGenerationKey = _session.weaponGenerationKey;
        _firing.rightCanonicalOwnershipKey = _session.equippedWeaponOwnershipKey;
        _firing.rightCanonicalCaptureSequence = 0;
        _firing.rightCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _firing.hasRightCanonicalHandWeaponLocal = true;
    }

    bool TwoHandedGrip::canCaptureRightNativeWeaponAimFrame() const
    {
        if (usesLeftFiringCarry() || _leftCarry.weaponNodeOwnershipBlockEngaged ||
            _visuals.returningWeapon.localTransition.active ||
            _visuals.weaponCollisionHandPresentationFromPreviousFrame[1] ||
            !scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(
                _scope.menuOpenThisFrame,
                _scope.safeHandFrames[1].rootRebaseActive)) {
            return false;
        }

        if (_session.state == TwoHandedState::Inactive ||
            _session.state == TwoHandedState::PrimaryOnly) {
            return true;
        }
        return _session.state == TwoHandedState::Gripping &&
               !weapon_support_authority_policy::
                   supportGripOwnsWeaponTransform(_session.authorityMode);
    }

    bool TwoHandedGrip::captureRightNativeWeaponAimFrame(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiNode* rightWand =
            playerNodes ? playerNodes->primaryWandNode : nullptr;
        if (!canCaptureRightNativeWeaponAimFrame() || !weaponNode ||
            currentEquippedWeaponOwnershipKey == 0 ||
            !rightWand || !isFiniteTransform(weaponNode->world) ||
            !isFiniteTransform(rightWand->world)) {
            return false;
        }

        RE::NiTransform weaponInRightWand =
            transform_math::composeTransforms(
                transform_math::invertTransform(rightWand->world),
                weaponNode->world);
        weaponInRightWand =
            left_firing_position_only_math::orientationOnly(
                weaponInRightWand);
        if (!isFiniteTransform(weaponInRightWand)) {
            return false;
        }

        const bool identityChanged =
            !_firing.rightNativeWeaponAimFrame.valid ||
            _firing.rightNativeWeaponAimFrame.weaponNodeIdentity != weaponNode ||
            _firing.rightNativeWeaponAimFrame.weaponGenerationKey !=
                currentWeaponGenerationKey ||
            _firing.rightNativeWeaponAimFrame.weaponOwnershipKey !=
                currentEquippedWeaponOwnershipKey;
        _firing.rightNativeWeaponAimFrame = RightNativeWeaponAimFrame{
            .weaponInWandOrientation = weaponInRightWand,
            .weaponNodeIdentity = weaponNode,
            .weaponGenerationKey = currentWeaponGenerationKey,
            .weaponOwnershipKey = currentEquippedWeaponOwnershipKey,
            .valid = true,
        };

        if (identityChanged) {
            const RE::NiPoint3 barrelInRightWand =
                transform_math::localVectorToWorld(
                    weaponInRightWand,
                    RE::NiPoint3{ 0.0f, 1.0f, 0.0f });
            ROCK_LOG_INFO(
                Weapon,
                "TwoHandedGrip: native right weapon aim captured generation={:016X} ownership={:016X} barrelInRightWand=({:.3f},{:.3f},{:.3f})",
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                barrelInRightWand.x,
                barrelInRightWand.y,
                barrelInRightWand.z);
        }
        return true;
    }

    bool TwoHandedGrip::hasRightNativeWeaponAimFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode && weaponOwnershipKey != 0 &&
               _firing.rightNativeWeaponAimFrame.valid &&
               _firing.rightNativeWeaponAimFrame.weaponNodeIdentity == weaponNode &&
               _firing.rightNativeWeaponAimFrame.weaponGenerationKey ==
                   weaponGenerationKey &&
               _firing.rightNativeWeaponAimFrame.weaponOwnershipKey ==
                   weaponOwnershipKey &&
               isFiniteTransform(
                   _firing.rightNativeWeaponAimFrame.weaponInWandOrientation);
    }

    void TwoHandedGrip::refreshNaturalHandInWandFrames()
    {
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return;
        }

        constexpr float kMaxBoneToDriverDistance = 30.0f;
        const auto refreshHand = [&](const bool isLeft,
                                     RE::NiNode* wandNode,
                                     RE::NiNode* dampedDriverNode,
                                     RE::NiTransform& outBoneInWand,
                                     bool& outWandValid,
                                     RE::NiTransform& outBoneInDampedDriver,
                                     bool& outDampedDriverValid) {
            if (hasVisualAuthorityForHand(isLeft)) {
                return;
            }

            RE::NiTransform handWorld{};
            if (!tryGetSolverHandTransform(isLeft, handWorld)) {
                return;
            }

            const auto captureRelation = [&](RE::NiNode* sourceNode,
                                             RE::NiTransform& outRelation,
                                             bool& outValid) {
                if (!sourceNode || !isFiniteTransform(sourceNode->world)) {
                    return;
                }
                const RE::NiTransform relation =
                    transform_math::composeTransforms(
                        transform_math::invertTransform(sourceNode->world),
                        handWorld);
                if (!isFiniteTransform(relation) ||
                    std::sqrt(dot(
                        relation.translate,
                        relation.translate)) >
                        kMaxBoneToDriverDistance) {
                    return;
                }
                outRelation = relation;
                outValid = true;
            };

            captureRelation(wandNode, outBoneInWand, outWandValid);
            captureRelation(
                dampedDriverNode,
                outBoneInDampedDriver,
                outDampedDriverValid);
        };

        refreshHand(
            false,
            playerNodes->primaryWandNode,
            playerNodes->primaryWeaponOffsetNOde,
            _firing.rightNaturalBoneInWand,
            _firing.hasRightNaturalBoneInWand,
            _firing.rightNaturalBoneInDampedDriver,
            _firing.hasRightNaturalBoneInDampedDriver);
        refreshHand(
            true,
            playerNodes->SecondaryWandNode,
            playerNodes->SecondaryMeleeWeaponOffsetNode2,
            _firing.leftNaturalBoneInWand,
            _firing.hasLeftNaturalBoneInWand,
            _firing.leftNaturalBoneInDampedDriver,
            _firing.hasLeftNaturalBoneInDampedDriver);
    }

    void TwoHandedGrip::refreshRightNativeCanonicalFrame(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        /*
         * Passive canonical capture: whenever the equipped weapon rides the
         * native RIGHT hand (no ROCK transform ownership), the live weapon
         * pose already carries FRIK's authored per-weapon offsets, so the
         * canonical right hold and its weapon-in-wand frame can refresh
         * continuously. Without this, a weapon that was never
         * right-firing-gripped in the session had no canonical, and a LEFT
         * takeover fell back to the raw squeeze capture - the per-weapon
         * offsets (e.g. the UMP's large forward offset) silently missing
         * from the mirrored left hold ("worked before by coincidence").
         */
        if (_visuals.weaponCollisionHandPresentationFromPreviousFrame[1] ||
            isManualOwnershipActive() || _leftCarry.weaponNodeOwnershipBlockEngaged ||
            _visuals.returningWeapon.localTransition.active ||
            !scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(_scope.menuOpenThisFrame, _scope.safeHandFrames[1].rootRebaseActive) || !weaponNode ||
            currentWeaponGenerationKey == 0 || !isFiniteTransform(weaponNode->world)) {
            return;
        }
        auto* playerNodes = f4vr::getPlayerNodes();
        RE::NiTransform rightHandWorld{};
        if (!playerNodes || !playerNodes->primaryWandNode ||
            !isFiniteTransform(playerNodes->primaryWandNode->world) ||
            !tryGetSolverHandTransform(false, rightHandWorld)) {
            return;
        }
        const RE::NiTransform boneInRightWand = transform_math::composeTransforms(
            transform_math::invertTransform(playerNodes->primaryWandNode->world), rightHandWorld);
        // Same wrist-range gate as the mirror's wand-map sampling, plus a
        // loose weapon-to-hand bound so a mid-equip/mid-teleport frame never
        // poisons the canonical.
        constexpr float kMaxBoneToWandDistance = 30.0f;
        constexpr float kMaxWeaponToHandDistance = 100.0f;
        const RE::NiPoint3 weaponToHand = sub(weaponNode->world.translate, rightHandWorld.translate);
        if (!isFiniteTransform(boneInRightWand) ||
            std::sqrt(dot(boneInRightWand.translate, boneInRightWand.translate)) > kMaxBoneToWandDistance ||
            std::sqrt(dot(weaponToHand, weaponToHand)) > kMaxWeaponToHandDistance) {
            return;
        }
        const RE::NiTransform canonicalHold = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), rightHandWorld);
        const RE::NiPoint3 canonicalGrip = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(rightHandWorld, false), weaponNode);
        if (!isFiniteTransform(canonicalHold) || !std::isfinite(canonicalGrip.x) || !std::isfinite(canonicalGrip.y) || !std::isfinite(canonicalGrip.z)) {
            return;
        }
        (void)captureRightNativeWeaponAimFrame(
            weaponNode,
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey);
        // The animation capture is a more direct authority than a later
        // presentation sample. Preserve it for this exact weapon identity,
        // generation, and ownership.
        if (hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            _firing.rightCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        _firing.rightCanonicalHandWeaponLocal = canonicalHold;
        _firing.rightCanonicalGripWeaponLocal = canonicalGrip;
        _firing.rightCanonicalWeaponNode = weaponNode;
        _firing.rightCanonicalGenerationKey = currentWeaponGenerationKey;
        _firing.rightCanonicalOwnershipKey = currentEquippedWeaponOwnershipKey;
        _firing.rightCanonicalCaptureSequence = 0;
        _firing.rightCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _firing.hasRightCanonicalHandWeaponLocal = true;
    }

    bool TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal(
        RE::NiTransform& outHandWeaponLocal,
        bool* outUsedAuthoredCanonical,
        const bool logDiagnostic) const
    {
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = false;
        }
        if (!hasRightFiringHandCanonicalFrame(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) ||
            !hasRightNativeWeaponAimFrame(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey)) {
            return false;
        }

        // Both relations come from the damped physical drivers. A takeover is
        // normally initiated while the left hand is still visually locked as
        // the support grip, so either rendered bone would contaminate the
        // authored seat with ROCK's own previous output.
        RE::NiTransform leftHandWorld{};
        RE::NiTransform rightHandWorld{};
        RE::NiTransform leftDriverWorld{};
        RE::NiTransform rightDriverWorld{};
        if (!tryResolvePhysicalHandFrame(
                true,
                leftHandWorld,
                leftDriverWorld) ||
            !tryResolvePhysicalHandFrame(
                false,
                rightHandWorld,
                rightDriverWorld)) {
            return false;
        }

        const bool mirrored =
            tryBuildMirroredLeftFiringHandWeaponLocalImpl(
                _firing.rightCanonicalHandWeaponLocal,
                _firing.rightCanonicalGripWeaponLocal,
                rightHandWorld,
                leftHandWorld,
                outHandWeaponLocal,
                false,
                logDiagnostic);
        if (!mirrored) {
            return false;
        }

        const bool usedAuthoredCanonical =
            _firing.rightCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = usedAuthoredCanonical;
        }
        if (logDiagnostic) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: authored left firing seat resolved source={} generation={:016X} ownership={:016X} capture={} handWeaponT=({:.3f},{:.3f},{:.3f}) nativeAim=ready",
                usedAuthoredCanonical ? "authored-animation" : "native-carry",
                _firing.rightCanonicalGenerationKey,
                _firing.rightCanonicalOwnershipKey,
                _firing.rightCanonicalCaptureSequence,
                outHandWeaponLocal.translate.x,
                outHandWeaponLocal.translate.y,
                outHandWeaponLocal.translate.z);
        }
        return true;
    }

    bool TwoHandedGrip::tryBuildMirroredRightSupportHandWeaponLocal(
        const RE::NiTransform& leftHandWeaponLocal,
        RE::NiTransform& outRightHandWeaponLocal) const
    {
        outRightHandWeaponLocal = {};
        if (!_firing.hasLeftNaturalBoneInWand || !_firing.hasRightNaturalBoneInWand ||
            !isFiniteTransform(leftHandWeaponLocal) ||
            !isFiniteTransform(_firing.leftNaturalBoneInWand) ||
            !isFiniteTransform(_firing.rightNaturalBoneInWand)) {
            return false;
        }

        /*
         * Mirror only ORIENTATION through the physical wand pair. The cached
         * bone-in-wand transforms remove hFRIK's asymmetric hand-bone
         * conventions, but their translations/scales are presentation state
         * and can be collapsed while ROCK owns left-primary carry. Feeding
         * those affine values through inverse composition produced enormous
         * intermediate translations and a catastrophically cancelled right
         * support seat. Position is anchored independently below, so rigid
         * zero-origin frames are the complete source authority here.
         */
        const auto orientationFrame = [](const RE::NiTransform& source) {
            RE::NiTransform result = source;
            result.translate = {};
            result.scale = 1.0f;
            return result;
        };
        const RE::NiTransform leftBoneInWandOrientation =
            orientationFrame(_firing.leftNaturalBoneInWand);
        const RE::NiTransform rightBoneInWandOrientation =
            orientationFrame(_firing.rightNaturalBoneInWand);
        const RE::NiTransform leftHandWeaponOrientation =
            orientationFrame(leftHandWeaponLocal);

        const RE::NiTransform weaponInLeftWand = transform_math::composeTransforms(
            leftBoneInWandOrientation,
            transform_math::invertTransform(leftHandWeaponOrientation));
        const RE::NiTransform weaponInRightWand =
            conjugateAcrossLateralMirror(weaponInLeftWand);
        const RE::NiTransform weaponInRightHand = transform_math::composeTransforms(
            transform_math::invertTransform(rightBoneInWandOrientation),
            weaponInRightWand);
        RE::NiTransform mirroredRightHandWeaponLocal = transform_math::invertTransform(weaponInRightHand);
        if (!isFiniteTransform(mirroredRightHandWeaponLocal)) {
            return false;
        }

        /*
         * Position is anchored directly by the actual solver palm seat.
         * Reflect the authored left seat across weapon-local X, clear the
         * orientation solve's translation, then place the right hand from its
         * own palm offset. This avoids subtracting two huge nearly-equal
         * floats and keeps the result weapon-relative and controller-free.
         */
        const RE::NiPoint3 leftPalmWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(leftHandWeaponLocal, true);
        const RE::NiPoint3 desiredRightPalmWeaponLocal{
            -leftPalmWeaponLocal.x,
            leftPalmWeaponLocal.y,
            leftPalmWeaponLocal.z,
        };
        mirroredRightHandWeaponLocal.translate = {};
        mirroredRightHandWeaponLocal.scale = leftHandWeaponLocal.scale;
        const RE::NiPoint3 rightPalmOffsetWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(mirroredRightHandWeaponLocal, false);
        if (!std::isfinite(desiredRightPalmWeaponLocal.x) ||
            !std::isfinite(desiredRightPalmWeaponLocal.y) ||
            !std::isfinite(desiredRightPalmWeaponLocal.z) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.x) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.y) ||
            !std::isfinite(rightPalmOffsetWeaponLocal.z)) {
            return false;
        }
        mirroredRightHandWeaponLocal.translate =
            sub(desiredRightPalmWeaponLocal, rightPalmOffsetWeaponLocal);
        if (!isFiniteTransform(mirroredRightHandWeaponLocal)) {
            return false;
        }

        const RE::NiPoint3 anchoredRightPalmWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                mirroredRightHandWeaponLocal,
                false);
        const RE::NiPoint3 anchorError =
            sub(anchoredRightPalmWeaponLocal, desiredRightPalmWeaponLocal);
        constexpr float kMaxPalmAnchorErrorGameUnits = 0.01f;
        if (!std::isfinite(anchorError.x) ||
            !std::isfinite(anchorError.y) ||
            !std::isfinite(anchorError.z) ||
            std::sqrt(dot(anchorError, anchorError)) >
                kMaxPalmAnchorErrorGameUnits) {
            return false;
        }

        outRightHandWeaponLocal = mirroredRightHandWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal(
        const RE::NiTransform& canonicalRightHandWeaponLocal,
        const RE::NiPoint3& firingGripWeaponLocal,
        const RE::NiTransform& rightHandWorld,
        const RE::NiTransform& leftHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const bool logDiagnostic)
    {
        return tryBuildMirroredLeftFiringHandWeaponLocalImpl(
            canonicalRightHandWeaponLocal,
            firingGripWeaponLocal,
            rightHandWorld,
            leftHandWorld,
            outHandWeaponLocal,
            true,
            logDiagnostic);
    }

    bool TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocalImpl(
        const RE::NiTransform& canonicalRightHandWeaponLocal,
        const RE::NiPoint3& firingGripWeaponLocal,
        const RE::NiTransform& rightHandWorld,
        const RE::NiTransform& leftHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const bool applyHandlingTrim,
        const bool logDiagnostic)
    {
        const auto& handlingSettings =
            equipped_weapon_handling_runtime::current();
        if (!isFiniteTransform(canonicalRightHandWeaponLocal) ||
            !std::isfinite(firingGripWeaponLocal.x) ||
            !std::isfinite(firingGripWeaponLocal.y) ||
            !std::isfinite(firingGripWeaponLocal.z)) {
            return false;
        }

        /*
         * WAND-CONJUGATION MIRROR. The aim requirement is controller-
         * relative: the tuned right-hand offsets align the barrel with the
         * RIGHT controller's forward, so the mirrored hold must align it
         * with the LEFT controller's forward with the lateral components
         * negated ("2 degrees left of the right wand" becomes "2 degrees
         * right of the left wand"). Left/right WAND device frames are the
         * physically exact mirror pair; the hand BONE conventions are not
         * mirrors (previous semantic-palm and bone-anchor mirrors both left
         * a residual yaw/side bias in-game). Conjugating the canonical hold
         * through the wand pair cancels every per-hand bone convention
         * inside the live-sampled bone-in-wand transforms:
         *
         *   weaponInLeftWand = Msag o weaponInRightWand o Mside
         *
         * with two reflections keeping the result a proper rotation: Msag
         * mirrors across the wand's sagittal plane (wand-local X lateral -
         * same axis family as the weapon frame the wand chain parents) and
         * Mside across the weapon's own side plane (+Y barrel, +X side),
         * which maps the grip from the weapon's right flank to its left.
         * Effect on the tuned offsets: yaw and roll negate, pitch and
         * fore/aft/vertical placement are preserved.
         *
         * The native first-person arm sync drags each hand bone to its wand
         * with a fixed per-hand map, so bone-in-wand is constant and
         * sampling it at takeover time is exact.
         */
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return false;
        }
        // Ambidextrous stands down in game-left-handed mode, so primary is
        // always the physical RIGHT wand here.
        RE::NiNode* rightWand = playerNodes->primaryWandNode;
        RE::NiNode* leftWand = playerNodes->SecondaryWandNode;
        if (!rightWand || !leftWand ||
            !isFiniteTransform(rightWand->world) || !isFiniteTransform(leftWand->world)) {
            return false;
        }

        const RE::NiTransform boneInRightWand =
            transform_math::composeTransforms(transform_math::invertTransform(rightWand->world), rightHandWorld);
        const RE::NiTransform boneInLeftWand =
            transform_math::composeTransforms(transform_math::invertTransform(leftWand->world), leftHandWorld);
        // A hand bone rides its wand at wrist range; a large offset means a
        // stale or foreign frame - fail closed to the live-capture fallback.
        constexpr float kMaxBoneToWandDistance = 30.0f;
        const auto transformOffsetLength = [](const RE::NiTransform& transform) {
            return std::sqrt(dot(transform.translate, transform.translate));
        };
        if (!isFiniteTransform(boneInRightWand) || !isFiniteTransform(boneInLeftWand) ||
            transformOffsetLength(boneInRightWand) > kMaxBoneToWandDistance ||
            transformOffsetLength(boneInLeftWand) > kMaxBoneToWandDistance) {
            return false;
        }

        const RE::NiTransform weaponInRightWand = transform_math::composeTransforms(
            boneInRightWand, transform_math::invertTransform(canonicalRightHandWeaponLocal));
        RE::NiTransform weaponInLeftWand =
            conjugateAcrossLateralMirror(weaponInRightWand);

        /*
         * Loose-model callers have no equipped native weapon-in-wand frame,
         * so their legacy solver hold still owns the optional aim trim here.
         * Equipped carry requests the untrimmed authored wrist relation and
         * applies the same trim to its separate native weapon orientation.
         */
        if (applyHandlingTrim) {
            weaponInLeftWand = transform_math::composeTransforms(
                makeLeftFiringWandAimTrim(handlingSettings),
                weaponInLeftWand);
        }

        const RE::NiTransform weaponInLeftHand = transform_math::composeTransforms(
            transform_math::invertTransform(boneInLeftWand), weaponInLeftWand);
        RE::NiTransform mirroredHandWeaponLocal = transform_math::invertTransform(weaponInLeftHand);

        if (!isFiniteTransform(mirroredHandWeaponLocal)) {
            return false;
        }

        /*
         * PALM-ANCHORED POSITION: the wand conjugation is the ORIENTATION
         * authority only. Deriving the translation through frame mirroring
         * left per-weapon height errors that no global knob can fix (UMP
         * too low while the P226 sits too high - weapons with authored
         * FRIK rotations/offsets each landed differently, because any
         * residual rotation-convention error displaces a hold by an amount
         * proportional to that weapon's own offsets). Instead the FIRING
         * GRIP POINT is pinned per weapon: it must sit at the same place in
         * the left palm as it does in the right palm. ROCK's hand bases
         * correspond anatomically with only Z flipped - empirical, from the
         * user-tuned palm pivots R(6.0,-2.0,+0.2) / L(6.0,-2.0,-0.2) - so
         * the target is simply (x, y, -z) of the grip's right-hand-local
         * position, plus the global offset knobs as palm-space nudges.
         * Per-weapon exact by construction; residuals are global-only.
         */
        const RE::NiPoint3 gripInRightHand = transform_math::localPointToWorld(
            transform_math::invertTransform(canonicalRightHandWeaponLocal), firingGripWeaponLocal);
        const float offsetX = applyHandlingTrim ?
            handlingSettings.leftFiringAimOffsetXGameUnits : 0.0f;
        const float offsetY = applyHandlingTrim ?
            handlingSettings.leftFiringAimOffsetYGameUnits : 0.0f;
        const float offsetZ = applyHandlingTrim ?
            handlingSettings.leftFiringAimOffsetZGameUnits : 0.0f;
        const RE::NiPoint3 gripTargetInLeftHand{
            gripInRightHand.x + offsetX,
            gripInRightHand.y + offsetY,
            -gripInRightHand.z + offsetZ
        };
        if (std::isfinite(gripTargetInLeftHand.x) && std::isfinite(gripTargetInLeftHand.y) && std::isfinite(gripTargetInLeftHand.z)) {
            RE::NiTransform anchoredWeaponInLeftHand = transform_math::invertTransform(mirroredHandWeaponLocal);
            const RE::NiPoint3 gripRotatedOnly = sub(
                transform_math::localPointToWorld(anchoredWeaponInLeftHand, firingGripWeaponLocal),
                anchoredWeaponInLeftHand.translate);
            anchoredWeaponInLeftHand.translate = sub(gripTargetInLeftHand, gripRotatedOnly);
            const RE::NiTransform anchoredHold = transform_math::invertTransform(anchoredWeaponInLeftHand);
            if (isFiniteTransform(anchoredHold)) {
                mirroredHandWeaponLocal = anchoredHold;
            }
        }

        // Takeover-event diagnostic: barrel (+Y weapon) direction in each
        // wand frame. A correct mirror negates x and preserves y/z; a wand
        // axis-convention mismatch shows up here as a different component
        // flipping.
        if (logDiagnostic) {
            const RE::NiPoint3 barrelInRightWand =
                sub(transform_math::localPointToWorld(weaponInRightWand, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), weaponInRightWand.translate);
            const RE::NiPoint3 barrelInLeftWand =
                sub(transform_math::localPointToWorld(weaponInLeftWand, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }), weaponInLeftWand.translate);
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: wand-conjugated left hand seat trim={} barrelInRightWand=({:.3f},{:.3f},{:.3f}) barrelInLeftWand=({:.3f},{:.3f},{:.3f}) boneWandDist=({:.2f},{:.2f})",
                applyHandlingTrim ? "legacy-hold" : "authored-only",
                barrelInRightWand.x,
                barrelInRightWand.y,
                barrelInRightWand.z,
                barrelInLeftWand.x,
                barrelInLeftWand.y,
                barrelInLeftWand.z,
                transformOffsetLength(boneInRightWand),
                transformOffsetLength(boneInLeftWand));
        }

        outHandWeaponLocal = mirroredHandWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryPromoteSupportGripToFiringGrip(RE::NiNode* weaponNode, const float dt)
    {
        const bool supportHandIsLeft = isSupportHandLeft();
        if (!weaponNode || !_handlingSettings.ambidextrousHandoffEnabled ||
            !canBeginPrimaryOnlyGripForHand(supportHandIsLeft)) {
            return false;
        }

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        // Pose selection (provider/authored/dynamic) and current transform
        // authority (visual-only/full) are orthogonal to handoff capability.
        // AttachOnly glue alone cannot inherit the firing grip. The distance
        // gate below keeps every other promotable grip tied to the firing grip.
        if (!weapon_support_authority_policy::canPromoteSupportGripToFiringGrip(
                supportGrip.active,
                supportGrip.attachOnly)) {
            return false;
        }

        /*
         * Promotion distance uses the support GRIP POINT (where the hand
         * actually grabbed the weapon), not the palm pivot: a shooting-cup
         * palm sits a hand-width away from the grip center and the tight
         * reattach radius silently declined every takeover. The dedicated
         * promotion radius keeps handguard/foregrip support grips out.
         */
        const RE::NiPoint3 supportGripWorld = resolvePartGripWorld(supportGrip, weaponNode);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
        const RE::NiPoint3 gripDelta = sub(supportGripWorld, firingGripWorld);
        const float supportGripToFiringGripDistance = std::sqrt(dot(gripDelta, gripDelta));
        if (!std::isfinite(supportGripToFiringGripDistance) ||
            supportGripToFiringGripDistance >
                _handlingSettings.firingGripPromotionRadiusGameUnits) {
            return false;
        }

        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, handTransform)) {
            return false;
        }

        // A left firing hand needs FRIK's right-hand weapon pose blocked for
        // the whole left-firing tenure; abort the promotion if that fails.
        if (supportHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            return false;
        }

        /*
         * Commit: the support hand takes over the SAME weapon-relative firing
         * grip in place, forcing the CANONICAL per-hand hold: a LEFT takeover
         * applies the canonical right-hand hold mirrored (authored offsets
         * adapted to the left bone basis), a RIGHT takeover re-takes its
         * canonical native hold directly - the promoted hand's live bone is
         * still part-grip-locked here, so a live capture froze that locked
         * angle and (for the right) poisoned the canonical snapshot below.
         * The live capture remains only as the no-canonical fallback.
         */
        RE::NiTransform newFiringHandWeaponLocal{};
        bool usedCanonicalHold = false;
        bool usedAuthoredCanonical = false;
        const char* holdSource = "live-capture";
        if (supportHandIsLeft) {
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    newFiringHandWeaponLocal,
                    &usedAuthoredCanonical)) {
                usedCanonicalHold = true;
                holdSource = usedAuthoredCanonical ?
                    "authored-mirror" :
                    "native-mirror";
            }
        } else if (hasRightFiringHandCanonicalFrame(
                       _session.weaponNode,
                       _session.weaponGenerationKey,
                       _session.equippedWeaponOwnershipKey)) {
            newFiringHandWeaponLocal = _firing.rightCanonicalHandWeaponLocal;
            usedCanonicalHold = true;
            holdSource = _firing.rightCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" :
                "native-canonical";
        }
        if (supportHandIsLeft && !usedCanonicalHold) {
            restoreFrikPrimaryWeaponPose();
            return false;
        }
        if (!usedCanonicalHold) {
            const RE::NiPoint3 palm = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, supportHandIsLeft);
            const RE::NiTransform adjustedHandTransform =
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palm, firingGripWorld);
            newFiringHandWeaponLocal =
                transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        }

        beginHandVisualReturn(isFiringHandLeft(), "ambidextrous-firing-hand-promotion");
        if (_handlingSettings.detachAuthority ==
            immersive_weapon_policy::DetachAuthority::
                IntegratedImmersive) {
            recordFiringGripDetachedHaptic();
        }
        setFiringHand(supportHandIsLeft, "support-grip-promotion");
        if (!transitionToPrimaryOnly(weaponNode, _session.weaponGenerationKey, _session.equippedWeaponOwnershipKey, "firing-grip-hand-promotion")) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: firing-grip promotion failed to enter primary-only; clearing authority");
            transitionToInactive(false);
            return true;
        }

        _firing.primaryHandWeaponLocal = newFiringHandWeaponLocal;
        _firing.hasPrimaryHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        if (usesLeftFiringCarry() &&
            !solveLeftFiringWeaponCarry(weaponNode, dt)) {
            return true;
        }
        _session.firingGripSequence = ++_session.gripCaptureSequence;
        _visuals.primaryHandLerp = {};
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = isFiringHandLeft();
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: support hand promoted to firing grip hand={} gripToGrip={:.2f} hold={}",
            firingHandName(),
            supportGripToFiringGripDistance,
            holdSource);
        return true;
    }
}

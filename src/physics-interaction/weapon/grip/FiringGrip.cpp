#include "physics-interaction/weapon/TwoHandedGripInternal.h"
#include "physics-interaction/weapon/NativeSecondaryPose.h"
#include "physics-interaction/weapon/WeaponGripCalibration.h"
#include "physics-interaction/weapon/HoldingGripAdapter.h"
#include "physics-interaction/weapon/WeaponGripSeat.h"

#include "physics-interaction/hand/TrackedHandIsolationPolicy.h"
#include "physics-interaction/weapon/WeaponAimBasis.h"

// Firing-hand grip: canonical right frame, right weapon aim orientation, left/right mirroring, authored primary firing grip canonical and finger pose, reattach, support-to-firing promotion, and PrimaryOnly (persistent equipped carry) sessions.

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
        RE::NiPoint3& outFiringGripWeaponLocal,
        const char** outFailureReason)
    {
        if (outFailureReason) {
            *outFailureReason = nullptr;
        }
        const auto reject = [&](const char* reason) {
            if (outFailureReason) {
                *outFailureReason = reason;
            }
            return false;
        };
        outFiringHandWeaponLocal = {};
        outFiringGripWeaponLocal = {};
        if (!canBeginPrimaryOnlyGripForHand(true)) {
            return reject("left-infrastructure-unavailable");
        }
        if (!hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            return reject("right-firing-canonical-unavailable");
        }

        RE::NiTransform rightWeaponAimInWand{};
        if (!tryResolveRightWeaponAimInWand(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                rightWeaponAimInWand)) {
            return reject("right-weapon-aim-unavailable");
        }

        if(_holdingHasLeft){outFiringHandWeaponLocal=_holdingLeft;outFiringGripWeaponLocal=computeGrabPalmPointWorldFromHandBasis(_holdingLeft,true);return true;}
        if (!weapon_grip_seat::leftFiringSeat(
                _holdingHasNative ? _holdingNativeRight : _firing.rightCanonicalHandWeaponLocal,
                _holdingMirrorPlaneX, outFiringHandWeaponLocal)) {
            return reject("mirrored-firing-frame-unavailable");
        }
        outFiringGripWeaponLocal = computeGrabPalmPointWorldFromHandBasis(outFiringHandWeaponLocal, true);
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
        const bool emitAttachHaptic,
        const char** outFailureReason)
    {
        if (outFailureReason) {
            *outFailureReason = nullptr;
        }
        const auto reject = [&](const char* reason) {
            if (outFailureReason) {
                *outFailureReason = reason;
            }
            return false;
        };
        if (!weaponNode) {
            return reject("weapon-node-unavailable");
        }
        if (currentEquippedWeaponOwnershipKey == 0) {
            return reject("weapon-ownership-unavailable");
        }
        if (_session.state != TwoHandedState::Inactive) {
            return reject("grip-session-already-active");
        }
        if (!canBeginPrimaryOnlyGripForHand(firingHandIsLeft)) {
            return reject("left-infrastructure-unavailable");
        }

        RE::NiTransform resolvedLeftHandWeaponLocal{};
        RE::NiPoint3 resolvedLeftFiringGripWeaponLocal{};
        if (firingHandIsLeft) {
            const char* canonicalFailureReason = nullptr;
            const bool currentCanonicalResolved =
                tryBuildCurrentLeftFiringGripCapture(
                    weaponNode,
                    currentWeaponGenerationKey,
                    currentEquippedWeaponOwnershipKey,
                    resolvedLeftHandWeaponLocal,
                    resolvedLeftFiringGripWeaponLocal,
                    &canonicalFailureReason);
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
                        capturedFiringGripWeaponLocal->z)) {
                    return reject(canonicalFailureReason);
                }
                RE::NiTransform rightWeaponAimInWand{};
                if (!tryResolveRightWeaponAimInWand(
                        weaponNode,
                        currentWeaponGenerationKey,
                        currentEquippedWeaponOwnershipKey,
                        rightWeaponAimInWand)) {
                    return reject("right-weapon-aim-unavailable");
                }
                resolvedLeftHandWeaponLocal =
                    *capturedFiringHandWeaponLocal;
                resolvedLeftFiringGripWeaponLocal =
                    *capturedFiringGripWeaponLocal;
            }
        }
        if (firingHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            return reject("primary-weapon-pose-blocker-unavailable");
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
            return reject("primary-only-transition-rejected");
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
                // Right-primary acquisition changes input ownership only.
                // Its in-flight return already lands on the authored carry;
                // cancelling it exposes the intermediate node for one frame.
                if (!usesNativeRightCarry()) {
                    clearWeaponVisualReturn("new-primary-acquisition", true, true);
                }
            }
            _session.weaponNode = weaponNode;
            _session.weaponGenerationKey = currentWeaponGenerationKey;
            _session.equippedWeaponOwnershipKey = currentEquippedWeaponOwnershipKey;
            _weaponNodeLocalBaseline = nativeWeaponLocalBaseline;
            _hasWeaponNodeLocalBaseline = true;

            RE::NiTransform primaryTransform{};
            if (tryGetSolverHandTransform(primaryHandIsLeft, primaryTransform)) {
                _firing.primaryGripLocal = worldToWeaponLocal(computeGrabPalmPointWorldFromHandBasis(primaryTransform, primaryHandIsLeft), weaponNode);
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
        if (!_visuals.returningWeapon.localTransition.active ||
            !_visuals.returningWeapon.keepFiringHandAttached) {
            clearPrimaryGripWorldAuthority(primaryHandIsLeft);
        }
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
        clearWeaponPoseHandoffBlend("transition-to-primary-only", true);
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
        _firing.transferredPrimaryGrip = {};
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
        performance_profiler::ScopedTimer gripStageTimer(performance_profiler::Scope::EquippedGripSolve);
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
                    primaryGripInput.held,
                    _handlingSettings.lastGripReleaseDropEnabled || canRetainOnSurface()),
                .supportGripRetained = false,
            });

        if (manualDecision.active &&
            primaryDetachEnabled &&
            !_handlingSettings.lastGripReleaseDropEnabled &&
            !primaryGripInput.held) {
            // The only carrier's open hand was refused a drop; the carry
            // continues unchanged below.
            recordGripReleaseRetained(isFiringHandLeft(), "primary-only");
        }

        if (manualDecision.dropRequested) {
            requestEquippedWeaponDrop("primary-only-grip-released",
                isFiringHandLeft() ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right,
                equipped_weapon_drop_policy::fromSetting(g_rockConfig.rockWeaponDropMode), dt, false, true);
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

        const RE::NiPoint3 palm = computeGrabPalmPointWorldFromHandBasis(handTransform, handIsLeft);
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
        const RE::NiPoint3 delta = sub(palm, firingGripWorld);
        const float distance = std::sqrt(dot(delta, delta));
        return std::isfinite(distance) &&
               distance <= _handlingSettings.firingGripReattachRadiusGameUnits;
    }

    bool TwoHandedGrip::tryBuildFiringGripZoneInput(
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        const float reachGameUnits,
        firing_grip_reattach_zone_policy::ZoneInput& outInput) const
    {
        outInput = {};
        if (!weaponNode ||
            !isInvertibleTransform(weaponNode->world)) {
            return false;
        }
        /*
         * The lateral axis is the seated canonical RIGHT palm normal on this
         * weapon (weapon LEFT), the same axis the authored support activation
         * cone is built from. The zone is symmetric in its sign, so one axis
         * serves handoff and reattachment on both sides. Before acquisition,
         * use the current canonical grip; an active session keeps its captured
         * grip. Missing or stale canonical data cannot create a hover zone.
         */
        if (!hasRightFiringHandCanonicalFrame(
                weaponNode,
                weaponGenerationKey,
                weaponOwnershipKey)) {
            if (_session.state == TwoHandedState::PartCarry) {
                ROCK_LOG_SAMPLE_WARN(Weapon, g_rockConfig.rockLogSampleMilliseconds,
                    "TwoHandedGrip: firing-grip zone unavailable reason=noCurrentCanonicalRightHold nodeMatch={} generation={:016X}/{:016X} ownership={:016X}/{:016X} captured={}",
                    _firing.rightCanonicalWeaponNode == weaponNode, _firing.rightCanonicalGenerationKey, weaponGenerationKey,
                    _firing.rightCanonicalOwnershipKey, weaponOwnershipKey, _firing.hasRightCanonicalHandWeaponLocal);
            }
            return false;
        }
        const RE::NiTransform seatedRightHandWorld =
            transform_math::composeTransforms(
                weaponNode->world,
                _firing.rightCanonicalHandWeaponLocal);
        if (!isFiniteTransform(seatedRightHandWorld)) {
            return false;
        }

        const bool capturedGripCurrent =
            _firing.hasPrimaryHandWeaponLocal &&
            _session.weaponNode == weaponNode &&
            _session.weaponGenerationKey == weaponGenerationKey &&
            _session.equippedWeaponOwnershipKey == weaponOwnershipKey;
        const RE::NiPoint3 gripWorld = weaponLocalToWorld(
            capturedGripCurrent ? _firing.primaryGripLocal :
                _firing.rightCanonicalGripWeaponLocal,
            weaponNode);
        const RE::NiPoint3 weaponLeftAxisWorld =
            computePalmNormalFromHandBasis(seatedRightHandWorld, false);
        using firing_grip_reattach_zone_policy::Vec3;
        const auto toZoneVector = [](const RE::NiPoint3& value) {
            return Vec3{ value.x, value.y, value.z };
        };
        outInput = {
            .gripWorld = toZoneVector(gripWorld),
            .weaponLeftAxisWorld = toZoneVector(weaponLeftAxisWorld),
            .reachGameUnits = reachGameUnits,
            .radiusGameUnits =
                _handlingSettings.firingGripReattachCylinderRadiusGameUnits,
        };
        return true;
    }

    bool TwoHandedGrip::tryEvaluateFiringGripZoneForHand(
        const bool handIsLeft,
        const firing_grip_reattach_zone_policy::ZoneInput& input,
        firing_grip_reattach_zone_policy::ZoneResult& outZone)
    {
        outZone = {};
        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(handIsLeft, handTransform)) {
            return false;
        }
        const RE::NiPoint3 palmWorld =
            computeGrabPalmPointWorldFromHandBasis(handTransform, handIsLeft);
        auto handInput = input;
        handInput.palmWorld = { palmWorld.x, palmWorld.y, palmWorld.z };
        outZone = firing_grip_reattach_zone_policy::evaluateZone(handInput);
        if (!outZone.axisValid) {
            return false;
        }

        // Overlay record: values only, rebuilt every update().
        auto& debugSnapshot = _firing.reattachDebugSnapshot;
        debugSnapshot.gripWorld = {
            input.gripWorld.x, input.gripWorld.y, input.gripWorld.z };
        debugSnapshot.weaponLeftAxisWorld = {
            input.weaponLeftAxisWorld.x, input.weaponLeftAxisWorld.y,
            input.weaponLeftAxisWorld.z };
        debugSnapshot.reachGameUnits = input.reachGameUnits;
        debugSnapshot.cylinderRadiusGameUnits = input.radiusGameUnits;
        debugSnapshot.valid = true;
        auto& handSample = debugSnapshot.hands[handIsLeft ? 0u : 1u];
        handSample.palmWorld = palmWorld;
        handSample.alongAxisGameUnits = outZone.alongAxisGameUnits;
        handSample.perpendicularDistanceGameUnits =
            outZone.perpendicularDistanceGameUnits;
        handSample.radialDistanceGameUnits = outZone.radialDistanceGameUnits;
        handSample.side = outZone.side;
        handSample.evaluated = true;
        handSample.reachPass = outZone.reachPass;
        handSample.radiusPass = outZone.radiusPass;
        handSample.inside = outZone.inside;
        return true;
    }

    void TwoHandedGrip::updateFiringGripZoneIndicator(
        RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const bool handIsLeft,
        const firing_grip_reattach_zone_policy::ZoneResult& zone)
    {
        _firing.reattachIndicatorFrame = {};
        if (!weaponNode || !zone.inside || !zone.indicatorValid ||
            !isInvertibleTransform(weaponNode->world)) {
            return;
        }
        const RE::NiPoint3 indicatorWorld{
            zone.indicatorWorld.x, zone.indicatorWorld.y, zone.indicatorWorld.z };
        const auto indicatorLocal = transform_math::worldPointToLocal(
            weaponNode->world, indicatorWorld);
        const bool valid = std::isfinite(indicatorLocal.x) &&
            std::isfinite(indicatorLocal.y) && std::isfinite(indicatorLocal.z);
        _firing.reattachIndicatorFrame = {
            .positionWeaponLocal = indicatorLocal,
            .weaponGenerationKey = weaponGenerationKey,
            .handIsLeft = handIsLeft,
            .weaponLocalValid = valid,
            .visible = valid,
        };
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
        const bool coreAuthoredStation = !authoredProviderAuthorityActive && !authoredAttachOnlyAuthorityActive;
        if (!weaponNode || weaponNode != _session.weaponNode ||
            (!coreAuthoredStation && (!handWeaponContact.valid ||
                !weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(
                    handWeaponContact.weaponGenerationKey, _session.weaponGenerationKey)))) return false;
        if (coreAuthoredStation) {
            firing_grip_reattach_zone_policy::ZoneInput zoneInput{};
            firing_grip_reattach_zone_policy::ZoneResult zone{};
            if (!tryBuildFiringGripZoneInput(weaponNode, _session.weaponGenerationKey,
                    _session.equippedWeaponOwnershipKey, _handlingSettings.firingGripReattachRadiusGameUnits, zoneInput) ||
                !tryEvaluateFiringGripZoneForHand(handIsLeft, zoneInput, zone) || !zone.inside) return false;
            provider::PartQuery query{};
            query.weaponGenerationKey = _session.weaponGenerationKey;
            query.bodyId = weapon_part_runtime::kInvalidBodyId;
            weapon_part_runtime::Resolution resolution{};
            (void)provider::resolveWeaponPartTarget(query, resolution);
            if (resolution.whitelistActive) return false;
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
                        coreAuthoredStation || handWeaponContact.acquisitionSource ==
                        WeaponInteractionAcquisitionSource::ProximityProbe,
                    .providerAuthorityActive = authoredProviderAuthorityActive,
                    .attachOnly = authoredAttachOnlyAuthorityActive,
                    .authoredCanonicalAvailable =
                        authoredProbeCanonicalAvailable,
                });
        if (coreAuthoredStation && !useAuthoredProbeCanonical) return false;
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
            // Left carry requires the canonical mirrored hold.
            return false;
        }
        if (!usedCanonicalHold) {
            const RE::NiPoint3 palm = computeGrabPalmPointWorldFromHandBasis(handTransform, handIsLeft);
            const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_firing.primaryGripLocal, weaponNode);
            const RE::NiTransform adjustedHandTransform =
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(handTransform, palm, firingGripWorld);
            _firing.primaryHandWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
        }

        // Validated: commit. A takeover by the other hand flips the firing
        // role only after its complete normalized hold is ready.
        _firing.transferredPrimaryGrip = {};
        if (!isFiringHand(handIsLeft)) {
            setFiringHand(handIsLeft, "firing-grip-reattach-other-hand");
        }
        _firing.hasPrimaryHandWeaponLocal = true;
        if (usedCanonicalHold) {
            _firing.primaryGripLocal = handIsLeft ?
                computeGrabPalmPointWorldFromHandBasis(_firing.primaryHandWeaponLocal, true) :
                _firing.rightCanonicalGripWeaponLocal;
        }
        rememberRightFiringHandCanonicalFrame(_firing.rightCanonicalInstanceContentKey);
        _session.firingGripSequence = ++_session.gripCaptureSequence;
        _visuals.primaryHandLerp = {};
        if(handIsLeft&&_holdingHasLeft)_firing.primaryGripLocal=computeGrabPalmPointWorldFromHandBasis(_holdingLeft,true);
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
        const std::uint64_t weaponInstanceContentKey,
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
            computeGrabPalmPointWorldFromHandBasis(
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
        _firing.rightCanonicalInstanceContentKey = weaponInstanceContentKey;
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
        const bool firingHandDetached = isFiringGripDetached();
        const bool transferred = _session.state == TwoHandedState::Gripping && isFiringHand(isLeft) &&
            _firing.transferredPrimaryGrip.valid();
        if (firingHandDetached && _firing.authoredFingerPosePublished) {
            // Fail closed: a detached firing hand never keeps a grip pose,
            // whichever hand published it.
            clearAuthoredPrimaryFiringGripFingerPose();
        }
        if (_firing.authoredFingerPoseSuppressed ||
            !authored_weapon_grip_capture_policy::shouldPublishAuthoredFiringFingerPose(
                targetHandHoldingObject,
                firingHandDetached) ||
            (!transferred && _firing.rightCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation)) {
            return false;
        }

        const auto& transforms = transferred ? _firing.transferredPrimaryGrip.fingerLocals :
            (isLeft ? _firing.leftFingerLocalTransforms : _firing.rightFingerLocalTransforms);
        const std::uint16_t mask = transferred ? _firing.transferredPrimaryGrip.fingerMask :
            (isLeft ? _firing.leftFingerLocalTransformMask : _firing.rightFingerLocalTransformMask);
        if (!transferred && mask != authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }

        if (_firing.authoredFingerPosePublished && _firing.publishedFingerPoseIsLeft != isLeft) {
            clearAuthoredPrimaryFiringGripFingerPose();
        }

        if (!_firing.authoredFingerPoseBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryHandWeaponPose(AUTHORED_PRIMARY_POSE_BLOCK_TAG, true)) {
                ROCK_LOG_SAMPLE_WARN(Animation, 2000,
                    "Authored finger publication rejected stage=pose-block hand={} ownership={:016X} capture={}",
                    isLeft ? "left" : "right", _firing.rightCanonicalOwnershipKey, _firing.rightCanonicalCaptureSequence);
                return false;
            }
            _firing.authoredFingerPoseBlockEngaged = true;
        }

        const auto hand = handFromBool(isLeft);
        _firing.publishedFingerPoseIsLeft = isLeft;
        const auto fingerPose = transferred ? frik_visual_authority::makeHandPoseDataFromJointValues(
            _firing.transferredPrimaryGrip.fingerValues) : frik_visual_authority::HandPoseData{};
        if (!frik_visual_authority::setHandPoseCustom(PRIMARY_GRIP_TAG, hand, fingerPose, GRIP_HAND_POSE_PRIORITY)) {
            ROCK_LOG_SAMPLE_WARN(Animation, 2000,
                "Authored finger publication rejected stage=custom-pose hand={} ownership={:016X} capture={}",
                isLeft ? "left" : "right", _firing.rightCanonicalOwnershipKey, _firing.rightCanonicalCaptureSequence);
            clearAuthoredPrimaryFiringGripFingerPose();
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride overrideData{};
        overrideData.enabledMask = mask;
        for (std::size_t index = 0; index < transforms.size(); ++index) {
            overrideData.localTransforms[index] = transforms[index];
        }
        if (mask && !frik_visual_authority::setHandPoseCustomLocalTransforms(PRIMARY_GRIP_TAG, hand, &overrideData, GRIP_HAND_POSE_PRIORITY)) {
            ROCK_LOG_SAMPLE_WARN(Animation, 2000,
                "Authored finger publication rejected stage=finger-locals hand={} ownership={:016X} capture={} mask=0x{:04X}",
                isLeft ? "left" : "right", _firing.rightCanonicalOwnershipKey, _firing.rightCanonicalCaptureSequence, mask);
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
        _holdingHasLeft=false;_holdingHasNative=false;
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
            !frik_visual_authority::publishHandWorld(
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
                currentWeaponGenerationKey,
                true,
                true,
                dynamic_weapon_collision_policy::VisualIntentSource::AuthoredPrimary)) {
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

    bool TwoHandedGrip::tryGetRightWeaponAimWorld(const float weaponScale, RE::NiTransform& outWorld)
    {
        outWorld = {};
        if (frik_hand_world_authority::weaponFilteringActive()) {
            RE::NiTransform wand{};
            return frik_hand_world_authority::tryGetWeaponWandWorld(false, wand) &&
                weapon_aim_basis::tryResolveWorld(wand, weaponScale, outWorld);
        }
        const auto* nodes = f4vr::getPlayerNodes();
        return nodes && nodes->primaryWandNode &&
            weapon_aim_basis::tryResolveWorld(nodes->primaryWandNode->world, weaponScale, outWorld);
    }

    bool TwoHandedGrip::tryGetLeftWeaponAimWorld(const float weaponScale, RE::NiTransform& outWorld)
    {
        outWorld = {};
        RE::NiTransform wand{};
        if (!frik_hand_world_authority::tryGetWeaponWandWorld(true, wand)) return false;
        const auto& handling = equipped_weapon_handling_runtime::current();
        return weapon_aim_basis::tryResolveWorld(wand, weaponScale,
            native_secondary_pose::firingWeaponInWand(true, handling.leftFiringAimYawDegrees, handling.leftFiringAimPitchDegrees),
            outWorld);
    }

    bool TwoHandedGrip::tryGetMeleeWeaponAimWorld(const bool isLeft,
        const RE::NiTransform& physicalHandWorld,
        const RE::NiTransform& authoredHandWeaponLocal,
        const float weaponScale, RE::NiTransform& outWorld)
    {
        outWorld = {};
        const auto* nodes = f4vr::getPlayerNodes();
        const auto* wand = nodes ? (isLeft ? nodes->SecondaryWandNode : nodes->primaryWandNode) : nullptr;
        return wand && weapon_aim_basis::tryResolveMeleeWorld(
            physicalHandWorld, authoredHandWeaponLocal, wand->world,
            weaponScale, g_rockConfig.rockMeleeGripPitchDegrees, outWorld);
    }

    bool TwoHandedGrip::tryGetAuthoredPrimaryTrackedFiringHandWorld(
        RE::NiTransform& outHandWorld) const
    {
        // A fresh skeleton needs a claim-free calibration before we can
        // replace its tracked hand. Otherwise our first output prevents the
        // next frame from reconstructing input and ownership oscillates.
        if (!frik_hand_world_authority::hasCalibratedRawHandFrame(false)) {
            outHandWorld = {};
            return false;
        }
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
        _firing.rightCanonicalInstanceContentKey = 0;
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

    void TwoHandedGrip::rememberRightFiringHandCanonicalFrame(
        const std::uint64_t weaponInstanceContentKey)
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
        _firing.rightCanonicalInstanceContentKey = weaponInstanceContentKey;
        _firing.rightCanonicalCaptureSequence = 0;
        _firing.rightCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _firing.hasRightCanonicalHandWeaponLocal = true;
    }

    /*
     * The right hand's weapon-in-controller orientation, which left carry
     * mirrors. It is computed on demand: a firearm uses ROCK's fixed
     * controller basis and a melee weapon its authored grip, so neither
     * depends on where the weapon node was parented. (It was once captured
     * only during native right carry; a support-carry restore with the left
     * firing never captured it and refused every left reattach, runtime
     * 14 Oct 22:08:13.)
     */
    bool TwoHandedGrip::tryResolveRightWeaponAimInWand(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey,
        RE::NiTransform& outWeaponInRightWand) const
    {
        outWeaponInRightWand = weapon_aim_basis::weaponInController();
        const bool meleeWeapon = _recoil.weaponEvidence.resolved && _recoil.weaponEvidence.sizeClass == WeaponSizeClass::Melee;
        if (!meleeWeapon) {
            return true;
        }
        // The uncorrected authored basis. Left carry applies the current
        // user pitch at solve time.
        auto* playerNodes = f4vr::getPlayerNodes();
        const RE::NiNode* rightWand = playerNodes ? playerNodes->primaryWandNode : nullptr;
        RE::NiTransform naturalHandWorld{}, meleeWorld{};
        if (!rightWand || !isFiniteTransform(rightWand->world) ||
            !hasRightFiringHandCanonicalFrame(weaponNode, weaponGenerationKey, weaponOwnershipKey) ||
            _firing.rightCanonicalSource != RightFiringCanonicalSource::AuthoredAnimation ||
            !tryResolveNaturalWandHandOrientationFrame(false, naturalHandWorld) ||
            !weapon_aim_basis::tryResolveMeleeWorld(naturalHandWorld,
                _firing.rightCanonicalHandWeaponLocal, 1.0f, meleeWorld)) {
            return false;
        }
        outWeaponInRightWand = transform_math::composeTransforms(transform_math::invertTransform(rightWand->world), meleeWorld);
        outWeaponInRightWand.translate = {};
        outWeaponInRightWand.scale = 1.0f;
        return isFiniteTransform(outWeaponInRightWand);
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
                                     RE::NiTransform& outBoneInWand,
                                     bool& outWandValid) {
            if (hasVisualAuthorityForHand(isLeft)) {
                return;
            }

            RE::NiTransform handWorld{};
            if (!frik_hand_world_authority::hasCalibratedRawHandFrame(isLeft) ||
                !frik_hand_world_authority::tryGetRawHandWorld(isLeft, handWorld)) {
                return;
            }

            const auto captureRelation = [&](const RE::NiTransform* sourceWorld,
                                             RE::NiTransform& outRelation,
                                             bool& outValid) {
                if (!sourceWorld || !isFiniteTransform(*sourceWorld)) {
                    return;
                }
                const RE::NiTransform relation =
                    transform_math::composeTransforms(
                        transform_math::invertTransform(*sourceWorld),
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

            captureRelation(wandNode ? &wandNode->world : nullptr, outBoneInWand, outWandValid);
        };

        refreshHand(
            false,
            playerNodes->primaryWandNode,
            _firing.rightNaturalBoneInWand,
            _firing.hasRightNaturalBoneInWand);
        refreshHand(
            true,
            playerNodes->SecondaryWandNode,
            _firing.leftNaturalBoneInWand,
            _firing.hasLeftNaturalBoneInWand);
    }

    bool TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal(
        RE::NiTransform& outHandWeaponLocal,
        bool* outUsedAuthoredCanonical,
        const bool logDiagnostic) const
    {
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = false;
        }
        RE::NiTransform rightWeaponAimInWand{};
        if (!hasRightFiringHandCanonicalFrame(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey) ||
            !tryResolveRightWeaponAimInWand(
                _session.weaponNode,
                _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey,
                rightWeaponAimInWand)) {
            return false;
        }

        if(_holdingHasLeft){outHandWeaponLocal=_holdingLeft;if(outUsedAuthoredCanonical)*outUsedAuthoredCanonical=true;return true;}
        const bool mirrored = weapon_grip_seat::leftFiringSeat(
            _holdingHasNative ? _holdingNativeRight : _firing.rightCanonicalHandWeaponLocal,
            _holdingMirrorPlaneX, outHandWeaponLocal);
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

    bool TwoHandedGrip::tryBuildMirroredSupportHandWeaponLocal(
        const RE::NiTransform& leftHandWeaponLocal,
        RE::NiTransform& outRightHandWeaponLocal, float mirrorPlaneX)
    {
        outRightHandWeaponLocal = {};
        if (!isFiniteTransform(leftHandWeaponLocal)) return false;
        outRightHandWeaponLocal = holding_grip_math::toNative(
            holding_grip_math::mirroredHandSeat(holding_grip_math::fromNative(leftHandWeaponLocal),mirrorPlaneX));
        return isFiniteTransform(outRightHandWeaponLocal);
    }

    weapon_support_authority_policy::FiringGripPromotionResult TwoHandedGrip::tryPromoteSupportGripToFiringGrip(
        RE::NiNode* weaponNode, const float dt, const char*& outReason)
    {
        using Result = weapon_support_authority_policy::FiringGripPromotionResult;
        const bool supportHandIsLeft = isSupportHandLeft();
        if (!_handlingSettings.ambidextrousHandoffEnabled) {
            outReason = "handoff-disabled";
            return Result::NotApplicable;
        }

        const WeaponPartGrip& supportGrip = partGrip(supportHandIsLeft);
        // Core support may promote only from the acquired firing station.
        // Explicit provider carry grips retain their cylinder-tested contract;
        // provider AttachOnly can never inherit firing authority.
        const bool coreFiringStation = supportGrip.acquisitionSource == WeaponInteractionAcquisitionSource::FiringGripZone;
        if (!supportGrip.active || supportGrip.attachOnly ||
            (!coreFiringStation && !supportGrip.providerPartAuthority.active)) {
            outReason = "support-grip-not-promotable";
            return Result::NotApplicable;
        }

        if (!weaponNode) {
            outReason = "weapon-node-unavailable";
            return Result::Blocked;
        }
        // Releasing a forward support hold leaves that hand carrying the
        // weapon; takeover applies only when it actually occupies the firing
        // station. Use the captured grip point, not the offset palm of a cup.
        firing_grip_reattach_zone_policy::ZoneInput promotionInput{};
        const float reach = _handlingSettings.firingGripReattachRadiusGameUnits;
        if (!tryBuildFiringGripZoneInput(weaponNode, _session.weaponGenerationKey,
                _session.equippedWeaponOwnershipKey, reach, promotionInput)) {
            outReason = "firing-zone-unavailable";
            return Result::Blocked;
        }
        // The shared station is latched at acquisition. A pistol's authored
        // cup is deliberately offset from that station and must not redefine it.
        const auto supportGripWorld = coreFiringStation ?
            weaponLocalToWorld(_firing.rightCanonicalGripWeaponLocal, weaponNode) : resolvePartGripWorld(supportGrip, weaponNode);
        promotionInput.palmWorld = {supportGripWorld.x, supportGripWorld.y, supportGripWorld.z};
        if (!weapon_support_authority_policy::canPromoteSupportGripToFiringGrip(supportGrip.active,
                supportGrip.attachOnly, firing_grip_reattach_zone_policy::evaluateZone(promotionInput).inside)) {
            outReason = "support-outside-firing-zone";
            return Result::NotApplicable;
        }
        if (!canBeginPrimaryOnlyGripForHand(supportHandIsLeft)) {
            outReason = "receiving-hand-unavailable";
            return Result::Blocked;
        }
        if (!hasRightFiringHandCanonicalFrame(weaponNode,
                _session.weaponGenerationKey, _session.equippedWeaponOwnershipKey) ||
            !isUsableHandAuthorityTransform(_firing.rightCanonicalHandWeaponLocal)) {
            outReason = "canonical-frame-not-current";
            return Result::Blocked;
        }
        RE::NiTransform handTransform{};
        if (!tryGetSolverHandTransform(supportHandIsLeft, handTransform)) {
            outReason = "receiving-hand-frame-unavailable";
            return Result::Blocked;
        }

        /*
         * Prepare: the support hand takes over the SAME weapon-relative firing
         * grip in place, forcing the CANONICAL per-hand hold: a LEFT takeover
         * applies the canonical right-hand hold mirrored (authored offsets
         * adapted to the left bone basis), a RIGHT takeover re-takes its
         * canonical native hold directly - the promoted hand's live bone is
         * still part-grip-locked here, so a live capture froze that locked
         * angle and (for the right) poisoned the canonical snapshot below.
         * The identity checks above validate the current canonical.
         */
        RE::NiTransform newFiringHandWeaponLocal{};
        RE::NiPoint3 newFiringGripWeaponLocal = _firing.rightCanonicalGripWeaponLocal;
        const char* holdSource = nullptr;
        if (supportHandIsLeft) {
            const char* captureFailure = nullptr;
            if (!tryBuildCurrentLeftFiringGripCapture(weaponNode, _session.weaponGenerationKey,
                    _session.equippedWeaponOwnershipKey, newFiringHandWeaponLocal, newFiringGripWeaponLocal, &captureFailure)) {
                outReason = captureFailure ? captureFailure : "firing-mirror-unavailable";
                return Result::Blocked;
            }
            holdSource = _firing.rightCanonicalSource == RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-mirror" : "native-mirror";
        } else {
            newFiringHandWeaponLocal = _firing.rightCanonicalHandWeaponLocal;
            holdSource = _firing.rightCanonicalSource == RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" : "native-canonical";
        }
        // Commit only after the receiving pose is ready. A failed preparation
        // leaves both grips and native pose ownership intact.
        if (supportHandIsLeft && !blockFrikPrimaryWeaponPose()) {
            outReason = "native-pose-block-failed";
            return Result::Blocked;
        }
        beginHandVisualReturn(isFiringHandLeft(), "ambidextrous-firing-hand-promotion");
        if (_handlingSettings.detachAuthority ==
            immersive_weapon_policy::DetachAuthority::
                IntegratedImmersive) {
            recordFiringGripDetachedHaptic();
        }
        setFiringHand(supportHandIsLeft, "support-grip-promotion");
        if (ownsWeaponTransform()) {
            /*
             * The promoted hand leaves the two-hand solve for one-hand carry.
             * Ease the rendered two-hand pose into that carry the same way a
             * support release does: the native right baseline return, or the
             * left carry's wand-aimed return.
             */
            if (usesNativeRightCarry()) {
                beginWeaponVisualReturn("support-grip-promotion");
            } else {
                beginLeftFiringSupportReleaseReturn("support-grip-promotion");
            }
        }
        if (!transitionToPrimaryOnly(weaponNode, _session.weaponGenerationKey, _session.equippedWeaponOwnershipKey, "firing-grip-hand-promotion")) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: firing-grip promotion failed to enter primary-only; clearing authority");
            transitionToInactive(false);
            return Result::Promoted;
        }

        _firing.primaryHandWeaponLocal = newFiringHandWeaponLocal;
        _firing.primaryGripLocal = newFiringGripWeaponLocal;
        _firing.hasPrimaryHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame(_firing.rightCanonicalInstanceContentKey);
        if (usesLeftFiringCarry() &&
            !solveLeftFiringWeaponCarry(weaponNode, dt)) {
            return Result::Promoted;
        }
        _session.firingGripSequence = ++_session.gripCaptureSequence;
        _visuals.primaryHandLerp = {};
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = isFiringHandLeft();
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: support hand promoted to firing grip hand={} reason=primary-release hold={}",
            firingHandName(), holdSource);
        return Result::Promoted;
    }
}

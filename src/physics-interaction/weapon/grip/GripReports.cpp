#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Outward grip reporting: occupancy, HandGripReport, haptic events, drop requests, and authored grip pose snapshots.

namespace rock
{
    void TwoHandedGrip::observeEquippedOwnership(std::uint64_t ownershipKey, std::uint64_t gripGenerationKey)
    {
        // A session can also have been started since the previous observation.
        // Validate that session against the item even when the observed key
        // stayed zero; no active session can substitute for inventory identity.
        if (_session.state != TwoHandedState::Inactive &&
            (isManualOwnershipActive() || _confirmedEquippedOwnershipKey != ownershipKey) &&
            _session.equippedWeaponOwnershipKey != ownershipKey) {
            transitionToInactive(false);
        }
        if (_confirmedEquippedOwnershipKey != ownershipKey) {
            ROCK_LOG_INFO(Weapon, "Equipped grip identity: owner={:016X}->{:016X} firingHand={} grabSession={}",
                _confirmedEquippedOwnershipKey, ownershipKey, firingHandName(), isManualOwnershipActive());
        }
        _confirmedEquippedOwnershipKey = ownershipKey;
        _confirmedEquippedGripGenerationKey = ownershipKey ? gripGenerationKey : 0;
    }

    EquippedWeaponGripOccupancy TwoHandedGrip::getGrabInputOccupancy() const noexcept
    {
        return equipped_weapon_toggle_grab_policy::forGrabInput(getGripOccupancy(),
            isManualOwnershipActive() && !isPersistentEquippedCarryInputAcquisitionPending());
    }

    EquippedWeaponGripOccupancy TwoHandedGrip::getGripOccupancy()
        const noexcept
    {
        const bool firingGripActive = isFiringGripOccupied();
        return EquippedWeaponGripOccupancy{
            .left = {
                .firingGripActive =
                    firingGripActive && isFiringHandLeft(),
                .partGripActive = partGrip(true).active,
                .partGripAttachOnly = partGrip(true).attachOnly,
            },
            .right = {
                .firingGripActive =
                    firingGripActive && !isFiringHandLeft(),
                .partGripActive = partGrip(false).active,
                .partGripAttachOnly = partGrip(false).attachOnly,
            },
        };
    }

    EquippedWeaponManualDropRequest TwoHandedGrip::consumeEquippedWeaponDropRequest()
    {
        const EquippedWeaponManualDropRequest request = _equippedWeaponDropRequest;
        _equippedWeaponDropRequest = {};
        return request;
    }

    void TwoHandedGrip::getHandGripReport(bool isLeft, HandGripReport& outReport) const
    {
        outReport = {};
        const bool handHasFiringRole = isFiringHand(isLeft);
        if (handHasFiringRole && isFiringGripOccupied() && !isManualOwnershipActive()) {
            // Logical ownership does not invent a captured pose/source node.
            outReport.kind = weapon_part_grip_report_policy::HandGripKind::FiringGrip;
            outReport.active = true;
            outReport.weaponGenerationKey = _confirmedEquippedGripGenerationKey;
            return;
        }
        const WeaponPartGrip& grip = partGrip(isLeft);
        const auto kind = weapon_part_grip_report_policy::resolveHandGripKind(
            _session.state == TwoHandedState::Gripping,
            _session.state == TwoHandedState::PartCarry,
            _session.state == TwoHandedState::PrimaryOnly,
            handHasFiringRole,
            grip.active,
            grip.attachOnly,
            _session.authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport);
        outReport.kind = kind;
        if (kind == weapon_part_grip_report_policy::HandGripKind::None) {
            return;
        }

        outReport.active = true;
        if (kind == weapon_part_grip_report_policy::HandGripKind::FiringGrip) {
            // In PrimaryOnly the weapon rides the FRIK-native hand attach and
            // ROCK holds no captured hand-to-weapon frame; hasHandPartLocal
            // stays false there by design.
            outReport.gripSequence = _session.firingGripSequence;
            outReport.weaponGenerationKey = _session.weaponGenerationKey;
            outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(_session.weaponNode);
            outReport.hasHandPartLocal = _firing.hasPrimaryHandWeaponLocal;
            outReport.handPartLocal = _firing.primaryHandWeaponLocal;
            return;
        }

        outReport.attachOnly = grip.attachOnly;
        outReport.authoredSupportGrip = grip.authoredSupportGrip;
        outReport.gripSequence = grip.gripSequence;
        outReport.weaponGenerationKey = grip.weaponGenerationKey != 0 ? grip.weaponGenerationKey : _session.weaponGenerationKey;
        outReport.bodyId = grip.contactBodyId;
        outReport.partKind = static_cast<std::uint32_t>(grip.partKind);
        outReport.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
        outReport.supportRole = static_cast<std::uint32_t>(grip.supportRole);
        outReport.socketRole = static_cast<std::uint32_t>(grip.socketRole);
        outReport.actionRole = static_cast<std::uint32_t>(grip.actionRole);
        outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(grip.attachmentRoot);
        if (grip.providerPartAuthority.active) {
            outReport.providerOwnerToken = grip.providerPartAuthority.ownerToken;
            outReport.providerGroupId = grip.providerPartAuthority.groupId;
            outReport.providerGrabMode = grip.providerPartAuthority.grabMode;
        }
        outReport.hasHandPartLocal = grip.hasSourceFrames || grip.hasHandWeaponLocal;
        outReport.handPartLocalIsSourceLocal = grip.hasSourceFrames;
        outReport.handPartLocal = grip.hasSourceFrames ? grip.handSourceLocal : grip.handWeaponLocal;
        outReport.sourceName = grip.sourceName;
        outReport.omodFormId = grip.omodFormId;
        outReport.attachPointFormId = grip.attachPointFormId;
        outReport.classificationSource = static_cast<std::uint32_t>(grip.classificationSource);
    }

    TwoHandedGripHapticEvents TwoHandedGrip::consumeHapticEvents()
    {
        const TwoHandedGripHapticEvents events = _hapticEvents;
        _hapticEvents = {};
        return events;
    }

    void TwoHandedGrip::recordFiringGripDetachedHaptic() noexcept
    {
        if (_hapticEvents.firingGripDetached) {
            return;
        }
        _hapticEvents.firingGripDetached = true;
        _hapticEvents.firingGripDetachedHandIsLeft = isFiringHandLeft();
    }

    void TwoHandedGrip::recordGripReleaseRetained(const bool isLeft, const char* reason)
    {
        (isLeft ? _gripReleaseRetained.left : _gripReleaseRetained.right) = true;
        if (partGrip(isLeft).active) partGrip(isLeft).releaseRequiresNewHold = true;
        bool& logged = _gripReleaseRetainedLogged[isLeft ? 1u : 0u];
        if (logged) {
            return;
        }
        logged = true;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: open-hand release refused hand={} reason={} generation={:016X} detachSource={} -- current grip retained",
            isLeft ? "left" : "right",
            reason ? reason : "unknown",
            _session.weaponGenerationKey,
            immersive_weapon_policy::authorityName(
                _handlingSettings.detachAuthority));
    }

    bool TwoHandedGrip::captureDropGripPose(bool isLeft, AuthoredWeaponGripPose& out) const
    {
        out = {};
        if (!_session.weaponNode || !_session.weaponGenerationKey ||
            _session.equippedWeaponOwnershipKey != _confirmedEquippedOwnershipKey) return false;
        out.isLeft = isLeft;
        out.weaponFormId = _recoil.weaponEvidence.formID;
        const auto& part = partGrip(isLeft);
        const bool support = _session.state == TwoHandedState::PartCarry && part.active &&
            part.authoredRole != loose_weapon_authored_grab_policy::Role::Firing;
        if (support) {
            out.role = loose_weapon_authored_grab_policy::Role::Support;
            if (part.authoredSupportGrip && part.hasHandWeaponLocal && part.fingerLocalTransformMask == 0x7FFFu) {
                out.handWeaponLocal = part.handWeaponLocal;
                out.fingerLocals = part.fingerLocalTransforms;
                out.fingerMask = part.fingerLocalTransformMask;
            } else if (!tryResolveAuthoredSupportGripCandidateForHand(isLeft, _session.weaponNode,
                    _session.weaponGenerationKey, out.handWeaponLocal, out.fingerLocals, out.fingerMask)) return false;
        } else {
            out.role = loose_weapon_authored_grab_policy::Role::Firing;
            const char* source = nullptr;
            if (!tryResolveAuthoredFiringHandCanonicalForProbe(isLeft, out.handWeaponLocal, source)) return false;
            out.fingerLocals = isLeft ? _firing.leftFingerLocalTransforms : _firing.rightFingerLocalTransforms;
            out.fingerMask = isLeft ? _firing.leftFingerLocalTransformMask : _firing.rightFingerLocalTransformMask;
        }
        RE::NiTransform physicalHandWorld{}, physicalDriverWorld{};
        if (!tryResolvePhysicalHandFrame(isLeft, physicalHandWorld, physicalDriverWorld)) return false;
        // Preserve the controller-to-weapon placement on drop, independently of
        // the authored wrist/fingers presented by the same-frame provider.
        out.placementHandWeaponLocal = transform_math::composeTransforms(
            transform_math::invertTransform(_session.weaponNode->world), physicalHandWorld);
        return out.valid();
    }

    bool TwoHandedGrip::requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (_equippedWeaponDropRequest.requested) {
            return true;
        }

        const auto occupied = getGripOccupancy();
        AuthoredWeaponGripPose pose{};
        const bool isLeft = equipped_weapon_drop_policy::isLeft(sourceHand);
        if (sourceHand == equipped_weapon_drop_policy::SourceHand::None ||
            !equipped_weapon_drop_policy::canStartAutoDrop(occupied.left.carriesWeapon(),
                occupied.right.carriesWeapon(), _firing.reattachHoverInsideZone) ||
            !captureDropGripPose(isLeft, pose)) {
            recordGripReleaseRetained(isLeft, "auto-drop-pose-occupancy-or-zone-unavailable");
            _firing.primaryReleaseIntent.pending = false;
            _firing.primaryReleaseDebounce = {};
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000,
                "Auto drop retained: hand={} bothCarriers={} authoredPose={} freeFiringStationHovered={} generation={:016X}",
                isLeft ? "left" : "right", occupied.left.carriesWeapon() && occupied.right.carriesWeapon(),
                pose.valid(), _firing.reattachHoverInsideZone, _session.weaponGenerationKey);
            return false;
        }

        _equippedWeaponDropRequest = EquippedWeaponManualDropRequest{
            .requested = true,
            .sourceHand = sourceHand,
            .pose = pose,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: equipped weapon drop requested reason={} sourceHand={} generation={:016X} detachSource={}",
            reason ? reason : "unknown",
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            _session.weaponGenerationKey,
            immersive_weapon_policy::authorityName(
                _handlingSettings.detachAuthority));
        return true;
    }

    void TwoHandedGrip::prepareEquippedWeaponDropCommit()
    {
        // Restore native parenting before RemoveItem can replace the equipped
        // scene. Keep the captured grip intact until the native result is known.
        releaseFiringHandWeaponNodeOwnership(_session.weaponNode);
    }

    void TwoHandedGrip::completeEquippedWeaponDrop(const EquippedWeaponManualDropRequest& request, bool committed)
    {
        if (!committed) {
            recordGripReleaseRetained(equipped_weapon_drop_policy::isLeft(request.sourceHand), "auto-drop-transfer-unavailable");
            _firing.primaryReleaseIntent.pending = false;
            _firing.primaryReleaseDebounce = {};
            return;
        }
        recordFiringGripDetachedHaptic();
        clearWeaponVisualReturn("equipped-weapon-drop", true, true);
        transitionToInactive(false);
    }

    bool TwoHandedGrip::getSelectedAuthoredGripPoseSnapshot(
        SelectedAuthoredGripPoseSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        const auto generationKey = _session.weaponGenerationKey != 0 ?
            _session.weaponGenerationKey :
            _firing.rightCanonicalGenerationKey;
        if (generationKey == 0) {
            return false;
        }

        const bool canonicalCurrent =
            _firing.hasRightCanonicalHandWeaponLocal &&
            _firing.rightCanonicalGenerationKey == generationKey &&
            (!_session.weaponNode ||
                _firing.rightCanonicalWeaponNode == _session.weaponNode);
        const bool supportCurrent =
            _support.authoredCandidate.valid &&
            _support.authoredCandidate.weaponGenerationKey == generationKey &&
            (!_session.weaponNode ||
                _support.authoredCandidate.weaponNode == _session.weaponNode);
        if (!canonicalCurrent && !supportCurrent) {
            return false;
        }

        outSnapshot.weaponGenerationKey = generationKey;
        if (canonicalCurrent) {
            outSnapshot.rightHandWeaponLocal =
                _firing.rightCanonicalHandWeaponLocal;
            outSnapshot.rightHandValid = true;
            outSnapshot.rightFingerLocalTransforms =
                _firing.rightFingerLocalTransforms;
            outSnapshot.rightFingerLocalTransformMask =
                _firing.rightFingerLocalTransformMask;
            outSnapshot.captureSequence =
                _firing.rightCanonicalCaptureSequence;
            outSnapshot.source =
                _firing.rightCanonicalSource ==
                        RightFiringCanonicalSource::AuthoredAnimation ?
                    SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest :
                    SelectedAuthoredGripPoseSnapshot::Source::RuntimeCanonical;
        }

        if (supportCurrent) {
            outSnapshot.leftHandWeaponLocal =
                _support.authoredCandidate.leftHandWeaponLocal;
            outSnapshot.leftHandValid = true;
            outSnapshot.leftFingerLocalTransforms =
                _support.authoredCandidate.leftFingerLocalTransforms;
            outSnapshot.leftFingerLocalTransformMask =
                _support.authoredCandidate.leftFingerLocalTransformMask;
            outSnapshot.captureSequence = (std::max)(
                outSnapshot.captureSequence,
                _support.authoredCandidate.captureSequence);
            if (!canonicalCurrent) {
                outSnapshot.rightHandWeaponLocal =
                    _support.authoredCandidate.rightHandWeaponLocal;
                outSnapshot.rightHandValid =
                    _support.authoredCandidate.rightMirrorValid;
                outSnapshot.rightFingerLocalTransforms =
                    _support.authoredCandidate.rightFingerLocalTransforms;
                outSnapshot.rightFingerLocalTransformMask =
                    _support.authoredCandidate.rightFingerLocalTransformMask;
            }
            outSnapshot.source =
                SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest;
        } else if (canonicalCurrent) {
            RE::NiTransform leftHandWeaponLocal{};
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    leftHandWeaponLocal,
                    nullptr,
                    false)) {
                outSnapshot.leftHandWeaponLocal = leftHandWeaponLocal;
                outSnapshot.leftHandValid = true;
                outSnapshot.leftFingerLocalTransforms =
                    _firing.leftFingerLocalTransforms;
                outSnapshot.leftFingerLocalTransformMask =
                    _firing.leftFingerLocalTransformMask;
            }
        }

        outSnapshot.variantKey = outSnapshot.captureSequence != 0 ?
            outSnapshot.captureSequence :
            generationKey;
        outSnapshot.valid =
            outSnapshot.rightHandValid || outSnapshot.leftHandValid;
        return outSnapshot.valid;
    }
}

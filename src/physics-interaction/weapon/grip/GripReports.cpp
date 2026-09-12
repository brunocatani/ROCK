#include "physics-interaction/weapon/TwoHandedGripInternal.h"

// Outward grip reporting: occupancy, HandGripReport, haptic events, drop requests, and authored grip pose snapshots.

namespace rock
{
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
        bool& logged = _gripReleaseRetainedLogged[isLeft ? 1u : 0u];
        if (logged) {
            return;
        }
        logged = true;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: open-hand release refused hand={} reason={} generation={:016X} detachSource={} -- last carrier keeps the weapon because the last-grip drop is disabled",
            isLeft ? "left" : "right",
            reason ? reason : "unknown",
            _session.weaponGenerationKey,
            immersive_weapon_policy::authorityName(
                _handlingSettings.detachAuthority));
    }

    void TwoHandedGrip::requestEquippedWeaponDrop(const char* reason, equipped_weapon_drop_policy::SourceHand sourceHand)
    {
        if (_equippedWeaponDropRequest.requested) {
            transitionToInactive(false);
            return;
        }

        _equippedWeaponDropRequest = EquippedWeaponManualDropRequest{
            .requested = true,
            .sourceHand = sourceHand,
        };
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: equipped weapon drop requested reason={} sourceHand={} generation={:016X} detachSource={}",
            reason ? reason : "unknown",
            equipped_weapon_drop_policy::sourceHandName(sourceHand),
            _session.weaponGenerationKey,
            immersive_weapon_policy::authorityName(
                _handlingSettings.detachAuthority));
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

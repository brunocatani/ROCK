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
                    firingGripActive && _firingHandIsLeft,
                .partGripActive = partGrip(true).active,
            },
            .right = {
                .firingGripActive =
                    firingGripActive && !_firingHandIsLeft,
                .partGripActive = partGrip(false).active,
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
        const bool isFiringHand = isLeft == _firingHandIsLeft;
        const WeaponPartGrip& grip = partGrip(isLeft);
        const auto kind = weapon_part_grip_report_policy::resolveHandGripKind(
            _state == TwoHandedState::Gripping,
            _state == TwoHandedState::PartCarry,
            _state == TwoHandedState::PrimaryOnly,
            isFiringHand,
            grip.active,
            grip.attachOnly,
            _authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport);
        outReport.kind = kind;
        if (kind == weapon_part_grip_report_policy::HandGripKind::None) {
            return;
        }

        outReport.active = true;
        if (kind == weapon_part_grip_report_policy::HandGripKind::FiringGrip) {
            // In PrimaryOnly the weapon rides the FRIK-native hand attach and
            // ROCK holds no captured hand-to-weapon frame; hasHandPartLocal
            // stays false there by design.
            outReport.gripSequence = _firingGripSequence;
            outReport.weaponGenerationKey = _activeWeaponGenerationKey;
            outReport.sourceRoot = reinterpret_cast<std::uintptr_t>(_activeWeaponNode);
            outReport.hasHandPartLocal = _hasFiringHandWeaponLocal;
            outReport.handPartLocal = _primaryHandWeaponLocal;
            return;
        }

        outReport.attachOnly = grip.attachOnly;
        outReport.authoredSupportGrip = grip.authoredSupportGrip;
        outReport.gripSequence = grip.gripSequence;
        outReport.weaponGenerationKey = grip.weaponGenerationKey != 0 ? grip.weaponGenerationKey : _activeWeaponGenerationKey;
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
        _hapticEvents.firingGripDetachedHandIsLeft = _firingHandIsLeft;
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
            _activeWeaponGenerationKey,
            immersive_weapon_policy::authorityName(
                _handlingSettings.detachAuthority));
        clearWeaponVisualReturn("equipped-weapon-drop", true, true);
        transitionToInactive(false);
    }

    bool TwoHandedGrip::getSelectedAuthoredGripPoseSnapshot(
        SelectedAuthoredGripPoseSnapshot& outSnapshot) const
    {
        outSnapshot = {};
        const auto generationKey = _activeWeaponGenerationKey != 0 ?
            _activeWeaponGenerationKey :
            _rightFiringHandCanonicalGenerationKey;
        if (generationKey == 0) {
            return false;
        }

        const bool canonicalCurrent =
            _hasRightFiringHandCanonicalWeaponLocal &&
            _rightFiringHandCanonicalGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _rightFiringHandCanonicalWeaponNode == _activeWeaponNode);
        const bool supportCurrent =
            _authoredSupportGripCandidate.valid &&
            _authoredSupportGripCandidate.weaponGenerationKey == generationKey &&
            (!_activeWeaponNode ||
                _authoredSupportGripCandidate.weaponNode == _activeWeaponNode);
        if (!canonicalCurrent && !supportCurrent) {
            return false;
        }

        outSnapshot.weaponGenerationKey = generationKey;
        if (canonicalCurrent) {
            outSnapshot.rightHandWeaponLocal =
                _rightFiringHandCanonicalWeaponLocal;
            outSnapshot.rightHandValid = true;
            outSnapshot.rightFingerLocalTransforms =
                _rightFiringFingerLocalTransforms;
            outSnapshot.rightFingerLocalTransformMask =
                _rightFiringFingerLocalTransformMask;
            outSnapshot.captureSequence =
                _rightFiringHandCanonicalCaptureSequence;
            outSnapshot.source =
                _rightFiringHandCanonicalSource ==
                        RightFiringCanonicalSource::AuthoredAnimation ?
                    SelectedAuthoredGripPoseSnapshot::Source::NativeIdlePreharvest :
                    SelectedAuthoredGripPoseSnapshot::Source::RuntimeCanonical;
        }

        if (supportCurrent) {
            outSnapshot.leftHandWeaponLocal =
                _authoredSupportGripCandidate.leftHandWeaponLocal;
            outSnapshot.leftHandValid = true;
            outSnapshot.leftFingerLocalTransforms =
                _authoredSupportGripCandidate.leftFingerLocalTransforms;
            outSnapshot.leftFingerLocalTransformMask =
                _authoredSupportGripCandidate.leftFingerLocalTransformMask;
            outSnapshot.captureSequence = (std::max)(
                outSnapshot.captureSequence,
                _authoredSupportGripCandidate.captureSequence);
            if (!canonicalCurrent) {
                outSnapshot.rightHandWeaponLocal =
                    _authoredSupportGripCandidate.rightHandWeaponLocal;
                outSnapshot.rightHandValid =
                    _authoredSupportGripCandidate.rightMirrorValid;
                outSnapshot.rightFingerLocalTransforms =
                    _authoredSupportGripCandidate.rightFingerLocalTransforms;
                outSnapshot.rightFingerLocalTransformMask =
                    _authoredSupportGripCandidate.rightFingerLocalTransformMask;
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
                    _leftFiringFingerLocalTransforms;
                outSnapshot.leftFingerLocalTransformMask =
                    _leftFiringFingerLocalTransformMask;
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

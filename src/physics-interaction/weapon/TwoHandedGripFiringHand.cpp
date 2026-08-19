#include "physics-interaction/weapon/TwoHandedGrip.h"
#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/EquippedWeaponHandlingRuntime.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "rock_support/Fo4VrRuntime.h"
#include "RockConfig.h"
#include "RockUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace rock
{
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::leftFiringInfrastructureAvailable;
    using two_handed_grip_detail::tryResolveWeaponRootLocal;
    using two_handed_grip_detail::WEAPON_NODE_OWNERSHIP_TAG;

    namespace
    {
        struct MirroredDriverConjugation
        {
            RE::NiTransform sourceWeaponInDriver{};
            RE::NiTransform targetWeaponInDriver{};
            RE::NiTransform targetHandWeaponLocal{};
        };

        [[nodiscard]] bool tryBuildMirroredDriverConjugation(
            const RE::NiTransform& sourceHandWeaponLocal,
            const RE::NiTransform& sourceBoneInDriver,
            const RE::NiTransform& targetBoneInDriver,
            const RE::NiTransform* targetDriverTrim,
            MirroredDriverConjugation& out)
        {
            out = {};
            RE::NiTransform lateralMirror{};
            lateralMirror.MakeIdentity();
            lateralMirror.rotate.entry[0][0] = -1.0f;

            out.sourceWeaponInDriver = transform_math::composeTransforms(
                sourceBoneInDriver,
                transform_math::invertTransform(
                    sourceHandWeaponLocal));
            out.targetWeaponInDriver = transform_math::composeTransforms(
                lateralMirror,
                transform_math::composeTransforms(
                    out.sourceWeaponInDriver,
                    lateralMirror));
            if (targetDriverTrim) {
                out.targetWeaponInDriver =
                    transform_math::composeTransforms(
                        *targetDriverTrim,
                        out.targetWeaponInDriver);
            }

            const RE::NiTransform weaponInTargetHand =
                transform_math::composeTransforms(
                    transform_math::invertTransform(targetBoneInDriver),
                    out.targetWeaponInDriver);
            out.targetHandWeaponLocal =
                transform_math::invertTransform(weaponInTargetHand);
            return isFiniteTransform(out.targetHandWeaponLocal);
        }

    }

    // ---- Firing-hand entry and persistent carry ----

    bool TwoHandedGrip::canBeginPrimaryOnlyGripForHand(const bool isLeft)
    {
        return !isLeft || leftFiringInfrastructureAvailable();
    }

    bool TwoHandedGrip::tryBuildCurrentLeftFiringGripCapture(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        RE::NiTransform& outFiringHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal) const
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

        RE::NiTransform rightHandWorld{};
        RE::NiTransform leftHandWorld{};
        if (!tryGetSolverHandTransform(false, rightHandWorld) ||
            !tryGetSolverHandTransform(true, leftHandWorld) ||
            !tryBuildMirroredLeftFiringHandWeaponLocal(
                _rightFiringHandCanonicalWeaponLocal,
                _rightFiringGripCanonicalWeaponLocal,
                rightHandWorld,
                leftHandWorld,
                outFiringHandWeaponLocal,
                true)) {
            return false;
        }

        outFiringGripWeaponLocal = _rightFiringGripCanonicalWeaponLocal;
        return true;
    }

    bool TwoHandedGrip::tryCaptureLeftFiringGripTransfer(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey,
        RE::NiTransform& outFiringHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal) const
    {
        outFiringHandWeaponLocal = {};
        outFiringGripWeaponLocal = {};

        const bool activeLeftCaptureCurrent =
            isManualOwnershipActive() &&
            _firingHandIsLeft &&
            _activeWeaponNode == weaponNode &&
            currentEquippedWeaponOwnershipKey != 0 &&
            _activeEquippedWeaponOwnershipKey ==
                currentEquippedWeaponOwnershipKey &&
            _hasFiringHandWeaponLocal &&
            isFiniteTransform(_primaryHandWeaponLocal) &&
            _primaryGripConfidence > 0.0f &&
            std::isfinite(_primaryGripLocal.x) &&
            std::isfinite(_primaryGripLocal.y) &&
            std::isfinite(_primaryGripLocal.z);
        if (activeLeftCaptureCurrent) {
            outFiringHandWeaponLocal = _primaryHandWeaponLocal;
            outFiringGripWeaponLocal = _primaryGripLocal;
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
        const bool retainUntilPhysicalGrip)
    {
        if (!weaponNode || currentEquippedWeaponOwnershipKey == 0 || _state != TwoHandedState::Inactive ||
            !canBeginPrimaryOnlyGripForHand(firingHandIsLeft)) {
            return false;
        }
        if (firingHandIsLeft &&
            (!capturedFiringHandWeaponLocal || !isFiniteTransform(*capturedFiringHandWeaponLocal) ||
                !capturedFiringGripWeaponLocal ||
                !std::isfinite(capturedFiringGripWeaponLocal->x) ||
                !std::isfinite(capturedFiringGripWeaponLocal->y) ||
                !std::isfinite(capturedFiringGripWeaponLocal->z))) {
            return false;
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
            _primaryHandWeaponLocal = *capturedFiringHandWeaponLocal;
            _hasFiringHandWeaponLocal = true;
        }

        if (!transitionToPrimaryOnly(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                "primary-grip-start")) {
            _primaryHandWeaponLocal = {};
            _hasFiringHandWeaponLocal = false;
            setFiringHand(false, "primary-grip-start-failed");
            restoreFrikPrimaryWeaponPose();
            return false;
        }
        if (firingHandIsLeft) {
            // The newly equipped node inherits the exact loose-model firing
            // grip; transitionToPrimaryOnly must not recapture it from the
            // left palm against FRIK's still-right-native first frame.
            _primaryGripLocal = *capturedFiringGripWeaponLocal;
            _primaryGripConfidence = 1.0f;
        }
        // Only a fresh grab pulses; transitionToPrimaryOnly is also reached
        // from support-release paths where the firing grip never changed.
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        _firingGripSequence = ++_gripCaptureSequence;
        if (retainUntilPhysicalGrip) {
            _persistentEquippedCarryActive = true;
            _persistentEquippedCarryDetachArmed = false;
        }
        return true;
    }

    bool TwoHandedGrip::beginPersistentEquippedCarry(
        RE::NiNode* weaponNode,
        const std::uint64_t currentWeaponGenerationKey,
        const std::uint64_t currentEquippedWeaponOwnershipKey)
    {
        if (!weaponNode || currentWeaponGenerationKey == 0 || currentEquippedWeaponOwnershipKey == 0 ||
            _state != TwoHandedState::Inactive || !canBeginPrimaryOnlyGripForHand(true) ||
            !hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey)) {
            return false;
        }

        RE::NiTransform mirroredLeftHold{};
        RE::NiPoint3 firingGripWeaponLocal{};
        if (!tryBuildCurrentLeftFiringGripCapture(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                mirroredLeftHold,
                firingGripWeaponLocal)) {
            return false;
        }

        if (!beginPrimaryOnlyGrip(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                true,
                &mirroredLeftHold,
                &firingGripWeaponLocal)) {
            return false;
        }

        _persistentEquippedCarryActive = true;
        _persistentEquippedCarryDetachArmed = false;
        const bool usedAuthoredCanonical =
            _rightFiringHandCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: persistent left-hand carry active generation={:016X} ownership={:016X} source={} capture={}",
            currentWeaponGenerationKey,
            currentEquippedWeaponOwnershipKey,
            usedAuthoredCanonical ? "authored-animation" : "native-carry",
            _rightFiringHandCanonicalCaptureSequence);
        return true;
    }

    void TwoHandedGrip::clearPersistentEquippedCarry(const char* reason)
    {
        if (!_persistentEquippedCarryActive) {
            return;
        }
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: clearing persistent left-hand carry reason={}",
            reason ? reason : "unknown");
        _persistentEquippedCarryActive = false;
        _persistentEquippedCarryDetachArmed = false;
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

    bool TwoHandedGrip::publishLeftFiringFeedForwardWeaponPose(RE::NiNode* weaponNode)
    {
        if (!weaponNode || weaponNode != _activeWeaponNode ||
            (_state != TwoHandedState::Gripping && _state != TwoHandedState::PrimaryOnly) ||
            !_firingHandIsLeft || !_hasFiringHandWeaponLocal) {
            return false;
        }

        RE::NiTransform leftFiringHandTransform{};
        if (!tryGetSolverHandTransform(true, leftFiringHandTransform)) {
            return false;
        }

        const RE::NiTransform feedForwardWeaponWorld = transform_math::composeTransforms(
            leftFiringHandTransform, transform_math::invertTransform(_primaryHandWeaponLocal));
        if (!isFiniteTransform(feedForwardWeaponWorld)) {
            return false;
        }
        return applyWeaponVisualAuthority(weaponNode, feedForwardWeaponWorld);
    }


    // ---- Primary-only carry and firing-grip reattach ----

    void TwoHandedGrip::updatePrimaryOnlyGrip(
        RE::NiNode* weaponNode,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const EquippedWeaponPrimaryGripInput& primaryGripInput,
        const bool primaryDetachEnabled)
    {
        equipped_weapon_manual_ownership_policy::RuntimeState manualState{
            .active = true,
            .ownershipKey = _activeEquippedWeaponOwnershipKey,
        };
        const auto manualDecision = equipped_weapon_manual_ownership_policy::update(manualState,
            equipped_weapon_manual_ownership_policy::Input{
                .weaponEquipped = weaponNode != nullptr,
                .ownershipKey = currentEquippedWeaponOwnershipKey,
                .startRequested = false,
                .primaryGripRetained = equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership(
                    primaryDetachEnabled,
                    primaryGripInput.held),
                .supportGripRetained = false,
            });

        if (manualDecision.dropRequested) {
            beginHandVisualReturn(_firingHandIsLeft, "primary-only-drop");
            requestEquippedWeaponDrop("primary-only-grip-released",
                _firingHandIsLeft ? equipped_weapon_drop_policy::SourceHand::Left : equipped_weapon_drop_policy::SourceHand::Right);
            return;
        }

        if (manualDecision.cleared) {
            beginHandVisualReturn(_firingHandIsLeft, "primary-only-released");
            if (_firingHandIsLeft) {
                beginWeaponVisualReturn("left-primary-only-released");
            }
            transitionToInactive(false);
            return;
        }

        if (!_firingHandIsLeft) {
            // Right firing hand: FRIK-native carry, ROCK bookkeeping only.
            _hasSolvedWeaponTransform = false;
            return;
        }

        // Left firing hand: FRIK cannot carry (its weapon glue targets the
        // right hand and is blocked); ROCK drives the weapon rigidly from the
        // left hand through the captured weapon-relative firing-grip frame.
        // The left hand's finger pose is hFRIK's mirrored weapon-hand copy,
        // driven by the same ownership block.
        (void)solveLeftFiringWeaponCarry(weaponNode);
    }

    bool TwoHandedGrip::solveLeftFiringWeaponCarry(RE::NiNode* weaponNode)
    {
        if (!weaponNode || !_hasFiringHandWeaponLocal) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the captured firing-grip frame is unavailable");
            transitionToInactive(false);
            return false;
        }

        RE::NiTransform firingHandTransform{};
        if (!tryGetSolverHandTransform(true, firingHandTransform)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because authoritative hand transforms are unavailable");
            transitionToInactive(false);
            return false;
        }
        rememberFiringRecoilReference(true, firingHandTransform);

        const RE::NiTransform solvedWeaponWorld =
            transform_math::composeTransforms(firingHandTransform, transform_math::invertTransform(_primaryHandWeaponLocal));
        if (!isFiniteTransform(solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because the weapon solve produced an invalid transform");
            transitionToInactive(false);
            return false;
        }

        if (!applyWeaponVisualAuthority(weaponNode, solvedWeaponWorld)) {
            _hasSolvedWeaponTransform = false;
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: clearing left-firing carry because ROCK visual weapon authority failed");
            transitionToInactive(false);
            return false;
        }

        (void)publishAuthoredPrimaryFiringGripFingerPose(true);
        _lastSolvedWeaponTransform = weaponNode->world;
        _hasSolvedWeaponTransform = true;

        /*
         * Carry-time aim diagnostic (~3s cadence): the barrel direction in
         * LEFT-wand coordinates during the live carry. Matching the
         * takeover's barrelInLeftWand proves the carry chain is faithful to
         * the committed hold (residual cant then lives in the left wand /
         * melee driver chain and the aim trim is the right knob); a drift
         * from the takeover value means the left bone-in-wand relationship
         * changed after the topology swap and the hold must be resampled.
         */
        if (++_leftFiringAimLogCounter >= 270) {
            _leftFiringAimLogCounter = 0;
            auto* playerNodes = f4vr::getPlayerNodes();
            if (playerNodes && playerNodes->SecondaryWandNode && isFiniteTransform(playerNodes->SecondaryWandNode->world)) {
                const RE::NiTransform weaponInLeftWandNow = transform_math::composeTransforms(
                    transform_math::invertTransform(playerNodes->SecondaryWandNode->world), weaponNode->world);
                const RE::NiPoint3 barrelNow = sub(
                    transform_math::localPointToWorld(weaponInLeftWandNow, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }),
                    weaponInLeftWandNow.translate);
                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: left-firing carry aim barrelInLeftWand=({:.3f},{:.3f},{:.3f})",
                    barrelNow.x,
                    barrelNow.y,
                    barrelNow.z);
            }
        }
        return true;
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

        if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(handWeaponContact.weaponGenerationKey, _activeWeaponGenerationKey)) {
            return false;
        }

        float distance = 0.0f;
        return tryComputePalmToFiringGripDistance(
                   weaponNode,
                   handTransform,
                   handIsLeft,
                   distance) &&
               distance <=
                   _handlingSettings.firingGripReattachRadiusGameUnits;
    }

    bool TwoHandedGrip::tryComputePalmToFiringGripDistance(
        RE::NiNode* weaponNode,
        const RE::NiTransform& handTransform,
        const bool handIsLeft,
        float& outDistance) const
    {
        if (!weaponNode) {
            return false;
        }
        const RE::NiPoint3 palm =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                handTransform,
                handIsLeft);
        const RE::NiPoint3 firingGripWorld =
            weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiPoint3 delta = sub(palm, firingGripWorld);
        const float distance = std::sqrt(dot(delta, delta));
        if (!std::isfinite(distance)) {
            return false;
        }
        outDistance = distance;
        return true;
    }

    bool TwoHandedGrip::tryComputePalmToGripDistanceForHand(
        RE::NiNode* weaponNode,
        const bool handIsLeft,
        float& outDistance) const
    {
        RE::NiTransform handTransform{};
        return tryGetSolverHandTransform(handIsLeft, handTransform) &&
               tryComputePalmToFiringGripDistance(
                   weaponNode,
                   handTransform,
                   handIsLeft,
                   outDistance);
    }

    TwoHandedGrip::CanonicalFiringHoldSelection
        TwoHandedGrip::selectCanonicalFiringHold(
            const bool handIsLeft,
            RE::NiNode* weaponNode,
            const RE::NiTransform& liveHandWorld,
            const RE::NiTransform* preferredAuthoredCanonical,
            const char* preferredAuthoredSource) const
    {
        CanonicalFiringHoldSelection selection{};
        if (preferredAuthoredCanonical) {
            selection.handWeaponLocal = *preferredAuthoredCanonical;
            selection.source = preferredAuthoredSource ?
                preferredAuthoredSource :
                "authored-canonical";
            return selection;
        }

        if (handIsLeft) {
            bool usedAuthoredCanonical = false;
            if (tryComputeMirroredLeftFiringHandWeaponLocal(
                    selection.handWeaponLocal,
                    &usedAuthoredCanonical)) {
                selection.source = usedAuthoredCanonical ?
                    "authored-mirror" :
                    "native-mirror";
                return selection;
            }
        } else if (hasRightFiringHandCanonicalFrame(
                       _activeWeaponNode,
                       _activeWeaponGenerationKey,
                       _activeEquippedWeaponOwnershipKey)) {
            selection.handWeaponLocal =
                _rightFiringHandCanonicalWeaponLocal;
            selection.source = _rightFiringHandCanonicalSource ==
                    RightFiringCanonicalSource::AuthoredAnimation ?
                "authored-canonical" :
                "native-canonical";
            return selection;
        }

        if (!weaponNode) {
            return selection;
        }
        const RE::NiPoint3 palm =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                liveHandWorld,
                handIsLeft);
        const RE::NiPoint3 firingGripWorld =
            weaponLocalToWorld(_primaryGripLocal, weaponNode);
        const RE::NiTransform adjustedHandWorld =
            weapon_two_handed_grip_math::alignHandFrameToGripPoint(
                liveHandWorld,
                palm,
                firingGripWorld);
        selection.handWeaponLocal = transform_math::composeTransforms(
            transform_math::invertTransform(weaponNode->world),
            adjustedHandWorld);
        return selection;
    }

    bool TwoHandedGrip::tryResolveAuthoredFiringHandCanonicalForProbe(
        const bool handIsLeft,
        RE::NiTransform& outHandWeaponLocal,
        const char*& outSource) const
    {
        outHandWeaponLocal = {};
        outSource = "unavailable";
        if (!hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) ||
            _rightFiringHandCanonicalSource !=
                RightFiringCanonicalSource::AuthoredAnimation) {
            return false;
        }

        if (handIsLeft) {
            if (_leftFiringFingerLocalTransformMask !=
                authored_weapon_grip_library::
                    kCompleteFiringFingerMask) {
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

        if (_rightFiringFingerLocalTransformMask !=
            authored_weapon_grip_library::kCompleteFiringFingerMask) {
            return false;
        }
        outHandWeaponLocal = _rightFiringHandCanonicalWeaponLocal;
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
                _activeWeaponGenerationKey)) {
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

        // Validated: commit. A takeover by the other hand flips the firing
        // role here and reuses the SAME captured weapon-relative grip point.
        if (handIsLeft != _firingHandIsLeft) {
            setFiringHand(handIsLeft, "firing-grip-reattach-other-hand");
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
        const auto hold = selectCanonicalFiringHold(
            handIsLeft,
            weaponNode,
            handTransform,
            useAuthoredProbeCanonical ? &authoredProbeCanonical : nullptr,
            authoredProbeCanonicalSource);
        _primaryHandWeaponLocal = hold.handWeaponLocal;
        _hasFiringHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _firingGripSequence = ++_gripCaptureSequence;
        _primaryHandVisualLerp = {};
        clearPrimaryDetachVisualAuthority(handIsLeft);
        if (!_firingHandIsLeft) {
            restoreFrikPrimaryWeaponPose();
        }
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = handIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: firing hand reattached at configured grip hand={} hold={}",
            handIsLeft ? "left" : "right",
            hold.source);
        return true;
    }


    // ---- Canonical frames, mirroring, and node ownership ----

    void TwoHandedGrip::clearRightFiringHandCanonicalFrame()
    {
        _rightFiringHandCanonicalWeaponLocal = {};
        _rightFiringGripCanonicalWeaponLocal = {};
        _rightFiringHandCanonicalWeaponNode = nullptr;
        _rightFiringHandCanonicalGenerationKey = 0;
        _rightFiringHandCanonicalOwnershipKey = 0;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::None;
        _hasRightFiringHandCanonicalWeaponLocal = false;
        _rightFiringFingerLocalTransforms = {};
        _leftFiringFingerLocalTransforms = {};
        _rightFiringFingerLocalTransformMask = 0;
        _leftFiringFingerLocalTransformMask = 0;
    }

    bool TwoHandedGrip::hasRightFiringHandCanonicalFrame(
        const RE::NiNode* weaponNode,
        const std::uint64_t weaponGenerationKey,
        const std::uint64_t weaponOwnershipKey) const
    {
        return weaponNode &&
               weaponGenerationKey != 0 &&
               _hasRightFiringHandCanonicalWeaponLocal &&
               _rightFiringHandCanonicalWeaponNode == weaponNode &&
               _rightFiringHandCanonicalGenerationKey == weaponGenerationKey &&
               _rightFiringHandCanonicalOwnershipKey == weaponOwnershipKey &&
               _rightFiringHandCanonicalSource != RightFiringCanonicalSource::None;
    }

    void TwoHandedGrip::rememberRightFiringHandCanonicalFrame()
    {
        if (_firingHandIsLeft || !_activeWeaponNode ||
            !_hasFiringHandWeaponLocal || _activeWeaponGenerationKey == 0) {
            return;
        }
        if (hasRightFiringHandCanonicalFrame(
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey) &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }
        _rightFiringHandCanonicalWeaponLocal = _primaryHandWeaponLocal;
        _rightFiringGripCanonicalWeaponLocal = _primaryGripLocal;
        _rightFiringHandCanonicalWeaponNode = _activeWeaponNode;
        _rightFiringHandCanonicalGenerationKey = _activeWeaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = _activeEquippedWeaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _hasRightFiringHandCanonicalWeaponLocal = true;
    }

    void TwoHandedGrip::refreshNaturalHandInWandFrames()
    {
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes) {
            return;
        }

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
                        two_handed_grip_detail::
                            kMaximumBoneToDriverDistanceGameUnits) {
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
            _rightNaturalBoneInWand,
            _hasRightNaturalBoneInWand,
            _rightNaturalBoneInDampedDriver,
            _hasRightNaturalBoneInDampedDriver);
        refreshHand(
            true,
            playerNodes->SecondaryWandNode,
            playerNodes->SecondaryMeleeWeaponOffsetNode2,
            _leftNaturalBoneInWand,
            _hasLeftNaturalBoneInWand,
            _leftNaturalBoneInDampedDriver,
            _hasLeftNaturalBoneInDampedDriver);
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
        if (_weaponCollisionHandPresentationFromPreviousFrame[1] ||
            isManualOwnershipActive() || _weaponNodeOwnershipBlockEngaged ||
            !scope_safe_hand_frame_math::canRefreshRightFiringCanonicalFrame(_scopeMenuOpenThisFrame, _scopeSafeHandFrames[1].rootRebaseActive) || !weaponNode ||
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
        constexpr float kMaxWeaponToHandDistance = 100.0f;
        const RE::NiPoint3 weaponToHand = sub(weaponNode->world.translate, rightHandWorld.translate);
        if (!isFiniteTransform(boneInRightWand) ||
            std::sqrt(dot(boneInRightWand.translate, boneInRightWand.translate)) >
                two_handed_grip_detail::
                    kMaximumBoneToDriverDistanceGameUnits ||
            std::sqrt(dot(weaponToHand, weaponToHand)) > kMaxWeaponToHandDistance) {
            return;
        }
        const RE::NiTransform canonicalHold = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), rightHandWorld);
        const RE::NiPoint3 canonicalGrip = worldToWeaponLocal(computeGrabLegacyPalmPivotAWorldFromHandBasis(rightHandWorld, false), weaponNode);
        if (!isFiniteTransform(canonicalHold) || !std::isfinite(canonicalGrip.x) || !std::isfinite(canonicalGrip.y) || !std::isfinite(canonicalGrip.z)) {
            return;
        }
        // The animation capture is a more direct authority than a later
        // presentation sample. Preserve it for this exact weapon identity,
        // generation, and ownership.
        if (hasRightFiringHandCanonicalFrame(
                weaponNode,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey) &&
            _rightFiringHandCanonicalSource ==
                RightFiringCanonicalSource::AuthoredAnimation) {
            return;
        }

        _rightFiringHandCanonicalWeaponLocal = canonicalHold;
        _rightFiringGripCanonicalWeaponLocal = canonicalGrip;
        _rightFiringHandCanonicalWeaponNode = weaponNode;
        _rightFiringHandCanonicalGenerationKey = currentWeaponGenerationKey;
        _rightFiringHandCanonicalOwnershipKey = currentEquippedWeaponOwnershipKey;
        _rightFiringHandCanonicalCaptureSequence = 0;
        _rightFiringHandCanonicalSource = RightFiringCanonicalSource::NativeCarry;
        _hasRightFiringHandCanonicalWeaponLocal = true;
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
                _activeWeaponNode,
                _activeWeaponGenerationKey,
                _activeEquippedWeaponOwnershipKey)) {
            return false;
        }

        RE::NiTransform leftHandWorld{};
        if (!tryGetSolverHandTransform(true, leftHandWorld)) {
            return false;
        }

        /*
         * A part-gripping right hand is visually locked to the weapon part,
         * so its live bone no longer expresses the natural bone-in-wand
         * relation the wand conjugation depends on - a left reattach from a
         * right offhand carry came out at whatever angle the lock left the
         * bone. Replay the natural relation (snapshotted during native right
         * carry) onto the live right wand instead; every unlocked case keeps
         * the live sample the confirmed takeover path uses.
         */
        RE::NiTransform rightHandWorld{};
        if (partGrip(false).active && _hasRightNaturalBoneInWand) {
            auto* playerNodes = f4vr::getPlayerNodes();
            if (!playerNodes || !playerNodes->primaryWandNode || !isFiniteTransform(playerNodes->primaryWandNode->world)) {
                return false;
            }
            rightHandWorld = transform_math::composeTransforms(playerNodes->primaryWandNode->world, _rightNaturalBoneInWand);
        } else if (!tryGetSolverHandTransform(false, rightHandWorld)) {
            return false;
        }

        const bool mirrored = tryBuildMirroredLeftFiringHandWeaponLocal(
            _rightFiringHandCanonicalWeaponLocal,
            _rightFiringGripCanonicalWeaponLocal,
            rightHandWorld,
            leftHandWorld,
            outHandWeaponLocal,
            logDiagnostic);
        if (!mirrored) {
            return false;
        }

        const bool usedAuthoredCanonical =
            _rightFiringHandCanonicalSource ==
            RightFiringCanonicalSource::AuthoredAnimation;
        if (outUsedAuthoredCanonical) {
            *outUsedAuthoredCanonical = usedAuthoredCanonical;
        }
        if (logDiagnostic) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: left firing hold resolved source={} generation={:016X} ownership={:016X} capture={} mirroredHandWeaponT=({:.3f},{:.3f},{:.3f})",
                usedAuthoredCanonical ? "authored-animation" : "native-carry",
                _rightFiringHandCanonicalGenerationKey,
                _rightFiringHandCanonicalOwnershipKey,
                _rightFiringHandCanonicalCaptureSequence,
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
        if (!_hasLeftNaturalBoneInWand || !_hasRightNaturalBoneInWand ||
            !isFiniteTransform(leftHandWeaponLocal) ||
            !isFiniteTransform(_leftNaturalBoneInWand) ||
            !isFiniteTransform(_rightNaturalBoneInWand)) {
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
            orientationFrame(_leftNaturalBoneInWand);
        const RE::NiTransform rightBoneInWandOrientation =
            orientationFrame(_rightNaturalBoneInWand);
        const RE::NiTransform leftHandWeaponOrientation =
            orientationFrame(leftHandWeaponLocal);

        MirroredDriverConjugation conjugation{};
        if (!tryBuildMirroredDriverConjugation(
                leftHandWeaponOrientation,
                leftBoneInWandOrientation,
                rightBoneInWandOrientation,
                nullptr,
                conjugation)) {
            return false;
        }
        RE::NiTransform mirroredRightHandWeaponLocal =
            conjugation.targetHandWeaponLocal;

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
        const auto transformOffsetLength = [](const RE::NiTransform& transform) {
            return std::sqrt(dot(transform.translate, transform.translate));
        };
        if (!isFiniteTransform(boneInRightWand) || !isFiniteTransform(boneInLeftWand) ||
            transformOffsetLength(boneInRightWand) >
                two_handed_grip_detail::
                    kMaximumBoneToDriverDistanceGameUnits ||
            transformOffsetLength(boneInLeftWand) >
                two_handed_grip_detail::
                    kMaximumBoneToDriverDistanceGameUnits) {
            return false;
        }

        /*
         * Global left-hold trim, applied on the WAND side of the conjugation
         * (PRE-composed in the LEFT WAND frame), never on the weapon side.
         * The error it corrects is the fixed frame-convention delta between
         * the two wand device frames, which sits to the LEFT of the
         * conjugated hold. A weapon-side (post-composed) trim conjugates
         * through each weapon's own hold and therefore acts along different
         * axes per weapon: only the calibration weapon looked right, and
         * weapons with large authored holds (UMP forward offset, hunting
         * rifle) showed the trim rotated into unrelated directions
         * (2026-07-12 regression). Pre-composing makes one calibration exact
         * for every weapon, and if the mirror is fully correct these trims
         * converge to zero.
         *
         * Axes are the left wand's hand-anatomical basis (user-calibrated
         * in-game): X = palm normal, Y = fingers forward, Z = thumb up.
         * Yaw rotates about Z (thumb), pitch about X (palm normal); if a
         * value moves the aim opposite to its documented direction, the
         * user flips its sign once. The trim is ROTATION-ONLY: position is
         * anchored per weapon below, so a translation here would fight it.
         */
        const float aimYawRadians =
            handlingSettings.leftFiringAimYawDegrees *
            two_handed_grip_detail::kDegreesToRadians;
        const float aimPitchRadians =
            handlingSettings.leftFiringAimPitchDegrees *
            two_handed_grip_detail::kDegreesToRadians;
        const bool trimActive =
            aimYawRadians != 0.0f || aimPitchRadians != 0.0f;
        RE::NiTransform wandTrim{};
        wandTrim.MakeIdentity();
        if (trimActive) {
            RE::NiTransform yawTrim{};
            yawTrim.MakeIdentity();
            if (aimYawRadians != 0.0f) {
                const float yawCos = std::cos(aimYawRadians);
                const float yawSin = std::sin(aimYawRadians);
                // yaw about wand +Z (thumb axis)
                yawTrim.rotate.entry[0][0] = yawCos;
                yawTrim.rotate.entry[0][1] = -yawSin;
                yawTrim.rotate.entry[1][0] = yawSin;
                yawTrim.rotate.entry[1][1] = yawCos;
            }
            RE::NiTransform pitchTrim{};
            pitchTrim.MakeIdentity();
            if (aimPitchRadians != 0.0f) {
                const float pitchCos = std::cos(aimPitchRadians);
                const float pitchSin = std::sin(aimPitchRadians);
                // pitch about wand +X (palm-normal axis)
                pitchTrim.rotate.entry[1][1] = pitchCos;
                pitchTrim.rotate.entry[1][2] = pitchSin;
                pitchTrim.rotate.entry[2][1] = -pitchSin;
                pitchTrim.rotate.entry[2][2] = pitchCos;
            }
            wandTrim =
                transform_math::composeTransforms(yawTrim, pitchTrim);
        }

        MirroredDriverConjugation conjugation{};
        if (!tryBuildMirroredDriverConjugation(
                canonicalRightHandWeaponLocal,
                boneInRightWand,
                boneInLeftWand,
                trimActive ? &wandTrim : nullptr,
                conjugation)) {
            return false;
        }
        const RE::NiTransform& weaponInRightWand =
            conjugation.sourceWeaponInDriver;
        const RE::NiTransform& weaponInLeftWand =
            conjugation.targetWeaponInDriver;
        RE::NiTransform mirroredHandWeaponLocal =
            conjugation.targetHandWeaponLocal;

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
        const RE::NiPoint3 gripTargetInLeftHand{
            gripInRightHand.x + handlingSettings.leftFiringAimOffsetXGameUnits,
            gripInRightHand.y + handlingSettings.leftFiringAimOffsetYGameUnits,
            -gripInRightHand.z + handlingSettings.leftFiringAimOffsetZGameUnits
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
                "TwoHandedGrip: wand-conjugated left hold barrelInRightWand=({:.3f},{:.3f},{:.3f}) barrelInLeftWand=({:.3f},{:.3f},{:.3f}) boneWandDist=({:.2f},{:.2f})",
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

    void TwoHandedGrip::setFiringHand(const bool isLeft, const char* reason)
    {
        if (_firingHandIsLeft == isLeft) {
            return;
        }

        clearFiringRecoilPresentationState();
        // Drop the old hand's role-tagged FRIK publications; the new hand's
        // grip-frame capture and pose publication are owned by the caller.
        clearPrimaryGripPose(_firingHandIsLeft);
        clearPrimaryDetachVisualAuthority(_firingHandIsLeft);
        _primaryHandVisualLerp = {};
        _primaryReleaseDebounce = {};
        if (_persistentEquippedCarryActive) {
            _persistentEquippedCarryDetachArmed = false;
        }
        _firingHandIsLeft = isLeft;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: firing hand switched to {} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }

    bool TwoHandedGrip::tryPromoteSupportGripToFiringGrip(RE::NiNode* weaponNode)
    {
        const bool supportHandIsLeft = !_firingHandIsLeft;
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
        const RE::NiPoint3 firingGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
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
        const auto hold = selectCanonicalFiringHold(
            supportHandIsLeft,
            weaponNode,
            handTransform);

        beginHandVisualReturn(_firingHandIsLeft, "ambidextrous-firing-hand-promotion");
        setFiringHand(supportHandIsLeft, "support-grip-promotion");
        if (!transitionToPrimaryOnly(weaponNode, _activeWeaponGenerationKey, _activeEquippedWeaponOwnershipKey, "firing-grip-hand-promotion")) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: firing-grip promotion failed to enter primary-only; clearing authority");
            transitionToInactive(false);
            return true;
        }

        _primaryHandWeaponLocal = hold.handWeaponLocal;
        _hasFiringHandWeaponLocal = true;
        rememberRightFiringHandCanonicalFrame();
        _firingGripSequence = ++_gripCaptureSequence;
        _primaryHandVisualLerp = {};
        _hapticEvents.firingGripAttached = true;
        _hapticEvents.firingGripAttachedHandIsLeft = _firingHandIsLeft;
        ROCK_LOG_INFO(Weapon,
            "TwoHandedGrip: support hand promoted to firing grip hand={} gripToGrip={:.2f} hold={}",
            _firingHandIsLeft ? "left" : "right",
            supportGripToFiringGripDistance,
            hold.source);
        return true;
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
        const bool wantLeftFiringCarry = _firingHandIsLeft &&
            (_state == TwoHandedState::Gripping || _state == TwoHandedState::PrimaryOnly);

        if (!wantLeftFiringCarry) {
            releaseFiringHandWeaponNodeOwnership(weaponNode);
            return;
        }

        if (!_weaponNodeOwnershipBlockEngaged) {
            if (!frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, true)) {
                // Fail closed: without the FRIK block the weapon node would
                // fight two per-frame owners.
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: left-firing carry aborted because the FRIK weapon-node ownership block is unavailable");
                transitionToInactive(false);
                return;
            }
            _weaponNodeOwnershipBlockEngaged = true;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership blocked for left-firing carry");
        }

        if (!weaponNode) {
            return;
        }

        RE::NiNode* leftHand = resolveFirstPersonHandNode(true);
        if (!leftHand) {
            return;
        }
        if (weaponNode->parent == leftHand) {
            _weaponNodeReparentedToLeftHand = true;
            return;
        }

        /*
         * Re-parent under LArm_Hand preserving world so the scene graph keeps
         * the weapon riding the firing hand at every point in the frame
         * (native fire/aim sampling included). Same operation FRIK performs
         * for the game's own left-handed mode, minus the mirrored offsets.
         */
        const RE::NiTransform worldBefore = weaponNode->world;
        RE::NiTransform localInLeftHand{};
        if (!tryResolveWeaponRootLocal(
                leftHand,
                worldBefore,
                localInLeftHand)) {
            ROCK_LOG_SAMPLE_WARN(
                Weapon,
                2000,
                "TwoHandedGrip: left-firing weapon reparent rejected an invalid target frame");
            return;
        }
        RE::NiPointer<RE::NiAVObject> detached;
        if (weaponNode->parent) {
            weaponNode->parent->DetachChild(weaponNode, detached);
        }
        leftHand->AttachChild(weaponNode, true);
        weaponNode->local = localInLeftHand;
        weaponNode->world = worldBefore;
        _weaponNodeReparentedToLeftHand = true;
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented under LArm_Hand for left-firing carry");
    }

    void TwoHandedGrip::releaseFiringHandWeaponNodeOwnership(RE::NiNode* weaponNode)
    {
        // This function is also the idempotent right-firing topology path, so
        // it may retire only the left-carry recoil state. Right full-two-hand
        // recoil remains live until that grip state itself ends.
        clearFiringRecoilPresentationState(true);
        if (_weaponNodeReparentedToLeftHand) {
            RE::NiNode* node = weaponNode ? weaponNode : _activeWeaponNode;
            RE::NiNode* rightHand = resolveFirstPersonHandNode(false);
            if (node && rightHand && node->parent != rightHand) {
                const RE::NiTransform worldBefore = node->world;
                RE::NiTransform localInRightHand{};
                if (!tryResolveWeaponRootLocal(
                        rightHand,
                        worldBefore,
                        localInRightHand)) {
                    ROCK_LOG_SAMPLE_WARN(
                        Weapon,
                        2000,
                        "TwoHandedGrip: right-hand weapon reparent rejected an invalid target frame");
                } else {
                    RE::NiPointer<RE::NiAVObject> detached;
                    if (node->parent) {
                        node->parent->DetachChild(node, detached);
                    }
                    rightHand->AttachChild(node, true);
                    node->local = localInRightHand;
                    node->world = worldBefore;
                    ROCK_LOG_INFO(Weapon, "TwoHandedGrip: equipped weapon node re-parented back under RArm_Hand");
                }
            }
            _weaponNodeReparentedToLeftHand = false;
        }

        if (_weaponNodeOwnershipBlockEngaged) {
            // FRIK also force-reattaches native weapon-node parenting once the
            // block releases (belt and braces for teardown without nodes).
            (void)frik_visual_authority::blockPrimaryWeaponNodeOwnership(WEAPON_NODE_OWNERSHIP_TAG, false);
            _weaponNodeOwnershipBlockEngaged = false;
            ROCK_LOG_INFO(Weapon, "TwoHandedGrip: FRIK weapon-node ownership restored");
        }
    }

}

#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"
#include "physics-interaction/weapon/WeaponGripAuthorityPolicy.h"

#include <cassert>

int main()
{
    using namespace rock::native_animation_authority_policy;

    using rock::weapon_grip_authority_policy::Availability;
    using rock::weapon_grip_authority_policy::select;
    using rock::weapon_grip_authority_policy::Source;

    static_assert(select(Availability{
                      .frikCustomFile = true,
                      .authoredAnimation = true,
                      .frikEmbeddedResource = true,
                      .allowFrikLiveNodeFallback = true,
                  }) == Source::FrikCustomFile);
    static_assert(select(Availability{
                      .authoredAnimation = true,
                      .frikEmbeddedResource = true,
                      .allowFrikLiveNodeFallback = true,
                  }) == Source::AuthoredAnimation);
    static_assert(select(Availability{
                      .frikEmbeddedResource = true,
                      .allowFrikLiveNodeFallback = true,
                  }) == Source::FrikEmbeddedResource);
    static_assert(select(Availability{
                      .allowFrikLiveNodeFallback = true,
                  }) == Source::FrikLiveNodeFallback);
    static_assert(select(Availability{}) == Source::None);

    struct AffineTransform
    {
        float scale;
        float translate;
    };
    constexpr auto affineCompose = [](AffineTransform parent, AffineTransform child) {
        return AffineTransform{
            parent.scale * child.scale,
            parent.translate + parent.scale * child.translate,
        };
    };
    constexpr auto affineInvert = [](AffineTransform value) {
        return AffineTransform{
            1.0f / value.scale,
            -value.translate / value.scale,
        };
    };
    constexpr AffineTransform liveControl{ 5.0f, 100.0f };
    constexpr AffineTransform authoredBaseline{ 2.0f, 10.0f };
    constexpr AffineTransform authoredCurrent{ 6.0f, 18.0f };
    constexpr auto affineCorrection = resolveControllerAnchoredPoseCorrection(
        liveControl,
        authoredBaseline,
        authoredCurrent,
        affineCompose,
        affineInvert);
    constexpr auto correctedAuthoredCurrent = affineCompose(affineCorrection, authoredCurrent);
    static_assert(correctedAuthoredCurrent.scale == 15.0f);
    static_assert(correctedAuthoredCurrent.translate == 120.0f);

    constexpr auto additiveCompose = [](float parent, float child) { return parent + child; };
    constexpr auto additiveInvert = [](float value) { return -value; };
    static_assert(resolveControllerAnchoredPoseCorrection(
                      100.0f,
                      10.0f,
                      13.0f,
                      additiveCompose,
                      additiveInvert) == 90.0f);
    static_assert(additiveCompose(
                      resolveControllerAnchoredPoseCorrection(
                          100.0f,
                          10.0f,
                          13.0f,
                          additiveCompose,
                          additiveInvert),
                      13.0f) == 103.0f);

    constexpr auto sourceToSharedTarget = resolvePoseCorrectionToWorldTarget(
        103.0f,
        13.0f,
        additiveCompose,
        additiveInvert);
    constexpr auto destinationToSharedTarget = resolvePoseCorrectionToWorldTarget(
        103.0f,
        -79940.0f,
        additiveCompose,
        additiveInvert);
    static_assert(additiveCompose(sourceToSharedTarget, 13.0f) == 103.0f);
    static_assert(additiveCompose(destinationToSharedTarget, -79940.0f) == 103.0f);

    constexpr AffineTransform nativeCycleWeaponModel{ 2.0f, 40.0f };
    constexpr AffineTransform nativeCycleHandModel{ 4.0f, 80.0f };
    constexpr auto nativeCycleHandInWeapon = resolveNativeHandInWeapon(
        nativeCycleWeaponModel,
        nativeCycleHandModel,
        affineCompose,
        affineInvert);
    constexpr AffineTransform fixedCycleWeaponWorld{ 4.0f, 200.0f };
    constexpr auto fixedCycleHandWorld = resolveAuthoredPrimaryHandWorld(
        fixedCycleWeaponWorld,
        nativeCycleHandInWeapon,
        affineCompose);
    static_assert(nativeCycleHandInWeapon.scale == 2.0f);
    static_assert(nativeCycleHandInWeapon.translate == 20.0f);
    static_assert(fixedCycleHandWorld.scale == 8.0f);
    static_assert(fixedCycleHandWorld.translate == 280.0f);

    constexpr ManualCycleTwoHandEligibility manualCycleEligible{
        .twoHandGripActive = true,
        .firingHandIsLeft = false,
        .weaponTransformOwned = true,
    };
    static_assert(canApplyManualCycleHandAnimation(manualCycleEligible));
    static_assert([=] {
        auto input = manualCycleEligible;
        input.twoHandGripActive = false;
        return !canApplyManualCycleHandAnimation(input);
    }());
    static_assert([=] {
        auto input = manualCycleEligible;
        input.firingHandIsLeft = true;
        return !canApplyManualCycleHandAnimation(input);
    }());
    static_assert([=] {
        auto input = manualCycleEligible;
        input.weaponTransformOwned = false;
        return !canApplyManualCycleHandAnimation(input);
    }());

    static_assert(!updateManualCycleHandMotionQualification(
        false,
        ManualCycleHandMotionSample{
            .translationGameUnits = 1.499f,
            .rotationDegrees = 9.999f,
        }));
    static_assert(updateManualCycleHandMotionQualification(
        false,
        ManualCycleHandMotionSample{
            .translationGameUnits =
                kManualCycleHandMotionTranslationThresholdGameUnits,
        }));
    static_assert(updateManualCycleHandMotionQualification(
        false,
        ManualCycleHandMotionSample{
            .rotationDegrees =
                kManualCycleHandMotionRotationThresholdDegrees,
        }));
    static_assert(updateManualCycleHandMotionQualification(
        true,
        ManualCycleHandMotionSample{}));

    // A primary-hand cycle is encoded by the Weapon child's animated inverse
    // local. Reusing its flattened baseline local cancels the hand motion and
    // falsely attributes the delta to the support hand.
    constexpr float nativePrimaryHandBaseline = 10.0f;
    constexpr float nativePrimaryHandCurrent = 13.0f;
    constexpr float animatedWeaponLocalBaseline = 5.0f;
    constexpr float animatedWeaponLocalCurrent = 2.0f;
    constexpr float nativeWeaponBaseline = additiveCompose(
        nativePrimaryHandBaseline,
        animatedWeaponLocalBaseline);
    constexpr float nativeWeaponCurrent = additiveCompose(
        nativePrimaryHandCurrent,
        animatedWeaponLocalCurrent);
    static_assert(nativeWeaponBaseline == nativeWeaponCurrent);
    static_assert(resolveNativeHandInWeapon(
                      nativeWeaponBaseline,
                      nativePrimaryHandBaseline,
                      additiveCompose,
                      additiveInvert) == -5.0f);
    static_assert(resolveNativeHandInWeapon(
                      nativeWeaponCurrent,
                      nativePrimaryHandCurrent,
                      additiveCompose,
                      additiveInvert) == -2.0f);
    constexpr float falselyMovedWeapon = additiveCompose(
        nativePrimaryHandCurrent,
        animatedWeaponLocalBaseline);
    static_assert(resolveNativeHandInWeapon(
                      falselyMovedWeapon,
                      nativePrimaryHandCurrent,
                      additiveCompose,
                      additiveInvert) == -5.0f);

    constexpr AffineTransform nativeCycleBaselineHandInWeapon{ 2.0f, 40.0f };
    constexpr AffineTransform nativeCycleCurrentHandInWeapon{ 4.0f, 90.0f };
    constexpr AffineTransform liveCycleBaselineHandInWeapon{ 3.0f, 10.0f };
    constexpr auto cycleHandCorrection = resolveControllerAnchoredPoseCorrection(
        liveCycleBaselineHandInWeapon,
        nativeCycleBaselineHandInWeapon,
        nativeCycleCurrentHandInWeapon,
        affineCompose,
        affineInvert);
    constexpr auto rebasedCycleHandInWeapon = affineCompose(
        cycleHandCorrection,
        nativeCycleCurrentHandInWeapon);
    static_assert(rebasedCycleHandInWeapon.scale == 6.0f);
    static_assert(rebasedCycleHandInWeapon.translate == 85.0f);

    constexpr AffineTransform authoredHandInWeapon{ 2.0f, 10.0f };
    constexpr AffineTransform liveWeaponWorld{ 6.0f, 100.0f };
    constexpr auto authoredHandWorld = resolveAuthoredPrimaryHandWorld(
        liveWeaponWorld,
        authoredHandInWeapon,
        affineCompose);
    static_assert(authoredHandWorld.scale == 12.0f);
    static_assert(authoredHandWorld.translate == 160.0f);

    constexpr AffineTransform trackedPrimaryHandWorld{ 12.0f, 300.0f };
    constexpr auto alignedWeaponWorld = resolveAuthoredPrimaryWeaponWorld(
        trackedPrimaryHandWorld,
        authoredHandInWeapon,
        affineCompose,
        affineInvert);
    constexpr auto alignedAuthoredHandWorld = affineCompose(
        alignedWeaponWorld,
        authoredHandInWeapon);
    static_assert(alignedWeaponWorld.scale == 6.0f);
    static_assert(alignedWeaponWorld.translate == 240.0f);
    static_assert(alignedAuthoredHandWorld.scale == trackedPrimaryHandWorld.scale);
    static_assert(alignedAuthoredHandWorld.translate == trackedPrimaryHandWorld.translate);

    constexpr AffineTransform authoredPrimaryHandModel{ 2.0f, 20.0f };
    constexpr AffineTransform authoredSupportHandModel{ 6.0f, 80.0f };
    constexpr auto supportHandInPrimaryHand =
        resolveAuthoredSupportHandInPrimaryHand(
            authoredPrimaryHandModel,
            authoredSupportHandModel,
            affineCompose,
            affineInvert);
    constexpr auto authoredSupportHandInWeapon =
        resolveAuthoredSupportHandInWeapon(
            authoredHandInWeapon,
            supportHandInPrimaryHand,
            affineCompose);
    constexpr auto authoredPrimaryHandWorldForSupport = affineCompose(
        liveWeaponWorld,
        authoredHandInWeapon);
    constexpr auto authoredSupportHandWorldForSupport = affineCompose(
        liveWeaponWorld,
        authoredSupportHandInWeapon);
    static_assert(supportHandInPrimaryHand.scale == 3.0f);
    static_assert(supportHandInPrimaryHand.translate == 30.0f);
    static_assert(authoredSupportHandInWeapon.scale == 6.0f);
    static_assert(authoredSupportHandInWeapon.translate == 70.0f);
    static_assert(authoredSupportHandWorldForSupport.scale == 36.0f);
    static_assert(authoredSupportHandWorldForSupport.translate == 520.0f);
    static_assert(affineCompose(
                      authoredPrimaryHandWorldForSupport,
                      supportHandInPrimaryHand)
                      .translate == authoredSupportHandWorldForSupport.translate);

    constexpr AuthoredPrimaryFiringGripEligibility authoredGripEligible{
        .enabled = true,
        .runtimeInitialized = true,
        .visualAuthorityAvailable = true,
        .localSkeletonReady = true,
        .menuBlocking = false,
        .compatibilityBlocking = false,
        .weaponDrawn = true,
        .weaponVisible = true,
        .weaponKeyValid = true,
        .captureValid = true,
        .captureNewerThanWeaponBoundary = true,
        .nativeReloadAuthorityActive = false,
        .conflictingWeaponTransformAuthorityActive = false,
        .weaponVisualReturnActive = false,
        .primaryHandHoldingObject = false,
        .leftHandedMode = false,
        .rockFiringHandIsLeft = false,
    };
    static_assert(shouldApplyAuthoredPrimaryFiringGrip(authoredGripEligible));
    static_assert([=] {
        auto input = authoredGripEligible;
        input.nativeReloadAuthorityActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.conflictingWeaponTransformAuthorityActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.weaponVisualReturnActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.primaryHandHoldingObject = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.leftHandedMode = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.rockFiringHandIsLeft = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.captureNewerThanWeaponBoundary = false;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());

    constexpr AuthoredSupportGripCandidateInput authoredSupportEligible{
        .featureEnabled = true,
        .proximityProbeAcquisition = true,
        .authoredSeatTouchAcquisition = false,
        .providerAuthorityActive = false,
        .attachOnly = false,
        .captureValid = true,
        .weaponIdentityMatches = true,
        .generationMatches = true,
        .completeFingerPose = true,
    };
    static_assert(shouldUseAuthoredSupportGrip(authoredSupportEligible));
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.proximityProbeAcquisition = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.proximityProbeAcquisition = false;
        input.authoredSeatTouchAcquisition = true;
        return shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.providerAuthorityActive = true;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.attachOnly = true;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.generationMatches = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());

    constexpr AuthoredFiringGripProbeInput authoredFiringProbeEligible{
        .featureEnabled = true,
        .proximityProbeAcquisition = true,
        .providerAuthorityActive = false,
        .attachOnly = false,
        .authoredCanonicalAvailable = true,
    };
    static_assert(shouldUseAuthoredFiringGripProbe(authoredFiringProbeEligible));
    static_assert([=] {
        auto input = authoredFiringProbeEligible;
        input.proximityProbeAcquisition = false;
        return !shouldUseAuthoredFiringGripProbe(input);
    }());
    static_assert([=] {
        auto input = authoredFiringProbeEligible;
        input.providerAuthorityActive = true;
        return !shouldUseAuthoredFiringGripProbe(input);
    }());
    static_assert([=] {
        auto input = authoredFiringProbeEligible;
        input.attachOnly = true;
        return !shouldUseAuthoredFiringGripProbe(input);
    }());
    static_assert([=] {
        auto input = authoredFiringProbeEligible;
        input.authoredCanonicalAvailable = false;
        return !shouldUseAuthoredFiringGripProbe(input);
    }());
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.captureValid = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = authoredSupportEligible;
        input.completeFingerPose = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());

    constexpr LocalReloadLeaseState awaitingReload{
        .watchdogFramesRemaining = 600,
        .startSequenceAtArm = 10,
        .endSequenceAtArm = 4,
        .observedReloadStart = false,
    };
    constexpr auto awaitingEvent = advanceLocalReloadLease(awaitingReload, LocalReloadLifecycleSignal{ 10, 4, false });
    static_assert(awaitingEvent.active());
    static_assert(!awaitingEvent.state.observedReloadStart);
    constexpr auto activeReload = advanceLocalReloadLease(awaitingEvent.state, LocalReloadLifecycleSignal{ 11, 4, true });
    static_assert(activeReload.active());
    static_assert(activeReload.state.observedReloadStart);
    constexpr auto completedReload = advanceLocalReloadLease(activeReload.state, LocalReloadLifecycleSignal{ 11, 5, false });
    static_assert(!completedReload.active());
    static_assert(completedReload.endReason == LocalReloadLeaseEndReason::ReloadEnded);

    constexpr auto alreadyActiveAtArm = advanceLocalReloadLease(
        LocalReloadLeaseState{
            .watchdogFramesRemaining = 600,
            .startSequenceAtArm = 11,
            .endSequenceAtArm = 4,
            .observedReloadStart = true,
        },
        LocalReloadLifecycleSignal{ 11, 5, false });
    static_assert(!alreadyActiveAtArm.active());
    static_assert(alreadyActiveAtArm.endReason == LocalReloadLeaseEndReason::ReloadEnded);

    constexpr auto watchdogExpired = advanceLocalReloadLease(LocalReloadLeaseState{
                                                                  .watchdogFramesRemaining = 1,
                                                                  .startSequenceAtArm = 11,
                                                                  .endSequenceAtArm = 5,
                                                                  .observedReloadStart = true,
                                                              },
        LocalReloadLifecycleSignal{ 11, 5, true });
    static_assert(!watchdogExpired.active());
    static_assert(watchdogExpired.endReason == LocalReloadLeaseEndReason::WatchdogExpired);

    constexpr LocalManualCycleLeaseState manualCycleArmed{
        .watchdogSecondsRemaining = 4.0f,
        .reloadStartSequenceAtArm = 7,
        .lastReloadEndSequence = 20,
        .observedReloadEndEvents = 0,
    };
    constexpr auto manualCycleEntered = advanceLocalManualCycleLease(
        manualCycleArmed,
        LocalManualCycleLifecycleSignal{ 7, 21, 0.0625f });
    static_assert(manualCycleEntered.active());
    static_assert(manualCycleEntered.state.observedReloadEndEvents == 1);
    static_assert(manualCycleEntered.state.watchdogSecondsRemaining == 3.9375f);
    constexpr auto manualCycleCompleted = advanceLocalManualCycleLease(
        manualCycleEntered.state,
        LocalManualCycleLifecycleSignal{ 7, 22, 0.25f });
    static_assert(!manualCycleCompleted.active());
    static_assert(
        manualCycleCompleted.endReason ==
        LocalManualCycleLeaseEndReason::CycleBracketEnded);

    constexpr auto manualCycleReloadPreempted = advanceLocalManualCycleLease(
        manualCycleArmed,
        LocalManualCycleLifecycleSignal{ 8, 20, 0.25f });
    static_assert(!manualCycleReloadPreempted.active());
    static_assert(
        manualCycleReloadPreempted.endReason ==
        LocalManualCycleLeaseEndReason::ReloadStarted);

    constexpr auto manualCycleWatchdogExpired = advanceLocalManualCycleLease(
        LocalManualCycleLeaseState{
            .watchdogSecondsRemaining = 0.01f,
            .reloadStartSequenceAtArm = 7,
            .lastReloadEndSequence = 20,
        },
        LocalManualCycleLifecycleSignal{ 7, 20, 0.02f });
    static_assert(!manualCycleWatchdogExpired.active());
    static_assert(
        manualCycleWatchdogExpired.endReason ==
        LocalManualCycleLeaseEndReason::WatchdogExpired);

    static_assert(kManualCyclePose == (kArms | kHands));
    static_assert((kManualCyclePose & kWeapon) == 0);
    static_assert(classifyBone("RArm_Collarbone") == kArms);
    static_assert(classifyBone("LArm_ForeArm3") == kArms);
    static_assert(classifyBone("RArm_Hand") == (kArms | kHands));
    static_assert(classifyBone("LArm_Finger23") == kHands);
    static_assert(classifyBone("RArm_Thumb1") == kHands);
    static_assert(classifyBone("Weapon") == kWeapon);
    static_assert(classifyBone("weaponleft") == kWeapon);

    static_assert(classifyBone("Root") == 0);
    static_assert(classifyBone("COM") == 0);
    static_assert(classifyBone("SPINE1") == 0);
    static_assert(classifyBone("Chest") == 0);
    static_assert(classifyBone("Head") == 0);
    static_assert(classifyBone("LLeg_Thigh") == 0);
    static_assert(classifyBone("WeaponMagazine") == 0);

    assert(isRequested(classifyBone("LArm_Hand"), kArms));
    assert(isRequested(classifyBone("LArm_Hand"), kHands));
    assert(!isRequested(classifyBone("LArm_Finger11"), kArms));
    assert(!isRequested(classifyBone("RArm_UpperArm"), kHands));
    assert(isRequested(classifyBone("WeaponLeft"), kReloadPose));
    return 0;
}

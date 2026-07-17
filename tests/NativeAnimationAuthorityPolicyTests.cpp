#include "physics-interaction/animation/NativeAnimationAuthorityPolicy.h"

#include <cassert>

int main()
{
    using namespace rock::native_animation_authority_policy;

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

    constexpr AffineTransform authoredWeaponInHand{ 2.0f, 10.0f };
    constexpr AffineTransform liveWeaponWorld{ 6.0f, 100.0f };
    constexpr auto authoredHandWorld = resolveAuthoredPrimaryHandWorld(
        liveWeaponWorld,
        authoredWeaponInHand,
        affineCompose,
        affineInvert);
    constexpr auto recomposedWeaponWorld = affineCompose(authoredHandWorld, authoredWeaponInHand);
    static_assert(recomposedWeaponWorld.scale == liveWeaponWorld.scale);
    static_assert(recomposedWeaponWorld.translate == liveWeaponWorld.translate);

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
        .manualWeaponAuthorityActive = false,
        .primaryHandHoldingObject = false,
        .leftHandedMode = false,
    };
    static_assert(shouldApplyAuthoredPrimaryFiringGrip(authoredGripEligible));
    static_assert([=] {
        auto input = authoredGripEligible;
        input.nativeReloadAuthorityActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = authoredGripEligible;
        input.manualWeaponAuthorityActive = true;
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
        input.captureNewerThanWeaponBoundary = false;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
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

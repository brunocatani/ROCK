#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"

int main()
{
    using namespace rock::authored_weapon_grip_capture_policy;

    struct AffineTransform
    {
        float scale;
        float translate;
    };
    constexpr auto compose = [](
                                 const AffineTransform parent,
                                 const AffineTransform child) {
        return AffineTransform{
            parent.scale * child.scale,
            parent.translate + parent.scale * child.translate,
        };
    };
    constexpr auto invert = [](const AffineTransform value) {
        return AffineTransform{
            1.0f / value.scale,
            -value.translate / value.scale,
        };
    };

    constexpr AffineTransform authoredHandInWeapon{ 2.0f, 10.0f };

    struct Point3
    {
        float x;
        float y;
        float z;
    };
    struct RigidTransform
    {
        float rotation;
        Point3 translate;
        float scale;
    };
    constexpr RigidTransform nativeWeaponWorld{
        17.0f,
        { 100.0f, 200.0f, 300.0f },
        2.0f,
    };
    constexpr Point3 authoredGripWeaponLocal{ 5.0f, 10.0f, 15.0f };
    constexpr Point3 trackedPalmWorld{ 130.0f, 250.0f, 370.0f };
    constexpr auto positionOnlyWeaponWorld =
        resolveAuthoredPrimaryWeaponWorldPositionOnly(
            nativeWeaponWorld,
            authoredGripWeaponLocal,
            trackedPalmWorld,
            [](const RigidTransform& transform, const Point3& point) {
                return Point3{
                    transform.translate.x + point.x * transform.scale,
                    transform.translate.y + point.y * transform.scale,
                    transform.translate.z + point.z * transform.scale,
                };
            });
    static_assert(positionOnlyWeaponWorld.rotation == nativeWeaponWorld.rotation);
    static_assert(positionOnlyWeaponWorld.scale == nativeWeaponWorld.scale);
    static_assert(positionOnlyWeaponWorld.translate.x == 120.0f);
    static_assert(positionOnlyWeaponWorld.translate.y == 230.0f);
    static_assert(positionOnlyWeaponWorld.translate.z == 340.0f);

    constexpr AffineTransform primaryHandModel{ 2.0f, 20.0f };
    constexpr AffineTransform supportHandModel{ 6.0f, 80.0f };
    constexpr auto supportInPrimary =
        resolveAuthoredSupportHandInPrimaryHand(
            primaryHandModel,
            supportHandModel,
            compose,
            invert);
    constexpr auto supportInWeapon = resolveAuthoredSupportHandInWeapon(
        authoredHandInWeapon,
        supportInPrimary,
        compose);
    static_assert(supportInPrimary.scale == 3.0f);
    static_assert(supportInPrimary.translate == 30.0f);
    static_assert(supportInWeapon.scale == 6.0f);
    static_assert(supportInWeapon.translate == 70.0f);

    constexpr AuthoredPrimaryFiringGripEligibility eligible{
        .runtimeInitialized = true,
        .visualAuthorityAvailable = true,
        .localSkeletonReady = true,
        .weaponDrawn = true,
        .weaponVisible = true,
        .weaponKeyValid = true,
        .captureValid = true,
        .captureNewerThanWeaponBoundary = true,
    };
    static_assert(
        evaluateAuthoredPrimaryFiringGrip(eligible).action ==
        AuthoredPrimaryAction::Apply);
    static_assert([=] {
        auto input = eligible;
        input.weaponVisible = false;
        input.equippedWeaponTransitionActive = true;
        return evaluateAuthoredPrimaryFiringGrip(input).action ==
               AuthoredPrimaryAction::Apply;
    }());
    static_assert([=] {
        auto input = eligible;
        input.weaponVisible = false;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action == AuthoredPrimaryAction::Clear &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::WeaponNotVisible;
    }());
    static_assert([=] {
        auto input = eligible;
        input.nativeReloadAuthorityActive = true;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action == AuthoredPrimaryAction::Clear &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::NativeReloadAuthority;
    }());
    static_assert([=] {
        auto input = eligible;
        input.conflictingWeaponTransformAuthorityActive = true;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action == AuthoredPrimaryAction::Clear &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::
                       ConflictingWeaponAuthority;
    }());
    static_assert([=] {
        auto input = eligible;
        input.weaponVisualReturnActive = true;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action ==
                   AuthoredPrimaryAction::RetainPoseOnly &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::WeaponVisualReturn;
    }());
    static_assert([=] {
        auto input = eligible;
        input.primaryHandHoldingObject = true;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action == AuthoredPrimaryAction::Clear &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::
                       PrimaryHandHoldingObject;
    }());
    static_assert([=] {
        auto input = eligible;
        input.rockFiringHandIsLeft = true;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action == AuthoredPrimaryAction::Clear &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::PhysicalLeftFiring;
    }());
    static_assert([=] {
        auto input = eligible;
        input.captureNewerThanWeaponBoundary = false;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action == AuthoredPrimaryAction::Clear &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::CaptureNotFresh;
    }());
    static_assert(shouldPublishAuthoredFiringFingerPose(false));
    static_assert(!shouldPublishAuthoredFiringFingerPose(true));

    constexpr auto returnHandoff = [=] {
        auto input = eligible;
        input.weaponVisualReturnActive = true;
        return input;
    }();
    static_assert(
        evaluateAuthoredPrimaryFiringGrip(returnHandoff).action ==
        AuthoredPrimaryAction::RetainPoseOnly);
    static_assert([=] {
        auto input = returnHandoff;
        input.weaponVisualReturnActive = false;
        input.equippedWeaponTransitionActive = true;
        input.captureValid = false;
        const auto decision = evaluateAuthoredPrimaryFiringGrip(input);
        return decision.action ==
                   AuthoredPrimaryAction::RetainPoseOnly &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::
                       EquipTransitionContinuity;
    }());
    static_assert([=] {
        auto input = returnHandoff;
        input.nativeReloadAuthorityActive = true;
        return evaluateAuthoredPrimaryFiringGrip(input).action ==
               AuthoredPrimaryAction::Clear;
    }());
    static_assert([=] {
        auto input = returnHandoff;
        input.primaryHandHoldingObject = true;
        return evaluateAuthoredPrimaryFiringGrip(input).action ==
               AuthoredPrimaryAction::Clear;
    }());
    static_assert([=] {
        auto input = returnHandoff;
        input.weaponVisualReturnActive = false;
        return evaluateAuthoredPrimaryFiringGrip(input).action ==
               AuthoredPrimaryAction::Apply;
    }());

    constexpr AuthoredSupportGripCandidateInput supportEligible{
        .interactionAcquisitionValid = true,
        .activationZoneValid = true,
        .captureValid = true,
        .weaponIdentityMatches = true,
        .generationMatches = true,
        .completeFingerPose = true,
    };
    static_assert(shouldUseAuthoredSupportGrip(supportEligible));
    static_assert([=] {
        auto input = supportEligible;
        input.interactionAcquisitionValid = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = supportEligible;
        input.activationZoneValid = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = supportEligible;
        input.providerAuthorityActive = true;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = supportEligible;
        input.attachOnly = true;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = supportEligible;
        input.completeFingerPose = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    constexpr AuthoredFiringGripProbeInput firingProbeEligible{
        .proximityProbeAcquisition = true,
        .authoredCanonicalAvailable = true,
    };
    static_assert(shouldUseAuthoredFiringGripProbe(firingProbeEligible));
    static_assert([=] {
        auto input = firingProbeEligible;
        input.providerAuthorityActive = true;
        return !shouldUseAuthoredFiringGripProbe(input);
    }());
    static_assert([=] {
        auto input = firingProbeEligible;
        input.attachOnly = true;
        return !shouldUseAuthoredFiringGripProbe(input);
    }());

    constexpr StableAuthoredSupportGripReuseInput stableSupportReusable{
        .snapshotValid = true,
        .weaponNodeValid = true,
        .weaponNodeMatches = true,
        .currentWeaponOwnershipKey = 11,
        .snapshotWeaponOwnershipKey = 11,
        .currentWeaponGenerationKey = 22,
        .snapshotWeaponGenerationKey = 22,
        .currentPrimaryGripCaptureSequence = 33,
        .snapshotPrimaryGripCaptureSequence = 33,
        .snapshotSupportGripCaptureSequence = 44,
        .snapshotFingerLocalTransformMask =
            kCompleteAuthoredSupportFingerLocalTransformMask,
    };
    static_assert(shouldReuseStableAuthoredSupportGrip(
        stableSupportReusable));
    static_assert([=] {
        auto input = stableSupportReusable;
        input.weaponNodeMatches = false;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportReusable;
        input.snapshotWeaponOwnershipKey = 12;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportReusable;
        input.snapshotWeaponGenerationKey = 23;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportReusable;
        input.snapshotPrimaryGripCaptureSequence = 34;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportReusable;
        input.snapshotSupportGripCaptureSequence = 0;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportReusable;
        input.snapshotFingerLocalTransformMask = 0x3FFFu;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());

    constexpr LeftFiringTakeoverSupportCaptureInput pendingLeftTakeover{
        .pendingStartMatchesCurrentWeapon = true,
        .primaryOnlyStartRequested = true,
        .firingHandIsLeft = true,
        .authoredOnlySupportGrabsEnabled = true,
    };
    static_assert(shouldReserveNativeRightSupportCaptureFrame(
        pendingLeftTakeover));
    static_assert([=] {
        auto input = pendingLeftTakeover;
        input.currentSupportCandidateAvailable = true;
        return !shouldReserveNativeRightSupportCaptureFrame(input);
    }());
    static_assert([=] {
        auto input = pendingLeftTakeover;
        input.nativeRightCaptureFrameReserved = true;
        return !shouldReserveNativeRightSupportCaptureFrame(input);
    }());
    static_assert([=] {
        auto input = pendingLeftTakeover;
        input.firingHandIsLeft = false;
        return !shouldReserveNativeRightSupportCaptureFrame(input);
    }());
    static_assert([=] {
        auto input = pendingLeftTakeover;
        input.authoredOnlySupportGrabsEnabled = false;
        return !shouldReserveNativeRightSupportCaptureFrame(input);
    }());

    static_assert(kArms == (1u << 0));
    static_assert(kHands == (1u << 1));
    static_assert(kWeapon == (1u << 2));
    static_assert(equalsIgnoreCase("RArm_Hand", "rarm_hand"));
    static_assert(!equalsIgnoreCase("Weapon", "WeaponLeft"));
    return 0;
}

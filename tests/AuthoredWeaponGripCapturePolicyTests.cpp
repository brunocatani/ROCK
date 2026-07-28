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
    constexpr AffineTransform liveWeaponWorld{ 6.0f, 100.0f };
    constexpr auto authoredHandWorld = resolveAuthoredPrimaryHandWorld(
        liveWeaponWorld,
        authoredHandInWeapon,
        compose);
    static_assert(authoredHandWorld.scale == 12.0f);
    static_assert(authoredHandWorld.translate == 160.0f);

    constexpr AffineTransform trackedHandWorld{ 12.0f, 300.0f };
    constexpr auto alignedWeaponWorld = resolveAuthoredPrimaryWeaponWorld(
        trackedHandWorld,
        authoredHandInWeapon,
        compose,
        invert);
    constexpr auto alignedHandWorld = compose(
        alignedWeaponWorld,
        authoredHandInWeapon);
    static_assert(alignedWeaponWorld.scale == 6.0f);
    static_assert(alignedWeaponWorld.translate == 240.0f);
    static_assert(alignedHandWorld.scale == trackedHandWorld.scale);
    static_assert(alignedHandWorld.translate == trackedHandWorld.translate);

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
    static_assert(shouldApplyAuthoredPrimaryFiringGrip(eligible));
    static_assert([=] {
        auto input = eligible;
        input.nativeReloadAuthorityActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = eligible;
        input.conflictingWeaponTransformAuthorityActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = eligible;
        input.weaponVisualReturnActive = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = eligible;
        input.primaryHandHoldingObject = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = eligible;
        input.rockFiringHandIsLeft = true;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert([=] {
        auto input = eligible;
        input.captureNewerThanWeaponBoundary = false;
        return !shouldApplyAuthoredPrimaryFiringGrip(input);
    }());
    static_assert(shouldPublishAuthoredFiringFingerPose(false));
    static_assert(!shouldPublishAuthoredFiringFingerPose(true));

    constexpr AuthoredSupportGripCandidateInput supportEligible{
        .proximityProbeAcquisition = true,
        .captureValid = true,
        .weaponIdentityMatches = true,
        .generationMatches = true,
        .authoredSeatWeaponSurfaceValid = true,
        .completeFingerPose = true,
    };
    static_assert(shouldUseAuthoredSupportGrip(supportEligible));
    static_assert([=] {
        auto input = supportEligible;
        input.proximityProbeAcquisition = false;
        input.authoredSeatTouchAcquisition = true;
        return shouldUseAuthoredSupportGrip(input);
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
    static_assert([=] {
        auto input = supportEligible;
        input.authoredSeatWeaponSurfaceValid = false;
        return !shouldUseAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = supportEligible;
        input.proximityProbeAcquisition = false;
        input.authoredSeatTouchAcquisition = true;
        input.authoredSeatWeaponSurfaceValid = false;
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

    static_assert(kArms == (1u << 0));
    static_assert(kHands == (1u << 1));
    static_assert(kWeapon == (1u << 2));
    static_assert(equalsIgnoreCase("RArm_Hand", "rarm_hand"));
    static_assert(!equalsIgnoreCase("Weapon", "WeaponLeft"));
    return 0;
}

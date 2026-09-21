#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/weapon/VanillaWeaponGripFrame.h"
#include <limits>

int main()
{
    using namespace rock::authored_weapon_grip_capture_policy;

    if (resolveWeaponPresentationScale(false, 0.865347f) != 1.0f) return 90;
    if (resolveWeaponPresentationScale(true, 1.0f) != 1.0f) return 91;
    if (resolveWeaponPresentationScale(true, 0.75f) != 0.75f) return 92;
    if (resolveWeaponPresentationScale(true, 1.25f) != 1.25f) return 93;
    if (resolveWeaponPresentationScale(true, 0.0f) != 0.0f) return 94;
    if (resolveWeaponPresentationScale(true, std::numeric_limits<float>::quiet_NaN()) != 1.0f) return 95;
    if (resolveWeaponPresentationScale(true, std::numeric_limits<float>::infinity()) != 1.0f) return 96;

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
        return decision.action == AuthoredPrimaryAction::RetainPoseOnly &&
               decision.reason ==
                   AuthoredPrimaryDecisionReason::
                       ConflictingWeaponAuthority;
    }());
    static_assert([=] {
        // Every suspension must still release the pose while the two-hand
        // solver owns weapon transforms.
        for (int suspension = 0; suspension < 11; ++suspension) {
            auto input = eligible;
            input.conflictingWeaponTransformAuthorityActive = true;
            switch (suspension) {
            case 0: input.runtimeInitialized = false; break;
            case 1: input.visualAuthorityAvailable = false; break;
            case 2: input.localSkeletonReady = false; break;
            case 3: input.menuBlocking = true; break;
            case 4: input.compatibilityBlocking = true; break;
            case 5: input.weaponKeyValid = false; break;
            case 6: input.nativeReloadAuthorityActive = true; break;
            case 7: input.primaryHandHoldingObject = true; break;
            case 8: input.weaponDrawn = false; break;
            case 9: input.weaponVisible = false; break;
            case 10: input.rockFiringHandIsLeft = true; break;
            }
            if (evaluateAuthoredPrimaryFiringGrip(input).action !=
                AuthoredPrimaryAction::Clear) {
                return false;
            }
        }
        return true;
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
    static_assert(shouldPublishAuthoredFiringFingerPose(false, false));
    static_assert(!shouldPublishAuthoredFiringFingerPose(true, false));
    static_assert(!shouldPublishAuthoredFiringFingerPose(false, true));
    static_assert(!shouldPublishAuthoredFiringFingerPose(true, true));

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
        .canonicalRelationMatches = true,
        .powerArmorMatches = true,
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
        input.canonicalRelationMatches = false;
        return !shouldReuseStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportReusable;
        input.powerArmorMatches = false;
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

    constexpr StableAuthoredSupportGripRebindInput stableSupportRebindable{
        .snapshotValid = true,
        .weaponNodeValid = true,
        .weaponNodeMatches = true,
        .currentWeaponOwnershipKey = 11,
        .snapshotWeaponOwnershipKey = 11,
        .currentWeaponGenerationKey = 23,
        .snapshotWeaponGenerationKey = 22,
        .currentWeaponInstanceContentKnown = true,
        .snapshotWeaponInstanceContentKnown = true,
        .currentWeaponInstanceContentKey = 55,
        .snapshotWeaponInstanceContentKey = 55,
        .canonicalRelationMatches = true,
        .powerArmorMatches = true,
        .snapshotSupportGripCaptureSequence = 44,
        .snapshotFingerLocalTransformMask =
            kCompleteAuthoredSupportFingerLocalTransformMask,
    };
    static_assert(shouldRebindStableAuthoredSupportGrip(
        stableSupportRebindable));
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.currentWeaponGenerationKey =
            input.snapshotWeaponGenerationKey;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.weaponNodeMatches = false;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.currentWeaponOwnershipKey = 12;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.currentWeaponInstanceContentKnown = false;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.snapshotWeaponInstanceContentKnown = false;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.snapshotWeaponInstanceContentKey = 56;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.canonicalRelationMatches = false;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.powerArmorMatches = false;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.snapshotSupportGripCaptureSequence = 0;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportRebindable;
        input.snapshotFingerLocalTransformMask = 0x3FFFu;
        return !shouldRebindStableAuthoredSupportGrip(input);
    }());

    constexpr StableAuthoredSupportGripAdoptInput stableSupportAdoptable{
        .snapshotValid = true,
        .weaponNodeValid = true,
        .currentWeaponOwnershipKey = 12,
        .currentWeaponGenerationKey = 24,
        .currentWeaponInstanceContentKnown = true,
        .snapshotWeaponInstanceContentKnown = true,
        .currentWeaponInstanceContentKey = 55,
        .snapshotWeaponInstanceContentKey = 55,
        .canonicalRelationMatches = true,
        .powerArmorMatches = true,
        .snapshotSupportGripCaptureSequence = 44,
        .snapshotFingerLocalTransformMask =
            kCompleteAuthoredSupportFingerLocalTransformMask,
    };
    static_assert(shouldAdoptStableAuthoredSupportGrip(
        stableSupportAdoptable));
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.snapshotValid = false;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.currentWeaponOwnershipKey = 0;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.currentWeaponGenerationKey = 0;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.currentWeaponInstanceContentKnown = false;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.snapshotWeaponInstanceContentKey = 56;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.canonicalRelationMatches = false;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.powerArmorMatches = false;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.snapshotSupportGripCaptureSequence = 0;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());
    static_assert([=] {
        auto input = stableSupportAdoptable;
        input.snapshotFingerLocalTransformMask = 0x3FFFu;
        return !shouldAdoptStableAuthoredSupportGrip(input);
    }());

    // Live support capture convergence witness: a run restarts on any
    // non-matching frame, converges only after kStableAuthoredSupportCaptureFrames
    // matching frames, and saturates instead of growing unbounded.
    static_assert(advanceStableAuthoredSupportCaptureWitness(0, false) == 1);
    static_assert(advanceStableAuthoredSupportCaptureWitness(1, true) == 2);
    static_assert(advanceStableAuthoredSupportCaptureWitness(2, true) == 3);
    static_assert(advanceStableAuthoredSupportCaptureWitness(3, true) == 3);
    static_assert(advanceStableAuthoredSupportCaptureWitness(3, false) == 1);
    static_assert(!stableAuthoredSupportCaptureConverged(0));
    static_assert(!stableAuthoredSupportCaptureConverged(1));
    static_assert(!stableAuthoredSupportCaptureConverged(2));
    static_assert(stableAuthoredSupportCaptureConverged(3));
    static_assert(stableAuthoredSupportCaptureConverged(kStableAuthoredSupportCaptureFrames));

    static_assert(kArms == (1u << 0));
    static_assert(kHands == (1u << 1));
    static_assert(kWeapon == (1u << 2));
    static_assert(equalsIgnoreCase("RArm_Hand", "rarm_hand"));
    static_assert(!equalsIgnoreCase("Weapon", "WeaponLeft"));

    // SMG animation anchors are expressed before its model registration.
    // Carry the whole authored hand pair through that translation, including
    // replacement animation offsets, without changing either orientation.
    namespace frame = rock::vanilla_weapon_grip_frame;
    static_assert(frame::hasVanillaModelRegistration(0x0015B043));
    static_assert(!frame::hasVanillaModelRegistration(0x0115B043));
    static_assert(!frame::hasVanillaModelRegistration(0x00024F55));
    struct Matrix { float entry[3][3]; };
    struct Grip { Matrix rotate; Point3 translate; float scale; };
    constexpr Matrix identity{{ {1,0,0}, {0,1,0}, {0,0,1} }};
    constexpr Matrix turned{{ {0,1,0}, {-1,0,0}, {0,0,1} }};
    constexpr Point3 registration{0, -14.02198f, 0};
    constexpr Grip vanilla{identity, {3.12409f, 7.72383f, -3.22147f}, 1};
    constexpr Grip replacement{turned, {1.90928f, 6.18517f, -3.67387f}, 0.9f};
    constexpr Grip support{turned, {-4.089f, 30.645f, -1.756f}, 1};
    constexpr auto seatedVanilla = frame::translateGrip(vanilla, registration);
    constexpr auto seatedReplacement = frame::translateGrip(replacement, registration);
    constexpr auto seatedSupport = frame::translateGrip(support, registration);
    static_assert(seatedVanilla.rotate.entry[0][0] == 1);
    static_assert(seatedReplacement.rotate.entry[0][1] == 1 && seatedReplacement.scale == 0.9f);
    static_assert(seatedReplacement.translate.x == replacement.translate.x);
    static_assert(seatedReplacement.translate.z == replacement.translate.z);
    if (std::abs((seatedReplacement.translate.y - seatedVanilla.translate.y) -
                 (replacement.translate.y - vanilla.translate.y)) > 0.00001f) return 70;
    if (std::abs((seatedSupport.translate.y - seatedVanilla.translate.y) -
                 (support.translate.y - vanilla.translate.y)) > 0.00001f) return 71;
    constexpr auto loose = frame::translateGrip(vanilla, Point3{});
    static_assert(loose.translate.y == vanilla.translate.y);
    Grip receiver{identity, registration, 1}, marker = receiver;
    if (!frame::registrationAgrees(receiver, marker)) return 72;
    marker.translate.y += 1;
    if (frame::registrationAgrees(receiver, marker)) return 73;
    marker = receiver;
    marker.rotate = turned;
    if (frame::registrationAgrees(receiver, marker)) return 74;
    marker = receiver;
    marker.scale = 2;
    if (frame::registrationAgrees(receiver, marker)) return 75;
    marker = receiver;
    marker.translate.y = std::numeric_limits<float>::quiet_NaN();
    if (frame::registrationAgrees(receiver, marker)) return 76;
    // Position-only hold tolerance: a floor near the origin, the float rounding of the world coordinate far from it.
    static_assert(positionOnlyHoldGripErrorTolerance(0.0f) == kPositionOnlyHoldGripErrorFloorGameUnits);
    static_assert(positionOnlyHoldGripErrorTolerance(100.0f) == kPositionOnlyHoldGripErrorFloorGameUnits);
    if (positionOnlyHoldGripErrorTolerance(76154.0f) <= 0.0133f) return 77;
    if (positionOnlyHoldGripErrorTolerance(76154.0f) >= 0.1f) return 78;
    if (positionOnlyHoldGripErrorTolerance(-76154.0f) != positionOnlyHoldGripErrorTolerance(76154.0f)) return 79;
    return 0;
}

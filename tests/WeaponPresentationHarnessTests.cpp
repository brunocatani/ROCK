#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/collision/DynamicWeaponCollisionPolicy.h"

#include "support/FakeFrikExternalAuthority.h"
#include "support/FakePhysicsProposalSource.h"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

namespace
{
    using namespace rock;
    using rock::test_support::FakeFrikExternalAuthority;
    using rock::test_support::FakePhysicsProposalSource;
    using rock::test_support::PhysicsProposalIdentity;
    using rock::test_support::PhysicsProposalSnapshot;
    using Hand = FakeFrikExternalAuthority::Hand;

    constexpr float kEpsilon = 0.002f;

    bool expectTrue(const char* label, const bool actual)
    {
        if (actual) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, const bool actual)
    {
        if (!actual) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectNear(const char* label, const float actual, const float expected, const float epsilon = kEpsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    RE::NiTransform identityTransform()
    {
        RE::NiTransform result{};
        result.rotate.MakeIdentity();
        result.scale = 1.0f;
        return result;
    }

    RE::NiMatrix3 rotationAboutZ(const float degrees)
    {
        const float radians = degrees * static_cast<float>(std::numbers::pi) / 180.0f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiMatrix3 result{};
        result.entry[0][0] = c;
        result.entry[0][1] = -s;
        result.entry[0][2] = 0.0f;
        result.entry[1][0] = s;
        result.entry[1][1] = c;
        result.entry[1][2] = 0.0f;
        result.entry[2][0] = 0.0f;
        result.entry[2][1] = 0.0f;
        result.entry[2][2] = 1.0f;
        return result;
    }

    RE::NiMatrix3 rotationAboutX(const float degrees)
    {
        const float radians = degrees * static_cast<float>(std::numbers::pi) / 180.0f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiMatrix3 result{};
        result.entry[0][0] = 1.0f;
        result.entry[0][1] = 0.0f;
        result.entry[0][2] = 0.0f;
        result.entry[1][0] = 0.0f;
        result.entry[1][1] = c;
        result.entry[1][2] = -s;
        result.entry[2][0] = 0.0f;
        result.entry[2][1] = s;
        result.entry[2][2] = c;
        return result;
    }

    RE::NiTransform makeTransform(const RE::NiMatrix3& rotation, const float x, const float y, const float z)
    {
        RE::NiTransform result{};
        result.rotate = rotation;
        result.translate = RE::NiPoint3{ x, y, z };
        result.scale = 1.0f;
        return result;
    }

    bool expectSameTransform(const char* label, const RE::NiTransform& actual, const RE::NiTransform& expected)
    {
        bool ok = true;
        ok &= expectNear((std::string(label) + " translate.x").c_str(), actual.translate.x, expected.translate.x);
        ok &= expectNear((std::string(label) + " translate.y").c_str(), actual.translate.y, expected.translate.y);
        ok &= expectNear((std::string(label) + " translate.z").c_str(), actual.translate.z, expected.translate.z);
        ok &= expectNear(
            (std::string(label) + " rotation").c_str(),
            dynamic_weapon_collision_policy::rotationDeltaDegrees(actual, expected),
            0.0f,
            0.05f);
        return ok;
    }

    // A publication is data. It says nothing about where the hand ends up.
    bool testPublicationIsDeferredAndDataOnly()
    {
        bool ok = true;
        FakeFrikExternalAuthority frik;
        const RE::NiTransform tracked = makeTransform(rotationAboutZ(0.0f), 0.0f, 0.0f, 0.0f);
        const RE::NiTransform target = makeTransform(rotationAboutZ(0.0f), 5.0f, 0.0f, 0.0f);
        frik.setTrackedHandWorld(Hand::Right, tracked);
        frik.solveSkeletonFrame();

        ok &= expectTrue("publish is accepted", frik.publish("ROCK_WeaponCollisionHand", Hand::Right, target, 110));
        // The claim exists, but this frame's skeleton has already been solved.
        ok &= expectSameTransform("the wrist does not move on the publishing frame", frik.presentedWrist(Hand::Right), tracked);

        frik.solveSkeletonFrame();
        ok &= expectSameTransform("the next skeleton pass consumes the claim", frik.presentedWrist(Hand::Right), target);

        // The claim persists. It is not a per-frame request.
        frik.solveSkeletonFrame();
        ok &= expectSameTransform("a retained claim keeps owning the wrist", frik.presentedWrist(Hand::Right), target);

        ok &= expectTrue("clear is accepted", frik.clear("ROCK_WeaponCollisionHand", Hand::Right));
        frik.solveSkeletonFrame();
        ok &= expectSameTransform("clearing returns the tracked hand", frik.presentedWrist(Hand::Right), tracked);
        return ok;
    }

    bool testWinnerArbitration()
    {
        bool ok = true;
        FakeFrikExternalAuthority frik;
        const RE::NiTransform grip = makeTransform(rotationAboutZ(0.0f), 1.0f, 0.0f, 0.0f);
        const RE::NiTransform collision = makeTransform(rotationAboutZ(0.0f), 2.0f, 0.0f, 0.0f);
        const RE::NiTransform laterGrip = makeTransform(rotationAboutZ(0.0f), 3.0f, 0.0f, 0.0f);

        ok &= expectTrue("grip publishes", frik.publish("ROCK_WeaponPrimaryGrip", Hand::Right, grip, 100));
        ok &= expectTrue("collision publishes", frik.publish("ROCK_WeaponCollisionHand", Hand::Right, collision, 110));
        FakeFrikExternalAuthority::Claim winner{};
        ok &= expectTrue("a winner exists", frik.tryGetWinner(Hand::Right, winner));
        ok &= expectTrue("the higher priority claim wins", winner.tag == "ROCK_WeaponCollisionHand");

        // Republishing the loser does not take the hand back.
        ok &= expectTrue("grip republishes", frik.publish("ROCK_WeaponPrimaryGrip", Hand::Right, laterGrip, 100));
        ok &= expectTrue("a winner still exists", frik.tryGetWinner(Hand::Right, winner));
        ok &= expectTrue("priority beats recency", winner.tag == "ROCK_WeaponCollisionHand");

        // At equal priority the later publication wins.
        ok &= expectTrue("a second equal claim publishes", frik.publish("ROCK_WeaponSupportGrip", Hand::Right, laterGrip, 110));
        ok &= expectTrue("a winner still exists", frik.tryGetWinner(Hand::Right, winner));
        ok &= expectTrue("recency breaks a priority tie", winner.tag == "ROCK_WeaponSupportGrip");
        return ok;
    }

    /*
     * The defect this whole work item exists for: publish returns true, the
     * solve silently falls back, and nothing in the API reports it. Only a
     * readback of the presented wrist can tell the difference.
     */
    bool testUnreachableTargetFallsBackSilently()
    {
        bool ok = true;
        FakeFrikExternalAuthority frik;
        const RE::NiTransform tracked = identityTransform();
        frik.setTrackedHandWorld(Hand::Right, tracked);
        frik.setReachLimitGameUnits(20.0f);

        const RE::NiTransform unreachable = makeTransform(rotationAboutZ(0.0f), 500.0f, 0.0f, 0.0f);
        ok &= expectTrue("an unreachable target still publishes", frik.publish("ROCK_WeaponCollisionHand", Hand::Right, unreachable, 110));
        frik.solveSkeletonFrame();
        ok &= expectTrue("the solve fell back", frik.lastSolveUsedFallback(Hand::Right));
        ok &= expectSameTransform("the wrist stayed on the tracked hand", frik.presentedWrist(Hand::Right), tracked);

        FakeFrikExternalAuthority::Claim winner{};
        ok &= expectTrue("the claim is still the winner after the fallback", frik.tryGetWinner(Hand::Right, winner));
        ok &= expectNear("the registry still reports the unreached target", winner.target.translate.x, unreachable.translate.x);

        const RE::NiTransform reachable = makeTransform(rotationAboutZ(0.0f), 5.0f, 0.0f, 0.0f);
        ok &= expectTrue("a reachable target publishes", frik.publish("ROCK_WeaponCollisionHand", Hand::Right, reachable, 110));
        frik.solveSkeletonFrame();
        ok &= expectFalse("a reachable target does not fall back", frik.lastSolveUsedFallback(Hand::Right));
        ok &= expectSameTransform("a reachable target is presented", frik.presentedWrist(Hand::Right), reachable);
        return ok;
    }

    // A two-hand group where hFRIK refuses the second hand. Without a rollback
    // the frame keeps one claim, which is exactly the split presentation.
    bool testPartialGroupNeedsRollback()
    {
        bool ok = true;
        FakeFrikExternalAuthority frik;
        frik.setRejectedTag("ROCK_WeaponCollisionHand_Left");
        const RE::NiTransform target = makeTransform(rotationAboutZ(0.0f), 4.0f, 0.0f, 0.0f);

        const bool rightPublished = frik.publish("ROCK_WeaponCollisionHand", Hand::Right, target, 110);
        const bool leftPublished = frik.publish("ROCK_WeaponCollisionHand_Left", Hand::Left, target, 110);
        ok &= expectTrue("the first hand publishes", rightPublished);
        ok &= expectFalse("the refused hand does not publish", leftPublished);
        ok &= expectTrue("a partial group leaves one live claim", frik.liveClaimCount() == 1u);

        // Rolling the transaction back must leave no owner behind.
        if (rightPublished) {
            ok &= expectTrue("rollback clears the accepted hand", frik.clear("ROCK_WeaponCollisionHand", Hand::Right));
        }
        ok &= expectTrue("rollback leaves no live claim", frik.liveClaimCount() == 0u);
        return ok;
    }

    bool testRecoilIsComposedIntoTheTarget()
    {
        bool ok = true;
        FakeFrikExternalAuthority frik;
        frik.setTrackedHandWorld(Hand::Right, identityTransform());
        const RE::NiTransform target = makeTransform(rotationAboutZ(0.0f), 4.0f, 0.0f, 0.0f);
        const RE::NiTransform kick = makeTransform(rotationAboutZ(0.0f), 0.0f, -2.0f, 0.0f);
        ok &= expectTrue("the target publishes", frik.publish("ROCK_WeaponCollisionHand", Hand::Right, target, 110));
        frik.setPendingRecoilDelta(Hand::Right, kick);
        frik.solveSkeletonFrame();
        ok &= expectSameTransform(
            "the kick is composed into the solved wrist",
            frik.presentedWrist(Hand::Right),
            transform_math::composeTransforms(kick, target));

        // The ticket is spent. The next solve returns the plain target.
        frik.solveSkeletonFrame();
        ok &= expectSameTransform("the recoil ticket is consumed once", frik.presentedWrist(Hand::Right), target);
        return ok;
    }

    /*
     * A collision correction is a world-space LEFT delta on the weapon. Batch
     * 3 defers it by one frame and then applies it to a NEW intent, so this
     * property is what makes the deferral sound: composing the correction
     * onto the current intent preserves the same world delta the physics
     * solve produced, and is NOT the same as re-applying it in weapon-local
     * space when the rotations do not commute.
     */
    bool testCorrectionIsAWorldLeftDelta()
    {
        bool ok = true;
        const RE::NiPoint3 center{ 1.0f, 2.0f, -3.0f };
        constexpr float kWeaponScale = 1.0f;

        const RE::NiTransform sampledRequestedProxy = makeTransform(rotationAboutZ(35.0f), 10.0f, 20.0f, 30.0f);
        const RE::NiTransform sampledLiveProxy = makeTransform(rotationAboutX(20.0f), 12.0f, 19.0f, 31.0f);
        // The player kept moving while the physics solve was in flight.
        const RE::NiTransform currentIntent = makeTransform(rotationAboutZ(80.0f), 40.0f, -5.0f, 12.0f);

        const RE::NiTransform sampledRequestedWeapon =
            dynamic_weapon_collision_policy::reconstructWeaponRoot(sampledRequestedProxy, center, kWeaponScale);
        const RE::NiTransform sampledLiveWeapon =
            dynamic_weapon_collision_policy::reconstructWeaponRoot(sampledLiveProxy, center, kWeaponScale);
        const RE::NiTransform resolved = dynamic_weapon_collision_policy::resolveCurrentIntentFromSample(
            sampledRequestedProxy,
            sampledLiveProxy,
            center,
            kWeaponScale,
            currentIntent);

        const RE::NiTransform sampledWorldDelta = transform_math::composeTransforms(
            sampledLiveWeapon,
            transform_math::invertTransform(sampledRequestedWeapon));
        const RE::NiTransform appliedWorldDelta = transform_math::composeTransforms(
            resolved,
            transform_math::invertTransform(currentIntent));
        ok &= expectSameTransform(
            "the correction reaches the new intent as the same world delta",
            appliedWorldDelta,
            sampledWorldDelta);

        // The weapon-local alternative is a different transform. If this ever
        // stops differing the test has lost its discriminating power.
        const RE::NiTransform weaponLocalAlternative = transform_math::composeTransforms(
            currentIntent,
            transform_math::composeTransforms(
                transform_math::invertTransform(sampledRequestedWeapon),
                sampledLiveWeapon));
        const float alternativeGapGameUnits =
            dynamic_weapon_collision_policy::translationDeltaGameUnits(weaponLocalAlternative, resolved);
        ok &= expectTrue(
            "the weapon-local alternative is a genuinely different transform",
            alternativeGapGameUnits > 1.0f);
        return ok;
    }

    // The group property: the hand takes the identical world delta as the
    // weapon, so the pair stays rigid however the correction is composed.
    bool testHandReframeTakesTheSameWorldDelta()
    {
        bool ok = true;
        const RE::NiTransform requestedWeapon = makeTransform(rotationAboutZ(15.0f), 3.0f, 4.0f, 5.0f);
        const RE::NiTransform resolvedWeapon = makeTransform(rotationAboutX(50.0f), 9.0f, -2.0f, 7.0f);
        const RE::NiTransform requestedHand = makeTransform(rotationAboutZ(-70.0f), 2.0f, 6.0f, 1.0f);

        const RE::NiTransform reframedHand =
            dynamic_weapon_collision_policy::reframeAttachedHand(requestedWeapon, resolvedWeapon, requestedHand);
        const RE::NiTransform weaponWorldDelta = transform_math::composeTransforms(
            resolvedWeapon,
            transform_math::invertTransform(requestedWeapon));
        const RE::NiTransform handWorldDelta = transform_math::composeTransforms(
            reframedHand,
            transform_math::invertTransform(requestedHand));
        ok &= expectSameTransform("the hand takes the weapon world delta", handWorldDelta, weaponWorldDelta);
        return ok;
    }

    /*
     * Retained contact is what keeps a collision claim alive after the
     * manifold stops producing points. It is also why a single touch can hold
     * a priority-110 hand claim for a third of a second. Pin the numbers so
     * the admission gate work changes them deliberately.
     */
    bool testContactRetentionAndVisualAuthority()
    {
        bool ok = true;
        namespace dwc = dynamic_weapon_collision_policy;
        constexpr float kFrameSeconds = 1.0f / 90.0f;

        float retention = 0.0f;
        retention = dwc::advanceProcessedManifoldContactRetention(retention, true, false, kFrameSeconds);
        ok &= expectNear("a positive witness arms full retention", retention, dwc::kProcessedManifoldContactRetentionSeconds);

        // A renewing witness holds the claim indefinitely.
        for (int frame = 0; frame < 200; ++frame) {
            retention = dwc::advanceProcessedManifoldContactRetention(retention, true, false, kFrameSeconds);
        }
        ok &= expectNear("a renewing witness never decays", retention, dwc::kProcessedManifoldContactRetentionSeconds);

        int framesUntilRelease = 0;
        while (retention > 0.0f && framesUntilRelease < 1000) {
            retention = dwc::advanceProcessedManifoldContactRetention(retention, false, false, kFrameSeconds);
            ++framesUntilRelease;
        }
        ok &= expectNear("retention decays to zero", retention, 0.0f);
        ok &= expectTrue("a single touch holds the claim for about a third of a second", framesUntilRelease >= 30 && framesUntilRelease <= 33);

        retention = dwc::advanceProcessedManifoldContactRetention(dwc::kProcessedManifoldContactRetentionSeconds, true, true, kFrameSeconds);
        ok &= expectNear("a teleport drops retention immediately", retention, 0.0f);

        // While contact is retained the weapon keeps a single owner even when
        // the correction is too small to see.
        const auto retainedInvisible = dwc::decideVisualAuthority(true, false);
        ok &= expectTrue("retained contact keeps publishing", retainedInvisible.publish);
        ok &= expectFalse("an invisible correction is not applied", retainedInvisible.useResolvedWeaponWorld);

        const auto retainedVisible = dwc::decideVisualAuthority(true, true);
        ok &= expectTrue("a visible correction publishes", retainedVisible.publish);
        ok &= expectTrue("a visible correction is applied", retainedVisible.useResolvedWeaponWorld);

        const auto freeSpace = dwc::decideVisualAuthority(false, true);
        ok &= expectFalse("free space yields the weapon", freeSpace.publish);
        return ok;
    }

    bool testProposalAdmission()
    {
        bool ok = true;
        FakePhysicsProposalSource source;
        const PhysicsProposalIdentity current{
            .world = 0xABCD,
            .bodyId = 42u,
            .generationKey = 0x1111'2222'3333'4444ull,
        };

        PhysicsProposalSnapshot snapshot{};
        snapshot.identity = current;
        snapshot.contactActive = true;
        source.publish(snapshot);

        PhysicsProposalSnapshot read{};
        ok &= expectTrue("a published proposal reads back", source.read(read));
        ok &= expectTrue("a current proposal is admissible", FakePhysicsProposalSource::isAdmissible(read, current));

        // The weapon was re-equipped between the solve and the read.
        auto staleGeneration = snapshot;
        staleGeneration.identity.generationKey = 0x9999ull;
        ok &= expectFalse("a stale generation is refused", FakePhysicsProposalSource::isAdmissible(staleGeneration, current));

        // The body set was rebuilt.
        auto staleBody = snapshot;
        staleBody.identity.bodyId = 43u;
        ok &= expectFalse("a stale body is refused", FakePhysicsProposalSource::isAdmissible(staleBody, current));

        // The world went away and came back.
        auto staleWorld = snapshot;
        staleWorld.identity.world = 0xDEAD;
        ok &= expectFalse("a stale world is refused", FakePhysicsProposalSource::isAdmissible(staleWorld, current));

        // A teleported body was moved, not solved: it carries no correction.
        auto teleported = snapshot;
        teleported.teleported = true;
        ok &= expectFalse("a teleported proposal is refused", FakePhysicsProposalSource::isAdmissible(teleported, current));

        source.clear();
        ok &= expectFalse("a cleared source reads nothing", source.read(read));
        return ok;
    }

    bool testSkeletonDropReleasesEveryClaim()
    {
        bool ok = true;
        FakeFrikExternalAuthority frik;
        const RE::NiTransform target = makeTransform(rotationAboutZ(0.0f), 4.0f, 0.0f, 0.0f);
        ok &= expectTrue("left publishes", frik.publish("ROCK_WeaponCollisionHand", Hand::Left, target, 110));
        ok &= expectTrue("right publishes", frik.publish("ROCK_WeaponCollisionHand", Hand::Right, target, 110));
        ok &= expectTrue("both claims are live", frik.liveClaimCount() == 2u);
        frik.dropSkeleton();
        ok &= expectTrue("a dropped skeleton takes every claim", frik.liveClaimCount() == 0u);
        FakeFrikExternalAuthority::Claim winner{};
        ok &= expectFalse("no winner survives the drop", frik.tryGetWinner(Hand::Right, winner));
        return ok;
    }
}

int main()
{
    bool ok = true;
    ok &= testPublicationIsDeferredAndDataOnly();
    ok &= testWinnerArbitration();
    ok &= testUnreachableTargetFallsBackSilently();
    ok &= testPartialGroupNeedsRollback();
    ok &= testRecoilIsComposedIntoTheTarget();
    ok &= testCorrectionIsAWorldLeftDelta();
    ok &= testHandReframeTakesTheSameWorldDelta();
    ok &= testContactRetentionAndVisualAuthority();
    ok &= testProposalAdmission();
    ok &= testSkeletonDropReleasesEveryClaim();
    return ok ? 0 : 1;
}

#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace
{
    using namespace rock::hand_world_claim_registry_policy;

    bool expectTrue(const char* label, bool value)
    {
        if (value) {
            return true;
        }
        std::printf("%s expected true\n", label);
        return false;
    }

    bool expectFalse(const char* label, bool value)
    {
        if (!value) {
            return true;
        }
        std::printf("%s expected false\n", label);
        return false;
    }

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }
        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

    template <class Enum>
    bool expectEnum(const char* label, Enum actual, Enum expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %d got %d\n", label, static_cast<int>(expected), static_cast<int>(actual));
        return false;
    }

    RE::NiTransform identity()
    {
        return rock::transform_math::makeIdentityTransform<RE::NiTransform>();
    }

    RE::NiTransform translated(float x, float y, float z)
    {
        RE::NiTransform t = identity();
        t.translate = RE::NiPoint3{ x, y, z };
        return t;
    }

    RE::NiTransform yawed(float degrees, float x = 0.0f, float y = 0.0f, float z = 0.0f)
    {
        const float radians = degrees * 0.017453292519943295f;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        RE::NiTransform t = identity();
        t.rotate.entry[0][0] = c;
        t.rotate.entry[0][1] = -s;
        t.rotate.entry[1][0] = s;
        t.rotate.entry[1][1] = c;
        t.translate = RE::NiPoint3{ x, y, z };
        return t;
    }

    DriverSample sample(const RE::NiTransform& world)
    {
        return DriverSample{ .world = world, .valid = true };
    }
}

int main()
{
    bool ok = true;

    // Commit, update in place, tie-break by publish order, remove.
    {
        Registry registry{};
        ok &= expectEnum("insert A", commit(registry, "ROCK_A", false, 100, translated(1, 0, 0), RebaseDriver::RightHand, sample(identity())), CommitResult::Inserted);
        ok &= expectEnum("insert B", commit(registry, "ROCK_B", false, 100, translated(2, 0, 0), RebaseDriver::RightHand, sample(identity())), CommitResult::Inserted);
        ok &= expectTrue("right has claim", hasClaim(registry, false));
        ok &= expectFalse("left has no claim", hasClaim(registry, true));
        const Claim* top = winner(registry, false);
        ok &= expectTrue("winner exists", top != nullptr);
        ok &= expectTrue("newest publish wins the tie", top && tagView(*top) == "ROCK_B");

        ok &= expectEnum("update A", commit(registry, "ROCK_A", false, 100, translated(3, 0, 0), RebaseDriver::RightHand, sample(identity())), CommitResult::Updated);
        top = winner(registry, false);
        ok &= expectTrue("re-published A wins the tie", top && tagView(*top) == "ROCK_A");
        ok &= expectTrue("claim count is two", claimCount(registry) == 2);

        ok &= expectEnum("higher priority", commit(registry, "ROCK_C", false, 110, translated(4, 0, 0), RebaseDriver::Static, {}), CommitResult::Inserted);
        top = winner(registry, false);
        ok &= expectTrue("priority beats order", top && tagView(*top) == "ROCK_C");

        ok &= expectTrue("remove C", remove(registry, "ROCK_C", false));
        ok &= expectFalse("remove C twice", remove(registry, "ROCK_C", false));
        ok &= expectFalse("remove wrong hand", remove(registry, "ROCK_A", true));
        top = winner(registry, false);
        ok &= expectTrue("A wins after C removed", top && tagView(*top) == "ROCK_A");
    }

    // Validation and capacity.
    {
        Registry registry{};
        ok &= expectEnum("empty tag", commit(registry, "", false, 1, identity(), RebaseDriver::Static, {}), CommitResult::InvalidTag);
        const std::string longTag(kTagCapacity, 'x');
        ok &= expectEnum("tag too long", commit(registry, longTag, false, 1, identity(), RebaseDriver::Static, {}), CommitResult::InvalidTag);
        RE::NiTransform bad = identity();
        bad.translate.x = std::nanf("");
        ok &= expectEnum("non-finite target", commit(registry, "ROCK_X", false, 1, bad, RebaseDriver::Static, {}), CommitResult::InvalidTarget);
        for (std::size_t i = 0; i < kMaxClaims; ++i) {
            const std::string tag = "ROCK_" + std::to_string(i);
            ok &= expectEnum("fill", commit(registry, tag, i % 2 == 0, 1, identity(), RebaseDriver::Static, {}), CommitResult::Inserted);
        }
        ok &= expectEnum("full", commit(registry, "ROCK_overflow", false, 1, identity(), RebaseDriver::Static, {}), CommitResult::Full);
        ok &= expectEnum("update while full", commit(registry, "ROCK_0", true, 1, identity(), RebaseDriver::Static, {}), CommitResult::Updated);
        const auto generation = registry.generation;
        clearAll(registry);
        ok &= expectTrue("cleared", claimCount(registry) == 0);
        ok &= expectTrue("generation bumped", registry.generation == generation + 1);
        ok &= expectTrue("publish order reset", registry.nextPublishOrder == 1);
    }

    // Rebase math: a driver translation and rotation move the target rigidly.
    {
        Registry registry{};
        const RE::NiTransform driverAtPublish = yawed(0.0f, 10.0f, 0.0f, 0.0f);
        const RE::NiTransform target = translated(12.0f, 1.0f, 0.0f); // 2 gu in front of the driver along +X, 1 gu +Y
        ok &= expectEnum("insert", commit(registry, "ROCK_G", false, 90, target, RebaseDriver::RightHand, sample(driverAtPublish)), CommitResult::Inserted);
        Claim* claim = find(registry, "ROCK_G", false);
        ok &= expectTrue("found", claim != nullptr);

        // Pure translation of the driver by (+5, 0, +2).
        const DriverSample moved = sample(yawed(0.0f, 15.0f, 0.0f, 2.0f));
        const RebasePlan translatedPlan = planRebase(*claim, moved);
        ok &= expectTrue("translation republishes", translatedPlan.republish);
        ok &= expectNear("translated x", translatedPlan.target.translate.x, 17.0f, 0.001f);
        ok &= expectNear("translated y", translatedPlan.target.translate.y, 1.0f, 0.001f);
        ok &= expectNear("translated z", translatedPlan.target.translate.z, 2.0f, 0.001f);
        applyRebase(*claim, translatedPlan, moved);
        ok &= expectNear("claim target advanced", claim->target.translate.x, 17.0f, 0.001f);
        ok &= expectNear("driver sample advanced", claim->driverAtPublish.world.translate.x, 15.0f, 0.001f);

        // Rotation of the driver by 90 degrees about Z in place: the offset (2, 1) becomes (-1, 2) in the stored-row convention.
        const DriverSample rotated = sample(yawed(90.0f, 15.0f, 0.0f, 2.0f));
        const RebasePlan rotatedPlan = planRebase(*claim, rotated);
        ok &= expectTrue("rotation republishes", rotatedPlan.republish);
        const float dx = rotatedPlan.target.translate.x - 15.0f;
        const float dy = rotatedPlan.target.translate.y - 0.0f;
        ok &= expectNear("rotated offset length", std::sqrt(dx * dx + dy * dy), std::sqrt(5.0f), 0.001f);
        ok &= expectNear("rotated 90 degrees", rotationDeltaDegrees(rotatedPlan.target, claim->target), 90.0f, 0.01f);

        // No motion: below epsilon, no republish, target unchanged.
        const RebasePlan stillPlan = planRebase(*claim, moved);
        ok &= expectFalse("no motion no republish", stillPlan.republish);
        ok &= expectNear("still target", stillPlan.target.translate.x, claim->target.translate.x, 0.0001f);

        // Static driver and missing samples never rebase.
        Claim staticClaim = *claim;
        staticClaim.driver = RebaseDriver::Static;
        ok &= expectFalse("static never republishes", planRebase(staticClaim, rotated).republish);
        ok &= expectFalse("invalid driver now", planRebase(*claim, DriverSample{}).republish);

        // A position driver carries the translation only: the driver's turn
        // leaves the target's orientation and its offset from the hand alone.
        Claim positionClaim = *claim;
        positionClaim.driver = RebaseDriver::RightHandPosition;
        positionClaim.target = translated(20.0f, 3.0f, 2.0f);
        positionClaim.driverAtPublish = moved;
        const DriverSample turnedAndMoved = sample(yawed(90.0f, 16.0f, 0.0f, 4.0f));
        const RebasePlan positionPlan = planRebase(positionClaim, turnedAndMoved);
        ok &= expectTrue("position republishes", positionPlan.republish);
        ok &= expectNear("position x", positionPlan.target.translate.x, 21.0f, 0.001f);
        ok &= expectNear("position y", positionPlan.target.translate.y, 3.0f, 0.001f);
        ok &= expectNear("position z", positionPlan.target.translate.z, 4.0f, 0.001f);
        ok &= expectNear("position keeps rotation", rotationDeltaDegrees(positionPlan.target, positionClaim.target), 0.0f, 0.001f);
        const DriverFrame positionFrame{ .sequence = 1, .hands = { moved, DriverSample{} } };
        ok &= expectTrue("position driver samples the hand", sampleForDriver(positionFrame, RebaseDriver::RightHandPosition) == &positionFrame.hands[0]);
        ok &= expectTrue("left position driver samples the left hand", sampleForDriver(positionFrame, RebaseDriver::LeftHandPosition) == &positionFrame.hands[1]);
        Claim noPublishSample = *claim;
        noPublishSample.driverAtPublish.valid = false;
        ok &= expectFalse("invalid publish sample", planRebase(noPublishSample, rotated).republish);
    }

    // Aim-axis driver: the hand's translation, and the turn of the axis from the other hand to this one.
    {
        Claim aimClaim{};
        aimClaim.valid = true;
        aimClaim.driver = RebaseDriver::RightHandAimAxis;
        aimClaim.target = translated(12.0f, 1.0f, 0.0f);
        aimClaim.driverAtPublish = sample(translated(10.0f, 0.0f, 0.0f));
        aimClaim.otherDriverAtPublish = sample(identity()); // axis at publish: +X
        // The other hand stays; this hand moves to +Y (axis turns 90 degrees about Z) and twists in place (ignored).
        const DriverSample ownNow = sample(yawed(30.0f, 0.0f, 10.0f, 0.0f));
        const DriverSample otherNow = sample(identity());
        const RebasePlan aimPlan = planRebase(aimClaim, ownNow, otherNow);
        ok &= expectTrue("aim republishes", aimPlan.republish);
        ok &= expectNear("aim x", aimPlan.target.translate.x, 2.0f, 0.001f);
        ok &= expectNear("aim y", aimPlan.target.translate.y, 11.0f, 0.001f);
        ok &= expectNear("aim z", aimPlan.target.translate.z, 0.0f, 0.001f);
        ok &= expectNear("aim turned 90 degrees", rotationDeltaDegrees(aimPlan.target, aimClaim.target), 90.0f, 0.01f);
        const RE::NiPoint3 forward = rock::transform_math::localVectorToWorld(aimPlan.target, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        ok &= expectNear("aim local X follows the axis (x)", forward.x, 0.0f, 0.001f);
        ok &= expectNear("aim local X follows the axis (y)", forward.y, 1.0f, 0.001f);
        ok &= expectNear("aim local X follows the axis (z)", forward.z, 0.0f, 0.001f);
        ok &= expectNear("aim result is a rotation", static_cast<float>(rock::transform_math::storedRotationOrthonormalityError(aimPlan.target.rotate)), 0.0f, 0.0001f);

        // Both hands translate together: the axis is unchanged, no turn.
        const RebasePlan togetherPlan = planRebase(aimClaim, sample(translated(13.0f, 2.0f, 1.0f)), sample(translated(3.0f, 2.0f, 1.0f)));
        ok &= expectTrue("together republishes", togetherPlan.republish);
        ok &= expectNear("together x", togetherPlan.target.translate.x, 15.0f, 0.001f);
        ok &= expectNear("together y", togetherPlan.target.translate.y, 3.0f, 0.001f);
        ok &= expectNear("together keeps rotation", rotationDeltaDegrees(togetherPlan.target, aimClaim.target), 0.0f, 0.001f);

        // Without the other hand's sample the driver degrades to translation only.
        const RebasePlan noOtherPlan = planRebase(aimClaim, ownNow, DriverSample{});
        ok &= expectNear("no other sample x", noOtherPlan.target.translate.x, 2.0f, 0.001f);
        ok &= expectNear("no other sample y", noOtherPlan.target.translate.y, 11.0f, 0.001f);
        ok &= expectNear("no other sample keeps rotation", rotationDeltaDegrees(noOtherPlan.target, aimClaim.target), 0.0f, 0.001f);
        Claim noOtherAtPublish = aimClaim;
        noOtherAtPublish.otherDriverAtPublish = {};
        ok &= expectNear("no other publish sample keeps rotation", rotationDeltaDegrees(planRebase(noOtherAtPublish, ownNow, otherNow).target, aimClaim.target), 0.0f, 0.001f);

        // Hands too close to define an axis: no turn.
        Claim closeClaim = aimClaim;
        closeClaim.driverAtPublish = sample(translated(0.5f, 0.0f, 0.0f));
        ok &= expectNear("degenerate axis keeps rotation", rotationDeltaDegrees(planRebase(closeClaim, sample(translated(0.0f, 0.5f, 0.0f)), otherNow).target, aimClaim.target), 0.0f, 0.001f);

        // Sample lookup: own hand as the driver, the other hand for the axis.
        DriverFrame frame{};
        frame.hands[handIndex(false)] = sample(translated(1, 0, 0));
        frame.hands[handIndex(true)] = sample(translated(2, 0, 0));
        ok &= expectTrue("right aim driver samples the right hand", sampleForDriver(frame, RebaseDriver::RightHandAimAxis) == &frame.hands[handIndex(false)]);
        ok &= expectTrue("right aim driver's other is the left hand", otherHandSampleForDriver(frame, RebaseDriver::RightHandAimAxis) == &frame.hands[handIndex(true)]);
        ok &= expectTrue("left aim driver's other is the right hand", otherHandSampleForDriver(frame, RebaseDriver::LeftHandAimAxis) == &frame.hands[handIndex(false)]);
        ok &= expectTrue("full driver has no other", otherHandSampleForDriver(frame, RebaseDriver::LeftHand) == nullptr);
        ok &= expectTrue("aim driver for hand", aimAxisDriverForHand(true) == RebaseDriver::LeftHandAimAxis && aimAxisDriverForHand(false) == RebaseDriver::RightHandAimAxis);
        ok &= expectTrue("aim drivers are not position drivers", !isPositionDriver(RebaseDriver::LeftHandAimAxis) && isAimAxisDriver(RebaseDriver::LeftHandAimAxis) && !isAimAxisDriver(RebaseDriver::LeftHandPosition));

        // Commit keeps the other sample for aim-axis drivers only; the pass advances both samples.
        Registry registry{};
        ok &= expectEnum("insert aim", commit(registry, "ROCK_S", true, 100, translated(2.0f, 10.0f, 0.0f), RebaseDriver::LeftHandAimAxis, sample(translated(0, 10, 0)), sample(identity())), CommitResult::Inserted);
        ok &= expectEnum("insert full", commit(registry, "ROCK_F", false, 100, translated(1, 0, 0), RebaseDriver::RightHand, sample(identity()), sample(translated(0, 10, 0))), CommitResult::Inserted);
        ok &= expectTrue("aim claim keeps the other sample", find(registry, "ROCK_S", true)->otherDriverAtPublish.valid);
        ok &= expectFalse("full claim drops the other sample", find(registry, "ROCK_F", false)->otherDriverAtPublish.valid);
        DriverFrame passFrame{};
        passFrame.sequence = 3;
        passFrame.hands[handIndex(true)] = sample(translated(-10.0f, 0.0f, 0.0f)); // left hand now at -X of the right: axis turned 90 degrees
        passFrame.hands[handIndex(false)] = sample(identity());
        RebasePassPlan passPlan{};
        planRebasePass(registry, passFrame, passPlan);
        const RebasePassEntry* aimEntry = nullptr;
        for (std::size_t i = 0; i < passPlan.count; ++i) {
            if (tagView(registry.claims[passPlan.entries[i].claimIndex]) == "ROCK_S") {
                aimEntry = &passPlan.entries[i];
            }
        }
        ok &= expectTrue("aim entry planned", aimEntry != nullptr && aimEntry->moved);
        if (aimEntry) {
            ok &= expectNear("pass turned the seat", rotationDeltaDegrees(aimEntry->target, find(registry, "ROCK_S", true)->target), 90.0f, 0.01f);
            commitRebasePassEntry(registry, *aimEntry, passFrame);
            ok &= expectNear("pass advanced the other sample", find(registry, "ROCK_S", true)->otherDriverAtPublish.world.translate.x, 0.0f, 0.001f);
            ok &= expectNear("pass advanced the own sample", find(registry, "ROCK_S", true)->driverAtPublish.world.translate.x, -10.0f, 0.001f);
        }
    }

    // Driver frame lookup.
    {
        DriverFrame frame{};
        frame.hands[handIndex(false)] = sample(translated(1, 0, 0));
        frame.hands[handIndex(true)] = sample(translated(2, 0, 0));
        ok &= expectTrue("right driver", sampleForDriver(frame, RebaseDriver::RightHand)->world.translate.x == 1.0f);
        ok &= expectTrue("left driver", sampleForDriver(frame, RebaseDriver::LeftHand)->world.translate.x == 2.0f);
        ok &= expectTrue("static driver has no sample", sampleForDriver(frame, RebaseDriver::Static) == nullptr);
        ok &= expectTrue("driver for hand", driverForHand(true) == RebaseDriver::LeftHand && driverForHand(false) == RebaseDriver::RightHand);
    }

    // Re-anchor: a claim rebased by a predicted sample is re-expressed against the actual one and republished next pass.
    {
        Registry registry{};
        ok &= expectEnum("insert P", commit(registry, "ROCK_P", false, 100, translated(12.0f, 1.0f, 0.0f), RebaseDriver::RightHand, sample(translated(10.0f, 0.0f, 0.0f))), CommitResult::Inserted);
        ok &= expectEnum("insert static", commit(registry, "ROCK_W", true, 100, translated(3.0f, 0.0f, 0.0f), RebaseDriver::Static, {}), CommitResult::Inserted);
        const DriverFrame predicted{ .sequence = 2, .hands = { sample(translated(15.0f, 0.0f, 0.0f)), DriverSample{} } };
        RebasePassPlan plan{};
        planRebasePass(registry, predicted, plan);
        for (std::size_t i = 0; i < plan.count; ++i) {
            if (plan.entries[i].moved || plan.entries[i].keepOrder) {
                commitRebasePassEntry(registry, plan.entries[i], predicted);
            }
        }
        ok &= expectNear("rebased by the prediction", find(registry, "ROCK_P", false)->target.translate.x, 17.0f, 0.001f);

        const DriverFrame actual{ .sequence = 2, .hands = { sample(translated(14.0f, 0.0f, 0.0f)), DriverSample{} } };
        reanchorClaims(registry, actual);
        const Claim* reanchored = find(registry, "ROCK_P", false);
        ok &= expectNear("re-anchored target", reanchored->target.translate.x, 16.0f, 0.001f);
        ok &= expectNear("re-anchored sample", reanchored->driverAtPublish.world.translate.x, 14.0f, 0.001f);
        ok &= expectTrue("flagged for publish", reanchored->needsPublish);
        ok &= expectNear("static claim untouched", find(registry, "ROCK_W", true)->target.translate.x, 3.0f, 0.001f);
        ok &= expectFalse("static claim not flagged", find(registry, "ROCK_W", true)->needsPublish);

        RebasePassPlan stillPlan{};
        planRebasePass(registry, actual, stillPlan);
        const RebasePassEntry* entry = nullptr;
        for (std::size_t i = 0; i < stillPlan.count; ++i) {
            if (tagView(registry.claims[stillPlan.entries[i].claimIndex]) == "ROCK_P") {
                entry = &stillPlan.entries[i];
            }
        }
        ok &= expectTrue("published without driver motion", entry && entry->moved);
        if (entry) {
            ok &= expectNear("published target is the re-anchored one", entry->target.translate.x, 16.0f, 0.001f);
            commitRebasePassEntry(registry, *entry, actual);
        }
        ok &= expectFalse("flag cleared by the publish", find(registry, "ROCK_P", false)->needsPublish);
        reanchorClaims(registry, actual);
        ok &= expectFalse("re-anchor to the same sample does not flag", find(registry, "ROCK_P", false)->needsPublish);
        ok &= expectEnum("owner republish clears the flag", commit(registry, "ROCK_P", false, 100, translated(20.0f, 1.0f, 0.0f), RebaseDriver::RightHand, sample(translated(14.0f, 0.0f, 0.0f))), CommitResult::Updated);
        ok &= expectFalse("fresh publish not flagged", find(registry, "ROCK_P", false)->needsPublish);
    }

    // Rebase pass: oldest first, moved claims republish, and an equal-priority tie keeps its winner.
    {
        Registry registry{};
        const DriverSample base = sample(identity());
        ok &= expectEnum("A right 100 moving", commit(registry, "ROCK_A", false, 100, translated(1, 0, 0), RebaseDriver::RightHand, base), CommitResult::Inserted);
        ok &= expectEnum("B right 100 static newest", commit(registry, "ROCK_B", false, 100, translated(2, 0, 0), RebaseDriver::Static, {}), CommitResult::Inserted);
        ok &= expectEnum("C right 85 static", commit(registry, "ROCK_C", false, 85, translated(3, 0, 0), RebaseDriver::Static, {}), CommitResult::Inserted);
        ok &= expectEnum("D left 100 moving", commit(registry, "ROCK_D", true, 100, translated(4, 0, 0), RebaseDriver::LeftHand, base), CommitResult::Inserted);
        ok &= expectTrue("B wins the right tie before the pass", tagView(*winner(registry, false)) == "ROCK_B");

        DriverFrame frame{};
        frame.sequence = 7;
        frame.hands[handIndex(false)] = sample(translated(5, 0, 0));
        frame.hands[handIndex(true)] = sample(identity()); // left driver did not move

        RebasePassPlan plan{};
        planRebasePass(registry, frame, plan);
        ok &= expectTrue("every claim planned", plan.count == 4);
        ok &= expectTrue("oldest first", plan.entries[0].claimIndex == 0 && plan.entries[1].claimIndex == 1 && plan.entries[2].claimIndex == 2 && plan.entries[3].claimIndex == 3);
        ok &= expectTrue("A moved", plan.entries[0].moved && !plan.entries[0].keepOrder);
        ok &= expectNear("A rebased x", plan.entries[0].target.translate.x, 6.0f, 0.001f);
        ok &= expectTrue("B keeps order", !plan.entries[1].moved && plan.entries[1].keepOrder);
        ok &= expectNear("B target unchanged", plan.entries[1].target.translate.x, 2.0f, 0.001f);
        ok &= expectTrue("C untouched (different priority)", !plan.entries[2].moved && !plan.entries[2].keepOrder);
        ok &= expectTrue("D untouched (driver still)", !plan.entries[3].moved && !plan.entries[3].keepOrder);

        for (std::size_t i = 0; i < plan.count; ++i) {
            if (plan.entries[i].moved || plan.entries[i].keepOrder) {
                commitRebasePassEntry(registry, plan.entries[i], frame);
            }
        }
        ok &= expectTrue("B still wins the right tie after the pass", tagView(*winner(registry, false)) == "ROCK_B");
        ok &= expectNear("A claim advanced", find(registry, "ROCK_A", false)->target.translate.x, 6.0f, 0.001f);
        ok &= expectNear("A driver sample advanced", find(registry, "ROCK_A", false)->driverAtPublish.world.translate.x, 5.0f, 0.001f);
        ok &= expectTrue("A newer than C", find(registry, "ROCK_A", false)->publishOrder > find(registry, "ROCK_C", false)->publishOrder);

        // Second pass with no motion plans nothing.
        planRebasePass(registry, frame, plan);
        bool anyPublish = false;
        for (std::size_t i = 0; i < plan.count; ++i) {
            anyPublish = anyPublish || plan.entries[i].moved || plan.entries[i].keepOrder;
        }
        ok &= expectFalse("still pass publishes nothing", anyPublish);
    }

    // Fallback detection: confirmed once after two far frames, reset when following again, skipped under recoil.
    {
        Registry registry{};
        ok &= expectEnum("insert", commit(registry, "ROCK_F", true, 100, translated(0, 0, 0), RebaseDriver::LeftHand, sample(identity())), CommitResult::Inserted);
        Claim* claim = find(registry, "ROCK_F", true);
        ok &= expectEnum("following", observeFallback(*claim, translated(0.5f, 0, 0), true, false), FallbackObservation::Following);
        ok &= expectEnum("far frame 1", observeFallback(*claim, translated(10.0f, 0, 0), true, false), FallbackObservation::Suspected);
        ok &= expectEnum("far frame 2 confirms", observeFallback(*claim, translated(10.0f, 0, 0), true, false), FallbackObservation::Confirmed);
        ok &= expectEnum("far frame 3 already reported", observeFallback(*claim, translated(10.0f, 0, 0), true, false), FallbackObservation::Suspected);
        ok &= expectEnum("recoil frame skipped", observeFallback(*claim, translated(10.0f, 0, 0), true, true), FallbackObservation::NotApplicable);
        ok &= expectEnum("invalid wrist skipped", observeFallback(*claim, translated(10.0f, 0, 0), false, false), FallbackObservation::NotApplicable);
        ok &= expectEnum("following resets", observeFallback(*claim, translated(0.0f, 0, 0), true, false), FallbackObservation::Following);
        ok &= expectEnum("far again suspected", observeFallback(*claim, translated(10.0f, 0, 0), true, false), FallbackObservation::Suspected);
        ok &= expectEnum("far again confirms", observeFallback(*claim, translated(10.0f, 0, 0), true, false), FallbackObservation::Confirmed);
        // A per-frame republish keeps the episode; a fresh insert starts clean.
        ok &= expectEnum("republish keeps episode", commit(registry, "ROCK_F", true, 100, translated(0, 0, 0), RebaseDriver::LeftHand, sample(identity())), CommitResult::Updated);
        ok &= expectTrue("episode preserved", claim->fallbackReported && claim->fallbackFrames == kFallbackConfirmFrames);
        ok &= expectTrue("remove F", remove(registry, "ROCK_F", true));
        ok &= expectEnum("reinsert F", commit(registry, "ROCK_F", true, 100, translated(0, 0, 0), RebaseDriver::LeftHand, sample(identity())), CommitResult::Inserted);
        claim = find(registry, "ROCK_F", true);
        ok &= expectTrue("fresh insert clean", claim && !claim->fallbackReported && claim->fallbackFrames == 0);
        // Rotation-only fallback.
        Claim rotatedClaim = *claim;
        rotatedClaim.fallbackFrames = 0;
        rotatedClaim.fallbackReported = false;
        ok &= expectEnum("rotation fallback suspected", observeFallback(rotatedClaim, yawed(40.0f), true, false), FallbackObservation::Suspected);
    }

    // A target whose basis is not a rotation is refused; drift is accepted.
    {
        Registry rotationRegistry{};
        RE::NiTransform stretched = translated(1.0f, 2.0f, 3.0f);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                stretched.rotate.entry[row][column] *= 1.2f;
            }
        }
        ok &= expectEnum("stretched target refused", commit(rotationRegistry, "ROCK_S", false, 100, stretched, RebaseDriver::Static, sample(identity())), CommitResult::InvalidTarget);
        RE::NiTransform drifted = translated(1.0f, 2.0f, 3.0f);
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                drifted.rotate.entry[row][column] *= 1.0003f;
            }
        }
        ok &= expectEnum("drifted target accepted", commit(rotationRegistry, "ROCK_S", false, 100, drifted, RebaseDriver::Static, sample(identity())), CommitResult::Inserted);
    }

    // End-of-frame presentation: carry the rendered chain by the change ROCK
    // made to its target since FRIK consumed it.
    {
        Registry registry{};
        const RE::NiTransform consumedTarget = translated(10.0f, 0.0f, 0.0f);
        const RE::NiTransform newTarget = yawed(5.0f, 12.0f, 0.0f, 0.0f);
        ok &= expectEnum("presentation claim", commit(registry, "ROCK_P", false, 100, consumedTarget, RebaseDriver::RightHand, sample(identity())), CommitResult::Inserted);
        const ConsumedTarget consumed = snapshotConsumedTarget(registry, false);
        ok &= expectTrue("consumed snapshot valid", consumed.valid);
        ok &= expectFalse("no consumed for left", snapshotConsumedTarget(registry, true).valid);

        PresentationPlan plan = planPresentation(consumed, winner(registry, false), consumedTarget, true);
        ok &= expectEnum("unchanged", plan.decision, PresentationDecision::Unchanged);

        // ROCK republished the seat 2 gu further and 5 deg turned.
        ok &= expectEnum("presentation republish", commit(registry, "ROCK_P", false, 100, newTarget, RebaseDriver::RightHand, sample(identity())), CommitResult::Updated);
        plan = planPresentation(consumed, winner(registry, false), consumedTarget, true);
        ok &= expectEnum("present", plan.decision, PresentationDecision::Present);
        ok &= expectNear("present translation", plan.translationGameUnits, 2.0f, 0.001f);
        ok &= expectNear("present rotation", plan.rotationDegrees, 5.0f, 0.01f);
        const RE::NiTransform carried = rock::transform_math::composeTransforms(plan.delta, consumedTarget);
        ok &= expectNear("carried wrist lands on the new target", translationDeltaGameUnits(carried, newTarget), 0.0f, 0.002f);
        ok &= expectNear("carried wrist rotation", rotationDeltaDegrees(carried, newTarget), 0.0f, 0.02f);
        ok &= expectTrue("presentation delta orthonormal", rock::transform_math::storedRotationOrthonormalityError(plan.delta.rotate) < 1e-5);

        // FRIK rendered the tracked hand instead (fallback or kick): left as drawn.
        plan = planPresentation(consumed, winner(registry, false), translated(20.0f, 0.0f, 0.0f), true);
        ok &= expectEnum("not following", plan.decision, PresentationDecision::NotFollowing);
        plan = planPresentation(consumed, winner(registry, false), consumedTarget, false);
        ok &= expectEnum("wrist unreadable", plan.decision, PresentationDecision::NoClaim);

        // A seat change beyond a rigid carry is left to FRIK's next solve.
        ok &= expectEnum("presentation jump", commit(registry, "ROCK_P", false, 100, translated(40.0f, 0.0f, 0.0f), RebaseDriver::RightHand, sample(identity())), CommitResult::Updated);
        plan = planPresentation(consumed, winner(registry, false), consumedTarget, true);
        ok &= expectEnum("too large", plan.decision, PresentationDecision::TooLarge);

        // Claim cleared during ROCK's frame, or never consumed: nothing to present.
        ok &= expectTrue("presentation remove", remove(registry, "ROCK_P", false));
        plan = planPresentation(consumed, winner(registry, false), consumedTarget, true);
        ok &= expectEnum("cleared", plan.decision, PresentationDecision::NoClaim);
        ok &= expectEnum("never consumed", planPresentation(ConsumedTarget{}, winner(registry, false), consumedTarget, true).decision, PresentationDecision::NoClaim);
    }

    // A shared object carries both fixed hand seats through one rigid
    // delta. Changing controller separation must not slide either seat.
    for (const auto driver : {RebaseDriver::RightObjectPivot, RebaseDriver::LeftObjectPivot}) {
        Claim front{}, rear{};
        front.valid = rear.valid = true;
        front.driver = rear.driver = driver;
        front.target = translated(8.0f, 2.0f, 0.0f);
        rear.target = translated(0.0f, 1.0f, 0.0f);
        front.driverAtPublish = rear.driverAtPublish = sample(identity());
        front.otherDriverAtPublish = rear.otherDriverAtPublish = sample(translated(10.0f, 0.0f, 0.0f));
        const auto frontAlong = planRebase(front, sample(identity()), sample(translated(30.0f, 0.0f, 0.0f)));
        const auto rearAlong = planRebase(rear, sample(identity()), sample(translated(30.0f, 0.0f, 0.0f)));
        ok &= expectNear("shared forward hand cannot slide along object", translationDeltaGameUnits(frontAlong.target, front.target), 0.0f, 0.001f);
        ok &= expectNear("shared rear hand cannot slide along object", translationDeltaGameUnits(rearAlong.target, rear.target), 0.0f, 0.001f);
        const auto wristTurn = planRebase(front, sample(yawed(60.0f)), sample(translated(10.0f, 0.0f, 0.0f)));
        ok &= expectNear("two-hand aim cancels carrier yaw across the grip axis", translationDeltaGameUnits(wristTurn.target, front.target), 0.0f, 0.001f);
        ok &= expectNear("two-hand aim keeps grip orientation through carrier yaw", rotationDeltaDegrees(wristTurn.target, front.target), 0.0f, 0.01f);
        const auto movedPrimary = sample(translated(3.0f, 4.0f, 5.0f));
        const auto turnedSupport = sample(translated(3.0f, 29.0f, 5.0f));
        const auto frontTurn = planRebase(front, movedPrimary, turnedSupport);
        const auto rearTurn = planRebase(rear, movedPrimary, turnedSupport);
        ok &= expectNear("front seat turns about primary x", frontTurn.target.translate.x, 1.0f, 0.001f);
        ok &= expectNear("front seat turns about primary y", frontTurn.target.translate.y, 12.0f, 0.001f);
        ok &= expectNear("rear seat turns about primary x", rearTurn.target.translate.x, 2.0f, 0.001f);
        ok &= expectNear("rear seat turns about primary y", rearTurn.target.translate.y, 4.0f, 0.001f);
        ok &= expectNear("shared seats keep separation", translationDeltaGameUnits(frontTurn.target, rearTurn.target),
            translationDeltaGameUnits(front.target, rear.target), 0.001f);
        ok &= expectNear("shared seats keep relative rotation", rotationDeltaDegrees(frontTurn.target, rearTurn.target), 0.0f, 0.01f);
        ok &= expectNear("shared seats follow locomotion z", frontTurn.target.translate.z, 5.0f, 0.001f);
        DriverFrame frame{};
        frame.hands[0] = sample(identity()); frame.hands[1] = sample(translated(10.0f, 0.0f, 0.0f));
        const auto primaryIndex = driver == RebaseDriver::LeftObjectPivot ? 1u : 0u;
        ok &= expectTrue("pivot driver samples its primary hand", sampleForDriver(frame, driver) == &frame.hands[primaryIndex]);
        ok &= expectTrue("pivot driver samples the other hand", otherHandSampleForDriver(frame, driver) == &frame.hands[1u - primaryIndex]);
    }

    if (!ok) {
        std::printf("HandWorldClaimRegistryPolicyTests FAILED\n");
        return 1;
    }
    std::printf("HandWorldClaimRegistryPolicyTests passed\n");
    return 0;
}

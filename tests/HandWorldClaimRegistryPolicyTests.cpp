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
        Claim noPublishSample = *claim;
        noPublishSample.driverAtPublish.valid = false;
        ok &= expectFalse("invalid publish sample", planRebase(noPublishSample, rotated).republish);
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

    if (!ok) {
        std::printf("HandWorldClaimRegistryPolicyTests FAILED\n");
        return 1;
    }
    std::printf("HandWorldClaimRegistryPolicyTests passed\n");
    return 0;
}

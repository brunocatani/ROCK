#include "physics-interaction/visual/HandWorldClaimRegistryPolicy.h"

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

    DriverSample sample(const RE::NiTransform& world)
    {
        return DriverSample{ .world = world, .valid = true };
    }
}

int main()
{
    bool ok = true;

    // Commit, update in place, tie-break by registration order (a republish keeps its place), remove.
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
        ok &= expectTrue("re-published A keeps its place, B still wins the tie", top && tagView(*top) == "ROCK_B");
        ok &= expectTrue("claim count is two", claimCount(registry) == 2);

        ok &= expectEnum("higher priority", commit(registry, "ROCK_C", false, 110, translated(4, 0, 0), RebaseDriver::Static, {}), CommitResult::Inserted);
        top = winner(registry, false);
        ok &= expectTrue("priority beats order", top && tagView(*top) == "ROCK_C");

        ok &= expectTrue("remove C", remove(registry, "ROCK_C", false));
        ok &= expectFalse("remove C twice", remove(registry, "ROCK_C", false));
        ok &= expectFalse("remove wrong hand", remove(registry, "ROCK_A", true));
        top = winner(registry, false);
        ok &= expectTrue("B still wins after C removed", top && tagView(*top) == "ROCK_B");

        // Clear and set again: the tag registers anew, takes the newest order and wins the tie.
        ok &= expectTrue("clear A", remove(registry, "ROCK_A", false));
        ok &= expectEnum("set A again", commit(registry, "ROCK_A", false, 100, translated(5, 0, 0), RebaseDriver::RightHand, sample(identity())), CommitResult::Inserted);
        top = winner(registry, false);
        ok &= expectTrue("re-registered A wins the tie", top && tagView(*top) == "ROCK_A");
        const Claim* claimB = find(registry, "ROCK_B", false);
        ok &= expectTrue("re-registered A is newer than B", top && claimB && top->publishOrder > claimB->publishOrder);
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
        clearAll(registry);
        ok &= expectTrue("cleared", claimCount(registry) == 0);
        ok &= expectTrue("publish order reset", registry.nextPublishOrder == 1);
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

    if (!ok) {
        std::printf("HandWorldClaimRegistryPolicyTests FAILED\n");
        return 1;
    }
    std::printf("HandWorldClaimRegistryPolicyTests passed\n");
    return 0;
}

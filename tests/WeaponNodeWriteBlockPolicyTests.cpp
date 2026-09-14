#include "physics-interaction/weapon/grip/WeaponNodeWriteBlockPolicy.h"

#include <cstdio>

namespace
{
    using namespace rock::weapon_node_write_block_policy;

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
        return expectTrue(label, !value);
    }

    bool expectEqual(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }
        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }
}

int main()
{
    bool ok = true;

    // Frames since write: a write resets, silence counts up, never-written saturates.
    ok &= expectEqual("write resets", advanceFramesSinceWrite(57, true), 0);
    ok &= expectEqual("silence counts", advanceFramesSinceWrite(0, false), 1);
    ok &= expectEqual("never written stays saturated", advanceFramesSinceWrite(kNeverWritten, false), kNeverWritten);
    ok &= expectEqual("never written then written", advanceFramesSinceWrite(kNeverWritten, true), 0);

    // Hold window: a write holds the block for kHoldFramesAfterWrite frames, then releases.
    ok &= expectTrue("held on the write frame", shouldHoldWriteBlock({ .framesSinceWrite = 0 }));
    ok &= expectTrue("held inside the window", shouldHoldWriteBlock({ .framesSinceWrite = kHoldFramesAfterWrite - 1 }));
    ok &= expectFalse("released after the window", shouldHoldWriteBlock({ .framesSinceWrite = kHoldFramesAfterWrite }));
    ok &= expectFalse("never written is not held", shouldHoldWriteBlock({}));

    // Ownership states hold regardless of writes.
    ok &= expectTrue("two-hand authority holds", shouldHoldWriteBlock({ .ownsWeaponTransform = true }));
    ok &= expectTrue("weapon return holds", shouldHoldWriteBlock({ .weaponReturnActive = true }));
    ok &= expectTrue("left carry holds", shouldHoldWriteBlock({ .leftCarryActive = true }));
    ok &= expectTrue("one-hand recoil envelope holds", shouldHoldWriteBlock({ .oneHandRecoilActive = true }));

    // Recoil writes keep a longer tail than other writes.
    ok &= expectTrue("recoil tail holds past the short window", shouldHoldWriteBlock({ .framesSinceRecoilWrite = kHoldFramesAfterWrite }));
    ok &= expectTrue("recoil tail holds to its last frame", shouldHoldWriteBlock({ .framesSinceRecoilWrite = kHoldFramesAfterRecoilWrite - 1 }));
    ok &= expectFalse("recoil tail releases after its window", shouldHoldWriteBlock({ .framesSinceRecoilWrite = kHoldFramesAfterRecoilWrite }));

    // The block is a state: one silent frame inside the window does not release it.
    std::uint32_t frames = advanceFramesSinceWrite(kNeverWritten, true);
    bool heldEveryFrame = true;
    for (std::uint32_t frame = 0; frame < kHoldFramesAfterWrite; ++frame) {
        heldEveryFrame = heldEveryFrame && shouldHoldWriteBlock({ .framesSinceWrite = frames });
        frames = advanceFramesSinceWrite(frames, false);
    }
    ok &= expectTrue("held across the whole window", heldEveryFrame);
    ok &= expectFalse("released at the end of the window", shouldHoldWriteBlock({ .framesSinceWrite = frames }));

    if (!ok) {
        std::printf("WeaponNodeWriteBlockPolicyTests failed\n");
        return 1;
    }
    std::printf("WeaponNodeWriteBlockPolicyTests passed\n");
    return 0;
}

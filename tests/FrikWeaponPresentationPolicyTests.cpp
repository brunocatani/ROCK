#include "physics-interaction/weapon/grip/FrikWeaponPresentationPolicy.h"

#include <cmath>
#include <cstdio>

namespace
{
    using namespace rock::frik_weapon_presentation_policy;

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

    RE::NiTransform translated(float x, float y, float z)
    {
        RE::NiTransform t{};
        t.rotate.entry[0][0] = 1.0f;
        t.rotate.entry[1][1] = 1.0f;
        t.rotate.entry[2][2] = 1.0f;
        t.translate = RE::NiPoint3{ x, y, z };
        t.scale = 1.0f;
        return t;
    }

    bool sameLocal(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        return a.translate.x == b.translate.x && a.translate.y == b.translate.y && a.translate.z == b.translate.z;
    }

    constexpr NodeIdentity kRightHandWeapon{ .node = 0x1000, .parent = 0x2000, .modelRoot = 0x4000, .inPowerArmor = false };
}

int main()
{
    bool ok = true;

    const RE::NiTransform offsetLocal = translated(3.0f, 9.0f, -1.0f);
    const RE::NiTransform reglueLocal = translated(4.4f, -1.9f, -3.1f);

    // Capture: only a local FRIK wrote (visible, not blocked) becomes the latch.
    const OffsetLatch written = captureOffsetLatch({}, { .identity = kRightHandWeapon, .local = offsetLocal, .nodeVisible = true });
    ok &= expectTrue("FRIK write is latched", written.valid && sameLocal(written.local, offsetLocal));

    const OffsetLatch whileHeld = captureOffsetLatch(written, { .identity = kRightHandWeapon, .local = reglueLocal, .nodeVisible = true, .writeBlockHeld = true });
    ok &= expectTrue("block held keeps the latch", whileHeld.valid && sameLocal(whileHeld.local, offsetLocal));

    const OffsetLatch whileHidden = captureOffsetLatch(written, { .identity = kRightHandWeapon, .local = reglueLocal, .nodeVisible = false });
    ok &= expectTrue("hidden weapon keeps the latch", whileHidden.valid && sameLocal(whileHidden.local, offsetLocal));

    NodeIdentity leftCarry = kRightHandWeapon;
    leftCarry.parent = 0x3000;
    const OffsetLatch reparented = captureOffsetLatch(written, { .identity = leftCarry, .local = reglueLocal, .nodeVisible = true, .writeBlockHeld = true });
    ok &= expectTrue("a held reparent keeps the same weapon's latch", reparented.valid && reparented.identity == kRightHandWeapon);

    NodeIdentity swapped = kRightHandWeapon;
    swapped.modelRoot = 0x5000;
    ok &= expectFalse("a swapped model drops the latch",
        captureOffsetLatch(written, { .identity = swapped, .local = reglueLocal, .nodeVisible = true, .writeBlockHeld = true }).valid);
    NodeIdentity powerArmor = kRightHandWeapon;
    powerArmor.inPowerArmor = true;
    ok &= expectFalse("entering power armor drops the latch",
        captureOffsetLatch(written, { .identity = powerArmor, .local = reglueLocal, .nodeVisible = false }).valid);
    ok &= expectFalse("an invalid identity drops the latch", captureOffsetLatch(written, { .identity = {}, .local = offsetLocal, .nodeVisible = true }).valid);

    RE::NiTransform nonFinite = offsetLocal;
    nonFinite.translate.x = std::nanf("");
    ok &= expectFalse("a non-finite write is not latched",
        captureOffsetLatch({}, { .identity = kRightHandWeapon, .local = nonFinite, .nodeVisible = true }).valid);
    ok &= expectFalse("a non-finite FRIK write drops the previous latch",
        captureOffsetLatch(written, { .identity = kRightHandWeapon, .local = nonFinite, .nodeVisible = true }).valid);

    NodeIdentity otherNode = kRightHandWeapon;
    otherNode.node = 0x6000;
    ok &= expectFalse("another node drops the latch",
        captureOffsetLatch(written, { .identity = otherNode, .local = reglueLocal, .nodeVisible = true, .writeBlockHeld = true }).valid);
    const OffsetLatch hiddenReparented = captureOffsetLatch(written, { .identity = leftCarry, .local = reglueLocal, .nodeVisible = false });
    ok &= expectTrue("a hidden reparented weapon keeps the latch", hiddenReparented.valid && sameLocal(hiddenReparented.local, offsetLocal));

    // Present: a node FRIK re-glued this frame, with a latch for the same parent and model.
    ok &= expectTrue("present while FRIK owns the node", shouldPresent(written, { .identity = kRightHandWeapon, .nodeVisible = true }));
    ok &= expectFalse("no present while ROCK owns the live pose",
        shouldPresent(written, { .identity = kRightHandWeapon, .nodeVisible = true, .rockOwnsLivePose = true }));
    ok &= expectFalse("no present for a hidden weapon", shouldPresent(written, { .identity = kRightHandWeapon, .nodeVisible = false }));
    ok &= expectFalse("no present under another parent", shouldPresent(written, { .identity = leftCarry, .nodeVisible = true }));
    ok &= expectFalse("no present on the swap frame", shouldPresent(written, { .identity = swapped, .nodeVisible = true }));
    ok &= expectFalse("no present for another node", shouldPresent(written, { .identity = otherNode, .nodeVisible = true }));
    ok &= expectFalse("no present across a power armor change", shouldPresent(written, { .identity = powerArmor, .nodeVisible = true }));
    ok &= expectFalse("no present without a latch", shouldPresent({}, { .identity = kRightHandWeapon, .nodeVisible = true }));

    // Restore: FRIK gets its re-glue local back whenever it rewrites the node next.
    ok &= expectTrue("restore while the block is released", shouldRestore(true, false));
    ok &= expectFalse("the node keeps its pose while the block is engaged", shouldRestore(true, true));
    ok &= expectFalse("nothing presented, nothing restored", shouldRestore(false, false));

    if (!ok) {
        std::printf("FrikWeaponPresentationPolicyTests failed\n");
        return 1;
    }
    std::printf("FrikWeaponPresentationPolicyTests passed\n");
    return 0;
}

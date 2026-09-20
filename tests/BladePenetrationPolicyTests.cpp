#include "physics-interaction/weapon/BladePenetrationPolicy.h"

#include <cstdio>
#include <limits>

namespace
{
    bool check(bool condition, const char* description)
    {
        if (!condition) std::printf("FAIL: %s\n", description);
        return condition;
    }

    bool near(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const auto delta = rock::blade_penetration::difference(a, b);
        return rock::blade_penetration::dot(delta, delta) < 0.00001f;
    }
}

int main()
{
    using namespace rock;
    using namespace blade_penetration;
    bool ok = true;
    const auto identity = transform_math::makeIdentityTransform<RE::NiTransform>();
    const Blade blade{ { 0, 12, 0 }, { 0, 1, 0 }, true };
    auto physical = identity;
    physical.translate = { 100, 200, 300 };
    auto requested = physical;
    requested.translate.y += 1;
    const auto tip = transform_math::localPointToWorld(physical, blade.tipLocal);
    ok &= check(evaluateEntry(blade, physical, requested, tip).accepted, "tip-first forward pressure enters");
    requested.translate.y -= 2;
    ok &= check(!evaluateEntry(blade, physical, requested, tip).accepted, "withdrawing cannot initiate entry");
    requested = physical;
    requested.translate.x += 3;
    requested.translate.y += 1;
    ok &= check(!evaluateEntry(blade, physical, requested, tip).accepted, "sideways push cannot initiate entry");
    requested = physical;
    requested.translate.y += 1;
    ok &= check(!evaluateEntry(blade, physical, requested, physical.translate).accepted, "hilt contact cannot enter");
    ok &= check(!evaluateEntry({}, physical, requested, tip).accepted, "unidentified blade remains solid");
    auto turned = requested;
    turned.rotate.entry[0][0] = 0; turned.rotate.entry[0][1] = 1;
    turned.rotate.entry[1][0] = -1; turned.rotate.entry[1][1] = 0;
    ok &= check(!evaluateEntry(blade, physical, turned, tip).accepted, "turning sideways at the surface cannot enter");

    auto target = identity;
    target.translate = { 100, 200, 300 };
    const auto entry = transform_math::composeTransforms(transform_math::invertTransform(target), physical);
    requested = physical;
    requested.translate = { 150, 220, 250 };
    const auto limited = guide(blade, entry, target, requested);
    ok &= check(limited.valid && near(limited.weaponWorld.translate, { 100, 205, 300 }),
        "depth stop rejects perpendicular movement on both axes");
    requested = physical;
    requested.translate.y -= 1;
    ok &= check(near(guide(blade, entry, target, requested).weaponWorld.translate, requested.translate), "partial withdrawal slides freely");
    ok &= check(!withdrawn(-3, 1) && !withdrawn(1, -3), "release requires both intent and physical tip to clear the entry");
    ok &= check(withdrawn(-3, -2), "withdrawal restores collision only after clearance");

    // Rotate the NPC and move it. Both the insertion line and captured blade
    // orientation must follow that BODY; world-up must not remain the guide.
    target.rotate = turned.rotate;
    target.translate = { -200, 500, 20 };
    requested = transform_math::composeTransforms(target, entry);
    requested.translate.x -= 3;
    requested.translate.z += 9;
    const auto moved = guide(blade, entry, target, requested);
    ok &= check(moved.valid && near(moved.weaponWorld.translate, { -203, 500, 20 }), "moving NPC carries the entry line");
    ok &= check(std::abs(moved.weaponWorld.rotate.entry[1][0] + 1) < 0.0001f, "NPC rotation carries the embedded blade orientation");

    // Readback is BODY, not COM. Test a nonzero center, rotation and scale so
    // a transposed readback or unscaled center cannot pass an identity test.
    auto root = target;
    root.scale = 2;
    const RE::NiPoint3 center{ 2, 7, 1 };
    auto body = root;
    body.translate = transform_math::localPointToWorld(root, center);
    body.scale = 1;
    const auto reconstructed = weaponFromBody(body, center, root.scale);
    ok &= check(near(reconstructed.translate, root.translate) &&
        near(transform_math::localPointToWorld(reconstructed, blade.tipLocal), transform_math::localPointToWorld(root, blade.tipLocal)),
        "BODY readback preserves scaled rotated blade geometry");

    requested.translate.x = std::numeric_limits<float>::quiet_NaN();
    ok &= check(!guide(blade, entry, target, requested).valid, "invalid pose cannot drive an embedded blade");
    ok &= check(!withdrawn(-3, std::numeric_limits<float>::quiet_NaN()), "invalid physical depth cannot authorize release");
    return ok ? 0 : 1;
}

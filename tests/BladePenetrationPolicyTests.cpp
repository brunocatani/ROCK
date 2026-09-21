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
    for (const auto formId : { 0x000FDC81u, 0x000913CAu, 0x00062AA3u, 0x00033FE0u, 0x00147BE4u, 0x00143AB5u, 0x000FA2FBu }) {
        ok &= check(family(formId) != Family::None, "standard swords and knives have blade profiles");
    }
    ok &= check(family(0x0008C14Du) == Family::None && family(0x00185D25u) == Family::None &&
        family(0x0008E736u) == Family::None && family(0x010FDC81u) == Family::None,
        "blunt melee and unknown forms cannot inherit blade penetration");
    ok &= check(sourceProfile(Family::ChineseSword, "SerratedBlade001:2") &&
        !sourceProfile(Family::ChineseSword, "SerratedBlade001:0") &&
        !sourceProfile(Family::Shishkebab, "TritiumDot:0") &&
        !sourceProfile(Family::Switchblade, "SerratedBlade_Blade:0"),
        "physical blades are distinguished from shock attachments and effect shells");
    const auto* machete = sourceProfile(Family::Machete, "Blade:1");
    ok &= check(machete && matchesSourceBounds(*machete, { -3.6171875f, -2.595703125f, -0.347900391f },
        { 2.322265625f, 37.125f, 0.347167969f }), "measured vanilla machete geometry qualifies");
    ok &= check(machete && !matchesSourceBounds(*machete, { -3.6171875f, -2.595703125f, -0.347900391f },
        { 2.322265625f, 50.0f, 0.347167969f }), "unverified mesh replacement stays solid");

    const float knifeDepth = maximumDepth(blade, { 0, 0, 0 }, 1.0f, 70.0f);
    ok &= check(std::abs(knifeDepth - 9.9f) < 0.0001f, "knife depth leaves three centimeters ahead of grip");
    const Blade sword{ { 0, 60, 0 }, { 0, 1, 0 }, true };
    const float swordDepth = maximumDepth(sword, { 0, 0, 0 }, 1.0f, 70.0f);
    ok &= check(std::abs(swordDepth - 57.9f) < 0.0001f, "long blade has no prototype depth cap");
    ok &= check(std::abs(maximumDepth(sword, { 8, 40, -3 }, 1.0f, 70.0f) - 17.9f) < 0.0001f,
        "moved grip changes depth by axial distance without counting lateral offset");
    ok &= check(std::abs(maximumDepth(blade, { 0, 0, 0 }, 2.0f, 70.0f) - 21.9f) < 0.0001f &&
        std::abs(maximumDepth(blade, { 0, 0, 0 }, 1.0f, 100.0f) - 9.0f) < 0.0001f,
        "geometry scale and engine distance conversion preserve physical hand clearance");
    const Blade rotatedBlade{ { -12, 0, 0 }, { -1, 0, 0 }, true };
    ok &= check(std::abs(maximumDepth(rotatedBlade, { -2, 4, 0 }, 1.0f, 70.0f) - 7.9f) < 0.0001f,
        "depth uses transformed blade axis rather than weapon local Y");
    ok &= check(maximumDepth(blade, { 0, 11, 0 }, 1.0f, 70.0f) == 0.0f &&
        maximumDepth(blade, { 0, 13, 0 }, 1.0f, 70.0f) == 0.0f &&
        maximumDepth({}, {}, 1.0f, 70.0f) == 0.0f &&
        maximumDepth(blade, {}, -1.0f, 70.0f) == 0.0f,
        "missing blade or hand at the tip cannot initiate penetration");
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
    const auto limited = guide(blade, entry, target, requested, knifeDepth);
    ok &= check(limited.valid && near(limited.weaponWorld.translate, { 100, 209.9f, 300 }),
        "depth stop rejects perpendicular movement on both axes");
    requested.translate.y = 300;
    const auto swordLimited = guide(sword, entry, target, requested, swordDepth);
    ok &= check(swordLimited.valid && near(swordLimited.weaponWorld.translate, { 100, 257.9f, 300 }),
        "sword slides through its full usable length with sideways movement locked");
    const auto movedGripDepth = maximumDepth(sword, { 0, 40, 0 }, 1.0f, 70.0f);
    ok &= check(near(guide(sword, entry, target, requested, movedGripDepth).weaponWorld.translate, { 100, 217.9f, 300 }),
        "updated grip shortens the stop without changing the captured entry line");
    requested = physical;
    requested.translate.y -= 1;
    ok &= check(near(guide(blade, entry, target, requested, knifeDepth).weaponWorld.translate, requested.translate), "partial withdrawal slides freely");
    ok &= check(!withdrawn(-3, 1) && !withdrawn(1, -3), "release requires both intent and physical tip to clear the entry");
    ok &= check(withdrawn(-3, -2), "withdrawal restores collision only after clearance");

    // Rotate the NPC and move it. Both the insertion line and captured blade
    // orientation must follow that BODY; world-up must not remain the guide.
    target.rotate = turned.rotate;
    target.translate = { -200, 500, 20 };
    requested = transform_math::composeTransforms(target, entry);
    requested.translate.x -= 3;
    requested.translate.z += 9;
    const auto moved = guide(blade, entry, target, requested, knifeDepth);
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
    ok &= check(!guide(blade, entry, target, requested, knifeDepth).valid, "invalid pose cannot drive an embedded blade");
    ok &= check(!guide(blade, entry, target, physical, 0.0f).valid &&
        !guide(blade, entry, target, physical, std::numeric_limits<float>::quiet_NaN()).valid &&
        maximumDepth(blade, requested.translate, 1.0f, 70.0f) == 0.0f,
        "invalid grip or depth fails closed");
    ok &= check(!withdrawn(-3, std::numeric_limits<float>::quiet_NaN()), "invalid physical depth cannot authorize release");
    return ok ? 0 : 1;
}

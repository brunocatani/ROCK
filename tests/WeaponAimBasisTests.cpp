#include "physics-interaction/weapon/WeaponAimBasis.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"

#include <cstdio>
#include <limits>

namespace
{
    bool expect(const char* label, bool result)
    {
        if (!result) std::printf("FAILED: %s\n", label);
        return result;
    }

    bool near(float a, float b) { return std::fabs(a - b) < 0.00002f; }
}

int main()
{
    using namespace rock;
    bool ok = true;
    RE::NiTransform controller{};
    controller.scale = 1.0f;
    controller.translate = { -335.86664f, -341.83038f, 107.02197f };
    // Correlated pre-FRIK controller and native driver sample, 2026-09-17.
    const float controllerRows[3][3] = {
        { 0.9608731f, 0.2424956f, 0.1338615f },
        { -0.1981389f, 0.2640419f, 0.9439402f },
        { 0.1935563f, -0.9335299f, 0.3017586f },
    };
    const float driverRows[3][3] = {
        { 0.9608731f, 0.2424956f, 0.1338615f },
        { -0.2679592f, 0.9361829f, 0.2275077f },
        { -0.0701493f, -0.2544755f, 0.9645317f },
    };
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) controller.rotate.entry[r][c] = controllerRows[r][c];
    RE::NiTransform weapon{};
    ok &= expect("controller basis reproduces the healthy native aim", weapon_aim_basis::tryResolveWorld(controller, 1.0f, weapon));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) ok &= expect("native rotation sample", near(weapon.rotate.entry[r][c], driverRows[r][c]));

    // Equipped and loose paths share the same aim; the authored palm seat
    // changes only placement, at either model scale and any world position.
    for (const float scale : { 1.0f, 0.865347f }) {
        RE::NiTransform aim{};
        ok &= expect("model scale accepted", weapon_aim_basis::tryResolveWorld(controller, scale, aim));
        const RE::NiPoint3 grip{ 0.815f, -0.734f, -2.679f };
        const RE::NiPoint3 palm{ -356.1f, -350.0f, 105.2f };
        const auto seated = authored_weapon_grip_capture_policy::resolveAuthoredPrimaryWeaponWorldPositionOnly(
            aim, grip, palm, [](const auto& t, const auto& p) { return transform_math::localPointToWorld(t, p); });
        const auto actualPalm = transform_math::localPointToWorld(seated, grip);
        ok &= expect("authored grip lands at the physical palm", near(actualPalm.x, palm.x) && near(actualPalm.y, palm.y) && near(actualPalm.z, palm.z));
        ok &= expect("model scale preserved", seated.scale == scale);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c) ok &= expect("seating preserves controller aim", seated.rotate.entry[r][c] == weapon.rotate.entry[r][c]);
    }
    RE::NiTransform moved = controller;
    moved.translate = { 9500.0f, -2400.0f, 600.0f };
    RE::NiTransform movedAim{};
    ok &= expect("player translation accepted", weapon_aim_basis::tryResolveWorld(moved, 1.0f, movedAim));
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c) ok &= expect("player translation cannot rotate aim", movedAim.rotate.entry[r][c] == weapon.rotate.entry[r][c]);
    ok &= expect("invalid scale rejected", !weapon_aim_basis::tryResolveWorld(controller, 0.0f, weapon));
    controller.rotate.entry[0][0] = std::numeric_limits<float>::quiet_NaN();
    ok &= expect("invalid controller rejected", !weapon_aim_basis::tryResolveWorld(controller, 1.0f, weapon));
    return ok ? 0 : 1;
}

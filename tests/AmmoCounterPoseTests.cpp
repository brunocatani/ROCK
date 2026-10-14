#include "physics-interaction/visual/AmmoCounterPose.h"

#include <cmath>
#include <cstdio>

namespace
{
    using namespace rock::ammo_counter_pose;

    bool expect(const char* message, const bool condition)
    {
        if (!condition) {
            std::fprintf(stderr, "FAILED: %s\n", message);
        }
        return condition;
    }

    bool near(float a, float b) { return std::abs(a - b) < 0.0001f; }
}

int main()
{
    bool ok = true;
    // Weapon aimed along world +Y, its +X world right, +Z up. A right-hand
    // weapon is held right of the head, a left-hand one left of it.
    const Point weaponRight{ 1.0f, 0.0f, 0.0f };
    const Point weaponUp{ 0.0f, 0.0f, 1.0f };
    const Point rightPalm{ 20.0f, 30.0f, 0.0f };
    const Point leftPalm{ -20.0f, 30.0f, 0.0f };
    const Point head{ 0.0f, 0.0f, 10.0f };

    const auto right = resolve(rightPalm, weaponRight, weaponUp, false, head);
    ok &= expect("a right-hand counter resolves", right.valid);
    ok &= expect("a right-hand counter faces the weapon's left, the palm side", near(right.front[0], -1.0f));
    ok &= expect("it sits off the palm side", near(right.loaded.center[0], rightPalm[0] - kOutward));

    const auto left = resolve(leftPalm, weaponRight, weaponUp, true, head);
    ok &= expect("a left-hand counter faces the weapon's right, the mirrored palm side", near(left.front[0], 1.0f));
    ok &= expect("it sits off the mirrored palm side", near(left.loaded.center[0], leftPalm[0] + kOutward));
    ok &= expect("both counters stay upright with the weapon", near(right.up[2], 1.0f) && near(left.up[2], 1.0f));
    ok &= expect("both counters read left to right toward the viewer",
        near(right.right[1], -1.0f) && near(left.right[1], 1.0f));
    ok &= expect("loaded rounds sit left of the reserve on both hands",
        right.loaded.center[1] > right.reserve.center[1] && left.loaded.center[1] < left.reserve.center[1]);

    ok &= expect("panels keep the number overlay aspect",
        near(right.loaded.width / right.loaded.height, kNumberAspect) &&
        near(right.reserve.width / right.reserve.height, kNumberAspect));
    const float loadedBottom = right.loaded.center[2] - right.loaded.height * kDigitHeightFraction * 0.5f;
    const float reserveBottom = right.reserve.center[2] - right.reserve.height * kDigitHeightFraction * 0.5f;
    ok &= expect("loaded and reserve digits share a baseline", near(loadedBottom, reserveBottom));

    // Look-at fade: facing from 45 degrees is full, from beyond 60 hidden.
    const Point anchor{ rightPalm[0] - kOutward, rightPalm[1], rightPalm[2] };
    ok &= expect("a counter facing the head is fully visible",
        near(resolve(rightPalm, weaponRight, weaponUp, false, { anchor[0] - 20.0f, anchor[1], anchor[2] }).opacity, kMaxOpacity));
    const auto edgeOn = resolve(rightPalm, weaponRight, weaponUp, false, { anchor[0] - 5.0f, anchor[1] - 20.0f, anchor[2] });
    ok &= expect("a counter seen edge-on is hidden", edgeOn.valid && edgeOn.opacity == 0.0f);
    ok &= expect("looking never moves or rotates the counter",
        near(edgeOn.loaded.center[0], right.loaded.center[0]) && near(edgeOn.front[0], right.front[0]));

    WeaponAnchor palmAnchor;
    const WeaponAnchor::Identity pistol{ 0, 0x100, 7 };
    const auto& captured = palmAnchor.update(pistol, { 1.0f, 2.0f, 3.0f });
    ok &= expect("the first palm is captured", near(captured[0], 1.0f));
    ok &= expect("a hand correction never moves the anchored counter", near(palmAnchor.update(pistol, { 4.0f, 5.0f, 6.0f })[0], 1.0f));
    ok &= expect("another weapon recaptures the palm", near(palmAnchor.update({ 1, 0x200, 9 }, { 4.0f, 5.0f, 6.0f })[0], 4.0f));
    palmAnchor.clear();
    ok &= expect("a cleared anchor recaptures", near(palmAnchor.update({ 1, 0x200, 9 }, { 8.0f, 0.0f, 0.0f })[0], 8.0f));

    ok &= expect("a degenerate weapon basis fails closed", !resolve(rightPalm, weaponUp, weaponUp, false, head).valid);
    ok &= expect("a non-finite palm fails closed", !resolve({ NAN, 0.0f, 0.0f }, weaponRight, weaponUp, false, head).valid);
    return ok ? 0 : 1;
}

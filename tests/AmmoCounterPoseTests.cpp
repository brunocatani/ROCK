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
    // Back of the hand faces -Y (toward a head at -Y); fingers point up (+Z).
    const Point wrist{ 0.0f, 0.0f, 0.0f };
    const Point fingers{ 0.0f, 0.3f, 1.0f };
    const Point back{ 0.0f, -1.0f, 0.0f };

    const auto facing = resolve(wrist, fingers, back, { 0.0f, -30.0f, 4.0f });
    ok &= expect("a hand-fixed pose resolves", facing.valid);
    ok &= expect("the panels face out of the back of the hand", near(facing.front[1], -1.0f));
    ok &= expect("up follows the fingers within the back plane", near(facing.up[2], 1.0f) && near(facing.up[1], 0.0f));
    ok &= expect("right completes a right-handed panel basis", near(facing.right[0], 1.0f));
    ok &= expect("a counter facing the head is fully visible", near(facing.opacity, kMaxOpacity));
    ok &= expect("loaded rounds sit left of the reserve", facing.loaded.center[0] < facing.reserve.center[0]);
    ok &= expect("panels keep the number overlay aspect",
        near(facing.loaded.width / facing.loaded.height, kNumberAspect) &&
        near(facing.reserve.width / facing.reserve.height, kNumberAspect));
    const float loadedBottom = facing.loaded.center[2] - facing.loaded.height * kDigitHeightFraction * 0.5f;
    const float reserveBottom = facing.reserve.center[2] - facing.reserve.height * kDigitHeightFraction * 0.5f;
    ok &= expect("loaded and reserve digits share a baseline", near(loadedBottom, reserveBottom));

    const auto moved = resolve(wrist, fingers, back, { 30.0f, -5.0f, 4.0f });
    ok &= expect("looking never moves or rotates the counter",
        moved.valid && near(moved.loaded.center[0], facing.loaded.center[0]) && near(moved.right[0], facing.right[0]));
    ok &= expect("a counter seen edge-on is hidden", moved.opacity == 0.0f);
    ok &= expect("a counter facing away is hidden", resolve(wrist, fingers, back, { 0.0f, 30.0f, 4.0f }).opacity == 0.0f);

    ok &= expect("fingers along the back normal give no orientation", !resolve(wrist, back, back, { 0.0f, -30.0f, 0.0f }).valid);
    ok &= expect("a non-finite wrist fails closed", !resolve({ NAN, 0.0f, 0.0f }, fingers, back, { 0.0f, -30.0f, 0.0f }).valid);
    return ok ? 0 : 1;
}

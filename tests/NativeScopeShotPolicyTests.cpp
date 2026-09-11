#include "physics-interaction/weapon/telemetry/NativeScopeShotPolicy.h"

#include <cstdio>
#include <limits>

namespace p = rock::native_scope_shot_policy;

int main()
{
    bool ok = true;
    const auto check = [&](bool value, const char* name) { if (!value) { std::printf("FAIL %s\n", name); ok = false; } };
    const auto near = [](float a, float b) { return std::abs(a - b) < 0.001f; };
    constexpr float halfPi = 1.57079632679f;
    const p::Point origin{ 10, -20, 30 };
    const auto forward = p::launchRay(origin, 0, 0);
    check(forward.valid && near(forward.direction.y, 1) && near(forward.direction.x, 0), "zero native angles face +Y");
    const auto right = p::launchRay(origin, halfPi, 0);
    check(near(right.direction.x, 1) && near(right.direction.y, 0), "positive yaw faces +X");
    const auto down = p::launchRay(origin, 0, halfPi);
    check(near(down.direction.z, -1), "positive native pitch faces down");
    check(near(p::angleDegrees(forward, right), 90), "orthogonal rays");
    check(near(p::angleDegrees(forward, p::launchRay(origin, 2 * halfPi, 0)), 180), "opposite rays");
    check(near(p::angleDegrees(forward, p::ray({100, 200, 300}, {0, 7, 0})), 0), "angular comparison ignores origins and vector magnitude");
    const auto endpoint = p::end(forward, 50);
    check(near(endpoint.x, 10) && near(endpoint.y, 30) && near(endpoint.z, 30), "world ray retains muzzle origin");
    check(!p::ray(origin, {}).valid, "zero direction fails closed");
    const float nan = std::numeric_limits<float>::quiet_NaN();
    check(!p::launchRay(origin, nan, 0).valid && !p::ray({nan, 0, 0}, {0, 1, 0}).valid, "nonfinite data fails closed");
    check(p::angleDegrees(forward, {}) == -1, "missing capture is unknown rather than aligned");
    const auto offset = p::planeOffset(forward, {1, 0, 0}, {0, 0, 1}, p::ray({13, -20, 24}, {0, 1, 0}));
    check(offset.valid && near(offset.rightGameUnits, 3) && near(offset.upGameUnits, -6), "parallel rays retain horizontal and bore-height separation");
    const auto crossing = p::planeOffset(p::ray({}, {0, 1, 0}), {1, 0, 0}, {0, 0, 1}, p::ray({-10, 0, 0}, {10, 100, 0}), 100);
    check(crossing.valid && near(crossing.rightGameUnits, 0), "converging ray reaches common sight plane");
    check(!p::planeOffset(forward, {1, 0, 0}, {0, 0, 1}, p::launchRay(origin, 2 * halfPi, 0)).valid, "behind-camera ray is unavailable");
    check(!p::planeOffset(forward, {}, {}, forward).valid, "missing sight basis is not reported as zero error");
    check(!p::planeOffset(forward, {1, 0, 0}, {0, 0, 1}, forward, nan).valid, "invalid reference plane rejected");
    check(std::abs(p::angleDegrees(forward, p::launchRay(origin, 0.001f / p::kRadiansToDegrees, 0)) - 0.001f) < 0.00001f,
        "small angular differences survive floating-point dot rounding");
    // Recorded scoped shot 55, PID 28668, 2026-09-11 13:57:38.253.
    // Its former angle-only display said zero although the launch line was
    // almost six game units below the sight line.
    const auto recordedScope = p::ray({-79223.617188f,90278.031250f,7950.995605f}, {0.702898145f,-0.710876703f,-0.024262231f});
    const auto recordedShot = p::ray({-79236.968750f,90291.851562f,7945.590332f}, {0.702897072f,-0.710877657f,-0.024263371f});
    const auto basisRight = p::ray({}, {recordedScope.direction.y, -recordedScope.direction.x, 0}).direction;
    const auto f = recordedScope.direction;
    const p::Point basisUp{basisRight.y*f.z-basisRight.z*f.y, basisRight.z*f.x-basisRight.x*f.z, basisRight.x*f.y-basisRight.y*f.x};
    const auto replay = p::planeOffset(recordedScope, basisRight, basisUp, recordedShot);
    check(replay.valid && std::abs(std::hypot(replay.rightGameUnits, replay.upGameUnits) - 5.874f) < 0.01f,
        "scoped runtime regression exposes positional mismatch hidden by zero degrees");
    check(p::fresh(9000, 1000, 8000) && !p::fresh(9001, 1000, 8000), "frozen shot expires at bounded age");
    check(!p::fresh(999, 1000, 8000) && !p::fresh(1000, 0, 8000), "future and absent timestamps rejected");
    // Round trip native angles across quadrants and elevation. Recovering
    // pitch through asin guards the sign used by the visual launch guide.
    for (float yaw : {-2.5f, -1.0f, 0.0f, 1.0f, 2.5f}) {
        for (float pitch : {-1.0f, -0.3f, 0.0f, 0.3f, 1.0f}) {
            const auto r = p::launchRay(origin, yaw, pitch);
            check(near(std::atan2(r.direction.x, r.direction.y), yaw) && near(-std::asin(r.direction.z), pitch), "native angle round trip");
        }
    }
    if (ok) std::puts("Native scope shot geometry checks passed");
    return ok ? 0 : 1;
}

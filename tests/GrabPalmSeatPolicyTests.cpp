#include "physics-interaction/grab/GrabPalmSeat.h"

#include <array>
#include <cstdio>
#include <limits>
#include <vector>

namespace
{
    using rock::grab_palm_seat::Point;
    struct Triangle { Point v0, v1, v2; };

    auto depth(const std::vector<Triangle>& mesh, float halfAlong = 3.5f, float halfAcross = 1.4f, float maxDepth = 30.0f)
    {
        return rock::grab_palm_seat::computeDepth(mesh, halfAlong, halfAcross, maxDepth, [](Point p) { return p; });
    }

    bool expect(bool condition, const char* message)
    {
        if (!condition) {
            std::printf("FAIL: %s\n", message);
        }
        return condition;
    }

    bool expectDepth(const std::vector<Triangle>& mesh, float expected, const char* message)
    {
        const auto result = depth(mesh);
        if (!result.valid || std::abs(result.depthGameUnits - expected) > 0.0001f) {
            std::printf("FAIL: %s: depth=%.6f expected=%.6f reason=%s\n", message, result.depthGameUnits, expected, result.reason);
            return false;
        }
        return true;
    }
}

int main()
{
    bool ok = true;
    const Triangle shaft{ { -1, -1, 0.5f }, { 1, -1, 0.5f }, { 0, 1, 0.5f } };
    std::vector<Triangle> cross{ shaft,
        { { 7, -1, 6.2f }, { 8, -1, 6.2f }, { 7.7f, 1, 6.2f } },
        { { -8, -1, 7 }, { -7, -1, 7 }, { -7.7f, 1, 7 } },
        { { -1, 7, 6 }, { 1, 7, 6 }, { 0, 8, 6 } },
        { { -1, -8, 6 }, { 1, -8, 6 }, { 0, -7, 6 } } };
    ok &= expectDepth(cross, 0.5f, "protrusions outside the palm cannot displace its shaft contact");
    cross.push_back({ { -1, -1, 2 }, { 1, -1, 2 }, { 0, 1, 2 } });
    ok &= expectDepth(cross, 2.0f, "geometry actually over the palm still prevents penetration");

    const std::vector<Triangle> coarseFace{
        { { -20, -20, -3 }, { 20, -20, 7 }, { 0, 20, 2 } }
    };
    ok &= expectDepth(coarseFace, 2.875f, "clip coarse tilted faces even when every vertex is outside the palm");
    auto reverseFace = coarseFace;
    std::swap(reverseFace[0].v0, reverseFace[0].v2);
    ok &= expectDepth(reverseFace, 2.875f, "triangle winding does not change palm clearance");
    ok &= expectDepth({ { { 0, 0, 0.5f }, { 8, 0, 6.5f }, { 8, 1, 6.5f } } },
        3.125f, "an intersecting tab contributes only the part over the palm");
    ok &= expectDepth({ { { -3.5f, -1.4f, 2 }, { 3.5f, -1.4f, 2 }, { 3.5f, 1.4f, 2 } } },
        2.0f, "boundary vertices survive clipping without duplicate overflow");
    ok &= expectDepth({ { { 3.5f, 1.4f, 1 }, { 3.5f, 1.4f, 1 }, { 7, 7, 9 } } },
        1.0f, "degenerate triangles touching a corner stay bounded");
    ok &= expectDepth({ { { -1, -1, -2 }, { 1, -1, -2 }, { 0, 1, -2 } } },
        0.0f, "mesh already outside the palm plane needs no push");

    // Scaling the complete geometry and footprint preserves physical units.
    auto scaled = cross;
    for (auto& triangle : scaled) {
        for (auto* p : { &triangle.v0, &triangle.v1, &triangle.v2 }) {
            p->along *= 2;
            p->across *= 2;
            p->depth *= 2;
        }
    }
    const auto scaledResult = depth(scaled, 7, 2.8f);
    ok &= expect(scaledResult.valid && std::abs(scaledResult.depthGameUnits - 4) < 0.0001f, "scaled palm and mesh retain the same contact");
    const auto limited = depth(cross, 3.5f, 1.4f, 1);
    ok &= expect(limited.valid && limited.depthGameUnits == 1, "configured maximum depth remains a limit");
    const auto disabled = depth(cross, 3.5f, 1.4f, 0);
    ok &= expect(disabled.valid && disabled.depthGameUnits == 0, "zero maximum depth retains disabled correction");
    ok &= expect(!depth({}).valid, "missing mesh is unavailable");
    ok &= expect(!depth(cross, 0).valid, "missing palm dimensions are unavailable");
    auto invalid = cross;
    invalid[0].v0.depth = std::numeric_limits<float>::quiet_NaN();
    ok &= expect(!depth(invalid).valid, "non-finite mesh cannot report a safe seat");
    return ok ? 0 : 1;
}

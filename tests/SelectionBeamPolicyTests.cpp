#include "physics-interaction/hand/SelectionBeamPolicy.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
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

    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        if (std::fabs(actual - expected) <= epsilon) {
            return true;
        }

        std::printf("%s expected %.6f got %.6f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::selection_beam_policy;

    bool ok = true;

    ok &= expectFalse("inactive frame does not render",
        shouldRender(Frame{
            .active = false,
            .startWorld = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .endWorld = RE::NiPoint3(20.0f, 0.0f, 0.0f),
        }));

    ok &= expectFalse("disabled config does not render",
        shouldRender(Frame{
            .active = true,
            .startWorld = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .endWorld = RE::NiPoint3(20.0f, 0.0f, 0.0f),
            .config = Config{ .enabled = false },
        }));

    ok &= expectFalse("short beam does not render",
        shouldRender(Frame{
            .active = true,
            .startWorld = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .endWorld = RE::NiPoint3(1.0f, 0.0f, 0.0f),
        }));

    ok &= expectTrue("valid far beam renders",
        shouldRender(Frame{
            .active = true,
            .startWorld = RE::NiPoint3(0.0f, 0.0f, 0.0f),
            .endWorld = RE::NiPoint3(120.0f, 0.0f, 0.0f),
        }));

    auto sanitized = sanitizeConfig(Config{
        .enabled = true,
        .segmentSizeGameUnits = std::numeric_limits<float>::quiet_NaN(),
        .curveLiftGameUnits = 999.0f,
        .alpha = -4.0f,
    });
    ok &= expectNear("nan segment size uses default", sanitized.segmentSizeGameUnits, kDefaultSegmentSizeGameUnits, 0.0001f);
    ok &= expectNear("curve lift clamps", sanitized.curveLiftGameUnits, 80.0f, 0.0001f);
    ok &= expectNear("alpha clamps", sanitized.alpha, 0.05f, 0.0001f);

    const RE::NiPoint3 start(0.0f, 0.0f, 0.0f);
    const RE::NiPoint3 end(100.0f, 0.0f, 0.0f);
    const RE::NiPoint3 control = makeControlPoint(start, end, 30.0f);
    ok &= expectNear("control x midpoint", control.x, 50.0f, 0.0001f);
    ok &= expectNear("control z lift", control.z, 30.0f, 0.0001f);

    const RE::NiPoint3 sample = sampleQuadraticBezier(start, control, end, 0.5f);
    ok &= expectNear("sample x midpoint", sample.x, 50.0f, 0.0001f);
    ok &= expectNear("sample z arced", sample.z, 15.0f, 0.0001f);

    return ok ? 0 : 1;
}

#include "physics-interaction/grab/GrabHeldObject.h"

#include <cmath>
#include <cstdio>
#include <limits>

namespace
{
    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }

        std::printf("%s expected %.5f got %.5f\n", label, expected, actual);
        return false;
    }

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
}

int main()
{
    using namespace rock::held_object_physics_math;

    bool ok = true;

    ok &= expectTrue("normal frame delta allows authority target", shouldQueueGrabAuthorityTargetForDelta(1.0f / 90.0f));
    ok &= expectFalse("oversized frame delta suppresses authority target", shouldQueueGrabAuthorityTargetForDelta(0.0501f));
    ok &= expectFalse("non-finite frame delta suppresses authority target", shouldQueueGrabAuthorityTargetForDelta(std::numeric_limits<float>::quiet_NaN()));
    ok &= expectFalse("zero frame delta suppresses authority target", shouldQueueGrabAuthorityTargetForDelta(0.0f));

    ok &= expectNear("default instant deviation threshold", instantDeviationReleaseThreshold(50.0f), 100.0f, 0.001f);
    ok &= expectTrue("instant deviation over threshold releases", instantDeviationExceeded(101.0f, 50.0f));
    ok &= expectFalse("instant deviation at threshold does not release", instantDeviationExceeded(100.0f, 50.0f));
    ok &= expectNear("larger configured deviation doubles threshold", instantDeviationReleaseThreshold(80.0f), 160.0f, 0.001f);
    ok &= expectFalse("normal deviation does not instant-release", instantDeviationExceeded(60.0f, 50.0f));

    const float firstExceededFrame = advanceDeviationSeconds(0.0f, 60.0f, 50.0f, 1.0f / 90.0f);
    ok &= expectNear("sustained deviation accumulates one normal frame", firstExceededFrame, 1.0f / 90.0f, 0.0001f);
    ok &= expectFalse("sustained deviation still requires configured time", deviationExceeded(firstExceededFrame, 2.0f));
    const float accumulatedExceeded = advanceDeviationSeconds(1.99f, 60.0f, 50.0f, 1.0f / 90.0f);
    ok &= expectTrue("sustained deviation releases after configured time", deviationExceeded(accumulatedExceeded, 2.0f));

    return ok ? 0 : 1;
}

#include "physics-interaction/grab/GrabHapticPolicy.h"

#include <cstdio>

namespace
{
    bool expectNear(const char* label, float actual, float expected, float epsilon)
    {
        const float delta = actual > expected ? actual - expected : expected - actual;
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::grab_haptic_policy;

    bool ok = true;

    MassPulseConfig grab{};
    grab.baseIntensity = 0.10f;
    grab.maxIntensity = 0.70f;
    grab.massScale = 0.05f;
    grab.massExponent = 1.0f;
    ok &= expectNear("grab mass pulse scales by mass", computeMassPulseIntensity(4.0f, grab), 0.30f, 0.001f);
    ok &= expectNear("grab mass pulse clamps to max", computeMassPulseIntensity(100.0f, grab), 0.70f, 0.001f);

    ImpactPulseConfig impact{};
    impact.baseIntensity = 0.12f;
    impact.maxIntensity = 0.85f;
    impact.speedScale = 0.01f;
    impact.massScale = 0.02f;
    impact.massExponent = 1.0f;
    impact.minSpeedGameUnitsPerSecond = 8.0f;
    impact.dampedMultiplier = 0.5f;
    ok &= expectNear("impact below speed threshold suppresses pulse", computeImpactPulseIntensity(10.0f, 7.0f, false, impact), 0.0f, 0.001f);
    ok &= expectNear("impact combines speed and mass", computeImpactPulseIntensity(10.0f, 20.0f, false, impact), 0.52f, 0.001f);
    ok &= expectNear("damped impact keeps at least base intensity", computeImpactPulseIntensity(10.0f, 20.0f, true, impact), 0.26f, 0.001f);

    impact.enabled = false;
    ok &= expectNear("disabled impact suppresses pulse", computeImpactPulseIntensity(10.0f, 20.0f, false, impact), 0.0f, 0.001f);

    return ok ? 0 : 1;
}

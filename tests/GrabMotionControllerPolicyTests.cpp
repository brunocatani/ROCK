#include "physics-interaction/grab/GrabMotionController.h"

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
    using namespace rock::grab_motion_controller;

    bool ok = true;

    MotorInput singleHand{};
    singleHand.enabled = false;
    singleHand.baseMaxForce = 2000.0f;
    singleHand.maxForceMultiplier = 1.0f;
    singleHand.mass = 2.0f;
    singleHand.forceToMassRatio = 500.0f;
    singleHand.angularToLinearForceRatio = 12.5f;
    singleHand.fadeInEnabled = false;
    singleHand.authorityForceScale = 1.0f;

    const auto single = solveMotorTargets(singleHand);
    ok &= expectNear("single hand mass cap", single.linearMaxForce, 1000.0f, 0.001f);
    ok &= expectNear("single hand angular ratio", single.angularMaxForce, 80.0f, 0.001f);

    MotorInput shared = singleHand;
    shared.authorityForceScale = 0.5f;
    const auto twoHand = solveMotorTargets(shared);
    ok &= expectNear("two hands share mass-capped linear authority", twoHand.linearMaxForce, 500.0f, 0.001f);
    ok &= expectNear("two hands share angular authority", twoHand.angularMaxForce, 40.0f, 0.001f);

    MotorInput adaptive = singleHand;
    adaptive.enabled = true;
    adaptive.positionErrorGameUnits = 20.0f;
    adaptive.fullPositionErrorGameUnits = 20.0f;
    adaptive.maxForceMultiplier = 4.0f;
    adaptive.mass = 100.0f;
    adaptive.authorityForceScale = 0.5f;
    const auto adaptiveShared = solveMotorTargets(adaptive);
    ok &= expectNear("two hands share adaptive boosted authority", adaptiveShared.linearMaxForce, 4000.0f, 0.001f);

    return ok ? 0 : 1;
}

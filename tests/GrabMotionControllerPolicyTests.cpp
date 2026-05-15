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

    MotorInput mediumMass = singleHand;
    mediumMass.mass = 10.0f;
    mediumMass.massResponsiveMaxForce = 9000.0f;
    const auto mediumMassOutput = solveMotorTargets(mediumMass);
    ok &= expectNear("medium generic object rises to mass-scaled force", mediumMassOutput.linearMaxForce, 5000.0f, 0.001f);
    ok &= expectNear("medium generic object angular force follows linear force", mediumMassOutput.angularMaxForce, 400.0f, 0.001f);

    MotorInput heavyMass = singleHand;
    heavyMass.mass = 50.0f;
    heavyMass.massResponsiveMaxForce = 9000.0f;
    const auto heavyMassOutput = solveMotorTargets(heavyMass);
    ok &= expectNear("heavy generic object reaches mass-responsive ceiling", heavyMassOutput.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("heavy generic object angular force follows ceiling", heavyMassOutput.angularMaxForce, 720.0f, 0.001f);

    MotorInput looseWeapon = singleHand;
    looseWeapon.baseMaxForce = 9000.0f;
    looseWeapon.mass = 1000.0f;
    looseWeapon.massResponsiveMaxForce = 9000.0f;
    const auto looseWeaponOutput = solveMotorTargets(looseWeapon);
    ok &= expectNear("loose weapon base force is not double-boosted", looseWeaponOutput.linearMaxForce, 9000.0f, 0.001f);
    ok &= expectNear("loose weapon angular force follows existing weapon ceiling", looseWeaponOutput.angularMaxForce, 720.0f, 0.001f);

    MotorInput adaptive = singleHand;
    adaptive.enabled = true;
    adaptive.positionErrorGameUnits = 20.0f;
    adaptive.fullPositionErrorGameUnits = 20.0f;
    adaptive.maxForceMultiplier = 4.0f;
    adaptive.mass = 100.0f;
    adaptive.authorityForceScale = 0.5f;
    const auto adaptiveShared = solveMotorTargets(adaptive);
    ok &= expectNear("two hands share adaptive boosted authority", adaptiveShared.linearMaxForce, 4000.0f, 0.001f);

    ok &= expectNear("long object disabled angular scale",
        computeLongObjectAngularSpeedScale(false, 96.0f, 24.0f, 0.35f),
        1.0f,
        0.001f);
    ok &= expectNear("short object angular scale",
        computeLongObjectAngularSpeedScale(true, 12.0f, 24.0f, 0.35f),
        1.0f,
        0.001f);
    ok &= expectNear("long object angular scale",
        computeLongObjectAngularSpeedScale(true, 48.0f, 24.0f, 0.35f),
        0.5f,
        0.001f);
    ok &= expectNear("very long object angular scale floor",
        computeLongObjectAngularSpeedScale(true, 200.0f, 24.0f, 0.35f),
        0.35f,
        0.001f);

    return ok ? 0 : 1;
}

#include "physics-interaction/hand/DynamicHandCollisionFeedbackPolicy.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"

#include <cstdio>

namespace
{
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

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
    using namespace rock::dynamic_hand_collision_feedback;

    bool ok = true;
    ok &= expectNear("inward velocity projects onto contact", projectedApproachSpeedGameUnitsPerSecond(
                                                                     Vec3{ 0.0f, -10.0f, 0.0f },
                                                                     Vec3{ 0.0f, 2.0f, 0.0f }),
        10.0f,
        0.001f);
    ok &= expectNear("tangential velocity does not pulse", projectedApproachSpeedGameUnitsPerSecond(
                                                               Vec3{ 10.0f, 0.0f, 0.0f },
                                                               Vec3{ 0.0f, 2.0f, 0.0f }),
        0.0f,
        0.001f);
    ok &= expectNear("retreat velocity does not pulse", projectedApproachSpeedGameUnitsPerSecond(
                                                            Vec3{ 0.0f, 10.0f, 0.0f },
                                                            Vec3{ 0.0f, 2.0f, 0.0f }),
        0.0f,
        0.001f);

    ContactPulseConfig config{};
    config.baseIntensity = 0.20f;
    config.maxIntensity = 0.60f;
    config.speedScale = 0.01f;
    config.minApproachSpeedGameUnitsPerSecond = 3.0f;
    config.cooldownSeconds = 0.10f;

    ContactPulseState state{};
    auto decision = updateContactPulse(state, 1, 10.0f, 0.0f, true, config);
    ok &= expectTrue("first contact entry fires", decision.fire);
    ok &= expectNear("entry intensity scales by approach speed", decision.intensity, 0.30f, 0.001f);

    decision = updateContactPulse(state, 1, 10.0f, 0.05f, true, config);
    ok &= expectFalse("same contact generation never repeats", decision.fire);

    decision = updateContactPulse(state, 2, 20.0f, 0.02f, true, config);
    ok &= expectFalse("cooldown suppresses a new contact generation", decision.fire);
    (void)updateContactPulse(state, 2, 0.0f, 0.04f, true, config);
    decision = updateContactPulse(state, 3, 20.0f, 0.0f, true, config);
    ok &= expectTrue("new entry fires after cooldown", decision.fire);
    ok &= expectNear("higher speed raises intensity", decision.intensity, 0.40f, 0.001f);

    (void)updateContactPulse(state, 3, 0.0f, 0.10f, true, config);
    decision = updateContactPulse(state, 4, 20.0f, 0.0f, false, config);
    ok &= expectFalse("stronger owner suppresses contact pulse", decision.fire);
    decision = updateContactPulse(state, 4, 20.0f, 0.0f, true, config);
    ok &= expectFalse("suppressed entry is consumed instead of replayed", decision.fire);

    config.enabled = false;
    decision = updateContactPulse(state, 5, 20.0f, 0.0f, true, config);
    ok &= expectFalse("disabled feedback consumes without firing", decision.fire);
    config.enabled = true;
    decision = updateContactPulse(state, 5, 20.0f, 0.0f, true, config);
    ok &= expectFalse("disabled entry does not replay when enabled", decision.fire);

    decision = updateContactPulse(state, 6, 2.0f, 0.0f, true, config);
    ok &= expectFalse("below-threshold approach does not pulse", decision.fire);

    using rock::dynamic_hand_collision_kinematics::forearmHandTargetResponseScale;
    ok &= expectNear("forearm leverage maps three-quarter reach to hand target",
        forearmHandTargetResponseScale(
            Vec3{ 0.0f, 0.0f, 0.0f },
            Vec3{ 12.0f, 0.0f, 0.0f },
            Vec3{ 9.0f, 0.0f, 0.0f }),
        4.0f / 3.0f,
        0.001f);
    ok &= expectNear("forearm leverage clamps extreme folded-arm gain",
        forearmHandTargetResponseScale(
            Vec3{ 0.0f, 0.0f, 0.0f },
            Vec3{ 12.0f, 0.0f, 0.0f },
            Vec3{ 2.0f, 0.0f, 0.0f }),
        2.5f,
        0.001f);
    ok &= expectNear("forearm leverage never weakens direct hand response",
        forearmHandTargetResponseScale(
            Vec3{ 0.0f, 0.0f, 0.0f },
            Vec3{ 8.0f, 0.0f, 0.0f },
            Vec3{ 10.0f, 0.0f, 0.0f }),
        1.0f,
        0.001f);
    ok &= expectNear("forearm leverage fails closed on degenerate arm geometry",
        forearmHandTargetResponseScale(
            Vec3{ 0.0f, 0.0f, 0.0f },
            Vec3{ 0.0f, 0.0f, 0.0f },
            Vec3{ 5.0f, 0.0f, 0.0f }),
        1.0f,
        0.001f);

    return ok ? 0 : 1;
}

#include "physics-interaction/hand/DynamicHandCollisionFeedbackPolicy.h"
#include "physics-interaction/hand/DynamicHandCollisionKinematics.h"

#include <cstdio>
#include <limits>

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

    using rock::dynamic_hand_collision_kinematics::combineTwinDeviations;
    // Palm, fingertip and forearm are children of the same blocked compound.
    // Increasing controller penetration must leave the presented hand at the
    // solved surface, regardless of which children reported the contact.
    constexpr float stoppedHandZ = 30.0f;
    constexpr std::array<float, 3> childOffsetsZ{ 0.0f, -2.0f, 5.0f };
    for (const float depth : { 1.0f, 5.0f, 10.0f, 20.0f }) {
        const float rawHandZ = stoppedHandZ - depth;
        std::array<Vec3, 3> deviations{};
        for (std::size_t child = 0; child < deviations.size(); ++child) {
            const float requestedZ = rawHandZ + childOffsetsZ[child];
            const float solvedZ = stoppedHandZ + childOffsetsZ[child];
            deviations[child].z = solvedZ - requestedZ;
        }
        for (const auto contacts : {
                 std::array<bool, 3>{ true, false, false },
                 std::array<bool, 3>{ false, false, true },
                 std::array<bool, 3>{ true, true, true } }) {
            const auto correction = combineTwinDeviations(deviations, contacts);
            ok &= expectNear("deeper barrier press holds hand at surface",
                rawHandZ + correction.z, stoppedHandZ, 0.001f);
            ok &= expectNear("barrier press adds no lateral drift", correction.x, 0.0f, 0.001f);
            ok &= expectNear("barrier press adds no forward drift", correction.y, 0.0f, 0.001f);
        }
    }

    const std::array<Vec3, 3> cornerDeviations{
        Vec3{ 3.0f, 0.0f, 0.0f }, Vec3{ 0.0f, 0.0f, 4.0f }, Vec3{ 0.0f, 0.0f, 2.0f }
    };
    const auto corner = combineTwinDeviations(cornerDeviations, std::array{ true, true, true });
    ok &= expectNear("corner keeps wall correction", corner.x, 3.0f, 0.001f);
    ok &= expectNear("shared surface uses deepest correction once", corner.z, 4.0f, 0.001f);

    const auto released = combineTwinDeviations(cornerDeviations, std::array{ false, false, false });
    ok &= expectNear("released wall no longer contributes", released.x, 0.0f, 0.001f);
    ok &= expectNear("released floor no longer contributes", released.z, 0.0f, 0.001f);

    const auto invalid = combineTwinDeviations(
        std::array{ Vec3{ std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f }, Vec3{ 0.0f, 0.0f, 4.0f } },
        std::array{ true, true });
    ok &= expectNear("invalid contact cannot contaminate other contacts", invalid.z, 4.0f, 0.001f);

    return ok ? 0 : 1;
}

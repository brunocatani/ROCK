#include "physics-interaction/consume/MouthConsumeDetector.h"
#include "physics-interaction/consume/MouthConsumePolicy.h"

#include <cmath>
#include <cstdio>
#include <cstring>

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
        const float delta = std::fabs(actual - expected);
        if (delta <= epsilon) {
            return true;
        }
        std::printf("%s expected %.4f got %.4f\n", label, expected, actual);
        return false;
    }
}

int main()
{
    using namespace rock::mouth_consume;
    namespace haptics = rock::mouth_consume_haptic_policy;

    bool ok = true;

    const auto center = computeMouthCenter(
        RE::NiPoint3{ 0.0f, 0.0f, 0.0f },
        RE::NiPoint3{ 0.0f, 1.0f, 0.0f },
        RE::NiPoint3{ 2.0f, 10.0f, -4.0f });
    ok &= expectNear("mouth center applies right offset", center.x, 2.0f, 0.001f);
    ok &= expectNear("mouth center applies forward offset", center.y, 10.0f, 0.001f);
    ok &= expectNear("mouth center applies vertical offset", center.z, -4.0f, 0.001f);

    DetectorConfig config{};
    config.hmdMouthOffsetGameUnits = RE::NiPoint3{ 0.0f, 10.0f, -4.0f };
    config.mouthRadiusGameUnits = 10.0f;
    config.enterPaddingGameUnits = 0.0f;
    config.exitPaddingGameUnits = 2.0f;
    config.minDwellSeconds = 0.05f;
    config.maxSpeedGameUnitsPerSecond = 120.0f;

    RuntimeState runtime{};
    DetectorInput input{};
    input.hasHmdFrame = true;
    input.hmdPositionWorld = RE::NiPoint3{ 0.0f, 0.0f, 0.0f };
    input.hmdForwardWorld = RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
    input.objectProbe.pointGame = RE::NiPoint3{ 0.0f, 10.0f, -4.0f };
    input.deltaSeconds = 0.03f;
    input.config = config;

    auto decision = evaluate(input, runtime);
    ok &= expectTrue("inside sphere enters candidate", decision.candidate);
    ok &= expectTrue("first frame reports entered", decision.enteredCandidate);
    ok &= expectFalse("first frame does not satisfy dwell", decision.confirmedForCommit);

    input.deltaSeconds = 0.06f;
    decision = evaluate(input, runtime);
    ok &= expectTrue("dwell confirms candidate", decision.confirmedForCommit);
    ok &= expectNear("center confidence clamps to one", decision.confidence, 1.0f, 0.001f);

    input.objectProbe.pointGame = RE::NiPoint3{ 11.5f, 10.0f, -4.0f };
    input.deltaSeconds = 1.0f;
    decision = evaluate(input, runtime);
    ok &= expectTrue("exit padding prevents mouth flicker", decision.candidate);

    resetRuntime(runtime);
    decision = evaluate(input, runtime);
    ok &= expectFalse("enter padding stays precise", decision.candidate);

    input.objectProbe.pointGame = RE::NiPoint3{ 0.0f, 10.0f, -4.0f };
    input.objectProbe.velocityGamePerSecond = RE::NiPoint3{ 200.0f, 0.0f, 0.0f };
    input.objectProbe.hasVelocity = true;
    decision = evaluate(input, runtime);
    ok &= expectFalse("fast probe is rejected", decision.candidate);

    haptics::CandidatePulseConfig haptic{};
    haptic.baseIntensity = 0.22f;
    haptic.maxIntensity = 0.45f;
    ok &= expectNear("mouth haptic starts at base", haptics::computeCandidatePulseIntensity(0.0f, haptic), 0.22f, 0.001f);
    ok &= expectNear("mouth haptic scales confidence", haptics::computeCandidatePulseIntensity(0.5f, haptic), 0.335f, 0.001f);
    ok &= expectNear("mouth haptic clamps", haptics::computeCandidatePulseIntensity(5.0f, haptic), 0.45f, 0.001f);

    haptic.enabled = false;
    ok &= expectNear("disabled mouth haptic suppresses pulse", haptics::computeCandidatePulseIntensity(1.0f, haptic), 0.0f, 0.001f);

    return ok ? 0 : 1;
}

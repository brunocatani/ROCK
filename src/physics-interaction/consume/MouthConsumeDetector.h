#pragma once

#include "RE/NetImmerse/NiPoint.h"

namespace rock::mouth_consume
{
    struct Probe
    {
        RE::NiPoint3 pointGame{};
        RE::NiPoint3 velocityGamePerSecond{};
        bool hasVelocity = false;
    };

    struct RuntimeState
    {
        bool candidate = false;
        bool confirmed = false;
        float dwellSeconds = 0.0f;
        float nextCandidatePulseTimeSeconds = 0.0f;
        RE::NiPoint3 lastProbePointGame{};
        bool hasLastProbePoint = false;
    };

    struct DetectorConfig
    {
        bool enabled = true;
        RE::NiPoint3 hmdMouthOffsetGameUnits{ 0.0f, 10.0f, -9.0f };
        float mouthRadiusGameUnits = 11.0f;
        float enterPaddingGameUnits = 0.0f;
        float exitPaddingGameUnits = 2.0f;
        float minDwellSeconds = 0.06f;
        float maxSpeedGameUnitsPerSecond = 120.0f;
    };

    struct DetectorInput
    {
        bool hasHmdFrame = false;
        RE::NiPoint3 hmdPositionWorld{};
        RE::NiPoint3 hmdForwardWorld{};
        Probe objectProbe{};
        bool hasObjectProbe = false;
        Probe handProbe{};
        bool hasHandProbe = false;
        float deltaSeconds = 0.0f;
        DetectorConfig config{};
    };

    struct Decision
    {
        bool candidate = false;
        bool confirmedForCommit = false;
        bool enteredCandidate = false;
        bool changedCandidate = false;
        RE::NiPoint3 mouthCenterGame{};
        float distanceGameUnits = 0.0f;
        float confidence = 0.0f;
        float speedGameUnitsPerSecond = 0.0f;
    };

    void resetRuntime(RuntimeState& state) noexcept;

    [[nodiscard]] bool finitePoint(const RE::NiPoint3& value) noexcept;
    [[nodiscard]] RE::NiPoint3 computeMouthCenter(
        const RE::NiPoint3& hmdPositionWorld,
        const RE::NiPoint3& hmdForwardWorld,
        const RE::NiPoint3& offsetGameUnits) noexcept;
    [[nodiscard]] float probeSpeed(const Probe& probe) noexcept;
    [[nodiscard]] float candidateConfidence(float distanceGameUnits, float thresholdGameUnits, float radiusGameUnits) noexcept;
    [[nodiscard]] Decision evaluate(const DetectorInput& input, RuntimeState& runtime) noexcept;
}

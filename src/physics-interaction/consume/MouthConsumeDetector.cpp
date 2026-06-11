#include "physics-interaction/consume/MouthConsumeDetector.h"

#include <algorithm>
#include <cmath>

namespace rock::mouth_consume
{
    namespace
    {
        const RE::NiPoint3 kWorldUp{ 0.0f, 0.0f, 1.0f };
        const RE::NiPoint3 kWorldForward{ 0.0f, 1.0f, 0.0f };

        [[nodiscard]] RE::NiPoint3 add(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) noexcept
        {
            return RE::NiPoint3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        }

        [[nodiscard]] RE::NiPoint3 sub(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) noexcept
        {
            return RE::NiPoint3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        [[nodiscard]] RE::NiPoint3 mul(const RE::NiPoint3& value, float scale) noexcept
        {
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        [[nodiscard]] float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) noexcept
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        [[nodiscard]] RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) noexcept
        {
            return RE::NiPoint3{
                lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.x * rhs.y - lhs.y * rhs.x,
            };
        }

        [[nodiscard]] float lengthSquared(const RE::NiPoint3& value) noexcept
        {
            return dot(value, value);
        }

        [[nodiscard]] float length(const RE::NiPoint3& value) noexcept
        {
            const float sq = lengthSquared(value);
            return sq > 0.0f && std::isfinite(sq) ? std::sqrt(sq) : 0.0f;
        }

        [[nodiscard]] RE::NiPoint3 normalizeOr(const RE::NiPoint3& value, const RE::NiPoint3& fallback) noexcept
        {
            const float len = length(value);
            if (len <= 0.00001f) {
                return fallback;
            }
            return mul(value, 1.0f / len);
        }

        [[nodiscard]] const Probe& selectedProbe(const DetectorInput& input) noexcept
        {
            return finitePoint(input.objectProbe.pointGame) ? input.objectProbe : input.handProbe;
        }

        [[nodiscard]] float resolvedProbeSpeed(const DetectorInput& input, const RuntimeState& runtime) noexcept
        {
            const Probe& probe = selectedProbe(input);
            if (probe.hasVelocity) {
                return probeSpeed(probe);
            }
            if (!runtime.hasLastProbePoint || input.deltaSeconds <= 0.000001f) {
                return 0.0f;
            }
            return length(sub(probe.pointGame, runtime.lastProbePointGame)) / input.deltaSeconds;
        }
    }

    void resetRuntime(RuntimeState& state) noexcept
    {
        state = {};
    }

    bool finitePoint(const RE::NiPoint3& value) noexcept
    {
        return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
    }

    RE::NiPoint3 computeMouthCenter(
        const RE::NiPoint3& hmdPositionWorld,
        const RE::NiPoint3& hmdForwardWorld,
        const RE::NiPoint3& offsetGameUnits) noexcept
    {
        const RE::NiPoint3 forward = normalizeOr(hmdForwardWorld, kWorldForward);
        RE::NiPoint3 right = normalizeOr(cross(forward, kWorldUp), RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        if (lengthSquared(right) <= 0.000001f) {
            right = RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        }

        return add(add(add(hmdPositionWorld, mul(right, offsetGameUnits.x)), mul(forward, offsetGameUnits.y)), mul(kWorldUp, offsetGameUnits.z));
    }

    float probeSpeed(const Probe& probe) noexcept
    {
        return probe.hasVelocity ? length(probe.velocityGamePerSecond) : 0.0f;
    }

    float candidateConfidence(float distanceGameUnits, float thresholdGameUnits, float radiusGameUnits) noexcept
    {
        if (!std::isfinite(distanceGameUnits) || !std::isfinite(thresholdGameUnits) || thresholdGameUnits <= 0.0001f) {
            return 0.0f;
        }

        const float normalized = std::clamp(1.0f - distanceGameUnits / thresholdGameUnits, 0.0f, 1.0f);
        const float coreBonus = distanceGameUnits <= radiusGameUnits ? 0.20f : 0.0f;
        return std::clamp(normalized + coreBonus, 0.0f, 1.0f);
    }

    Decision evaluate(const DetectorInput& input, RuntimeState& runtime) noexcept
    {
        Decision decision{};
        const Probe& probe = selectedProbe(input);
        auto updateProbeHistory = [&]() {
            runtime.lastProbePointGame = probe.pointGame;
            runtime.hasLastProbePoint = finitePoint(probe.pointGame);
        };

        const float speed = resolvedProbeSpeed(input, runtime);
        decision.speedGameUnitsPerSecond = speed;

        if (!input.config.enabled || !input.hasHmdFrame || !finitePoint(input.hmdPositionWorld) || !finitePoint(input.hmdForwardWorld) || !finitePoint(probe.pointGame)) {
            resetRuntime(runtime);
            updateProbeHistory();
            return decision;
        }

        if (std::isfinite(input.config.maxSpeedGameUnitsPerSecond) &&
            input.config.maxSpeedGameUnitsPerSecond > 0.0f &&
            speed > input.config.maxSpeedGameUnitsPerSecond) {
            runtime.candidate = false;
            runtime.confirmed = false;
            runtime.dwellSeconds = 0.0f;
            updateProbeHistory();
            return decision;
        }

        const RE::NiPoint3 mouthCenter = computeMouthCenter(input.hmdPositionWorld, input.hmdForwardWorld, input.config.hmdMouthOffsetGameUnits);
        const float radius = (std::max)(0.1f, input.config.mouthRadiusGameUnits);
        const bool wasCandidate = runtime.candidate;
        const float padding = wasCandidate ? input.config.exitPaddingGameUnits : input.config.enterPaddingGameUnits;
        const float threshold = radius + (std::max)(0.0f, padding);
        const float distanceGameUnits = length(sub(probe.pointGame, mouthCenter));
        if (!std::isfinite(distanceGameUnits) || distanceGameUnits > threshold) {
            runtime.candidate = false;
            runtime.confirmed = false;
            runtime.dwellSeconds = 0.0f;
            updateProbeHistory();
            return decision;
        }

        runtime.candidate = true;
        runtime.dwellSeconds = wasCandidate ? runtime.dwellSeconds + (std::max)(0.0f, input.deltaSeconds) : 0.0f;
        runtime.confirmed = runtime.dwellSeconds >= (std::max)(0.0f, input.config.minDwellSeconds);

        decision.candidate = true;
        decision.confirmedForCommit = runtime.confirmed;
        decision.enteredCandidate = !wasCandidate;
        decision.changedCandidate = !wasCandidate;
        decision.mouthCenterGame = mouthCenter;
        decision.distanceGameUnits = distanceGameUnits;
        decision.confidence = candidateConfidence(distanceGameUnits, threshold, radius);
        decision.speedGameUnitsPerSecond = speed;

        updateProbeHistory();
        return decision;
    }
}

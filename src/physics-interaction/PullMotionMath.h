#pragma once

#include <algorithm>
#include <cmath>

namespace frik::rock::pull_motion_math
{
    template <class Vec3>
    struct PullMotionInput
    {
        Vec3 handHavok{};
        Vec3 objectPointHavok{};
        Vec3 previousTargetHavok{};
        float elapsedSeconds = 0.0f;
        float durationSeconds = 0.0f;
        float applyVelocitySeconds = 0.2f;
        float trackHandSeconds = 0.1f;
        float destinationOffsetHavok = 0.01f;
        float maxVelocityHavok = 10.0f;
        bool hasPreviousTarget = false;
    };

    template <class Vec3>
    struct PullMotionResult
    {
        Vec3 targetHavok{};
        Vec3 velocityHavok{};
        float durationRemainingSeconds = 0.0f;
        bool applyVelocity = false;
        bool refreshTarget = false;
        bool expired = false;
    };

    inline float computePullDurationSeconds(float distanceHavok, float a, float b, float c)
    {
        const float clampedDistance = (std::max)(0.0f, distanceHavok);
        return (std::max)(0.001f, a + b * std::exp(-c * clampedDistance));
    }

    template <class Vec3>
    float vectorLength(const Vec3& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    template <class Vec3>
    Vec3 clampLength(Vec3 value, float maxLength)
    {
        if (maxLength <= 0.0f) {
            return value;
        }

        const float length = vectorLength(value);
        if (length <= maxLength || length <= 0.0001f) {
            return value;
        }

        const float scale = maxLength / length;
        value.x *= scale;
        value.y *= scale;
        value.z *= scale;
        return value;
    }

    template <class Vec3>
    PullMotionResult<Vec3> computePullMotion(const PullMotionInput<Vec3>& input)
    {
        PullMotionResult<Vec3> result{};
        result.refreshTarget = !input.hasPreviousTarget || input.elapsedSeconds <= input.trackHandSeconds;
        result.targetHavok = result.refreshTarget ? input.handHavok : input.previousTargetHavok;
        if (result.refreshTarget) {
            result.targetHavok.z += input.destinationOffsetHavok;
        }

        if (input.elapsedSeconds > input.applyVelocitySeconds) {
            result.expired = true;
            return result;
        }

        result.durationRemainingSeconds = input.durationSeconds - input.elapsedSeconds;
        if (result.durationRemainingSeconds <= 0.001f) {
            result.expired = true;
            return result;
        }

        Vec3 horizontalDelta{
            result.targetHavok.x - input.objectPointHavok.x,
            result.targetHavok.y - input.objectPointHavok.y,
            0.0f,
        };

        const float verticalDelta = result.targetHavok.z - input.objectPointHavok.z;
        result.velocityHavok.x = horizontalDelta.x / result.durationRemainingSeconds;
        result.velocityHavok.y = horizontalDelta.y / result.durationRemainingSeconds;
        result.velocityHavok.z = 0.5f * 9.81f * result.durationRemainingSeconds + verticalDelta / result.durationRemainingSeconds;
        result.velocityHavok = clampLength(result.velocityHavok, input.maxVelocityHavok);
        result.applyVelocity = true;
        return result;
    }
}

#pragma once

#include <cmath>
#include <cstdint>

namespace rock::runtime_state_policy
{
    inline constexpr float kFallbackDeltaSeconds = 1.0f / 90.0f;
    inline constexpr float kMaximumFrameDeltaSeconds = 0.1f;
    inline constexpr float kDefaultMovementThresholdGameUnits = 0.05f;

    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    [[nodiscard]] inline float sanitizeFrameDelta(float rawDeltaSeconds)
    {
        return (rawDeltaSeconds > 0.0f && rawDeltaSeconds <= kMaximumFrameDeltaSeconds) ? rawDeltaSeconds : kFallbackDeltaSeconds;
    }

    [[nodiscard]] inline Vec3 subtract(Vec3 lhs, Vec3 rhs)
    {
        return Vec3{
            .x = lhs.x - rhs.x,
            .y = lhs.y - rhs.y,
            .z = lhs.z - rhs.z,
        };
    }

    [[nodiscard]] inline float lengthSquared(Vec3 value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    struct PlayerSpaceTrackerState
    {
        bool hasPrevious = false;
        Vec3 previousPosition{};
    };

    struct PlayerSpaceTrackerInput
    {
        bool positionValid = false;
        Vec3 currentPosition{};
        float movementThresholdGameUnits = kDefaultMovementThresholdGameUnits;
    };

    struct PlayerSpaceTrackerDecision
    {
        bool valid = false;
        bool moving = false;
        Vec3 deltaGameUnits{};
    };

    inline PlayerSpaceTrackerDecision updatePlayerSpaceTracker(PlayerSpaceTrackerState& state, const PlayerSpaceTrackerInput& input)
    {
        if (!input.positionValid) {
            state = {};
            return {};
        }

        PlayerSpaceTrackerDecision decision{
            .valid = true,
        };

        if (state.hasPrevious) {
            decision.deltaGameUnits = subtract(input.currentPosition, state.previousPosition);
            const float threshold = input.movementThresholdGameUnits > 0.0f ? input.movementThresholdGameUnits : kDefaultMovementThresholdGameUnits;
            decision.moving = lengthSquared(decision.deltaGameUnits) > threshold * threshold;
        }

        state.hasPrevious = true;
        state.previousPosition = input.currentPosition;
        return decision;
    }

    struct SkeletonReadinessInput
    {
        bool rootNodeAvailable = false;
        bool rootParentAttached = false;
        bool flattenedTreeValid = false;
        bool requiredHandBonesResolved = false;
        bool bodyBonesRequired = false;
        bool requiredBodyBonesResolved = false;
    };

    [[nodiscard]] inline bool evaluateSkeletonReadiness(const SkeletonReadinessInput& input)
    {
        return input.rootNodeAvailable &&
            input.rootParentAttached &&
            input.flattenedTreeValid &&
            input.requiredHandBonesResolved &&
            (!input.bodyBonesRequired || input.requiredBodyBonesResolved);
    }
}

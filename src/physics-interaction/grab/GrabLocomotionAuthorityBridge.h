#pragma once

#include <algorithm>
#include <cmath>

namespace rock::grab_locomotion_authority_bridge
{
    inline constexpr float kDefaultMaxLeadSeconds = 0.012f;
    inline constexpr float kDefaultSmoothingHz = 45.0f;
    inline constexpr float kDefaultMaxOffsetGameUnits = 4.0f;
    inline constexpr float kDefaultResetDistanceGameUnits = 35.0f;
    inline constexpr int kRetainedVelocityFrames = 3;

    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    struct Config
    {
        bool enabled = true;
        float maxLeadSeconds = kDefaultMaxLeadSeconds;
        float smoothingHz = kDefaultSmoothingHz;
        float maxOffsetGameUnits = kDefaultMaxOffsetGameUnits;
        float resetDistanceGameUnits = kDefaultResetDistanceGameUnits;
    };

    struct State
    {
        bool hasSmoothedVelocity = false;
        bool hasVelocity = false;
        Vec3 velocityGameUnitsPerSecond{};
        int retainedVelocityFrames = 0;
    };

    struct Input
    {
        Config config{};
        bool playerSpaceValid = false;
        bool playerMoving = false;
        bool heldObjectActive = false;
        bool worldOrMenuReset = false;
        Vec3 playerPositionGame{};
        Vec3 playerDeltaGameUnits{};
        float deltaSeconds = 1.0f / 90.0f;
    };

    struct Output
    {
        bool active = false;
        bool reset = false;
        const char* resetReason = "none";
        Vec3 offsetGameUnits{};
        Vec3 velocityGameUnitsPerSecond{};
    };

    [[nodiscard]] inline bool finite(float value)
    {
        return std::isfinite(value);
    }

    [[nodiscard]] inline bool finite(Vec3 value)
    {
        return finite(value.x) && finite(value.y) && finite(value.z);
    }

    [[nodiscard]] inline Vec3 add(Vec3 lhs, Vec3 rhs)
    {
        return Vec3{
            .x = lhs.x + rhs.x,
            .y = lhs.y + rhs.y,
            .z = lhs.z + rhs.z,
        };
    }

    [[nodiscard]] inline Vec3 subtract(Vec3 lhs, Vec3 rhs)
    {
        return Vec3{
            .x = lhs.x - rhs.x,
            .y = lhs.y - rhs.y,
            .z = lhs.z - rhs.z,
        };
    }

    [[nodiscard]] inline Vec3 multiply(Vec3 value, float scalar)
    {
        return Vec3{
            .x = value.x * scalar,
            .y = value.y * scalar,
            .z = value.z * scalar,
        };
    }

    [[nodiscard]] inline float lengthSquared(Vec3 value)
    {
        return value.x * value.x + value.y * value.y + value.z * value.z;
    }

    [[nodiscard]] inline float length(Vec3 value)
    {
        return std::sqrt(lengthSquared(value));
    }

    [[nodiscard]] inline Vec3 clampLength(Vec3 value, float maxLength)
    {
        if (!finite(value) || maxLength <= 0.0f || !finite(maxLength)) {
            return {};
        }

        const float currentLength = length(value);
        if (currentLength <= maxLength || currentLength <= 0.0001f) {
            return value;
        }

        return multiply(value, maxLength / currentLength);
    }

    [[nodiscard]] inline float sanitizePositive(float value, float fallback, float minValue, float maxValue)
    {
        if (!finite(value)) {
            value = fallback;
        }
        return std::clamp(value, minValue, maxValue);
    }

    [[nodiscard]] inline float smoothingAlpha(float smoothingHz, float deltaSeconds)
    {
        const float hz = sanitizePositive(smoothingHz, kDefaultSmoothingHz, 0.0f, 240.0f);
        const float dt = sanitizePositive(deltaSeconds, 1.0f / 90.0f, 0.0f, 0.1f);
        if (hz <= 0.0f || dt <= 0.0f) {
            return 1.0f;
        }

        return std::clamp(1.0f - std::exp(-hz * dt), 0.0f, 1.0f);
    }

    [[nodiscard]] inline Config sanitizeConfig(Config config)
    {
        config.maxLeadSeconds = sanitizePositive(config.maxLeadSeconds, kDefaultMaxLeadSeconds, 0.0f, 0.05f);
        config.smoothingHz = sanitizePositive(config.smoothingHz, kDefaultSmoothingHz, 0.0f, 240.0f);
        config.maxOffsetGameUnits = sanitizePositive(config.maxOffsetGameUnits, kDefaultMaxOffsetGameUnits, 0.0f, 50.0f);
        config.resetDistanceGameUnits = sanitizePositive(config.resetDistanceGameUnits, kDefaultResetDistanceGameUnits, 1.0f, 500.0f);
        return config;
    }

    [[nodiscard]] inline Output reset(State& state, const char* reason)
    {
        state = {};
        Output output{};
        output.reset = true;
        output.resetReason = reason ? reason : "reset";
        return output;
    }

    [[nodiscard]] inline Output update(State& state, Input input)
    {
        input.config = sanitizeConfig(input.config);

        if (!input.config.enabled) {
            return reset(state, "disabled");
        }
        if (input.worldOrMenuReset) {
            return reset(state, "world-or-menu-reset");
        }
        if (!input.playerSpaceValid || !finite(input.playerPositionGame) || !finite(input.playerDeltaGameUnits)) {
            return reset(state, "invalid-player-space");
        }
        if (!input.heldObjectActive) {
            return reset(state, "no-held-object");
        }
        if (!finite(input.deltaSeconds) || input.deltaSeconds <= 0.0f || input.deltaSeconds > 0.1f) {
            return reset(state, "invalid-delta");
        }

        if (length(input.playerDeltaGameUnits) > input.config.resetDistanceGameUnits) {
            return reset(state, "root-jump");
        }

        Output output{};
        if (!input.playerMoving) {
            if (state.retainedVelocityFrames > 0) {
                --state.retainedVelocityFrames;
            } else {
                state.hasVelocity = false;
                state.hasSmoothedVelocity = false;
                state.velocityGameUnitsPerSecond = {};
            }
            output.velocityGameUnitsPerSecond = state.hasVelocity ? state.velocityGameUnitsPerSecond : Vec3{};
            return output;
        }

        const Vec3 measuredVelocity = multiply(input.playerDeltaGameUnits, 1.0f / input.deltaSeconds);
        if (!finite(measuredVelocity)) {
            return reset(state, "invalid-velocity");
        }

        if (!state.hasSmoothedVelocity) {
            state.velocityGameUnitsPerSecond = measuredVelocity;
            state.hasSmoothedVelocity = true;
        } else {
            const float alpha = smoothingAlpha(input.config.smoothingHz, input.deltaSeconds);
            state.velocityGameUnitsPerSecond = add(
                state.velocityGameUnitsPerSecond,
                multiply(subtract(measuredVelocity, state.velocityGameUnitsPerSecond), alpha));
        }
        state.hasVelocity = true;
        state.retainedVelocityFrames = kRetainedVelocityFrames;

        output.active = true;
        output.velocityGameUnitsPerSecond = state.velocityGameUnitsPerSecond;
        output.offsetGameUnits = clampLength(multiply(state.velocityGameUnitsPerSecond, input.config.maxLeadSeconds), input.config.maxOffsetGameUnits);
        return output;
    }
}

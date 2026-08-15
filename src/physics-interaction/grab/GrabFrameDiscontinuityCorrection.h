#pragma once

/*
 * Body-side correction for the part of player-space motion that does not fit
 * inside the Havok step consuming the current game-frame target.
 *
 * This is deliberately not a filter, smoother, prediction path, velocity
 * drive, or accumulated offset. The game frame publishes an exact player-space
 * displacement. Havok advances the held dynamic body for its own exact whole
 * step. Their difference is a one-frame position discontinuity:
 *
 *   correction = playerDelta * (1 - physicsDelta / sourceDelta)
 *
 * The correction is evaluated once for each queued grab-authority sample and
 * applied to the live body before its finite constraint motors solve. It is
 * zero while standing and when the two clocks cover the same interval. Contact,
 * hitches, teleports, invalid samples, and implausible deltas fail closed.
 */

#include <algorithm>
#include <cmath>

namespace rock::grab_frame_discontinuity
{
    inline constexpr float kMaximumSourceDeltaSeconds = 0.05f;
    inline constexpr float kMaximumPlayerDeltaGameUnits = 35.0f;
    inline constexpr float kMaximumCorrectionGameUnits = 2.0f;
    inline constexpr float kMinimumCorrectionGameUnits = 0.0001f;

    template <class Vec3>
    struct Input
    {
        Vec3 playerDeltaGameUnits{};
        float sourceDeltaSeconds = 0.0f;
        float physicsDeltaSeconds = 0.0f;
        float authorityScale = 1.0f;
        bool playerDeltaValid = false;
        bool heldBodyColliding = false;
    };

    template <class Vec3>
    struct Result
    {
        Vec3 deltaGameUnits{};
        float magnitudeGameUnits = 0.0f;
        bool apply = false;
        bool clamped = false;
        const char* reason = "invalid-player-delta";
    };

    template <class Vec3>
    [[nodiscard]] inline bool finiteVector(const Vec3& value)
    {
        return std::isfinite(value.x) &&
               std::isfinite(value.y) &&
               std::isfinite(value.z);
    }

    template <class Vec3>
    [[nodiscard]] inline float vectorMagnitude(const Vec3& value)
    {
        const float squared =
            value.x * value.x + value.y * value.y + value.z * value.z;
        return std::isfinite(squared) && squared >= 0.0f ?
                   std::sqrt(squared) :
                   0.0f;
    }

    template <class Vec3>
    [[nodiscard]] inline Result<Vec3> evaluate(const Input<Vec3>& input)
    {
        Result<Vec3> result{};
        if (!input.playerDeltaValid || !finiteVector(input.playerDeltaGameUnits)) {
            return result;
        }
        if (input.heldBodyColliding) {
            result.reason = "contact-active";
            return result;
        }
        if (!std::isfinite(input.sourceDeltaSeconds) ||
            input.sourceDeltaSeconds <= 0.000001f ||
            input.sourceDeltaSeconds > kMaximumSourceDeltaSeconds) {
            result.reason = "invalid-source-delta";
            return result;
        }
        if (!std::isfinite(input.physicsDeltaSeconds) ||
            input.physicsDeltaSeconds <= 0.000001f ||
            input.physicsDeltaSeconds > kMaximumSourceDeltaSeconds) {
            result.reason = "invalid-physics-delta";
            return result;
        }

        const float playerDeltaMagnitude =
            vectorMagnitude(input.playerDeltaGameUnits);
        if (playerDeltaMagnitude > kMaximumPlayerDeltaGameUnits) {
            result.reason = "player-discontinuity";
            return result;
        }

        const float authorityScale = std::clamp(
            std::isfinite(input.authorityScale) ? input.authorityScale : 0.0f,
            0.0f,
            1.0f);
        if (authorityScale <= 0.0f) {
            result.reason = "no-authority";
            return result;
        }

        const float uncoveredFraction =
            1.0f - input.physicsDeltaSeconds / input.sourceDeltaSeconds;
        result.deltaGameUnits = Vec3{
            input.playerDeltaGameUnits.x * uncoveredFraction * authorityScale,
            input.playerDeltaGameUnits.y * uncoveredFraction * authorityScale,
            input.playerDeltaGameUnits.z * uncoveredFraction * authorityScale,
        };
        result.magnitudeGameUnits = vectorMagnitude(result.deltaGameUnits);
        if (result.magnitudeGameUnits < kMinimumCorrectionGameUnits) {
            result.deltaGameUnits = {};
            result.magnitudeGameUnits = 0.0f;
            result.reason = "clocks-aligned";
            return result;
        }

        if (result.magnitudeGameUnits > kMaximumCorrectionGameUnits) {
            const float scale =
                kMaximumCorrectionGameUnits / result.magnitudeGameUnits;
            result.deltaGameUnits.x *= scale;
            result.deltaGameUnits.y *= scale;
            result.deltaGameUnits.z *= scale;
            result.magnitudeGameUnits = kMaximumCorrectionGameUnits;
            result.clamped = true;
        }

        result.apply = true;
        result.reason = result.clamped ? "bounded-clock-gap" : "clock-gap";
        return result;
    }
}

#pragma once

/*
 * HIGGS gets convincing wrapped fingers by driving the visible skeleton, not by
 * changing the held-object constraint. ROCK follows that ownership split: the
 * mesh solver still produces scalar joint curls, FRIK supplies the authored
 * local-transform baseline for those curls, and this policy decides when ROCK
 * may add bounded surface-aim corrections to that baseline.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace frik::rock::grab_finger_local_transform_math
{
    inline constexpr std::size_t kFingerLocalTransformCount = 15;
    inline constexpr std::uint16_t kFullFingerLocalTransformMask = 0x7FFF;
    inline constexpr float kDefaultLocalTransformSmoothingSpeed = 14.0f;
    inline constexpr float kDefaultMaxCorrectionDegrees = 35.0f;
    inline constexpr float kDefaultSurfaceAimStrength = 0.75f;
    inline constexpr float kDefaultThumbOppositionStrength = 1.0f;
    inline constexpr float kDefaultThumbAlternateCurveStrength = 0.65f;

    [[nodiscard]] inline std::uint16_t sanitizeFingerLocalTransformMask(std::uint16_t mask)
    {
        return static_cast<std::uint16_t>(mask & kFullFingerLocalTransformMask);
    }

    [[nodiscard]] inline bool shouldPublishLocalTransformPose(
        bool enabled,
        bool poseSolved,
        bool hasJointValues,
        bool hasBaselineApi,
        bool hasPublishApi)
    {
        return enabled && poseSolved && hasJointValues && hasBaselineApi && hasPublishApi;
    }

    [[nodiscard]] inline float sanitizeUnitStrength(float value, float fallback)
    {
        const float resolvedFallback = std::isfinite(fallback) ? std::clamp(fallback, 0.0f, 1.0f) : 0.0f;
        if (!std::isfinite(value)) {
            return resolvedFallback;
        }
        return std::clamp(value, 0.0f, 1.0f);
    }

    [[nodiscard]] inline float sanitizeMaxCorrectionDegrees(float value, float fallback)
    {
        const float resolvedFallback = std::isfinite(fallback) ? std::max(0.0f, fallback) : kDefaultMaxCorrectionDegrees;
        if (!std::isfinite(value)) {
            return resolvedFallback;
        }
        return std::max(0.0f, value);
    }

    [[nodiscard]] inline float sanitizeSmoothingSpeed(float value, float fallback)
    {
        const float resolvedFallback = std::isfinite(fallback) ? std::max(0.0f, fallback) : kDefaultLocalTransformSmoothingSpeed;
        if (!std::isfinite(value)) {
            return resolvedFallback;
        }
        return std::max(0.0f, value);
    }

    [[nodiscard]] inline float correctionStrengthForFinger(std::size_t fingerIndex, float surfaceAimStrength, float thumbOppositionStrength)
    {
        return fingerIndex == 0 ? thumbOppositionStrength : surfaceAimStrength;
    }

    [[nodiscard]] inline bool shouldApplySurfaceAimCorrection(std::size_t fingerIndex, bool alternateThumbPlaneCorrection)
    {
        return !(fingerIndex == 0 && alternateThumbPlaneCorrection);
    }

    [[nodiscard]] inline float alternateThumbSegmentCorrectionStrength(std::size_t segment, float strength)
    {
        static constexpr float kSegmentWeights[3]{ 0.25f, 0.55f, 0.85f };
        if (segment >= 3) {
            return 0.0f;
        }
        return sanitizeUnitStrength(strength, kDefaultThumbAlternateCurveStrength) * kSegmentWeights[segment];
    }

    template <class Vector>
    [[nodiscard]] inline float vectorDot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    [[nodiscard]] inline Vector vectorCross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    template <class Vector>
    [[nodiscard]] inline Vector vectorAdd(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    template <class Vector>
    [[nodiscard]] inline Vector vectorSub(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    template <class Vector>
    [[nodiscard]] inline Vector vectorScale(const Vector& value, float scale)
    {
        return Vector{ value.x * scale, value.y * scale, value.z * scale };
    }

    template <class Vector>
    [[nodiscard]] inline Vector normalizeVectorOrFallback(const Vector& value, const Vector& fallback)
    {
        const float lengthSquared = vectorDot(value, value);
        if (std::isfinite(lengthSquared) && lengthSquared > 0.000001f) {
            const float invLength = 1.0f / std::sqrt(lengthSquared);
            return vectorScale(value, invLength);
        }

        const float fallbackLengthSquared = vectorDot(fallback, fallback);
        if (std::isfinite(fallbackLengthSquared) && fallbackLengthSquared > 0.000001f) {
            const float invLength = 1.0f / std::sqrt(fallbackLengthSquared);
            return vectorScale(fallback, invLength);
        }

        return Vector{ 1.0f, 0.0f, 0.0f };
    }

    template <class Vector>
    [[nodiscard]] inline Vector alternateThumbPlaneCurlDirection(
        const Vector& openDirection,
        const Vector& alternateCurlNormal,
        float openBlend,
        float maxCurlAngleRadians)
    {
        const Vector normal = normalizeVectorOrFallback(alternateCurlNormal, Vector{ 0.0f, 0.0f, 1.0f });
        const Vector projectedOpen = vectorSub(openDirection, vectorScale(normal, vectorDot(openDirection, normal)));
        const Vector open = normalizeVectorOrFallback(projectedOpen, openDirection);
        const float blend = std::clamp(std::isfinite(openBlend) ? openBlend : 1.0f, 0.0f, 1.0f);
        const float angle = (1.0f - blend) * (std::isfinite(maxCurlAngleRadians) ? std::max(0.0f, maxCurlAngleRadians) : 0.0f);
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);

        return normalizeVectorOrFallback(vectorAdd(
                                             vectorAdd(
                                                 vectorScale(open, cosTheta),
                                                 vectorScale(vectorCross(normal, open), sinTheta)),
                                             vectorScale(normal, vectorDot(normal, open) * (1.0f - cosTheta))),
            open);
    }

    template <class Transform>
    [[nodiscard]] inline bool sceneTransformHasUsableBasis(const Transform& transform)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }

        return std::isfinite(transform.translate.x) &&
               std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) &&
               std::isfinite(transform.scale);
    }

    [[nodiscard]] inline float exponentialSmoothingAlpha(float speed, float deltaTime)
    {
        if (!std::isfinite(speed) || speed <= 0.0f || !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
            return 1.0f;
        }
        return std::clamp(1.0f - std::exp(-speed * deltaTime), 0.0f, 1.0f);
    }
}

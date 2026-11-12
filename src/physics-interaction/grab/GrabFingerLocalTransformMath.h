#pragma once

#include <algorithm>
#include <array>
#include <bitset>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string_view>
#include <vector>
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/GeneratedGrabFingerCalibration.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

/*
 * ROCK gets convincing wrapped fingers by driving the visible skeleton, not by
 * changing the held-object constraint. The mesh solver still produces scalar
 * joint curls, FRIK supplies the authored local-transform baseline for those
 * curls, and this policy decides when ROCK may add bounded surface-aim
 * corrections to that baseline.
 */

namespace rock::grab_finger_local_transform_math
{
    inline constexpr std::uint16_t kFullFingerLocalTransformMask = 0x7FFF;
    inline constexpr float kDefaultLocalTransformSmoothingSpeed = 14.0f;
    inline constexpr float kDefaultMaxCorrectionDegrees = 35.0f;
    inline constexpr float kDefaultSurfaceAimStrength = 0.75f;
    inline constexpr float kDefaultThumbOppositionStrength = 1.0f;
    inline constexpr float kDefaultThumbAlternateCurveStrength = 0.65f;
    inline constexpr float kMaxSurfaceAimCorrectionRadians = 0.08726646259971647f;

    [[nodiscard]] inline std::uint16_t sanitizeFingerLocalTransformMask(std::uint16_t mask)
    {
        return static_cast<std::uint16_t>(mask & kFullFingerLocalTransformMask);
    }

    [[nodiscard]] inline bool shouldPublishLocalTransformPose(bool enabled, bool poseSolved, bool hasCanonicalHandPose,
        bool hasBaselineApi, bool hasPublishApi)
    {
        return enabled && poseSolved && hasCanonicalHandPose && hasBaselineApi && hasPublishApi;
    }

    [[nodiscard]] inline float correctionStrengthForFinger(std::size_t fingerIndex, float surfaceAimStrength,
        float thumbOppositionStrength)
    {
        return fingerIndex == 0 ? thumbOppositionStrength : surfaceAimStrength;
    }

    [[nodiscard]] inline float surfaceAimSegmentCorrectionWeight(std::size_t fingerIndex, std::size_t segment)
    {
        static constexpr float kThumbWeights[3]{ 0.08f, 0.18f, 0.30f };
        static constexpr float kSurfaceFingerWeights[3]{ 0.05f, 0.12f, 0.20f };
        if (segment >= 3) {
            return 0.0f;
        }
        if (fingerIndex == 0) {
            return kThumbWeights[segment];
        }
        if (fingerIndex >= 2 && fingerIndex < 5) {
            return kSurfaceFingerWeights[segment];
        }
        return 0.0f;
    }

    [[nodiscard]] inline float boundedSurfaceAimCorrectionRadians(float rawAngleRadians, float strength,
        float maxCorrectionRadians, std::size_t fingerIndex, std::size_t segment)
    {
        if (!std::isfinite(rawAngleRadians) || rawAngleRadians <= 0.0f || !std::isfinite(maxCorrectionRadians) ||
            maxCorrectionRadians <= 0.0f) {
            return 0.0f;
        }

        const float segmentWeight = surfaceAimSegmentCorrectionWeight(fingerIndex, segment);
        if (segmentWeight <= 0.0f) {
            return 0.0f;
        }

        if (strength <= 0.0f) {
            return 0.0f;
        }

        return std::min(
            std::min(rawAngleRadians * strength * segmentWeight, maxCorrectionRadians * segmentWeight),
            kMaxSurfaceAimCorrectionRadians);
    }

    [[nodiscard]] inline bool shouldApplySurfaceAimCorrection(std::size_t fingerIndex,
        bool alternateThumbPlaneCorrection, bool thumbSurfaceFollowAllowed = true)
    {
        if (alternateThumbPlaneCorrection) {
            return false;
        }
        if (fingerIndex == 0) {
            return thumbSurfaceFollowAllowed;
        }
        return fingerIndex >= 2 && fingerIndex < 5;
    }

    [[nodiscard]] inline bool shouldApplyAlternateThumbLocalCorrection(bool usedAlternateThumbCurve,
        bool usedAlternateThumbSurfaceHit)
    {
        return usedAlternateThumbCurve && usedAlternateThumbSurfaceHit;
    }

    [[nodiscard]] inline float alternateThumbSegmentCorrectionStrength(std::size_t segment, float strength)
    {
        static constexpr float kSegmentWeights[3]{ 0.25f, 0.55f, 0.85f };
        if (segment >= 3) {
            return 0.0f;
        }
        return strength * kSegmentWeights[segment];
    }

    template <class Vector>
    [[nodiscard]] inline float vectorDot(const Vector& lhs, const Vector& rhs)
    {
        return vector_math::dot(lhs, rhs);
    }

    template <class Vector>
    [[nodiscard]] inline Vector vectorCross(const Vector& lhs, const Vector& rhs)
    {
        return vector_math::cross(lhs, rhs);
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
    [[nodiscard]] inline Vector alternateThumbPlaneCurlDirection(const Vector& openDirection,
        const Vector& alternateCurlNormal, float openBlend, float maxCurlAngleRadians)
    {
        const Vector normal = normalizeVectorOrFallback(alternateCurlNormal, Vector{ 0.0f, 0.0f, 1.0f });
        const Vector projectedOpen = vectorSub(openDirection, vectorScale(normal, vectorDot(openDirection, normal)));
        const Vector open = normalizeVectorOrFallback(projectedOpen, openDirection);
        const float blend = std::clamp(std::isfinite(openBlend) ? openBlend : 1.0f, 0.0f, 1.0f);
        const float angle =
            (1.0f - blend) * (std::isfinite(maxCurlAngleRadians) ? std::max(0.0f, maxCurlAngleRadians) : 0.0f);
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);

        return normalizeVectorOrFallback(
            vectorAdd(vectorAdd(vectorScale(open, cosTheta), vectorScale(vectorCross(normal, open), sinTheta)),
                vectorScale(normal, vectorDot(normal, open) * (1.0f - cosTheta))),
            open);
    }

    template <class Vector>
    [[nodiscard]] inline bool thumbAlternateSurfaceGuardAllows(const Vector& thumbTipWorld,
        const Vector& targetAxisWorld, const Vector& surfacePointWorld, const Vector& surfaceNormalWorld,
        float marginGameUnits, bool enabled, bool hasSurface)
    {
        if (!enabled) {
            return true;
        }
        if (!hasSurface || vectorDot(surfaceNormalWorld, surfaceNormalWorld) <= 0.000001f ||
            vectorDot(targetAxisWorld, targetAxisWorld) <= 0.000001f) {
            return false;
        }

        const Vector normal = normalizeVectorOrFallback(surfaceNormalWorld, Vector{ 0.0f, 0.0f, 1.0f });
        const Vector targetAxis = normalizeVectorOrFallback(targetAxisWorld, Vector{ 1.0f, 0.0f, 0.0f });
        const float margin = std::max(0.0f, std::isfinite(marginGameUnits) ? marginGameUnits : 0.0f);
        const float signedTipDistance = vectorDot(normal, vectorSub(thumbTipWorld, surfacePointWorld));
        if (!std::isfinite(signedTipDistance) || signedTipDistance < -margin) {
            return false;
        }

        const float inward = vectorDot(targetAxis, normal);
        if (!std::isfinite(inward) || inward >= 0.0f) {
            return true;
        }

        const Vector toSurface = vectorSub(surfacePointWorld, thumbTipWorld);
        const float travelToSurface = std::sqrt(std::max(0.0f, vectorDot(toSurface, toSurface)));
        const float boundedTravel =
            std::min(std::max(0.0f, travelToSurface), margin + std::max(0.0f, signedTipDistance));
        return signedTipDistance + inward * boundedTravel >= -margin;
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

        return std::isfinite(transform.translate.x) && std::isfinite(transform.translate.y) &&
            std::isfinite(transform.translate.z) && std::isfinite(transform.scale);
    }

    inline constexpr float kMaxPublishedFingerLocalTranslationGameUnits = 64.0f;
    inline constexpr float kMinPublishedFingerLocalScale = 0.01f;
    inline constexpr float kMaxPublishedFingerLocalScale = 4.0f;
    inline constexpr float kPublishedFingerRotationNormTolerance = 0.1f;
    inline constexpr float kPublishedFingerRotationDotTolerance = 0.1f;
    inline constexpr float kPublishedFingerRotationDeterminantTolerance = 0.2f;

    enum class FingerLocalTransformSafetyFailure : std::uint8_t
    {
        None,
        NonFinite,
        TranslationMagnitude,
        ScaleMagnitude,
        RotationNorm,
        RotationOrthogonality,
        RotationDeterminant,
    };

    [[nodiscard]] inline const char* fingerLocalTransformSafetyFailureName(FingerLocalTransformSafetyFailure failure)
    {
        switch (failure) {
        case FingerLocalTransformSafetyFailure::None:
            return "none";
        case FingerLocalTransformSafetyFailure::NonFinite:
            return "non-finite";
        case FingerLocalTransformSafetyFailure::TranslationMagnitude:
            return "translation-magnitude";
        case FingerLocalTransformSafetyFailure::ScaleMagnitude:
            return "scale-magnitude";
        case FingerLocalTransformSafetyFailure::RotationNorm:
            return "rotation-norm";
        case FingerLocalTransformSafetyFailure::RotationOrthogonality:
            return "rotation-orthogonality";
        case FingerLocalTransformSafetyFailure::RotationDeterminant:
            return "rotation-determinant";
        }
        return "unknown";
    }

    template <class Transform>
    [[nodiscard]] inline float storedRotationRowDot(const Transform& transform, std::size_t left, std::size_t right)
    {
        float result = 0.0f;
        for (std::size_t column = 0; column < 3; ++column) {
            result += transform.rotate.entry[left][column] * transform.rotate.entry[right][column];
        }
        return result;
    }

    template <class Transform>
    [[nodiscard]] inline float storedRotationDeterminant(const Transform& transform)
    {
        const auto& r = transform.rotate.entry;
        return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
            r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    }

    template <class Transform>
    [[nodiscard]] inline FingerLocalTransformSafetyFailure inspectStoredRotationBasis(const Transform& transform)
    {
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return FingerLocalTransformSafetyFailure::NonFinite;
                }
            }
            if (std::abs(storedRotationRowDot(transform, row, row) - 1.0f) > kPublishedFingerRotationNormTolerance) {
                return FingerLocalTransformSafetyFailure::RotationNorm;
            }
        }

        if (std::abs(storedRotationRowDot(transform, 0, 1)) > kPublishedFingerRotationDotTolerance ||
            std::abs(storedRotationRowDot(transform, 0, 2)) > kPublishedFingerRotationDotTolerance ||
            std::abs(storedRotationRowDot(transform, 1, 2)) > kPublishedFingerRotationDotTolerance) {
            return FingerLocalTransformSafetyFailure::RotationOrthogonality;
        }

        const float determinant = storedRotationDeterminant(transform);
        if (!std::isfinite(determinant) ||
            std::abs(determinant - 1.0f) > kPublishedFingerRotationDeterminantTolerance) {
            return FingerLocalTransformSafetyFailure::RotationDeterminant;
        }
        return FingerLocalTransformSafetyFailure::None;
    }

    template <class Transform>
    [[nodiscard]] inline FingerLocalTransformSafetyFailure inspectFingerLocalTransformForPublication(
        const Transform& transform)
    {
        if (!sceneTransformHasUsableBasis(transform)) {
            return FingerLocalTransformSafetyFailure::NonFinite;
        }
        if (transform.scale < kMinPublishedFingerLocalScale || transform.scale > kMaxPublishedFingerLocalScale) {
            return FingerLocalTransformSafetyFailure::ScaleMagnitude;
        }

        const float translationLengthSquared = transform.translate.x * transform.translate.x +
            transform.translate.y * transform.translate.y + transform.translate.z * transform.translate.z;
        constexpr float kMaxTranslationSquared =
            kMaxPublishedFingerLocalTranslationGameUnits * kMaxPublishedFingerLocalTranslationGameUnits;
        if (!std::isfinite(translationLengthSquared) || translationLengthSquared > kMaxTranslationSquared) {
            return FingerLocalTransformSafetyFailure::TranslationMagnitude;
        }
        return inspectStoredRotationBasis(transform);
    }

    [[nodiscard]] inline float exponentialSmoothingAlpha(float speed, float deltaTime)
    {
        if (!std::isfinite(speed) || speed <= 0.0f || !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
            return 1.0f;
        }
        return std::clamp(1.0f - std::exp(-speed * deltaTime), 0.0f, 1.0f);
    }
}

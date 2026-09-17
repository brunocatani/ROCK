#pragma once

#include "RE/NetImmerse/NiPoint.h"
#include "physics-interaction/VectorMath.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace rock::grab_pinch_pocket_policy
{
    // The old compact envelope admitted at most 10 * 10 * 4 cubic game units.
    inline constexpr float kDefaultMaxVolumeCubicGameUnits = 400.0f;
    inline constexpr float kMaxVolumeCubicGameUnits = 1000000.0f;
    inline constexpr float kDefaultMaxPocketDistanceGameUnits = 8.0f;
    inline constexpr float kDefaultMinFingerGapGameUnits = 1.0f;
    inline constexpr float kDefaultMaxFingerGapGameUnits = 12.0f;
    inline constexpr float kDefaultThumbIndexMaxOpenValue = 0.45f;
    inline constexpr float kDefaultOtherFingerCurlValue = 0.20f;
    inline constexpr float kDefaultSurfaceInsetGameUnits = 0.5f;
    inline constexpr float kDefaultDetectionDirectionHandspaceX = 1.0f;
    inline constexpr float kDefaultDetectionDirectionHandspaceY = 0.0f;
    inline constexpr float kDefaultDetectionDirectionHandspaceZ = 0.0f;
    inline constexpr float kDefaultDetectionAxisBlend = 0.65f;

    struct Config
    {
        bool enabled = true;
        float maxVolumeCubicGameUnits = kDefaultMaxVolumeCubicGameUnits;
        float maxPocketDistanceGameUnits = kDefaultMaxPocketDistanceGameUnits;
        float minFingerGapGameUnits = kDefaultMinFingerGapGameUnits;
        float maxFingerGapGameUnits = kDefaultMaxFingerGapGameUnits;
        float thumbIndexMaxOpenValue = kDefaultThumbIndexMaxOpenValue;
        float otherFingerCurlValue = kDefaultOtherFingerCurlValue;
        float surfaceInsetGameUnits = kDefaultSurfaceInsetGameUnits;
        RE::NiPoint3 detectionDirectionHandspace{
            kDefaultDetectionDirectionHandspaceX,
            kDefaultDetectionDirectionHandspaceY,
            kDefaultDetectionDirectionHandspaceZ
        };
        float detectionAxisBlend = kDefaultDetectionAxisBlend;
    };

    struct MeshExtentMetrics
    {
        RE::NiPoint3 minLocal{};
        RE::NiPoint3 maxLocal{};
        float minExtentGameUnits = 0.0f;
        float middleExtentGameUnits = 0.0f;
        float maxExtentGameUnits = 0.0f;
        float boundsVolumeCubicGameUnits = 0.0f;
        bool valid = false;
    };

    struct ObjectDecisionInput
    {
        Config config{};
        MeshExtentMetrics mesh{};
        bool closeGrab = false;
        bool handPocketOnlyGrab = false;
        bool looseWeaponGrab = false;
        bool ownerMatchesResolvedBody = false;
        bool hasFingerSnapshot = false;
        bool hasPinchSurface = false;
        bool multipleAcceptedBodies = false;
        float thumbIndexGapGameUnits = 0.0f;
        float pocketToSurfaceDistanceGameUnits = std::numeric_limits<float>::infinity();
    };

    struct ObjectDecision
    {
        const char* reason = "notEvaluated";
        bool accept = false;
        bool retryable = false;
    };

    struct StablePinchFingerPose
    {
        std::array<float, 5> values{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        std::array<float, 15> jointValues{};
    };

    [[nodiscard]] inline float finiteOr(float value, float fallback)
    {
        return std::isfinite(value) ? value : fallback;
    }

    [[nodiscard]] inline Config sanitizeConfig(Config config)
    {
        config.maxVolumeCubicGameUnits = std::clamp(finiteOr(config.maxVolumeCubicGameUnits, kDefaultMaxVolumeCubicGameUnits), 0.001f, kMaxVolumeCubicGameUnits);
        config.maxPocketDistanceGameUnits = std::clamp(finiteOr(config.maxPocketDistanceGameUnits, kDefaultMaxPocketDistanceGameUnits), 0.1f, 80.0f);
        config.minFingerGapGameUnits = std::clamp(finiteOr(config.minFingerGapGameUnits, kDefaultMinFingerGapGameUnits), 0.0f, 40.0f);
        config.maxFingerGapGameUnits = std::clamp(finiteOr(config.maxFingerGapGameUnits, kDefaultMaxFingerGapGameUnits), 0.1f, 80.0f);
        if (config.maxFingerGapGameUnits < config.minFingerGapGameUnits) {
            config.maxFingerGapGameUnits = config.minFingerGapGameUnits;
        }
        config.thumbIndexMaxOpenValue = std::clamp(finiteOr(config.thumbIndexMaxOpenValue, kDefaultThumbIndexMaxOpenValue), 0.0f, 1.0f);
        config.otherFingerCurlValue = std::clamp(finiteOr(config.otherFingerCurlValue, kDefaultOtherFingerCurlValue), 0.0f, 1.0f);
        config.surfaceInsetGameUnits = std::clamp(finiteOr(config.surfaceInsetGameUnits, kDefaultSurfaceInsetGameUnits), 0.0f, 8.0f);
        const float directionLenSq = vector_math::lengthSquared(config.detectionDirectionHandspace);
        if (!std::isfinite(directionLenSq) || directionLenSq <= 0.000001f) {
            config.detectionDirectionHandspace = RE::NiPoint3{
                kDefaultDetectionDirectionHandspaceX,
                kDefaultDetectionDirectionHandspaceY,
                kDefaultDetectionDirectionHandspaceZ
            };
        } else {
            const float invDirectionLen = 1.0f / std::sqrt(directionLenSq);
            config.detectionDirectionHandspace = RE::NiPoint3{
                config.detectionDirectionHandspace.x * invDirectionLen,
                config.detectionDirectionHandspace.y * invDirectionLen,
                config.detectionDirectionHandspace.z * invDirectionLen
            };
        }
        config.detectionAxisBlend = std::clamp(finiteOr(config.detectionAxisBlend, kDefaultDetectionAxisBlend), 0.0f, 1.0f);
        return config;
    }

    [[nodiscard]] inline bool isFinitePoint(const RE::NiPoint3& point)
    {
        return vector_math::hasFiniteComponents(point);
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value)
    {
        return vector_math::lengthSquared(value);
    }

    [[nodiscard]] inline float distance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        const RE::NiPoint3 delta{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        return std::sqrt((std::max)(0.0f, lengthSquared(delta)));
    }

    [[nodiscard]] inline RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lenSq = lengthSquared(value);
        if (!std::isfinite(lenSq) || lenSq <= 0.000001f) {
            const float fallbackLenSq = lengthSquared(fallback);
            if (!std::isfinite(fallbackLenSq) || fallbackLenSq <= 0.000001f) {
                return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
            }
            const float invFallbackLen = 1.0f / std::sqrt(fallbackLenSq);
            return RE::NiPoint3{ fallback.x * invFallbackLen, fallback.y * invFallbackLen, fallback.z * invFallbackLen };
        }

        const float invLen = 1.0f / std::sqrt(lenSq);
        return RE::NiPoint3{ value.x * invLen, value.y * invLen, value.z * invLen };
    }

    [[nodiscard]] inline RE::NiPoint3 closestPointOnSegment(const RE::NiPoint3& a, const RE::NiPoint3& b, const RE::NiPoint3& point)
    {
        const RE::NiPoint3 ab{ b.x - a.x, b.y - a.y, b.z - a.z };
        const float abLenSq = lengthSquared(ab);
        if (!std::isfinite(abLenSq) || abLenSq <= 0.000001f) {
            return a;
        }

        const RE::NiPoint3 ap{ point.x - a.x, point.y - a.y, point.z - a.z };
        const float t = std::clamp((ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / abLenSq, 0.0f, 1.0f);
        return RE::NiPoint3{ a.x + ab.x * t, a.y + ab.y * t, a.z + ab.z * t };
    }

    [[nodiscard]] inline MeshExtentMetrics computeMeshExtentsFromBounds(
        const RE::NiPoint3& minLocal,
        const RE::NiPoint3& maxLocal,
        float objectScale)
    {
        MeshExtentMetrics metrics{};
        if (!isFinitePoint(minLocal) || !isFinitePoint(maxLocal) ||
            !std::isfinite(objectScale) || objectScale <= 0.0f) {
            return metrics;
        }

        const float scale = objectScale;
        std::array<float, 3> extents{
            std::abs(maxLocal.x - minLocal.x) * scale,
            std::abs(maxLocal.y - minLocal.y) * scale,
            std::abs(maxLocal.z - minLocal.z) * scale,
        };
        std::sort(extents.begin(), extents.end());
        metrics.minLocal = minLocal;
        metrics.maxLocal = maxLocal;
        metrics.minExtentGameUnits = extents[0];
        metrics.middleExtentGameUnits = extents[1];
        metrics.maxExtentGameUnits = extents[2];
        // Object-local enclosing bounds include every captured mesh part.
        // This is deliberately a bounding-volume estimate: open and hollow
        // render meshes do not provide a dependable signed solid volume.
        metrics.boundsVolumeCubicGameUnits = static_cast<float>(
            static_cast<double>(extents[0]) * extents[1] * extents[2]);
        metrics.valid =
            std::isfinite(metrics.minExtentGameUnits) &&
            std::isfinite(metrics.middleExtentGameUnits) &&
            std::isfinite(metrics.maxExtentGameUnits) &&
            std::isfinite(metrics.boundsVolumeCubicGameUnits) &&
            metrics.maxExtentGameUnits > 0.0001f;
        return metrics;
    }

    template <class TriangleRange>
    [[nodiscard]] MeshExtentMetrics computeMeshExtents(const TriangleRange& triangles, float objectScale)
    {
        bool anyPoint = false;
        RE::NiPoint3 minLocal{
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)(),
            (std::numeric_limits<float>::max)()
        };
        RE::NiPoint3 maxLocal{
            (std::numeric_limits<float>::lowest)(),
            (std::numeric_limits<float>::lowest)(),
            (std::numeric_limits<float>::lowest)()
        };

        auto includePoint = [&](const RE::NiPoint3& point) {
            anyPoint = true;
            minLocal.x = (std::min)(minLocal.x, point.x);
            minLocal.y = (std::min)(minLocal.y, point.y);
            minLocal.z = (std::min)(minLocal.z, point.z);
            maxLocal.x = (std::max)(maxLocal.x, point.x);
            maxLocal.y = (std::max)(maxLocal.y, point.y);
            maxLocal.z = (std::max)(maxLocal.z, point.z);
        };

        for (const auto& triangle : triangles) {
            if (!isFinitePoint(triangle.v0) || !isFinitePoint(triangle.v1) || !isFinitePoint(triangle.v2)) {
                return {};
            }
            includePoint(triangle.v0);
            includePoint(triangle.v1);
            includePoint(triangle.v2);
        }

        return anyPoint ? computeMeshExtentsFromBounds(minLocal, maxLocal, objectScale) : MeshExtentMetrics{};
    }

    [[nodiscard]] inline ObjectDecision evaluateObject(const ObjectDecisionInput& input)
    {
        const Config config = sanitizeConfig(input.config);
        ObjectDecision decision{};

        if (!config.enabled) {
            decision.reason = "disabled";
            return decision;
        }
        if (!input.closeGrab) {
            decision.reason = "notCloseGrab";
            return decision;
        }
        if (input.multipleAcceptedBodies) {
            decision.reason = "multiBody";
            return decision;
        }
        if (!input.ownerMatchesResolvedBody) {
            decision.reason = "ownerMismatch";
            return decision;
        }
        if (!input.hasFingerSnapshot) {
            decision.reason = "missingFingerSnapshot";
            decision.retryable = true;
            return decision;
        }
        if (!input.mesh.valid) {
            decision.reason = "noMeshExtents";
            return decision;
        }

        if (!std::isfinite(input.mesh.boundsVolumeCubicGameUnits) || input.mesh.boundsVolumeCubicGameUnits < 0.0f) {
            decision.reason = "invalidObjectVolume";
            return decision;
        }
        if (input.mesh.boundsVolumeCubicGameUnits > config.maxVolumeCubicGameUnits) {
            decision.reason = "objectVolumeTooLarge";
            return decision;
        }

        if (!std::isfinite(input.thumbIndexGapGameUnits) ||
            input.thumbIndexGapGameUnits < config.minFingerGapGameUnits ||
            input.thumbIndexGapGameUnits > config.maxFingerGapGameUnits) {
            decision.reason = "fingerGapRejected";
            decision.retryable = true;
            return decision;
        }
        if (!input.hasPinchSurface) {
            decision.reason = "noPinchSurface";
            decision.retryable = true;
            return decision;
        }
        if (!std::isfinite(input.pocketToSurfaceDistanceGameUnits) ||
            input.pocketToSurfaceDistanceGameUnits > config.maxPocketDistanceGameUnits) {
            decision.reason = "surfaceTooFarFromPocket";
            decision.retryable = true;
            return decision;
        }

        decision.accept = true;
        decision.reason = "pinchObjectVolume";
        return decision;
    }

    [[nodiscard]] inline float oppositionHalfWidthGameUnits(const MeshExtentMetrics& metrics, float configuredSurfaceInsetGameUnits)
    {
        const float thinHalfWidth = std::clamp(finiteOr(metrics.middleExtentGameUnits, 0.0f) * 0.5f, 0.35f, 2.5f);
        return (std::max)(thinHalfWidth, std::clamp(finiteOr(configuredSurfaceInsetGameUnits, kDefaultSurfaceInsetGameUnits), 0.0f, 8.0f));
    }

    [[nodiscard]] inline StablePinchFingerPose buildStableOppositionFingerPose(
        Config rawConfig,
        float minFingerValue,
        std::size_t opposedFingerIndex)
    {
        const Config config = sanitizeConfig(rawConfig);
        const float minValue = std::clamp(finiteOr(minFingerValue, 0.2f), 0.0f, 1.0f);
        const float pinchValue = std::clamp(config.thumbIndexMaxOpenValue, minValue, 1.0f);
        const float otherValue = std::clamp(config.otherFingerCurlValue, 0.0f, 1.0f);
        const std::size_t opposedFinger =
            std::clamp(opposedFingerIndex,
                static_cast<std::size_t>(1),
                static_cast<std::size_t>(4));

        StablePinchFingerPose pose{};
        pose.values = { otherValue, otherValue, otherValue, otherValue, otherValue };
        pose.values[0] = pinchValue;
        pose.values[opposedFinger] = pinchValue;

        const float pinchClosed = 1.0f - pinchValue;
        pose.jointValues[0] = std::clamp(pinchValue + pinchClosed * 0.30f, minValue, 1.0f);
        pose.jointValues[1] = std::clamp(pinchValue + pinchClosed * 0.12f, minValue, 1.0f);
        pose.jointValues[2] = std::clamp(pinchValue, minValue, 1.0f);

        const float otherClosed = 1.0f - otherValue;
        for (std::size_t finger = 1; finger < pose.values.size(); ++finger) {
            const std::size_t base = finger * 3;
            pose.jointValues[base + 0] = std::clamp(otherValue + otherClosed * 0.25f, 0.0f, 1.0f);
            pose.jointValues[base + 1] = std::clamp(otherValue, 0.0f, 1.0f);
            pose.jointValues[base + 2] = std::clamp(otherValue - otherClosed * 0.15f, 0.0f, 1.0f);
        }

        const std::size_t opposedBase = opposedFinger * 3;
        pose.jointValues[opposedBase + 0] =
            std::clamp(
                pinchValue + pinchClosed * 0.20f,
                minValue,
                1.0f);
        pose.jointValues[opposedBase + 1] =
            std::clamp(pinchValue, minValue, 1.0f);
        pose.jointValues[opposedBase + 2] =
            std::clamp(
                pinchValue - pinchClosed * 0.08f,
                minValue,
                1.0f);

        return pose;
    }

    [[nodiscard]] inline StablePinchFingerPose buildStablePinchFingerPose(Config rawConfig, float minFingerValue)
    {
        return buildStableOppositionFingerPose(
            rawConfig,
            minFingerValue,
            1);
    }
}

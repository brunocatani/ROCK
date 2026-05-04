#pragma once

/*
 * ROCK needs the touched object point to be captured once and then held in the
 * object's own body space. HIGGS' dynamic grab path keeps one coherent
 * object-in-hand relationship after selection; this layer gives ROCK a richer
 * contact source by fitting a small palm-facing patch, while leaving Havok body
 * memory and FRIK runtime ownership outside the pure math.
 */

#include "TransformMath.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace frik::rock::grab_contact_patch_math
{
    template <class Vector>
    struct GrabContactPatchSample
    {
        std::uint32_t bodyId = 0x7FFF'FFFF;
        Vector point{};
        Vector normal{};
        float fraction = 1.0f;
        bool accepted = false;
        const char* rejectionReason = "none";
    };

    template <class Vector>
    struct GrabContactPatchResult
    {
        Vector contactPoint{};
        Vector normal{};
        Vector tangent{};
        Vector bitangent{};
        std::size_t hitCount = 0;
        std::size_t rejectedSampleCount = 0;
        float confidence = 0.0f;
        float meshSnapDeltaGameUnits = 0.0f;
        bool valid = false;
        bool orientationReliable = false;
        const char* fallbackReason = "uninitialized";
    };

    enum class GrabContactPatchPivotSource
    {
        None,
        SelectedHit,
        MeshSnap,
        PatchSample
    };

    template <class Vector>
    struct GrabContactPatchPivotDecision
    {
        Vector point{};
        GrabContactPatchPivotSource source = GrabContactPatchPivotSource::None;
        bool valid = false;
        bool replaceSelectedPoint = false;
        float selectionDeltaGameUnits = (std::numeric_limits<float>::max)();
        const char* reason = "uninitialized";
    };

    template <class Vector>
    inline Vector add(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    template <class Vector>
    inline Vector sub(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    template <class Vector>
    inline Vector mul(const Vector& value, float scale)
    {
        return Vector{ value.x * scale, value.y * scale, value.z * scale };
    }

    template <class Vector>
    inline float dot(const Vector& lhs, const Vector& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    template <class Vector>
    inline Vector cross(const Vector& lhs, const Vector& rhs)
    {
        return Vector{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return dot(value, value);
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(lengthSquared(value));
    }

    template <class Vector>
    inline Vector normalizeOrZero(const Vector& value)
    {
        const float len = length(value);
        if (len <= 1.0e-6f || !std::isfinite(len)) {
            return Vector{};
        }
        return mul(value, 1.0f / len);
    }

    template <class Vector>
    inline Vector negate(const Vector& value)
    {
        return Vector{ -value.x, -value.y, -value.z };
    }

    template <class Vector>
    inline Vector projectOntoPlane(const Vector& value, const Vector& normal)
    {
        return sub(value, mul(normal, dot(value, normal)));
    }

    template <class Vector>
    inline Vector stablePerpendicular(const Vector& normal)
    {
        const Vector xAxis{ 1.0f, 0.0f, 0.0f };
        const Vector yAxis{ 0.0f, 1.0f, 0.0f };
        const Vector zAxis{ 0.0f, 0.0f, 1.0f };
        const Vector hint = std::fabs(dot(normal, xAxis)) < 0.85f ? xAxis : (std::fabs(dot(normal, yAxis)) < 0.85f ? yAxis : zAxis);
        return normalizeOrZero(projectOntoPlane(hint, normal));
    }

    template <class Vector>
    inline Vector orientNormalTowardPalm(const Vector& normal, const Vector& palmNormal)
    {
        Vector oriented = normalizeOrZero(normal);
        const Vector palm = normalizeOrZero(palmNormal);
        if (lengthSquared(oriented) <= 0.0f || lengthSquared(palm) <= 0.0f) {
            return oriented;
        }
        if (dot(oriented, palm) > 0.0f) {
            oriented = negate(oriented);
        }
        return oriented;
    }

    inline const char* pivotSourceName(GrabContactPatchPivotSource source)
    {
        switch (source) {
        case GrabContactPatchPivotSource::SelectedHit:
            return "selectedHit";
        case GrabContactPatchPivotSource::MeshSnap:
            return "meshSnap";
        case GrabContactPatchPivotSource::PatchSample:
            return "patchSample";
        default:
            return "none";
        }
    }

    template <class Vector>
    inline Vector choosePrincipalPatchTangent(const std::vector<Vector>& points, const Vector& centroid, const Vector& normal, const Vector& tangentHint)
    {
        Vector axisT = normalizeOrZero(projectOntoPlane(tangentHint, normal));
        if (lengthSquared(axisT) <= 0.0f) {
            axisT = stablePerpendicular(normal);
        }
        Vector axisB = normalizeOrZero(cross(normal, axisT));
        if (lengthSquared(axisB) <= 0.0f) {
            return axisT;
        }

        float uu = 0.0f;
        float uv = 0.0f;
        float vv = 0.0f;
        for (const auto& point : points) {
            const Vector delta = sub(point, centroid);
            const float u = dot(delta, axisT);
            const float v = dot(delta, axisB);
            uu += u * u;
            uv += u * v;
            vv += v * v;
        }

        if (uu + vv <= 1.0e-6f) {
            return axisT;
        }

        const float angle = 0.5f * std::atan2(2.0f * uv, uu - vv);
        Vector tangent = add(mul(axisT, std::cos(angle)), mul(axisB, std::sin(angle)));
        tangent = normalizeOrZero(tangent);
        if (lengthSquared(tangent) <= 0.0f) {
            tangent = axisT;
        }
        return tangent;
    }

    template <class Vector>
    inline Vector clampPointToPatchSpan(const Vector& point, const std::vector<Vector>& samples, const Vector& centroid, const Vector& tangent, const Vector& bitangent)
    {
        if (samples.size() < 2 || lengthSquared(tangent) <= 0.0f || lengthSquared(bitangent) <= 0.0f) {
            return point;
        }

        float minT = (std::numeric_limits<float>::max)();
        float maxT = -(std::numeric_limits<float>::max)();
        float minB = (std::numeric_limits<float>::max)();
        float maxB = -(std::numeric_limits<float>::max)();
        for (const auto& sample : samples) {
            const Vector delta = sub(sample, centroid);
            const float t = dot(delta, tangent);
            const float b = dot(delta, bitangent);
            minT = (std::min)(minT, t);
            maxT = (std::max)(maxT, t);
            minB = (std::min)(minB, b);
            maxB = (std::max)(maxB, b);
        }

        const Vector delta = sub(point, centroid);
        const float clampedT = std::clamp(dot(delta, tangent), minT, maxT);
        const float clampedB = std::clamp(dot(delta, bitangent), minB, maxB);
        return add(centroid, add(mul(tangent, clampedT), mul(bitangent, clampedB)));
    }

    template <class Vector>
    inline GrabContactPatchResult<Vector> fitContactPatch(const std::vector<GrabContactPatchSample<Vector>>& samples,
        const Vector& palmPoint,
        const Vector& palmNormal,
        const Vector& tangentHint,
        float maxNormalAngleDegrees)
    {
        GrabContactPatchResult<Vector> result{};
        result.fallbackReason = "noAcceptedHits";

        std::vector<Vector> points;
        std::vector<Vector> normals;
        points.reserve(samples.size());
        normals.reserve(samples.size());

        for (const auto& sample : samples) {
            if (!sample.accepted) {
                ++result.rejectedSampleCount;
                continue;
            }

            const Vector orientedNormal = orientNormalTowardPalm(sample.normal, palmNormal);
            if (lengthSquared(orientedNormal) <= 0.0f) {
                ++result.rejectedSampleCount;
                continue;
            }

            points.push_back(sample.point);
            normals.push_back(orientedNormal);
        }

        if (points.empty()) {
            return result;
        }

        Vector averageNormal{};
        for (const auto& normal : normals) {
            averageNormal = add(averageNormal, normal);
        }
        averageNormal = normalizeOrZero(averageNormal);
        if (lengthSquared(averageNormal) <= 0.0f) {
            averageNormal = normals.front();
        }

        const float clampedAngle = std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);
        std::vector<Vector> coherentPoints;
        std::vector<Vector> coherentNormals;
        coherentPoints.reserve(points.size());
        coherentNormals.reserve(normals.size());
        for (std::size_t i = 0; i < points.size(); ++i) {
            if (dot(normals[i], averageNormal) >= minNormalDot) {
                coherentPoints.push_back(points[i]);
                coherentNormals.push_back(normals[i]);
            } else {
                ++result.rejectedSampleCount;
            }
        }

        if (coherentPoints.empty()) {
            return result;
        }

        Vector centroid{};
        for (const auto& point : coherentPoints) {
            centroid = add(centroid, point);
        }
        centroid = mul(centroid, 1.0f / static_cast<float>(coherentPoints.size()));

        Vector normal{};
        for (const auto& coherentNormal : coherentNormals) {
            normal = add(normal, coherentNormal);
        }
        normal = normalizeOrZero(normal);
        if (lengthSquared(normal) <= 0.0f) {
            normal = averageNormal;
        }

        if (coherentPoints.size() >= 3) {
            Vector tripletNormalSum = normal;
            for (std::size_t i = 0; i < coherentPoints.size(); ++i) {
                for (std::size_t j = i + 1; j < coherentPoints.size(); ++j) {
                    for (std::size_t k = j + 1; k < coherentPoints.size(); ++k) {
                        Vector tripletNormal = cross(sub(coherentPoints[j], coherentPoints[i]), sub(coherentPoints[k], coherentPoints[i]));
                        const float area = length(tripletNormal);
                        if (area <= 1.0e-5f) {
                            continue;
                        }
                        tripletNormal = mul(tripletNormal, 1.0f / area);
                        if (dot(tripletNormal, normal) < 0.0f) {
                            tripletNormal = negate(tripletNormal);
                        }
                        tripletNormalSum = add(tripletNormalSum, mul(tripletNormal, area));
                    }
                }
            }
            const Vector fittedNormal = normalizeOrZero(tripletNormalSum);
            if (lengthSquared(fittedNormal) > 0.0f) {
                normal = fittedNormal;
            }
        }

        Vector tangent = choosePrincipalPatchTangent(coherentPoints, centroid, normal, tangentHint);
        Vector bitangent = normalizeOrZero(cross(normal, tangent));
        if (lengthSquared(bitangent) <= 0.0f) {
            tangent = stablePerpendicular(normal);
            bitangent = normalizeOrZero(cross(normal, tangent));
        }

        const Vector projectedPalm = sub(palmPoint, mul(normal, dot(sub(palmPoint, centroid), normal)));
        result.contactPoint = clampPointToPatchSpan(projectedPalm, coherentPoints, centroid, tangent, bitangent);
        result.normal = normal;
        result.tangent = tangent;
        result.bitangent = bitangent;
        result.hitCount = coherentPoints.size();
        result.valid = true;

        if (coherentPoints.size() >= 3) {
            result.confidence = 1.0f;
            result.orientationReliable = true;
            result.fallbackReason = "none";
        } else if (coherentPoints.size() == 2) {
            result.confidence = 0.70f;
            result.orientationReliable = lengthSquared(tangent) > 0.0f && lengthSquared(bitangent) > 0.0f;
            result.fallbackReason = result.orientationReliable ? "twoHitPatch" : "twoHitNoTangent";
        } else {
            result.confidence = 0.35f;
            result.orientationReliable = false;
            result.contactPoint = coherentPoints.front();
            result.fallbackReason = "singleHitPivotOnly";
        }

        return result;
    }

    template <class Vector>
    inline bool contactPatchNormalMatchesSelection(const GrabContactPatchResult<Vector>& patch,
        const Vector& selectionNormal,
        bool hasSelectionNormal,
        const Vector& palmNormal,
        float maxNormalAngleDegrees)
    {
        if (!hasSelectionNormal) {
            return true;
        }
        if (!patch.valid) {
            return false;
        }

        const Vector patchNormal = orientNormalTowardPalm(patch.normal, palmNormal);
        const Vector selectedNormal = orientNormalTowardPalm(selectionNormal, palmNormal);
        if (lengthSquared(patchNormal) <= 0.0f || lengthSquared(selectedNormal) <= 0.0f) {
            return false;
        }

        const float clampedAngle = std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);
        return dot(patchNormal, selectedNormal) >= minNormalDot;
    }

    template <class Vector>
    inline GrabContactPatchPivotDecision<Vector> chooseContactPatchPivotPoint(const GrabContactPatchResult<Vector>& patch,
        const std::vector<GrabContactPatchSample<Vector>>& samples,
        const Vector& selectedHitPoint,
        bool hasSelectedHitPoint,
        const Vector& meshSnapPoint,
        bool hasMeshSnapPoint,
        float maxSelectionDeltaGameUnits,
        float maxSampleNormalAngleDegrees = 45.0f)
    {
        GrabContactPatchPivotDecision<Vector> decision{};

        auto useSelected = [&](const char* reason) {
            decision = {};
            decision.reason = reason;
            if (hasSelectedHitPoint) {
                decision.point = selectedHitPoint;
                decision.source = GrabContactPatchPivotSource::SelectedHit;
                decision.valid = true;
                decision.replaceSelectedPoint = false;
                decision.selectionDeltaGameUnits = 0.0f;
            }
            return decision;
        };

        if (!patch.valid) {
            return useSelected("patchInvalid");
        }

        const auto selectionDeltaAllowed = [&](const Vector& point, float& outDelta) {
            outDelta = hasSelectedHitPoint ? length(sub(point, selectedHitPoint)) : 0.0f;
            return !hasSelectedHitPoint || maxSelectionDeltaGameUnits < 0.0f || outDelta <= maxSelectionDeltaGameUnits;
        };

        if (hasMeshSnapPoint) {
            float selectionDelta = 0.0f;
            if (!selectionDeltaAllowed(meshSnapPoint, selectionDelta)) {
                return useSelected("meshSnapTooFarFromSelection");
            }

            decision.point = meshSnapPoint;
            decision.source = GrabContactPatchPivotSource::MeshSnap;
            decision.valid = true;
            decision.replaceSelectedPoint = true;
            decision.selectionDeltaGameUnits = selectionDelta;
            decision.reason = "meshSnap";
            return decision;
        }

        Vector nearestSample{};
        bool hasNearestSample = false;
        float bestDistanceSq = (std::numeric_limits<float>::max)();
        const float clampedAngle = std::clamp(maxSampleNormalAngleDegrees, 0.0f, 179.0f);
        const float minNormalDot = std::cos(clampedAngle * 3.14159265358979323846f / 180.0f);
        const Vector patchNormal = normalizeOrZero(patch.normal);
        for (const auto& sample : samples) {
            if (!sample.accepted) {
                continue;
            }
            const Vector sampleNormal = normalizeOrZero(sample.normal);
            if (lengthSquared(patchNormal) > 0.0f && lengthSquared(sampleNormal) > 0.0f &&
                std::fabs(dot(sampleNormal, patchNormal)) < minNormalDot) {
                continue;
            }

            const float distSq = lengthSquared(sub(sample.point, patch.contactPoint));
            if (distSq < bestDistanceSq) {
                bestDistanceSq = distSq;
                nearestSample = sample.point;
                hasNearestSample = true;
            }
        }

        if (patch.hitCount >= 2 && hasNearestSample) {
            float selectionDelta = 0.0f;
            if (!selectionDeltaAllowed(nearestSample, selectionDelta)) {
                return useSelected("patchSampleTooFarFromSelection");
            }

            decision.point = nearestSample;
            decision.source = GrabContactPatchPivotSource::PatchSample;
            decision.valid = true;
            decision.replaceSelectedPoint = true;
            decision.selectionDeltaGameUnits = selectionDelta;
            decision.reason = "nearestPatchSample";
            return decision;
        }

        return useSelected(patch.hitCount < 2 ? "insufficientPatchHitsForPivot" : "noValidatedPatchSample");
    }

    template <class Transform, class Vector>
    inline Vector freezePivotBBodyLocal(const Transform& bodyWorld, const Vector& surfacePointWorld)
    {
        return transform_math::worldPointToLocal(bodyWorld, surfacePointWorld);
    }
}

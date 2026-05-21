#pragma once

/*
 * Grab finger helpers are grouped here so pose math and live runtime transform sampling stay in one hand-contact surface.
 */


// ---- GrabFingerPoseMath.h ----

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock::grab_finger_pose_math
{
    /*
     * The first mesh-matched pose pass keeps the expensive choice localized:
     * ROCK computes a stable per-finger curl from visible mesh triangles, while
     * FRIK remains the only system that writes actual finger bones. The solver is
     * deliberately geometric and frame-independent so grab selection, hand lerp,
     * and two-handed grip can share the same result without owning skeleton state.
     */
    template <class Vector>
    struct Triangle
    {
        Vector v0{};
        Vector v1{};
        Vector v2{};
    };

    struct FingerCurlValue
    {
        float value = 0.0f;
        float rawCurveValue = 0.0f;
        bool hit = false;
        float distance = 0.0f;
        float hitPointX = 0.0f;
        float hitPointY = 0.0f;
        float hitPointZ = 0.0f;
        float hitNormalX = 0.0f;
        float hitNormalY = 0.0f;
        float hitNormalZ = 0.0f;
        bool hasHitPoint = false;
        bool hasHitNormal = false;
        bool openedByBehindContact = false;
        enum class HitKind : std::uint8_t
        {
            Miss,
            FrontValid,
            BehindCurlPlane,
            BackSurface,
            Rejected
        } hitKind = HitKind::Miss;
    };

    template <class Vector>
    struct ThumbAwareFingerCurveCurlValue
    {
        FingerCurlValue value{};
        FingerCurlValue primary{};
        FingerCurlValue alternateThumb{};
        bool usedAlternateThumbCurve = false;
    };

    inline bool shouldRunFallbackRayAfterCurveSolve(std::size_t fingerIndex, bool curveHit, bool usedAlternateThumbCurve)
    {
        /*
         * ROCK does not run a second straight-ray solve after choosing the
         * alternate thumb curve. Ray fallback remains valid for non-thumb misses
         * and primary-thumb misses, but an alternate-thumb decision must remain
         * internally consistent with the local transform override that follows.
         */
        if (curveHit) {
            return false;
        }
        return !(fingerIndex == 0 && usedAlternateThumbCurve);
    }

    template <class Vector>
    inline Vector sub(const Vector& a, const Vector& b)
    {
        return Vector{ a.x - b.x, a.y - b.y, a.z - b.z };
    }

    template <class Vector>
    inline Vector add(const Vector& a, const Vector& b)
    {
        return Vector{ a.x + b.x, a.y + b.y, a.z + b.z };
    }

    template <class Vector>
    inline Vector scale(const Vector& value, float scalar)
    {
        return Vector{ value.x * scalar, value.y * scalar, value.z * scalar };
    }

    template <class Vector>
    inline Vector cross(const Vector& a, const Vector& b)
    {
        return Vector{ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
    }

    template <class Vector>
    inline float dot(const Vector& a, const Vector& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(dot(value, value));
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return dot(value, value);
    }

    template <class Vector>
    inline Vector normalize(const Vector& value)
    {
        const float len = length(value);
        if (len <= 0.000001f) {
            return Vector{ 1.0f, 0.0f, 0.0f };
        }
        const float inv = 1.0f / len;
        return Vector{ value.x * inv, value.y * inv, value.z * inv };
    }

    template <class Vector>
    inline bool hasUsableDirection(const Vector& value)
    {
        return lengthSquared(value) > 0.000001f;
    }

    template <class Vector>
    inline Vector triangleNormal(const Triangle<Vector>& triangle)
    {
        return normalize(cross(sub(triangle.v1, triangle.v0), sub(triangle.v2, triangle.v0)));
    }

    template <class Vector>
    inline std::vector<Triangle<Vector>> filterTrianglesNearPoint(const std::vector<Triangle<Vector>>& triangles, const Vector& point, float maxDistanceSquared)
    {
        std::vector<Triangle<Vector>> result;
        if (!std::isfinite(maxDistanceSquared) || maxDistanceSquared <= 0.0f) {
            return result;
        }

        result.reserve((std::min)(triangles.size(), static_cast<std::size_t>(256)));
        for (const auto& triangle : triangles) {
            const Vector centroid = scale(add(add(triangle.v0, triangle.v1), triangle.v2), 1.0f / 3.0f);
            const float bestVertexOrCentroidDistance = (std::min)({
                lengthSquared(sub(centroid, point)),
                lengthSquared(sub(triangle.v0, point)),
                lengthSquared(sub(triangle.v1, point)),
                lengthSquared(sub(triangle.v2, point)),
            });
            if (bestVertexOrCentroidDistance <= maxDistanceSquared) {
                result.push_back(triangle);
            }
        }
        return result;
    }

    template <class Vector>
    inline float clampedUnitDot(const Vector& a, const Vector& b)
    {
        return std::clamp(dot(normalize(a), normalize(b)), -1.0f, 1.0f);
    }

    template <class Vector>
    inline float signedAngleAroundNormal(const Vector& vector, const Vector& zeroAngleVector, const Vector& normal)
    {
        const Vector v = normalize(vector);
        const Vector zero = normalize(zeroAngleVector);
        float angle = std::acos(clampedUnitDot(v, zero));
        if (dot(normalize(normal), cross(zero, v)) < 0.0f) {
            angle *= -1.0f;
        }
        return angle;
    }

    template <class Vector>
    inline bool planeIntersectsSegment(const Vector& planePoint, const Vector& planeNormal, const Vector& a, const Vector& b, Vector& outPoint)
    {
        const Vector normal = normalize(planeNormal);
        const float da = dot(normal, sub(a, planePoint));
        const float db = dot(normal, sub(b, planePoint));
        if ((da > 0.0f && db > 0.0f) || (da < 0.0f && db < 0.0f)) {
            return false;
        }

        const float denom = da - db;
        if (std::abs(denom) <= 0.000001f) {
            outPoint = a;
            return true;
        }

        const float t = std::clamp(da / denom, 0.0f, 1.0f);
        outPoint = add(a, scale(sub(b, a), t));
        return true;
    }

    template <class Vector>
    inline bool rayTriangleIntersection(const Vector& origin, const Vector& direction, const Triangle<Vector>& triangle, float maxDistance, float& outT)
    {
        constexpr float kRayEpsilon = 0.000001f;
        const Vector edge1 = sub(triangle.v1, triangle.v0);
        const Vector edge2 = sub(triangle.v2, triangle.v0);
        const Vector pvec = cross(direction, edge2);
        const float det = dot(edge1, pvec);
        if (std::abs(det) < kRayEpsilon) {
            return false;
        }

        const float invDet = 1.0f / det;
        const Vector tvec = sub(origin, triangle.v0);
        const float u = dot(tvec, pvec) * invDet;
        if (u < 0.0f || u > 1.0f) {
            return false;
        }

        const Vector qvec = cross(tvec, edge1);
        const float v = dot(direction, qvec) * invDet;
        if (v < 0.0f || u + v > 1.0f) {
            return false;
        }

        const float t = dot(edge2, qvec) * invDet;
        if (t < 0.0f || t > maxDistance) {
            return false;
        }

        outT = t;
        return true;
    }

    template <class Vector>
    inline Vector closestPointOnTriangle(const Vector& point, const Triangle<Vector>& triangle)
    {
        const Vector ab = sub(triangle.v1, triangle.v0);
        const Vector ac = sub(triangle.v2, triangle.v0);
        const Vector ap = sub(point, triangle.v0);
        const float d1 = dot(ab, ap);
        const float d2 = dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) {
            return triangle.v0;
        }

        const Vector bp = sub(point, triangle.v1);
        const float d3 = dot(ab, bp);
        const float d4 = dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) {
            return triangle.v1;
        }

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
            const float v = d1 / (d1 - d3);
            return add(triangle.v0, scale(ab, v));
        }

        const Vector cp = sub(point, triangle.v2);
        const float d5 = dot(ab, cp);
        const float d6 = dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) {
            return triangle.v2;
        }

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
            const float w = d2 / (d2 - d6);
            return add(triangle.v0, scale(ac, w));
        }

        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
            const Vector bc = sub(triangle.v2, triangle.v1);
            const float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return add(triangle.v1, scale(bc, w));
        }

        const float denom = 1.0f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return add(triangle.v0, add(scale(ab, v), scale(ac, w)));
    }

    template <class Vector>
    inline float segmentSegmentDistanceSquared(
        const Vector& p1, const Vector& q1, const Vector& p2, const Vector& q2, float& outFirstSegmentT, float* outSecondSegmentT = nullptr)
    {
        constexpr float kSegmentEpsilon = 0.000001f;
        const Vector d1 = sub(q1, p1);
        const Vector d2 = sub(q2, p2);
        const Vector r = sub(p1, p2);
        const float a = dot(d1, d1);
        const float e = dot(d2, d2);
        const float f = dot(d2, r);

        float s = 0.0f;
        float t = 0.0f;
        if (a <= kSegmentEpsilon && e <= kSegmentEpsilon) {
            outFirstSegmentT = 0.0f;
            return lengthSquared(sub(p1, p2));
        }

        if (a <= kSegmentEpsilon) {
            s = 0.0f;
            t = std::clamp(f / e, 0.0f, 1.0f);
        } else {
            const float c = dot(d1, r);
            if (e <= kSegmentEpsilon) {
                t = 0.0f;
                s = std::clamp(-c / a, 0.0f, 1.0f);
            } else {
                const float b = dot(d1, d2);
                const float denom = a * e - b * b;
                if (denom != 0.0f) {
                    s = std::clamp((b * f - c * e) / denom, 0.0f, 1.0f);
                }
                t = (b * s + f) / e;
                if (t < 0.0f) {
                    t = 0.0f;
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                } else if (t > 1.0f) {
                    t = 1.0f;
                    s = std::clamp((b - c) / a, 0.0f, 1.0f);
                }
            }
        }

        outFirstSegmentT = s;
        if (outSecondSegmentT) {
            *outSecondSegmentT = t;
        }
        const Vector c1 = add(p1, scale(d1, s));
        const Vector c2 = add(p2, scale(d2, t));
        return lengthSquared(sub(c1, c2));
    }

    template <class Vector>
    inline bool probeCapsuleTriangleIntersection(
        const Vector& origin, const Vector& direction, const Triangle<Vector>& triangle, float maxDistance, float probeRadius, float& outT, Vector* outClosestPoint = nullptr)
    {
        if (probeRadius <= 0.0f) {
            return false;
        }

        const Vector segmentEnd = add(origin, scale(direction, maxDistance));
        const float radiusSquared = probeRadius * probeRadius;
        float bestDistanceSquared = (std::numeric_limits<float>::max)();
        float bestT = maxDistance;
        Vector bestClosestPoint{};
        bool bestClosestPointValid = false;

        auto consider = [&](float distanceSquared, float segmentRatio, const Vector& closestPoint) {
            if (distanceSquared < bestDistanceSquared) {
                bestDistanceSquared = distanceSquared;
                bestT = std::clamp(segmentRatio, 0.0f, 1.0f) * maxDistance;
                bestClosestPoint = closestPoint;
                bestClosestPointValid = true;
            }
        };

        {
            const Vector closest = closestPointOnTriangle(origin, triangle);
            consider(lengthSquared(sub(origin, closest)), 0.0f, closest);
        }
        {
            const Vector closest = closestPointOnTriangle(segmentEnd, triangle);
            consider(lengthSquared(sub(segmentEnd, closest)), 1.0f, closest);
        }

        auto considerEdge = [&](const Vector& a, const Vector& b) {
            float segmentRatio = 0.0f;
            float edgeRatio = 0.0f;
            const float distanceSquared = segmentSegmentDistanceSquared(origin, segmentEnd, a, b, segmentRatio, &edgeRatio);
            consider(distanceSquared, segmentRatio, add(a, scale(sub(b, a), edgeRatio)));
        };

        considerEdge(triangle.v0, triangle.v1);
        considerEdge(triangle.v1, triangle.v2);
        considerEdge(triangle.v2, triangle.v0);

        if (bestDistanceSquared <= radiusSquared) {
            outT = bestT;
            if (outClosestPoint && bestClosestPointValid) {
                *outClosestPoint = bestClosestPoint;
            }
            return true;
        }
        return false;
    }

    template <class Vector>
    inline FingerCurlValue solveFingerCurlValue(const std::vector<Triangle<Vector>>& triangles,
        const Vector& origin,
        const Vector& direction,
        float maxDistance,
        float minValue,
        float probeRadius,
        const Vector& curlNormal = Vector{ 0.0f, 0.0f, 1.0f },
        const Vector& zeroAngleVector = Vector{ 1.0f, 0.0f, 0.0f },
        float maxCurlAngleRadians = 3.14159265358979323846f,
        const Vector& surfacePoint = Vector{},
        const Vector& surfaceNormal = Vector{},
        bool rejectBacksideHits = false,
        float surfacePlaneToleranceGameUnits = 0.0f,
        Vector* outHitPoint = nullptr)
    {
        FingerCurlValue result{};
        result.value = std::clamp(minValue, 0.0f, 1.0f);

        if (triangles.empty() || maxDistance <= 0.0001f) {
            return result;
        }

        const Vector dir = normalize(direction);
        float bestT = maxDistance;
        Vector bestHitPoint{};
        Vector bestHitNormal{};
        bool bestHitPointValid = false;
        bool bestHitNormalValid = false;
        bool sawBehindCurlPlane = false;
        bool sawBackSurface = false;
        const Vector normalizedCurlNormal = normalize(curlNormal);
        const Vector normalizedZeroAngle = normalize(zeroAngleVector);
        const bool hasSurfaceGate = rejectBacksideHits && hasUsableDirection(surfaceNormal);
        const Vector normalizedSurfaceNormal = hasSurfaceGate ? normalize(surfaceNormal) : Vector{};
        const float planeTolerance = std::max(0.0f, std::isfinite(surfacePlaneToleranceGameUnits) ? surfacePlaneToleranceGameUnits : 0.0f);
        const float maxCurlAngle = std::isfinite(maxCurlAngleRadians) && maxCurlAngleRadians > 0.0f ? maxCurlAngleRadians : 3.14159265358979323846f;
        for (const auto& triangle : triangles) {
            float t = 0.0f;
            Vector capsuleHitPoint{};
            bool hitPointFromCapsule = false;
            if (!rayTriangleIntersection(origin, dir, triangle, maxDistance, t)) {
                if (!probeCapsuleTriangleIntersection(origin, dir, triangle, maxDistance, probeRadius, t, &capsuleHitPoint)) {
                    continue;
                }
                hitPointFromCapsule = true;
            }

            const Vector hitPoint = hitPointFromCapsule ? capsuleHitPoint : add(origin, scale(dir, t));
            const Vector hitNormal = triangleNormal(triangle);
            if (rejectBacksideHits) {
                const Vector toHit = sub(hitPoint, origin);
                if (hasUsableDirection(toHit)) {
                    const float angle = signedAngleAroundNormal(toHit, normalizedZeroAngle, normalizedCurlNormal);
                    if (angle < -0.0001f && std::abs(angle) <= maxCurlAngle + 0.0001f) {
                        sawBehindCurlPlane = true;
                        continue;
                    }
                }

                if (hasSurfaceGate) {
                    const float planeDistance = dot(normalizedSurfaceNormal, sub(hitPoint, surfacePoint));
                    const float normalDot = dot(hitNormal, normalizedSurfaceNormal);
                    if (planeDistance < -planeTolerance || normalDot < -0.25f) {
                        sawBackSurface = true;
                        continue;
                    }
                }
            }

            if (t < bestT) {
                bestT = t;
                result.hit = true;
                result.hitKind = FingerCurlValue::HitKind::FrontValid;
                bestHitPoint = hitPoint;
                bestHitNormal = hitNormal;
                bestHitPointValid = true;
                bestHitNormalValid = hasUsableDirection(hitNormal);
            }
        }

        if (result.hit) {
            result.distance = bestT;
            result.rawCurveValue = bestT / maxDistance;
            result.value = std::clamp(result.rawCurveValue, std::clamp(minValue, 0.0f, 1.0f), 1.0f);
            if (outHitPoint && bestHitPointValid) {
                *outHitPoint = bestHitPoint;
            }
            if (bestHitPointValid) {
                result.hitPointX = bestHitPoint.x;
                result.hitPointY = bestHitPoint.y;
                result.hitPointZ = bestHitPoint.z;
                result.hasHitPoint = true;
            }
            if (bestHitNormalValid) {
                result.hitNormalX = bestHitNormal.x;
                result.hitNormalY = bestHitNormal.y;
                result.hitNormalZ = bestHitNormal.z;
                result.hasHitNormal = true;
            }
        } else if (sawBehindCurlPlane) {
            result.hit = true;
            result.value = 1.0f;
            result.rawCurveValue = -1.0f;
            result.openedByBehindContact = true;
            result.hitKind = FingerCurlValue::HitKind::BehindCurlPlane;
        } else if (sawBackSurface) {
            result.hitKind = FingerCurlValue::HitKind::BackSurface;
        }

        return result;
    }

    template <class Vector>
    inline FingerCurlValue solveFingerCurveCurlValue(const std::vector<Triangle<Vector>>& triangles, const Vector& center, const Vector& normal,
        const Vector& zeroAngleVector, float maxCurlAngleRadians, float fingerLength, float minValue, const Vector& surfacePoint = Vector{},
        const Vector& surfaceNormal = Vector{}, bool rejectBacksideHits = false, float surfacePlaneToleranceGameUnits = 0.0f)
    {
        /*
         * ROCK intersects the object's local triangle slice with the finger curl
         * disk and converts the hit angle back into an open/closed curve value.
         * This keeps finger solving geometric while using compact parametric
         * curves instead of generated lookup tables in runtime code.
         */
        FingerCurlValue result{};
        result.value = std::clamp(minValue, 0.0f, 1.0f);

        if (triangles.empty() || !std::isfinite(maxCurlAngleRadians) || maxCurlAngleRadians <= 0.0001f ||
            !std::isfinite(fingerLength) || fingerLength <= 0.0001f) {
            return result;
        }

        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        const Vector planeNormal = normalize(normal);
        const Vector zero = normalize(zeroAngleVector);
        const float maxAngle = (std::max)(0.0001f, maxCurlAngleRadians);
        constexpr float kCurveThickness = 0.35f;

        float bestPositiveAngle = (std::numeric_limits<float>::max)();
        Vector bestPositivePoint{};
        Vector bestPositiveNormal{};
        bool bestPositivePointValid = false;
        bool bestPositiveNormalValid = false;
        bool foundBehindContact = false;
        bool sawBackSurface = false;
        const bool hasSurfaceGate = rejectBacksideHits && hasUsableDirection(surfaceNormal);
        const Vector normalizedSurfaceNormal = hasSurfaceGate ? normalize(surfaceNormal) : Vector{};
        const float planeTolerance = std::max(0.0f, std::isfinite(surfacePlaneToleranceGameUnits) ? surfacePlaneToleranceGameUnits : 0.0f);

        auto considerPoint = [&](const Vector& point, const Vector& candidateSurfaceNormal) {
            const Vector fromCenter = sub(point, center);
            const float radius = length(fromCenter);
            if (radius <= 0.0001f || radius > fingerLength + kCurveThickness) {
                return;
            }

            const float angle = signedAngleAroundNormal(fromCenter, zero, planeNormal);
            if (angle < 0.0f && std::abs(angle) <= maxAngle) {
                foundBehindContact = true;
                return;
            }
            if (hasSurfaceGate) {
                const float planeDistance = dot(normalizedSurfaceNormal, sub(point, surfacePoint));
                const float normalDot = dot(candidateSurfaceNormal, normalizedSurfaceNormal);
                if (planeDistance < -planeTolerance || normalDot < -0.25f) {
                    sawBackSurface = true;
                    return;
                }
            }
            if (angle >= 0.0f && angle <= maxAngle && angle < bestPositiveAngle) {
                bestPositiveAngle = angle;
                bestPositivePoint = point;
                bestPositiveNormal = candidateSurfaceNormal;
                bestPositivePointValid = true;
                bestPositiveNormalValid = hasUsableDirection(candidateSurfaceNormal);
            }
        };

        for (const auto& triangle : triangles) {
            std::array<Vector, 3> intersections{};
            std::size_t intersectionCount = 0;
            auto addIntersection = [&](const Vector& a, const Vector& b) {
                if (intersectionCount >= intersections.size()) {
                    return;
                }
                Vector intersection{};
                if (planeIntersectsSegment(center, planeNormal, a, b, intersection)) {
                    intersections[intersectionCount++] = intersection;
                }
            };

            addIntersection(triangle.v0, triangle.v1);
            addIntersection(triangle.v1, triangle.v2);
            addIntersection(triangle.v2, triangle.v0);

            if (intersectionCount == 0) {
                continue;
            }

            const Vector candidateSurfaceNormal = triangleNormal(triangle);
            for (std::size_t i = 0; i < intersectionCount; ++i) {
                considerPoint(intersections[i], candidateSurfaceNormal);
            }
            if (intersectionCount >= 2) {
                considerPoint(scale(add(intersections[0], intersections[1]), 0.5f), candidateSurfaceNormal);
            }
        }

        if (bestPositiveAngle != (std::numeric_limits<float>::max)()) {
            result.hit = true;
            result.distance = bestPositiveAngle;
            result.rawCurveValue = 1.0f - (bestPositiveAngle / maxAngle);
            result.value = std::clamp(result.rawCurveValue, clampedMin, 1.0f);
            result.hitKind = FingerCurlValue::HitKind::FrontValid;
            if (bestPositivePointValid) {
                result.hitPointX = bestPositivePoint.x;
                result.hitPointY = bestPositivePoint.y;
                result.hitPointZ = bestPositivePoint.z;
                result.hasHitPoint = true;
            }
            if (bestPositiveNormalValid) {
                result.hitNormalX = bestPositiveNormal.x;
                result.hitNormalY = bestPositiveNormal.y;
                result.hitNormalZ = bestPositiveNormal.z;
                result.hasHitNormal = true;
            }
            return result;
        }

        if (foundBehindContact) {
            result.hit = true;
            result.value = 1.0f;
            result.rawCurveValue = -1.0f;
            result.openedByBehindContact = true;
            result.hitKind = FingerCurlValue::HitKind::BehindCurlPlane;
            return result;
        }

        if (sawBackSurface) {
            result.hitKind = FingerCurlValue::HitKind::BackSurface;
        }
        return result;
    }

    template <class Vector>
    inline ThumbAwareFingerCurveCurlValue<Vector> solveThumbAwareFingerCurveCurlValue(const std::vector<Triangle<Vector>>& triangles,
        const Vector& center,
        const Vector& primaryNormal,
        const Vector& alternateThumbNormal,
        const Vector& zeroAngleVector,
        float maxCurlAngleRadians,
        float fingerLength,
        float minValue,
        bool allowAlternateThumbCurve,
        const Vector& surfacePoint = Vector{},
        const Vector& surfaceNormal = Vector{},
        bool rejectBacksideHits = false,
        float surfacePlaneToleranceGameUnits = 0.0f)
    {
        /*
         * ROCK evaluates a second thumb curve because the thumb can close across
         * the palm instead of curling in the same plane as the other fingers. The
         * public five-finger pose contract stays intact, but thumb contacts on
         * weapon supports and small held objects can still influence the value
         * sent to FRIK when the normal calibrated plane misses.
         */
        ThumbAwareFingerCurveCurlValue<Vector> result{};
        result.primary = solveFingerCurveCurlValue(triangles,
            center,
            primaryNormal,
            zeroAngleVector,
            maxCurlAngleRadians,
            fingerLength,
            minValue,
            surfacePoint,
            surfaceNormal,
            rejectBacksideHits,
            surfacePlaneToleranceGameUnits);
        result.value = result.primary;

        if (!allowAlternateThumbCurve) {
            return result;
        }

        result.alternateThumb = solveFingerCurveCurlValue(triangles,
            center,
            alternateThumbNormal,
            zeroAngleVector,
            maxCurlAngleRadians,
            fingerLength,
            minValue,
            surfacePoint,
            surfaceNormal,
            rejectBacksideHits,
            surfacePlaneToleranceGameUnits);
        constexpr float kClosedEpsilon = 0.0001f;
        const bool primaryClosedOrMissed = !result.primary.hit || result.primary.rawCurveValue <= kClosedEpsilon;
        const bool primaryNeedsAlternate = primaryClosedOrMissed || result.primary.openedByBehindContact;
        const bool alternatePositive =
            result.alternateThumb.hit && !result.alternateThumb.openedByBehindContact && result.alternateThumb.rawCurveValue > kClosedEpsilon;
        const bool alternateClosedOrMissed =
            !result.alternateThumb.hit || (!result.alternateThumb.openedByBehindContact && result.alternateThumb.rawCurveValue <= kClosedEpsilon);
        const bool bothCurvesClosedOrMissed = primaryClosedOrMissed && !result.primary.openedByBehindContact && alternateClosedOrMissed;

        if (primaryNeedsAlternate && (alternatePositive || bothCurvesClosedOrMissed)) {
            result.value = result.alternateThumb;
            result.usedAlternateThumbCurve = true;
        }

        return result;
    }

    inline std::array<float, 15> expandFingerCurlsToJointValues(const std::array<float, 5>& values)
    {
        std::array<float, 15> joints{};
        for (std::size_t finger = 0; finger < values.size(); ++finger) {
            const float value = std::clamp(values[finger], 0.0f, 1.0f);
            const float closed = 1.0f - value;
            const float proximalOpenBias = (finger == 0) ? 0.15f : 0.25f;
            const float distalCloseBias = (finger == 0) ? 0.10f : 0.15f;
            const std::size_t base = finger * 3;
            joints[base + 0] = std::clamp(value + closed * proximalOpenBias, 0.0f, 1.0f);
            joints[base + 1] = value;
            joints[base + 2] = std::clamp(value - closed * distalCloseBias, 0.0f, 1.0f);
        }
        return joints;
    }

    inline std::array<float, 15> advanceJointValues(const std::array<float, 15>& current, const std::array<float, 15>& target, float speed, float deltaTime)
    {
        std::array<float, 15> result{};
        if (!std::isfinite(speed) || speed <= 0.0f) {
            return target;
        }

        const float dt = (std::isfinite(deltaTime) && deltaTime > 0.0f) ? deltaTime : (1.0f / 90.0f);
        const float step = speed * dt;
        for (std::size_t i = 0; i < result.size(); ++i) {
            const float from = std::clamp(current[i], 0.0f, 1.0f);
            const float to = std::clamp(target[i], 0.0f, 1.0f);
            const float delta = to - from;
            if (std::abs(delta) <= step) {
                result[i] = to;
            } else {
                result[i] = from + (delta > 0.0f ? step : -step);
            }
        }
        return result;
    }
}

// ---- GrabFingerPoseRuntime.h ----

#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/hand/HandFrame.h"

#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace rock::grab_finger_pose_runtime
{
    /*
     * ROCK solves grasped fingers from the held object's triangle geometry and
     * lets FRIK own the rendered skeleton. Probe bases, open vectors, and palm
     * curl normals come directly from the live root flattened finger bones.
     */
    struct SolvedGrabFingerPose
    {
        std::array<float, 5> values{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        std::array<float, 15> jointValues{};
        std::array<RE::NiPoint3, 5> probeStart{};
        std::array<RE::NiPoint3, 5> probeEnd{};
        std::array<RE::NiPoint3, 5> surfaceAimTarget{};
        std::array<RE::NiPoint3, 5> surfaceAimNormal{};
        std::array<std::uint8_t, 5> surfaceAimTargetValid{};
        std::array<std::uint8_t, 5> surfaceAimNormalValid{};
        std::array<RE::NiPoint3, 5> surfaceAimTargetObjectLocal{};
        std::array<RE::NiPoint3, 5> surfaceAimNormalObjectLocal{};
        std::array<std::uint8_t, 5> surfaceAimTargetObjectLocalValid{};
        std::array<std::uint8_t, 5> surfaceAimNormalObjectLocalValid{};
        std::array<grab_finger_pose_math::FingerCurlValue::HitKind, 5> hitKind{};
        int hitCount = 0;
        int candidateTriangleCount = 0;
        int poseTargetCount = 0;
        bool solved = false;
        bool hasJointValues = false;
        bool hasObjectLocalSurfaceAim = false;
        bool usedAlternateThumbCurve = false;
        bool usedAlternateThumbSurfaceHit = false;
        bool thumbSurfaceFollowAllowed = true;
        bool usedLiveRootFlattenedFingerBones = false;
        bool hasThumbAlternateCurveFrame = false;
        RE::NiPoint3 thumbAlternateCurveBaseWorld{};
        RE::NiPoint3 thumbAlternateCurveOpenDirectionWorld{};
        RE::NiPoint3 thumbAlternateCurveNormalWorld{};
        float thumbAlternateCurveMaxCurlAngleRadians = 0.0f;
        bool hasThumbCurveDiagnostics = false;
        grab_finger_pose_math::FingerCurlValue thumbPrimaryCurve{};
        grab_finger_pose_math::FingerCurlValue thumbAlternateCurve{};
    };

    inline void captureSurfaceAimObjectLocal(SolvedGrabFingerPose& pose, const RE::NiTransform& objectWorldTransform)
    {
        pose.surfaceAimTargetObjectLocal = {};
        pose.surfaceAimNormalObjectLocal = {};
        pose.surfaceAimTargetObjectLocalValid = {};
        pose.surfaceAimNormalObjectLocalValid = {};
        pose.hasObjectLocalSurfaceAim = false;

        if (!std::isfinite(objectWorldTransform.scale) || std::abs(objectWorldTransform.scale) <= 0.000001f) {
            return;
        }

        for (std::size_t finger = 0; finger < pose.surfaceAimTarget.size(); ++finger) {
            if (pose.surfaceAimTargetValid[finger]) {
                pose.surfaceAimTargetObjectLocal[finger] =
                    transform_math::worldPointToLocal(objectWorldTransform, pose.surfaceAimTarget[finger]);
                pose.surfaceAimTargetObjectLocalValid[finger] = 1;
                pose.hasObjectLocalSurfaceAim = true;
            }
            if (pose.surfaceAimNormalValid[finger]) {
                pose.surfaceAimNormalObjectLocal[finger] =
                    transform_math::worldVectorToLocal(objectWorldTransform, pose.surfaceAimNormal[finger]);
                pose.surfaceAimNormalObjectLocalValid[finger] = 1;
                pose.hasObjectLocalSurfaceAim = true;
            }
        }
    }

    inline void useThumbCurveOnlyPose(SolvedGrabFingerPose& pose)
    {
        pose.thumbSurfaceFollowAllowed = false;
        pose.usedAlternateThumbSurfaceHit = false;
        pose.surfaceAimTargetValid[0] = 0;
        pose.surfaceAimNormalValid[0] = 0;
        pose.surfaceAimTargetObjectLocalValid[0] = 0;
        pose.surfaceAimNormalObjectLocalValid[0] = 0;
        pose.surfaceAimTarget[0] = {};
        pose.surfaceAimNormal[0] = {};
        pose.surfaceAimTargetObjectLocal[0] = {};
        pose.surfaceAimNormalObjectLocal[0] = {};
        pose.hasObjectLocalSurfaceAim = false;
        for (std::size_t finger = 1; finger < pose.surfaceAimTargetObjectLocalValid.size(); ++finger) {
            if (pose.surfaceAimTargetObjectLocalValid[finger] || pose.surfaceAimNormalObjectLocalValid[finger]) {
                pose.hasObjectLocalSurfaceAim = true;
                break;
            }
        }
    }

    [[nodiscard]] inline SolvedGrabFingerPose resolveSurfaceAimObjectLocal(
        const SolvedGrabFingerPose& pose,
        const RE::NiTransform& objectWorldTransform)
    {
        if (!pose.hasObjectLocalSurfaceAim ||
            !std::isfinite(objectWorldTransform.scale) ||
            std::abs(objectWorldTransform.scale) <= 0.000001f) {
            return pose;
        }

        auto resolved = pose;
        for (std::size_t finger = 0; finger < resolved.surfaceAimTarget.size(); ++finger) {
            if (pose.surfaceAimTargetObjectLocalValid[finger]) {
                resolved.surfaceAimTarget[finger] =
                    transform_math::localPointToWorld(objectWorldTransform, pose.surfaceAimTargetObjectLocal[finger]);
                resolved.surfaceAimTargetValid[finger] = 1;
            }
            if (pose.surfaceAimNormalObjectLocalValid[finger]) {
                const RE::NiPoint3 normalWorld =
                    transform_math::localVectorToWorld(objectWorldTransform, pose.surfaceAimNormalObjectLocal[finger]);
                if ((normalWorld.x * normalWorld.x + normalWorld.y * normalWorld.y + normalWorld.z * normalWorld.z) > 0.000001f) {
                    resolved.surfaceAimNormal[finger] = normalizeDirection(normalWorld);
                    resolved.surfaceAimNormalValid[finger] = 1;
                }
            }
        }
        return resolved;
    }

    [[nodiscard]] inline float missedFingerCurlFallbackValue(
        bool hasExplicitFingerTarget,
        grab_finger_pose_math::FingerCurlValue::HitKind hitKind,
        float minValue)
    {
        if (!hasExplicitFingerTarget ||
            hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface ||
            hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::Rejected) {
            return 1.0f;
        }
        return std::clamp(minValue, 0.0f, 1.0f);
    }

    inline const std::array<float, 5>& fingerMaxAnglesRadians()
    {
        static const std::array<float, 5> kAngles{ 1.225f, 1.45f, 1.50f, 1.48f, 1.42f };
        return kAngles;
    }

    inline RE::NiPoint3 crossPoint(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return RE::NiPoint3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
    }

    inline float dotPoint(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lengthSquared <= 0.000001f) {
            return normalizeDirection(fallback);
        }

        const float inv = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inv, value.y * inv, value.z * inv);
    }

    inline RE::NiPoint3 liveThumbAlternateCurlNormalWorld(const RE::NiPoint3& openVectorWorld, const RE::NiPoint3& primaryNormalWorld,
        const RE::NiPoint3& baseWorld, const RE::NiPoint3& grabAnchorWorld, bool isLeft)
    {
        RE::NiPoint3 alternateNormal = normalizedOrFallback(crossPoint(openVectorWorld, primaryNormalWorld), RE::NiPoint3(0.0f, isLeft ? -1.0f : 1.0f, 0.0f));
        const RE::NiPoint3 towardPalm = normalizedOrFallback(grabAnchorWorld - baseWorld, RE::NiPoint3(0.0f, isLeft ? 1.0f : -1.0f, 0.0f));
        if (dotPoint(alternateNormal, crossPoint(openVectorWorld, towardPalm)) < 0.0f) {
            alternateNormal.x *= -1.0f;
            alternateNormal.y *= -1.0f;
            alternateNormal.z *= -1.0f;
        }
        return alternateNormal;
    }

    inline float distanceSquared(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        return dx * dx + dy * dy + dz * dz;
    }

    struct GrabFingerPoseTargetSet
    {
        std::array<RE::NiPoint3, 5> targets{};
        std::array<RE::NiPoint3, 5> targetNormals{};
        std::array<std::uint8_t, 5> targetValid{};
        std::array<std::uint8_t, 5> targetNormalValid{};
        RE::NiPoint3 seatPointWorld{};
        RE::NiPoint3 seatNormalWorld{};
        bool seatPointValid = false;
        bool seatNormalValid = false;
        bool useSeatPointForMissingTargets = true;
        bool useWholeMeshForMissingTargets = false;
        std::uint32_t targetCount = 0;
    };

    inline GrabFingerPoseTargetSet makeSharedGripPoseTarget(const RE::NiPoint3& grabGripPoint, const RE::NiPoint3& grabGripNormal = RE::NiPoint3{})
    {
        GrabFingerPoseTargetSet targets{};
        targets.seatPointWorld = grabGripPoint;
        targets.seatPointValid = true;
        if (distanceSquared(grabGripNormal, RE::NiPoint3{}) > 0.000001f) {
            targets.seatNormalWorld = normalizeDirection(grabGripNormal);
            targets.seatNormalValid = true;
        }
        targets.useSeatPointForMissingTargets = true;
        return targets;
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles, const RE::NiPoint3& grabGripPoint, float maxTriangleDistanceSquared,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        outTriangles.clear();
        if (!std::isfinite(maxTriangleDistanceSquared) || maxTriangleDistanceSquared <= 0.0f) {
            return;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> allTriangles;
        allTriangles.reserve((std::min)(triangles.size(), static_cast<std::size_t>(512)));
        for (const auto& triangle : triangles) {
            allTriangles.push_back({ triangle.v0, triangle.v1, triangle.v2 });
        }
        outTriangles = grab_finger_pose_math::filterTrianglesNearPoint(allTriangles, grabGripPoint, maxTriangleDistanceSquared);
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles, const GrabFingerPoseTargetSet& poseTargets, float maxTriangleDistanceSquared,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        outTriangles.clear();
        if (!std::isfinite(maxTriangleDistanceSquared) || maxTriangleDistanceSquared <= 0.0f) {
            return;
        }

        if (poseTargets.useWholeMeshForMissingTargets && poseTargets.targetCount == 0) {
            outTriangles.reserve(triangles.size());
            for (const auto& triangle : triangles) {
                outTriangles.push_back({ triangle.v0, triangle.v1, triangle.v2 });
            }
            return;
        }

        std::array<RE::NiPoint3, 6> points{};
        std::size_t pointCount = 0;
        if (poseTargets.seatPointValid) {
            points[pointCount++] = poseTargets.seatPointWorld;
        }
        for (std::size_t finger = 0; finger < poseTargets.targets.size() && pointCount < points.size(); ++finger) {
            if (poseTargets.targetValid[finger]) {
                points[pointCount++] = poseTargets.targets[finger];
            }
        }
        if (pointCount == 0) {
            return;
        }

        outTriangles.reserve((std::min)(triangles.size(), static_cast<std::size_t>(256)));
        for (const auto& triangle : triangles) {
            const auto candidate = grab_finger_pose_math::Triangle<RE::NiPoint3>{ triangle.v0, triangle.v1, triangle.v2 };
            const RE::NiPoint3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
            bool nearAnyTarget = false;
            for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
                const auto& point = points[pointIndex];
                const float bestVertexOrCentroidDistance = (std::min)({
                    distanceSquared(centroid, point),
                    distanceSquared(triangle.v0, point),
                    distanceSquared(triangle.v1, point),
                    distanceSquared(triangle.v2, point),
                });
                if (bestVertexOrCentroidDistance <= maxTriangleDistanceSquared) {
                    nearAnyTarget = true;
                    break;
                }
            }
            if (nearAnyTarget) {
                outTriangles.push_back(candidate);
            }
        }
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles, const RE::NiTransform& handTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const GrabFingerPoseTargetSet& poseTargets, float minValue, float maxTriangleDistanceSquared = 100.0f, bool useCurveSolver = true,
        const root_flattened_finger_skeleton_runtime::Snapshot* liveFingerSnapshot = nullptr, bool rejectBacksideHits = true, float surfacePlaneToleranceGameUnits = 1.5f,
        bool allowSurfaceAimTargets = true)
    {
        SolvedGrabFingerPose result{};
        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        result.values = { clampedMin, clampedMin, clampedMin, clampedMin, clampedMin };
        result.poseTargetCount = static_cast<int>(poseTargets.targetCount);

        if (triangles.empty() || !poseTargets.seatPointValid) {
            return result;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> candidateTriangles;
        appendCandidateTriangles(triangles, poseTargets, maxTriangleDistanceSquared, candidateTriangles);
        result.candidateTriangleCount = static_cast<int>(candidateTriangles.size());
        if (candidateTriangles.empty()) {
            return result;
        }

        constexpr float kFingerReachPadding = 6.0f;
        constexpr float kMinFingerProbeDistance = 6.0f;
        constexpr float kMaxFingerProbeDistance = 26.0f;
        constexpr float kFingerProbeRadius = 1.25f;
        const RE::NiPoint3 fallbackDirection = transformHandspaceDirection(handTransform, RE::NiPoint3(1.0f, 0.0f, 0.0f), isLeft);
        const auto& maxAngles = fingerMaxAnglesRadians();
        const auto liveLandmarks = liveFingerSnapshot ? root_flattened_finger_skeleton_runtime::buildLandmarkSet(*liveFingerSnapshot) : root_flattened_finger_skeleton_runtime::LandmarkSet{};
        if (!liveLandmarks.valid) {
            return result;
        }
        const RE::NiPoint3 curlNormalWorld = liveLandmarks.palmNormalWorld;
        const bool usePalmSeatProbeShift =
            poseTargets.useWholeMeshForMissingTargets && poseTargets.seatPointValid && poseTargets.targetCount == 0;
        const RE::NiPoint3 palmSeatProbeShift = usePalmSeatProbeShift ? poseTargets.seatPointWorld - grabAnchorWorld : RE::NiPoint3{};
        result.usedLiveRootFlattenedFingerBones = true;

        for (std::size_t finger = 0; finger < result.values.size(); ++finger) {
            const auto& live = liveLandmarks.fingers[finger];
            /*
             * ROCK solves grabbed fingers as if the palm-selected object point
             * has already been seated into the hand: startFingerPos +=
             * palmToPoint. Live root-flattened finger bases stay authoritative,
             * and the palm-seat probe shift is applied only for broad mesh curl.
             * Pinch/opposition still use their explicit per-finger targets.
             */
            const RE::NiPoint3 baseWorld = live.base + palmSeatProbeShift;
            const bool hasFingerTarget = poseTargets.targetValid[finger] != 0;
            const bool useTarget = hasFingerTarget || poseTargets.useSeatPointForMissingTargets;
            const RE::NiPoint3 fingerTargetWorld = hasFingerTarget ? poseTargets.targets[finger] : poseTargets.seatPointWorld;
            const bool hasFingerTargetNormal = hasFingerTarget && poseTargets.targetNormalValid[finger] != 0;
            const bool useTargetNormal = hasFingerTargetNormal || poseTargets.seatNormalValid;
            const RE::NiPoint3 fingerTargetNormalWorld = hasFingerTargetNormal ? poseTargets.targetNormals[finger] : poseTargets.seatNormalWorld;
            const RE::NiPoint3 openDirectionWorld = live.openDirection;
            const float fingerOpenLengthWorld = live.length;
            const RE::NiPoint3 thumbAlternateCurlNormalWorld = liveThumbAlternateCurlNormalWorld(openDirectionWorld, curlNormalWorld, baseWorld, grabAnchorWorld, isLeft);
            const RE::NiPoint3 toContact = useTarget ? fingerTargetWorld - baseWorld : openDirectionWorld;
            const float distanceToContact = useTarget ? std::sqrt(distanceSquared(fingerTargetWorld, baseWorld)) : fingerOpenLengthWorld;
            const float probeDistance = std::clamp(distanceToContact + kFingerReachPadding, kMinFingerProbeDistance, kMaxFingerProbeDistance);
            const RE::NiPoint3 probeDirection = normalizedOrFallback(toContact, fallbackDirection);
            result.probeStart[finger] = baseWorld;
            result.probeEnd[finger] = baseWorld + probeDirection * probeDistance;
            if (finger == 0) {
                result.hasThumbAlternateCurveFrame = true;
                result.thumbAlternateCurveBaseWorld = baseWorld;
                result.thumbAlternateCurveOpenDirectionWorld = normalizedOrFallback(openDirectionWorld, fallbackDirection);
                result.thumbAlternateCurveNormalWorld = normalizedOrFallback(thumbAlternateCurlNormalWorld, curlNormalWorld);
                result.thumbAlternateCurveMaxCurlAngleRadians = maxAngles[finger];
            }

            auto solved = grab_finger_pose_math::FingerCurlValue{};
            solved.value = clampedMin;
            RE::NiPoint3 fallbackHitPoint{};
            bool fallbackHitPointValid = false;
            bool curveSolverRan = false;
            bool curveSolverSelectedAlternateThumb = false;
            if (useCurveSolver) {
                const bool isThumb = finger == 0;
                const auto curveSolved = grab_finger_pose_math::solveThumbAwareFingerCurveCurlValue(candidateTriangles,
                    baseWorld,
                    curlNormalWorld,
                    thumbAlternateCurlNormalWorld,
                    openDirectionWorld,
                    maxAngles[finger],
                    fingerOpenLengthWorld,
                    clampedMin,
                    isThumb,
                    fingerTargetWorld,
                    fingerTargetNormalWorld,
                    rejectBacksideHits && useTargetNormal,
                    surfacePlaneToleranceGameUnits);
                curveSolverRan = true;
                curveSolverSelectedAlternateThumb = isThumb && curveSolved.usedAlternateThumbCurve;
                solved = curveSolved.value;
                result.usedAlternateThumbCurve = result.usedAlternateThumbCurve || curveSolved.usedAlternateThumbCurve;
                if (isThumb) {
                    result.hasThumbCurveDiagnostics = true;
                    result.thumbPrimaryCurve = curveSolved.primary;
                    result.thumbAlternateCurve = curveSolved.alternateThumb;
                    const bool alternateThumbSurfaceHit =
                        curveSolved.usedAlternateThumbCurve &&
                        curveSolved.alternateThumb.hit &&
                        curveSolved.alternateThumb.hasHitPoint &&
                        !curveSolved.alternateThumb.openedByBehindContact &&
                        curveSolved.alternateThumb.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid;
                    result.usedAlternateThumbSurfaceHit = alternateThumbSurfaceHit;
                    if (alternateThumbSurfaceHit) {
                        result.surfaceAimTarget[0] = RE::NiPoint3{
                            curveSolved.alternateThumb.hitPointX,
                            curveSolved.alternateThumb.hitPointY,
                            curveSolved.alternateThumb.hitPointZ
                        };
                        result.surfaceAimTargetValid[0] = 1;
                        if (curveSolved.alternateThumb.hasHitNormal) {
                            result.surfaceAimNormal[0] = normalizedOrFallback(
                                RE::NiPoint3{
                                    curveSolved.alternateThumb.hitNormalX,
                                    curveSolved.alternateThumb.hitNormalY,
                                    curveSolved.alternateThumb.hitNormalZ },
                                curlNormalWorld);
                            result.surfaceAimNormalValid[0] = 1;
                        }
                    }
                }
            }
            if (useTarget && !solved.hit &&
                (!curveSolverRan || grab_finger_pose_math::shouldRunFallbackRayAfterCurveSolve(finger, solved.hit, curveSolverSelectedAlternateThumb))) {
                solved = grab_finger_pose_math::solveFingerCurlValue(candidateTriangles,
                    baseWorld,
                    probeDirection,
                    probeDistance,
                    clampedMin,
                    kFingerProbeRadius,
                    curlNormalWorld,
                    openDirectionWorld,
                    maxAngles[finger],
                    fingerTargetWorld,
                    fingerTargetNormalWorld,
                    rejectBacksideHits && useTargetNormal,
                    surfacePlaneToleranceGameUnits,
                    &fallbackHitPoint);
                fallbackHitPointValid = solved.hit && solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid;
            }
            /*
             * Generic grab poses often use whole-mesh probing instead of
             * explicit per-finger targets. A miss in that mode means "no
             * surface on this finger path", not "close through the object".
             */
            if (!solved.hit &&
                (!useTarget ||
                    solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface ||
                    solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::Rejected)) {
                solved.value = missedFingerCurlFallbackValue(useTarget, solved.hitKind, clampedMin);
                solved.rawCurveValue = solved.value;
            }
            result.values[finger] = solved.value;
            result.hitKind[finger] = solved.hitKind;
            if (allowSurfaceAimTargets && useTarget && solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid) {
                if (result.surfaceAimTargetValid[finger] == 0) {
                    if (solved.hasHitPoint) {
                        result.surfaceAimTarget[finger] = RE::NiPoint3{ solved.hitPointX, solved.hitPointY, solved.hitPointZ };
                    } else {
                        result.surfaceAimTarget[finger] = fallbackHitPointValid ? fallbackHitPoint : fingerTargetWorld;
                    }
                    result.surfaceAimTargetValid[finger] = 1;
                }
                if (result.surfaceAimNormalValid[finger] == 0 && solved.hasHitNormal) {
                    result.surfaceAimNormal[finger] = normalizedOrFallback(
                        RE::NiPoint3{ solved.hitNormalX, solved.hitNormalY, solved.hitNormalZ },
                        curlNormalWorld);
                    result.surfaceAimNormalValid[finger] = 1;
                } else if (result.surfaceAimNormalValid[finger] == 0 && useTargetNormal && distanceSquared(fingerTargetNormalWorld, RE::NiPoint3{}) > 0.000001f) {
                    result.surfaceAimNormal[finger] = normalizedOrFallback(fingerTargetNormalWorld, curlNormalWorld);
                    result.surfaceAimNormalValid[finger] = 1;
                }
            }
            if (solved.hit) {
                result.hitCount++;
            }
        }

        result.solved = result.candidateTriangleCount > 0 && result.usedLiveRootFlattenedFingerBones;
        result.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(result.values);
        result.hasJointValues = result.solved;
        return result;
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles, const RE::NiTransform& handTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const RE::NiPoint3& grabGripPoint, float minValue, float maxTriangleDistanceSquared = 100.0f, bool useCurveSolver = true,
        const root_flattened_finger_skeleton_runtime::Snapshot* liveFingerSnapshot = nullptr, bool rejectBacksideHits = true, float surfacePlaneToleranceGameUnits = 1.5f,
        bool allowSurfaceAimTargets = true)
    {
        return solveGrabFingerPoseFromTriangles(triangles,
            handTransform,
            isLeft,
            grabAnchorWorld,
            makeSharedGripPoseTarget(grabGripPoint),
            minValue,
            maxTriangleDistanceSquared,
            useCurveSolver,
            liveFingerSnapshot,
            rejectBacksideHits,
            surfacePlaneToleranceGameUnits,
            allowSurfaceAimTargets);
    }
}

// ---- GrabFingerLocalTransformMath.h ----

/*
 * ROCK gets convincing wrapped fingers by driving the visible skeleton, not by
 * changing the held-object constraint. The mesh solver still produces scalar
 * joint curls, FRIK supplies the authored local-transform baseline for those
 * curls, and this policy decides when ROCK may add bounded surface-aim
 * corrections to that baseline.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace rock::grab_finger_local_transform_math
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

    [[nodiscard]] inline bool shouldApplySurfaceAimCorrection(
        std::size_t fingerIndex,
        bool alternateThumbPlaneCorrection,
        bool thumbSurfaceFollowAllowed = true)
    {
        return fingerIndex != 0 || (thumbSurfaceFollowAllowed && !alternateThumbPlaneCorrection);
    }

    [[nodiscard]] inline bool shouldApplyAlternateThumbLocalCorrection(bool usedAlternateThumbCurve, bool usedAlternateThumbSurfaceHit)
    {
        return usedAlternateThumbCurve && usedAlternateThumbSurfaceHit;
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

    template <class Vector>
    [[nodiscard]] inline bool thumbAlternateSurfaceGuardAllows(const Vector& thumbTipWorld,
        const Vector& targetAxisWorld,
        const Vector& surfacePointWorld,
        const Vector& surfaceNormalWorld,
        float marginGameUnits,
        bool enabled,
        bool hasSurface)
    {
        if (!enabled) {
            return true;
        }
        if (!hasSurface || vectorDot(surfaceNormalWorld, surfaceNormalWorld) <= 0.000001f || vectorDot(targetAxisWorld, targetAxisWorld) <= 0.000001f) {
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
        const float boundedTravel = std::min(std::max(0.0f, travelToSurface), margin + std::max(0.0f, signedTipDistance));
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

// ---- GrabFingerLocalTransformRuntime.h ----

/*
 * ROCK publishes full-hand local transforms only after FRIK has generated the
 * authored local pose for the same 15 joint values. That keeps FRIK as the hand
 * table owner, while ROCK contributes a bounded mesh-contact aim correction
 * derived from the root flattened finger bones and the current object surface
 * probes. The live transform source intentionally matches generated hand
 * colliders; FRIK remains only the pose publication API here.
 */

#include "physics-interaction/hand/HandSkeleton.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/TransformMath.h"
#include "api/FRIKApi.h"

#include "RE/NetImmerse/NiTransform.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string_view>

namespace rock::grab_finger_local_transform_runtime
{
    struct Options
    {
        bool enabled = true;
        float smoothingSpeed = grab_finger_local_transform_math::kDefaultLocalTransformSmoothingSpeed;
        float maxCorrectionDegrees = grab_finger_local_transform_math::kDefaultMaxCorrectionDegrees;
        float surfaceAimStrength = grab_finger_local_transform_math::kDefaultSurfaceAimStrength;
        float thumbOppositionStrength = grab_finger_local_transform_math::kDefaultThumbOppositionStrength;
        float thumbAlternateCurveStrength = grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength;
        bool thumbSurfaceSafetyEnabled = true;
        float thumbSurfaceSafetyMarginGameUnits = 1.0f;
    };

    struct State
    {
        std::array<RE::NiTransform, 15> currentTransforms{};
        std::uint16_t currentMask = 0;
        bool hasCurrentTransforms = false;
    };

    struct LiveFingerTransform
    {
        RE::NiTransform world{};
        RE::NiTransform parentWorld{};
        bool valid = false;
    };

    [[nodiscard]] inline Options sanitizeOptions(Options options)
    {
        options.smoothingSpeed = grab_finger_local_transform_math::sanitizeSmoothingSpeed(
            options.smoothingSpeed, grab_finger_local_transform_math::kDefaultLocalTransformSmoothingSpeed);
        options.maxCorrectionDegrees = grab_finger_local_transform_math::sanitizeMaxCorrectionDegrees(
            options.maxCorrectionDegrees, grab_finger_local_transform_math::kDefaultMaxCorrectionDegrees);
        options.surfaceAimStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            options.surfaceAimStrength, grab_finger_local_transform_math::kDefaultSurfaceAimStrength);
        options.thumbOppositionStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            options.thumbOppositionStrength, grab_finger_local_transform_math::kDefaultThumbOppositionStrength);
        options.thumbAlternateCurveStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            options.thumbAlternateCurveStrength, grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength);
        options.thumbSurfaceSafetyMarginGameUnits = std::max(
            0.0f,
            std::isfinite(options.thumbSurfaceSafetyMarginGameUnits) ? options.thumbSurfaceSafetyMarginGameUnits : 1.0f);
        return options;
    }

    [[nodiscard]] inline bool isFiniteRotation(const RE::NiMatrix3& rotation)
    {
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(rotation.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    [[nodiscard]] inline bool isFiniteTransform(const RE::NiTransform& transform)
    {
        return grab_finger_local_transform_math::sceneTransformHasUsableBasis(transform);
    }

    [[nodiscard]] inline float dot(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    [[nodiscard]] inline RE::NiPoint3 cross(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return RE::NiPoint3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x,
        };
    }

    [[nodiscard]] inline float lengthSquared(const RE::NiPoint3& value)
    {
        return dot(value, value);
    }

    [[nodiscard]] inline RE::NiPoint3 normalizeOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float valueLengthSquared = lengthSquared(value);
        if (std::isfinite(valueLengthSquared) && valueLengthSquared > 0.000001f) {
            const float invLength = 1.0f / std::sqrt(valueLengthSquared);
            return RE::NiPoint3{ value.x * invLength, value.y * invLength, value.z * invLength };
        }

        const float fallbackLengthSquared = lengthSquared(fallback);
        if (std::isfinite(fallbackLengthSquared) && fallbackLengthSquared > 0.000001f) {
            const float invLength = 1.0f / std::sqrt(fallbackLengthSquared);
            return RE::NiPoint3{ fallback.x * invLength, fallback.y * invLength, fallback.z * invLength };
        }

        return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
    }

    [[nodiscard]] inline RE::NiPoint3 orthogonalAxis(const RE::NiPoint3& value)
    {
        const RE::NiPoint3 axis = std::abs(value.x) < 0.7f ? RE::NiPoint3{ 1.0f, 0.0f, 0.0f } : RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
        return normalizeOrFallback(cross(value, axis), RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
    }

    [[nodiscard]] inline RE::NiMatrix3 storedRotationFromConventionalRows(const RE::NiPoint3 rows[3])
    {
        RE::NiMatrix3 result{};
        result.entry[0][0] = rows[0].x;
        result.entry[0][1] = rows[1].x;
        result.entry[0][2] = rows[2].x;
        result.entry[1][0] = rows[0].y;
        result.entry[1][1] = rows[1].y;
        result.entry[1][2] = rows[2].y;
        result.entry[2][0] = rows[0].z;
        result.entry[2][1] = rows[1].z;
        result.entry[2][2] = rows[2].z;
        return result;
    }

    [[nodiscard]] inline RE::NiMatrix3 axisAngleStored(const RE::NiPoint3& axisRaw, float angle)
    {
        const RE::NiPoint3 axis = normalizeOrFallback(axisRaw, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        const float x = axis.x;
        const float y = axis.y;
        const float z = axis.z;
        const float cosTheta = std::cos(angle);
        const float sinTheta = std::sin(angle);
        const float oneMinusCos = 1.0f - cosTheta;

        const RE::NiPoint3 conventionalRows[3]{
            RE::NiPoint3{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
            RE::NiPoint3{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
            RE::NiPoint3{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
        };
        return storedRotationFromConventionalRows(conventionalRows);
    }

    [[nodiscard]] inline RE::NiMatrix3 applyWorldRotationToStoredBasis(const RE::NiMatrix3& worldRotationStored, const RE::NiMatrix3& baseRotation)
    {
        RE::NiMatrix3 result{};
        const RE::NiPoint3 basis[3]{
            RE::NiPoint3{ baseRotation.entry[0][0], baseRotation.entry[0][1], baseRotation.entry[0][2] },
            RE::NiPoint3{ baseRotation.entry[1][0], baseRotation.entry[1][1], baseRotation.entry[1][2] },
            RE::NiPoint3{ baseRotation.entry[2][0], baseRotation.entry[2][1], baseRotation.entry[2][2] },
        };

        for (int axis = 0; axis < 3; ++axis) {
            const RE::NiPoint3 rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis[axis]);
            result.entry[axis][0] = rotated.x;
            result.entry[axis][1] = rotated.y;
            result.entry[axis][2] = rotated.z;
        }
        return result;
    }

    [[nodiscard]] inline bool applyAlternateThumbPlaneCorrection(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const std::array<LiveFingerTransform, 15>& liveNodes,
        float maxCorrectionRadians,
        float strength,
        bool surfaceSafetyEnabled,
        float surfaceSafetyMarginGameUnits,
        frik::api::FRIKApi::FingerLocalTransformOverride& transforms)
    {
        const float curveStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            strength, grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength);
        if (!grab_finger_local_transform_math::shouldApplyAlternateThumbLocalCorrection(
                fingerPose.usedAlternateThumbCurve,
                fingerPose.usedAlternateThumbSurfaceHit) ||
            !fingerPose.hasThumbAlternateCurveFrame ||
            curveStrength <= 0.0f || !std::isfinite(maxCorrectionRadians) || maxCorrectionRadians <= 0.0f) {
            return false;
        }

        const RE::NiPoint3 targetAxisWorld = grab_finger_local_transform_math::alternateThumbPlaneCurlDirection(
            fingerPose.thumbAlternateCurveOpenDirectionWorld,
            fingerPose.thumbAlternateCurveNormalWorld,
            fingerPose.values[0],
            fingerPose.thumbAlternateCurveMaxCurlAngleRadians);
        const bool hasThumbSurface =
            fingerPose.surfaceAimTargetValid[0] != 0 &&
            fingerPose.surfaceAimNormalValid[0] != 0 &&
            liveNodes[2].valid;
        if (!grab_finger_local_transform_math::thumbAlternateSurfaceGuardAllows(
                liveNodes[2].world.translate,
                targetAxisWorld,
                fingerPose.surfaceAimTarget[0],
                fingerPose.surfaceAimNormal[0],
                surfaceSafetyMarginGameUnits,
                surfaceSafetyEnabled,
                hasThumbSurface)) {
            return false;
        }

        bool applied = false;
        for (std::size_t segment = 0; segment < 3; ++segment) {
            const std::uint16_t bit = static_cast<std::uint16_t>(1U << segment);
            const auto& node = liveNodes[segment];
            if ((transforms.enabledMask & bit) == 0 || !node.valid) {
                continue;
            }

            const float segmentStrength = grab_finger_local_transform_math::alternateThumbSegmentCorrectionStrength(segment, curveStrength);
            if (segmentStrength <= 0.0f) {
                continue;
            }

            const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                fingerPose.thumbAlternateCurveOpenDirectionWorld);
            const float dotToTarget = std::clamp(dot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
            float angle = std::acos(dotToTarget);
            if (!std::isfinite(angle) || angle <= 0.0001f) {
                continue;
            }
            angle = std::min(angle, maxCorrectionRadians) * segmentStrength;

            RE::NiPoint3 axis = cross(currentAxisWorld, targetAxisWorld);
            if (lengthSquared(axis) <= 0.000001f) {
                axis = orthogonalAxis(currentAxisWorld);
            }

            const RE::NiMatrix3 rotationDelta = axisAngleStored(axis, angle);
            const RE::NiMatrix3 targetWorldRotation = applyWorldRotationToStoredBasis(rotationDelta, node.world.rotate);
            RE::NiTransform localTransform = transforms.localTransforms[segment];
            localTransform.rotate = transform_math::multiplyStoredRotations(targetWorldRotation, transform_math::transposeRotation(node.parentWorld.rotate));
            if (!isFiniteTransform(localTransform)) {
                continue;
            }

            transforms.localTransforms[segment] = localTransform;
            applied = true;
        }

        return applied;
    }

    [[nodiscard]] inline DirectSkeletonBoneReader& rootFlattenedFingerReader()
    {
        static DirectSkeletonBoneReader reader;
        return reader;
    }

    [[nodiscard]] inline const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
    {
        for (const auto& bone : snapshot.bones) {
            if (bone.name == name) {
                return &bone;
            }
        }
        return nullptr;
    }

    [[nodiscard]] inline const DirectSkeletonBoneEntry* findSnapshotBoneByTreeIndex(const DirectSkeletonBoneSnapshot& snapshot, int treeIndex)
    {
        if (treeIndex < 0) {
            return nullptr;
        }

        for (const auto& bone : snapshot.bones) {
            if (bone.treeIndex == treeIndex) {
                return &bone;
            }
        }
        return nullptr;
    }

    [[nodiscard]] inline bool resolveLiveFingerTransforms(bool isLeft, std::array<LiveFingerTransform, 15>& outNodes)
    {
        outNodes = {};

        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedFingerReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            return false;
        }

        for (std::size_t finger = 0; finger < 5; ++finger) {
            for (std::size_t segment = 0; segment < 3; ++segment) {
                const std::size_t index = finger * 3 + segment;
                const char* boneName = root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, finger, segment);
                const auto* node = boneName ? findSnapshotBone(snapshot, boneName) : nullptr;
                const auto* parent = node ? findSnapshotBoneByTreeIndex(snapshot, node->parentTreeIndex) : nullptr;
                if (!node || !parent || !isFiniteTransform(node->world) || !isFiniteTransform(parent->world)) {
                    return false;
                }
                outNodes[index] = LiveFingerTransform{
                    .world = node->world,
                    .parentWorld = parent->world,
                    .valid = true,
                };
            }
        }
        return true;
    }

    [[nodiscard]] inline bool buildSurfaceCorrectedLocalTransforms(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const frik::api::FRIKApi::FingerLocalTransformOverride& baseline,
        Options options,
        frik::api::FRIKApi::FingerLocalTransformOverride& outTransforms,
        const char** outFailureReason = nullptr)
    {
        if (outFailureReason) {
            *outFailureReason = "none";
        }
        outTransforms = baseline;
        outTransforms.enabledMask = grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(baseline.enabledMask);
        if (outTransforms.enabledMask != grab_finger_local_transform_math::kFullFingerLocalTransformMask) {
            if (outFailureReason) {
                *outFailureReason = "baseline-mask";
            }
            return false;
        }

        options = sanitizeOptions(options);
        if (!options.enabled) {
            return true;
        }

        constexpr float kDegreesToRadians = 0.01745329251994329577f;
        const float maxCorrectionRadians = options.maxCorrectionDegrees * kDegreesToRadians;
        bool anyCorrected = false;

        const bool wantsSurfaceCorrection =
            options.maxCorrectionDegrees > 0.0f &&
            (options.surfaceAimStrength > 0.0f ||
                (fingerPose.thumbSurfaceFollowAllowed && options.thumbOppositionStrength > 0.0f));
        const bool wantsAlternateThumbPlaneCorrection =
            fingerPose.thumbSurfaceFollowAllowed &&
            grab_finger_local_transform_math::shouldApplyAlternateThumbLocalCorrection(
                fingerPose.usedAlternateThumbCurve,
                fingerPose.usedAlternateThumbSurfaceHit) &&
            options.maxCorrectionDegrees > 0.0f &&
            options.thumbAlternateCurveStrength > 0.0f;
        if (wantsAlternateThumbPlaneCorrection && !fingerPose.hasThumbAlternateCurveFrame) {
            if (outFailureReason) {
                *outFailureReason = "alternate-thumb-frame";
            }
            return false;
        }

        std::array<LiveFingerTransform, 15> liveNodes{};
        const bool needsLiveNodes = wantsSurfaceCorrection || wantsAlternateThumbPlaneCorrection;
        if (needsLiveNodes && !resolveLiveFingerTransforms(isLeft, liveNodes)) {
            if (outFailureReason) {
                *outFailureReason = "live-root-finger-transforms";
            }
            return false;
        }

        if (wantsSurfaceCorrection) {
            for (std::size_t index = 0; index < liveNodes.size(); ++index) {
                const auto& node = liveNodes[index];
                if (!node.valid) {
                    continue;
                }

                const std::size_t finger = index / 3;
                if (!fingerPose.surfaceAimTargetValid[finger]) {
                    continue;
                }
                if (!grab_finger_local_transform_math::shouldApplySurfaceAimCorrection(
                        finger,
                        wantsAlternateThumbPlaneCorrection,
                        fingerPose.thumbSurfaceFollowAllowed)) {
                    continue;
                }

                const float strength = grab_finger_local_transform_math::correctionStrengthForFinger(
                    finger, options.surfaceAimStrength, options.thumbOppositionStrength);
                if (strength <= 0.0f) {
                    continue;
                }

                const RE::NiPoint3 currentAxisWorld = normalizeOrFallback(
                    transform_math::rotateLocalVectorToWorld(node.world.rotate, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }),
                    RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
                const RE::NiPoint3 toSurface = fingerPose.surfaceAimTarget[finger] - node.world.translate;
                if (lengthSquared(toSurface) <= 0.000001f) {
                    continue;
                }

                const RE::NiPoint3 targetAxisWorld = normalizeOrFallback(toSurface, currentAxisWorld);
                const float dotToTarget = std::clamp(dot(currentAxisWorld, targetAxisWorld), -1.0f, 1.0f);
                float angle = std::acos(dotToTarget);
                if (!std::isfinite(angle) || angle <= 0.0001f) {
                    continue;
                }
                angle = std::min(angle * strength, maxCorrectionRadians);

                RE::NiPoint3 axis = cross(currentAxisWorld, targetAxisWorld);
                if (lengthSquared(axis) <= 0.000001f) {
                    axis = orthogonalAxis(currentAxisWorld);
                }

                const RE::NiMatrix3 rotationDelta = axisAngleStored(axis, angle);
                const RE::NiMatrix3 targetWorldRotation = applyWorldRotationToStoredBasis(rotationDelta, node.world.rotate);
                RE::NiTransform localTransform = baseline.localTransforms[index];
                localTransform.rotate = transform_math::multiplyStoredRotations(targetWorldRotation, transform_math::transposeRotation(node.parentWorld.rotate));
                if (!isFiniteTransform(localTransform)) {
                    continue;
                }

                outTransforms.localTransforms[index] = localTransform;
                anyCorrected = true;
            }
        }

        const bool appliedAlternateThumbCurve = wantsAlternateThumbPlaneCorrection && applyAlternateThumbPlaneCorrection(
            fingerPose,
            liveNodes,
            maxCorrectionRadians,
            options.thumbAlternateCurveStrength,
            options.thumbSurfaceSafetyEnabled,
            options.thumbSurfaceSafetyMarginGameUnits,
            outTransforms);

        if (wantsAlternateThumbPlaneCorrection && !appliedAlternateThumbCurve) {
            if (outFailureReason) {
                *outFailureReason = "alternate-thumb-plane";
            }
            return false;
        }

        if (!anyCorrected &&
            !appliedAlternateThumbCurve &&
            (wantsSurfaceCorrection || wantsAlternateThumbPlaneCorrection) &&
            outFailureReason) {
            *outFailureReason = "no-surface-correction";
        }
        return anyCorrected ||
            appliedAlternateThumbCurve ||
            (!wantsSurfaceCorrection && !wantsAlternateThumbPlaneCorrection);
    }

    [[nodiscard]] inline frik::api::FRIKApi::FingerLocalTransformOverride smoothLocalTransforms(
        const frik::api::FRIKApi::FingerLocalTransformOverride& target,
        State& state,
        float smoothingSpeed,
        float deltaTime)
    {
        frik::api::FRIKApi::FingerLocalTransformOverride smoothed = target;
        const std::uint16_t targetMask = grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(target.enabledMask);
        if (!state.hasCurrentTransforms || state.currentMask != targetMask) {
            for (std::size_t i = 0; i < state.currentTransforms.size(); ++i) {
                state.currentTransforms[i] = target.localTransforms[i];
            }
            state.currentMask = targetMask;
            state.hasCurrentTransforms = true;
            return smoothed;
        }

        const float alpha = grab_finger_local_transform_math::exponentialSmoothingAlpha(smoothingSpeed, deltaTime);
        for (std::size_t i = 0; i < state.currentTransforms.size(); ++i) {
            if ((targetMask & (1U << i)) == 0) {
                continue;
            }

            RE::NiTransform next = target.localTransforms[i];
            next.translate.x = state.currentTransforms[i].translate.x + (target.localTransforms[i].translate.x - state.currentTransforms[i].translate.x) * alpha;
            next.translate.y = state.currentTransforms[i].translate.y + (target.localTransforms[i].translate.y - state.currentTransforms[i].translate.y) * alpha;
            next.translate.z = state.currentTransforms[i].translate.z + (target.localTransforms[i].translate.z - state.currentTransforms[i].translate.z) * alpha;
            next.scale = state.currentTransforms[i].scale + (target.localTransforms[i].scale - state.currentTransforms[i].scale) * alpha;

            const auto currentRotation = hand_visual_lerp_math::matrixToQuaternion(state.currentTransforms[i].rotate);
            const auto targetRotation = hand_visual_lerp_math::matrixToQuaternion(target.localTransforms[i].rotate);
            next.rotate = hand_visual_lerp_math::quaternionToMatrix<RE::NiMatrix3>(
                hand_visual_lerp_math::slerp(currentRotation, targetRotation, alpha));

            state.currentTransforms[i] = next;
            smoothed.localTransforms[i] = next;
        }
        smoothed.enabledMask = targetMask;
        return smoothed;
    }

    inline void clearLocalTransformOverride(const char* tag, frik::api::FRIKApi::Hand hand, int priority, State& state)
    {
        auto* api = frik::api::FRIKApi::inst;
        if (!api || !api->setHandPoseCustomLocalTransformsWithPriority || !state.hasCurrentTransforms) {
            state = {};
            return;
        }

        frik::api::FRIKApi::FingerLocalTransformOverride clearData{};
        api->setHandPoseCustomLocalTransformsWithPriority(tag, hand, &clearData, priority);
        state = {};
    }

    [[nodiscard]] inline bool publishLocalTransformPose(
        const char* tag,
        frik::api::FRIKApi::Hand hand,
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const std::array<float, 15>& jointValues,
        Options options,
        float deltaTime,
        int priority,
        State& state)
    {
        auto* api = frik::api::FRIKApi::inst;
        const bool canPublish = api &&
            grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                options.enabled,
                fingerPose.solved,
                fingerPose.hasJointValues,
                api->getHandPoseLocalTransformsForJointPositions != nullptr,
                api->setHandPoseCustomLocalTransformsWithPriority != nullptr);
        if (!canPublish) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik::api::FRIKApi::FingerLocalTransformOverride baseline{};
        if (!api->getHandPoseLocalTransformsForJointPositions(hand, jointValues.data(), &baseline)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik::api::FRIKApi::FingerLocalTransformOverride corrected{};
        if (!buildSurfaceCorrectedLocalTransforms(isLeft, fingerPose, baseline, options, corrected)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        const Options sanitizedOptions = sanitizeOptions(options);
        const auto smoothed = smoothLocalTransforms(corrected, state, sanitizedOptions.smoothingSpeed, deltaTime);
        api->setHandPoseCustomLocalTransformsWithPriority(tag, hand, &smoothed, priority);
        return true;
    }
}

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
        float contactCenterX = 0.0f;
        float contactCenterY = 0.0f;
        float contactCenterZ = 0.0f;
        float contactRadius = 0.0f;
        bool hasHitPoint = false;
        bool hasHitNormal = false;
        bool hasContactCenter = false;
        std::uint8_t selectedProbeIndex = 0xFF;
        std::array<std::uint16_t, 3> sweptProbeStartRow{};
        std::array<std::uint8_t, 3> sweptProbeStartRowValid{};
        bool openedByBehindContact = false;
        /*
         * Swept-arc solver only: no candidate triangle lies within the finger's
         * whole closing arc, so the finger cannot touch the object from here.
         * Callers use this to hold an anticipation pose during pull-to-grab
         * flight instead of closing on nothing.
         */
        bool outOfReach = false;
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
        FingerCurlValue sidePadThumb{};
        FingerCurlValue selectedThumbCurve{};
        grab_finger_calibration_data::BakedGrabThumbLane selectedThumbLane =
            grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
        float selectedThumbLaneNormalBlend = 0.0f;
        float selectedThumbLaneLocalCorrectionStrength = 0.0f;
        float selectedThumbLaneMaxCurlAngleRadians = 0.0f;
        Vector selectedThumbLaneNormal{};
        bool usedAlternateThumbCurve = false;
    };

    [[nodiscard]] inline const char* thumbLaneName(grab_finger_calibration_data::BakedGrabThumbLane lane)
    {
        using grab_finger_calibration_data::BakedGrabThumbLane;
        switch (lane) {
        case BakedGrabThumbLane::Opposition:
            return "opposition";
        case BakedGrabThumbLane::SidePad:
            return "sidePad";
        case BakedGrabThumbLane::Wrap:
        default:
            return "wrap";
        }
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
        return vector_math::cross(a, b);
    }

    template <class Vector>
    inline float dot(const Vector& a, const Vector& b)
    {
        return vector_math::dot(a, b);
    }

    template <class Vector>
    inline float length(const Vector& value)
    {
        return std::sqrt(dot(value, value));
    }

    template <class Vector>
    inline float lengthSquared(const Vector& value)
    {
        return vector_math::lengthSquared(value);
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

    /*
     * Closed-mesh occupancy for the over-open clearance decision. A proximity
     * sphere cannot distinguish "free" from "deep inside": once a probe is
     * farther than its radius from the enclosing surface, both cases return
     * no contact. The signed solid-angle sum does make that distinction while
     * deliberately rejecting isolated/open sheets (their winding magnitude
     * stays at or below one half). Inconsistent or incomplete winding fails
     * closed to the authored-open sweep; it can never manufacture an unsafe
     * over-open result.
     */
    template <class TriangleContainer, class Vector>
    [[nodiscard]] inline bool pointInsideClosedTriangleMesh(const TriangleContainer& triangles, const Vector& point)
    {
        constexpr double kTwoPi = 6.28318530717958647692;
        constexpr double kPointEpsilonSquared = 1.0e-12;
        double signedSolidAngle = 0.0;
        for (const auto& triangle : triangles) {
            const Vector a = sub(triangle.v0, point);
            const Vector b = sub(triangle.v1, point);
            const Vector c = sub(triangle.v2, point);
            const double aSquared = static_cast<double>(lengthSquared(a));
            const double bSquared = static_cast<double>(lengthSquared(b));
            const double cSquared = static_cast<double>(lengthSquared(c));
            if (!std::isfinite(aSquared) || !std::isfinite(bSquared) || !std::isfinite(cSquared)) {
                continue;
            }
            if (aSquared <= kPointEpsilonSquared || bSquared <= kPointEpsilonSquared ||
                cSquared <= kPointEpsilonSquared) {
                return true;
            }

            const double aLength = std::sqrt(aSquared);
            const double bLength = std::sqrt(bSquared);
            const double cLength = std::sqrt(cSquared);
            const double numerator = static_cast<double>(dot(a, cross(b, c)));
            const double denominator = aLength * bLength * cLength + static_cast<double>(dot(a, b)) * cLength +
                static_cast<double>(dot(b, c)) * aLength + static_cast<double>(dot(c, a)) * bLength;
            signedSolidAngle += 2.0 * std::atan2(numerator, denominator);
        }
        return std::isfinite(signedSolidAngle) && std::abs(signedSolidAngle) > kTwoPi;
    }

    template <class Vector>
    inline std::vector<Triangle<Vector>> filterTrianglesNearPoint(const std::vector<Triangle<Vector>>& triangles,
        const Vector& point, float maxDistanceSquared)
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
    inline bool planeIntersectsSegment(const Vector& planePoint, const Vector& planeNormal, const Vector& a,
        const Vector& b, Vector& outPoint)
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
    inline bool rayTriangleIntersection(const Vector& origin, const Vector& direction, const Triangle<Vector>& triangle,
        float maxDistance, float& outT)
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
    inline float segmentSegmentDistanceSquared(const Vector& p1, const Vector& q1, const Vector& p2, const Vector& q2,
        float& outFirstSegmentT, float* outSecondSegmentT = nullptr)
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
    inline bool probeCapsuleTriangleIntersection(const Vector& origin, const Vector& direction,
        const Triangle<Vector>& triangle, float maxDistance, float probeRadius, float& outT,
        Vector* outClosestPoint = nullptr)
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
            const float distanceSquared =
                segmentSegmentDistanceSquared(origin, segmentEnd, a, b, segmentRatio, &edgeRatio);
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
    inline FingerCurlValue solveFingerCurlValue(const std::vector<Triangle<Vector>>& triangles, const Vector& origin,
        const Vector& direction, float maxDistance, float minValue, float probeRadius,
        const Vector& curlNormal = Vector{ 0.0f, 0.0f, 1.0f },
        const Vector& zeroAngleVector = Vector{ 1.0f, 0.0f, 0.0f }, float maxCurlAngleRadians = 3.14159265358979323846f,
        const Vector& surfacePoint = Vector{}, const Vector& surfaceNormal = Vector{}, bool rejectBacksideHits = false,
        float surfacePlaneToleranceGameUnits = 0.0f, Vector* outHitPoint = nullptr)
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
        const float planeTolerance =
            std::max(0.0f, std::isfinite(surfacePlaneToleranceGameUnits) ? surfacePlaneToleranceGameUnits : 0.0f);
        const float maxCurlAngle = std::isfinite(maxCurlAngleRadians) && maxCurlAngleRadians > 0.0f
            ? maxCurlAngleRadians
            : 3.14159265358979323846f;
        for (const auto& triangle : triangles) {
            float t = 0.0f;
            Vector capsuleHitPoint{};
            bool hitPointFromCapsule = false;
            if (!rayTriangleIntersection(origin, dir, triangle, maxDistance, t)) {
                if (!probeCapsuleTriangleIntersection(origin, dir, triangle, maxDistance, probeRadius, t,
                        &capsuleHitPoint)) {
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

    enum class CalibratedFingerProbe : std::uint8_t
    {
        Tip,
        Outer,
        Inner
    };

    inline constexpr std::size_t kCalibratedFingerCurveSampleCount = 201;
    static_assert(grab_finger_calibration_data::kGrabFingerCalibrationSampleCount == kCalibratedFingerCurveSampleCount,
        "Generated grab-finger calibration sample count must match the runtime solver sample count.");

    /*
     * kMaxFingerOpenValue is the authored open pose (hFRIK slerp t = 1);
     * kMaxOverOpenValue is the hFRIK flex ceiling (slerp extrapolates to
     * t = 2, hyper-extension past the authored open pose). The baked arc
     * tables span [0, kMaxOverOpenValue] for every finger: rows past 1.0
     * carry negative angles relative to the value-1.0 zero reference. How
     * far the sweep may actually open is a per-call cap (config-driven),
     * not a table property.
     */
    inline constexpr float kMaxFingerOpenValue = 1.0f;
    inline constexpr float kMaxOverOpenValue = 2.0f;

    template <class Vector>
    struct CalibratedFingerCurveSample
    {
        float openValue = 1.0f;
        float angleRadians = 0.0f;
        float reachLength = 0.0f;
    };

    template <class Vector>
    struct CalibratedFingerProbeCurve
    {
        CalibratedFingerProbe probe = CalibratedFingerProbe::Tip;
        std::array<CalibratedFingerCurveSample<Vector>, kCalibratedFingerCurveSampleCount> samples{};
        std::size_t sampleCount = 0;
    };

    template <class Vector>
    struct CalibratedFingerCurve
    {
        Vector center{};
        Vector normal{ 0.0f, 0.0f, 1.0f };
        Vector zeroAngleVector{ 1.0f, 0.0f, 0.0f };
        std::array<CalibratedFingerProbeCurve<Vector>, 3> probes{};
        std::size_t probeCount = 0;
        float surfaceThickness = 0.35f;
    };

    template <class Vector>
    inline float maxCalibratedCurveAngle(const CalibratedFingerCurve<Vector>& curve)
    {
        float result = 0.0f;
        for (std::size_t i = 0; i < curve.probeCount && i < curve.probes.size(); ++i) {
            const auto& probe = curve.probes[i];
            if (probe.sampleCount > 0 && probe.sampleCount <= probe.samples.size()) {
                result = (std::max)(result, probe.samples[probe.sampleCount - 1].angleRadians);
            }
        }
        return result;
    }

    template <class Vector>
    inline float maxCalibratedCurveReach(const CalibratedFingerCurve<Vector>& curve)
    {
        float result = 0.0f;
        for (std::size_t probeIndex = 0; probeIndex < curve.probeCount && probeIndex < curve.probes.size();
            ++probeIndex) {
            const auto& probe = curve.probes[probeIndex];
            if (probe.sampleCount == 0 || probe.sampleCount > probe.samples.size()) {
                continue;
            }
            for (std::size_t sampleIndex = 0; sampleIndex < probe.sampleCount; ++sampleIndex) {
                result = (std::max)(result, probe.samples[sampleIndex].reachLength);
            }
        }
        return result;
    }

    [[nodiscard]] inline CalibratedFingerProbe calibratedFingerProbeFromBaked(
        grab_finger_calibration_data::BakedGrabFingerProbe probe)
    {
        using grab_finger_calibration_data::BakedGrabFingerProbe;
        switch (probe) {
        case BakedGrabFingerProbe::Outer:
            return CalibratedFingerProbe::Outer;
        case BakedGrabFingerProbe::Inner:
            return CalibratedFingerProbe::Inner;
        case BakedGrabFingerProbe::Tip:
        default:
            return CalibratedFingerProbe::Tip;
        }
    }

    template <class Vector>
    inline CalibratedFingerProbeCurve<Vector> makeBakedCalibratedFingerProbeCurve(
        const grab_finger_calibration_data::BakedGrabFingerProbeCurve& baked, float fingerLength)
    {
        CalibratedFingerProbeCurve<Vector> curve{};
        curve.probe = calibratedFingerProbeFromBaked(baked.probe);
        if (!std::isfinite(fingerLength) || fingerLength <= 0.0001f) {
            return curve;
        }

        curve.sampleCount = curve.samples.size();
        for (std::size_t i = 0; i < curve.samples.size(); ++i) {
            const auto& sample = baked.samples[i];
            /*
             * Over-open rows (openValue > 1) legitimately carry negative
             * angles: they extend the arc on the far side of the value-1.0
             * zero reference.
             */
            curve.samples[i] = CalibratedFingerCurveSample<Vector>{
                .openValue =
                    std::clamp(std::isfinite(sample.openValue) ? sample.openValue : 1.0f, 0.0f, kMaxOverOpenValue),
                .angleRadians = std::isfinite(sample.angleRadians) ? sample.angleRadians : 0.0f,
                .reachLength =
                    std::max(0.0001f, (std::isfinite(sample.reachScale) ? sample.reachScale : 1.0f) * fingerLength),
            };
        }
        return curve;
    }

    template <class Vector>
    inline CalibratedFingerCurve<Vector> makeBakedCalibratedFingerCurveFromBaked(
        const grab_finger_calibration_data::BakedGrabFingerCurve& baked, const Vector& center, const Vector& normal,
        const Vector& zeroAngleVector, float fingerLength, bool applyBakedNormalSign = true,
        float surfaceThicknessScaleOverride = -1.0f)
    {
        CalibratedFingerCurve<Vector> curve{};
        if (!std::isfinite(fingerLength) || fingerLength <= 0.0001f) {
            return curve;
        }

        const float normalSign = applyBakedNormalSign && baked.normalSign < 0.0f ? -1.0f : 1.0f;
        const float rawThicknessScale =
            std::isfinite(surfaceThicknessScaleOverride) && surfaceThicknessScaleOverride >= 0.0f
            ? surfaceThicknessScaleOverride
            : (std::isfinite(baked.surfaceThicknessScale) ? baked.surfaceThicknessScale : 0.05f);
        const float thicknessScale = std::clamp(rawThicknessScale, 0.0f, 0.25f);

        curve.center = center;
        curve.normal = scale(normal, normalSign);
        curve.zeroAngleVector = zeroAngleVector;
        curve.surfaceThickness = std::clamp(fingerLength * thicknessScale, 0.25f, 0.75f);
        curve.probeCount = curve.probes.size();
        for (std::size_t probe = 0; probe < curve.probes.size(); ++probe) {
            curve.probes[probe] = makeBakedCalibratedFingerProbeCurve<Vector>(baked.probes[probe], fingerLength);
        }
        return curve;
    }

    template <class Vector>
    inline CalibratedFingerCurve<Vector> makeBakedCalibratedFingerCurve(std::size_t fingerIndex, bool isLeft,
        bool inPowerArmor, const Vector& center, const Vector& normal, const Vector& zeroAngleVector,
        float fingerLength, bool applyBakedNormalSign = true)
    {
        CalibratedFingerCurve<Vector> curve{};
        if (fingerIndex >= 5) {
            return curve;
        }

        const auto& profile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        return makeBakedCalibratedFingerCurveFromBaked<Vector>(profile.fingers[fingerIndex], center, normal,
            zeroAngleVector, fingerLength, applyBakedNormalSign);
    }

    template <class Vector>
    [[nodiscard]] inline Vector blendedThumbLaneNormal(const Vector& primaryNormal, const Vector& alternateThumbNormal,
        float normalBlend)
    {
        const float blend = std::clamp(std::isfinite(normalBlend) ? normalBlend : 0.0f, 0.0f, 1.0f);
        const Vector blended = add(scale(primaryNormal, 1.0f - blend), scale(alternateThumbNormal, blend));
        if (hasUsableDirection(blended)) {
            return normalize(blended);
        }
        return blend >= 0.5f ? normalize(alternateThumbNormal) : normalize(primaryNormal);
    }

    template <class Vector>
    inline CalibratedFingerCurve<Vector> makeBakedCalibratedThumbLaneCurve(
        const grab_finger_calibration_data::BakedGrabThumbLaneCurve& lane,
        const grab_finger_calibration_data::BakedGrabFingerCurve& bakedCurve, const Vector& center,
        const Vector& normal, const Vector& zeroAngleVector, float fingerLength)
    {
        return makeBakedCalibratedFingerCurveFromBaked<Vector>(bakedCurve, center, normal, zeroAngleVector,
            fingerLength, lane.applyAuthoredNormalSign, lane.surfaceThicknessScale);
    }

    [[nodiscard]] inline float maxBakedFingerCurveAngleRadians(
        const grab_finger_calibration_data::BakedGrabFingerCurve& baked)
    {
        float result = 0.0f;
        for (const auto& probe : baked.probes) {
            result = (std::max)(result, probe.samples.back().angleRadians);
        }
        return result;
    }

    struct CalibratedChainCurlEstimate
    {
        float chordAngleRadians = 0.0f;
        float openValue = 1.0f;
        float normalSign = 1.0f;
        bool valid = false;
    };

    /*
     * The runtime anchors every arc reconstruction on the live chain CHORD
     * (pad - base), which is the true zero reference only while the chain is
     * fully open. The calibration's Tip probe tracks the pad point relative to
     * the base, so at any curl the chord direction is rotated by exactly the
     * Tip probe's baked angle and the chord length equals the Tip probe's
     * baked reach. Inverting the reach table therefore recovers the current
     * curl - and the chord's rotation - from live geometry alone: no
     * published-value bookkeeping, exact even mid-smoothing. normalSign is the
     * baked arc-plane sign the reconstruction applies to the caller's curl
     * normal; de-rotation must rotate by -chordAngle * normalSign around the
     * unsigned curl normal to stay in the same plane convention.
     * Reach decreases toward closed; the scan takes the first bracketing pair
     * from the open end so non-monotonic wiggles fail toward LESS de-rotation.
     *
     * The scan is restricted to rows with openValue <= 1: past the authored
     * open pose the chord SHORTENS again (hyper-extension folds the tip the
     * other way), so reach is ambiguous across the over-open region and a
     * length inversion there would map a curled chord onto an over-open
     * angle. Fingers holding an over-open pose use the recorded contact-row
     * rotation (GrabFingerArcAnchorHints) instead of this inversion.
     */
    [[nodiscard]] inline CalibratedChainCurlEstimate estimateCalibratedChainCurlFromChord(std::size_t fingerIndex,
        bool isLeft, bool inPowerArmor, float fingerLength, float liveChordLength)
    {
        CalibratedChainCurlEstimate result{};
        if (fingerIndex >= 5 || !std::isfinite(fingerLength) || fingerLength <= 0.0001f ||
            !std::isfinite(liveChordLength) || liveChordLength <= 0.0001f) {
            return result;
        }

        const auto& profile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        const auto& baked = profile.fingers[fingerIndex];
        const grab_finger_calibration_data::BakedGrabFingerProbeCurve* tipProbe = nullptr;
        for (const auto& probe : baked.probes) {
            if (probe.probe == grab_finger_calibration_data::BakedGrabFingerProbe::Tip) {
                tipProbe = &probe;
                break;
            }
        }
        if (!tipProbe || tipProbe->samples.empty()) {
            return result;
        }

        result.normalSign = baked.normalSign < 0.0f ? -1.0f : 1.0f;
        const auto& samples = tipProbe->samples;
        std::size_t scanStart = 0;
        while (scanStart < samples.size() && samples[scanStart].openValue > kMaxFingerOpenValue + 0.0001f) {
            ++scanStart;
        }
        if (scanStart >= samples.size()) {
            return result;
        }
        const float chordScale = liveChordLength / fingerLength;
        if (chordScale >= samples[scanStart].reachScale) {
            result.chordAngleRadians = samples[scanStart].angleRadians;
            result.openValue = samples[scanStart].openValue;
            result.valid = true;
            return result;
        }
        for (std::size_t i = scanStart; i + 1 < samples.size(); ++i) {
            const float reachA = samples[i].reachScale;
            const float reachB = samples[i + 1].reachScale;
            if (chordScale <= reachA && chordScale >= reachB) {
                const float span = reachA - reachB;
                const float t = span > 0.000001f ? std::clamp((reachA - chordScale) / span, 0.0f, 1.0f) : 0.0f;
                result.chordAngleRadians =
                    samples[i].angleRadians + (samples[i + 1].angleRadians - samples[i].angleRadians) * t;
                result.openValue = samples[i].openValue + (samples[i + 1].openValue - samples[i].openValue) * t;
                result.valid = true;
                return result;
            }
        }
        result.chordAngleRadians = samples.back().angleRadians;
        result.openValue = samples.back().openValue;
        result.valid = true;
        return result;
    }

    [[nodiscard]] inline float bakedCalibratedFingerMaxAngleRadians(std::size_t fingerIndex, bool isLeft,
        bool inPowerArmor)
    {
        if (fingerIndex >= 5) {
            return 0.0f;
        }

        const auto& profile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        return maxBakedFingerCurveAngleRadians(profile.fingers[fingerIndex]);
    }

    template <class Vector>
    inline Vector rotateAroundUnitAxis(const Vector& v, const Vector& unitAxis, float angleRadians)
    {
        const float c = std::cos(angleRadians);
        const float s = std::sin(angleRadians);
        const Vector axisCross = cross(unitAxis, v);
        const float axisDot = dot(unitAxis, v);
        return Vector{
            v.x * c + axisCross.x * s + unitAxis.x * axisDot * (1.0f - c),
            v.y * c + axisCross.y * s + unitAxis.y * axisDot * (1.0f - c),
            v.z * c + axisCross.z * s + unitAxis.z * axisDot * (1.0f - c),
        };
    }

    /*
     * Swept-arc finger curl solver. Every allowed baked row is geometrically
     * walked from the hFRIK flex ceiling toward closed. Contacts above the
     * authored open pose are clearance evidence, not automatically a selected
     * pose: an over-open contact is accepted only when the corresponding probe
     * at value 1.0 touches the mesh or lies inside a consistently wound closed
     * shell. This preserves the skull/dorsal-graze fix while also detecting the
     * thick-object case that a single value-1.0 proximity sphere cannot see.
     */
    template <class Vector, class SphereContactQuery, class PointInsideQuery>
    inline FingerCurlValue sweepCalibratedFingerCurveCurlValueWithContactQuery(
        const CalibratedFingerCurve<Vector>& curve, float minValue, float contactRadiusGameUnits, float maxOpenValue,
        SphereContactQuery&& sphereContact, PointInsideQuery&& pointInside)
    {
        FingerCurlValue result{};
        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        const float clampedMaxOpen = std::clamp(std::isfinite(maxOpenValue) ? maxOpenValue : kMaxOverOpenValue,
            kMaxFingerOpenValue, kMaxOverOpenValue);
        result.value = clampedMin;
        if (curve.probeCount == 0) {
            result.outOfReach = true;
            return result;
        }

        const Vector planeNormal = normalize(curve.normal);
        const Vector zero = normalize(curve.zeroAngleVector);
        const float radius =
            std::clamp(std::isfinite(contactRadiusGameUnits) ? contactRadiusGameUnits : 1.0f, 0.05f, 4.0f);
        const float maxReach = maxCalibratedCurveReach(curve);
        if (!std::isfinite(maxReach) || maxReach <= 0.0001f) {
            result.outOfReach = true;
            return result;
        }

        const float filterRadius = maxReach + radius + 0.5f;
        if (!sphereContact(curve.center, filterRadius, nullptr, nullptr)) {
            result.outOfReach = true;
            return result;
        }

        constexpr std::size_t kCoarseRowStep = 4;

        struct ContactCandidate
        {
            float openValue = -1.0f;
            float angle = 0.0f;
            Vector hitPoint{};
            Vector hitNormal{};
            Vector contactCenter{};
            std::uint8_t probeIndex = 0xFF;
            bool hitPointValid = false;
            bool hitNormalValid = false;

            [[nodiscard]] bool valid() const { return openValue >= 0.0f; }
        };

        ContactCandidate bestOverOpen{};
        ContactCandidate bestAuthoredOrClosed{};

        auto keepMostOpen = [](ContactCandidate& destination, const ContactCandidate& candidate) {
            if (candidate.valid() && candidate.openValue > destination.openValue + 0.0001f) {
                destination = candidate;
            }
        };

        for (std::size_t probeIndex = 0; probeIndex < curve.probeCount && probeIndex < curve.probes.size();
            ++probeIndex) {
            const auto& probe = curve.probes[probeIndex];
            if (probe.sampleCount == 0 || probe.sampleCount > probe.samples.size()) {
                continue;
            }

            // Rows are ordered most-open first; only an explicit cap can trim
            // the 2.0 -> 0.0 calibrated domain.
            std::size_t overOpenStartRow = 0;
            while (overOpenStartRow < probe.sampleCount &&
                probe.samples[overOpenStartRow].openValue > clampedMaxOpen + 0.0001f) {
                ++overOpenStartRow;
            }
            if (overOpenStartRow >= probe.sampleCount) {
                continue;
            }
            std::size_t authoredOpenRow = overOpenStartRow;
            while (authoredOpenRow < probe.sampleCount &&
                probe.samples[authoredOpenRow].openValue > kMaxFingerOpenValue + 0.0001f) {
                ++authoredOpenRow;
            }
            if (authoredOpenRow >= probe.sampleCount) {
                continue;
            }

            std::array<Vector, kCalibratedFingerCurveSampleCount> rowPositions{};
            float maxRowGap = 0.0f;
            for (std::size_t i = overOpenStartRow; i < probe.sampleCount; ++i) {
                const auto& sample = probe.samples[i];
                const Vector arm = rotateAroundUnitAxis(zero, planeNormal, sample.angleRadians);
                rowPositions[i] = add(curve.center, scale(arm, sample.reachLength));
                if (i > overOpenStartRow) {
                    maxRowGap = (std::max)(maxRowGap, length(sub(rowPositions[i], rowPositions[i - 1])));
                }
            }
            const float coarseRadius = radius + maxRowGap * static_cast<float>(kCoarseRowStep);

            if (probeIndex < result.sweptProbeStartRow.size()) {
                result.sweptProbeStartRow[probeIndex] = static_cast<std::uint16_t>(overOpenStartRow);
                result.sweptProbeStartRowValid[probeIndex] = 1;
            }

            auto firstContactInRange = [&](std::size_t beginRow, std::size_t endRow) {
                ContactCandidate candidate{};
                if (beginRow >= endRow || endRow > probe.sampleCount) {
                    return candidate;
                }
                bool found = false;
                for (std::size_t coarse = beginRow; coarse < endRow && !found; coarse += kCoarseRowStep) {
                    if (!sphereContact(rowPositions[coarse], coarseRadius, nullptr, nullptr)) {
                        continue;
                    }
                    const std::size_t bracketBegin =
                        coarse >= beginRow + kCoarseRowStep ? coarse - kCoarseRowStep : beginRow;
                    const std::size_t bracketEnd = (std::min)(coarse + kCoarseRowStep, endRow - 1);
                    for (std::size_t row = bracketBegin; row <= bracketEnd; ++row) {
                        Vector hitPoint{};
                        Vector hitNormal{};
                        if (!sphereContact(rowPositions[row], radius, &hitPoint, &hitNormal)) {
                            continue;
                        }
                        candidate.openValue = std::clamp(probe.samples[row].openValue, 0.0f, clampedMaxOpen);
                        candidate.angle = probe.samples[row].angleRadians;
                        candidate.hitPoint = hitPoint;
                        candidate.hitNormal = hitNormal;
                        candidate.contactCenter = rowPositions[row];
                        candidate.probeIndex = static_cast<std::uint8_t>(probeIndex);
                        candidate.hitPointValid = true;
                        candidate.hitNormalValid = hasUsableDirection(hitNormal);
                        found = true;
                        break;
                    }
                }
                return candidate;
            };

            /*
             * The geometric walk always starts at the configured ceiling.
             * Selection is split at authored-open so a dorsal-only contact can
             * be ignored without hiding the 2 -> 1 path from clearance logic
             * or diagnostics.
             */
            const ContactCandidate overOpenContact = firstContactInRange(overOpenStartRow, authoredOpenRow);
            if (overOpenContact.valid()) {
                const bool authoredOpenTouches = sphereContact(rowPositions[authoredOpenRow], radius, nullptr, nullptr);
                const bool authoredOpenInside = !authoredOpenTouches && pointInside(rowPositions[authoredOpenRow]);
                if (authoredOpenTouches || authoredOpenInside) {
                    keepMostOpen(bestOverOpen, overOpenContact);
                }
            }
            keepMostOpen(bestAuthoredOrClosed, firstContactInRange(authoredOpenRow, probe.sampleCount));
        }

        const ContactCandidate& selected = bestOverOpen.valid() ? bestOverOpen : bestAuthoredOrClosed;
        if (selected.valid()) {
            result.hit = true;
            result.hitKind = FingerCurlValue::HitKind::FrontValid;
            result.distance = selected.angle;
            result.rawCurveValue = selected.openValue;
            result.value = std::clamp(selected.openValue, clampedMin, clampedMaxOpen);
            result.contactCenterX = selected.contactCenter.x;
            result.contactCenterY = selected.contactCenter.y;
            result.contactCenterZ = selected.contactCenter.z;
            result.contactRadius = radius;
            result.hasContactCenter = true;
            result.selectedProbeIndex = selected.probeIndex;
            if (selected.hitPointValid) {
                result.hitPointX = selected.hitPoint.x;
                result.hitPointY = selected.hitPoint.y;
                result.hitPointZ = selected.hitPoint.z;
                result.hasHitPoint = true;
            }
            if (selected.hitNormalValid) {
                result.hitNormalX = selected.hitNormal.x;
                result.hitNormalY = selected.hitNormal.y;
                result.hitNormalZ = selected.hitNormal.z;
                result.hasHitNormal = true;
            }
        }
        return result;
    }

    template <class Vector>
    inline FingerCurlValue sweepCalibratedFingerCurveCurlValue(const std::vector<Triangle<Vector>>& triangles,
        const CalibratedFingerCurve<Vector>& curve, float minValue, float contactRadiusGameUnits,
        float maxOpenValue = kMaxOverOpenValue)
    {
        const float radius =
            std::clamp(std::isfinite(contactRadiusGameUnits) ? contactRadiusGameUnits : 1.0f, 0.05f, 4.0f);
        const float maxReach = maxCalibratedCurveReach(curve);
        const float filterRadius = std::isfinite(maxReach) && maxReach > 0.0001f ? maxReach + radius + 0.5f : 0.0f;
        const auto nearTriangles = filterRadius > 0.0f
            ? filterTrianglesNearPoint(triangles, curve.center, filterRadius * filterRadius)
            : std::vector<Triangle<Vector>>{};

        auto sphereContact = [&](const Vector& position, float testRadius, Vector* outPoint, Vector* outNormal) {
            const float radiusSquared = testRadius * testRadius;
            float bestDistanceSquared = radiusSquared;
            bool found = false;
            for (const auto& triangle : nearTriangles) {
                const Vector closest = closestPointOnTriangle(position, triangle);
                const float distanceSq = lengthSquared(sub(closest, position));
                if (distanceSq <= bestDistanceSquared) {
                    bestDistanceSquared = distanceSq;
                    found = true;
                    if (outPoint) {
                        *outPoint = closest;
                    }
                    if (outNormal) {
                        /*
                         * Orient the triangle normal toward the probe so the
                         * published surface normal always faces the finger pad.
                         */
                        Vector normal = triangleNormal(triangle);
                        if (dot(normal, sub(position, closest)) < 0.0f) {
                            normal = scale(normal, -1.0f);
                        }
                        *outNormal = normal;
                    }
                }
            }
            return found;
        };

        auto pointInside = [&](const Vector& position) { return pointInsideClosedTriangleMesh(triangles, position); };

        return sweepCalibratedFingerCurveCurlValueWithContactQuery(curve, minValue, contactRadiusGameUnits,
            maxOpenValue, sphereContact, pointInside);
    }

    /*
     * Thumb-aware wrapper over the swept-arc solver. Lane bookkeeping (wrap
     * first, then opposition/side-pad candidates with their normal blends and
     * local correction strengths) intentionally mirrors the retired
     * plane-slice selector so the downstream local-transform machinery keeps
     * its contract; only the per-lane curl solve changed.
     */
    template <class Vector, class CurveSolver>
    inline ThumbAwareFingerCurveCurlValue<Vector> sweepThumbAwareCalibratedFingerCurveCurlValueWithCurveSolver(
        std::size_t fingerIndex, bool isLeft, bool inPowerArmor, const Vector& center, const Vector& primaryNormal,
        const Vector& alternateThumbNormal, const Vector& zeroAngleVector, float fingerLength, float minValue,
        bool allowAlternateThumbCurve, float contactRadiusGameUnits, float maxOpenValue, CurveSolver&& solveCurve)
    {
        ThumbAwareFingerCurveCurlValue<Vector> result{};
        const auto primaryCurve = makeBakedCalibratedFingerCurve(fingerIndex, isLeft, inPowerArmor, center,
            primaryNormal, zeroAngleVector, fingerLength);
        result.primary = solveCurve(primaryCurve, minValue, contactRadiusGameUnits, maxOpenValue);
        result.value = result.primary;
        result.selectedThumbCurve = result.primary;
        result.selectedThumbLane = grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
        result.selectedThumbLaneNormal = normalize(primaryCurve.normal);
        result.selectedThumbLaneMaxCurlAngleRadians = maxCalibratedCurveAngle(primaryCurve);

        if (!allowAlternateThumbCurve || fingerIndex != 0) {
            return result;
        }

        constexpr float kClosedEpsilon = 0.0001f;
        const bool primaryClosedOrMissed = !result.primary.hit || result.primary.rawCurveValue <= kClosedEpsilon;
        const bool primaryNeedsAlternate = primaryClosedOrMissed || result.primary.openedByBehindContact;

        struct ThumbLaneSolveCandidate
        {
            grab_finger_calibration_data::BakedGrabThumbLane lane =
                grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
            FingerCurlValue value{};
            Vector normal{};
            float normalBlend = 0.0f;
            float localCorrectionStrength = 0.0f;
            float maxCurlAngleRadians = 0.0f;
            bool valid = false;
        };

        ThumbLaneSolveCandidate bestPositive{};
        ThumbLaneSolveCandidate closedFallback{};
        const auto& thumbProfile = grab_finger_calibration_data::bakedGrabThumbProfile(isLeft, inPowerArmor);
        const auto& thumbHandProfile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        for (const auto& lane : thumbProfile.lanes) {
            if (lane.lane == grab_finger_calibration_data::BakedGrabThumbLane::Wrap) {
                continue;
            }

            const auto& bakedLaneCurve =
                lane.curveSource == grab_finger_calibration_data::BakedGrabThumbCurveSource::SidePad
                ? thumbProfile.sidePadCurve
                : thumbHandProfile.fingers[0];
            const Vector laneNormal = blendedThumbLaneNormal(primaryNormal, alternateThumbNormal, lane.normalBlend);
            const auto laneCurve = makeBakedCalibratedThumbLaneCurve<Vector>(lane, bakedLaneCurve, center, laneNormal,
                zeroAngleVector, fingerLength);
            const FingerCurlValue laneSolved = solveCurve(laneCurve, minValue, contactRadiusGameUnits, maxOpenValue);

            if (lane.lane == grab_finger_calibration_data::BakedGrabThumbLane::Opposition) {
                result.alternateThumb = laneSolved;
            } else if (lane.lane == grab_finger_calibration_data::BakedGrabThumbLane::SidePad) {
                result.sidePadThumb = laneSolved;
            }

            ThumbLaneSolveCandidate candidate{
                .lane = lane.lane,
                .value = laneSolved,
                .normal = normalize(laneCurve.normal),
                .normalBlend = std::clamp(std::isfinite(lane.normalBlend) ? lane.normalBlend : 0.0f, 0.0f, 1.0f),
                .localCorrectionStrength = std::clamp(
                    std::isfinite(lane.localCorrectionStrength) ? lane.localCorrectionStrength : 0.0f, 0.0f, 1.0f),
                .maxCurlAngleRadians = maxCalibratedCurveAngle(laneCurve),
                .valid = true,
            };

            const bool candidatePositive =
                laneSolved.hit && !laneSolved.openedByBehindContact && laneSolved.rawCurveValue > kClosedEpsilon;
            if (candidatePositive &&
                (!bestPositive.valid || laneSolved.rawCurveValue > bestPositive.value.rawCurveValue + kClosedEpsilon ||
                    (std::abs(laneSolved.rawCurveValue - bestPositive.value.rawCurveValue) <= kClosedEpsilon &&
                        candidate.localCorrectionStrength > bestPositive.localCorrectionStrength))) {
                bestPositive = candidate;
            }

            const bool candidateClosedOrMissed =
                !laneSolved.hit || (!laneSolved.openedByBehindContact && laneSolved.rawCurveValue <= kClosedEpsilon);
            if (candidateClosedOrMissed && !closedFallback.valid) {
                closedFallback = candidate;
            }
        }

        if (primaryNeedsAlternate && bestPositive.valid) {
            result.value = bestPositive.value;
            result.selectedThumbCurve = bestPositive.value;
            result.selectedThumbLane = bestPositive.lane;
            result.selectedThumbLaneNormal = bestPositive.normal;
            result.selectedThumbLaneNormalBlend = bestPositive.normalBlend;
            result.selectedThumbLaneLocalCorrectionStrength = bestPositive.localCorrectionStrength;
            result.selectedThumbLaneMaxCurlAngleRadians = bestPositive.maxCurlAngleRadians;
            result.usedAlternateThumbCurve = true;
        } else if (primaryClosedOrMissed && !result.primary.openedByBehindContact && closedFallback.valid) {
            result.value = closedFallback.value;
            result.selectedThumbCurve = closedFallback.value;
            result.selectedThumbLane = closedFallback.lane;
            result.selectedThumbLaneNormal = closedFallback.normal;
            result.selectedThumbLaneNormalBlend = closedFallback.normalBlend;
            result.selectedThumbLaneLocalCorrectionStrength = closedFallback.localCorrectionStrength;
            result.selectedThumbLaneMaxCurlAngleRadians = closedFallback.maxCurlAngleRadians;
            result.usedAlternateThumbCurve = true;
        }

        /*
         * The thumb is "awaiting arrival" only when every lane's whole arc is
         * out of reach; a thumb that can reach the mesh through ANY lane is
         * genuinely closing, not anticipating.
         */
        result.value.outOfReach = !result.value.hit && result.primary.outOfReach && result.alternateThumb.outOfReach &&
            result.sidePadThumb.outOfReach;

        return result;
    }

    template <class Vector>
    inline ThumbAwareFingerCurveCurlValue<Vector> sweepThumbAwareCalibratedFingerCurveCurlValue(
        const std::vector<Triangle<Vector>>& triangles, std::size_t fingerIndex, bool isLeft, bool inPowerArmor,
        const Vector& center, const Vector& primaryNormal, const Vector& alternateThumbNormal,
        const Vector& zeroAngleVector, float fingerLength, float minValue, bool allowAlternateThumbCurve,
        float contactRadiusGameUnits, float maxOpenValue = kMaxOverOpenValue)
    {
        auto solveCurve = [&](const CalibratedFingerCurve<Vector>& curve, float curveMinValue, float curveContactRadius,
                              float curveMaxOpenValue) {
            return sweepCalibratedFingerCurveCurlValue(triangles, curve, curveMinValue, curveContactRadius,
                curveMaxOpenValue);
        };
        return sweepThumbAwareCalibratedFingerCurveCurlValueWithCurveSolver<Vector>(fingerIndex, isLeft, inPowerArmor,
            center, primaryNormal, alternateThumbNormal, zeroAngleVector, fingerLength, minValue,
            allowAlternateThumbCurve, contactRadiusGameUnits, maxOpenValue, solveCurve);
    }

    [[nodiscard]] inline float clampOpenValue(float value)
    {
        return std::clamp(std::isfinite(value) ? value : kMaxFingerOpenValue, 0.0f, kMaxOverOpenValue);
    }

    /*
     * Every finger may carry an over-open value (the sweep's per-call cap is
     * the policy boundary, not this expansion). The proximal/distal biases
     * scale with how CLOSED the finger is, so they vanish past 1.0 and
     * over-open joints hyper-extend uniformly - mirrored by the offline
     * generator's expand_rock_grab_open_value.
     */
    inline std::array<float, 15> expandFingerCurlsToJointValues(const std::array<float, 5>& values)
    {
        std::array<float, 15> joints{};
        for (std::size_t finger = 0; finger < values.size(); ++finger) {
            const float value = clampOpenValue(values[finger]);
            const float closed = kMaxFingerOpenValue - std::min(value, kMaxFingerOpenValue);
            const float proximalOpenBias = (finger == 0) ? 0.15f : 0.25f;
            const float distalCloseBias = (finger == 0) ? 0.10f : 0.15f;
            const std::size_t base = finger * 3;
            joints[base + 0] = std::clamp(value + closed * proximalOpenBias, 0.0f, kMaxOverOpenValue);
            joints[base + 1] = value;
            joints[base + 2] = std::clamp(value - closed * distalCloseBias, 0.0f, kMaxOverOpenValue);
        }
        return joints;
    }

    inline std::array<float, 15> advanceJointValues(const std::array<float, 15>& current,
        const std::array<float, 15>& target, float speed, float deltaTime)
    {
        std::array<float, 15> result{};
        if (!std::isfinite(speed) || speed <= 0.0f) {
            for (std::size_t i = 0; i < result.size(); ++i) {
                result[i] = clampOpenValue(target[i]);
            }
            return result;
        }

        const float dt = (std::isfinite(deltaTime) && deltaTime > 0.0f) ? deltaTime : 0.0f;
        if (dt <= 0.0f) {
            // Unmeasurable frame: hold the current pose instead of advancing
            // by fabricated time.
            for (std::size_t i = 0; i < result.size(); ++i) {
                result[i] = clampOpenValue(current[i]);
            }
            return result;
        }
        const float step = speed * dt;
        for (std::size_t i = 0; i < result.size(); ++i) {
            const float from = clampOpenValue(current[i]);
            const float to = clampOpenValue(target[i]);
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

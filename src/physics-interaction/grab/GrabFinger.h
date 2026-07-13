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

#include "physics-interaction/grab/GeneratedGrabFingerCalibration.h"

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
        grab_finger_calibration_data::BakedGrabThumbLane selectedThumbLane = grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
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

    enum class CalibratedFingerProbe : std::uint8_t
    {
        Tip,
        Outer,
        Inner
    };

    inline constexpr std::size_t kCalibratedFingerCurveSampleCount = 201;
    static_assert(
        grab_finger_calibration_data::kGrabFingerCalibrationSampleCount == kCalibratedFingerCurveSampleCount,
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
        for (std::size_t probeIndex = 0; probeIndex < curve.probeCount && probeIndex < curve.probes.size(); ++probeIndex) {
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
        const grab_finger_calibration_data::BakedGrabFingerProbeCurve& baked,
        float fingerLength)
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
                .openValue = std::clamp(std::isfinite(sample.openValue) ? sample.openValue : 1.0f, 0.0f, kMaxOverOpenValue),
                .angleRadians = std::isfinite(sample.angleRadians) ? sample.angleRadians : 0.0f,
                .reachLength = std::max(0.0001f, (std::isfinite(sample.reachScale) ? sample.reachScale : 1.0f) * fingerLength),
            };
        }
        return curve;
    }

    template <class Vector>
    inline CalibratedFingerCurve<Vector> makeBakedCalibratedFingerCurveFromBaked(
        const grab_finger_calibration_data::BakedGrabFingerCurve& baked,
        const Vector& center,
        const Vector& normal,
        const Vector& zeroAngleVector,
        float fingerLength,
        bool applyBakedNormalSign = true,
        float surfaceThicknessScaleOverride = -1.0f)
    {
        CalibratedFingerCurve<Vector> curve{};
        if (!std::isfinite(fingerLength) || fingerLength <= 0.0001f) {
            return curve;
        }

        const float normalSign = applyBakedNormalSign && baked.normalSign < 0.0f ? -1.0f : 1.0f;
        const float rawThicknessScale =
            std::isfinite(surfaceThicknessScaleOverride) && surfaceThicknessScaleOverride >= 0.0f ?
            surfaceThicknessScaleOverride :
            (std::isfinite(baked.surfaceThicknessScale) ? baked.surfaceThicknessScale : 0.05f);
        const float thicknessScale = std::clamp(
            rawThicknessScale,
            0.0f,
            0.25f);

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
    inline CalibratedFingerCurve<Vector> makeBakedCalibratedFingerCurve(
        std::size_t fingerIndex,
        bool isLeft,
        bool inPowerArmor,
        const Vector& center,
        const Vector& normal,
        const Vector& zeroAngleVector,
        float fingerLength,
        bool applyBakedNormalSign = true)
    {
        CalibratedFingerCurve<Vector> curve{};
        if (fingerIndex >= 5) {
            return curve;
        }

        const auto& profile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        return makeBakedCalibratedFingerCurveFromBaked<Vector>(
            profile.fingers[fingerIndex],
            center,
            normal,
            zeroAngleVector,
            fingerLength,
            applyBakedNormalSign);
    }

    template <class Vector>
    [[nodiscard]] inline Vector blendedThumbLaneNormal(
        const Vector& primaryNormal,
        const Vector& alternateThumbNormal,
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
        const grab_finger_calibration_data::BakedGrabFingerCurve& bakedCurve,
        const Vector& center,
        const Vector& normal,
        const Vector& zeroAngleVector,
        float fingerLength)
    {
        return makeBakedCalibratedFingerCurveFromBaked<Vector>(
            bakedCurve,
            center,
            normal,
            zeroAngleVector,
            fingerLength,
            lane.applyAuthoredNormalSign,
            lane.surfaceThicknessScale);
    }

    [[nodiscard]] inline float maxBakedFingerCurveAngleRadians(const grab_finger_calibration_data::BakedGrabFingerCurve& baked)
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
    [[nodiscard]] inline CalibratedChainCurlEstimate estimateCalibratedChainCurlFromChord(
        std::size_t fingerIndex,
        bool isLeft,
        bool inPowerArmor,
        float fingerLength,
        float liveChordLength)
    {
        CalibratedChainCurlEstimate result{};
        if (fingerIndex >= 5 ||
            !std::isfinite(fingerLength) || fingerLength <= 0.0001f ||
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
                result.chordAngleRadians = samples[i].angleRadians + (samples[i + 1].angleRadians - samples[i].angleRadians) * t;
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

    [[nodiscard]] inline float bakedCalibratedFingerMaxAngleRadians(std::size_t fingerIndex, bool isLeft, bool inPowerArmor)
    {
        if (fingerIndex >= 5) {
            return 0.0f;
        }

        const auto& profile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        return maxBakedFingerCurveAngleRadians(profile.fingers[fingerIndex]);
    }

    [[nodiscard]] inline float bakedCalibratedThumbLaneMaxAngleRadians(
        grab_finger_calibration_data::BakedGrabThumbLane lane,
        bool isLeft,
        bool inPowerArmor)
    {
        const auto& profile = grab_finger_calibration_data::bakedGrabThumbProfile(isLeft, inPowerArmor);
        const auto& handProfile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
        for (const auto& candidate : profile.lanes) {
            if (candidate.lane == lane) {
                const auto& curve = candidate.curveSource == grab_finger_calibration_data::BakedGrabThumbCurveSource::SidePad ?
                    profile.sidePadCurve :
                    handProfile.fingers[0];
                return maxBakedFingerCurveAngleRadians(curve);
            }
        }
        return 0.0f;
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
     * Swept-arc finger curl solver. Simulates the finger's actual closing
     * motion: each baked probe row (Tip/Outer/Inner sampled along the hFRIK
     * curl animation) is reconstructed as a world position on the curl arc,
     * walked from open toward closed, and the finger stops at the first row
     * whose contact sphere touches the mesh — the most-open first contact
     * across the three probes wins, exactly like a real finger closing onto a
     * surface. Volumetric by construction (pad radius), so off-plane geometry
     * the old plane-slice solver could not see stops the finger, and a stop
     * can never leave the finger inside the mesh. A finger whose whole arc is
     * farther than reach reports outOfReach so callers can hold an
     * anticipation pose instead of closing on nothing.
     *
     * maxOpenValue caps how far the "open end" of the walk starts: rows with
     * a larger openValue (over-open/hyper-extension rows baked past 1.0) are
     * skipped entirely. At 1.0 this is the classic authored-open sweep; above
     * it the finger may stop over-open when the mesh interpenetrates the
     * default open pose (objects thicker than the open finger span).
     */
    template <class Vector>
    inline FingerCurlValue sweepCalibratedFingerCurveCurlValue(
        const std::vector<Triangle<Vector>>& triangles,
        const CalibratedFingerCurve<Vector>& curve,
        float minValue,
        float contactRadiusGameUnits,
        float maxOpenValue = kMaxFingerOpenValue)
    {
        FingerCurlValue result{};
        const float clampedMin = std::clamp(minValue, 0.0f, 1.0f);
        const float clampedMaxOpen = std::clamp(
            std::isfinite(maxOpenValue) ? maxOpenValue : kMaxFingerOpenValue, kMaxFingerOpenValue, kMaxOverOpenValue);
        result.value = clampedMin;
        if (triangles.empty() || curve.probeCount == 0) {
            result.outOfReach = true;
            return result;
        }

        const Vector planeNormal = normalize(curve.normal);
        const Vector zero = normalize(curve.zeroAngleVector);
        const float radius = std::clamp(std::isfinite(contactRadiusGameUnits) ? contactRadiusGameUnits : 1.0f, 0.05f, 4.0f);
        const float maxReach = maxCalibratedCurveReach(curve);
        if (!std::isfinite(maxReach) || maxReach <= 0.0001f) {
            result.outOfReach = true;
            return result;
        }

        const float filterRadius = maxReach + radius + 0.5f;
        const auto nearTriangles = filterTrianglesNearPoint(triangles, curve.center, filterRadius * filterRadius);
        if (nearTriangles.empty()) {
            result.outOfReach = true;
            return result;
        }

        constexpr std::size_t kCoarseRowStep = 4;
        float bestOpenValue = -1.0f;
        float bestAngle = 0.0f;
        Vector bestHitPoint{};
        Vector bestHitNormal{};
        bool bestHitPointValid = false;
        bool bestHitNormalValid = false;

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

        for (std::size_t probeIndex = 0; probeIndex < curve.probeCount && probeIndex < curve.probes.size(); ++probeIndex) {
            const auto& probe = curve.probes[probeIndex];
            if (probe.sampleCount == 0 || probe.sampleCount > probe.samples.size()) {
                continue;
            }

            /*
             * Reconstruct the probe's arc path once; consecutive rows sit far
             * closer together than the contact radius, so sphere tests at the
             * rows cover the swept path with no tunneling gap. The coarse pass
             * inflates the radius by the largest row-to-row gap to stay
             * conservative over skipped rows, then the bracket is re-tested at
             * the exact radius.
             */
            /*
             * Rows are ordered most-open first; over-open rows above the cap
             * are skipped so the walk starts at the most-open ALLOWED pose.
             */
            std::size_t startRow = 0;
            while (startRow < probe.sampleCount && probe.samples[startRow].openValue > clampedMaxOpen + 0.0001f) {
                ++startRow;
            }
            if (startRow >= probe.sampleCount) {
                continue;
            }

            std::array<Vector, kCalibratedFingerCurveSampleCount> rowPositions{};
            float maxRowGap = 0.0f;
            for (std::size_t i = startRow; i < probe.sampleCount; ++i) {
                const auto& sample = probe.samples[i];
                const Vector arm = rotateAroundUnitAxis(zero, planeNormal, sample.angleRadians);
                rowPositions[i] = add(curve.center, scale(arm, sample.reachLength));
                if (i > startRow) {
                    maxRowGap = (std::max)(maxRowGap, length(sub(rowPositions[i], rowPositions[i - 1])));
                }
            }
            const float coarseRadius = radius + maxRowGap * static_cast<float>(kCoarseRowStep);

            bool probeContact = false;
            for (std::size_t coarse = startRow; coarse < probe.sampleCount && !probeContact; coarse += kCoarseRowStep) {
                if (!sphereContact(rowPositions[coarse], coarseRadius, nullptr, nullptr)) {
                    continue;
                }
                const std::size_t bracketBegin = coarse >= startRow + kCoarseRowStep ? coarse - kCoarseRowStep : startRow;
                const std::size_t bracketEnd = (std::min)(coarse + kCoarseRowStep, probe.sampleCount - 1);
                for (std::size_t row = bracketBegin; row <= bracketEnd; ++row) {
                    Vector hitPoint{};
                    Vector hitNormal{};
                    if (!sphereContact(rowPositions[row], radius, &hitPoint, &hitNormal)) {
                        continue;
                    }
                    const float openValue = std::clamp(probe.samples[row].openValue, 0.0f, clampedMaxOpen);
                    if (openValue > bestOpenValue + 0.0001f) {
                        bestOpenValue = openValue;
                        bestAngle = probe.samples[row].angleRadians;
                        bestHitPoint = hitPoint;
                        bestHitNormal = hitNormal;
                        bestHitPointValid = true;
                        bestHitNormalValid = hasUsableDirection(hitNormal);
                    }
                    probeContact = true;
                    break;
                }
            }
        }

        if (bestOpenValue >= 0.0f) {
            result.hit = true;
            result.hitKind = FingerCurlValue::HitKind::FrontValid;
            result.distance = bestAngle;
            result.rawCurveValue = bestOpenValue;
            result.value = std::clamp(bestOpenValue, clampedMin, clampedMaxOpen);
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
        }
        return result;
    }

    /*
     * Thumb-aware wrapper over the swept-arc solver. Lane bookkeeping (wrap
     * first, then opposition/side-pad candidates with their normal blends and
     * local correction strengths) intentionally mirrors the retired
     * plane-slice selector so the downstream local-transform machinery keeps
     * its contract; only the per-lane curl solve changed.
     */
    template <class Vector>
    inline ThumbAwareFingerCurveCurlValue<Vector> sweepThumbAwareCalibratedFingerCurveCurlValue(
        const std::vector<Triangle<Vector>>& triangles,
        std::size_t fingerIndex,
        bool isLeft,
        bool inPowerArmor,
        const Vector& center,
        const Vector& primaryNormal,
        const Vector& alternateThumbNormal,
        const Vector& zeroAngleVector,
        float fingerLength,
        float minValue,
        bool allowAlternateThumbCurve,
        float contactRadiusGameUnits,
        float maxOpenValue = kMaxFingerOpenValue)
    {
        ThumbAwareFingerCurveCurlValue<Vector> result{};
        const auto primaryCurve = makeBakedCalibratedFingerCurve(
            fingerIndex, isLeft, inPowerArmor, center, primaryNormal, zeroAngleVector, fingerLength);
        result.primary = sweepCalibratedFingerCurveCurlValue(triangles, primaryCurve, minValue, contactRadiusGameUnits, maxOpenValue);
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
            grab_finger_calibration_data::BakedGrabThumbLane lane = grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
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

            const auto& bakedLaneCurve = lane.curveSource == grab_finger_calibration_data::BakedGrabThumbCurveSource::SidePad ?
                thumbProfile.sidePadCurve :
                thumbHandProfile.fingers[0];
            const Vector laneNormal = blendedThumbLaneNormal(primaryNormal, alternateThumbNormal, lane.normalBlend);
            const auto laneCurve = makeBakedCalibratedThumbLaneCurve<Vector>(
                lane,
                bakedLaneCurve,
                center,
                laneNormal,
                zeroAngleVector,
                fingerLength);
            const FingerCurlValue laneSolved = sweepCalibratedFingerCurveCurlValue(triangles, laneCurve, minValue, contactRadiusGameUnits, maxOpenValue);

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
                .localCorrectionStrength = std::clamp(std::isfinite(lane.localCorrectionStrength) ? lane.localCorrectionStrength : 0.0f, 0.0f, 1.0f),
                .maxCurlAngleRadians = maxCalibratedCurveAngle(laneCurve),
                .valid = true,
            };

            const bool candidatePositive =
                laneSolved.hit && !laneSolved.openedByBehindContact && laneSolved.rawCurveValue > kClosedEpsilon;
            if (candidatePositive &&
                (!bestPositive.valid ||
                    laneSolved.rawCurveValue > bestPositive.value.rawCurveValue + kClosedEpsilon ||
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
        result.value.outOfReach = !result.value.hit &&
                                  result.primary.outOfReach &&
                                  result.alternateThumb.outOfReach &&
                                  result.sidePadThumb.outOfReach;

        return result;
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

    inline std::array<float, 15> advanceJointValues(const std::array<float, 15>& current, const std::array<float, 15>& target, float speed, float deltaTime)
    {
        std::array<float, 15> result{};
        if (!std::isfinite(speed) || speed <= 0.0f) {
            for (std::size_t i = 0; i < result.size(); ++i) {
                result[i] = clampOpenValue(target[i]);
            }
            return result;
        }

        const float dt = (std::isfinite(deltaTime) && deltaTime > 0.0f) ? deltaTime : (1.0f / 90.0f);
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
    constexpr std::size_t kMaxFingerPoseCandidateTriangles = 2048;

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
        /*
         * Signed in-plane rotation (about the unsigned palm curl normal, baked
         * normal-sign already applied) from the arc zero to the contact row
         * each finger was adopted at. Valid only for swept front contacts on
         * the palm-plane arc (thumb: Wrap lane only - alternate lanes solve in
         * a different plane). Held re-solves feed these back as
         * GrabFingerArcAnchorHints so the anchor de-rotation uses the KNOWN
         * commanded rotation instead of re-estimating it from the live chord.
         */
        std::array<float, 5> contactArcRotationRadians{};
        std::array<std::uint8_t, 5> contactArcRotationValid{};
        int hitCount = 0;
        int candidateTriangleCount = 0;
        int poseTargetCount = 0;
        bool solved = false;
        bool hasJointValues = false;
        bool hasObjectLocalSurfaceAim = false;
        bool usedAlternateThumbCurve = false;
        bool usedAlternateThumbSurfaceHit = false;
        bool thumbSurfaceFollowAllowed = true;
        grab_finger_calibration_data::BakedGrabThumbLane selectedThumbLane = grab_finger_calibration_data::BakedGrabThumbLane::Wrap;
        float selectedThumbLaneNormalBlend = 0.0f;
        float selectedThumbLaneLocalCorrectionStrength = 0.0f;
        bool usedLiveRootFlattenedFingerBones = false;
        bool hasThumbAlternateCurveFrame = false;
        RE::NiPoint3 thumbAlternateCurveBaseWorld{};
        RE::NiPoint3 thumbAlternateCurveOpenDirectionWorld{};
        RE::NiPoint3 thumbAlternateCurveNormalWorld{};
        float thumbAlternateCurveMaxCurlAngleRadians = 0.0f;
        bool hasThumbCurveDiagnostics = false;
        grab_finger_pose_math::FingerCurlValue thumbPrimaryCurve{};
        grab_finger_pose_math::FingerCurlValue thumbAlternateCurve{};
        grab_finger_pose_math::FingerCurlValue thumbSidePadCurve{};
    };

    /*
     * Known per-finger arc rotations from the pose currently DRIVING the
     * hand (the last adopted solve). A held re-solve de-rotates each live
     * chord by its known rotation - exact once the pose smoothing has
     * settled, unambiguous for over-open poses, and valid for the thumb
     * (whose chord-length inversion is spurious under opposition/twist).
     */
    struct GrabFingerArcAnchorHints
    {
        std::array<float, 5> rotationRadians{};
        std::array<std::uint8_t, 5> valid{};
    };

    [[nodiscard]] inline GrabFingerArcAnchorHints makeArcAnchorHintsFromPose(const SolvedGrabFingerPose& pose)
    {
        GrabFingerArcAnchorHints hints{};
        hints.rotationRadians = pose.contactArcRotationRadians;
        hints.valid = pose.contactArcRotationValid;
        return hints;
    }

    /*
     * Held re-solve rule: a thumb WITHOUT an exact arc-anchor hint (its last
     * adoption was an alternate lane, a miss, or a floored value) must keep
     * its previous pose instead of re-solving from the raw live chord. The
     * raw-chord anchor rotates with the thumb's own curl (opposition/twist,
     * out of the palm plane), so an unhinted re-solve maps pose A to pose B
     * and pose B back to pose A - a period-2 open/close cycle the adoption
     * deadband cannot break. The previous pose (capture, or the last hinted
     * adoption) is authoritative until a hinted re-solve exists. Surface-aim
     * entries are not carried: the thumb's are cleared at adoption by
     * useThumbIndexCurveOnlyPose anyway.
     */
    inline void keepThumbPoseFromPrevious(SolvedGrabFingerPose& pose, const SolvedGrabFingerPose& previous)
    {
        pose.values[0] = previous.values[0];
        for (std::size_t segment = 0; segment < 3; ++segment) {
            pose.jointValues[segment] = previous.jointValues[segment];
        }
        pose.hitKind[0] = previous.hitKind[0];
        pose.contactArcRotationRadians[0] = previous.contactArcRotationRadians[0];
        pose.contactArcRotationValid[0] = previous.contactArcRotationValid[0];
        pose.thumbSurfaceFollowAllowed = previous.thumbSurfaceFollowAllowed;
        pose.usedAlternateThumbCurve = previous.usedAlternateThumbCurve;
        pose.usedAlternateThumbSurfaceHit = previous.usedAlternateThumbSurfaceHit;
        pose.selectedThumbLane = previous.selectedThumbLane;
        pose.selectedThumbLaneNormalBlend = previous.selectedThumbLaneNormalBlend;
        pose.selectedThumbLaneLocalCorrectionStrength = previous.selectedThumbLaneLocalCorrectionStrength;
        pose.hasThumbAlternateCurveFrame = previous.hasThumbAlternateCurveFrame;
        pose.thumbAlternateCurveBaseWorld = previous.thumbAlternateCurveBaseWorld;
        pose.thumbAlternateCurveOpenDirectionWorld = previous.thumbAlternateCurveOpenDirectionWorld;
        pose.thumbAlternateCurveNormalWorld = previous.thumbAlternateCurveNormalWorld;
        pose.thumbAlternateCurveMaxCurlAngleRadians = previous.thumbAlternateCurveMaxCurlAngleRadians;
        pose.hasThumbCurveDiagnostics = previous.hasThumbCurveDiagnostics;
        pose.thumbPrimaryCurve = previous.thumbPrimaryCurve;
        pose.thumbAlternateCurve = previous.thumbAlternateCurve;
        pose.thumbSidePadCurve = previous.thumbSidePadCurve;
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

    struct FingerPadSurfaceEvidence
    {
        RE::NiPoint3 startWorld{};
        RE::NiPoint3 endWorld{};
        RE::NiPoint3 hitPointWorld{};
        RE::NiPoint3 hitNormalWorld{};
        float distanceGameUnits = 0.0f;
        float quality = 0.0f;
        bool hit = false;
        bool fromClosestSurface = false;
        bool padMayBeInsideSurface = false;
    };

    struct FingerPadProbeOptions
    {
        float probeRadiusGameUnits = 1.0f;
        float probeDistanceGameUnits = 6.0f;
        float closestSurfaceMaxDistanceGameUnits = 4.0f;
        float targetOverrideMinQuality = 0.5f;
    };

    inline FingerPadProbeOptions sanitizeFingerPadProbeOptions(FingerPadProbeOptions options)
    {
        if (!std::isfinite(options.probeRadiusGameUnits) || options.probeRadiusGameUnits <= 0.0f) {
            options.probeRadiusGameUnits = 1.0f;
        }
        if (!std::isfinite(options.probeDistanceGameUnits) || options.probeDistanceGameUnits <= 0.0f) {
            options.probeDistanceGameUnits = 6.0f;
        }
        if (!std::isfinite(options.closestSurfaceMaxDistanceGameUnits) || options.closestSurfaceMaxDistanceGameUnits <= 0.0f) {
            options.closestSurfaceMaxDistanceGameUnits = 4.0f;
        }

        options.probeRadiusGameUnits = std::clamp(options.probeRadiusGameUnits, 0.05f, 8.0f);
        options.probeDistanceGameUnits = std::clamp(options.probeDistanceGameUnits, 0.5f, 32.0f);
        options.closestSurfaceMaxDistanceGameUnits = std::clamp(options.closestSurfaceMaxDistanceGameUnits, 0.5f, 24.0f);
        options.targetOverrideMinQuality = std::isfinite(options.targetOverrideMinQuality) ? std::clamp(options.targetOverrideMinQuality, 0.0f, 1.0f) : 0.5f;
        return options;
    }

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

    inline void useThumbIndexCurveOnlyPose(SolvedGrabFingerPose& pose)
    {
        pose.thumbSurfaceFollowAllowed = false;
        pose.usedAlternateThumbSurfaceHit = false;
        for (std::size_t finger = 0; finger < 2 && finger < pose.surfaceAimTargetValid.size(); ++finger) {
            pose.surfaceAimTargetValid[finger] = 0;
            pose.surfaceAimNormalValid[finger] = 0;
            pose.surfaceAimTargetObjectLocalValid[finger] = 0;
            pose.surfaceAimNormalObjectLocalValid[finger] = 0;
            pose.surfaceAimTarget[finger] = {};
            pose.surfaceAimNormal[finger] = {};
            pose.surfaceAimTargetObjectLocal[finger] = {};
            pose.surfaceAimNormalObjectLocal[finger] = {};
        }
        pose.hasObjectLocalSurfaceAim = false;
        for (std::size_t finger = 2; finger < pose.surfaceAimTargetObjectLocalValid.size(); ++finger) {
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
        if (!hasExplicitFingerTarget) {
            return 0.3f;
        }
        if (hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface ||
            hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::Rejected) {
            return 1.0f;
        }
        return std::clamp(minValue, 0.0f, 1.0f);
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

    inline bool isFinitePoint(const RE::NiPoint3& point)
    {
        return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
    }

    inline float triangleDistanceSquaredToPoint(const TriangleData& triangle, const RE::NiPoint3& point)
    {
        const RE::NiPoint3 centroid = (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
        return (std::min)({
            distanceSquared(centroid, point),
            distanceSquared(triangle.v0, point),
            distanceSquared(triangle.v1, point),
            distanceSquared(triangle.v2, point),
        });
    }

    inline void appendTriangleCandidate(const TriangleData& triangle, std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
            return;
        }
        outTriangles.push_back({ triangle.v0, triangle.v1, triangle.v2 });
    }

    inline bool isFiniteTransformForFingerPadProbe(const RE::NiTransform& transform)
    {
        if (!isFinitePoint(transform.translate) || !std::isfinite(transform.scale) || std::abs(transform.scale) <= 0.000001f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    inline bool shouldRunFingerPadProbeRefinement(
        bool meshFingerPoseEnabled,
        bool grabFingerPosePublished,
        bool hasObjectTransform,
        bool hasLiveFingerSnapshot,
        bool hasTriangles)
    {
        return meshFingerPoseEnabled && grabFingerPosePublished && hasObjectTransform && hasLiveFingerSnapshot && hasTriangles;
    }

    inline bool computeFingerPadCenter(
        const root_flattened_finger_skeleton_runtime::FingerChain& chain,
        RE::NiPoint3& outPadCenterWorld)
    {
        if (!chain.valid || !isFinitePoint(chain.points[2])) {
            outPadCenterWorld = {};
            return false;
        }

        outPadCenterWorld = chain.points[2];
        return isFinitePoint(outPadCenterWorld);
    }

    inline bool computeOpposedFingerPadProbeDirection(
        const root_flattened_finger_skeleton_runtime::Snapshot& liveFingerSnapshot,
        const root_flattened_finger_skeleton_runtime::FingerLandmark& liveFinger,
        std::size_t finger,
        const RE::NiPoint3& padCenterWorld,
        RE::NiPoint3& outProbeDirection)
    {
        outProbeDirection = {};
        if (!liveFingerSnapshot.valid ||
            !liveFingerSnapshot.fingers[0].valid ||
            !liveFingerSnapshot.fingers[1].valid ||
            !isFinitePoint(padCenterWorld)) {
            return false;
        }

        const auto& aimChain = finger == 0 ? liveFingerSnapshot.fingers[1] : liveFingerSnapshot.fingers[0];
        if (!aimChain.valid || !isFinitePoint(aimChain.points[2])) {
            return false;
        }

        const RE::NiPoint3 toOpposedFinger = aimChain.points[2] - padCenterWorld;
        if (distanceSquared(toOpposedFinger, RE::NiPoint3{}) <= 0.000001f) {
            return false;
        }

        const RE::NiPoint3 fallback = liveFinger.valid ? liveFinger.openDirection : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        outProbeDirection = normalizedOrFallback(toOpposedFinger, fallback);
        return isFinitePoint(outProbeDirection) && distanceSquared(outProbeDirection, RE::NiPoint3{}) > 0.000001f;
    }

    inline RE::NiPoint3 fingerPadProbeDirection(
        const SolvedGrabFingerPose& pose,
        const GrabFingerPoseTargetSet& poseTargets,
        const root_flattened_finger_skeleton_runtime::FingerLandmark& liveFinger,
        std::size_t finger,
        const RE::NiPoint3& padCenterWorld)
    {
        RE::NiPoint3 targetWorld{};
        bool hasTarget = false;
        if (finger < pose.surfaceAimTargetValid.size() && pose.surfaceAimTargetValid[finger]) {
            targetWorld = pose.surfaceAimTarget[finger];
            hasTarget = true;
        } else if (finger < poseTargets.targetValid.size() && poseTargets.targetValid[finger]) {
            targetWorld = poseTargets.targets[finger];
            hasTarget = true;
        } else if (poseTargets.seatPointValid) {
            targetWorld = poseTargets.seatPointWorld;
            hasTarget = true;
        }

        const RE::NiPoint3 fallback = liveFinger.valid ? liveFinger.openDirection : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        if (!hasTarget || !isFinitePoint(targetWorld)) {
            return normalizedOrFallback(fallback, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        }

        return normalizedOrFallback(targetWorld - padCenterWorld, fallback);
    }

    inline RE::NiPoint3 flippedNormal(const RE::NiPoint3& normal)
    {
        return RE::NiPoint3{ -normal.x, -normal.y, -normal.z };
    }

    inline RE::NiPoint3 orientSurfaceNormalTowardPoint(
        const RE::NiPoint3& rawNormal,
        const RE::NiPoint3& surfacePoint,
        const RE::NiPoint3& pointWorld)
    {
        RE::NiPoint3 normal = normalizedOrFallback(rawNormal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 toPoint = pointWorld - surfacePoint;
        if (distanceSquared(toPoint, RE::NiPoint3{}) > 0.000001f && dotPoint(normal, toPoint) < 0.0f) {
            normal = flippedNormal(normal);
        }
        return normal;
    }

    inline RE::NiPoint3 orientSurfaceNormalAgainstProbe(
        const RE::NiPoint3& rawNormal,
        const RE::NiPoint3& probeDirectionWorld)
    {
        RE::NiPoint3 normal = normalizedOrFallback(rawNormal, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        if (dotPoint(normal, probeDirectionWorld) > 0.0f) {
            normal = flippedNormal(normal);
        }
        return normal;
    }

    inline bool padSurfaceNormalCompatible(
        const RE::NiPoint3& candidateNormalWorld,
        const RE::NiPoint3& preferredNormalWorld,
        bool hasPreferredNormal)
    {
        if (!hasPreferredNormal || distanceSquared(preferredNormalWorld, RE::NiPoint3{}) <= 0.000001f) {
            return true;
        }

        const RE::NiPoint3 candidate = normalizedOrFallback(candidateNormalWorld, RE::NiPoint3{ 0.0f, 0.0f, 1.0f });
        const RE::NiPoint3 preferred = normalizedOrFallback(preferredNormalWorld, candidate);
        return dotPoint(candidate, preferred) >= -0.55f;
    }

    inline bool hasFingerPadProbeLine(const FingerPadSurfaceEvidence& evidence)
    {
        return isFinitePoint(evidence.startWorld) && isFinitePoint(evidence.endWorld) &&
               distanceSquared(evidence.startWorld, evidence.endWorld) > 0.000001f;
    }

    inline FingerPadSurfaceEvidence solveFingerPadSurfaceEvidenceFromTriangles(
        const std::vector<TriangleData>& triangles,
        const RE::NiPoint3& padCenterWorld,
        const RE::NiPoint3& probeDirectionWorld,
        const RE::NiPoint3& preferredSurfaceNormalWorld,
        bool hasPreferredSurfaceNormal,
        FingerPadProbeOptions options = {})
    {
        options = sanitizeFingerPadProbeOptions(options);
        FingerPadSurfaceEvidence result{};
        const RE::NiPoint3 probeDirection = normalizedOrFallback(probeDirectionWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
        result.startWorld = padCenterWorld;
        result.endWorld = padCenterWorld + probeDirection * options.probeDistanceGameUnits;
        if (triangles.empty() || !isFinitePoint(padCenterWorld) || !isFinitePoint(probeDirection)) {
            return result;
        }

        FingerPadSurfaceEvidence bestClosest{};
        FingerPadSurfaceEvidence bestProbe{};
        bool bestClosestValid = false;
        bool bestProbeValid = false;
        float bestClosestDistanceSquared = options.closestSurfaceMaxDistanceGameUnits * options.closestSurfaceMaxDistanceGameUnits;
        float bestProbeDistance = options.probeDistanceGameUnits;

        auto considerClosest = [&](const RE::NiPoint3& point, const RE::NiPoint3& normal, float distanceSq) {
            if (!std::isfinite(distanceSq) || distanceSq > bestClosestDistanceSquared || !isFinitePoint(point) || !isFinitePoint(normal)) {
                return;
            }
            if (!padSurfaceNormalCompatible(normal, preferredSurfaceNormalWorld, hasPreferredSurfaceNormal)) {
                return;
            }

            const float distance = std::sqrt(std::max(0.0f, distanceSq));
            const RE::NiPoint3 toSurface = point - padCenterWorld;
            if (distance > options.probeRadiusGameUnits && dotPoint(toSurface, probeDirection) < -options.probeRadiusGameUnits) {
                return;
            }

            bestClosestDistanceSquared = distanceSq;
            bestClosest.startWorld = result.startWorld;
            bestClosest.endWorld = result.endWorld;
            bestClosest.hitPointWorld = point;
            bestClosest.hitNormalWorld = normal;
            bestClosest.distanceGameUnits = distance;
            bestClosest.quality = std::clamp(1.0f - (distance / options.closestSurfaceMaxDistanceGameUnits), 0.0f, 1.0f);
            bestClosest.hit = true;
            bestClosest.fromClosestSurface = true;
            bestClosest.padMayBeInsideSurface = distance <= options.probeRadiusGameUnits;
            bestClosestValid = true;
        };

        auto considerProbe = [&](const RE::NiPoint3& point, const RE::NiPoint3& normal, float travelDistance) {
            if (!std::isfinite(travelDistance) || travelDistance > bestProbeDistance || !isFinitePoint(point) || !isFinitePoint(normal)) {
                return;
            }
            if (!padSurfaceNormalCompatible(normal, preferredSurfaceNormalWorld, hasPreferredSurfaceNormal)) {
                return;
            }

            bestProbeDistance = travelDistance;
            const float pointDistance = std::sqrt(std::max(0.0f, distanceSquared(point, padCenterWorld)));
            bestProbe.startWorld = result.startWorld;
            bestProbe.endWorld = result.endWorld;
            bestProbe.hitPointWorld = point;
            bestProbe.hitNormalWorld = normal;
            bestProbe.distanceGameUnits = pointDistance;
            bestProbe.quality = std::clamp(1.0f - (travelDistance / options.probeDistanceGameUnits), 0.0f, 1.0f);
            bestProbe.hit = true;
            bestProbe.fromClosestSurface = false;
            bestProbe.padMayBeInsideSurface = pointDistance <= options.probeRadiusGameUnits || travelDistance <= options.probeRadiusGameUnits;
            bestProbeValid = true;
        };

        for (const auto& triangleData : triangles) {
            if (!isFinitePoint(triangleData.v0) || !isFinitePoint(triangleData.v1) || !isFinitePoint(triangleData.v2)) {
                continue;
            }

            const grab_finger_pose_math::Triangle<RE::NiPoint3> triangle{ triangleData.v0, triangleData.v1, triangleData.v2 };
            const RE::NiPoint3 rawNormal = grab_finger_pose_math::triangleNormal(triangle);
            if (distanceSquared(rawNormal, RE::NiPoint3{}) <= 0.000001f) {
                continue;
            }

            const RE::NiPoint3 closestPoint = grab_finger_pose_math::closestPointOnTriangle(padCenterWorld, triangle);
            const RE::NiPoint3 closestNormal = orientSurfaceNormalTowardPoint(rawNormal, closestPoint, padCenterWorld);
            considerClosest(closestPoint, closestNormal, distanceSquared(closestPoint, padCenterWorld));

            float travelDistance = 0.0f;
            RE::NiPoint3 probeHitPoint{};
            bool hasProbeHitPoint = false;
            if (grab_finger_pose_math::rayTriangleIntersection(padCenterWorld, probeDirection, triangle, options.probeDistanceGameUnits, travelDistance)) {
                probeHitPoint = padCenterWorld + probeDirection * travelDistance;
                hasProbeHitPoint = true;
            } else if (grab_finger_pose_math::probeCapsuleTriangleIntersection(
                           padCenterWorld,
                           probeDirection,
                           triangle,
                           options.probeDistanceGameUnits,
                           options.probeRadiusGameUnits,
                           travelDistance,
                           &probeHitPoint)) {
                hasProbeHitPoint = true;
            }
            if (hasProbeHitPoint) {
                considerProbe(probeHitPoint, orientSurfaceNormalAgainstProbe(rawNormal, probeDirection), travelDistance);
            }
        }

        if (bestClosestValid && (!bestProbeValid || bestClosest.padMayBeInsideSurface || bestClosest.quality >= bestProbe.quality)) {
            return bestClosest;
        }
        if (bestProbeValid) {
            return bestProbe;
        }
        return result;
    }

    /*
     * The proximity-scaled pad "open bias" (and its thumb over-open variant)
     * was removed deliberately: it mutated PUBLISHED values from live
     * pad-to-surface distance AFTER the held-re-solve deadband, forming a
     * publish->pad-moves->bias-changes feedback loop with no hysteresis -
     * the in-game finger twitch. Over-open is now a first-class swept-arc
     * result (baked rows past 1.0, capped per call), deterministic against
     * the mesh instead of reactive to the skeleton. Do not reintroduce a
     * live-skeleton-driven value mutation on the publish path.
     */
    inline bool shouldAcceptFingerPadTarget(
        const SolvedGrabFingerPose& pose,
        const GrabFingerPoseTargetSet& poseTargets,
        const FingerPadSurfaceEvidence& evidence,
        std::size_t finger,
        FingerPadProbeOptions options)
    {
        options = sanitizeFingerPadProbeOptions(options);
        if (!evidence.hit || evidence.quality < options.targetOverrideMinQuality || !isFinitePoint(evidence.hitPointWorld)) {
            return false;
        }
        if (!pose.thumbSurfaceFollowAllowed && finger < 2) {
            return false;
        }

        const bool existingTarget = finger < pose.surfaceAimTargetValid.size() && pose.surfaceAimTargetValid[finger] != 0;
        const bool explicitFingerTarget = finger < poseTargets.targetValid.size() && poseTargets.targetValid[finger] != 0;
        if (!existingTarget) {
            return true;
        }

        const float existingDistance = std::sqrt(std::max(0.0f, distanceSquared(pose.surfaceAimTarget[finger], evidence.startWorld)));
        if (evidence.padMayBeInsideSurface) {
            return true;
        }
        if (explicitFingerTarget) {
            return false;
        }

        constexpr float kReplacementSlopGameUnits = 0.25f;
        return evidence.distanceGameUnits + kReplacementSlopGameUnits < existingDistance;
    }

    inline bool refineGrabFingerPoseWithPadProbes(
        SolvedGrabFingerPose& pose,
        const std::vector<TriangleData>& worldTriangles,
        const GrabFingerPoseTargetSet& poseTargets,
        const root_flattened_finger_skeleton_runtime::Snapshot& liveFingerSnapshot,
        const RE::NiTransform& objectWorldTransform,
        bool meshFingerPoseEnabled,
        bool grabFingerPosePublished,
        std::array<FingerPadSurfaceEvidence, 5>& outEvidence,
        bool allowSurfaceTargetRefinement = true,
        FingerPadProbeOptions options = {})
    {
        options = sanitizeFingerPadProbeOptions(options);
        outEvidence = {};
        const bool liveSnapshotValid = liveFingerSnapshot.valid;
        const bool shouldRun = shouldRunFingerPadProbeRefinement(
            meshFingerPoseEnabled,
            grabFingerPosePublished,
            isFiniteTransformForFingerPadProbe(objectWorldTransform),
            liveSnapshotValid,
            !worldTriangles.empty());
        if (!shouldRun || !pose.solved) {
            return false;
        }

        const auto liveLandmarks = root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
        if (!liveLandmarks.valid) {
            return false;
        }

        bool anyProbeLine = false;
        for (std::size_t finger = 0; finger < outEvidence.size(); ++finger) {
            RE::NiPoint3 padCenterWorld{};
            if (!computeFingerPadCenter(liveFingerSnapshot.fingers[finger], padCenterWorld)) {
                continue;
            }

            RE::NiPoint3 probeDirection{};
            if (!computeOpposedFingerPadProbeDirection(liveFingerSnapshot, liveLandmarks.fingers[finger], finger, padCenterWorld, probeDirection)) {
                probeDirection = fingerPadProbeDirection(pose, poseTargets, liveLandmarks.fingers[finger], finger, padCenterWorld);
            }
            const bool hasPreferredNormal = finger < pose.surfaceAimNormalValid.size() && pose.surfaceAimNormalValid[finger] != 0;
            const RE::NiPoint3 preferredNormal = hasPreferredNormal ? pose.surfaceAimNormal[finger] : RE::NiPoint3{};
            FingerPadSurfaceEvidence evidence = solveFingerPadSurfaceEvidenceFromTriangles(
                worldTriangles,
                padCenterWorld,
                probeDirection,
                preferredNormal,
                hasPreferredNormal,
                options);
            outEvidence[finger] = evidence;
            anyProbeLine = anyProbeLine || hasFingerPadProbeLine(evidence);
            if (!evidence.hit) {
                continue;
            }

            if (allowSurfaceTargetRefinement && shouldAcceptFingerPadTarget(pose, poseTargets, evidence, finger, options)) {
                pose.surfaceAimTarget[finger] = evidence.hitPointWorld;
                pose.surfaceAimTargetValid[finger] = 1;
                if (distanceSquared(evidence.hitNormalWorld, RE::NiPoint3{}) > 0.000001f) {
                    pose.surfaceAimNormal[finger] = normalizedOrFallback(evidence.hitNormalWorld, liveLandmarks.palmNormalWorld);
                    pose.surfaceAimNormalValid[finger] = 1;
                }
            }
        }

        return anyProbeLine;
    }

    // Red surface targets are contact hints only. Side-to-side splay must stay anatomically bounded.
    inline constexpr float kMaxSurfaceContactSplayRadians = 0.08726646259971647f;
    inline constexpr float kDefaultSurfaceContactSplayMaxRadians = kMaxSurfaceContactSplayRadians;

    inline bool hasSurfaceContactSplayCandidates(const SolvedGrabFingerPose& pose)
    {
        for (const auto targetValid : pose.surfaceAimTargetValid) {
            if (targetValid != 0) {
                return true;
            }
        }
        return false;
    }

    inline RE::NiPoint3 rejectFromDirection(const RE::NiPoint3& value, const RE::NiPoint3& direction)
    {
        return RE::NiPoint3{
            value.x - direction.x * dotPoint(value, direction),
            value.y - direction.y * dotPoint(value, direction),
            value.z - direction.z * dotPoint(value, direction),
        };
    }

    inline float signedPalmPlaneSplayRadians(
        const RE::NiPoint3& openDirectionWorld,
        const RE::NiPoint3& targetDirectionWorld,
        const RE::NiPoint3& palmNormalWorld)
    {
        const RE::NiPoint3 palmNormal = normalizedOrFallback(palmNormalWorld, RE::NiPoint3{ 0.0f, 0.0f, -1.0f });
        const RE::NiPoint3 openPlanar = rejectFromDirection(openDirectionWorld, palmNormal);
        const RE::NiPoint3 targetPlanar = rejectFromDirection(targetDirectionWorld, palmNormal);

        if (dotPoint(openPlanar, openPlanar) <= 0.000001f || dotPoint(targetPlanar, targetPlanar) <= 0.000001f) {
            return 0.0f;
        }

        const RE::NiPoint3 open = normalizedOrFallback(openPlanar, openDirectionWorld);
        const RE::NiPoint3 target = normalizedOrFallback(targetPlanar, open);
        const float sinAngle = std::clamp(dotPoint(palmNormal, crossPoint(open, target)), -1.0f, 1.0f);
        const float cosAngle = std::clamp(dotPoint(open, target), -1.0f, 1.0f);
        const float angle = std::atan2(sinAngle, cosAngle);
        return std::isfinite(angle) ? angle : 0.0f;
    }

    inline float clampSurfaceContactSplayRadians(float splayRadians, float maxSplayRadians = kDefaultSurfaceContactSplayMaxRadians)
    {
        const float requestedMax = std::isfinite(maxSplayRadians) ? std::max(0.0f, maxSplayRadians) : kDefaultSurfaceContactSplayMaxRadians;
        const float resolvedMax = std::min(requestedMax, kMaxSurfaceContactSplayRadians);
        if (!std::isfinite(splayRadians) || resolvedMax <= 0.0f) {
            return 0.0f;
        }
        return std::clamp(splayRadians, -resolvedMax, resolvedMax);
    }

    inline bool buildSurfaceContactSplayValues(
        const SolvedGrabFingerPose& pose,
        const root_flattened_finger_skeleton_runtime::Snapshot& liveFingerSnapshot,
        std::array<float, 5>& outSplayRadians,
        float maxSplayRadians = kDefaultSurfaceContactSplayMaxRadians)
    {
        outSplayRadians = {};
        if (!hasSurfaceContactSplayCandidates(pose)) {
            return false;
        }

        const auto liveLandmarks = root_flattened_finger_skeleton_runtime::buildLandmarkSet(liveFingerSnapshot);
        if (!liveLandmarks.valid) {
            return false;
        }

        bool anySplay = false;
        for (std::size_t finger = 0; finger < pose.surfaceAimTargetValid.size(); ++finger) {
            if (pose.surfaceAimTargetValid[finger] == 0 || !liveLandmarks.fingers[finger].valid) {
                continue;
            }

            const RE::NiPoint3 toSurface = pose.surfaceAimTarget[finger] - liveLandmarks.fingers[finger].base;
            if (distanceSquared(toSurface, RE::NiPoint3{}) <= 0.000001f) {
                continue;
            }

            const float splayRadians = clampSurfaceContactSplayRadians(
                signedPalmPlaneSplayRadians(liveLandmarks.fingers[finger].openDirection, toSurface, liveLandmarks.palmNormalWorld),
                maxSplayRadians);
            if (std::abs(splayRadians) <= 0.0001f) {
                continue;
            }

            outSplayRadians[finger] = splayRadians;
            anySplay = true;
        }

        return anySplay;
    }

    inline bool resolveSurfaceContactSplayValues(
        bool isLeft,
        const SolvedGrabFingerPose& pose,
        std::array<float, 5>& outSplayRadians,
        float maxSplayRadians = kDefaultSurfaceContactSplayMaxRadians)
    {
        outSplayRadians = {};
        if (!hasSurfaceContactSplayCandidates(pose)) {
            return false;
        }

        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
        if (!root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, liveFingerSnapshot)) {
            return false;
        }

        return buildSurfaceContactSplayValues(pose, liveFingerSnapshot, outSplayRadians, maxSplayRadians);
    }

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

        outTriangles.reserve((std::min)(triangles.size(), kMaxFingerPoseCandidateTriangles));
        for (const auto& triangle : triangles) {
            if (triangleDistanceSquaredToPoint(triangle, grabGripPoint) <= maxTriangleDistanceSquared) {
                appendTriangleCandidate(triangle, outTriangles);
                if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                    break;
                }
            }
        }
    }

    inline void appendCandidateTriangles(const std::vector<TriangleData>& triangles, const GrabFingerPoseTargetSet& poseTargets, float maxTriangleDistanceSquared,
        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>>& outTriangles)
    {
        outTriangles.clear();
        if (!std::isfinite(maxTriangleDistanceSquared) || maxTriangleDistanceSquared <= 0.0f) {
            return;
        }

        if (poseTargets.useWholeMeshForMissingTargets && poseTargets.targetCount == 0) {
            outTriangles.reserve((std::min)(triangles.size(), kMaxFingerPoseCandidateTriangles));
            if (!poseTargets.seatPointValid || triangles.size() <= kMaxFingerPoseCandidateTriangles) {
                for (const auto& triangle : triangles) {
                    appendTriangleCandidate(triangle, outTriangles);
                    if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                        break;
                    }
                }
                return;
            }

            struct RankedTriangle
            {
                float distanceSquared = 0.0f;
                std::size_t index = 0;
            };

            std::vector<RankedTriangle> rankedTriangles;
            rankedTriangles.reserve(triangles.size());
            for (std::size_t i = 0; i < triangles.size(); ++i) {
                rankedTriangles.push_back(RankedTriangle{
                    triangleDistanceSquaredToPoint(triangles[i], poseTargets.seatPointWorld),
                    i,
                });
            }

            const auto selectedEnd = rankedTriangles.begin() + kMaxFingerPoseCandidateTriangles;
            std::nth_element(rankedTriangles.begin(), selectedEnd, rankedTriangles.end(), [](const RankedTriangle& lhs, const RankedTriangle& rhs) {
                if (lhs.distanceSquared == rhs.distanceSquared) {
                    return lhs.index < rhs.index;
                }
                return lhs.distanceSquared < rhs.distanceSquared;
            });
            std::sort(rankedTriangles.begin(), selectedEnd, [](const RankedTriangle& lhs, const RankedTriangle& rhs) {
                if (lhs.distanceSquared == rhs.distanceSquared) {
                    return lhs.index < rhs.index;
                }
                return lhs.distanceSquared < rhs.distanceSquared;
            });

            for (auto it = rankedTriangles.begin(); it != selectedEnd; ++it) {
                appendTriangleCandidate(triangles[it->index], outTriangles);
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

        outTriangles.reserve((std::min)(triangles.size(), kMaxFingerPoseCandidateTriangles));
        for (const auto& triangle : triangles) {
            bool nearAnyTarget = false;
            for (std::size_t pointIndex = 0; pointIndex < pointCount; ++pointIndex) {
                const auto& point = points[pointIndex];
                const float bestVertexOrCentroidDistance = triangleDistanceSquaredToPoint(triangle, point);
                if (bestVertexOrCentroidDistance <= maxTriangleDistanceSquared) {
                    nearAnyTarget = true;
                    break;
                }
            }
            if (nearAnyTarget) {
                appendTriangleCandidate(triangle, outTriangles);
                if (outTriangles.size() >= kMaxFingerPoseCandidateTriangles) {
                    break;
                }
            }
        }
    }

    inline SolvedGrabFingerPose solveGrabFingerPoseFromTriangles(const std::vector<TriangleData>& triangles, const RE::NiTransform& handTransform, bool isLeft,
        const RE::NiPoint3& grabAnchorWorld, const GrabFingerPoseTargetSet& poseTargets, float minValue, float maxTriangleDistanceSquared = 100.0f, bool useCurveSolver = true,
        const root_flattened_finger_skeleton_runtime::Snapshot* liveFingerSnapshot = nullptr, bool rejectBacksideHits = true, float surfacePlaneToleranceGameUnits = 1.5f,
        bool allowSurfaceAimTargets = true, float sweepContactRadiusGameUnits = 1.0f, float unreachableFingerOpenValue = -1.0f,
        float thumbSweepMaxOpenValue = grab_finger_pose_math::kMaxFingerOpenValue,
        float fingerSweepMaxOpenValue = grab_finger_pose_math::kMaxFingerOpenValue,
        const GrabFingerArcAnchorHints* arcAnchorHints = nullptr)
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
        const auto liveLandmarks = liveFingerSnapshot ? root_flattened_finger_skeleton_runtime::buildLandmarkSet(*liveFingerSnapshot) : root_flattened_finger_skeleton_runtime::LandmarkSet{};
        if (!liveLandmarks.valid) {
            return result;
        }
        const bool inPowerArmor = liveFingerSnapshot && liveFingerSnapshot->inPowerArmor;
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
            const float fingerOpenLengthWorld = live.length;
            /*
             * The live chord (pad - base) is rotated by the chain's CURRENT
             * curl, but the arc reconstruction treats it as the fully-open
             * zero reference. Solving from a curled chain therefore stopped
             * every finger short by exactly the current curl angle (the
             * "finger-width air gap") and made held re-solves oscillate: each
             * result re-rotated the next solve's reference.
             *
             * Anchor priority:
             * 1. A caller-provided arc-anchor hint (the rotation the finger
             *    was ADOPTED at) de-rotates exactly - no estimation, valid
             *    for the thumb and for over-open poses. Held re-solves pass
             *    hints from the pose currently driving the hand.
             * 2. Otherwise, estimate the current curl from the chord length
             *    via the baked Tip reach table (acquisition solves, where the
             *    chain is mid-motion and the inversion tracks the ACTUAL
             *    pose). THE THUMB IS EXCLUDED from the estimate: its chord
             *    shortens from opposition and twist - motion outside the arc
             *    plane - so the inversion reads a large spurious curl
             *    (session evidence 2026-07-13: thumb inside the mesh on
             *    every grab). With no hint the thumb uses its raw chord.
             */
            RE::NiPoint3 openDirectionWorld = live.openDirection;
            const bool hasArcAnchorHint = arcAnchorHints &&
                                          arcAnchorHints->valid[finger] != 0 &&
                                          std::isfinite(arcAnchorHints->rotationRadians[finger]);
            if (hasArcAnchorHint) {
                if (std::fabs(arcAnchorHints->rotationRadians[finger]) > 0.0035f) {
                    openDirectionWorld = normalizedOrFallback(
                        grab_finger_pose_math::rotateAroundUnitAxis(
                            openDirectionWorld,
                            curlNormalWorld,
                            -arcAnchorHints->rotationRadians[finger]),
                        openDirectionWorld);
                }
            } else if (finger != 0) {
                const auto& liveChain = liveFingerSnapshot->fingers[finger];
                const float liveChordLength = std::sqrt(distanceSquared(liveChain.points[2], liveChain.points[0]));
                const auto chordCurl = grab_finger_pose_math::estimateCalibratedChainCurlFromChord(
                    finger, isLeft, inPowerArmor, fingerOpenLengthWorld, liveChordLength);
                if (chordCurl.valid && std::fabs(chordCurl.chordAngleRadians) > 0.0035f) {
                    openDirectionWorld = normalizedOrFallback(
                        grab_finger_pose_math::rotateAroundUnitAxis(
                            openDirectionWorld,
                            curlNormalWorld,
                            -chordCurl.chordAngleRadians * chordCurl.normalSign),
                        openDirectionWorld);
                }
            }
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
                result.thumbAlternateCurveMaxCurlAngleRadians = grab_finger_pose_math::bakedCalibratedFingerMaxAngleRadians(finger, isLeft, inPowerArmor);
            }

            auto solved = grab_finger_pose_math::FingerCurlValue{};
            solved.value = clampedMin;
            RE::NiPoint3 fallbackHitPoint{};
            bool fallbackHitPointValid = false;
            bool curveSolverRan = false;
            if (useCurveSolver) {
                const bool isThumb = finger == 0;
                /*
                 * Swept-arc solver: the finger closes along its baked hFRIK
                 * arc and stops at first volumetric contact. Backside/surface
                 * gates are unnecessary here — far-side geometry lies later on
                 * the arc than the near surface, so a first-contact stop can
                 * never select it.
                 */
                const auto curveSolved = grab_finger_pose_math::sweepThumbAwareCalibratedFingerCurveCurlValue(candidateTriangles,
                    finger,
                    isLeft,
                    inPowerArmor,
                    baseWorld,
                    curlNormalWorld,
                    thumbAlternateCurlNormalWorld,
                    openDirectionWorld,
                    fingerOpenLengthWorld,
                    clampedMin,
                    isThumb,
                    sweepContactRadiusGameUnits,
                    isThumb ? thumbSweepMaxOpenValue : fingerSweepMaxOpenValue);
                curveSolverRan = true;
                solved = curveSolved.value;
                result.usedAlternateThumbCurve = result.usedAlternateThumbCurve || curveSolved.usedAlternateThumbCurve;
                if (isThumb) {
                    result.hasThumbCurveDiagnostics = true;
                    result.thumbPrimaryCurve = curveSolved.primary;
                    result.thumbAlternateCurve = curveSolved.alternateThumb;
                    result.thumbSidePadCurve = curveSolved.sidePadThumb;
                    result.selectedThumbLane = curveSolved.selectedThumbLane;
                    result.selectedThumbLaneNormalBlend = curveSolved.selectedThumbLaneNormalBlend;
                    result.selectedThumbLaneLocalCorrectionStrength = curveSolved.selectedThumbLaneLocalCorrectionStrength;
                    if (curveSolved.usedAlternateThumbCurve) {
                        result.thumbAlternateCurveNormalWorld = normalizedOrFallback(
                            curveSolved.selectedThumbLaneNormal,
                            thumbAlternateCurlNormalWorld);
                        if (curveSolved.selectedThumbLaneMaxCurlAngleRadians > 0.0001f) {
                            result.thumbAlternateCurveMaxCurlAngleRadians = curveSolved.selectedThumbLaneMaxCurlAngleRadians;
                        }
                    }
                    const auto& selectedThumbCurve = curveSolved.selectedThumbCurve;
                    const bool alternateThumbSurfaceHit =
                        curveSolved.usedAlternateThumbCurve &&
                        selectedThumbCurve.hit &&
                        selectedThumbCurve.hasHitPoint &&
                        !selectedThumbCurve.openedByBehindContact &&
                        selectedThumbCurve.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid;
                    result.usedAlternateThumbSurfaceHit = alternateThumbSurfaceHit;
                    if (alternateThumbSurfaceHit) {
                        result.surfaceAimTarget[0] = RE::NiPoint3{
                            selectedThumbCurve.hitPointX,
                            selectedThumbCurve.hitPointY,
                            selectedThumbCurve.hitPointZ
                        };
                        result.surfaceAimTargetValid[0] = 1;
                        if (selectedThumbCurve.hasHitNormal) {
                            result.surfaceAimNormal[0] = normalizedOrFallback(
                                RE::NiPoint3{
                                    selectedThumbCurve.hitNormalX,
                                    selectedThumbCurve.hitNormalY,
                                    selectedThumbCurve.hitNormalZ },
                                curlNormalWorld);
                            result.surfaceAimNormalValid[0] = 1;
                        }
                    }
                }
            }
            if (useTarget && !solved.hit && !curveSolverRan) {
                solved = grab_finger_pose_math::solveFingerCurlValue(candidateTriangles,
                    baseWorld,
                    probeDirection,
                    probeDistance,
                    clampedMin,
                    kFingerProbeRadius,
                    curlNormalWorld,
                    openDirectionWorld,
                    grab_finger_pose_math::bakedCalibratedFingerMaxAngleRadians(finger, isLeft, inPowerArmor),
                    fingerTargetWorld,
                    fingerTargetNormalWorld,
                    rejectBacksideHits && useTargetNormal,
                    surfacePlaneToleranceGameUnits,
                    &fallbackHitPoint);
                fallbackHitPointValid = solved.hit && solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid;
            }
            /*
             * Generic grab poses often use whole-mesh probing instead of
             * explicit per-finger targets. A miss in that mode deliberately
             * closes to 0.3 so missing targets are visible without reusing the
             * palm seat point as fake finger evidence.
             */
            if (!solved.hit &&
                (!useTarget ||
                    solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::BackSurface ||
                    solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::Rejected)) {
                solved.value = missedFingerCurlFallbackValue(useTarget, solved.hitKind, clampedMin);
                solved.rawCurveValue = solved.value;
            }
            /*
             * Whole arc out of reach: the object cannot be touched from the
             * current relative pose. During pull-to-grab flight callers pass
             * the anticipation value so the hand awaits arrival open instead
             * of closing on nothing; the legacy sentinel (-1) keeps the plain
             * miss semantics for at-hand solves.
             */
            if (curveSolverRan && !solved.hit && solved.outOfReach && unreachableFingerOpenValue >= 0.0f) {
                solved.value = std::clamp(unreachableFingerOpenValue, clampedMin, 1.0f);
                solved.rawCurveValue = solved.value;
            }
            result.values[finger] = solved.value;
            result.hitKind[finger] = solved.hitKind;
            /*
             * Record the contact-row rotation the finger was adopted at, for
             * feedback-free anchor de-rotation on held re-solves. Only
             * palm-plane sweep contacts qualify: the thumb's alternate lanes
             * rotate in a blended plane the palm-normal de-rotation cannot
             * invert, and a value floored by minValue no longer matches its
             * contact row.
             */
            if (curveSolverRan && solved.hit &&
                solved.hitKind == grab_finger_pose_math::FingerCurlValue::HitKind::FrontValid &&
                solved.rawCurveValue >= clampedMin - 0.0001f &&
                (finger != 0 || !result.usedAlternateThumbCurve)) {
                const auto& bakedAnchorProfile = grab_finger_calibration_data::bakedGrabFingerHandProfile(isLeft, inPowerArmor);
                const float bakedAnchorNormalSign = bakedAnchorProfile.fingers[finger].normalSign < 0.0f ? -1.0f : 1.0f;
                result.contactArcRotationRadians[finger] = solved.distance * bakedAnchorNormalSign;
                result.contactArcRotationValid[finger] = 1;
            }
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
        bool allowSurfaceAimTargets = true, float sweepContactRadiusGameUnits = 1.0f, float unreachableFingerOpenValue = -1.0f,
        float thumbSweepMaxOpenValue = grab_finger_pose_math::kMaxFingerOpenValue,
        float fingerSweepMaxOpenValue = grab_finger_pose_math::kMaxFingerOpenValue,
        const GrabFingerArcAnchorHints* arcAnchorHints = nullptr)
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
            allowSurfaceAimTargets,
            sweepContactRadiusGameUnits,
            unreachableFingerOpenValue,
            thumbSweepMaxOpenValue,
            fingerSweepMaxOpenValue,
            arcAnchorHints);
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
    inline constexpr float kMaxSurfaceAimCorrectionRadians = 0.08726646259971647f;

    [[nodiscard]] inline std::uint16_t sanitizeFingerLocalTransformMask(std::uint16_t mask)
    {
        return static_cast<std::uint16_t>(mask & kFullFingerLocalTransformMask);
    }

    [[nodiscard]] inline bool shouldPublishLocalTransformPose(
        bool enabled,
        bool poseSolved,
        bool hasCanonicalHandPose,
        bool hasBaselineApi,
        bool hasPublishApi)
    {
        return enabled && poseSolved && hasCanonicalHandPose && hasBaselineApi && hasPublishApi;
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

    [[nodiscard]] inline float surfaceAimSegmentCorrectionWeight(std::size_t fingerIndex, std::size_t segment)
    {
        static constexpr float kThumbWeights[3]{ 0.08f, 0.18f, 0.30f };
        if (segment >= 3) {
            return 0.0f;
        }
        return fingerIndex == 0 ? kThumbWeights[segment] : 0.0f;
    }

    [[nodiscard]] inline float boundedSurfaceAimCorrectionRadians(
        float rawAngleRadians,
        float strength,
        float maxCorrectionRadians,
        std::size_t fingerIndex,
        std::size_t segment)
    {
        if (!std::isfinite(rawAngleRadians) || rawAngleRadians <= 0.0f ||
            !std::isfinite(maxCorrectionRadians) || maxCorrectionRadians <= 0.0f) {
            return 0.0f;
        }

        const float segmentWeight = surfaceAimSegmentCorrectionWeight(fingerIndex, segment);
        if (segmentWeight <= 0.0f) {
            return 0.0f;
        }

        const float resolvedStrength = sanitizeUnitStrength(strength, 0.0f);
        if (resolvedStrength <= 0.0f) {
            return 0.0f;
        }

        return std::min(
            std::min(rawAngleRadians * resolvedStrength * segmentWeight, maxCorrectionRadians * segmentWeight),
            kMaxSurfaceAimCorrectionRadians);
    }

    [[nodiscard]] inline bool shouldApplySurfaceAimCorrection(
        std::size_t fingerIndex,
        bool alternateThumbPlaneCorrection,
        bool thumbSurfaceFollowAllowed = true)
    {
        return fingerIndex == 0 && thumbSurfaceFollowAllowed && !alternateThumbPlaneCorrection;
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
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"

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
        frik_visual_authority::FingerLocalTransformOverride& transforms)
    {
        const float configuredStrength = grab_finger_local_transform_math::sanitizeUnitStrength(
            strength, grab_finger_local_transform_math::kDefaultThumbAlternateCurveStrength);
        const float laneStrength = fingerPose.selectedThumbLaneLocalCorrectionStrength > 0.0f ?
            grab_finger_local_transform_math::sanitizeUnitStrength(fingerPose.selectedThumbLaneLocalCorrectionStrength, 1.0f) :
            1.0f;
        const float curveStrength = configuredStrength * laneStrength;
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
        const frik_visual_authority::FingerLocalTransformOverride& baseline,
        Options options,
        frik_visual_authority::FingerLocalTransformOverride& outTransforms,
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

                const std::size_t segment = index % 3;

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
                angle = grab_finger_local_transform_math::boundedSurfaceAimCorrectionRadians(
                    angle,
                    strength,
                    maxCorrectionRadians,
                    finger,
                    segment);
                if (angle <= 0.0001f) {
                    continue;
                }

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

    [[nodiscard]] inline frik_visual_authority::FingerLocalTransformOverride smoothLocalTransforms(
        const frik_visual_authority::FingerLocalTransformOverride& target,
        State& state,
        float smoothingSpeed,
        float deltaTime)
    {
        frik_visual_authority::FingerLocalTransformOverride smoothed = target;
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

    inline void clearLocalTransformOverride(const char* tag, frik_visual_authority::Hand hand, int priority, State& state)
    {
        if (!state.hasCurrentTransforms) {
            state = {};
            return;
        }

        frik_visual_authority::FingerLocalTransformOverride clearData{};
        (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(tag, hand, &clearData, priority);
        state = {};
    }

    [[nodiscard]] inline bool publishLocalTransformPose(
        const char* tag,
        frik_visual_authority::Hand hand,
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        const frik_visual_authority::HandPoseData& handPose,
        Options options,
        float deltaTime,
        int priority,
        State& state)
    {
        const auto* api = frik_visual_authority::api();
        const bool canPublish =
            grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                options.enabled,
                fingerPose.solved,
                true,
                api && api->getHandPoseLocalTransformsForPose != nullptr,
                api && api->setHandPoseCustomLocalTransformsWithPriority != nullptr);
        if (!canPublish) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride baseline{};
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(hand, handPose, &baseline)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride corrected{};
        if (!buildSurfaceCorrectedLocalTransforms(isLeft, fingerPose, baseline, options, corrected)) {
            clearLocalTransformOverride(tag, hand, priority, state);
            return false;
        }

        const Options sanitizedOptions = sanitizeOptions(options);
        const auto smoothed = smoothLocalTransforms(corrected, state, sanitizedOptions.smoothingSpeed, deltaTime);
        (void)frik_visual_authority::setHandPoseCustomLocalTransformsWithPriority(tag, hand, &smoothed, priority);
        return true;
    }
}

#pragma once

/*
 * Grab finger helpers are grouped here so pose math and live runtime transform sampling stay in one hand-contact surface.
 */


// ---- GrabFingerPoseMath.h ----

#include <array>
#include <bitset>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <ROCK/FingerSweep.h>

#include "physics-interaction/grab/GeneratedGrabFingerCalibration.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/performance/PerformanceProfiler.h"

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
            if (aSquared <= kPointEpsilonSquared || bSquared <= kPointEpsilonSquared || cSquared <= kPointEpsilonSquared) {
                return true;
            }

            const double aLength = std::sqrt(aSquared);
            const double bLength = std::sqrt(bSquared);
            const double cLength = std::sqrt(cSquared);
            const double numerator = static_cast<double>(dot(a, cross(b, c)));
            const double denominator = aLength * bLength * cLength +
                                       static_cast<double>(dot(a, b)) * cLength +
                                       static_cast<double>(dot(b, c)) * aLength +
                                       static_cast<double>(dot(c, a)) * bLength;
            signedSolidAngle += 2.0 * std::atan2(numerator, denominator);
        }
        return std::isfinite(signedSolidAngle) && std::abs(signedSolidAngle) > kTwoPi;
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

    // June 2, 2026 (88f8737d): parametric plane contact, without baked reach tables.
    using finger_sweep::kMaxFingerOpenValue;
    using finger_sweep::kMaxOverOpenValue;
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

    using finger_sweep::clampOpenValue;

    // Every finger may carry an over-open value; the sweep's per-call cap is the policy boundary.
    inline std::array<float, 15> expandFingerCurlsToJointValues(const std::array<float, 5>& values)
    {
        return finger_sweep::expandCurlsToJointValues(values);
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

    // A mesh finger pose: per-finger curls and the FRIK joint values ROCK publishes.
    struct SolvedGrabFingerPose
    {
        std::array<float, 5> values{ 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
        std::array<float, 15> jointValues{};
        std::uint8_t contactValidMask = 0;
        int hitCount = 0;
        int candidateTriangleCount = 0;
        std::uint32_t spatialNodeVisitCount = 0;
        std::uint32_t spatialTriangleTestCount = 0;
        bool solved = false;
        bool hasJointValues = false;
        bool usedSpatialIndex = false;
        bool usedLiveRootFlattenedFingerBones = false;
    };

    inline float dotPoint(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return vector_math::dot(a, b);
    }

    inline RE::NiPoint3 normalizedOrFallback(const RE::NiPoint3& value, const RE::NiPoint3& fallback)
    {
        const float lengthSquared = vector_math::lengthSquared(value);
        if (lengthSquared <= 0.000001f) {
            return normalizeDirection(fallback);
        }

        const float inv = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inv, value.y * inv, value.z * inv);
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
        return vector_math::hasFiniteComponents(point);
    }

    struct FingerPoseSpatialQueryStats
    {
        std::uint32_t nodeVisits = 0;
        std::uint32_t triangleTests = 0;
    };

    /*
     * Immutable, object-local BVH for the bounded finger-pose triangle set.
     * A regular grab builds it once at commit, then every swept-arc sphere
     * probe queries the exact same triangles without linearly revisiting all
     * 2048 candidates. Nodes and triangles own no engine pointers, and query
     * uses a fixed stack so the runtime sweep performs no heap allocation.
     */
    class FingerPoseTriangleSpatialIndex
    {
    public:
        void clear()
        {
            _triangles.clear();
            _triangleIndices.clear();
            _nodes.clear();
        }

        template <class TriangleContainer>
        bool buildFromLocalTriangles(const TriangleContainer& localTriangles)
        {
            performance_profiler::ScopedTimer timer(performance_profiler::Scope::GrabFingerIndexBuild);
            clear();
            const std::size_t triangleLimit = (std::min)(localTriangles.size(), kMaxFingerPoseCandidateTriangles);
            _triangles.reserve(triangleLimit);
            for (const auto& source : localTriangles) {
                if (!isFinitePoint(source.v0) || !isFinitePoint(source.v1) || !isFinitePoint(source.v2)) {
                    continue;
                }
                _triangles.push_back(grab_finger_pose_math::Triangle<RE::NiPoint3>{ source.v0, source.v1, source.v2 });
                if (_triangles.size() >= kMaxFingerPoseCandidateTriangles) {
                    break;
                }
            }
            if (_triangles.empty()) {
                return false;
            }

            // Keep the original float centroid arithmetic, split axes and tie
            // order. Compute each triangle's bounds/centroid once for the build.
            std::vector<TriangleBuildData> prepared;
            prepared.reserve(_triangles.size());
            for (const auto& triangle : _triangles) {
                TriangleBuildData data{ triangle.v0, triangle.v0, triangleCentroid(triangle) };
                expandBounds(data.minimum, data.maximum, triangle.v1);
                expandBounds(data.minimum, data.maximum, triangle.v2);
                prepared.push_back(data);
            }

            _triangleIndices.resize(_triangles.size());
            for (std::size_t i = 0; i < _triangleIndices.size(); ++i) {
                _triangleIndices[i] = static_cast<std::uint32_t>(i);
            }
            _nodes.reserve(_triangles.size() * 2);
            (void)buildNode(prepared, 0, static_cast<std::uint32_t>(_triangleIndices.size()));
            return !_nodes.empty();
        }

        [[nodiscard]] bool empty() const { return _nodes.empty(); }
        [[nodiscard]] std::size_t triangleCount() const { return _triangles.size(); }

        using CandidateMask = std::bitset<kMaxFingerPoseCandidateTriangles>;

        // The caller's world triangles must be this index's same finite, bounded
        // local triangle sequence transformed by objectWorldTransform. Transform
        // node bounds forward, rather than assuming an exactly orthonormal inverse.
        // Candidate bits preserve source order, including the pad solver's last tie.
        CandidateMask collectWorldBoundsCandidates(const RE::NiTransform& objectWorldTransform,
            const RE::NiPoint3& minimum, const RE::NiPoint3& maximum,
            FingerPoseSpatialQueryStats* stats = nullptr) const
        {
            CandidateMask candidates;
            if (_nodes.empty()) return candidates;
            double coefficients[3][3]{};
            for (int axis = 0; axis < 3; ++axis) {
                for (int local = 0; local < 3; ++local) {
                    coefficients[axis][local] = static_cast<double>(objectWorldTransform.rotate.entry[local][axis]) * objectWorldTransform.scale;
                }
            }
            std::array<std::uint32_t, 64> stack{};
            std::size_t count = 1;
            while (count != 0) {
                const auto& node = _nodes[stack[--count]];
                if (stats) ++stats->nodeVisits;
                bool outside = false;
                for (int axis = 0; axis < 3; ++axis) {
                    double low = axisValue(objectWorldTransform.translate, axis);
                    double high = low;
                    double magnitude = std::abs(low);
                    for (int local = 0; local < 3; ++local) {
                        const double a = coefficients[axis][local] * axisValue(node.boundsMin, local);
                        const double b = coefficients[axis][local] * axisValue(node.boundsMax, local);
                        low += (std::min)(a, b);
                        high += (std::max)(a, b);
                        magnitude += (std::max)(std::abs(a), std::abs(b));
                    }
                    // Widen only the rejection test for float world-coordinate
                    // and exact-predicate rounding. Extra candidates cannot alter
                    // results; non-finite bounds deliberately do not reject.
                    const double padding = 0.001 + magnitude * 0.00001;
                    if (std::isfinite(low) && std::isfinite(high) &&
                        (high + padding < axisValue(minimum, axis) || low - padding > axisValue(maximum, axis))) {
                        outside = true;
                        break;
                    }
                }
                if (outside) continue;
                if (node.isLeaf()) {
                    for (std::uint32_t i = 0; i < node.count; ++i) candidates.set(_triangleIndices[node.begin + i]);
                } else {
                    // Median splits of at most 2048 triangles need < 12 deferred siblings.
                    stack[count++] = node.right;
                    stack[count++] = node.left;
                }
            }
            return candidates;
        }

        [[nodiscard]] bool querySphereWorld(const RE::NiTransform& objectWorldTransform, const RE::NiPoint3& centerWorld, float radiusWorld, RE::NiPoint3* outPointWorld,
            RE::NiPoint3* outNormalWorld, FingerPoseSpatialQueryStats* stats = nullptr, bool orientNormalTowardCenter = true) const
        {
            if (_nodes.empty() || !isFinitePoint(centerWorld) || !std::isfinite(radiusWorld) || radiusWorld <= 0.0f || !std::isfinite(objectWorldTransform.scale) ||
                std::abs(objectWorldTransform.scale) <= 0.000001f) {
                return false;
            }

            const RE::NiPoint3 centerLocal = transform_math::worldPointToLocal(objectWorldTransform, centerWorld);
            const float radiusLocal = radiusWorld / std::abs(objectWorldTransform.scale);
            float bestDistanceSquared = radiusLocal * radiusLocal;
            std::uint32_t bestTriangleIndex = kInvalidIndex;
            RE::NiPoint3 bestPointLocal{};

            std::array<std::uint32_t, 64> stack{};
            std::size_t stackSize = 0;
            stack[stackSize++] = 0;
            while (stackSize > 0) {
                const std::uint32_t nodeIndex = stack[--stackSize];
                const Node& node = _nodes[nodeIndex];
                if (stats) {
                    ++stats->nodeVisits;
                }
                if (pointAabbDistanceSquared(centerLocal, node.boundsMin, node.boundsMax) > bestDistanceSquared) {
                    continue;
                }

                if (node.isLeaf()) {
                    for (std::uint32_t offset = 0; offset < node.count; ++offset) {
                        const std::uint32_t triangleIndex = _triangleIndices[node.begin + offset];
                        const auto& triangle = _triangles[triangleIndex];
                        if (stats) {
                            ++stats->triangleTests;
                        }
                        const RE::NiPoint3 closest = grab_finger_pose_math::closestPointOnTriangle(centerLocal, triangle);
                        const float distanceSquared = grab_finger_pose_math::lengthSquared(grab_finger_pose_math::sub(closest, centerLocal));
                        if (distanceSquared <= bestDistanceSquared &&
                            (bestTriangleIndex == kInvalidIndex || distanceSquared < bestDistanceSquared || triangleIndex < bestTriangleIndex)) {
                            bestDistanceSquared = distanceSquared;
                            bestTriangleIndex = triangleIndex;
                            bestPointLocal = closest;
                        }
                    }
                    continue;
                }

                const float leftDistance = pointAabbDistanceSquared(centerLocal, _nodes[node.left].boundsMin, _nodes[node.left].boundsMax);
                const float rightDistance = pointAabbDistanceSquared(centerLocal, _nodes[node.right].boundsMin, _nodes[node.right].boundsMax);
                const std::uint32_t nearChild = leftDistance <= rightDistance ? node.left : node.right;
                const std::uint32_t farChild = leftDistance <= rightDistance ? node.right : node.left;
                if (pointAabbDistanceSquared(centerLocal, _nodes[farChild].boundsMin, _nodes[farChild].boundsMax) <= bestDistanceSquared) {
                    if (stackSize >= stack.size()) {
                        return false;
                    }
                    stack[stackSize++] = farChild;
                }
                if (pointAabbDistanceSquared(centerLocal, _nodes[nearChild].boundsMin, _nodes[nearChild].boundsMax) <= bestDistanceSquared) {
                    if (stackSize >= stack.size()) {
                        return false;
                    }
                    stack[stackSize++] = nearChild;
                }
            }

            if (bestTriangleIndex == kInvalidIndex) {
                return false;
            }
            if (outPointWorld) {
                *outPointWorld = transform_math::localPointToWorld(objectWorldTransform, bestPointLocal);
            }
            if (outNormalWorld) {
                RE::NiPoint3 normalWorld = transform_math::localVectorToWorld(objectWorldTransform, grab_finger_pose_math::triangleNormal(_triangles[bestTriangleIndex]));
                normalWorld = normalizedOrFallback(normalWorld, RE::NiPoint3{});
                const RE::NiPoint3 pointWorld = transform_math::localPointToWorld(objectWorldTransform, bestPointLocal);
                // Unoriented, the normal keeps the mesh winding (outward), so a caller can tell inside from outside.
                if (orientNormalTowardCenter && dotPoint(normalWorld, centerWorld - pointWorld) < 0.0f) {
                    normalWorld = normalWorld * -1.0f;
                }
                *outNormalWorld = normalWorld;
            }
            return true;
        }

        [[nodiscard]] bool queryPointInsideWorld(
            const RE::NiTransform& objectWorldTransform,
            const RE::NiPoint3& pointWorld,
            FingerPoseSpatialQueryStats* stats = nullptr) const
        {
            if (_triangles.empty() || !isFinitePoint(pointWorld) || !std::isfinite(objectWorldTransform.scale) ||
                std::abs(objectWorldTransform.scale) <= 0.000001f) {
                return false;
            }
            if (stats) {
                stats->triangleTests += static_cast<std::uint32_t>(_triangles.size());
            }
            return grab_finger_pose_math::pointInsideClosedTriangleMesh(
                _triangles,
                transform_math::worldPointToLocal(objectWorldTransform, pointWorld));
        }

    private:
        static constexpr std::uint32_t kInvalidIndex = 0xFFFF'FFFFu;
        static constexpr std::uint32_t kLeafTriangleCount = 8;

        struct TriangleBuildData
        {
            RE::NiPoint3 minimum{}, maximum{}, centroid{};
        };

        struct Node
        {
            RE::NiPoint3 boundsMin{};
            RE::NiPoint3 boundsMax{};
            std::uint32_t left = kInvalidIndex;
            std::uint32_t right = kInvalidIndex;
            std::uint32_t begin = 0;
            std::uint32_t count = 0;

            [[nodiscard]] bool isLeaf() const { return count > 0; }
        };

        static void expandBounds(RE::NiPoint3& boundsMin, RE::NiPoint3& boundsMax, const RE::NiPoint3& point)
        {
            boundsMin.x = (std::min)(boundsMin.x, point.x);
            boundsMin.y = (std::min)(boundsMin.y, point.y);
            boundsMin.z = (std::min)(boundsMin.z, point.z);
            boundsMax.x = (std::max)(boundsMax.x, point.x);
            boundsMax.y = (std::max)(boundsMax.y, point.y);
            boundsMax.z = (std::max)(boundsMax.z, point.z);
        }

        [[nodiscard]] static RE::NiPoint3 triangleCentroid(const grab_finger_pose_math::Triangle<RE::NiPoint3>& triangle)
        {
            return (triangle.v0 + triangle.v1 + triangle.v2) * (1.0f / 3.0f);
        }

        [[nodiscard]] static float axisValue(const RE::NiPoint3& point, int axis) { return axis == 0 ? point.x : (axis == 1 ? point.y : point.z); }

        [[nodiscard]] static float pointAabbDistanceSquared(const RE::NiPoint3& point, const RE::NiPoint3& boundsMin, const RE::NiPoint3& boundsMax)
        {
            const float dx = point.x < boundsMin.x ? boundsMin.x - point.x : (point.x > boundsMax.x ? point.x - boundsMax.x : 0.0f);
            const float dy = point.y < boundsMin.y ? boundsMin.y - point.y : (point.y > boundsMax.y ? point.y - boundsMax.y : 0.0f);
            const float dz = point.z < boundsMin.z ? boundsMin.z - point.z : (point.z > boundsMax.z ? point.z - boundsMax.z : 0.0f);
            return dx * dx + dy * dy + dz * dz;
        }

        std::uint32_t buildNode(const std::vector<TriangleBuildData>& prepared, std::uint32_t begin, std::uint32_t end)
        {
            const float infinity = std::numeric_limits<float>::infinity();
            Node node{};
            node.boundsMin = RE::NiPoint3{ infinity, infinity, infinity };
            node.boundsMax = RE::NiPoint3{ -infinity, -infinity, -infinity };
            RE::NiPoint3 centroidMin{ infinity, infinity, infinity };
            RE::NiPoint3 centroidMax{ -infinity, -infinity, -infinity };
            for (std::uint32_t i = begin; i < end; ++i) {
                const auto& triangle = prepared[_triangleIndices[i]];
                expandBounds(node.boundsMin, node.boundsMax, triangle.minimum);
                expandBounds(node.boundsMin, node.boundsMax, triangle.maximum);
                expandBounds(centroidMin, centroidMax, triangle.centroid);
            }

            const std::uint32_t nodeIndex = static_cast<std::uint32_t>(_nodes.size());
            _nodes.push_back(node);
            const std::uint32_t count = end - begin;
            if (count <= kLeafTriangleCount) {
                _nodes[nodeIndex].begin = begin;
                _nodes[nodeIndex].count = count;
                return nodeIndex;
            }

            const RE::NiPoint3 centroidExtent = centroidMax - centroidMin;
            const int splitAxis = centroidExtent.x >= centroidExtent.y && centroidExtent.x >= centroidExtent.z ? 0 : (centroidExtent.y >= centroidExtent.z ? 1 : 2);
            const std::uint32_t middle = begin + count / 2;
            std::nth_element(_triangleIndices.begin() + begin, _triangleIndices.begin() + middle, _triangleIndices.begin() + end, [&](std::uint32_t lhs, std::uint32_t rhs) {
                const float lhsValue = axisValue(prepared[lhs].centroid, splitAxis);
                const float rhsValue = axisValue(prepared[rhs].centroid, splitAxis);
                return lhsValue == rhsValue ? lhs < rhs : lhsValue < rhsValue;
            });
            const std::uint32_t left = buildNode(prepared, begin, middle);
            const std::uint32_t right = buildNode(prepared, middle, end);
            _nodes[nodeIndex].left = left;
            _nodes[nodeIndex].right = right;
            return nodeIndex;
        }

        std::vector<grab_finger_pose_math::Triangle<RE::NiPoint3>> _triangles;
        std::vector<std::uint32_t> _triangleIndices;
        std::vector<Node> _nodes;
    };

}

// ---- GrabFingerLocalTransformMath.h ----

/*
 * Safety checks for per-bone finger local transforms ROCK hands to FRIK
 * (saved exact grab poses and authored weapon grips). A transform that would
 * scale, shear, mirror or fling a finger is rejected before publication.
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstddef>

namespace rock::grab_finger_local_transform_math
{
    inline constexpr std::uint16_t kFullFingerLocalTransformMask = 0x7FFF;
    [[nodiscard]] inline std::uint16_t sanitizeFingerLocalTransformMask(std::uint16_t mask)
    {
        return static_cast<std::uint16_t>(mask & kFullFingerLocalTransformMask);
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
        return r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1]) -
               r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0]) +
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
        if (!std::isfinite(determinant) || std::abs(determinant - 1.0f) > kPublishedFingerRotationDeterminantTolerance) {
            return FingerLocalTransformSafetyFailure::RotationDeterminant;
        }
        return FingerLocalTransformSafetyFailure::None;
    }

    template <class Transform>
    [[nodiscard]] inline FingerLocalTransformSafetyFailure inspectFingerLocalTransformForPublication(const Transform& transform)
    {
        if (!sceneTransformHasUsableBasis(transform)) {
            return FingerLocalTransformSafetyFailure::NonFinite;
        }
        if (transform.scale < kMinPublishedFingerLocalScale || transform.scale > kMaxPublishedFingerLocalScale) {
            return FingerLocalTransformSafetyFailure::ScaleMagnitude;
        }

        const float translationLengthSquared =
            transform.translate.x * transform.translate.x +
            transform.translate.y * transform.translate.y +
            transform.translate.z * transform.translate.z;
        constexpr float kMaxTranslationSquared =
            kMaxPublishedFingerLocalTranslationGameUnits * kMaxPublishedFingerLocalTranslationGameUnits;
        if (!std::isfinite(translationLengthSquared) || translationLengthSquared > kMaxTranslationSquared) {
            return FingerLocalTransformSafetyFailure::TranslationMagnitude;
        }
        return inspectStoredRotationBasis(transform);
    }

}

// ---- GrabFingerLocalTransformRuntime.h ----

/*
 * Publication state for full-hand finger local transforms under one FRIK tag:
 * the whole-override safety check, rejection logging, and clearing a held
 * override when a joint-value pose replaces it.
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
    struct State
    {
        std::array<RE::NiTransform, 15> currentTransforms{};
        std::uint16_t currentMask = 0;
        bool hasCurrentTransforms = false;
    };

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

    inline constexpr std::size_t kInvalidFingerLocalTransformIndex = 15;

    inline void logRejectedFingerTransform(
        bool isLeft,
        std::size_t index,
        const char* stage,
        const RE::NiTransform& transform,
        grab_finger_local_transform_math::FingerLocalTransformSafetyFailure failure)
    {
        const std::size_t finger = index / 3;
        const std::size_t segment = index % 3;
        const char* boneName = index < kInvalidFingerLocalTransformIndex ?
            root_flattened_finger_skeleton_runtime::fingerBoneName(isLeft, finger, segment) :
            nullptr;
        const float translationLengthSquared =
            transform.translate.x * transform.translate.x +
            transform.translate.y * transform.translate.y +
            transform.translate.z * transform.translate.z;
        const float translationLength =
            std::isfinite(translationLengthSquared) && translationLengthSquared >= 0.0f ?
            std::sqrt(translationLengthSquared) :
            translationLengthSquared;

        ROCK_LOG_SAMPLE_WARN(Hand,
            1000,
            "Dynamic grab finger transform rejected hand={} stage={} bone={} index={} detail={} scale={:.6f} translation={:.3f} rowNormSq=({:.4f},{:.4f},{:.4f}) rowDot=({:.4f},{:.4f},{:.4f}) determinant={:.4f}",
            isLeft ? "left" : "right",
            stage ? stage : "unknown",
            boneName ? boneName : "unknown",
            index,
            grab_finger_local_transform_math::fingerLocalTransformSafetyFailureName(failure),
            transform.scale,
            translationLength,
            grab_finger_local_transform_math::storedRotationRowDot(transform, 0, 0),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 1, 1),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 2, 2),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 0, 1),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 0, 2),
            grab_finger_local_transform_math::storedRotationRowDot(transform, 1, 2),
            grab_finger_local_transform_math::storedRotationDeterminant(transform));
    }

    [[nodiscard]] inline bool fingerLocalTransformOverrideIsSafeForPublication(
        const frik_visual_authority::FingerLocalTransformOverride& transforms,
        std::size_t* outFailureIndex = nullptr,
        grab_finger_local_transform_math::FingerLocalTransformSafetyFailure* outFailure = nullptr)
    {
        if (outFailureIndex) {
            *outFailureIndex = kInvalidFingerLocalTransformIndex;
        }
        if (outFailure) {
            *outFailure = grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::None;
        }
        if (grab_finger_local_transform_math::sanitizeFingerLocalTransformMask(transforms.enabledMask) !=
            grab_finger_local_transform_math::kFullFingerLocalTransformMask) {
            return false;
        }

        for (std::size_t index = 0; index < std::size(transforms.localTransforms); ++index) {
            const auto failure = grab_finger_local_transform_math::inspectFingerLocalTransformForPublication(
                transforms.localTransforms[index]);
            if (failure == grab_finger_local_transform_math::FingerLocalTransformSafetyFailure::None) {
                continue;
            }
            if (outFailureIndex) {
                *outFailureIndex = index;
            }
            if (outFailure) {
                *outFailure = failure;
            }
            return false;
        }
        return true;
    }

    inline void clearLocalTransformOverride(const char* tag, frik_visual_authority::Hand hand, int priority, State& state)
    {
        if (!state.hasCurrentTransforms) {
            state = {};
            return;
        }

        frik_visual_authority::FingerLocalTransformOverride clearData{};
        (void)frik_visual_authority::setHandPoseCustomLocalTransforms(tag, hand, &clearData, priority);
        state = {};
    }

}

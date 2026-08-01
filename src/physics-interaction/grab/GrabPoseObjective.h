#pragma once

#include "physics-interaction/grab/GrabCore.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <span>
#include <vector>

/*
 * Grab pose objective + deterministic pose solve, fitted offline against
 * Bruno's verified ground-truth captures (63 holds, 2026-08-01). Design and
 * validation record: Docs\ROCK\docs\2026-07-26-grab-pose-objective-fit.md.
 *
 * Architecture: the ONE-SIDED objective terms define the valid-grasp manifold
 * (contact is a CONSTRAINT - first-contact-stop - never an inward pull), and
 * an arrival-continuity regularizer picks the nearest manifold point from the
 * arrival pose. Offline validation: ident-start drift median 6.3deg/0.75gu,
 * 90% of realistic-scatter starts converge contact-equivalent to the label.
 *
 * Purity contract (enforced by tests/GrabPoseObjectiveSourceTests.ps1):
 * no logging, no globals, no RNG (the solve is a fixed-schedule pattern
 * search: reproducible seats, diagnosable bugs), no engine calls. Everything
 * is evaluated in proxy-local space with the hand fixed and the object pose
 * moving, exactly like the offline harness this must stay in lockstep with
 * (scratchpad fit.py/opt.py; fixture parity tests pin the term values).
 *
 * The solve runs ONCE at grab acquisition, never per-frame.
 */
namespace rock::grab_pose_objective
{
    // ---- pose delta ---------------------------------------------------------

    /*
     * Row-major rotation applied as v' = M * v (rows are output basis rows),
     * matching the offline harness' matvec. Kept as a plain struct instead of
     * RE::NiMatrix3 so the math here has exactly one convention.
     */
    struct Mat33
    {
        float m[3][3]{ { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f } };
    };

    // Rigid delta about the shape model's centroid: v' = M * (v - c) + c + t.
    struct PoseDelta
    {
        Mat33 rotate{};
        RE::NiPoint3 translate{};
    };

    [[nodiscard]] inline RE::NiPoint3 rotatePoint(const Mat33& m, const RE::NiPoint3& v)
    {
        return RE::NiPoint3{
            m.m[0][0] * v.x + m.m[0][1] * v.y + m.m[0][2] * v.z,
            m.m[1][0] * v.x + m.m[1][1] * v.y + m.m[1][2] * v.z,
            m.m[2][0] * v.x + m.m[2][1] * v.y + m.m[2][2] * v.z,
        };
    }

    [[nodiscard]] inline Mat33 multiply(const Mat33& a, const Mat33& b)
    {
        Mat33 out{};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                out.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
            }
        }
        return out;
    }

    [[nodiscard]] inline Mat33 axisAngle(const RE::NiPoint3& axisIn, float radians)
    {
        const float lengthSquared = axisIn.x * axisIn.x + axisIn.y * axisIn.y + axisIn.z * axisIn.z;
        if (!(lengthSquared > 1.0e-12f)) {
            return Mat33{};
        }
        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        const float x = axisIn.x * inverseLength;
        const float y = axisIn.y * inverseLength;
        const float z = axisIn.z * inverseLength;
        const float c = std::cos(radians);
        const float s = std::sin(radians);
        const float t = 1.0f - c;
        Mat33 out{};
        out.m[0][0] = t * x * x + c;
        out.m[0][1] = t * x * y - s * z;
        out.m[0][2] = t * x * z + s * y;
        out.m[1][0] = t * x * y + s * z;
        out.m[1][1] = t * y * y + c;
        out.m[1][2] = t * y * z - s * x;
        out.m[2][0] = t * x * z - s * y;
        out.m[2][1] = t * y * z + s * x;
        out.m[2][2] = t * z * z + c;
        return out;
    }

    [[nodiscard]] inline float rotationAngleRadians(const Mat33& m)
    {
        const float trace = m.m[0][0] + m.m[1][1] + m.m[2][2];
        const float clamped = (std::max)(-1.0f, (std::min)(1.0f, (trace - 1.0f) * 0.5f));
        return std::acos(clamped);
    }

    // ---- geometry primitives ------------------------------------------------

    namespace detail
    {
        [[nodiscard]] inline RE::NiPoint3 subtract(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            return RE::NiPoint3{ a.x - b.x, a.y - b.y, a.z - b.z };
        }

        [[nodiscard]] inline float dot(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        [[nodiscard]] inline RE::NiPoint3 crossProduct(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            return RE::NiPoint3{
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x,
            };
        }

        [[nodiscard]] inline float lengthOf(const RE::NiPoint3& a)
        {
            return std::sqrt((std::max)(0.0f, dot(a, a)));
        }

        [[nodiscard]] inline RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& a)
        {
            const float length = lengthOf(a);
            if (!(length > 1.0e-9f)) {
                return RE::NiPoint3{};
            }
            return RE::NiPoint3{ a.x / length, a.y / length, a.z / length };
        }

        // Ericson closest-point-on-triangle; mirrors the offline geom.py so
        // fixture goldens stay bit-comparable within float tolerance.
        [[nodiscard]] inline RE::NiPoint3 closestPointOnTriangle(
            const RE::NiPoint3& p,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const RE::NiPoint3& c)
        {
            const RE::NiPoint3 ab = subtract(b, a);
            const RE::NiPoint3 ac = subtract(c, a);
            const RE::NiPoint3 ap = subtract(p, a);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) {
                return a;
            }
            const RE::NiPoint3 bp = subtract(p, b);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) {
                return b;
            }
            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
                const float denom = d1 - d3;
                const float v = denom != 0.0f ? d1 / denom : 0.0f;
                return RE::NiPoint3{ a.x + ab.x * v, a.y + ab.y * v, a.z + ab.z * v };
            }
            const RE::NiPoint3 cp = subtract(p, c);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) {
                return c;
            }
            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
                const float denom = d2 - d6;
                const float w = denom != 0.0f ? d2 / denom : 0.0f;
                return RE::NiPoint3{ a.x + ac.x * w, a.y + ac.y * w, a.z + ac.z * w };
            }
            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
                const float denom = (d4 - d3) + (d5 - d6);
                const float w = denom != 0.0f ? (d4 - d3) / denom : 0.0f;
                const RE::NiPoint3 bc = subtract(c, b);
                return RE::NiPoint3{ b.x + bc.x * w, b.y + bc.y * w, b.z + bc.z * w };
            }
            const float denom = va + vb + vc;
            if (denom == 0.0f) {
                return a;
            }
            const float v = vb / denom;
            const float w = vc / denom;
            return RE::NiPoint3{
                a.x + ab.x * v + ac.x * w,
                a.y + ab.y * v + ac.y * w,
                a.z + ab.z * v + ac.z * w,
            };
        }

        [[nodiscard]] inline float pointTriangleDistance(
            const RE::NiPoint3& p,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const RE::NiPoint3& c)
        {
            return lengthOf(subtract(p, closestPointOnTriangle(p, a, b, c)));
        }

        [[nodiscard]] inline float pointTriangleDistanceSquared(
            const RE::NiPoint3& p,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const RE::NiPoint3& c)
        {
            const RE::NiPoint3 d = subtract(p, closestPointOnTriangle(p, a, b, c));
            return dot(d, d);
        }

        inline void closestSegmentSegment(
            const RE::NiPoint3& p1,
            const RE::NiPoint3& q1,
            const RE::NiPoint3& p2,
            const RE::NiPoint3& q2,
            RE::NiPoint3& outC1,
            RE::NiPoint3& outC2)
        {
            const RE::NiPoint3 d1 = subtract(q1, p1);
            const RE::NiPoint3 d2 = subtract(q2, p2);
            const RE::NiPoint3 r = subtract(p1, p2);
            const float a = dot(d1, d1);
            const float e = dot(d2, d2);
            const float f = dot(d2, r);
            float s = 0.0f;
            float t = 0.0f;
            if (a <= 1.0e-9f && e <= 1.0e-9f) {
                outC1 = p1;
                outC2 = p2;
                return;
            }
            if (a <= 1.0e-9f) {
                t = (std::max)(0.0f, (std::min)(1.0f, f / e));
            } else {
                const float c = dot(d1, r);
                if (e <= 1.0e-9f) {
                    s = (std::max)(0.0f, (std::min)(1.0f, -c / a));
                } else {
                    const float b = dot(d1, d2);
                    const float denom = a * e - b * b;
                    s = denom != 0.0f ? (std::max)(0.0f, (std::min)(1.0f, (b * f - c * e) / denom)) : 0.0f;
                    t = (b * s + f) / e;
                    if (t < 0.0f) {
                        t = 0.0f;
                        s = (std::max)(0.0f, (std::min)(1.0f, -c / a));
                    } else if (t > 1.0f) {
                        t = 1.0f;
                        s = (std::max)(0.0f, (std::min)(1.0f, (b - c) / a));
                    }
                }
            }
            outC1 = RE::NiPoint3{ p1.x + d1.x * s, p1.y + d1.y * s, p1.z + d1.z * s };
            outC2 = RE::NiPoint3{ p2.x + d2.x * t, p2.y + d2.y * t, p2.z + d2.z * t };
        }

        // Squared-space form: the solve calls this thousands of times per grab
        // and only needs ONE sqrt per admitted pair, not five.
        [[nodiscard]] inline float segmentTriangleDistanceSquared(
            const RE::NiPoint3& p,
            const RE::NiPoint3& q,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const RE::NiPoint3& c)
        {
            float best = (std::min)(pointTriangleDistanceSquared(p, a, b, c), pointTriangleDistanceSquared(q, a, b, c));
            const RE::NiPoint3 edges[3][2]{ { a, b }, { b, c }, { c, a } };
            for (const auto& edge : edges) {
                RE::NiPoint3 c1{};
                RE::NiPoint3 c2{};
                closestSegmentSegment(p, q, edge[0], edge[1], c1, c2);
                const RE::NiPoint3 d = subtract(c1, c2);
                best = (std::min)(best, dot(d, d));
            }
            return best;
        }

        [[nodiscard]] inline float segmentTriangleDistance(
            const RE::NiPoint3& p,
            const RE::NiPoint3& q,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const RE::NiPoint3& c)
        {
            return std::sqrt((std::max)(0.0f, segmentTriangleDistanceSquared(p, q, a, b, c)));
        }
    }

    // ---- shape model --------------------------------------------------------

    enum class ShapeClass
    {
        Compact,
        Rod,
        Plate,
    };

    struct ShapeClassifyThresholds
    {
        // Same gates the seat machinery uses (rockPullPresentationMinElongationRatio,
        // rockGrabSeatRollMinSecondElongationRatio); passed in so this header
        // stays pure of global config.
        float rodMinElongationRatio = 2.0f;
        float plateMinSecondElongationRatio = 1.25f;
    };

    struct GraspShapeModel
    {
        bool valid = false;
        ShapeClass shape = ShapeClass::Compact;
        RE::NiPoint3 centroid{};
        std::array<RE::NiPoint3, 3> axes{};  // principal axes, descending eigenvalue
        std::array<float, 3> eigenvalues{};
        float elongationRatio = 1.0f;
        float secondElongationRatio = 1.0f;
        float girthProfileMinGameUnits = 0.0f;  // rod only: thinnest local radius along the long axis
        /*
         * Per-triangle bounding radius about the triangle centroid, computed
         * ONCE here: rigid pose deltas preserve distances, so the solve never
         * recomputes them per evaluation (that redundancy cost a full-frame
         * freeze per grab before it was hoisted). Inflated by a hair so the
         * conservative reject can only keep MORE triangles than the exact
         * bound - identical distances, never a missed nearest triangle.
         */
        std::vector<float> triangleBoundRadii;
    };

    /*
     * Runtime triangle density: the solve cost is O(evaluations x capsules x
     * triangles) on the grab commit path, and for hand-sized objects nearly
     * every capsule-triangle pair is genuinely close (pruning cannot help),
     * so the density IS the frame budget. The objective was fitted at 260;
     * shipping at 130 was revalidated with the offline harness (same
     * convergence quality) after 260 measured ~2x over frame budget. Callers
     * must not feed denser meshes.
     */
    inline constexpr std::size_t kGrabPoseObjectiveMaxTriangles = 130;

    struct ObjectiveConstants
    {
        // Load-bearing measured constants; provenance in the fit doc.
        float palmSlackGameUnits = 2.5f;          // surface may float this far off the palm before palmProx engages
        float penetrationLimitGameUnits = 1.5f;   // measured collider-inflation ceiling across 63 holds
        float wrapClampGameUnits = 4.0f;          // fingertip gap contribution saturates here
        float girthWindowGameUnits = 3.0f;        // slab half-width along the rod axis, centred on the palm
        float girthWrappableRadiusGameUnits = 3.0f;   // local radius a hand can power-grip
        float girthMinTolGameUnits = 0.5f;        // slack over the object's own thinnest station
        /*
         * behindPalm: an inside-grab is BURIED - mesh on the far side of the
         * palm plane within a hand-sized footprint. Every verified hold reads
         * <= 1.8gu there (curvature wrap), so beyond the tolerance this is a
         * constraint violation, not a preference. Signed nearest-triangle
         * tests were measured and rejected: 28/66 verified holds have tips
         * legitimately tucked into concavities that read 'inside'.
         */
        float behindPalmFootprintRadiusGameUnits = 6.0f;
        float behindPalmToleranceGameUnits = 2.0f;
    };

    /*
     * Fitted 2026-08-01 against 63 verified holds (round 3, one-sided terms).
     * contacts/oppose/enclose fitted to ~zero across two independent rounds
     * and are deliberately NOT ported - dead terms stay deleted, not dormant.
     */
    struct ObjectiveWeights
    {
        float touch = 145.2505f;
        // CONSTRAINT weight, not fitted: behindPalm is zero on every verified
        // hold by construction, so no fit can estimate it - it only exists to
        // make buried/inside poses lose to any outside pose.
        float behindPalm = 150.0f;
        float palmProx = 12.9162f;
        float overPen = 2.7735f;
        float wrap = 1.5205f;
        float rodAxis = 14.9627f;
        float girth = 1.0850f;
    };

    struct HandCapsule
    {
        RE::NiPoint3 a{};
        RE::NiPoint3 b{};
        float radius = 0.0f;
    };

    struct HandVolumeModel
    {
        bool valid = false;
        std::vector<HandCapsule> capsules;               // every driven segment collider
        RE::NiPoint3 palmCenter{};
        float palmRadius = 0.0f;
        std::array<RE::NiPoint3, 5> tipCenters{};
        std::array<float, 5> tipRadii{};
        std::size_t tipCount = 0;
        RE::NiPoint3 palmNormal{};                       // oriented toward the object at build time
    };

    /*
     * Palm normal from the finger-base frame, mirroring the offline harness
     * (score.py): fingers axis = mean(finger bases) - palm heel, cross-palm =
     * index base - pinky base orthogonalised against it, normal = their cross
     * oriented toward the object centroid. This is the frame the weights were
     * fitted in; do not substitute the PalmFace frame rows.
     */
    [[nodiscard]] inline RE::NiPoint3 derivePalmNormal(
        const RE::NiPoint3& indexBase,
        const RE::NiPoint3& middleBase,
        const RE::NiPoint3& ringBase,
        const RE::NiPoint3& pinkyBase,
        const RE::NiPoint3& palmHeel,
        const RE::NiPoint3& palmFaceCenter,
        const RE::NiPoint3& objectCentroid)
    {
        using namespace detail;
        const RE::NiPoint3 basesMean{
            (indexBase.x + middleBase.x + ringBase.x + pinkyBase.x) * 0.25f,
            (indexBase.y + middleBase.y + ringBase.y + pinkyBase.y) * 0.25f,
            (indexBase.z + middleBase.z + ringBase.z + pinkyBase.z) * 0.25f,
        };
        const RE::NiPoint3 fingers = normalizeOrZero(subtract(basesMean, palmHeel));
        RE::NiPoint3 crossPalm = normalizeOrZero(subtract(indexBase, pinkyBase));
        const float projection = dot(crossPalm, fingers);
        crossPalm = normalizeOrZero(RE::NiPoint3{
            crossPalm.x - fingers.x * projection,
            crossPalm.y - fingers.y * projection,
            crossPalm.z - fingers.z * projection,
        });
        RE::NiPoint3 normal = normalizeOrZero(crossProduct(fingers, crossPalm));
        if (dot(normal, subtract(objectCentroid, palmFaceCenter)) < 0.0f) {
            normal = RE::NiPoint3{ -normal.x, -normal.y, -normal.z };
        }
        return normal;
    }

    /*
     * Area-weighted vertex PCA over the cached mesh (double accumulators:
     * 260-triangle sums drift visibly in float), Jacobi eigendecomposition,
     * axes sorted by descending eigenvalue. Mirrors the offline analyze.py
     * pipeline the ground-truth statistics came from.
     *
     * model.centroid is deliberately the UNWEIGHTED vertex mean, not the PCA's
     * area-weighted mean: it is the pose-delta pivot and the girth station
     * origin, and the offline validation that produced the shipped constants
     * measured rotation/translation deltas about that convention. The
     * area-weighted mean stays internal to the covariance.
     */
    [[nodiscard]] inline GraspShapeModel buildShapeModel(
        std::span<const GrabLocalTriangle> triangles,
        const ObjectiveConstants& constants = {},
        const ShapeClassifyThresholds& thresholds = {})
    {
        using namespace detail;
        GraspShapeModel model{};
        if (triangles.empty()) {
            return model;
        }

        double vertexMean[3]{};
        double weightSum = 0.0;
        double mean[3]{};
        for (const auto& triangle : triangles) {
            const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
            for (const auto& vertex : vertices) {
                vertexMean[0] += vertex.x;
                vertexMean[1] += vertex.y;
                vertexMean[2] += vertex.z;
            }
            const RE::NiPoint3 e0 = subtract(triangle.v1, triangle.v0);
            const RE::NiPoint3 e1 = subtract(triangle.v2, triangle.v0);
            const RE::NiPoint3 cross = crossProduct(e0, e1);
            const double area = 0.5 * std::sqrt((std::max)(0.0f, dot(cross, cross)));
            if (!(area > 1.0e-9)) {
                continue;
            }
            weightSum += area;
            mean[0] += area * (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0;
            mean[1] += area * (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0;
            mean[2] += area * (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0;
        }
        const double vertexCount = static_cast<double>(triangles.size() * 3);
        vertexMean[0] /= vertexCount;
        vertexMean[1] /= vertexCount;
        vertexMean[2] /= vertexCount;
        if (!(weightSum > 1.0e-9)) {
            return model;
        }
        mean[0] /= weightSum;
        mean[1] /= weightSum;
        mean[2] /= weightSum;

        double cov[3][3]{};
        for (const auto& triangle : triangles) {
            const RE::NiPoint3 e0 = subtract(triangle.v1, triangle.v0);
            const RE::NiPoint3 e1 = subtract(triangle.v2, triangle.v0);
            const RE::NiPoint3 cross = crossProduct(e0, e1);
            const double area = 0.5 * std::sqrt((std::max)(0.0f, dot(cross, cross)));
            if (!(area > 1.0e-9)) {
                continue;
            }
            const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
            for (const auto& vertex : vertices) {
                const double d[3]{ vertex.x - mean[0], vertex.y - mean[1], vertex.z - mean[2] };
                for (int i = 0; i < 3; ++i) {
                    for (int j = 0; j < 3; ++j) {
                        cov[i][j] += (area / 3.0) * d[i] * d[j];
                    }
                }
            }
        }
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                cov[i][j] /= weightSum;
            }
        }

        // Jacobi rotations on the symmetric 3x3.
        double a[3][3];
        double v[3][3]{ { 1.0, 0.0, 0.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 } };
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                a[i][j] = cov[i][j];
            }
        }
        for (int sweep = 0; sweep < 64; ++sweep) {
            int p = 0;
            int q = 1;
            double off = 0.0;
            for (int i = 0; i < 3; ++i) {
                for (int j = i + 1; j < 3; ++j) {
                    if (std::abs(a[i][j]) > off) {
                        off = std::abs(a[i][j]);
                        p = i;
                        q = j;
                    }
                }
            }
            if (off < 1.0e-12) {
                break;
            }
            const double theta = (a[q][q] - a[p][p]) / (2.0 * a[p][q]);
            const double t = (theta >= 0.0 ? 1.0 : -1.0) / (std::abs(theta) + std::sqrt(theta * theta + 1.0));
            const double c = 1.0 / std::sqrt(t * t + 1.0);
            const double s = t * c;
            for (int k = 0; k < 3; ++k) {
                const double akp = a[k][p];
                const double akq = a[k][q];
                a[k][p] = c * akp - s * akq;
                a[k][q] = s * akp + c * akq;
            }
            for (int k = 0; k < 3; ++k) {
                const double apk = a[p][k];
                const double aqk = a[q][k];
                a[p][k] = c * apk - s * aqk;
                a[q][k] = s * apk + c * aqk;
            }
            for (int k = 0; k < 3; ++k) {
                const double vkp = v[k][p];
                const double vkq = v[k][q];
                v[k][p] = c * vkp - s * vkq;
                v[k][q] = s * vkp + c * vkq;
            }
        }

        int order[3]{ 0, 1, 2 };
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                if (a[order[j]][order[j]] > a[order[i]][order[i]]) {
                    const int swap = order[i];
                    order[i] = order[j];
                    order[j] = swap;
                }
            }
        }

        model.valid = true;
        model.centroid = RE::NiPoint3{
            static_cast<float>(vertexMean[0]), static_cast<float>(vertexMean[1]), static_cast<float>(vertexMean[2])
        };
        for (int i = 0; i < 3; ++i) {
            const int k = order[i];
            model.eigenvalues[static_cast<std::size_t>(i)] = static_cast<float>((std::max)(0.0, a[k][k]));
            model.axes[static_cast<std::size_t>(i)] = RE::NiPoint3{
                static_cast<float>(v[0][k]), static_cast<float>(v[1][k]), static_cast<float>(v[2][k])
            };
        }
        const float l1 = model.eigenvalues[0];
        const float l2 = model.eigenvalues[1];
        const float l3 = model.eigenvalues[2];
        model.elongationRatio = std::sqrt(l1 / (std::max)(l2, l1 * 1.0e-4f));
        model.secondElongationRatio = std::sqrt(l2 / (std::max)(l3, l2 * 1.0e-4f));
        const bool rod = model.elongationRatio >= thresholds.rodMinElongationRatio;
        const bool flatFace = model.secondElongationRatio >= thresholds.plateMinSecondElongationRatio;
        model.shape = rod ? ShapeClass::Rod : (flatFace ? ShapeClass::Plate : ShapeClass::Compact);

        if (model.shape == ShapeClass::Rod) {
            /*
             * Thinnest local radius along the long axis (pose-invariant).
             * Absolute girth thresholds fail: PCA rods include chunky boxes
             * whose radial max is corner distance. Penalising girth only
             * beyond BOTH the wrappable limit AND this per-object minimum
             * means a broom head (thin handle available) is penalised while a
             * uniformly chunky carton is not.
             */
            const RE::NiPoint3 axis = model.axes[0];
            float lo = 0.0f;
            float hi = 0.0f;
            bool first = true;
            for (const auto& triangle : triangles) {
                const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
                for (const auto& vertex : vertices) {
                    const float station = dot(detail::subtract(vertex, model.centroid), axis);
                    if (first) {
                        lo = hi = station;
                        first = false;
                    } else {
                        lo = (std::min)(lo, station);
                        hi = (std::max)(hi, station);
                    }
                }
            }
            float best = 0.0f;
            bool found = false;
            for (float station = lo; station <= hi; station += 1.5f) {
                float slabMax = 0.0f;
                int slabCount = 0;
                for (const auto& triangle : triangles) {
                    const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
                    for (const auto& vertex : vertices) {
                        const RE::NiPoint3 fromCentroid = detail::subtract(vertex, model.centroid);
                        const float s = dot(fromCentroid, axis);
                        if (std::abs(s - station) <= constants.girthWindowGameUnits) {
                            const RE::NiPoint3 radial{
                                fromCentroid.x - axis.x * s,
                                fromCentroid.y - axis.y * s,
                                fromCentroid.z - axis.z * s,
                            };
                            slabMax = (std::max)(slabMax, detail::lengthOf(radial));
                            ++slabCount;
                        }
                    }
                }
                if (slabCount >= 4) {  // skip sparse taper bins at the ends
                    if (!found || slabMax < best) {
                        best = slabMax;
                        found = true;
                    }
                }
            }
            model.girthProfileMinGameUnits = found ? best : 0.0f;
        }

        model.triangleBoundRadii.reserve(triangles.size());
        for (const auto& triangle : triangles) {
            const RE::NiPoint3 center{
                (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0f,
                (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0f,
                (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0f,
            };
            const float radius = (std::max)(
                lengthOf(subtract(triangle.v0, center)),
                (std::max)(lengthOf(subtract(triangle.v1, center)), lengthOf(subtract(triangle.v2, center))));
            model.triangleBoundRadii.push_back(radius * 1.001f + 0.001f);
        }
        return model;
    }

    // ---- objective ----------------------------------------------------------

    struct TermValues
    {
        float touch = 0.0f;
        float behindPalm = 0.0f;
        float palmProx = 0.0f;
        float overPen = 0.0f;
        float wrap = 0.0f;
        float rodAxis = 0.0f;
        float girth = 0.0f;

        [[nodiscard]] float weightedTotal(const ObjectiveWeights& weights) const
        {
            return weights.touch * touch + weights.behindPalm * behindPalm +
                weights.palmProx * palmProx + weights.overPen * overPen +
                weights.wrap * wrap + weights.rodAxis * rodAxis + weights.girth * girth;
        }
    };

    /*
     * Evaluation context: everything about the OBJECT that is invariant under
     * a rigid pose delta, computed once per solve. Evaluations transform the
     * HAND by the inverse pose instead of transforming the whole mesh -
     * distances are rigid-invariant, so the term values are identical while
     * the per-evaluation cost drops from O(vertices) transforms to ~45. This
     * plus squared-distance rejects is what keeps the solve inside a frame
     * on the grab commit path (the naive form froze the game per grab).
     */
    struct EvaluationContext
    {
        std::span<const GrabLocalTriangle> triangles;
        std::vector<RE::NiPoint3> centers;      // triangle centroids, object frame
        std::vector<float> boundRadii;          // conservative bounding radii
        std::vector<float> girthStations;       // per vertex: dot(v - centroid, axis0)
        std::vector<float> girthRadials;        // per vertex: radial distance from axis0
    };

    [[nodiscard]] inline EvaluationContext buildEvaluationContext(
        std::span<const GrabLocalTriangle> triangles,
        const GraspShapeModel& model)
    {
        using namespace detail;
        EvaluationContext context{};
        context.triangles = triangles;
        const std::size_t triangleCount = triangles.size();
        context.centers.reserve(triangleCount);
        context.boundRadii.reserve(triangleCount);
        const bool haveModelBounds = model.triangleBoundRadii.size() == triangleCount;
        for (std::size_t i = 0; i < triangleCount; ++i) {
            const auto& triangle = triangles[i];
            const RE::NiPoint3 center{
                (triangle.v0.x + triangle.v1.x + triangle.v2.x) / 3.0f,
                (triangle.v0.y + triangle.v1.y + triangle.v2.y) / 3.0f,
                (triangle.v0.z + triangle.v1.z + triangle.v2.z) / 3.0f,
            };
            context.centers.push_back(center);
            context.boundRadii.push_back(haveModelBounds ? model.triangleBoundRadii[i]
                                                         : (std::max)(lengthOf(subtract(triangle.v0, center)),
                                                               (std::max)(lengthOf(subtract(triangle.v1, center)),
                                                                   lengthOf(subtract(triangle.v2, center)))));
        }
        if (model.shape == ShapeClass::Rod) {
            const RE::NiPoint3 axis = model.axes[0];
            context.girthStations.reserve(triangleCount * 3);
            context.girthRadials.reserve(triangleCount * 3);
            for (const auto& triangle : triangles) {
                const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
                for (const auto& vertex : vertices) {
                    const RE::NiPoint3 fromCentroid = subtract(vertex, model.centroid);
                    const float station = dot(fromCentroid, axis);
                    const RE::NiPoint3 radial{
                        fromCentroid.x - axis.x * station,
                        fromCentroid.y - axis.y * station,
                        fromCentroid.z - axis.z * station,
                    };
                    context.girthStations.push_back(station);
                    context.girthRadials.push_back(lengthOf(radial));
                }
            }
        }
        return context;
    }

    /*
     * Evaluate the one-sided terms for the object at `pose` (a rigid delta
     * about the shape model centroid). Internally the HAND is carried into
     * the object frame by the inverse delta; the offline sentinel convention
     * is kept: a capsule whose bounding reject filters every triangle reports
     * distance 1e9, so `touch` saturates rather than losing the term - the
     * solver relies on the seed being an arrival pose already near the hand.
     */
    [[nodiscard]] inline TermValues scorePoseWithContext(
        const EvaluationContext& context,
        const GraspShapeModel& model,
        const HandVolumeModel& hand,
        const PoseDelta& pose,
        const ObjectiveConstants& constants = {})
    {
        using namespace detail;
        TermValues terms{};
        if (!model.valid || !hand.valid || context.triangles.empty()) {
            return terms;
        }
        const std::size_t triangleCount = context.triangles.size();
        const RE::NiPoint3 c = model.centroid;
        // Inverse of the rigid delta v2 = R(v-c)+c+t applied to hand points:
        // h_obj = R^T (h - c - t) + c. Rotation transpose == inverse.
        const auto inversePoint = [&](const RE::NiPoint3& p) {
            const RE::NiPoint3 shifted{
                p.x - c.x - pose.translate.x,
                p.y - c.y - pose.translate.y,
                p.z - c.z - pose.translate.z,
            };
            return RE::NiPoint3{
                pose.rotate.m[0][0] * shifted.x + pose.rotate.m[1][0] * shifted.y + pose.rotate.m[2][0] * shifted.z + c.x,
                pose.rotate.m[0][1] * shifted.x + pose.rotate.m[1][1] * shifted.y + pose.rotate.m[2][1] * shifted.z + c.y,
                pose.rotate.m[0][2] * shifted.x + pose.rotate.m[1][2] * shifted.y + pose.rotate.m[2][2] * shifted.z + c.z,
            };
        };
        const auto inverseVector = [&](const RE::NiPoint3& v) {
            return RE::NiPoint3{
                pose.rotate.m[0][0] * v.x + pose.rotate.m[1][0] * v.y + pose.rotate.m[2][0] * v.z,
                pose.rotate.m[0][1] * v.x + pose.rotate.m[1][1] * v.y + pose.rotate.m[2][1] * v.z,
                pose.rotate.m[0][2] * v.x + pose.rotate.m[1][2] * v.y + pose.rotate.m[2][2] * v.z,
            };
        };
        const auto distanceSquared = [](const RE::NiPoint3& a, const RE::NiPoint3& b) {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return dx * dx + dy * dy + dz * dz;
        };

        // Exact branch-and-bound nearest-surface query, fully in squared
        // space: a triangle whose bounding sphere cannot beat the current
        // best is skipped without changing the result, and the winner costs
        // exactly one sqrt at the end.
        const auto nearestSurfaceDistance = [&](const RE::NiPoint3& point) {
            float bestSquared = 1.0e18f;
            float best = 1.0e9f;
            for (std::size_t i = 0; i < triangleCount; ++i) {
                const float admit = best + context.boundRadii[i];
                if (distanceSquared(context.centers[i], point) >= admit * admit) {
                    continue;
                }
                const auto& triangle = context.triangles[i];
                const float candidate =
                    pointTriangleDistanceSquared(point, triangle.v0, triangle.v1, triangle.v2);
                if (candidate < bestSquared) {
                    bestSquared = candidate;
                    best = std::sqrt((std::max)(0.0f, candidate));
                }
            }
            return best;
        };

        const RE::NiPoint3 palmObject = inversePoint(hand.palmCenter);
        const RE::NiPoint3 normalObject = inverseVector(hand.palmNormal);
        const float palmGap = nearestSurfaceDistance(palmObject) - hand.palmRadius;

        // Depth of mesh past the palm plane (opposite side from the object,
        // normal points TOWARD it), inside the lateral footprint. O(vertices),
        // no kernels - cheap enough to run every evaluation.
        float behindPalmDepth = 0.0f;
        for (const auto& triangle : context.triangles) {
            const RE::NiPoint3 vertices[3]{ triangle.v0, triangle.v1, triangle.v2 };
            for (const auto& vertex : vertices) {
                const RE::NiPoint3 rel = subtract(vertex, palmObject);
                const float axial = dot(rel, normalObject);
                if (axial >= 0.0f) {
                    continue;
                }
                const RE::NiPoint3 lateral{
                    rel.x - normalObject.x * axial,
                    rel.y - normalObject.y * axial,
                    rel.z - normalObject.z * axial,
                };
                if (dot(lateral, lateral) <=
                    constants.behindPalmFootprintRadiusGameUnits * constants.behindPalmFootprintRadiusGameUnits) {
                    behindPalmDepth = (std::max)(behindPalmDepth, -axial);
                }
            }
        }

        /*
         * touch needs the GLOBAL minimum capsule gap and overPen needs any
         * penetrating pair, so a pair only deserves the expensive
         * segment-triangle test when its optimistic gap (point-to-axis
         * distance minus triangle bound minus radius) could still penetrate
         * (< 0) or improve the champion gap. Exact branch-and-bound - the
         * loose reach-based reject admitted nearly every pair on
         * large-triangle meshes and put the whole solve over frame budget.
         */
        float worstPenetration = 0.0f;
        float minGap = 1.0e9f;
        for (const auto& capsule : hand.capsules) {
            const RE::NiPoint3 a = inversePoint(capsule.a);
            const RE::NiPoint3 b = inversePoint(capsule.b);
            const RE::NiPoint3 axis = subtract(b, a);
            const float axisLengthSquared = dot(axis, axis);
            // 'nearDistance', not 'near': windows.h defines near/far as empty
            // legacy macros and silently deletes the identifier.
            float nearDistance = 1.0e9f;
            for (std::size_t i = 0; i < triangleCount; ++i) {
                // Point-to-segment distance from the triangle center to the
                // capsule axis: the tightest cheap lower bound available,
                // compared in squared space (no sqrt on the reject path).
                const RE::NiPoint3 toCenter = subtract(context.centers[i], a);
                float t = axisLengthSquared > 1.0e-9f ? dot(toCenter, axis) / axisLengthSquared : 0.0f;
                t = (std::max)(0.0f, (std::min)(1.0f, t));
                const RE::NiPoint3 onAxis{ a.x + axis.x * t, a.y + axis.y * t, a.z + axis.z * t };
                const float axisDistanceSquared = distanceSquared(context.centers[i], onAxis);
                const float championGap = (std::max)((std::min)(minGap, nearDistance - capsule.radius), 0.0f);
                const float admit = championGap + context.boundRadii[i] + capsule.radius;
                if (axisDistanceSquared >= admit * admit) {
                    continue;
                }
                const auto& triangle = context.triangles[i];
                const float distance = std::sqrt((std::max)(0.0f,
                    segmentTriangleDistanceSquared(a, b, triangle.v0, triangle.v1, triangle.v2)));
                nearDistance = (std::min)(nearDistance, distance);
                worstPenetration = (std::max)(worstPenetration, capsule.radius - distance);
            }
            minGap = (std::min)(minGap, nearDistance - capsule.radius);
        }

        float wrapSum = 0.0f;
        for (std::size_t tip = 0; tip < hand.tipCount; ++tip) {
            const float best = nearestSurfaceDistance(inversePoint(hand.tipCenters[tip]));
            const float gap = (std::min)((std::max)(best - hand.tipRadii[tip], 0.0f), constants.wrapClampGameUnits);
            wrapSum += gap * gap;
        }

        const float touchRaw = (std::max)(0.0f, minGap);
        terms.touch = touchRaw * touchRaw;
        const float behindRaw = (std::max)(0.0f, behindPalmDepth - constants.behindPalmToleranceGameUnits);
        terms.behindPalm = behindRaw * behindRaw;
        const float palmRaw = (std::max)(0.0f, palmGap - constants.palmSlackGameUnits);
        terms.palmProx = palmRaw * palmRaw;
        const float overRaw = (std::max)(0.0f, worstPenetration - constants.penetrationLimitGameUnits);
        terms.overPen = overRaw * overRaw;
        terms.wrap = hand.tipCount > 0 ? wrapSum / static_cast<float>(hand.tipCount) : 0.0f;

        if (model.shape == ShapeClass::Rod) {
            // dot(R*axis, n) == dot(axis, R^T n): evaluate in the object frame.
            const float alignment = dot(normalizeOrZero(model.axes[0]), normalObject);
            terms.rodAxis = alignment * alignment;

            const float palmStation = dot(subtract(palmObject, c), model.axes[0]);
            float slabMax = 0.0f;
            for (std::size_t i = 0; i < context.girthStations.size(); ++i) {
                if (std::abs(context.girthStations[i] - palmStation) <= constants.girthWindowGameUnits) {
                    slabMax = (std::max)(slabMax, context.girthRadials[i]);
                }
            }
            const float threshold = (std::max)(
                constants.girthWrappableRadiusGameUnits,
                model.girthProfileMinGameUnits + constants.girthMinTolGameUnits);
            const float girthRaw = (std::max)(0.0f, slabMax - threshold);
            terms.girth = girthRaw * girthRaw;
        }
        return terms;
    }

    // Convenience form for one-off scoring (parity tests, diagnostics): builds
    // the context per call. The solve builds it once and reuses it.
    [[nodiscard]] inline TermValues scorePose(
        std::span<const GrabLocalTriangle> triangles,
        const GraspShapeModel& model,
        const HandVolumeModel& hand,
        const PoseDelta& pose,
        const ObjectiveConstants& constants = {})
    {
        const EvaluationContext context = buildEvaluationContext(triangles, model);
        return scorePoseWithContext(context, model, hand, pose, constants);
    }

    // ---- deterministic solve ------------------------------------------------

    struct SolveConfig
    {
        // Arrival-continuity regularizer: the solve is MINIMAL CORRECTION from
        // the arrival pose onto the valid-grasp manifold (first-contact-stop
        // as optimization). Deltas are measured from the SEED pose.
        float lambdaTranslatePerGameUnitSq = 0.15f;
        float lambdaRotatePerRadianSq = 6.0f;
        float initialStepDegrees = 8.0f;
        float initialStepGameUnits = 1.0f;
        // 1deg/0.1gu floor: well below the validated ident-drift median
        // (6.9deg/0.69gu), and each halving level costs a full failed round
        // of evaluations on the grab commit path.
        float minStepDegrees = 1.0f;
        float minStepGameUnits = 0.1f;
        int maxIterations = 64;
        /*
         * Translation-only mode for mid-hold re-seats (seated-pivot
         * reacquire): rotating a HELD object is a visible twitch, and the
         * rotation candidates are two thirds of the search cost. The retired
         * depth stop this replaces was translation-only for the same felt
         * reason.
         */
        bool enableRotation = true;
    };

    struct SolveResult
    {
        PoseDelta pose{};       // delta from the seed, about the model centroid
        TermValues terms{};     // objective terms at the solved pose
        float objectiveScore = 0.0f;  // weighted terms, without the regularizer
        float totalScore = 0.0f;      // objective + regularizer cost
        float rotationDegrees = 0.0f;
        float translationGameUnits = 0.0f;
        int iterations = 0;
    };

    /*
     * Fixed-schedule GREEDY pattern search over the 6 pose DOF: probe
     * rotation then translation per axis and sign in a fixed order, accept
     * the FIRST improving move, halve both steps when a full round finds
     * nothing, stop when both steps drop below their minima. First-improvement
     * (vs best-of-12) roughly halves the evaluation count on the grab commit
     * path and was revalidated offline at the shipped triangle density. NO
     * randomness - identical inputs always produce the identical seat. Runs
     * once at grab time.
     */
    [[nodiscard]] inline SolveResult solvePose(
        std::span<const GrabLocalTriangle> triangles,
        const GraspShapeModel& model,
        const HandVolumeModel& hand,
        const ObjectiveWeights& weights = {},
        const ObjectiveConstants& constants = {},
        const SolveConfig& config = {})
    {
        using namespace detail;
        SolveResult result{};
        if (!model.valid || !hand.valid || triangles.empty()) {
            return result;
        }

        // Object-side data is pose-invariant: build it once, evaluate many.
        const EvaluationContext context = buildEvaluationContext(triangles, model);
        const auto evaluate = [&](const PoseDelta& pose) {
            const TermValues terms = scorePoseWithContext(context, model, hand, pose, constants);
            const float angle = rotationAngleRadians(pose.rotate);
            const float translationSq = dot(pose.translate, pose.translate);
            return terms.weightedTotal(weights) +
                config.lambdaRotatePerRadianSq * angle * angle +
                config.lambdaTranslatePerGameUnitSq * translationSq;
        };

        PoseDelta current{};
        float currentScore = evaluate(current);
        float stepRadians = config.initialStepDegrees * 0.01745329252f;
        float stepGameUnits = config.initialStepGameUnits;
        const float minStepRadians = config.minStepDegrees * 0.01745329252f;

        const RE::NiPoint3 kAxes[3]{
            { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, { 0.0f, 0.0f, 1.0f }
        };

        int iteration = 0;
        for (; iteration < config.maxIterations; ++iteration) {
            bool improved = false;
            for (const auto& axis : kAxes) {
                for (const float sign : { 1.0f, -1.0f }) {
                    PoseDelta candidate{};
                    if (config.enableRotation) {
                        const Mat33 delta = axisAngle(axis, sign * stepRadians);
                        candidate.rotate = multiply(delta, current.rotate);
                        candidate.translate = rotatePoint(delta, current.translate);
                        const float score = evaluate(candidate);
                        if (score < currentScore) {
                            current = candidate;
                            currentScore = score;
                            improved = true;
                            break;
                        }
                    }
                    candidate = current;
                    candidate.translate = RE::NiPoint3{
                        current.translate.x + sign * stepGameUnits * axis.x,
                        current.translate.y + sign * stepGameUnits * axis.y,
                        current.translate.z + sign * stepGameUnits * axis.z,
                    };
                    const float translationScore = evaluate(candidate);
                    if (translationScore < currentScore) {
                        current = candidate;
                        currentScore = translationScore;
                        improved = true;
                        break;
                    }
                }
                if (improved) {
                    break;
                }
            }
            if (!improved) {
                stepRadians *= 0.5f;
                stepGameUnits *= 0.5f;
                if (stepRadians < minStepRadians && stepGameUnits < config.minStepGameUnits) {
                    ++iteration;
                    break;
                }
            }
        }

        result.pose = current;
        result.terms = scorePoseWithContext(context, model, hand, current, constants);
        result.objectiveScore = result.terms.weightedTotal(weights);
        result.totalScore = currentScore;
        result.rotationDegrees = rotationAngleRadians(current.rotate) * 57.29577951f;
        result.translationGameUnits = detail::lengthOf(current.translate);
        result.iterations = iteration;
        return result;
    }
}

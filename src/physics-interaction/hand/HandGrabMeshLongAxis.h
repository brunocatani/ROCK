// The long axis of a grabbed object's visual mesh, used to present long objects
// during a pull and to seat them.
#pragma once

#include "physics-interaction/hand/HandGrabInternal.h"

namespace rock::hand_grab_internal
{
    struct GrabMeshLongAxisResult
    {
        RE::NiPoint3 axisWorld{};
        RE::NiPoint3 secondAxisWorld{};
        float elongationRatio = 0.0f;
        float secondElongationRatio = 0.0f;
        std::uint32_t triangleCount = 0;
        const char* reason = "notEvaluated";
        bool valid = false;
    };

    /*
         * Principal axis of the rendered mesh via area-weighted PCA over the
         * extracted world-space triangles. NIF axes are not authored
         * consistently across props, so long-object orientation must come from
         * the geometry itself. elongationRatio = sqrt(lambda1/lambda2), the RMS
         * extent ratio between the dominant and second axis: ~1 for compact
         * objects, >2 for bottle/broom shapes. secondAxisWorld is the second
         * principal axis and secondElongationRatio = sqrt(lambda2/lambda3)
         * (lambda3 recovered from the covariance trace): ~1 for round cross
         * sections where roll about the long axis is meaningless, high for
         * plank/board shapes with a well-defined flat face. Both axis signs
         * are arbitrary; callers must align to the nearest hemisphere of
         * their target axis. Double accumulators because world coordinates
         * are large; covariance is built about the area-weighted mean.
         */
    inline GrabMeshLongAxisResult computeGrabMeshLongAxis(const std::vector<TriangleData>& worldTriangles)
    {
        GrabMeshLongAxisResult result{};
        result.triangleCount = static_cast<std::uint32_t>(worldTriangles.size());
        if (worldTriangles.empty()) {
            result.reason = "noTriangles";
            return result;
        }

        double weightSum = 0.0;
        double meanAccum[3] = {};
        auto isFinitePoint = [](const RE::NiPoint3& p) { return vector_math::hasFiniteComponents(p); };
        auto triangleArea = [](const TriangleData& tri) {
            const RE::NiPoint3 e0 = tri.v1 - tri.v0;
            const RE::NiPoint3 e1 = tri.v2 - tri.v0;
            const RE::NiPoint3 n = vector_math::cross(e0, e1);
            return 0.5f * std::sqrt((std::max)(0.0f, vector_math::lengthSquared(n)));
        };
        for (const auto& tri : worldTriangles) {
            if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                continue;
            }
            const float area = triangleArea(tri);
            if (!std::isfinite(area) || area <= 0.000001f) {
                continue;
            }
            weightSum += area;
            meanAccum[0] += static_cast<double>(area) * (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0;
            meanAccum[1] += static_cast<double>(area) * (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0;
            meanAccum[2] += static_cast<double>(area) * (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
        }
        if (weightSum <= 0.000001) {
            result.reason = "degenerateMeshArea";
            return result;
        }
        const double mean[3] = { meanAccum[0] / weightSum, meanAccum[1] / weightSum, meanAccum[2] / weightSum };

        // Symmetric covariance: [xx, xy, xz, yy, yz, zz]
        double cov[6] = {};
        for (const auto& tri : worldTriangles) {
            if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                continue;
            }
            const float area = triangleArea(tri);
            if (!std::isfinite(area) || area <= 0.000001f) {
                continue;
            }
            const double vertexWeight = static_cast<double>(area) / 3.0;
            const RE::NiPoint3* vertices[3] = { &tri.v0, &tri.v1, &tri.v2 };
            for (const auto* vertex : vertices) {
                const double d[3] = { vertex->x - mean[0], vertex->y - mean[1], vertex->z - mean[2] };
                cov[0] += vertexWeight * d[0] * d[0];
                cov[1] += vertexWeight * d[0] * d[1];
                cov[2] += vertexWeight * d[0] * d[2];
                cov[3] += vertexWeight * d[1] * d[1];
                cov[4] += vertexWeight * d[1] * d[2];
                cov[5] += vertexWeight * d[2] * d[2];
            }
        }
        for (double& entry : cov) {
            entry /= weightSum;
        }

        auto covMultiply = [](const double m[6], const double v[3], double out[3]) {
            out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
            out[1] = m[1] * v[0] + m[3] * v[1] + m[4] * v[2];
            out[2] = m[2] * v[0] + m[4] * v[1] + m[5] * v[2];
        };
        auto dominantEigen = [&covMultiply](const double m[6], double outAxis[3]) {
            // Deterministic start: the coordinate axis with the largest diagonal.
            double v[3] = {};
            if (m[0] >= m[3] && m[0] >= m[5]) {
                v[0] = 1.0;
            } else if (m[3] >= m[5]) {
                v[1] = 1.0;
            } else {
                v[2] = 1.0;
            }
            for (int iteration = 0; iteration < 48; ++iteration) {
                double next[3];
                covMultiply(m, v, next);
                const double lenSq = next[0] * next[0] + next[1] * next[1] + next[2] * next[2];
                if (!(lenSq > 1e-18)) {
                    break;
                }
                const double invLen = 1.0 / std::sqrt(lenSq);
                v[0] = next[0] * invLen;
                v[1] = next[1] * invLen;
                v[2] = next[2] * invLen;
            }
            double mv[3];
            covMultiply(m, v, mv);
            const double eigenvalue = mv[0] * v[0] + mv[1] * v[1] + mv[2] * v[2];
            outAxis[0] = v[0];
            outAxis[1] = v[1];
            outAxis[2] = v[2];
            return eigenvalue;
        };

        double axis1[3];
        const double lambda1 = dominantEigen(cov, axis1);
        if (!(lambda1 > 1e-9)) {
            result.reason = "degenerateCovariance";
            return result;
        }
        double deflated[6] = {
            cov[0] - lambda1 * axis1[0] * axis1[0],
            cov[1] - lambda1 * axis1[0] * axis1[1],
            cov[2] - lambda1 * axis1[0] * axis1[2],
            cov[3] - lambda1 * axis1[1] * axis1[1],
            cov[4] - lambda1 * axis1[1] * axis1[2],
            cov[5] - lambda1 * axis1[2] * axis1[2],
        };
        double axis2[3];
        const double lambda2 = (std::max)(0.0, dominantEigen(deflated, axis2));

        result.axisWorld = RE::NiPoint3{
            static_cast<float>(axis1[0]),
            static_cast<float>(axis1[1]),
            static_cast<float>(axis1[2]),
        };
        result.elongationRatio =
            static_cast<float>((std::min)(100.0, std::sqrt(lambda1 / (std::max)(lambda2, lambda1 * 1e-4))));
        if (lambda2 > 1e-9) {
            // Re-orthogonalize against axis1: deflation leaves numerical drift.
            const double axis12dot = axis2[0] * axis1[0] + axis2[1] * axis1[1] + axis2[2] * axis1[2];
            double axis2Ortho[3] = {
                axis2[0] - axis12dot * axis1[0],
                axis2[1] - axis12dot * axis1[1],
                axis2[2] - axis12dot * axis1[2],
            };
            const double axis2LenSq =
                axis2Ortho[0] * axis2Ortho[0] + axis2Ortho[1] * axis2Ortho[1] + axis2Ortho[2] * axis2Ortho[2];
            if (axis2LenSq > 1e-12) {
                const double invLen = 1.0 / std::sqrt(axis2LenSq);
                result.secondAxisWorld = RE::NiPoint3{
                    static_cast<float>(axis2Ortho[0] * invLen),
                    static_cast<float>(axis2Ortho[1] * invLen),
                    static_cast<float>(axis2Ortho[2] * invLen),
                };
                // Eigenvalues of a symmetric matrix sum to its trace, so
                // lambda3 needs no third power iteration.
                const double trace = cov[0] + cov[3] + cov[5];
                const double lambda3 = (std::max)(0.0, trace - lambda1 - lambda2);
                result.secondElongationRatio =
                    static_cast<float>((std::min)(100.0, std::sqrt(lambda2 / (std::max)(lambda3, lambda2 * 1e-4))));
            }
        }
        result.reason = "meshPrincipalAxis";
        result.valid = true;
        return result;
    }
}

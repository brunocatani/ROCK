#pragma once

/*
 * Exact segment-to-triangle distance on mesh-local triangles. Used by the
 * immersive-aid contact test, which measures hand capsules against the
 * item mesh in its own local space.
 */

#include "physics-interaction/grab/GrabCore.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rock::mesh_segment_distance
{
    namespace detail
    {
        [[nodiscard]] inline RE::NiPoint3 add(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
        }

        [[nodiscard]] inline RE::NiPoint3 subtract(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
        }

        [[nodiscard]] inline RE::NiPoint3 scale(const RE::NiPoint3& value, float scalar)
        {
            return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
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

        [[nodiscard]] inline RE::NiPoint3 closestPointOnTriangle(
            const RE::NiPoint3& point,
            const GrabLocalTriangle& triangle)
        {
            const RE::NiPoint3 ab = subtract(triangle.v1, triangle.v0);
            const RE::NiPoint3 ac = subtract(triangle.v2, triangle.v0);
            const RE::NiPoint3 ap = subtract(point, triangle.v0);
            const float d1 = dot(ab, ap);
            const float d2 = dot(ac, ap);
            if (d1 <= 0.0f && d2 <= 0.0f) {
                return triangle.v0;
            }

            const RE::NiPoint3 bp = subtract(point, triangle.v1);
            const float d3 = dot(ab, bp);
            const float d4 = dot(ac, bp);
            if (d3 >= 0.0f && d4 <= d3) {
                return triangle.v1;
            }

            const float vc = d1 * d4 - d3 * d2;
            if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
                return add(triangle.v0, scale(ab, d1 / (d1 - d3)));
            }

            const RE::NiPoint3 cp = subtract(point, triangle.v2);
            const float d5 = dot(ab, cp);
            const float d6 = dot(ac, cp);
            if (d6 >= 0.0f && d5 <= d6) {
                return triangle.v2;
            }

            const float vb = d5 * d2 - d1 * d6;
            if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
                return add(triangle.v0, scale(ac, d2 / (d2 - d6)));
            }

            const float va = d3 * d6 - d5 * d4;
            if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
                const RE::NiPoint3 bc = subtract(triangle.v2, triangle.v1);
                return add(triangle.v1, scale(bc, (d4 - d3) / ((d4 - d3) + (d5 - d6))));
            }

            const float inverse = 1.0f / (va + vb + vc);
            return add(triangle.v0, add(scale(ab, vb * inverse), scale(ac, vc * inverse)));
        }

        [[nodiscard]] inline float pointTriangleDistanceSquared(
            const RE::NiPoint3& point,
            const GrabLocalTriangle& triangle)
        {
            return lengthSquared(subtract(point, closestPointOnTriangle(point, triangle)));
        }

        [[nodiscard]] inline float pointSegmentDistanceSquared(
            const RE::NiPoint3& point,
            const RE::NiPoint3& a,
            const RE::NiPoint3& b)
        {
            const RE::NiPoint3 axis = subtract(b, a);
            const float axisSquared = lengthSquared(axis);
            const float ratio = axisSquared > 0.000001f ?
                                    std::clamp(dot(subtract(point, a), axis) / axisSquared, 0.0f, 1.0f) :
                                    0.0f;
            return lengthSquared(subtract(point, add(a, scale(axis, ratio))));
        }

        [[nodiscard]] inline float segmentSegmentDistanceSquared(
            const RE::NiPoint3& p1,
            const RE::NiPoint3& q1,
            const RE::NiPoint3& p2,
            const RE::NiPoint3& q2)
        {
            constexpr float epsilon = 0.000001f;
            const RE::NiPoint3 d1 = subtract(q1, p1);
            const RE::NiPoint3 d2 = subtract(q2, p2);
            const RE::NiPoint3 r = subtract(p1, p2);
            const float a = dot(d1, d1);
            const float e = dot(d2, d2);
            const float f = dot(d2, r);
            float s = 0.0f;
            float t = 0.0f;
            if (a <= epsilon && e <= epsilon) {
                return lengthSquared(subtract(p1, p2));
            }
            if (a <= epsilon) {
                t = std::clamp(f / e, 0.0f, 1.0f);
            } else {
                const float c = dot(d1, r);
                if (e <= epsilon) {
                    s = std::clamp(-c / a, 0.0f, 1.0f);
                } else {
                    const float b = dot(d1, d2);
                    const float denominator = a * e - b * b;
                    if (std::abs(denominator) > epsilon) {
                        s = std::clamp((b * f - c * e) / denominator, 0.0f, 1.0f);
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
            const RE::NiPoint3 first = add(p1, scale(d1, s));
            const RE::NiPoint3 second = add(p2, scale(d2, t));
            return lengthSquared(subtract(first, second));
        }

        [[nodiscard]] inline bool segmentIntersectsTriangle(
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const GrabLocalTriangle& triangle)
        {
            constexpr float epsilon = 0.000001f;
            const RE::NiPoint3 direction = subtract(b, a);
            const RE::NiPoint3 edge1 = subtract(triangle.v1, triangle.v0);
            const RE::NiPoint3 edge2 = subtract(triangle.v2, triangle.v0);
            const RE::NiPoint3 p = cross(direction, edge2);
            const float determinant = dot(edge1, p);
            if (std::abs(determinant) <= epsilon) {
                return false;
            }
            const float inverse = 1.0f / determinant;
            const RE::NiPoint3 t = subtract(a, triangle.v0);
            const float u = dot(t, p) * inverse;
            if (u < 0.0f || u > 1.0f) {
                return false;
            }
            const RE::NiPoint3 q = cross(t, edge1);
            const float v = dot(direction, q) * inverse;
            if (v < 0.0f || u + v > 1.0f) {
                return false;
            }
            const float ratio = dot(edge2, q) * inverse;
            return ratio >= 0.0f && ratio <= 1.0f;
        }

        [[nodiscard]] inline float segmentTriangleDistanceSquared(
            const RE::NiPoint3& a,
            const RE::NiPoint3& b,
            const GrabLocalTriangle& triangle)
        {
            if (segmentIntersectsTriangle(a, b, triangle)) {
                return 0.0f;
            }
            float best = (std::min)(pointTriangleDistanceSquared(a, triangle), pointTriangleDistanceSquared(b, triangle));
            best = (std::min)(best, segmentSegmentDistanceSquared(a, b, triangle.v0, triangle.v1));
            best = (std::min)(best, segmentSegmentDistanceSquared(a, b, triangle.v1, triangle.v2));
            return (std::min)(best, segmentSegmentDistanceSquared(a, b, triangle.v2, triangle.v0));
        }
    }
}

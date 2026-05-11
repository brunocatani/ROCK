#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock::debug_convex_hull_mesh
{
    /*
     * ROCK's overlay decodes FO4VR hknp support vertices on the render-submit
     * path, where unbounded triangulation is more dangerous than a slightly
     * approximate debug mesh. For ROCK's richer collider view we canonicalize
     * hull faces once, deduplicate planes/triangles, and approximate convex
     * radius by moving existing support vertices outward instead of multiplying
     * vertices and running a second hull solve.
     */
    struct Vec3
    {
        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
    };

    using Triangle = std::array<std::uint16_t, 3>;

    inline constexpr float kDefaultVertexDedupDistanceSq = 0.001f;
    inline constexpr float kPlaneDistanceEpsilon = 0.01f;
    inline constexpr float kPlaneNormalDotEpsilon = 0.999f;
    inline constexpr float kTriangleAreaSqEpsilon = 1.0e-6f;

    inline Vec3 add(const Vec3& lhs, const Vec3& rhs)
    {
        return Vec3{ lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z };
    }

    inline Vec3 sub(const Vec3& lhs, const Vec3& rhs)
    {
        return Vec3{ lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z };
    }

    inline Vec3 scale(const Vec3& value, float scalar)
    {
        return Vec3{ value.x * scalar, value.y * scalar, value.z * scalar };
    }

    inline float dot(const Vec3& lhs, const Vec3& rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    inline Vec3 cross(const Vec3& lhs, const Vec3& rhs)
    {
        return Vec3{ lhs.y * rhs.z - lhs.z * rhs.y, lhs.z * rhs.x - lhs.x * rhs.z, lhs.x * rhs.y - lhs.y * rhs.x };
    }

    inline float lengthSq(const Vec3& value)
    {
        return dot(value, value);
    }

    inline Vec3 normalized(const Vec3& value)
    {
        const float lenSq = lengthSq(value);
        if (lenSq <= 1.0e-12f) {
            return Vec3{ 0.0f, 0.0f, 1.0f };
        }

        return scale(value, 1.0f / std::sqrt(lenSq));
    }

    inline std::vector<Vec3> deduplicateVertices(const std::vector<Vec3>& vertices, float distanceSq = kDefaultVertexDedupDistanceSq)
    {
        std::vector<Vec3> unique;
        unique.reserve(vertices.size());

        for (const auto& vertex : vertices) {
            bool duplicate = false;
            for (const auto& existing : unique) {
                if (lengthSq(sub(existing, vertex)) < distanceSq) {
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate) {
                unique.push_back(vertex);
            }
        }

        return unique;
    }

    inline void inflateVerticesFromCentroid(std::vector<Vec3>& vertices, float radius)
    {
        if (vertices.empty() || radius <= 0.0f) {
            return;
        }

        Vec3 centroid{};
        for (const auto& vertex : vertices) {
            centroid = add(centroid, vertex);
        }
        centroid = scale(centroid, 1.0f / static_cast<float>(vertices.size()));

        for (auto& vertex : vertices) {
            const Vec3 direction = sub(vertex, centroid);
            if (lengthSq(direction) <= 1.0e-8f) {
                continue;
            }
            vertex = add(vertex, scale(normalized(direction), radius));
        }
    }

    struct FacePlane
    {
        Vec3 normal{};
        float offset = 0.0f;
    };

    inline bool samePlane(const FacePlane& lhs, const FacePlane& rhs)
    {
        return dot(lhs.normal, rhs.normal) > kPlaneNormalDotEpsilon && std::fabs(lhs.offset - rhs.offset) <= kPlaneDistanceEpsilon;
    }

    inline bool containsTriangle(const std::vector<Triangle>& triangles, Triangle candidate)
    {
        std::sort(candidate.begin(), candidate.end());
        for (auto existing : triangles) {
            std::sort(existing.begin(), existing.end());
            if (existing == candidate) {
                return true;
            }
        }
        return false;
    }

    inline void appendTriangle(std::vector<Triangle>& triangles, const std::vector<Vec3>& vertices, std::uint16_t a, std::uint16_t b, std::uint16_t c, const Vec3& outwardNormal)
    {
        if (a == b || b == c || a == c) {
            return;
        }

        Vec3 normal = cross(sub(vertices[b], vertices[a]), sub(vertices[c], vertices[a]));
        if (lengthSq(normal) <= kTriangleAreaSqEpsilon) {
            return;
        }

        if (dot(normal, outwardNormal) < 0.0f) {
            std::swap(b, c);
        }

        const Triangle candidate{ a, b, c };
        if (!containsTriangle(triangles, candidate)) {
            triangles.push_back(candidate);
        }
    }

    inline std::vector<std::uint16_t> sortedFaceIndices(const std::vector<Vec3>& vertices, const FacePlane& plane)
    {
        std::vector<std::uint16_t> indices;
        indices.reserve(vertices.size());

        for (std::size_t i = 0; i < vertices.size() && i <= static_cast<std::size_t>((std::numeric_limits<std::uint16_t>::max)()); ++i) {
            if (std::fabs(dot(plane.normal, vertices[i]) - plane.offset) <= kPlaneDistanceEpsilon) {
                indices.push_back(static_cast<std::uint16_t>(i));
            }
        }

        if (indices.size() < 3) {
            return {};
        }

        Vec3 center{};
        for (const auto index : indices) {
            center = add(center, vertices[index]);
        }
        center = scale(center, 1.0f / static_cast<float>(indices.size()));

        Vec3 reference = std::fabs(plane.normal.z) < 0.9f ? Vec3{ 0.0f, 0.0f, 1.0f } : Vec3{ 0.0f, 1.0f, 0.0f };
        Vec3 tangent = normalized(cross(reference, plane.normal));
        Vec3 bitangent = normalized(cross(plane.normal, tangent));

        std::sort(indices.begin(), indices.end(), [&](std::uint16_t lhs, std::uint16_t rhs) {
            const Vec3 lhsDelta = sub(vertices[lhs], center);
            const Vec3 rhsDelta = sub(vertices[rhs], center);
            const float lhsAngle = std::atan2(dot(lhsDelta, bitangent), dot(lhsDelta, tangent));
            const float rhsAngle = std::atan2(dot(rhsDelta, bitangent), dot(rhsDelta, tangent));
            return lhsAngle < rhsAngle;
        });

        return indices;
    }

    inline std::vector<Triangle> triangulateConvexHullFaces(const std::vector<Vec3>& vertices)
    {
        std::vector<Triangle> triangles;
        const std::size_t count = vertices.size();
        if (count < 3 || count > static_cast<std::size_t>((std::numeric_limits<std::uint16_t>::max)())) {
            return triangles;
        }

        Vec3 centroid{};
        for (const auto& vertex : vertices) {
            centroid = add(centroid, vertex);
        }
        centroid = scale(centroid, 1.0f / static_cast<float>(count));

        std::vector<FacePlane> acceptedPlanes;
        acceptedPlanes.reserve(count);

        for (std::size_t i = 0; i < count - 2; ++i) {
            for (std::size_t j = i + 1; j < count - 1; ++j) {
                for (std::size_t k = j + 1; k < count; ++k) {
                    Vec3 normal = cross(sub(vertices[j], vertices[i]), sub(vertices[k], vertices[i]));
                    if (lengthSq(normal) <= kTriangleAreaSqEpsilon) {
                        continue;
                    }
                    normal = normalized(normal);

                    bool hasPositive = false;
                    bool hasNegative = false;
                    for (std::size_t p = 0; p < count; ++p) {
                        const float side = dot(sub(vertices[p], vertices[i]), normal);
                        if (side > kPlaneDistanceEpsilon) {
                            hasPositive = true;
                        } else if (side < -kPlaneDistanceEpsilon) {
                            hasNegative = true;
                        }

                        if (hasPositive && hasNegative) {
                            break;
                        }
                    }

                    if (hasPositive && hasNegative) {
                        continue;
                    }

                    if (dot(sub(centroid, vertices[i]), normal) > 0.0f) {
                        normal = scale(normal, -1.0f);
                    }

                    const FacePlane plane{ normal, dot(normal, vertices[i]) };
                    bool alreadyAccepted = false;
                    for (const auto& accepted : acceptedPlanes) {
                        if (samePlane(accepted, plane)) {
                            alreadyAccepted = true;
                            break;
                        }
                    }

                    if (alreadyAccepted) {
                        continue;
                    }

                    const auto faceIndices = sortedFaceIndices(vertices, plane);
                    if (faceIndices.size() < 3) {
                        continue;
                    }

                    acceptedPlanes.push_back(plane);
                    for (std::size_t faceIndex = 1; faceIndex + 1 < faceIndices.size(); ++faceIndex) {
                        appendTriangle(triangles, vertices, faceIndices[0], faceIndices[faceIndex], faceIndices[faceIndex + 1], plane.normal);
                    }
                }
            }
        }

        return triangles;
    }

    inline std::uint32_t maxExpectedConvexHullTriangles(std::uint32_t vertexCount)
    {
        if (vertexCount < 3) {
            return 0;
        }

        return vertexCount * 3;
    }
}

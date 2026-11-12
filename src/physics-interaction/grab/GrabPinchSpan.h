#pragma once

#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/MeshGrab.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

/*
 * Mesh depth along one axis, inside a circular footprint around it: how far
 * the object reaches from a surface point toward one pad. The pinch decision
 * and the pinch closure solve measure it on both sides of the thumb-index axis
 * for every opening they try, about 36 measurements for one grab, all from the
 * same surface point.
 *
 * Measuring every triangle each time made a pinch grab cost a millisecond per
 * 500 triangles (12 ms on a 7,756-triangle object, more than a frame).
 * TriangleCull holds one bounding sphere per triangle and per block of
 * consecutive triangles, built once per grab. A sphere wholly behind the
 * surface point or wholly outside the footprint cylinder cannot hold a point
 * the measurement counts, so its triangles are skipped. Triangles that pass
 * run the unchanged measurement: the result is the one the full scan gives.
 */
namespace rock::grab_pinch_span
{
    struct Query
    {
        RE::NiPoint3 originLocal{};
        // Unit direction into the object.
        RE::NiPoint3 inwardLocal{};
        float footprintRadiusLocal = 0.0f;
        float maxDepthLocal = 0.0f;
    };

    struct Depth
    {
        float depthLocal = 0.0f;
        std::uint32_t footprintSampleCount = 0;
    };

    class TriangleCull
    {
    public:
        static constexpr std::size_t kBlockSize = 16;

        template <class Triangles>
        void build(const Triangles& triangles)
        {
            _triangles.clear();
            _blocks.clear();
            _triangles.reserve(triangles.size());
            for (const auto& triangle : triangles) {
                _triangles.push_back(boundTriangle(triangle.v0, triangle.v1, triangle.v2));
            }
            _blocks.reserve((_triangles.size() + kBlockSize - 1) / kBlockSize);
            for (std::size_t begin = 0; begin < _triangles.size(); begin += kBlockSize) {
                _blocks.push_back(boundBlock(begin, (std::min)(begin + kBlockSize, _triangles.size())));
            }
        }

        [[nodiscard]] std::size_t size() const noexcept { return _triangles.size(); }

        // Calls visit(index), in ascending order, for every triangle that can
        // hold a point in front of the origin and inside the footprint.
        template <class Visit>
        void forEachCandidate(const Query& query, Visit&& visit) const
        {
            for (std::size_t block = 0; block < _blocks.size(); ++block) {
                if (!canContribute(_blocks[block], query)) {
                    continue;
                }
                const std::size_t begin = block * kBlockSize;
                const std::size_t end = (std::min)(begin + kBlockSize, _triangles.size());
                for (std::size_t index = begin; index < end; ++index) {
                    if (canContribute(_triangles[index], query)) {
                        visit(index);
                    }
                }
            }
        }

    private:
        struct Sphere
        {
            RE::NiPoint3 center{};
            float radius = 0.0f;
        };

        // Rounding room for a sphere built from float coordinates.
        [[nodiscard]] static float pad(const RE::NiPoint3& center)
        {
            return 0.00001f * (std::abs(center.x) + std::abs(center.y) + std::abs(center.z));
        }

        [[nodiscard]] static Sphere boundTriangle(const RE::NiPoint3& v0, const RE::NiPoint3& v1,
            const RE::NiPoint3& v2)
        {
            Sphere sphere{};
            sphere.center = (v0 + v1 + v2) * (1.0f / 3.0f);
            const float farthestSquared = (std::max)({ vector_math::lengthSquared(v0 - sphere.center),
                vector_math::lengthSquared(v1 - sphere.center), vector_math::lengthSquared(v2 - sphere.center) });
            sphere.radius = std::sqrt(farthestSquared) * 1.01f + pad(sphere.center);
            return sphere;
        }

        [[nodiscard]] Sphere boundBlock(std::size_t begin, std::size_t end) const
        {
            Sphere sphere{};
            for (std::size_t index = begin; index < end; ++index) {
                sphere.center += _triangles[index].center;
            }
            sphere.center = sphere.center * (1.0f / static_cast<float>(end - begin));
            for (std::size_t index = begin; index < end; ++index) {
                const auto& member = _triangles[index];
                sphere.radius = (std::max)(sphere.radius,
                    std::sqrt(vector_math::lengthSquared(member.center - sphere.center)) + member.radius);
            }
            sphere.radius = sphere.radius * 1.01f + pad(sphere.center);
            return sphere;
        }

        /*
         * False only when every point of the sphere fails the measurement's
         * own tests (depth above zero, lateral distance within the footprint).
         * The slack grows with the distance from the origin, as the rounding
         * of those tests does. A non-finite sphere compares false everywhere
         * and stays a candidate, so the measurement decides as it always did.
         */
        [[nodiscard]] static bool canContribute(const Sphere& sphere, const Query& query)
        {
            const RE::NiPoint3 delta = sphere.center - query.originLocal;
            const float slackSquared = 0.0001f * vector_math::lengthSquared(delta);
            const float along = vector_math::dot(delta, query.inwardLocal);
            const float front = along + sphere.radius;
            if (front < 0.0f && front * front > slackSquared) {
                return false;
            }
            const RE::NiPoint3 lateral = delta - query.inwardLocal * along;
            const float reach = sphere.radius + query.footprintRadiusLocal;
            return !(vector_math::lengthSquared(lateral) > reach * reach + slackSquared);
        }

        std::vector<Sphere> _triangles;
        std::vector<Sphere> _blocks;
    };

    // The deepest mesh point inside the footprint, capped at maxDepthLocal.
    // Without a cull (or with one built for another mesh) every triangle is measured.
    template <class Triangles>
    [[nodiscard]] Depth measureDepth(const Triangles& triangles, const Query& query,
        const TriangleCull* cull = nullptr)
    {
        Depth result{};
        const float footprintRadiusSquared = query.footprintRadiusLocal * query.footprintRadiusLocal;
        auto considerPoint = [&](const RE::NiPoint3& pointLocal) {
            const RE::NiPoint3 delta = pointLocal - query.originLocal;
            const float depthLocal = vector_math::dot(delta, query.inwardLocal);
            if (!std::isfinite(depthLocal) || depthLocal <= 0.0f) {
                return;
            }
            const RE::NiPoint3 lateral = delta - query.inwardLocal * depthLocal;
            const float lateralSquared = vector_math::lengthSquared(lateral);
            if (!std::isfinite(lateralSquared) || lateralSquared > footprintRadiusSquared) {
                return;
            }
            ++result.footprintSampleCount;
            result.depthLocal = (std::max)(result.depthLocal, (std::min)(depthLocal, query.maxDepthLocal));
        };
        // Coarse faces can cross the axis with every vertex outside the
        // footprint; their nearest points to the axis are measured too.
        const std::array<RE::NiPoint3, 3> axisPointsLocal{
            query.originLocal,
            query.originLocal + query.inwardLocal * (query.maxDepthLocal * 0.5f),
            query.originLocal + query.inwardLocal * query.maxDepthLocal,
        };
        auto considerTriangle = [&](const auto& localTriangle) {
            considerPoint(localTriangle.v0);
            considerPoint(localTriangle.v1);
            considerPoint(localTriangle.v2);
            const TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
            for (const auto& axisPointLocal : axisPointsLocal) {
                float distanceSquared = 0.0f;
                considerPoint(closestPointOnTriangleToPoint(axisPointLocal, triangle, distanceSquared));
            }
        };
        if (cull && cull->size() == triangles.size()) {
            cull->forEachCandidate(query, [&](std::size_t index) { considerTriangle(triangles[index]); });
        } else {
            for (const auto& localTriangle : triangles) {
                considerTriangle(localTriangle);
            }
        }
        return result;
    }
}

#pragma once

#include "RE/NetImmerse/NiPoint.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock
{
    // Built with the immutable weapon body bank. Owns bounds and triangle
    // indices only; queries use the same triangles and exact distance routine
    // as the original surface probe. No allocation occurs during a query.
    class WeaponTriangleIndex
    {
    public:
        struct QueryStats { std::uint32_t nodes = 0; std::uint32_t triangles = 0; };

        void clear() { _nodes.clear(); _indices.clear(); }

        template <class Triangles>
        void build(const Triangles& triangles)
        {
            clear();
            _indices.reserve(triangles.size());
            // Temporary value-only build data: 48 bytes per source triangle,
            // released before publication. Bounds and double-precision centroids
            // are computed once, preserving the previous split axis and tie rule.
            std::vector<TriangleBuildData> prepared;
            prepared.reserve(triangles.size());
            for (std::size_t i = 0; i < triangles.size(); ++i) {
                const auto& t = triangles[i];
                auto& data = prepared.emplace_back();
                if (!finite(t.v0) || !finite(t.v1) || !finite(t.v2)) continue;
                _indices.push_back(static_cast<std::uint32_t>(i));
                data.min = data.max = t.v0;
                for (const auto& p : { t.v1, t.v2 }) {
                    data.min = { (std::min)(data.min.x, p.x), (std::min)(data.min.y, p.y), (std::min)(data.min.z, p.z) };
                    data.max = { (std::max)(data.max.x, p.x), (std::max)(data.max.y, p.y), (std::max)(data.max.z, p.z) };
                }
                for (int axis = 0; axis < 3; ++axis)
                    data.center[axis] = (static_cast<double>(component(t.v0, axis)) + component(t.v1, axis) + component(t.v2, axis)) / 3.0;
            }
            if (!_indices.empty()) {
                _nodes.reserve(_indices.size() / 2 + 1);
                buildNode(prepared, 0, static_cast<std::uint32_t>(_indices.size()));
            }
        }

        template <class Triangles, class Distance>
        float nearestDistanceSquared(const Triangles& triangles, const RE::NiPoint3& point,
            float radiusSquared, Distance&& distance, QueryStats* stats = nullptr) const
        {
            float best = (std::numeric_limits<float>::infinity)();
            if (_nodes.empty() || !finite(point) || std::isnan(radiusSquared) || radiusSquared < 0.0f) return best;
            // Median splits halve the index count at every level. A uint32
            // triangle count needs at most 32 deferred siblings.
            std::array<std::uint32_t, 64> stack{};
            std::size_t count = 1;
            while (count != 0) {
                const auto& node = _nodes[stack[--count]];
                if (stats) ++stats->nodes;
                const float limit = (std::min)(best, radiusSquared);
                if (boundsDistanceSquared(point, node) > limit) continue;
                if (node.count != 0) {
                    for (std::uint32_t i = node.begin; i < node.begin + node.count; ++i) {
                        if (stats) ++stats->triangles;
                        const float candidate = distance(point, triangles[_indices[i]]);
                        if (std::isfinite(candidate) && candidate >= 0.0f && candidate <= radiusSquared)
                            best = (std::min)(best, candidate);
                    }
                } else {
                    const float left = boundsDistanceSquared(point, _nodes[node.left]);
                    const float right = boundsDistanceSquared(point, _nodes[node.right]);
                    const auto nearChild = left <= right ? node.left : node.right;
                    const auto farChild = left <= right ? node.right : node.left;
                    if ((std::max)(left, right) <= limit) stack[count++] = farChild;
                    if ((std::min)(left, right) <= limit) stack[count++] = nearChild;
                }
            }
            return best;
        }

        template <class Triangles, class Predicate>
        bool anyTriangleOverlappingBounds(const Triangles& triangles, const RE::NiPoint3& minimum,
            const RE::NiPoint3& maximum, Predicate&& predicate, QueryStats* stats = nullptr) const
        {
            if (_nodes.empty()) return false;
            std::array<std::uint32_t, 64> stack{};
            std::size_t count = 1;
            while (count != 0) {
                const auto& node = _nodes[stack[--count]];
                if (stats) ++stats->nodes;
                // Widen rejection bounds for the float exact-contact predicate's
                // rounding at edges. Extra candidates never change its result.
                const float magnitude = (std::max)({ std::abs(node.min.x), std::abs(node.min.y), std::abs(node.min.z),
                    std::abs(node.max.x), std::abs(node.max.y), std::abs(node.max.z),
                    std::abs(minimum.x), std::abs(minimum.y), std::abs(minimum.z),
                    std::abs(maximum.x), std::abs(maximum.y), std::abs(maximum.z) });
                const float padding = 0.001f + magnitude * 0.00001f;
                if (maximum.x + padding < node.min.x || minimum.x - padding > node.max.x ||
                    maximum.y + padding < node.min.y || minimum.y - padding > node.max.y ||
                    maximum.z + padding < node.min.z || minimum.z - padding > node.max.z) continue;
                if (node.count != 0) {
                    for (std::uint32_t i = node.begin; i < node.begin + node.count; ++i) {
                        if (stats) ++stats->triangles;
                        if (predicate(triangles[_indices[i]])) return true;
                    }
                } else {
                    stack[count++] = node.right;
                    stack[count++] = node.left;
                }
            }
            return false;
        }

    private:
        struct Node
        {
            RE::NiPoint3 min{}, max{};
            std::uint32_t begin = 0, count = 0, left = 0, right = 0;
        };
        struct TriangleBuildData
        {
            RE::NiPoint3 min{}, max{};
            std::array<double, 3> center{};
        };
        std::vector<Node> _nodes;
        std::vector<std::uint32_t> _indices;

        static bool finite(const RE::NiPoint3& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); }
        static float component(const RE::NiPoint3& p, int axis) { return axis == 0 ? p.x : axis == 1 ? p.y : p.z; }
        static float boundsDistanceSquared(const RE::NiPoint3& p, const Node& node)
        {
            const float x = p.x - std::clamp(p.x, node.min.x, node.max.x);
            const float y = p.y - std::clamp(p.y, node.min.y, node.max.y);
            const float z = p.z - std::clamp(p.z, node.min.z, node.max.z);
            return x * x + y * y + z * z;
        }

        std::uint32_t buildNode(const std::vector<TriangleBuildData>& prepared, std::uint32_t begin, std::uint32_t count)
        {
            Node node{};
            node.min = prepared[_indices[begin]].min;
            node.max = prepared[_indices[begin]].max;
            for (std::uint32_t i = begin + 1; i < begin + count; ++i) {
                const auto& data = prepared[_indices[i]];
                node.min = { (std::min)(node.min.x, data.min.x), (std::min)(node.min.y, data.min.y), (std::min)(node.min.z, data.min.z) };
                node.max = { (std::max)(node.max.x, data.max.x), (std::max)(node.max.y, data.max.y), (std::max)(node.max.z, data.max.z) };
            }
            const auto index = static_cast<std::uint32_t>(_nodes.size());
            _nodes.push_back(node);
            if (count <= 8) {
                _nodes[index].begin = begin;
                _nodes[index].count = count;
                return index;
            }
            const auto extent = node.max - node.min;
            const int axis = extent.x >= extent.y && extent.x >= extent.z ? 0 : extent.y >= extent.z ? 1 : 2;
            const auto middle = begin + count / 2;
            std::nth_element(_indices.begin() + begin, _indices.begin() + middle, _indices.begin() + begin + count,
                [&](std::uint32_t a, std::uint32_t b) {
                    const double ca = prepared[a].center[axis], cb = prepared[b].center[axis];
                    return ca == cb ? a < b : ca < cb;
                });
            const auto left = buildNode(prepared, begin, middle - begin);
            const auto right = buildNode(prepared, middle, begin + count - middle);
            _nodes[index].left = left;
            _nodes[index].right = right;
            return index;
        }
    };
}

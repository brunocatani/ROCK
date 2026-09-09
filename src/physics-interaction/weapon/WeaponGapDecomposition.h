#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <span>
#include <unordered_map>
#include <vector>

namespace rock::weapon_gap_decomposition
{
    // Build-time geometry only. No engine pointers, native hull calls or frame state.
    // Cuts clip the original triangles, so both children retain every surface on
    // their side of the plane. We never replace a connected concavity with a point
    // median split, or drop a small island to satisfy the child budget.
    struct Point
    {
        double x{}, y{}, z{};
        Point operator+(Point b) const { return { x + b.x, y + b.y, z + b.z }; }
        Point operator-(Point b) const { return { x - b.x, y - b.y, z - b.z }; }
        Point operator*(double s) const { return { x * s, y * s, z * s }; }
    };
    using Triangle = std::array<Point, 3>;
    using Mesh = std::vector<Triangle>;
    inline double dot(Point a, Point b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
    inline Point cross(Point a, Point b) { return { a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x }; }
    inline double axis(Point p, int a) { return a == 0 ? p.x : a == 1 ? p.y : p.z; }
    inline Point center(const Triangle& t) { return (t[0] + t[1] + t[2]) * (1.0 / 3.0); }
    inline Point normal(const Triangle& t) { return cross(t[1] - t[0], t[2] - t[0]); }

    // A whole source must fit the native hull-creation slice (eight hulls).
    inline constexpr std::size_t kMaxChildren = 8;
    inline constexpr std::size_t kMaxInputTriangles = 65536;
    inline constexpr std::size_t kMaxWorkingTriangles = kMaxInputTriangles * 4;
    inline constexpr std::size_t kProbeCount = 96;
    inline constexpr std::size_t kMaxQueryWork = 262144;

    struct Result
    {
        std::vector<std::vector<Point>> pieces;
        std::size_t islands{};
        std::size_t cuts{};
        std::size_t queryWork{};
        bool budgetLimited{};
    };

    namespace detail
    {
        struct Bounds
        {
            Point min{ 1e100, 1e100, 1e100 }, max{ -1e100, -1e100, -1e100 };
            void add(Point p)
            {
                min = { (std::min)(min.x, p.x), (std::min)(min.y, p.y), (std::min)(min.z, p.z) };
                max = { (std::max)(max.x, p.x), (std::max)(max.y, p.y), (std::max)(max.z, p.z) };
            }
            bool intersects(Point origin, Point direction, double maxDistance) const
            {
                double lo = 0.0, hi = maxDistance;
                for (int a = 0; a < 3; ++a) {
                    const double d = axis(direction, a), o = axis(origin, a);
                    if (std::abs(d) < 1e-12) {
                        if (o < axis(min, a) || o > axis(max, a)) { return false; }
                    } else {
                        double first = (axis(min, a) - o) / d, last = (axis(max, a) - o) / d;
                        if (first > last) { std::swap(first, last); }
                        lo = (std::max)(lo, first);
                        hi = (std::min)(hi, last);
                        if (lo > hi) { return false; }
                    }
                }
                return true;
            }
        };

        inline bool hasVolume(std::span<const Triangle> mesh)
        {
            if (mesh.empty()) { return false; }
            const Point base = mesh.front()[0];
            Point edge{}, plane{};
            double edgeSquared = 0.0, planeSquared = 0.0;
            for (const auto& t : mesh) {
                for (Point p : t) {
                    const double d = dot(p - base, p - base);
                    if (d > edgeSquared) { edge = p - base; edgeSquared = d; }
                }
            }
            if (edgeSquared < 1e-8) { return false; }
            for (const auto& t : mesh) {
                for (Point p : t) {
                    Point n = cross(edge, p - base);
                    if (dot(n, n) > planeSquared) { plane = n; planeSquared = dot(n, n); }
                }
            }
            if (planeSquared < 1e-12) { return false; }
            for (const auto& t : mesh) {
                for (Point p : t) {
                    if (std::abs(dot(plane, p - base)) > std::sqrt(planeSquared) * 1e-4) { return true; }
                }
            }
            return false;
        }

        struct KeyHash
        {
            std::size_t operator()(const std::array<std::int64_t, 3>& key) const noexcept
            {
                std::size_t h = 0;
                for (auto v : key) { h ^= std::hash<std::int64_t>{}(v) + 0x9e3779b9u + (h << 6) + (h >> 2); }
                return h;
            }
        };

        inline std::vector<Mesh> islands(std::span<const Triangle> triangles, double weld, Result& result)
        {
            std::vector<std::size_t> parent(triangles.size());
            std::iota(parent.begin(), parent.end(), 0);
            const auto root = [&](std::size_t i) {
                while (parent[i] != i) { parent[i] = parent[parent[i]]; i = parent[i]; }
                return i;
            };
            std::unordered_map<std::array<std::int64_t, 3>, std::size_t, KeyHash> owners;
            owners.reserve(triangles.size() * 2);
            for (std::size_t i = 0; i < triangles.size(); ++i) {
                for (Point p : triangles[i]) {
                    const std::array<std::int64_t, 3> key{ std::llround(p.x / weld), std::llround(p.y / weld), std::llround(p.z / weld) };
                    auto [it, inserted] = owners.emplace(key, i);
                    if (!inserted) { parent[root(i)] = root(it->second); }
                }
            }
            std::vector<Mesh> parts;
            std::vector<std::size_t> slots(triangles.size(), triangles.size());
            for (std::size_t i = 0; i < triangles.size(); ++i) {
                const auto r = root(i);
                if (slots[r] == triangles.size()) { slots[r] = parts.size(); parts.emplace_back(); }
                parts[slots[r]].push_back(triangles[i]);
            }
            result.islands = parts.size();
            // Resolve globally reversed winding separately for each mesh island.
            // For open/mixed-winding surfaces probes are only a heuristic; cuts
            // still preserve the source surfaces and remain inside its envelope.
            for (auto& part : parts) {
                const Point origin = part.front()[0];
                double volume = 0.0;
                for (const auto& t : part) { volume += dot(t[0] - origin, cross(t[1] - origin, t[2] - origin)); }
                if (volume < 0.0) { for (auto& t : part) { std::swap(t[1], t[2]); } }
            }
            if (parts.size() > kMaxChildren ||
                std::any_of(parts.begin(), parts.end(), [](const auto& part) { return !hasVolume(part); })) {
                result.budgetLimited = parts.size() > kMaxChildren;
                Mesh joined;
                joined.reserve(triangles.size());
                for (const auto& part : parts) { joined.insert(joined.end(), part.begin(), part.end()); }
                parts.clear();
                parts.push_back(std::move(joined));
            }
            return parts;
        }

        class SurfaceTree
        {
            struct Node { Bounds bounds; std::size_t first{}, count{}, left{}, right{}; };
            std::span<const Triangle> _mesh;
            std::vector<std::size_t> _indices;
            std::vector<Node> _nodes;

            std::size_t build(std::size_t first, std::size_t count)
            {
                const auto index = _nodes.size();
                _nodes.emplace_back();
                Bounds bounds;
                for (std::size_t i = first; i < first + count; ++i) { for (auto p : _mesh[_indices[i]]) { bounds.add(p); } }
                _nodes[index] = { bounds, first, count, 0, 0 };
                if (count <= 8) { return index; }
                const auto span = bounds.max - bounds.min;
                const int a = span.x >= span.y && span.x >= span.z ? 0 : span.y >= span.z ? 1 : 2;
                const auto mid = first + count / 2;
                std::nth_element(_indices.begin() + first, _indices.begin() + mid, _indices.begin() + first + count,
                    [&](auto i, auto j) {
                        const double x = axis(center(_mesh[i]), a), y = axis(center(_mesh[j]), a);
                        return x == y ? i < j : x < y;
                    });
                const auto left = build(first, mid - first);
                const auto right = build(mid, first + count - mid);
                _nodes[index].count = 0;
                _nodes[index].left = left;
                _nodes[index].right = right;
                return index;
            }

            void trace(std::size_t index, Point origin, Point direction, std::size_t ignore,
                double& distance, std::size_t& hit, std::size_t& work) const
            {
                if (work >= kMaxQueryWork) { return; }
                ++work;
                const auto& node = _nodes[index];
                if (!node.bounds.intersects(origin, direction, distance)) { return; }
                if (node.count == 0) {
                    trace(node.left, origin, direction, ignore, distance, hit, work);
                    trace(node.right, origin, direction, ignore, distance, hit, work);
                    return;
                }
                for (std::size_t i = node.first; i < node.first + node.count && work < kMaxQueryWork; ++i) {
                    ++work;
                    const auto candidate = _indices[i];
                    if (candidate == ignore) { continue; }
                    const auto& t = _mesh[candidate];
                    const auto e1 = t[1] - t[0], e2 = t[2] - t[0];
                    const auto p = cross(direction, e2);
                    const double determinant = dot(e1, p);
                    if (std::abs(determinant) < 1e-12) { continue; }
                    const double inverse = 1.0 / determinant;
                    const auto delta = origin - t[0];
                    const double u = dot(delta, p) * inverse;
                    const auto q = cross(delta, e1);
                    const double v = dot(direction, q) * inverse;
                    const double d = dot(e2, q) * inverse;
                    if (u >= -1e-8 && v >= -1e-8 && u + v <= 1.0 + 1e-8 && d > 1e-6 && d < distance) {
                        distance = d; hit = candidate;
                    }
                }
            }

        public:
            explicit SurfaceTree(std::span<const Triangle> mesh) : _mesh(mesh), _indices(mesh.size())
            {
                std::iota(_indices.begin(), _indices.end(), 0);
                _nodes.reserve(mesh.size() / 2 + 1);
                build(0, mesh.size());
            }
            struct Cut { Point normal{}; double offset{}, score{}; };
            Cut findGap(double minimumGap, Result& result) const
            {
                Cut best;
                // Area-stratified samples prevent dense bevels from crowding out
                // the large inner faces of a bipod. Every probe has a fixed work cap.
                double totalArea = 0.0;
                for (const auto& t : _mesh) { auto n = normal(t); totalArea += std::sqrt(dot(n, n)); }
                if (totalArea < 1e-10) { return best; }
                std::size_t triangle = 0;
                double accumulated = std::sqrt(dot(normal(_mesh[0]), normal(_mesh[0])));
                for (std::size_t probe = 0; probe < kProbeCount && result.queryWork < kMaxQueryWork; ++probe) {
                    const double target = totalArea * (static_cast<double>(probe) + 0.5) / kProbeCount;
                    while (triangle + 1 < _mesh.size() && accumulated < target) {
                        ++triangle;
                        accumulated += std::sqrt(dot(normal(_mesh[triangle]), normal(_mesh[triangle])));
                    }
                    const auto n = normal(_mesh[triangle]);
                    const double length = std::sqrt(dot(n, n));
                    if (length < 1e-10) { continue; }
                    const auto direction = n * (1.0 / length);
                    const auto origin = center(_mesh[triangle]);
                    double distance = 1e100;
                    std::size_t hit = _mesh.size();
                    trace(0, origin, direction, triangle, distance, hit, result.queryWork);
                    // An incomplete nearest-hit query is never evidence of a gap.
                    if (result.queryWork >= kMaxQueryWork) { result.budgetLimited = true; break; }
                    if (hit == _mesh.size() || distance < minimumGap) { continue; }
                    const auto otherNormal = normal(_mesh[hit]);
                    if (dot(direction, otherNormal) >= -0.25 * std::sqrt(dot(otherNormal, otherNormal))) { continue; }
                    const double score = distance * std::sqrt(length);
                    if (score > best.score) { best = { direction, dot(direction, origin) + distance * 0.5, score }; }
                }
                return best;
            }
        };

        inline void clip(const Triangle& triangle, Point normal, double offset, Mesh& out)
        {
            std::array<Point, 4> polygon{};
            std::size_t count = 0;
            for (std::size_t i = 0; i < 3; ++i) {
                const auto a = triangle[i], b = triangle[(i + 1) % 3];
                const double da = dot(normal, a) - offset, db = dot(normal, b) - offset;
                if (da <= 0.0) { polygon[count++] = a; }
                if ((da < 0.0 && db > 0.0) || (da > 0.0 && db < 0.0)) { polygon[count++] = a + (b - a) * (da / (da - db)); }
            }
            for (std::size_t i = 1; i + 1 < count; ++i) { out.push_back({ polygon[0], polygon[i], polygon[i + 1] }); }
        }
    }

    inline Result decompose(std::span<const Triangle> triangles, double minimumGap, double weldGrid)
    {
        Result result;
        if (triangles.empty()) { return result; }
        if (triangles.size() > kMaxInputTriangles) { result.budgetLimited = true; return result; }
        if (!std::isfinite(minimumGap) || minimumGap <= 0.0 || !std::isfinite(weldGrid) || weldGrid <= 0.0) { return result; }
        for (const auto& t : triangles) {
            for (auto p : t) {
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
                    std::abs(p.x / weldGrid) > 1e15 || std::abs(p.y / weldGrid) > 1e15 || std::abs(p.z / weldGrid) > 1e15) { return result; }
            }
        }
        auto parts = detail::islands(triangles, weldGrid, result);
        std::size_t workingTriangles = triangles.size();
        for (std::size_t i = 0; i < parts.size() && result.queryWork < kMaxQueryWork;) {
            if (parts.size() >= kMaxChildren) { result.budgetLimited = true; break; }
            const detail::SurfaceTree tree(parts[i]);
            const auto cut = tree.findGap(minimumGap, result);
            if (cut.score <= 0.0) { ++i; continue; }
            Mesh left, right;
            left.reserve(parts[i].size()); right.reserve(parts[i].size());
            for (const auto& t : parts[i]) {
                detail::clip(t, cut.normal, cut.offset, left);
                detail::clip(t, cut.normal * -1.0, -cut.offset, right);
            }
            const auto nextTriangleCount = workingTriangles - parts[i].size() + left.size() + right.size();
            if (nextTriangleCount > kMaxWorkingTriangles) { result.budgetLimited = true; break; }
            if (!detail::hasVolume(left) || !detail::hasVolume(right)) { ++i; continue; }
            workingTriangles = nextTriangleCount;
            parts[i] = std::move(left);
            parts.push_back(std::move(right));
            ++result.cuts;
        }
        result.pieces.reserve(parts.size());
        for (const auto& part : parts) {
            auto& points = result.pieces.emplace_back();
            points.reserve(part.size() * 3);
            for (const auto& triangle : part) { points.insert(points.end(), triangle.begin(), triangle.end()); }
        }
        return result;
    }
}

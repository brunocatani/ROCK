#include "physics-interaction/weapon/WeaponTriangleIndex.h"
#include "physics-interaction/grab/MeshGrab.h"

#include <chrono>
#include <cstdio>
#include <random>

int main()
{
    std::vector<rock::TriangleData> triangles;
    for (int x = 0; x < 100; ++x) {
        for (int y = 0; y < 100; ++y) {
            rock::TriangleData t{};
            t.v0 = { x * 3.0f, y * 3.0f, 0.0f };
            t.v1 = t.v0 + RE::NiPoint3{ 2.0f, 0.0f, 0.0f };
            t.v2 = t.v0 + RE::NiPoint3{ 0.0f, 2.0f, 0.0f };
            triangles.push_back(t);
        }
    }
    // Degenerate and non-finite triangles must keep the brute-force behavior.
    rock::TriangleData degenerate{};
    triangles.push_back(degenerate);
    auto invalid = degenerate;
    invalid.v0.x = std::numeric_limits<float>::quiet_NaN();
    triangles.push_back(invalid);
    rock::WeaponTriangleIndex index;
    index.build(triangles);
    const auto distance = [](const RE::NiPoint3& p, const rock::TriangleData& t) {
        float value = std::numeric_limits<float>::infinity();
        (void)rock::closestPointOnTriangleToPoint(p, t, value);
        return value;
    };
    std::uint64_t bruteTriangleTests = 0;
    const auto brute = [&](const RE::NiPoint3& point, float radiusSquared) {
        float best = std::numeric_limits<float>::infinity();
        for (const auto& t : triangles) {
            const auto finite = [](const RE::NiPoint3& p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z); };
            if (!finite(t.v0) || !finite(t.v1) || !finite(t.v2)) continue;
            ++bruteTriangleTests;
            const float d = distance(point, t);
            if (std::isfinite(d) && d >= 0.0f && d <= radiusSquared) best = (std::min)(best, d);
        }
        return best;
    };
    std::mt19937 random(0x524F434B);
    std::uniform_real_distribution<float> position(-10.0f, 310.0f);
    std::uniform_real_distribution<float> radius(0.1f, 12.0f);
    rock::WeaponTriangleIndex::QueryStats stats{};
    double bruteMilliseconds = 0.0, indexMilliseconds = 0.0;
    constexpr int queries = 400;
    for (int i = 0; i < queries; ++i) {
        const RE::NiPoint3 point{ position(random), position(random), static_cast<float>(i % 11) - 5.0f };
        const float r = radius(random);
        const auto before = std::chrono::steady_clock::now();
        const float expected = brute(point, r * r);
        const auto middle = std::chrono::steady_clock::now();
        const float actual = index.nearestDistanceSquared(triangles, point, r * r, distance, &stats);
        const auto after = std::chrono::steady_clock::now();
        bruteMilliseconds += std::chrono::duration<double, std::milli>(middle - before).count();
        indexMilliseconds += std::chrono::duration<double, std::milli>(after - middle).count();
        if (actual != expected) {
            std::printf("query %d: expected %.9g got %.9g\n", i, expected, actual);
            return 1;
        }
    }
    const auto measuredBruteTests = bruteTriangleTests;
    const RE::NiPoint3 boundary{ 1.0f, 0.5f, 2.0f };
    if (index.nearestDistanceSquared(triangles, boundary, 4.0f, distance) != brute(boundary, 4.0f)) return 2;
    if (index.nearestDistanceSquared(triangles, boundary, std::nextafter(4.0f, 0.0f), distance) !=
        brute(boundary, std::nextafter(4.0f, 0.0f))) return 3;
    if (stats.triangles >= queries * 10000 / 10) return 4;
    std::printf("Exact surface parity: %d queries; triangle tests %u vs %llu; brute %.3fms, index %.3fms\n",
        queries, stats.triangles, static_cast<unsigned long long>(measuredBruteTests), bruteMilliseconds, indexMilliseconds);
    // Broad overlapping bounds exercise traversal without relying on a grid.
    triangles.clear();
    for (int i = 0; i < 128; ++i) {
        rock::TriangleData t{};
        t.v0 = { position(random), position(random), position(random) };
        t.v1 = { position(random), position(random), position(random) };
        t.v2 = { position(random), position(random), position(random) };
        triangles.push_back(t);
    }
    index.build(triangles);
    for (int i = 0; i < 200; ++i) {
        const RE::NiPoint3 point{ position(random), position(random), position(random) };
        const float limit = i % 2 == 0 ? 144.0f : std::numeric_limits<float>::infinity();
        if (index.nearestDistanceSquared(triangles, point, limit, distance) != brute(point, limit)) return 9;
    }
    index.clear();
    if (std::isfinite(index.nearestDistanceSquared(triangles, boundary, 4.0f, distance))) return 5;
    triangles.clear();
    index.build(triangles);
    if (std::isfinite(index.nearestDistanceSquared(triangles, boundary, 4.0f, distance))) return 6;
    triangles.push_back(invalid);
    index.build(triangles);
    if (std::isfinite(index.nearestDistanceSquared(triangles, boundary, 4.0f, distance))) return 7;
    triangles = { degenerate };
    index.build(triangles);
    if (index.nearestDistanceSquared(triangles, {}, 1.0f, distance) != brute({}, 1.0f)) return 8;
    return 0;
}

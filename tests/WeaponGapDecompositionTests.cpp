#include "physics-interaction/weapon/WeaponGapDecomposition.h"
#include "physics-interaction/weapon/WeaponGeometry.h"

#include <cstdio>
#include <chrono>
#include <stdexcept>

using namespace rock::weapon_gap_decomposition;

namespace
{
    bool check(bool value, const char* label)
    {
        if (!value) { std::printf("FAIL: %s\n", label); }
        return value;
    }

    void quad(Mesh& mesh, Point a, Point b, Point c, Point d)
    {
        mesh.push_back({ a, b, c }); mesh.push_back({ a, c, d });
    }

    Mesh fixture(bool solid)
    {
        // Closed extruded U: two legs and a connected upper bridge. Shared
        // cell faces are omitted, so gap detection must work within ONE island.
        constexpr double xs[]{ -5, -3, 3, 5 }, ys[]{ -12, -2, 0 };
        const auto occupied = [solid](int x, int y) { return x >= 0 && x < 3 && y >= 0 && y < 2 && (solid || x != 1 || y == 1); };
        Mesh mesh;
        for (int x = 0; x < 3; ++x) {
            for (int y = 0; y < 2; ++y) {
                if (!occupied(x, y)) { continue; }
                Point a{ xs[x], ys[y], -1 }, b{ xs[x + 1], ys[y], -1 }, c{ xs[x + 1], ys[y + 1], -1 }, d{ xs[x], ys[y + 1], -1 };
                const Point z{ 0, 0, 2 };
                quad(mesh, d, c, b, a); quad(mesh, a + z, b + z, c + z, d + z);
                if (!occupied(x, y - 1)) { quad(mesh, a, b, b + z, a + z); }
                if (!occupied(x + 1, y)) { quad(mesh, b, c, c + z, b + z); }
                if (!occupied(x, y + 1)) { quad(mesh, c, d, d + z, c + z); }
                if (!occupied(x - 1, y)) { quad(mesh, d, a, a + z, d + z); }
            }
        }
        return mesh;
    }

    std::vector<Point> unique(std::span<const Point> points)
    {
        std::vector<Point> result;
        for (auto p : points) {
            if (std::none_of(result.begin(), result.end(), [p](auto q) { return dot(p - q, p - q) < 1e-12; })) { result.push_back(p); }
        }
        return result;
    }

    bool insideHull(std::span<const Point> cloud, Point query)
    {
        // Independent oracle: every supporting plane must contain the query.
        const auto points = unique(cloud);
        for (std::size_t a = 0; a < points.size(); ++a) {
            for (std::size_t b = a + 1; b < points.size(); ++b) {
                for (std::size_t c = b + 1; c < points.size(); ++c) {
                    const auto n = cross(points[b] - points[a], points[c] - points[a]);
                    const double magnitude = std::sqrt(dot(n, n));
                    if (magnitude < 1e-8) { continue; }
                    bool positive = false, negative = false;
                    for (auto p : points) {
                        const double side = dot(n, p - points[a]) / magnitude;
                        positive |= side > 1e-6; negative |= side < -1e-6;
                    }
                    const double side = dot(n, query - points[a]) / magnitude;
                    if ((!positive && side > 1e-6) || (!negative && side < -1e-6)) { return false; }
                }
            }
        }
        return true;
    }

    bool inside(const Result& result, Point p)
    {
        return std::any_of(result.pieces.begin(), result.pieces.end(), [p](const auto& cloud) { return insideHull(cloud, p); });
    }

    bool covered(const Mesh& mesh, const Result& result)
    {
        for (const auto& t : mesh) {
            for (auto p : t) { if (!inside(result, p)) { return false; } }
            if (!inside(result, center(t))) { return false; }
        }
        return true;
    }

    Point transform(Point p)
    {
        // Splay the legs and rotate/translate the complete weapon part.
        p.x *= 1.0 - p.y * 0.05;
        const double x = p.x * 0.6 - p.y * 0.8, y = p.x * 0.8 + p.y * 0.6;
        return { x * 1.7 + 100, y * 1.7 - 37, p.z * 1.7 + 42 };
    }

    bool samePieces(const Result& a, const Result& b)
    {
        if (a.pieces.size() != b.pieces.size() || a.cuts != b.cuts || a.queryWork != b.queryWork || a.budgetLimited != b.budgetLimited) { return false; }
        for (std::size_t i = 0; i < a.pieces.size(); ++i) {
            if (a.pieces[i].size() != b.pieces[i].size()) { return false; }
            for (std::size_t j = 0; j < a.pieces[i].size(); ++j) {
                const auto x = a.pieces[i][j], y = b.pieces[i][j];
                if (x.x != y.x || x.y != y.y || x.z != y.z) { return false; }
            }
        }
        return true;
    }

    struct LiveLease
    {
        int& live;
        explicit LiveLease(int& count) : live(count) { ++live; }
        ~LiveLease() { --live; }
    };

    rock::weapon_geometry_work::Task cancellableChild(int& live)
    {
        LiveLease lease(live);
        co_yield 0;
        co_yield 0;
    }

    rock::weapon_geometry_work::Task cancellableParent(int& live)
    {
        LiveLease lease(live);
        auto child = cancellableChild(live);
        while (child.step()) { co_yield 0; }
    }

    rock::weapon_geometry_work::Task failedWork()
    {
        co_yield 0;
        throw std::runtime_error("geometry failure");
    }
}

int main()
{
    bool ok = true;
    auto convex = fixture(true);
    auto solid = decompose(convex, 1.0, 0.01);
    ok &= check(solid.pieces.size() == 1 && solid.cuts == 0, "convex source remains one hull");
    ok &= check(covered(convex, solid), "convex source coverage");

    auto mesh = fixture(false);
    auto u = decompose(mesh, 1.0, 0.01);
    ok &= check(u.islands == 1 && u.cuts > 0 && u.pieces.size() <= kMaxChildren, "connected U is partitioned within budget");
    ok &= check(!inside(u, { 0, -8, 0 }), "connected magazine/grip opening is empty");
    ok &= check(covered(mesh, u), "all U vertices and triangle centers retain coverage");
    for (auto& t : mesh) { for (auto& p : t) { p = transform(p); } std::swap(t[1], t[2]); }
    auto bipod = decompose(mesh, 1.0, 0.01);
    ok &= check(bipod.islands == 1 && bipod.cuts > 0, "reversed winding splayed connected bipod partitions");
    ok &= check(!inside(bipod, transform({ 0, -8, 0 })), "rotated scaled bipod opening is empty");
    ok &= check(covered(mesh, bipod), "bipod preserves surfaces across cuts");

    Result deferredU, deferredBipod;
    const auto plainU = fixture(false);
    auto firstJob = decomposeDeferred(plainU, 1.0, 0.01, deferredU);
    auto secondJob = decomposeDeferred(mesh, 1.0, 0.01, deferredBipod);
    bool firstPending = true, secondPending = true;
    std::size_t slices = 0;
    while (firstPending || secondPending) {
        if (firstPending) { firstPending = firstJob.step(); }
        if (secondPending) { secondPending = secondJob.step(); }
        ++slices;
    }
    ok &= check(slices > 10, "connected geometry yields within a single source");
    ok &= check(samePieces(u, deferredU) && samePieces(bipod, deferredBipod), "interleaved jobs preserve exact decomposition output");
    int live = 0;
    auto cancelled = cancellableParent(live);
    ok &= check(cancelled.step() && live == 2, "nested suspended work owns both leases");
    auto moved = std::move(cancelled);
    ok &= check(!cancelled.step() && live == 2, "moving suspended work retains ownership");
    moved = {};
    ok &= check(live == 0, "cancellation unwinds parent and child resources");
    auto failure = failedWork();
    ok &= check(failure.step(), "failure task initially suspends");
    bool caught = false;
    try { (void)failure.step(); } catch (const std::runtime_error&) { caught = true; }
    ok &= check(caught, "failure is delivered to the preparation catch boundary");

    Mesh dense;
    for (int repeat = 0; repeat < 1000; ++repeat) { dense.insert(dense.end(), plainU.begin(), plainU.end()); }
    Result denseResult;
    auto denseTask = decomposeDeferred(dense, 1.0, 0.01, denseResult);
    double longestStepMs = 0.0, totalStepMs = 0.0;
    std::size_t denseSteps = 0;
    bool densePending;
    do {
        const auto before = std::chrono::steady_clock::now();
        densePending = denseTask.step();
        const auto elapsed = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before).count();
        longestStepMs = (std::max)(longestStepMs, elapsed);
        totalStepMs += elapsed;
        ++denseSteps;
    } while (densePending);
    ok &= check(denseSteps > 100 && !denseResult.pieces.empty(), "dense source preparation is resumable");
    ok &= check(!inside(denseResult, {0, -8, 0}), "dense repeated geometry preserves its opening");
    std::printf("Deferred dense mesh: triangles=%zu steps=%zu activeMs=%.3f longestStepMs=%.3f\n", dense.size(), denseSteps, totalStepMs, longestStepMs);

    Mesh islands = convex;
    for (auto t : convex) { for (auto& p : t) { p.x += 30; } islands.push_back(t); }
    auto disconnected = decompose(islands, 1.0, 0.01);
    ok &= check(disconnected.islands == 2 && disconnected.pieces.size() == 2, "separate mesh islands stay separate");
    ok &= check(!inside(disconnected, { 15, -6, 0 }), "island gap stays empty");

    Mesh crowded;
    for (std::size_t i = 0; i < kMaxChildren + 1; ++i) {
        for (auto t : convex) { for (auto& p : t) { p.x += static_cast<double>(i) * 30; } crowded.push_back(t); }
    }
    auto bounded = decompose(crowded, 1.0, 0.01);
    ok &= check(bounded.pieces.size() <= kMaxChildren && bounded.budgetLimited, "child budget is reported");
    ok &= check(bounded.queryWork <= kMaxQueryWork, "query work stays bounded");
    // Use known exterior corners to check that budget pressure never drops islands.
    for (std::size_t i = 0; i < kMaxChildren + 1; ++i) {
        ok &= check(inside(bounded, { static_cast<double>(i) * 30 - 5, -12, -1 }), "budget retains every island");
    }
    ok &= check(decompose({}, 1.0, 0.01).pieces.empty(), "empty input");
    auto invalid = convex; invalid[0][0].x = std::numeric_limits<double>::quiet_NaN();
    ok &= check(decompose(invalid, 1.0, 0.01).pieces.empty(), "nonfinite input rejected");

    struct FloatPoint { float x, y, z; };
    std::vector<FloatPoint> boundaryPoints;
    for (int slice = 0; slice <= 7; ++slice) {
        for (int y = -1; y <= 1; ++y) {
            for (int z = -1; z <= 1; ++z) { boundaryPoints.push_back({ static_cast<float>(slice * 10), static_cast<float>(y), static_cast<float>(z) }); }
        }
    }
    std::vector<std::uint8_t> oldSelection(boundaryPoints.size()), newSelection(boundaryPoints.size());
    std::size_t oldCount = 0, newCount = 0;
    rock::weapon_collision_geometry_math::selectSlicedSupportPoints(boundaryPoints, oldSelection, oldCount, 252);
    rock::weapon_collision_geometry_math::selectSlicedSupportPoints(boundaryPoints, newSelection, newCount, 252, true);
    ok &= check(oldSelection == newSelection && oldCount == newCount, "slice boundaries and support ties remain identical");
    std::vector<FloatPoint> points;
    for (int latitude = 1; latitude < 35; ++latitude) {
        for (int longitude = 0; longitude < 96; ++longitude) {
            const float a = latitude * 3.14159265f / 35, b = longitude * 6.2831853f / 96;
            points.push_back({ 30 * std::sin(a) * std::cos(b), 17 * std::sin(a) * std::sin(b), 11 * std::cos(a) });
        }
    }
    const auto start = std::chrono::steady_clock::now();
    const auto original = rock::weapon_collision_geometry_math::fitConvexSupportPointCloud(points, 96, 252, 0.01f);
    const auto middle = std::chrono::steady_clock::now();
    const auto optimized = rock::weapon_collision_geometry_math::fitConvexSupportPointCloud(points, 96, 252, 0.01f, true);
    const auto end = std::chrono::steady_clock::now();
    ok &= check(original.repairPointCount > 0, "dense fixture exercises support repair");
    ok &= check(original.accepted == optimized.accepted && original.maxSupportError == optimized.maxSupportError &&
        original.repairPointCount == optimized.repairPointCount && original.points.size() == optimized.points.size(), "incremental support validation is equivalent");
    for (std::size_t i = 0; i < (std::min)(original.points.size(), optimized.points.size()); ++i) {
        const auto a = original.points[i], b = optimized.points[i];
        ok &= check(a.x == b.x && a.y == b.y && a.z == b.z, "support selection/order unchanged");
    }
    rock::weapon_collision_geometry_math::ConvexSupportFitResult<FloatPoint> deferredFit;
    auto fitting = rock::weapon_collision_geometry_math::fitConvexSupportPointCloudDeferred(points, 96, 252, 0.01f, true, deferredFit);
    std::size_t fittingSteps = 0;
    while (fitting.step()) { ++fittingSteps; }
    ok &= check(fittingSteps > 100 && deferredFit.selectedPointCount == optimized.selectedPointCount &&
        deferredFit.maxSupportError == optimized.maxSupportError && deferredFit.repairPointCount == optimized.repairPointCount,
        "deferred point fitting preserves acceptance and repair counts");
    std::printf("U children=%zu cuts=%zu; bipod children=%zu cuts=%zu; synthetic support repair old=%.3fms new=%.3fms repairs=%zu\n",
        u.pieces.size(), u.cuts, bipod.pieces.size(), bipod.cuts,
        std::chrono::duration<double, std::milli>(middle - start).count(),
        std::chrono::duration<double, std::milli>(end - middle).count(), original.repairPointCount);
    return ok ? 0 : 1;
}

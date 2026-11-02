// Selects the candidate weapon triangles for the support-hand finger solve,
// ranked by distance to the support seat and to each finger's reference points.
#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/weapon/collision/WeaponTriangleIndex.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <utility>
#include <vector>

/*
 * Candidate triangles for the equipped-weapon support finger solve.
 *
 * Every triangle is ranked by its distance to the support seat point and to
 * each finger lane's reference points (weapon frame). Each lane keeps its
 * nearest triangles and a global ranking keeps the nearest overall; the
 * rankings are merged round-robin per lane, then filled from the global list.
 * A ranking holds the k smallest entries under (distance, scan ordinal), a
 * strict total order, so its content does not depend on visiting order.
 *
 * That lets a part's equip-time WeaponTriangleIndex replace the full scan of
 * the part: a node is skipped only when every ranking is full and no triangle
 * inside it can be closer than that ranking's current worst entry. The bound
 * uses the node box transformed into the weapon frame with the scan's own
 * transform (an affine map keeps each triangle inside its transformed box),
 * padded for float rounding. A visited triangle gets the same test on its own
 * weapon-frame box before the exact distances, and then skips only the lane
 * points that cannot set a kept minimum (see measure). It keeps the scan's
 * ordinal and every distance that can decide a ranking is computed exactly as
 * the full scan does, so the selection is identical to scanning every triangle.
 *
 * Skipping only works once the rankings hold near triangles, so indexed parts
 * are walked nearest first (by their root box), each nearest child first.
 * A ranking's worst entry only improves, so a skip stays valid for the rest
 * of the walk.
 */
namespace rock::support_grip_triangle_selection
{
    inline constexpr std::size_t kLaneCount = 5;
    inline constexpr std::size_t kLaneReferenceCapacity = 10;
    inline constexpr std::size_t kGlobalRankingIndex = kLaneCount;

    struct RankedTriangle
    {
        float distanceSquared = 0.0f;
        std::uint64_t deterministicOrdinal = 0;
        TriangleData weaponLocalTriangle{};
    };

    inline bool rankedTriangleLess(const RankedTriangle& lhs, const RankedTriangle& rhs)
    {
        if (lhs.distanceSquared == rhs.distanceSquared) {
            return lhs.deterministicOrdinal < rhs.deterministicOrdinal;
        }
        return lhs.distanceSquared < rhs.distanceSquared;
    }

    using Rankings = std::array<std::vector<RankedTriangle>, kLaneCount + 1>;

    // Reference points in the weapon frame.
    struct References
    {
        RE::NiPoint3 seatPoint{};
        std::array<std::array<RE::NiPoint3, kLaneReferenceCapacity>, kLaneCount> lanePoints{};
        std::array<std::size_t, kLaneCount> laneCounts{};
    };

    // One weapon part with its source-to-weapon transform already validated.
    struct Part
    {
        std::span<const TriangleData> sourceTriangles{};
        // Equip-time index over sourceTriangles; null scans every triangle.
        const WeaponTriangleIndex* index{ nullptr };
        RE::NiTransform sourceToWeapon{};
        // Scan ordinal of sourceTriangles[0].
        std::uint64_t firstOrdinal{ 0 };
    };

    struct Stats
    {
        // Triangles given the exact distance measurement, and the point-to-triangle distances computed for them.
        std::uint64_t trianglesMeasured = 0;
        std::uint64_t exactDistances = 0;
        std::uint64_t nodesVisited = 0;
    };

    namespace detail
    {
        inline float boxDistanceSquared(const RE::NiPoint3& p, const RE::NiPoint3& minimum, const RE::NiPoint3& maximum)
        {
            const float x = p.x - std::clamp(p.x, minimum.x, maximum.x);
            const float y = p.y - std::clamp(p.y, minimum.y, maximum.y);
            const float z = p.z - std::clamp(p.z, minimum.z, maximum.z);
            return x * x + y * y + z * z;
        }

        // Float rounding of the per-vertex transform and of the exact distance; never shrinks the true bound.
        inline void padBox(RE::NiPoint3& minimum, RE::NiPoint3& maximum, float magnitude)
        {
            const float padding = 0.001f + magnitude * 0.00001f;
            minimum = { minimum.x - padding, minimum.y - padding, minimum.z - padding };
            maximum = { maximum.x + padding, maximum.y + padding, maximum.z + padding };
        }

        // Weapon-frame box of a source-frame box's eight corners, padded. False when a corner is not finite.
        inline bool weaponFrameBox(const RE::NiTransform& sourceToWeapon, const RE::NiPoint3& minimum,
            const RE::NiPoint3& maximum, RE::NiPoint3& outMinimum, RE::NiPoint3& outMaximum)
        {
            outMinimum = { (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(),
                (std::numeric_limits<float>::max)() };
            outMaximum = { -(std::numeric_limits<float>::max)(), -(std::numeric_limits<float>::max)(),
                -(std::numeric_limits<float>::max)() };
            float magnitude = 0.0f;
            for (int corner = 0; corner < 8; ++corner) {
                const RE::NiPoint3 local{ (corner & 1) ? maximum.x : minimum.x, (corner & 2) ? maximum.y : minimum.y,
                    (corner & 4) ? maximum.z : minimum.z };
                const RE::NiPoint3 p = transform_math::localPointToWorld(sourceToWeapon, local);
                if (!vector_math::hasFiniteComponents(p)) {
                    return false;
                }
                outMinimum = { (std::min)(outMinimum.x, p.x), (std::min)(outMinimum.y, p.y),
                    (std::min)(outMinimum.z, p.z) };
                outMaximum = { (std::max)(outMaximum.x, p.x), (std::max)(outMaximum.y, p.y),
                    (std::max)(outMaximum.z, p.z) };
                magnitude = (std::max)({ magnitude, std::abs(p.x), std::abs(p.y), std::abs(p.z) });
            }
            padBox(outMinimum, outMaximum, magnitude);
            return true;
        }

        // Padded box of a weapon-frame triangle's own vertices.
        inline void triangleBox(const TriangleData& triangle, RE::NiPoint3& outMinimum, RE::NiPoint3& outMaximum)
        {
            outMinimum = triangle.v0;
            outMaximum = triangle.v0;
            float magnitude = (std::max)({ std::abs(triangle.v0.x), std::abs(triangle.v0.y), std::abs(triangle.v0.z) });
            for (const auto& p : { triangle.v1, triangle.v2 }) {
                outMinimum = { (std::min)(outMinimum.x, p.x), (std::min)(outMinimum.y, p.y),
                    (std::min)(outMinimum.z, p.z) };
                outMaximum = { (std::max)(outMaximum.x, p.x), (std::max)(outMaximum.y, p.y),
                    (std::max)(outMaximum.z, p.z) };
                magnitude = (std::max)({ magnitude, std::abs(p.x), std::abs(p.y), std::abs(p.z) });
            }
            padBox(outMinimum, outMaximum, magnitude);
        }

        // Smallest distance from the box to the seat point or any lane point: the part visiting order key.
        inline float nearestReferenceBoxDistanceSquared(const References& references, const RE::NiPoint3& minimum,
            const RE::NiPoint3& maximum)
        {
            float bound = boxDistanceSquared(references.seatPoint, minimum, maximum);
            for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
                for (std::size_t referenceIndex = 0; referenceIndex < references.laneCounts[lane]; ++referenceIndex) {
                    bound = (std::min)(bound,
                        boxDistanceSquared(references.lanePoints[lane][referenceIndex], minimum, maximum));
                }
            }
            return bound;
        }

        inline void retainNearest(std::vector<RankedTriangle>& ranking, std::size_t limit,
            const RankedTriangle& candidate)
        {
            if (ranking.size() < limit) {
                ranking.push_back(candidate);
                std::push_heap(ranking.begin(), ranking.end(), rankedTriangleLess);
                return;
            }
            if (rankedTriangleLess(candidate, ranking.front())) {
                std::pop_heap(ranking.begin(), ranking.end(), rankedTriangleLess);
                ranking.back() = candidate;
                std::push_heap(ranking.begin(), ranking.end(), rankedTriangleLess);
            }
        }
    }

    /*
     * Ranks every triangle of the parts. rankings must be empty; each ranking
     * stays a max-heap (front = worst kept entry) until mergeRankings sorts it.
     */
    inline void rankParts(std::span<const Part> parts, const References& references, std::size_t laneLimit,
        std::size_t globalLimit, Rankings& rankings, Stats* stats = nullptr)
    {
        const auto finiteTriangle = [](const TriangleData& triangle) {
            return vector_math::hasFiniteComponents(triangle.v0) && vector_math::hasFiniteComponents(triangle.v1) &&
                vector_math::hasFiniteComponents(triangle.v2);
        };
        const auto toWeaponFrame = [](const TriangleData& sourceTriangle, const RE::NiTransform& sourceToWeapon) {
            return TriangleData{
                transform_math::localPointToWorld(sourceToWeapon, sourceTriangle.v0),
                transform_math::localPointToWorld(sourceToWeapon, sourceTriangle.v1),
                transform_math::localPointToWorld(sourceToWeapon, sourceTriangle.v2),
            };
        };
        /*
         * Exact distances of one weapon-frame triangle, offered to every ranking.
         * Given the triangle's padded box (indexed walk), a lane point is skipped
         * when its box distance already exceeds what both its lane and the global
         * ranking would accept: its exact distance is larger still, so it can set
         * neither minimum where that minimum would be kept. The seat distance is
         * always computed: a NaN there keeps the triangle out of the global
         * ranking, exactly as in the full scan.
         */
        const auto measure = [&](std::uint64_t ordinal, const TriangleData& weaponLocalTriangle,
                                 const RE::NiPoint3* boxMinimum, const RE::NiPoint3* boxMaximum) {
            if (stats)
                ++stats->trianglesMeasured;
            const auto& global = rankings[kGlobalRankingIndex];
            const bool globalFull = !global.empty() && global.size() >= globalLimit;
            float globalMinimumDistanceSquared = (std::numeric_limits<float>::infinity)();
            (void)closestPointOnTriangleToPoint(references.seatPoint, weaponLocalTriangle,
                globalMinimumDistanceSquared);
            if (stats)
                ++stats->exactDistances;
            for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
                const auto& ranking = rankings[lane];
                const bool bounded =
                    boxMinimum && boxMaximum && globalFull && !ranking.empty() && ranking.size() >= laneLimit;
                const float skipAbove =
                    bounded ? (std::max)(ranking.front().distanceSquared, global.front().distanceSquared) : 0.0f;
                float laneMinimumDistanceSquared = (std::numeric_limits<float>::infinity)();
                for (std::size_t referenceIndex = 0; referenceIndex < references.laneCounts[lane]; ++referenceIndex) {
                    if (bounded &&
                        detail::boxDistanceSquared(references.lanePoints[lane][referenceIndex], *boxMinimum,
                            *boxMaximum) > skipAbove) {
                        continue;
                    }
                    if (stats)
                        ++stats->exactDistances;
                    float distanceSquared = 0.0f;
                    (void)closestPointOnTriangleToPoint(references.lanePoints[lane][referenceIndex],
                        weaponLocalTriangle, distanceSquared);
                    if (std::isfinite(distanceSquared)) {
                        laneMinimumDistanceSquared = (std::min)(laneMinimumDistanceSquared, distanceSquared);
                    }
                }
                if (std::isfinite(laneMinimumDistanceSquared)) {
                    globalMinimumDistanceSquared = (std::min)(globalMinimumDistanceSquared, laneMinimumDistanceSquared);
                    detail::retainNearest(rankings[lane], laneLimit,
                        RankedTriangle{ .distanceSquared = laneMinimumDistanceSquared,
                            .deterministicOrdinal = ordinal,
                            .weaponLocalTriangle = weaponLocalTriangle });
                }
            }
            if (std::isfinite(globalMinimumDistanceSquared)) {
                detail::retainNearest(rankings[kGlobalRankingIndex], globalLimit,
                    RankedTriangle{ .distanceSquared = globalMinimumDistanceSquared,
                        .deterministicOrdinal = ordinal,
                        .weaponLocalTriangle = weaponLocalTriangle });
            }
        };

        // True when no triangle inside the weapon-frame box can enter any ranking.
        const auto boxCannotRank = [&](const RE::NiPoint3& minimum, const RE::NiPoint3& maximum) {
            const auto& global = rankings[kGlobalRankingIndex];
            if (global.size() < globalLimit || global.empty()) {
                return false;
            }
            float globalBound = detail::boxDistanceSquared(references.seatPoint, minimum, maximum);
            for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
                if (references.laneCounts[lane] == 0) {
                    continue;
                }
                const auto& ranking = rankings[lane];
                if (ranking.size() < laneLimit || ranking.empty()) {
                    return false;
                }
                float laneBound = (std::numeric_limits<float>::infinity)();
                for (std::size_t referenceIndex = 0; referenceIndex < references.laneCounts[lane]; ++referenceIndex) {
                    laneBound = (std::min)(laneBound,
                        detail::boxDistanceSquared(references.lanePoints[lane][referenceIndex], minimum, maximum));
                }
                // An entry at the worst distance with a lower ordinal still enters, so equality keeps the node.
                if (!(laneBound > ranking.front().distanceSquared)) {
                    return false;
                }
                globalBound = (std::min)(globalBound, laneBound);
            }
            return globalBound > global.front().distanceSquared;
        };

        // Parts without an index are scanned whole (the reference path); indexed parts are walked nearest first.
        std::vector<std::pair<float, std::size_t>> walkOrder{};
        walkOrder.reserve(parts.size());
        for (std::size_t partIndex = 0; partIndex < parts.size(); ++partIndex) {
            const auto& part = parts[partIndex];
            if (part.sourceTriangles.empty()) {
                continue;
            }
            if (!part.index) {
                for (std::size_t i = 0; i < part.sourceTriangles.size(); ++i) {
                    if (finiteTriangle(part.sourceTriangles[i])) {
                        measure(part.firstOrdinal + i, toWeaponFrame(part.sourceTriangles[i], part.sourceToWeapon),
                            nullptr, nullptr);
                    }
                }
                continue;
            }
            RE::NiPoint3 rootMinimum{}, rootMaximum{}, boxMinimum{}, boxMaximum{};
            float orderKey = 0.0f;
            if (part.index->bounds(rootMinimum, rootMaximum) &&
                detail::weaponFrameBox(part.sourceToWeapon, rootMinimum, rootMaximum, boxMinimum, boxMaximum)) {
                orderKey = detail::nearestReferenceBoxDistanceSquared(references, boxMinimum, boxMaximum);
            }
            walkOrder.emplace_back(std::isfinite(orderKey) ? orderKey : 0.0f, partIndex);
        }
        std::sort(walkOrder.begin(), walkOrder.end());

        for (const auto& [orderKey, partIndex] : walkOrder) {
            const auto& part = parts[partIndex];
            const RE::NiPoint3 orderPoint = transform_math::localPointToWorld(
                transform_math::invertTransform(part.sourceToWeapon), references.seatPoint);
            WeaponTriangleIndex::QueryStats queryStats{};
            part.index->visitPruned(
                orderPoint,
                [&](const RE::NiPoint3& minimum, const RE::NiPoint3& maximum) {
                    RE::NiPoint3 boxMinimum{}, boxMaximum{};
                    // A non-finite corner keeps the node: its triangles are checked one by one.
                    return !detail::weaponFrameBox(part.sourceToWeapon, minimum, maximum, boxMinimum, boxMaximum) ||
                        !boxCannotRank(boxMinimum, boxMaximum);
                },
                [&](std::uint32_t sourceIndex) {
                    const auto& sourceTriangle = part.sourceTriangles[sourceIndex];
                    if (!finiteTriangle(sourceTriangle)) {
                        return;
                    }
                    const TriangleData weaponLocalTriangle = toWeaponFrame(sourceTriangle, part.sourceToWeapon);
                    RE::NiPoint3 boxMinimum{}, boxMaximum{};
                    detail::triangleBox(weaponLocalTriangle, boxMinimum, boxMaximum);
                    if (!boxCannotRank(boxMinimum, boxMaximum)) {
                        measure(part.firstOrdinal + sourceIndex, weaponLocalTriangle, &boxMinimum, &boxMaximum);
                    }
                },
                &queryStats);
            if (stats)
                stats->nodesVisited += queryStats.nodes;
        }
    }

    // Sorts the rankings and merges up to limit unique triangles: lanes round-robin, then the global list.
    inline void mergeRankings(Rankings& rankings, std::size_t limit, std::vector<TriangleData>& outTriangles)
    {
        for (auto& ranking : rankings) {
            std::sort(ranking.begin(), ranking.end(), rankedTriangleLess);
        }
        std::vector<std::uint64_t> selectedOrdinals{};
        selectedOrdinals.reserve(limit);
        outTriangles.reserve(limit);
        const auto appendUnique = [&](const RankedTriangle& ranked) {
            if (outTriangles.size() >= limit ||
                std::find(selectedOrdinals.begin(), selectedOrdinals.end(), ranked.deterministicOrdinal) !=
                    selectedOrdinals.end()) {
                return false;
            }
            selectedOrdinals.push_back(ranked.deterministicOrdinal);
            outTriangles.push_back(ranked.weaponLocalTriangle);
            return true;
        };

        std::array<std::size_t, kLaneCount> laneCursors{};
        bool laneCandidateRemaining = true;
        while (outTriangles.size() < limit && laneCandidateRemaining) {
            laneCandidateRemaining = false;
            for (std::size_t lane = 0; lane < kLaneCount && outTriangles.size() < limit; ++lane) {
                auto& cursor = laneCursors[lane];
                const auto& ranking = rankings[lane];
                while (cursor < ranking.size()) {
                    laneCandidateRemaining = true;
                    const auto& candidate = ranking[cursor++];
                    if (appendUnique(candidate)) {
                        break;
                    }
                }
            }
        }
        for (const auto& ranked : rankings[kGlobalRankingIndex]) {
            if (outTriangles.size() >= limit) {
                break;
            }
            (void)appendUnique(ranked);
        }
    }
}

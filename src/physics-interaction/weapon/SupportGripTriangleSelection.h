#pragma once

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/weapon/WeaponTriangleIndex.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
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
 * padded for float rounding. Triangles that are visited are transformed and
 * measured exactly as the full scan does and keep the scan's ordinal, so the
 * selection is identical to scanning every triangle.
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
        std::uint64_t trianglesMeasured = 0;
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

        inline void retainNearest(std::vector<RankedTriangle>& ranking, std::size_t limit, const RankedTriangle& candidate)
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
    inline void rankParts(std::span<const Part> parts, const References& references, std::size_t laneLimit, std::size_t globalLimit, Rankings& rankings,
        Stats* stats = nullptr)
    {
        const auto measure = [&](std::uint64_t ordinal, const TriangleData& sourceTriangle, const RE::NiTransform& sourceToWeapon) {
            if (!vector_math::hasFiniteComponents(sourceTriangle.v0) || !vector_math::hasFiniteComponents(sourceTriangle.v1) ||
                !vector_math::hasFiniteComponents(sourceTriangle.v2)) {
                return;
            }
            if (stats) ++stats->trianglesMeasured;
            const TriangleData weaponLocalTriangle{
                transform_math::localPointToWorld(sourceToWeapon, sourceTriangle.v0),
                transform_math::localPointToWorld(sourceToWeapon, sourceTriangle.v1),
                transform_math::localPointToWorld(sourceToWeapon, sourceTriangle.v2),
            };
            float globalMinimumDistanceSquared = (std::numeric_limits<float>::infinity)();
            (void)closestPointOnTriangleToPoint(references.seatPoint, weaponLocalTriangle, globalMinimumDistanceSquared);
            for (std::size_t lane = 0; lane < kLaneCount; ++lane) {
                float laneMinimumDistanceSquared = (std::numeric_limits<float>::infinity)();
                for (std::size_t referenceIndex = 0; referenceIndex < references.laneCounts[lane]; ++referenceIndex) {
                    float distanceSquared = 0.0f;
                    (void)closestPointOnTriangleToPoint(references.lanePoints[lane][referenceIndex], weaponLocalTriangle, distanceSquared);
                    if (std::isfinite(distanceSquared)) {
                        laneMinimumDistanceSquared = (std::min)(laneMinimumDistanceSquared, distanceSquared);
                    }
                }
                if (std::isfinite(laneMinimumDistanceSquared)) {
                    globalMinimumDistanceSquared = (std::min)(globalMinimumDistanceSquared, laneMinimumDistanceSquared);
                    detail::retainNearest(rankings[lane], laneLimit,
                        RankedTriangle{ .distanceSquared = laneMinimumDistanceSquared, .deterministicOrdinal = ordinal, .weaponLocalTriangle = weaponLocalTriangle });
                }
            }
            if (std::isfinite(globalMinimumDistanceSquared)) {
                detail::retainNearest(rankings[kGlobalRankingIndex], globalLimit,
                    RankedTriangle{ .distanceSquared = globalMinimumDistanceSquared, .deterministicOrdinal = ordinal, .weaponLocalTriangle = weaponLocalTriangle });
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
                    laneBound = (std::min)(laneBound, detail::boxDistanceSquared(references.lanePoints[lane][referenceIndex], minimum, maximum));
                }
                // An entry at the worst distance with a lower ordinal still enters, so equality keeps the node.
                if (!(laneBound > ranking.front().distanceSquared)) {
                    return false;
                }
                globalBound = (std::min)(globalBound, laneBound);
            }
            return globalBound > global.front().distanceSquared;
        };

        for (const auto& part : parts) {
            if (part.sourceTriangles.empty()) {
                continue;
            }
            if (!part.index) {
                for (std::size_t i = 0; i < part.sourceTriangles.size(); ++i) {
                    measure(part.firstOrdinal + i, part.sourceTriangles[i], part.sourceToWeapon);
                }
                continue;
            }

            // Visiting order only affects how soon nodes can be skipped, never the result.
            const RE::NiPoint3 orderPoint =
                transform_math::localPointToWorld(transform_math::invertTransform(part.sourceToWeapon), references.seatPoint);
            WeaponTriangleIndex::QueryStats queryStats{};
            part.index->visitPruned(
                orderPoint,
                [&](const RE::NiPoint3& minimum, const RE::NiPoint3& maximum) {
                    RE::NiPoint3 boxMin{ (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)() };
                    RE::NiPoint3 boxMax{ -(std::numeric_limits<float>::max)(), -(std::numeric_limits<float>::max)(), -(std::numeric_limits<float>::max)() };
                    float magnitude = 0.0f;
                    for (int corner = 0; corner < 8; ++corner) {
                        const RE::NiPoint3 local{ (corner & 1) ? maximum.x : minimum.x, (corner & 2) ? maximum.y : minimum.y, (corner & 4) ? maximum.z : minimum.z };
                        const RE::NiPoint3 p = transform_math::localPointToWorld(part.sourceToWeapon, local);
                        if (!vector_math::hasFiniteComponents(p)) {
                            return true;
                        }
                        boxMin = { (std::min)(boxMin.x, p.x), (std::min)(boxMin.y, p.y), (std::min)(boxMin.z, p.z) };
                        boxMax = { (std::max)(boxMax.x, p.x), (std::max)(boxMax.y, p.y), (std::max)(boxMax.z, p.z) };
                        magnitude = (std::max)({ magnitude, std::abs(p.x), std::abs(p.y), std::abs(p.z) });
                    }
                    // Float rounding of the per-vertex transform; never shrinks the true bound.
                    const float padding = 0.001f + magnitude * 0.00001f;
                    boxMin = { boxMin.x - padding, boxMin.y - padding, boxMin.z - padding };
                    boxMax = { boxMax.x + padding, boxMax.y + padding, boxMax.z + padding };
                    return !boxCannotRank(boxMin, boxMax);
                },
                [&](std::uint32_t sourceIndex) { measure(part.firstOrdinal + sourceIndex, part.sourceTriangles[sourceIndex], part.sourceToWeapon); },
                &queryStats);
            if (stats) stats->nodesVisited += queryStats.nodes;
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
                std::find(selectedOrdinals.begin(), selectedOrdinals.end(), ranked.deterministicOrdinal) != selectedOrdinals.end()) {
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

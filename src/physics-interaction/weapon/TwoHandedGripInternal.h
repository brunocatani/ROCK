#pragma once

/*
 * Helpers shared by the TwoHandedGrip translation units.
 *
 * Keep one-file helpers in that file's anonymous namespace. Promote a helper
 * here only when more than one translation unit owns part of its flow.
 *
 * INTERNAL. Never include this file from a public include tree.
 */

#include "physics-interaction/grab/MeshGrab.h"

#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::two_handed_grip_detail
{
    inline constexpr std::size_t kSupportGripFingerLaneCount = 5;
    inline constexpr std::size_t kSupportGripFingerLaneReferenceCapacity = 10;
    inline constexpr std::size_t kSupportGripGlobalRankingIndex =
        kSupportGripFingerLaneCount;

    // A stable ordinal makes equal-distance triangle selection deterministic.
    struct RankedSupportGripTriangle
    {
        float distanceSquared = 0.0f;
        std::uint64_t deterministicOrdinal = 0;
        TriangleData weaponLocalTriangle{};
    };

    // Each finger lane keeps its own world-space solve references.
    struct SupportGripFingerReferenceSet
    {
        RE::NiPoint3 seatPointWorld{};
        std::array<
            std::array<
                RE::NiPoint3,
                kSupportGripFingerLaneReferenceCapacity>,
            kSupportGripFingerLaneCount>
            lanePointsWorld{};
        std::array<std::size_t, kSupportGripFingerLaneCount>
            lanePointCounts{};
        bool seatPointValid{ false };
    };
}

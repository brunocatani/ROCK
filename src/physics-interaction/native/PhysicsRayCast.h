#pragma once

#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>

namespace RE
{
    class bhkWorld;
}

namespace rock::physics_ray_cast
{
    struct ClosestSegmentResult
    {
        bool hit{ false };
        float hitFraction{ 1.0f };
        RE::NiPoint3 normalGame{};
        bool normalValid{ false };
    };

    /*
     * Executes Bethesda's bhkWorld PickObject wrapper. The wrapper owns its
     * required world synchronization; callers must stay on ROCK's game-thread
     * query boundary and must not retain the world or pick-data storage.
     */
    [[nodiscard]] bool castClosestSegment(
        RE::bhkWorld* world,
        const RE::NiPoint3& startGame,
        const RE::NiPoint3& endGame,
        std::uint32_t collisionFilterInfo,
        ClosestSegmentResult& outResult);
}

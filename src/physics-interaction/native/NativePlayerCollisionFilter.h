#pragma once

#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"

#include <cstddef>
#include <span>

namespace RE { class hknpWorld; }

namespace rock::native_player_collision
{
    inline constexpr std::size_t kMaximumPlayerBodies = 64;

    // Process-lifetime hook, installed once on the game thread. Only simulation
    // body pairs are filtered; native ray/shape query entry points stay intact.
    bool install() noexcept;

    // Single game-thread publisher; physics workers only read a quiesced value
    // snapshot. IDs are checked against live identity before any suppression.
    // An empty publication restores physical pairs in the supplied live world.
    void publish(RE::hknpWorld* world, std::span<const BodyIdentity> bodies);

    // Skeleton/world teardown: invalidate the snapshot without touching a world
    // whose lifetime may have ended. No collision bits or pair leases to restore.
    void abandon() noexcept;
}

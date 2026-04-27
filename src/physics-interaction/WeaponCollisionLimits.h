#pragma once

#include <cstddef>

namespace frik::rock
{
    /*
     * Weapon mesh collision is generated as several convex hull bodies so long
     * guns can keep stock, receiver, barrel, magazine, and accessories separated
     * instead of collapsing into one broad hull. This cap is shared rather than a
     * private literal so collision creation, debug publishing, and tests stay on
     * the same body budget.
     */
    inline constexpr std::size_t MAX_WEAPON_COLLISION_BODIES = 32;
}

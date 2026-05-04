#pragma once

#include <cstddef>

namespace frik::rock::hand_lifecycle_policy
{
    /*
     * Hand::reset is only safe after native Havok ownership has already been
     * released. Constraint IDs, held body IDs, saved inertia, and suppression
     * filters are not plain cache data; dropping them loses the only state ROCK
     * has for restoring the world.
     */
    inline constexpr bool requiresHavokCleanupBeforeReset(
        bool activeConstraintValid,
        bool handCollisionSuppressed,
        std::size_t heldBodyCount,
        bool savedObjectStateValid,
        bool handCollisionBodyValid)
    {
        return activeConstraintValid || handCollisionSuppressed || heldBodyCount > 0 || savedObjectStateValid || handCollisionBodyValid;
    }
}

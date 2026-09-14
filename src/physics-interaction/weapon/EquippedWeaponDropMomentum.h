#pragma once

#include <cstdint>

// Native body identity and solve progress for the equipped-to-loose handoff.
// Throw momentum is owned by the ordinary held-object release path.
namespace rock::equipped_weapon_drop_momentum
{
    /*
     * A body slot and motion ID are both reusable in hknp. This key carries
     * the stable native ownership evidence available from ROCK's exact-reference
     * body scan so a rebuilt body can never inherit a pending placement
     * intended for an older generation.
     */
    struct BodyIdentityKey
    {
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t motionId{ 0 };
        std::uint32_t motionFirstBodyId{ 0x7FFF'FFFF };
        std::uintptr_t shapeIdentity{ 0 };
        std::uintptr_t owningNodeIdentity{ 0 };
        std::uintptr_t collisionObjectIdentity{ 0 };
        std::uintptr_t physicsSystemInstanceIdentity{ 0 };
    };

    [[nodiscard]] constexpr bool sameBodyIdentity(const BodyIdentityKey& lhs, const BodyIdentityKey& rhs) noexcept
    {
        return lhs.bodyId == rhs.bodyId &&
               lhs.motionId == rhs.motionId &&
               lhs.motionFirstBodyId == rhs.motionFirstBodyId &&
               lhs.shapeIdentity == rhs.shapeIdentity &&
               lhs.owningNodeIdentity == rhs.owningNodeIdentity &&
               lhs.collisionObjectIdentity == rhs.collisionObjectIdentity &&
               lhs.physicsSystemInstanceIdentity == rhs.physicsSystemInstanceIdentity;
    }

    inline bool completedSettleStep(std::uint64_t discoverySequence, std::uint64_t completedSequence) noexcept
    {
        return completedSequence > discoverySequence;
    }

    inline bool publicationProgressStalled(
        std::uint64_t lastProgressSequence,
        std::uint64_t completedSequence,
        std::uint64_t maximumIdleSolveSteps) noexcept
    {
        return maximumIdleSolveSteps > 0 &&
               completedSequence >= lastProgressSequence &&
               completedSequence - lastProgressSequence >= maximumIdleSolveSteps;
    }

}

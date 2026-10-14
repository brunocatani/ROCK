#pragma once

#include <ROCK/Grab.h>
#include <cstdint>

namespace rock::provider
{
    // A hand's published interaction state plus the detail behind its
    // InputSuppressed and CollisionAvailable/TransitionSuppressed summary
    // flags. Consumers read only the public state; a change in either detail
    // still advances stateSequence.
    struct HandInteractionPublication : api::grab::HandInteractionState
    {
        std::uint32_t effectiveInputSuppressionFlags{ 0 };
        std::uint32_t collisionAvailabilityFlags{ 0 };
    };
}

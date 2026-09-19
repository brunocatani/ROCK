#pragma once

#include <cstdint>

namespace rock::transferred_weapon_grab_policy
{
    // Only an equipped-to-loose transfer owns this latch. Ordinary world
    // grabs and provider force grabs continue to use their existing input.
    enum class State : std::uint8_t
    {
        AwaitInitialRelease,
        Held,
        ReleaseArmed,
        ReleaseRequested,
    };

    [[nodiscard]] constexpr bool ownsInput(
        bool pendingTransfer, std::uint64_t committedGrab, std::uint64_t heldGrab) noexcept
    {
        return pendingTransfer || (committedGrab != 0 && committedGrab == heldGrab);
    }

    [[nodiscard]] constexpr bool advance(
        State& state, bool held, bool pressed, bool released, bool releaseAllowed = true) noexcept
    {
        if (!releaseAllowed) {
            // A gesture spent on a holster/provider cannot become a delayed
            // ground drop when that owner yields input back to ROCK.
            if (state != State::AwaitInitialRelease || !held) {
                state = State::Held;
            }
            return false;
        }
        switch (state) {
        case State::AwaitInitialRelease:
            if (!held) {
                state = State::Held;
            }
            break;
        case State::Held:
            if (pressed) {
                // A complete click may arrive in one input publication.
                state = released && !held ? State::ReleaseRequested : State::ReleaseArmed;
            }
            break;
        case State::ReleaseArmed:
            if (!held) {
                state = State::ReleaseRequested;
            }
            break;
        case State::ReleaseRequested:
            break;
        }
        // Persist a completed request through deferred reference/body creation.
        return state == State::ReleaseRequested;
    }
}

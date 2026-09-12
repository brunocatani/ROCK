#pragma once

#include <cstdint>

namespace rock::virtual_holsters
{
    struct Snapshot
    {
        bool ready{ false };
        bool isLeft{ false };
        bool inZone{ false };
        std::uint32_t slot{ 0 };
        std::uint32_t buttonId{ 0 };
    };

    // Frame-thread only, after GameLoaded. The optional F4SE DLL and its API
    // singleton live for the process; ROCK never loads or owns that DLL.
    [[nodiscard]] bool isLoaded();
    [[nodiscard]] Snapshot readSnapshot() noexcept;

    struct HandState
    {
        std::uint64_t ownershipKey{ 0 };
        bool buttonClaimed{ false };
        bool holdReleaseConsumed{ false };
    };

    struct Input
    {
        Snapshot holster{};
        std::uint64_t ownershipKey{ 0 };
        std::uint32_t grabButtonId{ 0 };
        bool isLeft{ false };
        bool weaponEngaged{ false };
        bool toggleGrab{ false };
        bool held{ false };
        bool pressed{ false };
        bool released{ false };
    };

    struct Decision
    {
        bool retainGrip{ false };
        bool consumeInput{ false };
        bool started{ false };
    };

    [[nodiscard]] inline constexpr Decision advance(
        HandState& state, const Input& input) noexcept
    {
        const bool occupied = input.weaponEngaged && input.ownershipKey != 0;
        if (state.ownershipKey != input.ownershipKey || !occupied || input.toggleGrab) {
            state.holdReleaseConsumed = false;
        }
        state.ownershipKey = input.ownershipKey;

        const bool inZone = occupied && input.holster.ready &&
            input.holster.inZone && input.holster.slot >= 1 && input.holster.slot <= 7 &&
            input.holster.buttonId == input.grabButtonId &&
            input.holster.isLeft == input.isLeft;
        const bool wasClaimed = state.buttonClaimed;
        const bool started = inZone && !wasClaimed && !state.holdReleaseConsumed &&
            (input.held || input.pressed || input.released || !input.toggleGrab);

        // Drain the entire physical cycle even if native holstering has already
        // removed the weapon or the hand has left the sphere. It cannot become
        // another weapon's toggle press or a world-object grab.
        state.buttonClaimed = input.held && (inZone || wasClaimed);

        if (!input.toggleGrab && occupied) {
            if ((inZone || wasClaimed) && !input.held) {
                state.holdReleaseConsumed = true;
            } else if (!inZone && !wasClaimed && input.pressed) {
                // A hold-mode release spent on holstering must not turn into a
                // delayed drop on sphere exit. A fresh squeeze rearms release.
                state.holdReleaseConsumed = false;
            }
        }

        const bool consumed = inZone || wasClaimed || state.holdReleaseConsumed;
        return {
            .retainGrip = occupied && consumed,
            .consumeInput = consumed,
            .started = started,
        };
    }
}

#pragma once

#include <cstdint>

/*
 * The game's own Workshop mode entry. The game decides everything about the
 * workshop (inside a buildable area, not already building, available to the
 * player); ROCK only chooses when to ask. Main thread only: the frame thread
 * or the input dispatch that runs on it.
 */
namespace rock::native_workshop
{
    enum class Status : std::uint8_t
    {
        // In a workshop's buildable area and free to enter (or entered).
        Available = 0,
        OutsideBuildArea = 1,
        AlreadyInWorkshop = 2,
        WorkshopUnavailable = 3,
        // ROCK could not verify the game's functions at load.
        Unsupported = 0xFF,
    };

    // Verifies the game's Workshop query and entry once; without them every call reports Unsupported.
    void install();

    // Whether entering Workshop mode here would succeed. Changes nothing.
    [[nodiscard]] Status query();

    // Enters Workshop mode when query() is Available; returns the game's answer.
    Status enter();

    [[nodiscard]] const char* statusName(Status status) noexcept;
}

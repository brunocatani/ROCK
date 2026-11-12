#pragma once

#include <cstdint>

/*
 * Where the native activate prompts (the world_Rollover "Open", "Talk",
 * "Take" prompts each wand shows for the object it points at) appear, chosen
 * by Prompts.ActivatePromptPlacement in ROCK.toml and applied every frame:
 *
 * - BothHands: the game's own behaviour. Each wand carries its own prompt, so
 *   two wands pointing at the same object show two identical prompts.
 * - CloserHand: each wand keeps its prompt, but when both point at the same
 *   object only the hand closer to it shows one; the other is hidden.
 * - AtObject: each prompt stands upright at the object its wand points at,
 *   turned toward the player about the vertical, at one fixed world size;
 *   when both wands point at the same object it shows once.
 *
 * Only the prompts' placement and visibility change. Which object each wand
 * picks, what the prompts say and how activation works stay native.
 *
 * The game builds one world-space model per wand (WSInteractRolloverPrimary
 * and WSInteractRolloverSecondaryModel) and attaches each root to its wand
 * node. The game thread moves those roots between the wand, a ROCK carrier
 * under the room node that stands at the object, and a culled ROCK carrier
 * that hides a duplicate, and returns them to their wands when the mode
 * changes.
 */
namespace rock::native_activate_prompts
{
    // Prompts.ActivatePromptPlacement values.
    enum class Placement : std::uint8_t
    {
        BothHands = 1,
        CloserHand = 2,
        AtObject = 3,
    };

    // Plugin load. False when the native code does not match: the prompts then
    // stay native whatever the setting says.
    bool install() noexcept;

    // Game thread, every frame: places both wand prompts for the placement.
    void update(Placement placement) noexcept;
}

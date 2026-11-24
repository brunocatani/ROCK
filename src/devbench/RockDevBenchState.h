/*
 * What the devbench tool's state action reports: one frame of ROCK's state,
 * filled on the frame thread at FrameEnd while the tool is armed, and turned
 * into JSON on devbench's listener thread, possibly after the frame's objects
 * are gone. So it holds plain values only: no pointers to forms, references or
 * nodes. The const char* names point at string literals.
 */
#pragma once

#include <array>
#include <cstdint>
#include <string>

namespace rock::devbench
{
    struct HandState
    {
        const char* state = "Idle";
        // What the hand holds: none, object, loose-weapon (ROCK grabs), equipped-weapon (the firing grip or the
        // second weapon) or weapon-part (a part grip on the equipped weapon).
        const char* held = "none";
        std::uint32_t heldFormId = 0;
        std::string heldName;
        std::uint32_t selectedFormId = 0;
        std::string selectedName;
        const char* selectedKind = "";
        bool selectionFar = false;
    };

    struct WeaponState
    {
        std::uint32_t formId = 0;
        std::string name;
        // The hand that holds its firing grip, or "none".
        const char* hand = "none";
    };

    struct RockState
    {
        bool frikBound = false;
        std::uint32_t frikApiVersion = 0;
        std::uint32_t providerGeneration = 0;
        std::uint32_t skeletonGeneration = 0;
        std::uint64_t configRevision = 0;
        std::uint64_t configFileRevision = 0;

        // A PhysicsInteraction exists: FRIK's skeleton is ready. The fields below are its.
        bool interaction = false;
        bool playerAvailable = false;
        bool weaponDrawn = false;
        bool menuBlocking = false;
        bool inputMenuBlocking = false;
        bool loading = false;
        // [0] left, [1] right.
        std::array<HandState, 2> hands{};
        const char* twoHandedGrip = "Inactive";
        // Equipment index 0, the player's own weapon, and index 1, ROCK's second weapon.
        std::array<WeaponState, 2> weapons{};
    };
}

#pragma once

#include "RE/NetImmerse/NiPoint.h"

#include <cstdint>

namespace RE
{
    class Actor;
    class bhkCharacterController;
}

namespace rock::character_controller_runtime
{
    struct CharacterControllerPositionSample
    {
        // The live bhkCharacterController position is natively expressed in
        // Havok units. Pointer values are identity/telemetry tokens only; they
        // are never dereferenced after this sample returns.
        RE::NiPoint3 positionHavok{};
        std::uintptr_t controllerIdentity = 0;
        std::uintptr_t controllerVtable = 0;
        bool valid = false;
    };

    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept;
    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept;

    // Read-only snapshot of the character controller's live physics position.
    // GetPositionImpl(false) is valid for both FO4VR controller implementations
    // and, inside a world-step callback, observes movement already completed by
    // the character-manager graph without mutating controller state.
    CharacterControllerPositionSample samplePlayerCharacterControllerPositionHavok() noexcept;

    // Player actor world position in game units. Used by runtime scans that
    // need a stable player-space origin; it is not applied to held-object or
    // visual-hand targets.
    bool tryGetPlayerActorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept;
}

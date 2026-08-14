#pragma once

#include "RE/NetImmerse/NiPoint.h"

namespace RE
{
    class Actor;
    class bhkCharacterController;
}

namespace rock::character_controller_runtime
{
    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept;
    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept;

    // Player actor world position in game units. Used by runtime scans that
    // need a stable player-space origin; it is not applied to held-object or
    // visual-hand targets.
    bool tryGetPlayerActorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept;
}

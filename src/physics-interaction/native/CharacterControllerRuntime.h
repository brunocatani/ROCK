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

    // Player locomotion velocity (character-controller cachedLinearVelocity), GAME UNITS.
    //
    // Raw read: walks Actor+0x300 (currentProcess) -> +0x08 (middleHigh) -> +0x3E8 (charController) ->
    // +0x250 (cachedLinearVelocity), all Ghidra-verified against the FO4VR binary
    // (PlayerCharacter::GetLinearVelocity @ 140dc80f0 + helper 140ec70b0). This deliberately does NOT use
    // CommonLibF4VR's MiddleHighProcessData::charController (declared 0x3E0, flat) because VR is 0x3E8 --
    // reading through the CommonLib member yields a bad pointer. Per-hop null gates fail closed; SEH-guarded.
    bool tryGetPlayerLocomotionVelocityRawGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept;

    // Accessor read: the game's own PlayerCharacter::GetLinearVelocity (verified vtable slot 0xAC), which
    // resolves the CC internally with the engine's correct offsets. Ground-truth cross-check for the raw read.
    bool tryGetPlayerLocomotionVelocityAccessorGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept;
}

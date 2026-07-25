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
    //
    // Consumer: the grab-authority room-velocity feed-forward (one-substep target prediction at the
    // physics flush). This is a physics-clock signal read on the physics thread by design.
    bool tryGetPlayerLocomotionVelocityRawGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept;

    // Player ROOM ANCHOR world position, GAME UNITS -- the locomotion anchor the held-object jag
    // correction differences per physics flush (never integrates; see GrabLocomotionJag.h).
    //
    // Raw read: the velocity chain above, then -> +0x470 (character impl) -> +0x70 (hkVector4f
    // position, HAVOK units, converted here). Both offsets were read out of
    // bhkCharacterController::GetPositionImpl (FO4VR 0x141e4dfd0, CommonLib vtable slot 0x33) by raw
    // disassembly 2026-07-25; that function loads the position from exactly [[this+0x470]+0x70] and,
    // when its a_applyCenterOffset argument is false, returns it unmodified. Corroborated by its
    // mirror SetPositionImpl (0x141e4df60), GetKeepDistanceImpl (0x141e4df40), InitPhysicsSystemImpl
    // (0x141e4da60), and by this+0x1A0 matching CommonLib's declared rotCenter.
    //
    // The member read is deliberate rather than a vtable call: a wrong vtable index would CALL an
    // arbitrary function, while a wrong member offset only yields implausible floats that the
    // caller's plausibility gate rejects. Fails closed, SEH-guarded, per-hop null gates.
    bool tryGetPlayerRoomAnchorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept;

    // Player ACTOR world position (TESObjectREFR::data.location), GAME UNITS -- the second room
    // anchor candidate for the jag correction. Unlike the controller position above this advances on
    // the game/render update rather than inside the physics step, which is the side of the clock
    // boundary the camera follows. Which of the two actually carries the camera's per-frame staircase
    // is decided by logged data, not by argument: both are sampled and logged every flush while
    // iGrabLocomotionJagAnchor selects the one that drives the correction.
    bool tryGetPlayerActorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept;
}

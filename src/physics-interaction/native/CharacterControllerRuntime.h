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
    enum class PlayerControllerImplementation : std::uint8_t
    {
        Unknown,
        Proxy,
        RigidBody,
    };

    enum class PlayerSupportState : std::uint8_t
    {
        Unsupported = 0,
        Sliding = 1,
        Supported = 2,
    };

    struct PlayerControllerState
    {
        bool valid{ false };
        bool positionValid{ false };
        bool velocityValid{ false };
        bool shapeValid{ false };
        bool supportNormalValid{ false };
        bool penetrationChecked{ false };
        bool penetrating{ false };
        PlayerControllerImplementation implementation{
            PlayerControllerImplementation::Unknown
        };
        PlayerSupportState supportState{ PlayerSupportState::Unsupported };
        std::uintptr_t controllerIdentity{ 0 };
        RE::NiPoint3 positionGame{};
        RE::NiPoint3 velocityGame{};
        RE::NiPoint3 supportNormal{};
        float radiusGame{ 0.0f };
        float heightGame{ 0.0f };
    };

    RE::bhkCharacterController* tryGetActorCharacterController(RE::Actor* actor) noexcept;
    RE::bhkCharacterController* tryGetPlayerCharacterController() noexcept;

    bool tryGetPlayerControllerState(
        PlayerControllerState& outState,
        bool checkPenetration) noexcept;
    bool requestPlayerJump(float heightGameUnits) noexcept;

    // Player locomotion velocity (character-controller cachedLinearVelocity), GAME UNITS.
    //
    // Raw read: walks Actor+0x300 (currentProcess) -> +0x08 (middleHigh) -> +0x3E8 (charController) ->
    // +0x250 (cachedLinearVelocity), all Ghidra-verified against the FO4VR binary
    // (PlayerCharacter::GetLinearVelocity @ 140dc80f0 + helper 140ec70b0). This deliberately does NOT use
    // CommonLibF4VR's MiddleHighProcessData::charController (declared 0x3E0, flat) because VR is 0x3E8 --
    // reading through the CommonLib member yields a bad pointer. Per-hop null gates fail closed; SEH-guarded.
    //
    // This is a physics-clock signal and must only be read from an engine-owned
    // phase where the controller state is stable.
    bool tryGetPlayerLocomotionVelocityRawGameUnits(RE::NiPoint3& outVelocityGameUnits) noexcept;

    // Player character-controller world position, in game units.
    //
    // Raw read: the velocity chain above, then -> +0x470 (character impl) -> +0x70 (hkVector4f
    // position, HAVOK units, converted here). Both offsets were read out of
    // bhkCharacterController::GetPositionImpl (FO4VR 0x141e4dfd0, CommonLib vtable slot 0x33) by raw
    // disassembly 2026-07-25; that function loads the position from exactly [[this+0x470]+0x70] and,
    // when its a_applyCenterOffset argument is false, returns it unmodified. Corroborated by its
    // mirror SetPositionImpl (0x141e4df60), GetKeepDistanceImpl (0x141e4df40), InitPhysicsSystemImpl
    // (0x141e4da60), and by this+0x1A0 matching CommonLib's declared rotCenter.
    //
    // The member read is deliberate rather than a vtable call: a wrong vtable
    // index would call an arbitrary function. The read fails closed, is
    // SEH-guarded, and checks each pointer hop.
    bool tryGetPlayerRoomAnchorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept;

    // Player actor world position (TESObjectREFR::data.location), in game units.
    bool tryGetPlayerActorPositionGameUnits(RE::NiPoint3& outPositionGameUnits) noexcept;
}

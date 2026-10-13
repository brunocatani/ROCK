#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::playercontroller {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::PlayerController;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr float kMaxNativeJumpHeightGame = 256.0f;

    enum class PlayerControllerImplementation : std::uint32_t
    {
        Unknown = 0,
        Proxy = 1,
        RigidBody = 2,
    };

    enum class PlayerSupportState : std::uint32_t
    {
        Unsupported = 0,
        Sliding = 1,
        Supported = 2,
    };

    // Bits 7 (penetration checked) and 8 (penetrating) are retired; never
    // reuse them.
    enum class PlayerControllerStateFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        PositionValid = 1u << 1,
        VelocityValid = 1u << 2,
        ShapeValid = 1u << 3,
        SupportNormalValid = 1u << 4,
        Supported = 1u << 5,
        Sliding = 1u << 6,
        Proxy = 1u << 9,
        RigidBody = 1u << 10,
    };

    // The generations must match the current publication (read them from a
    // state's sample); a stale request is refused with GenerationMismatch.
    struct PlayerControllerJumpRequest
    {
        std::uint32_t size{ sizeof(PlayerControllerJumpRequest) };
        std::uint32_t version{ 1 };
        float heightGameUnits{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    // The player's native character controller, in game units.
    struct PlayerControllerState
    {
        std::uint32_t size{ sizeof(PlayerControllerState) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint32_t flags{ 0 };
        PlayerControllerImplementation implementation{ PlayerControllerImplementation::Unknown };
        PlayerSupportState supportState{ PlayerSupportState::Unsupported };
        Point3 positionGame{};
        Point3 velocityGame{};
        Point3 supportNormalGame{};
        float radiusGame{ 0.0f };
        float heightGame{ 0.0f };
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Game thread, Read.
        Status(ROCK_CALL* getPlayerControllerState)(OwnerToken owner, PlayerControllerState* outState) noexcept;
        // Game thread, Write. Height in (0, kMaxNativeJumpHeightGame].
        Status(ROCK_CALL* requestPlayerControllerJump)(OwnerToken owner, const PlayerControllerJumpRequest* request) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
    };
}

#pragma pack(pop)

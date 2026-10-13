#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::playercontroller {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::PlayerController;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr float kMaxNativeJumpHeightGame = 256.0f;


    enum class PlayerControllerImplementationV1 : std::uint32_t
    {
        Unknown = 0,
        Proxy = 1,
        RigidBody = 2,
    };

    enum class PlayerSupportStateV1 : std::uint32_t
    {
        Unsupported = 0,
        Sliding = 1,
        Supported = 2,
    };

    enum class PlayerControllerStateFlagV1 : std::uint32_t
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

    struct PlayerControllerJumpRequestV1
    {
        std::uint32_t size{
            sizeof(PlayerControllerJumpRequestV1)
        };
        std::uint32_t version{ 1 };
        float heightGameUnits{ 0.0f };
        std::uint32_t reserved0{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };

    struct PlayerControllerStateV1
    {
        std::uint32_t size{ sizeof(PlayerControllerStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t flags{ 0 };
        PlayerControllerImplementationV1 implementation{
            PlayerControllerImplementationV1::Unknown
        };
        PlayerSupportStateV1 supportState{
            PlayerSupportStateV1::Unsupported
        };
        std::uint32_t reserved0{ 0 };
        Point3 positionGame{};
        Point3 velocityGame{};
        Point3 supportNormalGame{};
        float radiusGame{ 0.0f };
        float heightGame{ 0.0f };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[4]{};
    };


    // Permission bits are local to this interface major. Discovery grants none.
    enum class PermissionV1 : std::uint32_t { Read=1, Write=2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* getPlayerControllerStateV1)(std::uint64_t ownerToken, PlayerControllerStateV1* outState) noexcept;
        Status(ROCK_CALL* requestPlayerControllerJumpV1)(std::uint64_t ownerToken, const PlayerControllerJumpRequestV1* request) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
    };
}

#pragma pack(pop)

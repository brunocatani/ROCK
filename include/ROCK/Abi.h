#pragma once
#include <cstdint>
#include <cstddef>
#include <type_traits>
#define ROCK_CALL __cdecl

// Frozen common value vocabulary. Feature records belong to their own headers.
#pragma pack(push, 8)
namespace rock::api {
    enum class Hand : std::uint32_t
    {
        None = 0,
        Right = 1,
        Left = 2,
    };

    enum class Status : std::uint32_t
    {
        Ok = 0,
        NotReady = 1,
        InvalidArgument = 2,
        InvalidSize = 3,
        UnsupportedVersion = 4,
        CapacityFull = 5,
        OwnerNotRegistered = 6,
        OwnerConflict = 7,
        PermissionDenied = 8,
        WorldNotReady = 9,
        TargetInvalid = 10,
        TargetUnavailable = 11,
        HandUnavailable = 12,
        HandBusy = 13,
        ObjectAlreadyOwned = 14,
        RequestQueued = 15,
        RequestNotFound = 16,
        WrongThread = 17,
        AlreadyCommitted = 18,
        UnknownInterface = 100,
        UnsupportedMajor = 101,
        UnsupportedMinor = 102,
        TableTooSmall = 103,
        OwnerRevoked = 104,
        InternalError = 105,
        Busy = 106,
        Ambiguous = 107,
        GenerationMismatch = 108,
    };

    struct Transform
    {
        float rotate[9]{};
        float translate[3]{};
        float scale{ 1.0f };
    };

    struct Point3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    enum class PowerArmorPoint : std::uint32_t
    {
        None = 0,
        LeftArmorHand = 1,
        RightArmorHand = 2,
    };

    struct Bounds3
    {
        Point3 min{};
        Point3 max{};
        std::uint32_t valid{ 0 };
        std::uint32_t reserved{ 0 };
    };

    using OwnerToken = std::uint64_t;
    enum class InterfaceId : std::uint32_t {
        Core=1, Hands=2, Collision=3, Grab=4, Touch=5, Weapon=6,
        WeaponParts=7, Animation=8, Input=9,
        // 10 (References) is retired: its queries are part of Grab. Never reuse.
        References=10,
        PlayerController=11, Diagnostics=12, Configuration=13, NativeInteraction=14
    };
    struct ReferenceKey {
        std::uint32_t referenceFormId{};
        std::uint32_t referenceNativeHandle{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
    };
    struct Sample {
        std::uint64_t frameIndex{};
        std::uint64_t publicationSequence{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        std::uint32_t collisionGeneration{};
    };
    struct Stream {
        std::uint64_t oldestSequence{};
        std::uint64_t latestSequence{};
        std::uint64_t nextSequence{};
        std::uint64_t lostCount{};
        std::uint32_t copiedCount{};
        std::uint32_t remainingCount{};
    };
    static_assert(sizeof(void*) == 8, "ROCK requires a 64-bit consumer");
    static_assert(sizeof(Hand) == 4 && sizeof(Status) == 4);

}

#pragma pack(pop)

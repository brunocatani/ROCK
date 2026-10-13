#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::input {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Input;
    inline constexpr std::uint32_t kMajor = 1;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxSuppressions = 32;
    inline constexpr std::uint32_t kMaxSuppressionLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;


    enum class HandInputSuppressionFlagV1 : std::uint32_t
    {
        None = 0,
        SuppressNormalGrabPress = 1u << 0,
        SuppressGrabRelease = 1u << 1,
        SuppressHeldWeaponTriggerEquip = 1u << 2,
        SuppressGameplayCandidates = 1u << 3,
        SuppressOpenVrGameInput = 1u << 4,
        SuppressNativeVats = 1u << 5,
        SuppressNativeVans = 1u << 6,
        SuppressGrenadeQuickDraw = 1u << 7,
        ReserveTriggerGripChord = 1u << 8,
        ReserveButtonChord = 1u << 9,
        SuppressConfigModeChord =
            static_cast<std::uint32_t>(SuppressNormalGrabPress) |
            static_cast<std::uint32_t>(SuppressGrabRelease) |
            static_cast<std::uint32_t>(SuppressHeldWeaponTriggerEquip) |
            static_cast<std::uint32_t>(SuppressGameplayCandidates),
    };

    enum class InputAvailabilityReasonV1 : std::uint32_t
    {
        Available = 0,
        HookNotSampled = 1,
        BlockingMenu = 2,
        ReleaseToRearm = 3,
        InvalidButton = 4,
    };

    enum class LogicalInputActionV1 : std::uint32_t
    {
        Jump = 1,
    };

    enum class SuppressionInvalidationReasonV1 : std::uint32_t
    {
        None = 0,
        Expired = 1,
        GenerationChanged = 2,
        OwnerUnregistered = 3,
        ProviderLost = 4,
        ExplicitClear = 5,
        CallbackFault = 6,
    };

    struct HandInputSuppressionRequestV1
    {
        std::uint32_t size{ sizeof(HandInputSuppressionRequestV1) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t chordButtonsLow[2]{};
        std::uint32_t chordButtonsHigh[2]{};
        std::uint32_t reserved[4]{};
    };

    enum class NativeInputContextFlagV1 : std::uint32_t
    {
        Available = 1u << 0,
        MenuActive = 1u << 1,
        PrimaryActivationTarget = 1u << 2,
    };

    struct RawWandButtonStateV1
    {
        std::uint32_t size{ sizeof(RawWandButtonStateV1) };
        std::uint32_t version{ 1 };
        std::uint32_t available{ 0 };
        std::uint32_t held{ 0 };
        std::uint64_t sampleSequence{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        InputAvailabilityReasonV1 availabilityReason{
            InputAvailabilityReasonV1::HookNotSampled
        };
    };

    struct LogicalInputActionStateV1
    {
        std::uint32_t size{ sizeof(LogicalInputActionStateV1) };
        std::uint32_t version{ 1 };
        LogicalInputActionV1 action{
            LogicalInputActionV1::Jump
        };
        std::uint32_t available{ 0 };
        std::uint32_t held{ 0 };
        InputAvailabilityReasonV1 availabilityReason{
            InputAvailabilityReasonV1::HookNotSampled
        };
        std::uint64_t sampleSequence{ 0 };
        std::uint64_t pressSequence{ 0 };
        std::uint64_t frameIndex{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[4]{};
    };

    struct HandInputSuppressionStateV1
    {
        std::uint32_t size{ sizeof(HandInputSuppressionStateV1) };
        std::uint32_t version{ 1 };
        std::uint64_t frameIndex{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t callerFlags{ 0 };
        std::uint32_t effectiveFlags{ 0 };
        std::uint32_t callerLeaseActive{ 0 };
        std::uint64_t callerExpiresAfterFrame{ 0 };
        std::uint32_t callerRemainingFrames{ 0 };
        SuppressionInvalidationReasonV1 lastInvalidationReason{
            SuppressionInvalidationReasonV1::None
        };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t reserved[5]{};
    };


    // Permission bits are local to this interface major. Discovery grants none.
    enum class PermissionV1 : std::uint32_t { Read=1, Write=2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct EventV1 {
        std::uint32_t size{sizeof(EventV1)};
        std::uint32_t kind{};
        std::uint64_t sequence{};
        std::uint64_t frameIndex{};
        std::uint64_t ownerToken{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        std::uint32_t authority{}; std::uint32_t reason{};
    };
    enum class EventKindV1 : std::uint32_t { AuthorityLost=1 };

    struct ApiV1 {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor;
        static constexpr std::uint32_t minorVersion=kMinor;
        Status(ROCK_CALL* setHandInputSuppressionV1)(std::uint64_t ownerToken, const HandInputSuppressionRequestV1* request) noexcept;
        Status(ROCK_CALL* clearHandInputSuppressionV1)(std::uint64_t ownerToken, Hand hand) noexcept;
        Status(ROCK_CALL* getRawWandButtonStateV1)(OwnerToken ownerToken, Hand hand, std::uint32_t buttonId, RawWandButtonStateV1* outState) noexcept;
        Status(ROCK_CALL* isNativePipboyInputSuppressedV1)(OwnerToken ownerToken, std::uint32_t* outValue) noexcept;
        Status(ROCK_CALL* getHandInputSuppressionStateV1)(std::uint64_t ownerToken, Hand hand, HandInputSuppressionStateV1* outState) noexcept;
        Status(ROCK_CALL* getLogicalInputActionStateV1)(std::uint64_t ownerToken, LogicalInputActionV1 action, LogicalInputActionStateV1* outState) noexcept;
        Status(ROCK_CALL* getRawWandThumbstickV1)(OwnerToken ownerToken, Hand hand, float* outX, float* outY) noexcept;
        Status(ROCK_CALL* getNativeInputContextV1)(OwnerToken ownerToken, std::uint32_t* outValue) noexcept;
        // Family-local publication identity; compare before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, SampleV1* outSample) noexcept;
        Status(ROCK_CALL* copyEvents)(OwnerToken, std::uint64_t afterSequence, EventV1*, std::uint32_t capacity, StreamV1*) noexcept;
    };
}

#pragma pack(pop)

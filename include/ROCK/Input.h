#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::input {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Input;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kMaxSuppressions = 32;
    inline constexpr std::uint32_t kMaxSuppressionLeaseFrames = 120;
    inline constexpr std::uint32_t kMaxWeaponInputLeaseFrames = 120;
    inline constexpr std::uint32_t kEventCapacityPerOwner = 256;

    enum class HandInputSuppressionFlag : std::uint32_t
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

    enum class InputAvailabilityReason : std::uint32_t
    {
        Available = 0,
        HookNotSampled = 1,
        BlockingMenu = 2,
        ReleaseToRearm = 3,
        InvalidButton = 4,
    };

    enum class LogicalInputAction : std::uint32_t
    {
        Jump = 1,
    };

    enum class SuppressionInvalidationReason : std::uint32_t
    {
        None = 0,
        Expired = 1,
        GenerationChanged = 2,
        OwnerUnregistered = 3,
        ProviderLost = 4,
        ExplicitClear = 5,
        CallbackFault = 6,
    };

    enum class NativeInputContextFlag : std::uint32_t
    {
        Available = 1u << 0,
        MenuActive = 1u << 1,
        PrimaryActivationTarget = 1u << 2,
    };

    // chordButtons[0] is the left wand, [1] the right: OpenVR button bits
    // (1, 2, 7, 32, 33) reserved by ReserveButtonChord. The generations guard
    // the request; zero skips that guard.
    struct HandInputSuppressionRequest
    {
        std::uint32_t size{ sizeof(HandInputSuppressionRequest) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        std::uint32_t leaseFrames{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint64_t chordButtons[2]{};
    };

    // leaseFrameIndex is the lease clock callerExpiresAfterFrame counts
    // against; sample identifies the publication.
    struct HandInputSuppressionState
    {
        std::uint32_t size{ sizeof(HandInputSuppressionState) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint64_t leaseFrameIndex{ 0 };
        std::uint64_t callerExpiresAfterFrame{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t callerFlags{ 0 };
        std::uint32_t effectiveFlags{ 0 };
        std::uint32_t callerLeaseActive{ 0 };
        std::uint32_t callerRemainingFrames{ 0 };
        SuppressionInvalidationReason lastInvalidationReason{ SuppressionInvalidationReason::None };
    };

    // Level state of one physical button. sampleSequence and
    // sampleAgeMilliseconds describe the controller input sample.
    struct RawWandButtonState
    {
        std::uint32_t size{ sizeof(RawWandButtonState) };
        std::uint32_t version{ 1 };
        std::uint32_t available{ 0 };
        std::uint32_t held{ 0 };
        std::uint64_t sampleSequence{ 0 };
        std::uint32_t sampleAgeMilliseconds{ 0 };
        InputAvailabilityReason availabilityReason{ InputAvailabilityReason::HookNotSampled };
    };

    struct LogicalInputActionState
    {
        std::uint32_t size{ sizeof(LogicalInputActionState) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint64_t sampleSequence{ 0 };
        std::uint64_t pressSequence{ 0 };
        LogicalInputAction action{ LogicalInputAction::Jump };
        std::uint32_t available{ 0 };
        std::uint32_t held{ 0 };
        InputAvailabilityReason availabilityReason{ InputAvailabilityReason::HookNotSampled };
        std::uint32_t sampleAgeMilliseconds{ 0 };
    };

    // Bit 0 is retired; never reuse it.
    enum class PlacementClickFlag : std::uint32_t { InputReserved = 2 };

    // A reserved right-stick click belongs to an add-on's held placement
    // action, including its release; other users yield. objectFormId is the
    // submitted candidate, zero while a spent click drains.
    struct PlacementClickState
    {
        std::uint32_t size{ sizeof(PlacementClickState) };
        std::uint32_t version{ 1 };
        std::uint32_t flags{ 0 };
        std::uint32_t objectFormId{ 0 };
        Sample sample{};
    };

    enum class ReloadGestureFlag : std::uint32_t
    {
        // Suppress only that hand's native trigger button, axis and actions
        // while the lease holds; firing resumes after a physical release.
        BlockFire = 1u << 0,
        // ROCK's gameplay activation keeps priority: a workbench, use or take
        // target of the primary wand owns the whole press and release, and no
        // tap or scope hold is published for it. Without this flag, Accept is
        // masked from the game for the lease.
        GameplayPriority = 1u << 1,
    };

    /*
     * Reload gesture lease on the equipped weapon's firing hand. One owner at
     * a time; the request names the current weapon (form, generation key) and
     * sample (current or previous frame). ROCK classifies the Accept button
     * (X left, A right) with fManualScopeHoldSeconds: a short release adds one
     * to tapSequence; a hold stays ROCK scope input and never becomes a tap.
     * Menus, input loss, hand changes and changing the lease (including its
     * flags) cancel an unfinished gesture and require a release before
     * rearming. UI suppression wins.
     */
    struct ReloadGestureRequest
    {
        std::uint32_t size{ sizeof(ReloadGestureRequest) };
        std::uint32_t version{ 1 };
        std::uint64_t weaponGenerationKey{ 0 };
        Sample sample{};
        std::uint32_t weaponFormId{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t leaseFrames{ 2 };
        std::uint32_t flags{ 0 };
    };

    struct ReloadGestureState
    {
        std::uint32_t size{ sizeof(ReloadGestureState) };
        std::uint32_t version{ 1 };
        OwnerToken owner{ 0 };
        std::uint64_t tapSequence{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        Sample sample{};
        std::uint32_t active{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t held{ 0 };
        std::uint32_t scopeHeld{ 0 };
    };

    /*
     * Support-trigger lease: the physical trigger of the supporting hand of an
     * active two-hand grip. Suppresses only that trigger's game button, axis
     * and native action. A new claim or unavailable input requires a release
     * before a fresh press; pressSequence is monotonic across leases, so read
     * a baseline after acquiring. Clearing, expiry, hand changes and owner
     * loss drain a held trigger before returning it to gameplay. UI capture
     * wins and cancels pending input.
     */
    struct SupportTriggerRequest
    {
        std::uint32_t size{ sizeof(SupportTriggerRequest) };
        std::uint32_t version{ 1 };
        std::uint64_t weaponGenerationKey{ 0 };
        Sample sample{};
        std::uint32_t weaponFormId{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t leaseFrames{ 2 };
    };

    struct SupportTriggerState
    {
        std::uint32_t size{ sizeof(SupportTriggerState) };
        std::uint32_t version{ 1 };
        OwnerToken owner{ 0 };
        std::uint64_t pressSequence{ 0 };
        std::uint64_t expiresAfterFrame{ 0 };
        Sample sample{};
        std::uint32_t active{ 0 };
        Hand hand{ Hand::None };
        std::uint32_t held{ 0 };
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1, Write = 2 };
    inline constexpr std::uint32_t kSupportedPermissions = 3;

    struct Event
    {
        std::uint32_t size{ sizeof(Event) };
        std::uint32_t kind{};
        std::uint64_t sequence{};
        std::uint64_t frameIndex{};
        std::uint64_t ownerToken{};
        std::uint32_t worldGeneration{};
        std::uint32_t skeletonGeneration{};
        std::uint32_t providerGeneration{};
        std::uint32_t authority{};
        std::uint32_t reason{};
    };
    enum class EventKind : std::uint32_t { AuthorityLost = 1 };

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Game thread (Write); lease up to kMaxSuppressionLeaseFrames.
        Status(ROCK_CALL* setHandInputSuppression)(OwnerToken owner, const HandInputSuppressionRequest* request) noexcept;
        // Hand::None clears both hands.
        Status(ROCK_CALL* clearHandInputSuppression)(OwnerToken owner, Hand hand) noexcept;
        // Game thread (Read).
        Status(ROCK_CALL* getHandInputSuppressionState)(OwnerToken owner, Hand hand, HandInputSuppressionState* outState) noexcept;
        Status(ROCK_CALL* getRawWandButtonState)(OwnerToken owner, Hand hand, std::uint32_t buttonId, RawWandButtonState* outState) noexcept;
        Status(ROCK_CALL* getRawWandThumbstick)(OwnerToken owner, Hand hand, float* outX, float* outY) noexcept;
        Status(ROCK_CALL* isNativePipboyInputSuppressed)(OwnerToken owner, std::uint32_t* outValue) noexcept;
        // NativeInputContextFlag bits; 0 when the input hook is unavailable.
        Status(ROCK_CALL* getNativeInputContext)(OwnerToken owner, std::uint32_t* outValue) noexcept;
        Status(ROCK_CALL* getLogicalInputActionState)(OwnerToken owner, LogicalInputAction action, LogicalInputActionState* outState) noexcept;
        Status(ROCK_CALL* getPlacementClickState)(OwnerToken owner, PlacementClickState* outState) noexcept;
        // Game thread; Write for set/clear, Read for get. Lease 1 to
        // kMaxWeaponInputLeaseFrames frames.
        Status(ROCK_CALL* setReloadGesture)(OwnerToken owner, const ReloadGestureRequest* request) noexcept;
        Status(ROCK_CALL* clearReloadGesture)(OwnerToken owner) noexcept;
        Status(ROCK_CALL* getReloadGesture)(OwnerToken owner, ReloadGestureState* outState) noexcept;
        Status(ROCK_CALL* setSupportTrigger)(OwnerToken owner, const SupportTriggerRequest* request) noexcept;
        Status(ROCK_CALL* clearSupportTrigger)(OwnerToken owner) noexcept;
        Status(ROCK_CALL* getSupportTrigger)(OwnerToken owner, SupportTriggerState* outState) noexcept;
        // Any thread. Identity of the current publication.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
        // Any thread. Copies authority-lost events after afterSequence.
        Status(ROCK_CALL* copyEvents)(OwnerToken owner, std::uint64_t afterSequence, Event* outEvents, std::uint32_t capacity, Stream* outStream) noexcept;
    };
}

#pragma pack(pop)

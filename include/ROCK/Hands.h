#pragma once
#include "Abi.h"

#pragma pack(push, 8)
namespace rock::api::hands {
    inline constexpr InterfaceId kInterfaceId = InterfaceId::Hands;
    inline constexpr std::uint32_t kMajor = 2;
    inline constexpr std::uint32_t kMinor = 0;
    inline constexpr std::uint32_t kFingerLocalTransformCount = 15;

    // Bits 2 (primary role), 3 (offhand role) and 4 (scene node) are retired;
    // never reuse them.
    enum class HandFrameFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        Left = 1u << 1,
        RootFlattenedAuthority = 1u << 5,
        PresentedVisual = 1u << 6,
    };

    enum class PresentedHandPoseFlag : std::uint32_t
    {
        None = 0,
        Valid = 1u << 0,
        HandWorldValid = 1u << 1,
        FingerLocalsValid = 1u << 2,
        RootFlattenedReadback = 1u << 3,
    };

    // One hand's world transform. getHandFrame copies ROCK's control
    // publication; getPresentedHandFrame copies the last completed
    // presentation joined to the control publication captured with it.
    struct HandFrame
    {
        std::uint32_t size{ sizeof(HandFrame) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        Transform transform{};
        Sample sample{};
    };

    // The rendered wrist and the 15 finger joints (thumb, index, middle, ring,
    // pinky; three joints each), each local to its parent joint. Bit N of
    // fingerLocalTransformMask marks fingerLocalTransforms[N] as captured.
    struct PresentedHandPose
    {
        std::uint32_t size{ sizeof(PresentedHandPose) };
        std::uint32_t version{ 1 };
        Hand hand{ Hand::None };
        std::uint32_t flags{ 0 };
        Transform handWorld{};
        std::uint32_t fingerLocalTransformMask{ 0 };
        Transform fingerLocalTransforms[kFingerLocalTransformCount]{};
        std::uint64_t frameIndex{ 0 };
        std::uint64_t presentationSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
    };

    enum class HeadPoseFlag : std::uint32_t
    {
        None = 0,
        TransformValid = 1u << 0,
        ForwardValid = 1u << 1,
    };

    struct HeadPose
    {
        std::uint32_t size{ sizeof(HeadPose) };
        std::uint32_t version{ 1 };
        Sample sample{};
        std::uint32_t flags{ 0 };
        Transform transform{};
        float forwardWorld[3]{};
    };

    inline constexpr std::uint32_t kNoWeaponSlot = 0xFFFFFFFFu;

    // A hand's grip on an equipped weapon. Firing: that weapon's firing grip.
    // Support: any other grip of that weapon (its support grip or another
    // part grip). None: no equipped-weapon grip.
    enum class HandRole : std::uint32_t
    {
        None = 0,
        Firing = 1,
        Support = 2,
    };

    struct HandWeaponRole
    {
        // The weapon slot whose grip the hand holds; kNoWeaponSlot for None.
        std::uint32_t weaponSlot{ kNoWeaponSlot };
        HandRole role{ HandRole::None };
    };

    // What each physical hand holds of the equipped weapons in this
    // publication. A hand holds at most one grip of one weapon. Roles follow
    // grip transfers; they are not the player's configured default hand.
    struct Roles
    {
        std::uint32_t size{ sizeof(Roles) };
        std::uint32_t version{ 1 };
        Sample sample{};
        HandWeaponRole right{};
        HandWeaponRole left{};
    };

    // Permission bits are local to this interface major. Discovery grants none.
    enum class Permission : std::uint32_t { Read = 1 };
    inline constexpr std::uint32_t kSupportedPermissions = 1;

    struct Api
    {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor;
        static constexpr std::uint32_t minorVersion = kMinor;
        // Any thread. Copied control publication; check flags and sample.
        Status(ROCK_CALL* getHandFrame)(OwnerToken owner, Hand hand, HandFrame* outFrame) noexcept;
        // Game thread. Last completed presentation with its own frame and
        // generations. Early phases may return the preceding frame; before the
        // first completed capture the result is NotReady.
        Status(ROCK_CALL* getPresentedHandFrame)(OwnerToken owner, Hand hand, HandFrame* outFrame) noexcept;
        Status(ROCK_CALL* getPresentedHandPose)(OwnerToken owner, Hand hand, PresentedHandPose* outPose) noexcept;
        // Any thread. Copied control publication.
        Status(ROCK_CALL* getHeadPose)(OwnerToken owner, HeadPose* outPose) noexcept;
        Status(ROCK_CALL* getRoles)(OwnerToken owner, Roles* outRoles) noexcept;
        // Any thread. Identity of the current publication; compare it with a
        // record's sample before combining readbacks.
        Status(ROCK_CALL* getSample)(OwnerToken owner, Sample* outSample) noexcept;
    };
}

#pragma pack(pop)

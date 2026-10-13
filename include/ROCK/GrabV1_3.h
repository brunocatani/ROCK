#pragma once
#include "GrabV1_2.h"

#pragma pack(push, 8)
namespace rock::api::grab::v1_3 {
    inline constexpr std::uint32_t kMajor=1, kMinor=3;
    struct HeldObjectTarget {
        std::uint32_t size{sizeof(HeldObjectTarget)};
        Hand hand{Hand::None};
        std::uint32_t referenceFormId{};
        std::uint32_t leaseFrames{2};
        SampleV1 sample{};
        std::uint64_t gripSequence{};
        std::uint32_t connectedWeaponFormId{};
        std::uint32_t reserved{};
        std::uint64_t connectedWeaponGeneration{};
        Transform rootWorld{};
    };
    struct HeldObjectPose {
        std::uint32_t size{sizeof(HeldObjectPose)};
        Hand hand{Hand::None};
        std::uint32_t referenceFormId{},bodyId{0x7fffffffu};
        SampleV1 sample{};
        std::uint64_t gripSequence{};
        Transform rootWorld{};
    };
    // Negotiate Grab 1.3 / sizeof(Api), then bind Write. Frame thread only.
    // Guides the exact currently held loose reference through ROCK's existing
    // finite-force constraint. The reference remains dynamic and collidable;
    // this does not teleport, keyframe, hide or grab it. rootWorld uses native
    // game units / Ni basis rows. Scale must equal the current reference scale.
    // Supply the current Grab sample and gripSequence. A matching snapshot at
    // most one frame old is accepted; stale generations/grips are rejected.
    // One owner per physical hand. leaseFrames is 1..2. Releasing, changing
    // grabs, owner loss or expiry returns target authority to ordinary grabbing.
    // Two-handed/shared objects, fixed objects and loose firearms reject.
    // Optional connectedWeaponFormId + connectedWeaponGeneration must identify
    // the current equipped weapon. While renewed, only its collision pairs with
    // this held object are suppressed (e.g. insertion into a socket). Zero both
    // fields restores those contacts. World/other-object contacts stay active.
    // Pair changes are consumed before physics collision, using counted leases;
    // capacity overflow rejects rather than suppressing an incomplete body set.
    // Clear is idempotent; another owner's live lease returns OwnerConflict.
    // Old 1.0..1.2 table prefixes, requirements and behavior are unchanged.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_2::Api v1_2;
        Status(ROCK_CALL* setHeldObjectTarget)(OwnerToken,const HeldObjectTarget*) noexcept;
        Status(ROCK_CALL* clearHeldObjectTarget)(OwnerToken,Hand) noexcept;
        // Read permission, frame thread. Copies the solved rigid BODY pose
        // reconstructed at the reference root, without render prediction.
        // HandUnavailable means the observed grab no longer owns that hand.
        Status(ROCK_CALL* getHeldObjectPose)(OwnerToken,Hand,HeldObjectPose*) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1_2)==0);
    static_assert(offsetof(Api,setHeldObjectTarget)==sizeof(v1_2::Api));
    static_assert(sizeof(Api)==sizeof(v1_2::Api)+3*sizeof(void*));
}
#pragma pack(pop)

#pragma once
#include "InputV1_1.h"

#pragma pack(push,8)
namespace rock::api::input::v1_2 {
    inline constexpr InterfaceId kInterfaceId=input::kInterfaceId;
    inline constexpr std::uint32_t kMajor=1,kMinor=2,kMaximumLeaseFrames=120;
    struct ReloadInputRequest {
        std::uint32_t size{sizeof(ReloadInputRequest)},weaponFormId{};
        std::uint64_t weaponGenerationKey{};
        SampleV1 sample{};
        Hand hand{Hand::Left};
        std::uint32_t leaseFrames{2};
    };
    struct ReloadInputState {
        std::uint32_t size{sizeof(ReloadInputState)},active{};
        OwnerToken owner{};
        std::uint64_t pressSequence{},releaseSequence{},expiresAfterFrame{};
        SampleV1 sample{};
        Hand hand{Hand::None};
        std::uint32_t held{};
    };
    // Input 1.2 preserves 1.0/1.1 prefixes and Core requirements. Bind Input
    // Write for set/clear and Read for read. Owner-thread calls only. One owner
    // for the current equipped weapon; exact generation/sample required.
    // Captures only physical Accept (X on Left, A on Right), not the whole hand.
    // Raw input reads remain physical. ROCK's native reload/scope routing yields
    // for that button. Releasing/expiring drains a held gesture before rearming.
    // This does not suppress automatic/native reload producers: coordinate with
    // the animation owner before acquiring. All data is copied, no callbacks.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_1::Api v1_1;
        Status(ROCK_CALL* setReloadInput)(OwnerToken,const ReloadInputRequest*) noexcept;
        Status(ROCK_CALL* clearReloadInput)(OwnerToken) noexcept;
        Status(ROCK_CALL* getReloadInput)(OwnerToken,ReloadInputState*) noexcept;
    };
    static_assert(offsetof(Api,v1_1)==0);
    static_assert(offsetof(Api,setReloadInput)==sizeof(v1_1::Api));
    static_assert(sizeof(Api)==sizeof(v1_1::Api)+3*sizeof(void*));
    static_assert(sizeof(ReloadInputRequest)==56&&sizeof(ReloadInputState)==80);
}
#pragma pack(pop)

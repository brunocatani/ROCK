#pragma once
#include "InputV1_4.h"

#pragma pack(push, 8)
namespace rock::api::input::v1_5 {
    inline constexpr InterfaceId kInterfaceId = input::kInterfaceId;
    inline constexpr std::uint32_t kMajor = 1, kMinor = 5;
    struct SupportTriggerRequest {
        std::uint32_t size{sizeof(SupportTriggerRequest)}, weaponFormId{};
        std::uint64_t weaponGenerationKey{};
        Sample sample{};
        Hand hand{Hand::None};
        std::uint32_t leaseFrames{2};
    };
    struct SupportTriggerState {
        std::uint32_t size{sizeof(SupportTriggerState)}, active{};
        OwnerToken owner{};
        std::uint64_t pressSequence{}, expiresAfterFrame{};
        Sample sample{};
        Hand hand{Hand::None};
        std::uint32_t held{};
    };
    // Independent equipped-weapon lease: Write for set/clear, Read for get,
    // ROCK owner thread only, lease 1..120 frames and current/previous sample.
    // Only the physical supporting hand of an active two-hand grip may claim.
    // Suppresses only that physical trigger's game button/axis/native action;
    // firing remaps, grip release, sticks and Accept/scope remain independent.
    // A new claim or unavailable input requires release before a fresh press.
    // pressSequence is monotonic across leases; read a baseline after acquiring.
    // Clearing, expiry, hand changes and owner loss drain a held trigger before
    // returning it to gameplay. UI/menu capture wins and cancels pending input.
    // Existing 1.0..1.4 records, table prefix and semantics are unchanged.
    struct Api {
        static constexpr InterfaceId interfaceId = kInterfaceId;
        static constexpr std::uint32_t majorVersion = kMajor, minorVersion = kMinor;
        v1_4::Api v1_4;
        Status(ROCK_CALL* setSupportTrigger)(OwnerToken, const SupportTriggerRequest*) noexcept;
        Status(ROCK_CALL* clearSupportTrigger)(OwnerToken) noexcept;
        Status(ROCK_CALL* getSupportTrigger)(OwnerToken, SupportTriggerState*) noexcept;
    };
    static_assert(offsetof(Api, v1_4) == 0 && offsetof(Api, setSupportTrigger) == sizeof(v1_4::Api));
    static_assert(sizeof(Api) == 168 && sizeof(SupportTriggerRequest) == 56 && sizeof(SupportTriggerState) == 72);
}
#pragma pack(pop)

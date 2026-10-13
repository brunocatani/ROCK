#pragma once
#include "Abi.h"

#pragma pack(push,8)
namespace rock::api::equip_pickup
{
    inline constexpr char kExport[]="ROCK_EquipPickup_QueryV1";
    // Read-only, independently negotiated 1.0 provider; no Core owner required.
    // Called synchronously from a native pickup interceptor on the same thread
    // as ROCK's held-world-weapon equip transfer. Returns 1 only for the exact
    // reference AND complete quantity currently being acquired for equip.
    // Returns 0 outside that acquisition, for other refs/counts/threads, or
    // after scope exit. This is intent, not a promise that native equip succeeds.
    // Consumers may bypass storage admission for that acquisition only. No
    // permission is granted to subsequent ordinary pickups or bag stashes.
    struct Api {
        std::uint32_t size{sizeof(Api)},version{1};
        std::uint32_t(ROCK_CALL* isEquipPickup)(std::uint32_t reference,std::uint32_t count) noexcept;
    };
    using Query=const Api*(ROCK_CALL*)(std::uint32_t major,std::uint32_t minor,std::uint32_t bytes) noexcept;
    static_assert(sizeof(Api)==16 && offsetof(Api,isEquipPickup)==8);
}
#pragma pack(pop)

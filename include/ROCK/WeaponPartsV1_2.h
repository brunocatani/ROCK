#pragma once
#include "WeaponParts.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts::v1_2 {
    inline constexpr std::uint32_t kMajor=1,kMinor=2,kMaxCollisionSources=16;
    struct CollisionSuppression {
        std::uint32_t size{sizeof(CollisionSuppression)},leaseFrames{2};
        Sample sample{};
        std::uint64_t weaponGenerationKey{};
        std::uint32_t sourceCount{},reserved{};
        std::uint64_t sourceKeys[kMaxCollisionSources]{};
    };
    // Opt in with WeaponParts 1.2 / sizeof(Api) and Write permission. Frame
    // owner thread, including Core callbacks. Suppresses collision for generated
    // equipped-weapon bodies whose source is one of these nodes or a descendant.
    // Other weapon bodies and loose references are untouched. Does not change
    // source visibility, transforms, body existence, or weapon generation.
    // One collision owner per equipped weapon; live competing owners conflict.
    // Replaces that owner's entire selection atomically. Count 1..16, lease 1..2
    // frames, current generations and sample at most one frame old required.
    // Each source must resolve; at least one generated body must match overall.
    // Physics consumes accepted leases after frame callbacks. Clear, expiry,
    // owner/world/skeleton/weapon/collision loss restores this service's filter
    // changes through counted ownership. Renew each frame while a part is absent.
    // Existing 1.0/1.1 table prefix, records, permissions and calls are unchanged.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        ApiV1 v1;
        Status(ROCK_CALL* setCollisionSuppression)(OwnerToken,const CollisionSuppression*) noexcept;
        Status(ROCK_CALL* clearCollisionSuppression)(OwnerToken) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1)==0);
    static_assert(sizeof(CollisionSuppression)==184);
    static_assert(offsetof(Api,setCollisionSuppression)==sizeof(ApiV1));
    static_assert(sizeof(Api)==sizeof(ApiV1)+2*sizeof(void*));
}
#pragma pack(pop)

#pragma once
#include "WeaponPartsV1_4.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts::v1_5 {
    inline constexpr std::uint32_t kMajor=1,kMinor=5;
    // Opts this owner's points into pinky-side acquisition in addition to
    // 1.4's driven pose and palm/thumb/index probes. The little finger's base
    // at the palm and tracked tip are additional probes, not larger radii.
    // Missing pinky tracking preserves the other probes. Selection uses the
    // nearest allowed authored point across all available probes.
    // Same records, validation, Write permission, owner thread, replacement
    // and cleanup contract as 1.3/1.4. Their entry points retain their behavior.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_4::Api v1_4;
        Status(ROCK_CALL* setPinkyOperatingGrabPoints)(OwnerToken,const v1_3::GrabPoint*,std::uint32_t) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1_4)==0);
    static_assert(offsetof(Api,setPinkyOperatingGrabPoints)==sizeof(v1_4::Api));
    static_assert(sizeof(Api)==sizeof(v1_4::Api)+sizeof(void*));
}
#pragma pack(pop)

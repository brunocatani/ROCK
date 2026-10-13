#pragma once
#include "WeaponPartsV1_3.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts::v1_4 {
    inline constexpr std::uint32_t kMajor=1,kMinor=4;
    // Explicit opt-in for operating mechanisms. Same source-local records,
    // validation, owner-thread Write and lifecycle as 1.3 setGrabPoints.
    // Acquisition follows the current unexpired part-drive pose and chooses
    // the closest allowed point to the palm, thumb pad or index pad. Missing
    // finger tracking retains palm acquisition. Authored radii do not grow.
    // The indicator uses the final animated part pose. No scene writes or
    // reordered drive callbacks occur during acquisition. Older entry points
    // retain their original probe and animation-stage semantics.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_3::Api v1_3;
        Status(ROCK_CALL* setOperatingGrabPoints)(OwnerToken,const v1_3::GrabPoint*,std::uint32_t) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1_3)==0);
    static_assert(offsetof(Api,setOperatingGrabPoints)==sizeof(v1_3::Api));
    static_assert(sizeof(Api)==sizeof(v1_3::Api)+sizeof(void*));
}
#pragma pack(pop)

#pragma once
#include "WeaponPartsV1_5.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts::v1_6 {
    inline constexpr std::uint32_t kMajor=1,kMinor=6;
    // Opt-in point-anchored operating grabs. Retains 1.5 probe/radius rules.
    // Chooses the nearest registered collider surface to the selected point,
    // within its authored radius, using the current driven pose. No matching
    // surface means no acquisition. Capture seats the palm at that point and
    // preserves the approaching hand rotation; it does not project the hand
    // onto another part surface. The captured relation follows the moving part.
    // Same records, Write permission, owner thread, atomic replacement and
    // cleanup as 1.3. Every earlier entry point keeps its existing behavior.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_5::Api v1_5;
        Status(ROCK_CALL* setAnchoredOperatingGrabPoints)(OwnerToken,const v1_3::GrabPoint*,std::uint32_t) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1_5)==0);
    static_assert(offsetof(Api,setAnchoredOperatingGrabPoints)==sizeof(v1_5::Api));
    static_assert(sizeof(Api)==sizeof(v1_5::Api)+sizeof(void*));
}
#pragma pack(pop)

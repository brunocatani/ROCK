#pragma once
#include "WeaponPartsV1_2.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts::v1_3 {
    inline constexpr std::uint32_t kMajor=1,kMinor=3,kMaxGrabPoints=32;
    struct GrabPoint {
        std::uint32_t size{sizeof(GrabPoint)},pointId{};
        std::uint64_t weaponGenerationKey{},sourceKey{};
        std::uint32_t groupId{},handMask{3}; // left=1, right=2
        float pointInSource[3]{},radiusGame{3};
        std::uint32_t reserved[4]{};
    };
    // Opt in with WeaponParts 1.3 / sizeof(Api), Write, on the frame owner
    // thread. Register existing AttachOnly targets first, then their locations.
    // Source-local points follow the named current animated source, independently
    // of wrist pose and collider bounds. Radius is a world-game-unit distance.
    // Replaces this owner's entire point set atomically (1..32). Every group
    // must already belong to this owner on that weapon generation. IDs are
    // nonzero and unique within a group; handMask is 1, 2 or 3, radius (0,64].
    // Acquisition uses the closest eligible point within both its radius and
    // the normal hand-probe radius, before
    // ordinary surface candidates. Configured groups cannot acquire through
    // whole-part surfaces, including when their point frame is unavailable.
    // Point changes affect new acquisitions; an existing grip keeps its baseline.
    // Target replacement/clear, owner revoke and lifecycle reset remove points.
    // Explicit clear restores the owner's original target acquisition behavior.
    // All older table prefixes, records, calls and non-opted-in behavior remain.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_2::Api v1_2;
        Status(ROCK_CALL* setGrabPoints)(OwnerToken,const GrabPoint*,std::uint32_t) noexcept;
        Status(ROCK_CALL* clearGrabPoints)(OwnerToken) noexcept;
    };
    static_assert(std::is_standard_layout_v<GrabPoint> && sizeof(GrabPoint)==64);
    static_assert(offsetof(GrabPoint,pointInSource)==32);
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1_2)==0);
    static_assert(offsetof(Api,setGrabPoints)==sizeof(v1_2::Api));
    static_assert(sizeof(Api)==sizeof(v1_2::Api)+2*sizeof(void*));
}
#pragma pack(pop)

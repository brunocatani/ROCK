#pragma once
#include "WeaponPartsV1_6.h"

#pragma pack(push, 8)
namespace rock::api::weaponparts::v1_7 {
    inline constexpr std::uint32_t kMajor=1,kMinor=7,kMaxItemParts=96;
    struct ItemKey {
        std::uint32_t weapon{},owner{},unique{};
        bool operator==(const ItemKey&) const = default;
    };
    // Named path relative to the model's Weapon node. Local means the actual
    // parent-local node transform, in the existing ROCK Transform convention.
    // Hidden applies to this subtree and its generated collision geometry.
    struct ItemPart {
        char path[256]{};
        Transform local{};
        std::uint32_t hidden{};
    };
    struct ItemState {
        std::uint32_t size{sizeof(ItemState)},count{};
        ItemPart parts[kMaxItemParts]{};
    };
    // Exact native item identities: inventory owner/ExtraUniqueID, or dropped
    // reference/0. Descriptions contain values only; the provider copies them.
    // These additions do not change the 1.0--1.6 table prefix or behavior.
    // Write permission is required. Calls may originate from native identity
    // and serialization callbacks; no scene pointers may be supplied.
    // A description remains until moved, explicitly removed (after.owner=0),
    // cleared by its owner, or owner loss. Clients serialize their descriptions
    // and republish after load; provider tables never become save identities.
    struct Api {
        static constexpr InterfaceId interfaceId=kInterfaceId;
        static constexpr std::uint32_t majorVersion=kMajor,minorVersion=kMinor;
        v1_6::Api v1_6;
        Status(ROCK_CALL* setItemState)(OwnerToken,const ItemKey*,const ItemState*) noexcept;
        Status(ROCK_CALL* moveItemState)(OwnerToken,const ItemKey*,const ItemKey*) noexcept;
        Status(ROCK_CALL* clearItemStates)(OwnerToken) noexcept;
    };
    static_assert(std::is_standard_layout_v<Api> && offsetof(Api,v1_6)==0);
    static_assert(offsetof(Api,setItemState)==sizeof(v1_6::Api));
    static_assert(sizeof(Api)==sizeof(v1_6::Api)+3*sizeof(void*));
    static_assert(sizeof(ItemKey)==12 && sizeof(ItemPart)==312);
}
#pragma pack(pop)

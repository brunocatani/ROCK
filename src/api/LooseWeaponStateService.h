#pragma once
#include <ROCK/WeaponParts.h>
#include <span>
namespace RE { class TESObjectREFR; class NiAVObject; }
namespace rock::loose_weapon_state {
    namespace contract=api::weaponparts;
    bool install() noexcept;
    api::Status set(api::OwnerToken,const contract::ItemKey&,const contract::ItemState&);
    api::Status move(api::OwnerToken,const contract::ItemKey&,const contract::ItemKey&);
    api::Status clear(api::OwnerToken);
    std::size_t references(std::span<std::uint32_t> output);
    bool owns(std::uint32_t reference);
    bool apply(RE::TESObjectREFR*,RE::NiAVObject*);
    void restoreWithdrawn(); // interaction thread, before collision restoration
}

#pragma once
#include <ROCK/WeaponParts.h>
#include <span>

namespace RE
{
    class TESObjectREFR;
    class NiAVObject;
}

namespace rock::loose_weapon_state
{
    namespace contract = api::weaponparts;
    bool install() noexcept;
    api::Status set(api::OwnerToken, const contract::ItemKey&, const contract::ItemState&);
    api::Status move(api::OwnerToken, const contract::ItemKey&, const contract::ItemKey&);
    api::Status clear(api::OwnerToken);
    std::size_t references(std::span<std::uint32_t> output);
    bool owns(std::uint32_t reference);
    bool apply(RE::TESObjectREFR*, RE::NiAVObject*);
    /*
     * A weapon ROCK drops keeps the part poses consumers were driving in the
     * hand (an open bolt, a locked slide). Frame thread. Visual only: such a
     * state never claims the reference's collision (owns and references skip
     * it), is bound to that exact reference, and gives way to a consumer's
     * setItemState for the same item. It lasts until the reference is gone.
     */
    void carry(RE::TESObjectREFR* reference, const contract::ItemState& state);
    void restoreWithdrawn(); // interaction thread, before collision restoration
}

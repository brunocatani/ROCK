#pragma once
#include "physics-interaction/collision/NativePlayerCollisionPolicy.h"
#include <algorithm>
#include <array>
namespace rock::native_player_collision {
// Value identities only. The game-thread publisher sorts both bounded sets;
// simulation workers verify current identities before excluding an exact pair.
struct MagazineEjectionSnapshot {
    std::uintptr_t world{};
    std::array<BodyIdentity,32> magazines{};
    std::array<BodyIdentity,128> weapons{};
    std::size_t magazineCount{},weaponCount{};
    bool operator==(const MagazineEjectionSnapshot&) const = default;
    template<std::size_t N> static const BodyIdentity* find(const std::array<BodyIdentity,N>& bodies,std::size_t count,std::uint32_t id) {
        const auto end=bodies.begin()+count;
        const auto it=std::lower_bound(bodies.begin(),end,id,[](const auto& a,auto b){return a.bodyId<b;});
        return it!=end&&it->bodyId==id?&*it:nullptr;
    }
    bool containsPair(std::uint32_t a,std::uint32_t b) const {
        return a!=b&&((find(magazines,magazineCount,a)&&find(weapons,weaponCount,b))||
            (find(magazines,magazineCount,b)&&find(weapons,weaponCount,a)));
    }
    bool suppresses(std::uintptr_t liveWorld,const BodyIdentity& a,const BodyIdentity& b) const {
        if(!world||world!=liveWorld||a.bodyId==b.bodyId)return false;
        const auto* mag=find(magazines,magazineCount,a.bodyId);
        const auto* weapon=find(weapons,weaponCount,b.bodyId);
        if(mag&&weapon)return matchesLiveBody(*mag,a)&&matchesLiveBody(*weapon,b);
        mag=find(magazines,magazineCount,b.bodyId);weapon=find(weapons,weaponCount,a.bodyId);
        return mag&&weapon&&matchesLiveBody(*mag,b)&&matchesLiveBody(*weapon,a);
    }
};
}

#pragma once
#include <ROCK/WeaponParts.h>
#include <span>
#include <algorithm>

namespace rock::weapon_part_collision {
    template<class Node>
    bool selectedSource(const Node* node,std::span<const Node* const> roots,const Node* weaponRoot) noexcept {
        for(unsigned depth=0;node&&depth<=64;++depth,node=node->parent) {
            if(std::find(roots.begin(),roots.end(),node)!=roots.end())return true;
            if(node==weaponRoot)break;
        }
        return false;
    }
    // Value state owned exclusively by the provider frame thread.
    struct Lease {
        api::OwnerToken owner{};
        std::uint64_t expires{};
        api::weaponparts::CollisionSuppression request{};
        static bool sameGeneration(const api::Sample& a,const api::Sample& b) noexcept {
            return a.worldGeneration==b.worldGeneration&&a.skeletonGeneration==b.skeletonGeneration&&
                a.providerGeneration==b.providerGeneration&&a.collisionGeneration==b.collisionGeneration;
        }
        bool current(const api::Sample& sample) const noexcept {
            return owner&&sample.frameIndex<=expires&&sameGeneration(sample,request.sample);
        }
        api::Status set(api::OwnerToken caller,const api::weaponparts::CollisionSuppression& next,const api::Sample& sample) noexcept {
            if(!caller||!next.weaponGenerationKey||!next.sourceCount||next.sourceCount>api::weaponparts::kMaxCollisionSources||
                next.leaseFrames<1||next.leaseFrames>2)return api::Status::InvalidArgument;
            for(std::uint32_t i=0;i<next.sourceCount;++i)if(!next.sourceKeys[i])return api::Status::InvalidArgument;
            if(!sameGeneration(next.sample,sample)||next.sample.frameIndex>sample.frameIndex||sample.frameIndex-next.sample.frameIndex>1)return api::Status::GenerationMismatch;
            if(current(sample)&&owner!=caller)return api::Status::OwnerConflict;
            owner=caller;request=next;expires=sample.frameIndex+next.leaseFrames;return api::Status::Ok;
        }
        api::Status clear(api::OwnerToken caller,const api::Sample& sample) noexcept {
            if(current(sample)&&owner!=caller)return api::Status::OwnerConflict;
            *this={};return api::Status::Ok;
        }
    };
}

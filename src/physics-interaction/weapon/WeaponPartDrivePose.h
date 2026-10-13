#pragma once
#include "RE/NetImmerse/NiNode.h"
#include "physics-interaction/weapon/WeaponHierarchy.h"
#include <array>

namespace rock::weapon_drive_pose {
    // Frame-local read-only replay of the same ordered writes as the driver.
    // Native locals remain untouched. Borrowed nodes never survive this frame.
    template<class Node,std::size_t Capacity=64>
    class Frame {
    public:
        Frame(const Node* root,const RE::NiTransform& world):root_(root),rootWorld_(world){}
        bool world(const Node* source,RE::NiTransform& out) const {
            std::array<const Node*,64> path{};std::size_t n=0;auto* node=source;
            while(node&&node!=root_&&n<path.size()){path[n++]=node;node=node->parent;}
            if(!root_||node!=root_)return false;
            out=rootWorld_;
            while(n){const auto* next=path[--n];const auto* entry=find(next);
                out=transform_math::composeTransforms(out,entry?entry->local:next->local);
            }
            return valid(out);
        }
        bool apply(const Node* node,const RE::NiTransform& target,bool rootLocal,std::uint32_t priority) {
            if(!node||!node->parent||!valid(target))return false;
            auto* previous=find(node);if(previous&&priority<previous->priority)return false;
            RE::NiTransform parentWorld{};
            if(node==root_)parentWorld=node->parent->world;
            else if(!world(node->parent,parentWorld))return false;
            if(!valid(parentWorld))return false;
            const auto desired=transform_math::composeTransforms(rootLocal?rootWorld_:parentWorld,target);
            const auto local=transform_math::composeTransforms(transform_math::invertTransform(parentWorld),desired);
            if(!valid(desired)||!valid(local)||(!previous&&count_==entries_.size()))return false;
            if(!previous)previous=&entries_[count_++];
            *previous={node,local,priority};if(node==root_)rootWorld_=desired;return true;
        }
        std::size_t size() const {return count_;}
        bool contains(const Node* node) const {return find(node)!=nullptr;}
    private:
        static bool valid(const RE::NiTransform& t){return weapon_hierarchy::weaponTransformFinite(t)&&std::abs(t.scale)>1e-6f;}
        struct Entry {const Node* node{};RE::NiTransform local{};std::uint32_t priority{};};
        Entry* find(const Node* node){for(std::size_t i=0;i<count_;++i)if(entries_[i].node==node)return &entries_[i];return nullptr;}
        const Entry* find(const Node* node) const {for(std::size_t i=0;i<count_;++i)if(entries_[i].node==node)return &entries_[i];return nullptr;}
        const Node* root_{};RE::NiTransform rootWorld_{};
        std::array<Entry,Capacity> entries_{};std::size_t count_{};
    };
    using NativeFrame=Frame<RE::NiAVObject>;
}

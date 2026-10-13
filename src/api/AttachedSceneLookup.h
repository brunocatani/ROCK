#pragma once
#include <array>
#include <cstddef>

namespace rock::attached_scene_lookup
{
    // Query only attached scene children. BSFlattenedBoneTree's native name
    // lookup (VR 1C21260 -> 1C20D30) can create an empty NiNode from a bone
    // table entry; that proxy is not the consumer's visible display mesh.
    template<class Node,class Predicate,class Children>
    Node* find(Node* root,Predicate matches,Children children)
    {
        if(!root) return nullptr;
        constexpr std::size_t budget=4096;
        std::array<Node*,budget> pending;
        std::size_t count=1,visited{},slots{}; pending[0]=root;
        while(count && visited++<budget) {
            auto* node=pending[--count];
            if(matches(node)) return node;
            if(const auto* list=children(node)) for(const auto& child:*list) {
                if(++slots>budget) return nullptr;
                if(child.get()) {
                    if(count==budget) return nullptr;
                    pending[count++]=child.get();
                }
            }
        }
        return nullptr;
    }
    template<class Node> bool beneath(const Node* root,const Node* node)
    {
        if(!root || root==node) return false;
        for(unsigned depth=0;node && depth<64;++depth,node=node->parent) if(node==root) return true;
        return false;
    }
}

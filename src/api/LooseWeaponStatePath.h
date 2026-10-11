#pragma once
#include <string_view>
#include <array>
namespace rock::loose_weapon_state {
    // Match complete chains. Repeated attachment names are only ambiguous
    // when more than one complete path reaches a target.
    template<class Node,class Children,class Name>
    Node* completeItemPath(Node* root,std::string_view path,Children children,Name name) {
        struct Step {Node* node;std::string_view remaining;};
        std::array<Step,2048> work{};std::size_t count=1;work[0]={root,path};Node* found{};
        for(std::size_t i=0;i<count;++i) {
            const auto step=work[i];
            if(step.remaining.empty()){if(found&&found!=step.node)return nullptr;found=step.node;continue;}
            const auto slash=step.remaining.find('/');
            const auto component=step.remaining.substr(0,slash);
            bool overflow=false;
            children(step.node,[&](Node* child){
                if(name(child)!=component)return;
                if(count==work.size()){overflow=true;return;}
                work[count++]={child,slash==step.remaining.npos?std::string_view{}:step.remaining.substr(slash+1)};
            });
            if(overflow)return nullptr;
        }
        return found;
    }
    // A saved equipped path may include the instance wrapper that is itself
    // the root of the loose tree. Match a whole component, never a prefix.
    constexpr std::string_view relativeItemPath(std::string_view root,std::string_view path,std::string_view instance={}) noexcept {
        // Native loose assemblies use asset-defined root names, replacing
        // the actor's exact Weapon (base form) wrapper. P-Receiver is the
        // receiver attachment below that parent, not the parent itself.
        if(!root.empty()&&root!="Weapon"&&!instance.empty()) {
            const auto first=path.find('/');
            if(path.substr(0,first)==instance) {
                path=first==path.npos?std::string_view{}:path.substr(first+1);
                return path;
            }
        }
        const auto slash=path.find('/');
        if(!root.empty()&&path.substr(0,slash)==root)
            return slash==path.npos?std::string_view{}:path.substr(slash+1);
        return path;
    }
}

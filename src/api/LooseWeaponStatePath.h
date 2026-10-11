#pragma once
#include <string_view>
namespace rock::loose_weapon_state {
    // A saved equipped path may include the instance wrapper that is itself
    // the root of the loose tree. Match a whole component, never a prefix.
    constexpr std::string_view relativeItemPath(std::string_view root,std::string_view path,std::string_view instance={}) noexcept {
        // Native loose assemblies start at ReceiverParentObject, replacing
        // the actor's exact Weapon (base form) wrapper. P-Receiver is the
        // receiver attachment below that parent, not the parent itself.
        if(root=="ReceiverParentObject"&&!instance.empty()) {
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

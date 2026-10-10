#pragma once
#include <string_view>
namespace rock::loose_weapon_state {
    // A saved equipped path may include the instance wrapper that is itself
    // the root of the loose tree. Match a whole component, never a prefix.
    constexpr std::string_view relativeItemPath(std::string_view root,std::string_view path) noexcept {
        const auto slash=path.find('/');
        if(!root.empty()&&path.substr(0,slash)==root)
            return slash==path.npos?std::string_view{}:path.substr(slash+1);
        return path;
    }
}

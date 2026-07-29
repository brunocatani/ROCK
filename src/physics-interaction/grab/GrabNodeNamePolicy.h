#pragma once

#include <algorithm>
#include <cctype>
#include <string>
#include <string_view>

namespace rock::grab_node_name_policy
{
    /*
     * NIF marker names are part of ROCK's FO4VR asset contract. Keeping that
     * contract in one policy prevents stale non-ROCK namespaced INI values from
     * silently becoming runtime dependencies.
     */

    inline constexpr std::string_view kDefaultRightGrabNodeName = "ROCK:GrabR";
    inline constexpr std::string_view kDefaultLeftGrabNodeName = "ROCK:GrabL";
    inline constexpr std::string_view kDefaultGrabNodeNameBlacklist = "ROCK:GrabR,ROCK:GrabL";

    inline std::string_view defaultGrabNodeName(bool isLeft)
    {
        return isLeft ? kDefaultLeftGrabNodeName : kDefaultRightGrabNodeName;
    }

    inline bool startsWith(std::string_view value, std::string_view prefix)
    {
        return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
    }

    inline bool isForeignNamespacedGrabNodeName(std::string_view value)
    {
        const auto namespaceSeparator = value.find(':');
        return namespaceSeparator != std::string_view::npos && !startsWith(value, "ROCK:");
    }

    inline std::string sanitizeConfiguredGrabNodeName(std::string_view configuredName, bool isLeft)
    {
        if (configuredName.empty() || isForeignNamespacedGrabNodeName(configuredName)) {
            return std::string(defaultGrabNodeName(isLeft));
        }

        return std::string(configuredName);
    }

    inline std::string_view trimToken(std::string_view value)
    {
        while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front())) != 0) {
            value.remove_prefix(1);
        }
        while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())) != 0) {
            value.remove_suffix(1);
        }
        return value;
    }

    inline bool tokenEquals(std::string_view lhs, std::string_view rhs)
    {
        lhs = trimToken(lhs);
        rhs = trimToken(rhs);
        return lhs == rhs;
    }

    inline bool isListSeparator(char value)
    {
        return value == ',' || value == ';' || value == '|';
    }

    inline bool listContainsName(std::string_view configuredList, std::string_view nodeName)
    {
        /*
         * ROCK keeps authored anchors as explicit nodes, then filters those same
         * names from fallback mesh/surface sampling to avoid double-counting
         * marker geometry if an asset author uses visible helper meshes.
         */
        nodeName = trimToken(nodeName);
        if (nodeName.empty()) {
            return false;
        }

        std::size_t start = 0;
        while (start <= configuredList.size()) {
            std::size_t end = start;
            while (end < configuredList.size() && !isListSeparator(configuredList[end])) {
                ++end;
            }

            if (tokenEquals(configuredList.substr(start, end - start), nodeName)) {
                return true;
            }

            if (end >= configuredList.size()) {
                break;
            }
            start = end + 1;
        }
        return false;
    }

    inline bool shouldSkipNodeForMeshSurface(std::string_view configuredBlacklist, std::string_view nodeName)
    {
        return listContainsName(configuredBlacklist, nodeName);
    }

    inline bool isOppositeHandGrabNodeName(std::string_view nodeName, bool isLeft)
    {
        const auto oppositeDefault = defaultGrabNodeName(!isLeft);
        return tokenEquals(nodeName, oppositeDefault);
    }
}

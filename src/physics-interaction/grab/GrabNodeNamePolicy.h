#pragma once

#include <string>
#include <string_view>

namespace frik::rock::grab_node_name_policy
{
    /*
     * ROCK uses HIGGS as the behavior reference, but NIF marker names are part of
     * ROCK's FO4VR asset contract. Keeping that contract in one policy prevents
     * stale Skyrim/HIGGS INI values from silently becoming runtime dependencies.
     */

    inline constexpr std::string_view kDefaultRightGrabNodeName = "ROCK:GrabR";
    inline constexpr std::string_view kDefaultLeftGrabNodeName = "ROCK:GrabL";

    inline std::string_view defaultGrabNodeName(bool isLeft)
    {
        return isLeft ? kDefaultLeftGrabNodeName : kDefaultRightGrabNodeName;
    }

    inline bool startsWith(std::string_view value, std::string_view prefix)
    {
        return value.size() >= prefix.size() && value.substr(0, prefix.size()) == prefix;
    }

    inline bool isLegacyHiggsGrabNodeName(std::string_view value)
    {
        return startsWith(value, "HIGGS:");
    }

    inline std::string sanitizeConfiguredGrabNodeName(std::string_view configuredName, bool isLeft)
    {
        if (configuredName.empty() || isLegacyHiggsGrabNodeName(configuredName)) {
            return std::string(defaultGrabNodeName(isLeft));
        }

        return std::string(configuredName);
    }
}

#pragma once

#include <string_view>

namespace rock::weapon_material_visibility
{
    constexpr char pathCharacter(char value)
    {
        if (value == '\\') return '/';
        return value >= 'A' && value <= 'Z' ? static_cast<char>(value + ('a' - 'A')) : value;
    }

    constexpr bool pathEquals(std::string_view left, std::string_view right)
    {
        if (left.size() != right.size()) return false;
        for (std::size_t i = 0; i < left.size(); ++i)
            if (pathCharacter(left[i]) != pathCharacter(right[i])) return false;
        return true;
    }

    // Combined Arms' absent rails/stickers use this fully transparent texture
    // with material alpha=1 and alpha testing. Do not infer absence from a part
    // name, a generic "None" material name, or partially transparent textures.
    constexpr bool isInvisibleTexture(std::string_view path)
    {
        constexpr std::string_view prefix = "textures/";
        if (path.size() >= prefix.size() && pathEquals(path.substr(0, prefix.size()), prefix))
            path.remove_prefix(prefix.size());
        return pathEquals(path, "akmnv/invisible.dds");
    }

    struct CullDecision
    {
        bool culled;
        bool owned;
    };

    constexpr CullDecision decideCull(bool hiddenMaterial, bool currentlyCulled, bool owned)
    {
        if (hiddenMaterial) return { true, owned || !currentlyCulled };
        return { currentlyCulled && !owned, false };
    }
}

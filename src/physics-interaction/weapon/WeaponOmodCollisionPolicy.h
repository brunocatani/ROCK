#pragma once

#include <cstdint>
#include <cstddef>
#include <algorithm>
#include <span>
#include <string>
#include <string_view>

namespace rock::weapon_omod_collision_policy
{
    inline constexpr std::size_t kMaximumSceneNodes = 4096;
    inline constexpr std::size_t kMaximumTemplateNodes = 512;
    inline constexpr std::size_t kMaximumSources = 256;

    [[nodiscard]] inline bool cloneNamesPreserved(std::span<const std::string_view> original,
        std::span<const std::string_view> copied)
    {
        // Counts alone missed a clone with all names erased. Preserve names
        // and their multiplicity without depending on child iteration order.
        return original.size() == copied.size() &&
            std::is_permutation(original.begin(), original.end(), copied.begin());
    }

    // ConnectChild substitutes the authored |0 suffix; a mesh's :0 is
    // part of its geometry identity and must not be stripped like a bone name.
    [[nodiscard]] inline std::string instanceName(std::string_view name, std::uint32_t index)
    {
        const auto separator = name.find('|');
        if (index != 0 && separator != std::string_view::npos &&
            separator + 1 < name.size() && name[separator + 1] == '0') {
            return std::string(name.substr(0, separator + 1)) + std::to_string(index);
        }
        return std::string(name);
    }

    [[nodiscard]] inline std::string parentPointName(std::string_view childPoint)
    {
        if (childPoint.size() < 3 || childPoint[0] != 'C' || childPoint[1] != '-') {
            return {};
        }
        const auto delimiter = childPoint.find_first_of(",|");
        if (delimiter == 2) return {};
        return "P-" + std::string(childPoint.substr(2, delimiter == std::string_view::npos ? delimiter : delimiter - 2));
    }

    enum class Coverage : std::uint8_t
    {
        NativeGeometry,
        OwnedGeometry,
        Recoverable,
        IncompleteScan,
        AmbiguousParent,
        InactiveBranch,
        UnsupportedSkin,
    };

    [[nodiscard]] inline constexpr Coverage decide(bool scanComplete, std::size_t parentMatches,
        bool parentActive, bool skinned, bool nativeGeometry, bool ownedGeometry) noexcept
    {
        if (!scanComplete) return Coverage::IncompleteScan;
        if (parentMatches != 1) return Coverage::AmbiguousParent;
        if (!parentActive) return Coverage::InactiveBranch;
        if (nativeGeometry) return Coverage::NativeGeometry;
        if (ownedGeometry) return Coverage::OwnedGeometry;
        if (skinned) return Coverage::UnsupportedSkin;
        return Coverage::Recoverable;
    }
}

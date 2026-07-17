#pragma once

#include <array>
#include <cstddef>
#include <string_view>

namespace rock::weapon_effect_geometry_policy
{
    enum class ExclusionReason
    {
        None,
        EffectShaderProperty,
        BillboardAncestor,
        KnownEffectName,
    };

    struct Evidence
    {
        bool hasEffectShaderProperty{ false };
        bool hasBillboardAncestor{ false };
        std::string_view geometryName{};
    };

    [[nodiscard]] inline constexpr char foldAscii(char value) noexcept
    {
        return value >= 'A' && value <= 'Z' ? static_cast<char>(value + ('a' - 'A')) : value;
    }

    [[nodiscard]] inline constexpr bool equalsAsciiInsensitive(std::string_view lhs, std::string_view rhs) noexcept
    {
        if (lhs.size() != rhs.size()) {
            return false;
        }
        for (std::size_t i = 0; i < lhs.size(); ++i) {
            if (foldAscii(lhs[i]) != foldAscii(rhs[i])) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] inline constexpr bool containsAsciiInsensitive(std::string_view value, std::string_view token) noexcept
    {
        if (token.empty() || token.size() > value.size()) {
            return false;
        }
        for (std::size_t offset = 0; offset + token.size() <= value.size(); ++offset) {
            if (equalsAsciiInsensitive(value.substr(offset, token.size()), token)) {
                return true;
            }
        }
        return false;
    }

    /*
     * Fallback only for malformed/custom NIFs whose visual-effect geometry is
     * not backed by BSEffectShaderProperty and is not under NiBillboardNode.
     * Keep these tokens role-specific: broad words such as "laser", "light",
     * "scope", and "sight" also describe the physical module that must retain
     * collision.
     */
    [[nodiscard]] inline constexpr bool hasKnownEffectGeometryName(std::string_view geometryName) noexcept
    {
        constexpr std::array<std::string_view, 10> tokens{
            "lasersightbeam",
            "lasersightdot",
            "laserbeam",
            "laserdot",
            "laserray",
            "reticle",
            "parallax",
            "screengloweffect",
            "gloweffect",
            "lightfx",
        };
        for (const auto token : tokens) {
            if (containsAsciiInsensitive(geometryName, token)) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] inline constexpr ExclusionReason classify(const Evidence& evidence) noexcept
    {
        if (evidence.hasEffectShaderProperty) {
            return ExclusionReason::EffectShaderProperty;
        }
        if (evidence.hasBillboardAncestor) {
            return ExclusionReason::BillboardAncestor;
        }
        if (hasKnownEffectGeometryName(evidence.geometryName)) {
            return ExclusionReason::KnownEffectName;
        }
        return ExclusionReason::None;
    }

    [[nodiscard]] inline constexpr const char* exclusionReasonName(ExclusionReason reason) noexcept
    {
        switch (reason) {
        case ExclusionReason::EffectShaderProperty:
            return "effect-shader-property";
        case ExclusionReason::BillboardAncestor:
            return "billboard-ancestor";
        case ExclusionReason::KnownEffectName:
            return "known-effect-name";
        case ExclusionReason::None:
        default:
            return "none";
        }
    }
}

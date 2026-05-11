#pragma once

#include <cstdint>
#include <string_view>

namespace rock::see_through_scopes_policy
{
    /*
     * ROCK owns only the VR compatibility behavior that makes See-Through
     * Scopes weapon attachments usable with ROCK's weapon authority. The old
     * external scope plugin also changed FOV, weapon offsets, and cross-mod
     * scope-state messaging; those paths are deliberately excluded so aiming
     * through a scope cannot look like menu ownership or break two-hand support
     * grip state.
     */

    constexpr auto kCompatibilityConfigKey = "bSeeThroughScopesCompatibilityEnabled";
    constexpr auto kReticleAlignmentConfigKey = "bSeeThroughScopesReticleAlignmentEnabled";

    constexpr std::uint32_t kNativeScopeOverlayTarget = 48;
    constexpr std::uint8_t kBgsModPropertyBlockId = 1;

    constexpr float kDefaultReticleEyeOffsetGameUnits = 2.3f;
    constexpr float kDefaultReticleOffsetXGameUnits = 0.372727f;
    constexpr float kDefaultReticleOffsetZGameUnits = -0.149692f;
    constexpr float kDefaultReticleLookDotThreshold = 0.98f;
    constexpr float kDefaultReticleDistanceThresholdGameUnits = 20.0f;

    [[nodiscard]] constexpr char asciiLower(char c) noexcept
    {
        return c >= 'A' && c <= 'Z' ? static_cast<char>(c + ('a' - 'A')) : c;
    }

    [[nodiscard]] constexpr bool asciiIEquals(std::string_view left, std::string_view right) noexcept
    {
        if (left.size() != right.size()) {
            return false;
        }

        for (std::size_t i = 0; i < left.size(); ++i) {
            if (asciiLower(left[i]) != asciiLower(right[i])) {
                return false;
            }
        }

        return true;
    }

    [[nodiscard]] constexpr bool asciiIStartsWith(std::string_view text, std::string_view prefix) noexcept
    {
        if (text.size() < prefix.size()) {
            return false;
        }

        return asciiIEquals(text.substr(0, prefix.size()), prefix);
    }

    [[nodiscard]] constexpr bool hasBethesdaPluginExtension(std::string_view filename) noexcept
    {
        if (filename.size() < 4) {
            return false;
        }

        const auto extension = filename.substr(filename.size() - 4);
        return asciiIEquals(extension, ".esp") || asciiIEquals(extension, ".esm") || asciiIEquals(extension, ".esl");
    }

    [[nodiscard]] constexpr bool isSeeThroughScopesPluginFilename(std::string_view filename) noexcept
    {
        return asciiIStartsWith(filename, "3dscopes") && hasBethesdaPluginExtension(filename);
    }
}

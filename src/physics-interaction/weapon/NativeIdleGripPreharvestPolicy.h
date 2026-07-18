#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace rock::native_idle_grip_preharvest_policy
{
    inline constexpr std::size_t kFirstPersonGraphIndex = 1;

    struct FirstPersonSelection
    {
        bool valid{ false };
        std::size_t graphIndex{ 0 };
    };

    /*
     * Bethesda builds the background actor manager in paired graph order:
     * third person first, first person second. RequestAnimationSubGraph visits
     * those same graphs in order and appends matching identifiers in lockstep.
     * Never fall back to graph zero: that would silently harvest a flat/third-
     * person relation instead of the FO4VR first-person grip.
     */
    [[nodiscard]] constexpr FirstPersonSelection selectFirstPersonGraph(const std::size_t graphCount, const std::size_t identifierCount) noexcept
    {
        if (graphCount <= kFirstPersonGraphIndex || identifierCount <= kFirstPersonGraphIndex) {
            return {};
        }
        return FirstPersonSelection{
            .valid = true,
            .graphIndex = kFirstPersonGraphIndex,
        };
    }

    /*
     * hkaAnimationBinding uses an empty transformTrackToBoneIndices array to
     * mean identity mapping. A non-empty mapping must cover every sampled
     * transform track; truncated or malformed bindings fail closed.
     */
    [[nodiscard]] constexpr int findTransformTrackForBone(const int boneIndex, const int transformTrackCount,
        const std::span<const std::int16_t> transformTrackToBoneIndices) noexcept
    {
        if (boneIndex < 0 || transformTrackCount <= 0) {
            return -1;
        }
        if (transformTrackToBoneIndices.empty()) {
            return boneIndex < transformTrackCount ? boneIndex : -1;
        }
        if (transformTrackToBoneIndices.size() < static_cast<std::size_t>(transformTrackCount)) {
            return -1;
        }
        for (int trackIndex = 0; trackIndex < transformTrackCount; ++trackIndex) {
            if (transformTrackToBoneIndices[static_cast<std::size_t>(trackIndex)] == boneIndex) {
                return trackIndex;
            }
        }
        return -1;
    }

    [[nodiscard]] constexpr bool weaponIsDirectChildOfHand(const int weaponBoneIndex, const int handBoneIndex, const std::span<const std::int16_t> parentIndices) noexcept
    {
        return weaponBoneIndex >= 0 && handBoneIndex >= 0 && static_cast<std::size_t>(weaponBoneIndex) < parentIndices.size() &&
            parentIndices[static_cast<std::size_t>(weaponBoneIndex)] == handBoneIndex;
    }

    [[nodiscard]] constexpr bool clipPathHasStem(const std::string_view path, const std::string_view expectedStem) noexcept
    {
        const auto separator = path.find_last_of("/\\");
        const auto stemBegin = separator == std::string_view::npos ? 0 : separator + 1;
        const auto extension = path.find_last_of('.');
        const auto stemEnd = extension == std::string_view::npos || extension < stemBegin ? path.size() : extension;
        if (stemEnd - stemBegin != expectedStem.size()) {
            return false;
        }

        const auto asciiLower = [](const char value) { return value >= 'A' && value <= 'Z' ? static_cast<char>(value + ('a' - 'A')) : value; };
        for (std::size_t index = 0; index < expectedStem.size(); ++index) {
            if (asciiLower(path[stemBegin + index]) != asciiLower(expectedStem[index])) {
                return false;
            }
        }
        return true;
    }
}

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE
{
    class NiAVObject;
    class NiNode;
}

namespace rock::equipped_weapon_visual_state
{
    constexpr std::size_t kMaximumObservedPathNodes = 10;

    struct Snapshot
    {
        RE::NiNode* weaponRoot{ nullptr };
        RE::NiAVObject* exactInstance{ nullptr };
        std::array<RE::NiAVObject*, kMaximumObservedPathNodes> pathNodes{};
        std::size_t pathNodeCount{ 0 };
        bool ancestorPathVisible{ false };
        bool instanceLocallyVisible{ false };
    };

    [[nodiscard]] Snapshot observe(
        std::uint32_t weaponBaseFormID,
        std::uintptr_t excludedInstanceAddress = 0) noexcept;
    [[nodiscard]] bool isLocallyVisible(const RE::NiAVObject* node) noexcept;
    void setLocallyVisible(RE::NiAVObject* node, bool visible) noexcept;
    [[nodiscard]] bool restoreExactInstancePathVisibility(const Snapshot& snapshot) noexcept;
}

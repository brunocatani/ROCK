// Equipped weapon visibility observation (see EquippedWeaponVisualState.cpp).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <source_location>
#include "physics-interaction/weapon/scene/WeaponScenePath.h"

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

    // Owned by the transition coordinator and cleared with its skeleton state.
    struct ObservationCache
    {
        weapon_scene::Path<RE::NiAVObject> weaponPath;
        weapon_scene::Path<RE::NiAVObject, kMaximumObservedPathNodes> instancePath;
        std::uint32_t formID = 0;
        std::uintptr_t excludedInstance = 0;
    };

    [[nodiscard]] Snapshot observe(std::uint32_t weaponBaseFormID, std::uintptr_t excludedInstanceAddress = 0,
        ObservationCache* cache = nullptr) noexcept;
    [[nodiscard]] bool isLocallyVisible(const RE::NiAVObject* node) noexcept;
    // Records the call site of every change for the weapon visibility trace.
    void setLocallyVisible(RE::NiAVObject* node, bool visible,
        std::source_location site = std::source_location::current()) noexcept;

    // The last setLocallyVisible call ROCK made on a node, from a small ring
    // of recent calls. Diagnostic only: fields of one entry are written
    // without a lock, so a concurrent write can mix two entries.
    struct VisibilityWrite
    {
        const char* file{ nullptr };
        std::uint32_t line{ 0 };
        bool visible{ false };
    };
    [[nodiscard]] bool lastVisibilityWrite(const RE::NiAVObject* node, VisibilityWrite& out) noexcept;
    [[nodiscard]] bool restoreExactInstancePathVisibility(const Snapshot& snapshot) noexcept;
}

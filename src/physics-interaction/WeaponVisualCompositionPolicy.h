#pragma once

#include <cstdint>
#include <string_view>

namespace frik::rock::weapon_visual_composition_policy
{
    /*
     * Weapon mod swaps can expose the weapon node before the edited part's
     * renderer data has settled. ROCK therefore separates runtime visual
     * composition from transforms: geometry/visibility identity participates in
     * rebuild decisions, while ordinary animation and weapon motion do not.
     */
    inline constexpr std::uint64_t kWeaponVisualCompositionOffset = 1469598103934665603ull;
    inline constexpr std::uint64_t kWeaponVisualCompositionPrime = 1099511628211ull;
    inline constexpr std::uint32_t kRequiredStableVisualFrames = 3;

    struct VisualRecord
    {
        std::uintptr_t nodeAddress{ 0 };
        std::uintptr_t parentAddress{ 0 };
        std::string_view name{};
        std::uint32_t depth{ 0 };
        std::uint32_t childIndex{ 0 };
        std::uint32_t childCount{ 0 };
        bool visible{ false };
        bool triShape{ false };
        std::uintptr_t rendererData{ 0 };
        std::uintptr_t skinInstance{ 0 };
        std::uintptr_t vertexBlock{ 0 };
        std::uintptr_t triangleBlock{ 0 };
        std::uint64_t vertexDesc{ 0 };
        std::uint32_t numTriangles{ 0 };
        std::uint32_t numVertices{ 0 };
        std::uint32_t geometryType{ 0 };
    };

    struct VisualSettleState
    {
        std::uint64_t observedKey{ 0 };
        std::uint32_t stableFrames{ 0 };
    };

    struct VisualSettleResult
    {
        std::uint64_t observedKey{ 0 };
        std::uint32_t stableFrames{ 0 };
        bool keyChanged{ false };
        bool settled{ false };
    };

    inline void mixValue(std::uint64_t& key, std::uint64_t value)
    {
        key ^= value;
        key *= kWeaponVisualCompositionPrime;
    }

    inline void mixString(std::uint64_t& key, std::string_view value)
    {
        for (const char ch : value) {
            mixValue(key, static_cast<unsigned char>(ch));
        }
        mixValue(key, value.size());
    }

    inline void mixVisualRecord(std::uint64_t& key, const VisualRecord& record)
    {
        mixValue(key, record.nodeAddress);
        mixValue(key, record.parentAddress);
        mixString(key, record.name);
        mixValue(key, record.depth);
        mixValue(key, record.childIndex);
        mixValue(key, record.childCount);
        mixValue(key, record.visible ? 1ULL : 0ULL);
        mixValue(key, record.triShape ? 1ULL : 0ULL);
        mixValue(key, record.rendererData);
        mixValue(key, record.skinInstance);
        mixValue(key, record.vertexBlock);
        mixValue(key, record.triangleBlock);
        mixValue(key, record.vertexDesc);
        mixValue(key, record.numTriangles);
        mixValue(key, record.numVertices);
        mixValue(key, record.geometryType);
    }

    inline VisualSettleResult advanceVisualSettle(
        VisualSettleState& state,
        std::uint64_t observedKey,
        std::uint32_t requiredStableFrames = kRequiredStableVisualFrames)
    {
        if (observedKey == 0) {
            const bool changed = state.observedKey != 0 || state.stableFrames != 0;
            state.observedKey = 0;
            state.stableFrames = 0;
            return VisualSettleResult{
                .observedKey = 0,
                .stableFrames = 0,
                .keyChanged = changed,
                .settled = false,
            };
        }

        bool changed = false;
        if (state.observedKey != observedKey) {
            state.observedKey = observedKey;
            state.stableFrames = 1;
            changed = true;
        } else if (state.stableFrames < requiredStableFrames) {
            ++state.stableFrames;
        }

        return VisualSettleResult{
            .observedKey = state.observedKey,
            .stableFrames = state.stableFrames,
            .keyChanged = changed,
            .settled = state.stableFrames >= requiredStableFrames,
        };
    }
}

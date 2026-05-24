#pragma once

#include <cstdint>

namespace rock::physics_creation_gate_policy
{
    inline constexpr std::uint32_t kSkeletonReadyCreateDeferralFrames = 1;
    inline constexpr std::uint32_t kStableWorldFramesRequired = 2;

    enum class CreationBlockReason : std::uint32_t
    {
        None = 0,
        RockDisabled,
        ProviderUnavailable,
        SkeletonNotReady,
        ReadyEventDeferred,
        MenuBlocked,
        WorldUnavailable,
        WorldUnstable,
    };

    struct WorldStabilityState
    {
        std::uintptr_t bhkWorld = 0;
        std::uintptr_t hknpWorld = 0;
        std::uint32_t stableFrames = 0;
    };

    struct CreationGateInput
    {
        bool rockEnabled = false;
        bool providerAvailable = false;
        bool skeletonReady = false;
        bool runtimeMenuBlocking = false;
        bool compatibilityConfigBlocking = false;
        std::uintptr_t bhkWorld = 0;
        std::uintptr_t hknpWorld = 0;
        std::uint32_t readyDeferralFrames = 0;
        std::uint32_t requiredStableWorldFrames = kStableWorldFramesRequired;
    };

    struct CreationGateDecision
    {
        bool canCreate = false;
        bool keepRequestPending = true;
        CreationBlockReason blockReason = CreationBlockReason::None;
        std::uint32_t stableWorldFrames = 0;
    };

    inline constexpr bool hasCompleteWorld(const CreationGateInput& input)
    {
        return input.bhkWorld != 0 && input.hknpWorld != 0;
    }

    inline constexpr bool hasMenuBlock(const CreationGateInput& input)
    {
        return input.runtimeMenuBlocking || input.compatibilityConfigBlocking;
    }

    inline void resetWorldStability(WorldStabilityState& state)
    {
        state = {};
    }

    inline void observeCandidateWorld(WorldStabilityState& state, std::uintptr_t bhkWorld, std::uintptr_t hknpWorld)
    {
        if (bhkWorld == 0 || hknpWorld == 0) {
            resetWorldStability(state);
            return;
        }

        if (state.bhkWorld == bhkWorld && state.hknpWorld == hknpWorld) {
            if (state.stableFrames != UINT32_MAX) {
                ++state.stableFrames;
            }
            return;
        }

        state.bhkWorld = bhkWorld;
        state.hknpWorld = hknpWorld;
        state.stableFrames = 1;
    }

    inline CreationGateDecision evaluateCreationGate(WorldStabilityState& state, const CreationGateInput& input)
    {
        if (!input.rockEnabled) {
            resetWorldStability(state);
            return {
                .keepRequestPending = false,
                .blockReason = CreationBlockReason::RockDisabled,
            };
        }

        if (!input.providerAvailable) {
            resetWorldStability(state);
            return {
                .keepRequestPending = true,
                .blockReason = CreationBlockReason::ProviderUnavailable,
            };
        }

        if (!input.skeletonReady) {
            resetWorldStability(state);
            return {
                .keepRequestPending = true,
                .blockReason = CreationBlockReason::SkeletonNotReady,
            };
        }

        if (input.readyDeferralFrames > 0) {
            resetWorldStability(state);
            return {
                .keepRequestPending = true,
                .blockReason = CreationBlockReason::ReadyEventDeferred,
            };
        }

        if (hasMenuBlock(input)) {
            resetWorldStability(state);
            return {
                .keepRequestPending = true,
                .blockReason = CreationBlockReason::MenuBlocked,
            };
        }

        if (!hasCompleteWorld(input)) {
            resetWorldStability(state);
            return {
                .keepRequestPending = true,
                .blockReason = CreationBlockReason::WorldUnavailable,
            };
        }

        observeCandidateWorld(state, input.bhkWorld, input.hknpWorld);
        const auto requiredStableFrames = input.requiredStableWorldFrames == 0 ? 1 : input.requiredStableWorldFrames;
        if (state.stableFrames < requiredStableFrames) {
            return {
                .keepRequestPending = true,
                .blockReason = CreationBlockReason::WorldUnstable,
                .stableWorldFrames = state.stableFrames,
            };
        }

        return {
            .canCreate = true,
            .keepRequestPending = false,
            .blockReason = CreationBlockReason::None,
            .stableWorldFrames = state.stableFrames,
        };
    }
}

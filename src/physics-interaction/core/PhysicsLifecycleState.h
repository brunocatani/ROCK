#pragma once

#include "api/ROCKProviderApi.h"

#include <cstdint>

namespace rock::physics_lifecycle
{
    inline constexpr std::uint32_t kDefaultSettleFramesRequired = 2;

    struct RuntimeState
    {
        std::uintptr_t bhkWorld{ 0 };
        std::uintptr_t hknpWorld{ 0 };
        std::uint32_t worldGeneration{ 1 };
        std::uint32_t skeletonGeneration{ 1 };
        std::uint32_t providerGeneration{ 1 };
        std::uint32_t generatedBodiesWorldGeneration{ 0 };
        std::uint32_t generatedBodiesSkeletonGeneration{ 0 };
        std::uint32_t generatedBodiesProviderGeneration{ 0 };
        std::uint32_t stableFrameCount{ 0 };
        std::uint32_t settleFramesRequired{ kDefaultSettleFramesRequired };
        std::uint32_t flags{ 0 };
        provider::RockProviderLifecycleReason lastReason{ provider::RockProviderLifecycleReason::None };
        bool generatedBodiesValid{ false };
    };

    struct FrameInputs
    {
        std::uintptr_t bhkWorld{ 0 };
        std::uintptr_t hknpWorld{ 0 };
        std::uint32_t skeletonGeneration{ 1 };
        std::uint32_t providerGeneration{ 1 };
        bool providerReady{ false };
        bool skeletonReady{ false };
        bool menuBlocking{ false };
        bool configBlocking{ false };
        bool generatedBodiesValid{ false };
        std::uint32_t generatedBodiesWorldGeneration{ 0 };
        std::uint32_t generatedBodiesSkeletonGeneration{ 0 };
        std::uint32_t generatedBodiesProviderGeneration{ 0 };
        provider::RockProviderLifecycleReason reasonHint{ provider::RockProviderLifecycleReason::None };
    };

    [[nodiscard]] inline bool hasFlag(const RuntimeState& state, provider::RockProviderLifecycleFlag flag)
    {
        return provider::hasLifecycleFlag(state.flags, flag);
    }

    inline void addFlag(std::uint32_t& flags, provider::RockProviderLifecycleFlag flag)
    {
        flags |= static_cast<std::uint32_t>(flag);
    }

    inline void noteReason(RuntimeState& state, provider::RockProviderLifecycleReason reason)
    {
        if (reason != provider::RockProviderLifecycleReason::None) {
            state.lastReason = reason;
        }
    }

    inline void noteSkeletonGeneration(RuntimeState& state, std::uint32_t generation, provider::RockProviderLifecycleReason reason)
    {
        if (generation == 0) {
            generation = 1;
        }

        if (state.skeletonGeneration != generation) {
            state.skeletonGeneration = generation;
            state.stableFrameCount = 0;
            state.generatedBodiesValid = false;
            state.generatedBodiesWorldGeneration = 0;
            state.generatedBodiesSkeletonGeneration = 0;
            state.generatedBodiesProviderGeneration = 0;
            noteReason(state, reason);
        }
    }

    inline void noteProviderGeneration(RuntimeState& state, std::uint32_t generation, provider::RockProviderLifecycleReason reason)
    {
        if (generation == 0) {
            generation = 1;
        }

        if (state.providerGeneration != generation) {
            state.providerGeneration = generation;
            state.stableFrameCount = 0;
            state.generatedBodiesValid = false;
            state.generatedBodiesWorldGeneration = 0;
            state.generatedBodiesSkeletonGeneration = 0;
            state.generatedBodiesProviderGeneration = 0;
            noteReason(state, reason);
        }
    }

    inline void noteWorld(RuntimeState& state, std::uintptr_t bhkWorld, std::uintptr_t hknpWorld, provider::RockProviderLifecycleReason reason)
    {
        const bool worldAvailable = bhkWorld != 0 && hknpWorld != 0;
        if (!worldAvailable) {
            if (state.bhkWorld != 0 || state.hknpWorld != 0) {
                ++state.worldGeneration;
            }
            state.bhkWorld = 0;
            state.hknpWorld = 0;
            state.stableFrameCount = 0;
            state.generatedBodiesValid = false;
            state.generatedBodiesWorldGeneration = 0;
            state.generatedBodiesSkeletonGeneration = 0;
            state.generatedBodiesProviderGeneration = 0;
            noteReason(state, reason == provider::RockProviderLifecycleReason::None ? provider::RockProviderLifecycleReason::WorldUnavailable : reason);
            return;
        }

        if (state.bhkWorld != bhkWorld || state.hknpWorld != hknpWorld) {
            const bool hadWorld = state.bhkWorld != 0 || state.hknpWorld != 0;
            state.bhkWorld = bhkWorld;
            state.hknpWorld = hknpWorld;
            ++state.worldGeneration;
            state.stableFrameCount = 0;
            state.generatedBodiesValid = false;
            state.generatedBodiesWorldGeneration = 0;
            state.generatedBodiesSkeletonGeneration = 0;
            state.generatedBodiesProviderGeneration = 0;
            if (reason != provider::RockProviderLifecycleReason::None) {
                noteReason(state, reason);
            } else {
                noteReason(state, hadWorld ? provider::RockProviderLifecycleReason::WorldChanged : provider::RockProviderLifecycleReason::WorldAvailable);
            }
        }
    }

    [[nodiscard]] inline std::uint32_t buildFlags(const RuntimeState& state, const FrameInputs& input)
    {
        std::uint32_t flags = 0;
        const bool worldAvailable = state.bhkWorld != 0 && state.hknpWorld != 0;
        const bool transitionActive = worldAvailable && state.stableFrameCount < state.settleFramesRequired;
        const bool physicsWriteAllowed =
            input.providerReady &&
            input.skeletonReady &&
            worldAvailable &&
            state.generatedBodiesValid &&
            !input.menuBlocking &&
            !input.configBlocking &&
            !transitionActive;

        if (worldAvailable) {
            addFlag(flags, provider::RockProviderLifecycleFlag::WorldAvailable);
        }
        if (input.skeletonReady) {
            addFlag(flags, provider::RockProviderLifecycleFlag::SkeletonReady);
        }
        if (input.providerReady) {
            addFlag(flags, provider::RockProviderLifecycleFlag::ProviderReady);
        }
        if (input.menuBlocking) {
            addFlag(flags, provider::RockProviderLifecycleFlag::MenuBlocking);
        }
        if (input.configBlocking) {
            addFlag(flags, provider::RockProviderLifecycleFlag::ConfigBlocking);
        }
        if (transitionActive) {
            addFlag(flags, provider::RockProviderLifecycleFlag::LoadingOrWorldTransition);
        }
        if (state.generatedBodiesValid) {
            addFlag(flags, provider::RockProviderLifecycleFlag::GeneratedBodiesValid);
        }
        if (physicsWriteAllowed) {
            addFlag(flags, provider::RockProviderLifecycleFlag::PhysicsWriteAllowed);
        }
        if (input.providerReady && input.skeletonReady) {
            addFlag(flags, provider::RockProviderLifecycleFlag::VisualWriteAllowed);
        }

        return flags;
    }

    inline void observeFrame(RuntimeState& state, const FrameInputs& input)
    {
        noteProviderGeneration(state, input.providerGeneration, provider::RockProviderLifecycleReason::ProviderReady);
        noteSkeletonGeneration(state, input.skeletonGeneration, provider::RockProviderLifecycleReason::SkeletonReady);
        noteWorld(state, input.bhkWorld, input.hknpWorld, input.reasonHint);

        const bool worldAvailable = state.bhkWorld != 0 && state.hknpWorld != 0;
        state.generatedBodiesWorldGeneration = input.generatedBodiesWorldGeneration;
        state.generatedBodiesSkeletonGeneration = input.generatedBodiesSkeletonGeneration;
        state.generatedBodiesProviderGeneration = input.generatedBodiesProviderGeneration;
        const bool generatedBodiesMatchCurrentEpoch =
            input.generatedBodiesValid &&
            input.generatedBodiesWorldGeneration != 0 &&
            input.generatedBodiesWorldGeneration == state.worldGeneration &&
            input.generatedBodiesSkeletonGeneration == state.skeletonGeneration &&
            input.generatedBodiesProviderGeneration == state.providerGeneration;
        state.generatedBodiesValid = generatedBodiesMatchCurrentEpoch && worldAvailable && input.skeletonReady;
        if (worldAvailable && input.providerReady && input.skeletonReady && state.generatedBodiesValid) {
            if (state.stableFrameCount < state.settleFramesRequired) {
                ++state.stableFrameCount;
                if (state.stableFrameCount >= state.settleFramesRequired) {
                    noteReason(state, provider::RockProviderLifecycleReason::TransitionSettled);
                }
            }
        } else if (!worldAvailable || !input.skeletonReady || !state.generatedBodiesValid) {
            state.stableFrameCount = 0;
        }

        if (input.menuBlocking) {
            noteReason(state, provider::RockProviderLifecycleReason::MenuBlocked);
        } else if (input.configBlocking) {
            noteReason(state, provider::RockProviderLifecycleReason::ConfigBlocked);
        } else if (input.reasonHint != provider::RockProviderLifecycleReason::None) {
            noteReason(state, input.reasonHint);
        }

        state.flags = buildFlags(state, input);
    }
}

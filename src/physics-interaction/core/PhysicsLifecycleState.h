#pragma once

#include "api/ProviderRuntimeTypes.h"

#include <cstdint>

namespace rock::physics_lifecycle
{
    /*
     * Published lifecycle state for the Core provider snapshot. ROCK destroys
     * PhysicsInteraction on skeleton/session changes and re-inits on world
     * change, so physics writes open as soon as the generated bodies exist in
     * the current world; there is no extra settle delay.
     * LoadingOrWorldTransition means "world present, bodies not ready yet".
     */
    struct RuntimeState
    {
        std::uintptr_t bhkWorld{ 0 };
        std::uintptr_t hknpWorld{ 0 };
        std::uint32_t worldGeneration{ 1 };
        std::uint32_t skeletonGeneration{ 1 };
        std::uint32_t providerGeneration{ 1 };
        std::uint32_t stableFrameCount{ 0 };
        std::uint32_t flags{ 0 };
        provider::RockProviderLifecycleReason lastReason{ provider::RockProviderLifecycleReason::None };
    };

    struct FrameInputs
    {
        std::uintptr_t bhkWorld{ 0 };
        std::uintptr_t hknpWorld{ 0 };
        bool providerReady{ false };
        bool skeletonReady{ false };
        bool menuBlocking{ false };
        bool configBlocking{ false };
        bool generatedBodiesValid{ false };
        provider::RockProviderLifecycleReason reasonHint{ provider::RockProviderLifecycleReason::None };
    };

    inline void noteReason(RuntimeState& state, provider::RockProviderLifecycleReason reason)
    {
        if (reason != provider::RockProviderLifecycleReason::None) {
            state.lastReason = reason;
        }
    }

    inline void noteGeneration(std::uint32_t& current, std::uint32_t generation, RuntimeState& state, provider::RockProviderLifecycleReason reason)
    {
        generation = generation == 0 ? 1 : generation;
        if (current != generation) {
            current = generation;
            state.stableFrameCount = 0;
            noteReason(state, reason);
        }
    }

    inline void observeFrame(RuntimeState& state, const FrameInputs& input)
    {
        const bool worldAvailable = input.bhkWorld != 0 && input.hknpWorld != 0;
        if (state.bhkWorld != input.bhkWorld || state.hknpWorld != input.hknpWorld) {
            const bool hadWorld = state.bhkWorld != 0 || state.hknpWorld != 0;
            if (worldAvailable || hadWorld) {
                ++state.worldGeneration;
            }
            state.bhkWorld = input.bhkWorld;
            state.hknpWorld = input.hknpWorld;
            state.stableFrameCount = 0;
            using Reason = provider::RockProviderLifecycleReason;
            noteReason(state, input.reasonHint != Reason::None ? input.reasonHint :
                !worldAvailable ? Reason::WorldUnavailable : hadWorld ? Reason::WorldChanged : Reason::WorldAvailable);
        }

        const bool bodiesValid = worldAvailable && input.skeletonReady && input.generatedBodiesValid;
        const bool live = bodiesValid && input.providerReady;
        if (live && state.stableFrameCount == 0) {
            state.stableFrameCount = 1;
            noteReason(state, provider::RockProviderLifecycleReason::TransitionSettled);
        } else if (!live) {
            state.stableFrameCount = 0;
        }

        if (input.menuBlocking) {
            noteReason(state, provider::RockProviderLifecycleReason::MenuBlocked);
        } else if (input.configBlocking) {
            noteReason(state, provider::RockProviderLifecycleReason::ConfigBlocked);
        } else {
            noteReason(state, input.reasonHint);
        }

        using Flag = provider::RockProviderLifecycleFlag;
        std::uint32_t flags = 0;
        const auto add = [&flags](Flag flag, bool on) {
            if (on) flags |= static_cast<std::uint32_t>(flag);
        };
        add(Flag::WorldAvailable, worldAvailable);
        add(Flag::SkeletonReady, input.skeletonReady);
        add(Flag::ProviderReady, input.providerReady);
        add(Flag::MenuBlocking, input.menuBlocking);
        add(Flag::ConfigBlocking, input.configBlocking);
        add(Flag::LoadingOrWorldTransition, worldAvailable && !live);
        add(Flag::GeneratedBodiesValid, bodiesValid);
        add(Flag::PhysicsWriteAllowed, live && !input.menuBlocking && !input.configBlocking);
        add(Flag::VisualWriteAllowed, input.providerReady && input.skeletonReady);
        state.flags = flags;
    }
}

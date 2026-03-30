#include <cstdio>

#include "physics-interaction/core/PhysicsLifecycleState.h"

namespace
{
    bool expectBool(const char* label, bool actual, bool expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %d got %d\n", label, expected ? 1 : 0, actual ? 1 : 0);
        return false;
    }

    bool expectUInt(const char* label, std::uint32_t actual, std::uint32_t expected)
    {
        if (actual == expected) {
            return true;
        }

        std::printf("%s expected %u got %u\n", label, expected, actual);
        return false;
    }

    bool first_valid_world_settles_before_writes()
    {
        using namespace rock;
        using namespace rock::provider;
        using namespace rock::physics_lifecycle;

        RuntimeState state{};
        state.settleFramesRequired = 2;

        FrameInputs frame{};
        frame.bhkWorld = 0x1000;
        frame.hknpWorld = 0x2000;
        frame.providerReady = true;
        frame.skeletonReady = true;
        frame.generatedBodiesValid = false;

        observeFrame(state, frame);

        bool ok = true;
        ok &= expectUInt("first valid world increments generation", state.worldGeneration, 2);
        ok &= expectUInt("first valid world has no stable frame before generated rebuild", state.stableFrameCount, 0);
        ok &= expectBool("first valid world is available", hasFlag(state, RockProviderLifecycleFlag::WorldAvailable), true);
        ok &= expectBool("first valid world is still settling", hasFlag(state, RockProviderLifecycleFlag::LoadingOrWorldTransition), true);
        ok &= expectBool("first valid world blocks physics writes", hasFlag(state, RockProviderLifecycleFlag::PhysicsWriteAllowed), false);

        frame.generatedBodiesValid = true;
        frame.generatedBodiesWorldGeneration = state.worldGeneration;
        frame.generatedBodiesSkeletonGeneration = state.skeletonGeneration;
        frame.generatedBodiesProviderGeneration = state.providerGeneration;
        observeFrame(state, frame);
        ok &= expectUInt("generated rebuild frame has one stable frame", state.stableFrameCount, 1);
        ok &= expectBool("generated rebuild frame still blocks physics writes", hasFlag(state, RockProviderLifecycleFlag::PhysicsWriteAllowed), false);

        observeFrame(state, frame);
        ok &= expectUInt("second valid world reaches settle count", state.stableFrameCount, 2);
        ok &= expectBool("settled world opens physics writes", hasFlag(state, RockProviderLifecycleFlag::PhysicsWriteAllowed), true);
        ok &= expectBool("settled world clears transition flag", hasFlag(state, RockProviderLifecycleFlag::LoadingOrWorldTransition), false);
        ok &= expectBool(
            "settled world records transition reason",
            state.lastReason == RockProviderLifecycleReason::TransitionSettled,
            true);

        return ok;
    }

    bool menu_blocks_physics_but_not_visual_state()
    {
        using namespace rock;
        using namespace rock::provider;
        using namespace rock::physics_lifecycle;

        RuntimeState state{};
        state.settleFramesRequired = 1;

        FrameInputs frame{};
        frame.bhkWorld = 0x1000;
        frame.hknpWorld = 0x2000;
        frame.providerReady = true;
        frame.skeletonReady = true;
        frame.generatedBodiesValid = true;
        frame.generatedBodiesWorldGeneration = 2;
        frame.generatedBodiesSkeletonGeneration = 1;
        frame.generatedBodiesProviderGeneration = 1;
        observeFrame(state, frame);
        frame.generatedBodiesWorldGeneration = state.worldGeneration;
        frame.generatedBodiesSkeletonGeneration = state.skeletonGeneration;
        frame.generatedBodiesProviderGeneration = state.providerGeneration;
        observeFrame(state, frame);

        frame.menuBlocking = true;
        observeFrame(state, frame);

        bool ok = true;
        ok &= expectBool("menu frame keeps visual writes available", hasFlag(state, RockProviderLifecycleFlag::VisualWriteAllowed), true);
        ok &= expectBool("menu frame reports menu block", hasFlag(state, RockProviderLifecycleFlag::MenuBlocking), true);
        ok &= expectBool("menu frame blocks physics writes", hasFlag(state, RockProviderLifecycleFlag::PhysicsWriteAllowed), false);
        ok &= expectBool("menu frame records reason", state.lastReason == RockProviderLifecycleReason::MenuBlocked, true);
        return ok;
    }

    bool world_and_skeleton_changes_close_writes()
    {
        using namespace rock;
        using namespace rock::provider;
        using namespace rock::physics_lifecycle;

        RuntimeState state{};
        state.settleFramesRequired = 2;

        FrameInputs frame{};
        frame.bhkWorld = 0x1000;
        frame.hknpWorld = 0x2000;
        frame.providerReady = true;
        frame.skeletonReady = true;
        frame.generatedBodiesValid = true;
        frame.generatedBodiesWorldGeneration = 2;
        frame.generatedBodiesSkeletonGeneration = 1;
        frame.generatedBodiesProviderGeneration = 1;
        observeFrame(state, frame);
        frame.generatedBodiesWorldGeneration = state.worldGeneration;
        frame.generatedBodiesSkeletonGeneration = state.skeletonGeneration;
        frame.generatedBodiesProviderGeneration = state.providerGeneration;
        observeFrame(state, frame);
        observeFrame(state, frame);

        frame.hknpWorld = 0x3000;
        observeFrame(state, frame);

        bool ok = true;
        ok &= expectUInt("world change increments generation", state.worldGeneration, 3);
        ok &= expectUInt("world change restarts settle count", state.stableFrameCount, 0);
        ok &= expectBool("world change closes physics writes", hasFlag(state, RockProviderLifecycleFlag::PhysicsWriteAllowed), false);

        frame.generatedBodiesWorldGeneration = state.worldGeneration;
        frame.generatedBodiesSkeletonGeneration = state.skeletonGeneration;
        frame.generatedBodiesProviderGeneration = state.providerGeneration;
        observeFrame(state, frame);
        ok &= expectUInt("world rebuild starts settle count", state.stableFrameCount, 1);

        frame.skeletonGeneration = 2;
        observeFrame(state, frame);
        ok &= expectUInt("skeleton change records generation", state.skeletonGeneration, 2);
        ok &= expectBool("skeleton change keeps transition gate closed", hasFlag(state, RockProviderLifecycleFlag::LoadingOrWorldTransition), true);
        ok &= expectBool("skeleton change keeps physics writes closed", hasFlag(state, RockProviderLifecycleFlag::PhysicsWriteAllowed), false);

        frame.generatedBodiesWorldGeneration = state.worldGeneration;
        frame.generatedBodiesSkeletonGeneration = state.skeletonGeneration;
        frame.generatedBodiesProviderGeneration = state.providerGeneration;
        observeFrame(state, frame);
        ok &= expectUInt("skeleton rebuild starts settle count", state.stableFrameCount, 1);

        frame.hknpWorld = 0;
        observeFrame(state, frame);
        ok &= expectBool("world unavailable clears world flag", hasFlag(state, RockProviderLifecycleFlag::WorldAvailable), false);
        ok &= expectBool("world unavailable records reason", state.lastReason == RockProviderLifecycleReason::WorldUnavailable, true);
        return ok;
    }
}

int main()
{
    bool ok = true;
    ok &= first_valid_world_settles_before_writes();
    ok &= menu_blocks_physics_but_not_visual_state();
    ok &= world_and_skeleton_changes_close_writes();
    return ok ? 0 : 1;
}

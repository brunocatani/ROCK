#include "ExampleRuntime.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    constexpr std::uint64_t kSurfaceScope{ 0x5355524641434501ull };
    constexpr std::uint64_t kRightSurfaceTarget{ 0x5355524652494748ull };
    constexpr std::uint64_t kLeftSurfaceTarget{ 0x535552464C454654ull };

    std::uint32_t g_targetGeneration{ 1 };
    std::uint32_t g_worldGeneration{ 0 };
    std::uint32_t g_skeletonGeneration{ 0 };
    std::uint32_t g_providerGeneration{ 0 };
    std::array<std::uint64_t, 2> g_lastStateSequences{};
    bool g_scopePublished{ false };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    constexpr std::uint32_t targetFlag(
        const RockProviderTouchGrabTargetFlagV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    void clearScope(const std::uint64_t ownerToken) noexcept
    {
        if (g_scopePublished) {
            (void)RockProviderApi::inst->clearTouchGrabTargetsForScopeV1(
                ownerToken,
                kSurfaceScope);
            g_scopePublished = false;
        }
    }

    bool start(const std::uint64_t) noexcept
    {
        g_targetGeneration = 1;
        g_worldGeneration = 0;
        g_skeletonGeneration = 0;
        g_providerGeneration = 0;
        g_lastStateSequences = {};
        g_scopePublished = false;
        return true;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clearScope(ownerToken);
    }

    void reportTransitions(const std::uint64_t ownerToken) noexcept
    {
        std::array<RockProviderTouchGrabStateV1, 2> states{};
        std::uint32_t stateCount{ 0 };
        if (RockProviderApi::inst->copyTouchGrabStatesForScopeV1(
                ownerToken,
                kSurfaceScope,
                states.data(),
                static_cast<std::uint32_t>(states.size()),
                &stateCount) != RockProviderResultV1::Ok) {
            return;
        }

        for (std::uint32_t index = 0; index < stateCount; ++index) {
            const auto& state = states[index];
            const auto slot = state.targetId == kRightSurfaceTarget ? 0u : 1u;
            if (state.sequence == g_lastStateSequences[slot]) {
                continue;
            }
            g_lastStateSequences[slot] = state.sequence;
            char message[224]{};
            std::snprintf(
                message,
                sizeof(message),
                "%s surface target phase=%u body=%08X hands=%u releaseReason=%u",
                slot == 0 ? "Right" : "Left",
                static_cast<std::uint32_t>(state.phase),
                state.bodyId,
                state.activeHandMask,
                static_cast<std::uint32_t>(state.releaseReason));
            rock::sdk::example::logInfo(message);
        }
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            clearScope(ownerToken);
            return;
        }

        if (snapshot.worldGeneration != g_worldGeneration ||
            snapshot.skeletonGeneration != g_skeletonGeneration ||
            snapshot.providerGeneration != g_providerGeneration) {
            ++g_targetGeneration;
            if (g_targetGeneration == 0) {
                g_targetGeneration = 1;
            }
            g_worldGeneration = snapshot.worldGeneration;
            g_skeletonGeneration = snapshot.skeletonGeneration;
            g_providerGeneration = snapshot.providerGeneration;
            g_lastStateSequences = {};
        }

        std::array<RockProviderTouchGrabTargetV1, 2> targets{};
        targets[0].targetId = kRightSurfaceTarget;
        targets[0].flags =
            targetFlag(RockProviderTouchGrabTargetFlagV1::AllowRightHand) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchAnyBody) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchStaticMotion) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion);
        targets[1] = targets[0];
        targets[1].targetId = kLeftSurfaceTarget;
        targets[1].flags =
            targetFlag(RockProviderTouchGrabTargetFlagV1::AllowLeftHand) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchAnyBody) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchStaticMotion) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion);

        for (auto& target : targets) {
            target.targetGeneration = g_targetGeneration;
            target.kind = RockProviderTouchGrabKindV1::FixedAnchor;
            target.allowedLayerMask = ~std::uint64_t{ 0 };
            target.leaseFrames = 2;
            target.worldGeneration = snapshot.worldGeneration;
            target.skeletonGeneration = snapshot.skeletonGeneration;
            target.providerGeneration = snapshot.providerGeneration;
        }

        const auto result = RockProviderApi::inst->setTouchGrabTargetsForScopeV1(
            ownerToken,
            kSurfaceScope,
            targets.data(),
            static_cast<std::uint32_t>(targets.size()));
        g_scopePublished = result == RockProviderResultV1::Ok;
        if (g_scopePublished) {
            reportTransitions(ownerToken);
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKSurfaceClimber",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::TouchGrabTargets),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_TOUCH_GRAB_TARGETS_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

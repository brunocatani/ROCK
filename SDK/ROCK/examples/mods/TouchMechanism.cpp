#include "ExampleRuntime.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    // Replace these values from your own mechanism resolver, then opt in.
    bool kEnableMechanism = false;
    constexpr std::uint32_t kMechanismBodyId = 0x7FFF'FFFF;
    constexpr std::uint64_t kScopeToken = 0x4D454348414E4953ull;
    constexpr std::uint64_t kTargetId = 0x505249534D415449ull;

    std::uint32_t g_targetGeneration{ 1 };
    std::uint64_t g_lastSequence{ 0 };
    bool g_published{ false };

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

    void clear(const std::uint64_t ownerToken) noexcept
    {
        if (g_published) {
            (void)RockProviderApi::inst->clearTouchGrabTargetsForScopeV1(
                ownerToken,
                kScopeToken);
            g_published = false;
        }
    }

    bool start(const std::uint64_t) noexcept
    {
        g_targetGeneration = 1;
        g_lastSequence = 0;
        g_published = false;
        if (!kEnableMechanism) {
            rock::sdk::example::logInfo(
                "Touch mechanism is opt-in; provide a current body, pivot, axis, and coordinate");
        }
        return true;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableMechanism || kMechanismBodyId == 0x7FFF'FFFF ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            clear(ownerToken);
            return;
        }

        RockProviderTouchGrabTargetV1 target{};
        target.targetId = kTargetId;
        target.targetGeneration = g_targetGeneration;
        target.kind = RockProviderTouchGrabKindV1::LimitedPrismatic;
        target.flags =
            targetFlag(RockProviderTouchGrabTargetFlagV1::AllowRightHand) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::AllowLeftHand) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchKeyframedMotion) |
            targetFlag(RockProviderTouchGrabTargetFlagV1::MatchDynamicMotion);
        target.bodyId = kMechanismBodyId;
        target.leaseFrames = 2;
        target.worldGeneration = snapshot.worldGeneration;
        target.skeletonGeneration = snapshot.skeletonGeneration;
        target.providerGeneration = snapshot.providerGeneration;
        target.pivotWorldGame = { 0.0f, 0.0f, 0.0f };
        target.axisWorldGame = { 1.0f, 0.0f, 0.0f };
        target.minimumCoordinate = 0.0f;
        target.maximumCoordinate = 10.0f;
        target.currentCoordinate = 0.0f;

        g_published = RockProviderApi::inst->setTouchGrabTargetsForScopeV1(
                          ownerToken,
                          kScopeToken,
                          &target,
                          1) == RockProviderResultV1::Ok;
        if (!g_published) {
            return;
        }

        std::array<RockProviderTouchGrabStateV1, 1> states{};
        std::uint32_t stateCount{ 0 };
        if (RockProviderApi::inst->copyTouchGrabStatesForScopeV1(
                ownerToken,
                kScopeToken,
                states.data(),
                static_cast<std::uint32_t>(states.size()),
                &stateCount) != RockProviderResultV1::Ok ||
            stateCount == 0 || states[0].sequence == g_lastSequence) {
            return;
        }

        g_lastSequence = states[0].sequence;
        char message[192]{};
        std::snprintf(
            message,
            sizeof(message),
            "Mechanism phase=%u coordinate=%.3f hands=%u release=%u",
            static_cast<std::uint32_t>(states[0].phase),
            states[0].currentCoordinate,
            states[0].activeHandMask,
            static_cast<std::uint32_t>(states[0].releaseReason));
        rock::sdk::example::logInfo(message);
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKTouchMechanism",
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

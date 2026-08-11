#include "ExampleRuntime.h"

namespace
{
    using namespace rock::provider;

    // This intentionally changes presentation only. Keep it disabled until a
    // mod has a clear visual-authority activation and blending policy.
    bool kEnableVisualOffset = false;

    bool g_published{ false };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        g_published = false;
        if (!kEnableVisualOffset) {
            rock::sdk::example::logInfo(
                "Visual-hand authority is opt-in; define activation and blending before enabling");
        }
        return true;
    }

    void clear(const std::uint64_t ownerToken) noexcept
    {
        if (g_published) {
            (void)RockProviderApi::inst->clearHandVisualAuthorityV1(
                ownerToken,
                RockProviderHand::Right);
            g_published = false;
        }
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableVisualOffset ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::VisualWriteAllowed)) {
            clear(ownerToken);
            return;
        }

        RockProviderHandFrameV1 presented{};
        if (!RockProviderApi::inst->getPresentedHandFrameV1(
                RockProviderHand::Right,
                &presented)) {
            clear(ownerToken);
            return;
        }

        RockProviderHandVisualAuthorityRequestV1 request{};
        request.hand = RockProviderHand::Right;
        request.flags = static_cast<std::uint32_t>(
            RockProviderHandVisualAuthorityFlagV1::WorldTransform);
        request.priority = 50;
        request.worldTransform = presented.transform;
        request.worldTransform.translate[2] += 2.0f;
        request.leaseFrames = 2;
        request.worldGeneration = snapshot.worldGeneration;
        request.skeletonGeneration = snapshot.skeletonGeneration;
        request.providerGeneration = snapshot.providerGeneration;
        g_published = RockProviderApi::inst->setHandVisualAuthorityV1(
                          ownerToken,
                          &request) == RockProviderResultV1::Ok;
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKVisualHandOffset",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::HandVisualAuthority),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_PRESENTED_HAND_FRAMES_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

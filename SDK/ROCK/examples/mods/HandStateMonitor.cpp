#include "ExampleRuntime.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    std::uint64_t g_lastStateSequence{ 0 };
    std::uint64_t g_lastEventSequence{ 0 };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    void reportHand(
        const std::uint64_t ownerToken,
        const RockProviderHand hand) noexcept
    {
        RockProviderHandInteractionStateV1 state{};
        if (RockProviderApi::inst->getHandInteractionStateV1(
                ownerToken,
                hand,
                &state) != RockProviderResultV1::Ok) {
            return;
        }

        char message[224]{};
        std::snprintf(
            message,
            sizeof(message),
            "%s hand phase=%u targetKind=%u form=%08X body=%08X heldBodies=%u",
            hand == RockProviderHand::Right ? "Right" : "Left",
            static_cast<std::uint32_t>(state.phase),
            static_cast<std::uint32_t>(state.targetKind),
            state.targetFormId,
            state.primaryBodyId,
            state.heldBodyCount);
        rock::sdk::example::logInfo(message);
    }

    void pollEvents(const std::uint64_t ownerToken) noexcept
    {
        std::array<RockProviderEventV1, 16> events{};
        RockProviderEventStreamStateV1 stream{};
        const auto result = RockProviderApi::inst->copyProviderEventsSinceV1(
            ownerToken,
            g_lastEventSequence,
            events.data(),
            static_cast<std::uint32_t>(events.size()),
            &stream);
        if (result != RockProviderResultV1::Ok) {
            return;
        }

        for (std::uint32_t index = 0; index < stream.copiedCount; ++index) {
            const auto& event = events[index];
            char message[192]{};
            std::snprintf(
                message,
                sizeof(message),
                "Event sequence=%llu kind=%u hand=%u form=%08X result=%u",
                static_cast<unsigned long long>(event.sequence),
                static_cast<std::uint32_t>(event.kind),
                static_cast<std::uint32_t>(event.hand),
                event.formId,
                event.result);
            rock::sdk::example::logInfo(message);
        }
        if (stream.copiedCount != 0) {
            g_lastEventSequence = stream.lastCopiedSequence;
        }
    }

    bool start(const std::uint64_t) noexcept
    {
        g_lastStateSequence = 0;
        g_lastEventSequence = 0;
        return true;
    }

    void stop(const std::uint64_t) noexcept
    {
        g_lastStateSequence = 0;
        g_lastEventSequence = 0;
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (snapshot.stateSequence != g_lastStateSequence) {
            reportHand(ownerToken, RockProviderHand::Right);
            reportHand(ownerToken, RockProviderHand::Left);
            g_lastStateSequence = snapshot.stateSequence;
        }
        pollEvents(ownerToken);
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKHandStateMonitor",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::HandInteractionState) |
                capability(provider::RockProviderConsumerCapabilityV1::ProviderEvents),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_PROVIDER_EVENTS_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

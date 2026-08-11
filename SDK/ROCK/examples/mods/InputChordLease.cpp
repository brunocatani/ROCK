#include "ExampleRuntime.h"

#include <cstdio>

namespace
{
    using namespace rock::provider;

    constexpr std::uint32_t kTriggerButtonId = 33;
    constexpr std::uint32_t kFaceButtonId = 7;
    constexpr std::uint32_t kLeaseFrames = 3;

    bool g_leasePublished{ false };
    std::uint32_t g_lastEffectiveFlags{ ~std::uint32_t{ 0 } };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        g_leasePublished = false;
        g_lastEffectiveFlags = ~std::uint32_t{ 0 };
        return true;
    }

    void clear(const std::uint64_t ownerToken) noexcept
    {
        if (!RockProviderApi::inst) {
            return;
        }
        (void)RockProviderApi::inst->clearHandInputSuppressionV1(
            ownerToken,
            RockProviderHand::Right);
        (void)RockProviderApi::inst->clearHandInputSuppressionV1(
            ownerToken,
            RockProviderHand::Left);
        g_leasePublished = false;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        const auto hand = RockProviderApi::inst->getOffhandHandV1();
        RockProviderRawWandButtonStateV1 trigger{};
        RockProviderRawWandButtonStateV1 face{};
        const bool sampled = hand != RockProviderHand::None &&
            RockProviderApi::inst->getRawWandButtonStateV1(
                hand,
                kTriggerButtonId,
                &trigger) &&
            RockProviderApi::inst->getRawWandButtonStateV1(
                hand,
                kFaceButtonId,
                &face) &&
            trigger.available != 0 && face.available != 0;
        const bool chordHeld = sampled && trigger.held != 0 && face.held != 0;

        if (chordHeld) {
            RockProviderHandInputSuppressionRequestV1 request{};
            request.hand = hand;
            request.flags = static_cast<std::uint32_t>(
                RockProviderHandInputSuppressionFlagV1::SuppressConfigModeChord);
            request.leaseFrames = kLeaseFrames;
            request.worldGeneration = snapshot.worldGeneration;
            request.skeletonGeneration = snapshot.skeletonGeneration;
            request.providerGeneration = snapshot.providerGeneration;
            g_leasePublished =
                RockProviderApi::inst->setHandInputSuppressionV1(
                    ownerToken,
                    &request) == RockProviderResultV1::Ok;
        } else if (g_leasePublished) {
            clear(ownerToken);
        }

        RockProviderHandInputSuppressionStateV1 state{};
        if (hand != RockProviderHand::None &&
            RockProviderApi::inst->getHandInputSuppressionStateV1(
                ownerToken,
                hand,
                &state) == RockProviderResultV1::Ok &&
            state.effectiveFlags != g_lastEffectiveFlags) {
            g_lastEffectiveFlags = state.effectiveFlags;
            char message[192]{};
            std::snprintf(
                message,
                sizeof(message),
                "Input lease hand=%u caller=%08X effective=%08X remaining=%u",
                static_cast<std::uint32_t>(hand),
                state.callerFlags,
                state.effectiveFlags,
                state.callerRemainingFrames);
            rock::sdk::example::logInfo(message);
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKInputChordLease",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::HandInputSuppression) |
                capability(provider::RockProviderConsumerCapabilityV1::InputObservability),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_INPUT_OBSERVABILITY_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

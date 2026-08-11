#include "ExampleRuntime.h"

#include <cstdio>

namespace
{
    using namespace rock::provider;

    std::uint64_t g_phaseCallbackToken{ 0 };
    std::uint64_t g_lastWeaponGeneration{ ~std::uint64_t{ 0 } };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    void ROCK_PROVIDER_CALL phaseCallback(
        const RockProviderAnimationPhaseContextV1* context,
        void*)
    {
        if (!context || context->phase != RockProviderAnimationPhaseV1::Complete ||
            context->frameIndex % 300 != 0) {
            return;
        }
        char message[192]{};
        std::snprintf(
            message,
            sizeof(message),
            "Animation complete frame=%llu authority=%08X phaseFlags=%08X",
            static_cast<unsigned long long>(context->frameIndex),
            context->activeNativeAnimationAuthorityFlags,
            context->flags);
        rock::sdk::example::logInfo(message);
    }

    bool start(const std::uint64_t ownerToken) noexcept
    {
        g_phaseCallbackToken = 0;
        g_lastWeaponGeneration = ~std::uint64_t{ 0 };
        return RockProviderApi::inst->registerAnimationPhaseCallbackV1(
                   ownerToken,
                   &phaseCallback,
                   nullptr,
                   &g_phaseCallbackToken) == RockProviderResultV1::Ok &&
            g_phaseCallbackToken != 0;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        if (g_phaseCallbackToken != 0) {
            (void)RockProviderApi::inst->unregisterAnimationPhaseCallbackV1(
                ownerToken,
                g_phaseCallbackToken);
            g_phaseCallbackToken = 0;
        }
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (snapshot.weaponGenerationKey == g_lastWeaponGeneration) {
            return;
        }
        g_lastWeaponGeneration = snapshot.weaponGenerationKey;

        RockProviderNativeAnimationAuthorityStateV1 authority{};
        const bool authorityAvailable =
            RockProviderApi::inst->getNativeAnimationAuthorityStateV1(&authority);
        RockProviderAuthoredGripPoseV1 authored{};
        const auto authoredResult =
            RockProviderApi::inst->getSelectedAuthoredGripPoseV1(
                ownerToken,
                &authored);
        RockProviderPresentedHandPoseV1 presented{};
        const auto presentedResult =
            RockProviderApi::inst->getPresentedHandPoseV1(
                ownerToken,
                RockProviderHand::Right,
                &presented);

        char message[256]{};
        std::snprintf(
            message,
            sizeof(message),
            "Animation observation weaponGeneration=%llu authority=%s flags=%08X "
            "authoredPose=%u presentedPose=%u",
            static_cast<unsigned long long>(snapshot.weaponGenerationKey),
            authorityAvailable ? "available" : "unavailable",
            authority.activeFlags,
            static_cast<std::uint32_t>(authoredResult),
            static_cast<std::uint32_t>(presentedResult));
        rock::sdk::example::logInfo(message);
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKAnimationObserver",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::AnimationPhases) |
                capability(provider::RockProviderConsumerCapabilityV1::NativeAnimationAuthority) |
                capability(provider::RockProviderConsumerCapabilityV1::PoseReadback),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_POSE_READBACK_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

#include "ExampleRuntime.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    std::uint64_t g_lastWeaponGeneration{ ~std::uint64_t{ 0 } };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        g_lastWeaponGeneration = ~std::uint64_t{ 0 };
        return true;
    }

    void stop(const std::uint64_t) noexcept
    {
        g_lastWeaponGeneration = ~std::uint64_t{ 0 };
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (snapshot.weaponGenerationKey == g_lastWeaponGeneration) {
            return;
        }
        g_lastWeaponGeneration = snapshot.weaponGenerationKey;

        if (snapshot.weaponGenerationKey == 0 || snapshot.weaponFormId == 0) {
            rock::sdk::example::logInfo("No equipped weapon is published");
            return;
        }

        RockProviderWeaponCompositionStateV1 composition{};
        std::array<RockProviderWeaponCompositionEntryV1, 64> entries{};
        std::uint32_t entryCount{ 0 };
        const auto compositionResult =
            RockProviderApi::inst->getWeaponCompositionStateV1(
                ownerToken,
                &composition);
        if (compositionResult == RockProviderResultV1::Ok) {
            (void)RockProviderApi::inst->copyWeaponCompositionEntriesV1(
                ownerToken,
                entries.data(),
                static_cast<std::uint32_t>(entries.size()),
                &entryCount);
        }

        std::array<RockProviderWeaponPartPoseV1, 128> partPoses{};
        std::uint32_t partPoseCount{ 0 };
        (void)RockProviderApi::inst->copyWeaponPartPoseSnapshotV1(
            ownerToken,
            partPoses.data(),
            static_cast<std::uint32_t>(partPoses.size()),
            &partPoseCount);

        RockProviderScopeSightStateV1 scope{};
        const auto scopeResult = RockProviderApi::inst->getScopeSightStateV1(
            ownerToken,
            &scope);

        char message[256]{};
        std::snprintf(
            message,
            sizeof(message),
            "Weapon=%08X generation=%llu bodies=%u compositionEntries=%u partPoses=%u scope=%s",
            snapshot.weaponFormId,
            static_cast<unsigned long long>(snapshot.weaponGenerationKey),
            snapshot.weaponBodyCount,
            entryCount,
            partPoseCount,
            scopeResult == RockProviderResultV1::Ok ? "published" : "unavailable");
        rock::sdk::example::logInfo(message);
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKWeaponInspector",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::WeaponPartObservability) |
                capability(provider::RockProviderConsumerCapabilityV1::WeaponComposition) |
                capability(provider::RockProviderConsumerCapabilityV1::PoseReadback) |
                capability(provider::RockProviderConsumerCapabilityV1::ScopeSightState),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_POSE_READBACK_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

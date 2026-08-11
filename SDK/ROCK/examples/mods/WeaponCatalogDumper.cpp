#include "ExampleRuntime.h"

#include <algorithm>
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
        const std::uint64_t,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (snapshot.weaponGenerationKey == g_lastWeaponGeneration) {
            return;
        }
        g_lastWeaponGeneration = snapshot.weaponGenerationKey;

        RockProviderWeaponClassificationV1 classification{};
        const bool classified =
            RockProviderApi::inst->queryEquippedWeaponClassificationV1(
                &classification);

        std::array<RockProviderWeaponEvidenceDetailV1,
            ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1> details{};
        const auto detailCount = RockProviderApi::inst->copyWeaponEvidenceDetailsV1(
            details.data(),
            static_cast<std::uint32_t>(details.size()));

        std::array<RockProviderWeaponEmitterV1,
            ROCK_PROVIDER_MAX_WEAPON_EMITTERS_V1> emitters{};
        const auto emitterCount = RockProviderApi::inst->copyWeaponEmittersV1(
            emitters.data(),
            static_cast<std::uint32_t>(emitters.size()));

        char summary[256]{};
        std::snprintf(
            summary,
            sizeof(summary),
            "Weapon=%08X generation=%llu classification=%s sizeClass=%u "
            "evidence=%u emitters=%u",
            snapshot.weaponFormId,
            static_cast<unsigned long long>(snapshot.weaponGenerationKey),
            classified && classification.valid != 0 ? "valid" : "unavailable",
            static_cast<std::uint32_t>(classification.sizeClass),
            detailCount,
            emitterCount);
        rock::sdk::example::logInfo(summary);

        const auto rowsToLog = (std::min)(detailCount, 4u);
        for (std::uint32_t index = 0; index < rowsToLog; ++index) {
            const auto& detail = details[index];
            char row[288]{};
            std::snprintf(
                row,
                sizeof(row),
                "Part[%u] body=%08X kind=%u action=%u OMOD=%08X attach=%08X "
                "points=%u name=%s",
                index,
                detail.bodyId,
                detail.partKind,
                detail.actionRole,
                detail.omodFormId,
                detail.attachPointFormId,
                detail.pointCount,
                detail.sourceName);
            rock::sdk::example::logInfo(row);
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKWeaponCatalogDumper",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::WeaponPartObservability),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_WEAPON_EMITTERS_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

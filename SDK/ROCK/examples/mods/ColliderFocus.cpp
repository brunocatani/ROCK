#include "ExampleRuntime.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    // Focus replaces ROCK's normal collider overlay while its lease is alive.
    // Keep it opt-in in a general SDK package.
    bool kEnableFocus = false;
    constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

    std::uint64_t g_weaponGeneration{ 0 };
    std::uint32_t g_bodyId{ kInvalidBodyId };
    std::uint32_t g_partKind{ 0 };
    bool g_published{ false };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    void clear(const std::uint64_t ownerToken) noexcept
    {
        if (g_published) {
            (void)RockProviderApi::inst->clearColliderVisualizationOverrideV1(
                ownerToken);
            g_published = false;
        }
    }

    bool start(const std::uint64_t) noexcept
    {
        g_weaponGeneration = 0;
        g_bodyId = kInvalidBodyId;
        g_partKind = 0;
        g_published = false;
        if (!kEnableFocus) {
            rock::sdk::example::logInfo(
                "Collider focus is opt-in because it temporarily replaces the normal overlay");
        }
        return true;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    bool selectBody(const RockProviderFrameSnapshot& snapshot) noexcept
    {
        std::array<RockProviderWeaponEvidenceDetailV1,
            ROCK_PROVIDER_MAX_WEAPON_EVIDENCE_DETAILS_V1> details{};
        const auto count = RockProviderApi::inst->copyWeaponEvidenceDetailsV1(
            details.data(),
            static_cast<std::uint32_t>(details.size()));
        for (std::uint32_t index = 0; index < count; ++index) {
            if (details[index].weaponGenerationKey == snapshot.weaponGenerationKey &&
                details[index].bodyId != kInvalidBodyId) {
                g_weaponGeneration = snapshot.weaponGenerationKey;
                g_bodyId = details[index].bodyId;
                g_partKind = details[index].partKind;
                return true;
            }
        }
        return false;
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableFocus || snapshot.weaponGenerationKey == 0 ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::VisualWriteAllowed)) {
            clear(ownerToken);
            return;
        }

        if (snapshot.weaponGenerationKey != g_weaponGeneration) {
            clear(ownerToken);
            g_bodyId = kInvalidBodyId;
            if (!selectBody(snapshot)) {
                return;
            }
            char message[160]{};
            std::snprintf(
                message,
                sizeof(message),
                "Focused collider body=%08X partKind=%u",
                g_bodyId,
                g_partKind);
            rock::sdk::example::logInfo(message);
        }

        RockProviderColliderVisualizationRequestV1 request{};
        request.weaponGenerationKey = g_weaponGeneration;
        request.bodyId = g_bodyId;
        request.partKind = g_partKind;
        request.leaseFrames = 2;
        request.worldGeneration = snapshot.worldGeneration;
        request.skeletonGeneration = snapshot.skeletonGeneration;
        request.providerGeneration = snapshot.providerGeneration;
        g_published =
            RockProviderApi::inst->setColliderVisualizationOverrideV1(
                ownerToken,
                &request) == RockProviderResultV1::Ok;
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKColliderFocus",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::WeaponPartObservability) |
                capability(provider::RockProviderConsumerCapabilityV1::ColliderVisualizationOverride),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_COLLIDER_VISUALIZATION_OVERRIDE_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

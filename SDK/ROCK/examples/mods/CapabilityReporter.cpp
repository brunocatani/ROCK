#include "ExampleRuntime.h"

#include <cstdio>

namespace
{
    using namespace rock::provider;

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        RockProviderLimitsV1 base{};
        RockProviderLimitsExtV1 extended{};
        if (!RockProviderApi::inst->getProviderLimitsV1(&base) ||
            !RockProviderApi::inst->getProviderLimitsExtV1(&extended)) {
            rock::sdk::example::logWarning("Provider limits are unavailable");
            return false;
        }

        const auto frameBytes = RockProviderApi::inst->getPublicStructureSizeV1(
            RockProviderStructureIdV1::FrameSnapshot);
        const auto tableBytes = RockProviderApi::inst->getPublicStructureSizeV1(
            RockProviderStructureIdV1::ApiFunctionTable);

        char message[320]{};
        std::snprintf(
            message,
            sizeof(message),
            "API bytes=%u table bytes=%u frame bytes=%u consumers=%u callbacks=%u "
            "partDrives=%u touchTargets=%u raycastsPerFrame=%u featureBits=%08X:%08X",
            base.providerApiByteSize,
            tableBytes,
            frameBytes,
            extended.maxConsumers,
            extended.maxFrameCallbacks,
            extended.maxWeaponPartDrives,
            extended.maxTouchGrabTargets,
            extended.maxWorldRaycastsPerOwnerPerFrame,
            extended.featureBits2,
            extended.featureBits);
        rock::sdk::example::logInfo(message);
        return true;
    }

    void stop(const std::uint64_t) noexcept {}

    void frame(
        const std::uint64_t,
        const RockProviderFrameSnapshot&) noexcept
    {}
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKCapabilityReporter",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_EXTENDED_LIMITS_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

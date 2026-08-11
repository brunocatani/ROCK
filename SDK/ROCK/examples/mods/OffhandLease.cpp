#include "ExampleRuntime.h"

#include <cstdio>

namespace
{
    using namespace rock::provider;

    // Replace this constant with a real activity predicate in your mod.
    bool kEnableReservation = false;

    bool g_leaseActive{ false };
    std::uint64_t g_lastExpiry{ ~std::uint64_t{ 0 } };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        g_leaseActive = false;
        g_lastExpiry = ~std::uint64_t{ 0 };
        if (!kEnableReservation) {
            rock::sdk::example::logInfo(
                "Offhand reservation is opt-in; bind it to your activity state before enabling");
        }
        return true;
    }

    void release(const std::uint64_t ownerToken) noexcept
    {
        if (g_leaseActive) {
            (void)RockProviderApi::inst->releaseOffhandReservationV1(ownerToken);
            g_leaseActive = false;
        }
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        release(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableReservation ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            release(ownerToken);
            return;
        }

        RockProviderOffhandReservationRequestV1 request{};
        request.reservation = RockProviderOffhandReservation::ReloadReserved;
        request.leaseFrames = 2;
        request.worldGeneration = snapshot.worldGeneration;
        request.skeletonGeneration = snapshot.skeletonGeneration;
        request.providerGeneration = snapshot.providerGeneration;
        const auto result = g_leaseActive ?
            RockProviderApi::inst->renewOffhandReservationV1(ownerToken, &request) :
            RockProviderApi::inst->acquireOffhandReservationV1(ownerToken, &request);
        g_leaseActive = result == RockProviderResultV1::Ok;
        if (!g_leaseActive) {
            return;
        }

        RockProviderOffhandReservationStateV1 state{};
        if (RockProviderApi::inst->getOffhandReservationStateV1(
                ownerToken,
                &state) == RockProviderResultV1::Ok &&
            state.expiresAfterFrame != g_lastExpiry) {
            g_lastExpiry = state.expiresAfterFrame;
            char message[160]{};
            std::snprintf(
                message,
                sizeof(message),
                "Offhand reservation=%u active=%u remaining=%u",
                static_cast<std::uint32_t>(state.reservation),
                state.active,
                state.remainingFrames);
            rock::sdk::example::logInfo(message);
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKOffhandLease",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::OffhandReservation),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_OFFHAND_RESERVATION_LEASES_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

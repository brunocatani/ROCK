#include "ROCKProviderApi.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    std::uint64_t g_ownerToken = 0;
    std::uint64_t g_frameCallbackToken = 0;

    void ROCK_PROVIDER_CALL onRockFrame(const RockProviderFrameSnapshot* snapshot, void*)
    {
        if (!snapshot || !hasLifecycleFlag(snapshot->lifecycleFlags, RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            return;
        }

        std::array<RockProviderExternalContactV1, 16> contacts{};
        const std::uint32_t count = RockProviderApi::inst->getExternalContactSnapshotForOwnerV1(
            g_ownerToken,
            contacts.data(),
            static_cast<std::uint32_t>(contacts.size()));

        for (std::uint32_t i = 0; i < count; ++i) {
            const auto& contact = contacts[i];
            (void)contact;
        }
    }
}

bool StartRockConsumer()
{
    if (RockProviderApi::initialize(ROCK_PROVIDER_API_VERSION) != 0 || !RockProviderApi::inst) {
        return false;
    }

    RockProviderLimitsV1 limits{};
    if (!RockProviderApi::inst->getProviderLimitsV1(&limits) ||
        !hasFeatureBitV1(limits.featureBits, RockProviderFeatureBitV1::ConsumerRegistrationV1)) {
        return false;
    }

    RockProviderConsumerRegistrationV1 registration{};
    std::snprintf(registration.modName, sizeof(registration.modName), "MinimalProviderConsumer");
    registration.requestedCapabilities =
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::FrameSnapshots) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodies) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalContacts);

    RockProviderConsumerHandleV1 handle{};
    if (RockProviderApi::inst->registerConsumerV1(&registration, &handle) != RockProviderResultV1::Ok ||
        handle.ownerToken == 0) {
        return false;
    }

    g_ownerToken = handle.ownerToken;
    g_frameCallbackToken = RockProviderApi::inst->registerFrameCallback(&onRockFrame, nullptr);
    if (g_frameCallbackToken == 0) {
        RockProviderApi::inst->unregisterConsumerV1(g_ownerToken);
        g_ownerToken = 0;
        return false;
    }

    return true;
}

void StopRockConsumer()
{
    if (!RockProviderApi::inst) {
        return;
    }

    if (g_frameCallbackToken != 0) {
        RockProviderApi::inst->unregisterFrameCallback(g_frameCallbackToken);
        g_frameCallbackToken = 0;
    }

    if (g_ownerToken != 0) {
        RockProviderApi::inst->unregisterConsumerV1(g_ownerToken);
        g_ownerToken = 0;
    }
}

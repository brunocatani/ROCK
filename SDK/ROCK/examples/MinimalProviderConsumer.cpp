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

        std::array<RockProviderExternalContactV2, 16> contacts{};
        const std::uint32_t count = RockProviderApi::inst->getExternalContactSnapshotForOwnerV9(
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
    if (RockProviderApi::initialize(9) != 0 || !RockProviderApi::inst) {
        return false;
    }

    RockProviderLimitsV9 limits{};
    if (!RockProviderApi::inst->getProviderLimitsV9(&limits) ||
        !hasFeatureBitV9(limits.featureBits, RockProviderFeatureBitV9::ConsumerRegistrationV9)) {
        return false;
    }

    RockProviderConsumerRegistrationV9 registration{};
    std::snprintf(registration.modName, sizeof(registration.modName), "MinimalProviderConsumer");
    registration.requestedCapabilities =
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV9::FrameSnapshots) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV9::ExternalBodies) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV9::ExternalContacts);

    RockProviderConsumerHandleV9 handle{};
    if (RockProviderApi::inst->registerConsumerV9(&registration, &handle) != RockProviderResultV9::Ok ||
        handle.ownerToken == 0) {
        return false;
    }

    g_ownerToken = handle.ownerToken;
    g_frameCallbackToken = RockProviderApi::inst->registerFrameCallback(&onRockFrame, nullptr);
    if (g_frameCallbackToken == 0) {
        RockProviderApi::inst->unregisterConsumerV9(g_ownerToken);
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
        RockProviderApi::inst->unregisterConsumerV9(g_ownerToken);
        g_ownerToken = 0;
    }
}

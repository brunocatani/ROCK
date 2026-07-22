#include "ROCKProviderApi.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    std::uint64_t g_ownerToken = 0;
    std::uint64_t g_frameCallbackToken = 0;
    std::uint64_t g_contactSequence = 0;

    void ROCK_PROVIDER_CALL onRockFrame(const RockProviderFrameSnapshot* snapshot, void*)
    {
        if (!snapshot || !hasLifecycleFlag(snapshot->lifecycleFlags, RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            return;
        }

        std::array<RockProviderExternalContactRecordV1, 16> contacts{};
        RockProviderExternalContactStreamStateV1 streamState{};
        const auto result = RockProviderApi::inst->copyExternalContactsSinceV1(
            g_ownerToken,
            0,
            g_contactSequence,
            contacts.data(),
            static_cast<std::uint32_t>(contacts.size()),
            &streamState);
        if (result != RockProviderResultV1::Ok) {
            return;
        }

        for (std::uint32_t i = 0; i < streamState.copiedCount; ++i) {
            const auto& contact = contacts[i];
            (void)contact;
        }
        if (streamState.copiedCount != 0) {
            g_contactSequence = streamState.lastCopiedSequence;
        }
    }
}

bool StartRockConsumer()
{
    if (RockProviderApi::initialize(
            ROCK_PROVIDER_API_VERSION,
            ROCK_PROVIDER_API_V1_EXTERNAL_BODY_SCOPES_TABLE_BYTES) != 0 ||
        !RockProviderApi::inst) {
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
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalContacts) |
        static_cast<std::uint32_t>(RockProviderConsumerCapabilityV1::ExternalBodyScopes);

    RockProviderConsumerHandleV1 handle{};
    const auto registrationResult =
        RockProviderApi::inst->registerConsumerV1(&registration, &handle);
    if (registrationResult != RockProviderResultV1::Ok ||
        handle.ownerToken == 0 ||
        (handle.grantedCapabilities & registration.requestedCapabilities) !=
            registration.requestedCapabilities) {
        if (handle.ownerToken != 0) {
            (void)RockProviderApi::inst->unregisterConsumerV1(
                handle.ownerToken);
        }
        return false;
    }

    g_ownerToken = handle.ownerToken;
    const auto callbackResult =
        RockProviderApi::inst->registerFrameCallbackForOwnerV1(
            g_ownerToken,
            &onRockFrame,
            nullptr,
            &g_frameCallbackToken);
    if (callbackResult != RockProviderResultV1::Ok ||
        g_frameCallbackToken == 0) {
        (void)RockProviderApi::inst->unregisterConsumerV1(g_ownerToken);
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

    if (g_ownerToken != 0 && g_frameCallbackToken != 0) {
        (void)RockProviderApi::inst->unregisterFrameCallbackForOwnerV1(
            g_ownerToken,
            g_frameCallbackToken);
        g_frameCallbackToken = 0;
    }

    if (g_ownerToken != 0) {
        RockProviderApi::inst->unregisterConsumerV1(g_ownerToken);
        g_ownerToken = 0;
    }
    g_contactSequence = 0;
}

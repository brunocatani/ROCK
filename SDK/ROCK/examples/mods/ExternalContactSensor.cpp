#include "ExampleRuntime.h"

#include <array>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    // Supply a live external Havok body's current ID and generation from your
    // own runtime. The SDK example deliberately has no engine-specific resolver.
    bool kEnableSensor = false;
    constexpr std::uint32_t kExternalBodyId = 0x7FFF'FFFF;
    constexpr std::uint32_t kExternalBodyGeneration = 1;
    constexpr std::uint64_t kScopeToken = 0x434F4E5441435401ull;

    std::uint64_t g_cursor{ 0 };
    bool g_registered{ false };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    void clear(const std::uint64_t ownerToken) noexcept
    {
        if (g_registered) {
            (void)RockProviderApi::inst->clearExternalBodiesForScopeV1(
                ownerToken,
                kScopeToken);
            g_registered = false;
        }
        g_cursor = 0;
    }

    bool start(const std::uint64_t) noexcept
    {
        g_cursor = 0;
        g_registered = false;
        if (!kEnableSensor) {
            rock::sdk::example::logInfo(
                "External contact sensor is opt-in; connect it to your live body resolver");
        }
        return true;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableSensor || kExternalBodyId == 0x7FFF'FFFF ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::WorldAvailable)) {
            clear(ownerToken);
            return;
        }

        if (!g_registered) {
            RockProviderExternalBodyRegistration body{};
            body.bodyId = kExternalBodyId;
            body.ownerToken = ownerToken;
            body.generation = kExternalBodyGeneration;
            body.role = RockProviderExternalBodyRole::ActorRagdollBone;
            body.contactPolicy =
                RockProviderExternalBodyContactPolicy::ReportAllSourceKinds;
            g_registered =
                RockProviderApi::inst->registerExternalBodiesForScopeV1(
                    ownerToken,
                    kScopeToken,
                    &body,
                    1) == RockProviderResultV1::Ok;
            if (!g_registered) {
                return;
            }
        }

        std::array<RockProviderExternalContactRecordV1, 16> contacts{};
        RockProviderExternalContactStreamStateV1 stream{};
        if (RockProviderApi::inst->copyExternalContactsSinceV1(
                ownerToken,
                kScopeToken,
                g_cursor,
                contacts.data(),
                static_cast<std::uint32_t>(contacts.size()),
                &stream) != RockProviderResultV1::Ok) {
            return;
        }

        for (std::uint32_t index = 0; index < stream.copiedCount; ++index) {
            const auto& contact = contacts[index];
            char message[224]{};
            std::snprintf(
                message,
                sizeof(message),
                "External contact sequence=%llu source=%u hand=%u quality=%u weight=%.3f",
                static_cast<unsigned long long>(contact.sequence),
                static_cast<std::uint32_t>(contact.sourceKind),
                static_cast<std::uint32_t>(contact.sourceHand),
                static_cast<std::uint32_t>(contact.quality),
                contact.contactPointWeightSum);
            rock::sdk::example::logInfo(message);
        }
        if (stream.copiedCount != 0) {
            g_cursor = stream.lastCopiedSequence;
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKExternalContactSensor",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::ExternalBodies) |
                capability(provider::RockProviderConsumerCapabilityV1::ExternalContacts) |
                capability(provider::RockProviderConsumerCapabilityV1::ExternalBodyScopes),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_EXTERNAL_BODY_SCOPES_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

#include "ExampleRuntime.h"

#include <array>
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
        return true;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        (void)RockProviderApi::inst->clearDebugOverlayV1(ownerToken);
    }

    void appendContactLines(
        const std::uint64_t ownerToken,
        const RockProviderHand hand,
        std::array<RockProviderDebugOverlayLineV1, 40>& lines,
        std::uint32_t& lineCount) noexcept
    {
        std::array<RockProviderSemanticHandContactV1, 20> contacts{};
        std::uint32_t contactCount{ 0 };
        if (RockProviderApi::inst->copySemanticHandContactsV1(
                ownerToken,
                hand,
                0,
                contacts.data(),
                static_cast<std::uint32_t>(contacts.size()),
                &contactCount) != RockProviderResultV1::Ok) {
            return;
        }

        for (std::uint32_t index = 0;
             index < contactCount && lineCount < lines.size();
             ++index) {
            const auto& contact = contacts[index];
            auto& line = lines[lineCount++];
            line.startGame[0] = contact.contactPointGame.x;
            line.startGame[1] = contact.contactPointGame.y;
            line.startGame[2] = contact.contactPointGame.z;
            line.endGame[0] = contact.contactPointGame.x + contact.contactNormalGame.x * 4.0f;
            line.endGame[1] = contact.contactPointGame.y + contact.contactNormalGame.y * 4.0f;
            line.endGame[2] = contact.contactPointGame.z + contact.contactNormalGame.z * 4.0f;
            if (hand == RockProviderHand::Right) {
                line.color[0] = 0.2f;
                line.color[1] = 1.0f;
                line.color[2] = 0.35f;
            } else {
                line.color[0] = 1.0f;
                line.color[1] = 0.55f;
                line.color[2] = 0.15f;
            }
            line.color[3] = 1.0f;
        }
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::VisualWriteAllowed)) {
            (void)RockProviderApi::inst->clearDebugOverlayV1(ownerToken);
            return;
        }

        std::array<RockProviderDebugOverlayLineV1, 40> lines{};
        std::uint32_t lineCount{ 0 };
        appendContactLines(ownerToken, RockProviderHand::Right, lines, lineCount);
        appendContactLines(ownerToken, RockProviderHand::Left, lines, lineCount);

        std::array<RockProviderPlayerColliderDescriptorV1, 96> colliders{};
        std::uint32_t colliderCount{ 0 };
        (void)RockProviderApi::inst->copyPlayerColliderDescriptorsV1(
            ownerToken,
            colliders.data(),
            static_cast<std::uint32_t>(colliders.size()),
            &colliderCount);

        RockProviderDebugOverlayTextV1 text{};
        std::snprintf(
            text.text,
            sizeof(text.text),
            "Semantic contacts: %u | player colliders: %u",
            lineCount,
            colliderCount);
        text.x = 18.0f;
        text.y = 18.0f;
        text.textSize = 1.6f;

        RockProviderDebugOverlayPublicationV1 publication{};
        publication.lineCount = lineCount;
        publication.textCount = 1;
        publication.lines = lines.data();
        publication.textEntries = &text;
        publication.worldGeneration = snapshot.worldGeneration;
        publication.skeletonGeneration = snapshot.skeletonGeneration;
        publication.providerGeneration = snapshot.providerGeneration;
        publication.leaseFrames = 2;
        (void)RockProviderApi::inst->publishDebugOverlayV1(
            ownerToken,
            &publication);
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKContactVisualizer",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::SemanticHandContacts) |
                capability(provider::RockProviderConsumerCapabilityV1::PlayerColliderDescriptors) |
                capability(provider::RockProviderConsumerCapabilityV1::DebugOverlayPublication),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_PLAYER_COLLIDERS_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

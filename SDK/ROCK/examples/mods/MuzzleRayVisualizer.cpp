#include "ExampleRuntime.h"

#include <array>

namespace
{
    using namespace rock::provider;

    constexpr float kMaximumDistanceGame = 4096.0f;

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        return true;
    }

    void clear(const std::uint64_t ownerToken) noexcept
    {
        (void)RockProviderApi::inst->clearDebugOverlayV1(ownerToken);
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::WorldAvailable) ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::VisualWriteAllowed)) {
            clear(ownerToken);
            return;
        }

        RockProviderEquippedWeaponGripStateV1 weapon{};
        if (!RockProviderApi::inst->getEquippedWeaponGripStateV1(
                ownerToken,
                &weapon) ||
            weapon.weaponGenerationKey == 0) {
            clear(ownerToken);
            return;
        }

        RockProviderWorldRaycastRequestV1 request{};
        request.startGame = weapon.muzzleOriginGame;
        request.directionGame = weapon.muzzleDirectionGame;
        request.maxDistanceGame = kMaximumDistanceGame;
        request.worldGeneration = snapshot.worldGeneration;
        request.skeletonGeneration = snapshot.skeletonGeneration;
        request.providerGeneration = snapshot.providerGeneration;

        RockProviderWorldRaycastResultV1 result{};
        if (RockProviderApi::inst->queryWorldRaycastV1(
                ownerToken,
                &request,
                &result) != RockProviderResultV1::Ok) {
            clear(ownerToken);
            return;
        }

        std::array<RockProviderDebugOverlayLineV1, 2> lines{};
        auto& ray = lines[0];
        ray.startGame[0] = request.startGame.x;
        ray.startGame[1] = request.startGame.y;
        ray.startGame[2] = request.startGame.z;
        ray.endGame[0] = result.hitPointGame.x;
        ray.endGame[1] = result.hitPointGame.y;
        ray.endGame[2] = result.hitPointGame.z;
        ray.color[0] = result.hit != 0 ? 1.0f : 0.2f;
        ray.color[1] = result.hit != 0 ? 0.2f : 0.8f;
        ray.color[2] = 0.1f;
        ray.color[3] = 1.0f;

        std::uint32_t lineCount = 1;
        if (result.hit != 0) {
            auto& normal = lines[lineCount++];
            normal.startGame[0] = result.hitPointGame.x;
            normal.startGame[1] = result.hitPointGame.y;
            normal.startGame[2] = result.hitPointGame.z;
            normal.endGame[0] = result.hitPointGame.x + result.hitNormalGame.x * 6.0f;
            normal.endGame[1] = result.hitPointGame.y + result.hitNormalGame.y * 6.0f;
            normal.endGame[2] = result.hitPointGame.z + result.hitNormalGame.z * 6.0f;
            normal.color[0] = 0.2f;
            normal.color[1] = 0.7f;
            normal.color[2] = 1.0f;
            normal.color[3] = 1.0f;
        }

        RockProviderDebugOverlayPublicationV1 publication{};
        publication.lineCount = lineCount;
        publication.lines = lines.data();
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
            .pluginName = "ROCKSDKMuzzleRayVisualizer",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::EquippedWeaponGripState) |
                capability(provider::RockProviderConsumerCapabilityV1::WorldRaycasts) |
                capability(provider::RockProviderConsumerCapabilityV1::DebugOverlayPublication),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_WORLD_RAYCASTS_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

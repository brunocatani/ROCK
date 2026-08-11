#include "ExampleRuntime.h"

#include <cstdio>

namespace
{
    using namespace rock::provider;

    // Ownership and ambidextrous handoff alter equipped-weapon behavior. Keep
    // the example inert until your mod has an explicit activation policy.
    bool kEnableLeftHandPolicy = false;

    std::uint64_t g_requestedGeneration{ 0 };
    bool g_authorityPublished{ false };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    constexpr std::uint32_t handlingFlag(
        const RockProviderEquippedWeaponHandlingFlagV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    bool start(const std::uint64_t) noexcept
    {
        g_requestedGeneration = 0;
        g_authorityPublished = false;
        if (!kEnableLeftHandPolicy) {
            rock::sdk::example::logInfo(
                "Exact equipped-hand policy is opt-in; add a user-facing activation policy first");
        }
        return true;
    }

    void clear(const std::uint64_t ownerToken) noexcept
    {
        if (g_authorityPublished) {
            (void)RockProviderApi::inst->clearEquippedWeaponHandlingAuthorityV1(
                ownerToken);
            g_authorityPublished = false;
        }
        g_requestedGeneration = 0;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clear(ownerToken);
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableLeftHandPolicy || snapshot.weaponGenerationKey == 0 ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            clear(ownerToken);
            return;
        }

        RockProviderEquippedWeaponHandlingRequestV1 authority{};
        authority.flags =
            handlingFlag(RockProviderEquippedWeaponHandlingFlagV1::FiringGripOwnership) |
            handlingFlag(RockProviderEquippedWeaponHandlingFlagV1::AmbidextrousHandoff);
        authority.leaseFrames = 2;
        authority.worldGeneration = snapshot.worldGeneration;
        authority.skeletonGeneration = snapshot.skeletonGeneration;
        authority.providerGeneration = snapshot.providerGeneration;
        g_authorityPublished =
            RockProviderApi::inst->setEquippedWeaponHandlingAuthorityV1(
                ownerToken,
                &authority) == RockProviderResultV1::Ok;
        if (!g_authorityPublished ||
            g_requestedGeneration == snapshot.weaponGenerationKey) {
            return;
        }

        RockProviderEquippedWeaponHandRequestV1 request{};
        request.hand = RockProviderHand::Left;
        request.weaponFormId = snapshot.weaponFormId;
        request.weaponGenerationKey = snapshot.weaponGenerationKey;
        request.worldGeneration = snapshot.worldGeneration;
        request.skeletonGeneration = snapshot.skeletonGeneration;
        request.providerGeneration = snapshot.providerGeneration;
        const auto requestResult =
            RockProviderApi::inst->requestEquippedWeaponHandV1(
                ownerToken,
                &request);
        if (requestResult == RockProviderResultV1::Ok ||
            requestResult == RockProviderResultV1::RequestQueued) {
            g_requestedGeneration = snapshot.weaponGenerationKey;
        }

        RockProviderEquippedWeaponHandlingStateV1 state{};
        if (RockProviderApi::inst->getEquippedWeaponHandlingStateV1(&state)) {
            char message[192]{};
            std::snprintf(
                message,
                sizeof(message),
                "Hand request result=%u current=%u fixed=%u runtimeFlags=%08X",
                static_cast<std::uint32_t>(requestResult),
                static_cast<std::uint32_t>(state.currentFiringHand),
                static_cast<std::uint32_t>(state.fixedFiringHand),
                state.runtimeFlags);
            rock::sdk::example::logInfo(message);
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKEquippedHandPolicy",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::EquippedWeaponHandlingAuthority),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_EQUIPPED_WEAPON_HAND_REQUEST_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

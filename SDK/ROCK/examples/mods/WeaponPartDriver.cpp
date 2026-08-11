#include "ExampleRuntime.h"

#include <array>
#include <cmath>
#include <cstdio>

namespace
{
    using namespace rock::provider;

    // Keep false in distributed examples. Set true only after choosing which
    // semantic part your mod owns and replacing the demonstration motion.
    bool kEnableDemoMotion = false;
    constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    constexpr std::uint32_t kGroupId = 1;
    constexpr std::uint32_t kPriority = 100;
    constexpr std::uint32_t kDriveLeaseFrames = 2;

    std::uint64_t g_weaponGeneration{ 0 };
    std::uint32_t g_bodyId{ kInvalidBodyId };
    bool g_targetPublished{ false };
    bool g_drivePublished{ false };

    constexpr std::uint32_t capability(
        const RockProviderConsumerCapabilityV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    constexpr std::uint32_t targetFlag(
        const RockProviderWeaponPartTargetFlagV1 value) noexcept
    {
        return static_cast<std::uint32_t>(value);
    }

    void clearDrive(const std::uint64_t ownerToken) noexcept
    {
        if (g_drivePublished) {
            (void)RockProviderApi::inst->clearWeaponPartDriveTargetsV1(ownerToken);
            g_drivePublished = false;
        }
    }

    void clearAll(const std::uint64_t ownerToken) noexcept
    {
        clearDrive(ownerToken);
        if (g_targetPublished) {
            (void)RockProviderApi::inst->clearWeaponPartTargetsV1(ownerToken);
            g_targetPublished = false;
        }
        g_weaponGeneration = 0;
        g_bodyId = kInvalidBodyId;
    }

    bool start(const std::uint64_t) noexcept
    {
        g_weaponGeneration = 0;
        g_bodyId = kInvalidBodyId;
        g_targetPublished = false;
        g_drivePublished = false;
        if (!kEnableDemoMotion) {
            rock::sdk::example::logInfo(
                "Weapon-part drive demo is opt-in; edit kEnableDemoMotion after selecting a part policy");
        }
        return true;
    }

    void stop(const std::uint64_t ownerToken) noexcept
    {
        clearAll(ownerToken);
    }

    bool selectAndPublishTarget(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        std::array<RockProviderWeaponPartPoseV1,
            ROCK_PROVIDER_MAX_WEAPON_PART_POSES_V1> poses{};
        std::uint32_t poseCount{ 0 };
        if (RockProviderApi::inst->copyWeaponPartPoseSnapshotV1(
                ownerToken,
                poses.data(),
                static_cast<std::uint32_t>(poses.size()),
                &poseCount) != RockProviderResultV1::Ok) {
            return false;
        }

        const RockProviderWeaponPartPoseV1* selected{ nullptr };
        for (std::uint32_t index = 0; index < poseCount; ++index) {
            if (poses[index].weaponGenerationKey == snapshot.weaponGenerationKey &&
                poses[index].bodyId != kInvalidBodyId) {
                selected = &poses[index];
                break;
            }
        }
        if (!selected) {
            return false;
        }

        RockProviderWeaponPartTargetV1 target{};
        target.flags =
            targetFlag(RockProviderWeaponPartTargetFlagV1::MatchBodyId) |
            targetFlag(RockProviderWeaponPartTargetFlagV1::NonExclusive);
        target.grabMode = RockProviderWeaponPartGrabModeV1::AttachOnly;
        target.weaponGenerationKey = snapshot.weaponGenerationKey;
        target.bodyId = selected->bodyId;
        target.groupId = kGroupId;
        target.priority = kPriority;

        if (RockProviderApi::inst->setWeaponPartTargetsV1(
                ownerToken,
                &target,
                1) != RockProviderResultV1::Ok) {
            return false;
        }

        g_weaponGeneration = snapshot.weaponGenerationKey;
        g_bodyId = selected->bodyId;
        g_targetPublished = true;
        char message[192]{};
        std::snprintf(
            message,
            sizeof(message),
            "Selected demonstration part body=%08X generation=%llu",
            g_bodyId,
            static_cast<unsigned long long>(g_weaponGeneration));
        rock::sdk::example::logInfo(message);
        return true;
    }

    bool partIsHeld() noexcept
    {
        for (const auto hand : { RockProviderHand::Right, RockProviderHand::Left }) {
            RockProviderWeaponPartGripStateV1 grip{};
            if (RockProviderApi::inst->getWeaponPartGripStateV1(hand, &grip) &&
                grip.active != 0 && grip.weaponGenerationKey == g_weaponGeneration &&
                grip.bodyId == g_bodyId) {
                return true;
            }
        }
        return false;
    }

    void frame(
        const std::uint64_t ownerToken,
        const RockProviderFrameSnapshot& snapshot) noexcept
    {
        if (!kEnableDemoMotion ||
            !hasLifecycleFlag(
                snapshot.lifecycleFlags,
                RockProviderLifecycleFlag::PhysicsWriteAllowed)) {
            clearDrive(ownerToken);
            return;
        }

        if (snapshot.weaponGenerationKey == 0) {
            clearAll(ownerToken);
            return;
        }
        if (snapshot.weaponGenerationKey != g_weaponGeneration) {
            clearAll(ownerToken);
            if (!selectAndPublishTarget(ownerToken, snapshot)) {
                return;
            }
        }
        if (!g_targetPublished || !partIsHeld()) {
            clearDrive(ownerToken);
            return;
        }

        std::array<RockProviderWeaponPartPoseV1,
            ROCK_PROVIDER_MAX_WEAPON_PART_POSES_V1> poses{};
        std::uint32_t poseCount{ 0 };
        if (RockProviderApi::inst->copyWeaponPartPoseSnapshotV1(
                ownerToken,
                poses.data(),
                static_cast<std::uint32_t>(poses.size()),
                &poseCount) != RockProviderResultV1::Ok) {
            clearDrive(ownerToken);
            return;
        }

        const RockProviderWeaponPartPoseV1* current{ nullptr };
        for (std::uint32_t index = 0; index < poseCount; ++index) {
            if (poses[index].weaponGenerationKey == g_weaponGeneration &&
                poses[index].bodyId == g_bodyId) {
                current = &poses[index];
                break;
            }
        }
        if (!current) {
            clearDrive(ownerToken);
            return;
        }

        RockProviderWeaponPartDriveTargetV1 drive{};
        drive.flags = targetFlag(RockProviderWeaponPartTargetFlagV1::MatchBodyId);
        drive.driveSpace = RockProviderWeaponPartDriveSpaceV1::WeaponRootLocal;
        drive.weaponGenerationKey = g_weaponGeneration;
        drive.bodyId = g_bodyId;
        drive.groupId = kGroupId;
        drive.priority = kPriority;
        drive.leaseFrames = kDriveLeaseFrames;
        drive.targetTransform = current->weaponRootLocal;
        drive.targetTransform.translate[0] +=
            std::sin(static_cast<float>(snapshot.frameIndex) * 0.03f) * 2.0f;

        g_drivePublished =
            RockProviderApi::inst->setWeaponPartDriveTargetsV1(
                ownerToken,
                &drive,
                1) == RockProviderResultV1::Ok;

        if (g_drivePublished && snapshot.frameIndex % 120 == 0) {
            std::array<RockProviderWeaponPartDriveApplicationResultV1,
                ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_RESULTS_V1> results{};
            std::uint32_t resultCount{ 0 };
            if (RockProviderApi::inst->copyWeaponPartDriveApplicationResultsV1(
                    ownerToken,
                    results.data(),
                    static_cast<std::uint32_t>(results.size()),
                    &resultCount) == RockProviderResultV1::Ok &&
                resultCount != 0) {
                char message[160]{};
                std::snprintf(
                    message,
                    sizeof(message),
                    "Drive body=%08X result=%u",
                    results[0].bodyId,
                    static_cast<std::uint32_t>(results[0].result));
                rock::sdk::example::logInfo(message);
            }
        }
    }
}

namespace rock::sdk::example
{
    const Definition& definition() noexcept
    {
        static const Definition value{
            .pluginName = "ROCKSDKWeaponPartDriver",
            .pluginVersion = 1,
            .requestedCapabilities =
                capability(provider::RockProviderConsumerCapabilityV1::FrameSnapshots) |
                capability(provider::RockProviderConsumerCapabilityV1::WeaponPartInteraction) |
                capability(provider::RockProviderConsumerCapabilityV1::WeaponPartObservability),
            .minimumTableBytes =
                provider::ROCK_PROVIDER_API_V1_WEAPON_PART_OBSERVABILITY_TABLE_BYTES,
            .onStart = &start,
            .onStop = &stop,
            .onFrame = &frame,
        };
        return value;
    }
}

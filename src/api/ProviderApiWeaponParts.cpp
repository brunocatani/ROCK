#define ROCK_API_EXPORTS
// Build the DLL side of the public provider ABI.
#include "api/detail/ProviderWeaponParts.h"

#include "api/detail/ProviderApiCore.h"
#include "api/detail/ProviderApiEntryPoints.h"
#include "api/detail/ProviderTransformMath.h"
#include "api/detail/ProviderWeaponPartValidation.h"

#include <array>
#include <cstring>
#include <mutex>
#include <string_view>

#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/weapon/parts/WeaponPartRuntime.h"

namespace rock::provider::detail
{
    using namespace rock;

    bool ROCK_PROVIDER_CALL apiGetWeaponPartGripStateV1(RockProviderHand hand, RockProviderWeaponPartGripStateV1* outState)
    {
        if (!outState || outState->size != sizeof(RockProviderWeaponPartGripStateV1)) {
            return false;
        }
        if (hand != RockProviderHand::Right && hand != RockProviderHand::Left) {
            return false;
        }

        std::scoped_lock lock(s_snapshotMutex);
        if (!s_hasSnapshot || s_lastSnapshot.providerReady == 0) {
            return false;
        }
        *outState = s_lastPartGripStates[hand == RockProviderHand::Left ? 1u : 0u];
        return true;
    }


    void clearWeaponPartTargetsForOwnerLocked(std::uint64_t ownerToken)
    {
        for (auto& slot : s_weaponPartTargets) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void clearWeaponPartDrivesForOwnerLocked(std::uint64_t ownerToken)
    {
        for (auto& slot : s_weaponPartDrives) {
            if (slot.active && slot.ownerToken == ownerToken) {
                slot = {};
            }
        }
    }

    void pruneExpiredWeaponPartDrivesLocked(std::uint64_t frameIndex)
    {
        (void)pruneExpiredSlots(
            s_weaponPartDrives.size(),
            frameIndex,
            [](const std::size_t index) {
                return s_weaponPartDrives[index].active;
            },
            [](const std::size_t) { return false; },
            [](const std::size_t index) {
                return s_weaponPartDrives[index].expiresAfterFrame;
            },
            [](const std::size_t index, const auto reason) {
                auto& slot = s_weaponPartDrives[index];
                publishAuthorityLostEvent(
                    slot.ownerToken,
                    RockProviderAuthorityKindV1::WeaponPartDrive,
                    static_cast<std::uint32_t>(reason));
                slot = {};
            });
    }

    std::size_t availableWeaponPartTargetSlotsForOwnerLocked(std::uint64_t ownerToken)
    {
        std::size_t available = 0;
        for (const auto& slot : s_weaponPartTargets) {
            if (!slot.active || slot.ownerToken == ownerToken) {
                ++available;
            }
        }
        return available;
    }

    std::size_t availableWeaponPartDriveSlotsForOwnerLocked(std::uint64_t ownerToken)
    {
        std::size_t available = 0;
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active || slot.ownerToken == ownerToken) {
                ++available;
            }
        }
        return available;
    }


    RockProviderResultV1 ROCK_PROVIDER_CALL apiQueryWeaponPartTargetResolutionV1(
        const std::uint64_t ownerToken,
        const RockProviderWeaponPartResolutionQueryV1* query,
        RockProviderWeaponPartResolutionResultV1* outResolution)
    {
        auto entryResult = validateOutputEntry(ownerToken, query);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        entryResult = validateOutputEntry(ownerToken, outResolution);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponPartObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        RockProviderWeaponPartTargetQueryV1 internal{};
        internal.weaponGenerationKey = query->weaponGenerationKey;
        internal.bodyId = query->bodyId;
        internal.partKind = query->partKind;
        internal.reloadRole = query->reloadRole;
        internal.supportRole = query->supportRole;
        internal.socketRole = query->socketRole;
        internal.actionRole = query->actionRole;
        internal.sourceRoot = query->sourceRoot;
        std::memcpy(internal.sourceName, query->sourceName,
            sizeof(internal.sourceName));
        internal.sourceName[sizeof(internal.sourceName) - 1] = '\0';
        RockProviderWeaponPartTargetResolutionV1 resolution{};
        (void)resolveWeaponPartTargetV1(internal, resolution);

        *outResolution = {};
        outResolution->whitelistActive = resolution.whitelistActive;
        outResolution->matched = resolution.matched;
        outResolution->grabMode = resolution.grabMode;
        outResolution->groupId = resolution.groupId;
        outResolution->priority = resolution.priority;
        outResolution->winningOwnerToken = resolution.ownerToken;
        outResolution->weaponGenerationKey = query->weaponGenerationKey;
        outResolution->frameIndex = currentProviderFrameIndex();
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponPartPoseSnapshotV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponPartPoseV1* outParts,
        const std::uint32_t maxParts,
        std::uint32_t* outPartCount)
    {
        const auto entryResult = validateArrayEntry(
            ownerToken,
            outParts,
            maxParts,
            outPartCount,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        *outPartCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponPartObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outPartCount = pi->copyProviderWeaponPartPosesV1(
            outParts,
            maxParts);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiCopyWeaponPartDriveApplicationResultsV1(
        const std::uint64_t ownerToken,
        RockProviderWeaponPartDriveApplicationResultV1* outResults,
        const std::uint32_t maxResults,
        std::uint32_t* outResultCount)
    {
        const auto entryResult = validateArrayEntry(
            ownerToken,
            outResults,
            maxResults,
            outResultCount,
            EntryThreadPolicy::AnimationOwner);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }
        *outResultCount = 0;
        const auto ownerResult = validateReadCapability(
            ownerToken,
            RockProviderConsumerCapabilityV1::WeaponPartObservability);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        auto* pi = s_physicsInteraction.load(std::memory_order_acquire);
        if (!pi || !pi->isInitialized()) {
            return RockProviderResultV1::NotReady;
        }
        *outResultCount = pi->copyProviderWeaponPartDriveResultsV1(
            ownerToken,
            outResults,
            maxResults);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1) {
            return RockProviderResultV1::CapacityFull;
        }
        if (targetCount > 0 && !targets) {
            return RockProviderResultV1::InvalidArgument;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            const auto entryResult = validateEntry(&target);
            if (entryResult != RockProviderResultV1::Ok) {
                return entryResult;
            }
            if (!isValidWeaponPartGrabMode(target.grabMode) ||
                !hasValidWeaponPartMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName) ||
                !hasValidWeaponPartTargetSemantics(target)) {
                return RockProviderResultV1::InvalidArgument;
            }
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        if (targetCount > availableWeaponPartTargetSlotsForOwnerLocked(ownerToken)) {
            return RockProviderResultV1::CapacityFull;
        }

        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            bool stored = false;
            for (auto& slot : s_weaponPartTargets) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartTargetsForOwnerLocked(ownerToken);
                return RockProviderResultV1::CapacityFull;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartTargetsV1(std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(ownerToken);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearWeaponPartTargetsForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiSetWeaponPartDriveTargetsV1(
        std::uint64_t ownerToken,
        const RockProviderWeaponPartDriveTargetV1* targets,
        std::uint32_t targetCount)
    {
        if (targetCount > ROCK_PROVIDER_MAX_WEAPON_PART_DRIVES_V1) {
            return RockProviderResultV1::CapacityFull;
        }
        if (targetCount > 0 && !targets) {
            return RockProviderResultV1::InvalidArgument;
        }

        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto& target = targets[i];
            const auto entryResult = validateEntry(&target);
            if (entryResult != RockProviderResultV1::Ok) {
                return entryResult;
            }
            if (!isValidWeaponPartDriveSpace(target.driveSpace) ||
                target.leaseFrames == 0 ||
                !finiteProviderTransform(target.targetTransform, 0.0001f) ||
                !hasConcreteWeaponPartDriveMatcher(target.flags, target.bodyId, target.sourceRoot, target.sourceName)) {
                return RockProviderResultV1::InvalidArgument;
            }
        }
        if (ownerToken == 0) {
            return RockProviderResultV1::InvalidArgument;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }

        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        if (targetCount > availableWeaponPartDriveSlotsForOwnerLocked(ownerToken)) {
            return RockProviderResultV1::CapacityFull;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        for (std::uint32_t i = 0; i < targetCount; ++i) {
            const auto leaseFrames = provider_lease_policy::clampLeaseFrames(
                targets[i].leaseFrames,
                ROCK_PROVIDER_MAX_WEAPON_PART_DRIVE_LEASE_FRAMES_V1);
            const auto expiresAfterFrame =
                provider_lease_policy::exclusiveExpiryFrame(
                    frameIndex,
                    leaseFrames);
            bool stored = false;
            for (auto& slot : s_weaponPartDrives) {
                if (!slot.active) {
                    slot.active = true;
                    slot.ownerToken = ownerToken;
                    slot.expiresAfterFrame = expiresAfterFrame;
                    slot.target = targets[i];
                    slot.target.sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
                    stored = true;
                    break;
                }
            }
            if (!stored) {
                clearWeaponPartDrivesForOwnerLocked(ownerToken);
                return RockProviderResultV1::CapacityFull;
            }
        }
        return RockProviderResultV1::Ok;
    }

    RockProviderResultV1 ROCK_PROVIDER_CALL apiClearWeaponPartDriveTargetsV1(std::uint64_t ownerToken)
    {
        const auto entryResult = validateOwnerEntry(ownerToken);
        if (entryResult != RockProviderResultV1::Ok) {
            return entryResult;
        }

        std::scoped_lock lock(s_consumerMutex, s_weaponPartMutex);
        const auto ownerResult = validateRegisteredOwnerCapabilityLocked(ownerToken, RockProviderConsumerCapabilityV1::WeaponPartInteraction);
        if (ownerResult != RockProviderResultV1::Ok) {
            return ownerResult;
        }
        clearWeaponPartDrivesForOwnerLocked(ownerToken);
        return RockProviderResultV1::Ok;
    }
}

namespace rock::provider
{
    using namespace detail;

    bool resolveWeaponPartTargetV1(
        const RockProviderWeaponPartTargetQueryV1& query,
        RockProviderWeaponPartTargetResolutionV1& outResolution)
    {
        outResolution = {};
        std::array<weapon_part_runtime::Target, ROCK_PROVIDER_MAX_WEAPON_PART_TARGETS_V1> runtimeTargets{};
        {
            std::scoped_lock lock(s_weaponPartMutex);
            for (std::size_t i = 0; i < s_weaponPartTargets.size(); ++i) {
                runtimeTargets[i] = toRuntimeTarget(s_weaponPartTargets[i]);
            }
        }

        const weapon_part_runtime::Contact contact{
            .weaponGenerationKey = query.weaponGenerationKey,
            .bodyId = query.bodyId,
            .sourceRoot = query.sourceRoot,
            .sourceName = std::string_view(query.sourceName, boundedStringLength(query.sourceName, ROCK_PROVIDER_MAX_EVIDENCE_NAME)),
            .partKind = static_cast<WeaponPartKind>(query.partKind),
            .reloadRole = static_cast<WeaponReloadRole>(query.reloadRole),
            .supportRole = static_cast<WeaponSupportGripRole>(query.supportRole),
            .socketRole = static_cast<WeaponSocketRole>(query.socketRole),
            .actionRole = static_cast<WeaponActionRole>(query.actionRole),
        };
        const auto resolution = weapon_part_runtime::resolveTarget(runtimeTargets, contact);
        outResolution.whitelistActive = resolution.whitelistActive ? 1u : 0u;
        outResolution.matched = resolution.matched ? 1u : 0u;
        outResolution.grabMode = fromRuntimeGrabMode(resolution.grabMode);
        outResolution.groupId = resolution.groupId;
        outResolution.ownerToken = resolution.ownerToken;
        outResolution.priority = resolution.priority;
        // Non-exclusive targets can match without raising whitelistActive, so
        // a resolution is meaningful whenever either signal is set.
        return resolution.whitelistActive || resolution.matched;
    }

    std::uint32_t copyWeaponPartDriveTargetsV1(
        RockProviderWeaponPartDriveTargetV1* outTargets,
        std::uint32_t maxTargets,
        std::uint64_t* outOwnerTokens)
    {
        if (!outTargets || maxTargets == 0) {
            return 0;
        }

        const auto frameIndex = currentProviderFrameIndex();
        std::uint32_t copied = 0;
        std::scoped_lock lock(s_weaponPartMutex);
        pruneExpiredWeaponPartDrivesLocked(frameIndex);
        for (const auto& slot : s_weaponPartDrives) {
            if (!slot.active) {
                continue;
            }
            if (copied >= maxTargets) {
                break;
            }
            outTargets[copied] = slot.target;
            outTargets[copied].sourceName[ROCK_PROVIDER_MAX_EVIDENCE_NAME - 1] = '\0';
            if (outOwnerTokens) {
                outOwnerTokens[copied] = slot.ownerToken;
            }
            ++copied;
        }
        return copied;
    }

}

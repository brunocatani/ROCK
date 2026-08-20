#define ROCK_API_EXPORTS
/*
 * Build the DLL side of the public provider ABI.
 *
 * FRAME DIFF: pure comparison and sequence assignment over public frame structs.
 *
 * No locks, no globals, no engine access. That is what makes it unit testable, and
 * ROCKProviderFrameDiffTests covers it. Keep new logic here pure so the test stays
 * able to reach it.
 *
 * A sequence advances only when the payload actually changed. Consumers use that to
 * skip work, so a spurious advance costs them a frame of needless recompute.
 */
#include "api/detail/ProviderFrameDiff.h"

namespace rock::provider::detail
{
    [[nodiscard]] std::uint64_t advanceSequence(
        const std::uint64_t sequence) noexcept
    {
        return sequence == UINT64_MAX ? UINT64_MAX : sequence + 1;
    }

    [[nodiscard]] bool sameHeldBodies(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        if (left.heldBodyCount != right.heldBodyCount) {
            return false;
        }
        for (std::uint32_t index = 0; index < left.heldBodyCount; ++index) {
            if (left.heldBodyIds[index] != right.heldBodyIds[index]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool sameHandTarget(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        return left.targetKind == right.targetKind &&
               left.reservedTargetIdentity ==
                   right.reservedTargetIdentity &&
               left.targetFormId == right.targetFormId &&
               left.primaryBodyId == right.primaryBodyId &&
               sameHeldBodies(left, right);
    }

    [[nodiscard]] bool handGripActive(
        const RockProviderHandInteractionStateV1& state) noexcept
    {
        constexpr std::uint32_t gripFlags =
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::FiringGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartCarry);
        return state.phase == RockProviderHandInteractionPhaseV1::Holding ||
               (state.flags & gripFlags) != 0;
    }

    [[nodiscard]] bool sameHandGrip(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        constexpr std::uint32_t gripFlags =
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::FiringGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartGrip) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::PartCarry) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::LooseObject) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::LooseWeapon) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::TouchGrab) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::FixedSurfaceLatch) |
            static_cast<std::uint32_t>(
                RockProviderHandInteractionFlagV1::GlobalSurfaceLatch);
        return handGripActive(left) == handGripActive(right) &&
               (left.flags & gripFlags) == (right.flags & gripFlags) &&
               sameHandTarget(left, right);
    }

    [[nodiscard]] bool sameHandInteractionPayload(
        const RockProviderHandInteractionStateV1& left,
        const RockProviderHandInteractionStateV1& right) noexcept
    {
        return left.hand == right.hand &&
               left.phase == right.phase &&
               left.flags == right.flags &&
               sameHandTarget(left, right) &&
               left.effectiveInputSuppressionFlags ==
                   right.effectiveInputSuppressionFlags &&
               left.collisionAvailabilityFlags ==
                   right.collisionAvailabilityFlags &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration &&
               left.collisionGeneration == right.collisionGeneration;
    }

    void assignHandInteractionSequences(
        RockProviderHandInteractionStateV1& current,
        const RockProviderHandInteractionStateV1& previous,
        const bool hasPrevious)
    {
        if (!hasPrevious) {
            current.stateSequence = 1;
            current.targetSequence =
                sameHandTarget(current, RockProviderHandInteractionStateV1{}) ?
                    0 :
                    1;
            current.gripSequence = handGripActive(current) ? 1 : 0;
            current.releaseSequence = 0;
            return;
        }

        current.stateSequence = sameHandInteractionPayload(current, previous) ?
            previous.stateSequence :
            advanceSequence(previous.stateSequence);
        current.targetSequence = sameHandTarget(current, previous) ?
            previous.targetSequence :
            advanceSequence(previous.targetSequence);
        const bool wasGripActive = handGripActive(previous);
        const bool gripActive = handGripActive(current);
        current.gripSequence = gripActive &&
                (!wasGripActive || !sameHandGrip(current, previous)) ?
            advanceSequence(previous.gripSequence) :
            previous.gripSequence;
        current.releaseSequence = wasGripActive && !gripActive ?
            advanceSequence(previous.releaseSequence) :
            previous.releaseSequence;
    }

    [[nodiscard]] bool sameLifecyclePayload(
        const RockProviderFrameSnapshot& left,
        const RockProviderFrameSnapshot& right) noexcept
    {
        return left.frikSkeletonReady == right.frikSkeletonReady &&
               left.menuBlocking == right.menuBlocking &&
               left.configBlocking == right.configBlocking &&
               left.providerReady == right.providerReady &&
               left.physicsScaleRevision == right.physicsScaleRevision &&
               left.lifecycleFlags == right.lifecycleFlags &&
               left.lastLifecycleReason == right.lastLifecycleReason &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration;
    }

    [[nodiscard]] bool sameWeaponPayload(
        const RockProviderFrameSnapshot& left,
        const RockProviderFrameSnapshot& right) noexcept
    {
        if (left.weaponFormId != right.weaponFormId ||
            left.weaponGenerationKey != right.weaponGenerationKey ||
            left.weaponBodyCount != right.weaponBodyCount) {
            return false;
        }
        for (std::uint32_t index = 0; index < left.weaponBodyCount; ++index) {
            if (left.weaponBodyIds[index] != right.weaponBodyIds[index]) {
                return false;
            }
        }
        return true;
    }

    [[nodiscard]] bool sameEquippedWeaponPayload(
        const RockProviderEquippedWeaponStateV1& left,
        const RockProviderEquippedWeaponStateV1& right) noexcept
    {
        return left.flags == right.flags &&
               left.weaponFormId == right.weaponFormId &&
               left.weaponGenerationKey == right.weaponGenerationKey &&
               left.transitionSequence == right.transitionSequence &&
               left.terminalSequence == right.terminalSequence &&
               left.transitionSource == right.transitionSource &&
               left.terminalResult == right.terminalResult &&
               left.worldGeneration == right.worldGeneration &&
               left.skeletonGeneration == right.skeletonGeneration &&
               left.providerGeneration == right.providerGeneration;
    }
}

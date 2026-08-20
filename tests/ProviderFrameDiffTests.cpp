#include "api/detail/ProviderFrameDiff.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdint>
#include <limits>

namespace
{
    using namespace rock::provider;
    using namespace rock::provider::detail;

    RockProviderHandInteractionStateV1 makeHoldingState(
        const std::uint64_t targetIdentity,
        const std::uint32_t bodyId)
    {
        RockProviderHandInteractionStateV1 state{};
        state.hand = RockProviderHand::Right;
        state.phase = RockProviderHandInteractionPhaseV1::Holding;
        state.flags = static_cast<std::uint32_t>(
            RockProviderHandInteractionFlagV1::TouchGrab);
        state.targetKind = RockProviderBodyContactTargetKind::External;
        state.reservedTargetIdentity = targetIdentity;
        state.primaryBodyId = bodyId;
        state.heldBodyCount = 1;
        state.heldBodyIds[0] = bodyId;
        state.worldGeneration = 4;
        state.skeletonGeneration = 5;
        state.providerGeneration = 6;
        state.collisionGeneration = 7;
        return state;
    }

    void testSequenceAssignment()
    {
        assert(advanceSequence(0) == 1);
        assert(advanceSequence((std::numeric_limits<std::uint64_t>::max)()) ==
               (std::numeric_limits<std::uint64_t>::max)());

        RockProviderHandInteractionStateV1 empty{};
        assignHandInteractionSequences(empty, {}, false);
        assert(empty.stateSequence == 1);
        assert(empty.targetSequence == 0);
        assert(empty.gripSequence == 0);
        assert(empty.releaseSequence == 0);

        auto firstGrip = makeHoldingState(100, 20);
        assignHandInteractionSequences(firstGrip, {}, false);
        assert(firstGrip.stateSequence == 1);
        assert(firstGrip.targetSequence == 1);
        assert(firstGrip.gripSequence == 1);
        assert(firstGrip.releaseSequence == 0);

        auto unchanged = firstGrip;
        assignHandInteractionSequences(unchanged, firstGrip, true);
        assert(unchanged.stateSequence == firstGrip.stateSequence);
        assert(unchanged.targetSequence == firstGrip.targetSequence);
        assert(unchanged.gripSequence == firstGrip.gripSequence);
        assert(unchanged.releaseSequence == firstGrip.releaseSequence);

        auto retargeted = firstGrip;
        retargeted.reservedTargetIdentity = 101;
        retargeted.primaryBodyId = 21;
        retargeted.heldBodyIds[0] = 21;
        assignHandInteractionSequences(retargeted, firstGrip, true);
        assert(retargeted.stateSequence == 2);
        assert(retargeted.targetSequence == 2);
        assert(retargeted.gripSequence == 2);
        assert(retargeted.releaseSequence == 0);

        RockProviderHandInteractionStateV1 released{};
        released.hand = RockProviderHand::Right;
        released.phase = RockProviderHandInteractionPhaseV1::Idle;
        released.stateSequence = firstGrip.stateSequence;
        released.targetSequence = firstGrip.targetSequence;
        released.gripSequence = firstGrip.gripSequence;
        released.releaseSequence = firstGrip.releaseSequence;
        assignHandInteractionSequences(released, firstGrip, true);
        assert(released.stateSequence == 2);
        assert(released.targetSequence == 2);
        assert(released.gripSequence == 1);
        assert(released.releaseSequence == 1);
    }

    void testPayloadEquality()
    {
        const auto hand = makeHoldingState(200, 30);
        auto changedHand = hand;
        assert(sameHeldBodies(hand, changedHand));
        assert(sameHandTarget(hand, changedHand));
        assert(sameHandGrip(hand, changedHand));
        assert(sameHandInteractionPayload(hand, changedHand));
        changedHand.effectiveInputSuppressionFlags = 1;
        assert(!sameHandInteractionPayload(hand, changedHand));
        changedHand = hand;
        changedHand.heldBodyIds[0] = 31;
        assert(!sameHeldBodies(hand, changedHand));
        assert(!sameHandTarget(hand, changedHand));

        RockProviderFrameSnapshot lifecycle{};
        lifecycle.providerReady = 1;
        lifecycle.lifecycleFlags = 2;
        lifecycle.worldGeneration = 3;
        auto changedLifecycle = lifecycle;
        assert(sameLifecyclePayload(lifecycle, changedLifecycle));
        changedLifecycle.providerGeneration = 4;
        assert(!sameLifecyclePayload(lifecycle, changedLifecycle));

        RockProviderFrameSnapshot weapon{};
        weapon.weaponFormId = 10;
        weapon.weaponGenerationKey = 11;
        weapon.weaponBodyCount = 2;
        weapon.weaponBodyIds[0] = 12;
        weapon.weaponBodyIds[1] = 13;
        auto changedWeapon = weapon;
        assert(sameWeaponPayload(weapon, changedWeapon));
        changedWeapon.weaponBodyIds[1] = 14;
        assert(!sameWeaponPayload(weapon, changedWeapon));

        RockProviderEquippedWeaponStateV1 equipped{};
        equipped.weaponFormId = 20;
        equipped.weaponGenerationKey = 21;
        equipped.transitionSequence = 22;
        equipped.terminalSequence = 23;
        auto changedEquipped = equipped;
        assert(sameEquippedWeaponPayload(equipped, changedEquipped));
        changedEquipped.terminalSequence = 24;
        assert(!sameEquippedWeaponPayload(equipped, changedEquipped));
    }
}

int main()
{
    testSequenceAssignment();
    testPayloadEquality();
    return 0;
}

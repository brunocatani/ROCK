#include "api/ProviderStatePolicy.h"

#include <cassert>
#include <cstddef>

using namespace rock::provider;
using namespace rock::provider_state_policy;

template <class Flag>
constexpr std::uint32_t bit(Flag flag) { return static_cast<std::uint32_t>(flag); }

int main()
{
    using H = RockProviderHandInteractionFlagV1;
    using F = RockProviderHandStateFlag;
    using W = RockProviderEquippedWeaponHandlingRuntimeFlagV1;
    using C = RockProviderPlayerColliderFlagV1;
    using Phase = RockProviderHandInteractionPhaseV1;

    RockProviderEquippedWeaponHandlingStateV1 weapon{};
    weapon.weaponFormId = 42;
    weapon.weaponGenerationKey = 9;
    weapon.runtimeFlags = bit(W::WeaponPresent) | bit(W::FiringGripOccupied);
    RockProviderWeaponPartGripStateV1 grip{};

    const auto hand = [](RockProviderHand side) {
        RockProviderHandInteractionStateV1 state{};
        state.hand = side;
        state.flags = bit(H::Valid);
        return state;
    };
    // Inventory/holster equip occupies the firing hand without a ROCK capture.
    for (const auto side : {RockProviderHand::Right, RockProviderHand::Left}) {
        weapon.currentFiringHand = side;
        auto state = hand(side);
        state.phase = Phase::Selecting;
        state.flags |= bit(H::LooseObject);
        state.targetFormId = 123;
        state.primaryBodyId = 456;
        applyWeaponOccupancy(state, weapon, grip);
        assert(handHolding(state));
        assert(state.flags & bit(H::FiringGrip));
        assert(state.flags & bit(H::NativeWeaponCarry));
        assert(!(state.flags & (bit(H::LooseObject) | bit(H::RockGripActive))));
        assert(state.targetFormId == 42 && state.reservedTargetIdentity == 9);
        assert(state.targetKind == RockProviderBodyContactTargetKind::Weapon);
        assert(state.primaryBodyId == 0x7FFF'FFFF && state.heldBodyCount == 0);
        assert(handStateFlags(state, false) & bit(F::Holding));
        assert(handStateFlags(state, true) & bit(F::PhysicsDisabled));

        auto other = hand(side == RockProviderHand::Left ? RockProviderHand::Right : RockProviderHand::Left);
        applyWeaponOccupancy(other, weapon, grip);
        assert(!handHolding(other));
    }
    weapon.currentFiringHand = RockProviderHand::Right;
    grip.active = 1;
    grip.gripKind = RockProviderWeaponPartGripKindV1::FiringGrip;
    auto captured = hand(RockProviderHand::Right);
    applyWeaponOccupancy(captured, weapon, grip);
    assert(captured.flags & bit(H::RockGripActive));
    assert(!(captured.flags & bit(H::NativeWeaponCarry)));
    assert(handHolding(captured));

    // Detached firing hand is free; a carrying or attachment-only hand is busy.
    weapon.runtimeFlags = bit(W::WeaponPresent) | bit(W::PartCarryActive);
    auto detached = hand(RockProviderHand::Right);
    applyWeaponOccupancy(detached, weapon, {});
    assert(!handHolding(detached));
    grip.gripKind = RockProviderWeaponPartGripKindV1::PartCarry;
    auto carry = hand(RockProviderHand::Left);
    applyWeaponOccupancy(carry, weapon, grip);
    assert(handHolding(carry) && (carry.flags & bit(H::PartCarry)));
    grip.gripKind = RockProviderWeaponPartGripKindV1::AttachOnly;
    grip.attachOnly = 1;
    auto attached = hand(RockProviderHand::Left);
    applyWeaponOccupancy(attached, weapon, grip);
    assert(handHolding(attached) && (attached.flags & bit(H::AttachOnly)));
    assert(attached.flags & bit(H::PartGrip));
    assert(!(attached.flags & (bit(H::FiringGrip) | bit(H::PartCarry))));

    auto absent = hand(RockProviderHand::Right);
    applyWeaponOccupancy(absent, {}, grip);
    assert(!handHolding(absent));
    auto unknown = hand(RockProviderHand::Right);
    unknown.flags = 0;
    unknown.phase = Phase::Holding;
    assert(!handHolding(unknown));
    assert(!(handStateFlags(unknown, false) & (bit(F::Valid) | bit(F::Holding))));
    auto touch = hand(RockProviderHand::Left);
    touch.flags |= bit(H::TouchGrab) | bit(H::FixedSurfaceLatch);
    touch.phase = Phase::Holding;
    assert(handStateFlags(touch, false) & bit(F::Holding));
    for (const auto phase : {Phase::StashCandidate, Phase::ConsumeCandidate}) {
        touch.phase = phase;
        assert(handHolding(touch));
    }
    for (const auto phase : {Phase::Idle, Phase::Touching, Phase::Selecting, Phase::Releasing}) {
        touch.phase = phase;
        assert(!handHolding(touch));
    }
    assert(firingGripOccupied(true, false));
    assert(!firingGripOccupied(false, false));
    assert(!firingGripOccupied(true, true));

    // Allocation/readiness and the actual body filter answer different questions.
    assert(colliderFlags(true, true, false) & bit(C::Enabled));
    auto suppressed = colliderFlags(true, true, true);
    assert((suppressed & (bit(C::LifecycleAllowed) | bit(C::FilterKnown) | bit(C::CollisionSuppressed))) != 0);
    assert(!(suppressed & bit(C::Enabled)));
    assert(!(colliderFlags(true, false, false) & (bit(C::FilterKnown) | bit(C::Enabled))));
    assert(!(colliderFlags(false, true, false) & bit(C::Enabled)));

    // Reused outputs cannot retain a positive state after a failed query.
    clearQueryOutput(&captured);
    assert(captured.flags == 0 && captured.targetFormId == 0);
    assert(captured.size == sizeof(captured) && captured.version == ROCK_PROVIDER_API_VERSION);
    RockProviderWeaponContactResult contact{}; // Older output has no version member.
    contact.valid = 1;
    clearQueryOutput(&contact);
    assert(contact.valid == 0 && contact.bodyId == 0x7FFF'FFFF);
    RockProviderHandFrameV1 prefix{};
    prefix.size = 112;
    prefix.flags = 0xFFFF;
    prefix.frameIndex = 12345; // Beyond the legacy 112-byte prefix.
    clearQueryOutput(&prefix, 112);
    assert(prefix.size == 112 && prefix.flags == 0 && prefix.frameIndex == 12345);
    prefix.size = 4;
    prefix.flags = 77;
    clearQueryOutput(&prefix, 112);
    assert(prefix.flags == 77); // Undersized storage is never overwritten.
}

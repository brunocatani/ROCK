#pragma once

#include <span>
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "physics-interaction/weapon/LooseWeaponAuthoredGrabPolicy.h"
#include "physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h"

namespace RE
{
    class NiAVObject;
    class TESObjectWEAP;
    class TESObjectREFR;
}

namespace rock::loose_weapon_grip_zone
{
    /*
     * Firing-grip zone for loosely held weapons.
     *
     * While a hand holds a loose/dynamic weapon, this runtime resolves one
     * canonical firing relation from ROCK's learned native-animation pose.
     * It projects that fixed Weapon-relative grip onto the loose
     * model and tracks whether the holding palm is inside the grip radius.
     *
     * All geometry is weapon-root-local at the projection step and world at
     * the comparison step; nothing is stored hand-relative.
     *
     * Threading: all functions must be called from the game frame thread
     * (PhysicsInteraction frame update); state is not synchronized.
     */

    struct GripZoneDebug
    {
        bool valid{ false };
        bool palmValid{ false };
        bool insideRadius{ false };
        RE::NiPoint3 gripWorld{};
        RE::NiPoint3 palmWorld{};
        float palmToGripDistance{ 0.0f };
    };

    /*
     * Canonical primary-hand frame for this frame's loose-hold solves. Every
     * loose placement mirrors from the physical RIGHT hand; while ROCK
     * presents that hand (support lock, part carry, authored seat) the
     * rendered bone is ROCK's own output and must not feed the solve.
     * PhysicsInteraction publishes the weapon-authority physical frame once
     * per frame before any grab commit. A publication without a frame while
     * ROCK presents the hand fails the authored placement closed; without
     * ROCK presentation the rendered bone remains the legacy source.
     */
    struct CanonicalPrimaryHandFrame
    {
        RE::NiTransform handWorld{};
        bool valid{ false };
        bool presentedByRock{ false };
    };

    void publishCanonicalPrimaryHandFrame(const CanonicalPrimaryHandFrame& frame);

    void publishPhysicalLeftHandFrame(const CanonicalPrimaryHandFrame& frame);

    struct NearGrab
    {
        loose_weapon_authored_grab_policy::Role role{ loose_weapon_authored_grab_policy::Role::None };
        RE::NiTransform handWorld{};
        // Authored relation for hand/finger presentation, independent of the
        // physical relation used to place the weapon at ROCK's controller aim.
        RE::NiTransform handWeaponLocal{};
        RE::NiTransform placementHandWeaponLocal{};
    };

    struct AuthoredSupportDebug
    {
        bool valid{ false };
        bool eligible{ false };
        authored_weapon_grip_activation_policy::WeaponFamily family{};
        RE::NiPoint3 seatWorld{};
        RE::NiPoint3 probeWorld{};
        RE::NiPoint3 sideWorld{};
        RE::NiPoint3 downWorld{};
        RE::NiPoint3 referenceWorld{};
        float radius{ 0.0f };
    };

    // Frame-thread only. Reprojects both seats from the current loose root;
    // never borrows equipped ownership or retains scene pointers.
    bool tryResolveNearGrab(bool isLeft, RE::TESObjectREFR* ref, NearGrab& out);
    void updateNearGrabCandidate(bool isLeft, RE::TESObjectREFR* ref);
    bool tryGetAuthoredSupportDebug(bool isLeft, AuthoredSupportDebug& out);
    std::size_t collectIndicators(bool isLeft, RE::TESObjectREFR* ref, std::span<RE::NiPoint3> positions);

    /*
     * Refresh one hand's grip-zone state. Call once per frame per hand.
     * heldSettled must be true only while the grab is in its settled held
     * state (HeldBody); the inside-radius settle timer only accumulates then.
     * Pass holdingLooseWeapon=false to clear the hand's state.
     */
    void updateHeldLooseWeapon(
        bool isLeft,
        bool holdingLooseWeapon,
        RE::TESObjectREFR* heldRef,
        bool heldSettled,
        float dt,
        float equipRadiusGameUnits,
        bool firingGripEligible = true);

    /*
     * True when the hand's palm has stayed inside the configured grip radius
     * for the configured settle time while the grab is settled. The feature
     * toggle and same-hand ownership contract are enforced by the caller's
     * input policy, not here.
     */
    bool isGripZoneEquipSettled(bool isLeft, float settleSeconds);

    /*
     * Returns the canonical firing-hand frame already resolved for the held
     * weapon this frame. The frame is weapon-root-local and therefore remains
     * valid across the loose-reference to equipped-node inventory transfer.
     */
    bool tryGetFiringHandWeaponLocal(
        bool isLeft,
        RE::NiTransform& outHandWeaponLocal,
        RE::NiPoint3& outFiringGripWeaponLocal);

    /*
     * Stateless one-shot resolver of the loose weapon PLACEMENT hold. For an
     * authored grip this always derives the position-only hold from the
     * ROCK controller basis. It does not depend on a previous equip or a
     * provider offset; missing authored data leaves the grip unavailable. The other hand
     * receives the matching mirrored hold. Weapon world = hand world o
     * inverse(hold). Used by pull-catch and force-grab commit; frame-thread
     * only.
     */
    bool tryResolveLooseWeaponFiringHandHold(
        bool isLeft,
        RE::TESObjectREFR* weaponRef,
        RE::NiTransform& outHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const char** outReason);

    // Same resolver for a detached world model during loose-to-equipped
    // visual handoff. No reference lifetime is retained.
    bool tryResolveLooseWeaponFiringHandHoldForModel(
        bool isLeft,
        const RE::TESObjectWEAP* weapon,
        RE::NiAVObject* weaponRoot,
        RE::NiTransform& outHandWorld,
        RE::NiTransform& outHandWeaponLocal,
        const char** outReason);

    /*
     * Hover probe for the OPEN hand: projects the same firing-grip point onto
     * the hand's current selection candidate (not the held object) so the
     * player can feel, before grabbing, that a grab right now would land on
     * the firing grip and equip. Call once per frame per hand with the
     * selection ref while the hand is not holding; pass nullptr to clear.
     * The projection uses the canonical primary weapon attach surface for
     * both physical hands; grenade exclusion, feature toggles, and menu gating
     * are enforced by the caller.
     */
    void updateHoverCandidateWeapon(
        bool isLeft,
        RE::TESObjectREFR* candidateRef,
        float equipRadiusGameUnits);

    /*
     * True while the open palm hovers inside the configured grip-zone equip
     * radius over the current hover candidate. Continuous state, re-evaluated
     * by updateHoverCandidateWeapon each frame.
     */
    bool isGripZoneHoverInsideRadius(bool isLeft);

    /*
     * Read the last resolved grip zone for one hand for debug-overlay
     * publishing. Returns false when the hand has no valid state this frame.
     */
    bool tryGetGripZoneDebug(bool isLeft, GripZoneDebug& out);
}

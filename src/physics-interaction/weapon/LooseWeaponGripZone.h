#pragma once

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

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
     * canonical firing relation in strict authority order: a user hFRIK JSON,
     * ROCK's learned native-animation pose, then an hFRIK embedded cold
     * fallback. It projects that fixed Weapon-relative grip onto the loose
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
        float equipRadiusGameUnits);

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
     * Stateless one-shot resolver of the canonical firing hold for a loose
     * weapon in the tested hand: the primary hand receives the selected
     * canonical pose, the other hand receives ROCK's mirrored firing hold, both
     * expressed as the tested hand's live root-flattened frame plus the hand
     * transform in weapon-root-local space (weapon world = hand world o
     * inverse(hold)). Used by the pull-catch/force-grab commit to seat a far
     * grabbed weapon directly on its firing grip; the grip-zone runtimes
     * above share the same projection. Frame-thread only.
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

#pragma once

#include "RE/NetImmerse/NiPoint.h"

namespace RE
{
    class TESObjectREFR;
}

namespace rock::loose_weapon_grip_zone
{
    /*
     * Firing-grip zone for loosely held weapons.
     *
     * While a hand holds a loose/dynamic weapon, this runtime projects the
     * FRIK weapon offset (frik_weapon_offset_cache: saved offsets, FRIK.dll
     * embedded defaults, live-node fallback) onto the held world model to
     * find where the firing grip sits on the mesh, then tracks whether the
     * holding palm is inside the grip radius. PhysicsInteraction consumes
     * isGripZoneEquipSettled() to gate loose-weapon equip on actually holding
     * the gun by its grip; the debug overlay consumes the snapshot.
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
    void updateHeldLooseWeapon(bool isLeft, bool holdingLooseWeapon, RE::TESObjectREFR* heldRef, bool heldSettled, float dt);

    /*
     * True when the hand's palm has stayed inside the configured grip radius
     * for the configured settle time while the grab is settled. Hand-role
     * (primary vs offhand) and the feature toggle are enforced by the caller's
     * input policy, not here.
     */
    bool isGripZoneEquipSettled(bool isLeft);

    /*
     * Hover probe for the OPEN hand: projects the same firing-grip point onto
     * the hand's current selection candidate (not the held object) so the
     * player can feel, before grabbing, that a grab right now would land on
     * the firing grip and equip. Call once per frame per hand with the
     * selection ref while the hand is not holding; pass nullptr to clear.
     * Hand-role (primary only -- the projection runs through the primary
     * weapon attach node), grenade exclusion, feature toggles, and menu
     * gating are enforced by the caller.
     */
    void updateHoverCandidateWeapon(bool isLeft, RE::TESObjectREFR* candidateRef);

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

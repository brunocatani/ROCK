#pragma once

#include <cstdint>

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class NiNode;
    class TESObjectREFR;
}

namespace rock::loose_weapon_grip_probe
{
    /*
     * Verification spike for loose-weapon firing-grip detection.
     *
     * While a weapon is equipped and FRIK's saved offset glues the weapon node
     * to the firing hand, the firing-hand palm pivot IS the firing grip. This
     * module captures that relationship in weapon-node-local space (weapon-
     * relative, never hand-relative) keyed by weapon base form, then
     * re-projects it onto the dropped world model when the same weapon form is
     * later held as a loose/dynamic grab.
     *
     * Output is confirmation data only: rate-limited logs plus a per-hand
     * debug snapshot consumed by the debug overlay. No gameplay behavior reads
     * this state. Promotion/removal condition: replaced by a production
     * grip-zone module once the equipped-model to world-model space
     * correspondence is confirmed in-game.
     *
     * Threading: all functions must be called from the game frame thread
     * (PhysicsInteraction frame update); state is not synchronized.
     */

    struct ResolvedGripDebug
    {
        bool valid{ false };
        bool palmValid{ false };
        RE::NiPoint3 gripWorld{};
        RE::NiPoint3 palmWorld{};
        RE::NiTransform handTargetWorld{};
        float palmToGripDistance{ 0.0f };
    };

    /*
     * Learn/refresh the firing-grip frame from the currently equipped weapon.
     * Call once per frame with the resolved first-person weapon node. Skips
     * capture while ROCK owns the weapon transform (two-handed manual
     * ownership) because the node no longer reflects the pure FRIK offset.
     * Rate-limited internally; safe to call every frame.
     */
    void captureFromEquippedWeapon(RE::NiNode* weaponNode, bool firingHandIsLeft, bool weaponTransformOwnedByRock);

    /*
     * Re-project the learned grip onto a loosely held weapon reference and
     * refresh the per-hand debug snapshot. Call once per frame per hand;
     * pass holdingLooseWeapon=false to clear the hand's snapshot.
     */
    void updateHeldLooseWeaponProbe(bool isLeft, bool holdingLooseWeapon, RE::TESObjectREFR* heldRef);

    /*
     * Read the last resolved grip for one hand for debug-overlay publishing.
     * Returns false when the hand has no valid resolved grip this frame.
     */
    bool tryGetResolvedGripDebug(bool isLeft, ResolvedGripDebug& out);
}

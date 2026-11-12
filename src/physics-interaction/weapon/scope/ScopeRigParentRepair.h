// Scope rig re-parenting after a native unequip of a left-carried weapon (see
// ScopeRigParentRepair.cpp).
#pragma once

/*
 * ROCK's left-firing carry makes FRIK parent the player's scope rig
 * (ScopeParent and the primary weapon scope camera) under the "Weapon" node.
 * A native unequip of that weapon tears the node's children down, leaving
 * both rig nodes without a parent; FRIK's per-frame carry then reads the
 * missing parent before ROCK releases the carry (crash at FRIK
 * WeaponPositionAdjuster::carryScopeRigWithWeapon, observed twice at
 * runtime). The unequip can land between frames (a game task), so the
 * repair runs at FRIK's FrameBegin, which precedes FRIK's carry in
 * onFrameStart: an orphaned rig node returns to its wand-chain parent with
 * its world kept, as FRIK's own restore does; FRIK then carries or restores
 * it from a valid parent.
 */
namespace rock::scope_rig_parent_repair
{
    void reattachOrphanedScopeRig() noexcept;
}

#pragma once

// PalmTransform.h — Palm position/direction computation for ROCK.
//
// WHY THIS APPROACH (rewritten 2026-03-28):
// Previously derived forward direction from FRIK's finger bone
// (getBoneWorldTransform("RArm_Finger31")). This created an architectural
// inconsistency when we switched to raw VR wand nodes for collision body
// positioning — the collision body tracked the wand, but palm direction
// came from FRIK's IK skeleton.
//
// Now derives ALL directions from the wand node's rotation matrix.
// FO4VR wand axis convention (Ghidra-verified 2026-03-28, see HAVOK_CONVENTIONS.md):
//   +Y axis (column 1) = barrel/fingertip direction (tilted ~60° up from grip angle)
//   +X axis (column 0) = thumb side (right hand) / pinky side (left hand)
//   +Z axis (column 2) = back of hand (dorsal)
//   -Z axis             = palm face
//
// The configurable offsets (rockPalmOffsetForward/Up/Right) are applied along
// these wand-derived axes, NOT along world axes or FRIK bone axes.

#include "RockConfig.h"

namespace frik::rock
{
	/// Extract forward (fingertip) direction from wand rotation matrix.
	/// = Y axis = column 1 of NiMatrix3 (row-major: entry[row][col]).
	inline RE::NiPoint3 extractWandForward(const RE::NiMatrix3& rot)
	{
		return RE::NiPoint3(rot.entry[0][1], rot.entry[1][1], rot.entry[2][1]);
	}

	/// Extract right direction from wand rotation matrix.
	/// = X axis = column 0. For right hand this is thumb side, for left it's pinky side.
	inline RE::NiPoint3 extractWandRight(const RE::NiMatrix3& rot)
	{
		return RE::NiPoint3(rot.entry[0][0], rot.entry[1][0], rot.entry[2][0]);
	}

	/// Extract up (dorsal/back-of-hand) direction from wand rotation matrix.
	/// = Z axis = column 2. +Z = back of hand, -Z = palm face.
	inline RE::NiPoint3 extractWandUp(const RE::NiMatrix3& rot)
	{
		return RE::NiPoint3(rot.entry[0][2], rot.entry[1][2], rot.entry[2][2]);
	}

	/// Compute palm position from wand transform with configurable offset.
	/// All directions derived from the wand node's rotation matrix — no FRIK bone lookup.
	///
	/// Offsets are in game units, applied along wand-local axes:
	///   rockPalmOffsetForward = along Y axis (barrel, toward fingertips)
	///   rockPalmOffsetUp      = along Z axis (dorsal, away from palm)
	///   rockPalmOffsetRight   = along X axis (sign-flipped for left hand)
	inline RE::NiPoint3 computePalmPosition(const RE::NiTransform& handTransform, bool isLeft)
	{
		RE::NiPoint3 forward = extractWandForward(handTransform.rotate);
		RE::NiPoint3 right   = extractWandRight(handTransform.rotate);
		RE::NiPoint3 up      = extractWandUp(handTransform.rotate);

		// X axis mirrors between hands: right hand X = thumb, left hand X = pinky.
		// rockPalmOffsetRight is "toward thumb," so flip sign for left hand.
		float rightSign = isLeft ? -1.0f : 1.0f;

		return handTransform.translate
			+ forward * g_rockConfig.rockPalmOffsetForward
			+ up * g_rockConfig.rockPalmOffsetUp
			+ right * (g_rockConfig.rockPalmOffsetRight * rightSign);
	}

	/// Compute a rotation matrix from wand axes.
	/// Columns = right (X), forward (Y), up (Z) from the wand node.
	/// NiMatrix3 is row-major: entry[row][col].
	inline RE::NiMatrix3 computePalmRotation(const RE::NiTransform& handTransform, bool isLeft)
	{
		(void)isLeft;  // No per-hand adjustment needed — wand matrix is already correct
		return handTransform.rotate;
	}

	/// Compute palm forward direction (fingertip pointing direction).
	/// = Y axis of the wand rotation matrix = barrel direction.
	inline RE::NiPoint3 computePalmForward(const RE::NiTransform& handTransform, bool isLeft)
	{
		(void)isLeft;  // Y axis is the same for both hands (barrel direction)
		return extractWandForward(handTransform.rotate);
	}
}

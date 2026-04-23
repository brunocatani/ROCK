#pragma once

#include <cmath>

// PalmTransform.h — Transitional palm/collision transform helpers for ROCK.
//
// WHY THIS LAYER:
// ROCK's authored hand-space tuning was created around a stable local basis:
//   +X = fingertips, +Y = back of hand, +Z = lateral
// FRIK/FO4 already provide the final world hand transform, but that raw hand-node basis is:
//   +X = lateral, +Y = fingertips, +Z = back of hand
// The old canonicalize path tried to infer a new basis from semantics every frame, which
// made collider offsets depend on world heading and hand rotation. The safe contract is to
// keep the authored tuning basis, map it to the raw hand-node basis with a fixed axis
// permutation, and then use the original hand transform directly.
//
// This file now exposes two parallel helper families:
// 1. hand-space helpers (`*FromHandBasis`) — authored ROCK hand-space mapped onto the raw hand node
// 2. legacy wrappers (`computePalmPosition`, `computePalmForward`, `computePalmRotation`) —
//    preserved until Phase 3 migrates runtime call sites away from wand-local semantics.

#include "RockConfig.h"
#include "PhysicsUtils.h"

	namespace frik::rock
	{
		inline RE::NiPoint3 mirrorHandspaceZ(RE::NiPoint3 value, bool isLeft)
		{
			// In the authored ROCK hand-space, Z is the lateral axis. That axis mirrors
			// between hands; the authored basis itself stays stable.
			if (isLeft) {
				value.z *= -1.0f;
			}
			return value;
	}

	inline RE::NiPoint3 normalizeDirection(RE::NiPoint3 value)
	{
		const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
		if (lengthSquared <= 1.0e-8f) {
			return RE::NiPoint3(0.0f, 0.0f, 1.0f);
		}

		const float inverseLength = 1.0f / std::sqrt(lengthSquared);
		value.x *= inverseLength;
		value.y *= inverseLength;
		value.z *= inverseLength;
		return value;
	}

	inline RE::NiMatrix3 authoredHandspaceRotationFromRawHandBasis(const RE::NiMatrix3& rawRotation)
	{
		RE::NiMatrix3 result{};

		// Authored ROCK basis:
		//   X = fingertips, Y = back of hand, Z = lateral
		// Raw FO4/FRIK hand-node basis:
		//   X = lateral, Y = fingertips, Z = back of hand
		//
		// Reorder the raw basis columns into the authored axis order. This preserves the
		// intended collider/palm tuning semantics without inventing a new inferred basis.

		// Authored X (fingertips) follows raw Y.
		result.entry[0][0] = rawRotation.entry[0][1];
		result.entry[1][0] = rawRotation.entry[1][1];
		result.entry[2][0] = rawRotation.entry[2][1];

		// Authored Y (back of hand) follows raw Z.
		result.entry[0][1] = rawRotation.entry[0][2];
		result.entry[1][1] = rawRotation.entry[1][2];
		result.entry[2][1] = rawRotation.entry[2][2];

		// Authored Z (lateral) follows raw X.
		result.entry[0][2] = rawRotation.entry[0][0];
		result.entry[1][2] = rawRotation.entry[1][0];
		result.entry[2][2] = rawRotation.entry[2][0];
		return result;
	}

	inline RE::NiPoint3 transformHandspacePosition(const RE::NiTransform& handTransform,
		const RE::NiPoint3& localPosition, bool isLeft)
	{
		const RE::NiMatrix3 rigidRotation = authoredHandspaceRotationFromRawHandBasis(handTransform.rotate);
		return handTransform.translate + rigidRotation * (mirrorHandspaceZ(localPosition, isLeft) * handTransform.scale);
	}

	inline RE::NiPoint3 transformHandspaceDirection(const RE::NiTransform& handTransform,
		const RE::NiPoint3& localDirection, bool isLeft)
	{
		const RE::NiMatrix3 rigidRotation = authoredHandspaceRotationFromRawHandBasis(handTransform.rotate);
		return normalizeDirection(rigidRotation * mirrorHandspaceZ(localDirection, isLeft));
	}

	inline RE::NiTransform computeHandCollisionTransformFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
	{
		RE::NiTransform result = handTransform;
		result.rotate = authoredHandspaceRotationFromRawHandBasis(handTransform.rotate);
		result.translate = transformHandspacePosition(handTransform,
			g_rockConfig.rockHandCollisionOffsetHandspace * kHavokToGameScale, isLeft);
		return result;
	}

	inline RE::NiPoint3 computePalmPositionFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
	{
		return transformHandspacePosition(handTransform, g_rockConfig.rockPalmPositionHandspace, isLeft);
	}

	inline RE::NiPoint3 computePalmNormalFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
	{
		return transformHandspaceDirection(handTransform, g_rockConfig.rockPalmNormalHandspace, isLeft);
	}

	inline RE::NiPoint3 computePointingVectorFromHandBasis(const RE::NiTransform& handTransform, bool isLeft)
	{
		return transformHandspaceDirection(handTransform, g_rockConfig.rockPointingVectorHandspace, isLeft);
	}

	// -------------------------------------------------------------------------
	// Legacy wand-local helpers kept until Phase 3 source migration.
	// -------------------------------------------------------------------------

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

	/// Legacy palm position from wand transform with configurable offset.
	/// Preserved until runtime call sites migrate to the hand-space basis helpers.
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

	/// Legacy palm rotation matrix from wand axes.
	/// Columns = right (X), forward (Y), up (Z) from the wand node.
	/// NiMatrix3 is row-major: entry[row][col].
	inline RE::NiMatrix3 computePalmRotation(const RE::NiTransform& handTransform, bool isLeft)
	{
		(void)isLeft;  // No per-hand adjustment needed — wand matrix is already correct
		return handTransform.rotate;
	}

	/// Legacy palm forward direction (fingertip pointing direction).
	/// = Y axis of the wand rotation matrix = barrel direction.
	inline RE::NiPoint3 computePalmForward(const RE::NiTransform& handTransform, bool isLeft)
	{
		(void)isLeft;  // Y axis is the same for both hands (barrel direction)
		return extractWandForward(handTransform.rotate);
	}
}

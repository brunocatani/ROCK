#pragma once

// PhysicsUtils.h — Shared physics utilities for the ROCK physics module.
//
// WHY: Coordinate conversion between Bethesda game space and Havok physics space
// is needed throughout the physics module. Centralizing the scale factor and
// conversion helpers prevents bugs from inconsistent scaling and gives us a
// single place to adjust if the scale factor turns out to be different than expected.

#include "RE/Havok/hkVector4.h"

namespace frik::rock
{
	/// Scale factor: 1 Havok unit = ~70 Bethesda game units.
	/// Multiply game-space coordinates by this to get Havok-space coordinates.
	/// The game stores this at DAT_143718110 (REL::ID 771023).
	constexpr float kGameToHavokScale = 1.0f / 70.0f;
	constexpr float kHavokToGameScale = 70.0f;

	/// Convert a NiPoint3 (game space) to an hkVector4f (Havok space).
	inline RE::hkVector4f niPointToHkVector(const RE::NiPoint3& p)
	{
		return RE::hkVector4f{
			p.x * kGameToHavokScale,
			p.y * kGameToHavokScale,
			p.z * kGameToHavokScale,
			0.0f
		};
	}

	/// Convert an hkVector4f (Havok space) to a NiPoint3 (game space).
	inline RE::NiPoint3 hkVectorToNiPoint(const RE::hkVector4f& v)
	{
		return RE::NiPoint3{
			v.x * kHavokToGameScale,
			v.y * kHavokToGameScale,
			v.z * kHavokToGameScale
		};
	}

	/// Convert a NiMatrix3 rotation to an hkVector4f quaternion {x, y, z, w}.
	/// NiMatrix3 is a 3x3 rotation matrix stored row-major.
	/// Havok stores quaternions as hkVector4f with w = scalar component.
	///
	/// CRITICAL CONVENTION FIX (Session 2026-03-27):
	/// Havok's internal matrix→quaternion function FUN_141722c10 (Ghidra: 0x1722c10)
	/// uses the OPPOSITE sign convention for off-diagonal elements compared to the
	/// standard textbook formula:
	///   Textbook: qx = (m[2][1] - m[1][2]) * s
	///   Havok:    qx = (m[1][2] - m[2][1]) * s   ← produces the CONJUGATE
	///
	/// Both are valid quaternions for the same rotation (q and conj(q) encode the
	/// same rotation). BUT computeHardKeyFrame internally extracts the body's current
	/// orientation via FUN_141722c10 and computes q_diff = q_target * conj(q_body).
	/// If our target quaternion is already conjugated relative to Havok's convention,
	/// the diff becomes q_diff = conj(q) * conj(q_body) = conj(q_body * q),
	/// which produces angular velocity proportional to 2× the absolute world rotation
	/// angle — NOT the frame-to-frame delta. This caused 30-100+ rad/s spurious
	/// angular velocity on stationary hands, direction-dependent force, and left/right
	/// hand asymmetry.
	///
	/// Fix: Swap subtraction order in all 4 branches to match FUN_141722c10.
	/// Verified: dot(target, body) ≈ 1.0 after fix. Angular vel when still = 0-2 rad/s.
	inline RE::hkVector4f niRotToHkQuat(const RE::NiMatrix3& m)
	{
		// Matrix→quaternion conversion using Havok's sign convention (FUN_141722c10)
		RE::hkVector4f q;
		const float trace = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];

		if (trace > 0.0f) {
			const float s = 0.5f / sqrtf(trace + 1.0f);
			q.w = 0.25f / s;
			q.x = (m.entry[1][2] - m.entry[2][1]) * s;
			q.y = (m.entry[2][0] - m.entry[0][2]) * s;
			q.z = (m.entry[0][1] - m.entry[1][0]) * s;
		} else if (m.entry[0][0] > m.entry[1][1] && m.entry[0][0] > m.entry[2][2]) {
			const float s = 2.0f * sqrtf(1.0f + m.entry[0][0] - m.entry[1][1] - m.entry[2][2]);
			q.w = (m.entry[1][2] - m.entry[2][1]) / s;
			q.x = 0.25f * s;
			q.y = (m.entry[0][1] + m.entry[1][0]) / s;
			q.z = (m.entry[0][2] + m.entry[2][0]) / s;
		} else if (m.entry[1][1] > m.entry[2][2]) {
			const float s = 2.0f * sqrtf(1.0f + m.entry[1][1] - m.entry[0][0] - m.entry[2][2]);
			q.w = (m.entry[2][0] - m.entry[0][2]) / s;
			q.x = (m.entry[0][1] + m.entry[1][0]) / s;
			q.y = 0.25f * s;
			q.z = (m.entry[1][2] + m.entry[2][1]) / s;
		} else {
			const float s = 2.0f * sqrtf(1.0f + m.entry[2][2] - m.entry[0][0] - m.entry[1][1]);
			q.w = (m.entry[0][1] - m.entry[1][0]) / s;
			q.x = (m.entry[0][2] + m.entry[2][0]) / s;
			q.y = (m.entry[1][2] + m.entry[2][1]) / s;
			q.z = 0.25f * s;
		}

		// Normalize
		const float len = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
		if (len > 0.0f) {
			const float inv = 1.0f / len;
			q.x *= inv;
			q.y *= inv;
			q.z *= inv;
			q.w *= inv;
		}

		return q;
	}

	/// Convert a NiMatrix3 rotation to a float[4] quaternion {x, y, z, w}.
	/// For use with ComputeHardKeyFrame which takes float* parameters.
	inline void niRotToHkQuat(const RE::NiMatrix3& m, float outQuat[4])
	{
		auto q = niRotToHkQuat(m);
		outQuat[0] = q.x;
		outQuat[1] = q.y;
		outQuat[2] = q.z;
		outQuat[3] = q.w;
	}

	// =========================================================================
	// Center of Mass utilities (B8, blind audit 2026-03-31)
	//
	// In hknp, COM and body origin are DIFFERENT:
	//   motion+0x00 = Center of Mass (COM) in Havok world space
	//   body+0x30 (= body[12,13,14]) = body ORIGIN in Havok world space
	//   body_origin = COM - R * comOffset
	//   comOffset stored in W components: body+0x0C, +0x1C, +0x2C
	//
	// Constraint pivots are specified relative to body ORIGIN (not COM).
	// ROCK's grab point math (dot columns with delta from body origin) is correct.
	// COM is needed for: throw impulse application, gravity compensation,
	// tipping prediction, and angular momentum computation.
	//
	// Verified via Ghidra: bhkNPCollisionObject::GetCenterOfMassInWorld (0x141e08ef0)
	// reads motion+0x00 directly. hknpMotion::setFromMassProperties (0x1417d13a0)
	// writes motion+0x00 = transform * localCOM.
	// =========================================================================

	/// Read the Center of Mass (COM) position from the motion array in Havok world coordinates.
	/// Returns false if the body or motion is invalid.
	/// NOTE: motion+0x00 is COM, body[12,13,14] is body origin. They differ for
	/// objects with non-centered COM (most real objects).
	inline bool getBodyCOMWorld(RE::hknpWorld* world, RE::hknpBodyId bodyId,
		float& outX, float& outY, float& outZ)
	{
		if (!world || bodyId.value == 0x7FFF'FFFF) return false;

		auto* bodyArray = world->GetBodyArray();
		auto& body = bodyArray[bodyId.value];
		auto motionIndex = body.motionIndex;
		if (motionIndex == 0 || motionIndex > 4096) return false;

		auto* worldBytes = reinterpret_cast<char*>(world);
		auto* motionArrayPtr = *reinterpret_cast<char**>(worldBytes + 0xE0);
		if (!motionArrayPtr) return false;

		auto* motion = reinterpret_cast<float*>(motionArrayPtr + motionIndex * 0x80);
		outX = motion[0];  // COM.x at motion+0x00
		outY = motion[1];  // COM.y at motion+0x04
		outZ = motion[2];  // COM.z at motion+0x08
		return true;
	}

	/// Read the COM-to-body-origin offset from the body's rotation W components.
	/// This offset is in body-local space. World offset = R * comOffset.
	inline void getBodyCOMOffset(const float* bodyFloats,
		float& outX, float& outY, float& outZ)
	{
		outX = bodyFloats[3];   // body+0x0C = W of rotation column 0
		outY = bodyFloats[7];   // body+0x1C = W of rotation column 1
		outZ = bodyFloats[11];  // body+0x2C = W of rotation column 2
	}
}

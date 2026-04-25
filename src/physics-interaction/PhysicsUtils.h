#pragma once

// PhysicsUtils.h — Shared physics utilities for the ROCK physics module.
//
// WHY: Coordinate conversion between Bethesda game space and Havok physics space
// is needed throughout the physics module. Centralizing the scale factor and
// conversion helpers prevents bugs from inconsistent scaling and gives us a
// single place to adjust if the scale factor turns out to be different than expected.

#include "HavokOffsets.h"

#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/Havok/hkVector4.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"

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

	/// Resolve the bhkNPCollisionObject back-pointer stored on a live hknp body.
	///
	/// WHY: The FO4VR binary does expose a real body -> collisionObject -> owner-node
	/// contract, but not as a clean named helper like HIGGS/Skyrim VR. Centralizing
	/// the audited offsets here keeps later diagnostics and fixes on one typed path
	/// instead of repeating ad hoc `body+0x88` casts.
	inline RE::NiCollisionObject* getCollisionObjectFromBody(const RE::hknpBody* body)
	{
		if (!body) {
			return nullptr;
		}

		auto* bodyBytes = reinterpret_cast<const char*>(body);
		return *reinterpret_cast<RE::NiCollisionObject* const*>(
			bodyBytes + offsets::kBody_CollisionObjectBackPointer);
	}

	inline RE::NiCollisionObject* getCollisionObjectFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
	{
		if (!world || bodyId.value == 0x7FFF'FFFF) {
			return nullptr;
		}

		auto* bodyArray = world->GetBodyArray();
		if (!bodyArray) {
			return nullptr;
		}

		return getCollisionObjectFromBody(&bodyArray[bodyId.value]);
	}

	/// Resolve the owning NiAVObject from a NiCollisionObject.
	///
	/// WHY: The stable in-repo access path is the typed `sceneObject` field used by
	/// ObjectDetection.cpp. The previous raw offset read diverged from that contract
	/// and introduced a grab-time CTD in the new diagnostics path.
	inline RE::NiAVObject* getOwnerNodeFromCollisionObject(const RE::NiCollisionObject* collisionObject)
	{
		if (!collisionObject) {
			return nullptr;
		}

		return collisionObject->sceneObject;
	}

	inline RE::NiAVObject* getOwnerNodeFromBody(const RE::hknpBody* body)
	{
		return getOwnerNodeFromCollisionObject(getCollisionObjectFromBody(body));
	}

	inline RE::NiAVObject* getOwnerNodeFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
	{
		return getOwnerNodeFromCollisionObject(getCollisionObjectFromBody(world, bodyId));
	}

	/// Pack a NiMatrix3 into the 48-byte hkTransform rotation block consumed by
	/// bhkNPCollisionObject::SetTransform / hknpWorld::setBodyTransform.
	///
	/// WHY: Ghidra audit 2026-04-25 showed the FO4VR SetTransform wrapper passes
	/// the hkTransformf pointer through unchanged, and the native body setter copies
	/// float slots [0..2], [4..6], [8..10] directly into the live body. Native
	/// collision attach also seeds bodies by passing raw NiAVObject world rows.
	/// Therefore this path must preserve Ni rows; HIGGS's NiMatrixToHkMatrix column
	/// packing applies to hkMatrix3 constraint atoms, not FO4VR's CommonLibF4VR
	/// hkTransformf wrapper.
	inline RE::NiMatrix3 niRotToHkTransformRotation(const RE::NiMatrix3& m)
	{
		return m;
	}

	/// Rebuild a NiMatrix3 from the live hknp body rotation blocks.
	///
	/// WHY: The live hknp body stores the same row-indexed 3x4 slot layout written by
	/// the native SetTransform path. Rebuilding NiMatrix3 rows directly keeps visual
	/// diagnostics and body-local capture in the same transform convention used by
	/// FO4VR's native body-to-node writeback.
	inline RE::NiMatrix3 havokRotationBlocksToNiMatrix(const float* bodyFloats)
	{
		RE::NiMatrix3 result;
		result.entry[0][0] = bodyFloats[0];
		result.entry[0][1] = bodyFloats[1];
		result.entry[0][2] = bodyFloats[2];
		result.entry[1][0] = bodyFloats[4];
		result.entry[1][1] = bodyFloats[5];
		result.entry[1][2] = bodyFloats[6];
		result.entry[2][0] = bodyFloats[8];
		result.entry[2][1] = bodyFloats[9];
		result.entry[2][2] = bodyFloats[10];
		result.entry[0][3] = 0.0f;
		result.entry[1][3] = 0.0f;
		result.entry[2][3] = 0.0f;
		return result;
	}

	/// Project a Havok-world delta into hknp body-local coordinates.
	///
	/// WHY: FO4VR stores body transforms as row-indexed slots for SetTransform and
	/// visual sync, but native hknp local-point math uses row-vector multiplication:
	/// world = local * R + t. Therefore world-to-body-local is the dot product
	/// against each stored row. Do not replace this with NiMatrix3 column-vector
	/// inverse math; this helper feeds constraint pivot atoms, not Ni visual frames.
	inline RE::NiPoint3 worldDeltaToBodyLocal(const float* bodyFloats, const RE::NiPoint3& worldDelta)
	{
		return RE::NiPoint3{
			bodyFloats[0] * worldDelta.x + bodyFloats[1] * worldDelta.y + bodyFloats[2] * worldDelta.z,
			bodyFloats[4] * worldDelta.x + bodyFloats[5] * worldDelta.y + bodyFloats[6] * worldDelta.z,
			bodyFloats[8] * worldDelta.x + bodyFloats[9] * worldDelta.y + bodyFloats[10] * worldDelta.z
		};
	}

	/// Convert a NiMatrix3 rotation to an hkVector4f quaternion {x, y, z, w}.
	/// NiMatrix3 is a 3x3 rotation matrix stored row-major.
	/// Havok stores quaternions as hkVector4f with w = scalar component.
	///
	/// WHY: ROCK's manual keyframed hand/weapon path computes angular velocity via
	/// computeHardKeyFrame(), then teleports the body with the same raw-row hkTransformf
	/// contract used by SetTransform. Blind audit 2026-04-25 showed that FO4VR's
	/// body setter reads the standard row-indexed matrix slots directly.
	///
	/// For ROCK's manual path we must match:
	///   FUN_141722c10(niRotToHkTransformRotation(m))
	/// i.e. the exact raw-row quaternion convention used by native DriveToKeyFrame
	/// and by computeHardKeyFrame when it reads the live body rows. Because FO4VR's
	/// CommonLibF4VR hkTransformf preserves raw Ni rows, this is the conjugated
	/// raw-row convention, not HIGGS's packed hkMatrix3/textbook convention.
	inline RE::hkVector4f niRotToHkQuat(const RE::NiMatrix3& m)
	{
		// Matrix-to-quaternion conversion matching FO4VR FUN_141722c10 on raw rows.
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
	//   body_origin = COM - row-vector-transform(comOffset, R)
	//   comOffset stored in W components: body+0x0C, +0x1C, +0x2C
	//
	// Constraint pivots are specified relative to body ORIGIN (not COM).
	// ROCK's grab math must dot the world delta against the live hknp body rows
	// X=[0,1,2], Y=[4,5,6], Z=[8,9,10] from that body origin.
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
	/// This offset is in hknp body-local space. Native world offset uses row-vector
	/// multiplication with the body rows.
	inline void getBodyCOMOffset(const float* bodyFloats,
		float& outX, float& outY, float& outZ)
	{
		outX = bodyFloats[3];   // body+0x0C = row 0 W / local COM offset X
		outY = bodyFloats[7];   // body+0x1C = row 1 W / local COM offset Y
		outZ = bodyFloats[11];  // body+0x2C = row 2 W / local COM offset Z
	}
}

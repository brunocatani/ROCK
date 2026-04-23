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

	/// Pack a NiMatrix3 into the 48-byte hkTransform rotation block the way
	/// bhkNPCollisionObject::SetTransform consumes it.
	///
	/// WHY: FO4VR's live hknp body stores its world basis as three contiguous Havok
	/// columns (X=[0,1,2], Y=[4,5,6], Z=[8,9,10]), while SetTransform takes a raw
	/// 3x4 block whose rows become those live columns. Matching HIGGS's
	/// NiMatrixToHkMatrix convention therefore requires packing Ni columns into the
	/// outgoing rows. Passing NiMatrix3 through unchanged would make the live Havok
	/// columns equal the Ni rows and rotate against the wrong basis.
	inline RE::NiMatrix3 niRotToHkTransformRotation(const RE::NiMatrix3& m)
	{
		RE::NiMatrix3 packed;
		packed.entry[0][0] = m.entry[0][0];
		packed.entry[0][1] = m.entry[1][0];
		packed.entry[0][2] = m.entry[2][0];
		packed.entry[0][3] = 0.0f;
		packed.entry[1][0] = m.entry[0][1];
		packed.entry[1][1] = m.entry[1][1];
		packed.entry[1][2] = m.entry[2][1];
		packed.entry[1][3] = 0.0f;
		packed.entry[2][0] = m.entry[0][2];
		packed.entry[2][1] = m.entry[1][2];
		packed.entry[2][2] = m.entry[2][2];
		packed.entry[2][3] = 0.0f;
		return packed;
	}

	/// Rebuild a NiMatrix3 from the live hknp body rotation blocks.
	///
	/// WHY: The FO4VR body array stores three 16-byte Havok columns, not three Ni rows.
	/// Reconstructing NiMatrix3 from those contiguous basis vectors keeps grab math,
	/// held-object math, and transform witnesses aligned with the runtime body state.
	inline RE::NiMatrix3 havokRotationBlocksToNiMatrix(const float* bodyFloats)
	{
		RE::NiMatrix3 result;
		result.entry[0][0] = bodyFloats[0];
		result.entry[1][0] = bodyFloats[1];
		result.entry[2][0] = bodyFloats[2];
		result.entry[0][1] = bodyFloats[4];
		result.entry[1][1] = bodyFloats[5];
		result.entry[2][1] = bodyFloats[6];
		result.entry[0][2] = bodyFloats[8];
		result.entry[1][2] = bodyFloats[9];
		result.entry[2][2] = bodyFloats[10];
		result.entry[0][3] = 0.0f;
		result.entry[1][3] = 0.0f;
		result.entry[2][3] = 0.0f;
		return result;
	}

	/// Project a Havok-world delta into body-local coordinates using the live body basis.
	///
	/// WHY: hknp uses left-multiplication, so body-local = R^T * worldDelta. With the
	/// live body basis stored as contiguous Havok columns, each local component is the
	/// dot product of the world delta against one of those basis vectors.
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
	/// computeHardKeyFrame(), then teleports the body with a PACKED hkTransformf built
	/// by niRotToHkTransformRotation(). Blind audit 2026-04-21 showed that
	/// FUN_141722c10 only looks conjugated when it is fed a raw NiTransform row block,
	/// which is Bethesda's separate DriveToKeyFrame(NiTransform*) contract.
	///
	/// For ROCK's manual path we must instead match:
	///   FUN_141722c10(niRotToHkTransformRotation(m))
	/// i.e. the packed-Havok/live-body contract. That yields the normal textbook
	/// quaternion of the authored Ni rotation matrix, not its conjugate.
	inline RE::hkVector4f niRotToHkQuat(const RE::NiMatrix3& m)
	{
		// Matrix→quaternion conversion matching the packed hkTransformf/live-body basis.
		RE::hkVector4f q;
		const float trace = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];

		if (trace > 0.0f) {
			const float s = 0.5f / sqrtf(trace + 1.0f);
			q.w = 0.25f / s;
			q.x = (m.entry[2][1] - m.entry[1][2]) * s;
			q.y = (m.entry[0][2] - m.entry[2][0]) * s;
			q.z = (m.entry[1][0] - m.entry[0][1]) * s;
		} else if (m.entry[0][0] > m.entry[1][1] && m.entry[0][0] > m.entry[2][2]) {
			const float s = 2.0f * sqrtf(1.0f + m.entry[0][0] - m.entry[1][1] - m.entry[2][2]);
			q.w = (m.entry[2][1] - m.entry[1][2]) / s;
			q.x = 0.25f * s;
			q.y = (m.entry[0][1] + m.entry[1][0]) / s;
			q.z = (m.entry[0][2] + m.entry[2][0]) / s;
		} else if (m.entry[1][1] > m.entry[2][2]) {
			const float s = 2.0f * sqrtf(1.0f + m.entry[1][1] - m.entry[0][0] - m.entry[2][2]);
			q.w = (m.entry[0][2] - m.entry[2][0]) / s;
			q.x = (m.entry[0][1] + m.entry[1][0]) / s;
			q.y = 0.25f * s;
			q.z = (m.entry[1][2] + m.entry[2][1]) / s;
		} else {
			const float s = 2.0f * sqrtf(1.0f + m.entry[2][2] - m.entry[0][0] - m.entry[1][1]);
			q.w = (m.entry[1][0] - m.entry[0][1]) / s;
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
	// ROCK's grab math must dot the world delta against the live Havok basis columns
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
	/// This offset is in body-local space. World offset = R * comOffset.
	inline void getBodyCOMOffset(const float* bodyFloats,
		float& outX, float& outY, float& outZ)
	{
		outX = bodyFloats[3];   // body+0x0C = W of rotation column 0
		outY = bodyFloats[7];   // body+0x1C = W of rotation column 1
		outZ = bodyFloats[11];  // body+0x2C = W of rotation column 2
	}
}

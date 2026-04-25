#include "Hand.h"

#include "HavokOffsets.h"

// HandGrab.cpp — Grab lifecycle methods for the ROCK Hand class.
//
// WHY: HIGGS uses dynamic constraint-based grabs — the object stays DYNAMIC and
// a ragdoll constraint with position motors pulls it to the hand. The solver moves
// the object, not our code. This gives smooth, stutter-free hold with natural
// environment interaction (object collides with walls, stacks on surfaces, etc.).
//
// ALTERNATIVES CONSIDERED:
// 1. KEYFRAMED teleport (applyTransformToNode + Update(0x2000)) — our previous approach.
//    Causes one-frame stutter, object phases through environment. Abandoned.
// 2. computeHardKeyFrame + setBodyVelocity — velocity-based positioning. Works for
//    hand colliders but not for held objects (keyframed bodies don't integrate velocity).
// 3. Dynamic constraint (this approach) — object stays DYNAMIC, ragdoll constraint
//    motors pull it to the hand target. Solver integrates smoothly. HIGGS pattern.
//
// ARCHITECTURE (HIGGS pattern, verified by 10+7+3 agent investigations):
// - Hand body (bodyA) = KEYFRAMED, positioned via SetBodyTransform + velocity
//   via computeHardKeyFrame (set AFTER teleport so solver sees hand moving)
// - Held object (bodyB) = DYNAMIC, moved ONLY by constraint motors
// - No velocity injection on bodyB — motors are the sole mover
// - Constraint pivot = mesh surface contact point (not wrist/palm center)
// - m_targetPosition=0 drives pivots together automatically
// - Angular target set at grab time maintains relative orientation
//
// CC PUSH PREVENTION:
// processConstraintsCallback hook zeros surface velocity for held body IDs.

#include "RockConfig.h"
#include "GrabConstraint.h"
#include "MeshGrab.h"
#include "PalmTransform.h"
#include "PhysicsUtils.h"
#include "api/FRIKApi.h"
#include "RockUtils.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "f4vr/F4VRUtils.h"

#include <cmath>
#include <format>
#include <xmmintrin.h>  // _mm_store_ps for atomic 16-byte writes to constraint data

namespace frik::rock
{
		namespace
		{
			RE::NiPoint3 getMatrixColumn(const RE::NiMatrix3& matrix, int column)
			{
				return RE::NiPoint3(matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column]);
			}

			RE::NiPoint3 getMatrixRow(const RE::NiMatrix3& matrix, int row)
			{
				return RE::NiPoint3(matrix.entry[row][0], matrix.entry[row][1], matrix.entry[row][2]);
			}

			const char* nodeDebugName(const RE::NiAVObject* node)
			{
				if (!node) {
					return "(null)";
				}

				const char* name = node->name.c_str();
				return name ? name : "(unnamed)";
			}

			float translationDeltaGameUnits(const RE::NiTransform& a, const RE::NiTransform& b)
			{
				const RE::NiPoint3 delta = a.translate - b.translate;
				return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
			}

			float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
			{
				const RE::NiMatrix3 delta = a.Transpose() * b;
				float cosTheta = (delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f;
				if (cosTheta < -1.0f) {
					cosTheta = -1.0f;
				} else if (cosTheta > 1.0f) {
					cosTheta = 1.0f;
				}
				return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
			}

			float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
			{
				const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
				const float lenA = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
				const float lenB = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
				if (lenA < 0.0001f || lenB < 0.0001f) {
					return -1.0f;
				}

				float cosTheta = dot / (lenA * lenB);
				if (cosTheta < -1.0f) {
					cosTheta = -1.0f;
				} else if (cosTheta > 1.0f) {
					cosTheta = 1.0f;
				}
				return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
			}

			float max3(float a, float b, float c)
			{
				return (std::max)((std::max)(a, b), c);
			}

			RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix)
			{
				RE::NiMatrix3 result{};
				result.entry[0][0] = hkMatrix[0];
				result.entry[1][0] = hkMatrix[1];
				result.entry[2][0] = hkMatrix[2];
				result.entry[0][1] = hkMatrix[4];
				result.entry[1][1] = hkMatrix[5];
				result.entry[2][1] = hkMatrix[6];
				result.entry[0][2] = hkMatrix[8];
				result.entry[1][2] = hkMatrix[9];
				result.entry[2][2] = hkMatrix[10];
				return result;
			}

			RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix)
			{
				RE::NiMatrix3 result{};
				result.entry[0][0] = hkMatrix[0];
				result.entry[0][1] = hkMatrix[1];
				result.entry[0][2] = hkMatrix[2];
				result.entry[1][0] = hkMatrix[4];
				result.entry[1][1] = hkMatrix[5];
				result.entry[1][2] = hkMatrix[6];
				result.entry[2][0] = hkMatrix[8];
				result.entry[2][1] = hkMatrix[9];
				result.entry[2][2] = hkMatrix[10];
				return result;
			}

			RE::NiTransform makeIdentityTransform()
			{
				RE::NiTransform result{};
				result.rotate.entry[0][0] = 1.0f;
				result.rotate.entry[1][1] = 1.0f;
			result.rotate.entry[2][2] = 1.0f;
			result.scale = 1.0f;
			return result;
		}

		RE::NiTransform invertTransform(const RE::NiTransform& transform)
		{
			RE::NiTransform result = makeIdentityTransform();
			result.rotate = transform.rotate.Transpose();
			result.scale = (transform.scale > 0.0001f) ? (1.0f / transform.scale) : 1.0f;

			result.translate.x = -(result.rotate.entry[0][0] * transform.translate.x +
				result.rotate.entry[0][1] * transform.translate.y +
				result.rotate.entry[0][2] * transform.translate.z) * result.scale;
			result.translate.y = -(result.rotate.entry[1][0] * transform.translate.x +
				result.rotate.entry[1][1] * transform.translate.y +
				result.rotate.entry[1][2] * transform.translate.z) * result.scale;
				result.translate.z = -(result.rotate.entry[2][0] * transform.translate.x +
					result.rotate.entry[2][1] * transform.translate.y +
					result.rotate.entry[2][2] * transform.translate.z) * result.scale;
				return result;
			}

			RE::NiTransform makeAuthoredHandTransform(const RE::NiTransform& rawHandTransform, bool isLeft)
			{
				RE::NiTransform result = rawHandTransform;
				result.rotate = authoredHandspaceRotationFromRawHandBasis(rawHandTransform.rotate);
				(void)isLeft;
				return result;
			}

		RE::NiTransform multiplyTransforms(const RE::NiTransform& a, const RE::NiTransform& b)
		{
			RE::NiTransform result = makeIdentityTransform();
			result.rotate = a.rotate * b.rotate;
			result.scale = a.scale * b.scale;

			RE::NiPoint3 rotated;
			rotated.x = a.rotate.entry[0][0] * b.translate.x + a.rotate.entry[0][1] * b.translate.y + a.rotate.entry[0][2] * b.translate.z;
			rotated.y = a.rotate.entry[1][0] * b.translate.x + a.rotate.entry[1][1] * b.translate.y + a.rotate.entry[1][2] * b.translate.z;
			rotated.z = a.rotate.entry[2][0] * b.translate.x + a.rotate.entry[2][1] * b.translate.y + a.rotate.entry[2][2] * b.translate.z;

			result.translate.x = a.translate.x + rotated.x * a.scale;
			result.translate.y = a.translate.y + rotated.y * a.scale;
			result.translate.z = a.translate.z + rotated.z * a.scale;
			return result;
		}

		RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld,
			const RE::NiTransform& bodyLocalTransform)
		{
			return multiplyTransforms(bodyWorld, invertTransform(bodyLocalTransform));
		}

		RE::NiTransform getBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
		{
			RE::NiTransform result = makeIdentityTransform();
			if (!world || bodyId.value == INVALID_BODY_ID) {
				return result;
			}

			auto* bodyArray = world->GetBodyArray();
			auto* body = reinterpret_cast<float*>(&bodyArray[bodyId.value]);
			// FO4VR native visual sync treats the live hknp body rotation as row-indexed
			// NiTransform slots, so rebuild NiMatrix3 rows directly.
			result.rotate = havokRotationBlocksToNiMatrix(body);
			result.translate.x = body[12] * kHavokToGameScale;
			result.translate.y = body[13] * kHavokToGameScale;
			result.translate.z = body[14] * kHavokToGameScale;
			return result;
		}

		RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld,
			const RE::NiTransform& bodyWorld)
		{
			RE::NiTransform result = makeIdentityTransform();
			const RE::NiMatrix3 invNodeRot = nodeWorld.rotate.Transpose();
			const float invNodeScale = (nodeWorld.scale > 0.0001f) ? (1.0f / nodeWorld.scale) : 1.0f;
			const RE::NiPoint3 delta = bodyWorld.translate - nodeWorld.translate;

			result.rotate = invNodeRot * bodyWorld.rotate;
			result.translate.x = (invNodeRot.entry[0][0] * delta.x + invNodeRot.entry[0][1] * delta.y + invNodeRot.entry[0][2] * delta.z) * invNodeScale;
			result.translate.y = (invNodeRot.entry[1][0] * delta.x + invNodeRot.entry[1][1] * delta.y + invNodeRot.entry[1][2] * delta.z) * invNodeScale;
			result.translate.z = (invNodeRot.entry[2][0] * delta.x + invNodeRot.entry[2][1] * delta.y + invNodeRot.entry[2][2] * delta.z) * invNodeScale;
			return result;
		}
	}

	// =========================================================================
	// Native VR grab drop — Ghidra: 0x140F1AB90
	// Releases any existing native grab (BSMouseSpringAction) on the object.
	// Must be called when ROCK grabs to kill lingering native springs that
	// would otherwise fight our constraint and push the player.
	// =========================================================================
	static void nativeVRGrabDrop(void* playerChar, int handIndex)
	{
		typedef void func_t(void*, int);
		static REL::Relocation<func_t> func{ REL::Offset(offsets::kFunc_NativeVRGrabDrop) };
		func(playerChar, handIndex);
	}

	static constexpr std::uint32_t kCollisionFilterNoCollideBit = 1u << 14;

	void Hand::clearGrabHandCollisionSuppressionState()
	{
		_grabHandCollisionSuppressed = false;
		_grabHandCollisionWasDisabled = false;
		_grabHandCollisionBodyId = INVALID_BODY_ID;
	}

	void Hand::suppressHandCollisionForGrab(RE::hknpWorld* world)
	{
		if (!world || !hasCollisionBody()) return;

		const auto handBodyId = _handBody.getBodyId();
		if (handBodyId.value == INVALID_BODY_ID) return;

		typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
		static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{
			REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

		auto* bodyArray = world->GetBodyArray();
		auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[handBodyId.value]);
		const auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
		const bool firstSuppression = !_grabHandCollisionSuppressed || _grabHandCollisionBodyId != handBodyId.value;

		if (firstSuppression) {
			_grabHandCollisionSuppressed = true;
			_grabHandCollisionWasDisabled = (currentFilter & kCollisionFilterNoCollideBit) != 0;
			_grabHandCollisionBodyId = handBodyId.value;
		}

		const auto disabledFilter = currentFilter | kCollisionFilterNoCollideBit;
		if (disabledFilter != currentFilter) {
			setBodyCollisionFilterInfo(world, handBodyId.value, disabledFilter, 0);
			ROCK_LOG_INFO(Hand,
				"{} hand: grab hand collision suppressed bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBeforeGrab={}",
				handName(), handBodyId.value, currentFilter, disabledFilter,
				_grabHandCollisionWasDisabled ? "yes" : "no");
		} else if (firstSuppression) {
			ROCK_LOG_INFO(Hand,
				"{} hand: grab hand collision already suppressed bodyId={} filter=0x{:08X}",
				handName(), handBodyId.value, currentFilter);
		}
	}

	void Hand::restoreHandCollisionAfterGrab(RE::hknpWorld* world)
	{
		if (!_grabHandCollisionSuppressed) return;

		if (!world || !hasCollisionBody()) {
			clearGrabHandCollisionSuppressionState();
			return;
		}

		const auto handBodyId = _handBody.getBodyId();
		if (handBodyId.value == INVALID_BODY_ID || handBodyId.value != _grabHandCollisionBodyId) {
			clearGrabHandCollisionSuppressionState();
			return;
		}

		typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
		static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{
			REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

		auto* bodyArray = world->GetBodyArray();
		auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[handBodyId.value]);
		const auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
		const auto restoredFilter = _grabHandCollisionWasDisabled
			? (currentFilter | kCollisionFilterNoCollideBit)
			: (currentFilter & ~kCollisionFilterNoCollideBit);

		if (restoredFilter != currentFilter) {
			setBodyCollisionFilterInfo(world, handBodyId.value, restoredFilter, 0);
		}

		ROCK_LOG_INFO(Hand,
			"{} hand: grab hand collision restored bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={}",
			handName(), handBodyId.value, currentFilter, restoredFilter,
			_grabHandCollisionWasDisabled ? "yes" : "no");
		clearGrabHandCollisionSuppressionState();
	}

	// =========================================================================
	// Grab Initiation — Dynamic Constraint (HIGGS pattern)
	//
	// Object stays DYNAMIC. A ragdoll constraint with position motors connects
	// the hand body (bodyA, keyframed) to the object body (bodyB, dynamic).
	// The solver pulls the object toward the hand each physics step.
	// =========================================================================

	bool Hand::grabSelectedObject(RE::hknpWorld* world,
		const RE::NiTransform& handWorldTransform,
		float tau, float damping, float maxForce,
		float proportionalRecovery, float constantRecovery)
	{
		if (!hasSelection() || !world) return false;
		if (!hasCollisionBody()) return false;  // need hand body as constraint anchor

		const auto& sel = _currentSelection;
		if (!sel.refr || sel.bodyId.value == 0x7FFF'FFFF) return false;
		if (sel.refr->IsDeleted() || sel.refr->IsDisabled()) return false;

		auto objectBodyId = sel.bodyId;

		// --- Save state ---
		auto& body = world->GetBody(objectBodyId);
		_savedObjectState.bodyId = objectBodyId;
		_savedObjectState.refr = sel.refr;
		_savedObjectState.originalFilterInfo = *reinterpret_cast<std::uint32_t*>(
			reinterpret_cast<char*>(&body) + offsets::kBody_CollisionFilterInfo);

		auto* baseObj = sel.refr->GetObjectReference();
		const char* objName = "(unnamed)";
		if (baseObj) {
			auto nameView = RE::TESFullName::GetFullName(*baseObj, false);
			if (!nameView.empty()) objName = nameView.data();
		}

		// --- Full motion state diagnostic + DYNAMIC preset conversion ---
		// WHY: In hknp, motionPropertiesId at motion+0x38 is an INDEX into the
		// motionPropertiesLibrary. Bethesda game objects use custom presets (e.g. 5)
		// that may have velocity caps or high damping preventing constraint convergence.
		// HIGGS S03 Step 4 explicitly converts to MOTION_DYNAMIC (preset 1) at grab time.
		// body+0x40 flags: bit 1(0x2)=dynamic-initialized, bit 2(0x4)=keyframed
		// body+0x72: motionPropertiesId (uint8, matches lower byte of motion uint16)
		// motion+0x38: motionPropertiesId (uint16, the actual preset index)
		// motion+0x3A: maxLinearVelocity (uint16, half-float encoded)
		// motion+0x3C: maxAngularVelocity (uint16, half-float encoded)
		const char* motionTypeStr = "UNKNOWN";
		{
			auto* objMotion = world->GetBodyMotion(objectBodyId);
			auto* bodyPtr = reinterpret_cast<char*>(&body);
			if (objMotion) {
				auto* motionPtr = reinterpret_cast<char*>(objMotion);

				// Read full state
				std::uint32_t bodyFlags = *reinterpret_cast<std::uint32_t*>(bodyPtr + 0x40);
				std::uint8_t bodyMotionPropsId = *reinterpret_cast<std::uint8_t*>(bodyPtr + 0x72);
				std::uint16_t motionPropsId = *reinterpret_cast<std::uint16_t*>(motionPtr + offsets::kMotion_PropertiesId);
				std::uint16_t maxLinVelPacked = *reinterpret_cast<std::uint16_t*>(motionPtr + 0x3A);
				std::uint16_t maxAngVelPacked = *reinterpret_cast<std::uint16_t*>(motionPtr + 0x3C);

				// Decode half-floats for max velocity (upper 16 bits of float)
				std::uint32_t linVelU32 = static_cast<std::uint32_t>(maxLinVelPacked) << 16;
				std::uint32_t angVelU32 = static_cast<std::uint32_t>(maxAngVelPacked) << 16;
				float maxLinVel, maxAngVel;
				std::memcpy(&maxLinVel, &linVelU32, sizeof(float));
				std::memcpy(&maxAngVel, &angVelU32, sizeof(float));

				bool isDynamicInit = (bodyFlags & 0x2) != 0;
				bool isKeyframed = (bodyFlags & 0x4) != 0;

				switch (motionPropsId & 0xFF) {
				case 0: motionTypeStr = "STATIC"; break;
				case 1: motionTypeStr = "DYNAMIC"; break;
				case 2: motionTypeStr = "KEYFRAMED"; break;
				default: motionTypeStr = "OTHER"; break;
				}

				ROCK_LOG_INFO(Hand, "{} hand GRAB MOTION: body={} motionPropsId={} ({}) "
					"bodyFlags=0x{:08X} dynInit={} keyfr={} bodyPropsId={} "
					"maxLinVel={:.1f} maxAngVel={:.1f}",
					handName(), objectBodyId.value, motionPropsId, motionTypeStr,
					bodyFlags, isDynamicInit, isKeyframed, bodyMotionPropsId,
					maxLinVel, maxAngVel);

				// --- One-time dump of motionPropertiesLibrary entries ---
				// Library at world+0x5D0 (hkRefPtr), array at library+0x28, entry stride=0x40.
				// Ghidra-confirmed: rebuildMotionMassProperties indexes as motionPropsId * 0x40.
				{
					static bool libraryDumped = false;
					if (!libraryDumped) {
						libraryDumped = true;
						auto* worldPtr = reinterpret_cast<char*>(world);
						auto* libraryPtr = *reinterpret_cast<char**>(worldPtr + 0x5D0);
						if (libraryPtr) {
							auto* arrayBase = *reinterpret_cast<char**>(libraryPtr + 0x28);
							// hkArray: +0x00 = data ptr (at library+0x28), +0x08 = size (at library+0x30)
							int arraySize = *reinterpret_cast<int*>(libraryPtr + 0x30);
							ROCK_LOG_INFO(Hand, "=== MOTION PROPERTIES LIBRARY: {} entries, stride=0x40 ===",
								arraySize);
							int dumpCount = (arraySize < 16) ? arraySize : 16;
							if (arrayBase) {
								for (int i = 0; i < dumpCount; i++) {
									auto* entry = arrayBase + i * 0x40;
									// Dump raw bytes as floats and ints for analysis
									auto* f = reinterpret_cast<float*>(entry);
									auto* u = reinterpret_cast<std::uint32_t*>(entry);
									ROCK_LOG_INFO(Hand,
										"  [{}] +00: {:.4f} {:.4f} {:.4f} {:.4f} | "
										"+10: {:.4f} {:.4f} {:.4f} {:.4f} | "
										"+20: {:.4f} {:.4f} {:.4f} {:.4f} | "
										"+30: {:.4f} {:.4f} {:.4f} {:.4f}",
										i,
										f[0], f[1], f[2], f[3],
										f[4], f[5], f[6], f[7],
										f[8], f[9], f[10], f[11],
										f[12], f[13], f[14], f[15]);
									ROCK_LOG_INFO(Hand,
										"  [{}] hex: {:08X} {:08X} {:08X} {:08X} | "
										"{:08X} {:08X} {:08X} {:08X}",
										i,
										u[0], u[1], u[2], u[3],
										u[4], u[5], u[6], u[7]);
								}
							}
						}
					}
				}

				_savedObjectState.originalMotionPropsId = motionPropsId;
			}
		}

		// --- A1 FIX (2026-03-31): Use bhkNPCollisionObject::SetMotionType(DYNAMIC) ---
		// WHY: Previous code called raw setBodyMotionProperties which only changed
		// the motionPropertiesId (motion+0x38) but did NOT clear the KEYFRAMED flag
		// (body+0x40 bit 0x4). With the KEYFRAMED flag still set:
		//   1. FOIslandActivationCallback checks (flags & 0x5) → non-zero → may
		//      force object back to KEYFRAMED on island activation events
		//   2. Constraint motors push against infinite mass (keyframed bodies have
		//      zeroed inverse mass at motion+0x20) → zero acceleration
		//   3. IsBodyStaticOrKeyframed checks (body+0x40 & 0x5) → returns true →
		//      game treats held object as immovable
		//
		// bhkNPCollisionObject::SetMotionType(1=DYNAMIC) at 0x141e07300 handles:
		//   - Creating new motion slot with proper mass/inertia from shape
		//   - Clearing KEYFRAMED flag, setting DYNAMIC flags in body+0x40
		//   - Walking the linked body chain via body+0x64/0x68
		//   - Computing total mass and rebuilding mass properties
		//   - Rebuilding collision caches and updating broadphase
		//
		// Access path: selected collidable node → collisionObject → bhkNPCollisionObject
		{
			auto* motionNode = sel.hitNode ? sel.hitNode : sel.refr->Get3D();
			auto* collObj = motionNode ? motionNode->collisionObject.get() : nullptr;
			if (collObj) {
				typedef void (*SetMotionType_t)(void*, int);
				static REL::Relocation<SetMotionType_t> SetMotionType{
					REL::Offset(offsets::kFunc_CollisionObject_SetMotionType) };
				SetMotionType(collObj, 1);  // 1 = DYNAMIC

				// Verify the transition took effect
				auto* objMotion = world->GetBodyMotion(objectBodyId);
				auto* bodyPtr = reinterpret_cast<char*>(&body);
				if (objMotion) {
					std::uint32_t newFlags = *reinterpret_cast<std::uint32_t*>(bodyPtr + 0x40);
					std::uint16_t newPropsId = *reinterpret_cast<std::uint16_t*>(
						reinterpret_cast<char*>(objMotion) + offsets::kMotion_PropertiesId);
					bool isDynamic = (newFlags & 0x4) == 0;  // KEYFRAMED bit cleared
					ROCK_LOG_INFO(Hand, "{} hand SetMotionType(DYNAMIC): flags=0x{:08X} "
						"propsId={} isDynamic={} (KEYFRAMED bit {})",
						handName(), newFlags, newPropsId, isDynamic,
						(newFlags & 0x4) ? "STILL SET" : "cleared");
				}
			} else {
				ROCK_LOG_WARN(Hand, "{} hand: no collision object on refr — "
					"cannot call SetMotionType, using raw motionPropsId fallback",
					handName());
				// Fallback: raw motionPropertiesId change (less correct but functional)
				auto* objMotion = world->GetBodyMotion(objectBodyId);
				if (objMotion && _savedObjectState.originalMotionPropsId != 1) {
					typedef void setMotionProps_t(void*, int, short);
					static REL::Relocation<setMotionProps_t> setBodyMotionProperties{
						REL::Offset(offsets::kFunc_SetBodyMotionProperties) };
					setBodyMotionProperties(world, objectBodyId.value, 1);
				}
			}
		}

		ROCK_LOG_INFO(Hand, "{} hand GRAB: '{}' formID={:08X} bodyId={}",
			handName(), objName, sel.refr->GetFormID(), objectBodyId.value);

		// In-game notification so the user can see grab info in VR
		{
			auto msg = std::format("[ROCK] {} GRAB: {} ({})",
				_isLeft ? "L" : "R", objName, motionTypeStr);
			f4vr::showNotification(msg);
		}

		// Collect held body IDs for CC push prevention (processConstraintsCallback hook)
		collectHeldBodyIds(sel.refr);

		// Kill any native VR grab springs (native grab also blocked via Xbyak patch)
		{
			auto* player = RE::PlayerCharacter::GetSingleton();
			if (player) {
				nativeVRGrabDrop(player, 0);
				nativeVRGrabDrop(player, 1);
			}
		}

		// --- Resolve the node roles used by the selection/grab pipeline ---
		// WHY: HIGGS uses the selected collidable as the physics frame but extracts
		// geometry from the object root. FO4VR can report a body owner node that is
		// not the best visual mesh seed, so ROCK must preserve both roles.
		auto* rootNode = sel.refr->Get3D();
		auto* collidableNode = sel.hitNode ? sel.hitNode : rootNode;
		auto* meshSourceNode = sel.visualNode ? sel.visualNode : rootNode;
		if (!meshSourceNode) {
			meshSourceNode = collidableNode;
		}
		RE::NiTransform objectWorldTransform;
		if (collidableNode) {
			objectWorldTransform = collidableNode->world;
		} else {
			objectWorldTransform = handWorldTransform;
		}

		static bool s_runtimeScaleLogged = false;
		if (!s_runtimeScaleLogged) {
			auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
			const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;
			ROCK_LOG_INFO(Hand,
				"RUNTIME SCALE {}: handScale={:.3f} collidableScale={:.3f} vrScale={:.3f}",
				handName(), handWorldTransform.scale, collidableNode ? collidableNode->world.scale : -1.0f, vrScale);
			s_runtimeScaleLogged = true;
		}

		// --- Mesh-based grab point alignment ---
		RE::NiPoint3 grabSurfacePoint = objectWorldTransform.translate;
		bool meshGrabFound = false;
		MeshExtractionStats meshStats;
		const char* grabPointMode = "objectNodeOriginFallback";
		const char* grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";

		if (meshSourceNode) {
			std::vector<TriangleData> triangles;
			extractAllTriangles(meshSourceNode, triangles, 10, &meshStats);

			ROCK_LOG_INFO(Hand,
				"{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} "
				"static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} totalTris={}",
				handName(), nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode),
				meshStats.visitedShapes,
				meshStats.staticShapes, meshStats.staticTriangles,
				meshStats.dynamicShapes, meshStats.dynamicTriangles,
				meshStats.skinnedShapes, meshStats.skinnedTriangles,
				meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes,
				meshStats.totalTriangles());

			// --- Vertex format diagnostic for first BSTriShape in subtree ---
			// Log vertexDesc fields to verify FP16/FP32 detection is working.
			{
				RE::BSTriShape* firstTriShape = meshSourceNode->IsTriShape();
				if (!firstTriShape) {
					auto* meshNode = meshSourceNode->IsNode();
					if (meshNode) {
						auto& kids = meshNode->GetRuntimeData().children;
						const auto childCount = kids.size();
						for (auto ci = decltype(childCount){ 0 }; ci < childCount; ci++) {
							auto* kid = kids[ci].get();
							if (kid && kid->IsTriShape()) {
								firstTriShape = kid->IsTriShape();
								break;
							}
						}
					}
				}
				if (firstTriShape) {
					auto* tsBase = reinterpret_cast<char*>(firstTriShape);
					std::uint64_t vtxDesc = *reinterpret_cast<std::uint64_t*>(tsBase + VROffset::vertexDesc);
					std::uint32_t stride = static_cast<std::uint32_t>(vtxDesc & 0xF) * 4;
					std::uint32_t posOff = static_cast<std::uint32_t>((vtxDesc >> 2) & 0x3C);
					bool fullPrec = ((vtxDesc >> 54) & 1) != 0;
					std::uint8_t geomType = *reinterpret_cast<std::uint8_t*>(tsBase + 0x198);
					void* skinInst = *reinterpret_cast<void**>(tsBase + VROffset::skinInstance);

					ROCK_LOG_INFO(MeshGrab,
						"VertexDiag '{}': vtxDesc=0x{:016X} stride={} posOffset={} fullPrec={} geomType={} skinned={}",
						firstTriShape->name.c_str() ? firstTriShape->name.c_str() : "(null)",
						vtxDesc, stride, posOff, fullPrec ? 1 : 0,
						geomType, skinInst ? 1 : 0);
				}
			}

			if (!triangles.empty()) {
				// Simplified centroid distance diagnostic
				auto& t0 = triangles[0];
				float cx = (t0.v0.x + t0.v1.x + t0.v2.x) / 3.0f;
				float cy = (t0.v0.y + t0.v1.y + t0.v2.y) / 3.0f;
				float cz = (t0.v0.z + t0.v1.z + t0.v2.z) / 3.0f;

				auto* bodyArray = world->GetBodyArray();
				auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
				float distToBody = std::sqrt(
					(cx - objFloats[12]*70.0f) * (cx - objFloats[12]*70.0f) +
					(cy - objFloats[13]*70.0f) * (cy - objFloats[13]*70.0f) +
					(cz - objFloats[14]*70.0f) * (cz - objFloats[14]*70.0f));

				ROCK_LOG_INFO(MeshGrab, "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToBody={:.1f}gu",
					cx, cy, cz, distToBody);
			}

			if (!triangles.empty()) {
				RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
				RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);

				GrabPoint grabPt;
				if (findClosestGrabPoint(triangles, palmPos, palmDir, 1.0f, 1.0f, grabPt)) {
					grabSurfacePoint = grabPt.position;
					meshGrabFound = true;
					grabPointMode = "meshSurface";
					grabFallbackReason = "none";
					ROCK_LOG_INFO(Hand,
						"{} hand MESH GRAB: mode={} tris={} staticTris={} dynamicTris={} skinnedTris={} "
						"closest=({:.1f},{:.1f},{:.1f})",
						handName(), grabPointMode, triangles.size(),
						meshStats.staticTriangles, meshStats.dynamicTriangles, meshStats.skinnedTriangles,
						grabPt.position.x, grabPt.position.y, grabPt.position.z);
				} else {
					grabFallbackReason = "noClosestSurfacePoint";
				}
			} else {
				grabFallbackReason = "noTriangles";
			}
		}

		if (!meshGrabFound) {
			ROCK_LOG_WARN(Hand,
				"{} hand GRAB POINT FALLBACK: mode={} reason={} meshNode='{}' ownerNode='{}' rootNode='{}' "
				"shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} "
				"fallbackPoint=({:.1f},{:.1f},{:.1f})",
				handName(), grabPointMode, grabFallbackReason,
				nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode),
				meshStats.visitedShapes,
				meshStats.staticShapes, meshStats.staticTriangles,
				meshStats.dynamicShapes, meshStats.dynamicTriangles,
				meshStats.skinnedShapes, meshStats.skinnedTriangles,
				meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes,
				grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
		}

		_grabStartTime = 0.0f;

		// Capture the visual and physics grab transforms separately.
		// _grabHandSpace stays in the raw hand-node basis for visible-hand reversal.
		// _grabConstraintHandSpace uses the authored ROCK hand basis so the
		// constraint target shares the same local frame as the hand body / palm math.
		{
			const RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
			RE::NiTransform shiftedObject = objectWorldTransform;
			shiftedObject.translate.x += palmPos.x - grabSurfacePoint.x;
			shiftedObject.translate.y += palmPos.y - grabSurfacePoint.y;
			shiftedObject.translate.z += palmPos.z - grabSurfacePoint.z;

			// handInverse = Invert(handWorldTransform)
			const RE::NiMatrix3 invR = handWorldTransform.rotate.Transpose();
			const float invS = (handWorldTransform.scale > 0.0001f) ? (1.0f / handWorldTransform.scale) : 1.0f;
			RE::NiPoint3 invP;
			invP.x = -(invR.entry[0][0] * handWorldTransform.translate.x +
				invR.entry[0][1] * handWorldTransform.translate.y +
				invR.entry[0][2] * handWorldTransform.translate.z) * invS;
			invP.y = -(invR.entry[1][0] * handWorldTransform.translate.x +
				invR.entry[1][1] * handWorldTransform.translate.y +
				invR.entry[1][2] * handWorldTransform.translate.z) * invS;
			invP.z = -(invR.entry[2][0] * handWorldTransform.translate.x +
				invR.entry[2][1] * handWorldTransform.translate.y +
				invR.entry[2][2] * handWorldTransform.translate.z) * invS;

			// _grabHandSpace = handInverse * shiftedObject (visual/collidable node path)
			// result.rot = invR * shifted.rot
			// result.pos = invP + (invR * shifted.pos) * invS
			_grabHandSpace.rotate = invR * shiftedObject.rotate;
			_grabHandSpace.scale = invS * shiftedObject.scale;
			RE::NiPoint3 rotatedShiftedPos;
			rotatedShiftedPos.x = invR.entry[0][0] * shiftedObject.translate.x +
				invR.entry[0][1] * shiftedObject.translate.y +
				invR.entry[0][2] * shiftedObject.translate.z;
			rotatedShiftedPos.y = invR.entry[1][0] * shiftedObject.translate.x +
				invR.entry[1][1] * shiftedObject.translate.y +
				invR.entry[1][2] * shiftedObject.translate.z;
			rotatedShiftedPos.z = invR.entry[2][0] * shiftedObject.translate.x +
				invR.entry[2][1] * shiftedObject.translate.y +
				invR.entry[2][2] * shiftedObject.translate.z;
			_grabHandSpace.translate.x = invP.x + rotatedShiftedPos.x * invS;
			_grabHandSpace.translate.y = invP.y + rotatedShiftedPos.y * invS;
			_grabHandSpace.translate.z = invP.z + rotatedShiftedPos.z * invS;

			const RE::NiTransform liveBodyWorldAtGrab = getBodyWorldTransform(world, objectBodyId);
			auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
			auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
			auto* bodyCollisionObjectAtGrab = bhkWorldAtGrab
				? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId)
				: nullptr;
			auto* ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
			_heldNode = collidableNode;
			_grabBodyLocalTransform = collidableNode
				? computeRuntimeBodyLocalTransform(objectWorldTransform, liveBodyWorldAtGrab)
				: makeIdentityTransform();
			_grabOwnerBodyLocalTransform = ownerNodeAtGrab
				? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, liveBodyWorldAtGrab)
				: makeIdentityTransform();
			_grabRootBodyLocalTransform = rootNode
				? computeRuntimeBodyLocalTransform(rootNode->world, liveBodyWorldAtGrab)
				: makeIdentityTransform();

			const RE::NiTransform authoredHandTransform = makeAuthoredHandTransform(handWorldTransform, _isLeft);
			_grabConstraintHandSpace = multiplyTransforms(invertTransform(authoredHandTransform), shiftedObject);

			if (g_rockConfig.rockDebugGrabFrameLogging) {
				const RE::NiTransform& constraintGrabHandSpace = _grabConstraintHandSpace;

				const RE::NiPoint3 rawLateral = getMatrixColumn(handWorldTransform.rotate, 0);
				const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 1);
				const RE::NiPoint3 rawBack = getMatrixColumn(handWorldTransform.rotate, 2);
				const RE::NiPoint3 constraintFinger = getMatrixColumn(authoredHandTransform.rotate, 0);
				const RE::NiPoint3 constraintBack = getMatrixColumn(authoredHandTransform.rotate, 1);
				const RE::NiPoint3 constraintLateral = getMatrixColumn(authoredHandTransform.rotate, 2);
				const RE::NiPoint3 grabSpaceRawFinger = getMatrixColumn(_grabHandSpace.rotate, 0);
				const RE::NiPoint3 grabSpaceConstraintFinger = getMatrixColumn(constraintGrabHandSpace.rotate, 0);
				const RE::NiPoint3 grabPosDelta = constraintGrabHandSpace.translate - _grabHandSpace.translate;

				ROCK_LOG_INFO(Hand,
					"{} GRAB FRAME SNAPSHOT: rawVsConstraint rotDelta={:.2f}deg posDelta=({:.2f},{:.2f},{:.2f}) "
					"rawFinger=({:.3f},{:.3f},{:.3f}) rawBack=({:.3f},{:.3f},{:.3f}) rawLat=({:.3f},{:.3f},{:.3f}) "
					"constraintFinger=({:.3f},{:.3f},{:.3f}) constraintBack=({:.3f},{:.3f},{:.3f}) constraintLat=({:.3f},{:.3f},{:.3f})",
					handName(),
					rotationDeltaDegrees(_grabHandSpace.rotate, constraintGrabHandSpace.rotate),
					grabPosDelta.x, grabPosDelta.y, grabPosDelta.z,
					rawFinger.x, rawFinger.y, rawFinger.z,
					rawBack.x, rawBack.y, rawBack.z,
					rawLateral.x, rawLateral.y, rawLateral.z,
					constraintFinger.x, constraintFinger.y, constraintFinger.z,
					constraintBack.x, constraintBack.y, constraintBack.z,
					constraintLateral.x, constraintLateral.y, constraintLateral.z);

				ROCK_LOG_INFO(Hand,
					"{} GRAB FRAME TARGETS: grabHSRaw.pos=({:.2f},{:.2f},{:.2f}) grabHSConstraint.pos=({:.2f},{:.2f},{:.2f}) "
					"grabHSRawFinger=({:.3f},{:.3f},{:.3f}) grabHSConstraintFinger=({:.3f},{:.3f},{:.3f}) "
					"bodyLocal.pos=({:.2f},{:.2f},{:.2f}) bodyLocalFinger=({:.3f},{:.3f},{:.3f})",
					handName(),
					_grabHandSpace.translate.x, _grabHandSpace.translate.y, _grabHandSpace.translate.z,
					constraintGrabHandSpace.translate.x, constraintGrabHandSpace.translate.y, constraintGrabHandSpace.translate.z,
					grabSpaceRawFinger.x, grabSpaceRawFinger.y, grabSpaceRawFinger.z,
					grabSpaceConstraintFinger.x, grabSpaceConstraintFinger.y, grabSpaceConstraintFinger.z,
					_grabBodyLocalTransform.translate.x, _grabBodyLocalTransform.translate.y, _grabBodyLocalTransform.translate.z,
					_grabBodyLocalTransform.rotate.entry[0][0], _grabBodyLocalTransform.rotate.entry[1][0], _grabBodyLocalTransform.rotate.entry[2][0]);

				const RE::NiPoint3 rootBodyLocalFinger = getMatrixColumn(_grabRootBodyLocalTransform.rotate, 0);
				const RE::NiPoint3 ownerBodyLocalFinger = getMatrixColumn(_grabOwnerBodyLocalTransform.rotate, 0);
				ROCK_LOG_INFO(Hand,
					"{} GRAB NODE FRAMES: owner='{}'({:p}) hasCol={} ownsBodyCol={} "
					"root='{}'({:p}) held='{}'({:p}) mesh='{}'({:p}) sameOwnerHeld={} sameRootHeld={} "
					"ownerBodyLocal.pos=({:.2f},{:.2f},{:.2f}) ownerBodyLocalFinger=({:.3f},{:.3f},{:.3f}) "
					"rootBodyLocal.pos=({:.2f},{:.2f},{:.2f}) rootBodyLocalFinger=({:.3f},{:.3f},{:.3f})",
					handName(),
					nodeDebugName(ownerNodeAtGrab), static_cast<const void*>(ownerNodeAtGrab),
					(ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get()) ? "yes" : "no",
					(ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get() == bodyCollisionObjectAtGrab) ? "yes" : "no",
					nodeDebugName(rootNode), static_cast<const void*>(rootNode),
					nodeDebugName(collidableNode), static_cast<const void*>(collidableNode),
					nodeDebugName(meshSourceNode), static_cast<const void*>(meshSourceNode),
					ownerNodeAtGrab == collidableNode ? "yes" : "no",
					rootNode == collidableNode ? "yes" : "no",
					_grabOwnerBodyLocalTransform.translate.x,
					_grabOwnerBodyLocalTransform.translate.y,
					_grabOwnerBodyLocalTransform.translate.z,
					ownerBodyLocalFinger.x, ownerBodyLocalFinger.y, ownerBodyLocalFinger.z,
					_grabRootBodyLocalTransform.translate.x,
					_grabRootBodyLocalTransform.translate.y,
					_grabRootBodyLocalTransform.translate.z,
					rootBodyLocalFinger.x, rootBodyLocalFinger.y, rootBodyLocalFinger.z);
			}

			ROCK_LOG_INFO(Hand, "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
				"palmPos=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
				handName(),
				_grabHandSpace.translate.x, _grabHandSpace.translate.y, _grabHandSpace.translate.z,
				palmPos.x, palmPos.y, palmPos.z,
				grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
			ROCK_LOG_INFO(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}",
				handName(),
				_grabBodyLocalTransform.translate.x,
				_grabBodyLocalTransform.translate.y,
				_grabBodyLocalTransform.translate.z,
				_grabBodyLocalTransform.scale);
		}

		// --- Log body positions + COM for constraint debugging ---
		// B8 DIAG: Log both body origin and COM to measure the difference.
		// Constraint pivots use body origin (current math correct).
		// COM differs from body origin by R * comOffset for asymmetric objects.
		{
			auto* bodyArray = world->GetBodyArray();
			auto* handFloats = reinterpret_cast<float*>(&bodyArray[_handBody.getBodyId().value]);
			auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
			ROCK_LOG_INFO(Hand, "{} DIAG: handBody pos=({:.3f},{:.3f},{:.3f}) objBody pos=({:.3f},{:.3f},{:.3f})",
				handName(),
				handFloats[12], handFloats[13], handFloats[14],
				objFloats[12], objFloats[13], objFloats[14]);
			ROCK_LOG_INFO(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})",
				handName(),
				handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z,
				objectWorldTransform.translate.x, objectWorldTransform.translate.y, objectWorldTransform.translate.z);

			// B8: COM vs body origin diagnostic
			float comX, comY, comZ;
			if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ)) {
				float comOffX, comOffY, comOffZ;
				getBodyCOMOffset(objFloats, comOffX, comOffY, comOffZ);
				float comDist = std::sqrt(
					(comX - objFloats[12])*(comX - objFloats[12]) +
					(comY - objFloats[13])*(comY - objFloats[13]) +
					(comZ - objFloats[14])*(comZ - objFloats[14]));
				ROCK_LOG_INFO(Hand, "{} B8 COM: com=({:.3f},{:.3f},{:.3f}) origin=({:.3f},{:.3f},{:.3f}) "
					"offset=({:.4f},{:.4f},{:.4f}) dist={:.4f} ({:.1f}gu)",
					handName(), comX, comY, comZ,
					objFloats[12], objFloats[13], objFloats[14],
					comOffX, comOffY, comOffZ,
					comDist, comDist * 70.0f);
			}
		}

		// --- Zero object velocity before constraint creation ---
		// WHY: The object may have residual velocity (falling, bumped, etc.).
		// Without zeroing, the constraint fights existing momentum → spinning/orbiting.
		// HIGGS does this implicitly by damping nearby objects at grab time.
		// B1/B2 FIX: Use deferred-safe velocity wrapper.
		{
			alignas(16) float zeroVel[4] = { 0, 0, 0, 0 };
			typedef void (*setVelDeferred_t)(void*, std::uint32_t, const float*, const float*);
			static REL::Relocation<setVelDeferred_t> setBodyVelocityDeferred{
				REL::Offset(offsets::kFunc_SetBodyVelocityDeferred) };
			setBodyVelocityDeferred(world, objectBodyId.value, zeroVel, zeroVel);

			// Zero velocity on ALL held bodies (weapon children: scope, barrel, stock)
			for (auto bid : _heldBodyIds) {
				if (bid != objectBodyId.value) {
					setBodyVelocityDeferred(world, bid, zeroVel, zeroVel);
				}
			}
		}

		// --- Disable hand collision while holding ---
		// WHY: The hand collider (layer 43) collides with the held object (layer 5),
		// pushing it AWAY from the constraint target. The constraint pulls it back →
		// oscillation/spinning. HIGGS disables hand collision entirely while holding
		// (hand.cpp:662: filterInfo |= (1 << 14)) then calls updateCollisionFilterOnEntity.
		// CRITICAL: Must use setBodyCollisionFilterInfo API (not direct memory write)
		// so hknp rebuilds broadphase collision caches. Without rebuild, the physics
		// system never learns the filter changed and collisions continue.
		suppressHandCollisionForGrab(world);

		// --- Normalize object inertia for stable constraint grab ---
		// WHY: Without normalization, weapons with high inertia ratios cartwheel
		// around the hand. HIGGS clamps inertia ratio to 10x across axes.
		normalizeGrabbedInertia(world, objectBodyId, _savedObjectState);

		// --- Create constraint (object stays DYNAMIC) ---
		// Phase 0B fix: HIGGS uses SEPARATE pivot points for hand (palmPos) and object (grabPoint).
		// PivotA = palm center in Havok coords (hand-side anchor)
		// PivotB = grab surface point in Havok coords (object-side anchor)
		// The motor with m_targetPosition=0 drives pivotA_world == pivotB_world.
		// HIGGS S00 section 0.1a: pivotA = palmPos * havokWorldScale
		// HIGGS S00 section 0.1b: pivotB = ptPos * havokWorldScale
		//
		// TransformB rotation uses Inverse(_grabConstraintHandSpace * _grabBodyLocalTransform).
		// This matches the authored hand-space contract used by the hand body and palm math.
		{
			RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
			constexpr float INV_HAVOK_SCALE = 1.0f / 70.0f;
			const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);

			// PivotA world position = palm center in Havok coords
			float palmHk[4];
			palmHk[0] = palmPos.x * INV_HAVOK_SCALE;
			palmHk[1] = palmPos.y * INV_HAVOK_SCALE;
			palmHk[2] = palmPos.z * INV_HAVOK_SCALE;
			palmHk[3] = 0.0f;

			// PivotB world position = grab surface point in Havok coords
			float grabHk[4];
			grabHk[0] = grabSurfacePoint.x * INV_HAVOK_SCALE;
			grabHk[1] = grabSurfacePoint.y * INV_HAVOK_SCALE;
			grabHk[2] = grabSurfacePoint.z * INV_HAVOK_SCALE;
			grabHk[3] = 0.0f;

			// DIAGNOSTIC: Log grab-time positions and distances
			{
				float palmToGrab = std::sqrt(
					(palmHk[0]-grabHk[0])*(palmHk[0]-grabHk[0]) +
					(palmHk[1]-grabHk[1])*(palmHk[1]-grabHk[1]) +
					(palmHk[2]-grabHk[2])*(palmHk[2]-grabHk[2]));
				float palmToHandOrigin = std::sqrt(
					(palmPos.x - handWorldTransform.translate.x)*(palmPos.x - handWorldTransform.translate.x) +
					(palmPos.y - handWorldTransform.translate.y)*(palmPos.y - handWorldTransform.translate.y) +
					(palmPos.z - handWorldTransform.translate.z)*(palmPos.z - handWorldTransform.translate.z));
				ROCK_LOG_INFO(Hand, "GRAB DIAG {}: palmPos=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
					"grabSurface=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
					"palmToGrab_hk={:.4f} ({:.1f} game units) palmToHandOrigin={:.1f} game units",
					handName(),
					palmPos.x, palmPos.y, palmPos.z,
					handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z,
					grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z,
					meshGrabFound, grabPointMode, grabFallbackReason,
					palmToGrab, palmToGrab * 70.0f, palmToHandOrigin);
			}

			_activeConstraint = createGrabConstraint(
				world, _handBody.getBodyId(), objectBodyId,
				palmHk, grabHk, desiredBodyTransformHandSpace,
				tau, damping, maxForce,
				proportionalRecovery, constantRecovery);
		}

		if (!_activeConstraint.isValid()) {
			ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: constraint creation failed", handName());
			restoreHandCollisionAfterGrab(world);
			_savedObjectState.clear();
			_heldBodyIds.clear();
			return false;
		}

		// --- Log constraint data for debugging ---
		{
			auto* cd = static_cast<char*>(_activeConstraint.constraintData);
			float* pivotA = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
			float* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);
			float* tA_col0 = reinterpret_cast<float*>(cd + offsets::kTransformA_Col0);
			float* tB_col0 = reinterpret_cast<float*>(cd + offsets::kTransformB_Col0);
			ROCK_LOG_INFO(Hand, "{} DIAG: pivotA=({:.3f},{:.3f},{:.3f}) pivotB=({:.3f},{:.3f},{:.3f})",
				handName(),
				pivotA[0], pivotA[1], pivotA[2],
				pivotB[0], pivotB[1], pivotB[2]);
			ROCK_LOG_INFO(Hand, "{} DIAG: tA_col0=({:.3f},{:.3f},{:.3f}) tB_col0=({:.3f},{:.3f},{:.3f})",
				handName(),
				tA_col0[0], tA_col0[1], tA_col0[2],
				tB_col0[0], tB_col0[1], tB_col0[2]);
			ROCK_LOG_INFO(Hand, "{} DIAG: tau={:.3f} damping={:.2f} force={:.0f} propRecov={:.1f} constRecov={:.1f}",
				handName(), tau, damping, maxForce, proportionalRecovery, constantRecovery);
		}

		ROCK_LOG_INFO(Hand, "{} hand CONSTRAINT GRAB: constraintId={}, hand body={}, obj body={}, {} held bodies",
			handName(), _activeConstraint.constraintId,
			_handBody.getBodyId().value, objectBodyId.value, _heldBodyIds.size());

		// --- Phase 2A: Enable contact event flag on ALL held bodies ---
		// Ghidra: FUN_141e54750 shows body flag 0x80 must be enabled for contact events.
		// HIGGS S10: registers listener on all connected bodies, not just primary.
		{
			typedef void enableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
			static REL::Relocation<enableBodyFlags_t> enableBodyFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
			for (auto bid : _heldBodyIds) {
				enableBodyFlags(world, bid, 0x80, 0);
			}
		}
		_heldBodyContactFrame.store(100, std::memory_order_release);  // Reset: no contact yet

		// --- Publish thread-safe snapshot of held body IDs ---
		// Physics thread reads this via isHeldBodyId() and isHoldingAtomic().
		// Pattern: write array first, then publish count with release ordering.
		{
			int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
			for (int i = 0; i < count; i++) {
				_heldBodyIdsSnapshot[i] = _heldBodyIds[i];
			}
			_heldBodyIdsCount.store(count, std::memory_order_release);
			_isHoldingFlag.store(true, std::memory_order_release);
		}

		// --- Finger pose + state ---
		stopSelectionHighlight();
		frik::api::FRIKApi::inst->setHandPoseWithPriority("ROCK_Grab", handFromBool(_isLeft),
			frik::api::FRIKApi::HandPoses::Fist, 100);

		_state = HandState::HeldInit;

		ROCK_LOG_INFO(Hand, "{} hand GRAB SUCCESS → HeldInit: bodyId={}", handName(), objectBodyId.value);
		return true;
	}

	// =========================================================================
	// Per-Frame Held Update
	//
	// PivotA (palm in hand body-local) is set at grab time and constant.
	// PivotB and angular target (m_target_bRca) are updated per-frame from
		// the frozen physics grab-space transform.
	// Motor PARAMETERS (tau, force, damping) are updated for fade-in and
	// collision-adaptive behavior.
	// =========================================================================

	void Hand::updateHeldObject(RE::hknpWorld* world,
		const RE::NiTransform& handWorldTransform, float deltaTime,
		float forceFadeInTime, float tauMin, float tauMax,
		float tauIncrement, float tauDecrement,
		float closeThreshold, float farThreshold)
	{
		(void)handWorldTransform;
		(void)tauMax; (void)tauIncrement; (void)tauDecrement;
		(void)closeThreshold; (void)farThreshold;

		if (!isHolding() || !world) return;
		if (!_activeConstraint.isValid()) {
			releaseGrabbedObject(world);
			return;
		}

		if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() ||
			_savedObjectState.refr->IsDisabled()) {
			releaseGrabbedObject(world);
			return;
		}

		// Match HIGGS' held-state behavior: the grabbing hand remains no-collide
		// for the full hold, not only at the instant the constraint is created.
		suppressHandCollisionForGrab(world);

		_grabStartTime += deltaTime;

		// =====================================================================
		// Per-frame: update angular target and pivotB from frozen physics grab transform
		// composed with the stored body-local transform term.
		// =====================================================================
		if (_activeConstraint.constraintData) {
			auto* cd = static_cast<char*>(_activeConstraint.constraintData);
			const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);

			// Compute Inverse(desiredBodyTransformHandSpace)
			// desiredHandTransformHavokObjSpace = Invert(_grabConstraintHandSpace * _grabBodyLocalTransform)
			// Rotation inverse = Transpose (orthonormal matrix)
			RE::NiMatrix3 invRot = desiredBodyTransformHandSpace.rotate.Transpose();
			float invScale = (desiredBodyTransformHandSpace.scale > 0.0001f) ? (1.0f / desiredBodyTransformHandSpace.scale) : 1.0f;

			// Inverse translation: -(R^T * pos) / scale
			RE::NiPoint3 invPos;
			invPos.x = -(invRot.entry[0][0] * desiredBodyTransformHandSpace.translate.x +
				invRot.entry[0][1] * desiredBodyTransformHandSpace.translate.y +
				invRot.entry[0][2] * desiredBodyTransformHandSpace.translate.z) * invScale;
			invPos.y = -(invRot.entry[1][0] * desiredBodyTransformHandSpace.translate.x +
				invRot.entry[1][1] * desiredBodyTransformHandSpace.translate.y +
				invRot.entry[1][2] * desiredBodyTransformHandSpace.translate.z) * invScale;
			invPos.z = -(invRot.entry[2][0] * desiredBodyTransformHandSpace.translate.x +
				invRot.entry[2][1] * desiredBodyTransformHandSpace.translate.y +
				invRot.entry[2][2] * desiredBodyTransformHandSpace.translate.z) * invScale;

			// Phase 0B.5: Update angular target (m_target_bRca) at constraint+0xD0
			// = Inverse(_grabConstraintHandSpace * _grabBodyLocalTransform).rotation = invRot
			// Since transformA.rotation = identity: m_target_bRca = invRot
			//
			// THREAD SAFETY (C3 fix): The Havok solver may be reading this constraint
			// data on the physics thread via SIMD _mm_load_ps (16-byte aligned loads).
			// Using _mm_store_ps produces a single 16-byte store instruction which is
			// atomic on x86-64 when 16-byte aligned (Intel SDM Vol 3A §8.2.3.1).
			// Havok heap allocs are 16-byte aligned, and each column is at a 16-byte
			// boundary within the atom layout, so no torn reads are possible.
			{
				auto* target = reinterpret_cast<float*>(cd + ATOM_RAGDOLL_MOT + 0x10);
				// NiMatrix3 is row-major. Havok hkMatrix3 is column-major (16 bytes per column).
				// We write invRot in Havok column-major format:
				// col0 = [invRot[0][0], invRot[1][0], invRot[2][0], 0]
				// col1 = [invRot[0][1], invRot[1][1], invRot[2][1], 0]
				// col2 = [invRot[0][2], invRot[1][2], invRot[2][2], 0]
				alignas(16) float col0[4] = { invRot.entry[0][0], invRot.entry[1][0], invRot.entry[2][0], 0.0f };
				alignas(16) float col1[4] = { invRot.entry[0][1], invRot.entry[1][1], invRot.entry[2][1], 0.0f };
				alignas(16) float col2[4] = { invRot.entry[0][2], invRot.entry[1][2], invRot.entry[2][2], 0.0f };
				_mm_store_ps(target + 0, _mm_load_ps(col0));
				_mm_store_ps(target + 4, _mm_load_ps(col1));
				_mm_store_ps(target + 8, _mm_load_ps(col2));
			}

			// Per-frame pivotB update from frozen _grabConstraintHandSpace + hand-space palm basis.
			// This must use the same hand-local basis as grab creation; otherwise pivotB
			// jumps on the first held frame even when the hand transform itself is stable.
			{
				RE::NiPoint3 palmHS = mirrorHandspaceZ(g_rockConfig.rockPalmPositionHandspace, _isLeft) * desiredBodyTransformHandSpace.scale;

				// Transform palmHS through desiredHandTransformHavokObjSpace:
				// result = invRot * palmHS * invScale + invPos
				RE::NiPoint3 transformed;
				transformed.x = (invRot.entry[0][0]*palmHS.x + invRot.entry[0][1]*palmHS.y + invRot.entry[0][2]*palmHS.z) * invScale + invPos.x;
				transformed.y = (invRot.entry[1][0]*palmHS.x + invRot.entry[1][1]*palmHS.y + invRot.entry[1][2]*palmHS.z) * invScale + invPos.y;
				transformed.z = (invRot.entry[2][0]*palmHS.x + invRot.entry[2][1]*palmHS.y + invRot.entry[2][2]*palmHS.z) * invScale + invPos.z;

				// Apply object scale and Havok scale
				float objScale = (_heldNode ? _heldNode->world.scale : 1.0f);
				constexpr float kHavokScale = 1.0f / 70.0f;
				float combinedScale = objScale * kHavokScale;
				auto* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);

				// Atomic 16-byte write
				alignas(16) float newPivotB[4] = {
					transformed.x * combinedScale,
					transformed.y * combinedScale,
					transformed.z * combinedScale,
					0.0f
				};
				_mm_store_ps(pivotB, _mm_load_ps(newPivotB));
			}

			// Keep transformB.rotation fixed at grab-time body space.
			// HIGGS only updates the angular target and transformB.translation
			// during hold. Rewriting transformB.rotation every frame mutates the
			// live solver frame instead of only changing the angular target.
		}

		// =====================================================================
		// Phase 0B.6: Motor parameters — HIGGS defaults, not compensating values
		// Force ratio changed from 17:1 to 12.5:1 (HIGGS S04 section 4.4)
		// Tau from config (should be 0.03 after INI reset, not 1.0)
		// Recovery re-applied per-frame from config (HIGGS S05 5.8)
		// minForce = -maxForce per-frame (HIGGS S05 5.8a)
		// =====================================================================
		float targetForce = _activeConstraint.targetMaxForce;
		float currentLinearForce = 0.0f;
		float currentAngularForce = 0.0f;
		// HIGGS uses separate object angular/linear tau targets. The old ROCK path
		// drove every ordinary object toward actor tau (fGrabTauMax=0.8), which made
		// the runtime log look "strong" while deviating from the non-actor constraint
		// contract that HIGGS uses for clutter and weapons.
		const bool heldBodyColliding = isHeldBodyColliding();
		const float angularTauTarget = heldBodyColliding ? tauMin : g_rockConfig.rockGrabAngularTau;
		const float linearTauTarget = heldBodyColliding ? tauMin : g_rockConfig.rockGrabLinearTau;
		const float tauStep = g_rockConfig.rockGrabTauLerpSpeed * deltaTime;
		auto advanceTau = [tauStep](float current, float target) {
			const float delta = target - current;
			if (std::abs(delta) > tauStep) {
				return current + (delta > 0.0f ? tauStep : -tauStep);
			}
			return target;
		};
		const float currentAngularTau = advanceTau(
			_activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : _activeConstraint.currentTau,
			angularTauTarget);
		const float currentLinearTau = advanceTau(
			_activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : _activeConstraint.currentTau,
			linearTauTarget);
		tickHeldBodyContact();

		float ANGULAR_RATIO_START = g_rockConfig.rockGrabFadeInStartAngularRatio;
		float ANGULAR_RATIO_END = g_rockConfig.rockGrabAngularToLinearForceRatio;
		float FADE_IN_TIME = g_rockConfig.rockGrabForceFadeInTime;

		currentLinearForce = targetForce;

		if (_state == HandState::HeldInit) {
			// Angular ratio fades from 200:1 → 17:1 over FADE_IN_TIME
			float angFadeT = (std::min)(1.0f, _grabStartTime / FADE_IN_TIME);
			float currentAngularRatio = ANGULAR_RATIO_START + (ANGULAR_RATIO_END - ANGULAR_RATIO_START) * angFadeT;
			currentAngularForce = targetForce / currentAngularRatio;

			// Transition to Held after forceFadeInTime
			float fadeT = (forceFadeInTime > 0.001f)
				? (std::min)(1.0f, _grabStartTime / forceFadeInTime)
				: 1.0f;
			if (fadeT >= 0.999f) {
				_state = HandState::HeldBody;
				ROCK_LOG_INFO(Hand, "{} hand: HeldInit → HeldBody (force fade-in complete, {:.2f}s)",
					handName(), _grabStartTime);
			}
		} else {
			currentAngularForce = targetForce / ANGULAR_RATIO_END;
		}

		// =====================================================================
		// Mass-based force clamping (HIGGS pattern)
		// HIGGS: linearMaxForce = min(configMaxForce, mass * 600)
		// Without clamping, light objects get maxForce=10000 → acceleration of
		// 20000 m/s² for a 0.5kg object → massive overshoot → oscillation.
		// Re-enabled after confirming collision filter fix didn't solve
		// the rubberbanding — the root cause was unclamped motor force.
		// =====================================================================
		{
			float MASS_FORCE_RATIO = g_rockConfig.rockGrabMaxForceToMassRatio;
			auto* objMotion = world->GetBodyMotion(_savedObjectState.bodyId);
			if (objMotion) {
				// BUG FIX: The previous code read invInertiaY instead of invMass!
				// Packed int16 layout at motion+0x20 (8 bytes):
				//   [0] = invInertiaX at +0x20
				//   [1] = invInertiaY at +0x22
				//   [2] = invInertiaZ at +0x24
				//   [3] = invMass     at +0x26  ← THIS is what we need
				// Old code: read uint32 at +0x20, mask upper 16 → got invInertiaY (index [1])
				// This produced wrong mass → tiny force cap → object can't move
				//
				// Correct: read uint16 at +0x26 (invMass, index [3])
				// Unpack inverse mass: hknp packs floats as bfloat16 (upper 16 bits of
				// IEEE 754 float). Verified by Ghidra audit of buildEffMassMatrixAt
				// (0x1417d1ea0): uses PUNPCKLWD to shift packed<<16, then reads as float
				// with no integer conversion. This is NOT linear scaling.
				auto packedInvMass = *reinterpret_cast<std::uint16_t*>(
					reinterpret_cast<char*>(objMotion) + 0x26);

				if (packedInvMass > 0) {
					// Unpack: same as inertia — treat as upper 16 bits of a float
					// Reconstruct by placing the packed value in the upper 16 bits of a uint32
					std::uint32_t asUint = static_cast<std::uint32_t>(packedInvMass) << 16;
					float invMass = 0.0f;
					std::memcpy(&invMass, &asUint, sizeof(float));

					if (invMass > 0.0001f) {
						float mass = 1.0f / invMass;
						float massCappedForce = mass * MASS_FORCE_RATIO;
						if (massCappedForce < currentLinearForce) {
							currentLinearForce = massCappedForce;
							currentAngularForce = massCappedForce / ANGULAR_RATIO_END;
						}
					}
				}
			}
		}

		// =====================================================================
		// Phase 0B.6: Apply motor parameters per-frame.
		// HIGGS S05 5.8: Re-apply ALL motor params from config every frame.
		// minForce = -maxForce (after mass clamping, not creation value).
		// Recovery values re-applied (not stale from creation).
		// Damping from config (0.8 HIGGS default).
		// =====================================================================
		if (_activeConstraint.angularMotor) {
			_activeConstraint.angularMotor->tau = currentAngularTau;
			_activeConstraint.angularMotor->maxForce = currentAngularForce;
			_activeConstraint.angularMotor->minForce = -currentAngularForce;
			_activeConstraint.angularMotor->damping = g_rockConfig.rockGrabAngularDamping;
			_activeConstraint.angularMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabAngularProportionalRecovery;
			_activeConstraint.angularMotor->constantRecoveryVelocity = g_rockConfig.rockGrabAngularConstantRecovery;
		}
		if (_activeConstraint.linearMotor) {
			_activeConstraint.linearMotor->tau = currentLinearTau;
			_activeConstraint.linearMotor->maxForce = currentLinearForce;
			_activeConstraint.linearMotor->minForce = -currentLinearForce;
			_activeConstraint.linearMotor->damping = g_rockConfig.rockGrabLinearDamping;
			_activeConstraint.linearMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabLinearProportionalRecovery;
			_activeConstraint.linearMotor->constantRecoveryVelocity = g_rockConfig.rockGrabLinearConstantRecovery;
		}
		_activeConstraint.currentTau = currentLinearTau;
		_activeConstraint.currentMaxForce = currentLinearForce;

		// No velocity injection on bodyB — motors are the sole mover.
		// Hand body (bodyA) velocity is set in updateCollisionTransform.

		// =====================================================================
		// Throttled debug log (every ~0.5 second)
		// Logs PIVOT ERROR (actual motor error) not body center distance.
		// pivotA_world = handPos + R_hand * pivotA (pivotA≈0, so ≈ handPos)
		// pivotB_world = objPos + R_obj * grabPointLocal
		// Motor error = |pivotA_world - pivotB_world|
		// =====================================================================
		_heldLogCounter++;
		if (_heldLogCounter >= 45) {
			_heldLogCounter = 0;

			if (g_rockConfig.rockDebugGrabFrameLogging) {
				const RE::NiTransform authoredHandTransform = makeAuthoredHandTransform(handWorldTransform, _isLeft);
				const RE::NiTransform desiredNodeWorldRaw = multiplyTransforms(handWorldTransform, _grabHandSpace);
				const RE::NiTransform desiredNodeWorldConstraint = multiplyTransforms(authoredHandTransform, _grabConstraintHandSpace);
				const RE::NiTransform desiredBodyTransformHandSpaceRaw = multiplyTransforms(_grabHandSpace, _grabBodyLocalTransform);
				const RE::NiTransform desiredBodyTransformHandSpaceConstraint =
					multiplyTransforms(_grabConstraintHandSpace, _grabBodyLocalTransform);
				const RE::NiTransform desiredBodyWorldRaw =
					multiplyTransforms(handWorldTransform, desiredBodyTransformHandSpaceRaw);
				const RE::NiTransform desiredBodyWorldConstraint =
					multiplyTransforms(authoredHandTransform, desiredBodyTransformHandSpaceConstraint);
				const RE::NiMatrix3 invRot = desiredBodyTransformHandSpaceConstraint.rotate.Transpose();
				const RE::NiTransform liveBodyWorld = getBodyWorldTransform(world, _savedObjectState.bodyId);
				auto* ownerCell = _savedObjectState.refr ? _savedObjectState.refr->GetParentCell() : nullptr;
				auto* heldBhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
				auto* bodyCollisionObject = heldBhkWorld
					? RE::bhkNPCollisionObject::Getbhk(heldBhkWorld, _savedObjectState.bodyId)
					: nullptr;
				auto* ownerNode = bodyCollisionObject ? bodyCollisionObject->sceneObject : nullptr;
				auto* hitNode = (_currentSelection.refr == _savedObjectState.refr)
					? _currentSelection.hitNode
					: nullptr;
				auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;

				struct NodeFrameMetrics
				{
					RE::NiAVObject* node = nullptr;
					RE::NiTransform world = {};
					RE::NiTransform expectedWorld = {};
					RE::NiPoint3 finger = {};
					RE::NiPoint3 expectedFinger = {};
					float rotErrDeg = -1.0f;
					float posErrGameUnits = -1.0f;
					bool hasCollisionObject = false;
					bool ownsBodyCollisionObject = false;
				};

				const auto captureNodeMetrics = [&](RE::NiAVObject* node, const RE::NiTransform& bodyLocalTransform) {
					NodeFrameMetrics metrics{};
					metrics.node = node;
					metrics.world = node ? node->world : liveBodyWorld;
					metrics.expectedWorld = node
						? deriveNodeWorldFromBodyWorld(liveBodyWorld, bodyLocalTransform)
						: liveBodyWorld;
					metrics.finger = getMatrixColumn(metrics.world.rotate, 0);
					metrics.expectedFinger = getMatrixColumn(metrics.expectedWorld.rotate, 0);
					if (node) {
						metrics.rotErrDeg = rotationDeltaDegrees(metrics.world.rotate, metrics.expectedWorld.rotate);
						metrics.posErrGameUnits = translationDeltaGameUnits(metrics.world, metrics.expectedWorld);
						auto* nodeCollisionObject = node->collisionObject.get();
						metrics.hasCollisionObject = (nodeCollisionObject != nullptr);
						metrics.ownsBodyCollisionObject = (nodeCollisionObject != nullptr && nodeCollisionObject == bodyCollisionObject);
					}
					return metrics;
				};

				const NodeFrameMetrics ownerMetrics = captureNodeMetrics(ownerNode, _grabOwnerBodyLocalTransform);
				const NodeFrameMetrics hitMetrics = captureNodeMetrics(hitNode, _grabBodyLocalTransform);
				const NodeFrameMetrics heldMetrics = captureNodeMetrics(_heldNode, _grabBodyLocalTransform);
				const NodeFrameMetrics rootMetrics = captureNodeMetrics(rootNode, _grabRootBodyLocalTransform);

				const RE::NiPoint3 worldPosDelta = desiredNodeWorldConstraint.translate - desiredNodeWorldRaw.translate;
				const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 1);
				const RE::NiPoint3 constraintFinger = getMatrixColumn(authoredHandTransform.rotate, 0);
				const RE::NiPoint3 desiredRawFinger = getMatrixColumn(desiredBodyWorldRaw.rotate, 0);
				const RE::NiPoint3 desiredConstraintFinger = getMatrixColumn(desiredBodyWorldConstraint.rotate, 0);
				const RE::NiPoint3 bodyFinger = getMatrixColumn(liveBodyWorld.rotate, 0);
				const float rawRowMax = max3(
					axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldRaw.rotate, 0)),
					axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldRaw.rotate, 1)),
					axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldRaw.rotate, 2)));
				const float rawColMax = max3(
					axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldRaw.rotate, 0)),
					axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldRaw.rotate, 1)),
					axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldRaw.rotate, 2)));
				const float constraintRow0 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldConstraint.rotate, 0));
				const float constraintRow1 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldConstraint.rotate, 1));
				const float constraintRow2 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldConstraint.rotate, 2));
				const float constraintCol0 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldConstraint.rotate, 0));
				const float constraintCol1 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldConstraint.rotate, 1));
				const float constraintCol2 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldConstraint.rotate, 2));
				const float constraintRowMax = max3(constraintRow0, constraintRow1, constraintRow2);
				const float constraintColMax = max3(constraintCol0, constraintCol1, constraintCol2);

				// Angular target probe: do not change solver behavior here. The log compares
				// the live constraint memory against the authored HIGGS-style target so the
				// next fix can target packing/semantics instead of guessing at motor values.
				auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
				auto* target_bRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + 0x10);
				auto* transformB_rot = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Col0);
				const RE::NiMatrix3 targetAsHkColumns = matrixFromHkColumns(target_bRca);
				const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(target_bRca);
				const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformB_rot);
				const RE::NiMatrix3 constraintTargetInv = desiredBodyTransformHandSpaceConstraint.rotate.Transpose();
				const RE::NiMatrix3 rawTargetInv = desiredBodyTransformHandSpaceRaw.rotate.Transpose();
				const int ragdollMotorEnabled = *(constraintData + ATOM_RAGDOLL_MOT + 0x02) ? 1 : 0;

				ROCK_LOG_INFO(Hand,
					"{} GRAB FRAME HOLD: rawVsConstraintWorld={:.2f}deg rawVsConstraintTarget={:.2f}deg "
					"bodyVsRaw={:.2f}deg bodyVsConstraint={:.2f}deg "
					"ownerErr={:.2f}deg/{:.2f}gu hitErr={:.2f}deg/{:.2f}gu "
					"heldErr={:.2f}deg/{:.2f}gu rootErr={:.2f}deg/{:.2f}gu "
					"worldPosDelta=({:.2f},{:.2f},{:.2f}) "
					"rawFinger=({:.3f},{:.3f},{:.3f}) constraintFinger=({:.3f},{:.3f},{:.3f})",
					handName(),
					rotationDeltaDegrees(desiredNodeWorldRaw.rotate, desiredNodeWorldConstraint.rotate),
					rotationDeltaDegrees(desiredBodyTransformHandSpaceRaw.rotate, desiredBodyTransformHandSpaceConstraint.rotate),
					rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorldRaw.rotate),
					rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorldConstraint.rotate),
					ownerMetrics.rotErrDeg, ownerMetrics.posErrGameUnits,
					hitMetrics.rotErrDeg, hitMetrics.posErrGameUnits,
					heldMetrics.rotErrDeg, heldMetrics.posErrGameUnits,
					rootMetrics.rotErrDeg, rootMetrics.posErrGameUnits,
					worldPosDelta.x, worldPosDelta.y, worldPosDelta.z,
					rawFinger.x, rawFinger.y, rawFinger.z,
					constraintFinger.x, constraintFinger.y, constraintFinger.z);

				ROCK_LOG_INFO(Hand,
					"{} GRAB FRAME NODES: bodyColl={:p} "
					"owner='{}'({:p}) hasCol={} ownsBodyCol={} "
					"hit='{}'({:p}) hasCol={} ownsBodyCol={} "
					"held='{}'({:p}) hasCol={} ownsBodyCol={} "
					"root='{}'({:p}) hasCol={} ownsBodyCol={} "
					"sameOwnerHeld={} sameHitHeld={} sameRootHeld={}",
					handName(),
					static_cast<const void*>(bodyCollisionObject),
					nodeDebugName(ownerMetrics.node), static_cast<const void*>(ownerMetrics.node),
					ownerMetrics.hasCollisionObject ? "yes" : "no",
					ownerMetrics.ownsBodyCollisionObject ? "yes" : "no",
					nodeDebugName(hitMetrics.node), static_cast<const void*>(hitMetrics.node),
					hitMetrics.hasCollisionObject ? "yes" : "no",
					hitMetrics.ownsBodyCollisionObject ? "yes" : "no",
					nodeDebugName(heldMetrics.node), static_cast<const void*>(heldMetrics.node),
					heldMetrics.hasCollisionObject ? "yes" : "no",
					heldMetrics.ownsBodyCollisionObject ? "yes" : "no",
					nodeDebugName(rootMetrics.node), static_cast<const void*>(rootMetrics.node),
					rootMetrics.hasCollisionObject ? "yes" : "no",
					rootMetrics.ownsBodyCollisionObject ? "yes" : "no",
					ownerMetrics.node == heldMetrics.node ? "yes" : "no",
					hitMetrics.node == heldMetrics.node ? "yes" : "no",
					rootMetrics.node == heldMetrics.node ? "yes" : "no");

				ROCK_LOG_INFO(Hand,
					"{} GRAB FRAME VISUALS: desiredRawFinger=({:.3f},{:.3f},{:.3f}) desiredConstraintFinger=({:.3f},{:.3f},{:.3f}) "
					"bodyFinger=({:.3f},{:.3f},{:.3f}) "
					"ownerFinger=({:.3f},{:.3f},{:.3f}) ownerExpectedFinger=({:.3f},{:.3f},{:.3f}) "
					"hitFinger=({:.3f},{:.3f},{:.3f}) hitExpectedFinger=({:.3f},{:.3f},{:.3f}) "
					"heldFinger=({:.3f},{:.3f},{:.3f}) heldExpectedFinger=({:.3f},{:.3f},{:.3f}) "
					"rootFinger=({:.3f},{:.3f},{:.3f}) rootExpectedFinger=({:.3f},{:.3f},{:.3f}) "
					"targetInvFinger=({:.3f},{:.3f},{:.3f})",
					handName(),
					desiredRawFinger.x, desiredRawFinger.y, desiredRawFinger.z,
					desiredConstraintFinger.x, desiredConstraintFinger.y, desiredConstraintFinger.z,
					bodyFinger.x, bodyFinger.y, bodyFinger.z,
					ownerMetrics.finger.x, ownerMetrics.finger.y, ownerMetrics.finger.z,
					ownerMetrics.expectedFinger.x, ownerMetrics.expectedFinger.y, ownerMetrics.expectedFinger.z,
					hitMetrics.finger.x, hitMetrics.finger.y, hitMetrics.finger.z,
					hitMetrics.expectedFinger.x, hitMetrics.expectedFinger.y, hitMetrics.expectedFinger.z,
					heldMetrics.finger.x, heldMetrics.finger.y, heldMetrics.finger.z,
					heldMetrics.expectedFinger.x, heldMetrics.expectedFinger.y, heldMetrics.expectedFinger.z,
					rootMetrics.finger.x, rootMetrics.finger.y, rootMetrics.finger.z,
					rootMetrics.expectedFinger.x, rootMetrics.expectedFinger.y, rootMetrics.expectedFinger.z,
					invRot.entry[0][0], invRot.entry[1][0], invRot.entry[2][0]);

				ROCK_LOG_INFO(Hand,
					"{} GRAB ANGULAR PROBE: rawAxisErr(rowMax={:.2f} colMax={:.2f}) "
					"constraintAxisErr(rowMax={:.2f} colMax={:.2f} rows=({:.2f},{:.2f},{:.2f}) cols=({:.2f},{:.2f},{:.2f})) "
					"targetMem(colsVsConstraintInv={:.2f} rowsVsConstraintInv={:.2f} "
					"colsVsConstraintForward={:.2f} colsVsRawInv={:.2f} colsVsTransformB={:.2f}) "
					"ragEnabled={} angMotor={:p} angTau={:.3f} linTau={:.3f} angF={:.0f}",
					handName(),
					rawRowMax, rawColMax,
					constraintRowMax, constraintColMax,
					constraintRow0, constraintRow1, constraintRow2,
					constraintCol0, constraintCol1, constraintCol2,
					rotationDeltaDegrees(targetAsHkColumns, constraintTargetInv),
					rotationDeltaDegrees(targetAsHkRows, constraintTargetInv),
					rotationDeltaDegrees(targetAsHkColumns, desiredBodyTransformHandSpaceConstraint.rotate),
					rotationDeltaDegrees(targetAsHkColumns, rawTargetInv),
					rotationDeltaDegrees(targetAsHkColumns, transformBAsHkColumns),
					ragdollMotorEnabled,
					static_cast<const void*>(_activeConstraint.angularMotor),
					currentAngularTau,
					currentLinearTau,
					currentAngularForce);
			}

			auto* bodyArray = world->GetBodyArray();
			auto* h = reinterpret_cast<float*>(&bodyArray[_handBody.getBodyId().value]);
			auto* o = reinterpret_cast<float*>(&bodyArray[_savedObjectState.bodyId.value]);

			// Read ACTUAL pivotA and pivotB from the constraint data (not stale member vars)
			auto* cd = static_cast<char*>(_activeConstraint.constraintData);
			auto* pivotA_local = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
			auto* pivotB_local = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);

			// pivotA_world = handBodyPos + R_hand * pivotA_local (includes palm offset!)
			float pax = h[12] + h[0]*pivotA_local[0] + h[4]*pivotA_local[1] + h[8]*pivotA_local[2];
			float pay = h[13] + h[1]*pivotA_local[0] + h[5]*pivotA_local[1] + h[9]*pivotA_local[2];
			float paz = h[14] + h[2]*pivotA_local[0] + h[6]*pivotA_local[1] + h[10]*pivotA_local[2];

			// pivotB_world = objBodyPos + R_obj * pivotB_local (current per-frame updated value)
			float pbx = o[12] + o[0]*pivotB_local[0] + o[4]*pivotB_local[1] + o[8]*pivotB_local[2];
			float pby = o[13] + o[1]*pivotB_local[0] + o[5]*pivotB_local[1] + o[9]*pivotB_local[2];
			float pbz = o[14] + o[2]*pivotB_local[0] + o[6]*pivotB_local[1] + o[10]*pivotB_local[2];

			// Motor error = world-space distance between pivotA and pivotB
			float ex = pax - pbx, ey = pay - pby, ez = paz - pbz;
			float pivotErr = std::sqrt(ex*ex + ey*ey + ez*ez);

			// Body center distance for reference
			float dx = o[12] - h[12], dy = o[13] - h[13], dz = o[14] - h[14];
			float bodyDist = std::sqrt(dx*dx + dy*dy + dz*dz);

			// Object velocity from motion array (velocity at motion+0x40)
			float objVelMag = 0.0f;
			{
				auto* objMotion = world->GetBodyMotion(_savedObjectState.bodyId);
				if (objMotion) {
					auto* mv = reinterpret_cast<float*>(reinterpret_cast<char*>(objMotion) + 0x40);
					objVelMag = std::sqrt(mv[0]*mv[0] + mv[1]*mv[1] + mv[2]*mv[2]);
				}
			}

			ROCK_LOG_INFO(Hand, "{} HELD: angTau={:.3f} linTau={:.3f} linF={:.0f} angF={:.0f} "
				"ERR={:.4f}({:.1f}gu) bDist={:.3f} objVel={:.3f} "
				"paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
				"handW=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
				handName(), currentAngularTau, currentLinearTau, currentLinearForce, currentAngularForce,
				pivotErr, pivotErr * 70.0f, bodyDist, objVelMag,
				pax*70.0f, pay*70.0f, paz*70.0f,
				pbx*70.0f, pby*70.0f, pbz*70.0f,
				h[12]*70.0f, h[13]*70.0f, h[14]*70.0f,
				o[12]*70.0f, o[13]*70.0f, o[14]*70.0f);

			// In-game notification with world positions
			_notifCounter++;
			if (_notifCounter >= 6) {
				_notifCounter = 0;
				float paToHand = std::sqrt((pax-h[12])*(pax-h[12])+(pay-h[13])*(pay-h[13])+(paz-h[14])*(paz-h[14])) * 70.0f;
				float pbToObj = std::sqrt((pbx-o[12])*(pbx-o[12])+(pby-o[13])*(pby-o[13])+(pbz-o[14])*(pbz-o[14])) * 70.0f;
				f4vr::showNotification(std::format("[ROCK] err={:.1f}gu F={:.0f} vel={:.2f} paOff={:.1f} pbOff={:.1f}",
					pivotErr * 70.0f, currentLinearForce, objVelMag, paToHand, pbToObj));
			}
		}

		// =====================================================================
		// Keep held body ACTIVE every frame (HIGGS hand.cpp:4002).
		// Without this, Havok's deactivation system puts the object to sleep
		// after it stabilizes, and the constraint motors can't affect sleeping
		// bodies. This is why the motor works initially then stops.
		// =====================================================================
		{
			typedef void activateBody_t(void*, std::uint32_t);
			static REL::Relocation<activateBody_t> activateBody{ REL::Offset(offsets::kFunc_ActivateBody) };
			activateBody(world, _savedObjectState.bodyId.value);
		}
	}

	// =========================================================================
	// Release — Destroy constraint, object stays DYNAMIC
	//
	// WHY: Object was never changed from DYNAMIC. Destroying the constraint
	// releases it from the hand. The object keeps its current velocity from
	// the physics simulation (natural drop). Throw velocity will be added
	// in Phase 4.
	// =========================================================================

	void Hand::releaseGrabbedObject(RE::hknpWorld* world)
	{
		if (!isHolding()) return;

		ROCK_LOG_INFO(Hand, "{} hand RELEASE: bodyId={} constraintId={}",
			handName(), _savedObjectState.bodyId.value,
			_activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu);

		// Restore original inertia before destroying constraint
		restoreGrabbedInertia(world, _savedObjectState);

		// --- A1 FIX: Object stays DYNAMIC on release (HIGGS pattern) ---
		// WHY: HIGGS does NOT restore KEYFRAMED on release. The object stays
		// DYNAMIC so throw velocity can be applied (Phase 4), and so it falls
		// naturally after release. The game's own FOIslandDeactivationCallback
		// and TESObjectREFR::InitHavokPostLoad will eventually transition it
		// back to KEYFRAMED when it settles or the cell reloads.
		// We DO NOT call SetMotionType(KEYFRAMED) here — that would kill velocity.
		//
		// Restore the original motionPropertiesId preset if it was non-standard,
		// so the object uses its original damping/velocity caps after release.
		if (world && _savedObjectState.originalMotionPropsId != 0 &&
			_savedObjectState.originalMotionPropsId != 1) {
			// Restore via collision object if available, fallback to raw
			RE::NiAVObject* releaseNode = nullptr;
			if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted()) {
				releaseNode = _savedObjectState.refr->Get3D();
			}
			auto* releaseCollObj = releaseNode ? releaseNode->collisionObject.get() : nullptr;
			if (releaseCollObj) {
				// The engine will handle restoring to the correct preset
				// when the object eventually deactivates
				ROCK_LOG_INFO(Hand, "{} hand: object stays DYNAMIC on release "
					"(game deactivation will restore KEYFRAMED)",
					handName());
			} else {
				// Fallback: restore motionPropertiesId directly
				typedef void setMotionProps_t(void*, int, short);
				static REL::Relocation<setMotionProps_t> setBodyMotionProperties{
					REL::Offset(offsets::kFunc_SetBodyMotionProperties) };
				setBodyMotionProperties(world, _savedObjectState.bodyId.value,
					static_cast<short>(_savedObjectState.originalMotionPropsId));
				ROCK_LOG_INFO(Hand, "{} hand MOTION RESTORE: propsId {} → {} (fallback)",
					handName(), 1, _savedObjectState.originalMotionPropsId);
			}
		}

		// --- Phase 2A: Clear thread-safe state FIRST (before any body IDs freed) ---
		_isHoldingFlag.store(false, std::memory_order_release);
		_heldBodyIdsCount.store(0, std::memory_order_release);
		_heldBodyContactFrame.store(100, std::memory_order_release);

		// --- Phase 2A: Disable contact event flag on ALL held bodies ---
		if (world) {
			typedef void disableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
			static REL::Relocation<disableBodyFlags_t> disableBodyFlags{ REL::Offset(offsets::kFunc_DisableBodyFlags) };
			for (auto bid : _heldBodyIds) {
				disableBodyFlags(world, bid, 0x80, 0);
			}
		}

		// Destroy the grab constraint (removes from world, frees motor refs)
		if (_activeConstraint.isValid()) {
			destroyGrabConstraint(world, _activeConstraint);
		}

		restoreHandCollisionAfterGrab(world);

		// TODO Phase 4: Apply throw velocity here based on hand velocity history

		frik::api::FRIKApi::inst->clearHandPose("ROCK_Grab", handFromBool(_isLeft));
		_savedObjectState.clear();
		_activeConstraint.clear();
		_heldBodyIds.clear();
		_grabHandSpace = RE::NiTransform();
		_grabConstraintHandSpace = RE::NiTransform();
		_grabBodyLocalTransform = RE::NiTransform();
		_grabRootBodyLocalTransform = RE::NiTransform();
		_grabOwnerBodyLocalTransform = RE::NiTransform();
		_heldNode = nullptr;
		_currentSelection.clear();
		_state = HandState::Idle;

		ROCK_LOG_INFO(Hand, "{} hand: Idle", handName());
	}
}

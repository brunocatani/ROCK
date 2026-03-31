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

	// =========================================================================
	// Native VR grab drop — Ghidra: 0x140F1AB90
	// Releases any existing native grab (BSMouseSpringAction) on the object.
	// Must be called when ROCK grabs to kill lingering native springs that
	// would otherwise fight our constraint and push the player.
	// =========================================================================
	static void nativeVRGrabDrop(void* playerChar, int handIndex)
	{
		typedef void func_t(void*, int);
		static REL::Relocation<func_t> func{ REL::Offset(0xF1AB90) };
		func(playerChar, handIndex);
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

				// --- HIGGS S03 Step 4: Convert to DYNAMIC preset ---
				// Save original motionPropertiesId, then set to 1 (DYNAMIC preset).
				// The DYNAMIC preset has standard damping/velocity parameters that
				// allow constraint motors to converge properly.
				// Without this, custom Bethesda presets (e.g. 5) may have velocity
				// caps or high damping that prevent the object from reaching the hand.
				_savedObjectState.originalMotionPropsId = motionPropsId;

				if (motionPropsId != 1) {
					typedef void setMotionProps_t(void*, int, short);
					static REL::Relocation<setMotionProps_t> setBodyMotionProperties{
						REL::Offset(0x153b2f0) };
					setBodyMotionProperties(world, objectBodyId.value, 1);

					// Verify it took effect
					std::uint16_t newPropsId = *reinterpret_cast<std::uint16_t*>(motionPtr + offsets::kMotion_PropertiesId);
					ROCK_LOG_INFO(Hand, "{} hand MOTION CONVERT: {} → {} (DYNAMIC preset)",
						handName(), motionPropsId, newPropsId);
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

		// --- Get object world transform ---
		auto* node3D = sel.refr->Get3D();
		RE::NiTransform objectWorldTransform;
		if (node3D) {
			objectWorldTransform = node3D->world;
		} else {
			objectWorldTransform = handWorldTransform;
		}

		// --- DIAGNOSTIC: Compare hitNode (collidableNode) vs refRoot (Get3D) ---
		// HIGGS uses collidableNode = GetNodeFromCollidable(collidable) — the NiNode
		// where the collision body is attached. ROCK was using sel.refr->Get3D() (the
		// NIF root). If they differ, the 75gu offset is the distance between them, and
		// re-rooting to the root MOVES triangles AWAY from the physics body.
		{
			auto* bodyArray = world->GetBodyArray();
			auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
			float bodyPosX = objFloats[12] * 70.0f;
			float bodyPosY = objFloats[13] * 70.0f;
			float bodyPosZ = objFloats[14] * 70.0f;

			ROCK_LOG_INFO(Hand, "=== GRAB NODE DIAGNOSTIC ===");
			ROCK_LOG_INFO(Hand, "  refRoot (Get3D):  ({:.1f},{:.1f},{:.1f})",
				node3D ? node3D->world.translate.x : 0.0f,
				node3D ? node3D->world.translate.y : 0.0f,
				node3D ? node3D->world.translate.z : 0.0f);
			ROCK_LOG_INFO(Hand, "  hitNode (collNi): ({:.1f},{:.1f},{:.1f}) ptr={}",
				sel.hitNode ? sel.hitNode->world.translate.x : 0.0f,
				sel.hitNode ? sel.hitNode->world.translate.y : 0.0f,
				sel.hitNode ? sel.hitNode->world.translate.z : 0.0f,
				sel.hitNode ? "valid" : "NULL");
			ROCK_LOG_INFO(Hand, "  havokBody*70:     ({:.1f},{:.1f},{:.1f})",
				bodyPosX, bodyPosY, bodyPosZ);

			if (node3D && sel.hitNode) {
				float rootToHit = std::sqrt(
					(node3D->world.translate.x - sel.hitNode->world.translate.x) *
					(node3D->world.translate.x - sel.hitNode->world.translate.x) +
					(node3D->world.translate.y - sel.hitNode->world.translate.y) *
					(node3D->world.translate.y - sel.hitNode->world.translate.y) +
					(node3D->world.translate.z - sel.hitNode->world.translate.z) *
					(node3D->world.translate.z - sel.hitNode->world.translate.z));
				float bodyToHit = std::sqrt(
					(bodyPosX - sel.hitNode->world.translate.x) *
					(bodyPosX - sel.hitNode->world.translate.x) +
					(bodyPosY - sel.hitNode->world.translate.y) *
					(bodyPosY - sel.hitNode->world.translate.y) +
					(bodyPosZ - sel.hitNode->world.translate.z) *
					(bodyPosZ - sel.hitNode->world.translate.z));
				float bodyToRoot = std::sqrt(
					(bodyPosX - node3D->world.translate.x) *
					(bodyPosX - node3D->world.translate.x) +
					(bodyPosY - node3D->world.translate.y) *
					(bodyPosY - node3D->world.translate.y) +
					(bodyPosZ - node3D->world.translate.z) *
					(bodyPosZ - node3D->world.translate.z));
				ROCK_LOG_INFO(Hand, "  rootToHitNode={:.1f}gu  bodyToHitNode={:.1f}gu  bodyToRoot={:.1f}gu",
					rootToHit, bodyToHit, bodyToRoot);
				ROCK_LOG_INFO(Hand, "  hitNode==refRoot: {}  hitNode name: {}",
					(sel.hitNode == node3D) ? "YES" : "NO",
					sel.hitNode->name.c_str() ? sel.hitNode->name.c_str() : "(null)");
			}

			// Walk NIF tree (first 2 levels) to show structure
			if (node3D) {
				ROCK_LOG_INFO(Hand, "  NIF tree root: '{}' type=NiNode",
					node3D->name.c_str() ? node3D->name.c_str() : "(null)");
				auto* rootNiNode = node3D->IsNode();
				if (rootNiNode) {
					auto& kids = rootNiNode->GetRuntimeData().children;
					for (std::uint32_t ci = 0; ci < kids.size() && ci < 8; ci++) {
						auto* kid = kids[ci].get();
						if (!kid) continue;
						bool hasCollision = (kid->collisionObject.get() != nullptr);
						bool isTriShape = (kid->IsTriShape() != nullptr);
						ROCK_LOG_INFO(Hand, "    child[{}]: '{}' pos=({:.1f},{:.1f},{:.1f}) {}{}",
							ci,
							kid->name.c_str() ? kid->name.c_str() : "(null)",
							kid->world.translate.x, kid->world.translate.y, kid->world.translate.z,
							isTriShape ? "BSTriShape " : "NiNode ",
							hasCollision ? "+COLLISION" : "");
						// One more level
						auto* kidNode = kid->IsNode();
						if (kidNode) {
							auto& grandkids = kidNode->GetRuntimeData().children;
							for (std::uint32_t gi = 0; gi < grandkids.size() && gi < 8; gi++) {
								auto* gk = grandkids[gi].get();
								if (!gk) continue;
								bool gkColl = (gk->collisionObject.get() != nullptr);
								bool gkTri = (gk->IsTriShape() != nullptr);
								ROCK_LOG_INFO(Hand, "      grandchild[{}]: '{}' pos=({:.1f},{:.1f},{:.1f}) {}{}",
									gi,
									gk->name.c_str() ? gk->name.c_str() : "(null)",
									gk->world.translate.x, gk->world.translate.y, gk->world.translate.z,
									gkTri ? "BSTriShape " : "NiNode ",
									gkColl ? "+COLLISION" : "");
							}
						}
					}
				}
			}
		}

		// --- Mesh-based grab point alignment ---
		// INVESTIGATION: Use hitNode (collidableNode) as the mesh extraction root
		// instead of refRoot (Get3D). The hitNode is the NiNode where the physics
		// body is attached — its world transform matches the Havok body position.
		// Previous approach re-rooted to refRoot which may be 75gu away.
		RE::NiAVObject* meshRoot = sel.hitNode ? sel.hitNode : node3D;
		RE::NiPoint3 grabSurfacePoint = objectWorldTransform.translate;
		bool meshGrabFound = false;

		if (meshRoot) {
			std::vector<TriangleData> triangles;
			extractAllTriangles(meshRoot, triangles);

			// --- Vertex format diagnostic for first BSTriShape in subtree ---
			// Log vertexDesc fields to verify FP16/FP32 detection is working.
			{
				RE::BSTriShape* firstTriShape = meshRoot->IsTriShape();
				if (!firstTriShape) {
					// meshRoot is a NiNode — check first child
					auto* meshNode = meshRoot->IsNode();
					if (meshNode) {
						auto& kids = meshNode->GetRuntimeData().children;
						for (std::uint32_t ci = 0; ci < kids.size(); ci++) {
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

			// NO re-rooting — if we extract from the hitNode subtree, the
			// triShape->world should already be at the physics body position.
			// The re-rooting was moving triangles to refRoot which is WRONG
			// if refRoot != hitNode.
			if (!triangles.empty()) {
				// DIAGNOSTIC: log triangle centroid vs hitNode vs refRoot vs bodyPos
				auto& t0 = triangles[0];
				float cx = (t0.v0.x + t0.v1.x + t0.v2.x) / 3.0f;
				float cy = (t0.v0.y + t0.v1.y + t0.v2.y) / 3.0f;
				float cz = (t0.v0.z + t0.v1.z + t0.v2.z) / 3.0f;

				auto* bodyArray = world->GetBodyArray();
				auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);

				float distToHitNode = sel.hitNode ? std::sqrt(
					(cx - sel.hitNode->world.translate.x) * (cx - sel.hitNode->world.translate.x) +
					(cy - sel.hitNode->world.translate.y) * (cy - sel.hitNode->world.translate.y) +
					(cz - sel.hitNode->world.translate.z) * (cz - sel.hitNode->world.translate.z)) : -1.0f;
				float distToRoot = node3D ? std::sqrt(
					(cx - node3D->world.translate.x) * (cx - node3D->world.translate.x) +
					(cy - node3D->world.translate.y) * (cy - node3D->world.translate.y) +
					(cz - node3D->world.translate.z) * (cz - node3D->world.translate.z)) : -1.0f;
				float distToBody = std::sqrt(
					(cx - objFloats[12]*70.0f) * (cx - objFloats[12]*70.0f) +
					(cy - objFloats[13]*70.0f) * (cy - objFloats[13]*70.0f) +
					(cz - objFloats[14]*70.0f) * (cz - objFloats[14]*70.0f));

				ROCK_LOG_INFO(MeshGrab, "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToHitNode={:.1f} distToRoot={:.1f} distToBody={:.1f}",
					cx, cy, cz, distToHitNode, distToRoot, distToBody);
			}

			if (!triangles.empty()) {
				RE::NiPoint3 palmPos = computePalmPosition(handWorldTransform, _isLeft);
				RE::NiPoint3 palmDir = computePalmForward(handWorldTransform, _isLeft);

				GrabPoint grabPt;
				if (findClosestGrabPoint(triangles, palmPos, palmDir, 1.0f, 1.0f, grabPt)) {
					grabSurfacePoint = grabPt.position;
					meshGrabFound = true;
					ROCK_LOG_INFO(Hand, "{} hand MESH GRAB: {} tris, closest at ({:.1f},{:.1f},{:.1f})",
						handName(), triangles.size(),
						grabPt.position.x, grabPt.position.y, grabPt.position.z);
				}
			}
		}

		_grabStartTime = 0.0f;

		// --- Compute frozen hand-to-object transform for visual hand adjustment ---
		// HIGGS pattern (desiredNodeTransformHandSpace): stores "where the object should
		// be relative to the hand" at grab time. Per-frame, the hand is positioned at:
		//   adjustedHand = objectWorld * Invert(_grabHandSpace)
		// This makes the visible hand follow the object, hiding motor offset.
		{
			RE::NiPoint3 palmPos = computePalmPosition(handWorldTransform, _isLeft);
			RE::NiTransform shiftedObject = objectWorldTransform;
			shiftedObject.translate.x += palmPos.x - grabSurfacePoint.x;
			shiftedObject.translate.y += palmPos.y - grabSurfacePoint.y;
			shiftedObject.translate.z += palmPos.z - grabSurfacePoint.z;

			// handInverse = Invert(handWorldTransform)
			RE::NiMatrix3 invR = handWorldTransform.rotate.Transpose();
			float invS = (handWorldTransform.scale > 0.0001f) ? (1.0f / handWorldTransform.scale) : 1.0f;
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

			// _grabHandSpace = handInverse * shiftedObject
			// result.rot = invR * shifted.rot
			// result.pos = invP + (invR * shifted.pos) * invS
			_grabHandSpace.rotate = invR * shiftedObject.rotate;
			_grabHandSpace.scale = invS * shiftedObject.scale;
			RE::NiPoint3 rp;
			rp.x = invR.entry[0][0] * shiftedObject.translate.x + invR.entry[0][1] * shiftedObject.translate.y + invR.entry[0][2] * shiftedObject.translate.z;
			rp.y = invR.entry[1][0] * shiftedObject.translate.x + invR.entry[1][1] * shiftedObject.translate.y + invR.entry[1][2] * shiftedObject.translate.z;
			rp.z = invR.entry[2][0] * shiftedObject.translate.x + invR.entry[2][1] * shiftedObject.translate.y + invR.entry[2][2] * shiftedObject.translate.z;
			_grabHandSpace.translate.x = invP.x + rp.x * invS;
			_grabHandSpace.translate.y = invP.y + rp.y * invS;
			_grabHandSpace.translate.z = invP.z + rp.z * invS;
			_heldNode = node3D;

			ROCK_LOG_INFO(Hand, "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
				"palmPos=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
				handName(),
				_grabHandSpace.translate.x, _grabHandSpace.translate.y, _grabHandSpace.translate.z,
				palmPos.x, palmPos.y, palmPos.z,
				grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
		}

		// --- Compute grab point in object's Havok body-local space ---
		// WHY: The constraint pivot and velocity injection target must be at the
		// SURFACE contact point, not the hand body center (wrist/palm).
		// If the mesh grab found a surface point, use it. Otherwise fall back
		// to the hand body position.
		// grabPointLocal = R_obj^T * (grabWorldHk - objBodyPos)
		// This is the grab point in the object's body-local frame.
		{
			auto* bodyArray = world->GetBodyArray();
			auto* objBody = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
			auto* handBody = reinterpret_cast<float*>(&bodyArray[_collisionBodyId.value]);

			// Grab world position in Havok coords
			float grabHk[3];
			if (meshGrabFound) {
				// Use mesh surface contact point (game units → Havok)
				constexpr float INV_HAVOK_SCALE = 1.0f / 70.0f;
				grabHk[0] = grabSurfacePoint.x * INV_HAVOK_SCALE;
				grabHk[1] = grabSurfacePoint.y * INV_HAVOK_SCALE;
				grabHk[2] = grabSurfacePoint.z * INV_HAVOK_SCALE;
			} else {
				// Fallback: hand body position (palm)
				grabHk[0] = handBody[12];
				grabHk[1] = handBody[13];
				grabHk[2] = handBody[14];
			}

			// Vector from object origin to grab point (world Havok space)
			float dx = grabHk[0] - objBody[12];
			float dy = grabHk[1] - objBody[13];
			float dz = grabHk[2] - objBody[14];

			// Transform world-space delta to body-local space.
			// Body array is COLUMN-MAJOR (blind audit 2026-03-30: setBodyTransform at
			// 0x1415395e0 copies hkTransformf directly, no transpose. hkTransformf is
			// column-major). body[0,1,2] = column 0 = (R00, R10, R20).
			// Havok uses LEFT-MULTIPLICATION: world = R * local + t.
			// Inverse: local = R^T * delta. Dotting each column with delta gives R^T * delta.
			// component j = dot(column_j, delta) = (R^T * delta)[j].
			_grabPointLocal[0] = objBody[0] * dx + objBody[1] * dy + objBody[2] * dz;
			_grabPointLocal[1] = objBody[4] * dx + objBody[5] * dy + objBody[6] * dz;
			_grabPointLocal[2] = objBody[8] * dx + objBody[9] * dy + objBody[10] * dz;
			_grabPointLocal[3] = 0.0f;

			ROCK_LOG_INFO(Hand, "{} GRAB PIVOT: grabPointLocal=({:.4f},{:.4f},{:.4f}) "
				"grabHk=({:.3f},{:.3f},{:.3f}) objHk=({:.3f},{:.3f},{:.3f}) meshGrab={}",
				handName(),
				_grabPointLocal[0], _grabPointLocal[1], _grabPointLocal[2],
				grabHk[0], grabHk[1], grabHk[2],
				objBody[12], objBody[13], objBody[14],
				meshGrabFound ? "yes" : "no");
		}

		// --- Log body positions for constraint debugging ---
		{
			auto* bodyArray = world->GetBodyArray();
			auto* handFloats = reinterpret_cast<float*>(&bodyArray[_collisionBodyId.value]);
			auto* objFloats = reinterpret_cast<float*>(&bodyArray[objectBodyId.value]);
			ROCK_LOG_INFO(Hand, "{} DIAG: handBody pos=({:.3f},{:.3f},{:.3f}) objBody pos=({:.3f},{:.3f},{:.3f})",
				handName(),
				handFloats[12], handFloats[13], handFloats[14],
				objFloats[12], objFloats[13], objFloats[14]);
			ROCK_LOG_INFO(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})",
				handName(),
				handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z,
				objectWorldTransform.translate.x, objectWorldTransform.translate.y, objectWorldTransform.translate.z);
		}

		// --- Zero object velocity before constraint creation ---
		// WHY: The object may have residual velocity (falling, bumped, etc.).
		// Without zeroing, the constraint fights existing momentum → spinning/orbiting.
		// HIGGS does this implicitly by damping nearby objects at grab time.
		{
			alignas(16) float zeroVel[4] = { 0, 0, 0, 0 };
			typedef void setVel_t(void*, std::uint32_t, const float*, const float*);
			static REL::Relocation<setVel_t> setBodyVelocity{ REL::Offset(offsets::kFunc_SetBodyVelocity) };
			setBodyVelocity(world, objectBodyId.value, zeroVel, zeroVel);

			// Zero velocity on ALL held bodies (weapon children: scope, barrel, stock)
			for (auto bid : _heldBodyIds) {
				if (bid != objectBodyId.value) {
					setBodyVelocity(world, bid, zeroVel, zeroVel);
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
		{
			typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
			static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{
				REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

			auto* bodyArray = world->GetBodyArray();
			auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[_collisionBodyId.value]);
			auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
			auto newFilter = currentFilter | (1u << 14);  // bit 14 = no-collision flag
			setBodyCollisionFilterInfo(world, _collisionBodyId.value, newFilter, 0);  // 0 = rebuild caches
			ROCK_LOG_INFO(Hand, "{} hand: disabled hand collision (bit 14 + broadphase rebuild) filter=0x{:08X}",
				handName(), newFilter);
		}

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
		// Phase 0B.2: TransformB rotation uses Inverse(_grabHandSpace).rotation
		// (HIGGS S00 0.6b, simplified by PRE-1: no GetRigidBodyTLocalTransform in FO4VR)
		{
			RE::NiPoint3 palmPos = computePalmPosition(handWorldTransform, _isLeft);
			constexpr float INV_HAVOK_SCALE = 1.0f / 70.0f;

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
				float palmToWand = std::sqrt(
					(palmPos.x - handWorldTransform.translate.x)*(palmPos.x - handWorldTransform.translate.x) +
					(palmPos.y - handWorldTransform.translate.y)*(palmPos.y - handWorldTransform.translate.y) +
					(palmPos.z - handWorldTransform.translate.z)*(palmPos.z - handWorldTransform.translate.z));
				ROCK_LOG_INFO(Hand, "GRAB DIAG {}: palmPos=({:.1f},{:.1f},{:.1f}) wandPos=({:.1f},{:.1f},{:.1f}) "
					"grabSurface=({:.1f},{:.1f},{:.1f}) meshGrab={} "
					"palmToGrab_hk={:.4f} ({:.1f} game units) palmToWand={:.1f} game units",
					handName(),
					palmPos.x, palmPos.y, palmPos.z,
					handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z,
					grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z,
					meshGrabFound,
					palmToGrab, palmToGrab * 70.0f, palmToWand);
			}

			_activeConstraint = createGrabConstraint(
				world, _collisionBodyId, objectBodyId,
				palmHk, grabHk, _grabHandSpace,
				tau, damping, maxForce,
				proportionalRecovery, constantRecovery);
		}

		if (!_activeConstraint.isValid()) {
			ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: constraint creation failed", handName());
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
			_collisionBodyId.value, objectBodyId.value, _heldBodyIds.size());

		// --- Phase 2A: Enable contact event flag on ALL held bodies ---
		// Ghidra: FUN_141e54750 shows body flag 0x80 must be enabled for contact events.
		// HIGGS S10: registers listener on all connected bodies, not just primary.
		{
			typedef void enableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
			static REL::Relocation<enableBodyFlags_t> enableBodyFlags{ REL::Offset(0x153c090) };
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
	// the frozen _grabHandSpace transform.
	// Motor PARAMETERS (tau, force, damping) are updated for fade-in and
	// collision-adaptive behavior.
	// =========================================================================

	void Hand::updateHeldObject(RE::hknpWorld* world,
		const RE::NiTransform& handWorldTransform, float deltaTime,
		float forceFadeInTime, float tauMin, float tauMax,
		float tauIncrement, float tauDecrement,
		float closeThreshold, float farThreshold)
	{
		(void)tauIncrement; (void)tauDecrement;
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

		_grabStartTime += deltaTime;

		// =====================================================================
		// Per-frame: update angular target and pivotB from frozen _grabHandSpace.
		// =====================================================================
		if (_activeConstraint.constraintData) {
			auto* cd = static_cast<char*>(_activeConstraint.constraintData);

			// Compute Inverse(_grabHandSpace)
			// desiredHandTransformHavokObjSpace = Invert(_grabHandSpace)
			// Rotation inverse = Transpose (orthonormal matrix)
			RE::NiMatrix3 invRot = _grabHandSpace.rotate.Transpose();
			float invScale = (_grabHandSpace.scale > 0.0001f) ? (1.0f / _grabHandSpace.scale) : 1.0f;

			// Inverse translation: -(R^T * pos) / scale
			RE::NiPoint3 invPos;
			invPos.x = -(invRot.entry[0][0] * _grabHandSpace.translate.x +
				invRot.entry[0][1] * _grabHandSpace.translate.y +
				invRot.entry[0][2] * _grabHandSpace.translate.z) * invScale;
			invPos.y = -(invRot.entry[1][0] * _grabHandSpace.translate.x +
				invRot.entry[1][1] * _grabHandSpace.translate.y +
				invRot.entry[1][2] * _grabHandSpace.translate.z) * invScale;
			invPos.z = -(invRot.entry[2][0] * _grabHandSpace.translate.x +
				invRot.entry[2][1] * _grabHandSpace.translate.y +
				invRot.entry[2][2] * _grabHandSpace.translate.z) * invScale;

			// Phase 0B.5: Update angular target (m_target_bRca) at constraint+0xD0
			// = Inverse(_grabHandSpace).rotation = invRot
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

			// Per-frame pivotB update from frozen _grabHandSpace + palm offset.
			// UNDER INVESTIGATION (2026-03-30): First-frame pivotB jumps significantly
			// from creation-time value. Needs proper audit against HIGGS before changing.
			{
				// Palm offset in hand-local space (from config, game units)
				RE::NiPoint3 palmHS;
				palmHS.x = g_rockConfig.rockPalmOffsetRight;
				palmHS.y = g_rockConfig.rockPalmOffsetForward;
				palmHS.z = g_rockConfig.rockPalmOffsetUp;

				// Transform palmHS through desiredHandTransformHavokObjSpace:
				// result = invRot * palmHS * invScale + invPos
				RE::NiPoint3 transformed;
				transformed.x = (invRot.entry[0][0]*palmHS.x + invRot.entry[0][1]*palmHS.y + invRot.entry[0][2]*palmHS.z) * invScale + invPos.x;
				transformed.y = (invRot.entry[1][0]*palmHS.x + invRot.entry[1][1]*palmHS.y + invRot.entry[1][2]*palmHS.z) * invScale + invPos.y;
				transformed.z = (invRot.entry[2][0]*palmHS.x + invRot.entry[2][1]*palmHS.y + invRot.entry[2][2]*palmHS.z) * invScale + invPos.z;

				// Apply object scale and Havok scale
				float objScale = 1.0f;
				if (_savedObjectState.refr) {
					auto* objNode = _savedObjectState.refr->Get3D();
					if (objNode) objScale = objNode->world.scale;
				}
				constexpr float kHavokScale = 1.0f / 70.0f;
				float combinedScale = objScale * kHavokScale;

				auto* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);

				if (_pivotBLogCounter++ < 5) {
					ROCK_LOG_INFO(Hand, "{} PIVOTB[{}]: BEFORE=({:.4f},{:.4f},{:.4f}) AFTER=({:.4f},{:.4f},{:.4f}) "
						"palmHS=({:.1f},{:.1f},{:.1f}) invPos=({:.2f},{:.2f},{:.2f}) "
						"transformed=({:.2f},{:.2f},{:.2f}) objScale={:.2f} combScale={:.4f}",
						handName(), _pivotBLogCounter - 1,
						pivotB[0], pivotB[1], pivotB[2],
						transformed.x * combinedScale, transformed.y * combinedScale, transformed.z * combinedScale,
						palmHS.x, palmHS.y, palmHS.z,
						invPos.x, invPos.y, invPos.z,
						transformed.x, transformed.y, transformed.z,
						objScale, combinedScale);
				}

				// Atomic 16-byte write
				alignas(16) float newPivotB[4] = {
					transformed.x * combinedScale,
					transformed.y * combinedScale,
					transformed.z * combinedScale,
					0.0f
				};
				_mm_store_ps(pivotB, _mm_load_ps(newPivotB));
			}

			// Also update transformB.rotation to match angular target
			// (keeps constraint internal state consistent)
			// Atomic 16-byte writes (C3 fix — see angular target comment above)
			{
				auto* tB_col0 = reinterpret_cast<float*>(cd + offsets::kTransformB_Col0);
				auto* tB_col1 = reinterpret_cast<float*>(cd + offsets::kTransformB_Col1);
				auto* tB_col2 = reinterpret_cast<float*>(cd + offsets::kTransformB_Col2);
				alignas(16) float c0[4] = { invRot.entry[0][0], invRot.entry[1][0], invRot.entry[2][0], 0.0f };
				alignas(16) float c1[4] = { invRot.entry[0][1], invRot.entry[1][1], invRot.entry[2][1], 0.0f };
				alignas(16) float c2[4] = { invRot.entry[0][2], invRot.entry[1][2], invRot.entry[2][2], 0.0f };
				_mm_store_ps(tB_col0, _mm_load_ps(c0));
				_mm_store_ps(tB_col1, _mm_load_ps(c1));
				_mm_store_ps(tB_col2, _mm_load_ps(c2));
			}
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
		// Collision-adaptive tau: normal=tauMax, colliding=tauMin, smooth lerp
		float tauTarget = isHeldBodyColliding() ? tauMin : tauMax;
		float tauDelta = tauTarget - _activeConstraint.currentTau;
		float tauStep = g_rockConfig.rockGrabTauLerpSpeed * deltaTime;
		float currentTau;
		if (std::abs(tauDelta) > tauStep) {
			currentTau = _activeConstraint.currentTau + (tauDelta > 0 ? tauStep : -tauStep);
		} else {
			currentTau = tauTarget;
		}
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
				// Unpack: float = (ushort)packed / 65536.0f would give Havok's half-float
				// But actually hknp uses a different packing — let's use the same approach
				// as HIGGS: read the inverse mass from the float at a known offset.
				//
				// Actually the simplest correct approach: read the packed invMass int16,
				// reconstruct as float using the same packing as inertia normalization.
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
			_activeConstraint.angularMotor->tau = currentTau;
			_activeConstraint.angularMotor->maxForce = currentAngularForce;
			_activeConstraint.angularMotor->minForce = -currentAngularForce;
			_activeConstraint.angularMotor->damping = g_rockConfig.rockGrabAngularDamping;
			_activeConstraint.angularMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabAngularProportionalRecovery;
			_activeConstraint.angularMotor->constantRecoveryVelocity = g_rockConfig.rockGrabAngularConstantRecovery;
		}
		if (_activeConstraint.linearMotor) {
			_activeConstraint.linearMotor->tau = currentTau;
			_activeConstraint.linearMotor->maxForce = currentLinearForce;
			_activeConstraint.linearMotor->minForce = -currentLinearForce;
			_activeConstraint.linearMotor->damping = g_rockConfig.rockGrabLinearDamping;
			_activeConstraint.linearMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabLinearProportionalRecovery;
			_activeConstraint.linearMotor->constantRecoveryVelocity = g_rockConfig.rockGrabLinearConstantRecovery;
		}
		_activeConstraint.currentTau = currentTau;
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

			auto* bodyArray = world->GetBodyArray();
			auto* h = reinterpret_cast<float*>(&bodyArray[_collisionBodyId.value]);
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

			// PivotA offset from hand body center (should be palm offset, NOT zero)
			float paOffMag = std::sqrt(pivotA_local[0]*pivotA_local[0] +
				pivotA_local[1]*pivotA_local[1] + pivotA_local[2]*pivotA_local[2]);

			// Object velocity from motion array (velocity at motion+0x40)
			float objVelMag = 0.0f;
			{
				auto* objMotion = world->GetBodyMotion(_savedObjectState.bodyId);
				if (objMotion) {
					auto* mv = reinterpret_cast<float*>(reinterpret_cast<char*>(objMotion) + 0x40);
					objVelMag = std::sqrt(mv[0]*mv[0] + mv[1]*mv[1] + mv[2]*mv[2]);
				}
			}

			ROCK_LOG_INFO(Hand, "{} HELD: tau={:.3f} linF={:.0f} angF={:.0f} "
				"ERR={:.4f}({:.1f}gu) bDist={:.3f} objVel={:.3f} "
				"paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
				"handW=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
				handName(), currentTau, currentLinearForce, currentAngularForce,
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
			static REL::Relocation<activateBody_t> activateBody{ REL::Offset(0x1546ef0) };
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

		// --- Restore original motionPropertiesId (HIGGS S03 Step 4 reverse) ---
		// At grab time we converted to DYNAMIC preset (1). Restore original preset.
		if (world && _savedObjectState.originalMotionPropsId != 0 &&
			_savedObjectState.originalMotionPropsId != 1) {
			typedef void setMotionProps_t(void*, int, short);
			static REL::Relocation<setMotionProps_t> setBodyMotionProperties{
				REL::Offset(0x153b2f0) };
			setBodyMotionProperties(world, _savedObjectState.bodyId.value,
				static_cast<short>(_savedObjectState.originalMotionPropsId));
			ROCK_LOG_INFO(Hand, "{} hand MOTION RESTORE: {} → {} (original preset)",
				handName(), 1, _savedObjectState.originalMotionPropsId);
		}

		// --- Phase 2A: Clear thread-safe state FIRST (before any body IDs freed) ---
		_isHoldingFlag.store(false, std::memory_order_release);
		_heldBodyIdsCount.store(0, std::memory_order_release);
		_heldBodyContactFrame.store(100, std::memory_order_release);

		// --- Phase 2A: Disable contact event flag on ALL held bodies ---
		if (world) {
			typedef void disableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
			static REL::Relocation<disableBodyFlags_t> disableBodyFlags{ REL::Offset(0x153c150) };
			for (auto bid : _heldBodyIds) {
				disableBodyFlags(world, bid, 0x80, 0);
			}
		}

		// Destroy the grab constraint (removes from world, frees motor refs)
		if (_activeConstraint.isValid()) {
			destroyGrabConstraint(world, _activeConstraint);
		}

		// Re-enable hand collision (clear bit 14) with broadphase rebuild
		if (world && hasCollisionBody()) {
			typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
			static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{
				REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

			auto* bodyArray = world->GetBodyArray();
			auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[_collisionBodyId.value]);
			auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
			auto newFilter = currentFilter & ~(1u << 14);
			setBodyCollisionFilterInfo(world, _collisionBodyId.value, newFilter, 0);  // 0 = rebuild caches
			ROCK_LOG_INFO(Hand, "{} hand: re-enabled hand collision (bit 14 cleared + broadphase rebuild) filter=0x{:08X}",
				handName(), newFilter);
		}

		// TODO Phase 4: Apply throw velocity here based on hand velocity history

		frik::api::FRIKApi::inst->clearHandPose("ROCK_Grab", handFromBool(_isLeft));
		_savedObjectState.clear();
		_activeConstraint.clear();
		_heldBodyIds.clear();
		_heldNode = nullptr;
		_currentSelection.clear();
		_state = HandState::Idle;

		ROCK_LOG_INFO(Hand, "{} hand: Idle", handName());
	}
}

#include "ObjectDetection.h"
#include "RockConfig.h"
#include "MeshGrab.h"
#include "PalmTransform.h"

#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/bhkCharacterController.h"

namespace frik::rock
{
	// =========================================================================
	// Object Filtering — isGrabbable
	//
	// WHY: Not every physics body should be grabbable. We need to exclude the
	// player, static world geometry, doors, living NPCs, workshop objects, etc.
	// This mirrors HIGGS's IsObjectSelectable/IsObjectGrabbable filtering
	// (HIGGS_DATA/subsystems/07_utilities.md) adapted for FO4VR form types.
	// =========================================================================

	bool isGrabbable(RE::TESObjectREFR* ref, RE::TESObjectREFR* otherHandRef)
	{
		if (!ref) return false;

		// Never grab ourselves
		if (ref == RE::PlayerCharacter::GetSingleton()) return false;

		// Skip deleted or disabled refs
		if (ref->IsDeleted() || ref->IsDisabled()) return false;

		// Skip if other hand already holds this
		if (otherHandRef && ref == otherHandRef) return false;

		// Check base form type — whitelist of grabbable types
		auto* baseForm = ref->GetObjectReference();
		if (!baseForm) return false;

		bool isGrabbableType =
			baseForm->Is(RE::ENUM_FORM_ID::kMISC) ||  // Junk, components, misc items
			baseForm->Is(RE::ENUM_FORM_ID::kWEAP) ||  // Weapons on the ground
			baseForm->Is(RE::ENUM_FORM_ID::kAMMO) ||  // Ammo boxes/items
			baseForm->Is(RE::ENUM_FORM_ID::kALCH) ||  // Chems, food, drinks
			baseForm->Is(RE::ENUM_FORM_ID::kBOOK) ||  // Books, magazines
			baseForm->Is(RE::ENUM_FORM_ID::kKEYM) ||  // Keys
			baseForm->Is(RE::ENUM_FORM_ID::kNOTE) ||  // Notes, holotapes
			baseForm->Is(RE::ENUM_FORM_ID::kARMO) ||  // Dropped armor pieces
			baseForm->Is(RE::ENUM_FORM_ID::kFLOR) ||  // Flora (harvestable plants)
			baseForm->Is(RE::ENUM_FORM_ID::kACTI);    // Activators with physics

		if (!isGrabbableType) return false;

		// Must have 3D loaded (can't grab invisible objects)
		auto* root3D = ref->Get3D();
		if (!root3D) return false;

		// Must have collision object (LINKED, not Unlinked)
		// Objects without physics collision can't be grabbed
		auto* collObj = root3D->collisionObject.get();
		if (!collObj) return false;

		// TODO Phase 2.5: Workshop object exclusion
		// Check if ref has WorkshopItem keyword or is linked to a workshop.
		// For now, workshop objects pass the filter — they have physics and
		// are technically grabbable. Add workshop check when we understand
		// the keyword system better.

		return true;
	}

	// =========================================================================
	// Body → Reference Resolution
	//
	// WHY: QueryAabb returns hknpBodyIds. We need to map these back to
	// TESObjectREFR for gameplay filtering. The path is:
	//   bodyId → bhkNPCollisionObject::Getbhk() → NiCollisionObject::sceneObject
	//   → walk to root NiAVObject → TESObjectREFR::FindReferenceFor3D()
	//
	// bhkNPCollisionObject::Getbhk (REL::ID 730034) is a static function that
	// looks up the collision object from a bodyId using body.userData (+0x88).
	// =========================================================================

	RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		RE::hknpBodyId bodyId)
	{
		if (!bhkWorld || !hknpWorld) return nullptr;
		if (bodyId.value == 0x7FFF'FFFF) return nullptr;

		// Validate body — check it has a reasonable motionIndex (not garbage)
		auto& body = hknpWorld->GetBody(bodyId);
		if (body.motionIndex > 4096) return nullptr;

		// Static bodies (motionIndex == 0) are world geometry — skip
		if (body.motionIndex == 0) return nullptr;

		// Skip CHARCONTROLLER(30) — these are NPC movement capsules, not touchable geometry
		auto layer = body.collisionFilterInfo & 0x7F;
		if (layer == 30) {
			return nullptr;
		}

		// Validate userData is non-null before calling Getbhk.
		// body.userData (+0x88) is the back-reference. Bodies without game objects
		// (our hand bodies, NPC ragdoll parts) have userData=0.
		if (body.userData == 0) return nullptr;

		// Use bhkNPCollisionObject::Getbhk to get the collision object from bodyId.
		// This reads body.userData (+0x88) which is the back-reference stored by
		// bhkPhysicsSystem::CreateInstance (see 02-bethesda-wrappers.md).
		auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, bodyId);
		if (!collObj) return nullptr;

		// NiCollisionObject::sceneObject (+0x10) is the owner NiAVObject
		auto* sceneObj = collObj->sceneObject;
		if (!sceneObj) return nullptr;

		// Walk up to the root 3D node and resolve to TESObjectREFR
		auto* ref = RE::TESObjectREFR::FindReferenceFor3D(sceneObj);
		return ref;
	}

	// =========================================================================
	// Near Detection — QueryAabb + Dot Product
	//
	// WHY: We need to find objects near the player's palm. A thin raycast misses
	// objects you're reaching near but not pointing directly at. QueryAabb finds
	// all bodies within a box around the palm, then we filter by:
	//   1. Palm-forward dot product (only objects in front of the hand)
	//   2. isGrabbable (form type, not player, has collision, etc.)
	//   3. Distance (closest wins)
	//
	// hknpWorld::QueryAabb is self-locking (world+0x690) and does both broadphase
	// and narrowphase testing. The query struct (hknpAabbQuery, 0x40 bytes) is
	// fully RE'd from Ghidra.
	// =========================================================================

	SelectedObject findCloseObject(
		RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward,
		float nearRange, bool isLeft, RE::TESObjectREFR* otherHandRef)
	{
		SelectedObject result;

		if (!bhkWorld || !hknpWorld) return result;

		// Build AABB in Havok space around palm position
		const float halfRange = nearRange * kGameToHavokScale;
		auto palmHk = niPointToHkVector(palmPos);

		// Build query struct
		RE::hknpAabbQuery query{};
		query.filterRef = getQueryFilterRef(hknpWorld);
		query.materialId = 0xFFFF;  // any material

		// collisionFilterInfo: determines what layers the query detects.
		// The game's own QueryAabb caller (FUN_140cac140) uses 0x0006002D (group=6, layer=45).
		// We use layer 45 with group 11 to match our hand bodies' group.
		// The collision filter checks: can query's layer collide with candidate body's layer?
		query.collisionFilterInfo = (0x000B << 16) | 45;  // group=11, layer=45

		query.aabbMin[0] = palmHk.x - halfRange;
		query.aabbMin[1] = palmHk.y - halfRange;
		query.aabbMin[2] = palmHk.z - halfRange;
		query.aabbMin[3] = 0.0f;
		query.aabbMax[0] = palmHk.x + halfRange;
		query.aabbMax[1] = palmHk.y + halfRange;
		query.aabbMax[2] = palmHk.z + halfRange;
		query.aabbMax[3] = 0.0f;

		// Execute query — self-locking, no manual lock needed
		RE::hknpAllHitsCollector collector;
		hknpWorld->QueryAabb(&query, &collector);

		// Access hit data directly from inline array (avoids virtual dispatch linker issues)
		int numHits = collector.hits._size;

		// Periodic diagnostic — log query results every ~3 seconds.
		// Per-hand counters to avoid double-counting when both hands call each frame.
		if (g_rockConfig.rockDebugVerboseLogging) {
			static int nearDiagCounterRight = 0;
			static int nearDiagCounterLeft = 0;
			int& nearDiagCounter = isLeft ? nearDiagCounterLeft : nearDiagCounterRight;
			nearDiagCounter++;
			if (nearDiagCounter >= 270) {
				nearDiagCounter = 0;
				ROCK_LOG_INFO(Hand, "Near AABB [{}]: palm=({:.2f},{:.2f},{:.2f}) halfRange={:.3f} hits={} filterRef={}",
					isLeft ? "L" : "R", palmHk.x, palmHk.y, palmHk.z, halfRange, numHits, query.filterRef);
			}
		}

		if (numHits == 0) return result;

		// Process hits — find closest grabbable object by MESH SURFACE distance.
		// The AABB query is the broadphase gate. For each candidate that passes
		// all cheap filters (motionIndex, layer, userData, isGrabbable), we extract
		// the mesh triangles and compute the closest surface point to the palm line.
		// This means a rifle whose COM is 40gu away but whose stock is 5gu from
		// your hand will be detected — COM distance is NOT used as a rejection gate.
		float bestDist = FLT_MAX;
		auto* hits = collector.hits._data;

		for (int i = 0; i < numHits; i++) {
			auto hitBodyId = hits[i].hitBodyInfo.m_bodyId;

			// Skip our own hand collision bodies
			if (hitBodyId.value == 0x7FFF'FFFF) continue;

			// Get body position from motion (for fallback distance if no mesh)
			auto* motion = hknpWorld->GetBodyMotion(hitBodyId);
			if (!motion) continue;

			// Resolve body → TESObjectREFR
			auto* ref = resolveBodyToRef(bhkWorld, hknpWorld, hitBodyId);
			if (!ref) continue;

			// Apply grabbability filter
			if (!isGrabbable(ref, otherHandRef)) continue;

			// --- Mesh surface distance (primary metric) ---
			// Extract triangles from the object's visual mesh and find the closest
			// surface point to the palm line. This is what determines "how close is
			// this object to my hand" — not the center of mass.
			float dist = FLT_MAX;
			auto* node3D = ref->Get3D();
			if (node3D) {
				std::vector<TriangleData> triangles;
				extractAllTriangles(node3D, triangles);

				if (!triangles.empty()) {
					GrabPoint grabPt;
					if (findClosestGrabPoint(triangles, palmPos, palmForward,
							1.0f, 1.0f, grabPt)) {
						// grabPt.distance is weighted distSq — use sqrt for comparison
						dist = std::sqrt(grabPt.distance);
					}
				}
			}

			// Fallback: COM distance (for objects without CPU mesh data)
			if (dist >= FLT_MAX - 1.0f) {
				RE::NiPoint3 objPos = hkVectorToNiPoint(motion->position);
				dist = (objPos - palmPos).Length();
			}

			// Reject if surface is beyond nearRange (generous — surface, not COM)
			if (dist > nearRange) continue;

			// Track closest by surface distance
			if (dist < bestDist) {
				bestDist = dist;
				result.refr = ref;
				result.bodyId = hitBodyId;
				result.distance = dist;
				result.isFarSelection = false;

				// Get the hit NiAVObject for later use
				auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
				result.hitNode = collObj ? collObj->sceneObject : nullptr;
			}
		}

		return result;
	}

	// =========================================================================
	// Far Detection — bhkPickData Ray
	//
	// WHY: For objects the player is pointing at from a distance, a thin ray is
	// appropriate (you point at things with your finger, not your palm).
	// bhkWorld::PickObject handles world locking, coordinate conversion, and
	// provides NiAVObject resolution via GetNiAVObject().
	//
	// HIGGS uses a two-phase approach (ray pre-filter + sphere sweep), but for
	// Phase 2 we start with just the ray. The sphere sweep can be added later
	// if needed for pointing accuracy at medium range.
	// =========================================================================

	SelectedObject findFarObject(
		RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir,
		float farRange, RE::TESObjectREFR* otherHandRef)
	{
		SelectedObject result;

		if (!bhkWorld || !hknpWorld) return result;

		// Build ray from hand in pointing direction
		RE::NiPoint3 rayEnd(
			handPos.x + pointingDir.x * farRange,
			handPos.y + pointingDir.y * farRange,
			handPos.z + pointingDir.z * farRange);

		// bhkPickData: Bethesda's thread-safe raycast wrapper
		RE::bhkPickData pickData;
		pickData.SetStartEnd(handPos, rayEnd);

		// Set collision filter to detect interactive objects
		// 0x02420028 is the ItemPicker filter used by the game's activation system
		pickData.collisionFilter.filter = 0x02420028;

		if (!bhkWorld->PickObject(pickData)) return result;
		if (!pickData.HasHit()) return result;

		// Resolve hit to NiAVObject
		auto* hitNiObj = pickData.GetNiAVObject();
		if (!hitNiObj) return result;

		// Resolve NiAVObject to TESObjectREFR
		auto* ref = RE::TESObjectREFR::FindReferenceFor3D(hitNiObj);
		if (!ref) return result;

		// Apply grabbability filter
		if (!isGrabbable(ref, otherHandRef)) return result;

		// Calculate hit distance
		float hitFraction = pickData.GetHitFraction();
		float hitDist = hitFraction * farRange;

		// Get the body ID from the hit (for Phase 3 grab initiation)
		RE::hknpBodyId hitBodyId{ 0x7FFF'FFFF };
		auto* hitBody = pickData.GetBody();
		if (hitBody) {
			hitBodyId = hitBody->bodyId;
		}

		result.refr = ref;
		result.bodyId = hitBodyId;
		result.hitNode = hitNiObj;
		result.distance = hitDist;
		result.isFarSelection = true;

		return result;
	}
}

#pragma once

// ObjectDetection.h — Object selection & detection for ROCK Phase 2.
//
// WHY: Before we can grab objects (Phase 3), we need to detect what the player
// is reaching toward or pointing at. This module provides:
//   1. SelectedObject struct — tracks what each hand is targeting
//   2. isGrabbable filter — determines if a ref can be picked up
//   3. resolveBodyToRef — maps hknpBodyId → TESObjectREFR
//   4. findCloseObject — near detection via QueryAabb + dot product
//   5. findFarObject — far detection via bhkPickData ray
//
// Detection approach (decided 2026-03-19, Ghidra-verified):
//   Near: hknpWorld::QueryAabb (0x40 byte struct, self-locking) + palm-forward dot product
//   Far:  bhkWorld::PickObject (thin ray, self-locking)
//   Both give proper spatial detection without manual lock management.
//
// HIGGS uses CastShape (sphere sweep) for near detection, but hknp's CastShape
// query struct is ~0x78 bytes with precomputed inverse direction fields.
// QueryAabb + dot product achieves equivalent results with a simpler, fully RE'd struct.

#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/bhkPickData.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpAabbQuery.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/Havok/hknpAllHitsCollector.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"

namespace frik::rock
{
	// =========================================================================
	// SelectedObject — tracks a candidate/selected object for one hand
	// =========================================================================

	struct SelectedObject
	{
		RE::TESObjectREFR* refr = nullptr;    ///< The selected reference
		RE::hknpBodyId bodyId{ 0x7FFF'FFFF }; ///< Specific Havok body we'd grab
		RE::NiAVObject* hitNode = nullptr;     ///< NiAVObject that owns the collision
		float distance = FLT_MAX;              ///< Distance from hand to object (game units)
		bool isFarSelection = false;           ///< True = far ray, false = near AABB

		void clear()
		{
			refr = nullptr;
			bodyId.value = 0x7FFF'FFFF;
			hitNode = nullptr;
			distance = FLT_MAX;
			isFarSelection = false;
		}

		bool isValid() const { return refr != nullptr; }
	};

	// =========================================================================
	// Query helpers
	// =========================================================================

	/// Read the collision filter reference needed by all hknpWorld query structs.
	/// Standard pattern: *(*(world+0x150) + 0x5E8). Stable across frames.
	inline void* getQueryFilterRef(RE::hknpWorld* world)
	{
		if (!world) return nullptr;
		auto dispatcherData = *reinterpret_cast<std::uintptr_t*>(
			reinterpret_cast<std::uintptr_t>(world) + 0x150);
		if (!dispatcherData) return nullptr;
		return *reinterpret_cast<void**>(dispatcherData + 0x5E8);
	}

	// =========================================================================
	// Object filtering
	// =========================================================================

	/// Check if a TESObjectREFR can be grabbed by ROCK.
	/// Filters: player, deleted/disabled, static, doors, living NPCs, PA, workshop objects.
	bool isGrabbable(RE::TESObjectREFR* ref, RE::TESObjectREFR* otherHandRef = nullptr);

	// =========================================================================
	// Body → Reference resolution
	// =========================================================================

	/// Resolve an hknpBodyId to a TESObjectREFR using bhkNPCollisionObject.
	/// Path: bodyId → bhkNPCollisionObject::Getbhk → NiCollisionObject::sceneObject → FindReferenceFor3D
	/// Returns nullptr if body has no game-object back-reference.
	RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		RE::hknpBodyId bodyId);

	// =========================================================================
	// Detection functions
	// =========================================================================

	/// Near detection: QueryAabb around palm + dot product filter.
	/// Returns the closest grabbable object within nearRange of the palm,
	/// in the forward hemisphere (dot > threshold with palm forward direction).
	SelectedObject findCloseObject(
		RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward,
		float nearRange, bool isLeft, RE::TESObjectREFR* otherHandRef = nullptr);

	/// Far detection: bhkPickData thin ray from hand pointing direction.
	/// Returns the closest grabbable object hit by a ray up to farRange.
	SelectedObject findFarObject(
		RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
		const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir,
		float farRange, RE::TESObjectREFR* otherHandRef = nullptr);
}

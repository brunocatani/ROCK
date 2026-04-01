#pragma once

// WeaponCollision.h — Equipped weapon physics presence for ROCK.
//
// WHY: Equipped weapons in FO4VR have their collision disabled by the engine
// (FUN_1410552a0). This module creates a SEPARATE KEYFRAMED physics body that
// mirrors the weapon's collision shape on layer 44, giving the weapon physical
// presence — it pushes objects, NPCs, and enables future melee/contact features.
//
// APPROACH: Clone the weapon's existing hknpShape (shared with AddReference),
// create our own body on layer 44, position it each frame via SetBodyTransform
// + velocity. This follows HIGGS's weaponBody pattern exactly (hand.cpp:720-791).
//
// SAFETY: userData=0 (FOIslandActivationListener), shape AddReference/RemoveReference,
// atomic body ID for physics thread, dual world pointer check (bhk+hknp).

#include <atomic>

#include "BethesdaPhysicsBody.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

namespace RE
{
	class NiAVObject;
	class NiNode;
	class NiTransform;
	class bhkWorld;
}

namespace frik::rock
{
	/// Collision layer for ROCK weapon collision bodies.
	/// Layer 44 (UNUSED_1) — weapon pushes world/NPCs, does NOT collide with CC(30) or hand(43).
	inline constexpr std::uint32_t ROCK_WEAPON_LAYER = 44;

	/// Equipped weapon physics collision body manager.
	///
	/// Lifecycle: init() on world creation → update() per frame → shutdown() on world teardown.
	/// The weapon body is created/destroyed based on equipped weapon changes (per-frame FormID check).
	class WeaponCollision
	{
	public:
		// --- Public API (called by PhysicsInteraction) ---

		/// Cache world pointers. Body creation deferred to update().
		void init(RE::hknpWorld* world, void* bhkWorld);

		/// Reset state without calling DestroyBodies (world may be gone).
		void shutdown();

		/// Per-frame update: detect weapon changes, create/destroy body, position body.
		/// @param world              Current hknpWorld
		/// @param weaponNode         The weapon NiNode (from FRIK skeleton), or nullptr if no weapon
		/// @param dominantHandBodyId Body ID of the dominant hand collider (to disable during weapon equip)
		/// @param dt                 Frame delta time in seconds
		void update(RE::hknpWorld* world, RE::NiAVObject* weaponNode,
			RE::hknpBodyId dominantHandBodyId, float dt);

		/// Whether a weapon collision body currently exists in the world.
		bool hasWeaponBody() const { return _weaponBody.isValid(); }

		/// Get the weapon body ID (main thread only).
		RE::hknpBodyId getWeaponBodyId() const { return _weaponBody.getBodyId(); }

		/// Thread-safe body ID read (for physics thread — processConstraintsCallback).
		std::uint32_t getWeaponBodyIdAtomic() const {
			return _weaponBodyIdAtomic.load(std::memory_order_acquire);
		}

		/// Get the BethesdaPhysicsBody (for direct API access).
		BethesdaPhysicsBody& getWeaponBody() { return _weaponBody; }

		/// Destroy the weapon body from the world (called by PhysicsInteraction::shutdown).
		void destroyWeaponBody(RE::hknpWorld* world);

	private:
		static constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

		// --- Body lifecycle ---

		void createWeaponBody(RE::hknpWorld* world, const RE::hknpShape* shape,
			const RE::NiTransform& initialTransform);

		// --- Shape discovery ---

		/// Walk the weapon NiNode tree to find the first collision shape.
		/// Returns the hknpShape* from the first bhkNPCollisionObject found, or nullptr.
		const RE::hknpShape* findWeaponShape(RE::NiAVObject* weaponNode, RE::hknpWorld* world);

		/// Recursive helper for findWeaponShape.
		const RE::hknpShape* findWeaponShapeRecursive(RE::NiAVObject* node, RE::hknpWorld* world, int depth);

		// --- Equip detection ---

		/// Get a change-detection key for the equipped weapon (0 if none/unarmed).
		/// Uses the weapon node pointer — changes on equip, unequip, swap, mod.
		std::uint64_t getEquippedWeaponKey() const;

		// --- Per-frame positioning ---

		void updateBodyTransform(RE::hknpWorld* world, const RE::NiTransform& weaponTransform, float dt);

		// --- State ---

		BethesdaPhysicsBody  _weaponBody;
		const RE::hknpShape* _cachedShape{ nullptr };
		std::uint64_t    _cachedWeaponKey{ 0 };
		RE::hknpWorld*   _cachedWorld{ nullptr };
		void*            _cachedBhkWorld{ nullptr };
		bool             _hasPrevTransform{ false };
		bool             _weaponBodyPending{ false };

		// Previous frame position for velocity computation
		RE::NiPoint3 _prevPosition{};

		// Thread-safe body ID for physics thread access
		std::atomic<std::uint32_t> _weaponBodyIdAtomic{ INVALID_BODY_ID };

		// Deferred body creation retry counter (throttles retries to ~1/sec)
		int _retryCounter{ 0 };

		// Log throttle counter — reset per-instance for fresh diagnostics each session.
		int _posLogCounter{ 0 };

		// Dominant hand collision disable state
		bool _dominantHandDisabled{ false };
		RE::hknpBodyId _disabledHandBodyId{ INVALID_BODY_ID };

		/// Disable dominant hand collision (bit 14) to prevent overlap with weapon body.
		void disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId);

		/// Re-enable dominant hand collision.
		void enableDominantHandCollision(RE::hknpWorld* world);
	};

} // namespace frik::rock

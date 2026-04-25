#pragma once

// WeaponCollision.h — Equipped weapon physics presence for ROCK.
//
// WHY: Equipped weapons in FO4VR have their collision disabled by the engine
// (FUN_1410552a0). This module creates SEPARATE KEYFRAMED physics bodies that
// mirror the weapon's source collision bodies on layer 44, giving the weapon
// physical presence across every discovered collision part.
//
// APPROACH: Clone each existing hknpShape (shared with AddReference), create our
// own bodies on layer 44, and position each frame from the corresponding source
// body transform. This follows HIGGS's weaponBody pattern (hand.cpp:720-791),
// adapted for FO4VR weapon trees that expose multiple body IDs.
//
// SAFETY: userData=0 (FOIslandActivationListener), shape AddReference/RemoveReference,
// atomic body ID for physics thread, dual world pointer check (bhk+hknp).

#include <array>
#include <atomic>
#include <cstddef>

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
		WeaponCollision();

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

		/// Whether any weapon collision body currently exists in the world.
		bool hasWeaponBody() const;

		/// Number of mirrored weapon collision bodies currently active.
		std::uint32_t getWeaponBodyCount() const;

		/// Get the primary weapon body ID (main thread only; first active body).
		RE::hknpBodyId getWeaponBodyId() const;

		/// Thread-safe primary body ID read (for legacy physics-thread callers).
		std::uint32_t getWeaponBodyIdAtomic() const;

		/// Thread-safe membership test for all mirrored weapon bodies.
		bool isWeaponBodyIdAtomic(std::uint32_t bodyId) const;

		/// Get the primary BethesdaPhysicsBody (for direct API access).
		BethesdaPhysicsBody& getWeaponBody();

		/// Destroy the weapon body from the world (called by PhysicsInteraction::shutdown).
		void destroyWeaponBody(RE::hknpWorld* world);

	private:
		static constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;
		static constexpr std::size_t MAX_WEAPON_BODIES = 16;

		struct WeaponShapeSource
		{
			const RE::hknpShape* shape{ nullptr };
			RE::hknpBodyId sourceBodyId{ INVALID_BODY_ID };
			RE::NiAVObject* sourceNode{ nullptr };
		};

		using WeaponShapeSourceList = std::array<WeaponShapeSource, MAX_WEAPON_BODIES>;

		struct WeaponBodyInstance
		{
			BethesdaPhysicsBody body;
			const RE::hknpShape* shape{ nullptr };
			RE::hknpBodyId sourceBodyId{ INVALID_BODY_ID };
			RE::NiAVObject* sourceNode{ nullptr };
			bool hasPrevTransform{ false };
			RE::NiPoint3 prevPosition{};
		};

		// --- Body lifecycle ---

		void createWeaponBodies(RE::hknpWorld* world, const WeaponShapeSourceList& sources,
			std::size_t sourceCount);
		void resetWeaponBodiesWithoutDestroy();
		void clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef);
		void clearAtomicBodyIds();
		void publishAtomicBodyIds();

		// --- Shape discovery ---

		/// Walk the weapon NiNode tree to find all source collision bodies to mirror.
		std::size_t findWeaponShapeSources(RE::NiAVObject* weaponNode, RE::hknpWorld* world,
			WeaponShapeSourceList& outSources);

		/// Recursive helper for findWeaponShapeSources.
		void findWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::hknpWorld* world,
			int depth, WeaponShapeSourceList& outSources, std::size_t& sourceCount);
		bool addWeaponShapeSource(RE::hknpWorld* world, WeaponShapeSourceList& outSources,
			std::size_t& sourceCount, RE::hknpBodyId sourceBodyId, RE::NiAVObject* sourceNode,
			int depth);
		bool sourceBodiesStillValid(RE::hknpWorld* world) const;
		bool isSourceBodyValid(RE::hknpWorld* world, const WeaponBodyInstance& instance) const;
		RE::NiTransform getSourceBodyTransform(RE::hknpWorld* world,
			const WeaponBodyInstance& instance, const RE::NiTransform& fallbackTransform) const;

		// --- Equip detection ---

		/// Get a change-detection key for the equipped weapon (0 if none/unarmed).
		/// Uses the weapon node pointer — changes on equip, unequip, swap, mod.
		std::uint64_t getEquippedWeaponKey() const;

		// --- Per-frame positioning ---

		void updateBodyTransform(RE::hknpWorld* world, WeaponBodyInstance& instance,
			const RE::NiTransform& weaponTransform, float dt, std::size_t bodyIndex);

		// --- State ---

		std::array<WeaponBodyInstance, MAX_WEAPON_BODIES> _weaponBodies{};
		std::uint64_t    _cachedWeaponKey{ 0 };
		RE::hknpWorld*   _cachedWorld{ nullptr };
		void*            _cachedBhkWorld{ nullptr };
		bool             _weaponBodyPending{ false };

		// Thread-safe body IDs for physics thread access.
		std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyIdsAtomic;
		std::atomic<std::uint32_t> _weaponBodyCountAtomic{ 0 };

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

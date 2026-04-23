#pragma once

// PhysicsInteraction.h — Main ROCK physics interaction module.
//
// WHY: This is the central coordinator for hand-based physics interaction in FO4VR.
// It owns per-hand state, manages Havok body lifecycle, tracks object ownership,
// and provides the frame update entry point called after FRIK's skeleton update.
//
// Phase 1: Collision bodies, layer registration, position tracking, contact events.
// Phase 2: Near/far object detection, selection with hysteresis, touch state, F4SE events.

#include <atomic>
#include <array>
#include <unordered_set>

#include "HandBoneCache.h"
#include "Hand.h"
#include "PhysicsLog.h"
#include "TwoHandedGrip.h"
#include "WeaponCollision.h"

namespace RE
{
	class bhkWorld;
	class hknpWorld;
	class TESObjectREFR;
}

namespace frik::rock
{
	/// F4SE message types for physics interaction events.
	/// External mods receive these via F4SE messaging when registered as a FRIK listener.
	enum PhysicsMessageType : std::uint32_t
	{
		kPhysMsg_OnTouch          = 100,
		kPhysMsg_OnTouchEnd       = 101,
		kPhysMsg_OnGrab           = 102,
		kPhysMsg_OnRelease        = 103,
		kPhysMsg_OnPhysicsInit    = 104,
		kPhysMsg_OnPhysicsShutdown = 105,
	};

	/// Data payload for physics F4SE messages. Stack-copied per dispatch.
	struct PhysicsEventData
	{
		bool              isLeft;
		RE::TESObjectREFR* refr;
		std::uint32_t     formID;
		std::uint32_t     collisionLayer;
	};

	class PhysicsInteraction
	{
	public:
		/// Global instance pointer — used by contact event callbacks to avoid
		/// use-after-free when the old hknpWorld fires events after our object is deleted.
		/// Atomic because contact callbacks fire on the Havok physics thread.
		static inline std::atomic<PhysicsInteraction*> s_instance{ nullptr };

		/// Thread-safe flag checked by hooks that run on the physics thread
		/// (HandleBumpedCharacter, UpdateShapes). Set to false BEFORE deleting
		/// the skeleton, true AFTER init completes. Prevents hooks from accessing
		/// a skeleton that's being destroyed on the main thread.
		static inline std::atomic<bool> s_hooksEnabled{ false };

		/// Per-hand API disable flags. When set, ROCK skips collision updates
		/// and detection for that hand. Other mods can take over.
		static inline std::atomic<bool> s_rightHandDisabled{ false };
		static inline std::atomic<bool> s_leftHandDisabled{ false };

		PhysicsInteraction();
		~PhysicsInteraction();

		/// Called once when the FRIK skeleton becomes ready.
		void init();

		/// Called every frame after FRIK's full skeleton update.
		void update();

		/// Shut down the physics module. Destroys Havok bodies if the world is still valid,
		/// otherwise just resets internal state. Caller does not need to know world state.
		void shutdown();

		bool isInitialized() const { return _initialized; }

		// --- Object ownership ---

		bool physicsModOwnsObject(RE::TESObjectREFR* ref) const;
		void claimObject(RE::TESObjectREFR* ref);
		void releaseObject(RE::TESObjectREFR* ref);
		void releaseAllObjects();

		// --- Grab control ---

		/// Force the specified hand to drop its held object.
		/// Handles world access internally — safe to call from the API layer.
		/// Must be called from the main thread.
		void forceDropHeldObject(bool isLeft);

		// --- Hand access ---
		Hand& getRightHand() { return _rightHand; }
		Hand& getLeftHand() { return _leftHand; }
		const Hand& getRightHand() const { return _rightHand; }
		const Hand& getLeftHand() const { return _leftHand; }

	private:
		/// Validate critical memory offsets at startup. Fails fast with clear error
		/// if memory layout changed (e.g., game update) instead of crashing during gameplay.
		bool validateCriticalOffsets() const;

		/// Resolve or refresh the local hand-bone cache against the current skeleton.
		bool refreshHandBoneCache();

		/// Temporary Pre-00 validation scaffold: compare local cached hand nodes against
		/// the FRIK API transform, then compare derived hand-basis outputs, before any
		/// gameplay writes mutate visible hand nodes.
		void sampleHandTransformParity();

		/// Visible-hand-first runtime transform source for Pre-00. Prefers the local
		/// ROCK cache, falls back to FRIK API only when the cache is not yet ready.
		RE::NiTransform getInteractionHandTransform(bool isLeft) const;

		/// Node companion to getInteractionHandTransform(). Used for debug parenting and
		/// temporary visual-hand adjustment while Pre-00 migrates off FRIK API sourcing.
		RE::NiNode* getInteractionHandNode(bool isLeft) const;

		/// Get the bhkWorld for the player's current cell.
		RE::bhkWorld* getPlayerBhkWorld() const;

		/// Get the underlying hknpWorld from a bhkWorld (at offset +0x60).
		static RE::hknpWorld* getHknpWorld(RE::bhkWorld* bhk);

		/// Register our custom collision layer in the runtime filter matrix.
		/// Sets layer 43 to collide with world geometry but not character controller.
		void registerCollisionLayer(RE::hknpWorld* world);

		/// Create collision bodies for both hands (via BethesdaPhysicsBody pipeline).
		void createHandCollisions(RE::hknpWorld* world, void* bhkWorld);

		/// Destroy collision bodies for both hands.
		void destroyHandCollisions(void* bhkWorld);

		/// Update both hand collision positions to track VR controllers.
		void updateHandCollisions(RE::hknpWorld* world);

		/// Run object detection for both hands (near AABB + far ray).
		void updateSelection(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

		/// Process grab input and per-frame held object updates for both hands.
		void updateGrabInput(RE::hknpWorld* hknp);

		/// Resolve contact body IDs from physics thread and log with names.
		void resolveContacts(RE::bhkWorld* bhk, RE::hknpWorld* hknp);

		/// Resolve a single contact body ID to a game reference and log it.
		void resolveAndLogContact(const char* handName, RE::bhkWorld* bhk,
			RE::hknpWorld* hknp, RE::hknpBodyId bodyId);

		/// Subscribe to contact events on the hknpWorld.
		void subscribeContactEvents(RE::hknpWorld* world);

		/// Dispatch a physics event via F4SE messaging (broadcast to all listeners).
		void dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft,
			RE::TESObjectREFR* refr = nullptr, std::uint32_t formID = 0,
			std::uint32_t layer = 0);

		/// Static contact callback — dispatches to instance method.
		static void onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData);

		/// Process a contact event involving one of our hand bodies.
		void handleContactEvent(void* contactEventData);

		std::atomic<bool> _initialized{ false };
		bool _collisionLayerRegistered = false;
		std::uint64_t _expectedHandLayerMask = 0;  ///< Expected mask for per-frame verification
		HandBoneCache _handBoneCache;

		Hand _rightHand{ false };
		Hand _leftHand{ true };

		/// Weapon collision body manager (equipped weapon physics presence).
		WeaponCollision _weaponCollision;

		/// Physics-based two-handed weapon grip (replaces FRIK's offhand barrel grip).
		TwoHandedGrip _twoHandedGrip;

		/// THREAD SAFETY: Main-thread-only. All access paths verified (2026-03-28):
		/// - physicsModOwnsObject/claimObject/releaseObject: called from FRIKApi (Papyrus = main thread)
		/// - Internal grab/release paths in update(): main thread
		/// - releaseAllObjects(): main thread shutdown path
		/// If any future callsite runs off-main-thread, this MUST be mutex-protected.
		/// Uses FormIDs (not raw pointers) to avoid dangling pointer bugs when the
		/// engine deletes TESObjectREFR objects during cell unloads or scripts.
		std::unordered_set<std::uint32_t> _ownedObjects;

		/// Cached bhkWorld pointer — refreshed each frame, NEVER used across frames.
		RE::bhkWorld* _cachedBhkWorld = nullptr;

		/// Frame timing for velocity computation — sourced from FRIK's getFrameTime().
		float _deltaTime = 1.0f / 90.0f;  // default to 90Hz VR

		/// Contact event logging throttle (modified on physics thread via handleContactEvent).
		std::atomic<int> _contactLogCounter{ 0 };

		/// Last contact body IDs — written on physics thread, read on main thread.
		/// 0xFFFFFFFF = no contact this frame.
		std::atomic<std::uint32_t> _lastContactBodyRight{ 0xFFFFFFFF };
		std::atomic<std::uint32_t> _lastContactBodyLeft{ 0xFFFFFFFF };

		/// Off-hand touching weapon body flag — set on physics thread when off-hand
		/// collider (layer 43) contacts weapon body (layer 44).
		/// Cleared each frame on main thread after reading.
		std::atomic<bool> _offhandTouchingWeapon{ false };

		/// Cached config values for live-reload shape rebuild detection.
		float _cachedHalfExtentX = 0.0f;
		float _cachedHalfExtentY = 0.0f;
		float _cachedHalfExtentZ = 0.0f;
		int _handCacheResolveLogCounter = 0;

		struct RawHandParityState
		{
			RE::NiTransform previousApiTransform{};
			float lastPositionDelta = 0.0f;
			float lastRotationDeltaDegrees = 0.0f;
			int warnFrames = 0;
			int failFrames = 0;
			int lagFrames = 0;
			bool hasPreviousApiTransform = false;
		};

		std::array<RawHandParityState, 2> _rawHandParityStates{};
		int _paritySummaryCounter = 0;
		bool _parityEnabledLogged = false;
		bool _runtimeScaleLogged = false;

		/// Player-space delta tracking (Task 1.9).
		RE::NiPoint3 _prevSmoothedPos;
		int _deltaLogCounter = 0;
		bool _hasPrevPositions = false;

		// Log throttle counters — reset per-instance for fresh diagnostics each session.
		int _wpnNodeLogCounter = 0;
	};
}

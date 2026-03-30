#include "PhysicsInteraction.h"

#include "HavokOffsets.h"
#include "PalmTransform.h"
#include "PhysicsHooks.h"
#include "PhysicsUtils.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/FormComponents.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Havok/hknpWorld.h"

#include "api/FRIKApi.h"
#include "ROCKMain.h"
#include "RockUtils.h"
#include "RockConfig.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "vrcf/VRControllersManager.h"

namespace frik::rock
{
	/// Build hand collision transform from raw VR wand node (HIGGS pattern).
	/// Uses raw controller tracking instead of FRIK's IK-solved position.
	/// Applies configurable hand-local offset to position the collision box
	/// over the palm/fingers instead of at the wrist bone origin.
	/// HIGGS equivalent: ComputeHandCollisionTransform (hand.cpp:522-535).
	/// Defined here (file scope) so all functions in the file can use it.
	static RE::NiTransform computeHandCollisionTransform(RE::NiNode* wandNode, bool isLeft)
	{
		RE::NiTransform result = wandNode->world;

		float offX = g_rockConfig.rockHandCollisionOffsetX * (isLeft ? -1.0f : 1.0f);
		float offY = g_rockConfig.rockHandCollisionOffsetY;
		float offZ = g_rockConfig.rockHandCollisionOffsetZ;

		const auto& R = result.rotate;
		float worldOffX = R.entry[0][0]*offX + R.entry[0][1]*offY + R.entry[0][2]*offZ;
		float worldOffY = R.entry[1][0]*offX + R.entry[1][1]*offY + R.entry[1][2]*offZ;
		float worldOffZ = R.entry[2][0]*offX + R.entry[2][1]*offY + R.entry[2][2]*offZ;

		constexpr float kHavokToGame = 70.0f;
		result.translate.x += worldOffX * kHavokToGame;
		result.translate.y += worldOffY * kHavokToGame;
		result.translate.z += worldOffZ * kHavokToGame;

		return result;
	}

	PhysicsInteraction::PhysicsInteraction()
	{
		// Frame timing now uses FRIK's getFrameTime() — no local clock init needed.
		s_instance.store(this, std::memory_order_release);
		// Install hooks (once, not per-init)
		installBumpHook();
		installCCRadiusHook();
		installNativeGrabHook();
		installRefreshManifoldHook();

		ROCK_LOG_INFO(Init, "ROCK Physics Module v0.1 — created");
	}

	PhysicsInteraction::~PhysicsInteraction()
	{
		// Clear singleton FIRST — contact callbacks on the physics thread
		// may fire during destruction. With s_instance = nullptr, they'll no-op.
		s_instance.store(nullptr, std::memory_order_release);

		if (_initialized) {
			shutdown();
		}
		ROCK_LOG_INFO(Init, "ROCK Physics Module — destroyed");
	}

	// =========================================================================
	// Lifecycle
	// =========================================================================

	bool PhysicsInteraction::validateCriticalOffsets() const
	{
		// Check that the main loop hook site contains a CALL instruction (0xE8).
		// If a game update moved it, our hook would corrupt arbitrary code.
		REL::Relocation hookSite{ REL::Offset(0xd8405e) };
		auto* hookByte = reinterpret_cast<const std::uint8_t*>(hookSite.address());
		if (*hookByte != 0xE8 && *hookByte != 0xE9) {  // CALL or JMP (may be hooked already)
			ROCK_LOG_ERROR(Init, "Hook site 0xd8405e is not a CALL/JMP instruction (found {:#x})", *hookByte);
			return false;
		}

		// Check that bhkWorld -> hknpWorld offset produces a non-null pointer.
		auto* bhk = getPlayerBhkWorld();
		if (!bhk) {
			// No world yet — can't validate. Not an error, init will retry.
			ROCK_LOG_INFO(Init, "No bhkWorld available for offset validation (will retry)");
			return true;
		}

		auto* hknp = getHknpWorld(bhk);
		if (!hknp) {
			ROCK_LOG_ERROR(Init, "bhkWorld -> hknpWorld is null — offset may be wrong");
			return false;
		}

		// Check body array is accessible.
		auto* bodyArray = hknp->GetBodyArray();
		if (!bodyArray) {
			ROCK_LOG_ERROR(Init, "hknpWorld::GetBodyArray() returned null");
			return false;
		}

		ROCK_LOG_INFO(Init, "Critical offset validation passed");
		return true;
	}

	void PhysicsInteraction::init()
	{
		if (_initialized) {
			ROCK_LOG_WARN(Init, "init() called but already initialized — skipping");
			return;
		}

		// Validate critical memory offsets before any Havok body creation.
		// Fails fast with clear error if memory layout changed.
		if (!validateCriticalOffsets()) {
			ROCK_LOG_CRITICAL(Init, "ROCK DISABLED: critical Havok offset validation failed. "
				"This likely means a game update changed memory layouts.");
			return;
		}

		ROCK_LOG_INFO(Init, "Initializing ROCK physics module...");

		auto* bhk = getPlayerBhkWorld();
		if (!bhk) {
			ROCK_LOG_ERROR(Init, "Failed to get bhkWorld during init — deferring");
			return;
		}

		auto* hknp = getHknpWorld(bhk);
		if (!hknp) {
			ROCK_LOG_ERROR(Init, "Failed to get hknpWorld during init — deferring");
			return;
		}

		_cachedBhkWorld = bhk;

		// Register our custom collision layer (once per world)
		registerCollisionLayer(hknp);

		// Create hand collision bodies
		createHandCollisions(hknp);
		_cachedHalfExtentX = g_rockConfig.rockHandCollisionHalfExtentX;
		_cachedHalfExtentY = g_rockConfig.rockHandCollisionHalfExtentY;
		_cachedHalfExtentZ = g_rockConfig.rockHandCollisionHalfExtentZ;

		// Subscribe to contact events for touch detection
		subscribeContactEvents(hknp);

		// Initialize weapon collision system
		_weaponCollision.init(hknp);

		// Kill FRIK's offhand grip permanently — ROCK owns weapon interaction now
		if (frik::api::FRIKApi::inst) {
			frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", true);
			ROCK_LOG_INFO(Init, "FRIK offhand grip permanently suppressed");
		}

		// After body creation, call SetBodyTransform to establish correct broadphase position.
		// CreateBody places the body at origin (0,0,0) in the broadphase. SetBodyTransform
		// updates the motion position AND the broadphase tree via synchronizeBodiesFromMotion.
		{
			// Use raw wand nodes for initial positioning (consistent with per-frame path)
			auto* rWandInit = f4vr::getRightHandNode();
			auto* lWandInit = f4vr::getLeftHandNode();
			if (rWandInit && lWandInit) {
				auto rTransform = computeHandCollisionTransform(rWandInit, false);
				auto lTransform = computeHandCollisionTransform(lWandInit, true);
				_rightHand.updateCollisionTransform(hknp, rTransform, 0.011f);
				_leftHand.updateCollisionTransform(hknp, lTransform, 0.011f);
			} else if (frik::api::FRIKApi::inst) {
				// Fallback to FRIK API if wand nodes not yet available at init
				auto rTransform = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right);
				auto lTransform = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left);
				_rightHand.updateCollisionTransform(hknp, rTransform, 0.011f);
				_leftHand.updateCollisionTransform(hknp, lTransform, 0.011f);
			}
			ROCK_LOG_INFO(Init, "Initial hand positions set via SetBodyTransform");
		}

		// Reset delta tracking
		_hasPrevPositions = false;
		_deltaLogCounter = 0;
		_contactLogCounter = 0;

		_initialized = true;

		dispatchPhysicsMessage(kPhysMsg_OnPhysicsInit, false);

		ROCK_LOG_INFO(Init, "ROCK physics module initialized — bhkWorld={}, hknpWorld={}, R_body={}, L_body={}",
			static_cast<const void*>(bhk), static_cast<const void*>(hknp),
			_rightHand.getCollisionBodyId().value, _leftHand.getCollisionBodyId().value);
	}

	void PhysicsInteraction::update()
	{
		// CRITICAL: Update VR controller state for this frame.
		// Each DLL has its own copy of the VRControllers inline global (separate instances
		// per module). FRIK updates its copy in ModBase::onFrameUpdateSafe(), but ROCK
		// has its own copy that must be updated independently. Without this, isPressed()
		// and isReleased() always return false — grab never triggers.
		vrcf::VRControllers.update(f4vr::isLeftHandedMode());

		// Use FRIK's canonical frame time — both DLLs share the same clock source.
		// This prevents physics/skeleton drift from independent timers.
		_deltaTime = frik::api::FRIKApi::inst->getFrameTime();

		// Safety clamp: getFrameTime() returns 0 during init or on error.
		if (_deltaTime <= 0.0f || _deltaTime > 0.1f) {
			_deltaTime = 1.0f / 90.0f;
		}

		// Safety: don't run if skeleton isn't ready
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
			if (_initialized) {
				ROCK_LOG_WARN(Update, "Skeleton no longer ready — shutting down");
				shutdown();
			}
			return;
		}

		// Don't update during menus — release held objects first
		if (frik::api::FRIKApi::inst->isAnyMenuOpen()) {
			if (_initialized) {
				auto* bhkMenu = getPlayerBhkWorld();
				if (bhkMenu) {
					auto* hknpMenu = getHknpWorld(bhkMenu);
					if (hknpMenu) {
						if (_rightHand.isHolding()) _rightHand.releaseGrabbedObject(hknpMenu);
						if (_leftHand.isHolding()) _leftHand.releaseGrabbedObject(hknpMenu);
					}
				}
			}
			return;
		}

		// Skip if physics interaction is disabled in config
		if (!g_rockConfig.rockEnabled) {
			return;
		}

		// Refresh bhkWorld pointer every frame
		auto* bhk = getPlayerBhkWorld();
		if (!bhk) {
			if (_initialized) {
				ROCK_LOG_WARN(Update, "bhkWorld became null — shutting down");
				shutdown();
			}
			return;
		}

		// Detect cell change: bhkWorld changed
		if (_initialized && bhk != _cachedBhkWorld) {
			ROCK_LOG_INFO(Update, "bhkWorld changed (cell transition) — reinitializing");
			// The old bhkWorld/hknpWorld is already destroyed or being destroyed.
			// Do NOT call DestroyBodies on the old world — it would dereference freed memory.
			// Just reset our state; the old world cleans up its own bodies.
			shutdown();
		}

		// Lazy init if not yet initialized
		if (!_initialized) {
			init();
			if (!_initialized) {
				return;
			}
		}

		_cachedBhkWorld = bhk;

		auto* hknp = getHknpWorld(bhk);
		if (!hknp) {
			return;
		}

		// --- Per-frame collision layer verification (HIGGS EnsureHiggsCollisionLayer pattern) ---
		// The game may reset the collision filter matrix on cell load or world recreation.
		// Check our layer's mask every frame and re-register if it changed.
		// Reference: HIGGS S09 section 9.2, FRIK_STUDY/ADD_TO_PLAN.md section S01-1.9.2
		if (_collisionLayerRegistered && _expectedHandLayerMask != 0) {
			auto dispatcherData = *reinterpret_cast<std::uintptr_t*>(
				reinterpret_cast<std::uintptr_t>(hknp) + offsets::kHknpWorld_EventDispatcher);
			if (dispatcherData) {
				auto* filterPtr = *reinterpret_cast<void**>(dispatcherData + offsets::kDispatcher_FilterPtr);
				if (filterPtr) {
					auto* matrix = reinterpret_cast<std::uint64_t*>(
						reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);
					if (matrix[ROCK_HAND_LAYER] != _expectedHandLayerMask) {
						ROCK_LOG_WARN(Config, "Layer {} mask changed! Expected=0x{:016X} Current=0x{:016X} — re-registering",
							ROCK_HAND_LAYER, _expectedHandLayerMask, matrix[ROCK_HAND_LAYER]);
						_collisionLayerRegistered = false;
						registerCollisionLayer(hknp);
					}
				}
			}
		}

		// --- Check for config changes (live reload) ---
		// If collision shape size changed in INI, rebuild bodies
		if (_cachedHalfExtentX != g_rockConfig.rockHandCollisionHalfExtentX ||
			_cachedHalfExtentY != g_rockConfig.rockHandCollisionHalfExtentY ||
			_cachedHalfExtentZ != g_rockConfig.rockHandCollisionHalfExtentZ) {
			ROCK_LOG_INFO(Config, "Collision box config changed — rebuilding (X={:.4f}→{:.4f}, Y={:.4f}→{:.4f}, Z={:.4f}→{:.4f})",
				_cachedHalfExtentX, g_rockConfig.rockHandCollisionHalfExtentX,
				_cachedHalfExtentY, g_rockConfig.rockHandCollisionHalfExtentY,
				_cachedHalfExtentZ, g_rockConfig.rockHandCollisionHalfExtentZ);
			// Release any held objects before destroying hand bodies (constraint references bodyA)
			if (_rightHand.isHolding()) _rightHand.releaseGrabbedObject(hknp);
			if (_leftHand.isHolding()) _leftHand.releaseGrabbedObject(hknp);
			destroyHandCollisions(hknp);  // also destroys debug spheres
			createHandCollisions(hknp);
			_cachedHalfExtentX = g_rockConfig.rockHandCollisionHalfExtentX;
			_cachedHalfExtentY = g_rockConfig.rockHandCollisionHalfExtentY;
			_cachedHalfExtentZ = g_rockConfig.rockHandCollisionHalfExtentZ;
		}

		// --- Per-frame hand collision position update ---
		updateHandCollisions(hknp);

		// --- Weapon collision update ---
		// Find weapon node using VR-Reloads approach: search player->unkF0->rootNode
		// for a child named "Weapon". This is the actual weapon 3D root with collision
		// objects (P-Receiver, WeaponBolt, WeaponMagazine, etc.).
		{
			auto* rootNode = f4vr::getRootNode();
			RE::NiAVObject* weaponNode = rootNode ? f4vr::findNode(rootNode, "Weapon") : nullptr;
			// Pass dominant hand body ID so weapon collision can disable it during equip
			bool isLeftHanded = f4vr::isLeftHandedMode();
			auto dominantHandBodyId = isLeftHanded
				? _leftHand.getCollisionBodyId()
				: _rightHand.getCollisionBodyId();
			// Diagnostic: log weapon node status once per second
			if (g_rockConfig.rockDebugVerboseLogging) {
				if (++_wpnNodeLogCounter >= 90) {
					_wpnNodeLogCounter = 0;
					if (weaponNode) {
						ROCK_LOG_INFO(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={}",
							weaponNode->name.c_str(),
							weaponNode->world.translate.x, weaponNode->world.translate.y, weaponNode->world.translate.z,
							_weaponCollision.hasWeaponBody());
					} else {
						ROCK_LOG_INFO(Weapon, "WeaponNode: NULL hasBody={}", _weaponCollision.hasWeaponBody());
					}
				}
			}
			_weaponCollision.update(hknp, weaponNode, dominantHandBodyId, _deltaTime);
		}

		// --- Two-handed weapon grip update ---
		{
			bool offhandTouching = _offhandTouchingWeapon.exchange(false, std::memory_order_acquire);
			bool isLeftHanded = f4vr::isLeftHandedMode();
			bool gripPressed = vrcf::VRControllers.isPressHeldDown(vrcf::Hand::Offhand, g_rockConfig.rockGrabButtonID);

			auto* rootNode2 = f4vr::getRootNode();
			RE::NiNode* weaponNode = rootNode2 ? f4vr::findNode(rootNode2, "Weapon") : nullptr;

			_twoHandedGrip.update(weaponNode, offhandTouching, gripPressed,
				isLeftHanded, _deltaTime);
		}

		// --- Per-frame object detection (Phase 2) ---
		updateSelection(bhk, hknp);

		// --- Grab input + held object update (Phase 3) ---
		updateGrabInput(hknp);

		// --- Contact resolution (main thread — resolves body IDs from physics thread) ---
		resolveContacts(bhk, hknp);

		// Tick touch state and detect touch-end transitions for F4SE events
		bool wasTouchingR = _rightHand.isTouching();
		bool wasTouchingL = _leftHand.isTouching();
		_rightHand.tickTouchState();
		_leftHand.tickTouchState();
		if (wasTouchingR && !_rightHand.isTouching()) {
			dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, false,
				_rightHand.getLastTouchedRef(), _rightHand.getLastTouchedFormID(),
				_rightHand.getLastTouchedLayer());
		}
		if (wasTouchingL && !_leftHand.isTouching()) {
			dispatchPhysicsMessage(kPhysMsg_OnTouchEnd, true,
				_leftHand.getLastTouchedRef(), _leftHand.getLastTouchedFormID(),
				_leftHand.getLastTouchedLayer());
		}

		// --- Player-space delta logging (Task 1.9) ---
		_deltaLogCounter++;
		if (g_rockConfig.rockDebugVerboseLogging && _deltaLogCounter >= 90) {
			_deltaLogCounter = 0;

			if (frik::api::FRIKApi::inst) {
				const auto smoothPos = frik::api::FRIKApi::inst->getSmoothedPlayerPosition();
				const bool moving = frik::api::FRIKApi::inst->isPlayerMoving();

				if (_hasPrevPositions && moving) {
					const auto smoothDelta = smoothPos - _prevSmoothedPos;

					ROCK_LOG_DEBUG(Update, "PlayerSpace: smoothDelta=({:.2f},{:.2f},{:.2f}) moving={}",
						smoothDelta.x, smoothDelta.y, smoothDelta.z, moving);
				}

				_prevSmoothedPos = smoothPos;
				_hasPrevPositions = true;
			}
		}
	}

	void PhysicsInteraction::shutdown()
	{
		if (!_initialized) {
			return;
		}

		// 1. Notify listeners BEFORE any cleanup (FRIK pattern: event before delete)
		dispatchPhysicsMessage(kPhysMsg_OnPhysicsShutdown, false);

		ROCK_LOG_INFO(Init, "Shutting down ROCK physics module...");

		// 2. Havok cleanup — only if the world is still valid and matches our cache.
		//    On cell transitions the world is already dead; skip body operations.
		auto* currentBhk = getPlayerBhkWorld();
		const bool worldValid = _cachedBhkWorld && currentBhk == _cachedBhkWorld;

		if (worldValid) {
			auto* hknp = getHknpWorld(_cachedBhkWorld);
			if (_rightHand.isHolding()) _rightHand.releaseGrabbedObject(hknp);
			if (_leftHand.isHolding()) _leftHand.releaseGrabbedObject(hknp);
			_weaponCollision.destroyWeaponBody(hknp);
			destroyHandCollisions(hknp);
		} else {
			ROCK_LOG_INFO(Init, "World stale or null — skipping Havok body destruction");
		}

		// 3. Common cleanup — always runs regardless of world state.
		_rightHand.destroyDebugColliderVis();
		_leftHand.destroyDebugColliderVis();
		_twoHandedGrip.reset();
		_weaponCollision.shutdown();
		releaseAllObjects();
		_rightHand.reset();
		_leftHand.reset();

		// 4. Reset state
		_cachedBhkWorld = nullptr;
		_collisionLayerRegistered = false;
		_initialized = false;
		_hasPrevPositions = false;

		// 5. Free vtable shellcode (prevents executable memory leak on DLL hot-reload)
		cleanupGrabConstraintVtable();

		ROCK_LOG_INFO(Init, "ROCK physics module shut down");
	}

	// =========================================================================
	// F4SE Message Dispatch
	// =========================================================================

	void PhysicsInteraction::dispatchPhysicsMessage(std::uint32_t msgType, bool isLeft,
		RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer)
	{
		PhysicsEventData data{ isLeft, refr, formID, layer };
		// Dispatch through ROCK's own F4SE messaging interface
		if (auto* m = ::rock::getROCKMessaging()) {
			m->Dispatch(msgType, &data, sizeof(data), nullptr);
		}
	}

	// =========================================================================
	// Collision Layer Registration
	// =========================================================================

	void PhysicsInteraction::registerCollisionLayer(RE::hknpWorld* world)
	{
		// Phase 0A fix: Read collision filter from the hknpWorld's dispatcher,
		// NOT from the global singleton. After world recreation (cell load), the new
		// hknpWorld may use a different filter object than the persistent global singleton
		// at REL::Offset(0x59429B8). The global singleton is created once in
		// bhkWorld__InitSystem and never recreated, but the new world's dispatcher may
		// reference a fresh filter where our custom layers are unconfigured.
		//
		// Path: *(*(world + 0x150) + 0x5E8) → bhkCollisionFilter*
		// Same path used by ObjectDetection.h:getQueryFilterRef for query filters.
		// Matrix at filter + 0x1A0: uint64_t[64] layer collision bitfield.
		//
		// HIGGS pattern: EnsureHiggsCollisionLayer checks the mask EVERY FRAME
		// and re-registers if it changed (cell load, mod interference).
		// We remove the _collisionLayerRegistered early-return to allow re-registration.
		//
		// Reference: HIGGS S09 section 9.2, FRIK_STUDY/PHASE0A_layer_fix_analysis.md

		if (!world) {
			ROCK_LOG_ERROR(Config, "registerCollisionLayer: world is null");
			return;
		}

		// Read filter from the world's dispatcher (not global singleton)
		auto dispatcherData = *reinterpret_cast<std::uintptr_t*>(
			reinterpret_cast<std::uintptr_t>(world) + offsets::kHknpWorld_EventDispatcher);
		if (!dispatcherData) {
			ROCK_LOG_ERROR(Config, "World dispatcher data (+0x150) is null — cannot configure layer");
			return;
		}

		auto* filterPtr = *reinterpret_cast<void**>(dispatcherData + offsets::kDispatcher_FilterPtr);
		if (!filterPtr) {
			// Fallback: try the global singleton (may be the same object on first init)
			static REL::Relocation<void**> filterSingleton{ REL::Offset(0x59429B8) };
			filterPtr = *filterSingleton;
			if (!filterPtr) {
				ROCK_LOG_ERROR(Config, "Both world filter and global singleton are null — cannot configure layer");
				return;
			}
			ROCK_LOG_WARN(Config, "World filter null, fell back to global singleton at {:p}",
				filterPtr);
		}

		// Log which filter we're using for debugging
		{
			static REL::Relocation<void**> filterSingleton{ REL::Offset(0x59429B8) };
			auto* globalFilter = *filterSingleton;
			ROCK_LOG_INFO(Config, "Filter source: world={:p}, global={:p}, same={}",
				filterPtr, (void*)globalFilter, filterPtr == globalFilter);
		}

		auto* matrix = reinterpret_cast<std::uint64_t*>(
			reinterpret_cast<std::uintptr_t>(filterPtr) + offsets::kFilter_CollisionMatrix);

		auto disablePair = [&](std::uint32_t layerA, std::uint32_t layerB) {
			matrix[layerA] &= ~(1ULL << layerB);
			matrix[layerB] &= ~(1ULL << layerA);
		};

		auto enablePair = [&](std::uint32_t layerA, std::uint32_t layerB) {
			matrix[layerA] |= (1ULL << layerB);
			matrix[layerB] |= (1ULL << layerA);
		};

		// ALWAYS set layer 43 and 44 from scratch — don't rely on previous state.
		// After world recreation, the filter may be fresh (all zeros) or stale (old data).
		// HIGGS pattern: set everything we need explicitly, don't depend on InitFlags.

		ROCK_LOG_INFO(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_HAND_LAYER, matrix[ROCK_HAND_LAYER]);
		ROCK_LOG_INFO(Config, "Layer {} pre-set mask=0x{:016X}", ROCK_WEAPON_LAYER, matrix[ROCK_WEAPON_LAYER]);

		// --- Layer 43 (ROCK_HAND_LAYER): collides with everything except specific exclusions ---
		// Start by enabling collision with all layers 0-47
		for (std::uint32_t i = 0; i < 48; i++) {
			enablePair(ROCK_HAND_LAYER, i);
		}
		// Then disable specific pairs
		disablePair(ROCK_HAND_LAYER, 15);  // NONCOLLIDABLE
		disablePair(ROCK_HAND_LAYER, ROCK_HAND_LAYER);  // hand-hand (layer 43-43)
		if (!g_rockConfig.rockCollideWithCharControllers) {
			disablePair(ROCK_HAND_LAYER, 30);  // CC
		}

		// --- Layer 44 (ROCK_WEAPON_LAYER): pushes world objects/NPCs ---
		for (std::uint32_t i = 0; i < 48; i++) {
			enablePair(ROCK_WEAPON_LAYER, i);
		}
		disablePair(ROCK_WEAPON_LAYER, 0);   // UNIDENTIFIED
		disablePair(ROCK_WEAPON_LAYER, 15);  // NONCOLLIDABLE
		disablePair(ROCK_WEAPON_LAYER, 30);  // CHARCONTROLLER — prevents CC push
		// NOTE: 43↔44 collision is ENABLED — off-hand must detect weapon body for two-handed grip.
		disablePair(ROCK_WEAPON_LAYER, ROCK_WEAPON_LAYER); // self (weapon-weapon, layer 44-44)
		disablePair(ROCK_WEAPON_LAYER, 36);  // CAMERASPHERE
		disablePair(ROCK_WEAPON_LAYER, 41);  // LINEOFSIGHT
		disablePair(ROCK_WEAPON_LAYER, 42);  // PATHPICK

		// Store the expected mask for per-frame verification
		_expectedHandLayerMask = matrix[ROCK_HAND_LAYER];
		_collisionLayerRegistered = true;

		ROCK_LOG_INFO(Config, "Registered layer {} (hand) mask=0x{:016X}", ROCK_HAND_LAYER, matrix[ROCK_HAND_LAYER]);
		ROCK_LOG_INFO(Config, "Registered layer {} (weapon) mask=0x{:016X}", ROCK_WEAPON_LAYER, matrix[ROCK_WEAPON_LAYER]);
	}

	// =========================================================================
	// Hand Collision Bodies
	// =========================================================================

	void PhysicsInteraction::createHandCollisions(RE::hknpWorld* world)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
			ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
			return;
		}

		RE::hkVector4f identityQuat(0.0f, 0.0f, 0.0f, 1.0f);

		_rightHand.createCollision(world, identityQuat,
			g_rockConfig.rockHandCollisionHalfExtentX,
			g_rockConfig.rockHandCollisionHalfExtentY,
			g_rockConfig.rockHandCollisionHalfExtentZ);

		_leftHand.createCollision(world, identityQuat,
			g_rockConfig.rockHandCollisionHalfExtentX,
			g_rockConfig.rockHandCollisionHalfExtentY,
			g_rockConfig.rockHandCollisionHalfExtentZ);
	}

	void PhysicsInteraction::destroyHandCollisions(RE::hknpWorld* world)
	{
		_rightHand.destroyDebugColliderVis();
		_leftHand.destroyDebugColliderVis();
		_rightHand.destroyCollision(world);
		_leftHand.destroyCollision(world);
	}

	void PhysicsInteraction::updateHandCollisions(RE::hknpWorld* world)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
			return;
		}

		bool rightDisabled = s_rightHandDisabled.load(std::memory_order_acquire);
		bool leftDisabled = s_leftHandDisabled.load(std::memory_order_acquire);

		// --- HIGGS pattern: Use raw VR wand nodes for collision body positioning ---
		// HIGGS deliberately reads the hand transform BEFORE VRIK runs (hand.cpp:4100):
		//   "handTransform must be where the hand is in real life, as the hand
		//    collision is what drives the grab constraint."
		// The collision body should track the REAL controller position, not where
		// FRIK's IK places the visible hand. This ensures the constraint drives
		// the grabbed object to where the player's physical hand actually is.
		auto* rightWand = f4vr::getRightHandNode();
		auto* leftWand = f4vr::getLeftHandNode();

		// --- Diagnostic: periodic wand axis logging for axis convention mapping ---
		// Logs every ~2 seconds (180 frames at 90fps) during the first 60 seconds.
		// Hold controller in known orientations while watching the log:
		//   1. Point forward (fingertips away from you) — which column ≈ player facing?
		//   2. Palm flat down — which column ≈ -Z (world down)?
		//   3. Thumb up — which column ≈ +Z (world up)?
		// FO4VR world: +X=East, +Y=North, +Z=Up (Bethesda convention)
		if (g_rockConfig.rockDebugVerboseLogging) {
			_wandDiagFrame++;
			if (leftWand && (_wandDiagFrame % 180 == 0) && _wandDiagFrame < 5400) {
				const auto& R = leftWand->world.rotate;
				auto wandPos = leftWand->world.translate;
				auto frikPos = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left).translate;
				float dx = wandPos.x - frikPos.x;
				float dy = wandPos.y - frikPos.y;
				float dz = wandPos.z - frikPos.z;
				float dist = sqrtf(dx*dx + dy*dy + dz*dz);

				// Log wand axes as named directions for easier reading
				// Each column is a local axis expressed in world space
				ROCK_LOG_INFO(Init,
					"WAND_L[{:.0f}s] pos=({:.0f},{:.0f},{:.0f}) "
					"X=[{:.2f},{:.2f},{:.2f}] Y=[{:.2f},{:.2f},{:.2f}] Z=[{:.2f},{:.2f},{:.2f}] "
					"frikDist={:.0f}",
					_wandDiagFrame / 90.0f,
					wandPos.x, wandPos.y, wandPos.z,
					R.entry[0][0], R.entry[1][0], R.entry[2][0],  // X axis in world
					R.entry[0][1], R.entry[1][1], R.entry[2][1],  // Y axis in world
					R.entry[0][2], R.entry[1][2], R.entry[2][2],  // Z axis in world
					dist);
			}
		}

		// Build collision transforms from raw wand nodes with hand-local offset
		RE::NiTransform rightTransform, leftTransform;

		if (rightWand) {
			rightTransform = computeHandCollisionTransform(rightWand, false);
		} else {
			// Fallback to FRIK API if wand node unavailable
			rightTransform = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right);
			rightTransform.translate = computePalmPosition(rightTransform, false);
		}

		if (leftWand) {
			leftTransform = computeHandCollisionTransform(leftWand, true);
		} else {
			leftTransform = frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left);
			leftTransform.translate = computePalmPosition(leftTransform, true);
		}

		if (!rightDisabled) {
			_rightHand.updateCollisionTransform(world, rightTransform, _deltaTime);
			// Debug mesh attaches to wand node (not FRIK bone) so it tracks the
			// collision body correctly. Falls back to FRIK bone if wand unavailable.
			auto* rightDebugParent = rightWand ? static_cast<RE::NiNode*>(rightWand) : frik::api::FRIKApi::inst->getHandNode(frik::api::FRIKApi::Hand::Right);
			_rightHand.updateDebugColliderVis(rightTransform.translate, g_rockConfig.rockDebugShowColliders,
				rightDebugParent,
				g_rockConfig.rockHandCollisionHalfExtentX,
				g_rockConfig.rockHandCollisionHalfExtentY,
				g_rockConfig.rockHandCollisionHalfExtentZ,
				g_rockConfig.rockDebugColliderShape);
		}
		if (!leftDisabled) {
			_leftHand.updateCollisionTransform(world, leftTransform, _deltaTime);
			auto* leftDebugParent = leftWand ? static_cast<RE::NiNode*>(leftWand) : frik::api::FRIKApi::inst->getHandNode(frik::api::FRIKApi::Hand::Left);
			_leftHand.updateDebugColliderVis(leftTransform.translate, g_rockConfig.rockDebugShowColliders,
				leftDebugParent,
				g_rockConfig.rockHandCollisionHalfExtentX,
				g_rockConfig.rockHandCollisionHalfExtentY,
				g_rockConfig.rockHandCollisionHalfExtentZ,
				g_rockConfig.rockDebugColliderShape);
		}

	}

	// =========================================================================
	// Object Detection (Phase 2)
	// =========================================================================

	void PhysicsInteraction::updateSelection(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) return;

		// Palm transforms from raw wand nodes (consistent with collision body source).
		// Forward direction derived from wand Y axis, not FRIK finger bone.
		auto* rightWandSel = f4vr::getRightHandNode();
		auto* leftWandSel = f4vr::getLeftHandNode();

		RE::NiTransform rightTransform = rightWandSel
			? computeHandCollisionTransform(rightWandSel, false)
			: frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Right);
		auto rightPalmPos = computePalmPosition(rightTransform, false);
		auto rightPalmFwd = computePalmForward(rightTransform, false);
		rightTransform.translate = rightPalmPos;

		RE::NiTransform leftTransform = leftWandSel
			? computeHandCollisionTransform(leftWandSel, true)
			: frik::api::FRIKApi::inst->getHandWorldTransform(frik::api::FRIKApi::Hand::Left);
		auto leftPalmPos = computePalmPosition(leftTransform, true);
		auto leftPalmFwd = computePalmForward(leftTransform, true);
		leftTransform.translate = leftPalmPos;

		// Other hand's selection (for mutual exclusion)
		auto* rightHeldRef = _rightHand.hasSelection() ? _rightHand.getSelection().refr : nullptr;
		auto* leftHeldRef = _leftHand.hasSelection() ? _leftHand.getSelection().refr : nullptr;

		// Update each hand's selection
		if (!s_rightHandDisabled.load(std::memory_order_acquire)) {
			_rightHand.updateSelection(bhk, hknp,
				rightPalmPos, rightPalmFwd,
				g_rockConfig.rockNearDetectionRange, g_rockConfig.rockFarDetectionRange,
				leftHeldRef);
		}

		if (!s_leftHandDisabled.load(std::memory_order_acquire)) {
			_leftHand.updateSelection(bhk, hknp,
				leftPalmPos, leftPalmFwd,
				g_rockConfig.rockNearDetectionRange, g_rockConfig.rockFarDetectionRange,
				rightHeldRef);
		}
	}

	// =========================================================================
	// Grab Input & Held Object Update (Phase 3)
	// =========================================================================

	void PhysicsInteraction::updateGrabInput(RE::hknpWorld* hknp)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) return;

		int grabButton = g_rockConfig.rockGrabButtonID;

		// Process each hand
		auto processHand = [&](Hand& hand, bool isLeft) {
			if (isLeft && s_leftHandDisabled.load(std::memory_order_acquire)) return;
			if (!isLeft && s_rightHandDisabled.load(std::memory_order_acquire)) return;

			vrcf::Hand vrHand = isLeft ? vrcf::Hand::Left : vrcf::Hand::Right;

			if (hand.isHolding()) {
				// --- Currently holding: check for release ---
				if (vrcf::VRControllers.isReleased(vrHand, grabButton)) {
					auto* heldRef = hand.getHeldRef();  // Save before release clears it
					auto heldFormID = heldRef ? heldRef->GetFormID() : 0u;
					hand.releaseGrabbedObject(hknp);
					if (heldRef) releaseObject(heldRef);  // Remove from ownership set
					dispatchPhysicsMessage(kPhysMsg_OnRelease, isLeft, heldRef, heldFormID, 0);
				} else {
					// --- Per-frame held object update ---
					// Use RAW wand transform (no collision body offset) for grab computation.
					// HIGGS uses the raw hand bone for grab snapshot + pivotB formula.
					// The collision body offset positions the physics box (collision detection).
					// The palm offset (palmHS) positions the constraint pivot (grab anchoring).
					// These are SEPARATE — stacking them double-counts the forward offset.
					auto* wandNode = isLeft ? f4vr::getLeftHandNode() : f4vr::getRightHandNode();
					auto transform = wandNode
						? wandNode->world
						: frik::api::FRIKApi::inst->getHandWorldTransform(handFromBool(isLeft));
					hand.updateHeldObject(hknp, transform, _deltaTime,
						g_rockConfig.rockGrabForceFadeInTime,
						g_rockConfig.rockGrabTauMin, g_rockConfig.rockGrabTauMax,
						2.0f,   // tauIncrement per second
						5.0f,   // tauDecrement per second
						g_rockConfig.rockGrabCloseThreshold,
						g_rockConfig.rockGrabFarThreshold);

					// --- Visual hand adjustment (HIGGS pattern) ---
					// Phase 0B.NEW: Use hand node from FRIK API.
					// PRE-2 finding: FRIK renders the body from the 3P skeleton (BSFlattenedBoneTree).
					// The API's getHandNode() returns the appropriate bone for visual updates.
					// Reference: FRIK_STUDY/PRE2_updateWorldFinal_RESULT.md
					RE::NiTransform adjustedHand;
					if (hand.getAdjustedHandTransform(adjustedHand)) {
						auto handEnum = handFromBool(isLeft);
						auto* handNode = frik::api::FRIKApi::inst->getHandNode(handEnum);

						if (handNode) {
							handNode->world.translate = adjustedHand.translate;
							handNode->world.rotate = adjustedHand.rotate;
							f4vr::updateTransformsDown(handNode, false);
						}
					}
				}
			} else if (hand.getState() == HandState::SelectedClose && hand.hasSelection()) {
				// --- Has selection: check for grab ---
				if (vrcf::VRControllers.isPressed(vrHand, grabButton)) {
					// Block far-grab for truly distant objects (>50 game units).
					// Near-ish objects detected by raycast (within arm's reach) are allowed.
					// Phase 4 will add pull-to-hand for distant objects.
					if (hand.getSelection().isFarSelection && hand.getSelection().distance > 50.0f) {
						ROCK_LOG_INFO(Hand, "{} hand: far grab blocked (dist={:.1f}) — too far",
							hand.handName(), hand.getSelection().distance);
						return;
					}

					// Decision tree: some objects should activate, not grab
					auto* selRef = hand.getSelection().refr;
					if (selRef) {
						auto* baseObj = selRef->GetObjectReference();
						if (baseObj) {
							const char* typeStr = baseObj->GetFormTypeString();
							// Skip grab for non-physics interactables
							if (typeStr) {
								std::string_view t(typeStr);
								if (t == "DOOR" || t == "CONT" || t == "TERM" || t == "FURN") {
									return;  // Let normal activation handle these
								}
								// ACTI (activators): only grab if DYNAMIC motion.
								// Keyframed ACTIs (buttons, levers) should activate, not grab.
								if (t == "ACTI") {
									auto& sel = hand.getSelection();
									if (sel.bodyId.value != 0x7FFF'FFFF) {
										auto* motion = hknp->GetBodyMotion(sel.bodyId);
										if (motion) {
											auto motionProps = *reinterpret_cast<std::uint16_t*>(
												reinterpret_cast<char*>(motion) + offsets::kMotion_PropertiesId);
											if (motionProps == 2 || motionProps == 0) {
												return;  // KEYFRAMED or STATIC — activate, don't grab
											}
										}
									}
								}
							}
							// Skip alive NPCs (dead = grab body, alive = talk)
							if (std::string_view(baseObj->GetFormTypeString()) == "NPC_") {
								if (!selRef->IsDead(false)) return;
							}
						}
					}

					// Use RAW wand transform (no collision body offset) for grab initiation.
					// Same reasoning as per-frame update: collision offset and palm offset
					// are separate. The grab snapshot uses raw wand (like HIGGS uses raw bone).
					auto* grabWandNode = isLeft ? f4vr::getLeftHandNode() : f4vr::getRightHandNode();
					auto transform = grabWandNode
						? grabWandNode->world
						: frik::api::FRIKApi::inst->getHandWorldTransform(handFromBool(isLeft));

					bool grabbed = hand.grabSelectedObject(hknp, transform,
						g_rockConfig.rockGrabLinearTau,
						g_rockConfig.rockGrabLinearDamping,
						g_rockConfig.rockGrabConstraintMaxForce,
						g_rockConfig.rockGrabLinearProportionalRecovery,
						g_rockConfig.rockGrabLinearConstantRecovery);

					if (grabbed) {
						// Add to ownership set so FRIK/native don't interfere
						auto* heldRef = hand.getHeldRef();
						claimObject(heldRef);
						dispatchPhysicsMessage(kPhysMsg_OnGrab, isLeft, heldRef,
							heldRef ? heldRef->GetFormID() : 0, 0);
					}
				}
			}
		};

		processHand(_rightHand, false);
		processHand(_leftHand, true);
	}

	// =========================================================================
	// Contact Resolution (main thread)
	//
	// WHY: The contact callback fires on the physics thread and can only safely
	// read body IDs. Name/form resolution requires bhkNPCollisionObject::Getbhk
	// and scene graph access, which are NOT thread-safe. So the physics thread
	// stores body IDs in atomics, and we resolve them here on the main thread.
	// =========================================================================

	void PhysicsInteraction::resolveContacts(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
	{
		// Check right hand contact
		auto rightContactBody = _lastContactBodyRight.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
		if (rightContactBody != 0xFFFFFFFF) {
			resolveAndLogContact("Right", bhk, hknp, RE::hknpBodyId{ rightContactBody });
		}

		// Check left hand contact
		auto leftContactBody = _lastContactBodyLeft.exchange(0xFFFFFFFF, std::memory_order_acq_rel);
		if (leftContactBody != 0xFFFFFFFF) {
			resolveAndLogContact("Left", bhk, hknp, RE::hknpBodyId{ leftContactBody });
		}
	}

	void PhysicsInteraction::resolveAndLogContact(const char* handName, RE::bhkWorld* bhk,
		RE::hknpWorld* hknp, RE::hknpBodyId bodyId)
	{
		if (!bhk || !hknp) return;

		// Get body info
		auto& body = hknp->GetBody(bodyId);
		auto layer = body.collisionFilterInfo & 0x7F;

		// Try to resolve to a game reference
		auto* ref = resolveBodyToRef(bhk, hknp, bodyId);
		if (ref) {
			auto* baseObj = ref->GetObjectReference();
			const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";
			auto objName = baseObj
				? RE::TESFullName::GetFullName(*baseObj, false)
				: std::string_view{};
			const char* nameStr = objName.empty() ? "(unnamed)" : objName.data();

			ROCK_LOG_INFO(Hand, "{} hand TOUCHED [{}] '{}' formID={:08X} body={} layer={}",
				handName, typeName, nameStr, ref->GetFormID(), bodyId.value, layer);

			// Update hand touch state for API
			bool isLeft = (std::string_view(handName) == "Left");
			auto& hand = isLeft ? _leftHand : _rightHand;
			hand.setTouchState(ref, ref->GetFormID(), layer);
			dispatchPhysicsMessage(kPhysMsg_OnTouch, isLeft, ref, ref->GetFormID(), layer);
		} else {
			// Unresolvable body — log layer info for debugging
			ROCK_LOG_DEBUG(Hand, "{} hand touched body={} layer={} (unresolved)",
				handName, bodyId.value, layer);
		}
	}

	// =========================================================================
	// Contact Events
	// =========================================================================

	void PhysicsInteraction::subscribeContactEvents(RE::hknpWorld* world)
	{
		// Get the global contact event signal (event type 3 = contact)
		void* signal = world->GetEventSignal(RE::hknpEventType::kContact);
		if (!signal) {
			ROCK_LOG_ERROR(Init, "Failed to get contact event signal");
			return;
		}

		// Use subscribe_ext (what the game uses internally)
		// Callback struct: {functionPtr, context} = 16 bytes
		struct CallbackInfo {
			void* fn;
			std::uint64_t ctx;
		};

		// We store our PhysicsInteraction pointer as the userData for the subscription
		static CallbackInfo cbInfo;
		cbInfo.fn = reinterpret_cast<void*>(&PhysicsInteraction::onContactCallback);
		cbInfo.ctx = 0;

		// subscribe_ext(signal, userData, callbackInfo)
		typedef void subscribe_ext_t(void* signal, void* userData, void* callbackInfo);
		static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(0x3B9E50) };
		subscribeExt(signal, static_cast<void*>(this), &cbInfo);

		ROCK_LOG_INFO(Init, "Subscribed to contact events");
	}

	void PhysicsInteraction::onContactCallback(void* userData, void** worldPtrHolder, void* contactEventData)
	{
		// SAFETY: This fires on the PHYSICS THREAD. The old hknpWorld can fire events
		// asynchronously during exterior cell transitions after our object is deleted.
		// Check s_hooksEnabled FIRST — it's cleared before any deletion in releaseSkeleton.
		__try {
			(void)userData;
			if (!s_hooksEnabled.load(std::memory_order_acquire)) return;
			auto* self = s_instance.load(std::memory_order_acquire);
			if (self && self->_initialized.load(std::memory_order_acquire)) {
				self->handleContactEvent(contactEventData);
			}
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			static int sehLogCounter = 0;
			if (sehLogCounter++ % 100 == 0) {
				logger::error("[ROCK::Contact] SEH exception caught on physics thread (count={}) — "
					"likely stale world during cell transition", sehLogCounter);
			}
			s_hooksEnabled.store(false, std::memory_order_release);
		}
	}

	void PhysicsInteraction::handleContactEvent(void* contactEventData)
	{
		if (!contactEventData) return;

		// hknpContactSignalData layout:
		// +0x08 = bodyIdA (uint32)
		// +0x0C = bodyIdB (uint32)
		auto* data = reinterpret_cast<std::uint8_t*>(contactEventData);
		std::uint32_t bodyIdA = *reinterpret_cast<std::uint32_t*>(data + 0x08);
		std::uint32_t bodyIdB = *reinterpret_cast<std::uint32_t*>(data + 0x0C);

		const auto rightId = _rightHand.getCollisionBodyId().value;
		const auto leftId = _leftHand.getCollisionBodyId().value;

		// --- Phase 2A: Track held body contacts for tau switching ---
		// Thread-safe: uses isHoldingAtomic() and isHeldBodyId() which read from
		// atomic snapshot array, not the main-thread vector. Safe on physics thread.
		if (_rightHand.isHoldingAtomic()) {
			if (_rightHand.isHeldBodyId(bodyIdA) || _rightHand.isHeldBodyId(bodyIdB)) {
				// Contact involves the right hand's held body — check other isn't our hand collider
				std::uint32_t heldId = _rightHand.isHeldBodyId(bodyIdA) ? bodyIdA : bodyIdB;
				std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
				if (other != rightId && other != leftId) {
					_rightHand.notifyHeldBodyContact();
				}
			}
		}
		if (_leftHand.isHoldingAtomic()) {
			if (_leftHand.isHeldBodyId(bodyIdA) || _leftHand.isHeldBodyId(bodyIdB)) {
				std::uint32_t heldId = _leftHand.isHeldBodyId(bodyIdA) ? bodyIdA : bodyIdB;
				std::uint32_t other = (bodyIdA == heldId) ? bodyIdB : bodyIdA;
				if (other != rightId && other != leftId) {
					_leftHand.notifyHeldBodyContact();
				}
			}
		}

		// Check if either body is one of our hand collision bodies
		bool isRight = (bodyIdA == rightId || bodyIdB == rightId);
		bool isLeft = (bodyIdA == leftId || bodyIdB == leftId);

		if (!isRight && !isLeft) {
			return;  // Not our contact (hand body not involved)
		}

		// Check if the off-hand is touching the weapon collision body
		{
			std::uint32_t weaponBid = _weaponCollision.getWeaponBodyIdAtomic();
			if (weaponBid != 0x7FFF'FFFF) {
				// Determine which hand is the off-hand (non-dominant)
				bool offhandIsLeft = !f4vr::isLeftHandedMode();
				std::uint32_t offhandId = offhandIsLeft ? leftId : rightId;
				bool offhandInvolved = (bodyIdA == offhandId || bodyIdB == offhandId);
				bool weaponInvolved = (bodyIdA == weaponBid || bodyIdB == weaponBid);
				if (offhandInvolved && weaponInvolved) {
					_offhandTouchingWeapon.store(true, std::memory_order_release);
				}
			}
		}

		// Store contact body ID for main-thread resolution (can't call resolveBodyToRef
		// on the physics thread — bhkNPCollisionObject::Getbhk is not thread-safe).
		std::uint32_t otherBody = isRight
			? (bodyIdA == rightId ? bodyIdB : bodyIdA)
			: (bodyIdA == leftId ? bodyIdB : bodyIdA);

		if (isRight) {
			_lastContactBodyRight.store(otherBody, std::memory_order_release);
		} else {
			_lastContactBodyLeft.store(otherBody, std::memory_order_release);
		}

		// I6 FIX: Throttled logging with proper atomic pattern.
		// Previous code had TOCTOU: two concurrent callbacks could both see >= 30
		// and both log + reset. Use fetch_add + modulo to avoid the race.
		int logCount = _contactLogCounter.fetch_add(1, std::memory_order_relaxed);
		if (logCount % 30 == 0) {
			ROCK_LOG_DEBUG(Hand, "Contact: {} hand (body {}) hit body {}",
				isRight ? "Right" : "Left",
				isRight ? rightId : leftId, otherBody);
		}
	}

	// =========================================================================
	// Object Ownership
	// =========================================================================

	bool PhysicsInteraction::physicsModOwnsObject(RE::TESObjectREFR* ref) const
	{
		if (!ref) return false;
		return _ownedObjects.contains(ref->GetFormID());
	}

	void PhysicsInteraction::claimObject(RE::TESObjectREFR* ref)
	{
		if (!ref) return;
		auto formID = ref->GetFormID();
		_ownedObjects.insert(formID);
		ROCK_LOG_DEBUG(Hand, "Claimed object: formID={:08X}", formID);
	}

	void PhysicsInteraction::releaseObject(RE::TESObjectREFR* ref)
	{
		if (!ref) return;
		auto formID = ref->GetFormID();
		if (_ownedObjects.erase(formID)) {
			ROCK_LOG_DEBUG(Hand, "Released object: formID={:08X}", formID);
		}
	}

	void PhysicsInteraction::releaseAllObjects()
	{
		if (!_ownedObjects.empty()) {
			ROCK_LOG_DEBUG(Hand, "Releasing all {} owned objects", _ownedObjects.size());
			_ownedObjects.clear();
		}

		if (frik::api::FRIKApi::inst) {
			frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", false);
		}
	}

	void PhysicsInteraction::forceDropHeldObject(bool isLeft)
	{
		auto& hand = isLeft ? _leftHand : _rightHand;
		if (!hand.isHolding()) return;

		auto* bhk = getPlayerBhkWorld();
		if (!bhk) {
			ROCK_LOG_WARN(Hand, "forceDropHeldObject: no bhkWorld available");
			return;
		}
		auto* hknp = getHknpWorld(bhk);
		if (!hknp) {
			ROCK_LOG_WARN(Hand, "forceDropHeldObject: no hknpWorld available");
			return;
		}

		auto* heldRef = hand.getHeldRef();
		ROCK_LOG_INFO(Hand, "forceDropHeldObject: {} hand dropping {}",
			isLeft ? "Left" : "Right",
			heldRef ? heldRef->GetFormID() : 0);

		hand.releaseGrabbedObject(hknp);
	}

	// =========================================================================
	// bhkWorld Access
	// =========================================================================

	RE::bhkWorld* PhysicsInteraction::getPlayerBhkWorld() const
	{
		auto* player = RE::PlayerCharacter::GetSingleton();
		if (!player) return nullptr;

		auto* cell = player->GetParentCell();
		if (!cell) return nullptr;

		return cell->GetbhkWorld();
	}

	RE::hknpWorld* PhysicsInteraction::getHknpWorld(RE::bhkWorld* bhk)
	{
		if (!bhk) return nullptr;
		// bhkWorld+0x60 holds the hknpWorld* pointer (Ghidra-verified).
		constexpr std::uintptr_t kBhkWorld_HknpWorldPtr = 0x60;
		return *reinterpret_cast<RE::hknpWorld**>(
			reinterpret_cast<uintptr_t>(bhk) + kBhkWorld_HknpWorldPtr);
	}
}


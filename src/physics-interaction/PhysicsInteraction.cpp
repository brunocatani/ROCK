#include "PhysicsInteraction.h"

#include <algorithm>
#include <cmath>
#include <numbers>

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
		namespace
	{
		constexpr float kRawParityWarnPosition = 0.10f;
		constexpr float kRawParityWarnRotationDegrees = 0.5f;
		constexpr float kRawParityFailPosition = 0.50f;
		constexpr float kRawParityFailRotationDegrees = 2.0f;
		constexpr int kRawParityWarnFrames = 2;
		constexpr int kRawParityFailFrames = 10;
		constexpr int kRawParitySummaryFrames = 300;
		constexpr int kRawParityLagFrames = 5;
		constexpr float kRawParityLagSlack = 0.05f;

		struct TransformDelta
		{
			float position = 0.0f;
			float rotationDegrees = 0.0f;
		};

		TransformDelta measureTransformDelta(const RE::NiTransform& a, const RE::NiTransform& b)
		{
			const float dx = a.translate.x - b.translate.x;
			const float dy = a.translate.y - b.translate.y;
			const float dz = a.translate.z - b.translate.z;

			const auto qa = niRotToHkQuat(a.rotate);
			const auto qb = niRotToHkQuat(b.rotate);
			const float dot = std::clamp(std::fabs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w), 0.0f, 1.0f);
			const float angleRadians = 2.0f * std::acos(dot);

			return TransformDelta{
				.position = std::sqrt(dx * dx + dy * dy + dz * dz),
				.rotationDegrees = angleRadians * (180.0f / std::numbers::pi_v<float>)
			};
		}

		float measurePointDelta(const RE::NiPoint3& a, const RE::NiPoint3& b)
		{
			const float dx = a.x - b.x;
			const float dy = a.y - b.y;
			const float dz = a.z - b.z;
			return std::sqrt(dx * dx + dy * dy + dz * dz);
		}

		float measureDirectionDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
		{
			const float dot = std::clamp(a.x * b.x + a.y * b.y + a.z * b.z, -1.0f, 1.0f);
			return std::acos(dot) * (180.0f / std::numbers::pi_v<float>);
		}
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
		REL::Relocation hookSite{ REL::Offset(offsets::kHookSite_MainLoop) };
		auto* hookByte = reinterpret_cast<const std::uint8_t*>(hookSite.address());
		if (*hookByte != 0xE8 && *hookByte != 0xE9) {  // CALL or JMP (may be hooked already)
			ROCK_LOG_ERROR(Init, "Hook site 0x{:X} is not a CALL/JMP instruction (found {:#x})",
				offsets::kHookSite_MainLoop, *hookByte);
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

	bool PhysicsInteraction::refreshHandBoneCache()
	{
		const bool cacheNeedsResolve = !_handBoneCache.isReady() || _handBoneCache.hasSkeletonChanged();
		if (!cacheNeedsResolve) {
			return true;
		}

		if (_handBoneCache.hasSkeletonChanged()) {
			_handBoneCache.reset();
		}

		if (_handBoneCache.resolve()) {
			_handCacheResolveLogCounter = 0;
			return true;
		}

		if (g_rockConfig.rockDebugHandTransformParity) {
			if (++_handCacheResolveLogCounter == 1 || _handCacheResolveLogCounter % 90 == 0) {
				ROCK_LOG_WARN(Hand, "HandBoneCache unresolved; raw parity sampling skipped this frame");
			}
		}

		return false;
	}

	RE::NiTransform PhysicsInteraction::getInteractionHandTransform(bool isLeft) const
	{
		if (_handBoneCache.isReady()) {
			return _handBoneCache.getWorldTransform(isLeft);
		}

		if (frik::api::FRIKApi::inst) {
			return frik::api::FRIKApi::inst->getHandWorldTransform(handFromBool(isLeft));
		}

		return RE::NiTransform();
	}

	RE::NiNode* PhysicsInteraction::getInteractionHandNode(bool isLeft) const
	{
		if (_handBoneCache.isReady()) {
			return _handBoneCache.getNode(isLeft);
		}

		if (frik::api::FRIKApi::inst) {
			return frik::api::FRIKApi::inst->getHandNode(handFromBool(isLeft));
		}

		return nullptr;
	}

	void PhysicsInteraction::sampleHandTransformParity()
	{
		if (!g_rockConfig.rockDebugHandTransformParity) {
			_parityEnabledLogged = false;
			_paritySummaryCounter = 0;
			return;
		}

		if (!frik::api::FRIKApi::inst || !_handBoneCache.isReady()) {
			return;
		}

		if (!_parityEnabledLogged) {
			ROCK_LOG_INFO(Init, "Hand-transform parity enabled (raw + derived basis, local cache vs FRIK API, pre-write sampling)");
			_parityEnabledLogged = true;
		}

		const bool playerMoving = frik::api::FRIKApi::inst->isPlayerMoving();
		const bool emitSummary = (++_paritySummaryCounter >= kRawParitySummaryFrames);

		auto sampleHand = [&](bool isLeft) {
			auto& state = _rawHandParityStates[isLeft ? 1 : 0];
			const auto handEnum = handFromBool(isLeft);
			const auto localTransform = _handBoneCache.getWorldTransform(isLeft);
			const auto apiTransform = frik::api::FRIKApi::inst->getHandWorldTransform(handEnum);
			const auto delta = measureTransformDelta(localTransform, apiTransform);
			const auto localCollisionTransform = computeHandCollisionTransformFromHandBasis(localTransform, isLeft);
			const auto apiCollisionTransform = computeHandCollisionTransformFromHandBasis(apiTransform, isLeft);
			const auto localPalmPosition = computePalmPositionFromHandBasis(localTransform, isLeft);
			const auto apiPalmPosition = computePalmPositionFromHandBasis(apiTransform, isLeft);
			const auto localPalmNormal = computePalmNormalFromHandBasis(localTransform, isLeft);
			const auto apiPalmNormal = computePalmNormalFromHandBasis(apiTransform, isLeft);
			const auto localPointing = computePointingVectorFromHandBasis(localTransform, isLeft);
			const auto apiPointing = computePointingVectorFromHandBasis(apiTransform, isLeft);
			state.lastPositionDelta = delta.position;
			state.lastRotationDeltaDegrees = delta.rotationDegrees;

			const bool warnExceeded = delta.position > kRawParityWarnPosition ||
				delta.rotationDegrees > kRawParityWarnRotationDegrees;
			const bool failExceeded = delta.position > kRawParityFailPosition ||
				delta.rotationDegrees > kRawParityFailRotationDegrees;

			state.warnFrames = warnExceeded ? state.warnFrames + 1 : 0;
			state.failFrames = failExceeded ? state.failFrames + 1 : 0;

			const char* handLabel = isLeft ? "Left" : "Right";
			if (state.warnFrames == kRawParityWarnFrames) {
				ROCK_LOG_WARN(Hand, "{} raw hand parity warning: posDelta={:.3f} rotDelta={:.3f}deg",
					handLabel, delta.position, delta.rotationDegrees);
			}

			if (state.failFrames == kRawParityFailFrames) {
				ROCK_LOG_ERROR(Hand, "{} raw hand parity failure: posDelta={:.3f} rotDelta={:.3f}deg",
					handLabel, delta.position, delta.rotationDegrees);
			}

			if (playerMoving && state.hasPreviousApiTransform) {
				const auto prevApiDelta = measureTransformDelta(localTransform, state.previousApiTransform);
				if (prevApiDelta.position + kRawParityLagSlack < delta.position) {
					state.lagFrames++;
					if (state.lagFrames == kRawParityLagFrames) {
						ROCK_LOG_WARN(Hand, "{} hand parity suggests possible one-frame lag: currentDelta={:.3f} prevApiDelta={:.3f}",
							handLabel, delta.position, prevApiDelta.position);
					}
				} else {
					state.lagFrames = 0;
				}
			} else {
				state.lagFrames = 0;
			}

			state.previousApiTransform = apiTransform;
			state.hasPreviousApiTransform = true;

			if (emitSummary) {
				const char* summaryHandLabel = isLeft ? "L" : "R";
				ROCK_LOG_INFO(Hand,
					"{} parity: raw(pos={:.3f}, rot={:.3f}deg) basis(collider={:.3f}, palmPos={:.3f}, palmNormal={:.3f}deg, pointing={:.3f}deg)",
					summaryHandLabel,
					delta.position,
					delta.rotationDegrees,
					measurePointDelta(localCollisionTransform.translate, apiCollisionTransform.translate),
					measurePointDelta(localPalmPosition, apiPalmPosition),
					measureDirectionDeltaDegrees(localPalmNormal, apiPalmNormal),
					measureDirectionDeltaDegrees(localPointing, apiPointing));
			}
		};

		sampleHand(false);
		sampleHand(true);

		if (emitSummary) {
			_paritySummaryCounter = 0;
			const auto& right = _rawHandParityStates[0];
			const auto& left = _rawHandParityStates[1];
			ROCK_LOG_INFO(Hand,
				"Raw hand parity summary: R(pos={:.3f}, rot={:.3f}deg) L(pos={:.3f}, rot={:.3f}deg)",
				right.lastPositionDelta, right.lastRotationDeltaDegrees,
				left.lastPositionDelta, left.lastRotationDeltaDegrees);
		}
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
		if (!refreshHandBoneCache()) {
			ROCK_LOG_WARN(Init, "HandBoneCache not ready during init; runtime remains on pre-00 transform paths");
		}

		// Register our custom collision layer (once per world)
		registerCollisionLayer(hknp);

		// Create hand collision bodies
		createHandCollisions(hknp, bhk);
		_cachedHalfExtentX = g_rockConfig.rockHandCollisionHalfExtentX;
		_cachedHalfExtentY = g_rockConfig.rockHandCollisionHalfExtentY;
		_cachedHalfExtentZ = g_rockConfig.rockHandCollisionHalfExtentZ;

		// Subscribe to contact events for touch detection
		subscribeContactEvents(hknp);

		// Initialize weapon collision system
		_weaponCollision.init(hknp, bhk);

		// Kill FRIK's offhand grip permanently — ROCK owns weapon interaction now
		if (frik::api::FRIKApi::inst) {
			frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_Physics", true);
			ROCK_LOG_INFO(Init, "FRIK offhand grip permanently suppressed");
		}

		// After body creation, call SetBodyTransform to establish correct broadphase position.
		// CreateBody places the body at origin (0,0,0) in the broadphase. SetBodyTransform
		// updates the motion position AND the broadphase tree via synchronizeBodiesFromMotion.
		{
			auto rightTransform = computeHandCollisionTransformFromHandBasis(getInteractionHandTransform(false), false);
			auto leftTransform = computeHandCollisionTransformFromHandBasis(getInteractionHandTransform(true), true);
			_rightHand.updateCollisionTransform(hknp, rightTransform, 0.011f);
			_leftHand.updateCollisionTransform(hknp, leftTransform, 0.011f);
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
		if (!frik::api::FRIKApi::inst) {
			return;
		}

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
		if (!frik::api::FRIKApi::inst->isSkeletonReady()) {
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
						if (_rightHand.isHolding()) { auto* r = _rightHand.getHeldRef(); _rightHand.releaseGrabbedObject(hknpMenu); if (r) releaseObject(r); }
						if (_leftHand.isHolding()) { auto* r = _leftHand.getHeldRef(); _leftHand.releaseGrabbedObject(hknpMenu); if (r) releaseObject(r); }
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

		refreshHandBoneCache();
		sampleHandTransformParity();

		// --- Per-frame collision layer verification (HIGGS EnsureHiggsCollisionLayer pattern) ---
		// The game may reset the collision filter matrix on cell load or world recreation.
		// Check our layer's mask every frame and re-register if it changed.
		// Reference: HIGGS S09 section 9.2, FRIK_STUDY/ADD_TO_PLAN.md section S01-1.9.2
		if (_collisionLayerRegistered && _expectedHandLayerMask != 0) {
			auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(
				reinterpret_cast<std::uintptr_t>(hknp) + offsets::kHknpWorld_ModifierManager);
			if (modifierMgr) {
				auto* filterPtr = *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
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
			if (_rightHand.isHolding()) { auto* r = _rightHand.getHeldRef(); _rightHand.releaseGrabbedObject(hknp); if (r) releaseObject(r); }
			if (_leftHand.isHolding()) { auto* r = _leftHand.getHeldRef(); _leftHand.releaseGrabbedObject(hknp); if (r) releaseObject(r); }
			destroyHandCollisions(bhk);  // also destroys debug spheres
			createHandCollisions(hknp, bhk);
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
						ROCK_LOG_INFO(Weapon, "WeaponNode: '{}' pos=({:.1f},{:.1f},{:.1f}) hasBody={} bodyCount={}",
							weaponNode->name.c_str(),
							weaponNode->world.translate.x, weaponNode->world.translate.y, weaponNode->world.translate.z,
							_weaponCollision.hasWeaponBody(), _weaponCollision.getWeaponBodyCount());
					} else {
						// ROCK_LOG_INFO(Weapon, "WeaponNode: NULL hasBody={}", _weaponCollision.hasWeaponBody());
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
			if (_rightHand.isHolding()) { auto* r = _rightHand.getHeldRef(); _rightHand.releaseGrabbedObject(hknp); if (r) releaseObject(r); }
			if (_leftHand.isHolding()) { auto* r = _leftHand.getHeldRef(); _leftHand.releaseGrabbedObject(hknp); if (r) releaseObject(r); }
			_weaponCollision.destroyWeaponBody(hknp);
			destroyHandCollisions(_cachedBhkWorld);
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
		_handBoneCache.reset();
		_handCacheResolveLogCounter = 0;
		_paritySummaryCounter = 0;
		_parityEnabledLogged = false;
		_runtimeScaleLogged = false;
		_rawHandParityStates = {};

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
		// Phase 0A fix: Read collision filter from the hknpWorld's modifier manager,
		// NOT from the global singleton. After world recreation (cell load), the new
		// hknpWorld may use a different filter object than the persistent global singleton
		// at REL::Offset(offsets::kData_CollisionFilterSingleton). The global singleton is created once in
		// bhkWorld__InitSystem and never recreated, but the new world's modifier manager
		// may reference a fresh filter where our custom layers are unconfigured.
		//
		// Path: *(*(world + 0x150) + 0x5E8) → bhkCollisionFilter*
		// world+0x150 = hknpModifierManager* (Ghidra-verified: ctor 0x141540d90, param_1[0x2A])
		// modifierMgr+0x5E8 = filter ptr (Ghidra-verified: init fn 0x141725450 zeros this slot)
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

		// Read filter from the world's modifier manager (not global singleton)
		auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(
			reinterpret_cast<std::uintptr_t>(world) + offsets::kHknpWorld_ModifierManager);
		if (!modifierMgr) {
			ROCK_LOG_ERROR(Config, "World modifier manager (+0x150) is null — cannot configure layer");
			return;
		}

		auto* filterPtr = *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
		if (!filterPtr) {
			// Fallback: try the global singleton (may be the same object on first init)
			static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
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
			static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
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

	void PhysicsInteraction::createHandCollisions(RE::hknpWorld* world, void* bhkWorld)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
			ROCK_LOG_ERROR(Hand, "Cannot create hand collisions — skeleton not ready");
			return;
		}

		_rightHand.createCollision(world, bhkWorld,
			g_rockConfig.rockHandCollisionHalfExtentX,
			g_rockConfig.rockHandCollisionHalfExtentY,
			g_rockConfig.rockHandCollisionHalfExtentZ);

		_leftHand.createCollision(world, bhkWorld,
			g_rockConfig.rockHandCollisionHalfExtentX,
			g_rockConfig.rockHandCollisionHalfExtentY,
			g_rockConfig.rockHandCollisionHalfExtentZ);
	}

	void PhysicsInteraction::destroyHandCollisions(void* bhkWorld)
	{
		_rightHand.destroyDebugColliderVis();
		_leftHand.destroyDebugColliderVis();
		_rightHand.destroyDebugBasisVis();
		_leftHand.destroyDebugBasisVis();
		_rightHand.destroyCollision(bhkWorld);
		_leftHand.destroyCollision(bhkWorld);
	}

	void PhysicsInteraction::updateHandCollisions(RE::hknpWorld* world)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) {
			return;
		}

		bool rightDisabled = s_rightHandDisabled.load(std::memory_order_acquire);
		bool leftDisabled = s_leftHandDisabled.load(std::memory_order_acquire);

		const RE::NiTransform rightHandTransform = getInteractionHandTransform(false);
		const RE::NiTransform leftHandTransform = getInteractionHandTransform(true);
		RE::NiTransform rightTransform = computeHandCollisionTransformFromHandBasis(rightHandTransform, false);
		RE::NiTransform leftTransform = computeHandCollisionTransformFromHandBasis(leftHandTransform, true);
		const RE::NiPoint3 rightPalmCenter = computePalmPositionFromHandBasis(rightHandTransform, false);
		const RE::NiPoint3 leftPalmCenter = computePalmPositionFromHandBasis(leftHandTransform, true);

		if (!rightDisabled) {
			_rightHand.updateCollisionTransform(world, rightTransform, _deltaTime);
			auto* rightDebugParent = getInteractionHandNode(false);
			_rightHand.updateDebugColliderVis(rightTransform, g_rockConfig.rockDebugShowColliders,
				rightDebugParent,
				g_rockConfig.rockHandCollisionHalfExtentX,
				g_rockConfig.rockHandCollisionHalfExtentY,
				g_rockConfig.rockHandCollisionHalfExtentZ,
				g_rockConfig.rockDebugColliderShape);
			_rightHand.updateDebugBasisVis(rightTransform, rightPalmCenter, g_rockConfig.rockDebugShowPalmBasis, rightDebugParent);
		} else {
			_rightHand.destroyDebugBasisVis();
		}
		if (!leftDisabled) {
			_leftHand.updateCollisionTransform(world, leftTransform, _deltaTime);
			auto* leftDebugParent = getInteractionHandNode(true);
			_leftHand.updateDebugColliderVis(leftTransform, g_rockConfig.rockDebugShowColliders,
				leftDebugParent,
				g_rockConfig.rockHandCollisionHalfExtentX,
				g_rockConfig.rockHandCollisionHalfExtentY,
				g_rockConfig.rockHandCollisionHalfExtentZ,
				g_rockConfig.rockDebugColliderShape);
			_leftHand.updateDebugBasisVis(leftTransform, leftPalmCenter, g_rockConfig.rockDebugShowPalmBasis, leftDebugParent);
		} else {
			_leftHand.destroyDebugBasisVis();
		}

	}

	// =========================================================================
	// Object Detection (Phase 2)
	// =========================================================================

	void PhysicsInteraction::updateSelection(RE::bhkWorld* bhk, RE::hknpWorld* hknp)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady()) return;

		const RE::NiTransform rightTransform = getInteractionHandTransform(false);
		const RE::NiTransform leftTransform = getInteractionHandTransform(true);
		auto rightPalmPos = computePalmPositionFromHandBasis(rightTransform, false);
		auto rightPalmNormal = computePalmNormalFromHandBasis(rightTransform, false);
		auto rightPointing = computePointingVectorFromHandBasis(rightTransform, false);
		auto leftPalmPos = computePalmPositionFromHandBasis(leftTransform, true);
		auto leftPalmNormal = computePalmNormalFromHandBasis(leftTransform, true);
		auto leftPointing = computePointingVectorFromHandBasis(leftTransform, true);

		// Other hand's selection (for mutual exclusion)
		auto* rightHeldRef = _rightHand.hasSelection() ? _rightHand.getSelection().refr : nullptr;
		auto* leftHeldRef = _leftHand.hasSelection() ? _leftHand.getSelection().refr : nullptr;

		// Update each hand's selection
		if (!s_rightHandDisabled.load(std::memory_order_acquire)) {
			_rightHand.updateSelection(bhk, hknp,
				rightPalmPos, rightPalmNormal, rightPointing,
				g_rockConfig.rockNearDetectionRange, g_rockConfig.rockFarDetectionRange,
				leftHeldRef);
		}

		if (!s_leftHandDisabled.load(std::memory_order_acquire)) {
			_leftHand.updateSelection(bhk, hknp,
				leftPalmPos, leftPalmNormal, leftPointing,
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
					// Use the visible-hand-first raw hand transform (no collision offset).
					// Collision and palm offsets are applied separately inside the basis helpers.
					auto transform = getInteractionHandTransform(isLeft);
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
						auto* handNode = getInteractionHandNode(isLeft);

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

					// Use the visible-hand-first raw hand transform (no collision offset) for the
					// grab snapshot. Constraint pivots apply palm-space offsets separately.
					auto transform = getInteractionHandTransform(isLeft);

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
		static REL::Relocation<subscribe_ext_t> subscribeExt{ REL::Offset(offsets::kFunc_SubscribeContactEvent) };
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
			if (_weaponCollision.getWeaponBodyIdAtomic() != 0x7FFF'FFFF) {
				// Determine which hand is the off-hand (non-dominant)
				bool offhandIsLeft = !f4vr::isLeftHandedMode();
				std::uint32_t offhandId = offhandIsLeft ? leftId : rightId;
				bool offhandInvolved = (bodyIdA == offhandId || bodyIdB == offhandId);
				bool weaponInvolved = _weaponCollision.isWeaponBodyIdAtomic(bodyIdA) ||
					_weaponCollision.isWeaponBodyIdAtomic(bodyIdB);
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
		if (heldRef) releaseObject(heldRef);
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


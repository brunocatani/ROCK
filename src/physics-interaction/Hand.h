#pragma once

// Hand.h — Per-hand state for the ROCK physics interaction system.
//
// WHY: Each VR hand needs its own collision body, state machine, and grab state.
// Phase 1: Collision body creation/destruction, per-frame position tracking.
// Phase 2: Object detection/selection with hysteresis, touch state, debug sphere.
// Phase 3: Grab lifecycle — constraint creation, per-frame update, release.
//
// This mirrors HIGGS's Hand class (12_hand_class.md) but simplified for FO4VR:
// - No 4-phase frame hook (we run after FRIK's single update)
// - Body IDs instead of rigid body pointers (hknp vs hkp)
// - Finger override through FRIK's HandPoseManager tags

#include "RockConfig.h"
#include "GrabConstraint.h"
#include "ObjectDetection.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include "RE/Bethesda/BSTempEffect.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/Havok/hkReferencedObject.h"

namespace frik::rock
{
	/// Collision layer for ROCK hand collision bodies.
	/// Layer 43 — hand bodies. Collides with BIPED(8) for NPC touch/punch.
	/// Held objects keep their ORIGINAL layer (HIGGS pattern).
	/// CC push prevention is via processConstraintsCallback hook (surface velocity zeroing).
	constexpr std::uint32_t ROCK_HAND_LAYER = 43;

	/// Invalid body ID sentinel value (matches hknpBodyId template default).
	constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

	/// Create a box-shaped collision body using hknpConvexPolytopeShape.
	/// WHY: hknpBoxShape doesn't exist in FO4VR's hknp physics (Havok 2014).
	/// HIGGS uses hkpBoxShape for flat palm collision — better contact surface than
	/// capsule's rounded ends. We create a capsule (same 8-vert 6-face topology),
	/// then overwrite vtable to convex base and fill box vertex/normal/face data.
	/// Ghidra RE (2026-03-23): capsule vtable=0x142c9a208, convex=0x142c9a108,
	/// polytope factory at FUN_1416ddbc0 confirms convex vtable for polytope shapes.
	inline RE::hknpShape* CreateBoxShape(float hx, float hy, float hz, float convexRadius = 0.0f)
	{
		// Allocate via CreateCapsuleShape — identical memory layout (0x1B0 bytes).
		// Both are 8-vertex, 6-quad-face convex shapes with the same allocation.
		RE::hkVector4f start(-hx, 0.0f, 0.0f, 0.0f);
		RE::hkVector4f end(hx, 0.0f, 0.0f, 0.0f);
		float allocRadius = (hy > hz) ? hy : hz;
		auto* capsule = RE::hknpCapsuleShape::CreateCapsuleShape(start, end, allocRadius);
		if (!capsule) return nullptr;

		auto* s = reinterpret_cast<char*>(capsule);

		// --- Overwrite vtable: capsule → convex polytope ---
		// Capsule vtable (0x142c9a208) has capsule-specific AABB/raycast functions.
		// Convex base vtable (0x142c9a108) iterates vertices generically — correct for box.
		static REL::Relocation<std::uintptr_t> convexVtable{ REL::Offset(0x2C9A108) };
		*reinterpret_cast<std::uintptr_t*>(s) = convexVtable.address();

		// --- Overwrite type code: 0x01C3 (capsule-specific) → 0x0103 (convex polytope) ---
		*reinterpret_cast<std::uint16_t*>(s + 0x10) = 0x0103;

		// --- Set convex radius (0 = sharp edges, >0 = rounded corners) ---
		*reinterpret_cast<float*>(s + 0x14) = convexRadius;

		// --- Fill 8 box vertices at +0x70 (128 bytes) ---
		// Layout: V0-V3 at z=-hz, V4-V7 at z=+hz
		// Each vertex: {x, y, z, W} where W encodes vertex index in 0.5f mantissa
		auto* v = reinterpret_cast<float*>(s + 0x70);
		auto setVert = [&](int i, float x, float y, float z) {
			v[i*4+0] = x;
			v[i*4+1] = y;
			v[i*4+2] = z;
			std::uint32_t w = 0x3F000000u | static_cast<std::uint32_t>(i);
			v[i*4+3] = *reinterpret_cast<float*>(&w);
		};
		setVert(0, -hx, -hy, -hz);
		setVert(1, +hx, -hy, -hz);
		setVert(2, -hx, +hy, -hz);
		setVert(3, +hx, +hy, -hz);
		setVert(4, -hx, -hy, +hz);
		setVert(5, +hx, -hy, +hz);
		setVert(6, -hx, +hy, +hz);
		setVert(7, +hx, +hy, +hz);

		// --- Fill 6 face normals at +0xF0 (128 bytes, 8 slots, first 6 used) ---
		// Each normal: {nx, ny, nz, d} where d = distance from origin to face plane
		auto* n = reinterpret_cast<float*>(s + 0xF0);
		auto setNormal = [&](int i, float nx, float ny, float nz, float d) {
			n[i*4+0] = nx;
			n[i*4+1] = ny;
			n[i*4+2] = nz;
			n[i*4+3] = d;
		};
		setNormal(0, +1.0f,  0.0f,  0.0f, hx);  // +X face
		setNormal(1, -1.0f,  0.0f,  0.0f, hx);  // -X face
		setNormal(2,  0.0f, +1.0f,  0.0f, hy);  // +Y face
		setNormal(3,  0.0f, -1.0f,  0.0f, hy);  // -Y face
		setNormal(4,  0.0f,  0.0f, +1.0f, hz);  // +Z face
		setNormal(5,  0.0f,  0.0f, -1.0f, hz);  // -Z face
		// Padding normals 6-7 (copy face 0-1 for alignment)
		setNormal(6, +1.0f, 0.0f, 0.0f, hx);
		setNormal(7, -1.0f, 0.0f, 0.0f, hx);

		// --- Fill face index table at +0x170 (32 bytes) ---
		// Each entry: {int16 edgeOffset, uint8 numEdges=4, uint8 dataSize=4}
		auto* faceTable = reinterpret_cast<std::uint8_t*>(s + 0x170);
		for (int i = 0; i < 6; i++) {
			auto* entry = faceTable + i * 4;
			std::uint16_t offset = static_cast<std::uint16_t>(i * 4);
			*reinterpret_cast<std::uint16_t*>(entry) = offset;
			entry[2] = 4;  // numEdges
			entry[3] = 4;  // data size
		}

		// --- Fill edge data at +0x190 (24 bytes) ---
		// 6 faces × 4 vertex indices, CCW winding from outside
		auto* edges = reinterpret_cast<std::uint32_t*>(s + 0x190);
		edges[0] = 0x05070301;  // Face 0 (+X): V1,V3,V7,V5
		edges[1] = 0x00020604;  // Face 1 (-X): V4,V6,V2,V0
		edges[2] = 0x03070602;  // Face 2 (+Y): V2,V6,V7,V3
		edges[3] = 0x04050100;  // Face 3 (-Y): V0,V1,V5,V4
		edges[4] = 0x06070504;  // Face 4 (+Z): V4,V5,V7,V6
		edges[5] = 0x03020001;  // Face 5 (-Z): V1,V0,V2,V3

		// --- Update header fields for box (may differ from capsule Init values) ---
		// +0x44: numFaces (capsule Init may have set differently)
		*reinterpret_cast<std::uint16_t*>(s + 0x44) = 6;

		ROCK_LOG_INFO(Hand, "Created box shape: hx={:.4f} hy={:.4f} hz={:.4f} radius={:.4f}",
			hx, hy, hz, convexRadius);

		return reinterpret_cast<RE::hknpShape*>(capsule);
	}

	/// Register a custom "ROCK_Hand" material with high friction for natural grip.
	/// WHY: Default material (ID 0) has standard friction that lets objects slide off
	/// the hand. HIGGS sets a skin material on its hand body for proper grip feel.
	/// In hknp, materials are on the BODY (body+0x70), not the shape.
	/// Registered once via hknpMaterialLibrary::addMaterial, cached for reuse.
	inline RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world)
	{
		static RE::hknpMaterialId cachedId{ 0xFFFF };
		if (cachedId.value != 0xFFFF) return cachedId;

		if (!world) return { 0 };

		// Material library lives at world+0x5C8
		auto* matLibPtr = reinterpret_cast<void**>(
			reinterpret_cast<char*>(world) + 0x5C8);
		auto* matLib = *matLibPtr;
		if (!matLib) {
			ROCK_LOG_WARN(Hand, "Material library is null — using default material 0");
			return { 0 };
		}

		// Construct a default material (0x50 bytes) then customize friction
		alignas(16) char matBuffer[0x50];
		memset(matBuffer, 0, 0x50);

		// Call hknpMaterial constructor to set sane defaults
		typedef void (*matCtor_t)(void*);
		static REL::Relocation<matCtor_t> matCtor{ REL::Offset(0x1536CB0) };
		matCtor(matBuffer);

		// Set high friction for grip (prevents objects sliding off palm)
		// +0x11 = dynamicFriction (quantized uint8, 0-255 via LUT)
		//   200/255 ≈ 0.78 — high grip, objects stick to hand on contact
		*reinterpret_cast<std::uint8_t*>(matBuffer + 0x11) = 200;

		// +0x12 = staticFriction (hkHalf16)
		//   0x3C00 = 1.0f in IEEE half-precision — maximum static grip
		*reinterpret_cast<std::uint16_t*>(matBuffer + 0x12) = 0x3C00;

		// +0x28 = restitution (hkHalf16)
		//   0x0000 = 0.0f — no bounce on contact
		*reinterpret_cast<std::uint16_t*>(matBuffer + 0x28) = 0x0000;

		// +0x18 = frictionCombinePolicy (uint8): 2 = MAX (use whichever is higher)
		*reinterpret_cast<std::uint8_t*>(matBuffer + 0x18) = 2;

		// +0x10 = triggerType (uint8): 0 = normal (not a trigger volume)
		*reinterpret_cast<std::uint8_t*>(matBuffer + 0x10) = 0;

		// Register via addMaterial (world+0x5C8 = material library)
		typedef void (*addMat_t)(void*, std::uint16_t*, void*);
		static REL::Relocation<addMat_t> addMaterial{ REL::Offset(0x1537840) };

		std::uint16_t newId = 0xFFFF;
		addMaterial(matLib, &newId, matBuffer);

		if (newId != 0xFFFF) {
			cachedId.value = newId;
			ROCK_LOG_INFO(Hand, "Registered ROCK_Hand material ID={} (dynFriction=200, staticFriction=1.0, restitution=0.0)",
				newId);
		} else {
			ROCK_LOG_WARN(Hand, "Failed to register ROCK_Hand material — using default 0");
			return { 0 };
		}

		return cachedId;
	}

	/// Hand state machine states (HIGGS S17 pattern).
	/// Scaffolded with all states from HIGGS — unimplemented states are stubs that
	/// transition back to Idle. Implementing structural scaffolding upfront prevents
	/// costly retrofitting when features are added later (scaffold-first principle).
	enum class HandState : std::uint8_t
	{
		Idle,              ///< No interaction — detection runs, collision body tracks controller
		SelectedClose,     ///< Object detected near hand (was "Selecting")
		SelectedFar,       ///< Object detected far (pointing ray) — STUB
		SelectionLocked,   ///< Trigger held on far object, waiting for pull/near — STUB
		HeldInit,          ///< Grab constraint just created, motor force fading in
		HeldBody,          ///< Physics constraint active, per-frame update (was "Held")
		Pulled,            ///< Object flying toward hand (gravity glove) — STUB
		PreGrabItem,       ///< Waiting for looted item to spawn — STUB
		GrabFromOtherHand, ///< Waiting for other hand to release — STUB
		SelectedTwoHand,   ///< Pointing at other hand's weapon — STUB
		HeldTwoHanded,     ///< Both hands gripping weapon — STUB
	};

	/// Per-hand physics interaction state.
	class Hand
	{
	public:
		explicit Hand(bool isLeft) :
			_isLeft(isLeft) {}

		bool isLeft() const { return _isLeft; }
		HandState getState() const { return _state; }
		const char* handName() const { return _isLeft ? "Left" : "Right"; }

		/// Reset all state to defaults (used on skeleton invalidation).
		/// Does NOT destroy Havok bodies — caller must call destroyCollision first.
		/// Does NOT destroy grab constraints — caller must call releaseGrabbedObject first.
		void reset()
		{
			stopSelectionHighlight();
			_isHoldingFlag.store(false, std::memory_order_release);
			_heldBodyIdsCount.store(0, std::memory_order_release);
			_heldBodyContactFrame.store(100, std::memory_order_release);
			_state = HandState::Idle;
			_prevState = HandState::Idle;
			_idleDesired = false;
			_grabRequested = false;
			_releaseRequested = false;
			_collisionBodyId.value = INVALID_BODY_ID;
			_shape = nullptr;
			_currentSelection.clear();
			_cachedFarCandidate.clear();
			_farDetectCounter = 0;
			_selectionHoldFrames = 0;
			_deselectCooldown = 0;
			_lastDeselectedRef = nullptr;
			_lastTouchedRef = nullptr;
			_lastTouchedFormID = 0;
			_lastTouchedLayer = 0;
			_touchActiveFrames = 100;
			_activeConstraint.clear();
			_savedObjectState.clear();
			_grabPointLocal[0] = _grabPointLocal[1] = _grabPointLocal[2] = _grabPointLocal[3] = 0.0f;
			_grabStartTime = 0.0f;
			_heldLogCounter = 0;
			_pivotBLogCounter = 0;
			_notifCounter = 0;
			_heldBodyIds.clear();
			_grabHandSpace = RE::NiTransform();
			_heldNode = nullptr;
		}

		// --- Held body ID tracking ---
		// At grab time, collect ALL body IDs of the grabbed object via bhkPhysicsSystem.
		// Used by processConstraintsCallback hook to zero surface velocity for held bodies.

		/// Collect all body IDs from ALL collision objects in the NiNode tree.
		/// Weapons have child NiNodes (scope, barrel, stock) each with their
		/// own bhkNPCollisionObject and separate physics systems.
		void collectHeldBodyIds(RE::TESObjectREFR* refr)
		{
			_heldBodyIds.clear();
			if (!refr) return;
			auto* node3D = refr->Get3D();
			if (!node3D) return;
			collectBodyIdsRecursive(node3D);
		}

	private:
		void collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth = 10)
		{
			if (!node || maxDepth <= 0) return;

			auto* collObj = node->collisionObject.get();
			if (collObj) {
				// Verify this is a bhkNPCollisionObject, not a proxy or other type.
				// bhkNPCollisionProxyObject has different field layout and crashes
				// when we access +0x20 as a physics system pointer.
				// Check: a valid bhkNPCollisionObject has a non-null physics system
				// pointer at +0x20 that points to valid memory above the reserved
				// null-page range (first 64KB on Windows). Addresses below this
				// threshold are OS-reserved and never valid heap/global pointers.
				constexpr std::uintptr_t kMinValidPointer = 0x10000;
				auto* fieldAt20 = *reinterpret_cast<void**>(
					reinterpret_cast<char*>(collObj) + 0x20);
				if (fieldAt20 && reinterpret_cast<std::uintptr_t>(fieldAt20) > kMinValidPointer) {
					auto* physSystem = reinterpret_cast<RE::bhkPhysicsSystem*>(fieldAt20);
					auto* inst = physSystem->instance;
					if (inst && reinterpret_cast<std::uintptr_t>(inst) > kMinValidPointer) {
						for (std::int32_t i = 0; i < inst->bodyCount && i < 64; i++) {
							std::uint32_t bid = inst->bodyIds[i];
							if (bid != 0x7FFF'FFFF) {
								_heldBodyIds.push_back(bid);
							}
						}
					}
				}
			}

			auto* niNode = node->IsNode();
			if (niNode) {
				auto& kids = niNode->GetRuntimeData().children;
				for (std::uint32_t i = 0; i < kids.size(); i++) {
					auto* kid = kids[i].get();
					if (kid) collectBodyIdsRecursive(kid, maxDepth - 1);
				}
			}
		}

	public:

		/// Get the list of held body IDs (for diagnostics — main thread only).
		const std::vector<std::uint32_t>& getHeldBodyIds() const { return _heldBodyIds; }

		/// Check if a body ID belongs to the currently held object.
		/// Thread-safe: reads from the atomic snapshot array, not the vector.
		/// Called from processConstraintsCallback and contact callback on physics thread.
		bool isHeldBodyId(std::uint32_t bodyId) const
		{
			int count = _heldBodyIdsCount.load(std::memory_order_acquire);
			for (int i = 0; i < count; i++) {
				if (_heldBodyIdsSnapshot[i] == bodyId) return true;
			}
			return false;
		}

		/// Thread-safe: is the hand currently holding something?
		/// Safe for physics thread — reads atomic flag instead of _state.
		bool isHoldingAtomic() const {
			return _isHoldingFlag.load(std::memory_order_acquire);
		}

		// --- Selection (Phase 2) ---

		const SelectedObject& getSelection() const { return _currentSelection; }
		bool hasSelection() const { return _currentSelection.isValid(); }

		// --- Touch state (API) ---
		bool isTouching() const { return _touchActiveFrames < 5; }
		RE::TESObjectREFR* getLastTouchedRef() const { return _lastTouchedRef; }
		std::uint32_t getLastTouchedFormID() const { return _lastTouchedFormID; }
		std::uint32_t getLastTouchedLayer() const { return _lastTouchedLayer; }

		void setTouchState(RE::TESObjectREFR* refr, std::uint32_t formID, std::uint32_t layer) {
			_lastTouchedRef = refr;
			_lastTouchedFormID = formID;
			_lastTouchedLayer = layer;
			_touchActiveFrames = 0;
		}

		// --- Grab state (Phase 3) ---

		bool isHolding() const { return _state == HandState::HeldInit || _state == HandState::HeldBody; }
		RE::TESObjectREFR* getHeldRef() const { return _savedObjectState.refr; }
		const ActiveConstraint& getActiveConstraint() const { return _activeConstraint; }
		const SavedObjectState& getSavedObjectState() const { return _savedObjectState; }

		/// Compute the adjusted hand transform (hand follows object, HIGGS pattern).
		/// Returns true if adjustment is valid, false if not holding or node invalid.
		/// adjustedHand = objectWorld * Invert(grabHandSpace)
		bool getAdjustedHandTransform(RE::NiTransform& outTransform) const
		{
			if (!isHolding()) return false;
			// I3 FIX: Re-derive node from refr each call instead of trusting cached _heldNode.
			// If the object was unloaded mid-grab, refr->Get3D() returns nullptr safely.
			RE::NiAVObject* node = nullptr;
			if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted() &&
				!_savedObjectState.refr->IsDisabled()) {
				node = _savedObjectState.refr->Get3D();
			}
			if (!node) return false;

			// Invert _grabHandSpace: invR = R^T, invPos = -(R^T * pos) / scale
			RE::NiMatrix3 invRot = _grabHandSpace.rotate.Transpose();
			float invScale = (_grabHandSpace.scale > 0.0001f) ? (1.0f / _grabHandSpace.scale) : 1.0f;
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

			// Multiply: objectWorld * inverseGrab
			// result.rot = obj.rot * inv.rot
			// result.pos = obj.pos + (obj.rot * inv.pos) * obj.scale
			const auto& obj = node->world;
			outTransform.rotate = obj.rotate * invRot;
			outTransform.scale = obj.scale * invScale;
			RE::NiPoint3 rotatedPos;
			rotatedPos.x = obj.rotate.entry[0][0] * invPos.x + obj.rotate.entry[0][1] * invPos.y + obj.rotate.entry[0][2] * invPos.z;
			rotatedPos.y = obj.rotate.entry[1][0] * invPos.x + obj.rotate.entry[1][1] * invPos.y + obj.rotate.entry[1][2] * invPos.z;
			rotatedPos.z = obj.rotate.entry[2][0] * invPos.x + obj.rotate.entry[2][1] * invPos.y + obj.rotate.entry[2][2] * invPos.z;
			outTransform.translate.x = obj.translate.x + rotatedPos.x * obj.scale;
			outTransform.translate.y = obj.translate.y + rotatedPos.y * obj.scale;
			outTransform.translate.z = obj.translate.z + rotatedPos.z * obj.scale;

			return true;
		}

		/// Initiate grab on the currently selected object.
		/// Saves object state, changes collision layer, creates constraint.
		/// Transitions: Selecting → HeldInit → Held (next frame).
		bool grabSelectedObject(RE::hknpWorld* world,
			const RE::NiTransform& handWorldTransform,
			float tau, float damping, float maxForce,
			float proportionalRecovery, float constantRecovery);

		/// Per-frame update while holding — updates constraint target, adapts tau/force.
		void updateHeldObject(RE::hknpWorld* world,
			const RE::NiTransform& handWorldTransform, float deltaTime,
			float forceFadeInTime, float tauMin, float tauMax,
			float tauIncrement, float tauDecrement,
			float closeThreshold, float farThreshold);

		/// Release the grabbed object — destroy constraint, restore state.
		void releaseGrabbedObject(RE::hknpWorld* world);

		void tickTouchState() { _touchActiveFrames++; }

		/// Run near + far detection, apply priority and hysteresis.
		/// Called every frame from PhysicsInteraction::update.
		///
		/// Anti-flicker design:
		///   - Minimum hold time: must hold selection 15 frames before deselection allowed
		///   - Hysteresis: 2.5x detection range before clearing
		///   - Cooldown: after clearing, same object blocked for 10 frames
		///   - Near distance filter: reject hits beyond nearRange (AABB can catch far centers)
		void updateSelection(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld,
			const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward,
			float nearRange, float farRange, RE::TESObjectREFR* otherHandRef)
		{
			// Don't run selection while in grab/hold states (Phase 3+)
			if (_state != HandState::Idle && _state != HandState::SelectedClose) return;

			// Near detection — every frame
			auto nearCandidate = findCloseObject(bhkWorld, hknpWorld,
				palmPos, palmForward, nearRange, _isLeft, otherHandRef);

			// Far detection — every 3rd frame (90fps → 30Hz, still responsive)
			SelectedObject farCandidate;
			_farDetectCounter++;
			if (_farDetectCounter >= 3) {
				_farDetectCounter = 0;
				farCandidate = findFarObject(bhkWorld, hknpWorld,
					palmPos, palmForward, farRange, otherHandRef);
				_cachedFarCandidate = farCandidate;
			} else {
				farCandidate = _cachedFarCandidate;
			}

			// Cooldown: block reselection of recently-cleared object
			if (_deselectCooldown > 0) {
				_deselectCooldown--;
				if (nearCandidate.refr == _lastDeselectedRef) nearCandidate.clear();
				if (farCandidate.refr == _lastDeselectedRef) farCandidate.clear();
				if (_deselectCooldown == 0) _lastDeselectedRef = nullptr;
			}

			// Priority: near beats far
			SelectedObject best = nearCandidate.isValid() ? nearCandidate : farCandidate;

			if (best.refr == _currentSelection.refr && best.isValid()) {
				// Same object still detected — update distance, increment hold counter
				_currentSelection.distance = best.distance;
				_selectionHoldFrames++;
			} else if (best.isValid()) {
				// New/different object detected — log with object name and type
				auto* baseObj = best.refr->GetObjectReference();
				const char* typeName = baseObj ? baseObj->GetFormTypeString() : "???";
				// Get name from base form (TESBoundObject), not the placed reference
				auto objName = baseObj
					? RE::TESFullName::GetFullName(*baseObj, false)
					: std::string_view{};
				const char* nameStr = objName.empty() ? "(unnamed)" : objName.data();

				if (_currentSelection.isValid()) {
					ROCK_LOG_INFO(Hand, "{} hand switched → {} [{}] '{}' formID={:08X} dist={:.1f}",
						handName(),
						best.isFarSelection ? "far" : "near",
						typeName, nameStr,
						best.refr->GetFormID(), best.distance);
				} else {
					ROCK_LOG_INFO(Hand, "{} hand selected {} [{}] '{}' formID={:08X} dist={:.1f}",
						handName(), best.isFarSelection ? "far" : "near",
						typeName, nameStr,
						best.refr->GetFormID(), best.distance);
				}
				// Stop highlight on previous selection
				stopSelectionHighlight();

				_currentSelection = best;
				_state = HandState::SelectedClose;
				_selectionHoldFrames = 0;

				// Start highlight on new selection
				playSelectionHighlight(best.refr);
			} else if (_currentSelection.isValid()) {
				// No candidate found — apply hysteresis with minimum hold time
				constexpr int MIN_HOLD_FRAMES = 15;  // ~0.17s at 90Hz

				if (_selectionHoldFrames < MIN_HOLD_FRAMES) {
					// Too soon to deselect — keep current, increment hold
					_selectionHoldFrames++;
					return;
				}

				// Hysteresis: 2.5x detection range before clearing
				float hysteresisRange = _currentSelection.isFarSelection
					? farRange * 2.5f : nearRange * 2.5f;

				// Re-check distance to current selection (guard against stale bodyId)
				if (_currentSelection.bodyId.value != 0x7FFF'FFFF) {
					auto& body = hknpWorld->GetBody(_currentSelection.bodyId);
					// Validate body is still active (motionIndex > 0 and reasonable)
					if (body.motionIndex > 0 && body.motionIndex < 4096) {
						auto* motion = hknpWorld->GetBodyMotion(_currentSelection.bodyId);
						if (motion) {
							RE::NiPoint3 objPos = hkVectorToNiPoint(motion->position);
							_currentSelection.distance = (objPos - palmPos).Length();
						}
					} else {
						// Body was unloaded (NPC left cell, etc.) — clear immediately
						_currentSelection.clear();
						_state = HandState::Idle;
						_selectionHoldFrames = 0;
						return;
					}
				}

				// Check if ref is still valid
				bool refInvalid = !_currentSelection.refr ||
					_currentSelection.refr->IsDeleted() ||
					_currentSelection.refr->IsDisabled();

				if (refInvalid || _currentSelection.distance > hysteresisRange) {
					ROCK_LOG_INFO(Hand, "{} hand cleared (formID={:08X}, dist={:.1f}, held={}f)",
						handName(), _currentSelection.refr ? _currentSelection.refr->GetFormID() : 0,
						_currentSelection.distance, _selectionHoldFrames);
					stopSelectionHighlight();
					_lastDeselectedRef = _currentSelection.refr;
					_deselectCooldown = 10;  // block reselection for 10 frames (~0.11s)
					_currentSelection.clear();
					_state = HandState::Idle;
					_selectionHoldFrames = 0;
				}
			} else {
				// Nothing selected, nothing found — stay idle
				_state = HandState::Idle;
			}
		}

		// --- Collision body (Phase 1) ---

		RE::hknpBodyId getCollisionBodyId() const { return _collisionBodyId; }
		bool hasCollisionBody() const { return _collisionBodyId.value != INVALID_BODY_ID; }

		/// Create a box collision body in the Havok world.
		/// WHY: HIGGS uses hkpBoxShape for flat palm collision. A box better represents
		/// a palm than a capsule — flat contact surface, no rounded ends for objects
		/// to slide off. Created via RE'd convex polytope pattern (Ghidra 2026-03-23).
		/// @param world       The hknpWorld to create the body in
		/// @param orientation Initial rotation quaternion {x,y,z,w} in Havok format
		/// @param halfExtentX Palm half-width (Havok units, side to side)
		/// @param halfExtentZ Palm half-thickness (Havok units, palm normal direction)
		bool createCollision(RE::hknpWorld* world,
			const RE::hkVector4f& orientation, float halfExtentX, float halfExtentY, float halfExtentZ)
		{
			if (hasCollisionBody()) {
				ROCK_LOG_WARN(Hand, "{} hand already has collision body — skipping create", handName());
				return false;
			}

			if (!world) {
				ROCK_LOG_ERROR(Hand, "{} hand createCollision: world is null", handName());
				return false;
			}

			// Box-shaped palm collider from INI config (Havok units, divide game units by 70):
			//   X = palm half-width (side to side)
			//   Y = palm half-depth (finger direction)
			//   Z = palm half-thickness (palm normal)
			// Defaults: X=0.06 (~12cm), Y=0.02 (~4cm), Z=0.015 (~3cm)
			const float hx = halfExtentX;
			const float hy = halfExtentY;
			const float hz = halfExtentZ;
			auto* shape = CreateBoxShape(hx, hy, hz, g_rockConfig.rockHandCollisionBoxRadius);
			if (!shape) {
				ROCK_LOG_ERROR(Hand, "{} hand createCollision: box shape creation failed", handName());
				return false;
			}

			// Allocate motion for keyframed body
			auto motionId = world->AllocateMotion();

			// Fill body creation info
			RE::hknpBodyCinfo cinfo;
			cinfo.shape = shape;
			cinfo.motionId.value = motionId;
			cinfo.motionPropertiesId.value = 0xFF;  // Default — setBodyKeyframed will override to KEYFRAMED
			// FilterInfo format: bits 0-6 = layer, bits 16-31 = collision group.
			// Reference CLUTTER bodies have filterInfo like 0x000B0004 (group=11, layer=4).
			// Group 0 may be rejected by the filter. Use group 11 to match existing CLUTTER.
			cinfo.collisionFilterInfo = (0x000B << 16) | (ROCK_HAND_LAYER & 0x7F);
			// CRITICAL: Create body at ORIGIN (0,0,0), not at the hand position.
			// CreateBody stores cinfo.position as the COM-to-body offset in the W
			// components (body+0x0C/+0x1C/+0x2C). If non-zero, SetBodyTransform
			// produces rotation-dependent positional drift because:
			//   body.translation = motion.COM - R * W
			// With W=0, body.translation = motion.COM always (no drift).
			// We teleport to the correct position via SetBodyTransform after creation.
			cinfo.position = RE::hkVector4f(0.0f, 0.0f, 0.0f, 0.0f);
			cinfo.orientation = orientation;
			// High-friction hand material for natural grip (registered once, cached)
			cinfo.materialId = registerHandMaterial(world);
			cinfo.userData = 0;  // MUST be 0 — FOIslandActivationListener dereferences unconditionally
			cinfo.name = _isLeft ? "ROCK_LeftHand" : "ROCK_RightHand";

			// Create body in the world (immediate mode)
			auto bodyId = world->CreateBody(cinfo);
			if (bodyId.value == INVALID_BODY_ID) {
				ROCK_LOG_ERROR(Hand, "{} hand createCollision: CreateBody returned invalid ID", handName());
				// Shape was allocated by CreateCapsuleShape (Havok heap). On success, CreateBody
				// takes ownership and bumps refcount. On failure, we still hold the only reference.
				// Decrement refcount to signal the Havok heap can reclaim the memory.
				if (shape) {
					auto* refObj = reinterpret_cast<RE::hkReferencedObject*>(shape);
					std::uint32_t val = refObj->memSizeAndRefCount;
					std::uint16_t refCount = static_cast<std::uint16_t>(val & 0xFFFF);
					if (refCount > 0 && refCount != 0xFFFF) {
						refObj->memSizeAndRefCount = (val & 0xFFFF0000u) | static_cast<std::uint32_t>(refCount - 1);
						ROCK_LOG_WARN(Hand, "{} hand: shape at {:p} refcount decremented (body creation failed)", handName(), (void*)shape);
					}
				}
				return false;
			}

			_collisionBodyId = bodyId;
			_shape = shape;

			// Enable contact modifier flag (0x20000) so contact events fire for this body.
			// Enable keep-awake flag (0x8000000) so the body never goes to sleep.
			world->EnableBodyFlags(bodyId, 0x08020000, 1);

			// Set body as KEYFRAMED via Bethesda's proper API.
			// This does critical setup that raw cinfo.motionPropertiesId=2 doesn't:
			//   1. Zeroes inverse mass/inertia in motion (infinite mass — pushes without being pushed)
			//   2. Sets motionPropertiesId=2 in the MOTION struct (motion+0x38, separate from body+0x72)
			//   3. Sets IS_KEYFRAMED flag (0x04) in body flags (body+0x40)
			//   4. Rebuilds collision caches and updates broadphase
			{
				typedef void setKeyframed_t(void*, std::uint32_t);
				static REL::Relocation<setKeyframed_t> setBodyKeyframed{ REL::Offset(0x1DF5CB0) };
				setBodyKeyframed(world, bodyId.value);
			}

			// Unfreeze motion timeFactor (AllocateMotion creates at 0.0 = frozen)
			auto* motion_init = world->GetBodyMotion(bodyId);
			if (motion_init) {
				motion_init->timeFactor = 1.0f;
				motion_init->spaceSplitterWeight = 1.0f;
			}

			// Activate body
			world->ActivateBody(bodyId);

			ROCK_LOG_INFO(Hand, "{} hand collision created — bodyId={}, motionId={}",
				handName(), bodyId.value, motionId);

			return true;
		}

		/// Destroy the collision body from the Havok world.
		void destroyCollision(RE::hknpWorld* world)
		{
			if (!hasCollisionBody()) {
				return;
			}

			if (world) {
				world->DestroyBodies(&_collisionBodyId, 1);
				ROCK_LOG_INFO(Hand, "{} hand collision destroyed — bodyId={}", handName(), _collisionBodyId.value);
			} else {
				ROCK_LOG_WARN(Hand, "{} hand collision orphaned (world null) — bodyId={}", handName(), _collisionBodyId.value);
			}

			_collisionBodyId.value = INVALID_BODY_ID;
			// Note: shape memory is managed by Havok's ref counting after body creation
			_shape = nullptr;
		}

		/// Update the collision body position to track the VR controller.
		/// WHY: SetBodyTransform alone teleports with zero velocity — the solver sees
		/// a stationary hand and produces weak/inconsistent push impulses. Bethesda's
		/// pattern (bhkNPCollisionObject::UpdateKeyframedTransform at 0x141e086e0) sets
		/// velocity via computeHardKeyFrame BEFORE positioning, so the solver knows
		/// hand speed and computes proper collision response. HIGGS does the same via
		/// hkpKeyFrameUtility::applyHardKeyFrame (hand.cpp:699).
		void updateCollisionTransform(RE::hknpWorld* world, const RE::NiTransform& handTransform,
			float deltaTime)
		{
			if (!hasCollisionBody() || !world) return;

			const auto targetPos = niPointToHkVector(handTransform.translate);

			// Read current body position for teleport detection
			auto* bodyArray = world->GetBodyArray();
			auto* bodyFloats = reinterpret_cast<float*>(&bodyArray[_collisionBodyId.value]);
			float curX = bodyFloats[12], curY = bodyFloats[13], curZ = bodyFloats[14];

			float dx = targetPos.x - curX, dy = targetPos.y - curY, dz = targetPos.z - curZ;
			float dist = sqrtf(dx*dx + dy*dy + dz*dz);
			bool isTeleport = (dist > 5.0f);

			// --- Step 1: Compute velocity BEFORE teleport ---
			// hknpWorld::computeHardKeyFrame (0x14153a6a0) reads current body state
			// and computes both linear AND angular velocity to reach target in one step.
			// CRITICAL: Previous code had ZERO angular velocity — the solver couldn't
			// predict hand rotation, causing oscillation (20-agent investigation 2026-03-22).
			// HIGGS uses hkpKeyFrameUtility::applyHardKeyFrame which does the same.
			alignas(16) float linVelOut[4] = {0,0,0,0};
			alignas(16) float angVelOut[4] = {0,0,0,0};

			if (deltaTime > 0.0001f && !isTeleport) {
				alignas(16) float tgtPos[4] = { targetPos.x, targetPos.y, targetPos.z, 0.0f };
				alignas(16) float tgtQuat[4];
				niRotToHkQuat(handTransform.rotate, tgtQuat);  // PhysicsUtils.h — normalized

				// CRITICAL FIX (C2): Pass deltaTime, NOT 1/deltaTime.
				// Ghidra-verified: computeHardKeyFrame (0x14153a6a0) internally computes
				// rcpps(param5) = 1/param5. For velocity = displacement/dt, it needs
				// 1/param5 = 1/dt, therefore param5 = dt.
				// Bethesda's UpdateKeyframedTransform (0x141e086e0) passes global dt directly.
				// Previous code passed invDt (1/dt) → function computed 1/(1/dt) = dt
				// → velocity = displacement × dt ≈ 8000× too low at 90fps.
				typedef void computeHKF_t(void*, std::uint32_t, const float*, const float*,
					float, float*, float*);
				static REL::Relocation<computeHKF_t> computeHardKeyFrame{
					REL::Offset(0x153a6a0) };
				computeHardKeyFrame(world, _collisionBodyId.value,
					tgtPos, tgtQuat, deltaTime, linVelOut, angVelOut);
			}

			// --- Step 1b: Clamp velocity to prevent physics explosions ---
			// Safety net for tracking glitches / frame hitches. Scales to max magnitude
			// preserving direction — NOT zeroing. Motor damping needs a directional
			// velocity signal: force = -0.8 * (bodyB_vel - bodyA_vel). Zeroing bodyA
			// velocity makes damping oppose all bodyB motion (pure friction).
			// Thresholds are 10-25x above normal VR hand motion (1-50 rad/s, 5-20 m/s).
			// HIGGS setRotation fallback does NOT zero angular velocity either.
			{
				float MAX_LIN_VEL = g_rockConfig.rockMaxLinearVelocity;
				float MAX_ANG_VEL = g_rockConfig.rockMaxAngularVelocity;

				float linSpeed = sqrtf(linVelOut[0]*linVelOut[0] + linVelOut[1]*linVelOut[1] + linVelOut[2]*linVelOut[2]);
				if (linSpeed > MAX_LIN_VEL) {
					float s = MAX_LIN_VEL / linSpeed;
					linVelOut[0] *= s; linVelOut[1] *= s; linVelOut[2] *= s;
				}

				float angSpeed = sqrtf(angVelOut[0]*angVelOut[0] + angVelOut[1]*angVelOut[1] + angVelOut[2]*angVelOut[2]);
				if (angSpeed > MAX_ANG_VEL) {
					float s = MAX_ANG_VEL / angSpeed;
					angVelOut[0] *= s; angVelOut[1] *= s; angVelOut[2] *= s;
				}
			}

			// --- Step 2: Teleport body to target ---
			RE::hkTransformf hkTransform;
			hkTransform.rotation = handTransform.rotate;
			hkTransform.translation = RE::NiPoint4(targetPos.x, targetPos.y, targetPos.z, 0.0f);
			world->SetBodyTransform(_collisionBodyId, hkTransform, 0);

			// --- Step 3: Set velocity AFTER teleport ---
			// Always set velocity — zero on teleport to clear stale motion,
			// computed values on normal frames.
			{
				typedef void setVel_t(void*, std::uint32_t, const float*, const float*);
				static REL::Relocation<setVel_t> setBodyVelocity{ REL::Offset(0x1539F30) };
				setBodyVelocity(world, _collisionBodyId.value, linVelOut, angVelOut);
			}
		}

	private:
		bool _isLeft;
		HandState _state = HandState::Idle;
		HandState _prevState = HandState::Idle;  ///< Previous state for transition logging

		/// State transition control flags (HIGGS S17 pattern).
		/// idleDesired: set by any system that wants to drop to Idle (deviation, weapon drawn, etc.)
		/// grabRequested/releaseRequested: edge-detected from controller input
		bool _idleDesired = false;
		bool _grabRequested = false;
		bool _releaseRequested = false;

		/// Havok body ID for the hand collision shape.
		RE::hknpBodyId _collisionBodyId{ INVALID_BODY_ID };

		/// Cached shape pointer (owned by Havok after body creation).
		RE::hknpShape* _shape = nullptr;  // Box shape (convex polytope via CreateBoxShape)

		/// Debug visualization mesh (attached to scene graph). Shape switchable via INI.
		RE::NiNode* _debugSphere = nullptr;
		RE::NiNode* _debugParentNode = nullptr;  ///< Tracks which node the debug mesh is attached to
		int _debugShapeType = -1;  ///< Current loaded shape type (-1 = none loaded)

		// --- Selection state (Phase 2) ---
		SelectedObject _currentSelection;
		SelectedObject _cachedFarCandidate;  ///< Far detection runs every 3rd frame
		int _farDetectCounter = 0;
		int _selectionHoldFrames = 0;        ///< Frames current selection has been held
		int _deselectCooldown = 0;           ///< Frames remaining before reselection allowed
		RE::TESObjectREFR* _lastDeselectedRef = nullptr;  ///< Last deselected ref (for cooldown)

		// --- Touch state (for API) ---
		RE::TESObjectREFR* _lastTouchedRef = nullptr;
		std::uint32_t _lastTouchedFormID = 0;
		std::uint32_t _lastTouchedLayer = 0;
		int _touchActiveFrames = 100;  // Starts high = not touching

		// --- Held body collision tracking (Phase 2A: EntityCollisionListener equivalent) ---
		// Updated on the PHYSICS THREAD via handleContactEvent. Read on main thread.
		// Frame counter pattern: set to 0 on contact, incremented per main-thread frame.
		// IsColliding = counter < threshold. Stale contacts expire naturally.
		std::atomic<int> _heldBodyContactFrame{ 100 };  ///< Frames since last held-body contact (100 = no contact)

	public:
		/// Is the held body currently in contact with world geometry?
		/// Used for adaptive tau switching (0.03 normal → 0.01 colliding).
		/// HIGGS S10: EntityCollisionListener.IsColliding() equivalent.
		bool isHeldBodyColliding() const {
			return _heldBodyContactFrame.load(std::memory_order_acquire) < 5;  // Contact within last 5 frames
		}

		/// Called from PhysicsInteraction::handleContactEvent on the PHYSICS THREAD.
		/// Resets the contact frame counter when the held body has a contact.
		void notifyHeldBodyContact() {
			_heldBodyContactFrame.store(0, std::memory_order_release);
		}

		/// Tick the contact frame counter (called once per main-thread frame).
		/// Uses compare_exchange to avoid overwriting a physics-thread reset-to-zero
		/// that happened between our load and store (prevents losing contact notifications).
		void tickHeldBodyContact() {
			int current = _heldBodyContactFrame.load(std::memory_order_acquire);
			if (current < 100) {
				// Only increment if the value hasn't been changed by the physics thread
				_heldBodyContactFrame.compare_exchange_weak(current, current + 1,
					std::memory_order_release, std::memory_order_relaxed);
			}
		}

	private:
		// --- Grab state (Phase 3) ---
		ActiveConstraint _activeConstraint;
		SavedObjectState _savedObjectState;
		float _grabPointLocal[4] = {0,0,0,0};  ///< Grab point in object's Havok body-local space (for pivot)
		float _grabStartTime = 0.0f;  ///< Time since grab started (for HeldInit transition)
		int _heldLogCounter = 0;      ///< Per-hand throttle for held state debug log
		int _pivotBLogCounter = 0;    ///< Per-hand diagnostic counter for pivotB updates (S1 fix)
		int _notifCounter = 0;        ///< Per-hand in-game notification throttle (C2 fix)

		/// Frozen hand-to-object transform (HIGGS desiredNodeTransformHandSpace).
		/// Computed once at grab time: Invert(handWorld) * (objectWorld shifted by palmPos-grabPoint).
		/// Used per-frame for visual hand adjustment: adjustedHand = objectWorld * Invert(_grabHandSpace).
		RE::NiTransform _grabHandSpace;

		/// NiNode of the grabbed object's 3D root (for reading current world transform per-frame).
		/// I3 NOTE: _heldNode is a cached pointer to the held object's NiAVObject.
		/// It CAN become stale if the object's NiNode is destroyed mid-grab (cell
		/// unload, script disable). We guard with refr->IsDeleted()/IsDisabled()
		/// checks before every use, and re-derive via refr->Get3D() when possible.
		/// The refr validity check at the top of updateHeldObject catches most cases.
		RE::NiAVObject* _heldNode = nullptr;

		/// All body IDs of the currently held object (main thread only).
		/// Populated at grab time via collectHeldBodyIds.
		std::vector<std::uint32_t> _heldBodyIds;

		/// Thread-safe snapshot of held body IDs for physics thread access.
		/// Pattern: main thread writes array then publishes count with release.
		/// Physics thread reads count with acquire then reads array.
		/// Max 64 bodies per object (capped in collectBodyIdsRecursive).
		static constexpr int MAX_HELD_BODIES = 64;
		std::uint32_t _heldBodyIdsSnapshot[MAX_HELD_BODIES] = {};
		std::atomic<int> _heldBodyIdsCount{ 0 };

		/// Thread-safe holding flag — set when entering HeldInit, cleared on release.
		/// Physics thread uses this instead of reading non-atomic _state.
		std::atomic<bool> _isHoldingFlag{ false };


		// --- Selection highlight ---
		RE::TESObjectREFR* _highlightedRef = nullptr;  ///< Currently highlighted object
		RE::ShaderReferenceEffect* _highlightEffect = nullptr;  ///< Active shader effect

	public:
		/// Play highlight shader on a selected object (visual feedback before grab).
		/// Uses the game's built-in TESEffectShader system.
		void playSelectionHighlight(RE::TESObjectREFR* refr)
		{
			// TODO: Find correct FO4VR highlight shader FormID.
			// 0x00023038 doesn't exist in FO4VR. Need to search the ESM
			// or create a custom effect shader .esp.
			(void)refr;
		}

		/// Stop highlight shader on the currently highlighted object.
		void stopSelectionHighlight()
		{
			if (_highlightedRef && _highlightEffect) {
				_highlightEffect->finished = true;  // Tells the effect system to remove it
				_highlightEffect = nullptr;
			}
			_highlightedRef = nullptr;
		}

			/// Show/hide a debug box visualizer at the palm collider position.
		/// Box mesh is configurable via INI (iDebugColliderShape).
		/// Hot-reloadable — changing the INI value destroys the old mesh and loads the new one.
		/// Uses non-uniform scale via rotation matrix to match the actual collision box extents.
		void updateDebugSphere(const RE::NiPoint3& palmPos, bool show, RE::NiNode* parentNode,
			float hx = 0.06f, float hy = 0.02f, float hz = 0.015f, int shapeType = 0)
		{
			// Hot-reload: if shape type changed, destroy old and recreate
			if (_debugSphere && _debugShapeType != shapeType) {
				destroyDebugSphere();
			}

			if (show && !_debugSphere && parentNode) {
				const char* meshPath = nullptr;
				switch (shapeType) {
					case 0:  meshPath = "Data/Meshes/ROCK/DebugBox_Trigger256.nif"; break;
					case 1:  meshPath = "Data/Meshes/ROCK/DebugBox_Trigger512.nif"; break;
					case 2:  meshPath = "Data/Meshes/ROCK/DebugBox_Activator.nif"; break;
					case 3:  meshPath = "Data/Meshes/ROCK/DebugBox_Utility.nif"; break;
					case 4:  meshPath = "Data/Meshes/ROCK/DebugBox_VaultSuit.nif"; break;
					default: meshPath = "Data/Meshes/ROCK/DebugBox_Trigger256.nif"; break;
				}
				_debugSphere = f4cf::f4vr::getClonedNiNodeForNifFileSetName(meshPath);
				if (_debugSphere) {
					_debugSphere->name = RE::BSFixedString(_isLeft ? "ROCK_DebugL" : "ROCK_DebugR");
					parentNode->AttachChild(_debugSphere, true);
					_debugParentNode = parentNode;
					_debugSphere->flags.flags &= 0xfffffffffffffffe;
					_debugSphere->local.scale = 1.0f;
					_debugShapeType = shapeType;
					ROCK_LOG_INFO(Hand, "{} debug box created: type={} mesh={}",
						handName(), shapeType, meshPath);
				} else {
					ROCK_LOG_WARN(Hand, "{} debug box FAILED to load {}", handName(), meshPath);
				}
			}

			// Re-parent if the parent node changed (e.g. switched from FRIK bone to wand node)
			if (_debugSphere && parentNode && _debugParentNode != parentNode) {
				if (_debugParentNode) {
					_debugParentNode->DetachChild(_debugSphere);
				}
				parentNode->AttachChild(_debugSphere, true);
				_debugParentNode = parentNode;
			}

			if (_debugSphere && parentNode) {
				if (show) {
					_debugSphere->flags.flags &= 0xfffffffffffffffe;
					// Compute local offset: transform world palmPos into parent's local space
					RE::NiPoint3 offset = palmPos - parentNode->world.translate;
					_debugSphere->local.translate = parentNode->world.rotate.Transpose() * offset;

					// Non-uniform scale via rotation matrix to match collision box extents.
					// Scale factor depends on mesh size. Trigger boxes are 256 game units per side (half=128).
					float meshHalf = 128.0f;
					if (shapeType == 2) meshHalf = 64.0f;
					if (shapeType == 4) meshHalf = 32.0f;
					const float s = 70.0f / meshHalf;
					RE::NiMatrix3 scaledRot;
					scaledRot.entry[0][0] = hx * s;
					scaledRot.entry[1][0] = 0.0f;
					scaledRot.entry[2][0] = 0.0f;
					scaledRot.entry[0][1] = 0.0f;
					scaledRot.entry[1][1] = hy * s;
					scaledRot.entry[2][1] = 0.0f;
					scaledRot.entry[0][2] = 0.0f;
					scaledRot.entry[1][2] = 0.0f;
					scaledRot.entry[2][2] = hz * s;
					_debugSphere->local.rotate = scaledRot;
				} else {
					_debugSphere->flags.flags |= 0x1;
					_debugSphere->local.scale = 0;
				}
			}
		}

		void destroyDebugSphere()
		{
			if (_debugSphere) {
				_debugSphere->flags.flags |= 0x1;
				_debugSphere->local.scale = 0;
				if (_debugSphere->parent) {
					_debugSphere->parent->DetachChild(_debugSphere);
				}
				_debugSphere = nullptr;
				_debugParentNode = nullptr;
			}
		}
	};
}

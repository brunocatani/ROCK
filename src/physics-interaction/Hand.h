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
#include "BethesdaPhysicsBody.h"
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

#include <array>

namespace frik::rock
{
	/// Collision layer for ROCK hand collision bodies.
	/// Layer 43 — hand bodies. Collides with BIPED(8) for NPC touch/punch.
	/// Held objects keep their ORIGINAL layer (HIGGS pattern).
	/// CC push prevention is via processConstraintsCallback hook (surface velocity zeroing).
	constexpr std::uint32_t ROCK_HAND_LAYER = 43;

	/// Invalid body ID sentinel value (matches hknpBodyId template default).
	constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;

	/// Create a box-shaped collision body. See Hand.cpp for implementation.
	RE::hknpShape* CreateBoxShape(float hx, float hy, float hz, float convexRadius = 0.0f);

	/// Register a custom "ROCK_Hand" material with high friction. See Hand.cpp for implementation.
	RE::hknpMaterialId registerHandMaterial(RE::hknpWorld* world);

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
		void reset();

		// --- Held body ID tracking ---
		// At grab time, collect ALL body IDs of the grabbed object via bhkPhysicsSystem.
		// Used by processConstraintsCallback hook to zero surface velocity for held bodies.

		/// Collect all body IDs from ALL collision objects in the NiNode tree.
		/// Weapons have child NiNodes (scope, barrel, stock) each with their
		/// own bhkNPCollisionObject and separate physics systems.
		void collectHeldBodyIds(RE::TESObjectREFR* refr);

	private:
		void collectBodyIdsRecursive(RE::NiAVObject* node, int maxDepth = 10);

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
		bool getAdjustedHandTransform(RE::NiTransform& outTransform) const;

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
			const RE::NiPoint3& palmPos, const RE::NiPoint3& palmNormal,
			const RE::NiPoint3& pointingDirection,
			float nearRange, float farRange, RE::TESObjectREFR* otherHandRef);

		// --- Collision body (Phase 1) ---
		// Uses BethesdaPhysicsBody for proper engine integration (body+0x88, full API).

		RE::hknpBodyId getCollisionBodyId() const { return _handBody.getBodyId(); }
		bool hasCollisionBody() const { return _handBody.isValid(); }
		BethesdaPhysicsBody& getHandBody() { return _handBody; }
		const BethesdaPhysicsBody& getHandBody() const { return _handBody; }

		/// Create a box collision body via BethesdaPhysicsBody (proper 12-step pipeline).
		/// @param bhkWorld  The bhkWorld wrapper (needed for CreateInstance).
		bool createCollision(RE::hknpWorld* world, void* bhkWorld,
			float halfExtentX, float halfExtentY, float halfExtentZ);

		/// Destroy the collision body cleanly.
		void destroyCollision(void* bhkWorld);

		/// Update the collision body position to track the VR controller.
		/// Uses the manual Havok-space path: computeHardKeyFrame + setTransform + setVelocity.
		void updateCollisionTransform(RE::hknpWorld* world, const RE::NiTransform& handTransform,
			float deltaTime);

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

		/// Hand collision body with full Bethesda wrapper chain.
		/// Replaces raw _collisionBodyId + _shape with proper bhkNPCollisionObject pipeline.
		/// Gives access to DriveToKeyFrame, SetMotionType, body+0x88 back-pointer, etc.
		BethesdaPhysicsBody _handBody;

		/// Debug visualization mesh (attached to scene graph). Shape switchable via INI.
		RE::NiNode* _debugColliderVis = nullptr;         ///< Debug visualization mesh for the hand collider box
		RE::NiNode* _debugColliderVisParent = nullptr;   ///< Tracks which node the debug mesh is attached to
		int _debugColliderVisShape = -1;                  ///< Current loaded shape type (-1 = none loaded)
		std::array<RE::NiNode*, 8> _debugColliderVertexVis{};
		RE::NiNode* _debugHandOriginVis = nullptr;
		RE::NiNode* _debugPalmCenterVis = nullptr;
		RE::NiNode* _debugAxisXVis = nullptr;
		RE::NiNode* _debugAxisYVis = nullptr;
		RE::NiNode* _debugAxisZVis = nullptr;
		RE::NiNode* _debugBasisVisParent = nullptr;

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
		float _grabStartTime = 0.0f;  ///< Time since grab started (for HeldInit transition)
		int _heldLogCounter = 0;      ///< Per-hand throttle for held state debug log
		int _transformWitnessLogCounter = 0;  ///< Per-hand throttle for transform witness logging
		int _notifCounter = 0;        ///< Per-hand in-game notification throttle (C2 fix)

		/// Frozen visual hand-to-object transform.
		/// Computed once at grab time from the grabbed collidable node world transform:
		/// Invert(rawHandWorld) * shiftedObject.
		/// Used only for visible-hand reverse transform:
		/// adjustedHand = collidableNodeWorld * Invert(_grabHandSpace).
		RE::NiTransform _grabHandSpace;

		/// Frozen physics hand-to-object transform expressed in the authored ROCK hand basis.
		/// This is the constraint-facing equivalent of HIGGS desiredNodeTransformHandSpace and
		/// MUST share the same basis contract as the hand body / palm hand-space math.
		RE::NiTransform _grabConstraintHandSpace;

		/// FO4VR equivalent of HIGGS GetRigidBodyTLocalTransform.
		/// Rigid transform from the grabbed collidable node's local space to the
		/// Havok body local frame at grab time. This MUST be composed into the
		/// constraint transform chain; assuming identity makes pivotB/rotation drift.
		RE::NiTransform _grabBodyLocalTransform;

		/// Diagnostic-only root-node equivalent of _grabBodyLocalTransform.
		/// Captured from the reference 3D root at grab time so we can compare
		/// whether the grabbed collidable node or the reference root better
		/// matches the live body during hold.
		RE::NiTransform _grabRootBodyLocalTransform;

		/// Diagnostic-only owner-node equivalent of _grabBodyLocalTransform.
		/// Captured from the body-backed collision owner node at grab time so
		/// hold diagnostics can compare the binary-verified owner node against
		/// the selected collidable node and the reference root.
		RE::NiTransform _grabOwnerBodyLocalTransform;

		/// Grabbed collidable node (NOT blindly the 3D root) used for per-frame world
		/// transform/scale reads during hold.
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

		// Cached shader pointer — invalidated when config changes FormID
		RE::TESEffectShader* _cachedHighlightShader = nullptr;
		std::uint32_t _cachedHighlightFormID = 0;

	public:
		/// Play highlight shader on a selected object (visual feedback before grab).
		/// Uses FO4's TESEffectShader system. The FormID is configurable in ROCK.ini
		/// (sHighlightShaderFormID) and hot-reloaded via file-watcher.
		/// Edit the INI in-game, save, and the new shader applies on next selection.
		void playSelectionHighlight(RE::TESObjectREFR* refr)
		{
			if (!refr || refr == _highlightedRef) return;
			if (!g_rockConfig.rockHighlightEnabled || g_rockConfig.rockHighlightShaderFormID == 0) return;

			// Stop any existing highlight first
			stopSelectionHighlight();

			// Re-lookup shader if FormID changed (hot-reload support)
			if (_cachedHighlightFormID != g_rockConfig.rockHighlightShaderFormID) {
				_cachedHighlightFormID = g_rockConfig.rockHighlightShaderFormID;
				_cachedHighlightShader = RE::TESForm::GetFormByID<RE::TESEffectShader>(_cachedHighlightFormID);
				if (_cachedHighlightShader) {
					ROCK_LOG_INFO(Hand, "Highlight shader loaded: FormID 0x{:08X}", _cachedHighlightFormID);
				} else {
					ROCK_LOG_WARN(Hand, "Highlight shader FormID 0x{:08X} not found — disabling highlight", _cachedHighlightFormID);
				}
			}

			if (!_cachedHighlightShader) return;

			// Apply the shader with a long duration (we manage stop explicitly).
			_highlightEffect = refr->ApplyEffectShader(_cachedHighlightShader, 60.0f);
			_highlightedRef = refr;
		}

		/// Stop highlight shader on the currently highlighted object.
		/// Forces immediate removal by setting age past lifetime + finished flag.
		void stopSelectionHighlight()
		{
			if (_highlightEffect) {
				// Set finished AND force age past any duration to prevent fade.
				// Just finished=true allows the engine to fade out over time, causing
				// ghost highlights on rapidly-switched objects.
				_highlightEffect->finished = true;
				_highlightEffect->effectShaderAge = 999.0f;
				_highlightEffect->lifetime = 0.0f;
				_highlightEffect = nullptr;
			}
			_highlightedRef = nullptr;
		}

		/// Show/hide a debug collider visualizer using the actual collider transform center and rotation,
		/// plus tiny vertex markers at the 8 corners so the box extents are unambiguous.
		void updateDebugColliderVis(const RE::NiTransform& colliderTransform, bool show, RE::NiNode* parentNode,
			float hx = 0.05f, float hy = 0.09f, float hz = 0.015f, int shapeType = 0);

		/// Destroy the debug collider visualization mesh. See Hand.cpp for implementation.
		void destroyDebugColliderVis();

		/// Show/hide collider-centered basis markers plus a palm-center marker.
		/// The origin marker sits at the actual collider transform center so debug matches
		/// the real physics body placement even when hand-space offsets are non-zero.
		void updateDebugBasisVis(const RE::NiTransform& colliderTransform,
			const RE::NiPoint3& palmCenterWorld,
			bool show,
			RE::NiNode* parentNode);

		/// Destroy the raw hand-basis debug marker meshes. See Hand.cpp for implementation.
		void destroyDebugBasisVis();
	};
}

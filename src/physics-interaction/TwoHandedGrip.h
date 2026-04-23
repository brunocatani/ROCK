#pragma once

// TwoHandedGrip.h — Physics-based two-handed weapon gripping for ROCK.
//
// WHY: Replaces FRIK's angle-based offhand barrel detection with contact event
// detection (hand layer 43 touches weapon body layer 44) + meshGrabFound for
// precise grip point. The grip point determines WHERE on the weapon the offhand
// holds, giving natural leverage-based rotation. The weapon stays KEYFRAMED —
// rotation is computed geometrically from both hand positions (HIGGS pattern).
//
// KILLS: FRIK's WeaponPositionAdjuster offhand gripping (blockOffHandWeaponGripping tag system).
// USES: MeshGrab.h for surface point detection, HandPoseManager for finger curl.

#include <atomic>

#include "MeshGrab.h"
#include "PalmTransform.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"
#include "RE/NetImmerse/NiTransform.h"

namespace frik::rock
{
	/// Two-handed weapon grip state.
	enum class TwoHandedState
	{
		Inactive,        ///< Not gripping — FRIK's system may be active
		Touching,        ///< Off-hand is touching weapon body (contact detected)
		Gripping,        ///< Off-hand grip button held + grab point locked
	};

	/// Physics-based two-handed weapon grip.
	///
	/// Lifecycle: called from PhysicsInteraction::update() after weapon collision update.
	/// Requires WeaponCollision to have an active weapon body for contact detection.
	class TwoHandedGrip
	{
	public:
		/// Per-frame update. Handles state transitions, rotation, finger posing.
		/// @param weaponNode     The weapon NiNode (from FRIK, already positioned by primary hand)
		/// @param offhandTouching Whether the off-hand contact event fired this frame
		/// @param gripPressed    Whether the off-hand grip button is currently pressed
		/// @param isLeftHanded   Whether the player is left-handed (swaps primary/offhand)
		/// @param dt             Frame delta time
		void update(RE::NiNode* weaponNode,
			bool offhandTouching, bool gripPressed, bool isLeftHanded, float dt);

		/// Reset all state (called on shutdown/cell transition).
		void reset();

		/// Whether the off-hand is currently gripping the weapon.
		bool isGripping() const { return _state == TwoHandedState::Gripping; }

		/// Whether the off-hand is touching the weapon (but not yet gripping).
		bool isTouching() const { return _state == TwoHandedState::Touching; }

		/// Get the current state.
		TwoHandedState getState() const { return _state; }

	private:
		// --- State machine ---
		void transitionToTouching(RE::NiNode* weaponNode);
		void transitionToGripping(RE::NiNode* weaponNode, bool isLeftHanded);
		void transitionToInactive(bool isLeftHanded);

		// --- Per-frame updates ---
		void updateGripping(RE::NiNode* weaponNode,
			bool isLeftHanded, float dt);

		// --- Finger posing ---
		/// Set off-hand finger curl for barrel gripping.
		/// Uses a simple radius-based approximation (v1 — no mesh intersection).
		void setBarrelGripPose(bool isLeft);

		/// Clear off-hand finger pose (restore default).
		void clearBarrelGripPose(bool isLeft);

		// --- FRIK integration ---
		/// Kill FRIK's offhand grip and force-release if it was active.
		static void killFrikOffhandGrip();

		/// Restore FRIK's offhand grip system.
		static void restoreFrikOffhandGrip();

		// --- Helpers ---

		/// Convert a world-space point to weapon-local space.
		static RE::NiPoint3 worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode);

		/// Convert a weapon-local point to world space.
		static RE::NiPoint3 weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiNode* weaponNode);

		/// Move a hand bone to a world-space target position and propagate transforms.
		static void snapHandToWorldPos(RE::NiAVObject* handBone, const RE::NiPoint3& targetWorld,
			const RE::NiNode* weaponNode);

		// --- State ---
		TwoHandedState _state{ TwoHandedState::Inactive };

		/// Off-hand grab point in weapon-local space (from meshGrab — where the offhand touched).
		RE::NiPoint3 _offhandGripLocal{};

		/// Primary hand grip point in weapon-local space (where the dominant hand holds).
		RE::NiPoint3 _primaryGripLocal{};

		/// Grab normal (from mesh — used for future hand orientation alignment).
		RE::NiPoint3 _grabNormal{};

		/// Touch timeout — frames since last contact event. Touching expires after N frames.
		int _touchFrames{ 0 };
		static constexpr int TOUCH_TIMEOUT_FRAMES = 5;

		/// Smooth rotation blending factor (0-1, ramps up on grip activation).
		float _rotationBlend{ 0.0f };
		static constexpr float ROTATION_BLEND_SPEED = 8.0f;

		// Log throttle counter — reset per-instance for fresh diagnostics each session.
		int _gripLogCounter{ 0 };
	};

} // namespace frik::rock

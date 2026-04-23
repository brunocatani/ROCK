// TwoHandedGrip.cpp — Rigid-stick two-handed weapon gripping.
//
// MODEL: The weapon is a rigid stick. Two grip points are locked on it:
//   - Primary grip (pistol grip/handle) — where the dominant hand naturally holds
//   - Offhand grip (barrel/foregrip) — from meshGrabFound, where the player touched
//
// Each frame: compute weapon position + rotation from both controllers, then snap
// BOTH hand bones to their respective grip points on the weapon. Arms follow via IK.
// The weapon drives the hands, not the other way around.
//
// This replaces FRIK's broken approach where only the weapon rotates and the offhand
// floats at an imaginary point.

#include "TwoHandedGrip.h"

#include "api/FRIKApi.h"
#include "RockUtils.h"
#include "RockConfig.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"
#include "common/Quaternion.h"

namespace frik::rock
{
	/// Get hand world transform from raw wand node with collision offset applied,
	/// consistent with how collision body and grab system source their transforms.
	/// Falls back to FRIK API if wand node unavailable.
	static RE::NiTransform getWandTransform(bool isLeft)
	{
		auto* wand = isLeft ? f4vr::getLeftHandNode() : f4vr::getRightHandNode();
		if (wand) {
			// Apply the same collision offset as the collision body and grab system.
			// Without this, two-handed grip anchors at wrist while grab anchors at palm.
			RE::NiTransform result = wand->world;
			float offX = g_rockConfig.rockHandCollisionOffsetX * (isLeft ? -1.0f : 1.0f);
			float offY = g_rockConfig.rockHandCollisionOffsetY;
			float offZ = g_rockConfig.rockHandCollisionOffsetZ;
			const auto& R = result.rotate;
			constexpr float kHavokToGame = 70.0f;
			result.translate.x += (R.entry[0][0]*offX + R.entry[0][1]*offY + R.entry[0][2]*offZ) * kHavokToGame;
			result.translate.y += (R.entry[1][0]*offX + R.entry[1][1]*offY + R.entry[1][2]*offZ) * kHavokToGame;
			result.translate.z += (R.entry[2][0]*offX + R.entry[2][1]*offY + R.entry[2][2]*offZ) * kHavokToGame;
			return result;
		}
		return frik::api::FRIKApi::inst->getHandWorldTransform(handFromBool(isLeft));
	}
	// =========================================================================
	// Coordinate Helpers
	// =========================================================================

	RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiNode* weaponNode)
	{
		RE::NiPoint3 delta = sub(worldPos, weaponNode->world.translate);
		float invScale = 1.0f / weaponNode->world.scale;
		auto& R = weaponNode->world.rotate;
		// R^T * delta / scale  (transpose = inverse for orthonormal)
		return {
			(R.entry[0][0] * delta.x + R.entry[1][0] * delta.y + R.entry[2][0] * delta.z) * invScale,
			(R.entry[0][1] * delta.x + R.entry[1][1] * delta.y + R.entry[2][1] * delta.z) * invScale,
			(R.entry[0][2] * delta.x + R.entry[1][2] * delta.y + R.entry[2][2] * delta.z) * invScale
		};
	}

	RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiNode* weaponNode)
	{
		float s = weaponNode->world.scale;
		auto& R = weaponNode->world.rotate;
		return {
			(R.entry[0][0]*localPos.x + R.entry[0][1]*localPos.y + R.entry[0][2]*localPos.z)*s + weaponNode->world.translate.x,
			(R.entry[1][0]*localPos.x + R.entry[1][1]*localPos.y + R.entry[1][2]*localPos.z)*s + weaponNode->world.translate.y,
			(R.entry[2][0]*localPos.x + R.entry[2][1]*localPos.y + R.entry[2][2]*localPos.z)*s + weaponNode->world.translate.z
		};
	}

	void TwoHandedGrip::snapHandToWorldPos(RE::NiAVObject* handBone, const RE::NiPoint3& targetWorld,
		const RE::NiNode* weaponNode)
	{
		auto* parent = handBone->parent;
		if (!parent) return;

		RE::NiPoint3 delta = sub(targetWorld, parent->world.translate);
		float pScale = parent->world.scale;
		if (pScale < 0.001f) pScale = 1.0f;

		auto& pRot = parent->world.rotate;
		// parentWorldRot^T * delta / parentScale
		handBone->local.translate.x = (pRot.entry[0][0]*delta.x + pRot.entry[1][0]*delta.y + pRot.entry[2][0]*delta.z) / pScale;
		handBone->local.translate.y = (pRot.entry[0][1]*delta.x + pRot.entry[1][1]*delta.y + pRot.entry[2][1]*delta.z) / pScale;
		handBone->local.translate.z = (pRot.entry[0][2]*delta.x + pRot.entry[1][2]*delta.y + pRot.entry[2][2]*delta.z) / pScale;

		f4vr::updateTransformsDown(handBone, true, weaponNode ? weaponNode->name.c_str() : nullptr);
	}

	// =========================================================================
	// Per-Frame Update (State Machine)
	// =========================================================================

	void TwoHandedGrip::update(RE::NiNode* weaponNode,
		bool offhandTouching, bool gripPressed, bool isLeftHanded, float dt)
	{
		if (!frik::api::FRIKApi::inst || !frik::api::FRIKApi::inst->isSkeletonReady() || !weaponNode) {
			if (_state != TwoHandedState::Inactive) {
				transitionToInactive(isLeftHanded);
			}
			return;
		}

		switch (_state) {
		case TwoHandedState::Inactive:
			if (offhandTouching) {
				transitionToTouching(weaponNode);
			}
			break;

		case TwoHandedState::Touching:
			if (offhandTouching) {
				_touchFrames = 0;
			} else {
				_touchFrames++;
				if (_touchFrames > TOUCH_TIMEOUT_FRAMES) {
					_state = TwoHandedState::Inactive;
					break;
				}
			}
			if (gripPressed) {
				transitionToGripping(weaponNode, isLeftHanded);
			}
			break;

		case TwoHandedState::Gripping:
			if (!gripPressed) {
				transitionToInactive(isLeftHanded);
			} else {
				updateGripping(weaponNode, isLeftHanded, dt);
			}
			break;
		}
	}

	void TwoHandedGrip::reset()
	{
		if (_state == TwoHandedState::Gripping) {
			restoreFrikOffhandGrip();
			clearBarrelGripPose(true);
			clearBarrelGripPose(false);
		}
		_state = TwoHandedState::Inactive;
		_touchFrames = 0;
		_rotationBlend = 0.0f;
		_offhandGripLocal = {};
		_primaryGripLocal = {};
		_grabNormal = {};
	}

	// =========================================================================
	// State Transitions
	// =========================================================================

	void TwoHandedGrip::transitionToTouching(RE::NiNode* weaponNode)
	{
		_state = TwoHandedState::Touching;
		_touchFrames = 0;
		ROCK_LOG_INFO(Weapon, "TwoHandedGrip: TOUCHING weapon='{}'", weaponNode->name.c_str());
	}

	void TwoHandedGrip::transitionToGripping(
		RE::NiNode* weaponNode, bool isLeftHanded)
	{
		bool offhandIsLeft = !isLeftHanded;

		// 1. Kill FRIK's offhand grip
		killFrikOffhandGrip();

		// 2. Compute PRIMARY grip point (where the dominant hand currently is on the weapon)
		auto primaryTransform = getWandTransform(!offhandIsLeft);
		_primaryGripLocal = worldToWeaponLocal(primaryTransform.translate, weaponNode);

		// 3. Compute OFFHAND grip point via meshGrabFound (precise surface contact)
		auto offhandTransform = getWandTransform(offhandIsLeft);
		RE::NiPoint3 palmPos = computePalmPosition(offhandTransform, offhandIsLeft);
		RE::NiPoint3 palmDir = computePalmForward(offhandTransform, offhandIsLeft);

		std::vector<TriangleData> triangles;
		extractAllTriangles(weaponNode, triangles);

		GrabPoint grabPoint;
		bool meshFound = false;
		if (!triangles.empty()) {
			meshFound = findClosestGrabPoint(triangles, palmPos, palmDir,
				1.0f, 0.3f, grabPoint);
		}

		if (meshFound) {
			_offhandGripLocal = worldToWeaponLocal(grabPoint.position, weaponNode);
			_grabNormal = grabPoint.normal;
		} else {
			// Fallback: use palm position
			_offhandGripLocal = worldToWeaponLocal(palmPos, weaponNode);
			_grabNormal = palmDir;
		}

		// 4. Set finger pose
		setBarrelGripPose(offhandIsLeft);

		// 5. Enter gripping state
		_state = TwoHandedState::Gripping;
		_rotationBlend = 0.0f;

		// Diagnostic
		float gripDist = std::sqrt(dot(sub(_offhandGripLocal, _primaryGripLocal),
			sub(_offhandGripLocal, _primaryGripLocal)));
		ROCK_LOG_INFO(Weapon, "TwoHandedGrip: GRIP ACTIVATED — weapon='{}', "
			"primaryLocal=({:.3f},{:.3f},{:.3f}), offhandLocal=({:.3f},{:.3f},{:.3f}), "
			"gripSeparation={:.3f}, meshGrab={}, triangles={}",
			weaponNode->name.c_str(),
			_primaryGripLocal.x, _primaryGripLocal.y, _primaryGripLocal.z,
			_offhandGripLocal.x, _offhandGripLocal.y, _offhandGripLocal.z,
			gripDist, meshFound ? "YES" : "FALLBACK", triangles.size());
	}

	void TwoHandedGrip::transitionToInactive(bool isLeftHanded)
	{
		bool offhandIsLeft = !isLeftHanded;
		clearBarrelGripPose(offhandIsLeft);
		restoreFrikOffhandGrip();

		_state = TwoHandedState::Inactive;
		_touchFrames = 0;
		_rotationBlend = 0.0f;
		_offhandGripLocal = {};
		_primaryGripLocal = {};
		_grabNormal = {};

		ROCK_LOG_INFO(Weapon, "TwoHandedGrip: GRIP RELEASED");
	}

	// =========================================================================
	// Per-Frame Gripping — Rigid Stick Model
	// =========================================================================

	void TwoHandedGrip::updateGripping(RE::NiNode* weaponNode,
		bool isLeftHanded, float dt)
	{
		bool offhandIsLeft = !isLeftHanded;

		// Blend ramp (smooth transition on grip start)
		_rotationBlend = (std::min)(1.0f, _rotationBlend + dt * ROTATION_BLEND_SPEED);

		// --- Step 1: Read both controllers' world positions (raw wand nodes) ---
		auto primaryTransform = getWandTransform(!offhandIsLeft);
		auto offhandTransform = getWandTransform(offhandIsLeft);
		RE::NiPoint3 primaryController = primaryTransform.translate;
		RE::NiPoint3 offhandController = offhandTransform.translate;

		// --- Step 2: Compute weapon axis in BOTH spaces ---

		// Weapon-local axis: primary grip → offhand grip (the "barrel direction" between grips)
		RE::NiPoint3 axisLocal = sub(_offhandGripLocal, _primaryGripLocal);
		float axisLocalLen = std::sqrt(dot(axisLocal, axisLocal));
		if (axisLocalLen < 0.001f) return;  // Grips too close, can't determine axis
		axisLocal = normalize(axisLocal);

		// World-space desired axis: primary controller → offhand controller
		RE::NiPoint3 axisDesired = sub(offhandController, primaryController);
		float axisDesiredLen = std::sqrt(dot(axisDesired, axisDesired));
		if (axisDesiredLen < 0.001f) return;
		axisDesired = normalize(axisDesired);

		// Current weapon-world axis: transform the local axis into current world space
		RE::NiPoint3 axisCurrent = normalize(sub(
			weaponLocalToWorld(_offhandGripLocal, weaponNode),
			weaponLocalToWorld(_primaryGripLocal, weaponNode)));

		// --- Step 3: Compute rotation from current axis to desired axis ---
		common::Quaternion rotDelta;
		rotDelta.vec2Vec(axisCurrent, axisDesired);

		// Blend with identity for smooth transition
		if (_rotationBlend < 0.99f) {
			common::Quaternion identity(0, 0, 0, 1);
			identity.slerp(_rotationBlend, rotDelta);
			rotDelta = identity;
		}

		RE::NiMatrix3 worldRotDelta = rotDelta.getMatrix();

		// --- Step 4: Apply rotation in weapon-local space ---
		// Convert world rotation to local: localDelta = parentInv * worldDelta * parent
		RE::NiMatrix3 localRotDelta;
		auto* parent = weaponNode->parent;
		if (parent) {
			auto& pRot = parent->world.rotate;
			RE::NiMatrix3 pRotInv;
			for (int r = 0; r < 3; r++)
				for (int c = 0; c < 3; c++)
					pRotInv.entry[r][c] = pRot.entry[c][r];
			localRotDelta = pRotInv * (worldRotDelta * pRot);
		} else {
			localRotDelta = worldRotDelta;
		}

		weaponNode->local.rotate = localRotDelta * weaponNode->local.rotate;

		// --- Step 5: Position weapon so primary grip lands on primary controller ---
		// After rotation, the primary grip point is at:
		RE::NiPoint3 primaryGripWorld = weaponLocalToWorld(_primaryGripLocal, weaponNode);
		// We want it at primaryController. Compute offset:
		RE::NiPoint3 posOffset = sub(primaryController, primaryGripWorld);
		// Apply offset to weapon local translate (convert world offset to parent-local)
		if (parent) {
			auto& pRot = parent->world.rotate;
			float pScale = parent->world.scale;
			if (pScale < 0.001f) pScale = 1.0f;
			weaponNode->local.translate.x += (pRot.entry[0][0]*posOffset.x + pRot.entry[1][0]*posOffset.y + pRot.entry[2][0]*posOffset.z) / pScale;
			weaponNode->local.translate.y += (pRot.entry[0][1]*posOffset.x + pRot.entry[1][1]*posOffset.y + pRot.entry[2][1]*posOffset.z) / pScale;
			weaponNode->local.translate.z += (pRot.entry[0][2]*posOffset.x + pRot.entry[1][2]*posOffset.y + pRot.entry[2][2]*posOffset.z) / pScale;
		} else {
			weaponNode->local.translate.x += posOffset.x;
			weaponNode->local.translate.y += posOffset.y;
			weaponNode->local.translate.z += posOffset.z;
		}

		// Propagate weapon transforms so world matches local
		f4vr::updateTransforms(weaponNode);

		// --- Step 6: Snap BOTH hands to their grip points on the weapon ---
		// After weapon repositioning, compute where each grip point is now in world space,
		// then move each hand bone there. The hands are SLAVES to the weapon.

		RE::NiPoint3 primaryGripFinal = weaponLocalToWorld(_primaryGripLocal, weaponNode);
		RE::NiPoint3 offhandGripFinal = weaponLocalToWorld(_offhandGripLocal, weaponNode);

		// Primary hand = dominant hand = NOT the offhand
		auto primaryHandEnum = offhandIsLeft ? frik::api::FRIKApi::Hand::Right : frik::api::FRIKApi::Hand::Left;
		auto offhandHandEnum = offhandIsLeft ? frik::api::FRIKApi::Hand::Left : frik::api::FRIKApi::Hand::Right;
		auto* primaryHandNode = frik::api::FRIKApi::inst->getHandNode(primaryHandEnum);
		auto* offhandHandNode = frik::api::FRIKApi::inst->getHandNode(offhandHandEnum);

		if (primaryHandNode) {
			snapHandToWorldPos(primaryHandNode, primaryGripFinal, weaponNode);
		}
		if (offhandHandNode) {
			snapHandToWorldPos(offhandHandNode, offhandGripFinal, weaponNode);
		}

		// --- Diagnostic log (throttled) ---
		if (++_gripLogCounter >= 90) {
			_gripLogCounter = 0;
			float separation = std::sqrt(dot(sub(primaryGripFinal, offhandGripFinal),
				sub(primaryGripFinal, offhandGripFinal)));
			ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: blend={:.2f}, separation={:.1f}gu, "
				"primaryGrip=({:.1f},{:.1f},{:.1f}), offhandGrip=({:.1f},{:.1f},{:.1f})",
				_rotationBlend, separation,
				primaryGripFinal.x, primaryGripFinal.y, primaryGripFinal.z,
				offhandGripFinal.x, offhandGripFinal.y, offhandGripFinal.z);
		}
	}

	// =========================================================================
	// Finger Posing
	// =========================================================================

	void TwoHandedGrip::setBarrelGripPose(bool isLeft)
	{
		// Barrel grip — fingers wrap around cylindrical surface.
		// 0.0 = fully closed, 1.0 = fully open.
		// Thumb rests on top (open), other fingers curl around barrel.
		static constexpr float BARREL_GRIP_POSE[] = {
			0.85f, 0.80f, 0.75f,   // thumb: mostly open, slight curl
			0.35f, 0.30f, 0.25f,   // index: wraps around barrel
			0.30f, 0.25f, 0.20f,   // middle: tightest (longest finger)
			0.35f, 0.30f, 0.25f,   // ring: similar to index
			0.40f, 0.35f, 0.30f    // pinky: slightly less (shortest)
		};

		frik::api::FRIKApi::inst->setHandPoseCustomJointPositions("ROCK_BarrelGrip",
			handFromBool(isLeft), BARREL_GRIP_POSE);
	}

	void TwoHandedGrip::clearBarrelGripPose(bool isLeft)
	{
		frik::api::FRIKApi::inst->clearHandPose("ROCK_BarrelGrip", handFromBool(isLeft));
	}

	// =========================================================================
	// FRIK Integration
	// =========================================================================

	void TwoHandedGrip::killFrikOffhandGrip()
	{
		if (!frik::api::FRIKApi::inst) return;
		frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_TwoHanded", true);
		ROCK_LOG_INFO(Weapon, "FRIK offhand grip SUPPRESSED");
	}

	void TwoHandedGrip::restoreFrikOffhandGrip()
	{
		if (!frik::api::FRIKApi::inst) return;
		frik::api::FRIKApi::inst->blockOffHandWeaponGripping("ROCK_TwoHanded", false);
		ROCK_LOG_INFO(Weapon, "FRIK offhand grip RESTORED");
	}

} // namespace frik::rock

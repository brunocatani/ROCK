#pragma once

// BethesdaPhysicsBody.h — Proper Bethesda-wrapped physics body for ROCK.
//
// WHY THIS EXISTS:
// ROCK previously created hand collision bodies via raw hknpWorld::CreateBody,
// bypassing Bethesda's physics wrapper layer. This made bodies "orphans" —
// they existed in Havok but were invisible to the engine. 13 Bethesda wrapper
// functions were unusable, body+0x88 was null (33 systems silently failed),
// and every feature required a workaround.
//
// This class follows the engine's own CreatePhantomBody (0x140f0a340) pipeline
// to create bodies with full Bethesda wrapper chain:
//   hknpShape → hknpPhysicsSystemData → bhkPhysicsSystem → bhkNPCollisionObject
//   → CreateInstance (body+0x88 back-pointer) → SetMotionType
//
// WHAT THIS UNLOCKS:
// - DriveToKeyFrame: single-call body positioning with auto-teleport fallback
// - SetMotionType: full body chain transition with correct flag management
// - SetTransform/SetVelocity: deferred-safe (TLS+0x1528 check)
// - ApplyLinearImpulse/ApplyPointImpulseAt: for throw velocity
// - GetCenterOfMassInWorld: true COM position
// - IsBodyConstrained: constraint status check
// - body+0x88 back-pointer: 33 engine systems work (raypick, contact events, etc.)
//
// REFERENCE: Ghidra blind audit 2026-03-31, CreatePhantomBody golden reference.
// Full function addresses in HavokOffsets.h.

#include <cstdint>
#include <cstring>

#include "PhysicsLog.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

namespace frik::rock
{

/// Motion type constants for bhkNPCollisionObject::SetMotionType.
/// Ghidra-verified: 0=STATIC, 1=DYNAMIC, 2=KEYFRAMED (blind audit 2026-03-31).
enum class BethesdaMotionType : int
{
	Static    = 0,
	Dynamic   = 1,
	Keyframed = 2
};

/// A physics body created through the proper Bethesda pipeline.
///
/// Wraps bhkNPCollisionObject + bhkPhysicsSystem + hknpPhysicsSystemData,
/// providing access to ALL Bethesda physics wrapper functions.
///
/// Lifecycle:
///   1. create()         — 12-step pipeline, body enters the world
///   2. [per-frame]      — driveToKeyFrame(), setTransform(), etc.
///   3. destroy()        — reverse pipeline, clean removal from world
///
/// Thread safety: All wrapper methods delegate to Bethesda functions which
/// handle their own locking. Safe to call from any thread.
class BethesdaPhysicsBody
{
public:
	BethesdaPhysicsBody() = default;
	~BethesdaPhysicsBody() = default;

	// Non-copyable, non-movable (owns Havok/Bethesda heap allocations with raw pointers)
	BethesdaPhysicsBody(const BethesdaPhysicsBody&) = delete;
	BethesdaPhysicsBody& operator=(const BethesdaPhysicsBody&) = delete;
	BethesdaPhysicsBody(BethesdaPhysicsBody&&) = delete;
	BethesdaPhysicsBody& operator=(BethesdaPhysicsBody&&) = delete;

	// =====================================================================
	// Lifecycle
	// =====================================================================

	/// Create a fully-wrapped physics body following the CreatePhantomBody pipeline.
	///
	/// @param world       The hknpWorld to create the body in.
	/// @param bhkWorld    The bhkWorld wrapper (for CreateInstance). Get via getPlayerBhkWorld().
	/// @param shape       Pre-created hknpShape (ownership transferred — refcount incremented).
	/// @param filterInfo  Collision filter info (bits 0-6=layer, bits 16-31=group).
	/// @param materialId  Material ID for friction/restitution.
	/// @param motionType  Initial motion type (STATIC=0, DYNAMIC=1, KEYFRAMED=2).
	/// @param name        Debug name for logging.
	/// @return true on success, false on allocation or registration failure.
	bool create(RE::hknpWorld* world, void* bhkWorld,
		RE::hknpShape* shape,
		std::uint32_t filterInfo,
		RE::hknpMaterialId materialId,
		BethesdaMotionType motionType,
		const char* name = "ROCK_Body");

	/// Destroy the body, removing it from the world and freeing all allocations.
	/// Safe to call if not created or already destroyed.
	void destroy(void* bhkWorld);

	/// Reset all state without destroying (for use when the world died externally).
	void reset();

	/// Create and link an NiNode for this body (proper scene graph integration).
	/// Sets bidirectional links: collObj+0x10 = node, node+0x100 = collObj.
	/// Following BuildSceneNodeHierarchy pattern (0x140ef21a0).
	/// @param name  Debug name for the node (e.g., "ROCK_RightHand").
	/// @return true on success.
	bool createNiNode(const char* name);

	/// Destroy the NiNode and clear scene graph links. Called by destroy().
	void destroyNiNode();

	// =====================================================================
	// State queries
	// =====================================================================

	bool isValid() const { return _created && _collisionObject != nullptr; }
	RE::hknpBodyId getBodyId() const { return _bodyId; }
	void* getCollisionObject() const { return _collisionObject; }
	void* getPhysicsSystem() const { return _physicsSystem; }

	// =====================================================================
	// Bethesda API — Body positioning (every-frame operations)
	// =====================================================================

	/// Drive the body to a target transform using the engine's full PD controller.
	/// Internally: computeHardKeyFrame → velocity limit check → teleport fallback.
	/// Replaces the manual 3-step process (compute + SetBodyTransform + setBodyVelocity).
	/// @param target  Target NiTransform in game-space (Havok scaling handled internally).
	/// @param dt      Frame delta time in seconds.
	/// @return true on success.
	bool driveToKeyFrame(const RE::NiTransform& target, float dt);

	/// Set body transform directly (deferred-safe via TLS+0x1528 check).
	void setTransform(const RE::hkTransformf& transform);

	/// Set body velocity (deferred-safe combined linear+angular).
	void setVelocity(const float* linVel, const float* angVel);

	// =====================================================================
	// Bethesda API — Motion type and state
	// =====================================================================

	/// Full motion type transition (STATIC/DYNAMIC/KEYFRAMED).
	/// Properly clears/sets body+0x40 flags, walks body chain, rebuilds mass properties.
	void setMotionType(BethesdaMotionType type);

	/// Set collision filter info with broadphase cache rebuild.
	void setCollisionFilterInfo(std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);

	/// Set body mass with proper motion rebuild.
	void setMass(float mass);

	// =====================================================================
	// Bethesda API — Impulse and force (for throw, physics response)
	// =====================================================================

	/// Apply a linear impulse (pure translation, no torque).
	void applyLinearImpulse(const float* impulse);

	/// Apply an impulse at a world-space point (generates both linear + angular response).
	/// This is EXACTLY what throw needs: "apply force at the release point."
	void applyPointImpulse(const float* impulse, const float* worldPoint);

	// =====================================================================
	// Bethesda API — Queries
	// =====================================================================

	/// Get the center of mass in Havok world coordinates.
	/// COM ≠ body origin. COM is at motion+0x00, body origin is at body+0x30.
	/// Constraint pivots use body origin; impulses act at COM.
	bool getCenterOfMassWorld(float& outX, float& outY, float& outZ);

	/// Get collision filter info.
	std::uint32_t getCollisionFilterInfo();

	/// Get the body's shape pointer (safe dual-path: live world or serialized).
	void* getShape();

	/// Check if the body has any active constraints.
	/// Used before release to detect if object is part of a constraint network.
	bool isConstrained();

	// =====================================================================
	// Scaffolded — Future features (stubs with logging, not yet implemented)
	// These signatures are stable. Implementation will follow when features
	// are built. Having the API defined NOW prevents refactoring later.
	// =====================================================================

	/// Set velocities such that a specific world point achieves target velocity.
	/// For throw: "make the point where I released the object move at this velocity."
	/// Uses hknpMotion::setPointVelocity (0x1417d1a20) — solves via effective mass matrix.
	/// STUB: Logs warning and returns. Implementation in Phase 4 (throw).
	void setPointVelocity(const float* targetVel, const float* worldPoint);

	/// Enable/disable body flags (contact reporting, keep-awake, etc.).
	void enableBodyFlags(std::uint32_t flags, std::uint32_t mode);

	/// Activate body (wake from sleep).
	void activateBody();

	/// Register a per-body contact event signal (VR melee pattern).
	/// Much more efficient than global contact modifiers — only fires for THIS body.
	/// Pattern from VRMelee_SetupImpactBody_Keyframed (0x140f38630).
	/// STUB: Implementation deferred — need to resolve getEventSignalForBody address.
	void registerContactSignal(const char* signalName);

	/// Get the NiNode (if created via createNiNode).
	void* getNiNode() const { return _niNode; }

private:
	void* _collisionObject = nullptr;   ///< bhkNPCollisionObject* (0x30 bytes, Bethesda alloc)
	void* _physicsSystem = nullptr;     ///< bhkPhysicsSystem* (0x28 bytes, Bethesda alloc)
	void* _systemData = nullptr;        ///< hknpPhysicsSystemData* (0x78 bytes, Havok alloc)
	void* _niNode = nullptr;            ///< NiNode* (0x180 bytes, Bethesda alloc, optional scene graph node)
	RE::hknpBodyId _bodyId{ 0x7FFF'FFFF };
	bool _created = false;
};

} // namespace frik::rock

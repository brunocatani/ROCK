#pragma once

// GrabConstraint.h — Custom Havok grab constraint for ROCK (HIGGS architecture).
//
// WHY THIS APPROACH:
// HIGGS (Skyrim VR) uses a custom GrabConstraintData with 6 atoms:
//   SetLocalTransforms, SetupStabilization, RagdollMotor (3 angular),
//   3× LinMotor (position). NO ball socket, NO limits, NO friction.
// The angular motors control orientation, linear motors control position.
// This gives soft, stable holds without pendulum orbiting.
//
// PREVIOUS APPROACH (FAILED):
// We tried extending hkpRagdollConstraintData by appending linear motor atoms
// after its 8 existing atoms (including ball socket). This failed because:
//   1. Ball socket hard-constrains pivot → pendulum orbiting
//   2. Ragdoll Runtime has 18 solver results → overlaps with our motor state
//   3. Ragdoll getConstraintInfo accesses atoms via struct members → breaks when
//      we overwrite the ball socket with a linear motor
//   4. Wrong vtable slot for getRuntimeInfo (slot 9 vs actual slot 18)
//   5. Threading: CreateConstraint acquires write lock held by physics thread
//
// NEW APPROACH (THIS FILE):
// Build a clean custom constraint from scratch. No ragdoll constructor.
// Manual atom initialization with Ghidra-verified byte offsets.
// Hardcoded getConstraintInfo/getRuntimeInfo at correct vtable slots.
// Runtime layout: 12 solver results (6 angular + 6 linear), 0x100 reported size.
//
// Atom layout (6 atoms, 0x148 bytes starting at constraintData+0x20):
//   +0x000: SetLocalTransforms (0x90)  — body-local reference frames
//   +0x090: SetupStabilization (0x10)  — solver stabilization params
//   +0x0A0: RagdollMotor (0x60)        — 3 angular motor axes
//   +0x100: LinMotor0 (0x18)           — linear motor X
//   +0x118: LinMotor1 (0x18)           — linear motor Y
//   +0x130: LinMotor2 (0x18)           — linear motor Z
//   Total atoms: 0x148 bytes
//
// RagdollMotor atom internal layout (0x60 bytes, SDK standard):
//   +0x00: type(2) +0x02: isEnabled(1) +0x04: initOffset(2) +0x06: prevAngOffset(2)
//   +0x10: m_target_bRca (48 bytes, 3×16 identity matrix)
//   +0x40: m_motors[3] (24 bytes, 3 pointers)
//
// All addresses verified in Ghidra against FO4VR binary.
// Vtable slots verified via hknpConstraint::init decompilation (0x1417e39c0).

#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpConstraintCinfo.h"
#include "RE/Havok/hknpWorld.h"

namespace frik::rock
{
	/// hkpPositionConstraintMotor layout (0x30 bytes).
	/// Verified via Ghidra decompile of clone function at 0x141f610f0.
	/// SDK inheritance: hkBaseObject → hkReferencedObject → hkpConstraintMotor
	///   → hkpLimitedForceConstraintMotor → hkpPositionConstraintMotor
	struct HkPositionMotor
	{
		void* vtable;                          // +0x00
		std::uint16_t memSizeAndFlags;         // +0x08 — hkReferencedObject
		std::int16_t referenceCount;           // +0x0A — -1 (0xFFFF) = immortal
		std::uint32_t pad0C;                   // +0x0C
		std::uint8_t type;                     // +0x10 — motor type enum (1 = position, 2 = velocity)
		std::uint8_t pad11[7];                 // +0x11
		float minForce;                        // +0x18
		float maxForce;                        // +0x1C
		float tau;                             // +0x20 — stiffness (0=soft, 1=rigid)
		float damping;                         // +0x24 — velocity damping (0=oscillatory, 1=critical)
		float proportionalRecoveryVelocity;    // +0x28 — P-gain for position error
		float constantRecoveryVelocity;        // +0x2C — constant correction velocity
	};
	static_assert(sizeof(HkPositionMotor) == 0x30);

	/// Size of HkPositionMotor as allocated by Havok heap (Ghidra-verified: 0x30 bytes).
	/// Used by havokHeapFree to pass correct deallocation size.
	constexpr std::size_t HK_POSITION_MOTOR_SIZE = 0x30;

	/// Vtable address for hkpPositionConstraintMotor (Ghidra-verified).
	inline constexpr std::uintptr_t MOTOR_VTABLE_POSITION = 0x142e95fe8;

	// =========================================================================
	// Custom constraint data layout — matches HIGGS GrabConstraintData
	// =========================================================================

	/// Total constraint data: 0x20 (header) + 0x148 (atoms) = 0x168 bytes.
	/// Header is 0x20 bytes (not 0x18): vtable(8) + refcount(4) + pad(4) + userData(8) + alignment(8).
	/// Confirmed by Ghidra: ragdoll getConstraintInfo does `ADD RCX, 0x20` for atoms start.
	inline constexpr std::size_t GRAB_CONSTRAINT_SIZE = 0x168;

	/// Atom byte offsets within constraint data (constraint base + offset).
	/// Atoms start at +0x20 (verified via ragdoll getConstraintInfo decompilation).
	inline constexpr int ATOMS_START       = 0x20;    // header size before atoms
	inline constexpr int ATOM_TRANSFORMS   = 0x20;    // SetLocalTransforms, 0x90 bytes
	inline constexpr int ATOM_STABILIZE    = 0xB0;    // SetupStabilization, 0x10 bytes
	inline constexpr int ATOM_RAGDOLL_MOT  = 0xC0;    // RagdollMotor, 0x60 bytes
	inline constexpr int ATOM_LIN_MOTOR_0  = 0x120;   // LinMotor axis 0, 0x18 bytes
	inline constexpr int ATOM_LIN_MOTOR_1  = 0x138;   // LinMotor axis 1, 0x18 bytes
	inline constexpr int ATOM_LIN_MOTOR_2  = 0x150;   // LinMotor axis 2, 0x18 bytes
	inline constexpr int ATOMS_SIZE        = 0x148;    // total atoms size (6 atoms)

	/// Atom type IDs (Havok SDK enum, verified in FO4VR binary).
	inline constexpr std::uint16_t ATOM_TYPE_SET_LOCAL_TRANSFORMS = 2;
	inline constexpr std::uint16_t ATOM_TYPE_SETUP_STABILIZATION  = 23;
	inline constexpr std::uint16_t ATOM_TYPE_RAGDOLL_MOTOR        = 19;
	inline constexpr std::uint16_t ATOM_TYPE_LIN_MOTOR            = 11;

	/// CORRECTED Runtime layout for 12 solver results (6 angular + 6 linear).
	///
	/// CRITICAL: hknp advances m_constraintRuntime per atom (unlike hkp/Skyrim).
	/// RagdollMotor produces 6 results (3 axes × 2), each LinMotor produces 2.
	/// Total: 6 + 2 + 2 + 2 = 12 results × 8 bytes = 0x60 bytes for solver data.
	/// Motor state (init flags + previous targets) placed AFTER all solver results.
	///
	/// +0x00: RagdollMotor results  (3×2×8 = 0x30)  ptr=0x00 at entry
	/// +0x30: LinMotor0 results     (2×8   = 0x10)  ptr=0x30
	/// +0x40: LinMotor1 results     (2×8   = 0x10)  ptr=0x40
	/// +0x50: LinMotor2 results     (2×8   = 0x10)  ptr=0x50
	/// +0x60: m_initialized[3]      (angular motor init flags, 3 bytes)
	/// +0x64: m_previousTargetAngles[3]   (3 × float = 0x0C)
	/// +0x70: m_initializedLinear[3]      (linear motor init flags, 3 bytes)
	/// +0x74: m_previousTargetPositions[3] (3 × float = 0x0C)
	/// Total: 0x80 bytes, report 0x100 to Havok (2× safety).
	///
	/// Verified via Ghidra disassembly:
	///   RagdollMotor handler at 0x141a5873c: ADD [RSI+0x8], 0x30 (advances ptr)
	///   LinMotor handler at 0x141a57162: ADD [RSI+0x8], 0x10 (advances ptr)
	///   Motor offsets are RELATIVE to current ptr, NOT runtime base.
	inline constexpr int RUNTIME_SOLVER_RESULTS     = 12;
	inline constexpr int RUNTIME_REPORTED_SIZE      = 0x100;

	// Runtime byte offsets — RELATIVE to each atom's runtime pointer position.
	// RagdollMotor enters with ptr at 0x00:
	inline constexpr int RT_RAGDOLL_INIT_OFFSET       = 0x60;  // → absolute 0x60 (m_initialized[0])
	inline constexpr int RT_RAGDOLL_PREV_ANG_OFFSET   = 0x64;  // → absolute 0x64 (m_previousTargetAngles[0])
	// LinMotor offsets are per-axis, computed in createGrabConstraint (see GHIDRA_CONFIRMED_ADDRESSES.md)

	/// Vtable slots in FO4VR (verified via hknpConstraint::init decompilation).
	/// CRITICAL: FO4VR has MORE slots than Skyrim VR (extra base-class virtuals).
	inline constexpr int VTABLE_SLOT_GET_TYPE             = 4;   // +0x20
	inline constexpr int VTABLE_SLOT_GET_CONSTRAINT_INFO  = 5;   // +0x28
	inline constexpr int VTABLE_SLOT_IS_VALID             = 6;   // +0x30
	inline constexpr int VTABLE_SLOT_GET_RUNTIME_INFO     = 18;  // +0x90
	inline constexpr int VTABLE_SLOT_ADD_INSTANCE         = 20;  // +0xA0

	/// Ragdoll constraint vtable address (base for cloning).
	inline constexpr std::uintptr_t RAGDOLL_VTABLE = 0x142e18298;

	// =========================================================================
	// Public types
	// =========================================================================

	/// State of an active grab constraint.
	struct ActiveConstraint
	{
		std::uint32_t constraintId = 0x7FFF'FFFF;
		void* constraintData = nullptr;
		HkPositionMotor* angularMotor = nullptr;
		HkPositionMotor* linearMotor = nullptr;
		float currentTau = 0.0f;
		float currentMaxForce = 0.0f;
		float targetMaxForce = 0.0f;

		bool isValid() const { return constraintId != 0x7FFF'FFFF && constraintData != nullptr; }

		void clear()
		{
			constraintId = 0x7FFF'FFFF;
			constraintData = nullptr;
			angularMotor = nullptr;
			linearMotor = nullptr;
			currentTau = 0.0f;
			currentMaxForce = 0.0f;
			targetMaxForce = 0.0f;
		}
	};

	/// Packed inverse inertia layout at hknpMotion+0x20 (Ghidra-verified).
	/// 8 bytes containing 4 int16 values (via packssdw):
	///   [0] = invInertiaX, [1] = invInertiaY, [2] = invInertiaZ, [3] = invMass
	/// Unpacking: float = (ushort)packed * 65536.0f
	/// Verified in: bhkNPCollisionObject::SetMass (0x141e08c00),
	///              hknpMotion::setPointVelocity (0x1417d1a20),
	///              hknpMotion::buildEffMassMatrixAt (0x1417d1ea0)
	inline constexpr int MOTION_PACKED_INERTIA_OFFSET = 0x20;

	/// Saved state of an object before grab (restored on release).
	struct SavedObjectState
	{
		RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
		RE::TESObjectREFR* refr = nullptr;
		std::uint32_t originalFilterInfo = 0;
		std::uint16_t originalMotionPropsId = 0;
		float inverseMass = 0.0f;
		std::int16_t savedPackedInertia[3] = { 0, 0, 0 };  ///< Original packed int16 inverse inertia (X,Y,Z) at motion+0x20
		bool inertiaModified = false;                        ///< Whether we modified inertia (to know if restore needed)

		bool isValid() const { return bodyId.value != 0x7FFF'FFFF && refr != nullptr; }

		void clear()
		{
			bodyId.value = 0x7FFF'FFFF;
			refr = nullptr;
			originalFilterInfo = 0;
			originalMotionPropsId = 0;
			inverseMass = 0.0f;
			savedPackedInertia[0] = savedPackedInertia[1] = savedPackedInertia[2] = 0;
			inertiaModified = false;
		}
	};

	/// Normalize an object's inverse inertia tensor for stable constraint grab.
	/// WHY: Long objects (weapons) have very different inertia on different axes.
	/// Without normalization, angular motors can't resist rotation on the low-inertia
	/// axes, causing the object to cartwheel around the hand.
	/// HIGGS clamps the ratio between max and min inertia to 10x and enforces
	/// a minimum inertia of 0.01 (= max inverse inertia of 100).
	/// Saves original values to savedState for restoration on release.
	void normalizeGrabbedInertia(RE::hknpWorld* world, RE::hknpBodyId bodyId, SavedObjectState& savedState);

	/// Restore the original inertia tensor after releasing a grabbed object.
	void restoreGrabbedInertia(RE::hknpWorld* world, SavedObjectState& savedState);

	/// Allocate and initialize a hkpPositionConstraintMotor.
	HkPositionMotor* createPositionMotor(float tau, float damping,
		float proportionalRecoveryVelocity, float constantRecoveryVelocity,
		float minForce, float maxForce);

	/// Create grab constraint between hand and object.
	/// Builds custom constraint from scratch (HIGGS architecture).
	/// pivotWorldHk = grab point in Havok world coordinates (constraint pivot).
	/// Create grab constraint with HIGGS-correct pivot and transform computation.
	/// Phase 0B: pivotA from palmWorldHk (palm center), pivotB from grabWorldHk (surface point).
	/// TransformB rotation from Inverse(grabHandSpace).rotation (frozen grab-time offset).
	/// Reference: HIGGS S00 sections 0.1a, 0.6, 0.6b; PRE-1 result (identity GetRigidBodyTLocalTransform).
	ActiveConstraint createGrabConstraint(
		RE::hknpWorld* world,
		RE::hknpBodyId handBodyId, RE::hknpBodyId objectBodyId,
		const float* palmWorldHk, const float* grabWorldHk,
		const RE::NiTransform& grabHandSpace,
		float tau, float damping, float maxForce,
		float proportionalRecovery, float constantRecovery);

	/// Destroy grab constraint and free resources.
	void destroyGrabConstraint(RE::hknpWorld* world, ActiveConstraint& constraint);

	/// Free shellcode memory allocated by buildCustomVtable.
	/// Call on DLL unload to prevent executable memory leaks on hot-reload.
	void cleanupGrabConstraintVtable();
}

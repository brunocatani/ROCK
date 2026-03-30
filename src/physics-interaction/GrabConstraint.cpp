#include "GrabConstraint.h"
#include "RockConfig.h"

// GrabConstraint.cpp — Custom grab constraint for ROCK (clean rewrite).
//
// WHY REWRITE: The previous approach extended hkpRagdollConstraintData by
// appending linear motors after its 8 atoms. This failed due to:
//   - Ball socket causing pendulum orbiting
//   - Runtime memory overlap (18 vs 6 solver results)
//   - Wrong vtable slot for getRuntimeInfo (slot 9 vs actual slot 18)
//   - Original getConstraintInfo reading overwritten atoms as garbage
//   - Threading deadlock (CreateConstraint vs physics step write lock)
//
// NEW APPROACH: Build a custom constraint from scratch matching HIGGS exactly.
// 6 atoms: SetLocalTransforms + SetupStabilization + RagdollMotor + 3×LinMotor.
// No ball socket, no limits, no friction.
// Custom vtable with HARDCODED getConstraintInfo/getRuntimeInfo values.
// All byte offsets verified in Ghidra against FO4VR binary.

#include <limits>
#include <windows.h>

namespace frik::rock
{
	// =========================================================================
	// Havok heap allocator
	// Pattern from motor clone function (0x141f610f0):
	// TlsGetValue(DAT_145b63b20) → allocator at TLS+0x58 → vtable[1] = alloc
	// =========================================================================

	static void* havokHeapAlloc(std::size_t size)
	{
		static REL::Relocation<std::uint32_t*> s_tlsIndex{ REL::Offset(0x5B63B20) };
		LPVOID tlsBlock = TlsGetValue(*s_tlsIndex);
		if (!tlsBlock) return nullptr;

		auto** allocator = reinterpret_cast<void***>(
			reinterpret_cast<char*>(tlsBlock) + 0x58);
		if (!allocator || !*allocator) return nullptr;

		auto* vtable = reinterpret_cast<void* (**)(void*, std::size_t)>(**allocator);
		return vtable[1](*allocator, size);
	}

	/// Free memory via Havok's TLS-based heap allocator (vtable[2] = deallocate).
	/// Ghidra-verified (2026-03-22): dealloc takes 3 args (allocator, ptr, size).
	/// Every Havok destructor in the binary passes size as the 3rd arg.
	static void havokHeapFree(void* ptr, std::size_t size)
	{
		if (!ptr) return;

		static REL::Relocation<std::uint32_t*> s_tlsIndex{ REL::Offset(0x5B63B20) };
		LPVOID tlsBlock = TlsGetValue(*s_tlsIndex);
		if (!tlsBlock) return;

		auto** allocator = reinterpret_cast<void***>(
			reinterpret_cast<char*>(tlsBlock) + 0x58);
		if (!allocator || !*allocator) return;

		// vtable[2] = deallocate(allocator, ptr, size)
		auto* vtable = reinterpret_cast<void (**)(void*, void*, std::size_t)>(**allocator);
		vtable[2](*allocator, ptr, size);
	}

	// =========================================================================
	// Motor creation
	// =========================================================================

	HkPositionMotor* createPositionMotor(float tau, float damping,
		float proportionalRecoveryVelocity, float constantRecoveryVelocity,
		float minForce, float maxForce)
	{
		auto* motor = static_cast<HkPositionMotor*>(havokHeapAlloc(0x30));
		if (!motor) {
			ROCK_LOG_ERROR(GrabConstraint, "Failed to allocate position motor");
			return nullptr;
		}

		auto baseAddr = REL::Module::get().base();
		motor->vtable = reinterpret_cast<void*>(baseAddr + (MOTOR_VTABLE_POSITION - 0x140000000));
		motor->memSizeAndFlags = 0x0001;
		motor->referenceCount = -1;  // 0xFFFF = immortal (don't ref-count)
		motor->pad0C = 0;
		motor->type = 1;  // Position motor (TYPE 1, NOT 2 — type 2 is velocity motor which ignores damping field)
		std::memset(motor->pad11, 0, sizeof(motor->pad11));
		motor->minForce = minForce;
		motor->maxForce = maxForce;
		motor->tau = tau;
		motor->damping = damping;
		// CRITICAL: hkCalcMotorData (0x141afd6d9) checks the FIRST BYTE of motor+0x28
		// (proportionalRecoveryVelocity) as a boolean. If byte==0x00, the motor uses
		// constant velocity mode (motor->damping) instead of position-error-proportional
		// mode. IEEE 754 floats with zero LSB include 0.5, 1.0, 2.0, 4.0 — all BROKEN.
		// Safe values: 0.1, 0.3, 0.9, 1.1, 2.1, 4.1 (any float with non-zero LSB).
		// Ghidra-verified 2026-03-22 by 5-agent investigation.
		motor->proportionalRecoveryVelocity = proportionalRecoveryVelocity;
		motor->constantRecoveryVelocity = constantRecoveryVelocity;

		ROCK_LOG_INFO(GrabConstraint, "Motor created: tau={:.3f} damping={:.2f} "
			"force=[{:.0f},{:.0f}] propRecov={:.1f} constRecov={:.1f}",
			tau, damping, minForce, maxForce,
			proportionalRecoveryVelocity, constantRecoveryVelocity);

		return motor;
	}

	// =========================================================================
	// getConstraintInfoUtil — Ghidra: 0x141A4AD20
	// Static utility that walks an atom chain and computes ConstraintInfo
	// (schema sizes, solver results, temps) automatically.
	// Found by decompiling 4 constraint types' getConstraintInfo stubs —
	// all tail-call to this same address with (atoms, size, infoOut).
	// This is the SAME function HIGGS calls at SkyrimVR 0xACC490.
	// =========================================================================
	using GetConstraintInfoUtil_t = void(const void*, int, void*);
	static REL::Relocation<GetConstraintInfoUtil_t> s_getConstraintInfoUtil{ REL::Offset(0x1A4AD20) };

	// =========================================================================
	// Custom vtable — built once, shared by all grab constraints
	// =========================================================================

	// =========================================================================
	// C++ callback functions for vtable wrappers (called from assembly)
	// These allow logging from vtable overrides without complex asm.
	// =========================================================================

	// getRuntimeInfo — ALWAYS writes runtime values (ignores wantRuntime).
	// FO4VR's hknp calls with wantRuntime=0 (false), but we MUST provide
	// a runtime for our motors to store state. Writing values even when
	// wantRuntime=false forces the engine to allocate the runtime buffer.
	// Signature: (this=RCX, wantRuntime=EDX, infoOut=R8)
	// NOTE on calling convention: On Windows x64, __cdecl/__thiscall/__fastcall
	// are all identical (Microsoft x64 ABI). param1=RCX, param2=RDX, param3=R8.
	// So __cdecl is correct here — the compiler generates the same code regardless.
	// wantRuntime arrives in EDX (param2), infoOut in R8 (param3). We ignore
	// wantRuntime and always write, which is intentional (see comment above).
	static void __cdecl getRuntimeInfoCallback(void* thisPtr, void* param2, void* param3)
	{
		// param3 = R8 = infoOut (confirmed by logging: param2=0x0=wantRuntime, param3=valid ptr)
		if (param3) {
			auto* info = reinterpret_cast<std::int32_t*>(param3);
			info[0] = RUNTIME_REPORTED_SIZE;   // m_sizeOfExternalRuntime = 0x100 (12 results + motor state)
			info[1] = RUNTIME_SOLVER_RESULTS;  // m_numSolverResults = 12 (6 angular + 6 linear)
		}
	}

	// getConstraintInfo — calls getConstraintInfoUtil with our atom chain.
	// Equivalent to the 17-byte tail call but as a C++ function.
	// NOTE: __cdecl == __thiscall on Windows x64 (Microsoft x64 ABI).
	static void __cdecl getConstraintInfoCallback(void* thisPtr, void* infoOut)
	{
		auto* atoms = static_cast<char*>(thisPtr) + ATOMS_START;
		s_getConstraintInfoUtil(atoms, ATOMS_SIZE, infoOut);
	}

	// addInstance callback
	static void __cdecl addInstanceCallback(void* thisPtr, void* constraint, void* runtime, int sizeOfRuntime)
	{
		ROCK_LOG_INFO(GrabConstraint, "addInstance CALLED: this={:p} constraint={:p} runtime={:p} size={}",
			thisPtr, constraint, runtime, sizeOfRuntime);
		if (runtime) {
			std::memset(runtime, 0, sizeOfRuntime);
			ROCK_LOG_INFO(GrabConstraint, "  → memset runtime OK");
		}
	}

	static void* s_customVtable[64] = { nullptr };
	static bool s_vtableBuilt = false;
	// Track VirtualAlloc'd shellcode blocks for cleanup on DLL unload.
	// Without tracking, each hot-reload leaks executable memory pages.
	static constexpr int MAX_SHELLCODE_BLOCKS = 4;
	static void* s_shellcodeBlocks[MAX_SHELLCODE_BLOCKS] = { nullptr };
	static int s_shellcodeCount = 0;

	static void buildCustomVtable()
	{
		if (s_vtableBuilt) return;

		// Copy all slots from the ragdoll vtable as a base
		auto baseAddr = REL::Module::get().base();
		auto* ragdollVtable = reinterpret_cast<void**>(baseAddr + (RAGDOLL_VTABLE - 0x140000000));
		for (int i = 0; i < 64; i++) {
			s_customVtable[i] = ragdollVtable[i];
		}

		// Helper: allocate shellcode and register for cleanup on DLL unload.
		auto allocShellcode = [](std::size_t size) -> std::uint8_t* {
			auto* code = static_cast<std::uint8_t*>(
				VirtualAlloc(nullptr, size, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
			if (code && s_shellcodeCount < MAX_SHELLCODE_BLOCKS) {
				s_shellcodeBlocks[s_shellcodeCount++] = code;
			}
			return code;
		};

		// --- Slot 4 (+0x20): getType ---
		// Return CONSTRAINT_TYPE_CUSTOM (100) instead of TYPE_RAGDOLL.
		// WHY: The engine's init may use type-specific code paths for ragdoll
		// constraints that bypass our custom getRuntimeInfo. Returning CUSTOM
		// forces the generic path. HIGGS does the same.
		// Params: RCX = this. Returns int in EAX.
		{
			auto* code = allocShellcode(16);
			if (code) {
				int off = 0;
				// mov eax, 100  (CONSTRAINT_TYPE_CUSTOM)
				code[off++] = 0xB8;
				*reinterpret_cast<std::int32_t*>(code + off) = 100; off += 4;
				// ret
				code[off++] = 0xC3;
				s_customVtable[VTABLE_SLOT_GET_TYPE] = code;
			}
		}

		// --- Slot 6 (+0x30): isValid ---
		// Always return true. The ragdoll's isValid checks its specific atom layout
		// (limits, ball socket) which we don't have. HIGGS returns true always.
		// Params: RCX = this. Returns bool in AL.
		{
			auto* code = allocShellcode(16);
			if (code) {
				int off = 0;
				// mov al, 1
				code[off++] = 0xB0; code[off++] = 0x01;
				// ret
				code[off++] = 0xC3;
				s_customVtable[VTABLE_SLOT_IS_VALID] = code;
			}
		}

		// --- Slot 5 (+0x28): getConstraintInfo ---
		// Uses C++ callback for logging. Params: RCX=this, RDX=infoOut.
		// The callback calls getConstraintInfoUtil and logs the result.
		s_customVtable[VTABLE_SLOT_GET_CONSTRAINT_INFO] =
			reinterpret_cast<void*>(&getConstraintInfoCallback);

		// --- Slot 18 (+0x90): getRuntimeInfo ---
		// Uses C++ callback for logging. Logs all params to determine which
		// register contains the RuntimeInfo output.
		s_customVtable[VTABLE_SLOT_GET_RUNTIME_INFO] =
			reinterpret_cast<void*>(&getRuntimeInfoCallback);

		// --- Slot 20 (+0xA0): addInstance ---
		// Uses C++ callback for logging + memset.
		s_customVtable[VTABLE_SLOT_ADD_INSTANCE] =
			reinterpret_cast<void*>(&addInstanceCallback);

		// --- Slot 15 (+0x78): setSolvingMethod ---
		// The ragdoll's override writes to ragdoll-specific atom offsets that don't
		// exist in our layout → corruption. Override with no-op.
		// Params: RCX=this, EDX=method. Returns void.
		{
			auto* code = allocShellcode(16);
			if (code) {
				code[0] = 0xC3;  // ret (no-op)
				s_customVtable[15] = code;  // slot 15 = setSolvingMethod
			}
		}

		s_vtableBuilt = true;
		ROCK_LOG_INFO(GrabConstraint, "Custom vtable built at {:p} (6 overrides: getType/isValid/getConstraintInfo/setSolvingMethod/getRuntimeInfo/addInstance)",
			(void*)s_customVtable);
	}

	// =========================================================================
	// Constraint creation — clean custom constraint (HIGGS architecture)
	// =========================================================================

	ActiveConstraint createGrabConstraint(
		RE::hknpWorld* world,
		RE::hknpBodyId handBodyId, RE::hknpBodyId objectBodyId,
		const float* palmWorldHk, const float* grabWorldHk,
		const RE::NiTransform& grabHandSpace,
		float tau, float damping, float maxForce,
		float proportionalRecovery, float constantRecovery)
	{
		ActiveConstraint result;
		if (!world) {
			ROCK_LOG_ERROR(GrabConstraint, "createGrabConstraint: world is null");
			return result;
		}

		// --- Build vtable on first call ---
		buildCustomVtable();

		// --- Step 1: Allocate constraint data ---
		// 0x168 bytes: 0x20 header + 0x148 atoms (6 atoms, no ball socket/limits/friction)
		void* cd = havokHeapAlloc(GRAB_CONSTRAINT_SIZE);
		if (!cd) {
			ROCK_LOG_ERROR(GrabConstraint, "Failed to allocate constraint data ({:#x} bytes)", GRAB_CONSTRAINT_SIZE);
			return result;
		}
		std::memset(cd, 0, GRAB_CONSTRAINT_SIZE);

		// --- Step 2: Initialize header (0x20 bytes) ---
		// +0x00: vtable → our custom vtable
		// +0x08: memSizeAndFlags(2) + referenceCount(2) + pad(4) = hkReferencedObject
		// +0x10: userData (8 bytes) = 0
		// +0x18: alignment padding (8 bytes) = 0
		// +0x20: atoms start
		// Header size confirmed by Ghidra: ragdoll getConstraintInfo does ADD RCX, 0x20.
		*reinterpret_cast<void**>(cd) = s_customVtable;
		auto* header = static_cast<char*>(cd);
		*reinterpret_cast<std::uint16_t*>(header + 0x08) = 0x0001;  // memSizeAndFlags
		// I1 FIX: Set referenceCount to -1 (0xFFFF = immortal). This prevents
		// DestroyConstraints from decrementing to zero and double-freeing the data.
		// We manage the constraint data lifetime ourselves in destroyGrabConstraint.
		*reinterpret_cast<std::int16_t*>(header + 0x0A) = -1;       // referenceCount = immortal

		// --- Step 3: Initialize atom type fields ---
		// Each atom starts with uint16 type. The solver dispatches on these.
		*reinterpret_cast<std::uint16_t*>(header + ATOM_TRANSFORMS) = ATOM_TYPE_SET_LOCAL_TRANSFORMS;
		*reinterpret_cast<std::uint16_t*>(header + ATOM_STABILIZE) = ATOM_TYPE_SETUP_STABILIZATION;
		*reinterpret_cast<std::uint16_t*>(header + ATOM_RAGDOLL_MOT) = ATOM_TYPE_RAGDOLL_MOTOR;
		*reinterpret_cast<std::uint16_t*>(header + ATOM_LIN_MOTOR_0) = ATOM_TYPE_LIN_MOTOR;
		*reinterpret_cast<std::uint16_t*>(header + ATOM_LIN_MOTOR_1) = ATOM_TYPE_LIN_MOTOR;
		*reinterpret_cast<std::uint16_t*>(header + ATOM_LIN_MOTOR_2) = ATOM_TYPE_LIN_MOTOR;

		// --- Step 4: Initialize RagdollMotor atom ---
		// SDK standard layout (confirmed: atoms at +0x20 means SDK offsets apply):
		//   atom+0x02: m_isEnabled (uint8)
		//   atom+0x04: m_initializedOffset (int16) — from Runtime base to m_initialized
		//   atom+0x06: m_previousTargetAnglesOffset (int16)
		//   atom+0x10: m_target_bRca (hkMatrix3, 3×16 = 48 bytes, 16-byte aligned)
		//   atom+0x40: m_motors[3] (3 × 8-byte pointers)
		{
			auto* ragAtom = header + ATOM_RAGDOLL_MOT;
			*(ragAtom + 0x02) = 0;  // m_isEnabled = false (set true after motors)

			// Runtime offsets: RELATIVE to RagdollMotor's runtime ptr (at 0x00).
			// hknp advances ptr per atom, so these reach past ALL solver results (0x60)
			// to motor state area. Verified via FO4VR ragdoll initOffset=0x90 pattern.
			*reinterpret_cast<std::int16_t*>(ragAtom + 0x04) =
				static_cast<std::int16_t>(RT_RAGDOLL_INIT_OFFSET);      // 0x60
			*reinterpret_cast<std::int16_t*>(ragAtom + 0x06) =
				static_cast<std::int16_t>(RT_RAGDOLL_PREV_ANG_OFFSET);  // 0x64

			// m_target_bRca = identity matrix (3 column vectors, 16-byte aligned at +0x10)
			auto* target = reinterpret_cast<float*>(ragAtom + 0x10);
			target[0] = 1.0f; target[1] = 0.0f; target[2] = 0.0f; target[3] = 0.0f;  // col 0
			target[4] = 0.0f; target[5] = 1.0f; target[6] = 0.0f; target[7] = 0.0f;  // col 1
			target[8] = 0.0f; target[9] = 0.0f; target[10] = 1.0f; target[11] = 0.0f; // col 2
		}

		// --- Step 5: Initialize 3 linear motor atoms ---
		// Layout per atom (0x18 bytes, from SDK + Ghidra verification):
		//   +0x00: m_type (uint16) = 11
		//   +0x02: m_isEnabled (uint8) = false initially
		//   +0x03: m_motorAxis (uint8) = 0/1/2
		//   +0x04: m_initializedOffset (int16) — RELATIVE to this atom's runtime ptr
		//   +0x06: m_previousTargetPositionOffset (int16) — RELATIVE to this atom's runtime ptr
		//   +0x08: m_targetPosition (float) = 0.0 (drive to pivot)
		//   +0x0C: pad (4 bytes)
		//   +0x10: m_motor (ptr) — set in Step 6
		//
		// CRITICAL: hknp advances m_constraintRuntime per atom:
		//   RagdollMotor: advances by 0x30 (6 results × 8)
		//   LinMotor: advances by 0x10 (2 results × 8)
		// So each LinMotor's runtime ptr is at a different absolute position:
		//   LinMotor0: ptr = 0x30 (after RagdollMotor)
		//   LinMotor1: ptr = 0x40 (after LinMotor0)
		//   LinMotor2: ptr = 0x50 (after LinMotor1)
		// Offsets are RELATIVE to ptr, pointing into motor state area at 0x60+.
		// Verified in GHIDRA_CONFIRMED_ADDRESSES.md and Ghidra disassembly.
		{
			const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };

			// Per-atom relative offsets (from GHIDRA_CONFIRMED_ADDRESSES.md)
			// m_initializedLinear[3] at absolute 0x70, m_previousTargetPositions[3] at absolute 0x74
			// Each axis indexes into those arrays: init[axis] = byte, prevTarget[axis] = float
			const std::int16_t initOffsets[3]       = { 0x40, 0x31, 0x22 };  // 0x70-0x30, 0x71-0x40, 0x72-0x50
			const std::int16_t prevTargetOffsets[3]  = { 0x44, 0x38, 0x2C };  // 0x74-0x30, 0x78-0x40, 0x7C-0x50

			for (int axis = 0; axis < 3; axis++) {
				auto* atom = header + linAtomOffsets[axis];
				// type already set in Step 3
				*(atom + 0x02) = 0;  // m_isEnabled = false
				*(atom + 0x03) = static_cast<char>(axis);  // m_motorAxis

				*reinterpret_cast<std::int16_t*>(atom + 0x04) = initOffsets[axis];
				*reinterpret_cast<std::int16_t*>(atom + 0x06) = prevTargetOffsets[axis];

				*reinterpret_cast<float*>(atom + 0x08) = 0.0f;  // m_targetPosition
				// +0x0C pad already zeroed
				// +0x10 motor pointer set in Step 6
			}
		}

		ROCK_LOG_INFO(GrabConstraint, "Custom constraint data at {:p} ({:#x} bytes, 6 atoms, {} solver results, runtime {:#x})",
			cd, GRAB_CONSTRAINT_SIZE, RUNTIME_SOLVER_RESULTS, RUNTIME_REPORTED_SIZE);
		ROCK_LOG_INFO(GrabConstraint, "  RagdollMotor offsets: init={:#x} prevAng={:#x} (relative to ptr at 0x00)",
			RT_RAGDOLL_INIT_OFFSET, RT_RAGDOLL_PREV_ANG_OFFSET);
		ROCK_LOG_INFO(GrabConstraint, "  LinMotor offsets: [0x40,0x44] [0x31,0x38] [0x22,0x2C] (relative to per-atom ptrs at 0x30,0x40,0x50)");

		// --- Step 6: Set body-local reference frames (HIGGS pattern) ---
		// WHY NOT setInWorldSpace: It uses a twist axis to build arbitrary constraint
		// frames that don't align with the bodies. This causes LinMotor axes to push
		// in wrong directions (orbiting instead of convergence). 5-agent investigation
		// confirmed: HIGGS uses setInBodySpace with identity rotation for transformA
		// and R_obj^T * R_hand for transformB. We write directly to the atom.
		//
		// SetLocalTransforms atom layout (0x90 bytes at constraintData+0x20):
		//   +0x10: transformA.rotation col0 (16 bytes) → constraintData+0x30
		//   +0x20: transformA.rotation col1 (16 bytes) → constraintData+0x40
		//   +0x30: transformA.rotation col2 (16 bytes) → constraintData+0x50
		//   +0x40: transformA.translation   (16 bytes) → constraintData+0x60
		//   +0x50: transformB.rotation col0 (16 bytes) → constraintData+0x70
		//   +0x60: transformB.rotation col1 (16 bytes) → constraintData+0x80
		//   +0x70: transformB.rotation col2 (16 bytes) → constraintData+0x90
		//   +0x80: transformB.translation   (16 bytes) → constraintData+0xA0
		{
			auto* bodyArray = world->GetBodyArray();
			auto* handBody = reinterpret_cast<const float*>(&bodyArray[handBodyId.value]);
			auto* objBody = reinterpret_cast<const float*>(&bodyArray[objectBodyId.value]);

			// --- TransformA: IDENTITY rotation + pivotA ---
			// HIGGS uses identity rotation for the hand body's constraint frame.
			// This means LinMotor/AngMotor axes align with the hand body's own axes.
			auto* tA_col0 = reinterpret_cast<float*>(header + 0x30);
			auto* tA_col1 = reinterpret_cast<float*>(header + 0x40);
			auto* tA_col2 = reinterpret_cast<float*>(header + 0x50);
			auto* tA_pos  = reinterpret_cast<float*>(header + 0x60);

			// Identity rotation columns
			tA_col0[0] = 1.0f; tA_col0[1] = 0.0f; tA_col0[2] = 0.0f; tA_col0[3] = 0.0f;
			tA_col1[0] = 0.0f; tA_col1[1] = 1.0f; tA_col1[2] = 0.0f; tA_col1[3] = 0.0f;
			tA_col2[0] = 0.0f; tA_col2[1] = 0.0f; tA_col2[2] = 1.0f; tA_col2[3] = 0.0f;

			// Phase 0B.1: PivotA = palm center in hand body-local space
			// HIGGS S00 0.1a: pivotA uses palmPos, NOT the grab surface point.
			// Body-local = delta * R^T in Havok's right-multiplication convention.
			// Component j = dot(row_j, delta). Body is row-major (SetBodyTransform
			// copies NiMatrix3 directly, Ghidra-verified). See HAVOK_CONVENTIONS.md.
			float dxA = palmWorldHk[0] - handBody[12];
			float dyA = palmWorldHk[1] - handBody[13];
			float dzA = palmWorldHk[2] - handBody[14];
			tA_pos[0] = handBody[0]*dxA + handBody[1]*dyA + handBody[2]*dzA;
			tA_pos[1] = handBody[4]*dxA + handBody[5]*dyA + handBody[6]*dzA;
			tA_pos[2] = handBody[8]*dxA + handBody[9]*dyA + handBody[10]*dzA;
			tA_pos[3] = 0.0f;

			// --- Phase 0B.2: TransformB rotation = Inverse(grabHandSpace).rotation ---
			// HIGGS S00 0.6b: transformB.rotation = Inverse(desiredNodeTransformHandSpace
			//   * GetRigidBodyTLocalTransform()).rotation
			// PRE-1 result: GetRigidBodyTLocalTransform = identity in FO4VR
			// Simplified: transformB.rotation = Inverse(grabHandSpace).rotation
			// = Transpose(grabHandSpace.rotation) (rotation inverse = transpose for orthonormal)
			auto* tB_col0 = reinterpret_cast<float*>(header + 0x70);
			auto* tB_col1 = reinterpret_cast<float*>(header + 0x80);
			auto* tB_col2 = reinterpret_cast<float*>(header + 0x90);
			auto* tB_pos  = reinterpret_cast<float*>(header + 0xA0);

			{
				// Inverse(grabHandSpace).rotation = Transpose(grabHandSpace.rotate)
				// NiMatrix3 is row-major: entry[row][col]. Havok hkMatrix3 is column-major.
				// Column-major col_j[i] represents matrix element [i][j].
				// We want R^T: element [i][j] = R[j][i] = R.entry[j][i].
				// So col_j[i] = R.entry[j][i], i.e. col_j = row j of R.
				//
				// The per-frame update in HandGrab.cpp computes invRot = Transpose(R) first,
				// then writes columns of invRot (col_j[i] = invRot[i][j] = R[j][i]).
				// Both produce the same Havok matrix — just different intermediate steps.
				const auto& R = grabHandSpace.rotate;
				tB_col0[0] = R.entry[0][0]; tB_col0[1] = R.entry[0][1]; tB_col0[2] = R.entry[0][2]; tB_col0[3] = 0.0f;
				tB_col1[0] = R.entry[1][0]; tB_col1[1] = R.entry[1][1]; tB_col1[2] = R.entry[1][2]; tB_col1[3] = 0.0f;
				tB_col2[0] = R.entry[2][0]; tB_col2[1] = R.entry[2][1]; tB_col2[2] = R.entry[2][2]; tB_col2[3] = 0.0f;
			}

			// Phase 0B.1: PivotB = grab surface point in object body-local space
			// HIGGS S00 0.1b: pivotB uses ptPos (touch point on object surface)
			// Body-local via right-multiplication: dot(row_j, delta). Same as pivotA.
			float dxB = grabWorldHk[0] - objBody[12];
			float dyB = grabWorldHk[1] - objBody[13];
			float dzB = grabWorldHk[2] - objBody[14];
			tB_pos[0] = objBody[0]*dxB + objBody[1]*dyB + objBody[2]*dzB;
			tB_pos[1] = objBody[4]*dxB + objBody[5]*dyB + objBody[6]*dzB;
			tB_pos[2] = objBody[8]*dxB + objBody[9]*dyB + objBody[10]*dzB;
			tB_pos[3] = 0.0f;

			ROCK_LOG_INFO(GrabConstraint, "setInBodySpace: pivotA=({:.3f},{:.3f},{:.3f}) [palm] "
				"pivotB=({:.3f},{:.3f},{:.3f}) [surface] "
				"tB_col0=({:.3f},{:.3f},{:.3f})",
				tA_pos[0], tA_pos[1], tA_pos[2],
				tB_pos[0], tB_pos[1], tB_pos[2],
				tB_col0[0], tB_col0[1], tB_col0[2]);
		}

		// --- Step 6b: Set angular motor target from transformB rotation ---
		// Phase 0B.3: m_target_bRca = transformB.rotation (= Inverse(grabHandSpace).rotation)
		// HIGGS S05 5.3: setTargetRelativeOrientationOfBodies(desiredHandTransformHavokObjSpace.rot)
		// Since transformA.rotation = identity: target = bRa × identity = bRa = transformB.rotation
		{
			auto* transformB_rot = header + 0x70;
			auto* target_bRca = header + ATOM_RAGDOLL_MOT + 0x10;
			std::memcpy(target_bRca, transformB_rot, 48);  // 3 columns × 16 bytes

			auto* t = reinterpret_cast<float*>(target_bRca);
			ROCK_LOG_INFO(GrabConstraint, "target_bRca: col0=[{:.3f},{:.3f},{:.3f}] col1=[{:.3f},{:.3f},{:.3f}]",
				t[0], t[1], t[2], t[4], t[5], t[6]);
		}

		// --- Step 7: Create motors and attach (DISABLED — enabled after CreateConstraint) ---
		// HIGGS pattern: motors start DISABLED, constraint is created, THEN motors enabled.
		// This lets the constraint be validated before motors kick in.
		//
		// Phase 0B: HIGGS angular:linear force ratio = 12.5:1 (was 17:1).
		// HIGGS S04 4.4: grabConstraintAngularToLinearForceRatio = 12.5
		// C1 FIX: Angular motor uses angular-specific config, not the linear params
		// passed through the function signature. This ensures correct params from
		// the very first frame (before per-frame update overwrites them).
		float angularForceRatio = g_rockConfig.rockGrabAngularToLinearForceRatio;

		auto* angMotor = createPositionMotor(
			g_rockConfig.rockGrabAngularTau, g_rockConfig.rockGrabAngularDamping,
			g_rockConfig.rockGrabAngularProportionalRecovery, g_rockConfig.rockGrabAngularConstantRecovery,
			-maxForce / angularForceRatio, maxForce / angularForceRatio);

		auto* linMotor = createPositionMotor(tau, damping,
			proportionalRecovery, constantRecovery,
			-maxForce, maxForce);

		if (!angMotor || !linMotor) {
			ROCK_LOG_ERROR(GrabConstraint, "Motor creation failed — aborting");
			if (angMotor) havokHeapFree(angMotor, 0x30);
			if (linMotor) havokHeapFree(linMotor, 0x30);
			havokHeapFree(cd, GRAB_CONSTRAINT_SIZE);
			return result;
		}

		// Attach motors but leave DISABLED (isEnabled stays 0 from Step 4/5)
		{
			auto* ragAtom = header + ATOM_RAGDOLL_MOT;
			for (int i = 0; i < 3; i++) {
				*reinterpret_cast<void**>(ragAtom + 0x40 + i * 8) = angMotor;
			}
			// isEnabled stays false — enabled in Step 9
		}
		{
			const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };
			for (int axis = 0; axis < 3; axis++) {
				*reinterpret_cast<void**>(header + linAtomOffsets[axis] + 0x10) = linMotor;
				// isEnabled stays false — enabled in Step 9
			}
		}

		ROCK_LOG_INFO(GrabConstraint, "Motors attached (disabled): angular={:.0f}, linear={:.0f}",
			maxForce / angularForceRatio, maxForce);

		// --- Step 8: Create constraint in world ---
		// CRITICAL: Zero-initialize cinfo to clear flags byte at +0x10.
		// hknpConstraint::init reads cinfo.flags and preserves bit 2 (0x04 = disable flag)
		// via `flags & 0xFD`. If bit 2 is set from uninitialized stack memory,
		// createConstraint skips handleConstraintActivationMode entirely →
		// constraint exists but solver NEVER processes it → zero motor effect.
		// Ghidra-verified: 0x1417e39c0 (init) and 0x1415469b0 (createConstraint).
		RE::hknpConstraintCinfo cinfo{};  // Zero-init — flags = 0, bit 2 clear
		cinfo.constraintData = reinterpret_cast<RE::hkpConstraintData*>(cd);
		cinfo.bodyIdA = handBodyId.value;
		cinfo.bodyIdB = objectBodyId.value;

		std::uint32_t outId = 0x7FFF'FFFF;
		world->CreateConstraint(&outId, cinfo);

		if (outId == 0x7FFF'FFFF) {
			ROCK_LOG_ERROR(GrabConstraint, "CreateConstraint returned invalid ID");
			return result;
		}

		// --- Step 9: Enable ALL motors AFTER constraint is in the world ---
		// Both angular (RagdollMotor) and linear (LinMotor) enabled.
		// Constraint targets (pivotA/B, angular target) are FIXED from grab time.
		// Motor error comes from bodyA (hand) moving while bodyB (object) lags.
		{
			auto* ragAtom = header + ATOM_RAGDOLL_MOT;
			*(ragAtom + 0x02) = 1;  // ragdoll motors ENABLED

			const int linAtomOffsets[3] = { ATOM_LIN_MOTOR_0, ATOM_LIN_MOTOR_1, ATOM_LIN_MOTOR_2 };
			for (int axis = 0; axis < 3; axis++) {
				*(header + linAtomOffsets[axis] + 0x02) = 1;  // linear motor enabled
			}
		}

		ROCK_LOG_INFO(GrabConstraint, "Constraint created: id={}, hand={}, obj={}",
			outId, handBodyId.value, objectBodyId.value);

		result.constraintId = outId;
		result.constraintData = cd;
		result.angularMotor = angMotor;
		result.linearMotor = linMotor;
		result.currentTau = tau;
		result.currentMaxForce = 0.0f;
		result.targetMaxForce = maxForce;

		return result;
	}

	// =========================================================================
	// Constraint destruction
	// =========================================================================

	void destroyGrabConstraint(RE::hknpWorld* world, ActiveConstraint& constraint)
	{
		if (!constraint.isValid()) return;

		if (world) {
			std::uint32_t ids[1] = { constraint.constraintId };
			world->DestroyConstraints(ids, 1);
			ROCK_LOG_INFO(GrabConstraint, "Constraint {} destroyed", constraint.constraintId);
		}

		// Free Havok-heap-allocated motors and constraint data.
		// Motors are shared (same instance for all 3 axes), so free once each.
		// Ghidra: dealloc takes 3 args (allocator, ptr, size).
		if (constraint.angularMotor) {
			havokHeapFree(constraint.angularMotor, 0x30);  // sizeof(HkPositionMotor)
		}
		if (constraint.linearMotor && constraint.linearMotor != constraint.angularMotor) {
			havokHeapFree(constraint.linearMotor, 0x30);
		}
		if (constraint.constraintData) {
			havokHeapFree(constraint.constraintData, GRAB_CONSTRAINT_SIZE);  // 0x168
		}

		constraint.clear();
	}

	// =========================================================================
	// Inertia normalization (HIGGS pattern)
	//
	// WHY: Long objects (weapons, poles) have very different moments of inertia
	// on different axes. A rifle has low inertia around its barrel axis but high
	// inertia around the perpendicular axes. Without normalization, angular
	// motors can't resist rotation on the low-inertia axis → cartwheeling.
	//
	// GHIDRA DISCOVERY (2026-03-22):
	// hknpMotion stores inverse inertia as PACKED INT16 at motion+0x20, NOT as
	// floats at motion+0x60 (which is Skyrim's hkpMotion offset / velocity area).
	// 8 bytes at motion+0x20 contain 4 packed int16 via packssdw:
	//   int16[0] = invInertiaX, [1] = invInertiaY, [2] = invInertiaZ, [3] = invMass
	// Unpacking: float = (ushort)packed * 65536.0f
	// Verified in: bhkNPCollisionObject::SetMass (0x141e08c00),
	//              hknpMotion::setPointVelocity (0x1417d1a20),
	//              hknpMotion::buildEffMassMatrixAt (0x1417d1ea0),
	//              hknpMotion::setFromMassProperties (0x1417d13a0),
	//              hknpMotionManager::initializeMotion (0x1417e1e40)
	//
	// The ratio of packed shorts equals the ratio of physical inverse inertia.
	// So normalization works directly on the packed int16 values.
	//
	// HIGGS normalizes by:
	//   1. Clamping the ratio between max and min inverse inertia to 10x
	//   2. Enforcing a minimum inertia (via max inverse inertia cap)
	//   (grabbedObjectMaxInertiaRatio=10, grabbedObjectMinInertia=0.01)
	// =========================================================================

	void normalizeGrabbedInertia(RE::hknpWorld* world, RE::hknpBodyId bodyId, SavedObjectState& savedState)
	{
		if (!world) return;

		// Get body's motionIndex
		auto* bodyArray = world->GetBodyArray();
		auto& body = bodyArray[bodyId.value];
		auto motionIndex = body.motionIndex;
		if (motionIndex == 0 || motionIndex > 4096) return;  // static or invalid

		// Motion array at world+0xE0, stride 0x80 bytes
		auto* worldBytes = reinterpret_cast<char*>(world);
		auto* motionArrayPtr = *reinterpret_cast<char**>(worldBytes + 0xE0);
		if (!motionArrayPtr) return;

		auto* motion = motionArrayPtr + motionIndex * 0x80;

		// Packed inverse inertia: 4 int16 values at motion+0x20
		// Layout (from packssdw): [invInertiaX, invInertiaY, invInertiaZ, invMass]
		auto* packed = reinterpret_cast<std::int16_t*>(motion + MOTION_PACKED_INERTIA_OFFSET);

		// Save original packed inertia values (3 axes only — mass not modified)
		savedState.savedPackedInertia[0] = packed[0];  // invInertiaX
		savedState.savedPackedInertia[1] = packed[1];  // invInertiaY
		savedState.savedPackedInertia[2] = packed[2];  // invInertiaZ

		ROCK_LOG_INFO(GrabConstraint, "Inertia pre-normalize: body={} packed=[{},{},{}] mass={}",
			bodyId.value, packed[0], packed[1], packed[2], packed[3]);

		// Skip if any axis has zero or negative inertia (fixed/keyframed body)
		if (packed[0] <= 0 || packed[1] <= 0 || packed[2] <= 0) {
			ROCK_LOG_WARN(GrabConstraint, "Skipping inertia normalization: zero/negative packed value");
			return;
		}

		// Find min and max of the 3 inertia components
		// Higher packed value = higher inverse inertia = easier to spin on that axis
		std::int16_t minI = (std::min)({packed[0], packed[1], packed[2]});
		std::int16_t maxI = (std::max)({packed[0], packed[1], packed[2]});

		int MAX_INERTIA_RATIO = static_cast<int>(g_rockConfig.rockGrabMaxInertiaRatio);

		// I4 FIX: Compute in int32 to prevent overflow. minI * 10 can exceed
		// int16_t range (max 32767) if minI > 3276. Clamp result to int16 max.
		std::int32_t maxAllowedI32 = static_cast<std::int32_t>(minI) * MAX_INERTIA_RATIO;
		if (maxI > maxAllowedI32) {
			// Clamp: largest inverse inertia should be at most 10x the smallest
			constexpr std::int32_t kInt16Max = 32767;
			std::int16_t maxAllowed = static_cast<std::int16_t>(
				(std::min)(maxAllowedI32, kInt16Max));
			for (int i = 0; i < 3; i++) {
				if (packed[i] > maxAllowed) {
					packed[i] = maxAllowed;
				}
			}

			ROCK_LOG_INFO(GrabConstraint, "Inertia ratio clamped: [{},{},{}] → [{},{},{}] (max {}x min)",
				savedState.savedPackedInertia[0], savedState.savedPackedInertia[1], savedState.savedPackedInertia[2],
				packed[0], packed[1], packed[2], MAX_INERTIA_RATIO);
		}
		else {
			ROCK_LOG_INFO(GrabConstraint, "Inertia ratio OK ({:.1f}x), no clamping needed",
				static_cast<float>(maxI) / static_cast<float>(minI));
		}

		// invMass (packed[3]) is NOT modified — preserve original mass
		savedState.inertiaModified = true;
	}

	void restoreGrabbedInertia(RE::hknpWorld* world, SavedObjectState& savedState)
	{
		if (!world || !savedState.inertiaModified) return;

		auto* bodyArray = world->GetBodyArray();
		auto& body = bodyArray[savedState.bodyId.value];
		auto motionIndex = body.motionIndex;
		if (motionIndex == 0 || motionIndex > 4096) return;

		auto* worldBytes = reinterpret_cast<char*>(world);
		auto* motionArrayPtr = *reinterpret_cast<char**>(worldBytes + 0xE0);
		if (!motionArrayPtr) return;

		auto* motion = motionArrayPtr + motionIndex * 0x80;
		auto* packed = reinterpret_cast<std::int16_t*>(motion + MOTION_PACKED_INERTIA_OFFSET);

		// Restore original packed inertia (3 axes only — mass was never modified)
		packed[0] = savedState.savedPackedInertia[0];
		packed[1] = savedState.savedPackedInertia[1];
		packed[2] = savedState.savedPackedInertia[2];

		ROCK_LOG_INFO(GrabConstraint, "Inertia restored: body={} → packed=[{},{},{}]",
			savedState.bodyId.value, packed[0], packed[1], packed[2]);

		savedState.inertiaModified = false;
	}

	void cleanupGrabConstraintVtable()
	{
		for (int i = 0; i < s_shellcodeCount; i++) {
			if (s_shellcodeBlocks[i]) {
				VirtualFree(s_shellcodeBlocks[i], 0, MEM_RELEASE);
				s_shellcodeBlocks[i] = nullptr;
			}
		}
		s_shellcodeCount = 0;
		s_vtableBuilt = false;
		ROCK_LOG_INFO(GrabConstraint, "Vtable shellcode freed");
	}
}

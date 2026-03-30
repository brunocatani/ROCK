#include "PhysicsHooks.h"

#include "PhysicsInteraction.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "api/FRIKApi.h"
#include "RockConfig.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace frik::rock
{
	// =========================================================================
	// HandleBumpedCharacter Hook
	//
	// WHY: The game's character controller pushes the player away from objects
	// (tables, walls). In VR, this prevents reaching across tables to interact
	// with objects. We hook HandleBumpedCharacter and skip the bump when the
	// bumped object is near either hand — allowing you to lean into tables.
	// =========================================================================

	// Trampoline for the original function — set by write_branch return value
	using HandleBumpedCharacter_t = void(*)(void*, void*, void*);
	static HandleBumpedCharacter_t g_originalHandleBumped = nullptr;

	void hookedHandleBumpedCharacter(void* controller, void* bumpedCC, void* contactInfo)
	{
		// Fast path: if hooks are disabled or no instance, skip without touching Havok.
		// This prevents most SEH triggers. The __try remains as a safety net for TOCTOU races
		// where the world dies between this check and the actual Havok access.
		if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
			if (g_originalHandleBumped) {
				g_originalHandleBumped(controller, bumpedCC, contactInfo);
			}
			return;
		}

		// SAFETY: This hook runs on the PHYSICS THREAD. Wrap everything in SEH
		// including the call to the original function (its trampoline may be in
		// non-executable memory due to F4SE trampoline allocation issues).
		__try {
			if (PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire) &&
				frik::api::FRIKApi::inst && frik::api::FRIKApi::inst->isSkeletonReady() && g_rockConfig.rockEnabled) {

				// Skip ALL bumps while holding an object.
				// HandleBumpedCharacter is a SEPARATE code path from the collision filter —
				// it fires on CC proximity regardless of layer. A held rifle's barrel extends
				// well beyond the 50gu hand skip radius, so the old hand-distance check
				// missed it. Clutter didn't push because it stays small and close to the hand.
				// Weapons pushed because their long collision shape contacts the CC far from
				// the palm, the bump fires, and the original handler displaces the player.
				auto* pi = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
				if (pi) {
					if (pi->getRightHand().isHolding() || pi->getLeftHand().isHolding()) {
						static int holdSkipLogCounter = 0;
						if (++holdSkipLogCounter >= 30) {
							holdSkipLogCounter = 0;
							logger::info("[ROCK::Bump] Skipped bump — holding object");
						}
						return;
					}
				}

				// Use wand node positions for bump distance check (consistent with collision body)
				auto* rWandBump = f4vr::getRightHandNode();
				auto* lWandBump = f4vr::getLeftHandNode();
				if (rWandBump && lWandBump) {
					auto* ci = reinterpret_cast<float*>(contactInfo);
					RE::NiPoint3 contactPos(
						ci[0] * rock::kHavokToGameScale,
						ci[1] * rock::kHavokToGameScale,
						ci[2] * rock::kHavokToGameScale);

					const auto rightHand = rWandBump->world.translate;
					const auto leftHand = lWandBump->world.translate;

					auto distR = (contactPos - rightHand).Length();
					auto distL = (contactPos - leftHand).Length();

					constexpr float bumpSkipRadius = 50.0f;
					if (distR < bumpSkipRadius || distL < bumpSkipRadius) {
						static int skipLogCounter = 0;
						if (++skipLogCounter >= 10) {
							skipLogCounter = 0;
							logger::info("[ROCK::Bump] Skipped bump — hand dist R={:.1f} L={:.1f} contact=({:.1f},{:.1f},{:.1f})",
								distR, distL, contactPos.x, contactPos.y, contactPos.z);
						}
						return;
					}
				}
			}

			// Not near hands and not holding — run original bump behavior
			g_originalHandleBumped(controller, bumpedCC, contactInfo);
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			static int sehLogCounter = 0;
			if (sehLogCounter++ % 100 == 0) {
				logger::error("[ROCK::Bump] SEH exception caught on physics thread (count={}) — "
					"trampoline or stale pointer issue", sehLogCounter);
			}
		}
	}

	// =========================================================================
	// Character Controller Radius Scaling
	//
	// WHY: The CC capsule radius is too large for VR — keeps the player ~30cm
	// from tables/walls. We hook UpdateShapes (called every frame) and scale
	// down the radius after it's computed. This survives shape recreation
	// (crouch, power armor, etc.) because we re-apply the scale every frame.
	// =========================================================================

	typedef void (*UpdateShapes_t)(void*);
	static UpdateShapes_t g_originalUpdateShapes = nullptr;

	void hookedUpdateShapes(void* charController)
	{
		// Call original first — it computes capsule dimensions
		g_originalUpdateShapes(charController);

		// Scale down the radius
		if (g_rockConfig.rockEnabled && g_rockConfig.rockCharControllerRadiusScale < 1.0f) {
			auto* cc = reinterpret_cast<std::uint8_t*>(charController);
			auto* radius1 = reinterpret_cast<float*>(cc + 0x58);
			auto* radius2 = reinterpret_cast<float*>(cc + 0x5C);

			*radius1 *= g_rockConfig.rockCharControllerRadiusScale;
			*radius2 *= g_rockConfig.rockCharControllerRadiusScale;
		}
	}

	void installCCRadiusHook()
	{
		// DISABLED: The function at 0x1E1F130 cannot be safely hooked via F4SE trampoline.
		// Both write_branch<5> and write_branch<6> produce trampolines pointing to
		// non-executable memory, causing EXCEPTION_ACCESS_VIOLATION on NPC character
		// controller creation (FillCharacterControllerCInfo → UpdateShapes).
		// The CC radius scaling is a cosmetic feature (lets player get closer to tables).
		// TODO: Revisit with a different hooking strategy (Xbyak patch, direct memory write,
		// or hook a different function in the CC creation pipeline).
		ROCK_LOG_INFO(Init, "CC radius hook DISABLED (trampoline incompatible with target function)");
	}

	void installBumpHook()
	{
		static bool installed = false;
		if (installed) return;
		installed = true;

		// Hook HandleBumpedCharacter at its known address.
		// write_branch<6> overwrites the function entry with a JMP and returns a
		// trampoline thunk that executes the original first bytes + JMP to the rest.
		// We MUST save the original from the return value, not from the raw address.
		static REL::Relocation<std::uintptr_t> target{ REL::Offset(0x1E24980) };
		auto& trampoline = F4SE::GetTrampoline();

		g_originalHandleBumped = reinterpret_cast<HandleBumpedCharacter_t>(
			trampoline.write_branch<6>(target.address(),
				reinterpret_cast<std::uintptr_t>(&hookedHandleBumpedCharacter)));

		ROCK_LOG_INFO(Init, "Installed HandleBumpedCharacter hook at 0x{:X}, original at 0x{:X}",
			target.address(), reinterpret_cast<std::uintptr_t>(g_originalHandleBumped));
	}

	// =========================================================================
	// VR Native Grab Kill — Xbyak direct patch
	//
	// WHY: FO4VR's native grab creates BSMouseSpringAction springs on objects
	// near the VR crosshair. When ROCK holds a KEYFRAMED weapon, the native
	// spring can't move it — the reaction force pushes the PLAYER instead.
	// This was one of two push sources (the other being CC stepping on
	// KEYFRAMED bodies, fixed by processConstraintsCallback hook).
	//
	// FIX: Patch the VR Grab Initiate function (0x140F19250) entry to return
	// immediately. No trampoline needed — just overwrite the first bytes
	// with `xor eax,eax; ret` (return 0). This completely disables the
	// native grab system. ROCK handles all grabbing.
	//
	// Ghidra: 0x140f19250 — VR Grab Initiate
	// =========================================================================

	void installNativeGrabHook()
	{
		static bool installed = false;
		if (installed) return;
		installed = true;

		// Patch VR Grab Initiate to return 0 immediately: xor eax,eax (31 C0) ; ret (C3)
		static REL::Relocation<std::uintptr_t> target{ REL::Offset(0xF19250) };
		auto* addr = reinterpret_cast<std::uint8_t*>(target.address());

		// Change memory protection to allow writing
		DWORD oldProtect;
		if (VirtualProtect(addr, 3, PAGE_EXECUTE_READWRITE, &oldProtect)) {
			addr[0] = 0x31;  // xor eax, eax
			addr[1] = 0xC0;
			addr[2] = 0xC3;  // ret
			VirtualProtect(addr, 3, oldProtect, &oldProtect);
			ROCK_LOG_INFO(Init, "Patched VR Grab Initiate at 0x{:X} — native grab DISABLED (xor eax,eax; ret)",
				target.address());
		} else {
			ROCK_LOG_ERROR(Init, "FAILED to patch VR Grab Initiate at 0x{:X} — VirtualProtect failed",
				target.address());
		}
	}

	// =========================================================================
	// bhkCharProxyController::processConstraintsCallback Hook (HIGGS Layer 3)
	//
	// WHY: The CC's processConstraintsCallback (0x141E4B7E0) iterates each
	// contact constraint and decides how the CC interacts with it. For bodies
	// on WEAPON/CLUTTER layers (4,5,6,29), it checks (flags & 5) == 0:
	//   - DYNAMIC bodies (flags & 5 == 0) → bVar48=true → surface velocity zeroed → NO PUSH
	//   - KEYFRAMED bodies (flags & 5 != 0) → stepping/support code → PUSHES PLAYER
	//
	// Our held objects are KEYFRAMED, so the CC treats them as steppable surfaces.
	// FIX: Hook this function, call original, then zero surface velocity for any
	// constraint whose body matches a held object. This is the HIGGS pattern
	// (PlayerCharacterProxyListener::processConstraintsCallback zeros velocity
	// for moveable non-biped bodies).
	//
	// Ghidra: 0x141E4B7E0 — bhkCharProxyController::processConstraintsCallback
	// Called via listener dispatch at FUN_141b0e9c0: vtable+0x20 on listeners
	// at charProxy+0x100.
	//
	// Signature: void(controller, charProxy, manifold*, simplexInput)
	//   manifold: [0]=entry_array_ptr, [1]=count (each entry 0x40 bytes)
	//   simplexInput+0x48: constraint array ptr (each 0x40 bytes)
	//   simplexInput+0x50: constraint count
	//   Manifold entry+0x28: body index (= bodyId in hknp)
	//   Constraint entry+0x10: surface velocity (4 floats, 16 bytes)
	// =========================================================================

	using ProcessConstraints_t = void(*)(void*, void*, void*, void*);
	static ProcessConstraints_t g_originalProcessConstraints = nullptr;

	void hookedProcessConstraintsCallback(void* controller, void* charProxy,
		void* manifold, void* simplexInput)
	{
		// Fast path: if hooks are disabled, call original and skip post-processing.
		// This prevents most SEH triggers. The __try remains as a safety net for TOCTOU races
		// where the world dies between this check and the actual Havok access.
		if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
			if (g_originalProcessConstraints) {
				g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
			}
			return;
		}

		__try {
			// Call original — populates all constraints with surface velocities
			g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);

			// Only post-process if ROCK is active and holding something
			auto* pi = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
			if (!pi || !pi->isInitialized()) return;

			bool rightHolding = pi->getRightHand().isHolding();
			bool leftHolding = pi->getLeftHand().isHolding();
			if (!rightHolding && !leftHolding) return;

			// Read manifold and constraint arrays
			auto** manifoldPtrs = reinterpret_cast<char**>(manifold);
			char* manifoldEntries = manifoldPtrs[0];
			int manifoldCount = *reinterpret_cast<int*>(&manifoldPtrs[1]);

			char* constraintArray = *reinterpret_cast<char**>(
				reinterpret_cast<char*>(simplexInput) + 0x48);
			int constraintCount = *reinterpret_cast<int*>(
				reinterpret_cast<char*>(simplexInput) + 0x50);

			if (!manifoldEntries || !constraintArray) return;

			int count = (std::min)(manifoldCount, constraintCount);
			int zeroed = 0;

			// DIAGNOSTIC: dump raw data to find correct body ID offset
			static int diagDumpCounter = 0;
			bool doDiag = (++diagDumpCounter >= 45);
			if (doDiag) {
				diagDumpCounter = 0;
				logger::info("[ROCK::CC] DIAG: manifold={} entries={} mCount={} constraints={} cCount={}",
					manifold, (void*)manifoldEntries, manifoldCount,
					(void*)constraintArray, constraintCount);
			}

			for (int i = 0; i < count; i++) {
				char* manifoldEntry = manifoldEntries + i * 0x40;

				// Try multiple offsets to find body ID
				// Read raw uint32 values at various offsets
				std::uint32_t at20 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x20);
				std::uint32_t at24 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x24);
				std::uint32_t at28 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x28);
				std::uint32_t at2C = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x2C);
				std::uint32_t at30 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x30);
				std::uint32_t at34 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x34);
				std::uint32_t at38 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x38);
				std::uint32_t at3C = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x3C);

				if (doDiag) {
					logger::info("[ROCK::CC]   [{}] +20={} +24={} +28={} +2C={} +30={} +34={} +38={} +3C={}",
						i, at20, at24, at28, at2C, at30, at34, at38, at3C);
				}

				// C5 FIX: EXACT body ID matching only. The previous ±2 range tolerance
				// could zero surface velocity for completely unrelated bodies (adjacent
				// hknp body IDs are unrelated objects — walls, NPCs, etc.).
				// If SetMotionType creates new bodies, re-collect held IDs at that point
				// rather than fuzzy-matching here.
				std::uint32_t bodyId = at28;
				bool isHeld = false;
				auto checkExact = [](const std::vector<std::uint32_t>& ids, std::uint32_t bid) -> bool {
					for (auto id : ids) {
						if (bid == id) return true;
					}
					return false;
				};
				if (rightHolding) isHeld = checkExact(pi->getRightHand().getHeldBodyIds(), bodyId);
				if (!isHeld && leftHolding) isHeld = checkExact(pi->getLeftHand().getHeldBodyIds(), bodyId);

				if (isHeld) {
					// Zero surface velocity at constraint entry+0x10 (4 floats)
					char* constraintEntry = constraintArray + i * 0x40;
					auto* surfaceVel = reinterpret_cast<float*>(constraintEntry + 0x10);
					surfaceVel[0] = 0.0f;
					surfaceVel[1] = 0.0f;
					surfaceVel[2] = 0.0f;
					surfaceVel[3] = 0.0f;
					zeroed++;

					if (doDiag) {
						logger::info("[ROCK::CC]   → ZEROED constraint {} (matched body ID)", i);
					}
				}
			}

			if (zeroed > 0 && g_rockConfig.rockDebugVerboseLogging) {
				static int zeroLogCounter = 0;
				if (++zeroLogCounter >= 90) {
					zeroLogCounter = 0;
					logger::info("[ROCK::CC] Zeroed surface velocity for {} held body constraints", zeroed);
				}
			}
		}
		__except (EXCEPTION_EXECUTE_HANDLER) {
			static int sehCount = 0;
			if (sehCount++ % 100 == 0) {
				logger::error("[ROCK::CC] SEH exception in hookedProcessConstraintsCallback (count={})",
					sehCount);
			}
		}
	}

	void installRefreshManifoldHook()
	{
		static bool installed = false;
		if (installed) return;
		installed = true;

		// Hook bhkCharProxyController::processConstraintsCallback at 0x141E4B7E0
		// Uses manual trampoline with VirtualAlloc(PAGE_EXECUTE_READWRITE) instead of
		// F4SE trampoline, which allocates memory in non-executable regions causing
		// EXCEPTION_ACCESS_VIOLATION when the original function is called through it.
		static REL::Relocation<std::uintptr_t> target{ REL::Offset(0x1E4B7E0) };
		auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());

		// Step 1: Allocate executable memory for trampoline
		// The first 14 bytes of the target are: MOV RAX,RSP; MOV [RAX+0x20],R9;
		// MOV [RAX+0x18],R8; PUSH RBP; PUSH R12 — no RIP-relative instructions, safe to relocate.
		// CRITICAL: RAX is used by the function later (LEA RBP,[RAX-0x158]), so the JMP
		// back to the original MUST NOT clobber RAX. Use JMP [RIP+0] (FF 25 00 00 00 00)
		// followed by 8-byte absolute address — this clobbers NO registers.
		constexpr int STOLEN_BYTES = 14;
		constexpr int JMP_RIP_SIZE = 14;  // FF 25 00 00 00 00 + 8 byte addr
		auto* trampolineMem = reinterpret_cast<std::uint8_t*>(
			VirtualAlloc(nullptr, 64, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));

		if (!trampolineMem) {
			ROCK_LOG_ERROR(Init, "FAILED to allocate executable trampoline memory");
			return;
		}

		// Step 2: Copy stolen bytes to trampoline
		memcpy(trampolineMem, targetAddr, STOLEN_BYTES);

		// Step 3: Write JMP [RIP+0] back to original function + STOLEN_BYTES
		// This does NOT clobber any register (unlike MOV RAX,addr; JMP RAX)
		std::uintptr_t continueAddr = target.address() + STOLEN_BYTES;
		trampolineMem[STOLEN_BYTES + 0] = 0xFF;  // JMP [RIP+0]
		trampolineMem[STOLEN_BYTES + 1] = 0x25;
		trampolineMem[STOLEN_BYTES + 2] = 0x00;
		trampolineMem[STOLEN_BYTES + 3] = 0x00;
		trampolineMem[STOLEN_BYTES + 4] = 0x00;
		trampolineMem[STOLEN_BYTES + 5] = 0x00;
		*reinterpret_cast<std::uintptr_t*>(&trampolineMem[STOLEN_BYTES + 6]) = continueAddr;

		// Harden: remove write permission from trampoline page after all writes complete.
		DWORD oldTrampolineProtect;
		VirtualProtect(trampolineMem, 64, PAGE_EXECUTE_READ, &oldTrampolineProtect);

		g_originalProcessConstraints = reinterpret_cast<ProcessConstraints_t>(trampolineMem);

		// Step 4: Overwrite target function entry with JMP [RIP+0] to our hook
		// Same register-safe approach for the hook redirect
		DWORD oldProtect;
		if (VirtualProtect(targetAddr, STOLEN_BYTES, PAGE_EXECUTE_READWRITE, &oldProtect)) {
			std::uintptr_t hookAddr = reinterpret_cast<std::uintptr_t>(&hookedProcessConstraintsCallback);
			targetAddr[0] = 0xFF;   // JMP [RIP+0]
			targetAddr[1] = 0x25;
			targetAddr[2] = 0x00;
			targetAddr[3] = 0x00;
			targetAddr[4] = 0x00;
			targetAddr[5] = 0x00;
			*reinterpret_cast<std::uintptr_t*>(&targetAddr[6]) = hookAddr;
			VirtualProtect(targetAddr, STOLEN_BYTES, oldProtect, &oldProtect);

			ROCK_LOG_INFO(Init, "Installed processConstraintsCallback hook at 0x{:X} (Xbyak trampoline at 0x{:X})",
				target.address(), reinterpret_cast<std::uintptr_t>(trampolineMem));
		} else {
			ROCK_LOG_ERROR(Init, "FAILED to patch processConstraintsCallback — VirtualProtect failed");
			VirtualFree(trampolineMem, 0, MEM_RELEASE);
			g_originalProcessConstraints = nullptr;
		}
	}
}

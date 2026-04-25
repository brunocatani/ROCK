#include "PhysicsHooks.h"

#include "HavokOffsets.h"
#include "PhysicsInteraction.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RockConfig.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

namespace frik::rock
{

    using HandleBumpedCharacter_t = void (*)(void*, void*, void*);
    static HandleBumpedCharacter_t g_originalHandleBumped = nullptr;

    void hookedHandleBumpedCharacter(void* controller, void* bumpedCC, void* contactInfo)
    {
        if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
            if (g_originalHandleBumped) {
                g_originalHandleBumped(controller, bumpedCC, contactInfo);
            }
            return;
        }

        __try {
            if (PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire) && frik::api::FRIKApi::inst && frik::api::FRIKApi::inst->isSkeletonReady()) {
                auto* pi = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
                if (pi) {
                    if (pi->getRightHand().isHoldingAtomic() || pi->getLeftHand().isHoldingAtomic()) {
                        static int holdSkipLogCounter = 0;
                        if (++holdSkipLogCounter >= 30) {
                            holdSkipLogCounter = 0;
                            logger::info("[ROCK::Bump] Skipped bump — holding object");
                        }
                        return;
                    }
                }

                auto* rWandBump = f4vr::getRightHandNode();
                auto* lWandBump = f4vr::getLeftHandNode();
                if (rWandBump && lWandBump) {
                    auto* ci = reinterpret_cast<float*>(contactInfo);
                    RE::NiPoint3 contactPos(ci[0] * rock::kHavokToGameScale, ci[1] * rock::kHavokToGameScale, ci[2] * rock::kHavokToGameScale);

                    const auto rightHand = rWandBump->world.translate;
                    const auto leftHand = lWandBump->world.translate;

                    auto distR = (contactPos - rightHand).Length();
                    auto distL = (contactPos - leftHand).Length();

                    constexpr float bumpSkipRadius = 50.0f;
                    if (distR < bumpSkipRadius || distL < bumpSkipRadius) {
                        static int skipLogCounter = 0;
                        if (++skipLogCounter >= 10) {
                            skipLogCounter = 0;
                            logger::info("[ROCK::Bump] Skipped bump — hand dist R={:.1f} L={:.1f} contact=({:.1f},{:.1f},{:.1f})", distR, distL, contactPos.x, contactPos.y,
                                contactPos.z);
                        }
                        return;
                    }
                }
            }

            g_originalHandleBumped(controller, bumpedCC, contactInfo);
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehLogCounter = 0;
            if (sehLogCounter++ % 100 == 0) {
                logger::error(
                    "[ROCK::Bump] SEH exception caught on physics thread (count={}) — "
                    "trampoline or stale pointer issue",
                    sehLogCounter);
            }
        }
    }

    typedef void (*UpdateShapes_t)(void*);
    static UpdateShapes_t g_originalUpdateShapes = nullptr;

    void hookedUpdateShapes(void* charController)
    {
        g_originalUpdateShapes(charController);

        if (g_rockConfig.rockEnabled && g_rockConfig.rockCharControllerRadiusScale < 1.0f) {
            auto* cc = reinterpret_cast<std::uint8_t*>(charController);
            auto* radius1 = reinterpret_cast<float*>(cc + 0x58);
            auto* radius2 = reinterpret_cast<float*>(cc + 0x5C);

            *radius1 *= g_rockConfig.rockCharControllerRadiusScale;
            *radius2 *= g_rockConfig.rockCharControllerRadiusScale;
        }
    }

    void installCCRadiusHook() { ROCK_LOG_INFO(Init, "CC radius hook DISABLED (trampoline incompatible with target function)"); }

    void installBumpHook()
    {
        static bool installed = false;
        if (installed)
            return;
        installed = true;

        static REL::Relocation<std::uintptr_t> target{ REL::Offset(offsets::kFunc_HandleBumpedCharacter) };
        auto& trampoline = F4SE::GetTrampoline();

        g_originalHandleBumped =
            reinterpret_cast<HandleBumpedCharacter_t>(trampoline.write_branch<6>(target.address(), reinterpret_cast<std::uintptr_t>(&hookedHandleBumpedCharacter)));

        ROCK_LOG_INFO(Init, "Installed HandleBumpedCharacter hook at 0x{:X}, original at 0x{:X}", target.address(), reinterpret_cast<std::uintptr_t>(g_originalHandleBumped));
    }

    void installNativeGrabHook()
    {
        static bool installed = false;
        if (installed)
            return;
        installed = true;

        static REL::Relocation<std::uintptr_t> target{ REL::Offset(offsets::kFunc_VRGrabInitiate) };
        auto* addr = reinterpret_cast<std::uint8_t*>(target.address());

        DWORD oldProtect;
        if (VirtualProtect(addr, 3, PAGE_EXECUTE_READWRITE, &oldProtect)) {
            addr[0] = 0x31;
            addr[1] = 0xC0;
            addr[2] = 0xC3;
            VirtualProtect(addr, 3, oldProtect, &oldProtect);
            ROCK_LOG_INFO(Init, "Patched VR Grab Initiate at 0x{:X} — native grab DISABLED (xor eax,eax; ret)", target.address());
        } else {
            ROCK_LOG_ERROR(Init, "FAILED to patch VR Grab Initiate at 0x{:X} — VirtualProtect failed", target.address());
        }
    }

    using ProcessConstraints_t = void (*)(void*, void*, void*, void*);
    static ProcessConstraints_t g_originalProcessConstraints = nullptr;

    void hookedProcessConstraintsCallback(void* controller, void* charProxy, void* manifold, void* simplexInput)
    {
        if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
            if (g_originalProcessConstraints) {
                g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
            }
            return;
        }

        __try {
            g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);

            auto* pi = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            if (!pi || !pi->isInitialized())
                return;

            bool rightHolding = pi->getRightHand().isHoldingAtomic();
            bool leftHolding = pi->getLeftHand().isHoldingAtomic();
            if (!rightHolding && !leftHolding)
                return;

            auto** manifoldPtrs = reinterpret_cast<char**>(manifold);
            char* manifoldEntries = manifoldPtrs[0];
            int manifoldCount = *reinterpret_cast<int*>(&manifoldPtrs[1]);

            char* constraintArray = *reinterpret_cast<char**>(reinterpret_cast<char*>(simplexInput) + 0x48);
            int constraintCount = *reinterpret_cast<int*>(reinterpret_cast<char*>(simplexInput) + 0x50);

            if (!manifoldEntries || !constraintArray)
                return;

            int count = (std::min)(manifoldCount, constraintCount);
            int zeroed = 0;

            bool doDiag = false;

            for (int i = 0; i < count; i++) {
                char* manifoldEntry = manifoldEntries + i * 0x40;

                std::uint32_t at20 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x20);
                std::uint32_t at24 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x24);
                std::uint32_t at28 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x28);
                std::uint32_t at2C = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x2C);
                std::uint32_t at30 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x30);
                std::uint32_t at34 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x34);
                std::uint32_t at38 = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x38);
                std::uint32_t at3C = *reinterpret_cast<std::uint32_t*>(manifoldEntry + 0x3C);

                if (doDiag) {
                    logger::info("[ROCK::CC]   [{}] +20={} +24={} +28={} +2C={} +30={} +34={} +38={} +3C={}", i, at20, at24, at28, at2C, at30, at34, at38, at3C);
                }

                std::uint32_t bodyId = at28;
                bool isHeld = false;
                if (rightHolding)
                    isHeld = pi->getRightHand().isHeldBodyId(bodyId);
                if (!isHeld && leftHolding)
                    isHeld = pi->getLeftHand().isHeldBodyId(bodyId);

                if (isHeld) {
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

            if (zeroed > 0) {
                static int zeroLogCounter = 0;
                if (++zeroLogCounter >= 90) {
                    zeroLogCounter = 0;
                    logger::info("[ROCK::CC] Zeroed surface velocity for {} held body constraints", zeroed);
                }
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehCount = 0;
            if (sehCount++ % 100 == 0) {
                logger::error("[ROCK::CC] SEH exception in hookedProcessConstraintsCallback (count={})", sehCount);
            }
        }
    }

    void installRefreshManifoldHook()
    {
        static bool installed = false;
        if (installed)
            return;
        installed = true;

        static REL::Relocation<std::uintptr_t> target{ REL::Offset(offsets::kFunc_ProcessConstraintsCallback) };
        auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());

        constexpr int STOLEN_BYTES = 14;
        auto* trampolineMem = reinterpret_cast<std::uint8_t*>(VirtualAlloc(nullptr, 64, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));

        if (!trampolineMem) {
            ROCK_LOG_ERROR(Init, "FAILED to allocate executable trampoline memory");
            return;
        }

        memcpy(trampolineMem, targetAddr, STOLEN_BYTES);

        std::uintptr_t continueAddr = target.address() + STOLEN_BYTES;
        trampolineMem[STOLEN_BYTES + 0] = 0xFF;
        trampolineMem[STOLEN_BYTES + 1] = 0x25;
        trampolineMem[STOLEN_BYTES + 2] = 0x00;
        trampolineMem[STOLEN_BYTES + 3] = 0x00;
        trampolineMem[STOLEN_BYTES + 4] = 0x00;
        trampolineMem[STOLEN_BYTES + 5] = 0x00;
        *reinterpret_cast<std::uintptr_t*>(&trampolineMem[STOLEN_BYTES + 6]) = continueAddr;

        DWORD oldTrampolineProtect;
        VirtualProtect(trampolineMem, 64, PAGE_EXECUTE_READ, &oldTrampolineProtect);

        g_originalProcessConstraints = reinterpret_cast<ProcessConstraints_t>(trampolineMem);

        DWORD oldProtect;
        if (VirtualProtect(targetAddr, STOLEN_BYTES, PAGE_EXECUTE_READWRITE, &oldProtect)) {
            std::uintptr_t hookAddr = reinterpret_cast<std::uintptr_t>(&hookedProcessConstraintsCallback);
            targetAddr[0] = 0xFF;
            targetAddr[1] = 0x25;
            targetAddr[2] = 0x00;
            targetAddr[3] = 0x00;
            targetAddr[4] = 0x00;
            targetAddr[5] = 0x00;
            *reinterpret_cast<std::uintptr_t*>(&targetAddr[6]) = hookAddr;
            VirtualProtect(targetAddr, STOLEN_BYTES, oldProtect, &oldProtect);

            ROCK_LOG_INFO(Init, "Installed processConstraintsCallback hook at 0x{:X} (Xbyak trampoline at 0x{:X})", target.address(),
                reinterpret_cast<std::uintptr_t>(trampolineMem));
        } else {
            ROCK_LOG_ERROR(Init, "FAILED to patch processConstraintsCallback — VirtualProtect failed");
            VirtualFree(trampolineMem, 0, MEM_RELEASE);
            g_originalProcessConstraints = nullptr;
        }
    }
}

#include "physics-interaction/core/PhysicsHooks.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/NativeMeleeSuppressionPolicy.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RockConfig.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"
#include "f4vr/PlayerNodes.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cstring>
#include <string_view>

namespace frik::rock
{
    namespace
    {
        using NativeMeleeHandler_t = bool (*)(void*, RE::Actor*, RE::BSFixedString*);

        static NativeMeleeHandler_t g_originalWeaponSwingHandler = nullptr;
        static NativeMeleeHandler_t g_originalHitFrameHandler = nullptr;
        static std::array<std::atomic<std::uint64_t>, 2> g_nativeMeleePhysicalSwingExpiresAtMs{};
        constexpr std::uint64_t kNativeMeleePhysicalSwingLeaseMs = 250;

        bool isAddressInGameText(std::uintptr_t address)
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            return address >= text.address() && address < text.address() + text.size();
        }

        bool validateNativeMeleeVtableTarget(std::uintptr_t entryOffset, std::uintptr_t expectedFunctionOffset, const char* label)
        {
            REL::Relocation<std::uintptr_t> entry{ REL::Offset(entryOffset) };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot) {
                ROCK_LOG_ERROR(Init, "{} vtable validation failed: slot is null", label);
                return false;
            }

            const auto current = *slot;
            const auto expected = REL::Offset(expectedFunctionOffset).address();
            if (!current) {
                ROCK_LOG_ERROR(Init, "{} vtable validation failed: current target is null", label);
                return false;
            }

            if (current == expected) {
                return true;
            }

            if (isAddressInGameText(current)) {
                ROCK_LOG_ERROR(Init, "{} vtable validation failed: slot 0x{:X} points to game text 0x{:X}, expected 0x{:X}", label, entry.address(), current, expected);
                return false;
            }

            ROCK_LOG_WARN(Init, "{} vtable slot 0x{:X} is already patched to external target 0x{:X}; ROCK will chain it if hook install proceeds", label, entry.address(), current);
            return true;
        }

        bool isPlayerActor(const RE::Actor* actor)
        {
            const auto* player = RE::PlayerCharacter::GetSingleton();
            return actor && player && reinterpret_cast<const void*>(actor) == reinterpret_cast<const void*>(player);
        }

        bool isLeftSideString(const RE::BSFixedString* side)
        {
            if (!side) {
                return false;
            }

            const char* text = side->c_str();
            if (!text) {
                return false;
            }

            return std::string_view(text) == "Left";
        }

        bool isAnyNativeMeleePhysicalSwingActive()
        {
            const auto now = GetTickCount64();
            return native_melee_suppression::isPhysicalSwingLeaseActive(now, g_nativeMeleePhysicalSwingExpiresAtMs[0].load(std::memory_order_acquire)) ||
                   native_melee_suppression::isPhysicalSwingLeaseActive(now, g_nativeMeleePhysicalSwingExpiresAtMs[1].load(std::memory_order_acquire));
        }

        bool isNativeMeleePhysicalSwingActiveForSide(const RE::BSFixedString* side)
        {
            if (!side) {
                return isAnyNativeMeleePhysicalSwingActive();
            }

            const bool isLeft = isLeftSideString(side);
            const auto now = GetTickCount64();
            return native_melee_suppression::isPhysicalSwingLeaseActive(now, g_nativeMeleePhysicalSwingExpiresAtMs[isLeft ? 1 : 0].load(std::memory_order_acquire));
        }

        native_melee_suppression::NativeMeleePolicyInput makeNativeMeleePolicyInput(
            const native_melee_suppression::NativeMeleeEvent event, const RE::Actor* actor, const RE::BSFixedString* side)
        {
            return native_melee_suppression::NativeMeleePolicyInput{ .rockEnabled = g_rockConfig.rockEnabled,
                .suppressionEnabled = g_rockConfig.rockNativeMeleeSuppressionEnabled,
                .fullSuppression = g_rockConfig.rockNativeMeleeFullSuppression,
                .suppressWeaponSwing = g_rockConfig.rockNativeMeleeSuppressWeaponSwing,
                .suppressHitFrame = g_rockConfig.rockNativeMeleeSuppressHitFrame,
                .actorIsPlayer = isPlayerActor(actor),
                .physicalSwingActive = event == native_melee_suppression::NativeMeleeEvent::HitFrame ? isNativeMeleePhysicalSwingActiveForSide(side)
                                                                                                     : isAnyNativeMeleePhysicalSwingActive() };
        }

        bool applyNativeMeleeDecision(const native_melee_suppression::NativeMeleeEvent event, const native_melee_suppression::NativeMeleePolicyDecision& decision)
        {
            using native_melee_suppression::NativeMeleeEvent;
            using native_melee_suppression::NativeMeleeSuppressionAction;

            if (g_rockConfig.rockNativeMeleeDebugLogging || decision.action != NativeMeleeSuppressionAction::CallNative) {
                static std::atomic<std::uint32_t> weaponLogCounter{ 0 };
                static std::atomic<std::uint32_t> hitFrameLogCounter{ 0 };
                auto& counter = event == NativeMeleeEvent::WeaponSwing ? weaponLogCounter : hitFrameLogCounter;
                const auto count = counter.fetch_add(1, std::memory_order_relaxed) + 1;

                if (count == 1 || (g_rockConfig.rockNativeMeleeDebugLogging && count % 45 == 0) || count % 180 == 0) {
                    ROCK_LOG_DEBUG(Combat, "Native melee {} decision={} reason={} count={}",
                        event == NativeMeleeEvent::WeaponSwing ? "WeaponSwing" : "HitFrame",
                        decision.action == NativeMeleeSuppressionAction::CallNative      ? "native"
                            : decision.action == NativeMeleeSuppressionAction::ReturnHandled ? "handled"
                                                                                              : "unhandled",
                        decision.reason, count);
                }
            }

            switch (decision.action) {
            case native_melee_suppression::NativeMeleeSuppressionAction::CallNative:
                return true;
            case native_melee_suppression::NativeMeleeSuppressionAction::ReturnHandled:
                return true;
            case native_melee_suppression::NativeMeleeSuppressionAction::ReturnUnhandled:
                return false;
            }

            return true;
        }

        bool hookedWeaponSwingHandler(void* handler, RE::Actor* actor, RE::BSFixedString* side)
        {
            const auto input = makeNativeMeleePolicyInput(native_melee_suppression::NativeMeleeEvent::WeaponSwing, actor, side);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::WeaponSwing, input);

            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;
            const bool decisionResult = applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::WeaponSwing, decision);
            return shouldCallNative ? (g_originalWeaponSwingHandler ? g_originalWeaponSwingHandler(handler, actor, side) : false) : decisionResult;
        }

        bool hookedHitFrameHandler(void* handler, RE::Actor* actor, RE::BSFixedString* side)
        {
            const auto input = makeNativeMeleePolicyInput(native_melee_suppression::NativeMeleeEvent::HitFrame, actor, side);
            const auto decision = native_melee_suppression::evaluateNativeMeleeSuppression(native_melee_suppression::NativeMeleeEvent::HitFrame, input);

            const bool shouldCallNative = decision.action == native_melee_suppression::NativeMeleeSuppressionAction::CallNative;
            const bool decisionResult = applyNativeMeleeDecision(native_melee_suppression::NativeMeleeEvent::HitFrame, decision);
            return shouldCallNative ? (g_originalHitFrameHandler ? g_originalHitFrameHandler(handler, actor, side) : false) : decisionResult;
        }

        bool installNativeMeleeVtableHook(std::uintptr_t entryOffset, NativeMeleeHandler_t hook, NativeMeleeHandler_t& original, const char* label)
        {
            REL::Relocation<std::uintptr_t> entry{ REL::Offset(entryOffset) };
            auto* slot = reinterpret_cast<std::uintptr_t*>(entry.address());
            if (!slot) {
                ROCK_LOG_ERROR(Init, "FAILED to install {} hook: vtable slot is null", label);
                return false;
            }

            const auto hookAddress = reinterpret_cast<std::uintptr_t>(hook);
            const auto currentTarget = *slot;
            if (currentTarget == hookAddress) {
                ROCK_LOG_INFO(Init, "{} hook already installed at 0x{:X}", label, entry.address());
                return original != nullptr;
            }

            original = reinterpret_cast<NativeMeleeHandler_t>(currentTarget);
            if (!original) {
                ROCK_LOG_ERROR(Init, "FAILED to install {} hook at 0x{:X}: original is null", label, entry.address());
                return false;
            }

            DWORD oldProtect = 0;
            if (!VirtualProtect(slot, sizeof(*slot), PAGE_EXECUTE_READWRITE, &oldProtect)) {
                ROCK_LOG_ERROR(Init, "FAILED to install {} hook at 0x{:X}: VirtualProtect failed", label, entry.address());
                original = nullptr;
                return false;
            }

            *slot = hookAddress;
            FlushInstructionCache(GetCurrentProcess(), slot, sizeof(*slot));
            VirtualProtect(slot, sizeof(*slot), oldProtect, &oldProtect);

            ROCK_LOG_INFO(Init, "Installed {} vtable hook at 0x{:X}, original=0x{:X}, hook=0x{:X}", label, entry.address(), reinterpret_cast<std::uintptr_t>(original),
                hookAddress);
            return true;
        }
    }

    bool validateNativeMeleeSuppressionHookTargets()
    {
        const bool swingValid = validateNativeMeleeVtableTarget(
            offsets::kVtableEntry_WeaponSwingHandler_Handle, offsets::kFunc_WeaponSwingHandler_Handle, "WeaponSwingHandler::Handle");
        const bool hitFrameValid =
            validateNativeMeleeVtableTarget(offsets::kVtableEntry_HitFrameHandler_Handle, offsets::kFunc_HitFrameHandler_Handle, "HitFrameHandler::Handle");
        return swingValid && hitFrameValid;
    }

    void setNativeMeleePhysicalSwingActive(bool isLeft, bool active)
    {
        const auto expiresAt = active ? (GetTickCount64() + kNativeMeleePhysicalSwingLeaseMs) : 0;
        g_nativeMeleePhysicalSwingExpiresAtMs[isLeft ? 1 : 0].store(expiresAt, std::memory_order_release);
    }

    bool isNativeMeleePhysicalSwingActive(bool isLeft)
    {
        return native_melee_suppression::isPhysicalSwingLeaseActive(
            GetTickCount64(), g_nativeMeleePhysicalSwingExpiresAtMs[isLeft ? 1 : 0].load(std::memory_order_acquire));
    }


    using HandleBumpedCharacter_t = void (*)(void*, void*, void*);
    static HandleBumpedCharacter_t g_originalHandleBumped = nullptr;

    static void writeAbsoluteJump(std::uint8_t* target, std::uintptr_t destination)
    {
        target[0] = 0xFF;
        target[1] = 0x25;
        target[2] = 0x00;
        target[3] = 0x00;
        target[4] = 0x00;
        target[5] = 0x00;
        *reinterpret_cast<std::uintptr_t*>(target + 6) = destination;
    }

    static bool installEntryTrampolineHook(const char* label,
        std::uintptr_t targetOffset,
        const std::uint8_t* expectedPrefix,
        std::size_t stolenBytes,
        void* hook,
        void*& original)
    {
        if (stolenBytes < 14) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: stolen byte count {} cannot hold an absolute jump", label, stolenBytes);
            return false;
        }

        REL::Relocation<std::uintptr_t> target{ REL::Offset(targetOffset) };
        auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());
        if (!targetAddr || !expectedPrefix) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: target or validation bytes are null", label);
            return false;
        }

        if (std::memcmp(targetAddr, expectedPrefix, stolenBytes) != 0) {
            ROCK_LOG_ERROR(Init, "{} hook validation failed at 0x{:X}; native bytes changed, hook not installed", label, target.address());
            return false;
        }

        constexpr std::size_t kJumpBytes = 14;
        const std::size_t trampolineBytes = stolenBytes + kJumpBytes;
        auto* trampolineMem = reinterpret_cast<std::uint8_t*>(VirtualAlloc(nullptr, trampolineBytes, MEM_COMMIT | MEM_RESERVE, PAGE_EXECUTE_READWRITE));
        if (!trampolineMem) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline allocation failed", label);
            return false;
        }

        std::memcpy(trampolineMem, targetAddr, stolenBytes);
        writeAbsoluteJump(trampolineMem + stolenBytes, target.address() + stolenBytes);

        DWORD oldTrampolineProtect = 0;
        if (!VirtualProtect(trampolineMem, trampolineBytes, PAGE_EXECUTE_READ, &oldTrampolineProtect)) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline protection failed", label);
            VirtualFree(trampolineMem, 0, MEM_RELEASE);
            return false;
        }

        DWORD oldProtect = 0;
        if (!VirtualProtect(targetAddr, stolenBytes, PAGE_EXECUTE_READWRITE, &oldProtect)) {
            ROCK_LOG_ERROR(Init, "{} hook install failed at 0x{:X}: target protection failed", label, target.address());
            VirtualFree(trampolineMem, 0, MEM_RELEASE);
            return false;
        }

        writeAbsoluteJump(targetAddr, reinterpret_cast<std::uintptr_t>(hook));
        for (std::size_t i = kJumpBytes; i < stolenBytes; ++i) {
            targetAddr[i] = 0x90;
        }

        FlushInstructionCache(GetCurrentProcess(), targetAddr, stolenBytes);
        VirtualProtect(targetAddr, stolenBytes, oldProtect, &oldProtect);

        original = trampolineMem;
        ROCK_LOG_INFO(Init, "Installed {} hook at 0x{:X}, original trampoline=0x{:X}", label, target.address(), reinterpret_cast<std::uintptr_t>(trampolineMem));
        return true;
    }

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
                            ROCK_LOG_DEBUG(Bump, "Skipped bump — holding object");
                        }
                        return;
                    }
                }

                auto* rWandBump = f4vr::getRightHandNode();
                auto* lWandBump = f4vr::getLeftHandNode();
                if (rWandBump && lWandBump) {
                    auto* ci = reinterpret_cast<float*>(contactInfo);
                    RE::NiPoint3 contactPos(ci[0] * rock::havokToGameScale(), ci[1] * rock::havokToGameScale(), ci[2] * rock::havokToGameScale());

                    const auto rightHand = rWandBump->world.translate;
                    const auto leftHand = lWandBump->world.translate;

                    auto distR = (contactPos - rightHand).Length();
                    auto distL = (contactPos - leftHand).Length();

                    constexpr float bumpSkipRadius = 50.0f;
                    if (distR < bumpSkipRadius || distL < bumpSkipRadius) {
                        static int skipLogCounter = 0;
                        if (++skipLogCounter >= 10) {
                            skipLogCounter = 0;
                            ROCK_LOG_DEBUG(Bump, "Skipped bump — hand dist R={:.1f} L={:.1f} contact=({:.1f},{:.1f},{:.1f})", distR, distL, contactPos.x, contactPos.y,
                                contactPos.z);
                        }
                        return;
                    }
                }
            }

            if (g_originalHandleBumped) {
                g_originalHandleBumped(controller, bumpedCC, contactInfo);
            }
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

    void installBumpHook()
    {
        static bool installed = false;
        static bool installAttempted = false;
        if (installed)
            return;
        if (installAttempted)
            return;
        installAttempted = true;

        // Ghidra verified HandleBumpedCharacter at 0x141E24980 starts with ordinary
        // prologue instructions, not an existing branch/call. CommonLib write_branch
        // cannot derive a callable original from those bytes, so this hook uses an
        // explicit relocated-entry trampoline and validates the exact whole
        // instructions before patching.
        constexpr std::array<std::uint8_t, 15> expectedPrefix{
            0x48, 0x89, 0x5C, 0x24, 0x08,
            0x48, 0x89, 0x74, 0x24, 0x18,
            0x57,
            0x48, 0x83, 0xEC, 0x70
        };

        void* original = reinterpret_cast<void*>(g_originalHandleBumped);
        installed = installEntryTrampolineHook(
            "HandleBumpedCharacter", offsets::kFunc_HandleBumpedCharacter, expectedPrefix.data(), expectedPrefix.size(), &hookedHandleBumpedCharacter, original);
        g_originalHandleBumped = reinterpret_cast<HandleBumpedCharacter_t>(original);
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

    bool installNativeMeleeSuppressionHooks()
    {
        static bool weaponSwingInstalled = false;
        static bool hitFrameInstalled = false;
        if (weaponSwingInstalled && hitFrameInstalled) {
            return true;
        }

        if (!validateNativeMeleeSuppressionHookTargets()) {
            ROCK_LOG_ERROR(Init, "Native melee suppression hook validation failed; install deferred");
            return false;
        }

        if (!weaponSwingInstalled) {
            weaponSwingInstalled = installNativeMeleeVtableHook(
                offsets::kVtableEntry_WeaponSwingHandler_Handle, &hookedWeaponSwingHandler, g_originalWeaponSwingHandler, "WeaponSwingHandler::Handle");
        }
        if (!hitFrameInstalled) {
            hitFrameInstalled =
                installNativeMeleeVtableHook(offsets::kVtableEntry_HitFrameHandler_Handle, &hookedHitFrameHandler, g_originalHitFrameHandler, "HitFrameHandler::Handle");
        }

        ROCK_LOG_INFO(Init, "Native melee suppression hooks installed: weaponSwing={} hitFrame={} enabled={} full={} suppressSwing={} suppressHitFrame={}",
            weaponSwingInstalled ? "yes" : "no", hitFrameInstalled ? "yes" : "no", g_rockConfig.rockNativeMeleeSuppressionEnabled ? "yes" : "no",
            g_rockConfig.rockNativeMeleeFullSuppression ? "yes" : "no", g_rockConfig.rockNativeMeleeSuppressWeaponSwing ? "yes" : "no",
            g_rockConfig.rockNativeMeleeSuppressHitFrame ? "yes" : "no");
        return weaponSwingInstalled && hitFrameInstalled;
    }

    using ProcessConstraints_t = void (*)(void*, void*, void*, void*);
    static ProcessConstraints_t g_originalProcessConstraints = nullptr;

    void hookedProcessConstraintsCallback(void* controller, void* charProxy, void* manifold, void* simplexInput)
    {
        bool originalAttempted = false;
        if (!PhysicsInteraction::s_hooksEnabled.load(std::memory_order_acquire)) {
            if (g_originalProcessConstraints) {
                originalAttempted = true;
                g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
            }
            return;
        }

        __try {
            auto* pi = PhysicsInteraction::s_instance.load(std::memory_order_acquire);
            if (!pi || !pi->isInitialized()) {
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                }
                return;
            }

            bool rightHolding = pi->getRightHand().isHoldingAtomic();
            bool leftHolding = pi->getLeftHand().isHoldingAtomic();
            if (!rightHolding && !leftHolding) {
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                }
                return;
            }

            const bool diagnosticsEnabled = g_rockConfig.rockDebugGrabFrameLogging || g_rockConfig.rockDebugVerboseLogging;
            const auto contactPolicy = held_grab_cc_policy::evaluateHeldGrabContactPolicy(held_grab_cc_policy::HeldGrabContactPolicyInput{
                .hooksEnabled = true,
                .holdingHeldObject = rightHolding || leftHolding,
                .diagnosticsEnabled = diagnosticsEnabled,
            });
            if (!contactPolicy.mayFilterBeforeOriginal) {
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                }
                return;
            }

            const auto contactBuffers = held_grab_cc_policy::makeGeneratedContactBufferView(manifold, simplexInput);
            if (!contactBuffers.valid) {
                if (g_rockConfig.rockDebugVerboseLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Skipped held body character-controller pre-filter reason={} manifoldCount={} constraintCount={}",
                        contactBuffers.reason,
                        contactBuffers.manifoldCount,
                        contactBuffers.constraintCount);
                }
                if (g_originalProcessConstraints) {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                }
                return;
            }

            const auto filterResult = held_grab_cc_policy::filterGeneratedContactBuffers(contactBuffers, [&](std::uint32_t bodyId) {
                bool isHeld = false;
                if (rightHolding) {
                    isHeld = pi->getRightHand().isHeldBodyId(bodyId);
                }
                if (!isHeld && leftHolding) {
                    isHeld = pi->getLeftHand().isHeldBodyId(bodyId);
                }
                return isHeld;
            });

            if (diagnosticsEnabled && filterResult.valid) {
                if (filterResult.removedPairCount > 0) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Filtered {} held body character-controller contacts before original listener kept={} originalPairs={} reason={}",
                        filterResult.removedPairCount,
                        filterResult.keptPairCount,
                        filterResult.originalPairCount,
                        contactPolicy.reason);
                } else if (g_rockConfig.rockDebugVerboseLogging) {
                    ROCK_LOG_SAMPLE_DEBUG(CC,
                        g_rockConfig.rockLogSampleMilliseconds,
                        "Held body character-controller pre-filter found no held contacts originalPairs={} reason={}",
                        filterResult.originalPairCount,
                        filterResult.reason);
                }
            }

            if (g_originalProcessConstraints) {
                originalAttempted = true;
                g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            static int sehCount = 0;
            if (sehCount++ % 100 == 0) {
                logger::error("[ROCK::CC] SEH exception in hookedProcessConstraintsCallback (count={})", sehCount);
            }
            if (!originalAttempted && g_originalProcessConstraints) {
                __try {
                    originalAttempted = true;
                    g_originalProcessConstraints(controller, charProxy, manifold, simplexInput);
                } __except (EXCEPTION_EXECUTE_HANDLER) {
                }
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

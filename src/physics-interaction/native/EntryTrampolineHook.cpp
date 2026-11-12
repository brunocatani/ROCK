#include "physics-interaction/native/EntryTrampolineHook.h"

#include "physics-interaction/PhysicsLog.h"

#include <RPS/Runtime/HookChain.h>
#include <Windows.h>

#include <cstring>

namespace rock::entry_trampoline_hook
{
    namespace
    {
        constexpr DWORD kPageExecuteRead = 0x20u;
        constexpr DWORD kPageExecuteReadWrite = 0x40u;
        constexpr DWORD kVirtualMemoryCommitReserve = MEM_COMMIT | MEM_RESERVE;
        constexpr DWORD kVirtualMemoryRelease = MEM_RELEASE;
        // F4SE's stub for one 5-byte branch.
        constexpr std::size_t kChainedStubBytes = 14;

        void writeAbsoluteJump(std::uint8_t* target, std::uintptr_t destination)
        {
            target[0] = 0xFF;
            target[1] = 0x25;
            target[2] = 0x00;
            target[3] = 0x00;
            target[4] = 0x00;
            target[5] = 0x00;
            *reinterpret_cast<std::uintptr_t*>(target + 6) = destination;
        }
    }

    bool install(const char* label, std::uintptr_t targetModuleOffset, const std::uint8_t* expectedPrefix,
        std::size_t stolenBytes, void* hook, void*& original)
    {
        if (stolenBytes < 14) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: stolen byte count {} cannot hold an absolute jump", label,
                stolenBytes);
            return false;
        }

        REL::Relocation<std::uintptr_t> target{ REL::Offset(targetModuleOffset) };
        auto* targetAddr = reinterpret_cast<std::uint8_t*>(target.address());
        if (!targetAddr || !expectedPrefix || !hook) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: target, hook, or validation bytes are null", label);
            return false;
        }

        const auto resolved = RPS::Runtime::HookChain::resolveEntry(target.address(), { expectedPrefix, stolenBytes });
        if (!resolved) {
            ROCK_LOG_ERROR(Init,
                "{} hook validation failed at 0x{:X}; native bytes changed and no other plugin's detour is there, hook "
                "not installed",
                label, target.address());
            return false;
        }
        if (resolved.foreign()) {
            /*
             * Another plugin already detours this function with a jump at its
             * entry. Its detour stays this hook's original, so both run. Only a
             * 5-byte jump (through an F4SE stub near the game) replaces its
             * jump, never the bytes after it, which its trampoline may return
             * into.
             */
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < kChainedStubBytes) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: no trampoline space to chain behind {}", label,
                    resolved.owner);
                return false;
            }
            try {
                (void)trampoline.write_branch<5>(target.address(), reinterpret_cast<std::uintptr_t>(hook));
            } catch (...) {
                ROCK_LOG_ERROR(Init, "{} hook install failed: chained jump could not be written", label);
                return false;
            }
            original = reinterpret_cast<void*>(resolved.callee);
            ROCK_LOG_INFO(Init, "Installed {} hook at 0x{:X} chained behind {} (original=0x{:X})", label,
                target.address(), resolved.owner, resolved.callee);
            return true;
        }

        constexpr std::size_t kJumpBytes = 14;
        const std::size_t trampolineBytes = stolenBytes + kJumpBytes;
        auto* trampolineMem = reinterpret_cast<std::uint8_t*>(
            VirtualAlloc(nullptr, trampolineBytes, kVirtualMemoryCommitReserve, kPageExecuteReadWrite));
        if (!trampolineMem) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline allocation failed", label);
            return false;
        }

        std::memcpy(trampolineMem, targetAddr, stolenBytes);
        writeAbsoluteJump(trampolineMem + stolenBytes, target.address() + stolenBytes);

        DWORD oldTrampolineProtect = 0;
        if (!VirtualProtect(trampolineMem, trampolineBytes, kPageExecuteRead, &oldTrampolineProtect)) {
            ROCK_LOG_ERROR(Init, "{} hook install failed: trampoline protection failed", label);
            VirtualFree(trampolineMem, 0, kVirtualMemoryRelease);
            return false;
        }

        DWORD oldProtect = 0;
        if (!VirtualProtect(targetAddr, stolenBytes, kPageExecuteReadWrite, &oldProtect)) {
            ROCK_LOG_ERROR(Init, "{} hook install failed at 0x{:X}: target protection failed", label, target.address());
            VirtualFree(trampolineMem, 0, kVirtualMemoryRelease);
            return false;
        }

        writeAbsoluteJump(targetAddr, reinterpret_cast<std::uintptr_t>(hook));
        for (std::size_t i = kJumpBytes; i < stolenBytes; ++i) {
            targetAddr[i] = 0x90;
        }

        FlushInstructionCache(GetCurrentProcess(), targetAddr, stolenBytes);
        VirtualProtect(targetAddr, stolenBytes, oldProtect, &oldProtect);

        original = trampolineMem;
        ROCK_LOG_INFO(Init, "Installed {} hook at 0x{:X}, original trampoline=0x{:X}", label, target.address(),
            reinterpret_cast<std::uintptr_t>(trampolineMem));
        return true;
    }
}

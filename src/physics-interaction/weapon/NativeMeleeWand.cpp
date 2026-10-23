#include "physics-interaction/weapon/NativeMeleeWand.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <new>

namespace rock::native_melee_wand
{
    namespace
    {
        // FO4VR 1.2.72, verified in ghidra_fo4vr and checked live before patching.
        constexpr std::uintptr_t kPrimaryWand = 0x5AC8EB0;   // right wand singleton
        constexpr std::uintptr_t kSecondaryWand = 0x5AC72B8; // left wand singleton

        // mov reg, [rip+disp32] loads of the primary wand: velocity gate,
        // contact gun-bash speed, contact hit wand, contact hit velocity.
        struct Load
        {
            std::uintptr_t site;
            std::array<std::uint8_t, 3> opcode;
        };
        constexpr std::array<Load, 4> kLoads{ {
            { 0xEFED63, { 0x48, 0x8B, 0x05 } },
            { 0xEFF333, { 0x48, 0x8B, 0x05 } },
            { 0xEFF50A, { 0x4C, 0x8B, 0x35 } },
            { 0xEFF6F8, { 0x48, 0x8B, 0x05 } },
        } };
        constexpr std::size_t kLoadSize = 7;

        enum class State
        {
            Pending,
            Installed,
            Refused
        };
        // Frame thread only. The slot lives in trampoline memory for the
        // process lifetime, so the patched loads never reach freed memory.
        State s_state = State::Pending;
        std::atomic<std::uintptr_t>* s_slot = nullptr;
        bool s_lastLeft = false;

        std::uintptr_t wand(std::uintptr_t rva) noexcept
        {
            std::uintptr_t value = 0;
            (void)native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(REL::Offset(rva).address()), value);
            return value;
        }

        std::int64_t displacementTo(std::uintptr_t target, std::uintptr_t site) noexcept
        {
            return static_cast<std::int64_t>(target) - static_cast<std::int64_t>(REL::Offset(site).address() + kLoadSize);
        }

        bool loadMatches(const Load& load) noexcept
        {
            std::array<std::uint8_t, kLoadSize> live{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(load.site).address()), live.data(), live.size()) ||
                std::memcmp(live.data(), load.opcode.data(), load.opcode.size()) != 0) return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, live.data() + load.opcode.size(), sizeof(displacement));
            return displacement == displacementTo(REL::Offset(kPrimaryWand).address(), load.site);
        }

        bool install(std::uintptr_t primary) noexcept
        {
            if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
            for (const auto& load : kLoads) {
                if (!loadMatches(load)) {
                    ROCK_LOG_ERROR(Init, "Melee wand routing refused: load at {:#x} is not the primary wand", load.site);
                    return false;
                }
            }
            // The slot must sit within rel32 reach of the game; the trampoline does.
            constexpr std::size_t kSlotBytes = sizeof(std::atomic<std::uintptr_t>) + alignof(std::atomic<std::uintptr_t>);
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < kSlotBytes) {
                ROCK_LOG_ERROR(Init, "Melee wand routing refused: no trampoline space");
                return false;
            }
            const auto raw = reinterpret_cast<std::uintptr_t>(trampoline.allocate(kSlotBytes));
            const auto slot = (raw + alignof(std::atomic<std::uintptr_t>) - 1) & ~(std::uintptr_t{ alignof(std::atomic<std::uintptr_t>) } - 1);
            for (const auto& load : kLoads) {
                const auto displacement = displacementTo(slot, load.site);
                if (displacement < INT32_MIN || displacement > INT32_MAX) {
                    ROCK_LOG_ERROR(Init, "Melee wand routing refused: slot out of reach of {:#x}", load.site);
                    return false;
                }
            }
            s_slot = new (reinterpret_cast<void*>(slot)) std::atomic<std::uintptr_t>(primary);
            for (const auto& load : kLoads) {
                const auto displacement = static_cast<std::int32_t>(displacementTo(slot, load.site));
                REL::safe_write(REL::Offset(load.site).address() + load.opcode.size(), &displacement, sizeof(displacement));
            }
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_INFO(Init, "Melee wand routing installed: index-0 melee reads the wand of the hand that holds it");
            return true;
        }
    }

    void update(const bool indexZeroInLeftHand) noexcept
    {
        if (s_state == State::Refused) return;
        const auto primary = wand(kPrimaryWand);
        const auto secondary = wand(kSecondaryWand);
        if (!primary || !secondary) return;
        if (s_state == State::Pending) {
            s_state = install(primary) ? State::Installed : State::Refused;
            if (s_state != State::Installed) return;
        }
        s_slot->store(indexZeroInLeftHand ? secondary : primary, std::memory_order_release);
        if (indexZeroInLeftHand != s_lastLeft) {
            s_lastLeft = indexZeroInLeftHand;
            ROCK_LOG_INFO(Weapon, "Native melee wand: index 0 uses the {} wand", indexZeroInLeftHand ? "left" : "right");
        }
    }
}

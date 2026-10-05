#include "physics-interaction/native/NativeWeaponFadeSuppression.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <cstdint>

namespace rock::native_weapon_fade_suppression
{
    namespace
    {
        // Written once at F4SE Load; read by game-thread diagnostic startup.
        bool s_installed = false;
        constexpr std::uintptr_t kAlphaCommit = 0xEF7EAA;
        constexpr std::array<std::uint8_t, 13> kOriginal{
            0x0F,0x2E,0xB7,0x10,0x09,0x00,0x00, // ucomiss xmm6,[rdi+910]
            0x0F,0x84,0x7D,0x00,0x00,0x00,      // je 140EF7F34
        };
        constexpr std::array<std::uint8_t, 13> kOpaque{
            0x41,0x0F,0x28,0xF0,                 // movaps xmm6,xmm8 (1.0f)
            0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,0x90,
        };

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }
    }

    bool installed() noexcept
    {
        return s_installed;
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;

        // FO4VR raw witnesses: player update 140EF1AC3 calls 140EF7B00.
        // Every alpha branch in 140EF7DF5..7EA7 joins here. XMM8 is loaded
        // with 1.0 at 140EF7DE7 and remains unchanged through this block.
        // The native commit writes cached alpha then visits current geometry
        // (140EF7EE4 -> 140F4B270, or 140EF7F15 -> 140F022B0).
        // 140F4B2B9/C2 stores that scalar as fade/decal alpha.
        // Bypass the unchanged-alpha skip as well: a newly attached model
        // still needs initialization when the player's previous alpha was 1.
        // No timer, weapon state, cull bit, graph event or global LOD setting
        // is changed. The native traversal and its object lifetimes remain.
        if (!bytesMatch(kAlphaCommit, kOriginal) ||
            !bytesMatch(0xEF7DE7, std::array<std::uint8_t, 9>{0xF3,0x44,0x0F,0x10,0x05,0x50,0x78,0xD8,0x01}) ||
            !bytesMatch(0x2C7F640, std::array<std::uint8_t, 4>{0x00,0x00,0x80,0x3F}) ||
            !bytesMatch(0xEF7EB7, std::array<std::uint8_t, 16>{0xF3,0x0F,0x11,0xB7,0x10,0x09,0x00,0x00,0xF3,0x0F,0x11,0xB5,0x00,0x01,0x00,0x00}) ||
            !bytesMatch(0xEF7EE4, std::array<std::uint8_t, 5>{0xE8,0x87,0x33,0x05,0x00})) {
            ROCK_LOG_ERROR(Init, "Native weapon fade suppression rejected: player alpha instructions differ from verified FO4VR");
            return false;
        }
        try {
            const auto address = REL::Offset(kAlphaCommit).address();
            REL::safe_write(address, kOpaque.data(), kOpaque.size());
            if (!FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), kOpaque.size()) ||
                !bytesMatch(kAlphaCommit, kOpaque)) {
                ROCK_LOG_ERROR(Init, "Native weapon fade suppression could not verify the installed alpha patch");
                return false;
            }
            s_installed = true;
            ROCK_LOG_INFO(Init, "Native weapon fade suppression installed: player weapon alpha=1, refresh every update; all equip sources and melee/bash; combat timers unchanged");
            return true;
        } catch (...) {
            ROCK_LOG_ERROR(Init, "Native weapon fade suppression installation failed");
            return false;
        }
    }
}

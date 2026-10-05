#include "physics-interaction/native/NativeWeaponEquipAudio.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>

namespace rock::native_weapon_equip_audio
{
    namespace
    {
        using PlaySound = void (*)(RE::BGSSoundDescriptorForm*, std::uint32_t, const RE::NiPoint3*, RE::NiAVObject*);
        PlaySound s_playSound = nullptr;
        bool s_installed = false;
        constexpr std::uintptr_t kPlaySound = 0x81FBF0;

        struct Callsite
        {
            std::uintptr_t call;
            std::array<std::uint8_t, 5> original;
            std::uintptr_t positionLoad;
            std::array<std::uint8_t, 7> positionBytes;
        };
        // Raw FO4VR witnesses, 2026-10-04: FF4870/FF5FA0 select instance+38
        // (base+1D0) for equip; FF4900/FF5E40 select +40/+1D8 for unequip.
        // PlayerFastEquipSoundHandler FF4510 selects +48/+1E0. All five
        // calls marshal actor+0xD0 in R8 and ignore the playback result.
        // 81FBF0 forwards descriptor, flags, position and scene node to
        // 81FCD0 with a temporary sound handle; it returns void.
        constexpr std::array<Callsite, 5> kCalls{{
            {0xFF48D1, {0xE8,0x1A,0xB3,0x82,0xFF}, 0xFF48A4, {0x48,0x8D,0xB9,0xD0,0,0,0}},
            {0xFF4961, {0xE8,0x8A,0xB2,0x82,0xFF}, 0xFF4934, {0x48,0x8D,0xB9,0xD0,0,0,0}},
            {0xFF6046, {0xE8,0xA5,0x9B,0x82,0xFF}, 0xFF6019, {0x4C,0x8D,0xB1,0xD0,0,0,0}},
            {0xFF5EE6, {0xE8,0x05,0x9D,0x82,0xFF}, 0xFF5EB9, {0x4C,0x8D,0xB1,0xD0,0,0,0}},
            {0xFF4572, {0xE8,0x79,0xB6,0x82,0xFF}, 0xFF4563, {0x4C,0x8D,0x87,0xD0,0,0,0}},
        }};

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }

        void playEquipSound(RE::BGSSoundDescriptorForm* sound, std::uint32_t flags,
            const RE::NiPoint3* position, RE::NiAVObject* node)
        {
            // Native-caller thread, reentrant: no retained scene pointers,
            // actor mutation, shared per-event state or frame scheduling.
            if (isPlayerSource(reinterpret_cast<std::uintptr_t>(f4vr::getPlayer()),
                    reinterpret_cast<std::uintptr_t>(position))) {
                ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000, "Player weapon equip/unequip sound suppressed descriptor={:#x}",
                    reinterpret_cast<std::uintptr_t>(sound));
                return;
            }
            s_playSound(sound, flags, position, node);
        }

        bool verifyRedirect(std::uintptr_t address)
        {
            std::array<std::uint8_t, 5> call{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), call.data(), call.size()) ||
                call[0] != 0xE8) return false;
            std::int32_t displacement{};
            std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
            const auto stubAddress = address + 5 + displacement;
            std::array<std::uint8_t, 14> stub{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stubAddress), stub.data(), stub.size())) return false;
            // F4SE's shared five-byte-call stub is JMP [RIP+0], then the target.
            const std::array<std::uint8_t, 6> jump{0xFF,0x25,0,0,0,0};
            std::uintptr_t destination{};
            std::memcpy(&destination, stub.data() + jump.size(), sizeof(destination));
            return std::equal(jump.begin(), jump.end(), stub.begin()) &&
                destination == reinterpret_cast<std::uintptr_t>(&playEquipSound) &&
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(stubAddress), stub.size());
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        if (!bytesMatch(kPlaySound, std::array<std::uint8_t, 15>{
                0x48,0x83,0xEC,0x48,0x4C,0x89,0x4C,0x24,0x20,0x4D,0x8B,0xC8,0x44,0x8B,0xC2})) return false;
        for (const auto& site : kCalls) {
            if (!bytesMatch(site.call, site.original) || !bytesMatch(site.positionLoad, site.positionBytes)) {
                ROCK_LOG_ERROR(Init, "Weapon equip/unequip audio suppression rejected native callsite {:#x}", site.call);
                return false;
            }
        }
        auto& trampoline = F4SE::GetTrampoline();
        // All callsites share one F4SE absolute-jump stub for this destination.
        if (trampoline.free_size() < 14) return false;
        s_playSound = reinterpret_cast<PlaySound>(REL::Offset(kPlaySound).address());
        std::size_t patched = 0;
        try {
            for (const auto& site : kCalls) {
                ++patched;
                (void)trampoline.write_call<5>(REL::Offset(site.call).address(), &playEquipSound);
                if (!verifyRedirect(REL::Offset(site.call).address()) ||
                    !FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(REL::Offset(site.call).address()), 5))
                    throw std::runtime_error("sound call redirect verification failed");
            }
            s_installed = true;
            ROCK_LOG_INFO(Init, "Player weapon equip/unequip audio suppression installed: normal, unequip and fast-equip; 5 sound-only callsites");
            return true;
        } catch (...) {
            // A failed startup installation must not leave a subset of the
            // native audio routes redirected. No game events run during Load.
            for (std::size_t i = 0; i < patched; ++i) {
                const auto& site = kCalls[i];
                const auto address = REL::Offset(site.call).address();
                REL::safe_write(address, site.original.data(), site.original.size());
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), site.original.size());
            }
            ROCK_LOG_ERROR(Init, "Weapon equip/unequip audio suppression installation failed");
            return false;
        }
    }
}

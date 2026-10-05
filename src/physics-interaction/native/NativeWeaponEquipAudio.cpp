#include "physics-interaction/native/NativeWeaponEquipAudio.h"
#include "physics-interaction/native/NativeWeaponAnimationAudio.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsLog.h"
#include "RockConfig.h"
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
        using SelectItemSound = RE::BGSSoundDescriptorForm* (*)(RE::Actor*, RE::TESBoundObject*, bool, bool);
        SelectItemSound s_selectItemSound = nullptr;
        bool s_installed = false;
        // Retain until the new inventory/clip filters are runtime-qualified;
        // this distinguishes a missed source from a failed filter in one run.
        using BuildSound = bool (*)(void*, void*, const void*, float, std::uint32_t, void*);
        BuildSound s_buildSound = nullptr;
        struct TraceWindow
        {
            std::uint64_t sequence{}, firstFrame{};
            std::uint32_t weapon{}, remaining{};
            bool equip{};
        };
        thread_local TraceWindow t_trace{};
        constexpr std::uintptr_t kPlaySound = 0x81FBF0;

        struct Callsite
        {
            std::uintptr_t call;
            std::array<std::uint8_t, 5> original;
            std::uintptr_t positionLoad;
            std::array<std::uint8_t, 7> positionBytes;
            bool inventory{false};
        };
        // Raw FO4VR witnesses, 2026-10-04: FF4870/FF5FA0 select instance+38
        // (base+1D0) for equip; FF4900/FF5E40 select +40/+1D8 for unequip.
        // PlayerFastEquipSoundHandler FF4510 selects +48/+1E0. All five
        // calls marshal actor+0xD0 in R8 and ignore the playback result.
        // 81FBF0 forwards descriptor, flags, position and scene node to
        // 81FCD0 with a temporary sound handle; it returns void.
        constexpr std::array<Callsite, 6> kCalls{{
            {0xFF48D1, {0xE8,0x1A,0xB3,0x82,0xFF}, 0xFF48A4, {0x48,0x8D,0xB9,0xD0,0,0,0}},
            {0xFF4961, {0xE8,0x8A,0xB2,0x82,0xFF}, 0xFF4934, {0x48,0x8D,0xB9,0xD0,0,0,0}},
            {0xFF6046, {0xE8,0xA5,0x9B,0x82,0xFF}, 0xFF6019, {0x4C,0x8D,0xB1,0xD0,0,0,0}},
            {0xFF5EE6, {0xE8,0x05,0x9D,0x82,0xFF}, 0xFF5EB9, {0x4C,0x8D,0xB1,0xD0,0,0,0}},
            {0xFF4572, {0xE8,0x79,0xB6,0x82,0xFF}, 0xFF4563, {0x4C,0x8D,0x87,0xD0,0,0,0}},
            // Player pickup/putdown uses a separate descriptor selector.
            // F20720 and DC13D0 preserve RCX actor, RDX item, R8B pickup,
            // R9B use; F11950 independently supplies actor/item/pickup.
            // A null selector result skips ONLY this sound at F2073F.
            {0xF2073A, {0xE8,0x91,0x0C,0xEA,0xFF}, 0xDC13D0, {0x48,0x8B,0xC2,0x48,0x85,0xD2,0x74}, true},
        }};

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), N) && actual == expected;
        }

        void recordSound(const char* stage, const void* descriptor, std::uint32_t flags,
            bool built, const void* position = nullptr) noexcept
        {
            // Reject other native threads before touching frame/config state.
            if (!t_trace.remaining) return;
            const auto frame = runtime_state::currentFrame().frameIndex;
            if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled || frame < t_trace.firstFrame || frame - t_trace.firstFrame >= 120) {
                t_trace.remaining = 0;
                return;
            }
            --t_trace.remaining;
            try {
                std::uintptr_t vtable{};
                std::uint32_t soundForm{};
                // 1404A7B00/7B40 install the descriptor subobject at form+20;
                // 14081FD23 independently passes that subobject to BuildSound.
                // Decode a form only for that exact verified vtable.
                const bool descriptorReadable = descriptor && native_memory::tryReadValue(
                    reinterpret_cast<const std::uintptr_t*>(descriptor), vtable);
                const bool formKnown = descriptorReadable && vtable == REL::Offset(0x2CC7760).address() &&
                    native_memory::tryReadValue(&reinterpret_cast<const RE::TESForm*>(
                        reinterpret_cast<std::uintptr_t>(descriptor) - 0x20)->formID, soundForm);
                std::array<void*, 12> stack{};
                const auto count = RtlCaptureStackBackTrace(1, static_cast<DWORD>(stack.size()), stack.data(), nullptr);
                const auto address = [&](std::size_t i) { return reinterpret_cast<std::uintptr_t>(stack[i]); };
                ROCK_LOG_INFO(Weapon,
                    "EQUIP_AUDIO seq={} operation={} weapon={:08X} frame={} age={} stage={} descriptor={:#x} readable={} soundFormKnown={} sound={:08X} flags={:#x} built={} position={:#x} player={:#x} gameBase={:#x} stackCount={} stack=[{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x},{:#x}] budgetRemaining={}",
                    t_trace.sequence, t_trace.equip ? "equip" : "unequip", t_trace.weapon, frame, frame - t_trace.firstFrame,
                    stage, reinterpret_cast<std::uintptr_t>(descriptor), descriptorReadable, formKnown, soundForm, flags, built,
                    reinterpret_cast<std::uintptr_t>(position), reinterpret_cast<std::uintptr_t>(f4vr::getPlayer()),
                    REL::Module::get().base(), count, address(0), address(1), address(2), address(3), address(4), address(5),
                    address(6), address(7), address(8), address(9), address(10), address(11), t_trace.remaining);
            } catch (...) {
                t_trace.remaining = 0;
            }
        }

        bool traceBuildSound(void* manager, void* handle, const void* descriptor,
            float distance, std::uint32_t flags, void* extra)
        {
            const bool built = s_buildSound(manager, handle, descriptor, distance, flags, extra);
            recordSound("descriptor-build", descriptor, flags, built);
            return built;
        }

        void playEquipSound(RE::BGSSoundDescriptorForm* sound, std::uint32_t flags,
            const RE::NiPoint3* position, RE::NiAVObject* node)
        {
            // Native-caller thread, reentrant: no retained scene pointers,
            // actor mutation, shared per-event state or frame scheduling.
            const bool playerSource = isPlayerSource(reinterpret_cast<std::uintptr_t>(f4vr::getPlayer()),
                reinterpret_cast<std::uintptr_t>(position));
            recordSound(playerSource ? "equip-slot-muted" : "equip-slot-pass",
                sound ? reinterpret_cast<const char*>(sound) + 0x20 : nullptr, flags, false, position);
            if (playerSource) {
                ROCK_LOG_SAMPLE_DEBUG(Weapon, 1000, "Player weapon equip/unequip sound suppressed descriptor={:#x}",
                    reinterpret_cast<std::uintptr_t>(sound));
                return;
            }
            s_playSound(sound, flags, position, node);
        }

        RE::BGSSoundDescriptorForm* selectItemSound(RE::Actor* actor, RE::TESBoundObject* item,
            bool pickup, bool use)
        {
            auto* sound = s_selectItemSound(actor, item, pickup, use);
            // Both native item-sound resolvers (14DB70/14DC80) classify
            // WEAP from form+1A == 2B. Keep every other item and actor intact.
            const bool muted = isPlayerWeaponItem(reinterpret_cast<std::uintptr_t>(f4vr::getPlayer()),
                reinterpret_cast<std::uintptr_t>(actor), item && item->IsWeapon());
            recordSound(muted ? "inventory-slot-muted" : "inventory-slot-pass",
                sound ? reinterpret_cast<const char*>(sound) + 0x20 : nullptr, 0x10, false);
            if (muted) {
                try {
                    ROCK_LOG_SAMPLE_INFO(Weapon, 1000, "Player weapon inventory sound suppressed pickup={} weapon={:08X}",
                        pickup, item->formID);
                } catch (...) {
                    OutputDebugStringA("ROCK: weapon inventory audio suppression diagnostic failed\n");
                }
                return nullptr;
            }
            return sound;
        }

        bool verifyRedirect(std::uintptr_t address, std::uintptr_t expectedDestination)
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
                destination == expectedDestination &&
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(stubAddress), stub.size());
        }
    }

    void beginTransferTrace(std::uint32_t weaponForm, bool equip) noexcept
    {
        if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled) return;
        t_trace = {t_trace.sequence + 1, runtime_state::currentFrame().frameIndex, weaponForm, 64, equip};
        ROCK_LOG_INFO(Weapon, "EQUIP_AUDIO begin seq={} operation={} weapon={:08X} frame={} ready={} frames=120 events=64 thread={}",
            t_trace.sequence, equip ? "equip" : "unequip", weaponForm, t_trace.firstFrame, s_buildSound != nullptr, GetCurrentThreadId());
    }

    void traceAnimationSource(std::string_view tag, std::string_view clip, bool muted) noexcept
    {
        if (!t_trace.remaining) return;
        const auto frame = runtime_state::currentFrame().frameIndex;
        if (!g_rockConfig.rockDebugWeaponOmodDumpEnabled || frame < t_trace.firstFrame || frame - t_trace.firstFrame >= 120) return;
        --t_trace.remaining;
        try {
            ROCK_LOG_INFO(Weapon, "EQUIP_AUDIO seq={} operation={} weapon={:08X} frame={} age={} stage=animation-source tag={} clip={} muted={}",
                t_trace.sequence, t_trace.equip ? "equip" : "unequip", t_trace.weapon, frame, frame - t_trace.firstFrame, tag, clip, muted);
        } catch (...) {
            t_trace.remaining = 0;
        }
    }

    bool install() noexcept
    {
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        const bool animationReady = native_weapon_animation_audio::install();
        if (s_installed) return animationReady;
        // ABI witnesses: 14081FD23..3D passes manager, handle, descriptor,
        // XMM3 distance and two stack arguments. 141B4CF50..D105 consumes
        // those arguments and returns the native build result in AL.
        constexpr std::array<std::uint8_t, 15> buildEntry{
            0x48,0x8B,0xC4,0x48,0x89,0x58,0x08,0x48,0x89,0x68,0x10,0x48,0x89,0x70,0x18};
        void* originalBuild{};
        if (entry_trampoline_hook::install("Weapon transfer audio observation", 0x1B4CF50,
                buildEntry.data(), buildEntry.size(), reinterpret_cast<void*>(&traceBuildSound), originalBuild)) {
            s_buildSound = reinterpret_cast<BuildSound>(originalBuild);
        }
        if (!bytesMatch(kPlaySound, std::array<std::uint8_t, 15>{
                0x48,0x83,0xEC,0x48,0x4C,0x89,0x4C,0x24,0x20,0x4D,0x8B,0xC8,0x44,0x8B,0xC2})) return false;
        if (!bytesMatch(0xF2073F, std::array<std::uint8_t, 5>{0x48,0x85,0xC0,0x74,0x69})) return false;
        for (const auto& site : kCalls) {
            if (!bytesMatch(site.call, site.original) || !bytesMatch(site.positionLoad, site.positionBytes)) {
                ROCK_LOG_ERROR(Init, "Weapon equip/unequip audio suppression rejected native callsite {:#x}", site.call);
                return false;
            }
        }
        auto& trampoline = F4SE::GetTrampoline();
        // Five equipment calls share one stub; the item selector uses another.
        if (trampoline.free_size() < 28) return false;
        s_playSound = reinterpret_cast<PlaySound>(REL::Offset(kPlaySound).address());
        s_selectItemSound = reinterpret_cast<SelectItemSound>(REL::Offset(0xDC13D0).address());
        std::size_t patched = 0;
        try {
            for (const auto& site : kCalls) {
                ++patched;
                const auto destination = site.inventory ? reinterpret_cast<std::uintptr_t>(&selectItemSound) :
                    reinterpret_cast<std::uintptr_t>(&playEquipSound);
                (void)trampoline.write_call<5>(REL::Offset(site.call).address(), destination);
                if (!verifyRedirect(REL::Offset(site.call).address(), destination) ||
                    !FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(REL::Offset(site.call).address()), 5))
                    throw std::runtime_error("sound call redirect verification failed");
            }
            s_installed = true;
            ROCK_LOG_INFO(Init, "Player weapon equip/unequip audio suppression installed: normal, unequip, fast-equip and inventory pickup/putdown; 6 sound-only callsites; animationReady={}", animationReady);
            return animationReady;
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

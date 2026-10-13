#include "physics-interaction/native/NativeWeaponAnimationAudio.h"

#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/PhysicsLog.h"
#include "rock_support/Fo4VrRuntime.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <array>
#include <cstring>
#include <stdexcept>
#include <utility>

namespace rock::native_weapon_animation_audio
{
    namespace
    {
        using ProcessEvents = void (*)(void*, void*, void*);
        using CopyEvents = void (*)(void*, void*);
        ProcessEvents s_processEvents{};
        CopyEvents s_copyEvents{};
        bool s_installed{};

        // Borrowed only during the native call, on its calling thread. Nested
        // processing replaces this scope and restores it before returning.
        thread_local const void* t_playerGraph{};
        struct GraphScope
        {
            const void* previous;
            explicit GraphScope(const void* graph) : previous(std::exchange(t_playerGraph, graph)) {}
            ~GraphScope() { t_playerGraph = previous; }
        };

        // 190DC60 copies a ring of 24-byte events without transferring payload
        // ownership. 16A5F40 and the independent graph dump 16A9DF0 consume the
        // same layout. Only the external notification copy is changed below.
        struct Event { std::uint32_t id, padding; const void* payload; const void* sender; };
        struct Queue { Event* events; std::int32_t capacity, read, write, count; };
        static_assert(sizeof(Event) == 0x18 && sizeof(Queue) == 0x18);

        constexpr std::uintptr_t kProcessEvents = 0x16A5F40;
        constexpr std::array<std::uint8_t, 20> kProcessEntry{
            0x40,0x55,0x56,0x57,0x41,0x55,0x41,0x57,0x48,0x8D,0x6C,0x24,0xC9,0x48,0x81,0xEC,0x90,0,0,0};
        struct Callsite { std::uintptr_t rva; std::array<std::uint8_t, 5> bytes; };
        constexpr std::array<Callsite, 2> kCopyCalls{{
            {0x16A5FFB, {0xE8,0x60,0x7C,0x26,0x00}},
            {0x16A6058, {0xE8,0x03,0x7C,0x26,0x00}},
        }};

        template <std::size_t N>
        bool matches(std::uintptr_t rva, const std::array<std::uint8_t, N>& bytes)
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                actual.data(), actual.size()) && actual == bytes;
        }

        template <std::size_t N>
        bool copyText(const char* text, std::array<char, N>& out)
        {
            if (!native_memory::guardedCopyFromMemory(text, out.data(), out.size())) return false;
            return std::memchr(out.data(), '\0', out.size()) != nullptr;
        }

        // Use the native BSFixedString accessor (also called by 172DB0 and
        // 16A9DF0) so shallow strings keep their native interpretation.
        const char* fixedStringText(const void* value) noexcept
        {
            using GetText = const char* (*)(const void*);
            __try {
                return reinterpret_cast<GetText>(REL::Offset(0x1BC18F0).address())(value);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nullptr;
            }
        }

        void filterCopiedEvents(void* destination)
        {
            if (!t_playerGraph) return;
            Queue queue{};
            if (!native_memory::tryReadValue(static_cast<const Queue*>(destination), queue) || !queue.count) return;
            // This cap limits work on corrupt/abnormal native queues, never a
            // mute duration. Reject the whole copy if its bounds are unknown.
            if (!queue.events || queue.capacity <= 0 || queue.capacity > 4096 || queue.count < 0 ||
                queue.count > queue.capacity || queue.read < 0 || queue.read >= queue.capacity ||
                queue.write < 0 || queue.write > queue.capacity) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "Equip animation audio filter rejected notification queue bounds");
                return;
            }
            const void* database{};
            const void* names{};
            std::uint32_t nameCount{};
            // 16A60B8..60FD and 16AB011..B09B independently establish
            // graph+370 -> database, names+D8, count+E8; eight bytes per name.
            if (!native_memory::tryReadField(t_playerGraph, 0x370, database) || !database ||
                !native_memory::tryReadField(database, 0xD8, names) || !names ||
                !native_memory::tryReadField(database, 0xE8, nameCount) || !nameCount || nameCount > 65536) {
                ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "Equip animation audio filter rejected event-name table");
                return;
            }
            for (std::int32_t i = 0, index = queue.read; i < queue.count; ++i, index = (index + 1) % queue.capacity) {
                Event event{};
                if (!native_memory::tryReadValue(queue.events + index, event) || event.id >= nameCount) continue;
                const auto* nameSlot = static_cast<const char*>(names) + event.id * sizeof(std::uintptr_t);
                std::array<char, 32> tag{};
                if (!copyText(fixedStringText(nameSlot), tag) || !isSoundStart(tag.data())) continue;

                std::uintptr_t vtable{}, path{};
                std::array<char, 260> clip{};
                // 192F2F0/192F160 preserve the originating node at event+10;
                // 17BE5F0 resolves clones to their original node. Require the
                // concrete clip vtable. 192C660 constructs its hkStringPtr at
                // +90; 192CC15 uses that same tagged pointer as animation name.
                const bool knownClip = event.sender && native_memory::tryReadField(event.sender, 0, vtable) &&
                    vtable == REL::Offset(0x2E0FB38).address() &&
                    native_memory::tryReadField(event.sender, 0x90, path) &&
                    copyText(reinterpret_cast<const char*>(path & ~std::uintptr_t{1}), clip);
                if (!knownClip || !isEquipClip(clip.data())) continue;
                // 16A60C2 skips out-of-range IDs before accessing the payload.
                // The original queue still drives the behavior graph; native
                // payload cleanup, weapon events, and animation time are intact.
                constexpr std::uint32_t invalidEvent = 0xFFFFFFFF;
                const bool muted = native_memory::tryWriteValue(&queue.events[index].id, invalidEvent);
                if (!muted) {
                    ROCK_LOG_SAMPLE_WARN(Weapon, 5000, "Equip animation audio filter could not suppress notification");
                    continue;
                }
                ROCK_LOG_SAMPLE_INFO(Weapon, 1000, "Player weapon animation sound suppressed clip={} tag={}", clip.data(), tag.data());
            }
        }

        void copyEvents(void* destination, void* source)
        {
            s_copyEvents(destination, source);
            try {
                filterCopiedEvents(destination);
            } catch (...) {
                // A diagnostic allocation failure must not unwind through
                // Havok or prevent the native queue from being processed.
                OutputDebugStringA("ROCK: equip audio notification filtering interrupted by a C++ exception\n");
            }
        }

        void processEvents(void* graph, void* context, void* output)
        {
            std::uintptr_t vtable{}, owner{};
            // F2E4C2/E0C322 pass actor to the graph constructor; 16A3429
            // stores it at +380 and 16A8D70 uses it for the event holder.
            const auto player = reinterpret_cast<std::uintptr_t>(f4vr::getPlayer());
            const bool playerGraph = s_installed && player && graph && output &&
                native_memory::tryReadField(graph, 0, vtable) && vtable == REL::Offset(0x2E00A48).address() &&
                native_memory::tryReadField(graph, 0x380, owner) && owner == player;
            GraphScope scope(playerGraph ? graph : nullptr);
            s_processEvents(graph, context, output);
        }

        bool verifyCopyCall(std::uintptr_t address)
        {
            std::array<std::uint8_t, 5> call{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), call.data(), call.size()) || call[0] != 0xE8) return false;
            std::int32_t displacement{};
            std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
            const auto stubAddress = address + 5 + displacement;
            std::array<std::uint8_t, 14> stub{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stubAddress), stub.data(), stub.size())) return false;
            const std::array<std::uint8_t, 6> jump{0xFF,0x25,0,0,0,0};
            std::uintptr_t target{};
            std::memcpy(&target, stub.data() + 6, sizeof(target));
            return std::memcmp(stub.data(), jump.data(), jump.size()) == 0 &&
                target == reinterpret_cast<std::uintptr_t>(&copyEvents) &&
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(stubAddress), stub.size());
        }
    }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        if (!matches(kProcessEvents, kProcessEntry) ||
            !matches(0x190DC60, std::array<std::uint8_t, 16>{0x40,0x53,0x57,0x48,0x83,0xEC,0x48,0x8B,0x42,0x14,0x48,0x8B,0xDA,0x48,0x8B,0xF9}) ||
            !matches(0x16A3429, std::array<std::uint8_t, 7>{0x49,0x89,0xBE,0x80,0x03,0,0}) ||
            !matches(0x16A60B8, std::array<std::uint8_t, 16>{0x49,0x8B,0x85,0x70,0x03,0,0,0x89,0x55,0x13,0x3B,0x98,0xE8,0,0,0}) ||
            !matches(0x16A60F6, std::array<std::uint8_t, 7>{0x48,0x8B,0x88,0xD8,0,0,0}) ||
            !matches(0x192CC15, std::array<std::uint8_t, 7>{0x48,0x8B,0x87,0x90,0,0,0}) ||
            !matches(0x1BC18F0, std::array<std::uint8_t, 16>{0x48,0x8B,0x11,0x48,0x85,0xD2,0x74,0x18,0x8B,0x42,0x08,0xC1,0xE8,0x0E,0xA8,0x01})) {
            ROCK_LOG_ERROR(Init, "Equip animation audio filter rejected native function bytes");
            return false;
        }
        for (const auto& site : kCopyCalls) if (!matches(site.rva, site.bytes)) return false;
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 14) return false;
        s_copyEvents = reinterpret_cast<CopyEvents>(REL::Offset(0x190DC60).address());
        std::size_t patched{};
        try {
            for (const auto& site : kCopyCalls) {
                ++patched;
                const auto address = REL::Offset(site.rva).address();
                (void)trampoline.write_call<5>(address, &copyEvents);
                if (!verifyCopyCall(address) || !FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), 5))
                    throw std::runtime_error("animation audio copy redirect verification failed");
            }
            void* original{};
            if (!entry_trampoline_hook::install("Player equip animation audio context", kProcessEvents,
                    kProcessEntry.data(), kProcessEntry.size(), reinterpret_cast<void*>(&processEvents), original))
                throw std::runtime_error("animation audio context installation failed");
            s_processEvents = reinterpret_cast<ProcessEvents>(original);
            s_installed = true;
            ROCK_LOG_INFO(Init, "Player equip animation audio filter installed: source clip qualification, original behavior queue preserved");
            return true;
        } catch (...) {
            for (std::size_t i = 0; i < patched; ++i) {
                const auto& site = kCopyCalls[i];
                const auto address = REL::Offset(site.rva).address();
                REL::safe_write(address, site.bytes.data(), site.bytes.size());
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), site.bytes.size());
            }
            ROCK_LOG_ERROR(Init, "Player equip animation audio filter installation failed");
            return false;
        }
    }
}

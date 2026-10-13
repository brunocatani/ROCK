#include "physics-interaction/input/NativePipboyHandInput.h"

#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/NativePipboyHandMarkers.h"
#include "physics-interaction/PhysicsLog.h"
#include "RE/Bethesda/InputEvent.h"
#include "RE/Bethesda/UIMessage.h"
#include "RE/Scaleform/GFx/GFx_Player.h"
#include <REL/Relocation.h>
#include <Windows.h>
#include <openvr.h>
#include <array>
#include <atomic>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <utility>

namespace rock::native_pipboy_hand_input
{
    namespace
    {
        using queued_hand_input::Hand;
        using queued_hand_input::Origin;
        // Raw operands, not Ghidra's inferred C++ member labels. Two native
        // consumers load this movie at +40 (2198D02 and BA07C2). The typed
        // decompilation incorrectly labels the same member offset_0x38.
        constexpr std::ptrdiff_t kMenuMovieOffset = 0x40;
        constexpr std::uintptr_t kMovieInputQueueOffset = 0x268;
        using Button = void (*)(void*, RE::ButtonEvent*);
        using SendEvent = void (*)(const RE::BSFixedString*, RE::Scaleform::GFx::Event*);
        using Enqueue = std::uintptr_t (*)(void*, const RE::BSFixedString*, std::uint32_t, RE::UIMessage**);
        using ProcessMessage = std::uint32_t (*)(void*, RE::UIMessage*);
        using FlashCall = void (*)(void*, const RE::Scaleform::GFx::FunctionHandler::Params&);
        using AllocateEntry = void* (*)(void*);
        using DispatchKey = void (*)(void*, void*, void*);
        using DestroyMovie = void* (*)(void*, std::uint32_t);
        Button s_button = nullptr;
        SendEvent s_sendEvent = nullptr;
        Enqueue s_enqueue = nullptr;
        ProcessMessage s_processMessage = nullptr;
        FlashCall s_flashCall = nullptr;
        AllocateEntry s_allocateEntry = nullptr;
        DispatchKey s_dispatchKey = nullptr;
        DestroyMovie s_destroyMovie = nullptr;
        bool s_installed = false;
        std::atomic<std::uint64_t> s_sequence{0};
        std::atomic<std::uintptr_t> s_messageVtable{0};
        thread_local Origin s_origin{};
        thread_local void* s_dispatchMovie = nullptr;

        // Input -> native UIMessageQueue -> movie HandleEvent -> GFx's own
        // 100-entry ring -> key dispatch. The first transfer owns an immutable
        // message payload. Native ring entries borrow numeric origin records.
        // UI/movie owners may differ; this short lock protects ONLY the fixed
        // records, never a native call, allocation, logging or movie update.
        SRWLOCK s_recordsLock = SRWLOCK_INIT;
        queued_hand_input::Records s_records;
        class RecordsLock
        {
        public:
            RecordsLock() noexcept { AcquireSRWLockExclusive(&s_recordsLock); }
            ~RecordsLock() { ReleaseSRWLockExclusive(&s_recordsLock); }
            RecordsLock(const RecordsLock&) = delete;
            RecordsLock& operator=(const RecordsLock&) = delete;
        };

        class Scope
        {
        public:
            Scope(Origin origin, void* movie = nullptr) noexcept :
                _previous(std::exchange(s_origin, origin)),
                _movie(std::exchange(s_dispatchMovie, movie)) {}
            ~Scope() { s_origin = _previous; s_dispatchMovie = _movie; }
            Scope(const Scope&) = delete;
            Scope& operator=(const Scope&) = delete;
        private:
            Origin _previous;
            void* _movie;
        };

        const char* handName(Hand hand) noexcept
        {
            return hand == Hand::Left ? "left" : hand == Hand::Right ? "right" : "none";
        }

        void trace(const char* stage, Origin origin, const void* object, std::uint32_t detail = 0) noexcept
        {
            try {
                ROCK_LOG_INFO(Weapon,
                    "AKIMBO input stage={} token={} hand={} inputThread={} thread={} object={:#x} detail={}",
                    stage, origin.sequence, handName(origin.hand), origin.inputThread, GetCurrentThreadId(),
                    reinterpret_cast<std::uintptr_t>(object), detail);
            } catch (...) {
                OutputDebugStringA("ROCK: Pip-Boy input provenance diagnostic failed\n");
            }
        }

        Hand physicalHand(const RE::ButtonEvent* event) noexcept
        {
            if (!event || event->QDisabled() || event->idCode != vr::k_EButton_SteamVR_Trigger ||
                event->deviceID < 0 || event->device.get() < static_cast<RE::INPUT_DEVICE>(3) ||
                event->device.get() > static_cast<RE::INPUT_DEVICE>(5)) return Hand::None;
            auto* system = vr::VRSystem();
            if (!system) return Hand::None;
            switch (system->GetControllerRoleForTrackedDeviceIndex(
                static_cast<vr::TrackedDeviceIndex_t>(event->deviceID))) {
            case vr::TrackedControllerRole_LeftHand: return Hand::Left;
            case vr::TrackedControllerRole_RightHand: return Hand::Right;
            default: return Hand::None;
            }
        }

        void onButton(void* menu, RE::ButtonEvent* event) noexcept
        {
            const auto hand = physicalHand(event);
            Origin origin{};
            if (hand != Hand::None) {
                origin = {hand, s_sequence.fetch_add(1, std::memory_order_relaxed) + 1, GetCurrentThreadId()};
            }
            const Scope scope{origin};
            if (event && event->idCode == vr::k_EButton_SteamVR_Trigger &&
                (event->QJustPressed() || event->QReleased())) {
                trace(event->QPressed() ? "trigger-press" : "trigger-release", origin, menu, event->deviceID);
            }
            s_button(menu, event);
        }

        class HandMessage final : public RE::BSUIScaleformData
        {
        public:
            HandMessage(RE::Scaleform::GFx::Event* event, Origin source) noexcept : origin(source)
            {
                scaleformEvent = event;
            }
            const Origin origin;
        };
        static_assert(sizeof(RE::UIMessage) == 0x18);
        static_assert(sizeof(RE::BSUIScaleformData) == 0x20);

        void sendEvent(const RE::BSFixedString* menu, RE::Scaleform::GFx::Event* event) noexcept
        {
            if (s_origin.hand == Hand::None || !event) {
                s_sendEvent(menu, event);
                return;
            }
            void* queue = nullptr;
            if (!native_memory::tryReadValue(reinterpret_cast<void**>(REL::Offset(0x5932558).address()), queue) || !queue) {
                s_sendEvent(menu, event);
                return;
            }
            std::unique_ptr<HandMessage> owned;
            try {
                // Same game heap and one allocation as native 20D9BF0. Keep
                // the native event pointer and payload unchanged. Queue takes
                // ownership; its virtual deletion also destroys the origin.
                owned.reset(new HandMessage(event, s_origin));
            } catch (...) {
                trace("message-allocation-failed", s_origin, event);
                s_sendEvent(menu, event);
                return;
            }
            s_messageVtable.store(*reinterpret_cast<const std::uintptr_t*>(owned.get()), std::memory_order_release);
            trace("message-enqueue", s_origin, owned.get());
            RE::UIMessage* transfer = owned.release();
            // Verified 20D9D40 moves *transfer into the native locked queue and
            // nulls it; failure destroys it. Match the native caller's cleanup.
            (void)s_enqueue(queue, menu, 5, &transfer);
            const std::unique_ptr<RE::UIMessage> rejected{transfer};
        }

        std::uint32_t processMessage(void* menu, RE::UIMessage* message) noexcept
        {
            Origin origin{};
            void* movie = nullptr;
            if (message && *reinterpret_cast<const std::uintptr_t*>(message) ==
                    s_messageVtable.load(std::memory_order_acquire)) {
                origin = static_cast<const HandMessage*>(message)->origin;
                const bool readable = native_memory::tryReadField(menu, kMenuMovieOffset, movie);
                trace("message-dispatch", origin, message);
                try {
                    ROCK_LOG_INFO(Weapon,
                        "AKIMBO movie binding token={} menu={:#x} readable={} movie={:#x} expectedQueue={:#x}",
                        origin.sequence, reinterpret_cast<std::uintptr_t>(menu), readable,
                        reinterpret_cast<std::uintptr_t>(movie),
                        movie ? reinterpret_cast<std::uintptr_t>(movie) + kMovieInputQueueOffset : 0);
                } catch (...) {
                    OutputDebugStringA("ROCK: Pip-Boy movie binding diagnostic failed\n");
                }
            }
            const Scope scope{origin, movie};
            return s_processMessage(menu, message);
        }

        void* allocateEntry(void* queue, bool keyboard) noexcept
        {
            auto* entry = s_allocateEntry(queue);
            bool captured = false;
            const bool tagged = keyboard && s_origin.hand != Hand::None && s_dispatchMovie &&
                reinterpret_cast<std::uintptr_t>(queue) == reinterpret_cast<std::uintptr_t>(s_dispatchMovie) + kMovieInputQueueOffset;
            {
                const RecordsLock lock;
                // All seven native allocation callsites go through here.
                // Ring overwrite by mouse/touch/other input invalidates tags.
                s_records.discard(reinterpret_cast<std::uintptr_t>(entry));
                if (tagged) {
                    captured = s_records.replace(reinterpret_cast<std::uintptr_t>(entry),
                        reinterpret_cast<std::uintptr_t>(s_dispatchMovie), s_origin);
                }
            }
            if (tagged) trace(captured ? "key-enqueue" : "key-capacity-refused", s_origin, entry);
            else if (keyboard && s_origin.hand != Hand::None) {
                try {
                    ROCK_LOG_WARN(Weapon,
                        "AKIMBO key owner mismatch token={} queue={:#x} movie={:#x} expectedQueue={:#x}",
                        s_origin.sequence, reinterpret_cast<std::uintptr_t>(queue),
                        reinterpret_cast<std::uintptr_t>(s_dispatchMovie),
                        s_dispatchMovie ? reinterpret_cast<std::uintptr_t>(s_dispatchMovie) + kMovieInputQueueOffset : 0);
                } catch (...) {
                    OutputDebugStringA("ROCK: Pip-Boy queue owner diagnostic failed\n");
                }
            }
            return entry;
        }
        void* allocateKey(void* queue) noexcept { return allocateEntry(queue, true); }
        void* allocateOther(void* queue) noexcept { return allocateEntry(queue, false); }

        void dispatchKey(void* movie, void* entry, void* focus) noexcept
        {
            Origin origin{};
            {
                const RecordsLock lock;
                origin = s_records.take(reinterpret_cast<std::uintptr_t>(entry), reinterpret_cast<std::uintptr_t>(movie));
            }
            const Scope scope{origin};
            if (origin.hand != Hand::None) trace("key-dispatch", origin, entry);
            s_dispatchKey(movie, entry, focus);
        }

        void* destroyMovie(void* movie, std::uint32_t flags) noexcept
        {
            {
                const RecordsLock lock;
                s_records.retire(reinterpret_cast<std::uintptr_t>(movie));
            }
            native_pipboy_hand_markers::detach(movie);
            return s_destroyMovie(movie, flags);
        }

        void flashCall(void* menu, const RE::Scaleform::GFx::FunctionHandler::Params& params) noexcept
        {
            // PipboyMenu's code-object calls run on its movie thread.
            void* movie = nullptr;
            if (native_memory::tryReadField(menu, kMenuMovieOffset, movie) && movie)
                native_pipboy_hand_markers::attach(static_cast<RE::Scaleform::GFx::Movie*>(movie));
            if (reinterpret_cast<std::uintptr_t>(params.userData) == 12) {
                trace("inventory-callback", s_origin, menu, params.argCount);
                if (s_origin.hand == Hand::None) {
                    // Bounded, failure-only stack evidence distinguishes a
                    // further AS action queue from the native UI/key queues.
                    void* frames[12]{};
                    const auto count = CaptureStackBackTrace(0, 12, frames, nullptr);
                    try {
                        for (USHORT i = 0; i < count; ++i) {
                            const auto address = reinterpret_cast<std::uintptr_t>(frames[i]);
                            const auto base = REL::Module::get().base();
                            ROCK_LOG_INFO(Weapon, "AKIMBO missing-origin stack={} address={:#x} mainRva={:#x}",
                                i, address, address >= base && address - base < 0x7000000 ? address - base : 0);
                        }
                    } catch (...) {
                        OutputDebugStringA("ROCK: missing-origin stack diagnostic failed\n");
                    }
                }
            }
            s_flashCall(menu, params);
        }

        template <std::size_t N>
        bool bytesMatch(std::uintptr_t rva, const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(rva).address()),
                actual.data(), N) && actual == expected;
        }

        struct Call
        {
            std::uintptr_t rva;
            std::array<std::uint8_t, 5> bytes;
            std::uintptr_t hook;
        };
        struct Vtable
        {
            std::uintptr_t slot, original, hook;
        };
        bool pointerMatches(std::uintptr_t address, std::uintptr_t expected) noexcept
        {
            std::uintptr_t value = 0;
            return native_memory::tryReadValue(reinterpret_cast<const std::uintptr_t*>(address), value) && value == expected;
        }
        bool redirectMatches(const Call& site) noexcept
        {
            const auto address = REL::Offset(site.rva).address();
            std::array<std::uint8_t, 5> call{};
            if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), call.data(), call.size()) || call[0] != 0xE8) return false;
            std::int32_t displacement = 0;
            std::memcpy(&displacement, call.data() + 1, 4);
            const auto stub = address + 5 + displacement;
            constexpr std::array<std::uint8_t, 6> jump{0xFF,0x25,0,0,0,0};
            std::array<std::uint8_t, 6> actual{};
            return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stub), actual.data(), actual.size()) &&
                actual == jump && pointerMatches(stub + 6, site.hook) &&
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(stub), 14);
        }
    }

    Origin current() noexcept { return s_origin; }

    bool install() noexcept
    {
        if (s_installed) return true;
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) return false;
        const std::array<Call, 10> calls{{
            {0xBA2E4F, {0xE8,0x9C,0x6D,0x53,0x01}, reinterpret_cast<std::uintptr_t>(&sendEvent)},
            {0xBA2E8C, {0xE8,0x5F,0x6D,0x53,0x01}, reinterpret_cast<std::uintptr_t>(&sendEvent)},
            {0x2294F68, {0xE8,0x93,0xFE,0xFF,0xFF}, reinterpret_cast<std::uintptr_t>(&allocateKey)},
            {0x2294FEF, {0xE8,0x0C,0xFE,0xFF,0xFF}, reinterpret_cast<std::uintptr_t>(&allocateOther)},
            {0x2294DB8, {0xE8,0x43,0,0,0}, reinterpret_cast<std::uintptr_t>(&allocateOther)},
            {0x2294E88, {0xE8,0x73,0xFF,0xFF,0xFF}, reinterpret_cast<std::uintptr_t>(&allocateOther)},
            {0x2294ED8, {0xE8,0x23,0xFF,0xFF,0xFF}, reinterpret_cast<std::uintptr_t>(&allocateOther)},
            {0x22950AF, {0xE8,0x4C,0xFD,0xFF,0xFF}, reinterpret_cast<std::uintptr_t>(&allocateOther)},
            {0x2295108, {0xE8,0xF3,0xFC,0xFF,0xFF}, reinterpret_cast<std::uintptr_t>(&allocateOther)},
            {0x218FB74, {0xE8,0x07,0x02,0,0}, reinterpret_cast<std::uintptr_t>(&dispatchKey)},
        }};
        const std::array<Vtable, 4> vtables{{
            {0x2D58520, 0xBA2550, reinterpret_cast<std::uintptr_t>(&onButton)},
            {0x2D58430, 0xB9FF00, reinterpret_cast<std::uintptr_t>(&processMessage)},
            {0x2D58420, 0xBA0F30, reinterpret_cast<std::uintptr_t>(&flashCall)},
            {0x2EFA690, 0x2185540, reinterpret_cast<std::uintptr_t>(&destroyMovie)},
        }};
        for (const auto& call : calls) if (!bytesMatch(call.rva, call.bytes)) return false;
        for (const auto& table : vtables)
            if (!pointerMatches(REL::Offset(table.slot).address(), REL::Offset(table.original).address())) return false;
        // Couple each offset used by the reader to the actual instruction
        // operand. A future wrong constant cannot pass an independent stale
        // prefix check and silently discard all hand provenance again.
        if (!bytesMatch(0x2198D02, std::array<std::uint8_t,4>{0x48,0x8B,0x59,static_cast<std::uint8_t>(kMenuMovieOffset)}) ||
            !bytesMatch(0xBA07C2, std::array<std::uint8_t,4>{0x48,0x8B,0x4E,static_cast<std::uint8_t>(kMenuMovieOffset)}) ||
            !bytesMatch(0x218C352, std::array<std::uint8_t,7>{0x48,0x8D,0x8E,
                static_cast<std::uint8_t>(kMovieInputQueueOffset), static_cast<std::uint8_t>(kMovieInputQueueOffset >> 8),
                static_cast<std::uint8_t>(kMovieInputQueueOffset >> 16), static_cast<std::uint8_t>(kMovieInputQueueOffset >> 24)}) ||
            !bytesMatch(0x218FB57, std::array<std::uint8_t,7>{0x48,0x8D,0x8B,
                static_cast<std::uint8_t>(kMovieInputQueueOffset), static_cast<std::uint8_t>(kMovieInputQueueOffset >> 8),
                static_cast<std::uint8_t>(kMovieInputQueueOffset >> 16), static_cast<std::uint8_t>(kMovieInputQueueOffset >> 24)})) {
            ROCK_LOG_ERROR(Init, "Pip-Boy movie/queue layout validation failed");
            return false;
        }
        if (!bytesMatch(0xBA2550, std::array<std::uint8_t,14>{0x40,0x55,0x53,0x56,0x57,0x41,0x55,0x41,0x56,0x48,0x8D,0x6C,0x24,0xD1}) ||
            !bytesMatch(0xB9FF00, std::array<std::uint8_t,16>{0x48,0x89,0x5C,0x24,0x08,0x55,0x56,0x57,0x41,0x54,0x41,0x55,0x41,0x56,0x41,0x57}) ||
            !bytesMatch(0xBA0F30, std::array<std::uint8_t,11>{0x4C,0x8B,0xDC,0x55,0x53,0x57,0x48,0x8D,0x6C,0x24,0x90}) ||
            !bytesMatch(0x20D9D40, std::array<std::uint8_t,15>{0x48,0x89,0x5C,0x24,0x08,0x48,0x89,0x6C,0x24,0x10,0x48,0x89,0x74,0x24,0x18}) ||
            !bytesMatch(0x20D9BF0, std::array<std::uint8_t,13>{0x48,0x89,0x5C,0x24,0x10,0x55,0x56,0x41,0x56,0x48,0x83,0xEC,0x20}) ||
            !bytesMatch(0x2294E00, std::array<std::uint8_t,10>{0x48,0x83,0xB9,0x48,0x1F,0,0,0x64,0x75,0x2A}) ||
            !bytesMatch(0x218FD80, std::array<std::uint8_t,14>{0x48,0x89,0x5C,0x24,0x18,0x48,0x89,0x6C,0x24,0x20,0x56,0x57,0x41,0x57}) ||
            !bytesMatch(0x2185540, std::array<std::uint8_t,15>{0x48,0x89,0x5C,0x24,0x08,0x57,0x48,0x83,0xEC,0x20,0x8B,0xDA,0x48,0x8B,0xF9})) return false;
        auto& trampoline = F4SE::GetTrampoline();
        if (trampoline.free_size() < 56) return false;
        s_button = reinterpret_cast<Button>(REL::Offset(0xBA2550).address());
        s_sendEvent = reinterpret_cast<SendEvent>(REL::Offset(0x20D9BF0).address());
        s_enqueue = reinterpret_cast<Enqueue>(REL::Offset(0x20D9D40).address());
        s_processMessage = reinterpret_cast<ProcessMessage>(REL::Offset(0xB9FF00).address());
        s_flashCall = reinterpret_cast<FlashCall>(REL::Offset(0xBA0F30).address());
        s_allocateEntry = reinterpret_cast<AllocateEntry>(REL::Offset(0x2294E00).address());
        s_dispatchKey = reinterpret_cast<DispatchKey>(REL::Offset(0x218FD80).address());
        s_destroyMovie = reinterpret_cast<DestroyMovie>(REL::Offset(0x2185540).address());
        try {
            for (const auto& call : calls) {
                (void)trampoline.write_call<5>(REL::Offset(call.rva).address(), call.hook);
                if (!redirectMatches(call)) throw std::runtime_error("Pip-Boy queue call readback failed");
            }
            for (const auto& table : vtables) {
                REL::safe_write(REL::Offset(table.slot).address(), table.hook);
                if (!pointerMatches(REL::Offset(table.slot).address(), table.hook))
                    throw std::runtime_error("Pip-Boy queue vtable readback failed");
            }
            if (!FlushInstructionCache(GetCurrentProcess(), nullptr, 0))
                throw std::runtime_error("Pip-Boy queue instruction flush failed");
            s_installed = true;
            ROCK_LOG_INFO(Init, "Pip-Boy physical hand provenance installed across native UI messages and GFx key queue");
            return true;
        } catch (...) {
            // Startup only: all sites prevalidated, no runtime events yet.
            for (const auto& call : calls)
                REL::safe_write(REL::Offset(call.rva).address(), call.bytes.data(), call.bytes.size());
            for (const auto& table : vtables)
                REL::safe_write(REL::Offset(table.slot).address(), REL::Offset(table.original).address());
            FlushInstructionCache(GetCurrentProcess(), nullptr, 0);
            ROCK_LOG_ERROR(Init, "Pip-Boy queued input hook installation failed; native sites restored");
            return false;
        }
    }
}

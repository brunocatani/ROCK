#include "physics-interaction/native/WeaponActionTrace.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/ResourceUtils.h"
#include "RE/Bethesda/PlayerCharacter.h"

#include <Windows.h>
#include <TlHelp32.h>
#include <intrin.h>
#include <array>
#include <atomic>
#include <memory>
#include <string>
#include <vector>
#include <spdlog/async_logger.h>
#include <spdlog/details/thread_pool.h>
#include <spdlog/sinks/rotating_file_sink.h>

namespace rock::weapon_action_trace
{
    namespace
    {
        // FO4VR caller 140E6FEA0 and callees 140E712D0/140E72DA0,
        // raw disassembly checked 2026-09-11: RCX manager, RDX actor,
        // R8 object instance, R9 request; both return their result in AL.
        // Trace both branches without patching the entry bytes PALM validates.
        using EquipStep = bool (*)(void*, RE::Actor*, const RE::BGSObjectInstance*, void*);
        constexpr std::uintptr_t kCheckCall = 0xE6FFFA;
        constexpr std::uintptr_t kApplyCall = 0xE70036;
        constexpr std::array<std::uint8_t, 5> kCheckBytes{0xE8, 0xD1, 0x12, 0x00, 0x00};
        constexpr std::array<std::uint8_t, 5> kApplyBytes{0xE8, 0x65, 0x2D, 0x00, 0x00};
        EquipStep originalCheck = nullptr;
        EquipStep originalApply = nullptr;

        struct Module { std::uintptr_t base; std::uint32_t size; std::string name; };
        struct Session
        {
            std::vector<Module> modules; // Filled before publication; never refreshed in a hook.
            std::shared_ptr<spdlog::details::thread_pool> pool;
            std::shared_ptr<spdlog::async_logger> log;
        };
        std::unique_ptr<Session> session;
        std::atomic<bool> ready{false}, writerFailed{false}, failureReported{false};
        std::atomic<std::uint64_t> sequence{0}, handsContext{0};
        // Native requests can arrive on different engine threads. Each producer
        // has its own bounded rate budget; the async queue has a fixed capacity.
        struct Budget { ULONGLONG second{0}; unsigned count{0}, skipped{0}; };
        thread_local Budget budget;

        std::uint64_t record(const char* kind, RE::Actor* actor,
            const RE::BGSObjectInstance* object, bool suppressed,
            std::uintptr_t caller) noexcept
        {
            if (!ready.load(std::memory_order_acquire) ||
                writerFailed.load(std::memory_order_relaxed) || !logger::isDebugEnabled() ||
                !actor || actor != RE::PlayerCharacter::GetSingleton()) return 0;
            try {
                const auto now = GetTickCount64();
                if (budget.second != now / 1000) { budget.second = now / 1000; budget.count = 0; }
                if (budget.count++ >= 32) { ++budget.skipped; return 0; }

                std::uint32_t form = 0;
                std::uintptr_t instance = 0;
                bool readable = object == nullptr;
                if (object) {
                    RE::TESForm* base = nullptr;
                    readable = native_memory::tryReadValue(&object->object, base) && base &&
                        native_memory::tryReadValue(&base->formID, form);
                    // The established BGSObjectInstance prefix is two pointers;
                    // copy the smart-pointer representation without taking ownership.
                    if (!native_memory::guardedCopyFromMemory(&object->instanceData,
                            &instance, sizeof(instance))) readable = false;
                }

                std::array<void*, 16> stack{};
                const auto frames = RtlCaptureStackBackTrace(1,
                    static_cast<DWORD>(stack.size()), stack.data(), nullptr);
                std::array<char, 2048> stackText{};
                std::size_t used = 0;
                const auto appendAddress = [&](std::uintptr_t address) {
                    const Module* found = nullptr;
                    for (const auto& module : session->modules) {
                        if (address >= module.base && address - module.base < module.size) { found = &module; break; }
                    }
                    const auto result = fmt::format_to_n(stackText.data() + used, stackText.size() - used,
                        "{}{}+0x{:X}", used ? " | " : "", found ? found->name.c_str() : "unmapped",
                        found ? address - found->base : address);
                    used += (std::min)(result.size, stackText.size() - used);
                };
                appendAddress(caller);
                for (USHORT i = 0; i < frames && used < stackText.size(); ++i)
                    appendAddress(reinterpret_cast<std::uintptr_t>(stack[i]));

                const auto right = input_remap_runtime::readPhysicalTraceSnapshot(false);
                const auto left = input_remap_runtime::readPhysicalTraceSnapshot(true);
                const auto context = handsContext.load(std::memory_order_acquire);
                const auto id = sequence.fetch_add(1, std::memory_order_relaxed) + 1;
                session->log->info(
                    "WAT request={} kind={} tick={} thread={} form={:08X} instance=0x{:X} readable={} heldEquipSuppressed={} lastProviderFrame={} handFlags=0x{:02X} skipped={} queueOverruns={} callerStack=[{}]",
                    id, kind, now, GetCurrentThreadId(), form, instance, readable, suppressed,
                    context >> 8, context & 0xFF, budget.skipped, session->pool->overrun_counter(),
                    std::string_view(stackText.data(), used));
                budget.skipped = 0;
                // Raw levels/axis are observational atomic loads. Pending edges
                // may already have been consumed; they are not a second input reader.
                session->log->info(
                    "WAT input request={} right(valid={} seq={} ageMs={} buttons=0x{:X} trigger={} pendingPress=0x{:X} pendingRelease=0x{:X} rearm=0x{:X}) left(valid={} seq={} ageMs={} buttons=0x{:X} trigger={} pendingPress=0x{:X} pendingRelease=0x{:X} rearm=0x{:X})",
                    id, right.valid, right.sequence, right.ageMilliseconds, right.pressed, right.trigger,
                    right.pendingPressed, right.pendingReleased, right.rearm,
                    left.valid, left.sequence, left.ageMilliseconds, left.pressed, left.trigger,
                    left.pendingPressed, left.pendingReleased, left.rearm);
                return id;
            } catch (...) {
                writerFailed.store(true, std::memory_order_relaxed);
                return 0;
            }
        }

        void recordResult(std::uint64_t id, bool result) noexcept
        {
            if (!id) return;
            try { session->log->info("WAT return request={} nativeResult={}", id, result); }
            catch (...) { writerFailed.store(true, std::memory_order_relaxed); }
        }

        bool onCheck(void* manager, RE::Actor* actor, const RE::BGSObjectInstance* object, void* request)
        {
            const auto id = record("equip-check", actor, object, false,
                reinterpret_cast<std::uintptr_t>(_ReturnAddress()));
            const bool result = originalCheck(manager, actor, object, request);
            recordResult(id, result);
            return result;
        }

        bool onApply(void* manager, RE::Actor* actor, const RE::BGSObjectInstance* object, void* request)
        {
            const auto id = record("equip-apply", actor, object, false,
                reinterpret_cast<std::uintptr_t>(_ReturnAddress()));
            const bool result = originalApply(manager, actor, object, request);
            recordResult(id, result);
            return result;
        }

        bool validateCall(std::uintptr_t rva, const std::array<std::uint8_t, 5>& expected)
        {
            std::array<std::uint8_t, 5> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), actual.size()) && actual == expected;
        }
    }

    void initialize() noexcept
    {
        if (session || !logger::isDebugEnabled()) return;
        try {
            if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
                !validateCall(kCheckCall, kCheckBytes) || !validateCall(kApplyCall, kApplyBytes)) {
                ROCK_LOG_ERROR(Init, "Weapon action trace unavailable: native equip call validation failed");
                return;
            }
            auto next = std::make_unique<Session>();
            const auto path = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_0.9_WeaponActions.log");
            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 4 * 1024 * 1024, 2, true);
            next->pool = std::make_shared<spdlog::details::thread_pool>(1024, 1);
            next->log = std::make_shared<spdlog::async_logger>("ROCK_WeaponActions", sink,
                next->pool, spdlog::async_overflow_policy::overrun_oldest);
            next->log->set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
            next->log->flush_on(spdlog::level::info);
            next->log->set_error_handler([](const std::string&) { writerFailed.store(true, std::memory_order_relaxed); });

            const auto snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPMODULE, GetCurrentProcessId());
            if (snapshot == INVALID_HANDLE_VALUE) {
                ROCK_LOG_ERROR(Init, "Weapon action trace unavailable: cannot snapshot caller modules ({})", GetLastError());
                return;
            }
            // Close the OS snapshot even if a module-name allocation fails.
            struct CloseSnapshot { HANDLE handle; ~CloseSnapshot() { CloseHandle(handle); } } close{snapshot};
            MODULEENTRY32W entry{};
            entry.dwSize = sizeof(entry);
            if (!Module32FirstW(snapshot, &entry)) {
                ROCK_LOG_ERROR(Init, "Weapon action trace unavailable: empty caller module snapshot ({})", GetLastError());
                return;
            }
            do {
                std::array<char, 1024> name{};
                if (!WideCharToMultiByte(CP_UTF8, 0, entry.szModule, -1, name.data(),
                        static_cast<int>(name.size()), nullptr, nullptr)) {
                    ROCK_LOG_ERROR(Init, "Weapon action trace unavailable: cannot encode caller module name");
                    return;
                }
                next->modules.push_back({reinterpret_cast<std::uintptr_t>(entry.modBaseAddr), entry.modBaseSize, name.data()});
            } while (next->modules.size() < 512 && Module32NextW(snapshot, &entry));

            next->log->info("WAT start pid={} build={} {} modules={} debugLoggingGate=true maxRequestsPerThreadPerSecond=32 hands=right-low-nibble,left-high-nibble handBits=valid:1,surfaceLatch:2,looseObject:4,firingGrip:8 OpenVR=grip:2,trigger:33 drawForm=unspecified inputSnapshot=independent-atomic-loads",
                GetCurrentProcessId(), __DATE__, __TIME__, next->modules.size());
            // Both original targets are published before either callsite changes.
            originalCheck = reinterpret_cast<EquipStep>(REL::Offset(0xE712D0).address());
            originalApply = reinterpret_cast<EquipStep>(REL::Offset(0xE72DA0).address());
            session = std::move(next);
            auto& trampoline = F4SE::GetTrampoline();
            trampoline.write_call<5>(REL::Offset(kCheckCall).address(), &onCheck);
            trampoline.write_call<5>(REL::Offset(kApplyCall).address(), &onApply);
            ready.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Weapon action caller trace enabled: '{}' (existing debug logging; equip entry preserved)", path);
        } catch (...) {
            writerFailed.store(true, std::memory_order_relaxed);
            try { ROCK_LOG_ERROR(Init, "Weapon action trace initialization failed"); } catch (...) {}
        }
    }

    void invalidateContext() noexcept { handsContext.store(0, std::memory_order_release); }

    void publishHands(std::uint64_t frame, std::uint8_t hands) noexcept
    {
        handsContext.store((frame << 8) | hands, std::memory_order_release);
        if (writerFailed.load(std::memory_order_relaxed) && !failureReported.exchange(true)) {
            try { ROCK_LOG_ERROR(Weapon, "Weapon action caller trace stopped after a diagnostic writer/capture failure; trace is incomplete"); } catch (...) {}
        }
    }

    void recordDraw(RE::PlayerCharacter* player, bool draw, bool suppressedByHeldEquip, std::uintptr_t caller) noexcept
    {
        (void)record(draw ? "draw" : "sheathe", player, nullptr, suppressedByHeldEquip, caller);
    }
}

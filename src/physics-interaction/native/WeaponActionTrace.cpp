#include "physics-interaction/native/WeaponActionTrace.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/input/InputRemapRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/weapon/NativeWeaponQualification.h"
#include "rock_support/ResourceUtils.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESRace.h"

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
        BeforeEquip beforeEquip{};
        AfterEquip afterEquip{};
        bool equipHooksInstalled{};
        thread_local bool insideEquipBoundary{};

        struct Module { std::uintptr_t base; std::uint32_t size; std::string name; };
        struct Session
        {
            std::vector<Module> modules; // Filled before publication; never refreshed in a hook.
            std::shared_ptr<spdlog::details::thread_pool> pool;
            std::shared_ptr<spdlog::async_logger> log;
            // Provider frame thread only, after ready publication. The worker
            // receives formatted values and never follows these identities.
            native_weapon_qualification::Snapshot previousNative{};
            std::uint64_t nativeEpoch{};
            ULONGLONG nextNativeCapture{};
            std::uintptr_t nativeContractFailureRva{};
            bool hasNativeCapture{};
        };
        std::unique_ptr<Session> session;
        std::atomic<bool> ready{false}, writerFailed{false}, failureReported{false};
        std::atomic<std::uint64_t> sequence{0}, handsContext{0};
        std::atomic<std::uint64_t> contextEpoch{1};
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
            const bool observe = !insideEquipBoundary && actor == RE::PlayerCharacter::GetSingleton() && object && beforeEquip;
            struct BoundaryScope {
                bool active;
                explicit BoundaryScope(bool value) : active(value) { if (active) insideEquipBoundary = true; }
                ~BoundaryScope() { if (active) insideEquipBoundary = false; }
            } scope{observe};
            const bool permitted = !observe || beforeEquip(*object, request);
            const bool result = permitted && originalApply(manager, actor, object, request);
            if (observe && afterEquip) afterEquip(*object, result);
            recordResult(id, result);
            return result;
        }

        bool validateCall(std::uintptr_t rva, const std::array<std::uint8_t, 5>& expected)
        {
            std::array<std::uint8_t, 5> actual{};
            return native_memory::guardedCopyFromMemory(
                reinterpret_cast<const void*>(REL::Offset(rva).address()), actual.data(), actual.size()) && actual == expected;
        }

        std::uintptr_t nativeObservationContractFailure()
        {
            // Guard the actual member-access instructions, not only the file
            // version. Raw independent witnesses are in NativeWeaponQualification.h.
            struct Witness { std::uintptr_t rva; std::array<std::uint8_t, 12> bytes; };
            constexpr std::array witnesses{
                Witness{0xE09FD0, {0x40,0x53,0x48,0x83,0xEC,0x20,0x48,0x8B,0xD9,0x48,0x8B,0x89}},
                Witness{0x3E687D, {0x44,0x8B,0x8F,0xE8,0x05,0x00,0x00,0x83,0xC9,0xFF,0x33,0xD2}},
                Witness{0x3E6899, {0x48,0x8B,0x87,0xD8,0x05,0x00,0x00,0x4D,0x39,0x14,0x00,0x0F}},
                Witness{0xE80435, {0x49,0x8B,0x89,0x90,0x02,0x00,0x00,0x42,0x39,0x44,0x01,0x18}},
                Witness{0xEC48E3, {0x8B,0x58,0x18,0x48,0x85,0xC0,0x74,0x11,0xF0,0xFF,0x48,0x08}},
                Witness{0xE518BA, {0x8B,0x59,0x38,0xEB,0x0C,0x8B,0x9B,0x30,0x01,0x00,0x00,0xC1}},
                Witness{0xDE8650, {0x48,0x8B,0x59,0x30,0x48,0x85,0xDB,0x75,0x6C,0x48,0x8B,0x07}},
                Witness{0x54EC46, {0x8B,0xF3,0x3B,0x72,0x30,0x73,0x22,0x48,0x8B,0x4A,0x20,0x8B}},
            };
            for (const auto& witness : witnesses) {
                std::array<std::uint8_t, 12> actual{};
                if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(REL::Offset(witness.rva).address()),
                        actual.data(), actual.size()) || actual != witness.bytes) return witness.rva;
            }
            return 0;
        }

        native_weapon_qualification::Snapshot captureNativeState()
        {
            namespace qualification = native_weapon_qualification;
            if (session->nativeContractFailureRva)
                return {.stage = qualification::Stage::NativeContract,
                    .failedAddress = REL::Offset(session->nativeContractFailureRva).address()};
            auto* player = RE::PlayerCharacter::GetSingleton();
            const auto playerAddress = reinterpret_cast<std::uintptr_t>(player);
            std::uint8_t type{};
            std::uintptr_t vtable{}, raceGetter{};
            if (!qualification::detail::plausible(playerAddress) ||
                !native_memory::tryReadField(player, 0x1A, type) || type != 0x41 ||
                !native_memory::tryReadField(player, 0, vtable))
                return {.stage = qualification::Stage::Player, .failedAddress = playerAddress};
            // 3E6840 and EC6DB0 call this exact virtual slot before reading the
            // visual race. Refuse an altered dispatch rather than bypass it.
            if (!qualification::detail::plausible(vtable) ||
                !native_memory::tryReadField(reinterpret_cast<void*>(vtable), 0x488, raceGetter) ||
                raceGetter != REL::Offset(0xE09FD0).address())
                return {.stage = qualification::Stage::VisualRace, .failedAddress = vtable};
            auto* race = player->GetVisualsRace();
            RE::AIProcess* process{};
            RE::MiddleHighProcessData* middle{};
            if (!native_memory::tryReadValue(&player->currentProcess, process) ||
                !qualification::detail::plausible(reinterpret_cast<std::uintptr_t>(process)))
                return {.stage = qualification::Stage::Process, .failedAddress = reinterpret_cast<std::uintptr_t>(process)};
            if (!native_memory::tryReadValue(&process->middleHigh, middle))
                return {.stage = qualification::Stage::MiddleHigh, .failedAddress = reinterpret_cast<std::uintptr_t>(process)};
            return qualification::capture(reinterpret_cast<std::uintptr_t>(race),
                reinterpret_cast<std::uintptr_t>(middle), REL::Offset(0x2D7FCF8).address(),
                [](std::uintptr_t address, void* destination, std::size_t bytes) noexcept {
                    return native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), destination, bytes);
                });
        }

        void recordNativeState(std::uint64_t frame)
        {
            if (!ready.load(std::memory_order_acquire) || writerFailed.load(std::memory_order_relaxed)) return;
            if (!logger::isDebugEnabled()) { session->hasNativeCapture = false; return; }
            const auto epoch = contextEpoch.load(std::memory_order_acquire);
            if (epoch != session->nativeEpoch) {
                session->nativeEpoch = epoch;
                session->hasNativeCapture = false;
                session->nextNativeCapture = 0;
            }
            const auto now = GetTickCount64();
            if (now < session->nextNativeCapture) return;
            session->nextNativeCapture = now + 250;
            const auto observed = captureNativeState();
            if (session->hasNativeCapture && observed == session->previousNative) return;
            const auto id = sequence.fetch_add(1, std::memory_order_relaxed) + 1;
            session->log->info("AKQ sample={} frame={} epoch={} tick={} thread={} stage={} failedAddress=0x{:X} failedIndex={} race={:08X} slots={} equipped={} queueOverruns={} observationOnly=true",
                id, frame, epoch, now, GetCurrentThreadId(), native_weapon_qualification::stageName(observed.stage),
                observed.failedAddress, observed.failedIndex, observed.race, observed.slotCount, observed.equippedCount,
                session->pool->overrun_counter());
            if (observed.stage == native_weapon_qualification::Stage::Complete) {
                for (std::uint32_t i = 0; i < observed.slotCount; ++i) {
                    const auto& slot = observed.slots[i];
                    session->log->info("AKQ slot sample={} index={} form={:08X} node=\"{}\" nameTruncated={} parentCount={} parents={:08X},{:08X},{:08X},{:08X},{:08X},{:08X},{:08X},{:08X}",
                        id, i, slot.form, slot.node.data(), slot.nameTruncated, slot.parentCount,
                        slot.parents[0], slot.parents[1], slot.parents[2], slot.parents[3],
                        slot.parents[4], slot.parents[5], slot.parents[6], slot.parents[7]);
                }
                for (std::uint32_t i = 0; i < observed.equippedCount; ++i) {
                    const auto& item = observed.equipped[i];
                    session->log->info("AKQ item sample={} position={} form={:08X} type={} slot={:08X} index={} indexInMapping={} instance=0x{:X} data=0x{:X} weaponDataValid={} ammo={:08X} loaded={} attack={} muzzle=0x{:X}",
                        id, i, item.form, item.formType, item.slot, item.index, item.index < observed.slotCount,
                        item.instance, item.data, item.weaponDataValid, item.ammo, item.loaded, item.attackState, item.muzzle);
                }
            }
            session->previousNative = observed;
            session->hasNativeCapture = true;
        }
    }

    bool installEquipBoundary(BeforeEquip before, AfterEquip after) noexcept try
    {
        if (equipHooksInstalled) {
            beforeEquip = before;
            afterEquip = after;
            return true;
        }
        if (!REL::Module::IsVR() || REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
            !validateCall(kCheckCall, kCheckBytes) || !validateCall(kApplyCall, kApplyBytes) ||
            F4SE::GetTrampoline().free_size() < 32) return false;
        originalCheck = reinterpret_cast<EquipStep>(REL::Offset(0xE712D0).address());
        originalApply = reinterpret_cast<EquipStep>(REL::Offset(0xE72DA0).address());
        beforeEquip = before;
        afterEquip = after;
        F4SE::GetTrampoline().write_call<5>(REL::Offset(kCheckCall).address(), &onCheck);
        F4SE::GetTrampoline().write_call<5>(REL::Offset(kApplyCall).address(), &onApply);
        equipHooksInstalled = true;
        return true;
    }
    catch (...) {
        try { ROCK_LOG_ERROR(Init, "Weapon equip boundary installation failed"); } catch (...) {}
        return false;
    }

    void initialize() noexcept
    {
        if (session || !logger::isDebugEnabled()) return;
        try {
            if (!equipHooksInstalled && !installEquipBoundary(beforeEquip, afterEquip)) {
                ROCK_LOG_ERROR(Init, "Weapon action trace unavailable: native equip call validation failed");
                return;
            }
            auto next = std::make_unique<Session>();
            const auto path = resources::getPathInDocuments("/My Games/Fallout4VR/F4SE/ROCK_WeaponActions.log");
            auto sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(path, 4 * 1024 * 1024, 2, true);
            next->pool = std::make_shared<spdlog::details::thread_pool>(1024, 1);
            next->log = std::make_shared<spdlog::async_logger>("ROCK_WeaponActions", sink,
                next->pool, spdlog::async_overflow_policy::overrun_oldest);
            next->log->set_pattern("%Y-%m-%d %H:%M:%S.%e [%l] %v");
            next->log->flush_on(spdlog::level::info);
            next->log->set_error_handler([](const std::string&) { writerFailed.store(true, std::memory_order_relaxed); });
            next->nativeContractFailureRva = nativeObservationContractFailure();

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
            next->log->info("AKQ start revision=1 nativeContract={} failedWitnessRva=0x{:X} observationIntervalMs=250 slotsLimit={} equippedLimit={} parentLimit={} changesOnly=true captureOnly=true",
                !next->nativeContractFailureRva, next->nativeContractFailureRva, native_weapon_qualification::kMaximumSlots,
                native_weapon_qualification::kMaximumEquipped, native_weapon_qualification::kMaximumParents);
            session = std::move(next);
            ready.store(true, std::memory_order_release);
            ROCK_LOG_INFO(Init, "Weapon action caller trace enabled: '{}' (existing debug logging; equip entry preserved)", path);
        } catch (...) {
            writerFailed.store(true, std::memory_order_relaxed);
            try { ROCK_LOG_ERROR(Init, "Weapon action trace initialization failed"); } catch (...) {}
        }
    }

    void invalidateContext() noexcept
    {
        handsContext.store(0, std::memory_order_release);
        contextEpoch.fetch_add(1, std::memory_order_acq_rel);
    }

    void publishHands(std::uint64_t frame, std::uint8_t hands) noexcept
    {
        handsContext.store((frame << 8) | hands, std::memory_order_release);
        try { recordNativeState(frame); }
        catch (...) { writerFailed.store(true, std::memory_order_relaxed); }
        if (writerFailed.load(std::memory_order_relaxed) && !failureReported.exchange(true)) {
            try { ROCK_LOG_ERROR(Weapon, "Weapon action caller trace stopped after a diagnostic writer/capture failure; trace is incomplete"); } catch (...) {}
        }
    }

    void recordDraw(RE::PlayerCharacter* player, bool draw, bool suppressedByHeldEquip, std::uintptr_t caller) noexcept
    {
        (void)record(draw ? "draw" : "sheathe", player, nullptr, suppressedByHeldEquip, caller);
    }
}

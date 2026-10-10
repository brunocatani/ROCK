#include "physics-interaction/native/HeldWeaponInstantTransition.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/native/TriggerEquipReadinessTrace.h"

#include "RE/Bethesda/PlayerCharacter.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <intrin.h>

#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace rock::held_weapon_instant_transition
{
    namespace
    {
        using DrawWeaponMagicHands = void (*)(RE::PlayerCharacter*, bool);
        using PrepareEquippedWeaponDraw = void (*)(RE::PlayerCharacter*);

        constexpr std::uintptr_t kPrepareDrawEntry = 0xEF8E60;
        constexpr std::uintptr_t kEquipManagerPrepareCallsite = 0xE10794;
        constexpr std::ptrdiff_t kAttackCooldownOffset = 0x908;
        constexpr std::array<std::uint8_t, 5> kPrepareCall{
            0xE8, 0xC7, 0x86, 0x0E, 0x00,
        };
        // Startup-only installation; game-thread calls preserve the timer only
        // inside a trigger equip, never ordinary equipment or native bashing.
        bool s_attackDelayRemovalInstalled = false;
        bool s_presentationDeadlineVerified = false;

        constexpr std::uintptr_t kPlayerDrawWeaponEntry = 0x0F78D10;
        constexpr std::uintptr_t kEquipManagerDrawCallsite = 0x0E107A1;
        constexpr std::uintptr_t kEquipManagerDrawReturn = 0x0E107A7;
        constexpr std::uintptr_t kEquipManagerSheatheCallsite = 0x0E10988;
        constexpr std::uintptr_t kEquipManagerSheatheReturn = 0x0E1098E;
        constexpr std::uintptr_t kDrawWeaponVtableOffset = 0x648;

        constexpr std::array<std::uint8_t, 14> kExpectedPlayerDrawEntry{
            0x48, 0x89, 0x74, 0x24, 0x18,
            0x55,
            0x57,
            0x41, 0x56,
            0x48, 0x8D, 0x6C, 0x24, 0xB9,
        };
        constexpr std::array<std::uint8_t, 6> kExpectedEquipManagerVirtualCall{
            0xFF, 0x90, 0x48, 0x06, 0x00, 0x00,
        };

        struct TransactionScope
        {
            RE::PlayerCharacter* player{ nullptr };
            RequestReason reason{ RequestReason::InventoryEquip };
            held_weapon_instant_transition_policy::ActionTrace trace{};
        };

        DrawWeaponMagicHands s_originalDrawWeaponMagicHands = nullptr;
        std::atomic<bool> s_installed{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };
        thread_local TransactionScope* t_activeScope = nullptr;

        __declspec(noinline) void onDrawWeaponMagicHands(
            RE::PlayerCharacter* player,
            bool draw);

        template <std::size_t N>
        [[nodiscard]] bool bytesMatch(
            const std::uintptr_t moduleOffset,
            const std::array<std::uint8_t, N>& expected) noexcept
        {
            std::array<std::uint8_t, N> actual{};
            const auto address = REL::Offset(moduleOffset).address();
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(address),
                       actual.data(),
                       actual.size()) &&
                   actual == expected;
        }

        void onPrepareEquippedWeaponDraw(RE::PlayerCharacter* player)
        {
            if (t_activeScope && t_activeScope->player == player &&
                t_activeScope->reason == RequestReason::SameHandTrigger) {
                prepareDrawForTriggerEquip(player);
            } else {
                reinterpret_cast<PrepareEquippedWeaponDraw>(
                    REL::Offset(kPrepareDrawEntry).address())(player);
            }
        }

        void installTriggerEquipPreparation()
        {
            if (s_attackDelayRemovalInstalled) return;
            s_presentationDeadlineVerified =
                bytesMatch(0xEF8E93, std::array<std::uint8_t, 8>{ 0xF3,0x0F,0x11,0x81,0xA8,0x08,0x00,0x00 }) &&
                bytesMatch(0xEF7C1C, std::array<std::uint8_t, 9>{ 0x0F,0x2F,0x87,0xA8,0x08,0x00,0x00,0x72,0x15 });
            // Raw FO4VR 1.2.72: EquipManager calls preparation immediately
            // before its draw virtual call. Preparation writes +908; the
            // trigger attack gate at FCD90E rejects positive values there.
            // Leave the native entry intact for other mods and direct callers.
            if (!bytesMatch(kEquipManagerPrepareCallsite, kPrepareCall) ||
                !bytesMatch(kPrepareDrawEntry, std::array<std::uint8_t, 7>{
                    0x80, 0x89, 0xA3, 0x12, 0x00, 0x00, 0x80}) ||
                !bytesMatch(0xEF8EA3, std::array<std::uint8_t, 9>{
                    0xF3, 0x0F, 0x11, 0x81, 0x08, 0x09, 0x00, 0x00, 0xC3}) ||
                !bytesMatch(0xFCD90E, std::array<std::uint8_t, 7>{
                    0x0F, 0x2F, 0xB0, 0x08, 0x09, 0x00, 0x00})) {
                ROCK_LOG_ERROR(Init, "Trigger equip attack delay removal unavailable: native preparation/attack instructions changed");
                return;
            }
            auto& trampoline = F4SE::GetTrampoline();
            if (trampoline.free_size() < 14) {
                ROCK_LOG_ERROR(Init, "Trigger equip attack delay removal unavailable: trampoline capacity");
                return;
            }
            const auto address = REL::Offset(kEquipManagerPrepareCallsite).address();
            try {
                (void)trampoline.write_call<5>(address, &onPrepareEquippedWeaponDraw);
                std::array<std::uint8_t, 5> call{};
                if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(address), call.data(), call.size()) || call[0] != 0xE8)
                    throw std::runtime_error("prepare call redirect unreadable");
                std::int32_t displacement = 0;
                std::memcpy(&displacement, call.data() + 1, sizeof(displacement));
                const auto stubAddress = address + call.size() + displacement;
                std::array<std::uint8_t, 14> stub{};
                std::uintptr_t destination = 0;
                if (!native_memory::guardedCopyFromMemory(reinterpret_cast<const void*>(stubAddress), stub.data(), stub.size()))
                    throw std::runtime_error("prepare call stub unreadable");
                std::memcpy(&destination, stub.data() + 6, sizeof(destination));
                const std::array<std::uint8_t, 6> jump{ 0xFF, 0x25, 0, 0, 0, 0 };
                if (std::memcmp(stub.data(), jump.data(), jump.size()) != 0 ||
                    destination != reinterpret_cast<std::uintptr_t>(&onPrepareEquippedWeaponDraw) ||
                    !FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(stubAddress), stub.size()))
                    throw std::runtime_error("prepare call redirect mismatch");
                if (!FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), 5))
                    throw std::runtime_error("prepare call cache flush failed");
                s_attackDelayRemovalInstalled = true;
                ROCK_LOG_INFO(Init, "Trigger equip attack delay removal installed: preserve pre-existing combat cooldown at native preparation");
            } catch (...) {
                s_attackDelayRemovalInstalled = false;
                REL::safe_write(address, kPrepareCall.data(), kPrepareCall.size());
                FlushInstructionCache(GetCurrentProcess(), reinterpret_cast<const void*>(address), kPrepareCall.size());
                ROCK_LOG_ERROR(Init, "Trigger equip attack delay removal installation failed; native preparation retained");
            }
        }

        [[nodiscard]] bool entryHookMatches() noexcept
        {
            std::array<std::uint8_t, kExpectedPlayerDrawEntry.size()> expected{};
            expected[0] = 0xFF;
            expected[1] = 0x25;
            const auto hookAddress = reinterpret_cast<std::uintptr_t>(
                &onDrawWeaponMagicHands);
            std::memcpy(expected.data() + 6, &hookAddress, sizeof(hookAddress));

            std::array<std::uint8_t, kExpectedPlayerDrawEntry.size()> actual{};
            const auto entryAddress = REL::Offset(kPlayerDrawWeaponEntry).address();
            return native_memory::guardedCopyFromMemory(
                       reinterpret_cast<const void*>(entryAddress),
                       actual.data(),
                       actual.size()) &&
                   actual == expected;
        }

        class ScopeLease
        {
        public:
            explicit ScopeLease(TransactionScope& scope) noexcept
            {
                t_activeScope = &scope;
            }

            ~ScopeLease() noexcept
            {
                t_activeScope = nullptr;
            }

            ScopeLease(const ScopeLease&) = delete;
            ScopeLease& operator=(const ScopeLease&) = delete;
        };

        __declspec(noinline) void onDrawWeaponMagicHands(
            RE::PlayerCharacter* player,
            const bool draw)
        {
            auto* scope = t_activeScope;
            const auto returnAddress = reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            const auto decision = held_weapon_instant_transition_policy::classifyHookCall(
                scope != nullptr,
                scope && player == scope->player,
                draw,
                returnAddress,
                REL::Offset(kEquipManagerDrawReturn).address(),
                REL::Offset(kEquipManagerSheatheReturn).address());

            switch (decision) {
            case held_weapon_instant_transition_policy::HookDecision::SuppressDraw:
                held_weapon_instant_transition_policy::recordAction(
                    scope->trace,
                    held_weapon_instant_transition_policy::NativeAction::Draw);
                return;
            case held_weapon_instant_transition_policy::HookDecision::SuppressSheathe:
                held_weapon_instant_transition_policy::recordAction(
                    scope->trace,
                    held_weapon_instant_transition_policy::NativeAction::Sheathe);
                return;
            case held_weapon_instant_transition_policy::HookDecision::PassThroughPlayerMismatch:
                scope->trace.playerMismatch = true;
                break;
            case held_weapon_instant_transition_policy::HookDecision::PassThroughUnexpectedCaller:
                scope->trace.unexpectedCaller = true;
                break;
            case held_weapon_instant_transition_policy::HookDecision::PassThroughInactive:
            default:
                break;
            }

            if (s_originalDrawWeaponMagicHands) {
                s_originalDrawWeaponMagicHands(player, draw);
            }
        }
    }

    bool install() noexcept
    {
        if (s_installed.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(Init,
                "Held weapon instant transition unavailable: unsupported runtime");
            return false;
        }
        if (!bytesMatch(kEquipManagerDrawCallsite, kExpectedEquipManagerVirtualCall)) {
            ROCK_LOG_ERROR(Init,
                "Held weapon instant transition unavailable: EquipManager draw callsite changed");
            return false;
        }
        if (!bytesMatch(kEquipManagerSheatheCallsite, kExpectedEquipManagerVirtualCall)) {
            ROCK_LOG_ERROR(Init,
                "Held weapon instant transition unavailable: EquipManager sheathe callsite changed");
            return false;
        }
        void* original = reinterpret_cast<void*>(s_originalDrawWeaponMagicHands);
        const bool installed = entry_trampoline_hook::install(
            "PlayerCharacter::DrawWeaponMagicHands held weapon transaction",
            kPlayerDrawWeaponEntry,
            kExpectedPlayerDrawEntry.data(),
            kExpectedPlayerDrawEntry.size(),
            reinterpret_cast<void*>(&onDrawWeaponMagicHands),
            original);
        s_originalDrawWeaponMagicHands = reinterpret_cast<DrawWeaponMagicHands>(original);
        const bool ready = installed && s_originalDrawWeaponMagicHands != nullptr;
        s_ownerThreadId.store(0, std::memory_order_release);
        s_installed.store(ready, std::memory_order_release);
        if (ready) installTriggerEquipPreparation();
        return ready;
    }

    Readiness readinessFor(RE::PlayerCharacter* player) noexcept
    {
        if (!s_installed.load(std::memory_order_acquire)) {
            return { .reason = ReadinessReason::NotInstalled };
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            return { .reason = ReadinessReason::UnsupportedRuntime };
        }
        if (!player) {
            return { .reason = ReadinessReason::MissingPlayer };
        }
        const auto currentThreadId = GetCurrentThreadId();
        DWORD expectedOwnerThreadId = 0;
        (void)s_ownerThreadId.compare_exchange_strong(
            expectedOwnerThreadId,
            currentThreadId,
            std::memory_order_acq_rel,
            std::memory_order_acquire);
        if (currentThreadId != s_ownerThreadId.load(std::memory_order_acquire)) {
            return { .reason = ReadinessReason::WrongThread };
        }

        std::uintptr_t vtable = 0;
        std::uintptr_t drawSlot = 0;
        if (!native_memory::tryReadValue(
                reinterpret_cast<const std::uintptr_t*>(player),
                vtable) ||
            !vtable ||
            !native_memory::tryReadValue(
                reinterpret_cast<const std::uintptr_t*>(
                    vtable + kDrawWeaponVtableOffset),
                drawSlot)) {
            return { .reason = ReadinessReason::PlayerVtableUnreadable };
        }
        if (drawSlot != REL::Offset(kPlayerDrawWeaponEntry).address()) {
            return { .reason = ReadinessReason::PlayerDrawSlotChanged };
        }
        if (!entryHookMatches()) {
            return { .reason = ReadinessReason::EntryHookChanged };
        }
        if (!bytesMatch(kEquipManagerDrawCallsite, kExpectedEquipManagerVirtualCall)) {
            return { .reason = ReadinessReason::DrawCallsiteChanged };
        }
        if (!bytesMatch(kEquipManagerSheatheCallsite, kExpectedEquipManagerVirtualCall)) {
            return { .reason = ReadinessReason::SheatheCallsiteChanged };
        }
        return {
            .ready = true,
            .reason = ReadinessReason::Ready,
        };
    }

    ImmediateEquipResult equipImmediatelyWithoutActions(
        const ImmediateEquipInput& input) noexcept
    {
        ImmediateEquipResult result{};
        result.reason = input.reason;
        if (!input.manager || !input.player || !input.object || !input.equipSlot) {
            result.code = ImmediateEquipCode::MissingInput;
            return result;
        }

        const auto readiness = readinessFor(input.player);
        result.readinessReason = readiness.reason;
        if (!readiness.ready) {
            result.code = ImmediateEquipCode::CapabilityUnavailable;
            return result;
        }
        if (t_activeScope) {
            t_activeScope->trace.nestedScope = true;
            result.actionTrace.nestedScope = true;
            result.code = ImmediateEquipCode::NestedScope;
            return result;
        }

        TransactionScope scope{
            .player = input.player,
            .reason = input.reason,
        };
        result.attempted = true;
        trigger_equip_readiness_trace::stage("before-equip-manager");
        {
            const ScopeLease scopeLease{ scope };
            result.managerAccepted = input.manager->EquipObject(
                input.player,
                *input.object,
                input.stackID,
                1,
                input.equipSlot,
                false,
                false,
                false,
                true,
                false);
        }
        result.actionTrace = scope.trace;
        trigger_equip_readiness_trace::stage("after-equip-manager");

        if (!result.managerAccepted) {
            result.code = ImmediateEquipCode::ManagerRejected;
            return result;
        }
        if (!held_weapon_instant_transition_policy::isValidEquipActionTrace(
                result.actionTrace)) {
            result.code = ImmediateEquipCode::InvalidActionTrace;
            return result;
        }

        result.code = ImmediateEquipCode::Accepted;
        return result;
    }

    void prepareDrawForTriggerEquip(RE::PlayerCharacter* player) noexcept
    {
        if (!player) return;
        trigger_equip_readiness_trace::stage("before-native-preparation");
        const auto prepare = reinterpret_cast<PrepareEquippedWeaponDraw>(
            REL::Offset(kPrepareDrawEntry).address());
        float priorCooldown = 0.0f;
        const bool preserve = s_attackDelayRemovalInstalled && player &&
            native_memory::tryReadField(player, kAttackCooldownOffset, priorCooldown) &&
            std::isfinite(priorCooldown);
        prepare(player);
        if (!preserve) return;

        // This timer is shared with bash and other native combat input gates.
        // Restore its exact pre-call value, not an unconditional zero. The
        // synchronous preparation has no callbacks; its other writes (draw
        // flags) remain native. No delayed player or
        // weapon pointer is retained, and no per-frame cooldown clearing occurs.
        auto* timer = reinterpret_cast<float*>(reinterpret_cast<char*>(player) + kAttackCooldownOffset);
        if (!native_memory::tryWriteValue(timer, priorCooldown)) {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Trigger equip could not restore pre-existing attack cooldown");
            return;
        }
        ROCK_LOG_INFO(Weapon, "Trigger equip preparation preserved attack cooldown={:.4f}s; added equip delay removed", priorCooldown);
        // The captured half-second rejection is a second, independent native
        // presentation wait. Make only this trigger equip's deadline due now.
        // Leave bits 80/40 untouched: the native player update must still clear
        // the wait, issue its refresh task, and evaluate all other hide/attack
        // conditions. This does not force Drawn or bypass model/graph readiness.
        std::uint64_t ticks{};
        float secondsPerTick{};
        if (s_presentationDeadlineVerified &&
            native_memory::tryReadValue(reinterpret_cast<const std::uint64_t*>(REL::Offset(0x5BBC058).address()), ticks) &&
            native_memory::tryReadValue(reinterpret_cast<const float*>(REL::Offset(0x2C81B8C).address()), secondsPerTick) &&
            std::isfinite(secondsPerTick) && secondsPerTick > 0.0f) {
            const float due = static_cast<float>(ticks) * secondsPerTick;
            auto* deadline = reinterpret_cast<float*>(reinterpret_cast<char*>(player) + 0x8A8);
            if (std::isfinite(due) && native_memory::tryWriteValue(deadline, due)) {
                ROCK_LOG_INFO(Weapon, "Trigger equip presentation deadline due now; native readiness update retained");
            } else {
                ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Trigger equip presentation deadline write unavailable; native wait retained");
            }
        } else {
            ROCK_LOG_SAMPLE_WARN(Weapon, 1000, "Trigger equip presentation deadline evidence unavailable; native wait retained");
        }
        trigger_equip_readiness_trace::stage("after-native-preparation");
    }

    const char* requestReasonName(const RequestReason reason) noexcept
    {
        switch (reason) {
        case RequestReason::InventoryEquip:
            return "inventory-equip";
        case RequestReason::SameHandTrigger:
            return "same-hand-trigger";
        case RequestReason::GripZoneSettle:
            return "grip-zone-settle";
        default:
            return "unknown";
        }
    }

    const char* readinessReasonName(const ReadinessReason reason) noexcept
    {
        switch (reason) {
        case ReadinessReason::Ready:
            return "ready";
        case ReadinessReason::NotInstalled:
            return "not-installed";
        case ReadinessReason::UnsupportedRuntime:
            return "unsupported-runtime";
        case ReadinessReason::WrongThread:
            return "wrong-thread";
        case ReadinessReason::MissingPlayer:
            return "missing-player";
        case ReadinessReason::PlayerVtableUnreadable:
            return "player-vtable-unreadable";
        case ReadinessReason::PlayerDrawSlotChanged:
            return "player-draw-slot-changed";
        case ReadinessReason::EntryHookChanged:
            return "entry-hook-changed";
        case ReadinessReason::DrawCallsiteChanged:
            return "draw-callsite-changed";
        case ReadinessReason::SheatheCallsiteChanged:
            return "sheathe-callsite-changed";
        default:
            return "unknown";
        }
    }

    const char* immediateEquipCodeName(const ImmediateEquipCode code) noexcept
    {
        switch (code) {
        case ImmediateEquipCode::Accepted:
            return "accepted";
        case ImmediateEquipCode::MissingInput:
            return "missing-input";
        case ImmediateEquipCode::CapabilityUnavailable:
            return "capability-unavailable";
        case ImmediateEquipCode::NestedScope:
            return "nested-scope";
        case ImmediateEquipCode::ManagerRejected:
            return "manager-rejected";
        case ImmediateEquipCode::InvalidActionTrace:
            return "invalid-action-trace";
        default:
            return "not-attempted";
        }
    }

}

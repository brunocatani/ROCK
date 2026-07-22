#include "physics-interaction/native/HeldWeaponInstantTransition.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/NativeMemory.h"
#include "rock_support/Fo4VrRuntime.h"

#include "RE/Bethesda/Actor.h"
#include "RE/Bethesda/PlayerCharacter.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <intrin.h>

#include <array>
#include <atomic>
#include <cstring>

namespace rock::held_weapon_instant_transition
{
    namespace
    {
        using DrawWeaponMagicHands = void (*)(RE::PlayerCharacter*, bool);
        using CompleteWeaponDraw = void (*)(RE::Actor*);

        constexpr std::uintptr_t kPlayerDrawWeaponEntry = 0x0F78D10;
        constexpr std::uintptr_t kEquipManagerDrawCallsite = 0x0E107A1;
        constexpr std::uintptr_t kEquipManagerDrawReturn = 0x0E107A7;
        constexpr std::uintptr_t kEquipManagerSheatheCallsite = 0x0E10988;
        constexpr std::uintptr_t kEquipManagerSheatheReturn = 0x0E1098E;
        constexpr std::uintptr_t kCompleteWeaponDraw = 0x0DBE590;
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
        constexpr std::array<std::uint8_t, 16> kExpectedCompleteWeaponDrawEntry{
            0x40, 0x53, 0x48, 0x83, 0xEC, 0x30, 0x48, 0x8B,
            0xD9, 0x48, 0x8B, 0x89, 0x00, 0x03, 0x00, 0x00,
        };

        struct TransactionScope
        {
            RE::PlayerCharacter* player{ nullptr };
            RequestReason reason{ RequestReason::SameHandTrigger };
            held_weapon_instant_transition_policy::ActionTrace trace{};
        };

        struct CompletionPermitState
        {
            std::uint64_t id{ 0 };
            RE::PlayerCharacter* player{ nullptr };
            RequestReason reason{ RequestReason::SameHandTrigger };
        };

        DrawWeaponMagicHands s_originalDrawWeaponMagicHands = nullptr;
        std::atomic<bool> s_installed{ false };
        std::atomic<DWORD> s_ownerThreadId{ 0 };
        thread_local TransactionScope* t_activeScope = nullptr;
        thread_local CompletionPermitState t_completionPermit{};
        thread_local std::uint64_t t_nextCompletionPermit{ 1 };

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

        [[nodiscard]] bool equippedIdentityMatches(
            RE::PlayerCharacter* player,
            const EquippedIdentity& expected) noexcept
        {
            if (!player || expected.formID == 0) {
                return false;
            }

            auto* equipped = f4vr::getEquippedItem();
            auto* object = equipped ? equipped->item.object : nullptr;
            auto* instanceData = equipped ? equipped->item.instanceData.get() : nullptr;
            return object &&
                   object->formType == RE::ENUM_FORM_ID::kWEAP &&
                   object->formID == expected.formID &&
                   reinterpret_cast<std::uintptr_t>(instanceData) == expected.instanceData &&
                   equipped->equipIndex.index == expected.equipIndex;
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

        [[nodiscard]] std::uint64_t issueCompletionPermit(
            RE::PlayerCharacter* player,
            const RequestReason reason) noexcept
        {
            auto permit = t_nextCompletionPermit++;
            if (permit == 0) {
                permit = t_nextCompletionPermit++;
            }
            t_completionPermit = CompletionPermitState{
                .id = permit,
                .player = player,
                .reason = reason,
            };
            return permit;
        }

        void invalidateCompletionPermit() noexcept
        {
            t_completionPermit = {};
        }

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
        if (!bytesMatch(kCompleteWeaponDraw, kExpectedCompleteWeaponDrawEntry)) {
            ROCK_LOG_ERROR(Init,
                "Held weapon instant transition unavailable: native draw completion entry changed");
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
        if (!bytesMatch(kCompleteWeaponDraw, kExpectedCompleteWeaponDrawEntry)) {
            return { .reason = ReadinessReason::CompletionEntryChanged };
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
        invalidateCompletionPermit();
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

        if (!result.managerAccepted) {
            result.code = ImmediateEquipCode::ManagerRejected;
            return result;
        }
        if (!held_weapon_instant_transition_policy::isValidCompletionTrace(
                result.actionTrace)) {
            result.code = ImmediateEquipCode::InvalidActionTrace;
            return result;
        }

        result.completionPermit = issueCompletionPermit(input.player, input.reason);
        result.code = ImmediateEquipCode::Accepted;
        return result;
    }

    void discardCompletionPermit(const ImmediateEquipResult& equip) noexcept
    {
        if (equip.completionPermit != 0 &&
            t_completionPermit.id == equip.completionPermit) {
            invalidateCompletionPermit();
        }
    }

    CompletionResult completeDrawForExactCurrent(
        const ImmediateEquipResult& equip,
        const EquippedIdentity& expected) noexcept
    {
        CompletionResult result{};
        auto* player = f4vr::getPlayer();
        if (!player) {
            invalidateCompletionPermit();
            result.code = CompletionCode::MissingPlayer;
            return result;
        }

        const auto readiness = readinessFor(player);
        result.readinessReason = readiness.reason;
        if (!readiness.ready) {
            invalidateCompletionPermit();
            result.code = CompletionCode::CapabilityUnavailable;
            return result;
        }

        const bool authorized = equip.success() &&
            t_completionPermit.id != 0 &&
            t_completionPermit.id == equip.completionPermit &&
            t_completionPermit.player == player &&
            t_completionPermit.reason == equip.reason;
        invalidateCompletionPermit();
        if (!authorized) {
            result.code = CompletionCode::UnauthorizedTransaction;
            return result;
        }
        if (!equippedIdentityMatches(player, expected)) {
            result.code = CompletionCode::IdentityChangedBeforeCompletion;
            return result;
        }

        result.stateBefore = static_cast<std::uint32_t>(player->weaponState);
        result.stateAfter = result.stateBefore;
        if (!held_weapon_instant_transition_policy::isStableWeaponState(
                result.stateBefore)) {
            result.code = CompletionCode::InvalidWeaponState;
            return result;
        }

        const auto completeDraw = reinterpret_cast<CompleteWeaponDraw>(
            REL::Offset(kCompleteWeaponDraw).address());
        completeDraw(player);
        result.stateAfter = static_cast<std::uint32_t>(player->weaponState);

        if (!equippedIdentityMatches(player, expected)) {
            result.code = CompletionCode::IdentityChangedAfterCompletion;
            return result;
        }
        if (result.stateAfter != 3) {
            result.code = CompletionCode::NativeCompletionFailed;
            return result;
        }
        result.code = CompletionCode::Completed;
        return result;
    }

    const char* requestReasonName(const RequestReason reason) noexcept
    {
        switch (reason) {
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
        case ReadinessReason::CompletionEntryChanged:
            return "completion-entry-changed";
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

    const char* completionCodeName(const CompletionCode code) noexcept
    {
        switch (code) {
        case CompletionCode::Completed:
            return "completed";
        case CompletionCode::CapabilityUnavailable:
            return "capability-unavailable";
        case CompletionCode::UnauthorizedTransaction:
            return "unauthorized-transaction";
        case CompletionCode::MissingPlayer:
            return "missing-player";
        case CompletionCode::IdentityChangedBeforeCompletion:
            return "identity-changed-before-completion";
        case CompletionCode::IdentityChangedAfterCompletion:
            return "identity-changed-after-completion";
        case CompletionCode::InvalidWeaponState:
            return "invalid-weapon-state";
        case CompletionCode::NativeCompletionFailed:
            return "native-completion-failed";
        default:
            return "not-attempted";
        }
    }
}

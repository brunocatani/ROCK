#pragma once

#include "physics-interaction/native/HeldWeaponInstantTransitionPolicy.h"

#include <cstdint>

namespace RE
{
    class ActorEquipManager;
    class BGSEquipSlot;
    class BGSObjectInstance;
    class PlayerCharacter;
}

namespace rock::held_weapon_instant_transition
{
    enum class RequestReason : std::uint8_t
    {
        SameHandTrigger,
        GripZoneSettle,
    };

    enum class ReadinessReason : std::uint8_t
    {
        Ready,
        NotInstalled,
        UnsupportedRuntime,
        WrongThread,
        MissingPlayer,
        PlayerVtableUnreadable,
        PlayerDrawSlotChanged,
        EntryHookChanged,
        DrawCallsiteChanged,
        SheatheCallsiteChanged,
        CompletionEntryChanged,
    };

    struct Readiness
    {
        bool ready{ false };
        ReadinessReason reason{ ReadinessReason::NotInstalled };
    };

    enum class ImmediateEquipCode : std::uint8_t
    {
        NotAttempted,
        Accepted,
        MissingInput,
        CapabilityUnavailable,
        NestedScope,
        ManagerRejected,
        InvalidActionTrace,
    };

    struct ImmediateEquipInput
    {
        RE::ActorEquipManager* manager{ nullptr };
        RE::PlayerCharacter* player{ nullptr };
        const RE::BGSObjectInstance* object{ nullptr };
        std::uint32_t stackID{ 0 };
        const RE::BGSEquipSlot* equipSlot{ nullptr };
        RequestReason reason{ RequestReason::SameHandTrigger };
    };

    struct ImmediateEquipResult
    {
        ImmediateEquipCode code{ ImmediateEquipCode::NotAttempted };
        ReadinessReason readinessReason{ ReadinessReason::NotInstalled };
        RequestReason reason{ RequestReason::SameHandTrigger };
        held_weapon_instant_transition_policy::ActionTrace actionTrace{};
        std::uint64_t completionPermit{ 0 };
        bool attempted{ false };
        bool managerAccepted{ false };

        [[nodiscard]] bool success() const noexcept
        {
            return code == ImmediateEquipCode::Accepted && completionPermit != 0;
        }
    };

    struct EquippedIdentity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t instanceData{ 0 };
        std::uint32_t equipIndex{ 0 };
    };

    enum class CompletionCode : std::uint8_t
    {
        NotAttempted,
        Completed,
        CapabilityUnavailable,
        UnauthorizedTransaction,
        MissingPlayer,
        IdentityChangedBeforeCompletion,
        IdentityChangedAfterCompletion,
        InvalidWeaponState,
        NativeCompletionFailed,
    };

    struct CompletionResult
    {
        CompletionCode code{ CompletionCode::NotAttempted };
        ReadinessReason readinessReason{ ReadinessReason::NotInstalled };
        std::uint32_t stateBefore{ 0 };
        std::uint32_t stateAfter{ 0 };

        [[nodiscard]] bool success() const noexcept
        {
            return code == CompletionCode::Completed;
        }
    };

    // Optional startup capability. Failure disables only held trigger/grip-zone
    // equip before the loose reference is released; ordinary native calls pass
    // through unchanged.
    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] Readiness readinessFor(RE::PlayerCharacter* player) noexcept;

    // Main-thread-only transaction. The scope surrounds exactly one immediate
    // EquipObject call and suppresses only its verified player draw/sheathe
    // virtual callsites.
    [[nodiscard]] ImmediateEquipResult equipImmediatelyWithoutActions(
        const ImmediateEquipInput& input) noexcept;
    void discardCompletionPermit(const ImmediateEquipResult& equip) noexcept;

    // Consumes the one-shot permit returned by the immediately preceding exact
    // transaction and runs the verified engine-owned draw completion helper.
    [[nodiscard]] CompletionResult completeDrawForExactCurrent(
        const ImmediateEquipResult& equip,
        const EquippedIdentity& expected) noexcept;

    [[nodiscard]] const char* requestReasonName(RequestReason reason) noexcept;
    [[nodiscard]] const char* readinessReasonName(ReadinessReason reason) noexcept;
    [[nodiscard]] const char* immediateEquipCodeName(ImmediateEquipCode code) noexcept;
    [[nodiscard]] const char* completionCodeName(CompletionCode code) noexcept;
}

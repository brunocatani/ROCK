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
        bool attempted{ false };
        bool managerAccepted{ false };

        [[nodiscard]] bool success() const noexcept
        {
            return code == ImmediateEquipCode::Accepted;
        }
    };

    // Optional startup capability. Failure disables only held trigger/grip-zone
    // equip before the loose reference is released; ordinary native calls pass
    // through unchanged.
    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] Readiness readinessFor(RE::PlayerCharacter* player) noexcept;

    // Main-thread-only transaction. The scope surrounds exactly one immediate
    // EquipObject call and suppresses only its verified player draw/sheathe
    // virtual callsites. Exact equipped identity/stack validation belongs to
    // WeaponEquipTransfer; normal native draw presentation belongs to the
    // equipped-weapon transition coordinator.
    [[nodiscard]] ImmediateEquipResult equipImmediatelyWithoutActions(
        const ImmediateEquipInput& input) noexcept;

    [[nodiscard]] const char* requestReasonName(RequestReason reason) noexcept;
    [[nodiscard]] const char* readinessReasonName(ReadinessReason reason) noexcept;
    [[nodiscard]] const char* immediateEquipCodeName(ImmediateEquipCode code) noexcept;
}

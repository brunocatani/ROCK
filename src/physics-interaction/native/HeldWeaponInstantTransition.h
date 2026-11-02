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
        InventoryEquip,
        KeptDrawnUnequip,
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

        [[nodiscard]] bool success() const noexcept { return code == ImmediateEquipCode::Accepted; }
    };

    // Optional startup capability. Failure disables only held trigger/grip-zone
    // equip before the loose reference is released; ordinary native calls pass
    // through unchanged.
    [[nodiscard]] bool install() noexcept;
    [[nodiscard]] Readiness readinessFor(RE::PlayerCharacter* player) noexcept;

    // Game-thread native draw preparation for every player equip draw:
    // keeps pre-existing combat cooldowns instead of adding the equip delay
    // and makes the native presentation deadline due now.
    void prepareInstantDraw(RE::PlayerCharacter* player) noexcept;
    // After a native preparation added them: restores the pre-call attack
    // cooldown and makes the presentation deadline due now.
    void removeAddedPresentationGates(RE::PlayerCharacter* player, float priorCooldown, const char* source) noexcept;

    // Main-thread-only transaction. The scope surrounds exactly one immediate
    // EquipObject call and suppresses only its verified player draw/sheathe
    // virtual callsites. Exact equipped identity/stack validation belongs to
    // WeaponEquipTransfer; normal native draw presentation belongs to the
    // equipped-weapon transition coordinator.
    [[nodiscard]] ImmediateEquipResult equipImmediatelyWithoutActions(const ImmediateEquipInput& input) noexcept;

    /*
     * Main-thread-only transaction around exactly one immediate unequip that
     * leaves the other index's weapon equipped. FO4VR's unequip (140E10820)
     * sheathes the drawn player for every weapon but grenades and mines
     * (140E1094E..140E10988, the verified sheathe callsite; its drawn test
     * 140F0A130 returns true in VR). That sheathe would take the weapon that
     * stays in the other hand: its fire and the next equip wait out a native
     * sheathe and redraw. Only that callsite is suppressed; the unequip itself
     * stays native. The call reports nothing: the caller reads the result.
     */
    using KeptDrawnUnequipCall = void (*)(void* context) noexcept;

    struct KeptDrawnUnequipResult
    {
        ImmediateEquipCode code{ ImmediateEquipCode::NotAttempted };
        ReadinessReason readinessReason{ ReadinessReason::NotInstalled };
        held_weapon_instant_transition_policy::ActionTrace actionTrace{};
    };

    [[nodiscard]] KeptDrawnUnequipResult unequipKeepingDrawn(RE::PlayerCharacter* player, KeptDrawnUnequipCall call,
        void* context) noexcept;

    [[nodiscard]] const char* requestReasonName(RequestReason reason) noexcept;
    [[nodiscard]] const char* readinessReasonName(ReadinessReason reason) noexcept;
    [[nodiscard]] const char* immediateEquipCodeName(ImmediateEquipCode code) noexcept;
}

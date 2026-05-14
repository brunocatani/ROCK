#pragma once

/*
 * ROCK needs an equipped-instance witness that does not depend on FO4VR's
 * stale first-person weapon visual tree. This runtime records bounded engine
 * signals that are earlier than generated visual publication: game-load
 * baseline, player inventory equips, attach-mod mutation hints, stack writer
 * commits, and direct BGSObjectInstanceExtra OMOD mutations. The object-extra
 * path exists because FO4VR can mutate the already-equipped extra data without
 * routing through the stack functors that earlier hooks observed.
 * The witness is intentionally bounded: it is evidence for a pending native
 * visual remap window, not a permanent replacement for FO4VR's equipped state.
 * It never builds collision, mutates weapon mods, or queues native visual work;
 * the existing WeaponVisualRemapRuntime remains the only native remap boundary.
 */

#include "physics-interaction/weapon/WeaponAuthority.h"

#include <cstdint>
#include <string_view>

namespace rock::weapon_instance_witness_runtime
{
    enum class WeaponInstanceWitnessSource : std::uint8_t
    {
        None,
        GameLoadBaseline,
        InventoryEquip,
        AttachMod,
        WorkbenchApplyChanges,
    };

    struct AuthoritativeEquippedWeaponWitness
    {
        WeaponInstanceWitnessSource source{ WeaponInstanceWitnessSource::None };
        std::uint64_t sequence{ 0 };
        std::uint32_t weaponFormID{ 0 };
        std::uintptr_t weaponFormAddress{ 0 };
        std::uintptr_t instanceDataAddress{ 0 };
        std::uintptr_t instanceKeywordDataAddress{ 0 };
        std::uintptr_t objectInstanceExtraAddress{ 0 };
        std::uintptr_t equippedDataAddress{ 0 };
        std::uintptr_t equippedObjectAddress{ 0 };
        std::uint64_t instanceContentKey{ 0 };
        std::uint64_t objectIndexDataSignature{ 0 };
        std::uint32_t objectIndexDataCount{ 0 };
        std::uint32_t activeModCount{ 0 };
        std::uint32_t disabledModCount{ 0 };
        std::uint32_t mutatingModFormID{ 0 };
        std::uint32_t equipIndex{ 0 };
        std::uint32_t stackIndex{ 0 };
        std::uint16_t uniqueID{ 0 };
        std::uint8_t attachIndex{ 0 };
        std::uint8_t rank{ 0 };
        bool mutatingModRemoved{ false };
        std::string_view displayName{};
        std::uint64_t signature{ 0 };
    };

    [[nodiscard]] const char* sourceName(WeaponInstanceWitnessSource source) noexcept;
    [[nodiscard]] bool installWeaponInstanceWitnessRuntime();
    void noteGameSessionReset(const char* reason);
    void noteGameLoadWeaponWitnessBaseline();

    [[nodiscard]] bool tryGetAuthoritativeEquippedWeaponIdentity(
        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity& outIdentity,
        AuthoritativeEquippedWeaponWitness* outWitness = nullptr);

    [[nodiscard]] bool tryGetAuthoritativeEquippedWeaponWitness(
        std::uint64_t expectedSignature,
        AuthoritativeEquippedWeaponWitness& outWitness);

    [[nodiscard]] bool nativeVisualRemapAllowedForWitness(
        std::uint64_t expectedSignature,
        const char*& outReason);

    void clearAuthoritativeEquippedWeaponWitness(
        std::uint64_t settledSignature,
        const char* reason);
}

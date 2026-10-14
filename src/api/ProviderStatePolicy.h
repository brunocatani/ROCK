#pragma once

#include "api/HandInteractionPublication.h"
#include "api/ProviderRuntimeTypes.h"

#include <algorithm>
#include <cstring>

namespace rock::provider_state_policy
{
    using namespace rock::provider;

    // Clear reusable output on failure without writing beyond a caller's ABI
    // prefix or changing the size/version that the entrypoint must validate.
    template <class T>
    void clearQueryOutput(T* output, std::uint32_t minimumSize = sizeof(T)) noexcept
    {
        if (!output || output->size < minimumSize) return;
        T empty{};
        empty.size = output->size;
        if constexpr (requires { empty.version; }) empty.version = output->version;
        std::memcpy(output, &empty, (std::min<std::size_t>)(output->size, sizeof(T)));
    }

    [[nodiscard]] constexpr bool firingGripOccupied(bool weaponPresent, bool partCarry, bool surfaceSupported = false) noexcept
    {
        return weaponPresent && !partCarry && !surfaceSupported;
    }

    // ROCK's live handling of the weapon in slot 0, the one weapon it keeps
    // state for. Private; the WeaponState hands and handling flags derive
    // from it.
    struct WeaponHandling
    {
        bool weaponPresent{ false };
        // A hand holds the firing grip, native carry included; clear while
        // the firing hand carries the weapon by a part.
        bool firingGripOccupied{ false };
        bool manualOwnership{ false };
        bool partCarry{ false };
        bool leftFiringAvailable{ false };
        bool twoHandGrip{ false };
        api::Hand firingHand{ api::Hand::Right };
        std::uint32_t weaponFormId{ 0 };
        std::uint64_t weaponGenerationKey{ 0 };
    };

    // Fills a WeaponState's hands and handling flags; both stay empty without
    // a present weapon.
    inline void applyHandling(api::weapon::WeaponState& state, const WeaponHandling& handling) noexcept
    {
        if (!handling.weaponPresent) return;
        using Flag = api::weapon::HandlingStateFlag;
        const auto add = [&state](const bool enabled, const Flag flag) {
            if (enabled) state.handlingFlags |= static_cast<std::uint32_t>(flag);
        };
        add(handling.firingGripOccupied, Flag::FiringGripHeld);
        add(handling.manualOwnership, Flag::ManualOwnership);
        add(handling.partCarry, Flag::PartCarry);
        add(handling.leftFiringAvailable, Flag::LeftFiringAvailable);
        state.firingHand = handling.firingHand;
        if (handling.twoHandGrip)
            state.supportHand = handling.firingHand == api::Hand::Left ? api::Hand::Right : api::Hand::Left;
    }

    [[nodiscard]] constexpr bool handHolding(const provider::HandInteractionPublication& state) noexcept
    {
        using Flag = api::grab::HandInteractionFlag;
        using Phase = api::grab::HandInteractionPhase;
        if (!(state.flags & static_cast<std::uint32_t>(Flag::Valid))) return false;
        return state.phase == Phase::Holding || state.phase == Phase::StashCandidate ||
               state.phase == Phase::ConsumeCandidate;
    }

    // Detailed captured grips remain ROCK-owned. Generic hand occupancy also
    // includes native equipped carry; stale selection metadata must not leak
    // into that occupied hand and masquerade as a held loose reference.
    inline void applyWeaponOccupancy(provider::HandInteractionPublication& state,
        const WeaponHandling& handling,
        const api::weaponparts::GripState& grip) noexcept
    {
        using Flag = api::grab::HandInteractionFlag;
        if (!(state.flags & static_cast<std::uint32_t>(Flag::Valid)) || !handling.weaponPresent) return;
        const bool firing = state.hand == handling.firingHand && handling.firingGripOccupied;
        if (!firing && !grip.active) return;

        state.phase = api::grab::HandInteractionPhase::Holding;
        state.flags &= ~(static_cast<std::uint32_t>(Flag::LooseObject) |
                         static_cast<std::uint32_t>(Flag::LooseWeapon) |
                         static_cast<std::uint32_t>(Flag::HeldBodyListTruncated));
        state.targetKind = api::grab::BodyContactTargetKind::Weapon;
        state.targetFormId = handling.weaponFormId;
        state.reservedTargetIdentity = handling.weaponGenerationKey;
        state.primaryBodyId = grip.active ? grip.bodyId : 0x7FFF'FFFF;
        state.heldBodyCount = 0;
        std::fill_n(state.heldBodyIds, ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1, 0u);
        if (grip.active) state.flags |= static_cast<std::uint32_t>(Flag::RockGripActive);
        if (firing) {
            state.flags |= static_cast<std::uint32_t>(Flag::FiringGrip);
            if (!grip.active) state.flags |= static_cast<std::uint32_t>(Flag::NativeWeaponCarry);
        } else if (grip.gripKind == api::weaponparts::GripKind::PartCarry) {
            state.flags |= static_cast<std::uint32_t>(Flag::PartCarry);
        } else {
            state.flags |= static_cast<std::uint32_t>(Flag::PartGrip);
        }
        if (grip.attachOnly) state.flags |= static_cast<std::uint32_t>(Flag::AttachOnly);
    }

    [[nodiscard]] constexpr std::uint32_t colliderFlags(
        bool lifecycleAllowed, bool filterKnown, bool suppressed) noexcept
    {
        using Flag = api::collision::PlayerColliderFlag;
        std::uint32_t flags = static_cast<std::uint32_t>(Flag::Valid);
        if (lifecycleAllowed) flags |= static_cast<std::uint32_t>(Flag::LifecycleAllowed);
        if (filterKnown) flags |= static_cast<std::uint32_t>(Flag::FilterKnown);
        if (filterKnown && suppressed) flags |= static_cast<std::uint32_t>(Flag::CollisionSuppressed);
        if (lifecycleAllowed && filterKnown && !suppressed) flags |= static_cast<std::uint32_t>(Flag::Enabled);
        return flags;
    }
}

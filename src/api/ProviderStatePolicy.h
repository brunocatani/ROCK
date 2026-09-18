#pragma once

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

    [[nodiscard]] constexpr bool firingGripOccupied(bool weaponPresent, bool partCarry) noexcept
    {
        return weaponPresent && !partCarry;
    }

    [[nodiscard]] constexpr bool handHolding(const RockProviderHandInteractionStateV1& state) noexcept
    {
        using Flag = RockProviderHandInteractionFlagV1;
        using Phase = RockProviderHandInteractionPhaseV1;
        if (!(state.flags & static_cast<std::uint32_t>(Flag::Valid))) return false;
        return state.phase == Phase::Holding || state.phase == Phase::StashCandidate ||
               state.phase == Phase::ConsumeCandidate;
    }

    [[nodiscard]] constexpr std::uint32_t handStateFlags(
        const RockProviderHandInteractionStateV1& state, bool physicsDisabled) noexcept
    {
        using Flag = RockProviderHandStateFlag;
        std::uint32_t flags = physicsDisabled ? static_cast<std::uint32_t>(Flag::PhysicsDisabled) : 0;
        if (!(state.flags & static_cast<std::uint32_t>(RockProviderHandInteractionFlagV1::Valid))) return flags;
        flags |= static_cast<std::uint32_t>(Flag::Valid);
        if (handHolding(state)) flags |= static_cast<std::uint32_t>(Flag::Holding);
        if (state.phase == RockProviderHandInteractionPhaseV1::Touching)
            flags |= static_cast<std::uint32_t>(Flag::Touching);
        return flags;
    }

    // Detailed captured grips remain ROCK-owned. Generic hand occupancy also
    // includes native equipped carry; stale selection metadata must not leak
    // into that occupied hand and masquerade as a held loose reference.
    inline void applyWeaponOccupancy(RockProviderHandInteractionStateV1& state,
        const RockProviderEquippedWeaponHandlingStateV1& handling,
        const RockProviderWeaponPartGripStateV1& grip) noexcept
    {
        using Flag = RockProviderHandInteractionFlagV1;
        using Runtime = RockProviderEquippedWeaponHandlingRuntimeFlagV1;
        if (!(state.flags & static_cast<std::uint32_t>(Flag::Valid)) ||
            !(handling.runtimeFlags & static_cast<std::uint32_t>(Runtime::WeaponPresent))) return;
        const bool firing = state.hand == handling.currentFiringHand &&
            (handling.runtimeFlags & static_cast<std::uint32_t>(Runtime::FiringGripOccupied));
        if (!firing && !grip.active) return;

        state.phase = RockProviderHandInteractionPhaseV1::Holding;
        state.flags &= ~(static_cast<std::uint32_t>(Flag::LooseObject) |
                         static_cast<std::uint32_t>(Flag::LooseWeapon) |
                         static_cast<std::uint32_t>(Flag::HeldBodyListTruncated));
        state.targetKind = RockProviderBodyContactTargetKind::Weapon;
        state.targetFormId = handling.weaponFormId;
        state.reservedTargetIdentity = handling.weaponGenerationKey;
        state.primaryBodyId = grip.active ? grip.bodyId : 0x7FFF'FFFF;
        state.heldBodyCount = 0;
        std::fill_n(state.heldBodyIds, ROCK_PROVIDER_MAX_HAND_HELD_BODIES_V1, 0u);
        if (grip.active) state.flags |= static_cast<std::uint32_t>(Flag::RockGripActive);
        if (firing) {
            state.flags |= static_cast<std::uint32_t>(Flag::FiringGrip);
            if (!grip.active) state.flags |= static_cast<std::uint32_t>(Flag::NativeWeaponCarry);
        } else if (grip.gripKind == RockProviderWeaponPartGripKindV1::PartCarry) {
            state.flags |= static_cast<std::uint32_t>(Flag::PartCarry);
        } else {
            state.flags |= static_cast<std::uint32_t>(Flag::PartGrip);
        }
        if (grip.attachOnly) state.flags |= static_cast<std::uint32_t>(Flag::AttachOnly);
    }

    [[nodiscard]] constexpr std::uint32_t colliderFlags(
        bool lifecycleAllowed, bool filterKnown, bool suppressed) noexcept
    {
        using Flag = RockProviderPlayerColliderFlagV1;
        std::uint32_t flags = static_cast<std::uint32_t>(Flag::Valid);
        if (lifecycleAllowed) flags |= static_cast<std::uint32_t>(Flag::LifecycleAllowed);
        if (filterKnown) flags |= static_cast<std::uint32_t>(Flag::FilterKnown);
        if (filterKnown && suppressed) flags |= static_cast<std::uint32_t>(Flag::CollisionSuppressed);
        if (lifecycleAllowed && filterKnown && !suppressed) flags |= static_cast<std::uint32_t>(Flag::Enabled);
        return flags;
    }
}

#pragma once

#include "physics-interaction/weapon/equip/HeldWeaponEquipStatePolicy.h"

#include <cstdint>

namespace rock::weapon_transition_animation_acceleration_policy
{
    constexpr float kAcceleratedSpeedMultiplier = 100.0f;
    constexpr float kLeaseTimeoutSeconds = 2.0f;

    enum class Direction : std::uint8_t
    {
        Draw,
        Sheathe,
    };

    enum class LifecycleAction : std::uint8_t
    {
        KeepPending,
        Accelerate,
        Complete,
        Cancel,
    };

    [[nodiscard]] inline constexpr float scaleTransitionTimestep(
        const float timestep) noexcept
    {
        return timestep * kAcceleratedSpeedMultiplier;
    }

    [[nodiscard]] inline constexpr std::uint32_t transitionState(
        const Direction direction) noexcept
    {
        using NativeWeaponState =
            held_weapon_equip_state_policy::NativeWeaponState;
        return static_cast<std::uint32_t>(
            direction == Direction::Draw ?
                NativeWeaponState::Drawing :
                NativeWeaponState::Sheathing);
    }

    [[nodiscard]] inline constexpr std::uint32_t terminalState(
        const Direction direction) noexcept
    {
        using NativeWeaponState =
            held_weapon_equip_state_policy::NativeWeaponState;
        return static_cast<std::uint32_t>(
            direction == Direction::Draw ?
                NativeWeaponState::Drawn :
                NativeWeaponState::Sheathed);
    }

    [[nodiscard]] inline constexpr Direction directionForTransitionState(
        const std::uint32_t nativeState) noexcept
    {
        return nativeState == transitionState(Direction::Sheathe) ?
            Direction::Sheathe :
            Direction::Draw;
    }

    [[nodiscard]] inline constexpr bool shouldAccelerateSample(
        const bool registeredTransitionClip,
        const Direction direction,
        const std::uint32_t nativeState) noexcept
    {
        return registeredTransitionClip &&
               nativeState == transitionState(direction);
    }

    [[nodiscard]] inline constexpr LifecycleAction classifyLifecycle(
        const bool runtimeAllowed,
        const bool identityMatches,
        const Direction direction,
        const std::uint32_t nativeState,
        const float elapsedSeconds) noexcept
    {
        if (!runtimeAllowed || !identityMatches ||
            !held_weapon_equip_state_policy::isValidNativeWeaponState(
                nativeState)) {
            return LifecycleAction::Cancel;
        }
        if (nativeState == terminalState(direction)) {
            return LifecycleAction::Complete;
        }
        if (elapsedSeconds >= kLeaseTimeoutSeconds) {
            return LifecycleAction::Cancel;
        }
        if (nativeState == transitionState(direction)) {
            return LifecycleAction::Accelerate;
        }
        return LifecycleAction::KeepPending;
    }
}

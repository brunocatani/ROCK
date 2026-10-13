#pragma once

#include "physics-interaction/weapon/HeldWeaponEquipStatePolicy.h"
#include "physics-interaction/native/NativeWeaponAnimationAudio.h"

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

    [[nodiscard]] inline constexpr bool isDrawClip(std::string_view path) noexcept
    {
        const auto slash = path.find_last_of("/\\");
        if (slash != std::string_view::npos) path.remove_prefix(slash + 1);
        if (!native_weapon_animation_audio::isEquipClip(path)) return false;
        path.remove_suffix(4);
        if (path.size() >= 3 && native_weapon_animation_audio::equalName(path.substr(0,3), "WPN")) path.remove_prefix(3);
        return native_weapon_animation_audio::equalName(path,"Equip") ||
            native_weapon_animation_audio::equalName(path,"EquipFast");
    }

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
               (nativeState == transitionState(direction) ||
                (direction == Direction::Draw && nativeState == terminalState(direction)));
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
            // HK417 capture: Drawn at 19 ms, replacement WPNEquipFast starts
            // at 41 ms and runs 1.5 seconds if we drop the lease here. Keep
            // the existing bounded identity lease to admit delayed draw clips.
            // Registration filters draw filenames; idle/fire/reload never enter.
            if (direction == Direction::Draw && elapsedSeconds < kLeaseTimeoutSeconds)
                return LifecycleAction::KeepPending;
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

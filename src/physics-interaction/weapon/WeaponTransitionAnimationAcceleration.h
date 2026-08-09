#pragma once

#include "physics-interaction/weapon/WeaponTransitionAnimationAccelerationPolicy.h"

#include <cstdint>

namespace RE
{
    class PlayerCharacter;
}

namespace rock::weapon_transition_animation_acceleration
{
    struct Identity
    {
        std::uint32_t formID{ 0 };
        std::uintptr_t instanceData{ 0 };
        std::uint32_t equipIndex{ 0 };

        [[nodiscard]] bool valid() const noexcept
        {
            return formID != 0;
        }

        [[nodiscard]] bool operator==(const Identity&) const noexcept = default;
    };

    enum class RequestResult : std::uint8_t
    {
        Armed,
        AlreadyArmed,
        NotInstalled,
        MissingInput,
        WrongThread,
        UnencodablePlayer,
    };

    struct RequestInput
    {
        RE::PlayerCharacter* player{ nullptr };
        Identity identity{};
        weapon_transition_animation_acceleration_policy::Direction direction{
            weapon_transition_animation_acceleration_policy::Direction::Draw
        };
    };

    struct ServiceInput
    {
        RE::PlayerCharacter* player{ nullptr };
        Identity identity{};
        std::uint32_t nativeWeaponState{ 0 };
        bool runtimeAllowed{ false };
    };

    // Optional FO4VR 1.2.72 capability. A failed or partial install remains
    // inert and leaves native weapon animation speed untouched.
    [[nodiscard]] bool install() noexcept;

    // Game-thread request/service API. The animation evaluator may run on a
    // different engine thread; it consumes only the atomic encoded lease and
    // writes the output field owned by the channel currently being evaluated.
    [[nodiscard]] RequestResult request(const RequestInput& input) noexcept;
    void service(const ServiceInput& input) noexcept;
    void cancel(const char* reason) noexcept;
}

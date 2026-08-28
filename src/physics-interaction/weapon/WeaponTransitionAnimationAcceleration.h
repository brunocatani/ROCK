#pragma once

#include "physics-interaction/weapon/WeaponTransitionAnimationAccelerationPolicy.h"

#include <array>
#include <cstddef>
#include <cstdint>

namespace RE
{
    class PlayerCharacter;
}

namespace rock::weapon_transition_animation_acceleration
{
    inline constexpr std::size_t kMaximumActivationEvidenceClips = 32;

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
        MissingGraphManager,
        MissingGraphCharacters,
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

    struct ActivationEvidence
    {
        std::uint64_t sequence{ 0 };
        std::uint32_t matchedActivations{ 0 };
        std::uint32_t registeredUpdates{ 0 };
        std::uint32_t activeClips{ 0 };
        std::uint32_t updatedActiveClips{ 0 };
        std::array<std::uintptr_t, kMaximumActivationEvidenceClips>
            activeClipIdentities{};
        std::array<std::uintptr_t, kMaximumActivationEvidenceClips>
            updatedActiveClipIdentities{};
        bool exactLease{ false };

        [[nodiscard]] bool hasActiveUpdatedClips() const noexcept
        {
            return exactLease && matchedActivations > 0 &&
                   registeredUpdates > 0 && activeClips > 0 &&
                   updatedActiveClips > 0;
        }

        [[nodiscard]] bool hasUpdatedClipFrom(
            const ActivationEvidence& submission) const noexcept
        {
            for (const auto clip : updatedActiveClipIdentities) {
                if (clip == 0) {
                    continue;
                }
                for (const auto submittedClip :
                     submission.activeClipIdentities) {
                    if (clip == submittedClip) {
                        return true;
                    }
                }
            }
            return false;
        }
    };

    // Optional FO4VR 1.2.72 capability. A failed or partial install remains
    // inert and leaves native clip time advancement untouched.
    [[nodiscard]] bool install() noexcept;

    // Game-thread request/service API. Animation callbacks may run on a
    // different engine thread; they consume only bounded atomic snapshots of
    // the lease, player graph characters, and clips activated by that lease.
    [[nodiscard]] RequestResult request(const RequestInput& input) noexcept;
    [[nodiscard]] ActivationEvidence observeExactLeaseActivation(
        RE::PlayerCharacter* player,
        const Identity& identity,
        weapon_transition_animation_acceleration_policy::Direction direction) noexcept;
    void service(const ServiceInput& input) noexcept;
    void cancel(const char* reason) noexcept;
}

#pragma once

#include <ROCK/Collision.h>
#include <ROCK/Core.h>
#include <ROCK/Grab.h>

#include <cstdint>

namespace rock::provider
{
    // Optional values one publication carries.
    enum class PublicationFlag : std::uint32_t
    {
        HmdTransformValid = 1u << 0,
        HmdForwardValid = 1u << 1,
        RightHandTransformValid = 1u << 2,
        LeftHandTransformValid = 1u << 3,
    };

    // ROCK's per-frame publication. getSnapshot, frame callbacks and the
    // copied provider reads derive their values from the latest one. Private
    // to ROCK; it never crosses the DLL boundary.
    struct Publication
    {
        std::uint64_t frameIndex{ 0 };
        // Advances when lifecycle, hand, weapon, collision or hand-role state
        // changes; the Sample's stateSequence.
        std::uint64_t stateSequence{ 0 };
        std::uint32_t worldGeneration{ 0 };
        std::uint32_t skeletonGeneration{ 0 };
        std::uint32_t providerGeneration{ 0 };
        std::uint32_t collisionGeneration{ 0 };
        std::uint32_t lifecycleFlags{ 0 };
        api::core::LifecycleReason lastLifecycleReason{ api::core::LifecycleReason::None };
        std::uint32_t frikSkeletonReady{ 0 };
        std::uint32_t menuBlocking{ 0 };
        std::uint32_t configBlocking{ 0 };
        std::uint32_t providerReady{ 0 };
        std::uint32_t stableFrameCount{ 0 };
        // Zero for an unmeasurable frame, never a nominal rate.
        float deltaSeconds{ 0.0f };
        std::uint32_t flags{ 0 };
        float gameToHavokScale{ 0.0f };
        float havokToGameScale{ 0.0f };
        std::uint32_t physicsScaleRevision{ 0 };
        // Weapon slot 0, the one weapon ROCK keeps state for.
        std::uint32_t weaponFormId{ 0 };
        api::collision::WeaponBodies weapon{};
        // The controller hands ROCK interacts with, not the rendered bones.
        api::Transform rightHandTransform{};
        api::Transform leftHandTransform{};
        api::Transform hmdTransform{};
        float hmdForwardWorld[3]{};
        api::Hand primaryHand{ api::Hand::Right };
        api::Hand offhandHand{ api::Hand::Left };
        api::grab::HandReservationKind handReservation{ api::grab::HandReservationKind::Normal };

        [[nodiscard]] bool has(const PublicationFlag flag) const noexcept
        {
            return (flags & static_cast<std::uint32_t>(flag)) != 0;
        }

        [[nodiscard]] api::Sample sample() const noexcept
        {
            return { frameIndex, stateSequence, worldGeneration, skeletonGeneration, providerGeneration, collisionGeneration };
        }
    };
}

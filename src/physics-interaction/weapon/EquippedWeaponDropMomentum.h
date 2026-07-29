#pragma once

#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabMotionController.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

/*
 * Pure math for carrying hand momentum into the world ref spawned when an
 * equipped weapon is manually dropped. Per-frame hand motion samples are
 * collected while the weapon is manually carried (two-handed, part-carry, or
 * primary-only grip); on release they compose into the linear/angular
 * velocity applied to the dropped weapon's physics bodies so a throw gesture
 * keeps its momentum instead of spawning a dead weapon.
 */
namespace rock::equipped_weapon_drop_momentum
{
    inline constexpr std::size_t kHandMotionHistoryCapacity = 8;

    /*
     * Ring buffer of player-space-compensated hand motion samples in Havok
     * units. Mirrors the held-object throw history: player-space warps must
     * reset the history instead of pushing a warped sample.
     */
    template <class Vec3>
    struct HandMotionHistory
    {
        std::array<Vec3, kHandMotionHistoryCapacity> linearVelocityHavok{};
        std::array<Vec3, kHandMotionHistoryCapacity> angularVelocityRadiansPerSecond{};
        std::size_t count{ 0 };
        std::size_t next{ 0 };

        void reset()
        {
            count = 0;
            next = 0;
        }

        void push(const Vec3& linear, const Vec3& angular)
        {
            linearVelocityHavok[next] = linear;
            angularVelocityRadiansPerSecond[next] = angular;
            next = (next + 1) % kHandMotionHistoryCapacity;
            if (count < kHandMotionHistoryCapacity) {
                ++count;
            }
        }
    };

    template <class Vec3>
    struct ReleaseVelocity
    {
        bool hasData{ false };
        Vec3 linearVelocityHavok{};
        Vec3 angularVelocityRadiansPerSecond{};
        float longObjectAngularScale{ 1.0f };
        float angularVelocityCapRadiansPerSecond{ 18.0f };
    };

    struct ReleaseVelocitySettings
    {
        bool controllerDerivedEnabled{ true };
        float throwMultiplier{ 1.0f };
        float maxLinearVelocityHavok{ 12.0f };
        float angularVelocityScale{ 1.0f };
        float maxAngularVelocityRadiansPerSecond{ 18.0f };
        bool longObjectAngularScalingEnabled{ true };
        float longObjectLeverGameUnits{ 0.0f };
        float longObjectReferenceLeverGameUnits{ 24.0f };
        float longObjectMinAngularScale{ 0.35f };
    };

    /*
     * A body slot and motion ID are both reusable in hknp. This key carries
     * the stable native ownership evidence available from ROCK's exact-reference
     * body scan so a rebuilt body can never inherit a pending drop velocity
     * intended for an older generation.
     */
    struct BodyIdentityKey
    {
        std::uint32_t bodyId{ 0x7FFF'FFFF };
        std::uint32_t motionId{ 0 };
        std::uint32_t motionFirstBodyId{ 0x7FFF'FFFF };
        std::uintptr_t shapeIdentity{ 0 };
        std::uintptr_t owningNodeIdentity{ 0 };
        std::uintptr_t collisionObjectIdentity{ 0 };
        std::uintptr_t physicsSystemInstanceIdentity{ 0 };
    };

    [[nodiscard]] constexpr bool sameBodyIdentity(const BodyIdentityKey& lhs, const BodyIdentityKey& rhs) noexcept
    {
        return lhs.bodyId == rhs.bodyId &&
               lhs.motionId == rhs.motionId &&
               lhs.motionFirstBodyId == rhs.motionFirstBodyId &&
               lhs.shapeIdentity == rhs.shapeIdentity &&
               lhs.owningNodeIdentity == rhs.owningNodeIdentity &&
               lhs.collisionObjectIdentity == rhs.collisionObjectIdentity &&
               lhs.physicsSystemInstanceIdentity == rhs.physicsSystemInstanceIdentity;
    }

    inline bool completedSettleStep(std::uint64_t discoverySequence, std::uint64_t completedSequence) noexcept
    {
        return completedSequence > discoverySequence;
    }

    inline bool publicationProgressStalled(
        std::uint64_t lastProgressSequence,
        std::uint64_t completedSequence,
        std::uint64_t maximumIdleSolveSteps) noexcept
    {
        return maximumIdleSolveSteps > 0 &&
               completedSequence >= lastProgressSequence &&
               completedSequence - lastProgressSequence >= maximumIdleSolveSteps;
    }

    /*
     * Composes the release velocity through the same peak-window filter and
     * clamped composition the held-object throw path uses, so an equipped
     * weapon throw feels identical to throwing the same weapon as a loose
     * held object.
     */
    template <class Vec3>
    [[nodiscard]] inline ReleaseVelocity<Vec3> composeReleaseVelocity(
        const HandMotionHistory<Vec3>& history,
        const Vec3& playerVelocityHavok,
        const ReleaseVelocitySettings& settings)
    {
        ReleaseVelocity<Vec3> release{};
        release.longObjectAngularScale = grab_motion_controller::computeLongObjectAngularSpeedScale(
            settings.longObjectAngularScalingEnabled,
            settings.longObjectLeverGameUnits,
            settings.longObjectReferenceLeverGameUnits,
            settings.longObjectMinAngularScale);
        release.angularVelocityCapRadiansPerSecond = grab_motion_controller::computeAuthorityScaledAngularVelocityCap(
            settings.maxAngularVelocityRadiansPerSecond,
            1.0f,
            release.longObjectAngularScale);
        if (history.count == 0) {
            return release;
        }

        std::array<Vec3, kHandMotionHistoryCapacity> orderedLinear{};
        std::array<Vec3, kHandMotionHistoryCapacity> orderedAngular{};
        const std::size_t validCount = (std::min)(history.count, kHandMotionHistoryCapacity);
        const std::size_t firstIndex = (history.next + kHandMotionHistoryCapacity - validCount) % kHandMotionHistoryCapacity;
        for (std::size_t i = 0; i < validCount; ++i) {
            const std::size_t sourceIndex = (firstIndex + i) % kHandMotionHistoryCapacity;
            orderedLinear[i] = history.linearVelocityHavok[sourceIndex];
            orderedAngular[i] = history.angularVelocityRadiansPerSecond[sourceIndex];
        }

        release.linearVelocityHavok = grab_held_response::composeControllerReleaseVelocity(grab_held_response::ReleaseVelocityInput<Vec3>{
            .controllerDerivedEnabled = settings.controllerDerivedEnabled,
            .hasHandLocalVelocity = true,
            .handLocalVelocityHavok = held_object_physics_math::maxMagnitudeVelocity(orderedLinear, validCount),
            .playerVelocityHavok = playerVelocityHavok,
            .throwMultiplier = settings.throwMultiplier,
            .maxVelocityHavok = settings.maxLinearVelocityHavok,
        });
        release.angularVelocityRadiansPerSecond = grab_held_response::composeControllerReleaseAngularVelocity(grab_held_response::ReleaseAngularVelocityInput<Vec3>{
            .controllerDerivedEnabled = settings.controllerDerivedEnabled,
            .hasHandAngularVelocity = true,
            .handAngularVelocityRadiansPerSecond = held_object_physics_math::maxMagnitudeVelocity(orderedAngular, validCount),
            .angularVelocityScale = settings.angularVelocityScale,
            .maxAngularVelocityRadiansPerSecond = settings.maxAngularVelocityRadiansPerSecond,
        });
        release.angularVelocityRadiansPerSecond = grab_held_response::clampMagnitude(
            release.angularVelocityRadiansPerSecond,
            release.angularVelocityCapRadiansPerSecond);
        release.hasData = true;
        return release;
    }
}

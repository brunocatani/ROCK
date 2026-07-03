#pragma once

#include "physics-interaction/grab/GrabHeldObject.h"

#include <array>
#include <cstddef>

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
    };

    struct ReleaseVelocitySettings
    {
        bool controllerDerivedEnabled{ true };
        float throwMultiplier{ 1.0f };
        float maxLinearVelocityHavok{ 12.0f };
        float angularVelocityScale{ 1.0f };
        float maxAngularVelocityRadiansPerSecond{ 18.0f };
    };

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
        release.hasData = true;
        return release;
    }
}

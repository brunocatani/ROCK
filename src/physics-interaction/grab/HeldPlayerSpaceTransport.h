#pragma once

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>

namespace RE
{
    class hknpWorld;
}

namespace rock
{
    /*
     * Common-mode transport for bodies held in player space.
     *
     * Stick locomotion moves the grab proxy and the player through the world
     * together. The finite-force grab constraint must only solve motion of the
     * hand relative to the player. This service adds the live controller
     * velocity to each unique held motion before collision generation. It does
     * not modify or predict the grab target.
     *
     * The service owns the union of both hands for each flush. This prevents a
     * shared two-hand object from receiving the same player-space motion twice.
     */
    class HeldPlayerSpaceTransport
    {
    public:
        enum class Action : std::uint8_t
        {
            Idle,
            Rebase,
            Velocity,
            Warp,
            Hold,
            Failed,
        };

        struct Telemetry
        {
            Action action = Action::Idle;
            std::uint64_t sourceSequence = 0;
            std::uint64_t physicsStepSequence = 0;
            RE::NiPoint3 roomVelocityGameUnitsPerSecond{};
            RE::NiPoint3 largestVelocityDeltaGameUnitsPerSecond{};
            float sourceTranslationDeltaGameUnits = 0.0f;
            float sourceRotationDeltaDegrees = 0.0f;
            std::uint32_t requestedBodyCount = 0;
            std::uint32_t uniqueMotionCount = 0;
            std::uint32_t velocityWriteCount = 0;
            std::uint32_t warpWriteCount = 0;
            std::uint32_t failedWriteCount = 0;
            std::uint32_t warpCount = 0;
            std::uint32_t invalidVelocityCount = 0;
            bool sourceValid = false;
            bool controllerVelocityValid = false;
        };

        void queueSourceFrame(
            const RE::NiTransform& playerSpaceWorld,
            bool valid,
            std::uint64_t sourceSequence);

        void flushPreCollide(
            RE::hknpWorld* world,
            std::span<const std::uint32_t> heldBodyIds,
            const havok_physics_timing::PhysicsTimingSample& timing);

        void reset();
        [[nodiscard]] Telemetry telemetrySnapshot() const;
        [[nodiscard]] static const char* actionName(Action action) noexcept;

    private:
        static constexpr std::size_t kMaxTrackedMotions = 128;

        struct SourceFrame
        {
            RE::NiTransform world{};
            std::uint64_t sequence = 0;
            bool valid = false;
        };

        struct MotionState
        {
            std::uint32_t bodyId = 0x7FFF'FFFFu;
            std::uint32_t motionIndex = 0;
            std::uint32_t motionFirstBodyId = 0x7FFF'FFFFu;
            RE::NiPoint3 contributionGameUnitsPerSecond{};
        };

        void clearRuntimeLocked() noexcept;

        mutable std::mutex _mutex;
        RE::hknpWorld* _world = nullptr;
        SourceFrame _queuedSource{};
        SourceFrame _previousSource{};
        std::uint64_t _processedSourceSequence = 0;
        std::uint64_t _velocitySuppressedSourceSequence = 0;
        bool _previousSourceWarped = false;
        std::array<MotionState, kMaxTrackedMotions> _motionStates{};
        std::size_t _motionStateCount = 0;
        Telemetry _telemetry{};
        std::uint32_t _failureLogCounter = 0;
    };
}

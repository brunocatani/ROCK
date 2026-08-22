#pragma once

#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"

#include "RE/Havok/hknpWorld.h"

#include <atomic>
#include <cstdint>

namespace rock
{
    /*
     * FO4VR clears bhkWorld physics step listeners after each world update.
     * ROCK therefore owns a persistent native-compatible listener object and
     * re-registers it from the game-frame update while the callback itself
     * drives generated bodies at the beginning of the next Havok update.
     */
    class PhysicsStepDriveCoordinator
    {
    public:
        struct NativeStepListener;

        using DriveCallback = void (*)(void* userData, RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        PhysicsStepDriveCoordinator();

        void setDriveCallbacks(
            DriveCallback wholePreStepCallback,
            DriveCallback substepPreCollideCallback,
            DriveCallback betweenCollideAndSolveCallback,
            DriveCallback substepPostSolveCallback,
            void* userData);
        void registerForNextStep(void* bhkWorld, RE::hknpWorld* hknpWorld);
        void reset();

        PhysicsCallbackQuiescenceGate& callbackGate();

        struct PhysicsTimingTelemetry
        {
            std::uint64_t stepSequence = 0;
            std::uint64_t solveSequence = 0;
            double elapsedSimulatedSeconds = 0.0;
            std::uint64_t fallbackSampleCount = 0;
            float rawDeltaSeconds = 0.0f;
            float substepDeltaSeconds = 0.0f;
            std::uint32_t substepCount = 0;
            bool lastSampleUsedFallback = false;
        };

        const havok_physics_timing::PhysicsTimingSample& lastTimingSample() const { return _lastTimingSample; }
        std::uint64_t stepSequence() const { return _stepSequence; }
        /*
         * Cross-thread telemetry snapshot: the physics callbacks publish an
         * atomic mirror so the game thread can report the active physics
         * schedule without touching callback-owned state.
         */
        PhysicsTimingTelemetry physicsTimingTelemetry() const
        {
            return PhysicsTimingTelemetry{
                .stepSequence = _stepSequenceAtomic.load(std::memory_order_acquire),
                .solveSequence = _solveSequenceAtomic.load(std::memory_order_acquire),
                .elapsedSimulatedSeconds = _elapsedSimulatedAtomic.load(std::memory_order_acquire),
                .fallbackSampleCount = _fallbackSampleCountAtomic.load(std::memory_order_acquire),
                .rawDeltaSeconds = _rawDeltaSecondsAtomic.load(std::memory_order_acquire),
                .substepDeltaSeconds = _substepDeltaSecondsAtomic.load(std::memory_order_acquire),
                .substepCount = _substepCountAtomic.load(std::memory_order_acquire),
                .lastSampleUsedFallback = _lastSampleUsedFallbackAtomic.load(std::memory_order_acquire),
            };
        }
        void onBeforeWholePhysicsUpdate();
        void onBeforeAnyPhysicsStep(float substepProgress, float substepDeltaSeconds);
        void onBetweenCollideAndSolve(float substepProgress, float substepDeltaSeconds);
        void onAfterAnyPhysicsStep(float substepProgress, float substepDeltaSeconds);

    private:
        NativeStepListener* nativeListener();
        void stampTimingIdentity(havok_physics_timing::PhysicsTimingSample& sample) const;

        DriveCallback _wholePreStepCallback = nullptr;
        DriveCallback _substepPreCollideCallback = nullptr;
        DriveCallback _betweenCollideAndSolveCallback = nullptr;
        DriveCallback _substepPostSolveCallback = nullptr;
        void* _userData = nullptr;
        RE::hknpWorld* _registeredWorld = nullptr;
        havok_physics_timing::PhysicsTimingSample _lastTimingSample{};
        havok_physics_timing::PhysicsTimingSample _lastSubstepTimingSample{};
        std::uint64_t _registrationSequence = 0;
        /*
         * Step/solve sequences and the cumulative simulated clock are
         * monotonic for the coordinator lifetime: reset() drops world and
         * callback state but never rewinds these, so physics timestamps stored
         * by consumers keep ordering across world loss and recreation.
         */
        std::uint64_t _stepSequence = 0;
        std::uint64_t _solveSequence = 0;
        double _elapsedSimulatedSeconds = 0.0;
        std::uint64_t _fallbackSampleCount = 0;
        std::uint32_t _currentSubstepIndex = 0;
        // Telemetry mirror published by the physics callbacks (see
        // physicsTimingTelemetry).
        std::atomic<std::uint64_t> _stepSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _solveSequenceAtomic{ 0 };
        std::atomic<double> _elapsedSimulatedAtomic{ 0.0 };
        std::atomic<std::uint64_t> _fallbackSampleCountAtomic{ 0 };
        std::atomic<float> _rawDeltaSecondsAtomic{ 0.0f };
        std::atomic<float> _substepDeltaSecondsAtomic{ 0.0f };
        std::atomic<std::uint32_t> _substepCountAtomic{ 0 };
        std::atomic<bool> _lastSampleUsedFallbackAtomic{ false };
        NativeStepListener* _nativeListener = nullptr;
    };
}

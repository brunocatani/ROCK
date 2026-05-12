#pragma once

#include "physics-interaction/native/HavokPhysicsTiming.h"

#include "RE/Havok/hknpWorld.h"

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

        const havok_physics_timing::PhysicsTimingSample& lastTimingSample() const { return _lastTimingSample; }
        std::uint64_t stepSequence() const { return _stepSequence; }
        void onBeforeWholePhysicsUpdate();
        void onBeforeAnyPhysicsStep(float substepProgress, float substepDeltaSeconds);
        void onBetweenCollideAndSolve(float substepProgress, float substepDeltaSeconds);
        void onAfterAnyPhysicsStep(float substepProgress, float substepDeltaSeconds);

    private:
        NativeStepListener* nativeListener();

        DriveCallback _wholePreStepCallback = nullptr;
        DriveCallback _substepPreCollideCallback = nullptr;
        DriveCallback _betweenCollideAndSolveCallback = nullptr;
        DriveCallback _substepPostSolveCallback = nullptr;
        void* _userData = nullptr;
        RE::hknpWorld* _registeredWorld = nullptr;
        havok_physics_timing::PhysicsTimingSample _lastTimingSample{};
        havok_physics_timing::PhysicsTimingSample _lastSubstepTimingSample{};
        std::uint64_t _registrationSequence = 0;
        std::uint64_t _stepSequence = 0;
        std::uint32_t _currentSubstepIndex = 0;
        NativeStepListener* _nativeListener = nullptr;
    };
}

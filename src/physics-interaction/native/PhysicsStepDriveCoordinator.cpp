#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/ShellCasingGrace.h"
#include "RockConfig.h"

#include <REL/Relocation.h>

#include <atomic>
#include <cstddef>

namespace rock
{
    struct PhysicsStepDriveCoordinatorNativeStepListenerVTable
    {
        void (*unused0)();
        void (*beforeWhole)(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*);
        void (*afterBeforeWhole)();
        void (*beforeAny)(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*, float, float);
        void (*afterBeforeAny)();
        void (*betweenCollideAndSolve)(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*, std::uint32_t, float, float);
        void (*afterBetweenCollideAndSolve)();
        void (*afterAny)(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*, float, float);
        void (*afterAfterAny)();
        void (*afterWhole)(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*);
        void (*afterAfterWhole)();
    };

    struct PhysicsStepDriveCoordinatorCallbackState
    {
        PhysicsCallbackQuiescenceGate gate{};
        PhysicsCallbackQuiescenceGate::CallbackLease wholeUpdateLease{};
    };

    struct PhysicsStepDriveCoordinator::NativeStepListener
    {
        const PhysicsStepDriveCoordinatorNativeStepListenerVTable* vtable = nullptr;
        void* engineScratch = nullptr;
        std::atomic<PhysicsStepDriveCoordinator*> owner{ nullptr };
        PhysicsStepDriveCoordinatorCallbackState* callbackState = nullptr;
    };

    static_assert(sizeof(PhysicsStepDriveCoordinator::NativeStepListener) == 32);
    static_assert(offsetof(PhysicsStepDriveCoordinator::NativeStepListener, engineScratch) == 0x08);
    static_assert(offsetof(PhysicsStepDriveCoordinator::NativeStepListener, owner) == 0x10);

    namespace
    {
        using NativeStepListenerVTable = PhysicsStepDriveCoordinatorNativeStepListenerVTable;

        static_assert(offsetof(PhysicsStepDriveCoordinatorNativeStepListenerVTable, beforeWhole) == 0x08);
        static_assert(offsetof(PhysicsStepDriveCoordinatorNativeStepListenerVTable, beforeAny) == 0x18);
        static_assert(offsetof(PhysicsStepDriveCoordinatorNativeStepListenerVTable, betweenCollideAndSolve) == 0x28);
        static_assert(offsetof(PhysicsStepDriveCoordinatorNativeStepListenerVTable, afterAny) == 0x38);
        static_assert(offsetof(PhysicsStepDriveCoordinatorNativeStepListenerVTable, afterWhole) == 0x48);

        void noop()
        {}

        void beforeWhole(PhysicsStepDriveCoordinator::NativeStepListener* listener, std::uint32_t, void*)
        {
            if (!listener || !listener->callbackState) {
                return;
            }
            auto& callbackState = *listener->callbackState;
            callbackState.wholeUpdateLease = callbackState.gate.tryEnterCallback();
            if (callbackState.wholeUpdateLease) {
                if (auto* owner = listener->owner.load(std::memory_order_acquire)) {
                    owner->onBeforeWholePhysicsUpdate();
                }
            }
        }

        void beforeAny(PhysicsStepDriveCoordinator::NativeStepListener* listener, std::uint32_t, void*, float substepProgress, float substepDeltaSeconds)
        {
            if (!listener || !listener->callbackState || !listener->callbackState->wholeUpdateLease) {
                return;
            }
            if (auto* owner = listener->owner.load(std::memory_order_acquire)) {
                owner->onBeforeAnyPhysicsStep(substepProgress, substepDeltaSeconds);
            }
        }

        void betweenCollideAndSolve(
            PhysicsStepDriveCoordinator::NativeStepListener* listener,
            std::uint32_t,
            void*,
            std::uint32_t,
            float substepProgress,
            float substepDeltaSeconds)
        {
            if (!listener || !listener->callbackState || !listener->callbackState->wholeUpdateLease) {
                return;
            }
            if (auto* owner = listener->owner.load(std::memory_order_acquire)) {
                owner->onBetweenCollideAndSolve(substepProgress, substepDeltaSeconds);
            }
        }

        void afterAny(PhysicsStepDriveCoordinator::NativeStepListener* listener, std::uint32_t, void*, float substepProgress, float substepDeltaSeconds)
        {
            if (!listener || !listener->callbackState || !listener->callbackState->wholeUpdateLease) {
                return;
            }
            if (auto* owner = listener->owner.load(std::memory_order_acquire)) {
                owner->onAfterAnyPhysicsStep(substepProgress, substepDeltaSeconds);
            }
        }

        void afterWhole(PhysicsStepDriveCoordinator::NativeStepListener* listener, std::uint32_t, void*)
        {
            if (listener && listener->callbackState) {
                listener->callbackState->wholeUpdateLease = {};
            }
        }

        const NativeStepListenerVTable kStepListenerVTable{
            &noop,
            &beforeWhole,
            &noop,
            &beforeAny,
            &noop,
            &betweenCollideAndSolve,
            &noop,
            &afterAny,
            &noop,
            &afterWhole,
            &noop,
        };

        PhysicsStepDriveCoordinator::NativeStepListener* allocateProcessLifetimeListener(PhysicsStepDriveCoordinator* owner)
        {
            /*
             * bhkWorld owns listener arrays until its next update clears them. ROCK
             * can destroy/recreate PhysicsInteraction during FRIK skeleton resets,
             * so the native callback target must outlive the coordinator object.
             * The owner pointer is disabled on reset, while the vtable/object
             * storage remains valid for the process lifetime.
             */
            auto* listener = new PhysicsStepDriveCoordinator::NativeStepListener();
            listener->vtable = &kStepListenerVTable;
            listener->callbackState = new PhysicsStepDriveCoordinatorCallbackState();
            listener->owner.store(owner, std::memory_order_release);
            return listener;
        }
    }

    PhysicsStepDriveCoordinator::PhysicsStepDriveCoordinator()
    {
        _nativeListener = allocateProcessLifetimeListener(this);
    }

    void PhysicsStepDriveCoordinator::setDriveCallbacks(
        DriveCallback wholePreStepCallback,
        DriveCallback substepPreCollideCallback,
        DriveCallback betweenCollideAndSolveCallback,
        DriveCallback substepPostSolveCallback,
        void* userData)
    {
        auto mutation = callbackGate().pauseForMutation();
        _wholePreStepCallback = wholePreStepCallback;
        _substepPreCollideCallback = substepPreCollideCallback;
        _betweenCollideAndSolveCallback = betweenCollideAndSolveCallback;
        _substepPostSolveCallback = substepPostSolveCallback;
        _userData = userData;
    }

    void PhysicsStepDriveCoordinator::registerForNextStep(void* bhkWorld, RE::hknpWorld* hknpWorld)
    {
        if (!bhkWorld || !hknpWorld) {
            return;
        }

        auto& gate = callbackGate();
        gate.pauseAndWait();
        _registeredWorld = hknpWorld;
        shell_casing_grace::prepareFrame(hknpWorld, g_rockConfig.rockWeaponShellCollisionGraceMs,
            _elapsedSimulatedSeconds, _solveSequence);
        ++_registrationSequence;
        if (_nativeListener) {
            _nativeListener->vtable = &kStepListenerVTable;
            _nativeListener->owner.store(this, std::memory_order_release);
        }

        using AddStepListener_t = void (*)(void*, NativeStepListener*);
        static REL::Relocation<AddStepListener_t> addStepListener{ REL::Offset(offsets::kFunc_World_AddStepListener) };
        addStepListener(bhkWorld, nativeListener());
        gate.resumeCallbacks();
    }

    void PhysicsStepDriveCoordinator::reset()
    {
        auto& gate = callbackGate();
        gate.pauseAndWait();
        if (_nativeListener) {
            _nativeListener->owner.store(nullptr, std::memory_order_release);
        }
        shell_casing_grace::abandon();
        _registeredWorld = nullptr;
        _lastTimingSample = {};
        _lastSubstepTimingSample = {};
        _registrationSequence = 0;
        _currentSubstepIndex = 0;
        // _stepSequence, _solveSequence, and _elapsedSimulatedSeconds stay
        // monotonic across resets by contract (see the header).
    }

    PhysicsCallbackQuiescenceGate& PhysicsStepDriveCoordinator::callbackGate()
    {
        return _nativeListener->callbackState->gate;
    }

    PhysicsStepDriveCoordinator::NativeStepListener* PhysicsStepDriveCoordinator::nativeListener()
    {
        return _nativeListener;
    }

    void PhysicsStepDriveCoordinator::stampTimingIdentity(havok_physics_timing::PhysicsTimingSample& sample) const
    {
        sample.stepSequence = _stepSequence;
        sample.solveSequence = _solveSequence;
        sample.elapsedSimulatedSeconds = _elapsedSimulatedSeconds;
    }

    void PhysicsStepDriveCoordinator::onBeforeWholePhysicsUpdate()
    {
        _lastTimingSample = havok_physics_timing::sampleCurrentTiming();
        _lastSubstepTimingSample = {};
        ++_stepSequence;
        _currentSubstepIndex = 0;
        stampTimingIdentity(_lastTimingSample);
        if (_lastTimingSample.usedFallback) {
            ++_fallbackSampleCount;
        }
        _stepSequenceAtomic.store(_stepSequence, std::memory_order_release);
        _fallbackSampleCountAtomic.store(_fallbackSampleCount, std::memory_order_release);
        _rawDeltaSecondsAtomic.store(_lastTimingSample.rawDeltaSeconds, std::memory_order_release);
        _substepCountAtomic.store(_lastTimingSample.substepCount, std::memory_order_release);
        _lastSampleUsedFallbackAtomic.store(_lastTimingSample.usedFallback, std::memory_order_release);

        if (!_wholePreStepCallback || !_registeredWorld) {
            return;
        }

        _wholePreStepCallback(_userData, _registeredWorld, _lastTimingSample);
    }

    void PhysicsStepDriveCoordinator::onBeforeAnyPhysicsStep(float substepProgress, float substepDeltaSeconds)
    {
        auto timing =
            havok_physics_timing::makeSubstepTimingSample(_lastTimingSample, substepProgress, substepDeltaSeconds, _currentSubstepIndex);
        stampTimingIdentity(timing);
        _lastSubstepTimingSample = timing;
        ++_currentSubstepIndex;
        shell_casing_grace::beforeCollide(_registeredWorld, timing);

        if (!_substepPreCollideCallback || !_registeredWorld) {
            return;
        }

        _substepPreCollideCallback(_userData, _registeredWorld, timing);
    }

    void PhysicsStepDriveCoordinator::onBetweenCollideAndSolve(float substepProgress, float substepDeltaSeconds)
    {
        if (!_lastSubstepTimingSample.valid) {
            _lastSubstepTimingSample = havok_physics_timing::makeSubstepTimingSample(_lastTimingSample, substepProgress, substepDeltaSeconds, _currentSubstepIndex);
            stampTimingIdentity(_lastSubstepTimingSample);
        }

        if (!_betweenCollideAndSolveCallback || !_registeredWorld) {
            return;
        }

        const auto timing =
            havok_physics_timing::makeSubstepPhaseTimingSample(_lastSubstepTimingSample, havok_physics_timing::PhysicsStepPhase::BetweenCollideAndSolve);
        _betweenCollideAndSolveCallback(_userData, _registeredWorld, timing);
    }

    void PhysicsStepDriveCoordinator::onAfterAnyPhysicsStep(float substepProgress, float substepDeltaSeconds)
    {
        if (!_lastSubstepTimingSample.valid) {
            _lastSubstepTimingSample = havok_physics_timing::makeSubstepTimingSample(_lastTimingSample, substepProgress, substepDeltaSeconds, _currentSubstepIndex);
            stampTimingIdentity(_lastSubstepTimingSample);
        }

        auto timing =
            havok_physics_timing::makeSubstepPhaseTimingSample(_lastSubstepTimingSample, havok_physics_timing::PhysicsStepPhase::SubstepPostSolve);
        /*
         * The substep's solve completes at this callback: the post-solve
         * sample carries the updated solve identity and cumulative simulated
         * clock so retention and observation code can timestamp against the
         * state that now exists.
         */
        ++_solveSequence;
        if (havok_physics_timing::shouldAccumulateSimulatedTime(timing)) {
            _elapsedSimulatedSeconds += timing.substepDeltaSeconds;
        } else {
            ++_fallbackSampleCount;
        }
        stampTimingIdentity(timing);
        _solveSequenceAtomic.store(_solveSequence, std::memory_order_release);
        shell_casing_grace::afterSolve(_registeredWorld, timing);
        _elapsedSimulatedAtomic.store(_elapsedSimulatedSeconds, std::memory_order_release);
        _fallbackSampleCountAtomic.store(_fallbackSampleCount, std::memory_order_release);
        _substepDeltaSecondsAtomic.store(timing.substepDeltaSeconds, std::memory_order_release);

        if (!_substepPostSolveCallback || !_registeredWorld) {
            return;
        }

        _substepPostSolveCallback(_userData, _registeredWorld, timing);
    }
}

#include "physics-interaction/native/PhysicsStepDriveCoordinator.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/PhysicsLog.h"

#include <REL/Relocation.h>

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

    struct PhysicsStepDriveCoordinator::NativeStepListener
    {
        const PhysicsStepDriveCoordinatorNativeStepListenerVTable* vtable = nullptr;
        void* engineScratch = nullptr;
        PhysicsStepDriveCoordinator* owner = nullptr;
    };

    static_assert(sizeof(PhysicsStepDriveCoordinator::NativeStepListener) <= 32);
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
            if (listener && listener->owner) {
                listener->owner->onBeforeWholePhysicsUpdate();
            }
        }

        void beforeAny(PhysicsStepDriveCoordinator::NativeStepListener* listener, std::uint32_t, void*, float substepProgress, float substepDeltaSeconds)
        {
            if (listener && listener->owner) {
                listener->owner->onBeforeAnyPhysicsStep(substepProgress, substepDeltaSeconds);
            }
        }

        void betweenCollideAndSolve(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*, std::uint32_t, float, float)
        {}

        void afterAny(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*, float, float)
        {}

        void afterWhole(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*)
        {}

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
            listener->owner = owner;
            return listener;
        }
    }

    PhysicsStepDriveCoordinator::PhysicsStepDriveCoordinator()
    {
        _nativeListener = allocateProcessLifetimeListener(this);
    }

    void PhysicsStepDriveCoordinator::setDriveCallbacks(DriveCallback wholePreStepCallback, DriveCallback substepPreCollideCallback, void* userData)
    {
        _wholePreStepCallback = wholePreStepCallback;
        _substepPreCollideCallback = substepPreCollideCallback;
        _userData = userData;
    }

    void PhysicsStepDriveCoordinator::registerForNextStep(void* bhkWorld, RE::hknpWorld* hknpWorld)
    {
        if (!bhkWorld || !hknpWorld) {
            return;
        }

        _registeredWorld = hknpWorld;
        ++_registrationSequence;
        if (_nativeListener) {
            _nativeListener->vtable = &kStepListenerVTable;
            _nativeListener->owner = this;
        }

        using AddStepListener_t = void (*)(void*, NativeStepListener*);
        static REL::Relocation<AddStepListener_t> addStepListener{ REL::Offset(offsets::kFunc_World_AddStepListener) };
        addStepListener(bhkWorld, nativeListener());
    }

    void PhysicsStepDriveCoordinator::reset()
    {
        _registeredWorld = nullptr;
        _lastTimingSample = {};
        _registrationSequence = 0;
        _stepSequence = 0;
        _currentSubstepIndex = 0;
        if (_nativeListener && _nativeListener->owner == this) {
            _nativeListener->owner = nullptr;
        }
    }

    PhysicsStepDriveCoordinator::NativeStepListener* PhysicsStepDriveCoordinator::nativeListener()
    {
        return _nativeListener;
    }

    void PhysicsStepDriveCoordinator::onBeforeWholePhysicsUpdate()
    {
        _lastTimingSample = havok_physics_timing::sampleCurrentTiming();
        ++_stepSequence;
        _currentSubstepIndex = 0;

        if (!_wholePreStepCallback || !_registeredWorld) {
            return;
        }

        _wholePreStepCallback(_userData, _registeredWorld, _lastTimingSample);
    }

    void PhysicsStepDriveCoordinator::onBeforeAnyPhysicsStep(float substepProgress, float substepDeltaSeconds)
    {
        if (!_substepPreCollideCallback || !_registeredWorld) {
            return;
        }

        const auto timing =
            havok_physics_timing::makeSubstepTimingSample(_lastTimingSample, substepProgress, substepDeltaSeconds, _currentSubstepIndex);
        ++_currentSubstepIndex;
        _substepPreCollideCallback(_userData, _registeredWorld, timing);
    }
}

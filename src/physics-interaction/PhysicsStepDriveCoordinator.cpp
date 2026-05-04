#include "PhysicsStepDriveCoordinator.h"

#include "HavokOffsets.h"
#include "PhysicsLog.h"

#include <REL/Relocation.h>

#include <cstddef>
#include <new>

namespace frik::rock
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

        void beforeAny(PhysicsStepDriveCoordinator::NativeStepListener*, std::uint32_t, void*, float, float)
        {}

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
    }

    PhysicsStepDriveCoordinator::PhysicsStepDriveCoordinator()
    {
        auto* listener = new (_nativeListenerStorage) NativeStepListener();
        listener->vtable = &kStepListenerVTable;
        listener->owner = this;
    }

    void PhysicsStepDriveCoordinator::setDriveCallback(DriveCallback callback, void* userData)
    {
        _callback = callback;
        _userData = userData;
    }

    void PhysicsStepDriveCoordinator::registerForNextStep(void* bhkWorld, RE::hknpWorld* hknpWorld)
    {
        if (!bhkWorld || !hknpWorld) {
            return;
        }

        _registeredWorld = hknpWorld;
        ++_registrationSequence;

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
    }

    PhysicsStepDriveCoordinator::NativeStepListener* PhysicsStepDriveCoordinator::nativeListener()
    {
        return reinterpret_cast<NativeStepListener*>(_nativeListenerStorage);
    }

    void PhysicsStepDriveCoordinator::onBeforeWholePhysicsUpdate()
    {
        _lastTimingSample = havok_physics_timing::sampleCurrentTiming();
        ++_stepSequence;

        if (!_callback || !_registeredWorld) {
            return;
        }

        _callback(_userData, _registeredWorld, _lastTimingSample);
    }
}

#include "physics-interaction/native/NativeWorldLifetimeDiagnostics.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <intrin.h>

#include <array>
#include <cstdint>
#include <cstring>

namespace rock::native_world_lifetime_diagnostics
{
    namespace
    {
        using DestroyFn = void* (*)(void*, std::uint32_t);

        /*
         * FO4VR 1.2.72 raw disassembly, 2026-09-10: the reference-release
         * dispatcher at 0x1540960 tail-calls vtable[0] with EDX=1. Both
         * deleting destructors call ~hknpWorld at 0x1542570, then optionally
         * free the allocation and return its address. Observe before either
         * operation, without retaining pointers or changing ownership.
         */
        constexpr std::uintptr_t kBaseVtableRva = 0x2DFC540;
        constexpr std::uintptr_t kBethesdaVtableRva = 0x2E82B20;
        constexpr std::uintptr_t kBaseDestructorRva = 0x15565F0;
        constexpr std::uintptr_t kBethesdaDestructorRva = 0x1E028A0;
        constexpr std::array<std::uint8_t, 10> kExpectedDestructorEntry{
            0x48, 0x89, 0x5C, 0x24, 0x08, 0x57, 0x48, 0x83, 0xEC, 0x20,
        };

        // Published during plugin initialization, before engine callbacks.
        DestroyFn s_baseDestructor = nullptr;
        DestroyFn s_bethesdaDestructor = nullptr;
        bool s_installed = false;

        void traceDestruction(void* world, std::uint32_t flags,
            std::uintptr_t caller, const char* kind) noexcept
        {
            if (!logger::isDebugEnabled()) {
                return;
            }

            // This is one event per world lifetime, on its native destruction
            // thread. All storage is local and bounded; normal physics steps
            // do no tracing. Existing debug logging controls this diagnostic.
            std::uintptr_t vtable = 0;
            std::uintptr_t bodies = 0;
            std::uintptr_t constraints = 0;
            std::uint32_t references = 0;
            std::uint32_t constraintCount = 0;
            const bool readable =
                native_memory::tryReadField(world, 0, vtable) &&
                native_memory::tryReadField(world, 0x08, references) &&
                native_memory::tryReadField(world, offsets::kHknpWorld_BodyArrayPtr, bodies) &&
                native_memory::tryReadField(world, offsets::kHknpWorld_ConstraintArrayPtr, constraints) &&
                native_memory::tryReadField(world, offsets::kHknpWorld_ConstraintCount, constraintCount);
            std::array<void*, 12> stack{};
            const auto frames = RtlCaptureStackBackTrace(1,
                static_cast<DWORD>(stack.size()), stack.data(), nullptr);
            const auto moduleBase = REL::Module::get().base();
            ROCK_LOG_DEBUG(PhysicsSafety,
                "Physics-world destruction: kind={} world={:p} flags=0x{:X} thread={} physicsStep={} readable={} vtable=0x{:X} refWord=0x{:08X} bodyArray=0x{:X} constraintArray=0x{:X} constraints={} caller=0x{:X} gameBase=0x{:X}",
                kind, world, flags, GetCurrentThreadId(),
                havok_world_lock::detail::currentThreadInsidePhysicsStep(),
                readable, vtable, references, bodies, constraints, constraintCount,
                caller, moduleBase);
            ROCK_LOG_DEBUG(PhysicsSafety,
                "Physics-world destruction stack: world={:p} frames={} addresses={:p}/{:p}/{:p}/{:p}/{:p}/{:p}/{:p}/{:p}/{:p}/{:p}/{:p}/{:p}",
                world, frames, stack[0], stack[1], stack[2], stack[3],
                stack[4], stack[5], stack[6], stack[7], stack[8], stack[9],
                stack[10], stack[11]);
        }

        void* onBaseDestroy(void* world, std::uint32_t flags) noexcept
        {
            traceDestruction(world, flags,
                reinterpret_cast<std::uintptr_t>(_ReturnAddress()), "hknpWorld");
            return s_baseDestructor(world, flags);
        }

        void* onBethesdaDestroy(void* world, std::uint32_t flags) noexcept
        {
            traceDestruction(world, flags,
                reinterpret_cast<std::uintptr_t>(_ReturnAddress()), "hknpBSWorld");
            return s_bethesdaDestructor(world, flags);
        }

        bool validateDestructor(std::uintptr_t vtableRva,
            std::uintptr_t destructorRva) noexcept
        {
            const auto vtable = REL::Offset(vtableRva).address();
            const auto destructor = REL::Offset(destructorRva).address();
            return *reinterpret_cast<const std::uintptr_t*>(vtable) == destructor &&
                std::memcmp(reinterpret_cast<const void*>(destructor),
                    kExpectedDestructorEntry.data(), kExpectedDestructorEntry.size()) == 0;
        }
    }

    bool install() noexcept
    {
        if (s_installed) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72 ||
            !validateDestructor(kBaseVtableRva, kBaseDestructorRva) ||
            !validateDestructor(kBethesdaVtableRva, kBethesdaDestructorRva)) {
            ROCK_LOG_ERROR(Init,
                "Physics-world lifetime diagnostic unavailable: native destructor validation failed");
            return false;
        }

        s_baseDestructor = reinterpret_cast<DestroyFn>(REL::Offset(kBaseDestructorRva).address());
        s_bethesdaDestructor = reinterpret_cast<DestroyFn>(REL::Offset(kBethesdaDestructorRva).address());
        REL::Relocation<std::uintptr_t> baseVtable{ REL::Offset(kBaseVtableRva) };
        REL::Relocation<std::uintptr_t> bethesdaVtable{ REL::Offset(kBethesdaVtableRva) };
        baseVtable.write_vfunc(0, &onBaseDestroy);
        bethesdaVtable.write_vfunc(0, &onBethesdaDestroy);
        s_installed = true;
        ROCK_LOG_INFO(Init,
            "Physics-world lifetime diagnostic installed: destruction traces follow debug logging");
        return true;
    }
}

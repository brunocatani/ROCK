#pragma once

#include "physics-interaction/native/HavokOffsets.h"

#include "RE/Havok/hknpWorld.h"

#include <cstdint>
#include <intrin.h>

namespace rock::havok_world_lock
{
    /*
     * FO4VR serializes hknp world access through the reader-writer lock at
     * hknpWorld + kHknpWorld_AccessLock: engine mutation wrappers hold it for
     * write, engine queries hold it for read. The broadphase shape cast
     * dereferences candidate body shapes with no null check, so a query issued
     * WITHOUT the read lock races body-slot reuse (streaming, teardown, step
     * command buffers) and crashes on a half-mutated slot (six castShape CTDs
     * 2026-07-12/13; see
     * Docs/ROCK/lessons/2026-07-13-hknp-world-query-lock-discipline.md).
     *
     * Threading contract: threads inside the physics step publish a nonzero TLS
     * flag and already own the world; the engine skips locking there and so does
     * this guard. Never hold this lock across anything that can re-enter world
     * mutation.
     */
    namespace detail
    {
        using BSReadWriteLockFn_t = void (*)(void*);

        [[nodiscard]] inline bool currentThreadInsidePhysicsStep()
        {
            static REL::Relocation<const std::uint32_t*> exeTlsIndex{ REL::Offset(offsets::kGlobal_ExeTlsIndex) };
            const auto* tlsArray = reinterpret_cast<const std::uintptr_t*>(__readgsqword(0x58));
            if (!tlsArray) {
                return false;
            }

            const std::uintptr_t tlsBase = tlsArray[*exeTlsIndex];
            if (!tlsBase) {
                return false;
            }

            return *reinterpret_cast<const std::uint8_t*>(tlsBase + offsets::kTlsSlot_InPhysicsStepFlag) != 0;
        }
    }

    class ScopedWorldReadLock
    {
    public:
        explicit ScopedWorldReadLock(RE::hknpWorld* world)
        {
            if (!world || detail::currentThreadInsidePhysicsStep()) {
                return;
            }

            static REL::Relocation<detail::BSReadWriteLockFn_t> lockForRead{ REL::Offset(offsets::kFunc_BSReadWriteLock_LockForRead) };
            _lock = reinterpret_cast<void*>(reinterpret_cast<std::uintptr_t>(world) + offsets::kHknpWorld_AccessLock);
            lockForRead(_lock);
        }

        ~ScopedWorldReadLock()
        {
            if (!_lock) {
                return;
            }

            static REL::Relocation<detail::BSReadWriteLockFn_t> unlockForRead{ REL::Offset(offsets::kFunc_BSReadWriteLock_UnlockForRead) };
            unlockForRead(_lock);
        }

        ScopedWorldReadLock(const ScopedWorldReadLock&) = delete;
        ScopedWorldReadLock& operator=(const ScopedWorldReadLock&) = delete;
        ScopedWorldReadLock(ScopedWorldReadLock&&) = delete;
        ScopedWorldReadLock& operator=(ScopedWorldReadLock&&) = delete;

        [[nodiscard]] bool held() const { return _lock != nullptr; }

    private:
        void* _lock = nullptr;
    };
}

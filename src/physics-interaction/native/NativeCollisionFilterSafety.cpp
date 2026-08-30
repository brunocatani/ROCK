#include "physics-interaction/native/NativeCollisionFilterSafety.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <intrin.h>

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>

namespace rock::native_collision_filter_safety
{
    namespace
    {
        using GetCollisionFilterInfoFn = std::uint32_t* (*)(
            void*,
            std::uint32_t*);

        /*
         * Fallout4VR.exe 1.2.72,
         * bhkNPCollisionObject::GetCollisionFilterInfo. These are complete,
         * position-independent entry instructions verified on 2026-08-30.
         */
        constexpr std::array<std::uint8_t, 16> kExpectedFilterInfoEntry{
            0x48, 0x89, 0x5C, 0x24, 0x18,
            0x48, 0x89, 0x7C, 0x24, 0x20,
            0x41, 0x56,
            0x48, 0x83, 0xEC, 0x20,
        };

        constexpr std::uintptr_t kMaximumNullDerivedReadAddress = 0x1000'0000;
        constexpr std::uintptr_t kBodyStride = 0x90;
        constexpr std::uintptr_t kBodyFilterInfoOffset = 0x44;
        constexpr std::ptrdiff_t kCollisionObjectPhysicsSystemOffset = 0x20;
        constexpr std::ptrdiff_t kCollisionObjectSystemBodyIndexOffset = 0x28;
        constexpr std::uint32_t kInvalidFilterInfo = 0xFFFF'FFFFu;
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        enum ReadableField : std::uint32_t
        {
            kPhysicsSystemReadable = 1u << 0,
            kSystemBodyIndexReadable = 1u << 1,
            kWorldVtableReadable = 1u << 2,
            kWorldBodyArrayReadable = 1u << 3,
            kWorldConstraintArrayReadable = 1u << 4,
        };

        struct FaultContext
        {
            bool captured = false;
            std::uintptr_t exceptionAddress = 0;
            std::uintptr_t accessAddress = 0;
            std::uintptr_t rax = 0;
            std::uintptr_t rbx = 0;
            std::uintptr_t rcx = 0;
            std::uintptr_t rdx = 0;
            std::uintptr_t rsi = 0;
            std::uintptr_t rdi = 0;
            std::uintptr_t r14 = 0;
            std::uintptr_t rsp = 0;
        };

        struct DiagnosticState
        {
            std::uint32_t readableFields = 0;
            std::uintptr_t physicsSystem = 0;
            std::uint32_t systemBodyIndex = 0;
            std::uintptr_t worldVtable = 0;
            std::uintptr_t bodyArray = 0;
            std::uintptr_t constraintArray = 0;
            std::uint32_t faultBodyId = kInvalidBodyId;
            bool destroying = false;
        };

        GetCollisionFilterInfoFn s_originalGetCollisionFilterInfo = nullptr;
        std::atomic<bool> s_installed{ false };
        std::atomic<std::uintptr_t> s_expectedFaultAddress{ 0 };
        std::atomic<std::uintptr_t> s_destroyingWorldVtable{ 0 };
        std::atomic<std::uint64_t> s_suppressedFaultCount{ 0 };
        thread_local FaultContext t_faultContext{};
        thread_local std::uint32_t t_invalidFilterInfo = kInvalidFilterInfo;

        [[nodiscard]] bool shouldLogOccurrence(
            const std::uint64_t occurrence) noexcept
        {
            return occurrence <= 4 || std::has_single_bit(occurrence);
        }

        [[nodiscard]] std::uintptr_t gameTextRelativeAddress(
            const std::uintptr_t address) noexcept
        {
            const auto text = REL::Module::get().segment(REL::Segment::text);
            if (text.size() == 0 || address < text.address() ||
                address >= text.address() + text.size()) {
                return 0;
            }
            return address - REL::Module::get().base();
        }

        int captureVerifiedBodyArrayFault(
            EXCEPTION_POINTERS* exceptionPointers) noexcept
        {
            if (!exceptionPointers || !exceptionPointers->ExceptionRecord ||
                !exceptionPointers->ContextRecord) {
                return EXCEPTION_CONTINUE_SEARCH;
            }

            const auto* record = exceptionPointers->ExceptionRecord;
            const auto* context = exceptionPointers->ContextRecord;
            const auto expectedAddress =
                s_expectedFaultAddress.load(std::memory_order_acquire);
            const auto exceptionAddress =
                reinterpret_cast<std::uintptr_t>(record->ExceptionAddress);
            if (record->ExceptionCode != EXCEPTION_ACCESS_VIOLATION ||
                record->NumberParameters < 2 ||
                record->ExceptionInformation[0] != 0 ||
                expectedAddress == 0 || exceptionAddress != expectedAddress) {
                return EXCEPTION_CONTINUE_SEARCH;
            }

            const auto accessAddress = static_cast<std::uintptr_t>(
                record->ExceptionInformation[1]);
            const auto bodyAddress = static_cast<std::uintptr_t>(context->Rdi);
            if (bodyAddress >= kMaximumNullDerivedReadAddress ||
                accessAddress != bodyAddress + kBodyFilterInfoOffset) {
                return EXCEPTION_CONTINUE_SEARCH;
            }

            t_faultContext = {
                .captured = true,
                .exceptionAddress = exceptionAddress,
                .accessAddress = accessAddress,
                .rax = static_cast<std::uintptr_t>(context->Rax),
                .rbx = static_cast<std::uintptr_t>(context->Rbx),
                .rcx = static_cast<std::uintptr_t>(context->Rcx),
                .rdx = static_cast<std::uintptr_t>(context->Rdx),
                .rsi = static_cast<std::uintptr_t>(context->Rsi),
                .rdi = bodyAddress,
                .r14 = static_cast<std::uintptr_t>(context->R14),
                .rsp = static_cast<std::uintptr_t>(context->Rsp),
            };
            return EXCEPTION_EXECUTE_HANDLER;
        }

        [[nodiscard]] __declspec(noinline) bool invokeOriginal(
            void* collisionObject,
            std::uint32_t* filterInfoOut,
            std::uint32_t*& result) noexcept
        {
            auto* original = s_originalGetCollisionFilterInfo;
            if (!original) {
                result = nullptr;
                return false;
            }

            t_faultContext = {};
#if defined(_MSC_VER)
            __try {
                result = original(collisionObject, filterInfoOut);
                return true;
            } __except (captureVerifiedBodyArrayFault(GetExceptionInformation())) {
                result = nullptr;
                return false;
            }
#else
            result = original(collisionObject, filterInfoOut);
            return true;
#endif
        }

        DiagnosticState captureDiagnosticState(
            void* collisionObject,
            const FaultContext& fault) noexcept
        {
            DiagnosticState state{};
            if (native_memory::tryReadField(
                    collisionObject,
                    kCollisionObjectPhysicsSystemOffset,
                    state.physicsSystem)) {
                state.readableFields |= kPhysicsSystemReadable;
            }
            if (native_memory::tryReadField(
                    collisionObject,
                    kCollisionObjectSystemBodyIndexOffset,
                    state.systemBodyIndex)) {
                state.readableFields |= kSystemBodyIndexReadable;
            }

            if (fault.rsi != 0) {
                const auto* world = reinterpret_cast<const void*>(fault.rsi);
                if (native_memory::tryReadField(world, 0, state.worldVtable)) {
                    state.readableFields |= kWorldVtableReadable;
                }
                if (native_memory::tryReadField(
                        world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_BodyArrayPtr),
                        state.bodyArray)) {
                    state.readableFields |= kWorldBodyArrayReadable;
                }
                if (native_memory::tryReadField(
                        world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_ConstraintArrayPtr),
                        state.constraintArray)) {
                    state.readableFields |= kWorldConstraintArrayReadable;
                }
            }

            const auto destroyingVtable =
                s_destroyingWorldVtable.load(std::memory_order_acquire);
            state.destroying = destroyingVtable != 0 &&
                               state.worldVtable == destroyingVtable;
            if (state.bodyArray == 0 && fault.rdi % kBodyStride == 0) {
                const auto id = fault.rdi / kBodyStride;
                if (id <= kInvalidBodyId) {
                    state.faultBodyId = static_cast<std::uint32_t>(id);
                }
            }
            return state;
        }

        void recordSuppressedFault(
            void* collisionObject,
            const std::uintptr_t caller,
            const bool outputStored) noexcept
        {
            const auto fault = t_faultContext;
            const auto occurrence =
                s_suppressedFaultCount.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (!fault.captured || !shouldLogOccurrence(occurrence)) {
                return;
            }

            const auto state = captureDiagnosticState(collisionObject, fault);
            const auto callerRva = gameTextRelativeAddress(caller);
            const auto exceptionRva =
                gameTextRelativeAddress(fault.exceptionAddress);
            const auto insidePhysicsStep =
                havok_world_lock::detail::currentThreadInsidePhysicsStep();

            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Suppressed native collision-filter teardown fault: occurrence={} thread={} physicsStep={} collisionObject={:p} caller=0x{:X} callerRva=0x{:X} exceptionRva=0x{:X} access=0x{:X} outputStored={}",
                occurrence,
                GetCurrentThreadId(),
                insidePhysicsStep ? "yes" : "no",
                collisionObject,
                caller,
                callerRva,
                exceptionRva,
                fault.accessAddress,
                outputStored ? "yes" : "fallback");
            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Collision-filter teardown state: readable=0x{:02X} destroying={} world=0x{:X} worldVtable=0x{:X} bodyArray=0x{:X} constraintArray=0x{:X} faultBodyId={} computedBody=0x{:X} physicsSystem=0x{:X} systemBodyIndex={} regs(rax/rbx/rcx/rdx/r14/rsp)=0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}",
                state.readableFields,
                state.destroying ? "yes" : "no",
                fault.rsi,
                state.worldVtable,
                state.bodyArray,
                state.constraintArray,
                state.faultBodyId,
                fault.rdi,
                state.physicsSystem,
                state.systemBodyIndex,
                fault.rax,
                fault.rbx,
                fault.rcx,
                fault.rdx,
                fault.r14,
                fault.rsp);
        }

        __declspec(noinline) std::uint32_t* onGetCollisionFilterInfo(
            void* collisionObject,
            std::uint32_t* filterInfoOut) noexcept
        {
            const auto caller =
                reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            std::uint32_t* result = nullptr;
            if (invokeOriginal(collisionObject, filterInfoOut, result)) {
                return result;
            }

            t_invalidFilterInfo = kInvalidFilterInfo;
            const bool outputStored = filterInfoOut &&
                native_memory::tryWriteValue(
                    filterInfoOut,
                    t_invalidFilterInfo);
            recordSuppressedFault(collisionObject, caller, outputStored);
            return outputStored ? filterInfoOut : &t_invalidFilterInfo;
        }
    }

    bool install() noexcept
    {
        if (s_installed.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(Init,
                "Native collision-filter safety unavailable: unsupported runtime");
            return false;
        }

        s_expectedFaultAddress.store(
            REL::Offset(offsets::kFault_BhkNPCollisionObject_BodyFilterRead).address(),
            std::memory_order_release);
        s_destroyingWorldVtable.store(
            REL::Offset(offsets::kVtable_HknpWorldDestroying).address(),
            std::memory_order_release);

        void* original =
            reinterpret_cast<void*>(s_originalGetCollisionFilterInfo);
        const bool installed = entry_trampoline_hook::install(
            "bhkNPCollisionObject collision-filter teardown safety",
            offsets::kFunc_BhkNPCollisionObject_GetCollisionFilterInfo,
            kExpectedFilterInfoEntry.data(),
            kExpectedFilterInfoEntry.size(),
            reinterpret_cast<void*>(&onGetCollisionFilterInfo),
            original);
        s_originalGetCollisionFilterInfo =
            reinterpret_cast<GetCollisionFilterInfoFn>(original);
        const bool ready = installed &&
                           s_originalGetCollisionFilterInfo != nullptr;
        s_installed.store(ready, std::memory_order_release);
        if (!ready) {
            ROCK_LOG_ERROR(Init,
                "Native collision-filter safety unavailable: hook installation failed");
            return false;
        }

        ROCK_LOG_INFO(Init,
            "Native collision-filter teardown safety active: exactFaultRva=0x{:X}",
            offsets::kFault_BhkNPCollisionObject_BodyFilterRead);
        return true;
    }
}

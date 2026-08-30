#include "physics-interaction/native/NativeRagdollSafety.h"

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

namespace rock::native_ragdoll_safety
{
    namespace
    {
        using UpdateConstraintsFn = void (*)(void*);

        /*
         * Fallout4VR.exe 1.2.72, hkbnpRagdollInterface::updateConstraints.
         * Whole-instruction, position-independent entry bytes verified from
         * raw disassembly and Ghidra on 2026-08-30.
         */
        constexpr std::array<std::uint8_t, 15> kExpectedUpdateConstraintsEntry{
            0x48, 0x89, 0x4C, 0x24, 0x08,
            0x55,
            0x56,
            0x41, 0x56,
            0x41, 0x57,
            0x48, 0x83, 0xEC, 0x28,
        };

        constexpr std::uintptr_t kMaximumNullDerivedReadAddress = 0x1000'0000;
        constexpr std::ptrdiff_t kInterfacePhysicsInterfaceOffset = 0x10;
        constexpr std::ptrdiff_t kInterfaceRagdollDataOffset = 0x18;
        constexpr std::ptrdiff_t kPhysicsInterfaceWorldOffset = 0x18;
        constexpr std::ptrdiff_t kRagdollDataConstraintIdsOffset = 0x30;
        constexpr std::ptrdiff_t kRagdollDataConstraintCountOffset = 0x38;
        constexpr std::uintptr_t kConstraintStride = 0x38;
        constexpr std::uint32_t kInvalidConstraintId = 0xFFFF'FFFFu;

        enum ReadableField : std::uint32_t
        {
            kPhysicsInterfaceReadable = 1u << 0,
            kRagdollDataReadable = 1u << 1,
            kResolvedWorldReadable = 1u << 2,
            kConstraintIdsReadable = 1u << 3,
            kRagdollConstraintCountReadable = 1u << 4,
            kWorldVtableReadable = 1u << 5,
            kWorldBodyArrayReadable = 1u << 6,
            kWorldConstraintArrayReadable = 1u << 7,
            kWorldConstraintCountReadable = 1u << 8,
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
            std::uintptr_t r12 = 0;
            std::uintptr_t r15 = 0;
            std::uintptr_t rsp = 0;
        };

        struct DiagnosticState
        {
            std::uint32_t readableFields = 0;
            std::uintptr_t physicsInterface = 0;
            std::uintptr_t ragdollData = 0;
            std::uintptr_t resolvedWorld = 0;
            std::uintptr_t constraintIds = 0;
            std::uint32_t ragdollConstraintCount = 0;
            std::uintptr_t worldVtable = 0;
            std::uintptr_t bodyArray = 0;
            std::uintptr_t constraintArray = 0;
            std::uint32_t worldConstraintCount = 0;
            std::uint32_t faultConstraintId = kInvalidConstraintId;
            bool destroying = false;
        };

        UpdateConstraintsFn s_originalUpdateConstraints = nullptr;
        std::atomic<bool> s_installed{ false };
        std::atomic<std::uintptr_t> s_expectedFaultAddress{ 0 };
        std::atomic<std::uintptr_t> s_destroyingWorldVtable{ 0 };
        std::atomic<std::uint64_t> s_suppressedFaultCount{ 0 };
        thread_local FaultContext t_faultContext{};

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

        int captureVerifiedTeardownFault(
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
            if (accessAddress >= kMaximumNullDerivedReadAddress ||
                static_cast<std::uintptr_t>(context->R12) != accessAddress) {
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
                .rdi = static_cast<std::uintptr_t>(context->Rdi),
                .r12 = static_cast<std::uintptr_t>(context->R12),
                .r15 = static_cast<std::uintptr_t>(context->R15),
                .rsp = static_cast<std::uintptr_t>(context->Rsp),
            };
            return EXCEPTION_EXECUTE_HANDLER;
        }

        [[nodiscard]] __declspec(noinline) bool invokeOriginal(
            void* ragdollInterface) noexcept
        {
            auto* original = s_originalUpdateConstraints;
            if (!original) {
                return true;
            }

            t_faultContext = {};
#if defined(_MSC_VER)
            __try {
                original(ragdollInterface);
                return true;
            } __except (captureVerifiedTeardownFault(GetExceptionInformation())) {
                return false;
            }
#else
            original(ragdollInterface);
            return true;
#endif
        }

        DiagnosticState captureDiagnosticState(
            void* ragdollInterface,
            const FaultContext& fault) noexcept
        {
            DiagnosticState state{};
            if (native_memory::tryReadField(
                    ragdollInterface,
                    kInterfacePhysicsInterfaceOffset,
                    state.physicsInterface)) {
                state.readableFields |= kPhysicsInterfaceReadable;
            }
            if (native_memory::tryReadField(
                    ragdollInterface,
                    kInterfaceRagdollDataOffset,
                    state.ragdollData)) {
                state.readableFields |= kRagdollDataReadable;
            }
            if (state.physicsInterface != 0 && native_memory::tryReadField(
                    reinterpret_cast<const void*>(state.physicsInterface),
                    kPhysicsInterfaceWorldOffset,
                    state.resolvedWorld)) {
                state.readableFields |= kResolvedWorldReadable;
            }
            if (state.ragdollData != 0) {
                if (native_memory::tryReadField(
                        reinterpret_cast<const void*>(state.ragdollData),
                        kRagdollDataConstraintIdsOffset,
                        state.constraintIds)) {
                    state.readableFields |= kConstraintIdsReadable;
                }
                if (native_memory::tryReadField(
                        reinterpret_cast<const void*>(state.ragdollData),
                        kRagdollDataConstraintCountOffset,
                        state.ragdollConstraintCount)) {
                    state.readableFields |= kRagdollConstraintCountReadable;
                }
            }

            const auto world = fault.rsi != 0 ? fault.rsi : state.resolvedWorld;
            if (world != 0) {
                const auto* worldPtr = reinterpret_cast<const void*>(world);
                if (native_memory::tryReadField(worldPtr, 0, state.worldVtable)) {
                    state.readableFields |= kWorldVtableReadable;
                }
                if (native_memory::tryReadField(
                        worldPtr,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_BodyArrayPtr),
                        state.bodyArray)) {
                    state.readableFields |= kWorldBodyArrayReadable;
                }
                if (native_memory::tryReadField(
                        worldPtr,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_ConstraintArrayPtr),
                        state.constraintArray)) {
                    state.readableFields |= kWorldConstraintArrayReadable;
                }
                if (native_memory::tryReadField(
                        worldPtr,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_ConstraintCount),
                        state.worldConstraintCount)) {
                    state.readableFields |= kWorldConstraintCountReadable;
                }
            }

            const auto destroyingVtable =
                s_destroyingWorldVtable.load(std::memory_order_acquire);
            state.destroying = destroyingVtable != 0 &&
                               state.worldVtable == destroyingVtable;
            if (state.constraintArray == 0 &&
                fault.r12 % kConstraintStride == 0) {
                const auto id = fault.r12 / kConstraintStride;
                if (id <= kInvalidConstraintId) {
                    state.faultConstraintId = static_cast<std::uint32_t>(id);
                }
            }
            return state;
        }

        void recordSuppressedFault(
            void* ragdollInterface,
            const std::uintptr_t caller) noexcept
        {
            const auto fault = t_faultContext;
            const auto occurrence =
                s_suppressedFaultCount.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (!fault.captured || !shouldLogOccurrence(occurrence)) {
                return;
            }

            const auto state = captureDiagnosticState(ragdollInterface, fault);
            const auto callerRva = gameTextRelativeAddress(caller);
            const auto exceptionRva =
                gameTextRelativeAddress(fault.exceptionAddress);
            const auto insidePhysicsStep =
                havok_world_lock::detail::currentThreadInsidePhysicsStep();

            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Suppressed native ragdoll teardown fault: occurrence={} thread={} physicsStep={} interface={:p} caller=0x{:X} callerRva=0x{:X} exceptionRva=0x{:X} access=0x{:X}",
                occurrence,
                GetCurrentThreadId(),
                insidePhysicsStep ? "yes" : "no",
                ragdollInterface,
                caller,
                callerRva,
                exceptionRva,
                fault.accessAddress);
            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Ragdoll teardown state: readable=0x{:03X} destroying={} contextWorld=0x{:X} resolvedWorld=0x{:X} worldVtable=0x{:X} bodyArray=0x{:X} constraintArray=0x{:X} worldConstraintCount={} faultConstraintId={} ragdollData=0x{:X} constraintIds=0x{:X} ragdollConstraintCount={} regs(rax/rbx/rcx/rdx/r12/r15/rsp)=0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}",
                state.readableFields,
                state.destroying ? "yes" : "no",
                fault.rsi,
                state.resolvedWorld,
                state.worldVtable,
                state.bodyArray,
                state.constraintArray,
                state.worldConstraintCount,
                state.faultConstraintId,
                state.ragdollData,
                state.constraintIds,
                state.ragdollConstraintCount,
                fault.rax,
                fault.rbx,
                fault.rcx,
                fault.rdx,
                fault.r12,
                fault.r15,
                fault.rsp);
        }

        __declspec(noinline) void onUpdateConstraints(
            void* ragdollInterface) noexcept
        {
            const auto caller =
                reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            if (invokeOriginal(ragdollInterface)) {
                return;
            }

            /*
             * The verified fault instruction executes after the native read
             * lock was released. Returning here therefore abandons only this
             * teardown update and does not strand a world lock.
             */
            recordSuppressedFault(ragdollInterface, caller);
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
                "Native ragdoll safety unavailable: unsupported runtime");
            return false;
        }

        s_expectedFaultAddress.store(
            REL::Offset(offsets::kFault_HkbnpRagdollInterface_ConstraintRead).address(),
            std::memory_order_release);
        s_destroyingWorldVtable.store(
            REL::Offset(offsets::kVtable_HknpWorldDestroying).address(),
            std::memory_order_release);

        void* original = reinterpret_cast<void*>(s_originalUpdateConstraints);
        const bool installed = entry_trampoline_hook::install(
            "hkbnp ragdoll teardown safety",
            offsets::kFunc_HkbnpRagdollInterface_UpdateConstraints,
            kExpectedUpdateConstraintsEntry.data(),
            kExpectedUpdateConstraintsEntry.size(),
            reinterpret_cast<void*>(&onUpdateConstraints),
            original);
        s_originalUpdateConstraints =
            reinterpret_cast<UpdateConstraintsFn>(original);
        const bool ready = installed && s_originalUpdateConstraints != nullptr;
        s_installed.store(ready, std::memory_order_release);
        if (!ready) {
            ROCK_LOG_ERROR(Init,
                "Native ragdoll safety unavailable: hook installation failed");
            return false;
        }

        ROCK_LOG_INFO(Init,
            "Native ragdoll teardown safety active: exactFaultRva=0x{:X}",
            offsets::kFault_HkbnpRagdollInterface_ConstraintRead);
        return true;
    }
}

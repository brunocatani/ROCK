#include "physics-interaction/native/NativeCharacterProxySafety.h"

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

namespace rock::native_character_proxy_safety
{
    namespace
    {
        using ResolveWorldFn = void* (*)(void*);

        /*
         * Fallout4VR.exe 1.2.72 character-proxy world accessor at
         * 0x141E4DEC0. These are complete, position-independent entry
         * instructions verified from raw disassembly and Ghidra on
         * 2026-08-30.
         *
         * The accessor normally returns null when its body is unavailable.
         * Its verified destructor caller tests that null before touching the
         * returned bhkWorld, so null is also the native teardown result.
         */
        constexpr std::array<std::uint8_t, 14> kExpectedWorldAccessorEntry{
            0x40, 0x53,
            0x48, 0x83, 0xEC, 0x20,
            0x48, 0x8B, 0x01,
            0x48, 0x8D, 0x54, 0x24, 0x30,
        };

        constexpr std::uintptr_t kMaximumNullDerivedReadAddress = 0x1000'0000;
        constexpr std::uintptr_t kBodyStride = 0x90;
        constexpr std::uintptr_t kBodyFieldOffset = 0x6C;
        constexpr std::ptrdiff_t kControllerPhysicsSystemOffset = 0x20;
        constexpr std::ptrdiff_t kControllerSystemBodyIndexOffset = 0x28;
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        enum ReadableField : std::uint32_t
        {
            kPhysicsSystemReadable = 1u << 0,
            kSystemBodyIndexReadable = 1u << 1,
            kPhysicsSystemInstanceReadable = 1u << 2,
            kWorldReadable = 1u << 3,
            kWorldVtableReadable = 1u << 4,
            kWorldBodyArrayReadable = 1u << 5,
            kWorldConstraintArrayReadable = 1u << 6,
            kWorldConstraintCountReadable = 1u << 7,
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
            std::uintptr_t rsp = 0;
        };

        struct DiagnosticState
        {
            std::uint32_t readableFields = 0;
            std::uintptr_t physicsSystem = 0;
            std::uint32_t systemBodyIndex = 0;
            std::uintptr_t physicsSystemInstance = 0;
            std::uintptr_t world = 0;
            std::uintptr_t worldVtable = 0;
            std::uintptr_t bodyArray = 0;
            std::uintptr_t constraintArray = 0;
            std::uint32_t worldConstraintCount = 0;
            std::uint32_t faultBodyId = kInvalidBodyId;
            bool destroying = false;
        };

        ResolveWorldFn s_originalResolveWorld = nullptr;
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

        int captureVerifiedBodyFieldFault(
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
            const auto bodyAddress = static_cast<std::uintptr_t>(context->Rax);
            if (bodyAddress >= kMaximumNullDerivedReadAddress ||
                bodyAddress % kBodyStride != 0 ||
                accessAddress != bodyAddress + kBodyFieldOffset) {
                return EXCEPTION_CONTINUE_SEARCH;
            }

            t_faultContext = {
                .captured = true,
                .exceptionAddress = exceptionAddress,
                .accessAddress = accessAddress,
                .rax = bodyAddress,
                .rbx = static_cast<std::uintptr_t>(context->Rbx),
                .rcx = static_cast<std::uintptr_t>(context->Rcx),
                .rdx = static_cast<std::uintptr_t>(context->Rdx),
                .rsi = static_cast<std::uintptr_t>(context->Rsi),
                .rdi = static_cast<std::uintptr_t>(context->Rdi),
                .rsp = static_cast<std::uintptr_t>(context->Rsp),
            };
            return EXCEPTION_EXECUTE_HANDLER;
        }

        [[nodiscard]] __declspec(noinline) bool invokeOriginal(
            void* controller,
            void*& result) noexcept
        {
            auto* original = s_originalResolveWorld;
            if (!original) {
                result = nullptr;
                return false;
            }

            t_faultContext = {};
#if defined(_MSC_VER)
            __try {
                result = original(controller);
                return true;
            } __except (captureVerifiedBodyFieldFault(GetExceptionInformation())) {
                result = nullptr;
                return false;
            }
#else
            result = original(controller);
            return true;
#endif
        }

        DiagnosticState captureDiagnosticState(
            void* controller,
            const FaultContext& fault) noexcept
        {
            DiagnosticState state{};
            if (native_memory::tryReadField(
                    controller,
                    kControllerPhysicsSystemOffset,
                    state.physicsSystem)) {
                state.readableFields |= kPhysicsSystemReadable;
            }
            if (native_memory::tryReadField(
                    controller,
                    kControllerSystemBodyIndexOffset,
                    state.systemBodyIndex)) {
                state.readableFields |= kSystemBodyIndexReadable;
            }
            if (state.physicsSystem != 0 && native_memory::tryReadField(
                    reinterpret_cast<const void*>(state.physicsSystem),
                    static_cast<std::ptrdiff_t>(offsets::kBhkPhysicsSystem_Instance),
                    state.physicsSystemInstance)) {
                state.readableFields |= kPhysicsSystemInstanceReadable;
            }
            if (state.physicsSystemInstance != 0 && native_memory::tryReadField(
                    reinterpret_cast<const void*>(state.physicsSystemInstance),
                    static_cast<std::ptrdiff_t>(offsets::kHknpPhysicsSystemInstance_World),
                    state.world)) {
                state.readableFields |= kWorldReadable;
            }
            if (state.world != 0) {
                const auto* world = reinterpret_cast<const void*>(state.world);
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
                if (native_memory::tryReadField(
                        world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_ConstraintCount),
                        state.worldConstraintCount)) {
                    state.readableFields |= kWorldConstraintCountReadable;
                }
            }

            const auto destroyingVtable =
                s_destroyingWorldVtable.load(std::memory_order_acquire);
            state.destroying = destroyingVtable != 0 &&
                               state.worldVtable == destroyingVtable;
            if (fault.rax % kBodyStride == 0) {
                const auto id = fault.rax / kBodyStride;
                if (id <= kInvalidBodyId) {
                    state.faultBodyId = static_cast<std::uint32_t>(id);
                }
            }
            return state;
        }

        void recordSuppressedFault(
            void* controller,
            const std::uintptr_t caller) noexcept
        {
            const auto fault = t_faultContext;
            const auto occurrence =
                s_suppressedFaultCount.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (!fault.captured || !shouldLogOccurrence(occurrence)) {
                return;
            }

            const auto state = captureDiagnosticState(controller, fault);
            const auto callerRva = gameTextRelativeAddress(caller);
            const auto exceptionRva =
                gameTextRelativeAddress(fault.exceptionAddress);
            const auto insidePhysicsStep =
                havok_world_lock::detail::currentThreadInsidePhysicsStep();

            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Suppressed native character-proxy teardown fault: occurrence={} thread={} physicsStep={} controller={:p} caller=0x{:X} callerRva=0x{:X} exceptionRva=0x{:X} access=0x{:X}",
                occurrence,
                GetCurrentThreadId(),
                insidePhysicsStep ? "yes" : "no",
                controller,
                caller,
                callerRva,
                exceptionRva,
                fault.accessAddress);
            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Character-proxy teardown state: readable=0x{:02X} destroying={} physicsSystem=0x{:X} instance=0x{:X} world=0x{:X} worldVtable=0x{:X} bodyArray=0x{:X} constraintArray=0x{:X} worldConstraintCount={} systemBodyIndex={} faultBodyId={} computedBody=0x{:X} regs(rbx/rcx/rdx/rsi/rdi/rsp)=0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}/0x{:X}",
                state.readableFields,
                state.destroying ? "yes" : "no",
                state.physicsSystem,
                state.physicsSystemInstance,
                state.world,
                state.worldVtable,
                state.bodyArray,
                state.constraintArray,
                state.worldConstraintCount,
                state.systemBodyIndex,
                state.faultBodyId,
                fault.rax,
                fault.rbx,
                fault.rcx,
                fault.rdx,
                fault.rsi,
                fault.rdi,
                fault.rsp);
        }

        __declspec(noinline) void* onResolveWorld(void* controller) noexcept
        {
            const auto caller =
                reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            void* result = nullptr;
            if (invokeOriginal(controller, result)) {
                return result;
            }

            /*
             * The body pointer helper releases world+0x690 before the faulting
             * field read. Returning the accessor's native null result cannot
             * strand the read lock or expose a partially completed mutation.
             */
            recordSuppressedFault(controller, caller);
            return nullptr;
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
                "Native character-proxy safety unavailable: unsupported runtime");
            return false;
        }

        s_expectedFaultAddress.store(
            REL::Offset(offsets::kFault_BhkCharProxyController_BodyFieldRead).address(),
            std::memory_order_release);
        s_destroyingWorldVtable.store(
            REL::Offset(offsets::kVtable_HknpWorldDestroying).address(),
            std::memory_order_release);

        void* original = reinterpret_cast<void*>(s_originalResolveWorld);
        const bool installed = entry_trampoline_hook::install(
            "bhk character-proxy teardown safety",
            offsets::kFunc_BhkCharProxyController_WorldAccessor,
            kExpectedWorldAccessorEntry.data(),
            kExpectedWorldAccessorEntry.size(),
            reinterpret_cast<void*>(&onResolveWorld),
            original);
        s_originalResolveWorld = reinterpret_cast<ResolveWorldFn>(original);
        const bool ready = installed && s_originalResolveWorld != nullptr;
        s_installed.store(ready, std::memory_order_release);
        if (!ready) {
            ROCK_LOG_ERROR(Init,
                "Native character-proxy safety unavailable: hook installation failed");
            return false;
        }

        ROCK_LOG_INFO(Init,
            "Native character-proxy teardown safety active: exactFaultRva=0x{:X}",
            offsets::kFault_BhkCharProxyController_BodyFieldRead);
        return true;
    }
}

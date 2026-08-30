#include "physics-interaction/native/NativeShapeCastSafety.h"

#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/EntryTrampolineHook.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokWorldLock.h"
#include "physics-interaction/native/NativeMemory.h"

#include "RE/Havok/hknpWorld.h"

#include <REL/Relocation.h>
#include <Windows.h>
#include <intrin.h>

#include <array>
#include <atomic>
#include <bit>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace rock::native_shape_cast_safety
{
    namespace
    {
        using WorldCastShapeFn = void (*)(
            RE::hknpWorld*,
            void*,
            void*,
            void*,
            void*);

        using DispatcherCastShapeFn = void (*)(
            void*,
            void*,
            void*,
            void*,
            void*,
            void*,
            void*,
            void*,
            bool,
            void*,
            void*);

        /*
         * Both prologues are whole-instruction and position-independent. They
         * were verified from Fallout4VR.exe 1.2.72 raw disassembly alongside
         * the 2026-08-29 castShape crash at module+0x15FFD43.
         */
        constexpr std::array<std::uint8_t, 15> kExpectedWorldCastShapeEntry{
            0x48, 0x89, 0x5C, 0x24, 0x08,
            0x48, 0x89, 0x6C, 0x24, 0x10,
            0x48, 0x89, 0x74, 0x24, 0x18,
        };

        constexpr std::array<std::uint8_t, 14> kExpectedDispatcherCastShapeEntry{
            0x4C, 0x8B, 0xDC,
            0x49, 0x89, 0x6B, 0x10,
            0x49, 0x89, 0x73, 0x18,
            0x57,
            0x41, 0x54,
        };

        constexpr std::ptrdiff_t kShapeCastQueryShapeOffset = 0x20;
        constexpr std::ptrdiff_t kShapeQueryInfoBodyOffset = 0x00;
        constexpr std::ptrdiff_t kQueryFilterMaterialOffset = 0x00;
        constexpr std::ptrdiff_t kQueryFilterInfoOffset = 0x04;
        constexpr std::ptrdiff_t kQueryFilterCollisionObjectOffset = 0x08;
        constexpr std::uintptr_t kHknpBodyStride = 0x90;
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        enum ReadableField : std::uint32_t
        {
            kWorldVtableReadable = 1u << 0,
            kBodyArrayReadable = 1u << 1,
            kBroadPhaseReadable = 1u << 2,
            kWorldDispatcherReadable = 1u << 3,
            kInternalLockReadable = 1u << 4,
            kAccessLockReadable = 1u << 5,
            kQueryShapeReadable = 1u << 6,
            kCandidateBodyReadable = 1u << 7,
            kTargetFilterReadable = 1u << 8,
        };

        struct WorldCastContext
        {
            RE::hknpWorld* world = nullptr;
            std::uintptr_t caller = 0;
            std::uint32_t depth = 0;
        };

        struct FastWorldState
        {
            bool destroying = false;
            std::uintptr_t vtable = 0;
            std::uintptr_t dispatcher = 0;
        };

        struct DiagnosticState
        {
            std::uint32_t readableFields = 0;
            std::uintptr_t worldVtable = 0;
            std::uintptr_t bodyArray = 0;
            std::uintptr_t broadPhase = 0;
            std::uintptr_t worldDispatcher = 0;
            std::uint64_t internalLock = 0;
            std::uint64_t accessLock = 0;
            std::uintptr_t queryShape = 0;
            std::uintptr_t candidateBody = 0;
            std::uintptr_t collisionObject = 0;
            std::uint32_t candidateBodyId = kInvalidBodyId;
            std::uint32_t collisionFilterInfo = 0;
            std::uint16_t materialId = 0;
            bool destroying = false;
        };

        WorldCastShapeFn s_originalWorldCastShape = nullptr;
        DispatcherCastShapeFn s_originalDispatcherCastShape = nullptr;
        std::atomic<bool> s_dispatcherGuardInstalled{ false };
        std::atomic<std::uintptr_t> s_destroyingWorldVtable{ 0 };
        std::atomic<std::uint64_t> s_suppressedCastCount{ 0 };
        thread_local WorldCastContext t_worldCastContext{};
        thread_local bool t_recordingSuppression = false;

        class ScopedWorldCastContext
        {
        public:
            ScopedWorldCastContext(RE::hknpWorld* world, const std::uintptr_t caller) noexcept :
                _previous(t_worldCastContext)
            {
                t_worldCastContext = {
                    .world = world,
                    .caller = caller,
                    .depth = _previous.depth + 1,
                };
            }

            ~ScopedWorldCastContext() noexcept
            {
                t_worldCastContext = _previous;
            }

            ScopedWorldCastContext(const ScopedWorldCastContext&) = delete;
            ScopedWorldCastContext& operator=(const ScopedWorldCastContext&) = delete;

        private:
            WorldCastContext _previous{};
        };

        [[nodiscard]] bool tryReadFastWorldState(
            RE::hknpWorld* world,
            FastWorldState& out) noexcept
        {
            out = {};
            if (!world) {
                return false;
            }

#if defined(_MSC_VER)
            __try {
                const auto base = reinterpret_cast<std::uintptr_t>(world);
                out.vtable = *reinterpret_cast<const std::uintptr_t*>(base);
                out.dispatcher = *reinterpret_cast<const std::uintptr_t*>(
                    base + offsets::kHknpWorld_CollisionQueryDispatcher);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            const auto base = reinterpret_cast<std::uintptr_t>(world);
            out.vtable = *reinterpret_cast<const std::uintptr_t*>(base);
            out.dispatcher = *reinterpret_cast<const std::uintptr_t*>(
                base + offsets::kHknpWorld_CollisionQueryDispatcher);
#endif

            const auto destroyingVtable =
                s_destroyingWorldVtable.load(std::memory_order_acquire);
            out.destroying = destroyingVtable != 0 && out.vtable == destroyingVtable;
            return true;
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

        [[nodiscard]] bool shouldLogOccurrence(
            const std::uint64_t occurrence) noexcept
        {
            return occurrence <= 4 || std::has_single_bit(occurrence);
        }

        DiagnosticState captureDiagnosticState(
            const WorldCastContext& context,
            void* query,
            void* targetFilterData,
            void* targetShapeInfo) noexcept
        {
            DiagnosticState state{};
            if (context.world) {
                if (native_memory::tryReadField(
                        context.world,
                        0,
                        state.worldVtable)) {
                    state.readableFields |= kWorldVtableReadable;
                }
                if (native_memory::tryReadField(
                        context.world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_BodyArrayPtr),
                        state.bodyArray)) {
                    state.readableFields |= kBodyArrayReadable;
                }
                if (native_memory::tryReadField(
                        context.world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_BroadPhase),
                        state.broadPhase)) {
                    state.readableFields |= kBroadPhaseReadable;
                }
                if (native_memory::tryReadField(
                        context.world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_CollisionQueryDispatcher),
                        state.worldDispatcher)) {
                    state.readableFields |= kWorldDispatcherReadable;
                }
                if (native_memory::tryReadField(
                        context.world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_InternalQueryLock),
                        state.internalLock)) {
                    state.readableFields |= kInternalLockReadable;
                }
                if (native_memory::tryReadField(
                        context.world,
                        static_cast<std::ptrdiff_t>(offsets::kHknpWorld_AccessLock),
                        state.accessLock)) {
                    state.readableFields |= kAccessLockReadable;
                }
            }

            const auto destroyingVtable =
                s_destroyingWorldVtable.load(std::memory_order_acquire);
            state.destroying = destroyingVtable != 0 &&
                               state.worldVtable == destroyingVtable;

            if (query && native_memory::tryReadField(
                             query,
                             kShapeCastQueryShapeOffset,
                             state.queryShape)) {
                state.readableFields |= kQueryShapeReadable;
            }
            if (targetShapeInfo && native_memory::tryReadField(
                                           targetShapeInfo,
                                           kShapeQueryInfoBodyOffset,
                                           state.candidateBody)) {
                state.readableFields |= kCandidateBodyReadable;
            }
            if (targetFilterData) {
                std::uint16_t materialId = 0;
                std::uint32_t collisionFilterInfo = 0;
                std::uintptr_t collisionObject = 0;
                const bool filterReadable =
                    native_memory::tryReadField(
                        targetFilterData,
                        kQueryFilterMaterialOffset,
                        materialId) &&
                    native_memory::tryReadField(
                        targetFilterData,
                        kQueryFilterInfoOffset,
                        collisionFilterInfo) &&
                    native_memory::tryReadField(
                        targetFilterData,
                        kQueryFilterCollisionObjectOffset,
                        collisionObject);
                if (filterReadable) {
                    state.materialId = materialId;
                    state.collisionFilterInfo = collisionFilterInfo;
                    state.collisionObject = collisionObject;
                    state.readableFields |= kTargetFilterReadable;
                }
            }

            if ((state.readableFields &
                    (kBodyArrayReadable | kCandidateBodyReadable)) ==
                    (kBodyArrayReadable | kCandidateBodyReadable) &&
                state.bodyArray != 0 && state.candidateBody >= state.bodyArray) {
                const auto byteOffset = state.candidateBody - state.bodyArray;
                const auto bodyIndex = byteOffset / kHknpBodyStride;
                if (byteOffset % kHknpBodyStride == 0 &&
                    bodyIndex <= (std::numeric_limits<std::uint32_t>::max)()) {
                    state.candidateBodyId = static_cast<std::uint32_t>(bodyIndex);
                }
            }

            return state;
        }

        void recordSuppressedCast(
            const char* stage,
            const WorldCastContext& context,
            const std::uintptr_t dispatcherCaller,
            void* queryContext,
            void* query,
            void* targetShape,
            void* targetFilterData,
            void* targetShapeInfo) noexcept
        {
            const auto occurrence =
                s_suppressedCastCount.fetch_add(1, std::memory_order_acq_rel) + 1;
            if (!shouldLogOccurrence(occurrence) || t_recordingSuppression) {
                return;
            }

            t_recordingSuppression = true;
            const auto state = captureDiagnosticState(
                context,
                query,
                targetFilterData,
                targetShapeInfo);
            const auto worldCallerRva = gameTextRelativeAddress(context.caller);
            const auto dispatcherCallerRva =
                gameTextRelativeAddress(dispatcherCaller);
            const auto insidePhysicsStep =
                havok_world_lock::detail::currentThreadInsidePhysicsStep();

            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Suppressed unsafe native shape cast: stage={} occurrence={} thread={} physicsStep={} depth={} world={:p} worldCaller=0x{:X} worldCallerRva=0x{:X} dispatcherCaller=0x{:X} dispatcherCallerRva=0x{:X} queryContext={:p}",
                stage,
                occurrence,
                GetCurrentThreadId(),
                insidePhysicsStep ? "yes" : "no",
                context.depth,
                static_cast<void*>(context.world),
                context.caller,
                worldCallerRva,
                dispatcherCaller,
                dispatcherCallerRva,
                queryContext);
            ROCK_LOG_CRITICAL(PhysicsSafety,
                "Unsafe shape-cast state: readable=0x{:03X} destroying={} worldVtable=0x{:X} worldDispatcher=0x{:X} broadPhase=0x{:X} bodyArray=0x{:X} internalLock=0x{:016X} accessLock=0x{:016X} query={:p} queryShape=0x{:X} targetShape={:p} candidateBody=0x{:X} bodyId={} material={} filter=0x{:08X} layer={} collisionObject=0x{:X}",
                state.readableFields,
                state.destroying ? "yes" : "no",
                state.worldVtable,
                state.worldDispatcher,
                state.broadPhase,
                state.bodyArray,
                state.internalLock,
                state.accessLock,
                query,
                state.queryShape,
                targetShape,
                state.candidateBody,
                state.candidateBodyId,
                state.materialId,
                state.collisionFilterInfo,
                state.collisionFilterInfo & 0x7Fu,
                state.collisionObject);
            t_recordingSuppression = false;
        }

        __declspec(noinline) void onWorldCastShape(
            RE::hknpWorld* world,
            void* query,
            void* queryShapeInfo,
            void* collector,
            void* startPointCollector) noexcept
        {
            const auto caller =
                reinterpret_cast<std::uintptr_t>(_ReturnAddress());
            FastWorldState worldState{};
            const bool worldReadable = tryReadFastWorldState(world, worldState);
            if (!world || !worldReadable || worldState.destroying ||
                worldState.dispatcher == 0) {
                const char* stage = !world ?
                    "world-null" :
                    !worldReadable ?
                        "world-unreadable" :
                        worldState.destroying ?
                            "world-destroying" :
                            "world-dispatcher-null";
                const WorldCastContext context{
                    .world = world,
                    .caller = caller,
                    .depth = t_worldCastContext.depth + 1,
                };
                recordSuppressedCast(
                    stage,
                    context,
                    0,
                    nullptr,
                    query,
                    nullptr,
                    nullptr,
                    nullptr);
                return;
            }

            auto* original = s_originalWorldCastShape;
            if (!original) {
                return;
            }

            const ScopedWorldCastContext contextScope{ world, caller };
            original(
                world,
                query,
                queryShapeInfo,
                collector,
                startPointCollector);
        }

        __declspec(noinline) void onDispatcherCastShape(
            void* dispatcher,
            void* queryContext,
            void* query,
            void* queryShapeInfo,
            void* targetShape,
            void* targetFilterData,
            void* targetShapeInfo,
            void* targetTransform,
            const bool queryAgainstTarget,
            void* collector,
            void* startPointCollector) noexcept
        {
            if (!dispatcher) {
                recordSuppressedCast(
                    "dispatcher-null-after-world-entry",
                    t_worldCastContext,
                    reinterpret_cast<std::uintptr_t>(_ReturnAddress()),
                    queryContext,
                    query,
                    targetShape,
                    targetFilterData,
                    targetShapeInfo);
                return;
            }

            auto* original = s_originalDispatcherCastShape;
            if (!original) {
                return;
            }
            original(
                dispatcher,
                queryContext,
                query,
                queryShapeInfo,
                targetShape,
                targetFilterData,
                targetShapeInfo,
                targetTransform,
                queryAgainstTarget,
                collector,
                startPointCollector);
        }
    }

    bool install() noexcept
    {
        if (s_dispatcherGuardInstalled.load(std::memory_order_acquire)) {
            return true;
        }
        if (!REL::Module::IsVR() ||
            REL::Module::get().version() != F4SE::RUNTIME_VR_1_2_72) {
            ROCK_LOG_ERROR(Init,
                "Native shape-cast safety unavailable: unsupported runtime");
            return false;
        }

        s_destroyingWorldVtable.store(
            REL::Offset(offsets::kVtable_HknpWorldDestroying).address(),
            std::memory_order_release);

        void* dispatcherOriginal =
            reinterpret_cast<void*>(s_originalDispatcherCastShape);
        const bool dispatcherInstalled = entry_trampoline_hook::install(
            "hknp collision-query dispatcher safety",
            offsets::kFunc_HknpCollisionQueryDispatcherBase_CastShape,
            kExpectedDispatcherCastShapeEntry.data(),
            kExpectedDispatcherCastShapeEntry.size(),
            reinterpret_cast<void*>(&onDispatcherCastShape),
            dispatcherOriginal);
        s_originalDispatcherCastShape =
            reinterpret_cast<DispatcherCastShapeFn>(dispatcherOriginal);
        const bool dispatcherReady = dispatcherInstalled &&
                                     s_originalDispatcherCastShape != nullptr;
        s_dispatcherGuardInstalled.store(
            dispatcherReady,
            std::memory_order_release);
        if (!dispatcherReady) {
            ROCK_LOG_ERROR(Init,
                "Native shape-cast safety unavailable: dispatcher guard installation failed");
            return false;
        }

        void* worldOriginal = reinterpret_cast<void*>(s_originalWorldCastShape);
        const bool worldInstalled = entry_trampoline_hook::install(
            "hknp world shape-cast context",
            offsets::kFunc_HknpWorld_CastShape,
            kExpectedWorldCastShapeEntry.data(),
            kExpectedWorldCastShapeEntry.size(),
            reinterpret_cast<void*>(&onWorldCastShape),
            worldOriginal);
        s_originalWorldCastShape =
            reinterpret_cast<WorldCastShapeFn>(worldOriginal);
        const bool worldReady = worldInstalled &&
                                s_originalWorldCastShape != nullptr;
        if (!worldReady) {
            ROCK_LOG_WARN(Init,
                "Native shape-cast dispatcher guard is active without world/caller context");
        }

        ROCK_LOG_INFO(Init,
            "Native shape-cast safety active: dispatcherGuard=yes worldContext={}",
            worldReady ? "yes" : "no");
        return true;
    }
}

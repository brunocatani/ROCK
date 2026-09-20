/*
 * ROCK performance profiling is diagnostics-only because the interaction stack
 * must not let wall-clock timing influence gameplay state. The alternative was
 * to scatter timer calls directly through hand, body, weapon, and overlay code;
 * centralizing the boundary here keeps the high-resolution clock isolated in
 * the implementation file while still giving future optimization passes
 * measured subsystem costs.
 */
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rock::performance_profiler
{
    enum class Scope : std::uint8_t
    {
        FrameUpdate = 0,
        RuntimePreparation,
        WeaponEquipTransition,
        AuthoredPrimaryGrip,
        InteractionUpdate,
        EquippedWeaponInteraction,
        InteractionFinalize,
        RenderedHandCapture,
        ProviderPublication,
        HandColliderUpdate,
        BodyColliderUpdate,
        GeneratedColliderPhysicsFlush,
        WeaponCollision,
        WeaponCollisionTransforms,
        WeaponContactProbe,
        WeaponEmitterRefresh,
        WeaponIdentityRead,
        WeaponVisualObservation,
        GeneratedBodyContactRegistry,
        WeaponColliderBuild,
        WeaponColliderCreate,
        WeaponGapDecomposition,
        WeaponMeshExtraction,
        WeaponPointFitting,
        WeaponGapColliderBuild,
        WeaponGapColliderCreate,
        TwoHandedGripStart,
        EquippedWeaponFingerPoseCapture,
        SupportGripSuppression,
        SelectionCasts,
        DynamicHandCollisionFrame,
        DynamicHandCollisionPhysicsDrive,
        DynamicHandCollisionPostSolve,
        DebugOverlayPublish,
        DebugOverlayRender,
        ContactResolve,
        NativeContactCallback,
        NativeMeleeCallback,
        NativeMeleeDispatch,
        GrabAcquisitionBodyScan,
        GrabAcquisitionActivePrep,
        GrabMeshExtraction,
        GrabNearbyDampingBegin,
        GrabHeldObjectUpdate,
        GrabAuthorityFlush,
        GrabAuthorityAfterSolveDiagnostics,
        GrabNearbyDampingRestore,
        GrabNearbyDampingRestoreBodySearch,
        FramePrelude,
        FrameBeginPreparation,
        WeaponPresentation,
        FinalPresentation,
        HandFrameResolve,
        HandBoneCapture,
        BodyBoneCapture,
        FingerBoneCapture,
        SelectionHitProcessing,
        PhysicsSystemBodyScan,
        NativePlayerRefresh,
        NativePlayerPairFilter,
        HeldSceneWriter,
        ProviderFrameDispatch,
        ProviderFrameConsumer,
        ProviderAnimationDispatch,
        ProviderAnimationConsumer,
        NativeWorldReadWait,
        CallbackQuiescenceWait,
        NearbyDampingWait,
        NativeIdleGripHarvest,
        UnattributedMemoryQueries,
        GrabAcquisition,
        GrabSurfaceResolution,
        GrabFingerSolve,
        GrabFingerIndexBuild,
        GrabFingerPadProbes,
        NativePhysicsUpdate,
        NativePhysicsCollideInterval,
        NativePhysicsSolveInterval,
        Count
    };

    enum class Counter : std::uint8_t
    {
        WeaponRebuildQueued = 0,
        WeaponRebuildCanceled,
        WeaponRebuildCompleted,
        WeaponRebuildVisualRootDeferred,
        WeaponRebuildVisualStableWait,
        WeaponRebuildVisualSourceUnavailableRetained,
        WeaponRebuildVisualSourceUnavailableRetainExpired,
        WeaponRebuildReasonDriveRequested,
        WeaponRebuildReasonKeyChanged,
        WeaponRebuildReasonMissingBodies,
        WeaponKeyChangeVisualOnly,
        WeaponKeyChangeIdentityOnly,
        WeaponKeyChangeVisualAndIdentity,
        GrabAcquisitionCachePrewarm,
        GrabAcquisitionCacheHit,
        GrabAcquisitionCacheMiss,
        GrabAcquisitionCacheInvalidated,
        GrabNearbyDampingRestoreFailed,
        NativeMeleeRockPartnerDropped,
        NativeMeleeDecodeFailed,
        NativeReadRangeRejected,
        NativeWriteRangeRejected,
        PhysicsTimingSubstepsIncreased,
        Count
    };

    enum class ValueMetric : std::uint8_t
    {
        WeaponBuildVisibleTriShapes = 0,
        WeaponBuildGeneratedSources,
        WeaponBuildBodiesCreated,
        WeaponBuildTransientReloadSources,
        WeaponBuildBodyCount,
        WeaponBuildGapMode,
        WeaponBuildConvexes,
        WeaponBuildPoints,
        GrabAcquisitionVisitedNodes,
        GrabAcquisitionCollisionObjects,
        GrabAcquisitionBodyIds,
        GrabMeshTriangles,
        GrabNearbyDampingMotions,
        EquippedWeaponFingerPoseSourceTriangles,
        EquippedWeaponFingerPoseSelectedTriangles,
        EquippedWeaponFingerPoseSpatialNodeVisits,
        EquippedWeaponFingerPoseTriangleTests,
        NativeMeleeCallbacksPerFrame,
        RenderedSkeletonBones,
        ControllerSkeletonBones,
        SelectionRawHits,
        PhysicsOriginalSubsteps,
        PhysicsRequestedSubsteps,
        PhysicsCompletedSubsteps,
        PhysicsRawDeltaMicroseconds,
        GeneratedHandBodies,
        GeneratedBodyBodies,
        GeneratedWeaponBodies,
        FingerPadCandidateTriangles,
        FingerPadTriangleTests,
        Count
    };

    inline constexpr std::size_t kOverlayMaxLines = 8;
    inline constexpr std::size_t kOverlayLineLength = 128;
    using OverlayLines = std::array<std::array<char, kOverlayLineLength>, kOverlayMaxLines>;

    void refreshSettings(bool enabled, int logIntervalFrames, int warmupFrames, bool overlayTextEnabled) noexcept;
    bool enabled() noexcept;
    void beginFrame() noexcept;
    void endFrame() noexcept;
    void addEventCount(Scope scope, std::uint64_t count = 1) noexcept;
    void addCounter(Counter counter, std::uint64_t count = 1) noexcept;
    void observeValue(ValueMetric metric, std::uint64_t value) noexcept;
    bool overlayTextEnabled() noexcept;
    std::uint32_t copyOverlayLines(OverlayLines& outLines) noexcept;

    enum class MemoryQueryKind : std::uint8_t { Read, Write, Execute };

    struct MemoryQuerySample
    {
        std::uint64_t startTicks{ 0 };
        Scope scope{ Scope::UnattributedMemoryQueries };
        bool active{ false };
    };

    // One of every 64 queries on each calling thread is timed. All admitted
    // queries are counted; no per-query allocation, logging, or shared lock.
    MemoryQuerySample beginMemoryQuery() noexcept;
    void endMemoryQuery(MemoryQuerySample sample, MemoryQueryKind kind, bool apiSucceeded) noexcept;

    // Callback-owned wall-time interval. Unlike ScopedTimer, this never changes
    // thread-local query attribution and may end in a later callback. A settings
    // reset invalidates pending intervals. End consumes the sample exactly once.
    struct IntervalSample
    {
        std::uint64_t startTicks{ 0 };
        std::uint64_t generation{ 0 };
    };
    IntervalSample beginInterval() noexcept;
    bool endInterval(Scope scope, IntervalSample& sample) noexcept;

    class ScopedTimer
    {
    public:
        explicit ScopedTimer(Scope scope) noexcept;
        ~ScopedTimer();

        ScopedTimer(const ScopedTimer&) = delete;
        ScopedTimer& operator=(const ScopedTimer&) = delete;
        ScopedTimer(ScopedTimer&&) = delete;
        ScopedTimer& operator=(ScopedTimer&&) = delete;

        void stop() noexcept;

    private:
        Scope _scope{ Scope::Count };
        std::uint64_t _startTicks{ 0 };
        bool _active{ false };
        // Attribution is a value, never a pointer to a stack timer: a native
        // SEH recovery can bypass C++ unwinding. Normal scopes restore the parent.
        Scope _parentScope{ Scope::UnattributedMemoryQueries };
    };

}

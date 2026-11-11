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
#include <source_location>

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
        EquippedWeaponFingerTriangleSelection,
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
        GrabHeldObjectUpdate,
        GrabAuthorityFlush,
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
        GrabSelectionValidation,
        GrabBodyPreparation,
        GrabProxyPreparation,
        GrabMeshCapturePreparation,
        GrabBodyResolution,
        GrabResolvedBodyCapture,
        GrabPivotEvidence,
        GrabPinchPocket,
        GrabCommitPreparation,
        GrabBodyFrameCapture,
        GrabSeatCapture,
        GrabFrozenCommit,
        GrabPostFreeze,
        GrabConstraintCommit,
        GrabLocalTriangleCapture,
        GrabTriangleSelection,
        MeshStaticExtraction,
        MeshDynamicExtraction,
        MeshSkinnedExtraction,
        MeshPointQuery,
        MeshDirectionalQuery,
        GrabMeshQueryIndexBuild,
        NativeSimulationPairFilter,
        NativePairProfileBatch,
        NativeImpactListener,
        NativeImpactDispatch,
        NativeImpactConsumer,
        NativeImpactManifoldObserve,
        RagdollComponentRead,
        RagdollBodyRefresh,
        GrabAuthoredPreparation,
        GrabProbeContact,
        GrabProbeMeshRead,
        GrabProbeIndex,
        GrabProbeSolve,
        GrabLibraryPose,
        EquippedGripController,
        EquippedGripScopeFrames,
        EquippedGripActivation,
        EquippedGripSolve,
        EquippedGripVisualReturn,
        EquippedNativeScopeRead,
        EquippedNativeScopeAnchor,
        WeaponProbePoseCapture,
        WeaponSourcePoseUpdate,
        WeaponCompoundPoseUpdate,
        WeaponKinematicDrive,
        WeaponDynamicDrive,
        LooseWeaponCollision,
        LooseWeaponLayerRepair,
        // Sub-steps of interactionFinalize.
        FinalizeGrabInput,
        FinalizeHeldObjects,
        FinalizeHandFeedback,
        FinalizeConsumers,
        FinalizeHandoff,
        // Sub-steps of equippedWeaponInteraction.
        EquippedObserve,
        EquippedRouting,
        EquippedContacts,
        EquippedGripStart,
        EquippedGripUpdate,
        EquippedStashDrop,
        EquippedPublish,
        // Sub-steps of finalPresentation.
        PresentedHandPoseCapture,
        FinalPoseCommit,
        PresentationPublish,
        // Sub-steps of generatedFlush (physics thread).
        FlushHands,
        FlushBody,
        FlushWeapons,
        FlushDynamicHands,
        FlushConsumers,
        // Sub-steps of interactionFinalize not covered above.
        FinalizeSelection,
        FinalizeContacts,
        HeldLooseWeapons,
        HeldObjectRest,
        HeldExclusions,
        // Sub-steps of runtimePreparation.
        PrepSettings,
        PrepRuntimeState,
        PrepGates,
        // Sub-steps of nativeIdleGripHarvest, which runs native animation code.
        HarvestGraphHolderStart,
        HarvestSubgraphRequest,
        HarvestClipSample,
        HarvestLoadPoll,
        HarvestJobRelease,
        HarvestWarmRelease,
        HarvestSubgraphRelease,
        HarvestHolderDestroy,
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
        NativeMeleeRockPartnerDropped,
        NativeMeleeDecodeFailed,
        NativeReadRangeRejected,
        NativeWriteRangeRejected,
        PhysicsTimingSubstepsIncreased,
        GrabAcquisitionPeerHeld,
        GrabAcquisitionEquippedTransfer,
        GrabAcquisitionSucceeded,
        GrabSingleBodyVisualOwnerAccepted,
        GrabAuthoredPositionsOnly,
        GrabAuthoredArrivalDynamicFallback,
        GrabAuthoredSurfaceWorkSkipped,
        GrabAuthoredSurfaceWorkRequired,
        GrabAuthoredPreparationInvalidated,
        WeaponProbeFramesEnabled,
        WeaponProbeFramesSkipped,
        WeaponProbePoseBatches,
        WeaponProbeBatchReuses,
        ContactPairBatchTruncated,
        NativeWeaponSelfPairsRejected,
        NativeWeaponOwnerUnresolved,
        NativeWeaponOwnerResolvedOther,
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
        EquippedWeaponFingerPoseSourceTriangles,
        EquippedWeaponFingerPoseSelectedTriangles,
        EquippedWeaponFingerSelectionMeasuredTriangles,
        EquippedWeaponFingerSelectionNodeVisits,
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
        GrabMeshStaticTriangles,
        GrabMeshDynamicTriangles,
        GrabMeshSkinnedTriangles,
        GrabMeshCaptureAttempts,
        GrabMeshPayloadBytes,
        MeshPointQueryTriangles,
        MeshDirectionalQueryTriangles,
        MeshPointQueryTriangleTests,
        GrabTriangleSelectionTests,
        MeshStaticVerticesTransformed,
        SimulationPairsInput,
        SimulationPairsNative,
        SimulationPairsKept,
        RagdollSystemBodies,
        RagdollSystemConstraints,
        RagdollConnectedBodies,
        MeshSkinnedVerticesSource,
        MeshSkinnedVerticesEvaluated,
        MeshSkinnedBonesSource,
        MeshSkinnedBonesEvaluated,
        WeaponProbePoseParts,
        WeaponProbeBoundsCandidates,
        WeaponProbeSurfaceCandidates,
        WeaponColliderDriveBodies,
        Count
    };

    void refreshSettings(bool enabled, int logIntervalFrames, int warmupFrames) noexcept;
    bool enabled() noexcept;
    void beginFrame() noexcept;
    void endFrame() noexcept;
    void addEventCount(Scope scope, std::uint64_t count = 1) noexcept;
    void addCounter(Counter counter, std::uint64_t count = 1) noexcept;
    void observeValue(ValueMetric metric, std::uint64_t value) noexcept;
    struct ContactPair;
    enum class ContactStage : std::uint8_t;
    // Diagnostic values only; ownership and collision decisions stay with callers.
    void observeContactPair(ContactPair pair, ContactStage stage) noexcept;

    // A memory query is one native_memory range check (a page-touch probe).
    enum class MemoryQueryKind : std::uint8_t
    {
        Read,
        Write
    };

    struct MemoryQuerySample
    {
        std::uint64_t startTicks{ 0 };
        Scope scope{ Scope::UnattributedMemoryQueries };
        bool active{ false };
    };

    // One of every 64 queries on each calling thread is timed. All admitted
    // queries are counted; no per-query allocation, logging, or shared lock.
    MemoryQuerySample beginMemoryQuery() noexcept;
    void endMemoryQuery(MemoryQuerySample sample, MemoryQueryKind kind) noexcept;

    // Counts guarded native reads per source call site; each window logs the
    // hottest sites, so per-frame pointer walks can be found and restructured.
    void countGuardedRead(const std::source_location& site) noexcept;

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

    enum class ConsumerKind : std::uint8_t
    {
        Frame,
        Animation
    };

    /*
     * Wall time spent inside one provider consumer callback, keyed by the
     * callback's address. Consumer time runs inside ROCK's frame, so external
     * profilers count it as ROCK; each window names the owning module and
     * offset of every callback so it can be attributed to its mod.
     */
    class ConsumerTimer
    {
    public:
        ConsumerTimer(ConsumerKind kind, const void* callback) noexcept;
        ~ConsumerTimer();

        ConsumerTimer(const ConsumerTimer&) = delete;
        ConsumerTimer& operator=(const ConsumerTimer&) = delete;
        ConsumerTimer(ConsumerTimer&&) = delete;
        ConsumerTimer& operator=(ConsumerTimer&&) = delete;

    private:
        std::uint64_t _key{ 0 };
        std::uint64_t _startTicks{ 0 };
    };
}

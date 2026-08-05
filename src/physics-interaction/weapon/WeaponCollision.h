#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <unordered_set>
#include <vector>

#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
#include "physics-interaction/native/PhysicsCallbackQuiescenceGate.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/weapon/WeaponTypes.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSemantics.h"

#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class NiAVObject;
    class NiNode;
    class NiTransform;
    class bhkWorld;
}

namespace rock
{

    inline constexpr std::uint32_t ROCK_WEAPON_LAYER = 44;

    struct WeaponVisualKeyStats
    {
        std::uint32_t rootCount = 0;
        std::uint32_t nodeCount = 0;
        std::uint32_t triShapeCount = 0;
        std::uint32_t visibleTriShapeCount = 0;
        std::uint32_t missingRendererCount = 0;
        std::uint32_t emptyGeometryCount = 0;
        std::uint32_t invisibleNodeCount = 0;
    };

    class WeaponCollision
    {
    public:
        WeaponCollision();

        void setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate) { _physicsCallbackGate = gate; }

        struct WeaponBodySnapshot
        {
            std::uint64_t generationKey{ 0 };
            std::uint32_t count{ 0 };
            std::array<std::uint32_t, MAX_WEAPON_COLLISION_BODIES> bodyIds{};
        };

        struct ReleaseGeometrySnapshot
        {
            float leverGameUnits{ 0.0f };
            bool hasCapturedWeaponWorld{ false };
            RE::NiTransform capturedWeaponWorld{};
        };

        /*
         * Immutable, generation/ownership/form-keyed view of the assembled
         * optical-sight geometry. The anchor is expressed in the equipped
         * weapon root's local frame and is published under the same seqlock as
         * the generated body/evidence bank, so readers never combine bounds
         * from different weapon publications.
         */
        struct NativeScopeSightAnchorSnapshot
        {
            bool valid{ false };
            bool manualDirectTransitionRequired{ false };
            bool nativeScopeOverlayValid{ false };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t equippedWeaponOwnershipKey{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint32_t nativeScopeOverlayIndex{ 0 };
            RE::NiPoint3 anchorWeaponLocal{};
            RE::NiPoint3 sightBoundsMinWeaponLocal{};
            RE::NiPoint3 sightBoundsMaxWeaponLocal{};
            std::uint32_t sightBodyCount{ 0 };
        };

        static constexpr std::size_t kMaxWeaponCompositionEntries = 64;

        struct WeaponCompositionEntrySnapshot
        {
            std::uint32_t omodFormId{ 0 };
            std::uint32_t attachPointFormId{ 0 };
            std::uint32_t stableIndex{ 0 };
            std::uint32_t flags{ 0 };
            std::uint64_t semanticCoverageMask{ 0 };
        };

        struct WeaponCompositionSnapshot
        {
            std::array<WeaponCompositionEntrySnapshot,
                kMaxWeaponCompositionEntries> entries{};
            std::uint32_t entryCount{ 0 };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t compositionSignature{ 0 };
            std::uint64_t semanticCoverageMask{ 0 };
            std::uint64_t missingCoverageMask{ 0 };
            std::uint64_t publicationSequence{ 0 };
            std::uint32_t weaponFormId{ 0 };
        };

        /*
         * Frame-scoped, non-owning view of one generated weapon body's source
         * triangles. The triangle storage remains owned by WeaponCollision and
         * is immutable for the published weapon generation. Consumers must use
         * the view synchronously on the main thread and must not retain it past
         * the next WeaponCollision update/rebuild.
         *
         * Keeping the source-local representation avoids copying and
         * transforming an unbounded high-poly weapon part before a consumer can
         * apply its own bounded evidence policy.
         */
        struct SupportGripEvidenceView
        {
            std::span<const TriangleData> localTriangles{};
            RE::NiTransform localToWorld{};
            std::uint64_t weaponGenerationKey{ 0 };
            bool sourceNodeCurrent{ false };
        };

        /*
         * One-shot witness that a world-space point is within a caller-owned
         * radius of the current generated weapon surface. The query consumes
         * the cached source triangles synchronously and does not retain the
         * weapon root or any source-node pointer.
         */
        struct WeaponSurfaceProximityWitness
        {
            float distanceGameUnits{ 0.0f };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint64_t weaponGenerationKey{ 0 };
            bool sourceNodeCurrent{ false };
        };

        void init(RE::hknpWorld* world, void* bhkWorld);

        void shutdown();
        void abandonHavokStateAfterWorldLoss();

        void update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, float dt, bool weaponDrawn);

        void requestWorkbenchExitRebuild();

        bool hasWeaponBody() const;

        std::uint32_t getWeaponBodyCount() const;

        // One-frame, read-only release geometry query. The caller must consume
        // this before destroyWeaponBody retires the equipped body bank.
        ReleaseGeometrySnapshot getCurrentWeaponReleaseGeometry(
            const RE::NiPoint3& gripWorldPoint,
            const RE::NiTransform& capturedWeaponWorld) const;

        RE::hknpBodyId getWeaponBodyId() const;

        std::uint32_t getWeaponBodyIdAtomic() const;

        std::uint32_t getWeaponBodyIdAtomic(std::size_t index) const;

        WeaponBodySnapshot getWeaponBodySnapshotAtomic() const;

        bool isWeaponBodyIdAtomic(std::uint32_t bodyId) const;

        bool tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const;

        bool tryGetWeaponBodySampledVelocityAtomic(std::uint32_t bodyId, float* outVelocityHavok) const;

        bool tryGetWeaponContactDebugInfo(std::uint32_t bodyId, WeaponInteractionDebugInfo& outInfo) const;

        std::vector<WeaponCollisionProfileEvidenceDescriptor> getProfileEvidenceDescriptors() const;

        WeaponEmitterSnapshot getWeaponEmitterSnapshot() const;

        NativeScopeSightAnchorSnapshot getNativeScopeSightAnchorSnapshot() const;

        WeaponCompositionSnapshot getWeaponCompositionSnapshot() const;

        bool tryGetProfileEvidenceDescriptorForBodyId(
            std::uint32_t bodyId,
            WeaponCollisionProfileEvidenceDescriptor& outDescriptor,
            RE::NiAVObject*& outSourceNode) const;

        std::uint64_t getCurrentEquippedWeaponGenerationKey() const { return _cachedWeaponKey; }

        std::uint64_t getCurrentEquippedWeaponIdentityKey() const { return _observedEquippedWeaponIdentityKey; }

        std::uint64_t getCurrentEquippedWeaponOwnershipKey() const { return _observedEquippedWeaponOwnershipKey; }

        std::uint32_t getCurrentObservedEquippedWeaponFormID() const { return _observedEquippedWeaponFormID; }

        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity getEquippedWeaponClassification() const;

        std::uint64_t getCurrentWeaponGenerationKey() const { return _weaponBodySetKeyAtomic.load(std::memory_order_acquire); }

        bool tryFindInteractionContactNearPoint(
            const RE::NiAVObject* weaponNode,
            const RE::NiPoint3& probeWorldPoint,
            float probeRadiusGame,
            WeaponInteractionContact& outContact) const;

        bool tryFindCurrentWeaponSurfaceNearPoint(
            const RE::NiAVObject* currentWeaponRoot,
            const RE::NiPoint3& pointWorld,
            float maxDistanceGameUnits,
            WeaponSurfaceProximityWitness& outWitness) const;

        bool tryGetSupportGripEvidenceView(
            std::uint32_t bodyId,
            const RE::NiAVObject* currentWeaponRoot,
            SupportGripEvidenceView& outView) const;

        BethesdaPhysicsBody& getWeaponBody();

        void destroyWeaponBody(RE::hknpWorld* world);

        void invalidateForScaleChange(RE::hknpWorld* world);

        void updateBodiesFromCurrentSourceTransforms(
            RE::hknpWorld* world,
            RE::NiAVObject* fallbackWeaponNode,
            float sourceDeltaSeconds,
            const RE::NiAVObject* const* drivenSourceNodes = nullptr,
            std::size_t drivenSourceNodeCount = 0);

        void flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

        void serviceRetiredWeaponBodies(std::uint32_t completedPhysicsSteps = 1);

    private:
        static constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::size_t MAX_WEAPON_BODIES = MAX_WEAPON_COLLISION_BODIES;
        static constexpr std::uint32_t RETIRED_GENERATED_WEAPON_BODY_GRACE_STEPS = 8;
        static constexpr std::size_t MAX_RETIRED_GENERATED_WEAPON_BODY_PAYLOADS = MAX_WEAPON_BODIES * 4;

        struct GeneratedHullSource
        {
            std::vector<RE::NiPoint3> localPointsGame;
            std::vector<TriangleData> localTrianglesGame;
            std::vector<RE::NiPoint3> sourceLocalPointsGame;
            std::vector<TriangleData> sourceLocalTrianglesGame;
            std::vector<std::vector<RE::NiPoint3>> childLocalPointCloudsGame;
            RE::NiPoint3 localCenterGame{};
            RE::NiPoint3 sourceLocalCenterGame{};
            RE::NiPoint3 localMinGame{};
            RE::NiPoint3 localMaxGame{};
            RE::NiPoint3 sourceLocalMinGame{};
            RE::NiPoint3 sourceLocalMaxGame{};
            RE::NiAVObject* driveRoot{ nullptr };
            RE::NiAVObject* sourceRoot{ nullptr };
            std::uintptr_t sourceGroupId{ 0 };
            std::string sourceName;
            WeaponPartClassification semantic{};
            /*
             * NiTransform::scale of sourceRoot, captured at extraction time.
             * sourceLocalPointsGame/sourceLocalCenterGame are computed by
             * dividing world points by this same scale, so it must be
             * re-multiplied back in when those points are baked into a Havok
             * shape (Havok never re-applies NiNode scale to a built shape at
             * runtime) - otherwise a source node with scale != 1.0 produces a
             * collider baked at (trueSize / scale). Captured here instead of
             * re-read live at shape-build time, since body creation is staged
             * across frames after extraction.
             */
            float sourceNodeScale{ 1.0f };
        };

        struct WeaponBodyInstance
        {
            BethesdaPhysicsBody body;
            const RE::hknpShape* shape{ nullptr };
            RE::NiAVObject* driveNode{ nullptr };
            RE::NiAVObject* sourceNode{ nullptr };
            std::string sourceName;
            std::string sourceRootName;
            RE::NiPoint3 generatedLocalCenterGame{};
            RE::NiPoint3 generatedSourceLocalCenterGame{};
            RE::NiPoint3 generatedLocalMinGame{};
            RE::NiPoint3 generatedLocalMaxGame{};
            RE::NiPoint3 generatedSourceLocalMinGame{};
            RE::NiPoint3 generatedSourceLocalMaxGame{};
            std::vector<RE::NiPoint3> generatedLocalPointsGame{};
            std::vector<TriangleData> generatedLocalTrianglesGame{};
            std::vector<RE::NiPoint3> generatedSourceLocalPointsGame{};
            std::vector<TriangleData> generatedSourceLocalTrianglesGame{};
            std::uint32_t generatedPointCount{ 0 };
            WeaponPartClassification semantic{};
            bool ownsShapeRef{ false };
            GeneratedKeyframedBodyDriveState driveState{};
            std::uint32_t publicationIndex{ INVALID_BODY_ID };
        };

        struct RetiredWeaponBodyPayload
        {
            RetiredBethesdaPhysicsBodyPayload bodyPayload{};
            std::uint32_t remainingPhysicsSteps{ 0 };

            [[nodiscard]] bool occupied() const { return bodyPayload.occupied(); }
        };

        using WeaponBodyBank = std::array<WeaponBodyInstance, MAX_WEAPON_BODIES>;

        /*
         * Generated weapon body creation owns native body existence and the
         * initial collision-filter state only. Publication is intentionally a
         * separate lifecycle step so active external metadata cannot be emitted
         * for no-collide cached bodies.
         */
        struct GeneratedWeaponBodyCreateOptions
        {
            bool collisionEnabledOnCreate{ false };
        };

        struct GeneratedSourceCache
        {
            bool valid{ false };
            std::uint64_t equippedKey{ 0 };
            std::uint64_t visualKey{ 0 };
            float convexRadius{ -1.0f };
            float pointDedupGrid{ -1.0f };
            int supportFitTargetPoints{ -1 };
            float supportFitMaxErrorGameUnits{ -1.0f };
            std::vector<GeneratedHullSource> sources;
            weapon_generated_source_completeness_policy::GeneratedSourceCompleteness summary{};
        };

        struct PendingGeneratedWeaponBuild
        {
            bool active{ false };
            bool replacingExisting{ false };
            bool settingsChanged{ false };
            bool driveRequestedRebuild{ false };
            std::uint64_t equippedKey{ 0 };
            std::uint64_t visualKey{ 0 };
            std::uint64_t identityKey{ 0 };
            std::uint64_t ownershipKey{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint32_t visualRootCount{ 0 };
            std::uint32_t visibleTriShapeCount{ 0 };
            float convexRadius{ -1.0f };
            float pointDedupGrid{ -1.0f };
            int supportFitTargetPoints{ -1 };
            float supportFitMaxErrorGameUnits{ -1.0f };
            std::size_t nextSourceIndex{ 0 };
            std::size_t createdCount{ 0 };
            std::vector<GeneratedHullSource> sources;
            weapon_generated_source_completeness_policy::GeneratedSourceCompleteness summary{};
        };

        struct OmodCoverageAuditResult
        {
            bool ran{ false };
            bool sceneEnriched{ false };
        };

        WeaponBodyBank& activeWeaponBodies();
        const WeaponBodyBank& activeWeaponBodies() const;
        WeaponBodyBank& inactiveWeaponBodies();
        static bool bankHasWeaponBody(const WeaponBodyBank& bank);
        static std::uint32_t bankWeaponBodyCount(const WeaponBodyBank& bank);
        static RE::NiAVObject* resolvePackageDriveNode(const WeaponBodyBank& bank, RE::NiAVObject* fallbackWeaponNode);
        static weapon_generated_source_completeness_policy::GeneratedSourceCompleteness summarizeGeneratedSources(const std::vector<GeneratedHullSource>& sources);
        std::size_t createGeneratedWeaponBodiesInBank(
            RE::hknpWorld* world,
            const std::vector<GeneratedHullSource>& sources,
            WeaponBodyBank& bank,
            const GeneratedWeaponBodyCreateOptions& options);
        std::size_t createGeneratedWeaponBodiesInBankSlice(
            RE::hknpWorld* world,
            const std::vector<GeneratedHullSource>& sources,
            WeaponBodyBank& bank,
            const GeneratedWeaponBodyCreateOptions& options,
            std::size_t& nextSourceIndex,
            std::size_t maxSourceAttemptsThisFrame);
        void destroyWeaponBodyBank(WeaponBodyBank& bank, bool releaseShapeRef);
        void retireWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef);
        void retireWeaponBodyPayload(RetiredBethesdaPhysicsBodyPayload& payload);
        void setWeaponBodyBankCollisionEnabled(RE::hknpWorld* world, WeaponBodyBank& bank, bool enabled);
        void clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef);
        void clearAtomicBodyIds();
        void resetWeaponBodySetGeneration();
        void publishWeaponBodySetGeneration(const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& sourceCompleteness);
        void publishAtomicBodyIds(WeaponBodyBank& bank);
        void unpublishAtomicBodyIds();
        void beginWeaponBodyPublication();
        void endWeaponBodyPublication();
        std::vector<WeaponCollisionProfileEvidenceDescriptor> buildProfileEvidenceSnapshot(
            const WeaponBodyBank& bank,
            WeaponCompositionSnapshot& outComposition) const;
        WeaponEmitterSnapshot buildWeaponEmitterSnapshot(
            RE::NiAVObject* weaponNode,
            std::uint64_t equippedWeaponKey,
            std::uint64_t weaponGenerationKey,
            std::uint64_t rootSetKey) const;
        void updateWeaponEmitterSnapshot(RE::NiAVObject* weaponNode, std::uint64_t equippedWeaponKey);
        void clearWeaponEmitterSnapshot();
        void publishSampledVelocityAtomic(std::uint32_t publicationIndex, const GeneratedKeyframedBodyDriveQueueResult& queueResult);
        void dumpEquippedWeaponOmodEvidence(const WeaponBodyBank& bank, RE::NiAVObject* packageDriveNode);
        OmodCoverageAuditResult maybeRunWeaponOmodCoverageAudit(
            RE::NiAVObject* weaponNode,
            std::uint64_t auditedEquippedKey,
            bool forceBeforeInitialBuild = false);

        std::size_t findGeneratedWeaponShapeSources(
            RE::NiAVObject* weaponNode,
            std::uint64_t equippedWeaponKey,
            std::vector<GeneratedHullSource>& outSources,
            float maxSourceDistanceGame);

        void findGeneratedWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::NiAVObject* sourceRoot, const RE::NiTransform& weaponRootTransform,
            int depth,
            std::vector<GeneratedHullSource>& outSources,
            std::uint32_t& visitedShapes,
            std::uint32_t& extractedTriangles,
            const std::unordered_set<std::uintptr_t>& claimedSourceGroups,
            std::unordered_set<std::uintptr_t>& candidateExtractedSourceGroups,
            float maxSourceDistanceGame,
            std::uint32_t& culledForDistance,
            std::uint32_t& culledForEffectGeometry);
        RE::NiTransform makeGeneratedBodyWorldTransform(const RE::NiTransform& weaponRootTransform, const RE::NiPoint3& localCenterGame) const;
        bool weaponCollisionSettingsChanged() const;
        void handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex);
        void clearGeneratedSourceCompletenessTracking();
        void clearPendingWeaponVisualRebuild();
        void clearGeneratedSourceCache();
        void resetVisualSourceUnavailableRetention();
        bool canRetainCurrentWeaponBodiesForVisualSourceMiss(std::uint64_t observedIdentityKey, RE::NiAVObject* currentWeaponRoot, int retainFrameLimit);
        bool generatedSourceCacheMatches(std::uint64_t equippedKey, std::uint64_t visualKey) const;
        void storeGeneratedSourceCache(std::uint64_t equippedKey,
            std::uint64_t visualKey,
            std::vector<GeneratedHullSource> sources,
            const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary);
        void clearPendingGeneratedWeaponBuild(RE::hknpWorld* world, bool destroyTargetBank);
        bool beginPendingGeneratedWeaponBuild(std::uint64_t equippedKey,
            std::uint64_t visualKey,
            std::uint64_t identityKey,
            std::uint64_t ownershipKey,
            std::uint32_t weaponFormID,
            const WeaponVisualKeyStats& visualKeyStats,
            bool replacingExisting,
            bool settingsChanged,
            bool driveRequestedRebuild,
            std::vector<GeneratedHullSource> sources,
            const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary);
        bool advancePendingGeneratedWeaponBuild(RE::hknpWorld* world);
        bool pendingGeneratedWeaponBuildMatches(
            std::uint64_t equippedKey,
            std::uint64_t ownershipKey,
            std::uint32_t weaponFormID) const;
        void resetWeaponCollisionSettingsCache();

        std::uint64_t getEquippedWeaponIdentityKey(
            std::uint64_t* outIdentityKey = nullptr,
            std::uint64_t* outOwnershipKey = nullptr,
            WeaponSizeClass* outSizeClass = nullptr,
            std::uint32_t* outFormID = nullptr) const;
        std::uint64_t getWeaponVisualCompositionKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const;

        void maybeDumpWeaponAnimNodeDiagnostics(RE::NiAVObject* updateWeaponNode, std::uint64_t observedKey);

        void queueBodyTarget(WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float sourceDeltaSeconds);

        WeaponBodyBank _weaponBodies{};
        WeaponBodyBank _weaponReplacementBodies{};
        std::array<RetiredWeaponBodyPayload, MAX_RETIRED_GENERATED_WEAPON_BODY_PAYLOADS> _retiredWeaponBodyPayloads{};
        std::uint32_t _retiredWeaponBodyPayloadCount{ 0 };
        mutable std::mutex _retiredWeaponBodyPayloadMutex;
        bool _usingReplacementWeaponBodies{ false };
        PhysicsCallbackQuiescenceGate* _physicsCallbackGate{ nullptr };
        std::uint64_t _cachedWeaponKey{ 0 };
        std::uint64_t _cachedWeaponVisualKey{ 0 };
        std::uint64_t _cachedWeaponIdentityKey{ 0 };
        // Body-associated ownership witnesses. Unlike the observed fields
        // below, these remain bound to the currently published body set.
        std::uint64_t _cachedWeaponOwnershipKey{ 0 };
        std::uint32_t _cachedWeaponFormID{ 0 };
        // Available before generated bodies publish; the cached identity above
        // remains body-associated for replacement safety.
        std::uint64_t _observedEquippedWeaponIdentityKey{ 0 };
        // Instance-bound authority witness; never substitute this for a
        // collision generation or content-equivalence key.
        std::uint64_t _observedEquippedWeaponOwnershipKey{ 0 };
        // Form paired with the observed identity/ownership witnesses above.
        // Consumers use it to reject the one-frame old-generation/new-form
        // overlap during direct Pip-Boy equipment changes.
        std::uint32_t _observedEquippedWeaponFormID{ 0 };
        std::uint64_t _cachedWeaponBodySetKey{ 0 };
        std::uint64_t _weaponBodySetEpoch{ 0 };
        weapon_generated_source_completeness_policy::GeneratedSourceCompleteness _cachedGeneratedSourceCompleteness{};
        GeneratedSourceCache _generatedSourceCache{};
        PendingGeneratedWeaponBuild _pendingGeneratedWeaponBuild{};
        RE::hknpWorld* _cachedWorld{ nullptr };
        void* _cachedBhkWorld{ nullptr };
        std::atomic<bool> _driveRebuildRequested{ false };
        std::atomic<bool> _workbenchExitRebuildRequested{ false };
        std::atomic<std::uint32_t> _driveFailureCount{ 0 };

        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyIdsAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyPartKindsAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyReloadRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodySupportRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodySocketRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyActionRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyGripPosesAtomic;
        std::array<std::atomic<std::uintptr_t>, MAX_WEAPON_BODIES> _weaponBodyInteractionRootsAtomic;
        std::array<std::atomic<std::uintptr_t>, MAX_WEAPON_BODIES> _weaponBodySourceRootsAtomic;
        std::array<std::atomic<std::uint64_t>, MAX_WEAPON_BODIES> _weaponBodyGenerationKeysAtomic;
        std::array<std::atomic<float>, MAX_WEAPON_BODIES> _weaponBodySampledVelocityHavokXAtomic;
        std::array<std::atomic<float>, MAX_WEAPON_BODIES> _weaponBodySampledVelocityHavokYAtomic;
        std::array<std::atomic<float>, MAX_WEAPON_BODIES> _weaponBodySampledVelocityHavokZAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodySampledVelocityValidAtomic;
        std::atomic<std::uint32_t> _weaponBodyCountAtomic{ 0 };
        std::atomic<std::uint64_t> _weaponBodySetKeyAtomic{ 0 };
        std::atomic<std::uint64_t> _weaponBodyPublicationVersion{ 0 };
        mutable std::mutex _weaponEvidenceSnapshotMutex;
        std::vector<WeaponCollisionProfileEvidenceDescriptor> _profileEvidenceSnapshot;
        WeaponEmitterSnapshot _weaponEmitterSnapshot{};
        NativeScopeSightAnchorSnapshot _nativeScopeSightAnchorSnapshot{};
        WeaponCompositionSnapshot _weaponCompositionSnapshot{};
        std::uint64_t _weaponCompositionPublicationSequence{ 0 };
        // Debug OMOD evidence dump fires once per weapon generation key.
        std::uint64_t _lastOmodDumpGenerationKey{ 0 };
        /*
         * Post-build OMOD coverage audit cadence (bDebugWeaponOmodCoverageAudit).
         * Unlike the one-shot build-time dump, the audit re-observes the live
         * scene graphs seconds after publication to catch part models that the
         * engine attaches after ROCK's build window has closed.
         */
        std::uint64_t _omodCoverageAuditBodySetKey{ 0 };
        int _omodCoverageAuditFrameCounter{ 0 };
        std::uint32_t _omodCoverageAuditRunIndex{ 0 };
        // Run the mutating pre-build audit once for an exact equipped identity
        // and assembled root; the later cadence remains a safety net.
        std::uint64_t _omodPrebuildAuditEquippedKey{ 0 };
        RE::NiAVObject* _omodPrebuildAuditRoot{ nullptr };
        /*
         * Self-heal attempts are keyed by (weapon instance node address ^
         * OMOD formID): the same assembled tree is never retried (a failed or
         * name-unmatchable heal must not stack duplicate geometry across
         * audits), while an engine reassembly produces a new instance address
         * and legitimately re-opens healing.
         */
        std::unordered_set<std::uint64_t> _omodSelfHealAttempted;
        int _posLogCounter{ 0 };

        float _cachedConvexRadius{ -1.0f };
        float _cachedPointDedupGrid{ -1.0f };
        int _cachedSupportFitTargetPoints{ -1 };
        float _cachedSupportFitMaxErrorGameUnits{ -1.0f };
        std::uint64_t _pendingWeaponVisualRebuildKey{ 0 };
        std::uint64_t _pendingWeaponVisualWitnessKey{ 0 };
        std::size_t _pendingWeaponVisualVisibleTriShapeCount{ 0 };
        int _pendingWeaponVisualStableFrames{ 0 };
        std::uint64_t _visualSourceUnavailableRetainIdentityKey{ 0 };
        std::uintptr_t _visualSourceUnavailableRetainRoot{ 0 };
        int _visualSourceUnavailableRetainFrames{ 0 };
        int _weaponAnimNodeDumpFrameCounter{ 0 };
        std::uint64_t _lastWeaponAnimNodeDumpKey{ 0 };

    };

}

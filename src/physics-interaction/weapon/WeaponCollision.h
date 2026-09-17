#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <memory>
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
#include "physics-interaction/weapon/WeaponPartRuntime.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSemantics.h"
#include "physics-interaction/weapon/WeaponTriangleIndex.h"
#include "physics-interaction/weapon/WeaponScenePath.h"

#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyCinfo.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "RE/NetImmerse/NiSmartPointer.h"

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
            bool scopeEligible{ false };
            bool manualDirectTransitionRequired{ false };
            bool nativeScopeOverlayValid{ false };
            std::uint64_t weaponGenerationKey{ 0 };
            std::uint64_t equippedWeaponOwnershipKey{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint32_t nativeScopeOverlayIndex{ 0 };
            // Opaque identity witnesses only, never dereferenced by readers.
            std::uintptr_t scopeWeaponIdentity{ 0 };
            std::uintptr_t scopeInstanceIdentity{ 0 };
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
            std::uintptr_t sourceGroupId{ 0 };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
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
            RE::NiPoint3 closestPointWorld{};
            float distanceGameUnits{ 0.0f };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint64_t weaponGenerationKey{ 0 };
            bool sourceNodeCurrent{ false };
            bool valid{ false };
        };

        struct ApproximateBoundsSnapshot
        {
            bool valid{ false };
            std::uint64_t generationKey{ 0 };
            RE::NiPoint3 minWeaponLocal{};
            RE::NiPoint3 maxWeaponLocal{};
            RE::NiPoint3 centerWeaponLocal{};
            RE::NiPoint3 halfExtentsWeaponLocal{};
            std::uint32_t sourceBodyCount{ 0 };
        };

        enum class CompoundGeometrySnapshotFailure : std::uint8_t
        {
            None,
            NoGeneration,
            NoActiveBodies,
            MissingShape,
            MissingPointCloud,
            NonFinitePoint,
            DegeneratePointCloud,
            SourceTransformUnavailable,
            BodyCountChanged,
            GenerationChanged,
            InvalidBounds,
        };

        struct CompoundGeometryChildSnapshot
        {
            const RE::hknpShape* shape{ nullptr };
            RE::NiTransform shapeInWeapon{};
        };

        struct CompoundChildPoseSnapshot
        {
            RE::NiTransform shapeInWeapon{};
        };

        /*
         * Creation-only view of the active layer-44 hull geometry in one
         * weapon-root-local basis. Dynamic weapon collision consumes every
         * borrowed child shape synchronously while its native constructor
         * acquires independent references; no source node, body, or borrowed
         * hknp shape survives the call.
         */
        struct CompoundGeometrySnapshot
        {
            bool valid{ false };
            CompoundGeometrySnapshotFailure failure{ CompoundGeometrySnapshotFailure::None };
            std::uint64_t generationKey{ 0 };
            RE::NiPoint3 minWeaponLocal{};
            RE::NiPoint3 maxWeaponLocal{};
            RE::NiPoint3 centerWeaponLocal{};
            RE::NiPoint3 halfExtentsWeaponLocal{};
            std::vector<CompoundGeometryChildSnapshot> children;
            std::size_t sourcePointCount{ 0 };
            std::uint32_t sourceBodyCount{ 0 };
            std::uint32_t failedSourceIndex{ 0xFFFF'FFFFu };
            std::uint32_t failedBodyId{ 0x7FFF'FFFFu };
        };

        void init(RE::hknpWorld* world, void* bhkWorld);

        void shutdown();
        void abandonHavokStateAfterWorldLoss();

        void update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, float dt, bool weaponDrawn);

        void requestWorkbenchExitRebuild();
        void requestRebuildForReplacedSources();

        bool hasWeaponBody() const;

        std::uint32_t getWeaponBodyCount() const;

        bool getApproximateBoundsSnapshot(ApproximateBoundsSnapshot& outSnapshot) const;

        bool getCompoundGeometrySnapshot(CompoundGeometrySnapshot& outSnapshot) const;

        bool getCompoundChildPoseSnapshot(
            const RE::NiAVObject* currentWeaponRoot,
            std::uint64_t expectedGenerationKey,
            std::span<CompoundChildPoseSnapshot> outChildren,
            std::size_t& outChildCount) const;

        // One-frame, read-only release geometry query. The caller must consume
        // this before destroyWeaponBody retires the equipped body bank.
        ReleaseGeometrySnapshot getCurrentWeaponReleaseGeometry(
            const RE::NiPoint3& gripWorldPoint,
            const RE::NiTransform& capturedWeaponWorld) const;

        RE::hknpBodyId getWeaponBodyId() const;

        std::uint32_t getWeaponBodyIdAtomic() const;

        std::uint32_t getWeaponBodyIdAtomic(std::size_t index) const;

        WeaponBodySnapshot getWeaponBodySnapshotAtomic() const;
        // Main-thread presentation query; the output contains current positions only.
        std::size_t collectAttachOnlyGripIndicators(
            const RE::NiAVObject* currentWeaponRoot,
            std::span<const weapon_part_runtime::Target> targets,
            std::span<const std::uint32_t> candidateBodyIds,
            std::span<RE::NiPoint3> outPositions) const;
        // Main-thread debug publication only. Returns the exact pending target
        // for one generated weapon body in the active bank.
        bool tryGetBodyTargetForDebug(std::uint32_t bodyId, RE::NiTransform& outTarget) const;

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

        std::uint64_t getCurrentEquippedWeaponGenerationKey() const { return _identity.cachedWeaponKey; }

        std::uint64_t getCurrentEquippedWeaponIdentityKey() const { return _identity.observedIdentityKey; }

        std::uint64_t getCurrentEquippedWeaponOwnershipKey() const { return _identity.observedOwnershipKey; }

        std::uint64_t getCurrentEquippedWeaponInstanceContentKey() const { return _identity.observedInstanceContentKey; }

        std::uint32_t getCurrentObservedEquippedWeaponFormID() const { return _identity.observedFormID; }

        weapon_generation_identity_policy::EquippedWeaponGenerationIdentity getEquippedWeaponClassification() const;

        std::uint64_t getCurrentWeaponGenerationKey() const { return _published.setKey.load(std::memory_order_acquire); }

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

        std::size_t findCurrentWeaponSurfaceNearPoints(
            const RE::NiAVObject* currentWeaponRoot,
            std::span<const RE::NiPoint3> pointsWorld,
            float maxDistanceGameUnits,
            std::span<WeaponSurfaceProximityWitness> outWitnesses) const;

        bool tryGetSupportGripEvidenceView(
            std::uint32_t bodyId,
            const RE::NiAVObject* currentWeaponRoot,
            SupportGripEvidenceView& outView) const;

        std::size_t findSupportGripEvidenceViews(
            const RE::NiAVObject* currentWeaponRoot,
            std::span<SupportGripEvidenceView> outViews) const;

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

        void serviceRetiredWeaponBodies(RE::hknpWorld* currentWorld, std::uint32_t completedPhysicsSteps = 1);

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
            // Children use the source node's local frame, like sourceLocalPointsGame.
            std::vector<std::vector<RE::NiPoint3>> childLocalPointCloudsGame;
            RE::NiPoint3 localCenterGame{};
            RE::NiPoint3 sourceLocalCenterGame{};
            RE::NiPoint3 localMinGame{};
            RE::NiPoint3 localMaxGame{};
            RE::NiPoint3 sourceLocalMinGame{};
            RE::NiPoint3 sourceLocalMaxGame{};
            RE::NiAVObject* driveRoot{ nullptr };
            RE::NiAVObject* sourceRoot{ nullptr };
            RE::NiTransform sourceInWeapon{};
            std::uintptr_t sourceGroupId{ 0 };
            std::string sourceName;
            WeaponPartClassification semantic{};
            // BGSMaterialType::materialID resolved from the authoritative
            // native collision shape and shape key. Generated simple shapes
            // and compound leaves publish it through hknpShape::userData;
            // hknpBody::materialId remains the solver-material domain.
            std::uint32_t collisionSoundMaterialId{ 0 };
            bool sourceInWeaponAvailable{ false };
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
            /*
             * The engine can replace the equipped model under the same Weapon
             * node (a Pip-Boy redraw does), which frees the nodes the raw
             * pointers above name. These references keep them alive until the
             * body is cleared; a node detached from the model has no parent,
             * so the walks that resolve it fail instead of reading freed memory.
             */
            RE::NiPointer<RE::NiAVObject> driveNodeRef;
            RE::NiPointer<RE::NiAVObject> sourceNodeRef;
            std::string sourceName;
            std::string driveRootName;
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
            WeaponTriangleIndex generatedTriangleIndex;
            WeaponTriangleIndex generatedSourceTriangleIndex;
            std::uint32_t generatedPointCount{ 0 };
            std::uintptr_t generatedSourceGroupId{ 0 };
            WeaponPartClassification semantic{};
            bool ownsShapeRef{ false };
            GeneratedKeyframedBodyDriveState driveState{};
            std::uint32_t publicationIndex{ INVALID_BODY_ID };
        };

        struct RetiredWeaponBodyPayload
        {
            RetiredBethesdaPhysicsBodyPayload bodyPayload{};
            std::uint32_t remainingPhysicsSteps{ 0 };
            bool processLifetimeHold{ false };

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
            std::uint64_t ownershipKey{ 0 };
            std::uint64_t visualKey{ 0 };
            std::uintptr_t weaponRootAddress{ 0 };
            std::vector<GeneratedHullSource> sources;
            weapon_generated_source_completeness_policy::GeneratedSourceCompleteness summary{};
        };

        struct GeneratedRecaptureDiagnosticSource
        {
            std::uintptr_t sourceGroupId{ 0 };
            std::uintptr_t sourceRootAddress{ 0 };
            std::uintptr_t driveRootAddress{ 0 };
            std::string sourceName;
            RE::NiPoint3 weaponLocalCenter{};
            RE::NiPoint3 sourceLocalCenter{};
            RE::NiPoint3 sourceLocalMin{};
            RE::NiPoint3 sourceLocalMax{};
            std::vector<TriangleData> sourceLocalTriangles;
            std::size_t sourceLocalPointCount{ 0 };
            std::size_t sourceLocalTriangleCount{ 0 };
            float sourceNodeScale{ 1.0f };
        };

        struct GeneratedRecaptureDiagnostic
        {
            bool valid{ false };
            bool sawUndrawnInterval{ false };
            std::uint64_t equippedKey{ 0 };
            std::uint64_t identityKey{ 0 };
            std::uint64_t ownershipKey{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint32_t comparisonSequence{ 0 };
            std::vector<GeneratedRecaptureDiagnosticSource> sources;
        };

        struct PendingGeneratedWeaponBuild
        {
            bool active{ false };
            bool replacingExisting{ false };
            bool driveRequestedRebuild{ false };
            std::uint64_t equippedKey{ 0 };
            std::uint64_t visualKey{ 0 };
            std::uint64_t identityKey{ 0 };
            std::uint64_t ownershipKey{ 0 };
            std::uintptr_t weaponRootAddress{ 0 };
            std::uint32_t weaponFormID{ 0 };
            std::uint32_t visualRootCount{ 0 };
            std::uint32_t visibleTriShapeCount{ 0 };
            std::size_t nextSourceIndex{ 0 };
            std::size_t createdCount{ 0 };
            std::vector<GeneratedHullSource> sources;
            weapon_generated_source_completeness_policy::GeneratedSourceCompleteness summary{};
        };

        struct PendingSourcePreparation
        {
            RE::NiPointer<RE::NiAVObject> root;
            std::uint64_t equippedKey{}, ownershipKey{}, visualKey{};
            bool ready{ false };
            std::size_t frames{};
            double activeMilliseconds{}, maxSliceMilliseconds{};
            std::vector<GeneratedHullSource> sources;
            // Last member: destroy suspended work before its referenced outputs.
            weapon_geometry_work::Task task;
        };

        WeaponBodyBank& activeWeaponBodies();
        const WeaponBodyBank& activeWeaponBodies() const;
        WeaponBodyBank& inactiveWeaponBodies();
        static bool bankHasWeaponBody(const WeaponBodyBank& bank);
        static std::uint32_t bankWeaponBodyCount(const WeaponBodyBank& bank);
        static RE::NiAVObject* resolvePackageDriveNode(const WeaponBodyBank& bank, RE::NiAVObject* fallbackWeaponNode);
        bool activeWeaponBodyRootMatches(const RE::NiAVObject* currentWeaponRoot) const;
        bool retireActiveWeaponBodiesForSceneTransition(RE::hknpWorld* world, const char* reason);
        bool tryBuildSupportGripEvidenceView(
            const WeaponBodyInstance& instance,
            const RE::NiAVObject* currentWeaponRoot,
            SupportGripEvidenceView& outView) const;
        static bool resolveCompoundChildPose(
            const WeaponBodyInstance& instance,
            const RE::NiAVObject* packageDriveNode,
            CompoundChildPoseSnapshot& outPose);
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

        std::size_t findGeneratedWeaponShapeSources(
            RE::NiAVObject* weaponNode,
            std::uint64_t equippedWeaponKey,
            std::vector<GeneratedHullSource>& outSources);

        weapon_geometry_work::Task prepareGeneratedWeaponShapeSources(RE::NiPointer<RE::NiAVObject> root,
            std::uint64_t equippedWeaponKey, std::vector<GeneratedHullSource>& outSources, bool preserveGaps);
        weapon_geometry_work::Task findGeneratedWeaponShapeSourcesRecursive(RE::NiPointer<RE::NiAVObject> nodeOwner,
            RE::NiPointer<RE::NiAVObject> sourceOwner, RE::NiTransform weaponRootTransform,
            int depth,
            std::vector<GeneratedHullSource>& outSources,
            std::uint32_t& visitedShapes,
            std::uint32_t& extractedTriangles,
            const std::unordered_set<std::uintptr_t>& claimedSourceGroups,
            std::unordered_set<std::uintptr_t>& candidateExtractedSourceGroups,
            std::uint32_t& culledForEffectGeometry, bool preserveGaps);
        bool advanceSourcePreparation();
        RE::NiTransform makeGeneratedBodyWorldTransform(const RE::NiTransform& weaponRootTransform, const RE::NiPoint3& localCenterGame) const;
        void handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex);
        void clearGeneratedSourceCompletenessTracking();
        void clearPendingWeaponVisualRebuild();
        void clearGeneratedSourceCache();
        void recordGeneratedRecaptureDiagnostic(
            std::uint64_t equippedKey,
            std::uint64_t identityKey,
            std::uint64_t ownershipKey,
            std::uint32_t weaponFormID,
            const std::vector<GeneratedHullSource>& sources);
        void resetVisualSourceUnavailableRetention();
        bool canRetainCurrentWeaponBodiesForVisualSourceMiss(std::uint64_t observedIdentityKey, RE::NiAVObject* currentWeaponRoot, float retainSecondsLimit, float measuredDeltaSeconds);
        bool generatedSourceCacheMatches(
            std::uint64_t equippedKey,
            std::uint64_t ownershipKey,
            std::uint64_t visualKey,
            const RE::NiAVObject* weaponRoot) const;
        void storeGeneratedSourceCache(std::uint64_t equippedKey,
            std::uint64_t ownershipKey,
            std::uint64_t visualKey,
            const RE::NiAVObject* weaponRoot,
            std::vector<GeneratedHullSource> sources,
            const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary);
        void clearPendingGeneratedWeaponBuild(RE::hknpWorld* world, bool destroyTargetBank);
        bool beginPendingGeneratedWeaponBuild(std::uint64_t equippedKey,
            std::uint64_t visualKey,
            std::uint64_t identityKey,
            std::uint64_t ownershipKey,
            const RE::NiAVObject* weaponRoot,
            std::uint32_t weaponFormID,
            const WeaponVisualKeyStats& visualKeyStats,
            bool replacingExisting,
            bool driveRequestedRebuild,
            std::vector<GeneratedHullSource> sources,
            const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& summary);
        bool advancePendingGeneratedWeaponBuild(RE::hknpWorld* world);
        bool pendingGeneratedWeaponBuildMatches(
            std::uint64_t equippedKey,
            std::uint64_t ownershipKey,
            const RE::NiAVObject* weaponRoot,
            std::uint32_t weaponFormID) const;

        std::uint64_t getEquippedWeaponIdentityKey(
            std::uint64_t* outIdentityKey = nullptr,
            std::uint64_t* outOwnershipKey = nullptr,
            WeaponSizeClass* outSizeClass = nullptr,
            std::uint32_t* outFormID = nullptr,
            std::uint64_t* outInstanceContentKey = nullptr) const;
        std::uint64_t getWeaponVisualCompositionKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const;

        void maybeDumpWeaponAnimNodeDiagnostics(RE::NiAVObject* updateWeaponNode, std::uint64_t observedKey);

        void queueBodyTarget(WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float sourceDeltaSeconds);

        /*
         * ---- Partitioned member state ----
         * Each collision/ module owns one state struct below;
         * WeaponIdentityState is the shared identity core. The Havok world
         * binding and the physics-callback gate remain direct members.
         */

        /*
         * Shared identity core: which equipped weapon the published body set
         * belongs to, and which equipped weapon is currently observed on the
         * player. The cached fields are body-associated ownership witnesses -
         * unlike the observed fields, they remain bound to the currently
         * published body set (replacement safety).
         */
        struct WeaponIdentityState
        {
            std::uint64_t cachedWeaponKey{ 0 };
            std::uint64_t cachedVisualKey{ 0 };
            std::uint64_t cachedIdentityKey{ 0 };
            std::uint64_t cachedOwnershipKey{ 0 };
            std::uint32_t cachedFormID{ 0 };
            std::uint64_t cachedBodySetKey{ 0 };
            std::uint64_t bodySetEpoch{ 0 };
            // Available before generated bodies publish; the cached identity
            // above remains body-associated for replacement safety.
            std::uint64_t observedIdentityKey{ 0 };
            // Instance-bound authority witness; never substitute this for a
            // collision generation or content-equivalence key.
            std::uint64_t observedOwnershipKey{ 0 };
            // Form paired with the observed identity/ownership witnesses
            // above. Consumers use it to reject the one-frame
            // old-generation/new-form overlap during direct Pip-Boy
            // equipment changes.
            std::uint32_t observedFormID{ 0 };
            // Deterministic equipped-object-instance content witness. This
            // excludes transient engine pointer identity and is safe to
            // combine with a stable form identity for persisted
            // authored-pose lookups.
            std::uint64_t observedInstanceContentKey{ 0 };
            // Game-thread, one runtime frame. Cheap pointer witnesses also
            // invalidate an equip switch that occurs within that frame.
            mutable weapon_generation_identity_policy::EquippedWeaponGenerationIdentity frameClassification{};
            mutable std::uint64_t classificationFrame{ 0 };
            mutable bool classificationValid{ false };
        };

        // State owned by the WeaponCollisionBodies module: the live and
        // replacement body banks and the retired-payload holding area.
        struct WeaponBodyBankState
        {
            WeaponBodyBank bank{};
            WeaponBodyBank replacementBank{};
            bool usingReplacementBank{ false };
            std::array<RetiredWeaponBodyPayload, MAX_RETIRED_GENERATED_WEAPON_BODY_PAYLOADS> retiredPayloads{};
            std::uint32_t retiredPayloadCount{ 0 };
            mutable std::mutex retiredPayloadMutex;
        };

        /*
         * Lock-free body-set publication written by the bodies module and
         * read by the queries module (including off-thread readers). Every
         * field is atomic; `version` is the odd/even publication guard.
         * The arrays are deliberately left without initializers, exactly as
         * before the partition: `count` and `version` gate every read.
         */
        struct AtomicBodyPublicationState
        {
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> ids;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> partKinds;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> reloadRoles;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> supportRoles;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> socketRoles;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> actionRoles;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> gripPoses;
            std::array<std::atomic<std::uintptr_t>, MAX_WEAPON_BODIES> interactionRoots;
            std::array<std::atomic<std::uintptr_t>, MAX_WEAPON_BODIES> sourceRoots;
            std::array<std::atomic<std::uint64_t>, MAX_WEAPON_BODIES> generationKeys;
            std::array<std::atomic<float>, MAX_WEAPON_BODIES> sampledVelocityHavokX;
            std::array<std::atomic<float>, MAX_WEAPON_BODIES> sampledVelocityHavokY;
            std::array<std::atomic<float>, MAX_WEAPON_BODIES> sampledVelocityHavokZ;
            std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> sampledVelocityValid;
            std::atomic<std::uint32_t> count{ 0 };
            std::atomic<std::uint64_t> setKey{ 0 };
            std::atomic<std::uint64_t> version{ 0 };
        };

        // Read-side evidence snapshots published by the bodies module and
        // served by the queries module under `mutex`.
        struct EvidenceSnapshotState
        {
            mutable std::mutex mutex;
            std::vector<WeaponCollisionProfileEvidenceDescriptor> profileDescriptors;
            WeaponEmitterSnapshot emitters{};
            NativeScopeSightAnchorSnapshot sightAnchor{};
            WeaponCompositionSnapshot composition{};
            std::uint64_t compositionPublicationSequence{ 0 };
        };

        // State owned by the GeneratedWeaponSources module: the visual
        // source cache, the incremental pending build, visual-stability
        // gating, retention across visual-source misses, and detached-source
        // exclusion.
        struct GeneratedSourceState
        {
            bool preserveGaps{ false };
            // Committed mode remains distinct while a reload/hidden visual
            // defers replacement, so the requested rebuild cannot be lost.
            bool activePreserveGaps{ false };
            std::unique_ptr<PendingSourcePreparation> preparation;
            GeneratedSourceCache cache{};
            weapon_generated_source_completeness_policy::GeneratedSourceCompleteness cachedCompleteness{};
            PendingGeneratedWeaponBuild pendingBuild{};
            std::uint64_t pendingVisualRebuildKey{ 0 };
            std::uint64_t pendingVisualWitnessKey{ 0 };
            std::size_t pendingVisualVisibleTriShapeCount{ 0 };
            // Elapsed time the visual witness has stayed identical (seconds).
            float pendingVisualStableSeconds{ 0.0f };
            std::uint64_t visualUnavailableRetainIdentityKey{ 0 };
            std::uintptr_t visualUnavailableRetainRoot{ 0 };
            // Elapsed time the current bodies were retained across a visual
            // source miss (seconds).
            float visualUnavailableRetainSeconds{ 0.0f };
            std::uint64_t detachedExclusionEquippedKey{ 0 };
            std::unordered_set<std::uintptr_t> detachedExclusionGroups;
        };

        // Cross-thread rebuild requests and drive-failure accounting
        // consumed by the update module.
        struct DriveControlState
        {
            std::atomic<bool> rebuildRequested{ false };
            std::atomic<bool> workbenchExitRebuildRequested{ false };
            std::atomic<std::uint32_t> failureCount{ 0 };
        };

        // State owned by the collision diagnostics TUs
        // (WeaponCollisionDiagnostics and WeaponCollisionOmodDiagnostics).
        struct CollisionDiagnosticsState
        {
            GeneratedRecaptureDiagnostic generatedRecapture{};
            int animNodeDumpFrameCounter{ 0 };
            std::uint64_t lastAnimNodeDumpKey{ 0 };
            // Debug OMOD evidence dump fires once per weapon generation key.
            std::uint64_t lastOmodDumpGenerationKey{ 0 };
        };

        WeaponIdentityState _identity{};
        WeaponBodyBankState _bodies{};
        // Default-initialized on purpose; see the struct comment.
        AtomicBodyPublicationState _published;
        EvidenceSnapshotState _evidence;
        struct EmitterPath
        {
            weapon_scene::Path<RE::NiAVObject, 16> transform;
            weapon_scene::Path<RE::NiAVObject, 16> effect;
            std::size_t transformRoot = 0;
            std::size_t effectRoot = 0;
        };
        // Game-thread lookup cache; only the value snapshot crosses threads.
        std::array<EmitterPath, MAX_WEAPON_EMITTERS> _emitterPaths{};
        GeneratedSourceState _sources{};
        DriveControlState _drive{};
        CollisionDiagnosticsState _diagnostics{};

        // ---- Shared direct members ----
        PhysicsCallbackQuiescenceGate* _physicsCallbackGate{ nullptr };
        RE::hknpWorld* _cachedWorld{ nullptr };
        void* _cachedBhkWorld{ nullptr };

    };

}

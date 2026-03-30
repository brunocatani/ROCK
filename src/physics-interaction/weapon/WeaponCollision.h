#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_set>
#include <vector>

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokPhysicsTiming.h"
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

        struct WeaponBodySnapshot
        {
            std::uint64_t generationKey{ 0 };
            std::uint32_t count{ 0 };
            std::array<std::uint32_t, MAX_WEAPON_COLLISION_BODIES> bodyIds{};
        };

        void init(RE::hknpWorld* world, void* bhkWorld);

        void shutdown();

        void update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, RE::hknpBodyId dominantHandBodyId, float dt);

        bool hasWeaponBody() const;

        std::uint32_t getWeaponBodyCount() const;

        RE::hknpBodyId getWeaponBodyId() const;

        std::uint32_t getWeaponBodyIdAtomic() const;

        std::uint32_t getWeaponBodyIdAtomic(std::size_t index) const;

        WeaponBodySnapshot getWeaponBodySnapshotAtomic() const;

        bool isWeaponBodyIdAtomic(std::uint32_t bodyId) const;

        bool tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const;

        bool tryGetWeaponBodySampledVelocityAtomic(std::uint32_t bodyId, float* outVelocityHavok) const;

        bool tryGetWeaponContactDebugInfo(std::uint32_t bodyId, WeaponInteractionDebugInfo& outInfo) const;

        std::vector<WeaponCollisionProfileEvidenceDescriptor> getProfileEvidenceDescriptors() const;

        bool tryGetProfileEvidenceDescriptorForBodyId(
            std::uint32_t bodyId,
            WeaponCollisionProfileEvidenceDescriptor& outDescriptor,
            RE::NiAVObject*& outSourceNode) const;

        std::uint64_t getCurrentEquippedWeaponGenerationKey() const { return _cachedWeaponKey; }

        std::uint64_t getCurrentWeaponGenerationKey() const { return _weaponBodySetKeyAtomic.load(std::memory_order_acquire); }

        bool tryFindInteractionContactNearPoint(
            const RE::NiAVObject* weaponNode,
            const RE::NiPoint3& probeWorldPoint,
            float probeRadiusGame,
            WeaponInteractionContact& outContact) const;

        bool tryFindSoftContactForCapsule(
            const RE::NiAVObject* weaponNode,
            const RE::NiPoint3& capsuleStartWorld,
            const RE::NiPoint3& capsuleEndWorld,
            float capsuleRadiusGame,
            float radiusPaddingGame,
            const RE::NiPoint3& fallbackNormalWorld,
            WeaponSoftContactResult& outContact) const;

        BethesdaPhysicsBody& getWeaponBody();

        void destroyWeaponBody(RE::hknpWorld* world);

        void invalidateForScaleChange(RE::hknpWorld* world);

        void updateBodiesFromCurrentSourceTransforms(RE::hknpWorld* world, RE::NiAVObject* fallbackWeaponNode, float sourceDeltaSeconds);

        void flushPendingPhysicsDrive(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing);

    private:
        static constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::size_t MAX_WEAPON_BODIES = MAX_WEAPON_COLLISION_BODIES;

        struct GeneratedHullSource
        {
            std::vector<RE::NiPoint3> localPointsGame;
            std::vector<std::vector<RE::NiPoint3>> childLocalPointCloudsGame;
            RE::NiPoint3 localCenterGame{};
            RE::NiPoint3 localMinGame{};
            RE::NiPoint3 localMaxGame{};
            RE::NiAVObject* driveRoot{ nullptr };
            RE::NiAVObject* sourceRoot{ nullptr };
            std::uintptr_t sourceGroupId{ 0 };
            std::string sourceName;
            WeaponPartClassification semantic{};
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
            RE::NiPoint3 generatedLocalMinGame{};
            RE::NiPoint3 generatedLocalMaxGame{};
            std::vector<RE::NiPoint3> generatedLocalPointsGame{};
            std::uint32_t generatedPointCount{ 0 };
            WeaponPartClassification semantic{};
            bool ownsShapeRef{ false };
            GeneratedKeyframedBodyDriveState driveState{};
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
        void destroyWeaponBodyBank(WeaponBodyBank& bank, bool releaseShapeRef);
        void setWeaponBodyBankCollisionEnabled(RE::hknpWorld* world, WeaponBodyBank& bank, bool enabled);
        void clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef);
        void clearAtomicBodyIds();
        void resetWeaponBodySetGeneration();
        void publishWeaponBodySetGeneration(const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& sourceCompleteness);
        void publishAtomicBodyIds(const WeaponBodyBank& bank);
        void unpublishAtomicBodyIds();
        void beginWeaponBodyPublication();
        void endWeaponBodyPublication();
        std::vector<WeaponCollisionProfileEvidenceDescriptor> buildProfileEvidenceSnapshot(const WeaponBodyBank& bank) const;
        void disableWeaponBodiesForMissingVisual(RE::hknpWorld* world);
        bool shouldReenableCachedBodiesForReturnedVisual(std::uint64_t currentKey, bool visualSettled) const;
        void publishSampledVelocityAtomic(std::uint32_t bodyId, const GeneratedKeyframedBodyDriveState& driveState);
        void requestGeneratedSourceCompletenessProbeNextFrame();
        void requestPendingGeneratedRebuildAttemptNextFrame();

        std::size_t findGeneratedWeaponShapeSources(RE::NiAVObject* weaponNode, std::vector<GeneratedHullSource>& outSources);

        void findGeneratedWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::NiAVObject* sourceRoot, const RE::NiTransform& weaponRootTransform,
            int depth,
            std::vector<GeneratedHullSource>& outSources,
            std::uint32_t& visitedShapes,
            std::uint32_t& extractedTriangles,
            const std::unordered_set<std::uintptr_t>& claimedSourceGroups,
            std::unordered_set<std::uintptr_t>& candidateExtractedSourceGroups);
        RE::NiTransform makeGeneratedBodyWorldTransform(const RE::NiTransform& weaponRootTransform, const RE::NiPoint3& localCenterGame) const;
        bool weaponCollisionSettingsChanged() const;
        void handleGeneratedBodyDriveResult(const GeneratedKeyframedBodyDriveResult& result, const char* ownerName, std::uint32_t bodyIndex);
        void resetGeneratedSourceCompletenessTracking();
        void resetWeaponCollisionSettingsCache();
        void armGeneratedSourceCompletenessProbe();
        void resetEquippedWeaponWitnessTracking();
        void resetStaleWeaponVisualTracking();
        void tryEnrichGeneratedWeaponBodiesFromLateSources(RE::hknpWorld* world, RE::NiAVObject* weaponNode);

        static weapon_generation_identity_policy::GeneratedWeaponVisualWitness makeGeneratedWeaponVisualWitness(
            std::uint64_t visualKey,
            const WeaponVisualKeyStats& stats,
            const weapon_generated_source_completeness_policy::GeneratedSourceCompleteness& generatedSummary,
            bool hasBuildableSource);

        std::uint64_t getEquippedWeaponKey(
            RE::NiAVObject* weaponNode,
            WeaponVisualKeyStats& stats,
            std::uint64_t* outOwnerKey = nullptr,
            weapon_generation_identity_policy::EquippedWeaponInstanceWitness* outInstanceWitness = nullptr,
            std::uint64_t* outVisualKey = nullptr) const;

        void queueBodyTarget(WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float sourceDeltaSeconds);

        WeaponBodyBank _weaponBodies{};
        WeaponBodyBank _weaponReplacementBodies{};
        bool _usingReplacementWeaponBodies{ false };
        std::uint64_t _cachedWeaponKey{ 0 };
        std::uint64_t _pendingWeaponKey{ 0 };
        std::uint64_t _cachedEquippedWeaponOwnerKey{ 0 };
        std::uint64_t _pendingEquippedWeaponOwnerKey{ 0 };
        weapon_generation_identity_policy::EquippedWeaponInstanceWitness _cachedEquippedWeaponInstanceWitness{};
        weapon_generation_identity_policy::EquippedWeaponInstanceWitness _pendingEquippedWeaponInstanceWitness{};
        weapon_generation_identity_policy::GeneratedWeaponVisualWitness _cachedGeneratedWeaponVisualWitness{};
        weapon_generation_identity_policy::GeneratedWeaponVisualWitness _pendingGeneratedWeaponVisualWitness{};
        std::uint64_t _cachedWeaponBodySetKey{ 0 };
        std::uint64_t _weaponBodySetEpoch{ 0 };
        weapon_visual_composition_policy::VisualSettleState _visualSettleState{};
        weapon_generated_source_completeness_policy::GeneratedSourceSettleState _generatedSourceSettleState{};
        weapon_generated_source_completeness_policy::GeneratedSourceCompleteness _cachedGeneratedSourceCompleteness{};
        RE::hknpWorld* _cachedWorld{ nullptr };
        void* _cachedBhkWorld{ nullptr };
        bool _weaponBodyPending{ false };
        float _equippedVisualMissingSeconds{ 0.0f };
        bool _weaponBodiesDisabledForMissingVisual{ false };
        std::atomic<bool> _driveRebuildRequested{ false };
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
        mutable std::mutex _profileEvidenceSnapshotMutex;
        std::vector<WeaponCollisionProfileEvidenceDescriptor> _profileEvidenceSnapshot;

        int _retryCounter{ 0 };
        bool _pendingGeneratedRebuildAttemptRequested{ false };
        std::uint32_t _generatedSourceCompletenessProbeFramesRemaining{ 0 };
        std::uint32_t _generatedSourceCompletenessProbeCounter{ 0 };
        bool _generatedSourceCompletenessProbeRequested{ false };
        std::uint32_t _staleVisualPendingObservationCount{ 0 };
        std::uint64_t _lastNativeVisualRemapRequestedInstanceSignature{ 0 };
        weapon_native_visual_remap_policy::NativeVisualRemapAttemptState _nativeVisualRemapAttemptState{};

        int _posLogCounter{ 0 };

        float _cachedConvexRadius{ -1.0f };
        float _cachedPointDedupGrid{ -1.0f };
        int _cachedSupportFitTargetPoints{ -1 };
        float _cachedSupportFitMaxErrorGameUnits{ -1.0f };

        bool _dominantHandDisabled{ false };
        RE::hknpBodyId _disabledHandBodyId{ INVALID_BODY_ID };

        void disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId);

        void enableDominantHandCollision(RE::hknpWorld* world);
    };

}

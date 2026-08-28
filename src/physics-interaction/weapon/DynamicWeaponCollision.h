#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"
#include "physics-interaction/native/HavokCompoundShapeBuilder.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/weapon/WeaponCollision.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace RE
{
    class NiNode;
    class hknpShape;
    class hknpWorld;
}

namespace rock::havok_runtime
{
    struct ContactSignalPointResult;
}

namespace rock
{
    class PhysicsCallbackQuiescenceGate;
    struct PhysicsFrameContext;

    class DynamicWeaponCollisionRuntime
    {
    public:
        struct FrameResult
        {
            bool proxyActive{ false };
            bool applyVisualCorrection{ false };
            bool contactEpisodeStarted{ false };
            bool rawContactPointValid{ false };
            bool rawContactProxyWasBodyA{ false };
            bool otherBodyWorldValid{ false };
            std::uint32_t otherBodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherLayer{ 0 };
            std::uint32_t otherMotionIndex{ 0x7FFF'FFFFu };
            std::uint32_t rawContactPointCount{ 0 };
            std::uint32_t rawContactPointIndex{ 0 };
            std::uint64_t contactEpisode{ 0 };
            std::uint64_t contactSolveAge{ 0 };
            std::uintptr_t otherCollisionObject{ 0 };
            std::uintptr_t otherOwnerNode{ 0 };
            float rawContactPointWeightSum{ 0.0f };
            RE::NiPoint3 rawContactPointGame{};
            RE::NiPoint3 rawContactNormalHavok{};
            RE::NiTransform requestedWeaponWorld{};
            RE::NiTransform resolvedWeaponWorld{};
            RE::NiTransform requestedContactBodyWorld{};
            RE::NiTransform liveContactBodyWorld{};
            RE::NiTransform otherBodyWorld{};
            float translationCorrectionGameUnits{ 0.0f };
            float rotationCorrectionDegrees{ 0.0f };
        };

        struct DebugSnapshot
        {
            bool valid{ false };
            bool physicsSnapshotReadable{ false };
            bool physicsSnapshotValid{ false };
            bool physicsSnapshotIdentityCurrent{ false };
            bool physicsSnapshotContactActive{ false };
            bool physicsSnapshotTeleported{ false };
            bool contactActive{ false };
            bool visualCorrectionActive{ false };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint32_t authorityBodyId{ 0x7FFF'FFFFu };
            std::uint32_t constraintId{ 0x7FFF'FFFFu };
            std::uint32_t otherBodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherLayer{ 0 };
            std::uint32_t contactGraceSolves{ 0 };
            std::uint64_t generationKey{ 0 };
            std::uint64_t solveSequence{ 0 };
            std::uint64_t proxyPairCallbackSequence{ 0 };
            std::uint64_t obstacleCallbackSequence{ 0 };
            std::uint64_t rawPointCallbackSequence{ 0 };
            std::uint64_t processedManifoldCallbackSequence{ 0 };
            std::uint64_t admittedContactSequence{ 0 };
            std::uint32_t compoundChildCount{ 0 };
            std::size_t compoundPointCount{ 0 };
            RE::NiPoint3 centerWeaponLocal{};
            RE::NiPoint3 halfExtentsWeaponLocal{};
            RE::NiTransform requestedWeaponWorld{};
            RE::NiTransform liveWeaponWorld{};
            RE::NiTransform resolvedWeaponWorld{};
            float translationCorrectionGameUnits{ 0.0f };
            float rotationCorrectionDegrees{ 0.0f };
        };

        void setPhysicsCallbackGate(PhysicsCallbackQuiescenceGate* gate) { _physicsCallbackGate = gate; }

        void beginFrame(
            std::uint64_t frameIndex,
            RE::hknpWorld* world,
            void* bhkWorld,
            RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            bool enabled,
            bool suppressDefaultNativeIntent);

        static void observeWeaponVisualIntent(
            void* context,
            RE::NiNode* weaponNode,
            const RE::NiTransform& requestedWeaponWorld,
            std::uint64_t weaponGenerationKey);

        FrameResult finishFrame(
            const PhysicsFrameContext& frame,
            bool physicsWritesAllowed,
            RE::NiNode* weaponNode,
            std::uint64_t weaponGenerationKey,
            const WeaponCollision& weaponCollision);

        void flushPendingPhysicsDrive(
            RE::hknpWorld* world,
            const havok_physics_timing::PhysicsTimingSample& timing);
        void samplePostSolve(RE::hknpWorld* world, std::uint64_t solveSequence);

        bool isProxyBodyIdAtomic(std::uint32_t bodyId) const;
        struct ContactChildSource
        {
            bool valid{ false };
            std::uint32_t proxyBodyId{ 0x7FFF'FFFFu };
            std::uint32_t sourceBodyId{ 0x7FFF'FFFFu };
            std::uint32_t childIndex{ 0xFFFF'FFFFu };
            std::uint64_t weaponGenerationKey{ 0 };
        };
        [[nodiscard]] bool tryClassifyContactChildAtomic(
            std::uint32_t bodyId,
            std::uint32_t shapeKey,
            ContactChildSource& outSource) const noexcept;
        [[nodiscard]] bool tryConvertContactPointToWeaponLocal(
            const RE::NiPoint3& contactPointGame,
            std::uint64_t expectedGenerationKey,
            RE::NiPoint3& outPointWeaponLocal) const;
        void recordObstacleContactCallback(
            RE::hknpWorld* world,
            std::uint32_t proxyBodyId,
            std::uint32_t otherBodyId,
            bool otherLayerRead,
            std::uint32_t otherLayer,
            bool proxyWasBodyA,
            const havok_runtime::ContactSignalPointResult* rawContactPoint);
        void recordObstacleManifoldProcessedCallback(
            RE::hknpWorld* world,
            std::uint32_t proxyBodyId,
            std::uint32_t otherBodyId,
            bool otherLayerRead,
            std::uint32_t otherLayer);

        void retireAll(void* bhkWorld);
        void abandonHavokStateAfterWorldLoss();

        [[nodiscard]] RE::hknpBodyId proxyBodyIdForDebug() const;
        // Main-thread debug publication only. Returns the colliding contact
        // body's target reconstructed from the pending grip-authority target.
        [[nodiscard]] bool tryGetContactBodyTargetForDebug(RE::NiTransform& outTarget) const;
        bool getDebugSnapshot(DebugSnapshot& outSnapshot) const;

    private:
        struct AtomicTransform
        {
            std::array<std::atomic<float>, 9> rotation{};
            std::array<std::atomic<float>, 3> translation{};
            std::atomic<float> scale{ 1.0f };
        };

        struct PhysicsSnapshot
        {
            bool valid{ false };
            bool contactActive{ false };
            bool teleported{ false };
            std::uintptr_t world{ 0 };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherBodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherLayer{ 0 };
            std::uint32_t contactGraceSolves{ 0 };
            std::uint64_t generationKey{ 0 };
            std::uint64_t solveSequence{ 0 };
            float weaponScale{ 1.0f };
            RE::NiTransform requestedProxyBodyWorld{};
            RE::NiTransform liveProxyBodyWorld{};
        };

        struct ContactDiagnosticSnapshot
        {
            bool valid{ false };
            bool rawContactPointValid{ false };
            bool rawContactProxyWasBodyA{ false };
            bool otherBodyWorldValid{ false };
            std::uintptr_t world{ 0 };
            std::uint32_t bodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherBodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherLayer{ 0 };
            std::uint32_t otherMotionIndex{ 0x7FFF'FFFFu };
            std::uint32_t rawContactPointCount{ 0 };
            std::uint32_t rawContactPointIndex{ 0 };
            std::uint64_t generationKey{ 0 };
            std::uint64_t contactEpisode{ 0 };
            std::uint64_t contactSolveAge{ 0 };
            std::uintptr_t otherCollisionObject{ 0 };
            std::uintptr_t otherOwnerNode{ 0 };
            float rawContactPointWeightSum{ 0.0f };
            RE::NiPoint3 rawContactPointGame{};
            RE::NiPoint3 rawContactNormalHavok{};
            RE::NiTransform requestedProxyBodyWorld{};
            RE::NiTransform liveProxyBodyWorld{};
            RE::NiTransform otherBodyWorld{};
        };

        void captureVisualIntent(
            RE::NiNode* weaponNode,
            const RE::NiTransform& requestedWeaponWorld,
            std::uint64_t weaponGenerationKey);
        bool ensureProxyBody(
            const PhysicsFrameContext& frame,
            const WeaponCollision& weaponCollision,
            const RE::NiTransform& requestedWeaponWorld);
        bool queueCompoundChildTransforms(
            const WeaponCollision& weaponCollision,
            const RE::NiAVObject* weaponNode,
            float weaponScale);
        void retireProxyLocked(void* bhkWorld);
        void clearLocalProxyStateLocked();
        void clearPublishedPhysicsSnapshot();
        void publishPhysicsSnapshot(const PhysicsSnapshot& snapshot);
        bool readPhysicsSnapshot(PhysicsSnapshot& outSnapshot) const;
        void clearContactDiagnosticSnapshot();
        void publishContactDiagnosticSnapshot(const ContactDiagnosticSnapshot& snapshot);
        bool readContactDiagnosticSnapshot(ContactDiagnosticSnapshot& outSnapshot) const;
        static void storeAtomicTransform(AtomicTransform& target, const RE::NiTransform& value);
        static RE::NiTransform loadAtomicTransform(const AtomicTransform& source);

        PhysicsCallbackQuiescenceGate* _physicsCallbackGate{ nullptr };
        BethesdaPhysicsBody _body{};
        BethesdaPhysicsBody _authorityProxy{};
        ActiveConstraint _authorityConstraint{};
        havok_compound_shape_builder::DynamicCompoundShape _compoundShape{};
        mutable std::mutex _compoundPoseMutex;
        std::vector<WeaponCollision::CompoundChildPoseSnapshot> _compoundPoseScratch;
        std::vector<havok_compound_shape_builder::ChildTransform> _pendingCompoundChildTransforms;
        std::uint64_t _queuedCompoundPoseSequence{ 0 };
        std::uint64_t _consumedCompoundPoseSequence{ 0 };
        GeneratedKeyframedBodyDriveState _authorityDriveState{};
        RE::hknpWorld* _createdWorld{ nullptr };
        void* _createdBhkWorld{ nullptr };
        std::uint64_t _createdGenerationKey{ 0 };
        RE::NiPoint3 _createdCenterWeaponLocal{};
        RE::NiPoint3 _createdHalfExtentsWeaponLocal{};
        float _createdWeaponScale{ 1.0f };
        std::uint32_t _createdCompoundChildCount{ 0 };
        std::size_t _createdCompoundPointCount{ 0 };
        std::array<std::uint32_t, MAX_WEAPON_COLLISION_BODIES>
            _createdSourceBodyIds{};
        std::uint32_t _createdSourceBodyIdCount{ 0 };
        bool _created{ false };
        bool _droveThisSubstep{ false };
        bool _physicsRequestedTargetValid{ false };
        bool _physicsDriveTeleported{ false };
        RE::NiTransform _physicsRequestedAuthorityTarget{};
        RE::NiTransform _physicsRequestedTarget{};
        RE::NiTransform _physicsPreviousRequestedTarget{};
        bool _physicsPreviousRequestedTargetValid{ false };
        // Physics-thread-only persistence timer for the real colliding body.
        // The hidden authority target may jump immediately after a tracking
        // discontinuity, but blocked physical divergence must persist before
        // the contact body is recovered.
        float _divergenceDwellSeconds{ 0.0f };
        std::uint64_t _consumedContactSequence{ 0 };
        std::uint32_t _contactGraceSolves{ 0 };
        std::uint64_t _contactEpisode{ 0 };
        std::uint64_t _reportedContactEpisode{ 0 };
        std::uint64_t _postSolveSamplesSinceCreate{ 0 };
        std::uint64_t _lastEpisodeRawWitnessSequence{ 0 };
        std::uint32_t _activeContactOtherBodyId{ 0x7FFF'FFFFu };

        bool _frameAcceptingIntent{ false };
        bool _frameHasIntent{ false };
        std::uint64_t _frameIndex{ 0 };
        std::uint64_t _frameGenerationKey{ 0 };
        RE::hknpWorld* _frameWorld{ nullptr };
        void* _frameBhkWorld{ nullptr };
        RE::NiNode* _frameWeaponNode{ nullptr };
        RE::NiTransform _frameRequestedWeaponWorld{};
        DebugSnapshot _debugSnapshot{};

        std::atomic<bool> _enabledAtomic{ false };
        std::atomic<float> _gripRecoveryDistanceGameUnitsAtomic{ 210.0f };
        std::atomic<std::uint32_t> _bodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<bool> _rebuildRequestedAtomic{ false };
        std::atomic<std::uint64_t> _proxyPairCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _obstacleCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _rawPointCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _processedManifoldCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _contactSequenceAtomic{ 0 };
        std::atomic<std::uintptr_t> _contactWorldAtomic{ 0 };
        std::atomic<std::uint32_t> _contactProxyBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactOtherBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactOtherLayerAtomic{ 0 };
        std::atomic<std::uint32_t> _rawContactOtherBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _rawContactPointCountAtomic{ 0 };
        std::atomic<std::uint32_t> _rawContactPointIndexAtomic{ 0 };
        std::atomic<float> _rawContactPointWeightSumAtomic{ 0.0f };
        std::array<std::atomic<float>, 3> _rawContactPointHavokAtomic{};
        std::array<std::atomic<float>, 3> _rawContactNormalHavokAtomic{};
        std::atomic<bool> _rawContactProxyWasBodyAAtomic{ false };
        std::atomic<std::uint64_t> _rawContactWitnessSequenceAtomic{ 0 };

        std::atomic<std::uint64_t> _snapshotVersionAtomic{ 0 };
        std::atomic<bool> _snapshotValidAtomic{ false };
        std::atomic<bool> _snapshotContactActiveAtomic{ false };
        std::atomic<bool> _snapshotTeleportedAtomic{ false };
        std::atomic<std::uintptr_t> _snapshotWorldAtomic{ 0 };
        std::atomic<std::uint32_t> _snapshotBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _snapshotOtherBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _snapshotOtherLayerAtomic{ 0 };
        std::atomic<std::uint32_t> _snapshotContactGraceAtomic{ 0 };
        std::atomic<std::uint64_t> _snapshotGenerationKeyAtomic{ 0 };
        std::atomic<std::uint64_t> _snapshotSolveSequenceAtomic{ 0 };
        std::atomic<float> _snapshotWeaponScaleAtomic{ 1.0f };
        AtomicTransform _snapshotRequestedProxyBodyWorld{};
        AtomicTransform _snapshotLiveProxyBodyWorld{};

        std::atomic<std::uint64_t> _contactDiagnosticVersionAtomic{ 0 };
        std::atomic<bool> _contactDiagnosticValidAtomic{ false };
        std::atomic<bool> _contactDiagnosticRawPointValidAtomic{ false };
        std::atomic<bool> _contactDiagnosticProxyWasBodyAAtomic{ false };
        std::atomic<bool> _contactDiagnosticOtherWorldValidAtomic{ false };
        std::atomic<std::uintptr_t> _contactDiagnosticWorldAtomic{ 0 };
        std::atomic<std::uint32_t> _contactDiagnosticBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactDiagnosticOtherBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactDiagnosticOtherLayerAtomic{ 0 };
        std::atomic<std::uint32_t> _contactDiagnosticOtherMotionIndexAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactDiagnosticRawPointCountAtomic{ 0 };
        std::atomic<std::uint32_t> _contactDiagnosticRawPointIndexAtomic{ 0 };
        std::atomic<std::uint64_t> _contactDiagnosticGenerationKeyAtomic{ 0 };
        std::atomic<std::uint64_t> _contactDiagnosticEpisodeAtomic{ 0 };
        std::atomic<std::uint64_t> _contactDiagnosticSolveAgeAtomic{ 0 };
        std::atomic<std::uintptr_t> _contactDiagnosticOtherCollisionObjectAtomic{ 0 };
        std::atomic<std::uintptr_t> _contactDiagnosticOtherOwnerNodeAtomic{ 0 };
        std::atomic<float> _contactDiagnosticRawPointWeightSumAtomic{ 0.0f };
        std::array<std::atomic<float>, 3> _contactDiagnosticRawPointGameAtomic{};
        std::array<std::atomic<float>, 3> _contactDiagnosticRawNormalHavokAtomic{};
        AtomicTransform _contactDiagnosticRequestedProxyBodyWorld{};
        AtomicTransform _contactDiagnosticLiveProxyBodyWorld{};
        AtomicTransform _contactDiagnosticOtherBodyWorld{};
    };
}

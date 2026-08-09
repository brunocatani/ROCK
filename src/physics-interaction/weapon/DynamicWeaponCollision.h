#pragma once

#include "physics-interaction/native/BethesdaPhysicsBody.h"
#include "physics-interaction/native/GeneratedKeyframedBodyDrive.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiTransform.h"

#include <array>
#include <atomic>
#include <cstdint>

namespace RE
{
    class NiNode;
    class hknpShape;
    class hknpWorld;
}

namespace rock
{
    class PhysicsCallbackQuiescenceGate;
    struct PhysicsFrameContext;
    class WeaponCollision;

    class DynamicWeaponCollisionRuntime
    {
    public:
        struct FrameResult
        {
            bool proxyActive{ false };
            bool applyVisualCorrection{ false };
            RE::NiTransform requestedWeaponWorld{};
            RE::NiTransform resolvedWeaponWorld{};
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
            std::uint32_t otherBodyId{ 0x7FFF'FFFFu };
            std::uint32_t otherLayer{ 0 };
            std::uint32_t contactGraceSolves{ 0 };
            std::uint64_t generationKey{ 0 };
            std::uint64_t solveSequence{ 0 };
            std::uint64_t proxyPairCallbackSequence{ 0 };
            std::uint64_t worldSurfaceCallbackSequence{ 0 };
            std::uint64_t rawPointCallbackSequence{ 0 };
            std::uint64_t processedManifoldCallbackSequence{ 0 };
            std::uint64_t admittedContactSequence{ 0 };
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
            bool enabled);

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
        void recordWorldSurfaceContactCallback(
            RE::hknpWorld* world,
            std::uint32_t proxyBodyId,
            std::uint32_t otherBodyId,
            bool otherLayerRead,
            std::uint32_t otherLayer,
            bool rawContactPointValid);
        void recordWorldSurfaceManifoldProcessedCallback(
            RE::hknpWorld* world,
            std::uint32_t proxyBodyId,
            std::uint32_t otherBodyId,
            bool otherLayerRead,
            std::uint32_t otherLayer);

        void retireAll(void* bhkWorld);
        void abandonHavokStateAfterWorldLoss();

        [[nodiscard]] RE::hknpBodyId proxyBodyIdForDebug() const;
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

        void captureVisualIntent(
            RE::NiNode* weaponNode,
            const RE::NiTransform& requestedWeaponWorld,
            std::uint64_t weaponGenerationKey);
        bool ensureProxyBody(
            const PhysicsFrameContext& frame,
            const WeaponCollision& weaponCollision,
            const RE::NiTransform& requestedWeaponWorld);
        void retireProxyLocked(void* bhkWorld);
        void clearLocalProxyStateLocked();
        void clearPublishedPhysicsSnapshot();
        void publishPhysicsSnapshot(const PhysicsSnapshot& snapshot);
        bool readPhysicsSnapshot(PhysicsSnapshot& outSnapshot) const;
        static void storeAtomicTransform(AtomicTransform& target, const RE::NiTransform& value);
        static RE::NiTransform loadAtomicTransform(const AtomicTransform& source);

        PhysicsCallbackQuiescenceGate* _physicsCallbackGate{ nullptr };
        BethesdaPhysicsBody _body{};
        const RE::hknpShape* _shape{ nullptr };
        GeneratedKeyframedBodyDriveState _driveState{};
        RE::hknpWorld* _createdWorld{ nullptr };
        void* _createdBhkWorld{ nullptr };
        std::uint64_t _createdGenerationKey{ 0 };
        RE::NiPoint3 _createdCenterWeaponLocal{};
        RE::NiPoint3 _createdHalfExtentsWeaponLocal{};
        float _createdWeaponScale{ 1.0f };
        float _createdPaddingGameUnits{ 0.0f };
        bool _created{ false };
        bool _droveThisSubstep{ false };
        bool _physicsRequestedTargetValid{ false };
        bool _physicsLiveTargetValid{ false };
        bool _physicsDriveTeleported{ false };
        RE::NiTransform _physicsRequestedTarget{};
        RE::NiTransform _physicsLiveTarget{};
        float _divergenceDwellSeconds{ 0.0f };
        std::uint64_t _consumedContactSequence{ 0 };
        std::uint32_t _contactGraceSolves{ 0 };

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
        std::atomic<std::uint32_t> _bodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<bool> _rebuildRequestedAtomic{ false };
        std::atomic<std::uint64_t> _proxyPairCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _worldSurfaceCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _rawPointCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _processedManifoldCallbackSequenceAtomic{ 0 };
        std::atomic<std::uint64_t> _contactSequenceAtomic{ 0 };
        std::atomic<std::uintptr_t> _contactWorldAtomic{ 0 };
        std::atomic<std::uint32_t> _contactProxyBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactOtherBodyIdAtomic{ 0x7FFF'FFFFu };
        std::atomic<std::uint32_t> _contactOtherLayerAtomic{ 0 };

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
    };
}

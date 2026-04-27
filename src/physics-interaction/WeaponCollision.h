#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <string>
#include <vector>

#include "BethesdaPhysicsBody.h"
#include "HandCollisionSuppressionMath.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"
#include "WeaponCollisionLimits.h"
#include "WeaponSemanticTypes.h"

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

namespace frik::rock
{

    inline constexpr std::uint32_t ROCK_WEAPON_LAYER = 44;

    struct WeaponVisualKeyStats
    {
        std::uint32_t rootCount = 0;
        std::uint32_t nodeCount = 0;
        std::uint32_t triShapeCount = 0;
    };

    class WeaponCollision
    {
    public:
        WeaponCollision();

        void init(RE::hknpWorld* world, void* bhkWorld);

        void shutdown();

        void update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, RE::hknpBodyId dominantHandBodyId, float dt);

        bool hasWeaponBody() const;

        std::uint32_t getWeaponBodyCount() const;

        RE::hknpBodyId getWeaponBodyId() const;

        std::uint32_t getWeaponBodyIdAtomic() const;

        std::uint32_t getWeaponBodyIdAtomic(std::size_t index) const;

        bool isWeaponBodyIdAtomic(std::uint32_t bodyId) const;

        bool tryGetWeaponContactAtomic(std::uint32_t bodyId, WeaponInteractionContact& outContact) const;

        bool tryFindInteractionContactNearPoint(
            const RE::NiAVObject* weaponNode,
            const RE::NiPoint3& probeWorldPoint,
            float probeRadiusGame,
            WeaponInteractionContact& outContact) const;

        BethesdaPhysicsBody& getWeaponBody();

        void destroyWeaponBody(RE::hknpWorld* world);

        void updateBodiesFromWeaponRootTransform(RE::hknpWorld* world, const RE::NiTransform& weaponRootTransform, float dt);

    private:
        static constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::size_t MAX_WEAPON_BODIES = MAX_WEAPON_COLLISION_BODIES;

        struct GeneratedHullSource
        {
            std::vector<RE::NiPoint3> localPointsGame;
            RE::NiPoint3 localCenterGame{};
            RE::NiPoint3 localMinGame{};
            RE::NiPoint3 localMaxGame{};
            RE::NiAVObject* sourceRoot{ nullptr };
            std::string sourceName;
            WeaponPartClassification semantic{};
        };

        struct WeaponBodyInstance
        {
            BethesdaPhysicsBody body;
            const RE::hknpShape* shape{ nullptr };
            RE::NiAVObject* sourceNode{ nullptr };
            RE::NiPoint3 generatedLocalCenterGame{};
            RE::NiPoint3 generatedLocalMinGame{};
            RE::NiPoint3 generatedLocalMaxGame{};
            WeaponPartClassification semantic{};
            bool ownsShapeRef{ false };
            bool hasPrevTransform{ false };
            RE::NiPoint3 prevPosition{};
        };

        void createGeneratedWeaponBodies(RE::hknpWorld* world, const std::vector<GeneratedHullSource>& sources);
        void clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef);
        void clearAtomicBodyIds();
        void publishAtomicBodyIds();

        std::size_t findGeneratedWeaponShapeSources(RE::NiAVObject* weaponNode, std::vector<GeneratedHullSource>& outSources);

        void findGeneratedWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::NiAVObject* sourceRoot, const RE::NiTransform& weaponRootTransform, int depth,
            std::vector<GeneratedHullSource>& outSources, std::uint32_t& visitedShapes, std::uint32_t& extractedTriangles);
        RE::NiTransform makeGeneratedBodyWorldTransform(const RE::NiTransform& weaponRootTransform, const RE::NiPoint3& localCenterGame) const;
        bool weaponCollisionSettingsChanged() const;

        std::uint64_t getEquippedWeaponKey(RE::NiAVObject* weaponNode, WeaponVisualKeyStats& stats) const;

        void updateBodyTransform(RE::hknpWorld* world, WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float dt, std::size_t bodyIndex);

        std::array<WeaponBodyInstance, MAX_WEAPON_BODIES> _weaponBodies{};
        std::uint64_t _cachedWeaponKey{ 0 };
        RE::hknpWorld* _cachedWorld{ nullptr };
        void* _cachedBhkWorld{ nullptr };
        bool _weaponBodyPending{ false };

        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyIdsAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyPartKindsAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyReloadRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodySupportRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodySocketRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyActionRolesAtomic;
        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyGripPosesAtomic;
        std::atomic<std::uint32_t> _weaponBodyCountAtomic{ 0 };

        int _retryCounter{ 0 };

        int _posLogCounter{ 0 };

        float _cachedConvexRadius{ -1.0f };
        float _cachedPointDedupGrid{ -1.0f };

        bool _dominantHandDisabled{ false };
        hand_collision_suppression_math::SuppressionState _dominantHandSuppression{};
        bool _dominantHandBroadPhaseSuppressed{ false };
        RE::hknpBodyId _disabledHandBodyId{ INVALID_BODY_ID };

        void disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId);

        void enableDominantHandCollision(RE::hknpWorld* world);
    };

}

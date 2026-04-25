#pragma once

#include <array>
#include <atomic>
#include <cstddef>

#include "BethesdaPhysicsBody.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

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

        bool isWeaponBodyIdAtomic(std::uint32_t bodyId) const;

        BethesdaPhysicsBody& getWeaponBody();

        void destroyWeaponBody(RE::hknpWorld* world);

    private:
        static constexpr std::uint32_t INVALID_BODY_ID = 0x7FFF'FFFF;
        static constexpr std::size_t MAX_WEAPON_BODIES = 16;

        struct WeaponShapeSource
        {
            const RE::hknpShape* shape{ nullptr };
            RE::hknpBodyId sourceBodyId{ INVALID_BODY_ID };
            RE::NiAVObject* sourceNode{ nullptr };
        };

        using WeaponShapeSourceList = std::array<WeaponShapeSource, MAX_WEAPON_BODIES>;

        struct WeaponBodyInstance
        {
            BethesdaPhysicsBody body;
            const RE::hknpShape* shape{ nullptr };
            RE::hknpBodyId sourceBodyId{ INVALID_BODY_ID };
            RE::NiAVObject* sourceNode{ nullptr };
            bool hasPrevTransform{ false };
            RE::NiPoint3 prevPosition{};
        };

        void createWeaponBodies(RE::hknpWorld* world, const WeaponShapeSourceList& sources, std::size_t sourceCount);
        void resetWeaponBodiesWithoutDestroy();
        void clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef);
        void clearAtomicBodyIds();
        void publishAtomicBodyIds();

        std::size_t findWeaponShapeSources(RE::NiAVObject* weaponNode, RE::hknpWorld* world, WeaponShapeSourceList& outSources);

        void findWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::hknpWorld* world, int depth, WeaponShapeSourceList& outSources, std::size_t& sourceCount);
        bool addWeaponShapeSource(RE::hknpWorld* world, WeaponShapeSourceList& outSources, std::size_t& sourceCount, RE::hknpBodyId sourceBodyId, RE::NiAVObject* sourceNode,
            int depth);
        bool sourceBodiesStillValid(RE::hknpWorld* world) const;
        bool isSourceBodyValid(RE::hknpWorld* world, const WeaponBodyInstance& instance) const;
        RE::NiTransform getSourceBodyTransform(RE::hknpWorld* world, const WeaponBodyInstance& instance, const RE::NiTransform& fallbackTransform) const;

        std::uint64_t getEquippedWeaponKey() const;

        void updateBodyTransform(RE::hknpWorld* world, WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float dt, std::size_t bodyIndex);

        std::array<WeaponBodyInstance, MAX_WEAPON_BODIES> _weaponBodies{};
        std::uint64_t _cachedWeaponKey{ 0 };
        RE::hknpWorld* _cachedWorld{ nullptr };
        void* _cachedBhkWorld{ nullptr };
        bool _weaponBodyPending{ false };

        std::array<std::atomic<std::uint32_t>, MAX_WEAPON_BODIES> _weaponBodyIdsAtomic;
        std::atomic<std::uint32_t> _weaponBodyCountAtomic{ 0 };

        int _retryCounter{ 0 };

        int _posLogCounter{ 0 };

        bool _dominantHandDisabled{ false };
        bool _dominantHandCollisionWasDisabled{ false };
        RE::hknpBodyId _disabledHandBodyId{ INVALID_BODY_ID };

        void disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId);

        void enableDominantHandCollision(RE::hknpWorld* world);
    };

}

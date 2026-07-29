#pragma once

#include <cstdint>
#include <cstring>

#include "physics-interaction/PhysicsLog.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

namespace rock
{

    enum class BethesdaMotionType : int
    {
        Static = 0,
        Dynamic = 1,
        Keyframed = 2
    };

    struct RetiredBethesdaPhysicsBodyPayload
    {
        void* collisionObject = nullptr;
        void* niNode = nullptr;
        std::uint32_t bodyId = 0x7FFF'FFFF;

        [[nodiscard]] bool occupied() const { return collisionObject != nullptr || niNode != nullptr; }
    };

    class BethesdaPhysicsBody
    {
    public:
        BethesdaPhysicsBody() = default;
        ~BethesdaPhysicsBody() = default;

        BethesdaPhysicsBody(const BethesdaPhysicsBody&) = delete;
        BethesdaPhysicsBody& operator=(const BethesdaPhysicsBody&) = delete;
        BethesdaPhysicsBody(BethesdaPhysicsBody&&) = delete;
        BethesdaPhysicsBody& operator=(BethesdaPhysicsBody&&) = delete;

        bool create(RE::hknpWorld* world, void* bhkWorld, RE::hknpShape* shape, std::uint32_t filterInfo, RE::hknpMaterialId materialId, BethesdaMotionType motionType,
            const char* name = "ROCK_Body");

        void destroy(void* bhkWorld);

        bool retireFromWorld(void* bhkWorld, RetiredBethesdaPhysicsBodyPayload& outPayload);

        // Removes the body from the world now but defers freeing the underlying
        // bhkNPCollisionObject by a physics-step grace window (see .cpp). A
        // keyframed collider stays reachable from the hknp broadphase until the
        // next physics step rebuilds it, so freeing it in the same call — as
        // destroy() does — lets a native broadphase reader (foot-IK raycast,
        // navmesh obstacle manager) dereference freed memory and crash. Use this
        // for every teardown that happens while the world is still live; keep
        // destroy() only for world-loss/shutdown where no further step will run.
        void retireDeferred(void* bhkWorld);

        // Drains the shared deferred-retirement queue. Must be called from the
        // physics-step (post-solve) phase, once per completed step, so the grace
        // window is measured in real broadphase rebuilds.
        static void serviceRetiredDeferredPayloads(std::uint32_t completedPhysicsSteps = 1);

        static void releaseRetiredPayload(RetiredBethesdaPhysicsBodyPayload& payload);

        void reset();

        bool createNiNode(const char* name);

        void destroyNiNode();

        bool isValid() const { return _created && _collisionObject != nullptr; }
        RE::hknpBodyId getBodyId() const { return _bodyId; }
        void* getCollisionObject() const { return _collisionObject; }
        void* getPhysicsSystem() const { return _physicsSystem; }

        bool driveToKeyFrame(const RE::hkTransformf& target, float dt);

        bool setTransform(const RE::hkTransformf& transform);

        bool setVelocity(const float* linVel, const float* angVel);

        void setMotionType(BethesdaMotionType type);

        void setCollisionFilterInfo(std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);

        void setMass(float mass);

        bool applyLinearImpulse(const float* impulse);

        bool applyPointImpulse(const float* impulse, const float* worldPoint);

        bool getCenterOfMassWorld(float& outX, float& outY, float& outZ);

        std::uint32_t getCollisionFilterInfo();

        void* getShape();

        bool isConstrained();

        bool setPointVelocity(const float* targetVel, const float* worldPoint);

        void enableBodyFlags(std::uint32_t flags, std::uint32_t mode);

        void activateBody();

        void registerContactSignal(const char* signalName);

        void* getNiNode() const { return _niNode; }

    private:
        [[nodiscard]] bool matchesCreationWorld(RE::hknpWorld* world, void* bhkWorld) const;

        void* _collisionObject = nullptr;
        void* _physicsSystem = nullptr;
        void* _systemData = nullptr;
        void* _niNode = nullptr;
        RE::hknpWorld* _createdHknpWorld = nullptr;
        void* _createdBhkWorld = nullptr;
        RE::hknpBodyId _bodyId{ 0x7FFF'FFFF };
        bool _created = false;
    };

}

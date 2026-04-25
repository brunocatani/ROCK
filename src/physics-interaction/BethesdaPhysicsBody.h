#pragma once

#include <cstdint>
#include <cstring>

#include "PhysicsLog.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpWorld.h"

namespace frik::rock
{

    enum class BethesdaMotionType : int
    {
        Static = 0,
        Dynamic = 1,
        Keyframed = 2
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

        void reset();

        bool createNiNode(const char* name);

        void destroyNiNode();

        bool isValid() const { return _created && _collisionObject != nullptr; }
        RE::hknpBodyId getBodyId() const { return _bodyId; }
        void* getCollisionObject() const { return _collisionObject; }
        void* getPhysicsSystem() const { return _physicsSystem; }

        bool driveToKeyFrame(const RE::NiTransform& target, float dt);

        void setTransform(const RE::hkTransformf& transform);

        void setVelocity(const float* linVel, const float* angVel);

        void setMotionType(BethesdaMotionType type);

        void setCollisionFilterInfo(std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);

        void setMass(float mass);

        void applyLinearImpulse(const float* impulse);

        void applyPointImpulse(const float* impulse, const float* worldPoint);

        bool getCenterOfMassWorld(float& outX, float& outY, float& outZ);

        std::uint32_t getCollisionFilterInfo();

        void* getShape();

        bool isConstrained();

        void setPointVelocity(const float* targetVel, const float* worldPoint);

        void enableBodyFlags(std::uint32_t flags, std::uint32_t mode);

        void activateBody();

        void registerContactSignal(const char* signalName);

        void* getNiNode() const { return _niNode; }

    private:
        void* _collisionObject = nullptr;
        void* _physicsSystem = nullptr;
        void* _systemData = nullptr;
        void* _niNode = nullptr;
        RE::hknpBodyId _bodyId{ 0x7FFF'FFFF };
        bool _created = false;
    };

}

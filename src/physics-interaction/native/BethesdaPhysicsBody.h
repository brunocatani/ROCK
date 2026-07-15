#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "physics-interaction/PhysicsLog.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpMaterialId.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiTransform.h"

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
        void* _collisionObject = nullptr;
        void* _physicsSystem = nullptr;
        void* _systemData = nullptr;
        void* _niNode = nullptr;
        RE::hknpBodyId _bodyId{ 0x7FFF'FFFF };
        bool _created = false;
    };

    inline constexpr std::size_t kMaxBethesdaPhysicsBodyGroupMembers = 128;

    struct BethesdaPhysicsBodyGroupMemberCreateInfo
    {
        RE::hknpShape* shape = nullptr;
        std::uint32_t filterInfo = 0;
        RE::hknpMaterialId materialId{ 0 };
        // Normal Ni/BODY-array world frame. Group creation performs the
        // stored-basis to native quaternion conversion internally.
        RE::NiTransform initialWorld{};
        const char* name = "ROCK_GroupBody";
    };

    /*
     * One Bethesda hknpPhysicsSystemData containing N bodies and exactly one
     * motion cinfo. Every body cinfo names local motion zero, so FO4VR maps all
     * members to one world motion while preserving one collision-object wrapper
     * (and therefore one body identity) per generated shape.
     */
    class BethesdaPhysicsBodyGroup
    {
    public:
        class Member
        {
        public:
            [[nodiscard]] bool isValid() const;
            [[nodiscard]] RE::hknpBodyId getBodyId() const;
            [[nodiscard]] void* getCollisionObject() const;
            bool setVelocity(const float* linearVelocity, const float* angularVelocity);
            bool driveToKeyFrame(const RE::hkTransformf& target, float dt);

        private:
            friend class BethesdaPhysicsBodyGroup;
            Member(BethesdaPhysicsBodyGroup* owner, std::size_t index) : _owner(owner), _index(index) {}
            BethesdaPhysicsBodyGroup* _owner = nullptr;
            std::size_t _index = 0;
        };

        BethesdaPhysicsBodyGroup() = default;
        ~BethesdaPhysicsBodyGroup() = default;
        BethesdaPhysicsBodyGroup(const BethesdaPhysicsBodyGroup&) = delete;
        BethesdaPhysicsBodyGroup& operator=(const BethesdaPhysicsBodyGroup&) = delete;
        BethesdaPhysicsBodyGroup(BethesdaPhysicsBodyGroup&&) = delete;
        BethesdaPhysicsBodyGroup& operator=(BethesdaPhysicsBodyGroup&&) = delete;

        bool create(
            RE::hknpWorld* world,
            void* bhkWorld,
            const BethesdaPhysicsBodyGroupMemberCreateInfo* members,
            std::size_t memberCount,
            const char* systemName = "ROCK_BodyGroup");
        void destroy(void* bhkWorld);
        void retireDeferred(void* bhkWorld);
        // The owning hknp/bhk world has already been replaced or destroyed.
        // Drop ROCK's stale non-owning handles without calling native teardown.
        void abandonAfterWorldLoss();
        static void serviceRetiredDeferredPayloads(std::uint32_t completedPhysicsSteps = 1);

        [[nodiscard]] bool isValid() const { return _created && _memberCount != 0 && _physicsSystem != nullptr && _world != nullptr; }
        [[nodiscard]] bool belongsToWorld(const RE::hknpWorld* world) const { return isValid() && world == _world; }
        [[nodiscard]] std::size_t size() const { return _memberCount; }
        [[nodiscard]] RE::hknpBodyId getBodyId(std::size_t index) const;
        [[nodiscard]] void* getCollisionObject(std::size_t index) const;
        [[nodiscard]] std::uint32_t sharedMotionIndex() const { return _sharedMotionIndex; }
        [[nodiscard]] Member member(std::size_t index) { return Member(this, index); }

        bool setMemberTransformDeferred(RE::hknpWorld* world, std::size_t index, const RE::NiTransform& transform, int mode = 1);
        bool setSharedVelocity(const float* linearVelocity, const float* angularVelocity);
        bool rebuildMassProperties(RE::hknpWorld* world, int rebuildMode = 0);
        bool setCollisionFilterInfo(RE::hknpWorld* world, std::size_t index, std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);
        void activate(RE::hknpWorld* world);

    private:
        void reset();
        bool driveMemberToKeyFrame(std::size_t index, const RE::hkTransformf& target, float dt);

        std::array<void*, kMaxBethesdaPhysicsBodyGroupMembers> _collisionObjects{};
        std::array<void*, kMaxBethesdaPhysicsBodyGroupMembers> _niNodes{};
        std::array<RE::hknpBodyId, kMaxBethesdaPhysicsBodyGroupMembers> _bodyIds{};
        // Non-owning engine contexts. Valid only for the group's explicit
        // create-to-retire lifetime and used to reject cross-world body IDs.
        RE::hknpWorld* _world = nullptr;
        void* _bhkWorld = nullptr;
        void* _physicsSystem = nullptr;
        void* _systemData = nullptr;
        std::size_t _memberCount = 0;
        std::uint32_t _sharedMotionIndex = 0x7FFF'FFFF;
        bool _created = false;
    };

}

#pragma once

#include <cstddef>
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

    enum class BethesdaGeneratedBodyQuality : std::uint8_t
    {
        Default = 0,
        ForcedLinearCollisionLookAhead = 1,
    };

    struct BethesdaPhysicsBodyCreationOptions
    {
        float collisionLookAheadDistanceHavok = 0.0f;
        BethesdaGeneratedBodyQuality bodyQuality = BethesdaGeneratedBodyQuality::Default;
    };

    /*
     * FO4VR 1.2.72 hknpBody initialization at 0x1415616F0 copies body-cinfo
     * +0x1C to the runtime body's collision-look-ahead field (+0x3C) and
     * cinfo +0x50 to its body-quality byte (+0x7E). The native quality-library
     * initializer at 0x1417F2160 gives profile 1 both REQUEST (0x800) and
     * FORCE (0x1000) linear collision look-ahead. A 0.10 Havok-unit horizon
     * covers the observed 15-unit/s generated-motion cap across a 1/270 s
     * solve with margin, without expanding unrelated generated bodies.
     */
    inline constexpr BethesdaPhysicsBodyCreationOptions kTrackedDynamicBodyCreationOptions{
        0.10f,
        BethesdaGeneratedBodyQuality::ForcedLinearCollisionLookAhead,
    };

    struct RetiredBethesdaPhysicsBodyPayload
    {
        void* collisionObject = nullptr;
        void* niNode = nullptr;
        RE::hknpWorld* retiredHknpWorld = nullptr;
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
            const char* name = "ROCK_Body", const BethesdaPhysicsBodyCreationOptions& options = {});

        void destroy(void* bhkWorld);

        bool retireFromWorld(void* bhkWorld, RetiredBethesdaPhysicsBodyPayload& outPayload);

        // Removes the body from the world now, keeps the complete native wrapper
        // alive across a physics-step grace window, then converts it to a small
        // process-lifetime tombstone. FO4VR retains uncounted collision-object
        // pointers beyond broadphase removal, so the object address itself must
        // remain valid even after its node and physics system can be released.
        // Use this for every teardown that happens while the world is still live;
        // keep destroy() only for world-loss/shutdown where no further step runs.
        void retireDeferred(void* bhkWorld);

        // Advances the shared deferred-retirement queue only for the exact world
        // that removed each body. Must run in the physics post-solve phase so the
        // grace window is measured in completed broadphase rebuilds.
        static void serviceRetiredDeferredPayloads(RE::hknpWorld* currentWorld, std::uint32_t completedPhysicsSteps = 1);

        // Strips a world-removed payload to an inert collision-object tombstone
        // and transfers that address to the fixed process-lifetime quarantine.
        // Returns false without modifying payload when the quarantine is full.
        static bool quarantineRetiredPayload(RetiredBethesdaPhysicsBodyPayload& payload);

        // Queue exhaustion cannot authorize a native free. Transfer ownership to
        // an intentional process-lifetime hold and fail closed for future creates.
        static void retainRetiredPayloadForProcessLifetime(
            RetiredBethesdaPhysicsBodyPayload& payload,
            const char* owner,
            std::size_t capacity);

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

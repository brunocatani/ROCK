#pragma once

#include <cstddef>
#include <cstdint>

#include "PhysicsBodyFrame.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class hkVector4f;
    struct hknpBody;
    struct hknpMotion;
    class hknpWorld;
    class NiAVObject;
    class NiCollisionObject;
    class NiPoint3;
}

namespace frik::rock::havok_runtime
{
    /*
     * ROCK has several systems reading the same FO4VR hknp body and motion
     * fields: hand selection, grab constraints, collision suppression, debug
     * overlays, and generated collider bodies. Keeping that memory access in
     * one runtime facade preserves the HIGGS-style single Havok boundary while
     * preventing scattered call sites from growing their own offset checks,
     * allocator calls, and transform fallbacks.
     */
    inline constexpr std::uintptr_t kHknpWorldBodyHighWaterMarkOffset = 0x70;

    struct BodySnapshot
    {
        bool valid = false;
        RE::hknpBodyId bodyId{ body_frame::kInvalidBodyId };
        std::uint32_t motionIndex{ body_frame::kFreeMotionIndex };
        std::uint32_t collisionFilterInfo = 0;
        RE::hknpBody* body = nullptr;
        RE::hknpMotion* motion = nullptr;
        RE::NiCollisionObject* collisionObject = nullptr;
        RE::NiAVObject* ownerNode = nullptr;
    };

    struct ResolvedBodyWorldTransform
    {
        bool valid = false;
        RE::NiTransform transform{};
        body_frame::BodyFrameSource source{ body_frame::BodyFrameSource::Fallback };
        std::uint32_t motionIndex{ body_frame::kFreeMotionIndex };
    };

    struct ContactSignalPointResult
    {
        bool valid = false;
        std::uint32_t pointCount = 0;
        std::uint32_t selectedPointIndex = 0;
        float contactPointWeightSum = 0.0f;
        float contactPointHavok[4]{};
        float contactNormalHavok[4]{};
    };

    struct ContactSignalPointSelectionInput
    {
        std::uint32_t pointCount = 0;
        std::uint32_t contactIndex = 0;
        float pointWeights[4]{};
        float contactNormalHavok[4]{};
        float contactPointsHavok[4][4]{};
    };

    constexpr bool isValidBodyId(RE::hknpBodyId bodyId)
    {
        return bodyId.value != body_frame::kInvalidBodyId;
    }

    inline bool bodySlotCanBeRead(std::uint32_t bodyId, std::uint32_t highWaterMark)
    {
        return body_frame::bodySlotCanBeRead(bodyId, highWaterMark);
    }

    bool bodySlotLooksReadable(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    RE::hknpBody* getBodyArray(RE::hknpWorld* world);
    RE::hknpMotion* getMotionArray(RE::hknpWorld* world);
    RE::hknpBody* getBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    RE::hknpMotion* getMotion(RE::hknpWorld* world, std::uint32_t motionIndex);
    RE::hknpMotion* getBodyMotion(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    BodySnapshot snapshotBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);

    void* getQueryFilterRef(RE::hknpWorld* world);
    RE::NiCollisionObject* getCollisionObjectFromBody(const RE::hknpBody* body);
    RE::NiCollisionObject* getCollisionObjectFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    bool setCollisionObjectForBody(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiCollisionObject* collisionObject);
    RE::NiAVObject* getOwnerNodeFromCollisionObject(const RE::NiCollisionObject* collisionObject);
    RE::NiAVObject* getOwnerNodeFromBody(const RE::hknpBody* body);
    RE::NiAVObject* getOwnerNodeFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);

    RE::NiTransform bodyArrayWorldTransform(const RE::hknpBody& body);
    bool tryGetBodyArrayWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform);
    bool tryGetBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform);
    bool tryGetMotionWorldTransform(RE::hknpWorld* world, const RE::hknpBody& body, RE::NiTransform& outTransform);
    ResolvedBodyWorldTransform resolveLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    bool tryResolveLiveBodyWorldTransform(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform,
        body_frame::BodyFrameSource* outSource = nullptr,
        std::uint32_t* outMotionIndex = nullptr);

    bool getBodyCOMWorld(RE::hknpWorld* world, RE::hknpBodyId bodyId, float& outX, float& outY, float& outZ);
    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo);
    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);
    bool setBodyVelocityDeferred(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        const RE::hkVector4f& linearVelocity,
        const RE::hkVector4f& angularVelocity);
    bool activateBody(RE::hknpWorld* world, std::uint32_t bodyId);
    bool applyLinearVelocityDeltaDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiPoint3& velocityDeltaHavok);
    bool rebuildMotionMassProperties(RE::hknpWorld* world, std::uint32_t motionIndex, int rebuildMode = 0);
    bool selectContactSignalPoint(const ContactSignalPointSelectionInput& input, ContactSignalPointResult& outResult);
    bool tryExtractContactSignalPoint(RE::hknpWorld* world, const void* contactSignalData, ContactSignalPointResult& outResult);

    void* allocateHavok(std::size_t size);
    void freeHavok(void* ptr, std::size_t size);
    bool hkArrayReserveMore(void* arrayBase, int elementSize);
}

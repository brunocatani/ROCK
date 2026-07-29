#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "physics-interaction/PhysicsBodyFrame.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class bhkWorld;
    class bhkPhysicsSystem;
    class hkVector4f;
    struct hknpBody;
    struct hknpMotion;
    class hknpWorld;
    class NiAVObject;
    class NiCollisionObject;
    class NiPoint3;
}

namespace rock::havok_runtime
{
    /*
     * ROCK has several systems reading the same FO4VR hknp body and motion
     * fields: hand selection, grab constraints, collision suppression, debug
     * overlays, and generated collider bodies. Keeping that memory access in
     * one runtime facade preserves a single Havok boundary and prevents scattered
     * call sites from growing their own offset checks, allocator calls, and
     * transform fallbacks.
     */
    inline constexpr std::uintptr_t kHknpWorldBodyHighWaterMarkOffset = 0x70;
    inline constexpr std::uint32_t kMaxContactSignalPoints = 4;
    inline constexpr std::uint32_t kMaxMotionPropertiesSnapshotRecords = 16;

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

    struct MotionVelocityCaps
    {
        bool valid = false;
        std::uint16_t maxLinearVelocityPacked = 0;
        std::uint16_t maxAngularVelocityPacked = 0;
        float maxLinearVelocity = 0.0f;
        float maxAngularVelocity = 0.0f;
    };

    struct MotionPropertiesSnapshotRecord
    {
        float values[16]{};
        std::uint32_t words[16]{};
    };

    struct MotionPropertiesLibrarySnapshot
    {
        bool valid = false;
        std::uint32_t count = 0;
        std::uint32_t copiedCount = 0;
        MotionPropertiesSnapshotRecord records[kMaxMotionPropertiesSnapshotRecords]{};
    };

    enum class PhysicsSystemBodyScanStatus : std::uint8_t
    {
        Enumerated,
        InvalidArguments,
        MissingPhysicsSystem,
        MissingInstance,
        WorldMismatch,
        InvalidBodyCount,
        MissingBodyIds,
        UnreadableBodyIds,
        NoBodies,
        VisitorStopped,
    };

    struct PhysicsSystemBodyScanResult
    {
        PhysicsSystemBodyScanStatus status = PhysicsSystemBodyScanStatus::InvalidArguments;
        std::int32_t bodyCount = 0;
        std::uint32_t visitedBodies = 0;
        std::uint32_t skippedInvalidBodies = 0;

        [[nodiscard]] constexpr bool enumerated() const noexcept
        {
            return status == PhysicsSystemBodyScanStatus::Enumerated;
        }
    };

    constexpr bool isValidBodyId(RE::hknpBodyId bodyId)
    {
        return bodyId.value != body_frame::kInvalidBodyId;
    }

    inline bool bodySlotCanBeRead(std::uint32_t bodyId, std::uint32_t highWaterMark)
    {
        return body_frame::bodySlotCanBeRead(bodyId, highWaterMark);
    }

    bool tryReadBodyHighWaterMark(RE::hknpWorld* world, std::uint32_t& outHighWaterMark);
    bool bodySlotLooksReadable(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    RE::hknpBody* getBodyArray(RE::hknpWorld* world);
    RE::hknpMotion* getMotionArray(RE::hknpWorld* world);
    RE::hknpBody* getBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    RE::hknpMotion* getMotion(RE::hknpWorld* world, std::uint32_t motionIndex);
    RE::hknpMotion* getBodyMotion(RE::hknpWorld* world, RE::hknpBodyId bodyId);
    BodySnapshot snapshotBody(RE::hknpWorld* world, RE::hknpBodyId bodyId);

    RE::hknpWorld* getHknpWorldFromBhk(RE::bhkWorld* bhkWorld);
    RE::bhkPhysicsSystem* getPhysicsSystemFromCollisionObject(RE::NiCollisionObject* collisionObject);
    void* getPhysicsSystemInstance(RE::bhkPhysicsSystem* physicsSystem);
    const char* physicsSystemBodyScanStatusName(PhysicsSystemBodyScanStatus status);
    PhysicsSystemBodyScanResult forEachPhysicsSystemBodyIdDetailed(
        RE::NiCollisionObject* collisionObject,
        RE::hknpWorld* expectedWorld,
        std::uint32_t maxBodies,
        bool (*visitor)(std::uint32_t bodyId, void* userData),
        void* userData);
    bool forEachPhysicsSystemBodyId(
        RE::NiCollisionObject* collisionObject,
        RE::hknpWorld* expectedWorld,
        std::uint32_t maxBodies,
        bool (*visitor)(std::uint32_t bodyId, void* userData),
        void* userData);
    void* getQueryFilterRef(RE::hknpWorld* world);
    void* getQueryFilterRefWithFallback(RE::hknpWorld* world);
    std::uint64_t* getCollisionFilterMatrix(void* filter);
    std::uint64_t* getCollisionFilterMatrix(RE::hknpWorld* world, bool* outUsedFallback = nullptr);
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
    bool tryReadMotionPropertiesId(const RE::hknpMotion* motion, std::uint16_t& outMotionPropertiesId);
    bool tryReadBodyMotionPropertiesId(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint16_t& outMotionPropertiesId);
    bool tryReadMotionVelocityCaps(const RE::hknpMotion* motion, MotionVelocityCaps& outCaps);
    bool snapshotMotionPropertiesLibrary(RE::hknpWorld* world, MotionPropertiesLibrarySnapshot& outSnapshot);
    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo);
    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo, std::uint32_t rebuildMode = 0);
    bool setBodyVelocityDeferred(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        const RE::hkVector4f& linearVelocity,
        const RE::hkVector4f& angularVelocity);
    bool setBodyTransformDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiTransform& transform, int mode = 1);
    bool activateBody(RE::hknpWorld* world, std::uint32_t bodyId);
    bool enableBodyFlags(RE::hknpWorld* world, std::uint32_t bodyId, std::uint32_t flags, std::uint32_t mode);
    bool disableBodyFlags(RE::hknpWorld* world, std::uint32_t bodyId, std::uint32_t flags, std::uint32_t mode);
    bool acquireBodyFlagLease(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint32_t flags,
        std::uint32_t mode,
        std::uintptr_t ownerToken);
    bool releaseBodyFlagLease(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint32_t flags,
        std::uint32_t mode,
        std::uintptr_t ownerToken,
        bool restoreOnFinalLease = true);
    bool applyLinearVelocityDeltaDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiPoint3& velocityDeltaHavok);
    bool rebuildMotionMassProperties(RE::hknpWorld* world, std::uint32_t motionIndex, int rebuildMode = 0);
    inline bool isFinite3(const float* value)
    {
        return value && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
    }

    inline float lengthSquared3(const float* value)
    {
        return value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
    }

    inline void copyVector4(const float* source, float* target)
    {
        target[0] = source[0];
        target[1] = source[1];
        target[2] = source[2];
        target[3] = source[3];
    }

    inline bool selectContactSignalPoint(const ContactSignalPointSelectionInput& input, ContactSignalPointResult& outResult)
    {
        outResult = {};

        const std::uint32_t pointCount = (std::min)(input.pointCount, kMaxContactSignalPoints);
        if (pointCount == 0 || !isFinite3(input.contactNormalHavok)) {
            return false;
        }

        const float normalLengthSquared = lengthSquared3(input.contactNormalHavok);
        if (!std::isfinite(normalLengthSquared) || normalLengthSquared <= 0.000001f) {
            return false;
        }

        const float invNormalLength = 1.0f / std::sqrt(normalLengthSquared);
        outResult.contactNormalHavok[0] = input.contactNormalHavok[0] * invNormalLength;
        outResult.contactNormalHavok[1] = input.contactNormalHavok[1] * invNormalLength;
        outResult.contactNormalHavok[2] = input.contactNormalHavok[2] * invNormalLength;
        outResult.contactNormalHavok[3] = input.contactNormalHavok[3];

        float weightedPoint[4]{};
        float totalWeight = 0.0f;
        for (std::uint32_t i = 0; i < pointCount; ++i) {
            const float weight = input.pointWeights[i];
            if (!std::isfinite(weight) || weight <= 0.0f || !isFinite3(input.contactPointsHavok[i])) {
                continue;
            }

            totalWeight += weight;
            weightedPoint[0] += input.contactPointsHavok[i][0] * weight;
            weightedPoint[1] += input.contactPointsHavok[i][1] * weight;
            weightedPoint[2] += input.contactPointsHavok[i][2] * weight;
            weightedPoint[3] += input.contactPointsHavok[i][3] * weight;
        }

        outResult.pointCount = pointCount;
        outResult.contactPointWeightSum = totalWeight;

        if (totalWeight > 0.0f) {
            const float invWeight = 1.0f / totalWeight;
            outResult.contactPointHavok[0] = weightedPoint[0] * invWeight;
            outResult.contactPointHavok[1] = weightedPoint[1] * invWeight;
            outResult.contactPointHavok[2] = weightedPoint[2] * invWeight;
            outResult.contactPointHavok[3] = weightedPoint[3] * invWeight;
            outResult.valid = true;
            return true;
        }

        std::uint32_t selectedIndex = input.contactIndex < pointCount ? input.contactIndex : 0;
        if (!isFinite3(input.contactPointsHavok[selectedIndex])) {
            selectedIndex = pointCount;
            for (std::uint32_t i = 0; i < pointCount; ++i) {
                if (isFinite3(input.contactPointsHavok[i])) {
                    selectedIndex = i;
                    break;
                }
            }
        }

        if (selectedIndex >= pointCount) {
            outResult = {};
            return false;
        }

        outResult.selectedPointIndex = selectedIndex;
        copyVector4(input.contactPointsHavok[selectedIndex], outResult.contactPointHavok);
        outResult.valid = true;
        return true;
    }
    bool tryExtractContactSignalPoint(RE::hknpWorld* world, const void* contactSignalData, ContactSignalPointResult& outResult);

    void* allocateHavok(std::size_t size);
    void freeHavok(void* ptr, std::size_t size);
    bool hkArrayReserveMore(void* arrayBase, int elementSize);
}

#pragma once

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/TransformMath.h"

#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"
#include "RE/NetImmerse/NiTransform.h"

namespace rock
{
    inline constexpr std::uintptr_t kHknpWorldBodyHighWaterMarkOffset = havok_runtime::kHknpWorldBodyHighWaterMarkOffset;

    inline float gameToHavokScale() { return physics_scale::gameToHavok(); }

    inline float havokToGameScale() { return physics_scale::havokToGame(); }

    inline RE::hkVector4f niPointToHkVector(const RE::NiPoint3& p)
    {
        const float scale = gameToHavokScale();
        return RE::hkVector4f{ p.x * scale, p.y * scale, p.z * scale, 0.0f };
    }

    inline RE::NiPoint3 hkVectorToNiPoint(const RE::hkVector4f& v)
    {
        const float scale = havokToGameScale();
        return RE::NiPoint3{ v.x * scale, v.y * scale, v.z * scale };
    }

    inline void* getQueryFilterRef(RE::hknpWorld* world)
    {
        return havok_runtime::getQueryFilterRef(world);
    }

    inline RE::NiCollisionObject* getCollisionObjectFromBody(const RE::hknpBody* body)
    {
        return havok_runtime::getCollisionObjectFromBody(body);
    }

    inline bool bodySlotLooksReadable(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        return havok_runtime::bodySlotLooksReadable(world, bodyId);
    }

    inline RE::NiCollisionObject* getCollisionObjectFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        return havok_runtime::getCollisionObjectFromBody(world, bodyId);
    }

    inline RE::NiAVObject* getOwnerNodeFromCollisionObject(const RE::NiCollisionObject* collisionObject)
    {
        return havok_runtime::getOwnerNodeFromCollisionObject(collisionObject);
    }

    inline RE::NiAVObject* getOwnerNodeFromBody(const RE::hknpBody* body) { return havok_runtime::getOwnerNodeFromBody(body); }

    inline RE::NiAVObject* getOwnerNodeFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId) { return havok_runtime::getOwnerNodeFromBody(world, bodyId); }

    inline RE::NiMatrix3 niRotToHkTransformRotation(const RE::NiMatrix3& m) { return transform_math::niRowsToHavokColumns(m); }

    inline RE::NiMatrix3 havokRotationBlocksToNiMatrix(const float* bodyFloats) { return transform_math::havokColumnsToNiRows<RE::NiMatrix3>(bodyFloats); }

    using ResolvedBodyWorldTransform = havok_runtime::ResolvedBodyWorldTransform;

    inline RE::NiTransform bodyArrayWorldTransform(const RE::hknpBody& body)
    {
        return havok_runtime::bodyArrayWorldTransform(body);
    }

    inline bool tryGetBodyArrayWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
    {
        return havok_runtime::tryGetBodyArrayWorldTransform(world, bodyId, outTransform);
    }

    inline bool tryGetBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
    {
        return havok_runtime::tryGetBodyWorldTransform(world, bodyId, outTransform);
    }

    inline bool tryGetMotionWorldTransform(RE::hknpWorld* world, const RE::hknpBody& body, RE::NiTransform& outTransform)
    {
        return havok_runtime::tryGetMotionWorldTransform(world, body, outTransform);
    }

    inline ResolvedBodyWorldTransform resolveLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        return havok_runtime::resolveLiveBodyWorldTransform(world, bodyId);
    }

    inline bool tryResolveLiveBodyWorldTransform(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform,
        body_frame::BodyFrameSource* outSource = nullptr,
        std::uint32_t* outMotionIndex = nullptr)
    {
        return havok_runtime::tryResolveLiveBodyWorldTransform(world, bodyId, outTransform, outSource, outMotionIndex);
    }

    inline RE::NiPoint3 worldDeltaToBodyLocal(const float* bodyFloats, const RE::NiPoint3& worldDelta)
    {
        return RE::NiPoint3{ bodyFloats[0] * worldDelta.x + bodyFloats[1] * worldDelta.y + bodyFloats[2] * worldDelta.z,
            bodyFloats[4] * worldDelta.x + bodyFloats[5] * worldDelta.y + bodyFloats[6] * worldDelta.z,
            bodyFloats[8] * worldDelta.x + bodyFloats[9] * worldDelta.y + bodyFloats[10] * worldDelta.z };
    }

    inline RE::hkVector4f niRotToHkQuat(const RE::NiMatrix3& m)
    {
        float values[4]{};
        transform_math::niRowsToHavokQuaternion(m, values);
        RE::hkVector4f q;
        q.x = values[0];
        q.y = values[1];
        q.z = values[2];
        q.w = values[3];
        return q;
    }

    inline void niRotToHkQuat(const RE::NiMatrix3& m, float outQuat[4])
    {
        auto q = niRotToHkQuat(m);
        outQuat[0] = q.x;
        outQuat[1] = q.y;
        outQuat[2] = q.z;
        outQuat[3] = q.w;
    }

    inline bool getBodyCOMWorld(RE::hknpWorld* world, RE::hknpBodyId bodyId, float& outX, float& outY, float& outZ)
    {
        return havok_runtime::getBodyCOMWorld(world, bodyId, outX, outY, outZ);
    }

    inline void getBodyCOMOffset(const float* bodyFloats, float& outX, float& outY, float& outZ)
    {
        outX = bodyFloats[3];
        outY = bodyFloats[7];
        outZ = bodyFloats[11];
    }
}

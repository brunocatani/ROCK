#pragma once

#include "HavokOffsets.h"

#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"

namespace frik::rock
{

    constexpr float kGameToHavokScale = 1.0f / 70.0f;
    constexpr float kHavokToGameScale = 70.0f;

    inline RE::hkVector4f niPointToHkVector(const RE::NiPoint3& p) { return RE::hkVector4f{ p.x * kGameToHavokScale, p.y * kGameToHavokScale, p.z * kGameToHavokScale, 0.0f }; }

    inline RE::NiPoint3 hkVectorToNiPoint(const RE::hkVector4f& v) { return RE::NiPoint3{ v.x * kHavokToGameScale, v.y * kHavokToGameScale, v.z * kHavokToGameScale }; }

    inline RE::NiCollisionObject* getCollisionObjectFromBody(const RE::hknpBody* body)
    {
        if (!body) {
            return nullptr;
        }

        auto* bodyBytes = reinterpret_cast<const char*>(body);
        return *reinterpret_cast<RE::NiCollisionObject* const*>(bodyBytes + offsets::kBody_CollisionObjectBackPointer);
    }

    inline RE::NiCollisionObject* getCollisionObjectFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        if (!world || bodyId.value == 0x7FFF'FFFF) {
            return nullptr;
        }

        auto* bodyArray = world->GetBodyArray();
        if (!bodyArray) {
            return nullptr;
        }

        return getCollisionObjectFromBody(&bodyArray[bodyId.value]);
    }

    inline RE::NiAVObject* getOwnerNodeFromCollisionObject(const RE::NiCollisionObject* collisionObject)
    {
        if (!collisionObject) {
            return nullptr;
        }

        return collisionObject->sceneObject;
    }

    inline RE::NiAVObject* getOwnerNodeFromBody(const RE::hknpBody* body) { return getOwnerNodeFromCollisionObject(getCollisionObjectFromBody(body)); }

    inline RE::NiAVObject* getOwnerNodeFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId) { return getOwnerNodeFromCollisionObject(getCollisionObjectFromBody(world, bodyId)); }

    inline RE::NiMatrix3 niRotToHkTransformRotation(const RE::NiMatrix3& m) { return m; }

    inline RE::NiMatrix3 havokRotationBlocksToNiMatrix(const float* bodyFloats)
    {
        RE::NiMatrix3 result;
        result.entry[0][0] = bodyFloats[0];
        result.entry[0][1] = bodyFloats[1];
        result.entry[0][2] = bodyFloats[2];
        result.entry[1][0] = bodyFloats[4];
        result.entry[1][1] = bodyFloats[5];
        result.entry[1][2] = bodyFloats[6];
        result.entry[2][0] = bodyFloats[8];
        result.entry[2][1] = bodyFloats[9];
        result.entry[2][2] = bodyFloats[10];
        result.entry[0][3] = 0.0f;
        result.entry[1][3] = 0.0f;
        result.entry[2][3] = 0.0f;
        return result;
    }

    inline RE::NiPoint3 worldDeltaToBodyLocal(const float* bodyFloats, const RE::NiPoint3& worldDelta)
    {
        return RE::NiPoint3{ bodyFloats[0] * worldDelta.x + bodyFloats[1] * worldDelta.y + bodyFloats[2] * worldDelta.z,
            bodyFloats[4] * worldDelta.x + bodyFloats[5] * worldDelta.y + bodyFloats[6] * worldDelta.z,
            bodyFloats[8] * worldDelta.x + bodyFloats[9] * worldDelta.y + bodyFloats[10] * worldDelta.z };
    }

    inline RE::hkVector4f niRotToHkQuat(const RE::NiMatrix3& m)
    {
        RE::hkVector4f q;
        const float trace = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];

        if (trace > 0.0f) {
            const float s = 0.5f / sqrtf(trace + 1.0f);
            q.w = 0.25f / s;
            q.x = (m.entry[1][2] - m.entry[2][1]) * s;
            q.y = (m.entry[2][0] - m.entry[0][2]) * s;
            q.z = (m.entry[0][1] - m.entry[1][0]) * s;
        } else if (m.entry[0][0] > m.entry[1][1] && m.entry[0][0] > m.entry[2][2]) {
            const float s = 2.0f * sqrtf(1.0f + m.entry[0][0] - m.entry[1][1] - m.entry[2][2]);
            q.w = (m.entry[1][2] - m.entry[2][1]) / s;
            q.x = 0.25f * s;
            q.y = (m.entry[0][1] + m.entry[1][0]) / s;
            q.z = (m.entry[0][2] + m.entry[2][0]) / s;
        } else if (m.entry[1][1] > m.entry[2][2]) {
            const float s = 2.0f * sqrtf(1.0f + m.entry[1][1] - m.entry[0][0] - m.entry[2][2]);
            q.w = (m.entry[2][0] - m.entry[0][2]) / s;
            q.x = (m.entry[0][1] + m.entry[1][0]) / s;
            q.y = 0.25f * s;
            q.z = (m.entry[1][2] + m.entry[2][1]) / s;
        } else {
            const float s = 2.0f * sqrtf(1.0f + m.entry[2][2] - m.entry[0][0] - m.entry[1][1]);
            q.w = (m.entry[0][1] - m.entry[1][0]) / s;
            q.x = (m.entry[0][2] + m.entry[2][0]) / s;
            q.y = (m.entry[1][2] + m.entry[2][1]) / s;
            q.z = 0.25f * s;
        }

        const float len = sqrtf(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
        if (len > 0.0f) {
            const float inv = 1.0f / len;
            q.x *= inv;
            q.y *= inv;
            q.z *= inv;
            q.w *= inv;
        }

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
        if (!world || bodyId.value == 0x7FFF'FFFF)
            return false;

        auto* bodyArray = world->GetBodyArray();
        auto& body = bodyArray[bodyId.value];
        auto motionIndex = body.motionIndex;
        if (motionIndex == 0 || motionIndex > 4096)
            return false;

        auto* worldBytes = reinterpret_cast<char*>(world);
        auto* motionArrayPtr = *reinterpret_cast<char**>(worldBytes + 0xE0);
        if (!motionArrayPtr)
            return false;

        auto* motion = reinterpret_cast<float*>(motionArrayPtr + motionIndex * 0x80);
        outX = motion[0];
        outY = motion[1];
        outZ = motion[2];
        return true;
    }

    inline void getBodyCOMOffset(const float* bodyFloats, float& outX, float& outY, float& outZ)
    {
        outX = bodyFloats[3];
        outY = bodyFloats[7];
        outZ = bodyFloats[11];
    }
}

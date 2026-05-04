#include "physics-interaction/native/HavokRuntime.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/TransformMath.h"

#include "F4SE/Impl/PCH.h"
#include "RE/RTTI.h"
#include "RE/RTTI_IDs.h"
#include "RE/VTABLE_IDs.h"
#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpCollisionQueryCollector.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"
#include "RE/NetImmerse/NiPoint.h"
#include "REL/Relocation.h"

#include <algorithm>
#include <cmath>
#include <windows.h>

namespace rock::havok_runtime
{
    namespace
    {
        constexpr std::uint32_t kMaxContactSignalPoints = 4;

        struct alignas(16) NativeContactSignalPointBuffer
        {
            std::uint32_t pointCount = 0;
            std::uint32_t field04 = 0;
            std::uint32_t field08 = 0;
            std::uint32_t field0C = 0;
            float contactNormalHavok[4]{};
            float reserved20[4]{};
            float normalImpulseHavok[4]{};
            float contactPointsHavok[kMaxContactSignalPoints][4]{};
        };
        static_assert(offsetof(NativeContactSignalPointBuffer, contactNormalHavok) == 0x10);
        static_assert(offsetof(NativeContactSignalPointBuffer, contactPointsHavok) == 0x40);
        static_assert(sizeof(NativeContactSignalPointBuffer) == 0x80);

        RE::NiPoint3 hkVectorToNiPoint(const RE::hkVector4f& value)
        {
            const float scale = physics_scale::havokToGame();
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        bool isFinite3(const float* value)
        {
            return value && std::isfinite(value[0]) && std::isfinite(value[1]) && std::isfinite(value[2]);
        }

        float lengthSquared3(const float* value)
        {
            return value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
        }

        void copyVector4(const float* source, float* target)
        {
            target[0] = source[0];
            target[1] = source[1];
            target[2] = source[2];
            target[3] = source[3];
        }

        RE::NiMatrix3 havokRotationBlocksToNiMatrix(const float* bodyFloats)
        {
            return transform_math::havokColumnsToNiRows<RE::NiMatrix3>(bodyFloats);
        }

        bool tryReadBodyHighWaterMark(const void* world, std::uint32_t* outHighWaterMark)
        {
            if (!world || !outHighWaterMark) {
                return false;
            }

#if defined(_MSC_VER)
            __try {
                const auto worldAddress = reinterpret_cast<std::uintptr_t>(world);
                *outHighWaterMark = *reinterpret_cast<const std::uint32_t*>(worldAddress + kHknpWorldBodyHighWaterMarkOffset);
                return true;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
#else
            const auto worldAddress = reinterpret_cast<std::uintptr_t>(world);
            *outHighWaterMark = *reinterpret_cast<const std::uint32_t*>(worldAddress + kHknpWorldBodyHighWaterMarkOffset);
            return true;
#endif
        }

        RE::hknpBody* getReadableBodySlot(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            if (!bodySlotLooksReadable(world, bodyId)) {
                return nullptr;
            }

            auto* bodyArray = getBodyArray(world);
            if (!bodyArray) {
                return nullptr;
            }

            auto& body = bodyArray[bodyId.value];
            return body.bodyId.value == bodyId.value ? &body : nullptr;
        }

        bool pointerLooksReadable(const void* ptr)
        {
            return reinterpret_cast<std::uintptr_t>(ptr) > 0x10000;
        }
    }

    bool bodySlotLooksReadable(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        if (!world) {
            return false;
        }

        std::uint32_t highWaterMark = 0;
        if (!tryReadBodyHighWaterMark(world, &highWaterMark)) {
            return false;
        }

        return bodySlotCanBeRead(bodyId.value, highWaterMark);
    }

    RE::hknpBody* getBodyArray(RE::hknpWorld* world)
    {
        return world ? world->GetBodyArray() : nullptr;
    }

    RE::hknpMotion* getMotionArray(RE::hknpWorld* world)
    {
        return world ? world->GetMotionArray() : nullptr;
    }

    RE::hknpBody* getBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        auto* body = getReadableBodySlot(world, bodyId);
        if (!body || body->motionIndex == body_frame::kFreeMotionIndex) {
            return nullptr;
        }

        return body;
    }

    RE::hknpMotion* getMotion(RE::hknpWorld* world, std::uint32_t motionIndex)
    {
        if (!world || !body_frame::hasUsableMotionIndex(motionIndex)) {
            return nullptr;
        }

        auto* motionArray = getMotionArray(world);
        if (!motionArray) {
            return nullptr;
        }

        return &motionArray[motionIndex];
    }

    RE::hknpMotion* getBodyMotion(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        auto* body = getBody(world, bodyId);
        if (!body) {
            return nullptr;
        }

        return getMotion(world, body->motionIndex);
    }

    BodySnapshot snapshotBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        BodySnapshot snapshot{};
        snapshot.bodyId = bodyId;

        auto* body = getBody(world, bodyId);
        if (!body) {
            return snapshot;
        }

        snapshot.valid = true;
        snapshot.body = body;
        snapshot.motionIndex = body->motionIndex;
        snapshot.collisionFilterInfo = body->collisionFilterInfo;
        snapshot.motion = getMotion(world, body->motionIndex);
        snapshot.collisionObject = getCollisionObjectFromBody(body);
        snapshot.ownerNode = getOwnerNodeFromCollisionObject(snapshot.collisionObject);
        return snapshot;
    }

    void* getQueryFilterRef(RE::hknpWorld* world)
    {
        if (!world) {
            return nullptr;
        }

        const auto modifierMgr = *reinterpret_cast<std::uintptr_t*>(reinterpret_cast<std::uintptr_t>(world) + offsets::kHknpWorld_ModifierManager);
        if (!modifierMgr) {
            return nullptr;
        }

        return *reinterpret_cast<void**>(modifierMgr + offsets::kModifierMgr_FilterPtr);
    }

    RE::hknpWorld* getHknpWorldFromBhk(RE::bhkWorld* bhkWorld)
    {
        /*
         * The bhkWorld -> hknpWorld pointer is a FO4VR native layout seam used
         * by lifecycle, provider, and frame orchestration code. Keeping the raw
         * offset here makes world acquisition part of the audited Havok runtime
         * boundary instead of reintroducing layout reads in high-level systems.
         */
        if (!bhkWorld) {
            return nullptr;
        }

        return *reinterpret_cast<RE::hknpWorld**>(reinterpret_cast<std::uintptr_t>(bhkWorld) + offsets::kBhkWorld_HknpWorldPtr);
    }

    RE::bhkPhysicsSystem* getPhysicsSystemFromCollisionObject(RE::NiCollisionObject* collisionObject)
    {
        /*
         * NiCollisionObject -> bhkPhysicsSystem is a native ownership edge used
         * by object scanning and held-body capture. Keeping the offset read here
         * lets higher-level systems enumerate body ids without learning the
         * collision-object wrapper layout.
         */
        if (!collisionObject) {
            return nullptr;
        }

        auto* field = *reinterpret_cast<void**>(reinterpret_cast<char*>(collisionObject) + offsets::kCollisionObject_PhysSystemPtr);
        if (!pointerLooksReadable(field)) {
            return nullptr;
        }

        return reinterpret_cast<RE::bhkPhysicsSystem*>(field);
    }

    void* getPhysicsSystemInstance(RE::bhkPhysicsSystem* physicsSystem)
    {
        if (!pointerLooksReadable(physicsSystem)) {
            return nullptr;
        }

        auto* instance = physicsSystem->instance;
        return pointerLooksReadable(instance) ? instance : nullptr;
    }

    bool forEachPhysicsSystemBodyId(
        RE::NiCollisionObject* collisionObject,
        RE::hknpWorld* expectedWorld,
        std::uint32_t maxBodies,
        bool (*visitor)(std::uint32_t bodyId, void* userData),
        void* userData)
    {
        if (!visitor || maxBodies == 0) {
            return false;
        }

        auto* physicsSystem = getPhysicsSystemFromCollisionObject(collisionObject);
        auto* instance = static_cast<RE::hknpPhysicsSystemInstance*>(getPhysicsSystemInstance(physicsSystem));
        if (!instance || !instance->bodyIds) {
            return false;
        }

        if (expectedWorld && instance->world != expectedWorld) {
            return false;
        }

        const std::int32_t count = (std::min)(instance->bodyCount, static_cast<std::int32_t>(maxBodies));
        for (std::int32_t i = 0; i < count; ++i) {
            const std::uint32_t bodyId = instance->bodyIds[i];
            if (bodyId == body_frame::kInvalidBodyId || bodyId > 0x000F'FFFF) {
                continue;
            }
            if (!visitor(bodyId, userData)) {
                return false;
            }
        }

        return true;
    }

    void* getQueryFilterRefWithFallback(RE::hknpWorld* world)
    {
        if (auto* filter = getQueryFilterRef(world)) {
            return filter;
        }

        static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
        return *filterSingleton;
    }

    RE::NiCollisionObject* getCollisionObjectFromBody(const RE::hknpBody* body)
    {
        if (!body) {
            return nullptr;
        }

        const auto* bodyBytes = reinterpret_cast<const char*>(body);
        return *reinterpret_cast<RE::NiCollisionObject* const*>(bodyBytes + offsets::kBody_CollisionObjectBackPointer);
    }

    RE::NiCollisionObject* getCollisionObjectFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        return getCollisionObjectFromBody(getBody(world, bodyId));
    }

    bool setCollisionObjectForBody(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiCollisionObject* collisionObject)
    {
        auto* body = getBody(world, bodyId);
        if (!body) {
            return false;
        }

        body->userData = reinterpret_cast<std::uintptr_t>(collisionObject);
        return true;
    }

    RE::NiAVObject* getOwnerNodeFromCollisionObject(const RE::NiCollisionObject* collisionObject)
    {
        if (!collisionObject) {
            return nullptr;
        }

        return collisionObject->sceneObject;
    }

    RE::NiAVObject* getOwnerNodeFromBody(const RE::hknpBody* body)
    {
        return getOwnerNodeFromCollisionObject(getCollisionObjectFromBody(body));
    }

    RE::NiAVObject* getOwnerNodeFromBody(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        return getOwnerNodeFromCollisionObject(getCollisionObjectFromBody(world, bodyId));
    }

    RE::NiTransform bodyArrayWorldTransform(const RE::hknpBody& body)
    {
        RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
        const auto* bodyFloats = reinterpret_cast<const float*>(&body);
        result.rotate = havokRotationBlocksToNiMatrix(bodyFloats);
        const float scale = physics_scale::havokToGame();
        result.translate.x = bodyFloats[12] * scale;
        result.translate.y = bodyFloats[13] * scale;
        result.translate.z = bodyFloats[14] * scale;
        result.scale = 1.0f;
        return result;
    }

    bool tryGetBodyArrayWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
    {
        outTransform = transform_math::makeIdentityTransform<RE::NiTransform>();

        auto* body = getBody(world, bodyId);
        if (!body) {
            return false;
        }

        outTransform = bodyArrayWorldTransform(*body);
        return true;
    }

    bool tryGetBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
    {
        return tryGetBodyArrayWorldTransform(world, bodyId, outTransform);
    }

    bool tryGetMotionWorldTransform(RE::hknpWorld* world, const RE::hknpBody& body, RE::NiTransform& outTransform)
    {
        outTransform = transform_math::makeIdentityTransform<RE::NiTransform>();

        auto* motion = getMotion(world, body.motionIndex);
        if (!motion) {
            return false;
        }

        const float quaternion[4]{ motion->orientation.x, motion->orientation.y, motion->orientation.z, motion->orientation.w };
        outTransform.rotate = transform_math::havokQuaternionToNiRows<RE::NiMatrix3>(quaternion);
        outTransform.translate = hkVectorToNiPoint(motion->position);
        outTransform.scale = 1.0f;
        return true;
    }

    ResolvedBodyWorldTransform resolveLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        ResolvedBodyWorldTransform result{};
        result.transform = transform_math::makeIdentityTransform<RE::NiTransform>();

        auto* body = getBody(world, bodyId);
        if (!body) {
            return result;
        }

        result.motionIndex = body->motionIndex;
        const RE::NiTransform bodyTransform = bodyArrayWorldTransform(*body);

        RE::NiTransform motionTransform{};
        const bool hasMotionTransform = tryGetMotionWorldTransform(world, *body, motionTransform);
        result.source = body_frame::chooseLiveBodyFrameSource(true, hasMotionTransform);
        result.transform = result.source == body_frame::BodyFrameSource::MotionCenterOfMass ? motionTransform : bodyTransform;
        result.valid = result.source != body_frame::BodyFrameSource::Fallback;
        return result;
    }

    bool tryResolveLiveBodyWorldTransform(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform,
        body_frame::BodyFrameSource* outSource,
        std::uint32_t* outMotionIndex)
    {
        const auto resolved = resolveLiveBodyWorldTransform(world, bodyId);
        outTransform = resolved.transform;
        if (outSource) {
            *outSource = resolved.source;
        }
        if (outMotionIndex) {
            *outMotionIndex = resolved.motionIndex;
        }
        return resolved.valid;
    }

    bool getBodyCOMWorld(RE::hknpWorld* world, RE::hknpBodyId bodyId, float& outX, float& outY, float& outZ)
    {
        auto* motion = getBodyMotion(world, bodyId);
        if (!motion) {
            return false;
        }

        outX = motion->position.x;
        outY = motion->position.y;
        outZ = motion->position.z;
        return true;
    }

    bool tryReadFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t& outFilterInfo)
    {
        auto* body = getBody(world, bodyId);
        if (!body) {
            return false;
        }

        outFilterInfo = body->collisionFilterInfo;
        return true;
    }

    bool setFilterInfo(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint32_t filterInfo, std::uint32_t rebuildMode)
    {
        auto* body = getReadableBodySlot(world, bodyId);
        if (!body) {
            return false;
        }
        (void)body;

        using SetCollisionFilter_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
        static REL::Relocation<SetCollisionFilter_t> setBodyCollisionFilterInfo{ REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };
        setBodyCollisionFilterInfo(world, bodyId.value, filterInfo, rebuildMode);
        return true;
    }
    bool setBodyVelocityDeferred(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        const RE::hkVector4f& linearVelocity,
        const RE::hkVector4f& angularVelocity)
    {
        auto* body = getBody(world, RE::hknpBodyId{ bodyId });
        if (!body || !getMotion(world, body->motionIndex)) {
            return false;
        }

        alignas(16) float linear[4] = { linearVelocity.x, linearVelocity.y, linearVelocity.z, linearVelocity.w };
        alignas(16) float angular[4] = { angularVelocity.x, angularVelocity.y, angularVelocity.z, angularVelocity.w };

        using SetVelocityDeferred_t = void (*)(void*, std::uint32_t, const float*, const float*);
        static REL::Relocation<SetVelocityDeferred_t> setVelocityDeferred{ REL::Offset(offsets::kFunc_SetBodyVelocityDeferred) };
        setVelocityDeferred(world, bodyId, linear, angular);
        return true;
    }

    bool setBodyTransformDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiTransform& transform, int mode)
    {
        auto* body = getReadableBodySlot(world, RE::hknpBodyId{ bodyId });
        if (!body) {
            return false;
        }
        (void)body;

        RE::hkTransformf hkTransform;
        hkTransform.rotation = transform_math::niRowsToHavokColumns(transform.rotate);
        const float scale = physics_scale::gameToHavok();
        hkTransform.translation = RE::NiPoint4(transform.translate.x * scale, transform.translate.y * scale, transform.translate.z * scale, 0.0f);

        using SetTransformDeferred_t = void (*)(void*, std::uint32_t, const float*, int);
        static REL::Relocation<SetTransformDeferred_t> setBodyTransform{ REL::Offset(offsets::kFunc_SetBodyTransformDeferred) };
        setBodyTransform(world, bodyId, reinterpret_cast<const float*>(&hkTransform), mode);
        return true;
    }

    bool activateBody(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        auto* body = getReadableBodySlot(world, RE::hknpBodyId{ bodyId });
        if (!body) {
            return false;
        }

        using ActivateBody_t = void (*)(void*, std::uint32_t);
        static REL::Relocation<ActivateBody_t> activate{ REL::Offset(offsets::kFunc_ActivateBody) };
        activate(world, bodyId);
        return true;
    }

    bool enableBodyFlags(RE::hknpWorld* world, std::uint32_t bodyId, std::uint32_t flags, std::uint32_t mode)
    {
        auto* body = getReadableBodySlot(world, RE::hknpBodyId{ bodyId });
        if (!body) {
            return false;
        }
        (void)body;

        using EnableBodyFlags_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
        static REL::Relocation<EnableBodyFlags_t> enableFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
        enableFlags(world, bodyId, flags, mode);
        return true;
    }

    bool disableBodyFlags(RE::hknpWorld* world, std::uint32_t bodyId, std::uint32_t flags, std::uint32_t mode)
    {
        auto* body = getReadableBodySlot(world, RE::hknpBodyId{ bodyId });
        if (!body) {
            return false;
        }
        (void)body;

        using DisableBodyFlags_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
        static REL::Relocation<DisableBodyFlags_t> disableFlags{ REL::Offset(offsets::kFunc_DisableBodyFlags) };
        disableFlags(world, bodyId, flags, mode);
        return true;
    }

    bool applyLinearVelocityDeltaDeferred(RE::hknpWorld* world, std::uint32_t bodyId, const RE::NiPoint3& velocityDeltaHavok)
    {
        auto* motion = getBodyMotion(world, RE::hknpBodyId{ bodyId });
        if (!motion) {
            return false;
        }

        RE::hkVector4f linearVelocity{
            motion->linearVelocity.x + velocityDeltaHavok.x,
            motion->linearVelocity.y + velocityDeltaHavok.y,
            motion->linearVelocity.z + velocityDeltaHavok.z,
            0.0f
        };
        RE::hkVector4f angularVelocity{
            motion->angularVelocity.x,
            motion->angularVelocity.y,
            motion->angularVelocity.z,
            0.0f
        };

        return setBodyVelocityDeferred(world, bodyId, linearVelocity, angularVelocity);
    }

    bool rebuildMotionMassProperties(RE::hknpWorld* world, std::uint32_t motionIndex, int rebuildMode)
    {
        if (!world || !body_frame::hasUsableMotionIndex(motionIndex)) {
            return false;
        }

        using RebuildMotionMassProperties_t = void (*)(void*, std::uint32_t, int);
        static REL::Relocation<RebuildMotionMassProperties_t> rebuildMotionMassProperties{ REL::Offset(offsets::kFunc_RebuildMotionMassProperties) };
        rebuildMotionMassProperties(world, motionIndex, rebuildMode);
        return true;
    }

    bool selectContactSignalPoint(const ContactSignalPointSelectionInput& input, ContactSignalPointResult& outResult)
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

    bool tryExtractContactSignalPoint(RE::hknpWorld* world, const void* contactSignalData, ContactSignalPointResult& outResult)
    {
        outResult = {};
        if (!world || !contactSignalData) {
            return false;
        }

        /*
         * FO4VR already decodes the hknp contact-signal manifold for its own
         * collision listener before applying gameplay effects. Calling the same
         * helper keeps ROCK's RawPoint provider data aligned with Bethesda's
         * hknp path instead of duplicating an internal manifold layout that can
         * differ between single-point and multi-point contacts.
         */
        using ExtractContactSignalPoints_t = void (*)(const void*, RE::hknpWorld*, NativeContactSignalPointBuffer*);
        static REL::Relocation<ExtractContactSignalPoints_t> extractContactSignalPoints{ REL::Offset(offsets::kFunc_ExtractContactSignalPoints) };

        NativeContactSignalPointBuffer native{};
        extractContactSignalPoints(contactSignalData, world, &native);

        ContactSignalPointSelectionInput selection{};
        selection.pointCount = native.pointCount;

        const auto* signalBytes = static_cast<const std::uint8_t*>(contactSignalData);
        selection.contactIndex = signalBytes[0x18];
        const auto* pointWeights = reinterpret_cast<const float*>(signalBytes + 0x30);
        for (std::uint32_t i = 0; i < kMaxContactSignalPoints; ++i) {
            selection.pointWeights[i] = pointWeights[i];
            copyVector4(native.contactPointsHavok[i], selection.contactPointsHavok[i]);
        }
        copyVector4(native.contactNormalHavok, selection.contactNormalHavok);

        return selectContactSignalPoint(selection, outResult);
    }

    void* allocateHavok(std::size_t size)
    {
        static REL::Relocation<std::uint32_t*> tlsIndex{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
        LPVOID tlsBlock = TlsGetValue(*tlsIndex);
        if (!tlsBlock) {
            return nullptr;
        }

        auto** allocator = reinterpret_cast<void***>(reinterpret_cast<char*>(tlsBlock) + 0x58);
        if (!allocator || !*allocator) {
            return nullptr;
        }

        auto* vtable = reinterpret_cast<void* (**)(void*, std::size_t)>(**allocator);
        return vtable[1](*allocator, size);
    }

    void freeHavok(void* ptr, std::size_t size)
    {
        if (!ptr) {
            return;
        }

        static REL::Relocation<std::uint32_t*> tlsIndex{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
        LPVOID tlsBlock = TlsGetValue(*tlsIndex);
        if (!tlsBlock) {
            return;
        }

        auto** allocator = reinterpret_cast<void***>(reinterpret_cast<char*>(tlsBlock) + 0x58);
        if (!allocator || !*allocator) {
            return;
        }

        auto* vtable = reinterpret_cast<void (**)(void*, void*, std::size_t)>(**allocator);
        vtable[2](*allocator, ptr, size);
    }

    bool hkArrayReserveMore(void* arrayBase, int elementSize)
    {
        if (!arrayBase || elementSize <= 0) {
            return false;
        }

        using ReserveMore_t = void (*)(void*, void*, int);
        static REL::Relocation<ReserveMore_t> reserveMore{ REL::Offset(offsets::kFunc_HkArray_ReserveMore) };
        static REL::Relocation<std::uintptr_t> arrayAllocGlobal{ REL::Offset(offsets::kData_HkArrayAllocatorGlobal) };
        reserveMore(reinterpret_cast<void*>(arrayAllocGlobal.address()), arrayBase, elementSize);
        return true;
    }
}

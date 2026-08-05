#include "physics-interaction/native/HavokRuntime.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/NativeMemory.h"
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
#include <cstdint>
#include <cstring>
#include <mutex>
#include <utility>
#include <vector>
#include <windows.h>

namespace rock::havok_runtime
{
    namespace
    {
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

        struct BodyFlagLeaseEntry
        {
            RE::hknpWorld* world = nullptr;
            std::uint32_t bodyId = body_frame::kInvalidBodyId;
            std::uint32_t flags = 0;
            std::uint32_t originalEnabledFlags = 0;
            std::uint32_t mode = 0;
            std::vector<std::uintptr_t> ownerTokens;
        };

        std::mutex g_bodyFlagLeaseMutex;
        std::vector<BodyFlagLeaseEntry> g_bodyFlagLeases;

        bool bodyFlagLeaseMatches(
            const BodyFlagLeaseEntry& lease,
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            std::uint32_t flags,
            std::uint32_t mode)
        {
            return lease.world == world && lease.bodyId == bodyId && lease.flags == flags && lease.mode == mode;
        }

        bool bodyFlagLeaseContainsOwner(const BodyFlagLeaseEntry& lease, std::uintptr_t ownerToken)
        {
            return std::find(lease.ownerTokens.begin(), lease.ownerTokens.end(), ownerToken) != lease.ownerTokens.end();
        }

        RE::NiPoint3 hkVectorToNiPoint(const RE::hkVector4f& value)
        {
            const float scale = physics_scale::havokToGame();
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        RE::NiMatrix3 havokRotationBlocksToNiMatrix(const float* bodyFloats)
        {
            return transform_math::hknpBodyColumnsToNiStoredAxes<RE::NiMatrix3>(bodyFloats);
        }

        bool tryReadBodyHighWaterMarkRaw(const void* world, std::uint32_t* outHighWaterMark)
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
            return native_memory::pointerLooksReadable(ptr);
        }

        bool pointerRangeLooksReadable(const void* ptr, std::size_t byteCount)
        {
            return native_memory::pointerRangeLooksReadable(ptr, byteCount);
        }

        /*
         * Object-tree scans walk native collision wrappers that may contain
         * stale or transitional Havok pointers while a weapon is being pulled.
         * VirtualQuery rejects obviously bad pages, but the real contract at
         * this boundary is "do not let a native field read crash the caller."
         * Centralizing guarded reads keeps high-level grab code out of SEH and
         * makes every physics-system pointer/body-id read fail closed.
         */
        template <class T>
        bool tryReadValue(const T* address, T& out)
        {
            return native_memory::tryReadValue(address, out);
        }

        template <class T>
        bool tryReadField(const void* base, std::uintptr_t offset, T& out)
        {
            return native_memory::tryReadField(base, static_cast<std::ptrdiff_t>(offset), out);
        }

        float decodePackedVelocityCap(std::uint16_t packedValue)
        {
            const std::uint32_t bits = static_cast<std::uint32_t>(packedValue) << 16;
            float value = 0.0f;
            std::memcpy(&value, &bits, sizeof(value));
            return value;
        }
    }

    bool bodySlotLooksReadable(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        if (!world) {
            return false;
        }

        std::uint32_t highWaterMark = 0;
        if (!tryReadBodyHighWaterMarkRaw(world, &highWaterMark)) {
            return false;
        }

        return bodySlotCanBeRead(bodyId.value, highWaterMark);
    }

    bool tryReadBodyHighWaterMark(RE::hknpWorld* world, std::uint32_t& outHighWaterMark)
    {
        outHighWaterMark = 0;
        return tryReadBodyHighWaterMarkRaw(world, &outHighWaterMark) && outHighWaterMark <= body_frame::kMaxReadableBodyIndex;
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

        if (!pointerRangeLooksReadable(collisionObject, offsets::kCollisionObject_PhysSystemPtr + sizeof(void*))) {
            return nullptr;
        }

        void* field = nullptr;
        if (!tryReadField(collisionObject, offsets::kCollisionObject_PhysSystemPtr, field)) {
            return nullptr;
        }
        if (!pointerRangeLooksReadable(field, sizeof(RE::bhkPhysicsSystem))) {
            return nullptr;
        }

        return reinterpret_cast<RE::bhkPhysicsSystem*>(field);
    }

    void* getPhysicsSystemInstance(RE::bhkPhysicsSystem* physicsSystem)
    {
        if (!pointerRangeLooksReadable(physicsSystem, sizeof(RE::bhkPhysicsSystem))) {
            return nullptr;
        }

        RE::hknpPhysicsSystemInstance* instance = nullptr;
        if (!tryReadValue(&physicsSystem->instance, instance)) {
            return nullptr;
        }
        return pointerRangeLooksReadable(instance, sizeof(RE::hknpPhysicsSystemInstance)) ? instance : nullptr;
    }

    const char* physicsSystemBodyScanStatusName(PhysicsSystemBodyScanStatus status)
    {
        switch (status) {
        case PhysicsSystemBodyScanStatus::Enumerated:
            return "enumerated";
        case PhysicsSystemBodyScanStatus::InvalidArguments:
            return "invalid-arguments";
        case PhysicsSystemBodyScanStatus::MissingPhysicsSystem:
            return "missing-physics-system";
        case PhysicsSystemBodyScanStatus::MissingInstance:
            return "missing-instance";
        case PhysicsSystemBodyScanStatus::WorldMismatch:
            return "world-mismatch";
        case PhysicsSystemBodyScanStatus::InvalidBodyCount:
            return "invalid-body-count";
        case PhysicsSystemBodyScanStatus::MissingBodyIds:
            return "missing-body-ids";
        case PhysicsSystemBodyScanStatus::UnreadableBodyIds:
            return "unreadable-body-ids";
        case PhysicsSystemBodyScanStatus::NoBodies:
            return "no-bodies";
        case PhysicsSystemBodyScanStatus::VisitorStopped:
            return "visitor-stopped";
        }
        return "unknown";
    }

    PhysicsSystemBodyScanResult forEachPhysicsSystemBodyIdDetailed(
        RE::NiCollisionObject* collisionObject,
        RE::hknpWorld* expectedWorld,
        std::uint32_t maxBodies,
        bool (*visitor)(std::uint32_t bodyId, void* userData),
        void* userData)
    {
        PhysicsSystemBodyScanResult result{};
        if (!visitor || maxBodies == 0) {
            result.status = PhysicsSystemBodyScanStatus::InvalidArguments;
            return result;
        }

        auto* physicsSystem = getPhysicsSystemFromCollisionObject(collisionObject);
        if (!physicsSystem) {
            result.status = PhysicsSystemBodyScanStatus::MissingPhysicsSystem;
            return result;
        }

        auto* instance = static_cast<RE::hknpPhysicsSystemInstance*>(getPhysicsSystemInstance(physicsSystem));
        if (!instance) {
            result.status = PhysicsSystemBodyScanStatus::MissingInstance;
            return result;
        }

        RE::hknpWorld* instanceWorld = nullptr;
        std::uint32_t* bodyIds = nullptr;
        std::int32_t bodyCount = 0;
        if (!tryReadValue(&instance->world, instanceWorld) ||
            !tryReadValue(&instance->bodyIds, bodyIds) ||
            !tryReadValue(&instance->bodyCount, bodyCount)) {
            result.status = PhysicsSystemBodyScanStatus::MissingInstance;
            return result;
        }

        if (expectedWorld && instanceWorld != expectedWorld) {
            result.status = PhysicsSystemBodyScanStatus::WorldMismatch;
            result.bodyCount = bodyCount;
            return result;
        }

        result.bodyCount = bodyCount;
        constexpr std::int32_t kMaxReasonablePhysicsSystemBodies = 4096;
        if (bodyCount < 0 || bodyCount > kMaxReasonablePhysicsSystemBodies) {
            result.status = PhysicsSystemBodyScanStatus::InvalidBodyCount;
            return result;
        }

        if (bodyCount == 0) {
            result.status = PhysicsSystemBodyScanStatus::NoBodies;
            return result;
        }

        if (!bodyIds) {
            result.status = PhysicsSystemBodyScanStatus::MissingBodyIds;
            return result;
        }

        const std::int32_t count = (std::min)(bodyCount, static_cast<std::int32_t>(maxBodies));
        if (!pointerRangeLooksReadable(bodyIds, sizeof(std::uint32_t) * static_cast<std::size_t>(count))) {
            result.status = PhysicsSystemBodyScanStatus::UnreadableBodyIds;
            return result;
        }

        for (std::int32_t i = 0; i < count; ++i) {
            std::uint32_t bodyId = body_frame::kInvalidBodyId;
            if (!tryReadValue(bodyIds + i, bodyId)) {
                result.status = PhysicsSystemBodyScanStatus::UnreadableBodyIds;
                return result;
            }
            if (bodyId == body_frame::kInvalidBodyId || bodyId > 0x000F'FFFF) {
                ++result.skippedInvalidBodies;
                continue;
            }
            ++result.visitedBodies;
            if (!visitor(bodyId, userData)) {
                result.status = PhysicsSystemBodyScanStatus::VisitorStopped;
                return result;
            }
        }

        result.status = PhysicsSystemBodyScanStatus::Enumerated;
        return result;
    }

    bool forEachPhysicsSystemBodyId(
        RE::NiCollisionObject* collisionObject,
        RE::hknpWorld* expectedWorld,
        std::uint32_t maxBodies,
        bool (*visitor)(std::uint32_t bodyId, void* userData),
        void* userData)
    {
        return forEachPhysicsSystemBodyIdDetailed(collisionObject, expectedWorld, maxBodies, visitor, userData).enumerated();
    }

    void* getQueryFilterRefWithFallback(RE::hknpWorld* world)
    {
        if (auto* filter = getQueryFilterRef(world)) {
            return filter;
        }

        static REL::Relocation<void**> filterSingleton{ REL::Offset(offsets::kData_CollisionFilterSingleton) };
        return *filterSingleton;
    }

    std::uint64_t* getCollisionFilterMatrix(void* filter)
    {
        /*
         * The filter matrix is a native hknp filter layout field. Domain code
         * can own ROCK layer policy, but the offset that finds the matrix stays
         * in this runtime boundary.
         */
        if (!filter) {
            return nullptr;
        }

        return reinterpret_cast<std::uint64_t*>(reinterpret_cast<std::uintptr_t>(filter) + offsets::kFilter_CollisionMatrix);
    }

    std::uint64_t* getCollisionFilterMatrix(RE::hknpWorld* world, bool* outUsedFallback)
    {
        if (outUsedFallback) {
            *outUsedFallback = false;
        }

        if (auto* filter = getQueryFilterRef(world)) {
            return getCollisionFilterMatrix(filter);
        }

        if (outUsedFallback) {
            *outUsedFallback = true;
        }
        return getCollisionFilterMatrix(getQueryFilterRefWithFallback(world));
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

        RE::NiAVObject* sceneObject = nullptr;
        if (!tryReadValue(&collisionObject->sceneObject, sceneObject)) {
            return nullptr;
        }
        return pointerLooksReadable(sceneObject) ? sceneObject : nullptr;
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

    bool tryReadMotionPropertiesId(const RE::hknpMotion* motion, std::uint16_t& outMotionPropertiesId)
    {
        outMotionPropertiesId = 0;
        return tryReadField(motion, offsets::kMotion_PropertiesId, outMotionPropertiesId);
    }

    bool tryReadBodyMotionPropertiesId(RE::hknpWorld* world, RE::hknpBodyId bodyId, std::uint16_t& outMotionPropertiesId)
    {
        outMotionPropertiesId = 0;
        if (auto* motion = getBodyMotion(world, bodyId)) {
            return tryReadMotionPropertiesId(motion, outMotionPropertiesId);
        }
        return false;
    }

    bool tryReadMotionVelocityCaps(const RE::hknpMotion* motion, MotionVelocityCaps& outCaps)
    {
        outCaps = {};
        if (!motion) {
            return false;
        }

        std::uint16_t maxLinearVelocityPacked = 0;
        std::uint16_t maxAngularVelocityPacked = 0;
        if (!tryReadField(motion, offsets::kMotion_MaxLinearVelocityPacked, maxLinearVelocityPacked) ||
            !tryReadField(motion, offsets::kMotion_MaxAngularVelocityPacked, maxAngularVelocityPacked)) {
            return false;
        }

        outCaps.valid = true;
        outCaps.maxLinearVelocityPacked = maxLinearVelocityPacked;
        outCaps.maxAngularVelocityPacked = maxAngularVelocityPacked;
        outCaps.maxLinearVelocity = decodePackedVelocityCap(maxLinearVelocityPacked);
        outCaps.maxAngularVelocity = decodePackedVelocityCap(maxAngularVelocityPacked);
        return true;
    }

    bool snapshotMotionPropertiesLibrary(RE::hknpWorld* world, MotionPropertiesLibrarySnapshot& outSnapshot)
    {
        outSnapshot = {};
        if (!world) {
            return false;
        }

        char* library = nullptr;
        if (!tryReadField(world, offsets::kHknpWorld_MotionPropertiesLibraryPtr, library) || !library) {
            return false;
        }

        char* entries = nullptr;
        std::int32_t count = 0;
        if (!tryReadField(library, offsets::kMotionPropertiesLibrary_Entries, entries) ||
            !tryReadField(library, offsets::kMotionPropertiesLibrary_Count, count) ||
            count < 0) {
            return false;
        }

        outSnapshot.valid = true;
        outSnapshot.count = static_cast<std::uint32_t>(count);
        if (!entries || count == 0) {
            return true;
        }

        outSnapshot.copiedCount = (std::min)(outSnapshot.count, kMaxMotionPropertiesSnapshotRecords);
        for (std::uint32_t i = 0; i < outSnapshot.copiedCount; ++i) {
            const auto* entry = entries + (static_cast<std::uintptr_t>(i) * offsets::kMotionProperties_RecordSize);
            for (std::uint32_t wordIndex = 0; wordIndex < 16; ++wordIndex) {
                const auto fieldOffset = static_cast<std::uintptr_t>(wordIndex) * sizeof(std::uint32_t);
                (void)tryReadField(entry, fieldOffset, outSnapshot.records[i].values[wordIndex]);
                (void)tryReadField(entry, fieldOffset, outSnapshot.records[i].words[wordIndex]);
            }
        }

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
        hkTransform.rotation = transform_math::niStoredAxesToHknpBodyColumns(transform.rotate);
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

    bool acquireBodyFlagLease(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint32_t flags,
        std::uint32_t mode,
        std::uintptr_t ownerToken)
    {
        /*
         * Body flags are a shared Havok resource once both hands can attach to
         * the same dynamic object. Raw enable/disable calls make the first
         * release able to undo flags that the peer hand still depends on. Keep
         * a small owner-token lease table at the Havok runtime boundary so
         * higher-level grab code can release its own lease while the final
         * restore remains tied to the last live owner.
         */
        if (!world || flags == 0 || ownerToken == 0) {
            return false;
        }

        std::scoped_lock lock(g_bodyFlagLeaseMutex);
        auto leaseIt = std::find_if(g_bodyFlagLeases.begin(), g_bodyFlagLeases.end(), [&](const BodyFlagLeaseEntry& lease) {
            return bodyFlagLeaseMatches(lease, world, bodyId, flags, mode);
        });
        if (leaseIt != g_bodyFlagLeases.end()) {
            if (!bodyFlagLeaseContainsOwner(*leaseIt, ownerToken)) {
                leaseIt->ownerTokens.push_back(ownerToken);
            }
            return true;
        }

        auto* body = getReadableBodySlot(world, RE::hknpBodyId{ bodyId });
        if (!body) {
            return false;
        }

        const std::uint32_t originalEnabledFlags = body->flags & flags;
        if (!enableBodyFlags(world, bodyId, flags, mode)) {
            return false;
        }

        BodyFlagLeaseEntry lease{};
        lease.world = world;
        lease.bodyId = bodyId;
        lease.flags = flags;
        lease.originalEnabledFlags = originalEnabledFlags;
        lease.mode = mode;
        lease.ownerTokens.push_back(ownerToken);
        g_bodyFlagLeases.push_back(std::move(lease));
        return true;
    }

    bool releaseBodyFlagLease(
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint32_t flags,
        std::uint32_t mode,
        std::uintptr_t ownerToken,
        bool restoreOnFinalLease)
    {
        if (flags == 0 || ownerToken == 0) {
            return false;
        }

        std::scoped_lock lock(g_bodyFlagLeaseMutex);
        auto leaseIt = std::find_if(g_bodyFlagLeases.begin(), g_bodyFlagLeases.end(), [&](const BodyFlagLeaseEntry& lease) {
            if (world) {
                return bodyFlagLeaseMatches(lease, world, bodyId, flags, mode);
            }
            return lease.bodyId == bodyId && lease.flags == flags && lease.mode == mode && bodyFlagLeaseContainsOwner(lease, ownerToken);
        });
        if (leaseIt == g_bodyFlagLeases.end()) {
            return false;
        }

        auto ownerIt = std::find(leaseIt->ownerTokens.begin(), leaseIt->ownerTokens.end(), ownerToken);
        if (ownerIt == leaseIt->ownerTokens.end()) {
            return false;
        }

        leaseIt->ownerTokens.erase(ownerIt);
        if (!leaseIt->ownerTokens.empty()) {
            return true;
        }

        const std::uint32_t flagsIntroducedByLease = leaseIt->flags & ~leaseIt->originalEnabledFlags;
        const bool shouldRestore = restoreOnFinalLease && world && flagsIntroducedByLease != 0;
        g_bodyFlagLeases.erase(leaseIt);
        return shouldRestore ? disableBodyFlags(world, bodyId, flagsIntroducedByLease, mode) : true;
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

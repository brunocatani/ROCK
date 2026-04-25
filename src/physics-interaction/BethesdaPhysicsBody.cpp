#include "BethesdaPhysicsBody.h"

#include "HavokOffsets.h"
#include "PhysicsLog.h"

#include <windows.h>

namespace frik::rock
{

    static void* havokAlloc(std::size_t size)
    {
        static REL::Relocation<std::uint32_t*> s_tlsIndex{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
        LPVOID tlsBlock = TlsGetValue(*s_tlsIndex);
        if (!tlsBlock)
            return nullptr;

        auto** allocator = reinterpret_cast<void***>(reinterpret_cast<char*>(tlsBlock) + 0x58);
        if (!allocator || !*allocator)
            return nullptr;

        auto* vtable = reinterpret_cast<void* (**)(void*, std::size_t)>(**allocator);
        return vtable[1](*allocator, size);
    }

    static void havokFree(void* ptr, std::size_t size)
    {
        if (!ptr)
            return;

        static REL::Relocation<std::uint32_t*> s_tlsIndex{ REL::Offset(offsets::kData_HavokTlsAllocKey) };
        LPVOID tlsBlock = TlsGetValue(*s_tlsIndex);
        if (!tlsBlock)
            return;

        auto** allocator = reinterpret_cast<void***>(reinterpret_cast<char*>(tlsBlock) + 0x58);
        if (!allocator || !*allocator)
            return;

        auto* vtable = reinterpret_cast<void (**)(void*, void*, std::size_t)>(**allocator);
        vtable[2](*allocator, ptr, size);
    }

    static void releaseRefCounted(void* obj)
    {
        if (!obj)
            return;

        auto* refCountDword = reinterpret_cast<volatile long*>(reinterpret_cast<char*>(obj) + 0x08);

        for (;;) {
            long oldVal = *refCountDword;
            std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
            if (rc == 0 || rc == 0xFFFF)
                return;

            long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(rc - 1));

            if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal) {
                if (rc - 1 == 0) {
                    auto** vtable = *reinterpret_cast<void***>(obj);
                    auto destructor = reinterpret_cast<void (*)(void*, int)>(vtable[0]);
                    destructor(obj, 1);
                }
                return;
            }
        }
    }

    static void* bethesdaAlloc(std::size_t size)
    {
        typedef void* (*alloc_t)(void*, std::size_t, int);
        static REL::Relocation<alloc_t> allocFunc{ REL::Offset(offsets::kFunc_BethesdaAlloc) };
        static REL::Relocation<std::uintptr_t> allocPool{ REL::Offset(offsets::kData_BethesdaAllocatorPool) };

        return allocFunc(reinterpret_cast<void*>(allocPool.address()), size, 0);
    }

    using PhysicsSystemDataCtor_t = void* (*)(void*);
    using BodyCinfoCtor_t = void* (*)(void*);
    using MaterialCtor_t = void* (*)(void*);
    using PhysicsSystemCtor_t = void* (*)(void*, void*);
    using CollisionObjectCtor_t = void* (*)(void*, std::uint32_t, void*);
    using CreateInstance_t = void (*)(void*, void*);
    using SetMotionType_t = void (*)(void*, int);
    using LinkObject_t = void (*)(void*, void*);
    using GetBodyId_t = void (*)(void*, void*, std::uint32_t);

    using DriveToKeyFrame_t = bool (*)(void*, const void*, float);
    using SetTransform_t = void (*)(void*, const void*);
    using SetVelocity_t = void (*)(void*, const float*, const float*);
    using ApplyImpulse_t = void (*)(void*, const float*);
    using ApplyPointImpulse_t = void (*)(void*, const float*, const float*);
    using SetMass_t = void (*)(void*, float);
    using GetCOM_t = bool (*)(void*, float*);
    using GetFilterInfo_t = std::uint32_t (*)(void*);
    using GetShape_t = void* (*)(void*);
    using IsConstrained_t = bool (*)(void*);

    using SetCollisionFilter_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
    using EnableBodyFlags_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
    using ActivateBody_t = void (*)(void*, std::uint32_t);

    using HkArrayReserveMore_t = void (*)(void*, void*, int);

    bool BethesdaPhysicsBody::create(RE::hknpWorld* world, void* bhkWorld, RE::hknpShape* shape, std::uint32_t filterInfo, RE::hknpMaterialId materialId,
        BethesdaMotionType motionType, const char* name)
    {
        if (_created) {
            ROCK_LOG_WARN(BethesdaBody, "create() called on already-created body — destroy first");
            return false;
        }
        if (!world || !bhkWorld || !shape) {
            ROCK_LOG_ERROR(BethesdaBody, "create() null params: world={} bhkWorld={} shape={}", (void*)world, bhkWorld, (void*)shape);
            return false;
        }

        _systemData = havokAlloc(0x78);
        if (!_systemData) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate hknpPhysicsSystemData (0x78 bytes)");
            return false;
        }
        {
            static REL::Relocation<PhysicsSystemDataCtor_t> ctor{ REL::Offset(offsets::kFunc_PhysicsSystemData_Ctor) };
            ctor(_systemData);
        }

        auto hkArrayAppendOne = [&](char* arrayBase, int stride) -> char* {
            auto*& dataPtr = *reinterpret_cast<char**>(arrayBase);
            auto& size = *reinterpret_cast<std::int32_t*>(arrayBase + 0x08);
            auto& capFlags = *reinterpret_cast<std::int32_t*>(arrayBase + 0x0C);
            std::int32_t capacity = capFlags & 0x3FFFFFFF;

            if (size >= capacity) {
                using ReserveMore_t = void (*)(void*, void*, int);
                static REL::Relocation<ReserveMore_t> reserveMore{ REL::Offset(offsets::kFunc_HkArray_ReserveMore) };
                static REL::Relocation<std::uintptr_t> arrayAllocGlobal{ REL::Offset(offsets::kData_HkArrayAllocatorGlobal) };
                auto* allocParam = reinterpret_cast<void*>(arrayAllocGlobal.address());
                reserveMore(allocParam, arrayBase, stride);
            }

            if (!dataPtr)
                return nullptr;

            char* newEntry = dataPtr + size * stride;
            size++;
            return newEntry;
        };

        void* bodyCinfo = nullptr;
        {
            auto* bodyCinfoArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_BodyCinfos;
            bodyCinfo = hkArrayAppendOne(bodyCinfoArray, 0x60);
            if (!bodyCinfo) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to grow bodyCinfos array");

                havokFree(_systemData, 0x78);
                _systemData = nullptr;
                return false;
            }

            static REL::Relocation<BodyCinfoCtor_t> cinfoInit{ REL::Offset(offsets::kFunc_BodyCinfo_Ctor) };
            cinfoInit(bodyCinfo);

            auto* ci = reinterpret_cast<char*>(bodyCinfo);
            *reinterpret_cast<RE::hknpShape**>(ci + 0x00) = shape;
            *reinterpret_cast<std::uint32_t*>(ci + 0x08) = 0x7FFF'FFFF;
            *reinterpret_cast<std::uint32_t*>(ci + 0x0C) = 0x7FFF'FFFF;
            *reinterpret_cast<std::uint8_t*>(ci + 0x10) = 0xFF;
            *reinterpret_cast<std::uint16_t*>(ci + 0x12) = materialId.value;
            *reinterpret_cast<std::uint32_t*>(ci + 0x14) = filterInfo;
        }

        {
            auto* materialsArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_Materials;
            auto* material = hkArrayAppendOne(materialsArray, 0x50);
            if (!material) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to grow materials array");

                havokFree(_systemData, 0x78);
                _systemData = nullptr;
                return false;
            }

            using MaterialCtor_t = void* (*)(void*);
            static REL::Relocation<MaterialCtor_t> materialCtor{ REL::Offset(offsets::kFunc_MaterialCtor) };
            materialCtor(material);
        }

        {
            auto* shapeRefsArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_Shapes;
            auto* shapeSlot = hkArrayAppendOne(shapeRefsArray, 8);
            if (shapeSlot) {
                *reinterpret_cast<RE::hknpShape**>(shapeSlot) = shape;

                auto* refCountDword = reinterpret_cast<volatile long*>(reinterpret_cast<char*>(shape) + 0x08);
                for (;;) {
                    long oldVal = *refCountDword;
                    std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
                    if (rc == 0xFFFF)
                        break;
                    long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(rc + 1));
                    if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal)
                        break;
                }
            }
        }

        _physicsSystem = bethesdaAlloc(0x28);
        if (!_physicsSystem) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate bhkPhysicsSystem (0x28 bytes)");

            havokFree(_systemData, 0x78);
            _systemData = nullptr;
            return false;
        }
        {
            static REL::Relocation<PhysicsSystemCtor_t> physSysCtor{ REL::Offset(offsets::kFunc_PhysicsSystem_Ctor) };
            physSysCtor(_physicsSystem, _systemData);
        }

        _collisionObject = bethesdaAlloc(0x30);
        if (!_collisionObject) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate bhkNPCollisionObject (0x30 bytes)");

            releaseRefCounted(_physicsSystem);
            _physicsSystem = nullptr;
            _systemData = nullptr;
            return false;
        }
        {
            static REL::Relocation<CollisionObjectCtor_t> collObjCtor{ REL::Offset(offsets::kFunc_CollisionObject_Ctor) };
            collObjCtor(_collisionObject, 0, _physicsSystem);
        }

        {
            auto* bodyCinfoArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_BodyCinfos;
            auto* cinfoData = *reinterpret_cast<char**>(bodyCinfoArray);
            if (!cinfoData) {
                ROCK_LOG_ERROR(BethesdaBody, "bodyCinfo array data is null");
                releaseRefCounted(_collisionObject);
                _collisionObject = nullptr;
                _physicsSystem = nullptr;
                _systemData = nullptr;
                return false;
            }

            RE::hknpBodyCinfo cinfo;
            cinfo.shape = *reinterpret_cast<RE::hknpShape**>(cinfoData + 0x00);
            cinfo.collisionFilterInfo = *reinterpret_cast<std::uint32_t*>(cinfoData + 0x14);
            cinfo.materialId.value = *reinterpret_cast<std::uint16_t*>(cinfoData + 0x12);
            cinfo.motionPropertiesId.value = 0xFF;
            cinfo.position = RE::hkVector4f(0.0f, 0.0f, 0.0f, 0.0f);
            cinfo.orientation = RE::hkVector4f(0.0f, 0.0f, 0.0f, 1.0f);
            cinfo.userData = 0;
            cinfo.name = name;

            auto motionId = world->AllocateMotion();
            cinfo.motionId.value = motionId;

            _bodyId = world->CreateBody(cinfo);
            if (_bodyId.value == 0x7FFF'FFFF) {
                ROCK_LOG_ERROR(BethesdaBody, "CreateBody returned invalid ID");
                releaseRefCounted(_collisionObject);
                _collisionObject = nullptr;
                _physicsSystem = nullptr;
                _systemData = nullptr;
                return false;
            }

            ROCK_LOG_INFO(BethesdaBody, "Body created via raw CreateBody: bodyId={} motionId={}", _bodyId.value, motionId);
        }

        {
            constexpr std::size_t INSTANCE_SIZE = 0x30;
            auto* instance = reinterpret_cast<char*>(havokAlloc(INSTANCE_SIZE));
            if (!instance) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate physics system instance (0x30 bytes)");
                world->DestroyBodies(&_bodyId, 1);
                _bodyId.value = 0x7FFF'FFFF;
                releaseRefCounted(_collisionObject);
                _collisionObject = nullptr;
                _physicsSystem = nullptr;
                _systemData = nullptr;
                return false;
            }
            std::memset(instance, 0, INSTANCE_SIZE);

            auto* bodyIdArray = reinterpret_cast<std::uint32_t*>(havokAlloc(sizeof(std::uint32_t)));
            if (!bodyIdArray) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate physics system body ID array");
                havokFree(instance, INSTANCE_SIZE);
                world->DestroyBodies(&_bodyId, 1);
                _bodyId.value = 0x7FFF'FFFF;
                releaseRefCounted(_collisionObject);
                _collisionObject = nullptr;
                _physicsSystem = nullptr;
                _systemData = nullptr;
                return false;
            }
            *bodyIdArray = _bodyId.value;

            *reinterpret_cast<void**>(instance + 0x18) = world;
            *reinterpret_cast<std::uint32_t**>(instance + 0x20) = bodyIdArray;
            *reinterpret_cast<std::int32_t*>(instance + 0x28) = 1;

            *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18) = instance;

            ROCK_LOG_INFO(BethesdaBody, "Physics system instance constructed: instance={:p} world={:p} bodyId={}", (void*)instance, (void*)world, _bodyId.value);
        }

        {
            auto* bodyArray = world->GetBodyArray();
            auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[_bodyId.value]);
            *reinterpret_cast<void**>(bodyPtr + 0x88) = _collisionObject;

            ROCK_LOG_INFO(BethesdaBody, "body+0x88 back-pointer set manually: {:p}", _collisionObject);
        }

        if (motionType == BethesdaMotionType::Keyframed) {
            typedef void (*setKeyframed_t)(void*, std::uint32_t);
            static REL::Relocation<setKeyframed_t> setBodyKeyframed{ REL::Offset(offsets::kFunc_SetBodyKeyframed) };
            setBodyKeyframed(world, _bodyId.value);
        } else {
            static REL::Relocation<SetMotionType_t> setMotion{ REL::Offset(offsets::kFunc_CollisionObject_SetMotionType) };
            setMotion(_collisionObject, static_cast<int>(motionType));
        }

        {
            static REL::Relocation<EnableBodyFlags_t> enableFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
            enableFlags(world, _bodyId.value, 0x08020000, 1);
        }

        {
            static REL::Relocation<ActivateBody_t> activate{ REL::Offset(offsets::kFunc_ActivateBody) };
            activate(world, _bodyId.value);
        }

        _created = true;

        ROCK_LOG_INFO(BethesdaBody, "Created '{}': bodyId={} collObj={:p} physSys={:p} sysData={:p} motionType={}", name, _bodyId.value, _collisionObject, _physicsSystem,
            _systemData, static_cast<int>(motionType));

        {
            auto* bodyArray = world->GetBodyArray();
            auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[_bodyId.value]);
            auto* backPtr = *reinterpret_cast<void**>(bodyPtr + 0x88);
            if (backPtr == _collisionObject) {
                ROCK_LOG_INFO(BethesdaBody, "  body+0x88 back-pointer VERIFIED: {:p} == collisionObject", backPtr);
            } else {
                ROCK_LOG_ERROR(BethesdaBody, "  body+0x88 back-pointer MISMATCH: {:p} != {:p}", backPtr, _collisionObject);
            }
        }

        return true;
    }

    void BethesdaPhysicsBody::destroy(void* bhkWorld)
    {
        (void)bhkWorld;
        if (!_created)
            return;

        ROCK_LOG_INFO(BethesdaBody, "Destroying body: bodyId={} collObj={:p}", _bodyId.value, _collisionObject);

        destroyNiNode();

        if (_physicsSystem) {
            auto* physSysInstance = *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18);
            if (physSysInstance) {
                auto* worldPtr = *reinterpret_cast<RE::hknpWorld**>(physSysInstance + 0x18);
                if (worldPtr && _bodyId.value != 0x7FFF'FFFF) {
                    auto* bodyArray = worldPtr->GetBodyArray();
                    auto* bodyPtr = reinterpret_cast<char*>(&bodyArray[_bodyId.value]);
                    *reinterpret_cast<void**>(bodyPtr + 0x88) = nullptr;
                }
            }
        }

        if (_bodyId.value != 0x7FFF'FFFF && _physicsSystem) {
            auto* physSysInstance = *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18);
            if (physSysInstance) {
                auto* worldPtr = *reinterpret_cast<RE::hknpWorld**>(physSysInstance + 0x18);
                if (worldPtr) {
                    worldPtr->DestroyBodies(&_bodyId, 1);
                    ROCK_LOG_INFO(BethesdaBody, "Body {} destroyed from world", _bodyId.value);
                }
            }
        }

        if (_physicsSystem) {
            auto* instance = *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18);
            if (instance) {
                auto* bodyIdArray = *reinterpret_cast<void**>(instance + 0x20);
                if (bodyIdArray) {
                    havokFree(bodyIdArray, sizeof(std::uint32_t));
                }
                havokFree(instance, 0x30);

                *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18) = nullptr;
            }
        }

        if (_collisionObject) {
            releaseRefCounted(_collisionObject);
        }

        reset();
    }

    void BethesdaPhysicsBody::reset()
    {
        _collisionObject = nullptr;
        _physicsSystem = nullptr;
        _systemData = nullptr;
        _niNode = nullptr;
        _bodyId.value = 0x7FFF'FFFF;
        _created = false;
    }

    bool BethesdaPhysicsBody::createNiNode(const char* name)
    {
        if (!isValid() || !_collisionObject) {
            ROCK_LOG_ERROR(BethesdaBody, "createNiNode: body not valid or no collision object");
            return false;
        }
        if (_niNode) {
            ROCK_LOG_WARN(BethesdaBody, "createNiNode: already has NiNode — skipping");
            return true;
        }

        void* mem = bethesdaAlloc(offsets::kNiNodeSize);
        if (!mem) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate NiNode (0x{:X} bytes)", offsets::kNiNodeSize);
            return false;
        }

        std::memset(mem, 0, offsets::kNiNodeSize);

        {
            using NiNodeCtor_t = void (*)(void*);
            static REL::Relocation<NiNodeCtor_t> niNodeCtor{ REL::Offset(offsets::kFunc_NiNode_Ctor) };
            niNodeCtor(mem);
            _niNode = mem;
        }

        if (name && name[0]) {
            alignas(8) char bsStr[8] = {};
            using BSFixedStringCreate_t = void (*)(void*, const char*);
            static REL::Relocation<BSFixedStringCreate_t> createStr{ REL::Offset(offsets::kFunc_BSFixedString_Create) };
            createStr(bsStr, name);

            using NiNodeSetName_t = void (*)(void*, const void*);
            static REL::Relocation<NiNodeSetName_t> setName{ REL::Offset(offsets::kFunc_NiNode_SetName) };
            setName(_niNode, bsStr);

            std::memset(bsStr, 0, sizeof(bsStr));
        }

        {
            using LinkObject_t = void (*)(void*, void*);
            static REL::Relocation<LinkObject_t> linkObject{ REL::Offset(offsets::kFunc_CollisionObject_LinkObject) };
            linkObject(_collisionObject, _niNode);
        }

        {
            auto* collObjOwner = *reinterpret_cast<void**>(reinterpret_cast<char*>(_collisionObject) + 0x10);
            auto* nodeCollObj = *reinterpret_cast<void**>(reinterpret_cast<char*>(_niNode) + 0x100);

            bool ownerOk = (collObjOwner == _niNode);
            bool collOk = (nodeCollObj == _collisionObject);

            ROCK_LOG_INFO(BethesdaBody, "NiNode '{}' created: node={:p} collObj→owner={} node→collObj={}", name ? name : "(null)", _niNode, ownerOk ? "VERIFIED" : "MISMATCH",
                collOk ? "VERIFIED" : "MISMATCH");
        }

        return true;
    }

    void BethesdaPhysicsBody::destroyNiNode()
    {
        if (!_niNode)
            return;

        if (_collisionObject) {
            *reinterpret_cast<void**>(reinterpret_cast<char*>(_collisionObject) + 0x10) = nullptr;
        }

        *reinterpret_cast<void**>(reinterpret_cast<char*>(_niNode) + 0x100) = nullptr;

        releaseRefCounted(_niNode);
        _niNode = nullptr;

        ROCK_LOG_INFO(BethesdaBody, "NiNode destroyed");
    }

    void BethesdaPhysicsBody::registerContactSignal(const char* signalName)
    {
        if (!isValid())
            return;

        ROCK_LOG_WARN(BethesdaBody,
            "registerContactSignal('{}') NOT YET IMPLEMENTED — "
            "need getEventSignalForBody address",
            signalName ? signalName : "(null)");
    }

    bool BethesdaPhysicsBody::driveToKeyFrame(const RE::NiTransform& target, float dt)
    {
        if (!isValid())
            return false;
        static REL::Relocation<DriveToKeyFrame_t> drive{ REL::Offset(offsets::kFunc_CollisionObject_DriveToKeyFrame) };
        return drive(_collisionObject, &target, dt);
    }

    void BethesdaPhysicsBody::setTransform(const RE::hkTransformf& transform)
    {
        if (!isValid())
            return;
        static REL::Relocation<SetTransform_t> setXform{ REL::Offset(offsets::kFunc_CollisionObject_SetTransform) };
        setXform(_collisionObject, &transform);
    }

    void BethesdaPhysicsBody::setVelocity(const float* linVel, const float* angVel)
    {
        if (!isValid())
            return;
        static REL::Relocation<SetVelocity_t> setVel{ REL::Offset(offsets::kFunc_CollisionObject_SetVelocity) };
        setVel(_collisionObject, linVel, angVel);
    }

    void BethesdaPhysicsBody::setMotionType(BethesdaMotionType type)
    {
        if (!isValid())
            return;
        static REL::Relocation<SetMotionType_t> setMotion{ REL::Offset(offsets::kFunc_CollisionObject_SetMotionType) };
        setMotion(_collisionObject, static_cast<int>(type));
    }

    void BethesdaPhysicsBody::setCollisionFilterInfo(std::uint32_t filterInfo, std::uint32_t rebuildMode)
    {
        if (!isValid())
            return;

        static REL::Relocation<SetCollisionFilter_t> setFilter{ REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

        auto* physSysInst = *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18);
        if (!physSysInst)
            return;
        auto* world = *reinterpret_cast<void**>(physSysInst + 0x18);
        if (!world)
            return;
        setFilter(world, _bodyId.value, filterInfo, rebuildMode);
    }

    void BethesdaPhysicsBody::setMass(float mass)
    {
        if (!isValid())
            return;
        static REL::Relocation<SetMass_t> setM{ REL::Offset(offsets::kFunc_CollisionObject_SetMass) };
        setM(_collisionObject, mass);
    }

    void BethesdaPhysicsBody::applyLinearImpulse(const float* impulse)
    {
        if (!isValid())
            return;
        static REL::Relocation<ApplyImpulse_t> apply{ REL::Offset(offsets::kFunc_CollisionObject_ApplyLinearImpulse) };
        apply(_collisionObject, impulse);
    }

    void BethesdaPhysicsBody::applyPointImpulse(const float* impulse, const float* worldPoint)
    {
        if (!isValid())
            return;
        static REL::Relocation<ApplyPointImpulse_t> apply{ REL::Offset(offsets::kFunc_CollisionObject_ApplyPointImpulse) };
        apply(_collisionObject, impulse, worldPoint);
    }

    bool BethesdaPhysicsBody::getCenterOfMassWorld(float& outX, float& outY, float& outZ)
    {
        if (!isValid())
            return false;
        alignas(16) float com[4] = { 0, 0, 0, 0 };
        static REL::Relocation<GetCOM_t> getCOM{ REL::Offset(offsets::kFunc_CollisionObject_GetCOMWorld) };
        bool ok = getCOM(_collisionObject, com);
        if (ok) {
            outX = com[0];
            outY = com[1];
            outZ = com[2];
        }
        return ok;
    }

    std::uint32_t BethesdaPhysicsBody::getCollisionFilterInfo()
    {
        if (!isValid())
            return 0;
        static REL::Relocation<GetFilterInfo_t> getFilter{ REL::Offset(offsets::kFunc_CollisionObject_GetFilterInfo) };
        return getFilter(_collisionObject);
    }

    void* BethesdaPhysicsBody::getShape()
    {
        if (!isValid())
            return nullptr;
        static REL::Relocation<GetShape_t> getShp{ REL::Offset(offsets::kFunc_CollisionObject_GetShape) };
        return getShp(_collisionObject);
    }

    bool BethesdaPhysicsBody::isConstrained()
    {
        if (!isValid())
            return false;
        static REL::Relocation<IsConstrained_t> check{ REL::Offset(offsets::kFunc_IsBodyConstrained) };
        return check(_collisionObject);
    }

    void BethesdaPhysicsBody::setPointVelocity(const float* targetVel, const float* worldPoint)
    {
        (void)targetVel;
        (void)worldPoint;
        if (!isValid())
            return;

        ROCK_LOG_WARN(BethesdaBody, "setPointVelocity: NOT YET IMPLEMENTED (Phase 4 stub)");
    }

    void BethesdaPhysicsBody::enableBodyFlags(std::uint32_t flags, std::uint32_t mode)
    {
        if (!isValid())
            return;
        auto* physSysInst = *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18);
        if (!physSysInst)
            return;
        auto* world = *reinterpret_cast<void**>(physSysInst + 0x18);
        if (!world)
            return;

        static REL::Relocation<EnableBodyFlags_t> enableFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
        enableFlags(world, _bodyId.value, flags, mode);
    }

    void BethesdaPhysicsBody::activateBody()
    {
        if (!isValid())
            return;
        auto* physSysInst = *reinterpret_cast<char**>(reinterpret_cast<char*>(_physicsSystem) + 0x18);
        if (!physSysInst)
            return;
        auto* world = *reinterpret_cast<void**>(physSysInst + 0x18);
        if (!world)
            return;

        static REL::Relocation<ActivateBody_t> activate{ REL::Offset(offsets::kFunc_ActivateBody) };
        activate(world, _bodyId.value);
    }

}

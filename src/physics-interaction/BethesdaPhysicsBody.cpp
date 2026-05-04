#include "BethesdaPhysicsBody.h"

#include "HavokRefCount.h"
#include "HavokOffsets.h"
#include "HavokRuntime.h"
#include "PhysicsLog.h"

#include "RE/Havok/hknpBody.h"

#include <intrin.h>
#include <windows.h>

namespace frik::rock
{
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

    static void* bethesdaAllocatorPool()
    {
        static REL::Relocation<std::uintptr_t> allocPool{ REL::Offset(offsets::kData_BethesdaAllocatorPool) };
        return reinterpret_cast<void*>(allocPool.address());
    }

    static bool ensureBethesdaAllocatorInitialized()
    {
        using AllocatorInit_t = void* (*)(void*, std::uint32_t*);
        static REL::Relocation<AllocatorInit_t> initFunc{ REL::Offset(offsets::kFunc_BethesdaAllocatorInit) };
        static REL::Relocation<std::uint32_t*> allocatorState{ REL::Offset(offsets::kData_BethesdaAllocatorState) };

        auto* state = allocatorState.get();
        if (!state) {
            return false;
        }

        if (*state != 2) {
            initFunc(bethesdaAllocatorPool(), state);
        }

        return *state == 2;
    }

    static std::uint32_t* bethesdaAllocatorContextSlot()
    {
#if defined(_M_X64)
        static REL::Relocation<std::uint32_t*> tlsIndex{ REL::Offset(offsets::kData_BethesdaTlsIndex) };
        auto* tlsSlots = reinterpret_cast<void**>(__readgsqword(0x58));
        if (!tlsSlots) {
            return nullptr;
        }

        auto* tlsBlock = static_cast<std::uint8_t*>(tlsSlots[*tlsIndex]);
        if (!tlsBlock) {
            return nullptr;
        }

        return reinterpret_cast<std::uint32_t*>(tlsBlock + offsets::kBethesdaTlsAllocatorContext);
#else
        return nullptr;
#endif
    }

    class BethesdaAllocatorContextGuard
    {
    public:
        explicit BethesdaAllocatorContextGuard(std::uint32_t context)
        {
            _slot = bethesdaAllocatorContextSlot();
            if (_slot) {
                _previous = *_slot;
                *_slot = context;
                _active = true;
            }
        }

        ~BethesdaAllocatorContextGuard()
        {
            if (_active && _slot) {
                *_slot = _previous;
            }
        }

        BethesdaAllocatorContextGuard(const BethesdaAllocatorContextGuard&) = delete;
        BethesdaAllocatorContextGuard& operator=(const BethesdaAllocatorContextGuard&) = delete;

        bool active() const { return _active; }

    private:
        std::uint32_t* _slot = nullptr;
        std::uint32_t _previous = 0;
        bool _active = false;
    };

    static void* bethesdaAlloc(std::size_t size)
    {
        typedef void* (*alloc_t)(void*, std::size_t, std::uint32_t, char);
        static REL::Relocation<alloc_t> allocFunc{ REL::Offset(offsets::kFunc_BethesdaAlloc) };

        if (!ensureBethesdaAllocatorInitialized()) {
            return nullptr;
        }

        return allocFunc(bethesdaAllocatorPool(), size, 0, '\0');
    }

    using PhysicsSystemDataCtor_t = void* (*)(void*);
    using BodyCinfoCtor_t = void* (*)(void*);
    using MotionCinfoCtor_t = void* (*)(void*);
    using MaterialCtor_t = void* (*)(void*);
    using PhysicsSystemCtor_t = void* (*)(void*, void*);
    using PhysicsSystemGetBodyId_t = void (*)(void*, RE::hknpBodyId*, std::int32_t);
    using CollisionObjectCtor_t = void* (*)(void*, std::uint32_t, void*);
    using CollisionObjectAddToWorld_t = void (*)(void*, void*);
    using SetMotionType_t = void (*)(void*, int);
    using LinkObject_t = void (*)(void*, void*);
    using SetBodyKeyframed_t = void (*)(void*, std::uint32_t);
    using SetBodyMaterial_t = void (*)(void*, std::uint32_t, std::uint16_t, std::int32_t);
    using RemovePhysicsSystem_t = void (*)(void*, void*);

    using DriveToKeyFrame_t = std::uint8_t (*)(void*, const void*, float);
    using SetTransform_t = std::uint8_t (*)(void*, const void*);
    using SetVelocity_t = std::uint8_t (*)(void*, const float*, const float*);
    using ApplyImpulse_t = std::uint8_t (*)(void*, const float*);
    using ApplyPointImpulse_t = std::uint8_t (*)(void*, const float*, const float*);
    using SetMass_t = void (*)(void*, float);
    using GetCOM_t = std::uint8_t (*)(void*, float*);
    using GetFilterInfo_t = std::uint32_t* (*)(void*, std::uint32_t*);
    using GetShape_t = void* (*)(void*);
    using IsConstrained_t = bool (*)(void*);

    using EnableBodyFlags_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
    constexpr std::uint16_t kGeneratedSystemLocalMaterialIndex = 0;
    constexpr std::uint32_t kInvalidGeneratedId = 0x7FFF'FFFF;
    constexpr std::uint32_t kStaticLocalMotionIndex = kInvalidGeneratedId;

    static std::uint16_t generatedInitialQualityId(BethesdaMotionType motionType)
    {
        /*
         * Generated wrapper bodies enter FO4VR through hknpPhysicsSystemData,
         * not direct hknpWorld::CreateBody. Ghidra shows the wrapper treats
         * bodyCinfo+0x0C as a local motion-cinfo index and converts
         * 0x7FFFFFFF to static motion 0 before creation. Therefore non-static
         * ROCK colliders must supply a local motion cinfo up front and must not
         * patch in a world motion after AddToWorld. The keyframed state is
         * still applied after AddToWorld through Bethesda's own keyframed body
         * path, matching the last known-good direct-call behavior.
         */
        switch (motionType) {
        case BethesdaMotionType::Static:
            return 0;
        case BethesdaMotionType::Dynamic:
            return 1;
        case BethesdaMotionType::Keyframed:
            return 0xFF;
        }

        return 0xFF;
    }

    static bool isUsableGeneratedMotion(std::uint32_t motionIndex)
    {
        return motionIndex != 0 && motionIndex != 0x7FFF'FFFF && motionIndex != 0xFFFF'FFFF;
    }

    static void* nativePhysicsSystemInstance(void* physicsSystem)
    {
        /*
         * FO4VR keeps two ownership layers here: bhkPhysicsSystem is the
         * ref-counted Bethesda wrapper stored on bhkNPCollisionObject, while
         * bhkWorld::RemovePhysicsSystem and native body operations consume the
         * runtime hknpPhysicsSystemInstance created by AddToWorld. Centralizing
         * this unwrap prevents teardown and per-body calls from crossing the
         * wrapper/native boundary with the wrong pointer type.
         */
        if (!physicsSystem) {
            return nullptr;
        }

        return *reinterpret_cast<void**>(reinterpret_cast<char*>(physicsSystem) + offsets::kBhkPhysicsSystem_Instance);
    }

    static RE::hknpWorld* nativeWorldFromPhysicsSystem(void* physicsSystem)
    {
        auto* instance = nativePhysicsSystemInstance(physicsSystem);
        if (!instance) {
            return nullptr;
        }

        return *reinterpret_cast<RE::hknpWorld**>(reinterpret_cast<char*>(instance) + offsets::kHknpPhysicsSystemInstance_World);
    }

    static bool validateGeneratedBodyMotion(RE::hknpWorld* world, RE::hknpBodyId bodyId, BethesdaMotionType motionType)
    {
        if (!world || bodyId.value == kInvalidGeneratedId) {
            return false;
        }

        if (motionType == BethesdaMotionType::Static) {
            return true;
        }

        const auto before = havok_runtime::snapshotBody(world, bodyId);
        if (!before.valid) {
            ROCK_LOG_ERROR(BethesdaBody, "Native wrapper body {} is not readable after add-to-world", bodyId.value);
            return false;
        }

        if (isUsableGeneratedMotion(before.motionIndex)) {
            return true;
        }

        ROCK_LOG_ERROR(
            BethesdaBody,
            "Generated body {} was created without a live non-static motion: observedMotion={}",
            bodyId.value,
            before.motionIndex);
        return false;
    }

    static bool applyGeneratedBodyMaterial(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::hknpMaterialId materialId)
    {
        /*
         * FO4VR remaps hknpPhysicsSystemData material IDs through the local
         * system-data material array during AddToWorld. ROCK therefore seeds
         * generated systems with local material index 0 and assigns the desired
         * global world material only after the native body exists. This prevents
         * a local/global material mismatch from feeding invalid material data
         * into Havok's surface-velocity contact modifier.
         */
        if (!world || bodyId.value == 0x7FFF'FFFF || materialId.value == 0xFFFF) {
            return false;
        }

        static REL::Relocation<SetBodyMaterial_t> setBodyMaterial{ REL::Offset(offsets::kFunc_HknpWorld_SetBodyMaterial) };
        setBodyMaterial(world, bodyId.value, materialId.value, 0);

        const auto snapshot = havok_runtime::snapshotBody(world, bodyId);
        if (!snapshot.valid || !snapshot.body || snapshot.body->materialId.value != materialId.value) {
            ROCK_LOG_ERROR(
                BethesdaBody,
                "Generated body {} material assignment failed: requested={} observed={} readable={}",
                bodyId.value,
                materialId.value,
                snapshot.body ? snapshot.body->materialId.value : 0xFFFF,
                snapshot.valid ? "yes" : "no");
            return false;
        }

        ROCK_LOG_DEBUG(BethesdaBody, "Generated body {} assigned world material {}", bodyId.value, materialId.value);
        return true;
    }

    static void applyCollisionObjectMotionType(void* collisionObject, BethesdaMotionType motionType)
    {
        if (!collisionObject) {
            return;
        }

        static REL::Relocation<SetMotionType_t> setMotion{ REL::Offset(offsets::kFunc_CollisionObject_SetMotionType) };
        setMotion(collisionObject, static_cast<int>(motionType));
    }

    static void applyGeneratedBodyMotionType(RE::hknpWorld* world, void* collisionObject, RE::hknpBodyId bodyId, BethesdaMotionType motionType)
    {
        /*
         * ROCK-generated bodies are solver drivers. The bhk collision-object
         * wrapper is still needed for Bethesda methods, but keyframed driver
         * bodies must be promoted on the hknp body itself. Dynamic/fixed modes
         * keep using the wrapper method because Bethesda routes those through
         * the collision object consistently.
         */
        if (motionType == BethesdaMotionType::Keyframed && world && bodyId.value != 0x7FFF'FFFF) {
            static REL::Relocation<SetBodyKeyframed_t> setBodyKeyframed{ REL::Offset(offsets::kFunc_SetBodyKeyframed) };
            setBodyKeyframed(world, bodyId.value);
            return;
        }

        applyCollisionObjectMotionType(collisionObject, motionType);
    }

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

        _systemData = havok_runtime::allocateHavok(0x78);
        if (!_systemData) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate hknpPhysicsSystemData (0x78 bytes)");
            return false;
        }
        {
            static REL::Relocation<PhysicsSystemDataCtor_t> ctor{ REL::Offset(offsets::kFunc_PhysicsSystemData_Ctor) };
            ctor(_systemData);
        }
        bool ownsSystemDataLocalRef = true;
        auto releaseLocalSystemDataRef = [&]() {
            if (ownsSystemDataLocalRef && _systemData) {
                havok_ref_count::release(_systemData);
                ownsSystemDataLocalRef = false;
            }
        };

        auto hkArrayAppendOne = [&](char* arrayBase, int stride) -> char* {
            auto*& dataPtr = *reinterpret_cast<char**>(arrayBase);
            auto& size = *reinterpret_cast<std::int32_t*>(arrayBase + 0x08);
            auto& capFlags = *reinterpret_cast<std::int32_t*>(arrayBase + 0x0C);
            std::int32_t capacity = capFlags & 0x3FFFFFFF;

            if (size >= capacity) {
                if (!havok_runtime::hkArrayReserveMore(arrayBase, stride)) {
                    return nullptr;
                }
            }

            if (!dataPtr)
                return nullptr;

            char* newEntry = dataPtr + size * stride;
            size++;
            return newEntry;
        };

        std::uint32_t generatedLocalMotionIndex = kStaticLocalMotionIndex;
        if (motionType != BethesdaMotionType::Static) {
            auto* motionCinfoArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_MotionCinfos;
            const auto motionIndexBeforeAppend = *reinterpret_cast<std::int32_t*>(motionCinfoArray + 0x08);
            if (motionIndexBeforeAppend < 0) {
                ROCK_LOG_ERROR(BethesdaBody, "Generated motion-cinfo array has invalid size {}", motionIndexBeforeAppend);

                releaseLocalSystemDataRef();
                _systemData = nullptr;
                return false;
            }

            auto* motionCinfo = hkArrayAppendOne(motionCinfoArray, 0x70);
            if (!motionCinfo) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to grow motionCinfos array");

                releaseLocalSystemDataRef();
                _systemData = nullptr;
                return false;
            }

            static REL::Relocation<MotionCinfoCtor_t> motionCinfoCtor{ REL::Offset(offsets::kFunc_MotionCinfo_Ctor) };
            motionCinfoCtor(motionCinfo);
            generatedLocalMotionIndex = static_cast<std::uint32_t>(motionIndexBeforeAppend);
        }

        void* bodyCinfo = nullptr;
        {
            auto* bodyCinfoArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_BodyCinfos;
            bodyCinfo = hkArrayAppendOne(bodyCinfoArray, 0x60);
            if (!bodyCinfo) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to grow bodyCinfos array");

                releaseLocalSystemDataRef();
                _systemData = nullptr;
                return false;
            }

            static REL::Relocation<BodyCinfoCtor_t> cinfoInit{ REL::Offset(offsets::kFunc_BodyCinfo_Ctor) };
            cinfoInit(bodyCinfo);

            auto* ci = reinterpret_cast<char*>(bodyCinfo);
            *reinterpret_cast<RE::hknpShape**>(ci + 0x00) = shape;
            *reinterpret_cast<std::uint32_t*>(ci + 0x08) = kInvalidGeneratedId;
            *reinterpret_cast<std::uint32_t*>(ci + 0x0C) = generatedLocalMotionIndex;
            *reinterpret_cast<std::uint16_t*>(ci + 0x10) = generatedInitialQualityId(motionType);
            *reinterpret_cast<std::uint16_t*>(ci + 0x12) = kGeneratedSystemLocalMaterialIndex;
            *reinterpret_cast<std::uint32_t*>(ci + 0x14) = filterInfo;
            *reinterpret_cast<const char**>(ci + 0x20) = name;
            *reinterpret_cast<std::uintptr_t*>(ci + 0x28) = 0;
        }

        {
            auto* materialsArray = reinterpret_cast<char*>(_systemData) + offsets::kSysData_Materials;
            auto* material = hkArrayAppendOne(materialsArray, 0x50);
            if (!material) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to grow materials array");

                releaseLocalSystemDataRef();
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
                havok_ref_count::addRef(shape);
            }
        }

        _physicsSystem = bethesdaAlloc(0x28);
        if (!_physicsSystem) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate bhkPhysicsSystem (0x28 bytes)");

            releaseLocalSystemDataRef();
            _systemData = nullptr;
            return false;
        }
        {
            static REL::Relocation<PhysicsSystemCtor_t> physSysCtor{ REL::Offset(offsets::kFunc_PhysicsSystem_Ctor) };
            physSysCtor(_physicsSystem, _systemData);
            releaseLocalSystemDataRef();
        }

        /*
         * ROCK-generated colliders need Bethesda wrapper ownership and hknp
         * broadphase registration to behave like normal game bodies. FO4VR's
         * collision-object phase at vfunction49 creates the runtime physics
         * system instance, inserts it into bhkWorld, and publishes the body
         * back-pointer; calling the lower creation phase alone leaves a body ID
         * that Havok contact/constraint code does not actually solve against.
         */
        {
            BethesdaAllocatorContextGuard collisionObjectAllocatorContext{ 0x41 };
            if (!collisionObjectAllocatorContext.active()) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to access Bethesda allocator TLS context for bhkNPCollisionObject allocation");

                releaseRefCounted(_physicsSystem);
                _physicsSystem = nullptr;
                _systemData = nullptr;
                return false;
            }

            _collisionObject = bethesdaAlloc(0x30);
            if (!_collisionObject) {
                ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate bhkNPCollisionObject (0x30 bytes)");

                releaseRefCounted(_physicsSystem);
                _physicsSystem = nullptr;
                _systemData = nullptr;
                return false;
            }

            static REL::Relocation<CollisionObjectCtor_t> collObjCtor{ REL::Offset(offsets::kFunc_CollisionObject_Ctor) };
            collObjCtor(_collisionObject, 0, _physicsSystem);
        }

        if (!createNiNode(name)) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to create/link owner NiNode for generated body '{}'", name ? name : "(null)");
            releaseRefCounted(_collisionObject);
            _collisionObject = nullptr;
            _physicsSystem = nullptr;
            _systemData = nullptr;
            return false;
        }

        {
            static REL::Relocation<CollisionObjectAddToWorld_t> addToWorld{ REL::Offset(offsets::kFunc_CollisionObject_AddToWorld) };
            addToWorld(_collisionObject, bhkWorld);
        }

        RE::hknpBodyId bodyId{ kInvalidGeneratedId };
        {
            static REL::Relocation<PhysicsSystemGetBodyId_t> getBodyId{ REL::Offset(offsets::kFunc_PhysicsSystem_GetBodyId) };
            getBodyId(_physicsSystem, &bodyId, 0);
            if (bodyId.value == kInvalidGeneratedId) {
                ROCK_LOG_ERROR(BethesdaBody, "Native wrapper add-to-world returned invalid body id for '{}'", name ? name : "(null)");
                destroy(bhkWorld);
                return false;
            }

            _bodyId = bodyId;
        }

        if (!validateGeneratedBodyMotion(world, bodyId, motionType)) {
            destroy(bhkWorld);
            return false;
        }

        if (!applyGeneratedBodyMaterial(world, bodyId, materialId)) {
            destroy(bhkWorld);
            return false;
        }

        applyGeneratedBodyMotionType(world, _collisionObject, bodyId, motionType);

        havok_runtime::setFilterInfo(world, bodyId, filterInfo, 1);

        {
            static REL::Relocation<EnableBodyFlags_t> enableFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
            enableFlags(world, bodyId.value, 0x08020000, 1);
        }

        havok_runtime::activateBody(world, bodyId.value);

        const auto snapshot = havok_runtime::snapshotBody(world, bodyId);
        if (snapshot.valid) {
            ROCK_LOG_DEBUG(BethesdaBody,
                "Native wrapper body state: bodyId={} motion={} filter=0x{:08X} collObj={:p} ownerNode={:p}",
                bodyId.value,
                snapshot.motionIndex,
                snapshot.collisionFilterInfo,
                static_cast<void*>(snapshot.collisionObject),
                static_cast<void*>(snapshot.ownerNode));
        } else {
            ROCK_LOG_ERROR(BethesdaBody, "Native wrapper body {} is not readable after setup", bodyId.value);
        }

        _created = true;

        ROCK_LOG_DEBUG(BethesdaBody, "Created '{}': bodyId={} collObj={:p} physSys={:p} sysData={:p} motionType={}", name, _bodyId.value, _collisionObject, _physicsSystem,
            _systemData, static_cast<int>(motionType));

        {
            auto* backPtr = havok_runtime::getCollisionObjectFromBody(world, _bodyId);
            if (backPtr == _collisionObject) {
                ROCK_LOG_DEBUG(BethesdaBody, "  body+0x88 back-pointer VERIFIED: {:p} == collisionObject", static_cast<void*>(backPtr));
            } else {
                ROCK_LOG_ERROR(BethesdaBody, "  body+0x88 back-pointer MISMATCH: {:p} != {:p}", static_cast<void*>(backPtr), _collisionObject);
            }
        }

        return true;
    }

    void BethesdaPhysicsBody::destroy(void* bhkWorld)
    {
        if (!_created && !_collisionObject && !_niNode)
            return;

        ROCK_LOG_DEBUG(BethesdaBody, "Destroying body: bodyId={} collObj={:p}", _bodyId.value, _collisionObject);

        auto* physicsSystemInstance = nativePhysicsSystemInstance(_physicsSystem);
        if (bhkWorld && physicsSystemInstance) {
            static REL::Relocation<RemovePhysicsSystem_t> removePhysicsSystem{ REL::Offset(offsets::kFunc_BhkWorld_RemovePhysicsSystemInstance) };
            removePhysicsSystem(bhkWorld, physicsSystemInstance);
            ROCK_LOG_DEBUG(BethesdaBody, "Removed native physics system instance for body {}", _bodyId.value);
        } else if (_physicsSystem) {
            ROCK_LOG_WARN(
                BethesdaBody,
                "Destroying body {} without native physics-system removal: bhkWorld={:p} instance={:p}",
                _bodyId.value,
                bhkWorld,
                physicsSystemInstance);
        }

        destroyNiNode();

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
        if (!_collisionObject) {
            ROCK_LOG_ERROR(BethesdaBody, "createNiNode: no collision object");
            return false;
        }
        if (_niNode) {
            ROCK_LOG_DEBUG(BethesdaBody, "createNiNode: already has NiNode");
            return true;
        }

        void* mem = bethesdaAlloc(offsets::kNiNodeSize);
        if (!mem) {
            ROCK_LOG_ERROR(BethesdaBody, "Failed to allocate NiNode (0x{:X} bytes)", offsets::kNiNodeSize);
            return false;
        }

        std::memset(mem, 0, offsets::kNiNodeSize);

        {
            using NiNodeCtor_t = void* (*)(void*, std::uint16_t);
            static REL::Relocation<NiNodeCtor_t> niNodeCtor{ REL::Offset(offsets::kFunc_NiNode_Ctor) };
            niNodeCtor(mem, 0);
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
            auto* collObjOwner = *reinterpret_cast<void**>(reinterpret_cast<char*>(_collisionObject) + offsets::kCollisionObject_OwnerNode);
            auto* nodeCollObj = *reinterpret_cast<void**>(reinterpret_cast<char*>(_niNode) + offsets::kNiAVObject_CollisionObject);

            bool ownerOk = (collObjOwner == _niNode);
            bool collOk = (nodeCollObj == _collisionObject);

            ROCK_LOG_DEBUG(BethesdaBody, "NiNode '{}' created: node={:p} collObjOwner={} nodeCollObj={}", name ? name : "(null)", _niNode, ownerOk ? "VERIFIED" : "MISMATCH",
                collOk ? "VERIFIED" : "MISMATCH");
        }

        return true;
    }

    void BethesdaPhysicsBody::destroyNiNode()
    {
        if (!_niNode)
            return;

        if (_collisionObject) {
            *reinterpret_cast<void**>(reinterpret_cast<char*>(_collisionObject) + offsets::kCollisionObject_OwnerNode) = nullptr;
        }

        auto** nodeCollisionObjectSlot = reinterpret_cast<void**>(reinterpret_cast<char*>(_niNode) + offsets::kNiAVObject_CollisionObject);
        void* nodeCollisionObject = *nodeCollisionObjectSlot;
        *nodeCollisionObjectSlot = nullptr;
        if (nodeCollisionObject) {
            releaseRefCounted(nodeCollisionObject);
        }

        releaseRefCounted(_niNode);
        _niNode = nullptr;

        ROCK_LOG_DEBUG(BethesdaBody, "NiNode destroyed");
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

    bool BethesdaPhysicsBody::driveToKeyFrame(const RE::hkTransformf& target, float dt)
    {
        if (!isValid())
            return false;
        static REL::Relocation<DriveToKeyFrame_t> drive{ REL::Offset(offsets::kFunc_CollisionObject_DriveToKeyFrame) };
        return drive(_collisionObject, &target, dt) != 0;
    }

    bool BethesdaPhysicsBody::setTransform(const RE::hkTransformf& transform)
    {
        if (!isValid())
            return false;
        static REL::Relocation<SetTransform_t> setXform{ REL::Offset(offsets::kFunc_CollisionObject_SetTransform) };
        return setXform(_collisionObject, &transform) != 0;
    }

    bool BethesdaPhysicsBody::setVelocity(const float* linVel, const float* angVel)
    {
        if (!isValid())
            return false;
        static REL::Relocation<SetVelocity_t> setVel{ REL::Offset(offsets::kFunc_CollisionObject_SetVelocity) };
        return setVel(_collisionObject, linVel, angVel) != 0;
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

        auto* world = nativeWorldFromPhysicsSystem(_physicsSystem);
        if (!world)
            return;
        havok_runtime::setFilterInfo(world, _bodyId, filterInfo, rebuildMode);
    }

    void BethesdaPhysicsBody::setMass(float mass)
    {
        if (!isValid())
            return;
        static REL::Relocation<SetMass_t> setM{ REL::Offset(offsets::kFunc_CollisionObject_SetMass) };
        setM(_collisionObject, mass);
    }

    bool BethesdaPhysicsBody::applyLinearImpulse(const float* impulse)
    {
        if (!isValid())
            return false;
        static REL::Relocation<ApplyImpulse_t> apply{ REL::Offset(offsets::kFunc_CollisionObject_ApplyLinearImpulse) };
        return apply(_collisionObject, impulse) != 0;
    }

    bool BethesdaPhysicsBody::applyPointImpulse(const float* impulse, const float* worldPoint)
    {
        if (!isValid())
            return false;
        static REL::Relocation<ApplyPointImpulse_t> apply{ REL::Offset(offsets::kFunc_CollisionObject_ApplyPointImpulse) };
        return apply(_collisionObject, impulse, worldPoint) != 0;
    }

    bool BethesdaPhysicsBody::getCenterOfMassWorld(float& outX, float& outY, float& outZ)
    {
        if (!isValid())
            return false;
        alignas(16) float com[4] = { 0, 0, 0, 0 };
        static REL::Relocation<GetCOM_t> getCOM{ REL::Offset(offsets::kFunc_CollisionObject_GetCOMWorld) };
        bool ok = getCOM(_collisionObject, com) != 0;
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
        std::uint32_t filterInfo = 0xFFFF'FFFF;
        getFilter(_collisionObject, &filterInfo);
        return filterInfo;
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
        auto* world = nativeWorldFromPhysicsSystem(_physicsSystem);
        if (!world)
            return;

        static REL::Relocation<EnableBodyFlags_t> enableFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
        enableFlags(world, _bodyId.value, flags, mode);
    }

    void BethesdaPhysicsBody::activateBody()
    {
        if (!isValid())
            return;
        auto* world = nativeWorldFromPhysicsSystem(_physicsSystem);
        if (!world)
            return;

        havok_runtime::activateBody(world, _bodyId.value);
    }

}

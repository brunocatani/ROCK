#include "physics-interaction/native/BethesdaPhysicsBody.h"

#include "physics-interaction/native/HavokRefCount.h"
#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/PhysicsLog.h"

#include "RE/Havok/hknpBody.h"

#include <array>
#include <atomic>
#include <cmath>
#include <intrin.h>
#include <mutex>
#include <windows.h>

namespace rock
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
    using SetMass_t = void (*)(void*, float);
    using GetCOM_t = std::uint8_t (*)(void*, float*);
    using GetFilterInfo_t = std::uint32_t* (*)(void*, std::uint32_t*);
    using GetShape_t = void* (*)(void*);
    using IsConstrained_t = bool (*)(void*);

    using EnableBodyFlags_t = void (*)(void*, std::uint32_t, std::uint32_t, std::uint32_t);
    constexpr std::uint16_t kGeneratedSystemLocalMaterialIndex = 0;
    constexpr std::uint32_t kInvalidGeneratedId = 0x7FFF'FFFF;
    constexpr std::uint32_t kStaticLocalMotionIndex = kInvalidGeneratedId;
    constexpr std::uint32_t kGeneratedBodyRuntimeFlags = 0x0802'0000;
    constexpr std::uint32_t kRebuildBodyCollisionState = 0;

    namespace
    {
        struct RetiredDeferredBody
        {
            RetiredBethesdaPhysicsBodyPayload payload{};
            std::uint32_t remainingPhysicsSteps = 0;
            bool processLifetimeHold = false;

            [[nodiscard]] bool occupied() const { return payload.occupied(); }
        };

        /*
         * Fixed process-lifetime ownership for neutralized bhkNPCollisionObject
         * addresses. Native FO4VR readers retain uncounted raw pointers after a
         * generated body leaves the world. The 2026-08-28 weapon-switch crash at
         * Fallout4VR.exe+1E08DF3 proved that eight completed physics steps are not
         * a release boundary: GetCollisionFilterInfo still reached the old wrapper
         * and indexed hknpBody with its stale mapped ID.
         *
         * Each quarantined object is reduced to its 0x30-byte wrapper; its owner
         * node and bhkPhysicsSystem are detached after the existing grace window.
         * The address then remains valid until process exit. Capacity is fixed so
         * repeated weapon changes cannot grow an unbounded container. Exhaustion
         * fails closed by rejecting all subsequent generated-body creation.
         */
        inline constexpr std::uint32_t kRetiredDeferredBodyGraceSteps = 8;
        inline constexpr std::size_t kMaxRetiredDeferredBodies = 512;
        inline constexpr std::size_t kMaxProcessLifetimeCollisionObjectTombstones = 65'536;

        std::mutex s_retiredDeferredBodyMutex;
        std::array<RetiredDeferredBody, kMaxRetiredDeferredBodies> s_retiredDeferredBodies{};
        std::uint32_t s_retiredDeferredBodyCount = 0;
        std::mutex s_collisionObjectTombstoneMutex;
        std::array<void*, kMaxProcessLifetimeCollisionObjectTombstones> s_collisionObjectTombstones{};
        std::size_t s_collisionObjectTombstoneCount = 0;
        std::atomic_bool s_retirementQuarantineExhausted{ false };
        std::atomic_bool s_retirementCreateRejectionLogged{ false };

        void markRetirementQuarantineExhausted(
            const char* owner,
            std::uint32_t bodyId,
            void* collisionObject,
            std::size_t capacity)
        {
            const bool firstFailure =
                !s_retirementQuarantineExhausted.exchange(true, std::memory_order_acq_rel);
            if (firstFailure) {
                ROCK_LOG_CRITICAL(
                    BethesdaBody,
                    "Generated-body retirement quarantine exhausted owner={} body={} collisionObject={:p} capacity={}; future generated-body creation is disabled",
                    owner ? owner : "unknown",
                    bodyId,
                    collisionObject,
                    capacity);
            }
        }
    }

    static std::uint16_t generatedInitialMotionPropertiesId(BethesdaMotionType motionType)
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

    static bool validateGeneratedBodyCollisionProfile(
        RE::hknpWorld* world,
        RE::hknpBodyId bodyId,
        const BethesdaPhysicsBodyCreationOptions& options)
    {
        if (options.bodyQuality == BethesdaGeneratedBodyQuality::Default &&
            options.collisionLookAheadDistanceHavok == 0.0f) {
            return true;
        }

        const auto snapshot = havok_runtime::snapshotBody(world, bodyId);
        if (!snapshot.valid || !snapshot.body) {
            ROCK_LOG_ERROR(BethesdaBody, "Generated body {} collision profile is not readable", bodyId.value);
            return false;
        }

        const auto expectedQuality = static_cast<std::uint8_t>(options.bodyQuality);
        const auto observedQuality = snapshot.body->qualityId;
        const float observedLookAhead = snapshot.body->translation[3];
        if (observedQuality != expectedQuality ||
            !std::isfinite(observedLookAhead) ||
            std::abs(observedLookAhead - options.collisionLookAheadDistanceHavok) > 0.0001f) {
            ROCK_LOG_ERROR(
                BethesdaBody,
                "Generated body {} collision profile mismatch: requestedQuality={} observedQuality={} requestedLookAhead={:.4f} observedLookAhead={:.4f}",
                bodyId.value,
                expectedQuality,
                observedQuality,
                options.collisionLookAheadDistanceHavok,
                observedLookAhead);
            return false;
        }

        return true;
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
        BethesdaMotionType motionType, const char* name, const BethesdaPhysicsBodyCreationOptions& options)
    {
        if (_created) {
            ROCK_LOG_WARN(BethesdaBody, "create() called on already-created body — destroy first");
            return false;
        }
        if (s_retirementQuarantineExhausted.load(std::memory_order_acquire)) {
            if (!s_retirementCreateRejectionLogged.exchange(true, std::memory_order_acq_rel)) {
                ROCK_LOG_CRITICAL(
                    BethesdaBody,
                    "Generated-body creation rejected because the process-lifetime retirement quarantine is exhausted");
            }
            return false;
        }
        if (!world || !bhkWorld || !shape) {
            ROCK_LOG_ERROR(BethesdaBody, "create() null params: world={} bhkWorld={} shape={}", (void*)world, bhkWorld, (void*)shape);
            return false;
        }
        if (!std::isfinite(options.collisionLookAheadDistanceHavok) ||
            options.collisionLookAheadDistanceHavok < 0.0f) {
            ROCK_LOG_ERROR(
                BethesdaBody,
                "create() invalid collision look-ahead for '{}': {:.4f}",
                name ? name : "(null)",
                options.collisionLookAheadDistanceHavok);
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
            /*
             * Keep the constructor's dynamic-safe defaults. FO4VR
             * 0x1417A3A90 is initializeAsKeyFramed, not generic mass
             * derivation: it zeros cinfo+0x04 (inverse mass) and selects
             * keyframed properties. Re-labeling that result as dynamic makes
             * velocity-driven ROCK bodies move without solver displacement.
             * The body cinfo below selects the actual motion profile.
             */
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
            *reinterpret_cast<std::uint16_t*>(ci + 0x10) = generatedInitialMotionPropertiesId(motionType);
            *reinterpret_cast<std::uint16_t*>(ci + 0x12) = kGeneratedSystemLocalMaterialIndex;
            *reinterpret_cast<std::uint32_t*>(ci + 0x14) = filterInfo;
            *reinterpret_cast<float*>(ci + 0x1C) = options.collisionLookAheadDistanceHavok;
            *reinterpret_cast<const char**>(ci + 0x20) = name;
            *reinterpret_cast<std::uintptr_t*>(ci + 0x28) = 0;
            *reinterpret_cast<std::uint8_t*>(ci + 0x50) = static_cast<std::uint8_t>(options.bodyQuality);
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
        _createdHknpWorld = world;
        _createdBhkWorld = bhkWorld;

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

        if (!validateGeneratedBodyCollisionProfile(world, bodyId, options)) {
            destroy(bhkWorld);
            return false;
        }

        if (!applyGeneratedBodyMaterial(world, bodyId, materialId)) {
            destroy(bhkWorld);
            return false;
        }

        applyGeneratedBodyMotionType(world, _collisionObject, bodyId, motionType);

        /*
         * Batch the filter write with the flag publication below. FO4VR's
         * enable-flags mode zero runs 0x14153C5A0 after changing body+0x40;
         * raw disassembly shows that path invalidates cached collision state
         * and queues the live body for recomputation. Skipping it leaves a
         * newly inserted body physically solvable while filtered collision
         * modifiers can retain the pre-setup eligibility state.
         */
        havok_runtime::setFilterInfo(world, bodyId, filterInfo, 1);

        {
            static REL::Relocation<EnableBodyFlags_t> enableFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
            enableFlags(world, bodyId.value, kGeneratedBodyRuntimeFlags, kRebuildBodyCollisionState);
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

        auto* instanceWorld = nativeWorldFromPhysicsSystem(_physicsSystem);
        auto* wrapperWorld = havok_runtime::getHknpWorldFromBhk(reinterpret_cast<RE::bhkWorld*>(bhkWorld));
        if (instanceWorld != world || wrapperWorld != world) {
            ROCK_LOG_ERROR(
                BethesdaBody,
                "Generated body world identity mismatch after create: bodyId={} requestedHknp={:p} wrapperHknp={:p} instanceHknp={:p} bhk={:p}",
                _bodyId.value,
                static_cast<void*>(world),
                static_cast<void*>(wrapperWorld),
                static_cast<void*>(instanceWorld),
                bhkWorld);
            destroy(bhkWorld);
            return false;
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

    static void detachAndReleaseNiNode(void* collisionObject, void*& niNode)
    {
        if (!niNode)
            return;

        if (collisionObject) {
            *reinterpret_cast<void**>(reinterpret_cast<char*>(collisionObject) + offsets::kCollisionObject_OwnerNode) = nullptr;
        }

        auto** nodeCollisionObjectSlot = reinterpret_cast<void**>(reinterpret_cast<char*>(niNode) + offsets::kNiAVObject_CollisionObject);
        void* nodeCollisionObject = *nodeCollisionObjectSlot;
        *nodeCollisionObjectSlot = nullptr;
        if (nodeCollisionObject) {
            releaseRefCounted(nodeCollisionObject);
        }

        releaseRefCounted(niNode);
        niNode = nullptr;
    }

    void BethesdaPhysicsBody::destroy(void* bhkWorld)
    {
        if (!_created && !_collisionObject && !_niNode)
            return;

        ROCK_LOG_DEBUG(BethesdaBody, "Destroying body: bodyId={} collObj={:p}", _bodyId.value, _collisionObject);

        auto* physicsSystemInstance = nativePhysicsSystemInstance(_physicsSystem);
        if (bhkWorld && physicsSystemInstance && !matchesCreationWorld(_createdHknpWorld, bhkWorld)) {
            ROCK_LOG_ERROR(
                BethesdaBody,
                "Rejected body destruction through mismatched world: bodyId={} callerBhk={:p} createdBhk={:p} createdHknp={:p} instanceHknp={:p}",
                _bodyId.value,
                bhkWorld,
                _createdBhkWorld,
                static_cast<void*>(_createdHknpWorld),
                static_cast<void*>(nativeWorldFromPhysicsSystem(_physicsSystem)));
            return;
        }
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

    bool BethesdaPhysicsBody::retireFromWorld(void* bhkWorld, RetiredBethesdaPhysicsBodyPayload& outPayload)
    {
        if (outPayload.occupied()) {
            ROCK_LOG_ERROR(BethesdaBody, "retireFromWorld called with an occupied output payload");
            return false;
        }

        if (!_created && !_collisionObject && !_niNode) {
            return false;
        }

        ROCK_LOG_DEBUG(BethesdaBody, "Retiring body from world: bodyId={} collObj={:p}", _bodyId.value, _collisionObject);

        auto* physicsSystemInstance = nativePhysicsSystemInstance(_physicsSystem);
        if (bhkWorld && physicsSystemInstance && !matchesCreationWorld(_createdHknpWorld, bhkWorld)) {
            ROCK_LOG_ERROR(
                BethesdaBody,
                "Rejected body retirement through mismatched world: bodyId={} callerBhk={:p} createdBhk={:p} createdHknp={:p} instanceHknp={:p}",
                _bodyId.value,
                bhkWorld,
                _createdBhkWorld,
                static_cast<void*>(_createdHknpWorld),
                static_cast<void*>(nativeWorldFromPhysicsSystem(_physicsSystem)));
            return false;
        }
        if (bhkWorld && physicsSystemInstance) {
            static REL::Relocation<RemovePhysicsSystem_t> removePhysicsSystem{ REL::Offset(offsets::kFunc_BhkWorld_RemovePhysicsSystemInstance) };
            removePhysicsSystem(bhkWorld, physicsSystemInstance);
            ROCK_LOG_DEBUG(BethesdaBody, "Removed native physics system instance for retired body {}", _bodyId.value);
        } else if (_physicsSystem) {
            ROCK_LOG_WARN(
                BethesdaBody,
                "Retiring body {} without native physics-system removal: bhkWorld={:p} instance={:p}",
                _bodyId.value,
                bhkWorld,
                physicsSystemInstance);
        }

        outPayload.collisionObject = _collisionObject;
        outPayload.niNode = _niNode;
        outPayload.retiredHknpWorld = _createdHknpWorld;
        outPayload.bodyId = _bodyId.value;
        reset();
        return outPayload.occupied();
    }

    bool BethesdaPhysicsBody::quarantineRetiredPayload(RetiredBethesdaPhysicsBodyPayload& payload)
    {
        if (!payload.occupied()) {
            payload = {};
            return true;
        }

        auto* collisionObject = payload.collisionObject;
        if (!collisionObject) {
            auto* niNode = payload.niNode;
            detachAndReleaseNiNode(nullptr, niNode);
            payload = {};
            return true;
        }

        std::size_t tombstoneIndex = 0;
        {
            std::scoped_lock quarantineLock(s_collisionObjectTombstoneMutex);
            if (s_collisionObjectTombstoneCount >= s_collisionObjectTombstones.size()) {
                markRetirementQuarantineExhausted(
                    "collision-object-tombstone",
                    payload.bodyId,
                    collisionObject,
                    s_collisionObjectTombstones.size());
                return false;
            }
            tombstoneIndex = s_collisionObjectTombstoneCount++;
        }

        /*
         * Reserve permanent ownership before neutralizing the object. The
         * payload owns the original collision-object reference transferred from
         * BethesdaPhysicsBody, while the NiNode owns a second reference. Detach
         * the node first, then sever and release the bhkPhysicsSystem edge. Any
         * late native call through the retained wrapper now takes the same null
         * fail-closed path as bhkNPCollisionObject::GetCollisionFilterInfo.
         */
        auto* niNode = payload.niNode;
        detachAndReleaseNiNode(collisionObject, niNode);

        auto* physicsSystemSlot = reinterpret_cast<void* volatile*>(
            reinterpret_cast<std::uintptr_t>(collisionObject) +
            offsets::kCollisionObject_PhysSystemPtr);
        auto* observedPhysicsSystem =
            InterlockedCompareExchangePointer(physicsSystemSlot, nullptr, nullptr);
        auto* observedInstance = nativePhysicsSystemInstance(observedPhysicsSystem);
        auto* observedInstanceWorld =
            observedInstance ? nativeWorldFromPhysicsSystem(observedPhysicsSystem) : nullptr;
        const auto systemBodyIndex = *reinterpret_cast<const std::uint32_t*>(
            reinterpret_cast<std::uintptr_t>(collisionObject) +
            offsets::kCollisionObject_SystemBodyIndex);
        RE::hknpBodyId mappedBodyId{ kInvalidGeneratedId };
        const bool mappingReadable =
            observedPhysicsSystem &&
            observedInstance &&
            observedInstanceWorld == payload.retiredHknpWorld &&
            systemBodyIndex < 4096;
        if (mappingReadable) {
            static REL::Relocation<PhysicsSystemGetBodyId_t> getBodyId{
                REL::Offset(offsets::kFunc_PhysicsSystem_GetBodyId)
            };
            getBodyId(
                observedPhysicsSystem,
                &mappedBodyId,
                static_cast<std::int32_t>(systemBodyIndex));
        }

        ROCK_LOG_SAMPLE_DEBUG(
            BethesdaBody,
            1000,
            "Retired wrapper quarantine witness body={} mappedBody={} mappingReadable={} systemBodyIndex={} collisionObject={:p} physicsSystem={:p} instance={:p} retiredWorld={:p} instanceWorld={:p}",
            payload.bodyId,
            mappedBodyId.value,
            mappingReadable ? "yes" : "no",
            systemBodyIndex,
            collisionObject,
            observedPhysicsSystem,
            observedInstance,
            static_cast<void*>(payload.retiredHknpWorld),
            static_cast<void*>(observedInstanceWorld));

        auto* physicsSystem = InterlockedExchangePointer(physicsSystemSlot, nullptr);
        if (physicsSystem) {
            releaseRefCounted(physicsSystem);
        }

        s_collisionObjectTombstones[tombstoneIndex] = collisionObject;
        const auto bodyId = payload.bodyId;
        payload = {};

        ROCK_LOG_SAMPLE_DEBUG(
            BethesdaBody,
            1000,
            "Retired body {} collision object neutralized and quarantined for process lifetime activeQuarantined={}",
            bodyId,
            tombstoneIndex + 1);
        return true;
    }

    void BethesdaPhysicsBody::retainRetiredPayloadForProcessLifetime(
        RetiredBethesdaPhysicsBodyPayload& payload,
        const char* owner,
        std::size_t capacity)
    {
        if (!payload.occupied()) {
            payload = {};
            return;
        }

        markRetirementQuarantineExhausted(
            owner,
            payload.bodyId,
            payload.collisionObject,
            capacity);
        // Ownership is intentionally transferred to process lifetime without a
        // release. Creation is now disabled, so only the finite set of bodies
        // already live when capacity failed can enter this emergency path.
        payload = {};
    }

    void BethesdaPhysicsBody::retireDeferred(void* bhkWorld)
    {
        RetiredBethesdaPhysicsBodyPayload payload{};
        if (!retireFromWorld(bhkWorld, payload) || !payload.occupied()) {
            return;
        }

        std::scoped_lock lock(s_retiredDeferredBodyMutex);
        for (auto& retired : s_retiredDeferredBodies) {
            if (!retired.occupied()) {
                retired.payload = payload;
                retired.remainingPhysicsSteps = kRetiredDeferredBodyGraceSteps;
                ++s_retiredDeferredBodyCount;
                ROCK_LOG_SAMPLE_DEBUG(BethesdaBody,
                    1000,
                    "Body {} collision object retired for {} physics steps activeRetired={}",
                    payload.bodyId,
                    kRetiredDeferredBodyGraceSteps,
                    s_retiredDeferredBodyCount);
                return;
            }
        }

        // Queue exhaustion is a terminal safety boundary for generated-body
        // creation. Preserve this already-retired payload for process lifetime;
        // the remaining live set is finite once new creates are rejected.
        retainRetiredPayloadForProcessLifetime(
            payload,
            "deferred-body-queue",
            s_retiredDeferredBodies.size());
    }

    void BethesdaPhysicsBody::serviceRetiredDeferredPayloads(
        RE::hknpWorld* currentWorld,
        std::uint32_t completedPhysicsSteps)
    {
        if (!currentWorld || completedPhysicsSteps == 0) {
            return;
        }

        std::scoped_lock lock(s_retiredDeferredBodyMutex);
        for (auto& retired : s_retiredDeferredBodies) {
            if (!retired.occupied()) {
                continue;
            }
            if (retired.processLifetimeHold) {
                continue;
            }
            if (retired.payload.retiredHknpWorld != currentWorld) {
                retired.processLifetimeHold = true;
                ROCK_LOG_SAMPLE_WARN(
                    BethesdaBody,
                    1000,
                    "Retired body {} belongs to a departed Havok world; retaining its complete native payload for process lifetime",
                    retired.payload.bodyId);
                continue;
            }

            retired.remainingPhysicsSteps =
                retired.remainingPhysicsSteps > completedPhysicsSteps ? retired.remainingPhysicsSteps - completedPhysicsSteps : 0;
            if (retired.remainingPhysicsSteps != 0) {
                continue;
            }

            const auto bodyId = retired.payload.bodyId;
            if (!quarantineRetiredPayload(retired.payload)) {
                retired.processLifetimeHold = true;
                continue;
            }
            retired = {};
            if (s_retiredDeferredBodyCount > 0) {
                --s_retiredDeferredBodyCount;
            }
            ROCK_LOG_SAMPLE_DEBUG(BethesdaBody,
                1000,
                "Retired deferred body {} transferred to collision-object quarantine activeRetired={}",
                bodyId,
                s_retiredDeferredBodyCount);
        }
    }

    void BethesdaPhysicsBody::reset()
    {
        _collisionObject = nullptr;
        _physicsSystem = nullptr;
        _systemData = nullptr;
        _niNode = nullptr;
        _createdHknpWorld = nullptr;
        _createdBhkWorld = nullptr;
        _bodyId.value = 0x7FFF'FFFF;
        _created = false;
    }

    bool BethesdaPhysicsBody::matchesCreationWorld(RE::hknpWorld* world, void* bhkWorld) const
    {
        if (!world || !bhkWorld ||
            world != _createdHknpWorld ||
            bhkWorld != _createdBhkWorld) {
            return false;
        }

        auto* instanceWorld = nativeWorldFromPhysicsSystem(_physicsSystem);
        if (instanceWorld != world) {
            return false;
        }

        auto* wrapperWorld =
            havok_runtime::getHknpWorldFromBhk(reinterpret_cast<RE::bhkWorld*>(bhkWorld));
        return wrapperWorld == world;
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

        detachAndReleaseNiNode(_collisionObject, _niNode);

        ROCK_LOG_DEBUG(BethesdaBody, "NiNode destroyed");
    }

    void BethesdaPhysicsBody::registerContactSignal(const char* signalName)
    {
        if (!isValid())
            return;

        ROCK_LOG_WARN(BethesdaBody,
            "registerContactSignal('{}') unavailable until the contact signal binding is verified — "
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
        // The native setter uses aligned SIMD loads; callers only promise floats.
        alignas(16) const float linear[4] = { linVel[0], linVel[1], linVel[2], linVel[3] };
        alignas(16) const float angular[4] = { angVel[0], angVel[1], angVel[2], angVel[3] };
        static REL::Relocation<SetVelocity_t> setVel{ REL::Offset(offsets::kFunc_CollisionObject_SetVelocity) };
        return setVel(_collisionObject, linear, angular) != 0;
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

    bool BethesdaPhysicsBody::refreshCollisionFilter(RE::hknpWorld* world)
    {
        if (!isValid() || !world || nativeWorldFromPhysicsSystem(_physicsSystem) != world) {
            return false;
        }
        const auto body = havok_runtime::snapshotBody(world, _bodyId);
        if (!body.valid || body.collisionObject != _collisionObject) {
            return false;
        }
        // The native filter setter skips unchanged filterInfo. A matrix-only
        // edit therefore needs the same explicit cache rebuild used by the
        // native player pair filter; do not toggle body flags to force it.
        using RebuildBodyCaches = void (*)(RE::hknpWorld*, std::uint32_t);
        static REL::Relocation<RebuildBodyCaches> rebuild{
            REL::Offset(offsets::kFunc_RebuildBodyCollisionCaches) };
        rebuild(world, _bodyId.value);
        return true;
    }

    void BethesdaPhysicsBody::setMass(float mass)
    {
        if (!isValid())
            return;
        static REL::Relocation<SetMass_t> setM{ REL::Offset(offsets::kFunc_CollisionObject_SetMass) };
        setM(_collisionObject, mass);
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

    bool BethesdaPhysicsBody::setPointVelocity(const float* targetVel, const float* worldPoint)
    {
        (void)worldPoint;
        if (!isValid() || !targetVel) {
            return false;
        }

        /*
         * FO4VR point-velocity writes still need binary validation before ROCK
         * can expose a native point-specific call. Returning the existing
         * verified velocity-writer result keeps this method honest for callers:
         * it can fail, it mutates through a known Bethesda path, and it does not
         * pretend to apply an unverified world-point correction.
         */
        const float zeroAngular[4]{ 0.0f, 0.0f, 0.0f, 0.0f };
        return setVelocity(targetVel, zeroAngular);
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



#include "WeaponCollision.h"

#include "HavokOffsets.h"
#include "RockConfig.h"

#include <intrin.h>

#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hkReferencedObject.h"
#include "RE/Havok/hknpCapsuleShape.h"
#include "RE/Havok/hknpMotion.h"

#include "f4vr/PlayerNodes.h"

#include <string>

namespace frik::rock
{
    namespace
    {
        float matrixDeterminant(const RE::NiMatrix3& matrix)
        {
            return matrix.entry[0][0] * (matrix.entry[1][1] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][1]) -
                matrix.entry[0][1] * (matrix.entry[1][0] * matrix.entry[2][2] - matrix.entry[1][2] * matrix.entry[2][0]) +
                matrix.entry[0][2] * (matrix.entry[1][0] * matrix.entry[2][1] - matrix.entry[1][1] * matrix.entry[2][0]);
        }

        float bodyBasisDeterminant(const float* bodyFloats)
        {
            const float x0 = bodyFloats[0];
            const float x1 = bodyFloats[1];
            const float x2 = bodyFloats[2];
            const float y0 = bodyFloats[4];
            const float y1 = bodyFloats[5];
            const float y2 = bodyFloats[6];
            const float z0 = bodyFloats[8];
            const float z1 = bodyFloats[9];
            const float z2 = bodyFloats[10];

            return x0 * (y1 * z2 - y2 * z1) - y0 * (x1 * z2 - x2 * z1) + z0 * (x1 * y2 - x2 * y1);
        }

        RE::NiTransform makeIdentityTransform()
        {
            RE::NiTransform result{};
            result.rotate.entry[0][0] = 1.0f;
            result.rotate.entry[1][1] = 1.0f;
            result.rotate.entry[2][2] = 1.0f;
            result.scale = 1.0f;
            return result;
        }

        RE::NiTransform getBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            if (!world || bodyId.value == 0x7FFF'FFFF) {
                return result;
            }

            auto* bodyArray = world->GetBodyArray();
            auto* bodyFloats = reinterpret_cast<float*>(&bodyArray[bodyId.value]);
            result.rotate = havokRotationBlocksToNiMatrix(bodyFloats);
            result.translate.x = bodyFloats[12] * kHavokToGameScale;
            result.translate.y = bodyFloats[13] * kHavokToGameScale;
            result.translate.z = bodyFloats[14] * kHavokToGameScale;
            return result;
        }
    }

    static void shapeAddRef(const RE::hknpShape* shape)
    {
        if (!shape)
            return;
        auto* refCountDword = reinterpret_cast<volatile long*>(const_cast<char*>(reinterpret_cast<const char*>(shape)) + 0x08);
        for (;;) {
            long oldVal = *refCountDword;
            std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
            if (rc == 0xFFFF)
                return;
            long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(rc + 1));
            if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal)
                return;
        }
    }

    static void shapeRemoveRef(const RE::hknpShape* shape)
    {
        if (!shape)
            return;
        auto* refCountDword = reinterpret_cast<volatile long*>(const_cast<char*>(reinterpret_cast<const char*>(shape)) + 0x08);
        for (;;) {
            long oldVal = *refCountDword;
            std::uint16_t rc = static_cast<std::uint16_t>(oldVal & 0xFFFF);
            if (rc == 0xFFFF || rc == 0)
                return;
            long newVal = (oldVal & static_cast<long>(0xFFFF0000u)) | static_cast<long>(static_cast<std::uint16_t>(rc - 1));
            if (_InterlockedCompareExchange(refCountDword, newVal, oldVal) == oldVal)
                return;
        }
    }

    WeaponCollision::WeaponCollision() { clearAtomicBodyIds(); }

    bool WeaponCollision::hasWeaponBody() const
    {
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                return true;
            }
        }
        return false;
    }

    std::uint32_t WeaponCollision::getWeaponBodyCount() const
    {
        std::uint32_t count = 0;
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                ++count;
            }
        }
        return count;
    }

    RE::hknpBodyId WeaponCollision::getWeaponBodyId() const
    {
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                return instance.body.getBodyId();
            }
        }
        return RE::hknpBodyId{ INVALID_BODY_ID };
    }

    std::uint32_t WeaponCollision::getWeaponBodyIdAtomic() const
    {
        if (_weaponBodyCountAtomic.load(std::memory_order_acquire) == 0) {
            return INVALID_BODY_ID;
        }
        return _weaponBodyIdsAtomic[0].load(std::memory_order_acquire);
    }

    bool WeaponCollision::isWeaponBodyIdAtomic(std::uint32_t bodyId) const
    {
        if (bodyId == INVALID_BODY_ID) {
            return false;
        }

        const std::uint32_t count = _weaponBodyCountAtomic.load(std::memory_order_acquire);
        for (std::uint32_t i = 0; i < count && i < MAX_WEAPON_BODIES; ++i) {
            if (_weaponBodyIdsAtomic[i].load(std::memory_order_acquire) == bodyId) {
                return true;
            }
        }
        return false;
    }

    BethesdaPhysicsBody& WeaponCollision::getWeaponBody()
    {
        for (auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                return instance.body;
            }
        }
        return _weaponBodies[0].body;
    }

    void WeaponCollision::init(RE::hknpWorld* world, void* bhkWorld)
    {
        if (!g_rockConfig.rockWeaponCollisionEnabled) {
            ROCK_LOG_INFO(Weapon, "WeaponCollision disabled via config — skipping init");
            return;
        }
        _cachedWorld = world;
        _cachedBhkWorld = bhkWorld;
        _cachedWeaponKey = 0;
        _weaponBodyPending = false;
        _retryCounter = 0;
        clearAtomicBodyIds();
        ROCK_LOG_INFO(Weapon, "WeaponCollision initialized");
    }

    void WeaponCollision::shutdown()
    {
        resetWeaponBodiesWithoutDestroy();

        _cachedWeaponKey = 0;
        _cachedWorld = nullptr;
        _cachedBhkWorld = nullptr;
        _weaponBodyPending = false;
        _retryCounter = 0;
        _dominantHandDisabled = false;
        _dominantHandCollisionWasDisabled = false;
        _disabledHandBodyId.value = INVALID_BODY_ID;

        ROCK_LOG_INFO(Weapon, "WeaponCollision shutdown");
    }

    void WeaponCollision::update(RE::hknpWorld* world, RE::NiAVObject* weaponNode, RE::hknpBodyId dominantHandBodyId, float dt)
    {
        if (!g_rockConfig.rockWeaponCollisionEnabled)
            return;
        if (!world)
            return;

        if (world != _cachedWorld) {
            ROCK_LOG_INFO(Weapon, "hknpWorld changed — resetting weapon collision state");
            resetWeaponBodiesWithoutDestroy();
            _cachedWeaponKey = 0;
            _weaponBodyPending = false;
            _cachedWorld = world;
        }

        std::uint64_t currentKey = getEquippedWeaponKey();

        if (currentKey != _cachedWeaponKey) {
            ROCK_LOG_INFO(Weapon, "Weapon changed: {:016X} -> {:016X}", _cachedWeaponKey, currentKey);

            if (hasWeaponBody()) {
                destroyWeaponBody(world);
            }

            _cachedWeaponKey = currentKey;
            _weaponBodyPending = (currentKey != 0);
        }

        if (!weaponNode) {
            if (hasWeaponBody()) {
                ROCK_LOG_INFO(Weapon, "Weapon node gone (holstered?) — destroying weapon bodies");
                destroyWeaponBody(world);
            }
            if (_dominantHandDisabled) {
                enableDominantHandCollision(world);
            }
            return;
        }

        if (hasWeaponBody() && !sourceBodiesStillValid(world)) {
            ROCK_LOG_WARN(Weapon, "Weapon source body changed/invalid — rebuilding mirrored weapon bodies");
            destroyWeaponBody(world);
            _weaponBodyPending = true;
            _retryCounter = 0;
        }

        if (_weaponBodyPending) {
            const bool shouldAttemptCreate = (++_retryCounter >= 90) || (_retryCounter == 1);
            if (shouldAttemptCreate) {
                if (_retryCounter >= 90) {
                    _retryCounter = 0;
                }

                WeaponShapeSourceList sources{};
                const std::size_t sourceCount = findWeaponShapeSources(weaponNode, world, sources);
                if (sourceCount > 0) {
                    createWeaponBodies(world, sources, sourceCount);
                    _weaponBodyPending = !hasWeaponBody();
                    _retryCounter = 0;
                }
            }
        }

        if (hasWeaponBody() && !_dominantHandDisabled && dominantHandBodyId.value != INVALID_BODY_ID) {
            disableDominantHandCollision(world, dominantHandBodyId);
        } else if (!hasWeaponBody() && _dominantHandDisabled) {
            enableDominantHandCollision(world);
        }

        if (hasWeaponBody() && weaponNode) {
            for (std::size_t i = 0; i < _weaponBodies.size(); ++i) {
                auto& instance = _weaponBodies[i];
                if (!instance.body.isValid()) {
                    continue;
                }
                const RE::NiTransform sourceTransform = getSourceBodyTransform(world, instance, weaponNode->world);
                updateBodyTransform(world, instance, sourceTransform, dt, i);
            }
        }
    }

    std::uint64_t WeaponCollision::getEquippedWeaponKey() const
    {
        auto* playerNodes = f4vr::getPlayerNodes();
        if (!playerNodes)
            return 0;

        auto* weaponNode = playerNodes->primaryWeapontoWeaponNode;
        if (!weaponNode)
            return 0;

        return reinterpret_cast<std::uint64_t>(weaponNode);
    }

    std::size_t WeaponCollision::findWeaponShapeSources(RE::NiAVObject* weaponNode, RE::hknpWorld* world, WeaponShapeSourceList& outSources)
    {
        if (!weaponNode || !world)
            return 0;

        const char* rootName = weaponNode->name.c_str();
        auto* rootNiNode = weaponNode->IsNode();
        const int childCount = rootNiNode ? static_cast<int>(rootNiNode->GetRuntimeData().children.size()) : 0;
        const bool hasCollision = (weaponNode->collisionObject.get() != nullptr);
        ROCK_LOG_INFO(Weapon, "findWeaponShapeSources: root='{}' children={} hasCollision={} addr={:x}", rootName ? rootName : "(null)", childCount, hasCollision,
            reinterpret_cast<std::uintptr_t>(weaponNode));

        if (rootNiNode) {
            auto& kids = rootNiNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < kids.size() && i < 8; i++) {
                auto* kid = kids[i].get();
                if (!kid) {
                    continue;
                }

                const char* kidName = kid->name.c_str();
                const bool kidCol = (kid->collisionObject.get() != nullptr);
                auto* kidNode = kid->IsNode();
                const int kidChildren = kidNode ? static_cast<int>(kidNode->GetRuntimeData().children.size()) : 0;
                ROCK_LOG_INFO(Weapon, "  child[{}]: '{}' children={} hasCollision={}", i, kidName ? kidName : "(null)", kidChildren, kidCol);
            }
        }

        std::size_t sourceCount = 0;
        findWeaponShapeSourcesRecursive(weaponNode, world, 0, outSources, sourceCount);
        if (sourceCount == 0) {
            ROCK_LOG_INFO(Weapon, "NO collision source bodies found on weapon node tree (searched from '{}')", rootName ? rootName : "(null)");
        } else {
            ROCK_LOG_INFO(Weapon, "Found {} weapon collision source bod{}", sourceCount, sourceCount == 1 ? "y" : "ies");
        }
        return sourceCount;
    }

    void WeaponCollision::findWeaponShapeSourcesRecursive(RE::NiAVObject* node, RE::hknpWorld* world, int depth, WeaponShapeSourceList& outSources, std::size_t& sourceCount)
    {
        if (!node || !world || depth > 15 || sourceCount >= MAX_WEAPON_BODIES)
            return;

        auto* colObj = node->collisionObject.get();
        if (colObj) {
            ROCK_LOG_INFO(Weapon, "{}colObj found on '{}' at depth {}", std::string(depth * 2, ' '), node->name.c_str(), depth);

            auto* fieldAt20 = *reinterpret_cast<void**>(reinterpret_cast<char*>(colObj) + offsets::kCollisionObject_PhysSystemPtr);
            if (fieldAt20 && reinterpret_cast<std::uintptr_t>(fieldAt20) > 0x10000) {
                auto* physSystem = reinterpret_cast<RE::bhkPhysicsSystem*>(fieldAt20);
                auto* inst = physSystem->instance;
                if (inst && reinterpret_cast<std::uintptr_t>(inst) > 0x10000 && inst->bodyIds) {
                    ROCK_LOG_INFO(Weapon, "{}physSystem OK, bodyCount={}", std::string(depth * 2, ' '), inst->bodyCount);
                    if (inst->bodyCount > 0 && inst->bodyCount < 128) {
                        auto* sourceNode = getOwnerNodeFromCollisionObject(colObj);
                        if (!sourceNode) {
                            sourceNode = node;
                        }
                        for (std::int32_t i = 0; i < inst->bodyCount && sourceCount < MAX_WEAPON_BODIES; ++i) {
                            RE::hknpBodyId bodyId{ inst->bodyIds[i] };
                            addWeaponShapeSource(world, outSources, sourceCount, bodyId, sourceNode, depth);
                        }
                    }
                } else {
                    ROCK_LOG_INFO(Weapon, "{}instance invalid or null", std::string(depth * 2, ' '));
                }
            } else {
                ROCK_LOG_INFO(Weapon, "{}+0x20 field invalid (proxy?)", std::string(depth * 2, ' '));
            }
        }

        auto* niNode = node->IsNode();
        if (niNode) {
            auto& kids = niNode->GetRuntimeData().children;
            for (std::uint16_t i = 0; i < kids.size() && sourceCount < MAX_WEAPON_BODIES; i++) {
                auto* kid = kids[i].get();
                if (kid) {
                    findWeaponShapeSourcesRecursive(kid, world, depth + 1, outSources, sourceCount);
                }
            }
        }
    }

    bool WeaponCollision::addWeaponShapeSource(RE::hknpWorld* world, WeaponShapeSourceList& outSources, std::size_t& sourceCount, RE::hknpBodyId sourceBodyId,
        RE::NiAVObject* sourceNode, int depth)
    {
        if (!world || sourceBodyId.value == INVALID_BODY_ID || sourceCount >= MAX_WEAPON_BODIES) {
            return false;
        }

        for (std::size_t i = 0; i < sourceCount; ++i) {
            if (outSources[i].sourceBodyId.value == sourceBodyId.value) {
                return false;
            }
        }

        auto* bodyArray = world->GetBodyArray();
        auto& body = bodyArray[sourceBodyId.value];
        if (!body.shape) {
            return false;
        }

        outSources[sourceCount] = WeaponShapeSource{ body.shape, sourceBodyId, sourceNode };
        ROCK_LOG_INFO(Weapon, "{}source[{}]: bodyId={} shape={:x} flags=0x{:X} owner='{}'", std::string(depth * 2, ' '), sourceCount, sourceBodyId.value,
            reinterpret_cast<std::uintptr_t>(body.shape), body.flags, sourceNode ? sourceNode->name.c_str() : "(null)");
        ++sourceCount;
        return true;
    }

    bool WeaponCollision::sourceBodiesStillValid(RE::hknpWorld* world) const
    {
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid() && !isSourceBodyValid(world, instance)) {
                return false;
            }
        }
        return true;
    }

    bool WeaponCollision::isSourceBodyValid(RE::hknpWorld* world, const WeaponBodyInstance& instance) const
    {
        if (!world || !instance.shape || instance.sourceBodyId.value == INVALID_BODY_ID) {
            return false;
        }

        auto* bodyArray = world->GetBodyArray();
        auto& body = bodyArray[instance.sourceBodyId.value];
        return body.bodyId.value == instance.sourceBodyId.value && body.shape == instance.shape;
    }

    RE::NiTransform WeaponCollision::getSourceBodyTransform(RE::hknpWorld* world, const WeaponBodyInstance& instance, const RE::NiTransform& fallbackTransform) const
    {
        if (isSourceBodyValid(world, instance)) {
            return getBodyWorldTransform(world, instance.sourceBodyId);
        }
        if (instance.sourceNode) {
            return instance.sourceNode->world;
        }
        return fallbackTransform;
    }

    void WeaponCollision::createWeaponBodies(RE::hknpWorld* world, const WeaponShapeSourceList& sources, std::size_t sourceCount)
    {
        if (hasWeaponBody()) {
            ROCK_LOG_WARN(Weapon, "createWeaponBodies called but bodies already exist — skipping");
            return;
        }
        if (!world || !_cachedBhkWorld || sourceCount == 0)
            return;

        std::size_t createdCount = 0;
        const std::uint32_t filterInfo = (0x000B << 16) | (ROCK_WEAPON_LAYER & 0x7F);
        for (std::size_t sourceIndex = 0; sourceIndex < sourceCount && createdCount < MAX_WEAPON_BODIES; ++sourceIndex) {
            const auto& source = sources[sourceIndex];
            if (!source.shape || source.sourceBodyId.value == INVALID_BODY_ID) {
                continue;
            }

            auto& instance = _weaponBodies[createdCount];
            shapeAddRef(source.shape);
            instance.shape = source.shape;
            instance.sourceBodyId = source.sourceBodyId;
            instance.sourceNode = source.sourceNode;
            instance.hasPrevTransform = false;
            instance.prevPosition = {};

            const bool ok =
                instance.body.create(world, _cachedBhkWorld, const_cast<RE::hknpShape*>(source.shape), filterInfo, { 0 }, BethesdaMotionType::Keyframed, "ROCK_WeaponCollision");

            if (!ok) {
                ROCK_LOG_ERROR(Weapon, "BethesdaPhysicsBody::create failed for weapon collision source bodyId={}", source.sourceBodyId.value);
                shapeRemoveRef(source.shape);
                clearWeaponBodyInstance(instance, false);
                continue;
            }

            instance.body.createNiNode("ROCK_WeaponCollision");
            const RE::NiTransform initialTransform = getSourceBodyTransform(world, instance, source.sourceNode ? source.sourceNode->world : makeIdentityTransform());
            updateBodyTransform(world, instance, initialTransform, 0.016f, createdCount);

            ROCK_LOG_INFO(Weapon, "Weapon collision body created — mirrorIndex={} bodyId={} sourceBodyId={} layer=44", createdCount, instance.body.getBodyId().value,
                source.sourceBodyId.value);
            ++createdCount;
        }

        publishAtomicBodyIds();
        ROCK_LOG_INFO(Weapon, "Weapon collision mirror created {}/{} source bodies", createdCount, sourceCount);
    }

    void WeaponCollision::destroyWeaponBody(RE::hknpWorld* world)
    {
        if (!hasWeaponBody())
            return;

        (void)world;
        clearAtomicBodyIds();

        std::uint32_t destroyedCount = 0;
        for (auto& instance : _weaponBodies) {
            if (instance.body.isValid()) {
                instance.body.destroy(_cachedBhkWorld);
                ++destroyedCount;
            }
            clearWeaponBodyInstance(instance, true);
        }

        ROCK_LOG_INFO(Weapon, "Weapon collision bodies destroyed count={}", destroyedCount);
    }

    void WeaponCollision::resetWeaponBodiesWithoutDestroy()
    {
        clearAtomicBodyIds();
        for (auto& instance : _weaponBodies) {
            instance.body.reset();
            clearWeaponBodyInstance(instance, true);
        }
    }

    void WeaponCollision::clearWeaponBodyInstance(WeaponBodyInstance& instance, bool releaseShapeRef)
    {
        if (releaseShapeRef && instance.shape) {
            shapeRemoveRef(instance.shape);
        }
        instance.body.reset();
        instance.shape = nullptr;
        instance.sourceBodyId.value = INVALID_BODY_ID;
        instance.sourceNode = nullptr;
        instance.hasPrevTransform = false;
        instance.prevPosition = {};
    }

    void WeaponCollision::clearAtomicBodyIds()
    {
        _weaponBodyCountAtomic.store(0, std::memory_order_release);
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
    }

    void WeaponCollision::publishAtomicBodyIds()
    {
        std::uint32_t count = 0;
        for (auto& id : _weaponBodyIdsAtomic) {
            id.store(INVALID_BODY_ID, std::memory_order_release);
        }
        for (const auto& instance : _weaponBodies) {
            if (instance.body.isValid() && count < MAX_WEAPON_BODIES) {
                _weaponBodyIdsAtomic[count].store(instance.body.getBodyId().value, std::memory_order_release);
                ++count;
            }
        }
        _weaponBodyCountAtomic.store(count, std::memory_order_release);
    }

    void WeaponCollision::updateBodyTransform(RE::hknpWorld* world, WeaponBodyInstance& instance, const RE::NiTransform& weaponTransform, float dt, std::size_t bodyIndex)
    {
        if (!instance.body.isValid() || !world)
            return;

        const float targetX = weaponTransform.translate.x * kGameToHavokScale;
        const float targetY = weaponTransform.translate.y * kGameToHavokScale;
        const float targetZ = weaponTransform.translate.z * kGameToHavokScale;
        const bool logWitness = bodyIndex == 0 && g_rockConfig.rockDebugVerboseLogging && (++_posLogCounter >= 90);
        if (logWitness) {
            _posLogCounter = 0;
        }
        auto bodyId = instance.body.getBodyId();
        auto* bodyArray = world->GetBodyArray();
        auto* bodyFloats = reinterpret_cast<float*>(&bodyArray[bodyId.value]);
        const float curX = bodyFloats[12];
        const float curY = bodyFloats[13];
        const float curZ = bodyFloats[14];

        if (logWitness) {
            ROCK_LOG_INFO(Weapon, "updateBodyTransform[{}]: sourceBodyId={} niPos=({:.1f},{:.1f},{:.1f}) hkPos=({:.4f},{:.4f},{:.4f}) bodyId={} dt={:.4f}", bodyIndex,
                instance.sourceBodyId.value, weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z, targetX, targetY, targetZ,
                instance.body.getBodyId().value, dt);
        }

        if (instance.hasPrevTransform) {
            float dx = weaponTransform.translate.x - instance.prevPosition.x;
            float dy = weaponTransform.translate.y - instance.prevPosition.y;
            float dz = weaponTransform.translate.z - instance.prevPosition.z;
            float distSq = dx * dx + dy * dy + dz * dz;
            if (distSq > 1000.0f * 1000.0f) {
                instance.prevPosition = weaponTransform.translate;
                RE::hkTransformf hkTransform;
                hkTransform.rotation = weaponTransform.rotate;
                hkTransform.translation = RE::NiPoint4(targetX, targetY, targetZ, 0.0f);
                instance.body.setTransform(hkTransform);
                if (logWitness) {
                    ROCK_LOG_INFO(Weapon,
                        "transform witness: bodyId={} niPos=({:.1f},{:.1f},{:.1f}) hkTarget=({:.4f},{:.4f},{:.4f}) "
                        "liveOrigin=({:.4f},{:.4f},{:.4f}) teleport=world-reset targetDet={:.4f} liveDet={:.4f}",
                        bodyId.value, weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z, targetX, targetY, targetZ, curX, curY, curZ,
                        matrixDeterminant(weaponTransform.rotate), bodyBasisDeterminant(bodyFloats));
                }
                return;
            }
        }

        alignas(16) float linVelOut[4] = { 0, 0, 0, 0 };
        alignas(16) float angVelOut[4] = { 0, 0, 0, 0 };

        if (instance.hasPrevTransform && dt > 0.0001f) {
            alignas(16) float tgtPos[4] = { targetX, targetY, targetZ, 0.0f };
            alignas(16) float tgtQuat[4];
            niRotToHkQuat(weaponTransform.rotate, tgtQuat);

            typedef void (*computeHKF_t)(void*, std::uint32_t, const float*, const float*, float, float*, float*);
            static REL::Relocation<computeHKF_t> computeHardKeyFrame{ REL::Offset(offsets::kFunc_ComputeHardKeyFrame) };
            computeHardKeyFrame(world, bodyId.value, tgtPos, tgtQuat, dt, linVelOut, angVelOut);

            constexpr float MAX_VEL = 50.0f;
            float speed = std::sqrt(linVelOut[0] * linVelOut[0] + linVelOut[1] * linVelOut[1] + linVelOut[2] * linVelOut[2]);
            if (speed > MAX_VEL) {
                float scale = MAX_VEL / speed;
                linVelOut[0] *= scale;
                linVelOut[1] *= scale;
                linVelOut[2] *= scale;
            }

            constexpr float MAX_ANG_VEL = 100.0f;
            float angSpeed = std::sqrt(angVelOut[0] * angVelOut[0] + angVelOut[1] * angVelOut[1] + angVelOut[2] * angVelOut[2]);
            if (angSpeed > MAX_ANG_VEL) {
                float scale = MAX_ANG_VEL / angSpeed;
                angVelOut[0] *= scale;
                angVelOut[1] *= scale;
                angVelOut[2] *= scale;
            }
        }

        {
            RE::hkTransformf hkTransform;
            hkTransform.rotation = niRotToHkTransformRotation(weaponTransform.rotate);
            hkTransform.translation = RE::NiPoint4(targetX, targetY, targetZ, 0.0f);
            instance.body.setTransform(hkTransform);
        }
        instance.body.setVelocity(linVelOut, angVelOut);

        if (logWitness) {
            const auto targetQuat = niRotToHkQuat(weaponTransform.rotate);
            const float targetDet = matrixDeterminant(weaponTransform.rotate);
            const float liveDet = bodyBasisDeterminant(bodyFloats);

            ROCK_LOG_INFO(Weapon,
                "transform witness: bodyId={} niPos=({:.1f},{:.1f},{:.1f}) hkTarget=({:.4f},{:.4f},{:.4f}) "
                "liveOrigin=({:.4f},{:.4f},{:.4f}) targetDet={:.4f} liveDet={:.4f} dt={:.4f}",
                bodyId.value, weaponTransform.translate.x, weaponTransform.translate.y, weaponTransform.translate.z, targetX, targetY, targetZ, curX, curY, curZ, targetDet,
                liveDet, dt);

            ROCK_LOG_INFO(Weapon,
                "transform basis: targetRows=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "liveRows=[({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f}) ({:.4f},{:.4f},{:.4f})] "
                "quat=({:.4f},{:.4f},{:.4f},{:.4f}) linVel=({:.4f},{:.4f},{:.4f}) angVel=({:.4f},{:.4f},{:.4f})",
                weaponTransform.rotate.entry[0][0], weaponTransform.rotate.entry[0][1], weaponTransform.rotate.entry[0][2], weaponTransform.rotate.entry[1][0],
                weaponTransform.rotate.entry[1][1], weaponTransform.rotate.entry[1][2], weaponTransform.rotate.entry[2][0], weaponTransform.rotate.entry[2][1],
                weaponTransform.rotate.entry[2][2], bodyFloats[0], bodyFloats[1], bodyFloats[2], bodyFloats[4], bodyFloats[5], bodyFloats[6], bodyFloats[8], bodyFloats[9],
                bodyFloats[10], targetQuat.x, targetQuat.y, targetQuat.z, targetQuat.w, linVelOut[0], linVelOut[1], linVelOut[2], angVelOut[0], angVelOut[1], angVelOut[2]);
        }

        instance.prevPosition = weaponTransform.translate;
        instance.hasPrevTransform = true;
    }

    void WeaponCollision::disableDominantHandCollision(RE::hknpWorld* world, RE::hknpBodyId handBodyId)
    {
        if (!world || handBodyId.value == INVALID_BODY_ID)
            return;

        typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
        static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{ REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

        auto* bodyArray = world->GetBodyArray();
        auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[handBodyId.value]);
        auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
        _dominantHandCollisionWasDisabled = (currentFilter & (1u << 14)) != 0;
        auto newFilter = currentFilter | (1u << 14);
        setBodyCollisionFilterInfo(world, handBodyId.value, newFilter, 0);

        _dominantHandDisabled = true;
        _disabledHandBodyId = handBodyId;

        ROCK_LOG_INFO(Weapon, "Dominant hand collision DISABLED (bit 14) bodyId={} filter=0x{:08X}", handBodyId.value, newFilter);
    }

    void WeaponCollision::enableDominantHandCollision(RE::hknpWorld* world)
    {
        if (!world || _disabledHandBodyId.value == INVALID_BODY_ID)
            return;

        typedef void setCollisionFilter_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
        static REL::Relocation<setCollisionFilter_t> setBodyCollisionFilterInfo{ REL::Offset(offsets::kFunc_SetBodyCollisionFilterInfo) };

        auto* bodyArray = world->GetBodyArray();
        auto* handBodyPtr = reinterpret_cast<char*>(&bodyArray[_disabledHandBodyId.value]);
        auto currentFilter = *reinterpret_cast<std::uint32_t*>(handBodyPtr + offsets::kBody_CollisionFilterInfo);
        auto newFilter = _dominantHandCollisionWasDisabled ? (currentFilter | (1u << 14)) : (currentFilter & ~(1u << 14));
        setBodyCollisionFilterInfo(world, _disabledHandBodyId.value, newFilter, 0);

        ROCK_LOG_INFO(Weapon, "Dominant hand collision RE-ENABLED bodyId={} filter=0x{:08X}", _disabledHandBodyId.value, newFilter);

        _dominantHandDisabled = false;
        _dominantHandCollisionWasDisabled = false;
        _disabledHandBodyId.value = INVALID_BODY_ID;
    }

}

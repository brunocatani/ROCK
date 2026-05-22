#include "physics-interaction/collision/CollisionSuppressionRegistry.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/PhysicsLog.h"

namespace rock::collision_suppression_registry
{
    namespace
    {
        std::uint32_t leaseCount(std::uint32_t ownerMask)
        {
            std::uint32_t count = 0;
            while (ownerMask != 0) {
                count += ownerMask & 1u;
                ownerMask >>= 1u;
            }
            return count;
        }

        const char* ownerName(CollisionSuppressionOwner owner)
        {
            switch (owner) {
            case CollisionSuppressionOwner::Grab:
                return "Grab";
            case CollisionSuppressionOwner::WeaponDominantHand:
                return "WeaponDominantHand";
            case CollisionSuppressionOwner::WeaponSupportHand:
                return "WeaponSupportHand";
            case CollisionSuppressionOwner::NativePlayerBody:
                return "NativePlayerBody";
            }
            return "Unknown";
        }
    }

    CollisionSuppressionRegistry::RuntimeEntry* CollisionSuppressionRegistry::find(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        const auto it = std::find_if(_entries.begin(), _entries.end(), [&](const RuntimeEntry& entry) {
            return entry.world == world && entry.bodyId == bodyId;
        });
        return it != _entries.end() ? &*it : nullptr;
    }

    const CollisionSuppressionRegistry::RuntimeEntry* CollisionSuppressionRegistry::find(RE::hknpWorld* world, std::uint32_t bodyId) const
    {
        const auto it = std::find_if(_entries.begin(), _entries.end(), [&](const RuntimeEntry& entry) {
            return entry.world == world && entry.bodyId == bodyId;
        });
        return it != _entries.end() ? &*it : nullptr;
    }

    void CollisionSuppressionRegistry::erase(RE::hknpWorld* world, std::uint32_t bodyId)
    {
        _entries.erase(std::remove_if(_entries.begin(), _entries.end(), [&](const RuntimeEntry& entry) {
            return entry.world == world && entry.bodyId == bodyId;
        }), _entries.end());
    }

    bool CollisionSuppressionRegistry::bodyIdentityMatches(
        const RuntimeEntry& entry,
        std::uint32_t motionIndex,
        RE::NiCollisionObject* collisionObject,
        RE::NiAVObject* ownerNode)
    {
        return entry.motionIndex == motionIndex &&
               entry.collisionObject == collisionObject &&
               entry.ownerNode == ownerNode;
    }

    void CollisionSuppressionRegistry::captureBodyIdentity(
        RuntimeEntry& entry,
        RE::hknpWorld* world,
        std::uint32_t bodyId,
        std::uint32_t motionIndex,
        RE::NiCollisionObject* collisionObject,
        RE::NiAVObject* ownerNode)
    {
        entry.world = world;
        entry.bodyId = bodyId;
        entry.motionIndex = motionIndex;
        entry.collisionObject = collisionObject;
        entry.ownerNode = ownerNode;
    }

    RuntimeSuppressionResult CollisionSuppressionRegistry::acquire(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context)
    {
        RuntimeSuppressionResult result{};
        result.bodyId = bodyId;
        if (!world || bodyId == kInvalidBodyId) {
            result.readFailed = true;
            return result;
        }

        const std::uint32_t ownerMask = ownerBit(owner);
        const auto snapshot = havok_runtime::snapshotBody(world, RE::hknpBodyId{ bodyId });
        if (!snapshot.valid) {
            result.readFailed = true;
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Collision suppression acquire failed: owner={} bodyId={} context={} cannot snapshot body",
                ownerName(owner),
                bodyId,
                context ? context : "");
            return result;
        }

        auto* entry = find(world, bodyId);
        if (entry && !bodyIdentityMatches(*entry, snapshot.motionIndex, snapshot.collisionObject, snapshot.ownerNode)) {
            ROCK_LOG_WARN(Hand,
                "Collision suppression stale lease discarded before acquire: owner={} bodyId={} context={} oldMotion={} newMotion={} oldOwner={} newOwner={} oldCollision={} newCollision={}",
                ownerName(owner),
                bodyId,
                context ? context : "",
                entry->motionIndex,
                snapshot.motionIndex,
                static_cast<const void*>(entry->ownerNode),
                static_cast<const void*>(snapshot.ownerNode),
                static_cast<const void*>(entry->collisionObject),
                static_cast<const void*>(snapshot.collisionObject));
            erase(world, bodyId);
            entry = nullptr;
        }

        if (entry && (entry->ownerMask & ownerMask) != 0) {
            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                result.readFailed = true;
                ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Collision suppression refresh failed: owner={} bodyId={} context={} cannot read filter",
                    ownerName(owner),
                    bodyId,
                    context ? context : "");
                return result;
            }

            const std::uint32_t refreshedFilter = currentFilter | kSuppressionNoCollideBit;
            if (refreshedFilter != currentFilter) {
                body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, refreshedFilter);
            }

            result.valid = true;
            result.ownerAlreadyHeld = true;
            result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
            result.activeLeaseCount = leaseCount(entry->ownerMask);
            result.filterBefore = currentFilter;
            result.filterAfter = refreshedFilter;
            result.filterChanged = refreshedFilter != currentFilter;
            return result;
        }

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
            result.readFailed = true;
            ROCK_LOG_SAMPLE_WARN(Hand, 1000, "Collision suppression acquire failed: owner={} bodyId={} context={} cannot read filter",
                ownerName(owner),
                bodyId,
                context ? context : "");
            return result;
        }

        if (!entry) {
            RuntimeEntry newEntry{};
            captureBodyIdentity(newEntry, world, bodyId, snapshot.motionIndex, snapshot.collisionObject, snapshot.ownerNode);
            newEntry.originalFilter = currentFilter;
            newEntry.wasNoCollideBeforeSuppression = (currentFilter & kSuppressionNoCollideBit) != 0;
            newEntry.ownerMask = ownerMask;
            _entries.push_back(newEntry);
            entry = &_entries.back();
            result.firstLeaseForBody = true;
        } else {
            entry->ownerMask |= ownerMask;
        }

        const std::uint32_t disabledFilter = currentFilter | kSuppressionNoCollideBit;
        if (disabledFilter != currentFilter) {
            body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, disabledFilter);
        }

        result.valid = true;
        result.filterBefore = currentFilter;
        result.filterAfter = disabledFilter;
        result.filterChanged = disabledFilter != currentFilter;
        result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
        result.activeLeaseCount = leaseCount(entry->ownerMask);

        if (result.firstLeaseForBody || result.filterChanged) {
            ROCK_LOG_DEBUG(Hand,
                "Collision suppression acquired: owner={} bodyId={} context={} filter=0x{:08X}->0x{:08X} preDisabled={} leases={}",
                ownerName(owner),
                bodyId,
                context ? context : "",
                currentFilter,
                disabledFilter,
                entry->wasNoCollideBeforeSuppression ? "yes" : "no",
                result.activeLeaseCount);
        }

        return result;
    }

    RuntimeSuppressionResult CollisionSuppressionRegistry::release(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context)
    {
        RuntimeSuppressionResult result{};
        result.bodyId = bodyId;
        if (!world) {
            result.readFailed = hasLease(bodyId, owner);
            return result;
        }

        auto* entry = find(world, bodyId);
        if (!entry) {
            return result;
        }

        const auto snapshot = havok_runtime::snapshotBody(world, RE::hknpBodyId{ bodyId });
        if (!snapshot.valid) {
            const auto originalOwnerMask = entry->ownerMask;
            result.valid = true;
            result.staleLeaseDiscarded = true;
            result.bodyFullyReleased = true;
            result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
            result.activeLeaseCount = 0;
            ROCK_LOG_WARN(Hand,
                "Collision suppression stale lease discarded on unreadable release: owner={} bodyId={} context={} ownerMask=0x{:08X}",
                ownerName(owner),
                bodyId,
                context ? context : "",
                originalOwnerMask);
            erase(world, bodyId);
            return result;
        }

        if (!bodyIdentityMatches(*entry, snapshot.motionIndex, snapshot.collisionObject, snapshot.ownerNode)) {
            const auto oldMotionIndex = entry->motionIndex;
            auto* oldOwnerNode = entry->ownerNode;
            auto* oldCollisionObject = entry->collisionObject;
            const auto originalOwnerMask = entry->ownerMask;
            result.valid = true;
            result.staleLeaseDiscarded = true;
            result.bodyFullyReleased = true;
            result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
            result.activeLeaseCount = 0;
            ROCK_LOG_WARN(Hand,
                "Collision suppression stale lease discarded on identity mismatch: owner={} bodyId={} context={} ownerMask=0x{:08X} oldMotion={} newMotion={} oldOwner={} newOwner={} oldCollision={} newCollision={}",
                ownerName(owner),
                bodyId,
                context ? context : "",
                originalOwnerMask,
                oldMotionIndex,
                snapshot.motionIndex,
                static_cast<const void*>(oldOwnerNode),
                static_cast<const void*>(snapshot.ownerNode),
                static_cast<const void*>(oldCollisionObject),
                static_cast<const void*>(snapshot.collisionObject));
            erase(world, bodyId);
            return result;
        }

        std::uint32_t currentFilter = 0;
        if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
            result.readFailed = true;
            ROCK_LOG_WARN(Hand,
                "Collision suppression release deferred: owner={} bodyId={} context={} cannot read filter; lease preserved",
                ownerName(owner),
                bodyId,
                context ? context : "");
            return result;
        }

        entry->ownerMask &= ~ownerBit(owner);
        result.valid = true;
        result.filterBefore = currentFilter;
        result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
        result.activeLeaseCount = leaseCount(entry->ownerMask);

        if (entry->ownerMask != 0) {
            const std::uint32_t keptFilter = currentFilter | kSuppressionNoCollideBit;
            if (keptFilter != currentFilter) {
                body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, keptFilter);
            }
            result.filterAfter = keptFilter;
            result.filterChanged = keptFilter != currentFilter;
            ROCK_LOG_DEBUG(Hand,
                "Collision suppression owner released: owner={} bodyId={} context={} leasesRemaining={} filter=0x{:08X}->0x{:08X}",
                ownerName(owner),
                bodyId,
                context ? context : "",
                result.activeLeaseCount,
                currentFilter,
                keptFilter);
            return result;
        }

        const std::uint32_t restoredFilter =
            entry->wasNoCollideBeforeSuppression ? (currentFilter | kSuppressionNoCollideBit) : (currentFilter & ~kSuppressionNoCollideBit);
        if (restoredFilter != currentFilter) {
            body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, restoredFilter);
        }
        result.bodyFullyReleased = true;
        result.filterAfter = restoredFilter;
        result.filterChanged = restoredFilter != currentFilter;

        ROCK_LOG_DEBUG(Hand,
            "Collision suppression fully released: owner={} bodyId={} context={} filter=0x{:08X}->0x{:08X} restoreDisabled={}",
            ownerName(owner),
            bodyId,
            context ? context : "",
            currentFilter,
            restoredFilter,
            entry->wasNoCollideBeforeSuppression ? "yes" : "no");

        erase(world, bodyId);
        return result;
    }

    void CollisionSuppressionRegistry::releaseOwner(RE::hknpWorld* world, CollisionSuppressionOwner owner, const char* context)
    {
        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(_entries.size());
        for (const auto& entry : _entries) {
            if ((entry.ownerMask & ownerBit(owner)) != 0) {
                bodyIds.push_back(entry.bodyId);
            }
        }

        for (const auto bodyId : bodyIds) {
            release(world, bodyId, owner, context);
        }
    }

    bool CollisionSuppressionRegistry::hasLease(std::uint32_t bodyId, CollisionSuppressionOwner owner) const
    {
        const auto it = std::find_if(_entries.begin(), _entries.end(), [&](const RuntimeEntry& entry) {
            return entry.bodyId == bodyId && (entry.ownerMask & ownerBit(owner)) != 0;
        });
        return it != _entries.end();
    }

    void CollisionSuppressionRegistry::clear() { _entries.clear(); }

    CollisionSuppressionRegistry& globalCollisionSuppressionRegistry()
    {
        static CollisionSuppressionRegistry registry;
        return registry;
    }
}

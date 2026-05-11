#pragma once

    /*
     * Hand/weapon collision suppression is a shared ownership problem, not a local
     * toggle. Grab, dominant-weapon handling, and support grip can all suppress the
     * same hand body in the same frame. This registry models suppression as leases:
     * the first owner captures the original filter, later owners join the lease, and
     * the filter state is restored only after the last owner releases. Suppression is
     * filter-only by design because FO4VR's native body deactivation table is not
     * guaranteed for ROCK-generated bodies.
     */

#include <algorithm>
#include <cstdint>
#include <vector>

namespace RE
{
    class hknpWorld;
}

namespace rock::collision_suppression_registry
{
    inline constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
    inline constexpr std::uint32_t kSuppressionNoCollideBit = 1u << 14;

    enum class CollisionSuppressionOwner : std::uint8_t
    {
        Grab = 0,
        WeaponDominantHand = 1,
        WeaponSupportHand = 2,
    };

    inline constexpr std::uint32_t ownerBit(CollisionSuppressionOwner owner) { return 1u << static_cast<std::uint32_t>(owner); }

    struct SuppressionLeaseResult
    {
        bool valid = false;
        bool firstLeaseForBody = false;
        bool bodyFullyReleased = false;
        bool filterChanged = false;
        bool ownerAlreadyHeld = false;
        bool wasNoCollideBeforeSuppression = false;
        std::uint32_t bodyId = kInvalidBodyId;
        std::uint32_t filterBefore = 0;
        std::uint32_t filterAfter = 0;
        std::uint32_t activeLeaseCount = 0;
    };

    class PureCollisionSuppressionRegistry
    {
    public:
        SuppressionLeaseResult acquire(std::uint32_t bodyId, CollisionSuppressionOwner owner, std::uint32_t currentFilter)
        {
            SuppressionLeaseResult result{};
            result.bodyId = bodyId;
            result.filterBefore = currentFilter;
            result.filterAfter = currentFilter | kSuppressionNoCollideBit;
            result.filterChanged = result.filterAfter != currentFilter;
            if (bodyId == kInvalidBodyId) {
                return result;
            }

            const std::uint32_t bit = ownerBit(owner);
            auto* entry = find(bodyId);
            if (!entry) {
                BodyEntry newEntry{};
                newEntry.bodyId = bodyId;
                newEntry.originalFilter = currentFilter;
                newEntry.wasNoCollideBeforeSuppression = (currentFilter & kSuppressionNoCollideBit) != 0;
                newEntry.ownerMask = bit;
                _entries.push_back(newEntry);
                entry = &_entries.back();
                result.firstLeaseForBody = true;
            } else if ((entry->ownerMask & bit) != 0) {
                result.valid = true;
                result.ownerAlreadyHeld = true;
                result.filterAfter = currentFilter;
                result.filterChanged = false;
                result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
                result.activeLeaseCount = activeLeaseCount(bodyId);
                return result;
            } else {
                entry->ownerMask |= bit;
            }

            result.valid = true;
            result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
            result.activeLeaseCount = activeLeaseCount(bodyId);
            return result;
        }

        SuppressionLeaseResult release(std::uint32_t bodyId, CollisionSuppressionOwner owner, std::uint32_t currentFilter)
        {
            SuppressionLeaseResult result{};
            result.bodyId = bodyId;
            result.filterBefore = currentFilter;
            result.filterAfter = currentFilter;
            auto* entry = find(bodyId);
            if (!entry) {
                return result;
            }

            entry->ownerMask &= ~ownerBit(owner);
            result.valid = true;
            result.wasNoCollideBeforeSuppression = entry->wasNoCollideBeforeSuppression;
            result.activeLeaseCount = activeLeaseCount(bodyId);
            if (entry->ownerMask != 0) {
                result.filterAfter = currentFilter | kSuppressionNoCollideBit;
                result.filterChanged = result.filterAfter != currentFilter;
                return result;
            }

            result.bodyFullyReleased = true;
            result.filterAfter = entry->wasNoCollideBeforeSuppression ? (currentFilter | kSuppressionNoCollideBit) : (currentFilter & ~kSuppressionNoCollideBit);
            result.filterChanged = result.filterAfter != currentFilter;
            erase(bodyId);
            return result;
        }

        std::uint32_t activeLeaseCount(std::uint32_t bodyId) const
        {
            const auto* entry = find(bodyId);
            if (!entry) {
                return 0;
            }

            std::uint32_t count = 0;
            std::uint32_t mask = entry->ownerMask;
            while (mask != 0) {
                count += mask & 1u;
                mask >>= 1u;
            }
            return count;
        }

        bool hasBody(std::uint32_t bodyId) const { return find(bodyId) != nullptr; }

        void clear() { _entries.clear(); }

    private:
        struct BodyEntry
        {
            std::uint32_t bodyId = kInvalidBodyId;
            std::uint32_t originalFilter = 0;
            std::uint32_t ownerMask = 0;
            bool wasNoCollideBeforeSuppression = false;
        };

        BodyEntry* find(std::uint32_t bodyId)
        {
            const auto it = std::find_if(_entries.begin(), _entries.end(), [&](const BodyEntry& entry) {
                return entry.bodyId == bodyId;
            });
            return it != _entries.end() ? &*it : nullptr;
        }

        const BodyEntry* find(std::uint32_t bodyId) const
        {
            const auto it = std::find_if(_entries.begin(), _entries.end(), [&](const BodyEntry& entry) {
                return entry.bodyId == bodyId;
            });
            return it != _entries.end() ? &*it : nullptr;
        }

        void erase(std::uint32_t bodyId)
        {
            _entries.erase(std::remove_if(_entries.begin(), _entries.end(), [&](const BodyEntry& entry) {
                return entry.bodyId == bodyId;
            }), _entries.end());
        }

        std::vector<BodyEntry> _entries;
    };

    struct RuntimeSuppressionResult : SuppressionLeaseResult
    {
        bool readFailed = false;
    };

    class CollisionSuppressionRegistry
    {
    public:
        RuntimeSuppressionResult acquire(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context);
        RuntimeSuppressionResult release(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context);
        void releaseOwner(RE::hknpWorld* world, CollisionSuppressionOwner owner, const char* context);
        bool hasLease(std::uint32_t bodyId, CollisionSuppressionOwner owner) const;
        void clear();

    private:
        struct RuntimeEntry
        {
            std::uint32_t bodyId = kInvalidBodyId;
            std::uint32_t originalFilter = 0;
            std::uint32_t ownerMask = 0;
            bool wasNoCollideBeforeSuppression = false;
        };

        RuntimeEntry* find(std::uint32_t bodyId);
        const RuntimeEntry* find(std::uint32_t bodyId) const;
        void erase(std::uint32_t bodyId);

        std::vector<RuntimeEntry> _entries;
    };

    CollisionSuppressionRegistry& globalCollisionSuppressionRegistry();
}

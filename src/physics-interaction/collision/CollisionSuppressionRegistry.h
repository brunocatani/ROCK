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
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

namespace RE
{
    class hknpWorld;
    class NiAVObject;
    class NiCollisionObject;
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
        NativePlayerBody = 3,
        HeldLooseWeaponBody = 4,
        EquippedWeaponDropHand = 5,
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
        bool staleLeaseDiscarded = false;
        bool leaseIdentityValid = false;
        std::uint32_t leaseMotionIndex = 0;
        RE::NiCollisionObject* leaseCollisionObject = nullptr;
        RE::NiAVObject* leaseOwnerNode = nullptr;
    };

    class CollisionSuppressionRegistry
    {
    public:
        RuntimeSuppressionResult acquire(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context);
        RuntimeSuppressionResult refresh(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context);
        RuntimeSuppressionResult release(RE::hknpWorld* world, std::uint32_t bodyId, CollisionSuppressionOwner owner, const char* context);
        void releaseOwner(RE::hknpWorld* world, CollisionSuppressionOwner owner, const char* context);
        bool hasLease(std::uint32_t bodyId, CollisionSuppressionOwner owner) const;
        void clear();

    private:
        struct RuntimeEntry
        {
            RE::hknpWorld* world = nullptr;
            std::uint32_t bodyId = kInvalidBodyId;
            std::uint32_t originalFilter = 0;
            std::uint32_t ownerMask = 0;
            std::uint32_t motionIndex = 0;
            RE::NiCollisionObject* collisionObject = nullptr;
            RE::NiAVObject* ownerNode = nullptr;
            bool wasNoCollideBeforeSuppression = false;
        };

        RuntimeEntry* find(RE::hknpWorld* world, std::uint32_t bodyId);
        const RuntimeEntry* find(RE::hknpWorld* world, std::uint32_t bodyId) const;
        void erase(RE::hknpWorld* world, std::uint32_t bodyId);
        static bool bodyIdentityMatches(const RuntimeEntry& entry,
            std::uint32_t motionIndex,
            RE::NiCollisionObject* collisionObject,
            RE::NiAVObject* ownerNode);
        static void captureBodyIdentity(RuntimeEntry& entry,
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            std::uint32_t motionIndex,
            RE::NiCollisionObject* collisionObject,
            RE::NiAVObject* ownerNode);

        std::vector<RuntimeEntry> _entries;
    };

    CollisionSuppressionRegistry& globalCollisionSuppressionRegistry();

    struct DelayedRestoreTimer
    {
        bool pending = false;
        float remainingSeconds = 0.0f;
        std::uint32_t firstBodyId = kInvalidBodyId;
        std::uint32_t bodyCount = 0;

        bool begin(
            std::uint32_t bodyId,
            std::size_t activeBodyCount,
            float delaySeconds) noexcept
        {
            const float delay =
                std::isfinite(delaySeconds) && delaySeconds > 0.0f ?
                    delaySeconds : 0.0f;
            if (bodyId == kInvalidBodyId || activeBodyCount == 0 || delay <= 0.0f) {
                clear();
                return false;
            }
            pending = true;
            remainingSeconds = delay;
            firstBodyId = bodyId;
            bodyCount = static_cast<std::uint32_t>(activeBodyCount);
            return true;
        }

        bool advance(bool hasActiveBodies, float deltaSeconds) noexcept
        {
            if (!pending) {
                return false;
            }
            if (!hasActiveBodies) {
                clear();
                return false;
            }
            const float delta =
                std::isfinite(deltaSeconds) && deltaSeconds > 0.0f ?
                    deltaSeconds : 0.0f;
            remainingSeconds = (std::max)(0.0f, remainingSeconds - delta);
            return remainingSeconds <= 0.0f;
        }

        void clear() noexcept
        {
            pending = false;
            remainingSeconds = 0.0f;
            firstBodyId = kInvalidBodyId;
            bodyCount = 0;
        }
    };

    template <std::size_t Capacity>
    class SuppressionLeaseSet
    {
    public:
        explicit constexpr SuppressionLeaseSet(CollisionSuppressionOwner owner) noexcept :
            _owner(owner)
        {}

        [[nodiscard]] bool empty() const noexcept { return _count == 0; }
        [[nodiscard]] bool full() const noexcept { return _count >= Capacity; }
        [[nodiscard]] std::size_t size() const noexcept { return _count; }
        [[nodiscard]] std::uint32_t firstBodyId() const noexcept
        {
            return _count > 0 ? _bodyIds[0] : kInvalidBodyId;
        }
        [[nodiscard]] bool delayedRestorePending() const noexcept
        {
            return _delayedRestore.pending;
        }
        [[nodiscard]] float delayedRestoreRemainingSeconds() const noexcept
        {
            return _delayedRestore.remainingSeconds;
        }

        [[nodiscard]] bool contains(std::uint32_t bodyId) const noexcept
        {
            for (std::size_t index = 0; index < _count; ++index) {
                if (_bodyIds[index] == bodyId) {
                    return true;
                }
            }
            return false;
        }

        RuntimeSuppressionResult acquire(
            RE::hknpWorld* world,
            std::uint32_t bodyId,
            const char* context)
        {
            RuntimeSuppressionResult result{};
            result.bodyId = bodyId;
            const bool alreadyTracked = contains(bodyId);
            if (bodyId == kInvalidBodyId || (!alreadyTracked && full())) {
                return result;
            }

            result = globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                _owner,
                context);
            if (result.valid && !alreadyTracked) {
                _bodyIds[_count++] = bodyId;
            }
            return result;
        }

        template <class OnRelease>
        bool releaseAll(
            RE::hknpWorld* world,
            const char* context,
            OnRelease&& onRelease)
        {
            std::array<std::uint32_t, Capacity> pending{};
            std::size_t pendingCount = 0;
            for (std::size_t index = 0; index < _count; ++index) {
                const auto bodyId = _bodyIds[index];
                const auto result =
                    globalCollisionSuppressionRegistry().release(
                        world,
                        bodyId,
                        _owner,
                        context);
                std::invoke(onRelease, bodyId, result);
                if (result.readFailed &&
                    globalCollisionSuppressionRegistry().hasLease(
                        bodyId,
                        _owner)) {
                    pending[pendingCount++] = bodyId;
                }
            }
            _bodyIds = pending;
            _count = pendingCount;
            if (_count == 0) {
                cancelDelayedRestore();
            }
            return _count == 0;
        }

        template <class ShouldRelease, class OnRelease>
        void releaseWhere(
            RE::hknpWorld* world,
            const char* context,
            ShouldRelease&& shouldRelease,
            OnRelease&& onRelease)
        {
            std::array<std::uint32_t, Capacity> retained{};
            std::size_t retainedCount = 0;
            for (std::size_t index = 0; index < _count; ++index) {
                const auto bodyId = _bodyIds[index];
                if (!std::invoke(shouldRelease, bodyId)) {
                    retained[retainedCount++] = bodyId;
                    continue;
                }
                const auto result =
                    globalCollisionSuppressionRegistry().release(
                        world,
                        bodyId,
                        _owner,
                        context);
                std::invoke(onRelease, bodyId, result);
                if (result.readFailed &&
                    globalCollisionSuppressionRegistry().hasLease(
                        bodyId,
                        _owner)) {
                    retained[retainedCount++] = bodyId;
                }
            }
            _bodyIds = retained;
            _count = retainedCount;
            if (_count == 0) {
                cancelDelayedRestore();
            }
        }

        bool beginDelayedRestore(float delaySeconds) noexcept
        {
            return _delayedRestore.begin(firstBodyId(), size(), delaySeconds);
        }

        bool advanceDelayedRestore(float deltaSeconds) noexcept
        {
            return _delayedRestore.advance(!empty(), deltaSeconds);
        }

        void cancelDelayedRestore() noexcept
        {
            _delayedRestore.clear();
        }

        void clearTracking() noexcept
        {
            _bodyIds = {};
            _count = 0;
            cancelDelayedRestore();
        }

    private:
        CollisionSuppressionOwner _owner;
        std::array<std::uint32_t, Capacity> _bodyIds{};
        std::size_t _count = 0;
        DelayedRestoreTimer _delayedRestore{};
    };
}

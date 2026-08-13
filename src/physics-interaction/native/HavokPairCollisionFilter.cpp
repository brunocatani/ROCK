#include "physics-interaction/native/HavokPairCollisionFilter.h"

#include "physics-interaction/native/HavokOffsets.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"

#include "REL/Relocation.h"
#include "RE/Havok/hknpWorld.h"

#include <algorithm>

namespace rock
{
    namespace
    {
        constexpr std::uintptr_t kSignalTagMask = 0x3u;
        constexpr std::size_t kMaximumSignalSlots = 64;
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFFu;

        [[nodiscard]] std::uintptr_t constraintFilterVtable() noexcept
        {
            static REL::Relocation<std::uintptr_t> address{
                REL::Offset(offsets::kVtable_ConstraintCollisionFilter)
            };
            return address.address();
        }

        [[nodiscard]] std::uintptr_t constraintAddedCallback() noexcept
        {
            static REL::Relocation<std::uintptr_t> address{
                REL::Offset(offsets::kFunc_ConstraintFilterOnConstraintAdded)
            };
            return address.address();
        }

        [[nodiscard]] std::uintptr_t constraintRemovedCallback() noexcept
        {
            static REL::Relocation<std::uintptr_t> address{
                REL::Offset(offsets::kFunc_ConstraintFilterOnConstraintRemoved)
            };
            return address.address();
        }

        [[nodiscard]] bool filterOwnerMatches(
            const void* owner,
            RE::hknpWorld* world) noexcept
        {
            std::uintptr_t vtable = 0;
            std::uint8_t type = 0;
            RE::hknpWorld* reciprocalWorld = nullptr;
            return owner && world &&
                   native_memory::tryReadField(owner, 0, vtable) &&
                   vtable == constraintFilterVtable() &&
                   native_memory::tryReadField(
                       owner,
                       offsets::kConstraintCollisionFilter_Type,
                       type) &&
                   type == 1 &&
                   native_memory::tryReadField(
                       owner,
                       offsets::kConstraintCollisionFilter_World,
                       reciprocalWorld) &&
                   reciprocalWorld == world;
        }

        [[nodiscard]] bool signalContainsOwner(
            RE::hknpWorld* world,
            std::uintptr_t signalOffset,
            std::uintptr_t expectedCallback,
            const void* expectedOwner) noexcept
        {
            std::uintptr_t tagged = 0;
            if (!native_memory::tryReadField(world, signalOffset, tagged)) {
                return false;
            }

            std::uintptr_t previous = 0;
            for (std::size_t visited = 0;
                 visited < kMaximumSignalSlots;
                 ++visited) {
                const std::uintptr_t slotAddress = tagged & ~kSignalTagMask;
                if (!slotAddress || slotAddress == previous) {
                    return false;
                }
                const auto* slot = reinterpret_cast<const void*>(slotAddress);
                std::uintptr_t nextTagged = 0;
                const void* owner = nullptr;
                std::uintptr_t callback = 0;
                if (!native_memory::tryReadField(
                        slot,
                        offsets::kSignalSlot_NextTagged,
                        nextTagged) ||
                    !native_memory::tryReadField(
                        slot,
                        offsets::kSignalSlot_Owner,
                        owner) ||
                    !native_memory::tryReadField(
                        slot,
                        offsets::kSignalSlot_Callback,
                        callback)) {
                    return false;
                }
                if (owner == expectedOwner && callback == expectedCallback) {
                    return true;
                }
                previous = slotAddress;
                tagged = nextTagged;
                if ((tagged & ~kSignalTagMask) == 0) {
                    return false;
                }
            }
            return false;
        }

        [[nodiscard]] void* findConstraintPairFilter(
            RE::hknpWorld* world) noexcept
        {
            std::uintptr_t tagged = 0;
            if (!world ||
                !native_memory::tryReadField(
                    world,
                    offsets::kHknpWorld_ConstraintAddedSignal,
                    tagged)) {
                return nullptr;
            }

            void* candidate = nullptr;
            std::uintptr_t previous = 0;
            for (std::size_t visited = 0;
                 visited < kMaximumSignalSlots;
                 ++visited) {
                const std::uintptr_t slotAddress = tagged & ~kSignalTagMask;
                if (!slotAddress || slotAddress == previous) {
                    break;
                }
                const auto* slot = reinterpret_cast<const void*>(slotAddress);
                std::uintptr_t nextTagged = 0;
                void* owner = nullptr;
                std::uintptr_t callback = 0;
                if (!native_memory::tryReadField(
                        slot,
                        offsets::kSignalSlot_NextTagged,
                        nextTagged) ||
                    !native_memory::tryReadField(
                        slot,
                        offsets::kSignalSlot_Owner,
                        owner) ||
                    !native_memory::tryReadField(
                        slot,
                        offsets::kSignalSlot_Callback,
                        callback)) {
                    return nullptr;
                }

                if (callback == constraintAddedCallback() &&
                    filterOwnerMatches(owner, world) &&
                    signalContainsOwner(
                        world,
                        offsets::kHknpWorld_ConstraintRemovedSignal,
                        constraintRemovedCallback(),
                        owner)) {
                    if (candidate && candidate != owner) {
                        return nullptr;
                    }
                    candidate = owner;
                }

                previous = slotAddress;
                tagged = nextTagged;
                if ((tagged & ~kSignalTagMask) == 0) {
                    break;
                }
            }
            return candidate;
        }

        [[nodiscard]] bool sameKey(
            const HavokPairCollisionLeaseSet::DesiredPair& lhs,
            const HavokPairCollisionLeaseSet::DesiredPair& rhs) noexcept
        {
            return lhs.bodyA == rhs.bodyA && lhs.bodyB == rhs.bodyB;
        }
    }

    bool HavokPairCollisionLeaseSet::filterIdentityMatches(
        RE::hknpWorld* world) const noexcept
    {
        return _world == world && filterOwnerMatches(_filter, world);
    }

    bool HavokPairCollisionLeaseSet::resolveFilter(
        RE::hknpWorld* world) noexcept
    {
        if (filterIdentityMatches(world)) {
            return true;
        }
        _filter = findConstraintPairFilter(world);
        _world = _filter ? world : nullptr;
        return _filter != nullptr;
    }

    HavokPairCollisionLeaseSet::ReconcileResult
    HavokPairCollisionLeaseSet::reconcile(
        RE::hknpWorld* world,
        const DesiredPair* desiredPairs,
        const std::size_t desiredPairCount) noexcept
    {
        ReconcileResult result{};
        if (!world) {
            abandonWorld();
            return result;
        }
        if (_world && _world != world) {
            abandonWorld();
        }
        if (!resolveFilter(world)) {
            return result;
        }
        result.filterAvailable = true;

        std::array<PairIdentity, kMaximumPairs> desired{};
        std::size_t acceptedDesiredCount = 0;
        const std::size_t boundedDesiredCount =
            (std::min)(desiredPairCount, kMaximumPairs);
        for (std::size_t index = 0;
             desiredPairs && index < boundedDesiredCount;
             ++index) {
            DesiredPair normalized = desiredPairs[index];
            if (normalized.bodyA == kInvalidBodyId ||
                normalized.bodyB == kInvalidBodyId ||
                normalized.bodyA == normalized.bodyB ||
                normalized.ownerGroup >= result.activePairsByOwnerGroup.size()) {
                continue;
            }
            if (normalized.bodyB < normalized.bodyA) {
                std::swap(normalized.bodyA, normalized.bodyB);
            }
            bool duplicate = false;
            for (std::size_t prior = 0; prior < acceptedDesiredCount; ++prior) {
                const DesiredPair priorKey{
                    desired[prior].bodyA,
                    desired[prior].bodyB,
                    desired[prior].ownerGroup
                };
                if (sameKey(normalized, priorKey)) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }

            const auto snapshotA = havok_runtime::snapshotBody(
                world,
                RE::hknpBodyId{ normalized.bodyA });
            const auto snapshotB = havok_runtime::snapshotBody(
                world,
                RE::hknpBodyId{ normalized.bodyB });
            if (!snapshotA.valid || !snapshotB.valid ||
                !snapshotA.collisionObject || !snapshotB.collisionObject) {
                continue;
            }
            desired[acceptedDesiredCount++] = PairIdentity{
                .bodyA = normalized.bodyA,
                .bodyB = normalized.bodyB,
                .collisionObjectA = reinterpret_cast<std::uintptr_t>(
                    snapshotA.collisionObject),
                .collisionObjectB = reinterpret_cast<std::uintptr_t>(
                    snapshotB.collisionObject),
                .ownerGroup = normalized.ownerGroup,
                .valid = true,
            };
        }

        using PairMutation_t = std::uint32_t (*)(
            void*,
            RE::hknpWorld*,
            std::uint32_t,
            std::uint32_t);
        static REL::Relocation<PairMutation_t> disablePair{
            REL::Offset(offsets::kFunc_PairCollisionFilterDisablePair)
        };
        static REL::Relocation<PairMutation_t> enablePair{
            REL::Offset(offsets::kFunc_PairCollisionFilterEnablePair)
        };

        for (auto& active : _activePairs) {
            if (!active.valid) {
                continue;
            }
            const bool retained = std::any_of(
                desired.begin(),
                desired.begin() + acceptedDesiredCount,
                [&](const PairIdentity& candidate) {
                    return candidate.valid &&
                           candidate.bodyA == active.bodyA &&
                           candidate.bodyB == active.bodyB &&
                           candidate.collisionObjectA ==
                               active.collisionObjectA &&
                           candidate.collisionObjectB ==
                               active.collisionObjectB;
                });
            if (!retained) {
                (void)enablePair(
                    _filter,
                    world,
                    active.bodyA,
                    active.bodyB);
                active = {};
            }
        }

        for (std::size_t desiredIndex = 0;
             desiredIndex < acceptedDesiredCount;
             ++desiredIndex) {
            const auto& candidate = desired[desiredIndex];
            const bool alreadyActive = std::any_of(
                _activePairs.begin(),
                _activePairs.end(),
                [&](const PairIdentity& active) {
                    return active.valid &&
                           active.bodyA == candidate.bodyA &&
                           active.bodyB == candidate.bodyB &&
                           active.collisionObjectA ==
                               candidate.collisionObjectA &&
                           active.collisionObjectB ==
                               candidate.collisionObjectB;
                });
            if (alreadyActive) {
                continue;
            }
            auto freeIt = std::find_if(
                _activePairs.begin(),
                _activePairs.end(),
                [](const PairIdentity& entry) { return !entry.valid; });
            if (freeIt == _activePairs.end()) {
                break;
            }
            const std::uint32_t referenceCount = disablePair(
                _filter,
                world,
                candidate.bodyA,
                candidate.bodyB);
            if (referenceCount == 0) {
                continue;
            }
            *freeIt = candidate;
        }

        for (const auto& active : _activePairs) {
            if (!active.valid) {
                continue;
            }
            ++result.activePairCount;
            if (active.ownerGroup < result.activePairsByOwnerGroup.size()) {
                ++result.activePairsByOwnerGroup[active.ownerGroup];
            }
        }
        return result;
    }

    void HavokPairCollisionLeaseSet::abandonWorld() noexcept
    {
        _activePairs = {};
        _filter = nullptr;
        _world = nullptr;
    }
}

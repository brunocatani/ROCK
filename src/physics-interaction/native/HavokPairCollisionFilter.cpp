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

    }

    template <std::size_t MaximumPairs, std::size_t OwnerGroupCount>
    bool BasicHavokPairCollisionLeaseSet<MaximumPairs, OwnerGroupCount>::filterIdentityMatches(
        RE::hknpWorld* world) const noexcept
    {
        return _world == world && filterOwnerMatches(_filter, world);
    }

    template <std::size_t MaximumPairs, std::size_t OwnerGroupCount>
    bool BasicHavokPairCollisionLeaseSet<MaximumPairs, OwnerGroupCount>::resolveFilter(
        RE::hknpWorld* world) noexcept
    {
        if (filterIdentityMatches(world)) {
            return true;
        }
        _filter = findConstraintPairFilter(world);
        _world = _filter ? world : nullptr;
        return _filter != nullptr;
    }

    template <std::size_t MaximumPairs, std::size_t OwnerGroupCount>
    typename BasicHavokPairCollisionLeaseSet<MaximumPairs, OwnerGroupCount>::ReconcileResult
    BasicHavokPairCollisionLeaseSet<MaximumPairs, OwnerGroupCount>::reconcile(
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
            const auto snapshotA = havok_runtime::snapshotBodyIdentity(
                world,
                RE::hknpBodyId{ normalized.bodyA });
            const auto snapshotB = havok_runtime::snapshotBodyIdentity(
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

        const auto identityLess = [](const PairIdentity& left,
                                      const PairIdentity& right) {
            if (left.bodyA != right.bodyA) {
                return left.bodyA < right.bodyA;
            }
            if (left.bodyB != right.bodyB) {
                return left.bodyB < right.bodyB;
            }
            if (left.collisionObjectA != right.collisionObjectA) {
                return left.collisionObjectA < right.collisionObjectA;
            }
            return left.collisionObjectB < right.collisionObjectB;
        };
        const auto sameIdentity = [](const PairIdentity& left,
                                      const PairIdentity& right) {
            return left.bodyA == right.bodyA &&
                   left.bodyB == right.bodyB &&
                   left.collisionObjectA == right.collisionObjectA &&
                   left.collisionObjectB == right.collisionObjectB;
        };
        std::sort(
            desired.begin(),
            desired.begin() + acceptedDesiredCount,
            identityLess);
        std::size_t uniqueDesiredCount = 0;
        for (std::size_t index = 0;
             index < acceptedDesiredCount;
             ++index) {
            if (uniqueDesiredCount != 0 &&
                desired[uniqueDesiredCount - 1].bodyA == desired[index].bodyA &&
                desired[uniqueDesiredCount - 1].bodyB == desired[index].bodyB) {
                desired[uniqueDesiredCount - 1].ownerGroup = (std::min)(
                    desired[uniqueDesiredCount - 1].ownerGroup,
                    desired[index].ownerGroup);
                continue;
            }
            desired[uniqueDesiredCount++] = desired[index];
        }
        acceptedDesiredCount = uniqueDesiredCount;

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

        std::sort(
            _activePairs.begin(),
            _activePairs.begin() + _activePairCount,
            identityLess);
        const auto previousActivePairCount = _activePairCount;
        std::size_t retainedPairCount = 0;
        for (std::size_t activeIndex = 0;
             activeIndex < previousActivePairCount;
             ++activeIndex) {
            const auto& active = _activePairs[activeIndex];
            const auto desiredIt = std::lower_bound(
                desired.begin(),
                desired.begin() + acceptedDesiredCount,
                active,
                identityLess);
            const bool retained = desiredIt !=
                    desired.begin() + acceptedDesiredCount &&
                sameIdentity(*desiredIt, active);
            if (!retained) {
                (void)enablePair(
                    _filter,
                    world,
                    active.bodyA,
                    active.bodyB);
                continue;
            }
            auto retainedPair = active;
            retainedPair.ownerGroup = desiredIt->ownerGroup;
            _activePairs[retainedPairCount++] = retainedPair;
        }
        for (std::size_t index = retainedPairCount;
             index < previousActivePairCount;
             ++index) {
            _activePairs[index] = {};
        }
        _activePairCount = retainedPairCount;

        for (std::size_t desiredIndex = 0;
             desiredIndex < acceptedDesiredCount;
             ++desiredIndex) {
            const auto& candidate = desired[desiredIndex];
            const auto activeIt = std::lower_bound(
                _activePairs.begin(),
                _activePairs.begin() + retainedPairCount,
                candidate,
                identityLess);
            const bool alreadyActive = activeIt !=
                    _activePairs.begin() + retainedPairCount &&
                sameIdentity(*activeIt, candidate);
            if (alreadyActive) {
                continue;
            }
            if (_activePairCount >= _activePairs.size()) {
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
            _activePairs[_activePairCount++] = candidate;
        }

        std::sort(
            _activePairs.begin(),
            _activePairs.begin() + _activePairCount,
            identityLess);

        for (std::size_t index = 0; index < _activePairCount; ++index) {
            const auto& active = _activePairs[index];
            ++result.activePairCount;
            if (active.ownerGroup < result.activePairsByOwnerGroup.size()) {
                ++result.activePairsByOwnerGroup[active.ownerGroup];
            }
        }
        return result;
    }

    template <std::size_t MaximumPairs, std::size_t OwnerGroupCount>
    void BasicHavokPairCollisionLeaseSet<MaximumPairs, OwnerGroupCount>::abandonWorld() noexcept
    {
        _activePairs = {};
        _activePairCount = 0;
        _filter = nullptr;
        _world = nullptr;
    }

    template class BasicHavokPairCollisionLeaseSet<34, 2>;
}

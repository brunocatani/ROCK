#include "physics-interaction/object/DynamicWorldCarCollision.h"

#include "RockConfig.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/collision/CollisionLayerPolicy.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>

namespace rock
{
    namespace
    {
        [[nodiscard]] bool isValidBodyId(std::uint32_t bodyId) noexcept
        {
            return bodyId != 0x7FFF'FFFFu && bodyId != 0xFFFF'FFFFu;
        }

        [[nodiscard]] object_physics_body_set::BodySetScanOptions makeCarBodyScanOptions(std::uint32_t seedBodyId)
        {
            object_physics_body_set::BodySetScanOptions options{};
            options.mode = physics_body_classifier::InteractionMode::ActiveGrab;
            options.seedBodyId = seedBodyId;
            options.targetKind = grab_target::Kind::DynamicMovableStatic;
            options.requireSameResolvedRef = true;
            options.allowUnresolvedRefBodies = false;
            options.allowWeaponRefExpansion = false;
            options.maxDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
            return options;
        }

        [[nodiscard]] bool isExplodableCarReference(RE::TESObjectREFR* ref) noexcept
        {
            return ref && !ref->IsDeleted() && !ref->IsDisabled() && fo4vr::isExplodableCar(ref->GetObjectReference());
        }
    }

    bool DynamicWorldCarCollisionRuntime::slotMatchesReference(const TargetSlot& slot, RE::TESObjectREFR* ref) const
    {
        if (!slot.active() || !ref || slot.formId != ref->GetFormID()) {
            return false;
        }
        const auto retained = slot.handle.get();
        return retained && retained.get() == ref;
    }

    bool DynamicWorldCarCollisionRuntime::slotStillOwnsTags(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const TargetSlot& slot) const
    {
        if (!bhkWorld || !hknpWorld || !slot.active()) {
            return false;
        }
        const auto retained = slot.handle.get();
        auto* ref = retained.get();
        if (!isExplodableCarReference(ref)) {
            return false;
        }

        for (std::size_t index = 0; index < slot.bodyCount; ++index) {
            const auto& tagged = slot.bodies[index];
            std::uint32_t currentFilterInfo = 0;
            if (!isValidBodyId(tagged.bodyId) ||
                !body_collision::tryReadFilterInfo(hknpWorld, RE::hknpBodyId{ tagged.bodyId }, currentFilterInfo) ||
                currentFilterInfo != tagged.taggedFilterInfo ||
                resolveBodyToRef(bhkWorld, hknpWorld, RE::hknpBodyId{ tagged.bodyId }) != ref) {
                return false;
            }
        }
        return true;
    }

    bool DynamicWorldCarCollisionRuntime::tagReference(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const DynamicWorldCarTarget& target,
        TargetSlot& outSlot)
    {
        outSlot.clear();
        if (!bhkWorld || !hknpWorld || !isExplodableCarReference(target.ref)) {
            return false;
        }

        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(
            bhkWorld,
            hknpWorld,
            target.ref,
            makeCarBodyScanOptions(target.seedBodyId));

        std::array<TaggedBody, kMaxBodiesPerTarget> candidates{};
        std::size_t candidateCount = 0;
        for (const auto& record : bodySet.records) {
            if (!isValidBodyId(record.bodyId) || record.resolvedRef != target.ref) {
                continue;
            }

            const auto taggedLayer = collision_layer_policy::dynamicWorldCarLayerForNativeLayer(record.collisionLayer);
            if (!collision_layer_policy::isDynamicWorldCarLayer(taggedLayer)) {
                continue;
            }
            if (candidateCount >= candidates.size()) {
                ROCK_LOG_WARN(Hand,
                    "Dynamic-world car tagging rejected overflow: formID={:08X} bodies>{}",
                    target.ref->GetFormID(),
                    candidates.size());
                return false;
            }

            const std::uint32_t taggedFilterInfo =
                (record.filterInfo & ~collision_layer_policy::FO4_LAYER_FILTER_MASK) | taggedLayer;
            candidates[candidateCount++] = TaggedBody{
                .bodyId = record.bodyId,
                .originalFilterInfo = record.filterInfo,
                .taggedFilterInfo = taggedFilterInfo,
            };
        }

        if (candidateCount == 0) {
            return false;
        }

        std::size_t taggedCount = 0;
        for (; taggedCount < candidateCount; ++taggedCount) {
            const auto& candidate = candidates[taggedCount];
            std::uint32_t currentFilterInfo = 0;
            if (!body_collision::tryReadFilterInfo(hknpWorld, RE::hknpBodyId{ candidate.bodyId }, currentFilterInfo) ||
                currentFilterInfo != candidate.originalFilterInfo ||
                resolveBodyToRef(bhkWorld, hknpWorld, RE::hknpBodyId{ candidate.bodyId }) != target.ref ||
                !body_collision::setFilterInfo(hknpWorld, RE::hknpBodyId{ candidate.bodyId }, candidate.taggedFilterInfo)) {
                break;
            }
        }

        if (taggedCount != candidateCount) {
            for (std::size_t rollbackIndex = 0; rollbackIndex < taggedCount; ++rollbackIndex) {
                const auto& candidate = candidates[rollbackIndex];
                std::uint32_t currentFilterInfo = 0;
                if (body_collision::tryReadFilterInfo(hknpWorld, RE::hknpBodyId{ candidate.bodyId }, currentFilterInfo) &&
                    currentFilterInfo == candidate.taggedFilterInfo &&
                    resolveBodyToRef(bhkWorld, hknpWorld, RE::hknpBodyId{ candidate.bodyId }) == target.ref) {
                    (void)body_collision::setFilterInfo(hknpWorld, RE::hknpBodyId{ candidate.bodyId }, candidate.originalFilterInfo);
                }
            }
            ROCK_LOG_SAMPLE_WARN(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "Dynamic-world car tagging rolled back: formID={:08X} tagged={}/{}",
                target.ref->GetFormID(),
                taggedCount,
                candidateCount);
            return false;
        }

        outSlot.handle = target.ref->GetHandle();
        outSlot.formId = target.ref->GetFormID();
        outSlot.bodies = candidates;
        outSlot.bodyCount = candidateCount;
        ROCK_LOG_DEBUG(Hand,
            "Dynamic-world car collision enabled: formID={:08X} bodies={} seedBody={}",
            outSlot.formId,
            outSlot.bodyCount,
            target.seedBodyId);
        return true;
    }

    std::size_t DynamicWorldCarCollisionRuntime::restoreSlot(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        TargetSlot& slot,
        const char* reason)
    {
        if (!slot.active()) {
            slot.clear();
            return 0;
        }

        const auto retained = slot.handle.get();
        auto* ref = retained.get();
        const auto seedBodyId = slot.bodies[0].bodyId;
        std::size_t restored = 0;
        for (std::size_t index = 0; index < slot.bodyCount; ++index) {
            const auto& tagged = slot.bodies[index];
            std::uint32_t currentFilterInfo = 0;
            if (!bhkWorld || !hknpWorld || !ref ||
                !body_collision::tryReadFilterInfo(hknpWorld, RE::hknpBodyId{ tagged.bodyId }, currentFilterInfo) ||
                currentFilterInfo != tagged.taggedFilterInfo ||
                resolveBodyToRef(bhkWorld, hknpWorld, RE::hknpBodyId{ tagged.bodyId }) != ref) {
                continue;
            }
            if (body_collision::setFilterInfo(hknpWorld, RE::hknpBodyId{ tagged.bodyId }, tagged.originalFilterInfo)) {
                ++restored;
            }
        }

        if (ref) {
            restored += restoreTaggedBodiesForReference(bhkWorld, hknpWorld, ref, seedBodyId, reason);
        }

        ROCK_LOG_DEBUG(Hand,
            "Dynamic-world car collision released: formID={:08X} restored={}/{} reason={}",
            slot.formId,
            restored,
            slot.bodyCount,
            reason ? reason : "unknown");
        slot.clear();
        return restored;
    }

    std::size_t DynamicWorldCarCollisionRuntime::restoreTaggedBodiesForReference(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* ref,
        std::uint32_t seedBodyId,
        const char* reason)
    {
        if (!bhkWorld || !hknpWorld || !isExplodableCarReference(ref)) {
            return 0;
        }

        const auto bodySet = object_physics_body_set::scanObjectPhysicsBodySet(
            bhkWorld,
            hknpWorld,
            ref,
            makeCarBodyScanOptions(seedBodyId));
        std::size_t restored = 0;
        for (const auto& record : bodySet.records) {
            if (!isValidBodyId(record.bodyId) || record.resolvedRef != ref) {
                continue;
            }
            const auto nativeLayer = collision_layer_policy::nativeLayerForDynamicWorldCarLayer(record.collisionLayer);
            if (nativeLayer == collision_layer_policy::FO4_LAYER_UNIDENTIFIED) {
                continue;
            }
            const std::uint32_t nativeFilterInfo =
                (record.filterInfo & ~collision_layer_policy::FO4_LAYER_FILTER_MASK) | nativeLayer;
            if (resolveBodyToRef(bhkWorld, hknpWorld, RE::hknpBodyId{ record.bodyId }) == ref &&
                body_collision::setFilterInfo(hknpWorld, RE::hknpBodyId{ record.bodyId }, nativeFilterInfo)) {
                ++restored;
            }
        }
        if (restored != 0) {
            ROCK_LOG_DEBUG(Hand,
                "Dynamic-world car fallback restore: formID={:08X} bodies={} reason={}",
                ref->GetFormID(),
                restored,
                reason ? reason : "unknown");
        }
        return restored;
    }

    void DynamicWorldCarCollisionRuntime::restoreReference(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        RE::TESObjectREFR* ref,
        const char* reason)
    {
        if (!ref) {
            return;
        }
        std::uint32_t seedBodyId = 0x7FFF'FFFFu;
        bool restoredTrackedSlot = false;
        for (auto& slot : _slots) {
            if (!slotMatchesReference(slot, ref)) {
                continue;
            }
            if (slot.bodyCount != 0) {
                seedBodyId = slot.bodies[0].bodyId;
            }
            (void)restoreSlot(bhkWorld, hknpWorld, slot, reason);
            restoredTrackedSlot = true;
        }
        if (!restoredTrackedSlot) {
            (void)restoreTaggedBodiesForReference(bhkWorld, hknpWorld, ref, seedBodyId, reason);
        }
    }

    void DynamicWorldCarCollisionRuntime::restoreAll(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const char* reason)
    {
        for (auto& slot : _slots) {
            (void)restoreSlot(bhkWorld, hknpWorld, slot, reason);
        }
        _world = nullptr;
    }

    void DynamicWorldCarCollisionRuntime::abandon() noexcept
    {
        _slots = {};
        _world = nullptr;
    }

    void DynamicWorldCarCollisionRuntime::update(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        const std::array<DynamicWorldCarTarget, 2>& desiredTargets)
    {
        reconcileTargets(bhkWorld, hknpWorld, desiredTargets, false);
    }

    void DynamicWorldCarCollisionRuntime::synchronizeNearbyTargets(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        std::span<const DynamicWorldCarTarget> desiredTargets)
    {
        reconcileTargets(bhkWorld, hknpWorld, desiredTargets, true);
    }

    void DynamicWorldCarCollisionRuntime::reconcileTargets(
        RE::bhkWorld* bhkWorld,
        RE::hknpWorld* hknpWorld,
        std::span<const DynamicWorldCarTarget> desiredTargets,
        bool restoreMissingTargets)
    {
        if (!bhkWorld || !hknpWorld) {
            abandon();
            return;
        }
        if (_world && _world != bhkWorld) {
            abandon();
        }
        _world = bhkWorld;

        auto desiredContains = [&](RE::TESObjectREFR* ref) {
            return std::any_of(desiredTargets.begin(), desiredTargets.end(), [&](const DynamicWorldCarTarget& target) {
                return target.ref == ref;
            });
        };

        for (auto& slot : _slots) {
            if (!slot.active()) {
                continue;
            }
            const auto retained = slot.handle.get();
            auto* ref = retained.get();
            if (!ref || (restoreMissingTargets && !desiredContains(ref)) || !slotStillOwnsTags(bhkWorld, hknpWorld, slot)) {
                (void)restoreSlot(bhkWorld,
                    hknpWorld,
                    slot,
                    !ref ? "reference-lost" : (restoreMissingTargets ? "left-nearby-set" : "tag-ownership-lost"));
            }
        }

        for (std::size_t targetIndex = 0; targetIndex < desiredTargets.size(); ++targetIndex) {
            const auto& target = desiredTargets[targetIndex];
            if (!isExplodableCarReference(target.ref)) {
                continue;
            }
            const bool duplicateDesired = std::any_of(
                desiredTargets.begin(),
                desiredTargets.begin() + static_cast<std::ptrdiff_t>(targetIndex),
                [&](const DynamicWorldCarTarget& previous) { return previous.ref == target.ref; });
            if (duplicateDesired) {
                continue;
            }
            const bool alreadyTagged = std::any_of(_slots.begin(), _slots.end(), [&](const TargetSlot& slot) {
                return slotMatchesReference(slot, target.ref);
            });
            if (alreadyTagged) {
                continue;
            }
            auto emptySlot = std::find_if(_slots.begin(), _slots.end(), [](const TargetSlot& slot) { return !slot.active(); });
            if (emptySlot != _slots.end()) {
                (void)tagReference(bhkWorld, hknpWorld, target, *emptySlot);
            }
        }
    }
}

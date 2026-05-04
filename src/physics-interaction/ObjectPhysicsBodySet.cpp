#include "ObjectPhysicsBodySet.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "HavokOffsets.h"
#include "ObjectDetection.h"
#include "PhysicsLog.h"
#include "PhysicsUtils.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Bethesda/bhkPhysicsSystem.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiNode.h"

namespace frik::rock::object_physics_body_set
{
    namespace
    {
        float distanceSquared(const PurePoint3& a, const PurePoint3& b)
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return dx * dx + dy * dy + dz * dz;
        }

        bool containsBodyId(const std::vector<std::uint32_t>* bodyIds, std::uint32_t bodyId)
        {
            if (!bodyIds) {
                return false;
            }
            return std::find(bodyIds->begin(), bodyIds->end(), bodyId) != bodyIds->end();
        }

        bool nodeIsOrContains(RE::NiAVObject* ancestor, RE::NiAVObject* node)
        {
            if (!ancestor || !node) {
                return false;
            }
            for (auto* current = node; current; current = current->parent) {
                if (current == ancestor) {
                    return true;
                }
            }
            return false;
        }

        void recordReject(BodySetDiagnostics& diagnostics, physics_body_classifier::BodyRejectReason reason)
        {
            const auto index = static_cast<std::size_t>(reason);
            if (index < diagnostics.rejectCounts.size()) {
                ++diagnostics.rejectCounts[index];
            }
        }

        void scanNode(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            RE::TESObjectREFR* rootRef,
            RE::NiAVObject* node,
            int depth,
            const BodySetScanOptions& options,
            ObjectPhysicsBodySet& out,
            std::unordered_set<std::uint32_t>& seenBodyIds)
        {
            if (!node || depth < 0) {
                return;
            }

            ++out.diagnostics.visitedNodes;

            auto* collisionObject = node->collisionObject.get();
            if (collisionObject) {
                ++out.diagnostics.collisionObjects;

                constexpr std::uintptr_t kMinValidPointer = 0x10000;
                auto* fieldAt20 = *reinterpret_cast<void**>(reinterpret_cast<char*>(collisionObject) + offsets::kCollisionObject_PhysSystemPtr);
                if (fieldAt20 && reinterpret_cast<std::uintptr_t>(fieldAt20) > kMinValidPointer) {
                    auto* physSystem = reinterpret_cast<RE::bhkPhysicsSystem*>(fieldAt20);
                    auto* instance = physSystem->instance;
                    if (instance && reinterpret_cast<std::uintptr_t>(instance) > kMinValidPointer && instance->world == hknpWorld && instance->bodyIds) {
                        const std::int32_t count = (std::min)(instance->bodyCount, 256);
                        for (std::int32_t i = 0; i < count; ++i) {
                            const std::uint32_t rawBodyId = instance->bodyIds[i];
                            if (rawBodyId == INVALID_BODY_ID || rawBodyId > 0x000F'FFFF) {
                                continue;
                            }
                            if (!seenBodyIds.insert(rawBodyId).second) {
                                ++out.diagnostics.duplicateBodySkips;
                                continue;
                            }

                            RE::hknpBodyId bodyId{ rawBodyId };
                            auto* body = havok_runtime::getBody(hknpWorld, bodyId);
                            if (!body) {
                                continue;
                            }

                            ObjectPhysicsBodyRecord record{};
                            record.bodyId = rawBodyId;
                            record.motionId = body->motionIndex;
                            record.filterInfo = body->collisionFilterInfo;
                            record.collisionLayer = body->collisionFilterInfo & 0x7F;
                            auto* motion = havok_runtime::getMotion(hknpWorld, record.motionId);
                            const std::uint16_t motionPropertiesId =
                                motion ? motion->motionPropertiesId : static_cast<std::uint16_t>(body->motionPropertiesId);
                            record.bodyFlags = body->flags;
                            record.motionPropertiesId = motionPropertiesId;
                            record.motionType = physics_body_classifier::motionTypeFromBodyFlags(record.bodyFlags);
                            if (record.motionType == physics_body_classifier::BodyMotionType::Unknown) {
                                record.motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(motionPropertiesId);
                            }
                            record.owningNode = node;
                            record.collisionObject = collisionObject;
                            record.resolvedRef = rootRef;

                            const auto* bodyFloats = reinterpret_cast<const float*>(body);
                            record.positionGame.x = bodyFloats[12] * havokToGameScale();
                            record.positionGame.y = bodyFloats[13] * havokToGameScale();
                            record.positionGame.z = bodyFloats[14] * havokToGameScale();

                            auto* resolved = resolveBodyToRef(bhkWorld, hknpWorld, bodyId);
                            if (resolved) {
                                record.resolvedRef = resolved;
                            }

                            physics_body_classifier::BodyClassificationInput input{};
                            input.bodyId = rawBodyId;
                            input.motionId = record.motionId;
                            input.layer = record.collisionLayer;
                            input.filterInfo = record.filterInfo;
                            input.motionType = record.motionType;
                            input.bodyFlags = record.bodyFlags;
                            input.referenceDeletedOrDisabled =
                                !record.resolvedRef || record.resolvedRef->IsDeleted() || record.resolvedRef->IsDisabled();
                            input.isRockHandBody =
                                rawBodyId == options.rightHandBodyId || rawBodyId == options.leftHandBodyId ||
                                record.collisionLayer == collision_layer_policy::ROCK_LAYER_HAND;
                            input.isRockWeaponSourceBody = rawBodyId == options.sourceWeaponBodyId || rawBodyId == options.sourceBodyId;
                            input.isHeldBySameHand = containsBodyId(options.heldBySameHand, rawBodyId);
                            input.isPlayerBody = record.resolvedRef == RE::PlayerCharacter::GetSingleton();

                            const auto classification = physics_body_classifier::classifyBody(input, options.mode);
                            record.accepted = classification.accepted;
                            record.rejectReason = classification.reason;
                            if (!record.accepted) {
                                recordReject(out.diagnostics, record.rejectReason);
                            }

                            out.records.push_back(record);
                        }
                    }
                }
            }

            auto* niNode = node->IsNode();
            if (!niNode) {
                return;
            }

            auto& children = niNode->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                auto* child = children[i].get();
                if (child) {
                    scanNode(bhkWorld, hknpWorld, rootRef, child, depth - 1, options, out, seenBodyIds);
                }
            }
        }
    }

    std::size_t ObjectPhysicsBodySet::acceptedCount() const
    {
        return static_cast<std::size_t>(
            std::count_if(records.begin(), records.end(), [](const ObjectPhysicsBodyRecord& record) { return record.accepted; }));
    }

    std::size_t ObjectPhysicsBodySet::rejectedCount() const { return records.size() - acceptedCount(); }

    std::vector<std::uint32_t> ObjectPhysicsBodySet::acceptedBodyIds() const
    {
        std::vector<std::uint32_t> result;
        result.reserve(records.size());
        for (const auto& record : records) {
            if (record.accepted) {
                result.push_back(record.bodyId);
            }
        }
        return result;
    }

    std::vector<std::uint32_t> ObjectPhysicsBodySet::uniqueAcceptedMotionBodyIds() const
    {
        std::vector<std::uint32_t> result;
        std::unordered_set<std::uint32_t> seenMotionIds;
        result.reserve(records.size());
        diagnostics.duplicateMotionSkips = 0;
        for (const auto& record : records) {
            if (!record.accepted) {
                continue;
            }
            if (seenMotionIds.insert(record.motionId).second) {
                result.push_back(record.bodyId);
            } else {
                ++diagnostics.duplicateMotionSkips;
            }
        }
        return result;
    }

    std::vector<const ObjectPhysicsBodyRecord*> ObjectPhysicsBodySet::uniqueAcceptedMotionRecords() const
    {
        std::vector<const ObjectPhysicsBodyRecord*> result;
        std::unordered_set<std::uint32_t> seenMotionIds;
        result.reserve(records.size());
        diagnostics.duplicateMotionSkips = 0;
        for (const auto& record : records) {
            if (!record.accepted) {
                continue;
            }
            if (seenMotionIds.insert(record.motionId).second) {
                result.push_back(&record);
            } else {
                ++diagnostics.duplicateMotionSkips;
            }
        }
        return result;
    }

    bool ObjectPhysicsBodySet::containsAcceptedBody(std::uint32_t bodyId) const
    {
        return std::any_of(records.begin(), records.end(), [&](const ObjectPhysicsBodyRecord& record) {
            return record.accepted && record.bodyId == bodyId;
        });
    }

    const ObjectPhysicsBodyRecord* ObjectPhysicsBodySet::findRecord(std::uint32_t bodyId) const
    {
        const auto it = std::find_if(records.begin(), records.end(), [&](const ObjectPhysicsBodyRecord& record) { return record.bodyId == bodyId; });
        return it != records.end() ? &*it : nullptr;
    }

    const ObjectPhysicsBodyRecord* ObjectPhysicsBodySet::findAcceptedRecordByOwnerNode(RE::NiAVObject* ownerNode) const
    {
        if (!ownerNode) {
            return nullptr;
        }

        for (const auto& record : records) {
            if (record.accepted && record.owningNode == ownerNode) {
                return &record;
            }
        }

        for (const auto& record : records) {
            if (!record.accepted || !record.owningNode) {
                continue;
            }
            if (nodeIsOrContains(record.owningNode, ownerNode)) {
                return &record;
            }
        }

        for (const auto& record : records) {
            if (!record.accepted || !record.owningNode) {
                continue;
            }
            if (nodeIsOrContains(ownerNode, record.owningNode)) {
                return &record;
            }
        }

        return nullptr;
    }

    PrimaryBodyChoice ObjectPhysicsBodySet::choosePrimaryBody(std::uint32_t preferredBodyId, PurePoint3 targetPointGame) const
    {
        if (containsAcceptedBody(preferredBodyId)) {
            return PrimaryBodyChoice{ .bodyId = preferredBodyId, .reason = PrimaryBodyChoiceReason::PreferredHitAccepted };
        }

        float bestDistance = std::numeric_limits<float>::max();
        std::uint32_t bestBodyId = INVALID_BODY_ID;
        for (const auto& record : records) {
            if (!record.accepted) {
                continue;
            }
            const float distSq = distanceSquared(record.positionGame, targetPointGame);
            if (distSq < bestDistance) {
                bestDistance = distSq;
                bestBodyId = record.bodyId;
            }
        }

        if (bestBodyId == INVALID_BODY_ID) {
            return PrimaryBodyChoice{ .bodyId = INVALID_BODY_ID, .reason = PrimaryBodyChoiceReason::NoAcceptedBody };
        }
        return PrimaryBodyChoice{ .bodyId = bestBodyId, .reason = PrimaryBodyChoiceReason::NearestAcceptedFallback };
    }

    PrimaryBodyChoice ObjectPhysicsBodySet::choosePrimaryBodyWithSurfaceOwner(std::uint32_t preferredBodyId, RE::NiAVObject* ownerNode, PurePoint3 targetPointGame) const
    {
        if (const auto* surfaceOwner = findAcceptedRecordByOwnerNode(ownerNode)) {
            return PrimaryBodyChoice{ .bodyId = surfaceOwner->bodyId, .reason = PrimaryBodyChoiceReason::SurfaceOwnerAccepted };
        }

        return choosePrimaryBody(preferredBodyId, targetPointGame);
    }

    bool hasCollisionObjectInSubtree(RE::NiAVObject* root, int maxDepth)
    {
        if (!root || maxDepth < 0) {
            return false;
        }
        if (root->collisionObject.get()) {
            return true;
        }

        auto* niNode = root->IsNode();
        if (!niNode) {
            return false;
        }

        auto& children = niNode->GetRuntimeData().children;
        for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
            auto* child = children[i].get();
            if (child && hasCollisionObjectInSubtree(child, maxDepth - 1)) {
                return true;
            }
        }
        return false;
    }

    ObjectPhysicsBodySet scanObjectPhysicsBodySet(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::TESObjectREFR* ref, const BodySetScanOptions& options)
    {
        ObjectPhysicsBodySet result;
        result.rootRef = ref;
        if (!bhkWorld || !hknpWorld || !ref || ref->IsDeleted() || ref->IsDisabled()) {
            return result;
        }

        result.rootNode = ref->Get3D();
        if (!result.rootNode) {
            return result;
        }

        std::unordered_set<std::uint32_t> seenBodyIds;
        scanNode(bhkWorld, hknpWorld, ref, result.rootNode, (std::max)(0, options.maxDepth), options, result, seenBodyIds);
        return result;
    }
}

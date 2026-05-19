#include "physics-interaction/object/ObjectPhysicsBodySet.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>

#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/NativeMemory.h"
#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/PhysicsUtils.h"

#include "RE/Bethesda/BSHavok.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESForms.h"
#include "RE/Bethesda/TESObjectREFRs.h"
#include "RE/Havok/hknpBody.h"
#include "RE/Havok/hknpBodyId.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Havok/hknpWorld.h"
#include "RE/NetImmerse/NiAVObject.h"
#include "RE/NetImmerse/NiCollisionObject.h"
#include "RE/NetImmerse/NiNode.h"

namespace rock::object_physics_body_set
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
            constexpr int kMaxParentWalkDepth = 128;
            int depth = 0;
            for (auto* current = node; current && depth < kMaxParentWalkDepth; ++depth) {
                if (current == ancestor) {
                    return true;
                }
                RE::NiNode* parent = nullptr;
                if (!native_memory::tryReadValue(&current->parent, parent)) {
                    return false;
                }
                current = parent;
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

        const char* nodeName(RE::NiAVObject* node)
        {
            return node && node->name.c_str() ? node->name.c_str() : "(unnamed)";
        }

        bool isWeaponReference(RE::TESObjectREFR* ref)
        {
            auto* baseForm = ref ? ref->GetObjectReference() : nullptr;
            return baseForm && baseForm->Is(RE::ENUM_FORM_ID::kWEAP);
        }

        bool isInvalidPhysicsSystemStatus(havok_runtime::PhysicsSystemBodyScanStatus status)
        {
            using havok_runtime::PhysicsSystemBodyScanStatus;
            return status == PhysicsSystemBodyScanStatus::InvalidBodyCount ||
                   status == PhysicsSystemBodyScanStatus::UnreadableBodyIds ||
                   status == PhysicsSystemBodyScanStatus::MissingBodyIds;
        }

        struct ScanBodyContext
        {
            RE::bhkWorld* bhkWorld = nullptr;
            RE::hknpWorld* hknpWorld = nullptr;
            RE::TESObjectREFR* rootRef = nullptr;
            RE::NiAVObject* node = nullptr;
            RE::NiCollisionObject* collisionObject = nullptr;
            const BodySetScanOptions* options = nullptr;
            ObjectPhysicsBodySet* out = nullptr;
            std::unordered_set<std::uint32_t>* seenBodyIds = nullptr;
        };

        bool appendBodyRecord(ScanBodyContext& context, std::uint32_t rawBodyId, bool seeded)
        {
            if (!context.hknpWorld || !context.options || !context.out || !context.seenBodyIds) {
                return false;
            }

            auto& out = *context.out;
            auto& seenBodyIds = *context.seenBodyIds;
            const auto& options = *context.options;
            auto* hknpWorld = context.hknpWorld;

            if (!seenBodyIds.insert(rawBodyId).second) {
                ++out.diagnostics.duplicateBodySkips;
                return true;
            }

            RE::hknpBodyId bodyId{ rawBodyId };
            auto* body = havok_runtime::getBody(hknpWorld, bodyId);
            if (!body) {
                return true;
            }

            ObjectPhysicsBodyRecord record{};
            record.bodyId = rawBodyId;
            record.motionId = body->motionIndex;
            record.filterInfo = body->collisionFilterInfo;
            record.collisionLayer = body->collisionFilterInfo & 0x7F;
            auto* motion = havok_runtime::getMotion(hknpWorld, record.motionId);
            const std::uint16_t motionPropertiesId = motion ? motion->motionPropertiesId : static_cast<std::uint16_t>(body->motionPropertiesId);
            record.bodyFlags = body->flags;
            record.motionPropertiesId = motionPropertiesId;
            record.motionType = physics_body_classifier::motionTypeFromBodyFlags(record.bodyFlags);
            if (record.motionType == physics_body_classifier::BodyMotionType::Unknown) {
                record.motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(motionPropertiesId);
            }
            record.owningNode = context.node;
            record.collisionObject = context.collisionObject;
            record.resolvedRef = context.rootRef;
            record.seeded = seeded;

            const auto* bodyFloats = reinterpret_cast<const float*>(body);
            record.positionGame.x = bodyFloats[12] * havokToGameScale();
            record.positionGame.y = bodyFloats[13] * havokToGameScale();
            record.positionGame.z = bodyFloats[14] * havokToGameScale();

            auto* resolved = resolveBodyToRef(context.bhkWorld, hknpWorld, bodyId);
            if (resolved) {
                if (options.requireSameResolvedRef && resolved != context.rootRef) {
                    ++out.diagnostics.foreignRefBodySkips;
                    return true;
                }
                record.resolvedRef = resolved;
                record.refResolutionKnown = true;
            } else if (options.requireSameResolvedRef && !options.allowUnresolvedRefBodies && !seeded) {
                ++out.diagnostics.unresolvedRefBodySkips;
                return true;
            } else {
                /*
                 * Weapon and multipart loose-object systems can expose body ids
                 * that are valid under the selected scene tree but do not map
                 * back through bhkWorld owner lookup. Keep them for activation
                 * and lifecycle tracking, but mark the ownership as unresolved
                 * so diagnostics and future policy do not treat rootRef as a
                 * verified native owner.
                 */
                ++out.diagnostics.unresolvedRefBodiesAccepted;
            }

            physics_body_classifier::BodyClassificationInput input{};
            input.bodyId = rawBodyId;
            input.motionId = record.motionId;
            input.layer = record.collisionLayer;
            input.filterInfo = record.filterInfo;
            input.motionType = record.motionType;
            input.targetKind = options.targetKind;
            input.bodyFlags = record.bodyFlags;
            input.referenceDeletedOrDisabled = !record.resolvedRef || record.resolvedRef->IsDeleted() || record.resolvedRef->IsDisabled();
            input.isRockHandBody =
                rawBodyId == options.rightHandBodyId || rawBodyId == options.leftHandBodyId || record.collisionLayer == collision_layer_policy::ROCK_LAYER_HAND;
            input.isRockGeneratedBody = collision_layer_policy::isRockOwnedReusableLayer(record.collisionLayer);
            input.isRockWeaponSourceBody = rawBodyId == options.sourceWeaponBodyId || rawBodyId == options.sourceBodyId;
            input.isHeldBySameHand = containsBodyId(options.heldBySameHand, rawBodyId);
            input.isPlayerBody = record.resolvedRef == RE::PlayerCharacter::GetSingleton();
            input.isSelectedObjectTreeActiveGrabBody =
                options.allowSelectedObjectTreeActiveGrabLayers &&
                options.mode == physics_body_classifier::InteractionMode::ActiveGrab &&
                (seeded || nodeIsOrContains(out.rootNode, context.node) || record.resolvedRef == context.rootRef);

            const auto classification = physics_body_classifier::classifyBody(input, options.mode);
            record.accepted = classification.accepted;
            record.rejectReason = classification.reason;
            if (!record.accepted) {
                recordReject(out.diagnostics, record.rejectReason);
            }

            out.records.push_back(record);
            if (seeded) {
                ++out.diagnostics.seedBodiesAdded;
            }
            return true;
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

                ScanBodyContext context{ bhkWorld, hknpWorld, rootRef, node, collisionObject, &options, &out, &seenBodyIds };

                auto visitBody = [](std::uint32_t rawBodyId, void* userData) {
                    auto* context = static_cast<ScanBodyContext*>(userData);
                    if (!context) {
                        return false;
                    }
                    return appendBodyRecord(*context, rawBodyId, false);
                };
                const auto scanResult = havok_runtime::forEachPhysicsSystemBodyIdDetailed(collisionObject, hknpWorld, 256, visitBody, &context);
                if (!scanResult.enumerated()) {
                    if (isInvalidPhysicsSystemStatus(scanResult.status)) {
                        ++out.diagnostics.scanFailures;
                        ++out.diagnostics.invalidPhysicsSystems;
                        ROCK_LOG_SAMPLE_WARN(Hand,
                            1000,
                            "Object body scan skipped invalid physics system: status={} rootFormID={:08X} node='{}' bodyCount={} visited={} invalidIds={} seedBody={} weaponRef={}",
                            havok_runtime::physicsSystemBodyScanStatusName(scanResult.status),
                            rootRef ? rootRef->GetFormID() : 0,
                            nodeName(node),
                            scanResult.bodyCount,
                            scanResult.visitedBodies,
                            scanResult.skippedInvalidBodies,
                            options.seedBodyId,
                            isWeaponReference(rootRef) ? "yes" : "no");
                    } else {
                        ++out.diagnostics.benignScanSkips;
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

        void scanOwnerNodeWorldFallback(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            RE::TESObjectREFR* rootRef,
            RE::NiAVObject* rootNode,
            const BodySetScanOptions& options,
            ObjectPhysicsBodySet& out,
            std::unordered_set<std::uint32_t>& seenBodyIds)
        {
            /*
             * Active grab prep uses FO4VR's recursive NiAVObject wrappers, so
             * the native operation may touch selected-tree bodies whose owning
             * collision object has a stale or unreadable physics-system body-id
             * array. The fallback walks the hknp body array once and admits only
             * bodies whose native back-pointer resolves to a node under the
             * selected ref root; it does not broaden passive push or foreign-ref
             * ownership.
             */
            if (!options.allowOwnerNodeWorldFallbackScan || !hknpWorld || !rootRef || !rootNode) {
                return;
            }

            std::uint32_t highWaterMark = 0;
            if (!havok_runtime::tryReadBodyHighWaterMark(hknpWorld, highWaterMark)) {
                return;
            }

            auto* bodyArray = havok_runtime::getBodyArray(hknpWorld);
            if (!bodyArray) {
                return;
            }

            for (std::uint32_t rawBodyId = 0; rawBodyId <= highWaterMark; ++rawBodyId) {
                if (!havok_runtime::bodySlotCanBeRead(rawBodyId, highWaterMark)) {
                    continue;
                }

                ++out.diagnostics.ownerNodeFallbackSlotsScanned;
                const auto& body = bodyArray[rawBodyId];
                if (body.bodyId.value != rawBodyId || body.motionIndex == body_frame::kFreeMotionIndex) {
                    continue;
                }

                auto* ownerNode = havok_runtime::getOwnerNodeFromBody(&body);
                if (!nodeIsOrContains(rootNode, ownerNode)) {
                    continue;
                }

                ++out.diagnostics.ownerNodeFallbackCandidates;
                if (seenBodyIds.find(rawBodyId) != seenBodyIds.end()) {
                    ++out.diagnostics.ownerNodeFallbackDuplicateSkips;
                    continue;
                }

                auto* collisionObject = havok_runtime::getCollisionObjectFromBody(&body);
                ScanBodyContext fallbackContext{ bhkWorld, hknpWorld, rootRef, ownerNode, collisionObject, &options, &out, &seenBodyIds };
                const auto beforeCount = out.records.size();
                appendBodyRecord(fallbackContext, rawBodyId, false);
                if (out.records.size() > beforeCount) {
                    ++out.diagnostics.ownerNodeFallbackBodiesAdded;
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
        if (options.seedBodyId != INVALID_BODY_ID) {
            ScanBodyContext seedContext{ bhkWorld, hknpWorld, ref, options.seedHitNode ? options.seedHitNode : result.rootNode, nullptr, &options, &result, &seenBodyIds };
            appendBodyRecord(seedContext, options.seedBodyId, true);
        }

        if (options.allowWeaponRefExpansion || !isWeaponReference(ref)) {
            scanNode(bhkWorld, hknpWorld, ref, result.rootNode, (std::max)(0, options.maxDepth), options, result, seenBodyIds);
        } else {
            ++result.diagnostics.weaponExpansionSkips;
        }
        scanOwnerNodeWorldFallback(bhkWorld, hknpWorld, ref, result.rootNode, options, result, seenBodyIds);
        return result;
    }
}

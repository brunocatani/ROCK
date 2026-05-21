#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/FarSelectionBlacklistPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/object/PhysicsBodyClassifier.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/native/HavokRuntime.h"
#include "physics-interaction/native/PhysicsShapeCast.h"
#include "RockConfig.h"
#include "physics-interaction/hand/HandSelection.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/bhkCharacterController.h"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cstddef>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;
        constexpr int kMaxSelectionRejectTelemetryPerQuery = 8;

        const char* motionTypeName(physics_body_classifier::BodyMotionType motionType)
        {
            using physics_body_classifier::BodyMotionType;
            switch (motionType) {
            case BodyMotionType::Static:
                return "static";
            case BodyMotionType::Dynamic:
                return "dynamic";
            case BodyMotionType::Keyframed:
                return "keyframed";
            case BodyMotionType::Other:
                return "other";
            case BodyMotionType::Unknown:
            default:
                return "unknown";
            }
        }

        const char* nodeName(RE::NiAVObject* node)
        {
            return node && node->name.c_str() ? node->name.c_str() : "(none)";
        }

        const char* formTypeName(RE::TESObjectREFR* ref)
        {
            auto* baseForm = ref ? ref->GetObjectReference() : nullptr;
            const char* formType = baseForm ? baseForm->GetFormTypeString() : nullptr;
            return formType ? formType : "???";
        }

        struct RankedSelectionCandidate
        {
            SelectedObject selection{};
            selection_query_policy::ShapeCastCandidateScore score{};
        };

        void insertRankedSelectionCandidate(
            std::array<RankedSelectionCandidate, selection_query_policy::kMaxShapeCastPrecisionCandidates>& rankedCandidates,
            std::size_t& rankedCandidateCount,
            const SelectedObject& selection,
            const selection_query_policy::ShapeCastCandidateScore& score)
        {
            if (!selection.isValid() || !score.valid) {
                return;
            }

            std::size_t insertAt = rankedCandidateCount;
            for (std::size_t i = 0; i < rankedCandidateCount; ++i) {
                if (selection_query_policy::isBetterShapeCastCandidateScore(score, rankedCandidates[i].score)) {
                    insertAt = i;
                    break;
                }
            }

            if (insertAt >= rankedCandidates.size() && rankedCandidateCount >= rankedCandidates.size()) {
                return;
            }

            if (rankedCandidateCount < rankedCandidates.size()) {
                ++rankedCandidateCount;
            }

            for (std::size_t i = rankedCandidateCount - 1; i > insertAt; --i) {
                rankedCandidates[i] = rankedCandidates[i - 1];
            }

            rankedCandidates[insertAt] = RankedSelectionCandidate{
                .selection = selection,
                .score = score,
            };
        }

        struct SelectionRejectTelemetry
        {
            std::uint32_t layer = 0;
            std::uint32_t filterInfo = 0;
            std::uint32_t motionId = 0;
            std::uint16_t motionPropertiesId = 0;
            std::uint32_t bodyFlags = 0;
            physics_body_classifier::BodyMotionType motionType = physics_body_classifier::BodyMotionType::Unknown;
            bool hasBody = false;
            bool hitNodeInsideActorRoot = false;
        };

        SelectionRejectTelemetry makeSelectionRejectTelemetry(
            RE::TESObjectREFR* ref,
            RE::NiAVObject* hitNode,
            RE::hknpWorld* hknpWorld,
            RE::hknpBodyId bodyId)
        {
            SelectionRejectTelemetry telemetry{};

            if (ref && hitNode) {
                if (auto* root3D = ref->Get3D()) {
                    telemetry.hitNodeInsideActorRoot = actor_equipment_grab::nodeContainsNode(root3D, hitNode, 64);
                }
            }

            if (!hknpWorld || bodyId.value == kInvalidBodyId) {
                return telemetry;
            }

            auto* body = havok_runtime::getBody(hknpWorld, bodyId);
            if (!body) {
                return telemetry;
            }

            telemetry.hasBody = true;
            telemetry.filterInfo = body->collisionFilterInfo;
            telemetry.layer = body->collisionFilterInfo & 0x7F;
            telemetry.motionId = body->motionIndex;
            telemetry.bodyFlags = body->flags;
            telemetry.motionPropertiesId = static_cast<std::uint16_t>(body->motionPropertiesId);
            std::uint16_t resolvedMotionPropertiesId = 0;
            if (havok_runtime::tryReadBodyMotionPropertiesId(hknpWorld, bodyId, resolvedMotionPropertiesId)) {
                telemetry.motionPropertiesId = resolvedMotionPropertiesId;
            }
            telemetry.motionType = physics_body_classifier::motionTypeFromBodyFlags(telemetry.bodyFlags);
            if (telemetry.motionType == physics_body_classifier::BodyMotionType::Unknown) {
                telemetry.motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(telemetry.motionPropertiesId);
            }
            return telemetry;
        }

        void logSelectionRejectTelemetry(
            const char* queryName,
            const char* rejectStage,
            int hitIndex,
            RE::TESObjectREFR* ref,
            RE::NiAVObject* hitNode,
            RE::hknpWorld* hknpWorld,
            RE::hknpBodyId bodyId,
            const GrabTargetClassification* classification,
            const char* fallbackReason,
            bool isFarSelection,
            float signedAlongDistance,
            float lateralDistance,
            float hmdConeDot,
            int& loggedCount)
        {
            if (!g_rockConfig.rockDebugVerboseLogging || loggedCount >= kMaxSelectionRejectTelemetryPerQuery) {
                return;
            }

            ++loggedCount;
            const auto telemetry = makeSelectionRejectTelemetry(ref, hitNode, hknpWorld, bodyId);
            const auto targetKind = classification ? classification->kind : grab_target::Kind::None;
            const char* reason = classification && classification->reason ? classification->reason : fallbackReason;
            ROCK_LOG_DEBUG(Hand,
                "Selection reject: query={} reject-stage={} hit={} reason={} targetKind={} formType={} formID={:08X} body={} "
                "layer={} filter=0x{:08X} motionId={} motionProps={} motionType={} bodyFlags=0x{:08X} hasBody={} "
                "hitNode='{}' hitNodeInsideActorRoot={} far={} signedAlong={:.2f} lateral={:.2f} hmdDot={:.3f}",
                queryName ? queryName : "unknown",
                rejectStage ? rejectStage : "unknown",
                hitIndex,
                reason ? reason : "unknown",
                grab_target::name(targetKind),
                formTypeName(ref),
                ref ? ref->GetFormID() : 0,
                bodyId.value,
                telemetry.layer,
                telemetry.filterInfo,
                telemetry.motionId,
                telemetry.motionPropertiesId,
                motionTypeName(telemetry.motionType),
                telemetry.bodyFlags,
                telemetry.hasBody ? "yes" : "no",
                nodeName(hitNode),
                telemetry.hitNodeInsideActorRoot ? "yes" : "no",
                isFarSelection ? "yes" : "no",
                signedAlongDistance,
                lateralDistance,
                hmdConeDot);
        }

        bool isLooseGrabbableBaseType(RE::TESBoundObject* baseForm)
        {
            return baseForm &&
                   (baseForm->Is(RE::ENUM_FORM_ID::kMISC) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kWEAP) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kAMMO) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kALCH) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kBOOK) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kKEYM) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kNOTE) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kARMO) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kFLOR) ||
                       baseForm->Is(RE::ENUM_FORM_ID::kACTI));
        }

        bool hasDynamicDeadBipedBodyEvidence(RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId)
        {
            if (!hknpWorld || bodyId.value == kInvalidBodyId) {
                return false;
            }

            auto* body = havok_runtime::getBody(hknpWorld, bodyId);
            if (!body) {
                return false;
            }

            const auto layer = body->collisionFilterInfo & 0x7F;
            if (!grab_target::isDetachedGoreLayer(layer)) {
                return false;
            }

            auto motionType = physics_body_classifier::motionTypeFromBodyFlags(body->flags);
            if (motionType == physics_body_classifier::BodyMotionType::Unknown) {
                motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(static_cast<std::uint16_t>(body->motionPropertiesId));
            }
            return motionType == physics_body_classifier::BodyMotionType::Dynamic;
        }

        bool hasDynamicMovableStaticBodyEvidence(RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId)
        {
            if (!hknpWorld || bodyId.value == kInvalidBodyId) {
                return false;
            }

            auto* body = havok_runtime::getBody(hknpWorld, bodyId);
            if (!body) {
                return false;
            }

            auto motionType = physics_body_classifier::motionTypeFromBodyFlags(body->flags);
            if (motionType == physics_body_classifier::BodyMotionType::Unknown) {
                motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(static_cast<std::uint16_t>(body->motionPropertiesId));
            }
            return motionType == physics_body_classifier::BodyMotionType::Dynamic;
        }

        bool hasDynamicActorBodyEvidence(RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId)
        {
            if (!hknpWorld || bodyId.value == kInvalidBodyId) {
                return false;
            }

            auto* body = havok_runtime::getBody(hknpWorld, bodyId);
            if (!body) {
                return false;
            }

            const auto layer = body->collisionFilterInfo & 0x7F;
            if (!collision_layer_policy::isActorOrBipedLayer(layer)) {
                return false;
            }

            auto motionType = physics_body_classifier::motionTypeFromBodyFlags(body->flags);
            if (motionType == physics_body_classifier::BodyMotionType::Unknown) {
                motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(static_cast<std::uint16_t>(body->motionPropertiesId));
            }
            return motionType == physics_body_classifier::BodyMotionType::Dynamic;
        }

        bool hasDetachedGoreEvidence(RE::TESObjectREFR* ref, RE::NiAVObject* hitNode, RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId)
        {
            if (!hasDynamicDeadBipedBodyEvidence(hknpWorld, bodyId)) {
                return false;
            }

            /*
             * Whole dead actor bodies can expose dynamic DEADBIP bodies after
             * ragdolling. Detached gore is only accepted when the hit node is no
             * longer owned by the actor 3D tree; this keeps the earlier invariant
             * that a normal NPC body is not a pull target.
             */
            auto* root3D = ref ? ref->Get3D() : nullptr;
            return hitNode && root3D && !actor_equipment_grab::nodeContainsNode(root3D, hitNode, 64);
        }
    }

    GrabTargetClassification classifySelectionGrabTarget(RE::TESObjectREFR* ref,
        RE::hknpWorld* hknpWorld,
        RE::hknpBodyId bodyId,
        const OtherHandSelectionContext& otherHandContext,
        bool isFarSelection,
        RE::NiAVObject* hitNode,
        const RE::NiPoint3& hitPointWorld,
        bool hasHitPoint)
    {
        if (!ref) {
            return { .kind = grab_target::Kind::None, .reason = "null-ref", .grabbable = false };
        }

        if (ref == RE::PlayerCharacter::GetSingleton()) {
            return { .kind = grab_target::Kind::BlockedWholeActorBody, .reason = "player-ref", .grabbable = false };
        }

        if (ref->IsDeleted() || ref->IsDisabled()) {
            return { .kind = grab_target::Kind::None, .reason = "deleted-or-disabled", .grabbable = false };
        }

        if (otherHandContext.blocksReference(ref)) {
            return { .kind = grab_target::Kind::None, .reason = "reserved-by-other-hand", .grabbable = false };
        }

        auto* baseForm = ref->GetObjectReference();
        if (!baseForm) {
            return { .kind = grab_target::Kind::None, .reason = "missing-base-form", .grabbable = false };
        }

        auto* root3D = ref->Get3D();
        if (!root3D) {
            return { .kind = grab_target::Kind::None, .reason = "missing-3d", .grabbable = false };
        }

        if (baseForm->Is(RE::ENUM_FORM_ID::kNPC_)) {
            if (!ref->IsDead(false)) {
                return { .kind = grab_target::Kind::LiveActorScissors, .reason = "live-actor-requires-scissors", .grabbable = false };
            }

            if (isFarSelection) {
                auto equipmentSelection = actor_equipment_grab::resolveFarActorEquipmentSelection(ref, hitNode, hitPointWorld, hasHitPoint);
                if (equipmentSelection.isUsable()) {
                    return {
                        .kind = grab_target::Kind::ActorEquipment,
                        .reason = "far-dead-actor-equipment",
                        .grabbable = true,
                        .actorEquipment = equipmentSelection,
                    };
                }
            }

            if (hasDetachedGoreEvidence(ref, hitNode, hknpWorld, bodyId)) {
                return { .kind = grab_target::Kind::DetachedGore, .reason = "detached-deadbip-gore", .grabbable = true };
            }

            if (hasDynamicActorBodyEvidence(hknpWorld, bodyId)) {
                return { .kind = grab_target::Kind::DeadActorBody, .reason = "dead-actor-body-dynamic", .grabbable = true };
            }

            return { .kind = grab_target::Kind::BlockedWholeActorBody, .reason = isFarSelection ? "far-dead-actor-no-equipment" : "close-dead-actor-body", .grabbable = false };
        }

        if (baseForm->Is(RE::ENUM_FORM_ID::kMSTT) && hasDynamicMovableStaticBodyEvidence(hknpWorld, bodyId)) {
            return { .kind = grab_target::Kind::DynamicMovableStatic, .reason = "dynamic-mstt-body", .grabbable = true };
        }

        if (isLooseGrabbableBaseType(baseForm)) {
            if (!object_physics_body_set::hasCollisionObjectInSubtree(root3D, (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth))) {
                return { .kind = grab_target::Kind::LooseObject, .reason = "no-collision-subtree", .grabbable = false };
            }
            return { .kind = grab_target::Kind::LooseObject, .reason = "loose-form-whitelist", .grabbable = true };
        }

        if (hasDynamicDeadBipedBodyEvidence(hknpWorld, bodyId)) {
            return { .kind = grab_target::Kind::DetachedGore, .reason = "dynamic-deadbip-body", .grabbable = true };
        }

        return { .kind = grab_target::Kind::None, .reason = "unsupported-form-and-body", .grabbable = false };
    }

    bool isGrabbable(RE::TESObjectREFR* ref, const OtherHandSelectionContext& otherHandContext)
    {
        return classifySelectionGrabTarget(ref, nullptr, RE::hknpBodyId{ kInvalidBodyId }, otherHandContext).grabbable;
    }

    RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId)
    {
        if (!bhkWorld || !hknpWorld)
            return nullptr;
        if (bodyId.value == 0x7FFF'FFFF)
            return nullptr;

        auto* body = havok_runtime::getBody(hknpWorld, bodyId);
        if (!body || body->motionIndex > 4096)
            return nullptr;

        auto layer = body->collisionFilterInfo & 0x7F;
        if (layer == 30) {
            return nullptr;
        }

        if (body->userData == 0)
            return nullptr;

        auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, bodyId);
        if (!collObj)
            return nullptr;

        auto* sceneObj = collObj->sceneObject;
        if (!sceneObj)
            return nullptr;

        auto* ref = RE::TESObjectREFR::FindReferenceFor3D(sceneObj);
        return ref;
    }

    namespace
    {
        float pointDistance(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            const RE::NiPoint3 delta(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        bool normalizeGameDirection(RE::NiPoint3 value, RE::NiPoint3& out)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lengthSquared <= 1.0e-6f) {
                out = RE::NiPoint3{};
                return false;
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            out = RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
            return true;
        }

        RE::NiPoint3 hkDirectionToNiPoint(const RE::hkVector4f& value)
        {
            return RE::NiPoint3{ value.x, value.y, value.z };
        }

        float lateralDistanceToRay(const RE::NiPoint3& start, const RE::NiPoint3& directionUnit, const RE::NiPoint3& hit)
        {
            const RE::NiPoint3 startToHit(hit.x - start.x, hit.y - start.y, hit.z - start.z);
            const float along = startToHit.x * directionUnit.x + startToHit.y * directionUnit.y + startToHit.z * directionUnit.z;
            const RE::NiPoint3 lateral(startToHit.x - directionUnit.x * along, startToHit.y - directionUnit.y * along, startToHit.z - directionUnit.z * along);
            return std::sqrt(lateral.x * lateral.x + lateral.y * lateral.y + lateral.z * lateral.z);
        }

        float alongDistanceOnRay(const RE::NiPoint3& start, const RE::NiPoint3& directionUnit, const RE::NiPoint3& hit)
        {
            const RE::NiPoint3 startToHit(hit.x - start.x, hit.y - start.y, hit.z - start.z);
            return (std::max)(0.0f, startToHit.x * directionUnit.x + startToHit.y * directionUnit.y + startToHit.z * directionUnit.z);
        }

        float signedAlongDistanceOnRay(const RE::NiPoint3& start, const RE::NiPoint3& directionUnit, const RE::NiPoint3& hit)
        {
            const RE::NiPoint3 startToHit(hit.x - start.x, hit.y - start.y, hit.z - start.z);
            return startToHit.x * directionUnit.x + startToHit.y * directionUnit.y + startToHit.z * directionUnit.z;
        }

        float configuredNearReachDistance()
        {
            return g_rockConfig.rockNearCastDistanceGameUnits > 0.0f ? g_rockConfig.rockNearCastDistanceGameUnits : g_rockConfig.rockNearDetectionRange;
        }

        bool promotesFarHitToCloseSelection(const GrabTargetClassification& classification, const RE::NiPoint3& start, const RE::NiPoint3& hitPoint, float nearReachDistance)
        {
            return classification.kind != grab_target::Kind::ActorEquipment &&
                   selection_query_policy::shouldPromoteFarHitToClose(pointDistance(start, hitPoint), nearReachDistance);
        }

        SelectedObject chooseShapeCastSelection(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const RE::NiPoint3& start,
            const RE::NiPoint3& directionUnit,
            const RE::hknpAllHitsCollector& collector,
            bool isFarSelection,
            const OtherHandSelectionContext& otherHandContext,
            int& outCandidates,
            int& outRejectedInvalidBody,
            int& outRejectedNoRef,
            int& outRejectedNotGrabbable,
            int& outRejectedBehindPalm,
            int& outRejectedHmdCone,
            const FarSelectionHmdConeGate* farHmdConeGate,
            float nearPromotionDistance,
            int& outDuplicateBodies,
            bool logRejectTelemetry)
        {
            SelectedObject result;
            std::array<RankedSelectionCandidate, selection_query_policy::kMaxShapeCastPrecisionCandidates> rankedCandidates{};
            std::size_t rankedCandidateCount = 0;
            int loggedRejectTelemetry = 0;
            const char* queryName = isFarSelection ? "far" : "near";
            std::unordered_set<std::uint32_t> seenBodyIds;
            std::unordered_map<std::uint32_t, RE::TESObjectREFR*> refByBodyId;

            auto* hits = collector.hits._data;
            const int numHits = collector.hits._size;
            for (int i = 0; i < numHits; ++i) {
                const auto& hit = hits[i];
                auto hitBodyId = hit.hitBodyInfo.m_bodyId;
                if (hitBodyId.value == 0x7FFF'FFFF) {
                    ++outRejectedInvalidBody;
                    if (logRejectTelemetry) {
                        logSelectionRejectTelemetry(queryName, "invalid-body", i, nullptr, nullptr, hknpWorld, hitBodyId, nullptr, "invalid-body", isFarSelection, 0.0f, 0.0f, -1.0f,
                            loggedRejectTelemetry);
                    }
                    continue;
                }

                if (!seenBodyIds.insert(hitBodyId.value).second) {
                    ++outDuplicateBodies;
                }

                RE::TESObjectREFR* ref = nullptr;
                if (const auto cachedRef = refByBodyId.find(hitBodyId.value); cachedRef != refByBodyId.end()) {
                    ref = cachedRef->second;
                } else {
                    ref = resolveBodyToRef(bhkWorld, hknpWorld, hitBodyId);
                    refByBodyId.emplace(hitBodyId.value, ref);
                }
                if (!ref) {
                    ++outRejectedNoRef;
                    if (logRejectTelemetry) {
                        auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
                        auto* hitNode = collObj ? collObj->sceneObject : nullptr;
                        logSelectionRejectTelemetry(queryName, "no-ref", i, nullptr, hitNode, hknpWorld, hitBodyId, nullptr, "no-ref", isFarSelection, 0.0f, 0.0f, -1.0f,
                            loggedRejectTelemetry);
                    }
                    continue;
                }

                if (isFarSelection && otherHandContext.allowsSharedHeldReference(ref)) {
                    ++outRejectedNotGrabbable;
                    if (logRejectTelemetry) {
                        auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
                        auto* hitNode = collObj ? collObj->sceneObject : nullptr;
                        logSelectionRejectTelemetry(queryName, "shared-held-far", i, ref, hitNode, hknpWorld, hitBodyId, nullptr, "shared-held-far", isFarSelection, 0.0f, 0.0f, -1.0f,
                            loggedRejectTelemetry);
                    }
                    continue;
                }

                const RE::NiPoint3 hitPoint = hkVectorToNiPoint(hit.position);
                auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
                auto* hitNode = collObj ? collObj->sceneObject : nullptr;
                auto* baseForm = ref->GetObjectReference();
                const auto classification = classifySelectionGrabTarget(ref, hknpWorld, hitBodyId, otherHandContext, isFarSelection, hitNode, hitPoint, true);
                if (!classification.grabbable) {
                    ++outRejectedNotGrabbable;
                    if (logRejectTelemetry) {
                        logSelectionRejectTelemetry(queryName, "classification", i, ref, hitNode, hknpWorld, hitBodyId, &classification, "classification", isFarSelection, 0.0f, 0.0f, -1.0f,
                            loggedRejectTelemetry);
                    }
                    continue;
                }

                if (isFarSelection && !grab_target::canUseFarSelection(classification.kind)) {
                    ++outRejectedNotGrabbable;
                    if (logRejectTelemetry) {
                        const GrabTargetClassification rejectedClassification{
                            .kind = classification.kind,
                            .reason = "far-target-kind",
                            .grabbable = false,
                            .actorEquipment = classification.actorEquipment,
                        };
                        logSelectionRejectTelemetry(queryName, "far-target-kind", i, ref, hitNode, hknpWorld, hitBodyId, &rejectedClassification, "far-target-kind", isFarSelection, 0.0f, 0.0f,
                            -1.0f, loggedRejectTelemetry);
                    }
                    continue;
                }

                const bool farBlacklistConfigured =
                    isFarSelection &&
                    (!g_rockConfig.rockFarSelectionBlockedReferenceFormIds.empty() || !g_rockConfig.rockFarSelectionBlockedBaseFormIds.empty() ||
                        !g_rockConfig.rockFarSelectionBlockedFormTypes.empty() || !g_rockConfig.rockFarSelectionBlockedLayers.empty());
                if (farBlacklistConfigured) {
                    const auto farRejectTelemetry = makeSelectionRejectTelemetry(ref, hitNode, hknpWorld, hitBodyId);
                    const auto farBlacklistDecision =
                        far_selection_blacklist_policy::evaluateFarSelectionBlacklist(far_selection_blacklist_policy::FarSelectionBlacklistInput{
                            .isFarSelection = isFarSelection,
                            .referenceFormId = ref ? ref->GetFormID() : 0,
                            .baseFormId = baseForm ? baseForm->formID : 0,
                            .collisionLayer = farRejectTelemetry.layer,
                            .formType = baseForm && baseForm->GetFormTypeString() ? baseForm->GetFormTypeString() : "",
                            .blockedReferenceFormIds = g_rockConfig.rockFarSelectionBlockedReferenceFormIds,
                            .blockedBaseFormIds = g_rockConfig.rockFarSelectionBlockedBaseFormIds,
                            .blockedFormTypes = g_rockConfig.rockFarSelectionBlockedFormTypes,
                            .blockedLayers = g_rockConfig.rockFarSelectionBlockedLayers,
                        });
                    if (farBlacklistDecision.blocked) {
                        ++outRejectedNotGrabbable;
                        if (logRejectTelemetry) {
                            const GrabTargetClassification rejectedClassification{
                                .kind = classification.kind,
                                .reason = farBlacklistDecision.reason,
                                .grabbable = false,
                                .actorEquipment = classification.actorEquipment,
                            };
                            logSelectionRejectTelemetry(queryName,
                                "far-blacklist",
                                i,
                                ref,
                                hitNode,
                                hknpWorld,
                                hitBodyId,
                                &rejectedClassification,
                                farBlacklistDecision.reason,
                                isFarSelection,
                                0.0f,
                                0.0f,
                                -1.0f,
                                loggedRejectTelemetry);
                        }
                        continue;
                    }
                }

                ++outCandidates;
                const float lateralDistance = lateralDistanceToRay(start, directionUnit, hitPoint);
                const float signedAlongDistance = signedAlongDistanceOnRay(start, directionUnit, hitPoint);
                if (selection_query_policy::shouldRejectBehindPalmHit(isFarSelection, signedAlongDistance, g_rockConfig.rockCloseSelectionBehindPalmToleranceGameUnits)) {
                    ++outRejectedBehindPalm;
                    if (logRejectTelemetry) {
                        logSelectionRejectTelemetry(queryName, "behind-palm", i, ref, hitNode, hknpWorld, hitBodyId, &classification, "behind-palm", isFarSelection, signedAlongDistance,
                            lateralDistance, -1.0f, loggedRejectTelemetry);
                    }
                    continue;
                }
                const bool promotesToCloseSelection = isFarSelection && promotesFarHitToCloseSelection(classification, start, hitPoint, nearPromotionDistance);
                float hmdConeDot = -1.0f;
                bool hasHmdConeDot = false;
                if (selection_query_policy::shouldGateFarCandidateWithHmdCone(isFarSelection, promotesToCloseSelection) && farHmdConeGate) {
                    hasHmdConeDot = farHmdConeGate->enabled;
                    if (!farHmdConeGate->acceptsHitPoint(hitPoint, &hmdConeDot)) {
                        ++outRejectedHmdCone;
                        if (logRejectTelemetry) {
                            logSelectionRejectTelemetry(queryName, "hmd-cone", i, ref, hitNode, hknpWorld, hitBodyId, &classification, "hmd-cone", isFarSelection, signedAlongDistance,
                                lateralDistance, hmdConeDot, loggedRejectTelemetry);
                        }
                        continue;
                    }
                }
                const float alongDistance = alongDistanceOnRay(start, directionUnit, hitPoint);
                RE::NiPoint3 hitNormal{};
                const bool hasHitNormal = normalizeGameDirection(hkDirectionToNiPoint(hit.normal), hitNormal);
                const std::uint32_t shapeKey = hit.hitBodyInfo.m_shapeKey.storage;
                const float normalDotDirection = hasHitNormal ? hitNormal.x * directionUnit.x + hitNormal.y * directionUnit.y + hitNormal.z * directionUnit.z : 0.0f;
                const float lateralScoreScale = isFarSelection ? g_rockConfig.rockFarCastRadiusGameUnits : g_rockConfig.rockNearCastRadiusGameUnits;
                const float alongScoreScale = isFarSelection ? g_rockConfig.rockFarDetectionRange : configuredNearReachDistance();
                const auto candidateScore = selection_query_policy::scoreShapeCastCandidate(selection_query_policy::ShapeCastCandidateScoringInput{
                    .isFarSelection = isFarSelection,
                    .lateralDistance = lateralDistance,
                    .alongDistance = alongDistance,
                    .lateralScale = lateralScoreScale,
                    .alongScale = alongScoreScale,
                    .normalDotDirection = normalDotDirection,
                    .hasHitNormal = hasHitNormal,
                });
                if (!candidateScore.valid) {
                    continue;
                }

                SelectedObject candidate;
                candidate.refr = ref;
                candidate.bodyId = hitBodyId;
                candidate.hitPointWorld = hitPoint;
                candidate.hitNormalWorld = hitNormal;
                candidate.distance = alongDistance;
                candidate.signedAlongDistance = signedAlongDistance;
                candidate.lateralDistance = lateralDistance;
                candidate.hitFraction = hit.fraction.storage;
                candidate.selectionScore = candidateScore.score;
                candidate.hitShapeKey = shapeKey;
                candidate.hitShapeCollisionFilterInfo = hit.hitBodyInfo.m_shapeCollisionFilterInfo.storage;
                candidate.hmdConeDot = hmdConeDot;
                candidate.targetKind = classification.kind;
                candidate.isFarSelection = isFarSelection;
                candidate.hasHitPoint = true;
                candidate.hasHitNormal = hasHitNormal;
                candidate.hasHitShapeKey = shapeKey != 0xFFFF'FFFF;
                candidate.hasSelectionScore = true;
                candidate.hasHmdConeDot = hasHmdConeDot;
                candidate.actorEquipment = classification.actorEquipment;

                candidate.hitNode = hitNode;
                candidate.visualNode = classification.actorEquipment.visualNode ? classification.actorEquipment.visualNode : ref->Get3D();
                insertRankedSelectionCandidate(rankedCandidates, rankedCandidateCount, candidate, candidateScore);
            }

            if (rankedCandidateCount > 0) {
                result = rankedCandidates[0].selection;
            }
            return result;
        }
    }

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        const OtherHandSelectionContext& otherHandContext, const char* debugQueryName)
    {
        SelectedObject result;

        if (!bhkWorld || !hknpWorld)
            return result;

        RE::NiPoint3 direction{};
        if (!normalizeGameDirection(palmForward, direction))
            return result;

        const float configuredCastDistance = g_rockConfig.rockNearCastDistanceGameUnits > 0.0f ? g_rockConfig.rockNearCastDistanceGameUnits : nearRange;
        const float castDistance = (std::max)(0.0f, configuredCastDistance);
        const float castRadius = (std::max)(0.0f, g_rockConfig.rockNearCastRadiusGameUnits);

        RE::hknpAllHitsCollector collector;
        physics_shape_cast::SphereCastDiagnostics diagnostics;
        if (!physics_shape_cast::castSelectionSphere(
                hknpWorld,
                physics_shape_cast::SphereCastInput{ .startGame = palmPos,
                    .directionGame = direction,
                    .distanceGame = castDistance,
                    .radiusGame = castRadius,
                    .collisionFilterInfo = g_rockConfig.rockSelectionShapeCastFilterInfo },
                collector,
                &diagnostics)) {
            return result;
        }

        bool logNearMetric = false;
        if (g_rockConfig.rockDebugVerboseLogging) {
            static int nearDiagCounterRight = 0;
            static int nearDiagCounterLeft = 0;
            int& nearDiagCounter = isLeft ? nearDiagCounterLeft : nearDiagCounterRight;
            nearDiagCounter++;
            if (nearDiagCounter >= 270) {
                nearDiagCounter = 0;
                logNearMetric = true;
            }
        }

        int candidatesChecked = 0;
        int rejectedInvalidBody = 0;
        int rejectedNoRef = 0;
        int rejectedNotGrabbable = 0;
        int rejectedBehindPalm = 0;
        int rejectedHmdCone = 0;
        int duplicateBodies = 0;
        result = chooseShapeCastSelection(bhkWorld, hknpWorld, palmPos, direction, collector, false, otherHandContext, candidatesChecked, rejectedInvalidBody, rejectedNoRef,
            rejectedNotGrabbable, rejectedBehindPalm, rejectedHmdCone, nullptr, 0.0f, duplicateBodies, logNearMetric);

        if (logNearMetric) {
            ROCK_LOG_DEBUG(Hand,
                "Near shape cast [{}]: start=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) radius={:.1f} distance={:.1f} "
                "filter=0x{:08X} hits={} candidates={} dup={} rejectInvalid={} rejectNoRef={} rejectNotGrab={} rejectBehind={} selected={} formID={:08X} dist={:.2f} signedAlong={:.2f} lateral={:.2f} "
                "score={:.4f} normal=({:.2f},{:.2f},{:.2f}) shapeKey=0x{:08X}",
                debugQueryName ? debugQueryName : (isLeft ? "near-L" : "near-R"), palmPos.x, palmPos.y, palmPos.z, direction.x, direction.y, direction.z, castRadius, castDistance, diagnostics.collisionFilterInfo,
                diagnostics.hitCount, candidatesChecked, duplicateBodies, rejectedInvalidBody, rejectedNoRef, rejectedNotGrabbable, rejectedBehindPalm, result.isValid() ? "yes" : "no",
                result.refr ? result.refr->GetFormID() : 0, result.isValid() ? result.distance : -1.0f, result.isValid() ? result.signedAlongDistance : 0.0f,
                result.isValid() ? result.lateralDistance : 0.0f, result.hasSelectionScore ? result.selectionScore : -1.0f, result.hitNormalWorld.x, result.hitNormalWorld.y,
                result.hitNormalWorld.z, result.hitShapeKey);
        }

        return result;
    }

    SelectedObject findFarObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir, float farRange,
        const FarSelectionHmdConeGate& hmdConeGate,
        const OtherHandSelectionContext& otherHandContext)
    {
        SelectedObject result;

        if (!bhkWorld || !hknpWorld)
            return result;

        RE::NiPoint3 direction{};
        if (!normalizeGameDirection(pointingDir, direction))
            return result;

        float clippedFarRange = farRange;
        RE::NiPoint3 rayEnd(handPos.x + direction.x * farRange, handPos.y + direction.y * farRange, handPos.z + direction.z * farRange);

        RE::bhkPickData pickData;
        pickData.SetStartEnd(handPos, rayEnd);

        pickData.collisionFilter.filter = g_rockConfig.rockFarClipRayFilterInfo;

        if (bhkWorld->PickObject(pickData) && pickData.HasHit()) {
            clippedFarRange = (std::max)(0.0f, pickData.GetHitFraction() * farRange);
        }

        RE::hknpAllHitsCollector collector;
        physics_shape_cast::SphereCastDiagnostics diagnostics;
        if (!physics_shape_cast::castSelectionSphere(
                hknpWorld,
                physics_shape_cast::SphereCastInput{ .startGame = handPos,
                    .directionGame = direction,
                    .distanceGame = clippedFarRange,
                    .radiusGame = g_rockConfig.rockFarCastRadiusGameUnits,
                    .collisionFilterInfo = g_rockConfig.rockSelectionShapeCastFilterInfo },
                collector,
                &diagnostics)) {
            return result;
        }

        int candidatesChecked = 0;
        int rejectedInvalidBody = 0;
        int rejectedNoRef = 0;
        int rejectedNotGrabbable = 0;
        int rejectedBehindPalm = 0;
        int rejectedHmdCone = 0;
        int duplicateBodies = 0;
        const float configuredNearReach = configuredNearReachDistance();
        bool logFarMetric = false;
        if (g_rockConfig.rockDebugVerboseLogging) {
            static int farDiagCounter = 0;
            if (++farDiagCounter >= 270) {
                farDiagCounter = 0;
                logFarMetric = true;
            }
        }
        result = chooseShapeCastSelection(bhkWorld, hknpWorld, handPos, direction, collector, true, otherHandContext, candidatesChecked, rejectedInvalidBody, rejectedNoRef,
            rejectedNotGrabbable, rejectedBehindPalm, rejectedHmdCone, &hmdConeGate, configuredNearReach, duplicateBodies, logFarMetric);

        // Near reach remains collision-query based even when the directional close cast misses.
        // Released objects can rest beside or partly behind the palm, where the far sphere cast
        // sees a hit at the hand origin. Treating that as a pull creates a false "can't re-grab"
        // state; ROCK keeps active hand-reachable objects on the close-grab path.
        if (result.isValid() && result.hasHitPoint && result.targetKind != grab_target::Kind::ActorEquipment) {
            const float hitDistance = pointDistance(handPos, result.hitPointWorld);
            if (selection_query_policy::shouldPromoteFarHitToClose(hitDistance, configuredNearReach)) {
                result.isFarSelection = false;
                result.distance = hitDistance;
                const float normalDotDirection =
                    result.hasHitNormal ? result.hitNormalWorld.x * direction.x + result.hitNormalWorld.y * direction.y + result.hitNormalWorld.z * direction.z : 0.0f;
                const auto promotedCloseScore = selection_query_policy::scoreShapeCastCandidate(selection_query_policy::ShapeCastCandidateScoringInput{
                    .isFarSelection = false,
                    .lateralDistance = result.lateralDistance,
                    .alongDistance = hitDistance,
                    .lateralScale = g_rockConfig.rockNearCastRadiusGameUnits,
                    .alongScale = configuredNearReach,
                    .normalDotDirection = normalDotDirection,
                    .hasHitNormal = result.hasHitNormal,
                });
                result.hasSelectionScore = promotedCloseScore.valid;
                result.selectionScore = promotedCloseScore.valid ? promotedCloseScore.score : FLT_MAX;
            }
        }

        if (logFarMetric) {
                ROCK_LOG_DEBUG(Hand,
                    "Far shape cast: start=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) radius={:.1f} distance={:.1f}/{:.1f} "
                    "filter=0x{:08X} hits={} candidates={} dup={} rejectInvalid={} rejectNoRef={} rejectNotGrab={} rejectBehind={} rejectHmdCone={} hmdGate={} hmdDot={:.3f} selected={} formID={:08X} dist={:.1f} signedAlong={:.1f} lateral={:.1f} "
                    "score={:.4f} normal=({:.2f},{:.2f},{:.2f}) shapeKey=0x{:08X}",
                    handPos.x, handPos.y, handPos.z, direction.x, direction.y, direction.z, g_rockConfig.rockFarCastRadiusGameUnits, clippedFarRange, farRange,
                    diagnostics.collisionFilterInfo, diagnostics.hitCount, candidatesChecked, duplicateBodies, rejectedInvalidBody, rejectedNoRef, rejectedNotGrabbable,
                    rejectedBehindPalm, rejectedHmdCone, hmdConeGate.enabled ? (hmdConeGate.hasHmdFrame ? "ready" : "missing") : "off",
                    result.hasHmdConeDot ? result.hmdConeDot : -1.0f,
                    result.isValid() ? "yes" : "no", result.refr ? result.refr->GetFormID() : 0, result.isValid() ? result.distance : -1.0f,
                    result.isValid() ? result.signedAlongDistance : 0.0f, result.isValid() ? result.lateralDistance : 0.0f, result.hasSelectionScore ? result.selectionScore : -1.0f,
                    result.hitNormalWorld.x, result.hitNormalWorld.y,
                    result.hitNormalWorld.z, result.hitShapeKey);
        }

        return result;
    }
}

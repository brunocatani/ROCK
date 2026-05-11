#include "physics-interaction/object/ObjectDetection.h"
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
#include <cfloat>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace rock
{
    namespace
    {
        constexpr std::uint32_t kInvalidBodyId = 0x7FFF'FFFF;

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

            return { .kind = grab_target::Kind::BlockedWholeActorBody, .reason = isFarSelection ? "far-dead-actor-no-equipment" : "close-dead-actor-body", .grabbable = false };
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
            int& outDuplicateBodies)
        {
            SelectedObject result;
            float bestLateralDistance = FLT_MAX;
            float bestAlongDistance = FLT_MAX;
            std::unordered_set<std::uint32_t> seenBodyIds;
            std::unordered_map<std::uint32_t, RE::TESObjectREFR*> refByBodyId;

            auto* hits = collector.hits._data;
            const int numHits = collector.hits._size;
            for (int i = 0; i < numHits; ++i) {
                const auto& hit = hits[i];
                auto hitBodyId = hit.hitBodyInfo.m_bodyId;
                if (hitBodyId.value == 0x7FFF'FFFF) {
                    ++outRejectedInvalidBody;
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
                    continue;
                }

                if (isFarSelection && otherHandContext.allowsSharedHeldReference(ref)) {
                    ++outRejectedNotGrabbable;
                    continue;
                }

                const RE::NiPoint3 hitPoint = hkVectorToNiPoint(hit.position);
                auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
                auto* hitNode = collObj ? collObj->sceneObject : nullptr;
                const auto classification = classifySelectionGrabTarget(ref, hknpWorld, hitBodyId, otherHandContext, isFarSelection, hitNode, hitPoint, true);
                if (!classification.grabbable) {
                    ++outRejectedNotGrabbable;
                    continue;
                }

                ++outCandidates;
                const float lateralDistance = lateralDistanceToRay(start, directionUnit, hitPoint);
                const float signedAlongDistance = signedAlongDistanceOnRay(start, directionUnit, hitPoint);
                if (selection_query_policy::shouldRejectBehindPalmHit(isFarSelection, signedAlongDistance, g_rockConfig.rockCloseSelectionBehindPalmToleranceGameUnits)) {
                    ++outRejectedBehindPalm;
                    continue;
                }
                const bool promotesToCloseSelection = isFarSelection && promotesFarHitToCloseSelection(classification, start, hitPoint, nearPromotionDistance);
                float hmdConeDot = -1.0f;
                bool hasHmdConeDot = false;
                if (selection_query_policy::shouldGateFarCandidateWithHmdCone(isFarSelection, promotesToCloseSelection) && farHmdConeGate) {
                    hasHmdConeDot = farHmdConeGate->enabled;
                    if (!farHmdConeGate->acceptsHitPoint(hitPoint, &hmdConeDot)) {
                        ++outRejectedHmdCone;
                        continue;
                    }
                }
                const float alongDistance = alongDistanceOnRay(start, directionUnit, hitPoint);
                if (!selection_query_policy::isBetterShapeCastCandidate(isFarSelection, lateralDistance, alongDistance, bestLateralDistance, bestAlongDistance)) {
                    continue;
                }

                RE::NiPoint3 hitNormal{};
                const bool hasHitNormal = normalizeGameDirection(hkDirectionToNiPoint(hit.normal), hitNormal);
                const std::uint32_t shapeKey = hit.hitBodyInfo.m_shapeKey.storage;

                bestLateralDistance = lateralDistance;
                bestAlongDistance = alongDistance;
                result.refr = ref;
                result.bodyId = hitBodyId;
                result.hitPointWorld = hitPoint;
                result.hitNormalWorld = hitNormal;
                result.distance = alongDistance;
                result.signedAlongDistance = signedAlongDistance;
                result.lateralDistance = lateralDistance;
                result.hitFraction = hit.fraction.storage;
                result.hitShapeKey = shapeKey;
                result.hitShapeCollisionFilterInfo = hit.hitBodyInfo.m_shapeCollisionFilterInfo.storage;
                result.hmdConeDot = hmdConeDot;
                result.targetKind = classification.kind;
                result.isFarSelection = isFarSelection;
                result.hasHitPoint = true;
                result.hasHitNormal = hasHitNormal;
                result.hasHitShapeKey = shapeKey != 0xFFFF'FFFF;
                result.hasHmdConeDot = hasHmdConeDot;
                result.actorEquipment = classification.actorEquipment;

                result.hitNode = hitNode;
                result.visualNode = classification.actorEquipment.visualNode ? classification.actorEquipment.visualNode : ref->Get3D();
            }

            return result;
        }
    }

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        const OtherHandSelectionContext& otherHandContext)
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
            rejectedNotGrabbable, rejectedBehindPalm, rejectedHmdCone, nullptr, 0.0f, duplicateBodies);

        if (logNearMetric) {
            ROCK_LOG_DEBUG(Hand,
                "Near shape cast [{}]: start=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) radius={:.1f} distance={:.1f} "
                "filter=0x{:08X} hits={} candidates={} dup={} rejectInvalid={} rejectNoRef={} rejectNotGrab={} rejectBehind={} selected={} formID={:08X} dist={:.2f} signedAlong={:.2f} lateral={:.2f} "
                "normal=({:.2f},{:.2f},{:.2f}) shapeKey=0x{:08X}",
                isLeft ? "L" : "R", palmPos.x, palmPos.y, palmPos.z, direction.x, direction.y, direction.z, castRadius, castDistance, diagnostics.collisionFilterInfo,
                diagnostics.hitCount, candidatesChecked, duplicateBodies, rejectedInvalidBody, rejectedNoRef, rejectedNotGrabbable, rejectedBehindPalm, result.isValid() ? "yes" : "no",
                result.refr ? result.refr->GetFormID() : 0, result.isValid() ? result.distance : -1.0f, result.isValid() ? result.signedAlongDistance : 0.0f,
                result.isValid() ? result.lateralDistance : 0.0f, result.hitNormalWorld.x, result.hitNormalWorld.y,
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
        result = chooseShapeCastSelection(bhkWorld, hknpWorld, handPos, direction, collector, true, otherHandContext, candidatesChecked, rejectedInvalidBody, rejectedNoRef,
            rejectedNotGrabbable, rejectedBehindPalm, rejectedHmdCone, &hmdConeGate, configuredNearReach, duplicateBodies);

        // Near reach remains collision-query based even when the directional close cast misses.
        // Released objects can rest beside or partly behind the palm, where the far sphere cast
        // sees a hit at the hand origin. Treating that as a pull creates a false "can't re-grab"
        // state; ROCK keeps active hand-reachable objects on the close-grab path.
        if (result.isValid() && result.hasHitPoint && result.targetKind != grab_target::Kind::ActorEquipment) {
            const float hitDistance = pointDistance(handPos, result.hitPointWorld);
            if (selection_query_policy::shouldPromoteFarHitToClose(hitDistance, configuredNearReach)) {
                result.isFarSelection = false;
                result.distance = hitDistance;
            }
        }

        if (g_rockConfig.rockDebugVerboseLogging) {
            static int farDiagCounter = 0;
            if (++farDiagCounter >= 270) {
                farDiagCounter = 0;
                ROCK_LOG_DEBUG(Hand,
                    "Far shape cast: start=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) radius={:.1f} distance={:.1f}/{:.1f} "
                    "filter=0x{:08X} hits={} candidates={} dup={} rejectInvalid={} rejectNoRef={} rejectNotGrab={} rejectBehind={} rejectHmdCone={} hmdGate={} hmdDot={:.3f} selected={} formID={:08X} dist={:.1f} signedAlong={:.1f} lateral={:.1f} "
                    "normal=({:.2f},{:.2f},{:.2f}) shapeKey=0x{:08X}",
                    handPos.x, handPos.y, handPos.z, direction.x, direction.y, direction.z, g_rockConfig.rockFarCastRadiusGameUnits, clippedFarRange, farRange,
                    diagnostics.collisionFilterInfo, diagnostics.hitCount, candidatesChecked, duplicateBodies, rejectedInvalidBody, rejectedNoRef, rejectedNotGrabbable,
                    rejectedBehindPalm, rejectedHmdCone, hmdConeGate.enabled ? (hmdConeGate.hasHmdFrame ? "ready" : "missing") : "off",
                    result.hasHmdConeDot ? result.hmdConeDot : -1.0f,
                    result.isValid() ? "yes" : "no", result.refr ? result.refr->GetFormID() : 0, result.isValid() ? result.distance : -1.0f,
                    result.isValid() ? result.signedAlongDistance : 0.0f, result.isValid() ? result.lateralDistance : 0.0f, result.hitNormalWorld.x, result.hitNormalWorld.y,
                    result.hitNormalWorld.z, result.hitShapeKey);
            }
        }

        return result;
    }
}

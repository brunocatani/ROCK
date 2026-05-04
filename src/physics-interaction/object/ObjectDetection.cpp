#include "physics-interaction/object/ObjectDetection.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/hand/HandFrame.h"
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

    bool isGrabbable(RE::TESObjectREFR* ref, RE::TESObjectREFR* otherHandRef)
    {
        if (!ref)
            return false;

        if (ref == RE::PlayerCharacter::GetSingleton())
            return false;

        if (ref->IsDeleted() || ref->IsDisabled())
            return false;

        if (otherHandRef && ref == otherHandRef)
            return false;

        auto* baseForm = ref->GetObjectReference();
        if (!baseForm)
            return false;

        bool isGrabbableType = baseForm->Is(RE::ENUM_FORM_ID::kMISC) || baseForm->Is(RE::ENUM_FORM_ID::kWEAP) || baseForm->Is(RE::ENUM_FORM_ID::kAMMO) ||
            baseForm->Is(RE::ENUM_FORM_ID::kALCH) || baseForm->Is(RE::ENUM_FORM_ID::kBOOK) || baseForm->Is(RE::ENUM_FORM_ID::kKEYM) || baseForm->Is(RE::ENUM_FORM_ID::kNOTE) ||
            baseForm->Is(RE::ENUM_FORM_ID::kARMO) || baseForm->Is(RE::ENUM_FORM_ID::kFLOR) || baseForm->Is(RE::ENUM_FORM_ID::kACTI);

        if (!isGrabbableType)
            return false;

        auto* root3D = ref->Get3D();
        if (!root3D)
            return false;

        if (!object_physics_body_set::hasCollisionObjectInSubtree(root3D, (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth)))
            return false;

        return true;
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

        SelectedObject chooseShapeCastSelection(RE::bhkWorld* bhkWorld,
            RE::hknpWorld* hknpWorld,
            const RE::NiPoint3& start,
            const RE::NiPoint3& directionUnit,
            const RE::hknpAllHitsCollector& collector,
            bool isFarSelection,
            RE::TESObjectREFR* otherHandRef,
            int& outCandidates,
            int& outRejectedInvalidBody,
            int& outRejectedNoRef,
            int& outRejectedNotGrabbable,
            int& outDuplicateBodies)
        {
            SelectedObject result;
            float bestLateralDistance = FLT_MAX;
            float bestAlongDistance = FLT_MAX;
            std::unordered_set<std::uint32_t> seenBodyIds;
            std::unordered_map<std::uint32_t, RE::TESObjectREFR*> refByBodyId;
            std::unordered_map<std::uint32_t, bool> grabbableByBodyId;

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

                bool grabbable = false;
                if (const auto cachedGrabbable = grabbableByBodyId.find(hitBodyId.value); cachedGrabbable != grabbableByBodyId.end()) {
                    grabbable = cachedGrabbable->second;
                } else {
                    grabbable = isGrabbable(ref, otherHandRef);
                    grabbableByBodyId.emplace(hitBodyId.value, grabbable);
                }
                if (!grabbable) {
                    ++outRejectedNotGrabbable;
                    continue;
                }

                ++outCandidates;
                const RE::NiPoint3 hitPoint = hkVectorToNiPoint(hit.position);
                const float lateralDistance = lateralDistanceToRay(start, directionUnit, hitPoint);
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
                result.hitFraction = hit.fraction.storage;
                result.hitShapeKey = shapeKey;
                result.hitShapeCollisionFilterInfo = hit.hitBodyInfo.m_shapeCollisionFilterInfo.storage;
                result.isFarSelection = isFarSelection;
                result.hasHitPoint = true;
                result.hasHitNormal = hasHitNormal;
                result.hasHitShapeKey = shapeKey != 0xFFFF'FFFF;

                auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
                result.hitNode = collObj ? collObj->sceneObject : nullptr;
                result.visualNode = ref->Get3D();
            }

            return result;
        }
    }

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        RE::TESObjectREFR* otherHandRef)
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
        int duplicateBodies = 0;
        result = chooseShapeCastSelection(bhkWorld, hknpWorld, palmPos, direction, collector, false, otherHandRef, candidatesChecked, rejectedInvalidBody, rejectedNoRef,
            rejectedNotGrabbable, duplicateBodies);

        if (logNearMetric) {
            ROCK_LOG_DEBUG(Hand,
                "Near shape cast [{}]: start=({:.1f},{:.1f},{:.1f}) dir=({:.2f},{:.2f},{:.2f}) radius={:.1f} distance={:.1f} "
                "filter=0x{:08X} hits={} candidates={} dup={} rejectInvalid={} rejectNoRef={} rejectNotGrab={} selected={} formID={:08X} dist={:.2f} "
                "normal=({:.2f},{:.2f},{:.2f}) shapeKey=0x{:08X}",
                isLeft ? "L" : "R", palmPos.x, palmPos.y, palmPos.z, direction.x, direction.y, direction.z, castRadius, castDistance, diagnostics.collisionFilterInfo,
                diagnostics.hitCount, candidatesChecked, duplicateBodies, rejectedInvalidBody, rejectedNoRef, rejectedNotGrabbable, result.isValid() ? "yes" : "no",
                result.refr ? result.refr->GetFormID() : 0, result.isValid() ? result.distance : -1.0f, result.hitNormalWorld.x, result.hitNormalWorld.y,
                result.hitNormalWorld.z, result.hitShapeKey);
        }

        return result;
    }

    SelectedObject findFarObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir, float farRange,
        RE::TESObjectREFR* otherHandRef)
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
        int duplicateBodies = 0;
        result = chooseShapeCastSelection(bhkWorld, hknpWorld, handPos, direction, collector, true, otherHandRef, candidatesChecked, rejectedInvalidBody, rejectedNoRef,
            rejectedNotGrabbable, duplicateBodies);

        // Near reach remains collision-query based even when the directional close cast misses.
        // Released objects can rest beside or partly behind the palm, where the far sphere cast
        // sees a hit at the hand origin. Treating that as a pull creates a false "can't re-grab"
        // state; HIGGS keeps active hand-reachable objects on the close-grab path.
        const float configuredNearReach = g_rockConfig.rockNearCastDistanceGameUnits > 0.0f ? g_rockConfig.rockNearCastDistanceGameUnits : g_rockConfig.rockNearDetectionRange;
        if (result.isValid() && result.hasHitPoint) {
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
                    "filter=0x{:08X} hits={} candidates={} dup={} rejectInvalid={} rejectNoRef={} rejectNotGrab={} selected={} formID={:08X} dist={:.1f} "
                    "normal=({:.2f},{:.2f},{:.2f}) shapeKey=0x{:08X}",
                    handPos.x, handPos.y, handPos.z, direction.x, direction.y, direction.z, g_rockConfig.rockFarCastRadiusGameUnits, clippedFarRange, farRange,
                    diagnostics.collisionFilterInfo, diagnostics.hitCount, candidatesChecked, duplicateBodies, rejectedInvalidBody, rejectedNoRef, rejectedNotGrabbable,
                    result.isValid() ? "yes" : "no", result.refr ? result.refr->GetFormID() : 0, result.isValid() ? result.distance : -1.0f,
                    result.hitNormalWorld.x, result.hitNormalWorld.y, result.hitNormalWorld.z, result.hitShapeKey);
            }
        }

        return result;
    }
}

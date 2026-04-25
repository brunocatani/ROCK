#include "ObjectDetection.h"
#include "MeshGrab.h"
#include "PalmTransform.h"
#include "RockConfig.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Bethesda/bhkCharacterController.h"

namespace frik::rock
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

        auto* collObj = root3D->collisionObject.get();
        if (!collObj)
            return false;

        return true;
    }

    RE::TESObjectREFR* resolveBodyToRef(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, RE::hknpBodyId bodyId)
    {
        if (!bhkWorld || !hknpWorld)
            return nullptr;
        if (bodyId.value == 0x7FFF'FFFF)
            return nullptr;

        auto& body = hknpWorld->GetBody(bodyId);
        if (body.motionIndex > 4096)
            return nullptr;

        if (body.motionIndex == 0)
            return nullptr;

        auto layer = body.collisionFilterInfo & 0x7F;
        if (layer == 30) {
            return nullptr;
        }

        if (body.userData == 0)
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

    SelectedObject findCloseObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& palmPos, const RE::NiPoint3& palmForward, float nearRange, bool isLeft,
        RE::TESObjectREFR* otherHandRef)
    {
        SelectedObject result;

        if (!bhkWorld || !hknpWorld)
            return result;

        const float halfRange = nearRange * kGameToHavokScale;
        auto palmHk = niPointToHkVector(palmPos);

        RE::hknpAabbQuery query{};
        query.filterRef = getQueryFilterRef(hknpWorld);
        query.materialId = 0xFFFF;

        query.collisionFilterInfo = (0x000B << 16) | 45;

        query.aabbMin[0] = palmHk.x - halfRange;
        query.aabbMin[1] = palmHk.y - halfRange;
        query.aabbMin[2] = palmHk.z - halfRange;
        query.aabbMin[3] = 0.0f;
        query.aabbMax[0] = palmHk.x + halfRange;
        query.aabbMax[1] = palmHk.y + halfRange;
        query.aabbMax[2] = palmHk.z + halfRange;
        query.aabbMax[3] = 0.0f;

        RE::hknpAllHitsCollector collector;
        hknpWorld->QueryAabb(&query, &collector);

        int numHits = collector.hits._size;

        bool logNearMetric = false;
        if (g_rockConfig.rockDebugVerboseLogging) {
            static int nearDiagCounterRight = 0;
            static int nearDiagCounterLeft = 0;
            int& nearDiagCounter = isLeft ? nearDiagCounterLeft : nearDiagCounterRight;
            nearDiagCounter++;
            if (nearDiagCounter >= 270) {
                nearDiagCounter = 0;
                logNearMetric = true;
                ROCK_LOG_INFO(Hand, "Near AABB [{}]: palm=({:.2f},{:.2f},{:.2f}) halfRange={:.3f} hits={} filterRef={}", isLeft ? "L" : "R", palmHk.x, palmHk.y, palmHk.z,
                    halfRange, numHits, query.filterRef);
            }
        }

        if (numHits == 0)
            return result;

        float bestDist = FLT_MAX;
        auto* hits = collector.hits._data;
        int candidatesChecked = 0;
        int meshMetricCount = 0;
        int bodyFallbackCount = 0;
        int no3DCount = 0;
        int noTrianglesCount = 0;
        int noClosestPointCount = 0;
        const char* bestMetricMode = "none";
        const char* bestFallbackReason = "none";
        std::size_t bestTriangleCount = 0;
        MeshExtractionStats bestMeshStats;

        for (int i = 0; i < numHits; i++) {
            auto hitBodyId = hits[i].hitBodyInfo.m_bodyId;

            if (hitBodyId.value == 0x7FFF'FFFF)
                continue;

            auto* motion = hknpWorld->GetBodyMotion(hitBodyId);
            if (!motion)
                continue;

            auto* ref = resolveBodyToRef(bhkWorld, hknpWorld, hitBodyId);
            if (!ref)
                continue;

            if (!isGrabbable(ref, otherHandRef))
                continue;
            candidatesChecked++;

            float dist = FLT_MAX;
            const char* metricMode = "bodyComFallback";
            const char* fallbackReason = "none";
            bool hasFallbackReason = false;
            std::size_t candidateTriangleCount = 0;
            MeshExtractionStats candidateMeshStats;
            auto* node3D = ref->Get3D();
            if (node3D) {
                std::vector<TriangleData> triangles;
                extractAllTriangles(node3D, triangles, 10, &candidateMeshStats);
                candidateTriangleCount = triangles.size();

                if (!triangles.empty()) {
                    GrabPoint grabPt;
                    if (findClosestGrabPoint(triangles, palmPos, palmForward, 1.0f, 1.0f, grabPt)) {
                        dist = std::sqrt(grabPt.distance);
                        metricMode = "meshSurface";
                        meshMetricCount++;
                    } else {
                        noClosestPointCount++;
                        fallbackReason = "noClosestSurfacePoint";
                        hasFallbackReason = true;
                    }
                } else {
                    noTrianglesCount++;
                    fallbackReason = "noTriangles";
                    hasFallbackReason = true;
                }
            } else {
                no3DCount++;
                fallbackReason = "no3D";
                hasFallbackReason = true;
            }

            if (dist >= FLT_MAX - 1.0f) {
                if (!hasFallbackReason) {
                    fallbackReason = "unresolvedMeshDistance";
                }
                RE::NiPoint3 objPos = hkVectorToNiPoint(motion->position);
                dist = (objPos - palmPos).Length();
                bodyFallbackCount++;
            }

            if (dist > nearRange)
                continue;

            if (dist < bestDist) {
                bestDist = dist;
                result.refr = ref;
                result.bodyId = hitBodyId;
                result.distance = dist;
                result.isFarSelection = false;
                bestMetricMode = metricMode;
                bestFallbackReason = fallbackReason;
                bestTriangleCount = candidateTriangleCount;
                bestMeshStats = candidateMeshStats;

                auto* collObj = RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId);
                result.hitNode = collObj ? collObj->sceneObject : nullptr;
                result.visualNode = node3D;
            }
        }

        if (logNearMetric) {
            const float loggedBestDist = result.refr ? bestDist : -1.0f;
            ROCK_LOG_INFO(Hand,
                "Near metric [{}]: candidates={} meshSurface={} bodyComFallback={} no3D={} noTriangles={} noClosestPoint={} "
                "bestMode={} bestFallbackReason={} bestDist={:.1f} bestTris={} bestShapes={} "
                "bestStatic={}/{} bestDynamic={}/{} bestSkinned={}/{} bestDynamicSkinnedSkipped={}",
                isLeft ? "L" : "R", candidatesChecked, meshMetricCount, bodyFallbackCount, no3DCount, noTrianglesCount, noClosestPointCount, bestMetricMode, bestFallbackReason,
                loggedBestDist, bestTriangleCount, bestMeshStats.visitedShapes, bestMeshStats.staticShapes, bestMeshStats.staticTriangles, bestMeshStats.dynamicShapes,
                bestMeshStats.dynamicTriangles, bestMeshStats.skinnedShapes, bestMeshStats.skinnedTriangles, bestMeshStats.dynamicSkinnedSkipped);
        }

        return result;
    }

    SelectedObject findFarObject(RE::bhkWorld* bhkWorld, RE::hknpWorld* hknpWorld, const RE::NiPoint3& handPos, const RE::NiPoint3& pointingDir, float farRange,
        RE::TESObjectREFR* otherHandRef)
    {
        SelectedObject result;

        if (!bhkWorld || !hknpWorld)
            return result;

        RE::NiPoint3 rayEnd(handPos.x + pointingDir.x * farRange, handPos.y + pointingDir.y * farRange, handPos.z + pointingDir.z * farRange);

        RE::bhkPickData pickData;
        pickData.SetStartEnd(handPos, rayEnd);

        pickData.collisionFilter.filter = 0x02420028;

        if (!bhkWorld->PickObject(pickData))
            return result;
        if (!pickData.HasHit())
            return result;

        auto* hitNiObj = pickData.GetNiAVObject();
        if (!hitNiObj)
            return result;

        RE::hknpBodyId hitBodyId{ 0x7FFF'FFFF };
        auto* hitBody = pickData.GetBody();
        if (hitBody) {
            hitBodyId = hitBody->bodyId;
        }

        auto* collObj = hitBodyId.value != 0x7FFF'FFFF ? RE::bhkNPCollisionObject::Getbhk(bhkWorld, hitBodyId) : nullptr;
        auto* ownerNode = collObj ? collObj->sceneObject : nullptr;
        auto* refNode = ownerNode ? ownerNode : hitNiObj;

        auto* ref = RE::TESObjectREFR::FindReferenceFor3D(refNode);
        if (!ref && refNode != hitNiObj) {
            ref = RE::TESObjectREFR::FindReferenceFor3D(hitNiObj);
        }
        if (!ref)
            return result;

        if (!isGrabbable(ref, otherHandRef))
            return result;

        float hitFraction = pickData.GetHitFraction();
        float hitDist = hitFraction * farRange;

        result.refr = ref;
        result.bodyId = hitBodyId;
        result.hitNode = ownerNode ? ownerNode : hitNiObj;
        result.visualNode = hitNiObj;
        result.distance = hitDist;
        result.isFarSelection = true;

        return result;
    }
}

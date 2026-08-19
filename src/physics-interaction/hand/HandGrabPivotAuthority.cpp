#include "physics-interaction/hand/HandGrabPivotAuthority.h"

#include "physics-interaction/hand/HandGrabMath.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace rock::hand_grab_detail
{
    float computeLocalMeshMaxDistanceFromPoint(const std::vector<GrabLocalTriangle>& localTriangles, const RE::NiPoint3& originLocal)
    {
        if (localTriangles.empty()) {
            return 0.0f;
        }
    
        float maxDistanceSquared = 0.0f;
        auto visit = [&](const RE::NiPoint3& point) {
            const RE::NiPoint3 delta = point - originLocal;
            const float distanceSquared = delta.x * delta.x + delta.y * delta.y + delta.z * delta.z;
            if (std::isfinite(distanceSquared)) {
                maxDistanceSquared = (std::max)(maxDistanceSquared, distanceSquared);
            }
        };
        for (const auto& triangle : localTriangles) {
            visit(triangle.v0);
            visit(triangle.v1);
            visit(triangle.v2);
        }
        return maxDistanceSquared > 0.0f ? std::sqrt(maxDistanceSquared) : 0.0f;
    }
    
    const char* grabPivotAuthoritySourceName(GrabPivotAuthoritySource source)
    {
        switch (source) {
        case GrabPivotAuthoritySource::AuthoredGrabNode:
            return "authoredGrabNode";
        case GrabPivotAuthoritySource::ContactPatchMeshSnap:
            return "contactPatchMeshSnap";
        case GrabPivotAuthoritySource::ContactPatchPositionOnly:
            return "contactPatchPositionOnly";
        case GrabPivotAuthoritySource::SelectionHitMeshSnap:
            return "selectionHitMeshSnap";
        case GrabPivotAuthoritySource::PalmPocketMeshPoint:
            return "palmPocketMeshPoint";
        case GrabPivotAuthoritySource::PinchPocketMeshPoint:
            return "pinchPocketMeshPoint";
        case GrabPivotAuthoritySource::GripSupportModel:
            return "gripSupportModel";
        case GrabPivotAuthoritySource::LooseWeaponPrimaryAttach:
            return "looseWeaponPrimaryAttach";
        case GrabPivotAuthoritySource::PalmRayMeshPoint:
            return "palmRayMeshPoint";
        case GrabPivotAuthoritySource::CollisionFallback:
            return "collisionFallback";
        default:
            return "none";
        }
    }
    
    void applyFrozenGrabAuthorityFrameToGrabFrame(
        CanonicalGrabFrame& frame,
        const grab_authority_frame_math::FrozenGrabAuthorityFrame<RE::NiTransform>& frozen)
    {
        if (!frozen.valid) {
            return;
        }
    
        frame.rawHandSpace = frozen.rawHandSpace;
        frame.handBodyToRawHandAtGrab = frozen.handBodyToRawHandAtGrab;
        frame.proxyAuthorityHandSpace = frozen.proxyAuthorityHandSpace;
        frame.proxyAuthorityBodyHandSpace = frozen.proxyAuthorityBodyHandSpace;
        frame.bodyLocal = frozen.bodyLocal;
        frame.bodyWorldAtGrab = frozen.bodyWorldAtGrab;
        frame.rootBodyLocal = frozen.rootBodyLocal;
        frame.ownerBodyLocal = frozen.ownerBodyLocal;
        frame.gripPointLocal = frozen.gripPointLocal;
        frame.gripPointBodyLocalGame = frozen.pivotBBodyLocalGame;
        frame.pivotBBodyLocalGame = frozen.pivotBBodyLocalGame;
        frame.pivotBConstraintLocalGame = frozen.pivotBConstraintLocalGame;
        frame.pivotAHandBodyLocalGame = frozen.pivotAHandBodyLocalGame;
        frame.grabPivotWorldAtGrab = frozen.grabPivotWorldAtGrab;
        frame.gripPointWorldAtGrab = frozen.gripPointWorldAtGrab;
        frame.desiredObjectWorldAtGrab = frozen.desiredObjectWorld;
        frame.desiredBodyWorldAtGrab = frozen.desiredBodyWorld;
        frame.hasFrozenPivotB = true;
        frame.hasGripPoint = true;
    }
    
    GrabPivotAuthoritySource inferGrabPivotAuthoritySource(const char* mode, bool positionOnlyPatch)
    {
        if (!mode) {
            return GrabPivotAuthoritySource::None;
        }
        if (std::strcmp(mode, "grabNodeAnchor") == 0 || std::strcmp(mode, "authoredGrabNodeFrame") == 0) {
            return GrabPivotAuthoritySource::AuthoredGrabNode;
        }
        if (std::strcmp(mode, "contactPatchMeshSnap") == 0 || std::strcmp(mode, "contactPatchMeshSnapPositionOnly") == 0 ||
            std::strcmp(mode, "contactPatch") == 0 || std::strcmp(mode, "contactPatchSamplePivot") == 0) {
            return positionOnlyPatch ? GrabPivotAuthoritySource::ContactPatchPositionOnly : GrabPivotAuthoritySource::ContactPatchMeshSnap;
        }
        if (std::strcmp(mode, "selectionHitMeshSnap") == 0 || std::strcmp(mode, "contactPatchSelectedHitPivot") == 0) {
            return GrabPivotAuthoritySource::SelectionHitMeshSnap;
        }
        if (std::strcmp(mode, "palmPocketMeshSurface") == 0) {
            return GrabPivotAuthoritySource::PalmPocketMeshPoint;
        }
        if (std::strcmp(mode, "pinchPocket") == 0) {
            return GrabPivotAuthoritySource::PinchPocketMeshPoint;
        }
        if (std::strcmp(mode, "gripSupportForcedSinglePoint") == 0 ||
            std::strcmp(mode, "gripSupportForcedSameSurface") == 0 ||
            std::strcmp(mode, "gripSupportOpposedPinch") == 0 ||
            std::strcmp(mode, "gripSupportLongHandleAxis") == 0 ||
            std::strcmp(mode, "gripSupportPalmWrap") == 0) {
            return GrabPivotAuthoritySource::GripSupportModel;
        }
        if (std::strcmp(mode, "looseWeaponPrimaryAttach") == 0) {
            return GrabPivotAuthoritySource::LooseWeaponPrimaryAttach;
        }
        if (std::strcmp(mode, "meshSurface") == 0) {
            return GrabPivotAuthoritySource::PalmRayMeshPoint;
        }
        if (std::strcmp(mode, "collisionSurface") == 0 || std::strcmp(mode, "selectionHitPointFallback") == 0) {
            return GrabPivotAuthoritySource::CollisionFallback;
        }
        return GrabPivotAuthoritySource::None;
    }
    
    bool pivotAuthoritySourceShouldReacquireAtSeat(const char* source, bool positionOnly)
    {
        if (positionOnly) {
            return true;
        }
        if (!source) {
            return false;
        }
        return std::strcmp(source, grabPivotAuthoritySourceName(GrabPivotAuthoritySource::SelectionHitMeshSnap)) == 0 ||
               std::strcmp(source, grabPivotAuthoritySourceName(GrabPivotAuthoritySource::PalmRayMeshPoint)) == 0 ||
               std::strcmp(source, grabPivotAuthoritySourceName(GrabPivotAuthoritySource::CollisionFallback)) == 0;
    }
    
    bool pivotAuthoritySourceIsFinalPinchOrSupport(const char* source)
    {
        return source &&
               (std::strcmp(source, grabPivotAuthoritySourceName(GrabPivotAuthoritySource::PinchPocketMeshPoint)) == 0 ||
                   std::strcmp(source, grabPivotAuthoritySourceName(GrabPivotAuthoritySource::GripSupportModel)) == 0 ||
                   std::strcmp(source, grabPivotAuthoritySourceName(GrabPivotAuthoritySource::LooseWeaponPrimaryAttach)) == 0);
    }
    
    bool pivotAuthoritySourceCanYieldToPalmPocket(GrabPivotAuthoritySource source)
    {
        switch (source) {
        case GrabPivotAuthoritySource::SelectionHitMeshSnap:
        case GrabPivotAuthoritySource::PalmRayMeshPoint:
        case GrabPivotAuthoritySource::ContactPatchMeshSnap:
        case GrabPivotAuthoritySource::ContactPatchPositionOnly:
        case GrabPivotAuthoritySource::CollisionFallback:
        case GrabPivotAuthoritySource::None:
            return true;
        case GrabPivotAuthoritySource::AuthoredGrabNode:
        case GrabPivotAuthoritySource::PalmPocketMeshPoint:
        case GrabPivotAuthoritySource::PinchPocketMeshPoint:
        case GrabPivotAuthoritySource::GripSupportModel:
        case GrabPivotAuthoritySource::LooseWeaponPrimaryAttach:
            return false;
        }
        return false;
    }
    
    bool pivotAuthorityRequiresSettledVisualRelation(
        const char* source,
        bool positionOnly,
        grab_three_phase::AcquisitionPhase acquisitionPhase)
    {
        return acquisitionPhase != grab_three_phase::AcquisitionPhase::TouchHeld &&
               pivotAuthoritySourceShouldReacquireAtSeat(source, positionOnly);
    }
    
    float finitePositiveOr(float value, float fallback)
    {
        return std::isfinite(value) && value > 0.0f ? value : fallback;
    }
    
    GrabPivotAuthorityCandidate assessGrabPivotAuthorityCandidate(
        const GrabPivotAuthorityCandidateInput& input)
    {
        if (!input.pocket || !input.selection || !input.localMeshTriangles) {
            return {};
        }

        const auto& pocket = *input.pocket;
        const auto& selection = *input.selection;
        const auto& localMeshTriangles = *input.localMeshTriangles;
        GrabPivotAuthorityCandidate candidate{};
        candidate.mode = input.mode ? input.mode : "none";
        candidate.source = inferGrabPivotAuthoritySource(candidate.mode, input.positionOnlyPatch);
        candidate.point = input.point;
        candidate.normal = input.normal;
        candidate.normalTrusted = input.normalTrusted;
        candidate.positionOnlyPatch = input.positionOnlyPatch;
        candidate.positionConfidence = std::clamp(input.positionConfidence, 0.0f, 1.0f);
        candidate.valid = grab_three_phase::isFinite(input.point);
        if (!candidate.valid) {
            return candidate;
        }
    
        const RE::NiPoint3 pocketAnchor = pocket.valid ? pocket.palmCenterWorld : input.point;
        candidate.pivotToPocketGameUnits = pocket.valid ? pointDistanceGameUnits(input.point, pocketAnchor) : 0.0f;
        candidate.selectionDeltaGameUnits =
            selection.hasHitPoint ? pointDistanceGameUnits(selection.hitPointWorld, input.point) : 0.0f;
        candidate.authorityDeltaGameUnits =
            input.hasAuthorityPoint ? pointDistanceGameUnits(input.authorityPoint, input.point) : 0.0f;

        if (!localMeshTriangles.empty() && grab_three_phase::isFinite(input.objectWorldTransform)) {
            const float objectScale = finitePositiveOr(input.objectWorldTransform.scale, 1.0f);
            const RE::NiPoint3 pointLocal = transform_math::worldPointToLocal(input.objectWorldTransform, input.point);
            candidate.longLeverGameUnits = computeLocalMeshMaxDistanceFromPoint(localMeshTriangles, pointLocal) * objectScale;
        }
    
        const float authorityPenalty =
                input.hasAuthorityPoint ? std::clamp(candidate.authorityDeltaGameUnits, 0.0f, 16.0f) * 0.30f : 0.0f;
        const float selectionPenalty =
            selection.hasHitPoint ? std::clamp(candidate.selectionDeltaGameUnits, 0.0f, 24.0f) * 0.20f : 0.0f;
        const float leverPenalty = std::clamp(candidate.longLeverGameUnits, 0.0f, 48.0f) * 0.08f;
        const float confidenceBonus = candidate.positionConfidence * 0.75f;
        candidate.score = candidate.pivotToPocketGameUnits + authorityPenalty + selectionPenalty + leverPenalty - confidenceBonus;
        return candidate;
    }
    
    GrabPivotAuthorityResolution resolveMeshBackedGrabPivotAuthority(
        const GrabPivotAuthorityCandidate& meshCandidate,
        const GrabPivotAuthorityCandidate& patchCandidate,
        const GrabPivotAuthorityCandidate& palmPocketCandidate,
        bool patchComparable,
        bool patchMeshSnapped,
        bool palmPocketEligible,
        float probeSpacingGameUnits,
        float meshSnapMaxDistanceGameUnits,
        float alignmentMaxSelectionDeltaGameUnits)
    {
        /*
         * Position and normal confidence are intentionally separate. The
         * only contact-patch path that may replace pivot B is the guarded
         * mesh-backed, owner-coherent, position-only gate below. Patch/finger
         * normals stay validation and pose evidence; they do not become
         * object orientation authority here.
         */
        GrabPivotAuthorityResolution result{};
        result.selected = meshCandidate;
    
        const float baseDelta = (std::max)(1.0f, finitePositiveOr(probeSpacingGameUnits, 3.0f));
        const float meaningfulImprovement = (std::max)(1.0f, baseDelta * 0.5f);
    
        auto preferCandidate = [&](const GrabPivotAuthorityCandidate& candidate) {
            result.selected = candidate;
        };
    
        if (palmPocketEligible && palmPocketCandidate.valid && meshCandidate.valid &&
            pivotAuthoritySourceCanYieldToPalmPocket(meshCandidate.source)) {
            const float pocketImprovement = meshCandidate.pivotToPocketGameUnits - palmPocketCandidate.pivotToPocketGameUnits;
            const bool selectionStillCoherent =
                std::isfinite(palmPocketCandidate.selectionDeltaGameUnits) &&
                palmPocketCandidate.selectionDeltaGameUnits <= finitePositiveOr(alignmentMaxSelectionDeltaGameUnits, baseDelta) + baseDelta;
            if (selectionStillCoherent && (pocketImprovement >= meaningfulImprovement || palmPocketCandidate.score < meshCandidate.score)) {
                result.reason = "palmPocketMeshPointImprovesSeat";
                result.usePalmPocketPivot = true;
                preferCandidate(palmPocketCandidate);
            }
        }
    
        if (patchCandidate.valid && result.selected.valid) {
            result.pocketImprovementGameUnits =
                result.selected.pivotToPocketGameUnits - patchCandidate.pivotToPocketGameUnits;
            result.scoreImprovement = result.selected.score - patchCandidate.score;
        }
        const float patchDeltaFromSelectedAuthority =
            patchCandidate.valid && result.selected.valid ?
                pointDistanceGameUnits(result.selected.point, patchCandidate.point) :
                patchCandidate.authorityDeltaGameUnits;
        const auto patchAuthorityDecision = grab_pivot_authority_policy::chooseMeshBackedPatchPivotAuthority(
            grab_pivot_authority_policy::MeshBackedPatchPivotAuthorityInput{
                .baselineValid = result.selected.valid,
                .patchComparable = patchComparable,
                .patchValid = patchCandidate.valid,
                .patchMeshSnapped = patchMeshSnapped,
                .selectedPivotToPocketGameUnits = result.selected.pivotToPocketGameUnits,
                .patchPivotToPocketGameUnits = patchCandidate.pivotToPocketGameUnits,
                .selectedScore = result.selected.score,
                .patchScore = patchCandidate.score,
                .patchAuthorityDeltaGameUnits = patchDeltaFromSelectedAuthority,
                .patchSelectionDeltaGameUnits = patchCandidate.selectionDeltaGameUnits,
                .probeSpacingGameUnits = probeSpacingGameUnits,
                .meshSnapMaxDistanceGameUnits = meshSnapMaxDistanceGameUnits,
                .alignmentMaxSelectionDeltaGameUnits = alignmentMaxSelectionDeltaGameUnits });
        result.baseAuthorityDeltaGameUnits = patchAuthorityDecision.baseAuthorityDeltaGameUnits;
        result.extendedAuthorityDeltaGameUnits = patchAuthorityDecision.extendedAuthorityDeltaGameUnits;
        result.pocketImprovementGameUnits = patchAuthorityDecision.pocketImprovementGameUnits;
        result.scoreImprovement = patchAuthorityDecision.scoreImprovement;
        if (std::strcmp(result.reason, "notEvaluated") == 0) {
            result.reason = patchAuthorityDecision.reason;
        }
        return result;
    }
    
    SeatedGrabPivotReacquireCandidate findSeatedGrabPivotNearPalmPocket(
        const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiTransform& currentNodeWorld,
        const RE::NiTransform& currentBodyWorld,
        const RE::NiPoint3& palmPocketWorld,
        const RE::NiPoint3& palmNormalWorld,
        float maxPocketDistanceGameUnits,
        float maxNormalAngleDegrees)
    {
        /*
         * Pull/converge promotion must not reuse a weak startup pivot just
         * because the object has reached the hand. Once the object is inside
         * the palm pocket, reacquire the closest live mesh point to the
         * actual palm/proxy anchor and freeze that same point for visual and
         * constraint authority. This keeps the rendered hand relation and
         * pivot-B solver point on the same corner of the object.
         */
        SeatedGrabPivotReacquireCandidate result{};
        if (localTriangles.empty() ||
            !grab_three_phase::isFinite(currentNodeWorld) ||
            !grab_three_phase::isFinite(currentBodyWorld) ||
            !grab_three_phase::isFinite(palmPocketWorld)) {
            result.reason = "missingMeshOrFrame";
            return result;
        }
    
        const RE::NiPoint3 pocketNodeLocal = transform_math::worldPointToLocal(currentNodeWorld, palmPocketWorld);
        float bestDistanceSquared = std::numeric_limits<float>::max();
        GrabLocalTriangle bestTriangle{};
        RE::NiPoint3 bestPointNodeLocal{};
        bool found = false;
        for (const auto& localTriangle : localTriangles) {
            TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
            float distanceSquared = 0.0f;
            const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(pocketNodeLocal, triangle, distanceSquared);
            if (!std::isfinite(distanceSquared) || distanceSquared >= bestDistanceSquared) {
                continue;
            }
    
            bestDistanceSquared = distanceSquared;
            bestTriangle = localTriangle;
            bestPointNodeLocal = candidate;
            found = true;
        }
    
        if (!found) {
            result.reason = "noLocalTriangleCandidate";
            return result;
        }
    
        const RE::NiPoint3 bestPointWorld = transform_math::localPointToWorld(currentNodeWorld, bestPointNodeLocal);
        const float pocketDistance = pointDistanceGameUnits(bestPointWorld, palmPocketWorld);
        const float maxPocketDistance =
            std::isfinite(maxPocketDistanceGameUnits) && maxPocketDistanceGameUnits > 0.0f ? maxPocketDistanceGameUnits : 0.0f;
        if (maxPocketDistance > 0.0f && pocketDistance > maxPocketDistance) {
            result.reason = "meshPointOutsidePocketEnvelope";
            result.pocketDistanceGameUnits = pocketDistance;
            result.meshDistanceGameUnits = std::sqrt((std::max)(0.0f, bestDistanceSquared));
            return result;
        }
    
        const RE::NiPoint3 localEdge0 = bestTriangle.v1 - bestTriangle.v0;
        const RE::NiPoint3 localEdge1 = bestTriangle.v2 - bestTriangle.v0;
        const RE::NiPoint3 normalNodeLocal = normalizeOrZero(crossProduct(localEdge0, localEdge1));
        RE::NiPoint3 normalWorld = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, normalNodeLocal));
        const RE::NiPoint3 palmNormal = normalizeOrZero(palmNormalWorld);
        if (dotProduct(normalWorld, palmNormal) > 0.0f) {
            normalWorld = RE::NiPoint3{ -normalWorld.x, -normalWorld.y, -normalWorld.z };
        }
    
        const float minNormalDot =
            std::cos(std::clamp(maxNormalAngleDegrees, 0.0f, 179.0f) * 3.14159265358979323846f / 180.0f);
        const float normalFacing = std::fabs(dotProduct(normalizeOrZero(normalWorld), palmNormal));
    
        result.pointWorld = bestPointWorld;
        result.pointNodeLocal = bestPointNodeLocal;
        result.pointBodyLocalGame = transform_math::worldPointToLocal(currentBodyWorld, bestPointWorld);
        result.normalWorld = normalWorld;
        result.normalNodeLocal = normalNodeLocal;
        result.pocketDistanceGameUnits = pocketDistance;
        result.meshDistanceGameUnits = std::sqrt((std::max)(0.0f, bestDistanceSquared));
        result.longLeverGameUnits = computeLocalMeshMaxDistanceFromPoint(localTriangles, bestPointNodeLocal) *
                                    finitePositiveOr(currentNodeWorld.scale, 1.0f);
        result.normalTrusted = normalFacing >= minNormalDot;
        result.reason = result.normalTrusted ? "seatedMeshPivotReacquired" : "seatedMeshPivotPositionOnly";
        result.valid = true;
        return result;
    }
    
    GrabSeatDepthStopResult computeGrabSeatDepthStop(
        const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiTransform& objectNodeWorld,
        const RE::NiPoint3& gripPointWorld,
        const RE::NiPoint3& palmNormalWorld,
        float footprintRadiusGameUnits,
        float maxDepthGameUnits)
    {
        GrabSeatDepthStopResult result{};
        if (!std::isfinite(maxDepthGameUnits) || maxDepthGameUnits <= 0.0f) {
            result.reason = "seatDepthDisabled";
            return result;
        }
        if (localTriangles.empty()) {
            result.reason = "noLocalTriangles";
            return result;
        }
        if (!grab_three_phase::isFinite(objectNodeWorld) ||
            !grab_three_phase::isFinite(gripPointWorld) ||
            !grab_three_phase::isFinite(palmNormalWorld)) {
            result.reason = "nonFiniteSeatFrame";
            return result;
        }
        const RE::NiPoint3 inwardLocal = normalizeOrZero(transform_math::worldVectorToLocal(
            objectNodeWorld,
            RE::NiPoint3{ -palmNormalWorld.x, -palmNormalWorld.y, -palmNormalWorld.z }));
        if (lengthSquared(inwardLocal) <= 0.000001f) {
            result.reason = "degeneratePalmNormal";
            return result;
        }
    
        const float objectScale = finitePositiveOr(objectNodeWorld.scale, 1.0f);
        const float footprintRadiusLocal = (std::max)(0.1f, finitePositiveOr(footprintRadiusGameUnits, 10.0f)) / objectScale;
        const float footprintRadiusLocalSquared = footprintRadiusLocal * footprintRadiusLocal;
        const float maxDepthLocal = maxDepthGameUnits / objectScale;
        const RE::NiPoint3 gripLocal = transform_math::worldPointToLocal(objectNodeWorld, gripPointWorld);
        if (!grab_three_phase::isFinite(gripLocal)) {
            result.reason = "nonFiniteGripLocal";
            return result;
        }
    
        float bestDepthLocal = 0.0f;
        std::uint32_t footprintSampleCount = 0;
        auto considerLocalPoint = [&](const RE::NiPoint3& pointLocal) {
            const RE::NiPoint3 delta = pointLocal - gripLocal;
            const float depthLocal = dotProduct(delta, inwardLocal);
            if (!std::isfinite(depthLocal) || depthLocal <= 0.0f) {
                return;
            }
            const RE::NiPoint3 lateral = delta - inwardLocal * depthLocal;
            const float lateralSquared = lengthSquared(lateral);
            if (!std::isfinite(lateralSquared) || lateralSquared > footprintRadiusLocalSquared) {
                return;
            }
            ++footprintSampleCount;
            bestDepthLocal = (std::max)(bestDepthLocal, (std::min)(depthLocal, maxDepthLocal));
        };
        const std::array<float, 3> axisProbeDepthsLocal{ 0.0f, maxDepthLocal * 0.5f, maxDepthLocal };
        for (const auto& localTriangle : localTriangles) {
            considerLocalPoint(localTriangle.v0);
            considerLocalPoint(localTriangle.v1);
            considerLocalPoint(localTriangle.v2);
            const TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
            for (const float axisDepthLocal : axisProbeDepthsLocal) {
                const RE::NiPoint3 axisPointLocal = gripLocal + inwardLocal * axisDepthLocal;
                float distanceSquared = 0.0f;
                considerLocalPoint(closestPointOnTriangleToPoint(axisPointLocal, triangle, distanceSquared));
            }
        }
    
        result.depthGameUnits = bestDepthLocal * objectScale;
        result.footprintSampleCount = footprintSampleCount;
        result.reason = footprintSampleCount > 0 ? "meshSupportDepth" : "noMeshInsideFootprint";
        result.valid = true;
        return result;
    }
    
    [[nodiscard]] float gripAxisTiltRadiansForHand(bool isLeft)
    {
        const float degrees = isLeft ? -g_rockConfig.rockPullPresentationGripAxisTiltDegrees
                                     : g_rockConfig.rockPullPresentationGripAxisTiltDegrees;
        return degrees * 0.01745329252f;
    }
    
    RE::NiTransform rotateTransformWorldAboutPoint(
        const RE::NiTransform& transform,
        const RE::NiPoint3& unitAxisWorld,
        float angleRadians,
        const RE::NiPoint3& pivotWorld)
    {
        RE::NiTransform rotated = transform;
        for (int row = 0; row < 3; ++row) {
            const RE::NiPoint3 rowWorld{
                transform.rotate.entry[row][0],
                transform.rotate.entry[row][1],
                transform.rotate.entry[row][2],
            };
            const RE::NiPoint3 rotatedRow = grab_finger_pose_math::rotateAroundUnitAxis(rowWorld, unitAxisWorld, angleRadians);
            rotated.rotate.entry[row][0] = rotatedRow.x;
            rotated.rotate.entry[row][1] = rotatedRow.y;
            rotated.rotate.entry[row][2] = rotatedRow.z;
        }
        const RE::NiPoint3 pivotOffset = transform.translate - pivotWorld;
        const RE::NiPoint3 rotatedOffset = grab_finger_pose_math::rotateAroundUnitAxis(pivotOffset, unitAxisWorld, angleRadians);
        rotated.translate = pivotWorld + rotatedOffset;
        return rotated;
    }
    
    SeatedPalmPocketSupportPatch buildSeatedPalmPocketSupportPatch(
        const std::vector<GrabLocalTriangle>& localTriangles,
        std::uint32_t bodyId,
        const RE::NiTransform& currentNodeWorld,
        const RE::NiPoint3& palmPocketWorld,
        const RE::NiPoint3& anchorWorld,
        const RE::NiPoint3& palmNormalWorld,
        const RE::NiPoint3& palmTangentWorld,
        const RE::NiPoint3& palmBitangentWorld,
        float objectLeverEstimateGameUnits)
    {
        /*
         * This is the held-time equivalent of the palm-pocket capture patch,
         * but it samples the cached visual mesh in object-local space instead
         * of running hknp casts after the grab is already active. The selected
         * BODY-local pivot remains the authority; these samples only upgrade
         * the support model once the object is seated at the palm.
         */
        SeatedPalmPocketSupportPatch result{};
        if (localTriangles.empty() ||
            bodyId == INVALID_BODY_ID ||
            !grab_three_phase::isFinite(currentNodeWorld) ||
            !grab_three_phase::isFinite(palmPocketWorld) ||
            !grab_three_phase::isFinite(anchorWorld)) {
            result.reason = "missingMeshOrFrame";
            result.patch.fallbackReason = result.reason;
            return result;
        }
    
        const RE::NiPoint3 palmNormal = normalizeOrZero(palmNormalWorld);
        RE::NiPoint3 palmTangent = normalizeOrZero(palmTangentWorld);
        RE::NiPoint3 palmBitangent = normalizeOrZero(palmBitangentWorld);
        if (lengthSquared(palmBitangent) <= 0.0f) {
            palmBitangent = normalizeOrZero(crossProduct(palmNormal, palmTangent));
        }
        if (lengthSquared(palmNormal) <= 0.0f || lengthSquared(palmTangent) <= 0.0f) {
            result.reason = "invalidPalmFrame";
            result.patch.fallbackReason = result.reason;
            return result;
        }
    
        const auto probeGeometry = grab_contact_patch_math::computeContactPatchProbeGeometry(
            g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits,
            g_rockConfig.rockGrabContactPatchProbeRadiusGameUnits,
            objectLeverEstimateGameUnits,
            g_rockConfig.rockGrabSmallObjectReferenceLeverGameUnits,
            g_rockConfig.rockGrabLongObjectReferenceLeverGameUnits);
        const float spacing = probeGeometry.spacingGameUnits;
        const float radius = probeGeometry.radiusGameUnits;
        result.probeSpacingGameUnits = spacing;
        result.probeRadiusGameUnits = radius;
    
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> offsets{};
        const auto probePatternCount =
            grab_contact_patch_math::buildContactPatchProbeOffsets(offsets, palmTangent, palmBitangent, spacing);
        const int probeCount = (std::min)(
            std::clamp(g_rockConfig.rockGrabContactPatchProbeCount, 1, static_cast<int>(kMaxGrabContactPatchSamples)),
            static_cast<int>(probePatternCount));
    
        std::vector<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>> fitSamples;
        fitSamples.reserve(static_cast<std::size_t>(probeCount));
        const float nodeScale = finitePositiveOr(currentNodeWorld.scale, 1.0f);
        const float duplicateDistance = (std::max)(0.20f, (std::min)(0.75f, radius * 0.25f));
        const float duplicateDistanceSquared = duplicateDistance * duplicateDistance;
    
        for (int probe = 0; probe < probeCount; ++probe) {
            const RE::NiPoint3 probeWorld = palmPocketWorld + offsets[probe];
            const RE::NiPoint3 probeLocal = transform_math::worldPointToLocal(currentNodeWorld, probeWorld);
    
            float bestDistanceSquared = std::numeric_limits<float>::max();
            GrabLocalTriangle bestTriangle{};
            RE::NiPoint3 bestPointLocal{};
            bool found = false;
            for (const auto& localTriangle : localTriangles) {
                TriangleData triangle{ localTriangle.v0, localTriangle.v1, localTriangle.v2 };
                float distanceSquared = 0.0f;
                const RE::NiPoint3 candidate = closestPointOnTriangleToPoint(probeLocal, triangle, distanceSquared);
                if (!std::isfinite(distanceSquared) || distanceSquared >= bestDistanceSquared) {
                    continue;
                }
    
                const RE::NiPoint3 localEdge0 = localTriangle.v1 - localTriangle.v0;
                const RE::NiPoint3 localEdge1 = localTriangle.v2 - localTriangle.v0;
                if (lengthSquared(crossProduct(localEdge0, localEdge1)) <= 1.0e-8f) {
                    continue;
                }
    
                bestDistanceSquared = distanceSquared;
                bestTriangle = localTriangle;
                bestPointLocal = candidate;
                found = true;
            }
    
            if (!found) {
                continue;
            }
    
            const RE::NiPoint3 pointWorld = transform_math::localPointToWorld(currentNodeWorld, bestPointLocal);
            bool duplicate = false;
            for (const auto& sample : fitSamples) {
                if (lengthSquared(sample.point - pointWorld) <= duplicateDistanceSquared) {
                    duplicate = true;
                    break;
                }
            }
            if (duplicate) {
                continue;
            }
    
            const RE::NiPoint3 localEdge0 = bestTriangle.v1 - bestTriangle.v0;
            const RE::NiPoint3 localEdge1 = bestTriangle.v2 - bestTriangle.v0;
            const RE::NiPoint3 normalNodeLocal = normalizeOrZero(crossProduct(localEdge0, localEdge1));
            RE::NiPoint3 normalWorld = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, normalNodeLocal));
            normalWorld = grab_contact_patch_math::orientNormalTowardPalm(normalWorld, palmNormal);
            if (lengthSquared(normalWorld) <= 0.0f) {
                continue;
            }
    
            grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3> sample{};
            sample.bodyId = bodyId;
            sample.point = pointWorld;
            sample.normal = normalWorld;
            sample.fraction = std::sqrt((std::max)(0.0f, bestDistanceSquared)) * nodeScale;
            sample.accepted = true;
            sample.rejectionReason = "seatedPalmPocketMeshSample";
            fitSamples.push_back(sample);
        }
    
        result.rawSampleCount = static_cast<std::uint32_t>((std::min)(
            fitSamples.size(),
            static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
        if (fitSamples.empty()) {
            result.reason = "noSeatedPalmPocketSamples";
            result.patch.fallbackReason = result.reason;
            return result;
        }
    
        const float anchorDepthLimit = (std::max)(1.0f, radius + spacing * 0.50f);
        const float clusterDepthLimit = (std::max)(0.75f, radius + spacing * 0.35f);
        const float anchorLateralLimit = (std::max)(
            radius * 2.0f,
            (std::max)(
                spacing * 2.0f + radius,
                finitePositiveOr(g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits, 4.0f) + spacing));
        const auto surfaceCluster = grab_contact_patch_math::filterContactPatchSameSurfaceCluster(fitSamples,
            anchorWorld,
            palmNormal,
            anchorDepthLimit,
            clusterDepthLimit,
            anchorLateralLimit,
            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
        result.clusterRejectedSampleCount = static_cast<std::uint32_t>((std::min)(
            surfaceCluster.clusterRejectedCount,
            static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
        result.clusterDepthSpreadGameUnits = surfaceCluster.maxDepthSpreadGameUnits;
        result.clusterMaxLateralGameUnits = surfaceCluster.maxLateralDistanceGameUnits;
        result.clusterReason = surfaceCluster.reason;
        if (!surfaceCluster.valid) {
            result.reason = surfaceCluster.reason ? surfaceCluster.reason : "seatedPalmPocketClusterFailed";
            result.patch.fallbackReason = result.reason;
            return result;
        }
    
        fitSamples = surfaceCluster.samples;
        for (const auto& sample : fitSamples) {
            if (result.sampleCount < result.samples.size()) {
                result.samples[result.sampleCount++] = sample;
            }
        }
    
        result.patch = grab_contact_patch_math::fitContactPatch(fitSamples,
            anchorWorld,
            palmNormal,
            palmTangent,
            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
        if (!result.patch.valid) {
            result.reason = result.patch.fallbackReason ? result.patch.fallbackReason : "seatedPalmPocketPatchFailed";
            return result;
        }
    
        const float minNormalDot =
            std::cos(std::clamp(g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees, 0.0f, 179.0f) *
                     3.14159265358979323846f / 180.0f);
        const float normalFacing = std::fabs(dotProduct(normalizeOrZero(result.patch.normal), palmNormal));
        result.normalTrusted = result.patch.orientationReliable && normalFacing >= minNormalDot;
        result.valid = true;
        result.reason = result.normalTrusted ? "seatedPalmPocketSupportPatch" : "seatedPalmPocketSupportPositionOnly";
        return result;
    }
}

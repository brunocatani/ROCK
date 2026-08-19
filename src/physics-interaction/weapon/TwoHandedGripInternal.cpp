#include "physics-interaction/weapon/TwoHandedGripInternal.h"

#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rock::two_handed_grip_detail
{
    grab_pinch_pocket_policy::Config currentWeaponOppositionPocketConfig()
    {
        return grab_pinch_pocket_policy::sanitizeConfig(
            grab_pinch_pocket_policy::Config{
                .enabled = g_rockConfig.rockGrabPinchPocketEnabled,
                .compactMaxExtentGameUnits =
                    g_rockConfig.rockGrabPinchCompactMaxExtentGameUnits,
                .thinRodMaxLengthGameUnits =
                    g_rockConfig.rockGrabPinchThinRodMaxLengthGameUnits,
                .thinRodMaxCrossSectionGameUnits =
                    g_rockConfig.rockGrabPinchThinRodMaxCrossSectionGameUnits,
                .maxPocketDistanceGameUnits =
                    g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits,
                .minFingerGapGameUnits =
                    g_rockConfig.rockGrabPinchMinFingerGapGameUnits,
                .maxFingerGapGameUnits =
                    g_rockConfig.rockGrabPinchMaxFingerGapGameUnits,
                .thumbIndexMaxOpenValue =
                    g_rockConfig.rockGrabPinchThumbIndexMaxOpenValue,
                .otherFingerCurlValue =
                    g_rockConfig.rockGrabPinchOtherFingerCurlValue,
                .surfaceInsetGameUnits =
                    g_rockConfig.rockGrabPinchSurfaceInsetGameUnits,
                .detectionDirectionHandspace =
                    g_rockConfig.rockGrabPinchDetectionDirectionHandspace,
                .detectionAxisBlend =
                    g_rockConfig.rockGrabPinchDetectionAxisBlend,
            });
    }

    gunstock_alignment_policy::FineTuneDegrees configuredGunstockFineTune()
    {
        return {
            .pitchDegrees =
                g_rockConfig.rockGunstockAlignmentPitchDegrees,
            .yawDegrees =
                g_rockConfig.rockGunstockAlignmentYawDegrees,
            .rollDegrees =
                g_rockConfig.rockGunstockAlignmentRollDegrees,
        };
    }

    RE::NiMatrix3 orthonormalizeStoredRotation(
        const RE::NiMatrix3& rotation)
    {
        // The two-anchor solve feeds this rotation into the next frame.
        // Restore a rigid basis before float error can become visible shear.
        const RE::NiPoint3 row0{
            rotation.entry[0][0],
            rotation.entry[0][1],
            rotation.entry[0][2],
        };
        const RE::NiPoint3 row1{
            rotation.entry[1][0],
            rotation.entry[1][1],
            rotation.entry[1][2],
        };
        const RE::NiPoint3 axis0 = weaponSolverNormalize(row0);
        RE::NiPoint3 axis2 = weaponSolverCross(axis0, row1);
        axis2 = weaponSolverNormalize(axis2);
        const RE::NiPoint3 axis1 = weaponSolverCross(axis2, axis0);

        RE::NiMatrix3 result = rotation;
        result.entry[0][0] = axis0.x;
        result.entry[0][1] = axis0.y;
        result.entry[0][2] = axis0.z;
        result.entry[1][0] = axis1.x;
        result.entry[1][1] = axis1.y;
        result.entry[1][2] = axis1.z;
        result.entry[2][0] = axis2.x;
        result.entry[2][1] = axis2.y;
        result.entry[2][2] = axis2.z;
        return result;
    }

    bool leftFiringInfrastructureAvailable()
    {
        // Both blockers are one contract. A partial block lets FRIK fight ROCK.
        return frik_visual_authority::canBlockPrimaryHandWeaponPose() &&
               frik_visual_authority::canBlockPrimaryWeaponNodeOwnership();
    }

    RE::NiPoint3 lerpPoint(
        const RE::NiPoint3& from,
        const RE::NiPoint3& to,
        const float alpha)
    {
        const float t = std::clamp(alpha, 0.0f, 1.0f);
        return RE::NiPoint3{
            from.x + (to.x - from.x) * t,
            from.y + (to.y - from.y) * t,
            from.z + (to.z - from.z) * t,
        };
    }

    bool isFiniteRotation(const RE::NiMatrix3& rotation)
    {
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                if (!std::isfinite(rotation.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    bool isFiniteTransform(const RE::NiTransform& transform)
    {
        return isFiniteRotation(transform.rotate) &&
               std::isfinite(transform.translate.x) &&
               std::isfinite(transform.translate.y) &&
               std::isfinite(transform.translate.z) &&
               std::isfinite(transform.scale);
    }

    bool isInvertibleTransform(const RE::NiTransform& transform)
    {
        return isFiniteTransform(transform) &&
               std::abs(transform.scale) > 0.0001f;
    }

    bool areTransformsNearlyEqual(
        const RE::NiTransform& lhs,
        const RE::NiTransform& rhs,
        const float epsilon)
    {
        for (std::size_t row = 0; row < 3; ++row) {
            for (std::size_t column = 0; column < 3; ++column) {
                if (std::abs(
                        lhs.rotate.entry[row][column] -
                        rhs.rotate.entry[row][column]) > epsilon) {
                    return false;
                }
            }
        }
        return std::abs(lhs.translate.x - rhs.translate.x) <= epsilon &&
               std::abs(lhs.translate.y - rhs.translate.y) <= epsilon &&
               std::abs(lhs.translate.z - rhs.translate.z) <= epsilon &&
               std::abs(lhs.scale - rhs.scale) <= epsilon;
    }

    bool arePointsNearlyEqual(
        const RE::NiPoint3& lhs,
        const RE::NiPoint3& rhs,
        const float epsilon)
    {
        return std::abs(lhs.x - rhs.x) <= epsilon &&
               std::abs(lhs.y - rhs.y) <= epsilon &&
               std::abs(lhs.z - rhs.z) <= epsilon;
    }

    bool isUsableHandAuthorityTransform(const RE::NiTransform& transform)
    {
        // hFRIK uses 0.00001 as its intentional ScopeMenu hide scale.
        return isFiniteTransform(transform) &&
               std::abs(transform.scale) > 0.0001f;
    }

    bool validateWeaponPresentationSubtree(
        RE::NiAVObject* object,
        const RE::NiTransform& presentationWorldDelta,
        const std::size_t depth,
        std::size_t& objectCount)
    {
        if (!object ||
            depth > WEAPON_PRESENTATION_MAX_DEPTH ||
            objectCount >= WEAPON_PRESENTATION_MAX_OBJECTS ||
            !isFiniteTransform(object->world)) {
            return false;
        }
        ++objectCount;

        const RE::NiTransform reframedWorld =
            weapon_visual_authority_math::applyPresentationWorldDelta(
                presentationWorldDelta,
                object->world);
        if (!isFiniteTransform(reframedWorld)) {
            return false;
        }

        auto* node = object->IsNode();
        if (!node) {
            return true;
        }
        for (const auto& child : node->children) {
            if (child &&
                !validateWeaponPresentationSubtree(
                    child.get(),
                    presentationWorldDelta,
                    depth + 1,
                    objectCount)) {
                return false;
            }
        }
        return true;
    }

    void applyWeaponPresentationDeltaToDescendants(
        RE::NiNode* parent,
        const RE::NiTransform& presentationWorldDelta)
    {
        for (const auto& child : parent->children) {
            if (!child) {
                continue;
            }
            child->world =
                weapon_visual_authority_math::applyPresentationWorldDelta(
                    presentationWorldDelta,
                    child->world);
            if (auto* childNode = child->IsNode()) {
                    applyWeaponPresentationDeltaToDescendants(
                    childNode,
                    presentationWorldDelta);
            }
        }
    }

    bool tryResolveWeaponRootLocal(
        const RE::NiNode* parent,
        const RE::NiTransform& weaponWorld,
        RE::NiTransform& outWeaponLocal)
    {
        if (!isFiniteTransform(weaponWorld)) {
            return false;
        }

        outWeaponLocal = weaponWorld;
        if (parent) {
            if (!isInvertibleTransform(parent->world)) {
                return false;
            }
            outWeaponLocal =
                weapon_visual_authority_math::worldTargetToParentLocal(
                    parent->world,
                    weaponWorld);
        } else {
            outWeaponLocal = weaponWorld;
        }
        return isFiniteTransform(outWeaponLocal);
    }

    bool restoreWeaponRootPreservingPresentedDescendants(
        RE::NiNode* weaponNode,
        const RE::NiTransform& weaponWorld)
    {
        if (!weaponNode) {
            return false;
        }
        RE::NiTransform weaponLocal{};
        if (!tryResolveWeaponRootLocal(
                weaponNode->parent,
                weaponWorld,
                weaponLocal)) {
            return false;
        }
        weaponNode->local = weaponLocal;
        weaponNode->world = weaponWorld;
        return true;
    }

    bool moveWeaponPresentationRigidly(
        RE::NiNode* weaponNode,
        const RE::NiTransform& solvedWeaponWorld)
    {
        if (!weaponNode ||
            !isInvertibleTransform(weaponNode->world) ||
            !isFiniteTransform(solvedWeaponWorld)) {
            return false;
        }

        RE::NiTransform solvedWeaponLocal{};
        if (!tryResolveWeaponRootLocal(
                weaponNode->parent,
                solvedWeaponWorld,
                solvedWeaponLocal)) {
            return false;
        }

        const RE::NiTransform oldWeaponWorld = weaponNode->world;
        const RE::NiTransform presentationWorldDelta =
            weapon_visual_authority_math::makePresentationWorldDelta(
                oldWeaponWorld,
                solvedWeaponWorld);
        if (!isFiniteTransform(presentationWorldDelta)) {
            return false;
        }

        // Validate before the first write. One invalid mod node cancels all.
        std::size_t objectCount = 0;
        if (!validateWeaponPresentationSubtree(
                weaponNode,
                presentationWorldDelta,
                0,
                objectCount)) {
            return false;
        }

        weaponNode->local = solvedWeaponLocal;
        weaponNode->world = solvedWeaponWorld;
        applyWeaponPresentationDeltaToDescendants(
            weaponNode,
            presentationWorldDelta);
        return true;
    }

    bool resolveAuthoredSupportPalmSeatProximityFromPoints(
        const RE::NiTransform& weaponWorld,
        const RE::NiPoint3& liveTouchProbeWorld,
        const RE::NiPoint3& authoredPalmSeatWeaponLocal,
        AuthoredSupportPalmSeatProximity& out)
    {
        out = {};
        if (!isFiniteTransform(weaponWorld) ||
            std::abs(weaponWorld.scale) <= 0.0001f ||
            !std::isfinite(liveTouchProbeWorld.x) ||
            !std::isfinite(liveTouchProbeWorld.y) ||
            !std::isfinite(liveTouchProbeWorld.z) ||
            !std::isfinite(authoredPalmSeatWeaponLocal.x) ||
            !std::isfinite(authoredPalmSeatWeaponLocal.y) ||
            !std::isfinite(authoredPalmSeatWeaponLocal.z)) {
            return false;
        }

        out.authoredPalmSeatWeaponLocal = authoredPalmSeatWeaponLocal;
        out.authoredPalmSeatWorld = transform_math::localPointToWorld(
            weaponWorld,
            out.authoredPalmSeatWeaponLocal);
        out.liveTouchProbeWorld = liveTouchProbeWorld;
        out.liveTouchProbeWeaponLocal = transform_math::worldPointToLocal(
            weaponWorld,
            out.liveTouchProbeWorld);
        if (!std::isfinite(out.authoredPalmSeatWorld.x) ||
            !std::isfinite(out.authoredPalmSeatWorld.y) ||
            !std::isfinite(out.authoredPalmSeatWorld.z) ||
            !std::isfinite(out.liveTouchProbeWeaponLocal.x) ||
            !std::isfinite(out.liveTouchProbeWeaponLocal.y) ||
            !std::isfinite(out.liveTouchProbeWeaponLocal.z)) {
            return false;
        }

        // The activation gate uses one current weapon-relative distance.
        const RE::NiPoint3 localDelta{
            out.liveTouchProbeWeaponLocal.x -
                out.authoredPalmSeatWeaponLocal.x,
            out.liveTouchProbeWeaponLocal.y -
                out.authoredPalmSeatWeaponLocal.y,
            out.liveTouchProbeWeaponLocal.z -
                out.authoredPalmSeatWeaponLocal.z,
        };
        out.weaponRelativeDistanceGameUnits =
            std::sqrt(
                localDelta.x * localDelta.x +
                localDelta.y * localDelta.y +
                localDelta.z * localDelta.z) *
            std::abs(weaponWorld.scale);

        // World readback is diagnostic. It does not select the grip.
        const RE::NiPoint3 worldDelta{
            out.liveTouchProbeWorld.x - out.authoredPalmSeatWorld.x,
            out.liveTouchProbeWorld.y - out.authoredPalmSeatWorld.y,
            out.liveTouchProbeWorld.z - out.authoredPalmSeatWorld.z,
        };
        out.worldReadbackDistanceGameUnits = std::sqrt(
            worldDelta.x * worldDelta.x +
            worldDelta.y * worldDelta.y +
            worldDelta.z * worldDelta.z);
        out.frameAgreementErrorGameUnits = std::abs(
            out.weaponRelativeDistanceGameUnits -
            out.worldReadbackDistanceGameUnits);
        return std::isfinite(out.weaponRelativeDistanceGameUnits) &&
               std::isfinite(out.worldReadbackDistanceGameUnits) &&
               std::isfinite(out.frameAgreementErrorGameUnits);
    }

    bool resolveAuthoredSupportPalmSeatProximity(
        const RE::NiTransform& weaponWorld,
        const RE::NiTransform& liveHandWorld,
        const RE::NiTransform& authoredHandWeaponLocal,
        const bool isLeft,
        AuthoredSupportPalmSeatProximity& out)
    {
        out = {};
        if (!isUsableHandAuthorityTransform(liveHandWorld) ||
            !isFiniteTransform(authoredHandWeaponLocal) ||
            std::abs(authoredHandWeaponLocal.scale) <= 0.0001f) {
            return false;
        }

        const RE::NiTransform authoredHandWorld =
            transform_math::composeTransforms(
                weaponWorld,
                authoredHandWeaponLocal);
        const RE::NiPoint3 authoredPalmSeatWeaponLocal =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                authoredHandWeaponLocal,
                isLeft);
        const RE::NiPoint3 liveTouchProbeWorld =
            computeGrabLegacyPalmPivotAWorldFromHandBasis(
                liveHandWorld,
                isLeft);
        if (!isFiniteTransform(authoredHandWorld) ||
            !resolveAuthoredSupportPalmSeatProximityFromPoints(
                weaponWorld,
                liveTouchProbeWorld,
                authoredPalmSeatWeaponLocal,
                out)) {
            return false;
        }

        out.authoredHandWorld = authoredHandWorld;
        return true;
    }

    authored_weapon_grip_activation_policy::DirectionGateResult
        evaluateAuthoredSupportGripDirectionGate(
            AuthoredSupportGripDebugSnapshot& snapshot,
            const RE::NiPoint3& lastStableDirectionWorld,
            const bool lastStableDirectionValid,
            const bool rightFiringLeftSupportScope)
    {
        using ActivationVec3 =
            authored_weapon_grip_activation_policy::Vec3;
        const auto toActivationVector = [](const RE::NiPoint3& value) {
            return ActivationVec3{ value.x, value.y, value.z };
        };
        const auto gate =
            authored_weapon_grip_activation_policy::evaluateDirectionGate(
                authored_weapon_grip_activation_policy::DirectionGateInput{
                    .weaponFamily = snapshot.weaponFamily,
                    .authoredSeatWorld = toActivationVector(
                        snapshot.authoredPalmSeatWorld),
                    .liveProbeWorld = toActivationVector(
                        snapshot.liveTouchProbeWorld),
                    .leftAxisWorld = toActivationVector(
                        snapshot.leftAxisWorld),
                    .downAxisWorld = toActivationVector(
                        snapshot.downAxisWorld),
                    .lastStableDirectionWorld = toActivationVector(
                        lastStableDirectionWorld),
                    .radialCapGameUnits = snapshot.radialCapGameUnits,
                    .lastStableDirectionValid =
                        lastStableDirectionValid,
                    .rightFiringLeftSupportScope =
                        rightFiringLeftSupportScope,
                });

        snapshot.approachDirectionWorld = RE::NiPoint3{
            gate.approachDirectionWorld.x,
            gate.approachDirectionWorld.y,
            gate.approachDirectionWorld.z,
        };
        snapshot.leftDot = gate.leftDot;
        snapshot.downDot = gate.downDot;
        snapshot.selectedCone = gate.selectedCone;
        snapshot.classifierSupported = gate.familySupported;
        snapshot.directionUsedLastStableSample =
            gate.usedLastStableDirection;
        snapshot.radialPass = gate.radialPass;
        snapshot.directionPass = gate.directionPass;
        snapshot.scopePass = gate.scopePass;
        snapshot.activationSpatialPass = gate.spatialPass;
        return gate;
    }

    namespace
    {
        bool rankedSupportGripTriangleLess(const RankedSupportGripTriangle& lhs, const RankedSupportGripTriangle& rhs)
        {
            if (lhs.distanceSquared == rhs.distanceSquared) {
                return lhs.deterministicOrdinal <
                       rhs.deterministicOrdinal;
            }
            return lhs.distanceSquared < rhs.distanceSquared;
        }

        struct TransformedSupportGripTriangleView
        {
            std::span<const TriangleData> localTriangles{};
            RE::NiTransform localToWorld{};

            [[nodiscard]] std::size_t size() const noexcept { return localTriangles.size(); }

            [[nodiscard]] TriangleData operator[](std::size_t index) const
            {
                const auto& triangle = localTriangles[index];
                return TriangleData{
                    transform_math::localPointToWorld(localToWorld, triangle.v0),
                    transform_math::localPointToWorld(localToWorld, triangle.v1),
                    transform_math::localPointToWorld(localToWorld, triangle.v2),
                };
            }
        };
    }

    void selectNearestSupportGripFingerTriangles(
        std::span<const WeaponCollision::SupportGripEvidenceView> evidenceViews,
        const RE::NiTransform& weaponWorld,
        const SupportGripFingerReferenceSet& referenceSet,
        std::size_t maxTriangles,
        std::array<std::vector<RankedSupportGripTriangle>,
            kSupportGripFingerLaneCount + 1>& rankingScratch,
        std::vector<TriangleData>& outTriangles)
    {
        for (auto& ranking : rankingScratch) {
            ranking.clear();
        }
        outTriangles.clear();
        const std::size_t boundedLimit = (std::min)(maxTriangles, grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles);
        if (boundedLimit == 0 ||
            evidenceViews.empty() ||
            !referenceSet.seatPointValid ||
            !weapon_support_acquisition_math::isUsableTransform(
                weaponWorld)) {
            return;
        }

        SupportGripFingerReferenceSet localReferences{};
        localReferences.seatPointWorld =
            transform_math::worldPointToLocal(
                weaponWorld,
                referenceSet.seatPointWorld);
        localReferences.seatPointValid =
            grab_finger_pose_runtime::isFinitePoint(
                localReferences.seatPointWorld);
        if (!localReferences.seatPointValid) {
            return;
        }
        for (std::size_t lane = 0;
             lane < kSupportGripFingerLaneCount;
             ++lane) {
            const std::size_t count = (std::min)(
                referenceSet.lanePointCounts[lane],
                kSupportGripFingerLaneReferenceCapacity);
            for (std::size_t point = 0; point < count; ++point) {
                const RE::NiPoint3 localPoint =
                    transform_math::worldPointToLocal(
                        weaponWorld,
                        referenceSet.lanePointsWorld[lane][point]);
                if (!grab_finger_pose_runtime::isFinitePoint(
                        localPoint)) {
                    continue;
                }
                localReferences.lanePointsWorld[lane]
                    [localReferences.lanePointCounts[lane]++] =
                    localPoint;
            }
            if (localReferences.lanePointCounts[lane] >
                kSupportGripFingerLaneReferenceCapacity) {
                return;
            }
        }

        const std::size_t laneLimit = (std::max)(
            static_cast<std::size_t>(1),
            (boundedLimit + kSupportGripFingerLaneCount - 1) /
                kSupportGripFingerLaneCount);
        for (std::size_t lane = 0;
             lane < kSupportGripFingerLaneCount;
             ++lane) {
            rankingScratch[lane].reserve(laneLimit);
        }
        rankingScratch[kSupportGripGlobalRankingIndex].reserve(
            boundedLimit);

        const auto retainNearest = [](
                                       std::vector<RankedSupportGripTriangle>& ranking,
                                       const std::size_t limit,
                                       const RankedSupportGripTriangle& candidate) {
            if (ranking.size() < limit) {
                ranking.push_back(candidate);
                std::push_heap(
                    ranking.begin(),
                    ranking.end(),
                    rankedSupportGripTriangleLess);
                return;
            }
            if (rankedSupportGripTriangleLess(
                    candidate,
                    ranking.front())) {
                std::pop_heap(
                    ranking.begin(),
                    ranking.end(),
                    rankedSupportGripTriangleLess);
                ranking.back() = candidate;
                std::push_heap(
                    ranking.begin(),
                    ranking.end(),
                    rankedSupportGripTriangleLess);
            }
        };

        std::uint64_t deterministicOrdinal = 0;
        for (const auto& evidenceView : evidenceViews) {
            if (evidenceView.localTriangles.empty() ||
                !weapon_support_acquisition_math::isUsableTransform(
                    evidenceView.localToWorld)) {
                continue;
            }
            const RE::NiTransform sourceToWeapon =
                transform_math::composeTransforms(
                    transform_math::invertTransform(weaponWorld),
                    evidenceView.localToWorld);
            if (!weapon_support_acquisition_math::isUsableTransform(
                    sourceToWeapon)) {
                continue;
            }

            for (const auto& sourceTriangle :
                 evidenceView.localTriangles) {
                const std::uint64_t ordinal =
                    deterministicOrdinal++;
                if (!grab_finger_pose_runtime::isFinitePoint(
                        sourceTriangle.v0) ||
                    !grab_finger_pose_runtime::isFinitePoint(
                        sourceTriangle.v1) ||
                    !grab_finger_pose_runtime::isFinitePoint(
                        sourceTriangle.v2)) {
                    continue;
                }

                const TriangleData weaponLocalTriangle{
                    transform_math::localPointToWorld(
                        sourceToWeapon,
                        sourceTriangle.v0),
                    transform_math::localPointToWorld(
                        sourceToWeapon,
                        sourceTriangle.v1),
                    transform_math::localPointToWorld(
                        sourceToWeapon,
                        sourceTriangle.v2),
                };
                float globalMinimumDistanceSquared =
                    (std::numeric_limits<float>::infinity)();
                (void)closestPointOnTriangleToPoint(
                    localReferences.seatPointWorld,
                    weaponLocalTriangle,
                    globalMinimumDistanceSquared);
                for (std::size_t lane = 0;
                     lane < kSupportGripFingerLaneCount;
                     ++lane) {
                    float laneMinimumDistanceSquared =
                        (std::numeric_limits<float>::infinity)();
                    for (std::size_t referenceIndex = 0;
                         referenceIndex <
                             localReferences.lanePointCounts[lane];
                         ++referenceIndex) {
                        float distanceSquared = 0.0f;
                        (void)closestPointOnTriangleToPoint(
                            localReferences.lanePointsWorld[lane]
                                [referenceIndex],
                            weaponLocalTriangle,
                            distanceSquared);
                        if (std::isfinite(distanceSquared)) {
                            laneMinimumDistanceSquared = (std::min)(
                                laneMinimumDistanceSquared,
                                distanceSquared);
                        }
                    }
                    if (std::isfinite(laneMinimumDistanceSquared)) {
                        globalMinimumDistanceSquared = (std::min)(
                            globalMinimumDistanceSquared,
                            laneMinimumDistanceSquared);
                        retainNearest(
                            rankingScratch[lane],
                            laneLimit,
                            RankedSupportGripTriangle{
                                .distanceSquared =
                                    laneMinimumDistanceSquared,
                                .deterministicOrdinal = ordinal,
                                .weaponLocalTriangle =
                                    weaponLocalTriangle,
                            });
                    }
                }
                if (std::isfinite(globalMinimumDistanceSquared)) {
                    retainNearest(
                        rankingScratch[
                            kSupportGripGlobalRankingIndex],
                        boundedLimit,
                        RankedSupportGripTriangle{
                            .distanceSquared =
                                globalMinimumDistanceSquared,
                            .deterministicOrdinal = ordinal,
                            .weaponLocalTriangle =
                                weaponLocalTriangle,
                        });
                }
            }
        }

        for (auto& ranking : rankingScratch) {
            std::sort(
                ranking.begin(),
                ranking.end(),
                rankedSupportGripTriangleLess);
        }
        std::vector<std::uint64_t> selectedOrdinals{};
        selectedOrdinals.reserve(boundedLimit);
        outTriangles.reserve(boundedLimit);
        const auto appendUnique = [&](
                                      const RankedSupportGripTriangle& ranked) {
            if (outTriangles.size() >= boundedLimit ||
                std::find(
                    selectedOrdinals.begin(),
                    selectedOrdinals.end(),
                    ranked.deterministicOrdinal) !=
                    selectedOrdinals.end()) {
                return false;
            }
            selectedOrdinals.push_back(
                ranked.deterministicOrdinal);
            outTriangles.push_back(ranked.weaponLocalTriangle);
            return true;
        };

        std::array<std::size_t,
            kSupportGripFingerLaneCount>
            laneCursors{};
        bool laneCandidateRemaining = true;
        while (outTriangles.size() < boundedLimit &&
               laneCandidateRemaining) {
            laneCandidateRemaining = false;
            for (std::size_t lane = 0;
                 lane < kSupportGripFingerLaneCount &&
                 outTriangles.size() < boundedLimit;
                 ++lane) {
                auto& cursor = laneCursors[lane];
                const auto& ranking = rankingScratch[lane];
                while (cursor < ranking.size()) {
                    laneCandidateRemaining = true;
                    const auto& candidate = ranking[cursor++];
                    if (appendUnique(candidate)) {
                        break;
                    }
                }
            }
        }
        for (const auto& ranked :
             rankingScratch[kSupportGripGlobalRankingIndex]) {
            if (outTriangles.size() >= boundedLimit) {
                break;
            }
            (void)appendUnique(ranked);
        }
    }

    ScopeHandAuthorityCleanupVisualSnapshot captureScopeHandAuthorityCleanupVisuals(RE::NiNode* weaponNode)
    {
        ScopeHandAuthorityCleanupVisualSnapshot snapshot{};
        if (weaponNode && isFiniteTransform(weaponNode->world)) {
            snapshot.weapon = weaponNode;
            snapshot.weaponWorld = weaponNode->world;
            snapshot.weaponValid = true;
        }

        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
        if (scopeCamera) {
            RE::NiTransform scopeCameraWorld = scopeCamera->world;
            if (scopeCamera->parent) {
                scopeCameraWorld = transform_math::composeTransforms(scopeCamera->parent->world, scopeCamera->local);
            }
            if (isFiniteTransform(scopeCameraWorld)) {
                snapshot.scopeCamera = scopeCamera;
                snapshot.scopeCameraWorld = scopeCameraWorld;
                snapshot.scopeCameraValid = true;
            }
        }
        return snapshot;
    }

    void restoreScopeHandAuthorityCleanupVisuals(const ScopeHandAuthorityCleanupVisualSnapshot& snapshot)
    {
        if (snapshot.weaponValid && snapshot.weapon) {
            (void)restoreWeaponRootPreservingPresentedDescendants(
                snapshot.weapon,
                snapshot.weaponWorld);
        }

        if (snapshot.scopeCameraValid && snapshot.scopeCamera) {
            if (snapshot.scopeCamera->parent) {
                snapshot.scopeCamera->local = weapon_visual_authority_math::worldTargetToParentLocal(
                    snapshot.scopeCamera->parent->world,
                    snapshot.scopeCameraWorld);
                f4vr::updateTransforms(snapshot.scopeCamera);
            } else {
                snapshot.scopeCamera->local = snapshot.scopeCameraWorld;
                snapshot.scopeCamera->world = snapshot.scopeCameraWorld;
            }
        }
    }

    NativeScopeCameraFollowCapture captureNativeScopeCameraFollow(const RE::NiNode* weaponNode)
    {
        const auto* playerNodes = f4vr::getPlayerNodes();
        auto* scopeCamera = playerNodes ? playerNodes->primaryWeaponScopeCamera : nullptr;
        if (!weaponNode || !scopeCamera || !isFiniteTransform(weaponNode->world)) {
            return {};
        }

        RE::NiTransform cameraWorld = scopeCamera->world;
        if (scopeCamera->parent) {
            cameraWorld = transform_math::composeTransforms(scopeCamera->parent->world, scopeCamera->local);
        }
        if (!isFiniteTransform(cameraWorld)) {
            return {};
        }

        return NativeScopeCameraFollowCapture{
            .camera = scopeCamera,
            .weaponWorldBefore = weaponNode->world,
            .cameraWorldBefore = cameraWorld,
            .valid = true,
        };
    }

    NativeScopeCameraFollowResult applyNativeScopeCameraWorldTarget(const NativeScopeCameraFollowCapture& capture, const RE::NiTransform& targetCameraWorld)
    {
        NativeScopeCameraFollowResult result{};
        if (!capture.valid || !capture.camera || !isFiniteTransform(targetCameraWorld)) {
            return result;
        }
        result.targetCameraWorld = targetCameraWorld;
        result.targetValid = true;

        auto* scopeCamera = capture.camera;
        if (scopeCamera->parent) {
            const RE::NiTransform targetCameraLocal = weapon_visual_authority_math::worldTargetToParentLocal(
                scopeCamera->parent->world,
                targetCameraWorld);
            if (!isFiniteTransform(targetCameraLocal)) {
                return result;
            }
            scopeCamera->local = targetCameraLocal;
            f4vr::updateTransforms(scopeCamera);
            result.writeApplied = true;
            const RE::NiTransform immediateCameraWorld = scopeCamera->world;
            if (isFiniteTransform(immediateCameraWorld)) {
                result.immediateCameraWorldAfter = immediateCameraWorld;
                result.immediateReadbackValid = true;
            }
            return result;
        }

        scopeCamera->local = targetCameraWorld;
        scopeCamera->world = targetCameraWorld;
        result.writeApplied = true;
        result.immediateCameraWorldAfter = scopeCamera->world;
        result.immediateReadbackValid = isFiniteTransform(result.immediateCameraWorldAfter);
        return result;
    }

    NativeScopeCameraDebugSnapshot makeNativeScopeCameraDebugSnapshot(const NativeScopeCameraDebugSnapshot& previous, const std::uint64_t weaponGenerationKey,
        const NativeScopeCameraWriteSource writeSource, const NativeScopeCameraFollowCapture& capture, const NativeScopeCameraFollowResult& result,
        const native_scope_sight_anchor_policy::AnchorSource anchorSource)
    {
        NativeScopeCameraDebugSnapshot snapshot{};
        snapshot.applySequence = previous.applySequence + 1;
        snapshot.weaponGenerationKey = weaponGenerationKey;
        snapshot.framesSinceApply = 0;
        snapshot.writeSource = writeSource;
        snapshot.captureValid = capture.valid;
        snapshot.targetValid = result.targetValid;
        snapshot.writeApplied = result.writeApplied;
        snapshot.immediateReadbackValid = result.immediateReadbackValid;
        snapshot.anchorSource = anchorSource;
        if (capture.valid) {
            snapshot.cameraWorldBefore = capture.cameraWorldBefore;
        }
        if (result.targetValid) {
            snapshot.targetCameraWorld = result.targetCameraWorld;
        }
        if (result.immediateReadbackValid) {
            snapshot.immediateCameraWorldAfter = result.immediateCameraWorldAfter;
        }
        return snapshot;
    }

    DirectSkeletonBoneReader& rootFlattenedTwoHandedReader()
    {
        static DirectSkeletonBoneReader reader;
        return reader;
    }

    static const DirectSkeletonBoneEntry* findSnapshotBone(const DirectSkeletonBoneSnapshot& snapshot, std::string_view name)
    {
        for (const auto& bone : snapshot.bones) {
            if (bone.name == name) {
                return &bone;
            }
        }
        return nullptr;
    }

    bool buildFullHandLocalTransformsForMeshPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose& meshFingerPose,
        const frik_visual_authority::HandPoseData& handPose,
        const DirectSkeletonBoneSnapshot* capturedFingerSnapshot,
        std::array<RE::NiTransform, 15>& outLocalTransforms,
        std::uint16_t& outMask)
    {
        auto* api = frik_visual_authority::api();
        const bool canPublish =
            grab_finger_local_transform_math::shouldPublishLocalTransformPose(
                g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                meshFingerPose.solved,
                true,
                api && api->getHandPoseLocalTransformsForPose != nullptr,
                api && api->setHandPoseCustomLocalTransforms != nullptr);
        if (!canPublish) {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: full-hand local transform override skipped hand={} enabled={} api={} baselineApi={} publishApi={}",
                isLeft ? "left" : "right",
                g_rockConfig.rockGrabMeshLocalTransformPoseEnabled ? "yes" : "no",
                api ? "yes" : "no",
                (api && api->getHandPoseLocalTransformsForPose) ? "yes" : "no",
                (api && api->setHandPoseCustomLocalTransforms) ? "yes" : "no");
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride baseline{};
        if (!frik_visual_authority::getHandPoseLocalTransformsForPose(handFromBool(isLeft), handPose, &baseline)) {
            ROCK_LOG_WARN(Weapon, "TwoHandedGrip: full-hand local transform override failed hand={} reason=baseline-query", isLeft ? "left" : "right");
            return false;
        }

        frik_visual_authority::FingerLocalTransformOverride corrected{};
        const char* failureReason = "unknown";
        if (!grab_finger_local_transform_runtime::buildSurfaceCorrectedLocalTransforms(isLeft,
                meshFingerPose,
                baseline,
                grab_finger_local_transform_runtime::Options{
                    .enabled = g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                    .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                    .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                    .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                    .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                    .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                    .thumbSurfaceSafetyEnabled = g_rockConfig.rockGrabThumbSurfaceSafetyEnabled,
                    .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
                },
                corrected,
                &failureReason,
                capturedFingerSnapshot)) {
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: full-hand local transform override failed hand={} reason={}",
                isLeft ? "left" : "right",
                failureReason ? failureReason : "unknown");
            return false;
        }

        outMask = corrected.enabledMask;
        for (std::size_t i = 0; i < outLocalTransforms.size(); ++i) {
            outLocalTransforms[i] = corrected.localTransforms[i];
        }
        return outMask == grab_finger_local_transform_math::kFullFingerLocalTransformMask;
    }

    bool tryGetRootFlattenedHandBoneTransform(bool isLeft, RE::NiTransform& outTransform)
    {
        outTransform = {};
        DirectSkeletonBoneSnapshot snapshot{};
        if (!rootFlattenedTwoHandedReader().capture(skeleton_bone_debug_math::DebugSkeletonBoneMode::HandsAndForearmsOnly,
                skeleton_bone_debug_math::DebugSkeletonBoneSource::GameRootFlattenedBoneTree,
                snapshot)) {
            return false;
        }

        const auto* handBone = findSnapshotBone(snapshot, isLeft ? "LArm_Hand" : "RArm_Hand");
        if (!handBone || !isUsableHandAuthorityTransform(handBone->world)) {
            return false;
        }

        outTransform = handBone->world;
        return true;
    }
}

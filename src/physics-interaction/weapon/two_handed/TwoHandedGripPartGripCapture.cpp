#include "physics-interaction/weapon/two_handed/TwoHandedGrip.h"

/*
 * PART GRIP CAPTURE: turning a hand near a weapon part into a committed grip.
 *
 * capturePartGrip at the bottom of this file is only the sequence. The work
 * lives in PartGripCapturePhases just above it, one method per acquisition
 * stage, called in this order: authored-support latch, dynamic mesh seat,
 * root-flattened finger references, evidence gather and frozen-mesh finger
 * solve, opposition pocket and pose selection, local-transform override,
 * publish.
 *
 * ORDERING COUPLING: the capture calls refreshAuthoredSupportGripActivationState
 * in TwoHandedGripAuthoredGrip.cpp mid-capture with requirePoseEvidence = true,
 * then reads the snapshot that call produced. The two files are tightly ordered
 * on purpose. The authored TU carries the matching note.
 *
 * The finger solve borrows the FingerPoseSolveScratch owned by the core TU, so a
 * re-grab does not reallocate its ranking vectors on the hot path.
 */
#include "physics-interaction/weapon/two_handed/TwoHandedGripInternal.h"

#include "api/ROCKProviderApiInternal.h"
#include "physics-interaction/actor/ActorEquipmentGrab.h"
#include "physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/weapon/authored_grip/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/WeaponAuthority.h"
#include "physics-interaction/weapon/collision/WeaponCollision.h"
#include "physics-interaction/weapon/WeaponGeometry.h"
#include "physics-interaction/weapon/WeaponSupport.h"
#include "RockConfig.h"
#include "RockUtils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>

namespace rock
{
    using two_handed_grip_detail::AuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::buildFullHandLocalTransformsForMeshPose;
    using two_handed_grip_detail::buildProviderPartTargetQuery;
    using two_handed_grip_detail::currentWeaponOppositionPocketConfig;
    using two_handed_grip_detail::GRIP_HAND_POSE_PRIORITY;
    using two_handed_grip_detail::isFiniteTransform;
    using two_handed_grip_detail::kSupportGripFingerLaneCount;
    using two_handed_grip_detail::kSupportGripFingerLaneReferenceCapacity;
    using two_handed_grip_detail::RankedSupportGripTriangle;
    using two_handed_grip_detail::resolveAuthoredSupportPalmSeatProximity;
    using two_handed_grip_detail::rootFlattenedTwoHandedReader;
    using two_handed_grip_detail::selectNearestSupportGripFingerTriangles;
    using two_handed_grip_detail::SUPPORT_GRIP_TAG;
    using two_handed_grip_detail::SupportGripFingerReferenceSet;
    using two_handed_grip_detail::tryGetRootFlattenedHandBoneTransform;
    using two_handed_grip_detail::WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS;
    using two_handed_grip_detail::WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS;

    namespace
    {
        void applyStableWeaponOppositionPose(
            grab_finger_pose_runtime::SolvedGrabFingerPose& pose,
            const grab_pinch_pocket_policy::Config& config,
            const std::size_t opposedFingerIndex)
        {
            const auto stable =
                grab_pinch_pocket_policy::
                    buildStableOppositionFingerPose(
                        config,
                        g_rockConfig.rockGrabFingerMinValue,
                        opposedFingerIndex);
            pose.values = stable.values;
            pose.jointValues = stable.jointValues;
            pose.surfaceAimTarget = {};
            pose.surfaceAimNormal = {};
            pose.surfaceAimTargetValid = {};
            pose.surfaceAimNormalValid = {};
            pose.surfaceAimTargetObjectLocal = {};
            pose.surfaceAimNormalObjectLocal = {};
            pose.surfaceAimTargetObjectLocalValid = {};
            pose.surfaceAimNormalObjectLocalValid = {};
            pose.contactArcRotationRadians = {};
            pose.contactArcRotationValid = {};
            pose.hasObjectLocalSurfaceAim = false;
            pose.usedAlternateThumbCurve = false;
            pose.usedAlternateThumbSurfaceHit = false;
            pose.hasJointValues = true;
            pose.solved = true;
        }


        // Present cached local evidence to the existing world-space selector.
        struct TransformedSupportGripTriangleView
        {
            std::span<const TriangleData> localTriangles{};
            RE::NiTransform localToWorld{};

            [[nodiscard]] std::size_t size() const noexcept
            {
                return localTriangles.size();
            }

            [[nodiscard]] TriangleData operator[](
                const std::size_t index) const
            {
                const auto& triangle = localTriangles[index];
                return TriangleData{
                    transform_math::localPointToWorld(
                        localToWorld,
                        triangle.v0),
                    transform_math::localPointToWorld(
                        localToWorld,
                        triangle.v1),
                    transform_math::localPointToWorld(
                        localToWorld,
                        triangle.v2),
                };
            }
        };

    }

    bool TwoHandedGrip::tryCaptureRootFlattenedPalmWorld(bool isLeft, RE::NiPoint3& outPalmWorld, RE::NiTransform& outHandWorld)
    {
        outPalmWorld = {};
        if (!tryGetRootFlattenedHandBoneTransform(isLeft, outHandWorld)) {
            return false;
        }
        outPalmWorld = computeGrabLegacyPalmPivotAWorldFromHandBasis(outHandWorld, isLeft);
        return true;
    }


    RE::NiPoint3 TwoHandedGrip::worldToWeaponLocal(const RE::NiPoint3& worldPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::worldPointToLocal(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, worldPos);
    }


    RE::NiPoint3 TwoHandedGrip::weaponLocalToWorld(const RE::NiPoint3& localPos, const RE::NiAVObject* weaponNode)
    {
        if (!weaponNode) {
            return {};
        }
        return weapon_collision_geometry_math::localPointToWorld(weaponNode->world.rotate, weaponNode->world.translate, weaponNode->world.scale, localPos);
    }


    RE::NiPoint3 TwoHandedGrip::resolvePartGripWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::localPointToWorld(supportAttachmentRoot->world, grip.gripSourceLocal);
        }
        return weaponLocalToWorld(grip.gripLocal, weaponNode);
    }


    RE::NiPoint3 TwoHandedGrip::resolvePartGripWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        return worldToWeaponLocal(resolvePartGripWorld(grip, weaponNode), weaponNode);
    }


    RE::NiPoint3 TwoHandedGrip::resolvePartGripNormalWeaponLocal(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            const RE::NiPoint3 supportNormalWorld = transform_math::localVectorToWorld(supportAttachmentRoot->world, grip.normalSourceLocal);
            return transform_math::worldVectorToLocal(weaponNode->world, supportNormalWorld);
        }
        return grip.normalLocal;
    }


    RE::NiTransform TwoHandedGrip::resolvePartGripHandWorld(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (auto* supportAttachmentRoot = resolveCurrentSupportAttachmentRoot(grip, weaponNode)) {
            return transform_math::composeTransforms(supportAttachmentRoot->world, grip.handSourceLocal);
        }
        if (!weaponNode) {
            return RE::NiTransform{};
        }
        return weapon_support_authority_policy::buildVisualOnlySupportHandWorld(weaponNode->world, grip.handWeaponLocal);
    }


    RE::NiAVObject* TwoHandedGrip::resolveCurrentSupportAttachmentRoot(const WeaponPartGrip& grip, RE::NiNode* weaponNode) const
    {
        if (!grip.hasSourceFrames || !grip.attachmentRoot || !weaponNode) {
            return nullptr;
        }
        return actor_equipment_grab::nodeContainsNode(weaponNode, grip.attachmentRoot, 64) ? grip.attachmentRoot : nullptr;
    }


    /*
     * One phase object owns one support-grip acquisition. Its references do not
     * escape capturePartGrip, and each method follows acquisition order.
     */
    struct TwoHandedGrip::PartGripCapturePhases
    {
        PartGripCapturePhases(
            TwoHandedGrip& ownerIn,
            const bool isLeftIn,
            RE::NiNode* const weaponNodeIn,
            const WeaponInteractionDecision& decisionIn,
            const WeaponCollision& weaponCollisionIn,
            const WeaponProviderPartAuthority& providerPartAuthorityIn,
            const bool firingGripProximityAuthorityEnabledIn) :
            owner(ownerIn),
            isLeft(isLeftIn),
            weaponNode(weaponNodeIn),
            decision(decisionIn),
            weaponCollision(weaponCollisionIn),
            providerPartAuthority(providerPartAuthorityIn),
            firingGripProximityAuthorityEnabled(
                firingGripProximityAuthorityEnabledIn),
            grip(ownerIn.partGrip(isLeftIn)),
            fingerScratch(
                ownerIn._fingerPoseSolveScratch->hands[
                    isLeftIn ? 0u : 1u])
        {}

        bool begin(RE::NiTransform* outCapturedHandWorld)
        {
            if (outCapturedHandWorld) {
                *outCapturedHandWorld = {};
            }
            if (!weaponNode) {
                return false;
            }
            if (!weapon_authority_lifecycle_policy::isWeaponContactGenerationCurrent(decision.weaponGenerationKey, owner._activeWeaponGenerationKey)) {
                ROCK_LOG_DEBUG(Weapon, "TwoHandedGrip: part grip capture skipped because contact generation is stale hand={}", isLeft ? "left" : "right");
                return false;
            }

            handTransform = {};
            if (!owner.tryGetSolverHandTransform(isLeft, handTransform)) {
                ROCK_LOG_WARN(Weapon, "TwoHandedGrip: part grip capture skipped because authoritative hand transforms are unavailable hand={}", isLeft ? "left" : "right");
                return false;
            }
            if (outCapturedHandWorld) {
                *outCapturedHandWorld = handTransform;
            }

            grip = {};
            supportAttachmentRoot = decision.sourceRoot ? decision.sourceRoot : static_cast<RE::NiAVObject*>(weaponNode);
            grip.gripPose = decision.gripPose != WeaponGripPoseId::None ? decision.gripPose : WeaponGripPoseId::BarrelWrap;
            grip.partKind = decision.partKind;
            grip.attachmentRoot = supportAttachmentRoot;
            grip.providerPartAuthority = providerPartAuthority.active ? providerPartAuthority : WeaponProviderPartAuthority{};
            grip.attachOnly = weapon_part_grip_report_policy::providerGrabModeIsAttachOnly(
                grip.providerPartAuthority.active,
                grip.providerPartAuthority.grabMode);
            grip.contactBodyId = decision.bodyId;
            grip.reloadRole = decision.reloadRole;
            grip.socketRole = decision.socketRole;
            grip.actionRole = decision.actionRole;
            grip.weaponGenerationKey = decision.weaponGenerationKey;
            grip.gripSequence = ++owner._gripCaptureSequence;
            {
                // The routing decision carries no support role or authored source
                // name; both come from the evidence descriptor keyed by the
                // contact body, matching the provider target-query construction.
                WeaponCollisionProfileEvidenceDescriptor descriptor{};
                RE::NiAVObject* descriptorSourceNode = nullptr;
                if (weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(decision.bodyId, descriptor, descriptorSourceNode) &&
                    descriptor.weaponGenerationKey == decision.weaponGenerationKey) {
                    grip.supportRole = descriptor.semantic.supportGripRole;
                    grip.omodFormId = descriptor.omodFormId;
                    grip.attachPointFormId = descriptor.semantic.attachPointFormId;
                    grip.classificationSource = descriptor.semantic.classificationSource;
                    const std::size_t copyLength = (std::min)(descriptor.sourceName.size(), grip.sourceName.size() - 1);
                    std::memcpy(grip.sourceName.data(), descriptor.sourceName.data(), copyLength);
                    grip.sourceName[copyLength] = '\0';
                } else if (grip.providerPartAuthority.active) {
                    grip.supportRole = static_cast<WeaponSupportGripRole>(grip.providerPartAuthority.supportRole);
                    grip.sourceName = grip.providerPartAuthority.sourceName;
                    grip.sourceName[grip.sourceName.size() - 1] = '\0';
                }
            }

            palmPos = computeGrabLegacyPalmPivotAWorldFromHandBasis(handTransform, isLeft);
            palmDir = computePalmNormalFromHandBasis(handTransform, isLeft);


            return true;
        }

        bool tryCaptureAuthoredSupportGrip()
        {
            /*
             * Acquisition-only authored priority. Physical contact and the
             * proximity probe are equivalent entry sources: an eligible authored
             * relation wins over both, but only inside the enforced family cone
             * and radial cap. The captured palm plus at least two distal
             * fingertips must also have current generated-mesh witnesses, so an
             * animation-zero/default support hand cannot escape to an unrelated
             * world-space pose. Every rejected authored candidate continues into
             * the unrestricted dynamic mesh grab below. The final authored seat
             * also selects visual-only versus full weapon authority. Provider
             * AttachOnly remains PAPER/consumer glue. Once selected, the exact
             * hand/weapon relation and 15 finger locals are latched; later
             * candidate changes cannot move it.
             */
            authoredWeaponIdentityMatches =
                owner._authoredSupportGripCandidate.weaponNode == weaponNode;
            authoredGenerationMatches =
                owner._authoredSupportGripCandidate.weaponGenerationKey ==
                decision.weaponGenerationKey;
            authoredSupportHandWeaponLocal = {};
            authoredSupportFingerLocalTransforms = {};
            authoredSupportFingerLocalTransformMask = 0;
            authoredSupportCandidateForHandValid =
                owner.tryResolveAuthoredSupportGripCandidateForHand(
                    isLeft,
                    weaponNode,
                    decision.weaponGenerationKey,
                    authoredSupportHandWeaponLocal,
                    authoredSupportFingerLocalTransforms,
                    authoredSupportFingerLocalTransformMask);
            authoredSupportProximity = {};
            authoredSupportHandWorld = {};
            authoredSupportPalmWeaponLocal = {};
            authoredSupportPalmNormalWorld = {};
            authoredSupportTouchProbeDistance =
                (std::numeric_limits<float>::infinity)();
            authoredSupportPalmToFiringGripDistance =
                (std::numeric_limits<float>::infinity)();
            authoredSupportFrameValid = false;
            if (authoredSupportCandidateForHandValid &&
                resolveAuthoredSupportPalmSeatProximity(
                    weaponNode->world,
                    handTransform,
                    authoredSupportHandWeaponLocal,
                    isLeft,
                    authoredSupportProximity)) {
                authoredSupportHandWorld = authoredSupportProximity.authoredHandWorld;
                authoredSupportPalmWeaponLocal =
                    authoredSupportProximity.authoredPalmSeatWeaponLocal;
                authoredSupportPalmNormalWorld =
                    computePalmNormalFromHandBasis(
                        authoredSupportHandWorld,
                        isLeft);
                authoredSupportTouchProbeDistance =
                    authoredSupportProximity.weaponRelativeDistanceGameUnits;
                authoredSupportFrameValid =
                    std::isfinite(authoredSupportTouchProbeDistance) &&
                    std::isfinite(authoredSupportPalmNormalWorld.x) &&
                    std::isfinite(authoredSupportPalmNormalWorld.y) &&
                    std::isfinite(authoredSupportPalmNormalWorld.z);
            }

            owner.refreshAuthoredSupportGripActivationState(
                weaponNode,
                decision.weaponGenerationKey,
                weaponCollision,
                true);
            const auto& authoredActivation =
                owner._authoredSupportGripDebugSnapshot;
            authoredActivationStateMatches =
                authoredActivation.valid &&
                authoredActivation.supportHandIsLeft == isLeft &&
                authoredActivation.weaponGenerationKey ==
                    decision.weaponGenerationKey &&
                authoredActivation.captureSequence ==
                    owner._authoredSupportGripCandidate.captureSequence;
            authoredActivationZoneValid =
                authoredActivationStateMatches &&
                authoredActivation.activationSpatialPass;
            authoredPoseSurfaceEvidenceValid =
                authoredActivationStateMatches &&
                authoredActivation.poseEvidencePass;
            authoredSeatWeaponSurfaceValid =
                authoredActivationStateMatches &&
                (authoredActivation.poseSurfaceWitnessMask & 0x01u) != 0;
            authoredSupportSurfaceDistance =
                authoredSeatWeaponSurfaceValid ?
                authoredActivation.poseSurfaceDistanceGameUnits[0] :
                (std::numeric_limits<float>::infinity)();

            authoredSupportAuthorityGateValid =
                !firingGripProximityAuthorityEnabled;
            authoredSupportAuthorityMode =
                weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;
            if (authoredSupportFrameValid &&
                firingGripProximityAuthorityEnabled &&
                std::isfinite(weaponNode->world.scale)) {
                const RE::NiPoint3 authoredSeatToFiringGripLocal =
                    sub(authoredSupportPalmWeaponLocal, owner._primaryGripLocal);
                const float authoredSeatToFiringGripLocalDistance = std::sqrt(
                    dot(authoredSeatToFiringGripLocal,
                        authoredSeatToFiringGripLocal));
                authoredSupportPalmToFiringGripDistance =
                    authoredSeatToFiringGripLocalDistance *
                    std::abs(weaponNode->world.scale);
                if (std::isfinite(authoredSupportPalmToFiringGripDistance)) {
                    authoredSupportAuthorityMode =
                        weapon_support_authority_policy::resolveFiringGripProximityAuthorityMode(
                            authoredSupportPalmToFiringGripDistance,
                            owner._handlingSettings.firingGripProximitySupportRadiusGameUnits);
                    authoredSupportAuthorityGateValid = true;
                }
            }

            authoredInteractionAcquisitionValid =
                decision.acquisitionSource ==
                    WeaponInteractionAcquisitionSource::PhysicalContact ||
                decision.acquisitionSource ==
                    WeaponInteractionAcquisitionSource::ProximityProbe;

            useAuthoredSupportGrip =
                authored_weapon_grip_capture_policy::shouldUseAuthoredSupportGrip(
                    authored_weapon_grip_capture_policy::AuthoredSupportGripCandidateInput{
                        .interactionAcquisitionValid =
                            authoredInteractionAcquisitionValid,
                        .activationZoneValid = authoredActivationZoneValid,
                        .authoredPoseSurfaceEvidenceValid =
                            authoredPoseSurfaceEvidenceValid,
                        .providerAuthorityActive = providerPartAuthority.active,
                        .attachOnly = grip.attachOnly,
                        .captureValid =
                            authoredSupportCandidateForHandValid &&
                            authoredSupportFrameValid &&
                            authoredSupportAuthorityGateValid,
                        .weaponIdentityMatches = authoredWeaponIdentityMatches,
                        .generationMatches = authoredGenerationMatches,
                        .authoredSeatWeaponSurfaceValid =
                            authoredSeatWeaponSurfaceValid,
                        .completeFingerPose =
                            authoredSupportFingerLocalTransformMask ==
                            authored_weapon_grip_library::
                                kCompleteFiringFingerMask,
                    });
            if (useAuthoredSupportGrip) {
                if (firingGripProximityAuthorityEnabled) {
                    owner._authorityMode = authoredSupportAuthorityMode;
                }
                grip.authoredSupportGrip = true;
                grip.authoredSupportCaptureSequence =
                    owner._authoredSupportGripCandidate.captureSequence;
                grip.attachmentRoot = weaponNode;
                grip.gripLocal = authoredSupportPalmWeaponLocal;
                grip.grabNormalWorld = authoredSupportPalmNormalWorld;
                grip.normalLocal = transform_math::worldVectorToLocal(
                    weaponNode->world,
                    authoredSupportPalmNormalWorld);
                grip.handWeaponLocal =
                    authoredSupportHandWeaponLocal;
                grip.hasHandWeaponLocal = true;
                grip.hasSourceFrames = false;
                grip.hasAttachmentWeaponLocal = false;

                // hFRIK requires the role-tagged numeric pose to exist before the
                // exact per-joint local override can win at the same priority.
                owner.setSupportGripPose(isLeft, nullptr, nullptr);
                grip.fingerLocalTransforms =
                    authoredSupportFingerLocalTransforms;
                grip.fingerLocalTransformMask =
                    authoredSupportFingerLocalTransformMask;
                grip.hasFingerLocalTransforms = true;
                grip.visualLerp = {};
                grip.active = true;

                performance_profiler::observeValue(
                    performance_profiler::ValueMetric::EquippedWeaponFingerPoseSourceTriangles,
                    0);
                performance_profiler::observeValue(
                    performance_profiler::ValueMetric::EquippedWeaponFingerPoseSelectedTriangles,
                    0);
                if (isLeft) {
                    owner._hapticEvents.leftPartGripCaptured = true;
                } else {
                    owner._hapticEvents.rightPartGripCaptured = true;
                }

                ROCK_LOG_INFO(Weapon,
                    "TwoHandedGrip: authored support grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) touchToSeat={:.3f} radialCap={:.3f} surfaceDistance={:.3f} poseWitnesses={}/6 poseMask={:02X} leftDot={:.3f} downDot={:.3f} cone={} authoredSeatToFiringGrip={:.3f} seatLocal=({:.3f},{:.3f},{:.3f}) touchLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} capture={} generation={:016X} acquisition={} authority={} priority=provider>authored>dynamic",
                    isLeft ? "left" : "right",
                    weaponNode->name.c_str(),
                    grip.gripLocal.x,
                    grip.gripLocal.y,
                    grip.gripLocal.z,
                    authoredSupportTouchProbeDistance,
                    authoredActivation.radialCapGameUnits,
                    authoredSupportSurfaceDistance,
                    static_cast<unsigned>(
                        authoredActivation.poseSurfaceWitnessCount),
                    static_cast<unsigned>(
                        authoredActivation.poseSurfaceWitnessMask),
                    authoredActivation.leftDot,
                    authoredActivation.downDot,
                    authored_weapon_grip_activation_policy::allowedConeName(
                        authoredActivation.selectedCone),
                    authoredSupportPalmToFiringGripDistance,
                    authoredSupportPalmWeaponLocal.x,
                    authoredSupportPalmWeaponLocal.y,
                    authoredSupportPalmWeaponLocal.z,
                    authoredSupportProximity.liveTouchProbeWeaponLocal.x,
                    authoredSupportProximity.liveTouchProbeWeaponLocal.y,
                    authoredSupportProximity.liveTouchProbeWeaponLocal.z,
                    authoredSupportProximity.frameAgreementErrorGameUnits,
                    grip.authoredSupportCaptureSequence,
                    owner._activeWeaponGenerationKey,
                    decision.acquisitionSource ==
                            WeaponInteractionAcquisitionSource::PhysicalContact ?
                        "physical-contact" :
                        "probe",
                    owner._authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                        "visual-only" :
                        "full");
                return true;
            }

            if (authoredSupportCandidateForHandValid) {
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: authored support grip rejected; continuing to dynamic hand={} source={} family={} activation={} pose={} palm={} witnesses={}/6 mask={:02X} distance={:.3f} cap={:.3f} leftDot={:.3f} downDot={:.3f} class={} radial={} direction={} scope={} provider={} attachOnly={} capture={} identity={} generation={} fingers={}",
                    isLeft ? "left" : "right",
                    decision.acquisitionSource ==
                            WeaponInteractionAcquisitionSource::PhysicalContact ?
                        "physical-contact" :
                        (decision.acquisitionSource ==
                                WeaponInteractionAcquisitionSource::ProximityProbe ?
                            "probe" : "none"),
                    authored_weapon_grip_activation_policy::weaponFamilyName(
                        authoredActivation.weaponFamily),
                    authoredActivationZoneValid ? "pass" : "fail",
                    authoredPoseSurfaceEvidenceValid ? "pass" : "fail",
                    authoredSeatWeaponSurfaceValid ? "pass" : "fail",
                    static_cast<unsigned>(
                        authoredActivation.poseSurfaceWitnessCount),
                    static_cast<unsigned>(
                        authoredActivation.poseSurfaceWitnessMask),
                    authoredActivation.weaponRelativeDistanceGameUnits,
                    authoredActivation.radialCapGameUnits,
                    authoredActivation.leftDot,
                    authoredActivation.downDot,
                    authoredActivation.classifierSupported ? "pass" : "fail",
                    authoredActivation.radialPass ? "pass" : "fail",
                    authoredActivation.directionPass ? "pass" : "fail",
                    authoredActivation.scopePass ? "pass" : "fail",
                    providerPartAuthority.active ? "yes" : "no",
                    grip.attachOnly ? "yes" : "no",
                    authoredSupportFrameValid &&
                            authoredSupportAuthorityGateValid ?
                        "pass" : "fail",
                    authoredWeaponIdentityMatches ? "pass" : "fail",
                    authoredGenerationMatches ? "pass" : "fail",
                    authoredSupportFingerLocalTransformMask ==
                            authored_weapon_grip_library::
                                kCompleteFiringFingerMask ?
                        "pass" : "fail");
            }


            return false;
        }

        void captureDynamicMeshSeat()
        {
            for (auto& ranking : fingerScratch.rankings) {
                ranking.clear();
            }
            fingerScratch.localTriangles.clear();
            fingerScratch.worldTriangles.clear();
            fingerScratch.spatialIndex.clear();

            evidenceView = {};
            cachedTrianglesFound = weaponCollision.tryGetSupportGripEvidenceView(decision.bodyId, weaponNode, evidenceView) &&
                evidenceView.weaponGenerationKey == decision.weaponGenerationKey &&
                evidenceView.weaponGenerationKey == owner._activeWeaponGenerationKey;
            contactedSourceTriangleCount =
                cachedTrianglesFound ?
                evidenceView.localTriangles.size() :
                0u;

            grabPoint = {};
            meshFound = false;
            if (cachedTrianglesFound) {
                const TransformedSupportGripTriangleView worldEvidence{
                    .localTriangles = evidenceView.localTriangles,
                    .localToWorld = evidenceView.localToWorld,
                };
                meshFound = findClosestGrabPoint(
                    worldEvidence,
                    palmPos,
                    palmDir,
                    g_rockConfig.rockGrabLateralWeight,
                    g_rockConfig.rockGrabDirectionalWeight,
                    grabPoint,
                    g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits);
            }

            if (meshFound) {
                grip.gripLocal = owner.worldToWeaponLocal(grabPoint.position, weaponNode);
                grip.grabNormalWorld = grabPoint.normal;
            } else {
                grip.gripLocal = owner.worldToWeaponLocal(palmPos, weaponNode);
                grip.grabNormalWorld = palmDir;
            }
            gripWorldPoint = meshFound ? grabPoint.position : palmPos;
            const float surfaceSeatMaxRadians =
                g_rockConfig.rockWeaponSupportSurfaceSeatEnabled && meshFound ?
                g_rockConfig.rockWeaponSupportSurfaceSeatMaxDegrees *
                    two_handed_grip_detail::kDegreesToRadians :
                0.0f;
            const auto surfaceSeat =
                weapon_support_acquisition_math::
                    alignHandFrameToGripSurface<
                        RE::NiTransform,
                        RE::NiPoint3>(
                        handTransform,
                        palmPos,
                        palmDir,
                        gripWorldPoint,
                        grip.grabNormalWorld,
                        surfaceSeatMaxRadians);
            adjustedHandTransform =
                surfaceSeat.valid ?
                surfaceSeat.handWorld :
                weapon_two_handed_grip_math::alignHandFrameToGripPoint(
                    handTransform,
                    palmPos,
                    gripWorldPoint);
            grip.surfaceSeatRotationRadians =
                surfaceSeat.valid ?
                surfaceSeat.appliedRotationRadians :
                0.0f;
            const RE::NiPoint3 seatedPalmNormal =
                computePalmNormalFromHandBasis(
                    adjustedHandTransform,
                    isLeft);
            grip.handWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), adjustedHandTransform);
            grip.hasHandWeaponLocal = true;
            grip.normalLocal = transform_math::worldVectorToLocal(
                weaponNode->world,
                seatedPalmNormal);
            if (supportAttachmentRoot) {
                grip.gripSourceLocal = transform_math::worldPointToLocal(supportAttachmentRoot->world, gripWorldPoint);
                grip.normalSourceLocal = transform_math::worldVectorToLocal(
                    supportAttachmentRoot->world,
                    seatedPalmNormal);
                grip.handSourceLocal = transform_math::composeTransforms(transform_math::invertTransform(supportAttachmentRoot->world), adjustedHandTransform);
                grip.attachmentWeaponLocal = transform_math::composeTransforms(transform_math::invertTransform(weaponNode->world), supportAttachmentRoot->world);
                grip.hasSourceFrames = true;
                grip.hasAttachmentWeaponLocal = true;
            }


        }

        void captureRootFlattenedFingerReferences()
        {
            /*
             * Capture the root-flattened fingers once for this transaction. The
             * compact sweep snapshot and any exact-local thumb/surface correction
             * must describe the same pre-authority hand, not two scene reads split
             * by grip publication.
             */
            capturedFingerBoneSnapshot = {};
            capturedFingerBoneSnapshotValid =
                rootFlattenedTwoHandedReader().capture(
                    skeleton_bone_debug_math::DebugSkeletonBoneMode::
                        HandsAndForearmsOnly,
                    skeleton_bone_debug_math::DebugSkeletonBoneSource::
                        GameRootFlattenedBoneTree,
                    capturedFingerBoneSnapshot);
            capturedFingerSnapshot = {};
            capturedFingerSnapshotValid =
                capturedFingerBoneSnapshotValid &&
                root_flattened_finger_skeleton_runtime::
                    buildFingerSkeletonSnapshot(
                        capturedFingerBoneSnapshot,
                        isLeft,
                        capturedFingerSnapshot);
            fingerReferenceSet = {};
            fingerReferenceSet.seatPointWorld = gripWorldPoint;
            fingerReferenceSet.seatPointValid =
                grab_finger_pose_runtime::isFinitePoint(gripWorldPoint);
            if (capturedFingerSnapshotValid) {
                const RE::NiTransform rawToSeatedWorld =
                    transform_math::composeTransforms(
                        adjustedHandTransform,
                        transform_math::invertTransform(handTransform));
                const auto liveLandmarks =
                    root_flattened_finger_skeleton_runtime::
                        buildLandmarkSet(capturedFingerSnapshot);
                std::array<RE::NiPoint3,
                    kSupportGripFingerLaneCount>
                    commandedOpenDirectionsWorld{};
                const bool commandedDirectionsValid =
                    grab_finger_pose_runtime::
                        resolveCommandedOpenDirectionsWorld(
                            isLeft,
                            adjustedHandTransform,
                            commandedOpenDirectionsWorld);
                const RE::NiPoint3 seatedSweepNormal =
                    liveLandmarks.valid ?
                    transform_math::localVectorToWorld(
                        rawToSeatedWorld,
                        liveLandmarks.palmNormalWorld) :
                    RE::NiPoint3{};
            const auto appendLanePoint = [this](
                                                 const std::size_t lane,
                                                 const RE::NiPoint3& pointWorld) {
                    if (lane >= kSupportGripFingerLaneCount ||
                        !grab_finger_pose_runtime::isFinitePoint(
                            pointWorld)) {
                        return;
                    }
                    auto& count =
                        fingerReferenceSet.lanePointCounts[lane];
                    if (count >=
                        kSupportGripFingerLaneReferenceCapacity) {
                        return;
                    }
                    fingerReferenceSet.lanePointsWorld[lane][count++] =
                        pointWorld;
                };

                for (std::size_t finger = 0;
                     finger < capturedFingerSnapshot.fingers.size();
                     ++finger) {
                    const auto& chain =
                        capturedFingerSnapshot.fingers[finger];
                    if (!chain.valid) {
                        continue;
                    }
                    for (const auto& pointWorld : chain.points) {
                        appendLanePoint(
                            finger,
                            transform_math::localPointToWorld(
                                rawToSeatedWorld,
                                pointWorld));
                    }

                    if (!liveLandmarks.valid ||
                        !commandedDirectionsValid ||
                        finger >= liveLandmarks.fingers.size() ||
                        !liveLandmarks.fingers[finger].valid ||
                        !std::isfinite(
                            liveLandmarks.fingers[finger].length) ||
                        liveLandmarks.fingers[finger].length <=
                            0.0001f) {
                        continue;
                    }
                    const RE::NiPoint3 seatedBase =
                        transform_math::localPointToWorld(
                            rawToSeatedWorld,
                            liveLandmarks.fingers[finger].base);
                    const auto sweepCurve =
                        grab_finger_pose_math::
                            makeBakedCalibratedFingerCurve<
                                RE::NiPoint3>(
                                finger,
                                isLeft,
                                capturedFingerSnapshot.inPowerArmor,
                                seatedBase,
                                seatedSweepNormal,
                                commandedOpenDirectionsWorld[finger],
                                liveLandmarks.fingers[finger].length);
                    const auto* tipProbe =
                        sweepCurve.probeCount > 0 ?
                        &sweepCurve.probes[0] :
                        nullptr;
                    if (!tipProbe || tipProbe->sampleCount == 0 ||
                        tipProbe->sampleCount >
                            tipProbe->samples.size()) {
                        continue;
                    }
                    constexpr std::size_t kSweepSamples = 7;
                    const RE::NiPoint3 curveNormal =
                        grab_finger_pose_runtime::normalizedOrFallback(
                            sweepCurve.normal,
                            seatedSweepNormal);
                    const RE::NiPoint3 curveZero =
                        grab_finger_pose_runtime::normalizedOrFallback(
                            sweepCurve.zeroAngleVector,
                            commandedOpenDirectionsWorld[finger]);
                    for (std::size_t sample = 0;
                         sample < kSweepSamples;
                         ++sample) {
                        const std::size_t row =
                            sample * (tipProbe->sampleCount - 1) /
                            (kSweepSamples - 1);
                        const auto& baked = tipProbe->samples[row];
                        const RE::NiPoint3 arm =
                            grab_finger_pose_math::rotateAroundUnitAxis(
                                curveZero,
                                curveNormal,
                                baked.angleRadians);
                        appendLanePoint(
                            finger,
                            grab_finger_pose_math::add(
                                sweepCurve.center,
                                grab_finger_pose_math::scale(
                                    arm,
                                    baked.reachLength)));
                    }
                }
            }


        }

        void gatherEvidenceAndSolveFrozenFingerPose()
        {
            compositeEvidenceViews = {};
            compositeEvidenceViewCount = 0;
            sourceTriangleCount = 0;
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled) {
                const std::size_t discoveredViewCount =
                    weaponCollision.findSupportGripEvidenceViews(
                        weaponNode,
                        compositeEvidenceViews);
                for (std::size_t index = 0;
                     index < discoveredViewCount;
                     ++index) {
                    const auto& candidateView =
                        compositeEvidenceViews[index];
                    if (candidateView.weaponGenerationKey !=
                            decision.weaponGenerationKey ||
                        candidateView.weaponGenerationKey !=
                            owner._activeWeaponGenerationKey ||
                        candidateView.localTriangles.empty()) {
                        continue;
                    }
                    if (compositeEvidenceViewCount != index) {
                        compositeEvidenceViews[
                            compositeEvidenceViewCount] = candidateView;
                    }
                    sourceTriangleCount +=
                        candidateView.localTriangles.size();
                    ++compositeEvidenceViewCount;
                }
                if (compositeEvidenceViewCount == 0 &&
                    cachedTrianglesFound) {
                    compositeEvidenceViews[0] = evidenceView;
                    compositeEvidenceViewCount = 1;
                    sourceTriangleCount =
                        evidenceView.localTriangles.size();
                }
                selectNearestSupportGripFingerTriangles(
                    std::span<const WeaponCollision::SupportGripEvidenceView>(
                        compositeEvidenceViews.data(),
                        compositeEvidenceViewCount),
                    weaponNode->world,
                    fingerReferenceSet,
                    grab_finger_pose_runtime::
                        kMaxFingerPoseCandidateTriangles,
                    fingerScratch.rankings,
                    fingerScratch.localTriangles);
            }
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::
                    EquippedWeaponFingerPoseSourceTriangles,
                static_cast<std::uint64_t>(sourceTriangleCount));
            performance_profiler::observeValue(
                performance_profiler::ValueMetric::
                    EquippedWeaponFingerPoseSelectedTriangles,
                static_cast<std::uint64_t>(
                    fingerScratch.localTriangles.size()));

            frozenSolve = {};
            frozenGripPoint = {};
            meshFingerPose = {};
            meshFingerPosePtr = nullptr;
            capturedFingerSplayRadians = {};
            capturedFingerSplayRadiansPtr = nullptr;
            spatialIndexBuilt = false;
            commandedOpenDirectionsValid = false;
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && !fingerScratch.localTriangles.empty()) {
                const RE::NiTransform seatedToRawWorld =
                    transform_math::composeTransforms(
                        handTransform,
                        transform_math::invertTransform(
                            adjustedHandTransform));
                const RE::NiTransform frozenMeshWorld = weapon_two_handed_grip_math::virtualizeMeshForSeatedHand(
                    weaponNode->world,
                    handTransform,
                    adjustedHandTransform);
                frozenGripPoint = weapon_two_handed_grip_math::virtualizeWorldPointForSeatedHand(
                    gripWorldPoint,
                    handTransform,
                    adjustedHandTransform);
                const RE::NiPoint3 frozenGripNormal =
                    transform_math::localVectorToWorld(
                        seatedToRawWorld,
                        grip.grabNormalWorld);
                auto fingerPoseTargets = grab_finger_pose_runtime::makeSharedGripPoseTarget(frozenGripPoint, frozenGripNormal);
                fingerPoseTargets.useSeatPointForMissingTargets = false;
                fingerPoseTargets.useWholeMeshForMissingTargets = true;
                /*
                 * Equipped support keeps the indexed frozen base, but owns its
                 * presentation policy. Loose-grab thumb/index clearing and generic
                 * pad-probe refinement erased useful weapon-surface opposition.
                 */
                frozenSolve =
                    grab_finger_pose_runtime::solveFrozenMeshFingerPoseBase(
                    fingerScratch.localTriangles,
                    frozenMeshWorld,
                    handTransform,
                    isLeft,
                    frozenGripPoint,
                    fingerPoseTargets,
                    fingerScratch.spatialIndex,
                    fingerScratch.worldTriangles,
                    grab_finger_pose_runtime::FrozenMeshFingerPoseSolveOptions{
                        .minValue = g_rockConfig.rockGrabFingerMinValue,
                        .maxTriangleDistanceSquared = g_rockConfig.rockGrabMaxTriangleDistance,
                        .rejectBacksideHits = g_rockConfig.rockGrabFingerRejectBacksideHits,
                        .surfacePlaneToleranceGameUnits = g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                        .allowSurfaceAimTargets = true,
                        .sweepContactRadiusGameUnits = g_rockConfig.rockGrabFingerSweepContactRadiusGameUnits,
                        .thumbSweepMaxOpenValue = g_rockConfig.rockGrabThumbSweepMaxOpenValue,
                        .fingerSweepMaxOpenValue = g_rockConfig.rockGrabFingerSweepMaxOpenValue,
                        .meshFingerPoseEnabled = g_rockConfig.rockGrabMeshFingerPoseEnabled,
                        .captureSweepDebug = false,
                    },
                    capturedFingerSnapshotValid ?
                        &capturedFingerSnapshot :
                        nullptr);
                grab_finger_pose_runtime::captureSurfaceAimObjectLocal(
                    frozenSolve.pose,
                    frozenMeshWorld);
                spatialIndexBuilt = frozenSolve.spatialIndexBuilt;
                commandedOpenDirectionsValid = frozenSolve.commandedOpenDirectionsValid;
                meshFingerPose = frozenSolve.pose;
                performance_profiler::observeValue(
                    performance_profiler::ValueMetric::EquippedWeaponFingerPoseSpatialNodeVisits,
                    meshFingerPose.spatialNodeVisitCount);
                performance_profiler::observeValue(
                    performance_profiler::ValueMetric::EquippedWeaponFingerPoseTriangleTests,
                    meshFingerPose.spatialTriangleTestCount);

            }


        }

        void applyOppositionPocketAndSelectFingerPose()
        {
            if (meshFingerPose.solved) {
                const bool completeDirectFingerEvidence =
                    grab_finger_pose_runtime::
                        hasCompleteFingerContactEvidence(
                            meshFingerPose);
                const auto oppositionConfig =
                    currentWeaponOppositionPocketConfig();
                const auto oppositionPocket =
                    !completeDirectFingerEvidence &&
                            oppositionConfig.enabled &&
                            frozenSolve.liveFingerSnapshotValid ?
                        grab_finger_pose_runtime::
                            findLocalOppositionPocketEvidence(
                                fingerScratch.worldTriangles,
                                frozenSolve.liveFingerSnapshot,
                                frozenGripPoint,
                                meshFingerPose.contactValidMask,
                                oppositionConfig.
                                    minFingerGapGameUnits,
                                (std::max)(
                                    oppositionConfig.
                                        maxFingerGapGameUnits,
                                    WEAPON_OPPOSITION_MAX_FINGER_GAP_GAME_UNITS),
                                oppositionConfig.
                                    maxPocketDistanceGameUnits,
                                (std::min)(
                                    WEAPON_OPPOSITION_SEGMENT_PROBE_RADIUS_GAME_UNITS,
                                    (std::max)(
                                        0.0f,
                                        g_rockConfig.
                                            rockGrabFingerSweepContactRadiusGameUnits))) :
                        grab_finger_pose_runtime::
                            OppositionPocketEvidence{};
                if (oppositionPocket.valid) {
                    applyStableWeaponOppositionPose(
                        meshFingerPose,
                        oppositionConfig,
                        oppositionPocket.opposedFingerIndex);
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: local opposition pocket accepted hand={} kind={} directMask=0x{:02X} endpointMask=0x{:02X} directEndpoints=0x{:02X} gap={:.3f} gripSurfaceDistance={:.3f}",
                        isLeft ? "left" : "right",
                        grab_finger_pose_runtime::
                            oppositionPocketKindName(
                                oppositionPocket.kind),
                        static_cast<unsigned>(
                            meshFingerPose.contactValidMask),
                        static_cast<unsigned>(
                            oppositionPocket.endpointMask),
                        static_cast<unsigned>(
                            oppositionPocket.directEndpointMask),
                        oppositionPocket.fingerGapGameUnits,
                        oppositionPocket.
                            gripToSurfaceDistanceGameUnits);
                }
                const bool completeFingerEvidence =
                    completeDirectFingerEvidence ||
                    oppositionPocket.valid;
                if (completeFingerEvidence) {
                    meshFingerPosePtr = &meshFingerPose;
                } else {
                    ROCK_LOG_INFO(
                        Weapon,
                        "TwoHandedGrip: mesh finger pose failed closed hand={} contactMask=0x{:02X} requiredMask=0x{:02X} hits={} sources={} sourceTriangles={} candidateTriangles={}",
                        isLeft ? "left" : "right",
                        static_cast<unsigned>(
                            meshFingerPose.contactValidMask),
                        static_cast<unsigned>(
                            grab_finger_pose_runtime::
                                kCompleteFingerContactMask),
                        meshFingerPose.hitCount,
                        compositeEvidenceViewCount,
                        sourceTriangleCount,
                        meshFingerPose.candidateTriangleCount);
                }
                if (completeDirectFingerEvidence &&
                    frozenSolve.liveFingerSnapshotValid &&
                    grab_finger_pose_runtime::buildSurfaceContactSplayValues(
                        meshFingerPose,
                        frozenSolve.liveFingerSnapshot,
                        capturedFingerSplayRadians)) {
                    capturedFingerSplayRadiansPtr = &capturedFingerSplayRadians;
                }
                ROCK_LOG_DEBUG(Weapon,
                    "TwoHandedGrip: mesh finger pose hand={} values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} contactMask=0x{:02X} sources={} sourceTris={} candidateTris={} spatial={} nodes={} tests={} commandedAnchors={} altThumb={} thumbLane={}",
                    isLeft ? "left" : "right",
                    meshFingerPose.values[0],
                    meshFingerPose.values[1],
                    meshFingerPose.values[2],
                    meshFingerPose.values[3],
                    meshFingerPose.values[4],
                    meshFingerPose.hitCount,
                    static_cast<unsigned>(
                        meshFingerPose.contactValidMask),
                    compositeEvidenceViewCount,
                    sourceTriangleCount,
                    meshFingerPose.candidateTriangleCount,
                    spatialIndexBuilt ? "yes" : "no",
                    meshFingerPose.spatialNodeVisitCount,
                    meshFingerPose.spatialTriangleTestCount,
                    commandedOpenDirectionsValid ? "yes" : "no",
                    meshFingerPose.usedAlternateThumbCurve ? "yes" : "no",
                    grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                if (meshFingerPose.hasThumbCurveDiagnostics) {
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: thumb curve primary(hit={} value={:.2f} behind={}) opposition(hit={} value={:.2f} behind={}) sidePad(hit={} value={:.2f} behind={}) selected={}",
                        meshFingerPose.thumbPrimaryCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbPrimaryCurve.value,
                        meshFingerPose.thumbPrimaryCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbAlternateCurve.value,
                        meshFingerPose.thumbAlternateCurve.openedByBehindContact ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.hit ? "yes" : "no",
                        meshFingerPose.thumbSidePadCurve.value,
                        meshFingerPose.thumbSidePadCurve.openedByBehindContact ? "yes" : "no",
                        grab_finger_pose_math::thumbLaneName(meshFingerPose.selectedThumbLane));
                }
            }
        }


        void prepareFingerLocalTransformOverride()
        {
            owner.setSupportGripPose(
                isLeft,
                meshFingerPosePtr,
                capturedFingerSplayRadiansPtr,
                SupportGripPoseFallback::FullyClosed);
            if (meshFingerPosePtr && grip.hasFingerPose) {
                std::array<RE::NiTransform, 15> localTransforms{};
                std::uint16_t localTransformMask = 0;
                const auto handPose = grip.hasFingerSplay ?
                    frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose, grip.fingerSplayRadians) :
                    frik_visual_authority::makeHandPoseDataFromJointValues(grip.fingerPose);
                if (buildFullHandLocalTransformsForMeshPose(
                        isLeft,
                        *meshFingerPosePtr,
                        handPose,
                        capturedFingerBoneSnapshotValid ?
                            &capturedFingerBoneSnapshot :
                            nullptr,
                        localTransforms,
                        localTransformMask)) {
                    grip.fingerLocalTransforms = localTransforms;
                    grip.fingerLocalTransformMask = localTransformMask;
                    grip.hasFingerLocalTransforms = true;
                    ROCK_LOG_DEBUG(Weapon,
                        "TwoHandedGrip: full-hand local transform override prepared hand={} mask=0x{:04X}",
                        isLeft ? "left" : "right",
                        grip.fingerLocalTransformMask);
                }
            }
        }


        bool publishDynamicGrip()
        {

            grip.visualLerp = {};
            grip.active = true;
            if (isLeft) {
                owner._hapticEvents.leftPartGripCaptured = true;
            } else {
                owner._hapticEvents.rightPartGripCaptured = true;
            }

            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: part grip captured hand={} weapon='{}' gripLocal=({:.3f},{:.3f},{:.3f}) meshGrab={} sourceTriangles={} sources={} contactedTriangles={} fingerTriangles={} cachedTriangles={} sourceNodeCurrent={} surfaceSeat={:.2f}deg authoredSupport=NO acquisition={} authority={} provider={} attachOnly={} authoredCandidate={} authoredFrame={} authoredIdentity={} authoredGeneration={} authoredSurface={} surfaceDistance={:.3f} surfaceRadius={:.3f} authoredFingerMask=0x{:04X} touchToAuthoredSeat={:.3f} authoredSeatLocal=({:.3f},{:.3f},{:.3f}) touchProbeLocal=({:.3f},{:.3f},{:.3f}) frameError={:.4f} partKind={} pose={} generation={:016X}",
                isLeft ? "left" : "right",
                weaponNode->name.c_str(),
                grip.gripLocal.x,
                grip.gripLocal.y,
                grip.gripLocal.z,
                meshFound ? "YES" : "FALLBACK",
                sourceTriangleCount,
                compositeEvidenceViewCount,
                contactedSourceTriangleCount,
                fingerScratch.localTriangles.size(),
                cachedTrianglesFound ? "yes" : "no",
                cachedTrianglesFound && evidenceView.sourceNodeCurrent ? "yes" : "no",
                grip.surfaceSeatRotationRadians *
                    two_handed_grip_detail::kRadiansToDegrees,
                decision.acquisitionSource == WeaponInteractionAcquisitionSource::PhysicalContact ?
                    "contact" :
                    (decision.acquisitionSource == WeaponInteractionAcquisitionSource::ProximityProbe ? "probe" : "none"),
                owner._authorityMode == weapon_support_authority_policy::WeaponSupportAuthorityMode::VisualOnlySupport ?
                    "visual-only" :
                    "full",
                providerPartAuthority.active ? "yes" : "no",
                grip.attachOnly ? "yes" : "no",
                authoredSupportCandidateForHandValid ? "yes" : "no",
                authoredSupportFrameValid ? "yes" : "no",
                authoredWeaponIdentityMatches ? "yes" : "no",
                authoredGenerationMatches ? "yes" : "no",
                authoredSeatWeaponSurfaceValid ? "yes" : "no",
                authoredSupportSurfaceDistance,
                g_rockConfig.rockWeaponInteractionTouchRadius,
                authoredSupportFingerLocalTransformMask,
                authoredSupportTouchProbeDistance,
                authoredSupportPalmWeaponLocal.x,
                authoredSupportPalmWeaponLocal.y,
                authoredSupportPalmWeaponLocal.z,
                authoredSupportProximity.liveTouchProbeWeaponLocal.x,
                authoredSupportProximity.liveTouchProbeWeaponLocal.y,
                authoredSupportProximity.liveTouchProbeWeaponLocal.z,
                authoredSupportProximity.frameAgreementErrorGameUnits,
                static_cast<int>(grip.partKind),
                static_cast<int>(grip.gripPose),
                owner._activeWeaponGenerationKey);
            return true;

        }

        TwoHandedGrip& owner;
        bool isLeft{ false };
        RE::NiNode* weaponNode{ nullptr };
        const WeaponInteractionDecision& decision;
        const WeaponCollision& weaponCollision;
        const WeaponProviderPartAuthority& providerPartAuthority;
        bool firingGripProximityAuthorityEnabled{ false };

        WeaponPartGrip& grip;
        FingerPoseSolveScratch::HandScratch& fingerScratch;
        RE::NiTransform handTransform{};
        RE::NiAVObject* supportAttachmentRoot{ nullptr };
        RE::NiPoint3 palmPos{};
        RE::NiPoint3 palmDir{};

        bool authoredWeaponIdentityMatches{ false };
        bool authoredGenerationMatches{ false };
        RE::NiTransform authoredSupportHandWeaponLocal{};
        std::array<RE::NiTransform, 15>
            authoredSupportFingerLocalTransforms{};
        std::uint16_t authoredSupportFingerLocalTransformMask{ 0 };
        bool authoredSupportCandidateForHandValid{ false };
        AuthoredSupportPalmSeatProximity authoredSupportProximity{};
        RE::NiTransform authoredSupportHandWorld{};
        RE::NiPoint3 authoredSupportPalmWeaponLocal{};
        RE::NiPoint3 authoredSupportPalmNormalWorld{};
        float authoredSupportTouchProbeDistance{
            (std::numeric_limits<float>::infinity)()
        };
        float authoredSupportPalmToFiringGripDistance{
            (std::numeric_limits<float>::infinity)()
        };
        bool authoredSupportFrameValid{ false };
        bool authoredActivationStateMatches{ false };
        bool authoredActivationZoneValid{ false };
        bool authoredPoseSurfaceEvidenceValid{ false };
        bool authoredSeatWeaponSurfaceValid{ false };
        float authoredSupportSurfaceDistance{
            (std::numeric_limits<float>::infinity)()
        };
        bool authoredSupportAuthorityGateValid{ false };
        weapon_support_authority_policy::WeaponSupportAuthorityMode
            authoredSupportAuthorityMode{
                weapon_support_authority_policy::
                    WeaponSupportAuthorityMode::FullTwoHandedSolver
            };
        bool authoredInteractionAcquisitionValid{ false };
        bool useAuthoredSupportGrip{ false };

        WeaponCollision::SupportGripEvidenceView evidenceView{};
        bool cachedTrianglesFound{ false };
        std::size_t contactedSourceTriangleCount{ 0 };
        GrabPoint grabPoint{};
        bool meshFound{ false };
        RE::NiPoint3 gripWorldPoint{};
        RE::NiTransform adjustedHandTransform{};

        DirectSkeletonBoneSnapshot capturedFingerBoneSnapshot{};
        bool capturedFingerBoneSnapshotValid{ false };
        root_flattened_finger_skeleton_runtime::Snapshot
            capturedFingerSnapshot{};
        bool capturedFingerSnapshotValid{ false };
        SupportGripFingerReferenceSet fingerReferenceSet{};

        std::array<WeaponCollision::SupportGripEvidenceView,
            MAX_WEAPON_COLLISION_BODIES>
            compositeEvidenceViews{};
        std::size_t compositeEvidenceViewCount{ 0 };
        std::size_t sourceTriangleCount{ 0 };
        grab_finger_pose_runtime::FrozenMeshFingerPoseSolveResult
            frozenSolve{};
        RE::NiPoint3 frozenGripPoint{};
        grab_finger_pose_runtime::SolvedGrabFingerPose meshFingerPose{};
        const grab_finger_pose_runtime::SolvedGrabFingerPose*
            meshFingerPosePtr{ nullptr };
        std::array<float, 5> capturedFingerSplayRadians{};
        const std::array<float, 5>*
            capturedFingerSplayRadiansPtr{ nullptr };
        bool spatialIndexBuilt{ false };
        bool commandedOpenDirectionsValid{ false };
    };


    bool TwoHandedGrip::capturePartGrip(
        bool isLeft,
        RE::NiNode* weaponNode,
        const WeaponInteractionDecision& decision,
        const WeaponCollision& weaponCollision,
        const WeaponProviderPartAuthority& providerPartAuthority,
        const bool firingGripProximityAuthorityEnabled,
        RE::NiTransform* const outCapturedHandWorld)
    {
        PartGripCapturePhases capture{
            *this,
            isLeft,
            weaponNode,
            decision,
            weaponCollision,
            providerPartAuthority,
            firingGripProximityAuthorityEnabled,
        };
        if (!capture.begin(outCapturedHandWorld)) {
            return false;
        }

        performance_profiler::ScopedTimer fingerPoseCaptureTimer(
            performance_profiler::Scope::
                EquippedWeaponFingerPoseCapture);

        // Authored support has first claim on a valid contact acquisition.
        if (capture.tryCaptureAuthoredSupportGrip()) {
            return true;
        }

        // Dynamic capture seats the palm before it samples any finger state.
        capture.captureDynamicMeshSeat();
        capture.captureRootFlattenedFingerReferences();

        // The frozen-mesh solve uses the pre-authority finger snapshot.
        capture.gatherEvidenceAndSolveFrozenFingerPose();

        // A local opposition witness can complete an incomplete direct solve.
        capture.applyOppositionPocketAndSelectFingerPose();

        // Exact joint locals override the numeric pose before publication.
        capture.prepareFingerLocalTransformOverride();
        return capture.publishDynamicGrip();
    }

    void TwoHandedGrip::lockPartGripToWeaponRoot(bool isLeft)
    {
        /*
         * Part-carry feeds its own solved weapon transform back as the next
         * frame's base, so grips must resolve exclusively through the captured
         * weapon-root frames while it is active. Following live part-node
         * chains lets any per-frame part animation integrate into a steady
         * carry drift and pulls the locked hand visuals apart (verified by
         * telemetry: rigid-weapon grip separation grew frame over frame).
         */
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.hasSourceFrames = false;
        grip.hasAttachmentWeaponLocal = false;
    }


    void TwoHandedGrip::releasePartGrip(bool isLeft, const char* reason, const bool smoothHandReturn)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (!grip.active) {
            return;
        }
        if (dynamicSupportAcquisitionMatches(isLeft, grip)) {
            clearDynamicSupportAcquisition(reason, true);
        }
        if (smoothHandReturn) {
            beginHandVisualReturn(isLeft, reason);
        }
        clearSupportGripPose(isLeft);
        grip = {};
        ROCK_LOG_INFO(Weapon, "TwoHandedGrip: part grip released hand={} reason={}", isLeft ? "left" : "right", reason ? reason : "unknown");
    }


    bool TwoHandedGrip::providerPartAuthorityStillCurrent(WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey)
    {
        if (!grip.providerPartAuthority.active) {
            return true;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.providerPartAuthority.weaponGenerationKey) {
            return false;
        }

        const auto query = buildProviderPartTargetQuery(
            grip.providerPartAuthority.weaponGenerationKey,
            grip.providerPartAuthority.bodyId,
            grip.providerPartAuthority.partKind,
            grip.providerPartAuthority.reloadRole,
            grip.providerPartAuthority.supportRole,
            grip.providerPartAuthority.socketRole,
            grip.providerPartAuthority.actionRole,
            grip.providerPartAuthority.sourceRoot,
            grip.providerPartAuthority.sourceName);

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        if (!::rock::provider::resolveWeaponPartTargetV1(query, resolution)) {
            return false;
        }
        return resolution.matched != 0 &&
               resolution.ownerToken == grip.providerPartAuthority.ownerToken &&
               resolution.groupId == grip.providerPartAuthority.groupId &&
               static_cast<std::uint32_t>(resolution.grabMode) == grip.providerPartAuthority.grabMode;
    }


    bool TwoHandedGrip::providerPartTargetNewlyMatchesGrip(const WeaponPartGrip& grip, std::uint64_t currentWeaponGenerationKey) const
    {
        /*
         * Upgrade twin of providerPartAuthorityStillCurrent: a support grip
         * captured WITHOUT provider authority whose own part NOW resolves to
         * a matched provider target — a consumer armed its whitelist while
         * the hand was already holding the part (PAPER_Toolkit: pulling the
         * trigger mid-hold switches an authority grab to attach-only). The
         * caller releases the grip; the still-held grab recaptures within a
         * couple of frames under the new resolution, through the same
         * re-resolve path the downgrade direction uses when a target
         * disappears mid-grip. The query is built from the grip's own
         * captured contact identity, not the live contact, so a flickering
         * contact cannot convert against the wrong part.
         */
        if (!grip.active || grip.providerPartAuthority.active) {
            return false;
        }
        if (currentWeaponGenerationKey == 0 || currentWeaponGenerationKey != grip.weaponGenerationKey) {
            return false;
        }

        const auto query = buildProviderPartTargetQuery(
            grip.weaponGenerationKey,
            grip.contactBodyId,
            static_cast<std::uint32_t>(grip.partKind),
            static_cast<std::uint32_t>(grip.reloadRole),
            static_cast<std::uint32_t>(grip.supportRole),
            static_cast<std::uint32_t>(grip.socketRole),
            static_cast<std::uint32_t>(grip.actionRole),
            reinterpret_cast<std::uintptr_t>(grip.attachmentRoot),
            grip.sourceName);

        ::rock::provider::RockProviderWeaponPartTargetResolutionV1 resolution{};
        return ::rock::provider::resolveWeaponPartTargetV1(query, resolution) && resolution.matched != 0;
    }


    bool TwoHandedGrip::tryRebindPartGripToCurrentGeneration(
        WeaponPartGrip& grip,
        std::uint64_t currentWeaponGenerationKey,
        const WeaponCollision& weaponCollision)
    {
        if (!grip.active) {
            return true;
        }

        WeaponCollisionProfileEvidenceDescriptor bestDescriptor{};
        RE::NiAVObject* bestSourceNode = nullptr;
        int bestScore = 0;
        float bestDistanceSquared = (std::numeric_limits<float>::max)();
        bool bestAmbiguous = false;
        const std::string_view capturedSourceName{ grip.sourceName.data() };
        const auto distanceSquaredToBounds = [&grip](const WeaponEvidenceBounds3& bounds) {
            if (!bounds.valid) {
                return (std::numeric_limits<float>::max)();
            }
            const auto axisDistance = [](float value, float minimum, float maximum) {
                if (value < minimum) {
                    return minimum - value;
                }
                if (value > maximum) {
                    return value - maximum;
                }
                return 0.0f;
            };
            const float dx = axisDistance(grip.gripLocal.x, bounds.min.x, bounds.max.x);
            const float dy = axisDistance(grip.gripLocal.y, bounds.min.y, bounds.max.y);
            const float dz = axisDistance(grip.gripLocal.z, bounds.min.z, bounds.max.z);
            return dx * dx + dy * dy + dz * dz;
        };
        const auto bodyCount = weaponCollision.getWeaponBodyCount();
        for (std::uint32_t i = 0; i < bodyCount; ++i) {
            const auto bodyId = weaponCollision.getWeaponBodyIdAtomic(i);
            WeaponCollisionProfileEvidenceDescriptor descriptor{};
            RE::NiAVObject* sourceNode = nullptr;
            if (!weaponCollision.tryGetProfileEvidenceDescriptorForBodyId(bodyId, descriptor, sourceNode) ||
                !descriptor.valid || descriptor.weaponGenerationKey != currentWeaponGenerationKey) {
                continue;
            }

            const bool sourcePointerMatches = sourceNode && sourceNode == grip.attachmentRoot;
            const bool sourceNameMatches = !capturedSourceName.empty() && descriptor.sourceName == capturedSourceName;
            if ((!sourcePointerMatches && !sourceNameMatches) || descriptor.semantic.partKind != grip.partKind) {
                continue;
            }
            if (grip.omodFormId != 0 && descriptor.omodFormId != grip.omodFormId) {
                continue;
            }
            if (grip.attachPointFormId != 0 && descriptor.semantic.attachPointFormId != grip.attachPointFormId) {
                continue;
            }

            const int score = sourcePointerMatches ? 2 : 1;
            const float distanceSquared = distanceSquaredToBounds(descriptor.localBoundsGame);
            constexpr float kDistanceTieEpsilon = 0.0001f;
            if (score > bestScore ||
                (score == bestScore && distanceSquared + kDistanceTieEpsilon < bestDistanceSquared)) {
                bestScore = score;
                bestDistanceSquared = distanceSquared;
                bestAmbiguous = false;
                bestDescriptor = descriptor;
                bestSourceNode = sourceNode;
            } else if (score == bestScore &&
                       (distanceSquared == bestDistanceSquared ||
                           (std::isfinite(distanceSquared) && std::isfinite(bestDistanceSquared) &&
                               std::fabs(distanceSquared - bestDistanceSquared) <= kDistanceTieEpsilon))) {
                bestAmbiguous = true;
            }
        }

        if (bestScore == 0 || bestAmbiguous) {
            ROCK_LOG_WARN(Weapon,
                "TwoHandedGrip: part grip rebind failed closed hand={} generation={:016X} source='{}' part={} omod={:08X} attachPoint={:08X} reason={}",
                (&grip == &_partGrips[0]) ? "left" : "right",
                currentWeaponGenerationKey,
                capturedSourceName,
                static_cast<std::uint32_t>(grip.partKind),
                grip.omodFormId,
                grip.attachPointFormId,
                bestScore == 0 ? "missing" : "ambiguous");
            return false;
        }

        grip.weaponGenerationKey = currentWeaponGenerationKey;
        // The support input-to-target relation belongs to the old collision
        // generation. The game-thread reconciliation step recaptures it from
        // current transforms only after the new generation is eligible.
        grip.supportInputBaseline = {};
        grip.contactBodyId = bestDescriptor.bodyId;
        grip.attachmentRoot = grip.authoredSupportGrip ?
            _activeWeaponNode :
            (bestSourceNode ? bestSourceNode : grip.attachmentRoot);
        grip.partKind = bestDescriptor.semantic.partKind;
        grip.reloadRole = bestDescriptor.semantic.reloadRole;
        grip.supportRole = bestDescriptor.semantic.supportGripRole;
        grip.socketRole = bestDescriptor.semantic.socketRole;
        grip.actionRole = bestDescriptor.semantic.actionRole;
        grip.omodFormId = bestDescriptor.omodFormId;
        grip.attachPointFormId = bestDescriptor.semantic.attachPointFormId;
        grip.classificationSource = bestDescriptor.semantic.classificationSource;
        const auto copyLength = (std::min)(bestDescriptor.sourceName.size(), grip.sourceName.size() - 1);
        std::memcpy(grip.sourceName.data(), bestDescriptor.sourceName.data(), copyLength);
        grip.sourceName[copyLength] = '\0';

        if (grip.providerPartAuthority.active) {
            grip.providerPartAuthority.weaponGenerationKey = currentWeaponGenerationKey;
            grip.providerPartAuthority.bodyId = bestDescriptor.bodyId;
            grip.providerPartAuthority.sourceRoot = reinterpret_cast<std::uintptr_t>(bestSourceNode);
            grip.providerPartAuthority.partKind = static_cast<std::uint32_t>(grip.partKind);
            grip.providerPartAuthority.reloadRole = static_cast<std::uint32_t>(grip.reloadRole);
            grip.providerPartAuthority.supportRole = static_cast<std::uint32_t>(grip.supportRole);
            grip.providerPartAuthority.socketRole = static_cast<std::uint32_t>(grip.socketRole);
            grip.providerPartAuthority.actionRole = static_cast<std::uint32_t>(grip.actionRole);
            std::memcpy(
                grip.providerPartAuthority.sourceName.data(),
                grip.sourceName.data(),
                grip.providerPartAuthority.sourceName.size());
        }
        return true;
    }


    bool TwoHandedGrip::reconcileCollisionGeneration(
        RE::NiNode* currentWeaponNode,
        std::uint64_t currentWeaponGenerationKey,
        std::uint64_t currentEquippedWeaponOwnershipKey,
        const WeaponCollision& weaponCollision)
    {
        if (!equipped_weapon_manual_ownership_policy::canPreserveManualOwnership(
                _activeEquippedWeaponOwnershipKey,
                currentEquippedWeaponOwnershipKey,
                currentWeaponGenerationKey,
                _state != TwoHandedState::PrimaryOnly)) {
            return false;
        }
        if (currentWeaponGenerationKey == 0) {
            // PrimaryOnly rides the native firing-hand attach and can retain
            // ownership while the complete collider set is still building.
            _activeWeaponNode = currentWeaponNode;
            _activeWeaponGenerationKey = 0;
            _weaponNodeLocalBaseline = currentWeaponNode->local;
            _hasWeaponNodeLocalBaseline = true;
            return true;
        }

        const bool generationChanged = _activeWeaponGenerationKey != currentWeaponGenerationKey;
        const bool weaponRootChanged = _activeWeaponNode != currentWeaponNode;
        if (generationChanged || weaponRootChanged) {
            const auto previousGeneration = _activeWeaponGenerationKey;
            _activeWeaponNode = currentWeaponNode;
            _activeWeaponGenerationKey = currentWeaponGenerationKey;
            if (weaponRootChanged) {
                _weaponNodeLocalBaseline = currentWeaponNode->local;
                _hasWeaponNodeLocalBaseline = true;
            }
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: preserving manual ownership across collision rebuild oldGeneration={:016X} newGeneration={:016X} ownership={:016X} rootChanged={}",
                previousGeneration,
                currentWeaponGenerationKey,
                currentEquippedWeaponOwnershipKey,
                weaponRootChanged ? "yes" : "no");
        }

        if (generationChanged || weaponRootChanged) {
            for (auto& grip : _partGrips) {
                if (grip.active && !tryRebindPartGripToCurrentGeneration(grip, currentWeaponGenerationKey, weaponCollision)) {
                    return false;
                }
            }
        }
        return true;
    }


    void TwoHandedGrip::setSupportGripPose(
        bool isLeft,
        const grab_finger_pose_runtime::SolvedGrabFingerPose* meshFingerPose,
        const std::array<float, 5>* capturedSplayRadians,
        const SupportGripPoseFallback fallback)
    {
        WeaponPartGrip& grip = partGrip(isLeft);
        if (meshFingerPose && meshFingerPose->solved) {
            grip.fingerPose = meshFingerPose->hasJointValues ? meshFingerPose->jointValues : grab_finger_pose_math::expandFingerCurlsToJointValues(meshFingerPose->values);
            grip.fingerSplayRadians = capturedSplayRadians ? *capturedSplayRadians : std::array<float, 5>{};
            grip.hasFingerSplay = capturedSplayRadians != nullptr;
            grip.hasFingerPose = true;
            return;
        }

        float fallbackValue = 0.0f;
        if (fallback == SupportGripPoseFallback::SelectedClose) {
            const float fallbackMin =
                std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
            const float configuredFallback =
                std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
            fallbackValue = std::clamp(
                configuredFallback,
                fallbackMin,
                1.0f);
        }
        const std::array<float, 5> fallbackCurls{
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
            fallbackValue,
        };
        grip.fingerPose = grab_finger_pose_math::expandFingerCurlsToJointValues(fallbackCurls);
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = true;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;
        if (fallback == SupportGripPoseFallback::FullyClosed) {
            ROCK_LOG_INFO(Weapon,
                "TwoHandedGrip: using whole-hand-closed finger fallback hand={} value={:.2f}",
                isLeft ? "left" : "right",
                fallbackValue);
        } else {
            ROCK_LOG_DEBUG(Weapon,
                "TwoHandedGrip: using selected-close finger fallback hand={} value={:.2f}",
                isLeft ? "left" : "right",
                fallbackValue);
        }
    }


    void TwoHandedGrip::clearSupportGripPose(bool isLeft)
    {
        _hasLastPublishedHandWorld[isLeft ? 0u : 1u] = false;
        WeaponPartGrip& grip = partGrip(isLeft);
        grip.fingerPose = {};
        grip.fingerSplayRadians = {};
        grip.hasFingerPose = false;
        grip.hasFingerSplay = false;
        grip.fingerLocalTransforms = {};
        grip.fingerLocalTransformMask = 0;
        grip.hasFingerLocalTransforms = false;

        (void)frik_visual_authority::clearHandPose(SUPPORT_GRIP_TAG, handFromBool(isLeft));
        deferOrClearHandAuthorityRole(
            scope_safe_hand_frame_math::HandAuthorityRole::SupportGrip,
            isLeft);
    }

}

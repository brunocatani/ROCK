#include "physics-interaction/hand/HandGrabContactEvidence.h"

#include "physics-interaction/hand/HandGrabMath.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/native/query/PhysicsShapeCast.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/TransformMath.h"
#include "RockConfig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace rock::hand_grab_detail
{
    bool nodeIsOrDescendsFrom(const RE::NiAVObject* root, const RE::NiAVObject* node)
    {
        if (!root || !node) {
            return false;
        }
    
        for (auto* current = node; current; current = current->parent) {
            if (current == root) {
                return true;
            }
        }
        return false;
    }
    
    bool acceptsSelectedMultibodyOwnerlessVisualMesh(const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        RE::NiAVObject* surfaceOwnerNode,
        const object_physics_body_set::ObjectPhysicsBodyRecord* surfaceOwnerRecord)
    {
        /*
         * Multipart refs can expose visible geometry and hknp collision
         * owners as sibling nodes under the same selected reference. When
         * the visible mesh has no accepted collision owner record, the
         * selected body remains the acquisition authority; a concrete
         * mismatched owner record still fails closed.
         */
        return selection.refr == bodySet.rootRef &&
               bodySet.acceptedCount() > 1 &&
               resolvedBodyId != object_physics_body_set::INVALID_BODY_ID &&
               resolvedBodyId == selection.bodyId.value &&
               bodySet.containsAcceptedBody(selection.bodyId.value) &&
               surfaceOwnerNode &&
               !surfaceOwnerRecord &&
               nodeIsOrDescendsFrom(bodySet.rootNode, surfaceOwnerNode);
    }
    
    void rememberRejectedContactPatchBody(RuntimeGrabContactPatch& result, std::uint32_t bodyId)
    {
        if (bodyId == INVALID_BODY_ID || bodyId == 0x7FFF'FFFF) {
            return;
        }
        for (std::uint32_t index = 0; index < result.rejectedBodyIdCount; ++index) {
            if (result.rejectedBodyIds[index] == bodyId) {
                return;
            }
        }
        if (result.rejectedBodyIdCount < result.rejectedBodyIds.size()) {
            result.rejectedBodyIds[result.rejectedBodyIdCount++] = bodyId;
        }
    }
    
    grab_pinch_pocket_policy::Config currentPinchPocketConfig()
    {
        return grab_pinch_pocket_policy::sanitizeConfig(grab_pinch_pocket_policy::Config{
            .enabled = g_rockConfig.rockGrabPinchPocketEnabled,
            .compactMaxExtentGameUnits = g_rockConfig.rockGrabPinchCompactMaxExtentGameUnits,
            .thinRodMaxLengthGameUnits = g_rockConfig.rockGrabPinchThinRodMaxLengthGameUnits,
            .thinRodMaxCrossSectionGameUnits = g_rockConfig.rockGrabPinchThinRodMaxCrossSectionGameUnits,
            .maxPocketDistanceGameUnits = g_rockConfig.rockGrabPinchMaxPocketDistanceGameUnits,
            .minFingerGapGameUnits = g_rockConfig.rockGrabPinchMinFingerGapGameUnits,
            .maxFingerGapGameUnits = g_rockConfig.rockGrabPinchMaxFingerGapGameUnits,
            .thumbIndexMaxOpenValue = g_rockConfig.rockGrabPinchThumbIndexMaxOpenValue,
            .otherFingerCurlValue = g_rockConfig.rockGrabPinchOtherFingerCurlValue,
            .surfaceInsetGameUnits = g_rockConfig.rockGrabPinchSurfaceInsetGameUnits,
            .detectionDirectionHandspace = g_rockConfig.rockGrabPinchDetectionDirectionHandspace,
            .detectionAxisBlend = g_rockConfig.rockGrabPinchDetectionAxisBlend,
        });
    }
    
    RE::NiPoint3 pinchPadPointFromSnapshot(const root_flattened_finger_skeleton_runtime::FingerChain& chain)
    {
        return chain.points[2];
    }
    
    RuntimePinchPocketCandidate buildRuntimePinchPocketCandidate(
        const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const RE::NiTransform& objectWorldTransform,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
        const std::vector<GrabLocalTriangle>& localMeshTriangles,
        const RE::NiPoint3& currentObjectPointWorld,
        const RE::NiTransform& handWorldTransform,
        bool isLeft,
        bool closeGrab,
        bool handPocketOnlyGrab,
        bool authoredGrabNode,
        bool looseWeaponGrab)
    {
        RuntimePinchPocketCandidate candidate{};
        const auto config = currentPinchPocketConfig();
        const float objectScale =
            std::isfinite(objectWorldTransform.scale) && objectWorldTransform.scale > 0.0f ? objectWorldTransform.scale : 1.0f;
        candidate.meshExtents = grab_pinch_pocket_policy::computeMeshExtents(localMeshTriangles, objectScale);
    
        root_flattened_finger_skeleton_runtime::Snapshot fingerSnapshot{};
        const bool hasFingerSnapshot =
            root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(isLeft, fingerSnapshot) &&
            fingerSnapshot.valid &&
            fingerSnapshot.fingers[0].valid &&
            fingerSnapshot.fingers[1].valid;
    
        bool hasPinchSurface = false;
        bool ownerMatchesResolvedBody = false;
        if (hasFingerSnapshot && !surfaceTriangles.empty()) {
            candidate.thumbPadWorld = pinchPadPointFromSnapshot(fingerSnapshot.fingers[0]);
            candidate.indexPadWorld = pinchPadPointFromSnapshot(fingerSnapshot.fingers[1]);
            candidate.thumbIndexGapGameUnits =
                grab_pinch_pocket_policy::distance(candidate.thumbPadWorld, candidate.indexPadWorld);
            candidate.pinchAxisWorld =
                grab_pinch_pocket_policy::normalizeOrFallback(candidate.indexPadWorld - candidate.thumbPadWorld, RE::NiPoint3{ 1.0f, 0.0f, 0.0f });
            candidate.pinchPocketWorld =
                grab_pinch_pocket_policy::closestPointOnSegment(candidate.thumbPadWorld, candidate.indexPadWorld, currentObjectPointWorld);
            const RE::NiPoint3 configuredDetectionWorld =
                transformHandspaceDirection(handWorldTransform, config.detectionDirectionHandspace, isLeft);
            const RE::NiPoint3 configuredDetectionNormal =
                grab_pinch_pocket_policy::normalizeOrFallback(configuredDetectionWorld, candidate.pinchAxisWorld);
            candidate.pinchDetectionDirectionWorld =
                grab_pinch_pocket_policy::normalizeOrFallback(candidate.pinchAxisWorld * config.detectionAxisBlend +
                                                                  configuredDetectionNormal * (1.0f - config.detectionAxisBlend),
                    candidate.pinchAxisWorld);
    
            GrabSurfaceHit surfaceHit{};
            hasPinchSurface = findClosestGrabSurfaceHitToPointPositionOnly(
                surfaceTriangles,
                candidate.pinchPocketWorld,
                candidate.pinchDetectionDirectionWorld,
                config.maxPocketDistanceGameUnits,
                surfaceHit);
            if (hasPinchSurface) {
                candidate.surfaceHit = surfaceHit;
                candidate.pocketToSurfaceDistanceGameUnits =
                    grab_pinch_pocket_policy::distance(candidate.pinchPocketWorld, surfaceHit.position);
                if (surfaceHit.sourceNode) {
                    const auto* ownerRecord = bodySet.findAcceptedRecordByOwnerNode(surfaceHit.sourceNode);
                    ownerMatchesResolvedBody =
                        (ownerRecord && ownerRecord->bodyId == resolvedBodyId) ||
                        acceptsSelectedMultibodyOwnerlessVisualMesh(selection,
                            bodySet,
                            resolvedBodyId,
                            surfaceHit.sourceNode,
                            ownerRecord);
                }
            }
        }
    
        candidate.decision = grab_pinch_pocket_policy::evaluateObject(grab_pinch_pocket_policy::ObjectDecisionInput{
            .config = config,
            .mesh = candidate.meshExtents,
            .closeGrab = closeGrab,
            .handPocketOnlyGrab = handPocketOnlyGrab,
            .authoredGrabNode = authoredGrabNode,
            .looseWeaponGrab = looseWeaponGrab,
            .ownerMatchesResolvedBody = ownerMatchesResolvedBody,
            .hasFingerSnapshot = hasFingerSnapshot,
            .hasPinchSurface = hasPinchSurface,
            .multipleAcceptedBodies = bodySet.acceptedCount() > 1,
            .thumbIndexGapGameUnits = candidate.thumbIndexGapGameUnits,
            .pocketToSurfaceDistanceGameUnits = candidate.pocketToSurfaceDistanceGameUnits,
        });
        candidate.valid = candidate.decision.accept;
        return candidate;
    }
    
    RuntimeMultiFingerGripContact buildRuntimeMultiFingerGripContact(RE::hknpWorld* world,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const RE::NiTransform& objectWorldTransform,
        const hand_semantic_contact_state::SemanticContactCollection& semanticContacts,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
        const Hand* hand,
        bool includeLiveColliderProbes)
    {
        RuntimeMultiFingerGripContact result{};
        result.reason = "disabled";
        if (!g_rockConfig.rockGrabMultiFingerContactValidationEnabled) {
            return result;
        }
        if (!world || resolvedBodyId == INVALID_BODY_ID) {
            result.reason = "invalidWorldOrBody";
            result.gripSet.reason = result.reason;
            return result;
        }
        if (surfaceTriangles.empty()) {
            result.reason = "noSurfaceTriangles";
            result.gripSet.reason = result.reason;
            return result;
        }
    
        std::vector<grab_multi_finger_contact_math::FingerContactPatch<RE::NiPoint3>> patches;
        patches.reserve(semanticContacts.count + hand_collider_semantics::kHandColliderBodyCountPerHand);
        std::array<bool, grab_multi_finger_contact_math::kMaxFingerGroups> semanticGroups{};
        std::array<bool, grab_multi_finger_contact_math::kMaxFingerGroups> liveProbeGroups{};
    
        auto appendPatchFromHandBody = [&](std::uint32_t handBodyId,
                                           hand_collider_semantics::HandColliderRole role,
                                           hand_collider_semantics::HandFinger finger,
                                           hand_collider_semantics::HandFingerSegment segment,
                                           std::uint32_t framesSinceContact,
                                           float sourceQualityScale,
                                           bool liveProbeSource,
                                           const hand_semantic_contact_state::SemanticContactVector* semanticContactPointGame = nullptr) {
            if (handBodyId == hand_semantic_contact_state::kInvalidBodyId) {
                return;
            }
            if (finger == hand_collider_semantics::HandFinger::None) {
                finger = hand_collider_semantics::fingerForRole(role);
            }
            if (finger == hand_collider_semantics::HandFinger::None) {
                return;
            }
    
            ++result.candidateContactCount;
            if (liveProbeSource) {
                ++result.liveProbeCandidateContactCount;
            } else {
                ++result.semanticCandidateContactCount;
            }
    
            const bool hasSemanticContactPoint =
                semanticContactPointGame && hand_semantic_contact_state::isFiniteVector(*semanticContactPointGame);
            RE::NiTransform handContactWorld{};
            const bool hasLiveHandBodyPoint =
                tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ handBodyId }, handContactWorld);
            if (!hasSemanticContactPoint && !hasLiveHandBodyPoint) {
                return;
            }
            const RE::NiPoint3 evidencePointWorld = hasSemanticContactPoint ?
                RE::NiPoint3{ semanticContactPointGame->x, semanticContactPointGame->y, semanticContactPointGame->z } :
                handContactWorld.translate;
    
            RE::NiPoint3 directionToObject = normalizeOrZero(objectWorldTransform.translate - evidencePointWorld);
            if (directionToObject.x == 0.0f && directionToObject.y == 0.0f && directionToObject.z == 0.0f) {
                directionToObject = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
            }
    
            GrabSurfaceHit hit{};
            if (!findClosestGrabSurfaceHit(surfaceTriangles,
                    evidencePointWorld,
                    directionToObject,
                    g_rockConfig.rockGrabLateralWeight,
                    g_rockConfig.rockGrabDirectionalWeight,
                    hit,
                    g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits)) {
                return;
            }
    
            const auto* ownerRecord = hit.sourceNode ? bodySet.findAcceptedRecordByOwnerNode(hit.sourceNode) : nullptr;
            if (!ownerRecord || ownerRecord->bodyId != resolvedBodyId) {
                ++result.rejectedOwnerCount;
                return;
            }
    
            const float contactDistance = pointDistanceGameUnits(evidencePointWorld, hit.position);
            const float maxContactDistance = (std::max)(0.0f, g_rockConfig.rockGrabFingerContactMeshSnapMaxDistanceGameUnits);
            if (maxContactDistance > 0.0f && contactDistance > maxContactDistance) {
                ++result.rejectedDistanceCount;
                return;
            }
    
            ++result.meshHitCount;
            if (liveProbeSource) {
                ++result.liveProbeMeshHitCount;
            } else {
                ++result.semanticMeshHitCount;
            }
    
            grab_multi_finger_contact_math::FingerContactPatch<RE::NiPoint3> patch{};
            patch.valid = true;
            patch.finger = finger;
            patch.segment = segment;
            patch.role = role;
            patch.handBodyId = handBodyId;
            patch.objectBodyId = resolvedBodyId;
            patch.handPointWorld = evidencePointWorld;
            patch.objectPointWorld = hit.position;
            patch.normalWorld = hit.normal;
            patch.quality = sourceQualityScale / (1.0f + contactDistance);
            patch.framesSinceContact = framesSinceContact;
            patches.push_back(patch);
    
            const int fingerIndex = grab_multi_finger_contact_math::fingerIndex(finger);
            if (fingerIndex >= 0 && static_cast<std::size_t>(fingerIndex) < result.groupHits.size()) {
                result.groupHits[static_cast<std::size_t>(fingerIndex)] = hit;
                if (liveProbeSource) {
                    liveProbeGroups[static_cast<std::size_t>(fingerIndex)] = true;
                } else {
                    semanticGroups[static_cast<std::size_t>(fingerIndex)] = true;
                }
            }
        };
    
        for (std::size_t i = 0; i < semanticContacts.count && i < semanticContacts.records.size(); ++i) {
            const auto& contact = semanticContacts.records[i];
            if (!contact.valid || contact.otherBodyId != resolvedBodyId || contact.handBodyId == hand_semantic_contact_state::kInvalidBodyId) {
                continue;
            }
            appendPatchFromHandBody(contact.handBodyId,
                contact.role,
                contact.finger,
                contact.segment,
                contact.framesSinceContact,
                1.0f,
                false,
                hand_semantic_contact_state::hasUsableContactPoint(contact) ? &contact.contactPointGame : nullptr);
        }
    
        if (includeLiveColliderProbes && hand) {
            const std::uint32_t colliderCount = hand->getHandColliderBodyCount();
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                const std::uint32_t handBodyId = hand->getHandColliderBodyIdAtomic(i);
                HandColliderBodyMetadata metadata{};
                if (!hand->tryGetHandColliderMetadata(handBodyId, metadata) || !metadata.valid || metadata.primaryPalmAnchor) {
                    continue;
                }
                const auto finger = metadata.finger != hand_collider_semantics::HandFinger::None ? metadata.finger : hand_collider_semantics::fingerForRole(metadata.role);
                if (finger == hand_collider_semantics::HandFinger::None) {
                    continue;
                }
                appendPatchFromHandBody(metadata.bodyId,
                    metadata.role,
                    finger,
                    metadata.segment,
                    1,
                    0.75f,
                    true);
            }
        }
    
        for (bool group : semanticGroups) {
            if (group) {
                ++result.semanticGroupCount;
            }
        }
        for (bool group : liveProbeGroups) {
            if (group) {
                ++result.liveProbeGroupCount;
            }
        }
    
        grab_multi_finger_contact_math::GripContactSetOptions options{};
        options.enabled = true;
        options.targetBodyId = resolvedBodyId;
        options.minimumFingerGroups = g_rockConfig.rockGrabMinFingerContactGroups;
        options.maxContactAgeFrames = static_cast<std::uint32_t>((std::max)(0, g_rockConfig.rockGrabOppositionContactMaxAgeFrames));
        options.minimumSpreadGameUnits = g_rockConfig.rockGrabMinFingerContactSpreadGameUnits;
        result.gripSet = grab_multi_finger_contact_math::buildGripContactSet(patches, options);
        result.reason = result.gripSet.reason;
        return result;
    }
    
    RuntimeGrabContactPatch buildRuntimeGrabContactPatch(RE::hknpWorld* world,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const SelectedObject& selection,
        const RE::NiPoint3& grabPivotAWorld,
        const RE::NiPoint3& seatedPivotAnchorWorld,
        bool hasSeatedPivotAnchor,
        const RE::NiPoint3& palmNormalWorld,
        const RE::NiPoint3& palmTangentWorld,
        const RE::NiPoint3& palmBitangentWorld,
        float objectLeverEstimateGameUnits,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles)
    {
        RuntimeGrabContactPatch result{};
        if (!world || resolvedBodyId == INVALID_BODY_ID) {
            result.patch.fallbackReason = "invalidWorldOrBody";
            return result;
        }
    
        const RE::NiPoint3 palmNormal = normalizeOrZero(palmNormalWorld);
        const RE::NiPoint3 palmTangent = normalizeOrZero(palmTangentWorld);
        RE::NiPoint3 palmBitangent = normalizeOrZero(palmBitangentWorld);
        if (palmBitangent.x == 0.0f && palmBitangent.y == 0.0f && palmBitangent.z == 0.0f) {
            palmBitangent = normalizeOrZero(crossProduct(palmNormal, palmTangent));
        }
        if ((palmNormal.x == 0.0f && palmNormal.y == 0.0f && palmNormal.z == 0.0f) ||
            (palmTangent.x == 0.0f && palmTangent.y == 0.0f && palmTangent.z == 0.0f)) {
            result.patch.fallbackReason = "invalidPalmFrame";
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
        result.probeScale = probeGeometry.scale;
        result.probeScaleReason = probeGeometry.reason;
        std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> offsets{};
        const auto probePatternCount =
            grab_contact_patch_math::buildContactPatchProbeOffsets(offsets, palmTangent, palmBitangent, spacing);
        const int probeCount = (std::min)(
            std::clamp(g_rockConfig.rockGrabContactPatchProbeCount, 1, static_cast<int>(kMaxGrabContactPatchSamples)),
            static_cast<int>(probePatternCount));
    
        const float configuredNearDistance =
            g_rockConfig.rockNearCastDistanceGameUnits > 0.0f ? g_rockConfig.rockNearCastDistanceGameUnits : g_rockConfig.rockNearDetectionRange;
        const float selectionDistance = selection.hasHitPoint ? pointDistanceGameUnits(grabPivotAWorld, selection.hitPointWorld) : 0.0f;
        const float castDistance =
            (std::max)(10.0f, (std::max)(configuredNearDistance, selectionDistance + radius * 4.0f + spacing));
    
        std::vector<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>> fitSamples;
        fitSamples.reserve(static_cast<std::size_t>(probeCount));
        for (int probe = 0; probe < probeCount; ++probe) {
            const RE::NiPoint3 probeOrigin = grabPivotAWorld + offsets[probe];
            const RE::NiPoint3 start = probeOrigin - palmNormal * radius;
    
            RE::hknpAllHitsCollector collector;
            physics_shape_cast::SphereCastDiagnostics diagnostics;
            if (!physics_shape_cast::castSelectionSphere(
                    world,
                    physics_shape_cast::SphereCastInput{ .startGame = start,
                        .directionGame = palmNormal,
                        .distanceGame = castDistance,
                        .radiusGame = radius,
                        .collisionFilterInfo = g_rockConfig.rockSelectionShapeCastFilterInfo },
                    collector,
                    &diagnostics)) {
                continue;
            }
    
            result.castHitCount += diagnostics.hitCount;
            const auto* hits = collector.hits._data;
            const int hitCount = collector.hits._size;
            float bestFraction = (std::numeric_limits<float>::max)();
            grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3> bestSample{};
            bool foundProbeHit = false;
            bool bestProbeWasMeshRecovered = false;
            for (int hitIndex = 0; hitIndex < hitCount; ++hitIndex) {
                const auto& hit = hits[hitIndex];
                const RE::NiPoint3 normal = normalizeOrZero(RE::NiPoint3{ hit.normal.x, hit.normal.y, hit.normal.z });
                if (normal.x == 0.0f && normal.y == 0.0f && normal.z == 0.0f) {
                    ++result.rejectedInvalidNormals;
                    continue;
                }
    
                const RE::NiPoint3 hitPoint = hkVectorToNiPoint(hit.position);
                const bool exactBodyHit = hit.hitBodyInfo.m_bodyId.value == resolvedBodyId;
                GrabSurfaceHit recoveredMeshHit{};
                bool meshRecoveredHit = false;
                /*
                 * Contact patch authority must follow the same rendered-object surface
                 * contract as mesh grab. Exact hknp body id is still the fast path, but
                 * FO4VR near-palm casts can first touch hand/proxy/world collision while
                 * still lying on the selected object's visible surface. In that case the
                 * mesh snap validates the sample against the resolved object body.
                 */
                if (!exactBodyHit && !surfaceTriangles.empty() && g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits > 0.0f) {
                    if (findClosestGrabSurfaceHitToPoint(surfaceTriangles,
                            hitPoint,
                            normal,
                            g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits,
                            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees,
                            recoveredMeshHit)) {
                        const auto* recoveredOwnerRecord =
                            recoveredMeshHit.sourceNode ? bodySet.findAcceptedRecordByOwnerNode(recoveredMeshHit.sourceNode) : nullptr;
                        meshRecoveredHit =
                            (recoveredOwnerRecord && recoveredOwnerRecord->bodyId == resolvedBodyId) ||
                            acceptsSelectedMultibodyOwnerlessVisualMesh(selection,
                                bodySet,
                                resolvedBodyId,
                                recoveredMeshHit.sourceNode,
                                recoveredOwnerRecord);
                    }
                }
    
                if (!exactBodyHit && !meshRecoveredHit) {
                    ++result.rejectedBodyHits;
                    rememberRejectedContactPatchBody(result, hit.hitBodyInfo.m_bodyId.value);
                    continue;
                }
    
                const float fraction = hit.fraction.storage;
                if (fraction >= bestFraction) {
                    continue;
                }
    
                bestFraction = fraction;
                bestSample.bodyId = exactBodyHit ? hit.hitBodyInfo.m_bodyId.value : resolvedBodyId;
                bestSample.point = meshRecoveredHit ? recoveredMeshHit.position : hitPoint;
                bestSample.normal = meshRecoveredHit ? normalizeOrZero(recoveredMeshHit.normal) : normal;
                bestSample.fraction = fraction;
                bestSample.accepted = true;
                bestSample.rejectionReason = meshRecoveredHit ? "meshRecoveredBody" : "none";
                foundProbeHit = true;
                bestProbeWasMeshRecovered = meshRecoveredHit;
            }
    
            if (foundProbeHit) {
                if (bestProbeWasMeshRecovered) {
                    ++result.meshRecoveredSamples;
                } else {
                    ++result.exactBodySamples;
                }
                fitSamples.push_back(bestSample);
                if (result.sampleCount < result.samples.size()) {
                    result.samples[result.sampleCount++] = bestSample;
                }
            }
        }
    
        const RE::NiPoint3 patchAnchor = hasSeatedPivotAnchor ? seatedPivotAnchorWorld : grabPivotAWorld;
        const float anchorDepthLimit = (std::max)(1.0f, radius + spacing * 0.50f);
        const float clusterDepthLimit = (std::max)(0.75f, radius + spacing * 0.35f);
        const float anchorLateralLimit = (std::max)(
            radius * 2.0f,
            (std::max)(
                spacing * 2.0f + radius,
                g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits + spacing));
        const auto surfaceCluster = grab_contact_patch_math::filterContactPatchSameSurfaceCluster(fitSamples,
            patchAnchor,
            palmNormal,
            anchorDepthLimit,
            clusterDepthLimit,
            anchorLateralLimit,
            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
        result.rawAcceptedSampleCount = static_cast<std::uint32_t>((std::min)(surfaceCluster.rawAcceptedCount,
            static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
        result.clusterRejectedSampleCount = static_cast<std::uint32_t>((std::min)(surfaceCluster.clusterRejectedCount,
            static_cast<std::size_t>((std::numeric_limits<std::uint32_t>::max)())));
        result.clusterDepthSpreadGameUnits = surfaceCluster.maxDepthSpreadGameUnits;
        result.clusterMaxLateralGameUnits = surfaceCluster.maxLateralDistanceGameUnits;
        result.clusterReason = surfaceCluster.reason;
        if (!surfaceCluster.valid) {
            result.samples = {};
            result.sampleCount = 0;
            result.patch.fallbackReason = surfaceCluster.reason ? surfaceCluster.reason : "contactPatchClusterFailed";
            result.pointMode = "contactPatchClusterFailed";
            return result;
        }
    
        fitSamples = surfaceCluster.samples;
        result.samples = {};
        result.sampleCount = 0;
        for (const auto& sample : fitSamples) {
            if (result.sampleCount < result.samples.size()) {
                result.samples[result.sampleCount++] = sample;
            }
        }
    
        result.patch = grab_contact_patch_math::fitContactPatch(fitSamples,
            patchAnchor,
            palmNormal,
            palmTangent,
            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
        if (!result.patch.valid) {
            result.pointMode = "contactPatchFailed";
            return result;
        }
        result.normalTrusted = result.patch.orientationReliable;
    
        result.pointMode = "contactPatch";
        if (!surfaceTriangles.empty() && g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits > 0.0f) {
            GrabSurfaceHit snapHit{};
            if (findClosestGrabSurfaceHitToPoint(surfaceTriangles,
                    result.patch.contactPoint,
                    result.patch.normal,
                    g_rockConfig.rockGrabContactPatchMeshSnapMaxDistanceGameUnits,
                    g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees,
                    snapHit)) {
                bool ownerMatches = true;
                if (snapHit.sourceNode) {
                    const auto* snapOwnerRecord = bodySet.findAcceptedRecordByOwnerNode(snapHit.sourceNode);
                    ownerMatches =
                        (snapOwnerRecord && snapOwnerRecord->bodyId == resolvedBodyId) ||
                        acceptsSelectedMultibodyOwnerlessVisualMesh(selection,
                            bodySet,
                            resolvedBodyId,
                            snapHit.sourceNode,
                            snapOwnerRecord);
                }
    
                if (ownerMatches) {
                    const float snapDelta = pointDistanceGameUnits(result.patch.contactPoint, snapHit.position);
                    result.patch.meshSnapDeltaGameUnits = snapDelta;
                    result.patch.contactPoint = snapHit.position;
                    result.patch.normal = grab_contact_patch_math::orientNormalTowardPalm(snapHit.normal, palmNormal);
                    result.patch.tangent = grab_contact_patch_math::normalizeOrZero(grab_contact_patch_math::projectOntoPlane(result.patch.tangent, result.patch.normal));
                    if (grab_contact_patch_math::lengthSquared(result.patch.tangent) <= 0.0f) {
                        result.patch.tangent = grab_contact_patch_math::stablePerpendicular(result.patch.normal);
                    }
                    result.patch.bitangent = grab_contact_patch_math::normalizeOrZero(grab_contact_patch_math::cross(result.patch.normal, result.patch.tangent));
                    result.meshSnapped = true;
                    result.meshSnapHit = snapHit;
                    result.pointMode = "contactPatchMeshSnap";
                }
            }
        }
    
        const bool normalMatchesSelection = grab_contact_patch_math::contactPatchNormalMatchesSelection(result.patch,
                selection.hitNormalWorld,
                selection.hasHitNormal,
                palmNormal,
                g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
        if (!normalMatchesSelection) {
            /*
             * A failed normal comparison means the patch normal cannot own
             * pose/orientation. It does not mean the palm probe's point is
             * useless. Keeping the point alive prevents the mesh ray from
             * replacing a good near-contact position with a farther visual
             * point and rotating the object around the wrong pivot.
             */
            result.patch.orientationReliable = false;
            result.patch.fallbackReason = "selectionNormalMismatch";
            result.normalTrusted = false;
            result.positionOnly = true;
            result.pointMode = result.meshSnapped ? "contactPatchMeshSnapPositionOnly" : "contactPatchPositionOnly";
        } else {
            result.normalTrusted = result.patch.orientationReliable;
        }
    
        result.pivotDecision = grab_contact_patch_math::chooseContactPatchPivotPoint(result.patch,
            fitSamples,
            selection.hitPointWorld,
            selection.hasHitPoint,
            result.meshSnapped ? result.meshSnapHit.position : RE::NiPoint3{},
            result.meshSnapped,
            g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
            g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
        if (!result.pivotDecision.valid) {
            result.patch.valid = false;
            result.patch.orientationReliable = false;
            result.patch.confidence = 0.0f;
            result.patch.fallbackReason = result.pivotDecision.reason ? result.pivotDecision.reason : "noValidatedPatchPivot";
            result.pointMode = "contactPatchNoValidatedPivot";
            return result;
        }
    
        switch (result.pivotDecision.source) {
        case grab_contact_patch_math::GrabContactPatchPivotSource::MeshSnap:
            result.pointMode = "contactPatchMeshSnap";
            break;
        case grab_contact_patch_math::GrabContactPatchPivotSource::PatchSample:
            result.pointMode = "contactPatchSamplePivot";
            break;
        case grab_contact_patch_math::GrabContactPatchPivotSource::SelectedHit:
            result.pointMode = "contactPatchSelectedHitPivot";
            break;
        default:
            result.pointMode = "contactPatch";
            break;
        }
    
        return result;
    }
}


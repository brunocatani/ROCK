#include "Hand.h"

#include "HavokOffsets.h"

#include "BodyCollisionControl.h"
#include "ActiveObjectPrepPolicy.h"
#include "CollisionSuppressionRegistry.h"
#include "DebugPivotMath.h"
#include "GrabConstraint.h"
#include "GrabConstraintMath.h"
#include "GrabContactPatchMath.h"
#include "GrabContactEvidencePolicy.h"
#include "GrabContactSourcePolicy.h"
#include "GrabFrameMath.h"
#include "GrabFingerLocalTransformRuntime.h"
#include "GrabFingerPoseRuntime.h"
#include "GrabMultiFingerContactMath.h"
#include "GrabMotionController.h"
#include "GrabNodeInfoMath.h"
#include "GrabNodeNamePolicy.h"
#include "GrabOppositionFrameMath.h"
#include "GrabSurfaceFrameMath.h"
#include "GrabVisualAuthorityPolicy.h"
#include "HandVisualLerpMath.h"
#include "HeldObjectBodySetPolicy.h"
#include "HeldObjectDampingMath.h"
#include "HeldObjectPhysicsMath.h"
#include "MeshGrab.h"
#include "ObjectPhysicsBodySet.h"
#include "PalmTransform.h"
#include "PhysicsBodyFrame.h"
#include "PhysicsShapeCast.h"
#include "PhysicsRecursiveWrappers.h"
#include "PhysicsUtils.h"
#include "PullMotionMath.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "TransformMath.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"

#include <cmath>
#include <format>
#include <array>
#include <limits>
#include <string_view>
#include <xmmintrin.h>

namespace frik::rock
{
    namespace
    {
        RE::NiPoint3 getMatrixColumn(const RE::NiMatrix3& matrix, int column) { return RE::NiPoint3(matrix.entry[0][column], matrix.entry[1][column], matrix.entry[2][column]); }

        RE::NiPoint3 getMatrixRow(const RE::NiMatrix3& matrix, int row) { return RE::NiPoint3(matrix.entry[row][0], matrix.entry[row][1], matrix.entry[row][2]); }

        RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
        {
            const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
            if (lengthSquared <= 1.0e-8f) {
                return RE::NiPoint3{};
            }

            const float inverseLength = 1.0f / std::sqrt(lengthSquared);
            return RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
        }

        const char* nodeDebugName(const RE::NiAVObject* node)
        {
            if (!node) {
                return "(null)";
            }

            const char* name = node->name.c_str();
            return name ? name : "(unnamed)";
        }

        const char* primaryBodyChoiceReasonName(object_physics_body_set::PrimaryBodyChoiceReason reason)
        {
            using object_physics_body_set::PrimaryBodyChoiceReason;
            switch (reason) {
            case PrimaryBodyChoiceReason::PreferredHitAccepted:
                return "preferredHitAccepted";
            case PrimaryBodyChoiceReason::SurfaceOwnerAccepted:
                return "surfaceOwnerAccepted";
            case PrimaryBodyChoiceReason::NearestAcceptedFallback:
                return "nearestAcceptedFallback";
            case PrimaryBodyChoiceReason::NoAcceptedBody:
                return "noAcceptedBody";
            case PrimaryBodyChoiceReason::None:
            default:
                return "none";
            }
        }

        const char* grabOrientationModeName(grab_surface_frame_math::GrabOrientationMode mode)
        {
            using grab_surface_frame_math::GrabOrientationMode;
            switch (mode) {
            case GrabOrientationMode::PreserveObjectRotation:
                return "PreserveObjectRotation";
            case GrabOrientationMode::SurfaceNormalAuto:
                return "SurfaceNormalAuto";
            case GrabOrientationMode::AuthoredOnly:
                return "AuthoredOnly";
            default:
                return "Unknown";
            }
        }

        const char* grabSurfaceFaceKindName(grab_surface_frame_math::GrabSurfaceFaceKind kind)
        {
            using grab_surface_frame_math::GrabSurfaceFaceKind;
            switch (kind) {
            case GrabSurfaceFaceKind::Side:
                return "side";
            case GrabSurfaceFaceKind::CapTopBottom:
                return "capTopBottom";
            case GrabSurfaceFaceKind::NarrowEdge:
                return "narrowEdge";
            case GrabSurfaceFaceKind::Ambiguous:
            default:
                return "ambiguous";
            }
        }

        const char* grabSurfaceTangentSourceName(grab_surface_frame_math::GrabSurfaceTangentSource source)
        {
            using grab_surface_frame_math::GrabSurfaceTangentSource;
            switch (source) {
            case GrabSurfaceTangentSource::TriangleLongestEdge:
                return "triangleLongestEdge";
            case GrabSurfaceTangentSource::PreservedObjectRoll:
                return "preservedObjectRoll";
            case GrabSurfaceTangentSource::AuthoredFrame:
                return "authoredFrame";
            case GrabSurfaceTangentSource::ObjectLongAxis:
                return "objectLongAxis";
            case GrabSurfaceTangentSource::ContactPatchPrincipal:
                return "contactPatchPrincipal";
            case GrabSurfaceTangentSource::Fallback:
            default:
                return "fallback";
            }
        }

        const char* grabSurfaceAlignmentDecisionName(grab_surface_frame_math::GrabSurfaceAlignmentDecision decision)
        {
            using grab_surface_frame_math::GrabSurfaceAlignmentDecision;
            switch (decision) {
            case GrabSurfaceAlignmentDecision::Accepted:
                return "accepted";
            case GrabSurfaceAlignmentDecision::RejectedMode:
                return "rejectedMode";
            case GrabSurfaceAlignmentDecision::RejectedMissingSelectionHit:
                return "rejectedMissingSelectionHit";
            case GrabSurfaceAlignmentDecision::RejectedOwnerMismatch:
                return "rejectedOwnerMismatch";
            case GrabSurfaceAlignmentDecision::RejectedPivotDistance:
                return "rejectedPivotDistance";
            case GrabSurfaceAlignmentDecision::RejectedSelectionDistance:
                return "rejectedSelectionDistance";
            case GrabSurfaceAlignmentDecision::RejectedLowConfidence:
                return "rejectedLowConfidence";
            case GrabSurfaceAlignmentDecision::RejectedAmbiguousTangent:
            default:
                return "rejectedAmbiguousTangent";
            }
        }

        float pointDistanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const RE::NiPoint3 delta = a - b;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        grab_surface_frame_math::GrabOrientationMode configuredGrabOrientationMode()
        {
            using grab_surface_frame_math::GrabOrientationMode;
            switch (g_rockConfig.rockGrabOrientationMode) {
            case 0:
                return GrabOrientationMode::PreserveObjectRotation;
            case 2:
                return GrabOrientationMode::AuthoredOnly;
            case 1:
            default:
                return GrabOrientationMode::SurfaceNormalAuto;
            }
        }

        GrabSurfaceHit makeCollisionQueryGrabSurfaceHit(const SelectedObject& selection, RE::NiAVObject* fallbackOwnerNode)
        {
            GrabSurfaceHit result{};
            if (!selection.hasHitPoint || !selection.hasHitNormal) {
                return result;
            }

            result.position = selection.hitPointWorld;
            result.normal = normalizeOrZero(selection.hitNormalWorld);
            result.triangleIndex = -1;
            result.distance = selection.distance;
            result.sourceNode = selection.hitNode ? selection.hitNode : fallbackOwnerNode;
            result.sourceShape = nullptr;
            result.sourceKind = GrabSurfaceSourceKind::CollisionQuery;
            result.shapeKey = selection.hitShapeKey;
            result.shapeCollisionFilterInfo = selection.hitShapeCollisionFilterInfo;
            result.hitFraction = selection.hitFraction;
            result.hasSelectionHit = true;
            result.selectionToMeshDistanceGameUnits = 0.0f;
            result.hasShapeKey = selection.hasHitShapeKey;
            result.hasTriangle = false;
            result.valid = grab_surface_frame_math::lengthSquared(result.normal) > 0.0f;
            return result;
        }

        struct RuntimeGrabContactPatch
        {
            grab_contact_patch_math::GrabContactPatchResult<RE::NiPoint3> patch{};
            std::array<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>, kMaxGrabContactPatchSamples> samples{};
            std::uint32_t sampleCount = 0;
            int castHitCount = 0;
            int rejectedBodyHits = 0;
            int rejectedInvalidNormals = 0;
            bool meshSnapped = false;
            GrabSurfaceHit meshSnapHit{};
            grab_contact_patch_math::GrabContactPatchPivotDecision<RE::NiPoint3> pivotDecision{};
            const char* pointMode = "contactPatchUnavailable";
        };

        struct RuntimeMultiFingerGripContact
        {
            grab_multi_finger_contact_math::GripContactSet<RE::NiPoint3> gripSet{};
            std::array<GrabSurfaceHit, grab_multi_finger_contact_math::kMaxFingerGroups> groupHits{};
            std::uint32_t candidateContactCount = 0;
            std::uint32_t meshHitCount = 0;
            std::uint32_t semanticCandidateContactCount = 0;
            std::uint32_t semanticMeshHitCount = 0;
            std::uint32_t liveProbeCandidateContactCount = 0;
            std::uint32_t liveProbeMeshHitCount = 0;
            std::uint32_t semanticGroupCount = 0;
            std::uint32_t liveProbeGroupCount = 0;
            std::uint32_t rejectedOwnerCount = 0;
            std::uint32_t rejectedDistanceCount = 0;
            const char* reason = "disabled";
        };

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
                                               bool liveProbeSource) {
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

                RE::NiTransform handContactWorld{};
                if (!tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ handBodyId }, handContactWorld)) {
                    return;
                }

                RE::NiPoint3 directionToObject = normalizeOrZero(objectWorldTransform.translate - handContactWorld.translate);
                if (directionToObject.x == 0.0f && directionToObject.y == 0.0f && directionToObject.z == 0.0f) {
                    directionToObject = RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
                }

                GrabSurfaceHit hit{};
                if (!findClosestGrabSurfaceHit(surfaceTriangles,
                        handContactWorld.translate,
                        directionToObject,
                        g_rockConfig.rockGrabLateralWeight,
                        g_rockConfig.rockGrabDirectionalWeight,
                        hit)) {
                    return;
                }

                const auto* ownerRecord = hit.sourceNode ? bodySet.findAcceptedRecordByOwnerNode(hit.sourceNode) : nullptr;
                if (!ownerRecord || ownerRecord->bodyId != resolvedBodyId) {
                    ++result.rejectedOwnerCount;
                    return;
                }

                const float contactDistance = pointDistanceGameUnits(handContactWorld.translate, hit.position);
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
                patch.handPointWorld = handContactWorld.translate;
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
                    false);
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
            const RE::NiPoint3& palmNormalWorld,
            const RE::NiPoint3& palmTangentWorld,
            const RE::NiPoint3& palmBitangentWorld,
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
                palmBitangent = normalizeOrZero(grab_surface_frame_math::cross(palmNormal, palmTangent));
            }
            if ((palmNormal.x == 0.0f && palmNormal.y == 0.0f && palmNormal.z == 0.0f) ||
                (palmTangent.x == 0.0f && palmTangent.y == 0.0f && palmTangent.z == 0.0f)) {
                result.patch.fallbackReason = "invalidPalmFrame";
                return result;
            }

            const int probeCount = std::clamp(g_rockConfig.rockGrabContactPatchProbeCount, 1, static_cast<int>(kMaxGrabContactPatchSamples));
            const float spacing = (std::max)(0.0f, g_rockConfig.rockGrabContactPatchProbeSpacingGameUnits);
            const float radius = (std::max)(0.1f, g_rockConfig.rockGrabContactPatchProbeRadiusGameUnits);
            std::array<RE::NiPoint3, kMaxGrabContactPatchSamples> offsets{
                RE::NiPoint3{},
                palmTangent * spacing,
                palmTangent * -spacing,
                palmBitangent * spacing,
                palmBitangent * -spacing,
            };

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
                for (int hitIndex = 0; hitIndex < hitCount; ++hitIndex) {
                    const auto& hit = hits[hitIndex];
                    if (hit.hitBodyInfo.m_bodyId.value != resolvedBodyId) {
                        ++result.rejectedBodyHits;
                        continue;
                    }

                    const RE::NiPoint3 normal = normalizeOrZero(RE::NiPoint3{ hit.normal.x, hit.normal.y, hit.normal.z });
                    if (normal.x == 0.0f && normal.y == 0.0f && normal.z == 0.0f) {
                        ++result.rejectedInvalidNormals;
                        continue;
                    }

                    const float fraction = hit.fraction.storage;
                    if (fraction >= bestFraction) {
                        continue;
                    }

                    bestFraction = fraction;
                    bestSample.bodyId = hit.hitBodyInfo.m_bodyId.value;
                    bestSample.point = hkVectorToNiPoint(hit.position);
                    bestSample.normal = normal;
                    bestSample.fraction = fraction;
                    bestSample.accepted = true;
                    bestSample.rejectionReason = "none";
                    foundProbeHit = true;
                }

                if (foundProbeHit) {
                    fitSamples.push_back(bestSample);
                    if (result.sampleCount < result.samples.size()) {
                        result.samples[result.sampleCount++] = bestSample;
                    }
                }
            }

            result.patch = grab_contact_patch_math::fitContactPatch(fitSamples,
                grabPivotAWorld,
                palmNormal,
                palmTangent,
                g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees);
            if (!result.patch.valid) {
                result.pointMode = "contactPatchFailed";
                return result;
            }

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
                        ownerMatches = snapOwnerRecord && snapOwnerRecord->bodyId == resolvedBodyId;
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

            if (!grab_contact_patch_math::contactPatchNormalMatchesSelection(result.patch,
                    selection.hitNormalWorld,
                    selection.hasHitNormal,
                    palmNormal,
                    g_rockConfig.rockGrabContactPatchMaxNormalAngleDegrees)) {
                result.patch.valid = false;
                result.patch.orientationReliable = false;
                result.patch.confidence = 0.0f;
                result.patch.fallbackReason = "selectionNormalMismatch";
                result.pointMode = "contactPatchRejectedSelectionNormal";
                return result;
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

        float translationDeltaGameUnits(const RE::NiTransform& a, const RE::NiTransform& b)
        {
            const RE::NiPoint3 delta = a.translate - b.translate;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
        {
            const RE::NiMatrix3 delta = a.Transpose() * b;
            float cosTheta = (delta.entry[0][0] + delta.entry[1][1] + delta.entry[2][2] - 1.0f) * 0.5f;
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        float axisDeltaDegrees(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const float dot = a.x * b.x + a.y * b.y + a.z * b.z;
            const float lenA = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
            const float lenB = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
            if (lenA < 0.0001f || lenB < 0.0001f) {
                return -1.0f;
            }

            float cosTheta = dot / (lenA * lenB);
            if (cosTheta < -1.0f) {
                cosTheta = -1.0f;
            } else if (cosTheta > 1.0f) {
                cosTheta = 1.0f;
            }
            return std::acos(cosTheta) * (180.0f / 3.14159265358979323846f);
        }

        float max3(float a, float b, float c) { return (std::max)((std::max)(a, b), c); }

        RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = hkMatrix[0];
            result.entry[1][0] = hkMatrix[1];
            result.entry[2][0] = hkMatrix[2];
            result.entry[0][1] = hkMatrix[4];
            result.entry[1][1] = hkMatrix[5];
            result.entry[2][1] = hkMatrix[6];
            result.entry[0][2] = hkMatrix[8];
            result.entry[1][2] = hkMatrix[9];
            result.entry[2][2] = hkMatrix[10];
            return result;
        }

        RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix)
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = hkMatrix[0];
            result.entry[0][1] = hkMatrix[1];
            result.entry[0][2] = hkMatrix[2];
            result.entry[1][0] = hkMatrix[4];
            result.entry[1][1] = hkMatrix[5];
            result.entry[1][2] = hkMatrix[6];
            result.entry[2][0] = hkMatrix[8];
            result.entry[2][1] = hkMatrix[9];
            result.entry[2][2] = hkMatrix[10];
            return result;
        }

        RE::NiTransform makeIdentityTransform()
        {
            return transform_math::makeIdentityTransform<RE::NiTransform>();
        }

        physics_recursive_wrappers::MotionPreset motionPresetFromMotionType(
            physics_body_classifier::BodyMotionType motionType,
            std::uint16_t fallbackMotionPropertiesId)
        {
            switch (motionType) {
            case physics_body_classifier::BodyMotionType::Static:
                return physics_recursive_wrappers::MotionPreset::Static;
            case physics_body_classifier::BodyMotionType::Keyframed:
                return physics_recursive_wrappers::MotionPreset::Keyframed;
            case physics_body_classifier::BodyMotionType::Dynamic:
                return physics_recursive_wrappers::MotionPreset::Dynamic;
            default:
                break;
            }

            switch (fallbackMotionPropertiesId & 0xFF) {
            case 0:
                return physics_recursive_wrappers::MotionPreset::Static;
            case 2:
                return physics_recursive_wrappers::MotionPreset::Keyframed;
            case 1:
            default:
                return physics_recursive_wrappers::MotionPreset::Dynamic;
            }
        }

        active_grab_body_lifecycle::BodyLifecycleAudit restoreActiveGrabLifecycle(RE::hknpWorld* world,
            const active_grab_body_lifecycle::BodyLifecycleSnapshot& snapshot,
            const active_grab_body_lifecycle::BodyRestorePlan& plan,
            std::uint32_t primaryBodyId,
            const char* handName,
            const char* context)
        {
            auto audit = active_grab_body_lifecycle::makeLifecycleAudit(snapshot, plan, primaryBodyId);
            if (!world) {
                return audit;
            }

            for (const auto& entry : plan.entries) {
                const auto bodyId = entry.record.bodyId;
                if (bodyId == INVALID_BODY_ID) {
                    continue;
                }

                if (entry.restoreFilter) {
                    body_collision::setFilterInfo(world, RE::hknpBodyId{ bodyId }, entry.record.filterInfo);
                }
            }

            for (const auto& command : snapshot.makeMotionRestoreCommands(plan)) {
                auto* ownerNode = reinterpret_cast<RE::NiAVObject*>(command.ownerKey);
                if (!ownerNode) {
                    continue;
                }
                physics_recursive_wrappers::setMotionRecursive(
                    ownerNode,
                    motionPresetFromMotionType(command.motionType, command.motionPropertiesId),
                    command.recursive,
                    command.force,
                    command.activate);
            }

            ROCK_LOG_DEBUG(Hand,
                "{} hand grab lifecycle audit {}: bodies={} converted={} restoredMotion={} restoredFilter={} dampingSnapshots={} inertiaSnapshots={} primaryBody={}",
                handName ? handName : "?",
                context ? context : "",
                audit.bodyCount,
                audit.convertedCount,
                audit.restoredMotionCount,
                audit.restoredFilterCount,
                audit.dampingSnapshotCount,
                audit.inertiaSnapshotCount,
                audit.primaryBodyId);
            return audit;
        }

        std::uint16_t motionPropsIdFromRecordMotionType(physics_body_classifier::BodyMotionType motionType, std::uint16_t fallback)
        {
            switch (motionType) {
            case physics_body_classifier::BodyMotionType::Static:
                return 0;
            case physics_body_classifier::BodyMotionType::Dynamic:
                return 1;
            case physics_body_classifier::BodyMotionType::Keyframed:
                return 2;
            default:
                return fallback;
            }
        }

        RE::NiTransform invertTransform(const RE::NiTransform& transform) { return transform_math::invertTransform(transform); }

        RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child) { return transform_math::composeTransforms(parent, child); }

        RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld, const RE::NiTransform& bodyLocalTransform)
        {
            return multiplyTransforms(bodyWorld, invertTransform(bodyLocalTransform));
        }

        std::vector<GrabLocalTriangle> cacheTrianglesInLocalSpace(const std::vector<TriangleData>& worldTriangles, const RE::NiTransform& nodeWorld)
        {
            std::vector<GrabLocalTriangle> localTriangles;
            localTriangles.reserve(worldTriangles.size());
            for (const auto& triangle : worldTriangles) {
                localTriangles.push_back(GrabLocalTriangle{
                    transform_math::worldPointToLocal(nodeWorld, triangle.v0),
                    transform_math::worldPointToLocal(nodeWorld, triangle.v1),
                    transform_math::worldPointToLocal(nodeWorld, triangle.v2),
                });
            }
            return localTriangles;
        }

        std::vector<TriangleData> rebuildTrianglesInWorldSpace(const std::vector<GrabLocalTriangle>& localTriangles, const RE::NiTransform& nodeWorld)
        {
            std::vector<TriangleData> worldTriangles;
            worldTriangles.reserve(localTriangles.size());
            for (const auto& triangle : localTriangles) {
                worldTriangles.push_back(TriangleData{
                    transform_math::localPointToWorld(nodeWorld, triangle.v0),
                    transform_math::localPointToWorld(nodeWorld, triangle.v1),
                    transform_math::localPointToWorld(nodeWorld, triangle.v2),
                });
            }
            return worldTriangles;
        }

        RE::NiAVObject* findNamedNodeRecursive(RE::NiAVObject* root, std::string_view name, int maxDepth = 12)
        {
            if (!root || name.empty() || maxDepth < 0) {
                return nullptr;
            }

            const char* nodeName = root->name.c_str();
            if (nodeName && name == nodeName) {
                return root;
            }

            auto* node = root->IsNode();
            if (!node) {
                return nullptr;
            }

            auto& children = node->GetRuntimeData().children;
            for (auto i = decltype(children.size()){ 0 }; i < children.size(); ++i) {
                if (auto* found = findNamedNodeRecursive(children[i].get(), name, maxDepth - 1)) {
                    return found;
                }
            }
            return nullptr;
        }

        RE::NiTransform getLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryResolveLiveBodyWorldTransform(world, bodyId, result);
            return result;
        }

        RE::NiTransform computeRuntimeBodyLocalTransform(const RE::NiTransform& nodeWorld, const RE::NiTransform& bodyWorld)
        {
            return multiplyTransforms(invertTransform(nodeWorld), bodyWorld);
        }

        RE::NiPoint3 computeLocalBoundsLongAxis(const std::vector<GrabLocalTriangle>& localTriangles)
        {
            if (localTriangles.empty()) {
                return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
            }

            RE::NiPoint3 minPoint{ (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)(), (std::numeric_limits<float>::max)() };
            RE::NiPoint3 maxPoint{ (std::numeric_limits<float>::lowest)(), (std::numeric_limits<float>::lowest)(), (std::numeric_limits<float>::lowest)() };
            auto visit = [&](const RE::NiPoint3& point) {
                minPoint.x = (std::min)(minPoint.x, point.x);
                minPoint.y = (std::min)(minPoint.y, point.y);
                minPoint.z = (std::min)(minPoint.z, point.z);
                maxPoint.x = (std::max)(maxPoint.x, point.x);
                maxPoint.y = (std::max)(maxPoint.y, point.y);
                maxPoint.z = (std::max)(maxPoint.z, point.z);
            };
            for (const auto& triangle : localTriangles) {
                visit(triangle.v0);
                visit(triangle.v1);
                visit(triangle.v2);
            }

            const RE::NiPoint3 extent = maxPoint - minPoint;
            if (extent.y >= extent.x && extent.y >= extent.z) {
                return RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
            }
            if (extent.z >= extent.x && extent.z >= extent.y) {
                return RE::NiPoint3{ 0.0f, 0.0f, 1.0f };
            }
            return RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        }

        RE::NiTransform buildDesiredObjectWorldFromAuthoredGrabNode(const RE::NiTransform& objectWorldTransform,
            const RE::NiTransform& grabNodeWorld,
            const RE::NiTransform& handWorldTransform,
            const RE::NiPoint3& grabPivotAWorld)
        {
            return grab_node_info_math::buildDesiredObjectWorldFromAuthoredGrabNode(objectWorldTransform, grabNodeWorld, handWorldTransform, grabPivotAWorld);
        }

        void logGrabNodeInfo(const char* handName,
            bool isLeft,
            const RE::NiAVObject* parentNode,
            const RE::NiAVObject* authoredGrabNode,
            const RE::NiTransform& desiredObjectWorld,
            const RE::NiTransform& handWorldTransform,
            const RE::NiPoint3& grabPivotAWorld,
            const char* grabPointMode,
            grab_surface_frame_math::GrabOrientationMode orientationMode,
            grab_surface_frame_math::GrabSurfaceAlignmentDecision alignmentDecision)
        {
            if (!g_rockConfig.rockPrintGrabNodeInfo) {
                return;
            }

            const RE::NiTransform grabNodeLocal =
                grab_node_info_math::computeGrabNodeLocalTransformForCurrentGrab(desiredObjectWorld, handWorldTransform, grabPivotAWorld);
            const auto nifskopeEulerDegrees = grab_node_info_math::nifskopeMatrixToEulerDegrees(grabNodeLocal.rotate);
            const char* configuredNodeName = isLeft ? g_rockConfig.rockGrabNodeNameLeft.c_str() : g_rockConfig.rockGrabNodeNameRight.c_str();

            ROCK_LOG_INFO(Hand, "{} ROCK GRAB NODE INFO BEGIN", handName);
            ROCK_LOG_INFO(Hand, "Parent: {}", nodeDebugName(parentNode));
            ROCK_LOG_INFO(Hand, "Name: {}", configuredNodeName);
            ROCK_LOG_INFO(Hand,
                "Translation: {:.4f} {:.4f} {:.4f}",
                grabNodeLocal.translate.x,
                grabNodeLocal.translate.y,
                grabNodeLocal.translate.z);
            ROCK_LOG_INFO(Hand,
                "Rotation: {:.4f} {:.4f} {:.4f}",
                nifskopeEulerDegrees.x,
                nifskopeEulerDegrees.y,
                nifskopeEulerDegrees.z);
            ROCK_LOG_INFO(Hand,
                "Source: authoredNode={} authoredName={} pointMode={} orientationMode={} alignment={}",
                authoredGrabNode ? "yes" : "no",
                authoredGrabNode ? nodeDebugName(authoredGrabNode) : "none",
                grabPointMode ? grabPointMode : "unknown",
                grabOrientationModeName(orientationMode),
                grabSurfaceAlignmentDecisionName(alignmentDecision));
            ROCK_LOG_INFO(Hand, "{} ROCK GRAB NODE INFO END", handName);
        }

        struct HeldMotionCompensationResult
        {
            RE::NiPoint3 primaryLocalLinearVelocity{};
            bool hasPrimaryVelocity = false;
        };

        HeldMotionCompensationResult applyHeldMotionCompensation(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
            const RE::NiPoint3& previousPlayerSpaceVelocityHavok,
            float damping,
            bool residualDampingEnabled)
        {
            HeldMotionCompensationResult result{};
            if (!world) {
                return result;
            }

            const bool applyPlayerSpaceVelocity = playerSpaceFrame.enabled && !playerSpaceFrame.warp;
            const RE::NiPoint3 previousPlayerVelocity = applyPlayerSpaceVelocity ? previousPlayerSpaceVelocityHavok : RE::NiPoint3{};
            (void)damping;
            (void)residualDampingEnabled;

            constexpr std::size_t kMaxSampledMotionSlots = 96;
            std::array<std::uint32_t, kMaxSampledMotionSlots> sampledMotionSlots{};
            std::size_t sampledMotionSlotCount = 0;

            auto motionSlotAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                    if (sampledMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto sampleBody = [&](std::uint32_t bodyId) {
                if (bodyId == INVALID_BODY_ID) {
                    return;
                }

                auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
                if (!body) {
                    return;
                }

                const std::uint32_t motionIndex = body->motionIndex;
                if (!body_frame::hasUsableMotionIndex(motionIndex) || motionSlotAlreadySampled(motionIndex)) {
                    return;
                }

                if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                    return;
                }

                auto* motion = havok_runtime::getMotion(world, motionIndex);
                if (!motion) {
                    return;
                }

                sampledMotionSlots[sampledMotionSlotCount++] = motionIndex;

                const RE::NiPoint3 currentLinearVelocity{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z };
                const RE::NiPoint3 localLinearVelocity{
                    currentLinearVelocity.x - previousPlayerVelocity.x,
                    currentLinearVelocity.y - previousPlayerVelocity.y,
                    currentLinearVelocity.z - previousPlayerVelocity.z,
                };

                if (bodyId == primaryBodyId.value) {
                    result.primaryLocalLinearVelocity = localLinearVelocity;
                    result.hasPrimaryVelocity = true;
                }
            };

            sampleBody(primaryBodyId.value);
            for (const auto bodyId : heldBodyIds) {
                sampleBody(bodyId);
            }

            return result;
        }

        void setHeldLinearVelocity(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const RE::NiPoint3& linearVelocity,
            float angularVelocityKeep = 1.0f)
        {
            if (!world) {
                return;
            }

            constexpr std::size_t kMaxVelocityMotionSlots = 96;
            std::array<std::uint32_t, kMaxVelocityMotionSlots> updatedMotionSlots{};
            std::size_t updatedMotionSlotCount = 0;

            auto alreadyUpdated = [&updatedMotionSlots, &updatedMotionSlotCount](std::uint32_t motionIndex) {
                for (std::size_t i = 0; i < updatedMotionSlotCount; ++i) {
                    if (updatedMotionSlots[i] == motionIndex) {
                        return true;
                    }
                }
                return false;
            };

            auto setBody = [&](std::uint32_t bodyId) {
                if (bodyId == INVALID_BODY_ID) {
                    return;
                }

                auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ bodyId });
                if (!body) {
                    return;
                }

                const std::uint32_t motionIndex = body->motionIndex;
                if (!body_frame::hasUsableMotionIndex(motionIndex) || alreadyUpdated(motionIndex)) {
                    return;
                }

                if (updatedMotionSlotCount >= updatedMotionSlots.size()) {
                    return;
                }

                auto* motion = havok_runtime::getMotion(world, motionIndex);
                if (!motion) {
                    return;
                }

                updatedMotionSlots[updatedMotionSlotCount++] = motionIndex;
                const float angularKeep = std::clamp(std::isfinite(angularVelocityKeep) ? angularVelocityKeep : 1.0f, 0.0f, 1.0f);
                havok_runtime::setBodyVelocityDeferred(world,
                    bodyId,
                    RE::hkVector4f{ linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f },
                    RE::hkVector4f{ motion->angularVelocity.x * angularKeep, motion->angularVelocity.y * angularKeep, motion->angularVelocity.z * angularKeep, 0.0f });
            };

            setBody(primaryBodyId.value);
            for (const auto bodyId : heldBodyIds) {
                setBody(bodyId);
            }
        }

        float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            if (!world || bodyId.value == INVALID_BODY_ID) {
                return 0.0f;
            }

            auto* motion = havok_runtime::getBodyMotion(world, bodyId);
            if (!motion) {
                return 0.0f;
            }

            auto packedInvMass = static_cast<std::uint16_t>(motion->packedInverseInertia[3]);
            if (packedInvMass == 0) {
                return 0.0f;
            }

            std::uint32_t asUint = static_cast<std::uint32_t>(packedInvMass) << 16;
            float invMass = 0.0f;
            std::memcpy(&invMass, &asUint, sizeof(float));
            if (!std::isfinite(invMass) || invMass <= 0.0001f) {
                return 0.0f;
            }
            return 1.0f / invMass;
        }

        void applyRockGrabHandPose(bool isLeft,
            const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
            std::array<float, 15>& currentJointPose,
            bool& hasCurrentJointPose,
            std::array<RE::NiTransform, 15>& currentLocalTransforms,
            std::uint16_t& currentLocalTransformMask,
            bool& hasCurrentLocalTransforms,
            float deltaTime)
        {
            auto* api = frik::api::FRIKApi::inst;
            if (!api) {
                return;
            }

            const auto hand = handFromBool(isLeft);
            grab_finger_local_transform_runtime::State localTransformState{
                .currentTransforms = currentLocalTransforms,
                .currentMask = currentLocalTransformMask,
                .hasCurrentTransforms = hasCurrentLocalTransforms,
            };
            auto syncLocalTransformState = [&]() {
                currentLocalTransforms = localTransformState.currentTransforms;
                currentLocalTransformMask = localTransformState.currentMask;
                hasCurrentLocalTransforms = localTransformState.hasCurrentTransforms;
            };

            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved && fingerPose.hasJointValues &&
                api->setHandPoseCustomJointPositionsWithPriority) {
                if (!hasCurrentJointPose) {
                    currentJointPose = fingerPose.jointValues;
                    hasCurrentJointPose = true;
                } else {
                    currentJointPose = grab_finger_pose_math::advanceJointValues(
                        currentJointPose, fingerPose.jointValues, g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
                }

                api->setHandPoseCustomJointPositionsWithPriority("ROCK_Grab", hand, currentJointPose.data(), 100);
                const bool publishedLocalTransforms = grab_finger_local_transform_runtime::publishLocalTransformPose("ROCK_Grab",
                    hand,
                    isLeft,
                    fingerPose,
                    currentJointPose,
                    grab_finger_local_transform_runtime::Options{
                        .enabled = g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                        .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                        .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                        .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                        .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                        .thumbAlternateCurveStrength = 0.0f,
                    },
                    deltaTime,
                    100,
                    localTransformState);
                syncLocalTransformState();
                if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand FINGER JOINT POSE: thumb=({:.2f},{:.2f},{:.2f}) index=({:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={} localTransforms={} mask=0x{:04X}",
                        isLeft ? "Left" : "Right", currentJointPose[0], currentJointPose[1], currentJointPose[2], currentJointPose[3], currentJointPose[4],
                        currentJointPose[5], fingerPose.hitCount, fingerPose.candidateTriangleCount, fingerPose.usedAlternateThumbCurve ? "yes" : "no",
                        publishedLocalTransforms ? "yes" : "no", currentLocalTransformMask);
                }
                return;
            }

            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && fingerPose.solved && api->setHandPoseCustomFingerPositionsWithPriority) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                syncLocalTransformState();
                api->setHandPoseCustomFingerPositionsWithPriority("ROCK_Grab", hand, fingerPose.values[0], fingerPose.values[1], fingerPose.values[2], fingerPose.values[3],
                    fingerPose.values[4], 100);
                if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand FINGER POSE: mesh values=({:.2f},{:.2f},{:.2f},{:.2f},{:.2f}) hits={} candidateTris={} altThumb={}",
                        isLeft ? "Left" : "Right", fingerPose.values[0], fingerPose.values[1], fingerPose.values[2], fingerPose.values[3], fingerPose.values[4],
                        fingerPose.hitCount, fingerPose.candidateTriangleCount, fingerPose.usedAlternateThumbCurve ? "yes" : "no");
                }
                return;
            }

            hasCurrentJointPose = false;
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
            syncLocalTransformState();
            api->setHandPoseWithPriority("ROCK_Grab", hand, frik::api::FRIKApi::HandPoses::Fist, 100);
            if (g_rockConfig.rockGrabMeshFingerPoseEnabled && g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand, "{} hand FINGER POSE: fallback fist solved={} hits={} candidateTris={}", isLeft ? "Left" : "Right", fingerPose.solved ? "yes" : "no",
                    fingerPose.hitCount, fingerPose.candidateTriangleCount);
            }
        }

        constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";
        constexpr int GRAB_EXTERNAL_HAND_PRIORITY = 90;

        void applyGrabExternalHandWorldTransform(bool isLeft, const RE::NiTransform& adjustedHandTransform)
        {
            auto* api = frik::api::FRIKApi::inst;
            if (!api || !api->applyExternalHandWorldTransform) {
                return;
            }

            api->applyExternalHandWorldTransform(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft), adjustedHandTransform, GRAB_EXTERNAL_HAND_PRIORITY);
        }

        void clearGrabExternalHandWorldTransform(bool isLeft)
        {
            auto* api = frik::api::FRIKApi::inst;
            if (!api || !api->clearExternalHandWorldTransform) {
                return;
            }

            api->clearExternalHandWorldTransform(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
        }

        void logRuntimeScaleIfChanged(bool isLeft, const char* handName, const RE::NiTransform& handWorldTransform, const RE::NiAVObject* collidableNode)
        {
            struct RuntimeScaleLogState
            {
                bool initialized = false;
                float handScale = 0.0f;
                float collidableScale = 0.0f;
                float vrScale = 0.0f;
            };

            static std::array<RuntimeScaleLogState, 2> s_scaleStates{};
            auto& state = s_scaleStates[isLeft ? 1 : 0];
            auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
            const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;
            const float collidableScale = collidableNode ? collidableNode->world.scale : -1.0f;
            auto changedEnough = [](float a, float b) {
                return std::fabs(a - b) > 0.01f;
            };

            if (!state.initialized || changedEnough(state.handScale, handWorldTransform.scale) || changedEnough(state.collidableScale, collidableScale) ||
                changedEnough(state.vrScale, vrScale)) {
                ROCK_LOG_DEBUG(Hand,
                    "Runtime scale {}: handScale={:.3f} collidableScale={:.3f} vrScale={:.3f} previous=({:.3f},{:.3f},{:.3f})",
                    handName,
                    handWorldTransform.scale,
                    collidableScale,
                    vrScale,
                    state.handScale,
                    state.collidableScale,
                    state.vrScale);
                state.initialized = true;
                state.handScale = handWorldTransform.scale;
                state.collidableScale = collidableScale;
                state.vrScale = vrScale;
            }
        }
    }

    static void nativeVRGrabDrop(void* playerChar, int handIndex)
    {
        typedef void func_t(void*, int, std::uint64_t);
        static REL::Relocation<func_t> func{ REL::Offset(offsets::kFunc_NativeVRGrabDrop) };
        func(playerChar, handIndex, 0);
    }

    void Hand::clearGrabHandCollisionSuppressionState()
    {
        hand_collision_suppression_math::clear(_grabHandCollisionSuppression);
        hand_collision_suppression_math::clear(_grabHandCollisionDelayedRestore);
    }

    void Hand::suppressHandCollisionForGrab(RE::hknpWorld* world)
    {
        hand_collision_suppression_math::clear(_grabHandCollisionDelayedRestore);

        if (!world || !hasCollisionBody())
            return;

        auto suppressBody = [&](std::uint32_t bodyId) {
            if (bodyId == INVALID_BODY_ID) {
                return;
            }

            std::uint32_t currentFilter = 0;
            if (!body_collision::tryReadFilterInfo(world, RE::hknpBodyId{ bodyId }, currentFilter)) {
                return;
            }

            const auto suppression = hand_collision_suppression_math::beginSuppression(_grabHandCollisionSuppression, bodyId, currentFilter);
            if (!suppression.stored) {
                ROCK_LOG_WARN(Hand, "{} hand: grab hand collision suppression set full; bodyId={} left active", handName(), bodyId);
                return;
            }

            const auto registryResult = collision_suppression_registry::globalCollisionSuppressionRegistry().acquire(
                world,
                bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::Grab,
                "held-grab-hand");

            if (registryResult.valid && (registryResult.filterChanged || registryResult.firstLeaseForBody)) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: grab hand collision lease acquired bodyId={} filter=0x{:08X}->0x{:08X} wasDisabledBeforeGrab={} leases={}",
                    handName(),
                    bodyId,
                    registryResult.filterBefore,
                    registryResult.filterAfter,
                    registryResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                    registryResult.activeLeaseCount);
            }
        };

        const std::uint32_t colliderCount = _boneColliders.getBodyCount();
        if (colliderCount > 0) {
            for (std::uint32_t i = 0; i < colliderCount; ++i) {
                suppressBody(_boneColliders.getBodyIdAtomic(i));
            }
        } else {
            suppressBody(_handBody.getBodyId().value);
        }
    }

    void Hand::restoreHandCollisionAfterGrab(RE::hknpWorld* world)
    {
        if (!hand_collision_suppression_math::hasActive(_grabHandCollisionSuppression))
            return;

        if (!world) {
            ROCK_LOG_WARN(Hand,
                "{} hand: cannot restore grab hand collision yet (world={}); preserving suppression state",
                handName(),
                static_cast<const void*>(world));
            return;
        }

        bool restoreDeferred = false;
        for (const auto& entry : _grabHandCollisionSuppression.entries) {
            if (!entry.active || entry.bodyId == INVALID_BODY_ID) {
                continue;
            }

            const auto releaseResult = collision_suppression_registry::globalCollisionSuppressionRegistry().release(
                world,
                entry.bodyId,
                collision_suppression_registry::CollisionSuppressionOwner::Grab,
                "held-grab-hand");
            if (releaseResult.readFailed) {
                restoreDeferred = true;
                continue;
            }

            ROCK_LOG_DEBUG(Hand,
                "{} hand: grab hand collision lease released bodyId={} filter=0x{:08X}->0x{:08X} restoreDisabled={} fullyReleased={}",
                handName(),
                entry.bodyId,
                releaseResult.filterBefore,
                releaseResult.filterAfter,
                releaseResult.wasNoCollideBeforeSuppression ? "yes" : "no",
                releaseResult.bodyFullyReleased ? "yes" : "no");
        }
        if (restoreDeferred) {
            ROCK_LOG_WARN(Hand, "{} hand: grab hand collision restore deferred; suppression leases preserved", handName());
            return;
        }
        clearGrabHandCollisionSuppressionState();
    }

    void Hand::updateDelayedGrabHandCollisionRestore(RE::hknpWorld* world, float deltaTime)
    {
        if (!hand_collision_suppression_math::advanceDelayedRestore(_grabHandCollisionDelayedRestore, _grabHandCollisionSuppression, deltaTime)) {
            return;
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand: delayed grab hand collision restore ready bodies={} firstBodyId={} delayRemaining={:.3f}",
            handName(),
            _grabHandCollisionDelayedRestore.bodyCount,
            _grabHandCollisionDelayedRestore.bodyId,
            _grabHandCollisionDelayedRestore.remainingSeconds);
        restoreHandCollisionAfterGrab(world);
    }

    bool Hand::getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_activeConstraint.isValid() || !_activeConstraint.constraintData || !_handBody.isValid() ||
            _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        const auto handBodyId = _handBody.getBodyId();
        if (handBodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform handBodyWorld{};
        RE::NiTransform objectBodyWorld{};
        if (!tryResolveLiveBodyWorldTransform(world, handBodyId, handBodyWorld) ||
            !tryResolveLiveBodyWorldTransform(world, _savedObjectState.bodyId, objectBodyWorld)) {
            return false;
        }

        auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
        auto* pivotALocal = reinterpret_cast<const float*>(constraintData + offsets::kTransformA_Pos);
        auto* pivotBLocal = reinterpret_cast<const float*>(constraintData + offsets::kTransformB_Pos);
        const RE::NiPoint3 pivotALocalGame{ pivotALocal[0] * havokToGameScale(), pivotALocal[1] * havokToGameScale(), pivotALocal[2] * havokToGameScale() };
        const RE::NiPoint3 pivotBLocalGame{ pivotBLocal[0] * havokToGameScale(), pivotBLocal[1] * havokToGameScale(), pivotBLocal[2] * havokToGameScale() };

        out.handPivotWorld = transform_math::localPointToWorld(handBodyWorld, pivotALocalGame);
        out.objectPivotWorld = transform_math::localPointToWorld(objectBodyWorld, pivotBLocalGame);
        out.handBodyWorld = handBodyWorld.translate;
        out.objectBodyWorld = objectBodyWorld.translate;

        const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
        out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
        return true;
    }

    bool Hand::getGrabSurfaceFrameDebugSnapshot(RE::hknpWorld* world, GrabSurfaceFrameDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.hasSurfaceHit || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        const RE::NiTransform liveBodyWorld = getLiveBodyWorldTransform(world, _savedObjectState.bodyId);
        const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabFrame.bodyLocal);
        out.contactPointWorld = transform_math::localPointToWorld(currentNodeWorld, _grabFrame.surfacePointLocal);

        const RE::NiPoint3 normalLocal = _grabFrame.hasSurfaceFrame ? _grabFrame.surfaceFrameLocal.normal : _grabFrame.surfaceNormalLocal;
        const RE::NiPoint3 normalWorld = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, normalLocal));
        if (normalWorld.x == 0.0f && normalWorld.y == 0.0f && normalWorld.z == 0.0f) {
            return false;
        }

        constexpr float kFrameAxisLengthGameUnits = 12.0f;
        out.normalEndWorld = out.contactPointWorld + normalWorld * kFrameAxisLengthGameUnits;

        if (_grabFrame.hasSurfaceFrame) {
            const RE::NiPoint3 tangentWorld = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, _grabFrame.surfaceFrameLocal.tangent));
            const RE::NiPoint3 bitangentWorld = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, _grabFrame.surfaceFrameLocal.bitangent));
            out.hasTangent = !(tangentWorld.x == 0.0f && tangentWorld.y == 0.0f && tangentWorld.z == 0.0f);
            out.hasBitangent = !(bitangentWorld.x == 0.0f && bitangentWorld.y == 0.0f && bitangentWorld.z == 0.0f);
            out.tangentEndWorld = out.hasTangent ? out.contactPointWorld + tangentWorld * kFrameAxisLengthGameUnits : out.contactPointWorld;
            out.bitangentEndWorld = out.hasBitangent ? out.contactPointWorld + bitangentWorld * kFrameAxisLengthGameUnits : out.contactPointWorld;
        } else {
            out.tangentEndWorld = out.contactPointWorld;
            out.bitangentEndWorld = out.contactPointWorld;
        }

        return true;
    }

    bool Hand::getGrabContactPatchDebugSnapshot(RE::hknpWorld* world, GrabContactPatchDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.hasContactPatch || _grabFrame.contactPatchSampleCount == 0 || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        const RE::NiTransform liveBodyWorld = getLiveBodyWorldTransform(world, _savedObjectState.bodyId);
        const std::uint32_t count = (std::min)(_grabFrame.contactPatchSampleCount, static_cast<std::uint32_t>(out.samplePointsWorld.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            out.samplePointsWorld[i] = transform_math::localPointToWorld(liveBodyWorld, _grabFrame.contactPatchSamples[i].point);
        }
        out.sampleCount = count;
        return count > 0;
    }

    bool Hand::getGrabTransformTelemetrySnapshot(RE::hknpWorld* world,
        const RE::NiTransform& rawHandWorld,
        bool visualAuthorityEnabled,
        grab_transform_telemetry::RuntimeSample& out) const
    {
        out = {};
        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        out.valid = true;
        out.isLeft = _isLeft;
        out.visualAuthorityEnabled = visualAuthorityEnabled;
        out.rawHandWorld = rawHandWorld;
        out.heldFormId = _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0;
        out.heldBodyId = _savedObjectState.bodyId.value;
        out.handBodyId = _handBody.isValid() ? _handBody.getBodyId().value : INVALID_BODY_ID;

        if (_handBody.isValid() && _handBody.getBodyId().value != INVALID_BODY_ID) {
            const auto handBodyFrame = resolveLiveBodyWorldTransform(world, _handBody.getBodyId());
            if (handBodyFrame.valid) {
                out.handBodyWorld = handBodyFrame.transform;
                out.handBodySource = handBodyFrame.source;
                out.handMotionIndex = handBodyFrame.motionIndex;
                out.hasHandBodyWorld = true;
                out.rawToHandBody = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.handBodyWorld);
            }
        }

        const auto heldBodyFrame = resolveLiveBodyWorldTransform(world, _savedObjectState.bodyId);
        out.heldBodyWorld = heldBodyFrame.transform;
        out.heldBodySource = heldBodyFrame.source;
        out.heldMotionIndex = heldBodyFrame.motionIndex;
        out.hasHeldBodyWorld = heldBodyFrame.valid;
        if (heldBodyFrame.valid) {
            out.heldBodyDerivedNodeWorld = deriveNodeWorldFromBodyWorld(out.heldBodyWorld, _grabFrame.bodyLocal);
            out.hasHeldBodyDerivedNodeWorld = true;
        }

        /*
         * Telemetry keeps the same split as runtime authority: the held visual
         * node is the HIGGS/FRIK visual source, while the body-derived node is
         * only a physics diagnostic. Naming both separately prevents screenshots
         * and logs from treating a reconstructed body frame as the visual node.
         */
        RE::NiAVObject* heldVisualNode = nullptr;
        if (_savedObjectState.refr && !_savedObjectState.refr->IsDeleted() && !_savedObjectState.refr->IsDisabled()) {
            heldVisualNode = _grabFrame.heldNode ? _grabFrame.heldNode : _savedObjectState.refr->Get3D();
        }
        if (heldVisualNode) {
            out.heldNodeWorld = heldVisualNode->world;
            out.hasHeldNodeWorld = true;
        } else if (out.hasHeldBodyDerivedNodeWorld) {
            out.heldNodeWorld = out.heldBodyDerivedNodeWorld;
            out.hasHeldNodeWorld = true;
        }
        if (out.hasHeldNodeWorld && out.hasHeldBodyDerivedNodeWorld) {
            out.bodyDerivedNodeToHeldNode = grab_transform_telemetry::measureTransformDelta(out.heldBodyDerivedNodeWorld, out.heldNodeWorld);
        }
        if (_grabFrame.hasTelemetryCapture) {
            out.hasGrabStartFrames = true;
            out.liveHandWorldAtGrab = _grabFrame.liveHandWorldAtGrab;
            out.handBodyWorldAtGrab = _grabFrame.handBodyWorldAtGrab;
            out.objectNodeWorldAtGrab = _grabFrame.objectNodeWorldAtGrab;
            out.desiredObjectWorldAtGrab = _grabFrame.desiredObjectWorldAtGrab;
            out.rawHandSpace = _grabFrame.rawHandSpace;
            out.constraintHandSpace = _grabFrame.constraintHandSpace;
            out.handBodyToRawHandAtGrab = _grabFrame.handBodyToRawHandAtGrab;
            out.bodyLocal = _grabFrame.bodyLocal;
            out.currentRawDesiredObjectWorld =
                grab_transform_telemetry::computeCurrentDesiredObjectFromFrame(out.rawHandWorld, _grabFrame.rawHandSpace);
            if (out.hasHeldNodeWorld) {
                out.heldNodeToDesiredObjectAtGrab =
                    grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.desiredObjectWorldAtGrab);
                out.heldNodeToRawDesiredObject =
                    grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.currentRawDesiredObjectWorld);
            }
            if (out.hasHandBodyWorld) {
                out.currentConstraintDesiredObjectWorld =
                    grab_transform_telemetry::computeCurrentDesiredObjectFromFrame(out.handBodyWorld, _grabFrame.constraintHandSpace);
                out.currentConstraintDesiredBodyWorld =
                    grab_transform_telemetry::computeCurrentDesiredBodyFromHandBody(out.handBodyWorld, _grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
                if (out.hasHeldNodeWorld) {
                    out.heldNodeToConstraintDesiredObject =
                        grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.currentConstraintDesiredObjectWorld);
                }
                out.heldBodyToConstraintDesiredBody =
                    grab_transform_telemetry::measureTransformDelta(out.heldBodyWorld, out.currentConstraintDesiredBodyWorld);
            }
        }
        if (out.hasHeldNodeWorld) {
            out.higgsReverseTargetWorld = grab_transform_telemetry::computeHiggsReverseTarget(out.heldNodeWorld, _grabFrame.rawHandSpace);
            out.hasHiggsReverseTarget = true;
            out.rawToHiggsReverse = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.higgsReverseTargetWorld);
            out.rawToHiggsReverseAxes = grab_transform_telemetry::axisAlignmentDots(out.rawHandWorld.rotate, out.higgsReverseTargetWorld.rotate);
            out.constraintReverseTargetWorld =
                grab_transform_telemetry::computeConstraintReverseTarget(out.heldNodeWorld, _grabFrame.constraintHandSpace);
            out.hasConstraintReverseTarget = true;
            out.rawToConstraintReverse = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.constraintReverseTargetWorld);
            out.rawToConstraintReverseAxes = grab_transform_telemetry::axisAlignmentDots(out.rawHandWorld.rotate, out.constraintReverseTargetWorld.rotate);
            if (out.hasHandBodyWorld) {
                out.handBodyToConstraintReverse = grab_transform_telemetry::measureTransformDelta(out.handBodyWorld, out.constraintReverseTargetWorld);
            }
            out.higgsToConstraintReverse = grab_transform_telemetry::measureTransformDelta(out.higgsReverseTargetWorld, out.constraintReverseTargetWorld);
        }

        if (computeAdjustedHandTransformTarget(world, out.rockVisualTargetWorld)) {
            out.hasRockVisualTarget = true;
            out.rockVisualTargetUsedSurfaceFrame = false;
            out.rawToRockVisual = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.rockVisualTargetWorld);
            if (out.hasHiggsReverseTarget) {
                out.higgsToRockVisual = grab_transform_telemetry::measureTransformDelta(out.higgsReverseTargetWorld, out.rockVisualTargetWorld);
            }
            out.rawToRockVisualAxes = grab_transform_telemetry::axisAlignmentDots(out.rawHandWorld.rotate, out.rockVisualTargetWorld.rotate);
        }

        if (_hasAdjustedHandTransform) {
            out.adjustedHandWorld = _adjustedHandTransform;
            out.hasAdjustedHandWorld = true;
            out.rawToAdjusted = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.adjustedHandWorld);
        }

        GrabPivotDebugSnapshot pivot{};
        if (getGrabPivotDebugSnapshot(world, pivot)) {
            out.pivotAWorld = pivot.handPivotWorld;
            out.pivotBWorld = pivot.objectPivotWorld;
            const auto pivotDelta = grab_transform_telemetry::measurePointPair(pivot.handPivotWorld, pivot.objectPivotWorld);
            out.pivotDeltaWorld = pivotDelta.delta;
            out.pivotErrorGameUnits = pivotDelta.distance;
        }

        if (_activeConstraint.constraintData) {
            auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
            auto* targetBRca = reinterpret_cast<const float*>(constraintData + ATOM_RAGDOLL_MOT + 0x10);
            auto* transformBRotation = reinterpret_cast<const float*>(constraintData + offsets::kTransformB_Col0);
            auto* transformBTranslation = reinterpret_cast<const float*>(constraintData + offsets::kTransformB_Pos);

            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
            const RE::NiTransform desiredBodyToHandSpace = invertTransform(desiredBodyTransformHandSpace);
            const RE::NiMatrix3 targetAsHkColumns = matrixFromHkColumns(targetBRca);
            const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(targetBRca);
            const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformBRotation);

            out.constraintTransformBLocalGame = RE::NiPoint3{
                transformBTranslation[0] * havokToGameScale(),
                transformBTranslation[1] * havokToGameScale(),
                transformBTranslation[2] * havokToGameScale(),
            };
            out.desiredTransformBLocalGame =
                grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformHandSpace, _grabFrame.pivotAHandBodyLocalGame);
            out.transformBLocalDelta = grab_transform_telemetry::measurePointPair(out.constraintTransformBLocalGame, out.desiredTransformBLocalGame);
            out.targetColumnsToConstraintInverseDegrees = rotationDeltaDegrees(targetAsHkColumns, desiredBodyToHandSpace.rotate);
            out.targetRowsToConstraintInverseDegrees = rotationDeltaDegrees(targetAsHkRows, desiredBodyToHandSpace.rotate);
            out.targetColumnsToConstraintForwardDegrees = rotationDeltaDegrees(targetAsHkColumns, desiredBodyTransformHandSpace.rotate);
            out.targetColumnsToTransformBDegrees = rotationDeltaDegrees(targetAsHkColumns, transformBAsHkColumns);
            out.ragdollMotorEnabled = *(constraintData + ATOM_RAGDOLL_MOT + 0x02) != 0;
            out.heldBodyMass = readBodyMass(world, _savedObjectState.bodyId);
            if (_activeConstraint.angularMotor) {
                out.angularMotorTau = _activeConstraint.angularMotor->tau;
                out.angularMotorDamping = _activeConstraint.angularMotor->damping;
                out.angularMotorMaxForce = (std::max)(std::fabs(_activeConstraint.angularMotor->minForce), std::fabs(_activeConstraint.angularMotor->maxForce));
            }
            if (_activeConstraint.linearMotor) {
                out.linearMotorTau = _activeConstraint.linearMotor->tau;
                out.linearMotorMaxForce = (std::max)(std::fabs(_activeConstraint.linearMotor->minForce), std::fabs(_activeConstraint.linearMotor->maxForce));
            }
            out.hasConstraintAngularTelemetry = true;
        }

        return true;
    }

    bool Hand::startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform)
    {
        /*
         * Long-range pull remains a dynamic-object operation. ROCK promotes the selected
         * object tree through FO4VR's recursive wrappers, scans the resulting dynamic body
         * set, and then applies short-lived predicted velocity to the accepted motions.
         * This mirrors HIGGS' pull behavior without introducing a keyframed object path
         * that would conflict with the dynamic grab constraint used after arrival.
         */
        if (!world || _state != HandState::SelectionLocked || !_currentSelection.isValid() || !_currentSelection.isFarSelection) {
            return false;
        }

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            clearSelectionState(true);
            return false;
        }

        auto* rootNode = selectedRef->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no 3D root", handName());
            clearSelectionState(true);
            return false;
        }

        auto* ownerCell = selectedRef->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: selected ref has no bhkWorld", handName());
            clearSelectionState(true);
            return false;
        }

        std::uint16_t selectedOriginalMotionPropsId = 1;
        auto selectedOriginalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        if (_currentSelection.bodyId.value != INVALID_BODY_ID) {
            if (auto* selectedMotion = havok_runtime::getBodyMotion(world, _currentSelection.bodyId)) {
                selectedOriginalMotionPropsId = *reinterpret_cast<std::uint16_t*>(reinterpret_cast<char*>(selectedMotion) + offsets::kMotion_PropertiesId);
            }
            if (auto* selectedBody = havok_runtime::getBody(world, _currentSelection.bodyId)) {
                selectedOriginalMotionType = physics_body_classifier::motionTypeFromBodyFlags(selectedBody->flags);
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        const bool motionConverted =
            physics_recursive_wrappers::setMotionRecursive(rootNode, physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
        const bool collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);
        auto restoreFailedPullPrep = [&]() {
            if (active_object_prep_policy::shouldRestoreMotionAfterFailedActivePrep(motionConverted, selectedOriginalMotionPropsId)) {
                physics_recursive_wrappers::setMotionRecursive(
                    rootNode,
                    motionPresetFromMotionType(selectedOriginalMotionType, selectedOriginalMotionPropsId),
                    true,
                    true,
                    false);
            }
        };
        const auto preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, selectedRef, scanOptions);
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);
        const RE::NiPoint3 primaryChoiceTarget = _currentSelection.hasHitPoint ? _currentSelection.hitPointWorld : grabPivotAForPrimaryChoice;
        const auto primaryChoice = preparedBodySet.choosePrimaryBody(_currentSelection.bodyId.value, object_physics_body_set::PurePoint3{ primaryChoiceTarget });

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand PULL failed: no accepted dynamic body after recursive prep formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
                "setMotion={} enableCollision={}",
                handName(),
                selectedRef->GetFormID(),
                beforePrepBodySet.records.size(),
                preparedBodySet.records.size(),
                preparedBodySet.acceptedCount(),
                preparedBodySet.rejectedCount(),
                motionConverted ? "ok" : "failed",
                collisionEnabled ? "ok" : "failed");
            restoreFailedPullPrep();
            clearSelectionState(true);
            return false;
        }

        _pulledPrimaryBodyId = primaryChoice.bodyId;
        _pulledBodyIds = preparedBodySet.uniqueAcceptedMotionBodyIds();
        if (_pulledBodyIds.empty()) {
            _pulledBodyIds.push_back(_pulledPrimaryBodyId);
        }

        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        _currentSelection.bodyId = RE::hknpBodyId{ _pulledPrimaryBodyId };
        if (!_currentSelection.visualNode) {
            _currentSelection.visualNode = rootNode;
        }
        if (!_currentSelection.hitNode) {
            _currentSelection.hitNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_WARN(Hand, "{} hand PULL failed: primary dynamic body has no motion bodyId={}", handName(), _pulledPrimaryBodyId);
            restoreFailedPullPrep();
            clearPullRuntimeState();
            clearSelectionState(true);
            return false;
        }

        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavok = niPointToHkVector(grabPivotWorld);
        RE::NiTransform primaryBodyWorld{};
        const bool hasPrimaryBodyWorld = tryGetBodyWorldTransform(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, primaryBodyWorld);
        RE::NiPoint3 ownerNodePosition{};
        bool hasOwnerNode = false;
        if (auto* ownerNode = getOwnerNodeFromBody(world, RE::hknpBodyId{ _pulledPrimaryBodyId })) {
            ownerNodePosition = ownerNode->world.translate;
            hasOwnerNode = true;
        }
        const auto selectedPointAnchor = body_frame::chooseSelectionDistanceAnchor(_currentSelection.hasHitPoint,
            _currentSelection.hitPointWorld,
            hasPrimaryBodyWorld,
            primaryBodyWorld.translate,
            hasOwnerNode,
            ownerNodePosition,
            true,
            hkVectorToNiPoint(motion->position),
            grabPivotWorld);
        const auto selectedPointWorld = selectedPointAnchor.position;
        const auto selectedPointHavok = niPointToHkVector(selectedPointWorld);
        _pullPointOffsetHavok = RE::NiPoint3{
            selectedPointHavok.x - motion->position.x,
            selectedPointHavok.y - motion->position.y,
            selectedPointHavok.z - motion->position.z,
        };

        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };
        const RE::NiPoint3 grabPivotPointHavok{ grabPivotHavok.x, grabPivotHavok.y, grabPivotHavok.z };
        const float pullDistance = (grabPivotPointHavok - objectPointHavok).Length();

        _pullElapsedSeconds = 0.0f;
        _pullDurationSeconds = pull_motion_math::computePullDurationSeconds(pullDistance, g_rockConfig.rockPullDurationA, g_rockConfig.rockPullDurationB, g_rockConfig.rockPullDurationC);
        _pullTargetHavok = {};
        _pullHasTarget = false;
        const auto transition = applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::BeginPull });
        if (!transition.accepted) {
            clearPullRuntimeState();
            restoreFailedPullPrep();
            return false;
        }
        stopSelectionHighlight();

        ROCK_LOG_INFO(Hand,
            "{} hand PULL start: formID={:08X} primaryBody={} bodyCount={} distanceHk={:.3f} duration={:.3f}s setMotion={} enableCollision={}",
            handName(),
            selectedRef->GetFormID(),
            _pulledPrimaryBodyId,
            _pulledBodyIds.size(),
            pullDistance,
            _pullDurationSeconds,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed");
        return true;
    }

    bool Hand::updateDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float deltaTime)
    {
        if (_state != HandState::Pulled || !world || _pulledPrimaryBodyId == INVALID_BODY_ID || !_currentSelection.isValid()) {
            return false;
        }

        auto* selectedRef = _currentSelection.refr;
        if (!selectedRef || selectedRef->IsDeleted() || selectedRef->IsDisabled()) {
            clearSelectionState(true);
            return false;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            clearSelectionState(true);
            return false;
        }

        const auto grabPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
        const auto grabPivotHavokVector = niPointToHkVector(grabPivotWorld);
        const RE::NiPoint3 grabPivotHavok{ grabPivotHavokVector.x, grabPivotHavokVector.y, grabPivotHavokVector.z };
        const RE::NiPoint3 objectPointHavok{
            motion->position.x + _pullPointOffsetHavok.x,
            motion->position.y + _pullPointOffsetHavok.y,
            motion->position.z + _pullPointOffsetHavok.z,
        };

        const float distanceGameUnits = (grabPivotHavok - objectPointHavok).Length() * havokToGameScale();
        _currentSelection.distance = distanceGameUnits;
        if (distanceGameUnits <= g_rockConfig.rockPullAutoGrabDistanceGameUnits) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = RE::NiPoint3{
                objectPointHavok.x * havokToGameScale(),
                objectPointHavok.y * havokToGameScale(),
                objectPointHavok.z * havokToGameScale(),
            };
            _currentSelection.hasHitPoint = true;
            clearPullRuntimeState();
            applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::PullArrivedClose });
            ROCK_LOG_DEBUG(Hand, "{} hand PULL arrived -> close grab window dist={:.1f}", handName(), distanceGameUnits);
            return true;
        }

        const auto motionResult = pull_motion_math::computePullMotion<RE::NiPoint3>(
            pull_motion_math::PullMotionInput<RE::NiPoint3>{
                .handHavok = grabPivotHavok,
                .objectPointHavok = objectPointHavok,
                .previousTargetHavok = _pullTargetHavok,
                .elapsedSeconds = _pullElapsedSeconds,
                .durationSeconds = _pullDurationSeconds,
                .applyVelocitySeconds = g_rockConfig.rockPullApplyVelocityTime,
                .trackHandSeconds = g_rockConfig.rockPullTrackHandTime,
                .destinationOffsetHavok = g_rockConfig.rockPullDestinationZOffsetHavok,
                .maxVelocityHavok = g_rockConfig.rockPullMaxVelocityHavok,
                .hasPreviousTarget = _pullHasTarget,
            });

        _pullElapsedSeconds += (std::max)(0.0f, deltaTime);
        if (motionResult.refreshTarget) {
            _pullTargetHavok = motionResult.targetHavok;
            _pullHasTarget = true;
        }

        if (motionResult.expired || !motionResult.applyVelocity) {
            ROCK_LOG_DEBUG(Hand, "{} hand PULL expired before arrival dist={:.1f}", handName(), distanceGameUnits);
            clearSelectionState(true);
            return false;
        }

        setHeldLinearVelocity(world, RE::hknpBodyId{ _pulledPrimaryBodyId }, _pulledBodyIds, motionResult.velocityHavok,
            pull_motion_math::angularVelocityKeepForDamping(g_rockConfig.rockPulledAngularDamping, deltaTime));
        for (const auto bodyId : _pulledBodyIds) {
            physics_recursive_wrappers::activateBody(world, bodyId);
        }

        return false;
    }

    bool Hand::grabSelectedObject(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform, float tau, float damping, float maxForce, float proportionalRecovery,
        float constantRecovery)
    {
        if (!hasSelection() || !world)
            return false;
        if (!hasCollisionBody())
            return false;

        const auto& sel = _currentSelection;
        if (!sel.refr || sel.bodyId.value == 0x7FFF'FFFF)
            return false;
        if (sel.refr->IsDeleted() || sel.refr->IsDisabled())
            return false;

        auto objectBodyId = sel.bodyId;
        auto* rootNode = sel.refr->Get3D();
        if (!rootNode) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no 3D root", handName());
            return false;
        }

        auto* ownerCell = sel.refr->GetParentCell();
        auto* bhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
        if (!bhkWorld) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected ref has no bhkWorld for object-tree scan", handName());
            return false;
        }

        auto* body = havok_runtime::getBody(world, objectBodyId);
        if (!body) {
            ROCK_LOG_WARN(Hand, "{} hand GRAB failed: selected body no longer readable bodyId={}", handName(), objectBodyId.value);
            return false;
        }

        auto* baseObj = sel.refr->GetObjectReference();
        std::string objName = "(unnamed)";
        if (baseObj) {
            auto nameView = RE::TESFullName::GetFullName(*baseObj, false);
            if (!nameView.empty())
                objName = std::string(nameView);
        }

        const char* motionTypeStr = "UNKNOWN";
        std::uint16_t selectedOriginalMotionPropsId = 1;
        {
            auto* objMotion = havok_runtime::getBodyMotion(world, objectBodyId);
            auto* bodyPtr = reinterpret_cast<char*>(body);
            if (objMotion) {
                auto* motionPtr = reinterpret_cast<char*>(objMotion);

                std::uint32_t bodyFlags = *reinterpret_cast<std::uint32_t*>(bodyPtr + 0x40);
                std::uint8_t bodyMotionPropsId = *reinterpret_cast<std::uint8_t*>(bodyPtr + 0x72);
                std::uint16_t motionPropsId = *reinterpret_cast<std::uint16_t*>(motionPtr + offsets::kMotion_PropertiesId);
                selectedOriginalMotionPropsId = motionPropsId;
                std::uint16_t maxLinVelPacked = *reinterpret_cast<std::uint16_t*>(motionPtr + 0x3A);
                std::uint16_t maxAngVelPacked = *reinterpret_cast<std::uint16_t*>(motionPtr + 0x3C);

                std::uint32_t linVelU32 = static_cast<std::uint32_t>(maxLinVelPacked) << 16;
                std::uint32_t angVelU32 = static_cast<std::uint32_t>(maxAngVelPacked) << 16;
                float maxLinVel, maxAngVel;
                std::memcpy(&maxLinVel, &linVelU32, sizeof(float));
                std::memcpy(&maxAngVel, &angVelU32, sizeof(float));

                bool isDynamicInit = (bodyFlags & 0x2) != 0;
                bool isKeyframed = (bodyFlags & 0x4) != 0;

                switch (motionPropsId & 0xFF) {
                case 0:
                    motionTypeStr = "STATIC";
                    break;
                case 1:
                    motionTypeStr = "DYNAMIC";
                    break;
                case 2:
                    motionTypeStr = "KEYFRAMED";
                    break;
                default:
                    motionTypeStr = "OTHER";
                    break;
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand GRAB MOTION: body={} motionPropsId={} ({}) "
                    "bodyFlags=0x{:08X} dynInit={} keyfr={} bodyPropsId={} "
                    "maxLinVel={:.1f} maxAngVel={:.1f}",
                    handName(), objectBodyId.value, motionPropsId, motionTypeStr, bodyFlags, isDynamicInit, isKeyframed, bodyMotionPropsId, maxLinVel, maxAngVel);

                {
                    static bool libraryDumped = false;
                    if (!libraryDumped) {
                        libraryDumped = true;
                        auto* worldPtr = reinterpret_cast<char*>(world);
                        auto* libraryPtr = *reinterpret_cast<char**>(worldPtr + 0x5D0);
                        if (libraryPtr) {
                            auto* arrayBase = *reinterpret_cast<char**>(libraryPtr + 0x28);

                            int arraySize = *reinterpret_cast<int*>(libraryPtr + 0x30);
                            ROCK_LOG_TRACE(Hand, "Motion properties library: {} entries, stride=0x40", arraySize);
                            int dumpCount = (arraySize < 16) ? arraySize : 16;
                            if (arrayBase) {
                                for (int i = 0; i < dumpCount; i++) {
                                    auto* entry = arrayBase + i * 0x40;

                                    auto* f = reinterpret_cast<float*>(entry);
                                    auto* u = reinterpret_cast<std::uint32_t*>(entry);
                                    ROCK_LOG_TRACE(Hand,
                                        "  [{}] +00: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                        "+10: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                        "+20: {:.4f} {:.4f} {:.4f} {:.4f} | "
                                        "+30: {:.4f} {:.4f} {:.4f} {:.4f}",
                                        i, f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15]);
                                    ROCK_LOG_TRACE(Hand,
                                        "  [{}] hex: {:08X} {:08X} {:08X} {:08X} | "
                                        "{:08X} {:08X} {:08X} {:08X}",
                                        i, u[0], u[1], u[2], u[3], u[4], u[5], u[6], u[7]);
                                }
                            }
                        }
                    }
                }
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, sel.refr, scanOptions);
        active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle;
        activeLifecycle.captureBeforeActivePrep(beforePrepBodySet);

        const bool motionConverted =
            physics_recursive_wrappers::setMotionRecursive(rootNode, physics_recursive_wrappers::MotionPreset::Dynamic, true, true, true);
        const bool collisionEnabled = physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);

        auto preparedBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, sel.refr, scanOptions);
        activeLifecycle.markPreparedBodies(preparedBodySet);
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);

        auto restoreFailedGrabPrep = [&]() {
            restoreActiveGrabLifecycle(world,
                activeLifecycle,
                activeLifecycle.restorePlanForFailure(),
                objectBodyId.value,
                handName(),
                "failed-setup");
        };

        ROCK_LOG_DEBUG(Hand,
            "{} hand object-tree prep: ref='{}' formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "setMotion={} enableCollision={}",
            handName(),
            objName,
            sel.refr->GetFormID(),
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed");

        auto* collidableNode = sel.hitNode ? sel.hitNode : rootNode;
        auto* meshSourceNode = sel.visualNode ? sel.visualNode : rootNode;
        if (!meshSourceNode) {
            meshSourceNode = collidableNode;
        }
        RE::NiTransform objectWorldTransform;
        if (collidableNode) {
            objectWorldTransform = collidableNode->world;
        } else {
            objectWorldTransform = handWorldTransform;
        }

        RE::NiPoint3 grabSurfacePoint = sel.hasHitPoint ? sel.hitPointWorld : objectWorldTransform.translate;
        float selectionToMeshDistanceGameUnits = 0.0f;
        bool meshGrabFound = false;
        MeshExtractionStats meshStats;
        std::vector<TriangleData> grabMeshTriangles;
        std::vector<GrabSurfaceTriangleData> grabSurfaceTriangles;
        GrabSurfaceHit grabSurfaceHit{};
        RuntimeGrabContactPatch contactPatchRuntime{};
        RuntimeMultiFingerGripContact multiFingerGripRuntime{};
        bool contactPatchUsed = false;
        bool multiFingerGripUsed = false;
        grab_visual_authority_policy::VisualAuthorityContactState visualAuthorityContact{};
        RE::NiAVObject* surfaceOwnerNode = nullptr;
        RE::NiAVObject* authoredGrabNode = nullptr;
        const bool meshContactOnly = g_rockConfig.rockGrabMeshContactOnly;
        const char* grabPointMode = sel.hasHitPoint ? "selectionHitPointFallback" : "objectNodeOriginFallback";
        const char* grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";

        /*
         * hknp selection identifies the object/body. In mesh-authoritative mode
         * it is logged as collision evidence only; the grabbed point and frame
         * must come from visual geometry or an authored ROCK grab node.
         */
        if (sel.hasHitPoint && sel.hasHitNormal) {
            const auto collisionSurfaceHit = makeCollisionQueryGrabSurfaceHit(sel, collidableNode);
            if (collisionSurfaceHit.valid) {
                if (!meshContactOnly) {
                    grabSurfaceHit = collisionSurfaceHit;
                    grabSurfacePoint = grabSurfaceHit.position;
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "collisionSurface";
                    grabFallbackReason = "none";
                }
                ROCK_LOG_DEBUG(Hand,
                    "{} hand COLLISION SELECTION HIT: body={} activeGrabPoint={} point=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) "
                    "fraction={:.4f} shapeKey=0x{:08X} filter=0x{:08X} owner='{}'",
                    handName(),
                    sel.bodyId.value,
                    meshContactOnly ? "no" : "yes",
                    collisionSurfaceHit.position.x,
                    collisionSurfaceHit.position.y,
                    collisionSurfaceHit.position.z,
                    collisionSurfaceHit.normal.x,
                    collisionSurfaceHit.normal.y,
                    collisionSurfaceHit.normal.z,
                    collisionSurfaceHit.hitFraction,
                    collisionSurfaceHit.shapeKey,
                    collisionSurfaceHit.shapeCollisionFilterInfo,
                    nodeDebugName(collisionSurfaceHit.sourceNode));
            }
        }

        if (meshSourceNode) {
            extractAllSurfaceTriangles(meshSourceNode, grabMeshTriangles, grabSurfaceTriangles, 10, &meshStats);

            ROCK_LOG_DEBUG(Hand,
                "{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} "
                "static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} totalTris={}",
                handName(), nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes, meshStats.staticShapes,
                meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, meshStats.totalTriangles());

            {
                RE::BSTriShape* firstTriShape = meshSourceNode->IsTriShape();
                if (!firstTriShape) {
                    auto* meshNode = meshSourceNode->IsNode();
                    if (meshNode) {
                        auto& kids = meshNode->GetRuntimeData().children;
                        const auto childCount = kids.size();
                        for (auto ci = decltype(childCount){ 0 }; ci < childCount; ci++) {
                            auto* kid = kids[ci].get();
                            if (kid && kid->IsTriShape()) {
                                firstTriShape = kid->IsTriShape();
                                break;
                            }
                        }
                    }
                }
                if (firstTriShape) {
                    auto* tsBase = reinterpret_cast<char*>(firstTriShape);
                    std::uint64_t vtxDesc = *reinterpret_cast<std::uint64_t*>(tsBase + VROffset::vertexDesc);
                    std::uint32_t stride = static_cast<std::uint32_t>(vtxDesc & 0xF) * 4;
                    std::uint32_t posOff = static_cast<std::uint32_t>((vtxDesc >> 2) & 0x3C);
                    bool fullPrec = ((vtxDesc >> 54) & 1) != 0;
                    std::uint8_t geomType = *reinterpret_cast<std::uint8_t*>(tsBase + 0x198);
                    void* skinInst = *reinterpret_cast<void**>(tsBase + VROffset::skinInstance);

                    ROCK_LOG_TRACE(MeshGrab, "VertexDiag '{}': vtxDesc=0x{:016X} stride={} posOffset={} fullPrec={} geomType={} skinned={}",
                        firstTriShape->name.c_str() ? firstTriShape->name.c_str() : "(null)", vtxDesc, stride, posOff, fullPrec ? 1 : 0, geomType, skinInst ? 1 : 0);
                }
            }

            if (!grabMeshTriangles.empty()) {
                auto& t0 = grabMeshTriangles[0];
                float cx = (t0.v0.x + t0.v1.x + t0.v2.x) / 3.0f;
                float cy = (t0.v0.y + t0.v1.y + t0.v2.y) / 3.0f;
                float cz = (t0.v0.z + t0.v1.z + t0.v2.z) / 3.0f;

                if (auto* liveBody = havok_runtime::getBody(world, objectBodyId)) {
                    auto* objFloats = reinterpret_cast<float*>(liveBody);
                    const float hkToGameScale = havokToGameScale();
                    float distToBody = std::sqrt((cx - objFloats[12] * hkToGameScale) * (cx - objFloats[12] * hkToGameScale) +
                        (cy - objFloats[13] * hkToGameScale) * (cy - objFloats[13] * hkToGameScale) +
                        (cz - objFloats[14] * hkToGameScale) * (cz - objFloats[14] * hkToGameScale));

                    ROCK_LOG_TRACE(MeshGrab, "TRI[0] centroid=({:.1f},{:.1f},{:.1f}) distToBody={:.1f}gu", cx, cy, cz, distToBody);
                }
            }

            if (g_rockConfig.rockGrabNodeAnchorsEnabled) {
                const std::string_view primaryNodeName = _isLeft ? std::string_view(g_rockConfig.rockGrabNodeNameLeft) : std::string_view(g_rockConfig.rockGrabNodeNameRight);
                const std::string_view fallbackNodeName = grab_node_name_policy::defaultGrabNodeName(_isLeft);
                RE::NiAVObject* grabNode = findNamedNodeRecursive(meshSourceNode, primaryNodeName, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                if (!grabNode && fallbackNodeName != primaryNodeName) {
                    grabNode = findNamedNodeRecursive(meshSourceNode, fallbackNodeName, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                }
                if (grabNode) {
                    authoredGrabNode = grabNode;
                    surfaceOwnerNode = grabNode;
                    grabSurfaceHit = {};
                    grabSurfacePoint = grabNode->world.translate;
                    meshGrabFound = true;
                    grabPointMode = "grabNodeAnchor";
                    grabFallbackReason = "none";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand GRAB NODE: node='{}' point=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        nodeDebugName(grabNode),
                        grabSurfacePoint.x,
                        grabSurfacePoint.y,
                        grabSurfacePoint.z);
                }
            }

            if (!meshGrabFound && !grabSurfaceTriangles.empty()) {
                RE::NiPoint3 grabPivotAWorld = computeGrabPivotAWorld(world, handWorldTransform);
                RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);

                if (findClosestGrabSurfaceHit(grabSurfaceTriangles, grabPivotAWorld, palmDir, g_rockConfig.rockGrabLateralWeight, g_rockConfig.rockGrabDirectionalWeight, grabSurfaceHit)) {
                    grabSurfacePoint = grabSurfaceHit.position;
                    selectionToMeshDistanceGameUnits =
                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabSurfacePoint) : std::numeric_limits<float>::max();
                    grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                    grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "meshSurface";
                    grabFallbackReason = "none";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH GRAB: mode={} tris={} staticTris={} dynamicTris={} skinnedTris={} "
                        "closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' shape='{}' selectionHit={} selectionDelta={:.1f}gu",
                        handName(), grabPointMode, grabMeshTriangles.size(), meshStats.staticTriangles, meshStats.dynamicTriangles, meshStats.skinnedTriangles, grabSurfaceHit.position.x,
                        grabSurfaceHit.position.y, grabSurfaceHit.position.z, grabSurfaceHit.triangleIndex, grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                        nodeDebugName(grabSurfaceHit.sourceNode), nodeDebugName(grabSurfaceHit.sourceShape), sel.hasHitPoint ? "yes" : "no",
                        sel.hasHitPoint ? selectionToMeshDistanceGameUnits : -1.0f);
                } else {
                    grabFallbackReason = "noClosestSurfacePoint";
                }
            } else if (!meshGrabFound) {
                grabFallbackReason = "noTriangles";
            }
        }

        if (!meshGrabFound) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB POINT FALLBACK: mode={} reason={} meshNode='{}' ownerNode='{}' rootNode='{}' "
                "shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} "
                "fallbackPoint=({:.1f},{:.1f},{:.1f})",
                handName(), grabPointMode, grabFallbackReason, nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes,
                meshStats.staticShapes, meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
        }

        const bool hasMeshSurfaceContact =
            meshGrabFound && grabSurfaceHit.valid && grabSurfaceHit.sourceKind != GrabSurfaceSourceKind::CollisionQuery;
        const auto contactSourcePolicy = grab_contact_source_policy::evaluateGrabContactSourcePolicy(
            meshContactOnly,
            g_rockConfig.rockGrabRequireMeshContact,
            hasMeshSurfaceContact,
            authoredGrabNode != nullptr);
        const auto grabContactQualityMode = grab_contact_evidence_policy::sanitizeQualityMode(g_rockConfig.rockGrabContactQualityMode);
        const bool multiFingerEvidenceEnabled =
            g_rockConfig.rockGrabMultiFingerContactValidationEnabled &&
            grabContactQualityMode != grab_contact_evidence_policy::GrabContactQualityMode::LegacyPermissive &&
            !authoredGrabNode &&
            meshContactOnly &&
            g_rockConfig.rockGrabRequireMeshContact;
        const bool hybridFingerProbeEvidenceEnabled =
            multiFingerEvidenceEnabled &&
            grabContactQualityMode == grab_contact_evidence_policy::GrabContactQualityMode::HybridEvidence;
        const bool strictMultiFingerGripRequired =
            multiFingerEvidenceEnabled &&
            grabContactQualityMode == grab_contact_evidence_policy::GrabContactQualityMode::StrictMultiFinger;
        const bool deferMissingMeshContactToPatch =
            contactSourcePolicy.failWithoutMesh &&
            !authoredGrabNode &&
            !grabSurfaceTriangles.empty() &&
            (strictMultiFingerGripRequired ||
                (contactSourcePolicy.allowContactPatchPivot && g_rockConfig.rockGrabContactPatchEnabled));
        if (contactSourcePolicy.failWithoutMesh && !deferMissingMeshContactToPatch) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: mesh contact required for '{}' formID={:08X}; collision point was not used as pivot "
                "meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} totalTris={} reason={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode),
                meshStats.visitedShapes,
                meshStats.totalTriangles(),
                contactSourcePolicy.reason);
            restoreFailedGrabPrep();
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        const RE::NiPoint3 primaryChoiceTarget = meshGrabFound ? grabSurfacePoint : (sel.hasHitPoint ? sel.hitPointWorld : grabPivotAForPrimaryChoice);
        const auto primaryChoice =
            preparedBodySet.choosePrimaryBodyWithSurfaceOwner(sel.bodyId.value, surfaceOwnerNode, object_physics_body_set::PurePoint3{ primaryChoiceTarget });
        bool surfaceOwnerMatchesResolvedBody = true;
        if (grabSurfaceHit.valid) {
            if (grabSurfaceHit.sourceKind == GrabSurfaceSourceKind::CollisionQuery) {
                surfaceOwnerMatchesResolvedBody = primaryChoice.bodyId == sel.bodyId.value;
            } else {
                const auto* surfaceOwnerRecord = preparedBodySet.findAcceptedRecordByOwnerNode(surfaceOwnerNode);
                surfaceOwnerMatchesResolvedBody = surfaceOwnerRecord && surfaceOwnerRecord->bodyId == primaryChoice.bodyId;
            }
            grabSurfaceHit.resolvedOwnerMatchesBody = surfaceOwnerMatchesResolvedBody;
        }
        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB BODY RESOLUTION: selectedBody={} resolvedBody={} reason={} sourceNode='{}' sourceKind={} ownerMatch={} target=({:.1f},{:.1f},{:.1f})",
            handName(),
            sel.bodyId.value,
            primaryChoice.bodyId,
            primaryBodyChoiceReasonName(primaryChoice.reason),
            nodeDebugName(surfaceOwnerNode),
            grabSurfaceHit.valid ? grabSurfaceSourceKindName(grabSurfaceHit.sourceKind) : (authoredGrabNode ? "authoredNode" : "fallback"),
            surfaceOwnerMatchesResolvedBody ? "yes" : "no",
            primaryChoiceTarget.x,
            primaryChoiceTarget.y,
            primaryChoiceTarget.z);

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: no accepted dynamic body after recursive prep for '{}' formID={:08X} visitedNodes={} collisionObjects={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                preparedBodySet.diagnostics.visitedNodes,
                preparedBodySet.diagnostics.collisionObjects);
            restoreFailedGrabPrep();
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        if (grab_contact_source_policy::shouldRejectMeshOwnerMismatch(meshContactOnly,
                g_rockConfig.rockGrabRequireMeshContact,
                hasMeshSurfaceContact,
                authoredGrabNode != nullptr,
                surfaceOwnerMatchesResolvedBody)) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: mesh contact owner does not match resolved body for '{}' formID={:08X}; "
                "selectedBody={} resolvedBody={} sourceNode='{}' sourceKind={} target=({:.1f},{:.1f},{:.1f})",
                handName(),
                objName,
                sel.refr->GetFormID(),
                sel.bodyId.value,
                primaryChoice.bodyId,
                nodeDebugName(surfaceOwnerNode),
                grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                primaryChoiceTarget.x,
                primaryChoiceTarget.y,
                primaryChoiceTarget.z);
            restoreFailedGrabPrep();
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        objectBodyId = RE::hknpBodyId{ primaryChoice.bodyId };
        auto& preparedBody = world->GetBody(objectBodyId);
        _savedObjectState.bodyId = objectBodyId;
        _savedObjectState.refr = sel.refr;
        _savedObjectState.originalFilterInfo = *reinterpret_cast<std::uint32_t*>(reinterpret_cast<char*>(&preparedBody) + offsets::kBody_CollisionFilterInfo);
        _savedObjectState.originalMotionPropsId = selectedOriginalMotionPropsId;
        if (const auto* originalPrimaryRecord = beforePrepBodySet.findRecord(objectBodyId.value)) {
            _savedObjectState.originalMotionPropsId = motionPropsIdFromRecordMotionType(originalPrimaryRecord->motionType, selectedOriginalMotionPropsId);
        }

        {
            auto* ownerCellAtResolution = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtResolution = ownerCellAtResolution ? ownerCellAtResolution->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtResolution = bhkWorldAtResolution ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtResolution, objectBodyId) : nullptr;
            if (auto* resolvedOwnerNode = bodyCollisionObjectAtResolution ? bodyCollisionObjectAtResolution->sceneObject : nullptr) {
                collidableNode = resolvedOwnerNode;
                objectWorldTransform = collidableNode->world;
            }
        }

        const auto semanticContacts = collectFreshSemanticContactsForBody(
            objectBodyId.value,
            static_cast<std::uint32_t>(g_rockConfig.rockGrabOppositionContactMaxAgeFrames));

        hand_semantic_contact_state::SemanticContactRecord semanticContact{};
        const bool hasSemanticContact = getLastSemanticContact(semanticContact);
        const auto semanticDecision = hand_semantic_contact_state::evaluateSemanticPivotCandidate(
            g_rockConfig.rockGrabUseSemanticFingerContactPivot,
            semanticContact,
            objectBodyId.value,
            static_cast<std::uint32_t>(g_rockConfig.rockGrabOppositionContactMaxAgeFrames));
        if (hasSemanticContact && !strictMultiFingerGripRequired && !authoredGrabNode && !grabSurfaceTriangles.empty()) {
            if (semanticDecision.accept) {
                RE::NiTransform semanticSourceWorld{};
                if (tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ semanticContact.handBodyId }, semanticSourceWorld)) {
                    const RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);
                    GrabSurfaceHit semanticMeshHit{};
                    if (findClosestGrabSurfaceHit(
                            grabSurfaceTriangles,
                            semanticSourceWorld.translate,
                            palmDir,
                            g_rockConfig.rockGrabLateralWeight,
                            g_rockConfig.rockGrabDirectionalWeight,
                            semanticMeshHit)) {
                        grabSurfaceHit = semanticMeshHit;
                        grabSurfacePoint = semanticMeshHit.position;
                        surfaceOwnerNode = semanticMeshHit.sourceNode;
                        meshGrabFound = true;
                        grabPointMode = "semanticHandMeshSurface";
                        grabFallbackReason = "none";
                        selectionToMeshDistanceGameUnits =
                            sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabSurfacePoint) : std::numeric_limits<float>::max();
                        grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                        grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand SEMANTIC GRAB SURFACE: role={} sourceBody={} targetBody={} point=({:.1f},{:.1f},{:.1f}) "
                            "source=({:.1f},{:.1f},{:.1f}) frames={} tri={} owner='{}'",
                            handName(),
                            hand_collider_semantics::roleName(semanticContact.role),
                            semanticContact.handBodyId,
                            objectBodyId.value,
                            grabSurfacePoint.x,
                            grabSurfacePoint.y,
                            grabSurfacePoint.z,
                            semanticSourceWorld.translate.x,
                            semanticSourceWorld.translate.y,
                            semanticSourceWorld.translate.z,
                            semanticContact.framesSinceContact,
                            grabSurfaceHit.triangleIndex,
                            nodeDebugName(grabSurfaceHit.sourceNode));
                    }
                }
            } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand semantic contact ignored for grab pivot: reason={} contactBody={} targetBody={} role={} frames={}",
                    handName(),
                    semanticDecision.reason,
                    semanticContact.otherBodyId,
                    objectBodyId.value,
                    hand_collider_semantics::roleName(semanticContact.role),
                    semanticContact.framesSinceContact);
            }
        }

        if (contactSourcePolicy.allowContactPatchPivot && g_rockConfig.rockGrabContactPatchEnabled && !authoredGrabNode && !sel.isFarSelection) {
            const RE::NiPoint3 grabPivotAWorld = computeGrabPivotAWorld(world, handWorldTransform);
            const RE::NiPoint3 palmNormalWorld = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);
            const RE::NiPoint3 palmTangentWorld = transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft);
            const RE::NiPoint3 palmBitangentWorld = transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, _isLeft);
            contactPatchRuntime = buildRuntimeGrabContactPatch(world,
                preparedBodySet,
                objectBodyId.value,
                sel,
                grabPivotAWorld,
                palmNormalWorld,
                palmTangentWorld,
                palmBitangentWorld,
                grabSurfaceTriangles);

            const bool contactPatchAcceptedForPivot = grab_contact_source_policy::shouldAcceptContactPatchPivot(
                contactSourcePolicy,
                contactPatchRuntime.patch.valid,
                contactPatchRuntime.meshSnapped);
            if (contactPatchAcceptedForPivot) {
                contactPatchUsed = true;
                grabSurfacePoint = contactPatchRuntime.pivotDecision.point;
                grabSurfaceHit.position = contactPatchRuntime.pivotDecision.point;
                grabSurfaceHit.normal = contactPatchRuntime.patch.normal;
                grabSurfaceHit.triangleIndex = contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.triangleIndex : -1;
                grabSurfaceHit.distance = 0.0f;
                grabSurfaceHit.sourceNode = contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.sourceNode : collidableNode;
                grabSurfaceHit.sourceShape = contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.sourceShape : nullptr;
                grabSurfaceHit.triangle = contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.triangle : TriangleData{};
                grabSurfaceHit.sourceKind = contactPatchRuntime.meshSnapped ? contactPatchRuntime.meshSnapHit.sourceKind : GrabSurfaceSourceKind::CollisionQuery;
                grabSurfaceHit.shapeKey = sel.hitShapeKey;
                grabSurfaceHit.shapeCollisionFilterInfo = sel.hitShapeCollisionFilterInfo;
                grabSurfaceHit.hitFraction = sel.hitFraction;
                grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabSurfacePoint);
                grabSurfaceHit.selectionToMeshDistanceGameUnits = contactPatchRuntime.pivotDecision.selectionDeltaGameUnits;
                grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                grabSurfaceHit.resolvedOwnerMatchesBody = true;
                grabSurfaceHit.hasTriangle = contactPatchRuntime.meshSnapped && contactPatchRuntime.meshSnapHit.hasTriangle;
                grabSurfaceHit.hasShapeKey = sel.hasHitShapeKey;
                grabSurfaceHit.valid = true;
                surfaceOwnerNode = grabSurfaceHit.sourceNode;
                selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                surfaceOwnerMatchesResolvedBody = true;
                meshGrabFound = true;
                grabPointMode = contactPatchRuntime.pointMode;
                grabFallbackReason = contactPatchRuntime.pivotDecision.reason ? contactPatchRuntime.pivotDecision.reason :
                    (contactPatchRuntime.patch.fallbackReason ? contactPatchRuntime.patch.fallbackReason : "none");
                if (contactPatchRuntime.meshSnapped) {
                    grab_visual_authority_policy::markValidatedContact(visualAuthorityContact, "contactPatchMeshSnap");
                }

                ROCK_LOG_DEBUG(Hand,
                    "{} hand CONTACT PATCH: body={} mode={} samples={}/{} castsHits={} rejectBody={} rejectNormal={} "
                    "pivot=({:.1f},{:.1f},{:.1f}) pivotSource={} replaceSelected={} selectionDelta={:.2f} "
                    "fitPoint=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) tangent=({:.3f},{:.3f},{:.3f}) "
                    "confidence={:.2f} reliable={} meshSnap={} snapDelta={:.2f} fallback={}",
                    handName(),
                    objectBodyId.value,
                    contactPatchRuntime.pointMode,
                    contactPatchRuntime.patch.hitCount,
                    contactPatchRuntime.sampleCount,
                    contactPatchRuntime.castHitCount,
                    contactPatchRuntime.rejectedBodyHits,
                    contactPatchRuntime.rejectedInvalidNormals,
                    grabSurfacePoint.x,
                    grabSurfacePoint.y,
                    grabSurfacePoint.z,
                    grab_contact_patch_math::pivotSourceName(contactPatchRuntime.pivotDecision.source),
                    contactPatchRuntime.pivotDecision.replaceSelectedPoint ? "yes" : "no",
                    contactPatchRuntime.pivotDecision.selectionDeltaGameUnits,
                    contactPatchRuntime.patch.contactPoint.x,
                    contactPatchRuntime.patch.contactPoint.y,
                    contactPatchRuntime.patch.contactPoint.z,
                    contactPatchRuntime.patch.normal.x,
                    contactPatchRuntime.patch.normal.y,
                    contactPatchRuntime.patch.normal.z,
                    contactPatchRuntime.patch.tangent.x,
                    contactPatchRuntime.patch.tangent.y,
                    contactPatchRuntime.patch.tangent.z,
                    contactPatchRuntime.patch.confidence,
                    contactPatchRuntime.patch.orientationReliable ? "yes" : "no",
                    contactPatchRuntime.meshSnapped ? "yes" : "no",
                    contactPatchRuntime.patch.meshSnapDeltaGameUnits,
                    grabFallbackReason);
            } else if (contactPatchRuntime.patch.valid && g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand CONTACT PATCH rejected for pivot: body={} mode={} meshSnap={} requireMeshSnap={} fallback={}",
                    handName(),
                    objectBodyId.value,
                    contactPatchRuntime.pointMode,
                    contactPatchRuntime.meshSnapped ? "yes" : "no",
                    contactSourcePolicy.requireContactPatchMeshSnap ? "yes" : "no",
                    contactSourcePolicy.requireContactPatchMeshSnap ? "meshSnapRequired" : "contactPatchRejected");
            } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand CONTACT PATCH failed: body={} samples={} castsHits={} rejectBody={} rejectNormal={} fallback={}",
                    handName(),
                    objectBodyId.value,
                    contactPatchRuntime.sampleCount,
                    contactPatchRuntime.castHitCount,
                    contactPatchRuntime.rejectedBodyHits,
                    contactPatchRuntime.rejectedInvalidNormals,
                    contactPatchRuntime.patch.fallbackReason ? contactPatchRuntime.patch.fallbackReason : "unknown");
            }
        }

        if (multiFingerEvidenceEnabled) {
            multiFingerGripRuntime = buildRuntimeMultiFingerGripContact(world,
                preparedBodySet,
                objectBodyId.value,
                objectWorldTransform,
                semanticContacts,
                grabSurfaceTriangles,
                this,
                hybridFingerProbeEvidenceEnabled);
            if (multiFingerGripRuntime.gripSet.valid) {
                multiFingerGripUsed = true;
                const auto& gripSet = multiFingerGripRuntime.gripSet;
                GrabSurfaceHit representativeHit{};
                for (const auto& group : gripSet.groups) {
                    if (!group.valid) {
                        continue;
                    }
                    const int index = grab_multi_finger_contact_math::fingerIndex(group.finger);
                    if (index >= 0 && static_cast<std::size_t>(index) < multiFingerGripRuntime.groupHits.size() &&
                        multiFingerGripRuntime.groupHits[static_cast<std::size_t>(index)].valid) {
                        representativeHit = multiFingerGripRuntime.groupHits[static_cast<std::size_t>(index)];
                        break;
                    }
                }

                grabSurfacePoint = gripSet.contactCenterWorld;
                grabSurfaceHit = representativeHit;
                grabSurfaceHit.position = gripSet.contactCenterWorld;
                grabSurfaceHit.normal = gripSet.averageNormalWorld;
                grabSurfaceHit.distance = 0.0f;
                grabSurfaceHit.hasTriangle = representativeHit.valid && representativeHit.hasTriangle;
                grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                grabSurfaceHit.selectionToMeshDistanceGameUnits =
                    sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabSurfacePoint) : std::numeric_limits<float>::max();
                grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(gripSet.handCenterWorld, grabSurfacePoint);
                grabSurfaceHit.resolvedOwnerMatchesBody = true;
                grabSurfaceHit.valid = true;
                surfaceOwnerNode = representativeHit.sourceNode ? representativeHit.sourceNode : collidableNode;
                surfaceOwnerMatchesResolvedBody = true;
                selectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                meshGrabFound = true;
                grabPointMode = "multiFingerContactPatch";
                grabFallbackReason = gripSet.reason;
                grab_visual_authority_policy::markValidatedContact(visualAuthorityContact, "multiFingerContactPatch");

                ROCK_LOG_DEBUG(Hand,
                    "{} hand MULTI-FINGER GRIP: body={} groups={} semanticGroups={} probeGroups={} candidates={} meshHits={} "
                    "semanticHits={} probeHits={} rejectOwner={} rejectDistance={} "
                    "handCenter=({:.1f},{:.1f},{:.1f}) contactCenter=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) "
                    "spread={:.2f} reason={}",
                    handName(),
                    objectBodyId.value,
                    gripSet.groupCount,
                    multiFingerGripRuntime.semanticGroupCount,
                    multiFingerGripRuntime.liveProbeGroupCount,
                    multiFingerGripRuntime.candidateContactCount,
                    multiFingerGripRuntime.meshHitCount,
                    multiFingerGripRuntime.semanticMeshHitCount,
                    multiFingerGripRuntime.liveProbeMeshHitCount,
                    multiFingerGripRuntime.rejectedOwnerCount,
                    multiFingerGripRuntime.rejectedDistanceCount,
                    gripSet.handCenterWorld.x,
                    gripSet.handCenterWorld.y,
                    gripSet.handCenterWorld.z,
                    gripSet.contactCenterWorld.x,
                    gripSet.contactCenterWorld.y,
                    gripSet.contactCenterWorld.z,
                    gripSet.averageNormalWorld.x,
                    gripSet.averageNormalWorld.y,
                    gripSet.averageNormalWorld.z,
                    gripSet.spreadGameUnits,
                    gripSet.reason);
            } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand MULTI-FINGER GRIP rejected: body={} groups={} semanticGroups={} probeGroups={} candidates={} meshHits={} "
                    "semanticHits={} probeHits={} rejectOwner={} rejectDistance={} reason={}",
                    handName(),
                    objectBodyId.value,
                    multiFingerGripRuntime.gripSet.groupCount,
                    multiFingerGripRuntime.semanticGroupCount,
                    multiFingerGripRuntime.liveProbeGroupCount,
                    multiFingerGripRuntime.candidateContactCount,
                    multiFingerGripRuntime.meshHitCount,
                    multiFingerGripRuntime.semanticMeshHitCount,
                    multiFingerGripRuntime.liveProbeMeshHitCount,
                    multiFingerGripRuntime.rejectedOwnerCount,
                    multiFingerGripRuntime.rejectedDistanceCount,
                    multiFingerGripRuntime.reason ? multiFingerGripRuntime.reason : "unknown");
            }
        }

        grab_contact_evidence_policy::GrabContactEvidenceDecision contactEvidenceDecision{};
        if (multiFingerEvidenceEnabled) {
            grab_contact_evidence_policy::GrabContactEvidenceInput evidenceInput{};
            evidenceInput.qualityMode = static_cast<int>(grabContactQualityMode);
            evidenceInput.multiFingerValidationEnabled = g_rockConfig.rockGrabMultiFingerContactValidationEnabled;
            evidenceInput.contactPatchAccepted = contactPatchUsed;
            evidenceInput.contactPatchMeshSnapped = contactPatchRuntime.meshSnapped;
            evidenceInput.contactPatchReliable = contactPatchRuntime.patch.orientationReliable;
            evidenceInput.contactPatchConfidence = contactPatchRuntime.patch.confidence;
            evidenceInput.multiFingerGripValid = multiFingerGripRuntime.gripSet.valid;
            evidenceInput.semanticFingerGroups = multiFingerGripRuntime.semanticGroupCount;
            evidenceInput.probeFingerGroups = multiFingerGripRuntime.liveProbeGroupCount;
            evidenceInput.combinedFingerGroups = multiFingerGripRuntime.gripSet.groupCount;
            evidenceInput.minimumFingerGroups =
                static_cast<std::uint32_t>((std::max)(1, g_rockConfig.rockGrabMinFingerContactGroups));
            contactEvidenceDecision = grab_contact_evidence_policy::evaluateGrabContactEvidence(evidenceInput);

            ROCK_LOG_DEBUG(Hand,
                "{} hand CONTACT EVIDENCE: mode={} accept={} level={} reason={} patch={} meshSnap={} reliable={} confidence={:.2f} "
                "multiFingerValid={} semanticGroups={} probeGroups={} combinedGroups={} minGroups={} useMultiFingerPivot={}",
                handName(),
                grab_contact_evidence_policy::contactQualityModeName(grabContactQualityMode),
                contactEvidenceDecision.accept ? "yes" : "no",
                grab_contact_evidence_policy::contactEvidenceLevelName(contactEvidenceDecision.level),
                contactEvidenceDecision.reason,
                contactPatchUsed ? "yes" : "no",
                contactPatchRuntime.meshSnapped ? "yes" : "no",
                contactPatchRuntime.patch.orientationReliable ? "yes" : "no",
                contactPatchRuntime.patch.confidence,
                multiFingerGripRuntime.gripSet.valid ? "yes" : "no",
                multiFingerGripRuntime.semanticGroupCount,
                multiFingerGripRuntime.liveProbeGroupCount,
                multiFingerGripRuntime.gripSet.groupCount,
                g_rockConfig.rockGrabMinFingerContactGroups,
                contactEvidenceDecision.useMultiFingerPivot ? "yes" : "no");
        }

        if (multiFingerEvidenceEnabled && !contactEvidenceDecision.accept) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: contact evidence rejected '{}' formID={:08X}; "
                "mode={} level={} groups={} semanticGroups={} probeGroups={} candidates={} meshHits={} rejectOwner={} rejectDistance={} reason={} selectionFar={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                grab_contact_evidence_policy::contactQualityModeName(grabContactQualityMode),
                grab_contact_evidence_policy::contactEvidenceLevelName(contactEvidenceDecision.level),
                multiFingerGripRuntime.gripSet.groupCount,
                multiFingerGripRuntime.semanticGroupCount,
                multiFingerGripRuntime.liveProbeGroupCount,
                multiFingerGripRuntime.candidateContactCount,
                multiFingerGripRuntime.meshHitCount,
                multiFingerGripRuntime.rejectedOwnerCount,
                multiFingerGripRuntime.rejectedDistanceCount,
                contactEvidenceDecision.reason,
                sel.isFarSelection ? "yes" : "no");
            restoreFailedGrabPrep();
            clearGrabExternalHandWorldTransform(_isLeft);
            _savedObjectState.clear();
            return false;
        }

        if (deferMissingMeshContactToPatch && !strictMultiFingerGripRequired && !contactPatchUsed && !multiFingerGripUsed) {
            ROCK_LOG_WARN(Hand,
                "{} hand GRAB failed: mesh contact required for '{}' formID={:08X}; contact patch did not mesh-snap to a validated pivot "
                "meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} totalTris={} patchMode={} patchFallback={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                nodeDebugName(meshSourceNode),
                nodeDebugName(collidableNode),
                nodeDebugName(rootNode),
                meshStats.visitedShapes,
                meshStats.totalTriangles(),
                contactPatchRuntime.pointMode,
                contactPatchRuntime.patch.fallbackReason ? contactPatchRuntime.patch.fallbackReason : "unknown");
            restoreFailedGrabPrep();
            clearGrabExternalHandWorldTransform(_isLeft);
            _savedObjectState.clear();
            return false;
        }

        logRuntimeScaleIfChanged(_isLeft, handName(), handWorldTransform, collidableNode);

        ROCK_LOG_INFO(Hand, "{} hand GRAB: '{}' formID={:08X} bodyId={}", handName(), objName, sel.refr->GetFormID(), objectBodyId.value);

        if (g_rockConfig.rockDebugShowGrabNotifications) {
            auto msg = std::format("[ROCK] {} GRAB: {} ({})", _isLeft ? "L" : "R", objName, motionTypeStr);
            f4vr::showNotification(msg);
        }

        _heldBodyIds = preparedBodySet.acceptedBodyIds();
        if (_heldBodyIds.empty()) {
            _heldBodyIds.push_back(objectBodyId.value);
        }

        {
            auto* player = RE::PlayerCharacter::GetSingleton();
            if (player) {
                nativeVRGrabDrop(player, 0);
                nativeVRGrabDrop(player, 1);
            }
        }

        _grabStartTime = 0.0f;

        {
            const RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
            RE::NiPoint3 grabPivotAWorld = computeGrabPivotAWorld(world, handWorldTransform);
            if (multiFingerGripUsed) {
                grabPivotAWorld = multiFingerGripRuntime.gripSet.handCenterWorld;
            }
            const RE::NiTransform liveBodyWorldAtGrab = getLiveBodyWorldTransform(world, objectBodyId);
            const RE::NiTransform handBodyWorldAtGrab = getLiveBodyWorldTransform(world, _handBody.getBodyId());
            auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtGrab = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
            auto* ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
            _grabFrame.heldNode = collidableNode;
            _grabFrame.bodyLocal = collidableNode ? computeRuntimeBodyLocalTransform(objectWorldTransform, liveBodyWorldAtGrab) : makeIdentityTransform();
            _grabFrame.ownerBodyLocal = ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, liveBodyWorldAtGrab) : makeIdentityTransform();
            _grabFrame.rootBodyLocal = rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, liveBodyWorldAtGrab) : makeIdentityTransform();
            _grabFrame.localMeshTriangles.clear();
            _grabFrame.surfacePointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabSurfacePoint);
            _grabFrame.surfaceHitLocal = _grabFrame.surfacePointLocal;
            _grabFrame.surfaceNormalLocal = grabSurfaceHit.valid ? transform_math::worldVectorToLocal(objectWorldTransform, grabSurfaceHit.normal) : RE::NiPoint3{};
            _grabFrame.surfacePointBodyLocalGame = grab_contact_patch_math::freezePivotBBodyLocal(liveBodyWorldAtGrab, grabSurfacePoint);
            _grabFrame.pivotBBodyLocalGame = _grabFrame.surfacePointBodyLocalGame;
            _grabFrame.hasFrozenPivotB = true;
            _grabFrame.hasSurfaceHit = grabSurfaceHit.valid || authoredGrabNode != nullptr;
            _grabFrame.hasSurfaceFrame = false;
            _grabFrame.surfaceTriangleIndex = grabSurfaceHit.valid && grabSurfaceHit.hasTriangle ? static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex) : 0xFFFF'FFFF;
            _grabFrame.surfaceShapeKey = grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
            _grabFrame.surfaceShapeCollisionFilterInfo = grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
            _grabFrame.surfaceHitFraction = grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
            _grabFrame.hasSurfaceShapeKey = grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
            _grabFrame.contactPatchSamples = {};
            _grabFrame.contactPatchSampleCount = 0;
            _grabFrame.hasContactPatch = contactPatchUsed;
            _grabFrame.contactPatchMeshSnapDeltaGameUnits = contactPatchRuntime.patch.meshSnapDeltaGameUnits;
            _grabFrame.hasMultiFingerContactPatch = multiFingerGripUsed;
            _grabFrame.multiFingerContactGroupCount = multiFingerGripRuntime.gripSet.groupCount;
            _grabFrame.multiFingerContactReason = multiFingerGripRuntime.reason ? multiFingerGripRuntime.reason : "none";
            _grabFrame.multiFingerContactSpreadGameUnits = multiFingerGripRuntime.gripSet.spreadGameUnits;
            _grabFrame.multiFingerGripCenterWorldAtGrab = multiFingerGripRuntime.gripSet.contactCenterWorld;
            _grabFrame.multiFingerHandCenterWorldAtGrab = multiFingerGripRuntime.gripSet.handCenterWorld;
            _grabFrame.multiFingerAverageNormalWorldAtGrab = multiFingerGripRuntime.gripSet.averageNormalWorld;
            if (contactPatchUsed) {
                const std::uint32_t copyCount = (std::min)(contactPatchRuntime.sampleCount, static_cast<std::uint32_t>(_grabFrame.contactPatchSamples.size()));
                for (std::uint32_t i = 0; i < copyCount; ++i) {
                    auto sample = contactPatchRuntime.samples[i];
                    sample.point = transform_math::worldPointToLocal(liveBodyWorldAtGrab, sample.point);
                    sample.normal = transform_math::worldVectorToLocal(liveBodyWorldAtGrab, sample.normal);
                    _grabFrame.contactPatchSamples[i] = sample;
                }
                _grabFrame.contactPatchSampleCount = copyCount;
            }
            _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(primaryChoice.reason);
            _grabFrame.orientationModeUsed = grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation;
            _grabFrame.surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedMode;
            _grabFrame.surfaceFrameFallbackReason = "preserveObjectRotation";
            _grabFrame.surfaceFrameConfidence = 0.0f;
            _grabFrame.surfacePivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabSurfacePoint);
            _grabFrame.surfaceSelectionToMeshDistanceGameUnits =
                grabSurfaceHit.valid && grabSurfaceHit.hasSelectionHit ? grabSurfaceHit.selectionToMeshDistanceGameUnits : (sel.hasHitPoint ? 0.0f : std::numeric_limits<float>::max());
            if (grabSurfaceHit.valid) {
                grabSurfaceHit.pivotToSurfaceDistanceGameUnits = _grabFrame.surfacePivotToSurfaceDistanceGameUnits;
            }
            _grabFrame.hasMeshPoseData = false;
            if (!grabMeshTriangles.empty()) {
                _grabFrame.localMeshTriangles = cacheTrianglesInLocalSpace(grabMeshTriangles, objectWorldTransform);
                _grabFrame.hasMeshPoseData = !_grabFrame.localMeshTriangles.empty();
            }

            const auto requestedOrientationMode = configuredGrabOrientationMode();
            RE::NiTransform desiredObjectWorld = objectWorldTransform;
            bool usedOppositionFrame = false;
            const auto oppositionContacts = hand_semantic_contact_state::selectThumbOppositionContacts(semanticContacts);
            if (g_rockConfig.rockGrabOppositionFrameEnabled && !multiFingerGripUsed && !authoredGrabNode && !grabSurfaceTriangles.empty() && oppositionContacts.valid) {
                RE::NiTransform thumbWorld{};
                RE::NiTransform opposingWorld{};
                if (tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ oppositionContacts.thumb.handBodyId }, thumbWorld) &&
                    tryResolveLiveBodyWorldTransform(world, RE::hknpBodyId{ oppositionContacts.opposing.handBodyId }, opposingWorld)) {
                    const RE::NiPoint3 thumbToOpposing = normalizeOrZero(opposingWorld.translate - thumbWorld.translate);
                    const RE::NiPoint3 opposingToThumb = normalizeOrZero(thumbWorld.translate - opposingWorld.translate);
                    GrabSurfaceHit thumbHit{};
                    GrabSurfaceHit opposingHit{};
                    const bool thumbHitFound =
                        findClosestGrabSurfaceHit(grabSurfaceTriangles,
                            thumbWorld.translate,
                            thumbToOpposing,
                            g_rockConfig.rockGrabLateralWeight,
                            g_rockConfig.rockGrabDirectionalWeight,
                            thumbHit);
                    const bool opposingHitFound =
                        findClosestGrabSurfaceHit(grabSurfaceTriangles,
                            opposingWorld.translate,
                            opposingToThumb,
                            g_rockConfig.rockGrabLateralWeight,
                            g_rockConfig.rockGrabDirectionalWeight,
                            opposingHit);
                    auto hitOwnerMatchesResolvedBody = [&](const GrabSurfaceHit& hit) {
                        if (!hit.valid || !hit.sourceNode) {
                            return false;
                        }
                        const auto* hitOwnerRecord = preparedBodySet.findAcceptedRecordByOwnerNode(hit.sourceNode);
                        return hitOwnerRecord && hitOwnerRecord->bodyId == objectBodyId.value;
                    };

                    if (thumbHitFound && opposingHitFound && hitOwnerMatchesResolvedBody(thumbHit) && hitOwnerMatchesResolvedBody(opposingHit)) {
                        const RE::NiPoint3 thumbObjectLocal = transform_math::worldPointToLocal(objectWorldTransform, thumbHit.position);
                        const RE::NiPoint3 opposingObjectLocal = transform_math::worldPointToLocal(objectWorldTransform, opposingHit.position);

                        grab_opposition_frame_math::OppositionFrameInput<RE::NiTransform, RE::NiPoint3> oppositionInput{};
                        oppositionInput.enabled = true;
                        oppositionInput.objectWorld = objectWorldTransform;
                        oppositionInput.thumbObjectLocal = thumbObjectLocal;
                        oppositionInput.opposingObjectLocal = opposingObjectLocal;
                        oppositionInput.thumbHandWorld = thumbWorld.translate;
                        oppositionInput.opposingHandWorld = opposingWorld.translate;
                        oppositionInput.objectRollAxisLocal = computeLocalBoundsLongAxis(_grabFrame.localMeshTriangles);
                        oppositionInput.handRollAxisWorld = transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft);

                        const auto oppositionFrame = grab_opposition_frame_math::buildOppositionDesiredObjectWorld(oppositionInput);
                        if (oppositionFrame.valid) {
                            usedOppositionFrame = true;
                            desiredObjectWorld = oppositionFrame.desiredObjectWorld;
                            grabPivotAWorld = oppositionFrame.pivotWorld;
                            grabSurfacePoint = (thumbHit.position + opposingHit.position) * 0.5f;
                            grabSurfaceHit = thumbHit;
                            grabSurfaceHit.position = grabSurfacePoint;
                            grabSurfaceHit.distance = (thumbHit.distance + opposingHit.distance) * 0.5f;
                            grabSurfaceHit.hasTriangle = false;
                            grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                            grabSurfaceHit.selectionToMeshDistanceGameUnits =
                                sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabSurfacePoint) : std::numeric_limits<float>::max();
                            grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabSurfacePoint);
                            surfaceOwnerNode = thumbHit.sourceNode;
                            surfaceOwnerMatchesResolvedBody = true;
                            meshGrabFound = true;
                            grabPointMode = "semanticOppositionFrame";
                            grabFallbackReason = "none";

                            _grabFrame.surfacePointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabSurfacePoint);
                            _grabFrame.surfaceHitLocal = _grabFrame.surfacePointLocal;
                            _grabFrame.surfaceNormalLocal = transform_math::worldVectorToLocal(objectWorldTransform, thumbHit.normal);
                            _grabFrame.surfacePointBodyLocalGame = grab_contact_patch_math::freezePivotBBodyLocal(liveBodyWorldAtGrab, grabSurfacePoint);
                            _grabFrame.pivotBBodyLocalGame = _grabFrame.surfacePointBodyLocalGame;
                            _grabFrame.hasSurfaceHit = true;
                            _grabFrame.surfaceTriangleIndex = 0xFFFF'FFFF;
                            _grabFrame.surfaceShapeKey = thumbHit.shapeKey;
                            _grabFrame.surfaceShapeCollisionFilterInfo = thumbHit.shapeCollisionFilterInfo;
                            _grabFrame.surfaceHitFraction = thumbHit.hitFraction;
                            _grabFrame.hasSurfaceShapeKey = thumbHit.hasShapeKey;
                            _grabFrame.surfacePivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabSurfacePoint);
                            _grabFrame.surfaceSelectionToMeshDistanceGameUnits = grabSurfaceHit.selectionToMeshDistanceGameUnits;
                            _grabFrame.orientationModeUsed = grab_surface_frame_math::GrabOrientationMode::SurfaceNormalAuto;
                            _grabFrame.surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::Accepted;
                            _grabFrame.surfaceFrameConfidence = 1.0f;
                            _grabFrame.surfaceFrameFallbackReason = "thumbOpposition";
                            _grabFrame.oppositionFrameReason = oppositionFrame.reason;
                            _grabFrame.hasOppositionFrame = true;
                            _grabFrame.oppositionThumbWorldAtGrab = thumbWorld.translate;
                            _grabFrame.oppositionOpposingWorldAtGrab = opposingWorld.translate;
                            _grabFrame.oppositionThumbObjectLocal = thumbObjectLocal;
                            _grabFrame.oppositionOpposingObjectLocal = opposingObjectLocal;
                            grab_visual_authority_policy::markValidatedContact(visualAuthorityContact, "thumbOpposition");

                            ROCK_LOG_DEBUG(Hand,
                                "{} hand OPPOSITION GRAB FRAME: thumb={} body={} opposing={} body={} pivot=({:.1f},{:.1f},{:.1f}) "
                                "thumbHand=({:.1f},{:.1f},{:.1f}) opposingHand=({:.1f},{:.1f},{:.1f}) "
                                "thumbHit=({:.1f},{:.1f},{:.1f}) opposingHit=({:.1f},{:.1f},{:.1f}) reason={}",
                                handName(),
                                hand_collider_semantics::roleName(oppositionContacts.thumb.role),
                                oppositionContacts.thumb.handBodyId,
                                hand_collider_semantics::roleName(oppositionContacts.opposing.role),
                                oppositionContacts.opposing.handBodyId,
                                grabPivotAWorld.x,
                                grabPivotAWorld.y,
                                grabPivotAWorld.z,
                                thumbWorld.translate.x,
                                thumbWorld.translate.y,
                                thumbWorld.translate.z,
                                opposingWorld.translate.x,
                                opposingWorld.translate.y,
                                opposingWorld.translate.z,
                                thumbHit.position.x,
                                thumbHit.position.y,
                                thumbHit.position.z,
                                opposingHit.position.x,
                                opposingHit.position.y,
                                opposingHit.position.z,
                                oppositionFrame.reason);
                        } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                            ROCK_LOG_DEBUG(Hand,
                                "{} hand opposition grab frame rejected: reason={} thumb={} opposing={}",
                                handName(),
                                oppositionFrame.reason,
                                hand_collider_semantics::roleName(oppositionContacts.thumb.role),
                                hand_collider_semantics::roleName(oppositionContacts.opposing.role));
                        }
                    } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                        ROCK_LOG_DEBUG(Hand,
                            "{} hand opposition grab frame missing mesh hits: thumbHit={} opposingHit={} thumbOwner={} opposingOwner={} contacts={} reason={}",
                            handName(),
                            thumbHitFound ? "yes" : "no",
                            opposingHitFound ? "yes" : "no",
                            hitOwnerMatchesResolvedBody(thumbHit) ? "yes" : "no",
                            hitOwnerMatchesResolvedBody(opposingHit) ? "yes" : "no",
                            semanticContacts.count,
                            oppositionContacts.reason);
                    }
                } else if (g_rockConfig.rockDebugGrabFrameLogging) {
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand opposition grab frame rejected: stale or missing live hand contact bodies thumb={} opposing={}",
                        handName(),
                        oppositionContacts.thumb.handBodyId,
                        oppositionContacts.opposing.handBodyId);
                }
            } else if (g_rockConfig.rockDebugGrabFrameLogging && g_rockConfig.rockGrabOppositionFrameEnabled && !authoredGrabNode) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand opposition grab frame unavailable: contacts={} reason={} triangles={}",
                    handName(),
                    semanticContacts.count,
                    oppositionContacts.reason,
                    grabSurfaceTriangles.size());
            }

            if (authoredGrabNode) {
                desiredObjectWorld = buildDesiredObjectWorldFromAuthoredGrabNode(objectWorldTransform, authoredGrabNode->world, handWorldTransform, grabPivotAWorld);
                _grabFrame.orientationModeUsed = grab_surface_frame_math::GrabOrientationMode::AuthoredOnly;
                _grabFrame.surfaceFrameLocal.faceKind = grab_surface_frame_math::GrabSurfaceFaceKind::Side;
                _grabFrame.surfaceFrameLocal.tangentSource = grab_surface_frame_math::GrabSurfaceTangentSource::AuthoredFrame;
                _grabFrame.surfaceFrameLocal.confidence = 1.0f;
                _grabFrame.surfaceFrameLocal.fallbackReason = "none";
                _grabFrame.surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::Accepted;
                _grabFrame.surfaceFrameConfidence = 1.0f;
                _grabFrame.surfaceFrameFallbackReason = "none";
                _grabFrame.hasSurfaceFrame = true;
                grab_visual_authority_policy::markValidatedContact(visualAuthorityContact, "authoredGrabNode");
            } else if (multiFingerGripUsed) {
                desiredObjectWorld = objectWorldTransform;
                _grabFrame.orientationModeUsed = grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation;
                _grabFrame.surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedMode;
                _grabFrame.surfaceFrameConfidence = 1.0f;
                _grabFrame.surfaceFrameFallbackReason = "multiFingerGripPreserveObject";
                _grabFrame.hasSurfaceFrame = false;
            } else if (usedOppositionFrame) {
                _grabFrame.hasSurfaceFrame = false;
            } else {
                grab_surface_frame_math::DesiredObjectFrame<RE::NiTransform> desiredFrame{};
                if (grabSurfaceHit.valid) {
                    const RE::NiPoint3 localLongAxis =
                        g_rockConfig.rockGrabAlignmentUseHandParallelTangent ? computeLocalBoundsLongAxis(_grabFrame.localMeshTriangles) : RE::NiPoint3{};
                    RE::NiPoint3 localSurfaceNormal = grab_surface_frame_math::normalizeOrZero(_grabFrame.surfaceNormalLocal);
                    if (contactPatchUsed && contactPatchRuntime.patch.valid) {
                        localSurfaceNormal = grab_surface_frame_math::normalizeOrZero(transform_math::worldVectorToLocal(objectWorldTransform, contactPatchRuntime.patch.normal));
                        RE::NiPoint3 localPatchTangent =
                            grab_surface_frame_math::normalizeOrZero(transform_math::worldVectorToLocal(objectWorldTransform, contactPatchRuntime.patch.tangent));
                        if (grab_surface_frame_math::lengthSquared(localPatchTangent) <= 0.0f) {
                            localPatchTangent = grab_surface_frame_math::stablePerpendicular(localSurfaceNormal);
                        }
                        const bool capFace = grab_surface_frame_math::lengthSquared(localLongAxis) > 0.0f &&
                                             std::fabs(grab_surface_frame_math::dot(localSurfaceNormal, grab_surface_frame_math::normalizeOrZero(localLongAxis))) >=
                                                 g_rockConfig.rockGrabSurfaceCapNormalDotThreshold;
                        _grabFrame.surfaceFrameLocal.normal = localSurfaceNormal;
                        _grabFrame.surfaceFrameLocal.tangent = localPatchTangent;
                        _grabFrame.surfaceFrameLocal.bitangent = grab_surface_frame_math::normalizeOrZero(grab_surface_frame_math::cross(localSurfaceNormal, localPatchTangent));
                        _grabFrame.surfaceFrameLocal.faceKind = contactPatchRuntime.patch.orientationReliable ?
                            (capFace ? grab_surface_frame_math::GrabSurfaceFaceKind::CapTopBottom : grab_surface_frame_math::GrabSurfaceFaceKind::Side) :
                            grab_surface_frame_math::GrabSurfaceFaceKind::Ambiguous;
                        _grabFrame.surfaceFrameLocal.tangentSource = grab_surface_frame_math::GrabSurfaceTangentSource::ContactPatchPrincipal;
                        _grabFrame.surfaceFrameLocal.confidence = contactPatchRuntime.patch.orientationReliable ? contactPatchRuntime.patch.confidence : 0.0f;
                        _grabFrame.surfaceFrameLocal.fallbackReason =
                            contactPatchRuntime.patch.fallbackReason ? contactPatchRuntime.patch.fallbackReason : "none";
                    } else if (grabSurfaceHit.hasTriangle) {
                        const GrabLocalTriangle localHitTriangle{
                            transform_math::worldPointToLocal(objectWorldTransform, grabSurfaceHit.triangle.v0),
                            transform_math::worldPointToLocal(objectWorldTransform, grabSurfaceHit.triangle.v1),
                            transform_math::worldPointToLocal(objectWorldTransform, grabSurfaceHit.triangle.v2),
                        };
                        localSurfaceNormal =
                            grab_surface_frame_math::normalizeOrZero(grab_surface_frame_math::cross(localHitTriangle.v1 - localHitTriangle.v0, localHitTriangle.v2 - localHitTriangle.v1));
                        const RE::NiPoint3 preservedLocalTangent = g_rockConfig.rockGrabAlignmentUseHandParallelTangent ?
                            grab_surface_frame_math::chooseObjectDerivedTangentHint(localSurfaceNormal, localLongAxis) :
                            RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
                        _grabFrame.surfaceFrameLocal = grab_surface_frame_math::buildSurfaceFrameFromTriangle(localHitTriangle.v0,
                            localHitTriangle.v1,
                            localHitTriangle.v2,
                            preservedLocalTangent,
                            localLongAxis,
                            g_rockConfig.rockGrabSurfaceCapNormalDotThreshold,
                            g_rockConfig.rockGrabSurfacePreserveRollForCaps);
                    } else {
                        const RE::NiPoint3 preservedLocalTangent = g_rockConfig.rockGrabAlignmentUseHandParallelTangent ?
                            grab_surface_frame_math::chooseObjectDerivedTangentHint(localSurfaceNormal, localLongAxis) :
                            RE::NiPoint3{ 0.0f, 1.0f, 0.0f };
                        _grabFrame.surfaceFrameLocal = grab_surface_frame_math::buildSurfaceFrameFromNormal(localSurfaceNormal,
                            preservedLocalTangent,
                            localLongAxis,
                            g_rockConfig.rockGrabSurfaceCapNormalDotThreshold,
                            g_rockConfig.rockGrabSurfacePreserveRollForCaps);
                    }
                    const RE::NiPoint3 palmNormalWorld = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);
                    const RE::NiPoint3 palmTangentWorld = transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft);
                    const auto alignmentDecision = grab_surface_frame_math::evaluateSurfaceAlignmentGate(_grabFrame.surfaceFrameLocal,
                        requestedOrientationMode,
                        grabSurfaceHit.hasSelectionHit,
                        surfaceOwnerMatchesResolvedBody,
                        _grabFrame.surfacePivotToSurfaceDistanceGameUnits,
                        _grabFrame.surfaceSelectionToMeshDistanceGameUnits,
                        g_rockConfig.rockGrabAlignmentMaxPivotToSurfaceDistance,
                        g_rockConfig.rockGrabAlignmentMaxSelectionToMeshDistance,
                        g_rockConfig.rockGrabAlignmentRequireResolvedOwnerMatch,
                        g_rockConfig.rockGrabSurfaceFrameMinConfidence);
                    _grabFrame.surfaceAlignmentDecision = alignmentDecision;
                    const auto effectiveOrientationMode = alignmentDecision == grab_surface_frame_math::GrabSurfaceAlignmentDecision::Accepted ?
                        requestedOrientationMode :
                        grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation;
                    desiredFrame = grab_surface_frame_math::buildDesiredObjectWorldFromSurfaceFrame(objectWorldTransform,
                        _grabFrame.surfacePointLocal,
                        _grabFrame.surfaceFrameLocal,
                        grabPivotAWorld,
                        palmNormalWorld,
                        palmTangentWorld,
                        effectiveOrientationMode,
                        g_rockConfig.rockGrabSurfaceFrameMinConfidence);
                    if (alignmentDecision != grab_surface_frame_math::GrabSurfaceAlignmentDecision::Accepted) {
                        desiredFrame.alignmentDecision = alignmentDecision;
                        desiredFrame.fallbackReason = grabSurfaceAlignmentDecisionName(alignmentDecision);
                    }
                    _grabFrame.hasSurfaceFrame = _grabFrame.surfaceFrameLocal.usable();
                } else {
                    grab_surface_frame_math::GrabSurfaceFrame<RE::NiPoint3> fallbackFrame{};
                    _grabFrame.surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedLowConfidence;
                    desiredFrame = grab_surface_frame_math::buildDesiredObjectWorldFromSurfaceFrame(objectWorldTransform,
                        _grabFrame.surfacePointLocal,
                        fallbackFrame,
                        grabPivotAWorld,
                        computePalmNormalFromHandBasis(handWorldTransform, _isLeft),
                        transformHandspaceDirection(handWorldTransform, RE::NiPoint3{ 1.0f, 0.0f, 0.0f }, _isLeft),
                        grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation,
                        g_rockConfig.rockGrabSurfaceFrameMinConfidence);
                }
                desiredObjectWorld = desiredFrame.transform;
                _grabFrame.orientationModeUsed = desiredFrame.modeUsed;
                _grabFrame.surfaceAlignmentDecision = desiredFrame.alignmentDecision;
                _grabFrame.surfaceFrameConfidence = desiredFrame.confidence;
                _grabFrame.surfaceFrameFallbackReason = desiredFrame.fallbackReason;
            }

            grab_visual_authority_policy::assignFallbackReasonIfInvalid(visualAuthorityContact, grabPointMode);
            _grabFrame.visualAuthorityContactValid = visualAuthorityContact.valid;
            _grabFrame.visualAuthorityContactReason = visualAuthorityContact.reason;

            logGrabNodeInfo(handName(),
                _isLeft,
                collidableNode,
                authoredGrabNode,
                desiredObjectWorld,
                handWorldTransform,
                grabPivotAWorld,
                grabPointMode,
                _grabFrame.orientationModeUsed,
                _grabFrame.surfaceAlignmentDecision);

            const auto splitGrabFrame = grab_frame_math::buildSplitGrabFrameFromDesiredObject(
                handWorldTransform,
                handBodyWorldAtGrab,
                desiredObjectWorld,
                _grabFrame.bodyLocal,
                grabPivotAWorld);
            _grabFrame.rawHandSpace = splitGrabFrame.rawHandSpace;
            _grabFrame.constraintHandSpace = splitGrabFrame.constraintHandSpace;
            _grabFrame.handBodyToRawHandAtGrab = splitGrabFrame.handBodyToRawHandAtGrab;
            _grabFrame.pivotAHandBodyLocalGame = splitGrabFrame.pivotAHandBodyLocal;
            _grabFrame.liveHandWorldAtGrab = handWorldTransform;
            _grabFrame.handBodyWorldAtGrab = handBodyWorldAtGrab;
            _grabFrame.objectNodeWorldAtGrab = objectWorldTransform;
            _grabFrame.desiredObjectWorldAtGrab = desiredObjectWorld;
            _grabFrame.hasTelemetryCapture = true;
            _grabFrame.grabPivotWorldAtGrab = grabPivotAWorld;
            _grabFrame.surfacePointWorldAtGrab = grabSurfacePoint;
            _grabFrame.handScaleAtGrab = handWorldTransform.scale;
            _adjustedHandTransform = handWorldTransform;
            _hasAdjustedHandTransform = false;
            if (!g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled) {
                clearGrabExternalHandWorldTransform(_isLeft);
            }
            _grabVisualLerpElapsed = 0.0f;
            _grabDeviationExceededSeconds = 0.0f;
            const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabSurfacePoint;
            const float initialGrabDistance =
                std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
            _grabVisualLerpDuration = held_object_physics_math::computeHandLerpDuration(initialGrabDistance, g_rockConfig.rockGrabHandLerpTimeMin,
                g_rockConfig.rockGrabHandLerpTimeMax, g_rockConfig.rockGrabHandLerpMinDistance, g_rockConfig.rockGrabHandLerpMaxDistance);
            const bool preserveObjectRotation =
                _grabFrame.orientationModeUsed == grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation;
            const bool needsLargeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
            const bool needsOrientationSync = !preserveObjectRotation;
            const bool needsAuthoredOrOppositionSync = authoredGrabNode != nullptr || _grabFrame.hasOppositionFrame;
            _grabFrame.fadeInGrabConstraint = needsLargeInitialSync || needsOrientationSync || needsAuthoredOrOppositionSync;
            if (needsLargeInitialSync) {
                _grabFrame.motorFadeReason = "largeInitialSync";
            } else if (needsAuthoredOrOppositionSync) {
                _grabFrame.motorFadeReason = authoredGrabNode ? "authoredGrabNode" : "oppositionFrame";
            } else if (needsOrientationSync) {
                _grabFrame.motorFadeReason = "orientationAlignment";
            } else {
                _grabFrame.motorFadeReason = "none";
            }
            _grabFingerPoseFrameCounter = 0;
            _heldLocalLinearVelocityHistory = {};
            _heldLocalLinearVelocityHistoryCount = 0;
            _heldLocalLinearVelocityHistoryNext = 0;
            _lastPlayerSpaceVelocityHavok = {};

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiTransform& constraintGrabHandSpace = _grabFrame.constraintHandSpace;
                const RE::NiPoint3 grabPivotAHandspace = computeGrabPivotAHandspacePosition(_isLeft);
                auto* vrScaleSetting = f4vr::getIniSetting("fVrScale:VR");
                const float vrScale = vrScaleSetting ? vrScaleSetting->GetFloat() : -1.0f;

                const RE::NiPoint3 rawLateral = getMatrixColumn(handWorldTransform.rotate, 0);
                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 rawBack = getMatrixColumn(handWorldTransform.rotate, 1);
                const RE::NiPoint3 constraintFinger = getMatrixColumn(handBodyWorldAtGrab.rotate, 2);
                const RE::NiPoint3 constraintBack = getMatrixColumn(handBodyWorldAtGrab.rotate, 1);
                const RE::NiPoint3 constraintLateral = getMatrixColumn(handBodyWorldAtGrab.rotate, 0);
                const RE::NiPoint3 grabSpaceRawFinger = getMatrixColumn(_grabFrame.rawHandSpace.rotate, 0);
                const RE::NiPoint3 grabSpaceConstraintFinger = getMatrixColumn(constraintGrabHandSpace.rotate, 0);
                const RE::NiPoint3 grabPosDelta = constraintGrabHandSpace.translate - _grabFrame.rawHandSpace.translate;
                const float rawVsConstraintRot = rotationDeltaDegrees(_grabFrame.rawHandSpace.rotate, constraintGrabHandSpace.rotate);
                const float rawVsConstraintPos = std::sqrt(grabPosDelta.x * grabPosDelta.x + grabPosDelta.y * grabPosDelta.y + grabPosDelta.z * grabPosDelta.z);

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SUMMARY: vrScale={:.3f} handScale={:.3f} bodyScale={:.3f} objectScale={:.3f} "
                    "pivotHS=({:.2f},{:.2f},{:.2f}) pivotW=({:.1f},{:.1f},{:.1f}) surfaceW=({:.1f},{:.1f},{:.1f}) "
                    "pivotBLocal=({:.2f},{:.2f},{:.2f}) "
                    "rawConstraintDelta={:.2f}deg/{:.2f}gu pivotALocal=({:.2f},{:.2f},{:.2f}) meshMode={} meshTris={} "
                    "orientation={} alignment={} face={} confidence={:.2f} tangent={} pivotSurface={:.1f}gu selectionMesh={:.1f}gu "
                    "shapeKey=0x{:08X} shapeFilter=0x{:08X} hitFraction={:.4f} contactPatch={} patchHits={} snapDelta={:.1f}gu "
                    "multiFinger={} mfGroups={} mfSpread={:.2f}gu mfReason={} "
                    "opposition={} oppReason={} motorFade={} motorFadeReason={} bodyReason={} fallback={} visualAuthContact={} visualAuthReason={}",
                    handName(), vrScale, handWorldTransform.scale, handBodyWorldAtGrab.scale, collidableNode ? collidableNode->world.scale : -1.0f,
                    grabPivotAHandspace.x, grabPivotAHandspace.y, grabPivotAHandspace.z, grabPivotAWorld.x, grabPivotAWorld.y, grabPivotAWorld.z, grabSurfacePoint.x,
                    grabSurfacePoint.y, grabSurfacePoint.z, _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z,
                    rawVsConstraintRot, rawVsConstraintPos, _grabFrame.pivotAHandBodyLocalGame.x,
                    _grabFrame.pivotAHandBodyLocalGame.y, _grabFrame.pivotAHandBodyLocalGame.z, grabPointMode, grabMeshTriangles.size(),
                    grabOrientationModeName(_grabFrame.orientationModeUsed), grabSurfaceAlignmentDecisionName(_grabFrame.surfaceAlignmentDecision),
                    grabSurfaceFaceKindName(_grabFrame.surfaceFrameLocal.faceKind), _grabFrame.surfaceFrameConfidence,
                    grabSurfaceTangentSourceName(_grabFrame.surfaceFrameLocal.tangentSource), _grabFrame.surfacePivotToSurfaceDistanceGameUnits,
                    _grabFrame.surfaceSelectionToMeshDistanceGameUnits, _grabFrame.surfaceShapeKey, _grabFrame.surfaceShapeCollisionFilterInfo, _grabFrame.surfaceHitFraction,
                    _grabFrame.hasContactPatch ? "yes" : "no", _grabFrame.contactPatchSampleCount, _grabFrame.contactPatchMeshSnapDeltaGameUnits,
                    _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.multiFingerContactReason,
                    _grabFrame.hasOppositionFrame ? "yes" : "no", _grabFrame.oppositionFrameReason, _grabFrame.fadeInGrabConstraint ? "yes" : "no",
                    _grabFrame.motorFadeReason, _grabFrame.bodyResolutionReason, _grabFrame.surfaceFrameFallbackReason,
                    _grabFrame.visualAuthorityContactValid ? "yes" : "no", _grabFrame.visualAuthorityContactReason);

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SNAPSHOT: rawVsConstraint rotDelta={:.2f}deg posDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) rawBack=({:.3f},{:.3f},{:.3f}) rawLat=({:.3f},{:.3f},{:.3f}) "
                    "constraintFinger=({:.3f},{:.3f},{:.3f}) constraintBack=({:.3f},{:.3f},{:.3f}) constraintLat=({:.3f},{:.3f},{:.3f})",
                    handName(), rawVsConstraintRot, grabPosDelta.x, grabPosDelta.y, grabPosDelta.z, rawFinger.x,
                    rawFinger.y, rawFinger.z, rawBack.x, rawBack.y, rawBack.z, rawLateral.x, rawLateral.y, rawLateral.z, constraintFinger.x, constraintFinger.y, constraintFinger.z,
                    constraintBack.x, constraintBack.y, constraintBack.z, constraintLateral.x, constraintLateral.y, constraintLateral.z);

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME TARGETS: grabHSRaw.pos=({:.2f},{:.2f},{:.2f}) grabHSConstraint.pos=({:.2f},{:.2f},{:.2f}) "
                    "grabHSRawFinger=({:.3f},{:.3f},{:.3f}) grabHSConstraintFinger=({:.3f},{:.3f},{:.3f}) "
                    "bodyLocal.pos=({:.2f},{:.2f},{:.2f}) bodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, constraintGrabHandSpace.translate.x,
                    constraintGrabHandSpace.translate.y, constraintGrabHandSpace.translate.z, grabSpaceRawFinger.x, grabSpaceRawFinger.y, grabSpaceRawFinger.z,
                    grabSpaceConstraintFinger.x, grabSpaceConstraintFinger.y, grabSpaceConstraintFinger.z, _grabFrame.bodyLocal.translate.x, _grabFrame.bodyLocal.translate.y,
                    _grabFrame.bodyLocal.translate.z, _grabFrame.bodyLocal.rotate.entry[0][0], _grabFrame.bodyLocal.rotate.entry[1][0],
                    _grabFrame.bodyLocal.rotate.entry[2][0]);

                const RE::NiPoint3 rootBodyLocalFinger = getMatrixColumn(_grabFrame.rootBodyLocal.rotate, 0);
                const RE::NiPoint3 ownerBodyLocalFinger = getMatrixColumn(_grabFrame.ownerBodyLocal.rotate, 0);
                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB NODE FRAMES: owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) held='{}'({:p}) mesh='{}'({:p}) sameOwnerHeld={} sameRootHeld={} "
                    "ownerBodyLocal.pos=({:.2f},{:.2f},{:.2f}) ownerBodyLocalFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootBodyLocal.pos=({:.2f},{:.2f},{:.2f}) rootBodyLocalFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), nodeDebugName(ownerNodeAtGrab), static_cast<const void*>(ownerNodeAtGrab),
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get()) ? "yes" : "no",
                    (ownerNodeAtGrab && ownerNodeAtGrab->collisionObject.get() == bodyCollisionObjectAtGrab) ? "yes" : "no", nodeDebugName(rootNode),
                    static_cast<const void*>(rootNode), nodeDebugName(collidableNode), static_cast<const void*>(collidableNode), nodeDebugName(meshSourceNode),
                    static_cast<const void*>(meshSourceNode), ownerNodeAtGrab == collidableNode ? "yes" : "no", rootNode == collidableNode ? "yes" : "no",
                    _grabFrame.ownerBodyLocal.translate.x, _grabFrame.ownerBodyLocal.translate.y, _grabFrame.ownerBodyLocal.translate.z, ownerBodyLocalFinger.x,
                    ownerBodyLocalFinger.y, ownerBodyLocalFinger.z, _grabFrame.rootBodyLocal.translate.x, _grabFrame.rootBodyLocal.translate.y,
                    _grabFrame.rootBodyLocal.translate.z, rootBodyLocalFinger.x, rootBodyLocalFinger.y, rootBodyLocalFinger.z);
            }

            ROCK_LOG_DEBUG(Hand,
                "{} GRAB HAND SPACE: pos=({:.1f},{:.1f},{:.1f}) "
                "palmPos=({:.1f},{:.1f},{:.1f}) pivotA=({:.1f},{:.1f},{:.1f}) grabPt=({:.1f},{:.1f},{:.1f})",
                handName(), _grabFrame.rawHandSpace.translate.x, _grabFrame.rawHandSpace.translate.y, _grabFrame.rawHandSpace.translate.z, palmPos.x, palmPos.y, palmPos.z, grabPivotAWorld.x,
                grabPivotAWorld.y, grabPivotAWorld.z, grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z);
            ROCK_LOG_DEBUG(Hand, "{} BODY LOCAL: pos=({:.2f},{:.2f},{:.2f}) scale={:.3f}", handName(), _grabFrame.bodyLocal.translate.x, _grabFrame.bodyLocal.translate.y,
                _grabFrame.bodyLocal.translate.z, _grabFrame.bodyLocal.scale);
        }

        {
            RE::NiTransform handBodyDiag{};
            RE::NiTransform objectBodyDiag{};
            const bool hasLiveDiag = tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyDiag) &&
                                     tryResolveLiveBodyWorldTransform(world, objectBodyId, objectBodyDiag);
            if (hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} DIAG: handBodyLive pos=({:.1f},{:.1f},{:.1f}) objBodyLive pos=({:.1f},{:.1f},{:.1f})",
                    handName(),
                    handBodyDiag.translate.x,
                    handBodyDiag.translate.y,
                    handBodyDiag.translate.z,
                    objectBodyDiag.translate.x,
                    objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
            ROCK_LOG_TRACE(Hand, "{} DIAG: handNi pos=({:.1f},{:.1f},{:.1f}) objNi pos=({:.1f},{:.1f},{:.1f})", handName(), handWorldTransform.translate.x,
                handWorldTransform.translate.y, handWorldTransform.translate.z, objectWorldTransform.translate.x, objectWorldTransform.translate.y,
                objectWorldTransform.translate.z);

            float comX, comY, comZ;
            if (getBodyCOMWorld(world, objectBodyId, comX, comY, comZ) && hasLiveDiag) {
                ROCK_LOG_TRACE(Hand,
                    "{} B8 COM LIVE: comHk=({:.3f},{:.3f},{:.3f}) objBodyLive=({:.1f},{:.1f},{:.1f})",
                    handName(),
                    comX,
                    comY,
                    comZ,
                    objectBodyDiag.translate.x,
                    objectBodyDiag.translate.y,
                    objectBodyDiag.translate.z);
            }
        }

        {
            const RE::hkVector4f zeroVel{ 0.0f, 0.0f, 0.0f, 0.0f };
            havok_runtime::setBodyVelocityDeferred(world, objectBodyId.value, zeroVel, zeroVel);

            for (auto bid : _heldBodyIds) {
                if (bid != objectBodyId.value) {
                    havok_runtime::setBodyVelocityDeferred(world, bid, zeroVel, zeroVel);
                }
            }
        }

        suppressHandCollisionForGrab(world);

        normalizeGrabbedInertiaForBodies(world, objectBodyId, _heldBodyIds, _savedObjectState);

        {
            RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
            RE::NiPoint3 grabPivotAWorld =
                _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
            const float gameToHkScale = gameToHavokScale();
            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);

            float pivotAHk[4];
            pivotAHk[0] = grabPivotAWorld.x * gameToHkScale;
            pivotAHk[1] = grabPivotAWorld.y * gameToHkScale;
            pivotAHk[2] = grabPivotAWorld.z * gameToHkScale;
            pivotAHk[3] = 0.0f;

            float surfaceWorldHk[4];
            surfaceWorldHk[0] = grabSurfacePoint.x * gameToHkScale;
            surfaceWorldHk[1] = grabSurfacePoint.y * gameToHkScale;
            surfaceWorldHk[2] = grabSurfacePoint.z * gameToHkScale;
            surfaceWorldHk[3] = 0.0f;

            float pivotBLocalHk[4];
            pivotBLocalHk[0] = _grabFrame.pivotBBodyLocalGame.x * gameToHkScale;
            pivotBLocalHk[1] = _grabFrame.pivotBBodyLocalGame.y * gameToHkScale;
            pivotBLocalHk[2] = _grabFrame.pivotBBodyLocalGame.z * gameToHkScale;
            pivotBLocalHk[3] = 0.0f;

            {
                float pivotAToGrab = std::sqrt(
                    (pivotAHk[0] - surfaceWorldHk[0]) * (pivotAHk[0] - surfaceWorldHk[0]) + (pivotAHk[1] - surfaceWorldHk[1]) * (pivotAHk[1] - surfaceWorldHk[1]) +
                    (pivotAHk[2] - surfaceWorldHk[2]) * (pivotAHk[2] - surfaceWorldHk[2]));
                float palmToHandOrigin = std::sqrt((palmPos.x - handWorldTransform.translate.x) * (palmPos.x - handWorldTransform.translate.x) +
                    (palmPos.y - handWorldTransform.translate.y) * (palmPos.y - handWorldTransform.translate.y) +
                    (palmPos.z - handWorldTransform.translate.z) * (palmPos.z - handWorldTransform.translate.z));
                float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) * (grabPivotAWorld.x - handWorldTransform.translate.x) +
                    (grabPivotAWorld.y - handWorldTransform.translate.y) * (grabPivotAWorld.y - handWorldTransform.translate.y) +
                    (grabPivotAWorld.z - handWorldTransform.translate.z) * (grabPivotAWorld.z - handWorldTransform.translate.z));
                ROCK_LOG_DEBUG(Hand,
                    "GRAB DIAG {}: palmPos=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
                    "pivotA=({:.1f},{:.1f},{:.1f}) grabSurface=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
                    "frozenPivotB=({:.2f},{:.2f},{:.2f}) contactPatch={} patchHits={} multiFinger={} mfGroups={} mfSpread={:.2f} "
                    "pivotAToGrab_hk={:.4f} ({:.1f} game units) palmToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units "
                    "selectionToMesh={:.1f} alignment={} orientation={} face={} tangent={} visualAuthContact={} visualAuthReason={}",
                    handName(), palmPos.x, palmPos.y, palmPos.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                    grabPivotAWorld.y, grabPivotAWorld.z, grabSurfacePoint.x, grabSurfacePoint.y, grabSurfacePoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                    _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z, _grabFrame.hasContactPatch ? "yes" : "no",
                    _grabFrame.contactPatchSampleCount, _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, pivotAToGrab,
                    pivotAToGrab * havokToGameScale(), palmToHandOrigin, pivotAToHandOrigin, _grabFrame.surfaceSelectionToMeshDistanceGameUnits,
                    grabSurfaceAlignmentDecisionName(_grabFrame.surfaceAlignmentDecision), grabOrientationModeName(_grabFrame.orientationModeUsed),
                    grabSurfaceFaceKindName(_grabFrame.surfaceFrameLocal.faceKind), grabSurfaceTangentSourceName(_grabFrame.surfaceFrameLocal.tangentSource),
                    _grabFrame.visualAuthorityContactValid ? "yes" : "no", _grabFrame.visualAuthorityContactReason);
            }

            _activeConstraint = createGrabConstraint(world, _handBody.getBodyId(), objectBodyId, _grabFrame.handBodyWorldAtGrab, grabPivotAWorld, pivotBLocalHk,
                desiredBodyTransformHandSpace, tau, damping, maxForce, proportionalRecovery, constantRecovery);
        }

        if (!_activeConstraint.isValid()) {
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: constraint creation failed", handName());
            clearGrabExternalHandWorldTransform(_isLeft);
            restoreGrabbedInertia(world, _savedObjectState);
            restoreFailedGrabPrep();
            restoreHandCollisionAfterGrab(world);
            _savedObjectState.clear();
            _heldBodyIds.clear();
            return false;
        }

        {
            auto* cd = static_cast<char*>(_activeConstraint.constraintData);
            float* pivotA = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
            float* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);
            float* tA_col0 = reinterpret_cast<float*>(cd + offsets::kTransformA_Col0);
            float* tB_col0 = reinterpret_cast<float*>(cd + offsets::kTransformB_Col0);
            ROCK_LOG_TRACE(Hand, "{} DIAG: pivotA=({:.3f},{:.3f},{:.3f}) pivotB=({:.3f},{:.3f},{:.3f})", handName(), pivotA[0], pivotA[1], pivotA[2], pivotB[0], pivotB[1],
                pivotB[2]);
            ROCK_LOG_TRACE(Hand, "{} DIAG: tA_col0=({:.3f},{:.3f},{:.3f}) tB_col0=({:.3f},{:.3f},{:.3f})", handName(), tA_col0[0], tA_col0[1], tA_col0[2], tB_col0[0], tB_col0[1],
                tB_col0[2]);
            ROCK_LOG_TRACE(Hand, "{} DIAG: tau={:.3f} damping={:.2f} force={:.0f} propRecov={:.1f} constRecov={:.1f}", handName(), tau, damping, maxForce, proportionalRecovery,
                constantRecovery);
        }

        ROCK_LOG_DEBUG(Hand, "{} hand constraint grab: constraintId={}, hand body={}, obj body={}, {} held bodies", handName(), _activeConstraint.constraintId,
            _handBody.getBodyId().value, objectBodyId.value, _heldBodyIds.size());

        {
            typedef void enableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
            static REL::Relocation<enableBodyFlags_t> enableBodyFlags{ REL::Offset(offsets::kFunc_EnableBodyFlags) };
            for (auto bid : _heldBodyIds) {
                enableBodyFlags(world, bid, 0x80, 0);
            }
        }
        _heldBodyContactFrame.store(100, std::memory_order_release);
        _activeGrabLifecycle = std::move(activeLifecycle);

        {
            int count = (std::min)(static_cast<int>(_heldBodyIds.size()), MAX_HELD_BODIES);
            for (int i = 0; i < count; i++) {
                _heldBodyIdsSnapshot[i] = _heldBodyIds[i];
            }
            _heldBodyIdsCount.store(count, std::memory_order_release);
            _isHoldingFlag.store(true, std::memory_order_release);
        }

        if (g_rockConfig.rockGrabNearbyDampingEnabled) {
            object_physics_body_set::BodySetScanOptions dampingOptions{};
            dampingOptions.mode = physics_body_classifier::InteractionMode::PassivePush;
            dampingOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
            dampingOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
            dampingOptions.heldBySameHand = &_heldBodyIds;
            dampingOptions.maxDepth = (std::max)(1, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
            _nearbyGrabDamping = nearby_grab_damping::beginNearbyGrabDamping(bhkWorld,
                world,
                sel.refr,
                _heldBodyIds,
                grabSurfacePoint,
                g_rockConfig.rockGrabNearbyDampingRadius,
                g_rockConfig.rockGrabNearbyDampingSeconds,
                g_rockConfig.rockGrabNearbyLinearDamping,
                g_rockConfig.rockGrabNearbyAngularDamping,
                dampingOptions);
        } else {
            _nearbyGrabDamping.clear();
        }

        stopSelectionHighlight();
        clearSelectedCloseFingerPose();
        const bool useObjectReverseAlignedFingerPoseAtGrab =
            grab_visual_authority_policy::shouldUseObjectReverseAlignedHandForFingerPose(
                g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled, _hasAdjustedHandTransform, _grabFrame.visualAuthorityContactValid);
        const RE::NiTransform& initialFingerHandTransform = useObjectReverseAlignedFingerPoseAtGrab ? _adjustedHandTransform : handWorldTransform;
        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshotAtGrab{};
        const auto* liveFingerSnapshotAtGrabPtr =
            root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshotAtGrab) ? &liveFingerSnapshotAtGrab : nullptr;
        const RE::NiPoint3 fingerPosePivotWorld =
            _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, initialFingerHandTransform);
        const auto fingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled ?
            grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                grabMeshTriangles, initialFingerHandTransform, _isLeft,
                fingerPosePivotWorld, grabSurfacePoint,
                g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, true, liveFingerSnapshotAtGrabPtr) :
            grab_finger_pose_runtime::SolvedGrabFingerPose{};
        _grabFingerProbeStart = fingerPose.probeStart;
        _grabFingerProbeEnd = fingerPose.probeEnd;
        _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
        applyRockGrabHandPose(_isLeft,
            fingerPose,
            _grabFingerJointPose,
            _hasGrabFingerJointPose,
            _grabFingerLocalTransforms,
            _grabFingerLocalTransformMask,
            _hasGrabFingerLocalTransforms,
            0.0f);

        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::GrabCommitSucceeded });
        clearPullRuntimeState();

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), objectBodyId.value);
        return true;
    }

    void Hand::updateHeldObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
        float deltaTime,
        float forceFadeInTime,
        float tauMin)
    {
        if (!isHolding() || !world)
            return;
        if (!_activeConstraint.isValid()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate);
            return;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate);
            return;
        }

        nearby_grab_damping::tickNearbyGrabDamping(world, _nearbyGrabDamping, deltaTime);

        suppressHandCollisionForGrab(world);

        _grabStartTime += deltaTime;

        if (g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled) {
            RE::NiTransform adjustedTarget{};
            if (computeAdjustedHandTransformTarget(world, adjustedTarget)) {
                if (!_hasAdjustedHandTransform) {
                    _adjustedHandTransform = handWorldTransform;
                    _grabVisualLerpElapsed = 0.0f;
                    _hasAdjustedHandTransform = true;
                }

                const bool lerpExpired = _grabVisualLerpElapsed >= _grabVisualLerpDuration;
                if (!g_rockConfig.rockGrabHandLerpEnabled || lerpExpired) {
                    _adjustedHandTransform = adjustedTarget;
                } else {
                    const auto advanced = hand_visual_lerp_math::advanceTransform(_adjustedHandTransform, adjustedTarget, g_rockConfig.rockGrabLerpSpeed,
                        g_rockConfig.rockGrabLerpAngularSpeed, deltaTime);
                    _adjustedHandTransform = advanced.transform;
                    _grabVisualLerpElapsed += (std::max)(0.0f, deltaTime);
                    if (advanced.reachedTarget || _grabVisualLerpElapsed >= _grabVisualLerpDuration) {
                        _adjustedHandTransform = adjustedTarget;
                    }
                }
            } else {
                _hasAdjustedHandTransform = false;
                clearGrabExternalHandWorldTransform(_isLeft);
            }
            if (_hasAdjustedHandTransform) {
                if (grab_visual_authority_policy::shouldApplyObjectReverseAlignedExternalHandTransform(
                        g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled, true, _grabFrame.visualAuthorityContactValid)) {
                    applyGrabExternalHandWorldTransform(_isLeft, _adjustedHandTransform);
                }
            }
        } else if (_hasAdjustedHandTransform) {
            _hasAdjustedHandTransform = false;
            clearGrabExternalHandWorldTransform(_isLeft);
        }

        if (_activeConstraint.constraintData) {
            auto* cd = static_cast<char*>(_activeConstraint.constraintData);
            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);

            const RE::NiTransform desiredBodyToHandSpace = invertTransform(desiredBodyTransformHandSpace);
            RE::NiPoint3 pivotAHandBodyLocalGame = _grabFrame.pivotAHandBodyLocalGame;

            {
                auto* target = reinterpret_cast<float*>(cd + ATOM_RAGDOLL_MOT + 0x10);
                grab_constraint_math::writeHavokRotationColumns(target, desiredBodyToHandSpace.rotate);
            }

            {
                RE::NiTransform liveHandBodyWorld{};
                if (_handBody.getBodyId().value != INVALID_BODY_ID && tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), liveHandBodyWorld)) {
                    RE::NiPoint3 grabPivotAWorld{};
                    if (_grabFrame.hasOppositionFrame) {
                        grabPivotAWorld = transform_math::localPointToWorld(liveHandBodyWorld, _grabFrame.pivotAHandBodyLocalGame);
                    } else {
                        grabPivotAWorld = computeGrabPivotAWorld(world, handWorldTransform);
                        pivotAHandBodyLocalGame = grab_constraint_math::computeConstraintPivotLocalGame(liveHandBodyWorld, grabPivotAWorld);
                        _grabFrame.pivotAHandBodyLocalGame = pivotAHandBodyLocalGame;
                    }
                    auto* pivotA = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
                    grab_constraint_math::writeConstraintPivotLocalTranslation(pivotA, liveHandBodyWorld, grabPivotAWorld, gameToHavokScale());
                }
            }

            {
                auto* pivotB = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);
                grab_constraint_math::writeDynamicTransformBTranslation(pivotB, desiredBodyTransformHandSpace, pivotAHandBodyLocalGame, gameToHavokScale());
            }

        }

        float grabPositionErrorGameUnits = 0.0f;
        float grabRotationErrorDegrees = 0.0f;
        {
            const RE::NiTransform desiredBodyTransformHandSpace = multiplyTransforms(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
            const RE::NiTransform handBodyWorld = getLiveBodyWorldTransform(world, _handBody.getBodyId());
            const RE::NiTransform desiredBodyWorld = multiplyTransforms(handBodyWorld, desiredBodyTransformHandSpace);
            const RE::NiTransform liveBodyWorld = getLiveBodyWorldTransform(world, _savedObjectState.bodyId);
            grabPositionErrorGameUnits = translationDeltaGameUnits(liveBodyWorld, desiredBodyWorld);
            grabRotationErrorDegrees = rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorld.rotate);
        }

        _grabDeviationExceededSeconds = held_object_physics_math::advanceDeviationSeconds(
            _grabDeviationExceededSeconds, grabPositionErrorGameUnits, g_rockConfig.rockGrabMaxDeviation, deltaTime);
        if (held_object_physics_math::deviationExceeded(_grabDeviationExceededSeconds, g_rockConfig.rockGrabMaxDeviationTime)) {
            ROCK_LOG_WARN(Hand,
                "{} hand release: held object exceeded max deviation ({:.1f}gu > {:.1f}gu for {:.2f}s)",
                handName(),
                grabPositionErrorGameUnits,
                g_rockConfig.rockGrabMaxDeviation,
                _grabDeviationExceededSeconds);
            releaseGrabbedObject(world);
            return;
        }

        const bool heldBodyColliding = isHeldBodyColliding();
        tickHeldBodyContact();

        grab_motion_controller::MotorInput motorInput{};
        motorInput.enabled = g_rockConfig.rockGrabAdaptiveMotorEnabled;
        motorInput.heldBodyColliding = heldBodyColliding;
        motorInput.positionErrorGameUnits = grabPositionErrorGameUnits;
        motorInput.rotationErrorDegrees = grabRotationErrorDegrees;
        motorInput.fullPositionErrorGameUnits = g_rockConfig.rockGrabAdaptivePositionFullError;
        motorInput.fullRotationErrorDegrees = g_rockConfig.rockGrabAdaptiveRotationFullError;
        motorInput.baseLinearTau = g_rockConfig.rockGrabLinearTau;
        motorInput.baseAngularTau = g_rockConfig.rockGrabAngularTau;
        motorInput.collisionTau = tauMin;
        motorInput.maxTau = g_rockConfig.rockGrabTauMax;
        motorInput.currentLinearTau = _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : _activeConstraint.currentTau;
        motorInput.currentAngularTau = _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : _activeConstraint.currentTau;
        motorInput.tauLerpSpeed = g_rockConfig.rockGrabTauLerpSpeed;
        motorInput.deltaTime = deltaTime;
        motorInput.baseMaxForce = _activeConstraint.targetMaxForce;
        motorInput.maxForceMultiplier = g_rockConfig.rockGrabAdaptiveMaxForceMultiplier;
        motorInput.mass = readBodyMass(world, _savedObjectState.bodyId);
        motorInput.forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio;
        motorInput.angularToLinearForceRatio = g_rockConfig.rockGrabAngularToLinearForceRatio;
        motorInput.fadeInEnabled = _grabFrame.fadeInGrabConstraint;
        motorInput.fadeElapsed = _state == HandState::HeldInit ? _grabStartTime : forceFadeInTime;
        motorInput.fadeDuration = forceFadeInTime;
        motorInput.fadeStartAngularRatio = g_rockConfig.rockGrabFadeInStartAngularRatio;

        const auto motorTargets = grab_motion_controller::solveMotorTargets(motorInput);
        const float currentLinearForce = motorTargets.linearMaxForce;
        const float currentAngularForce = motorTargets.angularMaxForce;
        const float currentLinearTau = motorTargets.linearTau;
        const float currentAngularTau = motorTargets.angularTau;

        if (_state == HandState::HeldInit) {
            if (motorTargets.fadeFactor >= 0.999f) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::HeldFadeComplete });
                ROCK_LOG_DEBUG(Hand, "{} hand: HeldInit -> HeldBody (force fade-in complete, {:.2f}s)", handName(), _grabStartTime);
            }
        }

        if (_activeConstraint.angularMotor) {
            _activeConstraint.angularMotor->tau = currentAngularTau;
            _activeConstraint.angularMotor->maxForce = currentAngularForce;
            _activeConstraint.angularMotor->minForce = -currentAngularForce;
            _activeConstraint.angularMotor->damping = g_rockConfig.rockGrabAngularDamping;
            _activeConstraint.angularMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabAngularProportionalRecovery;
            _activeConstraint.angularMotor->constantRecoveryVelocity = g_rockConfig.rockGrabAngularConstantRecovery;
        }
        if (_activeConstraint.linearMotor) {
            _activeConstraint.linearMotor->tau = currentLinearTau;
            _activeConstraint.linearMotor->maxForce = currentLinearForce;
            _activeConstraint.linearMotor->minForce = -currentLinearForce;
            _activeConstraint.linearMotor->damping = g_rockConfig.rockGrabLinearDamping;
            _activeConstraint.linearMotor->proportionalRecoveryVelocity = g_rockConfig.rockGrabLinearProportionalRecovery;
            _activeConstraint.linearMotor->constantRecoveryVelocity = g_rockConfig.rockGrabLinearConstantRecovery;
        }
        _activeConstraint.currentTau = currentLinearTau;
        _activeConstraint.currentMaxForce = currentLinearForce;

        {
            const auto compensationResult = applyHeldMotionCompensation(world, _savedObjectState.bodyId, _heldBodyIds, playerSpaceFrame,
                _lastPlayerSpaceVelocityHavok, g_rockConfig.rockGrabVelocityDamping, g_rockConfig.rockGrabResidualVelocityDamping);
            if (compensationResult.hasPrimaryVelocity) {
                _heldLocalLinearVelocityHistory[_heldLocalLinearVelocityHistoryNext] = compensationResult.primaryLocalLinearVelocity;
                _heldLocalLinearVelocityHistoryNext = (_heldLocalLinearVelocityHistoryNext + 1) % _heldLocalLinearVelocityHistory.size();
                if (_heldLocalLinearVelocityHistoryCount < _heldLocalLinearVelocityHistory.size()) {
                    ++_heldLocalLinearVelocityHistoryCount;
                }
            }
            _lastPlayerSpaceVelocityHavok = (playerSpaceFrame.enabled && !playerSpaceFrame.warp) ? playerSpaceFrame.velocityHavok : RE::NiPoint3{};
        }

        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && _grabFrame.hasMeshPoseData && !_grabFrame.localMeshTriangles.empty()) {
            const int updateInterval = (std::max)(1, g_rockConfig.rockGrabFingerPoseUpdateInterval);
            ++_grabFingerPoseFrameCounter;
            if (_grabFingerPoseFrameCounter >= updateInterval) {
                _grabFingerPoseFrameCounter = 0;
                const RE::NiTransform liveBodyWorld = getLiveBodyWorldTransform(world, _savedObjectState.bodyId);
                const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(liveBodyWorld, _grabFrame.bodyLocal);
                const auto worldTriangles = rebuildTrianglesInWorldSpace(_grabFrame.localMeshTriangles, currentNodeWorld);
                const RE::NiPoint3 grabSurfaceWorld = transform_math::localPointToWorld(currentNodeWorld, _grabFrame.surfacePointLocal);
                const bool useObjectReverseAlignedFingerPose =
                    grab_visual_authority_policy::shouldUseObjectReverseAlignedHandForFingerPose(
                        g_rockConfig.rockGrabObjectVisualHandAuthorityEnabled, _hasAdjustedHandTransform, _grabFrame.visualAuthorityContactValid);
                const RE::NiTransform& fingerHandTransform = useObjectReverseAlignedFingerPose ? _adjustedHandTransform : handWorldTransform;
                root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshot{};
                const auto* liveFingerSnapshotPtr =
                    root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshot) ? &liveFingerSnapshot : nullptr;
                const auto fingerPose = grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(worldTriangles, fingerHandTransform, _isLeft,
                    computeGrabPivotAWorld(world, fingerHandTransform), grabSurfaceWorld, g_rockConfig.rockGrabFingerMinValue,
                    g_rockConfig.rockGrabMaxTriangleDistance, true, liveFingerSnapshotPtr);
                _grabFingerProbeStart = fingerPose.probeStart;
                _grabFingerProbeEnd = fingerPose.probeEnd;
                _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
                applyRockGrabHandPose(_isLeft,
                    fingerPose,
                    _grabFingerJointPose,
                    _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms,
                    _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms,
                    deltaTime);
            }
        }

        _heldLogCounter++;
        if (_heldLogCounter >= 45) {
            _heldLogCounter = 0;

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiTransform liveHandBodyWorld = getLiveBodyWorldTransform(world, _handBody.getBodyId());
                const RE::NiTransform desiredNodeWorldRaw = multiplyTransforms(handWorldTransform, _grabFrame.rawHandSpace);
                const RE::NiTransform desiredNodeWorldConstraint = multiplyTransforms(liveHandBodyWorld, _grabFrame.constraintHandSpace);
                const RE::NiTransform desiredBodyTransformHandSpaceRaw = multiplyTransforms(_grabFrame.rawHandSpace, _grabFrame.bodyLocal);
                const RE::NiTransform desiredBodyTransformHandSpaceConstraint = multiplyTransforms(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
                const RE::NiTransform desiredBodyWorldRaw = multiplyTransforms(handWorldTransform, desiredBodyTransformHandSpaceRaw);
                const RE::NiTransform desiredBodyWorldConstraint = multiplyTransforms(liveHandBodyWorld, desiredBodyTransformHandSpaceConstraint);
                const RE::NiMatrix3 invRot = desiredBodyTransformHandSpaceConstraint.rotate.Transpose();
                const RE::NiTransform liveBodyWorld = getLiveBodyWorldTransform(world, _savedObjectState.bodyId);
                auto* ownerCell = _savedObjectState.refr ? _savedObjectState.refr->GetParentCell() : nullptr;
                auto* heldBhkWorld = ownerCell ? ownerCell->GetbhkWorld() : nullptr;
                auto* bodyCollisionObject = heldBhkWorld ? RE::bhkNPCollisionObject::Getbhk(heldBhkWorld, _savedObjectState.bodyId) : nullptr;
                auto* ownerNode = bodyCollisionObject ? bodyCollisionObject->sceneObject : nullptr;
                auto* hitNode = (_currentSelection.refr == _savedObjectState.refr) ? _currentSelection.hitNode : nullptr;
                auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;

                struct NodeFrameMetrics
                {
                    RE::NiAVObject* node = nullptr;
                    RE::NiTransform world = {};
                    RE::NiTransform expectedWorld = {};
                    RE::NiPoint3 finger = {};
                    RE::NiPoint3 expectedFinger = {};
                    float rotErrDeg = -1.0f;
                    float posErrGameUnits = -1.0f;
                    bool hasCollisionObject = false;
                    bool ownsBodyCollisionObject = false;
                };

                const auto captureNodeMetrics = [&](RE::NiAVObject* node, const RE::NiTransform& bodyLocalTransform) {
                    NodeFrameMetrics metrics{};
                    metrics.node = node;
                    metrics.world = node ? node->world : liveBodyWorld;
                    metrics.expectedWorld = node ? deriveNodeWorldFromBodyWorld(liveBodyWorld, bodyLocalTransform) : liveBodyWorld;
                    metrics.finger = getMatrixColumn(metrics.world.rotate, 0);
                    metrics.expectedFinger = getMatrixColumn(metrics.expectedWorld.rotate, 0);
                    if (node) {
                        metrics.rotErrDeg = rotationDeltaDegrees(metrics.world.rotate, metrics.expectedWorld.rotate);
                        metrics.posErrGameUnits = translationDeltaGameUnits(metrics.world, metrics.expectedWorld);
                        auto* nodeCollisionObject = node->collisionObject.get();
                        metrics.hasCollisionObject = (nodeCollisionObject != nullptr);
                        metrics.ownsBodyCollisionObject = (nodeCollisionObject != nullptr && nodeCollisionObject == bodyCollisionObject);
                    }
                    return metrics;
                };

                const NodeFrameMetrics ownerMetrics = captureNodeMetrics(ownerNode, _grabFrame.ownerBodyLocal);
                const NodeFrameMetrics hitMetrics = captureNodeMetrics(hitNode, _grabFrame.bodyLocal);
                const NodeFrameMetrics heldMetrics = captureNodeMetrics(_grabFrame.heldNode, _grabFrame.bodyLocal);
                const NodeFrameMetrics rootMetrics = captureNodeMetrics(rootNode, _grabFrame.rootBodyLocal);

                const RE::NiPoint3 worldPosDelta = desiredNodeWorldConstraint.translate - desiredNodeWorldRaw.translate;
                const RE::NiPoint3 rawFinger = getMatrixColumn(handWorldTransform.rotate, 2);
                const RE::NiPoint3 constraintFinger = getMatrixColumn(liveHandBodyWorld.rotate, 2);
                const RE::NiPoint3 desiredRawFinger = getMatrixColumn(desiredBodyWorldRaw.rotate, 0);
                const RE::NiPoint3 desiredConstraintFinger = getMatrixColumn(desiredBodyWorldConstraint.rotate, 0);
                const RE::NiPoint3 bodyFinger = getMatrixColumn(liveBodyWorld.rotate, 0);
                const float rawRowMax = max3(axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldRaw.rotate, 2)));
                const float rawColMax = max3(axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldRaw.rotate, 2)));
                const float constraintRow0 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldConstraint.rotate, 0));
                const float constraintRow1 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldConstraint.rotate, 1));
                const float constraintRow2 = axisDeltaDegrees(getMatrixRow(liveBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldConstraint.rotate, 2));
                const float constraintCol0 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldConstraint.rotate, 0));
                const float constraintCol1 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldConstraint.rotate, 1));
                const float constraintCol2 = axisDeltaDegrees(getMatrixColumn(liveBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldConstraint.rotate, 2));
                const float constraintRowMax = max3(constraintRow0, constraintRow1, constraintRow2);
                const float constraintColMax = max3(constraintCol0, constraintCol1, constraintCol2);

                auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
                auto* target_bRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + 0x10);
                auto* transformB_rot = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Col0);
                const RE::NiMatrix3 targetAsHkColumns = matrixFromHkColumns(target_bRca);
                const RE::NiMatrix3 targetAsHkRows = matrixFromHkRows(target_bRca);
                const RE::NiMatrix3 transformBAsHkColumns = matrixFromHkColumns(transformB_rot);
                const RE::NiMatrix3 constraintTargetInv = desiredBodyTransformHandSpaceConstraint.rotate.Transpose();
                const RE::NiMatrix3 rawTargetInv = desiredBodyTransformHandSpaceRaw.rotate.Transpose();
                const int ragdollMotorEnabled = *(constraintData + ATOM_RAGDOLL_MOT + 0x02) ? 1 : 0;

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME HOLD: rawVsConstraintWorld={:.2f}deg rawVsConstraintTarget={:.2f}deg "
                    "bodyVsRaw={:.2f}deg bodyVsConstraint={:.2f}deg "
                    "ownerErr={:.2f}deg/{:.2f}gu hitErr={:.2f}deg/{:.2f}gu "
                    "heldErr={:.2f}deg/{:.2f}gu rootErr={:.2f}deg/{:.2f}gu "
                    "worldPosDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) constraintFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), rotationDeltaDegrees(desiredNodeWorldRaw.rotate, desiredNodeWorldConstraint.rotate),
                    rotationDeltaDegrees(desiredBodyTransformHandSpaceRaw.rotate, desiredBodyTransformHandSpaceConstraint.rotate),
                    rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorldRaw.rotate), rotationDeltaDegrees(liveBodyWorld.rotate, desiredBodyWorldConstraint.rotate),
                    ownerMetrics.rotErrDeg, ownerMetrics.posErrGameUnits, hitMetrics.rotErrDeg, hitMetrics.posErrGameUnits, heldMetrics.rotErrDeg, heldMetrics.posErrGameUnits,
                    rootMetrics.rotErrDeg, rootMetrics.posErrGameUnits, worldPosDelta.x, worldPosDelta.y, worldPosDelta.z, rawFinger.x, rawFinger.y, rawFinger.z,
                    constraintFinger.x, constraintFinger.y, constraintFinger.z);

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB FRAME NODES: bodyColl={:p} "
                    "owner='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "hit='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "held='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "root='{}'({:p}) hasCol={} ownsBodyCol={} "
                    "sameOwnerHeld={} sameHitHeld={} sameRootHeld={}",
                    handName(), static_cast<const void*>(bodyCollisionObject), nodeDebugName(ownerMetrics.node), static_cast<const void*>(ownerMetrics.node),
                    ownerMetrics.hasCollisionObject ? "yes" : "no", ownerMetrics.ownsBodyCollisionObject ? "yes" : "no", nodeDebugName(hitMetrics.node),
                    static_cast<const void*>(hitMetrics.node), hitMetrics.hasCollisionObject ? "yes" : "no", hitMetrics.ownsBodyCollisionObject ? "yes" : "no",
                    nodeDebugName(heldMetrics.node), static_cast<const void*>(heldMetrics.node), heldMetrics.hasCollisionObject ? "yes" : "no",
                    heldMetrics.ownsBodyCollisionObject ? "yes" : "no", nodeDebugName(rootMetrics.node), static_cast<const void*>(rootMetrics.node),
                    rootMetrics.hasCollisionObject ? "yes" : "no", rootMetrics.ownsBodyCollisionObject ? "yes" : "no", ownerMetrics.node == heldMetrics.node ? "yes" : "no",
                    hitMetrics.node == heldMetrics.node ? "yes" : "no", rootMetrics.node == heldMetrics.node ? "yes" : "no");

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB FRAME VISUALS: desiredRawFinger=({:.3f},{:.3f},{:.3f}) desiredConstraintFinger=({:.3f},{:.3f},{:.3f}) "
                    "bodyFinger=({:.3f},{:.3f},{:.3f}) "
                    "ownerFinger=({:.3f},{:.3f},{:.3f}) ownerExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "hitFinger=({:.3f},{:.3f},{:.3f}) hitExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "heldFinger=({:.3f},{:.3f},{:.3f}) heldExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "rootFinger=({:.3f},{:.3f},{:.3f}) rootExpectedFinger=({:.3f},{:.3f},{:.3f}) "
                    "targetInvFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), desiredRawFinger.x, desiredRawFinger.y, desiredRawFinger.z, desiredConstraintFinger.x, desiredConstraintFinger.y, desiredConstraintFinger.z,
                    bodyFinger.x, bodyFinger.y, bodyFinger.z, ownerMetrics.finger.x, ownerMetrics.finger.y, ownerMetrics.finger.z, ownerMetrics.expectedFinger.x,
                    ownerMetrics.expectedFinger.y, ownerMetrics.expectedFinger.z, hitMetrics.finger.x, hitMetrics.finger.y, hitMetrics.finger.z, hitMetrics.expectedFinger.x,
                    hitMetrics.expectedFinger.y, hitMetrics.expectedFinger.z, heldMetrics.finger.x, heldMetrics.finger.y, heldMetrics.finger.z, heldMetrics.expectedFinger.x,
                    heldMetrics.expectedFinger.y, heldMetrics.expectedFinger.z, rootMetrics.finger.x, rootMetrics.finger.y, rootMetrics.finger.z, rootMetrics.expectedFinger.x,
                    rootMetrics.expectedFinger.y, rootMetrics.expectedFinger.z, invRot.entry[0][0], invRot.entry[1][0], invRot.entry[2][0]);

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB ANGULAR PROBE: rawAxisErr(rowMax={:.2f} colMax={:.2f}) "
                    "constraintAxisErr(rowMax={:.2f} colMax={:.2f} rows=({:.2f},{:.2f},{:.2f}) cols=({:.2f},{:.2f},{:.2f})) "
                    "targetMem(colsVsConstraintInv={:.2f} rowsVsConstraintInv={:.2f} "
                    "colsVsConstraintForward={:.2f} colsVsRawInv={:.2f} colsVsTransformB={:.2f}) "
                    "motorErr=({:.2f}gu,{:.2f}deg,{:.2f}) mass={:.2f} "
                    "fade={:.2f} fadeEnabled={} fadeReason={} "
                    "ragEnabled={} angMotor={:p} angTau={:.3f} linTau={:.3f} linF={:.0f} angF={:.0f}",
                    handName(), rawRowMax, rawColMax, constraintRowMax, constraintColMax, constraintRow0, constraintRow1, constraintRow2, constraintCol0, constraintCol1,
                    constraintCol2, rotationDeltaDegrees(targetAsHkColumns, constraintTargetInv), rotationDeltaDegrees(targetAsHkRows, constraintTargetInv),
                    rotationDeltaDegrees(targetAsHkColumns, desiredBodyTransformHandSpaceConstraint.rotate), rotationDeltaDegrees(targetAsHkColumns, rawTargetInv),
                    rotationDeltaDegrees(targetAsHkColumns, transformBAsHkColumns), grabPositionErrorGameUnits, grabRotationErrorDegrees, motorTargets.errorFactor, motorInput.mass,
                    motorTargets.fadeFactor, motorInput.fadeInEnabled ? "yes" : "no", _grabFrame.motorFadeReason,
                    ragdollMotorEnabled, static_cast<const void*>(_activeConstraint.angularMotor), currentAngularTau, currentLinearTau, currentLinearForce, currentAngularForce);
            }

            auto* cd = static_cast<char*>(_activeConstraint.constraintData);
            auto* pivotA_local = reinterpret_cast<float*>(cd + offsets::kTransformA_Pos);
            auto* pivotB_local = reinterpret_cast<float*>(cd + offsets::kTransformB_Pos);
            const RE::NiPoint3 pivotALocalGame{ pivotA_local[0] * havokToGameScale(), pivotA_local[1] * havokToGameScale(), pivotA_local[2] * havokToGameScale() };
            const RE::NiPoint3 pivotBLocalGame{ pivotB_local[0] * havokToGameScale(), pivotB_local[1] * havokToGameScale(), pivotB_local[2] * havokToGameScale() };

            RE::NiTransform liveHandBodyWorld{};
            RE::NiTransform liveObjectBodyWorld{};
            const bool hasLivePivotFrames = tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), liveHandBodyWorld) &&
                                            tryResolveLiveBodyWorldTransform(world, _savedObjectState.bodyId, liveObjectBodyWorld);
            const RE::NiPoint3 pivotAWorld = hasLivePivotFrames ? transform_math::localPointToWorld(liveHandBodyWorld, pivotALocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 pivotBWorld = hasLivePivotFrames ? transform_math::localPointToWorld(liveObjectBodyWorld, pivotBLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 pivotError = pivotAWorld - pivotBWorld;
            const float pivotErrGame = hasLivePivotFrames ? std::sqrt(pivotError.x * pivotError.x + pivotError.y * pivotError.y + pivotError.z * pivotError.z) : 0.0f;

            const RE::NiPoint3 bodyDelta = liveObjectBodyWorld.translate - liveHandBodyWorld.translate;
            const float bodyDistGame = hasLivePivotFrames ? std::sqrt(bodyDelta.x * bodyDelta.x + bodyDelta.y * bodyDelta.y + bodyDelta.z * bodyDelta.z) : 0.0f;

            float objVelMag = 0.0f;
            {
                auto* objMotion = havok_runtime::getBodyMotion(world, _savedObjectState.bodyId);
                if (objMotion) {
                    const auto& velocity = objMotion->linearVelocity;
                    objVelMag = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
                }
            }

            ROCK_LOG_DEBUG(Hand,
                "{} HELD: angTau={:.3f} linTau={:.3f} linF={:.0f} angF={:.0f} "
                "fade={:.2f}/{} reason={} ERR={:.1f}gu bDist={:.1f}gu objVel={:.3f} "
                "paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
                "handW=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
                handName(), currentAngularTau, currentLinearTau, currentLinearForce, currentAngularForce, motorTargets.fadeFactor, motorInput.fadeInEnabled ? "on" : "off",
                _grabFrame.motorFadeReason, pivotErrGame, bodyDistGame, objVelMag,
                pivotAWorld.x, pivotAWorld.y, pivotAWorld.z, pivotBWorld.x, pivotBWorld.y, pivotBWorld.z,
                liveHandBodyWorld.translate.x, liveHandBodyWorld.translate.y, liveHandBodyWorld.translate.z,
                liveObjectBodyWorld.translate.x, liveObjectBodyWorld.translate.y, liveObjectBodyWorld.translate.z);

            _notifCounter++;
            if (hasLivePivotFrames && g_rockConfig.rockDebugShowGrabNotifications && _notifCounter >= 6) {
                _notifCounter = 0;
                const RE::NiPoint3 pivotAToHand = pivotAWorld - liveHandBodyWorld.translate;
                const RE::NiPoint3 pivotBToObject = pivotBWorld - liveObjectBodyWorld.translate;
                const float paToHand = std::sqrt(pivotAToHand.x * pivotAToHand.x + pivotAToHand.y * pivotAToHand.y + pivotAToHand.z * pivotAToHand.z);
                const float pbToObj = std::sqrt(pivotBToObject.x * pivotBToObject.x + pivotBToObject.y * pivotBToObject.y + pivotBToObject.z * pivotBToObject.z);
                f4vr::showNotification(
                    std::format("[ROCK] err={:.1f}gu F={:.0f} vel={:.2f} paOff={:.1f} pbOff={:.1f}", pivotErrGame, currentLinearForce, objVelMag, paToHand, pbToObj));
            }
        }

        {
            const auto wakeBodies = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(_savedObjectState.bodyId.value, _heldBodyIds);
            for (const auto bodyId : wakeBodies) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
        }
    }

    void Hand::releaseGrabbedObject(RE::hknpWorld* world, GrabReleaseCollisionRestoreMode collisionRestoreMode)
    {
        if (!isHolding())
            return;

        ROCK_LOG_INFO(Hand, "{} hand RELEASE: bodyId={} constraintId={}", handName(), _savedObjectState.bodyId.value,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu);

        nearby_grab_damping::restoreNearbyGrabDamping(world, _nearbyGrabDamping);

        if (world && _heldLocalLinearVelocityHistoryCount > 0) {
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedHistory{};
            const std::size_t historySize = _heldLocalLinearVelocityHistory.size();
            const std::size_t firstIndex = (_heldLocalLinearVelocityHistoryNext + historySize - _heldLocalLinearVelocityHistoryCount) % historySize;
            for (std::size_t i = 0; i < _heldLocalLinearVelocityHistoryCount; ++i) {
                orderedHistory[i] = _heldLocalLinearVelocityHistory[(firstIndex + i) % historySize];
            }

            const RE::NiPoint3 localReleaseVelocity = held_object_physics_math::maxMagnitudeVelocity(orderedHistory, _heldLocalLinearVelocityHistoryCount);
            const RE::NiPoint3 releaseVelocity =
                held_object_physics_math::composeReleaseVelocity(localReleaseVelocity, _lastPlayerSpaceVelocityHavok, g_rockConfig.rockThrowVelocityMultiplier);
            setHeldLinearVelocity(world, _savedObjectState.bodyId, _heldBodyIds, releaseVelocity);
            ROCK_LOG_DEBUG(Hand,
                "{} hand RELEASE VELOCITY: local=({:.3f},{:.3f},{:.3f}) player=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) history={} multiplier={:.2f}",
                handName(), localReleaseVelocity.x, localReleaseVelocity.y, localReleaseVelocity.z, _lastPlayerSpaceVelocityHavok.x, _lastPlayerSpaceVelocityHavok.y,
                _lastPlayerSpaceVelocityHavok.z, releaseVelocity.x, releaseVelocity.y, releaseVelocity.z, _heldLocalLinearVelocityHistoryCount,
                g_rockConfig.rockThrowVelocityMultiplier);
        }

        restoreGrabbedInertia(world, _savedObjectState);

        if (world && _activeGrabLifecycle.size() > 0) {
            restoreActiveGrabLifecycle(world,
                _activeGrabLifecycle,
                _activeGrabLifecycle.restorePlanForRelease(active_grab_body_lifecycle::BodyRestorePolicy::ProtectComplexSystemOwned),
                _savedObjectState.bodyId.value,
                handName(),
                "release");
        }

        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        _heldBodyContactFrame.store(100, std::memory_order_release);

        if (world) {
            typedef void disableBodyFlags_t(void*, std::uint32_t, std::uint32_t, std::uint32_t);
            static REL::Relocation<disableBodyFlags_t> disableBodyFlags{ REL::Offset(offsets::kFunc_DisableBodyFlags) };
            for (auto bid : _heldBodyIds) {
                disableBodyFlags(world, bid, 0x80, 0);
            }
        }

        if (_activeConstraint.isValid()) {
            destroyGrabConstraint(world, _activeConstraint);
        }

        const bool delayRestore = collisionRestoreMode == GrabReleaseCollisionRestoreMode::Delayed &&
                                  hand_collision_suppression_math::beginDelayedRestore(
                                      _grabHandCollisionDelayedRestore, _grabHandCollisionSuppression, g_rockConfig.rockGrabReleaseHandCollisionDelaySeconds);
        if (delayRestore) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand: grab hand collision restore delayed bodies={} firstBodyId={} seconds={:.3f}",
                handName(),
                _grabHandCollisionDelayedRestore.bodyCount,
                _grabHandCollisionDelayedRestore.bodyId,
                _grabHandCollisionDelayedRestore.remainingSeconds);
        } else {
            restoreHandCollisionAfterGrab(world);
        }

        if (frik::api::FRIKApi::inst) {
            frik::api::FRIKApi::inst->clearHandPose("ROCK_Grab", handFromBool(_isLeft));
        }
        clearGrabExternalHandWorldTransform(_isLeft);
        clearSelectedCloseFingerPose();
        _savedObjectState.clear();
        _activeGrabLifecycle.clear();
        _activeConstraint.clear();
        _heldBodyIds.clear();
        _grabFrame.clear();
        _adjustedHandTransform = RE::NiTransform();
        _hasAdjustedHandTransform = false;
        _grabVisualLerpElapsed = 0.0f;
        _grabVisualLerpDuration = g_rockConfig.rockGrabLerpMaxTime;
        _grabDeviationExceededSeconds = 0.0f;
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _grabFingerPoseFrameCounter = 0;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _lastPlayerSpaceVelocityHavok = {};
        _currentSelection.clear();
        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::ReleaseRequested });

        ROCK_LOG_DEBUG(Hand, "{} hand: Idle", handName());
    }
}

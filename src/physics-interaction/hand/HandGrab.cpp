#include "physics-interaction/hand/Hand.h"

#include "physics-interaction/native/HavokOffsets.h"

#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugPivotMath.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/PhysicsShapeCast.h"
#include "physics-interaction/native/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/PhysicsUtils.h"
#include "physics-interaction/native/PhysicsScale.h"
#include "physics-interaction/native/HavokMaterialRegistry.h"
#include "physics-interaction/native/HavokRefCount.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "api/FRIKApi.h"
#include "f4vr/F4VRUtils.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <algorithm>
#include <array>
#include <limits>
#include <string_view>
#include <xmmintrin.h>

namespace rock
{
    namespace
    {
        const char* releaseDispositionName(GrabReleaseDisposition disposition) noexcept
        {
            switch (disposition) {
            case GrabReleaseDisposition::PhysicalDrop:
                return "physical-drop";
            case GrabReleaseDisposition::PendingInventoryTransfer:
                return "pending-inventory-transfer";
            case GrabReleaseDisposition::TransferToInventory:
                return "transfer-to-inventory";
            case GrabReleaseDisposition::OwnershipHandoff:
                return "ownership-handoff";
            }
            return "unknown";
        }

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

        float lengthSquared(const RE::NiPoint3& value)
        {
            return value.x * value.x + value.y * value.y + value.z * value.z;
        }

        RE::NiPoint3 crossProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return RE::NiPoint3{
                lhs.y * rhs.z - lhs.z * rhs.y,
                lhs.z * rhs.x - lhs.x * rhs.z,
                lhs.x * rhs.y - lhs.y * rhs.x,
            };
        }

        float dotProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
        {
            return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
        }

        RE::NiPoint3 scalePoint(const RE::NiPoint3& value, float scalar)
        {
            return RE::NiPoint3{ value.x * scalar, value.y * scalar, value.z * scalar };
        }

        RE::NiPoint3 gamePointToHavokPoint(const RE::NiPoint3& value)
        {
            const float scale = physics_scale::gameToHavok();
            return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
        }

        RE::NiMatrix3 storedRotationFromConventionalRows(const RE::NiPoint3 rows[3])
        {
            RE::NiMatrix3 result{};
            result.entry[0][0] = rows[0].x;
            result.entry[0][1] = rows[1].x;
            result.entry[0][2] = rows[2].x;
            result.entry[1][0] = rows[0].y;
            result.entry[1][1] = rows[1].y;
            result.entry[1][2] = rows[2].y;
            result.entry[2][0] = rows[0].z;
            result.entry[2][1] = rows[1].z;
            result.entry[2][2] = rows[2].z;
            return result;
        }

        RE::NiMatrix3 axisAngleStored(const RE::NiPoint3& axisRaw, float angle)
        {
            const RE::NiPoint3 axis = normalizeOrZero(axisRaw);
            if (lengthSquared(axis) <= 0.000001f || !std::isfinite(angle)) {
                return transform_math::makeIdentityRotation<RE::NiMatrix3>();
            }

            const float x = axis.x;
            const float y = axis.y;
            const float z = axis.z;
            const float cosTheta = std::cos(angle);
            const float sinTheta = std::sin(angle);
            const float oneMinusCos = 1.0f - cosTheta;

            const RE::NiPoint3 rows[3]{
                RE::NiPoint3{ cosTheta + x * x * oneMinusCos, x * y * oneMinusCos - z * sinTheta, x * z * oneMinusCos + y * sinTheta },
                RE::NiPoint3{ y * x * oneMinusCos + z * sinTheta, cosTheta + y * y * oneMinusCos, y * z * oneMinusCos - x * sinTheta },
                RE::NiPoint3{ z * x * oneMinusCos - y * sinTheta, z * y * oneMinusCos + x * sinTheta, cosTheta + z * z * oneMinusCos },
            };
            return storedRotationFromConventionalRows(rows);
        }

        RE::NiMatrix3 applyWorldRotationToStoredBasis(const RE::NiMatrix3& worldRotationStored, const RE::NiMatrix3& baseRotation)
        {
            RE::NiMatrix3 result{};
            for (int axis = 0; axis < 3; ++axis) {
                const RE::NiPoint3 basis{ baseRotation.entry[axis][0], baseRotation.entry[axis][1], baseRotation.entry[axis][2] };
                const RE::NiPoint3 rotated = transform_math::rotateLocalVectorToWorld(worldRotationStored, basis);
                result.entry[axis][0] = rotated.x;
                result.entry[axis][1] = rotated.y;
                result.entry[axis][2] = rotated.z;
            }
            return result;
        }

        RE::NiPoint3 angularVelocityFromRotationDelta(const RE::NiMatrix3& previous, const RE::NiMatrix3& current, float deltaTime)
        {
            if (!std::isfinite(deltaTime) || deltaTime <= 0.000001f) {
                return RE::NiPoint3{};
            }

            RE::NiPoint3 axisSum{};
            for (int column = 0; column < 3; ++column) {
                axisSum = axisSum + crossProduct(getMatrixColumn(previous, column), getMatrixColumn(current, column));
            }

            const RE::NiPoint3 axis = normalizeOrZero(axisSum);
            if (lengthSquared(axis) <= 0.000001f) {
                return RE::NiPoint3{};
            }

            const float trace =
                previous.entry[0][0] * current.entry[0][0] + previous.entry[0][1] * current.entry[0][1] + previous.entry[0][2] * current.entry[0][2] +
                previous.entry[1][0] * current.entry[1][0] + previous.entry[1][1] * current.entry[1][1] + previous.entry[1][2] * current.entry[1][2] +
                previous.entry[2][0] * current.entry[2][0] + previous.entry[2][1] * current.entry[2][1] + previous.entry[2][2] * current.entry[2][2];
            const float angle = std::acos(std::clamp((trace - 1.0f) * 0.5f, -1.0f, 1.0f));
            if (!std::isfinite(angle) || angle <= 0.000001f) {
                return RE::NiPoint3{};
            }

            return scalePoint(axis, angle / deltaTime);
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

        float pointDistanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
        {
            const RE::NiPoint3 delta = a - b;
            return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
        }

        float looseWeaponMultiplier(bool looseWeaponGrab, float multiplier)
        {
            return looseWeaponGrab ? (std::isfinite(multiplier) ? multiplier : 1.0f) : 1.0f;
        }

        float scaleDriveValue(float value, float multiplier)
        {
            return (std::isfinite(value) ? value : 0.0f) * (std::isfinite(multiplier) ? multiplier : 1.0f);
        }

        float angularForceRatioForMultiplier(float ratio, float angularForceMultiplier)
        {
            const float sanitizedRatio = (std::isfinite(ratio) && ratio > 0.001f) ? ratio : 12.5f;
            const float sanitizedMultiplier = (std::isfinite(angularForceMultiplier) && angularForceMultiplier > 0.001f) ? angularForceMultiplier : 1.0f;
            return (std::max)(0.001f, sanitizedRatio / sanitizedMultiplier);
        }

        GrabConstraintMotorTuning buildSharedConstraintMotorTuning(
            float tau,
            float damping,
            float maxForce,
            float proportionalRecovery,
            float constantRecovery,
            bool looseWeaponGrab)
        {
            const float linearTauMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier);
            const float angularTauMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier);
            const float linearDampingMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier);
            const float angularDampingMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier);
            const float maxForceMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier);
            const float angularForceMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier);
            const float linearRecoveryMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
            const float angularRecoveryMultiplier =
                looseWeaponMultiplier(looseWeaponGrab, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);

            const float linearMaxForce = (std::max)(0.0f, scaleDriveValue(maxForce, maxForceMultiplier));
            const float angularForceRatio =
                angularForceRatioForMultiplier(g_rockConfig.rockGrabAngularToLinearForceRatio, angularForceMultiplier);

            return GrabConstraintMotorTuning{
                .linearTau = scaleDriveValue(tau, linearTauMultiplier),
                .linearDamping = scaleDriveValue(damping, linearDampingMultiplier),
                .linearProportionalRecovery = scaleDriveValue(proportionalRecovery, linearRecoveryMultiplier),
                .linearConstantRecovery = scaleDriveValue(constantRecovery, linearRecoveryMultiplier),
                .linearMaxForce = linearMaxForce,
                .angularTau = scaleDriveValue(g_rockConfig.rockGrabAngularTau, angularTauMultiplier),
                .angularDamping = scaleDriveValue(g_rockConfig.rockGrabAngularDamping, angularDampingMultiplier),
                .angularProportionalRecovery =
                    scaleDriveValue(g_rockConfig.rockGrabAngularProportionalRecovery, angularRecoveryMultiplier),
                .angularConstantRecovery =
                    scaleDriveValue(g_rockConfig.rockGrabAngularConstantRecovery, angularRecoveryMultiplier),
                .angularMaxForce = grab_motion_controller::angularForceFromRatio(linearMaxForce, angularForceRatio),
            };
        }

        bool sharedContextMatchesSelection(const GrabSharedObjectContext& sharedContext, const SelectedObject& selection)
        {
            return sharedContext.hasPeerState() && selection.refr && sharedContext.peerSavedObjectState->refr == selection.refr;
        }

        bool isLooseWeaponGrabTarget(const SelectedObject& selection)
        {
            if (!selection.refr || !grab_target::canUseRockActiveGrab(selection.targetKind)) {
                return false;
            }

            auto* selectedBase = selection.refr->GetObjectReference();
            return selectedBase && selectedBase->Is(RE::ENUM_FORM_ID::kWEAP);
        }

        const char* heldObjectDriveModeName(HeldObjectDriveMode mode)
        {
            switch (mode) {
            case HeldObjectDriveMode::ProxyConstraint:
                return "proxyConstraint";
            case HeldObjectDriveMode::SharedConstraint:
                return "sharedConstraint";
            case HeldObjectDriveMode::NativeMouseSpring:
            default:
                return "nativeMouseSpring";
            }
        }

        RE::NiPoint3 constraintDrivePivotBBodyLocalGame(const CanonicalGrabFrame& frame)
        {
            /*
             * ROCK writes the constraint's object-space transform-B pivot from
             * the same captured hand/body frame used for angular drive. The
             * native mouse-spring surface pivot remains for one-hand grabs, but
             * constraint grabs must measure and solve against this active pivot
             * so linear and angular two-hand authority do not disagree.
             */
            const RE::NiTransform desiredBodyTransformHandSpace =
                grab_frame_math::desiredBodyInHandBodySpace(frame.constraintHandSpace, frame.bodyLocal);
            return grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformHandSpace, frame.pivotAHandBodyLocalGame);
        }

        std::uintptr_t heldBodyFlagLeaseOwner(const Hand* hand)
        {
            return reinterpret_cast<std::uintptr_t>(hand) ^ 0x524F434B48454C44ull;
        }

        void copyPeerInertiaSnapshot(SavedObjectState& target, const SavedObjectState& peer)
        {
            target.savedPackedInertia[0] = peer.savedPackedInertia[0];
            target.savedPackedInertia[1] = peer.savedPackedInertia[1];
            target.savedPackedInertia[2] = peer.savedPackedInertia[2];
            target.inertiaModified = peer.inertiaModified;
            target.motionInertiaStates = peer.motionInertiaStates;
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
            result.signedAlongPalmDistanceGameUnits = selection.signedAlongDistance;
            result.lateralPalmDistanceGameUnits = selection.lateralDistance;
            result.hasShapeKey = selection.hasHitShapeKey;
            result.hasTriangle = false;
            result.valid = lengthSquared(result.normal) > 0.0f;
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

        grab_finger_pose_runtime::GrabFingerPoseTargetSet buildRuntimeFingerPoseTargets(
            const RE::NiPoint3& seatPointWorld,
            const RE::NiPoint3& seatNormalWorld)
        {
            auto targets = grab_finger_pose_runtime::makeSharedGripPoseTarget(seatPointWorld, seatNormalWorld);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = true;
            return targets;
        }

        void storeFingerPoseTargetsInGrabFrame(CanonicalGrabFrame& frame,
            const grab_finger_pose_runtime::GrabFingerPoseTargetSet& targets,
            const RE::NiTransform& objectWorldTransform)
        {
            frame.fingerPoseTargetLocal = {};
            frame.fingerPoseTargetNormalLocal = {};
            frame.fingerPoseTargetValid = {};
            frame.fingerPoseTargetNormalValid = {};
            frame.fingerPoseTargetCount = 0;
            for (std::size_t finger = 0; finger < targets.targets.size(); ++finger) {
                if (!targets.targetValid[finger]) {
                    continue;
                }
                frame.fingerPoseTargetLocal[finger] = transform_math::worldPointToLocal(objectWorldTransform, targets.targets[finger]);
                frame.fingerPoseTargetValid[finger] = 1;
                if (targets.targetNormalValid[finger]) {
                    frame.fingerPoseTargetNormalLocal[finger] = transform_math::worldVectorToLocal(objectWorldTransform, targets.targetNormals[finger]);
                    frame.fingerPoseTargetNormalValid[finger] = 1;
                }
                ++frame.fingerPoseTargetCount;
            }
        }

        grab_finger_pose_runtime::GrabFingerPoseTargetSet rebuildFingerPoseTargetsFromGrabFrame(
            const CanonicalGrabFrame& frame,
            const RE::NiTransform& currentNodeWorld)
        {
            const RE::NiPoint3 seatNormalWorld = frame.hasGripPoint ? transform_math::localVectorToWorld(currentNodeWorld, frame.gripNormalLocal) : RE::NiPoint3{};
            auto targets =
                grab_finger_pose_runtime::makeSharedGripPoseTarget(transform_math::localPointToWorld(currentNodeWorld, frame.gripPointLocal), seatNormalWorld);
            targets.useSeatPointForMissingTargets = false;
            targets.useWholeMeshForMissingTargets = true;
            for (std::size_t finger = 0; finger < frame.fingerPoseTargetLocal.size(); ++finger) {
                if (!frame.fingerPoseTargetValid[finger]) {
                    continue;
                }
                targets.targets[finger] = transform_math::localPointToWorld(currentNodeWorld, frame.fingerPoseTargetLocal[finger]);
                targets.targetValid[finger] = 1;
                if (frame.fingerPoseTargetNormalValid[finger]) {
                    targets.targetNormals[finger] = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, frame.fingerPoseTargetNormalLocal[finger]));
                    targets.targetNormalValid[finger] = 1;
                }
                ++targets.targetCount;
            }
            return targets;
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
                        hit,
                        g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits)) {
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
                palmBitangent = normalizeOrZero(crossProduct(palmNormal, palmTangent));
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
                "{} hand grab lifecycle audit {}: bodies={} converted={} restoredMotion={} restoredFilter={} dampingSnapshots={} inertiaSnapshots={} latePrepared={} incompleteScan={} primaryBody={}",
                handName ? handName : "?",
                context ? context : "",
                audit.bodyCount,
                audit.convertedCount,
                audit.restoredMotionCount,
                audit.restoredFilterCount,
                audit.dampingSnapshotCount,
                audit.inertiaSnapshotCount,
                audit.latePreparedBodyCount,
                audit.incompleteNativeScan ? "yes" : "no",
                audit.primaryBodyId);
            return audit;
        }

        bool restoreIncompleteActivePrepRoot(
            RE::NiAVObject* rootNode,
            std::uint16_t originalMotionPropsId,
            const char* handName,
            const char* context)
        {
            /*
             * Late-discovered bodies have already been touched by recursive
             * active prep, but their original per-body state was not captured.
             * Individual restore would manufacture state. When a scan is known
             * incomplete, restore the object root with the selected body's
             * original motion preset so the whole Fallout-owned system returns
             * to one coherent motion mode.
             */
            if (!rootNode) {
                return false;
            }

            const auto motionType = physics_body_classifier::motionTypeFromMotionPropertiesId(originalMotionPropsId);
            const bool restored = physics_recursive_wrappers::setMotionRecursive(
                rootNode,
                motionPresetFromMotionType(motionType, originalMotionPropsId),
                true,
                true,
                false);

            ROCK_LOG_WARN(Hand,
                "{} hand {}: recursive root restore after incomplete object scan root='{}' motionProps={} result={}",
                handName ? handName : "?",
                context ? context : "incomplete-scan",
                nodeDebugName(rootNode),
                originalMotionPropsId,
                restored ? "ok" : "failed");
            return restored;
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

        RE::NiAVObject* findAuthoredGrabNodeRecursive(RE::NiAVObject* root,
            std::string_view name,
            bool isLeft,
            bool rejectOppositeHandAnchor,
            int maxDepth = 12)
        {
            /*
             * ROCK gives authored grab nodes priority over mesh-derived contact
             * and keeps marker names out of mesh selection. A hand-side guard is
             * still required because FO4VR assets can legitimately carry both
             * ROCK:GrabR and ROCK:GrabL markers; the right hand should never bind
             * to the left marker because that silently flips authored poses.
             */
            auto* found = findNamedNodeRecursive(root, name, maxDepth);
            if (!found || !rejectOppositeHandAnchor) {
                return found;
            }

            const char* foundName = found->name.c_str();
            if (foundName && grab_node_name_policy::isOppositeHandGrabNodeName(foundName, isLeft)) {
                return nullptr;
            }
            return found;
        }

        RE::NiTransform getLiveBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryResolveLiveBodyWorldTransform(world, bodyId, result);
            return result;
        }

        bool tryGetNativeMouseSpringBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
        {
            /*
             * FO4VR's hknpBSMouseSpringAction and the held visual node follow the
             * hknp body-array slot. The motion frame remains useful for COM/weight
             * diagnostics, but native dynamic grab pivots and body targets must use
             * this BODY frame or the object can converge to the hand and then rotate
             * from a different convention.
             */
            outTransform = makeIdentityTransform();
            return tryGetBodyArrayWorldTransform(world, bodyId, outTransform);
        }

        bool tryGetGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId, RE::NiTransform& outTransform)
        {
            /*
             * Dynamic grab object-side state is measured from the hknp BODY slot.
             * The hand side may be a native action target, generated collider, or
             * hidden proxy depending on drive mode, but the held object's contact
             * pivot and visual node relation must stay in BODY space. MOTION is
             * COM/weight/diagnostic data only for this path.
             */
            outTransform = makeIdentityTransform();
            return tryGetNativeMouseSpringBodyWorldTransform(world, bodyId, outTransform);
        }

        RE::NiTransform getGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId)
        {
            RE::NiTransform result = makeIdentityTransform();
            tryGetGrabAuthorityBodyWorldTransform(world, bodyId, result);
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

        void logGrabNodeInfo(const char* handName,
            bool isLeft,
            const RE::NiAVObject* parentNode,
            const RE::NiAVObject* authoredGrabNode,
            const RE::NiTransform& desiredObjectWorld,
            const RE::NiTransform& handWorldTransform,
            const RE::NiPoint3& grabPivotAWorld,
            const char* grabPointMode)
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
                "Source: authoredNode={} authoredName={} pointMode={}",
                authoredGrabNode ? "yes" : "no",
                authoredGrabNode ? nodeDebugName(authoredGrabNode) : "none",
                grabPointMode ? grabPointMode : "unknown");
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
            const RE::NiPoint3& previousPlayerSpaceVelocityHavok)
        {
            HeldMotionCompensationResult result{};
            if (!world) {
                return result;
            }

            const bool applyPlayerSpaceVelocity = playerSpaceFrame.enabled && !playerSpaceFrame.warp;
            const RE::NiPoint3 previousPlayerVelocity = applyPlayerSpaceVelocity ? previousPlayerSpaceVelocityHavok : RE::NiPoint3{};

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

        void setHeldVelocity(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const RE::NiPoint3& linearVelocity,
            const RE::NiPoint3& angularVelocity,
            bool overrideAngularVelocity,
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
                const RE::hkVector4f angularHavok = overrideAngularVelocity ?
                    RE::hkVector4f{ angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f } :
                    RE::hkVector4f{ motion->angularVelocity.x * angularKeep, motion->angularVelocity.y * angularKeep, motion->angularVelocity.z * angularKeep, 0.0f };
                havok_runtime::setBodyVelocityDeferred(world,
                    bodyId,
                    RE::hkVector4f{ linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f },
                    angularHavok);
            };

            setBody(primaryBodyId.value);
            for (const auto bodyId : heldBodyIds) {
                setBody(bodyId);
            }
        }

        void setHeldLinearVelocity(RE::hknpWorld* world,
            RE::hknpBodyId primaryBodyId,
            const std::vector<std::uint32_t>& heldBodyIds,
            const RE::NiPoint3& linearVelocity,
            float angularVelocityKeep = 1.0f)
        {
            setHeldVelocity(world, primaryBodyId, heldBodyIds, linearVelocity, RE::NiPoint3{}, false, angularVelocityKeep);
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
            float deltaTime,
            bool publishLocalTransforms = true)
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

            if (!g_rockConfig.rockGrabMeshFingerPoseEnabled) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
                syncLocalTransformState();
                if (api->clearHandPose) {
                    api->clearHandPose("ROCK_Grab", hand);
                }
                return;
            }

            if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved && fingerPose.hasJointValues && api->setHandPoseCustomJointPositionsWithPriority) {
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
                        .enabled = publishLocalTransforms && g_rockConfig.rockGrabMeshLocalTransformPoseEnabled,
                        .smoothingSpeed = g_rockConfig.rockGrabFingerLocalTransformSmoothingSpeed,
                        .maxCorrectionDegrees = g_rockConfig.rockGrabFingerLocalTransformMaxCorrectionDegrees,
                        .surfaceAimStrength = g_rockConfig.rockGrabFingerSurfaceAimStrength,
                        .thumbOppositionStrength = g_rockConfig.rockGrabThumbOppositionStrength,
                        .thumbAlternateCurveStrength = g_rockConfig.rockGrabThumbAlternateCurveStrength,
                        .thumbSurfaceSafetyEnabled = g_rockConfig.rockGrabThumbSurfaceSafetyEnabled,
                        .thumbSurfaceSafetyMarginGameUnits = g_rockConfig.rockGrabThumbSurfaceSafetyMarginGameUnits,
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

            if (fingerPose.solved && api->setHandPoseCustomFingerPositionsWithPriority) {
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
            const float fallbackMin =
                std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f, 0.0f, 1.0f);
            const float configuredFallback =
                std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f;
            const float fallbackValue = std::clamp(configuredFallback, fallbackMin, 1.0f);
            if (api->setHandPoseCustomFingerPositionsWithPriority) {
                api->setHandPoseCustomFingerPositionsWithPriority("ROCK_Grab", hand, fallbackValue, fallbackValue, fallbackValue, fallbackValue, fallbackValue, 100);
            } else if (api->setHandPoseWithPriority) {
                api->setHandPoseWithPriority("ROCK_Grab", hand, frik::api::FRIKApi::HandPoses::Fist, 100);
            }
            if (g_rockConfig.rockDebugGrabFrameLogging) {
                ROCK_LOG_DEBUG(Hand,
                    "{} hand FINGER POSE: using selected-close fallback value={:.2f} solved={} hits={} candidateTris={}",
                    isLeft ? "Left" : "Right",
                    fallbackValue,
                    fingerPose.solved ? "yes" : "no",
                    fingerPose.hitCount,
                    fingerPose.candidateTriangleCount);
            }
        }

        grab_finger_pose_runtime::SolvedGrabFingerPose buildAcquisitionFingerPose(
            const grab_finger_pose_runtime::SolvedGrabFingerPose& targetPose,
            float progress)
        {
            auto pose = targetPose;
            const float t = std::clamp(std::isfinite(progress) ? progress : 0.0f, 0.0f, 1.0f);
            const float precloseValue =
                std::clamp(std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue) ? g_rockConfig.rockSelectedCloseFingerAnimValue : 0.9f, 0.0f, 1.0f);

            for (std::size_t finger = 0; finger < pose.values.size(); ++finger) {
                const float solvedValue =
                    targetPose.solved ? std::clamp(targetPose.values[finger], 0.0f, 1.0f) : precloseValue;
                const float closingTarget = (std::min)(precloseValue, solvedValue);
                pose.values[finger] = precloseValue + (closingTarget - precloseValue) * t;
            }

            pose.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(pose.values);
            pose.solved = true;
            pose.hasJointValues = true;
            return pose;
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

    RE::NiPoint3 Hand::activeGrabDrivePivotBBodyLocalGame() const
    {
        if (_heldDriveMode == HeldObjectDriveMode::ProxyConstraint && _grabAuthorityProxyFrameValid) {
            return _grabAuthorityPivotBBodyLocalGame;
        }
        return _grabFrame.pivotBBodyLocalGame;
    }

    void Hand::clearGrabAuthorityProxyRuntimeLocked()
    {
        _grabAuthorityProxyBhkWorld = nullptr;
        _grabAuthorityProxyHknpWorld = nullptr;
        _grabAuthorityPivotAProxyLocalGame = {};
        _grabAuthorityPivotBBodyLocalGame = {};
        _grabAuthorityProxyFrameValid = false;
        _grabAuthorityPendingTarget = {};
        _lastAppliedGrabAuthorityProxyWorld = {};
        _hasLastAppliedGrabAuthorityProxyWorld = false;
        _grabAuthorityProxyFlushSequence = 0;
        _grabAuthorityProxyLogCounter = 0;
        _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
    }

    void Hand::clearGrabAuthorityProxyRuntime()
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        clearGrabAuthorityProxyRuntimeLocked();
    }

    void Hand::destroyGrabAuthorityProxyLocked(RE::bhkWorld* bhkWorld)
    {
        auto* destroyWorld = bhkWorld ? bhkWorld : _grabAuthorityProxyBhkWorld;
        if (_grabAuthorityProxy.isValid()) {
            _grabAuthorityProxy.destroy(destroyWorld);
        } else {
            _grabAuthorityProxy.reset();
        }
        clearGrabAuthorityProxyRuntimeLocked();
    }

    void Hand::destroyGrabAuthorityProxy(RE::bhkWorld* bhkWorld)
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        destroyGrabAuthorityProxyLocked(bhkWorld);
    }

    void Hand::abandonGrabAuthorityProxyLocked()
    {
        _grabAuthorityProxy.reset();
        clearGrabAuthorityProxyRuntimeLocked();
    }

    void Hand::abandonGrabAuthorityProxy()
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        abandonGrabAuthorityProxyLocked();
    }

    bool Hand::tryGetHeldObjectGrabPivotWorld(RE::hknpWorld* world, RE::NiPoint3& outPivotWorld) const
    {
        outPivotWorld = {};

        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform objectBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, objectBodyWorld)) {
            return false;
        }

        outPivotWorld = transform_math::localPointToWorld(objectBodyWorld, activeGrabDrivePivotBBodyLocalGame());
        return std::isfinite(outPivotWorld.x) && std::isfinite(outPivotWorld.y) && std::isfinite(outPivotWorld.z);
    }

    bool Hand::getGrabPivotDebugSnapshot(RE::hknpWorld* world, GrabPivotDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform objectBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, objectBodyWorld)) {
            return false;
        }

        if (_nativeGrab.isValid()) {
            out.handPivotWorld = _nativeGrab.targetPointWorldGame();
            out.objectPivotWorld = transform_math::localPointToWorld(objectBodyWorld, activeGrabDrivePivotBBodyLocalGame());
            out.handBodyWorld = out.handPivotWorld;
            if (_handBody.isValid() && _handBody.getBodyId().value != INVALID_BODY_ID) {
                RE::NiTransform handBodyWorld{};
                if (tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyWorld)) {
                    out.handBodyWorld = handBodyWorld.translate;
                }
            }
            out.objectBodyWorld = objectBodyWorld.translate;

            const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
            out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
            return true;
        }

        if (!_activeConstraint.isValid() || !_activeConstraint.constraintData) {
            return false;
        }

        RE::NiTransform anchorBodyWorld{};
        if (_heldDriveMode == HeldObjectDriveMode::ProxyConstraint) {
            if (!_grabAuthorityProxy.isValid() || _grabAuthorityProxy.getBodyId().value == INVALID_BODY_ID ||
                !tryResolveLiveBodyWorldTransform(world, _grabAuthorityProxy.getBodyId(), anchorBodyWorld)) {
                return false;
            }
        } else if (_handBody.isValid() && _handBody.getBodyId().value != INVALID_BODY_ID) {
            if (!tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), anchorBodyWorld)) {
                return false;
            }
        } else {
            return false;
        }

        auto* constraintData = static_cast<const char*>(_activeConstraint.constraintData);
        auto* pivotALocal = reinterpret_cast<const float*>(constraintData + offsets::kTransformA_Pos);
        auto* pivotBLocal = reinterpret_cast<const float*>(constraintData + offsets::kTransformB_Pos);
        const RE::NiPoint3 pivotALocalGame{ pivotALocal[0] * havokToGameScale(), pivotALocal[1] * havokToGameScale(), pivotALocal[2] * havokToGameScale() };
        const RE::NiPoint3 pivotBLocalGame{ pivotBLocal[0] * havokToGameScale(), pivotBLocal[1] * havokToGameScale(), pivotBLocal[2] * havokToGameScale() };
        out.handPivotWorld = transform_math::localPointToWorld(anchorBodyWorld, pivotALocalGame);
        out.objectPivotWorld = transform_math::localPointToWorld(objectBodyWorld, pivotBLocalGame);
        out.handBodyWorld = anchorBodyWorld.translate;
        out.objectBodyWorld = objectBodyWorld.translate;

        const RE::NiPoint3 error = out.handPivotWorld - out.objectPivotWorld;
        out.pivotErrorGameUnits = std::sqrt(error.x * error.x + error.y * error.y + error.z * error.z);
        return true;
    }

    bool Hand::getGrabPocketNormalDebugSnapshot(RE::hknpWorld* world, GrabPocketNormalDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.hasGripPoint || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform grabBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
            return false;
        }
        const RE::NiTransform currentNodeWorld = deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.bodyLocal);
        out.contactPointWorld = transform_math::localPointToWorld(currentNodeWorld, _grabFrame.gripPointLocal);

        const RE::NiPoint3 normalLocal = _grabFrame.gripNormalLocal;
        const RE::NiPoint3 normalWorld = normalizeOrZero(transform_math::localVectorToWorld(currentNodeWorld, normalLocal));
        if (normalWorld.x == 0.0f && normalWorld.y == 0.0f && normalWorld.z == 0.0f) {
            return false;
        }

        constexpr float kFrameAxisLengthGameUnits = 12.0f;
        out.normalEndWorld = out.contactPointWorld + normalWorld * kFrameAxisLengthGameUnits;

        return true;
    }

    bool Hand::getGrabContactPatchDebugSnapshot(RE::hknpWorld* world, GrabContactPatchDebugSnapshot& out) const
    {
        out = {};

        if (!world || !isHolding() || !_grabFrame.hasContactPatch || _grabFrame.contactPatchSampleCount == 0 || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform grabBodyWorld{};
        if (!tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
            return false;
        }
        const std::uint32_t count = (std::min)(_grabFrame.contactPatchSampleCount, static_cast<std::uint32_t>(out.samplePointsWorld.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            out.samplePointsWorld[i] = transform_math::localPointToWorld(grabBodyWorld, _grabFrame.contactPatchSamples[i].point);
        }
        out.sampleCount = count;
        return count > 0;
    }

    bool Hand::getGrabTransformTelemetrySnapshot(RE::hknpWorld* world,
        const RE::NiTransform& rawHandWorld,
        grab_transform_telemetry::RuntimeSample& out) const
    {
        out = {};
        if (!world || !isHolding() || _savedObjectState.bodyId.value == INVALID_BODY_ID) {
            return false;
        }

        out.valid = true;
        out.isLeft = _isLeft;
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
        out.heldBodyMass = readBodyMass(world, _savedObjectState.bodyId);

        RE::NiTransform heldNativeBodyWorld{};
        if (tryGetNativeMouseSpringBodyWorldTransform(world, _savedObjectState.bodyId, heldNativeBodyWorld)) {
            out.heldNativeBodyWorld = heldNativeBodyWorld;
            out.hasHeldNativeBodyWorld = true;
            if (heldBodyFrame.valid) {
                out.heldNativeBodyToHeldBody = grab_transform_telemetry::measureTransformDelta(out.heldNativeBodyWorld, out.heldBodyWorld);
            }
        }

        if (heldBodyFrame.valid || out.hasHeldNativeBodyWorld) {
            const RE::NiTransform& bodyLocalAuthorityFrame = out.hasHeldNativeBodyWorld ? out.heldNativeBodyWorld : out.heldBodyWorld;
            out.heldBodyDerivedNodeWorld = deriveNodeWorldFromBodyWorld(bodyLocalAuthorityFrame, _grabFrame.bodyLocal);
            out.hasHeldBodyDerivedNodeWorld = true;
        }

        /*
         * Telemetry keeps the same split as runtime authority: the held visual
         * node is the scene graph source, while body-derived node shows what the
         * native BODY authority predicts from the frozen frame. In a correct
         * native dynamic grab those should be nearly identical; MOTION is logged
         * separately as COM/weight diagnostics.
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
            {
                const RE::NiPoint3 currentPalmAnchorWorld = computeGrabPivotAWorld(world, out.rawHandWorld);
                const RE::NiPoint3 currentDesiredGripWorld =
                    transform_math::localPointToWorld(out.currentRawDesiredObjectWorld, _grabFrame.gripPointLocal);
                out.relationPivotErr =
                    pointDistanceGameUnits(currentPalmAnchorWorld, currentDesiredGripWorld);
                out.rotationPreservedDeg =
                    rotationDeltaDegrees(_grabFrame.objectNodeWorldAtGrab.rotate, _grabFrame.desiredObjectWorldAtGrab.rotate);
                out.normalAuthority = false;
                out.authoredRotationAuthority =
                    _grabFrame.activeGrabPointMode && std::strcmp(_grabFrame.activeGrabPointMode, "authoredGrabNodeFrame") == 0;
                out.hasGrabRelationInvariants = true;
            }
            if (out.hasHeldNodeWorld) {
                out.heldNodeToDesiredObjectAtGrab =
                    grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.desiredObjectWorldAtGrab);
                out.heldNodeToRawDesiredObject =
                    grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.currentRawDesiredObjectWorld);
            }
            const auto nativeState = _nativeGrab.debugState();
            if (nativeState.hasTarget) {
                out.currentConstraintDesiredBodyWorld = nativeState.targetBodyWorldGame;
                out.currentConstraintDesiredObjectWorld = deriveNodeWorldFromBodyWorld(nativeState.targetBodyWorldGame, _grabFrame.bodyLocal);
                if (out.hasHeldNodeWorld) {
                    out.heldNodeToConstraintDesiredObject =
                        grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.currentConstraintDesiredObjectWorld);
                }
                if (out.hasHeldNativeBodyWorld) {
                    out.heldBodyToConstraintDesiredBody =
                        grab_transform_telemetry::measureTransformDelta(out.heldNativeBodyWorld, out.currentConstraintDesiredBodyWorld);
                } else if (out.hasHeldBodyWorld) {
                    out.heldBodyToConstraintDesiredBody =
                        grab_transform_telemetry::measureTransformDelta(out.heldBodyWorld, out.currentConstraintDesiredBodyWorld);
                }
                out.bodyTargetNodeErr =
                    grab_transform_telemetry::measureTransformDelta(out.currentConstraintDesiredObjectWorld, out.currentRawDesiredObjectWorld);
            } else if (out.hasHandBodyWorld) {
                out.currentConstraintDesiredObjectWorld =
                    grab_transform_telemetry::computeCurrentDesiredObjectFromFrame(out.handBodyWorld, _grabFrame.constraintHandSpace);
                out.currentConstraintDesiredBodyWorld =
                    grab_transform_telemetry::computeCurrentDesiredBodyFromHandBody(out.handBodyWorld, _grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
                if (out.hasHeldNodeWorld) {
                    out.heldNodeToConstraintDesiredObject =
                        grab_transform_telemetry::measureHeldNodeVsDesiredObject(out.heldNodeWorld, out.currentConstraintDesiredObjectWorld);
                }
                if (out.hasHeldBodyWorld) {
                    out.heldBodyToConstraintDesiredBody =
                        grab_transform_telemetry::measureTransformDelta(out.heldBodyWorld, out.currentConstraintDesiredBodyWorld);
                }
                out.bodyTargetNodeErr = grab_transform_telemetry::measureTransformDelta(
                    deriveNodeWorldFromBodyWorld(out.currentConstraintDesiredBodyWorld, _grabFrame.bodyLocal),
                    out.currentConstraintDesiredObjectWorld);
            }
        }
        if (out.hasHeldNodeWorld) {
            out.heldRelativeHandTargetWorld = grab_transform_telemetry::computeHeldRelativeHandTarget(out.heldNodeWorld, _grabFrame.rawHandSpace);
            out.hasHeldRelativeHandTarget = true;
            out.rawToHeldRelativeHandTarget = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.heldRelativeHandTargetWorld);
            out.rawToHeldRelativeHandTargetAxes = grab_transform_telemetry::axisAlignmentDots(out.rawHandWorld.rotate, out.heldRelativeHandTargetWorld.rotate);
            out.constraintReverseTargetWorld =
                grab_transform_telemetry::computeConstraintReverseTarget(out.heldNodeWorld, _grabFrame.constraintHandSpace);
            out.hasConstraintReverseTarget = true;
            out.rawToConstraintReverse = grab_transform_telemetry::measureTransformDelta(out.rawHandWorld, out.constraintReverseTargetWorld);
            out.rawToConstraintReverseAxes = grab_transform_telemetry::axisAlignmentDots(out.rawHandWorld.rotate, out.constraintReverseTargetWorld.rotate);
            if (out.hasHandBodyWorld) {
                out.handBodyToConstraintReverse = grab_transform_telemetry::measureTransformDelta(out.handBodyWorld, out.constraintReverseTargetWorld);
            }
            out.heldRelativeToConstraintReverse =
                grab_transform_telemetry::measureTransformDelta(out.heldRelativeHandTargetWorld, out.constraintReverseTargetWorld);
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

    bool Hand::createConstraintGrabDrive(RE::hknpWorld* world,
        RE::hknpBodyId objectBodyId,
        const RE::NiTransform& handWorldTransform,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        bool looseWeaponGrab,
        const char* reason)
    {
        /*
         * Loose-object two-hand grabbing uses one constraint per hand, both
         * attached to the same dynamic body, with the object-to-hand angular
         * frame written through the same convention used by createGrabConstraint.
         * Single-hand grabs keep the native mouse-spring path because FO4VR
         * already gives that path stable dynamic-body follow behavior; shared
         * grabs need full angular authority from both hands so the object can
         * pivot instead of only translating.
         */
        if (!world || objectBodyId.value == INVALID_BODY_ID || !_handBody.isValid() || _handBody.getBodyId().value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform handBodyWorld = _grabFrame.handBodyWorldAtGrab;
        if (!tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), handBodyWorld)) {
            handBodyWorld = _grabFrame.hasTelemetryCapture ? _grabFrame.handBodyWorldAtGrab : handWorldTransform;
        }

        const RE::NiPoint3 grabPivotAWorld = _grabFrame.hasTelemetryCapture ?
            transform_math::localPointToWorld(handBodyWorld, _grabFrame.pivotAHandBodyLocalGame) :
            computeGrabPivotAWorld(world, handWorldTransform);
        const RE::NiTransform desiredBodyTransformHandSpace =
            grab_frame_math::desiredBodyInHandBodySpace(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
        const RE::NiPoint3 activePivotBBodyLocalGame = constraintDrivePivotBBodyLocalGame(_grabFrame);

        const float gameToHkScale = gameToHavokScale();
        float pivotBBodyLocalHk[4]{
            activePivotBBodyLocalGame.x * gameToHkScale,
            activePivotBBodyLocalGame.y * gameToHkScale,
            activePivotBBodyLocalGame.z * gameToHkScale,
            0.0f,
        };

        const GrabConstraintMotorTuning motorTuning =
            buildSharedConstraintMotorTuning(tau, damping, maxForce, proportionalRecovery, constantRecovery, looseWeaponGrab);

        _activeConstraint = createGrabConstraint(world,
            _handBody.getBodyId(),
            objectBodyId,
            handBodyWorld,
            grabPivotAWorld,
            pivotBBodyLocalHk,
            desiredBodyTransformHandSpace,
            motorTuning);
        if (!_activeConstraint.isValid()) {
            ROCK_LOG_ERROR(Hand,
                "{} hand shared constraint grab drive failed: body={} handBody={} reason={}",
                handName(),
                objectBodyId.value,
                _handBody.getBodyId().value,
                reason ? reason : "unknown");
            return false;
        }

        _heldDriveMode = HeldObjectDriveMode::SharedConstraint;
        ROCK_LOG_DEBUG(Hand,
            "{} hand shared constraint grab drive: constraint={} looseWeapon={} handBody={} objBody={} linearTau={:.3f} angularTau={:.3f} linearDamping={:.2f} angularDamping={:.2f} linearForce={:.0f} angularForce={:.0f} reason={}",
            handName(),
            _activeConstraint.constraintId,
            looseWeaponGrab ? "yes" : "no",
            _handBody.getBodyId().value,
            objectBodyId.value,
            motorTuning.linearTau,
            motorTuning.angularTau,
            motorTuning.linearDamping,
            motorTuning.angularDamping,
            motorTuning.linearMaxForce,
            motorTuning.angularMaxForce,
            reason ? reason : "unknown");
        return true;
    }

    bool Hand::createProxyConstraintGrabDrive(RE::bhkWorld* bhkWorld,
        RE::hknpWorld* world,
        RE::hknpBodyId objectBodyId,
        const RE::NiTransform& proxyWorldTransform,
        const RE::NiPoint3& grabPivotAWorld,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        bool looseWeaponGrab,
        const char* reason)
    {
        /*
         * The proxy path keeps the working ROCK contact/palm relation but moves
         * the solver anchor off the semantic hand collider. Body A is a hidden
         * keyframed no-contact proxy driven from the root-flattened hand frame
         * in the physics between phase; body B remains the dynamic held object.
         * This preserves the non-COM pivot while letting finite motors express
         * mass, collision, angular lag, and loose-weapon weight.
         */
        if (!bhkWorld || !world || objectBodyId.value == INVALID_BODY_ID) {
            return false;
        }

        if (_grabAuthorityProxy.isValid()) {
            destroyGrabAuthorityProxy(bhkWorld);
        }

        auto* proxyShape = grab_authority_proxy::buildProxyShape();
        if (!proxyShape) {
            ROCK_LOG_ERROR(Hand, "{} hand proxy constraint grab failed: proxy shape creation failed reason={}", handName(), reason ? reason : "unknown");
            return false;
        }

        const std::uint32_t proxyFilterInfo = grab_authority_proxy::noContactFilterInfo();
        const auto material = havok_material_registry::registerGeneratedBodyMaterial(world);
        const char* proxyName = _isLeft ? "ROCK_LeftGrabAuthorityProxy" : "ROCK_RightGrabAuthorityProxy";
        if (!_grabAuthorityProxy.create(
                world,
                bhkWorld,
                proxyShape,
                proxyFilterInfo,
                material,
                BethesdaMotionType::Keyframed,
                proxyName)) {
            havok_ref_count::release(proxyShape);
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: proxy body creation failed policy={} filter=0x{:08X} reason={}",
                handName(),
                grab_authority_proxy::filterPolicyName(),
                proxyFilterInfo,
                reason ? reason : "unknown");
            return false;
        }
        havok_ref_count::release(proxyShape);

        const RE::hkTransformf initialProxyHavok = grab_authority_proxy::makeHavokTransform(proxyWorldTransform);
        float zeroVelocity[4]{};
        const bool setTransformOk = _grabAuthorityProxy.setTransform(initialProxyHavok);
        const bool setVelocityOk = _grabAuthorityProxy.setVelocity(zeroVelocity, zeroVelocity);
        if (!setTransformOk || !setVelocityOk) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: initial proxy drive failed setTransform={} setVelocity={} proxyBody={} reason={}",
                handName(),
                setTransformOk ? "ok" : "fail",
                setVelocityOk ? "ok" : "fail",
                _grabAuthorityProxy.getBodyId().value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }

        std::uint32_t actualFilterInfo = 0;
        const bool filterReadOk = havok_runtime::tryReadFilterInfo(world, _grabAuthorityProxy.getBodyId(), actualFilterInfo);
        if (!filterReadOk || !grab_authority_proxy::hasNoContactFilterInfo(actualFilterInfo)) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: proxy no-contact filter invalid read={} filter=0x{:08X} expectedPolicy={} proxyBody={} reason={}",
                handName(),
                filterReadOk ? "ok" : "fail",
                actualFilterInfo,
                grab_authority_proxy::filterPolicyName(),
                _grabAuthorityProxy.getBodyId().value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }

        const RE::NiPoint3 pivotAProxyLocalGame = transform_math::worldPointToLocal(proxyWorldTransform, grabPivotAWorld);
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::desiredBodyInHandBodySpace(_grabFrame.rawHandSpace, _grabFrame.bodyLocal);
        const RE::NiPoint3 activePivotBBodyLocalGame =
            grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformProxySpace, pivotAProxyLocalGame);

        const float gameToHkScale = gameToHavokScale();
        float pivotBBodyLocalHk[4]{
            activePivotBBodyLocalGame.x * gameToHkScale,
            activePivotBBodyLocalGame.y * gameToHkScale,
            activePivotBBodyLocalGame.z * gameToHkScale,
            0.0f,
        };

        const GrabConstraintMotorTuning motorTuning =
            buildSharedConstraintMotorTuning(tau, damping, maxForce, proportionalRecovery, constantRecovery, looseWeaponGrab);

        _activeConstraint = createGrabConstraint(world,
            _grabAuthorityProxy.getBodyId(),
            objectBodyId,
            proxyWorldTransform,
            grabPivotAWorld,
            pivotBBodyLocalHk,
            desiredBodyTransformProxySpace,
            motorTuning);
        if (!_activeConstraint.isValid()) {
            ROCK_LOG_ERROR(Hand,
                "{} hand proxy constraint grab failed: constraint creation failed proxyBody={} objBody={} reason={}",
                handName(),
                _grabAuthorityProxy.getBodyId().value,
                objectBodyId.value,
                reason ? reason : "unknown");
            destroyGrabAuthorityProxy(bhkWorld);
            return false;
        }

        {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            _grabAuthorityProxyBhkWorld = bhkWorld;
            _grabAuthorityProxyHknpWorld = world;
            _grabAuthorityPivotAProxyLocalGame = pivotAProxyLocalGame;
            _grabAuthorityPivotBBodyLocalGame = activePivotBBodyLocalGame;
            _grabAuthorityProxyFrameValid = true;
            _grabAuthorityPendingTarget = GrabAuthorityProxyPendingTarget{
                .proxyWorld = proxyWorldTransform,
                .deltaTime = 0.0f,
                .forceFadeInTime = 0.0f,
                .tauMin = 0.0f,
                .grabPositionErrorGameUnits = 0.0f,
                .grabRotationErrorDegrees = 0.0f,
                .heldBodyColliding = false,
                .valid = true,
            };
            _lastAppliedGrabAuthorityProxyWorld = proxyWorldTransform;
            _hasLastAppliedGrabAuthorityProxyWorld = true;
            _grabAuthorityProxyFlushSequence = 0;
            _grabAuthorityProxyLogCounter = 0;
            _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);
        }

        _heldDriveMode = HeldObjectDriveMode::ProxyConstraint;
        ROCK_LOG_DEBUG(Hand,
            "{} hand proxy constraint grab drive: constraint={} looseWeapon={} proxyBody={} objBody={} filter=0x{:08X} pivotAProxy=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) linearTau={:.3f} angularTau={:.3f} linearForce={:.0f} angularForce={:.0f} reason={}",
            handName(),
            _activeConstraint.constraintId,
            looseWeaponGrab ? "yes" : "no",
            _grabAuthorityProxy.getBodyId().value,
            objectBodyId.value,
            actualFilterInfo,
            pivotAProxyLocalGame.x,
            pivotAProxyLocalGame.y,
            pivotAProxyLocalGame.z,
            activePivotBBodyLocalGame.x,
            activePivotBBodyLocalGame.y,
            activePivotBBodyLocalGame.z,
            motorTuning.linearTau,
            motorTuning.angularTau,
            motorTuning.linearMaxForce,
            motorTuning.angularMaxForce,
            reason ? reason : "unknown");
        return true;
    }

    bool Hand::updateConstraintGrabDriveTarget(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        RE::NiTransform& outDesiredObjectWorld,
        RE::NiTransform& outDesiredBodyWorld,
        RE::NiPoint3& outDesiredTargetPointWorld,
        RE::NiPoint3& outActivePivotBBodyLocalGame)
    {
        outDesiredObjectWorld = multiplyTransforms(handWorldTransform, _grabFrame.rawHandSpace);
        outDesiredBodyWorld = multiplyTransforms(handWorldTransform, multiplyTransforms(_grabFrame.rawHandSpace, _grabFrame.bodyLocal));
        outDesiredTargetPointWorld = transform_math::localPointToWorld(outDesiredBodyWorld, _grabFrame.pivotBBodyLocalGame);
        outActivePivotBBodyLocalGame = _grabFrame.pivotBBodyLocalGame;

        if (!_activeConstraint.isValid() || !_activeConstraint.constraintData || !_handBody.isValid() || _handBody.getBodyId().value == INVALID_BODY_ID) {
            return false;
        }

        RE::NiTransform liveHandBodyWorld{};
        if (!tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), liveHandBodyWorld)) {
            return false;
        }

        const RE::NiTransform desiredBodyTransformHandSpace =
            grab_frame_math::desiredBodyInHandBodySpace(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal);
        outDesiredObjectWorld = multiplyTransforms(liveHandBodyWorld, _grabFrame.constraintHandSpace);
        outDesiredBodyWorld = multiplyTransforms(liveHandBodyWorld, desiredBodyTransformHandSpace);

        auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
        auto* transformAPos = reinterpret_cast<float*>(constraintData + offsets::kTransformA_Pos);
        const float gameToHkScale = gameToHavokScale();
        transformAPos[0] = _grabFrame.pivotAHandBodyLocalGame.x * gameToHkScale;
        transformAPos[1] = _grabFrame.pivotAHandBodyLocalGame.y * gameToHkScale;
        transformAPos[2] = _grabFrame.pivotAHandBodyLocalGame.z * gameToHkScale;
        transformAPos[3] = 0.0f;

        auto* transformBRotation = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Col0);
        auto* transformBTranslation = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Pos);
        auto* targetBRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + 0x10);
        grab_constraint_math::writeInitialGrabAngularFrame(transformBRotation, targetBRca, desiredBodyTransformHandSpace);
        grab_constraint_math::writeDynamicTransformBTranslation(
            transformBTranslation,
            desiredBodyTransformHandSpace,
            _grabFrame.pivotAHandBodyLocalGame,
            gameToHkScale);

        outActivePivotBBodyLocalGame = constraintDrivePivotBBodyLocalGame(_grabFrame);
        outDesiredTargetPointWorld = transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);
        return true;
    }

    bool Hand::updateProxyConstraintGrabDriveTarget(RE::hknpWorld* world,
        const RE::NiTransform& proxyWorldTransform,
        RE::NiTransform& outDesiredObjectWorld,
        RE::NiTransform& outDesiredBodyWorld,
        RE::NiPoint3& outDesiredTargetPointWorld,
        RE::NiPoint3& outActivePivotBBodyLocalGame)
    {
        const RE::NiTransform desiredBodyTransformProxySpace =
            grab_frame_math::desiredBodyInHandBodySpace(_grabFrame.rawHandSpace, _grabFrame.bodyLocal);
        outDesiredObjectWorld = multiplyTransforms(proxyWorldTransform, _grabFrame.rawHandSpace);
        outDesiredBodyWorld = multiplyTransforms(proxyWorldTransform, desiredBodyTransformProxySpace);
        outActivePivotBBodyLocalGame = activeGrabDrivePivotBBodyLocalGame();
        outDesiredTargetPointWorld = transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);

        if (!world || !_activeConstraint.isValid() || !_activeConstraint.constraintData || !_grabAuthorityProxy.isValid() ||
            !_grabAuthorityProxyFrameValid || _grabAuthorityProxy.getBodyId().value == INVALID_BODY_ID) {
            return false;
        }

        auto* constraintData = static_cast<char*>(_activeConstraint.constraintData);
        auto* transformAPos = reinterpret_cast<float*>(constraintData + offsets::kTransformA_Pos);
        const float gameToHkScale = gameToHavokScale();
        transformAPos[0] = _grabAuthorityPivotAProxyLocalGame.x * gameToHkScale;
        transformAPos[1] = _grabAuthorityPivotAProxyLocalGame.y * gameToHkScale;
        transformAPos[2] = _grabAuthorityPivotAProxyLocalGame.z * gameToHkScale;
        transformAPos[3] = 0.0f;

        auto* transformBRotation = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Col0);
        auto* transformBTranslation = reinterpret_cast<float*>(constraintData + offsets::kTransformB_Pos);
        auto* targetBRca = reinterpret_cast<float*>(constraintData + ATOM_RAGDOLL_MOT + 0x10);
        grab_constraint_math::writeInitialGrabAngularFrame(transformBRotation, targetBRca, desiredBodyTransformProxySpace);
        grab_constraint_math::writeDynamicTransformBTranslation(
            transformBTranslation,
            desiredBodyTransformProxySpace,
            _grabAuthorityPivotAProxyLocalGame,
            gameToHkScale);

        outActivePivotBBodyLocalGame =
            grab_constraint_math::computeDynamicTransformBTranslationGame(desiredBodyTransformProxySpace, _grabAuthorityPivotAProxyLocalGame);
        _grabAuthorityPivotBBodyLocalGame = outActivePivotBBodyLocalGame;
        outDesiredTargetPointWorld = transform_math::localPointToWorld(outDesiredBodyWorld, outActivePivotBBodyLocalGame);
        return true;
    }

    void Hand::updateConstraintGrabDriveMotors(RE::hknpWorld* world,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        float grabPositionErrorGameUnits,
        float grabRotationErrorDegrees,
        bool heldBodyColliding)
    {
        if (!_activeConstraint.isValid() || !_activeConstraint.linearMotor || !_activeConstraint.angularMotor) {
            return;
        }

        const float looseLinearTauMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearTauMultiplier);
        const float looseAngularTauMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularTauMultiplier);
        const float looseCollisionTauMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier);
        const float looseLinearDampingMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier);
        const float looseAngularDampingMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier);
        const float looseMaxForceMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintMaxForceMultiplier);
        const float looseAngularForceMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularForceMultiplier);
        const float looseLinearRecoveryMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier);
        const float looseAngularRecoveryMultiplier =
            looseWeaponMultiplier(_heldObjectIsLooseWeapon, g_rockConfig.rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier);
        const float sharedBaseMaxForce = scaleDriveValue(g_rockConfig.rockGrabConstraintMaxForce, looseMaxForceMultiplier);

        const auto output = grab_motion_controller::solveMotorTargets(grab_motion_controller::MotorInput{
            .enabled = g_rockConfig.rockGrabAdaptiveMotorEnabled,
            .heldBodyColliding = heldBodyColliding,
            .positionErrorGameUnits = grabPositionErrorGameUnits,
            .rotationErrorDegrees = grabRotationErrorDegrees,
            .fullPositionErrorGameUnits = g_rockConfig.rockGrabAdaptivePositionFullError,
            .fullRotationErrorDegrees = g_rockConfig.rockGrabAdaptiveRotationFullError,
            .baseLinearTau = scaleDriveValue(g_rockConfig.rockGrabLinearTau, looseLinearTauMultiplier),
            .baseAngularTau = scaleDriveValue(g_rockConfig.rockGrabAngularTau, looseAngularTauMultiplier),
            .collisionTau = scaleDriveValue(tauMin, looseCollisionTauMultiplier),
            .maxTau = g_rockConfig.rockGrabTauMax,
            .currentLinearTau = _activeConstraint.linearMotor->tau,
            .currentAngularTau = _activeConstraint.angularMotor->tau,
            .tauLerpSpeed = g_rockConfig.rockGrabTauLerpSpeed,
            .deltaTime = deltaTime,
            .baseMaxForce = sharedBaseMaxForce,
            .maxForceMultiplier = g_rockConfig.rockGrabAdaptiveMaxForceMultiplier,
            .mass = readBodyMass(world, _savedObjectState.bodyId),
            .forceToMassRatio = g_rockConfig.rockGrabMaxForceToMassRatio,
            .angularToLinearForceRatio =
                angularForceRatioForMultiplier(g_rockConfig.rockGrabAngularToLinearForceRatio, looseAngularForceMultiplier),
            .fadeInEnabled = _grabFrame.fadeInGrabConstraint,
            .fadeElapsed = _grabStartTime,
            .fadeDuration = forceFadeInTime,
            .fadeStartAngularRatio =
                angularForceRatioForMultiplier(g_rockConfig.rockGrabFadeInStartAngularRatio, looseAngularForceMultiplier),
        });

        _activeConstraint.linearMotor->tau = output.linearTau;
        _activeConstraint.linearMotor->damping = scaleDriveValue(g_rockConfig.rockGrabLinearDamping, looseLinearDampingMultiplier);
        _activeConstraint.linearMotor->proportionalRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabLinearProportionalRecovery, looseLinearRecoveryMultiplier);
        _activeConstraint.linearMotor->constantRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabLinearConstantRecovery, looseLinearRecoveryMultiplier);
        _activeConstraint.linearMotor->minForce = -output.linearMaxForce;
        _activeConstraint.linearMotor->maxForce = output.linearMaxForce;

        _activeConstraint.angularMotor->tau = output.angularTau;
        _activeConstraint.angularMotor->damping = scaleDriveValue(g_rockConfig.rockGrabAngularDamping, looseAngularDampingMultiplier);
        _activeConstraint.angularMotor->proportionalRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabAngularProportionalRecovery, looseAngularRecoveryMultiplier);
        _activeConstraint.angularMotor->constantRecoveryVelocity =
            scaleDriveValue(g_rockConfig.rockGrabAngularConstantRecovery, looseAngularRecoveryMultiplier);
        _activeConstraint.angularMotor->minForce = -output.angularMaxForce;
        _activeConstraint.angularMotor->maxForce = output.angularMaxForce;

        _activeConstraint.currentTau = output.linearTau;
        _activeConstraint.currentMaxForce = output.linearMaxForce;
        _activeConstraint.targetMaxForce = sharedBaseMaxForce;
    }

    void Hand::queueProxyGrabAuthorityTarget(const RE::NiTransform& proxyWorldTransform,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        float grabPositionErrorGameUnits,
        float grabRotationErrorDegrees,
        bool heldBodyColliding)
    {
        std::scoped_lock lock(_grabAuthorityProxyMutex);
        if (_heldDriveMode != HeldObjectDriveMode::ProxyConstraint || !_grabAuthorityProxy.isValid()) {
            return;
        }

        _grabAuthorityPendingTarget.proxyWorld = proxyWorldTransform;
        _grabAuthorityPendingTarget.deltaTime = deltaTime;
        _grabAuthorityPendingTarget.forceFadeInTime = forceFadeInTime;
        _grabAuthorityPendingTarget.tauMin = tauMin;
        _grabAuthorityPendingTarget.grabPositionErrorGameUnits = grabPositionErrorGameUnits;
        _grabAuthorityPendingTarget.grabRotationErrorDegrees = grabRotationErrorDegrees;
        _grabAuthorityPendingTarget.heldBodyColliding = heldBodyColliding;
        _grabAuthorityPendingTarget.valid = true;
    }

    bool Hand::promoteHeldObjectToConstraintDrive(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        float tau,
        float damping,
        float maxForce,
        float proportionalRecovery,
        float constantRecovery,
        const char* reason)
    {
        if (!isHolding() || !world || !_savedObjectState.isValid()) {
            return false;
        }

        if (_heldDriveMode == HeldObjectDriveMode::SharedConstraint || _heldDriveMode == HeldObjectDriveMode::ProxyConstraint) {
            return _activeConstraint.isValid();
        }

        if (!_nativeGrab.isValid()) {
            return false;
        }

        if (!createConstraintGrabDrive(
                world,
                _savedObjectState.bodyId,
                handWorldTransform,
                tau,
                damping,
                maxForce,
                proportionalRecovery,
                constantRecovery,
                _heldObjectIsLooseWeapon,
                reason ? reason : "peer-joined-held-object")) {
            return false;
        }

        _nativeGrab.destroy(world, true);
        _nativeGrab.clear();
        _nativeGrabReleasePending.store(false, std::memory_order_release);
        ROCK_LOG_DEBUG(Hand,
            "{} hand promoted held object to shared constraint drive: formID={:08X} body={} reason={}",
            handName(),
            _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0,
            _savedObjectState.bodyId.value,
            reason ? reason : "peer-joined-held-object");
        return true;
    }

    bool Hand::startDynamicPull(RE::hknpWorld* world, const RE::NiTransform& handWorldTransform)
    {
        /*
         * Long-range pull remains a dynamic-object operation. ROCK promotes the
         * selected object tree through FO4VR's recursive wrappers, scans the
         * resulting dynamic body set, and then applies short-lived predicted
         * velocity to the accepted motions. It avoids a keyframed object path
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

        if (!grab_target::canUseRockDynamicPull(_currentSelection.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL blocked: targetKind={} formID={:08X}; actor targets require the loot/SCISSORS path",
                handName(),
                grab_target::name(_currentSelection.targetKind),
                selectedRef->GetFormID());
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

        auto* selectedBase = selectedRef->GetObjectReference();
        const char* selectedType = selectedBase ? selectedBase->GetFormTypeString() : "???";
        auto selectedNameView = selectedBase ? RE::TESFullName::GetFullName(*selectedBase, false) : std::string_view{};
        const std::string selectedName = selectedNameView.empty() ? std::string("(unnamed)") : std::string(selectedNameView);
        const bool selectedIsWeapon = selectedType && std::string_view(selectedType) == "WEAP";

        std::uint16_t selectedOriginalMotionPropsId = 1;
        auto selectedOriginalMotionType = physics_body_classifier::BodyMotionType::Unknown;
        if (_currentSelection.bodyId.value != INVALID_BODY_ID) {
            havok_runtime::tryReadBodyMotionPropertiesId(world, _currentSelection.bodyId, selectedOriginalMotionPropsId);
            if (auto* selectedBody = havok_runtime::getBody(world, _currentSelection.bodyId)) {
                selectedOriginalMotionType = physics_body_classifier::motionTypeFromBodyFlags(selectedBody->flags);
            }
        }

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.seedBodyId = _currentSelection.bodyId.value;
        scanOptions.targetKind = _currentSelection.targetKind;
        scanOptions.seedHitNode = _currentSelection.hitNode;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowWeaponRefExpansion = true;
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
        ROCK_LOG_DEBUG(Hand,
            "{} hand PULL scan: type={} weapon={} name='{}' formID={:08X} seedBody={} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} collisionObjects={} visitedNodes={}",
            handName(),
            selectedType ? selectedType : "???",
            selectedIsWeapon ? "yes" : "no",
            selectedName,
            selectedRef->GetFormID(),
            scanOptions.seedBodyId,
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            preparedBodySet.diagnostics.seedBodiesAdded,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips,
            preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes);
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);
        const RE::NiPoint3 primaryChoiceTarget = _currentSelection.hasHitPoint ? _currentSelection.hitPointWorld : grabPivotAForPrimaryChoice;
        const auto primaryChoice = preparedBodySet.choosePrimaryBody(_currentSelection.bodyId.value, object_physics_body_set::PurePoint3{ primaryChoiceTarget });

        if (primaryChoice.bodyId == INVALID_BODY_ID) {
            ROCK_LOG_WARN(Hand,
                "{} hand PULL failed: no accepted dynamic body after recursive prep formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
                "seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} setMotion={} enableCollision={}",
                handName(),
                selectedRef->GetFormID(),
                beforePrepBodySet.records.size(),
                preparedBodySet.records.size(),
                preparedBodySet.acceptedCount(),
                preparedBodySet.rejectedCount(),
                preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
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
        armPullCatchIntent(selectedRef, _pulledPrimaryBodyId, _currentSelection.targetKind);

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
            clearPullCatchIntent("beginPullRejected");
            restoreFailedPullPrep();
            return false;
        }
        stopSelectionHighlight();

        ROCK_LOG_INFO(Hand,
            "{} hand PULL start: type={} weapon={} formID={:08X} primaryBody={} bodyCount={} seeded={} scanFailures={} invalidSystems={} benignSkips={} unresolvedAccepted={} distanceHk={:.3f} duration={:.3f}s setMotion={} enableCollision={}",
            handName(),
            selectedType ? selectedType : "???",
            selectedIsWeapon ? "yes" : "no",
            selectedRef->GetFormID(),
            _pulledPrimaryBodyId,
            _pulledBodyIds.size(),
            preparedBodySet.diagnostics.seedBodiesAdded,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
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
            ROCK_LOG_DEBUG(Hand, "{} hand PULL invalidated before arrival: selected ref unavailable", handName());
            clearSelectionState(true);
            return false;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ _pulledPrimaryBodyId });
        if (!motion) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL invalidated before arrival: primary motion missing bodyId={}",
                handName(),
                _pulledPrimaryBodyId);
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
        const float configuredAutoGrabDistance = (std::max)(0.1f, g_rockConfig.rockPullAutoGrabDistanceGameUnits);
        const float nearConvergeDistance = (std::max)(configuredAutoGrabDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
        const float pocketBand = (std::max)(0.0f, g_rockConfig.rockGrabPocketRadiusGameUnits);
        const float arrivalDistance = (std::max)(configuredAutoGrabDistance, (std::min)(nearConvergeDistance, configuredAutoGrabDistance + pocketBand));
        if (distanceGameUnits <= arrivalDistance) {
            _currentSelection.isFarSelection = false;
            _currentSelection.hitPointWorld = RE::NiPoint3{
                objectPointHavok.x * havokToGameScale(),
                objectPointHavok.y * havokToGameScale(),
                objectPointHavok.z * havokToGameScale(),
            };
            _currentSelection.hasHitPoint = true;
            markPullCatchIntentArrived();
            clearPullRuntimeState();
            applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::PullArrivedClose });
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL arrived -> close grab window dist={:.1f} arrival={:.1f} configuredAuto={:.1f} near={:.1f} pocketBand={:.1f}",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                configuredAutoGrabDistance,
                nearConvergeDistance,
                pocketBand);
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
                .ownerGraceSeconds = g_rockConfig.rockPullOwnerGraceSeconds,
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

        if (motionResult.expired) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand PULL owner expired before arrival dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s ownerGrace={:.3f}s",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                _pullElapsedSeconds,
                _pullDurationSeconds,
                g_rockConfig.rockPullOwnerGraceSeconds);
            clearSelectionState(true);
            return false;
        }

        if (!motionResult.applyVelocity) {
            ROCK_LOG_SAMPLE_DEBUG(Hand,
                g_rockConfig.rockLogSampleMilliseconds,
                "{} hand PULL holding owner after velocity window dist={:.1f} arrival={:.1f} elapsed={:.3f}s duration={:.3f}s ownerGrace={:.3f}s",
                handName(),
                distanceGameUnits,
                arrivalDistance,
                _pullElapsedSeconds,
                _pullDurationSeconds,
                g_rockConfig.rockPullOwnerGraceSeconds);
            for (const auto bodyId : _pulledBodyIds) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
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
        float constantRecovery, const GrabSharedObjectContext& sharedContext)
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

        if (!grab_target::canUseRockActiveGrab(sel.targetKind)) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand GRAB blocked: targetKind={} formID={:08X}; actor targets are not normal ROCK physical grabs",
                handName(),
                grab_target::name(sel.targetKind),
                sel.refr ? sel.refr->GetFormID() : 0);
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }

        const bool joiningPeerHeldObject = sharedContextMatchesSelection(sharedContext, sel);
        const bool grabbedFromPullCatch = pullCatchIntentMatchesSelection();
        const bool looseWeaponGrab = isLooseWeaponGrabTarget(sel);

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
            if (objMotion) {
                std::uint32_t bodyFlags = body->flags;
                std::uint8_t bodyMotionPropsId = static_cast<std::uint8_t>(body->motionPropertiesId);
                std::uint16_t motionPropsId = selectedOriginalMotionPropsId;
                if (havok_runtime::tryReadMotionPropertiesId(objMotion, motionPropsId)) {
                    selectedOriginalMotionPropsId = motionPropsId;
                }

                havok_runtime::MotionVelocityCaps velocityCaps{};
                const bool hasVelocityCaps = havok_runtime::tryReadMotionVelocityCaps(objMotion, velocityCaps);
                const float maxLinVel = hasVelocityCaps ? velocityCaps.maxLinearVelocity : 0.0f;
                const float maxAngVel = hasVelocityCaps ? velocityCaps.maxAngularVelocity : 0.0f;

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
                        havok_runtime::MotionPropertiesLibrarySnapshot motionProperties{};
                        if (havok_runtime::snapshotMotionPropertiesLibrary(world, motionProperties)) {
                            ROCK_LOG_TRACE(Hand, "Motion properties library: {} entries, stride=0x40", motionProperties.count);
                            for (std::uint32_t i = 0; i < motionProperties.copiedCount; i++) {
                                const auto& record = motionProperties.records[i];
                                const auto* f = record.values;
                                const auto* u = record.words;
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

        object_physics_body_set::BodySetScanOptions scanOptions{};
        scanOptions.mode = physics_body_classifier::InteractionMode::ActiveGrab;
        scanOptions.rightHandBodyId = _isLeft ? INVALID_BODY_ID : _handBody.getBodyId().value;
        scanOptions.leftHandBodyId = _isLeft ? _handBody.getBodyId().value : INVALID_BODY_ID;
        scanOptions.sourceBodyId = _handBody.getBodyId().value;
        scanOptions.seedBodyId = sel.bodyId.value;
        scanOptions.targetKind = sel.targetKind;
        scanOptions.seedHitNode = sel.hitNode;
        scanOptions.requireSameResolvedRef = true;
        scanOptions.allowWeaponRefExpansion = true;
        scanOptions.heldBySameHand = &_heldBodyIds;
        scanOptions.maxDepth = g_rockConfig.rockObjectPhysicsTreeMaxDepth;

        const auto beforePrepBodySet = object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, sel.refr, scanOptions);
        active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle;
        activeLifecycle.captureBeforeActivePrep(beforePrepBodySet);

        const bool motionConverted = joiningPeerHeldObject ? true :
                                                               physics_recursive_wrappers::setMotionRecursive(
                                                                   rootNode,
                                                                   physics_recursive_wrappers::MotionPreset::Dynamic,
                                                                   true,
                                                                   true,
                                                                   true);
        const bool collisionEnabled = joiningPeerHeldObject ? true : physics_recursive_wrappers::enableCollisionRecursive(rootNode, true, true, true);

        auto preparedBodySet =
            joiningPeerHeldObject ? beforePrepBodySet : object_physics_body_set::scanObjectPhysicsBodySet(bhkWorld, world, sel.refr, scanOptions);
        if (!joiningPeerHeldObject) {
            activeLifecycle.markPreparedBodies(preparedBodySet);
        } else if (sharedContext.peerActiveGrabLifecycle) {
            activeLifecycle = *sharedContext.peerActiveGrabLifecycle;
        }
        const RE::NiPoint3 grabPivotAForPrimaryChoice = computeGrabPivotAWorld(world, handWorldTransform);

        auto restoreFailedGrabPrep = [&]() {
            if (!joiningPeerHeldObject) {
                restoreActiveGrabLifecycle(world,
                    activeLifecycle,
                    activeLifecycle.restorePlanForFailure(),
                    objectBodyId.value,
                    handName(),
                    "failed-setup");
                if (activeLifecycle.hasIncompleteNativeScan()) {
                    restoreIncompleteActivePrepRoot(rootNode, selectedOriginalMotionPropsId, handName(), "failed-setup-incomplete-scan");
                }
            }
        };

        ROCK_LOG_DEBUG(Hand,
            "{} hand object-tree prep: ref='{}' formID={:08X} beforeBodies={} afterBodies={} accepted={} rejected={} "
            "seedBody={} seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} "
            "latePrepared={} incompleteScan={} collisionObjects={} visitedNodes={} setMotion={} enableCollision={} sharedPeer={}",
            handName(),
            objName,
            sel.refr->GetFormID(),
            beforePrepBodySet.records.size(),
            preparedBodySet.records.size(),
            preparedBodySet.acceptedCount(),
            preparedBodySet.rejectedCount(),
            scanOptions.seedBodyId,
            preparedBodySet.diagnostics.seedBodiesAdded,
            preparedBodySet.diagnostics.scanFailures,
            preparedBodySet.diagnostics.invalidPhysicsSystems,
            preparedBodySet.diagnostics.benignScanSkips,
            preparedBodySet.diagnostics.foreignRefBodySkips,
            preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
            preparedBodySet.diagnostics.unresolvedRefBodySkips,
            activeLifecycle.latePreparedBodyCount(),
            activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no",
            preparedBodySet.diagnostics.collisionObjects,
            preparedBodySet.diagnostics.visitedNodes,
            motionConverted ? "ok" : "failed",
            collisionEnabled ? "ok" : "failed",
            joiningPeerHeldObject ? "yes" : "no");

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

        RE::NiPoint3 grabGripPoint = sel.hasHitPoint ? sel.hitPointWorld : objectWorldTransform.translate;
        float selectionToMeshDistanceGameUnits = 0.0f;
        bool meshGrabFound = false;
        MeshExtractionStats meshStats;
        std::vector<TriangleData> grabMeshTriangles;
        std::vector<GrabSurfaceTriangleData> grabSurfaceTriangles;
        GrabSurfaceHit grabSurfaceHit{};
        RuntimeGrabContactPatch contactPatchRuntime{};
        RuntimeMultiFingerGripContact multiFingerGripRuntime{};
        RE::NiPoint3 palmSeatPointWorld{};
        RE::NiPoint3 fingerEvidencePointWorld{};
        GrabSurfaceHit palmSeatSurfaceHit{};
        GrabSurfaceHit fingerEvidenceSurfaceHit{};
        bool contactPatchUsed = false;
        bool multiFingerGripUsed = false;
        bool palmSeatPointValid = false;
        bool fingerEvidencePointValid = false;
        RE::NiAVObject* surfaceOwnerNode = nullptr;
        RE::NiAVObject* authoredGrabNode = nullptr;
        const bool meshContactOnly = g_rockConfig.rockGrabMeshContactOnly;
        const char* grabPointMode = sel.hasHitPoint ? "selectionHitPointFallback" : "objectNodeOriginFallback";
        const char* grabFallbackReason = meshSourceNode ? "noTriangles" : "noMeshSourceNode";
        const char* palmSeatPointMode = grabPointMode;
        const char* palmSeatFallbackReason = grabFallbackReason;
        const char* fingerEvidencePointMode = "none";
        const char* fingerEvidenceFallbackReason = "none";

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
                    grabGripPoint = grabSurfaceHit.position;
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
            extractAllSurfaceTriangles(meshSourceNode, grabMeshTriangles, grabSurfaceTriangles, 10, &meshStats, g_rockConfig.rockGrabNodeNameBlacklist);

            ROCK_LOG_DEBUG(Hand,
                "{} hand mesh extraction: meshNode='{}' ownerNode='{}' rootNode='{}' shapes={} "
                "static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} blacklisted={} totalTris={}",
                handName(), nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes, meshStats.staticShapes,
                meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, meshStats.blacklistedShapes, meshStats.totalTriangles());

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
                RE::NiAVObject* grabNode = findAuthoredGrabNodeRecursive(
                    meshSourceNode, primaryNodeName, _isLeft, g_rockConfig.rockGrabNodeRejectOppositeHandAnchor, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                if (!grabNode && fallbackNodeName != primaryNodeName) {
                    grabNode = findAuthoredGrabNodeRecursive(
                        meshSourceNode, fallbackNodeName, _isLeft, g_rockConfig.rockGrabNodeRejectOppositeHandAnchor, g_rockConfig.rockObjectPhysicsTreeMaxDepth);
                }
                if (grabNode) {
                    authoredGrabNode = grabNode;
                    surfaceOwnerNode = grabNode;
                    grabSurfaceHit = {};
                    grabGripPoint = grabNode->world.translate;
                    meshGrabFound = true;
                    grabPointMode = "grabNodeAnchor";
                    grabFallbackReason = "none";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand GRAB NODE: node='{}' point=({:.1f},{:.1f},{:.1f})",
                        handName(),
                        nodeDebugName(grabNode),
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z);
                }
            }

            if (!meshGrabFound && !grabSurfaceTriangles.empty()) {
                RE::NiPoint3 grabPivotAWorld = computeGrabPivotAWorld(world, handWorldTransform);
                RE::NiPoint3 palmDir = computePalmNormalFromHandBasis(handWorldTransform, _isLeft);

                int rejectedBehindSurface = 0;
                if (findClosestGrabSurfaceHit(grabSurfaceTriangles,
                        grabPivotAWorld,
                        palmDir,
                        g_rockConfig.rockGrabLateralWeight,
                        g_rockConfig.rockGrabDirectionalWeight,
                        grabSurfaceHit,
                        g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                        &rejectedBehindSurface)) {
                    grabGripPoint = grabSurfaceHit.position;
                    selectionToMeshDistanceGameUnits =
                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, grabGripPoint) : std::numeric_limits<float>::max();
                    grabSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                    grabSurfaceHit.selectionToMeshDistanceGameUnits = selectionToMeshDistanceGameUnits;
                    surfaceOwnerNode = grabSurfaceHit.sourceNode;
                    meshGrabFound = true;
                    grabPointMode = "meshSurface";
                    grabFallbackReason = "none";
                    ROCK_LOG_DEBUG(Hand,
                        "{} hand MESH GRAB: mode={} tris={} staticTris={} dynamicTris={} skinnedTris={} "
                        "closest=({:.1f},{:.1f},{:.1f}) tri={} source={} owner='{}' shape='{}' selectionHit={} selectionDelta={:.1f}gu surfaceAlong={:.1f} surfaceLateral={:.1f} rejectBehind={}",
                        handName(), grabPointMode, grabMeshTriangles.size(), meshStats.staticTriangles, meshStats.dynamicTriangles, meshStats.skinnedTriangles, grabSurfaceHit.position.x,
                        grabSurfaceHit.position.y, grabSurfaceHit.position.z, grabSurfaceHit.triangleIndex, grabSurfaceSourceKindName(grabSurfaceHit.sourceKind),
                        nodeDebugName(grabSurfaceHit.sourceNode), nodeDebugName(grabSurfaceHit.sourceShape), sel.hasHitPoint ? "yes" : "no",
                        sel.hasHitPoint ? selectionToMeshDistanceGameUnits : -1.0f, grabSurfaceHit.signedAlongPalmDistanceGameUnits, grabSurfaceHit.lateralPalmDistanceGameUnits,
                        rejectedBehindSurface);
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
                "shapes={} static={}/{} dynamic={}/{} skinned={}/{} dynamicSkinnedSkipped={} emptyShapes={} blacklisted={} "
                "fallbackPoint=({:.1f},{:.1f},{:.1f})",
                handName(), grabPointMode, grabFallbackReason, nodeDebugName(meshSourceNode), nodeDebugName(collidableNode), nodeDebugName(rootNode), meshStats.visitedShapes,
                meshStats.staticShapes, meshStats.staticTriangles, meshStats.dynamicShapes, meshStats.dynamicTriangles, meshStats.skinnedShapes, meshStats.skinnedTriangles,
                meshStats.dynamicSkinnedSkipped, meshStats.emptyShapes, meshStats.blacklistedShapes, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
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

        const RE::NiPoint3 primaryChoiceTarget = meshGrabFound ? grabGripPoint : (sel.hasHitPoint ? sel.hitPointWorld : grabPivotAForPrimaryChoice);
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
                "{} hand GRAB failed: no accepted dynamic body after recursive prep for '{}' formID={:08X} visitedNodes={} collisionObjects={} seeded={} scanFailures={} invalidSystems={} benignSkips={} foreignSkips={} unresolvedAccepted={} unresolvedSkips={} latePrepared={} incompleteScan={}",
                handName(),
                objName,
                sel.refr->GetFormID(),
                preparedBodySet.diagnostics.visitedNodes,
                preparedBodySet.diagnostics.collisionObjects,
                preparedBodySet.diagnostics.seedBodiesAdded,
                preparedBodySet.diagnostics.scanFailures,
                preparedBodySet.diagnostics.invalidPhysicsSystems,
                preparedBodySet.diagnostics.benignScanSkips,
                preparedBodySet.diagnostics.foreignRefBodySkips,
                preparedBodySet.diagnostics.unresolvedRefBodiesAccepted,
                preparedBodySet.diagnostics.unresolvedRefBodySkips,
                activeLifecycle.latePreparedBodyCount(),
                activeLifecycle.hasIncompleteNativeScan() ? "yes" : "no");
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
        auto* preparedBody = havok_runtime::getBody(world, objectBodyId);
        if (!preparedBody) {
            ROCK_LOG_ERROR(Hand, "{} grabSelectedObject: prepared primary body {} is not readable after object prep", handName(), objectBodyId.value);
            restoreFailedGrabPrep();
            clearGrabExternalHandWorldTransform(_isLeft);
            return false;
        }
        _savedObjectState.bodyId = objectBodyId;
        _savedObjectState.refr = sel.refr;
        _savedObjectState.targetKind = sel.targetKind;
        _savedObjectState.originalFilterInfo = preparedBody->collisionFilterInfo;
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
                grabGripPoint = contactPatchRuntime.pivotDecision.point;
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
                grabSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
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
                    grabGripPoint.x,
                    grabGripPoint.y,
                    grabGripPoint.z,
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

        palmSeatPointWorld = grabGripPoint;
        palmSeatSurfaceHit = grabSurfaceHit;
        palmSeatPointMode = grabPointMode;
        palmSeatFallbackReason = grabFallbackReason;
        palmSeatPointValid = meshGrabFound || sel.hasHitPoint || authoredGrabNode != nullptr;

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

                fingerEvidencePointWorld = gripSet.contactCenterWorld;
                fingerEvidenceSurfaceHit = representativeHit;
                fingerEvidenceSurfaceHit.position = gripSet.contactCenterWorld;
                fingerEvidenceSurfaceHit.normal = gripSet.averageNormalWorld;
                fingerEvidenceSurfaceHit.distance = 0.0f;
                fingerEvidenceSurfaceHit.hasTriangle = representativeHit.valid && representativeHit.hasTriangle;
                fingerEvidenceSurfaceHit.hasSelectionHit = sel.hasHitPoint;
                fingerEvidenceSurfaceHit.selectionToMeshDistanceGameUnits =
                    sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, fingerEvidencePointWorld) : std::numeric_limits<float>::max();
                fingerEvidenceSurfaceHit.pivotToSurfaceDistanceGameUnits = pointDistanceGameUnits(computeGrabPivotAWorld(world, handWorldTransform), fingerEvidencePointWorld);
                fingerEvidenceSurfaceHit.resolvedOwnerMatchesBody = true;
                fingerEvidenceSurfaceHit.valid = true;
                fingerEvidencePointMode = "multiFingerContactPatch";
                fingerEvidenceFallbackReason = gripSet.reason;
                fingerEvidencePointValid = true;

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

        grabSurfaceHit = palmSeatSurfaceHit;
        grabGripPoint = palmSeatPointWorld;
        grabPointMode = palmSeatPointMode;
        grabFallbackReason = palmSeatFallbackReason;
        selectionToMeshDistanceGameUnits =
            palmSeatSurfaceHit.valid ? palmSeatSurfaceHit.selectionToMeshDistanceGameUnits : selectionToMeshDistanceGameUnits;

        ROCK_LOG_DEBUG(Hand,
            "{} hand GRAB POINT EVIDENCE: handPocket=palmSeat activeMode={} activeUsesFingerEvidence=no "
            "palmSeatValid={} palmSeatMode={} palmSeat=({:.1f},{:.1f},{:.1f}) "
            "fingerEvidenceValid={} fingerEvidenceMode={} fingerEvidence=({:.1f},{:.1f},{:.1f})",
            handName(),
            grabPointMode,
            palmSeatPointValid ? "yes" : "no",
            palmSeatPointMode,
            palmSeatPointWorld.x,
            palmSeatPointWorld.y,
            palmSeatPointWorld.z,
            fingerEvidencePointValid ? "yes" : "no",
            fingerEvidencePointMode,
            fingerEvidencePointWorld.x,
            fingerEvidencePointWorld.y,
            fingerEvidencePointWorld.z);

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
            if (player && !joiningPeerHeldObject) {
                nativeVRGrabDrop(player, 0);
                nativeVRGrabDrop(player, 1);
            }
        }

        _grabStartTime = 0.0f;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();

        {
            const RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
            const RE::NiPoint3 palmPocketPivotWorld = computeGrabPivotAWorld(world, handWorldTransform);
            RE::NiPoint3 grabPivotAWorld = palmPocketPivotWorld;
            RE::NiTransform grabBodyWorldAtGrab{};
            if (!tryGetGrabAuthorityBodyWorldTransform(world, objectBodyId, grabBodyWorldAtGrab)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: native grab BODY frame unreadable bodyId={} formID={:08X}",
                    handName(),
                    objectBodyId.value,
                    sel.refr ? sel.refr->GetFormID() : 0);
                _grabFrame.clear();
                _heldBodyIds.clear();
                _heldBodyIdsCount.store(0, std::memory_order_release);
                restoreFailedGrabPrep();
                return false;
            }
            RE::NiTransform motionBodyWorldAtGrab{};
            const bool hasMotionBodyWorldAtGrab =
                tryResolveLiveBodyWorldTransform(world, objectBodyId, motionBodyWorldAtGrab);
            const RE::NiTransform handBodyWorldAtGrab = getLiveBodyWorldTransform(world, _handBody.getBodyId());
            auto* ownerCellAtGrab = sel.refr ? sel.refr->GetParentCell() : nullptr;
            auto* bhkWorldAtGrab = ownerCellAtGrab ? ownerCellAtGrab->GetbhkWorld() : nullptr;
            auto* bodyCollisionObjectAtGrab = bhkWorldAtGrab ? RE::bhkNPCollisionObject::Getbhk(bhkWorldAtGrab, objectBodyId) : nullptr;
            auto* ownerNodeAtGrab = bodyCollisionObjectAtGrab ? bodyCollisionObjectAtGrab->sceneObject : nullptr;
            _grabFrame.heldNode = collidableNode;
            /*
             * Native dynamic grab is driven in FO4VR's hknp BODY frame. Runtime
             * logs from mixed bottle/book grabs show that this BODY frame is also
             * the frame followed by the held visual node. Applying an additional
             * node-to-body local offset here double-counts the body-local relation:
             * the object point can mathematically target the palm while the visible
             * object remains offset by the captured BODY/MOTION gap. Keep BODY as
             * the object frame for this native path; owner/root locals below remain
             * diagnostics for detecting unexpected scene-node divergence.
             */
            _grabFrame.bodyLocal = makeIdentityTransform();
            _grabFrame.ownerBodyLocal = ownerNodeAtGrab ? computeRuntimeBodyLocalTransform(ownerNodeAtGrab->world, grabBodyWorldAtGrab) : makeIdentityTransform();
            _grabFrame.rootBodyLocal = rootNode ? computeRuntimeBodyLocalTransform(rootNode->world, grabBodyWorldAtGrab) : makeIdentityTransform();
            _grabFrame.localMeshTriangles.clear();
            _grabFrame.gripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
            _grabFrame.gripEvidenceLocal = _grabFrame.gripPointLocal;
            _grabFrame.gripNormalLocal = grabSurfaceHit.valid ? transform_math::worldVectorToLocal(objectWorldTransform, grabSurfaceHit.normal) : RE::NiPoint3{};
            _grabFrame.gripPointBodyLocalGame = grab_contact_patch_math::freezePivotBBodyLocal(grabBodyWorldAtGrab, grabGripPoint);
            _grabFrame.pivotBBodyLocalGame = _grabFrame.gripPointBodyLocalGame;
            _grabFrame.hasFrozenPivotB = true;
            _grabFrame.hasGripPoint = grabSurfaceHit.valid || authoredGrabNode != nullptr;
            _grabFrame.gripEvidenceTriangleIndex = grabSurfaceHit.valid && grabSurfaceHit.hasTriangle ? static_cast<std::uint32_t>(grabSurfaceHit.triangleIndex) : 0xFFFF'FFFF;
            _grabFrame.gripEvidenceShapeKey = grabSurfaceHit.valid ? grabSurfaceHit.shapeKey : 0xFFFF'FFFF;
            _grabFrame.gripEvidenceShapeCollisionFilterInfo = grabSurfaceHit.valid ? grabSurfaceHit.shapeCollisionFilterInfo : 0;
            _grabFrame.gripEvidenceHitFraction = grabSurfaceHit.valid ? grabSurfaceHit.hitFraction : 1.0f;
            _grabFrame.hasGripEvidenceShapeKey = grabSurfaceHit.valid && grabSurfaceHit.hasShapeKey;
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
            _grabFrame.palmSeatPointWorldAtGrab = palmSeatPointWorld;
            _grabFrame.fingerEvidencePointWorldAtGrab = fingerEvidencePointWorld;
            _grabFrame.hasPalmSeatPoint = palmSeatPointValid;
            _grabFrame.hasFingerEvidencePoint = fingerEvidencePointValid;
            _grabFrame.activeGrabPointUsesMultiFingerEvidence = false;
            _grabFrame.activeGrabPointMode = grabPointMode;
            _grabFrame.palmSeatPointMode = palmSeatPointMode;
            _grabFrame.fingerEvidencePointMode = fingerEvidencePointMode;
            if (contactPatchUsed) {
                const std::uint32_t copyCount = (std::min)(contactPatchRuntime.sampleCount, static_cast<std::uint32_t>(_grabFrame.contactPatchSamples.size()));
                for (std::uint32_t i = 0; i < copyCount; ++i) {
                    auto sample = contactPatchRuntime.samples[i];
                    sample.point = transform_math::worldPointToLocal(grabBodyWorldAtGrab, sample.point);
                    sample.normal = transform_math::worldVectorToLocal(grabBodyWorldAtGrab, sample.normal);
                    _grabFrame.contactPatchSamples[i] = sample;
                }
                _grabFrame.contactPatchSampleCount = copyCount;
            }
            _grabFrame.bodyResolutionReason = primaryBodyChoiceReasonName(primaryChoice.reason);
            _grabFrame.pocketToGripDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
            _grabFrame.selectionToGripEvidenceDistanceGameUnits =
                grabSurfaceHit.valid && grabSurfaceHit.hasSelectionHit ? grabSurfaceHit.selectionToMeshDistanceGameUnits : (sel.hasHitPoint ? 0.0f : std::numeric_limits<float>::max());
            if (grabSurfaceHit.valid) {
                grabSurfaceHit.pivotToSurfaceDistanceGameUnits = _grabFrame.pocketToGripDistanceGameUnits;
            }
            _grabFrame.hasMeshPoseData = false;
            if (!grabMeshTriangles.empty()) {
                _grabFrame.localMeshTriangles = cacheTrianglesInLocalSpace(grabMeshTriangles, objectWorldTransform);
                _grabFrame.hasMeshPoseData = !_grabFrame.localMeshTriangles.empty();
            }

            RE::NiTransform desiredObjectWorld = objectWorldTransform;
            const auto oppositionContacts = hand_semantic_contact_state::selectThumbOppositionContacts(semanticContacts);
            /*
             * Runtime grab authority is intentionally single-source. Mesh hits,
             * authored nodes, semantic contacts, contact patches, and multi-finger
             * contacts are evidence; they no longer rotate the object, move the
             * visible hand, select a second grip owner, or replace the hand pocket.
             * This keeps one dynamic-grab authority and avoids the old ROCK fallback
             * where surface frames and object-driven hand solving competed with the
             * root-flattened hand convention.
             */
            {
                auto pocket = grab_three_phase::buildGrabPocketFrame(
                    handWorldTransform,
                    _isLeft,
                    g_rockConfig.rockGrabPocketDepthGameUnits,
                    g_rockConfig.rockGrabPocketRadiusGameUnits);
                auto gripArea = grab_three_phase::buildObjectGripArea(grab_three_phase::GripAreaInput{
                    .objectBodyWorld = grabBodyWorldAtGrab,
                    .contactSeedWorld = grabGripPoint,
                    .centerOfMassWorld = grabBodyWorldAtGrab.translate,
                    .interiorDirectionWorld = RE::NiPoint3{},
                    .preferredInsetGameUnits = 0.0f,
                    .maxInsetGameUnits = 0.0f,
                    .centerOfMassValid = false,
                    .interiorDirectionValid = false,
                    .source = grabPointMode,
                });
                const RE::NiPoint3 phaseGripSeed = gripArea.valid ? gripArea.contactSeedWorld : grabGripPoint;
                const RE::NiPoint3 palmSeatAnchorWorld = pocket.valid ? pocket.palmCenterWorld : grabPivotAWorld;
                const RE::NiPoint3 gripToPocket = pocket.valid ? (phaseGripSeed - palmSeatAnchorWorld) : RE::NiPoint3{};
                const float gripToPocketDistance =
                    pocket.valid ? std::sqrt(gripToPocket.x * gripToPocket.x + gripToPocket.y * gripToPocket.y + gripToPocket.z * gripToPocket.z) :
                                   std::numeric_limits<float>::max();
                const float stableTouchEnvelope = (std::max)(g_rockConfig.rockGrabTouchAcquireDistanceGameUnits, pocket.pocketRadiusGameUnits);
                const bool hasStablePocketTouchContact =
                    oppositionContacts.valid && gripToPocketDistance <= stableTouchEnvelope;
                const auto phaseDecision = grab_three_phase::classifyAcquisitionPhase(grab_three_phase::PhaseClassificationInput{
                    .pocket = pocket,
                    .gripSeedWorld = phaseGripSeed,
                    .hasFreshTouchContact = hasStablePocketTouchContact,
                    .isFarSelection = sel.isFarSelection,
                    .touchAcquireDistanceGameUnits = g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                    .touchContactMaxDistanceGameUnits = stableTouchEnvelope,
                    .nearConvergeDistanceGameUnits = g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                    .behindPalmToleranceGameUnits = g_rockConfig.rockGrabSurfaceBehindPalmToleranceGameUnits,
                });

                if (pocket.valid && gripArea.valid && phaseDecision.accepted) {
                    _grabObjectGripAtGrab = gripArea;
                    _grabAcquisitionPhase = phaseDecision.phase;
                    const bool useAuthoredGrabFrame = authoredGrabNode != nullptr;
                    const char* relationMode = useAuthoredGrabFrame ? "authoredGrabNodeFrame" : "rockPointToPalm";

                    grabPivotAWorld = pocket.palmCenterWorld;
                    grabGripPoint = gripArea.contactSeedWorld;
                    grabPointMode = relationMode;
                    grabFallbackReason = phaseDecision.reason;

                    _grabFrame.gripPointLocal = transform_math::worldPointToLocal(objectWorldTransform, grabGripPoint);
                    _grabFrame.gripEvidenceLocal = _grabFrame.gripPointLocal;
                    _grabFrame.gripNormalLocal = transform_math::worldVectorToLocal(objectWorldTransform, pocket.palmNormalWorld);
                    _grabFrame.gripPointBodyLocalGame = grab_contact_patch_math::freezePivotBBodyLocal(grabBodyWorldAtGrab, grabGripPoint);
                    _grabFrame.pivotBBodyLocalGame = _grabFrame.gripPointBodyLocalGame;
                    _grabFrame.hasFrozenPivotB = true;
                    _grabFrame.hasGripPoint = true;
                    _grabFrame.pocketToGripDistanceGameUnits = pointDistanceGameUnits(grabPivotAWorld, grabGripPoint);
                    _grabFrame.selectionToGripEvidenceDistanceGameUnits =
                        sel.hasHitPoint ? pointDistanceGameUnits(sel.hitPointWorld, gripArea.contactSeedWorld) : std::numeric_limits<float>::max();
                    _grabFrame.grabPivotWorldAtGrab = grabPivotAWorld;
                    _grabFrame.gripPointWorldAtGrab = grabGripPoint;
                    _grabFrame.activeGrabPointMode = grabPointMode;
                    _grabFrame.palmSeatPointMode = "threePhasePocket";
                    _grabFrame.fingerEvidencePointMode = "evidenceOnly";
                    _grabFrame.activeGrabPointUsesMultiFingerEvidence = false;
                    _grabFrame.fingerPoseAimValid = true;
                    _grabFrame.fingerPoseAimReason = "rockPointToPalmEvidence";
                    if (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::NearConverging ||
                        _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::GravityPulling) {
                        _grabFrame.fingerPoseAimValid = false;
                        _grabFrame.fingerPoseAimReason = "nearConvergePendingTouch";
                    }

                    desiredObjectWorld = grab_frame_math::shiftObjectToAlignGripWithPocket(
                        objectWorldTransform,
                        grabPivotAWorld,
                        grabGripPoint);
                    if (useAuthoredGrabFrame) {
                        desiredObjectWorld = grab_node_info_math::buildDesiredObjectWorldFromAuthoredGrabNode(
                            objectWorldTransform,
                            authoredGrabNode->world,
                            handWorldTransform,
                            grabPivotAWorld);
                    }
                    const float pulledGrabAdjust =
                        (grabbedFromPullCatch && !useAuthoredGrabFrame) ?
                            (std::max)(0.0f, g_rockConfig.rockPulledGrabHandAdjustDistanceGameUnits) :
                            0.0f;
                    if (pulledGrabAdjust > 0.0f) {
                        desiredObjectWorld.translate = desiredObjectWorld.translate + pocket.palmNormalWorld * pulledGrabAdjust;
                    }

                    const auto threePhaseFingerTargets =
                        buildRuntimeFingerPoseTargets(grabGripPoint,
                            pocket.palmNormalWorld);
                    storeFingerPoseTargetsInGrabFrame(_grabFrame, threePhaseFingerTargets, objectWorldTransform);

                    ROCK_LOG_DEBUG(Hand,
                        "{} THREE-PHASE GRAB CAPTURE: relation={} rotation={} phase={} reason={} touchContact={} stableTouch={} pocket=({:.1f},{:.1f},{:.1f}) "
                        "palm=({:.1f},{:.1f},{:.1f}) normal=({:.3f},{:.3f},{:.3f}) seed=({:.1f},{:.1f},{:.1f}) "
                        "grip=({:.1f},{:.1f},{:.1f}) gripLocal=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) dist={:.1f} signedPalm={:.1f} pulledAdjust={:.1f} inset={:.2f} insetSource={}",
                        handName(),
                        relationMode,
                        useAuthoredGrabFrame ? "authoredNode" : "preserve",
                        grab_three_phase::phaseName(_grabAcquisitionPhase),
                        phaseDecision.reason,
                        semanticContacts.count > 0 ? "yes" : "no",
                        hasStablePocketTouchContact ? "yes" : "no",
                        pocket.pocketCenterWorld.x,
                        pocket.pocketCenterWorld.y,
                        pocket.pocketCenterWorld.z,
                        pocket.palmCenterWorld.x,
                        pocket.palmCenterWorld.y,
                        pocket.palmCenterWorld.z,
                        pocket.palmNormalWorld.x,
                        pocket.palmNormalWorld.y,
                        pocket.palmNormalWorld.z,
                        gripArea.contactSeedWorld.x,
                        gripArea.contactSeedWorld.y,
                        gripArea.contactSeedWorld.z,
                        grabGripPoint.x,
                        grabGripPoint.y,
                        grabGripPoint.z,
                        _grabFrame.gripPointLocal.x,
                        _grabFrame.gripPointLocal.y,
                        _grabFrame.gripPointLocal.z,
                        _grabFrame.pivotBBodyLocalGame.x,
                        _grabFrame.pivotBBodyLocalGame.y,
                        _grabFrame.pivotBBodyLocalGame.z,
                        phaseDecision.gripToPocketDistanceGameUnits,
                        phaseDecision.signedPalmDistanceGameUnits,
                        pulledGrabAdjust,
                        gripArea.seedInsetGameUnits,
                        gripArea.fallbackReason);
                } else {
                    _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
                    _grabObjectGripAtGrab = {};
                    _heldObjectIsLooseWeapon = false;
                    _grabFrame.clear();
                    _heldBodyIds.clear();
                    _heldBodyIdsCount.store(0, std::memory_order_release);
                    _grabFingerPosePublished = false;
                    if (frik::api::FRIKApi::inst && frik::api::FRIKApi::inst->clearHandPose) {
                        frik::api::FRIKApi::inst->clearHandPose("ROCK_Grab", handFromBool(_isLeft));
                    }
                    clearGrabExternalHandWorldTransform(_isLeft);
                    restoreFailedGrabPrep();
                    ROCK_LOG_WARN(Hand,
                        "{} THREE-PHASE GRAB ABORT: pocketValid={} gripValid={} accepted={} reason={} dist={:.2f}gu stableTouch={}",
                        handName(),
                        pocket.valid ? "yes" : "no",
                        gripArea.valid ? "yes" : "no",
                        phaseDecision.accepted ? "yes" : "no",
                        phaseDecision.reason,
                        gripToPocketDistance,
                        hasStablePocketTouchContact ? "yes" : "no");
                    return false;
                }
            }

            logGrabNodeInfo(handName(),
                _isLeft,
                collidableNode,
                authoredGrabNode,
                desiredObjectWorld,
                handWorldTransform,
                grabPivotAWorld,
                grabPointMode);

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
            _grabFrame.gripPointWorldAtGrab = grabGripPoint;
            _grabFrame.handScaleAtGrab = handWorldTransform.scale;
            clearGrabExternalHandWorldTransform(_isLeft);
            _grabVisualHandTransform = handWorldTransform;
            _hasGrabVisualHandTransform = false;
            _grabVisualDeviationExceededSeconds = 0.0f;
            _grabDeviationExceededSeconds = 0.0f;
            const RE::NiPoint3 initialGrabDelta = grabPivotAWorld - grabGripPoint;
            const float initialGrabDistance =
                std::sqrt(initialGrabDelta.x * initialGrabDelta.x + initialGrabDelta.y * initialGrabDelta.y + initialGrabDelta.z * initialGrabDelta.z);
            _nativeGrabReleasePending.store(false, std::memory_order_release);
            const bool needsLargeInitialSync = initialGrabDistance >= g_rockConfig.rockGrabHandLerpMinDistance;
            _grabFrame.fadeInGrabConstraint = needsLargeInitialSync;
            if (needsLargeInitialSync) {
                _grabFrame.motorFadeReason = "largeInitialSync";
            } else {
                _grabFrame.motorFadeReason = "none";
            }
            _grabFingerPoseFrameCounter = 0;
            _grabFingerPoseAccumulatedDeltaTime = 0.0f;
            _heldLocalLinearVelocityHistory = {};
            _heldLocalLinearVelocityHistoryCount = 0;
            _heldLocalLinearVelocityHistoryNext = 0;
            _heldLocalHandVelocityHistory = {};
            _heldHandAngularVelocityHistory = {};
            _heldHandVelocityHistoryCount = 0;
            _heldHandVelocityHistoryNext = 0;
            _lastHeldObjectLocalLinearVelocityHavok = {};
            _hasLastHeldObjectLocalLinearVelocityHavok = false;
            _previousHeldRawHandWorld = {};
            _previousHeldHandPositionHavok = {};
            _lastHeldHandPositionHavok = {};
            _hasPreviousHeldRawHandWorld = false;
            _hasLastHeldHandPositionHavok = false;
            _lastPlayerSpaceVelocityHavok = {};
            _grabAuthorityProxyReleasePending.store(false, std::memory_order_release);

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
                const float motionVsGrabRot = hasMotionBodyWorldAtGrab ? rotationDeltaDegrees(motionBodyWorldAtGrab.rotate, grabBodyWorldAtGrab.rotate) : -1.0f;
                const float motionVsGrabPos = hasMotionBodyWorldAtGrab ? translationDeltaGameUnits(motionBodyWorldAtGrab, grabBodyWorldAtGrab) : -1.0f;

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME SUMMARY: vrScale={:.3f} handScale={:.3f} bodyScale={:.3f} objectScale={:.3f} "
                    "pivotHS=({:.2f},{:.2f},{:.2f}) pocketW=({:.1f},{:.1f},{:.1f}) gripW=({:.1f},{:.1f},{:.1f}) "
                    "pivotBLocal=({:.2f},{:.2f},{:.2f}) "
                    "rawConstraintDelta={:.2f}deg/{:.2f}gu motionDiagVsGrab={:.2f}deg/{:.2f}gu pivotALocal=({:.2f},{:.2f},{:.2f}) meshMode={} meshTris={} "
                    "pocketGrip={:.1f}gu selectionGripEvidence={:.1f}gu "
                    "shapeKey=0x{:08X} shapeFilter=0x{:08X} hitFraction={:.4f} contactPatch={} patchHits={} snapDelta={:.1f}gu "
                    "multiFinger={} mfGroups={} mfSpread={:.2f}gu mfReason={} "
                    "activePoint={} activeUsesFingerEvidence={} palmSeatPoint={} fingerEvidencePoint={} "
                    "poseTargets={} motorFade={} motorFadeReason={} bodyReason={} "
                    "fingerPoseAim={} fingerPoseAimReason={}",
                    handName(), vrScale, handWorldTransform.scale, handBodyWorldAtGrab.scale, collidableNode ? collidableNode->world.scale : -1.0f,
                    grabPivotAHandspace.x, grabPivotAHandspace.y, grabPivotAHandspace.z, grabPivotAWorld.x, grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x,
                    grabGripPoint.y, grabGripPoint.z, _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z,
                    rawVsConstraintRot, rawVsConstraintPos, motionVsGrabRot, motionVsGrabPos, _grabFrame.pivotAHandBodyLocalGame.x,
                    _grabFrame.pivotAHandBodyLocalGame.y, _grabFrame.pivotAHandBodyLocalGame.z, grabPointMode, grabMeshTriangles.size(),
                    _grabFrame.pocketToGripDistanceGameUnits, _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.gripEvidenceShapeKey, _grabFrame.gripEvidenceShapeCollisionFilterInfo, _grabFrame.gripEvidenceHitFraction,
                    _grabFrame.hasContactPatch ? "yes" : "no", _grabFrame.contactPatchSampleCount, _grabFrame.contactPatchMeshSnapDeltaGameUnits,
                    _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.multiFingerContactReason,
                    _grabFrame.activeGrabPointMode, _grabFrame.activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
                    _grabFrame.palmSeatPointMode, _grabFrame.fingerEvidencePointMode,
                    _grabFrame.fingerPoseTargetCount, _grabFrame.fadeInGrabConstraint ? "yes" : "no",
                    _grabFrame.motorFadeReason, _grabFrame.bodyResolutionReason,
                    _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);

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
                grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z);
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

        if (joiningPeerHeldObject && sharedContext.peerSavedObjectState) {
            copyPeerInertiaSnapshot(_savedObjectState, *sharedContext.peerSavedObjectState);
            ROCK_LOG_DEBUG(Hand,
                "{} hand joined peer-held object inertia snapshot: formID={:08X} peerMotions={} inertiaModified={}",
                handName(),
                sel.refr ? sel.refr->GetFormID() : 0,
                sharedContext.peerSavedObjectState->motionInertiaStates.size(),
                sharedContext.peerSavedObjectState->inertiaModified ? "yes" : "no");
        } else {
            normalizeGrabbedInertiaForBodies(world, objectBodyId, _heldBodyIds, _savedObjectState);
        }

        {
            RE::NiPoint3 palmPos = computePalmPositionFromHandBasis(handWorldTransform, _isLeft);
            RE::NiPoint3 grabPivotAWorld =
                _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, handWorldTransform);
            const float gameToHkScale = gameToHavokScale();
            const RE::NiTransform desiredBodyTransformRawHandSpace = multiplyTransforms(_grabFrame.rawHandSpace, _grabFrame.bodyLocal);
            const RE::NiTransform initialDesiredBodyWorld = multiplyTransforms(handWorldTransform, desiredBodyTransformRawHandSpace);

            float pivotAHk[4];
            pivotAHk[0] = grabPivotAWorld.x * gameToHkScale;
            pivotAHk[1] = grabPivotAWorld.y * gameToHkScale;
            pivotAHk[2] = grabPivotAWorld.z * gameToHkScale;
            pivotAHk[3] = 0.0f;

            float gripWorldHk[4];
            gripWorldHk[0] = grabGripPoint.x * gameToHkScale;
            gripWorldHk[1] = grabGripPoint.y * gameToHkScale;
            gripWorldHk[2] = grabGripPoint.z * gameToHkScale;
            gripWorldHk[3] = 0.0f;

            {
                float pivotAToGrab = std::sqrt(
                    (pivotAHk[0] - gripWorldHk[0]) * (pivotAHk[0] - gripWorldHk[0]) + (pivotAHk[1] - gripWorldHk[1]) * (pivotAHk[1] - gripWorldHk[1]) +
                    (pivotAHk[2] - gripWorldHk[2]) * (pivotAHk[2] - gripWorldHk[2]));
                float palmToHandOrigin = std::sqrt((palmPos.x - handWorldTransform.translate.x) * (palmPos.x - handWorldTransform.translate.x) +
                    (palmPos.y - handWorldTransform.translate.y) * (palmPos.y - handWorldTransform.translate.y) +
                    (palmPos.z - handWorldTransform.translate.z) * (palmPos.z - handWorldTransform.translate.z));
                float pivotAToHandOrigin = std::sqrt((grabPivotAWorld.x - handWorldTransform.translate.x) * (grabPivotAWorld.x - handWorldTransform.translate.x) +
                    (grabPivotAWorld.y - handWorldTransform.translate.y) * (grabPivotAWorld.y - handWorldTransform.translate.y) +
                    (grabPivotAWorld.z - handWorldTransform.translate.z) * (grabPivotAWorld.z - handWorldTransform.translate.z));
                ROCK_LOG_DEBUG(Hand,
                    "GRAB DIAG {}: palmPos=({:.1f},{:.1f},{:.1f}) handPos=({:.1f},{:.1f},{:.1f}) "
                    "pocket=({:.1f},{:.1f},{:.1f}) grip=({:.1f},{:.1f},{:.1f}) meshGrab={} grabPointMode={} fallbackReason={} "
                    "frozenPivotB=({:.2f},{:.2f},{:.2f}) contactPatch={} patchHits={} multiFinger={} mfGroups={} mfSpread={:.2f} "
                    "activePoint={} activeUsesFingerEvidence={} palmSeatPoint={} fingerEvidencePoint={} "
                    "pivotAToGrab_hk={:.4f} ({:.1f} game units) palmToHandOrigin={:.1f} pivotAToHandOrigin={:.1f} game units "
                    "selectionToGripEvidence={:.1f} fingerPoseAim={} fingerPoseAimReason={}",
                    handName(), palmPos.x, palmPos.y, palmPos.z, handWorldTransform.translate.x, handWorldTransform.translate.y, handWorldTransform.translate.z, grabPivotAWorld.x,
                    grabPivotAWorld.y, grabPivotAWorld.z, grabGripPoint.x, grabGripPoint.y, grabGripPoint.z, meshGrabFound, grabPointMode, grabFallbackReason,
                    _grabFrame.pivotBBodyLocalGame.x, _grabFrame.pivotBBodyLocalGame.y, _grabFrame.pivotBBodyLocalGame.z, _grabFrame.hasContactPatch ? "yes" : "no",
                    _grabFrame.contactPatchSampleCount, _grabFrame.hasMultiFingerContactPatch ? "yes" : "no", _grabFrame.multiFingerContactGroupCount,
                    _grabFrame.multiFingerContactSpreadGameUnits, _grabFrame.activeGrabPointMode, _grabFrame.activeGrabPointUsesMultiFingerEvidence ? "yes" : "no",
                    _grabFrame.palmSeatPointMode, _grabFrame.fingerEvidencePointMode, pivotAToGrab,
                    pivotAToGrab * havokToGameScale(), palmToHandOrigin, pivotAToHandOrigin, _grabFrame.selectionToGripEvidenceDistanceGameUnits,
                    _grabFrame.fingerPoseAimValid ? "yes" : "no", _grabFrame.fingerPoseAimReason);
            }

            const char* driveReason = joiningPeerHeldObject ? "joining-peer-held-loose-object" : "ordinary-dynamic-loose-object";
            if (!createProxyConstraintGrabDrive(
                    bhkWorld,
                    world,
                    objectBodyId,
                    handWorldTransform,
                    grabPivotAWorld,
                    tau,
                    damping,
                    maxForce,
                    proportionalRecovery,
                    constantRecovery,
                    looseWeaponGrab,
                    driveReason)) {
                ROCK_LOG_ERROR(Hand,
                    "{} hand GRAB FAILED: proxy constraint creation failed bodyId={} targetBody=({:.2f},{:.2f},{:.2f}) pivotB=({:.2f},{:.2f},{:.2f}) joiningPeer={}",
                    handName(),
                    objectBodyId.value,
                    initialDesiredBodyWorld.translate.x,
                    initialDesiredBodyWorld.translate.y,
                    initialDesiredBodyWorld.translate.z,
                    activeGrabDrivePivotBBodyLocalGame().x,
                    activeGrabDrivePivotBBodyLocalGame().y,
                    activeGrabDrivePivotBBodyLocalGame().z,
                    joiningPeerHeldObject ? "yes" : "no");
            }
        }

        const bool driveCreated = _heldDriveMode == HeldObjectDriveMode::ProxyConstraint && _activeConstraint.isValid() && _grabAuthorityProxy.isValid();
        if (!driveCreated) {
            ROCK_LOG_ERROR(Hand, "{} hand GRAB FAILED: proxy-constraint dynamic grab creation failed", handName());
            destroyGrabAuthorityProxy(bhkWorld);
            clearGrabExternalHandWorldTransform(_isLeft);
            if (!joiningPeerHeldObject) {
                restoreGrabbedInertia(world, _savedObjectState);
            }
            restoreFailedGrabPrep();
            restoreHandCollisionAfterGrab(world);
            _savedObjectState.clear();
            _heldBodyIds.clear();
            _heldObjectIsLooseWeapon = false;
            return false;
        }

        _heldObjectIsLooseWeapon = looseWeaponGrab;
        if (_heldDriveMode == HeldObjectDriveMode::NativeMouseSpring) {
            ROCK_LOG_DEBUG(Hand,
                "{} hand dynamic grab created: driveMode=nativeMouseSpring looseWeapon={} action={:p} hand body={} obj body={} heldBodies={} nativeLinearScale={:.2f} nativeAngularScale={:.2f} nativeClampScale={:.2f}",
                handName(),
                _heldObjectIsLooseWeapon ? "yes" : "no",
                _nativeGrab.action(),
                _handBody.getBodyId().value,
                objectBodyId.value,
                _heldBodyIds.size(),
                native_mouse_spring_grab::composeTuningScale(
                    g_rockConfig.rockGrabNativeMouseSpringLinearResponseScale,
                    _heldObjectIsLooseWeapon ? g_rockConfig.rockGrabLooseWeaponNativeLinearResponseMultiplier : 1.0f),
                native_mouse_spring_grab::composeTuningScale(
                    g_rockConfig.rockGrabNativeMouseSpringAngularResponseScale,
                    _heldObjectIsLooseWeapon ? g_rockConfig.rockGrabLooseWeaponNativeAngularResponseMultiplier : 1.0f),
                native_mouse_spring_grab::composeTuningScale(
                    g_rockConfig.rockGrabNativeMouseSpringAngularClampScale,
                    _heldObjectIsLooseWeapon ? g_rockConfig.rockGrabLooseWeaponNativeAngularClampMultiplier : 1.0f));
        } else {
            const float sharedLinearForce = _activeConstraint.linearMotor ?
                (std::max)(std::fabs(_activeConstraint.linearMotor->minForce), std::fabs(_activeConstraint.linearMotor->maxForce)) :
                maxForce;
            const float sharedAngularForce = _activeConstraint.angularMotor ?
                (std::max)(std::fabs(_activeConstraint.angularMotor->minForce), std::fabs(_activeConstraint.angularMotor->maxForce)) :
                0.0f;
            ROCK_LOG_DEBUG(Hand,
                "{} hand dynamic grab created: driveMode={} looseWeapon={} constraint={} proxyBody={} handBody={} objBody={} heldBodies={} linearTau={:.3f} angularTau={:.3f} linearDamping={:.2f} angularDamping={:.2f} linearForce={:.0f} angularForce={:.0f} propRecov={:.1f} constRecov={:.1f}",
                handName(),
                heldObjectDriveModeName(_heldDriveMode),
                _heldObjectIsLooseWeapon ? "yes" : "no",
                _activeConstraint.constraintId,
                _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
                _handBody.getBodyId().value,
                objectBodyId.value,
                _heldBodyIds.size(),
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->tau : tau,
                _activeConstraint.angularMotor ? _activeConstraint.angularMotor->tau : g_rockConfig.rockGrabAngularTau,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->damping : damping,
                _activeConstraint.angularMotor ? _activeConstraint.angularMotor->damping : g_rockConfig.rockGrabAngularDamping,
                sharedLinearForce,
                sharedAngularForce,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->proportionalRecoveryVelocity : proportionalRecovery,
                _activeConstraint.linearMotor ? _activeConstraint.linearMotor->constantRecoveryVelocity : constantRecovery);
        }

        for (auto bid : _heldBodyIds) {
            havok_runtime::acquireBodyFlagLease(world, bid, 0x80, 0, heldBodyFlagLeaseOwner(this));
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
                grabGripPoint,
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
        const RE::NiTransform& initialFingerHandTransform = handWorldTransform;
        root_flattened_finger_skeleton_runtime::Snapshot liveFingerSnapshotAtGrab{};
        const auto* liveFingerSnapshotAtGrabPtr =
            root_flattened_finger_skeleton_runtime::resolveLiveFingerSkeletonSnapshot(_isLeft, liveFingerSnapshotAtGrab) ? &liveFingerSnapshotAtGrab : nullptr;
        const RE::NiPoint3 fingerPosePivotWorld =
            _grabFrame.hasTelemetryCapture ? _grabFrame.grabPivotWorldAtGrab : computeGrabPivotAWorld(world, initialFingerHandTransform);
        const auto initialFingerPoseTargets = rebuildFingerPoseTargetsFromGrabFrame(_grabFrame, objectWorldTransform);
        const auto fingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled ?
            grab_finger_pose_runtime::solveGrabFingerPoseFromTriangles(
                grabMeshTriangles, initialFingerHandTransform, _isLeft,
                fingerPosePivotWorld, initialFingerPoseTargets,
                g_rockConfig.rockGrabFingerMinValue, g_rockConfig.rockGrabMaxTriangleDistance, true, liveFingerSnapshotAtGrabPtr,
                g_rockConfig.rockGrabFingerRejectBacksideHits, g_rockConfig.rockGrabFingerSurfacePlaneToleranceGameUnits,
                _grabFrame.fingerPoseAimValid) :
            grab_finger_pose_runtime::SolvedGrabFingerPose{};
        _grabFingerPose = fingerPose;
        _hasGrabFingerPose = g_rockConfig.rockGrabMeshFingerPoseEnabled;
        _grabFingerProbeStart = fingerPose.probeStart;
        _grabFingerProbeEnd = fingerPose.probeEnd;
        _hasGrabFingerProbeDebug = fingerPose.candidateTriangleCount > 0;
        _grabFingerPosePublished =
            _grabAcquisitionPhase != grab_three_phase::AcquisitionPhase::NearConverging &&
            _grabAcquisitionPhase != grab_three_phase::AcquisitionPhase::GravityPulling;
        if (_grabFingerPosePublished) {
            applyRockGrabHandPose(_isLeft,
                fingerPose,
                _grabFingerJointPose,
                _hasGrabFingerJointPose,
                _grabFingerLocalTransforms,
                _grabFingerLocalTransformMask,
                _hasGrabFingerLocalTransforms,
                0.0f);
        } else {
            ROCK_LOG_DEBUG(Hand,
                "{} THREE-PHASE GRAB POSE: delayed until touch phase candidateTriangles={} poseTargets={}",
                handName(),
                fingerPose.candidateTriangleCount,
                fingerPose.poseTargetCount);
        }

        applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::GrabCommitSucceeded });
        clearPullRuntimeState();
        clearPullCatchIntent(grabbedFromPullCatch ? "pullCatchGrabbed" : "grabbed");

        ROCK_LOG_INFO(Hand, "{} hand grab success -> HeldInit: bodyId={}", handName(), objectBodyId.value);
        return true;
    }

    Hand::HeldHandMotionSample Hand::recordHeldControllerMotionSample(
        const RE::NiTransform& handWorldTransform,
        const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
        float deltaTime)
    {
        HeldHandMotionSample handMotion{};
        const bool warpedPlayerSpace = playerSpaceFrame.enabled && playerSpaceFrame.warp;
        const bool usableDeltaTime = std::isfinite(deltaTime) && deltaTime > 0.000001f;
        const RE::NiPoint3 currentHandPositionHavok = gamePointToHavokPoint(handWorldTransform.translate);
        _lastHeldHandPositionHavok = currentHandPositionHavok;
        _hasLastHeldHandPositionHavok = true;

        if (_hasPreviousHeldRawHandWorld && usableDeltaTime && !warpedPlayerSpace) {
            const RE::NiPoint3 rawHandVelocityHavok = scalePoint(currentHandPositionHavok - _previousHeldHandPositionHavok, 1.0f / deltaTime);
            const RE::NiPoint3 playerVelocityHavok =
                (playerSpaceFrame.enabled && !playerSpaceFrame.warp) ? playerSpaceFrame.velocityHavok : RE::NiPoint3{};
            handMotion.localLinearVelocityHavok = rawHandVelocityHavok - playerVelocityHavok;
            handMotion.hasLocalLinearVelocity = true;

            handMotion.angularVelocityRadiansPerSecond =
                angularVelocityFromRotationDelta(_previousHeldRawHandWorld.rotate, handWorldTransform.rotate, deltaTime);
            handMotion.hasAngularVelocity = lengthSquared(handMotion.angularVelocityRadiansPerSecond) > 0.000001f;

            _heldLocalHandVelocityHistory[_heldHandVelocityHistoryNext] = handMotion.localLinearVelocityHavok;
            _heldHandAngularVelocityHistory[_heldHandVelocityHistoryNext] = handMotion.angularVelocityRadiansPerSecond;
            _heldHandVelocityHistoryNext = (_heldHandVelocityHistoryNext + 1) % _heldLocalHandVelocityHistory.size();
            if (_heldHandVelocityHistoryCount < _heldLocalHandVelocityHistory.size()) {
                ++_heldHandVelocityHistoryCount;
            }
        } else if (warpedPlayerSpace) {
            _heldLocalHandVelocityHistory = {};
            _heldHandAngularVelocityHistory = {};
            _heldHandVelocityHistoryCount = 0;
            _heldHandVelocityHistoryNext = 0;
        }

        _previousHeldRawHandWorld = handWorldTransform;
        _previousHeldHandPositionHavok = currentHandPositionHavok;
        _hasPreviousHeldRawHandWorld = true;
        return handMotion;
    }

    void Hand::recordHeldObjectVelocitySample(RE::hknpWorld* world, const HeldObjectPlayerSpaceFrame& playerSpaceFrame)
    {
        const auto compensationResult = applyHeldMotionCompensation(world, _savedObjectState.bodyId, _heldBodyIds, playerSpaceFrame, _lastPlayerSpaceVelocityHavok);
        if (compensationResult.hasPrimaryVelocity) {
            _heldLocalLinearVelocityHistory[_heldLocalLinearVelocityHistoryNext] = compensationResult.primaryLocalLinearVelocity;
            _heldLocalLinearVelocityHistoryNext = (_heldLocalLinearVelocityHistoryNext + 1) % _heldLocalLinearVelocityHistory.size();
            if (_heldLocalLinearVelocityHistoryCount < _heldLocalLinearVelocityHistory.size()) {
                ++_heldLocalLinearVelocityHistoryCount;
            }
            _lastHeldObjectLocalLinearVelocityHavok = compensationResult.primaryLocalLinearVelocity;
            _hasLastHeldObjectLocalLinearVelocityHavok = true;
        }
        _lastPlayerSpaceVelocityHavok = (playerSpaceFrame.enabled && !playerSpaceFrame.warp) ? playerSpaceFrame.velocityHavok : RE::NiPoint3{};
    }

    void Hand::captureHeldReleaseMotion(
        RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
        float deltaTime)
    {
        if (!isHolding() || !world) {
            return;
        }

        recordHeldControllerMotionSample(handWorldTransform, playerSpaceFrame, deltaTime);
        recordHeldObjectVelocitySample(world, playerSpaceFrame);
    }

    void Hand::applyReleaseVelocitySnapshot(RE::hknpWorld* world, const GrabReleaseOutcome::VelocitySnapshot& snapshot) const
    {
        if (!world || !snapshot.available) {
            return;
        }

        std::vector<std::uint32_t> bodyIds;
        bodyIds.reserve(snapshot.bodyCount);
        const auto count = (std::min<std::uint32_t>)(snapshot.bodyCount, static_cast<std::uint32_t>(snapshot.bodyIds.size()));
        for (std::uint32_t i = 0; i < count; ++i) {
            bodyIds.push_back(snapshot.bodyIds[i]);
        }

        setHeldVelocity(world,
            snapshot.primaryBodyId,
            bodyIds,
            snapshot.linearVelocityHavok,
            snapshot.angularVelocityRadiansPerSecond,
            snapshot.overrideAngularVelocity);
    }

    void Hand::updateHeldObject(RE::hknpWorld* world,
        const RE::NiTransform& handWorldTransform,
        const HeldObjectPlayerSpaceFrame& playerSpaceFrame,
        float deltaTime,
        float forceFadeInTime,
        float tauMin,
        const GrabReleaseContext& releaseContext)
    {
        if (!isHolding() || !world)
            return;
        if (_heldDriveMode == HeldObjectDriveMode::NativeMouseSpring && _nativeGrabReleasePending.load(std::memory_order_acquire)) {
            ROCK_LOG_WARN(Hand, "{} hand release: native mouse-spring flush failure marked grab invalid", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }
        if (_heldDriveMode == HeldObjectDriveMode::NativeMouseSpring && !_nativeGrab.isValid()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }
        if (_heldDriveMode == HeldObjectDriveMode::SharedConstraint && !_activeConstraint.isValid()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }
        if (_heldDriveMode == HeldObjectDriveMode::ProxyConstraint &&
            (_grabAuthorityProxyReleasePending.load(std::memory_order_acquire) || !_activeConstraint.isValid() || !_grabAuthorityProxy.isValid())) {
            ROCK_LOG_WARN(Hand, "{} hand release: proxy constraint authority marked grab invalid", handName());
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }

        if (!_savedObjectState.refr || _savedObjectState.refr->IsDeleted() || _savedObjectState.refr->IsDisabled()) {
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
            return;
        }

        nearby_grab_damping::tickNearbyGrabDamping(world, _nearbyGrabDamping, deltaTime);

        suppressHandCollisionForGrab(world);

        _grabStartTime += deltaTime;

        const HeldHandMotionSample handMotion = recordHeldControllerMotionSample(handWorldTransform, playerSpaceFrame, deltaTime);

        /*
         * ROCK freezes the visible object/node relation in the root-flattened
         * hand frame, then composes it with the rigid-body local transform for
         * the driven body target. The custom proxy constraint consumes the same
         * raw relation in the physics between phase; BODY remains object-space
         * authority and MOTION remains COM/weight/diagnostic data only.
         */
        RE::NiTransform desiredObjectWorld = multiplyTransforms(handWorldTransform, _grabFrame.rawHandSpace);
        const RE::NiTransform desiredBodyTransformRawHandSpace = multiplyTransforms(_grabFrame.rawHandSpace, _grabFrame.bodyLocal);
        RE::NiTransform desiredBodyWorld = multiplyTransforms(handWorldTransform, desiredBodyTransformRawHandSpace);
        RE::NiPoint3 desiredTargetPointWorld =
            native_mouse_spring_grab::computeTargetPointWorldGame(desiredBodyWorld, activeGrabDrivePivotBBodyLocalGame());
        RE::NiPoint3 activePivotBBodyLocalGame = activeGrabDrivePivotBBodyLocalGame();
        RE::NiTransform nativeTargetBodyWorld = desiredBodyWorld;
        float grabPositionErrorGameUnits = 0.0f;
        float grabRotationErrorDegrees = 0.0f;
        bool hasGrabPositionError = false;
        {
            RE::NiTransform grabBodyWorld{};
            if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
                const RE::NiPoint3 liveGripWorld = transform_math::localPointToWorld(grabBodyWorld, activePivotBBodyLocalGame);
                grabPositionErrorGameUnits = pointDistanceGameUnits(liveGripWorld, desiredTargetPointWorld);
                hasGrabPositionError = true;
                if (_grabFrame.heldNode) {
                    grabRotationErrorDegrees = rotationDeltaDegrees(_grabFrame.heldNode->world.rotate, desiredObjectWorld.rotate);
                } else {
                    grabRotationErrorDegrees = rotationDeltaDegrees(grabBodyWorld.rotate, desiredBodyWorld.rotate);
                }
            }
        }

        const bool heldBodyColliding = isHeldBodyColliding();
        grab_held_response::AdaptiveHeldLeadOutput<RE::NiPoint3> adaptiveLead{};
        if (_heldDriveMode == HeldObjectDriveMode::NativeMouseSpring) {
            const float looseWeaponAdaptiveLeadMultiplier =
                _heldObjectIsLooseWeapon ? g_rockConfig.rockGrabLooseWeaponAdaptiveLeadMultiplier : 1.0f;
            adaptiveLead = grab_held_response::solveAdaptiveHeldLead(grab_held_response::AdaptiveHeldLeadInput<RE::NiPoint3>{
                .enabled = g_rockConfig.rockGrabAdaptiveHeldResponseEnabled,
                .heldBodyColliding = heldBodyColliding,
                .hasHandLocalVelocity = handMotion.hasLocalLinearVelocity,
                .hasObjectLocalVelocity = _hasLastHeldObjectLocalLinearVelocityHavok,
                .hasHandAngularVelocity = handMotion.hasAngularVelocity,
                .handLocalVelocityHavok = handMotion.localLinearVelocityHavok,
                .objectLocalVelocityHavok = _lastHeldObjectLocalLinearVelocityHavok,
                .handAngularVelocityRadiansPerSecond = handMotion.angularVelocityRadiansPerSecond,
                .positionErrorGameUnits = grabPositionErrorGameUnits,
                .fullErrorGameUnits = g_rockConfig.rockGrabAdaptiveHeldFullErrorGameUnits,
                .leadTimeMaxSeconds = g_rockConfig.rockGrabAdaptiveHeldLeadTimeMax * looseWeaponAdaptiveLeadMultiplier,
                .maxLeadDistanceGameUnits = g_rockConfig.rockGrabAdaptiveHeldMaxLeadDistanceGameUnits,
                .maxAngularLeadDegrees = g_rockConfig.rockGrabAdaptiveHeldMaxAngularLeadDegrees,
                .havokToGameScale = physics_scale::havokToGame(),
                .collisionLeadScale = g_rockConfig.rockGrabAdaptiveHeldCollisionLeadScale,
            });
            if (adaptiveLead.appliedLinear) {
                nativeTargetBodyWorld.translate = nativeTargetBodyWorld.translate + adaptiveLead.linearLeadGameUnits;
            }
            if (adaptiveLead.appliedAngular) {
                const RE::NiMatrix3 leadRotation = axisAngleStored(handMotion.angularVelocityRadiansPerSecond, adaptiveLead.angularLeadRadians);
                nativeTargetBodyWorld.rotate = applyWorldRotationToStoredBasis(leadRotation, nativeTargetBodyWorld.rotate);
            }

            if (!_nativeGrab.queueTarget(nativeTargetBodyWorld)) {
                ROCK_LOG_WARN(Hand, "{} hand release: native mouse-spring target queue failed", handName());
                releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                return;
            }
        } else if (_heldDriveMode == HeldObjectDriveMode::ProxyConstraint) {
            queueProxyGrabAuthorityTarget(
                handWorldTransform,
                deltaTime,
                forceFadeInTime,
                tauMin,
                grabPositionErrorGameUnits,
                grabRotationErrorDegrees,
                heldBodyColliding);
        } else {
            if (!updateConstraintGrabDriveTarget(
                    world, handWorldTransform, desiredObjectWorld, desiredBodyWorld, desiredTargetPointWorld, activePivotBBodyLocalGame)) {
                ROCK_LOG_WARN(Hand, "{} hand release: shared constraint target update failed", handName());
                releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Immediate, releaseContext);
                return;
            }
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
            releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
            return;
        }

        tickHeldBodyContact();
        if (_heldDriveMode == HeldObjectDriveMode::SharedConstraint) {
            updateConstraintGrabDriveMotors(
                world,
                deltaTime,
                forceFadeInTime,
                tauMin,
                grabPositionErrorGameUnits,
                grabRotationErrorDegrees,
                heldBodyColliding);
        }

        const bool convergingAcquisitionPhase =
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::NearConverging ||
            _grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::GravityPulling;
        const float acquisitionVisualEnvelope =
            grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(
                g_rockConfig.rockGrabTouchAcquireDistanceGameUnits,
                g_rockConfig.rockGrabNearConvergeDistanceGameUnits,
                g_rockConfig.rockGrabAcquisitionVisualStartDistanceGameUnits);
        const bool acquisitionVisualEligible =
            convergingAcquisitionPhase &&
            hasGrabPositionError &&
            _grabObjectGripAtGrab.valid &&
            grabPositionErrorGameUnits <= acquisitionVisualEnvelope;
        if (_grabFrame.hasTelemetryCapture &&
            (_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld || acquisitionVisualEligible)) {
            RE::NiTransform heldVisualNodeWorld{};
            bool hasHeldVisualNodeWorld = false;
            if (_grabFrame.heldNode) {
                heldVisualNodeWorld = _grabFrame.heldNode->world;
                hasHeldVisualNodeWorld = true;
            } else {
                RE::NiTransform grabBodyWorld{};
                if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld)) {
                    heldVisualNodeWorld = deriveNodeWorldFromBodyWorld(grabBodyWorld, _grabFrame.bodyLocal);
                    hasHeldVisualNodeWorld = true;
                }
            }

            if (hasHeldVisualNodeWorld) {
                /*
                 * ROCK visual hand update:
                 *     adjustedHand = heldObjectWorld * inverse(frozenObjectHandSpace)
                 *
                 * This is intentionally visual-only. The native mouse spring target
                 * above remains the only object motor authority, so the rendered
                 * hand can settle to the object without feeding wrist/object
                 * rotation back into the grab relation.
                 */
                RE::NiTransform targetVisualHandWorld =
                    hand_visual_lerp_math::buildHeldObjectRelativeHandWorld(heldVisualNodeWorld, _grabFrame.rawHandSpace);
                targetVisualHandWorld.scale = handWorldTransform.scale;

                if (!_hasGrabVisualHandTransform) {
                    _grabVisualHandTransform = handWorldTransform;
                    _hasGrabVisualHandTransform = true;
                }

                RE::NiTransform nextVisualHandWorld = targetVisualHandWorld;
                if (g_rockConfig.rockGrabHandLerpEnabled) {
                    const auto advancedVisual = hand_visual_lerp_math::advanceTransform(
                        _grabVisualHandTransform,
                        targetVisualHandWorld,
                        g_rockConfig.rockGrabLerpSpeed,
                        g_rockConfig.rockGrabLerpAngularSpeed,
                        deltaTime);
                    nextVisualHandWorld = advancedVisual.transform;
                }

                const float visualHandDeviationGameUnits =
                    pointDistanceGameUnits(nextVisualHandWorld.translate, handWorldTransform.translate);
                _grabVisualDeviationExceededSeconds = held_object_physics_math::advanceDeviationSeconds(
                    _grabVisualDeviationExceededSeconds,
                    visualHandDeviationGameUnits,
                    g_rockConfig.rockGrabMaxDeviation,
                    deltaTime);
                if (held_object_physics_math::deviationExceeded(_grabVisualDeviationExceededSeconds, g_rockConfig.rockGrabMaxDeviationTime)) {
                    ROCK_LOG_WARN(Hand,
                        "{} hand release: visual held-object hand target exceeded max deviation ({:.1f}gu > {:.1f}gu for {:.2f}s)",
                        handName(),
                        visualHandDeviationGameUnits,
                        g_rockConfig.rockGrabMaxDeviation,
                        _grabVisualDeviationExceededSeconds);
                    clearGrabExternalHandWorldTransform(_isLeft);
                    releaseGrabbedObject(world, GrabReleaseCollisionRestoreMode::Delayed, releaseContext);
                    return;
                }

                _grabVisualHandTransform = nextVisualHandWorld;
                applyGrabExternalHandWorldTransform(_isLeft, _grabVisualHandTransform);

                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} GRAB VISUAL HAND: relation=heldRelative phase={} visualOnly=yes target=({:.1f},{:.1f},{:.1f}) applied=({:.1f},{:.1f},{:.1f}) live=({:.1f},{:.1f},{:.1f}) deviation={:.2f}gu normalAuthority=false",
                    handName(),
                    grab_three_phase::phaseName(_grabAcquisitionPhase),
                    targetVisualHandWorld.translate.x,
                    targetVisualHandWorld.translate.y,
                    targetVisualHandWorld.translate.z,
                    _grabVisualHandTransform.translate.x,
                    _grabVisualHandTransform.translate.y,
                    _grabVisualHandTransform.translate.z,
                    handWorldTransform.translate.x,
                    handWorldTransform.translate.y,
                    handWorldTransform.translate.z,
                    visualHandDeviationGameUnits);
            } else {
                _grabVisualDeviationExceededSeconds = 0.0f;
                if (_hasGrabVisualHandTransform) {
                    clearGrabExternalHandWorldTransform(_isLeft);
                    _hasGrabVisualHandTransform = false;
                }
            }
        } else {
            _grabVisualDeviationExceededSeconds = 0.0f;
            if (_hasGrabVisualHandTransform) {
                clearGrabExternalHandWorldTransform(_isLeft);
                _hasGrabVisualHandTransform = false;
            }
        }

        if (convergingAcquisitionPhase && _grabObjectGripAtGrab.valid) {
            const auto previousAcquisitionPhase = _grabAcquisitionPhase;
            RE::NiTransform grabBodyWorld{};
            const bool hasGrabBody = tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabBodyWorld);
            const RE::NiPoint3 liveGripWorld =
                hasGrabBody ? transform_math::localPointToWorld(grabBodyWorld, activePivotBBodyLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 targetGripWorld = desiredTargetPointWorld;
            const RE::NiPoint3 gripError = targetGripWorld - liveGripWorld;
            const float gripErrorGameUnits = hasGrabBody ? std::sqrt(gripError.x * gripError.x + gripError.y * gripError.y + gripError.z * gripError.z) :
                                                           std::numeric_limits<float>::max();
            const float touchDistance = (std::max)(0.1f, g_rockConfig.rockGrabTouchAcquireDistanceGameUnits);
            const auto convergenceDecision =
                grab_three_phase::evaluateConvergencePromotion(grab_three_phase::ConvergencePromotionInput{
                    .hasGrabBody = hasGrabBody,
                    .heldBodyColliding = heldBodyColliding,
                    .gripErrorGameUnits = gripErrorGameUnits,
                    .previousGripErrorGameUnits = _grabConvergePreviousGripErrorGameUnits,
                    .deltaSeconds = deltaTime,
                    .elapsedSeconds = _grabStartTime,
                    .maxTimeSeconds = g_rockConfig.rockGrabConvergeMaxTimeSeconds,
                    .touchDistanceGameUnits = touchDistance,
                    .pocketRadiusGameUnits = g_rockConfig.rockGrabPocketRadiusGameUnits,
                    .stableInsidePocketFrames = _grabConvergeStableInsidePocketFrames,
                    .requiredStableInsidePocketFrames = g_rockConfig.rockGrabConvergeStableFrames,
                    .maxSeparatingSpeedGameUnitsPerSecond = g_rockConfig.rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond,
                });
            _grabConvergeStableInsidePocketFrames = convergenceDecision.nextStableInsidePocketFrames;
            _grabConvergePreviousGripErrorGameUnits = gripErrorGameUnits;
            const bool reachedTouchRange = convergenceDecision.reachedTouchRange;
            const bool convergenceTimedOutInsidePocket = convergenceDecision.timedOutInsidePocket;
            if (convergenceTimedOutInsidePocket) {
                ROCK_LOG_SAMPLE_DEBUG(Hand,
                    g_rockConfig.rockLogSampleMilliseconds,
                    "{} THREE-PHASE GRAB CONVERGE PROMOTION READY: phase={} gripErr={:.2f}gu touch={:.2f}gu pocket={:.2f}gu elapsed={:.3f}s colliding={} stableFrames={} sepSpeed={:.2f}gu/s",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    gripErrorGameUnits,
                    touchDistance,
                    g_rockConfig.rockGrabPocketRadiusGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no",
                    _grabConvergeStableInsidePocketFrames,
                    convergenceDecision.separatingSpeedGameUnitsPerSecond);
            }

            if (hasGrabBody && !reachedTouchRange && !convergenceTimedOutInsidePocket && g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose &&
                !_grabFingerPosePublished) {
                const float nearDistance = (std::max)(touchDistance, g_rockConfig.rockGrabNearConvergeDistanceGameUnits);
                const float progressDenominator = (std::max)(0.001f, nearDistance - touchDistance);
                const float acquisitionProgress = 1.0f - std::clamp((gripErrorGameUnits - touchDistance) / progressDenominator, 0.0f, 1.0f);
                const auto acquisitionFingerPose = buildAcquisitionFingerPose(_grabFingerPose, acquisitionProgress);
                applyRockGrabHandPose(_isLeft,
                    acquisitionFingerPose,
                    _grabFingerJointPose,
                    _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms,
                    _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms,
                    deltaTime,
                    false);
            }

            if (reachedTouchRange || convergenceTimedOutInsidePocket) {
                const char* promotionReason =
                    reachedTouchRange ? "threePhaseTouchReachedFrozenRelation" : "threePhaseTimeoutInsidePocket";
                _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::TouchHeld;
                _grabFrame.fadeInGrabConstraint = false;
                _grabFrame.motorFadeReason = promotionReason;
                ROCK_LOG_DEBUG(Hand,
                    "{} THREE-PHASE GRAB TRANSITION: {} -> TouchHeld relation=frozenRockPointToPalm reason={} gripErr={:.2f}gu elapsed={:.3f}s colliding={} posePublished={}",
                    handName(),
                    grab_three_phase::phaseName(previousAcquisitionPhase),
                    promotionReason,
                    gripErrorGameUnits,
                    _grabStartTime,
                    heldBodyColliding ? "yes" : "no",
                    _grabFingerPosePublished ? "yes" : "no");

                if (g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose && !_grabFingerPosePublished) {
                    applyRockGrabHandPose(_isLeft,
                        _grabFingerPose,
                        _grabFingerJointPose,
                        _hasGrabFingerJointPose,
                        _grabFingerLocalTransforms,
                        _grabFingerLocalTransformMask,
                        _hasGrabFingerLocalTransforms,
                        0.0f);
                    _grabFingerPosePublished = true;
                }
            }
        }

        const float fadeDuration = std::max(forceFadeInTime, 0.0001f);
        const float nativeFadeFactor = _grabFrame.fadeInGrabConstraint ? std::clamp(_grabStartTime / fadeDuration, 0.0f, 1.0f) : 1.0f;

        if (_state == HandState::HeldInit) {
            if (nativeFadeFactor >= 0.999f) {
                applyTransition(HandTransitionRequest{ .event = HandInteractionEvent::HeldFadeComplete });
                ROCK_LOG_DEBUG(Hand,
                    "{} hand: HeldInit -> HeldBody ({} dynamic grab fade complete, {:.2f}s)",
                    handName(),
                    heldObjectDriveModeName(_heldDriveMode),
                    _grabStartTime);
            }
        }

        recordHeldObjectVelocitySample(world, playerSpaceFrame);

        if (g_rockConfig.rockGrabMeshFingerPoseEnabled && _hasGrabFingerPose && _grabFingerPosePublished) {
            const int updateInterval = (std::max)(1, g_rockConfig.rockGrabFingerPoseUpdateInterval);
            _grabFingerPoseAccumulatedDeltaTime += (std::max)(0.0f, std::isfinite(deltaTime) ? deltaTime : 0.0f);
            ++_grabFingerPoseFrameCounter;
            if (_grabFingerPoseFrameCounter >= updateInterval) {
                _grabFingerPoseFrameCounter = 0;
                const float sanitizedDeltaTime = std::isfinite(deltaTime) ? (std::max)(0.0f, deltaTime) : 0.0f;
                const float fingerPoseDeltaTime = _grabFingerPoseAccumulatedDeltaTime > 0.0f ? _grabFingerPoseAccumulatedDeltaTime : sanitizedDeltaTime;
                _grabFingerPoseAccumulatedDeltaTime = 0.0f;
                applyRockGrabHandPose(_isLeft,
                    _grabFingerPose,
                    _grabFingerJointPose,
                    _hasGrabFingerJointPose,
                    _grabFingerLocalTransforms,
                    _grabFingerLocalTransformMask,
                    _hasGrabFingerLocalTransforms,
                    fingerPoseDeltaTime);
            }
        }

        _heldLogCounter++;
        if (_heldLogCounter >= 45) {
            _heldLogCounter = 0;

            const auto nativeState = _nativeGrab.debugState();
            RE::NiTransform liveHandBodyWorld{};
            RE::NiTransform grabObjectBodyWorld{};
            RE::NiTransform motionObjectBodyWorld{};
            const bool hasLiveHandBody =
                _handBody.getBodyId().value != INVALID_BODY_ID && tryResolveLiveBodyWorldTransform(world, _handBody.getBodyId(), liveHandBodyWorld);
            const bool hasGrabObjectBody = tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, grabObjectBodyWorld);
            const bool hasMotionObjectBody = tryResolveLiveBodyWorldTransform(world, _savedObjectState.bodyId, motionObjectBodyWorld);

            if (g_rockConfig.rockDebugGrabFrameLogging) {
                const RE::NiTransform desiredNodeWorldRaw = multiplyTransforms(handWorldTransform, _grabFrame.rawHandSpace);
                const RE::NiTransform desiredNodeWorldConstraint = hasLiveHandBody ? multiplyTransforms(liveHandBodyWorld, _grabFrame.constraintHandSpace) : desiredNodeWorldRaw;
                const RE::NiTransform desiredBodyTransformHandSpaceRaw = multiplyTransforms(_grabFrame.rawHandSpace, _grabFrame.bodyLocal);
                const RE::NiTransform desiredBodyWorldRaw = multiplyTransforms(handWorldTransform, desiredBodyTransformHandSpaceRaw);
                const RE::NiTransform desiredBodyWorldConstraint =
                    hasLiveHandBody ? multiplyTransforms(liveHandBodyWorld, multiplyTransforms(_grabFrame.constraintHandSpace, _grabFrame.bodyLocal)) : desiredBodyWorldRaw;
                const RE::NiTransform grabAuthorityBodyWorld = hasGrabObjectBody ? grabObjectBodyWorld : getGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId);
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
                    metrics.world = node ? node->world : grabAuthorityBodyWorld;
                    metrics.expectedWorld = node ? deriveNodeWorldFromBodyWorld(grabAuthorityBodyWorld, bodyLocalTransform) : grabAuthorityBodyWorld;
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
                const RE::NiPoint3 bodyFinger = getMatrixColumn(grabAuthorityBodyWorld.rotate, 0);
                const float motionDiagVsGrabRot =
                    (hasMotionObjectBody && hasGrabObjectBody) ? rotationDeltaDegrees(motionObjectBodyWorld.rotate, grabObjectBodyWorld.rotate) : -1.0f;
                const float motionDiagVsGrabPos =
                    (hasMotionObjectBody && hasGrabObjectBody) ? translationDeltaGameUnits(motionObjectBodyWorld, grabObjectBodyWorld) : -1.0f;
                const float rawRowMax = max3(axisDeltaDegrees(getMatrixRow(grabAuthorityBodyWorld.rotate, 0), getMatrixRow(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixRow(grabAuthorityBodyWorld.rotate, 1), getMatrixRow(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixRow(grabAuthorityBodyWorld.rotate, 2), getMatrixRow(desiredBodyWorldRaw.rotate, 2)));
                const float rawColMax = max3(axisDeltaDegrees(getMatrixColumn(grabAuthorityBodyWorld.rotate, 0), getMatrixColumn(desiredBodyWorldRaw.rotate, 0)),
                    axisDeltaDegrees(getMatrixColumn(grabAuthorityBodyWorld.rotate, 1), getMatrixColumn(desiredBodyWorldRaw.rotate, 1)),
                    axisDeltaDegrees(getMatrixColumn(grabAuthorityBodyWorld.rotate, 2), getMatrixColumn(desiredBodyWorldRaw.rotate, 2)));

                ROCK_LOG_DEBUG(Hand,
                    "{} GRAB FRAME HOLD: rawVsConstraintWorld={:.2f}deg rawVsConstraintTarget={:.2f}deg "
                    "bodyVsRaw={:.2f}deg bodyVsConstraint={:.2f}deg "
                    "ownerErr={:.2f}deg/{:.2f}gu hitErr={:.2f}deg/{:.2f}gu "
                    "heldErr={:.2f}deg/{:.2f}gu rootErr={:.2f}deg/{:.2f}gu "
                    "worldPosDelta=({:.2f},{:.2f},{:.2f}) "
                    "rawFinger=({:.3f},{:.3f},{:.3f}) constraintFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), rotationDeltaDegrees(desiredNodeWorldRaw.rotate, desiredNodeWorldConstraint.rotate),
                    rotationDeltaDegrees(desiredBodyWorldRaw.rotate, desiredBodyWorldConstraint.rotate),
                    rotationDeltaDegrees(grabAuthorityBodyWorld.rotate, desiredBodyWorldRaw.rotate), rotationDeltaDegrees(grabAuthorityBodyWorld.rotate, desiredBodyWorldConstraint.rotate),
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
                    "rootFinger=({:.3f},{:.3f},{:.3f}) rootExpectedFinger=({:.3f},{:.3f},{:.3f})",
                    handName(), desiredRawFinger.x, desiredRawFinger.y, desiredRawFinger.z, desiredConstraintFinger.x, desiredConstraintFinger.y, desiredConstraintFinger.z,
                    bodyFinger.x, bodyFinger.y, bodyFinger.z, ownerMetrics.finger.x, ownerMetrics.finger.y, ownerMetrics.finger.z, ownerMetrics.expectedFinger.x,
                    ownerMetrics.expectedFinger.y, ownerMetrics.expectedFinger.z, hitMetrics.finger.x, hitMetrics.finger.y, hitMetrics.finger.z, hitMetrics.expectedFinger.x,
                    hitMetrics.expectedFinger.y, hitMetrics.expectedFinger.z, heldMetrics.finger.x, heldMetrics.finger.y, heldMetrics.finger.z, heldMetrics.expectedFinger.x,
                    heldMetrics.expectedFinger.y, heldMetrics.expectedFinger.z, rootMetrics.finger.x, rootMetrics.finger.y, rootMetrics.finger.z, rootMetrics.expectedFinger.x,
                    rootMetrics.expectedFinger.y, rootMetrics.expectedFinger.z);

                ROCK_LOG_TRACE(Hand,
                    "{} GRAB ANGULAR PROBE: rawAxisErr(rowMax={:.2f} colMax={:.2f}) driveErr=({:.2f}gu,{:.2f}deg) mass={:.2f} "
                    "motionDiagVsGrab={:.2f}deg/{:.2f}gu fade={:.2f} fadeEnabled={} fadeReason={} drive={} nativeAction={:p} constraint={} queued={} flushed={} failedFlushes={} lastDt={:.6f}",
                    handName(),
                    rawRowMax,
                    rawColMax,
                    grabPositionErrorGameUnits,
                    grabRotationErrorDegrees,
                    readBodyMass(world, _savedObjectState.bodyId),
                    motionDiagVsGrabRot,
                    motionDiagVsGrabPos,
                    nativeFadeFactor,
                    _grabFrame.fadeInGrabConstraint ? "yes" : "no",
                    _grabFrame.motorFadeReason,
                    heldObjectDriveModeName(_heldDriveMode),
                    nativeState.action,
                    _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                    nativeState.queuedTargets,
                    nativeState.flushedTargets,
                    nativeState.failedFlushes,
                    nativeState.lastFlushDeltaSeconds);
            }

            const bool hasGrabPivotFrame = hasGrabObjectBody;
            const RE::NiPoint3 pivotAWorld = desiredTargetPointWorld;
            const RE::NiPoint3 pivotBWorld = hasGrabPivotFrame ? transform_math::localPointToWorld(grabObjectBodyWorld, activePivotBBodyLocalGame) : RE::NiPoint3{};
            const RE::NiPoint3 pivotError = pivotAWorld - pivotBWorld;
            const float pivotErrGame = hasGrabPivotFrame ? std::sqrt(pivotError.x * pivotError.x + pivotError.y * pivotError.y + pivotError.z * pivotError.z) : 0.0f;

            const RE::NiPoint3 bodyDelta = hasGrabObjectBody ? (grabObjectBodyWorld.translate - desiredBodyWorld.translate) : RE::NiPoint3{};
            const float bodyDistGame = hasGrabObjectBody ? std::sqrt(bodyDelta.x * bodyDelta.x + bodyDelta.y * bodyDelta.y + bodyDelta.z * bodyDelta.z) : 0.0f;

            float objVelMag = 0.0f;
            {
                auto* objMotion = havok_runtime::getBodyMotion(world, _savedObjectState.bodyId);
                if (objMotion) {
                    const auto& velocity = objMotion->linearVelocity;
                    objVelMag = std::sqrt(velocity.x * velocity.x + velocity.y * velocity.y + velocity.z * velocity.z);
                }
            }

            const float adaptiveLeadLinearGame =
                std::sqrt(adaptiveLead.linearLeadGameUnits.x * adaptiveLead.linearLeadGameUnits.x +
                          adaptiveLead.linearLeadGameUnits.y * adaptiveLead.linearLeadGameUnits.y +
                          adaptiveLead.linearLeadGameUnits.z * adaptiveLead.linearLeadGameUnits.z);
            constexpr float kRadiansToDegrees = 57.295779513082320876f;
            const std::uint32_t heldFormId = _savedObjectState.refr ? _savedObjectState.refr->GetFormID() : 0;

            ROCK_LOG_DEBUG(Hand,
                "{} HELD dynamic: drive={} looseWeapon={} formID={:08X} action={:p} constraint={} queued={} flushed={} failedFlushes={} lastDt={:.6f} "
                "phase={} posePublished={} fade={:.2f}/{} reason={} colliding={} adaptiveLeadFactor={:.2f} adaptiveLeadLinear={:.2f}gu adaptiveLeadAngular={:.1f}deg ERR={:.1f}gu rotErr={:.1f}deg bDist={:.1f}gu objVel={:.3f} "
                "paW=({:.1f},{:.1f},{:.1f}) pbW=({:.1f},{:.1f},{:.1f}) "
                "targetBody=({:.1f},{:.1f},{:.1f}) objW=({:.1f},{:.1f},{:.1f})",
                handName(),
                heldObjectDriveModeName(_heldDriveMode),
                _heldObjectIsLooseWeapon ? "yes" : "no",
                heldFormId,
                nativeState.action,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                nativeState.queuedTargets,
                nativeState.flushedTargets,
                nativeState.failedFlushes,
                nativeState.lastFlushDeltaSeconds,
                grab_three_phase::phaseName(_grabAcquisitionPhase),
                _grabFingerPosePublished ? "yes" : "no",
                nativeFadeFactor,
                _grabFrame.fadeInGrabConstraint ? "on" : "off",
                _grabFrame.motorFadeReason,
                heldBodyColliding ? "yes" : "no",
                adaptiveLead.factor,
                adaptiveLeadLinearGame,
                adaptiveLead.angularLeadRadians * kRadiansToDegrees,
                pivotErrGame,
                grabRotationErrorDegrees,
                bodyDistGame,
                objVelMag,
                pivotAWorld.x, pivotAWorld.y, pivotAWorld.z, pivotBWorld.x, pivotBWorld.y, pivotBWorld.z,
                desiredBodyWorld.translate.x, desiredBodyWorld.translate.y, desiredBodyWorld.translate.z,
                grabObjectBodyWorld.translate.x, grabObjectBodyWorld.translate.y, grabObjectBodyWorld.translate.z);

            _notifCounter++;
            if (hasGrabPivotFrame && g_rockConfig.rockDebugShowGrabNotifications && _notifCounter >= 6) {
                _notifCounter = 0;
                const RE::NiPoint3 pivotAToHand = pivotAWorld - desiredBodyWorld.translate;
                const RE::NiPoint3 pivotBToObject = pivotBWorld - grabObjectBodyWorld.translate;
                const float paToHand = std::sqrt(pivotAToHand.x * pivotAToHand.x + pivotAToHand.y * pivotAToHand.y + pivotAToHand.z * pivotAToHand.z);
                const float pbToObj = std::sqrt(pivotBToObject.x * pivotBToObject.x + pivotBToObject.y * pivotBToObject.y + pivotBToObject.z * pivotBToObject.z);
                f4vr::showNotification(
                    std::format("[ROCK] err={:.1f}gu vel={:.2f} flush={} paOff={:.1f} pbOff={:.1f}", pivotErrGame, objVelMag, nativeState.flushedTargets, paToHand, pbToObj));
            }
        }

        {
            const auto wakeBodies = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(_savedObjectState.bodyId.value, _heldBodyIds);
            for (const auto bodyId : wakeBodies) {
                physics_recursive_wrappers::activateBody(world, bodyId);
            }
        }
    }

    void Hand::flushPendingHeldNativeGrab(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        if (!_nativeGrab.isValid()) {
            return;
        }

        const float driveDelta = havok_physics_timing::driveDeltaSeconds(timing);
        if (!_nativeGrab.flush(world, driveDelta)) {
            const auto state = _nativeGrab.debugState();
            _nativeGrabReleasePending.store(true, std::memory_order_release);
            ROCK_LOG_WARN(Hand,
                "{} hand native dynamic grab flush failed; release queued: world={:p} action={:p} bodyId={} queued={} flushed={} failedFlushes={}",
                handName(),
                static_cast<void*>(world),
                state.action,
                state.bodyId.value,
                state.queuedTargets,
                state.flushedTargets,
                state.failedFlushes);
        }
    }

    void Hand::flushPendingCustomGrabAuthority(RE::hknpWorld* world, const havok_physics_timing::PhysicsTimingSample& timing)
    {
        GrabAuthorityProxyPendingTarget pending{};
        RE::NiTransform previousProxyWorld{};
        RE::hknpBodyId proxyBodyId{ INVALID_BODY_ID };
        bool setTransformOk = false;
        bool setVelocityOk = false;
        bool targetUpdateOk = false;
        bool shouldLog = false;
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiPoint3 desiredTargetPointWorld{};
        RE::NiPoint3 activePivotBBodyLocalGame{};
        {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            const bool hasAuthority = _heldDriveMode == HeldObjectDriveMode::ProxyConstraint &&
                                      _grabAuthorityProxy.isValid() &&
                                      _grabAuthorityProxyHknpWorld == world &&
                                      _grabAuthorityPendingTarget.valid;
            if (!hasAuthority) {
                return;
            }

            pending = _grabAuthorityPendingTarget;
            previousProxyWorld = _hasLastAppliedGrabAuthorityProxyWorld ? _lastAppliedGrabAuthorityProxyWorld : pending.proxyWorld;
            proxyBodyId = _grabAuthorityProxy.getBodyId();

            const float driveDelta = havok_physics_timing::driveDeltaSeconds(timing);
            const RE::hkTransformf targetHavok = grab_authority_proxy::makeHavokTransform(pending.proxyWorld);
            float linearVelocityHavok[4]{};
            float angularVelocityHavok[4]{};
            grab_authority_proxy::computeLinearVelocityHavok(previousProxyWorld, pending.proxyWorld, driveDelta, linearVelocityHavok);

            setTransformOk = _grabAuthorityProxy.setTransform(targetHavok);
            setVelocityOk = _grabAuthorityProxy.setVelocity(linearVelocityHavok, angularVelocityHavok);
            if (!setTransformOk || !setVelocityOk) {
                _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
            } else {
                targetUpdateOk = updateProxyConstraintGrabDriveTarget(
                    world,
                    pending.proxyWorld,
                    desiredObjectWorld,
                    desiredBodyWorld,
                    desiredTargetPointWorld,
                    activePivotBBodyLocalGame);
                if (!targetUpdateOk) {
                    _grabAuthorityProxyReleasePending.store(true, std::memory_order_release);
                } else {
                    updateConstraintGrabDriveMotors(
                        world,
                        driveDelta,
                        pending.forceFadeInTime,
                        pending.tauMin,
                        pending.grabPositionErrorGameUnits,
                        pending.grabRotationErrorDegrees,
                        pending.heldBodyColliding);

                    _lastAppliedGrabAuthorityProxyWorld = pending.proxyWorld;
                    _hasLastAppliedGrabAuthorityProxyWorld = true;
                    ++_grabAuthorityProxyFlushSequence;
                    ++_grabAuthorityProxyLogCounter;
                    if (_grabAuthorityProxyLogCounter >= 45) {
                        _grabAuthorityProxyLogCounter = 0;
                        shouldLog = true;
                    }
                }
            }
        }

        if (!setTransformOk || !setVelocityOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab drive failed; release queued: proxyBody={} setTransform={} setVelocity={} substep={}/{} dt={:.6f}",
                handName(),
                proxyBodyId.value,
                setTransformOk ? "ok" : "fail",
                setVelocityOk ? "ok" : "fail",
                timing.substepIndex,
                timing.substepCount,
                havok_physics_timing::driveDeltaSeconds(timing));
            return;
        }

        if (!targetUpdateOk) {
            ROCK_LOG_WARN(Hand,
                "{} hand proxy dynamic grab target update failed; release queued: proxyBody={} constraint={} substep={}/{}",
                handName(),
                proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount);
            return;
        }

        if (shouldLog && g_rockConfig.rockDebugGrabFrameLogging) {
            std::uint32_t filterInfo = 0;
            const bool filterReadOk = havok_runtime::tryReadFilterInfo(world, proxyBodyId, filterInfo);
            ROCK_LOG_DEBUG(Hand,
                "{} PROXY GRAB AUTHORITY: proxyBody={} constraint={} substep={}/{} dt={:.6f} target=({:.1f},{:.1f},{:.1f}) desiredBody=({:.1f},{:.1f},{:.1f}) pivotB=({:.2f},{:.2f},{:.2f}) err={:.2f}gu rotErr={:.2f}deg colliding={} filterRead={} filter=0x{:08X} noContact={}",
                handName(),
                proxyBodyId.value,
                _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
                timing.substepIndex,
                timing.substepCount,
                havok_physics_timing::driveDeltaSeconds(timing),
                pending.proxyWorld.translate.x,
                pending.proxyWorld.translate.y,
                pending.proxyWorld.translate.z,
                desiredBodyWorld.translate.x,
                desiredBodyWorld.translate.y,
                desiredBodyWorld.translate.z,
                activePivotBBodyLocalGame.x,
                activePivotBBodyLocalGame.y,
                activePivotBBodyLocalGame.z,
                pending.grabPositionErrorGameUnits,
                pending.grabRotationErrorDegrees,
                pending.heldBodyColliding ? "yes" : "no",
                filterReadOk ? "ok" : "fail",
                filterInfo,
                grab_authority_proxy::hasNoContactFilterInfo(filterInfo) ? "yes" : "no");
        }
    }

    GrabReleaseOutcome Hand::releaseGrabbedObject(
        RE::hknpWorld* world,
        GrabReleaseCollisionRestoreMode collisionRestoreMode,
        const GrabReleaseContext& releaseContext)
    {
        GrabReleaseOutcome outcome{};
        outcome.finalObjectRelease = releaseContext.finalObjectRelease;
        if (!isHolding()) {
            return outcome;
        }

        outcome.released = true;
        outcome.refr = _savedObjectState.refr;
        outcome.formID = outcome.refr ? outcome.refr->GetFormID() : 0;

        const auto nativeState = _nativeGrab.debugState();
        ROCK_LOG_INFO(Hand,
            "{} hand RELEASE: bodyId={} nativeAction={:p} nativeQueued={} nativeFlushed={} nativeFailed={} constraintId={} proxyBody={} finalObjectRelease={} disposition={} reason={}",
            handName(),
            _savedObjectState.bodyId.value,
            nativeState.action,
            nativeState.queuedTargets,
            nativeState.flushedTargets,
            nativeState.failedFlushes,
            _activeConstraint.isValid() ? _activeConstraint.constraintId : 0x7FFF'FFFFu,
            _grabAuthorityProxy.isValid() ? _grabAuthorityProxy.getBodyId().value : INVALID_BODY_ID,
            releaseContext.finalObjectRelease ? "yes" : "no",
            releaseDispositionName(releaseContext.disposition),
            releaseContext.reason ? releaseContext.reason : "none");

        nearby_grab_damping::restoreNearbyGrabDamping(world, _nearbyGrabDamping);

        const bool captureReleaseVelocity =
            (releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop ||
                releaseContext.disposition == GrabReleaseDisposition::PendingInventoryTransfer) &&
            releaseContext.finalObjectRelease && world && (_heldLocalLinearVelocityHistoryCount > 0 || _heldHandVelocityHistoryCount > 0);
        if (captureReleaseVelocity) {
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedObjectHistory{};
            const std::size_t historySize = _heldLocalLinearVelocityHistory.size();
            const std::size_t firstIndex = (_heldLocalLinearVelocityHistoryNext + historySize - _heldLocalLinearVelocityHistoryCount) % historySize;
            for (std::size_t i = 0; i < _heldLocalLinearVelocityHistoryCount; ++i) {
                orderedObjectHistory[i] = _heldLocalLinearVelocityHistory[(firstIndex + i) % historySize];
            }

            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedHandHistory{};
            std::array<RE::NiPoint3, GRAB_RELEASE_VELOCITY_HISTORY> orderedAngularHistory{};
            const std::size_t handHistorySize = _heldLocalHandVelocityHistory.size();
            const std::size_t firstHandIndex = (_heldHandVelocityHistoryNext + handHistorySize - _heldHandVelocityHistoryCount) % handHistorySize;
            for (std::size_t i = 0; i < _heldHandVelocityHistoryCount; ++i) {
                const std::size_t sourceIndex = (firstHandIndex + i) % handHistorySize;
                orderedHandHistory[i] = _heldLocalHandVelocityHistory[sourceIndex];
                orderedAngularHistory[i] = _heldHandAngularVelocityHistory[sourceIndex];
            }

            const RE::NiPoint3 objectLocalReleaseVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedObjectHistory, _heldLocalLinearVelocityHistoryCount);
            const RE::NiPoint3 handLocalReleaseVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedHandHistory, _heldHandVelocityHistoryCount);
            const RE::NiPoint3 handAngularVelocity =
                held_object_physics_math::maxMagnitudeVelocity(orderedAngularHistory, _heldHandVelocityHistoryCount);

            RE::NiPoint3 tangentialVelocityHavok{};
            bool hasTangentialVelocity = false;
            RE::NiPoint3 releaseLeverOriginHavok = _lastHeldHandPositionHavok;
            bool hasReleaseLeverOrigin = _hasLastHeldHandPositionHavok;
            RE::NiTransform releaseBodyWorld{};
            if (tryGetGrabAuthorityBodyWorldTransform(world, _savedObjectState.bodyId, releaseBodyWorld)) {
                releaseLeverOriginHavok =
                    gamePointToHavokPoint(transform_math::localPointToWorld(releaseBodyWorld, activeGrabDrivePivotBBodyLocalGame()));
                hasReleaseLeverOrigin = true;
            }
            if (hasReleaseLeverOrigin && lengthSquared(handAngularVelocity) > 0.000001f) {
                float comX = 0.0f;
                float comY = 0.0f;
                float comZ = 0.0f;
                if (havok_runtime::getBodyCOMWorld(world, _savedObjectState.bodyId, comX, comY, comZ)) {
                    tangentialVelocityHavok = grab_held_response::computeTangentialVelocityFromAngularSwing(
                        handAngularVelocity,
                        releaseLeverOriginHavok,
                        RE::NiPoint3{ comX, comY, comZ });
                    hasTangentialVelocity = lengthSquared(tangentialVelocityHavok) > 0.000001f;
                }
            }

            const RE::NiPoint3 releaseVelocity =
                grab_held_response::composeControllerReleaseVelocity(grab_held_response::ReleaseVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandLocalVelocity = _heldHandVelocityHistoryCount > 0,
                    .hasObjectLocalVelocity = _heldLocalLinearVelocityHistoryCount > 0,
                    .hasTangentialVelocity = hasTangentialVelocity,
                    .handLocalVelocityHavok = handLocalReleaseVelocity,
                    .objectLocalVelocityHavok = objectLocalReleaseVelocity,
                    .playerVelocityHavok = _lastPlayerSpaceVelocityHavok,
                    .tangentialVelocityHavok = tangentialVelocityHavok,
                    .objectVelocityBlend = g_rockConfig.rockGrabThrowObjectVelocityBlend,
                    .tangentialVelocityScale = g_rockConfig.rockGrabThrowTangentialVelocityScale,
                    .throwMultiplier = g_rockConfig.rockThrowVelocityMultiplier,
                    .maxVelocityHavok = g_rockConfig.rockGrabThrowMaxVelocityHavok,
                });
            const RE::NiPoint3 releaseAngularVelocity =
                grab_held_response::composeControllerReleaseAngularVelocity(grab_held_response::ReleaseAngularVelocityInput<RE::NiPoint3>{
                    .controllerDerivedEnabled = g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled,
                    .hasHandAngularVelocity = _heldHandVelocityHistoryCount > 0,
                    .handAngularVelocityRadiansPerSecond = handAngularVelocity,
                    .angularVelocityScale = g_rockConfig.rockGrabThrowAngularVelocityScale,
                    .maxAngularVelocityRadiansPerSecond = g_rockConfig.rockGrabThrowMaxAngularVelocityRadiansPerSecond,
                });
            const bool overrideAngularVelocity =
                g_rockConfig.rockGrabControllerDerivedThrowVelocityEnabled && lengthSquared(releaseAngularVelocity) > 0.000001f;
            outcome.velocity.available = true;
            outcome.velocity.primaryBodyId = _savedObjectState.bodyId;
            outcome.velocity.linearVelocityHavok = releaseVelocity;
            outcome.velocity.angularVelocityRadiansPerSecond = releaseAngularVelocity;
            outcome.velocity.overrideAngularVelocity = overrideAngularVelocity;
            for (const auto bodyId : _heldBodyIds) {
                if (outcome.velocity.bodyCount >= outcome.velocity.bodyIds.size()) {
                    break;
                }
                outcome.velocity.bodyIds[outcome.velocity.bodyCount++] = bodyId;
            }

            const bool applyReleaseVelocity = releaseContext.disposition == GrabReleaseDisposition::PhysicalDrop;
            if (applyReleaseVelocity) {
                setHeldVelocity(world, _savedObjectState.bodyId, _heldBodyIds, releaseVelocity, releaseAngularVelocity, overrideAngularVelocity);
            }
            ROCK_LOG_DEBUG(Hand,
                "{} hand RELEASE VELOCITY: applied={} handLocal=({:.3f},{:.3f},{:.3f}) objectLocal=({:.3f},{:.3f},{:.3f}) tangent=({:.3f},{:.3f},{:.3f}) angularRaw=({:.3f},{:.3f},{:.3f}) angularFinal=({:.3f},{:.3f},{:.3f}) player=({:.3f},{:.3f},{:.3f}) final=({:.3f},{:.3f},{:.3f}) lever=({:.3f},{:.3f},{:.3f}) objectHistory={} handHistory={} multiplier={:.2f}",
                handName(),
                applyReleaseVelocity ? "yes" : "no",
                handLocalReleaseVelocity.x,
                handLocalReleaseVelocity.y,
                handLocalReleaseVelocity.z,
                objectLocalReleaseVelocity.x,
                objectLocalReleaseVelocity.y,
                objectLocalReleaseVelocity.z,
                tangentialVelocityHavok.x,
                tangentialVelocityHavok.y,
                tangentialVelocityHavok.z,
                handAngularVelocity.x,
                handAngularVelocity.y,
                handAngularVelocity.z,
                releaseAngularVelocity.x,
                releaseAngularVelocity.y,
                releaseAngularVelocity.z,
                _lastPlayerSpaceVelocityHavok.x,
                _lastPlayerSpaceVelocityHavok.y,
                _lastPlayerSpaceVelocityHavok.z,
                releaseVelocity.x,
                releaseVelocity.y,
                releaseVelocity.z,
                releaseLeverOriginHavok.x,
                releaseLeverOriginHavok.y,
                releaseLeverOriginHavok.z,
                _heldLocalLinearVelocityHistoryCount,
                _heldHandVelocityHistoryCount,
                g_rockConfig.rockThrowVelocityMultiplier);
        }

        if (releaseContext.finalObjectRelease) {
            restoreGrabbedInertia(world, _savedObjectState);
        }

        if (releaseContext.finalObjectRelease && world && _activeGrabLifecycle.size() > 0) {
            restoreActiveGrabLifecycle(world,
                _activeGrabLifecycle,
                _activeGrabLifecycle.restorePlanForRelease(active_grab_body_lifecycle::BodyRestorePolicy::ProtectComplexSystemOwned),
                _savedObjectState.bodyId.value,
                handName(),
                "release");
            if (_activeGrabLifecycle.hasIncompleteNativeScan()) {
                auto* rootNode = _savedObjectState.refr ? _savedObjectState.refr->Get3D() : nullptr;
                restoreIncompleteActivePrepRoot(rootNode, _savedObjectState.originalMotionPropsId, handName(), "release-incomplete-scan");
            }
        }

        _isHoldingFlag.store(false, std::memory_order_release);
        _heldBodyIdsCount.store(0, std::memory_order_release);
        _heldBodyContactFrame.store(100, std::memory_order_release);

        if (world) {
            for (auto bid : _heldBodyIds) {
                havok_runtime::releaseBodyFlagLease(world, bid, 0x80, 0, heldBodyFlagLeaseOwner(this), releaseContext.finalObjectRelease);
            }
        }

        if (_heldDriveMode == HeldObjectDriveMode::ProxyConstraint) {
            std::scoped_lock lock(_grabAuthorityProxyMutex);
            if (_activeConstraint.isValid()) {
                destroyGrabConstraint(world, _activeConstraint);
            }
            destroyGrabAuthorityProxyLocked(nullptr);
        } else {
            if (_activeConstraint.isValid()) {
                destroyGrabConstraint(world, _activeConstraint);
            }
            if (_nativeGrab.isValid()) {
                _nativeGrab.destroy(world, releaseContext.finalObjectRelease);
            }
            if (_grabAuthorityProxy.isValid()) {
                destroyGrabAuthorityProxy(nullptr);
            } else {
                clearGrabAuthorityProxyRuntime();
            }
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
        _nativeGrab.clear();
        clearGrabAuthorityProxyRuntime();
        _heldDriveMode = HeldObjectDriveMode::NativeMouseSpring;
        _heldBodyIds.clear();
        _grabFrame.clear();
        _grabAcquisitionPhase = grab_three_phase::AcquisitionPhase::Idle;
        _grabObjectGripAtGrab = {};
        _heldObjectIsLooseWeapon = false;
        _grabFingerPosePublished = false;
        _grabConvergeStableInsidePocketFrames = 0;
        _grabConvergePreviousGripErrorGameUnits = std::numeric_limits<float>::max();
        _grabDeviationExceededSeconds = 0.0f;
        _grabVisualHandTransform = {};
        _hasGrabVisualHandTransform = false;
        _grabVisualDeviationExceededSeconds = 0.0f;
        _nativeGrabReleasePending.store(false, std::memory_order_release);
        _grabFingerProbeStart = {};
        _grabFingerProbeEnd = {};
        _hasGrabFingerProbeDebug = false;
        _grabFingerJointPose = {};
        _grabFingerLocalTransforms = {};
        _grabFingerLocalTransformMask = 0;
        _grabFingerPose = {};
        _hasGrabFingerJointPose = false;
        _hasGrabFingerLocalTransforms = false;
        _hasGrabFingerPose = false;
        _grabFingerPoseFrameCounter = 0;
        _grabFingerPoseAccumulatedDeltaTime = 0.0f;
        _heldLocalLinearVelocityHistory = {};
        _heldLocalLinearVelocityHistoryCount = 0;
        _heldLocalLinearVelocityHistoryNext = 0;
        _heldLocalHandVelocityHistory = {};
        _heldHandAngularVelocityHistory = {};
        _heldHandVelocityHistoryCount = 0;
        _heldHandVelocityHistoryNext = 0;
        _lastHeldObjectLocalLinearVelocityHavok = {};
        _hasLastHeldObjectLocalLinearVelocityHavok = false;
        _previousHeldRawHandWorld = {};
        _previousHeldHandPositionHavok = {};
        _lastHeldHandPositionHavok = {};
        _hasPreviousHeldRawHandWorld = false;
        _hasLastHeldHandPositionHavok = false;
        _lastPlayerSpaceVelocityHavok = {};
        _currentSelection.clear();
        const HandInteractionEvent releaseEvent =
            releaseContext.disposition == GrabReleaseDisposition::TransferToInventory && _state == HandState::StashCandidate ?
                HandInteractionEvent::CommitStash :
                HandInteractionEvent::ReleaseRequested;
        applyTransition(HandTransitionRequest{ .event = releaseEvent });

        ROCK_LOG_DEBUG(Hand, "{} hand: Idle", handName());
        return outcome;
    }
}

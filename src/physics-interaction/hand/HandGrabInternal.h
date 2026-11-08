// Shared internals of the hand grab implementation (HandGrab*.cpp,
// HandHeldObject.cpp, HandGrabDebug.cpp): vector and transform helpers in the
// grab code's conventions, grab evidence frames, held-body activation, flag
// leases, mass and velocity, grab motor tuning, active-grab lifecycle restore
// and grab finger pose application, all inline in rock::hand_grab_internal;
// then the definitions of Hand's nested grab-stage records (Hand.h declares
// them), which pass data between the grab stages in different files.
// Include only from those files.
#pragma once

#include "api/ProviderRuntimeServices.h"
#include "physics-interaction/core/PhysicsInteraction.h"
#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/HandFingerMirrorMath.h"

#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/scene/HeldScenePresentation.h"

#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/TransformMath.h"
#include "physics-interaction/VectorMath.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/consume/ImmersiveAid.h"
#include "physics-interaction/consume/ImmersiveAidPose.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabFingerCapsuleSweep.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/GrabLocalGraspSolve.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabPalmSeat.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/grab/SavedGrabOffsetStore.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/hand/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/native/havok/BodyCollisionControl.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "physics-interaction/native/havok/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/havok/PhysicsScale.h"
#include "physics-interaction/native/havok/PhysicsUtils.h"
#include "physics-interaction/native/query/PhysicsShapeCast.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikHandWorldAuthority.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/weapon/authored/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/classification/WeaponTypePolicy.h"
#include "physics-interaction/weapon/grip/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/grip/WeaponSupport.h"
#include "physics-interaction/weapon/handling/GunstockMode.h"
#include "rock_support/Fo4VrRuntime.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <xmmintrin.h>

namespace rock::hand_grab_internal
{
    inline hand_visual_lerp_math::VisualReturnConfig looseWeaponAcquireConfig()
    {
        return {
            .minSeconds = g_rockConfig.rockWeaponSupportGripHandLerpTimeMin,
            .maxSeconds = g_rockConfig.rockWeaponSupportGripHandLerpTimeMax,
            .minDistanceGameUnits = g_rockConfig.rockWeaponSupportGripHandLerpMinDistance,
            .maxDistanceGameUnits = g_rockConfig.rockWeaponSupportGripHandLerpMaxDistance,
            .minAngleDegrees = hand_visual_lerp_math::kEquippedWeaponReturnConfig.minAngleDegrees,
            .maxAngleDegrees = hand_visual_lerp_math::kEquippedWeaponReturnConfig.maxAngleDegrees,
        };
    }

    struct GrabRollbackAction
    {
        void* context = nullptr;
        void (*invoke)(void*) noexcept = nullptr;

        void rollback() const noexcept
        {
            if (context && invoke) {
                invoke(context);
            }
        }
    };

    static_assert(kGrabCollisionSuppressionArmBodyCountPerHand == kBodyBoneGrabSuppressionArmBodyCountPerSide,
        "Normal grab arm-collider suppression capacity must match the body collider arm-chain query.");

    inline RE::NiPoint3 normalizeOrZero(const RE::NiPoint3& value)
    {
        const float lengthSquared = value.x * value.x + value.y * value.y + value.z * value.z;
        if (lengthSquared <= 1.0e-8f) {
            return RE::NiPoint3{};
        }

        const float inverseLength = 1.0f / std::sqrt(lengthSquared);
        return RE::NiPoint3(value.x * inverseLength, value.y * inverseLength, value.z * inverseLength);
    }

    inline float lengthSquared(const RE::NiPoint3& value) { return vector_math::lengthSquared(value); }

    inline RE::NiPoint3 crossProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs)
    {
        return vector_math::cross(lhs, rhs);
    }

    inline float dotProduct(const RE::NiPoint3& lhs, const RE::NiPoint3& rhs) { return vector_math::dot(lhs, rhs); }

    inline RE::NiPoint3 stablePerpendicularAxis(const RE::NiPoint3& normal)
    {
        const RE::NiPoint3 normalizedNormal = normalizeOrZero(normal);
        if (lengthSquared(normalizedNormal) <= 0.000001f) {
            return {};
        }

        const RE::NiPoint3 reference =
            std::fabs(normalizedNormal.z) < 0.75f ? RE::NiPoint3{ 0.0f, 0.0f, 1.0f } : RE::NiPoint3{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 axis = normalizeOrZero(crossProduct(reference, normalizedNormal));
        if (lengthSquared(axis) <= 0.000001f) {
            axis = normalizeOrZero(crossProduct(RE::NiPoint3{ 0.0f, 1.0f, 0.0f }, normalizedNormal));
        }
        return axis;
    }

    inline RE::NiPoint3 gamePointToHavokPoint(const RE::NiPoint3& value)
    {
        const float scale = physics_scale::gameToHavok();
        return RE::NiPoint3{ value.x * scale, value.y * scale, value.z * scale };
    }

    inline RE::NiPoint3 angularVelocityFromRotationDelta(const RE::NiMatrix3& previous, const RE::NiMatrix3& current,
        float deltaTime)
    {
        return held_object_physics_math::angularVelocityFromRotationDelta<RE::NiMatrix3, RE::NiPoint3>(previous,
            current, deltaTime);
    }

    inline RE::NiPoint3 rotationCorrectionAxisWorld(const RE::NiMatrix3& current, const RE::NiMatrix3& target)
    {
        return normalizeOrZero(angularVelocityFromRotationDelta(current, target, 1.0f));
    }

    inline const char* nodeDebugName(const RE::NiAVObject* node)
    {
        if (!node) {
            return "(null)";
        }

        const char* name = node->name.c_str();
        return name ? name : "(unnamed)";
    }

    inline const char* primaryBodyChoiceReasonName(object_physics_body_set::PrimaryBodyChoiceReason reason)
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

    inline float pointDistanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        const RE::NiPoint3 delta = a - b;
        return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }

    inline float vectorMagnitude(const RE::NiPoint3& value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
    }

    inline float sharedGrabAuthorityForceScale(bool peerHandStillHolding)
    {
        /*
             * Two ROCK proxy constraints on one loose object must share the
             * finite force budget instead of each hand receiving a full HIGGS-
             * style mass-capped motor. The selected grip points remain
             * independent; only the total per-object authority is budgeted.
             */
        return peerHandStillHolding ? 0.5f : 1.0f;
    }

    inline const char* bodyMotionTypeName(physics_body_classifier::BodyMotionType motionType)
    {
        using physics_body_classifier::BodyMotionType;
        switch (motionType) {
        case BodyMotionType::Static:
            return "Static";
        case BodyMotionType::Dynamic:
            return "Dynamic";
        case BodyMotionType::Keyframed:
            return "Keyframed";
        case BodyMotionType::Other:
            return "Other";
        case BodyMotionType::Unknown:
        default:
            return "Unknown";
        }
    }

    inline const object_physics_body_set::ObjectPhysicsBodyRecord* diagnosticRejectedBodyRecord(
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet, std::uint32_t preferredBodyId)
    {
        if (const auto* preferred = bodySet.findRecord(preferredBodyId); preferred && !preferred->accepted) {
            return preferred;
        }
        for (const auto& record : bodySet.records) {
            if (!record.accepted) {
                return &record;
            }
        }
        return nullptr;
    }

    inline float effectiveGrabMotorMass(float mass)
    {
        return grab_motion_controller::effectiveMotorMass(mass, g_rockConfig.rockGrabEffectiveMotorMassFloorEnabled,
            g_rockConfig.rockGrabEffectiveMotorMassFloor);
    }

    inline bool isFiniteNiTransform(const RE::NiTransform& value)
    {
        bool rotationFinite = true;
        for (std::uint32_t row = 0; row < 3; ++row) {
            for (std::uint32_t column = 0; column < 3; ++column) {
                rotationFinite = rotationFinite && std::isfinite(value.rotate.entry[row][column]);
            }
        }
        return rotationFinite && std::isfinite(value.translate.x) && std::isfinite(value.translate.y) &&
            std::isfinite(value.translate.z) && std::isfinite(value.scale) && value.scale > 0.0001f;
    }

    inline bool nodeIsOrDescendsFrom(const RE::NiAVObject* root, const RE::NiAVObject* node);

    inline RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child);

    inline RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld,
        const RE::NiTransform& bodyLocalTransform);

    inline const RE::TESObjectWEAP* looseWeaponFormFromRef(RE::TESObjectREFR* refr)
    {
        auto* selectedBase = refr ? refr->GetObjectReference() : nullptr;
        return selectedBase ? selectedBase->As<RE::TESObjectWEAP>() : nullptr;
    }

    inline const RE::TESObjectWEAP* selectedLooseWeaponForm(const SelectedObject& selection)
    {
        return looseWeaponFormFromRef(selection.refr);
    }

    inline bool isThrowableLooseWeapon(const RE::TESObjectWEAP* weapon)
    {
        if (!weapon) {
            return false;
        }
        /*
             * WEAPON_TYPE is stored as a single enum value in FO4VR. Keep this
             * as direct equality instead of EnumSet::any so guns cannot alias
             * thrown types through bit-style tests.
             */
        return weapon->weaponData.type == RE::WEAPON_TYPE::kGrenade ||
            weapon->weaponData.type == RE::WEAPON_TYPE::kMine;
    }

    inline bool nodeIsOrDescendsFrom(const RE::NiAVObject* root, const RE::NiAVObject* node)
    {
        if (!root || !node) {
            return false;
        }

        for (unsigned depth = 0; node && depth < 64; ++depth, node = node->parent) {
            if (node == root) {
                return true;
            }
        }
        return false;
    }

    inline bool acceptsSelectedOwnerlessVisualMesh(const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet, std::uint32_t resolvedBodyId,
        RE::NiAVObject* surfaceOwnerNode, const object_physics_body_set::ObjectPhysicsBodyRecord* surfaceOwnerRecord)
    {
        if (!selection.refr || selection.refr != bodySet.rootRef ||
            resolvedBodyId == object_physics_body_set::INVALID_BODY_ID || resolvedBodyId != selection.bodyId.value ||
            !surfaceOwnerNode || surfaceOwnerRecord || !nodeIsOrDescendsFrom(bodySet.rootNode, surfaceOwnerNode)) {
            return false;
        }
        const auto* selectedBody = bodySet.findRecord(resolvedBodyId);
        if (!selectedBody || !selectedBody->accepted)
            return false;

        // Preserve the established selected-body authority for multipart refs.
        if (bodySet.acceptedCount() > 1)
            return true;

        // Single-body modular weapons can place the handguard/magazine beside
        // the receiver's collision branch. A seeded body is valid evidence
        // even when its native system cannot be enumerated (benignScanSkips),
        // but the seed alone must not stand in for verified reference ownership.
        if (selection.targetKind != grab_target::Kind::LooseObject || !looseWeaponFormFromRef(selection.refr) ||
            selection.refr->Get3D() != bodySet.rootNode || bodySet.records.size() != 1 ||
            !selectedBody->refResolutionKnown || selectedBody->resolvedRef != selection.refr ||
            bodySet.diagnostics.scanFailures || bodySet.diagnostics.invalidPhysicsSystems ||
            bodySet.diagnostics.depthLimitSkips || bodySet.diagnostics.foreignRefBodySkips ||
            bodySet.diagnostics.unresolvedRefBodySkips) {
            return false;
        }
        return grab_contact_source_policy::isSingleBodyOwnerlessVisualBranch(bodySet.rootNode, selectedBody->owningNode,
            surfaceOwnerNode);
    }

    inline constexpr const char* kHeldObjectDriveName = "proxyConstraint";

    constexpr std::uint32_t heldCollisionParticipationFlags(grab_target::Kind targetKind)
    {
        // Key-2 manifold events (0x40, also used by hands/bipod) report a
        // loose prop's sustained surface contact even without an impact.
        // Other grab kinds keep their existing notification contract.
        return 0x80u | (targetKind == grab_target::Kind::LooseObject ? 0x40u : 0u);
    }

    inline constexpr std::uint32_t kHeldCollisionParticipationFlagMode = 0u;

    inline constexpr std::uint32_t kHeldAuthorityBodyFlags = 0x08000000u;

    inline constexpr std::uint32_t kHeldAuthorityBodyFlagMode = 1u;

    inline std::uintptr_t heldBodyFlagLeaseOwner(const Hand* hand)
    {
        return reinterpret_cast<std::uintptr_t>(hand) ^ 0x524F434B48454C44ull;
    }

    struct HeldBodyActivationSummary
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t activatedCount = 0;
        std::uint32_t failedActivationCount = 0;
    };

    struct HeldBodyFlagLeaseSummary
    {
        std::uint32_t bodyCount = 0;
        std::uint32_t collisionLeaseCount = 0;
        std::uint32_t authorityLeaseCount = 0;
        std::uint32_t failedLeaseCount = 0;
    };

    inline HeldBodyActivationSummary activateHeldObjectBodySet(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds)
    {
        HeldBodyActivationSummary summary{};
        if (!world) {
            return summary;
        }

        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (physics_recursive_wrappers::activateBody(world, bodyId)) {
                ++summary.activatedCount;
            } else {
                ++summary.failedActivationCount;
            }
        }
        return summary;
    }

    inline HeldBodyFlagLeaseSummary acquireHeldObjectBodyFlagLeases(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, grab_target::Kind targetKind, std::uintptr_t ownerToken)
    {
        /*
             * Proxy-constraint grab replaced the native held-object action, but the
             * old path owned two different flag contracts: collision flags were
             * leased across the accepted held body set by ROCK, while the native action leased
             * 0x08000000 only on its selected primary body. Keeping that split is
             * important for multipart weapons because secondary collision bodies
             * should participate in the hold without all becoming grab-authority
             * bodies.
             */
        HeldBodyFlagLeaseSummary summary{};
        if (!world || ownerToken == 0) {
            return summary;
        }

        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (havok_runtime::acquireBodyFlagLease(world, bodyId, heldCollisionParticipationFlags(targetKind),
                    kHeldCollisionParticipationFlagMode, ownerToken)) {
                ++summary.collisionLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }

        if (primaryBodyId != INVALID_BODY_ID) {
            if (havok_runtime::acquireBodyFlagLease(world, primaryBodyId, kHeldAuthorityBodyFlags,
                    kHeldAuthorityBodyFlagMode, ownerToken)) {
                ++summary.authorityLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
        return summary;
    }

    inline HeldBodyFlagLeaseSummary releaseHeldObjectBodyFlagLeases(RE::hknpWorld* world, std::uint32_t primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, grab_target::Kind targetKind, std::uintptr_t ownerToken,
        bool restoreOnFinalLease)
    {
        HeldBodyFlagLeaseSummary summary{};
        if (!world || ownerToken == 0) {
            return summary;
        }

        const auto bodyIds = held_object_body_set_policy::makePrimaryFirstUniqueBodyList(primaryBodyId, heldBodyIds);
        summary.bodyCount = static_cast<std::uint32_t>(bodyIds.size());
        for (const auto bodyId : bodyIds) {
            if (havok_runtime::releaseBodyFlagLease(world, bodyId, heldCollisionParticipationFlags(targetKind),
                    kHeldCollisionParticipationFlagMode, ownerToken, restoreOnFinalLease)) {
                ++summary.collisionLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }

        if (primaryBodyId != INVALID_BODY_ID) {
            if (havok_runtime::releaseBodyFlagLease(world, primaryBodyId, kHeldAuthorityBodyFlags,
                    kHeldAuthorityBodyFlagMode, ownerToken, restoreOnFinalLease)) {
                ++summary.authorityLeaseCount;
            } else {
                ++summary.failedLeaseCount;
            }
        }
        return summary;
    }

    struct RuntimePinchPocketCandidate
    {
        GrabSurfaceHit surfaceHit{};
        grab_pinch_pocket_policy::MeshExtentMetrics meshExtents{};
        grab_pinch_pocket_policy::ObjectDecision decision{};
        RE::NiPoint3 thumbPadWorld{};
        RE::NiPoint3 indexPadWorld{};
        RE::NiPoint3 pinchPocketWorld{};
        RE::NiPoint3 pinchAxisWorld{ 1.0f, 0.0f, 0.0f };
        RE::NiPoint3 pinchDetectionDirectionWorld{ 1.0f, 0.0f, 0.0f };
        float thumbIndexGapGameUnits = 0.0f;
        float pocketToSurfaceDistanceGameUnits = std::numeric_limits<float>::infinity();
        grab_pinch_pocket_policy::ClosureSolution closure{};
        bool valid = false;
    };

    inline RE::NiTransform gripEvidenceWorldFrame(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
    {
        if (frame.gripEvidence.gripSourceNode) {
            return frame.gripEvidence.gripSourceNode->world;
        }
        if (frame.gripEvidence.hasGripSourceNodePoint) {
            return frame.gripEvidence.gripSourceNodeWorldAtGrab;
        }
        return fallbackWorld;
    }

    inline RE::NiTransform gripEvidenceWorldFrame(const ImmutableGrabCaptureTelemetry& capture,
        const RE::NiTransform& fallbackWorld)
    {
        if (capture.gripEvidence.gripSourceNode) {
            return capture.gripEvidence.gripSourceNode->world;
        }
        if (capture.gripEvidence.hasGripSourceNodePoint) {
            return capture.gripEvidence.gripSourceNodeWorldAtGrab;
        }
        return fallbackWorld;
    }

    inline RE::NiPoint3 gripEvidencePointWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
    {
        if (frame.gripEvidence.hasGripSourceNodePoint) {
            return transform_math::localPointToWorld(gripEvidenceWorldFrame(frame, fallbackWorld),
                frame.gripEvidence.gripPointSourceNodeLocal);
        }
        return transform_math::localPointToWorld(fallbackWorld, frame.gripEvidence.gripPointLocal);
    }

    inline RE::NiPoint3 gripEvidenceNormalWorld(const CanonicalGrabFrame& frame, const RE::NiTransform& fallbackWorld)
    {
        if (frame.gripEvidence.hasGripSourceNodeNormal) {
            return normalizeOrZero(transform_math::localVectorToWorld(gripEvidenceWorldFrame(frame, fallbackWorld),
                frame.gripEvidence.gripNormalSourceNodeLocal));
        }
        return normalizeOrZero(transform_math::localVectorToWorld(fallbackWorld, frame.gripEvidence.gripNormalLocal));
    }

    inline float translationDeltaGameUnits(const RE::NiTransform& a, const RE::NiTransform& b)
    {
        const RE::NiPoint3 delta = a.translate - b.translate;
        return std::sqrt(delta.x * delta.x + delta.y * delta.y + delta.z * delta.z);
    }

    inline float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
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

    struct GrabCaptureTransformRefreshSample
    {
        const char* role = "unknown";
        RE::NiAVObject* node = nullptr;
        bool validBefore = true;
        bool validAfter = true;
        float positionDeltaGameUnits = 0.0f;
        float rotationDeltaDegrees = 0.0f;
    };

    struct GrabCaptureTransformRefreshResult
    {
        std::array<GrabCaptureTransformRefreshSample, 6> samples{};
        std::uint32_t count = 0;
        bool ok = true;
    };

    inline bool grabCaptureRefreshAlreadyVisited(const GrabCaptureTransformRefreshResult& result,
        const RE::NiAVObject* node)
    {
        for (std::uint32_t i = 0; i < result.count; ++i) {
            if (result.samples[i].node == node) {
                return true;
            }
        }
        return false;
    }

    inline void refreshGrabCaptureNodeTransform(GrabCaptureTransformRefreshResult& result, const char* role,
        RE::NiAVObject* node, bool refreshScene = true)
    {
        if (!node || result.count >= result.samples.size() || grabCaptureRefreshAlreadyVisited(result, node)) {
            return;
        }

        auto& sample = result.samples[result.count++];
        sample.role = role ? role : "unknown";
        sample.node = node;

        const RE::NiTransform before = node->world;
        sample.validBefore = grab_three_phase::isFinite(before);

        // Ragdoll world transforms are the engine's current physics/skin
        // output. Acquisition must sample that pose without recalculating it.
        if (refreshScene) {
            RE::NiUpdateData update{};
            node->UpdateTransforms(update);
        }

        const RE::NiTransform after = node->world;
        sample.validAfter = grab_three_phase::isFinite(after);
        sample.positionDeltaGameUnits = translationDeltaGameUnits(before, after);
        sample.rotationDeltaDegrees = rotationDeltaDegrees(before.rotate, after.rotate);
        result.ok = result.ok && sample.validAfter;
    }

    inline RE::NiMatrix3 matrixFromHkColumns(const float* hkMatrix)
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

    inline RE::NiMatrix3 matrixFromHkRows(const float* hkMatrix)
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

    inline RE::NiTransform rotationOnlyTransform(const RE::NiMatrix3& rotation)
    {
        RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
        result.rotate = rotation;
        return result;
    }

    inline RE::NiTransform makeIdentityTransform() { return transform_math::makeIdentityTransform<RE::NiTransform>(); }

    inline RE::NiTransform reconstructBodyWorldFromProxyInBody(const RE::NiTransform& proxyWorld,
        const RE::NiMatrix3& proxyInBodyRotation, const RE::NiPoint3& transformBLocalGame,
        const RE::NiPoint3& pivotAProxyLocalGame)
    {
        RE::NiTransform proxyInBody = makeIdentityTransform();
        proxyInBody.rotate = proxyInBodyRotation;
        const RE::NiPoint3 rotatedPivotA = transform_math::localVectorToWorld(proxyInBody, pivotAProxyLocalGame);
        proxyInBody.translate = transformBLocalGame - rotatedPivotA;
        const RE::NiTransform bodyInProxy = transform_math::invertTransform(proxyInBody);
        return grab_frame_math::objectFromGeneratedProxyLocalSpace(proxyWorld, bodyInProxy);
    }

    inline RE::NiTransform reconstructSolverEffectiveBodyWorld(const RE::NiTransform& proxyWorld,
        const RE::NiMatrix3& transformARotation, const RE::NiMatrix3& transformBRotation,
        const RE::NiMatrix3& targetBRcaRotation, const RE::NiPoint3& transformBLocalGame,
        const RE::NiPoint3& anchorAWorld, float bodyScale)
    {
        const RE::NiMatrix3 constraintAWorldRotation =
            transform_math::composeTransforms(proxyWorld, rotationOnlyTransform(transformARotation)).rotate;
        const RE::NiMatrix3 desiredConstraintBWorldRotation =
            transform_math::composeTransforms(rotationOnlyTransform(constraintAWorldRotation),
                rotationOnlyTransform(transform_math::transposeRotation(targetBRcaRotation)))
                .rotate;
        const RE::NiMatrix3 desiredBodyRotation =
            transform_math::composeTransforms(rotationOnlyTransform(desiredConstraintBWorldRotation),
                rotationOnlyTransform(transform_math::transposeRotation(transformBRotation)))
                .rotate;

        RE::NiTransform result = makeIdentityTransform();
        result.rotate = desiredBodyRotation;
        result.scale = std::isfinite(bodyScale) && bodyScale > 0.0f ? bodyScale : 1.0f;
        result.translate = anchorAWorld - transform_math::localVectorToWorld(result, transformBLocalGame);
        return result;
    }

    inline physics_recursive_wrappers::MotionPreset motionPresetFromMotionType(
        physics_body_classifier::BodyMotionType motionType, std::uint16_t fallbackMotionPropertiesId)
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

    inline active_grab_body_lifecycle::BodyLifecycleAudit restoreActiveGrabLifecycle(RE::hknpWorld* world,
        const active_grab_body_lifecycle::BodyLifecycleSnapshot& snapshot,
        const active_grab_body_lifecycle::BodyRestorePlan& plan, std::uint32_t primaryBodyId, const char* handName,
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
            physics_recursive_wrappers::setMotionRecursive(ownerNode,
                motionPresetFromMotionType(command.motionType, command.motionPropertiesId), command.recursive,
                command.force, command.activate);
        }

        ROCK_LOG_DEBUG(Hand,
            "{} hand grab lifecycle audit {}: targetKind={} intent={} bodies={} converted={} restoredMotion={} "
            "restoredFilter={} preservedMotion={} preservedFilter={} dampingSnapshots={} inertiaSnapshots={} "
            "latePrepared={} incompleteScan={} primaryBody={}",
            handName ? handName : "?", context ? context : "", grab_target::name(audit.targetKind),
            active_grab_body_lifecycle::releaseIntentName(audit.intent), audit.bodyCount, audit.convertedCount,
            audit.restoredMotionCount, audit.restoredFilterCount, audit.preservedConvertedMotionCount,
            audit.preservedConvertedFilterCount, audit.dampingSnapshotCount, audit.inertiaSnapshotCount,
            audit.latePreparedBodyCount, audit.incompleteNativeScan ? "yes" : "no", audit.primaryBodyId);
        return audit;
    }

    inline bool restoreIncompleteActivePrepRoot(RE::NiAVObject* rootNode, std::uint16_t originalMotionPropsId,
        const char* handName, const char* context)
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
        const bool restored = physics_recursive_wrappers::setMotionRecursive(rootNode,
            motionPresetFromMotionType(motionType, originalMotionPropsId), true, true, false);

        ROCK_LOG_WARN(Hand,
            "{} hand {}: recursive root restore after incomplete object scan root='{}' motionProps={} result={}",
            handName ? handName : "?", context ? context : "incomplete-scan", nodeDebugName(rootNode),
            originalMotionPropsId, restored ? "ok" : "failed");
        return restored;
    }

    inline RE::NiTransform invertTransform(const RE::NiTransform& transform)
    {
        return transform_math::invertTransform(transform);
    }

    inline RE::NiTransform multiplyTransforms(const RE::NiTransform& parent, const RE::NiTransform& child)
    {
        return transform_math::composeTransforms(parent, child);
    }

    inline RE::NiTransform deriveNodeWorldFromBodyWorld(const RE::NiTransform& bodyWorld,
        const RE::NiTransform& bodyLocalTransform)
    {
        return multiplyTransforms(bodyWorld, invertTransform(bodyLocalTransform));
    }

    // Replaces outLocalTriangles' contents, reusing its capacity.
    inline void cacheTrianglesInLocalSpace(const std::vector<TriangleData>& worldTriangles,
        const RE::NiTransform& nodeWorld, std::vector<GrabLocalTriangle>& outLocalTriangles)
    {
        performance_profiler::ScopedTimer stageTimer(performance_profiler::Scope::GrabLocalTriangleCapture);
        outLocalTriangles.clear();
        outLocalTriangles.reserve(worldTriangles.size());
        for (const auto& triangle : worldTriangles) {
            outLocalTriangles.push_back(GrabLocalTriangle{
                transform_math::worldPointToLocal(nodeWorld, triangle.v0),
                transform_math::worldPointToLocal(nodeWorld, triangle.v1),
                transform_math::worldPointToLocal(nodeWorld, triangle.v2),
            });
        }
    }

    inline bool tryGetGrabAuthorityBodyWorldTransform(RE::hknpWorld* world, RE::hknpBodyId bodyId,
        RE::NiTransform& outTransform)
    {
        /*
             * Dynamic grab object-side state is measured from the hknp BODY slot.
             * The hand side is ROCK's hidden no-contact proxy. The held object's
             * contact pivot and visual node relation stay in BODY space; MOTION is
             * COM/weight/diagnostic data only and never grip authority.
             */
        outTransform = makeIdentityTransform();
        return tryGetBodyArrayWorldTransform(world, bodyId, outTransform);
    }

    inline float computeLocalMeshMaxDistanceFromPoint(const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiPoint3& originLocal)
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

    using GrabPivotAuthoritySource = grab_authority_frame_math::GrabAuthorityPivotSource;

    inline float finitePositiveOr(float value, float fallback)
    {
        return std::isfinite(value) && value > 0.0f ? value : fallback;
    }

    using GrabSeatDepthStopResult = grab_palm_seat::DepthResult;

    struct GrabMeshLongAxisResult
    {
        RE::NiPoint3 axisWorld{};
        RE::NiPoint3 secondAxisWorld{};
        float elongationRatio = 0.0f;
        float secondElongationRatio = 0.0f;
        std::uint32_t triangleCount = 0;
        const char* reason = "notEvaluated";
        bool valid = false;
    };

    /*
         * Principal axis of the rendered mesh via area-weighted PCA over the
         * extracted world-space triangles. NIF axes are not authored
         * consistently across props, so long-object orientation must come from
         * the geometry itself. elongationRatio = sqrt(lambda1/lambda2), the RMS
         * extent ratio between the dominant and second axis: ~1 for compact
         * objects, >2 for bottle/broom shapes. secondAxisWorld is the second
         * principal axis and secondElongationRatio = sqrt(lambda2/lambda3)
         * (lambda3 recovered from the covariance trace): ~1 for round cross
         * sections where roll about the long axis is meaningless, high for
         * plank/board shapes with a well-defined flat face. Both axis signs
         * are arbitrary; callers must align to the nearest hemisphere of
         * their target axis. Double accumulators because world coordinates
         * are large; covariance is built about the area-weighted mean.
         */
    inline GrabMeshLongAxisResult computeGrabMeshLongAxis(const std::vector<TriangleData>& worldTriangles)
    {
        GrabMeshLongAxisResult result{};
        result.triangleCount = static_cast<std::uint32_t>(worldTriangles.size());
        if (worldTriangles.empty()) {
            result.reason = "noTriangles";
            return result;
        }

        double weightSum = 0.0;
        double meanAccum[3] = {};
        auto isFinitePoint = [](const RE::NiPoint3& p) { return vector_math::hasFiniteComponents(p); };
        auto triangleArea = [](const TriangleData& tri) {
            const RE::NiPoint3 e0 = tri.v1 - tri.v0;
            const RE::NiPoint3 e1 = tri.v2 - tri.v0;
            const RE::NiPoint3 n = vector_math::cross(e0, e1);
            return 0.5f * std::sqrt((std::max)(0.0f, vector_math::lengthSquared(n)));
        };
        for (const auto& tri : worldTriangles) {
            if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                continue;
            }
            const float area = triangleArea(tri);
            if (!std::isfinite(area) || area <= 0.000001f) {
                continue;
            }
            weightSum += area;
            meanAccum[0] += static_cast<double>(area) * (tri.v0.x + tri.v1.x + tri.v2.x) / 3.0;
            meanAccum[1] += static_cast<double>(area) * (tri.v0.y + tri.v1.y + tri.v2.y) / 3.0;
            meanAccum[2] += static_cast<double>(area) * (tri.v0.z + tri.v1.z + tri.v2.z) / 3.0;
        }
        if (weightSum <= 0.000001) {
            result.reason = "degenerateMeshArea";
            return result;
        }
        const double mean[3] = { meanAccum[0] / weightSum, meanAccum[1] / weightSum, meanAccum[2] / weightSum };

        // Symmetric covariance: [xx, xy, xz, yy, yz, zz]
        double cov[6] = {};
        for (const auto& tri : worldTriangles) {
            if (!isFinitePoint(tri.v0) || !isFinitePoint(tri.v1) || !isFinitePoint(tri.v2)) {
                continue;
            }
            const float area = triangleArea(tri);
            if (!std::isfinite(area) || area <= 0.000001f) {
                continue;
            }
            const double vertexWeight = static_cast<double>(area) / 3.0;
            const RE::NiPoint3* vertices[3] = { &tri.v0, &tri.v1, &tri.v2 };
            for (const auto* vertex : vertices) {
                const double d[3] = { vertex->x - mean[0], vertex->y - mean[1], vertex->z - mean[2] };
                cov[0] += vertexWeight * d[0] * d[0];
                cov[1] += vertexWeight * d[0] * d[1];
                cov[2] += vertexWeight * d[0] * d[2];
                cov[3] += vertexWeight * d[1] * d[1];
                cov[4] += vertexWeight * d[1] * d[2];
                cov[5] += vertexWeight * d[2] * d[2];
            }
        }
        for (double& entry : cov) {
            entry /= weightSum;
        }

        auto covMultiply = [](const double m[6], const double v[3], double out[3]) {
            out[0] = m[0] * v[0] + m[1] * v[1] + m[2] * v[2];
            out[1] = m[1] * v[0] + m[3] * v[1] + m[4] * v[2];
            out[2] = m[2] * v[0] + m[4] * v[1] + m[5] * v[2];
        };
        auto dominantEigen = [&covMultiply](const double m[6], double outAxis[3]) {
            // Deterministic start: the coordinate axis with the largest diagonal.
            double v[3] = {};
            if (m[0] >= m[3] && m[0] >= m[5]) {
                v[0] = 1.0;
            } else if (m[3] >= m[5]) {
                v[1] = 1.0;
            } else {
                v[2] = 1.0;
            }
            for (int iteration = 0; iteration < 48; ++iteration) {
                double next[3];
                covMultiply(m, v, next);
                const double lenSq = next[0] * next[0] + next[1] * next[1] + next[2] * next[2];
                if (!(lenSq > 1e-18)) {
                    break;
                }
                const double invLen = 1.0 / std::sqrt(lenSq);
                v[0] = next[0] * invLen;
                v[1] = next[1] * invLen;
                v[2] = next[2] * invLen;
            }
            double mv[3];
            covMultiply(m, v, mv);
            const double eigenvalue = mv[0] * v[0] + mv[1] * v[1] + mv[2] * v[2];
            outAxis[0] = v[0];
            outAxis[1] = v[1];
            outAxis[2] = v[2];
            return eigenvalue;
        };

        double axis1[3];
        const double lambda1 = dominantEigen(cov, axis1);
        if (!(lambda1 > 1e-9)) {
            result.reason = "degenerateCovariance";
            return result;
        }
        double deflated[6] = {
            cov[0] - lambda1 * axis1[0] * axis1[0],
            cov[1] - lambda1 * axis1[0] * axis1[1],
            cov[2] - lambda1 * axis1[0] * axis1[2],
            cov[3] - lambda1 * axis1[1] * axis1[1],
            cov[4] - lambda1 * axis1[1] * axis1[2],
            cov[5] - lambda1 * axis1[2] * axis1[2],
        };
        double axis2[3];
        const double lambda2 = (std::max)(0.0, dominantEigen(deflated, axis2));

        result.axisWorld = RE::NiPoint3{
            static_cast<float>(axis1[0]),
            static_cast<float>(axis1[1]),
            static_cast<float>(axis1[2]),
        };
        result.elongationRatio =
            static_cast<float>((std::min)(100.0, std::sqrt(lambda1 / (std::max)(lambda2, lambda1 * 1e-4))));
        if (lambda2 > 1e-9) {
            // Re-orthogonalize against axis1: deflation leaves numerical drift.
            const double axis12dot = axis2[0] * axis1[0] + axis2[1] * axis1[1] + axis2[2] * axis1[2];
            double axis2Ortho[3] = {
                axis2[0] - axis12dot * axis1[0],
                axis2[1] - axis12dot * axis1[1],
                axis2[2] - axis12dot * axis1[2],
            };
            const double axis2LenSq =
                axis2Ortho[0] * axis2Ortho[0] + axis2Ortho[1] * axis2Ortho[1] + axis2Ortho[2] * axis2Ortho[2];
            if (axis2LenSq > 1e-12) {
                const double invLen = 1.0 / std::sqrt(axis2LenSq);
                result.secondAxisWorld = RE::NiPoint3{
                    static_cast<float>(axis2Ortho[0] * invLen),
                    static_cast<float>(axis2Ortho[1] * invLen),
                    static_cast<float>(axis2Ortho[2] * invLen),
                };
                // Eigenvalues of a symmetric matrix sum to its trace, so
                // lambda3 needs no third power iteration.
                const double trace = cov[0] + cov[3] + cov[5];
                const double lambda3 = (std::max)(0.0, trace - lambda1 - lambda2);
                result.secondElongationRatio =
                    static_cast<float>((std::min)(100.0, std::sqrt(lambda2 / (std::max)(lambda3, lambda2 * 1e-4))));
            }
        }
        result.reason = "meshPrincipalAxis";
        result.valid = true;
        return result;
    }

    inline void setHeldVelocity(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, const RE::NiPoint3& linearVelocity,
        const RE::NiPoint3& angularVelocity, bool overrideAngularVelocity, float angularVelocityKeep = 1.0f,
        bool includeConnectedLinearVelocity = true, bool includeConnectedAngularVelocity = true)
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

        auto setBody = [&](std::uint32_t bodyId, bool primaryBody) {
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
            const bool applyLinearForBody = primaryBody || includeConnectedLinearVelocity;
            const float angularKeep =
                std::clamp(std::isfinite(angularVelocityKeep) ? angularVelocityKeep : 1.0f, 0.0f, 1.0f);
            const bool overrideAngularForBody =
                overrideAngularVelocity && (primaryBody || includeConnectedAngularVelocity);
            const RE::hkVector4f linearHavok = applyLinearForBody
                ? RE::hkVector4f{ linearVelocity.x, linearVelocity.y, linearVelocity.z, 0.0f }
                : RE::hkVector4f{ motion->linearVelocity.x, motion->linearVelocity.y, motion->linearVelocity.z, 0.0f };
            const RE::hkVector4f angularHavok = overrideAngularForBody
                ? RE::hkVector4f{ angularVelocity.x, angularVelocity.y, angularVelocity.z, 0.0f }
                : RE::hkVector4f{ motion->angularVelocity.x * angularKeep, motion->angularVelocity.y * angularKeep,
                      motion->angularVelocity.z * angularKeep, 0.0f };
            havok_runtime::setBodyVelocityDeferred(world, bodyId, linearHavok, angularHavok);
        };

        setBody(primaryBodyId.value, true);
        if (includeConnectedLinearVelocity || includeConnectedAngularVelocity) {
            for (const auto bodyId : heldBodyIds) {
                setBody(bodyId, false);
            }
        }
    }

    inline float readBodyMass(RE::hknpWorld* world, RE::hknpBodyId bodyId)
    {
        if (!world || bodyId.value == INVALID_BODY_ID) {
            return 0.0f;
        }

        auto* motion = havok_runtime::getBodyMotion(world, bodyId);
        if (!motion) {
            return 0.0f;
        }

        const auto packedInvMass = static_cast<std::int16_t>(motion->packedInverseInertia[3]);
        if (packedInvMass == 0) {
            return 0.0f;
        }

        return grab_mass_policy::massFromInverseMass(unpackBfloat16(packedInvMass));
    }

    struct HeldBodyMassSummary
    {
        float primaryMass = 0.0f;
        float aggregateMass = 0.0f;
        std::uint32_t sampledBodies = 0;
        std::uint32_t uniqueMotions = 0;

        [[nodiscard]] float motorMass() const noexcept
        {
            if (std::isfinite(aggregateMass) && aggregateMass > 0.0f) {
                return aggregateMass;
            }
            return (std::isfinite(primaryMass) && primaryMass > 0.0f) ? primaryMass : 0.0f;
        }
    };

    inline HeldBodyMassSummary readHeldBodyMassSummary(RE::hknpWorld* world, RE::hknpBodyId primaryBodyId,
        const std::vector<std::uint32_t>& heldBodyIds, bool includeConnectedBodies = true)
    {
        /*
             * Dynamic grab owns one held object even when FO4VR exposes that
             * object as several hknp bodies. Lifecycle, inertia normalization,
             * release velocity, and nearby damping already operate on the whole
             * accepted body set. The motor mass budget must use the same object
             * scope, with unique-motion dedupe, so multipart loose weapons are
             * not budgeted from whichever child body happened to be selected.
             */
        HeldBodyMassSummary summary{};
        summary.primaryMass = readBodyMass(world, primaryBodyId);
        if (!world) {
            summary.aggregateMass = summary.primaryMass;
            return summary;
        }

        constexpr std::size_t kMaxMassMotionSlots = 96;
        std::array<std::uint32_t, kMaxMassMotionSlots> sampledMotionSlots{};
        std::size_t sampledMotionSlotCount = 0;

        auto motionAlreadySampled = [&sampledMotionSlots, &sampledMotionSlotCount](std::uint32_t motionIndex) {
            for (std::size_t i = 0; i < sampledMotionSlotCount; ++i) {
                if (sampledMotionSlots[i] == motionIndex) {
                    return true;
                }
            }
            return false;
        };

        auto sampleBody = [&](std::uint32_t rawBodyId) {
            if (rawBodyId == INVALID_BODY_ID) {
                return;
            }

            auto* body = havok_runtime::getBody(world, RE::hknpBodyId{ rawBodyId });
            if (!body || !body_frame::hasUsableMotionIndex(body->motionIndex) ||
                motionAlreadySampled(body->motionIndex)) {
                return;
            }
            if (sampledMotionSlotCount >= sampledMotionSlots.size()) {
                return;
            }

            const float mass = readBodyMass(world, RE::hknpBodyId{ rawBodyId });
            if (!std::isfinite(mass) || mass <= 0.0f) {
                return;
            }

            sampledMotionSlots[sampledMotionSlotCount++] = body->motionIndex;
            summary.aggregateMass += mass;
            ++summary.sampledBodies;
            summary.uniqueMotions = static_cast<std::uint32_t>(sampledMotionSlotCount);
        };

        sampleBody(primaryBodyId.value);
        if (includeConnectedBodies) {
            for (const auto bodyId : heldBodyIds) {
                sampleBody(bodyId);
            }
        }

        if (!(std::isfinite(summary.aggregateMass) && summary.aggregateMass > 0.0f)) {
            summary.aggregateMass = summary.primaryMass;
        }
        return summary;
    }

    inline held_object_contact_policy::HeldContactOtherMotion classifyHeldContactOtherMotion(RE::hknpWorld* world,
        std::uint32_t bodyId)
    {
        if (!world || bodyId == INVALID_BODY_ID) {
            return held_object_contact_policy::HeldContactOtherMotion::Unknown;
        }

        auto* motion = havok_runtime::getBodyMotion(world, RE::hknpBodyId{ bodyId });
        if (!motion) {
            return held_object_contact_policy::HeldContactOtherMotion::Unknown;
        }

        const float mass = readBodyMass(world, RE::hknpBodyId{ bodyId });
        return mass > 0.0f ? held_object_contact_policy::HeldContactOtherMotion::Dynamic
                           : held_object_contact_policy::HeldContactOtherMotion::FixedOrStatic;
    }

    /*
         * Publishes a mesh finger pose as FRIK joint values. Mesh poses carry no
         * per-bone local transforms, so any exact-local pose still held under
         * this tag (a saved offset) is cleared first.
         */
    inline void applyRockGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        std::array<float, 15>& currentJointPose, bool& hasCurrentJointPose,
        std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask,
        bool& hasCurrentLocalTransforms, float deltaTime, bool snapJointPoseToTarget = false)
    {
        if (!frik_visual_authority::isAvailable()) {
            return;
        }

        const auto hand = handFromBool(isLeft);
        grab_finger_local_transform_runtime::State localTransformState{
            .currentTransforms = currentLocalTransforms,
            .currentMask = currentLocalTransformMask,
            .hasCurrentTransforms = hasCurrentLocalTransforms,
        };
        grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
        currentLocalTransforms = localTransformState.currentTransforms;
        currentLocalTransformMask = localTransformState.currentMask;
        hasCurrentLocalTransforms = localTransformState.hasCurrentTransforms;

        if (!g_rockConfig.rockGrabMeshFingerPoseEnabled) {
            hasCurrentJointPose = false;
            (void)frik_visual_authority::clearHandPose("ROCK_Grab", hand);
            return;
        }

        if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved) {
            const auto targetJointPose = fingerPose.hasJointValues
                ? fingerPose.jointValues
                : grab_finger_pose_math::expandFingerCurlsToJointValues(fingerPose.values);
            if (!hasCurrentJointPose || snapJointPoseToTarget) {
                currentJointPose = targetJointPose;
                hasCurrentJointPose = true;
            } else {
                currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose,
                    g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
            }
            if (!frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand,
                    frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose), 100)) {
                hasCurrentJointPose = false;
            }
            return;
        }

        hasCurrentJointPose = false;
        if (fingerPose.solved) {
            const auto handPose = frik_visual_authority::makeUniformHandPoseData(fingerPose.values[0],
                fingerPose.values[1], fingerPose.values[2], fingerPose.values[3], fingerPose.values[4]);
            (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, handPose, 100);
            return;
        }

        const float fallbackMin =
            std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f,
                0.0f, 1.0f);
        const float configuredFallback = std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue)
            ? g_rockConfig.rockSelectedCloseFingerAnimValue
            : 0.9f;
        const float fallbackValue = std::clamp(configuredFallback, fallbackMin, 1.0f);
        const auto fallbackPose = frik_visual_authority::makeUniformHandPoseData(fallbackValue, fallbackValue,
            fallbackValue, fallbackValue, fallbackValue);
        (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, fallbackPose, 100);
    }

    /*
         * FingerPoses.LooseGrabFingerPoseMode = 1: the pre-capsule publication for a swept-arc
         * mesh pose (contact splay, then the surface-aim local-transform layer when
         * publishLocalTransforms is set), unchanged.
         */
    inline void applySweptArcGrabHandPose(bool isLeft, const grab_finger_pose_runtime::SolvedGrabFingerPose& fingerPose,
        std::array<float, 15>& currentJointPose, bool& hasCurrentJointPose,
        std::array<RE::NiTransform, 15>& currentLocalTransforms, std::uint16_t& currentLocalTransformMask,
        bool& hasCurrentLocalTransforms, float deltaTime, bool publishLocalTransforms = true,
        bool snapJointPoseToTarget = false)
    {
        if (!frik_visual_authority::isAvailable()) {
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
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100,
                localTransformState);
            syncLocalTransformState();
            (void)frik_visual_authority::clearHandPose("ROCK_Grab", hand);
            return;
        }

        if (g_rockConfig.rockGrabMeshJointPoseEnabled && fingerPose.solved) {
            const auto targetJointPose = fingerPose.hasJointValues
                ? fingerPose.jointValues
                : grab_finger_pose_math::expandFingerCurlsToJointValues(fingerPose.values);
            if (!hasCurrentJointPose || snapJointPoseToTarget) {
                currentJointPose = targetJointPose;
                hasCurrentJointPose = true;
            } else {
                currentJointPose = grab_finger_pose_math::advanceJointValues(currentJointPose, targetJointPose,
                    g_rockConfig.rockGrabFingerPoseSmoothingSpeed, deltaTime);
            }

            std::array<float, 5> currentSplayRadians{};
            (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
            const auto currentHandPose =
                frik_visual_authority::makeHandPoseDataFromJointValues(currentJointPose, currentSplayRadians);
            if (!frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, currentHandPose, 100)) {
                hasCurrentJointPose = false;
                grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100,
                    localTransformState);
                syncLocalTransformState();
                return;
            }
            (void)grab_finger_local_transform_runtime::publishLocalTransformPose("ROCK_Grab", hand, isLeft, fingerPose,
                currentHandPose,
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
                deltaTime, 100, localTransformState);
            syncLocalTransformState();
            return;
        }

        if (fingerPose.solved) {
            hasCurrentJointPose = false;
            grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100,
                localTransformState);
            syncLocalTransformState();
            std::array<float, 5> currentSplayRadians{};
            (void)grab_finger_pose_runtime::resolveSurfaceContactSplayValues(isLeft, fingerPose, currentSplayRadians);
            const auto handPose =
                frik_visual_authority::makeUniformHandPoseData(fingerPose.values[0], fingerPose.values[1],
                    fingerPose.values[2], fingerPose.values[3], fingerPose.values[4], currentSplayRadians);
            (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, handPose, 100);
            return;
        }

        hasCurrentJointPose = false;
        grab_finger_local_transform_runtime::clearLocalTransformOverride("ROCK_Grab", hand, 100, localTransformState);
        syncLocalTransformState();
        const float fallbackMin =
            std::clamp(std::isfinite(g_rockConfig.rockGrabFingerMinValue) ? g_rockConfig.rockGrabFingerMinValue : 0.2f,
                0.0f, 1.0f);
        const float configuredFallback = std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue)
            ? g_rockConfig.rockSelectedCloseFingerAnimValue
            : 0.9f;
        const float fallbackValue = std::clamp(configuredFallback, fallbackMin, 1.0f);
        const auto fallbackPose = frik_visual_authority::makeUniformHandPoseData(fallbackValue, fallbackValue,
            fallbackValue, fallbackValue, fallbackValue);
        (void)frik_visual_authority::setHandPoseCustom("ROCK_Grab", hand, fallbackPose, 100);
    }

    inline grab_finger_pose_runtime::SolvedGrabFingerPose buildAcquisitionFingerPose(
        const grab_finger_pose_runtime::SolvedGrabFingerPose& targetPose, float progress)
    {
        auto pose = targetPose;
        const float t = std::clamp(std::isfinite(progress) ? progress : 0.0f, 0.0f, 1.0f);
        const float precloseValue = std::clamp(std::isfinite(g_rockConfig.rockSelectedCloseFingerAnimValue)
                ? g_rockConfig.rockSelectedCloseFingerAnimValue
                : 0.9f,
            0.0f, 1.0f);

        for (std::size_t finger = 0; finger < pose.values.size(); ++finger) {
            const float solvedValue =
                targetPose.solved ? std::clamp(targetPose.values[finger], 0.0f, 1.0f) : precloseValue;
            const float closingTarget = (std::min)(precloseValue, solvedValue);
            pose.values[finger] = precloseValue + (closingTarget - precloseValue) * t;
        }

        if (targetPose.solved && targetPose.hasJointValues) {
            std::array<float, 5> precloseValues{ precloseValue, precloseValue, precloseValue, precloseValue,
                precloseValue };
            const auto precloseJoints = grab_finger_pose_math::expandFingerCurlsToJointValues(precloseValues);
            for (std::size_t joint = 0; joint < pose.jointValues.size(); ++joint) {
                const float targetJoint = std::clamp(targetPose.jointValues[joint], 0.0f, 1.0f);
                pose.jointValues[joint] = precloseJoints[joint] + (targetJoint - precloseJoints[joint]) * t;
            }
        } else {
            pose.jointValues = grab_finger_pose_math::expandFingerCurlsToJointValues(pose.values);
        }
        pose.solved = true;
        pose.hasJointValues = true;
        return pose;
    }

    inline constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";

    inline void clearGrabExternalHandWorldTransform(bool isLeft)
    {
        (void)frik_visual_authority::clearHandWorld(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
    }

    inline bool isUsableGrabVisualTransform(const RE::NiTransform& transform)
    {
        if (!std::isfinite(transform.translate.x) || !std::isfinite(transform.translate.y) ||
            !std::isfinite(transform.translate.z) || !std::isfinite(transform.scale) ||
            std::abs(transform.scale) <= 0.0001f) {
            return false;
        }
        for (int row = 0; row < 3; ++row) {
            for (int column = 0; column < 3; ++column) {
                if (!std::isfinite(transform.rotate.entry[row][column])) {
                    return false;
                }
            }
        }
        return true;
    }

    struct GrabPalmBasisDelta
    {
        float rotationDegrees = -1.0f;
        float xAxisDegrees = -1.0f;
        float yAxisDegrees = -1.0f;
        float zAxisDegrees = -1.0f;
        float rawDeterminant = 0.0f;
        float proxyDeterminant = 0.0f;
    };

    inline void nativeVRGrabDrop(void* playerChar, int handIndex)
    {
        typedef void func_t(void*, int, std::uint64_t);
        static REL::Relocation<func_t> func{ REL::Offset(offsets::kFunc_NativeVRGrabDrop) };
        func(playerChar, handIndex, 0);
    }
}

namespace rock
{
    // Every grab implementation file uses these helpers unqualified.
    using namespace hand_grab_internal;

    struct Hand::ValidatedGrabSelection
    {
        RE::NiPointer<RE::TESObjectREFR> retainedRef;
        RE::hknpBodyId bodyId{};
        RE::NiAVObject* rootNode = nullptr;
        RE::bhkWorld* bhkWorld = nullptr;
        RE::TESBoundObject* baseObject = nullptr;
        std::string objectName{ "(unnamed)" };
        bool joiningPeerHeldObject = false;
        bool grabbedFromPullCatch = false;
        bool looseWeaponGrab = false;
        bool handPocketOnlyGrab = false;
    };

    struct Hand::GrabBodyPreparation
    {
        object_physics_body_set::BodySetScanOptions scanOptions{};
        active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle{};
        object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet{};
        object_physics_body_set::ObjectPhysicsBodySet preparedBodySet{};
        bool consumedPullPrepLifecycle = false;
        bool beforePrepScanCacheHit = false;
        bool motionConverted = true;
        bool collisionEnabled = true;
        bool preparedScanCacheHit = false;
        bool preparedBodySetPostPrepComplete = false;
    };

    struct Hand::GrabProxyPreparation
    {
        RE::NiTransform handBodyWorldAtGrab{};
        RE::NiTransform proxyFrameWorldAtGrab{};
        RE::NiTransform proxyAuthorityFrameWorldAtGrab{};
        RE::NiPoint3 grabAuthorityPivotAWorld{};
        RE::NiPoint3 palmPocketPivotAWorld{};
        RE::NiPoint3 grabPivotAForPrimaryChoice{};
        GrabPalmBasisDelta palmBasisDelta{};
        const char* proxyFrameSourceAtGrab = "unresolved";
        float palmPocketToProxyDeltaGameUnits = 0.0f;
        bool hasPalmProxyFrameAtGrab = false;
    };

    struct Hand::GrabMeshCaptureSetup
    {
        RE::NiAVObject* collidableNode = nullptr;
        RE::NiAVObject* meshSourceNode = nullptr;
        RE::NiTransform objectWorldTransform{};
    };

    struct Hand::GrabSurfaceEvidence
    {
        RE::NiPoint3 gripPoint{};
        float selectionToMeshDistanceGameUnits = 0.0f;
        bool meshGrabFound = false;
        GrabSurfaceHit surfaceHit{};
        RE::NiAVObject* surfaceOwnerNode = nullptr;
        const char* pointMode = "noContactPointPending";
        GrabPivotAuthoritySource pointAuthoritySource = GrabPivotAuthoritySource::None;
        const char* fallbackReason = "noMeshSourceNode";
    };

    struct Hand::GrabBodyResolution
    {
        object_physics_body_set::PrimaryBodyChoice primaryChoice{};
        mechanical_connected_body_set::MechanicalScope mechanicalScope{};
        RE::NiPoint3 primaryChoiceTarget{};
        bool surfaceOwnerMatchesResolvedBody = true;
        bool relaxedArticulatedAuthority = false;
    };

    struct Hand::ResolvedGrabBodyCapture
    {
        RE::hknpBodyId bodyId{};
        RE::NiAVObject* collidableNode = nullptr;
        RE::NiTransform objectWorldTransform{};
    };

    struct Hand::GrabPivotEvidence
    {
        hand_semantic_contact_state::SemanticContactCollection semanticContacts{};
        GrabPivotAuthoritySource authoritySource = GrabPivotAuthoritySource::None;
        bool authorityNormalTrusted = false;
        bool authorityPositionOnly = false;
        float authorityPositionConfidence = 0.0f;
        float authorityPocketDistanceGameUnits = std::numeric_limits<float>::max();
        float authoritySelectionDeltaGameUnits = std::numeric_limits<float>::max();
        float authorityLongLeverGameUnits = 0.0f;
    };

    struct Hand::GrabCommitPreparationInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        RE::NiAVObject* collidableNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        const std::string* objectName = nullptr;
        const mechanical_connected_body_set::MechanicalScope* mechanicalScope = nullptr;
        const GrabSharedObjectContext* sharedContext = nullptr;
        const object_physics_body_set::ObjectPhysicsBodySet* preparedBodySet = nullptr;
        bool joiningPeerHeldObject = false;
    };

    struct Hand::GrabBodyFrameCaptureInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        GrabSurfaceEvidence* surface = nullptr;
        const GrabPivotEvidence* pivotEvidence = nullptr;
        const GrabBodyResolution* bodyResolution = nullptr;
        const std::vector<GrabLocalTriangle>* localMeshTriangles = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        std::uint64_t traceId = 0;
        const std::string* objectName = nullptr;
        bool authoredSeatPrepared = false;
    };

    struct Hand::GrabBodyFrameCapture
    {
        RE::NiPoint3 palmPosition{};
        RE::NiTransform grabBodyWorld{};
        RE::NiTransform motionBodyWorld{};
        body_frame::BodyFrameSource motionBodySource = body_frame::BodyFrameSource::Fallback;
        bool hasMotionBodyWorld = false;
        bool constraintUsesMotionBody = false;
        RE::NiTransform constraintBodyWorld{};
        RE::NiPoint3 grabPivotAWorld{};
        RE::bhkNPCollisionObject* bodyCollisionObject = nullptr;
        RE::NiAVObject* ownerNode = nullptr;
        RE::NiTransform objectToBody{};
        RE::NiTransform ownerBodyLocal{};
        RE::NiTransform rootBodyLocal{};
        RE::NiPoint3 selectedGripPointLocal{};
        RE::NiPoint3 selectedPivotBBodyLocalGame{};
        std::vector<GrabLocalTriangle> fingerPoseLocalMeshTriangles{};
        bool clearExternalOnFailure = false;
    };

    struct Hand::GrabSeatCaptureInput
    {
        // The raw hand frame the commit freezes rawHandSpace against.
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        GrabSurfaceEvidence* surface = nullptr;
        GrabPivotEvidence* pivotEvidence = nullptr;
        GrabBodyFrameCapture* bodyFrame = nullptr;
        const RuntimePinchPocketCandidate* pinchPocketCandidate = nullptr;
        saved_grab_offset::HandOffset* savedGrabOffset = nullptr;
        bool* hasSavedGrabOffset = nullptr;
        RE::NiAVObject* rootNode = nullptr;
        RE::NiAVObject* collidableNode = nullptr;
        RE::hknpBodyId objectBodyId{};
        bool grabbedFromPullCatch = false;
        bool looseWeaponGrab = false;
        bool joiningPeerHeldObject = false;
        const AuthoredWeaponGripPose* transferPose = nullptr;
        const loose_weapon_grip_zone::NearGrab* preparedAuthoredGrip = nullptr;
        RE::NiPoint3 canonicalPivotNormalWorld{};
        GrabRollbackAction rollback{};
    };

    struct Hand::GrabSeatCaptureResult
    {
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform authoredHandWeaponLocal{};
        bool hasAuthoredHandRelation = false;
        GrabPivotAuthoritySource resolvedAuthoritySource = GrabPivotAuthoritySource::None;
        const char* resolvedAuthorityReason = "notResolved";
    };

    struct Hand::GrabFrozenCommitInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabBodyFrameCapture* bodyFrame = nullptr;
        GrabSeatCaptureResult* seatCapture = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::hknpBodyId objectBodyId{};
        std::uint64_t traceId = 0;
        GrabRollbackAction rollback{};
    };

    struct Hand::GrabPostFreezeInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabMeshCaptureSetup* meshCapture = nullptr;
        const GrabMeshExtraction* mesh = nullptr;
        const GrabBodyFrameCapture* bodyFrame = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::hknpBodyId objectBodyId{};
        const GrabSharedObjectContext* sharedContext = nullptr;
        bool joiningPeerHeldObject = false;
        bool looseWeaponGrab = false;
    };

    struct Hand::GrabConstraintCommitInput
    {
        const RE::NiTransform* handWorldTransform = nullptr;
        const GrabProxyPreparation* proxy = nullptr;
        const GrabSurfaceEvidence* surface = nullptr;
        RE::bhkWorld* bhkWorld = nullptr;
        RE::hknpBodyId objectBodyId{};
        active_grab_body_lifecycle::BodyLifecycleSnapshot* activeLifecycle = nullptr;
        const saved_grab_offset::HandOffset* savedGrabOffset = nullptr;
        bool hasSavedGrabOffset = false;
        bool joiningPeerHeldObject = false;
        bool looseWeaponGrab = false;
        float tau = 0.0f;
        float damping = 0.0f;
        float maxForce = 0.0f;
        float proportionalRecovery = 0.0f;
        float constantRecovery = 0.0f;
        GrabRollbackAction rollback{};
    };

    struct Hand::HeldDriveUpdate
    {
        RE::NiTransform proxyAuthorityWorld{};
        RE::NiTransform desiredObjectWorld{};
        RE::NiTransform desiredBodyWorld{};
        RE::NiTransform solvedBodyWorld{};
        /*
         * BODY-frame pose the scene writer presents for the held node this
         * frame (current target composed with the previous solved residual).
         * Valid only when the target transport accepted this frame's sample.
         */
        RE::NiTransform presentedBodyWorld{};
        RE::NiPoint3 activePivotBBodyLocalGame{};
        RE::NiPoint3 desiredTargetPointWorld{};
        RE::NiPoint3 liveGripWorldForAuthority{};
        const char* proxyAuthoritySource = "notProxy";
        const char* heldMotorContactReason = "no-recent-contact";
        float pivotTrackingErrorGameUnits = 0.0f;
        float grabRotationErrorDegrees = 0.0f;
        float authorityForceScale = 1.0f;
        float averageGrabDeviationGameUnits = 0.0f;
        bool hasProxyAuthorityFrame = false;
        bool hasPivotTrackingError = false;
        bool hasPresentedBodyWorld = false;
        bool heldBodyColliding = false;
        bool heldMotorContactSoftening = false;
    };
}

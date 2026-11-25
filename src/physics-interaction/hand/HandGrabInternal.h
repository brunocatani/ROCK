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
#include "physics-interaction/grab/HeldScenePresentation.h"

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
        case PrimaryBodyChoiceReason::NearestAccepted:
            return "nearestAccepted";
        case PrimaryBodyChoiceReason::NoAcceptedBody:
            return "noAcceptedBody";
        case PrimaryBodyChoiceReason::None:
        default:
            return "none";
        }
    }

    inline float pointDistanceGameUnits(const RE::NiPoint3& a, const RE::NiPoint3& b)
    {
        return vector_math::distance(a, b);
    }

    inline float vectorMagnitude(const RE::NiPoint3& value) { return vector_math::length(value); }

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
        return grab_motion_controller::effectiveMotorMass(mass, true, g_rockConfig.rockGrabEffectiveMotorMassFloor);
    }

    // Stricter than transform_math::isInvertibleTransform: a negative (mirrored) scale is refused too.
    inline bool isInvertiblePositiveScaleTransform(const RE::NiTransform& value)
    {
        return transform_math::isInvertibleTransform(value) && value.scale > 0.0f;
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
        float pinchSpanGameUnits = std::numeric_limits<float>::infinity();
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
        return vector_math::distance(a.translate, b.translate);
    }

    inline float rotationDeltaDegrees(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
    {
        return transform_math::rotationDeltaDegrees(a, b);
    }

    // The nodes one grab capture has refreshed, each once, and whether every
    // refreshed world transform was finite.
    struct GrabCaptureTransformRefreshResult
    {
        std::array<RE::NiAVObject*, 6> nodes{};
        std::uint32_t count = 0;
        bool ok = true;
    };

    inline bool grabCaptureRefreshAlreadyVisited(const GrabCaptureTransformRefreshResult& result,
        const RE::NiAVObject* node)
    {
        for (std::uint32_t i = 0; i < result.count; ++i) {
            if (result.nodes[i] == node) {
                return true;
            }
        }
        return false;
    }

    inline void refreshGrabCaptureNodeTransform(GrabCaptureTransformRefreshResult& result, RE::NiAVObject* node,
        bool refreshScene = true)
    {
        if (!node || result.count >= result.nodes.size() || grabCaptureRefreshAlreadyVisited(result, node)) {
            return;
        }

        result.nodes[result.count++] = node;

        // Ragdoll world transforms are the engine's current physics/skin
        // output. Acquisition must sample that pose without recalculating it.
        if (refreshScene) {
            RE::NiUpdateData update{};
            node->UpdateTransforms(update);
        }

        result.ok = result.ok && grab_three_phase::isFinite(node->world);
    }

    inline RE::NiTransform rotationOnlyTransform(const RE::NiMatrix3& rotation)
    {
        RE::NiTransform result = transform_math::makeIdentityTransform<RE::NiTransform>();
        result.rotate = rotation;
        return result;
    }

    inline RE::NiTransform makeIdentityTransform() { return transform_math::makeIdentityTransform<RE::NiTransform>(); }

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

    using GrabSeatDepthStopResult = grab_palm_seat::DepthResult;

    inline constexpr const char* GRAB_EXTERNAL_HAND_TAG = "ROCK_GrabVisual";

    inline void clearGrabExternalHandWorldTransform(bool isLeft)
    {
        (void)frik_visual_authority::clearHandWorld(GRAB_EXTERNAL_HAND_TAG, handFromBool(isLeft));
    }

    inline constexpr const char* GRAB_HAND_POSE_TAG = "ROCK_Grab";

    // FRIK's primary weapon hand pose block a loose weapon's authored grab pose takes.
    [[nodiscard]] inline const char* grabPrimaryPoseBlockTag(bool isLeft)
    {
        return isLeft ? "ROCK_GrabPrimaryPoseLeft" : "ROCK_GrabPrimaryPoseRight";
    }

    /*
     * Ends the grab's finger pose and the primary weapon hand pose block that
     * goes with it. Every end of a grab clears both: a block left behind keeps
     * FRIK's weapon hand pose off that hand, also in FRIK's weapon adjustment.
     */
    inline void clearGrabHandPose(bool isLeft)
    {
        (void)frik_visual_authority::clearHandPose(GRAB_HAND_POSE_TAG, handFromBool(isLeft));
        (void)frik_visual_authority::blockPrimaryHandWeaponPose(grabPrimaryPoseBlockTag(isLeft), false);
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
        // A close grab no finger probe touches: the object comes to the hand
        // and seats like a pull that has just arrived (grabSelectedObject).
        bool reachArrival = false;
        // A close grab no finger probe touches, of an actor's limb, a corpse
        // or a severed part (never another object): the grab takes the mesh point
        // nearest the palm within close reach and the hold closes the gap.
        bool handReach = false;
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
        const char* gripReason = "noMeshSourceNode";
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

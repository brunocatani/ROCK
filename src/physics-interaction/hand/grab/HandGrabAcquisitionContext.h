#pragma once

/*
 * GrabAcquisitionContext: the state one grab acquisition carries between its
 * phases. It exists so the phase methods can take ONE parameter instead of the
 * fifty locals the acquisition monolith used to declare up front.
 *
 * The fields are grouped in RUN order, and each group is written by the phase
 * named in its comment and read by the phases after it. A field written by two
 * phases is a coupling worth a comment at both writers.
 *
 * Three TUs share it: HandGrabAcquire.cpp (guards and the resolve phases),
 * HandGrabSeatCapture.cpp (the seat solve), HandGrabCommit.cpp (drive commit and
 * finger-pose publish). Hand.h forward-declares the type; only these TUs need
 * the definition.
 */

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/hand/grab/HandGrabInternal.h"
#include "physics-interaction/hand/grab/HandGrabMath.h"
#include "physics-interaction/hand/grab/HandGrabTrace.h"
#include "physics-interaction/hand/grab/HandGrabVisualDetail.h"
#include "physics-interaction/hand/grab/HandGrabContactEvidence.h"
#include "physics-interaction/hand/grab/HandGrabFingerPose.h"
#include "physics-interaction/hand/grab/HandGrabSupportModel.h"
#include "physics-interaction/hand/grab/HandGrabOffsetSources.h"
#include "physics-interaction/hand/grab/HandGrabBodySetRuntime.h"
#include "physics-interaction/hand/grab/HandGrabPivotAuthority.h"
#include "physics-interaction/body/BodyBoneColliderSet.h"
#include "physics-interaction/native/havok/HavokOffsets.h"
#include "physics-interaction/native/BodyCollisionControl.h"
#include "physics-interaction/native/CharacterControllerRuntime.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/collision/CollisionSuppressionRegistry.h"
#include "physics-interaction/debug/DebugMath.h"
#include "physics-interaction/debug/GrabClockDebugFeed.h"
#include "physics-interaction/hand/HeldBodyRenderPose.h"
#include "physics-interaction/native/SceneWriterProbe.h"
#include "physics-interaction/grenade/LooseGrenadeRuntime.h"
#include "physics-interaction/grab/GrabAuthorityProxy.h"
#include "physics-interaction/grab/GrabConstraint.h"
#include "physics-interaction/grab/GrabConstraintMath.h"
#include "physics-interaction/grab/GrabContact.h"
#include "physics-interaction/grab/GrabCore.h"
#include "physics-interaction/grab/saved/SavedGrabOffsetStore.h"
#include "physics-interaction/grab/GrabFinger.h"
#include "physics-interaction/grab/GrabMassPolicy.h"
#include "physics-interaction/grab/GrabMotionController.h"
#include "physics-interaction/grab/GrabNodeInfoMath.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/grab/GrabHeldObject.h"
#include "physics-interaction/grab/MeshGrab.h"
#include "physics-interaction/object/MechanicalConnectedBodySet.h"
#include "physics-interaction/object/CarInteractionPolicy.h"
#include "physics-interaction/object/ObjectPhysicsBodySet.h"
#include "physics-interaction/weapon/authored_grip/AuthoredWeaponGripLibrary.h"
#include "physics-interaction/weapon/LooseWeaponGripZone.h"
#include "physics-interaction/weapon/WeaponTypePolicy.h"
#include "physics-interaction/object/SkinnedBodyResolver.h"
#include "physics-interaction/performance/PerformanceProfiler.h"
#include "physics-interaction/visual/FrikVisualAuthorityBridge.h"
#include "physics-interaction/visual/PreFrikHandAuthorityPolicy.h"
#include "physics-interaction/hand/skeleton/HandFrame.h"
#include "physics-interaction/hand/HandVisual.h"
#include "physics-interaction/core/PhysicsHooks.h"
#include "physics-interaction/core/RockRuntimeState.h"
#include "physics-interaction/PhysicsBodyFrame.h"
#include "physics-interaction/native/query/PhysicsShapeCast.h"
#include "physics-interaction/native/query/PhysicsRecursiveWrappers.h"
#include "physics-interaction/native/query/PhysicsUtils.h"
#include "physics-interaction/native/query/PhysicsScale.h"
#include "physics-interaction/native/havok/HavokMaterialRegistry.h"
#include "physics-interaction/native/havok/HavokRefCount.h"
#include "RE/Havok/hkVector4.h"
#include "RE/Havok/hknpMotion.h"
#include "RE/Bethesda/PlayerCharacter.h"
#include "RE/Bethesda/TESBoundObjects.h"
#include "RE/NetImmerse/NiUpdateData.h"
#include "RockConfig.h"
#include "RockUtils.h"
#include "physics-interaction/TransformMath.h"
#include "rock_support/Fo4VrRuntime.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <algorithm>
#include <array>
#include <atomic>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>
#include <xmmintrin.h>

namespace rock
{
    namespace hand_grab_detail
    {
        struct GrabAcquisitionContext
        {
            // Phase inputs stay valid for this synchronous acquisition call.
            RE::hknpWorld* world = nullptr;
            RE::bhkWorld* bhkWorld = nullptr;
            SelectedObject selection{};
            const GrabSharedObjectContext* sharedContext = nullptr;
            const BodyBoneColliderSet* bodyBoneColliders = nullptr;
            RE::NiAVObject* rootNode = nullptr;
            active_grab_body_lifecycle::BodyLifecycleSnapshot activeLifecycle{};

            // Drive tuning is fixed for the full acquisition.
            RE::NiTransform handWorldTransform{};
            float tau = 0.0f;
            float damping = 0.0f;
            float maxForce = 0.0f;
            float proportionalRecovery = 0.0f;
            float constantRecovery = 0.0f;

            // Body-set ownership controls failure unwind and final commit.
            RE::hknpBodyId objectBodyId{};
            std::uint16_t selectedOriginalMotionPropsId = 1;
            bool joiningPeerHeldObject = false;
            bool grabbedFromPullCatch = false;
            bool consumedPullPrepLifecycle = false;
            bool looseWeaponGrab = false;
            bool handPocketOnlyGrab = false;
            std::string objectName{ "(unnamed)" };
            const char* motionType = "UNKNOWN";
            std::uint64_t grabTraceId = 0;
            std::uint32_t bodySetSeedBodyId = object_physics_body_set::INVALID_BODY_ID;
            object_physics_body_set::ObjectPhysicsBodySet beforePrepBodySet;
            object_physics_body_set::ObjectPhysicsBodySet preparedBodySet;
            bool beforePrepScanCacheHit = false;
            bool preparedScanCacheHit = false;
            bool preparedBodySetPostPrepComplete = false;
            bool motionConverted = true;
            bool collisionEnabled = true;

            // Capture frames define the palm and object authority spaces.
            RE::NiTransform handBodyWorldAtGrab{};
            RE::NiTransform proxyFrameWorldAtGrab{};
            const char* proxyFrameSourceAtGrab = "unresolved";
            bool hasPalmProxyFrameAtGrab = false;
            RE::NiPoint3 grabAuthorityPivotAWorld{};
            RE::NiPoint3 palmPocketPivotAWorld{};
            float palmPocketToProxyDeltaGameUnits = 0.0f;
            GrabPalmBasisDelta grabPalmBasisDelta{};
            RE::NiPoint3 grabPivotAForPrimaryChoice{};
            RE::NiTransform proxyAuthorityFrameWorldAtGrab{};
            RE::NiAVObject* collidableNode = nullptr;
            RE::NiAVObject* meshSourceNode = nullptr;
            RE::NiTransform objectWorldTransform{};

            // Mesh capture owns geometry and the initial surface evidence.
            RE::NiPoint3 grabGripPoint{};
            float selectionToMeshDistanceGameUnits = 0.0f;
            bool meshGrabFound = false;
            MeshExtractionStats meshStats{};
            std::vector<TriangleData> grabMeshTriangles;
            std::vector<TriangleData> grabFingerPoseMeshTriangles;
            std::vector<GrabSurfaceTriangleData> grabSurfaceTriangles;
            std::vector<GrabLocalTriangle> grabLocalMeshTriangles;
            std::vector<GrabLocalTriangle> grabFingerPoseLocalMeshTriangles;
            GrabSurfaceHit grabSurfaceHit{};
            RuntimeGrabContactPatch contactPatchRuntime{};
            RuntimeMultiFingerGripContact multiFingerGripRuntime{};
            RE::NiPoint3 palmSeatPointWorld{};
            RE::NiPoint3 fingerEvidencePointWorld{};
            GrabSurfaceHit palmSeatSurfaceHit{};
            GrabSurfaceHit fingerEvidenceSurfaceHit{};
            bool contactPatchEvidenceAvailable = false;
            bool multiFingerGripUsed = false;
            bool palmSeatPointValid = false;
            bool fingerEvidencePointValid = false;
            bool activeGrabPointUsesMultiFingerEvidence = false;
            const char* contactPatchPivotAuthorityReason = "notEvaluated";
            const char* pivotAuthoritySource = "notEvaluated";
            bool pivotAuthorityNormalTrusted = false;
            bool pivotAuthorityPositionOnly = false;
            float pivotAuthorityPositionConfidence = 0.0f;
            float pivotAuthorityPocketDistanceGameUnits = std::numeric_limits<float>::max();
            float pivotAuthoritySelectionDeltaGameUnits = std::numeric_limits<float>::max();
            float pivotAuthorityLongLeverGameUnits = 0.0f;
            RE::NiAVObject* surfaceOwnerNode = nullptr;
            RE::NiAVObject* authoredGrabNode = nullptr;
            bool meshContactOnly = false;
            bool hasMeshSurfaceContact = false;
            const char* grabPointMode = "none";
            const char* grabFallbackReason = "none";
            const char* palmSeatPointMode = "none";
            const char* palmSeatFallbackReason = "none";
            const char* fingerEvidencePointMode = "none";
            const char* fingerEvidenceFallbackReason = "none";

            // Body resolution binds the selected surface to one drive scope.
            grab_contact_evidence_policy::GrabContactQualityMode grabContactQualityMode =
                grab_contact_evidence_policy::GrabContactQualityMode::LegacyPermissive;
            grab_contact_source_policy::GrabContactSourcePolicy contactSourcePolicy{};
            bool multiFingerEvidenceEnabled = false;
            bool hybridFingerProbeEvidenceEnabled = false;
            object_physics_body_set::PrimaryBodyChoice primaryChoice{};
            bool surfaceOwnerMatchesResolvedBody = true;
            mechanical_connected_body_set::MechanicalScope mechanicalScope{};
            bool relaxedArticulatedAuthority = false;
            bool visualMeshPivotAvailable = false;
            bool canonicalPivotAvailable = false;
            RE::NiPoint3 canonicalPivotPointWorld{};
            RE::NiPoint3 canonicalPivotNormalWorld{};
            const char* canonicalPivotMode = "none";

            // Contact evidence supplies the canonical capture inputs.
            hand_semantic_contact_state::SemanticContactCollection semanticContacts{};
            grab_three_phase::GrabPocketFrame acquisitionPocket{};
            GrabSurfaceHit palmPocketSurfaceHit{};
            bool palmPocketMeshAvailable = false;
            RuntimePinchPocketCandidate pinchPocketCandidate{};

            // Final capture results feed the drive and finger-pose publish.
            ResolvedGrabOffsetSource resolvedGrabOffsetSource{};
        };
    }
}

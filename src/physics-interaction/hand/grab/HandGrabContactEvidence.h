#pragma once

#include "physics-interaction/hand/Hand.h"
#include "physics-interaction/grab/GrabPinchPocket.h"

#include <array>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace rock::hand_grab_detail
{
    struct RuntimeGrabContactPatch
    {
        grab_contact_patch_math::GrabContactPatchResult<RE::NiPoint3> patch{};
        std::array<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>, kMaxGrabContactPatchSamples> samples{};
        std::array<std::uint32_t, 8> rejectedBodyIds{};
        std::uint32_t sampleCount = 0;
        std::uint32_t rejectedBodyIdCount = 0;
        int castHitCount = 0;
        int rejectedBodyHits = 0;
        int rejectedInvalidNormals = 0;
        int exactBodySamples = 0;
        int meshRecoveredSamples = 0;
        std::uint32_t rawAcceptedSampleCount = 0;
        std::uint32_t clusterRejectedSampleCount = 0;
        float clusterDepthSpreadGameUnits = 0.0f;
        float clusterMaxLateralGameUnits = 0.0f;
        float probeSpacingGameUnits = 0.0f;
        float probeRadiusGameUnits = 0.0f;
        float probeScale = 1.0f;
        const char* probeScaleReason = "configured";
        const char* clusterReason = "notEvaluated";
        bool meshSnapped = false;
        GrabSurfaceHit meshSnapHit{};
        grab_contact_patch_math::GrabContactPatchPivotDecision<RE::NiPoint3> pivotDecision{};
        const char* pointMode = "contactPatchUnavailable";
        bool normalTrusted = false;
        bool positionOnly = false;
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
        bool valid = false;
    };

    [[nodiscard]] bool nodeIsOrDescendsFrom(const RE::NiAVObject* root, const RE::NiAVObject* node);
    [[nodiscard]] bool acceptsSelectedMultibodyOwnerlessVisualMesh(
        const SelectedObject& selection,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        RE::NiAVObject* surfaceOwnerNode,
        const object_physics_body_set::ObjectPhysicsBodyRecord* surfaceOwnerRecord);
    void rememberRejectedContactPatchBody(RuntimeGrabContactPatch& result, std::uint32_t bodyId);
    [[nodiscard]] grab_pinch_pocket_policy::Config currentPinchPocketConfig();
    [[nodiscard]] RuntimePinchPocketCandidate buildRuntimePinchPocketCandidate(
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
        bool looseWeaponGrab);
    [[nodiscard]] RuntimeMultiFingerGripContact buildRuntimeMultiFingerGripContact(
        RE::hknpWorld* world,
        const object_physics_body_set::ObjectPhysicsBodySet& bodySet,
        std::uint32_t resolvedBodyId,
        const RE::NiTransform& objectWorldTransform,
        const hand_semantic_contact_state::SemanticContactCollection& semanticContacts,
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles,
        const Hand* hand,
        bool includeLiveColliderProbes);
    [[nodiscard]] RuntimeGrabContactPatch buildRuntimeGrabContactPatch(
        RE::hknpWorld* world,
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
        const std::vector<GrabSurfaceTriangleData>& surfaceTriangles);
}

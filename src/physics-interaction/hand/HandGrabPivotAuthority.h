#pragma once

#include "physics-interaction/hand/Hand.h"

#include <array>
#include <cstdint>
#include <limits>
#include <vector>

namespace rock::hand_grab_detail
{
    enum class GrabPivotAuthoritySource : std::uint8_t
    {
        None,
        AuthoredGrabNode,
        ContactPatchMeshSnap,
        ContactPatchPositionOnly,
        SelectionHitMeshSnap,
        PalmPocketMeshPoint,
        PinchPocketMeshPoint,
        GripSupportModel,
        LooseWeaponPrimaryAttach,
        PalmRayMeshPoint,
        CollisionFallback
    };

    struct GrabPivotAuthorityCandidate
    {
        const char* mode = "none";
        GrabPivotAuthoritySource source = GrabPivotAuthoritySource::None;
        RE::NiPoint3 point{};
        RE::NiPoint3 normal{};
        float pivotToPocketGameUnits = std::numeric_limits<float>::max();
        float selectionDeltaGameUnits = std::numeric_limits<float>::max();
        float authorityDeltaGameUnits = std::numeric_limits<float>::max();
        float longLeverGameUnits = 0.0f;
        float positionConfidence = 0.0f;
        float score = std::numeric_limits<float>::max();
        bool normalTrusted = false;
        bool positionOnlyPatch = false;
        bool valid = false;
    };

    struct GrabPivotAuthorityCandidateInput
    {
        const char* mode = "none";
        RE::NiPoint3 point{};
        RE::NiPoint3 normal{};
        bool normalTrusted = false;
        bool positionOnlyPatch = false;
        float positionConfidence = 0.0f;
        const grab_three_phase::GrabPocketFrame* pocket = nullptr;
        const SelectedObject* selection = nullptr;
        RE::NiPoint3 authorityPoint{};
        bool hasAuthorityPoint = false;
        RE::NiTransform objectWorldTransform{};
        const std::vector<GrabLocalTriangle>* localMeshTriangles = nullptr;
    };

    struct GrabPivotAuthorityResolution
    {
        const char* reason = "notEvaluated";
        GrabPivotAuthorityCandidate selected{};
        float baseAuthorityDeltaGameUnits = 0.0f;
        float extendedAuthorityDeltaGameUnits = 0.0f;
        float pocketImprovementGameUnits = 0.0f;
        float scoreImprovement = 0.0f;
        bool usePalmPocketPivot = false;
    };

    struct SeatedGrabPivotReacquireCandidate
    {
        RE::NiPoint3 pointWorld{};
        RE::NiPoint3 pointNodeLocal{};
        RE::NiPoint3 pointBodyLocalGame{};
        RE::NiPoint3 normalWorld{};
        RE::NiPoint3 normalNodeLocal{};
        float pocketDistanceGameUnits = std::numeric_limits<float>::max();
        float meshDistanceGameUnits = std::numeric_limits<float>::max();
        float longLeverGameUnits = 0.0f;
        const char* reason = "notEvaluated";
        bool normalTrusted = false;
        bool valid = false;
    };

    struct GrabSeatDepthStopResult
    {
        float depthGameUnits = 0.0f;
        std::uint32_t footprintSampleCount = 0;
        const char* reason = "notEvaluated";
        bool valid = false;
    };

    struct SeatedPalmPocketSupportPatch
    {
        grab_contact_patch_math::GrabContactPatchResult<RE::NiPoint3> patch{};
        std::array<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>, kMaxGrabContactPatchSamples> samples{};
        std::uint32_t sampleCount = 0;
        std::uint32_t rawSampleCount = 0;
        std::uint32_t clusterRejectedSampleCount = 0;
        float clusterDepthSpreadGameUnits = 0.0f;
        float clusterMaxLateralGameUnits = 0.0f;
        float probeSpacingGameUnits = 0.0f;
        float probeRadiusGameUnits = 0.0f;
        const char* reason = "notEvaluated";
        const char* clusterReason = "notEvaluated";
        bool valid = false;
        bool normalTrusted = false;
    };

    [[nodiscard]] float computeLocalMeshMaxDistanceFromPoint(
        const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiPoint3& originLocal);
    [[nodiscard]] const char* grabPivotAuthoritySourceName(GrabPivotAuthoritySource source);
    void applyFrozenGrabAuthorityFrameToGrabFrame(
        CanonicalGrabFrame& frame,
        const grab_authority_frame_math::FrozenGrabAuthorityFrame<RE::NiTransform>& frozen);
    [[nodiscard]] GrabPivotAuthoritySource inferGrabPivotAuthoritySource(
        const char* mode,
        bool positionOnlyPatch = false);
    [[nodiscard]] bool pivotAuthoritySourceShouldReacquireAtSeat(const char* source, bool positionOnly);
    [[nodiscard]] bool pivotAuthoritySourceIsFinalPinchOrSupport(const char* source);
    [[nodiscard]] bool pivotAuthoritySourceCanYieldToPalmPocket(GrabPivotAuthoritySource source);
    [[nodiscard]] bool pivotAuthorityRequiresSettledVisualRelation(
        const char* source,
        bool positionOnly,
        grab_three_phase::AcquisitionPhase acquisitionPhase);
    [[nodiscard]] float finitePositiveOr(float value, float fallback);
    [[nodiscard]] GrabPivotAuthorityCandidate assessGrabPivotAuthorityCandidate(
        const GrabPivotAuthorityCandidateInput& input);
    [[nodiscard]] GrabPivotAuthorityResolution resolveMeshBackedGrabPivotAuthority(
        const GrabPivotAuthorityCandidate& meshCandidate,
        const GrabPivotAuthorityCandidate& patchCandidate,
        const GrabPivotAuthorityCandidate& palmPocketCandidate,
        bool patchComparable,
        bool patchMeshSnapped,
        bool palmPocketEligible,
        float probeSpacingGameUnits,
        float meshSnapMaxDistanceGameUnits,
        float alignmentMaxSelectionDeltaGameUnits);
    [[nodiscard]] SeatedGrabPivotReacquireCandidate findSeatedGrabPivotNearPalmPocket(
        const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiTransform& currentNodeWorld,
        const RE::NiTransform& currentBodyWorld,
        const RE::NiPoint3& palmPocketWorld,
        const RE::NiPoint3& palmNormalWorld,
        float maxPocketDistanceGameUnits,
        float maxNormalAngleDegrees);
    [[nodiscard]] GrabSeatDepthStopResult computeGrabSeatDepthStop(
        const std::vector<GrabLocalTriangle>& localTriangles,
        const RE::NiTransform& objectNodeWorld,
        const RE::NiPoint3& gripPointWorld,
        const RE::NiPoint3& palmNormalWorld,
        float footprintRadiusGameUnits,
        float maxDepthGameUnits);
    [[nodiscard]] float gripAxisTiltRadiansForHand(bool isLeft);
    [[nodiscard]] RE::NiTransform rotateTransformWorldAboutPoint(
        const RE::NiTransform& transform,
        const RE::NiPoint3& unitAxisWorld,
        float angleRadians,
        const RE::NiPoint3& pivotWorld);
    [[nodiscard]] SeatedPalmPocketSupportPatch buildSeatedPalmPocketSupportPatch(
        const std::vector<GrabLocalTriangle>& localTriangles,
        std::uint32_t bodyId,
        const RE::NiTransform& currentNodeWorld,
        const RE::NiPoint3& palmPocketWorld,
        const RE::NiPoint3& anchorWorld,
        const RE::NiPoint3& palmNormalWorld,
        const RE::NiPoint3& palmTangentWorld,
        const RE::NiPoint3& palmBitangentWorld,
        float objectLeverEstimateGameUnits);
}

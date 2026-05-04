#pragma once

/*
 * ROCK keeps one canonical object-local grab frame so constraint pivots, visible
 * hand ownership, mesh finger solving, and debug logging all read the same data.
 * The previous scattered fields worked, but every new grab-quality feature had to
 * remember the same hand/body/node transforms separately. This wrapper preserves
 * the existing transform convention while making the grab frame an explicit piece
 * of held-object state.
 */

#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"
#include "GrabContactPatchMath.h"
#include "GrabSurfaceFrameMath.h"

#include <array>
#include <cstdint>
#include <vector>

namespace RE
{
    class NiAVObject;
}

namespace frik::rock
{
    inline constexpr std::size_t kMaxGrabContactPatchSamples = 5;

    struct GrabLocalTriangle
    {
        RE::NiPoint3 v0{};
        RE::NiPoint3 v1{};
        RE::NiPoint3 v2{};
    };

    struct CanonicalGrabFrame
    {
        RE::NiTransform rawHandSpace{};
        RE::NiTransform constraintHandSpace{};
        RE::NiTransform handBodyToRawHandAtGrab{};
        RE::NiTransform bodyLocal{};
        RE::NiTransform rootBodyLocal{};
        RE::NiTransform ownerBodyLocal{};
        RE::NiTransform liveHandWorldAtGrab{};
        RE::NiTransform handBodyWorldAtGrab{};
        RE::NiTransform objectNodeWorldAtGrab{};
        RE::NiTransform desiredObjectWorldAtGrab{};
        RE::NiPoint3 pivotAHandBodyLocalGame{};
        RE::NiPoint3 grabPivotWorldAtGrab{};
        RE::NiPoint3 surfacePointWorldAtGrab{};
        RE::NiPoint3 surfacePointLocal{};
        RE::NiPoint3 surfaceHitLocal{};
        RE::NiPoint3 surfaceNormalLocal{};
        RE::NiPoint3 oppositionThumbWorldAtGrab{};
        RE::NiPoint3 oppositionOpposingWorldAtGrab{};
        RE::NiPoint3 oppositionThumbObjectLocal{};
        RE::NiPoint3 oppositionOpposingObjectLocal{};
        RE::NiPoint3 multiFingerGripCenterWorldAtGrab{};
        RE::NiPoint3 multiFingerHandCenterWorldAtGrab{};
        RE::NiPoint3 multiFingerAverageNormalWorldAtGrab{};
        RE::NiPoint3 surfacePointBodyLocalGame{};
        RE::NiPoint3 pivotBBodyLocalGame{};
        grab_surface_frame_math::GrabSurfaceFrame<RE::NiPoint3> surfaceFrameLocal{};
        grab_surface_frame_math::GrabOrientationMode orientationModeUsed{ grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation };
        grab_surface_frame_math::GrabSurfaceAlignmentDecision surfaceAlignmentDecision{ grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedMode };
        std::array<grab_contact_patch_math::GrabContactPatchSample<RE::NiPoint3>, kMaxGrabContactPatchSamples> contactPatchSamples{};
        std::uint32_t surfaceTriangleIndex = 0xFFFF'FFFF;
        std::uint32_t surfaceShapeKey = 0xFFFF'FFFF;
        std::uint32_t surfaceShapeCollisionFilterInfo = 0;
        std::uint32_t contactPatchSampleCount = 0;
        std::uint32_t multiFingerContactGroupCount = 0;
        float surfaceHitFraction = 1.0f;
        float surfaceFrameConfidence = 0.0f;
        float surfacePivotToSurfaceDistanceGameUnits = 0.0f;
        float surfaceSelectionToMeshDistanceGameUnits = 0.0f;
        float contactPatchMeshSnapDeltaGameUnits = 0.0f;
        float multiFingerContactSpreadGameUnits = 0.0f;
        float handScaleAtGrab = 1.0f;
        const char* bodyResolutionReason = "none";
        const char* surfaceFrameFallbackReason = "none";
        const char* oppositionFrameReason = "none";
        const char* multiFingerContactReason = "none";
        const char* visualAuthorityContactReason = "none";
        /*
         * HIGGS only fades dynamic-grab motor authority when the object must be
         * synced from an initial/custom alignment. ROCK stores that decision in
         * the canonical frame so the constraint, visual hand, and diagnostics
         * agree about whether startup softness is part of this grab or stale
         * legacy behavior.
         */
        const char* motorFadeReason = "none";
        std::vector<GrabLocalTriangle> localMeshTriangles;
        RE::NiAVObject* heldNode = nullptr;
        bool hasMeshPoseData = false;
        bool hasSurfaceHit = false;
        bool hasSurfaceFrame = false;
        bool hasSurfaceShapeKey = false;
        bool hasFrozenPivotB = false;
        bool hasContactPatch = false;
        bool hasMultiFingerContactPatch = false;
        bool hasTelemetryCapture = false;
        bool hasOppositionFrame = false;
        bool visualAuthorityContactValid = false;
        bool fadeInGrabConstraint = false;

        void clear()
        {
            rawHandSpace = RE::NiTransform();
            constraintHandSpace = RE::NiTransform();
            handBodyToRawHandAtGrab = RE::NiTransform();
            bodyLocal = RE::NiTransform();
            rootBodyLocal = RE::NiTransform();
            ownerBodyLocal = RE::NiTransform();
            liveHandWorldAtGrab = RE::NiTransform();
            handBodyWorldAtGrab = RE::NiTransform();
            objectNodeWorldAtGrab = RE::NiTransform();
            desiredObjectWorldAtGrab = RE::NiTransform();
            pivotAHandBodyLocalGame = {};
            grabPivotWorldAtGrab = {};
            surfacePointWorldAtGrab = {};
            surfacePointLocal = {};
            surfaceHitLocal = {};
            surfaceNormalLocal = {};
            oppositionThumbWorldAtGrab = {};
            oppositionOpposingWorldAtGrab = {};
            oppositionThumbObjectLocal = {};
            oppositionOpposingObjectLocal = {};
            multiFingerGripCenterWorldAtGrab = {};
            multiFingerHandCenterWorldAtGrab = {};
            multiFingerAverageNormalWorldAtGrab = {};
            surfacePointBodyLocalGame = {};
            pivotBBodyLocalGame = {};
            surfaceFrameLocal = {};
            orientationModeUsed = grab_surface_frame_math::GrabOrientationMode::PreserveObjectRotation;
            surfaceAlignmentDecision = grab_surface_frame_math::GrabSurfaceAlignmentDecision::RejectedMode;
            contactPatchSamples = {};
            surfaceTriangleIndex = 0xFFFF'FFFF;
            surfaceShapeKey = 0xFFFF'FFFF;
            surfaceShapeCollisionFilterInfo = 0;
            contactPatchSampleCount = 0;
            multiFingerContactGroupCount = 0;
            surfaceHitFraction = 1.0f;
            surfaceFrameConfidence = 0.0f;
            surfacePivotToSurfaceDistanceGameUnits = 0.0f;
            surfaceSelectionToMeshDistanceGameUnits = 0.0f;
            contactPatchMeshSnapDeltaGameUnits = 0.0f;
            multiFingerContactSpreadGameUnits = 0.0f;
            handScaleAtGrab = 1.0f;
            bodyResolutionReason = "none";
            surfaceFrameFallbackReason = "none";
            oppositionFrameReason = "none";
            multiFingerContactReason = "none";
            visualAuthorityContactReason = "none";
            motorFadeReason = "none";
            localMeshTriangles.clear();
            heldNode = nullptr;
            hasMeshPoseData = false;
            hasSurfaceHit = false;
            hasSurfaceFrame = false;
            hasSurfaceShapeKey = false;
            hasFrozenPivotB = false;
            hasContactPatch = false;
            hasMultiFingerContactPatch = false;
            hasTelemetryCapture = false;
            hasOppositionFrame = false;
            visualAuthorityContactValid = false;
            fadeInGrabConstraint = false;
        }

        bool isValid() const { return heldNode != nullptr || hasMeshPoseData || !localMeshTriangles.empty(); }
    };
}

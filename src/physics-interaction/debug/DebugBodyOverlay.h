#pragma once

#include <array>
#include <cstdint>

#include "api/ROCKProviderApi.h"
#include "physics-interaction/weapon/WeaponTypes.h"
#include "physics-interaction/debug/SkeletonBoneDebugMath.h"
#include "physics-interaction/hand/HandColliderTypes.h"

#include "RE/Havok/hknpBodyId.h"
#include "RE/NetImmerse/NiPoint.h"
#include "RE/NetImmerse/NiTransform.h"

namespace RE
{
    class hknpWorld;
}

namespace rock::debug
{
    enum class BodyOverlayRole : std::uint8_t
    {
        RightHand,
        LeftHand,
        RightHandSegment,
        LeftHandSegment,
        BodyTorsoSegment,
        BodyArmSegment,
        BodyLegSegment,
        BodyFootSegment,
        Weapon,
        RightGrabAuthorityProxy,
        LeftGrabAuthorityProxy,
        RightGrabPivotSourceCollider,
        LeftGrabPivotSourceCollider,
        Target
    };

    enum class AxisOverlayRole : std::uint8_t
    {
        RightHandRaw,
        LeftHandRaw,
        RightHandBody,
        LeftHandBody,
        TargetBody,
        WeaponAuthority,
        NativeScopeLiveCamera,
        NativeScopeRockTarget,
        RightWeaponPrimaryGrip,
        LeftWeaponSupportGrip,
        RightFrikAppliedHand,
        LeftFrikAppliedHand,
        RightGrabHeldRelativeHandTarget,
        LeftGrabHeldRelativeHandTarget,
        RightGrabRockVisualTarget,
        LeftGrabRockVisualTarget,
        RightGrabDesiredObject,
        LeftGrabDesiredObject,
        RightGrabHeldNode,
        LeftGrabHeldNode,
        RightGrabPalmGeneratedDirect,
        LeftGrabPalmGeneratedDirect,
        RightGrabPalmAuthorityFrame,
        LeftGrabPalmAuthorityFrame,
        RightGrabAuthorityProxyTarget,
        LeftGrabAuthorityProxyTarget,
        RightGrabProxyReadback,
        LeftGrabProxyReadback,
        RightGrabForceTorqueLiveBody,
        LeftGrabForceTorqueLiveBody,
        RightGrabForceTorqueDesiredBody,
        LeftGrabForceTorqueDesiredBody,
        RightGrabMotorConstraintA,
        LeftGrabMotorConstraintA,
        RightGrabMotorConstraintB,
        LeftGrabMotorConstraintB,
        RightGrabMotorAtomTargetBody,
        LeftGrabMotorAtomTargetBody,
        RightGrabMotorColumnTargetBody,
        LeftGrabMotorColumnTargetBody,
        RightGrabMotorRelationInputBody,
        LeftGrabMotorRelationInputBody,
        RightGrabMotorRelationInverseBody,
        LeftGrabMotorRelationInverseBody,
        RightGrabMotorSolverEffectiveBody,
        LeftGrabMotorSolverEffectiveBody,
        RightCustomCalibrationOffset,
        LeftCustomCalibrationOffset
    };

    enum class AxisOverlaySource : std::uint8_t
    {
        Transform,
        Body
    };

    enum class AxisOverlayBasis : std::uint8_t
    {
        /*
         * Palm/proxy seating diagnostics need to distinguish the normal Ni
         * local-vector view from the generated collider axes authored into
         * matrix columns. Runtime behavior is not selected here; this only
         * tells the debug tripod which basis it is exposing.
         */
        NiLocalVectorToWorld,
        StoredColumns
    };

    enum class MarkerOverlayRole : std::uint8_t
    {
        RightGrabAnchor,
        LeftGrabAnchor,
        RightPalmNormal,
        LeftPalmNormal,
        RightPointing,
        LeftPointing,
        RightPalmPocketCenter,
        LeftPalmPocketCenter,
        RightPalmPocketRadius,
        LeftPalmPocketRadius,
        RightPinchPocketCenter,
        LeftPinchPocketCenter,
        RightPinchPocketAxis,
        LeftPinchPocketAxis,
        RightPinchDetectionDirection,
        LeftPinchDetectionDirection,
        RightGrabPivotA,
        LeftGrabPivotA,
        RightGrabPivotB,
        LeftGrabPivotB,
        RightGrabPivotError,
        LeftGrabPivotError,
        RightGrabSurfacePoint,
        LeftGrabSurfacePoint,
        RightGrabSurfaceNormal,
        LeftGrabSurfaceNormal,
        RightGrabContactPatchSample,
        LeftGrabContactPatchSample,
        RightGrabForceTorqueTargetPivot,
        LeftGrabForceTorqueTargetPivot,
        RightGrabForceTorqueLivePivot,
        LeftGrabForceTorqueLivePivot,
        RightGrabForceTorqueCorrection,
        LeftGrabForceTorqueCorrection,
        RightGrabForceTorqueLever,
        LeftGrabForceTorqueLever,
        RightGrabForceTorqueAxis,
        LeftGrabForceTorqueAxis,
        RightGrabMotorAnchorA,
        LeftGrabMotorAnchorA,
        RightGrabMotorAnchorB,
        LeftGrabMotorAnchorB,
        RightGrabMotorAtomTargetPivot,
        LeftGrabMotorAtomTargetPivot,
        RightGrabMotorAngularCommand,
        LeftGrabMotorAngularCommand,
        RightGrabMotorTargetBodyDelta,
        LeftGrabMotorTargetBodyDelta,
        RightGrabActivePivotBLiveBody,
        LeftGrabActivePivotBLiveBody,
        RightGrabActivePivotBDesiredBody,
        LeftGrabActivePivotBDesiredBody,
        RightGrabActivePivotBVisualNode,
        LeftGrabActivePivotBVisualNode,
        RightGrabActivePivotBVisualLock,
        LeftGrabActivePivotBVisualLock,
        RightGrabAuthorityProxyTarget,
        LeftGrabAuthorityProxyTarget,
        RightGrabAuthorityProxyOffset,
        LeftGrabAuthorityProxyOffset,
        RightGrabPivotSourceTriangle,
        LeftGrabPivotSourceTriangle,
        RightGrabPivotSourceMeshPoint,
        LeftGrabPivotSourceMeshPoint,
        RightGrabPivotSourceVisualMeshPoint,
        LeftGrabPivotSourceVisualMeshPoint,
        RightGrabPivotSourceCapturePoint,
        LeftGrabPivotSourceCapturePoint,
        RightGrabPivotSourceBodyVisualLock,
        LeftGrabPivotSourceBodyVisualLock,
        RightGrabPivotSourceCaptureMutation,
        LeftGrabPivotSourceCaptureMutation,
        RightGrabPivotSourceContactPoint,
        LeftGrabPivotSourceContactPoint,
        RightGrabSupportFramePivot,
        LeftGrabSupportFramePivot,
        RightGrabSupportFrameNormal,
        LeftGrabSupportFrameNormal,
        RightGrabSupportFrameAxis,
        LeftGrabSupportFrameAxis,
        RightGrabSupportFrameBinormal,
        LeftGrabSupportFrameBinormal,
        RightGrabFingerProbe,
        LeftGrabFingerProbe,
        RightGrabFingerPadProbe,
        LeftGrabFingerPadProbe,
        RightGrabFingerSurfaceTarget,
        LeftGrabFingerSurfaceTarget,
        GrabFingerSweepTip,
        GrabFingerSweepOuter,
        GrabFingerSweepInner,
        GrabFingerSweepPivot,
        GrabFingerSweepAuthoredOpen,
        GrabFingerSweepContact,
        GrabFingerSweepHitNormal,
        GrabFingerSweepMiss,
        GrabFingerSweepOutOfReach,
        GrabFingerSweepOverOpen,
        GrabFingerSweepClosedLimit,
        GrabFingerSweepLiveSkeleton,
        RightHandBoneContact,
        LeftHandBoneContact,
        NativeScopeLiveCamera,
        NativeScopeRockTarget,
        NativeScopeImmediateReadback,
        NativeScopePreWriteCamera,
        NativeScopeParentComposedCamera,
        NativeScopeHmd,
        NativeScopeCameraParent,
        NativeScopeSightBounds,
        NativeScopeMismatch,
        RightWeaponPrimaryGrip,
        LeftWeaponSupportGrip,
        RightWeaponAuthorityMismatch,
        LeftWeaponAuthorityMismatch,
        RightRootFlattenedFingerSkeleton,
        LeftRootFlattenedFingerSkeleton,
        TargetVisualOrigin,
        TargetRawBodyOrigin,
        TargetBodyTransformOrigin,
        TargetMotionOrigin,
        TargetBestOriginCandidate,
        TargetOriginErrorLine,
        RightGrabHeldRelativeHandTargetError,
        LeftGrabHeldRelativeHandTargetError,
        RightGrabRockVisualError,
        LeftGrabRockVisualError,
        RightGrabHeldDesiredError,
        LeftGrabHeldDesiredError,
        RightGrabTelemetryLabelAnchor,
        LeftGrabTelemetryLabelAnchor,
        RightDynamicHandRequestedDeviation,
        LeftDynamicHandRequestedDeviation,
        RightDynamicHandSolverResidual,
        LeftDynamicHandSolverResidual,
        AuthoredSupportGripPalmSeat,
        AuthoredSupportGripLiveSample,
        AuthoredSupportGripError
    };

    enum class SkeletonOverlayRole : std::uint8_t
    {
        Core,
        Head,
        RightArm,
        LeftArm,
        RightFinger,
        LeftFinger,
        RightLeg,
        LeftLeg
    };

    struct BodyOverlayEntry
    {
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        BodyOverlayRole role{ BodyOverlayRole::Target };
    };

    struct AxisOverlayEntry
    {
        AxisOverlaySource source{ AxisOverlaySource::Transform };
        AxisOverlayRole role{ AxisOverlayRole::TargetBody };
        RE::hknpBodyId bodyId{ 0x7FFF'FFFF };
        RE::NiTransform transform{};
        RE::NiPoint3 translationStart{};
        AxisOverlayBasis basis{ AxisOverlayBasis::NiLocalVectorToWorld };
        bool drawTranslationLine{ false };
    };

    struct MarkerOverlayEntry
    {
        MarkerOverlayRole role{ MarkerOverlayRole::RightGrabAnchor };
        RE::NiPoint3 position{};
        RE::NiPoint3 lineEnd{};
        float size{ 2.0f };
        bool drawPoint{ true };
        bool drawLine{ false };
    };

    struct SkeletonOverlayEntry
    {
        SkeletonOverlayRole role{ SkeletonOverlayRole::Core };
        RE::NiTransform transform{};
        RE::NiPoint3 parentPosition{};
        float pointSize{ 1.4f };
        float axisLength{ 4.0f };
        bool hasParent{ false };
        bool drawPoint{ true };
        bool drawAxis{ true };
        bool inPowerArmor{ false };
    };

    struct TextOverlayEntry
    {
        char text[128]{};
        float x{ 18.0f };
        float y{ 18.0f };
        float size{ 2.0f };
        float color[4]{ 0.90f, 1.0f, 0.95f, 0.92f };
        RE::NiPoint3 worldAnchor{};
        bool worldAnchored{ false };
    };

    struct ColoredLineOverlayEntry
    {
        RE::NiPoint3 start{};
        RE::NiPoint3 end{};
        float color[4]{ 1.0f, 1.0f, 1.0f, 1.0f };
    };

    struct BodyOverlayFrame
    {
        RE::hknpWorld* world{ nullptr };
        std::array<BodyOverlayEntry,
            MAX_WEAPON_COLLISION_BODIES +
                (hand_collider_semantics::kHandColliderBodyCountPerHand * 2) +
                skeleton_bone_debug_math::kStandardBodyColliderDescriptors.size() + 8>
            entries{};
        std::array<AxisOverlayEntry, 96> axisEntries{};
        std::array<MarkerOverlayEntry, 512> markerEntries{};
        std::array<SkeletonOverlayEntry, skeleton_bone_debug_math::skeletonOverlayBudget()> skeletonEntries{};
        // Borrowed only for the synchronous PublishFrame call, which copies
        // the bounded prefix before returning.
        const provider::RockProviderDebugOverlayLineV1* coloredLineEntries{ nullptr };
        std::array<TextOverlayEntry, 96> textEntries{};
        std::uint32_t count{ 0 };
        std::uint32_t axisCount{ 0 };
        std::uint32_t markerCount{ 0 };
        std::uint32_t skeletonCount{ 0 };
        std::uint32_t coloredLineCount{ 0 };
        std::uint32_t textCount{ 0 };
        bool drawRockBodies{ false };
        bool drawTargetBodies{ false };
        bool drawAxes{ false };
        bool drawMarkers{ false };
        bool drawSkeleton{ false };
        bool drawColoredLines{ false };
        bool drawText{ false };
    };

    void Install();
    bool IsInstalled();
    void PublishFrame(const BodyOverlayFrame& frame);
    void ClearFrame();
    void ClearShapeCache();
    // Stops and joins the owned CPU shape worker without uninstalling the
    // process-scoped OpenVR hook or D3D resources. PublishFrame restarts it.
    void ShutdownShapePipeline();
    // Current-frame left-eye stereo origin (game world units) from the same
    // fail-closed engine read the overlay camera uses. Diagnostic sampling only;
    // returns false without touching the overlay's stereo health telemetry.
    bool TryGetCurrentStereoOrigin(RE::NiPoint3& outOrigin);
}

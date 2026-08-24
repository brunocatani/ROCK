#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>

#include "RE/NetImmerse/NiPoint.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <SimpleIni.h>

#ifndef MAX_PATH
#define ROCK_DEFINED_MAX_PATH_FOR_FILEWATCH 1
#define MAX_PATH 260
#endif
#include <thomasmonkman-filewatch/FileWatch.hpp>
#if defined(ROCK_DEFINED_MAX_PATH_FOR_FILEWATCH)
#undef MAX_PATH
#undef ROCK_DEFINED_MAX_PATH_FOR_FILEWATCH
#endif

#include "physics-interaction/hand/HandSelection.h"
#include "physics-interaction/hand/SelectionBeamPolicy.h"
#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"
#include "physics-interaction/input/PipboyPauseGesturePolicy.h"
#include "physics-interaction/native/HavokTimingFixPolicy.h"

namespace rock
{
    class RockConfig
    {
    public:
        ~RockConfig() { stopFileWatch(); }

        void load();

        void reload();

        void processPendingConfigReload();

        void stopFileWatch();

        void subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback);
        void unsubscribeFromConfigChanged(const std::string& key);
        [[nodiscard]] std::filesystem::path getConfigDirectory() const;

        void suppressNextFileWatchReload() { _ignoreNextIniFileChange.store(true); }

        [[nodiscard]] bool persistPhysicsBool(const char* key, bool value);
        [[nodiscard]] bool persistGrabLegacyPalmPivotAHandspace(bool isLeft, const RE::NiPoint3& value);

        bool rockHavokTimingFixEnabled = true;
        float rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::kDefaultMinPhysicsFrameRate;
        int rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::kDefaultMaxSubsteps;

        bool rockSuppressRightFavoritesGameInput = true;
        // Function-level controls; raw OpenVR button state remains untouched.
        bool rockSuppressNativeVats = false;
        bool rockSuppressPipboyGameInputWhileHolding = true;
        float rockPipboyPauseHoldSeconds = pipboy_pause_gesture_policy::kDefaultHoldSeconds;
        bool rockSuppressTakeEquipGameInputWhileHolding = true;
        std::string rockSuppressTakeEquipFormTypes = "WEAP,ARMO,AMMO,MISC,INGR,ALCH,BOOK,KEYM,SLGM";
        bool rockSuppressNativeGrabHoverHaptics = true;
        bool rockGrabInputIntentStateEnabled = true;
        float rockGrabInputLeewaySeconds = 0.12f;
        float rockGrabInputForceSeconds = 0.08f;

        bool rockDeveloperModeEnabled = false;

        int rockLogLevel = 2;
        std::string rockLogPattern = "%Y-%m-%d %H:%M:%S.%e [%l] %v";
        int rockLogSampleMilliseconds = 2000;
        /*
         * Clock-domain note: the remaining *Frames diagnostic settings below
         * (profiler warmup/report windows, dump/audit/log intervals) are
         * deliberate PUBLICATION-COUNT SAMPLING — "log every Nth frame" for
         * video/frame correlation — not gameplay durations. Gameplay behavior
         * settings are seconds-based.
         */
        bool rockPerformanceProfilerEnabled = false;
        int rockPerformanceProfilerLogIntervalFrames = 300;
        int rockPerformanceProfilerWarmupFrames = 120;
        bool rockPerformanceProfilerOverlayText = false;

        RE::NiPoint3 rockPalmNormalHandspace = RE::NiPoint3(0.0f, 1.0f, 0.0f);
        RE::NiPoint3 rockPointingVectorHandspace = RE::NiPoint3(0.0f, 1.0f, 0.0f);
        bool rockReversePalmNormal = true;
        bool rockReverseFarGrabNormal = true;

        // ROCK-exclusive equipped-weapon hand preference. This never reads or
        // mutates Fallout 4 VR's native handedness/controller mapping.
        bool rockLeftHandedMode = false;
        // Physical-gunstock mode combines two independent stages: support
        // attach consumes only post-capture motion, then final presentation
        // aligns the neutral bore to the damped firing-wrist +X axis.
        bool rockGunstockModeEnabled = false;
        // Additive final-alignment trim in the untrimmed damped firing-wrist
        // frame. Yaw is +Z, pitch is +Y, and roll is +X (the aligned bore).
        float rockGunstockAlignmentPitchDegrees = 0.0f;
        float rockGunstockAlignmentYawDegrees = 0.0f;
        float rockGunstockAlignmentRollDegrees = 0.0f;
        // ROCK-native firing-grip handoff. This remains independent from the
        // addon-owned realistic detach/drop feature set.
        bool rockAmbidextrousFiringGripEnabled = true;
        float rockFiringGripPromotionRadius = 5.0f;
        float rockLeftFiringAimYawDegrees = 0.0f;
        float rockLeftFiringAimPitchDegrees = 0.0f;
        float rockLeftFiringAimOffsetXGameUnits = 0.0f;
        float rockLeftFiringAimOffsetYGameUnits = 0.0f;
        float rockLeftFiringAimOffsetZGameUnits = 0.0f;
        bool rockWeaponCollisionBlocksProjectiles = false;
        bool rockWeaponCollisionBlocksSpells = false;
        bool rockWeaponCollisionDynamicBoxEnabled = true;
        float rockWeaponCollisionDynamicBoxPaddingGameUnits = 0.5f;
        float rockWeaponCollisionDynamicInverseInertiaMultiplier = 1.2f;
        float rockWeaponCollisionDynamicMaxLinearVelocityHavok = 15.0f;
        float rockWeaponCollisionDynamicMaxAngularVelocityRadians = 35.0f;
        float rockWeaponCollisionDynamicDivergenceTeleportGameUnits = 80.0f;
        float rockWeaponCollisionDynamicDivergenceTeleportDwellSeconds = 0.3f;
        float rockWeaponCollisionDynamicRenderMinTranslationGameUnits = 0.05f;
        float rockWeaponCollisionDynamicRenderMinRotationDegrees = 0.25f;
        // Elapsed stable-witness window before a generation-driven weapon
        // visual rebuild commits (historical 8-frame tuning at 90 Hz).
        float rockWeaponCollisionVisualStabilizationSeconds = 8.0f / 90.0f;
        float rockWeaponCollisionMaxLinearVelocity = 50.0f;
        float rockWeaponCollisionMaxAngularVelocity = 100.0f;
        float rockWeaponSizeClassPistolMaxWeight = 6.0f;
        float rockWeaponSizeClassRifleMaxWeight = 20.0f;
        float rockWeaponInteractionTouchRadius = 2.0f;
        float rockWeaponInteractionProbeRadius = 12.0f;
        float rockFiringGripProximitySupportRadius = 6.0f;
        float rockRealisticGrenadeFuseSeconds = 5.0f;
        bool rockWeaponSupportGripHandLerpEnabled = true;
        float rockWeaponSupportGripHandLerpTimeMin = 0.12f;
        float rockWeaponSupportGripHandLerpTimeMax = 0.20f;
        float rockWeaponSupportGripHandLerpMinDistance = 1.0f;
        float rockWeaponSupportGripHandLerpMaxDistance = 14.0f;
        bool rockWeaponSupportSurfaceSeatEnabled = true;
        float rockWeaponSupportSurfaceSeatMaxDegrees = 35.0f;
        bool rockWeaponVisualReturnEnabled = true;
        float rockWeaponVisualReturnTimeMin = 0.12f;
        float rockWeaponVisualReturnTimeMax = 0.20f;
        float rockWeaponVisualReturnMinDistance = 1.0f;
        float rockWeaponVisualReturnMaxDistance = 14.0f;
        float rockWeaponVisualReturnMinAngleDegrees = 5.0f;
        float rockWeaponVisualReturnMaxAngleDegrees = 90.0f;

        // The held firing-hand A/X gesture is the sole native-scope activation
        // path. A release before the threshold remains reload input.
        float rockManualScopeHoldSeconds = 0.30f;

        // Used only when generated Scope/Sight geometry is unavailable, or
        // explicitly forced for an incorrectly accepted optic collider.
        // Position and rotation are relative to the firing-grip palm seat in
        // equipped Weapon axes; rotation follows native camera calibration.
        bool rockNativeScopeForceFiringGripFallback = false;
        float rockNativeScopeFiringGripFallbackOffsetXGameUnits = 0.0f;
        float rockNativeScopeFiringGripFallbackOffsetYGameUnits = 0.0f;
        float rockNativeScopeFiringGripFallbackOffsetZGameUnits = 0.0f;
        float rockNativeScopeFiringGripFallbackPitchDegrees = 0.0f;
        float rockNativeScopeFiringGripFallbackYawDegrees = 0.0f;
        float rockNativeScopeFiringGripFallbackRollDegrees = 0.0f;

        // Fine tuning for FO4VR's game-native world_scope.nif overlay. These
        // offsets are applied in the calibrated model-root frame after ROCK
        // anchors it to the resolved scope point; they do not move the native
        // activation camera or alter its entry detection.
        float rockNativeScopeOverlayOffsetXGameUnits = 0.0f;
        float rockNativeScopeOverlayOffsetYGameUnits = 0.0f;
        float rockNativeScopeOverlayOffsetZGameUnits = 0.0f;
        float rockNativeScopeOverlayPitchDegrees = 0.0f;
        float rockNativeScopeOverlayYawDegrees = 0.0f;
        float rockNativeScopeOverlayRollDegrees = 0.0f;

        /*
         * Canonical free-hand world collision: dynamic palm/finger proxy
         * bodies are solver-clipped by static world surfaces and drive the
         * rendered hand through one-way visual authority.
         */
        bool rockNativeMeleeSuppressionEnabled = true;
        bool rockNativeMeleeFullSuppression = true;
        bool rockNativeMeleeSuppressWeaponSwing = true;
        bool rockNativeMeleeSuppressHitFrame = true;
        bool rockNativeMeleeDebugLogging = false;
        bool rockNativeCharacterControllerObjectContactFilterEnabled = true;

        bool rockHighlightEnabled = true;
        int rockHighlightIntensityMode = 3;
        std::string rockHighlightColor = "orange";
        bool rockSelectionBeamEnabled = true;
        float rockSelectionBeamSegmentSizeGameUnits = selection_beam_policy::kDefaultSegmentSizeGameUnits;
        float rockSelectionBeamCurveLiftGameUnits = selection_beam_policy::kDefaultCurveLiftGameUnits;
        float rockSelectionBeamAlpha = selection_beam_policy::kDefaultAlpha;

        bool rockDebugShowColliders = false;
        bool rockDebugShowTargetColliders = false;
        bool rockDebugShowHandAxes = false;
        bool rockDebugShowGrabPivots = false;
        bool rockDebugShowGrabPocketNormal = false;
        bool rockDebugDrawGrabContactPatch = false;
        bool rockDebugDrawGrabForceTorque = false;
        bool rockDebugDrawGrabForceTorqueText = false;
        bool rockDebugDrawGrabPivotSourceCollider = false;
        bool rockDebugDrawGrabPivotSourceEvidence = false;
        bool rockDebugDrawGrabSupportFrame = false;
        bool rockDebugDrawGrabPockets = false;
        bool rockDebugShowGrabFingerProbes = false;
        bool rockDebugShowGrabFingerSweptArc = false;
        bool rockDebugShowGrabFingerSweptArcText = true;
        bool rockDebugShowGrabFingerSweptArcLiveSkeleton = true;
        bool rockDebugShowPalmVectors = false;
        bool rockDebugDrawHandColliders = false;
        bool rockDebugDrawHandBoneColliders = false;
        bool rockDebugDrawDynamicHandColliders = false;
        bool rockDebugDrawHandBoneContacts = false;
        bool rockDebugDrawGrabAuthorityProxy = false;
        int rockDebugMaxHandBoneBodiesDrawn = 48;
        int rockDebugMaxBodyBoneBodiesDrawn = 32;
        bool rockDebugDrawWeaponColliders = false;
        bool rockDebugDrawNativeScopeActivation = false;
        bool rockDebugDrawAuthoredGripActivationZones = false;
        bool rockDebugDrawGunstockAlignment = false;
        bool rockDebugDrawDynamicWeaponColliders = false;
        bool rockDebugDumpWeaponAnimNodes = false;
        int rockDebugMaxWeaponBodiesDrawn = 100;
        int rockDebugWeaponAnimNodeDumpIntervalFrames = 120;
        int rockDebugMaxShapeCapturesPerFrame = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeCapturesPerFrame);
        int rockDebugMaxConvexSupportVertices = 8;
        int rockDebugMaxCompoundChildren = static_cast<int>(debug_overlay_policy::kDefaultMaxCompoundChildren);
        int rockDebugMaxCompoundDepth = static_cast<int>(debug_overlay_policy::kDefaultMaxCompoundDepth);
        int rockDebugMaxShapeQueuedJobs = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeQueuedJobs);
        int rockDebugMaxShapeCompletedJobs = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeCompletedJobs);
        int rockDebugMaxShapeUploadsPerFrame = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeUploadsPerFrame);
        int rockDebugMaxShapeCacheEntries = static_cast<int>(debug_overlay_policy::kDefaultShapeCacheBudget);
        int rockDebugMaxShapeCacheBytes = static_cast<int>(debug_overlay_runtime::kDefaultMaxShapeCacheBytes);
        int rockDebugMaxBodyInstances = static_cast<int>(debug_overlay_runtime::kDefaultMaxBodyInstances);
        int rockDebugMaxLineVertices = static_cast<int>(debug_overlay_policy::kDefaultLineVertexBudget);
        int rockDebugMaxTextVertices = static_cast<int>(debug_overlay_runtime::kDefaultMaxTextVertices);
        bool rockDebugUseBoundsForHeavyConvex = true;
        bool rockDebugVerboseLogging = false;
        bool rockDebugGrabFrameLogging = false;
        bool rockDebugVideoSyncMarker = false;
        float rockDebugVideoSyncMarkerSize = 4.0f;
        bool rockDebugGrabFingerPoseLogging = false;
        bool rockDebugGrabTimelineTrace = false;
        bool rockDebugGrabAfterSolveAnomalySampling = false;
        bool rockDebugGrabTransformTelemetry = false;
        bool rockDebugGrabTransformTelemetryText = false;
        bool rockDebugGrabTransformTelemetryAxes = false;
        int rockDebugGrabTimelineTraceIntervalFrames = 1;
        int rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        int rockDebugGrabTransformTelemetryTextMode = 0;
        bool rockDebugShowGrabNotifications = false;
        bool rockDebugShowWeaponNotifications = false;
        bool rockDebugWeaponOmodDumpEnabled = false;
        bool rockDebugWeaponOmodCoverageAudit = false;
        int rockDebugWeaponOmodCoverageAuditIntervalFrames = 450;
        bool rockDebugWeaponOmodSelfHeal = false;
        bool rockDebugWorkbenchWeaponReattach = false;
        bool rockDebugHandTransformParity = false;
        bool rockDebugWorldObjectOriginDiagnostics = false;
        int rockDebugWorldObjectOriginLogIntervalFrames = 120;
        float rockDebugWorldObjectOriginMismatchWarnGameUnits = 5.0f;
        bool rockDebugShowRootFlattenedFingerSkeletonMarkers = false;
        bool rockDebugShowSkeletonBoneVisualizer = false;
        bool rockDebugDrawSkeletonBoneAxes = false;
        bool rockDebugLogSkeletonBones = false;
        int rockDebugSkeletonBoneMode = 1;
        int rockDebugSkeletonBoneSource = 1;
        int rockDebugMaxSkeletonBonesDrawn = 256;
        int rockDebugMaxSkeletonBoneAxesDrawn = 80;
        int rockDebugSkeletonBoneLogIntervalFrames = 120;
        bool rockDebugLogSkeletonBoneTruncation = false;
        float rockDebugRootFlattenedFingerSkeletonMarkerSize = 1.4f;
        float rockDebugSkeletonBonePointSize = 1.4f;
        float rockDebugSkeletonBoneAxisLength = 4.0f;
        std::string rockDebugSkeletonBoneLogFilter = "RArm_Hand,LArm_Hand,RArm_Finger23,LArm_Finger23,Chest,Pelvis";
        std::string rockDebugSkeletonAxisBoneFilter = "";

        bool rockBodyBoneCollidersEnabled = true;
        bool rockBodyBoneLegAndFootCollidersEnabled = false;
        float rockBodyBoneColliderStandardRadiusScale = 1.0f;
        float rockBodyBoneColliderStandardLengthScale = 1.0f;
        float rockBodyBoneColliderStandardConvexRadiusScale = 1.0f;
        float rockBodyBoneColliderPowerArmorRadiusScale = 1.0f;
        float rockBodyBoneColliderPowerArmorLengthScale = 1.0f;
        float rockBodyBoneColliderPowerArmorConvexRadiusScale = 1.0f;
        float rockBodyBoneColliderTorsoRadiusScale = 1.0f;
        float rockBodyBoneColliderArmRadiusScale = 1.0f;
        float rockBodyBoneColliderLegRadiusScale = 1.0f;
        float rockBodyBoneColliderFootRadiusScale = 1.0f;
        float rockBodyBoneColliderTorsoLengthScale = 1.0f;
        float rockBodyBoneColliderArmLengthScale = 1.0f;
        float rockBodyBoneColliderLegLengthScale = 1.0f;
        float rockBodyBoneColliderFootLengthScale = 1.0f;
        std::string rockBodyBoneColliderZoneScaleOverrides = "";
        std::string rockBodyBoneColliderRadiusScaleOverrides = "";
        bool rockHandCollisionStaticWorldEnabled = true;
        bool rockGlobalSurfaceGrabEnabled = true;
        bool rockExperimentalSurfaceMeshGrabEnabled = false;
        float rockExperimentalSurfaceMeshGrabMaxProjectionDistanceGameUnits = 48.0f;
        int rockExperimentalSurfaceMeshGrabMaxTriangles = 20000;
        int rockExperimentalSurfaceMeshGrabMaxPatchTriangles = 2048;
        std::string rockHandBoneColliderRadiusScaleOverrides = "";
        std::string rockHandPalmColliderDimensionScaleOverrides = "";
        bool rockHandBoneCollidersRequirePalmAnchor = true;
        bool rockHandBoneCollidersRequireAllFingerBones = true;
        float rockHandBoneColliderMaxLinearVelocity = 200.0f;
        float rockHandBoneColliderMaxAngularVelocity = 500.0f;

        float rockNearDetectionRange = 25.0f;
        float rockFarDetectionRange = 350.0f;
        float rockNearCastRadiusGameUnits = 3.5f;
        float rockNearCastDistanceGameUnits = 7.0f;
        float rockFarCastRadiusGameUnits = 21.0f;
        int rockCloseSelectionAngleDegrees = selection_query_policy::kDefaultSelectionAimAngleDegrees;
        int rockFarSelectionAngleDegrees = selection_query_policy::kDefaultSelectionAimAngleDegrees;
        bool rockFarSelectionHmdConeEnabled = true;
        float rockFarSelectionHmdConeHalfAngleDegrees = selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees;
        std::string rockFarSelectionBlockedReferenceFormIds = "";
        std::string rockFarSelectionBlockedBaseFormIds = "";
        std::string rockFarSelectionBlockedFormTypes = "";
        std::string rockFarSelectionBlockedLayers = "";
        float rockCloseSelectionBehindPalmToleranceGameUnits = 2.0f;
        std::uint32_t rockSelectionShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        std::uint32_t rockFarClipRayFilterInfo = selection_query_policy::kDefaultFarClipRayFilterInfo;
        float rockPullApplyVelocityTime = 0.2f;
        float rockPullOwnerGraceSeconds = 1.0f;
        float rockPullTrackHandTime = 0.1f;
        float rockPullDestinationZOffsetHavok = 0.01f;
        float rockPullDurationA = 0.715619f;
        float rockPullDurationB = -0.415619f;
        float rockPullDurationC = 0.656256f;
        float rockPullMaxVelocityHavok = 10.0f;
        float rockPullAutoGrabDistanceGameUnits = 18.0f;
        float rockPullCatchRetryMaxTimeSeconds = 0.65f;
        bool rockPullCatchWideReacquireEnabled = true;
        float rockPullCatchWideReacquireRadiusGameUnits = 32.0f;
        float rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42.0f;
        int rockObjectPhysicsTreeMaxDepth = 12;
        bool rockDynamicPushAssistEnabled = true;
        float rockDynamicPushMinSpeed = 0.35f;
        float rockDynamicPushMaxImpulse = 2.0f;
        float rockDynamicPushCooldownSeconds = 0.08f;

        float rockGrabLinearTau = 0.03f;
        float rockGrabLinearDamping = 0.8f;
        float rockGrabLinearProportionalRecovery = 2.0f;
        float rockGrabLinearConstantRecovery = 1.0f;

        float rockGrabAngularTau = 0.03f;
        float rockGrabAngularDamping = 0.8f;
        float rockGrabAngularProportionalRecovery = 2.0f;
        float rockGrabAngularConstantRecovery = 1.0f;

        float rockGrabConstraintMaxForce = 2000.0f;
        float rockGrabMaxForceToMassRatio = 500.0f;
        float rockForceGrabAttachSettleSeconds = 0.10f;
        bool rockGrabEffectiveMotorMassFloorEnabled = true;
        float rockGrabEffectiveMotorMassFloor = 2.0f;
        bool rockGrabPhysicsRateForceScalingEnabled = true;
        float rockGrabPhysicsRateReferenceHz = 90.0f;
        float rockGrabPhysicsRateForceScaleExponent = 0.5f;
        float rockGrabPhysicsRateMinForceScale = 0.75f;
        float rockGrabPhysicsRateMaxForceScale = 1.35f;
        float rockGrabForceFadeInTime = 0.1f;
        RE::NiPoint3 rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        RE::NiPoint3 rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, 0.0f, 0.0f);
        float rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = 4.5f;
        float rockGrabLooseWeaponSharedConstraintAngularForceMultiplier = 2.0f;
        float rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = 1.0f;
        float rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = 1.0f;
        float rockGrabTauMin = 0.01f;
        float rockGrabTauLerpSpeed = 0.5f;
        bool rockGrabLongObjectAngularScalingEnabled = true;
        float rockGrabLongObjectReferenceLeverGameUnits = 24.0f;
        float rockGrabLongObjectMinAngularScale = 0.35f;
        bool rockGrabPivotQualityAngularScalingEnabled = true;
        float rockGrabPositionOnlyAngularScale = 0.55f;
        float rockGrabSmallObjectReferenceLeverGameUnits = 12.0f;
        float rockGrabSmallObjectAngularScale = 0.65f;
        float rockGrabLowContactSupportAngularScale = 0.75f;
        float rockGrabMinAngularAuthorityScale = 0.30f;
        float rockGrabWeakPivotTwistScale = 0.35f;

        float rockGrabMaxInertiaRatio = 10.0f;
        float rockGrabMinInertia = 0.01f;

        float rockGrabMaxDeviation = 50.0f;
        float rockGrabMaxDeviationTime = 2.0f;
        float rockThrowVelocityMultiplier = 1.5f;
        bool rockGrabControllerDerivedThrowVelocityEnabled = true;
        float rockGrabThrowObjectVelocityBlend = 0.35f;
        float rockGrabThrowTangentialVelocityScale = 1.0f;
        float rockGrabThrowMaxVelocityHavok = 12.0f;
        float rockGrabThrowAngularVelocityScale = 1.0f;
        float rockGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0f;
        float rockGrabReleaseHandCollisionDelaySeconds = 0.10f;
        bool rockShoulderStashEnabled = true;
        // Equipped weapons remain equipped and use the native sheath/draw
        // transition. This ROCK-owned shoulder gesture is independent of an
        // addon's physical detach/drop capability.
        bool rockEquippedWeaponShoulderStashEnabled = true;
        bool rockShoulderStashUseBodyZoneColliders = true;
        bool rockShoulderStashUseHmdBackVolume = true;
        float rockShoulderStashEnterPaddingGameUnits = 5.0f;
        float rockShoulderStashExitPaddingGameUnits = 8.0f;
        float rockShoulderStashMinDwellSeconds = 0.08f;
        float rockShoulderStashMaxSpeedGameUnitsPerSecond = 140.0f;
        // Elapsed shoulder-stash contact freshness and miss tolerance
        // (historical 4- and 18-frame tunings at 90 Hz).
        float rockShoulderStashRecentContactSeconds = 4.0f / 90.0f;
        float rockShoulderStashSustainedContactMissSeconds = 18.0f / 90.0f;
        RE::NiPoint3 rockShoulderStashHmdBackRightOffsetGameUnits = RE::NiPoint3(14.0f, -18.0f, -6.85f);
        RE::NiPoint3 rockShoulderStashHmdBackLeftOffsetGameUnits = RE::NiPoint3(-14.0f, -18.0f, -6.85f);
        float rockShoulderStashHmdBackRadiusGameUnits = 11.0f;
        float rockShoulderStashHmdBackEnterPaddingGameUnits = 0.0f;
        float rockShoulderStashHmdBackExitPaddingGameUnits = 2.0f;
        float rockShoulderStashHmdBackMinBehindGameUnits = 4.0f;
        bool rockShoulderStashShowCollectedNotifications = true;
        bool rockMouthConsumeEnabled = true;
        bool rockMouthConsumeAllowPoison = false;
        RE::NiPoint3 rockMouthConsumeHmdOffsetGameUnits = RE::NiPoint3(0.0f, 7.0f, -7.0f);
        float rockMouthConsumeRadiusGameUnits = 5.5f;
        float rockMouthConsumeEnterPaddingGameUnits = 0.0f;
        float rockMouthConsumeExitPaddingGameUnits = 1.0f;
        float rockMouthConsumeMinDwellSeconds = 0.08f;
        float rockMouthConsumeMaxSpeedGameUnitsPerSecond = 120.0f;
        bool rockGrabNearbyDampingEnabled = true;
        float rockGrabNearbyDampingRadius = 90.0f;
        float rockGrabNearbyDampingSeconds = 0.35f;
        float rockGrabNearbyLinearDamping = 3.0f;
        float rockGrabNearbyAngularDamping = 5.5f;
        bool rockGrabHeldMassMovementSlowdownEnabled = true;
        float rockGrabHeldMassMovementMassProportion = 0.675f;
        float rockGrabHeldMassMovementMassExponent = 1.0f;
        float rockGrabHeldMassMovementMaxReduction = 75.0f;
        float rockGrabHeldMassMovementFadeOutSeconds = 5.0f;
        float rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        float rockGrabNearConvergeDistanceGameUnits = 28.0f;
        float rockGrabPocketDepthGameUnits = 7.0f;
        float rockGrabPocketRadiusGameUnits = 9.0f;
        float rockGrabSeatDepthMaxGameUnits = 30.0f;
        float rockGrabSeatDepthFootprintRadiusGameUnits = 10.0f;
        // Penetration backstop only, and only for plate-shaped objects: a
        // narrow footprint bounds detectable tilt penetration to r*sin(theta),
        // which is why the seating radius above cannot double as the safety
        // check. Irregular shapes keep the seating radius.
        float rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits = 6.0f;
        float rockGrabSeatDepthSkinGameUnits = 0.5f;
        float rockGrabGripInsetGameUnits = 2.0f;
        float rockGrabGripMaxInsetGameUnits = 6.0f;
        float rockGrabConvergeMaxTimeSeconds = 0.35f;
        // Elapsed stable dwell inside the grab pocket before convergence
        // promotes (historical 3-frame tuning at 90 Hz).
        float rockGrabConvergeStableSeconds = 3.0f / 90.0f;
        float rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        float rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        bool rockGrabMultiFingerContactValidationEnabled = true;
        int rockGrabContactQualityMode = 1;
        int rockGrabMinFingerContactGroups = 3;
        float rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        float rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        float rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
        // Elapsed opposition/patch contact freshness (historical 5-frame
        // tuning at 90 Hz).
        float rockGrabOppositionContactMaxAgeSeconds = 5.0f / 90.0f;
        bool rockGrabPinchPocketEnabled = true;
        bool rockGrabPinchCloseSelectionEnabled = true;
        float rockGrabPinchCompactMaxExtentGameUnits = 8.0f;
        float rockGrabPinchThinRodMaxLengthGameUnits = 18.0f;
        float rockGrabPinchThinRodMaxCrossSectionGameUnits = 4.0f;
        float rockGrabPinchMaxPocketDistanceGameUnits = 8.0f;
        float rockGrabPinchMinFingerGapGameUnits = 1.0f;
        float rockGrabPinchMaxFingerGapGameUnits = 12.0f;
        float rockGrabPinchThumbIndexMaxOpenValue = 0.45f;
        float rockGrabPinchOtherFingerCurlValue = 0.20f;
        float rockGrabPinchSurfaceInsetGameUnits = 0.5f;
        RE::NiPoint3 rockGrabPinchDetectionDirectionHandspace = RE::NiPoint3(1.0f, 0.0f, 0.0f);
        float rockGrabPinchDetectionAxisBlend = 0.65f;
        bool rockGrabHandLerpEnabled = true;
        float rockGrabHandLerpTimeMin = 0.10f;
        float rockGrabHandLerpTimeMax = 0.20f;
        float rockGrabHandLerpMinDistance = 7.0f;
        float rockGrabHandLerpMaxDistance = 14.0f;
        bool rockGrabHandReturnEnabled = true;
        float rockGrabHandReturnTimeMin = 0.10f;
        float rockGrabHandReturnTimeMax = 0.20f;
        float rockGrabHandReturnMinDistance = 7.0f;
        float rockGrabHandReturnMaxDistance = 14.0f;
        float rockGrabHandReturnMinAngleDegrees = 5.0f;
        float rockGrabHandReturnMaxAngleDegrees = 90.0f;
        bool rockGrabMeshFingerPoseEnabled = true;
        bool rockGrabMeshJointPoseEnabled = true;
        int rockGrabFingerPoseUpdateInterval = 3;
        float rockGrabFingerMinValue = 0.2f;
        float rockGrabFingerPoseSmoothingSpeed = 14.0f;
        bool rockGrabMeshLocalTransformPoseEnabled = true;
        float rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        float rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        float rockGrabFingerSurfaceAimStrength = 0.75f;
        bool rockGrabFingerRejectBacksideHits = true;
        float rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        float rockGrabFingerSweepContactRadiusGameUnits = 1.0f;
        float rockGrabFingerSweepMaxOpenValue = 2.0f;
        float rockGrabThumbSweepMaxOpenValue = 2.0f;
        float rockGrabFingerPoseResolveWindowSeconds = 2.0f;
        float rockGrabThumbOppositionStrength = 1.0f;
        float rockGrabThumbAlternateCurveStrength = 0.65f;
        bool rockGrabThumbSurfaceSafetyEnabled = true;
        float rockGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
        float rockGrabLateralWeight = 0.6f;
        float rockGrabDirectionalWeight = 0.4f;
        float rockGrabMaxTriangleDistance = 100.0f;
        bool rockGrabMeshContactOnly = true;
        bool rockGrabRequireMeshContact = true;
        bool rockGrabContactPatchEnabled = true;
        int rockGrabContactPatchProbeCount = 9;
        float rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        float rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        float rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        float rockGrabContactPatchMaxNormalAngleDegrees = 35.0f;
        float rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        bool rockGrabNodeAnchorsEnabled = true;
        bool rockGrabNodeRejectOppositeHandAnchor = true;
        bool rockPrintGrabNodeInfo = false;
        std::string rockGrabNodeNameRight = "ROCK:GrabR";
        std::string rockGrabNodeNameLeft = "ROCK:GrabL";
        std::string rockGrabNodeNameBlacklist = "ROCK:GrabR,ROCK:GrabL";
        bool rockSelectedCloseFingerCurlEnabled = true;
        float rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        float rockSelectedCloseFingerAnimValue = 0.9f;
        float rockPulledAngularDamping = 8.0f;
        bool rockPullToObjectCenterEnabled = true;
        bool rockPullLongAxisPresentationEnabled = true;
        bool rockForceGrabSeatAlignmentEnabled = true;
        bool rockGrabSeatRollAlignmentEnabled = true;
        float rockPullPresentationMinElongationRatio = 2.0f;
        float rockGrabSeatRollMinSecondElongationRatio = 1.25f;
        float rockPullPresentationAngularGainPerSecond = 6.0f;
        float rockPullPresentationMaxAngularSpeedRadiansPerSecond = 8.0f;
        float rockPullPresentationGripAxisTiltDegrees = 10.0f;

        RE::NiPoint3 rockRightGrabLegacyPalmPivotAHandspace = RE::NiPoint3(6.0f, -2.0f, 0.2f);
        RE::NiPoint3 rockLeftGrabLegacyPalmPivotAHandspace = RE::NiPoint3(6.0f, -2.0f, -0.2f);

        bool rockGrabHapticsEnabled = true;
        float rockGrabHapticDurationSeconds = 0.055f;
        float rockGrabHapticBaseIntensity = 0.12f;
        float rockGrabHapticMaxIntensity = 0.80f;
        float rockGrabHapticMassScale = 0.06f;
        float rockGrabHapticMassExponent = 0.60f;
        float rockPullStartHapticIntensity = 0.18f;
        float rockPullCatchHapticIntensity = 0.22f;
        float rockSelectionLockHapticIntensity = 0.15f;
        float rockSelectionLockReleaseHapticIntensity = 0.10f;
        float rockSelectionLockReleaseHapticDurationSeconds = 0.02f;
        bool rockSurfaceGrabHapticsEnabled = true;
        float rockSurfaceGrabHapticDurationSeconds = 0.075f;
        float rockSurfaceGrabHapticIntensity = 0.85f;
        bool rockHeldImpactHapticsEnabled = true;
        float rockHeldImpactHapticDurationSeconds = 0.035f;
        float rockHeldImpactHapticBaseIntensity = 0.12f;
        float rockHeldImpactHapticMaxIntensity = 0.85f;
        float rockHeldImpactHapticSpeedScale = 0.006f;
        float rockHeldImpactHapticMassScale = 0.035f;
        float rockHeldImpactHapticMassExponent = 0.55f;
        float rockHeldImpactHapticMinSpeedGameUnits = 8.0f;
        float rockHeldImpactHapticCooldownSeconds = 0.12f;
        float rockHeldImpactHapticDampedMultiplier = 0.55f;
        bool rockShoulderStashHapticsEnabled = true;
        float rockShoulderStashCandidateHapticDurationSeconds = 0.075f;
        float rockShoulderStashCandidateHapticBaseIntensity = 0.20f;
        float rockShoulderStashCandidateHapticIntensity = 0.42f;
        float rockShoulderStashCandidateHapticIntervalSeconds = 0.075f;
        float rockShoulderStashCommitHapticDurationSeconds = 0.12f;
        float rockShoulderStashCommitHapticIntensity = 0.85f;
        bool rockMouthConsumeHapticsEnabled = true;
        float rockMouthConsumeCandidateHapticDurationSeconds = 0.050f;
        float rockMouthConsumeCandidateHapticBaseIntensity = 0.22f;
        float rockMouthConsumeCandidateHapticIntensity = 0.45f;
        float rockMouthConsumeCandidateHapticIntervalSeconds = 0.075f;
        float rockMouthConsumeCommitHapticDurationSeconds = 0.12f;
        float rockMouthConsumeCommitHapticIntensity = 0.85f;

    private:
        void resetToDefaults();

        void readValuesFromIni(CSimpleIniA& ini);

        [[nodiscard]] bool saveRuntimeIni(CSimpleIniA& ini, const char* reason);

        void startFileWatch();

        std::string _iniFilePath;

        std::unique_ptr<filewatch::FileWatch<std::string>> _fileWatch;

        std::atomic<std::filesystem::file_time_type> _lastIniFileWriteTime;

        std::unordered_map<std::string, std::function<void(const std::string&)>> _onConfigChangedSubscribers;

        std::atomic<bool> _ignoreNextIniFileChange = false;

        std::atomic<bool> _selfIniWriteInProgress = false;

        std::atomic<std::filesystem::file_time_type> _lastSelfIniWriteTime{};

        std::atomic<bool> _reloadPending = false;

        std::thread _fileWatchInitThread;
    };

    inline RockConfig g_rockConfig;
}

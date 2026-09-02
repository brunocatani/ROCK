#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <string>
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

#include "physics-interaction/hand/SelectionBeamPolicy.h"
#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"
#include "physics-interaction/input/PipboyPauseGesturePolicy.h"
#include "physics-interaction/native/HavokTimingFixPolicy.h"

namespace rock
{
    struct RockConfigValues
    {
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

        // ROCK-native Immersive Weapons firing-role boundary. The same
        // release, part-carry, reattach and haptic contract follows whichever
        // physical hand currently occupies the firing grip.
        bool rockFiringGripDetachEnabled = true;
        bool rockFiringGripDetachPosePreservationEnabled = true;
        float rockFiringGripReattachRadiusGameUnits = 12.0f;
        float rockFiringGripReattachCylinderRadiusGameUnits = 2.0f;
        float rockFiringGripHapticDurationSeconds = 0.10f;
        float rockFiringGripAttachHapticIntensity = 0.85f;
        float rockFiringGripDetachHapticIntensity = 0.30f;

        // ROCK-native firing-grip handoff. This remains independent from the
        // role-neutral Immersive Weapons detach/drop feature set.
        bool rockAmbidextrousFiringGripEnabled = true;
        // ROCK-local support acquisition policy. Matched provider part
        // targets retain their declared authority; current weapons without a
        // usable authored support pose may use the qualified dynamic fallback.
        bool rockAuthoredOnlyEquippedWeaponSupportGrabsEnabled = true;
        // Optional ROCK-owned input mode for equipped-weapon grips only.
        bool rockEquippedWeaponToggleGrabEnabled = false;
        float rockFiringGripPromotionRadius = 5.0f;
        float rockLeftFiringAimYawDegrees = 0.0f;
        float rockLeftFiringAimPitchDegrees = 0.0f;
        float rockLeftFiringAimOffsetXGameUnits = 0.0f;
        float rockLeftFiringAimOffsetYGameUnits = 0.0f;
        float rockLeftFiringAimOffsetZGameUnits = 0.0f;
        bool rockWeaponCollisionBlocksProjectiles = false;
        bool rockWeaponCollisionBlocksSpells = false;
        // Elapsed stable-witness window before a generation-driven weapon
        // visual rebuild commits (historical 8-frame tuning at 90 Hz).
        float rockWeaponCollisionVisualStabilizationSeconds = 8.0f / 90.0f;
        float rockWeaponCollisionMaxLinearVelocity = 50.0f;
        float rockWeaponCollisionMaxAngularVelocity = 100.0f;
        float rockWeaponCollisionGripRecoveryDistanceGameUnits = 210.0f;
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
        bool rockNativeMeleeSuppressionEnabled = false;
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
        bool rockDebugDrawColliderPhaseDiagnostics = false;
        bool rockDebugShowHandAxes = false;
        bool rockDebugShowGrabPivots = false;
        bool rockDebugDrawGrabPockets = false;
        bool rockDebugShowGrabFingerSweptArc = false;
        bool rockDebugShowGrabFingerSweptArcText = true;
        bool rockDebugShowGrabFingerSweptArcLiveSkeleton = true;
        bool rockDebugShowPalmVectors = false;
        bool rockDebugDrawHandColliders = false;
        bool rockDebugDrawHandBoneColliders = false;
        bool rockDebugDrawBodyBoneColliders = false;
        bool rockDebugDrawDynamicHandColliders = false;
        bool rockDebugDrawHandBoneContacts = false;
        bool rockDebugDrawGrabAuthorityProxy = false;
        int rockDebugMaxHandBoneBodiesDrawn = 48;
        int rockDebugMaxBodyBoneBodiesDrawn = 32;
        bool rockDebugDrawWeaponColliders = false;
        bool rockDebugDrawGrabbedWeaponPartCollider = false;
        bool rockDebugDrawNativeScopeActivation = false;
        bool rockDebugDrawAuthoredGripActivationZones = false;
        bool rockDebugDrawWeaponAuthority = false;
        bool rockDebugGripFailureTelemetry = false;
        bool rockDebugDrawLooseWeaponGripZones = false;
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
        bool rockDebugShowWeaponNotifications = false;
        bool rockDebugWeaponOmodDumpEnabled = false;
        bool rockDebugWeaponOmodCoverageAudit = false;
        int rockDebugWeaponOmodCoverageAuditIntervalFrames = 450;
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
        bool rockSurfaceMeshGrabEnabled = false;
        float rockSurfaceMeshGrabMaxProjectionDistanceGameUnits = 48.0f;
        int rockSurfaceMeshGrabMaxTriangles = 20000;
        int rockSurfaceMeshGrabMaxPatchTriangles = 2048;
        std::string rockHandBoneColliderRadiusScaleOverrides = "";
        std::string rockHandPalmColliderDimensionScaleOverrides = "";
        bool rockHandBoneCollidersRequirePalmAnchor = true;
        float rockHandBoneColliderMaxLinearVelocity = 200.0f;
        float rockHandBoneColliderMaxAngularVelocity = 500.0f;

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
        RE::NiPoint3 rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        RE::NiPoint3 rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
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
        float rockGrabPocketDepthGameUnits = 7.0f;
        float rockGrabPocketRadiusGameUnits = 9.0f;
        float rockGrabSeatDepthMaxGameUnits = 30.0f;
        float rockGrabSeatDepthFootprintRadiusGameUnits = 10.0f;
        float rockGrabSeatDepthSkinGameUnits = 0.5f;
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
        float rockGrabThumbOppositionStrength = 1.0f;
        float rockGrabThumbAlternateCurveStrength = 0.65f;
        bool rockGrabThumbSurfaceSafetyEnabled = true;
        float rockGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
        float rockGrabLateralWeight = 0.6f;
        float rockGrabDirectionalWeight = 0.4f;
        float rockGrabMaxTriangleDistance = 100.0f;
        bool rockSelectedCloseFingerCurlEnabled = true;
        float rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        float rockSelectedCloseFingerAnimValue = 0.9f;
        bool rockPullToObjectCenterEnabled = true;
        bool rockPullLongAxisPresentationEnabled = true;
        float rockPullPresentationMinElongationRatio = 2.0f;
        float rockPullPresentationAngularGainPerSecond = 6.0f;
        float rockPullPresentationMaxAngularSpeedRadiansPerSecond = 8.0f;

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
    };

    class RockConfig : public RockConfigValues
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

        [[nodiscard]] bool persistPhysicsBool(const char* key, bool value);
        [[nodiscard]] bool persistGrabLegacyPalmPivotAHandspace(bool isLeft, const RE::NiPoint3& value);

    private:
        void resetToDefaults();

        void readValuesFromIni(CSimpleIniA& ini, bool materializeMissingDefaults = false);

        [[nodiscard]] bool createDefaultIniIfMissing();

        [[nodiscard]] bool saveRuntimeIni(CSimpleIniA& ini, const char* reason);

        void startFileWatch();

        std::string _iniFilePath;

        std::unique_ptr<filewatch::FileWatch<std::string>> _fileWatch;

        std::atomic<std::filesystem::file_time_type> _lastIniFileWriteTime;

        std::unordered_map<std::string, std::function<void(const std::string&)>> _onConfigChangedSubscribers;

        std::atomic<bool> _selfIniWriteInProgress = false;

        std::atomic<std::filesystem::file_time_type> _lastSelfIniWriteTime{};

        std::atomic<bool> _reloadPending = false;
    };

    inline RockConfig g_rockConfig;
}

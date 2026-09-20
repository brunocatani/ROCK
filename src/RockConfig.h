#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "RE/NetImmerse/NiPoint.h"
#include "physics-interaction/native/ShellCasingGracePolicy.h"

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

#include "config/ConfigurationStore.h"
#include "physics-interaction/debug/DebugOverlayRuntimeSettings.h"
#include "physics-interaction/input/PipboyPauseGesturePolicy.h"
#include "physics-interaction/input/BareFistGesturePolicy.h"
#include "physics-interaction/native/HavokTimingFixPolicy.h"
#include "physics-interaction/weapon/GripZoneIndicatorPolicy.h"

namespace rock
{
    struct RockConfigValues
    {
        bool operator==(const RockConfigValues&) const = default;

        bool rockHavokTimingFixEnabled = true;
        bool rockVatsPhysicsFixes = true;
        float rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::kDefaultMinPhysicsFrameRate;
        int rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::kDefaultMaxSubsteps;

        // Function-level controls; raw OpenVR button state remains untouched.
        bool rockSuppressNativeVats = false;
        float rockPipboyPauseHoldSeconds = pipboy_pause_gesture_policy::kDefaultHoldSeconds;
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

        // Optional fixed injector grips and body-contact consumption.
        bool rockImmersiveAidEnabled = true;

        // Master switch for every ROCK-owned visual recoil profile and delivery.
        bool rockImmersiveRecoil = true;
        bool rockBipodMode = true;
        float rockLaserRecoilPercent = 100.0f;
        float rockPistolOneHandRecoilPercent = 300.0f;
        float rockPistolTwoHandRecoilPercent = 80.0f;
        float rockRifleOneHandRecoilPercent = 300.0f;
        float rockRifleTwoHandRecoilPercent = 80.0f;
        float rockShotgunOneHandRecoilPercent = 300.0f;
        float rockShotgunTwoHandRecoilPercent = 80.0f;
        float rockHeavyOneHandRecoilPercent = 300.0f;
        float rockHeavyTwoHandRecoilPercent = 292.1f;
        float rockDefaultOneHandRecoilPercent = 300.0f;
        float rockDefaultTwoHandRecoilPercent = 80.0f;


        // ROCK-native Immersive Weapons firing-role boundary. The same
        // release, part-carry, reattach and haptic contract follows whichever
        // physical hand currently occupies the firing grip.
        bool rockDetachEitherHand = true;
        bool rockFiringGripDetachPosePreservationEnabled = true;
        // 1: keep equipped; 2: transfer to a held loose weapon; 3: drop without grabbing.
        int rockWeaponDropMode = 1;
        bool rockKeepPreviousWeaponInHandOnEquip = false;
        // 1: toggle both grips; 2: toggle firing only; 3: hold both grips.
        int rockWeaponGrabMode = 1;
        // Allow dynamic support grabs beyond authored grips. False preserves
        // authored-grip preference and the existing missing-pose fallback.
        bool rockGrabAnywhereOnWeapon = false;
        float rockMeleeGripPitchDegrees = 0.0f;
        float rockFiringGripReattachRadiusGameUnits = 10.0f;
        float rockFiringGripReattachCylinderRadiusGameUnits = 3.0f;
        float rockGripZoneIndicatorDiameterGameUnits =
            grip_zone_indicator_policy::kDefaultDiameterGameUnits;
        float rockFiringGripHapticDurationSeconds = 0.10f;
        float rockFiringGripAttachHapticIntensity = 0.85f;
        float rockFiringGripDetachHapticIntensity = 0.30f;

        // ROCK-native firing-grip handoff. This remains independent from the
        // role-neutral Immersive Weapons detach/drop feature set.
        bool rockAmbidextrousFiringGripEnabled = true;
        float rockLeftFiringAimYawDegrees = 0.0f;
        float rockLeftFiringAimPitchDegrees = 0.0f;
        float rockLeftFiringAimOffsetXGameUnits = 0.0f;
        float rockLeftFiringAimOffsetYGameUnits = 0.0f;
        float rockLeftFiringAimOffsetZGameUnits = 0.0f;
        RE::NiPoint3 rockLeftFiringGripOffsetGameUnits{ 0.0f, 0.0f, 0.0f };
        RE::NiPoint3 rockRightSupportGripOffsetGameUnits{ 0.0f, 0.0f, 0.0f };
        bool rockWeaponCollisionBlocksProjectiles = false;
        bool rockWeaponCollisionBlocksSpells = false;
        bool npcDynamicCollisions = false;
        float rockWeaponShellCollisionGraceMs = shell_casing_grace::kDefaultMilliseconds;
        // A/B comparison: original support hulls or bounded gap-preserving compounds.
        bool rockWeaponCollisionPreserveGaps = true;
        // Elapsed stable-witness window before a generation-driven weapon
        // visual rebuild commits (historical 8-frame tuning at 90 Hz).
        float rockWeaponCollisionVisualStabilizationSeconds = 0.0889f;
        float rockWeaponCollisionMaxLinearVelocity = 800.0f;
        float rockWeaponCollisionMaxAngularVelocity = 800.0f;
        float rockWeaponCollisionGripRecoveryDistanceGameUnits = 210.0f;
        float rockWeaponInteractionTouchRadius = 2.0f;
        float rockWeaponInteractionProbeRadius = 12.0f;
        float rockFiringGripProximitySupportRadius = 8.0f;
        bool rockImmersiveGrenades = true;
        float rockRealisticGrenadeFuseSeconds = 3.0f;
        bool rockWeaponSupportGripHandLerpEnabled = true;
        float rockWeaponSupportGripHandLerpTimeMin = 0.12f;
        float rockWeaponSupportGripHandLerpTimeMax = 0.20f;
        float rockWeaponSupportGripHandLerpMinDistance = 1.0f;
        float rockWeaponSupportGripHandLerpMaxDistance = 14.0f;
        bool rockWeaponSupportSurfaceSeatEnabled = true;
        float rockWeaponSupportSurfaceSeatMaxDegrees = 35.0f;

        bool rockEnableImmersiveScopes = true;

        // While immersive scopes are enabled, the held firing-hand A/X gesture is the sole native-scope activation
        // path. A release before the threshold remains reload input.
        float rockManualScopeHoldSeconds = 0.30f;

        // Used only when generated Scope/Sight geometry is unavailable, or
        // explicitly forced for an incorrectly accepted optic collider.
        // Position and rotation are relative to the firing-grip palm seat in
        // equipped Weapon axes; rotation follows native camera calibration.
        bool rockNativeScopeForceFiringGripFallback = false;
        float rockNativeScopeFiringGripFallbackOffsetXGameUnits = 0.0f;
        float rockNativeScopeFiringGripFallbackOffsetYGameUnits = 0.0f;
        float rockNativeScopeFiringGripFallbackOffsetZGameUnits = 10.0f;
        float rockNativeScopeFiringGripFallbackPitchDegrees = 0.0f;
        float rockNativeScopeFiringGripFallbackYawDegrees = 0.0f;
        float rockNativeScopeFiringGripFallbackRollDegrees = 0.0f;

        // Fine tuning for FO4VR's game-native world_scope.nif overlay. These
        // offsets are applied in the calibrated model-root frame after ROCK
        // anchors it to the resolved scope point; they do not move the native
        // activation camera or alter its entry detection.
        float rockNativeScopeOverlayOffsetXGameUnits = 0.0f;
        float rockNativeScopeOverlayOffsetYGameUnits = 10.0f;
        float rockNativeScopeOverlayOffsetZGameUnits = 0.0f;
        float rockNativeScopeOverlayPitchDegrees = 0.0f;
        float rockNativeScopeOverlayYawDegrees = 0.0f;
        float rockNativeScopeOverlayRollDegrees = 0.0f;

        /*
         * Canonical free-hand world collision: dynamic palm/finger proxy
         * bodies are solver-clipped by static world surfaces and drive the
         * rendered hand through one-way visual authority.
         */
        bool rockEnableVanillaMelee = true;
        bool rockRockyModeEnabled = true;
        float rockRockyModeHoldSeconds = bare_fist_gesture::kDefaultHoldSeconds;
        bool rockNativeCharacterControllerObjectContactFilterEnabled = true;

        bool rockHighlightEnabled = true;
        int rockHighlightIntensityMode = 2;
        std::string rockHighlightColor = "blue";

        bool rockDebugShowColliders = false;
        bool rockDebugShowTargetColliders = false;
        bool rockDebugDrawColliderPhaseDiagnostics = false;
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
        bool rockDebugDrawBodyBoneColliders = false;
        bool rockDebugDrawDynamicHandColliders = false;
        bool rockDebugDrawHandBoneContacts = false;
        bool rockDebugDrawGrabAuthorityProxy = false;
        int rockDebugMaxHandBoneBodiesDrawn = 48;
        int rockDebugMaxBodyBoneBodiesDrawn = 32;
        bool rockDebugDrawWeaponColliders = false;
        bool rockDebugDrawGrabbedWeaponPartCollider = false;
        bool rockDebugDrawNativeScopeActivation = false;
        bool rockDebugNativeScopeShotAlignment = false;
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
        bool rockDebugHandTransformParity = false;
        bool rockDebugHandWorldAuthority = false;
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
        std::string rockHandBoneColliderRadiusScaleOverrides = "";
        std::string rockHandPalmColliderDimensionScaleOverrides = "PalmAnchor=0.8,1.0,2.0";
        bool rockHandBoneCollidersRequirePalmAnchor = true;
        float rockHandBoneColliderMaxLinearVelocity = 800.0f;
        float rockHandBoneColliderMaxAngularVelocity = 800.0f;

        int rockObjectPhysicsTreeMaxDepth = 12;
        bool rockDynamicPushAssistEnabled = true;
        float rockDynamicPushMinSpeed = 0.35f;
        float rockDynamicPushMaxImpulse = 2.0f;
        float rockDynamicPushCooldownSeconds = 0.08f;

        float rockGrabLinearTau = 0.1f;
        float rockGrabLinearDamping = 0.8f;
        float rockGrabLinearProportionalRecovery = 2.0f;
        float rockGrabLinearConstantRecovery = 1.0f;

        float rockGrabAngularTau = 0.03f;
        float rockGrabAngularDamping = 0.8f;
        float rockGrabAngularProportionalRecovery = 2.0f;
        float rockGrabAngularConstantRecovery = 1.0f;

        float rockGrabConstraintMaxForce = 2000.0f;
        float rockGrabMaxForceToMassRatio = 500.0f;
        float rockGrabFreeLinearAcceleration = 1000.0f;
        float rockGrabFreeAngularAcceleration = 6000.0f;
        float rockForceGrabAttachSettleSeconds = 0.10f;
        bool rockGrabEffectiveMotorMassFloorEnabled = true;
        float rockGrabEffectiveMotorMassFloor = 2.0f;
        float rockGrabForceFadeInTime = 0.1f;
        RE::NiPoint3 rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.5f, -1.0f, 0.0f);
        RE::NiPoint3 rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.5f, -1.0f, 0.0f);
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
        float rockGrabThrowMaxVelocityHavok = 12.0f;
        float rockGrabThrowAngularVelocityScale = 1.0f;
        float rockGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0f;
        float rockGrabReleaseHandCollisionDelaySeconds = 0.8f;
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
        float rockShoulderStashRecentContactSeconds = 0.0444f;
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
        float rockGrabConvergeMaxTimeSeconds = 0.35f;
        // Elapsed stable dwell inside the grab pocket before convergence
        // promotes (historical 3-frame tuning at 90 Hz).
        float rockGrabConvergeStableSeconds = 0.0333f;
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
        float rockGrabOppositionContactMaxAgeSeconds = 0.0556f;
        bool rockGrabPinchPocketEnabled = true;
        bool rockGrabPinchCloseSelectionEnabled = true;
        float rockGrabPinchMaxVolumeCubicGameUnits = 100.0f;
        float rockGrabPinchMaxPocketDistanceGameUnits = 8.0f;
        float rockGrabPinchMinFingerGapGameUnits = 1.0f;
        float rockGrabPinchMaxFingerGapGameUnits = 12.0f;
        float rockGrabPinchThumbIndexMaxOpenValue = 0.45f;
        float rockGrabPinchOtherFingerCurlValue = 0.20f;
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
        float rockGrabFingerSweepContactRadiusGameUnits = 0.6f;
        float rockGrabFingerSweepMaxOpenValue = 2.0f;
        float rockGrabThumbSweepMaxOpenValue = 1.5f;
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
        bool rockSelectedCloseFingerCurlEnabled = true;
        float rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        float rockSelectedCloseFingerAnimValue = 0.9f;
        bool rockPullToObjectCenterEnabled = true;
        bool rockPullLongAxisPresentationEnabled = true;
        bool rockForceGrabSeatAlignmentEnabled = true;
        bool rockGrabSeatRollAlignmentEnabled = true;
        float rockPullPresentationMinElongationRatio = 2.0f;
        float rockGrabSeatRollMinSecondElongationRatio = 1.25f;
        float rockPullPresentationAngularGainPerSecond = 6.0f;
        float rockPullPresentationMaxAngularSpeedRadiansPerSecond = 8.0f;
        float rockPullPresentationGripAxisTiltDegrees = 15.0f;

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

        static void buildCompiledDefaults(CSimpleIniA& target);
        [[nodiscard]] static RockConfigValues parseValues(CSimpleIniA& source);
        [[nodiscard]] std::uint64_t configRevision() const noexcept { return _configRevision.load(std::memory_order_acquire); }
        [[nodiscard]] bool visitSettings(configuration_api::Group group, configuration_api::VisitorV1 visitor, void* context) const;
        [[nodiscard]] bool persistSetting(configuration_api::Group group, const char* section, const char* key, const char* value, std::string& error);

        void processPendingConfigReload();

        void stopFileWatch();

        void subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback);
        void unsubscribeFromConfigChanged(const std::string& key);
        [[nodiscard]] std::filesystem::path getConfigDirectory() const;


    private:
        void resetToDefaults();

        void readValuesFromIni(CSimpleIniA& ini, bool materializeMissingDefaults = false);

        [[nodiscard]] bool loadStore(bool createConsumer);

        void startFileWatch();

        // Configuration tasks may visit/save concurrently with a runtime reload.
        // Only load/reload applies live values on the runtime owner. API writes
        // persist under this lock and request that owner's normal reload.
        mutable std::mutex _storeMutex;
        std::unique_ptr<config::ConfigurationStore> _store;
        std::unique_ptr<filewatch::FileWatch<std::string>> _fileWatch;
        std::unordered_map<std::string, std::function<void(const std::string&)>> _onConfigChangedSubscribers;
        std::atomic<std::uint64_t> _configRevision{ 0 };
        std::atomic<std::int64_t> _lastFileEventTicks{ 0 };

        std::atomic<bool> _reloadPending = false;
    };

    inline RockConfig g_rockConfig;
}

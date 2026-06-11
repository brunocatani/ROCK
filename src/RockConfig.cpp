

#include "RockConfig.h"

#include <ShlObj.h>
#include <SimpleIni.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <thread>

#include "common/CommonUtils.h"
#include "physics-interaction/grab/GrabNodeNamePolicy.h"
#include "physics-interaction/grab/GrabLocomotionAuthorityBridge.h"
#include "physics-interaction/grab/GrabPinchPocket.h"
#include "physics-interaction/grab/GrabThreePhase.h"
#include "physics-interaction/hand/HandLifecycle.h"
#include "physics-interaction/input/InputRemapPolicy.h"
#include "physics-interaction/grab/NearbyGrabDamping.h"
#include "physics-interaction/PhysicsLog.h"
#include "physics-interaction/RockLoggingPolicy.h"
#include "physics-interaction/weapon/SeeThroughScopesPolicy.h"
#include "resources.h"

namespace
{

    constexpr auto SECTION = "PhysicsInteraction";
    constexpr auto DEBUG_SECTION = "Debug";
    constexpr int kDefaultWeaponCollisionSupportFitTargetPoints = 96;
    constexpr int kMinWeaponCollisionSupportFitTargetPoints = 4;
    constexpr int kMaxWeaponCollisionSupportFitTargetPoints = 252;
    constexpr int kDefaultWeaponCollisionVisualStabilizationFrames = 8;
    constexpr int kMaxWeaponCollisionVisualStabilizationFrames = 60;
    constexpr float kDefaultWeaponCollisionSupportFitMaxErrorGameUnits = 0.5f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier = 4.5f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = 1.0f;
    constexpr float kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = 1.0f;
    constexpr float kDefaultGrabThrowObjectVelocityBlend = 0.35f;
    constexpr float kDefaultGrabThrowTangentialVelocityScale = 1.0f;
    constexpr float kDefaultGrabThrowMaxVelocityHavok = 12.0f;
    constexpr float kDefaultGrabThrowAngularVelocityScale = 1.0f;
    constexpr float kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond = 18.0f;
    constexpr float kDefaultGrabLongObjectReferenceLeverGameUnits = 24.0f;
    constexpr float kDefaultGrabLongObjectMinAngularScale = 0.35f;
    constexpr float kDefaultGrabEffectiveMotorMassFloor = 2.0f;
    constexpr float kDefaultGrabPhysicsRateReferenceHz = 90.0f;
    constexpr float kDefaultGrabPhysicsRateForceScaleExponent = 0.5f;
    constexpr float kDefaultGrabPhysicsRateMinForceScale = 0.75f;
    constexpr float kDefaultGrabPhysicsRateMaxForceScale = 1.35f;
    constexpr float kDefaultGrabPositionOnlyAngularScale = 0.55f;
    constexpr float kDefaultGrabSmallObjectReferenceLeverGameUnits = 12.0f;
    constexpr float kDefaultGrabSmallObjectAngularScale = 0.65f;
    constexpr float kDefaultGrabLowContactSupportAngularScale = 0.75f;
    constexpr float kDefaultGrabMinAngularAuthorityScale = 0.30f;
    constexpr float kDefaultGrabWeakPivotTwistScale = 0.35f;
    constexpr float kDefaultGrabMinInertia = 0.01f;
    constexpr float kDefaultGrabThumbSurfaceSafetyMarginGameUnits = 1.0f;
    constexpr float kDefaultNearCastRadiusGameUnits = 3.5f;
    constexpr float kDefaultNearCastDistanceGameUnits = 7.0f;
    const RE::NiPoint3 kDefaultPalmNormalHandspace{ 0.0f, 1.0f, 0.0f };
    constexpr bool kDefaultSeeThroughScopesRightEyeDominant = true;

    std::string resolveIniPath()
    {
        char documents[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(nullptr, CSIDL_MYDOCUMENTS, nullptr, 0, documents))) {
            return std::string(documents) + R"(\My Games\Fallout4VR\ROCK_Config\ROCK.ini)";
        }

        ROCK_LOG_WARN(Config, "SHGetFolderPath failed — using fallback ROCK.ini path");
        return R"(Data\F4SE\Plugins\ROCK.ini)";
    }

    float readClampedFloat(CSimpleIniA& ini, const char* section, const char* key, float currentValue, float fallback, float minValue, float maxValue)
    {
        float value = static_cast<float>(ini.GetDoubleValue(section, key, currentValue));
        if (!std::isfinite(value)) {
            ROCK_LOG_WARN(Config, "Invalid {}={} -- using {:.2f}", key, value, fallback);
            value = fallback;
        }
        return std::clamp(value, minValue, maxValue);
    }

    int readSelectionAimAngleDegrees(CSimpleIniA& ini, const char* section, const char* key, int currentValue)
    {
        const int configuredValue = static_cast<int>(ini.GetLongValue(section, key, currentValue));
        const int sanitizedValue = rock::selection_query_policy::sanitizeSelectionAimAngleDegrees(configuredValue);
        if (configuredValue != sanitizedValue) {
            ROCK_LOG_WARN(Config, "Invalid {}={} -- using {}", key, configuredValue, sanitizedValue);
        }
        return sanitizedValue;
    }
}

namespace rock
{

    void RockConfig::resetToDefaults()
    {
        rockEnabled = true;
        rockHavokTimingFixEnabled = true;
        rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::kDefaultMinPhysicsFrameRate;
        rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::kDefaultMaxSubsteps;

        rockInputRemapEnabled = true;
        rockRightWeaponReadyButtonID = 32;
        rockSuppressRightGrabGameInput = true;
        rockSuppressRightFavoritesGameInput = true;
        rockSuppressNativeReadyWeaponAutoReady = true;
        rockSuppressNativeMeleeThrowGameInput = true;
        rockVirtualHolstersCompatibilityEnabled = true;
        rockVirtualHolstersDeferGrabInZone = true;
        rockVirtualHolstersDeferWeaponToggleInZone = true;
        rockVirtualHolstersDeferOnlyMatchingButton = false;
        rockGrabInputIntentStateEnabled = true;
        rockGrabInputLeewaySeconds = 0.12f;
        rockGrabInputForceSeconds = 0.08f;

        rockLogLevel = logging_policy::DefaultLogLevel;
        rockLogPattern = logging_policy::DefaultLogPattern;
        rockLogSampleMilliseconds = logging_policy::DefaultLogSampleMilliseconds;
        rockPerformanceProfilerEnabled = false;
        rockPerformanceProfilerLogIntervalFrames = 300;
        rockPerformanceProfilerWarmupFrames = 120;
        rockPerformanceProfilerOverlayText = false;

        rockPalmNormalHandspace = kDefaultPalmNormalHandspace;
        rockPointingVectorHandspace = RE::NiPoint3(0.0f, 1.0f, 0.0f);
        rockReversePalmNormal = true;
        rockReverseFarGrabNormal = true;

        rockWeaponCollisionEnabled = true;
        rockWeaponCollisionBlocksProjectiles = false;
        rockWeaponCollisionBlocksSpells = false;
        rockWeaponCollisionStaticWorldEnabled = true;
        rockWeaponCollisionGroupingMode = weapon_collision_grouping_policy::kDefaultWeaponCollisionGroupingMode;
        rockWeaponCollisionVisualStabilizationFrames = kDefaultWeaponCollisionVisualStabilizationFrames;
        rockWeaponCollisionConvexRadius = 0.01f;
        rockWeaponCollisionPointDedupGrid = 0.002f;
        rockWeaponCollisionSupportFitTargetPoints = kDefaultWeaponCollisionSupportFitTargetPoints;
        rockWeaponCollisionSupportFitMaxErrorGameUnits = kDefaultWeaponCollisionSupportFitMaxErrorGameUnits;
        rockWeaponCollisionMaxLinearVelocity = 50.0f;
        rockWeaponCollisionMaxAngularVelocity = 100.0f;
        rockWeaponInteractionProbeRadius = 12.0f;
        rockVisualOnlySidearmSupportGripEnabled = true;
        rockWeaponSupportGripHandLerpEnabled = true;
        rockWeaponSupportGripHandLerpTimeMin = 0.12f;
        rockWeaponSupportGripHandLerpTimeMax = 0.20f;
        rockWeaponSupportGripHandLerpMinDistance = 1.0f;
        rockWeaponSupportGripHandLerpMaxDistance = 14.0f;
        rockSeeThroughScopesCompatibilityEnabled = true;
        rockSeeThroughScopesReticleAlignmentEnabled = true;
        rockSeeThroughScopesRightEyeDominant = kDefaultSeeThroughScopesRightEyeDominant;
        rockSeeThroughScopesEyeOffsetGameUnits = see_through_scopes_policy::kDefaultReticleEyeOffsetGameUnits;
        rockSeeThroughScopesReticleOffsetXGameUnits = see_through_scopes_policy::kDefaultReticleOffsetXGameUnits;
        rockSeeThroughScopesReticleOffsetZGameUnits = see_through_scopes_policy::kDefaultReticleOffsetZGameUnits;
        rockSeeThroughScopesLookDotThreshold = see_through_scopes_policy::kDefaultReticleLookDotThreshold;
        rockSeeThroughScopesDistanceThresholdGameUnits = see_through_scopes_policy::kDefaultReticleDistanceThresholdGameUnits;

        rockSoftContactWorldEnabled = true;
        rockSoftContactVisualPriority = 80;
        rockSoftContactWorldRadiusPaddingGameUnits = 1.5f;
        rockSoftContactWorldContactPaddingGameUnits = 0.35f;
        rockSoftContactWorldSkinGameUnits = 0.5f;
        rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits = 0.025f;
        rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits = 10.0f;
        rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits = 18.0f;
        rockSoftContactWorldMaxCorrectionGameUnits = 18.0f;
        rockSoftContactWorldReleaseLerpEnabled = true;
        rockSoftContactWorldReleaseLerpTimeMin = 0.06f;
        rockSoftContactWorldReleaseLerpTimeMax = 0.12f;
        rockSoftContactWorldReleaseLerpMinDistance = 0.5f;
        rockSoftContactWorldReleaseLerpMaxDistance = 18.0f;
        rockSoftContactWorldShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        rockSoftContactWorldHapticsEnabled = true;
        rockSoftContactWorldHapticDurationSeconds = 0.035f;
        rockSoftContactWorldHapticBaseIntensity = 0.18f;
        rockSoftContactWorldHapticMaxIntensity = 0.55f;
        rockSoftContactWorldHapticSpeedScale = 0.006f;
        rockSoftContactWorldHapticMinApproachSpeedGameUnits = 3.0f;
        rockSoftContactWorldHapticCooldownSeconds = 0.12f;

        rockNativeMeleeSuppressionEnabled = true;
        rockNativeMeleeFullSuppression = true;
        rockNativeMeleeSuppressWeaponSwing = true;
        rockNativeMeleeSuppressHitFrame = true;
        rockNativeMeleeDebugLogging = false;
        rockNativeCharacterControllerObjectContactFilterEnabled = true;

        rockHighlightEnabled = true;
        rockSelectionBeamEnabled = true;
        rockSelectionBeamSegmentSizeGameUnits = selection_beam_policy::kDefaultSegmentSizeGameUnits;
        rockSelectionBeamCurveLiftGameUnits = selection_beam_policy::kDefaultCurveLiftGameUnits;
        rockSelectionBeamAlpha = selection_beam_policy::kDefaultAlpha;

        rockDebugShowColliders = false;
        rockDebugShowTargetColliders = false;
        rockDebugShowHandAxes = false;
        rockDebugShowGrabPivots = false;
        rockDebugShowGrabPocketNormal = false;
        rockDebugDrawGrabContactPatch = false;
        rockDebugDrawGrabForceTorque = false;
        rockDebugDrawGrabForceTorqueText = false;
        rockDebugDrawGrabPivotSourceCollider = false;
        rockDebugDrawGrabPivotSourceEvidence = false;
        rockDebugDrawGrabSupportFrame = false;
        rockDebugDrawGrabPockets = false;
        rockDebugShowGrabFingerProbes = false;
        rockDebugShowPalmVectors = false;
        rockDebugDrawHandColliders = false;
        rockDebugDrawHandBoneColliders = false;
        rockDebugDrawHandBoneContacts = false;
        rockDebugDrawSoftContacts = false;
        rockDebugDrawGrabAuthorityProxy = false;
        rockDebugMaxHandBoneBodiesDrawn = 48;
        rockDebugMaxBodyBoneBodiesDrawn = 32;
        rockDebugDrawWeaponColliders = false;
        rockDebugDumpWeaponAnimNodes = false;
        rockDebugMaxWeaponBodiesDrawn = 100;
        rockDebugWeaponAnimNodeDumpIntervalFrames = 120;
        rockDebugMaxShapeGenerationsPerFrame = 100;
        rockDebugMaxConvexSupportVertices = 6;
        rockDebugUseBoundsForHeavyConvex = true;
        rockDebugContactTargetIdentityLogging = false;
        rockDebugContactTargetIdentitySampleMilliseconds = 500;
        rockDebugVerboseLogging = false;
        rockDebugGrabFrameLogging = false;
        rockDebugGrabTimelineTrace = false;
        rockDebugGrabTransformTelemetry = false;
        rockDebugGrabTransformTelemetryText = false;
        rockDebugGrabTransformTelemetryAxes = false;
        rockDebugGrabTimelineTraceIntervalFrames = 1;
        rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        rockDebugGrabTransformTelemetryTextMode = 0;
        rockDebugShowGrabNotifications = false;
        rockDebugShowWeaponNotifications = false;
        rockDebugHandTransformParity = false;
        rockDebugWorldObjectOriginDiagnostics = false;
        rockDebugWorldObjectOriginLogIntervalFrames = 120;
        rockDebugWorldObjectOriginMismatchWarnGameUnits = 5.0f;
        rockDebugCustomCalibrationOffset = false;
        rockDebugShowRootFlattenedFingerSkeletonMarkers = false;
        rockDebugShowSkeletonBoneVisualizer = false;
        rockDebugDrawSkeletonBoneAxes = false;
        rockDebugLogSkeletonBones = false;
        rockDebugSkeletonBoneMode = 1;
        rockDebugSkeletonBoneSource = 1;
        rockDebugMaxSkeletonBonesDrawn = 256;
        rockDebugMaxSkeletonBoneAxesDrawn = 80;
        rockDebugSkeletonBoneLogIntervalFrames = 120;
        rockDebugLogSkeletonBoneTruncation = false;
        rockDebugRootFlattenedFingerSkeletonMarkerSize = 1.4f;
        rockDebugSkeletonBonePointSize = 1.4f;
        rockDebugSkeletonBoneAxisLength = 4.0f;
        rockDebugSkeletonBoneLogFilter = "RArm_Hand,LArm_Hand,RArm_Finger23,LArm_Finger23,Chest,Pelvis";
        rockDebugSkeletonAxisBoneFilter = "";

        rockHandColliderRuntimeMode = 1;
        rockBodyBoneCollidersEnabled = true;
        rockBodyBoneCollisionStaticWorldEnabled = true;
        rockBodyBoneColliderStandardRadiusScale = 1.0f;
        rockBodyBoneColliderStandardLengthScale = 1.0f;
        rockBodyBoneColliderStandardConvexRadiusScale = 1.0f;
        rockBodyBoneColliderPowerArmorRadiusScale = 1.0f;
        rockBodyBoneColliderPowerArmorLengthScale = 1.0f;
        rockBodyBoneColliderPowerArmorConvexRadiusScale = 1.0f;
        rockBodyBoneColliderTorsoRadiusScale = 1.0f;
        rockBodyBoneColliderArmRadiusScale = 1.0f;
        rockBodyBoneColliderLegRadiusScale = 1.0f;
        rockBodyBoneColliderFootRadiusScale = 1.0f;
        rockBodyBoneColliderTorsoLengthScale = 1.0f;
        rockBodyBoneColliderArmLengthScale = 1.0f;
        rockBodyBoneColliderLegLengthScale = 1.0f;
        rockBodyBoneColliderFootLengthScale = 1.0f;
        rockBodyBoneColliderZoneScaleOverrides = "";
        rockBodyBoneColliderRadiusScaleOverrides = "";
        rockHandCollisionStaticWorldEnabled = true;
        rockHandBoneColliderRadiusScaleOverrides = "";
        rockHandPalmColliderDimensionScaleOverrides = "";
        rockHandBoneCollidersRequirePalmAnchor = true;
        rockHandBoneCollidersRequireAllFingerBones = true;
        rockHandBoneColliderMaxLinearVelocity = 200.0f;
        rockHandBoneColliderMaxAngularVelocity = 500.0f;

        rockNearDetectionRange = 25.0f;
        rockFarDetectionRange = 350.0f;
        rockNearCastRadiusGameUnits = kDefaultNearCastRadiusGameUnits;
        rockNearCastDistanceGameUnits = kDefaultNearCastDistanceGameUnits;
        rockFarCastRadiusGameUnits = 21.0f;
        rockCloseSelectionAngleDegrees = selection_query_policy::kDefaultSelectionAimAngleDegrees;
        rockFarSelectionAngleDegrees = selection_query_policy::kDefaultSelectionAimAngleDegrees;
        rockFarSelectionHmdConeEnabled = true;
        rockFarSelectionHmdConeHalfAngleDegrees = selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees;
        rockFarSelectionBlockedReferenceFormIds.clear();
        rockFarSelectionBlockedBaseFormIds.clear();
        rockFarSelectionBlockedFormTypes.clear();
        rockFarSelectionBlockedLayers.clear();
        rockCloseSelectionBehindPalmToleranceGameUnits = 2.0f;
        rockSelectionShapeCastFilterInfo = selection_query_policy::kDefaultShapeCastFilterInfo;
        rockFarClipRayFilterInfo = selection_query_policy::kDefaultFarClipRayFilterInfo;
        rockPullApplyVelocityTime = 0.2f;
        rockPullOwnerGraceSeconds = 1.0f;
        rockPullTrackHandTime = 0.1f;
        rockPullDestinationZOffsetHavok = 0.01f;
        rockPullDurationA = 0.715619f;
        rockPullDurationB = -0.415619f;
        rockPullDurationC = 0.656256f;
        rockPullMaxVelocityHavok = 10.0f;
        rockPullAutoGrabDistanceGameUnits = 18.0f;
        rockPullCatchRetryMaxTimeSeconds = 0.65f;
        rockPullCatchWideReacquireEnabled = true;
        rockPullCatchWideReacquireRadiusGameUnits = 32.0f;
        rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42.0f;
        rockObjectPhysicsTreeMaxDepth = 12;
        rockDynamicPushAssistEnabled = true;
        rockDynamicPushMinSpeed = 0.35f;
        rockDynamicPushMaxImpulse = 2.0f;
        rockDynamicPushCooldownSeconds = 0.08f;

        rockGrabLinearTau = 0.03f;
        rockGrabLinearDamping = 0.8f;
        rockGrabLinearProportionalRecovery = 2.0f;
        rockGrabLinearConstantRecovery = 1.0f;

        rockGrabAngularTau = 0.03f;
        rockGrabAngularDamping = 0.8f;
        rockGrabAngularProportionalRecovery = 2.0f;
        rockGrabAngularConstantRecovery = 1.0f;

        rockGrabConstraintMaxForce = 2000.0f;
        rockGrabMaxForceToMassRatio = 500.0f;
        rockGrabEffectiveMotorMassFloorEnabled = true;
        rockGrabEffectiveMotorMassFloor = kDefaultGrabEffectiveMotorMassFloor;
        rockGrabPhysicsRateForceScalingEnabled = true;
        rockGrabPhysicsRateReferenceHz = kDefaultGrabPhysicsRateReferenceHz;
        rockGrabPhysicsRateForceScaleExponent = kDefaultGrabPhysicsRateForceScaleExponent;
        rockGrabPhysicsRateMinForceScale = kDefaultGrabPhysicsRateMinForceScale;
        rockGrabPhysicsRateMaxForceScale = kDefaultGrabPhysicsRateMaxForceScale;

        rockGrabForceFadeInTime = 0.1f;
        rockGrabRagdollDecompositionMode = -1;
        rockRightGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockLeftGrabAuthorityProxyOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockRightCustomOGAOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockLeftCustomOGAOffsetGameUnits = RE::NiPoint3(0.0f, -2.0f, 0.0f);
        rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier;
        rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier;
        rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier;
        rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier;
        rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier;
        rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier;
        rockGrabTauMin = 0.01f;
        rockGrabTauLerpSpeed = 0.5f;
        rockGrabLongObjectAngularScalingEnabled = true;
        rockGrabLongObjectReferenceLeverGameUnits = kDefaultGrabLongObjectReferenceLeverGameUnits;
        rockGrabLongObjectMinAngularScale = kDefaultGrabLongObjectMinAngularScale;
        rockGrabPivotQualityAngularScalingEnabled = true;
        rockGrabPositionOnlyAngularScale = kDefaultGrabPositionOnlyAngularScale;
        rockGrabSmallObjectReferenceLeverGameUnits = kDefaultGrabSmallObjectReferenceLeverGameUnits;
        rockGrabSmallObjectAngularScale = kDefaultGrabSmallObjectAngularScale;
        rockGrabLowContactSupportAngularScale = kDefaultGrabLowContactSupportAngularScale;
        rockGrabMinAngularAuthorityScale = kDefaultGrabMinAngularAuthorityScale;
        rockGrabWeakPivotTwistScale = kDefaultGrabWeakPivotTwistScale;

        rockGrabMaxInertiaRatio = 10.0f;
        rockGrabMinInertia = kDefaultGrabMinInertia;

        rockGrabMaxDeviation = 50.0f;
        rockGrabMaxDeviationTime = 2.0f;
        rockGrabButtonID = 2;
        rockThrowVelocityMultiplier = 1.5f;
        rockGrabControllerDerivedThrowVelocityEnabled = true;
        rockGrabThrowObjectVelocityBlend = kDefaultGrabThrowObjectVelocityBlend;
        rockGrabThrowTangentialVelocityScale = kDefaultGrabThrowTangentialVelocityScale;
        rockGrabThrowMaxVelocityHavok = kDefaultGrabThrowMaxVelocityHavok;
        rockGrabThrowAngularVelocityScale = kDefaultGrabThrowAngularVelocityScale;
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond;
        rockGrabReleaseHandCollisionDelaySeconds = 0.10f;
        rockShoulderStashEnabled = true;
        rockShoulderStashUseBodyZoneColliders = true;
        rockShoulderStashUseHmdBackVolume = true;
        rockShoulderStashEnterPaddingGameUnits = 5.0f;
        rockShoulderStashExitPaddingGameUnits = 8.0f;
        rockShoulderStashMinDwellSeconds = 0.08f;
        rockShoulderStashMaxSpeedGameUnitsPerSecond = 140.0f;
        rockShoulderStashRecentContactFrames = 4;
        rockShoulderStashSustainedContactMissFrames = 18;
        rockShoulderStashHmdBackRightOffsetGameUnits = RE::NiPoint3(14.0f, -18.0f, -6.85f);
        rockShoulderStashHmdBackLeftOffsetGameUnits = RE::NiPoint3(-14.0f, -18.0f, -6.85f);
        rockShoulderStashHmdBackRadiusGameUnits = 11.0f;
        rockShoulderStashHmdBackEnterPaddingGameUnits = 0.0f;
        rockShoulderStashHmdBackExitPaddingGameUnits = 2.0f;
        rockShoulderStashHmdBackMinBehindGameUnits = 4.0f;
        rockShoulderStashShowCollectedNotifications = true;
        rockGrabVelocityDamping = 0.25f;
        rockGrabPlayerSpaceCompensation = true;
        rockGrabPlayerSpaceWarpDistance = 35.0f;
        rockGrabPlayerSpaceWarpMinRotationDegrees = 0.6f;
        rockGrabPlayerSpaceTransformWarpEnabled = true;
        rockGrabLocomotionAuthorityBridgeEnabled = true;
        rockGrabLocomotionAuthorityMaxLeadSeconds = grab_locomotion_authority_bridge::kDefaultMaxLeadSeconds;
        rockGrabLocomotionAuthoritySmoothingHz = grab_locomotion_authority_bridge::kDefaultSmoothingHz;
        rockGrabLocomotionAuthorityMaxOffsetGameUnits = grab_locomotion_authority_bridge::kDefaultMaxOffsetGameUnits;
        rockGrabLocomotionAuthorityResetDistanceGameUnits = grab_locomotion_authority_bridge::kDefaultResetDistanceGameUnits;
        rockGrabResidualVelocityDamping = true;
        rockGrabNearbyDampingEnabled = true;
        rockGrabNearbyDampingRadius = 90.0f;
        rockGrabNearbyDampingSeconds = 0.35f;
        rockGrabNearbyLinearDamping = 3.0f;
        rockGrabNearbyAngularDamping = 5.5f;
        rockGrabHeldMassMovementSlowdownEnabled = true;
        rockGrabHeldMassMovementMassProportion = 0.675f;
        rockGrabHeldMassMovementMassExponent = 1.0f;
        rockGrabHeldMassMovementMaxReduction = 75.0f;
        rockGrabHeldMassMovementFadeOutSeconds = 5.0f;
        rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        rockGrabNearConvergeDistanceGameUnits = 28.0f;
        rockGrabPocketDepthGameUnits = 7.0f;
        rockGrabPocketRadiusGameUnits = 9.0f;
        rockGrabGripInsetGameUnits = 2.0f;
        rockGrabGripMaxInsetGameUnits = 6.0f;
        rockGrabConvergeMaxTimeSeconds = 0.35f;
        rockGrabConvergeStableFrames = 3;
        rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        rockGrabMultiFingerContactValidationEnabled = true;
        rockGrabContactQualityMode = 1;
        rockGrabMinFingerContactGroups = 3;
        rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
        rockGrabOppositionContactMaxAgeFrames = 5;
        rockGrabPinchPocketEnabled = true;
        rockGrabPinchCloseSelectionEnabled = true;
        rockGrabPinchCompactMaxExtentGameUnits = grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits;
        rockGrabPinchThinRodMaxLengthGameUnits = grab_pinch_pocket_policy::kDefaultThinRodMaxLengthGameUnits;
        rockGrabPinchThinRodMaxCrossSectionGameUnits = grab_pinch_pocket_policy::kDefaultThinRodMaxCrossSectionGameUnits;
        rockGrabPinchMaxPocketDistanceGameUnits = grab_pinch_pocket_policy::kDefaultMaxPocketDistanceGameUnits;
        rockGrabPinchMinFingerGapGameUnits = grab_pinch_pocket_policy::kDefaultMinFingerGapGameUnits;
        rockGrabPinchMaxFingerGapGameUnits = grab_pinch_pocket_policy::kDefaultMaxFingerGapGameUnits;
        rockGrabPinchThumbIndexMaxOpenValue = grab_pinch_pocket_policy::kDefaultThumbIndexMaxOpenValue;
        rockGrabPinchOtherFingerCurlValue = grab_pinch_pocket_policy::kDefaultOtherFingerCurlValue;
        rockGrabPinchSurfaceInsetGameUnits = grab_pinch_pocket_policy::kDefaultSurfaceInsetGameUnits;
        rockGrabPinchDetectionDirectionHandspace = RE::NiPoint3(grab_pinch_pocket_policy::kDefaultDetectionDirectionHandspaceX,
            grab_pinch_pocket_policy::kDefaultDetectionDirectionHandspaceY,
            grab_pinch_pocket_policy::kDefaultDetectionDirectionHandspaceZ);
        rockGrabPinchDetectionAxisBlend = grab_pinch_pocket_policy::kDefaultDetectionAxisBlend;
        rockGrabHandLerpEnabled = true;
        rockGrabHandLerpTimeMin = 0.10f;
        rockGrabHandLerpTimeMax = 0.20f;
        rockGrabHandLerpMinDistance = 7.0f;
        rockGrabHandLerpMaxDistance = 14.0f;
        rockGrabMeshFingerPoseEnabled = true;
        rockGrabMeshJointPoseEnabled = true;
        rockGrabFingerPoseUpdateInterval = 3;
        rockGrabFingerMinValue = 0.2f;
        rockGrabFingerPoseSmoothingSpeed = 14.0f;
        rockGrabMeshLocalTransformPoseEnabled = true;
        rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        rockGrabFingerSurfaceAimStrength = 0.75f;
        rockGrabFingerRejectBacksideHits = true;
        rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        rockGrabThumbOppositionStrength = 1.0f;
        rockGrabThumbAlternateCurveStrength = 0.65f;
        rockGrabThumbSurfaceSafetyEnabled = true;
        rockGrabThumbSurfaceSafetyMarginGameUnits = kDefaultGrabThumbSurfaceSafetyMarginGameUnits;
        rockGrabLateralWeight = 0.6f;
        rockGrabDirectionalWeight = 0.4f;
        rockGrabMaxTriangleDistance = 100.0f;
        rockGrabMeshContactOnly = true;
        rockGrabRequireMeshContact = true;
        rockGrabContactPatchEnabled = true;
        rockGrabContactPatchProbeCount = 9;
        rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        rockGrabContactPatchMaxNormalAngleDegrees = 35.0f;
        rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        rockGrabNodeAnchorsEnabled = true;
        rockGrabNodeRejectOppositeHandAnchor = true;
        rockPrintGrabNodeInfo = false;
        rockGrabNodeNameRight = grab_node_name_policy::defaultGrabNodeName(false);
        rockGrabNodeNameLeft = grab_node_name_policy::defaultGrabNodeName(true);
        rockGrabNodeNameBlacklist = std::string(grab_node_name_policy::kDefaultGrabNodeNameBlacklist);
        rockSelectedCloseFingerCurlEnabled = true;
        rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        rockSelectedCloseFingerAnimValue = 0.9f;
        rockPulledAngularDamping = 8.0f;
        rockPulledGrabHandAdjustDistanceGameUnits = 10.5f;

        rockRightGrabLegacyPalmPivotAHandspace = RE::NiPoint3(6.0f, -2.0f, 0.2f);
        rockLeftGrabLegacyPalmPivotAHandspace = RE::NiPoint3(6.0f, -2.0f, -0.2f);

        rockGrabHapticsEnabled = true;
        rockGrabHapticDurationSeconds = 0.055f;
        rockGrabHapticBaseIntensity = 0.12f;
        rockGrabHapticMaxIntensity = 0.80f;
        rockGrabHapticMassScale = 0.06f;
        rockGrabHapticMassExponent = 0.60f;
        rockPullStartHapticIntensity = 0.18f;
        rockPullCatchHapticIntensity = 0.22f;
        rockSelectionLockHapticIntensity = 0.15f;
        rockSelectionLockReleaseHapticIntensity = 0.10f;
        rockSelectionLockReleaseHapticDurationSeconds = 0.02f;
        rockHeldImpactHapticsEnabled = true;
        rockHeldImpactHapticDurationSeconds = 0.035f;
        rockHeldImpactHapticBaseIntensity = 0.12f;
        rockHeldImpactHapticMaxIntensity = 0.85f;
        rockHeldImpactHapticSpeedScale = 0.006f;
        rockHeldImpactHapticMassScale = 0.035f;
        rockHeldImpactHapticMassExponent = 0.55f;
        rockHeldImpactHapticMinSpeedGameUnits = 8.0f;
        rockHeldImpactHapticCooldownSeconds = 0.12f;
        rockHeldImpactHapticDampedMultiplier = 0.55f;
        rockShoulderStashHapticsEnabled = true;
        rockShoulderStashCandidateHapticDurationSeconds = 0.075f;
        rockShoulderStashCandidateHapticBaseIntensity = 0.20f;
        rockShoulderStashCandidateHapticIntensity = 0.42f;
        rockShoulderStashCandidateHapticIntervalSeconds = 0.075f;
        rockShoulderStashCommitHapticDurationSeconds = 0.12f;
        rockShoulderStashCommitHapticIntensity = 0.85f;

    }

    void RockConfig::readValuesFromIni(CSimpleIniA& ini)
    {
        auto readVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            value.x = static_cast<float>(ini.GetDoubleValue(SECTION, keyX, value.x));
            value.y = static_cast<float>(ini.GetDoubleValue(SECTION, keyY, value.y));
            value.z = static_cast<float>(ini.GetDoubleValue(SECTION, keyZ, value.z));
        };
        auto readOptionalVec3 = [&](const char* keyX, const char* keyY, const char* keyZ, RE::NiPoint3& value) {
            const bool hasAny = ini.GetValue(SECTION, keyX, nullptr) || ini.GetValue(SECTION, keyY, nullptr) || ini.GetValue(SECTION, keyZ, nullptr);
            if (!hasAny) {
                return false;
            }

            readVec3(keyX, keyY, keyZ, value);
            return true;
        };
        auto readHexFilter = [&](const char* key, std::uint32_t currentValue, std::uint32_t fallback) {
            char hexBuf[16] = {};
            snprintf(hexBuf, sizeof(hexBuf), "%08X", currentValue);
            const char* hexStr = ini.GetValue(SECTION, key, hexBuf);
            if (!hexStr || !hexStr[0]) {
                return selection_query_policy::sanitizeFilterInfo(currentValue, fallback);
            }

            return selection_query_policy::sanitizeFilterInfo(static_cast<std::uint32_t>(std::strtoul(hexStr, nullptr, 16)), fallback);
        };
        rockLogLevel = logging_policy::clampLogLevel(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogLevel", rockLogLevel)));
        rockLogPattern = ini.GetValue(DEBUG_SECTION, "sLogPattern", rockLogPattern.c_str());
        if (rockLogPattern.empty()) {
            rockLogPattern = logging_policy::DefaultLogPattern;
        }
        rockLogSampleMilliseconds =
            logging_policy::sanitizeSampleMilliseconds(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iLogSampleMilliseconds", rockLogSampleMilliseconds)));
        rockPerformanceProfilerEnabled = ini.GetBoolValue(DEBUG_SECTION, "bPerformanceProfilerEnabled", rockPerformanceProfilerEnabled);
        rockPerformanceProfilerLogIntervalFrames =
            std::clamp(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iPerformanceProfilerLogIntervalFrames", rockPerformanceProfilerLogIntervalFrames)), 30, 54000);
        rockPerformanceProfilerWarmupFrames =
            std::clamp(static_cast<int>(ini.GetLongValue(DEBUG_SECTION, "iPerformanceProfilerWarmupFrames", rockPerformanceProfilerWarmupFrames)), 0, 54000);
        rockPerformanceProfilerOverlayText = ini.GetBoolValue(DEBUG_SECTION, "bPerformanceProfilerOverlayText", rockPerformanceProfilerOverlayText);
        logger::setLogLevelAndPattern(rockLogLevel, rockLogPattern);

        rockEnabled = ini.GetBoolValue(SECTION, "bEnabled", rockEnabled);
        rockHavokTimingFixEnabled = ini.GetBoolValue(SECTION, "bHavokTimingFixEnabled", rockHavokTimingFixEnabled);
        rockHavokTimingFixMinPhysicsFrameRate = havok_timing_fix_policy::sanitizeMinPhysicsFrameRate(
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHavokTimingFixMinPhysicsFrameRate", rockHavokTimingFixMinPhysicsFrameRate)));
        rockHavokTimingFixMaxSubsteps = havok_timing_fix_policy::sanitizeMaxSubsteps(
            static_cast<int>(ini.GetLongValue(SECTION, "iHavokTimingFixMaxSubsteps", rockHavokTimingFixMaxSubsteps)));
        rockInputRemapEnabled = ini.GetBoolValue(SECTION, "bInputRemapEnabled", rockInputRemapEnabled);
        rockRightWeaponReadyButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iRightWeaponReadyButtonID", rockRightWeaponReadyButtonID));
        if (!input_remap_policy::isValidButtonId(rockRightWeaponReadyButtonID)) {
            ROCK_LOG_WARN(Config, "iRightWeaponReadyButtonID must be 0..63; using 32");
            rockRightWeaponReadyButtonID = 32;
        }
        rockSuppressRightGrabGameInput = ini.GetBoolValue(SECTION, "bSuppressRightGrabGameInput", rockSuppressRightGrabGameInput);
        rockSuppressRightFavoritesGameInput = ini.GetBoolValue(SECTION, "bSuppressRightFavoritesGameInput", rockSuppressRightFavoritesGameInput);
        rockSuppressNativeReadyWeaponAutoReady = ini.GetBoolValue(SECTION, "bSuppressNativeReadyWeaponAutoReady", rockSuppressNativeReadyWeaponAutoReady);
        rockSuppressNativeMeleeThrowGameInput = ini.GetBoolValue(SECTION, "bSuppressNativeMeleeThrowGameInput", rockSuppressNativeMeleeThrowGameInput);
        rockVirtualHolstersCompatibilityEnabled = ini.GetBoolValue(SECTION, "bVirtualHolstersCompatibilityEnabled", rockVirtualHolstersCompatibilityEnabled);
        rockVirtualHolstersDeferGrabInZone = ini.GetBoolValue(SECTION, "bVirtualHolstersDeferGrabInZone", rockVirtualHolstersDeferGrabInZone);
        rockVirtualHolstersDeferWeaponToggleInZone = ini.GetBoolValue(SECTION, "bVirtualHolstersDeferWeaponToggleInZone", rockVirtualHolstersDeferWeaponToggleInZone);
        rockVirtualHolstersDeferOnlyMatchingButton = ini.GetBoolValue(SECTION, "bVirtualHolstersDeferOnlyMatchingButton", rockVirtualHolstersDeferOnlyMatchingButton);
        rockGrabInputIntentStateEnabled = ini.GetBoolValue(SECTION, "bGrabInputIntentStateEnabled", rockGrabInputIntentStateEnabled);
        rockGrabInputLeewaySeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabInputLeewaySeconds", rockGrabInputLeewaySeconds));
        if (!std::isfinite(rockGrabInputLeewaySeconds) || rockGrabInputLeewaySeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabInputLeewaySeconds={} -- using 0.12", rockGrabInputLeewaySeconds);
            rockGrabInputLeewaySeconds = 0.12f;
        }
        rockGrabInputLeewaySeconds = std::clamp(rockGrabInputLeewaySeconds, 0.0f, 0.5f);
        rockGrabInputForceSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabInputForceSeconds", rockGrabInputForceSeconds));
        if (!std::isfinite(rockGrabInputForceSeconds) || rockGrabInputForceSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabInputForceSeconds={} -- using 0.08", rockGrabInputForceSeconds);
            rockGrabInputForceSeconds = 0.08f;
        }
        rockGrabInputForceSeconds = std::clamp(rockGrabInputForceSeconds, 0.0f, 0.3f);

        readVec3("fPalmNormalHandspaceX", "fPalmNormalHandspaceY", "fPalmNormalHandspaceZ", rockPalmNormalHandspace);
        readVec3("fPointingVectorHandspaceX", "fPointingVectorHandspaceY", "fPointingVectorHandspaceZ", rockPointingVectorHandspace);
        rockReversePalmNormal = ini.GetBoolValue(SECTION, "bReversePalmNormal", rockReversePalmNormal);
        rockReverseFarGrabNormal = ini.GetBoolValue(SECTION, "bReverseFarGrabNormal", rockReverseFarGrabNormal);

        rockWeaponCollisionEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionEnabled", rockWeaponCollisionEnabled);
        rockWeaponCollisionBlocksProjectiles = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksProjectiles", rockWeaponCollisionBlocksProjectiles);
        rockWeaponCollisionBlocksSpells = ini.GetBoolValue(SECTION, "bWeaponCollisionBlocksSpells", rockWeaponCollisionBlocksSpells);
        rockWeaponCollisionStaticWorldEnabled = ini.GetBoolValue(SECTION, "bWeaponCollisionStaticWorldEnabled", rockWeaponCollisionStaticWorldEnabled);
        rockWeaponCollisionGroupingMode = static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionGroupingMode", rockWeaponCollisionGroupingMode));
        const auto sanitizedWeaponCollisionGroupingMode = weapon_collision_grouping_policy::sanitizeWeaponCollisionGroupingMode(rockWeaponCollisionGroupingMode);
        if (static_cast<int>(sanitizedWeaponCollisionGroupingMode) != rockWeaponCollisionGroupingMode) {
            ROCK_LOG_WARN(Config,
                "Unsupported iWeaponCollisionGroupingMode={} - using {}",
                rockWeaponCollisionGroupingMode,
                weapon_collision_grouping_policy::weaponCollisionGroupingModeName(sanitizedWeaponCollisionGroupingMode));
            rockWeaponCollisionGroupingMode = static_cast<int>(sanitizedWeaponCollisionGroupingMode);
        }
        rockWeaponCollisionVisualStabilizationFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionVisualStabilizationFrames", rockWeaponCollisionVisualStabilizationFrames));
        if (rockWeaponCollisionVisualStabilizationFrames < 0 ||
            rockWeaponCollisionVisualStabilizationFrames > kMaxWeaponCollisionVisualStabilizationFrames) {
            ROCK_LOG_WARN(Config,
                "Invalid iWeaponCollisionVisualStabilizationFrames={} - using {}",
                rockWeaponCollisionVisualStabilizationFrames,
                kDefaultWeaponCollisionVisualStabilizationFrames);
            rockWeaponCollisionVisualStabilizationFrames = kDefaultWeaponCollisionVisualStabilizationFrames;
        }
        rockWeaponCollisionConvexRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionConvexRadius", rockWeaponCollisionConvexRadius));
        rockWeaponCollisionPointDedupGrid = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionPointDedupGrid", rockWeaponCollisionPointDedupGrid));
        rockWeaponCollisionSupportFitTargetPoints =
            static_cast<int>(ini.GetLongValue(SECTION, "iWeaponCollisionSupportFitTargetPoints", rockWeaponCollisionSupportFitTargetPoints));
        if (rockWeaponCollisionSupportFitTargetPoints < kMinWeaponCollisionSupportFitTargetPoints ||
            rockWeaponCollisionSupportFitTargetPoints > kMaxWeaponCollisionSupportFitTargetPoints) {
            ROCK_LOG_WARN(Config,
                "Invalid iWeaponCollisionSupportFitTargetPoints={} - using {}",
                rockWeaponCollisionSupportFitTargetPoints,
                kDefaultWeaponCollisionSupportFitTargetPoints);
            rockWeaponCollisionSupportFitTargetPoints = kDefaultWeaponCollisionSupportFitTargetPoints;
        }
        rockWeaponCollisionSupportFitMaxErrorGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionSupportFitMaxErrorGameUnits", rockWeaponCollisionSupportFitMaxErrorGameUnits));
        if (!std::isfinite(rockWeaponCollisionSupportFitMaxErrorGameUnits) || rockWeaponCollisionSupportFitMaxErrorGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config,
                "Invalid fWeaponCollisionSupportFitMaxErrorGameUnits={} - using {:.2f}",
                rockWeaponCollisionSupportFitMaxErrorGameUnits,
                kDefaultWeaponCollisionSupportFitMaxErrorGameUnits);
            rockWeaponCollisionSupportFitMaxErrorGameUnits = kDefaultWeaponCollisionSupportFitMaxErrorGameUnits;
        }
        rockWeaponCollisionMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxLinearVelocity", rockWeaponCollisionMaxLinearVelocity));
        rockWeaponCollisionMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponCollisionMaxAngularVelocity", rockWeaponCollisionMaxAngularVelocity));
        rockWeaponInteractionProbeRadius = static_cast<float>(ini.GetDoubleValue(SECTION, "fWeaponInteractionProbeRadius", rockWeaponInteractionProbeRadius));
        rockVisualOnlySidearmSupportGripEnabled =
            ini.GetBoolValue(SECTION, "bVisualOnlySidearmSupportGripEnabled", rockVisualOnlySidearmSupportGripEnabled);
        rockWeaponSupportGripHandLerpEnabled = ini.GetBoolValue(SECTION, "bWeaponSupportGripHandLerpEnabled", rockWeaponSupportGripHandLerpEnabled);
        rockWeaponSupportGripHandLerpTimeMin = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpTimeMin",
            rockWeaponSupportGripHandLerpTimeMin,
            0.12f,
            0.0f,
            1.0f);
        rockWeaponSupportGripHandLerpTimeMax = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpTimeMax",
            rockWeaponSupportGripHandLerpTimeMax,
            0.20f,
            rockWeaponSupportGripHandLerpTimeMin,
            1.0f);
        rockWeaponSupportGripHandLerpMinDistance = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpMinDistance",
            rockWeaponSupportGripHandLerpMinDistance,
            1.0f,
            0.0f,
            80.0f);
        rockWeaponSupportGripHandLerpMaxDistance = readClampedFloat(ini,
            SECTION,
            "fWeaponSupportGripHandLerpMaxDistance",
            rockWeaponSupportGripHandLerpMaxDistance,
            14.0f,
            rockWeaponSupportGripHandLerpMinDistance,
            120.0f);
        rockSeeThroughScopesCompatibilityEnabled =
            ini.GetBoolValue(SECTION, "bSeeThroughScopesCompatibilityEnabled", rockSeeThroughScopesCompatibilityEnabled);
        rockSeeThroughScopesReticleAlignmentEnabled =
            ini.GetBoolValue(SECTION, "bSeeThroughScopesReticleAlignmentEnabled", rockSeeThroughScopesReticleAlignmentEnabled);
        rockSeeThroughScopesRightEyeDominant =
            ini.GetBoolValue(SECTION, "bSeeThroughScopesRightEyeDominant", rockSeeThroughScopesRightEyeDominant);
        rockSeeThroughScopesEyeOffsetGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSeeThroughScopesEyeOffsetGameUnits", rockSeeThroughScopesEyeOffsetGameUnits));
        if (!std::isfinite(rockSeeThroughScopesEyeOffsetGameUnits) || rockSeeThroughScopesEyeOffsetGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config,
                "Invalid fSeeThroughScopesEyeOffsetGameUnits={} - using {:.2f}",
                rockSeeThroughScopesEyeOffsetGameUnits,
                see_through_scopes_policy::kDefaultReticleEyeOffsetGameUnits);
            rockSeeThroughScopesEyeOffsetGameUnits = see_through_scopes_policy::kDefaultReticleEyeOffsetGameUnits;
        }
        rockSeeThroughScopesEyeOffsetGameUnits = std::clamp(rockSeeThroughScopesEyeOffsetGameUnits, 0.0f, 10.0f);
        rockSeeThroughScopesReticleOffsetXGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSeeThroughScopesReticleOffsetXGameUnits", rockSeeThroughScopesReticleOffsetXGameUnits));
        if (!std::isfinite(rockSeeThroughScopesReticleOffsetXGameUnits)) {
            ROCK_LOG_WARN(Config,
                "Invalid fSeeThroughScopesReticleOffsetXGameUnits={} - using {:.6f}",
                rockSeeThroughScopesReticleOffsetXGameUnits,
                see_through_scopes_policy::kDefaultReticleOffsetXGameUnits);
            rockSeeThroughScopesReticleOffsetXGameUnits = see_through_scopes_policy::kDefaultReticleOffsetXGameUnits;
        }
        rockSeeThroughScopesReticleOffsetXGameUnits = std::clamp(rockSeeThroughScopesReticleOffsetXGameUnits, -10.0f, 10.0f);
        rockSeeThroughScopesReticleOffsetZGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSeeThroughScopesReticleOffsetZGameUnits", rockSeeThroughScopesReticleOffsetZGameUnits));
        if (!std::isfinite(rockSeeThroughScopesReticleOffsetZGameUnits)) {
            ROCK_LOG_WARN(Config,
                "Invalid fSeeThroughScopesReticleOffsetZGameUnits={} - using {:.6f}",
                rockSeeThroughScopesReticleOffsetZGameUnits,
                see_through_scopes_policy::kDefaultReticleOffsetZGameUnits);
            rockSeeThroughScopesReticleOffsetZGameUnits = see_through_scopes_policy::kDefaultReticleOffsetZGameUnits;
        }
        rockSeeThroughScopesReticleOffsetZGameUnits = std::clamp(rockSeeThroughScopesReticleOffsetZGameUnits, -10.0f, 10.0f);
        rockSeeThroughScopesLookDotThreshold =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSeeThroughScopesLookDotThreshold", rockSeeThroughScopesLookDotThreshold));
        if (!std::isfinite(rockSeeThroughScopesLookDotThreshold)) {
            ROCK_LOG_WARN(Config,
                "Invalid fSeeThroughScopesLookDotThreshold={} - using {:.2f}",
                rockSeeThroughScopesLookDotThreshold,
                see_through_scopes_policy::kDefaultReticleLookDotThreshold);
            rockSeeThroughScopesLookDotThreshold = see_through_scopes_policy::kDefaultReticleLookDotThreshold;
        }
        rockSeeThroughScopesLookDotThreshold = std::clamp(rockSeeThroughScopesLookDotThreshold, 0.0f, 1.0f);
        rockSeeThroughScopesDistanceThresholdGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSeeThroughScopesDistanceThresholdGameUnits", rockSeeThroughScopesDistanceThresholdGameUnits));
        if (!std::isfinite(rockSeeThroughScopesDistanceThresholdGameUnits) || rockSeeThroughScopesDistanceThresholdGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config,
                "Invalid fSeeThroughScopesDistanceThresholdGameUnits={} - using {:.2f}",
                rockSeeThroughScopesDistanceThresholdGameUnits,
                see_through_scopes_policy::kDefaultReticleDistanceThresholdGameUnits);
            rockSeeThroughScopesDistanceThresholdGameUnits = see_through_scopes_policy::kDefaultReticleDistanceThresholdGameUnits;
        }
        rockSeeThroughScopesDistanceThresholdGameUnits = std::clamp(rockSeeThroughScopesDistanceThresholdGameUnits, 1.0f, 100.0f);

        rockSoftContactWorldEnabled = ini.GetBoolValue(SECTION, "bSoftContactWorldEnabled", rockSoftContactWorldEnabled);
        rockSoftContactVisualPriority = static_cast<int>(ini.GetLongValue(SECTION, "iSoftContactVisualPriority", rockSoftContactVisualPriority));
        rockSoftContactVisualPriority = std::clamp(rockSoftContactVisualPriority, 0, 99);
        rockSoftContactWorldRadiusPaddingGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldRadiusPaddingGameUnits", rockSoftContactWorldRadiusPaddingGameUnits));
        if (!std::isfinite(rockSoftContactWorldRadiusPaddingGameUnits) || rockSoftContactWorldRadiusPaddingGameUnits < 0.0f) {
            rockSoftContactWorldRadiusPaddingGameUnits = 1.5f;
        }
        rockSoftContactWorldContactPaddingGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldContactPaddingGameUnits", rockSoftContactWorldContactPaddingGameUnits));
        if (!std::isfinite(rockSoftContactWorldContactPaddingGameUnits) || rockSoftContactWorldContactPaddingGameUnits < 0.0f) {
            rockSoftContactWorldContactPaddingGameUnits = 0.35f;
        }
        rockSoftContactWorldSkinGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldSkinGameUnits", rockSoftContactWorldSkinGameUnits));
        if (!std::isfinite(rockSoftContactWorldSkinGameUnits) || rockSoftContactWorldSkinGameUnits < 0.0f) {
            rockSoftContactWorldSkinGameUnits = 0.5f;
        }
        rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION,
                "fSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits",
                rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits));
        if (!std::isfinite(rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits) ||
            rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits < 0.0f) {
            rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits = 0.025f;
        }
        rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION,
                "fSoftContactWorldCachedPlaneMaxTangentDriftGameUnits",
                rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits));
        if (!std::isfinite(rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits) || rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits <= 0.0f) {
            rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits = 10.0f;
        }
        rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION,
                "fSoftContactWorldCachedPlaneMaxClearDistanceGameUnits",
                rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits));
        if (!std::isfinite(rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits) || rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits <= 0.0f) {
            rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits = 18.0f;
        }
        rockSoftContactWorldMaxCorrectionGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldMaxCorrectionGameUnits", rockSoftContactWorldMaxCorrectionGameUnits));
        if (!std::isfinite(rockSoftContactWorldMaxCorrectionGameUnits) || rockSoftContactWorldMaxCorrectionGameUnits <= 0.0f) {
            rockSoftContactWorldMaxCorrectionGameUnits = 18.0f;
        }
        rockSoftContactWorldReleaseLerpEnabled = ini.GetBoolValue(SECTION, "bSoftContactWorldReleaseLerpEnabled", rockSoftContactWorldReleaseLerpEnabled);
        rockSoftContactWorldReleaseLerpTimeMin = readClampedFloat(ini,
            SECTION,
            "fSoftContactWorldReleaseLerpTimeMin",
            rockSoftContactWorldReleaseLerpTimeMin,
            0.06f,
            0.0f,
            0.5f);
        rockSoftContactWorldReleaseLerpTimeMax = readClampedFloat(ini,
            SECTION,
            "fSoftContactWorldReleaseLerpTimeMax",
            rockSoftContactWorldReleaseLerpTimeMax,
            0.12f,
            rockSoftContactWorldReleaseLerpTimeMin,
            0.5f);
        rockSoftContactWorldReleaseLerpMinDistance = readClampedFloat(ini,
            SECTION,
            "fSoftContactWorldReleaseLerpMinDistance",
            rockSoftContactWorldReleaseLerpMinDistance,
            0.5f,
            0.0f,
            100.0f);
        rockSoftContactWorldReleaseLerpMaxDistance = readClampedFloat(ini,
            SECTION,
            "fSoftContactWorldReleaseLerpMaxDistance",
            rockSoftContactWorldReleaseLerpMaxDistance,
            18.0f,
            rockSoftContactWorldReleaseLerpMinDistance,
            200.0f);
        rockSoftContactWorldShapeCastFilterInfo = readHexFilter(
            "sSoftContactWorldShapeCastFilterInfo",
            rockSoftContactWorldShapeCastFilterInfo,
            selection_query_policy::kDefaultShapeCastFilterInfo);
        rockSoftContactWorldHapticsEnabled = ini.GetBoolValue(SECTION, "bSoftContactWorldHapticsEnabled", rockSoftContactWorldHapticsEnabled);
        rockSoftContactWorldHapticDurationSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldHapticDurationSeconds", rockSoftContactWorldHapticDurationSeconds));
        if (!std::isfinite(rockSoftContactWorldHapticDurationSeconds) || rockSoftContactWorldHapticDurationSeconds < 0.0f) {
            rockSoftContactWorldHapticDurationSeconds = 0.035f;
        }
        rockSoftContactWorldHapticDurationSeconds = std::clamp(rockSoftContactWorldHapticDurationSeconds, 0.0f, 0.2f);
        rockSoftContactWorldHapticBaseIntensity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldHapticBaseIntensity", rockSoftContactWorldHapticBaseIntensity));
        rockSoftContactWorldHapticBaseIntensity =
            std::clamp(std::isfinite(rockSoftContactWorldHapticBaseIntensity) ? rockSoftContactWorldHapticBaseIntensity : 0.18f, 0.0f, 1.0f);
        rockSoftContactWorldHapticMaxIntensity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldHapticMaxIntensity", rockSoftContactWorldHapticMaxIntensity));
        rockSoftContactWorldHapticMaxIntensity =
            std::clamp(std::isfinite(rockSoftContactWorldHapticMaxIntensity) ? rockSoftContactWorldHapticMaxIntensity : 0.55f,
                rockSoftContactWorldHapticBaseIntensity,
                1.0f);
        rockSoftContactWorldHapticSpeedScale =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldHapticSpeedScale", rockSoftContactWorldHapticSpeedScale));
        if (!std::isfinite(rockSoftContactWorldHapticSpeedScale) || rockSoftContactWorldHapticSpeedScale < 0.0f) {
            rockSoftContactWorldHapticSpeedScale = 0.006f;
        }
        rockSoftContactWorldHapticMinApproachSpeedGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldHapticMinApproachSpeedGameUnits", rockSoftContactWorldHapticMinApproachSpeedGameUnits));
        if (!std::isfinite(rockSoftContactWorldHapticMinApproachSpeedGameUnits) || rockSoftContactWorldHapticMinApproachSpeedGameUnits < 0.0f) {
            rockSoftContactWorldHapticMinApproachSpeedGameUnits = 3.0f;
        }
        rockSoftContactWorldHapticCooldownSeconds =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSoftContactWorldHapticCooldownSeconds", rockSoftContactWorldHapticCooldownSeconds));
        if (!std::isfinite(rockSoftContactWorldHapticCooldownSeconds) || rockSoftContactWorldHapticCooldownSeconds < 0.0f) {
            rockSoftContactWorldHapticCooldownSeconds = 0.12f;
        }

        rockNativeMeleeSuppressionEnabled = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressionEnabled", rockNativeMeleeSuppressionEnabled);
        rockNativeMeleeFullSuppression = ini.GetBoolValue(SECTION, "bNativeMeleeFullSuppression", rockNativeMeleeFullSuppression);
        rockNativeMeleeSuppressWeaponSwing = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressWeaponSwing", rockNativeMeleeSuppressWeaponSwing);
        rockNativeMeleeSuppressHitFrame = ini.GetBoolValue(SECTION, "bNativeMeleeSuppressHitFrame", rockNativeMeleeSuppressHitFrame);
        rockNativeMeleeDebugLogging = ini.GetBoolValue(SECTION, "bNativeMeleeDebugLogging", rockNativeMeleeDebugLogging);
        rockNativeCharacterControllerObjectContactFilterEnabled = ini.GetBoolValue(
            SECTION, "bNativeCharacterControllerObjectContactFilterEnabled", rockNativeCharacterControllerObjectContactFilterEnabled);

        rockHighlightEnabled = ini.GetBoolValue(SECTION, "bHighlightEnabled", rockHighlightEnabled);
        rockSelectionBeamEnabled = ini.GetBoolValue(SECTION, "bSelectionBeamEnabled", rockSelectionBeamEnabled);
        rockSelectionBeamSegmentSizeGameUnits = readClampedFloat(ini,
            SECTION,
            "fSelectionBeamSegmentSizeGameUnits",
            rockSelectionBeamSegmentSizeGameUnits,
            selection_beam_policy::kDefaultSegmentSizeGameUnits,
            0.2f,
            6.0f);
        rockSelectionBeamCurveLiftGameUnits = readClampedFloat(ini,
            SECTION,
            "fSelectionBeamCurveLiftGameUnits",
            rockSelectionBeamCurveLiftGameUnits,
            selection_beam_policy::kDefaultCurveLiftGameUnits,
            0.0f,
            80.0f);
        rockSelectionBeamAlpha = readClampedFloat(ini,
            SECTION,
            "fSelectionBeamAlpha",
            rockSelectionBeamAlpha,
            selection_beam_policy::kDefaultAlpha,
            0.05f,
            1.0f);

        rockDebugShowColliders = ini.GetBoolValue(SECTION, "bDebugShowColliders", rockDebugShowColliders);
        rockDebugShowTargetColliders = ini.GetBoolValue(SECTION, "bDebugShowTargetColliders", rockDebugShowTargetColliders);
        rockDebugShowHandAxes = ini.GetBoolValue(SECTION, "bDebugShowHandAxes", rockDebugShowHandAxes);
        rockDebugShowGrabPivots = ini.GetBoolValue(SECTION, "bDebugShowGrabPivots", rockDebugShowGrabPivots);
        rockDebugShowGrabPocketNormal = ini.GetBoolValue(SECTION, "bDebugShowGrabPocketNormal", rockDebugShowGrabPocketNormal);
        rockDebugDrawGrabContactPatch = ini.GetBoolValue(SECTION, "bDebugDrawGrabContactPatch", rockDebugDrawGrabContactPatch);
        rockDebugDrawGrabForceTorque = ini.GetBoolValue(SECTION, "bDebugDrawGrabForceTorque", rockDebugDrawGrabForceTorque);
        rockDebugDrawGrabForceTorqueText = ini.GetBoolValue(SECTION, "bDebugDrawGrabForceTorqueText", rockDebugDrawGrabForceTorqueText);
        rockDebugDrawGrabPivotSourceCollider =
            ini.GetBoolValue(SECTION, "bDebugDrawGrabPivotSourceCollider", rockDebugDrawGrabPivotSourceCollider);
        rockDebugDrawGrabPivotSourceEvidence =
            ini.GetBoolValue(SECTION, "bDebugDrawGrabPivotSourceEvidence", rockDebugDrawGrabPivotSourceEvidence);
        rockDebugDrawGrabSupportFrame = ini.GetBoolValue(SECTION, "bDebugDrawGrabSupportFrame", rockDebugDrawGrabSupportFrame);
        rockDebugDrawGrabPockets = ini.GetBoolValue(SECTION, "bDebugDrawGrabPockets", rockDebugDrawGrabPockets);
        rockDebugShowGrabFingerProbes = ini.GetBoolValue(SECTION, "bDebugShowGrabFingerProbes", rockDebugShowGrabFingerProbes);
        rockDebugShowPalmVectors = ini.GetBoolValue(SECTION, "bDebugShowPalmVectors", rockDebugShowPalmVectors);
        rockDebugDrawHandColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandColliders", rockDebugDrawHandColliders);
        rockDebugDrawHandBoneColliders = ini.GetBoolValue(SECTION, "bDebugDrawHandBoneColliders", rockDebugDrawHandBoneColliders);
        rockDebugDrawHandBoneContacts = ini.GetBoolValue(SECTION, "bDebugDrawHandBoneContacts", rockDebugDrawHandBoneContacts);
        rockDebugDrawSoftContacts = ini.GetBoolValue(SECTION, "bDebugDrawSoftContacts", rockDebugDrawSoftContacts);
        rockDebugDrawGrabAuthorityProxy = ini.GetBoolValue(SECTION, "bDebugDrawGrabAuthorityProxy", rockDebugDrawGrabAuthorityProxy);
        rockDebugMaxHandBoneBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxHandBoneBodiesDrawn", rockDebugMaxHandBoneBodiesDrawn));
        if (rockDebugMaxHandBoneBodiesDrawn < 0) {
            rockDebugMaxHandBoneBodiesDrawn = 0;
        } else if (rockDebugMaxHandBoneBodiesDrawn > 48) {
            rockDebugMaxHandBoneBodiesDrawn = 48;
        }
        rockDebugMaxBodyBoneBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxBodyBoneBodiesDrawn", rockDebugMaxBodyBoneBodiesDrawn));
        if (rockDebugMaxBodyBoneBodiesDrawn < 0) {
            rockDebugMaxBodyBoneBodiesDrawn = 0;
        } else if (rockDebugMaxBodyBoneBodiesDrawn > 64) {
            rockDebugMaxBodyBoneBodiesDrawn = 64;
        }
        rockDebugDrawWeaponColliders = ini.GetBoolValue(SECTION, "bDebugDrawWeaponColliders", rockDebugDrawWeaponColliders);
        rockDebugDumpWeaponAnimNodes = ini.GetBoolValue(SECTION, "bDebugDumpWeaponAnimNodes", rockDebugDumpWeaponAnimNodes);
        rockDebugMaxWeaponBodiesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxWeaponBodiesDrawn", rockDebugMaxWeaponBodiesDrawn));
        rockDebugWeaponAnimNodeDumpIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugWeaponAnimNodeDumpIntervalFrames", rockDebugWeaponAnimNodeDumpIntervalFrames));
        if (rockDebugWeaponAnimNodeDumpIntervalFrames < 1) {
            rockDebugWeaponAnimNodeDumpIntervalFrames = 1;
        }
        rockDebugMaxShapeGenerationsPerFrame = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxShapeGenerationsPerFrame", rockDebugMaxShapeGenerationsPerFrame));
        rockDebugMaxConvexSupportVertices = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxConvexSupportVertices", rockDebugMaxConvexSupportVertices));
        rockDebugUseBoundsForHeavyConvex = ini.GetBoolValue(SECTION, "bDebugUseBoundsForHeavyConvex", rockDebugUseBoundsForHeavyConvex);
        rockDebugContactTargetIdentityLogging =
            ini.GetBoolValue(SECTION, "bDebugContactTargetIdentityLogging", rockDebugContactTargetIdentityLogging);
        rockDebugContactTargetIdentitySampleMilliseconds = logging_policy::sanitizeSampleMilliseconds(static_cast<int>(
            ini.GetLongValue(SECTION, "iDebugContactTargetIdentitySampleMilliseconds", rockDebugContactTargetIdentitySampleMilliseconds)));
        rockDebugVerboseLogging = ini.GetBoolValue(SECTION, "bDebugVerboseLogging", rockDebugVerboseLogging);
        rockDebugGrabFrameLogging = ini.GetBoolValue(SECTION, "bDebugGrabFrameLogging", rockDebugGrabFrameLogging);
        rockDebugGrabTimelineTrace = ini.GetBoolValue(SECTION, "bDebugGrabTimelineTrace", rockDebugGrabTimelineTrace);
        rockDebugGrabTimelineTraceIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugGrabTimelineTraceIntervalFrames", rockDebugGrabTimelineTraceIntervalFrames));
        if (rockDebugGrabTimelineTraceIntervalFrames < 1) {
            rockDebugGrabTimelineTraceIntervalFrames = 1;
        }
        rockDebugGrabTransformTelemetry = ini.GetBoolValue(SECTION, "bDebugGrabTransformTelemetry", rockDebugGrabTransformTelemetry);
        rockDebugGrabTransformTelemetryText = ini.GetBoolValue(SECTION, "bDebugGrabTransformTelemetryText", rockDebugGrabTransformTelemetryText);
        rockDebugGrabTransformTelemetryAxes = ini.GetBoolValue(SECTION, "bDebugGrabTransformTelemetryAxes", rockDebugGrabTransformTelemetryAxes);
        rockDebugGrabTransformTelemetryLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugGrabTransformTelemetryLogIntervalFrames", rockDebugGrabTransformTelemetryLogIntervalFrames));
        if (rockDebugGrabTransformTelemetryLogIntervalFrames < 1) {
            rockDebugGrabTransformTelemetryLogIntervalFrames = 1;
        }
        rockDebugGrabTransformTelemetryTextMode =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugGrabTransformTelemetryTextMode", rockDebugGrabTransformTelemetryTextMode));
        if (rockDebugGrabTransformTelemetryTextMode < 0 || rockDebugGrabTransformTelemetryTextMode > 1) {
            rockDebugGrabTransformTelemetryTextMode = 0;
        }
        rockDebugShowGrabNotifications = ini.GetBoolValue(SECTION, "bDebugShowGrabNotifications", rockDebugShowGrabNotifications);
        rockDebugShowWeaponNotifications = ini.GetBoolValue(SECTION, "bDebugShowWeaponNotifications", rockDebugShowWeaponNotifications);
        rockDebugHandTransformParity = ini.GetBoolValue(SECTION, "bDebugHandTransformParity", rockDebugHandTransformParity);
        rockDebugWorldObjectOriginDiagnostics =
            ini.GetBoolValue(SECTION, "bDebugWorldObjectOriginDiagnostics", rockDebugWorldObjectOriginDiagnostics);
        rockDebugWorldObjectOriginLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugWorldObjectOriginLogIntervalFrames", rockDebugWorldObjectOriginLogIntervalFrames));
        if (rockDebugWorldObjectOriginLogIntervalFrames < 1) {
            rockDebugWorldObjectOriginLogIntervalFrames = 1;
        }
        rockDebugWorldObjectOriginMismatchWarnGameUnits = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fDebugWorldObjectOriginMismatchWarnGameUnits", rockDebugWorldObjectOriginMismatchWarnGameUnits));
        if (!std::isfinite(rockDebugWorldObjectOriginMismatchWarnGameUnits) || rockDebugWorldObjectOriginMismatchWarnGameUnits < 0.0f) {
            rockDebugWorldObjectOriginMismatchWarnGameUnits = 0.0f;
        }
        rockDebugCustomCalibrationOffset = ini.GetBoolValue(SECTION, "customcalibrationoffset", rockDebugCustomCalibrationOffset);
        rockDebugShowRootFlattenedFingerSkeletonMarkers =
            ini.GetBoolValue(SECTION, "bDebugShowRootFlattenedFingerSkeletonMarkers", rockDebugShowRootFlattenedFingerSkeletonMarkers);
        rockDebugShowSkeletonBoneVisualizer = ini.GetBoolValue(SECTION, "bDebugShowSkeletonBoneVisualizer", rockDebugShowSkeletonBoneVisualizer);
        rockDebugSkeletonBoneMode = static_cast<int>(ini.GetLongValue(SECTION, "iDebugSkeletonBoneMode", rockDebugSkeletonBoneMode));
        if (rockDebugSkeletonBoneMode < 0 || rockDebugSkeletonBoneMode > 3) {
            rockDebugSkeletonBoneMode = 1;
        }
        rockDebugSkeletonBoneSource = static_cast<int>(ini.GetLongValue(SECTION, "iDebugSkeletonBoneSource", rockDebugSkeletonBoneSource));
        if (rockDebugSkeletonBoneSource != 1 && rockDebugSkeletonBoneSource != 2) {
            rockDebugSkeletonBoneSource = 1;
        }
        rockDebugDrawSkeletonBoneAxes = ini.GetBoolValue(SECTION, "bDebugDrawSkeletonBoneAxes", rockDebugDrawSkeletonBoneAxes);
        rockDebugLogSkeletonBones = ini.GetBoolValue(SECTION, "bDebugLogSkeletonBones", rockDebugLogSkeletonBones);
        rockDebugLogSkeletonBoneTruncation = ini.GetBoolValue(SECTION, "bDebugLogSkeletonBoneTruncation", rockDebugLogSkeletonBoneTruncation);
        rockDebugSkeletonBoneLogFilter = ini.GetValue(SECTION, "sDebugSkeletonBoneLogFilter", rockDebugSkeletonBoneLogFilter.c_str());
        rockDebugSkeletonAxisBoneFilter = ini.GetValue(SECTION, "sDebugSkeletonAxisBoneFilter", rockDebugSkeletonAxisBoneFilter.c_str());
        rockDebugSkeletonBoneLogIntervalFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iDebugSkeletonBoneLogIntervalFrames", rockDebugSkeletonBoneLogIntervalFrames));
        if (rockDebugSkeletonBoneLogIntervalFrames < 1) {
            rockDebugSkeletonBoneLogIntervalFrames = 1;
        }
        rockDebugMaxSkeletonBonesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxSkeletonBonesDrawn", rockDebugMaxSkeletonBonesDrawn));
        if (rockDebugMaxSkeletonBonesDrawn < 0) {
            rockDebugMaxSkeletonBonesDrawn = 0;
        } else if (rockDebugMaxSkeletonBonesDrawn > 768) {
            rockDebugMaxSkeletonBonesDrawn = 768;
        }
        rockDebugMaxSkeletonBoneAxesDrawn = static_cast<int>(ini.GetLongValue(SECTION, "iDebugMaxSkeletonBoneAxesDrawn", rockDebugMaxSkeletonBoneAxesDrawn));
        if (rockDebugMaxSkeletonBoneAxesDrawn < 0) {
            rockDebugMaxSkeletonBoneAxesDrawn = 0;
        } else if (rockDebugMaxSkeletonBoneAxesDrawn > 768) {
            rockDebugMaxSkeletonBoneAxesDrawn = 768;
        }
        rockDebugRootFlattenedFingerSkeletonMarkerSize =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugRootFlattenedFingerSkeletonMarkerSize", rockDebugRootFlattenedFingerSkeletonMarkerSize));
        if (rockDebugRootFlattenedFingerSkeletonMarkerSize < 0.1f) {
            rockDebugRootFlattenedFingerSkeletonMarkerSize = 0.1f;
        }
        rockDebugSkeletonBonePointSize = static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugSkeletonBonePointSize", rockDebugSkeletonBonePointSize));
        if (rockDebugSkeletonBonePointSize < 0.1f) {
            rockDebugSkeletonBonePointSize = 0.1f;
        }
        rockDebugSkeletonBoneAxisLength = static_cast<float>(ini.GetDoubleValue(SECTION, "fDebugSkeletonBoneAxisLength", rockDebugSkeletonBoneAxisLength));
        if (rockDebugSkeletonBoneAxisLength < 0.1f) {
            rockDebugSkeletonBoneAxisLength = 0.1f;
        }

        rockHandColliderRuntimeMode = static_cast<int>(ini.GetLongValue(SECTION, "iHandColliderRuntimeMode", rockHandColliderRuntimeMode));
        if (rockHandColliderRuntimeMode < 0 || rockHandColliderRuntimeMode > 1) {
            ROCK_LOG_WARN(Config, "Invalid iHandColliderRuntimeMode={} - using BoneDerivedHands", rockHandColliderRuntimeMode);
            rockHandColliderRuntimeMode = 1;
        }
        rockBodyBoneCollidersEnabled = ini.GetBoolValue(SECTION, "bBodyBoneCollidersEnabled", rockBodyBoneCollidersEnabled);
        rockBodyBoneCollisionStaticWorldEnabled = ini.GetBoolValue(SECTION, "bBodyBoneCollisionStaticWorldEnabled", rockBodyBoneCollisionStaticWorldEnabled);
        auto readBodyBoneScale = [&](const char* key, float currentValue) {
            const auto value = static_cast<float>(ini.GetDoubleValue(SECTION, key, currentValue));
            if (!std::isfinite(value)) {
                ROCK_LOG_WARN(Config, "Invalid {}={} - using 1.0", key, value);
                return 1.0f;
            }
            if (value < 0.05f || value > 8.0f) {
                const float clamped = std::clamp(value, 0.05f, 8.0f);
                ROCK_LOG_WARN(Config, "Clamped {} from {} to {}", key, value, clamped);
                return clamped;
            }
            return value;
        };
        rockBodyBoneColliderStandardRadiusScale = readBodyBoneScale("fBodyBoneColliderStandardRadiusScale", rockBodyBoneColliderStandardRadiusScale);
        rockBodyBoneColliderStandardLengthScale = readBodyBoneScale("fBodyBoneColliderStandardLengthScale", rockBodyBoneColliderStandardLengthScale);
        rockBodyBoneColliderStandardConvexRadiusScale =
            readBodyBoneScale("fBodyBoneColliderStandardConvexRadiusScale", rockBodyBoneColliderStandardConvexRadiusScale);
        rockBodyBoneColliderPowerArmorRadiusScale = readBodyBoneScale("fBodyBoneColliderPowerArmorRadiusScale", rockBodyBoneColliderPowerArmorRadiusScale);
        rockBodyBoneColliderPowerArmorLengthScale = readBodyBoneScale("fBodyBoneColliderPowerArmorLengthScale", rockBodyBoneColliderPowerArmorLengthScale);
        rockBodyBoneColliderPowerArmorConvexRadiusScale =
            readBodyBoneScale("fBodyBoneColliderPowerArmorConvexRadiusScale", rockBodyBoneColliderPowerArmorConvexRadiusScale);
        rockBodyBoneColliderTorsoRadiusScale = readBodyBoneScale("fBodyBoneColliderTorsoRadiusScale", rockBodyBoneColliderTorsoRadiusScale);
        rockBodyBoneColliderArmRadiusScale = readBodyBoneScale("fBodyBoneColliderArmRadiusScale", rockBodyBoneColliderArmRadiusScale);
        rockBodyBoneColliderLegRadiusScale = readBodyBoneScale("fBodyBoneColliderLegRadiusScale", rockBodyBoneColliderLegRadiusScale);
        rockBodyBoneColliderFootRadiusScale = readBodyBoneScale("fBodyBoneColliderFootRadiusScale", rockBodyBoneColliderFootRadiusScale);
        rockBodyBoneColliderTorsoLengthScale = readBodyBoneScale("fBodyBoneColliderTorsoLengthScale", rockBodyBoneColliderTorsoLengthScale);
        rockBodyBoneColliderArmLengthScale = readBodyBoneScale("fBodyBoneColliderArmLengthScale", rockBodyBoneColliderArmLengthScale);
        rockBodyBoneColliderLegLengthScale = readBodyBoneScale("fBodyBoneColliderLegLengthScale", rockBodyBoneColliderLegLengthScale);
        rockBodyBoneColliderFootLengthScale = readBodyBoneScale("fBodyBoneColliderFootLengthScale", rockBodyBoneColliderFootLengthScale);
        rockBodyBoneColliderZoneScaleOverrides = ini.GetValue(SECTION, "sBodyBoneColliderZoneScaleOverrides", rockBodyBoneColliderZoneScaleOverrides.c_str());
        rockBodyBoneColliderRadiusScaleOverrides = ini.GetValue(SECTION, "sBodyBoneColliderRadiusScaleOverrides", rockBodyBoneColliderRadiusScaleOverrides.c_str());
        rockHandCollisionStaticWorldEnabled = ini.GetBoolValue(SECTION, "bHandCollisionStaticWorldEnabled", rockHandCollisionStaticWorldEnabled);
        rockHandBoneColliderRadiusScaleOverrides = ini.GetValue(SECTION, "sHandBoneColliderRadiusScaleOverrides", rockHandBoneColliderRadiusScaleOverrides.c_str());
        rockHandPalmColliderDimensionScaleOverrides =
            ini.GetValue(SECTION, "sHandPalmColliderDimensionScaleOverrides", rockHandPalmColliderDimensionScaleOverrides.c_str());
        rockHandBoneCollidersRequirePalmAnchor = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequirePalmAnchor", rockHandBoneCollidersRequirePalmAnchor);
        rockHandBoneCollidersRequireAllFingerBones = ini.GetBoolValue(SECTION, "bHandBoneCollidersRequireAllFingerBones", rockHandBoneCollidersRequireAllFingerBones);
        rockHandBoneColliderMaxLinearVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHandBoneColliderMaxLinearVelocity", rockHandBoneColliderMaxLinearVelocity));
        rockHandBoneColliderMaxAngularVelocity =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fHandBoneColliderMaxAngularVelocity", rockHandBoneColliderMaxAngularVelocity));
        if (!std::isfinite(rockHandBoneColliderMaxLinearVelocity) || rockHandBoneColliderMaxLinearVelocity <= 0.0f) {
            rockHandBoneColliderMaxLinearVelocity = 200.0f;
        }
        if (!std::isfinite(rockHandBoneColliderMaxAngularVelocity) || rockHandBoneColliderMaxAngularVelocity <= 0.0f) {
            rockHandBoneColliderMaxAngularVelocity = 500.0f;
        }

        rockNearDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fNearDetectionRange", rockNearDetectionRange));
        rockFarDetectionRange = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarDetectionRange", rockFarDetectionRange));
        rockNearCastRadiusGameUnits = readClampedFloat(ini,
            SECTION,
            "fNearCastRadiusGameUnits",
            rockNearCastRadiusGameUnits,
            kDefaultNearCastRadiusGameUnits,
            0.0f,
            kDefaultNearCastRadiusGameUnits);
        rockNearCastDistanceGameUnits = readClampedFloat(ini,
            SECTION,
            "fNearCastDistanceGameUnits",
            rockNearCastDistanceGameUnits,
            kDefaultNearCastDistanceGameUnits,
            0.1f,
            kDefaultNearCastDistanceGameUnits);
        rockFarCastRadiusGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fFarCastRadiusGameUnits", rockFarCastRadiusGameUnits));
        rockCloseSelectionAngleDegrees =
            readSelectionAimAngleDegrees(ini, SECTION, "iCloseSelectionAngleDegrees", rockCloseSelectionAngleDegrees);
        rockFarSelectionAngleDegrees =
            readSelectionAimAngleDegrees(ini, SECTION, "iFarSelectionAngleDegrees", rockFarSelectionAngleDegrees);
        rockFarSelectionHmdConeEnabled = ini.GetBoolValue(SECTION, "bFarSelectionHmdConeEnabled", rockFarSelectionHmdConeEnabled);
        rockFarSelectionHmdConeHalfAngleDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fFarSelectionHmdConeHalfAngleDegrees", rockFarSelectionHmdConeHalfAngleDegrees));
        rockFarSelectionBlockedReferenceFormIds =
            ini.GetValue(SECTION, "sFarSelectionBlockedReferenceFormIDs", rockFarSelectionBlockedReferenceFormIds.c_str());
        rockFarSelectionBlockedBaseFormIds = ini.GetValue(SECTION, "sFarSelectionBlockedBaseFormIDs", rockFarSelectionBlockedBaseFormIds.c_str());
        rockFarSelectionBlockedFormTypes = ini.GetValue(SECTION, "sFarSelectionBlockedFormTypes", rockFarSelectionBlockedFormTypes.c_str());
        rockFarSelectionBlockedLayers = ini.GetValue(SECTION, "sFarSelectionBlockedLayers", rockFarSelectionBlockedLayers.c_str());
        const float sanitizedFarSelectionHmdConeHalfAngleDegrees =
            selection_query_policy::sanitizeFarSelectionHmdConeHalfAngleDegrees(rockFarSelectionHmdConeHalfAngleDegrees);
        if (sanitizedFarSelectionHmdConeHalfAngleDegrees != rockFarSelectionHmdConeHalfAngleDegrees) {
            ROCK_LOG_WARN(Config,
                "Invalid fFarSelectionHmdConeHalfAngleDegrees={} -- using {}",
                rockFarSelectionHmdConeHalfAngleDegrees,
                sanitizedFarSelectionHmdConeHalfAngleDegrees);
            rockFarSelectionHmdConeHalfAngleDegrees = sanitizedFarSelectionHmdConeHalfAngleDegrees;
        }
        rockCloseSelectionBehindPalmToleranceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fCloseSelectionBehindPalmToleranceGameUnits", rockCloseSelectionBehindPalmToleranceGameUnits));
        if (!std::isfinite(rockCloseSelectionBehindPalmToleranceGameUnits) || rockCloseSelectionBehindPalmToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fCloseSelectionBehindPalmToleranceGameUnits={} -- using 2.0", rockCloseSelectionBehindPalmToleranceGameUnits);
            rockCloseSelectionBehindPalmToleranceGameUnits = 2.0f;
        }
        rockSelectionShapeCastFilterInfo =
            readHexFilter("sSelectionShapeCastFilterInfo", rockSelectionShapeCastFilterInfo, selection_query_policy::kDefaultShapeCastFilterInfo);
        rockFarClipRayFilterInfo = readHexFilter("sFarClipRayFilterInfo", rockFarClipRayFilterInfo, selection_query_policy::kDefaultFarClipRayFilterInfo);
        rockPullApplyVelocityTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullApplyVelocityTime", rockPullApplyVelocityTime));
        rockPullOwnerGraceSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullOwnerGraceSeconds", rockPullOwnerGraceSeconds));
        if (!std::isfinite(rockPullOwnerGraceSeconds) || rockPullOwnerGraceSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullOwnerGraceSeconds={} -- using 1.0", rockPullOwnerGraceSeconds);
            rockPullOwnerGraceSeconds = 1.0f;
        }
        rockPullOwnerGraceSeconds = std::clamp(rockPullOwnerGraceSeconds, 0.0f, 3.0f);
        rockPullTrackHandTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullTrackHandTime", rockPullTrackHandTime));
        rockPullDestinationZOffsetHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDestinationZOffsetHavok", rockPullDestinationZOffsetHavok));
        rockPullDurationA = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationA", rockPullDurationA));
        rockPullDurationB = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationB", rockPullDurationB));
        rockPullDurationC = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullDurationC", rockPullDurationC));
        rockPullMaxVelocityHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullMaxVelocityHavok", rockPullMaxVelocityHavok));
        rockPullAutoGrabDistanceGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullAutoGrabDistanceGameUnits", rockPullAutoGrabDistanceGameUnits));
        rockPullCatchRetryMaxTimeSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fPullCatchRetryMaxTimeSeconds", rockPullCatchRetryMaxTimeSeconds));
        if (!std::isfinite(rockPullCatchRetryMaxTimeSeconds) || rockPullCatchRetryMaxTimeSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullCatchRetryMaxTimeSeconds={} -- using 0.65", rockPullCatchRetryMaxTimeSeconds);
            rockPullCatchRetryMaxTimeSeconds = 0.65f;
        }
        rockPullCatchWideReacquireEnabled = ini.GetBoolValue(SECTION, "bPullCatchWideReacquireEnabled", rockPullCatchWideReacquireEnabled);
        rockPullCatchWideReacquireRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPullCatchWideReacquireRadiusGameUnits", rockPullCatchWideReacquireRadiusGameUnits));
        if (!std::isfinite(rockPullCatchWideReacquireRadiusGameUnits) || rockPullCatchWideReacquireRadiusGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullCatchWideReacquireRadiusGameUnits={} -- using 32.0", rockPullCatchWideReacquireRadiusGameUnits);
            rockPullCatchWideReacquireRadiusGameUnits = 32.0f;
        }
        rockPullCatchWideReacquireRadiusGameUnits = std::clamp(rockPullCatchWideReacquireRadiusGameUnits, 0.0f, 120.0f);
        rockPullCatchWideReacquireMaxBodyDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPullCatchWideReacquireMaxBodyDistanceGameUnits", rockPullCatchWideReacquireMaxBodyDistanceGameUnits));
        if (!std::isfinite(rockPullCatchWideReacquireMaxBodyDistanceGameUnits) || rockPullCatchWideReacquireMaxBodyDistanceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPullCatchWideReacquireMaxBodyDistanceGameUnits={} -- using 42.0", rockPullCatchWideReacquireMaxBodyDistanceGameUnits);
            rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42.0f;
        }
        rockPullCatchWideReacquireMaxBodyDistanceGameUnits = std::clamp(rockPullCatchWideReacquireMaxBodyDistanceGameUnits, 0.0f, 160.0f);
        rockObjectPhysicsTreeMaxDepth = static_cast<int>(ini.GetLongValue(SECTION, "iObjectPhysicsTreeMaxDepth", rockObjectPhysicsTreeMaxDepth));
        rockDynamicPushAssistEnabled = ini.GetBoolValue(SECTION, "bDynamicPushAssistEnabled", rockDynamicPushAssistEnabled);
        rockDynamicPushMinSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fDynamicPushMinSpeed", rockDynamicPushMinSpeed));
        rockDynamicPushMaxImpulse = static_cast<float>(ini.GetDoubleValue(SECTION, "fDynamicPushMaxImpulse", rockDynamicPushMaxImpulse));
        rockDynamicPushCooldownSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fDynamicPushCooldownSeconds", rockDynamicPushCooldownSeconds));

        rockGrabLinearTau = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearTau", rockGrabLinearTau));
        rockGrabLinearDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearDamping", rockGrabLinearDamping));
        rockGrabLinearProportionalRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearProportionalRecovery", rockGrabLinearProportionalRecovery));
        rockGrabLinearConstantRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLinearConstantRecovery", rockGrabLinearConstantRecovery));

        rockGrabAngularTau = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularTau", rockGrabAngularTau));
        rockGrabAngularDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularDamping", rockGrabAngularDamping));
        rockGrabAngularProportionalRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularProportionalRecovery", rockGrabAngularProportionalRecovery));
        rockGrabAngularConstantRecovery = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAngularConstantRecovery", rockGrabAngularConstantRecovery));

        rockGrabConstraintMaxForce = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConstraintMaxForce", rockGrabConstraintMaxForce));
        rockGrabMaxForceToMassRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxForceToMassRatio", rockGrabMaxForceToMassRatio));
        rockGrabEffectiveMotorMassFloorEnabled =
            ini.GetBoolValue(SECTION, "bGrabEffectiveMotorMassFloorEnabled", rockGrabEffectiveMotorMassFloorEnabled);
        rockGrabEffectiveMotorMassFloor = readClampedFloat(ini,
            SECTION,
            "fGrabEffectiveMotorMassFloor",
            rockGrabEffectiveMotorMassFloor,
            kDefaultGrabEffectiveMotorMassFloor,
            0.0f,
            100.0f);
        rockGrabPhysicsRateForceScalingEnabled =
            ini.GetBoolValue(SECTION, "bGrabPhysicsRateForceScalingEnabled", rockGrabPhysicsRateForceScalingEnabled);
        rockGrabPhysicsRateReferenceHz = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateReferenceHz",
            rockGrabPhysicsRateReferenceHz,
            kDefaultGrabPhysicsRateReferenceHz,
            1.0f,
            240.0f);
        rockGrabPhysicsRateForceScaleExponent = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateForceScaleExponent",
            rockGrabPhysicsRateForceScaleExponent,
            kDefaultGrabPhysicsRateForceScaleExponent,
            0.0f,
            2.0f);
        rockGrabPhysicsRateMinForceScale = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateMinForceScale",
            rockGrabPhysicsRateMinForceScale,
            kDefaultGrabPhysicsRateMinForceScale,
            0.1f,
            2.0f);
        rockGrabPhysicsRateMaxForceScale = readClampedFloat(ini,
            SECTION,
            "fGrabPhysicsRateMaxForceScale",
            rockGrabPhysicsRateMaxForceScale,
            kDefaultGrabPhysicsRateMaxForceScale,
            rockGrabPhysicsRateMinForceScale,
            3.0f);

        rockGrabForceFadeInTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabForceFadeInTime", rockGrabForceFadeInTime));
        rockGrabRagdollDecompositionMode =
            static_cast<int>(ini.GetLongValue(SECTION, "iGrabRagdollDecompositionMode", rockGrabRagdollDecompositionMode));
        if (rockGrabRagdollDecompositionMode < -1 || rockGrabRagdollDecompositionMode > 1) {
            ROCK_LOG_WARN(Config, "Invalid iGrabRagdollDecompositionMode={} -- using -1", rockGrabRagdollDecompositionMode);
            rockGrabRagdollDecompositionMode = -1;
        }
        rockRightGrabAuthorityProxyOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightGrabAuthorityProxyOffsetXGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits.x));
        rockRightGrabAuthorityProxyOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightGrabAuthorityProxyOffsetYGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits.y));
        rockRightGrabAuthorityProxyOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightGrabAuthorityProxyOffsetZGameUnits", rockRightGrabAuthorityProxyOffsetGameUnits.z));
        rockLeftGrabAuthorityProxyOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftGrabAuthorityProxyOffsetXGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits.x));
        rockLeftGrabAuthorityProxyOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftGrabAuthorityProxyOffsetYGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits.y));
        rockLeftGrabAuthorityProxyOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftGrabAuthorityProxyOffsetZGameUnits", rockLeftGrabAuthorityProxyOffsetGameUnits.z));
        rockRightCustomOGAOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightCustomOGAOffsetXGameUnits", rockRightCustomOGAOffsetGameUnits.x));
        rockRightCustomOGAOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightCustomOGAOffsetYGameUnits", rockRightCustomOGAOffsetGameUnits.y));
        rockRightCustomOGAOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fRightCustomOGAOffsetZGameUnits", rockRightCustomOGAOffsetGameUnits.z));
        rockLeftCustomOGAOffsetGameUnits.x =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftCustomOGAOffsetXGameUnits", rockLeftCustomOGAOffsetGameUnits.x));
        rockLeftCustomOGAOffsetGameUnits.y =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftCustomOGAOffsetYGameUnits", rockLeftCustomOGAOffsetGameUnits.y));
        rockLeftCustomOGAOffsetGameUnits.z =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fLeftCustomOGAOffsetZGameUnits", rockLeftCustomOGAOffsetGameUnits.z));
        rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintLinearTauMultiplier",
            rockGrabLooseWeaponSharedConstraintLinearTauMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularTauMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularTauMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintCollisionTauMultiplier",
            rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintLinearDampingMultiplier",
            rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintLinearDampingMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularDampingMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularDampingMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintMaxForceMultiplier",
            rockGrabLooseWeaponSharedConstraintMaxForceMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier,
            0.05f,
            8.0f);
        rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier",
            rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier,
            0.05f,
            4.0f);
        rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = readClampedFloat(ini,
            SECTION,
            "fGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier",
            rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier,
            kDefaultGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier,
            0.05f,
            4.0f);

        rockGrabTauMin = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauMin", rockGrabTauMin));
        rockGrabTauLerpSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTauLerpSpeed", rockGrabTauLerpSpeed));
        rockGrabLongObjectAngularScalingEnabled = ini.GetBoolValue(SECTION, "bGrabLongObjectAngularScalingEnabled", rockGrabLongObjectAngularScalingEnabled);
        rockGrabLongObjectReferenceLeverGameUnits = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fGrabLongObjectReferenceLeverGameUnits", rockGrabLongObjectReferenceLeverGameUnits));
        rockGrabLongObjectReferenceLeverGameUnits =
            std::clamp(std::isfinite(rockGrabLongObjectReferenceLeverGameUnits) ? rockGrabLongObjectReferenceLeverGameUnits : kDefaultGrabLongObjectReferenceLeverGameUnits,
                1.0f,
                240.0f);
        rockGrabLongObjectMinAngularScale = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fGrabLongObjectMinAngularScale", rockGrabLongObjectMinAngularScale));
        rockGrabLongObjectMinAngularScale =
            std::clamp(std::isfinite(rockGrabLongObjectMinAngularScale) ? rockGrabLongObjectMinAngularScale : kDefaultGrabLongObjectMinAngularScale,
                0.05f,
                1.0f);
        rockGrabPivotQualityAngularScalingEnabled =
            ini.GetBoolValue(SECTION, "bGrabPivotQualityAngularScalingEnabled", rockGrabPivotQualityAngularScalingEnabled);
        rockGrabPositionOnlyAngularScale = readClampedFloat(ini,
            SECTION,
            "fGrabPositionOnlyAngularScale",
            rockGrabPositionOnlyAngularScale,
            kDefaultGrabPositionOnlyAngularScale,
            0.05f,
            1.0f);
        rockGrabSmallObjectReferenceLeverGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabSmallObjectReferenceLeverGameUnits",
            rockGrabSmallObjectReferenceLeverGameUnits,
            kDefaultGrabSmallObjectReferenceLeverGameUnits,
            1.0f,
            120.0f);
        rockGrabSmallObjectAngularScale = readClampedFloat(ini,
            SECTION,
            "fGrabSmallObjectAngularScale",
            rockGrabSmallObjectAngularScale,
            kDefaultGrabSmallObjectAngularScale,
            0.05f,
            1.0f);
        rockGrabLowContactSupportAngularScale = readClampedFloat(ini,
            SECTION,
            "fGrabLowContactSupportAngularScale",
            rockGrabLowContactSupportAngularScale,
            kDefaultGrabLowContactSupportAngularScale,
            0.05f,
            1.0f);
        rockGrabMinAngularAuthorityScale = readClampedFloat(ini,
            SECTION,
            "fGrabMinAngularAuthorityScale",
            rockGrabMinAngularAuthorityScale,
            kDefaultGrabMinAngularAuthorityScale,
            0.05f,
            1.0f);
        rockGrabWeakPivotTwistScale = readClampedFloat(ini,
            SECTION,
            "fGrabWeakPivotTwistScale",
            rockGrabWeakPivotTwistScale,
            kDefaultGrabWeakPivotTwistScale,
            0.0f,
            1.0f);

        rockGrabMaxInertiaRatio = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxInertiaRatio", rockGrabMaxInertiaRatio));
        rockGrabMinInertia = readClampedFloat(ini,
            SECTION,
            "fGrabMinInertia",
            rockGrabMinInertia,
            kDefaultGrabMinInertia,
            0.0001f,
            100.0f);

        rockGrabMaxDeviation = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviation", rockGrabMaxDeviation));
        rockGrabMaxDeviationTime = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxDeviationTime", rockGrabMaxDeviationTime));
        rockGrabButtonID = static_cast<int>(ini.GetLongValue(SECTION, "iGrabButtonID", rockGrabButtonID));
        if (!input_remap_policy::isValidButtonId(rockGrabButtonID)) {
            ROCK_LOG_WARN(Config, "iGrabButtonID must be 0..63; using 2");
            rockGrabButtonID = 2;
        }
        rockThrowVelocityMultiplier = static_cast<float>(ini.GetDoubleValue(SECTION, "fThrowVelocityMultiplier", rockThrowVelocityMultiplier));
        rockGrabControllerDerivedThrowVelocityEnabled =
            ini.GetBoolValue(SECTION, "bGrabControllerDerivedThrowVelocityEnabled", rockGrabControllerDerivedThrowVelocityEnabled);
        rockGrabThrowObjectVelocityBlend =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowObjectVelocityBlend", rockGrabThrowObjectVelocityBlend));
        rockGrabThrowObjectVelocityBlend = std::clamp(
            std::isfinite(rockGrabThrowObjectVelocityBlend) ? rockGrabThrowObjectVelocityBlend : kDefaultGrabThrowObjectVelocityBlend,
            0.0f,
            1.0f);
        rockGrabThrowTangentialVelocityScale =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowTangentialVelocityScale", rockGrabThrowTangentialVelocityScale));
        rockGrabThrowTangentialVelocityScale = std::clamp(
            std::isfinite(rockGrabThrowTangentialVelocityScale) ? rockGrabThrowTangentialVelocityScale : kDefaultGrabThrowTangentialVelocityScale,
            0.0f,
            2.0f);
        rockGrabThrowMaxVelocityHavok = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowMaxVelocityHavok", rockGrabThrowMaxVelocityHavok));
        rockGrabThrowMaxVelocityHavok = std::clamp(
            std::isfinite(rockGrabThrowMaxVelocityHavok) ? rockGrabThrowMaxVelocityHavok : kDefaultGrabThrowMaxVelocityHavok,
            1.0f,
            60.0f);
        rockGrabThrowAngularVelocityScale =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThrowAngularVelocityScale", rockGrabThrowAngularVelocityScale));
        rockGrabThrowAngularVelocityScale = std::clamp(
            std::isfinite(rockGrabThrowAngularVelocityScale) ? rockGrabThrowAngularVelocityScale : kDefaultGrabThrowAngularVelocityScale,
            0.0f,
            2.0f);
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = static_cast<float>(
            ini.GetDoubleValue(SECTION, "fGrabThrowMaxAngularVelocityRadiansPerSecond", rockGrabThrowMaxAngularVelocityRadiansPerSecond));
        rockGrabThrowMaxAngularVelocityRadiansPerSecond = std::clamp(
            std::isfinite(rockGrabThrowMaxAngularVelocityRadiansPerSecond) ? rockGrabThrowMaxAngularVelocityRadiansPerSecond : kDefaultGrabThrowMaxAngularVelocityRadiansPerSecond,
            0.0f,
            60.0f);
        rockGrabReleaseHandCollisionDelaySeconds =
            rock::hand_collision_suppression_math::sanitizeDelaySeconds(
                static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabReleaseHandCollisionDelaySeconds", rockGrabReleaseHandCollisionDelaySeconds)));
        rockGrabVelocityDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabVelocityDamping", rockGrabVelocityDamping));
        rockGrabPlayerSpaceCompensation = ini.GetBoolValue(SECTION, "bGrabPlayerSpaceCompensation", rockGrabPlayerSpaceCompensation);
        rockGrabPlayerSpaceWarpDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPlayerSpaceWarpDistance", rockGrabPlayerSpaceWarpDistance));
        rockGrabPlayerSpaceWarpMinRotationDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPlayerSpaceWarpMinRotationDegrees", rockGrabPlayerSpaceWarpMinRotationDegrees));
        rockGrabPlayerSpaceTransformWarpEnabled = ini.GetBoolValue(SECTION, "bGrabPlayerSpaceTransformWarpEnabled", rockGrabPlayerSpaceTransformWarpEnabled);
        rockGrabLocomotionAuthorityBridgeEnabled =
            ini.GetBoolValue(SECTION, "bGrabLocomotionAuthorityBridgeEnabled", rockGrabLocomotionAuthorityBridgeEnabled);
        rockGrabLocomotionAuthorityMaxLeadSeconds = readClampedFloat(ini,
            SECTION,
            "fGrabLocomotionAuthorityMaxLeadSeconds",
            rockGrabLocomotionAuthorityMaxLeadSeconds,
            grab_locomotion_authority_bridge::kDefaultMaxLeadSeconds,
            0.0f,
            0.05f);
        rockGrabLocomotionAuthoritySmoothingHz = readClampedFloat(ini,
            SECTION,
            "fGrabLocomotionAuthoritySmoothingHz",
            rockGrabLocomotionAuthoritySmoothingHz,
            grab_locomotion_authority_bridge::kDefaultSmoothingHz,
            0.0f,
            240.0f);
        rockGrabLocomotionAuthorityMaxOffsetGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabLocomotionAuthorityMaxOffsetGameUnits",
            rockGrabLocomotionAuthorityMaxOffsetGameUnits,
            grab_locomotion_authority_bridge::kDefaultMaxOffsetGameUnits,
            0.0f,
            50.0f);
        rockGrabLocomotionAuthorityResetDistanceGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabLocomotionAuthorityResetDistanceGameUnits",
            rockGrabLocomotionAuthorityResetDistanceGameUnits,
            grab_locomotion_authority_bridge::kDefaultResetDistanceGameUnits,
            1.0f,
            500.0f);
        rockGrabResidualVelocityDamping = ini.GetBoolValue(SECTION, "bGrabResidualVelocityDamping", rockGrabResidualVelocityDamping);
        rockGrabNearbyDampingEnabled = ini.GetBoolValue(SECTION, "bGrabNearbyDampingEnabled", rockGrabNearbyDampingEnabled);
        rockGrabNearbyDampingRadius =
            nearby_grab_damping::sanitizeRadius(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyDampingRadius", rockGrabNearbyDampingRadius)));
        rockGrabNearbyDampingSeconds =
            nearby_grab_damping::sanitizeDuration(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyDampingSeconds", rockGrabNearbyDampingSeconds)));
        rockGrabNearbyLinearDamping =
            nearby_grab_damping::sanitizeHknpDampingCoefficient(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyLinearDamping", rockGrabNearbyLinearDamping)));
        rockGrabNearbyAngularDamping =
            nearby_grab_damping::sanitizeHknpDampingCoefficient(static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearbyAngularDamping", rockGrabNearbyAngularDamping)));
        rockGrabHeldMassMovementSlowdownEnabled =
            ini.GetBoolValue(SECTION, "bGrabHeldMassMovementSlowdownEnabled", rockGrabHeldMassMovementSlowdownEnabled);
        rockGrabHeldMassMovementMassProportion = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementMassProportion",
            rockGrabHeldMassMovementMassProportion,
            0.675f,
            0.0f,
            10.0f);
        rockGrabHeldMassMovementMassExponent = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementMassExponent",
            rockGrabHeldMassMovementMassExponent,
            1.0f,
            0.0f,
            4.0f);
        rockGrabHeldMassMovementMaxReduction = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementMaxReduction",
            rockGrabHeldMassMovementMaxReduction,
            75.0f,
            0.0f,
            99.0f);
        rockGrabHeldMassMovementFadeOutSeconds = readClampedFloat(ini,
            SECTION,
            "fGrabHeldMassMovementFadeOutSeconds",
            rockGrabHeldMassMovementFadeOutSeconds,
            5.0f,
            0.0f,
            60.0f);
        rockGrabTouchAcquireDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabTouchAcquireDistanceGameUnits", rockGrabTouchAcquireDistanceGameUnits));
        if (!std::isfinite(rockGrabTouchAcquireDistanceGameUnits) || rockGrabTouchAcquireDistanceGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabTouchAcquireDistanceGameUnits={} -- using 4.0", rockGrabTouchAcquireDistanceGameUnits);
            rockGrabTouchAcquireDistanceGameUnits = 4.0f;
        }
        rockGrabNearConvergeDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabNearConvergeDistanceGameUnits", rockGrabNearConvergeDistanceGameUnits));
        if (!std::isfinite(rockGrabNearConvergeDistanceGameUnits) || rockGrabNearConvergeDistanceGameUnits < rockGrabTouchAcquireDistanceGameUnits) {
            ROCK_LOG_WARN(Config,
                "Invalid fGrabNearConvergeDistanceGameUnits={} -- using touch distance {}",
                rockGrabNearConvergeDistanceGameUnits,
                rockGrabTouchAcquireDistanceGameUnits);
            rockGrabNearConvergeDistanceGameUnits = rockGrabTouchAcquireDistanceGameUnits;
        }
        rockGrabPocketDepthGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPocketDepthGameUnits", rockGrabPocketDepthGameUnits));
        if (!std::isfinite(rockGrabPocketDepthGameUnits) || rockGrabPocketDepthGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabPocketDepthGameUnits={} -- using 7.0", rockGrabPocketDepthGameUnits);
            rockGrabPocketDepthGameUnits = 7.0f;
        }
        rockGrabPocketRadiusGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabPocketRadiusGameUnits", rockGrabPocketRadiusGameUnits));
        if (!std::isfinite(rockGrabPocketRadiusGameUnits) || rockGrabPocketRadiusGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabPocketRadiusGameUnits={} -- using 9.0", rockGrabPocketRadiusGameUnits);
            rockGrabPocketRadiusGameUnits = 9.0f;
        }
        rockGrabGripInsetGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabGripInsetGameUnits", rockGrabGripInsetGameUnits));
        if (!std::isfinite(rockGrabGripInsetGameUnits) || rockGrabGripInsetGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabGripInsetGameUnits={} -- using 2.0", rockGrabGripInsetGameUnits);
            rockGrabGripInsetGameUnits = 2.0f;
        }
        rockGrabGripMaxInsetGameUnits = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabGripMaxInsetGameUnits", rockGrabGripMaxInsetGameUnits));
        if (!std::isfinite(rockGrabGripMaxInsetGameUnits) || rockGrabGripMaxInsetGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabGripMaxInsetGameUnits={} -- using 6.0", rockGrabGripMaxInsetGameUnits);
            rockGrabGripMaxInsetGameUnits = 6.0f;
        }
        rockGrabConvergeMaxTimeSeconds = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabConvergeMaxTimeSeconds", rockGrabConvergeMaxTimeSeconds));
        if (!std::isfinite(rockGrabConvergeMaxTimeSeconds) || rockGrabConvergeMaxTimeSeconds < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabConvergeMaxTimeSeconds={} -- using 0.35", rockGrabConvergeMaxTimeSeconds);
            rockGrabConvergeMaxTimeSeconds = 0.35f;
        }
        rockGrabConvergeStableFrames = static_cast<int>(ini.GetLongValue(SECTION, "iGrabConvergeStableFrames", rockGrabConvergeStableFrames));
        if (rockGrabConvergeStableFrames < 1) {
            ROCK_LOG_WARN(Config, "Invalid iGrabConvergeStableFrames={} -- using 3", rockGrabConvergeStableFrames);
            rockGrabConvergeStableFrames = 3;
        }
        rockGrabConvergeStableFrames = std::clamp(rockGrabConvergeStableFrames, 1, 12);
        rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond =
            static_cast<float>(ini.GetDoubleValue(
                SECTION,
                "fGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond",
                rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond));
        if (!std::isfinite(rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond) || rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond < 0.0f) {
            ROCK_LOG_WARN(Config,
                "Invalid fGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond={} -- using 40.0",
                rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond);
            rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40.0f;
        }
        rockGrabAcquisitionVisualStartDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(
                SECTION,
                "fGrabAcquisitionVisualStartDistanceGameUnits",
                rockGrabAcquisitionVisualStartDistanceGameUnits));
        if (!std::isfinite(rockGrabAcquisitionVisualStartDistanceGameUnits) || rockGrabAcquisitionVisualStartDistanceGameUnits <= 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabAcquisitionVisualStartDistanceGameUnits={} -- using 28.0", rockGrabAcquisitionVisualStartDistanceGameUnits);
            rockGrabAcquisitionVisualStartDistanceGameUnits = 28.0f;
        }
        rockGrabAcquisitionVisualStartDistanceGameUnits =
            grab_three_phase::computeAcquisitionVisualEnvelopeGameUnits(
                rockGrabTouchAcquireDistanceGameUnits,
                rockGrabNearConvergeDistanceGameUnits,
                rockGrabAcquisitionVisualStartDistanceGameUnits);
        rockGrabMultiFingerContactValidationEnabled =
            ini.GetBoolValue(SECTION, "bGrabMultiFingerContactValidationEnabled", rockGrabMultiFingerContactValidationEnabled);
        rockGrabContactQualityMode = static_cast<int>(ini.GetLongValue(SECTION, "iGrabContactQualityMode", rockGrabContactQualityMode));
        rockGrabContactQualityMode = std::clamp(rockGrabContactQualityMode, 0, 2);
        rockGrabMinFingerContactGroups =
            static_cast<int>(ini.GetLongValue(SECTION, "iGrabMinFingerContactGroups", rockGrabMinFingerContactGroups));
        rockGrabMinFingerContactGroups = std::clamp(rockGrabMinFingerContactGroups, 1, 5);
        rockGrabMinFingerContactSpreadGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMinFingerContactSpreadGameUnits", rockGrabMinFingerContactSpreadGameUnits));
        if (!std::isfinite(rockGrabMinFingerContactSpreadGameUnits) || rockGrabMinFingerContactSpreadGameUnits < 0.0f) {
            rockGrabMinFingerContactSpreadGameUnits = 1.0f;
        }
        rockGrabFingerContactMeshSnapMaxDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerContactMeshSnapMaxDistanceGameUnits", rockGrabFingerContactMeshSnapMaxDistanceGameUnits));
        if (!std::isfinite(rockGrabFingerContactMeshSnapMaxDistanceGameUnits) || rockGrabFingerContactMeshSnapMaxDistanceGameUnits < 0.0f) {
            rockGrabFingerContactMeshSnapMaxDistanceGameUnits = 10.0f;
        }
        rockGrabSurfaceBehindPalmToleranceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabSurfaceBehindPalmToleranceGameUnits", rockGrabSurfaceBehindPalmToleranceGameUnits));
        if (!std::isfinite(rockGrabSurfaceBehindPalmToleranceGameUnits) || rockGrabSurfaceBehindPalmToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabSurfaceBehindPalmToleranceGameUnits={} -- using 1.5", rockGrabSurfaceBehindPalmToleranceGameUnits);
            rockGrabSurfaceBehindPalmToleranceGameUnits = 1.5f;
        }
        rockGrabOppositionContactMaxAgeFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iGrabOppositionContactMaxAgeFrames", rockGrabOppositionContactMaxAgeFrames));
        rockGrabOppositionContactMaxAgeFrames = std::clamp(rockGrabOppositionContactMaxAgeFrames, 0, 60);
        rockGrabPinchPocketEnabled = ini.GetBoolValue(SECTION, "bGrabPinchPocketEnabled", rockGrabPinchPocketEnabled);
        rockGrabPinchCloseSelectionEnabled = ini.GetBoolValue(SECTION, "bGrabPinchCloseSelectionEnabled", rockGrabPinchCloseSelectionEnabled);
        rockGrabPinchCompactMaxExtentGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchCompactMaxExtentGameUnits",
            rockGrabPinchCompactMaxExtentGameUnits,
            grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits,
            1.0f,
            grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits);
        rockGrabPinchThinRodMaxLengthGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchThinRodMaxLengthGameUnits",
            rockGrabPinchThinRodMaxLengthGameUnits,
            grab_pinch_pocket_policy::kDefaultThinRodMaxLengthGameUnits,
            1.0f,
            120.0f);
        rockGrabPinchThinRodMaxCrossSectionGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchThinRodMaxCrossSectionGameUnits",
            rockGrabPinchThinRodMaxCrossSectionGameUnits,
            grab_pinch_pocket_policy::kDefaultThinRodMaxCrossSectionGameUnits,
            0.1f,
            40.0f);
        rockGrabPinchMaxPocketDistanceGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMaxPocketDistanceGameUnits",
            rockGrabPinchMaxPocketDistanceGameUnits,
            grab_pinch_pocket_policy::kDefaultMaxPocketDistanceGameUnits,
            0.1f,
            80.0f);
        rockGrabPinchMinFingerGapGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMinFingerGapGameUnits",
            rockGrabPinchMinFingerGapGameUnits,
            grab_pinch_pocket_policy::kDefaultMinFingerGapGameUnits,
            0.0f,
            40.0f);
        rockGrabPinchMaxFingerGapGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchMaxFingerGapGameUnits",
            rockGrabPinchMaxFingerGapGameUnits,
            grab_pinch_pocket_policy::kDefaultMaxFingerGapGameUnits,
            0.1f,
            80.0f);
        if (rockGrabPinchMaxFingerGapGameUnits < rockGrabPinchMinFingerGapGameUnits) {
            rockGrabPinchMaxFingerGapGameUnits = rockGrabPinchMinFingerGapGameUnits;
        }
        rockGrabPinchThumbIndexMaxOpenValue = readClampedFloat(ini,
            SECTION,
            "fGrabPinchThumbIndexMaxOpenValue",
            rockGrabPinchThumbIndexMaxOpenValue,
            grab_pinch_pocket_policy::kDefaultThumbIndexMaxOpenValue,
            0.0f,
            1.0f);
        rockGrabPinchOtherFingerCurlValue = readClampedFloat(ini,
            SECTION,
            "fGrabPinchOtherFingerCurlValue",
            rockGrabPinchOtherFingerCurlValue,
            grab_pinch_pocket_policy::kDefaultOtherFingerCurlValue,
            0.0f,
            1.0f);
        rockGrabPinchSurfaceInsetGameUnits = readClampedFloat(ini,
            SECTION,
            "fGrabPinchSurfaceInsetGameUnits",
            rockGrabPinchSurfaceInsetGameUnits,
            grab_pinch_pocket_policy::kDefaultSurfaceInsetGameUnits,
            0.0f,
            8.0f);
        readVec3("fGrabPinchDetectionDirectionHandspaceX",
            "fGrabPinchDetectionDirectionHandspaceY",
            "fGrabPinchDetectionDirectionHandspaceZ",
            rockGrabPinchDetectionDirectionHandspace);
        rockGrabPinchDetectionAxisBlend = readClampedFloat(ini,
            SECTION,
            "fGrabPinchDetectionAxisBlend",
            rockGrabPinchDetectionAxisBlend,
            grab_pinch_pocket_policy::kDefaultDetectionAxisBlend,
            0.0f,
            1.0f);
        {
            auto pinchDetectionConfig = grab_pinch_pocket_policy::Config{};
            pinchDetectionConfig.detectionDirectionHandspace = rockGrabPinchDetectionDirectionHandspace;
            pinchDetectionConfig.detectionAxisBlend = rockGrabPinchDetectionAxisBlend;
            const auto sanitizedPinchDetectionConfig = grab_pinch_pocket_policy::sanitizeConfig(pinchDetectionConfig);
            rockGrabPinchDetectionDirectionHandspace = sanitizedPinchDetectionConfig.detectionDirectionHandspace;
            rockGrabPinchDetectionAxisBlend = sanitizedPinchDetectionConfig.detectionAxisBlend;
        }
        rockGrabHandLerpEnabled = ini.GetBoolValue(SECTION, "bGrabHandLerpEnabled", rockGrabHandLerpEnabled);
        rockGrabHandLerpTimeMin = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpTimeMin",
            rockGrabHandLerpTimeMin,
            0.10f,
            0.0f,
            1.0f);
        rockGrabHandLerpTimeMax = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpTimeMax",
            rockGrabHandLerpTimeMax,
            0.20f,
            rockGrabHandLerpTimeMin,
            1.0f);
        rockGrabHandLerpMinDistance = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpMinDistance",
            rockGrabHandLerpMinDistance,
            7.0f,
            0.0f,
            80.0f);
        rockGrabHandLerpMaxDistance = readClampedFloat(ini,
            SECTION,
            "fGrabHandLerpMaxDistance",
            rockGrabHandLerpMaxDistance,
            14.0f,
            rockGrabHandLerpMinDistance,
            120.0f);
        rockGrabMeshFingerPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshFingerPoseEnabled", rockGrabMeshFingerPoseEnabled);
        rockGrabMeshJointPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshJointPoseEnabled", rockGrabMeshJointPoseEnabled);
        rockGrabFingerPoseUpdateInterval = static_cast<int>(ini.GetLongValue(SECTION, "iGrabFingerPoseUpdateInterval", rockGrabFingerPoseUpdateInterval));
        rockGrabFingerPoseUpdateInterval = std::clamp(rockGrabFingerPoseUpdateInterval, 1, 60);
        rockGrabFingerMinValue = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerMinValue", rockGrabFingerMinValue));
        if (!std::isfinite(rockGrabFingerMinValue)) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerMinValue={} -- using 0.2", rockGrabFingerMinValue);
            rockGrabFingerMinValue = 0.2f;
        }
        rockGrabFingerMinValue = std::clamp(rockGrabFingerMinValue, 0.0f, 1.0f);
        rockGrabFingerPoseSmoothingSpeed = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerPoseSmoothingSpeed", rockGrabFingerPoseSmoothingSpeed));
        if (!std::isfinite(rockGrabFingerPoseSmoothingSpeed) || rockGrabFingerPoseSmoothingSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerPoseSmoothingSpeed={} -- using 14.0", rockGrabFingerPoseSmoothingSpeed);
            rockGrabFingerPoseSmoothingSpeed = 14.0f;
        }
        rockGrabMeshLocalTransformPoseEnabled = ini.GetBoolValue(SECTION, "bGrabMeshLocalTransformPoseEnabled", rockGrabMeshLocalTransformPoseEnabled);
        rockGrabFingerLocalTransformSmoothingSpeed =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerLocalTransformSmoothingSpeed", rockGrabFingerLocalTransformSmoothingSpeed));
        if (!std::isfinite(rockGrabFingerLocalTransformSmoothingSpeed) || rockGrabFingerLocalTransformSmoothingSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerLocalTransformSmoothingSpeed={} -- using 14.0", rockGrabFingerLocalTransformSmoothingSpeed);
            rockGrabFingerLocalTransformSmoothingSpeed = 14.0f;
        }
        rockGrabFingerLocalTransformMaxCorrectionDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerLocalTransformMaxCorrectionDegrees", rockGrabFingerLocalTransformMaxCorrectionDegrees));
        if (!std::isfinite(rockGrabFingerLocalTransformMaxCorrectionDegrees) || rockGrabFingerLocalTransformMaxCorrectionDegrees < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerLocalTransformMaxCorrectionDegrees={} -- using 35.0", rockGrabFingerLocalTransformMaxCorrectionDegrees);
            rockGrabFingerLocalTransformMaxCorrectionDegrees = 35.0f;
        }
        rockGrabFingerSurfaceAimStrength = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerSurfaceAimStrength", rockGrabFingerSurfaceAimStrength));
        rockGrabFingerSurfaceAimStrength = std::clamp(std::isfinite(rockGrabFingerSurfaceAimStrength) ? rockGrabFingerSurfaceAimStrength : 0.75f, 0.0f, 1.0f);
        rockGrabFingerRejectBacksideHits = ini.GetBoolValue(SECTION, "bGrabFingerRejectBacksideHits", rockGrabFingerRejectBacksideHits);
        rockGrabFingerSurfacePlaneToleranceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabFingerSurfacePlaneToleranceGameUnits", rockGrabFingerSurfacePlaneToleranceGameUnits));
        if (!std::isfinite(rockGrabFingerSurfacePlaneToleranceGameUnits) || rockGrabFingerSurfacePlaneToleranceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fGrabFingerSurfacePlaneToleranceGameUnits={} -- using 1.5", rockGrabFingerSurfacePlaneToleranceGameUnits);
            rockGrabFingerSurfacePlaneToleranceGameUnits = 1.5f;
        }
        rockGrabThumbOppositionStrength = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbOppositionStrength", rockGrabThumbOppositionStrength));
        rockGrabThumbOppositionStrength = std::clamp(std::isfinite(rockGrabThumbOppositionStrength) ? rockGrabThumbOppositionStrength : 1.0f, 0.0f, 1.0f);
        rockGrabThumbAlternateCurveStrength =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbAlternateCurveStrength", rockGrabThumbAlternateCurveStrength));
        rockGrabThumbAlternateCurveStrength = std::clamp(std::isfinite(rockGrabThumbAlternateCurveStrength) ? rockGrabThumbAlternateCurveStrength : 0.65f, 0.0f, 1.0f);
        rockGrabThumbSurfaceSafetyEnabled = ini.GetBoolValue(SECTION, "bGrabThumbSurfaceSafetyEnabled", rockGrabThumbSurfaceSafetyEnabled);
        rockGrabThumbSurfaceSafetyMarginGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabThumbSurfaceSafetyMarginGameUnits", rockGrabThumbSurfaceSafetyMarginGameUnits));
        rockGrabThumbSurfaceSafetyMarginGameUnits = std::clamp(
            std::isfinite(rockGrabThumbSurfaceSafetyMarginGameUnits) ? rockGrabThumbSurfaceSafetyMarginGameUnits : kDefaultGrabThumbSurfaceSafetyMarginGameUnits,
            0.0f,
            5.0f);
        rockGrabLateralWeight = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabLateralWeight", rockGrabLateralWeight));
        rockGrabDirectionalWeight = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabDirectionalWeight", rockGrabDirectionalWeight));
        rockGrabMaxTriangleDistance = static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabMaxTriangleDistance", rockGrabMaxTriangleDistance));
        rockGrabMeshContactOnly = ini.GetBoolValue(SECTION, "bGrabMeshContactOnly", rockGrabMeshContactOnly);
        rockGrabRequireMeshContact = ini.GetBoolValue(SECTION, "bGrabRequireMeshContact", rockGrabRequireMeshContact);
        rockGrabContactPatchEnabled = ini.GetBoolValue(SECTION, "bGrabContactPatchEnabled", rockGrabContactPatchEnabled);
        rockGrabContactPatchProbeCount = static_cast<int>(ini.GetLongValue(SECTION, "iGrabContactPatchProbeCount", rockGrabContactPatchProbeCount));
        rockGrabContactPatchProbeCount = std::clamp(rockGrabContactPatchProbeCount, 1, 9);
        rockGrabContactPatchProbeSpacingGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchProbeSpacingGameUnits", rockGrabContactPatchProbeSpacingGameUnits));
        if (!std::isfinite(rockGrabContactPatchProbeSpacingGameUnits) || rockGrabContactPatchProbeSpacingGameUnits < 0.0f) {
            rockGrabContactPatchProbeSpacingGameUnits = 3.0f;
        }
        rockGrabContactPatchProbeRadiusGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchProbeRadiusGameUnits", rockGrabContactPatchProbeRadiusGameUnits));
        if (!std::isfinite(rockGrabContactPatchProbeRadiusGameUnits) || rockGrabContactPatchProbeRadiusGameUnits <= 0.0f) {
            rockGrabContactPatchProbeRadiusGameUnits = 2.0f;
        }
        rockGrabContactPatchMeshSnapMaxDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchMeshSnapMaxDistanceGameUnits", rockGrabContactPatchMeshSnapMaxDistanceGameUnits));
        if (!std::isfinite(rockGrabContactPatchMeshSnapMaxDistanceGameUnits) || rockGrabContactPatchMeshSnapMaxDistanceGameUnits < 0.0f) {
            rockGrabContactPatchMeshSnapMaxDistanceGameUnits = 6.0f;
        }
        rockGrabContactPatchMaxNormalAngleDegrees =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabContactPatchMaxNormalAngleDegrees", rockGrabContactPatchMaxNormalAngleDegrees));
        rockGrabContactPatchMaxNormalAngleDegrees = std::clamp(rockGrabContactPatchMaxNormalAngleDegrees, 0.0f, 179.0f);
        rockGrabAlignmentMaxSelectionToMeshDistance =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fGrabAlignmentMaxSelectionToMeshDistance", rockGrabAlignmentMaxSelectionToMeshDistance));
        if (!std::isfinite(rockGrabAlignmentMaxSelectionToMeshDistance)) {
            rockGrabAlignmentMaxSelectionToMeshDistance = 8.0f;
        }
        rockGrabNodeAnchorsEnabled = ini.GetBoolValue(SECTION, "bGrabNodeAnchorsEnabled", rockGrabNodeAnchorsEnabled);
        rockGrabNodeRejectOppositeHandAnchor = ini.GetBoolValue(SECTION, "bGrabNodeRejectOppositeHandAnchor", rockGrabNodeRejectOppositeHandAnchor);
        rockPrintGrabNodeInfo = ini.GetBoolValue(SECTION, "bPrintGrabNodeInfo", rockPrintGrabNodeInfo);
        rockGrabNodeNameRight =
            grab_node_name_policy::sanitizeConfiguredGrabNodeName(ini.GetValue(SECTION, "sGrabNodeNameRight", rockGrabNodeNameRight.c_str()), false);
        rockGrabNodeNameLeft = grab_node_name_policy::sanitizeConfiguredGrabNodeName(ini.GetValue(SECTION, "sGrabNodeNameLeft", rockGrabNodeNameLeft.c_str()), true);
        rockGrabNodeNameBlacklist = ini.GetValue(SECTION, "sGrabNodeNameBlacklist", rockGrabNodeNameBlacklist.c_str());
        rockSelectedCloseFingerCurlEnabled = ini.GetBoolValue(SECTION, "bSelectedCloseFingerCurlEnabled", rockSelectedCloseFingerCurlEnabled);
        rockSelectedCloseFingerAnimMaxHandSpeed =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fSelectedCloseFingerAnimMaxHandSpeed", rockSelectedCloseFingerAnimMaxHandSpeed));
        if (!std::isfinite(rockSelectedCloseFingerAnimMaxHandSpeed) || rockSelectedCloseFingerAnimMaxHandSpeed < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fSelectedCloseFingerAnimMaxHandSpeed={} -- using 0.9", rockSelectedCloseFingerAnimMaxHandSpeed);
            rockSelectedCloseFingerAnimMaxHandSpeed = 0.9f;
        }
        rockSelectedCloseFingerAnimValue = static_cast<float>(ini.GetDoubleValue(SECTION, "fSelectedCloseFingerAnimValue", rockSelectedCloseFingerAnimValue));
        if (!std::isfinite(rockSelectedCloseFingerAnimValue)) {
            ROCK_LOG_WARN(Config, "Invalid fSelectedCloseFingerAnimValue={} -- using 0.9", rockSelectedCloseFingerAnimValue);
            rockSelectedCloseFingerAnimValue = 0.9f;
        }
        rockSelectedCloseFingerAnimValue = std::clamp(rockSelectedCloseFingerAnimValue, 0.0f, 1.0f);
        rockPulledAngularDamping = static_cast<float>(ini.GetDoubleValue(SECTION, "fPulledAngularDamping", rockPulledAngularDamping));
        rockPulledGrabHandAdjustDistanceGameUnits =
            static_cast<float>(ini.GetDoubleValue(SECTION, "fPulledGrabHandAdjustDistanceGameUnits", rockPulledGrabHandAdjustDistanceGameUnits));
        if (!std::isfinite(rockPulledGrabHandAdjustDistanceGameUnits) || rockPulledGrabHandAdjustDistanceGameUnits < 0.0f) {
            ROCK_LOG_WARN(Config, "Invalid fPulledGrabHandAdjustDistanceGameUnits={} -- using 10.5", rockPulledGrabHandAdjustDistanceGameUnits);
            rockPulledGrabHandAdjustDistanceGameUnits = 10.5f;
        }

        readOptionalVec3("fRightGrabLegacyPalmPivotAHandspaceX", "fRightGrabLegacyPalmPivotAHandspaceY", "fRightGrabLegacyPalmPivotAHandspaceZ", rockRightGrabLegacyPalmPivotAHandspace);
        readOptionalVec3("fLeftGrabLegacyPalmPivotAHandspaceX", "fLeftGrabLegacyPalmPivotAHandspaceY", "fLeftGrabLegacyPalmPivotAHandspaceZ", rockLeftGrabLegacyPalmPivotAHandspace);

        auto readClampedFloat = [&](const char* key, float& value, float fallback, float minValue, float maxValue) {
            value = static_cast<float>(ini.GetDoubleValue(SECTION, key, value));
            if (!std::isfinite(value)) {
                ROCK_LOG_WARN(Config, "Invalid {}={} -- using {}", key, value, fallback);
                value = fallback;
            }
            value = std::clamp(value, minValue, maxValue);
        };

        rockShoulderStashEnabled = ini.GetBoolValue(SECTION, "bShoulderStashEnabled", rockShoulderStashEnabled);
        rockShoulderStashUseBodyZoneColliders =
            ini.GetBoolValue(SECTION, "bShoulderStashUseBodyZoneColliders", rockShoulderStashUseBodyZoneColliders);
        rockShoulderStashUseHmdBackVolume =
            ini.GetBoolValue(SECTION, "bShoulderStashUseHmdBackVolume", rockShoulderStashUseHmdBackVolume);
        readClampedFloat("fShoulderStashEnterPaddingGameUnits", rockShoulderStashEnterPaddingGameUnits, 5.0f, 0.0f, 40.0f);
        readClampedFloat("fShoulderStashExitPaddingGameUnits", rockShoulderStashExitPaddingGameUnits, 8.0f, 0.0f, 60.0f);
        readClampedFloat("fShoulderStashMinDwellSeconds", rockShoulderStashMinDwellSeconds, 0.08f, 0.0f, 1.0f);
        readClampedFloat("fShoulderStashMaxSpeedGameUnitsPerSecond", rockShoulderStashMaxSpeedGameUnitsPerSecond, 140.0f, 0.0f, 1000.0f);
        rockShoulderStashRecentContactFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iShoulderStashRecentContactFrames", rockShoulderStashRecentContactFrames));
        rockShoulderStashRecentContactFrames = std::clamp(rockShoulderStashRecentContactFrames, 0, 60);
        rockShoulderStashSustainedContactMissFrames =
            static_cast<int>(ini.GetLongValue(SECTION, "iShoulderStashSustainedContactMissFrames", rockShoulderStashSustainedContactMissFrames));
        rockShoulderStashSustainedContactMissFrames = std::clamp(rockShoulderStashSustainedContactMissFrames, 0, 120);
        readOptionalVec3("fShoulderStashHmdBackRightOffsetXGameUnits",
            "fShoulderStashHmdBackRightOffsetYGameUnits",
            "fShoulderStashHmdBackRightOffsetZGameUnits",
            rockShoulderStashHmdBackRightOffsetGameUnits);
        readOptionalVec3("fShoulderStashHmdBackLeftOffsetXGameUnits",
            "fShoulderStashHmdBackLeftOffsetYGameUnits",
            "fShoulderStashHmdBackLeftOffsetZGameUnits",
            rockShoulderStashHmdBackLeftOffsetGameUnits);
        readClampedFloat("fShoulderStashHmdBackRadiusGameUnits", rockShoulderStashHmdBackRadiusGameUnits, 11.0f, 1.0f, 80.0f);
        readClampedFloat(
            "fShoulderStashHmdBackEnterPaddingGameUnits", rockShoulderStashHmdBackEnterPaddingGameUnits, 0.0f, 0.0f, 40.0f);
        readClampedFloat(
            "fShoulderStashHmdBackExitPaddingGameUnits", rockShoulderStashHmdBackExitPaddingGameUnits, 2.0f, 0.0f, 60.0f);
        readClampedFloat(
            "fShoulderStashHmdBackMinBehindGameUnits", rockShoulderStashHmdBackMinBehindGameUnits, 4.0f, 0.0f, 40.0f);
        rockShoulderStashShowCollectedNotifications =
            ini.GetBoolValue(SECTION, "bShoulderStashShowCollectedNotifications", rockShoulderStashShowCollectedNotifications);

        rockGrabHapticsEnabled = ini.GetBoolValue(SECTION, "bGrabHapticsEnabled", rockGrabHapticsEnabled);
        readClampedFloat("fGrabHapticDurationSeconds", rockGrabHapticDurationSeconds, 0.055f, 0.0f, 0.2f);
        readClampedFloat("fGrabHapticBaseIntensity", rockGrabHapticBaseIntensity, 0.12f, 0.0f, 1.0f);
        readClampedFloat("fGrabHapticMaxIntensity", rockGrabHapticMaxIntensity, 0.80f, rockGrabHapticBaseIntensity, 1.0f);
        readClampedFloat("fGrabHapticMassScale", rockGrabHapticMassScale, 0.06f, 0.0f, 1.0f);
        readClampedFloat("fGrabHapticMassExponent", rockGrabHapticMassExponent, 0.60f, 0.0f, 2.0f);
        readClampedFloat("fPullStartHapticIntensity", rockPullStartHapticIntensity, 0.18f, 0.0f, 1.0f);
        readClampedFloat("fPullCatchHapticIntensity", rockPullCatchHapticIntensity, 0.22f, 0.0f, 1.0f);
        readClampedFloat("fSelectionLockHapticIntensity", rockSelectionLockHapticIntensity, 0.15f, 0.0f, 1.0f);
        readClampedFloat("fSelectionLockReleaseHapticIntensity", rockSelectionLockReleaseHapticIntensity, 0.10f, 0.0f, 1.0f);
        readClampedFloat("fSelectionLockReleaseHapticDurationSeconds", rockSelectionLockReleaseHapticDurationSeconds, 0.02f, 0.0f, 0.2f);
        rockHeldImpactHapticsEnabled = ini.GetBoolValue(SECTION, "bHeldImpactHapticsEnabled", rockHeldImpactHapticsEnabled);
        readClampedFloat("fHeldImpactHapticDurationSeconds", rockHeldImpactHapticDurationSeconds, 0.035f, 0.0f, 0.2f);
        readClampedFloat("fHeldImpactHapticBaseIntensity", rockHeldImpactHapticBaseIntensity, 0.12f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticMaxIntensity", rockHeldImpactHapticMaxIntensity, 0.85f, rockHeldImpactHapticBaseIntensity, 1.0f);
        readClampedFloat("fHeldImpactHapticSpeedScale", rockHeldImpactHapticSpeedScale, 0.006f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticMassScale", rockHeldImpactHapticMassScale, 0.035f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticMassExponent", rockHeldImpactHapticMassExponent, 0.55f, 0.0f, 2.0f);
        readClampedFloat("fHeldImpactHapticMinSpeedGameUnits", rockHeldImpactHapticMinSpeedGameUnits, 8.0f, 0.0f, 1000.0f);
        readClampedFloat("fHeldImpactHapticCooldownSeconds", rockHeldImpactHapticCooldownSeconds, 0.12f, 0.0f, 1.0f);
        readClampedFloat("fHeldImpactHapticDampedMultiplier", rockHeldImpactHapticDampedMultiplier, 0.55f, 0.0f, 1.0f);
        rockShoulderStashHapticsEnabled =
            ini.GetBoolValue(SECTION, "bShoulderStashHapticsEnabled", rockShoulderStashHapticsEnabled);
        readClampedFloat(
            "fShoulderStashCandidateHapticDurationSeconds", rockShoulderStashCandidateHapticDurationSeconds, 0.075f, 0.0f, 0.2f);
        readClampedFloat(
            "fShoulderStashCandidateHapticBaseIntensity", rockShoulderStashCandidateHapticBaseIntensity, 0.20f, 0.0f, 1.0f);
        readClampedFloat("fShoulderStashCandidateHapticIntensity",
            rockShoulderStashCandidateHapticIntensity,
            0.42f,
            rockShoulderStashCandidateHapticBaseIntensity,
            1.0f);
        readClampedFloat("fShoulderStashCandidateHapticIntervalSeconds", rockShoulderStashCandidateHapticIntervalSeconds, 0.075f, 0.0f, 2.0f);
        readClampedFloat(
            "fShoulderStashCommitHapticDurationSeconds", rockShoulderStashCommitHapticDurationSeconds, 0.12f, 0.0f, 0.2f);
        readClampedFloat("fShoulderStashCommitHapticIntensity", rockShoulderStashCommitHapticIntensity, 0.85f, 0.0f, 1.0f);

    }

    void RockConfig::load()
    {
        _iniFilePath = resolveIniPath();
        ROCK_LOG_INFO(Config, "Loading ROCK config from: {}", _iniFilePath);

        f4cf::common::createDirDeep(_iniFilePath);

        f4cf::common::createFileFromResourceIfNotExists(_iniFilePath, "ROCK", IDR_ROCK_INI, true);

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini not found or unreadable (code {}), using compiled-in defaults", static_cast<int>(rc));
        }

        resetToDefaults();
        readValuesFromIni(ini);

        ROCK_LOG_INFO(Config,
            "ROCK config loaded (rockEnabled={}, logLevel={} {}, sample={}ms)",
            rockEnabled,
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds);

        startFileWatch();
    }

    void RockConfig::reload()
    {
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "reload() called before load() — delegating to load()");
            load();
            return;
        }

        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error rc = ini.LoadFile(_iniFilePath.c_str());
        if (rc < 0) {
            ROCK_LOG_WARN(Config, "ROCK.ini reload failed (code {}), retaining current values", static_cast<int>(rc));
            return;
        }

        resetToDefaults();
        readValuesFromIni(ini);
        ROCK_LOG_INFO(Config,
            "ROCK config reloaded (rockEnabled={}, logLevel={} {}, sample={}ms, grabRagdollDecompMode={})",
            rockEnabled,
            rockLogLevel,
            logging_policy::logLevelName(rockLogLevel),
            rockLogSampleMilliseconds,
            rockGrabRagdollDecompositionMode);
    }

    std::filesystem::path RockConfig::getConfigDirectory() const
    {
        if (_iniFilePath.empty()) {
            return std::filesystem::path(resolveIniPath()).parent_path();
        }
        return std::filesystem::path(_iniFilePath).parent_path();
    }

    bool RockConfig::saveRuntimeIni(CSimpleIniA& ini, const char* reason)
    {
        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        _selfIniWriteInProgress.store(true, std::memory_order_release);
        suppressNextFileWatchReload();

        const SI_Error saveRc = ini.SaveFile(path.c_str(), false);
        std::error_code ec;
        const auto writeTime = std::filesystem::last_write_time(path, ec);
        if (!ec) {
            _lastSelfIniWriteTime.store(writeTime, std::memory_order_release);
            _lastIniFileWriteTime.store(writeTime, std::memory_order_release);
        }

        _selfIniWriteInProgress.store(false, std::memory_order_release);
        _ignoreNextIniFileChange.store(false, std::memory_order_release);

        if (saveRc < 0) {
            ROCK_LOG_WARN(Config, "Failed to persist ROCK.ini runtime change '{}' (code {})", reason ? reason : "unknown", static_cast<int>(saveRc));
            return false;
        }

        ROCK_LOG_DEBUG(Config, "Persisted ROCK.ini runtime change '{}'", reason ? reason : "unknown");
        return true;
    }

    bool RockConfig::persistPhysicsBool(const char* key, bool value)
    {
        if (!key || !key[0]) {
            return false;
        }

        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error loadRc = ini.LoadFile(path.c_str());
        if (loadRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini bool '{}': load failed with code {}", key, static_cast<int>(loadRc));
            return false;
        }

        const SI_Error setRc = ini.SetBoolValue(SECTION, key, value, nullptr, true);
        if (setRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini bool '{}': set failed with code {}", key, static_cast<int>(setRc));
            return false;
        }

        return saveRuntimeIni(ini, key);
    }

    bool RockConfig::persistGrabLegacyPalmPivotAHandspace(bool isLeft, const RE::NiPoint3& value)
    {
        const std::string path = _iniFilePath.empty() ? resolveIniPath() : _iniFilePath;
        CSimpleIniA ini;
        ini.SetUnicode(false);
        const SI_Error loadRc = ini.LoadFile(path.c_str());
        if (loadRc < 0) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini {} legacy palm pivot A: load failed with code {}", isLeft ? "left" : "right", static_cast<int>(loadRc));
            return false;
        }

        const char* keyX = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceX" : "fRightGrabLegacyPalmPivotAHandspaceX";
        const char* keyY = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceY" : "fRightGrabLegacyPalmPivotAHandspaceY";
        const char* keyZ = isLeft ? "fLeftGrabLegacyPalmPivotAHandspaceZ" : "fRightGrabLegacyPalmPivotAHandspaceZ";
        bool ok = true;
        ok &= ini.SetDoubleValue(SECTION, keyX, value.x, nullptr, true) >= 0;
        ok &= ini.SetDoubleValue(SECTION, keyY, value.y, nullptr, true) >= 0;
        ok &= ini.SetDoubleValue(SECTION, keyZ, value.z, nullptr, true) >= 0;
        if (!ok) {
            ROCK_LOG_WARN(Config, "Cannot persist ROCK.ini {} legacy palm pivot A: set failed", isLeft ? "left" : "right");
            return false;
        }

        return saveRuntimeIni(ini, isLeft ? "left legacy palm pivot A" : "right legacy palm pivot A");
    }

    void RockConfig::processPendingConfigReload()
    {
        if (!_reloadPending.exchange(false, std::memory_order_acq_rel)) {
            return;
        }

        ROCK_LOG_INFO(Config, "ROCK.ini change detected, reloading on frame thread...");
        reload();

        for (const auto& [key, subscriber] : _onConfigChangedSubscribers) {
            ROCK_LOG_DEBUG(Config, "Notify config change subscriber '{}'", key);
            subscriber(key);
        }
    }

    void RockConfig::startFileWatch()
    {
        if (_fileWatch) {
            return;
        }
        if (_iniFilePath.empty()) {
            ROCK_LOG_WARN(Config, "Cannot start file watch — INI path not resolved");
            return;
        }

        if (_fileWatchInitThread.joinable()) {
            _fileWatchInitThread.join();
        }

        _fileWatchInitThread = std::thread([this]() {
            ROCK_LOG_DEBUG(Config, "Starting file watch on '{}'", _iniFilePath);

            _fileWatch = std::make_unique<filewatch::FileWatch<std::string>>(_iniFilePath, [this](const std::string&, const filewatch::Event changeType) {
                if (changeType != filewatch::Event::modified) {
                    return;
                }

                constexpr auto delay = std::chrono::milliseconds(200);

                auto prevWriteTime = _lastIniFileWriteTime.load();
                std::error_code ec;
                const auto writeTime = std::filesystem::last_write_time(_iniFilePath, ec);
                if (ec || writeTime - prevWriteTime < delay) {
                    return;
                }

                const auto selfWriteTime = _lastSelfIniWriteTime.load(std::memory_order_acquire);
                if (_selfIniWriteInProgress.load(std::memory_order_acquire) ||
                    (selfWriteTime != std::filesystem::file_time_type{} && writeTime <= selfWriteTime)) {
                    _lastIniFileWriteTime.store(writeTime, std::memory_order_release);
                    if (!_selfIniWriteInProgress.load(std::memory_order_acquire)) {
                        _lastSelfIniWriteTime.store(std::filesystem::file_time_type{}, std::memory_order_release);
                    }
                    _ignoreNextIniFileChange.store(false, std::memory_order_release);
                    return;
                }

                if (!_lastIniFileWriteTime.compare_exchange_strong(prevWriteTime, writeTime)) {
                    return;
                }

                bool expected = true;
                if (_ignoreNextIniFileChange.compare_exchange_strong(expected, false)) {
                    return;
                }

                auto now = std::filesystem::file_time_type::clock::now();
                auto lastEventTime = _lastIniFileWriteTime.load();
                while (now - lastEventTime < delay) {
                    std::this_thread::sleep_for(std::max(std::chrono::milliseconds(0), std::chrono::duration_cast<std::chrono::milliseconds>(delay - (now - lastEventTime))));
                    now = std::filesystem::file_time_type::clock::now();
                    lastEventTime = _lastIniFileWriteTime.load();
                }

                _reloadPending.store(true, std::memory_order_release);
            });
        });

        _fileWatchInitThread.join();
    }

    void RockConfig::stopFileWatch()
    {
        if (_fileWatchInitThread.joinable()) {
            _fileWatchInitThread.join();
        }
        if (_fileWatch) {
            ROCK_LOG_DEBUG(Config, "Stopping file watch on ROCK.ini");
            _fileWatch.reset();
        }
    }

    void RockConfig::subscribeForConfigChanged(const std::string& key, std::function<void(const std::string&)> callback) { _onConfigChangedSubscribers[key] = std::move(callback); }

    void RockConfig::unsubscribeFromConfigChanged(const std::string& key) { _onConfigChangedSubscribers.erase(key); }
}

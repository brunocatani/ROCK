param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

$removedRuntimeFiles = @(
    'src/physics-interaction/animation/NativeAnimationAuthority.cpp',
    'src/physics-interaction/animation/NativeAnimationAuthority.h',
    'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h'
)
foreach ($relativePath in $removedRuntimeFiles) {
    if (Test-Path -LiteralPath (Join-Path $Root $relativePath)) {
        $failures.Add("ROCK still owns removed reload/bolt runtime file '$relativePath'.")
    }
}

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_UpdateFirstPersonArm\s*=\s*0xEF6280[\s\S]*kCallsite_UpdateFirstPersonArmPrimaryReturn\s*=\s*0xEF610D[\s\S]*kCallsite_UpdateFirstPersonArmSecondaryReturn\s*=\s*0xEF6150[\s\S]*kFunc_PlayerPostUpdateAnimationGraphManager\s*=\s*0xF2F0A0' 'Authored grip capture must retain the independently verified graph-output and paired arm offsets.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'kExpectedUpdateFirstPersonArmPrefix[\s\S]*0x48,\s*0x8B,\s*0xC4,\s*0x55,\s*0x53,\s*0x41,\s*0x56,[\s\S]*0x48,\s*0x8D,\s*0xA8,\s*0xF8,\s*0xFE,\s*0xFF,\s*0xFF[\s\S]*kFunc_UpdateFirstPersonArm' 'The ROCK-owned arm hook must retain byte validation.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'kExpectedPostFrikPrefix[\s\S]*kFunc_PlayerPostUpdateAnimationGraphManager[\s\S]*onPostUpdateAnimationGraphManager' 'The shared graph-output coordinator must retain byte validation against hFRIK''s verified patch identity.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'onPostUpdateAnimationGraphManager[\s\S]*NativeGraphOutput[\s\S]*captureAuthoredSupportGraphPose\(\)[\s\S]*s_originalPostUpdate' 'Addon capture and ROCK grip capture must share the proven pre-presentation graph-output boundary.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 's_originalUpdateFirstPersonArm[\s\S]*captureNativePrimaryFiringGrip' 'The later primary arm pass must pair the graph sample with the exact weapon-specific primary relation.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'currentNativeAnimationAuthorityFlagsV1\(\)\s*!=\s*0' 'Authored grip capture must yield to ROCK V1 animation authority owners.'
Reject-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapture.cpp' 'WeaponFireHandler|ReloadStateChangeHandler|setLocalReload|ManualCycle|applyCapturedPose' 'ROCK-authored grip capture must not retain reload or bolt runtime ownership.'

Require-Text 'src/ROCKMain.cpp' 'dispatchAnimationPhaseCallbacksV1\([\s\S]*BeforeRock[\s\S]*onFrameUpdate\(\)[\s\S]*AfterRock[\s\S]*Complete' 'ROCK must expose ordered addon animation phases around its update.'
Reject-Text 'src/ROCKMain.cpp' 'setLocalReloadTestEnabled|setLocalManualCycleTestEnabled|applyCapturedPose|installPostUpdateHook' 'ROCK main must not execute the separated reload/bolt runtime.'
Require-Text 'src/ROCKMain.cpp' 'authored_weapon_grip_capture::installHook\(\)' 'ROCK must retain its authored equipped-weapon grip capture hook.'
Require-Text 'src/ROCKMain.cpp' 'authored_weapon_grip_capture::setEnabled\(\s*authoredGripCaptureRuntimeEnabled\s*\)' 'ROCK must unconditionally run authored equipped-weapon grip capture while the runtime is eligible.'
Require-Text 'src/api/ROCKProviderApi.h' 'enum class RockProviderAnimationPhaseV1[\s\S]*NativeGraphOutput' 'ROCK V1 must expose the proven native graph-output capture phase.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'phaseFrameIndex\s*=\s*s_activeAnimationPhaseFrameIndex[\s\S]*if \(phaseFrameIndex == 0\)' 'NativeGraphOutput and the later ROCK phases must share one frame identity.'
Reject-Text 'src/api/ROCKProviderApi.cpp' 'phase == RockProviderAnimationPhaseV1::BeforeRock\s*\|\|' 'BeforeRock must not replace the frame identity opened by NativeGraphOutput.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'phase == RockProviderAnimationPhaseV1::Complete[\s\S]*s_activeAnimationPhaseFrameIndex\.store\(0' 'The shared animation phase frame must close deterministically at Complete.'

Require-Text 'src/api/ROCKProviderApi.h' 'AnimationPhases[\s\S]*EquippedWeaponGripState[\s\S]*HandVisualAuthority[\s\S]*NativeAnimationRuntimeProvider' 'ROCK V1 must publish the complete Reanimate support capability surface.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiSetNativeAnimationAuthorityV1[\s\S]*apiIsProviderReady\(\)[\s\S]*NativeAnimationAuthoritySlot' 'ROCK must coordinate animation leases without requiring an in-process animation hook.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'apiPublishNativeAnimationRuntimeV1[\s\S]*s_nativeAnimationRuntimePublication' 'ROCK must accept status publication from the runtime addon.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'queryProviderEquippedWeaponGripStateV1[\s\S]*RightHandInWeaponValid[\s\S]*LeftHandInWeaponValid' 'ROCK must expose exact equipped-weapon grip baselines to addons.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'leftHandHoldingObject\s*=\s*_leftHand\.isHolding\(\)[\s\S]*rightHandHoldingObject\s*=\s*_rightHand\.isHolding\(\)[\s\S]*setGrabbedObjectHandPoseOwnership\(\s*leftHandHoldingObject,\s*rightHandHoldingObject\s*\)[\s\S]*_authoredPrimaryFiringGrip\.update' 'The authored-grip phase must publish current per-physical-hand ROCK grab ownership before an equipped firing pose can be selected.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'void TwoHandedGrip::update\([\s\S]*setGrabbedObjectHandPoseOwnership\(\s*frameInput\.leftHandHoldingObject,\s*frameInput\.rightHandHoldingObject\s*\)[\s\S]*switch \(_state\)' 'The equipped-weapon state machine must refresh per-hand ROCK grab ownership before any state can publish a firing pose.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'publishAuthoredPrimaryFiringGripFingerPose[\s\S]*targetHandHoldingObject[\s\S]*shouldPublishAuthoredFiringFingerPose\(\s*targetHandHoldingObject\s*\)' 'Every authored firing-finger publication must yield to an ordinary ROCK grab on the target physical hand.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'setGrabbedObjectHandPoseOwnership[\s\S]*publishedHandHoldingObject[\s\S]*clearAuthoredPrimaryFiringGripFingerPose\(\)' 'Acquiring a ROCK object must immediately clear an already-published equipped firing pose without ending equipped-weapon transform ownership.'
Require-Text 'src/physics-interaction/animation/AuthoredWeaponGripCapturePolicy.h' 'AuthoredSupportGripCandidateInput[\s\S]*authoredSeatWeaponSurfaceValid[\s\S]*shouldUseAuthoredSupportGrip\([\s\S]*input\.authoredSeatWeaponSurfaceValid' 'Authored support selection must require a current weapon-surface witness in addition to capture freshness and completeness.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryFindCurrentWeaponSurfaceNearPoint\(\s*weaponNode,\s*authoredSupportProximity\.authoredPalmSeatWorld,\s*g_rockConfig\.rockWeaponInteractionTouchRadius,\s*authoredSupportSurfaceWitness\)' 'The resolved authored palm seat must be checked against current weapon geometry with the small surface-vicinity radius.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'shouldUseAuthoredSupportGrip\([\s\S]*\.authoredSeatWeaponSurfaceValid\s*=\s*authoredSeatWeaponSurfaceValid[\s\S]*if \(useAuthoredSupportGrip\)[\s\S]*return true;[\s\S]*tryGetSupportGripEvidenceView\(decision\.bodyId, weaponNode, evidenceView\)' 'An off-weapon authored support candidate must fall through to the existing dynamic mesh/finger solver for that hand.'

Reject-Text 'src/RockConfig.h' 'rockNativeReloadAnimationAuthorityTestEnabled|rockNativeReloadAnimationPartialAuthorityTestEnabled' 'Reload validation configuration must not remain in ROCK.'
Reject-Text 'src/RockConfig.h' 'rockAuthoredPrimaryFiringGripTestEnabled' 'The production authored primary/equipped-grip path must not retain an experimental config switch.'
Reject-Text 'data/config/ROCK.ini' 'bAuthoredPrimaryFiringGripTestEnabled' 'Users must not be able to disable the production authored primary/equipped-grip path.'
Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' 'input\.enabled|experiment-disabled' 'The production authored primary grip runtime must not retain its removed experimental gate.'

if ($failures.Count -gt 0) {
    Write-Host 'AuthoredWeaponGripCaptureSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'AuthoredWeaponGripCaptureSourceTests passed.' -ForegroundColor Green

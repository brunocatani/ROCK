param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/RockConfig.h' 'rockWeaponSupportGripHandLerpEnabled[\s\S]*rockWeaponSupportGripHandLerpTimeMin[\s\S]*rockWeaponSupportGripHandLerpTimeMax[\s\S]*rockWeaponSupportGripHandLerpMinDistance[\s\S]*rockWeaponSupportGripHandLerpMaxDistance' `
    'RockConfig must expose weapon support grip hand visual lerp settings.'
Require-Text 'src/RockConfig.cpp' 'bWeaponSupportGripHandLerpEnabled[\s\S]*fWeaponSupportGripHandLerpTimeMin[\s\S]*fWeaponSupportGripHandLerpTimeMax[\s\S]*fWeaponSupportGripHandLerpMinDistance[\s\S]*fWeaponSupportGripHandLerpMaxDistance' `
    'RockConfig must load weapon support grip hand visual lerp settings from ROCK.ini.'
Require-Text 'src/RockConfig.cpp' 'bWeaponSupportSurfaceSeatEnabled[\s\S]*fWeaponSupportSurfaceSeatMaxDegrees' `
    'RockConfig must load the bounded support surface-seat policy.'

Require-Text 'src/physics-interaction/hand/HandVisual.h' 'computeDistanceMappedDurationGameUnits[\s\S]*blendTransformOverDuration' `
    'Visual hand helper must expose time-based distance-mapped transform blending.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeDistanceMappedDurationGameUnits\([\s\S]*rockGrabHandLerpTimeMin[\s\S]*rockGrabHandLerpTimeMax[\s\S]*rockGrabHandLerpMinDistance[\s\S]*rockGrabHandLerpMaxDistance' `
    'Normal grab visual hand smoothing must use the explicit grab hand lerp duration settings.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'resolveLockedHandVisualTarget[\s\S]*rockWeaponSupportGripHandLerpEnabled[\s\S]*blendTransformOverDuration' `
    'Authored/provider/visual-only support paths must retain the established external-hand transition.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'shouldUseDynamicSupportAcquisition[\s\S]{0,500}FullTwoHandedSolver[\s\S]{0,300}!authoredSupportGrip[\s\S]{0,200}!providerAuthorityActive[\s\S]{0,200}!attachOnly' `
    'Synchronized acquisition must be eligible only for normal non-authored full-authority support grips.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'shortestArcSlerpFromIdentity[\s\S]*applyRotationAroundPrimaryPivot[\s\S]*localPointToWorld\([\s\S]*primaryGripLocal[\s\S]*primaryTargetWorld' `
    'Dynamic acquisition math must shortest-arc slerp the composite correction and re-solve translation from the live primary pivot.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'shouldUseDynamicSupportAcquisition\([\s\S]{0,600}supportGrip\.authoredSupportGrip[\s\S]{0,300}supportGrip\.providerPartAuthority\.active[\s\S]{0,300}supportGrip\.attachOnly[\s\S]{0,6000}beginDynamicSupportAcquisition\([\s\S]{0,900}updateFullWeaponAuthorityGrip\(weaponNode,\s*0\.0f\)' `
    'A successful normal dynamic capture must start its witnessed transaction and publish exact alpha zero before transitionToGripping returns.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'useDynamicSupportAcquisition[\s\S]*initializeDynamicSupportBaseline\([\s\S]*supportBaselineAttachPublication[\s\S]*weaponWorldAtCapture' `
    'Normal dynamic support must capture paired driver-to-seat baselines and force an exact zero-delta first weapon publication.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'tryCaptureSupportInputBaseline\([\s\S]*invertTransform\(supportInputWorld\)[\s\S]*supportGripTargetWorld[\s\S]*tryResolveSupportInputTarget\(' `
    'Dynamic support must keep the rigid post-capture input calibration primitive.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'tryCaptureDynamicSupportDriverBaseline\([\s\S]*primaryDriverWorld[\s\S]*primaryGripTargetWorld[\s\S]*supportDriverWorld[\s\S]*supportGripTargetWorld[\s\S]*tryResolveDynamicSupportDriverTargets\(' `
    'Normal dynamic support must capture both physical driver relations as one transaction.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' '_currentHandDriverFrames\[0\]\s*=\s*frameInput\.leftHandDriverFrame[\s\S]*_currentHandDriverFrames\[1\]\s*=\s*frameInput\.rightHandDriverFrame[\s\S]*tryResolveDynamicSupportDriverTargets\([\s\S]*calibratedPrimaryTransform[\s\S]*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*tryCaptureDynamicSupportDriverBaseline\([\s\S]*pairedDynamicDrivers\s*=\s*true' `
    'Dynamic solver inputs must come from paired frame-scoped drivers, not ROCK-rendered hand roots.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryResolveDynamicSupportDriverTargets\([\s\S]*calibratedPrimaryTransform[\s\S]*applyLockedHandVisualAuthority\(weaponNode,\s*applyPrimaryHandAuthority,\s*true,\s*dt,\s*&primaryTransform,\s*&supportTransform\)' `
    'Controller-isolated transforms must steer the weapon while original rendered transforms remain visual interpolation starts only.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'solverInput\.supportTargetWorld\s*=\s*dynamicAcquisition\s*\?[\s\S]{0,300}lockedSupportControllerTarget[\s\S]*applyRotationAroundPrimaryPivot' `
    'Dynamic acquisition must solve the complete locked target before applying one partial composite rotation.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'supportGrip\.authoredSupportGrip[\s\S]{0,500}!supportGrip\.providerPartAuthority\.active[\s\S]{0,300}!supportGrip\.attachOnly[\s\S]{0,700}updateFullWeaponAuthorityGrip\(weaponNode,\s*0\.0f\)' `
    'Authored full-authority support must publish an exact zero-delta attach frame before its first timed correction.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'supportNormalTwistFactor\s*=\s*SUPPORT_NORMAL_TWIST_FACTOR\s*\*[\s\S]{0,300}supportGrip\.authoredSupportGrip\s*&&\s*!dynamicAcquisition\s*\?[\s\S]{0,120}_rotationBlend' `
    'Authored support palm-normal twist must use the same acquisition ramp as its positional aim correction.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'resolveDynamicSupportAcquisitionHandTarget[\s\S]{0,1200}_dynamicSupportAcquisition\.easedAlpha[\s\S]*synchronizedDynamicAcquisition[\s\S]*resolveDynamicSupportAcquisitionHandTarget' `
    'Dynamic primary and support hand roots must consume the same eased acquisition alpha as weapon steering.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'supportGripAppliesPrimaryHandAuthority\(_authorityMode\)[\s\S]*applyLockedHandVisualAuthority\(weaponNode,\s*applyPrimaryHandAuthority,\s*true,\s*dt,\s*&primaryTransform,\s*&supportTransform\)' `
    'Full two-handed weapon authority must gate primary visual authority while preserving live hand-frame inputs.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'supportBaselineAttachPublication[\s\S]{0,700}weaponWorldAtCapture[\s\S]{0,700}applyWeaponVisualAuthority\(weaponNode, appliedWeaponWorld\)[\s\S]{0,1200}applyLockedHandVisualAuthority\(weaponNode,\s*applyPrimaryHandAuthority,\s*true,\s*dt,\s*&primaryTransform,\s*&supportTransform\)' `
    'Dynamic attach must preserve the weapon while retaining the established live-input visual hand transition.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'transitionToPrimaryOnly[\s\S]*clearPrimaryGripFingerPose\(\s*primaryHandIsLeft,\s*_returningWeaponVisual\.localTransition\.active\s*&&\s*_returningWeaponVisual\.followsAuthoredPrimaryGrip\)[\s\S]*clearPrimaryGripWorldAuthority\(primaryHandIsLeft\)[\s\S]*restoreFrikPrimaryWeaponPose' `
    'Primary-only ownership must clear the locked hand root while retaining an in-flight authored firing pose through the return handoff.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'tryResolveAuthoredPrimaryWeaponReturnTargetLocal[\s\S]*tryGetAuthoredPrimaryTrackedFiringHandWorld[\s\S]*nativeWeaponWorld[\s\S]*resolveAuthoredPrimaryWeaponWorldPositionOnly[\s\S]*worldTargetToParentLocal' `
    'A right-firing weapon return must use the physical driver and live native weapon frame consumed by mandatory position-only alignment.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'state\.followsAuthoredPrimaryGrip[\s\S]*state\.lastTargetLocal[\s\S]*driveVisualReturn\(\s*state\.localTransition,\s*targetLocal' `
    'The weapon return must retain the last valid authored endpoint and never fall back to the native offset mid-handoff.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'updateVisualOnlySupportGrip\(RE::NiNode\* weaponNode,\s*float dt\)[\s\S]*applyLockedHandVisualAuthority\(weaponNode,\s*false,\s*true,\s*dt' `
    'Visual-only sidearm support grip must use the same support hand visual lerp path.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'event=start[\s\S]{0,1200}captureToFirstPublicationFrames=0[\s\S]*event=complete' `
    'Dynamic acquisition telemetry must record start and completion without a per-frame hot log.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'dynamic support acquisition event=cancel' `
    'Dynamic acquisition lifecycle cleanup must emit an event-scoped cancellation record.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'handLerp=\(' `
    'Two-handed weapon telemetry must retain sampled hand interpolation alpha/duration.'
Require-Text 'data/config/ROCK_example.ini' 'Normal dynamic full-authority grabs[\s\S]*authored,[\s\S]*provider-owned,[\s\S]*AttachOnly,[\s\S]*visual-only' `
    'Repository config must describe the synchronized dynamic-acquisition semantics and unchanged paths.'

$weaponText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp')
$transitionStart = $weaponText.IndexOf('void TwoHandedGrip::transitionToGripping')
$transitionEnd = if ($transitionStart -ge 0) { $weaponText.IndexOf('void TwoHandedGrip::transitionToInactive', $transitionStart) } else { -1 }
if ($transitionStart -lt 0 -or $transitionEnd -lt 0) {
    $failures.Add('Support-grip acquisition function boundary could not be located.')
} else {
    $transitionText = $weaponText.Substring($transitionStart, $transitionEnd - $transitionStart)
    if ($transitionText -match 'capturePartGrip[\s\S]*clearSupportGripPose') {
        $failures.Add('A committed support acquisition must not erase its newly captured scalar or exact finger transforms before first publication.')
    }
}

$applyWeaponStart = $weaponText.IndexOf('bool TwoHandedGrip::applyWeaponVisualAuthority')
$applyWeaponEnd = if ($applyWeaponStart -ge 0) { $weaponText.IndexOf('bool TwoHandedGrip::applyFiringHandLockedVisual', $applyWeaponStart) } else { -1 }
if ($applyWeaponStart -lt 0 -or $applyWeaponEnd -lt 0) {
    $failures.Add('Weapon visual authority function boundary could not be located.')
} else {
    $applyWeaponText = $weaponText.Substring($applyWeaponStart, $applyWeaponEnd - $applyWeaponStart)
    if ($applyWeaponText -match 'blendTransformOverDuration|resolveLockedHandVisualTarget|resolveDynamicSupportAcquisitionHandTarget|rockWeaponSupportGripHandLerp|applyRotationAroundPrimaryPivot') {
        $failures.Add('applyWeaponVisualAuthority must remain a pure publisher; acquisition interpolation belongs before the single authoritative write.')
    }
}

$visualOnlyStart = $weaponText.IndexOf('void TwoHandedGrip::updateVisualOnlySupportGrip')
$visualOnlyEnd = if ($visualOnlyStart -ge 0) { $weaponText.IndexOf('void TwoHandedGrip::setSupportGripPose', $visualOnlyStart) } else { -1 }
if ($visualOnlyStart -lt 0 -or $visualOnlyEnd -lt 0) {
    $failures.Add('Visual-only support function boundary could not be located.')
} else {
    $visualOnlyText = $weaponText.Substring($visualOnlyStart, $visualOnlyEnd - $visualOnlyStart)
    if ($visualOnlyText -match 'applyWeaponVisualAuthority|applyRotationAroundPrimaryPivot|beginDynamicSupportAcquisition') {
        $failures.Add('Visual-only support must never acquire weapon-transform authority or enter synchronized dynamic acquisition.')
    }
}

$physicsText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/core/PhysicsInteraction.cpp')
$gripUpdateIndex = $physicsText.IndexOf('_twoHandedGrip.update(')
$bodyUpdateIndex = $physicsText.IndexOf('_weaponCollision.updateBodiesFromCurrentSourceTransforms(', $gripUpdateIndex)
$muzzleUpdateIndex = $physicsText.IndexOf('applyFinalWeaponMuzzleAuthority()', $bodyUpdateIndex)
if ($gripUpdateIndex -lt 0 -or $bodyUpdateIndex -lt 0 -or $muzzleUpdateIndex -lt 0 -or
    $gripUpdateIndex -ge $bodyUpdateIndex -or $bodyUpdateIndex -ge $muzzleUpdateIndex) {
    $failures.Add('The authoritative blended weapon write must remain before generated-body and final muzzle publication.')
}

Reject-Text 'src/RockConfig.h' 'rockGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Removed generic grab startup lerp config fields must not remain in RockConfig.'
Reject-Text 'src/RockConfig.cpp' 'fGrabLerp(Speed|AngularSpeed|MaxTime)|rockGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Removed generic grab startup lerp config loading must not remain.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLerp(Speed|AngularSpeed)' `
    'Normal grab visual hand smoothing must not use old speed-based generic grab lerp settings.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Weapon support visual lerp source boundaries passed.'

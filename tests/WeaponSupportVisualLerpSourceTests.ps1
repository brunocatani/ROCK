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
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'shouldUseDynamicSupportAcquisition[\s\S]{0,500}FullTwoHandedSolver[\s\S]{0,300}!authoredSupportGrip[\s\S]{0,200}!providerAuthorityActive[\s\S]{0,200}!attachOnly' `
    'Synchronized acquisition must be eligible only for normal non-authored full-authority support grips.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'shortestArcSlerpFromIdentity[\s\S]*applyRotationAroundPrimaryPivot[\s\S]*localPointToWorld\([\s\S]*primaryGripLocal[\s\S]*primaryTargetWorld' `
    'Dynamic acquisition math must shortest-arc slerp the composite correction and re-solve translation from the live primary pivot.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'tryCaptureSupportInputBaseline\([\s\S]*invertTransform\(supportInputWorld\)[\s\S]*supportGripTargetWorld[\s\S]*tryResolveSupportInputTarget\(' `
    'Dynamic and gunstock support must share one rigid post-capture input calibration primitive.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' 'tryCaptureDynamicSupportDriverBaseline\([\s\S]*primaryDriverWorld[\s\S]*primaryGripTargetWorld[\s\S]*supportDriverWorld[\s\S]*supportGripTargetWorld[\s\S]*tryResolveDynamicSupportDriverTargets\(' `
    'Normal dynamic support must capture both physical driver relations as one transaction.'
Require-Text 'data/config/ROCK.dev.ini' 'Normal dynamic full-authority grabs[\s\S]*authored,[\s\S]*provider-owned,[\s\S]*AttachOnly,[\s\S]*visual-only' `
    'Repository config must describe the synchronized dynamic-acquisition semantics and unchanged paths.'


Reject-Text 'src/RockConfig.h' 'rockGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Removed generic grab startup lerp config fields must not remain in RockConfig.'
Reject-Text 'src/RockConfig.cpp' 'fGrabLerp(Speed|AngularSpeed|MaxTime)|rockGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Removed generic grab startup lerp config loading must not remain.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Weapon support visual lerp source boundaries passed.'

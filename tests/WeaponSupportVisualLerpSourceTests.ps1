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
Require-Text 'data/config/ROCK.ini' 'bWeaponSupportGripHandLerpEnabled[\s\S]*fWeaponSupportGripHandLerpTimeMin[\s\S]*fWeaponSupportGripHandLerpTimeMax[\s\S]*fWeaponSupportGripHandLerpMinDistance[\s\S]*fWeaponSupportGripHandLerpMaxDistance' `
    'Packaged ROCK.ini must document weapon support grip hand visual lerp settings.'

Require-Text 'src/physics-interaction/hand/HandVisual.h' 'computeDistanceMappedDurationGameUnits[\s\S]*blendTransformOverDuration' `
    'Visual hand helper must expose time-based distance-mapped transform blending.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeDistanceMappedDurationGameUnits\([\s\S]*rockGrabHandLerpTimeMin[\s\S]*rockGrabHandLerpTimeMax[\s\S]*rockGrabHandLerpMinDistance[\s\S]*rockGrabHandLerpMaxDistance' `
    'Normal grab visual hand smoothing must use the explicit grab hand lerp duration settings.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'resolveLockedHandVisualTarget[\s\S]*rockWeaponSupportGripHandLerpEnabled[\s\S]*blendTransformOverDuration' `
    'Two-handed weapon grab must smooth only the locked external hand visual target.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'applyLockedHandVisualAuthority\(weaponNode,\s*true,\s*true,\s*dt,\s*&primaryTransform,\s*&supportTransform\)' `
    'Full two-handed weapon authority must pass live hand transforms into visual-only hand lerp.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'updateVisualOnlySupportGrip\(RE::NiNode\* weaponNode,\s*float dt\)[\s\S]*applyLockedHandVisualAuthority\(weaponNode,\s*false,\s*true,\s*dt' `
    'Visual-only sidearm support grip must use the same support hand visual lerp path.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'handLerp=\(' `
    'Two-handed weapon telemetry must include sampled hand lerp alpha/duration.'

$weaponText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp')
$applyWeaponStart = $weaponText.IndexOf('bool TwoHandedGrip::applyWeaponVisualAuthority')
$applyWeaponEnd = if ($applyWeaponStart -ge 0) { $weaponText.IndexOf('bool TwoHandedGrip::applyLockedHandVisualAuthority', $applyWeaponStart) } else { -1 }
if ($applyWeaponStart -lt 0 -or $applyWeaponEnd -lt 0) {
    $failures.Add('Weapon visual authority function boundary could not be located.')
} else {
    $applyWeaponText = $weaponText.Substring($applyWeaponStart, $applyWeaponEnd - $applyWeaponStart)
    if ($applyWeaponText -match 'blendTransformOverDuration|resolveLockedHandVisualTarget|rockWeaponSupportGripHandLerp') {
        $failures.Add('Weapon visual authority must remain immediate; only external hand visual targets may be smoothed.')
    }
}

Reject-Text 'src/RockConfig.h' 'rockGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Removed generic grab startup lerp config fields must not remain in RockConfig.'
Reject-Text 'src/RockConfig.cpp' 'fGrabLerp(Speed|AngularSpeed|MaxTime)|rockGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Removed generic grab startup lerp config loading must not remain.'
Reject-Text 'data/config/ROCK.ini' 'fGrabLerp(Speed|AngularSpeed|MaxTime)' `
    'Packaged ROCK.ini must not keep obsolete generic grab startup lerp keys.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLerp(Speed|AngularSpeed)' `
    'Normal grab visual hand smoothing must not use old speed-based generic grab lerp settings.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Weapon support visual lerp source boundaries passed.'

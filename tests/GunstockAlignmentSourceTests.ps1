param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source([string]$RelativePath) {
    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $failures.Add("Missing source file: $RelativePath")
        return ''
    }
    return Get-Content -LiteralPath $path -Raw
}

function Require-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text(
    [string]$RelativePath,
    [string]$Pattern,
    [string]$Message
) {
    $text = Read-Source $RelativePath
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.h' `
    'rockGunstockAlignBarrelToControllerForward\s*=\s*false' `
    'Gunstock correction must remain an explicit opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'GUNSTOCK_SECTION\s*=\s*"Gunstock"[\s\S]*rockGunstockAlignBarrelToControllerForward\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*GUNSTOCK_SECTION,[\s\S]*"bAlignBarrelToControllerForward"' `
    'Gunstock configuration must have a false default and load from its dedicated INI section.'

foreach ($iniPath in @(
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*bAlignBarrelToControllerForward\s*=\s*false' `
        'Both shipped INIs must expose the disabled-by-default gunstock option.'
}

Require-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'kRequiredStableSamples\s*=\s*6[\s\S]*kStableSampleCosine[\s\S]*tryCaptureControllerLocalBore[\s\S]*localForward\s*\{\s*0\.0f,\s*1\.0f,\s*0\.0f\s*\}[\s\S]*tryBuildWorldCorrection[\s\S]*rotateRigidlyAroundPivot[\s\S]*precompensateWorldTarget' `
    'The value-only policy must latch six stable projectile +Y samples, build one rigid correction, and retain recoil precompensation.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_twoHandedGrip\.update\([\s\S]*_twoHandedGrip\.applyGunstockAlignment\([\s\S]*getEquippedProjectileNode\(\)[\s\S]*reconcileEquippedWeaponHandAssignmentAfterGrip\(\)[\s\S]*_weaponCollision\.update' `
    'Gunstock alignment must run after the existing grip solve, use the verified projectile node, and precede downstream weapon collision consumption.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'isRawButtonPhysicallyHeld\([\s\S]*kOpenVrSteamVrTriggerButtonId[\s\S]*applyGunstockAlignment\([\s\S]*gunstockNeutralSampleBlocked[\s\S]*frame\.reloadBoundaryActive\s*\|\|\s*frame\.menuBlocked' `
    'Neutral calibration must be blocked by physical trigger state while correction authority yields across animation/menu boundaries.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyGunstockAlignment\([\s\S]*directionLatch\.latched[\s\S]*tryCaptureControllerLocalBore[\s\S]*tryBuildWorldCorrection[\s\S]*getHandWorldTransform[\s\S]*rotateRigidlyAroundPivot[\s\S]*applyWeaponVisualAuthority' `
    'The runtime must latch first, read final hands, rotate the complete group, and publish the weapon last.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'leftManualFiringRecoil[\s\S]*deriveAppliedWorldDelta[\s\S]*precompensateWorldTarget[\s\S]*PRIMARY_GRIP_TAG[\s\S]*SUPPORT_GRIP_TAG' `
    'Left manual firing must derive and precompensate hFRIK recoil while existing grip-role tags remain reusable.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'GUNSTOCK_ALIGNMENT_TAG\s*=\s*"ROCK_GunstockAlignment"[\s\S]*clearGunstockDedicatedHandAuthority[\s\S]*clearExternalHandWorldTransform' `
    'Native/tracked hand correction must use and explicitly clear a dedicated authority tag.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock neutral direction latches on sixth stable sample[\s\S]*gunstock correction sends neutral bore to controller plus-Y[\s\S]*gunstock rigid correction preserves firing-hand weapon relation[\s\S]*gunstock precompensation survives noncommuting recoil delta[\s\S]*gunstock fixed neutral correction does not erase live recoil' `
    'Pure regression tests must lock calibration, rigid grouping, recoil precompensation, and live-recoil preservation.'

Reject-Text 'src/RockConfig.h' `
    'rockGunstockDebug|rockDebugDrawGunstock' `
    'The deferred gunstock orientation visualizer is not part of this implementation.'

Reject-Text 'src/api/ROCKProviderApi.h' `
    'Gunstock' `
    'Gunstock mode is internal presentation policy and must not alter ROCK API V1.'

if ($failures.Count -gt 0) {
    Write-Host 'GunstockAlignmentSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GunstockAlignmentSourceTests passed.' -ForegroundColor Green

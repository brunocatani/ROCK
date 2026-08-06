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

Require-Text 'src/RockConfig.h' `
    'rockDebugDrawGunstockAlignment\s*=\s*false' `
    'The gunstock orientation visualizer must remain independently opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockDebugDrawGunstockAlignment\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*"bDebugDrawGunstockAlignment"' `
    'The visualizer must load from the existing PhysicsInteraction section with a false default.'

foreach ($iniPath in @(
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*bAlignBarrelToControllerForward\s*=\s*false' `
        'Both shipped INIs must expose the disabled-by-default gunstock option.'
    Require-Text $iniPath `
        '\[PhysicsInteraction\][\s\S]*bDebugDrawGunstockAlignment\s*=\s*false' `
        'Both shipped INIs must expose the independent disabled-by-default orientation visualizer.'
}

Require-Text 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h' `
    'kRequiredStableSamples\s*=\s*6[\s\S]*kStableSampleCosine[\s\S]*tryCaptureControllerLocalBore[\s\S]*localForward\s*\{\s*0\.0f,\s*1\.0f,\s*0\.0f\s*\}[\s\S]*tryBuildWorldCorrection[\s\S]*targetForwardWorld[\s\S]*rotateRigidlyAroundPivot[\s\S]*precompensateWorldTarget' `
    'The value-only policy must latch six stable projectile +Y samples, accept an explicit world target, build one rigid correction, and retain recoil precompensation.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_twoHandedGrip\.update\([\s\S]*getEquippedProjectileNode\(\)[\s\S]*_twoHandedGrip\.applyGunstockAlignment\([\s\S]*reconcileEquippedWeaponHandAssignmentAfterGrip\(\)[\s\S]*_weaponCollision\.update' `
    'Gunstock alignment must run after the existing grip solve, use the verified projectile node, and precede downstream weapon collision consumption.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'prepareGunstockAlignmentDebugSnapshot\([\s\S]*applyGunstockAlignment\([\s\S]*updateBodiesFromCurrentSourceTransforms[\s\S]*applyFinalWeaponMuzzleAuthority\(\)[\s\S]*finalizeGunstockAlignmentDebugSnapshot\(' `
    'The visualizer must capture before alignment and publish final readback only after collision and muzzle authority.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'isRawButtonPhysicallyHeld\([\s\S]*kOpenVrSteamVrTriggerButtonId[\s\S]*applyGunstockAlignment\([\s\S]*gunstockNeutralSampleBlocked[\s\S]*frame\.reloadBoundaryActive\s*\|\|\s*frame\.menuBlocked' `
    'Neutral calibration must be blocked by physical trigger state while correction authority yields across animation/menu boundaries.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyGunstockAlignment\([\s\S]*getHandWorldTransform[\s\S]*localWristForward\s*\{\s*1\.0f,\s*0\.0f,\s*0\.0f\s*\}[\s\S]*directionLatch\.latched[\s\S]*tryCaptureControllerLocalBore[\s\S]*tryBuildWorldCorrection[\s\S]*wristForwardWorld[\s\S]*rotateRigidlyAroundPivot[\s\S]*applyWeaponVisualAuthority' `
    'The runtime must read firing-wrist +X, latch the neutral barrel, rotate the complete group toward that target, and publish the weapon last.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'leftManualFiringRecoil[\s\S]*deriveAppliedWorldDelta[\s\S]*precompensateWorldTarget[\s\S]*PRIMARY_GRIP_TAG[\s\S]*SUPPORT_GRIP_TAG' `
    'Left manual firing must derive and precompensate hFRIK recoil while existing grip-role tags remain reusable.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'GUNSTOCK_ALIGNMENT_TAG\s*=\s*"ROCK_GunstockAlignment"[\s\S]*clearGunstockDedicatedHandAuthority[\s\S]*clearExternalHandWorldTransform' `
    'Native/tracked hand correction must use and explicitly clear a dedicated authority tag.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockAlignmentDebugSnapshot[\s\S]*std::uintptr_t\s+weaponNodeIdentity[\s\S]*std::uintptr_t\s+fireNodeIdentity[\s\S]*controllerWorld[\s\S]*weaponWorldBefore[\s\S]*finalFireNodeWorld[\s\S]*published' `
    'The renderer bridge must be a value-only coherent snapshot with non-dereferenced identity witnesses.'

$gunstockSnapshot = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.h'),
    '(?ms)struct\s+GunstockAlignmentDebugSnapshot\s*\{(?<body>.*?)^\s{4}\};')
if (-not $gunstockSnapshot.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.h: Could not isolate the gunstock debug snapshot.')
} elseif ($gunstockSnapshot.Groups['body'].Value -match 'RE::Ni(?:Node|AVObject)\s*\*') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.h: The gunstock debug snapshot must not retain engine pointers.')
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'populateGunstockAlignmentDebugPrediction[\s\S]*prepareGunstockAlignmentDebugSnapshot[\s\S]*AlignmentDisabled[\s\S]*finalizeGunstockAlignmentDebugSnapshot[\s\S]*finalLiveFireWorld' `
    'Behavior-off diagnostics must compute a read-only correction preview and final live readback.'

Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'GunstockFiringController[\s\S]*GunstockFireNodeFinal[\s\S]*GunstockWristForward[\s\S]*GunstockCorrectionArc[\s\S]*GunstockCorrectionAxis' `
    'The existing bounded overlay must own explicit gunstock axis, ray, pivot, and correction-arc roles.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'drawGunstockAlignment[\s\S]*getGunstockAlignmentDebugSnapshot[\s\S]*kCorrectionArcSegments\s*=\s*16[\s\S]*FIRING WRIST \+X - GUNSTOCK FORWARD[\s\S]*ACTUAL FINAL LIVE FIRE \+Y[\s\S]*neutralResidual[\s\S]*liveDeviation[\s\S]*dataAge=0' `
    'The visualizer must render bounded tripods, comparison rays, a fixed correction arc, and current-frame status.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'CONTROLLER \+Y - GUNSTOCK FORWARD|controllerAxis=\+Y' `
    'Gunstock diagnostics must not present controller +Y as the alignment target.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock neutral direction latches on sixth stable sample[\s\S]*gunstock correction sends neutral bore to firing wrist plus-X[\s\S]*gunstock rigid correction preserves firing-hand weapon relation[\s\S]*gunstock precompensation survives noncommuting recoil delta[\s\S]*gunstock fixed neutral correction does not erase live recoil' `
    'Pure regression tests must lock calibration, rigid grouping, recoil precompensation, and live-recoil preservation.'

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

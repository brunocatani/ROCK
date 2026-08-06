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
    'rockGunstockModeEnabled\s*=\s*false' `
    'Gunstock baseline mode must remain an explicit opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockGunstockModeEnabled\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*GUNSTOCK_SECTION,[\s\S]*"bAlignBarrelToControllerForward",[\s\S]*rockGunstockModeEnabled' `
    'The historical production key must load the renamed baseline mode without changing the external INI contract.'

Require-Text 'src/RockConfig.h' `
    'rockDebugDrawGunstockSupportBaseline\s*=\s*false' `
    'Gunstock baseline diagnostics must remain independently opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockDebugDrawGunstockSupportBaseline\s*=\s*false[\s\S]*"bDebugDrawGunstockAlignment",\s*rockDebugDrawGunstockSupportBaseline' `
    'The historical diagnostic key must load the baseline visualizer.'

foreach ($iniPath in @(
    'data/config/ROCK.ini',
    'data/mod/ROCK_Config/ROCK.ini'
)) {
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*no longer forces a barrel-axis correction[\s\S]*bAlignBarrelToControllerForward\s*=\s*false' `
        'Both shipped INIs must describe and expose the disabled-by-default attach-baseline mode.'
    Require-Text $iniPath `
        '\[PhysicsInteraction\][\s\S]*calibrated grip target[\s\S]*bDebugDrawGunstockAlignment\s*=\s*false' `
        'Both shipped INIs must expose the disabled-by-default baseline visualizer.'
}

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'tryCaptureGunstockSupportBaseline\([\s\S]*invertTransform\(supportInputWorld\)[\s\S]*supportGripTargetWorld[\s\S]*tryResolveGunstockSupportTarget\([\s\S]*supportInputWorld,[\s\S]*inputToGripTargetLocal' `
    'The value-only policy must freeze input-to-grip relation and resolve later support motion through it.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'isUsableTransform\(supportInputWorld\)[\s\S]*isUsableTransform\(supportGripTargetWorld\)[\s\S]*isUsableTransform\(relation\)[\s\S]*isUsableTransform\(inputToGripTargetLocal\)[\s\S]*isUsableTransform\(target\)' `
    'Baseline capture and resolution must fail closed on unusable transforms.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryGetSolverHandTransform\(isLeft, handTransform\)[\s\S]*if \(outCapturedHandWorld\)[\s\S]*\*outCapturedHandWorld\s*=\s*handTransform' `
    'Grip capture must return the exact damped support sample used by acquisition.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'rockGunstockModeEnabled[\s\S]*FullTwoHandedSolver[\s\S]*resolvePartGripHandWorld\(supportGrip, weaponNode\)[\s\S]*tryCaptureGunstockSupportBaseline\([\s\S]*supportCaptureHandWorld[\s\S]*supportGripTargetWorld[\s\S]*weaponWorldAtCapture\s*=[\s\S]*weaponNode->world[\s\S]*firstPublicationPending\s*=\s*true' `
    'Every full-authority support grip must capture the visual grip target against the same damped input and save the primary weapon transform.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'const bool useDynamicSupportAcquisition\s*=[\s\S]*!gunstockBaselineActive[\s\S]*shouldUseDynamicSupportAcquisition[\s\S]*_rotationBlend\s*=\s*gunstockBaselineActive\s*\?\s*1\.0f\s*:\s*0\.0f[\s\S]*if \(gunstockBaselineActive\)[\s\S]*updateFullWeaponAuthorityGrip\(weaponNode, 0\.0f\)' `
    'Gunstock attach must bypass the old absolute acquisition blend and publish one immediate attach transaction.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'calibratedSupportTransform\s*=\s*supportTransform[\s\S]*tryResolveGunstockSupportTarget\([\s\S]*supportTransform[\s\S]*inputToGripTargetLocal[\s\S]*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*calibratedSupportTransform[\s\S]*supportNormalTargetWorld\s*=\s*computePalmNormalFromHandBasis\([\s\S]*calibratedSupportTransform' `
    'The existing tandem solver must receive both position and twist from the calibrated support-hand target.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'gunstockAttachPublication[\s\S]*appliedWeaponWorld\s*=[\s\S]*weaponWorldAtCapture[\s\S]*applyWeaponVisualAuthority\(weaponNode, appliedWeaponWorld\)[\s\S]*applyLockedHandVisualAuthority\([\s\S]*&primaryTransform,[\s\S]*&supportTransform[\s\S]*firstPublicationPending\s*=\s*false' `
    'The first publication must retain the captured weapon exactly while the ordinary visual hand seat uses the live support input.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'void TwoHandedGrip::reset\(\)[\s\S]*_authoredSupportGripDebugSnapshot\s*=\s*\{\};[\s\S]*_gunstockSupportBaselineDebugSnapshot\s*=\s*\{\};' `
    'Runtime reset must invalidate any previously published gunstock baseline diagnostic frame.'

Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'const RE::NiTransform liveWeaponWorld\s*=\s*input\.weaponNode->world;[\s\S]*frik_visual_authority::getHandWorldTransform' `
    'Primary firing-grip alignment must retain its normal live weapon and hFRIK hand authority path.'

Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'tryGetGunstockTrackedFiringHandWorld|gunstockTrackedHandWorld' `
    'Gunstock mode must not substitute a special firing-hand frame.'

$deletedPolicy = Join-Path $Root 'src/physics-interaction/weapon/GunstockAlignmentPolicy.h'
if (Test-Path -LiteralPath $deletedPolicy) {
    $failures.Add('src/physics-interaction/weapon/GunstockAlignmentPolicy.h: The superseded final barrel-correction policy must remain deleted.')
}

foreach ($sourcePath in @(
    'src/physics-interaction/core/PhysicsInteraction.cpp',
    'src/physics-interaction/weapon/TwoHandedGrip.h',
    'src/physics-interaction/weapon/TwoHandedGrip.cpp'
)) {
    Reject-Text $sourcePath `
        'applyGunstockAlignment|prepareGunstockAlignmentDebugSnapshot|finalizeGunstockAlignmentDebugSnapshot|tryResolveGunstockPhysicalFiringFrame|tryResolveGunstockPrimaryGroupCorrection|GUNSTOCK_ALIGNMENT_TAG|gunstockNeutralSampleBlocked|DirectionLatch' `
        'No post-solve barrel correction, firing-frame override, calibration latch, or dedicated hand authority may remain.'
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' `
    'struct\s+GunstockSupportBaselineDebugSnapshot[\s\S]*supportInputWorld[\s\S]*calibratedSupportWorld[\s\S]*supportGripHandWorld[\s\S]*weaponWorldBefore[\s\S]*weaponWorldAfter[\s\S]*attachRotationDegrees[\s\S]*published' `
    'The visualizer bridge must expose coherent baseline and attach-invariant values.'

$gunstockSnapshot = [regex]::Match(
    (Read-Source 'src/physics-interaction/weapon/TwoHandedGrip.h'),
    '(?ms)struct\s+GunstockSupportBaselineDebugSnapshot\s*\{(?<body>.*?)^\s{4}\};')
if (-not $gunstockSnapshot.Success) {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.h: Could not isolate the gunstock baseline debug snapshot.')
} elseif ($gunstockSnapshot.Groups['body'].Value -match 'RE::Ni(?:Node|AVObject)\s*\*') {
    $failures.Add('src/physics-interaction/weapon/TwoHandedGrip.h: The gunstock baseline debug snapshot must not retain engine pointers.')
}

Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'GunstockSupportInputBone[\s\S]*GunstockCalibratedSupportBone[\s\S]*GunstockGripTargetHand[\s\S]*GunstockWeaponBefore[\s\S]*GunstockWeaponAfter[\s\S]*GunstockPrimaryPivot[\s\S]*GunstockSupportTargetPoint' `
    'The bounded overlay must own baseline input, calibrated target, grip target, and before/after weapon roles.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'drawGunstockSupportBaseline[\s\S]*getGunstockSupportBaselineDebugSnapshot[\s\S]*GunstockSupportInputBone[\s\S]*GunstockCalibratedSupportBone[\s\S]*GunstockGripTargetHand[\s\S]*GunstockWeaponBefore[\s\S]*GunstockWeaponAfter[\s\S]*attachWeaponDelta' `
    'The visualizer must render the new attach-baseline transaction and its measured delta.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'CONTROLLER \+Y - GUNSTOCK FORWARD|FIRING WRIST \+X - GUNSTOCK FORWARD|ACTUAL FINAL LIVE FIRE \+Y|GunstockCorrectionArc' `
    'Baseline diagnostics must not claim or render a forced controller/barrel axis.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'unchanged support input reproduces exact authored target[\s\S]*carries only the post-attach rigid support delta[\s\S]*rejects non-finite captured support baseline[\s\S]*calibrated attach target leaves weapon unchanged[\s\S]*post-attach support delta drives existing tandem solver' `
    'Pure regressions must lock attach identity, delta transport, tandem response, and fail-closed behavior.'

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

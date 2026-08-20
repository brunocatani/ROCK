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
    'Gunstock mode must remain an explicit opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockGunstockModeEnabled\s*=\s*false[\s\S]*GetBoolValue\([\s\S]*GUNSTOCK_SECTION,[\s\S]*"bAlignBarrelToControllerForward",[\s\S]*rockGunstockModeEnabled' `
    'The historical production key must load both gunstock stages without changing the external INI contract.'

Require-Text 'src/RockConfig.h' `
    'rockGunstockAlignmentPitchDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentYawDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentRollDegrees\s*=\s*0\.0f' `
    'Gunstock fine tuning must remain zero-default so existing alignment is unchanged.'

Require-Text 'src/RockConfig.cpp' `
    'rockGunstockAlignmentPitchDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentYawDegrees\s*=\s*0\.0f[\s\S]*rockGunstockAlignmentRollDegrees\s*=\s*0\.0f[\s\S]*"fAlignmentPitchDegrees"[\s\S]*-180\.0f,[\s\S]*180\.0f[\s\S]*"fAlignmentYawDegrees"[\s\S]*-180\.0f,[\s\S]*180\.0f[\s\S]*"fAlignmentRollDegrees"[\s\S]*-180\.0f,[\s\S]*180\.0f' `
    'All three gunstock fine-tune axes must load as finite clamped degree values.'

Require-Text 'src/RockConfig.h' `
    'rockDebugDrawGunstockAlignment\s*=\s*false' `
    'Gunstock diagnostics must remain independently opt-in.'

Require-Text 'src/RockConfig.cpp' `
    'rockDebugDrawGunstockAlignment\s*=\s*false[\s\S]*"bDrawGunstockAlignment",\s*rockDebugDrawGunstockAlignment' `
    'The normalized diagnostic key must load the combined alignment visualizer.'

foreach ($iniPath in @(
    'data/config/ROCK.ini'
)) {
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*two independent stages[\s\S]*neutral fire-node[\s\S]*firing-wrist \+X[\s\S]*bAlignBarrelToControllerForward\s*=\s*false' `
        'Both shipped INIs must describe and expose the disabled-by-default support baseline and final wrist alignment.'
    Require-Text $iniPath `
        '\[Gunstock\][\s\S]*same damped-driver pivot[\s\S]*yaw about bone \+Z[\s\S]*pitch about bone \+Y[\s\S]*roll about aligned \+X[\s\S]*fAlignmentPitchDegrees\s*=\s*0\.0[\s\S]*fAlignmentYawDegrees\s*=\s*0\.0[\s\S]*fAlignmentRollDegrees\s*=\s*0\.0' `
        'Both shipped INIs must document and expose zero-default wrist-space fine tuning.'
    Require-Text $iniPath `
        '\[DebugOverlay\][\s\S]*firing/support bone triads[\s\S]*support attach-baseline[\s\S]*bDrawGunstockAlignment\s*=\s*false' `
        'The canonical INI must describe and expose the combined disabled-by-default visualizer.'
}

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'tryCaptureSupportInputBaseline\([\s\S]*invertTransform\(supportInputWorld\)[\s\S]*supportGripTargetWorld[\s\S]*tryResolveSupportInputTarget\([\s\S]*supportInputWorld,[\s\S]*inputToGripTargetLocal' `
    'The support policy must freeze input-to-grip relation and resolve only later support motion through it.'



Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' `
    'alignOpticalAxesToBore|camera \+X[\s\S]*bore \+Y' `
    'Gunstock scopes must not replace the captured native camera calibration with an unverified axis mapping.'

Require-Text 'src/ROCKMain.cpp' `
    'dispatchAnimationPhaseCallbacksV1\([\s\S]*AfterRock[\s\S]*finalizeGunstockPresentationAfterNativeAnimation\(\)[\s\S]*dispatchAnimationPhaseCallbacksV1\([\s\S]*Complete' `
    'The full-reload correction must run after animation AfterRock publication and before the Complete phase.'


Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'drawGunstockAlignment[\s\S]*getGunstockAlignmentDebugSnapshot[\s\S]*GunstockLeftHand[\s\S]*GunstockFiringHand[\s\S]*GunstockRenderedFiringHand[\s\S]*RENDERED FIRING HAND BONE[\s\S]*FIRING WRIST \+X - AUTOMATIC TARGET[\s\S]*PREDICTED FINE-TUNED NEUTRAL[\s\S]*ACTUAL FINAL LIVE FIRE \+Y[\s\S]*GunstockCorrectionArc[\s\S]*fineTune pitch\(\+Y\)[\s\S]*liveResidual=[\s\S]*renderedRelation=[\s\S]*getGunstockSupportBaselineDebugSnapshot[\s\S]*GunstockSupportInputBone[\s\S]*GunstockCalibratedSupportBone[\s\S]*GunstockWeaponAfter[\s\S]*attachWeaponDelta' `
    'The visualizer must render input/rendered firing bones, configured fine-tuned intent, actual relation error, and support attach baseline together.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock zero fine tune preserves automatic correction exactly[\s\S]*gunstock positive yaw sends plus-X toward plus-Y[\s\S]*gunstock positive pitch sends plus-X toward minus-Z[\s\S]*gunstock pure roll fine tune remains active with aligned bore[\s\S]*gunstock combined correction reaches fine-tuned target[\s\S]*gunstock fine tune keeps the damped-driver reference pivot fixed[\s\S]*gunstock fine tune preserves the firing-grip weapon relation[\s\S]*gunstock fine-tuned weapon preserves retained rigid scope frame' `
    'Pure regressions must cover zero compatibility, axis signs, pure roll, target composition, fixed pivot, firing-grip rigidity, and rigid fine-tuned scope transport.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'GunstockFiringController|GunstockLeftController|CONTROLLER \+Y - GUNSTOCK FORWARD|controllerAxis=\+Y' `
    'Gunstock diagnostics must use bone triads and wrist +X, not controller-axis tripods.'

Require-Text 'tests/WeaponInteractionPolicyTests.cpp' `
    'gunstock live enable produces one edge[\s\S]*gunstock fire node alone does not classify thrown weapons as firearms[\s\S]*gunstock gun type plus valid fire node establishes generation eligibility[\s\S]*gunstock eligibility survives transient fire-node loss[\s\S]*gunstock eligibility never crosses weapon generations[\s\S]*gunstock neutral direction latches on sixth stable sample[\s\S]*gunstock correction sends neutral bore to firing wrist plus-X[\s\S]*gunstock rigid correction preserves firing-hand weapon relation[\s\S]*gunstock precompensation survives noncommuting recoil delta[\s\S]*gunstock post-animation correction preserves authored left-hand relation[\s\S]*gunstock post-animation correction preserves authored right-hand relation[\s\S]*unchanged support input reproduces exact authored target[\s\S]*calibrated attach target leaves weapon unchanged[\s\S]*post-attach support delta drives existing tandem solver' `
    'Pure regressions must cover mode edges, eligibility lifetime, final alignment, recoil continuity, full-reload rigid hand relations, support attach identity, and tandem delta.'

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

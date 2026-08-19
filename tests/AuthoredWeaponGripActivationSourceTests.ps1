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
        $failures.Add("$Path`: $Message")
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add("$Path`: $Message")
    }
}

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'kRightHandEquipSlotFormID\s*=\s*0x00013F42u[\s\S]{0,160}kBothHandsEquipSlotFormID\s*=\s*0x00013F45u' `
    'The pure classifier must retain the locally verified RightHand and BothHands behavior-slot identities.'

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'WeaponFamily::OneHandGun[\s\S]*selectedCone\s*=\s*AllowedCone::Left[\s\S]*WeaponFamily::TwoHandGun[\s\S]*leftPass\s*\|\|\s*downPass' `
    'One-hand weapons must expose LEFT only while two-hand weapons expose the LEFT/DOWN union.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState\([\s\S]*evaluateDirectionGate\([\s\S]*findCurrentWeaponSurfaceNearPoints\(' `
    'The activation state must consume the shared policy and bounded captured-pose surface witnesses.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'snapshot\.radialCapGameUnits\s*=\s*g_rockConfig\.rockWeaponAuthoredGripActivationRadius' `
    'Authored-seat reach must use its dedicated setting instead of the unrelated mesh-probe radius.'

Require-Text 'src/RockConfig.cpp' `
    'readClampedFloat\(ini,[\s\S]{0,160}"fWeaponAuthoredGripActivationRadius"[\s\S]{0,160}16\.0f,[\s\S]{0,80}2\.0f,[\s\S]{0,80}32\.0f\)' `
    'The dedicated authored-seat radius must be loaded with its canonical default and bounded runtime range.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'snapshot\.radialCapGameUnits\s*=\s*g_rockConfig\.rockWeaponInteractionProbeRadius' `
    'The general weapon mesh-probe radius must not silently redefine authored-seat activation reach.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState\([\s\S]*true\);[\s\S]*\.activationZoneValid\s*=\s*authoredActivationZoneValid[\s\S]*\.authoredPoseSurfaceEvidenceValid\s*=[\s\S]*authoredPoseSurfaceEvidenceValid' `
    'Authored capture must force a current activation-zone and captured-pose evaluation.'

Reject-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'semanticTargetEligible|semanticPass' `
    'Weapon-part semantic metadata must not redefine geometric authored-cone membership.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'semanticTargetEligible' `
    'Authored activation must not veto a passing cone because generated contact geometry also owns reload/action/socket metadata.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'rockDebugDrawAuthoredGripActivationZones[\s\S]*resolveConeBoundaryDimensions[\s\S]*drawWireCone[\s\S]*ENFORCED AUTHORED ACTIVATION' `
    'The pre-grab overlay must be independently enabled and explicitly identify its enforced verdict.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'snapshot\.radialCapGameUnits,\s*12\.0f|axis\.x\s*\*\s*axisLength\s*\+[\s\S]{0,180}tangentA\.x\s*\*\s*radialA' `
    'The debug wire cone must not draw the old sqrt(2)-oversized boundary or clamp the configured reach back to 12 units.'

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'fWeaponAuthoredGripActivationRadius\s*=\s*16\.0' `
        'The authored-grip activation radius must have the same bounded default in both canonical configuration copies.'
    Require-Text $configPath `
        'bDebugDrawAuthoredGripActivationZones\s*=\s*false' `
        'The authored-grip activation visualizer must remain default-off.'
}

if ($failures.Count -gt 0) {
    Write-Host 'Authored weapon-grip activation source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Authored weapon-grip activation source boundary passed.'

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

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'baseEquipSlot\s*=\s*weapon->GetEquipSlot\(nullptr\)[\s\S]{0,260}effectiveEquipSlot\s*=\s*weapon->GetEquipSlot\(instanceData\)' `
    'Authored-grip family diagnostics must read both base and effective BGSEquipType behavior slots.'

Require-Text 'src/physics-interaction/weapon/WeaponClassificationPolicy.h' `
    'kRightHandEquipSlotFormID\s*=\s*0x00013F42u[\s\S]{0,200}kBothHandsLeftOptionalEquipSlotFormID\s*=\s*[\s\r\n]*0x0004334Du[\s\S]{0,200}kBothHandsEquipSlotFormID\s*=\s*0x00013F45u' `
    'The shared classifier must retain the locally verified RightHand, BothHandsLeftOptional, and BothHands behavior-slot identities.'

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'WeaponFamily::OneHandGun[\s\S]*selectedRegion\s*=\s*ActivationRegion::Left[\s\S]*WeaponFamily::TwoHandGun[\s\S]*sweptArcDotSquared[\s\S]*ActivationRegion::Arc' `
    'One-hand weapons must expose LEFT only while two-hand weapons use the LEFT-to-DOWN swept activation region.'

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'kIndicatorOffsetGameUnits\s*=\s*5\.0f[\s\S]*evaluateIndicator[\s\S]*WeaponFamily::OneHandGun[\s\S]*leftAxisWorld[\s\S]*WeaponFamily::TwoHandGun[\s\S]*downAxisWorld' `
    'The gameplay indicator must retain the fixed five-unit LEFT/DOWN family anchors.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState\([\s\S]*evaluateDirectionGate\([\s\S]*findCurrentWeaponSurfaceNearPoints\(' `
    'The activation state must consume the shared policy and bounded captured-pose surface witnesses.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState\([\s\S]*true\);[\s\S]*\.activationZoneValid\s*=\s*authoredActivationZoneValid[\s\S]*\.authoredPoseSurfaceEvidenceValid\s*=[\s\S]*authoredPoseSurfaceEvidenceValid' `
    'Authored capture must force a current activation-zone and captured-pose evaluation.'

Reject-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'semanticTargetEligible|semanticPass' `
    'Weapon-part semantic metadata must not redefine geometric authored-cone membership.'

Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'semanticTargetEligible' `
    'Authored activation must not veto a passing cone because generated contact geometry also owns reload/action/socket metadata.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'finishUpdate[\s\S]{0,2200}getGripOccupancy\(\)[\s\S]{0,1200}supportHandWeaponEngaged[\s\S]{0,1200}evaluateIndicator[\s\S]{0,1200}_authoredSupportGripIndicatorFrame' `
    'Indicator visibility must be finalized from post-transition hand occupancy on the common update exit.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'authoredInteractionCandidateValid[\s\S]{0,1800}supportGripAllowed[\s\S]{0,800}providerPartAuthorityActive[\s\S]{0,800}supportHandHoldingObject' `
    'The gameplay cue must require a routed, unreserved, provider-free candidate and an available support hand.'

Require-Text 'src/physics-interaction/weapon/AuthoredSupportGripIndicatorEffect.cpp' `
    'Data/Meshes/ROCK/selection_beam_segment\.nif[\s\S]*loadNifFromFile[\s\S]*AttachChild[\s\S]*worldPointToLocal[\s\S]*setVisible\(true, true\)' `
    'The gameplay cue must preload and reuse the packaged green sphere as a world-root scenegraph effect.'

Require-Text 'src/physics-interaction/weapon/AuthoredSupportGripIndicatorEffect.cpp' `
    'void\s+AuthoredSupportGripIndicatorEffect::shutdown\(\)[\s\S]*clearMarker\(true\)[\s\S]*void\s+AuthoredSupportGripIndicatorEffect::abandonSceneGraph\(\)[\s\S]*clearMarker\(false\)' `
    'The gameplay cue must distinguish valid detach from stale-world abandonment.'

Reject-Text 'src/physics-interaction/weapon/AuthoredSupportGripIndicatorEffect.cpp' `
    'DebugBodyOverlay|PhysicsInteractionDebugOverlay|debug::' `
    'The gameplay indicator must not depend on debug-overlay admission or rendering.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_authoredSupportGripIndicator\.preload\(\)[\s\S]*updateAuthoredSupportGripIndicator\(\);[\s\S]*publishDebugBodyOverlay\(frame\)' `
    'PhysicsInteraction must preload and update the gameplay cue independently before debug publication.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(worldValid\)[\s\S]{0,400}_authoredSupportGripIndicator\.shutdown\(\)[\s\S]*else[\s\S]{0,400}_authoredSupportGripIndicator\.abandonSceneGraph\(\)' `
    'Physics shutdown must preserve the cue scene-node valid/stale world distinction.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'rockDebugDrawAuthoredGripActivationZones[\s\S]*drawWireCone[\s\S]*drawWireSweptActivationRegion[\s\S]*ENFORCED AUTHORED ACTIVATION' `
    'The pre-grab overlay must draw the one-hand cone and two-hand swept region while explicitly identifying its enforced verdict.'

foreach ($configPath in @('data/config/ROCK_example.ini')) {
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

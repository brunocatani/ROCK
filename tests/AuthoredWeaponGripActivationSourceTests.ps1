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

function Require-Path {
    param([string]$Path, [string]$Message)
    if (-not (Test-Path -LiteralPath (Join-Path $Root $Path))) {
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
    'HandTopology[\s\S]*RightFiringLeftSupport[\s\S]*LeftFiringRightSupport[\s\S]*resolveHandTopology[\s\S]*ActivationRegion::Left[\s\S]*ActivationRegion::Right' `
    'Authored activation must retain separate right-fire/left-support and left-fire/right-support topology identities.'

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'orientRightFiringAxisForTopology[\s\S]*RightFiringLeftSupport[\s\S]*rightFiringAxisWeaponLocal[\s\S]*LeftFiringRightSupport[\s\S]*-rightFiringAxisWeaponLocal\.x[\s\S]*rightFiringAxisWeaponLocal\.y[\s\S]*rightFiringAxisWeaponLocal\.z' `
    'Left firing must mirror the native authored activation axes through weapon-local X instead of reusing the left-facing cone.'

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'WeaponFamily::OneHandGun[\s\S]*selectedRegion\s*=\s*lateralRegion[\s\S]*WeaponFamily::TwoHandGun[\s\S]*sweptArcDotSquared[\s\S]*ActivationRegion::Arc' `
    'One-hand weapons must use the topology side while two-hand weapons sweep from that side to DOWN.'

Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h' `
    'kIndicatorOffsetGameUnits\s*=\s*3\.0f[\s\S]*evaluateIndicator[\s\S]*WeaponFamily::OneHandGun[\s\S]*indicatorAxis\s*=\s*input\.supportSideAxisWorld[\s\S]*WeaponFamily::TwoHandGun[\s\S]*tryNormalize\([\s\S]*input\.supportSideAxisWorld[\s\S]*tryNormalize\(input\.downAxisWorld[\s\S]*normalizedSupportSide\.x\s*\+\s*normalizedDown\.x' `
    'The gameplay indicator must use a three-unit topology-side anchor for one-hand weapons and the normalized side/DOWN diagonal for two-hand weapons.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'authoredIndicatorSupportHandIsLeft\s*=\s*supportHandIsLeft[\s\S]{0,1800}\.supportSideAxisWorld\s*=\s*toIndicatorVector\([\s\S]{0,120}authoredActivation\.supportSideAxisWorld' `
    'Indicator placement and ownership must follow the current mirrored support-hand topology for both firing hands.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState\([\s\S]*evaluateDirectionGate\([\s\S]*findCurrentWeaponSurfaceNearPoints\(' `
    'The activation state must consume the shared policy and retain optional collider visualization witnesses.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryResolveAuthoredSupportActivationAxes[\s\S]*worldVectorToLocal\([\s\S]*orientRightFiringAxisForTopology[\s\S]*localVectorToWorld\(' `
    'Runtime activation axes must mirror through the current Weapon frame before returning to world space.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'resolveHandTopology\(\s*_firingHandIsLeft,\s*supportHandIsLeft\)[\s\S]*_authoredSupportLastStableDirectionHandTopology\s*!=[\s\S]*handTopology[\s\S]*_authoredSupportLastStableApproachDirectionWorld\s*=\s*\{\}' `
    'A firing-hand topology change must clear the last approach direction so LEFT state cannot leak into RIGHT activation.'

foreach ($activationPath in @(
    'src/physics-interaction/weapon/AuthoredWeaponGripActivationPolicy.h',
    'src/physics-interaction/weapon/TwoHandedGrip.cpp',
    'src/physics-interaction/weapon/TwoHandedGrip.h')) {
    Reject-Text $activationPath `
        'rightFiringLeftSupportScope|\.leftAxisWorld|\.scopePass' `
        'Authored activation must not restore the retired one-sided LEFT-only scope contract.'
}

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshAuthoredSupportGripActivationState\([\s\S]*\.activationZoneValid\s*=\s*authoredActivationZoneValid[\s\S]*\.captureValid\s*=' `
    'Authored capture must require the current activation zone and captured-pose relation without forcing collision evidence.'

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
    'authoredSeatAcquisitionAvailable[\s\S]{0,900}activationSpatialPass[\s\S]{0,900}supportGripAllowed[\s\S]{0,900}providerPartAuthorityActive[\s\S]{0,1800}supportHandHoldingObject' `
    'The gameplay cue and authored acquisition must use the authored activation region directly, without a routed collider candidate.'

Require-Path 'data/mod/Meshes/ROCK/authored_support_grip_indicator_bright.nif' `
    'The high-visibility authored support-grip sphere must be packaged as a ROCK-owned asset.'

Require-Text 'src/physics-interaction/weapon/AuthoredSupportGripIndicatorEffect.cpp' `
    'Data/Meshes/ROCK/authored_support_grip_indicator_bright\.nif[\s\S]*loadNifObjectFromFile[\s\S]*AttachChild[\s\S]*worldPointToLocal[\s\S]*setVisible\(true, true\)' `
    'The gameplay cue must preload and reuse the dedicated bright sphere as a world-root scenegraph effect.'

Require-Text 'src/rock_support/Fo4VrRuntime.cpp' `
    'loadNifRootAddress[\s\S]*loadNifFromFile[\s\S]*loadNifObjectFromFile' `
    'The runtime NIF loader must preserve node callers while allowing the indicator to own a BSTriShape root safely.'

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
    'rockDebugDrawAuthoredGripActivationZones[\s\S]*topology=%s[\s\S]*supportSideAxisWorld[\s\S]*drawWireCone[\s\S]*drawWireSweptActivationRegion[\s\S]*AUTHORED ACTIVATION[\s\S]*collision diagnostic witnesses' `
    'The pre-grab overlay must draw the topology-specific cone and label collision witnesses as diagnostics.'

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

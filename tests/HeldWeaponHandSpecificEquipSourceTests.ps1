param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    if ((Read-Source $RelativePath) -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    if ((Read-Source $RelativePath) -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'triggerPressedEdge\s*&&\s*input\.triggerInputHand\s*==\s*input\.heldWeaponHand' `
    'Held-weapon trigger equip must require the trigger and held weapon to belong to the same physical hand.'

Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    '\(input\.gripZoneEquipEnabled\s*&&\s*input\.gripZoneEquipSettled\)' `
    'Firing-grip-zone equip must be available to either physical hand without a primary-hand gate.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'std::array<std::atomic<bool>,\s*2>\s+s_handHeldWeapon' `
    'Native trigger suppression must track held-weapon ownership for both physical hands.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'eventHandHeldWeapon\s*=\s*s_handHeldWeapon\[eventHandIsLeft\s*\?\s*0u\s*:\s*1u\]' `
    'Native trigger suppression must read held-weapon ownership for the physical hand that emitted the event.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'loose_weapon_grip_zone::tryGetFiringHandWeaponLocal\([\s\S]{0,180}pendingGripStart\.firingHandWeaponLocal' `
    'Loose equip must carry the canonical weapon-relative firing-hand frame across inventory transfer.'

Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'gripWeaponLocal\s*=\s*transform_math::worldPointToLocal\(attachedRootWorld,\s*canonicalPalmWorld\)[\s\S]{0,180}localPointToWorld\(looseRoot->world,\s*state\.gripWeaponLocal\)' `
    'The firing-grip point must be derived from the canonical attach pose and then projected through the loose weapon, never derived from the tested hand.'

Reject-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'worldPointToLocal\(attachedRootWorld,\s*palmWorld\)' `
    'The tested palm must not define the firing-grip point.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    'struct\s+HeldWeaponTriggerEquipIntent[\s\S]{0,240}formID[\s\S]{0,160}remainingSeconds' `
    'The retained trigger intent must be identity-bound and time-bounded.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'triggerEquipIntent\s*=\s*HeldWeaponTriggerEquipIntent\{[\s\S]{0,220}\.formID\s*=\s*selectedRef->GetFormID\(\)[\s\S]{0,120}\.remainingSeconds\s*=\s*0\.35f' `
    'A same-hand trigger edge coincident with grab commit must be retained for that exact weapon instead of being lost.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'replayedSameHandTrigger\s*=\s*triggerEquipIntent\.pending\s*&&\s*heldRefForGameplay\s*&&\s*triggerEquipIntent\.formID\s*==\s*heldRefForGameplay->GetFormID\(\)' `
    'A retained trigger edge must replay only after the same hand holds the exact selected weapon.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'processHand\(_rightHand,\s*false\);\s*publishHandInputOwnership\(_rightHand,\s*false\);[\s\S]{0,120}processHand\(_leftHand,\s*true\);\s*publishHandInputOwnership\(_leftHand,\s*true\)' `
    'Input ownership must be republished immediately after each hand transition so left Pip-Boy suppression cannot lag a committed grab.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_pendingEquippedWeaponPrimaryOnlyGripStart\s*=\s*pendingGripStart' `
    'Loose equip must preserve the originating hand and captured weapon-local frame across inventory transfer.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'firingHandIsLeft\s*&&[\s\S]{0,180}!capturedFiringHandWeaponLocal[\s\S]{0,500}blockFrikPrimaryWeaponPose\(\)[\s\S]{0,500}setFiringHand\(firingHandIsLeft,[\s\S]{0,300}_primaryHandWeaponLocal\s*=\s*\*capturedFiringHandWeaponLocal' `
    'Left primary-only ownership must fail closed without the captured loose-weapon hold and commit the originating hand before transition.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'RE::TESObjectREFR\* gripZoneHoverCandidate\s*=\s*nullptr;\s*if\s*\(\s*!isLeft\s*&&' `
    'Grip-zone hover/equip discovery must not remain right-hand-only.'

if ($failures.Count -gt 0) {
    Write-Host 'Held weapon hand-specific equip source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Held weapon hand-specific equip source boundary passed.'

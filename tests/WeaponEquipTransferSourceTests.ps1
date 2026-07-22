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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.h' `
    'EquippedWeaponMismatch' `
    'Held weapon equip transfer must expose a distinct mismatch reason.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'const auto equippedAfter = readEquippedWeaponSnapshot\(\);[\s\S]{0,260}observedEquippedFormID[\s\S]{0,260}equippedAfter\.weapon != result\.weapon[\s\S]{0,180}EquipReason::EquippedWeaponMismatch[\s\S]{0,180}return result;[\s\S]{0,180}result\.success = true;' `
    'Held weapon equip transfer must verify the runtime equipped base form before reporting success.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'observedEquipped=\{:08X\}' `
    'Auto-equip logging must include the observed equipped form for mismatch diagnosis.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'const bool nativeDrawFollowupRequested\s*=\s*equipResult\.success\s*&&\s*requestHeldWeaponNativeDrawFollowup\(player\);' `
    'The native draw followup must remain gated on verified equip success.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'requestHeldWeaponNativeDrawFollowup[\s\S]{0,420}shouldSubmitDrawFollowup\(nativeState\)[\s\S]{0,180}DrawWeaponMagicHands\(true\)' `
    'The draw followup must use exact FO4VR weapon states instead of broad GetWeaponMagicDrawn semantics.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'canBeginEquip\(nativeStateBeforeEquip\)[\s\S]{0,900}shouldRearmTrigger\(nativeStateBeforeEquip,\s*triggeredByInput\)[\s\S]*hand\.captureHeldReleaseMotion' `
    'A native weapon transition must defer before physical release and preserve the same-hand trigger request.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.h' `
    'struct\s+EquipInput[\s\S]{0,300}NiPointer<RE::TESObjectREFR>\s+heldRef[\s\S]*struct\s+EquipResult[\s\S]{0,700}NiPointer<RE::TESObjectREFR>\s+untransferredRef' `
    'The equip transaction must own the released reference and return it only when native pickup did not acquire it.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'transferHeldWeaponToPlayerAndEquip\(EquipInput input\)[\s\S]{0,300}result\.untransferredRef\s*=\s*std::move\(input\.heldRef\)[\s\S]*ActivateRef\([\s\S]{0,1400}result\.untransferredRef\.reset\(\);[\s\S]*EquipObject\(' `
    'ROCK must release its world-reference lease after ActivateRef and before EquipObject.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'releaseGrabbedObject[\s\S]{0,320}transferHeldWeaponToPlayerAndEquip[\s\S]{0,180}releaseOutcome\.takeRetainedReference\(\)[\s\S]*postEquipRef\s*=\s*equipResult\.untransferredRef\.get\(\)' `
    'The handoff caller must move release ownership into the transaction and use its failure pin for events.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon equip transfer source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon equip transfer source boundary passed.'

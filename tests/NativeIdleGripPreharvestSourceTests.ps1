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
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}














































Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'NativeIdleGripPreharvest\.h[\s\S]*RE::NiPointer<RE::TESObjectREFR>\s+nativeIdleGripCandidate[\s\S]*hand\.getSavedObjectState\(\)\.retainedRef[\s\S]*hand\.getSelection\(\)\.retainedRef[\s\S]*native_idle_grip_preharvest::observeCandidate\(std::move\(nativeIdleGripCandidate\)\)[\s\S]*gripZoneHoverHapticsEnabled' `
    'The frame owner must offer retained held or selected weapons before input commit, independently of optional hover haptics.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'weaponGenerationKey[\s\S]*equippedGenerationMatchesForm[\s\S]*getCurrentObservedEquippedWeaponFormID\(\)\s*==\s*equippedWeapon->formID[\s\S]*native_idle_grip_preharvest::observeEquippedWeapon\([\s\S]*equippedWeapon[\s\S]*weaponNode[\s\S]*currentEquippedWeaponInstanceData\(equippedWeapon\)[\s\S]*_authoredPrimaryFiringGrip\.update' `
    'A stable directly equipped weapon must enter preharvest before authored pose lookup, without pairing the new form with a stale scene generation.'



Require-Text 'src/ROCKMain.cpp' `
    'authored_weapon_grip_cache::preload\(\)' `
    'Persisted grip records must be loaded once during normal ROCK startup before gameplay lookups.'

Reject-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'native_idle_grip_preharvest::observeCandidate' `
    'Native preharvest scheduling must not become coupled to grip-zone projection or hover-haptic feature gates again.'


if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native idle-grip preharvest source boundaries passed.'

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
    'const bool nativeDrawRequested = equipResult\.success && requestImmediateHeldWeaponNativeDraw\(\);' `
    'Native draw requests must remain gated on verified equip success.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon equip transfer source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon equip transfer source boundary passed.'

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

$physics = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$main = 'src/ROCKMain.cpp'

Require-Text $physics `
    'updateEquippedWeaponTransition\(\)[\s\S]{0,1000}weapon_transition_animation_acceleration::service[\s\S]{0,900}runtimeAllowed[\s\S]{0,500}localMenuBlocking[\s\S]{0,500}compatibilityConfigBlocking' `
    'The main frame must service exact identity and runtime cancellation before transition coordination.'
Require-Text $main `
    'held_weapon_instant_transition::install\(\)[\s\S]{0,700}weapon_transition_animation_acceleration::install\(\)' `
    'The clip acceleration capability must install after the held-equip action interceptor.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Weapon transition animation acceleration source boundaries passed.'

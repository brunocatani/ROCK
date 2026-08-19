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

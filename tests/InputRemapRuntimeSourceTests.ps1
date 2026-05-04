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
    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/InputRemapRuntime.cpp' 'ContainerMenu' 'ContainerMenu must stay game-stopping so menu actions such as Store All/Take All receive raw input.'
Require-Text 'src/physics-interaction/InputRemapRuntime.cpp' 'FavoritesMenu' 'FavoritesMenu must stay game-stopping once it is actually open.'
Reject-Text 'src/physics-interaction/InputRemapRuntime.cpp' '"WSLootMenu"' 'WSLootMenu is a gameplay quick-loot overlay and must not keep remap suppression disabled during normal play.'
Require-Text 'src/physics-interaction/InputRemapRuntime.cpp' 'decision\.filteredAxisMask' 'Game-facing input filtering must apply policy-requested OpenVR analog axis suppression.'
Require-Text 'src/physics-interaction/InputRemapRuntime.cpp' 'state->rAxis\[axisIndex\]\.x = 0\.0f;' 'Right trigger suppression must clear the OpenVR Axis1.x analog value, not only the button bit.'
Require-Text 'src/physics-interaction/InputRemapRuntime.cpp' 'suppressRightTriggerGameInput = g_rockConfig\.rockSuppressNativeReadyWeaponAutoReady' 'Native auto-ready suppression must also remove right-trigger game input while holstered.'

if ($failures.Count -gt 0) {
    Write-Host 'Input remap runtime source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Input remap runtime source boundary passed.'

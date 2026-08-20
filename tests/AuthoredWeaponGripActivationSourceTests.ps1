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

Require-Text 'src/RockConfig.cpp' `
    'readClampedFloat\(ini,[\s\S]{0,160}"fWeaponAuthoredGripActivationRadius"[\s\S]{0,160}16\.0f,[\s\S]{0,80}2\.0f,[\s\S]{0,80}32\.0f\)' `
    'The dedicated authored-seat radius must be loaded with its canonical default and bounded runtime range.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'rockDebugDrawAuthoredGripActivationZones[\s\S]*resolveConeBoundaryDimensions[\s\S]*drawWireCone[\s\S]*ENFORCED AUTHORED ACTIVATION' `
    'The pre-grab overlay must be independently enabled and explicitly identify its enforced verdict.'

Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'snapshot\.radialCapGameUnits,\s*12\.0f|axis\.x\s*\*\s*axisLength\s*\+[\s\S]{0,180}tangentA\.x\s*\*\s*radialA' `
    'The debug wire cone must not draw the old sqrt(2)-oversized boundary or clamp the configured reach back to 12 units.'

foreach ($configPath in @('data/config/ROCK.ini')) {
    Require-Text $configPath `
        'fWeaponAuthoredGripActivationRadius\s*=\s*16\.0' `
        'The authored-grip activation radius must have the same bounded default in both canonical configuration copies.'
    Require-Text $configPath `
        'bDrawAuthoredGripActivationZones\s*=\s*false' `
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

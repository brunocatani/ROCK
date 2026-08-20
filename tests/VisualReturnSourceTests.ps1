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

Require-Text 'src/physics-interaction/hand/HandVisual.h' `
    'struct VisualReturnTransition[\s\S]*lastApplied[\s\S]*computeVisualReturnDuration[\s\S]*\(std::max\)\(linearDuration, angularDuration\)[\s\S]*advanceVisualReturn' `
    'The shared return transition must retain its last applied pose and choose the greater translation/rotation duration.'

Require-Text 'src/RockConfig.cpp' `
    'bWeaponVisualReturnEnabled[\s\S]*fWeaponVisualReturnTimeMin[\s\S]*fWeaponVisualReturnTimeMax[\s\S]*fWeaponVisualReturnMinDistance[\s\S]*fWeaponVisualReturnMaxDistance[\s\S]*fWeaponVisualReturnMinAngleDegrees[\s\S]*fWeaponVisualReturnMaxAngleDegrees' `
    'Equipped-weapon visual return must have an independently loaded and clamped setting family.'
Require-Text 'src/RockConfig.cpp' `
    'bGrabHandReturnEnabled[\s\S]*fGrabHandReturnTimeMin[\s\S]*fGrabHandReturnTimeMax[\s\S]*fGrabHandReturnMinDistance[\s\S]*fGrabHandReturnMaxDistance[\s\S]*fGrabHandReturnMinAngleDegrees[\s\S]*fGrabHandReturnMaxAngleDegrees' `
    'Generic grabbed-hand return must have an independently loaded and clamped setting family.'


foreach ($configPath in @('data/config/ROCK.ini')) {
    Require-Text $configPath `
        'bWeaponVisualReturnEnabled\s*=\s*true[\s\S]*fWeaponVisualReturnMaxAngleDegrees[\s\S]*bGrabHandReturnEnabled\s*=\s*true[\s\S]*fGrabHandReturnMaxAngleDegrees' `
        "$configPath must publish both visual-return setting families."
}

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Visual return source boundaries passed.'

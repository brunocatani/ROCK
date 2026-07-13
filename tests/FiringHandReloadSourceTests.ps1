param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'input\.eventHand\s*==\s*input\.firingHand' `
    'Reload policy must require the Activate event and firing grip to belong to the same physical hand.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'eventHandIsLeft\s*=\s*primaryHandEvent\s*\?\s*primaryHandIsLeft\s*:\s*!primaryHandIsLeft[\s\S]{0,260}firingHandIsLeft\s*=\s*s_equippedWeaponLeftHandFiringActive' `
    'Runtime reload routing must convert native primary/secondary wand identity into physical left/right identity and compare it with live firing-hand ownership.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldRouteFiringHandActivateReload\(inputEvent\)[\s\S]{0,700}shouldSuppressNativeTakeEquipActionEvent\(inputEvent\)' `
    'A firing-hand reload press must dispatch before same-button take/equip classification can consume it.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'left X routes reload while the left hand owns the firing grip[\s\S]{0,500}right A cannot reload while the left hand owns the firing grip' `
    'Policy tests must cover both left-X acceptance and opposite right-A rejection during left firing.'

if ($failures.Count -gt 0) {
    Write-Host 'Firing-hand reload source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Firing-hand reload source boundary passed.'

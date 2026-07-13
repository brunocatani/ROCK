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

function Reject-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'input\.eventHandResolved[\s\S]{0,100}input\.eventHand\s*==\s*input\.firingHand' `
    'Reload policy must require an unambiguous physical event hand that matches the firing grip.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldRouteFiringHandActivateReload[\s\S]{0,1800}s_controllers[\s\S]{0,800}resolvePhysicalButtonHand[\s\S]{0,600}firingHandIsLeft\s*=\s*s_equippedWeaponLeftHandFiringActive' `
    'Runtime reload routing must resolve the physical controller from ROCK raw button snapshots and compare it with live firing-hand ownership.'

Reject-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldRouteFiringHandActivateReload[\s\S]{0,1200}primaryHandIsLeft' `
    'Reload routing must not infer physical hand from FO4VR primary/secondary wand identity.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldRouteFiringHandActivateReload\(inputEvent\)[\s\S]{0,700}shouldSuppressNativeTakeEquipActionEvent\(inputEvent\)' `
    'A firing-hand reload press must dispatch before same-button take/equip classification can consume it.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'exclusive physical left button resolves to left[\s\S]{0,1800}missing controller snapshot fails closed[\s\S]{0,10000}left X routes reload while the left hand owns the firing grip[\s\S]{0,1200}right A cannot reload while the left hand owns the firing grip' `
    'Policy tests must cover physical hand resolution, left-X acceptance, ambiguity rejection, and opposite right-A rejection during left firing.'

if ($failures.Count -gt 0) {
    Write-Host 'Firing-hand reload source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Firing-hand reload source boundary passed.'

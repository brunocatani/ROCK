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

# A-side: the hooked ActivateHandler only ever receives PRIMARY-wand events
# (live-verified 2026-07-12), so the route must gate on primary-wand identity
# matching the firing grip, never on a raw-snapshot guess of the event hand.
Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'input\.primaryHandEvent\s*&&\s*input\.firingHandIsPrimaryHand' `
    'A-side reload policy must require a primary-wand event whose physical hand owns the firing grip.'

Reject-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'resolvePhysicalButtonHand' `
    'The raw-snapshot event-hand resolver is retired: it assumed secondary-wand events reach the ActivateHandler (they never do) and rejected legitimate presses on shared-button co-press.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldRouteFiringHandActivateReload[\s\S]{0,1400}isPrimaryWandInputEvent\(event\)[\s\S]{0,600}firingHandIsLeft\s*=\s*s_equippedWeaponLeftHandFiringActive' `
    'Runtime A-side reload routing must derive the event hand from primary-wand identity and compare it with live firing-hand ownership.'

# X-side: the secondary wand accept button never produces an engine event, so
# the reload must be dispatched from ROCK''s own raw press edge each frame.
Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'shouldDispatchSecondaryHandReloadPress[\s\S]{0,600}firingHandIsSecondaryHand\s*&&\s*input\.acceptButtonPressedEdge' `
    'X-side reload policy must gate the raw accept-button press edge on secondary-hand firing-grip ownership.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'updateFiringHandReloadInput[\s\S]{0,2400}consumeRawButtonState\(secondaryHandIsLeft,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]{0,2400}dispatchNativeReloadAction' `
    'Runtime X-side reload must consume the secondary wand''s raw accept press edge every frame and dispatch the native reload action.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'input_remap_runtime::updateFiringHandReloadInput\(\)' `
    'PhysicsInteraction must drive the per-frame X-side reload poll so press edges are consumed before any early return.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldRouteFiringHandActivateReload\(inputEvent\)[\s\S]{0,700}shouldSuppressNativeTakeEquipActionEvent\(inputEvent\)' `
    'A firing-hand reload press must dispatch before same-button take/equip classification can consume it.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'right A cannot reload while the left hand owns the firing grip[\s\S]{0,3000}left X routes reload while the left hand owns the firing grip[\s\S]{0,600}left X cannot reload while the right hand owns the firing grip' `
    'Policy tests must cover left-X acceptance during left firing plus both opposite-hand rejections.'

if ($failures.Count -gt 0) {
    Write-Host 'Firing-hand reload source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Firing-hand reload source boundary passed.'

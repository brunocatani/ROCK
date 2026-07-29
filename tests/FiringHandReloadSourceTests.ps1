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

# Automatic X-side: the secondary wand accept button never produces an engine
# event, so reload must still be dispatched from ROCK's raw press edge. Manual
# scope mode consumes both physical A/X streams before classifying tap vs hold.
Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'shouldDispatchSecondaryHandReloadPress[\s\S]{0,600}firingHandIsSecondaryHand\s*&&\s*input\.acceptButtonPressedEdge' `
    'X-side reload policy must gate the raw accept-button press edge on secondary-hand firing-grip ownership.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'updateFiringHandReloadInput[\s\S]{0,1600}consumeRawButtonState\(true,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]{0,500}consumeRawButtonState\(false,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]{0,5000}secondaryHandIsLeft\s*\?\s*leftAcceptState\s*:\s*rightAcceptState[\s\S]{0,1800}dispatchNativeReloadAction' `
    'Runtime must drain both physical accept streams, preserve automatic secondary-wand reload, and dispatch the native reload action.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'updateFiringHandReloadInput[\s\S]{0,1800}consumeRawButtonState\(true,[\s\S]*consumeRawButtonState\(false,[\s\S]{0,600}isAnyProviderOpenVrGameInputSuppressed\(\)[\s\S]{0,300}blockManualScopeInputUntilRelease\(\)[\s\S]{0,120}return' `
    'Configurator/provider game-input suppression must drain A/X edges and cancel tap/hold state before any reload dispatch.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'dispatchNativeReloadAction[\s\S]{0,900}s_gameplayInputAllowed[\s\S]{0,350}isInputBlockingMenuActive\(\)[\s\S]{0,500}isAnyProviderOpenVrGameInputSuppressedAtDispatch\(\)[\s\S]{0,350}s_weaponDrawn' `
    'The native reload dispatcher must enforce gameplay, menu, provider-lease, and weapon-drawn gates itself.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'currentProviderHandInputSuppressionFlagsAtDispatch[\s\S]{0,500}currentHandInputSuppressionFlagsV1\([\s\S]{0,120}RockProviderHand::Right[\s\S]{0,220}currentHandInputSuppressionFlagsV1\([\s\S]{0,120}RockProviderHand::Left[\s\S]{0,500}isAnyProviderOpenVrGameInputSuppressedAtDispatch' `
    'Reload dispatch must observe a Configurator/provider lease acquired before the later per-hand cache refresh in the same frame.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'setProviderOpenVrGameInputSuppressed[\s\S]{0,800}exchange\([\s\S]{0,450}suppressed\s*&&\s*!wasSuppressed[\s\S]{0,350}blockManualScopeInputUntilRelease' `
    'A newly active provider UI lease must immediately invalidate an in-progress reload gesture.'

Reject-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'requestLocalReloadTestLease|rockNativeReloadAnimationAuthorityTestEnabled' `
    'Input routing must not pre-arm native animation authority; the verified reload-start hook owns that lifecycle.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'input_remap_runtime::updateFiringHandReloadInput\(runtime\.deltaSeconds\)' `
    'PhysicsInteraction must drive per-frame tap/hold arbitration with frame time before any early return.'

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

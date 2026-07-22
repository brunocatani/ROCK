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

# Ghidra-verified FO4VR 1.2.72 MenuControls ownership. These constants keep the
# hook attached to the native semantic Pause handler and route taps through the
# existing PipboyHandler instead of duplicating Bethesda's menu state machine.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'kMenuOpenHandlerHandleEventFunctionOffset\s*=\s*0x1326760' `
    'MenuOpenHandler processor RVA must remain pinned to the verified FO4VR function.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'kMenuOpenHandlerHandleEventVTableSlotOffset\s*=\s*0x2DCC850' `
    'MenuOpenHandler vtable slot must remain pinned to the verified FO4VR slot.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'kMenuControlsSingletonOffset\s*=\s*0x5A3B888[\s\S]{0,180}kMenuControlsPipboyHandlerOffset\s*=\s*0x68[\s\S]{0,180}kPipboyHandlerVTableOffset\s*=\s*0x2DCC778' `
    'Tap routing must resolve and validate the verified native PipboyHandler object.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'kNativeSecondaryWandDeviceIdOffset\s*=\s*0x8D0' `
    'Pause/Pip-Boy arbitration must use Bethesda secondary-wand identity for handedness.'

# The gesture follows semantic Pause and the engine's secondary-wand device,
# never a hard-coded physical left/right controller.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'hookedMenuOpenEventHandler[\s\S]{0,500}eventNameMatches\(inputEvent,\s*kNativeEventPause\)[\s\S]{0,160}isSecondaryWandInputEvent\(inputEvent\)' `
    'Only the native secondary-wand Pause event may enter tap/hold arbitration.'
Reject-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'hookedMenuOpenEventHandler[\s\S]{0,2600}isLeftHandedMode\(' `
    'Pause arbitration must not hard-code a physical hand; native secondary-wand identity owns ambidextrous mapping.'

# A tap replays the native PipboyHandler press/release lifecycle. A hold feeds
# one synthetic release to the original MenuOpenHandler so Bethesda's own
# player/menu eligibility checks remain authoritative.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'dispatchNativePipboyTap[\s\S]{0,1600}strUserEvent\s*=\s*kNativeEventWandTrigger\.data\(\)[\s\S]{0,500}value\s*=\s*1\.0f[\s\S]{0,500}s_originalPipboyEventHandler[\s\S]{0,500}value\s*=\s*0\.0f[\s\S]{0,500}s_originalPipboyEventHandler' `
    'Short Pause release must replay the complete native WandTrigger tap through PipboyHandler.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'dispatchNativePauseHold[\s\S]{0,1200}value\s*=\s*0\.0f[\s\S]{0,700}s_originalMenuOpenEventHandler' `
    'Crossing the hold threshold must dispatch once through Bethesda MenuOpenHandler.'
Require-Text 'src/physics-interaction/input/PipboyPauseGesturePolicy.h' `
    'case State::Pending:[\s\S]{0,900}dispatchPause\s*=\s*true[\s\S]{0,400}dispatchPipboy\s*=\s*true[\s\S]{0,900}State::PauseCommitted' `
    'The pure gesture policy must distinguish short release from a committed Pause hold.'

# Moving Pip-Boy open must not consume the trigger's independent flashlight
# handler. Only the previous interaction/provider suppression remains there.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'decideAndTracePipboyOpenSuppression[\s\S]{0,500}shouldSuppressLegacyPipboyTriggerOpenEvent' `
    'The old secondary-trigger Pip-Boy open must be suppressed during gameplay.'
Reject-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'decideAndTracePipboyLightSuppression[\s\S]{0,700}shouldSuppressLegacyPipboyTriggerOpenEvent' `
    'Moving Pip-Boy open must not remove the native trigger flashlight hold.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'shouldInstallPipboyPauseArbitrationHooks\(settings\.enabled\)[\s\S]{0,180}installPipboyPauseArbitrationHooks' `
    'Enabled ROCK input remapping must always install the three arbitration hooks.'

# The only tuning surface is a bounded duration; both shipped configs must
# expose the same default and the policy tests must lock the state transitions.
Require-Text 'src/RockConfig.cpp' `
    'sanitizedHoldSeconds\([\s\S]{0,180}fPipboyPauseHoldSeconds' `
    'Configured hold duration must be sanitized before runtime use.'
Require-Text 'data/config/ROCK.ini' `
    '(?m)^fPipboyPauseHoldSeconds\s*=\s*0\.35\s*$' `
    'Developer config must ship the 0.35-second hold default.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' `
    '(?m)^fPipboyPauseHoldSeconds\s*=\s*0\.35\s*$' `
    'Packaged config must ship the 0.35-second hold default.'
Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'short Pause-button release opens the Pip-Boy[\s\S]{0,2200}hold threshold opens native Pause exactly once[\s\S]{0,1800}release after Pause hold cannot open the Pip-Boy' `
    'Policy tests must cover tap, hold, one-shot dispatch, and release suppression.'

if ($failures.Count -gt 0) {
    Write-Host 'Pip-Boy/Pause input source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Pip-Boy/Pause input source boundary passed.'

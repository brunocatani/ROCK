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

# Ghidra-verified FO4VR 1.2.72 boundary: MenuOpenHandler has one direct call
# into the native helper after its semantic Pause, eligibility, and primary-
# wand gates. The call target must be decoded and checked before patching.
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'kNativeVatsVansDecisionFunctionOffset\s*=\s*0x0BEB280[\s\S]{0,180}kNativeVatsVansDecisionCallSiteOffset\s*=\s*0x1326990' `
    'The native VATS/V.A.N.S. helper and unique MenuOpenHandler callsite must stay pinned to verified FO4VR RVAs.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'kNativeVansHoldThresholdSettingOffset\s*=\s*0x3756388[\s\S]*readNativeVansHoldThresholdSeconds[\s\S]{0,500}sanitizedHoldSeconds' `
    'ROCK must read and validate the verified native fVANSButtonHeldThreshold value.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'installNativeVatsVansInputSuppressionHook[\s\S]{0,1300}callBytes\[0\]\s*!=\s*0xE8[\s\S]{0,900}decodedTarget\s*!=\s*expectedTarget[\s\S]{0,900}write_call<5>\([\s\S]{0,180}&hookedNativeVatsVansDecision' `
    'The callsite hook must validate both CALL rel32 shape and decoded native target before patching.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    's_nativeVatsVansDecisionHookInstallFailed[\s\S]*installNativeVatsVansInputSuppressionHook[\s\S]{0,500}s_nativeVatsVansDecisionHookInstallFailed\.load[\s\S]{0,2200}s_nativeVatsVansDecisionHookInstallFailed\.store' `
    'A permanent byte-validation failure must be remembered so the frame loop cannot spam retries.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'updateNativeActionSuppressionHooks[\s\S]{0,300}installNativeVatsVansInputSuppressionHook\(\)' `
    'Provider VATS/V.A.N.S. suppression must install independently of ROCK input-remap configuration.'

# Both per-action flags use the existing bounded provider lease contract.
# SuppressOpenVrGameInput remains the broad control and maps to both phases.
Require-Text 'src/api/ROCKProviderApi.h' `
    'SuppressNativeVats\s*=\s*1u\s*<<\s*5[\s\S]{0,180}SuppressNativeVans\s*=\s*1u\s*<<\s*6' `
    'The V1 SDK must publish stable, independent VATS and V.A.N.S. suppression bits.'

Require-Text 'src/api/ROCKProviderApi.cpp' `
    'kImplementedHandInputSuppressionFlagsV1[\s\S]{0,500}SuppressNativeVats[\s\S]{0,220}SuppressNativeVans' `
    'The provider must accept both new flags through the existing lease setter.'

# ROCK.ini retains only the optional release-to-VATS control. V.A.N.S.
# suppression is mandatory and does not route through raw OpenVR state.
Require-Text 'src/RockConfig.h' `
    'rockSuppressNativeVats\s*=\s*false' `
    'RockConfig must default the optional native VATS suppression control off.'
Require-Text 'src/RockConfig.h' `
    '^(?![\s\S]*rockSuppressNativeVans)[\s\S]*$' `
    'RockConfig must not retain local V.A.N.S. configuration authority.'

Require-Text 'src/RockConfig.cpp' `
    'rockSuppressNativeVats\s*=\s*ini\.GetBoolValue\(SECTION,\s*"bSuppressNativeVats",\s*rockSuppressNativeVats\);' `
    'RockConfig must load the optional native VATS suppression control.'
Require-Text 'src/RockConfig.cpp' `
    '^(?![\s\S]*bSuppressNativeVans)[\s\S]*$' `
    'RockConfig must not parse a V.A.N.S. setting.'

foreach ($configPath in @('data/config/ROCK_example.ini')) {
    Require-Text $configPath `
        'bSuppressNativeVats\s*=\s*false' `
        'ROCK_example.ini must expose optional VATS suppression with a safe default.'
    Require-Text $configPath `
        '^(?![\s\S]*bSuppressNativeVans)[\s\S]*$' `
        'ROCK_example.ini must not expose mandatory V.A.N.S. suppression.'
}

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'hookedNativeVatsVansDecision[\s\S]{0,2600}native_vats_input_suppression_policy::update[\s\S]{0,700}\.heldSeconds\s*=\s*button->QHeldDownSecs\(\)[\s\S]{0,180}\.holdSeconds\s*=\s*holdSeconds[\s\S]{0,800}\.suppressVats\s*=\s*g_rockConfig\.rockSuppressNativeVats\s*\|\|[\s\S]{0,500}SuppressNativeVats[\s\S]{0,300}\.suppressVans\s*=\s*true[\s\S]{0,180}\.reserveHoldGesture\s*=\s*true' `
    'The native helper hook must keep tap VATS optional, suppress V.A.N.S., and reserve held release for ROCK.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'observePrimaryVatsGrenadeGesture[\s\S]{0,1500}vats_grenade_gesture_policy::update[\s\S]{0,900}s_pendingGrenadeQuickDrawHoldRequest\.store[\s\S]{0,1600}hookedMenuOpenEventHandler[\s\S]{0,1000}NativeWandIdentity::Primary[\s\S]{0,300}observePrimaryVatsGrenadeGesture' `
    'Primary-wand MenuOpen input must publish quick draw only from the shared hold gesture.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'consumeGrenadeQuickDrawHoldRequest[\s\S]{0,700}s_pendingGrenadeQuickDrawHoldRequest\.exchange[\s\S]{0,500}s_gameplayInputAllowed[\s\S]{0,300}!isInputBlockingMenuActive' `
    'The frame thread must consume one eligible grenade hold request and reject stale menu/gameplay requests.'

# Provider phase flags remain orthogonal. ROCK adds a separate hold reservation:
# short release remains VATS, while a threshold-qualified release is consumed.
Require-Text 'src/physics-interaction/input/NativeVatsInputSuppressionPolicy.h' `
    'if\s*\(input\.buttonDown\)[\s\S]{0,1300}reserveHoldGesture[\s\S]{0,400}holdThresholdReached[\s\S]{0,800}if\s*\(input\.released\)[\s\S]{0,700}suppressHeldGestureRelease[\s\S]{0,500}suppressVats[\s\S]{0,500}reset\(state\)' `
    'The pure policy must consume V.A.N.S. while held and consume VATS release only after the reserved hold threshold.'

Require-Text 'src/physics-interaction/input/VatsGrenadeGesturePolicy.h' `
    'kDefaultHoldSeconds\s*=\s*0\.25f[\s\S]*case State::Idle[\s\S]{0,900}input\.pressed\s*&&\s*input\.held[\s\S]{0,900}State::Pending' `
    'The shared gesture policy must keep the initial B press pending.'
Require-Text 'src/physics-interaction/input/VatsGrenadeGesturePolicy.h' `
    'case State::Pending[\s\S]{0,1400}heldSeconds\s*>=\s*holdSeconds[\s\S]{0,500}State::HoldCommitted[\s\S]{0,180}decision\.requestGrenade\s*=\s*true' `
    'The shared gesture policy must publish grenade exactly once at the native hold threshold.'

Require-Text 'src/physics-interaction/input/NativeVatsInputSuppressionPolicy.h' `
    'input\.justPressed[\s\S]{0,300}reset\(state\)' `
    'A newly observed press must clear any stale latch left by a release hidden behind a menu transition.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'V\.A\.N\.S\.-only suppression preserves release-to-VATS[\s\S]{0,5000}ROCK grenade hold consumes eventual VATS release[\s\S]{0,1800}ROCK short tap preserves release-to-VATS[\s\S]{0,5000}VATS-only suppression preserves the native V\.A\.N\.S\. hold path[\s\S]{0,5000}combined suppression consumes the later release after both leases expire' `
    'Policy tests must preserve provider phase independence while enforcing tap-VATS and hold-grenade exclusivity.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'VATS-button press does not immediately draw grenade[\s\S]{0,2600}VATS hold threshold draws grenade once[\s\S]{0,1800}committed VATS hold does not repeat grenade request[\s\S]{0,2600}short VATS tap never draws grenade' `
    'Gesture tests must reject press-time quick draw, trigger once on hold, and preserve short taps.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'a newly observed press discards stale suppression from a lost release' `
    'Policy tests must prevent a menu-hidden release from poisoning the next physical gesture.'

if ($failures.Count -gt 0) {
    Write-Host 'Native VATS/V.A.N.S. input suppression source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Native VATS/V.A.N.S. input suppression source boundary passed.'

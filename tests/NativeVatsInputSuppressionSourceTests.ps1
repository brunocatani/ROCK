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

# ROCK.ini exposes the same two independent function-level controls without
# routing through the raw OpenVR suppression path.
Require-Text 'src/RockConfig.h' `
    'rockSuppressNativeVats\s*=\s*false[\s\S]{0,180}rockSuppressNativeVans\s*=\s*false' `
    'RockConfig must default both local native-action suppression controls off.'

Require-Text 'src/RockConfig.cpp' `
    'rockSuppressNativeVats\s*=\s*ini\.GetBoolValue\(SECTION,\s*"bSuppressNativeVats",\s*rockSuppressNativeVats\);[\s\S]{0,300}rockSuppressNativeVans\s*=\s*ini\.GetBoolValue\(SECTION,\s*"bSuppressNativeVans",\s*rockSuppressNativeVans\);' `
    'RockConfig must load both local native-action suppression controls.'

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'bSuppressNativeVats\s*=\s*false[\s\S]{0,500}bSuppressNativeVans\s*=\s*false' `
        'Shipped ROCK.ini must expose both native-action controls with safe defaults.'
}

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'hookedNativeVatsVansDecision[\s\S]{0,1800}SuppressOpenVrGameInput[\s\S]{0,600}native_vats_input_suppression_policy::update[\s\S]{0,1000}\.suppressVats\s*=\s*g_rockConfig\.rockSuppressNativeVats\s*\|\|[\s\S]{0,500}SuppressNativeVats[\s\S]{0,500}\.suppressVans\s*=\s*g_rockConfig\.rockSuppressNativeVans\s*\|\|[\s\S]{0,500}SuppressNativeVans' `
    'The native helper hook must combine local INI controls with broad, VATS-only, and V.A.N.S.-only provider leases.'

# The policy is deliberately orthogonal: VATS-only forwards down samples so
# V.A.N.S. remains available; V.A.N.S.-only forwards release so normal VATS
# remains available. Each suppression latch survives lease expiry until the
# corresponding physical gesture reaches release.
Require-Text 'src/physics-interaction/input/NativeVatsInputSuppressionPolicy.h' `
    'if\s*\(input\.buttonDown\)[\s\S]{0,900}suppressVansWhileDown[\s\S]{0,400}decision\.forwardNative\s*=\s*false[\s\S]{0,700}if\s*\(input\.released\)[\s\S]{0,500}suppressVatsOnRelease[\s\S]{0,500}reset\(state\)' `
    'The pure policy must suppress V.A.N.S. on down, VATS on release, and rearm only after release.'

Require-Text 'src/physics-interaction/input/NativeVatsInputSuppressionPolicy.h' `
    'input\.justPressed[\s\S]{0,300}reset\(state\)' `
    'A newly observed press must clear any stale latch left by a release hidden behind a menu transition.'

Require-Text 'tests/InputRemapPolicyTests.cpp' `
    'V\.A\.N\.S\.-only suppression preserves release-to-VATS[\s\S]{0,2600}VATS-only suppression preserves the native V\.A\.N\.S\. hold path[\s\S]{0,5000}combined suppression consumes the later release after both leases expire' `
    'Policy tests must lock both independent modes, combined mode, and lease-expiry latching.'

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

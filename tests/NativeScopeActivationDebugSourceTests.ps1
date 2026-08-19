param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing file for text check")
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if ((Test-Path -LiteralPath $path) -and ((Get-Content -Raw -LiteralPath $path) -match $Pattern)) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.h' 'rockDebugDrawNativeScopeActivation\s*=\s*false' `
    'Native-scope visualization must have a dedicated opt-in runtime gate.'
Require-Text 'src/RockConfig.cpp' 'rockDebugDrawNativeScopeActivation\s*=\s*false[\s\S]*bDebugDrawNativeScopeActivation' `
    'The diagnostic gate must reset fail-closed and load from ROCK.ini.'
Require-Text 'src/RockConfig.h' 'sole native-scope activation[\s\S]*rockManualScopeHoldSeconds\s*=\s*0\.30f' `
    'The firing-hand hold must be the sole native-scope activation contract.'
Require-Text 'src/RockConfig.cpp' 'fManualScopeHoldSeconds' `
    'The native-scope hold threshold must load from ROCK.ini.'
Reject-Text 'src/RockConfig.h' 'rockAutoActivateScope' `
    'The retired cone/button mode switch must not remain in runtime configuration.'
Reject-Text 'src/RockConfig.cpp' 'bAutoActivateScope|rockAutoActivateScope' `
    'The retired cone/button mode switch must not remain in config loading.'
Require-Text 'src/RockConfig.h' 'rockNativeScopeForceFiringGripFallback\s*=\s*false[\s\S]*rockNativeScopeFiringGripFallbackOffsetXGameUnits\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackOffsetYGameUnits\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackOffsetZGameUnits\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackPitchDegrees\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackYawDegrees\s*=\s*0\.0f[\s\S]*rockNativeScopeFiringGripFallbackRollDegrees\s*=\s*0\.0f' `
    'Missing-optic fallback must expose one force switch and a neutral six-degree-of-freedom Weapon-local firing-grip frame.'
Require-Text 'src/RockConfig.cpp' 'bNativeScopeForceFiringGripFallback[\s\S]*fNativeScopeFiringGripFallbackOffsetXGameUnits[\s\S]*fNativeScopeFiringGripFallbackOffsetYGameUnits[\s\S]*fNativeScopeFiringGripFallbackOffsetZGameUnits[\s\S]*fNativeScopeFiringGripFallbackPitchDegrees[\s\S]*fNativeScopeFiringGripFallbackYawDegrees[\s\S]*fNativeScopeFiringGripFallbackRollDegrees' `
    'Firing-grip fallback position and rotation controls must load from the NativeScopes section.'
Require-Text 'data/config/ROCK.ini' 'bDebugDrawNativeScopeActivation\s*=\s*false' `
    'The development config template must keep the native-scope diagnostic disabled by default.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' 'bDebugDrawNativeScopeActivation\s*=\s*false' `
    'The packaged config template must keep the native-scope diagnostic disabled by default.'
Require-Text 'src/RockConfig.h' 'rockNativeScopeOverlayOffsetXGameUnits[\s\S]*rockNativeScopeOverlayOffsetYGameUnits[\s\S]*rockNativeScopeOverlayOffsetZGameUnits[\s\S]*rockNativeScopeOverlayPitchDegrees[\s\S]*rockNativeScopeOverlayYawDegrees[\s\S]*rockNativeScopeOverlayRollDegrees' `
    'Native scope overlay placement must expose three model-local position and three rotation tuning values.'
Require-Text 'src/RockConfig.cpp' 'NATIVE_SCOPES_SECTION\s*=\s*"NativeScopes"[\s\S]*fNativeScopeOverlayOffsetXGameUnits[\s\S]*fNativeScopeOverlayOffsetYGameUnits[\s\S]*fNativeScopeOverlayOffsetZGameUnits[\s\S]*fNativeScopeOverlayPitchDegrees[\s\S]*fNativeScopeOverlayYawDegrees[\s\S]*fNativeScopeOverlayRollDegrees' `
    'Native scope overlay tuning must load from its independent NativeScopes INI section.'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath '\[NativeScopes\][\s\S]*fManualScopeHoldSeconds\s*=\s*0\.30' `
        'Native scope templates must expose the sole A/X hold threshold.'
    Reject-Text $configPath 'bAutoActivateScope' `
        'Native scope templates must not advertise the retired cone activation path.'
    Require-Text $configPath '\[NativeScopes\][\s\S]*bNativeScopeForceFiringGripFallback\s*=\s*false[\s\S]*fNativeScopeFiringGripFallbackOffsetXGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackOffsetYGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackOffsetZGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackPitchDegrees\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackYawDegrees\s*=\s*0\.0[\s\S]*fNativeScopeFiringGripFallbackRollDegrees\s*=\s*0\.0' `
        'Native scope templates must expose a neutral, opt-in-force six-degree-of-freedom firing-grip fallback.'
    Require-Text $configPath '\[NativeScopes\][\s\S]*fNativeScopeOverlayOffsetXGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayOffsetYGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayOffsetZGameUnits\s*=\s*0\.0[\s\S]*fNativeScopeOverlayPitchDegrees\s*=\s*0\.0[\s\S]*fNativeScopeOverlayYawDegrees\s*=\s*0\.0[\s\S]*fNativeScopeOverlayRollDegrees\s*=\s*0\.0' `
        'Native scope overlay template tuning must default to a neutral additive transform.'
}

Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'alignOpticalAxesToBore|camera \+X[\s\S]*bore \+Y' `
    'Scope presentation must not invent a camera-to-bore axis mapping over the captured native frame.'
Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'followWeaponWorldChange\s*\(' `
    'The controller-relative rigid-delta fallback must stay removed.'
Require-Text 'src/ROCKMain.cpp' 'hookNativeScopeGeometryDecision[\s\S]*callBytes\[0\]\s*!=\s*0xE8[\s\S]*decodedTarget\s*!=\s*expectedTarget[\s\S]*kExpectedNativeDecisionTest[\s\S]*write_call<5>\(callSiteAddress,\s*&onNativeScopeGeometryDecision\)[\s\S]*kRockDecisionTest[\s\S]*REL::safe_write' `
    'The exact verified geometry call site and original target must be validated before patching.'
Require-Text 'src/ROCKMain.cpp' 'bool onNativeScopeGeometryDecision[\s\S]*finalGeometryDecision\s*=\s*nativeGeometryDecision[\s\S]*nativeForceDecision[\s\S]*finalGeometryDecision\s*=\s*input_remap_runtime::isManualScopeActivationRequested\(\)[\s\S]*buttonDecisionApplied\s*=\s*true[\s\S]*s_originalNativeScopeStateTransition\(player,\s*finalGeometryDecision\)[\s\S]*buttonDecisionApplied\s*\?\s*true\s*:\s*finalGeometryDecision' `
    'The hook must preserve Bethesda force priority, discard ordinary cone results, and drive transitions only from held input.'
Reject-Text 'src/ROCKMain.cpp' 'rockAutoActivateScope|tryResolveNativeScopeGeometryDecision' `
    'The top-level scope hook must not retain a selectable cone path.'
Require-Text 'src/ROCKMain.cpp' 'configureNativeWorldScopeForManualTarget[\s\S]*kFunc_NativeWorldScopeConfigure[\s\S]*kData_NativeWorldScopeSingleton[\s\S]*kData_NativeWorldScopePrimaryVtable[\s\S]*driveManualScopeTransitionFallback[\s\S]*nativeForceDecision[\s\S]*tryGetManualScopeDirectTransitionTarget[\s\S]*isManualScopeActivationRequested\(\)[\s\S]*configureNativeWorldScopeForManualTarget[\s\S]*s_originalNativeScopeStateTransition\(player,\s*true\)[\s\S]*s_originalNativeScopeStateTransition\(player,\s*false\)' `
    'Manual hold must validate and configure native WSScope before directly transitioning an unflagged magnified scope.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' 'consumeRawButtonState\(true,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]*consumeRawButtonState\(false,\s*input_remap_policy::kOpenVrAcceptButtonId\)[\s\S]*manual_scope_input_policy::update[\s\S]*decision\.scopeRequested[\s\S]*decision\.dispatchReload' `
    'Button-only scope and release-time reload must share one physical firing-hand A/X gesture classifier.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' 'shouldDeferFiringHandActivateForManualScope\(inputEvent\)[\s\S]*markInputEventStopped\(inputEvent\)[\s\S]*return;' `
    'Primary-wand press-time reload must be deferred while manual scope classifies the hold.'
Require-Text 'src/ROCKMain.cpp' 's_originalGameLoopFunc\(rcx\);[\s\S]*synchronizeNativeScopePresentationAfterFrikUpdate\(\);[\s\S]*onFrameUpdate\(\);' `
    'Presentation must synchronize after hFRIK and before ROCK final weapon authority.'
Reject-Text 'src/ROCKMain.cpp' 'prepareNativeScopeCameraForGameUpdate|finalizeNativeScopeOverlayAfterGameUpdate' `
    'The disproven pre/post displaced-call scope handoff must not remain.'

$overlay = 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp'
Require-Text $overlay 'drawNativeScopeActivation\s*=\s*g_rockConfig\.rockDebugDrawNativeScopeActivation' `
    'Overlay publication must be independently gated by the native-scope diagnostic setting.'
Require-Text $overlay 'primaryWeaponScopeCamera[\s\S]*scopeCamera->world[\s\S]*composeTransforms\(scopeCameraParent->world,\s*scopeCamera->local\)' `
    'The overlay must distinguish the engine node stored world from its parent/local recomposition.'
Require-Text $overlay 'getNativeScopeSightAnchorSnapshot\(\)[\s\S]*sightBoundsMinWeaponLocal[\s\S]*kBoundsEdges[\s\S]*NativeScopeSightBounds' `
    'The generated Sight union and rear-center anchor must be visible in weapon world space.'
Require-Text $overlay 'getNativeScopeCameraDebugSnapshot\(\)[\s\S]*NativeScopePreWriteCamera[\s\S]*NativeScopeImmediateReadback' `
    'The overlay must expose the recorded pre-write and immediate-readback handoff stages.'
Require-Text $overlay 'getNativeScopeResolvedAnchorSnapshot[\s\S]*FIRING GRIP FALLBACK ANCHOR[\s\S]*scopeAnchorSourceName[\s\S]*writeSnapshot\.anchorSource' `
    'The overlay must expose the selected generated-sight or firing-grip anchor used by the actual camera write.'
Require-Text $overlay 'getNativeScopeCameraTargetPreviewSnapshot\(\)[\s\S]*matchesCurrentEquippedWeapon\([\s\S]*resolveRigidAnchorFrameWorld\([\s\S]*targetPreviewSnapshot\.cameraWeaponLocal[\s\S]*targetFromResolvedPreview\s*=\s*true' `
    'The pre-activation visualizer must resolve the same identity-bound weapon-local target used by the camera writer.'
Require-Text $overlay 'FALLBACK PREVIEW ORIGIN[\s\S]*FALLBACK AIM \(\+X\)[\s\S]*FALLBACK UP \(\+Z\)[\s\S]*scopeMenuIndependent=yes' `
    'Fallback tuning must show a persistent origin, pointing direction, and roll-readable up guide while ScopeMenu is closed.'
Reject-Text $overlay 'applyWeaponLocalRotationOffset\s*\(' `
    'The visualizer must consume the retained production target instead of duplicating fallback rotation math.'
Require-Text $overlay 'fallback tune:[\s\S]*rockNativeScopeFiringGripFallbackOffsetXGameUnits[\s\S]*rockNativeScopeFiringGripFallbackPitchDegrees[\s\S]*rockNativeScopeFiringGripFallbackYawDegrees[\s\S]*rockNativeScopeFiringGripFallbackRollDegrees' `
    'The in-game diagnostic panel must expose the active firing-grip fallback position and rotation tuning.'
Require-Text $overlay 'scopeWriteSourceName[\s\S]*post-frik-presentation-sync[\s\S]*weapon-visual-authority[\s\S]*writeSnapshot\.writeSource' `
    'The in-game panel must distinguish presentation synchronization from final weapon authority.'
Require-Text $overlay 'hmdPositionWorld[\s\S]*NativeScopeHmd[\s\S]*HMD->live[\s\S]*HMD->target' `
    'The headset relationship to the live and intended activation anchors must be visible and quantified.'
Require-Text $overlay 'getNativeScopeActivationDebugSnapshot[\s\S]*scope input seq=[\s\S]*manualInputRequested[\s\S]*rendererStateValid[\s\S]*rendererActive' `
    'The physical button request and renderer response must be visible at runtime.'

if ($failures.Count -gt 0) {
    Write-Host 'NativeScopeActivationDebugSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'NativeScopeActivationDebugSourceTests passed.' -ForegroundColor Green

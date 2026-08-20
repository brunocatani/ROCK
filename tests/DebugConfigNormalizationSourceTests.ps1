param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)
    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path) -or (Get-Content -Raw -LiteralPath $path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)
    $path = Join-Path $Root $RelativePath
    if ((Test-Path -LiteralPath $path) -and (Get-Content -Raw -LiteralPath $path) -match $Pattern) {
        $failures.Add($Message)
    }
}

if (Test-Path -LiteralPath (Join-Path $Root 'data/mod/ROCK_Config/ROCK.ini')) {
    $failures.Add('The packaged tree must not retain a second ROCK.ini authority.')
}

Require-Text 'CMakeLists.txt' 'data/config/ROCK\.ini"\s+"\$\{copy_path\}/ROCK_Config/ROCK\.ini' `
    'Local deployment must copy the canonical ROCK.ini.'
Require-Text 'cmake/package.cmake' 'data/config/ROCK\.ini"\s+DESTINATION\s+"\$\{PACKAGE_STAGE_DIR\}/ROCK_Config' `
    'Release packaging must copy the canonical ROCK.ini.'
Require-Text 'data/config/ROCK.ini' '(?ms)^\[Debug\].*?^bEnabled\s*=\s*true\s*$.*?^bControllerEnabled\s*=\s*false\s*$.*?^bMonitorEnabled\s*=\s*false\s*$' `
    'The canonical debug hierarchy must preserve the production gates.'
Require-Text 'src/RockConfig.cpp' 'resolveEffectiveDebugSettings[\s\S]*colliderOverlayEnabled[\s\S]*rockDebugShowTargetColliders[\s\S]*rockDebugDrawDynamicHandColliders[\s\S]*rockDebugDrawDynamicWeaponColliders[\s\S]*rockDebugProviderColliderFocusEnabled' `
    'Every collider child must resolve through the collider master.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDiagnostics.cpp' 'colliderClockDebugActive\s*=\s*g_rockConfig\.rockDebugColliderClockLogging' `
    'Collider-clock logging must use its explicit channel.'
Require-Text 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'if \(!g_rockConfig\.rockDebugControllerEnabled\)' `
    'The XInput debug controller must fail closed before polling.'
Require-Text 'src/monitor/GrabClockMonitorConfig.h' 'bool enabled\{ false \}' `
    'The grab-clock monitor must default disabled.'
Require-Text 'src/RockConfig.cpp' 'warnUnknownKeys[\s\S]*Unknown ROCK\.ini setting' `
    'The loader must report unknown settings.'
Require-Text 'src/RockConfig.cpp' 'readClampedFloat\(ini,\s*DEBUG_OVERLAY_SECTION,\s*"fGrabRenderClockProbeOffsetGameUnits"[\s\S]*readClampedFloat\(ini,\s*DEBUG_OVERLAY_SECTION,\s*"fGrabSceneWriterProbeOffsetZGameUnits"' `
    'Visual clock probes must load from the debug overlay section.'
Require-Text 'src/RockConfig.cpp' 'if \(!rockDebugOverlayEnabled\) \{\s*rockGrabRenderClockProbeOffsetGameUnits = 0\.0f;\s*rockGrabSceneWriterProbeOffsetZGameUnits = 0\.0f;' `
    'The overlay master must neutralize visual clock probes.'
Require-Text 'src/physics-interaction/weapon/collision/WeaponCollisionOmodAudit.cpp' 'kMandatoryOmodSelfHealIntervalFrames\s*=\s*450[\s\S]*ROCK_OMOD_DIAGNOSTIC_INFO[\s\S]*if \(selfHealCandidates\.empty\(\)\)' `
    'OMOD self-heal must be mandatory while detailed OMOD logging remains gated.'
Require-Text 'src/physics-interaction/weapon/collision/WeaponCollision.cpp' 'if \(!generationDrivenRebuild \|\| omodPrebuildAuditCurrent\)' `
    'Pre-build OMOD enrichment must not depend on a user config switch.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' 'kMandatoryNativeInputSuppression\s*=\s*true[\s\S]*input_remap_policy::Settings\{[\s\S]{0,500}\.enabled\s*=\s*g_rockConfig\.rockEnabled[\s\S]{0,500}\.suppressRightGrabGameInput\s*=\s*kMandatoryNativeInputSuppression[\s\S]{0,500}\.suppressRightTriggerGameInput\s*=\s*kMandatoryNativeInputSuppression[\s\S]{0,500}\.suppressNativeMeleeThrowGameInput\s*=\s*kMandatoryNativeInputSuppression' `
    'Core ROCK input ownership must not depend on user suppression switches.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' 'pipboy_pause_gesture_policy::Input\{[\s\S]{0,300}\.enabled\s*=\s*g_rockConfig\.rockEnabled' `
    'The Y-button Pip-Boy/Pause route must follow only the ROCK core gate.'
Require-Text 'data/config/ROCK.ini' '(?ms)^\[Input\].*?^bSuppressTakeEquipGameInputWhileHolding\s*=\s*true\s*$' `
    'Take/equip suppression while holding must remain user-configurable.'

$obsolete = 'bDebugDrawGrabProxySemanticAxesOnly|bShoulderStashSkipActivateBooks|bShoulderStashSkipActivateNotes|bWeaponCollisionNativeVisualRemapEnabled|bDebugGrabFingerPoseLogging|bDebugWorkbenchWeaponReattach|bWeaponOmodSelfHealEnabled|rockExperimentalWeaponOmodSelfHealEnabled|bInputRemapEnabled|rockInputRemapEnabled|bSuppressRightGrabGameInput|rockSuppressRightGrabGameInput|bSuppressNativeReadyWeaponAutoReady|rockSuppressNativeReadyWeaponAutoReady|bSuppressNativeMeleeThrowGameInput|rockSuppressNativeMeleeThrowGameInput'
Reject-Text 'data/config/ROCK.ini' $obsolete 'The canonical INI must not contain obsolete settings.'
Reject-Text 'src/RockConfig.h' $obsolete 'RockConfig must not expose obsolete settings.'
Reject-Text 'src/RockConfig.cpp' $obsolete 'RockConfig must not load obsolete settings.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugConfigNormalizationSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugConfigNormalizationSourceTests passed.' -ForegroundColor Green

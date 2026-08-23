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
    if (-not (Test-Path -LiteralPath $path)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kFunc_BhkWorldSetDeltaTime\s*=\s*0x1DF7120' `
    'FO4VR bhkWorld::SetDeltaTime offset must be named explicitly near the hook.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kHookSite_BhkWorldSetDeltaTimeMainCall\s*=\s*0x0D84BD0' `
    'The verified main update call site must be named explicitly near the hook.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'validateBhkWorldSetDeltaTimeMainCallSite[\s\S]*0x140D84BD0[\s\S]*0x141DF7120' `
    'Timing fix must validate the verified call site before installing.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'g_originalBhkWorldSetDeltaTime\(rawDeltaSeconds\);[\s\S]*tryReadBhkWorldTimingState' `
    'Timing hook must call native SetDeltaTime and preserve its state before the late coherent decision.'
Require-Text 'src/ROCKMain.cpp' 'beginFrameTiming\([\s\S]*applyHavokTimingFixForGameFrame\(frameTiming\)[\s\S]*BeforeRock' `
    'The coherent schedule must be applied after the shared source sample and before target publication.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'tryReadGlobalSimulationTimeMultiplier\(globalTimeMultiplier\)[\s\S]*sourceDeltaSeconds = frameTiming\.deltaSeconds' `
    'The late decision must scale the authoritative source interval only by the verified native global simulation multiplier.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_GlobalSimulationTimeMultiplier\s*=\s*0x3881630' `
    'The FO4VR global simulation multiplier offset must remain explicit and version-auditable.'
Reject-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'BSTimer::QGlobalTimeMultiplier' `
    'CommonLibF4VR leaves the VR multiplier relocation unresolved and this accessor crashes.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'tryWriteBhkWorldTimingState\(coherent\)' `
    'Timing fix must publish one coherent timing state instead of independent partial writes.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'remainderDeltaSeconds = decision\.presentationPhaseSeconds[\s\S]*previousRemainderDeltaSeconds = decision\.presentationPhaseSeconds' `
    'The coherent schedule must publish the completed-solve phase to both global remainder fields.'
Require-Text 'src/physics-interaction/native/HavokTimingFixPolicy.h' 'Publish zero globally[\s\S]*characterPresentationPhaseInitialized' `
    'The timing policy must separate global collision-object presentation from the player character phase.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kVtableEntry_CharProxyController_Presentation\s*=\s*0x2E894E0[\s\S]*kFunc_CharProxyController_Presentation\s*=\s*0x1E4E050[\s\S]*kVtableEntry_CharRigidBodyController_Presentation\s*=\s*0x2E89CE0[\s\S]*kFunc_CharRigidBodyController_Presentation\s*=\s*0x1E53CD0' `
    'Both verified FO4VR character-controller presentation entries must remain explicit and version-auditable.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'isPlayerCharacterController\(controller\)[\s\S]*\*liveRemainder = characterPhase;[\s\S]*original\(controller, output, applyCenterOffset\);[\s\S]*\*liveRemainder = globalPresentationPhase;' `
    'Only the player controller may receive the shadow native phase, and the global phase must be restored immediately.'
Reject-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'remainderDeltaSeconds = decision\.nativeNextRemainderDeltaSeconds' `
    'The native fixed-step remainder cycle must not drive an every-frame coherent physics schedule.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kData_BhkWorldRawDeltaSeconds[\s\S]*kData_BhkWorldSubstepDeltaSeconds[\s\S]*kData_BhkWorldRemainderDeltaSeconds[\s\S]*kData_BhkWorldPreviousRemainderDeltaSeconds[\s\S]*kData_BhkWorldAccumulatedDeltaSeconds[\s\S]*kData_BhkWorldSubstepCount' `
    'The coherent timing transaction must own every native delta, remainder, accumulated-time, and substep-count field.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'rockDebugVerboseLogging[\s\S]*rockDebugGrabFrameLogging[\s\S]*HAVOK_TIMING_FIX' `
    'Timing-fix diagnostics must remain behind existing explicit debug logging gates.'
Require-Text 'src/RockConfig.h' 'rockHavokTimingFixEnabled[\s\S]*rockHavokTimingFixMinPhysicsFrameRate[\s\S]*rockHavokTimingFixMaxSubsteps' `
    'RockConfig must expose timing-fix settings.'
Require-Text 'src/RockConfig.cpp' 'bHavokTimingFixEnabled[\s\S]*fHavokTimingFixMinPhysicsFrameRate[\s\S]*iHavokTimingFixMaxSubsteps' `
    'RockConfig must load timing-fix settings from ROCK.ini.'
Require-Text 'src/ROCKMain.cpp' 'g_rockConfig\.load\(\);[\s\S]*installHavokTimingFixHook\(\)' `
    'Timing hook must be installed after config load so config reload can toggle behavior.'
Reject-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kData_BhkWorldBaseSubstepDeltaSeconds' `
    'Timing fix must not write the engine base substep global; original SetDeltaTime should reset state when disabled.'

if ($failures.Count -gt 0) {
    Write-Host 'HavokTimingFixSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'HavokTimingFixSourceTests passed.' -ForegroundColor Green

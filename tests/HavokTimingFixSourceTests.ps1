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

Require-Text 'docs/implementation-notes/2026-06-01-havok-timing-fix.md' '0x141DF7120[\s\S]*0x140D84BD0' `
    'Timing-fix note must record the Ghidra-verified SetDeltaTime function and hook call site.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kFunc_BhkWorldSetDeltaTime\s*=\s*0x1DF7120' `
    'FO4VR bhkWorld::SetDeltaTime offset must be named explicitly near the hook.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kHookSite_BhkWorldSetDeltaTimeMainCall\s*=\s*0x0D84BD0' `
    'The verified main update call site must be named explicitly near the hook.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'validateBhkWorldSetDeltaTimeMainCallSite[\s\S]*0x140D84BD0[\s\S]*0x141DF7120' `
    'Timing fix must validate the verified call site before installing.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'g_originalBhkWorldSetDeltaTime\(rawDeltaSeconds\);[\s\S]*writeBhkWorldFloatGlobal\(offsets::kData_BhkWorldSubstepDeltaSeconds' `
    'Timing fix must call the original SetDeltaTime before overriding current timing globals.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'writeBhkWorldUintGlobal\(offsets::kData_BhkWorldSubstepCount' `
    'Timing fix must override FO4VR substep count together with substep delta.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'rockDebugVerboseLogging[\s\S]*rockDebugGrabFrameLogging[\s\S]*HAVOK_TIMING_FIX' `
    'Timing-fix diagnostics must remain behind existing explicit debug logging gates.'
Require-Text 'src/RockConfig.h' 'rockHavokTimingFixEnabled[\s\S]*rockHavokTimingFixMinPhysicsFrameRate[\s\S]*rockHavokTimingFixMaxSubsteps' `
    'RockConfig must expose timing-fix settings.'
Require-Text 'src/RockConfig.cpp' 'bHavokTimingFixEnabled[\s\S]*fHavokTimingFixMinPhysicsFrameRate[\s\S]*iHavokTimingFixMaxSubsteps' `
    'RockConfig must load timing-fix settings from ROCK.ini.'
Require-Text 'data/config/ROCK.ini' 'bHavokTimingFixEnabled[\s\S]*fHavokTimingFixMinPhysicsFrameRate[\s\S]*iHavokTimingFixMaxSubsteps' `
    'Packaged ROCK.ini must document timing-fix settings.'
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

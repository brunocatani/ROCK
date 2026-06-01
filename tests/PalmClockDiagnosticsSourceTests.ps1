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
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' '_palmClockGameFrameIndex[\s\S]*_palmClockGameDeltaSeconds' `
    'Palm clock diagnostics must publish the game-frame stamp through atomic state for physics callbacks.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'logPalmClockSampleForHand[\s\S]*PALM_CLOCK' `
    'Palm clock diagnostics must emit a structured PALM_CLOCK row.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.tryGetPalmAnchorTarget\(palmTargetWorld\)[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*livePalm\)' `
    'Palm clock diagnostics must compare the queued palm target against the live palm-anchor body.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'rawToTarget[\s\S]*rawToLive[\s\S]*targetToLive' `
    'Palm clock diagnostics must include raw-to-target, raw-to-live, and target-to-live deltas.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'game-after-hand-collider-queue' `
    'Palm clock diagnostics must sample after the game-frame hand collider queue.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'game-before-held-update' `
    'Palm clock diagnostics must sample immediately before held-object update.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'physics-after-collider-drive[\s\S]*physics-between-before-grab-flush[\s\S]*physics-after-solve' `
    'Palm clock diagnostics must sample the Havok collider, grab-authority, and post-solve phases.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'rockDebugGrabTimelineTrace[\s\S]*rockDebugGrabFrameLogging[\s\S]*rockDebugVerboseLogging' `
    'Palm clock diagnostics must remain behind existing explicit debug logging gates.'

if ($failures.Count -gt 0) {
    Write-Host 'PalmClockDiagnosticsSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'PalmClockDiagnosticsSourceTests passed.' -ForegroundColor Green

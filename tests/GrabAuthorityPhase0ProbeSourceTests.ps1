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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Path {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (Test-Path -LiteralPath $path) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Reject-Path 'src/physics-interaction/native/GrabAuthorityPhase0Probe.h' `
    'Phase 0 diagnostic probe header should be removed now that proxy grab authority is production.'
Reject-Path 'src/physics-interaction/native/GrabAuthorityPhase0Probe.cpp' `
    'Phase 0 diagnostic probe implementation should be removed now that proxy grab authority is production.'

Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'betweenCollideAndSolve\) == 0x28[\s\S]*afterBetweenCollideAndSolve\) == 0x30[\s\S]*betweenCollideAndSolveNoop[\s\S]*afterBetweenCollideAndSolve[\s\S]*onAfterBetweenCollideAndSolve' `
    'The +0x28 graph-builder slot must remain a no-op and +0x30 finish must own the pre-solve grab flush.'
Require-Text 'src/physics-interaction/native/PhysicsStepDriveCoordinator.cpp' 'BetweenCollideAndSolveFinish' `
    'The callback timing sample must identify the verified post-graph, pre-solve finish boundary.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.h' 'GrabAuthorityPhase0|grab_authority_phase0|Phase0' `
    'PhysicsInteraction header must not retain Phase 0 probe declarations, includes, or members.'
Reject-Text 'src/RockConfig.h' 'GrabAuthorityPhase0|Phase0' `
    'RockConfig must not expose removed Phase 0 diagnostic settings.'
Reject-Text 'src/RockConfig.cpp' 'GrabAuthorityPhase0|Phase0' `
    'RockConfig must not read removed Phase 0 diagnostic settings.'

if ($failures.Count -gt 0) {
    Write-Host "GrabAuthorityPhase0ProbeSourceTests failed:" -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host "GrabAuthorityPhase0ProbeSourceTests passed." -ForegroundColor Green

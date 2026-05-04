param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$text = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/ROCKMain.cpp')

function Require-Pattern {
    param(
        [string]$Pattern,
        [string]$Message
    )

    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Pattern 'void ensurePhysicsInteractionForReadySkeleton\(\)' 'ROCKMain must have an explicit lazy creation path for config re-enable after FRIK skeleton readiness.'
Require-Pattern 'frikApi->isSkeletonReady\(\)[\s\S]{0,360}createPhysicsInteraction\(\)' 'Lazy creation must require a ready FRIK skeleton before creating PhysicsInteraction.'
Require-Pattern 'onFrameUpdate\(\)[\s\S]{0,900}ensurePhysicsInteractionForReadySkeleton\(\)' 'Frame update must call lazy lifecycle recovery after config/input gating.'
Require-Pattern '!s_physicsInteraction[\s\S]{0,260}createPhysicsInteraction\(\)' 'Lifecycle recovery must create PhysicsInteraction when no instance exists.'
Require-Pattern 's_physicsInteraction->update\(\)[\s\S]{0,120}publishPhysicsInteractionIfReady\(\)' 'Existing PhysicsInteraction frame updates must still publish after deferred initialization succeeds.'

if ($failures.Count -gt 0) {
    Write-Host 'ROCK main lifecycle source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ROCK main lifecycle source boundary passed.'

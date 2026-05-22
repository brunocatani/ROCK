param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.h' 'RE::hknpWorld\* world' `
    'Runtime collision suppression entries must be bound to the hknp world.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.h' 'RE::NiCollisionObject\* collisionObject' `
    'Runtime collision suppression entries must capture native collision-object identity.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.h' 'RE::NiAVObject\* ownerNode' `
    'Runtime collision suppression entries must capture owner-node identity.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.h' 'staleLeaseDiscarded' `
    'Runtime collision suppression release results must report stale lease discards.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp' 'havok_runtime::snapshotBody' `
    'Runtime collision suppression must snapshot body identity before acquiring or releasing leases.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp' 'bodyIdentityMatches' `
    'Runtime collision suppression must compare body identity before reusing a body-id entry.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp' 'stale lease discarded before acquire' `
    'Acquire must discard stale leases before applying suppression to a reused body id.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp' 'stale lease discarded on unreadable release' `
    'Unreadable release must discard stale leases instead of preserving body-id-only entries.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp' 'stale lease discarded on identity mismatch' `
    'Release must discard stale leases when the body id now resolves to a different native body.'

if ($failures.Count -gt 0) {
    Write-Host 'Collision suppression registry source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Collision suppression registry source boundary passed.'

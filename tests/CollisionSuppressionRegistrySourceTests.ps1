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
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.h' 'RuntimeSuppressionResult refresh\(' `
    'Runtime collision suppression must expose an identity-safe lease refresh operation.'
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.h' 'leaseIdentityValid' `
    'Successful runtime acquisition must expose the exact leased body identity to physics-step consumers.'
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
Require-Text 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp' 'stale lease discarded before refresh' `
    'Refresh must discard stale leases before writing to a reused body id.'

$registrySource = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/collision/CollisionSuppressionRegistry.cpp')
$refreshMatch = [regex]::Match(
    $registrySource,
    '(?s)RuntimeSuppressionResult CollisionSuppressionRegistry::refresh\(.*?(?=RuntimeSuppressionResult CollisionSuppressionRegistry::release\()')
if (-not $refreshMatch.Success) {
    $failures.Add('The runtime refresh implementation must remain independently inspectable from release.')
} else {
    $refreshBody = $refreshMatch.Value
    $identityCheckIndex = $refreshBody.IndexOf('bodyIdentityMatches')
    $filterWriteIndex = $refreshBody.IndexOf('body_collision::setFilterInfo')
    if ($identityCheckIndex -lt 0 -or $filterWriteIndex -lt 0 -or $identityCheckIndex -gt $filterWriteIndex) {
        $failures.Add('Runtime refresh must verify native body identity before reasserting the collision filter.')
    }
    if ($refreshBody -match '_entries\.push_back|captureBodyIdentity') {
        $failures.Add('Runtime refresh must never acquire a lease for a replacement body generation.')
    }
}

$interactionSource = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/core/PhysicsInteraction.cpp')
$nativeRefreshMatch = [regex]::Match(
    $interactionSource,
    '(?s)void PhysicsInteraction::refreshNativePlayerCollisionSuppression\(.*?(?=void PhysicsInteraction::refreshNativePlayerCollisionSuppressionFromPhysicsSubstep\()')
if (-not $nativeRefreshMatch.Success) {
    $failures.Add('Native-player collision refresh must remain independently inspectable from its update scan.')
} else {
    $nativeRefreshBody = $nativeRefreshMatch.Value
    if ($nativeRefreshBody -notmatch 'globalCollisionSuppressionRegistry\(\)\.refresh\(') {
        $failures.Add('Native-player collision refresh must delegate reassertion to the identity-safe registry.')
    }
    if ($nativeRefreshBody -match 'body_collision::tryReadFilterInfo|body_collision::setFilterInfo|kSuppressionNoCollideBit') {
        $failures.Add('Native-player collision refresh must not perform body-id-only collision filter access.')
    }
    if ($nativeRefreshBody -notmatch 'staleLeaseDiscarded') {
        $failures.Add('Native-player collision refresh must evict stale cached body ids immediately.')
    }
}

$physicsRefreshMatch = [regex]::Match(
    $interactionSource,
    '(?s)void PhysicsInteraction::refreshNativePlayerCollisionSuppressionFromPhysicsSubstep\(.*?(?=void PhysicsInteraction::updateNativePlayerCollisionSuppression\()')
if (-not $physicsRefreshMatch.Success) {
    $failures.Add('Physics-step native-player refresh must remain independently inspectable from game-frame mutation.')
} else {
    $physicsRefreshBody = $physicsRefreshMatch.Value
    $snapshotIndex = $physicsRefreshBody.IndexOf('havok_runtime::snapshotBody')
    $identityCheckIndex = $physicsRefreshBody.IndexOf('snapshot.motionIndex != leasedBody.motionIndex')
    $filterWriteIndex = $physicsRefreshBody.IndexOf('body_collision::setFilterInfo')
    if ($snapshotIndex -lt 0 -or $identityCheckIndex -lt 0 -or $filterWriteIndex -lt 0 -or
        $snapshotIndex -gt $identityCheckIndex -or $identityCheckIndex -gt $filterWriteIndex) {
        $failures.Add('Physics-step refresh must snapshot and verify the leased native identity before any filter write.')
    }
    if ($physicsRefreshBody -notmatch 'snapshot\.collisionObject != leasedBody\.collisionObject' -or
        $physicsRefreshBody -notmatch 'snapshot\.ownerNode != leasedBody\.ownerNode') {
        $failures.Add('Physics-step refresh must compare motion, collision-object, and owner-node identity.')
    }
    if ($physicsRefreshBody -match 'globalCollisionSuppressionRegistry|staleLeaseDiscarded') {
        $failures.Add('Physics-step refresh must remain read-only with respect to the game-thread suppression registry.')
    }
    if ($physicsRefreshBody -match '_nativePlayerCollisionSuppressedBodies(?:\[[^\]]+\])?\s*=(?!=)|_nativePlayerCollisionSuppressedBodyCount\s*=(?!=)') {
        $failures.Add('Physics-step refresh must not mutate the cache published by the game frame.')
    }
}

$restoreMatch = [regex]::Match(
    $interactionSource,
    '(?s)void PhysicsInteraction::restoreNativePlayerCollisionSuppression\(.*?(?=void PhysicsInteraction::refreshNativePlayerCollisionSuppression\()')
if (-not $restoreMatch.Success -or $restoreMatch.Value -notmatch 'callbackGate\(\)\.pauseForMutation\(\)') {
    $failures.Add('Native-player suppression restore must quiesce the physics callback cache reader.')
}

$updateMatch = [regex]::Match(
    $interactionSource,
    '(?s)void PhysicsInteraction::updateNativePlayerCollisionSuppression\(.*?(?=void PhysicsInteraction::onGeneratedColliderPhysicsSubstep\()')
if (-not $updateMatch.Success -or $updateMatch.Value -notmatch 'callbackGate\(\)\.pauseForMutation\(\)') {
    $failures.Add('Native-player suppression update must quiesce the physics callback cache reader.')
}

if ($failures.Count -gt 0) {
    Write-Host 'Collision suppression registry source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Collision suppression registry source boundary passed.'

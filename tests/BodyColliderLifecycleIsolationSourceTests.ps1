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

$physicsInteractionPath = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$physicsInteractionText = Get-Content -Raw -LiteralPath (Join-Path $Root $physicsInteractionPath)

$generatedBodiesStart = $physicsInteractionText.IndexOf('bool PhysicsInteraction::generatedBodiesExistForConfig() const')
$generatedBodiesEnd = if ($generatedBodiesStart -ge 0) {
    $physicsInteractionText.IndexOf('bool PhysicsInteraction::generatedBodiesMatchLifecycle', $generatedBodiesStart)
} else {
    -1
}
if ($generatedBodiesStart -lt 0 -or $generatedBodiesEnd -lt 0) {
    $failures.Add('Generated-body lifecycle helper boundary could not be located.')
} else {
    $generatedBodiesText = $physicsInteractionText.Substring($generatedBodiesStart, $generatedBodiesEnd - $generatedBodiesStart)
    if ($generatedBodiesText -match '_bodyBoneColliders\.hasBodies\(\)|rockBodyBoneCollidersEnabled') {
        $failures.Add('Core generated-body lifecycle validity must not depend on optional body bone collider availability.')
    }
    if ($generatedBodiesText -notmatch '_rightHand\.hasCollisionBody\(\)\s*&&\s*_leftHand\.hasCollisionBody\(\)') {
        $failures.Add('Core generated-body lifecycle validity must be anchored to both generated hand collision bodies.')
    }
}

Require-Text $physicsInteractionPath `
    'Generated body lifecycle rebuild continuing without body bone colliders; runtime update will retry' `
    'Lifecycle rebuild must continue with valid hand bodies when optional body bone collider creation fails.'

$rebuildStart = $physicsInteractionText.IndexOf('bool PhysicsInteraction::rebuildGeneratedBodiesForLifecycle')
$rebuildEnd = if ($rebuildStart -ge 0) {
    $physicsInteractionText.IndexOf('void PhysicsInteraction::observeLifecycleFrame', $rebuildStart)
} else {
    -1
}
if ($rebuildStart -lt 0 -or $rebuildEnd -lt 0) {
    $failures.Add('Generated-body rebuild helper boundary could not be located.')
} else {
    $rebuildText = $physicsInteractionText.Substring($rebuildStart, $rebuildEnd - $rebuildStart)
    $bodyCreateMatch = [regex]::Match(
        $rebuildText,
        '(?s)if\s*\(\s*g_rockConfig\.rockBodyBoneCollidersEnabled\s*&&\s*!createBodyBoneCollisions\(hknp,\s*bhk\)\s*\)\s*\{(?<block>.*?)\n\s*\}'
    )
    if (!$bodyCreateMatch.Success) {
        $failures.Add('Lifecycle rebuild must explicitly isolate optional body bone collider creation failure.')
    } else {
        $bodyCreateFailureBlock = $bodyCreateMatch.Groups['block'].Value
        if ($bodyCreateFailureBlock -match 'markGeneratedBodiesInvalidated\(\)|return\s+false') {
            $failures.Add('Optional body bone collider creation failure must not invalidate or fail the core generated-body lifecycle rebuild.')
        }
        if ($bodyCreateFailureBlock -notmatch '_bodyBoneColliderCreateRetryFrames\s*=\s*120') {
            $failures.Add('Optional body bone collider creation failure must schedule the existing body collider retry path.')
        }
    }
}

Require-Text $physicsInteractionPath `
    'Body bone colliders were not available during init; runtime update will retry' `
    'Initial body bone collider creation failure must remain non-fatal during ROCK initialization.'

if ($failures.Count -gt 0) {
    Write-Host 'BodyColliderLifecycleIsolationSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'BodyColliderLifecycleIsolationSourceTests passed.' -ForegroundColor Green

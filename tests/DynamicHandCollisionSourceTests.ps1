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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Require-OrderedText {
    param(
        [string]$Path,
        [string[]]$Patterns,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $remaining = $text.Substring($offset)
        $match = [regex]::Match($remaining, $pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

# Stage A of the soft-collision overhaul: dynamic velocity-driven hand proxies.
# See Docs/ROCK/docs/2026-07-13-soft-collision-overhaul-roadmap.md §8.

# Live-world teardown must use deferred retirement (2026-07-08 UAF lesson).
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'body\.retireDeferred\(' `
    'Dynamic hand twin teardown must go through retireDeferred.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'body\.destroy\(' `
    'Dynamic hand twins must never destroy() a live-world body immediately.'

# The twins must mirror the production collider conventions: frames and shapes
# come from the HandBoneColliderSet publication, never re-derived geometry.
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'hand\.dynamicTwinTargets\(\)' `
    'Dynamic hand twins must consume the HandBoneColliderSet role-frame publication.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'hand\.buildDynamicTwinShape\(' `
    'Dynamic hand twins must build shapes through the shared collider hull construction.'
Require-OrderedText 'src/physics-interaction/hand/HandBoneColliderSet.cpp' @(
    'publishTwinSlot\(twinTargets\.palm',
    'HandFingerSegment::Tip',
    '_dynamicTwinTargets = twinTargets;'
) 'HandBoneColliderSet must publish palm anchor and fingertip twin frames every update.'

# Render-follow pipeline: combine per-body deviations, smooth (rest twitch), gate.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'combineTwinDeviations\(',
    'smoothAppliedDeviation\(',
    'applyExternalHandWorldTransform\('
) 'Dynamic hand render-follow must combine, smooth, then apply the deviation.'

# Deviation must be sampled POST-SOLVE against the same substep's commanded
# target: pre-collide sampling leaks one substep of tracking lag into the
# rendered hand (locomotion drag + at-rest refresh twitch).
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'void DynamicHandCollisionRuntime::samplePostSolveDeviations\(',
    'tryResolveLiveBodyWorldTransform\(',
    'commandedTargetGame'
) 'Dynamic hand deviation must be sampled post-solve against the commanded target.'
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    'void PhysicsInteraction::observeCustomGrabAuthorityAfterSolve\(',
    '_dynamicHandCollision\.samplePostSolveDeviations\(world\);'
) 'Dynamic hand post-solve sampling must run in the after-solve physics phase.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'liveBodyGamePosition\.x - result\.targetGamePosition' `
    'Dynamic hand deviation must not be derived from the pre-collide drive telemetry.'

# Contact press cap: an established contact must lean, not slam. The drive
# clamps only the velocity component along the press direction, after the
# hard-keyframe computation; the caller feeds the direction from the last
# post-solve deviation.
Require-OrderedText 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' @(
    'kFunc_ComputeHardKeyFrame',
    'hasContactPressDirection',
    'contactPressMaxVelocityHavok',
    'setVelocity\('
) 'Dynamic drive must clamp the contact press velocity after the hard-keyframe computation.'
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'lastPostSolveDeviationValid',
    'mode\.hasContactPressDirection = true;',
    'driveGeneratedKeyframedBody\('
) 'Dynamic hand flush must arm the press cap from the last post-solve deviation.'

# The twins get their own visualization flag, independent of the keyframed
# collider debug draws.
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'if \(drawDynamicHandColliders\)' `
    'Dynamic hand twins must draw behind their own bDebugDrawDynamicHandColliders flag.'

# The drive keeps chasing the wand while another system owns the hand pose:
# target queueing must happen BEFORE the ownership gate.
Require-OrderedText 'src/physics-interaction/hand/DynamicHandCollision.cpp' @(
    'queueGeneratedKeyframedBodyTarget\(',
    'ownedByStrongerSystem'
) 'Dynamic hand drive must queue the wand target before evaluating visual ownership gates.'

# Mutual exclusion: while the dynamic drive is enabled, the soft-contact
# runtime is reset once and skipped so exactly one system owns hand visuals.
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    '_dynamicHandCollision\.updateFrame\(',
    'if \(g_rockConfig\.rockHandCollisionDynamicDrive\)',
    '_softContactRuntime\.reset\(\);',
    '_softContactRuntime\.update\('
) 'Dynamic hand drive and soft contact must stay mutually exclusive visual authorities.'

# Existing keyframed callers must keep their behavior: the drive-mode parameter
# stays defaulted to keyframe placement.
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' `
    'const GeneratedBodyDriveMode& mode = \{\}' `
    'driveGeneratedKeyframedBody must default to keyframe placement for existing callers.'

# The proxy layer must remain world-surface-only and applied with the other
# generated rows.
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'ROCK_LAYER_DYNAMIC_HAND_PROXY = 48',
    'buildRockDynamicHandProxyExpectedMask\(\)',
    'isWorldSurfaceLayer\(layer\)'
) 'Dynamic hand proxy layer must be 48 with a world-surface-only mask.'
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'inline void applyRockGeneratedLayerPolicies\(',
    'applyRockDynamicHandProxyLayerPolicy\(matrix\);'
) 'Dynamic hand proxy layer row must be applied with the other generated layer rows.'

# The proxy drive flush must run beside the other generated collider flushes.
Require-OrderedText 'src/physics-interaction/core/PhysicsInteraction.cpp' @(
    '_weaponCollision\.flushPendingPhysicsDrive\(world, timing\);',
    '_dynamicHandCollision\.flushPendingPhysicsDrive\(world, timing\);'
) 'Dynamic hand proxy drive must flush in the generated collider physics substep.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic hand collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic hand collision source boundary passed.'

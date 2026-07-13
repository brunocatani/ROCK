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
    'proxyBody\.retireDeferred\(' `
    'Dynamic hand proxy teardown must go through retireDeferred.'
Reject-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'proxyBody\.destroy\(' `
    'Dynamic hand proxy must never destroy() a live-world body immediately.'

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

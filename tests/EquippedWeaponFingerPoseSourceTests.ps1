param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath) -or (Get-Content -Raw -LiteralPath $fullPath) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Require-OrderedText {
    param([string]$Path, [string[]]$Patterns, [string]$Message)

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $match = [regex]::Match(
            $text.Substring($offset),
            $pattern,
            [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)

    $fullPath = Join-Path $Root $Path
    if ((Test-Path -LiteralPath $fullPath) -and (Get-Content -Raw -LiteralPath $fullPath) -match $Pattern) {
        $failures.Add($Message)
    }
}

# Grip-point selection inspects the contacted part exactly. All five fingers
# share one weapon-local candidate pool. Per-finger baked sweep lanes reserve
# bounded coverage before deterministic global fill, then one shared BVH solve
# remains capped at 2,048 triangles.
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'constexpr std::size_t kMaxFingerPoseCandidateTriangles\s*=\s*2048' `
    'The aggregate equipped-hand triangle ceiling must remain 2,048.'

# The support hand is minimally surface-aligned and seated while the weapon
# remains unchanged. Move the evidence by the full inverse hand-seat relation
# for the one-shot solve so the live skeleton observes that exact relation.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'solveFrozenMeshFingerPoseBase\(',
    'buildFromLocalTriangles\(boundedLocalTriangles\)',
    'resolveCommandedOpenDirectionsWorld\(',
    'FingerPoseMeshRelation::AlreadyAtCommandedSeat',
    'solveFrozenMeshFingerPose\(',
    'solveFrozenMeshFingerPoseBase\(',
    'useThumbIndexCurveOnlyPose\(result\.pose\)',
    'refineGrabFingerPoseWithPadProbes\(',
    'captureSurfaceAimObjectLocal\(result\.pose, frozenMeshWorldTransform\)'
) 'Regular loose grabs must layer their thumb/index and pad policy over the shared indexed frozen base.'
Require-OrderedText 'src/physics-interaction/hand/RootFlattenedFingerSkeletonRuntime.cpp' @(
    'buildFingerSkeletonSnapshot\(',
    'boneSnapshot\.valid',
    'findSnapshotBone\(\s*boneSnapshot',
    'resolveLiveFingerSkeletonSnapshot\(',
    'buildFingerSkeletonSnapshot\('
) 'Compact finger landmarks must be derivable from the same direct bone snapshot used by exact-local posing.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'resolveFingerTransforms\(',
    'const DirectSkeletonBoneSnapshot& snapshot',
    'buildSurfaceCorrectedLocalTransforms\(',
    'const DirectSkeletonBoneSnapshot\* capturedFingerSnapshot',
    'resolveFingerTransforms\(\s*\*capturedFingerSnapshot'
) 'Exact-local finger correction must accept the capture transaction snapshot instead of forcing a second scene read.'

# Physical equipped grips accept either five direct lane contacts or one local
# thumb-index/thumb-pinky containment witness with a stable opposition pose.
# All other incomplete solves publish one fully closed hand.
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'kCompleteFingerContactMask\s*=\s*0x1F[\s\S]*contactValidMask[\s\S]*hasCompleteFingerContactEvidence\(' `
    'The solver must expose an explicit five-lane completion witness.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'findLocalOppositionPocketEvidence\(',
    'directEndpointMask\s*==\s*0',
    'rayTriangleIntersection\(',
    'probeCapsuleTriangleIntersection\(',
    'gripDistance\s*<=\s*maxGripDistance'
) 'Weapon opposition recovery must require direct endpoint evidence and a local pad-to-pad mesh crossing.'

# Keep the hitch observable without adding hot-loop logs or timers that affect
# gameplay: one capture scope plus bounded work counters are profiler-only.
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'EquippedWeaponFingerPoseCapture[\s\S]*EquippedWeaponFingerPoseSourceTriangles[\s\S]*EquippedWeaponFingerPoseSelectedTriangles[\s\S]*EquippedWeaponFingerPoseSpatialNodeVisits[\s\S]*EquippedWeaponFingerPoseTriangleTests' `
    'Equipped finger-pose capture must expose time, source size, bounded size, and exact BVH work.'

if ($failures.Count -gt 0) {
    Write-Host 'Equipped weapon finger pose source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Equipped weapon finger pose source boundary passed.'

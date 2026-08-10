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

# The contacted generated body owns the grip point. Finger posing additionally
# borrows deduplicated frame-scoped views from the whole equipped weapon.
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' `
    'struct SupportGripEvidenceView[\s\S]*std::span<const TriangleData> localTriangles[\s\S]*RE::NiTransform localToWorld[\s\S]*sourceGroupId[\s\S]*bodyId[\s\S]*weaponGenerationKey[\s\S]*findSupportGripEvidenceViews\(' `
    'Equipped finger posing must consume generation-tagged, source-identifiable local triangle views.'
Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'tryBuildSupportGripEvidenceView\(',
    'tryResolveDescendantWorldTransform\(',
    'generatedSourceLocalTrianglesGame',
    'outView\.localTriangles = std::span<const TriangleData>',
    'outView\.localToWorld = localToWorld',
    'outView\.sourceGroupId',
    'outView\.weaponGenerationKey = getCurrentWeaponGenerationKey\(\)'
) 'Support grip evidence views must stay local, transformed, source-identifiable, and generation validated.'
Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'tryGetSupportGripEvidenceView\(',
    'instance\.body\.getBodyId\(\)\.value != bodyId',
    'tryBuildSupportGripEvidenceView\('
) 'Grip-point evidence must remain tied to the contacted body.'
Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'findSupportGripEvidenceViews\(',
    'seenSourceGroups',
    'tryBuildSupportGripEvidenceView\(',
    'std::find\(',
    'outViews\[viewCount\+\+\] = view'
) 'Finger evidence must enumerate and deduplicate equipped-weapon render sources.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' `
    'tryBuildSupportGripEvidenceTriangles' `
    'The retired world-triangle copy API must not return.'

# Grip-point selection inspects the contacted part exactly. All five fingers
# share one weapon-local candidate pool. Per-finger baked sweep lanes reserve
# bounded coverage before deterministic global fill, then one shared BVH solve
# remains capped at 2,048 triangles.
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'tryGetSupportGripEvidenceView\(decision\.bodyId, weaponNode, evidenceView\)',
    'TransformedSupportGripTriangleView worldEvidence',
    'findClosestGrabPoint\('
) 'Equipped physical grip-point selection must remain tied to the contacted part.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'resolveLiveFingerSkeletonSnapshot\(',
    'findSupportGripEvidenceViews\(',
    'selectNearestSupportGripFingerTriangles\(',
    'compositeEvidenceViews',
    'fingerReferenceSet',
    'grab_finger_pose_runtime::\s*kMaxFingerPoseCandidateTriangles',
    'solveFrozenMeshFingerPose\(',
    'fingerScratch\.localTriangles',
    'fingerScratch\.spatialIndex'
) 'Equipped physical grips must select one bounded weapon-wide pool against the seated hand and solve it through one shared BVH.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'if \(g_rockConfig\.rockGrabMeshFingerPoseEnabled\)[\s\S]{0,2500}selectNearestSupportGripFingerTriangles\(' `
    'Disabling mesh finger posing must skip composite candidate ranking while preserving contacted-part grip selection.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'deterministicOrdinal',
    'localReferences',
    'sourceToWeapon',
    'laneMinimumDistanceSquared',
    'retainNearest\(',
    'laneCursors',
    'kSupportGripGlobalRankingIndex'
) 'Composite evidence must use deterministic per-lane bounded ranking and global fill in one weapon-local frame.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'makeBakedCalibratedFingerCurve<',
    'kSweepSamples\s*=\s*7',
    'rotateAroundUnitAxis\(',
    'appendLanePoint\(',
    'selectNearestSupportGripFingerTriangles\('
) 'Each finger lane must rank geometry against sampled calibrated closing-arc coverage.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'constexpr std::size_t kMaxFingerPoseCandidateTriangles\s*=\s*2048' `
    'The aggregate equipped-hand triangle ceiling must remain 2,048.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'solveGrabFingerPoseFromTriangles\(' `
    'Equipped grips must not bypass the shared indexed frozen-mesh boundary.'
$twoHandedGripText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp')
$equippedSharedSolveCount = [regex]::Matches($twoHandedGripText, 'solveFrozenMeshFingerPose\(').Count
if ($equippedSharedSolveCount -ne 1) {
    $failures.Add("Equipped weapon finger posing must have exactly one shared solve site at grip capture; found $equippedSharedSolveCount.")
}

# The support hand is minimally surface-aligned and seated while the weapon
# remains unchanged. Move the evidence by the full inverse hand-seat relation
# for the one-shot solve so the live skeleton observes that exact relation.
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'alignHandFrameToGripSurface<',
    'grip\.surfaceSeatRotationRadians',
    'virtualizeMeshForSeatedHand\(',
    'virtualizeWorldPointForSeatedHand\(',
    'solveFrozenMeshFingerPose\('
) 'Equipped finger solving must evaluate the final bounded surface-seated hand/weapon relation up front.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'solveFrozenMeshFingerPose\(',
    'buildFromLocalTriangles\(boundedLocalTriangles\)',
    'resolveCommandedOpenDirectionsWorld\(',
    'FingerPoseMeshRelation::AlreadyAtCommandedSeat',
    'useThumbIndexCurveOnlyPose\(result\.pose\)',
    'refineGrabFingerPoseWithPadProbes\(',
    'captureSurfaceAimObjectLocal\(result\.pose, frozenMeshWorldTransform\)'
) 'Loose and equipped regular grips must share calibrated anchors, BVH queries, thumb/index policy, pad refinement, and local surface capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'solveFrozenMeshFingerPose\(' `
    'Regular loose grabs must use the same shared frozen-mesh solver boundary.'

# Physical equipped grips accept either five direct lane contacts or one local
# thumb-index/thumb-pinky containment witness with a stable opposition pose.
# All other incomplete solves publish one fully closed hand.
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'BARREL_WRAP_POSE|HANDGUARD_CLAMP_POSE|FOREGRIP_POSE|PUMP_GRIP_POSE|MAGWELL_HOLD_POSE|RECEIVER_SUPPORT_POSE|poseValuesForGrip' `
    'Retired six-mode canned weapon poses must stay removed.'
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
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'hasCompleteFingerContactEvidence\(',
    'findLocalOppositionPocketEvidence\(',
    'applyStableWeaponOppositionPose\(',
    'completeDirectFingerEvidence\s*\|\|',
    'SupportGripPoseFallback::FullyClosed'
) 'Only complete direct evidence or a validated stable opposition pocket may avoid the fully closed fallback.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'hasCompleteFingerContactEvidence\(',
    'SupportGripPoseFallback::FullyClosed',
    'fallbackValue\s*=\s*0\.0f',
    'whole-hand-closed'
) 'Any incomplete equipped mesh solve must discard partial curls and publish one fully closed hand.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'meshFingerPose = frozenSolve\.pose',
    'hasCompleteFingerContactEvidence\(',
    'meshFingerPosePtr = &meshFingerPose',
    'buildSurfaceContactSplayValues\(',
    'SupportGripPoseFallback::FullyClosed',
    'grip\.active = true'
) 'Equipped physical grips must cache the solved pose before activating hand authority.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'resolveSurfaceContactSplayValues\(' `
    'Equipped grip capture must reuse the solver snapshot instead of traversing the live skeleton again for splay.'

# Keep the hitch observable without adding hot-loop logs or timers that affect
# gameplay: one capture scope plus bounded work counters are profiler-only.
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'EquippedWeaponFingerPoseCapture[\s\S]*EquippedWeaponFingerPoseSourceTriangles[\s\S]*EquippedWeaponFingerPoseSelectedTriangles[\s\S]*EquippedWeaponFingerPoseSpatialNodeVisits[\s\S]*EquippedWeaponFingerPoseTriangleTests' `
    'Equipped finger-pose capture must expose time, source size, bounded size, and exact BVH work.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'ScopedTimer fingerPoseCaptureTimer\(performance_profiler::Scope::EquippedWeaponFingerPoseCapture\)' `
    'The equipped finger-pose hitch boundary must have a dedicated profiler scope.'

if ($failures.Count -gt 0) {
    Write-Host 'Equipped weapon finger pose source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Equipped weapon finger pose source boundary passed.'

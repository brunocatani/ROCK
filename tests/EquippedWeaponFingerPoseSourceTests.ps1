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

# The contacted generated body owns the evidence. WeaponCollision lends a
# frame-scoped local view and transform; it must not rebuild/copy every cached
# triangle into a temporary world-space vector at grip time.
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' `
    'struct SupportGripEvidenceView[\s\S]*std::span<const TriangleData> localTriangles[\s\S]*RE::NiTransform localToWorld[\s\S]*weaponGenerationKey' `
    'Equipped finger posing must consume a generation-tagged, part-local triangle view.'
Require-OrderedText 'src/physics-interaction/weapon/WeaponCollision.cpp' @(
    'tryGetSupportGripEvidenceView\(',
    'instance\.body\.getBodyId\(\)\.value != bodyId',
    'generatedSourceLocalTrianglesGame',
    'outView\.localTriangles = std::span<const TriangleData>',
    'outView\.localToWorld = driveRoot->world',
    'outView\.weaponGenerationKey = getCurrentWeaponGenerationKey\(\)'
) 'Support grip evidence must stay body-specific, local, transformed, and generation validated.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' `
    'tryBuildSupportGripEvidenceTriangles' `
    'The retired world-triangle copy API must not return.'

# Grip-point selection may inspect the contacted part exactly, but all five
# fingers share one nearest-surface candidate pool capped at 2,048. The one BVH
# is then shared by the full hand solve; there is no per-finger cap multiplication.
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'tryGetSupportGripEvidenceView\(decision\.bodyId, weaponNode, evidenceView\)',
    'TransformedSupportGripTriangleView worldEvidence',
    'findClosestGrabPoint\(',
    'cachedTrianglesFound && g_rockConfig\.rockGrabMeshFingerPoseEnabled',
    'selectNearestSupportGripFingerTriangles\(',
    'evidenceView\.localTriangles',
    'grab_finger_pose_runtime::kMaxFingerPoseCandidateTriangles',
    'solveFrozenMeshFingerPose\(',
    'fingerScratch\.localTriangles',
    'fingerScratch\.spatialIndex'
) 'Equipped physical grips must select one bounded part-local pool and solve it through one shared BVH.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'cachedTrianglesFound && g_rockConfig\.rockGrabMeshFingerPoseEnabled[\s\S]{0,400}selectNearestSupportGripFingerTriangles\(' `
    'Disabling mesh finger posing must also skip bounded-pool ranking while preserving exact grip-point selection.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'if \(sourceTriangles\.size\(\) <= boundedLimit\)',
    'outTriangles\.reserve\(sourceTriangles\.size\(\)\)',
    'outTriangles\.push_back\(triangle\)',
    'return;',
    'rankingScratch\.reserve\(boundedLimit\)'
) 'Under-cap contacted parts must bypass heap ranking while oversized parts retain nearest-surface selection.'
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

# The support hand is translated onto the selected point after capture. Move
# the evidence by the inverse translation for the one-shot solve so the live
# skeleton observes the exact final hand/weapon relation before publication.
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'alignHandFrameToGripPoint\(handTransform, palmPos, gripWorldPoint\)',
    'virtualizeMeshForTranslatedHandSeat\(',
    'virtualizeGripPointForTranslatedHandSeat\(',
    'solveFrozenMeshFingerPose\('
) 'Equipped finger solving must evaluate the final translated hand/weapon relation up front.'
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

# Physical equipped grips cache the one-shot result. Semantic part categories
# remain metadata only and may never select one of the retired canned poses.
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'BARREL_WRAP_POSE|HANDGUARD_CLAMP_POSE|FOREGRIP_POSE|PUMP_GRIP_POSE|MAGWELL_HOLD_POSE|RECEIVER_SUPPORT_POSE|poseValuesForGrip' `
    'Retired six-mode canned weapon poses must stay removed.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'rockSelectedCloseFingerAnimValue[\s\S]{0,500}expandFingerCurlsToJointValues\(fallbackCurls\)' `
    'An unavailable mesh solve must use the same uniform selected-close fallback as regular grabs.'
Require-OrderedText 'src/physics-interaction/weapon/TwoHandedGrip.cpp' @(
    'meshFingerPose = frozenSolve\.pose',
    'meshFingerPosePtr = &meshFingerPose',
    'buildSurfaceContactSplayValues\(',
    'setSupportGripPose\(isLeft, meshFingerPosePtr, capturedFingerSplayRadiansPtr\)',
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

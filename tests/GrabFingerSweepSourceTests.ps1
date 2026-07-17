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

# Swept-arc grab finger solver: fingers close along their baked hFRIK arcs and
# stop at first volumetric mesh contact. See
# Docs/ROCK/docs/2026-07-13-grab-finger-sweep-and-live-resolve.md.

# The sweep core must reconstruct probe arc rows and delegate exact volumetric
# contact to a query. The vector wrapper retains the plain/test/two-hand route.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'inline FingerCurlValue sweepCalibratedFingerCurveCurlValueWithContactQuery\(',
    'sphereContact\(curve\.center, filterRadius',
    'rotateAroundUnitAxis\(',
    'sphereContact\(rowPositions\[row\], radius'
) 'Swept-arc solver must walk reconstructed arc rows through an exact sphere-contact query.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'inline FingerCurlValue sweepCalibratedFingerCurveCurlValue\(',
    'filterTrianglesNearPoint\(',
    'closestPointOnTriangle\('
) 'The vector sweep wrapper must retain bounded volumetric triangle contact for non-index callers.'

# The runtime curve branch must use the sweep, not the retired plane-slice
# solvers (their gate/lane patch machinery must stay deleted).
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'sweepThumbAwareCalibratedFingerCurveCurlValueWithCurveSolver<RE::NiPoint3>' `
    'Grab finger runtime must route indexed curls through the same thumb-aware swept-arc solver.'
Reject-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'solveThumbAwareCalibratedFingerCurveCurlValue|solveCalibratedFingerCurveCurlValue|solveFingerCurveCurlValue|shouldRunFallbackRayAfterCurveSolve' `
    'Retired plane-slice curve solvers must not come back; the sweep is the only arc solver.'

# The straight-ray fallback exists only for the non-curve (pinch) route: a
# sweep miss is authoritative (nothing lies on the finger arc).
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'if \(useTarget && !solved\.hit && !curveSolverRan\)' `
    'Ray fallback must run only when the swept-arc solver did not.'

# Out-of-reach fingers hold the caller-provided anticipation value (pull
# flight); the -1 sentinel keeps at-hand miss semantics.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'bool outOfReach = false;',
    'solved\.outOfReach && unreachableFingerOpenValue >= 0\.0f'
) 'Swept-arc solver must expose out-of-reach so callers can hold an anticipation pose.'

# Regular grabs solve once against the already-frozen target relation. The
# bounded local mesh is indexed once, commanded open directions anchor the
# calibrated arcs, and surface contacts are stored object-local.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'targetObjectWorld =',
    '_grabFrame\.desiredObjectWorldAtGrab',
    'solveFrozenMeshFingerPose\(',
    'localFingerPoseTriangles',
    'targetObjectWorld',
    '_grabFingerTriangleIndex'
) 'Regular grab commit must solve one object-local endpoint against the frozen target relation.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'solveFrozenMeshFingerPose\(',
    'rebuildBoundedWorldTriangles\(',
    'buildFromLocalTriangles\(boundedLocalTriangles\)',
    'resolveCommandedOpenDirectionsWorld\(',
    'solveGrabFingerPoseFromTriangles\(',
    'FingerPoseMeshRelation::AlreadyAtCommandedSeat',
    'useThumbIndexCurveOnlyPose\(result\.pose\)',
    'refineGrabFingerPoseWithPadProbes\(',
    'captureSurfaceAimObjectLocal\(result\.pose, frozenMeshWorldTransform\)'
) 'The shared frozen-mesh boundary must own the indexed solve, calibrated anchors, refinement, and object-local capture.'

# The index must own a bounded object-local BVH and perform exact closest-point
# tests with a fixed query stack; no allocation or full candidate scan is
# allowed inside each arc-row probe. The deep-inside clearance check is a
# separate, bounded query issued only after an over-open contact candidate.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'class FingerPoseTriangleSpatialIndex',
    'buildFromLocalTriangles\(',
    'querySphereWorld\(',
    'std::array<std::uint32_t, 64> stack',
    'pointAabbDistanceSquared\(',
    'closestPointOnTriangle\(',
    'std::nth_element\('
) 'Finger arc probes must use the bounded object-local BVH with exact leaf tests and no query allocation.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'const bool useSpatialIndex =',
    'result\.candidateTriangleCount = static_cast<int>\(spatialIndex->triangleCount\(\)\)',
    'querySphereWorld\(',
    'queryPointInsideWorld\(',
    'result\.spatialTriangleTestCount = spatialQueryStats\.triangleTests'
) 'The runtime solve must use and report spatial-index work for regular grabs.'

# Acquisition only blends toward that immutable endpoint. It must never rebuild
# mesh triangles or invoke the geometric solver while the object converges.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    '_grabFingerPosePublished\) \{',
    'resolveSurfaceAimObjectLocal\(_grabFingerPose, desiredObjectWorld\)',
    'buildAcquisitionFingerPose\(resolvedTargetPose, acquisitionProgress\)',
    'applyRockGrabHandPose\('
) 'Grab acquisition must blend toward the pre-solved target without geometric re-solves.'

# The commanded zero reconstruction walks authored open-pose chain origins.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'resolveCommandedOpenDirectionsWorld\(',
    'getHandPoseLocalTransformsForPose\(',
    'computeCommandedOpenDirectionsHandLocal\(',
    'localVectorToWorld\(handWorldTransform'
) 'Regular target-space solves must anchor arcs on the authored commanded-open hand model.'

# The live chain chord is rotated by the CURRENT curl; anchoring the arc zero
# on it directly stopped every finger short by that curl (air gap) and made
# held re-solves oscillate. The runtime must de-rotate the chord to the true
# open reference via the baked Tip reach-table inversion, in the baked
# arc-plane sign convention - and the inversion must skip the ambiguous
# over-open rows (the chord shortens again past the authored open pose).
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'estimateCalibratedChainCurlFromChord\(',
    'BakedGrabFingerProbe::Tip',
    'samples\[scanStart\]\.openValue > kMaxFingerOpenValue',
    'chordScale <= reachA && chordScale >= reachB'
) 'Chord curl estimation must invert the baked Tip probe reach table, restricted to the sub-open region.'

# Anchor priority in the solve: a caller-provided COMMANDED open direction
# (zero rendered-finger feedback by construction) wins outright; without one
# the chord inversion covers the four fingers only (the thumb chord shortens
# from opposition/twist and the inversion misreads it).
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'RE::NiPoint3 openDirectionWorld = live\.openDirection;',
    'if \(hasCommandedOpenDirection\) \{',
    'openDirectionWorld = \(\*commandedOpenDirectionsWorld\)\[finger\];',
    '\} else if \(finger != 0\) \{',
    'estimateCalibratedChainCurlFromChord\(',
    '-chordCurl\.chordAngleRadians \* chordCurl\.normalSign'
) 'The runtime solve must prefer commanded open directions and fall back to the chord inversion for the four fingers only.'

# The commanded zero reconstruction must walk the authored open pose's chain
# bone origins - the bake's own zero definition - and never touch rendered
# geometry.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'computeCommandedOpenDirectionsHandLocal\(',
    'composeTransforms\(bone1',
    'composeTransforms\(bone2',
    'bone3\.translate - bone1\.translate'
) 'Commanded open directions must be reconstructed from the authored open-pose chain bone origins.'

# Solved poses retain their exact contact-row rotation for diagnostics.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'contactArcRotationRadians\[finger\] = solved\.distance \* bakedAnchorNormalSign',
    'contactArcRotationValid\[finger\] = 1'
) 'Sweep solves must record the selected contact-row rotation.'

# TouchHeld atomically publishes the same stored endpoint, resolved through
# object-local surface aims. It does not fire the regular solver again.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'finalPoseObjectWorld =',
    'resolveSurfaceAimObjectLocal\(_grabFingerPose, finalPoseObjectWorld\)',
    'applyRockGrabHandPose\(_isLeft,',
    '0\.0f,\s*true,\s*true'
) 'TouchHeld must snap the pre-solved endpoint atomically with local transforms enabled.'

# Pinch is the sole deferred special route and keeps its existing at-touch
# non-curve solve and policy.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'pinch solve deferred until TouchHeld',
    'if \(!_grabFingerPosePublished\) \{',
    'const bool pinchFingerPose = _grabFrame\.seatMode == GrabSeatMode::PinchPocket;',
    'solveGrabFingerPoseFromTriangles\(',
    'applyPinchFingerPosePolicy\(_grabFingerPose'
) 'Pinch must remain the only deferred at-touch finger solve.'

# The former live convergence and held adoption loops are the regression:
# they must stay absent, including their interval/deadline state.
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'anticipationOpenValue|heldResolveMaxValueDelta|heldPoseSmoothingSettled|FINGER-CYCLE ADOPT|FINGER POSE FROZEN|FINGER POSE RESOLVE WINDOW EXPIRED|_grabFingerPoseFrozen|_grabFingerPoseResolveElapsedSeconds|rockGrabFingerPoseUpdateInterval' `
    'Normal grabs must not retain live convergence or held settle re-solve machinery.'

$handGrabText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$handGrabSolveCount = [regex]::Matches($handGrabText, 'solveGrabFingerPoseFromTriangles\(').Count
if ($handGrabSolveCount -ne 2) {
    $failures.Add("HandGrab must retain exactly two direct non-curve solve sites for pinch commit/deferred pinch; found $handGrabSolveCount.")
}

# Additional publish-path pad probes are debug-overlay-only work; capture-time
# target refinement remains one-shot.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'rockDebugShowGrabFingerProbes\) \{',
    'refineGrabFingerPoseWithPadProbes\('
) 'Publish-path pad probes must be gated behind the finger-probe overlay flag.'

# The proximity-scaled pad open bias mutated PUBLISHED values from live pad
# distance AFTER the deadband - the finger-twitch feedback loop. It must not
# come back in any form; over-open is a swept-arc result now.
Reject-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'fingerPadOpenBiasValue|fingerPadThumbOverOpenValue|kThumbOverOpenStartValue|openBiasStrength' `
    'The pad open-bias feedback loop must stay deleted; the sweep owns finger values.'

# Over-open (past the authored open pose) is a first-class sweep capability:
# the caps are config-driven and passed at every solve site.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'inline constexpr float kMaxFingerOpenValue = 1\.0f;',
    'inline constexpr float kMaxOverOpenValue = 2\.0f;'
) 'Open-value ceilings must model the authored open pose and the hFRIK flex ceiling.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'sweepCalibratedFingerCurveCurlValueWithContactQuery\(',
    'float maxOpenValue,',
    'samples\[overOpenStartRow\]\.openValue > clampedMaxOpen',
    'std::clamp\(probe\.samples\[row\]\.openValue, 0\.0f, clampedMaxOpen\)'
) 'The sweep must honor a per-call max-open cap by skipping rows above it.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'float maxOpenValue = kMaxOverOpenValue\)' `
    'The vector sweep wrapper must traverse the full hFRIK flex domain by default.'

# The geometric walk and its diagnostics always begin at the configured
# ceiling. Publishing an over-open contact still requires evidence that the
# authored-open probe is blocked: either its contact sphere touches or its
# center is enclosed by a consistently wound shell. This covers thick objects
# without reviving dorsal-only hyperextension (the skull case, 2026-07-13).
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'sweptProbeStartRow\[probeIndex\] = static_cast<std::uint16_t>\(overOpenStartRow\)',
    'firstContactInRange\(overOpenStartRow, authoredOpenRow\)',
    'authoredOpenTouches = sphereContact\(rowPositions\[authoredOpenRow\], radius',
    'pointInside\(rowPositions\[authoredOpenRow\]\)',
    'keepMostOpen\(bestOverOpen, overOpenContact\)'
) 'The full over-open path must be walked while selection remains gated by authored-open obstruction evidence.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'enum class FingerPoseMeshRelation',
    'CurrentMeshRequiresVirtualSeat',
    'AlreadyAtCommandedSeat',
    'resolveFingerSweepBaseWorld\('
) 'Finger-pose pivots must explicitly distinguish current geometry from already-seated geometry.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'FingerPoseMeshRelation::AlreadyAtCommandedSeat' `
    'Regular target-space grabs must keep their calibrated pivot on the live proximal bone.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    '\.thumbSweepMaxOpenValue\s*=\s*g_rockConfig\.rockGrabThumbSweepMaxOpenValue,[\s\S]{0,160}\.fingerSweepMaxOpenValue\s*=\s*g_rockConfig\.rockGrabFingerSweepMaxOpenValue' `
    'Grab solve sites must pass the config-driven thumb/finger sweep max-open caps.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '\.thumbSweepMaxOpenValue\s*=\s*g_rockConfig\.rockGrabThumbSweepMaxOpenValue,[\s\S]{0,160}\.fingerSweepMaxOpenValue\s*=\s*g_rockConfig\.rockGrabFingerSweepMaxOpenValue' `
    'The two-handed support-hand solve must pass the same sweep max-open caps.'
Require-Text 'tools/generate_grab_finger_calibration.py' `
    'OVER_OPEN_MAX = 2\.0' `
    'The calibration bake must sample the full over-open range for every finger.'
Require-Text 'src/RockConfig.h' `
    'rockGrabFingerSweepMaxOpenValue = 2\.0f' `
    'Normal fingers must default to the same full 2.0 sweep ceiling as the calibration.'
foreach ($path in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $path 'fGrabFingerSweepMaxOpenValue\s*=\s*2\.0' 'Reference INIs must expose the full 2.0 normal-finger sweep by default.'
}

# The presentation grip axis is not pure cross-palm Z (the thumb occupies that
# line): both alignment sites must apply the configurable tilt toward X.
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'std::sin\(gripAxisTiltRadians\), 0\.0f, std::cos\(gripAxisTiltRadians\)' `
    'The pull-flight presentation servo must use the tilted grip axis.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'pocket\.crossPalmWorld \* std::cos\(gripAxisTiltRadians\) \+\s*pocket\.fingerForwardWorld \* std::sin\(gripAxisTiltRadians\)' `
    'The force-grab seat alignment must use the same tilted grip axis.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab finger sweep source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab finger sweep source boundary passed.'

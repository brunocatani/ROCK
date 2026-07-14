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

# The sweep core must reconstruct probe arc rows and test them volumetrically.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'inline FingerCurlValue sweepCalibratedFingerCurveCurlValue\(',
    'filterTrianglesNearPoint\(',
    'rotateAroundUnitAxis\(',
    'closestPointOnTriangle\('
) 'Swept-arc solver must walk reconstructed arc rows with volumetric contact tests.'

# The runtime curve branch must use the sweep, not the retired plane-slice
# solvers (their gate/lane patch machinery must stay deleted).
Require-Text 'src/physics-interaction/grab/GrabFinger.h' `
    'sweepThumbAwareCalibratedFingerCurveCurlValue\(candidateTriangles' `
    'Grab finger runtime must solve curls through the thumb-aware swept-arc solver.'
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

# Acquisition (pull-to-grab / close grab) re-solves fingers LIVE against the
# converging object every frame; pinch pockets keep the confirmed-good blend.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    '!_grabFingerPosePublished\) \{',
    'GrabSeatMode::PinchPocket;',
    'buildAcquisitionFingerPose\(',
    'rebuildFingerPoseWorldTrianglesFromGrabFrame\(_grabFrame, currentNodeWorld\)',
    'anticipationOpenValue,',
    'useThumbIndexCurveOnlyPose\(liveFingerPose\)'
) 'Grab acquisition must live re-solve wrap fingers against the converging object (pinch keeps the blend).'

# The held update interval must re-solve the curls against the live seat, not
# republish the promotion-instant snapshot for the whole hold - anchored on
# the KNOWN adopted contact rotations, never on live-geometry estimation.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'rockGrabFingerPoseUpdateInterval',
    'tryGetGrabDriveObjectWorldTransform\(',
    'heldPinchFingerPose',
    'makeArcAnchorHintsFromPose\(_grabFingerPose\)',
    'solveGrabFingerPoseFromTriangles\(',
    '&heldArcAnchorHints\);'
) 'Held finger pose must re-solve curls at the update interval against the live seat, anchored by the adopted-pose arc hints.'

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

# Anchor priority in the solve: a caller-provided arc-anchor hint (the
# rotation the finger was ADOPTED at) de-rotates exactly and covers the thumb
# and over-open poses; without a hint the inversion covers the four fingers
# only (the thumb chord shortens from opposition/twist and the inversion
# misreads it).
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'RE::NiPoint3 openDirectionWorld = live\.openDirection;',
    'if \(hasArcAnchorHint\) \{',
    '-arcAnchorHints->rotationRadians\[finger\]',
    '\} else if \(finger != 0\) \{',
    'estimateCalibratedChainCurlFromChord\(',
    '-chordCurl\.chordAngleRadians \* chordCurl\.normalSign'
) 'The runtime solve must prefer adopted arc-anchor hints and fall back to the chord inversion for the four fingers only.'

# Adopted poses must record the contact rotation the hint mechanism feeds
# back (palm-plane sweep contacts only).
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'contactArcRotationRadians\[finger\] = solved\.distance \* bakedAnchorNormalSign',
    'contactArcRotationValid\[finger\] = 1'
) 'Sweep solves must record the adopted contact-row rotation for held re-solve anchoring.'

# Held re-solves within noise of the current pose must not churn new FRIK
# targets every interval (finger micro-twitch).
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'heldResolveMaxValueDelta',
    'liveFingerPose\.solved && heldResolveMaxValueDelta > 0\.02f'
) 'Held finger re-solve must apply a publish deadband.'

# The interval re-solve exists ONLY to track the settling seat. Once
# consecutive re-solves land inside the deadband and smoothing has reached
# its target, the pose FREEZES: no more mesh rebuilds, solves, pad probes,
# or publishes for the rest of the hold. A converged grip must never re-pose
# because the held object was pushed or physically deviated - and the
# per-interval O(triangles) work must stop (FPS on high-poly weapon/part
# meshes).
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    '!_grabFingerPoseFrozen\) \{',
    'heldResolveAdopted',
    'kGrabFingerPoseFreezeQuietResolves = 3;',
    '_grabFingerPoseFrozen = true;',
    'FINGER POSE FROZEN'
) 'Held finger pose must converge-then-freeze; a converged grip never re-poses or re-scans the mesh.'
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'captureSurfaceAimObjectLocal\(_grabFingerPose, currentNodeWorld\);',
    '_grabFingerPoseQuietResolves = 0;',
    '_grabFingerPoseFrozen = false;'
) 'Pose re-captures must unfreeze the held finger pose.'

# Solving against a chain still blending toward the last adopted target reads
# a LAGGING chord: the anchor de-rotation is off by the lag and successive
# adoptions ping-pong (open/close twitch). The held re-solve must be gated on
# the applied joints having reached the commanded pose; the publish keeps
# advancing the smoothing every interval regardless.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'bool heldPoseSmoothingSettled = true;',
    'if \(heldPoseSmoothingSettled\) \{',
    'rebuildFingerPoseWorldTrianglesFromGrabFrame\(_grabFrame, currentNodeWorld\)',
    'solveGrabFingerPoseFromTriangles\(',
    '\} // heldPoseSmoothingSettled',
    'applyRockGrabHandPose\('
) 'Held re-solves must wait for the pose smoothing to settle; publishes continue regardless.'

# A thumb without an exact arc-anchor hint must keep its previous pose: the
# raw-chord anchor rotates with the thumb's own curl, so an unhinted re-solve
# period-2 cycles between two poses (the thumb open/close twitch).
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    '&heldArcAnchorHints\);',
    'heldArcAnchorHints\.valid\[0\] == 0',
    'keepThumbPoseFromPrevious\(liveFingerPose, _grabFingerPose\);'
) 'Unhinted held thumb re-solves must keep the previous thumb pose.'

# Pad probes on the publish paths are debug-overlay-only work (target
# refinement is capture-only, open bias is deleted) and iterate every world
# triangle per finger - they must not run when the overlay is off.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'rockDebugShowGrabFingerProbes\) \{',
    'refineGrabFingerPoseWithPadProbes\('
) 'Publish-path pad probes must be gated behind the finger-probe overlay flag.'

# Convergence has a wall-time budget: a grab still adopting past the resolve
# window is cycling (pose A re-solves to pose B and back), not settling. It
# must freeze regardless and log the expiry loudly.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    '_grabFingerPoseResolveElapsedSeconds \+=',
    'rockGrabFingerPoseResolveWindowSeconds\) \{',
    'FINGER POSE RESOLVE WINDOW EXPIRED'
) 'Held finger re-solves must freeze at the resolve-window deadline (anti-livelock).'

# Every held adoption must be traceable: the FINGER-CYCLE log carries anchor
# hints, contact rotations, and the hand-relative object position so an
# infinite cycle names its driver (chain feedback vs object motion) offline.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'rockDebugGrabFingerPoseLogging\) \{',
    'FINGER-CYCLE ADOPT',
    'relPos=',
    '\+\+_grabFingerPoseAdoptionCount;'
) 'Held pose adoptions must be traceable via the FINGER-CYCLE debug log.'

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
    'float maxOpenValue = kMaxFingerOpenValue\)',
    'samples\[startRow\]\.openValue > clampedMaxOpen',
    'std::clamp\(probe\.samples\[row\]\.openValue, 0\.0f, clampedMaxOpen\)'
) 'The sweep must honor a per-call max-open cap by skipping rows above it.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'rockGrabThumbSweepMaxOpenValue,\s*g_rockConfig\.rockGrabFingerSweepMaxOpenValue' `
    'Grab solve sites must pass the config-driven thumb/finger sweep max-open caps.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'rockGrabThumbSweepMaxOpenValue, g_rockConfig\.rockGrabFingerSweepMaxOpenValue' `
    'The two-handed support-hand solve must pass the same sweep max-open caps.'
Require-Text 'tools/generate_grab_finger_calibration.py' `
    'OVER_OPEN_MAX = 2\.0' `
    'The calibration bake must sample the full over-open range for every finger.'

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

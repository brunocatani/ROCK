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
    'anticipationOpenValue\);',
    'useThumbIndexCurveOnlyPose\(liveFingerPose\)'
) 'Grab acquisition must live re-solve wrap fingers against the converging object (pinch keeps the blend).'

# The held update interval must re-solve the curls against the live seat, not
# republish the promotion-instant snapshot for the whole hold.
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'rockGrabFingerPoseUpdateInterval',
    'tryGetGrabDriveObjectWorldTransform\(',
    'heldPinchFingerPose',
    'solveGrabFingerPoseFromTriangles\(',
    'rockGrabFingerSweepContactRadiusGameUnits\);'
) 'Held finger pose must re-solve curls at the update interval against the live seat.'

# The live chain chord is rotated by the CURRENT curl; anchoring the arc zero
# on it directly stopped every finger short by that curl (air gap) and made
# held re-solves oscillate. The runtime must de-rotate the chord to the true
# open reference via the baked Tip reach-table inversion, in the baked
# arc-plane sign convention.
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'estimateCalibratedChainCurlFromChord\(',
    'BakedGrabFingerProbe::Tip',
    'chordScale <= reachA && chordScale >= reachB'
) 'Chord curl estimation must invert the baked Tip probe reach table.'
Require-OrderedText 'src/physics-interaction/grab/GrabFinger.h' @(
    'RE::NiPoint3 openDirectionWorld = live\.openDirection;',
    'estimateCalibratedChainCurlFromChord\(',
    '-chordCurl\.chordAngleRadians \* chordCurl\.normalSign'
) 'The runtime solve must de-rotate the live chord to the open reference before anchoring arcs on it.'

# Held re-solves within noise of the current pose must not churn new FRIK
# targets every interval (finger micro-twitch).
Require-OrderedText 'src/physics-interaction/hand/HandGrab.cpp' @(
    'heldResolveMaxValueDelta',
    'liveFingerPose\.solved && heldResolveMaxValueDelta > 0\.02f'
) 'Held finger re-solve must apply a publish deadband.'

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

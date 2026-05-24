param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/grab/GrabPinchPocket.h' 'evaluateObject' `
    'Pinch-pocket classification must remain isolated in a pure policy helper.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'enum class GrabSeatMode[\s\S]*PinchPocket' `
    'Canonical grab frames must carry an explicit pinch seat mode.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimePinchPocketCandidate[\s\S]*resolveLiveFingerSkeletonSnapshot' `
    'Pinch pocket must be captured from the root-flattened finger snapshot at grab commit.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pinchDetectionDirectionWorld[\s\S]*findClosestGrabSurfaceHitToPointPositionOnly' `
    'Pinch mesh surface search must use the explicit pinch detection direction, not the palm normal.'
Require-Text 'src/physics-interaction/core/PhysicsFrameContext.h' 'pinchPocketWorld[\s\S]*hasPinchPocketWorld' `
    'Frame context must carry the live thumb-index pinch pocket.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'resolveLiveFingerSkeletonSnapshot[\s\S]*pinchPocketWorld' `
    'Debug pocket markers must use the live thumb-index pocket snapshot.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'resolvePinchOriginIfNeeded[\s\S]*resolveLiveFingerSkeletonSnapshot[\s\S]*resolvedPinchOrigin' `
    'Runtime pinch selection must lazily resolve the live thumb-index pocket after palm selection misses.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'rockGrabPinchCloseSelectionEnabled[\s\S]*findCloseObject\(bhkWorld,[\s\S]*resolvedPinchOrigin,[\s\S]*pinchDirection' `
    'Pinch close selection must cast from the live pinch pocket after palm close selection misses.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'currentCloseSelectionOrigin[\s\S]*pinchCloseSelectionFallback[\s\S]*pinchOrigin[\s\S]*body_frame::distance' `
    'Pinch close selections must keep hysteresis distance tied to the live pinch origin.'
Require-Text 'src/physics-interaction/object/ObjectDetection.h' 'pinchCloseSelectionFallback' `
    'Pinch-direction selections must carry source identity into grab commit.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'sel\.pinchCloseSelectionFallback && !pinchPocketCandidate\.valid[\s\S]*return false;' `
    'Pinch-direction selections must fail closed instead of falling through to palm-pocket authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool usingPinchPocket = pinchPocketCandidate\.valid && gripArea\.valid' `
    'Grab commit must arbitrate pinch and palm as mutually exclusive seat choices.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.seatMode = usingPinchPocket \? GrabSeatMode::PinchPocket : GrabSeatMode::PalmPocket' `
    'Accepted capture must store the selected seat mode on the grab frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if \(_grabFrame\.seatMode == GrabSeatMode::PinchPocket\)[\s\S]*return false;' `
    'Pinch grabs must not run held palm-pocket support refresh.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'usingPinchPocket \? grab_three_phase::AcquisitionPhase::TouchHeld : phaseDecision\.phase' `
    'Pinch grabs must commit directly to TouchHeld instead of entering the converging same-point final-freeze path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool pinchFingerPose = _grabFrame\.seatMode == GrabSeatMode::PinchPocket' `
    'Initial finger solve must identify pinch pose mode from stored seat state.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabMaxTriangleDistance, !pinchFingerPose, liveFingerSnapshotAtGrabPtr' `
    'Pinch finger solve must bypass the generic thumb curve solver.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyPinchFingerPosePolicy\(fingerPose, _grabFrame, g_rockConfig\.rockGrabFingerMinValue' `
    'Pinch finger pose must be post-processed into fixed thumb/index curves and closed other fingers.'
Require-Text 'src/physics-interaction/grab/GrabPinchPocket.h' 'buildStablePinchFingerPose[\s\S]*jointValues\[0\][\s\S]*jointValues\[2\]' `
    'Pinch finger pose must publish a stable whole-thumb joint shape instead of raw mesh-solver curl.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildStablePinchFingerPose[\s\S]*surfaceAimTargetValid\[finger\] = 0[\s\S]*pose\.solved = true' `
    'Pinch pose must clear raw mesh-solver thumb/index aim instead of installing object-local mesh-follow targets.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'useThumbIndexCurveOnlyPose\(fingerPose\)[\s\S]*captureSurfaceAimObjectLocal\(fingerPose, objectWorldTransform\)' `
    'Object grabs must disable thumb/index mesh-follow before capturing object-local surface aim.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' 'thumbSurfaceFollowAllowed[\s\S]*shouldApplySurfaceAimCorrection' `
    'Thumb mesh-follow authority must be explicit so ROCK object grabs can use fixed thumb/index curves.'
Reject-Text 'src/physics-interaction/grab/GrabFinger.h' 'usedPinchThumbOpposition|applyPinchThumbOppositionCorrection|shouldApplyPinchThumbLocalCorrection|pinchThumbSegmentCorrectionStrength' `
    'Pinch thumb must not keep a separate mesh-follow/opposition correction path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'usedPinchThumbOpposition|applyPinchThumbOppositionCorrection|shouldApplyPinchThumbLocalCorrection|pinchThumbSegmentCorrectionStrength' `
    'Hand grab must not re-enable the old pinch thumb mesh-follow path.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'drawGrabPockets[\s\S]*LeftPalmPocketCenter[\s\S]*LeftPinchPocketCenter[\s\S]*LeftPinchDetectionDirection' `
    'Debug overlay must expose per-hand palm and pinch pocket markers.'
Require-Text 'data/config/ROCK.ini' 'bDebugDrawGrabPockets[\s\S]*bGrabPinchCloseSelectionEnabled[\s\S]*fGrabPinchDetectionDirectionHandspaceX[\s\S]*fGrabPinchDetectionAxisBlend' `
    'Packaged INI must document the grab-pocket debug and pinch-direction tuning keys.'
Require-Text 'CMakeLists.txt' 'ROCKGrabPinchPocketPolicyTests' `
    'Pinch-pocket policy tests must be part of ROCKPolicyTestBinaries.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab pinch-pocket source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab pinch-pocket source boundary passed.'

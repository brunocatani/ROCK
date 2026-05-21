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

Require-Text 'src/physics-interaction/grab/GrabPinchPocket.h' 'evaluateObject' `
    'Pinch-pocket classification must remain isolated in a pure policy helper.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'enum class GrabSeatMode[\s\S]*PinchPocket' `
    'Canonical grab frames must carry an explicit pinch seat mode.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimePinchPocketCandidate[\s\S]*resolveLiveFingerSkeletonSnapshot' `
    'Pinch pocket must be captured from the root-flattened finger snapshot at grab commit.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool usingPinchPocket = pinchPocketCandidate\.valid && gripArea\.valid' `
    'Grab commit must arbitrate pinch and palm as mutually exclusive seat choices.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.seatMode = usingPinchPocket \? GrabSeatMode::PinchPocket : GrabSeatMode::PalmPocket' `
    'Accepted capture must store the selected seat mode on the grab frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if \(_grabFrame\.seatMode == GrabSeatMode::PinchPocket\)[\s\S]*return false;' `
    'Pinch grabs must not run held palm-pocket support refresh.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'promotionRequested &&\s*_grabFrame\.seatMode != GrabSeatMode::PinchPocket' `
    'Pinch grabs must not enter seated palm-pocket reacquire.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool pinchFingerPose = _grabFrame\.seatMode == GrabSeatMode::PinchPocket' `
    'Initial finger solve must identify pinch pose mode from stored seat state.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabMaxTriangleDistance, !pinchFingerPose, liveFingerSnapshotAtGrabPtr' `
    'Pinch finger solve must bypass the generic thumb curve solver.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyPinchFingerPosePolicy\(fingerPose, _grabFrame, objectWorldTransform' `
    'Pinch finger pose must be post-processed into explicit thumb/index targets and closed other fingers.'
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

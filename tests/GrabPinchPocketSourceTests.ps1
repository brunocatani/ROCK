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
Require-Text 'src/physics-interaction/core/PhysicsFrameContext.h' 'pinchPocketWorld[\s\S]*hasPinchPocketWorld' `
    'Frame context must carry the live thumb-index pinch pocket.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'resolvePinchOriginIfNeeded[\s\S]*resolveLiveFingerSkeletonSnapshot[\s\S]*resolvedPinchOrigin' `
    'Runtime pinch selection must lazily resolve the live thumb-index pocket after palm selection misses.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'const RE::NiPoint3 thumbPad = fingerSnapshot\.fingers\[0\]\.points\[2\];[\s\S]*const RE::NiPoint3 indexPad = fingerSnapshot\.fingers\[1\]\.points\[2\];[\s\S]*resolvedPinchOrigin = \(thumbPad \+ indexPad\) \* 0\.5f;' `
    'Pinch close selection must cast from the center between distal thumb/index source points.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'rockGrabPinchCloseSelectionEnabled[\s\S]*findCloseObject\(bhkWorld,[\s\S]*resolvedPinchOrigin,[\s\S]*pinchDirection' `
    'Pinch close selection must cast from the live pinch pocket after palm close selection misses.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'currentCloseSelectionOrigin[\s\S]*pinchCloseSelectionFallback[\s\S]*pinchOrigin[\s\S]*body_frame::distance' `
    'Pinch close selections must keep hysteresis distance tied to the live pinch origin.'
Require-Text 'src/physics-interaction/object/ObjectDetection.h' 'pinchCloseSelectionFallback' `
    'Pinch-direction selections must carry source identity into grab commit.'
Require-Text 'src/RockConfig.cpp' 'fGrabPinchCompactMaxExtentGameUnits[\s\S]*grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits,[\s\S]*1\.0f,[\s\S]*grab_pinch_pocket_policy::kDefaultCompactMaxExtentGameUnits' `
    'Runtime config loading must cap compact pinch extent at the current policy limit.'
Require-Text 'src/physics-interaction/grab/GrabPinchPocket.h' 'buildStableOppositionFingerPose[\s\S]*jointValues\[0\][\s\S]*jointValues\[2\][\s\S]*buildStablePinchFingerPose[\s\S]*buildStableOppositionFingerPose' `
    'Pinch finger pose must publish the stable whole-thumb opposition shape instead of raw mesh-solver curl.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' 'thumbSurfaceFollowAllowed[\s\S]*shouldApplySurfaceAimCorrection' `
    'Thumb mesh-follow authority must be explicit so ROCK object grabs can use fixed thumb/index curves.'
Reject-Text 'src/physics-interaction/grab/GrabFinger.h' 'usedPinchThumbOpposition|applyPinchThumbOppositionCorrection|shouldApplyPinchThumbLocalCorrection|pinchThumbSegmentCorrectionStrength' `
    'Pinch thumb must not keep a separate mesh-follow/opposition correction path.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'drawGrabPockets[\s\S]*LeftPalmPocketCenter[\s\S]*LeftPinchPocketCenter[\s\S]*LeftPinchDetectionDirection' `
    'Debug overlay must expose per-hand palm and pinch pocket markers.'
Require-Text 'CMakeLists.txt' 'ROCKGrabPinchPocketPolicyTests' `
    'Pinch-pocket policy tests must be part of ROCKPolicyTestBinaries.'
Require-Text 'src/physics-interaction/grab/GrabPinchPocket.h' 'compactBySize && pinchableThickness[\s\S]*compactTooThickToPinch' `
    'Compact pinch classification must also bound the thinnest extent - mugs and cans are compact by size but too thick to hold between two finger pads.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab pinch-pocket source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab pinch-pocket source boundary passed.'

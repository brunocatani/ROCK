param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'struct\s+OrientationBasis' 'Grab telemetry must carry full native X/Y/Z orientation bases, not only a single finger axis.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'makeOrientationBasis[\s\S]*rotateLocalVectorToWorld' 'Orientation basis telemetry must use ROCK/FO4VR localVectorToWorld convention.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'formatBasisDelta' 'Logs must include basis dot and per-axis degree deltas for visual reconstruction.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_boneColliders\.tryGetPalmAnchorTarget\(out\.palmAnchorTargetWorld\)' 'Telemetry must include the generated palm-anchor target frame from the root-flattened hand collider pipeline.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*return\s+colliderFrame;' 'The full-convention diagnostic build must make the generated-collider to grab-authority adapter identity.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmAnchorGrabAuthorityWorld\s*=[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(out\.palmAnchorTargetWorld\)' 'Telemetry must include the grab-authority boundary interpretation of the generated palm collider frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'proxyReadbackWorld\s*=\s*proxyWorld' 'Telemetry must include live proxy/body-A readback for visual comparison against the palm authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveLiveFingerSkeletonSnapshot\(_isLeft,\s*fingerSnapshot\)' 'Telemetry must include root-flattened finger landmark lines for the palm-to-finger direction.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GRAB BASIS CAPTURE side=\{\} phase=capture convention=niLocalVectorToWorld' 'Grab commit must log capture-time hand/proxy/object/BODY orientation before runtime motion can hide the snap source.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GRAB BASIS CAPTURE DELTA side=\{\} phase=capture' 'Grab commit must log capture-time basis deltas.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS \{\} \{\} side=\{\} convention=niLocalVectorToWorld' 'Per-frame telemetry must emit side-separated native-axis basis records.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS DELTA \{\} \{\} side=\{\}' 'Per-frame telemetry must emit side-separated basis deltas.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightGrabPalmGeneratedDirect' 'Overlay must draw the generated palm frame as directly consumed by grab math for convention comparison.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightGrabPalmAuthorityFrame' 'Overlay must draw the converted palm grab-authority frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightGrabProxyReadback' 'Overlay must draw live proxy/body-A readback when available.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS FRAMECOMPARE' 'Logs must include generated palm, grab authority, and proxy readback basis comparison.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'rootPalmLine=\{\}' 'Per-frame telemetry must include the palm-to-finger-base line.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'publishGrabTelemetry\(_rightHand,\s*false\)' 'Right-hand telemetry must remain an independent stream.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'publishGrabTelemetry\(_leftHand,\s*true\)' 'Left-hand telemetry must remain an independent stream.'

if ($failures.Count -gt 0) {
    Write-Host 'GrabOrientationBasisTelemetrySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GrabOrientationBasisTelemetrySourceTests passed.' -ForegroundColor Green

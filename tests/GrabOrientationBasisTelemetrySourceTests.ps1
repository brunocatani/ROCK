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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'struct\s+OrientationBasis' 'Grab telemetry must carry full native X/Y/Z orientation bases, not only a single finger axis.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'makeOrientationBasis[\s\S]*rotateLocalVectorToWorld' 'Orientation basis telemetry must use ROCK/FO4VR localVectorToWorld convention.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'formatBasisDelta' 'Logs must include basis dot and per-axis degree deltas for visual reconstruction.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_boneColliders\.tryGetPalmAnchorTarget\(out\.palmAnchorTargetWorld\)' 'Telemetry must include the generated palm-anchor target frame from the root-flattened hand collider pipeline.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'matrixFromAxes[\s\S]*matrix\.entry\[1\]\[0\]\s*=\s*xAxis\.y[\s\S]*matrix\.entry\[0\]\[1\]\s*=\s*yAxis\.x' 'Generated collider placement must keep the in-game verified column-stored native convention.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab telemetry must preserve the explicit generated-collider to corrected-proxy authority boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmAnchorGrabAuthorityBase\s*=[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(out\.handBodyWorld\)[\s\S]*palmAnchorGrabAuthorityWorld\s*=\s*applyGrabAuthorityProxyLocalOffsetToFrame\(palmAnchorGrabAuthorityBase,\s*_isLeft\)' 'Telemetry must include the offset hidden-proxy grab-authority interpretation of the actual live palm-anchor body frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'proxyReadbackWorld\s*=\s*proxyWorld' 'Telemetry must include live proxy/body-A readback for visual comparison against the palm authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveLiveFingerSkeletonSnapshot\(_isLeft,\s*fingerSnapshot\)' 'Telemetry must include root-flattened finger landmark lines for the palm-to-finger direction.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GRAB BASIS CAPTURE side=\{\} phase=capture convention=niLocalVectorToWorld' 'Grab commit must log capture-time hand/proxy/object/BODY orientation before runtime motion can hide the snap source.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GRAB BASIS CAPTURE DELTA side=\{\} phase=capture' 'Grab commit must log capture-time basis deltas.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS \{\} \{\} side=\{\} convention=niLocalVectorToWorld' 'Per-frame telemetry must emit side-separated native-axis basis records.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS DELTA \{\} \{\} side=\{\}' 'Per-frame telemetry must emit side-separated basis deltas.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightGrabPalmGeneratedDirect' 'Overlay must draw the generated palm frame as directly consumed by grab math for convention comparison.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightGrabPalmAuthorityFrame' 'Overlay must draw the converted palm grab-authority frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightGrabProxyReadback' 'Overlay must draw live proxy/body-A readback when available.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS FRAMECHAIN' 'Logs must include generated palm, grab authority, and proxy readback basis comparison.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS RAW_FALLBACK_PROXY_SEAT' 'Logs must expose raw-hand fallback proxy-seat diagnostics without retaining the old INI pivot.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'rawSeatPresent=\{\} activeSource=\{\}' 'Per-frame telemetry must separate fallback proxy-seat diagnostics from live palm authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB ANGULAR_DELTA' 'Per-frame telemetry must compare controller, authority, desired-object, and held-object angular deltas.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rawFallbackProxySeatWorld\s*=\s*computeFallbackGrabAuthorityProxySeatWorld' 'Telemetry must use the active proxy-seat fallback for raw-hand comparison diagnostics.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'nativeFlattenedHandToPalmAnchorTarget' 'Telemetry must distinguish the flattened hand frame from the generated palm frame.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'rawFallbackProxySeatToPalmAnchor' 'Telemetry must measure active fallback proxy-seat distance to the bone-derived palm without retaining the old configured pivot.'
Reject-Text 'src/physics-interaction/hand/HandFrame.h' 'GrabPivotAHandspace|computePalmPositionFromHandBasis|computeGrabPivotAPositionFromHandBasis' 'The retired configured hand-space pivot helpers must not remain in HandFrame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*palmReference\)[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)\.translate' 'Runtime dynamic grab pivot A must be sourced from the offset hidden grab authority proxy derived from the live palm-anchor body frame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*return computeFallbackGrabAuthorityProxySeatWorld\(fallbackHandWorldTransform,\s*_isLeft\)' 'Runtime dynamic grab pivot fallback must use the active proxy-seat offset instead of the retired INI-configured pivot.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'buildGrabPocketFrameWithPalmCenter' 'Dynamic grab pocket construction must accept the already-resolved generated palm center.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildGrabPocketFrameWithPalmCenter\([\s\S]*grabPivotAWorld' 'Dynamic grab capture must pass the generated palm pivot into three-phase pocket construction.'
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

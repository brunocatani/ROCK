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
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab telemetry must expose the corrected palm proxy authority adapter from generated collider columns to runtime rows.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmAnchorGrabAuthorityBase\s*=[\s\S]*makePalmAnchorGrabAuthorityBaseFrame\(out\.handBodyWorld\)[\s\S]*palmAnchorGrabAuthorityWorld\s*=\s*applyRuntimeGrabAuthorityProxyOffsetToFrame\(palmAnchorGrabAuthorityBase,\s*out\.rawHandWorld,\s*_isLeft\)' 'Telemetry must include the generated palm base plus raw-hand rotation authority frame.'
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
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB BASIS LEGACY_PIVOT' 'Logs must expose old INI pivot contamination separately from bone-derived palm authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'legacyActive=no activeSource=\{\}' 'Per-frame telemetry must prove the old INI pivot is diagnostic-only, not runtime authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB ANGULAR_DELTA' 'Per-frame telemetry must compare controller, authority, desired-object, and held-object angular deltas.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'currentConstraintDesiredObjectWorld|currentConstraintDesiredBodyWorld|constraintReverseTargetWorld|conHSRev|GRAB FRAMECHAIN CANDIDATES|GRAB FRAMECHAIN CANDIDATE|GRAB FRAMECHAIN AXES' 'Per-frame telemetry must not keep retired constraint-space desired/reverse/framechain diagnostics.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.(constraintHandSpace|constraintBodyHandSpace|constraintBodyLocal)|desiredConstraintBodyWorld|objectToConstraintBodyAtGrab' 'Grab implementation must not retain retired constraint-space capture relations.'
Reject-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'FrameChainCandidate|kFrameChainCandidateCapacity|hasFrameChainCandidates|frameChainCandidates' 'Telemetry structs must not preserve the retired framechain candidate dump.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'legacyConfiguredPivotAWorld\s*=\s*computeGrabPivotAPositionFromHandBasis' 'Telemetry must label the old INI pivot separately instead of treating it as the generated palm.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'nativeFlattenedHandToPalmAnchorTarget' 'Telemetry must distinguish the flattened hand frame from the generated palm frame.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'legacyConfiguredPivotAToPalmAnchor' 'Telemetry must measure old configured pivot distance to the bone-derived palm without making it an authority reference.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'handBodyFingerBaseLineWorld' 'Telemetry must keep a finger-base direction anchored from the live hand body frame.'
Require-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'palmAnchorFingerBaseLineWorld' 'Telemetry must keep a finger-base direction anchored from the generated palm frame.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'computeGrabPivotAHandspacePosition[\s\S]*g_rockConfig\.rockLeftGrabPivotAHandspace[\s\S]*g_rockConfig\.rockRightGrabPivotAHandspace' 'Source proof must keep the old INI-configured handspace point available only for legacy telemetry and non-dynamic-grab callers.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*palmReference\)[\s\S]*makePalmAnchorGrabAuthorityBaseFrame\(palmReference\.world\)[\s\S]*applyRuntimeGrabAuthorityProxyOffsetToFrame\(proxyBaseWorld,\s*fallbackHandWorldTransform,\s*_isLeft\)\.translate' 'Runtime dynamic grab pivot A must preserve the live palm-anchor origin while using raw hand rotation for the active authority frame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*return fallbackHandWorldTransform\.translate' 'Runtime dynamic grab pivot fallback must use raw hand origin instead of the retired INI-configured offset.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'buildGrabPocketFrameWithPalmCenter' 'Dynamic grab pocket construction must accept the already-resolved palm-pocket center.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'rejectFromAxis\(rawFingerForwardWorld,\s*frame\.palmNormalWorld\)[\s\S]*rejectFromAxis\(thumbSideCandidate,\s*frame\.fingerForwardWorld\)' 'Dynamic grab pocket roll must project raw hand axes around the unchanged palm normal.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildGrabPocketFrameWithPalmCenter\([\s\S]*grabPivotAWorld' 'Dynamic grab capture must pass the selected palm-pocket pivot into three-phase pocket construction.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'rootPalmLine=\{\}' 'Per-frame telemetry must include the palm-to-finger-base line.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'handBodyFingerBaseLineWorld\s*=' 'Telemetry must compute a finger-base direction from the live hand body origin.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmAnchorFingerBaseLineWorld\s*=' 'Telemetry must compute a finger-base direction from the generated palm origin.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'handBodyPalmLine=\{\} generatedPalmLine=\{\}' 'Per-frame telemetry must expose hand-body and generated-palm finger-base lines separately from the raw-hand line.'
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

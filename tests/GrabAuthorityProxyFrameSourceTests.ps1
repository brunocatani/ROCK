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

Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+LivePalmAnchorReference' 'Dynamic grab must have an explicit live palm-anchor reference type.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryResolveLivePalmAnchorReference' 'Dynamic grab must resolve the actual palm anchor body as the hand-side reference.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' '_latestPalmAnchorTarget\s*=\s*anchorFrame\.transform' 'Palm target must be captured from the same sampled role frame queued for the generated palm anchor.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryResolveLivePalmAnchorReference[\s\S]*tryResolveLiveBodyWorldTransform\(world,\s*_handBody\.getBodyId\(\)' 'Live palm authority must read the actual palm-anchor hknp body frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*tryResolveLivePalmAnchorReference[\s\S]*livePalmAnchorMotionGrabFrame' 'Proxy authority must resolve from the live palm-anchor body frame.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'matrixFromAxes[\s\S]*matrix\.entry\[1\]\[0\]\s*=\s*xAxis\.y[\s\S]*matrix\.entry\[0\]\[1\]\s*=\s*yAxis\.x' 'Generated hand/body colliders must keep the in-game verified column-stored native placement convention.'
Reject-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab authority proxy must not transpose the generated palm frame; in-game proxy isolation proved that convention is wrong.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)' 'Proxy authority must pass the live palm body frame through the explicit boundary.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rootFlattenedPalmAnchorTarget' 'Proxy authority must not use the sampled palm target as the active body-A frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rawHandFallback' 'Proxy authority must not silently fall back to raw controller space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildSplitGrabFrameFromDesiredObject\(\s*handWorldTransform,\s*proxyFrameWorldAtGrab' 'Constraint hand-space frame must be captured from the proxy/palm frame, not live stale hknp readback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*grabBodyWorldAtGrab\)' 'Proxy authority must preserve the visual-object to hknp BODY relation captured at grab time.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintUsesMotionBodyAtGrab\s*=\s*false' 'Proxy authority must keep hknp constraint body-B data in the rigid BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*grabBodyWorldAtGrab' 'Proxy authority must encode transform-B and angular target data against the BODY frame, not MOTION/COM.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToConstraintBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*constraintBodyWorldAtGrab\)' 'Proxy authority must store a separate solver-local object relation for hknp constraint body-B atoms.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*objectToBodyAtGrab' 'Proxy authority must store the captured visual-object to BODY relation instead of assuming BODY equals the visible node.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintBodyLocal\s*=\s*objectToConstraintBodyAtGrab' 'Proxy authority must not reuse the visual BODY relation as the custom constraint solver relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyWorld\s*=\s*multiplyTransforms\(desiredObjectWorld,\s*objectToBodyAtGrab\)' 'Proxy authority must derive the desired BODY frame from the desired visual object frame instead of treating visual local coordinates as body local.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredConstraintBodyWorld\s*=\s*multiplyTransforms\(desiredObjectWorld,\s*objectToConstraintBodyAtGrab\)' 'Proxy authority must derive the solver body-B frame from the desired visual object frame and the captured solver-local relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintBodyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(proxyFrameWorldAtGrab,\s*desiredConstraintBodyWorld\)' 'Proxy authority must store a solver-space relation separate from the visual BODY relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'convertNativeBodyReadbackToConventionalObjectFrame\(grabBodyWorldAtGrab\)' 'Proxy authority must capture a separate conventional object-frame candidate without changing BODY pivot authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintConventionalBodyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(proxyFrameWorldAtGrab,\s*desiredConventionalBodyWorld\)' 'Proxy authority must keep a conventional angular candidate separate from the BODY linear/pivot relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveProxyConstraintAngularDriveTargetWorld\([\s\S]*makeNativeAngularBoundaryTargetFromVisualTarget' 'Proxy angular velocity drive must choose its own angular target instead of always consuming the BODY linear target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyProxyConstraintAngularVelocityDrive\(\s*world,\s*desiredAngularTargetWorld' 'Proxy direct angular velocity must consume the selected angular target, not the BODY linear target unconditionally.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeRawRotationPalmTranslationFrame\(handWorldTransform,\s*proxyFrameWorldAtGrab\)' 'Proxy authority must have a hybrid frame that keeps palm translation while using raw hand rotation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.rawRotationProxyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(rawRotationProxyFrameWorldAtGrab,\s*desiredObjectWorld\)' 'Proxy authority must capture object relation in the hybrid raw-rotation/palm-translation frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'g_rockConfig\.rockGrabObjectRotationReferenceMode\s*==\s*3[\s\S]*rawRotationProxyBodyHandSpace' 'Mode 3 must drive the held object from the hybrid raw-rotation proxy relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab' 'Proxy constraint creation must use the resolved palm-frame proxy transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformProxySpace\s*=\s*_grabFrame\.constraintBodyHandSpace' 'Proxy constraint target math must use the captured solver-space proxy relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outActivePivotBBodyLocalGame\s*=\s*activeProxyConstraintPivotBLocalGame\(\)' 'Proxy constraint target math must use the frozen solver-local copy of the selected grip pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'transformBTranslation\[0\]\s*=\s*outActivePivotBBodyLocalGame\.x\s*\*\s*gameToHkScale' 'Transform-B must be written from the frozen solver-local selected grip pivot, not a recomputed visual/object local point.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy runtime error and pivot telemetry must read body-B through the BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(authorityFrame,[\s\S]*rawRotationProxyHandSpace[\s\S]*constraintHandSpace' 'Proxy desired object frame must compose from the selected authority frame: hybrid raw-rotation/palm-translation in mode 3, proxy palm frame otherwise.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueProxyGrabAuthorityTarget\(\s*proxyAuthorityWorld' 'Held updates must queue the resolved palm-frame proxy target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueGeneratedKeyframedBodyTarget\(\s*_grabAuthorityProxyDriveState,\s*pending\.proxyWorld' 'Proxy authority must queue the live palm body frame through the generated keyframed-body drive path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveGeneratedKeyframedBody\([\s\S]*_grabAuthorityProxy[\s\S]*"grab-authority-proxy"' 'Proxy authority must move with the same native generated-body drive path as the actual palm collider.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryResolveLiveBodyWorldTransform\(world,\s*proxyBodyId,\s*proxyReadbackBetween' 'Proxy readback telemetry must inspect the same live frame shown by the visual debug overlay.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeHardKeyframeVelocityForTarget\(\s*world,\s*proxyBodyId,\s*pending\.proxyWorld,\s*driveDelta,[\s\S]*angularVelocityHavok\)' 'Proxy authority may keep FO4VR hard-keyframe velocity as diagnostic fallback when palm motion velocity is unavailable.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'ComputeHardKeyFrame_t[\s\S]*offsets::kFunc_ComputeHardKeyFrame' 'Proxy and held-object angular velocity must use the FO4VR native hard-keyframe angular-vector boundary.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationColumns\(transformBRotation,\s*bodyToHandRotation\)' 'FO4VR custom grab angular setup must write transform-B as Havok column blocks, matching HIGGS hkMatrix storage.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationColumns\(targetBRca,\s*bodyToHandRotation\)' 'FO4VR custom grab angular setup must write target_bRca as Havok column blocks so the solver sees the inverse body-to-hand relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveQueuedTargets\s*=\s*_grabAuthorityProxyQueuedSequence' 'Held telemetry must report proxy queue counters when proxy authority is active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveFlushedTargets\s*=\s*_grabAuthorityProxyFlushSequence' 'Held telemetry must report proxy flush counters when proxy authority is active.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.rawHandSpace\)' 'Proxy body-A convention must not use the raw controller hand-space relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*makeIdentityTransform\(\)' 'Proxy authority must not erase the visual-object to BODY relation; doing so causes immediate angular correction after grab.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*constraintUsesMotionBodyAtGrab\s*\?\s*motionBodyWorldAtGrab\s*:\s*grabBodyWorldAtGrab' 'Proxy authority must not route body-B constraint data through MOTION/COM.'
Reject-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationRows\(transformBRotation|writeHavokRotationRows\(targetBRca' 'Proxy angular setup must not use the row write that makes telemetry rows match while hknp consumes the forward relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeAngularVelocityRadiansPerSecond\(previousProxyWorld,\s*pending\.proxyWorld,\s*driveDelta,\s*angularVelocityHavok\)' 'Proxy authority must not return to the old matrix-column angular delta helper.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'proxyDrive=exactZeroVelocity' 'Proxy authority must not return to the sampled-target exact-zero-velocity drive policy.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab authority proxy frame source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab authority proxy frame source test passed.'

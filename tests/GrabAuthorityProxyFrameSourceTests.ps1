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
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'computeGrabAuthorityProxyOffsetLocalGame[\s\S]*rockLeftGrabAuthorityProxyOffsetGameUnits[\s\S]*rockRightGrabAuthorityProxyOffsetGameUnits' 'Hidden grab authority proxy offset must be a per-hand INI-backed local offset, not a real hand-collider move.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'applyGrabAuthorityProxyLocalOffsetToFrame[\s\S]*localVectorToWorld\(proxyFrameWorld,\s*localOffset\)' 'Hidden grab authority proxy offset must be applied in proxy local space.'
Require-Text 'src/RockConfig.h' 'rockRightGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the right-hand hidden proxy offset.'
Require-Text 'src/RockConfig.h' 'rockLeftGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the left-hand hidden proxy offset.'
Require-Text 'src/RockConfig.cpp' 'fRightGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the right-hand hidden proxy offset from INI.'
Require-Text 'src/RockConfig.cpp' 'fLeftGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the left-hand hidden proxy offset from INI.'
Require-Text 'data/config/ROCK.ini' 'fRightGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0' 'Packaged INI must expose the right-hand hidden proxy offset.'
Require-Text 'data/config/ROCK.ini' 'fLeftGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0' 'Packaged INI must expose the left-hand hidden proxy offset.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'matrixFromAxes[\s\S]*matrix\.entry\[1\]\[0\]\s*=\s*xAxis\.y[\s\S]*matrix\.entry\[0\]\[1\]\s*=\s*yAxis\.x' 'Generated hand/body colliders must keep the in-game verified column-stored native placement convention.'
Reject-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab authority proxy must not transpose the generated palm frame; in-game proxy isolation proved that convention is wrong.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)' 'Proxy authority must pass the live palm body frame through the explicit boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)' 'Resolved hidden proxy frame must consume the configured local offset.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmProxyWorld\s*=\s*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)' 'Per-frame hidden proxy target must keep consuming the configured local offset.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pending\.proxyWorld\s*=\s*makeRawRotationPalmTranslationFrame\(pending\.rawHandWorld,\s*palmProxyWorld\)' 'The hidden proxy body must carry raw hand rotation with palm-collider translation, so transform-A does not become a second angular authority.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)\.translate' 'Pivot-A must use the offset hidden proxy origin derived from the live palm anchor.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rootFlattenedPalmAnchorTarget' 'Proxy authority must not use the sampled palm target as the active body-A frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rawHandFallback' 'Proxy authority must not silently fall back to raw controller space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildSplitGrabFrameFromDesiredObject\(\s*handWorldTransform,\s*proxyFrameWorldAtGrab' 'Constraint hand-space frame must be captured from the proxy/palm frame, not live stale hknp readback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*grabBodyWorldAtGrab\)' 'Proxy authority must preserve the visual-object to hknp BODY relation captured at grab time.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintUsesMotionBodyAtGrab\s*=\s*[\s\S]*motionBodySourceAtGrab\s*==\s*body_frame::BodyFrameSource::MotionCenterOfMass' 'Proxy authority must use the solver MOTION frame for body-B constraint bytes when FO4VR exposes one.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*[\s\S]*constraintUsesMotionBodyAtGrab\s*\?\s*motionBodyWorldAtGrab\s*:\s*grabBodyWorldAtGrab' 'Proxy authority must split visible BODY grip evidence from solver body-B constraint data.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToConstraintBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*constraintBodyWorldAtGrab\)' 'Proxy authority must store a separate solver-local object relation for hknp constraint body-B atoms.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*objectToBodyAtGrab' 'Proxy authority must store the captured visual-object to BODY relation instead of assuming BODY equals the visible node.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintBodyLocal\s*=\s*objectToConstraintBodyAtGrab' 'Proxy authority must not reuse the visual BODY relation as the custom constraint solver relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.pivotBConstraintLocalGame\s*=\s*constraintDrivePivotBBodyLocalGame\(_grabFrame\)' 'Proxy authority must pair transform-B from PivotA and the captured body-A/body-B relation after frame capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyWorld\s*=\s*multiplyTransforms\(desiredObjectWorld,\s*objectToBodyAtGrab\)' 'Proxy authority must derive the desired BODY frame from the desired visual object frame instead of treating visual local coordinates as body local.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredConstraintBodyWorld\s*=\s*multiplyTransforms\(desiredObjectWorld,\s*objectToConstraintBodyAtGrab\)' 'Proxy authority must derive the solver body-B frame from the desired visual object frame and the captured solver-local relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintBodyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(proxyFrameWorldAtGrab,\s*desiredConstraintBodyWorld\)' 'Proxy authority must store a solver-space relation separate from the visual BODY relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'convertNativeBodyReadbackToConventionalObjectFrame|constraintConventionalBodyHandSpace|desiredConventionalBodyWorld' 'Proxy authority must not retain removed conventional angular candidates.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveProxyConstraintAngularDriveTargetWorld|makeNativeAngularBoundaryTargetFromVisualTarget|applyProxyConstraintAngularVelocityDrive' 'Proxy authority must not keep the removed direct angular-velocity target path when the ragdoll atom owns rotation.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'setGrabMotorAtomsActive\(header,\s*true,\s*true\)' 'Proxy grab constraints must enable the ragdoll angular atom together with the linear atoms.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'writeGrabTransformARotation\(transformARotation,\s*transformAFrameLocal\.rotate\)' 'Proxy target updates must keep transform-A paired with the already raw-rotated proxy body frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'writeGrabRagdollAngularTarget\(targetBRca,\s*desiredBodyTransformBodyASpace,\s*transformAFrameLocal\.rotate\)' 'Proxy target updates must refresh target_bRca from the raw hand/object relation without adding a second live angular authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeRawRotationPalmTranslationFrame\(handWorldTransform,\s*proxyFrameWorldAtGrab\)' 'Proxy authority must have a hybrid frame that keeps palm translation while using raw hand rotation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.rawRotationProxyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(rawRotationProxyFrameWorldAtGrab,\s*desiredObjectWorld\)' 'Proxy authority must capture object relation in the hybrid raw-rotation/palm-translation frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kGrabObjectRotationReferenceName\s*=\s*"rawRotationPalmTranslation"' 'The validated raw-rotation/palm-translation authority must be the only production rotation reference.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabObjectRotationReferenceMode|grabObjectRotationReferenceModeName|legacyBody|convertedBodyCapture|splitVisualNativeBoundary' 'Proxy authority must not keep the removed selectable rotation modes.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab' 'Proxy constraint creation must use the resolved palm-frame proxy transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformAuthoritySpace\s*=\s*_grabFrame\.rawRotationProxyBodyHandSpace' 'Proxy constraint target math must use the captured raw-hand angular authority relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformBodyASpace\s*=\s*_grabFrame\.rawRotationProxyBodyHandSpace' 'Ragdoll atom target math must use the raw-hand object-to-proxy relation while transform-A remains stable.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outActivePivotBBodyLocalGame\s*=\s*[\s\S]*computeDynamicTransformBTranslationGame\(\s*desiredBodyTransformBodyASpace,\s*_grabAuthorityPivotAProxyLocalGame\s*\)' 'Proxy constraint target math must derive transform-B from the same body-A palm/object relation used by the constraint.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'transformBTranslation\[0\]\s*=\s*outActivePivotBBodyLocalGame\.x\s*\*\s*gameToHkScale' 'Transform-B must be written from the HIGGS-style paired body-A/body-B pivot relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*tryResolveLiveBodyWorldTransform\(world,\s*bodyId,\s*outTransform,\s*&source,\s*&motionIndex\)' 'Proxy runtime error and pivot telemetry must read body-B through the solver-facing live frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(authorityFrame,\s*_grabFrame\.rawRotationProxyHandSpace\)' 'Proxy desired object frame must always compose from the validated hybrid raw-rotation/palm-translation relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueProxyGrabAuthorityTarget\(\s*proxyAuthorityWorld' 'Held updates must queue the resolved palm-frame proxy target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueGeneratedKeyframedBodyTarget\(\s*_grabAuthorityProxyDriveState,\s*pending\.proxyWorld' 'Proxy authority must queue the live palm body frame through the generated keyframed-body drive path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveGeneratedKeyframedBody\([\s\S]*_grabAuthorityProxy[\s\S]*"grab-authority-proxy"' 'Proxy authority must move with the same native generated-body drive path as the actual palm collider.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryResolveLiveBodyWorldTransform\(world,\s*proxyBodyId,\s*proxyReadbackBetween' 'Proxy readback telemetry must inspect the same live frame shown by the visual debug overlay.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeHardKeyframeVelocityForTarget\(\s*world,\s*proxyBodyId,\s*pending\.proxyWorld,\s*driveDelta,[\s\S]*angularVelocityHavok\)' 'Proxy authority may keep FO4VR hard-keyframe velocity as diagnostic fallback when palm motion velocity is unavailable.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'ComputeHardKeyFrame_t[\s\S]*offsets::kFunc_ComputeHardKeyFrame' 'Proxy drive telemetry must keep the FO4VR native hard-keyframe velocity boundary.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationColumns\(transformBRotation,\s*bodyToBodyARotation\)' 'FO4VR custom grab angular setup must write transform-B as Havok column blocks from the body-to-body-A relation.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationColumns\(targetBRca,\s*targetRotation\)' 'FO4VR custom grab angular setup must write target_bRca as Havok column blocks after composing body-to-body-A with transform-A.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveQueuedTargets\s*=\s*_grabAuthorityProxyQueuedSequence' 'Held telemetry must report proxy queue counters when proxy authority is active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveFlushedTargets\s*=\s*_grabAuthorityProxyFlushSequence' 'Held telemetry must report proxy flush counters when proxy authority is active.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.rawHandSpace\)' 'Proxy body-A convention must not use the raw controller hand-space relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*makeIdentityTransform\(\)' 'Proxy authority must not erase the visual-object to BODY relation; doing so causes immediate angular correction after grab.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.pivotBBodyLocalGame\s*=\s*_grabFrame\.gripPointBodyLocalGame' 'Proxy authority must keep a visible BODY-local copy of the selected grip point while solver bytes use the drive frame.'
Reject-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationRows\(transformBRotation|writeHavokRotationRows\(targetBRca' 'Proxy angular setup must not use the row write that makes telemetry rows match while hknp consumes the forward relation.'
Reject-Text 'src/physics-interaction/grab/GrabAuthorityProxyMotion.h' 'computeAngularVelocityRadiansPerSecond|matrixColumn|axisSum' 'The old manual matrix-column proxy angular helper must stay removed; FO4VR hard-keyframe angular velocity owns that native boundary.'
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

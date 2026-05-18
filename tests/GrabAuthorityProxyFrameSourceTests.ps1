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
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabAuthorityProxyDebugSnapshot' 'Debug overlay must expose a grab authority proxy target snapshot that is not dependent on an active grab.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' '_latestPalmAnchorTarget\s*=\s*anchorFrame\.transform' 'Palm target must be captured from the same sampled role frame queued for the generated palm anchor.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryResolveLivePalmAnchorReference[\s\S]*tryResolveLiveBodyWorldTransform\(world,\s*_handBody\.getBodyId\(\)' 'Live palm authority must read the actual palm-anchor hknp body frame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::getGrabAuthorityProxyDebugSnapshot\(RE::hknpWorld\* world,\s*const RE::NiTransform& rawHandWorld[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*palmReference\)[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(palmAuthorityBaseWorld,\s*_isLeft\)' 'Idle proxy debug target must show the corrected proxy-local palm seat before a grab creates the readback proxy body.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*tryResolveLivePalmAnchorReference[\s\S]*livePalmAnchorMotionGrabFrame' 'Proxy authority must resolve from the live palm-anchor body frame.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'computeGrabAuthorityProxyOffsetLocalGame[\s\S]*rockLeftGrabAuthorityProxyOffsetGameUnits[\s\S]*rockRightGrabAuthorityProxyOffsetGameUnits' 'Hidden grab authority proxy offset must be a per-hand INI-backed local offset, not a real hand-collider move.'
Reject-Text 'src/physics-interaction/hand/HandFrame.h' 'makeGrabStartupCaptureAuthorityFrame' 'Grab-start capture must not keep the old raw-rotation startup proxy workaround.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'applyGrabAuthorityProxyLocalOffsetToFrame[\s\S]*localVectorToWorld\(proxyFrameWorld,\s*localOffset\)' 'Hidden grab authority proxy offset must be applied in proxy local space.'
Require-Text 'src/RockConfig.h' 'rockRightGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the right-hand hidden proxy offset.'
Require-Text 'src/RockConfig.h' 'rockLeftGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the left-hand hidden proxy offset.'
Require-Text 'src/RockConfig.cpp' 'fRightGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the right-hand hidden proxy offset from INI.'
Require-Text 'src/RockConfig.cpp' 'fLeftGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the left-hand hidden proxy offset from INI.'
Require-Text 'src/RockConfig.h' 'persistGrabAuthorityProxyOffset' 'RockConfig must expose persistence for the active hidden proxy offset, not only the legacy handspace pivot.'
Require-Text 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'selectedProxyOffset[\s\S]*rockLeftGrabAuthorityProxyOffsetGameUnits[\s\S]*rockRightGrabAuthorityProxyOffsetGameUnits' 'Debug controller tuning must edit the active grab authority proxy offset.'
Require-Text 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'persistGrabAuthorityProxyOffset' 'Debug controller tuning must persist the active grab authority proxy offset.'
Require-Text 'data/config/ROCK.ini' 'fRightGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0[\s\S]*fRightGrabAuthorityProxyOffsetYGameUnits\s*=\s*-2\.0' 'Packaged INI must expose the corrected right-hand hidden proxy palm-seat offset.'
Require-Text 'data/config/ROCK.ini' 'fLeftGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0[\s\S]*fLeftGrabAuthorityProxyOffsetYGameUnits\s*=\s*-2\.0' 'Packaged INI must expose the corrected left-hand hidden proxy palm-seat offset.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'matrixFromAxes[\s\S]*matrix\.entry\[1\]\[0\]\s*=\s*xAxis\.y[\s\S]*matrix\.entry\[0\]\[1\]\s*=\s*yAxis\.x' 'Generated hand/body colliders must keep the in-game verified column-stored native placement convention.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab authority proxy must adapt generated column-stored palm frames into proxy-local vector convention.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'buildPalmAnchorFrame[\s\S]*palmDepthAxis\s*=\s*normalizeOr\(cross\(result\.crossPalmAxis,\s*result\.xAxis\)' 'Palm anchor generation must derive runtime local Y as the palm-depth axis.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'buildPalmDepthAlignedSegmentFrame[\s\S]*Force local Y to palm depth' 'Palm-adjacent segment boxes must force local Y to the corrected palm-depth axis.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'PalmFace[\s\S]*outFrame\.transform\.translate\s*=\s*outFrame\.transform\.translate\s*-\s*palm\.palmDepthAxis' 'Palm face collider seating must move along local -Y palm depth.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'PalmHeel[\s\S]*ThumbPad[\s\S]*buildPalmDepthAlignedSegmentFrame\(input,\s*palm\.palmDepthAxis\)' 'PalmHeel and ThumbPad box frames must keep anatomical placement while using the corrected palm-depth axis.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'makePalmBoxHullPoints<RE::NiPoint3>\(length,\s*palmDepth,\s*crossPalmWidth\)' 'Palm collider boxes must use local Y as depth and local Z as cross-palm width.'
Require-Text 'src/physics-interaction/hand/RootFlattenedFingerSkeletonRuntime.cpp' 'rotateNiLocalToWorld\(handNode->world\.rotate,\s*RE::NiPoint3\(0\.0f,\s*-1\.0f,\s*0\.0f\)\)' 'Root-flattened palm landmark normal must use runtime local -Y as the palm-face direction.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)' 'Proxy authority must pass the live palm body frame through the explicit boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)' 'Resolved hidden proxy frame must consume the configured local offset.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*makeGrabStartupCaptureAuthorityFrame' 'Runtime hidden proxy drive must not consume the grab-start raw-rotation capture frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pending\.proxyWorld\s*=\s*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)' 'Per-frame hidden proxy target must keep consuming the configured local offset.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(proxyBaseWorld,\s*_isLeft\)\.translate' 'Pivot-A must use the offset hidden proxy origin derived from the live palm anchor.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabStartupCapturePivotAWorld[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(palmAuthorityBaseWorld,\s*_isLeft\)\.translate' 'Grab-start pivot capture must apply the configured offset through corrected proxy-local palm space.'
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
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'convertNativeBodyReadbackToConventionalObjectFrame|constraintConventionalBodyHandSpace|desiredConventionalBodyWorld' 'Proxy authority must not retain removed conventional angular candidates.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveProxyConstraintAngularDriveTargetWorld\([\s\S]*makeNativeAngularBoundaryTargetFromVisualTarget' 'Proxy angular velocity drive must choose its own angular target instead of always consuming the BODY linear target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyProxyConstraintAngularVelocityDrive\(\s*world,\s*desiredAngularTargetWorld' 'Proxy direct angular velocity must consume the selected angular target, not the BODY linear target unconditionally.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeRawRotationPalmTranslationFrame|rawRotationProxyHandSpace|rawRotationProxyBodyHandSpace|rawRotationPalmTranslation' 'Proxy authority must not keep the old hybrid raw-rotation workaround after the palm Y/Z correction.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.proxyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(proxyFrameWorldAtGrab,\s*desiredObjectWorld\)' 'Proxy authority must capture object relation in corrected proxy-palm space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kGrabObjectRotationReferenceName\s*=\s*"correctedProxyPalm"' 'The corrected proxy-palm authority must be the production rotation reference.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabObjectRotationReferenceMode|grabObjectRotationReferenceModeName|legacyBody|convertedBodyCapture|splitVisualNativeBoundary' 'Proxy authority must not keep the removed selectable rotation modes.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab' 'Proxy constraint creation must use the resolved palm-frame proxy transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformProxySpace\s*=\s*_grabFrame\.proxyBodyHandSpace' 'Proxy constraint creation must seed the ragdoll angular target from the corrected proxy-palm BODY relation used by held updates.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outActivePivotBBodyLocalGame\s*=\s*activeProxyConstraintPivotBLocalGame\(\)' 'Proxy constraint target math must use the frozen solver-local copy of the selected grip pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'transformBTranslation\[0\]\s*=\s*outActivePivotBBodyLocalGame\.x\s*\*\s*gameToHkScale' 'Transform-B must be written from the frozen solver-local selected grip pivot, not a recomputed visual/object local point.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy runtime error and pivot telemetry must read body-B through the BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.proxyHandSpace\)' 'Proxy desired object frame must always compose from the corrected proxy-palm relation.'
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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabPivotAForPrimaryChoice\s*=\s*computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)' 'Close-grab acquisition must use the corrected proxy-local startup capture pivot before body and mesh authority are frozen.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'acquisitionGrabPivotAWorld\s*=\s*computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)' 'Three-phase acquisition must use the same corrected proxy-local startup capture pivot.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'drawGrabAuthorityProxy[\s\S]*getGrabAuthorityProxyDebugSnapshot\(hknp,\s*rawHandWorld,\s*snapshot\)[\s\S]*RightGrabAuthorityProxyTarget' 'Proxy overlay must draw the computed grab-start capture frame before a grab creates a readback proxy body.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' 'enum class\s+AxisOverlayBasis[\s\S]*StoredColumns' 'Proxy/palm overlay must expose a stored-column axis mode for generated collider seating checks.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'entry\.basis\s*==\s*AxisOverlayBasis::StoredColumns[\s\S]*storedColumnAxis\(entry\.transform\.rotate,\s*0\)' 'Generated palm seating axes must draw the authored collider columns, not the generic Ni local-vector view.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryGetPalmAnchorTarget\(RE::NiTransform& outTarget\)\s+const' 'Proxy debug overlay must be able to draw the live generated palm target before relying on body readback.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'addGrabAuthorityAxisReference[\s\S]*tryGetPalmAnchorTarget\(palmAnchorTarget\)[\s\S]*addStoredColumnAxisTransform\([\s\S]*RightGrabPalmGeneratedDirect' 'Proxy overlay must include the generated palm XYZ marker in the same view as the proxy target/readback markers.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_twoHandedGrip\.update\([\s\S]*weaponHandAnchors[\s\S]*frame\.deltaSeconds' 'Two-handed weapon support grip must receive the same frame-level grab authority proxy anchors as normal grab.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'resolveGrabAuthorityProxySeatOrFallback[\s\S]*computeFallbackGrabAuthorityProxySeatWorld' 'Two-handed weapon support grip fallback must use the active proxy-seat offset, not the retired hand-space pivot.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'primaryProxySeatWorld[\s\S]*handAnchors\.rightGrabAuthorityProxySeatWorld[\s\S]*supportProxySeatWorld[\s\S]*handAnchors\.leftGrabAuthorityProxySeatWorld' 'Two-handed weapon grip capture must use corrected right/left proxy-seat anchors.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'primaryController[\s\S]*handAnchors\.rightGrabAuthorityProxySeatWorld[\s\S]*supportController[\s\S]*handAnchors\.leftGrabAuthorityProxySeatWorld' 'Two-handed weapon solver targets must use corrected right/left proxy-seat anchors.'
Reject-Text 'src/RockConfig.h' 'GrabPivotAHandspace' 'Retired grab pivot hand-space config must not remain declared.'
Reject-Text 'src/RockConfig.cpp' 'GrabPivotAHandspace' 'Retired grab pivot hand-space config must not remain read or persisted.'
Reject-Text 'data/config/ROCK.ini' 'GrabPivotAHandspace' 'Packaged INI must expose only the active grab authority proxy offset.'
Reject-Text 'src/physics-interaction/hand/HandFrame.h' 'computeGrabPivotAHandspacePosition|computePalmPositionFromHandBasis|computeGrabPivotAPositionFromHandBasis' 'Retired hand-space pivot helpers must stay deleted.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'computeGrabPivotAPositionFromHandBasis|computePalmPositionFromHandBasis' 'Two-handed weapon support grip must not reintroduce old configured hand-space palm points.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.rawHandSpace\)' 'Proxy body-A convention must not use the raw controller hand-space relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*makeIdentityTransform\(\)' 'Proxy authority must not erase the visual-object to BODY relation; doing so causes immediate angular correction after grab.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*constraintUsesMotionBodyAtGrab\s*\?\s*motionBodyWorldAtGrab\s*:\s*grabBodyWorldAtGrab' 'Proxy authority must not route body-B constraint data through MOTION/COM.'
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

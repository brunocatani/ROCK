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
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryResolveLivePalmAnchorReference[\s\S]*tryGetBodyArrayWorldTransform\(world,\s*_handBody\.getBodyId\(\),\s*livePalmWorld\)[\s\S]*BodyFrameSource::BodyTransform[\s\S]*tryResolveLiveBodyWorldTransform\(world,\s*_handBody\.getBodyId\(\)' 'Live palm authority must prefer the actual BODY origin and use MOTION only as fallback/diagnostic evidence.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::getGrabAuthorityProxyDebugSnapshot\(RE::hknpWorld\* world,\s*const RE::NiTransform& rawHandWorld[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*palmReference\)[\s\S]*makeGrabAuthoritySemanticPalmBaseFrame\(rawHandWorld,\s*palmReference\.world\.translate,\s*_isLeft\)[\s\S]*makeGrabAuthorityProxyFrameFromSemanticPalmPocket\(rawHandWorld,\s*palmReference\.world\.translate,\s*_isLeft\)' 'Idle proxy debug target must show the raw-rotation semantic palm-pocket authority target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*tryResolveLivePalmAnchorReference[\s\S]*livePalmAnchorBodyRawSemanticFrame' 'Proxy authority must resolve from the live palm-anchor translation but publish a raw-rotation semantic frame.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'computeGrabAuthorityProxyOffsetLocalGame[\s\S]*rockLeftGrabAuthorityProxyOffsetGameUnits[\s\S]*rockRightGrabAuthorityProxyOffsetGameUnits' 'Hidden grab authority proxy offset must be a per-hand INI-backed local offset, not a real hand-collider move.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'buildGrabAuthoritySemanticPalmFrame[\s\S]*rawRotationPalmTranslationWorld\s*=\s*rawHandWorld[\s\S]*rawRotationPalmTranslationWorld\.translate\s*=\s*palmAnchorWorld' 'Grab authority must keep palm-anchor translation while using raw hand rotation.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'makeGrabAuthorityProxyFrameFromSemanticPalmPocket[\s\S]*palmDepthWorld[\s\S]*frame\.fingerForwardWorld\s*\*\s*localOffset\.x[\s\S]*palmDepthWorld\s*\*\s*localOffset\.y[\s\S]*frame\.acrossPalmWorld\s*\*\s*localOffset\.z' 'Hidden grab authority proxy offset must be applied in semantic palm-pocket axes: X fingers, Y palm depth, Z across palm.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryComputeGrabRawRollPalmPocketPivotAWorld' 'Raw-roll palm-pocket pivot computation must be an explicit helper so held paths do not reuse generated proxy translation.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'input\.grabAnchorWorld\s*=\s*hand\.computeGrabStartupCapturePivotAWorld\(hknp,\s*input\.rawHandWorld\)' 'Frame debug/selection grab anchor must show the raw-roll palm-pocket startup point.'
Require-Text 'src/RockConfig.h' 'rockRightGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the right-hand hidden proxy offset.'
Require-Text 'src/RockConfig.h' 'rockLeftGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the left-hand hidden proxy offset.'
Require-Text 'src/RockConfig.cpp' 'fRightGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the right-hand hidden proxy offset from INI.'
Require-Text 'src/RockConfig.cpp' 'fLeftGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the left-hand hidden proxy offset from INI.'
Require-Text 'data/config/ROCK.ini' 'fRightGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0[\s\S]*fRightGrabAuthorityProxyOffsetYGameUnits\s*=\s*-2\.0' 'Packaged INI must expose the corrected right-hand hidden proxy palm-seat offset.'
Require-Text 'data/config/ROCK.ini' 'fLeftGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0[\s\S]*fLeftGrabAuthorityProxyOffsetYGameUnits\s*=\s*-2\.0' 'Packaged INI must expose the corrected left-hand hidden proxy palm-seat offset.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'matrixFromAxes[\s\S]*matrix\.entry\[1\]\[0\]\s*=\s*xAxis\.y[\s\S]*matrix\.entry\[0\]\[1\]\s*=\s*yAxis\.x' 'Generated hand/body colliders must keep the in-game verified column-stored native placement convention.'
Reject-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab authority proxy must not transpose the generated palm frame; in-game proxy isolation proved that convention is wrong.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'generatedColliderFrameToGrabAuthorityFrame\(palmReference\.world\)' 'Proxy authority must pass the live palm body frame through the explicit boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*makeGrabAuthorityProxyFrameFromSemanticPalmPocket\(rawHandWorld,\s*proxyBaseWorld\.translate,\s*_isLeft\)' 'Resolved hidden proxy frame must consume the configured local offset through raw-rotation semantic palm-pocket axes.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*pending\.proxyWorld\.rotate\s*=\s*proxyBaseWorld\.rotate|resolveGrabAuthorityProxyFrame[\s\S]*palmReference\.world\.rotate' 'Runtime hidden proxy drive must not publish generated palm rotation as authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rawSemanticProxyWorld\s*=\s*makeGrabAuthorityProxyFrameFromSemanticPalmPocket\(pending\.rawHandWorld,\s*proxyBaseWorld\.translate,\s*_isLeft\)[\s\S]*pending\.proxyWorld\.rotate\s*=\s*rawSemanticProxyWorld\.rotate[\s\S]*resolveActiveGrabAuthorityPivotAWorld\(world,\s*pending\.rawHandWorld,\s*activeProxyPivotAWorld\)[\s\S]*pending\.proxyWorld\.translate\s*=\s*activeProxyPivotAWorld' 'Physics flush must preserve raw hand rotation and the active raw-roll/pinch pivot translation instead of overwriting the proxy target with generated palm rotation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(world,\s*handWorldTransform,\s*&handBodyWorldAtGrab,\s*proxyFrameWorldAtGrab[\s\S]*grabAuthorityPivotAWorld\s*=\s*proxyFrameWorldAtGrab\.translate[\s\S]*palmPocketPivotAWorld\s*=\s*computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)[\s\S]*grabPivotAForPrimaryChoice\s*=\s*palmPocketPivotAWorld' 'Grab commit must resolve the live proxy frame, then use the raw-roll palm-pocket startup pivot for mesh/contact evidence selection.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'The hidden proxy is body A[\s\S]*RE::NiPoint3\s+grabPivotAWorld\s*=\s*palmPocketPivotAWorld[\s\S]*proxyFrameWorldAtGrab\s*=\s*makeProxyFrameWithPivotOrigin\(proxyFrameWorldAtGrab,\s*grabPivotAWorld\)[\s\S]*buildSplitGrabFrameFromDesiredObject\(\s*handWorldTransform,\s*proxyFrameWorldAtGrab[\s\S]*createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*handWorldTransform,\s*grabPivotAWorld' 'Committed body-A proxy must move to the raw-roll pocket point so constraint pivot A remains local zero.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmPocketPivotAWorld\s*=\s*computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)' 'Close grab palm-pocket acquisition must seed from the startup raw-rotation capture point.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*tryComputeGrabRawRollPalmPocketPivotAWorld\(world,\s*fallbackHandWorldTransform,\s*rawSemanticPalmPocketPivotWorld\)[\s\S]*return\s+rawSemanticPalmPocketPivotWorld' 'Pivot-A must delegate to the raw-rotation semantic palm-pocket helper.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryComputeGrabRawRollPalmPocketPivotAWorld[\s\S]*makeGrabAuthorityProxyFrameFromSemanticPalmPocket\(\s*rawHandWorldTransform,\s*palmReference\.world\.translate,\s*_isLeft\)[\s\S]*outPivotWorld\s*=\s*proxyFrameWorld\.translate' 'Raw-roll palm-pocket pivot capture must apply the configured offset through semantic palm-pocket axes.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabStartupCapturePivotAWorld[\s\S]*tryComputeGrabRawRollPalmPocketPivotAWorld\(world,\s*rawHandWorldTransform,\s*rawRollPalmPocketPivotWorld\)' 'Grab-start pivot capture must delegate to the shared raw-roll palm-pocket helper.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rootFlattenedPalmAnchorTarget' 'Proxy authority must not use the sampled palm target as the active body-A frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rawHandFallback' 'Proxy authority must not silently fall back to raw controller space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'proxyFrameWorldAtGrab\s*=\s*makeProxyFrameWithPivotOrigin\(proxyFrameWorldAtGrab,\s*grabPivotAWorld\)[\s\S]*buildSplitGrabFrameFromDesiredObject\(\s*handWorldTransform,\s*proxyFrameWorldAtGrab' 'Constraint hand-space frame must be captured from the active pocket-origin proxy frame, not live stale hknp readback.'
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
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveProxyConstraintAngularDriveTargetWorld|makeNativeAngularBoundaryTargetFromVisualTarget|applyProxyConstraintAngularVelocityDrive' 'Proxy authority must not retain the removed direct angular velocity drive.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'setGrabMotorAtomsActive\(header,\s*true,\s*true\)' 'Proxy angular authority must always enable the ragdoll atom motor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeRawRotationPalmTranslationFrame\(handWorldTransform,\s*proxyFrameWorldAtGrab\)' 'Proxy authority must have a hybrid frame that keeps palm translation while using raw hand rotation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.rawRotationProxyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(rawRotationProxyFrameWorldAtGrab,\s*desiredObjectWorld\)' 'Proxy authority must capture object relation in the hybrid raw-rotation/palm-translation frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kGrabObjectRotationReferenceName\s*=\s*"rawRotationPalmTranslation"' 'The validated raw-rotation/palm-translation authority must be the only production rotation reference.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabObjectRotationReferenceMode|grabObjectRotationReferenceModeName|legacyBody|convertedBodyCapture|splitVisualNativeBoundary' 'Proxy authority must not keep the removed selectable rotation modes.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab' 'Proxy constraint creation must use the resolved palm-frame proxy transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const RE::NiPoint3\s+pivotAProxyLocalGame\{\};[\s\S]*constraintPivotAWorld\s*=\s*proxyWorldTransform\.translate[\s\S]*createGrabConstraint\(world,[\s\S]*proxyWorldTransform,[\s\S]*constraintPivotAWorld' 'Proxy constraint creation must keep body-A pivot local zero and move the proxy origin to the active grab pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformProxySpace\s*=\s*_grabFrame\.rawRotationProxyBodyHandSpace' 'Proxy constraint creation must seed the ragdoll angular target from the same raw-rotation proxy BODY relation used by held updates.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive[\s\S]*desiredBodyTransformProxySpace\s*=\s*_grabFrame\.constraintBodyHandSpace' 'Proxy constraint creation must not seed the ragdoll angular target from the older constraint-space relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outActivePivotBBodyLocalGame\s*=\s*activeProxyConstraintPivotBLocalGame\(\)' 'Proxy constraint target math must use the frozen solver-local copy of the selected grip pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'transformBTranslation\[0\]\s*=\s*outActivePivotBBodyLocalGame\.x\s*\*\s*gameToHkScale' 'Transform-B must be written from the frozen solver-local selected grip pivot, not a recomputed visual/object local point.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy runtime error and pivot telemetry must read body-B through the BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(authorityFrame,\s*_grabFrame\.rawRotationProxyHandSpace\)' 'Proxy desired object frame must always compose from the validated hybrid raw-rotation/palm-translation relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveActiveGrabAuthorityPivotAWorld\(world,\s*handWorldTransform,\s*activeProxyPivotAWorld\)[\s\S]*proxyAuthorityWorld\s*=\s*makeProxyFrameWithPivotOrigin\(proxyAuthorityWorld,\s*activeProxyPivotAWorld\)[\s\S]*queueProxyGrabAuthorityTarget\(\s*proxyAuthorityWorld' 'Held updates must queue the active raw-roll/pinch pivot as the hidden proxy origin.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueGeneratedKeyframedBodyTarget\(\s*_grabAuthorityProxyDriveState,\s*pending\.proxyWorld' 'Proxy authority must queue the active pocket-origin proxy frame through the generated keyframed-body drive path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveGeneratedKeyframedBody\([\s\S]*_grabAuthorityProxy[\s\S]*"grab-authority-proxy"' 'Proxy authority must move with the same native generated-body drive path as the actual palm collider.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryResolveLiveBodyWorldTransform\(world,\s*proxyBodyId,\s*proxyReadbackBetween' 'Proxy readback telemetry must inspect the same live frame shown by the visual debug overlay.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeHardKeyframeVelocityForTarget\(\s*world,\s*proxyBodyId,\s*pending\.proxyWorld,\s*driveDelta,[\s\S]*angularVelocityHavok\)' 'Proxy authority may keep FO4VR hard-keyframe velocity as diagnostic fallback when palm motion velocity is unavailable.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'ComputeHardKeyFrame_t[\s\S]*offsets::kFunc_ComputeHardKeyFrame' 'Proxy telemetry may use FO4VR native hard-keyframe velocity when palm motion velocity is unavailable.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationColumns\(transformBRotation,\s*bodyToHandRotation\)' 'FO4VR custom grab angular setup must write transform-B as Havok column blocks, matching HIGGS hkMatrix storage.'
Require-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationRows\(targetBRca,\s*bodyToHandRotation\)' 'FO4VR custom grab angular setup must write target_bRca in the solver-row view so top grabs see the inverse body-to-hand relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveQueuedTargets\s*=\s*_grabAuthorityProxyQueuedSequence' 'Held telemetry must report proxy queue counters when proxy authority is active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveFlushedTargets\s*=\s*_grabAuthorityProxyFlushSequence' 'Held telemetry must report proxy flush counters when proxy authority is active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabPivotAForPrimaryChoice\s*=\s*palmPocketPivotAWorld' 'Close-grab primary body choice must use the raw-roll palm-pocket acquisition pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'acquisitionGrabPivotAWorld\s*=\s*palmPocketPivotAWorld' 'Three-phase acquisition must use the same raw-roll palm-pocket acquisition pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildGrabPocketFrameWithPalmCenter\(\s*handWorldTransform,\s*_isLeft,\s*acquisitionGrabPivotAWorld' 'Palm-pocket mesh acquisition must be built from the resolved proxy-origin acquisition pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimeGrabContactPatch\([\s\S]*acquisitionGrabPivotAWorld[\s\S]*canonicalPivotPointWorld' 'Contact patch candidate scoring must consume the resolved proxy-origin acquisition pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'fingerEvidenceSurfaceHit\.pivotToSurfaceDistanceGameUnits\s*=\s*pointDistanceGameUnits\(palmPocketPivotAWorld,\s*fingerEvidencePointWorld\)' 'Finger evidence telemetry must measure against the raw-roll palm-pocket acquisition pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmTangentWorld\s*=\s*acquisitionPocket\.fingerForwardWorld[\s\S]*palmBitangentWorld\s*=\s*acquisitionPocket\.thumbSideWorld' 'Contact-patch probing must share the projected raw-roll palm-pocket basis instead of rebuilding a mismatched tangent frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'livePivotAWorld\s*=\s*proxyAuthorityWorld\.translate' 'Seated palm-pocket reacquire must not fall back to the generated proxy origin as the pocket pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryComputeGrabRawRollPalmPocketPivotAWorld\(world,\s*handWorldTransform,\s*livePivotAWorld\)[\s\S]*buildGrabPocketFrameWithPalmCenter\([\s\S]*livePivotAWorld[\s\S]*findSeatedGrabPivotNearPalmPocket\([\s\S]*livePivotAWorld' 'Seated palm-pocket reacquire must search from the current raw-roll palm-pocket pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryComputeGrabRawRollPalmPocketPivotAWorld\(world,\s*handWorldTransform,\s*supportPivotAWorld\)[\s\S]*findSeatedGrabPivotNearPalmPocket\([\s\S]*supportPivotAWorld' 'Held support refresh must search from the current raw-roll palm-pocket pivot.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'drawGrabAuthorityProxy[\s\S]*getGrabAuthorityProxyDebugSnapshot\(hknp,\s*rawHandWorld,\s*snapshot\)[\s\S]*RightGrabAuthorityProxyTarget' 'Proxy overlay must draw the computed grab-start capture frame before a grab creates a readback proxy body.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' 'enum class\s+AxisOverlayBasis[\s\S]*StoredColumns' 'Proxy/palm overlay must expose a stored-column axis mode for generated collider seating checks.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'entry\.basis\s*==\s*AxisOverlayBasis::StoredColumns[\s\S]*storedColumnAxis\(entry\.transform\.rotate,\s*0\)' 'Generated palm seating axes must draw the authored collider columns, not the generic Ni local-vector view.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryGetPalmAnchorTarget\(RE::NiTransform& outTarget\)\s+const' 'Proxy debug overlay must be able to draw the live generated palm target before relying on body readback.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'addGrabAuthorityAxisReference[\s\S]*tryGetPalmAnchorTarget\(palmAnchorTarget\)[\s\S]*addStoredColumnAxisTransform\([\s\S]*RightGrabPalmGeneratedDirect' 'Proxy overlay must include the generated palm XYZ marker in the same view as the proxy target/readback markers.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.rawHandSpace\)' 'Proxy body-A convention must not use the raw controller hand-space relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*makeIdentityTransform\(\)' 'Proxy authority must not erase the visual-object to BODY relation; doing so causes immediate angular correction after grab.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*constraintUsesMotionBodyAtGrab\s*\?\s*motionBodyWorldAtGrab\s*:\s*grabBodyWorldAtGrab' 'Proxy authority must not route body-B constraint data through MOTION/COM.'
Reject-Text 'src/physics-interaction/grab/GrabConstraintMath.h' 'writeHavokRotationRows\(transformBRotation' 'Proxy angular setup must keep transform-B in Havok column storage; only target_bRca uses the solver-row convention.'
Reject-Text 'src/physics-interaction/grab/GrabAuthorityProxyMotion.h' 'computeAngularVelocityRadiansPerSecond|matrixColumn|axisSum' 'The old manual matrix-column proxy angular helper must stay removed; proxy angular telemetry uses the FO4VR hard-keyframe helper only as fallback.'
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

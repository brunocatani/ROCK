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
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::getGrabAuthorityProxyDebugSnapshot\(RE::hknpWorld\* world,\s*const RE::NiTransform& rawHandWorld[\s\S]*tryResolveLivePalmAnchorReference\(world,\s*palmReference\)[\s\S]*makePalmAnchorGrabAuthorityBaseFrame\(palmReference\.world\)[\s\S]*makeGrabStartupCaptureAuthorityFrame\(rawHandWorld,\s*palmAuthorityBaseWorld\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(startupCaptureFrameWorld,\s*_isLeft\)' 'Idle proxy debug target must show the raw-hand grab-start capture offset without changing the runtime proxy drive frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*tryResolveLivePalmAnchorReference[\s\S]*livePalmAnchorMotionGrabFrame' 'Proxy authority must resolve from the live palm-anchor body frame.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'computeGrabAuthorityProxyOffsetLocalGame[\s\S]*rockLeftGrabAuthorityProxyOffsetGameUnits[\s\S]*rockRightGrabAuthorityProxyOffsetGameUnits' 'Hidden grab authority proxy offset must be a per-hand INI-backed local offset, not a real hand-collider move.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'makeGrabStartupCaptureAuthorityFrame[\s\S]*result\.rotate\s*=\s*rawHandWorld\.rotate[\s\S]*result\.scale\s*=\s*rawHandWorld\.scale[\s\S]*return\s+result' 'Grab-start capture must keep the generated palm seat point while using raw hand rotation for angular authority.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'applyGrabAuthorityProxyLocalOffsetToFrame[\s\S]*localVectorToWorld\(proxyFrameWorld,\s*localOffset\)' 'Hidden grab authority proxy offset must be applied in proxy local space.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'applyRuntimeGrabAuthorityProxyOffsetToFrame[\s\S]*runtimeAuthorityFrame\.rotate\s*=\s*rawHandWorld\.rotate[\s\S]*runtimeAuthorityFrame\.scale\s*=\s*rawHandWorld\.scale[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(runtimeAuthorityFrame,\s*isLeft\)' 'Runtime proxy offset must attach raw hand rotation before applying the hidden seat offset.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryComputeGrabRawRollPalmPocketPivotAWorld' 'Raw-roll palm-pocket pivot computation must be an explicit helper so held paths do not reuse generated proxy translation.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'input\.grabAnchorWorld\s*=\s*hand\.computeGrabStartupCapturePivotAWorld\(hknp,\s*input\.rawHandWorld\)' 'Frame debug/selection grab anchor must show the raw-roll palm-pocket startup point.'
Require-Text 'src/RockConfig.h' 'rockRightGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the right-hand hidden proxy offset.'
Require-Text 'src/RockConfig.h' 'rockLeftGrabAuthorityProxyOffsetGameUnits' 'RockConfig must expose the left-hand hidden proxy offset.'
Require-Text 'src/RockConfig.cpp' 'fRightGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the right-hand hidden proxy offset from INI.'
Require-Text 'src/RockConfig.cpp' 'fLeftGrabAuthorityProxyOffsetXGameUnits' 'RockConfig must read the left-hand hidden proxy offset from INI.'
Require-Text 'data/config/ROCK.ini' 'fRightGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0[\s\S]*fRightGrabAuthorityProxyOffsetYGameUnits\s*=\s*-2\.0' 'Packaged INI must expose the corrected right-hand hidden proxy palm-seat offset.'
Require-Text 'data/config/ROCK.ini' 'fLeftGrabAuthorityProxyOffsetXGameUnits\s*=\s*0\.0[\s\S]*fLeftGrabAuthorityProxyOffsetYGameUnits\s*=\s*-2\.0' 'Packaged INI must expose the corrected left-hand hidden proxy palm-seat offset.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'matrixFromAxes[\s\S]*matrix\.entry\[1\]\[0\]\s*=\s*xAxis\.y[\s\S]*matrix\.entry\[0\]\[1\]\s*=\s*yAxis\.x' 'Generated hand/body colliders must keep the in-game verified column-stored native placement convention.'
Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderFrameToGrabAuthorityFrame[\s\S]*transposeStoredRotation\(colliderFrame\.rotate\)' 'Grab authority proxy must adapt generated palm collider columns into corrected palm proxy row-basis authority.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'makePalmAnchorGrabAuthorityBaseFrame' 'Proxy authority must have one explicit boundary that adapts the generated palm base before raw hand rotation is attached.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*makePalmAnchorGrabAuthorityBaseFrame\(palmReference\.world\)[\s\S]*applyRuntimeGrabAuthorityProxyOffsetToFrame\(proxyBaseWorld,\s*rawHandWorld,\s*_isLeft\)' 'Resolved hidden proxy frame must keep the live palm base origin while applying raw hand rotation and local offset.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*makeGrabStartupCaptureAuthorityFrame' 'Runtime hidden proxy drive must not consume the grab-start raw-rotation capture frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pending\.proxyWorld\s*=\s*applyRuntimeGrabAuthorityProxyOffsetToFrame\(proxyBaseWorld,\s*pending\.rawHandWorld,\s*_isLeft\)' 'Physics flush must keep the resolved runtime offset proxy frame instead of rebinding the proxy origin to pivot A.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(world,\s*handWorldTransform,\s*&handBodyWorldAtGrab,\s*proxyFrameWorldAtGrab[\s\S]*grabAuthorityPivotAWorld\s*=\s*proxyFrameWorldAtGrab\.translate[\s\S]*palmPocketPivotAWorld\s*=\s*computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)[\s\S]*grabPivotAForPrimaryChoice\s*=\s*palmPocketPivotAWorld' 'Grab commit must resolve the live proxy frame, then use the raw-roll palm-pocket startup pivot for mesh/contact evidence selection.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'The hidden proxy is body A[\s\S]*RE::NiPoint3\s+grabPivotAWorld\s*=\s*palmPocketPivotAWorld[\s\S]*freezeGrabAuthorityFrame<RE::NiTransform>[\s\S]*\.proxyWorld\s*=\s*proxyFrameWorldAtGrab[\s\S]*\.pivotAWorld\s*=\s*grabPivotAWorld[\s\S]*applyFrozenGrabAuthorityFrameToGrabFrame\(_grabFrame,\s*frozenAuthorityFrame\)[\s\S]*createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*handWorldTransform,\s*grabPivotAWorld' 'Committed body-A proxy must keep the resolved proxy frame, freeze pivot A separately, and create the constraint from that coherent body-local authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmPocketPivotAWorld\s*=\s*computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)' 'Close grab palm-pocket acquisition must seed from the startup raw-hand palm proxy capture point.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabPivotAWorld[\s\S]*makePalmAnchorGrabAuthorityBaseFrame\(palmReference\.world\)[\s\S]*applyRuntimeGrabAuthorityProxyOffsetToFrame\(proxyBaseWorld,\s*fallbackHandWorldTransform,\s*_isLeft\)\.translate' 'Pivot-A must use the live palm anchor base origin while routing runtime rotation and offset through the raw hand authority frame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::tryComputeGrabRawRollPalmPocketPivotAWorld[\s\S]*makePalmAnchorGrabAuthorityBaseFrame\(palmReference\.world\)[\s\S]*makeGrabStartupCaptureAuthorityFrame\(rawHandWorldTransform,\s*palmAuthorityBaseWorld\)[\s\S]*applyGrabAuthorityProxyLocalOffsetToFrame\(startupCaptureFrameWorld,\s*_isLeft\)\.translate' 'Palm-pocket pivot capture must apply the configured offset through the raw-rotation palm authority frame.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Hand::computeGrabStartupCapturePivotAWorld[\s\S]*tryComputeGrabRawRollPalmPocketPivotAWorld\(world,\s*rawHandWorldTransform,\s*rawRollPalmPocketPivotWorld\)' 'Grab-start pivot capture must delegate to the shared raw-roll palm-pocket helper.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rootFlattenedPalmAnchorTarget' 'Proxy authority must not use the sampled palm target as the active body-A frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rawHandFallback' 'Proxy authority must not silently fall back to raw controller space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'freezeGrabAuthorityFrame<RE::NiTransform>[\s\S]*\.proxyWorld\s*=\s*proxyFrameWorldAtGrab[\s\S]*\.pivotAWorld\s*=\s*grabPivotAWorld' 'Grab-start hand/object relation must freeze the resolved proxy frame and the selected pivot A as separate pieces of the authority contract.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*grabBodyWorldAtGrab\)' 'Proxy authority must preserve the visual-object to hknp BODY relation captured at grab time.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintUsesMotionBodyAtGrab\s*=\s*false' 'Proxy authority must keep hknp constraint body-B data in the rigid BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*grabBodyWorldAtGrab' 'Proxy authority must encode transform-B and angular target data against the BODY frame, not MOTION/COM.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToConstraintBodyAtGrab' 'Proxy authority cleanup must remove the stale solver-local object relation capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyFrozenGrabAuthorityFrameToGrabFrame[\s\S]*frame\.bodyLocal\s*=\s*frozen\.bodyLocal' 'Proxy authority must store the captured visual-object to BODY relation through the frozen authority frame instead of assuming BODY equals the visible node.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintBodyLocal' 'Proxy authority cleanup must remove the retired duplicate solver-local BODY relation.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'desiredBodyWorld\s*=\s*transform_math::composeTransforms\(frozen\.desiredObjectWorld,\s*frozen\.bodyLocal\)' 'Proxy authority must derive the desired BODY frame from the frozen visual-object to BODY relation instead of treating visual local coordinates as body local.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredConstraintBodyWorld' 'Proxy authority cleanup must remove the retired constraint-space desired BODY target.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.(constraintHandSpace|constraintBodyHandSpace)' 'Proxy authority cleanup must remove retired constraint-space capture fields.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'convertNativeBodyReadbackToConventionalObjectFrame|constraintConventionalBodyHandSpace|desiredConventionalBodyWorld' 'Proxy authority must not retain removed conventional angular candidates.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveProxyConstraintAngularDriveTargetWorld|makeNativeAngularBoundaryTargetFromVisualTarget|applyProxyConstraintAngularVelocityDrive' 'Proxy authority must not retain the removed direct angular velocity drive.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'setGrabMotorAtomsActive\(header,\s*true,\s*true\)' 'Proxy angular authority must always enable the ragdoll atom motor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeRawHandPalmProxyAuthorityFrame\(proxyFrameWorldAtGrab\)' 'Proxy authority must freeze object and BODY relations through the raw-hand palm proxy frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyFrozenGrabAuthorityFrameToGrabFrame[\s\S]*frame\.rawRotationProxyHandSpace\s*=\s*frozen\.rawRotationProxyHandSpace' 'Proxy authority must capture object relation in the raw-hand palm proxy frame through the frozen authority contract.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyFrozenGrabAuthorityFrameToGrabFrame[\s\S]*frame\.rawRotationProxyBodyHandSpace\s*=\s*frozen\.rawRotationProxyBodyHandSpace' 'Proxy authority must capture BODY relation in the same raw-hand palm proxy frame used by the live angular target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kGrabObjectRotationReferenceName\s*=\s*"rawHandPalmProxy"' 'The raw-hand palm proxy authority must be the only production rotation reference.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabObjectRotationReferenceMode|grabObjectRotationReferenceModeName|legacyBody|convertedBodyCapture|splitVisualNativeBoundary' 'Proxy authority must not keep the removed selectable rotation modes.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab' 'Proxy constraint creation must use the resolved palm-frame proxy transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintPivotAWorld\s*=\s*grabPivotAWorld[\s\S]*pivotAProxyLocalGame\s*=\s*grab_constraint_math::computeConstraintPivotLocalGame\(proxyWorldTransform,\s*constraintPivotAWorld\)[\s\S]*createGrabConstraint\(world,[\s\S]*proxyWorldTransform,[\s\S]*constraintPivotAWorld' 'Proxy constraint creation must keep the proxy body at its offset frame and encode pivot A as an explicit local point on body A.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformProxySpace\s*=\s*_grabFrame\.rawRotationProxyBodyHandSpace' 'Proxy constraint creation must seed the ragdoll angular target from the same raw-hand proxy BODY relation used by held updates.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive[\s\S]*desiredBodyTransformProxySpace\s*=\s*_grabFrame\.constraintBodyHandSpace' 'Proxy constraint creation must not seed the ragdoll angular target from the older constraint-space relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outActivePivotBBodyLocalGame\s*=\s*activeProxyConstraintPivotBLocalGame\(\)' 'Proxy constraint target math must use the frozen solver-local copy of the selected grip pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'transformBTranslation\[0\]\s*=\s*outActivePivotBBodyLocalGame\.x\s*\*\s*gameToHkScale' 'Transform-B must be written from the frozen solver-local selected grip pivot, not a recomputed visual/object local point.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy runtime error and pivot telemetry must read body-B through the BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(authorityFrame,\s*_grabFrame\.rawRotationProxyHandSpace\)' 'Proxy desired object frame must always compose from the validated raw-hand palm proxy relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(world,\s*handWorldTransform,\s*nullptr,\s*proxyAuthorityWorld,\s*proxyAuthoritySource\)[\s\S]*queueProxyGrabAuthorityTarget\(\s*proxyAuthorityWorld' 'Held updates must queue the resolved hidden proxy frame directly instead of rebinding the proxy origin to pivot A.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueGeneratedKeyframedBodyTarget\(\s*_grabAuthorityProxyDriveState,\s*pending\.proxyWorld' 'Proxy authority must queue the active pocket-origin proxy frame through the generated keyframed-body drive path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveGeneratedKeyframedBody\([\s\S]*_grabAuthorityProxy[\s\S]*"grab-authority-proxy"' 'Proxy authority must move with the same native generated-body drive path as the actual palm collider.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryResolveLiveBodyWorldTransform\(world,\s*proxyBodyId,\s*proxyReadbackBetween' 'Proxy readback telemetry must inspect the same live frame shown by the visual debug overlay.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'observeCustomGrabAuthorityAfterSolve[\s\S]*const bool debugGrabFrameLogging[\s\S]*const bool shouldSampleForAnomaly[\s\S]*if \(!debugGrabFrameLogging && !shouldSampleForAnomaly\)' 'Proxy after-solve anomaly sampling must not require verbose grab-frame logging.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeGrabPalmBasisDelta[\s\S]*xAxisDegrees[\s\S]*yAxisDegrees[\s\S]*zAxisDegrees[\s\S]*rawDeterminant[\s\S]*proxyDeterminant' 'Proxy telemetry must expose raw-hand to generated-palm per-axis basis deltas and handedness determinants.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'PROXY GRAB PALM BASIS MISMATCH[\s\S]*axisDeg=\(\{:\.1f\},\{:\.1f\},\{:\.1f\}\)[\s\S]*determinant=\(\{:\.3f\},\{:\.3f\}\)' 'Grab-start palm basis telemetry must report per-axis raw/proxy deltas before constraint creation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'PROXY GRAB FRAME MISMATCH[\s\S]*rawToProxyTarget[\s\S]*targetAxisDeg[\s\S]*liveAxisDeg[\s\S]*determinantTarget[\s\S]*targetRowsInv[\s\S]*targetColsToTransformB' 'Proxy after-solve telemetry must expose the raw/proxy frame mismatch, palm-basis axes, and constraint-frame invariants without relying on verbose grab logging.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'RAGDOLL ANGULAR PROBE[\s\S]*beforeErr[\s\S]*afterErr[\s\S]*reduce[\s\S]*axisDot[\s\S]*forceA[\s\S]*targetRowsInv[\s\S]*gripBefore[\s\S]*pivotLever' 'Proxy after-solve telemetry must prove whether the ragdoll angular atom reduces body rotation error independently from the linear pivot.'
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
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'supportPivotAWorld|refreshHeldAuthoritySupport|evaluateHeldSupportRefresh' 'Held support refresh must stay removed; TouchHeld authority is frozen until release.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'drawGrabAuthorityProxy[\s\S]*getGrabAuthorityProxyDebugSnapshot\(hknp,\s*rawHandWorld,\s*snapshot\)[\s\S]*RightGrabAuthorityProxyTarget' 'Proxy overlay must draw the computed grab-start capture frame before a grab creates a readback proxy body.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' 'enum class\s+AxisOverlayBasis[\s\S]*StoredColumns' 'Proxy/palm overlay must expose a stored-column axis mode for generated collider seating checks.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'entry\.basis\s*==\s*AxisOverlayBasis::StoredColumns[\s\S]*storedColumnAxis\(entry\.transform\.rotate,\s*0\)' 'Generated palm seating axes must draw the authored collider columns, not the generic Ni local-vector view.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryGetPalmAnchorTarget\(RE::NiTransform& outTarget\)\s+const' 'Proxy debug overlay must be able to draw the live generated palm target before relying on body readback.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'addGrabAuthorityAxisReference[\s\S]*tryGetPalmAnchorTarget\(palmAnchorTarget\)[\s\S]*addStoredColumnAxisTransform\([\s\S]*RightGrabPalmGeneratedDirect' 'Proxy overlay must include the generated palm XYZ marker in the same view as the proxy target/readback markers.'
Require-Text 'src/RockConfig.h' 'rockDebugDrawGrabProxySemanticAxesOnly' 'RockConfig must expose a narrow generated-proxy/semantic-axis debug mode.'
Require-Text 'src/RockConfig.cpp' 'bDebugDrawGrabProxySemanticAxesOnly' 'RockConfig must read the narrow generated-proxy/semantic-axis debug mode from INI.'
Require-Text 'data/config/ROCK.ini' 'bDebugDrawGrabProxySemanticAxesOnly\s*=\s*false' 'Packaged INI must expose the narrow generated-proxy/semantic-axis overlay disabled by default.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' 'RightGrabSemanticHandFrame[\s\S]*LeftGrabSemanticHandFrame' 'Debug axis roles must include a dedicated semantic hand frame for proxy-axis parity checks.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'drawGrabProxySemanticAxesOnly[\s\S]*makeSemanticGripFrame[\s\S]*buildGrabPocketFrameWithPalmCenter[\s\S]*semanticPalmDepthWorld[\s\S]*semanticCrossPalmWorld[\s\S]*matrixFromAxes<RE::NiMatrix3>\([\s\S]*semanticPocket\.fingerForwardWorld[\s\S]*semanticPalmDepthWorld[\s\S]*semanticCrossPalmWorld' 'Narrow proxy semantic overlay must build its semantic frame in corrected palm proxy space: X=fingers, Y=palm depth, Z=cross-palm.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'addProxySemanticAxes[\s\S]*tryGetPalmAnchorTarget\(generatedPalmWorld\)[\s\S]*generatedColliderFrameToGrabAuthorityFrame\(generatedPalmWorld\)[\s\S]*applyRuntimeGrabAuthorityProxyOffsetToFrame\(generatedAuthorityBaseWorld,\s*rawHandWorld,\s*isLeft\)[\s\S]*RightGrabPalmGeneratedDirect[\s\S]*RightGrabSemanticHandFrame' 'Narrow proxy semantic overlay must draw generated palm, raw-hand proxy authority, and semantic grip axes together.'
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

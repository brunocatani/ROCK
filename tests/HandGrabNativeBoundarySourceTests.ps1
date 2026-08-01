param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Path {
    param(
        [string]$Path,
        [string]$Message
    )

    if (Test-Path -LiteralPath (Join-Path $Root $Path)) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_NativeVRGrabDrop\s*=\s*0xF1AB90' 'Native VR drop offset must remain explicit at the verified address.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'typedef void func_t\(void\*,\s*int,\s*std::uint64_t\)' 'Native VR drop wrapper must expose the verified third flag argument.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'func\(playerChar,\s*handIndex,\s*0\)' 'Native VR drop wrapper must pass the game-observed third flag value 0.'

Reject-Text 'src/physics-interaction/native/HavokOffsets.h' 'MouseSpring' 'Mouse-spring offsets and tuning constants must not remain available to production grab code.'
Reject-Path 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'Native mouse-spring wrapper implementation must be removed.'
Reject-Path 'src/physics-interaction/native/NativeMouseSpringGrab.h' 'Native mouse-spring wrapper header must be removed.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(\s*world,\s*handWorldTransform,\s*&handBodyWorldAtGrab,\s*proxyFrameWorldAtGrab,[\s\S]*GrabAuthorityProxyFramePolicy::LivePalmOnly\)[\s\S]*grabAuthorityPivotAWorld\s*=\s*proxyFrameWorldAtGrab\.translate[\s\S]*palmPocketPivotAWorld\s*=\s*proxyFrameWorldAtGrab\.translate[\s\S]*grabPivotAForPrimaryChoice\s*=\s*palmPocketPivotAWorld' 'Close dynamic grab must resolve the hidden proxy frame before generated/proxy-local palm-pocket mesh/contact acquisition.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'The hidden proxy is body A[\s\S]*RE::NiPoint3\s+grabPivotAWorld\s*=\s*palmPocketPivotAWorld[\s\S]*freezeGrabAuthorityFrame<RE::NiTransform>[\s\S]*\.proxyWorld\s*=\s*proxyFrameWorldAtGrab[\s\S]*\.pivotAWorld\s*=\s*grabPivotAWorld[\s\S]*applyFrozenGrabAuthorityFrameToGrabFrame\(_grabFrame,\s*frozenAuthorityFrame\)[\s\S]*createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*handWorldTransform,\s*grabPivotAWorld' 'Close dynamic grab must keep the hidden proxy at its seat frame, freeze pivot A separately, and create one coherent body-local authority frame before constraint creation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmPocketPivotAWorld\s*=\s*proxyFrameWorldAtGrab\.translate' 'Close dynamic grab post-prep authority must use the already resolved generated/proxy startup pivot only for the palm-pocket point.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'tryComputeGrabProxyLocalPalmPocketPivotAWorld' 'Held-time palm-pocket support must share an explicit generated/proxy-local pivot helper.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'livePivotAWorld\s*=\s*proxyAuthorityWorld\.translate' 'Seated palm-pocket promotion must not use the generated proxy origin as the pocket pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryComputeGrabProxyLocalPalmPocketPivotAWorld\(world,\s*livePivotAWorld\)[\s\S]*_grabAuthorityPivotAProxyLocalGame\s*=\s*frozenSeatAuthorityFrame\.pivotAHandBodyLocalGame' 'Seated palm-pocket promotion must keep the promoted pivot A as an explicit local point on the hidden proxy body.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'computePivotAHandBodyLocal[\s\S]*generatedColliderWorldPointToLocal\(handBodyWorld,\s*grabPivotWorld\)' 'Frozen pivot-A proxy local capture must use generated stored-column local space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*handWorldTransform,\s*grabPivotAWorld' 'Close dynamic grab must create the hidden proxy plus custom finite-force constraint from the resolved generated/proxy palm-authority frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrab\.create\(\s*world,\s*objectBodyId' 'Ordinary dynamic close grab must not create native mouse-spring as its production authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'solveAdaptiveHeldLead|nativeTargetBodyWorld|_nativeGrab\.queueTarget' 'Held-object updates must not keep removed native mouse-spring adaptive target logic.'
Reject-Text 'src/physics-interaction/grab/GrabHeldObject.h' 'AdaptiveHeldLead|solveAdaptiveHeldLead|responseFactor' 'Removed native/adaptive target-leading helpers must not remain as unused grab authority scaffolding.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(\s*world,\s*handWorldTransform,\s*nullptr,\s*proxyAuthorityWorld,\s*proxyAuthoritySource,\s*GrabAuthorityProxyFramePolicy::PreferQueuedPalmTarget\)[\s\S]*makeGeneratedProxyAuthorityRelationFrame\(proxyAuthorityWorld\)' 'Held-object targets must use generated/proxy authority rotation and the resolved hidden proxy frame without rebinding its origin to pivot A.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveActiveGrabAuthorityPivotAWorld\(\s*const RE::NiTransform& proxyWorldTransform[\s\S]*generatedProxyLocalPointToWorld\(proxyWorldTransform,\s*_grabFrame\.pivotAHandBodyLocalGame\)' 'Held-object transform-A pivot must replay the frozen generated/proxy local pivot.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pinchPivotRawHandLocal|localPointToWorld\(rawHandWorldTransform' 'Pinch-pocket held pivot replay must not use raw hand local space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activePivotBBodyLocalGame\s*=\s*activeProxyConstraintPivotBLocalGame\(\)' 'Held-object updates must refresh the active proxy body-local pivot before composing target points.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'localPointToWorld\(desiredBodyWorld,\s*activePivotBBodyLocalGame\)' 'Held-object proxy target point must derive from the same active body-local pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabAuthorityBodyWorldTransform[\s\S]*tryGetBodyArrayWorldTransform' 'Proxy-constraint object-side authority must keep using the BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy-constraint and native object-side reads must use the rigid BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintUsesMotionBodyAtGrab\s*=\s*false' 'Proxy-constraint grab capture must keep body-B constraint data in the rigid BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*grabBodyWorldAtGrab' 'Proxy-constraint grab capture must encode body-B pivots and desired targets in the rigid BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'refreshGrabCaptureTransforms\(rootNode,\s*meshSourceNode,\s*collidableNode\)[\s\S]*objectWorldTransform\s*=\s*collidableNode->world[\s\S]*objectToBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*grabBodyWorldAtGrab\)' 'Dynamic grab capture must refresh selected scene-node transforms before sampling objectWorldTransform for BODY relation capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolvedOwnerNode\s*=\s*bodyCollisionObjectAtResolution[\s\S]*refreshGrabCaptureNodeTransform\(resolvedOwnerRefresh,\s*"resolvedOwner",\s*resolvedOwnerNode\)[\s\S]*objectWorldTransform\s*=\s*collidableNode->world[\s\S]*cacheTrianglesInLocalSpace\(grabMeshTriangles,\s*objectWorldTransform\)' 'Dynamic grab capture must refresh the final BODY owner node before resampling objectWorldTransform for mesh-local and BODY relation capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'node->UpdateTransforms\(update\)' 'Grab capture refresh must use the narrow transform update path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'refreshGrabCaptureNodeTransform[\s\S]*node->Update\(' 'Grab capture refresh must not call full scene-node Update in the capture helper.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GRAB_TRACE stage=capture_refresh' 'Grab capture refresh must be visible in the grab timeline trace.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.physicsRateForceScalingEnabled\s*=\s*g_rockConfig\.rockGrabPhysicsRateForceScalingEnabled[\s\S]*\.physicsDeltaSeconds\s*=\s*deltaTime[\s\S]*\.physicsRateMaxForceScale\s*=\s*g_rockConfig\.rockGrabPhysicsRateMaxForceScale' 'Dynamic grab motors must feed live Havok drive delta and config into physics-rate force scaling.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'std::atomic<float>\s+_lastGrabPhysicsHz\{\s*90\.0f\s*\}[\s\S]*std::atomic<float>\s+_lastGrabPhysicsRateForceScale\{\s*1\.0f\s*\}' 'Physics-rate grab telemetry must be synchronized because motor writes and diagnostic reads can cross runtime callbacks.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_lastGrabPhysicsHz\.store\(output\.physicsHz,\s*std::memory_order_relaxed\)[\s\S]*_lastGrabPhysicsRateForceScale\.store\(output\.physicsRateForceScale,\s*std::memory_order_relaxed\)[\s\S]*_lastGrabPhysicsHz\.load\(std::memory_order_relaxed\)[\s\S]*_lastGrabPhysicsRateForceScale\.load\(std::memory_order_relaxed\)' 'Physics-rate grab telemetry must use atomic store/load snapshots in grab motor diagnostics.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'scaledBaseForce\s*=\s*baseForce\s*\*\s*out\.physicsRateForceScale[\s\S]*capForceByMass\(scaledBaseForce\s*\*\s*out\.fadeFactor,\s*motorMass,\s*input\.forceToMassRatio\)\s*\*\s*authorityForceScale' 'Physics-rate force scaling must apply before fade, mass cap, and authority scale in the grab motor policy.'
Require-Text 'src/RockConfig.h' 'rockGrabPhysicsRateForceScalingEnabled[\s\S]*rockGrabPhysicsRateReferenceHz[\s\S]*rockGrabPhysicsRateForceScaleExponent[\s\S]*rockGrabPhysicsRateMinForceScale[\s\S]*rockGrabPhysicsRateMaxForceScale' 'Physics-rate grab force scaling must expose explicit config state.'

$grabDriveTextForBoundary = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$grabDriveStart = $grabDriveTextForBoundary.IndexOf('bool Hand::tryGetGrabDriveObjectWorldTransform')
$grabDriveEnd = if ($grabDriveStart -ge 0) { $grabDriveTextForBoundary.IndexOf('RE::NiPoint3 Hand::activeProxyConstraintPivotBLocalGame', $grabDriveStart) } else { -1 }
if ($grabDriveStart -lt 0 -or $grabDriveEnd -lt 0) {
    $failures.Add('Grab drive object-frame helper boundary could not be located.')
} else {
    $grabDriveHelperText = $grabDriveTextForBoundary.Substring($grabDriveStart, $grabDriveEnd - $grabDriveStart)
    if ($grabDriveHelperText -match 'tryResolveLiveBodyWorldTransform|MotionCenterOfMass|HeldObjectDriveMode') {
        $failures.Add('Proxy-constraint runtime reads must not switch body-B to the live MOTION/COM frame.')
    }
}
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*grabBodyWorldAtGrab\)' 'Dynamic grab must capture the visible object to BODY relation from FO4VR BODY readback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'freezeGrabAuthorityFrame<RE::NiTransform>[\s\S]*\.objectWorld\s*=\s*objectWorldTransform[\s\S]*\.bodyWorld\s*=\s*grabBodyWorldAtGrab[\s\S]*\.desiredBodyWorld\s*=\s*desiredBodyWorld[\s\S]*applyFrozenGrabAuthorityFrameToGrabFrame\(_grabFrame,\s*frozenAuthorityFrame\)' 'Dynamic grab must freeze an explicit BODY solver target while preserving the visual object-to-BODY relation through the authority frame contract.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'pivotBBodyLocalGame\s*=\s*transform_math::worldPointToLocal\(input\.bodyWorld,\s*input\.gripPointWorld\)' 'Grab pivotB must be frozen body-local inside the single authority-frame freeze function.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'pivotBConstraintLocalGame\s*=\s*transform_math::worldPointToLocal\(input\.constraintBodyWorld,\s*input\.gripPointWorld\)' 'Proxy constraint pivotB must be frozen from the same selected point inside the single authority-frame freeze function.'

$authorityFrameText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$applyFrozenStart = $authorityFrameText.IndexOf('void applyFrozenGrabAuthorityFrameToGrabFrame')
$applyFrozenEnd = if ($applyFrozenStart -ge 0) { $authorityFrameText.IndexOf('GrabPivotAuthoritySource inferGrabPivotAuthoritySource', $applyFrozenStart) } else { -1 }
if ($applyFrozenStart -lt 0 -or $applyFrozenEnd -lt 0) {
    $failures.Add('Frozen authority frame apply helper boundary could not be located.')
} else {
    $outsideApplyFrozen = $authorityFrameText.Remove($applyFrozenStart, $applyFrozenEnd - $applyFrozenStart)
    if ($outsideApplyFrozen -match '_grabFrame\.(rawHandSpace|handBodyToRawHandAtGrab|proxyAuthorityHandSpace|proxyAuthorityBodyHandSpace|bodyLocal|rootBodyLocal|ownerBodyLocal|gripPointLocal|gripPointBodyLocalGame|pivotBBodyLocalGame|pivotBConstraintLocalGame|pivotAHandBodyLocalGame|desiredObjectWorldAtGrab|desiredBodyWorldAtGrab|hasFrozenPivotB|hasGripPoint)\s*=') {
        $failures.Add('Solver authority frame fields must only be written through applyFrozenGrabAuthorityFrameToGrabFrame.')
    }
}

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.objectBodyWorld\s*=\s*grabBodyWorldAtGrab' 'Three-phase grip area must capture body-local grip data in the native BODY grab authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'relationMode\s*=\s*usingPinchPocket\s*\?\s*"pinchPocket"\s*:\s*"rockPointToPalm"' 'Grab commit must route authored nodes through the same pinch/support authority path as generic grabs.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'shiftObjectToAlignGripWithPocket\(\s*grabBodyWorldAtGrab,\s*grabPivotAWorld,\s*grabGripPoint\s*\)[\s\S]*deriveNodeWorldFromBodyWorld\(desiredBodyWorld,\s*objectToBodyAtGrab\)' 'Grab commit must translate the selected BODY-local grip point to the palm anchor, then derive the visual object target from the BODY relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildDesiredObjectWorldFromAuthoredGrabNode' 'Authored grab nodes must not bypass final pinch/support authority with authored rotation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'restoreFailedGrabPrep\s*=\s*\[\&\]\(\)[\s\S]*_savedObjectState\.clear\(\)' 'Failed grab setup must clear any staged saved object state before returning.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'no object-side contact point[\s\S]*object origin/COM fallback is not valid dynamic grab authority' 'Dynamic grab must fail when no contact/authored object-side pivot exists instead of falling back to object origin or COM.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabPivotAWorld\s*=\s*pocket\.palmCenterWorld' 'Three-phase grab capture must seat the selected point at the palm anchor, not the depth-offset pocket center.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'threePhaseTouchReachedFrozenRelation' 'Near/far convergence must transition to held without recapturing the object-hand relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'threePhaseTimeoutInsidePocket' 'Near/far convergence timeout must promote to held inside the hand pocket instead of only logging.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildAcquisitionFingerPose' 'Near/far convergence must publish an acquisition finger pose before final touch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'publishLocalTransforms && g_rockConfig\.rockGrabMeshLocalTransformPoseEnabled' 'Acquisition finger pose must not publish surface local-transform corrections before final touch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabSeatDepthMaxGameUnits' 'Pulled and close grabs must seat by mesh support depth instead of a fixed hand-back adjustment.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildHeldObjectRelativeHandWorld\(heldVisualNodeWorld,\s*_grabFrame\.rawHandSpace\)' 'Held visual hand target must use the ROCK held-relative relation from the live held object and frozen raw hand-space relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'shouldSmoothHeldObjectRelativeHand\([\s\S]*_grabAcquisitionPhase == grab_three_phase::AcquisitionPhase::TouchHeld[\s\S]*visualPublishDecision\.acquisition[\s\S]*if \(smoothVisualHand\)' 'Held visual hand must smooth only during acquisition and follow the held object immediately once TouchHeld.'
Require-Text 'src/physics-interaction/hand/HandVisual.h' 'shouldSmoothHeldObjectRelativeHand[\s\S]*lerpEnabled && acquisitionVisual && !touchHeldPhase' 'Held visual hand smoothing policy must keep TouchHeld object-relative follow uncapped.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyGrabExternalHandWorldTransform\(_isLeft,\s*_grabVisualHandTransform\)' 'Held visual hand/arm pose must be published as an external FRIK transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'visual held-object hand target exceeded max deviation' 'Visual hand/arm target must release instead of pulling the rendered hand away indefinitely.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'relationPivotErr=.*rotationPreservedDeg=.*bodyTargetNodeErr=.*normalAuthority=.*authoredRotation' 'Grab telemetry must log the relation invariants needed to verify the plan in screenshots/logs.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'targetBodyOverlayFrameSource[\s\S]*BodyOverlayRole::Target[\s\S]*BodyOverlayFrameSource::BodyArrayTransform[\s\S]*extractBody\(source\.world,\s*entry\.bodyId,\s*targetBodyOverlayFrameSource\(entry\.role\)' 'Target collider visualizer must capture selected/held object colliders from the same hknp BODY frame used by grab body-B authority, not the generic MOTION/COM frame.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'targetAxisOverlayFrameSource[\s\S]*AxisOverlayRole::TargetBody[\s\S]*BodyOverlayFrameSource::BodyArrayTransform[\s\S]*extractBody\(source\.world,\s*published\.entry\.bodyId,\s*targetAxisOverlayFrameSource\(published\.entry\.role\)' 'Target body axes in the visualizer must also be captured from the hknp BODY frame so the drawn orientation matches body-B authority.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabForceTorqueDebugSnapshot' 'Focused grab force/torque overlay must expose a case-specific snapshot instead of reusing noisy generic markers.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'This view is intentionally built from the same BODY-local pivot' 'Focused grab force/torque overlay must document why it follows active BODY pivot authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'bDebugDrawGrabForceTorque|rockDebugDrawGrabForceTorque' 'Focused grab force/torque overlay must be controlled by its own debug toggle.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabPivotSourceCollider' 'Focused grab force/torque overlay must draw the hknp body/collider owning pivot B.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabPivotSourceTriangle' 'Focused grab force/torque overlay must draw the mesh triangle evidence that selected the pivot when available.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabSupportFrameDebugSnapshot' 'Grab support-frame overlay must expose its own snapshot instead of reusing force/torque telemetry.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'rockDebugDrawGrabSupportFrame[\s\S]*getGrabSupportFrameDebugSnapshot' 'Grab support-frame overlay must be controlled by a dedicated debug toggle.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'supportFrameNormalBodyLocal[\s\S]*supportFrameAxisBodyLocal[\s\S]*supportFrameBinormalBodyLocal' 'Grab support-frame overlay must replay the captured face frame through BODY-local authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'authority must come from the object surface[\s\S]*grabSurfaceHit\.valid \? grabSurfaceHit\.normal[\s\S]*contactPatchRuntime\.meshSnapped \? contactPatchRuntime\.meshSnapHit\.normal[\s\S]*pocket\.palmNormalWorld' 'Grab support normal authority must prefer mesh/contact face normals before falling back to the palm/proxy normal.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'struct\s+ImmutableGrabCaptureTelemetry' 'Grab telemetry must preserve the original capture evidence separately from mutable live authority fields.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.freezeCaptureTelemetry\(objectBodyId\.value\)' 'Grab commit must freeze immutable capture telemetry before seated-pivot reacquire can mutate live authority.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'SeatedPivotReacquire' 'Seated pivot replacement must be an explicit acquisition phase, not hidden inside normal held telemetry.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB track' 'Focused grab overlay must label solver distance as pivot tracking, not generic grab error.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GRAB err' 'Focused grab overlay must not present pivot tracking distance as generic grab error.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabPivotSourceBodyVisualLock' 'Focused grab overlay must draw the body-derived versus visual-node mesh lock delta.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'Active pivot-B markers answer a different question than mesh evidence' 'Focused grab overlay must document the distinction between visual mesh evidence and the active solver pivot.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabActivePivotBLiveBody' 'Focused grab overlay must draw the exact active pivot B through the live BODY frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabActivePivotBDesiredBody' 'Focused grab overlay must draw the exact active pivot B through the desired BODY frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabActivePivotBVisualNode' 'Focused grab overlay must draw the active solver pivot converted back through the visible node when available.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'pbLock' 'Focused grab overlay text must report the active pivot-B visual lock error, not an ambiguous generic lock value.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'held object drive body readback failed before queuing grab authority' 'Held update must not queue grab authority or deviation samples from missing body readback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform\(world,\s*_savedObjectState\.bodyId' 'Held-object convergence and active-drive pivot telemetry must use the drive-specific object frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'motionDiagVsGrab' 'Grab telemetry must expose MOTION/COM diagnostic drift from native BODY grab authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'havok_runtime::tryReadMotionVelocityCaps' 'Hand grab motion diagnostics must read hknp velocity caps through the native runtime boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'havok_runtime::snapshotMotionPropertiesLibrary' 'Hand grab motion-property library diagnostics must read hknp library layout through the native runtime boundary.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kMotion_MaxLinearVelocityPacked|kMotion_MaxAngularVelocityPacked|kHknpWorld_MotionPropertiesLibraryPtr|kMotionPropertiesLibrary_Entries|kMotionPropertiesLibrary_Count|kMotionProperties_RecordSize|motionPtr\s*\+\s*0x3A|motionPtr\s*\+\s*0x3C|worldPtr\s*\+\s*0x5D0|libraryPtr\s*\+\s*0x28|libraryPtr\s*\+\s*0x30' 'HandGrab must not carry raw hknp motion diagnostic or motion-property library layout reads.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'SolvedGrabFingerPose\s+_grabFingerPose' 'Grab finger pose must be stored once solved instead of re-solving from the live dynamic body every update.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'FingerPoseTriangleSpatialIndex\s+_grabFingerTriangleIndex' 'Each hand must own one bounded object-local finger triangle index for the active grab.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'targetObjectWorld\s*=[\s\S]*_grabFrame\.desiredObjectWorldAtGrab[\s\S]*solveFrozenMeshFingerPose\([\s\S]*localFingerPoseTriangles[\s\S]*_grabFingerTriangleIndex' 'Regular grab commit must solve once against the frozen target relation through the shared local spatial-index boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveSurfaceAimObjectLocal\(_grabFingerPose,\s*desiredObjectWorld\)[\s\S]*buildAcquisitionFingerPose\(resolvedTargetPose,\s*acquisitionProgress\)' 'Near/gravity acquisition must blend toward the pre-solved endpoint instead of re-solving transient geometry.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pinch solve deferred until TouchHeld[\s\S]*if \(!_grabFingerPosePublished\)[\s\S]*applyPinchFingerPosePolicy\(_grabFingerPose' 'Pinch must retain its established deferred at-touch solve as the sole special route.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'finalPoseObjectWorld[\s\S]*resolveSurfaceAimObjectLocal\(_grabFingerPose,\s*finalPoseObjectWorld\)[\s\S]*0\.0f,\s*true,\s*true' 'TouchHeld must atomically publish the same object-local endpoint without a second regular solve.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'heldResolveMaxValueDelta|FINGER-CYCLE ADOPT|_grabFingerPoseFrozen|rockGrabFingerPoseUpdateInterval' 'Held updates must not retain repeated finger geometry solve/adoption machinery.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.h' 'copyGrabSuppressionArmBodyIdsAtomic' 'Body colliders must expose a side-local arm-chain body ID query for normal grab suppression.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'role != BoneColliderRole::ForearmSegment && role != BoneColliderRole::HandSegment' 'Normal grab arm-chain suppression must include only the same-side forearm and wrist/hand body segments.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'kGrabCollisionSuppressionBodyCountPerHand[\s\S]*kHandColliderBodyCountPerHand[\s\S]*kGrabCollisionSuppressionArmBodyCountPerHand' 'Normal grab collision suppression capacity must cover the full hand suite plus the same-side arm chain.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'copyGrabSuppressionArmBodyIdsAtomic\(_isLeft,\s*armBodyIds\.data\(\),\s*armBodyIds\.size\(\)\)' 'Normal held-object grabs must lease the same-side forearm/wrist body colliders while the grab owns the hand.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.grabSelectedObject\([\s\S]*&_bodyBoneColliders[\s\S]*sharedContext' 'Grab commit must pass body colliders into normal grab collision suppression.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.updateHeldObject\([\s\S]*&_bodyBoneColliders[\s\S]*makeGrabReleaseContext' 'Held update must keep the same-side arm-chain collision leases refreshed while a normal grab is active.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'copyGrabSuppressionArmBodyIdsAtomic|held-grab-arm-chain' 'Two-handed equipped weapon grab must not inherit normal-grab forearm collision suppression.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::flushPendingHeldNativeGrab' 'Native action fallback/diagnostic flush must remain removed.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::flushPendingCustomGrabAuthority' 'Proxy constraint dynamic grab authority must flush from the between-collide-and-solve boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'updateProxyConstraintGrabDriveTarget' 'Proxy constraint dynamic grab must refresh constraint transforms and angular target from the captured palm-authority hand relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueProxyGrabAuthorityTarget' 'Game-frame held updates must queue proxy targets instead of writing solver authority directly.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'drive=\{\} bodyDriveMode=\{\} driveReason=\{\} forceShare=\{:\.2f\} linearScope=\{\} angularScope=\{\} massScope=\{\} looseWeapon=' 'Dynamic grab creation telemetry must identify runtime drive mode, body-set drive policy, shared force budget, scopes, and loose-weapon status.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveMode=nativeMouseSpring[^\r\n]*constraintTau' 'Native mouse-spring telemetry must not present shared constraint tau as active native tuning.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'isLooseWeaponGrabTarget[\s\S]*grab_target::canUseRockActiveGrab[\s\S]*RE::ENUM_FORM_ID::kWEAP' 'Loose weapon detection must be limited to normal active grab refs, not equipped weapon support or actor equipment.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'acceptsSelectedMultibodyOwnerlessVisualMesh[\s\S]*bodySet\.acceptedCount\(\)\s*>\s*1[\s\S]*!surfaceOwnerRecord[\s\S]*nodeIsOrDescendsFrom\(bodySet\.rootNode,\s*surfaceOwnerNode\)' 'Ownerless visual mesh fallback must be limited to selected multipart object-family nodes with no concrete accepted owner record.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'surfaceOwnerMatchesResolvedBody\s*=[\s\S]*surfaceOwnerRecord->bodyId\s*==\s*primaryChoice\.bodyId[\s\S]*acceptsSelectedMultibodyOwnerlessVisualMesh\(sel,\s*preparedBodySet,\s*primaryChoice\.bodyId' 'Grab commit owner resolution must accept ownerless same-family multipart mesh evidence without accepting concrete mismatched owners.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimeGrabContactPatch[\s\S]*acceptsSelectedMultibodyOwnerlessVisualMesh\(selection,\s*bodySet,\s*resolvedBodyId' 'Contact-patch mesh recovery must use the same ownerless multipart family rule as grab commit.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_heldObjectIsLooseWeapon\s*=\s*looseWeaponGrab' 'Held state must remember non-equipped dynamic weapon refs for runtime telemetry and neutral multipliers.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier' 'Loose non-equipped weapon proxy-constraint tuning must have an explicit neutral linear tau multiplier surface.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponAdaptiveLeadMultiplier' 'Loose non-equipped weapon native/adaptive lead tuning must be removed with mouse-spring authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier' 'Loose non-equipped weapon shared-constraint tuning must have an explicit neutral linear tau multiplier surface.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'looseAngularForceMultiplier[\s\S]*rockGrabLooseWeaponSharedConstraintAngularForceMultiplier[\s\S]*\.angularForceMultiplier\s*=\s*looseAngularForceMultiplier' 'Live held-object angular force must keep loose-weapon rotation authority separate from linear pull authority.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.h' 'struct GrabConstraintMotorTuning' 'Shared constraint creation must accept a full linear/angular motor tuning profile.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rejectedFixedOrNonDynamicCount\s*=[\s\S]*bodySetRejectCount\(preparedBodySet,\s*physics_body_classifier::BodyRejectReason::StaticMotion\)[\s\S]*bodySetRejectCount\(preparedBodySet,\s*physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep\)' 'Body-set drive classification must treat only post-prep fixed/non-dynamic bodies as fixed-attached evidence.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rejectedFixedOrNonDynamicCount\s*=[\s\S]*bodySetRejectCount\(beforePrepBodySet,\s*physics_body_classifier::BodyRejectReason::StaticMotion\)' 'Body-set drive classification must not treat successfully converted pre-prep static bodies as fixed attachments.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'readHeldBodyMassSummary\(\s*world,\s*_savedObjectState\.bodyId,\s*_heldBodyIds,\s*_heldDriveDecision\.includeConnectedMass\)' 'Dynamic grab motor budgeting must route held mass through body-set drive scope, not only the selected primary body.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.mass\s*=\s*massSummary\.motorMass\(\)' 'Dynamic grab motor target solving must receive the aggregate held-body mass summary.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.mass\s*=\s*readBodyMass\(world,\s*_savedObjectState\.bodyId\)' 'Dynamic grab motor target solving must not return to primary-body-only mass.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pendingHeldAuthority\s*=\s*evaluateRuntimeHeldAuthority\([\s\S]*updateConstraintGrabDriveMotors\([\s\S]*pendingHeldAuthority[\s\S]*angularDriveOk\s*=[\s\S]*_activeConstraint\.usesRagdollAngularMotorAtom\(\)[\s\S]*_activeConstraint\.linearMotor[\s\S]*_activeConstraint\.angularMotor' 'Proxy authority flush must compute one runtime held-authority state and apply it through the solver-owned angular motor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseAuthority\s*=\s*evaluateRuntimeHeldAuthority\([\s\S]*releaseAngularVelocityCap\s*=\s*grab_motion_controller::computeAuthorityScaledAngularVelocityCap[\s\S]*releaseAngularVelocity\s*=\s*clampAngularVelocityVector\(releaseAngularVelocity,\s*releaseAngularVelocityCap\)' 'Release angular velocity must reuse held authority for the final angular cap.'
Reject-Text 'src/physics-interaction/hand/Hand.h' 'refreshHeldAuthoritySupport' 'Held-time support refresh must not exist; TouchHeld authority is frozen and immutable.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'refreshHeldAuthoritySupport\(' 'Held-time support refresh must not mutate solver authority after capture.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'setHeldAngularVelocity|applyProxyConstraintAngularVelocityDrive|world->SetBodyAngularVelocity' 'Dynamic grab must not retain the removed direct angular velocity writer.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.h' 'NativeHardKeyframeVelocity|grabAngularAuthorityFromConfig' 'Grab angular authority must not retain the removed mode-0 selector.'
Reject-Text 'src/RockConfig.cpp' 'iGrabAngularAuthorityMode|rockGrabAngularAuthorityMode' 'Config loading must not expose the removed angular authority selector.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kHeldCollisionParticipationFlags\s*=\s*0x80u' 'Proxy dynamic grab must retain the held collision-participation body flag lease by name.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kHeldAuthorityBodyFlags\s*=\s*0x08000000u' 'Proxy dynamic grab must own the held authority body flag lease formerly provided by the native action side effect.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'acquireHeldObjectBodyFlagLeases\(world,\s*_savedObjectState\.bodyId\.value,\s*_heldBodyIds' 'Grab commit must acquire held-body flag leases from the accepted primary-first body set.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseHeldObjectBodyFlagLeases\(\s*world,\s*_savedObjectState\.bodyId\.value,\s*_heldBodyIds' 'Release must restore held-body flag leases through the same accepted primary-first body set.'
$handGrabFlagLeaseText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$acquireFlagLeaseStart = $handGrabFlagLeaseText.IndexOf('HeldBodyFlagLeaseSummary acquireHeldObjectBodyFlagLeases')
$releaseFlagLeaseStart = $handGrabFlagLeaseText.IndexOf('HeldBodyFlagLeaseSummary releaseHeldObjectBodyFlagLeases')
$nextHelperAfterRelease = $handGrabFlagLeaseText.IndexOf('void copyPeerInertiaSnapshot', $releaseFlagLeaseStart)
if ($acquireFlagLeaseStart -lt 0 -or $releaseFlagLeaseStart -lt 0 -or $nextHelperAfterRelease -lt 0) {
    $failures.Add('Held body flag lease helper boundaries could not be located.')
} else {
    $acquireFlagLeaseText = $handGrabFlagLeaseText.Substring($acquireFlagLeaseStart, $releaseFlagLeaseStart - $acquireFlagLeaseStart)
    $releaseFlagLeaseText = $handGrabFlagLeaseText.Substring($releaseFlagLeaseStart, $nextHelperAfterRelease - $releaseFlagLeaseStart)
    $acquireCollisionLoopStart = $acquireFlagLeaseText.IndexOf('for (const auto bodyId : bodyIds)')
    $acquireAuthorityStart = $acquireFlagLeaseText.IndexOf('if (primaryBodyId != INVALID_BODY_ID)', $acquireCollisionLoopStart)
    $releaseCollisionLoopStart = $releaseFlagLeaseText.IndexOf('for (const auto bodyId : bodyIds)')
    $releaseAuthorityStart = $releaseFlagLeaseText.IndexOf('if (primaryBodyId != INVALID_BODY_ID)', $releaseCollisionLoopStart)
    if ($acquireCollisionLoopStart -lt 0 -or $acquireAuthorityStart -lt 0 -or $releaseCollisionLoopStart -lt 0 -or $releaseAuthorityStart -lt 0) {
        $failures.Add('Held body flag lease helpers must split set-wide collision flags from primary-only authority flags.')
    } else {
        $acquireCollisionLoopText = $acquireFlagLeaseText.Substring($acquireCollisionLoopStart, $acquireAuthorityStart - $acquireCollisionLoopStart)
        $releaseCollisionLoopText = $releaseFlagLeaseText.Substring($releaseCollisionLoopStart, $releaseAuthorityStart - $releaseCollisionLoopStart)
        if ($acquireCollisionLoopText -notmatch 'kHeldCollisionParticipationFlags' -or $releaseCollisionLoopText -notmatch 'kHeldCollisionParticipationFlags') {
            $failures.Add('Held collision-participation flag leases must remain set-wide for multipart objects.')
        }
        if ($acquireCollisionLoopText -match 'kHeldAuthorityBodyFlags' -or $releaseCollisionLoopText -match 'kHeldAuthorityBodyFlags') {
            $failures.Add('Held authority body flags must not be leased across every multipart child body.')
        }
        if ($acquireFlagLeaseText -notmatch 'primaryBodyId,\s*kHeldAuthorityBodyFlags' -or $releaseFlagLeaseText -notmatch 'primaryBodyId,\s*kHeldAuthorityBodyFlags') {
            $failures.Add('Held authority body flags must stay primary-body-only to match the former native grab action.')
        }
    }
}

Reject-Text 'src/physics-interaction/grab/GrabMotionController.h' 'HeldSupportRefresh|evaluateHeldSupportRefresh' 'Held support refresh policy must stay removed; release safety may read contact evidence but cannot rewrite TouchHeld authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'heldAuthority\s*=\s*evaluateRuntimeHeldAuthority' 'Held updates must still evaluate runtime authority for contact softening and release safety without refreshing solver pivots.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activateHeldObjectBodySet\(world,\s*objectBodyId\.value,\s*_heldBodyIds\)' 'Close grab commit must explicitly wake the accepted held-object body set after zeroing velocities.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeLocalMeshMaxDistanceFromPoint\(_grabFrame\.localMeshTriangles,\s*_grabFrame\.gripPointLocal\)' 'Dynamic grab must capture long-object lever length from the selected grip point and cached local mesh.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kMaxGrabRuntimeSurfaceContactTriangles[\s\S]*selectNearestGrabSurfaceTriangles[\s\S]*buildRuntimeMultiFingerGripContact\([\s\S]*\*multiFingerTriangleSource' 'Runtime multi-finger grab validation must use a bounded local surface triangle set instead of rescanning high-poly weapon meshes per finger candidate.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kMaxGrabRuntimeFingerPoseTriangles[\s\S]*selectNearestGrabFingerPoseTriangles[\s\S]*fingerPoseLocalMeshTriangles[\s\S]*solveFrozenMeshFingerPose\([\s\S]*localFingerPoseTriangles' 'Finger posing must feed the shared solver from the bounded local mesh cache while keeping the full mesh for authority and lever diagnostics.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' 'kMaxFingerPoseCandidateTriangles[\s\S]*useWholeMeshForMissingTargets[\s\S]*std::nth_element' 'Whole-mesh finger fallback must rank and cap high-poly mesh candidates instead of accepting every triangle.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseLongObjectAngularScale\s*=\s*grab_motion_controller::computeLongObjectAngularSpeedScale\([\s\S]*rockGrabLongObjectAngularScalingEnabled' 'Release angular velocity cap must apply the configured long-object lever scale.'
Require-Text 'src/RockConfig.cpp' 'kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier\s*=\s*4\.5f' 'Loose non-equipped weapon custom authority must preserve the HIGGS-style 9000-vs-2000 base linear force ratio.'
Require-Text 'src/RockConfig.cpp' 'kDefaultGrabLooseWeaponSharedConstraintAngularForceMultiplier\s*=\s*2\.0f' 'Loose non-equipped weapon angular authority must boost rotation without changing linear pull authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrabReleasePending|native mouse-spring flush failure' 'Native mouse-spring release/failure state must be removed.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'tryResolveLivePalmAnchorReference[\s\S]*tryResolveLiveBodyWorldTransform' 'Grab pivot authority must resolve from the live palm-anchor body frame.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'flushPendingHeldNativeGrab' 'Physics step coordinator must not flush removed native mouse-spring authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_rightHand\.flushPendingCustomGrabAuthority\(world,\s*timing\)' 'Right custom grab proxy authority must flush from the between-collide-and-solve coordinator.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_leftHand\.flushPendingCustomGrabAuthority\(world,\s*timing\)' 'Left custom grab proxy authority must flush from the between-collide-and-solve coordinator.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_rightHand\.abandonHavokStateAfterWorldLoss\(\)' 'Right hand must abandon stale native grab state before reset when the hknp world is lost.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_leftHand\.abandonHavokStateAfterWorldLoss\(\)' 'Left hand must abandon stale native grab state before reset when the hknp world is lost.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.hasPendingPullCatchCommit\(\)' 'Far-pull arrival must retry close grab commit while the original grip is still held.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'cancelled pull catch commit because grip was released' 'Far-pull catch intent must preserve hold-to-cancel semantics.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.hasArrivedPullCatchIntent\(\)\s*&&\s*!hand\.hasPendingPullCatchCommit\(\)' 'Stale pull-catch ownership must be cancelled and released if selection no longer matches the arrived pull owner.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'reacquirePullCatchCloseSelection' 'Arrived pull-catch ownership must try a ROCK target-specific wide close reacquire before cancelling.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'ROCK gives the pulled object a wider target-specific close grab query' 'Pull-catch wide reacquire must document the ROCK behavior it owns.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.notePullCatchCommitAttemptFailed\(\)' 'Pull-catch retry timeout must begin after an actual failed close commit attempt.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'GrabEventType::PullCatchAttempt' 'Pull-catch retry attempts must be visible through the grab event stream.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'dispatchGrabCommittedEvent[\s\S]*fillGrabEventBodyKinematics' 'Grab-committed events must populate mass and body kinematics for haptics/API consumers.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC[\s\S]*return;' 'Grab haptics must honor per-event suppression flags.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'const bool lockedSelection\s*=\s*hand\.lockFarSelection\(\);\s*if \(lockedSelection\)\s*\{[\s\S]*GrabEventType::SelectionLocked[\s\S]*ROCK_GRAB_EVENT_FLAG_SUPPRESS_HAPTIC[\s\S]*const bool pullStarted\s*=\s*lockedSelection\s*&&\s*hand\.startDynamicPull' 'Far-pull start must publish SelectionLocked before dynamic pull startup while suppressing same-frame selection haptics.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'const bool pullStarted\s*=\s*lockedSelection\s*&&\s*hand\.startDynamicPull[\s\S]*GrabEventType::SelectionLocked' 'Far-pull start must not publish SelectionLocked after startDynamicPull can clear failed selections internally.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'ownerGraceSeconds\s*=\s*g_rockConfig\.rockPullOwnerGraceSeconds' 'Dynamic pull must keep pulled-object ownership after the velocity drive window.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'PULL holding owner after velocity window' 'Dynamic pull must not release the selected owner merely because velocity application ended.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'motionResult\.expired\s*\|\|\s*!motionResult\.applyVelocity' 'Dynamic pull owner expiry must stay separate from the short velocity application window.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.captureHeldReleaseMotion\(hknp,\s*handInput\.rawHandWorld,\s*frame\.deltaSeconds\);[\s\S]*hand\.releaseGrabbedObject' 'Normal grip release must capture the current-frame controller/body velocity sample before release velocity is composed.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'composeControllerReleaseAngularVelocity' 'Controller-derived release angular velocity must use the pure capped policy instead of raw hand angular velocity.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'gamePointToHavokPoint\(transform_math::localPointToWorld\(releaseBodyWorld,\s*activeProxyConstraintPivotBLocalGame\(\)\)\)' 'Tangential throw velocity must use the active proxy-constraint grab pivot lever arm when the body frame is readable.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Letting normal near/far queries refresh this selection can orphan' 'Selection refresh must be frozen while an arrived pull-catch owns the ref/body.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_state == HandState::SelectedClose[\s\S]*!_currentSelection\.isFarSelection[\s\S]*_currentSelection\.bodyId\.value == _pullCatchIntent\.primaryBodyId' 'Pending pull-catch commit must require the original close body and must not match a far selection refresh.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct PullCatchIntent' 'Far-pull catch state must be explicit hand lifecycle state, not inferred from a stale input edge.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'evaluateConvergencePromotion' 'Convergence timeout promotion must be a testable policy with stability gating.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeAcquisitionVisualEnvelopeGameUnits' 'Pre-touch visual hand authority must use a bounded acquisition envelope instead of the full near-converge distance.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'struct PullCatchSeatSafetyInput[\s\S]*grabbedFromPullCatch[\s\S]*usingPinchPocket[\s\S]*struct PullCatchSeatSafetyDecision[\s\S]*allowImmediateTouchHeld[\s\S]*requireSettledVisualRelation' 'Pull-catch palm seating safety must be an explicit testable policy contract.'
Reject-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'allowPulledAdjust|pulledAdjustDistanceGameUnits' 'The fixed-distance pulled-grab adjust is retired: its desired-world translation was erased by pivot re-alignment at freeze. The seat depth stop owns surface seating.'
Reject-Text 'src/RockConfig.cpp' 'fPulledGrabHandAdjustDistanceGameUnits' 'The retired pulled-grab adjust must not keep a dead config key that pretends to tune seating.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GrabSeatDepthStopResult computeGrabSeatDepthStop\([\s\S]*worldVectorToLocal\([\s\S]*-palmNormalWorld\.x[\s\S]*closestPointOnTriangleToPoint\(axisPointLocal,\s*triangle,\s*distanceSquared\)' 'Seat depth must be a mesh support-function query along the palm normal with axis probes covering coarse tessellation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if\s*\(!looseWeaponPrimaryAttachApplied\s*&&\s*!usingPinchPocket\s*&&\s*pocket\.valid\)\s*\{[\s\S]*computeGrabSeatDepthStop\(\s*grabLocalMeshTriangles,\s*seatObjectWorld,[\s\S]*grabPivotAWorld\s*=\s*grabPivotAWorld\s*\+\s*pocket\.palmNormalWorld\s*\*\s*seatDepthOffsetGameUnits;[\s\S]*shiftObjectToAlignGripWithPocket\(\s*seatBodyWorld,\s*grabPivotAWorld,\s*grabGripPoint\)' 'Grab capture must apply the seat depth stop against the SEAT orientation by moving pivot A along the palm normal (the only translation authority at freeze), excluding weapon attach frames and pinch pockets.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool seatSwingAlignmentWanted =\s*sel\.forcedArrival && g_rockConfig\.rockForceGrabSeatAlignmentEnabled;[\s\S]*computeGrabMeshLongAxis\(grabMeshTriangles\)[\s\S]*rockPullPresentationMinElongationRatio[\s\S]*pocket\.crossPalmWorld[\s\S]*rotateTransformWorldAboutPoint\(\s*grabBodyWorldAtGrab, rotationAxis, angleRadians, grabGripPoint\)' 'Forced arrivals must seat-align the mesh long axis to the cross-palm line by rotating about the grip point, gated on the shared elongation ratio.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool seatRollAlignmentWanted =\s*programmaticArrival && g_rockConfig\.rockGrabSeatRollAlignmentEnabled;[\s\S]*rockGrabSeatRollMinSecondElongationRatio[\s\S]*crossProduct\(currentAxisWorld, currentSecondAxisWorld\)[\s\S]*rotateTransformWorldAboutPoint\(\s*seatBodyWorld, currentAxisWorld, rollAngleRadians, grabGripPoint\)' 'Programmatic arrivals must roll the seat about the mesh long axis so the thinnest-extent face meets the palm, gated on the second elongation ratio (minimal-arc swings leave roll untouched by construction).'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rotateTransformWorldAboutPoint\(\s*seatBodyWorld, plateAxis' 'Plates must NOT have their face swung onto the palm: 31 user-verified plate holds put the face normal 63 deg (IQR 57..72) off the palm normal because plates are held by an edge or corner. That alignment fought the intended grip and was removed 2026-07-25 - do not reintroduce it without new ground truth.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'seatRollReason = seatPlateShape \? "plateEdgeHoldNoFaceAlign" : "belowSecondElongationGate";' 'The plate class must still be resolved and reported (the penetration backstop selects its footprint from it) while explicitly performing no face alignment.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'float gripAxisTiltRadiansForHand\(bool isLeft\)[\s\S]*isLeft \? -g_rockConfig\.rockPullPresentationGripAxisTiltDegrees[\s\S]*: g_rockConfig\.rockPullPresentationGripAxisTiltDegrees' 'Grip-axis tilt must carry the hand sign: it is measured from the cross-palm axis, which comes from the non-mirrored authored handspace convention, so authored +Z is thumbward on one hand and pinkyward on the other. Ground truth is +34 deg right / -29 deg left in that convention.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockPullPresentationGripAxisTiltDegrees \* 0\.01745329252f' 'Grip-axis tilt must go through gripAxisTiltRadiansForHand, never be converted inline - an inline conversion silently drops the per-hand sign.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmPlaneOvershoot = computeGrabSeatDepthStop\(\s*grabLocalMeshTriangles,\s*desiredObjectWorld,\s*pocket\.palmCenterWorld,[\s\S]*SEAT DEPTH BACKSTOP' 'Grab capture must re-measure penetration from the palm plane on the FINAL seat pose and loudly push out any overshoot the grip-point depth stop missed (grip seeds at/behind the surface make the primary stop a silent no-op).'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const float backstopFootprintRadiusGameUnits =\s*seatPlateShape \? g_rockConfig\.rockGrabSeatPenetrationBackstopFootprintRadiusGameUnits\s*: g_rockConfig\.rockGrabSeatDepthFootprintRadiusGameUnits;' 'The penetration backstop footprint must be shape-selected: the tuned seating radius bounds detectable tilt penetration to r*sin(theta) and cannot double as the safety check for flat faces, while widening it for every shape would push seats out on geometry that merely passes beside the palm.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'requiresSettledVisualHandRelation &&\s*!_grabFrame\.hasSeatedPivotReacquire &&\s*!_grabFrame\.hasSettledVisualHandRelation\) \{\s*_grabFrame\.hasSettledVisualHandRelation = true;' 'TouchHeld promotion must never carry an unsatisfiable settled-visual-relation requirement: a rejected seated reacquire settles the visual hand to the frozen commanded relation instead of blocking the publish gate for the whole hold.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'RE::NiTransform rotateTransformWorldAboutPoint\([\s\S]*rotateAroundUnitAxis\(rowWorld, unitAxisWorld, angleRadians\)[\s\S]*rotated\.translate = pivotWorld \+ rotatedOffset' 'World-side seat rotation must go through the stored-row convention helper, never raw column math.'
Reject-Text 'src/RockConfig.cpp' 'bCalibratedGrenadeOffsetsEnabled' 'The hardcoded grenade/Molotov calibrations are retired; throwables seat through the generic machinery until fresh presets are captured.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'calibrated_grenade_offset' 'Grab offset resolution must not reference the retired calibrated grenade preset policy.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeGrabSeatDepthStop\(\s*_grabFrame\.localMeshTriangles,\s*currentNodeWorld,\s*promotedPointWorld,\s*palmNormalWorld,[\s\S]*seatPivotAWorld\s*=\s*livePivotAWorld\s*\+[\s\S]*shiftObjectToAlignGripWithPocket\(grabBodyWorld,\s*seatPivotAWorld,\s*promotedPointWorld\)[\s\S]*\.pivotAWorld\s*=\s*seatPivotAWorld' 'Seated pivot reacquire must apply the same seat depth stop so promotion cannot undo the capture-time surface seat.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'GrabMeshLongAxisResult computeGrabMeshLongAxis\([\s\S]*triangleArea[\s\S]*dominantEigen[\s\S]*elongationRatio\s*=\s*static_cast<float>' 'Long-object orientation must come from area-weighted mesh PCA, not authored NIF axes.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_pullPresentationValid\s*=\s*false;[\s\S]*rockPullLongAxisPresentationEnabled && hasPrimaryBodyWorld[\s\S]*longAxis\.elongationRatio >= g_rockConfig\.rockPullPresentationMinElongationRatio[\s\S]*_pullPresentationAxisBodyLocal\s*=\s*axisBodyLocal' 'Pull start must capture the presentation axis in primary-body local space, gated on elongation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if \(dotProduct\(currentAxisWorld, targetAxisWorld\) < 0\.0f\)[\s\S]*angleRadians \* \(std::max\)\(0\.0f, g_rockConfig\.rockPullPresentationAngularGainPerSecond\)[\s\S]*rockPullPresentationMaxAngularSpeedRadiansPerSecond' 'The flight presentation servo must align to the nearest hemisphere with an angle-proportional, capped angular speed.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if \(presentationActive\) \{[\s\S]*setHeldVelocity\(world, RE::hknpBodyId\{ _pulledPrimaryBodyId \}, _pulledBodyIds, motionResult\.velocityHavok,\s*presentationAngularVelocity,\s*true,[\s\S]*\} else \{[\s\S]*angularVelocityKeepForDamping' 'Presentation must drive angular velocity only during pull flight and fall back to plain damping when inactive; held objects keep the no-rotate rule.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_pullPresentationAxisBodyLocal = \{\};[\s\S]*_pullPresentationValid = false;' 'Pull runtime cleanup must clear the presentation axis state.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if \(g_rockConfig\.rockPullToObjectCenterEnabled\) \{\s*_pullPointOffsetHavok = \{\};' 'Far pulls must track the object center, not the selection ray hit, when center-pull is enabled.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void clearGrabHandPose\(bool isLeft\)' 'Hand reset/world-loss cleanup must have one explicit helper for clearing the ROCK_Grab FRIK tag.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void Hand::reset\(\)[\s\S]*clearGrabHandPose\(_isLeft\)[\s\S]*clearGrabExternalHandWorldTransform\(_isLeft\)' 'Hand reset must clear both ROCK_Grab pose and ROCK_GrabVisual external transform tags.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void Hand::abandonHavokStateAfterWorldLoss\(\)[\s\S]*clearGrabHandPose\(_isLeft\)[\s\S]*clearGrabExternalHandWorldTransform\(_isLeft\)' 'World-loss abandon must clear both ROCK_Grab pose and ROCK_GrabVisual external transform tags.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'if\s*\(!node \|\| !node->IsTriShape\(\)\)\s*\{[\s\S]*return false;[\s\S]*blacklistedShapes' 'Grab mesh extraction blacklist must skip matching geometry only, not marker parent subtrees.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'findClosestGrabSurfaceHitToPointPositionOnly' 'Grab mesh lookup must provide a position-only palm-pocket query so bad normals do not discard usable close-grab pivot evidence.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'kContactPatchProbePatternSampleCount\s*=\s*9' 'Grab contact patch sampling must expose a bounded nine-point palm-plane probe pattern for thin/small objects.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'buildContactPatchProbeOffsets' 'Grab contact patch probe offsets must be testable pure policy, not embedded only in the Havok runtime loop.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'computeContactPatchProbeGeometry' 'Grab contact patch probe geometry must be scaled by pure object-size policy.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'namespace rock::grab_support_model_math[\s\S]*GripSupportKind[\s\S]*OpposedPinch[\s\S]*LongHandleAxis[\s\S]*PalmWrap' 'Grip support classification must be a testable policy layer after contact-patch sampling.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'patchOnlyPair[\s\S]*ContactPatch[\s\S]*normalsOpposed[\s\S]*rolesAreOpposed' 'Grip support must not treat a thin same-face contact-patch line as opposed pivot authority.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'kMaxGrabContactPatchSamples\s*=\s*grab_contact_patch_math::kContactPatchProbePatternSampleCount' 'Grab frame contact patch storage must track the bounded probe policy max.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'gripSupportKind[\s\S]*gripSupportAuthoredPivot' 'Grab frame must store support-model metadata separately from contact patch samples.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildContactPatchProbeOffsets' 'Runtime grab contact patches must consume the shared bounded probe pattern.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'contactPatchObjectLeverEstimateGameUnits[\s\S]*computeLocalMeshMaxDistanceFromPoint\(grabLocalMeshTriangles,\s*canonicalPivotLocal\)[\s\S]*buildRuntimeGrabContactPatch\([\s\S]*contactPatchObjectLeverEstimateGameUnits' 'Runtime contact-patch probes must receive object lever evidence before casting.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimeGripSupportModel[\s\S]*appendOffsetProbe\(acrossAxis[\s\S]*appendOffsetProbe\(fingerAxis[\s\S]*GripSupportRole::ThumbPad[\s\S]*GripSupportRole::IndexPad' 'Runtime grip support must add across-palm, finger, and thumb/index support probes instead of only palm-normal patch samples.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'supportHitMatchesResolvedBody[\s\S]*acceptsSelectedMultibodyOwnerlessVisualMesh' 'Runtime grip support probes must reject support hits outside the held body or selected multipart visual mesh family.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'gripEvidencePointWorld\s*=\s*grabGripPoint[\s\S]*gripSupportRuntime\.model\.canAuthorPivot[\s\S]*pivotAuthoritySource\s*=\s*grabPivotAuthoritySourceName\(GrabPivotAuthoritySource::GripSupportModel\)[\s\S]*storeGripSourceEvidence\(_grabFrame,[\s\S]*gripEvidencePointWorld' 'Grip support pivot promotion must preserve the original surface point as visual/finger evidence while moving only the active solver pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'forceRuntimeGripSupportAuthority[\s\S]*forcedSupportGroupFromWeakEvidence[\s\S]*forcedSupportGroupFromGrabPoint' 'Grab commit must force weak support evidence into SupportGroup authority instead of rejecting valid non-pinch grabs.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'gripSupportForcedSinglePoint[\s\S]*gripSupportForcedSameSurface[\s\S]*GrabPivotAuthoritySource::GripSupportModel' 'Forced support upgrades must still classify as final support authority.'
Require-Text 'tests/GrabContactPatchClusterPolicyTests.cpp' 'long same-face line cannot author pivot' 'Policy tests must prove a thin same-face patch line cannot move pivot B.'
Require-Text 'src/RockConfig.cpp' 'std::clamp\(rockGrabContactPatchProbeCount,\s*1,\s*9\)' 'Config loading must allow the full nine-sample contact patch pattern.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'closePalmPocketMeshAuthority' 'Close seated grabs must publish palm-pocket mesh authority before generic mesh fallback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'SelectionHitMeshSnap[\s\S]*requiresSettledVisualHandRelation' 'Selection-hit mesh snaps that start outside touch must wait for seated palm-pocket visual relation before publishing the visual hand.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'reachedTouchMayPromote\s*=\s*reachedTouchRange\s*&&\s*seatedReacquireSatisfied' 'Touch-range promotion must not bypass required seated pivot reacquire.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'clearGrabExternalHandWorldTransform\(_isLeft\)[\s\S]*_hasGrabVisualHandTransform\s*=\s*false[\s\S]*THREE-PHASE GRAB SEATED SUPPORT-GROUP PROMOTION' 'Seated support promotion must clear any previous visual hand relation before publishing again.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'evaluateSeatedPalmPocketPromotion' 'Seated palm-pocket promotion must be a testable policy, not an ad hoc runtime branch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildSeatedPalmPocketSupportPatch[\s\S]*buildContactPatchProbeOffsets' 'Seated mesh-start promotion must rebuild bounded palm-pocket support samples instead of staying at a one-point mesh refresh.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'seatedPalmPocketRetarget' 'Medium-distance seated palm-pocket promotion must not retarget over frames; it keeps the original frozen authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'frozenSeatAuthorityFrame\s*=\s*grab_authority_frame_math::freezeGrabAuthorityFrame<RE::NiTransform>[\s\S]*applyFrozenGrabAuthorityFrameToGrabFrame\(_grabFrame,\s*frozenSeatAuthorityFrame\)' 'Accepted seated palm-pocket promotion must rebuild the frozen authority frame once, not patch individual grab references.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'seatedRetargetRejectedKeepFrozen' 'Rejected seated palm-pocket candidates must explicitly keep the original frozen authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'countFreshSemanticFingerContactGroups' 'Seated palm-pocket promotion must not turn raw semantic finger counts into motor authority.'
Reject-Text 'src/physics-interaction/grab/GrabMotionController.h' 'currentContactPatchUsedAsPivot|contactPatchUsedAsPivot|decision\.contactPatchSampleCount' 'Contact patch evidence must not publish held-time pivot support or mutate active authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimeFingerPoseTargets\(promotedPointWorld,\s*promotedNormalWorld\)[\s\S]*storeFingerPoseTargetsInGrabFrame\(_grabFrame,\s*seatedPoseTargets' 'Seated palm-pocket promotion must rebuild finger pose targets around the promoted palm-pocket seat.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'awaitingSettledVisualRelation' 'Visual hand publish gate must expose the settled-relation block reason.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'chooseMeshBackedPatchPivotAuthority[\s\S]*contactPatchPivotEvidenceOnly' 'Grab contact patch pivot policy must keep validated patches as evidence-only instead of frozen pivot authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'Contact patches stay validation/pose evidence[\s\S]*must not replace the frozen BODY-local pivot' 'Grab pivot authority must document that small or corner contact patches cannot replace pivot-B.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'positionOnlyPatch\.source\s*=\s*GrabPivotAuthoritySource::ContactPatchPositionOnly' 'Accepted contact-patch pivot authority must not be reintroduced as runtime telemetry.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'usePatchPivot\s*=\s*true' 'Contact patch authority must not reintroduce a broad unconditional pivot-B replacement.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'gripPointSourceNodeLocal' 'Grab frame must preserve mesh evidence in source-node local space instead of treating collidable-node local coordinates as universal mesh authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'Mesh triangles choose a world-space position only[\s\S]*source-node evidence[\s\S]*BODY-local pivot B' 'Mesh-backed grabs must document that triangles provide position evidence only and cannot inject object-native axes into hand/object authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'storeGripSourceEvidence\(_grabFrame,[\s\S]*grabSurfaceHit\.sourceNode\s*\?\s*grabSurfaceHit\.sourceNode\s*:\s*collidableNode[\s\S]*grabGripPoint' 'Grab commit must store source-node visual evidence separately from the BODY-local solver pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'currentRawDesiredBodyWorld[\s\S]*activeProxyConstraintPivotBLocalGame\(\)' 'Relation telemetry must compare the active BODY-local solver pivot, not the mesh-source local evidence point.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rebuildFingerPoseTargetsFromGrabFrame[\s\S]*gripEvidencePointWorld\(frame,\s*currentNodeWorld\)' 'Finger pose rebuild must read mesh evidence through the explicit source-node evidence path when available.'

$meshAuthorityText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$closePocketStart = $meshAuthorityText.IndexOf('Close seated grabs need one position authority')
$selectionSnapAfterClose = if ($closePocketStart -ge 0) { $meshAuthorityText.IndexOf('grabPointMode = "selectionHitMeshSnap"', $closePocketStart) } else { -1 }
if ($closePocketStart -lt 0 -or $selectionSnapAfterClose -lt 0) {
    $failures.Add('Palm-pocket mesh authority and selection-hit mesh snap blocks must both be locatable.')
} elseif ($closePocketStart -ge $selectionSnapAfterClose) {
    $failures.Add('Close seated palm-pocket mesh authority must run before selection-hit mesh snap.')
}

$handGrabText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$genericPoseStart = $handGrabText.IndexOf('publishLocalTransformPose("ROCK_Grab"')
$genericPoseEnd = if ($genericPoseStart -ge 0) { $handGrabText.IndexOf('localTransformState);', $genericPoseStart) } else { -1 }
if ($genericPoseStart -lt 0 -or $genericPoseEnd -lt 0) {
    $failures.Add('Generic grab local-transform publish block could not be located for alternate thumb guard.')
} else {
    $genericPoseText = $handGrabText.Substring($genericPoseStart, $genericPoseEnd - $genericPoseStart)
    if ($genericPoseText -notmatch '\.thumbAlternateCurveStrength\s*=\s*g_rockConfig\.rockGrabThumbAlternateCurveStrength') {
        $failures.Add('Generic grab local-transform publishing must honor configured alternate thumb correction strength.')
    }
    if ($genericPoseText -notmatch '\.thumbSurfaceSafetyEnabled\s*=\s*g_rockConfig\.rockGrabThumbSurfaceSafetyEnabled') {
        $failures.Add('Generic grab local-transform publishing must honor configured thumb surface safety.')
    }
    if ($genericPoseText -match '\.thumbAlternateCurveStrength\s*=\s*0\.0f') {
        $failures.Add('Generic grab must not disable alternate thumb local-transform correction after selecting the alternate curve.')
    }
}

$handText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/Hand.cpp')
$pivotStart = $handText.IndexOf('RE::NiPoint3 Hand::computeGrabPivotAWorld')
$pivotEnd = if ($pivotStart -ge 0) { $handText.IndexOf('void Hand::recordSemanticContact', $pivotStart) } else { -1 }
if ($pivotStart -lt 0 -or $pivotEnd -lt 0) {
    $failures.Add('Hand::computeGrabPivotAWorld boundary could not be located for palm-anchor authority guard.')
} else {
    $pivotText = $handText.Substring($pivotStart, $pivotEnd - $pivotStart)
    if ($pivotText -notmatch 'tryResolveLivePalmAnchorReference\(world,\s*palmReference\)') {
        $failures.Add('Grab pivot capture must read the actual live palm-anchor body reference.')
    }
    if ($pivotText -match '_boneColliders\.tryGetPalmAnchorTarget') {
        $failures.Add('Grab pivot capture must not return to the sampled palm target as active authority.')
    }
}

Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'typedef void func_t\(void\*,\s*int\)' 'Native VR drop wrapper must not leave R8 uninitialized by using the old two-argument signature.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'func\(playerChar,\s*handIndex\)' 'Native VR drop wrapper must not call the native function without the verified third argument.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId' 'Ordinary dynamic close grabs must use the custom proxy constraint authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'HeldObjectDriveMode|_heldDriveMode|heldObjectDriveModeName' 'Runtime HandGrab must not retain removed drive-mode scaffolding.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'joiningPeerHeldObject[\s\S]*createProxyConstraintGrabDrive' 'Peer-held loose-object joins must use the same proxy constraint authority instead of a second native-only translation drive.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive[\s\S]*createGrabConstraint\(world' 'Proxy dynamic grab creation must be isolated behind the explicit custom authority helper.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'setBodyKeyframed' 'Held object grab must remain dynamic and must not switch the held object to keyframed motion.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rebuildTrianglesInWorldSpace' 'Held-object finger pose must not be re-solved from live body-derived mesh triangles while the dynamic object is settling.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*constraintUsesMotionBodyAtGrab\s*\?\s*motionBodyWorldAtGrab\s*:\s*grabBodyWorldAtGrab' 'Proxy-constraint grab capture must not encode body-B constraint data through MOTION/COM.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'freezePivotBBodyLocal\(motionBodyWorldAtGrab' 'Grab pivotB must not be frozen from MOTION/COM diagnostics.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*makeIdentityTransform\(\)' 'Dynamic grab must not collapse BODY and the visible object into the same frame; logs show that causes instant angular correction.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.objectBodyWorld\s*=\s*motionBodyWorldAtGrab' 'Three-phase grip area must not use MOTION/COM diagnostics as authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeBodyTargetWithLocalGripAtPocket\(' 'Runtime HandGrab must not use body-target-to-pocket math; ROCK dynamic grab preserves object rotation and freezes point-to-palm relation.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabGripPoint\s*=\s*sel\.hasHitPoint\s*\?\s*sel\.hitPointWorld\s*:\s*objectWorldTransform\.translate' 'Dynamic grab must not initialize missing contact evidence from object origin/COM.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'recapturedNodeTarget|recapturedBodyTarget' 'Held-object convergence must not recapture the grab frame after the object starts moving.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrab\.queueTarget\(desiredObjectWorld' 'Held-object updates must not feed visual object/node rotation directly to the native mouse-spring action.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'initialNativeTargetObjectWorld' 'Close grab creation must not feed visual object/node rotation directly to the native mouse-spring action.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabPivotAWorld\s*=\s*pocket\.pocketCenterWorld' 'Runtime grab pivot must not use the depth-offset pocket center as motor authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'preferredInsetGameUnits\s*=\s*g_rockConfig\.rockGrabGripInsetGameUnits' 'Runtime grab motor pivot must not inset the selected grip point into a guessed object interior.'
Reject-Text 'src/physics-interaction/grab/GrabCore.h' 'visualAuthorityContactValid|visualAuthorityContactReason|wholeHandVisualAuthority|visualTranslationAuthority|visualRotationAuthority|GrabMotorPivot|motorPivot|hasSurfaceFrame|surfaceFrameLocal|orientationModeUsed|surfaceAlignmentDecision|hasOppositionFrame|oppositionFrameReason|oppositionThumb|oppositionOpposing|surfacePointWorldAtGrab|surfacePointLocal|surfaceHitLocal|surfaceNormalLocal|surfacePointBodyLocalGame|surfacePivotToSurfaceDistanceGameUnits|surfaceSelectionToMeshDistanceGameUnits|surfaceTriangleIndex|surfaceShapeKey|surfaceShapeCollisionFilterInfo|surfaceHitFraction|hasSurfaceHit' 'Grab frame state must not keep legacy visual-authority, surface-frame, opposition-frame, motor-pivot, or surface-named canonical grip state.'
Reject-Text 'src/RockConfig.h' 'rockGrabUseBoneDerivedPalmPivot' 'Grab pivot capture must not keep a dead live hknp palm-anchor readback config switch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'THREE-PHASE GRAB CAPTURE' 'Generic grab commit must pass through the hand-pocket acquisition capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'handPocket=palmSeat' 'Generic grab must expose one hand-pocket palm-seat authority path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'configuredGrabOrientationMode|grab_visual_authority_policy|buildDesiredObjectWorldFromSurfaceFrame|buildOppositionDesiredObjectWorld|shouldUseObjectReverseAlignedHandForFingerPose|shouldApplyObjectReverseAlignedExternalHandTransform|evaluateGrabOutputAuthority|GRAB POINT AUTHORITY|GrabMotorPivot|motorPivot|pinchMotorPivot|wholeHandVisualAuthority|visualTranslationAuthority|visualRotationAuthority|hasSurfaceFrame|surfaceFrameLocal|orientationModeUsed|surfaceAlignmentDecision|hasOppositionFrame|oppositionFrameReason' 'Runtime HandGrab must not use legacy surface/opposition/pinch/visual grab authority.'
Reject-Text 'src/physics-interaction/hand/Hand.cpp' 'GrabVisualAuthorityPolicy|grab_visual_authority_policy|trackingFallback' 'Hand adjusted-transform accessors must not preserve legacy object-owned visual hand authority.'
Reject-Text 'src/physics-interaction/grab/GrabContact.h' 'GrabMotorPivot|PinchSmallObject|smallObjectPinch|chooseActiveGrabPoint|grab_surface_frame_math|grab_opposition_frame_math|GrabOrientationMode|GrabSurfaceAlignmentDecision|buildDesiredObjectWorldFromSurfaceFrame|buildOppositionDesiredObjectWorld' 'Grab contact policy must not keep removed generic motor-pivot, surface-frame, opposition-frame, or pinch authority.'
Reject-Text 'src/physics-interaction/grab/GrabContact.h' '"legacyContactSources"|return\s+"legacyPermissive"|reason\s*=.*"legacyPermissive"' 'Grab contact diagnostics must use compatibility/permissive-fallback wording instead of legacy labels.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.cpp' '\[surface\]' 'Active grab constraint pivot diagnostics must not use stale surface-frame terminology.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' '\[relation-pivot-b\]' 'Active grab constraint pivot diagnostics must label transform-B as the relation-derived body-local pivot.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'desiredPackedMass\s*=\s*packed\[3\][\s\S]*savedMotionState\.massModified\s*=\s*true[\s\S]*Loose weapon mass preserved through inertia rebuild[\s\S]*if\s*\(inertiaModified\)\s*\{[\s\S]*rebuildMotionMassProperties\(world,\s*motionIndex\)[\s\S]*packed\[0\]\s*=\s*desiredPackedInertia\[0\][\s\S]*if\s*\(savedMotionState\.massModified\)\s*\{[\s\S]*packed\[3\]\s*=\s*desiredPackedMass' 'Grab motion mass rebuild must preserve loose-weapon packed mass even when the pre-rebuild mass did not require clamping.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' 'rebuildMotionMassProperties\(world,\s*savedMotion\.motionIndex\)[\s\S]*packed\[0\]\s*=\s*savedMotion\.savedPackedInertia\[0\][\s\S]*packed\[1\]\s*=\s*savedMotion\.savedPackedInertia\[1\][\s\S]*packed\[2\]\s*=\s*savedMotion\.savedPackedInertia\[2\][\s\S]*if\s*\(savedMotion\.massModified\)\s*\{[\s\S]*packed\[3\]\s*=\s*savedMotion\.savedPackedMass' 'Grab motion restore must reapply saved packed inertia and mass after Havok rebuilds restored motion properties.'
Reject-Text 'src/physics-interaction/hand/HandVisual.h' 'hand_visual_authority_math|buildAppliedVisualAuthorityHandWorld|buildObjectOwnedReverseAlignedHandWorld|buildSplitFrameReverseAlignedHandWorld|buildHandBoneWorldFromContactFrame' 'Hand visual helpers must not keep removed object-owned reverse visual authority.'
Reject-Text 'src/RockConfig.h' 'rockGrabThreePhaseEnabled|rockGrabObjectVisualHandAuthorityEnabled|rockGrabOrientationMode|rockGrabSmallObjectPinchPivotEnabled|rockGrabUseSemanticFingerContactPivot|rockGrabOppositionFrameEnabled|rockGrabThreePhasePreserveRotationDuringConverge|rockGrabThreePhaseDelayFingerPoseUntilTouch|rockGrabThreePhaseDisableVisualHandAuthority' 'RockConfig must not expose toggles that can re-enable legacy grab authority.'
Reject-Text 'src/RockConfig.h' 'rockDebugShowGrabSurfaceFrame' 'Debug config must use pocket-normal naming, not stale surface-frame naming.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'effectiveGrabMotorMass\(massSummaryAtCreation\.motorMass\(\)\)' 'Proxy constraint creation must seed motor force caps from the effective motor mass floor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.mass\s*=\s*massSummary\.motorMass\(\)' 'Dynamic held updates must pass raw Havok aggregate mass into the grab motor policy.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.effectiveMotorMassFloorEnabled\s*=\s*g_rockConfig\.rockGrabEffectiveMotorMassFloorEnabled' 'Dynamic held updates must pass the effective motor mass floor toggle into the grab motor policy.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.effectiveMotorMassFloor\s*=\s*g_rockConfig\.rockGrabEffectiveMotorMassFloor' 'Dynamic held updates must pass the effective motor mass floor value into the grab motor policy.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'Patch/contact/lever quality remains available to release safety[\s\S]*not live motor authority' 'Grab motor policy must document that patch quality cannot weaken live held motor force.'

if ($failures.Count -gt 0) {
    Write-Host 'Hand grab native boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Hand grab native boundary passed.'

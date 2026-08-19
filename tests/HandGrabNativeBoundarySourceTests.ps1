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

Reject-Text 'src/physics-interaction/native/HavokOffsets.h' 'MouseSpring' 'Mouse-spring offsets and tuning constants must not remain available to production grab code.'
Reject-Path 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'Native mouse-spring wrapper implementation must be removed.'
Reject-Path 'src/physics-interaction/native/NativeMouseSpringGrab.h' 'Native mouse-spring wrapper header must be removed.'

Require-Text 'src/physics-interaction/hand/Hand.h' 'tryComputeGrabProxyLocalPalmPocketPivotAWorld' 'Held-time palm-pocket support must share an explicit generated/proxy-local pivot helper.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'computePivotAHandBodyLocal[\s\S]*generatedColliderWorldPointToLocal\(handBodyWorld,\s*grabPivotWorld\)' 'Frozen pivot-A proxy local capture must use generated stored-column local space.'
Reject-Text 'src/physics-interaction/grab/GrabHeldObject.h' 'AdaptiveHeldLead|solveAdaptiveHeldLead|responseFactor' 'Removed native/adaptive target-leading helpers must not remain as unused grab authority scaffolding.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'std::atomic<float>\s+_lastGrabPhysicsHz\{\s*90\.0f\s*\}[\s\S]*std::atomic<float>\s+_lastGrabPhysicsRateForceScale\{\s*1\.0f\s*\}' 'Physics-rate grab telemetry must be synchronized because motor writes and diagnostic reads can cross runtime callbacks.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'scaledBaseForce\s*=\s*baseForce\s*\*\s*out\.physicsRateForceScale[\s\S]*capForceByMass\(scaledBaseForce\s*\*\s*out\.fadeFactor,\s*motorMass,\s*input\.forceToMassRatio\)\s*\*\s*authorityForceScale' 'Physics-rate force scaling must apply before fade, mass cap, and authority scale in the grab motor policy.'
Require-Text 'src/RockConfig.h' 'rockGrabPhysicsRateForceScalingEnabled[\s\S]*rockGrabPhysicsRateReferenceHz[\s\S]*rockGrabPhysicsRateForceScaleExponent[\s\S]*rockGrabPhysicsRateMinForceScale[\s\S]*rockGrabPhysicsRateMaxForceScale' 'Physics-rate grab force scaling must expose explicit config state.'

Require-Text 'src/physics-interaction/grab/GrabCore.h' 'pivotBBodyLocalGame\s*=\s*transform_math::worldPointToLocal\(input\.bodyWorld,\s*input\.gripPointWorld\)' 'Grab pivotB must be frozen body-local inside the single authority-frame freeze function.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'pivotBConstraintLocalGame\s*=\s*transform_math::worldPointToLocal\(input\.constraintBodyWorld,\s*input\.gripPointWorld\)' 'Proxy constraint pivotB must be frozen from the same selected point inside the single authority-frame freeze function.'

Require-Text 'src/physics-interaction/hand/HandVisual.h' 'shouldSmoothHeldObjectRelativeHand[\s\S]*lerpEnabled && acquisitionVisual && !touchHeldPhase' 'Held visual hand smoothing policy must keep TouchHeld object-relative follow uncapped.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'relationPivotErr=.*rotationPreservedDeg=.*bodyTargetNodeErr=.*normalAuthority=.*authoredRotation' 'Grab telemetry must log the relation invariants needed to verify the plan in screenshots/logs.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabForceTorqueDebugSnapshot' 'Focused grab force/torque overlay must expose a case-specific snapshot instead of reusing noisy generic markers.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'bDebugDrawGrabForceTorque|rockDebugDrawGrabForceTorque' 'Focused grab force/torque overlay must be controlled by its own debug toggle.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GrabPivotSourceCollider' 'Focused grab force/torque overlay must draw the hknp body/collider owning pivot B.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GrabPivotSourceTriangle' 'Focused grab force/torque overlay must draw the mesh triangle evidence that selected the pivot when available.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabSupportFrameDebugSnapshot' 'Grab support-frame overlay must expose its own snapshot instead of reusing force/torque telemetry.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'rockDebugDrawGrabSupportFrame[\s\S]*getGrabSupportFrameDebugSnapshot' 'Grab support-frame overlay must be controlled by a dedicated debug toggle.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'struct\s+ImmutableGrabCaptureTelemetry' 'Grab telemetry must preserve the original capture evidence separately from mutable live authority fields.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'SeatedPivotReacquire' 'Seated pivot replacement must be an explicit acquisition phase, not hidden inside normal held telemetry.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GRAB track' 'Focused grab overlay must label solver distance as pivot tracking, not generic grab error.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GRAB err' 'Focused grab overlay must not present pivot tracking distance as generic grab error.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GrabPivotSourceBodyVisualLock' 'Focused grab overlay must draw the body-derived versus visual-node mesh lock delta.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GrabActivePivotBLiveBody' 'Focused grab overlay must draw the exact active pivot B through the live BODY frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GrabActivePivotBDesiredBody' 'Focused grab overlay must draw the exact active pivot B through the desired BODY frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'GrabActivePivotBVisualNode' 'Focused grab overlay must draw the active solver pivot converted back through the visible node when available.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' 'pbLock' 'Focused grab overlay text must report the active pivot-B visual lock error, not an ambiguous generic lock value.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'SolvedGrabFingerPose\s+_grabFingerPose' 'Grab finger pose must be stored once solved instead of re-solving from the live dynamic body every update.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'FingerPoseTriangleSpatialIndex\s+_grabFingerTriangleIndex' 'Each hand must own one bounded object-local finger triangle index for the active grab.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.h' 'copyGrabSuppressionArmBodyIdsAtomic' 'Body colliders must expose a side-local arm-chain body ID query for normal grab suppression.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'role != BoneColliderRole::ForearmSegment && role != BoneColliderRole::HandSegment' 'Normal grab arm-chain suppression must include only the same-side forearm and wrist/hand body segments.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'kGrabCollisionSuppressionBodyCountPerHand[\s\S]*kHandColliderBodyCountPerHand[\s\S]*kGrabCollisionSuppressionArmBodyCountPerHand' 'Normal grab collision suppression capacity must cover the full hand suite plus the same-side arm chain.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.grabSelectedObject\([\s\S]*&_bodyBoneColliders[\s\S]*sharedContext' 'Grab commit must pass body colliders into normal grab collision suppression.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.updateHeldObject\([\s\S]*&_bodyBoneColliders[\s\S]*makeGrabReleaseContext' 'Held update must keep the same-side arm-chain collision leases refreshed while a normal grab is active.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.h' 'struct GrabConstraintMotorTuning' 'Shared constraint creation must accept a full linear/angular motor tuning profile.'
Reject-Text 'src/physics-interaction/hand/Hand.h' 'refreshHeldAuthoritySupport' 'Held-time support refresh must not exist; TouchHeld authority is frozen and immutable.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.h' 'NativeHardKeyframeVelocity|grabAngularAuthorityFromConfig' 'Grab angular authority must not retain the removed mode-0 selector.'
Reject-Text 'src/RockConfig.cpp' 'iGrabAngularAuthorityMode|rockGrabAngularAuthorityMode' 'Config loading must not expose the removed angular authority selector.'
Reject-Text 'src/physics-interaction/grab/GrabMotionController.h' 'HeldSupportRefresh|evaluateHeldSupportRefresh' 'Held support refresh policy must stay removed; release safety may read contact evidence but cannot rewrite TouchHeld authority.'
Require-Text 'src/physics-interaction/grab/GrabFinger.h' 'kMaxFingerPoseCandidateTriangles[\s\S]*useWholeMeshForMissingTargets[\s\S]*std::nth_element' 'Whole-mesh finger fallback must rank and cap high-poly mesh candidates instead of accepting every triangle.'
Require-Text 'src/RockConfig.cpp' 'kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier\s*=\s*4\.5f' 'Loose non-equipped weapon custom authority must preserve the HIGGS-style 9000-vs-2000 base linear force ratio.'
Require-Text 'src/RockConfig.cpp' 'kDefaultGrabLooseWeaponSharedConstraintAngularForceMultiplier\s*=\s*2\.0f' 'Loose non-equipped weapon angular authority must boost rotation without changing linear pull authority.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'tryResolveLivePalmAnchorReference[\s\S]*tryResolveLiveBodyWorldTransform' 'Grab pivot authority must resolve from the live palm-anchor body frame.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'flushPendingHeldNativeGrab' 'Physics step coordinator must not flush removed native mouse-spring authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_rightHand\.flushPendingCustomGrabAuthority\(world,\s*timing,\s*consumptionControllerRoot,\s*scale\.havokToGame\)' 'Right custom grab proxy authority must use the shared post-character-movement root sample.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_leftHand\.flushPendingCustomGrabAuthority\(world,\s*timing,\s*consumptionControllerRoot,\s*scale\.havokToGame\)' 'Left custom grab proxy authority must use the shared post-character-movement root sample.'
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
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.captureHeldReleaseMotion\(hknp,\s*handInput\.rawHandWorld,\s*frame\.deltaSeconds\);[\s\S]*hand\.releaseGrabbedObject' 'Normal grip release must capture the current-frame controller/body velocity sample before release velocity is composed.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Letting normal near/far queries refresh this selection can orphan' 'Selection refresh must be frozen while an arrived pull-catch owns the ref/body.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_state == HandState::SelectedClose[\s\S]*!_currentSelection\.isFarSelection[\s\S]*_currentSelection\.bodyId\.value == _pullCatchIntent\.primaryBodyId' 'Pending pull-catch commit must require the original close body and must not match a far selection refresh.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct PullCatchIntent' 'Far-pull catch state must be explicit hand lifecycle state, not inferred from a stale input edge.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'evaluateConvergencePromotion' 'Convergence timeout promotion must be a testable policy with stability gating.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'struct PullCatchSeatSafetyInput[\s\S]*grabbedFromPullCatch[\s\S]*usingPinchPocket[\s\S]*struct PullCatchSeatSafetyDecision[\s\S]*allowImmediateTouchHeld[\s\S]*requireSettledVisualRelation' 'Pull-catch palm seating safety must be an explicit testable policy contract.'
Reject-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'allowPulledAdjust|pulledAdjustDistanceGameUnits' 'The fixed-distance pulled-grab adjust is retired: its desired-world translation was erased by pivot re-alignment at freeze. The seat depth stop owns surface seating.'
Reject-Text 'src/RockConfig.cpp' 'fPulledGrabHandAdjustDistanceGameUnits' 'The retired pulled-grab adjust must not keep a dead config key that pretends to tune seating.'
Reject-Text 'src/RockConfig.cpp' 'bCalibratedGrenadeOffsetsEnabled' 'The hardcoded grenade/Molotov calibrations are retired; throwables seat through the generic machinery until fresh presets are captured.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_pullPresentationAxisBodyLocal = \{\};[\s\S]*_pullPresentationValid = false;' 'Pull runtime cleanup must clear the presentation axis state.'
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
Require-Text 'tests/GrabContactPatchClusterPolicyTests.cpp' 'long same-face line cannot author pivot' 'Policy tests must prove a thin same-face patch line cannot move pivot B.'
Require-Text 'src/RockConfig.cpp' 'std::clamp\(rockGrabContactPatchProbeCount,\s*1,\s*9\)' 'Config loading must allow the full nine-sample contact patch pattern.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'evaluateSeatedPalmPocketPromotion' 'Seated palm-pocket promotion must be a testable policy, not an ad hoc runtime branch.'
Reject-Text 'src/physics-interaction/grab/GrabMotionController.h' 'currentContactPatchUsedAsPivot|contactPatchUsedAsPivot|decision\.contactPatchSampleCount' 'Contact patch evidence must not publish held-time pivot support or mutate active authority.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'awaitingSettledVisualRelation' 'Visual hand publish gate must expose the settled-relation block reason.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'chooseMeshBackedPatchPivotAuthority[\s\S]*contactPatchPivotEvidenceOnly' 'Grab contact patch pivot policy must keep validated patches as evidence-only instead of frozen pivot authority.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'gripPointSourceNodeLocal' 'Grab frame must preserve mesh evidence in source-node local space instead of treating collidable-node local coordinates as universal mesh authority.'

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

Reject-Text 'src/physics-interaction/grab/GrabCore.h' 'visualAuthorityContactValid|visualAuthorityContactReason|wholeHandVisualAuthority|visualTranslationAuthority|visualRotationAuthority|GrabMotorPivot|motorPivot|hasSurfaceFrame|surfaceFrameLocal|orientationModeUsed|surfaceAlignmentDecision|hasOppositionFrame|oppositionFrameReason|oppositionThumb|oppositionOpposing|surfacePointWorldAtGrab|surfacePointLocal|surfaceHitLocal|surfaceNormalLocal|surfacePointBodyLocalGame|surfacePivotToSurfaceDistanceGameUnits|surfaceSelectionToMeshDistanceGameUnits|surfaceTriangleIndex|surfaceShapeKey|surfaceShapeCollisionFilterInfo|surfaceHitFraction|hasSurfaceHit' 'Grab frame state must not keep legacy visual-authority, surface-frame, opposition-frame, motor-pivot, or surface-named canonical grip state.'
Reject-Text 'src/RockConfig.h' 'rockGrabUseBoneDerivedPalmPivot' 'Grab pivot capture must not keep a dead live hknp palm-anchor readback config switch.'
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
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'Patch/contact/lever quality remains available to release safety[\s\S]*not live motor authority' 'Grab motor policy must document that patch quality cannot weaken live held motor force.'

if ($failures.Count -gt 0) {
    Write-Host 'Hand grab native boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Hand grab native boundary passed.'

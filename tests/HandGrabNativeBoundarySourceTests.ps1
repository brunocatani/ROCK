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

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(world,\s*handWorldTransform,\s*&handBodyWorldAtGrab,\s*proxyFrameWorldAtGrab[\s\S]*grabAuthorityPivotAWorld\s*=\s*proxyFrameWorldAtGrab\.translate[\s\S]*grabPivotAForPrimaryChoice\s*=\s*grabAuthorityPivotAWorld' 'Close dynamic grab must resolve the hidden proxy frame before mesh/contact acquisition.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'The hidden proxy is body A[\s\S]*RE::NiPoint3\s+grabPivotAWorld\s*=\s*grabAuthorityPivotAWorld[\s\S]*buildSplitGrabFrameFromDesiredObject\(\s*handWorldTransform,\s*proxyFrameWorldAtGrab[\s\S]*createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*handWorldTransform,\s*grabPivotAWorld' 'Close dynamic grab must commit body-A pivot from the same resolved proxy origin used by acquisition.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'palmPocketPivotWorld\s*=\s*computeGrabStartupCapturePivotAWorld' 'Close dynamic grab must not freeze an offset startup pivot into the proxy body-A local pivot.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeGrabStartupCapturePivotAWorld\(world,\s*handWorldTransform\)' 'Close dynamic grab post-prep authority must not use the startup raw-rotation capture pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*handWorldTransform,\s*grabPivotAWorld' 'Close dynamic grab must create the hidden proxy plus custom finite-force constraint from the resolved palm-authority proxy frame while preserving raw hand rotation authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrab\.create\(\s*world,\s*objectBodyId' 'Ordinary dynamic close grab must not create native mouse-spring as its production authority.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'solveAdaptiveHeldLead|nativeTargetBodyWorld|_nativeGrab\.queueTarget' 'Held-object updates must not keep removed native mouse-spring adaptive target logic.'
Reject-Text 'src/physics-interaction/grab/GrabHeldObject.h' 'AdaptiveHeldLead|solveAdaptiveHeldLead|responseFactor' 'Removed native/adaptive target-leading helpers must not remain as unused grab authority scaffolding.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'makeRawRotationPalmTranslationFrame\(handWorldTransform,\s*proxyAuthorityWorld\)' 'Held-object targets must use the root-flattened raw hand rotation and live palm/proxy translation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activePivotBBodyLocalGame\s*=\s*activeProxyConstraintPivotBLocalGame\(\)' 'Held-object updates must refresh the active proxy body-local pivot before composing target points.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'localPointToWorld\(desiredBodyWorld,\s*activePivotBBodyLocalGame\)' 'Held-object proxy target point must derive from the same active body-local pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabAuthorityBodyWorldTransform[\s\S]*tryGetBodyArrayWorldTransform' 'Proxy-constraint object-side authority must keep using the BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy-constraint and native object-side reads must use the rigid BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintUsesMotionBodyAtGrab\s*=\s*false' 'Proxy-constraint grab capture must keep body-B constraint data in the rigid BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*grabBodyWorldAtGrab' 'Proxy-constraint grab capture must encode body-B pivots and desired targets in the rigid BODY frame.'

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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.bodyLocal\s*=\s*objectToBodyAtGrab' 'Dynamic grab must preserve visual object rotation by composing desired BODY targets through the captured visual-to-BODY relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'freezePivotBBodyLocal\(grabBodyWorldAtGrab,\s*grabGripPoint\)' 'Grab pivotB must keep a BODY-local copy for visual/native/release handling.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'freezePivotBBodyLocal\(constraintBodyWorldAtGrab,\s*grabGripPoint\)' 'Proxy constraint must freeze the same selected grip point in the hknp solver frame for transform-B.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.objectBodyWorld\s*=\s*grabBodyWorldAtGrab' 'Three-phase grip area must capture body-local grip data in the native BODY grab authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'relationMode\s*=\s*useAuthoredGrabFrame\s*\?\s*"authoredGrabNodeFrame"\s*:\s*"rockPointToPalm"' 'Grab commit must use ROCK point-to-palm for generic grabs and reserve rotation override for authored grab nodes only.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'shiftObjectToAlignGripWithPocket\(\s*objectWorldTransform,\s*grabPivotAWorld,\s*grabGripPoint\s*\)' 'Grab commit must preserve object rotation and translate the selected grip point to the palm anchor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if\s*\(useAuthoredGrabFrame\)[\s\S]*buildDesiredObjectWorldFromAuthoredGrabNode' 'Authored grab node rotation must be an explicit branch, not generic surface/ray authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'no object-side contact point[\s\S]*object origin/COM fallback is not valid dynamic grab authority' 'Dynamic grab must fail when no contact/authored object-side pivot exists instead of falling back to object origin or COM.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabPivotAWorld\s*=\s*pocket\.palmCenterWorld' 'Three-phase grab capture must seat the selected point at the palm anchor, not the depth-offset pocket center.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'threePhaseTouchReachedFrozenRelation' 'Near/far convergence must transition to held without recapturing the object-hand relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'threePhaseTimeoutInsidePocket' 'Near/far convergence timeout must promote to held inside the hand pocket instead of only logging.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildAcquisitionFingerPose' 'Near/far convergence must publish an acquisition finger pose before final touch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'publishLocalTransforms && g_rockConfig\.rockGrabMeshLocalTransformPoseEnabled' 'Acquisition finger pose must not publish surface local-transform corrections before final touch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockPulledGrabHandAdjustDistanceGameUnits' 'Pulled object grabs must keep the hand-back adjustment as explicit tuning.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildHeldObjectRelativeHandWorld\(heldVisualNodeWorld,\s*_grabFrame\.rawHandSpace\)' 'Held visual hand target must use the ROCK held-relative relation from the live held object and frozen raw hand-space relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyGrabExternalHandWorldTransform\(_isLeft,\s*_grabVisualHandTransform\)' 'Held visual hand/arm pose must be published as an external FRIK transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'visual held-object hand target exceeded max deviation' 'Visual hand/arm target must release instead of pulling the rendered hand away indefinitely.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'relationPivotErr=.*rotationPreservedDeg=.*bodyTargetNodeErr=.*normalAuthority=.*authoredRotation' 'Grab telemetry must log the relation invariants needed to verify the plan in screenshots/logs.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct\s+GrabForceTorqueDebugSnapshot' 'Focused grab force/torque overlay must expose a case-specific snapshot instead of reusing noisy generic markers.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'This view is intentionally built from the same BODY-local pivot' 'Focused grab force/torque overlay must document why it follows active BODY pivot authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'bDebugDrawGrabForceTorque|rockDebugDrawGrabForceTorque' 'Focused grab force/torque overlay must be controlled by its own debug toggle.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabPivotSourceCollider' 'Focused grab force/torque overlay must draw the hknp body/collider owning pivot B.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GrabPivotSourceTriangle' 'Focused grab force/torque overlay must draw the mesh triangle evidence that selected the pivot when available.'
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
Require-Text 'data/config/ROCK.ini' 'bDebugDrawGrabForceTorque\s*=\s*false' 'Packaged ROCK.ini must expose focused grab force/torque visualization disabled by default.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform\(world,\s*_savedObjectState\.bodyId' 'Held-object convergence and active-drive pivot telemetry must use the drive-specific object frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'motionDiagVsGrab' 'Grab telemetry must expose MOTION/COM diagnostic drift from native BODY grab authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'havok_runtime::tryReadMotionVelocityCaps' 'Hand grab motion diagnostics must read hknp velocity caps through the native runtime boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'havok_runtime::snapshotMotionPropertiesLibrary' 'Hand grab motion-property library diagnostics must read hknp library layout through the native runtime boundary.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kMotion_MaxLinearVelocityPacked|kMotion_MaxAngularVelocityPacked|kHknpWorld_MotionPropertiesLibraryPtr|kMotionPropertiesLibrary_Entries|kMotionPropertiesLibrary_Count|kMotionProperties_RecordSize|motionPtr\s*\+\s*0x3A|motionPtr\s*\+\s*0x3C|worldPtr\s*\+\s*0x5D0|libraryPtr\s*\+\s*0x28|libraryPtr\s*\+\s*0x30' 'HandGrab must not carry raw hknp motion diagnostic or motion-property library layout reads.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'SolvedGrabFingerPose\s+_grabFingerPose' 'Grab finger pose must be stored at commit instead of re-solving from the live dynamic body every update.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFingerPose\s*=\s*fingerPose' 'Grab commit must capture the mesh-solved finger pose in the canonical grab state.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'captureSurfaceAimObjectLocal\(fingerPose,\s*objectWorldTransform\)' 'Grab commit must freeze mesh-solved surface aim points in object-local space.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'auto publishFingerPose\s*=\s*_grabFingerPose[\s\S]*resolveSurfaceAimObjectLocal\(_grabFingerPose,\s*currentNodeWorld\)[\s\S]*applyRockGrabHandPose\(_isLeft,\s*publishFingerPose' 'Held-object updates must reapply the captured finger pose with object-local surface aim, not re-solve from the moving physics body.'
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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'drive=\{\} bodyDriveMode=\{\} driveReason=\{\} forceScale=\{:\.2f\} linearScope=\{\} angularScope=\{\} massScope=\{\} looseWeapon=' 'Dynamic grab creation telemetry must identify runtime drive mode, body-set drive policy, scopes, and loose-weapon status.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveMode=nativeMouseSpring[^\r\n]*constraintTau' 'Native mouse-spring telemetry must not present shared constraint tau as active native tuning.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'isLooseWeaponGrabTarget[\s\S]*grab_target::canUseRockActiveGrab[\s\S]*RE::ENUM_FORM_ID::kWEAP' 'Loose weapon detection must be limited to normal active grab refs, not equipped weapon support or actor equipment.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'acceptsSelectedMultibodyOwnerlessVisualMesh[\s\S]*bodySet\.acceptedCount\(\)\s*>\s*1[\s\S]*!surfaceOwnerRecord[\s\S]*nodeIsOrDescendsFrom\(bodySet\.rootNode,\s*surfaceOwnerNode\)' 'Ownerless visual mesh fallback must be limited to selected multipart object-family nodes with no concrete accepted owner record.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'surfaceOwnerMatchesResolvedBody\s*=[\s\S]*surfaceOwnerRecord->bodyId\s*==\s*primaryChoice\.bodyId[\s\S]*acceptsSelectedMultibodyOwnerlessVisualMesh\(sel,\s*preparedBodySet,\s*primaryChoice\.bodyId' 'Grab commit owner resolution must accept ownerless same-family multipart mesh evidence without accepting concrete mismatched owners.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimeGrabContactPatch[\s\S]*acceptsSelectedMultibodyOwnerlessVisualMesh\(selection,\s*bodySet,\s*resolvedBodyId' 'Contact-patch mesh recovery must use the same ownerless multipart family rule as grab commit.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_heldObjectIsLooseWeapon\s*=\s*looseWeaponGrab' 'Held state must remember non-equipped dynamic weapon refs for runtime telemetry and neutral multipliers.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier' 'Loose non-equipped weapon proxy-constraint tuning must have an explicit neutral linear tau multiplier surface.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponAdaptiveLeadMultiplier' 'Loose non-equipped weapon native/adaptive lead tuning must be removed with mouse-spring authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier' 'Loose non-equipped weapon shared-constraint tuning must have an explicit neutral linear tau multiplier surface.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintAngularForceMultiplier' 'Loose non-equipped weapon shared-constraint tuning must have an explicit neutral angular force multiplier surface.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.h' 'struct GrabConstraintMotorTuning' 'Shared constraint creation must accept a full linear/angular motor tuning profile.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rejectedFixedOrNonDynamicCount\s*=[\s\S]*bodySetRejectCount\(preparedBodySet,\s*physics_body_classifier::BodyRejectReason::StaticMotion\)[\s\S]*bodySetRejectCount\(preparedBodySet,\s*physics_body_classifier::BodyRejectReason::NotDynamicAfterActivePrep\)' 'Body-set drive classification must treat only post-prep fixed/non-dynamic bodies as fixed-attached evidence.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rejectedFixedOrNonDynamicCount\s*=[\s\S]*bodySetRejectCount\(beforePrepBodySet,\s*physics_body_classifier::BodyRejectReason::StaticMotion\)' 'Body-set drive classification must not treat successfully converted pre-prep static bodies as fixed attachments.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'readHeldBodyMassSummary\(\s*world,\s*_savedObjectState\.bodyId,\s*_heldBodyIds,\s*_heldDriveDecision\.includeConnectedMass\)' 'Dynamic grab motor budgeting must route held mass through body-set drive scope, not only the selected primary body.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.mass\s*=\s*massSummary\.motorMass\(\)' 'Dynamic grab motor target solving must receive the aggregate held-body mass summary.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '\.mass\s*=\s*readBodyMass\(world,\s*_savedObjectState\.bodyId\)' 'Dynamic grab motor target solving must not return to primary-body-only mass.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pendingHeldAuthority\s*=\s*evaluateRuntimeHeldAuthority\([\s\S]*updateConstraintGrabDriveMotors\([\s\S]*pendingHeldAuthority[\s\S]*angularDriveOk\s*=[\s\S]*_activeConstraint\.usesRagdollAngularMotorAtom\(\)[\s\S]*_activeConstraint\.linearMotor[\s\S]*_activeConstraint\.angularMotor' 'Proxy authority flush must compute one runtime held-authority state and apply it through the solver-owned angular motor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseAuthority\s*=\s*evaluateRuntimeHeldAuthority\([\s\S]*releaseAngularVelocityCap\s*=\s*grab_motion_controller::computeAuthorityScaledAngularVelocityCap[\s\S]*releaseAngularVelocity\s*=\s*clampAngularVelocityVector\(releaseAngularVelocity,\s*releaseAngularVelocityCap\)' 'Release angular velocity must reuse held authority for the final angular cap.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'refreshHeldAuthoritySupport' 'Held-time support refresh must be an explicit hand method, not an implicit pivot-reacquire side effect.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'refreshHeldAuthoritySupport\(world,\s*handWorldTransform,\s*proxyAuthorityWorld,\s*activePivotBBodyLocalGame\)[\s\S]*heldAuthority\s*=\s*evaluateRuntimeHeldAuthority' 'Held support refresh must update authority metadata before runtime held authority is evaluated.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'setHeldAngularVelocity|applyProxyConstraintAngularVelocityDrive|world->SetBodyAngularVelocity' 'Dynamic grab must not retain the removed direct angular velocity writer.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.h' 'NativeHardKeyframeVelocity|grabAngularAuthorityFromConfig' 'Grab angular authority must not retain the removed mode-0 selector.'
Reject-Text 'src/RockConfig.cpp' 'iGrabAngularAuthorityMode|rockGrabAngularAuthorityMode' 'Config loading must not expose the removed angular authority selector.'
Reject-Text 'data/config/ROCK.ini' 'iGrabAngularAuthorityMode' 'Packaged ROCK.ini must not expose the removed angular authority selector.'
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

$heldSupportRefreshText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$heldSupportRefreshStart = $heldSupportRefreshText.IndexOf('bool Hand::refreshHeldAuthoritySupport')
$heldSupportRefreshEnd = if ($heldSupportRefreshStart -ge 0) { $heldSupportRefreshText.IndexOf('void Hand::clearGrabAuthorityProxyRuntimeLocked', $heldSupportRefreshStart) } else { -1 }
if ($heldSupportRefreshStart -lt 0 -or $heldSupportRefreshEnd -lt 0) {
    $failures.Add('Held support refresh helper boundary could not be located.')
} else {
    $heldSupportRefreshBody = $heldSupportRefreshText.Substring($heldSupportRefreshStart, $heldSupportRefreshEnd - $heldSupportRefreshStart)
    if ($heldSupportRefreshBody -match '_grabFrame\.(pivotBBodyLocalGame|pivotBConstraintLocalGame|rawHandSpace|constraintHandSpace|constraintBodyHandSpace|rawRotationProxyHandSpace|rawRotationProxyBodyHandSpace|gripPointLocal|gripPointBodyLocalGame)\s*=') {
        $failures.Add('Held support refresh must not move the solver pivot or rewrite captured hand/object transforms.')
    }
    if ($heldSupportRefreshBody -notmatch 'evaluateHeldSupportRefresh' -or $heldSupportRefreshBody -notmatch 'keepFrozenPivot=yes') {
        $failures.Add('Held support refresh must route decisions through the pure metadata-only policy and log the frozen-pivot invariant.')
    }
}
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activateHeldObjectBodySet\(world,\s*objectBodyId\.value,\s*_heldBodyIds\)' 'Close grab commit must explicitly wake the accepted held-object body set after zeroing velocities.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeLocalMeshMaxDistanceFromPoint\(_grabFrame\.localMeshTriangles,\s*_grabFrame\.gripPointLocal\)' 'Dynamic grab must capture long-object lever length from the selected grip point and cached local mesh.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseLongObjectAngularScale\s*=\s*grab_motion_controller::computeLongObjectAngularSpeedScale\([\s\S]*rockGrabLongObjectAngularScalingEnabled' 'Release angular velocity cap must apply the configured long-object lever scale.'
Require-Text 'data/config/ROCK.ini' 'bGrabLongObjectAngularScalingEnabled\s*=\s*true' 'Packaged ROCK.ini must expose long-object angular scaling.'
Require-Text 'data/config/ROCK.ini' 'fGrabLongObjectReferenceLeverGameUnits\s*=\s*24\.0' 'Packaged ROCK.ini must expose the long-object reference lever length.'
Require-Text 'data/config/ROCK.ini' 'fGrabLongObjectMinAngularScale\s*=\s*0\.35' 'Packaged ROCK.ini must expose the long-object angular scale floor.'
Reject-Text 'data/config/ROCK.ini' 'MouseSpring|iGrabObjectRotationReferenceMode|fGrabLooseWeaponNative|fGrabLooseWeaponAdaptiveLeadMultiplier' 'Packaged ROCK.ini must not expose removed mouse-spring, native loose-weapon, adaptive-lead, or rotation-mode switches.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponSharedConstraintLinearTauMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini must expose neutral loose non-equipped weapon shared linear tau tuning.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponSharedConstraintAngularForceMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini must expose neutral loose non-equipped weapon shared angular force tuning.'
Require-Text 'src/RockConfig.cpp' 'kDefaultGrabLooseWeaponSharedConstraintMaxForceMultiplier\s*=\s*4\.5f' 'Loose non-equipped weapon custom authority must preserve the HIGGS-style 9000-vs-2000 base linear force ratio.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponSharedConstraintMaxForceMultiplier\s*=\s*4\.5' 'Packaged ROCK.ini must use the loose weapon base linear force multiplier before mass capping.'
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
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.captureHeldReleaseMotion\(hknp,\s*handInput\.rawHandWorld,\s*_heldObjectPlayerSpaceFrame,\s*frame\.deltaSeconds\);[\s\S]*hand\.releaseGrabbedObject' 'Normal grip release must capture the current-frame controller/body velocity sample before release velocity is composed.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'composeControllerReleaseAngularVelocity' 'Controller-derived release angular velocity must use the pure capped policy instead of raw hand angular velocity.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'gamePointToHavokPoint\(transform_math::localPointToWorld\(releaseBodyWorld,\s*activeProxyConstraintPivotBLocalGame\(\)\)\)' 'Tangential throw velocity must use the active proxy-constraint grab pivot lever arm when the body frame is readable.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'Letting normal near/far queries refresh this selection can orphan' 'Selection refresh must be frozen while an arrived pull-catch owns the ref/body.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_state == HandState::SelectedClose[\s\S]*!_currentSelection\.isFarSelection[\s\S]*_currentSelection\.bodyId\.value == _pullCatchIntent\.primaryBodyId' 'Pending pull-catch commit must require the original close body and must not match a far selection refresh.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'struct PullCatchIntent' 'Far-pull catch state must be explicit hand lifecycle state, not inferred from a stale input edge.'
Require-Text 'src/physics-interaction/grab/GrabThreePhase.h' 'evaluateConvergencePromotion' 'Convergence timeout promotion must be a testable policy with stability gating.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeAcquisitionVisualEnvelopeGameUnits' 'Pre-touch visual hand authority must use a bounded acquisition envelope instead of the full near-converge distance.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'grabbedFromPullCatch && !useAuthoredGrabFrame' 'Pulled hand-back adjustment must not bias authored grab-node transforms.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void clearGrabHandPose\(bool isLeft\)' 'Hand reset/world-loss cleanup must have one explicit helper for clearing the ROCK_Grab FRIK tag.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void Hand::reset\(\)[\s\S]*clearGrabHandPose\(_isLeft\)[\s\S]*clearGrabExternalHandWorldTransform\(_isLeft\)' 'Hand reset must clear both ROCK_Grab pose and ROCK_GrabVisual external transform tags.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void Hand::abandonHavokStateAfterWorldLoss\(\)[\s\S]*clearGrabHandPose\(_isLeft\)[\s\S]*clearGrabExternalHandWorldTransform\(_isLeft\)' 'World-loss abandon must clear both ROCK_Grab pose and ROCK_GrabVisual external transform tags.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'if\s*\(!node \|\| !node->IsTriShape\(\)\)\s*\{[\s\S]*return false;[\s\S]*blacklistedShapes' 'Grab mesh extraction blacklist must skip matching geometry only, not marker parent subtrees.'
Require-Text 'src/physics-interaction/grab/MeshGrab.h' 'findClosestGrabSurfaceHitToPointPositionOnly' 'Grab mesh lookup must provide a position-only palm-pocket query so bad normals do not discard usable close-grab pivot evidence.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'kContactPatchProbePatternSampleCount\s*=\s*9' 'Grab contact patch sampling must expose a bounded nine-point palm-plane probe pattern for thin/small objects.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'buildContactPatchProbeOffsets' 'Grab contact patch probe offsets must be testable pure policy, not embedded only in the Havok runtime loop.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'computeContactPatchProbeGeometry' 'Grab contact patch probe geometry must be scaled by pure object-size policy.'
Require-Text 'src/physics-interaction/grab/GrabCore.h' 'kMaxGrabContactPatchSamples\s*=\s*grab_contact_patch_math::kContactPatchProbePatternSampleCount' 'Grab frame contact patch storage must track the bounded probe policy max.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildContactPatchProbeOffsets' 'Runtime grab contact patches must consume the shared bounded probe pattern.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'contactPatchObjectLeverEstimateGameUnits[\s\S]*computeLocalMeshMaxDistanceFromPoint\(grabLocalMeshTriangles,\s*canonicalPivotLocal\)[\s\S]*buildRuntimeGrabContactPatch\([\s\S]*contactPatchObjectLeverEstimateGameUnits' 'Runtime contact-patch probes must receive object lever evidence before casting.'
Require-Text 'src/RockConfig.cpp' 'std::clamp\(rockGrabContactPatchProbeCount,\s*1,\s*9\)' 'Config loading must allow the full nine-sample contact patch pattern.'
Require-Text 'data/config/ROCK.ini' 'iGrabContactPatchProbeCount\s*=\s*9' 'Packaged ROCK.ini must enable the nine-sample contact patch pattern by default.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'closePalmPocketMeshAuthority' 'Close seated grabs must publish palm-pocket mesh authority before generic mesh fallback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'SelectionHitMeshSnap[\s\S]*requiresSettledVisualHandRelation' 'Selection-hit mesh snaps that start outside touch must wait for seated palm-pocket visual relation before publishing the visual hand.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'reachedTouchMayPromote\s*=\s*reachedTouchRange\s*&&\s*seatedReacquireSatisfied' 'Touch-range promotion must not bypass required seated pivot reacquire.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'clearGrabExternalHandWorldTransform\(_isLeft\)[\s\S]*_hasGrabVisualHandTransform\s*=\s*false[\s\S]*THREE-PHASE GRAB SEATED PALM-POCKET PROMOTION' 'Seated pivot promotion must clear any previous visual hand relation before publishing again.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'evaluateSeatedPalmPocketPromotion' 'Seated palm-pocket promotion must be a testable policy, not an ad hoc runtime branch.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildSeatedPalmPocketSupportPatch[\s\S]*buildContactPatchProbeOffsets' 'Seated mesh-start promotion must rebuild bounded palm-pocket support samples instead of staying at a one-point mesh refresh.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'seatedPalmPocketRetarget[\s\S]*requiresSettledVisualHandRelation\s*=\s*!promotionDecision\.completeSeatedRelation' 'Medium-distance seated palm-pocket promotion must retarget over frames without claiming the visual relation is settled early.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'copyCompletedSeatedPivotSupportToGrabFrame[\s\S]*contactPatchSamples[\s\S]*contactPatchSampleCount[\s\S]*contactPatchUsedAsPivot\s*=\s*_grabFrame\.contactPatchSampleCount\s*>\s*0' 'Completed seated palm-pocket promotion must update the active pivot support sample model.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'clearUnsettledSeatedContactPatchAuthority[\s\S]*contactPatchSampleCount\s*=\s*0[\s\S]*contactPatchUsedAsPivot\s*=\s*false' 'Partial seated palm-pocket retarget must clear active contact-patch authority until the visual relation is settled.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'countFreshSemanticFingerContactGroups' 'Seated palm-pocket promotion must not turn raw semantic finger counts into motor authority.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'decision\.contactPatchSampleCount\s*=\s*input\.currentContactPatchUsedAsPivot\s*\?[\s\S]*input\.currentContactPatchSampleCount[\s\S]*1u' 'Held support refresh must collapse non-pivot evidence samples to one live candidate sample before publishing pivot support.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimeFingerPoseTargets\(promotedPointWorld,\s*promotedNormalWorld\)[\s\S]*storeFingerPoseTargetsInGrabFrame\(_grabFrame,\s*seatedPoseTargets' 'Seated palm-pocket promotion must rebuild finger pose targets around the promoted palm-pocket seat.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'awaitingSettledVisualRelation' 'Visual hand publish gate must expose the settled-relation block reason.'
Require-Text 'src/physics-interaction/grab/GrabContact.h' 'chooseMeshBackedPatchPivotAuthority[\s\S]*meshBackedPatchPivotPositionOnly' 'Grab contact patch pivot authority must be a named pure policy with a position-only acceptance reason.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'mesh-snapped contact patch can[\s\S]*position-only authority policy[\s\S]*owner, selection, palm-pocket, and score gates' 'Grab pivot authority must document the narrow mesh-backed contact-patch replacement gate.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'positionOnlyPatch\.source\s*=\s*GrabPivotAuthoritySource::ContactPatchPositionOnly' 'Accepted contact-patch pivot authority must remain position-only in runtime telemetry.'
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
Reject-Text 'data/config/ROCK.ini' 'bGrabUseBoneDerivedPalmPivot' 'ROCK.ini must not expose a dead live hknp palm-anchor readback toggle.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'THREE-PHASE GRAB CAPTURE' 'Generic grab commit must pass through the hand-pocket acquisition capture.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'handPocket=palmSeat' 'Generic grab must expose one hand-pocket palm-seat authority path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'configuredGrabOrientationMode|grab_visual_authority_policy|buildDesiredObjectWorldFromSurfaceFrame|buildOppositionDesiredObjectWorld|shouldUseObjectReverseAlignedHandForFingerPose|shouldApplyObjectReverseAlignedExternalHandTransform|evaluateGrabOutputAuthority|GRAB POINT AUTHORITY|GrabMotorPivot|motorPivot|pinchMotorPivot|wholeHandVisualAuthority|visualTranslationAuthority|visualRotationAuthority|hasSurfaceFrame|surfaceFrameLocal|orientationModeUsed|surfaceAlignmentDecision|hasOppositionFrame|oppositionFrameReason' 'Runtime HandGrab must not use legacy surface/opposition/pinch/visual grab authority.'
Reject-Text 'src/physics-interaction/hand/Hand.cpp' 'GrabVisualAuthorityPolicy|grab_visual_authority_policy|trackingFallback' 'Hand adjusted-transform accessors must not preserve legacy object-owned visual hand authority.'
Reject-Text 'src/physics-interaction/grab/GrabContact.h' 'GrabMotorPivot|PinchSmallObject|smallObjectPinch|chooseActiveGrabPoint|grab_surface_frame_math|grab_opposition_frame_math|GrabOrientationMode|GrabSurfaceAlignmentDecision|buildDesiredObjectWorldFromSurfaceFrame|buildOppositionDesiredObjectWorld' 'Grab contact policy must not keep removed generic motor-pivot, surface-frame, opposition-frame, or pinch authority.'
Reject-Text 'src/physics-interaction/grab/GrabContact.h' '"legacyContactSources"|return\s+"legacyPermissive"|reason\s*=.*"legacyPermissive"' 'Grab contact diagnostics must use compatibility/permissive-fallback wording instead of legacy labels.'
Reject-Text 'src/physics-interaction/grab/GrabConstraint.cpp' '\[surface\]' 'Active grab constraint pivot diagnostics must not use stale surface-frame terminology.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.cpp' '\[active-pivot-b\]' 'Active grab constraint pivot diagnostics must label transform-B as the active body-local pivot.'
Reject-Text 'src/physics-interaction/hand/HandVisual.h' 'hand_visual_authority_math|buildAppliedVisualAuthorityHandWorld|buildObjectOwnedReverseAlignedHandWorld|buildSplitFrameReverseAlignedHandWorld|buildHandBoneWorldFromContactFrame' 'Hand visual helpers must not keep removed object-owned reverse visual authority.'
Reject-Text 'src/RockConfig.h' 'rockGrabThreePhaseEnabled|rockGrabObjectVisualHandAuthorityEnabled|rockGrabOrientationMode|rockGrabSmallObjectPinchPivotEnabled|rockGrabUseSemanticFingerContactPivot|rockGrabOppositionFrameEnabled|rockGrabThreePhasePreserveRotationDuringConverge|rockGrabThreePhaseDelayFingerPoseUntilTouch|rockGrabThreePhaseDisableVisualHandAuthority' 'RockConfig must not expose toggles that can re-enable legacy grab authority.'
Reject-Text 'data/config/ROCK.ini' 'bGrabThreePhaseEnabled|bGrabObjectVisualHandAuthorityEnabled|iGrabOrientationMode|bGrabSmallObjectPinchPivotEnabled|bGrabUseSemanticFingerContactPivot|bGrabOppositionFrameEnabled|bGrabThreePhasePreserveRotationDuringConverge|bGrabThreePhaseDelayFingerPoseUntilTouch|bGrabThreePhaseDisableVisualHandAuthority' 'Packaged ROCK.ini must not expose legacy grab authority toggles.'
Reject-Text 'src/RockConfig.h' 'rockDebugShowGrabSurfaceFrame' 'Debug config must use pocket-normal naming, not stale surface-frame naming.'
Reject-Text 'data/config/ROCK.ini' 'bDebugShowGrabSurfaceFrame' 'Packaged ROCK.ini must use pocket-normal naming, not stale surface-frame naming.'

if ($failures.Count -gt 0) {
    Write-Host 'Hand grab native boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Hand grab native boundary passed.'

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

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_NativeVRGrabDrop\s*=\s*0xF1AB90' 'Native VR drop offset must remain explicit at the verified address.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'typedef void func_t\(void\*,\s*int,\s*std::uint64_t\)' 'Native VR drop wrapper must expose the verified third flag argument.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'func\(playerChar,\s*handIndex,\s*0\)' 'Native VR drop wrapper must pass the game-observed third flag value 0.'

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_MouseSpringAction_Ctor\s*=\s*0x1E4A850' 'Native mouse-spring action constructor offset must remain explicit at the verified FO4VR VR address.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_MouseSpringAction_Update\s*=\s*0x1E4A940' 'Native mouse-spring action update offset must remain explicit at the verified FO4VR VR address.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_MouseSpringAction_SetTargetPosition\s*=\s*0x1E4A960' 'Native mouse-spring target-position setter offset must remain explicit.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_MouseSpringAction_SetTargetTransform\s*=\s*0x1E4A980' 'Native mouse-spring target-transform setter offset must remain explicit.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_MouseSpringLinearResponseBase\s*=\s*0x3754370' 'Native mouse-spring cinfo tuning must read FO4VR native constants instead of invented stiffness.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_MouseSpringAngularResponseBase\s*=\s*0x3754358' 'Native mouse-spring angular response tuning must read the verified FO4VR native constant.'

Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'kMouseSpringActionSize\s*=\s*0xE0' 'Native mouse-spring wrapper must allocate the verified hknpBSMouseSpringAction size.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'kMouseSpringCinfoSize\s*=\s*0xB0' 'Native mouse-spring wrapper must build the verified cinfo size.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'getBodyMotion\(world,\s*bodyId\)' 'Native mouse-spring grab must validate that the held object remains a dynamic body with a readable motion.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'gameToHavokScale\(\)' 'Native mouse-spring target and local pivot must convert game units to Havok units.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'niRowsToHavokColumns' 'Native mouse-spring target orientation must use the shared verified Ni-to-Havok rotation convention.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'makeMouseSpringTargetRotation' 'Native mouse-spring orientation must isolate its native action boundary convention from generated-body rotation conversion.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'transposeRotation\(targetBodyWorldRotation\)' 'Native mouse-spring orientation must preserve the working backup native boundary conversion.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'computeTargetPointWorldGame' 'Native mouse-spring target point must be derived from the held-body target frame and frozen body-local pivot.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'MouseSpringSetTargetPosition_t' 'Native mouse-spring wrapper must update target position through the verified native setter.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'MouseSpringSetTargetTransform_t' 'Native mouse-spring wrapper must update target orientation through the verified native setter.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'MouseSpringUpdate_t' 'Native mouse-spring wrapper must flush Bethesda smoothing through the verified native update function.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.h' 'composeTuningScale\(float baseScale,\s*float multiplier\)' 'Native mouse-spring tuning must compose final applied scale through one shared helper before logging or cinfo handoff.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'kData_MouseSpringLinearResponseBase[\s\S]*sanitizeTuningScale\(tuning\.linearResponseScale\)' 'Native mouse-spring linear response scale must tune the FO4VR native cinfo value, not the shared constraint tau.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'kData_MouseSpringAngularResponseBase[\s\S]*sanitizeTuningScale\(tuning\.angularResponseScale\)' 'Native mouse-spring angular response scale must tune the FO4VR native cinfo value, not the shared constraint tau.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.h' 'mutable std::mutex _mutex' 'Native mouse-spring action state must be synchronized across frame target queueing and Havok-step flushing.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'std::scoped_lock lock\(_mutex\)' 'Native mouse-spring create, queue, flush, destroy, and debug snapshots must lock the action state.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame\(world,\s*handWorldTransform,\s*&handBodyWorldAtGrab,\s*proxyFrameWorldAtGrab' 'Close dynamic grab must resolve the hidden proxy frame from the live palm-anchor authority before creating the custom finite-force constraint.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab,\s*grabPivotAWorld' 'Close dynamic grab must create the hidden proxy plus custom finite-force constraint from the resolved palm-authority proxy frame.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrab\.create\(\s*world,\s*objectBodyId' 'Ordinary dynamic close grab must not create native mouse-spring as its production authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'RE::NiTransform nativeTargetBodyWorld = desiredBodyWorld' 'Held-object adaptive response must start from the body-composed ROCK target frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'solveAdaptiveHeldLead' 'Ordinary native held-object updates must apply the bounded adaptive held-response policy before queueing the native target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrab\.queueTarget\(nativeTargetBodyWorld\)' 'Held-object updates must queue the body-composed/adapted ROCK target frame, not the visual node frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'multiplyTransforms\(handWorldTransform,\s*desiredBodyTransformRawHandSpace\)' 'Held-object native targets must use the root-flattened raw hand frame, not the generated hand body frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activePivotBBodyLocalGame\s*=\s*activeGrabDrivePivotBBodyLocalGame\(\)' 'Held-object updates must refresh the active body-local pivot before composing native or proxy target points.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'computeTargetPointWorldGame\(desiredBodyWorld,\s*activePivotBBodyLocalGame\)' 'Held-object native linear target point must derive from the active body-local pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'localPointToWorld\(desiredBodyWorld,\s*activePivotBBodyLocalGame\)' 'Held-object proxy target point must derive from the same active body-local pivot.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.h' 'targetBodyWorldGame' 'Native mouse-spring debug state must expose the body-composed target frame.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'computeTargetPointWorldGame\(initialTargetBodyWorldGame,\s*localGrabPointBodyGame\)' 'Native mouse-spring creation must derive its target point from the same body-composed target frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetNativeMouseSpringBodyWorldTransform[\s\S]*tryGetBodyArrayWorldTransform' 'Native body-array readback must remain available as diagnostics.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabAuthorityBodyWorldTransform[\s\S]*tryGetNativeMouseSpringBodyWorldTransform' 'Native mouse-spring object-side authority must keep using the BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform[\s\S]*return\s+tryGetGrabAuthorityBodyWorldTransform\(world,\s*bodyId,\s*outTransform\)' 'Proxy-constraint and native object-side reads must use the rigid BODY grab-authority frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintUsesMotionBodyAtGrab\s*=\s*false' 'Proxy-constraint grab capture must keep body-B constraint data in the rigid BODY frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintBodyWorldAtGrab\s*=\s*grabBodyWorldAtGrab' 'Proxy-constraint grab capture must encode body-B pivots and desired targets in the rigid BODY frame.'

$grabDriveTextForBoundary = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/hand/HandGrab.cpp')
$grabDriveStart = $grabDriveTextForBoundary.IndexOf('bool Hand::tryGetGrabDriveObjectWorldTransform')
$grabDriveEnd = if ($grabDriveStart -ge 0) { $grabDriveTextForBoundary.IndexOf('RE::NiPoint3 Hand::activeGrabDrivePivotBBodyLocalGame', $grabDriveStart) } else { -1 }
if ($grabDriveStart -lt 0 -or $grabDriveEnd -lt 0) {
    $failures.Add('Grab drive object-frame helper boundary could not be located.')
} else {
    $grabDriveHelperText = $grabDriveTextForBoundary.Substring($grabDriveStart, $grabDriveEnd - $grabDriveStart)
    if ($grabDriveHelperText -match 'tryResolveLiveBodyWorldTransform|MotionCenterOfMass|HeldObjectDriveMode::ProxyConstraint') {
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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'tryGetGrabDriveObjectWorldTransform\(world,\s*_savedObjectState\.bodyId' 'Held-object convergence and active-drive pivot telemetry must use the drive-specific object frame.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'motionDiagVsGrab' 'Grab telemetry must expose MOTION/COM diagnostic drift from native BODY grab authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'havok_runtime::tryReadMotionVelocityCaps' 'Hand grab motion diagnostics must read hknp velocity caps through the native runtime boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'havok_runtime::snapshotMotionPropertiesLibrary' 'Hand grab motion-property library diagnostics must read hknp library layout through the native runtime boundary.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'kMotion_MaxLinearVelocityPacked|kMotion_MaxAngularVelocityPacked|kHknpWorld_MotionPropertiesLibraryPtr|kMotionPropertiesLibrary_Entries|kMotionPropertiesLibrary_Count|kMotionProperties_RecordSize|motionPtr\s*\+\s*0x3A|motionPtr\s*\+\s*0x3C|worldPtr\s*\+\s*0x5D0|libraryPtr\s*\+\s*0x28|libraryPtr\s*\+\s*0x30' 'HandGrab must not carry raw hknp motion diagnostic or motion-property library layout reads.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'SolvedGrabFingerPose\s+_grabFingerPose' 'Grab finger pose must be stored at commit instead of re-solving from the live dynamic body every update.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFingerPose\s*=\s*fingerPose' 'Grab commit must capture the mesh-solved finger pose in the canonical grab state.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'applyRockGrabHandPose\(_isLeft,\s*_grabFingerPose' 'Held-object updates must reapply the captured finger pose rather than recomputing from a moving physics body.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::flushPendingHeldNativeGrab' 'Native action fallback/diagnostic updates must still be flushed from the Havok step boundary when that mode is active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::flushPendingCustomGrabAuthority' 'Proxy constraint dynamic grab authority must flush from the between-collide-and-solve boundary.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'updateProxyConstraintGrabDriveTarget' 'Proxy constraint dynamic grab must refresh constraint transforms and angular target from the captured palm-authority hand relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueProxyGrabAuthorityTarget' 'Game-frame held updates must queue proxy targets instead of writing solver authority directly.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveMode=\{\} looseWeapon=' 'Dynamic grab creation telemetry must identify the runtime drive mode and loose-weapon status.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveMode=nativeMouseSpring[^\r\n]*constraintTau' 'Native mouse-spring telemetry must not present shared constraint tau as active native tuning.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'isLooseWeaponGrabTarget[\s\S]*grab_target::canUseRockActiveGrab[\s\S]*RE::ENUM_FORM_ID::kWEAP' 'Loose weapon detection must be limited to normal active grab refs, not equipped weapon support or actor equipment.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_heldObjectIsLooseWeapon\s*=\s*looseWeaponGrab' 'Held state must remember non-equipped dynamic weapon refs for runtime telemetry and neutral multipliers.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier' 'Loose non-equipped weapon proxy-constraint tuning must have an explicit neutral linear tau multiplier surface.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponAdaptiveLeadMultiplier' 'Loose non-equipped weapon adaptive lead tuning remains available to any native fallback mode.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier' 'Loose non-equipped weapon shared-constraint tuning must have an explicit neutral linear tau multiplier surface.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'rockGrabLooseWeaponSharedConstraintAngularForceMultiplier' 'Loose non-equipped weapon shared-constraint tuning must have an explicit neutral angular force multiplier surface.'
Require-Text 'src/physics-interaction/grab/GrabConstraint.h' 'struct GrabConstraintMotorTuning' 'Shared constraint creation must accept a full linear/angular motor tuning profile.'
Require-Text 'data/config/ROCK.ini' 'fGrabNativeMouseSpringLinearResponseScale\s*=\s*1\.35' 'Packaged ROCK.ini may keep native mouse-spring response tuning for diagnostics/fallback wrapper coverage.'
Require-Text 'data/config/ROCK.ini' 'fGrabNativeMouseSpringAngularResponseScale\s*=\s*0\.75' 'Packaged ROCK.ini may keep native mouse-spring angular response tuning for diagnostics/fallback wrapper coverage.'
Require-Text 'data/config/ROCK.ini' 'fGrabNativeMouseSpringAngularClampScale\s*=\s*0\.85' 'Packaged ROCK.ini may keep native mouse-spring angular clamp tuning for diagnostics/fallback wrapper coverage.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponNativeLinearResponseMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini may keep neutral loose non-equipped weapon native tuning for diagnostics/fallback wrapper coverage.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponAdaptiveLeadMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini must expose neutral loose non-equipped weapon adaptive lead tuning.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponSharedConstraintLinearTauMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini must expose neutral loose non-equipped weapon shared linear tau tuning.'
Require-Text 'data/config/ROCK.ini' 'fGrabLooseWeaponSharedConstraintAngularForceMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini must expose neutral loose non-equipped weapon shared angular force tuning.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_nativeGrabReleasePending\.store\(true' 'Native mouse-spring flush failure must queue a deterministic held-object release instead of only logging.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'native mouse-spring flush failure marked grab invalid' 'Held-object update must consume native flush failures and release the stale grab state.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'tryResolveLivePalmAnchorReference[\s\S]*tryResolveLiveBodyWorldTransform' 'Grab pivot authority must resolve from the live palm-anchor body frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_rightHand\.flushPendingHeldNativeGrab\(world,\s*timing\)' 'Right held-object native action must flush from the physics-step coordinator.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_leftHand\.flushPendingHeldNativeGrab\(world,\s*timing\)' 'Left held-object native action must flush from the physics-step coordinator.'
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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'gamePointToHavokPoint\(transform_math::localPointToWorld\(releaseBodyWorld,\s*activeGrabDrivePivotBBodyLocalGame\(\)\)\)' 'Tangential throw velocity must use the active held-object grab pivot lever arm when the body frame is readable.'
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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'HeldObjectDriveMode::ProxyConstraint[\s\S]*createProxyConstraintGrabDrive' 'Ordinary dynamic close grabs must use the custom proxy constraint authority.'
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
Reject-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'SetBodyKeyframed|setBodyKeyframed|kFunc_SetBodyKeyframed' 'Native mouse-spring grab must drive a dynamic body by velocity, not keyframe the held object.'
Reject-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'return\s+targetBodyWorldRotation;' 'Native mouse-spring orientation must not bypass the working backup boundary conversion.'
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

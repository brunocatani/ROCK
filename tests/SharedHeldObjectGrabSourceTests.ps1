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

Require-Text 'src/physics-interaction/hand/HandSelection.h' 'OtherHandSelectionContext' 'Selection must distinguish exclusive peer refs from shareable peer-held refs.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'allowsSharedHeldReference' 'Far selection must not silently pull objects already held by the peer hand.'
Reject-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'ref\s*==\s*otherHandRef' 'Object detection must not reject the peer-held ref with the old blanket equality guard.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'makeGrabSharedObjectContext' 'Grab commit must pass peer-held object snapshots into the hand runtime.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'makeGrabReleaseContext' 'Release must know whether the peer hand still holds the same object.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'acquirePeerHeldCloseSelection' 'A delayed second-hand grip press must be able to synthesize a close selection for the peer-held object.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'promoteHeldObjectToConstraintDrive' 'When the second hand joins, the peer hand must already have or promote to custom constraint authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'claimOwnerForHand' 'ROCK-owned object tracking must use per-hand owner masks so pull-to-grab promotion cannot leave stale claims.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'ownerMask\s*\|=\s*claimOwnerBit\(owner\)' 'ROCK-owned object tracking must claim each owner idempotently instead of incrementing duplicate same-hand claims.'
Require-Text 'src/physics-interaction/hand/Hand.h' 'enum class HeldObjectDriveMode[\s\S]*NativeMouseSpring[\s\S]*ProxyConstraint' 'Held objects must explicitly track native diagnostic/fallback and proxy custom constraint drive modes.'
Reject-Text 'src/physics-interaction/hand/Hand.h' 'SharedConstraint' 'Held-object drive modes must not retain the removed semantic hand-body shared constraint fallback.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'peer-held close selection acquired' 'Delayed shared grabs must log the narrow peer-held close acquisition path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'copyPeerInertiaSnapshot' 'The second hand must inherit original inertia snapshots instead of saving already-normalized values.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive' 'The joining hand must create a ROCK proxy constraint drive instead of a second native-only translation drive.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createConstraintGrabDrive' 'Joining-hand promotion must not resurrect the removed semantic hand-body shared constraint creator.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'updateProxyConstraintGrabDriveTarget' 'Shared held-object updates must refresh angular target and transform-B pivot through the proxy constraint math path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'bool\s+Hand::updateConstraintGrabDriveTarget' 'Shared held-object target updates must not keep the removed generated hand-body readback authority path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'sharedGrabAuthorityForceScale\(releaseContext\.peerHandStillHolding\)' 'Two-hand loose-object updates must share one finite force budget instead of giving each hand full authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'sharedGrabAuthorityForceScale\(joiningPeerHeldObject\)' 'The joining hand must start with a shared force budget on the same frame it creates proxy authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_heldDriveMode == HeldObjectDriveMode::ProxyConstraint[\s\S]*sharedGrabAuthorityForceScale\(true\)' 'A peer hand already using proxy authority must immediately switch its pending force budget to shared mode.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pending\.authorityForceScale' 'Between-phase proxy motor updates must consume the queued shared force budget.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'authorityForceScale' 'The pure motor controller must scale final mass-capped force by per-hand authority share.'
Require-Text 'src/RockConfig.cpp' 'rockGrabAdaptiveMaxForceMultiplier\s*=\s*1\.0f' 'Default dynamic grab force must stay finite instead of applying a HIGGS-incompatible 4x lag force boost.'
Require-Text 'src/RockConfig.h' 'rockGrabAdaptiveMaxForceMultiplier\s*=\s*1\.0f' 'Header default dynamic grab force must match the finite source default.'
Require-Text 'data/config/ROCK.ini' 'fGrabAdaptiveMaxForceMultiplier\s*=\s*1\.0' 'Packaged ROCK.ini must keep adaptive max-force amplification neutral by default.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintDrivePivotBBodyLocalGame' 'Constraint grabs must create and measure against the active transform-B pivot, not the native frozen surface pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activePivotBBodyLocalGame' 'Held-object error, convergence, and debug logging must use the active constraint pivot for shared grabs.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'updateConstraintGrabDriveMotors' 'Custom held-object updates must drive both angular and linear motors while held.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseContext\.finalObjectRelease' 'Held object restoration must be gated on final shared-object release.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'player\s*&&\s*!joiningPeerHeldObject' 'Joining an already ROCK-held object must not call native VR drop on the peer hand.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'evaluateSemanticPivotCandidate' 'Delayed peer-held close selection must prefer fresh hand contact evidence before falling back to body origin.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'const auto transition = applyTransition\(HandTransitionRequest\{ \.event = HandInteractionEvent::SelectionFoundClose \}\)' 'Peer-held fallback selection must commit only after the state machine accepts close selection.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'refreshedPeerHeldSelection' 'A delayed second-hand press must refresh the peer-held close candidate before grab commit.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'clearSelectionState\(false\)' 'Stale peer-held close selections must be cleared when the refreshed reach check fails.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'acquireBodyFlagLease' 'Shared held-object body flags must be leased across hands instead of raw enabled/disabled per release.'
Require-Text 'src/physics-interaction/native/NativeMouseSpringGrab.cpp' 'releaseBodyFlagLease' 'Native mouse-spring body flags must remain enabled while the peer hand still holds the object.'

if ($failures.Count -gt 0) {
    Write-Host 'Shared held-object grab source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Shared held-object grab source test passed.'

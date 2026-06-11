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
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'peerHeldCloseSelectionReady' 'Peer-held input intent must not consume a press until a close peer-held selection exists.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'peerHeldRetryCommitIntent' 'Peer-held retry intent must be able to drive grab commit without a fresh pressed edge.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'peer-held join retry started[\s\S]*peer-held join retry cancelled[\s\S]*lastRefusal' 'Peer-held retry diagnostics must log start, cancellation, and the last refusal reason.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'peerHeldCloseCandidate' 'Peer-held input must not treat peer ownership alone as a ready grab consumer.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.hasSelection\(\)\s*&&\s*hand\.getSelection\(\)\.refr\s*!=\s*peer\.getHeldRef\(\)' 'Peer-held fallback must not override unrelated close or far selections.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'hand\.hasSelection\(\)\s*&&\s*!hand\.getSelection\(\)\.isFarSelection\s*&&\s*hand\.getSelection\(\)\.refr\s*!=\s*peer\.getHeldRef\(\)' 'Peer-held fallback must not be guarded only against unrelated close selections.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'promoteHeldObjectToConstraintDrive' 'When the second hand joins, the peer hand must already have or promote to custom constraint authority.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'claimOwnerForHand' 'ROCK-owned object tracking must use per-hand owner masks so pull-to-grab promotion cannot leave stale claims.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'ownerMask\s*\|=\s*claimOwnerBit\(owner\)' 'ROCK-owned object tracking must claim each owner idempotently instead of incrementing duplicate same-hand claims.'
Reject-Text 'src/physics-interaction/hand/Hand.h' 'enum class HeldObjectDriveMode|_heldDriveMode|NativeMouseSpring' 'Held objects must not retain removed drive-mode scaffolding or native mouse-spring fallback.'
Reject-Text 'src/physics-interaction/hand/Hand.h' 'SharedConstraint' 'Held-object drive modes must not retain the removed semantic hand-body shared constraint fallback.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'peer-held close selection acquired' 'Delayed shared grabs must log the narrow peer-held close acquisition path.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_currentSelection\.isValid\(\)\s*&&\s*_currentSelection\.refr\s*!=\s*peerRef' 'Peer-held close acquisition must refuse unrelated far selections before replacing the selected ref.'
Reject-Text 'src/physics-interaction/hand/Hand.cpp' '_currentSelection\.isValid\(\)\s*&&\s*!_currentSelection\.isFarSelection\s*&&\s*_currentSelection\.refr\s*!=\s*peerRef' 'Peer-held close acquisition must not treat unrelated far selections as replaceable.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'copyPeerInertiaSnapshot' 'The second hand must inherit original inertia snapshots instead of saving already-normalized values.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildCommittedHeldBodyIds' 'Shared grabs must centralize committed held-body set ownership instead of assigning directly from a local rescan.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'sharedContext\.peerHeldBodyIds[\s\S]*makePrimaryFirstUniqueBodyList\(primaryBodyId,\s*sourceBodyIds\)' 'A joining hand must inherit the peer accepted body set while keeping its selected primary body first.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_heldBodyIds\s*=\s*buildCommittedHeldBodyIds\(objectBodyId\.value,\s*mechanicalScope\.committedBodyIds,\s*sharedContext,\s*adoptedPeerHeldBodySet\)' 'Grab commit must use the shared body-set helper for mechanical held-body activation, flag leases, release, and restoration.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' '_heldBodyIds\s*=\s*preparedBodySet\.acceptedBodyIds\(\)' 'Grab commit must not discard the peer held-body snapshot when joining a shared object.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive' 'The joining hand must create a ROCK proxy constraint drive instead of a second native-only translation drive.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createConstraintGrabDrive' 'Joining-hand promotion must not resurrect the removed semantic hand-body shared constraint creator.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'updateProxyConstraintGrabDriveTarget' 'Shared held-object updates must refresh angular target and transform-B pivot through the proxy constraint math path.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'bool\s+Hand::updateConstraintGrabDriveTarget' 'Shared held-object target updates must not keep the removed generated hand-body readback authority path.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'sharedGrabAuthorityForceScale\(releaseContext\.peerHandStillHolding\)' 'Two-hand loose-object updates must share one finite force budget instead of giving each hand full authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'sharedGrabAuthorityForceScale\(joiningPeerHeldObject\)' 'The joining hand must start with a shared force budget on the same frame it creates proxy authority.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_activeConstraint\.isValid\(\) && _grabAuthorityProxy\.isValid\(\)[\s\S]*sharedGrabAuthorityForceScale\(true\)' 'A peer hand already using proxy authority must immediately switch its pending force budget to shared mode.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'pending\.authorityForceScale' 'Between-phase proxy motor updates must consume the queued shared force budget.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'authorityForceScale' 'The pure motor controller must scale final mass-capped force by per-hand authority share.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildProxyConstraintMotorTuning[\s\S]*capForceByMass[\s\S]*authorityForceScale' 'Initial proxy constraint creation must seed the same mass-capped shared force budget used by live motor updates.'
Reject-Text 'src/physics-interaction/grab/GrabMotionController.h' 'maxForceMultiplier|massResponsiveMaxForce|positionErrorGameUnits|fullPositionErrorGameUnits|maxTau' 'Held proxy motors must not keep adaptive lag-force/tau fields after returning to fixed HIGGS-style dynamic grab motors.'
Reject-Text 'src/RockConfig.h' 'rockGrabAdaptive|rockGrabTauMax|rockGrabMassResponsiveMaxForce' 'Header config must not preserve removed adaptive grab motor settings.'
Reject-Text 'data/config/ROCK.ini' 'fGrabAdaptive|bGrabAdaptive|fGrabTauMax|fGrabMassResponsiveMaxForce' 'Packaged ROCK.ini must not expose removed adaptive grab motor settings.'
Require-Text 'data/config/ROCK.ini' 'fNearCastRadiusGameUnits\s*=\s*3\.5' 'Packaged ROCK.ini must keep close selection to HIGGS-scale sphere radius.'
Require-Text 'data/config/ROCK.ini' 'fNearCastDistanceGameUnits\s*=\s*7\.0' 'Packaged ROCK.ini must keep close selection to HIGGS-scale short reach.'
Require-Text 'src/RockConfig.cpp' '"fNearCastRadiusGameUnits"[\s\S]*kDefaultNearCastRadiusGameUnits[\s\S]*0\.0f[\s\S]*kDefaultNearCastRadiusGameUnits' 'Runtime config loading must clamp existing production near-cast radius values down to the close-grab default.'
Require-Text 'src/RockConfig.cpp' '"fNearCastDistanceGameUnits"[\s\S]*kDefaultNearCastDistanceGameUnits[\s\S]*0\.1f[\s\S]*kDefaultNearCastDistanceGameUnits' 'Runtime config loading must clamp existing production near-cast distance values down to the close-grab default.'
Require-Text 'data/config/ROCK.ini' 'fGrabNearbyDampingRadius\s*=\s*90\.0' 'Packaged ROCK.ini must preserve the broad nearby damping radius that gives clutter a stable grab feel.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNearbyDampingRadius\s*=\s*nearby_grab_damping::sanitizeRadius' 'Runtime config loading must sanitize nearby damping radius without clamping away broad production values.'
Require-Text 'data/config/ROCK.ini' 'bGrabHeldMassMovementSlowdownEnabled\s*=\s*true' 'Packaged ROCK.ini must expose HIGGS-style held-mass movement slowdown.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'targetReduction\s*>\s*0\.0f\s*\|\|\s*previousReduction\s*<=\s*0\.0f' 'Held-mass movement restore must not skip clearing a tiny remaining SpeedMult penalty when the target reduction reaches zero.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'constraintDrivePivotBBodyLocalGame' 'Shared-grab cleanup must remove the retired helper that only mirrored the active transform-B pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activeProxyConstraintPivotBLocalGame' 'Constraint grabs must create and measure against the active transform-B pivot, not the native frozen surface pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'activePivotBBodyLocalGame' 'Held-object error, convergence, and debug logging must use the active constraint pivot for shared grabs.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'updateConstraintGrabDriveMotors' 'Custom held-object updates must drive both angular and linear motors while held.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseContext\.finalObjectRelease' 'Held object restoration must be gated on final shared-object release.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'player\s*&&\s*!joiningPeerHeldObject' 'Joining an already ROCK-held object must not call native VR drop on the peer hand.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'shared-held-far' 'Peer-held objects must stay rejected from far selection instead of becoming shared far-pull targets.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'recordSemanticContact\(handSource->metadata,\s*contactRoute\.targetBodyId,\s*semanticContactPoint,\s*semanticContactNormal\)' 'Semantic hand contact records must preserve raw callback point and normal evidence when available.'
Require-Text 'src/physics-interaction/hand/HandLifecycle.h' 'hasContactPointGame[\s\S]*hasContactNormalGame' 'Semantic contact records must expose optional point and normal payloads.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'evaluateSemanticPivotCandidate' 'Delayed peer-held close selection must prefer fresh hand contact evidence before falling back to body origin.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'semanticContactPoint[\s\S]*semanticHandBodyOriginFallback[\s\S]*peerHeldBodyOriginFallback' 'Peer-held close acquisition must prefer semantic contact points before hand-body and held-body-origin fallbacks.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'semanticContactPointGame[\s\S]*evidencePointWorld[\s\S]*findClosestGrabSurfaceHit\(surfaceTriangles,\s*evidencePointWorld' 'Grab commit multi-finger validation must snap from stored semantic contact points before live body centers.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'const auto transition = applyTransition\(HandTransitionRequest\{ \.event = HandInteractionEvent::SelectionFoundClose \}\)' 'Peer-held fallback selection must commit only after the state machine accepts close selection.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'refreshedPeerHeldSelection' 'A delayed second-hand press must refresh the peer-held close candidate before grab commit.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'clearSelectionState\(false\)' 'Stale peer-held close selections must be cleared when the refreshed reach check fails.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'acquireBodyFlagLease' 'Shared held-object body flags must be leased across hands instead of raw enabled/disabled per release.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'releaseBodyFlagLease' 'Held-object body flags must remain leased while the peer hand still holds the object.'

if ($failures.Count -gt 0) {
    Write-Host 'Shared held-object grab source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Shared held-object grab source test passed.'

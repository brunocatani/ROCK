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
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'authorityForceScale' 'The pure motor controller must scale final mass-capped force by per-hand authority share.'
Reject-Text 'src/physics-interaction/grab/GrabMotionController.h' 'maxForceMultiplier|massResponsiveMaxForce|positionErrorGameUnits|fullPositionErrorGameUnits|maxTau' 'Held proxy motors must not keep adaptive lag-force/tau fields after returning to fixed HIGGS-style dynamic grab motors.'
Reject-Text 'src/RockConfig.h' 'rockGrabAdaptive|rockGrabTauMax|rockGrabMassResponsiveMaxForce' 'Header config must not preserve removed adaptive grab motor settings.'
Require-Text 'src/RockConfig.cpp' '"fNearCastRadiusGameUnits"[\s\S]*kDefaultNearCastRadiusGameUnits[\s\S]*0\.0f[\s\S]*kDefaultNearCastRadiusGameUnits' 'Runtime config loading must clamp existing production near-cast radius values down to the close-grab default.'
Require-Text 'src/RockConfig.cpp' '"fNearCastDistanceGameUnits"[\s\S]*kDefaultNearCastDistanceGameUnits[\s\S]*0\.1f[\s\S]*kDefaultNearCastDistanceGameUnits' 'Runtime config loading must clamp existing production near-cast distance values down to the close-grab default.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNearbyDampingRadius\s*=\s*nearby_grab_damping::sanitizeRadius' 'Runtime config loading must sanitize nearby damping radius without clamping away broad production values.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'targetReduction\s*>\s*0\.0f\s*\|\|\s*previousReduction\s*<=\s*0\.0f' 'Held-mass movement restore must not skip clearing a tiny remaining SpeedMult penalty when the target reduction reaches zero.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'shared-held-far' 'Peer-held objects must stay rejected from far selection instead of becoming shared far-pull targets.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' 'recordSemanticContact\(handSource->metadata,\s*contactRoute\.targetBodyId,\s*semanticContactPoint,\s*semanticContactNormal\)' 'Semantic hand contact records must preserve raw callback point and normal evidence when available.'
Require-Text 'src/physics-interaction/hand/HandLifecycle.h' 'hasContactPointGame[\s\S]*hasContactNormalGame' 'Semantic contact records must expose optional point and normal payloads.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'evaluateSemanticPivotCandidate' 'Delayed peer-held close selection must prefer fresh hand contact evidence before falling back to body origin.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'semanticContactPoint[\s\S]*semanticHandBodyOriginFallback[\s\S]*peerHeldBodyOriginFallback' 'Peer-held close acquisition must prefer semantic contact points before hand-body and held-body-origin fallbacks.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'const auto transition = applyTransition\(HandTransitionRequest\{ \.event = HandInteractionEvent::SelectionFoundClose \}\)' 'Peer-held fallback selection must commit only after the state machine accepts close selection.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'refreshedPeerHeldSelection' 'A delayed second-hand press must refresh the peer-held close candidate before grab commit.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'clearSelectionState\(false\)' 'Stale peer-held close selections must be cleared when the refreshed reach check fails.'

if ($failures.Count -gt 0) {
    Write-Host 'Shared held-object grab source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Shared held-object grab source test passed.'

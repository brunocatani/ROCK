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

Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'allowsSharedHeldReference' 'Far selection must not silently pull objects already held by the peer hand.'
Reject-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'ref\s*==\s*otherHandRef' 'Object detection must not reject the peer-held ref with the old blanket equality guard.'
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
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'shared-held-far' 'Peer-held objects must stay rejected from far selection instead of becoming shared far-pull targets.'
Require-Text 'src/physics-interaction/hand/HandLifecycle.h' 'hasContactPointGame[\s\S]*hasContactNormalGame' 'Semantic contact records must expose optional point and normal payloads.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'evaluateSemanticPivotCandidate' 'Delayed peer-held close selection must prefer fresh hand contact evidence before falling back to body origin.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'semanticContactPoint[\s\S]*semanticHandBodyOriginFallback[\s\S]*peerHeldBodyOriginFallback' 'Peer-held close acquisition must prefer semantic contact points before hand-body and held-body-origin fallbacks.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'const auto transition = applyTransition\(HandTransitionRequest\{ \.event = HandInteractionEvent::SelectionFoundClose \}\)' 'Peer-held fallback selection must commit only after the state machine accepts close selection.'

if ($failures.Count -gt 0) {
    Write-Host 'Shared held-object grab source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Shared held-object grab source test passed.'

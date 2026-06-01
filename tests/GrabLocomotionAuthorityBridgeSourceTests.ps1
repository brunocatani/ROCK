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

Require-Text 'src/physics-interaction/grab/GrabLocomotionAuthorityBridge.h' 'kDefaultMaxLeadSeconds\s*=\s*0\.012f[\s\S]*kDefaultSmoothingHz\s*=\s*45\.0f[\s\S]*kDefaultMaxOffsetGameUnits\s*=\s*4\.0f[\s\S]*kDefaultResetDistanceGameUnits\s*=\s*35\.0f' `
    'Bridge policy must expose the approved default lead, smoothing, cap, and reset distance.'
Require-Text 'src/RockConfig.h' 'rockGrabLocomotionAuthorityBridgeEnabled[\s\S]*rockGrabLocomotionAuthorityMaxLeadSeconds[\s\S]*rockGrabLocomotionAuthoritySmoothingHz[\s\S]*rockGrabLocomotionAuthorityMaxOffsetGameUnits[\s\S]*rockGrabLocomotionAuthorityResetDistanceGameUnits' `
    'RockConfig must expose all locomotion authority bridge settings.'
Require-Text 'src/RockConfig.cpp' 'bGrabLocomotionAuthorityBridgeEnabled[\s\S]*fGrabLocomotionAuthorityMaxLeadSeconds[\s\S]*fGrabLocomotionAuthoritySmoothingHz[\s\S]*fGrabLocomotionAuthorityMaxOffsetGameUnits[\s\S]*fGrabLocomotionAuthorityResetDistanceGameUnits' `
    'RockConfig must read all locomotion authority bridge settings from INI.'
Require-Text 'data/config/ROCK.ini' 'bGrabLocomotionAuthorityBridgeEnabled\s*=\s*true[\s\S]*fGrabLocomotionAuthorityMaxLeadSeconds\s*=\s*0\.012[\s\S]*fGrabLocomotionAuthoritySmoothingHz\s*=\s*45\.0[\s\S]*fGrabLocomotionAuthorityMaxOffsetGameUnits\s*=\s*4\.0[\s\S]*fGrabLocomotionAuthorityResetDistanceGameUnits\s*=\s*35\.0' `
    'Packaged INI must expose the approved locomotion authority bridge defaults.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' 'grab_locomotion_authority_bridge::State\s+_grabLocomotionAuthorityBridge' `
    'PhysicsInteraction must own the bridge state.'
Require-Text 'src/physics-interaction/core/PhysicsFrameContext.h' 'unbridgedRawHandWorld[\s\S]*locomotionAuthorityOffsetGame[\s\S]*locomotionAuthorityBridged' `
    'PhysicsFrameContext must carry raw-vs-bridged hand authority diagnostics.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'updateGrabLocomotionAuthorityBridge\(frame\.deltaSeconds,\s*frame\.worldReady\)[\s\S]*input\.unbridgedRawHandWorld\s*=\s*input\.rawHandWorld[\s\S]*holdingForLocomotionAuthority\s*=\s*hand\.isHoldingAtomic\(\)[\s\S]*input\.locomotionAuthorityOffsetGame\s*=\s*\(locomotionAuthorityBridged && holdingForLocomotionAuthority\)\s*\?\s*locomotionAuthorityOffsetGame[\s\S]*input\.rawHandWorld\.translate\s*=\s*input\.rawHandWorld\.translate\s*\+\s*input\.locomotionAuthorityOffsetGame' `
    'Frame construction must bridge held HandFrameInput.rawHandWorld with the same computed offset while leaving a free hand raw.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'updateHandCollisions[\s\S]*_rightHand\.updateCollisionTransform\(world,\s*frame\.right\.rawHandWorld,\s*frame\.deltaSeconds,\s*frame\.right\.locomotionAuthorityOffsetGame\)[\s\S]*_leftHand\.updateCollisionTransform\(world,\s*frame\.left\.rawHandWorld,\s*frame\.deltaSeconds,\s*frame\.left\.locomotionAuthorityOffsetGame\)' `
    'Generated hand collider updates must receive the same bridge offset as frame authority.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' '_boneColliders\.update\(world,\s*_isLeft,\s*rollAuthorityWorld,\s*_handBody,\s*deltaTime,\s*authorityTranslationOffsetGame\)' `
    'Hand must forward the bridge offset to the bone-collider set.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'captureBoneLookup\([\s\S]*authorityTranslationOffsetGame[\s\S]*applyAuthorityTranslationOffset\(outLookup,\s*authorityTranslationOffsetGame\)' `
    'Hand bone capture must apply the bridge offset before role frames are derived.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'applyAuthorityTranslationOffset[\s\S]*lookup\.hand\.translate[\s\S]*lookup\.rollAuthorityWorld\.translate[\s\S]*lookup\.fingers\[fingerIndex\][\s\S]*lookup\.fingerBases\[fingerIndex\]' `
    'The bridge offset must reach hand, roll-authority, finger, and finger-base lookup transforms.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'LOCOMOTION_AUTH hand=\{\}[\s\S]*rawDelta=\(\{:\.2f\},\{:\.2f\},\{:\.2f\}\)[\s\S]*velocity=\(\{:\.2f\},\{:\.2f\},\{:\.2f\}\)[\s\S]*offset=\(\{:\.2f\},\{:\.2f\},\{:\.2f\}\)[\s\S]*reset=\{\}' `
    'Runtime debug logging must emit the requested LOCOMOTION_AUTH fields.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'PALM_CLOCK[\s\S]*rawToBridged=\{:\.3f\}gu/\{:\.3f\}deg[\s\S]*bridgeOffset=\(\{:\.2f\},\{:\.2f\},\{:\.2f\}\)' `
    'Palm clock diagnostics must include raw-vs-bridged hand authority.'
Require-Text 'CMakeLists.txt' 'ROCKGrabLocomotionAuthorityBridgePolicyTests[\s\S]*GrabLocomotionAuthorityBridgePolicyTests\.cpp[\s\S]*ROCK_COMMONLIB_TEST_TARGETS[\s\S]*ROCKGrabLocomotionAuthorityBridgePolicyTests' `
    'The bridge policy test must be registered in the policy test target.'

if ($failures.Count -gt 0) {
    Write-Host 'GrabLocomotionAuthorityBridgeSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GrabLocomotionAuthorityBridgeSourceTests passed.' -ForegroundColor Green

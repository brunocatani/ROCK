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

Require-Text 'src/rock_support/Fo4VrRuntime.h' `
    '0x6E0 \+ offsetof\(PlayerNodes, primaryWandNode\) == 0x6F0[\s\S]*primaryWeaponOffsetNOde\) == 0x718[\s\S]*SecondaryWandNode\) == 0x768[\s\S]*SecondaryMeleeWeaponOffsetNode2\) == 0x790' `
    'The Ghidra-verified FO4VR wand and weapon-driver slots must remain build-enforced.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' `
    'frame\.gameFrameIndex = runtime_state::currentFrame\(\)\.frameIndex[\s\S]*primaryWandNode[\s\S]*SecondaryWandNode[\s\S]*primaryWeaponOffsetNOde[\s\S]*SecondaryMeleeWeaponOffsetNode2' `
    'The coherent frame snapshot must capture the game generation and all four producer nodes.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' `
    'state\.sourceFrameIndex = sourceFrameIndex;[\s\S]*\+\+state\.queuedSequence[\s\S]*result\.sourceSequence = state\.queuedSequence;[\s\S]*result\.sourceFrameIndex = state\.sourceFrameIndex;' `
    'Source frame identity must travel under the same mutex as the consumed generated-body target.'
Require-Text 'src/physics-interaction/weapon/DynamicWeaponCollision.cpp' `
    'frame\.gameFrameIndex[\s\S]*snapshot\.sourceGameFrameIndex = _physicsSourceGameFrameIndex[\s\S]*snapshot\.sourceQueueSequence = _physicsSourceQueueSequence[\s\S]*snapshot\.physicsSubstepProgress = timing\.substepProgress' `
    'Weapon post-solve telemetry must correlate the consumed game source with Havok timing.'
Require-Text 'src/physics-interaction/weapon/DynamicWeaponCollision.cpp' `
    '_physicsSourceGameFrameIndex = 0;[\s\S]*_physicsSourceQueueSequence = 0;[\s\S]*clearGeneratedKeyframedBodyDriveState\(_authorityDriveState\);' `
    'Weapon proxy retirement must clear both retained and queued source generations.'
Require-Text 'src/physics-interaction/hand/DynamicHandCollision.cpp' `
    'frame\.gameFrameIndex[\s\S]*slot\.droveSourceGameFrameIndex = result\.sourceFrameIndex[\s\S]*sourceGameFrameIndex = owner\.droveSourceGameFrameIndex[\s\S]*solveSequence = solveSequence' `
    'Hand post-solve telemetry must correlate the consumed game source with the solve generation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'kTraceFramesPerEpisode = 360[\s\S]*colliderClockDebugActive[\s\S]*rockDebugShowColliders[\s\S]*rockDebugDrawHandColliders[\s\S]*rockDebugDrawHandBoneColliders[\s\S]*rockDebugDrawDynamicHandColliders[\s\S]*rockDebugDrawWeaponColliders[\s\S]*rockDebugDrawDynamicWeaponColliders[\s\S]*_colliderClockHasLoggedFrame = false[\s\S]*_colliderClockFramesRemaining[\s\S]*grabStarted[\s\S]*COLLIDER_CLOCK begin' `
    'Collider clock logging must cover hand/body and weapon collider overlays while remaining debug-gated and bounded per episode.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'COLLIDER_CLOCK input[\s\S]*COLLIDER_CLOCK frame[\s\S]*COLLIDER_CLOCK weapon[\s\S]*COLLIDER_CLOCK hand[\s\S]*GRAB_LOCOMOTION drive[\s\S]*GRAB_LOCOMOTION visual' `
    'The trace must emit correlated controller, game-frame, collider, held-object, and visual-hand records.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'source\(frame/age/queue/solve\)[\s\S]*physics\(raw/sub/rem/accum/index/count/progress\)' `
    'Weapon trace rows must expose both producer generations and native physics cadence.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'source\(frame/age/queue/solve/seqlock\)[\s\S]*rawToWand/wandToDriver/rawToFrik/requestedToLive' `
    'Hand trace rows must expose producer, publication, and solved-body mismatch deltas.'
Require-Text 'src/rock_support/VRControllers.cpp' `
    'getPollSnapshot[\s\S]*current\.unPacketNum[\s\S]*previous\.unPacketNum[\s\S]*packetChanged[\s\S]*current\.rAxis' `
    'The controller trace must retain OpenVR packet identity and every polled analog axis.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'sourceGameFrameIndex = runtime_state::currentFrame\(\)\.frameIndex[\s\S]*_lastAppliedGrabAuthoritySourceGameFrameIndex = pending\.sourceGameFrameIndex[\s\S]*_grabAuthorityProxyLastFlushTiming = timing[\s\S]*_grabAuthorityProxyLastAfterSolveTiming = timing' `
    'Grab authority telemetry must carry the queued game frame through flush and retain both solve-phase timing samples.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'GRAB_LOCOMOTION drive[\s\S]*source\(frame/age/queue/pending/flush/after\)[\s\S]*steps\(raw/target/proxy/body\)[\s\S]*GRAB_LOCOMOTION visual[\s\S]*gaps\(bodyDerivedToNode/heldHandToFrik/rawToHeldHand\)[\s\S]*steps\(node/heldHand/frik\)' `
    'Grab locomotion rows must localize both cadence stair-steps and the body-to-scene-to-FRIK presentation boundary.'

if ($failures.Count -gt 0) {
    Write-Host 'ColliderClockDiagnosticsSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ColliderClockDiagnosticsSourceTests passed.' -ForegroundColor Green

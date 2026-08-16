param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-File {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (Test-Path -LiteralPath $path) {
        $failures.Add("$RelativePath`: $Message")
    }
}

# The failed stick-locomotion compensation systems were removed, not disabled.
# The grab-authority source-clock resampler is the one clock-boundary fix; none
# of these may return as dormant alternative behavior paths.

Reject-File 'src/physics-interaction/grab/GrabLocomotionAuthorityBridge.h' `
    'The locomotion-authority bridge was removed; do not reintroduce it.'
Reject-File 'src/physics-interaction/grab/HeldPlayerSpaceRegistry.h' `
    'The held player-space central writer was removed; do not reintroduce it.'
Reject-File 'src/physics-interaction/grab/HeldPlayerSpaceRegistry.cpp' `
    'The held player-space central writer was removed; do not reintroduce it.'
Reject-File 'src/physics-interaction/grab/GrabLocomotionJag.h' `
    'Held-object actor/controller anchor translation was removed; do not reintroduce it.'
Reject-File 'src/physics-interaction/grab/GrabFrameDiscontinuityCorrection.h' `
    'The wall-clock versus physics-clock held-body translation was removed; do not reintroduce it.'
Reject-File 'tests/GrabLocomotionJagPolicyTests.cpp' `
    'The deleted locomotion-jag policy must not return as a dormant alternative path.'

$sourceFiles = @(
    'src/RockConfig.h',
    'src/RockConfig.cpp',
    'src/physics-interaction/core/PhysicsInteraction.cpp',
    'src/physics-interaction/core/PhysicsInteraction.h',
    'src/physics-interaction/core/PhysicsInteractionFrame.inl',
    'src/physics-interaction/core/PhysicsHooks.cpp',
    'src/physics-interaction/core/PhysicsHooks.h',
    'src/physics-interaction/hand/Hand.h',
    'src/physics-interaction/hand/HandGrab.cpp',
    'src/physics-interaction/grab/GrabHeldObject.h',
    'src/physics-interaction/grab/GrabAuthoritySourceClockResampler.h',
    'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp',
    'src/physics-interaction/native/CharacterControllerRuntime.h',
    'src/physics-interaction/native/CharacterControllerRuntime.cpp',
    'src/physics-interaction/native/HavokOffsets.h',
    'src/physics-interaction/weapon/EquippedWeaponDropMomentum.h'
)

foreach ($file in $sourceFiles) {
    Reject-Text $file 'GrabPlayerSpaceCompensation|GrabPlayerSpaceWarp|HeldObjectPlayerSpaceFrame|held_player_space' `
        'Held player-space compensation/warp was removed (phase-dead additive velocity; the proxy solve overwrote it).'
    Reject-Text $file 'GrabLocomotionAuthority|grab_locomotion_authority' `
        'The locomotion-authority bridge was removed (behavior-changing parallel path, not a clock correction).'
    Reject-Text $file 'AlignedRoom|alignedRoom' `
        'The aligned-room ApplyMovementDelta correction was removed (compared commanded vs actual trajectories and saturated).'
    Reject-Text $file 'LocomotionStutterProbe|LOCO_STUTTER' `
        'The locomotion stutter probe was removed; the resampler policy tests own that diagnostic purpose.'
    Reject-Text $file 'ApplyMovementDelta' `
        'The ApplyMovementDelta hook was removed with the aligned-room correction and probe.'
    Reject-Text $file 'GrabLocomotionJag|grabJag|jagCorrection|jagActor|jagController' `
        'Held-object translation from player actor/controller anchor deltas was removed.'
    Reject-Text $file 'GrabRoomVelocityFeedForward|applyRoomVelocityFeedForward|roomFeedForward|tryGetPlayerLocomotionVelocityRawGameUnits' `
        'Player-controller velocity must never be added ahead of the queued grab target.'
    Reject-Text $file 'tryGetPlayerRoomAnchorPositionGameUnits' `
        'The player controller room-anchor reader had no owner after compensation removal and must stay absent.'
    Reject-Text $file 'playerVelocityHavok|applyHeldMotionCompensation|HeldMotionCompensationResult' `
        'Release composition and body sampling must not retain dormant player-compensation APIs.'
}

# Packaged INIs must not carry the removed keys back.
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Reject-Text $ini 'GrabPlayerSpace|GrabLocomotionAuthority|GrabAlignedRoom|LocomotionStutterProbe|GrabVelocityDamping|GrabResidualVelocityDamping|GrabRoomVelocityFeedForward|GrabLocomotionJag' `
        'Removed compensation/probe settings must not reappear in packaged INIs.'
}

Reject-Text 'CMakeLists.txt' 'GrabLocomotionJagPolicyTests' `
    'The removed jag policy target must not remain in the normal test graph.'
Reject-Text 'src/physics-interaction/hand/Hand.h' `
    'applyHeldFrameDiscontinuityCorrectionLocked|playerSpaceDeltaGameUnits|playerSpaceDeltaValid|_grabFrameCorrection' `
    'Grab authority must not retain state or APIs for the removed held-body correction.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'GrabFrameDiscontinuityCorrection|grab_frame_discontinuity|applyHeldFrameDiscontinuityCorrectionLocked|HELD FRAME CORRECTION' `
    'Grab authority must not translate live held bodies from mismatched wall and physics clocks.'

if ($failures.Count -gt 0) {
    Write-Host 'LocomotionCompensationRemovalSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'LocomotionCompensationRemovalSourceTests passed.' -ForegroundColor Green

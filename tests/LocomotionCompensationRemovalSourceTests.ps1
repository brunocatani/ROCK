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
    'src/physics-interaction/native/HavokOffsets.h'
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
}

# Packaged INIs must not carry the removed keys back.
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Reject-Text $ini 'GrabPlayerSpace|GrabLocomotionAuthority|GrabAlignedRoom|LocomotionStutterProbe|GrabVelocityDamping|GrabResidualVelocityDamping' `
        'Removed compensation/probe settings must not reappear in packaged INIs.'
}

if ($failures.Count -gt 0) {
    Write-Host 'LocomotionCompensationRemovalSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'LocomotionCompensationRemovalSourceTests passed.' -ForegroundColor Green

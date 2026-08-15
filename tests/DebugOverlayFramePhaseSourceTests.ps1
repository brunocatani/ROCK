param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $failures.Add("$RelativePath`: missing source file")
        return ''
    }
    return Get-Content -Raw -LiteralPath $path
}

function Require-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

$mainPath = 'src/ROCKMain.cpp'
$physicsPath = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$headerPath = 'src/physics-interaction/core/PhysicsInteraction.h'
$mainText = Read-Source $mainPath
$physicsText = Read-Source $physicsPath
$headerText = Read-Source $headerPath

$mainHook = [regex]::Match(
    $mainText,
    'void\s+onGameFrameUpdateHook\([^\)]*\)\s*\{(?<body>[\s\S]*?)(?=\r?\n\s*bool\s+hookMainLoop)')
if (-not $mainHook.Success) {
    $failures.Add("$mainPath`: could not isolate onGameFrameUpdateHook")
} else {
    $body = $mainHook.Groups['body'].Value
    Require-Pattern $body `
        'RockProviderAnimationPhaseV1::Complete[\s\S]*publishDebugOverlayAfterFrameCallbacks\(\)' `
        'Immutable overlay publication must remain after the provider Complete animation phase.'
}

$update = [regex]::Match(
    $physicsText,
    'void\s+PhysicsInteraction::update\(\)\s*\{(?<body>[\s\S]*?)\n\s*\}\n\n\s*void\s+PhysicsInteraction::publishDebugOverlayAfterFrameCallbacks')
if (-not $update.Success) {
    $failures.Add("$physicsPath`: could not isolate PhysicsInteraction::update")
} else {
    $body = $update.Groups['body'].Value
    Require-Pattern $body `
        '_pendingDebugOverlayFrame\s*=\s*\{\}' `
        'Every physics update must invalidate any unconsumed overlay frame before an early return.'
    Require-Pattern $body `
        'dispatchFrameCallbacks\(\*this\)[\s\S]*registerForNextStep\(bhk,\s*hknp\)[\s\S]*_pendingDebugOverlayFrame\s*=\s*PendingDebugOverlayFrame' `
        'A pending overlay frame may become valid only after callbacks and final collider ownership publication.'
    Reject-Pattern $body `
        'publishDebugBodyOverlay\(frame\)' `
        'PhysicsInteraction::update must not freeze immutable overlay geometry before outer frame callbacks finish.'
}

$latePublish = [regex]::Match(
    $physicsText,
    'void\s+PhysicsInteraction::publishDebugOverlayAfterFrameCallbacks\(\)\s*\{(?<body>[\s\S]*?)(?=\r?\n\s*void\s+PhysicsInteraction::updateAuthoredPrimaryFiringGrip)')
if (-not $latePublish.Success) {
    $failures.Add("$physicsPath`: could not isolate the late overlay publication method")
} else {
    $body = $latePublish.Groups['body'].Value
    Require-Pattern $body `
        'const\s+auto\s+frame\s*=\s*_pendingDebugOverlayFrame\.context\s*;[\s\S]*_pendingDebugOverlayFrame\s*=\s*\{\}' `
        'Late overlay publication must consume the pending frame exactly once before validation.'
    Require-Pattern $body `
        'frame\.gameFrameIndex\s*!=\s*runtime_state::currentFrame\(\)\.frameIndex' `
        'Late publication must reject a frame that crossed the game-frame boundary.'
    Require-Pattern $body `
        'currentBhk\s*!=\s*frame\.bhkWorld\s*\|\|\s*currentHknp\s*!=\s*frame\.hknpWorld' `
        'Late publication must reject a changed Havok world before resolving live body state.'
    Require-Pattern $body `
        'publishDebugBodyOverlay\(frame\)' `
        'The validated late phase must perform the immutable overlay publication.'
}

Require-Pattern $headerText `
    'struct\s+PendingDebugOverlayFrame[\s\S]*PhysicsFrameContext\s+context[\s\S]*bool\s+valid' `
    'PhysicsInteraction must own an explicit same-frame pending overlay handoff.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayFramePhaseSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayFramePhaseSourceTests passed.' -ForegroundColor Green

param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Timing-debt audit for the ROCK timing-normalization project.
#
# Rule 1 freezes the register of behavior-bearing nominal-rate (1/90) fallback
# sites. A NEW fallback anywhere in src/ fails this test. Removing a fallback
# without shrinking the register also fails, so the register stays a truthful
# ledger of the remaining debt while the migration phases burn it down.
#
# Rule 2 freezes the register of frame-count configuration keys. New gameplay
# duration settings must be seconds-based.
#
# Rule 3 freezes wall-clock ownership. steady_clock/tick reads are only allowed
# in the files that own an explicit wall-clock contract (I/O debounce, resource
# watchdogs, diagnostics, lifecycle leases, and the game-frame sampler itself).
#
# Rule 4 keeps the timing core honest: code under src/physics-interaction/timing/
# must pass timing snapshots, never unqualified float deltas.
#
# Allowed 90 values are classified by symbol, not by raw number. Angle limits,
# game-unit radii, debug coordinates, log cadences, and the named grab-force
# calibration reference are not clocks and are not tracked here.

$failures = [System.Collections.Generic.List[string]]::new()

$srcRoot = Join-Path $Root 'src'
if (-not (Test-Path -LiteralPath $srcRoot)) {
    Write-Host 'TimingDebtSourceTests failed: src root missing.' -ForegroundColor Red
    exit 1
}

# ---------------------------------------------------------------------------
# Rule 1: nominal-rate fallback debt register (pattern: 1/90 delta fabrication)
# ---------------------------------------------------------------------------

$nominalRatePattern = '1\s*\.\s*0?f?\s*/\s*90(\s*\.\s*0*f?)?'

$nominalRateDebtRegister = @{
    'src/physics-interaction/core/RockRuntimeStatePolicy.h'          = 1
    'src/physics-interaction/input/DebugControllerRuntime.cpp'       = 1
    'src/physics-interaction/hand/DynamicHandCollisionFeedbackPolicy.h' = 1
    'src/physics-interaction/hand/SurfaceFingerCollisionPolicy.h'    = 1
    'src/physics-interaction/weapon/TwoHandedGrip.cpp'               = 1
    'src/physics-interaction/grab/GrabFinger.h'                      = 1
    'src/physics-interaction/feedback/FeedbackHaptics.cpp'           = 1
}

$sourceFiles = Get-ChildItem -LiteralPath $srcRoot -Recurse -File |
    Where-Object { $_.Extension -in @('.h', '.cpp', '.inl') }

$observedCounts = @{}
foreach ($file in $sourceFiles) {
    $text = Get-Content -Raw -LiteralPath $file.FullName
    $fileMatches = [regex]::Matches($text, $nominalRatePattern)
    if ($fileMatches.Count -gt 0) {
        $relative = $file.FullName.Substring($Root.Length).TrimStart('\', '/').Replace('\', '/')
        $observedCounts[$relative] = $fileMatches.Count
    }
}

foreach ($relative in $observedCounts.Keys) {
    if (-not $nominalRateDebtRegister.ContainsKey($relative)) {
        $failures.Add("$relative`: new nominal-rate (1/90) fallback introduced; behavior code must consume a validated timing snapshot instead.")
        continue
    }
    if ($observedCounts[$relative] -gt $nominalRateDebtRegister[$relative]) {
        $failures.Add("$relative`: nominal-rate (1/90) fallback count grew from $($nominalRateDebtRegister[$relative]) to $($observedCounts[$relative]); new debt is forbidden.")
    } elseif ($observedCounts[$relative] -lt $nominalRateDebtRegister[$relative]) {
        $failures.Add("$relative`: nominal-rate fallback count shrank from $($nominalRateDebtRegister[$relative]) to $($observedCounts[$relative]); update the debt register in tests/TimingDebtSourceTests.ps1 in the same change.")
    }
}
foreach ($relative in $nominalRateDebtRegister.Keys) {
    if (-not $observedCounts.ContainsKey($relative)) {
        $failures.Add("$relative`: registered nominal-rate fallback no longer present; remove its entry from the debt register in tests/TimingDebtSourceTests.ps1.")
    }
}

# ---------------------------------------------------------------------------
# Rule 2: frame-count configuration key register
# ---------------------------------------------------------------------------

$allowedGameplayFrameKeys = @(
    'iWeaponCollisionVisualStabilizationFrames',
    'iGrabConvergeStableFrames',
    'iGrabOppositionContactMaxAgeFrames',
    'iShoulderStashRecentContactFrames',
    'iShoulderStashSustainedContactMissFrames'
)
$allowedDiagnosticFrameKeys = @(
    'iPerformanceProfilerLogIntervalFrames',
    'iPerformanceProfilerWarmupFrames',
    'iDebugWeaponAnimNodeDumpIntervalFrames',
    'iDebugGrabTimelineTraceIntervalFrames',
    'iDebugGrabTransformTelemetryLogIntervalFrames',
    'iDebugWeaponOmodCoverageAuditIntervalFrames',
    'iDebugWorldObjectOriginLogIntervalFrames',
    'iDebugSkeletonBoneLogIntervalFrames'
)
$allowedFrameKeys = $allowedGameplayFrameKeys + $allowedDiagnosticFrameKeys

$rockConfigPath = Join-Path $Root 'src/RockConfig.cpp'
if (Test-Path -LiteralPath $rockConfigPath) {
    $configText = Get-Content -Raw -LiteralPath $rockConfigPath
    $frameKeyMatches = [regex]::Matches($configText, '"(i[A-Za-z0-9]*Frames)"')
    $seenFrameKeys = [System.Collections.Generic.HashSet[string]]::new()
    foreach ($match in $frameKeyMatches) {
        [void]$seenFrameKeys.Add($match.Groups[1].Value)
    }
    foreach ($key in $seenFrameKeys) {
        if ($allowedFrameKeys -notcontains $key) {
            $failures.Add("src/RockConfig.cpp: new frame-count configuration key '$key'; gameplay durations must be seconds-based settings.")
        }
    }
    foreach ($key in $allowedFrameKeys) {
        if (-not $seenFrameKeys.Contains($key)) {
            $failures.Add("src/RockConfig.cpp: registered frame-count key '$key' no longer parsed; remove it from the register in tests/TimingDebtSourceTests.ps1.")
        }
    }
} else {
    $failures.Add('src/RockConfig.cpp: missing file for frame-key audit')
}

# ---------------------------------------------------------------------------
# Rule 3: wall-clock ownership register
# ---------------------------------------------------------------------------

$wallClockPattern = 'steady_clock|GetTickCount64|QueryPerformanceCounter|system_clock'
$allowedWallClockFiles = @(
    'src/physics-interaction/timing/RockGameTiming.cpp',
    'src/physics-interaction/debug/DebugBodyOverlay.cpp',
    'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl',
    'src/physics-interaction/input/InputRemapRuntime.cpp',
    'src/physics-interaction/performance/PerformanceProfiler.cpp',
    'src/physics-interaction/weapon/WeaponTransitionAnimationAcceleration.cpp',
    'src/physics-interaction/weapon/NativeIdleGripPreharvest.cpp',
    'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.cpp',
    'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.h',
    'src/physics-interaction/weapon/EquipVisualBridge.cpp',
    'src/physics-interaction/weapon/EquipVisualBridge.h',
    'src/rock_support/Logger.h',
    'src/rock_support/VRControllers.cpp'
)

foreach ($file in $sourceFiles) {
    $relative = $file.FullName.Substring($Root.Length).TrimStart('\', '/').Replace('\', '/')
    $text = Get-Content -Raw -LiteralPath $file.FullName
    if ($text -match $wallClockPattern -and $allowedWallClockFiles -notcontains $relative) {
        $failures.Add("$relative`: wall-clock read outside the ownership register; gameplay behavior must consume game or physics timing snapshots.")
    }
}

# ---------------------------------------------------------------------------
# Rule 4: timing-core parameter discipline
# ---------------------------------------------------------------------------

$timingCoreRoot = Join-Path $srcRoot 'physics-interaction/timing'
if (Test-Path -LiteralPath $timingCoreRoot) {
    $timingFiles = Get-ChildItem -LiteralPath $timingCoreRoot -Recurse -File |
        Where-Object { $_.Extension -in @('.h', '.cpp', '.inl') }
    foreach ($file in $timingFiles) {
        $text = Get-Content -Raw -LiteralPath $file.FullName
        $relative = $file.FullName.Substring($Root.Length).TrimStart('\', '/').Replace('\', '/')
        if ($text -match '\(\s*float\s+delta(Time|Seconds)\b' -or $text -match ',\s*float\s+delta(Time|Seconds)\b') {
            $failures.Add("$relative`: timing core must pass timing snapshots, not unqualified float delta parameters.")
        }
        if ($text -match $nominalRatePattern) {
            $failures.Add("$relative`: timing core must never fabricate a nominal 1/90 delta.")
        }
    }
}

# ---------------------------------------------------------------------------
# Allowed-by-symbol 90 values must keep their names
# ---------------------------------------------------------------------------

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing file for text check")
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/RockConfig.h' 'rockGrabPhysicsRateReferenceHz' `
    'The grab-force calibration reference must stay a named symbol, not a raw 90.'
Require-Text 'src/physics-interaction/grab/GrabMotionController.h' 'physicsRateReferenceHz' `
    'Grab motor force scaling must consume the named calibration reference.'

if ($failures.Count -gt 0) {
    Write-Host 'TimingDebtSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'TimingDebtSourceTests passed.' -ForegroundColor Green

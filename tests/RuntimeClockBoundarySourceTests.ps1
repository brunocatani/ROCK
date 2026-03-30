param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$Root = (Resolve-Path -LiteralPath $Root).Path

function Get-RepoText {
    param([string]$Path)

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return $null
    }
    return Get-Content -Raw -LiteralPath $fullPath
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-RepoText $Path
    if ($null -eq $text -or $text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-RepoText $Path
    if ($null -ne $text -and $text -match $Pattern) {
        $failures.Add($Message)
    }
}

$runtimeFiles = Get-ChildItem -LiteralPath (Join-Path $Root 'src') -Recurse -File |
    Where-Object {
        $_.Extension -in @('.cpp', '.h', '.hpp', '.inl') -and
        $_.FullName -ne (Join-Path $Root 'src\RockConfig.cpp') -and
        $_.FullName -ne (Join-Path $Root 'src\physics-interaction\performance\PerformanceProfiler.cpp')
    }

$bannedRuntimeClockPatterns = @(
    'GetTickCount64',
    'GetTickCount\s*\(',
    'QueryPerformanceCounter',
    'timeGetTime',
    'std::chrono::',
    'std::this_thread::sleep_for',
    'std::this_thread::sleep_until',
    '\bSleep\s*\(',
    '\bSetTimer\s*\(',
    '\bKillTimer\s*\('
)

foreach ($file in $runtimeFiles) {
    $relativePath = $file.FullName.Substring($Root.Length).TrimStart('\', '/')
    $text = Get-Content -Raw -LiteralPath $file.FullName
    foreach ($pattern in $bannedRuntimeClockPatterns) {
        if ($text -match $pattern) {
            $failures.Add("$relativePath`: runtime gameplay/physics code must not use wall-clock or thread-sleep API '$pattern'. Use frame, game, or Havok physics timing instead.")
        }
    }
}

Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'g_nativeMeleeFrameClock' 'Native melee physical-swing bridge must use the ROCK frame clock, not wall-clock milliseconds.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kNativeMeleePhysicalSwingLeaseFrames' 'Native melee physical-swing bridge must express its lease in update frames.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'advanceNativeMeleeFrameClock' 'Native melee frame clock must be advanced explicitly from the ROCK update loop.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'advanceNativeMeleeFrameClock\(\)' 'PhysicsInteraction::update must advance the native melee frame clock.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'clearNativeMeleePhysicalSwingLeases\(\)' 'PhysicsInteraction::shutdown must clear native melee frame leases.'

Reject-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'GetTickCount64|kNativeMeleePhysicalSwingLeaseMs|ExpiresAtMs' 'Native melee suppression must not use wall-clock milliseconds.'
Reject-Text 'src/physics-interaction/grab/GrabTelemetry.h' 'tickMs' 'Grab telemetry stamps must remain frame-based, not wall-clock based.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'GetTickCount64|tickMs' 'Grab telemetry overlay must not sample wall-clock time.'

Require-Text 'src/RockConfig.cpp' 'std::this_thread::sleep_for' 'The only approved runtime wall-clock wait is the ROCK.ini file-watch debounce in RockConfig.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'QueryPerformanceCounter' 'The diagnostics profiler owns the only approved high-resolution runtime timer boundary.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'if \(!s_settings\.enabled\.load\([\s\S]*?return;[\s\S]*?_startTicks = queryPerformanceTicks\(\);' 'Profiler scoped timers must return before sampling QueryPerformanceCounter when the profiler is disabled.'

if ($failures.Count -gt 0) {
    Write-Error ($failures -join [Environment]::NewLine)
}

Write-Host 'Runtime clock boundary passed.'

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

Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabHeldObjectUpdate' `
    'Profiler must expose game-frame held-object update timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabAuthorityFlush' `
    'Profiler must expose physics-step grab authority flush timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' 'GrabAuthorityAfterSolveDiagnostics' `
    'Profiler must expose gated after-solve diagnostic timing when diagnostics are enabled.'

Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'grabHeldObjectUpdate' `
    'Profiler output must name held-object update timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'grabAuthorityFlush' `
    'Profiler output must name grab authority flush timing.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.cpp' 'grabAuthorityAfterSolveDiagnostics' `
    'Profiler output must name after-solve diagnostic timing.'

Require-Text 'src/RockConfig.h' 'rockDebugGrabAfterSolveAnomalySampling' `
    'After-solve anomaly sampling must have a separate explicit debug gate.'
Require-Text 'src/RockConfig.cpp' 'bDebugGrabAfterSolveAnomalySampling' `
    'RockConfig must read the separate after-solve anomaly sampling gate.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::updateHeldObject[\s\S]*Scope::GrabHeldObjectUpdate' `
    'Held-object game-frame work must be measurable separately from total frame time.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::flushPendingCustomGrabAuthority[\s\S]*addEventCount\(performance_profiler::Scope::GrabAuthorityFlush\)[\s\S]*ScopedTimer profilerTimer\(performance_profiler::Scope::GrabAuthorityFlush\)' `
    'Active custom grab authority flushes must count and time physics-step work.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'void Hand::observeCustomGrabAuthorityAfterSolve[\s\S]*const bool debugGrabFrameLogging[\s\S]*const bool timelineTraceLogging[\s\S]*const bool shouldSampleForAnomaly\s*=\s*g_rockConfig\.rockDebugGrabAfterSolveAnomalySampling[\s\S]*if \(!debugGrabFrameLogging && !timelineTraceLogging && !shouldSampleForAnomaly\)\s*\{\s*return;\s*\}[\s\S]*Scope::GrabAuthorityAfterSolveDiagnostics[\s\S]*hasConstraintFrameMetrics' `
    'After-solve body readback and constraint atom diagnostics must stay behind explicit grab debug, timeline, or anomaly-sampling gates.'

Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'const bool shouldSampleForAnomaly\s*=\s*afterSolveSequence\s*<=\s*16u' `
    'After-solve anomaly sampling must not run unconditionally during normal gameplay.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab authority performance source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab authority performance source boundary passed.'

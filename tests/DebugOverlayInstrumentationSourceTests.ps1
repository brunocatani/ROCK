param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$timerHeader = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayGpuTimer.h')
$timerSource = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayGpuTimer.cpp')
$statsHeader = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayStats.h')
$overlay = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugBodyOverlay.cpp')

function Require-In {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-In {
    param([string]$Text, [string]$Pattern, [string]$Message)
    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-In $timerHeader 'kSlotCount\s*=\s*4' `
    'GPU timing must use the fixed four-slot query ring.'
Require-In $timerSource 'D3D11_QUERY_TIMESTAMP_DISJOINT' `
    'Each timing slot must own a timestamp-disjoint query.'
Require-In $timerSource 'D3D11_QUERY_TIMESTAMP' `
    'Each timing slot must own timestamp queries.'
Require-In $timerSource 'Begin\(slot\.disjoint\.Get\(\)\)[\s\S]*End\(slot\.start\.Get\(\)\)' `
    'A sample must begin the disjoint interval before its start timestamp.'
Require-In $timerSource 'End\(_slots\[slot\]\.end\.Get\(\)\)[\s\S]*End\(_slots\[slot\]\.disjoint\.Get\(\)\)' `
    'A sample must close its end timestamp before the disjoint interval.'
Require-In $timerSource 'constexpr UINT flags\s*=\s*D3D11_ASYNC_GETDATA_DONOTFLUSH' `
    'Asynchronous readback must explicitly prohibit implicit command-buffer flushes.'

$getDataCalls = [regex]::Matches($timerSource, '(?s)GetData\([^;]*;')
if ($getDataCalls.Count -ne 3) {
    $failures.Add("GPU timing must keep exactly three bounded GetData probes; found $($getDataCalls.Count).")
} else {
    foreach ($call in $getDataCalls) {
        if ($call.Value -notmatch ',\s*flags\s*\)') {
            $failures.Add('Every GPU timing GetData probe must use D3D11_ASYNC_GETDATA_DONOTFLUSH.')
        }
    }
}

Reject-In $timerSource '->Flush\s*\(' `
    'GPU timing must never flush the immediate context.'
Reject-In $timerSource '(?i)\bwhile\s*\(|\bSleep\s*\(|WaitForSingleObject\s*\(' `
    'GPU timing must never wait or poll in an unbounded loop.'

Require-In $overlay 'DebugOverlayGpuTimer\.h' `
    'The compositor overlay must use the isolated GPU timing module.'
Require-In $overlay 'TimestampQueryRing gpuTimer' `
    'The query ring must share the D3D resource lifetime.'
Require-In $overlay 'if \(resources\.gpuTimer\.initialize\(device\)\)[\s\S]*rendering will continue without GPU timing' `
    'GPU timing initialization failure must leave rendering operational and explain the fallback.'
Reject-In $overlay 'if\s*\(\s*!resources\.gpuTimer\.initialize\(device\)\s*\)[\s\S]{0,100}return false' `
    'Optional GPU timing must not become part of renderer initialization success.'

$readyMethod = [regex]::Match($overlay, '(?s)\[\[nodiscard\]\]\s*bool ready\(\) const noexcept\s*\{.*?\n\s*\}')
if (!$readyMethod.Success) {
    $failures.Add('The D3D resource readiness contract could not be located.')
} elseif ($readyMethod.Value -match 'gpuTimer') {
    $failures.Add('The optional GPU timer must not be required by D3DResources::ready().')
}

Require-In $overlay 'gpuTimerScope\s*=\s*s_d3d\.gpuTimer\.begin\(context\)[\s\S]*drawBodyBatch\([\s\S]*drawLineBatch\([\s\S]*drawTextOverlays\(' `
    'One RAII GPU sample must span all overlay body, line, and text draws.'
Require-In $overlay 'drawTextOverlays\([^;]*;\s*\}\s*\r?\n\s*if \(frame->settings\.verboseLogging' `
    'The GPU timing scope must close before CPU logging begins.'
Require-In $overlay 's_d3d\.gpuTimer\.stats\(\)' `
    'Verbose diagnostics must publish non-blocking GPU timing counters.'
Require-In $overlay 's_frameAdmission\.stats\(\)' `
    'Verbose diagnostics must distinguish frame-admission rejection causes.'

Require-In $statsHeader 'struct RuntimeStats' `
    'Per-frame overlay counters must live in the isolated statistics contract.'
Require-In $overlay 'using OverlayRuntimeStats\s*=\s*debug_overlay_stats::RuntimeStats' `
    'The compositor must consume the isolated statistics contract.'
Reject-In $overlay 'struct OverlayRuntimeStats' `
    'The superseded monolithic runtime-statistics definition must remain removed.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayInstrumentationSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayInstrumentationSourceTests passed.' -ForegroundColor Green

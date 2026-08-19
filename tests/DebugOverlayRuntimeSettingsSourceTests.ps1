param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$configHeader = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/RockConfig.h')
$configSource = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/RockConfig.cpp')
$repoIni = Get-Content -Raw -LiteralPath (Join-Path $Root 'data/config/ROCK.ini')
$modIni = Get-Content -Raw -LiteralPath (Join-Path $Root 'data/mod/ROCK_Config/ROCK.ini')

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

$keys = @(
    'iDebugMaxShapeCapturesPerFrame',
    'iDebugMaxConvexSupportVertices',
    'iDebugMaxCompoundChildren',
    'iDebugMaxCompoundDepth',
    'iDebugMaxShapeQueuedJobs',
    'iDebugMaxShapeCompletedJobs',
    'iDebugMaxShapeUploadsPerFrame',
    'iDebugMaxShapeCacheEntries',
    'iDebugMaxShapeCacheBytes',
    'iDebugMaxBodyInstances',
    'iDebugMaxLineVertices',
    'iDebugMaxTextVertices'
)

foreach ($key in $keys) {
    Require-In $configSource ([regex]::Escape($key)) "RockConfig does not load $key."
    Require-In $repoIni "(?m)^$([regex]::Escape($key))\s*=" "Repository config is missing $key."
    Require-In $modIni "(?m)^$([regex]::Escape($key))\s*=" "Packaged config is missing $key."
}

Require-In $configHeader 'DebugOverlayRuntimeSettings\.h' `
    'RockConfig defaults must come from the shared overlay settings contract.'
Require-In $configSource 'RequestedLimits[\s\S]*sanitize\(requestedOverlayLimits\)' `
    'INI values must be sanitized through the shared bounded settings contract.'

$all = $configHeader + $configSource + $repoIni + $modIni
Reject-In $all 'iDebugMaxShapeGenerationsPerFrame|rockDebugMaxShapeGenerationsPerFrame|makeOverlaySettingsKey|clampShapeGenerationsPerFrame' `
    'Stale generation naming/settings helpers must not survive the capture-budget migration.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayRuntimeSettingsSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayRuntimeSettingsSourceTests passed.' -ForegroundColor Green

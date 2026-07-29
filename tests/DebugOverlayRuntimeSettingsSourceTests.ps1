param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$configHeader = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/RockConfig.h')
$configSource = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/RockConfig.cpp')
$overlay = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugBodyOverlay.cpp')
$policy = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayPolicy.h')
$runtime = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayRuntimeSettings.h')
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
Require-In $overlay 'captureOverlayRenderSettings[\s\S]*RequestedLimits[\s\S]*debug_overlay_runtime::sanitize' `
    'Published immutable frames must carry a sanitized overlay limit snapshot.'
Require-In $overlay 'maxShapeCapturesPerFrame[\s\S]*shapePipeline\(\)\.reserve' `
    'Publisher-side shape capture must enforce the configured per-frame cap before reservation/capture.'
Require-In $overlay 'processCompletedUploads\([\s\S]*maxShapeUploadsPerFrame' `
    'Compositor uploads must use the configured upload cap.'
Require-In $overlay 'draws\.size\(\)\s*>=\s*frame\.settings\.limits\.maxBodyInstances' `
    'Body instance emission must use the configured fixed-stream cap.'
Require-In $overlay 'beginFrame\(frame->settings\.limits\.maxLineVertices\)' `
    'Line collection must use the configured per-frame vertex cap.'
Require-In $overlay 'maxVertices\s*=\s*frame\.settings\.limits\.maxTextVertices' `
    'Text generation must use the configured per-frame vertex cap.'
Require-In $runtime 'maxShapeCompletedJobs\s*>\s*limits\.maxShapeQueuedJobs[\s\S]*maxShapeCompletedJobs\s*=\s*limits\.maxShapeQueuedJobs' `
    'Completed CPU meshes must remain part of the total bounded shape backlog.'

$all = $configHeader + $configSource + $overlay + $policy + $runtime + $repoIni + $modIni
Reject-In $all 'iDebugMaxShapeGenerationsPerFrame|rockDebugMaxShapeGenerationsPerFrame|makeOverlaySettingsKey|clampShapeGenerationsPerFrame' `
    'Stale generation naming/settings helpers must not survive the capture-budget migration.'
Reject-In $overlay 'kTextVertexCapacity|lineVertexBudget\(' `
    'Renderer stream limits must not bypass the published runtime settings snapshot.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayRuntimeSettingsSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayRuntimeSettingsSourceTests passed.' -ForegroundColor Green

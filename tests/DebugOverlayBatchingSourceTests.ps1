param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$overlay = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugBodyOverlay.cpp')
$shaders = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayShaders.h')
$lines = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/debug/DebugOverlayLineBatch.h')

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

Require-In $overlay 'static_assert\(sizeof\(BodyInstanceData\)\s*==\s*80\)' `
    'Body instances must retain the explicit 80-byte matrix/color layout.'
Require-In $overlay 'D3D11_INPUT_PER_INSTANCE_DATA,\s*2' `
    'The body input layout must advance one object record for each stereo pair.'
Require-In $overlay 'Map\(s_d3d\.bodyInstanceVB[\s\S]*DrawIndexedInstanced\(shape->indexCount,\s*runLength\s*\*\s*2' `
    'Bodies must use one mapped instance stream and stereo-paired indexed runs.'
Require-In $overlay 'draws\[runEnd\]\.shape\s*==\s*draws\[runStart\]\.shape' `
    'Body batching must merge adjacent equal meshes without globally reordering transparent diagnostics.'
Require-In $overlay 'offsets\[2\][\s\S]*runStart\s*\*\s*sizeof\(BodyInstanceData\)' `
    'Each adjacent run must bind its exact contiguous instance subrange.'
Require-In $overlay 'kCanonicalSphereGeometryFingerprint[\s\S]*detailUniformScale\s*=\s*convexRadius\s*\*\s*havokToGameScale' `
    'Direct spheres must share canonical geometry while carrying exact radius in their model scale.'
Require-In $overlay 'XMMatrixScaling\([\s\S]*entry\.detailUniformScale[\s\S]*\*\s*model' `
    'Canonical sphere scale must be composed before the captured body transform.'
Reject-In $overlay 'uploadColorModel|uploadModel|modelCB|PerObjectVSData' `
    'Per-object/per-color constant-buffer maps must remain removed.'

Require-In $shaders 'const uint eyeIndex\s*=\s*input\.instanceId\s*&\s*1' `
    'Instanced shaders must derive the eye from each stereo instance pair.'
Require-In $shaders 'float4 modelRow0\s*:\s*IROW0[\s\S]*float4 color\s*:\s*ICOLOR0' `
    'The body shader must consume matrix and color from the instance stream.'
Require-In $shaders 'kStereoColorVertex[\s\S]*float4 color\s*:\s*COLOR0' `
    'The world-line shader must consume per-vertex color.'
Require-In $shaders 'kScreenTextVertex[\s\S]*float4 color\s*:\s*COLOR0' `
    'The screen-text shader must consume per-vertex color.'

Require-In $overlay 'auto& lineBatch\s*=\s*s_d3d\.scratch->lines[\s\S]*lineBatch\.beginFrame\(frame->settings\.limits\.maxLineVertices\)' `
    'Line collection must reuse prepared process-lifetime scratch with the published frame budget.'
Require-In $overlay 'ColoredVertex start[\s\S]*DrawInstanced\(static_cast<UINT>\(batch\.vertexCount\(\)\),\s*2' `
    'All ordered colored lines must upload and draw as one stereo batch.'
Reject-In $overlay 'lineColorLess|std::sort\(ordered|LineDrawRun' `
    'Line batching must not reorder diagnostics or create per-color draw runs.'
Require-In $lines 'std::vector<KeySlot>\s+_slots[\s\S]*generation' `
    'Line deduplication must use reusable generation-stamped flat scratch.'
Reject-In $lines 'unordered_set' `
    'Per-line node allocation must not return to the render path.'

Require-In $overlay 'auto& vertices\s*=\s*s_d3d\.scratch->textVertices[\s\S]*vertices\.clear\(\)' `
    'Text geometry must reuse one aggregate prepared vector.'
Require-In $overlay 'textRejectedVertices\s*\+=\s*rejectedVertices' `
    'Text overflow telemetry must count actually rejected vertices.'
Require-In $overlay 'memcpy\(mapped\.pData,\s*vertices\.data\(\),\s*vertices\.size\(\)\s*\*\s*sizeof\(ColoredVertex\)\)[\s\S]*context->Draw\(' `
    'Aggregate colored text must use one upload followed by one draw.'
Reject-In $overlay 'std::vector<Vertex>\s+vertices|vertices\.reserve\(4096\)' `
    'Per-entry text allocations must remain removed.'

$bodyMaps = [regex]::Matches($overlay, 'Map\(s_d3d\.bodyInstanceVB').Count
$lineMaps = [regex]::Matches($overlay, 'Map\(s_d3d\.axisLineVB').Count
$textMaps = [regex]::Matches($overlay, 'Map\(s_d3d\.textVB').Count
if ($bodyMaps -ne 1 -or $lineMaps -ne 1 -or $textMaps -ne 1) {
    $failures.Add("Expected one source upload site per dynamic stream; found body=$bodyMaps line=$lineMaps text=$textMaps.")
}

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayBatchingSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayBatchingSourceTests passed.' -ForegroundColor Green

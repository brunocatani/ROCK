param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$relativePath = 'src/physics-interaction/debug/DebugBodyOverlay.cpp'
$path = Join-Path $Root $relativePath
$text = Get-Content -Raw -LiteralPath $path

function Require-Pattern {
    param([string]$Pattern, [string]$Message)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$Pattern, [string]$Message)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Pattern 'DebugOverlaySnapshotPool\.h' `
    'Overlay publication must use the bounded, tested immutable-snapshot pool.'
Require-Pattern 'std::atomic<std::shared_ptr<const PublishedOverlayFrame>>\s+s_publishedFrame' `
    'The compositor boundary must publish an immutable atomic shared_ptr.'
Require-Pattern 's_publishedFrame\.load\(std::memory_order_acquire\)' `
    'The compositor must acquire one immutable frame snapshot.'
Require-Pattern 's_publishedFrame\.store\([\s\S]*std::memory_order_release\)' `
    'Frame publication must use release ordering.'
Require-Pattern 'buildPublishedFrame[\s\S]*extractBody\(source\.world' `
    'Live Havok body state must be resolved while building the game-thread publication.'
Require-Pattern 'buildPublishedFrame[\s\S]*captureBodyWorldAabb\(source\.world' `
    'The publication must capture the real body AABB before crossing into the compositor thread.'
Require-Pattern 'world->GetBodyAabb\(bodyId,\s*&raw\)' `
    'Body bounds must use the current CommonLibF4VR world wrapper instead of imported raw standalone offsets.'
Require-Pattern 'captureOverlayRenderSettings[\s\S]*g_rockConfig' `
    'Mutable overlay settings must be captured into the publication.'
Require-Pattern 'std::shared_ptr<const GpuShape>\s+shapeOwner' `
    'A render lookup must retain immutable GPU buffers across concurrent cache invalidation.'

Reject-Pattern 's_frameMutex' `
    'The compositor must not contend on the retired full-frame mutex.'
Reject-Pattern 'frame\s*=\s*s_frame' `
    'The compositor must not copy the full logical frame.'
Reject-Pattern 'kBodyAabb16|kAabbDecompressOffset|kAabbDecompressScale' `
    'ROCK must not import the standalone visualizer raw AABB offsets.'
Reject-Pattern 'reinterpret_cast<const\s+std::int16_t\s*\*>' `
    'Compressed hknp body AABB components are unsigned; the signed standalone decoder is forbidden.'

$drawMatch = [regex]::Match(
    $text,
    'void\s+drawOverlayToSubmittedTexture\([^\)]*\)\s*\{(?<body>[\s\S]*?)\n\s*void\s+reportOverlayExceptionOnce')
if (-not $drawMatch.Success) {
    $failures.Add('Could not isolate the compositor draw function for boundary checks.')
} else {
    $drawBody = $drawMatch.Groups['body'].Value
    foreach ($forbidden in @('g_rockConfig', 'extractBody(', 'makeShapeKey(', 'generateShape(', 'computeShapeGeometryFingerprint(')) {
        if ($drawBody.Contains($forbidden)) {
            $failures.Add("Compositor draw must not access '$forbidden'.")
        }
    }
}

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlaySnapshotSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlaySnapshotSourceTests passed.' -ForegroundColor Green

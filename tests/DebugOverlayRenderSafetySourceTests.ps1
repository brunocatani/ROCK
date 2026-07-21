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

Require-Pattern 'DebugOverlayFrameAdmission\.h' `
    'The renderer must use the tested publication-serial/reentrancy admission policy.'
Require-Pattern 's_frameAdmission\.tryAcquire\(\)' `
    'The Submit hook must acquire one RAII frame lease before drawing.'
Require-Pattern 'VRSubmitHook\([^\)]*[\s\S]*?\)\s*noexcept' `
    'The OpenVR Submit hook must be noexcept.'
Require-Pattern 'GSGetShader\(' 'The render pass must preserve the geometry shader.'
Require-Pattern 'HSGetShader\(' 'The render pass must preserve the hull shader.'
Require-Pattern 'DSGetShader\(' 'The render pass must preserve the domain shader.'
Require-Pattern 'GSSetShader\(nullptr' 'The overlay must disable the prior geometry shader while drawing.'
Require-Pattern 'HSSetShader\(nullptr' 'The overlay must disable the prior hull shader while drawing.'
Require-Pattern 'DSSetShader\(nullptr' 'The overlay must disable the prior domain shader while drawing.'
Require-Pattern 'IAGetVertexBuffers\(0,\s*2' 'The render pass must preserve vertex-buffer slots zero and one.'
Require-Pattern '~RenderPassGuard\(\)' 'D3D state restoration must be owned by an RAII render-pass guard.'
Require-Pattern 'bool uploadCamera\(' 'Camera upload must report failure instead of binding stale data.'
Require-Pattern 'BodyInstanceData' 'Body model/color data must use the bounded instanced vertex stream.'
Require-Pattern 'bodyInstanceUploadFailureReported' 'Instance-buffer upload failure must fail closed and report once.'

Reject-Pattern 'MainRenderCandidate' 'The no-op hardcoded main-render trampoline must remain removed.'
Reject-Pattern '0xD844BC' 'The retired hardcoded main-render callsite must remain removed.'
Reject-Pattern 'void beginFrame\(' 'Manual begin/end frame ownership must not replace the RAII render pass.'
Reject-Pattern 'void endFrame\(' 'Manual begin/end frame ownership must not replace the RAII render pass.'
Reject-Pattern 'uploadColorModel|uploadModel|modelCB' 'Per-draw model constant-buffer updates must remain removed.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayRenderSafetySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayRenderSafetySourceTests passed.' -ForegroundColor Green

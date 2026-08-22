param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# Game-frame timing coherence contract (timing-normalization Phase 2):
# one timing identity is created before any animation phase and shared by
# BeforeRock, ROCK update, AfterRock, Complete, and provider publication.

$failures = [System.Collections.Generic.List[string]]::new()

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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

# The hook begins frame timing after the original game call and before the
# BeforeRock dispatch, and every phase dispatch shares that snapshot.
Require-Text 'src/ROCKMain.cpp' 's_originalGameLoopFunc\(rcx\);[\s\S]*beginFrameTiming\([\s\S]*BeforeRock' `
    'Frame timing must begin after the original game call and before BeforeRock.'
Require-Text 'src/ROCKMain.cpp' 'BeforeRock,\s*frameTiming\)' `
    'BeforeRock must receive the shared frame timing snapshot.'
Require-Text 'src/ROCKMain.cpp' 'AfterRock,\s*frameTiming\)' `
    'AfterRock must receive the shared frame timing snapshot.'
Require-Text 'src/ROCKMain.cpp' 'Complete,\s*frameTiming\)' `
    'Complete must receive the shared frame timing snapshot.'
Reject-Text 'src/ROCKMain.cpp' 'dispatchAnimationPhaseCallbacksV1\([^)]*currentFrame\(\)\.deltaSeconds' `
    'Phase dispatch must not read the runtime snapshot delta directly.'

# Menu state is sampled once per frame, in beginFrameTiming; updateFrame
# consumes that sample instead of resampling.
Require-Text 'src/physics-interaction/core/RockRuntimeState.cpp' 'beginFrameTiming\(const bool menuInputBlocking\)[\s\S]*beginGameFrame\(' `
    'beginFrameTiming must own the single per-frame clock and menu sample.'

# Provider publication is truthful: validity flags are conditional and the
# published delta is never resanitized or fabricated.
Require-Text 'src/physics-interaction/core/PhysicsInteractionProvider.inl' 'if \(runtime\.timing\.valid\) \{[\s\S]*DeltaSecondsValid' `
    'Frame snapshot DeltaSecondsValid must be set only for a measured delta.'
Require-Text 'src/api/ROCKProviderApi.cpp' 'context\.deltaSeconds = timing\.valid \? timing\.deltaSeconds : 0\.0f;' `
    'Animation context delta must publish the shared snapshot truthfully.'

# No consumer resanitizes the centrally sanitized delta.
Reject-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'deltaSeconds\s*>\s*0\.0f\s*&&\s*deltaSeconds\s*<=\s*0\.1f' `
    'buildFrameContext must not resanitize the central game delta.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' '_deltaTime\s*=\s*1\.0f\s*/\s*90' `
    'PhysicsInteraction must not fabricate a nominal-rate delta.'

if ($failures.Count -gt 0) {
    Write-Host 'GameFrameTimingCoherenceSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GameFrameTimingCoherenceSourceTests passed.' -ForegroundColor Green

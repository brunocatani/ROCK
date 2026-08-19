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

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' '_palmClockGameFrameIndex[\s\S]*_palmClockGameDeltaSeconds' `
    'Palm clock diagnostics must publish the game-frame stamp through atomic state for physics callbacks.'

if ($failures.Count -gt 0) {
    Write-Host 'PalmClockDiagnosticsSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'PalmClockDiagnosticsSourceTests passed.' -ForegroundColor Green

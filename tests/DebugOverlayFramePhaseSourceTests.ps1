param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        $failures.Add("$RelativePath`: missing source file")
        return ''
    }
    return Get-Content -Raw -LiteralPath $path
}

function Require-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

$mainPath = 'src/ROCKMain.cpp'
$headerPath = 'src/physics-interaction/core/PhysicsInteraction.h'
$mainText = Read-Source $mainPath
$headerText = Read-Source $headerPath

$mainHook = [regex]::Match(
    $mainText,
    'void\s+onGameFrameUpdateHook\([^\)]*\)\s*\{(?<body>[\s\S]*?)(?=\r?\n\s*bool\s+hookMainLoop)')



Require-Pattern $headerText `
    'struct\s+PendingDebugOverlayFrame[\s\S]*PhysicsFrameContext\s+context[\s\S]*bool\s+valid' `
    'PhysicsInteraction must own an explicit same-frame pending overlay handoff.'

if ($failures.Count -gt 0) {
    Write-Host 'DebugOverlayFramePhaseSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'DebugOverlayFramePhaseSourceTests passed.' -ForegroundColor Green

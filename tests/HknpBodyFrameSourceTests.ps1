param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/TransformMath.h' 'hknpBodyColumnsToNiStoredAxes' 'TransformMath must expose an explicit FO4VR hknp BODY read conversion.'
Require-Text 'src/physics-interaction/TransformMath.h' 'niStoredAxesToHknpBodyColumns' 'TransformMath must expose an explicit FO4VR hknp BODY write conversion.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host "hknp BODY frame source policy checks passed."

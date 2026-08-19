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

Require-Text 'src/physics-interaction/hand/Hand.h' '_grabDeviationHistory' 'Held-object deviation must keep a short history before release decisions.'
Require-Text 'src/physics-interaction/hand/Hand.h' '_grabVisualDeviationHistory' 'Visual hand deviation must keep a short history before release decisions.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab deviation parity source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab deviation parity source test passed.'

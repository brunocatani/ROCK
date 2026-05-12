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
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'recordDeviationAverage\(\s*_grabDeviationHistory' 'Physical held-object release should use averaged deviation, not a single-frame spike.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'recordDeviationAverage\(\s*_grabVisualDeviationHistory' 'Visual hand release should use averaged hand deviation, matching HIGGS-style smoothing.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'max deviation average' 'Deviation release logs must identify the averaged value used for the decision.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'avgDeviation=' 'Held visual telemetry must expose raw and averaged deviation for runtime diagnosis.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab deviation parity source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab deviation parity source test passed.'

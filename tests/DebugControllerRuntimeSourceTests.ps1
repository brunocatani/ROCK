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

    $fullPath = Join-Path $Root $Path
    $text = Get-Content -Raw -LiteralPath $fullPath
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

    $fullPath = Join-Path $Root $Path
    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'LoadLibraryA' 'Debug controller must load XInput dynamically so ROCK does not gain a mandatory import.'
Require-Text 'src/physics-interaction/input/DebugControllerRuntime.cpp' 'GetProcAddress\(s_xinput\.module,\s*"XInputGetState"\)' 'Debug controller must resolve XInputGetState explicitly.'
Require-Text 'src/ROCKMain.cpp' 'debug_controller_runtime::update\(gameplayInputAllowed' 'Debug controller actions must be gated by ROCK gameplay-input eligibility.'
Require-Text 'src/RockConfig.cpp' 'suppressNextFileWatchReload\(\)' 'Runtime INI writes must use ROCK file-watch self-write suppression.'
Require-Text 'src/RockConfig.cpp' 'persistGrabPivotAHandspace' 'Grab-pivot tuning must persist through the existing ROCK config path.'
Reject-Text 'CMakeLists.txt' 'xinput\.lib' 'ROCK must not link a static XInput import library for the debug controller.'

if ($failures.Count -gt 0) {
    Write-Host 'Debug controller runtime source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Debug controller runtime source boundary passed.'

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

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_NativeVRGrabDrop\s*=\s*0xF1AB90' 'Native VR drop offset must remain explicit at the verified address.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'typedef void func_t\(void\*,\s*int,\s*std::uint64_t\)' 'Native VR drop wrapper must expose the verified third flag argument.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'func\(playerChar,\s*handIndex,\s*0\)' 'Native VR drop wrapper must pass the game-observed third flag value 0.'

Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'typedef void func_t\(void\*,\s*int\)' 'Native VR drop wrapper must not leave R8 uninitialized by using the old two-argument signature.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'func\(playerChar,\s*handIndex\)' 'Native VR drop wrapper must not call the native function without the verified third argument.'

if ($failures.Count -gt 0) {
    Write-Host 'Hand grab native boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Hand grab native boundary passed.'

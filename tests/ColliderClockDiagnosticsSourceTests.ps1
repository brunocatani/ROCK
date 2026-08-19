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

Require-Text 'src/rock_support/Fo4VrRuntime.h' `
    '0x6E0 \+ offsetof\(PlayerNodes, primaryWandNode\) == 0x6F0[\s\S]*primaryWeaponOffsetNOde\) == 0x718[\s\S]*SecondaryWandNode\) == 0x768[\s\S]*SecondaryMeleeWeaponOffsetNode2\) == 0x790' `
    'The Ghidra-verified FO4VR wand and weapon-driver slots must remain build-enforced.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' `
    'state\.sourceFrameIndex = sourceFrameIndex;[\s\S]*\+\+state\.queuedSequence[\s\S]*result\.sourceSequence = state\.queuedSequence;[\s\S]*result\.sourceFrameIndex = state\.sourceFrameIndex;' `
    'Source frame identity must travel under the same mutex as the consumed generated-body target.'

Require-Text 'src/rock_support/VRControllers.cpp' `
    'getPollSnapshot[\s\S]*current\.unPacketNum[\s\S]*previous\.unPacketNum[\s\S]*packetChanged[\s\S]*current\.rAxis' `
    'The controller trace must retain OpenVR packet identity and every polled analog axis.'

if ($failures.Count -gt 0) {
    Write-Host 'ColliderClockDiagnosticsSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ColliderClockDiagnosticsSourceTests passed.' -ForegroundColor Green

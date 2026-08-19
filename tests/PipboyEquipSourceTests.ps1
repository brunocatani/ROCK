param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) { $failures.Add($Message) }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) { $failures.Add($Message) }
}

Require-Text 'src/RockConfig.h' `
    'rockLeftHandedMode\s*=\s*false' `
    'Base ROCK must expose one fixed-hand preference, defaulting to physical right.'
Require-Text 'src/RockConfig.cpp' `
    'WEAPON_HANDEDNESS_SECTION\s*=\s*"WeaponHandedness"[\s\S]*GetBoolValue\(\s*WEAPON_HANDEDNESS_SECTION,\s*"bLeftHandedMode"' `
    'The fixed ROCK-exclusive hand preference must load only from [WeaponHandedness].'

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $configText = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $handednessMatch = [regex]::Match($configText, '(?ms)^\[WeaponHandedness\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (!$handednessMatch.Success -or
        $handednessMatch.Groups['body'].Value -notmatch '(?m)^bLeftHandedMode\s*=\s*false\s*$') {
        $failures.Add("$configPath`: ROCK fixed handedness must be under [WeaponHandedness] and default right.")
    }
    if ($configText -match '(?m)^b(?:MenuTriggerHandEquipEnabled|EquipPreferredHandLeft|PipboyTriggerHandEquipEnabled|PipboyPreferredHandLeft)\s*=') {
        $failures.Add("$configPath`: addon-owned Pip-Boy hand-selection keys must not remain in ROCK.")
    }
}

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    's_pipboyMenuGeneration[\s\S]*rawTransition\.pressedEdges\s*\|\s*rawTransition\.releasedEdges[\s\S]*publishPipboyTriggerTransition\(hand\)[\s\S]*consumePipboyEquipTriggerResolution' `
    'Pip-Boy trigger evidence must be captured before blocking-menu gameplay edges are cleared and consumed per selection.'






if ($failures.Count -gt 0) {
    foreach ($failure in $failures) { Write-Error $failure }
    exit 1
}

Write-Host 'Pip-Boy trigger-hand equip source boundaries passed.'

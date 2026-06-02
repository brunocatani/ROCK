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

Require-Text 'src/physics-interaction/stash/ShoulderStashTransfer.cpp' 'ActivateRef\(' `
    'Shoulder stash transfer must use native reference activation so collectible scripts and perk grants run.'
Reject-Text 'src/physics-interaction/stash/ShoulderStashTransfer.cpp' 'PickUpObject\(' `
    'Shoulder stash transfer must not directly call Actor::PickUpObject for books, notes, magazines, or holotapes.'
Reject-Text 'src/physics-interaction/stash/ShoulderStashTransfer.h' 'skipActivate(Books|Notes)' `
    'Obsolete collectible activation bypass flags must not remain in transfer input.'
Reject-Text 'src/RockConfig.h' 'rockShoulderStashSkipActivate(Books|Notes)' `
    'Obsolete shoulder-stash collectible activation bypass config must not remain in runtime config.'
Reject-Text 'src/RockConfig.cpp' 'bShoulderStashSkipActivate(Books|Notes)' `
    'Obsolete shoulder-stash collectible activation bypass INI keys must not be read.'
Reject-Text 'data/config/ROCK.ini' 'bShoulderStashSkipActivate(Books|Notes)' `
    'Packaged config must not advertise the obsolete collectible activation bypass.'

if ($failures.Count -gt 0) {
    Write-Host 'ShoulderStashTransferSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ShoulderStashTransferSourceTests passed.' -ForegroundColor Green

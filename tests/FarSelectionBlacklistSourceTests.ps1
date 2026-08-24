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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/object/FarSelectionBlacklistPolicy.h' 'FarSelectionBlacklistInput' `
    'Far selection blacklist must be a named policy with explicit inputs.'
Require-Text 'src/physics-interaction/object/FarSelectionBlacklistPolicy.h' 'blockedReferenceFormIds' `
    'Far selection blacklist must support probed reference form IDs.'
Require-Text 'src/physics-interaction/object/FarSelectionBlacklistPolicy.h' 'blockedBaseFormIds' `
    'Far selection blacklist must support probed base form IDs.'
Require-Text 'src/physics-interaction/object/FarSelectionBlacklistPolicy.h' 'blockedFormTypes' `
    'Far selection blacklist must support form-type blocks for doors, activators, and other broad classes.'
Require-Text 'src/physics-interaction/object/FarSelectionBlacklistPolicy.h' 'blockedLayers' `
    'Far selection blacklist must support layer blocks such as ANIMSTATIC.'
Require-Text 'src/physics-interaction/hand/HandSelection.h' 'kFarSelectionBlockedReferenceFormIds[\s\S]*kFarSelectionBlockedBaseFormIds[\s\S]*kFarSelectionBlockedFormTypes[\s\S]*kFarSelectionBlockedLayers' `
    'Far selection blacklist inputs must remain fixed compiled policy.'
Reject-Text 'src/RockConfig.h' 'rockFarSelectionBlocked' `
    'ROCK config must not retain far-selection blacklist storage.'
Reject-Text 'src/RockConfig.cpp' 'sFarSelectionBlocked' `
    'ROCK config must not parse far-selection blacklist values.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'far_selection_blacklist_policy::evaluateFarSelectionBlacklist' `
    'findFarObject must reject blacklisted candidates before they can highlight or pull.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'far-blacklist' `
    'Far-selection blacklist rejects need a stable diagnostic reason in selection telemetry.'

if ($failures.Count -gt 0) {
    Write-Host 'Far selection blacklist source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Far selection blacklist source boundary passed.'

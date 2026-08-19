param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Path {
    param(
        [string]$RelativePath,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: $Message")
    }
}

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

Require-Path 'data/mod/Meshes/ROCK/selection_beam_segment.nif' `
    'Selection beam NIF segment asset must be packaged as ROCK-owned data.'
Require-Text 'src/RockConfig.h' 'rockSelectionBeamEnabled[\s\S]*rockSelectionBeamSegmentSizeGameUnits[\s\S]*rockSelectionBeamCurveLiftGameUnits[\s\S]*rockSelectionBeamAlpha' `
    'RockConfig must expose selection beam settings.'
Require-Text 'src/RockConfig.cpp' 'bSelectionBeamEnabled[\s\S]*fSelectionBeamSegmentSizeGameUnits[\s\S]*fSelectionBeamCurveLiftGameUnits[\s\S]*fSelectionBeamAlpha' `
    'RockConfig must load selection beam settings from ROCK.ini.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void\s+Hand::reset\(\)[\s\S]*_selectionBeam\.shutdown\(\)' `
    'Hand reset must detach and release beam scene nodes while the scenegraph is still valid.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'abandonHavokStateAfterWorldLoss\(\)[\s\S]*_selectionBeam\.abandonSceneGraph\(\)' `
    'World-loss cleanup must abandon beam scene nodes without touching stale parent pointers.'
Require-Text 'src/RockConfig.h' 'SelectionBeamPolicy\.h' `
    'RockConfig must depend only on the lightweight beam policy defaults, not scenegraph runtime headers.'
Require-Text 'CMakeLists.txt' 'copy_directory\s+"\$\{ROOT_DIR\}/data/mod"\s+"\$\{copy_path\}"' `
    'Auto-deploy must copy packaged mesh data alongside the plugin.'

if ($failures.Count -gt 0) {
    Write-Host 'SelectionBeamSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'SelectionBeamSourceTests passed.' -ForegroundColor Green

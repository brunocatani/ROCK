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
Require-Text 'data/config/ROCK.ini' 'bSelectionBeamEnabled[\s\S]*fSelectionBeamSegmentSizeGameUnits[\s\S]*fSelectionBeamCurveLiftGameUnits[\s\S]*fSelectionBeamAlpha' `
    'Packaged ROCK.ini must document selection beam settings.'
Require-Text 'src/physics-interaction/hand/SelectionBeamEffect.cpp' 'Data/Meshes/ROCK/selection_beam_segment\.nif' `
    'Selection beam scenegraph effect must load the packaged ROCK mesh asset.'
Reject-Text 'src/physics-interaction/hand/SelectionBeamEffect.cpp' 'getClonedNiNodeForNifFileSetName' `
    'Selection beam must not reload the NIF once per segment; load one template and clone from it.'
Require-Text 'src/physics-interaction/hand/SelectionBeamEffect.cpp' 'loadNifFromFile[\s\S]*cloneNode\(_sourceTemplate\.get\(\)' `
    'Selection beam should preload one source template and clone segment nodes from it.'
Require-Text 'src/physics-interaction/hand/SelectionBeamEffect.cpp' 'void\s+SelectionBeamEffect::abandonSceneGraph\(\)[\s\S]*clearSegments\(false\)' `
    'Selection beam must have a stale-world cleanup path that does not dereference the old parent.'
Require-Text 'src/physics-interaction/hand/SelectionBeamEffect.cpp' 'void\s+SelectionBeamEffect::hide\(\)[\s\S]*if\s*\(!_active\)\s*\{[\s\S]*return;' `
    'Selection beam hide must be idempotent after the pool has been created.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'void\s+Hand::reset\(\)[\s\S]*_selectionBeam\.shutdown\(\)' `
    'Hand reset must detach and release beam scene nodes while the scenegraph is still valid.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'abandonHavokStateAfterWorldLoss\(\)[\s\S]*_selectionBeam\.abandonSceneGraph\(\)' `
    'World-loss cleanup must abandon beam scene nodes without touching stale parent pointers.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'preloadSelectionBeam\(\)[\s\S]*preloadSelectionBeam\(\)' `
    'Physics init must prewarm both beam pools outside first far-selection use.'
Require-Text 'src/physics-interaction/hand/SelectionBeamEffect.h' 'SelectionBeamPolicy\.h' `
    'Runtime beam owner should consume the lightweight policy header instead of duplicating policy logic.'
Require-Text 'src/RockConfig.h' 'SelectionBeamPolicy\.h' `
    'RockConfig must depend only on the lightweight beam policy defaults, not scenegraph runtime headers.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'isLocalSkeletonReady\(\)[\s\S]*stopSelectionBeam\(\)[\s\S]*stopSelectionBeam\(\)' `
    'Selection beam must be hidden immediately when the local skeleton is unavailable.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'updateSelectionBeam\(frame\.hknpWorld,\s*frame\.right\.grabAnchorWorld\)' `
    'Right hand selection update must refresh the beam from the real hand anchor.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'updateSelectionBeam\(frame\.hknpWorld,\s*frame\.left\.grabAnchorWorld\)' `
    'Left hand selection update must refresh the beam from the real hand anchor.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'frame\.right\.disabled[\s\S]*stopSelectionBeam\(\)' `
    'Disabled right hand frames must hide the beam.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'frame\.left\.disabled[\s\S]*stopSelectionBeam\(\)' `
    'Disabled left hand frames must hide the beam.'
Require-Text 'CMakeLists.txt' 'copy_directory\s+"\$\{ROOT_DIR\}/data/mod"\s+"\$\{copy_path\}"' `
    'Auto-deploy must copy packaged mesh data alongside the plugin.'

Reject-Text 'src/physics-interaction/hand/SelectionBeamEffect.cpp' 'DebugBodyOverlay|debug_overlay|PhysicsInteractionDebugOverlay|PerkUtilities|FUN_141d5ade0|CalculateProjectileTrajectory|VANS' `
    'Selection beam must not be implemented through debug overlay, VANS, or speculative vanilla grenade trajectory calls.'

if ($failures.Count -gt 0) {
    Write-Host 'SelectionBeamSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'SelectionBeamSourceTests passed.' -ForegroundColor Green

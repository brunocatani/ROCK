param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/object/PhysicsBodyClassifier.h' `
    'activeLooseWorldSurfaceLayer[\s\S]*input\.targetKind == grab_target::Kind::LooseObject[\s\S]*collision_layer_policy::isWorldSurfaceLayer\(input\.layer\)' `
    'Active-grab body classification must allow loose pickup refs that retain STATIC/ANIMSTATIC layers through the layer gate.'

Require-Text 'src/physics-interaction/object/PhysicsBodyClassifier.h' `
    'activeLooseWorldSurfaceLayer[\s\S]*interactionLayer[\s\S]*activeLooseWorldSurfaceLayer' `
    'Loose world-surface-layer bodies must be accepted through the normal interaction-layer decision, not a side fallback.'

Require-Text 'src/physics-interaction/object/PhysicsBodyClassifier.h' `
    'motion[\s\S]*checks below still fail closed unless prep produced a dynamic body' `
    'The loose static-layer exception must document that motion validation still owns runtime safety.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'diagnosticRejectedBodyRecord\([\s\S]*preparedBodySet[\s\S]*rejectReason=\{\}[\s\S]*rejectLayer=\{\}[\s\S]*rejectMotion=\{\}[\s\S]*rejectFlags=0x\{:08X\}[\s\S]*rejectMotionProps=\{\}' `
    'Pull/grab failure telemetry must expose the concrete body reject reason, layer, motion, flags, and motion props.'

if ($failures.Count -gt 0) {
    Write-Host 'Loose static-layer grab source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Loose static-layer grab source boundary passed.'

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

Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'SelectionRejectTelemetry' `
    'Selection rejection diagnostics must keep a focused per-hit telemetry record.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'rockDebugVerboseLogging' `
    'Selection rejection diagnostics must stay behind the existing verbose debug gate.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'classification->reason' `
    'Selection rejection logs must include the grab-target classification reason.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'hitNodeInsideActorRoot' `
    'Detached gore diagnostics must report whether the hit node is still owned by the actor root.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'motionTypeName' `
    'Selection rejection logs must expose the resolved body motion type.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'reject-stage=\{\}' `
    'Selection rejection logs must include an explicit rejection-stage field.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'logSelectionRejectTelemetry\(queryName,\s*"classification"' `
    'Selection logs must identify classification-stage rejections.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'logSelectionRejectTelemetry\(queryName,\s*"behind-palm"' `
    'Selection logs must identify behind-palm rejections.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'logSelectionRejectTelemetry\(queryName,\s*"hmd-cone"' `
    'Selection logs must identify HMD-cone rejections.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'bodyFlags=0x\{:08X\}' `
    'Selection logs must include hknp body flags for motion-state diagnosis.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'motionProps=\{\}' `
    'Selection logs must include motion-properties ids for ACTI/static/keyframed diagnosis.'

Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'RockConfig.h' `
    'Object body rejection diagnostics must use the verbose debug gate.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'rockDebugVerboseLogging' `
    'Object body rejection diagnostics must stay behind the existing verbose debug gate.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'BODY reject:' `
    'Object body scans must log accepted-ref bodies that fail active-grab classification.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'physics_body_classifier::rejectReasonName\(record\.rejectReason\)' `
    'Object body rejection logs must include the exact body classifier reason.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'targetKind=\{\}' `
    'Object body rejection logs must include the grab target kind.'
Require-Text 'src/physics-interaction/object/ObjectPhysicsBodySet.cpp' 'refKnown=\{\}' `
    'Object body rejection logs must distinguish resolved ownership from unresolved fallback.'

if ($failures.Count -gt 0) {
    Write-Host 'Selection rejection telemetry source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Selection rejection telemetry source boundary passed.'

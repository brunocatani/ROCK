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

Require-Text 'src/physics-interaction/object/GrabTargetKind.h' 'requiresHandPocketGrab' `
    'Grab target kind policy must expose the hand-pocket-only target set.'
Require-Text 'src/physics-interaction/object/GrabTargetKind.h' 'Kind::DynamicMovableStatic[\s\S]*Kind::DetachedGore[\s\S]*Kind::DeadActorBody' `
    'Dynamic MSTT, detached gore/deadbip, and dead actor bodies must all be hand-pocket-only targets.'
Require-Text 'src/physics-interaction/object/GrabTargetKind.h' 'canUseFarSelection[\s\S]*!requiresHandPocketGrab' `
    'Hand-pocket-only targets must not enter far selection/highlight.'
Require-Text 'src/physics-interaction/object/GrabTargetKind.h' 'canUseRockDynamicPull[\s\S]*!requiresHandPocketGrab' `
    'Hand-pocket-only targets must not enter pull-to-grab/gravity-glove dynamic pull.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'Kind::DeadActorBody' `
    'Dead actor bodies must classify separately from detached gore and blocked whole actor bodies.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'dead-actor-body-dynamic' `
    'Dead actor body classification needs a stable diagnostic reason.'
Require-Text 'src/physics-interaction/object/ObjectDetection.cpp' 'if \(isFarSelection && !grab_target::canUseFarSelection\(classification\.kind\)\)' `
    'Far selection must reject hand-pocket-only targets before highlighting or pull.'


if ($failures.Count -gt 0) {
    Write-Host 'Hand pocket only grab source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Hand pocket only grab source boundary passed.'

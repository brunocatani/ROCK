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

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'handPocketOnlyGrab\s*=\s*grab_target::requiresHandPocketGrab\(sel\.targetKind\)' `
    'Hand grab must derive a per-selection hand-pocket-only policy.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'closeGrabNeedsPalmPocketMeshAuthority[\s\S]*handPocketOnlyGrab' `
    'Hand-pocket-only targets must force palm-pocket mesh authority even outside ordinary close-grab fallback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'extractAllSurfaceTriangles[\s\S]*handPocketOnlyGrab' `
    'Hand-pocket-only grabs must request position-only skinned surface evidence when bone owners are unavailable.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'hand-pocket-only target requires pinch or palm-pocket support authority' `
    'Hand-pocket-only targets must fail instead of falling back to selection-hit, palm-ray, contact-patch, or collision pivots.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildRuntimePinchPocketCandidate[\s\S]*if \(handPocketOnlyGrab &&\s*!pinchPocketCandidate\.valid[\s\S]*failHandPocketOnlyGrab' `
    'Hand-pocket-only targets must evaluate final pinch authority before enforcing the palm-pocket support fallback gate.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'handPocketPositionOnlySkinnedSurface[\s\S]*!grabSurfaceHit\.hasSkinInfluences' `
    'Hand-pocket-only skinned surfaces without bone owners must be accepted only through the selected accepted body.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'if \(!handPocketOnlyGrab && contactSourcePolicy\.allowContactPatchPivot' `
    'Hand-pocket-only targets must not run contact-patch pivot selection.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'multiFingerEvidenceEnabled[\s\S]*!handPocketOnlyGrab' `
    'Hand-pocket-only targets must not run multi-finger contact evidence.'

if ($failures.Count -gt 0) {
    Write-Host 'Hand pocket only grab source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Hand pocket only grab source boundary passed.'

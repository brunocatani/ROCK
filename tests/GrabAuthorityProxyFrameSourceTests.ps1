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

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.h' 'tryGetPalmAnchorTarget' 'Generated hand collider runtime must expose the latest root-flattened palm target for hidden grab authority.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'Dynamic grab authority needs the same root-flattened palm convention' 'Palm target accessor must document why hknp readback is not authority.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' '_latestPalmAnchorTarget\s*=\s*anchorFrame\.transform' 'Palm target must be captured from the same sampled role frame queued for the generated palm anchor.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'resolveGrabAuthorityProxyFrame[\s\S]*rootFlattenedPalmAnchorTarget' 'Proxy authority must resolve from the root-flattened palm target before fallbacks.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'buildSplitGrabFrameFromDesiredObject\(\s*handWorldTransform,\s*proxyFrameWorldAtGrab' 'Constraint hand-space frame must be captured from the proxy/palm frame, not live stale hknp readback.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'objectToBodyAtGrab\s*=\s*computeRuntimeBodyLocalTransform\(objectWorldTransform,\s*grabBodyWorldAtGrab\)' 'Proxy authority must preserve the visual-object to hknp BODY relation captured at grab time.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyWorld\s*=\s*multiplyTransforms\(desiredObjectWorld,\s*objectToBodyAtGrab\)' 'Proxy authority must derive the desired BODY frame from the desired visual object frame instead of treating visual local coordinates as body local.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.constraintBodyHandSpace\s*=\s*grab_frame_math::objectInFrameSpace\(proxyFrameWorldAtGrab,\s*desiredBodyWorld\)' 'Proxy authority must store a BODY-space relation separate from the visual object hand-space relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'createProxyConstraintGrabDrive\(\s*bhkWorld,\s*world,\s*objectBodyId,\s*proxyFrameWorldAtGrab' 'Proxy constraint creation must use the resolved palm-frame proxy transform.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'desiredBodyTransformProxySpace\s*=\s*_grabFrame\.constraintBodyHandSpace' 'Proxy constraint target math must use the captured BODY-space proxy relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outActivePivotBBodyLocalGame\s*=\s*activeGrabDrivePivotBBodyLocalGame\(\)' 'Proxy constraint target math must keep using the frozen BODY-local grip pivot.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'transformBTranslation\[0\]\s*=\s*outActivePivotBBodyLocalGame\.x\s*\*\s*gameToHkScale' 'Transform-B must be written from the frozen BODY-local pivot, not a recomputed visual/object local point.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.constraintHandSpace\)' 'Proxy desired object frame must be composed from the proxy palm frame and constraint hand-space relation.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'queueProxyGrabAuthorityTarget\(\s*proxyAuthorityWorld' 'Held updates must queue the resolved palm-frame proxy target.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveQueuedTargets\s*=\s*_grabAuthorityProxyQueuedSequence' 'Held telemetry must report proxy queue counters when proxy authority is active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'driveFlushedTargets\s*=\s*_grabAuthorityProxyFlushSequence' 'Held telemetry must report proxy flush counters when proxy authority is active.'
Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' 'outDesiredObjectWorld\s*=\s*multiplyTransforms\(proxyWorldTransform,\s*_grabFrame\.rawHandSpace\)' 'Proxy body-A convention must not use the raw controller hand-space relation.'

if ($failures.Count -gt 0) {
    Write-Host 'Grab authority proxy frame source test failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Grab authority proxy frame source test passed.'

param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

# --- Storage layout -----------------------------------------------------------

# --- The capture must carry the question, not just the answer ------------------

Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*_grabFrame\.localMeshTriangles' 'The capture must record the cached mesh the grab machinery actually scored; reconstructing geometry from the NIF offline would silently score different triangles (replacers, scale, skinning, node selection).'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*mesh\.nodeInObjectRoot = storeFrame\(grab_frame_math::objectInGeneratedProxyLocalSpace\(rootNode->world, meshNode->world\)\)' 'The capture must relate the mesh node space to the object root: the saved pose is root-relative while the mesh is node-relative, so a nested collidable node would misalign the label against the geometry.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*dynamicTwinTargets\(\)' 'The capture must record the driven hand collider frames, so a solver is fitted against the real hand volumes rather than a reconstruction of them.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*grab_frame_math::objectInGeneratedProxyLocalSpace\(proxyWorld' 'Every captured frame must be expressed in the same proxy-local space as the saved offset, through the same helper, or label and context cannot be compared.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*seat\.objectProxyLocal = inProxyLocal\(telemetry\.desiredObjectWorld\)' 'The capture must record the seat ROCK itself committed: every ground-truth record is a before/after pair, and without the before there is no measurable error for a solver to close.'

# --- Seat diagnostics survive later authority rewrites ------------------------

Require-Text 'src/physics-interaction/grab/GrabCore.h' 'captureTelemetry\.seatDiagnostics = seatDiagnostics;' 'Seat diagnostics must be frozen into the immutable capture telemetry: the live frame can be rewritten by reacquire, and the capture describes the CAPTURE-time decision.'

# --- Save gesture wiring ------------------------------------------------------


# --- Write-only contract ------------------------------------------------------

# --- Hand volume and centre of mass ------------------------------------------

Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*_boneColliders\.segmentColliderFrames\(\)' 'The capture must record the full driven hand volume from the published segment frames.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*getBodyMotion\(world, getSavedObjectState\(\)\.bodyId\)[\s\S]*physics\.comTrusted = insideBounds;[\s\S]*ROCK_LOG_WARN' 'The centre of mass must come from the held body motion and be gated on the mesh bounds, loudly flagging an implausible read - a rigid body COM is always inside its own hull, so outside means the offset is wrong.'

if ($failures.Count -gt 0) {
    Write-Host 'Saved grab capture boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Saved grab capture boundary passed.'

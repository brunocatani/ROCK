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

Require-Text 'src/physics-interaction/grab/SavedGrabOffsetStore.cpp' 'capturePathForObject\(const FormRef& object, const std::string& hand\) const[\s\S]*_directory \+ "\\\\captures\\\\"' 'Ground-truth captures must be written to a captures SUBDIRECTORY: preload() parses every *.json in the offset directory as an offset file, so a sibling capture would be counted unreadable on every boot.'
Require-Text 'src/physics-interaction/grab/SavedGrabOffsetStore.cpp' 'capturePathForObject\(capture\.object, capture\.hand\)' 'Capture files must be keyed per object AND per hand: captures are never read back, so a merged right/left document could not be updated without discarding the other hand.'
Require-Text 'src/physics-interaction/grab/SavedGrabOffsetStore.cpp' 'void saveCapture\([\s\S]*enqueue\(PendingWrite' 'Capture writes must ride the existing background writer queue, never touch the frame thread with disk I/O.'

# --- The capture must carry the question, not just the answer ------------------

Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*_grabFrame\.localMeshTriangles' 'The capture must record the cached mesh the grab machinery actually scored; reconstructing geometry from the NIF offline would silently score different triangles (replacers, scale, skinning, node selection).'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*mesh\.nodeInObjectRoot = storeFrame\(grab_frame_math::objectInGeneratedProxyLocalSpace\(rootNode->world, meshNode->world\)\)' 'The capture must relate the mesh node space to the object root: the saved pose is root-relative while the mesh is node-relative, so a nested collidable node would misalign the label against the geometry.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*dynamicTwinTargets\(\)' 'The capture must record the driven hand collider frames, so a solver is fitted against the real hand volumes rather than a reconstruction of them.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*grab_frame_math::objectInGeneratedProxyLocalSpace\(proxyWorld' 'Every captured frame must be expressed in the same proxy-local space as the saved offset, through the same helper, or label and context cannot be compared.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' 'bool Hand::tryBuildSavedGrabCapture[\s\S]*seat\.objectProxyLocal = inProxyLocal\(telemetry\.desiredObjectWorld\)' 'The capture must record the seat ROCK itself committed: every ground-truth record is a before/after pair, and without the before there is no measurable error for a solver to close.'

# --- Seat diagnostics survive later authority rewrites ------------------------

Require-Text 'src/physics-interaction/grab/GrabCore.h' 'captureTelemetry\.seatDiagnostics = seatDiagnostics;' 'Seat diagnostics must be frozen into the immutable capture telemetry: the live frame can be rewritten by reacquire, and the capture describes the CAPTURE-time decision.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' '_grabFrame\.seatDiagnostics = GrabSeatDiagnostics\{[\s\S]*\.acquisitionMode =[\s\S]*\.shapeClass =[\s\S]*\.penetrationBackstopReason = seatPenetrationBackstopReason' 'Grab capture must record the acquisition mode, shape class and every seat correction outcome, so a ground-truth capture explains which decisions produced the seat it is paired with.'

# --- Save gesture wiring ------------------------------------------------------

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'saved_grab_offset::save\(file\);[\s\S]*tryBuildSavedGrabCapture\(proxyWorld, capture\.capture\)[\s\S]*saved_grab_offset::saveCapture\(capture\)' 'The save gesture must write the ground-truth capture alongside the offset, or a verified pose arrives with no context to score it against.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'readGrabEventBodyMass\(hknpWorld, heldBodyId\)' 'The capture must record the held body mass: mass is not derivable from the render mesh, and balance in the palm is one of the terms a pose solver has to explain.'

# --- Write-only contract ------------------------------------------------------

Reject-Text 'src/physics-interaction/grab/SavedGrabCaptureFormat.h' 'bool parse\(' 'Captures are write-only ground-truth records consumed offline; adding a runtime parse path would make them load-bearing for gameplay and put their size on the boot path.'
Require-Text 'src/physics-interaction/grab/SavedGrabCaptureFormat.h' 'must NOT be verified against the FO4VR binary|has NOT been verified against the FO4VR binary' 'The physics capture must keep its unverified-centre-of-mass caveat: the Havok body frame is recorded as raw data and must not be read as a COM until the layout is confirmed from disassembly.'

if ($failures.Count -gt 0) {
    Write-Host 'Saved grab capture boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Saved grab capture boundary passed.'

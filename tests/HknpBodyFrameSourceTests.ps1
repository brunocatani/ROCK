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

Require-Text 'src/physics-interaction/TransformMath.h' 'hknpBodyColumnsToNiStoredAxes' 'TransformMath must expose an explicit FO4VR hknp BODY read conversion.'
Require-Text 'src/physics-interaction/TransformMath.h' 'niStoredAxesToHknpBodyColumns' 'TransformMath must expose an explicit FO4VR hknp BODY write conversion.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'havokRotationBlocksToNiMatrix[\s\S]*hknpBodyColumnsToNiStoredAxes' 'hknp BODY readback must copy native local-axis blocks into ROCK stored axes.'
Require-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'setBodyTransformDeferred[\s\S]*niStoredAxesToHknpBodyColumns' 'Deferred BODY writes must preserve ROCK stored axes as native hknp local-axis blocks.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'bodyToWorldMatrix[\s\S]*XMMatrixSet\(transform\[0\],\s*transform\[1\],\s*transform\[2\][\s\S]*transform\[4\],\s*transform\[5\],\s*transform\[6\][\s\S]*transform\[8\],\s*transform\[9\],\s*transform\[10\]' 'Target collider visualizer must draw hknp BODY axis blocks with the same non-transposed convention used by solver BODY readback.'
Require-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'motionToShapeWorldMatrix[\s\S]*bodyTransform\[3\][\s\S]*bodyTransform\[7\][\s\S]*bodyTransform\[11\][\s\S]*XMVector3TransformNormal[\s\S]*position\[0\]\s*-\s*centerOfMassOffset\.x' 'Live MOTION readback must convert center-of-mass world back to the hknp BODY/shape origin before drawing generated colliders and body-backed axes.'
Reject-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'havokRotationBlocksToNiMatrix[\s\S]*havokColumnsToNiRows' 'hknp BODY readback must not use the old transpose helper.'
Reject-Text 'src/physics-interaction/native/HavokRuntime.cpp' 'setBodyTransformDeferred[\s\S]*niRowsToHavokColumns' 'Deferred BODY writes must not transpose ROCK stored axes at the hknp BODY boundary.'
Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'bodyToWorldMatrix[\s\S]*XMMatrixSet\(transform\[0\],\s*transform\[4\],\s*transform\[8\]' 'Target collider visualizer must not transpose hknp BODY axis blocks when building the D3D model matrix.'
Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' 'motionToWorldMatrix\(position,\s*orientation\)' 'Debug overlay must not place a BODY shape directly at the hknp MOTION center of mass.'

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host "hknp BODY frame source policy checks passed."

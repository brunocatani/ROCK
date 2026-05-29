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

Require-Text 'src/physics-interaction/hand/HandColliderTypes.h' 'generatedColliderLocalVectorToWorld[\s\S]*matrix\.entry\[0\]\[1\]\s*\*\s*localVector\.y[\s\S]*matrix\.entry\[1\]\[2\]\s*\*\s*localVector\.z' 'Generated collider local-vector conversion must read the stored column axes.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'zoneOverride\.hasLocalOffset[\s\S]*generatedColliderLocalVectorToWorld\(outFrame\.transform,\s*zoneOverride\.localOffsetGame\)' 'Body zone local offsets must use generated collider stored-column space.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'generatedProxyLocalVectorToWorld[\s\S]*generatedColliderLocalVectorToWorld\(proxyFrameWorld,\s*localVector\)' 'Grab proxy local offsets must share the generated collider stored-column converter.'

Reject-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'transform_math::localVectorToWorld\(outFrame\.transform,\s*zoneOverride\.localOffsetGame\)' 'Body zone local offsets must not use row-axis TransformMath.'
Reject-Text 'src/physics-interaction/hand/HandFrame.h' 'transform_math::localVectorToWorld\(proxyFrameWorld,\s*localOffset\)' 'Grab proxy local offsets must not use row-axis TransformMath.'

if ($failures.Count -gt 0) {
    Write-Host 'GeneratedColliderLocalOffsetSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'GeneratedColliderLocalOffsetSourceTests passed.' -ForegroundColor Green

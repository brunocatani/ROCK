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

Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'bodyCollisionObjectMismatch' 'Generated body drive results must expose stale body/collision-object ownership failures.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'shouldRequestRebuild\(\)[\s\S]*bodyCollisionObjectMismatch' 'Generated collider owners must rebuild when a live body no longer points back to the wrapper collision object.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'liveBody = world \? havok_runtime::getBody\(world, body\.getBodyId\(\)\) : nullptr[\s\S]*getCollisionObjectFromBody\(liveBody\)[\s\S]*bodyCollisionObjectMismatch = true[\s\S]*return result;[\s\S]*driveToKeyFrame' 'Generated body drive must fail closed on body/collision-object mismatch before calling the native keyframe drive.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'ownerMismatch=\{\}' 'Proxy grab release logs must distinguish collision-object ownership mismatches from missing bodies.'

if ($failures.Count -gt 0) {
    Write-Host 'GeneratedKeyframedBodyDriveSafetySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GeneratedKeyframedBodyDriveSafetySourceTests passed.' -ForegroundColor Green

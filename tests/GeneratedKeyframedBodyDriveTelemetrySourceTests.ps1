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

Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'targetToBodyRotationDegrees' 'Generated body drive results must expose target-vs-live rotation error.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' 'targetAxisXWorld[\s\S]*liveBodyAxisZWorld' 'Generated body drive results must expose target and live body basis axes.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'fillRotationReadbackTelemetry' 'Generated body drive must compute rotation readback telemetry from actual live body transforms.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'Generated body frame compare owner=\{\} bodyIndex=\{\} bodyId=\{\}' 'Grab-frame logging must emit sampled generated-body frame compare lines.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'bodyRotErr=\{:\.2f\} axisDeg=\(\{:\.1f\},\{:\.1f\},\{:\.1f\}\)' 'Generated body frame compare logs must include total and per-axis rotation error.'
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' 'g_rockConfig\.rockDebugGrabFrameLogging' 'Generated body frame compare logs must be available with grab-frame diagnostics, not only global verbose logging.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'bodyRotErr=\{:\.2f\}' 'Hand generated collider rebuild warnings must include rotation error.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'bodyRotErr=\{:\.2f\}' 'Body generated collider rebuild warnings must include rotation error.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'bodyRotErr=\{:\.2f\}' 'Weapon generated collider rebuild warnings must include rotation error.'

if ($failures.Count -gt 0) {
    Write-Host 'GeneratedKeyframedBodyDriveTelemetrySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'GeneratedKeyframedBodyDriveTelemetrySourceTests passed.' -ForegroundColor Green

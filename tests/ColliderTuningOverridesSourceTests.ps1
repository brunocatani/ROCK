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

Require-Text 'src/RockConfig.h' 'rockBodyBoneColliderRadiusScaleOverrides' 'Config must expose body per-collider radius scale overrides.'
Require-Text 'src/RockConfig.h' 'rockHandBoneColliderRadiusScaleOverrides' 'Config must expose hand capsule-like radius scale overrides.'
Require-Text 'src/RockConfig.h' 'rockHandPalmColliderDimensionScaleOverrides' 'Config must expose palm-box local X/Y/Z scale overrides.'
Require-Text 'src/RockConfig.cpp' '"sBodyBoneColliderRadiusScaleOverrides"' 'Config loader must read body per-collider radius scale overrides.'
Require-Text 'src/RockConfig.cpp' '"sHandBoneColliderRadiusScaleOverrides"' 'Config loader must read hand capsule-like radius scale overrides.'
Require-Text 'src/RockConfig.cpp' '"sHandPalmColliderDimensionScaleOverrides"' 'Config loader must read palm-box X/Y/Z scale overrides.'

Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'bodyRadiusScaleOverride\(descriptor,\s*inPowerArmor\)' 'Body collider frame construction must apply per-collider radius overrides.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'mixBodyColliderSignatureString\(signature,\s*g_rockConfig\.rockBodyBoneColliderRadiusScaleOverrides\)' 'Body collider rebuild signature must include radius override changes.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'StartBone|bonePairOverrideKeyMatches' 'Body radius overrides must support bone-pair identity, not only broad role scales.'


if ($failures.Count -gt 0) {
    Write-Host 'ColliderTuningOverridesSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'ColliderTuningOverridesSourceTests passed.' -ForegroundColor Green

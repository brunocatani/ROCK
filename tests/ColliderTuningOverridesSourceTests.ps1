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
Require-Text 'data/config/ROCK.ini' 'sBodyBoneColliderRadiusScaleOverrides' 'Default INI must document body per-collider radius scale overrides.'
Require-Text 'data/config/ROCK.ini' 'sHandBoneColliderRadiusScaleOverrides' 'Default INI must document hand capsule-like radius scale overrides.'
Require-Text 'data/config/ROCK.ini' 'sHandPalmColliderDimensionScaleOverrides' 'Default INI must document palm-box X/Y/Z scale overrides.'

Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'bodyRadiusScaleOverride\(descriptor,\s*inPowerArmor\)' 'Body collider frame construction must apply per-collider radius overrides.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'mixBodyColliderSignatureString\(signature,\s*g_rockConfig\.rockBodyBoneColliderRadiusScaleOverrides\)' 'Body collider rebuild signature must include radius override changes.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'StartBone|bonePairOverrideKeyMatches' 'Body radius overrides must support bone-pair identity, not only broad role scales.'

Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.h' '_cachedTuningSignature' 'Hand collider set must track config tuning changes for shape rebuilds.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'handColliderTuningSignature' 'Hand collider implementation must compute a tuning signature from hand override strings.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' '_cachedTuningSignature\s*!=\s*tuningSignature' 'Hand collider update must rebuild when hand tuning changes.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'hand_collider_semantics::isPalmRole\(role\)\s*\|\|\s*g_rockConfig\.rockHandBoneColliderRadiusScaleOverrides\.empty\(\)' 'Hand radius overrides must not be the palm-box tuning path.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'palmDimensionScaleOverride\(role,\s*_lastCapturedPowerArmor\)' 'Palm-box shape construction must consume X/Y/Z scale overrides.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'makePalmBoxHullPoints<RE::NiPoint3>\(length,\s*scaledPalmDepth,\s*scaledCrossPalmWidth\)' 'Palm-box hulls must apply separate local Y and Z dimensions.'
Require-Text 'src/physics-interaction/hand/HandBoneColliderSet.cpp' 'colliderDimensionsWithinLimits[\s\S]*scaledCrossPalmWidth' 'Palm-box dimension overrides must be bounded before Havok shape creation.'

if ($failures.Count -gt 0) {
    Write-Host 'ColliderTuningOverridesSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure" -ForegroundColor Red
    }
    exit 1
}

Write-Host 'ColliderTuningOverridesSourceTests passed.' -ForegroundColor Green

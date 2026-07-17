param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/native/HavokOffsets.h' `
    'kFunc_PlayerPostUpdateAnimationGraphManager\s*=\s*0xF2F0A0' `
    'The selective capture hook must stay pinned to the independently verified FO4VR function entry.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    '0x48,\s*0x8B,\s*0xC4,\s*0x55,\s*0x48,\s*0x83,\s*0xEC,\s*0x60,[\s\S]*0x90,\s*0x90,\s*0x90,\s*0x90,\s*0x90,\s*0x90' `
    'Hook installation must validate the native prologue plus hFRIK post-patch NOP identity.'
Require-Text 'src/ROCKMain.cpp' `
    'case\s+LE::kSkeletonReady:[\s\S]*installPostUpdateHook\(\)' `
    'ROCK must install only after the FRIK skeleton-ready boundary.'
Require-Text 'src/ROCKMain.cpp' `
    'beginRockFrame\(\)[\s\S]*applyCapturedPose\(\)[\s\S]*onFrameUpdate\(\)[\s\S]*applyCapturedPose\(\)[\s\S]*completeRockFrame\(\)' `
    'The captured pose must bracket ROCK sampling and remain the final visual writer.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'equalsIgnoreCase\(name,\s*"Weapon"\)[\s\S]*LArm_[\s\S]*RArm_' `
    'Bone authority must be an explicit arms/hands/two-weapon-root allowlist.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'classifyBone[\s\S]*return\s+kReloadPose' `
    'The classifier must never broadly grant every pose flag to arbitrary body bones.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'ROCK_PROVIDER_API_VERSION\s*=\s*1[\s\S]*NativeAnimationAuthority[\s\S]*setNativeAnimationAuthorityV1[\s\S]*clearNativeAnimationAuthorityV1' `
    'The authority lease must append to API V1 without a version bump.'
Require-Text 'src/api/ROCKProviderApi.cpp' `
    'clearNativeAnimationAuthorityForOwnerLocked\(ownerToken\)' `
    'Consumer unregister must deterministically release native animation authority.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'rockNativeReloadAnimationAuthorityTestEnabled[\s\S]*requestLocalReloadTestLease\(\)' `
    'The ROCK-only validation flag must arm a bounded lease only when a native reload dispatch succeeds.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'NativeAnimationAuthoritySourceTests passed.' -ForegroundColor Green

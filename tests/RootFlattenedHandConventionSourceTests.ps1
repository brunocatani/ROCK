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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
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

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

$handConventionFiles = @(
    'src/physics-interaction/HandBoneCache.h',
    'src/physics-interaction/RootFlattenedFingerSkeletonRuntime.cpp',
    'src/physics-interaction/GrabFingerLocalTransformRuntime.h',
    'src/physics-interaction/TwoHandedGrip.cpp'
)

foreach ($file in $handConventionFiles) {
    Require-Text $file 'DirectSkeletonBoneReader' "$file must resolve live hand/finger transforms through DirectSkeletonBoneReader."
    Require-Text $file 'DebugSkeletonBoneSource::GameRootFlattenedBoneTree' "$file must use the same root flattened bone tree source as generated hand colliders."
    Reject-Text $file 'getFirstPersonSkeleton\s*\(' "$file must not read the first-person skeleton for hand/grab/finger authority."
    Reject-Text $file 'getFirstPersonBoneTree\s*\(' "$file must not read the first-person bone tree for hand/grab/finger authority."
}

Require-Text 'src/physics-interaction/HandFrameResolver.h' 'root flattened' 'Hand frame resolver must document root flattened hand-frame authority.'
Reject-Text 'src/physics-interaction/HandFrameResolver.h' 'first-person skeleton hand bones directly' 'Hand frame resolver must not describe first-person skeleton authority.'
Reject-Text 'src/physics-interaction/GrabFingerPoseRuntime.h' 'first-person finger skeleton' 'Finger pose runtime comments must not describe first-person skeleton authority.'
Reject-Text 'src/physics-interaction/GrabFingerPoseRuntime.h' 'FrikFingerSkeletonRuntime|frik_finger_skeleton_runtime' 'Finger pose runtime must use root flattened runtime names.'
Reject-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'FrikFingerSkeleton|frik_finger_skeleton_runtime' 'Physics interaction debug/runtime names must not claim FRIK finger skeleton authority.'
Require-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'resolve\(\)' 'PhysicsInteraction must refresh the copied root flattened hand transforms through HandBoneCache::resolve.'
Require-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'fillProviderTransform\(_handBoneCache\.getWorldTransform\(false\),\s*outSnapshot\.rightHandTransform\)' 'Provider right-hand transform must use the root flattened hand cache.'
Require-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'fillProviderTransform\(_handBoneCache\.getWorldTransform\(true\),\s*outSnapshot\.leftHandTransform\)' 'Provider left-hand transform must use the root flattened hand cache.'
Require-Text 'src/api/ROCKApi.cpp' 'tryGetRootFlattenedHandTransform' 'Public ROCK hand palm helpers must use the root flattened hand transform accessor.'
Reject-Text 'src/api/ROCKApi.cpp' 'getHandWorldTransform\s*\(' 'Public ROCK hand palm helpers must not use FRIK API hand transforms.'
Require-Text 'src/physics-interaction/PhysicsInteraction.cpp' '!rootHandReady' 'Frame hand input must be disabled when the root flattened hand cache is unavailable.'
Reject-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'fillProviderTransform\(api->getHandWorldTransform' 'Provider hand transforms must not use FRIK API hand transforms.'
Reject-Text 'src/physics-interaction/PhysicsInteraction.cpp' 'getNode\(isLeft\)' 'PhysicsInteraction must not fetch first-person hand nodes for the interaction frame.'
Require-Text 'src/physics-interaction/TwoHandedGrip.cpp' 'tryGetHandBoneTransform' 'Two-handed weapon support must treat missing root flattened hands as failure, not identity fallback.'
Reject-Text 'src/physics-interaction/TwoHandedGrip.cpp' 'getHandWorldTransform\s*\(' 'Two-handed weapon support must not use FRIK API hand transforms as hand-frame authority.'

if ($failures.Count -gt 0) {
    Write-Host 'Root flattened hand convention failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Root flattened hand convention passed.'

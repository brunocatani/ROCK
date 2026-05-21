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

Require-Text 'src/physics-interaction/hand/HandSkeleton.h' 'HandBoneCache::resolve\(\)[\s\S]*buildSemanticHandFrameFromSnapshot\(snapshot,\s*false[\s\S]*buildSemanticHandFrameFromSnapshot\(snapshot,\s*true' `
    'HandBoneCache must derive both semantic hand frames from the already refreshed root-flattened skeleton snapshot.'
Require-Text 'src/physics-interaction/hand/HandSkeleton.h' 'getSemanticHandFrame\(bool isLeft[\s\S]*CachedSemanticHandFrameData' `
    'HandBoneCache must expose copied per-frame semantic hand-frame data instead of forcing hot-path recaptures.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' '_handBoneCache\.getSemanticHandFrame\(isLeft,\s*semanticHandFrame\)' `
    'Frame input must consume the cached semantic frame from HandBoneCache.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'transformSemanticHandFrameDirection\([\s\S]*semanticHandFrame[\s\S]*rockPointingVectorHandspace[\s\S]*rockReverseFarGrabNormal' `
    'Pointing direction must be interpreted in the unified root-flattened semantic frame, not legacy authored handspace.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionFrame.inl' 'computePointingVectorFromHandBasis|resolveLiveSemanticHandFrame' `
    'Frame input must not call legacy pointing conversion or recapture semantic hand frames in the hot path.'
Require-Text 'data/config/ROCK.ini' 'fPointingVectorHandspaceX\s*=\s*0\.0[\s\S]*fPointingVectorHandspaceY\s*=\s*-1\.0[\s\S]*fPointingVectorHandspaceZ\s*=\s*0\.0[\s\S]*bReverseFarGrabNormal\s*=\s*false' `
    'Packaged INI must keep semantic -Y with reverse=false so far selection resolves to palm-face by default.'
Require-Text 'src/api/ROCKApi.cpp' 'tryGetRootFlattenedSemanticHandFrame' `
    'Public palm API must read the cached semantic frame through PhysicsInteraction.'
Require-Text 'src/physics-interaction/hand/HandFrame.h' 'LEGACY HANDSPACE WARNING[\s\S]*SemanticHandFrame[\s\S]*Do not use these legacy helpers for dynamic grab' `
    'Legacy authored handspace must carry a loud warning that new runtime hand code uses SemanticHandFrame instead.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' 'TWO-HANDED WEAPON HANDSPACE WARNING[\s\S]*compatibility island[\s\S]*Do not copy hand-space logic from this file' `
    'Two-handed weapon support must be explicitly marked as an isolated handspace exception.'

if ($failures.Count -gt 0) {
    Write-Host 'SemanticHandFrameSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'SemanticHandFrameSourceTests passed.' -ForegroundColor Green

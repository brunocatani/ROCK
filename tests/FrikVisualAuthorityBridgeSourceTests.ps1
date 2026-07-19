param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$workspaceRoot = Split-Path -Parent $Root

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

function Reject-ExternalText {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $workspaceRoot $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Require-ExternalText {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $workspaceRoot $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'g_cachedHandPosePublications' `
    'FRIK scalar hand-pose publication must retain a bounded cache to avoid duplicate hot-path provider calls.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'getHandPoseSetTagState\(tag,\s*hand\)\s*==\s*HandPoseTagState::Active' `
    'Cached FRIK hand-pose skips must verify the tag is still active before suppressing a publish.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'sameHandPoseData\(entry->pose,\s*handPose\)' `
    'Cached FRIK hand-pose skips must compare the full hand-pose payload.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' 'clearHandPose\([^)]*\)[\s\S]*invalidateCachedHandPosePublication\(tag,\s*hand\)' `
    'Clearing a FRIK hand-pose tag must invalidate ROCK cached publication state.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'g_cachedFingerLocalTransformPublications[\s\S]*sameFingerLocalTransforms[\s\S]*shouldSkipCachedFingerLocalTransformPublication[\s\S]*invalidateCachedFingerLocalTransformPublication' `
    'Exact finger-local publication must use bounded active-tag caching and invalidate with its scalar owner.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'FRIKAPI_MirrorPrimaryWeaponFingerLocalTransforms[\s\S]*mirrorPrimaryWeaponFingerLocalTransforms' `
    'ROCK must feature-detect hFRIK''s anatomical primary-pose mirror without changing the FRIK function-table ABI.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'FRIKAPI_MirrorFingerLocalTransforms[\s\S]*mirrorFingerLocalTransforms\(Hand sourceHand[\s\S]*sourceHand\s*==\s*Hand::Right[\s\S]*legacyFn' `
    'ROCK must feature-detect bidirectional anatomical mirroring while retaining right-to-left compatibility with older hFRIK builds.'

Reject-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' 'logger::info\("Hand pose:' `
    'hFRIK must not info-log hand-pose override set/clear operations from the runtime hot path.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' 'logger::debug\("Hand pose:' `
    'hFRIK hand-pose stack transition diagnostics should remain debug-only.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' `
    'mirrorFingerLocalTransforms[\s\S]*sourceIsLeft[\s\S]*tryTransferMirroredThumbBase[\s\S]*measureAnimatedFlexSplay[\s\S]*blendBoneRotation' `
    'hFRIK must mirror harvested finger locals in either physical direction through the same anatomy-aware path.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' `
    'Skeleton::isPrimaryWeaponNodeOwnershipBlocked\(\)\s*&&\s*!isPrimaryWeaponPoseBlocked\(\)' `
    'A blocked native primary pose must allow ROCK''s exact explicit pose to win during physical-left firing carry.'
Require-ExternalText 'hFRIK/src/api/FRIKApi.cpp' `
    'FRIKAPI_MirrorFingerLocalTransforms[\s\S]*sourceHand\s*!=\s*FRIKApi::Hand::Left[\s\S]*sourceHand\s*!=\s*FRIKApi::Hand::Right[\s\S]*mirrorFingerLocalTransforms[\s\S]*FRIKAPI_MirrorPrimaryWeaponFingerLocalTransforms[\s\S]*FRIKAPI_MirrorFingerLocalTransforms\(FRIKApi::Hand::Right' `
    'hFRIK must expose a bidirectional standalone mirror export and retain the original primary-pose export as a compatibility wrapper.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

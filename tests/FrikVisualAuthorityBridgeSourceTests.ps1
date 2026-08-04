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
    'FRIKAPI_MirrorFingerLocalTransforms[\s\S]*mirrorFingerLocalTransforms\(Hand sourceHand[\s\S]*fn\(sourceHand,\s*&sourceTransforms,\s*&outTargetTransforms\)' `
    'ROCK must use hFRIK''s single bidirectional anatomical mirror export.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'blockPrimaryHandWeaponPose\(const char\* tag,\s*bool block\)[\s\S]*api\(\)[\s\S]*frikApi->blockPrimaryHandWeaponPose\(tag,\s*block\)' `
    'ROCK must use the V5 function table for primary weapon-pose blocking.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'canBlockPrimaryHandWeaponPose\(\)[\s\S]*frikApi->blockPrimaryHandWeaponPose\s*!=\s*nullptr' `
    'ROCK must feature-detect primary weapon-pose blocking through the V5 table.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'PresentedHandNodeCache[\s\S]*getHandWorldTransform\(Hand hand\)[\s\S]*isSkeletonReadyHint\(\)[\s\S]*getFirstPersonSkeleton\(\)[\s\S]*findNode\(skeleton,\s*"RArm_Hand"\)[\s\S]*findNode\(skeleton,\s*"LArm_Hand"\)[\s\S]*handNode->world' `
    'ROCK must read final presented hands directly from the game first-person scene nodes.'
Require-Text 'src/ROCKMain.cpp' `
    'kSkeletonReady[\s\S]*resetPresentedHandNodeCache\(\)[\s\S]*kSkeletonDestroying[\s\S]*resetPresentedHandNodeCache\(\)' `
    'Game hand-node caches must be invalidated at both hFRIK skeleton lifecycle edges.'

Reject-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' 'logger::(?:info|debug)\("Hand pose:' `
    'hFRIK must not retain log-only hand-pose override bookkeeping in the runtime hot path.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' `
    'mirrorFingerLocalTransforms[\s\S]*sourceIsLeft[\s\S]*tryTransferMirroredThumbBase[\s\S]*measureAnimatedFlexSplay[\s\S]*blendBoneRotation' `
    'hFRIK must mirror harvested finger locals in either physical direction through the same anatomy-aware path.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' `
    'Skeleton::isPrimaryWeaponNodeOwnershipBlocked\(\)\s*&&\s*!isPrimaryWeaponPoseBlocked\(\)' `
    'A blocked native primary pose must allow ROCK''s exact explicit pose to win during physical-left firing carry.'
Require-ExternalText 'hFRIK/src/api/FRIKApi.cpp' `
    'FRIKAPI_MirrorFingerLocalTransforms[\s\S]*sourceHand\s*!=\s*FRIKApi::Hand::Left[\s\S]*sourceHand\s*!=\s*FRIKApi::Hand::Right[\s\S]*mirrorFingerLocalTransforms' `
    'hFRIK must expose the bidirectional anatomy-aware mirror export.'
Reject-ExternalText 'hFRIK/src/api/FRIKApi.cpp' `
    'FRIKAPI_BlockPrimaryHandWeaponPose|FRIKAPI_MirrorPrimaryWeaponFingerLocalTransforms' `
    'hFRIK must not retain duplicate direct-export paths for V5 table behavior or legacy mirroring.'
Reject-ExternalText 'hFRIK/src/api/FRIKApi.h' `
    'getHandWorldTransform' `
    'hFRIK V5 must not wrap game-owned first-person hand transforms.'
Reject-ExternalText 'ROCK/src/api/FRIKApi.h' `
    'getHandWorldTransform' `
    'ROCK must not retain the removed hFRIK hand-transform table entry.'
Reject-ExternalText 'hFRIK/src/api/FRIKApi.h' `
    'using\s+HandPoses|setHandPoseCustomFingerPositionsWithPriority|kPowerArmorChanged|Fist\s*=' `
    'hFRIK V5 must not retain unused aliases, convenience calls, lifecycle events, or public fist exposure.'
Reject-ExternalText 'ROCK/src/api/FRIKApi.h' `
    'using\s+HandPoses|setHandPoseCustomFingerPositionsWithPriority|kPowerArmorChanged|Fist\s*=' `
    'ROCK must mirror the cleaned hFRIK V5 contract.'
Reject-ExternalText 'hFRIK/src/FRIK.cpp' `
    'logSkeletonInitializationBlocked|Dispatched kSkeleton|Loading menu is open, defer skeleton initialization|initialization delayed after release' `
    'hFRIK must keep skeleton readiness and lifecycle behavior free of added log-only scaffolding.'
Reject-ExternalText 'hFRIK/src/PlayerControlsHandler.h' `
    'Player controls - Reset restored' `
    'hFRIK control-state restoration must not retain its added success-only log.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

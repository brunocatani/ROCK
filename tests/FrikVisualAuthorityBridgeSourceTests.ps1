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
    'mirrorFingerLocalTransforms\(Hand sourceHand[\s\S]*frikApi->mirrorFingerLocalTransforms\(sourceHand,\s*&sourceTransforms,\s*&outTargetTransforms\)' `
    'ROCK must use hFRIK API V2''s bidirectional anatomical mirror table entry.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'blockPrimaryHandWeaponPose\(const char\* tag,\s*bool block\)[\s\S]*api\(\)[\s\S]*frikApi->blockPrimaryHandWeaponPose\(tag,\s*block\)' `
    'ROCK must use the API V2 function table for primary weapon-pose blocking.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'canBlockPrimaryHandWeaponPose\(\)[\s\S]*frikApi->blockPrimaryHandWeaponPose\s*!=\s*nullptr' `
    'ROCK must feature-detect primary weapon-pose blocking through the API V2 table.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'PresentedHandNodeCache[\s\S]*getHandWorldTransform\(Hand hand\)[\s\S]*isSkeletonReadyHint\(\)[\s\S]*getFirstPersonSkeleton\(\)[\s\S]*findNode\(skeleton,\s*"RArm_Hand"\)[\s\S]*findNode\(skeleton,\s*"LArm_Hand"\)[\s\S]*handNode->world' `
    'ROCK must read final presented hands directly from the game first-person scene nodes.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'g_trackedHandWorldPublications[\s\S]*rememberTrackedHandWorldPublication\(tag,\s*hand\)[\s\S]*invalidateTrackedHandWorldPublication\(tag,\s*hand\)[\s\S]*resetTrackedHandWorldPublications\(\)' `
    'Persistent FRIK V2 hand-world publications must be tracked per tag and cleared at lifecycle reset.'
Require-Text 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'g_handWorldPublicationReady[\s\S]*applyExternalHandWorldTransform[\s\S]*!detail::g_handWorldPublicationReady\[handIndex\][\s\S]*setExternalHandWorldPublicationReady' `
    'Persistent hand publication must fail closed until controller reconstruction is calibrated.'
Require-Text 'src/ROCKMain.cpp' `
    'kSkeletonReady[\s\S]*resetPresentedHandNodeCache\(\)[\s\S]*kSkeletonDestroying[\s\S]*resetPresentedHandNodeCache\(\)' `
    'Game hand-node caches must be invalidated at both hFRIK skeleton lifecycle edges.'

Require-ExternalText 'hFRIK/src/skeleton/HandPoseMath.cpp' `
    'mirrorFingerLocalTransforms[\s\S]*sourceIsLeft[\s\S]*mirrorBoneToOppositeHand[\s\S]*animatedSource\.rotate[\s\S]*FULL_LOCAL_TRANSFORM_MASK' `
    'hFRIK must mirror harvested finger locals in either physical direction through the same anatomy-aware path.'
Require-ExternalText 'hFRIK/src/skeleton/HandPose.cpp' `
    'g_externalAuthority\.isPrimaryWeaponNodeOwnershipBlocked\(\)[\s\S]*!g_externalAuthority\.isPrimaryWeaponPoseBlocked\(\)' `
    'A blocked native primary pose must allow ROCK''s exact explicit pose to win during physical-left firing carry.'
Require-ExternalText 'hFRIK/src/api/FRIKApiV2.cpp' `
    'mirrorFingerLocalTransforms[\s\S]*sourceHand\s*!=\s*FRIKApiV2::Hand::Left[\s\S]*sourceHand\s*!=\s*FRIKApiV2::Hand::Right[\s\S]*core::mirrorFingerLocalTransforms' `
    'hFRIK API V2 must expose the bidirectional anatomy-aware mirror table entry.'
Reject-ExternalText 'hFRIK/src/api/FRIKApiV2.cpp' `
    'FRIKAPI_BlockPrimaryHandWeaponPose|FRIKAPI_MirrorPrimaryWeaponFingerLocalTransforms' `
    'hFRIK must not retain duplicate direct-export paths for API V2 table behavior or legacy mirroring.'
Reject-ExternalText 'hFRIK/src/api/FRIKApiV2.h' `
    'getHandWorldTransform' `
    'hFRIK API V2 must not wrap game-owned first-person hand transforms.'
Reject-ExternalText 'ROCK/src/api/FRIKApiV2.h' `
    'getHandWorldTransform' `
    'ROCK must not retain the removed hFRIK hand-transform table entry.'
Reject-ExternalText 'hFRIK/src/api/FRIKApiV2.h' `
    'using\s+HandPoses|setHandPoseCustomFingerPositionsWithPriority|kPowerArmorChanged' `
    'hFRIK API V2 must not retain unused aliases, convenience calls, or lifecycle events.'
Reject-ExternalText 'ROCK/src/api/FRIKApiV2.h' `
    'using\s+HandPoses|setHandPoseCustomFingerPositionsWithPriority|kPowerArmorChanged' `
    'ROCK must mirror the cleaned hFRIK API V2 contract.'
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

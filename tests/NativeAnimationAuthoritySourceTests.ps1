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
    'beginRockFrame\([\s\S]{0,120}deltaSeconds[\s\S]*applyCapturedPose\([\s\S]{0,120}ApplyPhase::BeforeRock[\s\S]*onFrameUpdate\(\)[\s\S]*applyCapturedPose\([\s\S]{0,120}ApplyPhase::AfterRock[\s\S]*completeRockFrame\(\)' `
    'Native authority must expose explicit pre/post ROCK phases so hand-only IK cannot contaminate the controller-owned weapon solve.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'equalsIgnoreCase\(name,\s*"Weapon"\)[\s\S]*LArm_[\s\S]*RArm_' `
    'Bone authority must be an explicit arms/hands/two-weapon-root allowlist.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'desiredAnchor\s*=\s*liveControl\s*\*\s*authoredDelta[\s\S]*correction\s*=\s*desiredAnchor\s*\*\s*inverse\(authoredCurrent\)[\s\S]*resolveControllerAnchoredPoseCorrection' `
    'Native reload motion must preserve one rigid authored pose inside the live controller aim frame.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'classifyBone[\s\S]*return\s+kReloadPose' `
    'The classifier must never broadly grant every pose flag to arbitrary body bones.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' `
    'kFunc_ReloadStateChangeHandler_Handle\s*=\s*0x0FF2B90[\s\S]*kFunc_GetReloadStartStateToken\s*=\s*0x16A3070[\s\S]*kFunc_GetReloadEndStateToken\s*=\s*0x16A30D0[\s\S]*kVtableEntry_ReloadStateChangeHandler_Handle\s*=\s*0x2D8D300' `
    'Reload lifecycle authority must stay pinned to the independently verified FO4VR handler, tokens, and vtable slot.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' `
    'kFunc_WeaponFireHandler_Handle\s*=\s*0x0FF2A40[\s\S]*kVtableEntry_WeaponFireHandler_Handle\s*=\s*0x2D8D2E8' `
    'Manual-cycle authority must stay pinned to the independently verified FO4VR WeaponFire handler and vtable slot.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onReloadStateChange[\s\S]*nativeReloadStartStateToken[\s\S]*s_playerReloadStartSequence\.fetch_add[\s\S]*nativeReloadEndStateToken[\s\S]*s_playerReloadEndSequence\.fetch_add' `
    'The lifecycle hook must classify the verified Bethesda start/end tokens and publish player-only event sequences.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'advanceLocalReloadLease[\s\S]*installReloadStateChangeHook[\s\S]*expectedTarget[\s\S]*kFunc_ReloadStateChangeHandler_Handle[\s\S]*VirtualProtect' `
    'The local ROCK test lease must consume event sequences from a validated ReloadStateChangeHandler vtable hook.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onWeaponFire[\s\S]*actor\s*!=\s*player[\s\S]*currentPlayerWeaponInstanceData[\s\S]*WEAPON_FLAGS::kBoltAction[\s\S]*s_localManualCycleTestRequestSequence\.fetch_add' `
    'The hand-only cycle must arm only from a handled player WeaponFire event and the live instance-aware Bethesda bolt-action flag.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onWeaponFire[\s\S]*reloadEndSequenceBeforeFire[\s\S]*s_originalWeaponFire[\s\S]*s_localManualCycleReloadEndSequenceAtArm\.store\([\s\S]*reloadEndSequenceBeforeFire' `
    'The manual-cycle bracket baseline must be sampled before Bethesda can synchronously publish the clip frame-zero marker.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'advanceLocalManualCycleLease[\s\S]*installWeaponFireHook[\s\S]*kVtableEntry_WeaponFireHandler_Handle[\s\S]*kFunc_WeaponFireHandler_Handle[\s\S]*VirtualProtect' `
    'The manual-cycle lease must consume the native ReloadEnd bracket after a validated WeaponFire vtable hook.'
Require-Text 'src/ROCKMain.cpp' `
    'setLocalManualCycleTestEnabled\([\s\S]{0,180}rockNativeReloadAnimationAuthorityTestEnabled' `
    'The manual-cycle path must remain gated by the existing native reload authority experiment flag.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'hasNativeManualCycleTwoHandAuthority[\s\S]*TwoHandedState::Gripping[\s\S]*isFiringHandLeft\(\)[\s\S]*ownsWeaponTransform\(\)' `
    'Native cycle hand animation must require a right-primary full two-hand weapon solver.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onWeaponFire[\s\S]*s_manualCycleTwoHandAuthorityActive\.load[\s\S]*WEAPON_FLAGS::kBoltAction[\s\S]*s_localManualCycleTestLeaseActive\.store\(true' `
    'One-hand firing must retain FO4VR parts-only behavior by never arming the local hand-animation lease.'
Require-Text 'src/ROCKMain.cpp' `
    's_originalGameLoopFunc\(rcx\)[\s\S]*refreshNativeManualCycleTwoHandAuthority\(\)[\s\S]*beginRockFrame[\s\S]*onFrameUpdate\(\)[\s\S]*refreshNativeManualCycleTwoHandAuthority\(\)[\s\S]*ApplyPhase::AfterRock' `
    'Two-hand eligibility must be sampled before lease consumption and again after ROCK grip transitions before final pose publication.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'gunState\s*==\s*RE::GUN_STATE::kReloading' `
    'FO4VR does not publish this VR reload path through ActorState::gunState; lifecycle code must not regress to that poll.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'weaponInControlParent\s*=\s*weaponNode->local[\s\S]*composeTransforms\(\s*aimFrame\.controlParent->world,\s*aimFrame\.weaponInControlParent\)[\s\S]*applyControllerAimFrame[\s\S]*nativeBaselineWeaponWorld' `
    'The visible first-person weapon must derive a non-accumulating controller frame for the complete authored pose.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'auto&\s+aimFrame\s*=\s*s_sourceAimFrame[\s\S]*resolveWorldTargetCorrection\(\s*aimFrame\.desiredWeaponWorld,\s*nativeWeaponWorld\)' `
    'The full-body arms must resolve to the visible weapon world target instead of a hidden destination Weapon target.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    's_destinationAimFrame|prepareControllerAimFrame\(\s*\*s_cache\.destinationTree' `
    'The hidden full-body Weapon node must never establish an independent controller anchor.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'parentIsSelected\(transform\.parPos,[\s\S]*composeTransforms\(correction,\s*nativeRootWorld\)' `
    'The controller aim correction must be applied once at selected hierarchy roots, not independently per bone.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'kManualCyclePose\s*=\s*kArms\s*\|\s*kHands[\s\S]*advanceLocalManualCycleLease[\s\S]*observedReloadEndEvents\s*>=\s*2' `
    'Manual-cycle authority must exclude Weapon and end on the native bolt/lever clip bracket.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'captureManualCyclePose[\s\S]*animatedWeaponLocal\s*=\s*authoritativeLocal\(weaponTransform\)[\s\S]*nativeWeaponModel[\s\S]*resolveNativeHandInWeapon[\s\S]*primaryHandInWeapon[\s\S]*supportHandInWeapon' `
    'Hand-only cycling must derive both physical hands from the live animated Weapon local in one native graph frame.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'ManualCycleHandRebase[\s\S]*manualCycleHandRebases[\s\S]*getHandWorldTransform\(hand\)[\s\S]*liveBaselineHandInWeapon[\s\S]*nativeBaselineHandInWeapon[\s\S]*resolveControllerAnchoredPoseCorrection[\s\S]*rebasedHandInWeapon' `
    'Each manual-cycle hand must rebase only the native animation delta onto its live ROCK grip instead of publishing Bethesda''s absolute flat-game basis.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'applyCapturedPose\(const ApplyPhase phase\)[\s\S]*ApplyPhase::BeforeRock[\s\S]*return true;[\s\S]*applyManualCyclePoseAfterRock' `
    'Manual-cycle IK must be a no-op before ROCK and publish only in the explicit final phase.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'publishManualCycleHandVisual[\s\S]*applyExternalHandWorldTransform[\s\S]*kManualCycleVisualAuthorityPriority[\s\S]*applyManualCyclePoseAfterRock[\s\S]*restoreFixedVisibleWeaponTarget' `
    'Manual-cycle IK must outrank grip-locked visual authority and restore the exact controller-fixed Weapon world.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'kManualCyclePrimaryFingerBoneNames[\s\S]*captureManualCycleFingerLocals[\s\S]*publishManualCycleHandVisual[\s\S]*setHandPoseCustomWithPriority[\s\S]*setHandPoseCustomLocalTransformsWithPriority[\s\S]*clearManualCycleVisualAuthority' `
    'Manual-cycle authority must establish the required base pose, attach native finger locals, and deterministically release both hand-pose and world-transform tags.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'clearManualCycleVisualAuthorityPreservingWeapon[\s\S]*beginRockFrame[\s\S]*s_frameManualCycleCleanupPending\s*=\s*manualCycleVisualAuthorityPublished\(\)[\s\S]*Keep ROCK''s higher-priority cycle tags selected[\s\S]*completeRockFrame[\s\S]*s_frameManualCycleCleanupPending[\s\S]*clearManualCycleVisualAuthorityPreservingWeapon\(\)' `
    'Active cycling and a parts-only lease edge must retain the selected overlay until ROCK refreshes the hidden controller grip targets.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'if\s*\(manualCycleRequested\)\s*\{[^}]*clearManualCycleVisualAuthority' `
    'Active cycle setup must not reselect stale previous-frame grip targets before ROCK solves the weapon.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'applyCapturedPose[\s\S]*manualCycleStillRequested[\s\S]*clearManualCycleVisualAuthorityPreservingWeapon\(\)[\s\S]*s_frameManualCycleApplied\s*=\s*true' `
    'Support release during ROCK update must cancel the hand overlay after the current weapon solve without disturbing the weapon world.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'resolveWorldTargetCorrection\(\s*aimFrame\.controlWeaponWorld,\s*nativeWeaponWorld\)' `
    'Hand-only cycling must never return to a rigid collarbone-root correction against the Weapon.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'nativeReloadAuthorityActive\s*=[\s\S]{0,180}effectiveFlags\s*&[\s\S]{0,120}kWeapon' `
    'Arms/hands-only cycling must not suspend ROCK authored weapon alignment.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'ROCK_PROVIDER_API_VERSION\s*=\s*1[\s\S]*NativeAnimationAuthority[\s\S]*setNativeAnimationAuthorityV1[\s\S]*clearNativeAnimationAuthorityV1' `
    'The authority lease must append to API V1 without a version bump.'
Require-Text 'src/api/ROCKProviderApi.cpp' `
    'clearNativeAnimationAuthorityForOwnerLocked\(ownerToken\)' `
    'Consumer unregister must deterministically release native animation authority.'
Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'rockNativeReloadAnimationAuthorityTestEnabled[\s\S]*requestLocalReloadTestLease\(\)' `
    'The ROCK-only validation flag must arm a bounded lease only when a native reload dispatch succeeds.'

Require-Text 'src/physics-interaction/native/HavokOffsets.h' `
    'kFunc_UpdateFirstPersonArm\s*=\s*0xEF6280[\s\S]*kCallsite_UpdateFirstPersonArmPrimaryReturn\s*=\s*0xEF610D[\s\S]*kCallsite_UpdateFirstPersonArmSecondaryReturn\s*=\s*0xEF6150' `
    'The firing-grip probe must stay pinned to the independently verified FO4VR helper and paired Bethesda arm-call returns.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'kExpectedUpdateFirstPersonArmPrefix[\s\S]*0x48,\s*0x8B,\s*0xC4,\s*0x55,\s*0x53,\s*0x41,\s*0x56,[\s\S]*0x48,\s*0x8D,\s*0xA8,\s*0xF8,\s*0xFE,\s*0xFF,\s*0xFF[\s\S]*kFunc_UpdateFirstPersonArm' `
    'The native arm interception must validate the complete position-independent FO4VR prologue before patching.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onUpdateFirstPersonArm[\s\S]*_ReturnAddress\(\)[\s\S]*s_originalUpdateFirstPersonArm\([\s\S]*primaryPass\s*=\s*returnAddress\s*==\s*s_nativePrimaryArmReturnAddress[\s\S]*supportPass\s*=\s*returnAddress\s*==\s*s_nativeSupportArmReturnAddress[\s\S]*primaryPass\s*\?[\s\S]*captureNativePrimaryFiringGrip\(\)[\s\S]*captureNativeAuthoredSupportGrip\(\)' `
    'The hook must call the native helper first and capture only Bethesda''s verified paired arm passes, never hFRIK''s later calls.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'captureNativePrimaryFiringGrip[\s\S]*weaponTransform\.parPos\s*!=\s*handIndex[\s\S]*invertTransform\(weaponTransform\.refNode->world\)[\s\S]*handTransform\.refNode->world[\s\S]*s_authoredPrimaryHandInWeapon\s*=\s*handInWeapon' `
    'The authored grip must be captured as Bethesda''s pre-hFRIK primary hand in the visible Weapon world frame.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'tryResolvePrimaryFiringGripAlignment[\s\S]*expectedWeaponNode->parent\s*!=\s*capturedHandNode[\s\S]*outAuthoredPrimaryHandInWeapon\s*=\s*s_authoredPrimaryHandInWeapon[\s\S]*resolveAuthoredPrimaryWeaponWorld\([\s\S]*trackedPrimaryHandWorld[\s\S]*outAuthoredPrimaryHandInWeapon' `
    'The experiment must invert the captured native relation onto the tracked primary hand, return that exact Weapon-relative canonical, and reject a changed hand/weapon hierarchy.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    '"LArm_Finger11"[\s\S]*"LArm_Finger53"[\s\S]*composeAuthoredLogicalModelTransform[\s\S]*parPos[\s\S]*authoritativeLocal[\s\S]*captureAuthoredSupportGraphPose[\s\S]*resolveAuthoredSupportHandInPrimaryHand[\s\S]*authoritativeLocal\(source->transforms\[transformIndex\]\)[\s\S]*s_authoredSupportGraphPoseSequence\.fetch_add' `
    'The post-animation capture must reconstruct both authored hands through the flattened local hierarchy and retain all 15 authoritative finger locals.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onPostUpdateAnimationGraphManager[\s\S]*captureAuthoredSupportGraphPose\(\)[\s\S]*captureNativePose\(\)[\s\S]*s_originalPostUpdate\(holder\)' `
    'The authored hand relation must be captured at the proven graph-output boundary before the later native presentation writers.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'captureNativeAuthoredSupportGrip[\s\S]*graphPoseSequence\s*==\s*0[\s\S]*graphPoseSequence\s*!=\s*s_primaryFiringGripGraphPoseSequence[\s\S]*s_lastConsumedAuthoredSupportGraphPoseSequence[\s\S]*resolveAuthoredSupportHandInWeapon[\s\S]*s_authoredSupportHandInPrimaryHand[\s\S]*s_authoredSupportGraphFingerLocals' `
    'The paired support pass must consume exactly the graph pose matched by the immediately preceding primary capture.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'resolveAuthoredSupportHandInPrimaryHand[\s\S]*invert\(authoredPrimaryHandModel\)[\s\S]*authoredSupportHandModel[\s\S]*resolveAuthoredSupportHandInWeapon[\s\S]*compose\(primaryHandInWeapon,\s*supportHandInPrimaryHand\)' `
    'The support target must combine the same-tree authored hand relation with the validated per-weapon primary hand anchor.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'tryResolveAuthoredSupportGrip[\s\S]*capturedPrimaryHandNode[\s\S]*expectedWeaponNode\s*!=\s*s_authoredSupportGripWeaponNode\.load[\s\S]*expectedWeaponNode->parent\s*!=\s*capturedPrimaryHandNode[\s\S]*outSupportHandInWeapon\s*=\s*s_authoredSupportHandInWeapon[\s\S]*outFingerLocalTransforms\s*=\s*s_authoredSupportFingerLocals[\s\S]*kAuthoredSupportFingerTransformMask' `
    'The live support resolver must reject ROCK-reparented scene topology and publish only a complete right-primary hand/finger frame.'

$nativeAuthorityText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/animation/NativeAnimationAuthority.cpp')
$primaryCaptureMatch = [regex]::Match(
    $nativeAuthorityText,
    '(?s)captureNativePrimaryFiringGrip\(\).*?(?=\s+__declspec\(noinline\))')
if (-not $primaryCaptureMatch.Success) {
    $failures.Add('The native primary firing-grip capture function could not be isolated for source validation.')
} elseif ($primaryCaptureMatch.Value -match 'refNode->local|weaponTransform\.local|authoritativeLocal\s*\(') {
    $failures.Add('The authored primary grip must never capture hFRIK''s downstream Weapon local/offset path.')
}

$supportGraphCaptureMatch = [regex]::Match(
    $nativeAuthorityText,
    '(?s)captureAuthoredSupportGraphPose\(\).*?(?=\s+\[\[nodiscard\]\]\s+bool\s+captureNativeAuthoredSupportGrip\(\))')
if (-not $supportGraphCaptureMatch.Success) {
    $failures.Add('The authored support graph-pose capture function could not be isolated for source validation.')
} elseif ($supportGraphCaptureMatch.Value -match '!fingerTransform\.refNode|fingerTransform\.refNode->local') {
    $failures.Add('The authored support grip must accept authoritative flattened finger locals when a scene refNode is absent.')
} elseif ($supportGraphCaptureMatch.Value -match '(?:support|primary)HandTransform\.(?:refNode->)?world|\.transforms\[[^\]]+\]\.world') {
    $failures.Add('The authored support graph relation must never be derived from live/presentation world transforms.')
}

$supportPairCaptureMatch = [regex]::Match(
    $nativeAuthorityText,
    '(?s)captureNativeAuthoredSupportGrip\(\).*?(?=\s+\[\[nodiscard\]\]\s+bool\s+captureNativePrimaryFiringGrip\(\))')
if (-not $supportPairCaptureMatch.Success) {
    $failures.Add('The paired authored support-grip capture function could not be isolated for source validation.')
} elseif ($supportPairCaptureMatch.Value -match 'supportHandTransform\.(?:refNode->)?world|weaponTransform\.(?:refNode->)?world') {
    $failures.Add('The authored support target must never be derived from live/presentation world transforms.')
}

Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'supportCaptureStatus\.valid[\s\S]*authoredSupportGripCaptureFailureReasonName[\s\S]*secondaryPassSequence[\s\S]*invalidOrMissingFingerMask' `
    'A failed authored support capture must publish rate-limited stage and finger-mask telemetry for runtime diagnosis.'

Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'getHandWorldTransform\([\s\S]*Hand::Primary[\s\S]*tryResolvePrimaryFiringGripAlignment[\s\S]*applyAuthoredPrimaryGripWeaponAlignment' `
    'The experiment must preserve the controller-driven primary hand and apply the inverse solve through the shared weapon visual path.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'tryResolvePrimaryFiringGripAlignment[\s\S]*authoredPrimaryHandInWeapon[\s\S]*applyAuthoredPrimaryGripWeaponAlignment[\s\S]*setAuthoredPrimaryFiringGripCanonical\([\s\S]*authoredPrimaryHandInWeapon[\s\S]*input\.weaponGenerationKey[\s\S]*currentWeaponKey[\s\S]*resolvedCaptureSequence' `
    'A successful right-hand alignment must explicitly bind the exact authored Hand-in-Weapon relation to generation, equipped ownership, and capture sequence for physical-left mirroring.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'findPrimaryWeaponOffset\([\s\S]{0,500}OffsetSource::CustomFile[\s\S]{0,600}custom-frik-weapon-offset[\s\S]*applyAuthoredPrimaryGripWeaponAlignment' `
    'A custom hFRIK weapon JSON must be resolved at the equip boundary and suspend ROCK authored alignment before any weapon transform write.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'currentRevision\(\)[\s\S]{0,500}frikOffsetCacheRevision\s*!=\s*_frikOffsetCacheRevision[\s\S]{0,800}OffsetSource::CustomFile[\s\S]{0,800}custom-frik-weapon-offset-change[\s\S]{0,300}captureSequenceFloor' `
    'A live custom-file add/remove must re-evaluate the equipped weapon from the in-memory revision, release stale authority, and require a fresh graph capture.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    '!harvestedRelationAvailable\s*&&\s*!authored_weapon_grip_library::publish\([\s\S]{0,350}CaptureSource::LiveEquippedGraph' `
    'Live equipped capture must remain only the bounded compatibility fallback when no native-idle harvest exists.'
Require-Text 'src/physics-interaction/weapon/EquipVisualBridge.cpp' `
    'customFrikOffsetPresent\s*=[\s\S]{0,300}OffsetSource::CustomFile[\s\S]{0,700}tryResolveLooseWeaponFiringHandHoldForModel[\s\S]{0,700}else if\s*\(\s*!customFrikOffsetPresent' `
    'Loose-to-equipped visual handoff must re-resolve the filewatch-published shared authority and never fall back across a custom override.'
Require-Text 'src/physics-interaction/grab/FrikWeaponOffsetCache.cpp' `
    'FileWatch<std::string>[\s\S]*reloadCache\(\)[\s\S]*FileWatch invokes callbacks on its own worker' `
    'Custom hFRIK offset changes must reload on a filewatch worker, never through frame/input/animation filesystem polling.'
Reject-Text 'src/physics-interaction/grab/FrikWeaponOffsetCache.cpp' `
    'findPrimaryWeaponOffset\([\s\S]{0,1800}directory_iterator' `
    'Runtime offset lookup must not perform directory I/O.'
Require-Text 'src/physics-interaction/grab/FrikWeaponOffsetCache.cpp' `
    'loadCustomOffsets[\s\S]{0,1400}OffsetSource::CustomFile[\s\S]*findPrimaryWeaponOffsetLocked[\s\S]{0,1800}\.source\s*=\s*offset->source' `
    'The hFRIK cache must retain per-entry custom-vs-embedded provenance through final lookup.'
Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripLibrary.cpp' `
    'std::array<Entry,\s*kCapacity>[\s\S]*weaponFormId[\s\S]*variantKey[\s\S]*inPowerArmor[\s\S]*nativeIdleMatchCount[\s\S]*selectLookup' `
    'The learned loose grip library must be bounded and keyed by weapon, stock variant, and power-armor topology while preferring only an unambiguous native-idle fallback.'
Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripLibrary.cpp' `
    'shouldAcceptPublication\([\s\S]*CaptureSource::NativeIdlePreharvest[\s\S]*rightFiringFingerPose' `
    'A native-idle relation and its complete finger pose must remain authoritative over later live fallback frames.'
Reject-Text 'src/physics-interaction/weapon/AuthoredWeaponGripLibrary.cpp' `
    'PreharvestBaseline|observeLiveEquivalence|Authored grip equivalence' `
    'The completed preharvest path must not retain the obsolete live proving/equivalence machinery.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'harvestedRelationAvailable[\s\S]*buildMirroredLeftFingerPose[\s\S]*authoredPrimaryHandInWeapon\s*=\s*authoredLookup\.rightHandWeaponLocal[\s\S]*setAuthoredPrimaryFiringGripCanonical\([\s\S]*rightFingerPose[\s\S]*leftFingerPose[\s\S]*publishAuthoredPrimaryFiringGripFingerPose\(false\)' `
    'Equipped primary alignment must consume harvested relation and complete right/left finger poses without waiting for a live proof sample.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'input\.rockFiringHandIsLeft[\s\S]*setAuthoredPrimaryFiringGripCanonical\([\s\S]*authoredLookup\.rightHandWeaponLocal[\s\S]*publishAuthoredPrimaryFiringGripFingerPose\(true\)[\s\S]*physical-left-firing-canonical-only' `
    'Physical-left firing must bind and publish the mirrored harvested canonical without entering the right-controller weapon alignment solve.'
Require-Text 'src/physics-interaction/weapon/EquipVisualBridge.cpp' `
    'kHandPoseHandoffPriority\s*=\s*99[\s\S]*CaptureSource::NativeIdlePreharvest[\s\S]*publishHandPoseHandoff[\s\S]*initial-hand-transform-publish-failed[\s\S]*blockPrimaryHandWeaponPose[\s\S]*setHandPoseCustomLocalTransformsWithPriority[\s\S]*applyExternalHandWorldTransform' `
    'Loose-to-equipped transition must retain the exact pose and hand frame below equipped priority until positive authority acquisition.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '\.weaponFormID\s*=\s*equipResult\.weapon\s*\?\s*equipResult\.weapon->formID\s*:\s*equipResult\.observedEquippedFormID' `
    'The equip bridge must match the equipped instance by base weapon form, never by the temporary loose reference ID.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'hasPublishedAuthoredPrimaryFiringGripFingerPose[\s\S]*completeHandPoseHandoff\("equipped-authored-pose-acquired"\)' `
    'The transition pose must be released only after the equipped exact-pose tag is positively active.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'grabOffsetFingerPoseSource\.valid[\s\S]*applyRockGrabHandPose[\s\S]*else\s*\{[\s\S]*publishLooseWeaponPrimaryAttachHandPose' `
    'Synthetic loose-weapon attach must preserve explicit saved finger authority before consulting the authored/default pose resolver.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'publishLooseWeaponPrimaryAttachHandPose[\s\S]{0,2600}rightFiringFingerPose\.complete\(\)[\s\S]{0,1600}mirrorPrimaryWeaponFingerLocalTransforms[\s\S]{0,1800}setHandPoseCustomLocalTransformsWithPriority[\s\S]{0,1600}setHandPoseWithPriority' `
    'The authored/default resolver must publish only a complete harvested right/mirrored-left pose and retain the generic named-pose fallback.'
Require-Text 'src/physics-interaction/hand/Hand.cpp' `
    'clearGrabHandPose[\s\S]*clearHandPose\(GRAB_HAND_POSE_TAG[\s\S]*blockPrimaryHandWeaponPose\([\s\S]*false' `
    'Grab cleanup must release both the exact pose tag and its native-primary-pose blocker.'
Require-Text 'src/physics-interaction/weapon/WeaponGripAuthorityPolicy.h' `
    'frikCustomFile[\s\S]{0,300}Source::FrikCustomFile[\s\S]{0,300}authoredAnimation[\s\S]{0,300}Source::AuthoredAnimation[\s\S]{0,300}frikEmbeddedResource[\s\S]{0,300}Source::FrikEmbeddedResource' `
    'Weapon grip authority must encode custom hFRIK JSON above ROCK authored animation and embedded data below it.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'input\.leftHandedMode[\s\S]*clearAuthoredPrimaryFiringGripCanonical\([\s\S]*game-left-handed-mode[\s\S]*rockFiringHandIsLeft\s*=\s*input\.rockFiringHandIsLeft' `
    'Global game-left topology must discard the right-authored canonical while ROCK physical-left firing remains an explicit eligibility state.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'AuthoredPrimaryFiringGripRuntime::reset[\s\S]*clearAuthoredPrimaryFiringGripCanonical\(reason\)[\s\S]*weapon-boundary[\s\S]*clearAuthoredPrimaryFiringGripCanonical' `
    'Feature/lifecycle reset and weapon identity changes must deterministically release the authored canonical.'
Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'applyExternalHandWorldTransform|clearExternalHandWorldTransform|Hand::(?:Offhand|Left|Right)|weaponNode->(?:local|world)\s*=|blockPrimaryWeaponNodeOwnership' `
    'The primary-only alignment must not move either hand, write the node outside the shared weapon path, or enter hFRIK''s left-carry topology.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'input\.nativeReloadAuthorityActive[\s\S]*endSession\("native-reload-authority"\)[\s\S]*captureSequenceFloor' `
    'Native reload authority must suspend the alignment and require a fresh capture before restoring it.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'clearAuthoredSupportGripCandidate\(\)[\s\S]*publishLiveAuthoredSupportCandidate[\s\S]*tryResolveAuthoredSupportGrip[\s\S]*setAuthoredSupportGripCandidate[\s\S]*_stableAuthoredSupportGrip\s*=\s*StableAuthoredSupportGripSnapshot[\s\S]*weaponNodeIdentity[\s\S]*weaponOwnershipKey[\s\S]*weaponGenerationKey[\s\S]*primaryGripCaptureSequence[\s\S]*supportCaptureSequence' `
    'The frame-ephemeral candidate must snapshot a topology-valid native support relation under every equipped/canonical identity key.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'publishStableAuthoredSupportCandidate[\s\S]*stable\.weaponNodeIdentity\s*!=\s*input\.weaponNode[\s\S]*stable\.weaponOwnershipKey\s*!=\s*currentWeaponKey[\s\S]*stable\.weaponGenerationKey\s*!=\s*input\.weaponGenerationKey[\s\S]*stable\.primaryGripCaptureSequence\s*!=\s*primaryGripCaptureSequence[\s\S]*setAuthoredSupportGripCandidate' `
    'Physical-left support publication must reject a stable snapshot unless weapon identity, ownership, generation, and authored canonical all still match.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'applyAuthoredPrimaryGripWeaponAlignment[\s\S]*publishLiveAuthoredSupportCandidate\(resolvedCaptureSequence\)' `
    'Right-primary alignment must refresh the topology-valid support snapshot after establishing the final weapon basis.'

$primaryRuntimeText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp')
$physicalLeftSupportMatch = [regex]::Match(
    $primaryRuntimeText,
    '(?s)if\s*\(input\.rockFiringHandIsLeft\)\s*\{.*?(?=\s+const\s+native_animation_authority_policy::AuthoredPrimaryFiringGripEligibility)')
if (-not $physicalLeftSupportMatch.Success) {
    $failures.Add('The physical-left firing branch could not be isolated for support-source validation.')
} elseif ($physicalLeftSupportMatch.Value -notmatch 'publishStableAuthoredSupportCandidate\(\s*authoredLookup\.captureSequence\s*\)') {
    $failures.Add('Physical-left firing must republish the last topology-valid support snapshot for right-hand mirroring.')
} elseif ($physicalLeftSupportMatch.Value -match 'publishLiveAuthoredSupportCandidate|tryResolveAuthoredSupportGrip') {
    $failures.Add('Physical-left firing must never consume a fresh native support capture after ROCK has reparented Weapon.')
}
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'blocksAuthoredPrimaryGripWeaponAlignment[\s\S]{0,700}_firingHandIsLeft[\s\S]{0,180}_weaponNodeOwnershipBlockEngaged[\s\S]{0,180}ownsWeaponTransform\(\)' `
    'Authored alignment must distinguish conflicting weapon-transform ownership from right-primary bookkeeping ownership.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyAuthoredPrimaryGripWeaponAlignment[\s\S]*blocksAuthoredPrimaryGripWeaponAlignment\(\)[\s\S]*isWeaponVisualReturnActive\(\)[\s\S]*applyWeaponVisualAuthority' `
    'Authored alignment must reuse the scope-aware weapon visual path and yield to conflicting transform/return authority.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'setAuthoredPrimaryFiringGripCanonical[\s\S]*weaponOwnershipKey\s*==\s*0[\s\S]*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*rightHandWeaponLocal[\s\S]*false[\s\S]*_rightFiringHandCanonicalWeaponLocal\s*=\s*rightHandWeaponLocal[\s\S]*_rightFiringHandCanonicalOwnershipKey\s*=\s*weaponOwnershipKey[\s\S]*RightFiringCanonicalSource::AuthoredAnimation' `
    'The authored canonical must derive its grip seat directly in Weapon space from the captured right-hand relation and retain explicit ownership/source authority.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'hasRightFiringHandCanonicalFrame[\s\S]*_rightFiringHandCanonicalWeaponNode\s*==\s*weaponNode[\s\S]*_rightFiringHandCanonicalGenerationKey\s*==\s*weaponGenerationKey[\s\S]*_rightFiringHandCanonicalOwnershipKey\s*==\s*weaponOwnershipKey' `
    'Canonical consumption must reject the one-frame equip race where node and collision generation are unchanged but equipped ownership has advanced.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshNaturalHandInWandFrames[\s\S]*refreshHand\(false,[\s\S]*_rightNaturalBoneInWand[\s\S]*refreshHand\(true,[\s\S]*_leftNaturalBoneInWand[\s\S]*refreshRightNativeCanonicalFrame[\s\S]*hasRightFiringHandCanonicalFrame[\s\S]*RightFiringCanonicalSource::AuthoredAnimation[\s\S]*return;[\s\S]*RightFiringCanonicalSource::NativeCarry' `
    'Native presentation refresh must retain both physical hand-to-wand anatomy frames without overwriting a matching authored canonical.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryComputeMirroredLeftFiringHandWeaponLocal[\s\S]*hasRightFiringHandCanonicalFrame[\s\S]*tryBuildMirroredLeftFiringHandWeaponLocal\([\s\S]*_rightFiringHandCanonicalWeaponLocal,[\s\S]*_rightFiringGripCanonicalWeaponLocal[\s\S]*authored-animation' `
    'Physical-left firing must consume the generation- and identity-matched canonical plus its authored Weapon-relative palm seat through the existing wand mirror.'

$leftMirrorConsumerMatch = [regex]::Match(
    (Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp')),
    '(?s)bool\s+TwoHandedGrip::tryComputeMirroredLeftFiringHandWeaponLocal\(.*?(?=\s+bool\s+TwoHandedGrip::tryBuildMirroredLeftFiringHandWeaponLocal\()')
if (-not $leftMirrorConsumerMatch.Success) {
    $failures.Add('The left-firing canonical consumer could not be isolated for source validation.')
} elseif ($leftMirrorConsumerMatch.Value -match '_primaryGripLocal') {
    $failures.Add('The authored left mirror must never fall back to the live/session _primaryGripLocal when a canonical frame is selected.')
}
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'capturePartGrip[\s\S]*resolveAuthoredSupportPalmSeatProximity[\s\S]*authoredSupportPalmWeaponLocal[\s\S]*authoredSeatToFiringGripLocal[\s\S]*_primaryGripLocal[\s\S]*resolveFiringGripProximityAuthorityMode[\s\S]*authoredSeatTouchAcquisition[\s\S]*WeaponInteractionAcquisitionSource::PhysicalContact[\s\S]*authoredSupportTouchProbeDistance\s*<=[\s\S]*rockWeaponInteractionTouchRadius[\s\S]*shouldUseAuthoredSupportGrip[\s\S]*authoredSeatTouchAcquisition\s*=\s*authoredSeatTouchAcquisition[\s\S]*providerAuthorityActive\s*=\s*providerPartAuthority\.active[\s\S]*attachOnly\s*=\s*grip\.attachOnly[\s\S]*_authorityMode\s*=\s*authoredSupportAuthorityMode[\s\S]*grip\.handWeaponLocal\s*=[\s\S]*fingerLocalTransforms' `
    'Support acquisition must use the live touch probe versus the final yellow Weapon-relative palm seat, retain dynamic touch outside that radius, and derive authority from the final authored seat.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'shouldUseAuthoredSupportGrip[\s\S]*input\.proximityProbeAcquisition[\s\S]*input\.authoredSeatTouchAcquisition[\s\S]*!input\.providerAuthorityActive[\s\S]*!input\.attachOnly' `
    'The authored support selector must admit broad probes plus yellow-seat touch acquisition while rejecting provider authority and AttachOnly reload glue.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'shouldUseAuthoredFiringGripProbe[\s\S]*input\.proximityProbeAcquisition[\s\S]*!input\.providerAuthorityActive[\s\S]*!input\.attachOnly' `
    'Authored firing-grip probe takeover must preserve provider and AttachOnly exclusions without changing the selected support authority mode.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'fullTwoHandedAuthority' `
    'Authored pose selection must not silently promote or reject a grip based on its already-selected weapon authority mode.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryReattachFiringGrip[\s\S]*authoredProviderAuthorityActive[\s\S]*authoredAttachOnlyAuthorityActive[\s\S]*shouldUseAuthoredFiringGripProbe[\s\S]*providerAuthorityActive\s*=\s*authoredProviderAuthorityActive[\s\S]*attachOnly\s*=\s*authoredAttachOnlyAuthorityActive[\s\S]*updatePartCarryGrip[\s\S]*authoredProviderAuthorityActive\s*=[\s\S]*leftRuntimeState\.providerPartAuthority\.active[\s\S]*rightRuntimeState\.providerPartAuthority\.active[\s\S]*authoredAttachOnlyAuthorityActive\s*=[\s\S]*providerGrabModeIsAttachOnly[\s\S]*partGrip\(true\)\.attachOnly[\s\S]*partGrip\(false\)\.attachOnly' `
    'Firing-grip probe takeover must wire both hands'' live provider/AttachOnly state without rewriting the support authority mode.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'legacyPalmPivotWorld\s*=\s*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*handInput\.rawHandWorld[\s\S]*rockWeaponInteractionTouchRadius[\s\S]*if\s*\(touchObserved\)[\s\S]*else if\s*\(weaponNode\s*&&\s*probeAllowed\)[\s\S]*rockWeaponInteractionProbeRadius[\s\S]*weapon_interaction_acquisition_policy::resolve\([\s\S]*touchObserved[\s\S]*WeaponInteractionAcquisitionSource::PhysicalContact[\s\S]*WeaponInteractionAcquisitionSource::ProximityProbe' `
    'Both physical hands and both weapon roles must classify a small legacy-palm collider overlap as touch before considering the broad authored probe.'
Require-Text 'src/physics-interaction/weapon/WeaponInteraction.h' `
    'kTouchGraceFrames\s*=\s*2[\s\S]*touchGraceFramesRemaining[\s\S]*decision\.acquisitionSource\s*=\s*contact\.acquisitionSource' `
    'Acquisition provenance must survive one-frame palm-overlap jitter and propagate through weapon routing.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'setAuthoredSupportGripCandidate[\s\S]*refreshAuthoredSupportRightMirror[\s\S]*tryBuildMirroredRightSupportHandWeaponLocal[\s\S]*mirrorFingerLocalTransforms\([\s\S]*Hand::Left[\s\S]*candidate\.rightMirrorValid\s*=\s*true[\s\S]*tryResolveAuthoredSupportGripCandidateForHand[\s\S]*candidate\.rightMirrorValid' `
    'Physical-right support must consume a complete transform and anatomical finger mirror derived from Bethesda''s physical-left support pose.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryBuildMirroredRightSupportHandWeaponLocal[\s\S]*orientationFrame[\s\S]*result\.translate\s*=\s*\{\}[\s\S]*result\.scale\s*=\s*1\.0f[\s\S]*leftPalmWeaponLocal[\s\S]*desiredRightPalmWeaponLocal[\s\S]*mirroredRightHandWeaponLocal\.translate\s*=\s*\{\}[\s\S]*rightPalmOffsetWeaponLocal[\s\S]*mirroredRightHandWeaponLocal\.translate\s*=[\s\S]*sub\(desiredRightPalmWeaponLocal,\s*rightPalmOffsetWeaponLocal\)[\s\S]*anchorError' `
    'The physical-right support mirror must solve orientation as a rigid zero-origin frame and anchor its palm directly in Weapon space without affine translation cancellation.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'mirroredRightHandWeaponLocal\.translate\s*\+[\s\S]{0,80}sub\(desiredRightPalmWeaponLocal' `
    'The right-support mirror must never restore the unstable large-translation cancellation path.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'refreshNaturalHandInWandFrames\(\);[\s\S]*refreshAuthoredSupportRightMirror\(\);' `
    'A right-support mirror that arrived before natural wand frames must be retried after those frames refresh.'
Require-Text 'src/RockConfig.cpp' `
    'fWeaponInteractionTouchRadius[\s\S]*2\.0f[\s\S]*0\.25f[\s\S]*6\.0f[\s\S]*fWeaponInteractionProbeRadius' `
    'The touch sphere must expose a bounded small radius independently from the broad authored probe.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'resolveAuthoredSupportPalmSeatProximity[\s\S]*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*authoredHandWeaponLocal[\s\S]*computeGrabLegacyPalmPivotAWorldFromHandBasis\([\s\S]*liveHandWorld[\s\S]*worldPointToLocal\([\s\S]*liveTouchProbeWeaponLocal[\s\S]*authoredPalmSeatWeaponLocal[\s\S]*weaponLocalDistance\s*\*\s*std::abs\(weaponWorld\.scale\)' `
    'The authored touch gate must compare the live palm probe with the yellow authored palm seat in one current Weapon frame and convert the result back to game units.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'resolveAuthoredSupportWeaponRelativeProximity|liveHandWeaponLocal\.translate\.[xyz]\s*-\s*authoredHandWeaponLocal\.translate\.[xyz]' `
    'The authored selector must never return to the green/blue wrist-bone origin distance.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'drawAuthoredSupportGripDebug[\s\S]*getAuthoredSupportGripDebugSnapshot[\s\S]*AuthoredSupportGripPalmSeat[\s\S]*authoredPalmSeatWorld[\s\S]*AuthoredSupportGripLiveSample[\s\S]*liveTouchProbeWorld[\s\S]*touchRadiusGameUnits[\s\S]*insideTouchRadius' `
    'The experimental path must make the yellow authored palm seat the guide and visualize the live touch probe, radius, and inside/outside decision.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'AuthoredSupportGripTarget|snapshot\.authoredHandWorld|snapshot\.liveHandWorld' `
    'The debug overlay must not retain the obsolete green authored wrist target or blue live wrist sample.'
Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' `
    'AuthoredSupportGripTarget' `
    'The obsolete green authored wrist marker role must be removed rather than left as dead debug infrastructure.'
Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.cpp' `
    'AuthoredSupportGripTarget' `
    'The obsolete green authored wrist marker color path must be removed with the marker.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyAuthoredPrimaryGripWeaponAlignment[\s\S]{0,600}isManualOwnershipActive\(\)' `
    'Right-hand PrimaryOnly bookkeeping must not suppress the authored weapon calibration.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'conflictingWeaponTransformAuthorityActive\s*=\s*_twoHandedGrip\.blocksAuthoredPrimaryGripWeaponAlignment\(\)' `
    'Authored alignment eligibility must consume the narrow transform-conflict predicate.'
Require-Text 'src/ROCKMain.cpp' `
    's_physicsInteraction->updateAuthoredPrimaryFiringGripExperiment\(\);[\s\S]{0,180}s_physicsInteraction->update\(\)' `
    'The authored weapon alignment must run before ROCK collision, probes, and manual grip capture.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'updateAuthoredPrimaryFiringGripExperiment[\s\S]*getCurrentEquippedWeaponOwnershipKey\(\)[\s\S]*weaponOwnershipKey\s*=\s*currentEquippedWeaponFormId\(\)' `
    'The experiment must retain a weapon freshness key when generated weapon collision is disabled.'
Require-Text 'src/RockConfig.cpp' `
    'rockAuthoredPrimaryFiringGripTestEnabled\s*=\s*false[\s\S]*GetBoolValue\(\s*EXPERIMENTAL_SECTION,\s*"bAuthoredPrimaryFiringGripTestEnabled"' `
    'The authored firing-grip experiment must default off and load only from [Experimental].'
Reject-Text 'src/RockConfig.cpp' `
    'rockAuthoredSupportGripSnapRadius|fAuthoredSupportGripSnapRadius' `
    'The obsolete distance-gated authored support radius must not remain in runtime configuration.'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $configText = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $experimentalMatch = [regex]::Match($configText, '(?ms)^\[Experimental\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (-not $experimentalMatch.Success -or
        $experimentalMatch.Groups['body'].Value -notmatch '(?m)^bAuthoredPrimaryFiringGripTestEnabled\s*=\s*false\s*$' -or
        $experimentalMatch.Groups['body'].Value -match '(?m)^fAuthoredSupportGripSnapRadius\s*=') {
        $failures.Add("$configPath`: Authored firing-grip experiment must exist under [Experimental] without the obsolete distance gate.")
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'NativeAnimationAuthoritySourceTests passed.' -ForegroundColor Green

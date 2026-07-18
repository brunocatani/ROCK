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
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'desiredAnchor\s*=\s*liveControl\s*\*\s*authoredDelta[\s\S]*correction\s*=\s*desiredAnchor\s*\*\s*inverse\(authoredCurrent\)[\s\S]*resolveControllerAnchoredPoseCorrection' `
    'Native reload motion must preserve one rigid authored pose inside the live controller aim frame.'
Reject-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'classifyBone[\s\S]*return\s+kReloadPose' `
    'The classifier must never broadly grant every pose flag to arbitrary body bones.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' `
    'kFunc_ReloadStateChangeHandler_Handle\s*=\s*0x0FF2B90[\s\S]*kFunc_GetReloadStartStateToken\s*=\s*0x16A3070[\s\S]*kFunc_GetReloadEndStateToken\s*=\s*0x16A30D0[\s\S]*kVtableEntry_ReloadStateChangeHandler_Handle\s*=\s*0x2D8D300' `
    'Reload lifecycle authority must stay pinned to the independently verified FO4VR handler, tokens, and vtable slot.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onReloadStateChange[\s\S]*nativeReloadStartStateToken[\s\S]*s_playerReloadStartSequence\.fetch_add[\s\S]*nativeReloadEndStateToken[\s\S]*s_playerReloadEndSequence\.fetch_add' `
    'The lifecycle hook must classify the verified Bethesda start/end tokens and publish player-only event sequences.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'advanceLocalReloadLease[\s\S]*installReloadStateChangeHook[\s\S]*expectedTarget[\s\S]*kFunc_ReloadStateChangeHandler_Handle[\s\S]*VirtualProtect' `
    'The local ROCK test lease must consume event sequences from a validated ReloadStateChangeHandler vtable hook.'
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
    'tryResolveAuthoredSupportGrip[\s\S]*expectedWeaponNode->parent\s*!=\s*capturedPrimaryHandNode[\s\S]*outSupportHandInWeapon\s*=\s*s_authoredSupportHandInWeapon[\s\S]*outFingerLocalTransforms\s*=\s*s_authoredSupportFingerLocals[\s\S]*kAuthoredSupportFingerTransformMask' `
    'The support resolver must retain scene identity and publish one complete generation-ready hand/finger frame.'

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
    'clearAuthoredSupportGripCandidate\(\)[\s\S]*applyAuthoredPrimaryGripWeaponAlignment[\s\S]*tryResolveAuthoredSupportGrip[\s\S]*setAuthoredSupportGripCandidate' `
    'The support candidate must be frame-ephemeral and published only after the primary weapon alignment establishes its final basis.'
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
    'refreshRightNativeCanonicalFrame[\s\S]*_rightNaturalBoneInWand\s*=\s*boneInRightWand[\s\S]*hasRightFiringHandCanonicalFrame[\s\S]*RightFiringCanonicalSource::AuthoredAnimation[\s\S]*return;[\s\S]*RightFiringCanonicalSource::NativeCarry' `
    'Native presentation refresh must retain anatomy sampling without overwriting a matching authored canonical.'
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
    'capturePartGrip[\s\S]*shouldUseAuthoredSupportGrip[\s\S]*providerAuthorityActive\s*=\s*providerPartAuthority\.active[\s\S]*grip\.handWeaponLocal\s*=[\s\S]*fingerLocalTransforms[\s\S]*return\s+true;[\s\S]*tryGetSupportGripEvidenceView' `
    'Support acquisition must prioritize provider authority, latch the complete authored frame inside its zone, and retain the existing dynamic mesh fallback.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthorityPolicy.h' `
    'shouldUseAuthoredSupportGrip[\s\S]*!input\.providerAuthorityActive[\s\S]*input\.weaponRelativeHandDistanceGameUnits\s*<=\s*input\.snapRadiusGameUnits' `
    'The authored support selector must preserve provider priority and require tight Weapon-relative hand-target proximity.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'resolveAuthoredSupportWeaponRelativeProximity[\s\S]*invertTransform\(weaponWorld\)[\s\S]*liveHandWeaponLocal[\s\S]*authoredHandWeaponLocal\.translate\.x[\s\S]*weaponLocalDistance\s*\*\s*std::abs\(weaponWorld\.scale\)' `
    'The authored support distance must compare live and captured LArm_Hand translations in one current Weapon frame and convert the local result back to game units.'
Reject-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'authoredSupportPalmDistance|sub\(\s*palmPos,\s*authoredSupportPalmWorld\s*\)' `
    'The authored selector must not regress to world-space or configured-palm proximity.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' `
    'drawAuthoredSupportGripDebug[\s\S]*getAuthoredSupportGripDebugSnapshot[\s\S]*AuthoredSupportGripTarget[\s\S]*AuthoredSupportGripPalmSeat[\s\S]*AuthoredSupportGripLiveSample[\s\S]*weaponRelativeDistanceGameUnits' `
    'The experimental path must continuously visualize its authored target, solver palm seat, live sample, and measured Weapon-relative error before acquisition.'
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
    'rockAuthoredPrimaryFiringGripTestEnabled\s*=\s*false[\s\S]*rockAuthoredSupportGripSnapRadius\s*=\s*2\.0f[\s\S]*GetBoolValue\(\s*EXPERIMENTAL_SECTION,\s*"bAuthoredPrimaryFiringGripTestEnabled"[\s\S]*"fAuthoredSupportGripSnapRadius"[\s\S]*std::clamp\(rockAuthoredSupportGripSnapRadius,\s*0\.25f,\s*12\.0f\)' `
    'The authored firing-grip experiment must default off and load a bounded support snap radius only from [Experimental].'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $configText = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $experimentalMatch = [regex]::Match($configText, '(?ms)^\[Experimental\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (-not $experimentalMatch.Success -or
        $experimentalMatch.Groups['body'].Value -notmatch '(?m)^bAuthoredPrimaryFiringGripTestEnabled\s*=\s*false\s*$' -or
        $experimentalMatch.Groups['body'].Value -notmatch '(?m)^fAuthoredSupportGripSnapRadius\s*=\s*2\.0\s*$') {
        $failures.Add("$configPath`: Authored firing-grip experiment and support snap radius must exist under [Experimental] with safe defaults.")
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'NativeAnimationAuthoritySourceTests passed.' -ForegroundColor Green

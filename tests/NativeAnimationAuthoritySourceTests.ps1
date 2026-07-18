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
    'kFunc_UpdateFirstPersonArm\s*=\s*0xEF6280[\s\S]*kCallsite_UpdateFirstPersonArmPrimaryReturn\s*=\s*0xEF610D' `
    'The primary firing-grip probe must stay pinned to the independently verified FO4VR helper and Bethesda primary-call return.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'kExpectedUpdateFirstPersonArmPrefix[\s\S]*0x48,\s*0x8B,\s*0xC4,\s*0x55,\s*0x53,\s*0x41,\s*0x56,[\s\S]*0x48,\s*0x8D,\s*0xA8,\s*0xF8,\s*0xFE,\s*0xFF,\s*0xFF[\s\S]*kFunc_UpdateFirstPersonArm' `
    'The native arm interception must validate the complete position-independent FO4VR prologue before patching.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'onUpdateFirstPersonArm[\s\S]*_ReturnAddress\(\)[\s\S]*s_originalUpdateFirstPersonArm\([\s\S]*returnAddress\s*!=\s*s_nativePrimaryArmReturnAddress[\s\S]*captureNativePrimaryFiringGrip\(\)' `
    'The hook must call the native helper first and capture only Bethesda''s verified primary pass, never hFRIK''s later calls.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'captureNativePrimaryFiringGrip[\s\S]*weaponTransform\.parPos\s*!=\s*handIndex[\s\S]*invertTransform\(weaponTransform\.refNode->world\)[\s\S]*handTransform\.refNode->world[\s\S]*s_authoredPrimaryHandInWeapon\s*=\s*handInWeapon' `
    'The authored grip must be captured as Bethesda''s pre-hFRIK primary hand in the visible Weapon world frame.'
Require-Text 'src/physics-interaction/animation/NativeAnimationAuthority.cpp' `
    'tryResolvePrimaryFiringGripAlignment[\s\S]*expectedWeaponNode->parent\s*!=\s*capturedHandNode[\s\S]*resolveAuthoredPrimaryWeaponWorld\([\s\S]*trackedPrimaryHandWorld[\s\S]*s_authoredPrimaryHandInWeapon' `
    'The experiment must invert the captured native relation onto the tracked primary hand and reject a changed hand/weapon hierarchy.'

$nativeAuthorityText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/animation/NativeAnimationAuthority.cpp')
$primaryCaptureMatch = [regex]::Match(
    $nativeAuthorityText,
    '(?s)captureNativePrimaryFiringGrip\(\).*?(?=\s+__declspec\(noinline\))')
if (-not $primaryCaptureMatch.Success) {
    $failures.Add('The native primary firing-grip capture function could not be isolated for source validation.')
} elseif ($primaryCaptureMatch.Value -match 'refNode->local|weaponTransform\.local|authoritativeLocal\s*\(') {
    $failures.Add('The authored primary grip must never capture hFRIK''s downstream Weapon local/offset path.')
}

Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'getHandWorldTransform\([\s\S]*Hand::Primary[\s\S]*tryResolvePrimaryFiringGripAlignment[\s\S]*applyAuthoredPrimaryGripWeaponAlignment' `
    'The experiment must preserve the controller-driven primary hand and apply the inverse solve through the shared weapon visual path.'
Reject-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'applyExternalHandWorldTransform|clearExternalHandWorldTransform|Hand::(?:Offhand|Left|Right)|weaponNode->(?:local|world)\s*=|blockPrimaryWeaponNodeOwnership' `
    'The primary-only alignment must not move either hand, write the node outside the shared weapon path, or enter hFRIK''s left-carry topology.'
Require-Text 'src/physics-interaction/weapon/AuthoredPrimaryFiringGrip.cpp' `
    'input\.nativeReloadAuthorityActive[\s\S]*endSession\("native-reload-authority"\)[\s\S]*captureSequenceFloor' `
    'Native reload authority must suspend the alignment and require a fresh capture before restoring it.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyAuthoredPrimaryGripWeaponAlignment[\s\S]*isManualOwnershipActive\(\)[\s\S]*isWeaponVisualReturnActive\(\)[\s\S]*applyWeaponVisualAuthority' `
    'Authored alignment must reuse the scope-aware weapon visual path and yield to manual/return authority.'
Require-Text 'src/ROCKMain.cpp' `
    's_physicsInteraction->updateAuthoredPrimaryFiringGripExperiment\(\);[\s\S]{0,180}s_physicsInteraction->update\(\)' `
    'The authored weapon alignment must run before ROCK collision, probes, and manual grip capture.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'updateAuthoredPrimaryFiringGripExperiment[\s\S]*getCurrentEquippedWeaponOwnershipKey\(\)[\s\S]*weaponOwnershipKey\s*=\s*currentEquippedWeaponFormId\(\)' `
    'The experiment must retain a weapon freshness key when generated weapon collision is disabled.'
Require-Text 'src/RockConfig.cpp' `
    'rockAuthoredPrimaryFiringGripTestEnabled\s*=\s*false[\s\S]*GetBoolValue\(\s*EXPERIMENTAL_SECTION,\s*"bAuthoredPrimaryFiringGripTestEnabled"' `
    'The authored firing-grip experiment must default off and load only from [Experimental].'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $configText = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $experimentalMatch = [regex]::Match($configText, '(?ms)^\[Experimental\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (-not $experimentalMatch.Success -or
        $experimentalMatch.Groups['body'].Value -notmatch '(?m)^bAuthoredPrimaryFiringGripTestEnabled\s*=\s*false\s*$') {
        $failures.Add("$configPath`: Authored primary firing-grip experiment must exist under [Experimental] and default off.")
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'NativeAnimationAuthoritySourceTests passed.' -ForegroundColor Green

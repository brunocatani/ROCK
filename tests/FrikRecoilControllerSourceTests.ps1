param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$workspaceRoot = Split-Path -Parent $Root
$hfrikRoot = Join-Path $workspaceRoot 'hFRIK'

function Require-Text {
    param(
        [string]$Base,
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Base $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Base,
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-Content -Raw -LiteralPath (Join-Path $Base $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text $hfrikRoot 'src/api/FRIKApi.h' 'FRIK_API_VERSION\s*=\s*5' `
    'hFRIK recoil control must remain in the additive V5 API contract.'
Require-Text $Root 'src/api/FRIKApi.h' 'FRIK_API_VERSION\s*=\s*5' `
    'ROCK must consume the additive V5 FRIK API contract.'
Require-Text $hfrikRoot 'src/api/FRIKApi.h' `
    'getApiStructSize\(\)\s*!=\s*sizeof\(FRIKApi\)' `
    'The additive V5 consumer handshake must reject any mismatched function table.'
Require-Text $Root 'src/api/FRIKApi.h' `
    'getApiStructSize\(\)\s*!=\s*sizeof\(FRIKApi\)' `
    'ROCK must require an exact additive V5 function-table match before dereferencing members.'

Reject-Text $hfrikRoot 'src/Config.h' 'rawWeaponRecoil' `
    'Raw weapon recoil must not remain user-configurable.'
Reject-Text $hfrikRoot 'src/Config.cpp' 'RawWeaponRecoil|rawWeaponRecoil' `
    'The removed raw recoil setting must not be read at runtime.'
Reject-Text $hfrikRoot 'data/config/FRIK.ini' 'RawWeaponRecoil' `
    'The shipped FRIK INI must not expose raw recoil.'

Require-Text $hfrikRoot 'src/api/RecoilControllerRuntime.cpp' `
    'MAX_RECOIL_CONTROLLERS\s*=\s*16[\s\S]*g_invokingRecoilController[\s\S]*priority[\s\S]*generation' `
    'hFRIK recoil ownership must use a bounded, non-reentrant priority registry.'
Require-Text $hfrikRoot 'src/api/RecoilControllerRuntime.cpp' `
    'controller\(&sample,\s*&response,\s*ordered\[index\]->userData\)[\s\S]*continue;[\s\S]*isValidResponse' `
    'A declining or invalid recoil controller must allow the next controller or regular FRIK fallback.'

Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'handleLeftHandedWeaponNodesSwitch\(\);[\s\S]*prepareWeaponHandRecoilFrame\(\);[\s\S]*setArms\(false\);[\s\S]*setArms\(true\);' `
    'hFRIK must sample recoil exactly once before both arm passes.'
Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'primaryWeaponKickbackRecoilNode[\s\S]*isFiniteTransform\(kickbackNode->local\)[\s\S]*kickbackNode->parent[\s\S]*isFiniteTransform\(kickbackNode->parent->world\)[\s\S]*return;[\s\S]*nativeKickLocal\s*=\s*kickbackNode->local[\s\S]*resolveWeaponHandRecoil' `
    'hFRIK must validate the native kick frame internally before invoking external recoil controllers.'
Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'ScopedKickbackNeutralizer[\s\S]*_weaponHandRecoilResponseAccepted[\s\S]*dampenHand\(offsetNode,\s*isLeft\);[\s\S]*Update1StPersonArm' `
    'Accepted API recoil must replace native hand recoil while a declined frame retains the original dampen-and-arm pipeline.'
Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'applyExternalHandWorldTransform[\s\S]*applyControlledWeaponHandRecoil\(isLeft,\s*controlledWorldTarget\)' `
    'External hand targets must receive the same cached controlled recoil as hFRIK arm targets.'
Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'delivery\s*==\s*api::FRIKApi::RecoilDelivery::Damped[\s\S]*dampenControlledWeaponHandRecoil[\s\S]*else\s*\{[\s\S]*controlledKickLocal' `
    'hFRIK must distinguish damped controller delivery from direct controller delivery.'

Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'registerWeaponHandRecoilController\([\s\S]*WEAPON_RECOIL_CONTROLLER_TAG[\s\S]*controlWeaponHandRecoil[\s\S]*this[\s\S]*GRIP_HAND_POSE_PRIORITY' `
    'ROCK must register its recoil controller with instance-bound lifetime.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '~TwoHandedGrip\(\)[\s\S]*unregisterWeaponHandRecoilController' `
    'ROCK must unregister the recoil callback before its instance storage is destroyed.'
Require-Text $Root 'src/physics-interaction/weapon/WeaponSupport.h' `
    'shouldApplyVisualOnlySupportRecoilAssist[\s\S]{0,500}WeaponSupportAuthorityMode::VisualOnlySupport[\s\S]{0,250}supportGripActive[\s\S]{0,250}!providerAuthorityActive[\s\S]{0,120}!attachOnly' `
    'Only a committed core visual-only support grip may receive recoil-only authority.'
Require-Text $Root 'src/physics-interaction/weapon/WeaponAuthority.h' `
    'kVisualOnlySupportTranslationFraction\s*=\s*0\.45f[\s\S]{0,160}kVisualOnlySupportRotationFraction\s*=\s*0\.30f[\s\S]*tryBuildVisualOnlySupportKick[\s\S]*controlledHalfAngle[\s\S]*kVisualOnlySupportTranslationFraction' `
    'Close support recoil must use explicit bounded linear and angular fractions.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'hasVisualOnlySupportRecoilAssist[\s\S]{0,1800}TwoHandedState::Gripping[\s\S]{0,1800}shouldApplyVisualOnlySupportRecoilAssist[\s\S]*controlWeaponHandRecoil[\s\S]{0,1200}tryBuildVisualOnlySupportKick[\s\S]{0,1300}leftFiringCarryAuthority[\s\S]{0,900}leftFiringCarryAuthority\s*\?[\s\S]{0,120}RecoilHandMask::None[\s\S]{0,120}RecoilHandMask::Primary[\s\S]{0,250}RecoilDelivery::Direct[\s\S]{0,250}controlledKickLocal' `
    'ROCK must suppress hFRIK hand recoil only while ROCK owns physical-left carry and retain the physical-right visual-only assist.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'void TwoHandedGrip::captureLeftFiringWeaponRecoil\([\s\S]{0,2600}tryResolveKickFrame\([\s\S]{0,900}_leftFiringWeaponRecoilParentWorld\s*=\s*recoilParentWorld[\s\S]{0,160}_leftFiringWeaponRecoilLocal\s*=\s*recoilLocal[\s\S]{0,160}_leftFiringWeaponRecoilSampleValid\s*=\s*true' `
    'ROCK must retain one compact parent/local recoil frame without callback scene writes.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '_leftFiringWeaponRecoilReadyThisUpdate[\s\S]{0,700}_firingHandIsLeft[\s\S]{0,300}TwoHandedState::Gripping[\s\S]{0,350}FullTwoHandedSolver[\s\S]{0,1800}tryApplyKickToWorldTarget\([\s\S]{0,240}_leftFiringWeaponRecoilParentWorld[\s\S]{0,160}_leftFiringWeaponRecoilLocal[\s\S]{0,500}_leftFiringWeaponRecoilSupportConstrainedThisUpdate\s*=\s*true' `
    'ROCK must feed physical-left recoil into the full two-hand primary target so the support target constrains the kick.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'solveLeftFiringWeaponCarry[\s\S]{0,2600}handKickReady[\s\S]{0,400}tryApplyKickToWorldTarget\([\s\S]{0,240}_leftFiringWeaponRecoilParentWorld[\s\S]{0,180}_leftFiringWeaponRecoilLocal[\s\S]{0,2200}applyExternalHandWorldTransform\([\s\S]{0,300}firingHandTargetWorld' `
    'ROCK must apply its resolved recoil frame to the one-hand firing target before FRIK publication.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'solveLeftFiringWeaponCarry[\s\S]{0,3500}handKickReady[\s\S]{0,1200}weaponKickReady[\s\S]{0,1200}_leftFiringWeaponRecoilReadyThisUpdate\s*=\s*false' `
    'ROCK must resolve and consume the one-hand firing-hand/weapon recoil pair before either final publication.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyExternalHandWorldTransform\([\s\S]{0,350}firingHandTargetWorld[\s\S]{0,2200}applyWeaponVisualAuthority\(weaponNode,\s*weaponTargetWorld\)' `
    'ROCK must publish the recoiled firing hand before the matching recoiled weapon, allowing visual support to follow that weapon in the same update.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'leftFiringCarryRequested[\s\S]{0,500}_leftFiringWeaponRecoilSampleValid[\s\S]{0,450}if\s*\(!responseAccepted\)[\s\S]{0,300}return false;' `
    'A missing ROCK recoil frame must decline ownership and preserve regular FRIK fallback.'
foreach ($rockRecoilPath in @(
        'src/physics-interaction/weapon/TwoHandedGrip.cpp',
        'src/physics-interaction/weapon/TwoHandedGrip.h',
        'src/physics-interaction/weapon/TwoHandedGripDebug.cpp',
        'src/physics-interaction/weapon/WeaponAuthority.h')) {
    Reject-Text $Root $rockRecoilPath `
        '_leftFiringWeaponRecoilWorldDelta|resolveWorldDelta|left supported recoil hand precompensation|requestedFiringHandWorld|applyLeftFiringWeaponRecoil' `
        'ROCK must not reconstruct, cache, precompensate, or terminally reapply recoil outside its owning solve.'
}
Require-Text $Root 'src/ROCKMain.cpp' `
    'registerWeaponHandRecoilController\s*!=\s*nullptr[\s\S]*unregisterWeaponHandRecoilController\s*!=\s*nullptr' `
    'ROCK startup must fail closed when the matching V5 recoil-controller table is absent.'
Require-Text $hfrikRoot 'src/api/FRIKApi.h' `
    'struct\s+RecoilSample[\s\S]*structSize[\s\S]*reserved0\[3\][\s\S]*nativeKickLocal[\s\S]*sizeof\(RecoilSample\)\s*==\s*112' `
    'hFRIK must expose only the solve-critical native kick sample.'
Reject-Text $hfrikRoot 'src/api/FRIKApi.h' `
    'RecoilContextFlag|contextFlags|physicalPrimaryHand|std::uint64_t\s+sequence|float\s+deltaSeconds' `
    'hFRIK must not mirror game-derived context or redundant frame bookkeeping through recoil V5.'
Reject-Text $Root 'src/api/FRIKApi.h' `
    'RecoilContextFlag|contextFlags|physicalPrimaryHand|std::uint64_t\s+sequence|float\s+deltaSeconds' `
    'ROCK must mirror the minimal solve-critical recoil sample.'
Reject-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    '_weaponHandRecoilSequence|RecoilContextFlag|physicalPrimaryHand' `
    'hFRIK must keep physical-hand routing internal without rebuilding removed public recoil metadata.'
Reject-Text $hfrikRoot 'src/api/FRIKApi.h' `
    'RecoilState|getWeaponHandRecoilState' `
    'hFRIK V5 must not expose unused recoil telemetry.'
Reject-Text $hfrikRoot 'src/api/RecoilControllerRuntime.cpp' `
    'RecoilState|getWeaponHandRecoilState|g_lastRecoilState|g_hasLastRecoilState' `
    'hFRIK must not build or cache unused per-frame recoil telemetry.'
Reject-Text $Root 'src/api/FRIKApi.h' `
    'RecoilState|getWeaponHandRecoilState' `
    'ROCK must mirror the telemetry-free hFRIK V5 contract.'
Reject-Text $Root 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'RecoilState|getWeaponHandRecoilState' `
    'ROCK must not retain an unused recoil telemetry bridge.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

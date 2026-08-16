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

Require-Text $hfrikRoot 'src/api/FRIKApiV2.h' 'FRIK_API_V2_VERSION\s*=\s*1' `
    'hFRIK recoil control must remain in the API V2 contract.'
Require-Text $Root 'src/api/FRIKApiV2.h' 'FRIK_API_V2_VERSION\s*=\s*1' `
    'ROCK must consume the FRIK API V2 contract.'
Require-Text $hfrikRoot 'src/api/FRIKApiV2.h' `
    'getApiStructSize\(\)\s*!=\s*sizeof\(FRIKApiV2\)' `
    'The API V2 consumer handshake must reject any mismatched function table.'
Require-Text $Root 'src/api/FRIKApiV2.h' `
    'getApiStructSize\(\)\s*!=\s*sizeof\(FRIKApiV2\)' `
    'ROCK must require an exact API V2 function-table match before dereferencing members.'

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
    'handleLeftHandedWeaponNodesSwitch\(\);[\s\S]*_weaponHandRecoil\.onFrameUpdate\([\s\S]*setArms\(false\);[\s\S]*setArms\(true\);' `
    'hFRIK must sample recoil exactly once before both arm passes.'
Require-Text $hfrikRoot 'src/skeleton/WeaponHandRecoil.cpp' `
    'primaryWeaponKickbackRecoilNode[\s\S]*isFiniteTransform\(kickbackNode->local\)[\s\S]*kickbackNode->parent[\s\S]*isFiniteTransform\(kickbackNode->parent->world\)[\s\S]*return;[\s\S]*nativeKickLocal\s*=\s*kickbackNode->local[\s\S]*resolveWeaponHandRecoil' `
    'hFRIK must validate the native kick frame internally before invoking external recoil controllers.'
Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'ScopedNativeKickNeutralizer[\s\S]*dampenHand\(offsetNode,\s*isLeft\);[\s\S]*Update1StPersonArm' `
    'Accepted API recoil must replace native hand recoil while a declined frame retains the original dampen-and-arm pipeline.'
Require-Text $hfrikRoot 'src/skeleton/Skeleton.cpp' `
    'g_externalAuthority\.getHandWorldTransform[\s\S]*_weaponHandRecoil\.applyToHandWorldTarget\(isLeft,\s*handWorldTarget\)' `
    'External hand targets must receive the same cached controlled recoil as hFRIK arm targets.'
Require-Text $hfrikRoot 'src/skeleton/WeaponHandRecoil.cpp' `
    'delivery\s*==\s*api::FRIKApiV2::RecoilDelivery::Damped[\s\S]*_controlledKickLocal\s*=\s*dampen[\s\S]*else\s*\{[\s\S]*_controlledKickLocal\s*=\s*response\.controlledKickLocal' `
    'hFRIK must distinguish damped controller delivery from direct controller delivery.'

Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'registerWeaponHandRecoilController\([\s\S]*WEAPON_RECOIL_CONTROLLER_TAG[\s\S]*controlWeaponHandRecoil[\s\S]*this[\s\S]*GRIP_HAND_POSE_PRIORITY' `
    'ROCK must register its recoil controller with instance-bound lifetime.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '~TwoHandedGrip\(\)[\s\S]*unregisterWeaponHandRecoilController' `
    'ROCK must unregister the recoil callback before its instance storage is destroyed.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '_weaponNodeOwnershipBlockEngaged[\s\S]*_firingHandIsLeft[\s\S]*isManualOwnershipActive\(\)[\s\S]*RecoilHandMask::Primary[\s\S]*RecoilDelivery::Direct[\s\S]*nativeKickLocal' `
    'ROCK may consume recoil only for its active physical-left firing carry and must preserve the validated native kick directly.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'controlWeaponHandRecoil[\s\S]*areTransformsNearlyEqual\([\s\S]*sample->nativeKickLocal[\s\S]*_leftFiringRecoilAcceptedSequence[\s\S]*_leftFiringRecoilAcceptedGenerationKey\s*=\s*self->_activeWeaponGenerationKey[\s\S]*else\s*\{[\s\S]*_leftFiringRecoilConsumedSequence\s*=\s*self->_leftFiringRecoilAcceptedSequence' `
    'A non-neutral accepted FRIK recoil frame must issue a generation-bound, one-shot ROCK presentation ticket, while the next neutral frame retires any skipped ticket instead of replaying stale recoil.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'solveLeftFiringWeaponCarry[\s\S]*rememberLeftFiringRecoilReference\(firingHandTransform\)' `
    'The left-carry solve must preserve its clean firing-hand target as the initial recoil reference.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'recordPublishedHandWorld[\s\S]{0,900}rememberLeftFiringRecoilReference\(appliedWorld\)' `
    'Late grip, gunstock, or collision hand publications must replace the initial recoil reference with their clean logical target.'
Require-Text $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'applyLeftFiringWeaponRecoilPresentation[\s\S]*_leftFiringRecoilConsumedSequence\s*=\s*acceptedSequence[\s\S]*_leftFiringRecoilAcceptedGenerationKey[\s\S]*tryGetRootFlattenedHandBoneTransform[\s\S]*deriveAppliedWorldDelta[\s\S]*composeTransforms\(\s*recoilWorldDelta,\s*weaponNode->world\)[\s\S]*applyWeaponVisualAuthority\([\s\S]*false\)' `
    'ROCK must consume each accepted recoil once and apply the measured hand recoil delta as a final generation-safe weapon presentation overlay.'
Require-Text $Root 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'finishWeaponCollisionPresentationFrame[\s\S]{0,500}applyLeftFiringWeaponRecoilPresentation[\s\S]{0,700}updateBodiesFromCurrentSourceTransforms[\s\S]{0,700}applyFinalWeaponMuzzleAuthority' `
    'Left-firing weapon recoil must run after all weapon writers and before collision-body and muzzle consumers.'
Require-Text $Root 'src/ROCKMain.cpp' `
    'registerWeaponHandRecoilController\s*!=\s*nullptr[\s\S]*unregisterWeaponHandRecoilController\s*!=\s*nullptr' `
    'ROCK startup must fail closed when the matching API V2 recoil-controller table is absent.'
Require-Text $hfrikRoot 'src/api/FRIKApiV2.h' `
    'struct\s+RecoilSample[\s\S]*structSize[\s\S]*reserved0\[3\][\s\S]*nativeKickLocal[\s\S]*sizeof\(RecoilSample\)\s*==\s*112' `
    'hFRIK must expose only the solve-critical native kick sample.'
Reject-Text $hfrikRoot 'src/api/FRIKApiV2.h' `
    'RecoilContextFlag|contextFlags|physicalPrimaryHand|std::uint64_t\s+sequence|float\s+deltaSeconds' `
    'hFRIK must not mirror game-derived context or redundant frame bookkeeping through recoil API V2.'
Reject-Text $Root 'src/api/FRIKApiV2.h' `
    'RecoilContextFlag|contextFlags|physicalPrimaryHand|std::uint64_t\s+sequence|float\s+deltaSeconds' `
    'ROCK must mirror the minimal solve-critical recoil sample.'
Reject-Text $hfrikRoot 'src/skeleton/WeaponHandRecoil.cpp' `
    '_weaponHandRecoilSequence|RecoilContextFlag|physicalPrimaryHand' `
    'hFRIK must keep physical-hand routing internal without rebuilding removed public recoil metadata.'
Reject-Text $hfrikRoot 'src/api/FRIKApiV2.h' `
    'RecoilState|getWeaponHandRecoilState' `
    'hFRIK API V2 must not expose unused recoil telemetry.'
Reject-Text $hfrikRoot 'src/api/RecoilControllerRuntime.cpp' `
    'RecoilState|getWeaponHandRecoilState|g_lastRecoilState|g_hasLastRecoilState' `
    'hFRIK must not build or cache unused per-frame recoil telemetry.'
Reject-Text $Root 'src/api/FRIKApiV2.h' `
    'RecoilState|getWeaponHandRecoilState' `
    'ROCK must mirror the telemetry-free hFRIK API V2 contract.'
Reject-Text $Root 'src/physics-interaction/visual/FrikVisualAuthorityBridge.h' `
    'RecoilState|getWeaponHandRecoilState' `
    'ROCK must not retain an unused recoil telemetry bridge.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

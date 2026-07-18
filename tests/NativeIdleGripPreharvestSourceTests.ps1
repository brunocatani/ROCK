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

$source = 'src/physics-interaction/weapon/NativeIdleGripPreharvest.cpp'
$policy = 'src/physics-interaction/weapon/NativeIdleGripPreharvestPolicy.h'

Require-Text $source `
    'kExtraAnimGraphPreloadCtor\s*=\s*0x0C8FC0[\s\S]*kRequestAnimationSubGraph\s*=\s*0x10162B0[\s\S]*kGetClipGeneratorBinding\s*=\s*0x1774800[\s\S]*kGetAnimationFilesForSubgraph\s*=\s*0x1769140[\s\S]*kFindBoneWithName\s*=\s*0x190A580' `
    'The proof must remain pinned to the independently verified FO4VR native preload, request, clip, and skeleton functions.'
Require-Text $source `
    'RUNTIME_VR_1_2_72[\s\S]*validateNativeEntry\("ExtraAnimGraphPreload::ctor"[\s\S]*validateNativeEntry\("GetClipGeneratorBinding"' `
    'Hardcoded FO4VR calls must retain executable identity and live-byte gates.'
Require-Text $source `
    'validateNativeEntry\("ExtraAnimGraphPreload::IsFinishedLoading",\s*kIsFinishedLoading,\s*std::array<std::uint8_t,\s*7>\{\s*0x40,\s*0x53,\s*0x57,\s*0x48,\s*0x83,\s*0xEC,\s*0x28\s*\}\)' `
    'The IsFinishedLoading gate must retain the verified redundant REX prefix present in Fallout4VR.exe 1.2.72.'
Require-Text $policy `
    'kFirstPersonGraphIndex\s*=\s*1[\s\S]*graphCount\s*<=\s*kFirstPersonGraphIndex[\s\S]*identifierCount\s*<=\s*kFirstPersonGraphIndex' `
    'The sampler must fail closed unless Bethesda produced the paired first-person graph and identifier.'
Require-Text $source `
    'findBoneWithName\(skeleton,\s*"Weapon"[\s\S]*findBoneWithName\(skeleton,\s*"RArm_Hand"[\s\S]*WPNIdleReady[\s\S]*WPNIdle' `
    'The proof must sample authored idle clips and resolve the exact Weapon/RArm_Hand skeleton relation.'
Require-Text $source `
    'kAnimationFileLookupSingleton\s*=\s*0x5B64318[\s\S]*tryReadValue\([\s\S]*lookupSingleton[\s\S]*getAnimationFilesForSubgraph\(\s*&outSubgraphIdentifier\)[\s\S]*clipPathHasStem[\s\S]*trySampleClip' `
    'Clip binding must consume the exact winning AnimationFileData path for the selected subgraph instead of guessing a basename.'
Require-Text $source `
    'isFinishedLoading[\s\S]*requestAnimationSubGraph[\s\S]*isAnimationSubGraphLoaded[\s\S]*releaseAnimationSubGraph[\s\S]*extraDtor' `
    'Off-screen native loads must be polled asynchronously and released before destroying their preload owner.'
Require-Text $source `
    'rockAuthoredPrimaryFiringGripTestEnabled' `
    'The proof must remain behind the existing ROCK authored-grip experiment.'
Require-Text $source `
    'authored_weapon_grip_library::publish' `
    'The proof must publish only through ROCK''s bounded authored-grip cache.'
Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'NativeIdleGripPreharvest\.h[\s\S]*observeCandidate\([\s\S]*updateHoverCandidateWeapon[\s\S]*observeCandidate\(candidateRef\)' `
    'Held and hover loose-weapon paths must advance the demand-driven preharvest state machine.'
Reject-Text $source `
    'PAPER|PAPERRedux|PAPER_Redux' `
    'The narrow ROCK proof must not acquire a PAPER dependency.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native idle-grip preharvest source boundaries passed.'

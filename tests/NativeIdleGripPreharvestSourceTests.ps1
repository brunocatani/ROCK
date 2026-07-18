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
    'kSimpleAnimationGraphManagerHolderCtor\s*=\s*0x0811F10[\s\S]*kCreateBackgroundSimpleManager\s*=\s*0x0811FE0[\s\S]*kIsAnimationLoadingComplete\s*=\s*0x08122C0[\s\S]*kRequestAnimationSubGraph\s*=\s*0x10162B0[\s\S]*kGetClipGeneratorBinding\s*=\s*0x1774800[\s\S]*kGetAnimationFilesForSubgraph\s*=\s*0x1769140[\s\S]*kLoadIdleAnimationResource\s*=\s*0x1728BA0[\s\S]*kMoveAnimationResourceHandle\s*=\s*0x172AB40[\s\S]*kIsHkxDerivativeDbData\s*=\s*0x152C0D0[\s\S]*kRetrieveBindingFromContainer\s*=\s*0x17865C0[\s\S]*kFindBoneWithName\s*=\s*0x190A580' `
    'The proof must remain pinned to the independently verified FO4VR graph, direct idle-resource, binding, and skeleton functions.'
Require-Text $source `
    'RUNTIME_VR_1_2_72[\s\S]*validateNativeEntry\([\s\S]*"SimpleAnimationGraphManagerHolder::ctor"[\s\S]*validateNativeEntry\("GetClipGeneratorBinding"' `
    'Hardcoded FO4VR calls must retain executable identity and live-byte gates.'
Require-Text $source `
    'validateNativeEntry\(\s*"SimpleAnimationGraphManagerHolder::ctor",\s*kSimpleAnimationGraphManagerHolderCtor,\s*std::array<std::uint8_t,\s*6>\{\s*0x40,\s*0x53,\s*0x48,\s*0x83,\s*0xEC,\s*0x20\s*\}\)' `
    'The plain-holder constructor gate must retain the verified redundant REX prefix present in Fallout4VR.exe 1.2.72.'
Require-Text $source `
    'validateNativeEntry\("SimpleAnimationGraphManagerHolder::IsAnimationLoadingComplete",\s*kIsAnimationLoadingComplete,\s*std::array<std::uint8_t,\s*9>\{\s*0x48,\s*0x8B,\s*0x41,\s*0x10,\s*0x48,\s*0x85,\s*0xC0,\s*0x74,\s*0x0C\s*\}\)' `
    'The plain-holder completion poll must retain its verified Fallout4VR.exe 1.2.72 byte gate.'
Require-Text $source `
    'validateNativeEntry\("LoadIdle",\s*kLoadIdleAnimationResource[\s\S]*validateNativeEntry\("BShkbHkxDB resource-handle move assignment",\s*kMoveAnimationResourceHandle[\s\S]*validateNativeEntry\("BShkbHkxDBUtils::IsHkxDerivativeDBData",\s*kIsHkxDerivativeDbData[\s\S]*validateNativeEntry\("BShkbUtils::RetrieveBindingFromContainer",\s*kRetrieveBindingFromContainer' `
    'Every direct idle-resource call must retain its independently verified Fallout4VR.exe 1.2.72 live-byte gate.'
Require-Text $policy `
    'kFirstPersonGraphIndex\s*=\s*1[\s\S]*graphCount\s*<=\s*kFirstPersonGraphIndex[\s\S]*identifierCount\s*<=\s*kFirstPersonGraphIndex' `
    'The sampler must fail closed unless Bethesda produced the paired first-person graph and identifier.'
Require-Text $source `
    'findBoneWithName\(skeleton,\s*"Weapon"[\s\S]*findBoneWithName\(skeleton,\s*"RArm_Hand"[\s\S]*WPNIdleReady[\s\S]*WPNIdle' `
    'The proof must sample authored idle clips and resolve the exact Weapon/RArm_Hand skeleton relation.'
Require-Text $source `
    'kAnimationTransformTrackCountOffset\s*=\s*0x18' `
    'The sampler must read hkaAnimation transform-track count at the two-witness FO4VR offset, not reinterpret duration as an integer.'
Reject-Text $source `
    'kAnimationTransformTrackCountOffset\s*=\s*0x14' `
    'hkaAnimation +0x14 is the float duration and must never return as the transform-track count.'
Require-Text $source `
    'enum class IdleGripExtractionFailure[\s\S]*ClipBindingUnavailable[\s\S]*WeaponNotDirectChildOfHand[\s\S]*WeaponTrackUnavailable[\s\S]*SampledWeaponTransformInvalid' `
    'Idle pose extraction must retain precise fail-closed stage identities instead of collapsing every unavailable datum.'
Require-Text $source `
    'IdleGripExtractionDiagnostics[\s\S]*idlePathMatchCount[\s\S]*sampleAttemptCount[\s\S]*weaponParentIndex[\s\S]*trySampleClip[\s\S]*failExtraction' `
    'The sampler must collect bounded runtime evidence for clip, skeleton, topology, mapping, and transform failures.'
Require-Text $source `
    'extractionFailureName\(extractionDiagnostics\.failure\)[\s\S]*Native idle-grip preharvest extraction detail[\s\S]*failJob\(state,\s*failure\)' `
    'A failed extraction must emit its one-shot evidence and preserve the exact stage in the terminal job reason.'
Reject-Text $source `
    'firstPersonIdleWeaponTrackUnavailable' `
    'The obsolete catch-all idle extraction reason must not hide the exact native datum that failed.'
Require-Text $source `
    'kAnimationFileLookupSingleton\s*=\s*0x5B64318[\s\S]*tryReadValue\([\s\S]*lookupSingleton[\s\S]*getAnimationFilesForSubgraph\(\s*&outSubgraphIdentifier\)[\s\S]*clipPathHasStem[\s\S]*trySampleClip' `
    'Clip binding must consume the exact winning AnimationFileData path for the selected subgraph instead of guessing a basename.'
Require-Text $source `
    'PopulateGraphProjectsToLoad[\s\S]*graphProjects\.size\(\)\s*<\s*2[\s\S]*createBackgroundSimpleManager' `
    'Off-screen native loads must create the plain holder from both player graph projects.'
Require-Text $source `
    'kSubgraphOutputInlineCapacity\s*=\s*2[\s\S]*kSmallArrayInlineStorageOffset\s*=\s*0x8[\s\S]*prepareNativeSubgraphOutput[\s\S]*output\.reserve\(kSubgraphOutputInlineCapacity\)[\s\S]*output\.capacity\(\)\s*==\s*kSubgraphOutputInlineCapacity[\s\S]*output\.data\(\)[\s\S]*expectedInlineData' `
    'Native subgraph outputs must repair and verify CommonLibF4VR small-array inline storage before crossing the ABI.'
Require-Text $source `
    'state\.job\s*=\s*std::move\(candidate\)[\s\S]*prepareNativeSubgraphOutput\(state\.job\.subgraphHandles\)[\s\S]*prepareNativeSubgraphOutput\(state\.job\.subgraphIdentifiers\)[\s\S]*graphHolderCtor' `
    'Both native output arrays must be prepared in their final stable Job storage before background loading starts.'
Require-Text $source `
    'Phase::BaseGraphsLoading[\s\S]*isAnimationLoadingComplete[\s\S]*requestAnimationSubGraph[\s\S]*Phase::WeaponSubgraphLoading[\s\S]*isAnimationSubGraphLoaded' `
    'Off-screen native loads must poll both asynchronous load stages before sampling.'
Require-Text $source `
    'getClipGeneratorBinding[\s\S]*loadIdleAnimationResource[\s\S]*Phase::IdleClipLoading[\s\S]*animationResourceCanExposeData[\s\S]*retrieveBindingFromContainer[\s\S]*trySampleAnimationBinding' `
    'An idle absent from the behavior clip map must use Bethesda''s retained asynchronous HKX resource path and the same frame-zero sampler.'
Require-Text $source `
    'idleClipResource\.entry[\s\S]*moveAnimationResourceHandle\(&job\.idleClipResource,\s*&empty\)[\s\S]*releaseAnimationSubGraph[\s\S]*graphHolderDtor' `
    'The retained idle HKX must be released through Bethesda''s BShkbHkxDB handle operation before graph teardown.'
Require-Text $source `
    'releaseAnimationSubGraph\([\s\S]*graphHolderDtor' `
    'Weapon subgraphs must be released before destroying the plain graph holder.'
Reject-Text $source `
    'ExtraAnimGraphPreload|kLoadAnimGraphs|isFinishedLoading' `
    'The actor-bound ExtraAnimGraphPreload completion path must not return after its verified FO4VR crash.'
Require-Text $source `
    'rockAuthoredPrimaryFiringGripTestEnabled' `
    'The proof must remain behind the existing ROCK authored-grip experiment.'
Require-Text $source `
    'authored_weapon_grip_library::publish' `
    'The proof must publish only through ROCK''s bounded authored-grip cache.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'NativeIdleGripPreharvest\.h[\s\S]*nativeIdleGripCandidate[\s\S]*hand\.isHoldingLooseWeapon\(\)[\s\S]*hand\.hasSelection\(\)[\s\S]*native_idle_grip_preharvest::observeCandidate\(nativeIdleGripCandidate\)[\s\S]*rockGripZoneHoverHapticsEnabled' `
    'The frame owner must offer held or raw selected weapons before input commit, independently of optional hover haptics.'
Reject-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'native_idle_grip_preharvest::observeCandidate' `
    'Native preharvest scheduling must not become coupled to grip-zone projection or hover-haptic feature gates again.'
Reject-Text $source `
    'PAPER|PAPERRedux|PAPER_Redux' `
    'The narrow ROCK proof must not acquire a PAPER dependency.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native idle-grip preharvest source boundaries passed.'

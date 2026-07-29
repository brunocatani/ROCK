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
    'kFirstPersonGraphIndex\s*=\s*1[\s\S]*graphCount\s*<=\s*kFirstPersonGraphIndex[\s\S]*handleCount\s*<=\s*kFirstPersonGraphIndex[\s\S]*identifierCount\s*<=\s*kFirstPersonGraphIndex' `
    'The sampler must fail closed unless Bethesda produced the paired first-person graph, handle, and identifier.'
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
    'enum class IdleGripExtractionFailure[\s\S]*FirstPersonSubgraphHandleUnavailable[\s\S]*ClipBindingUnavailable[\s\S]*WeaponNotDirectChildOfHand[\s\S]*WeaponTrackUnavailable[\s\S]*SampledWeaponTransformInvalid[\s\S]*IncompleteFiringFingerPose' `
    'Idle pose extraction must retain precise fail-closed stage identities instead of collapsing every unavailable datum.'
Require-Text $source `
    'IdleGripExtractionDiagnostics[\s\S]{0,1800}idlePathMatchCount[\s\S]{0,500}sampleAttemptCount[\s\S]{0,500}graphHandleMatchCount[\s\S]{0,500}subgraphHandle[\s\S]{0,500}bindingSubgraphIdentifier[\s\S]{0,1200}weaponParentIndex' `
    'The sampler must collect bounded runtime evidence for clip, skeleton, topology, mapping, and transform failures.'
Require-Text $source `
    'trySampleClip[\s\S]*failExtraction' `
    'Every clip sampling failure must retain a precise extraction identity.'
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
    'kGraphLoadedSubgraphsOffset\s*=\s*0x3A0[\s\S]*kLoadedSubgraphEntryStride\s*=\s*0x48[\s\S]*kLoadedSubgraphHandleOffset\s*=\s*0x00[\s\S]*kBindingTableSubgraphIdentifierOffset\s*=\s*0xC0[\s\S]*kBindingTableNodeStride\s*=\s*0x18' `
    'The malformed-AnimationFileData fallback must retain the audited loaded-subgraph and clip-map layout.'
Require-Text $source `
    'tryFindLoadedGraphIdlePath[\s\S]*subgraphHandle[\s\S]*BSAutoLock<RE::BSSpinLock>[\s\S]*candidateHandle\s*!=\s*subgraphHandle[\s\S]*outBindingSubgraphIdentifier\s*=\s*candidateIdentifier[\s\S]*idleClipPriority[\s\S]*sameClipPath' `
    'The fallback must select only the native handle-owned loaded entry, preserve idle-path priority, and fail closed on ambiguous preferred paths.'
Require-Text $source `
    'const auto tryGraphPathFallback[\s\S]{0,1800}trySampleClip\(state,\s*graph,\s*bindingSubgraphIdentifier[\s\S]{0,500}animationFiles->empty\(\)[\s\S]{0,300}tryGraphPathFallback\(IdleGripExtractionFailure::AnimationFileListEmpty\)' `
    'An absent numeric AnimationFileData record must recover only a path owned by the selected native handle and bind through that table''s actual identifier.'
Reject-Text $source `
    '35006BE1|Actors\\\\AKsAR15s\\\\Character\\\\_1stPerson\\\\Animations\\\\SVD' `
    'The generic graph-path fallback must never hardcode the observed SVD form or asset path.'
Require-Text $source `
    'convertHavokLocalTransform[\s\S]*outLocal\.rotate\s*=\s*transform_math::transposeRotation\(transform_math::havokQuaternionToNiRows<RE::NiMatrix3>\(sampledLocal\.rotation\)\)[\s\S]*convertWeaponTrackToHandInWeapon[\s\S]*outHandInWeapon\s*=\s*transform_math::invertTransform\(weaponLocal\)' `
    'Sampled Havok rotation must enter ROCK stored-axis convention before inversion so rotation and translation are corrected together.'
Reject-Text $source `
    'outLocal\.rotate\s*=\s*transform_math::havokQuaternionToNiRows<RE::NiMatrix3>\(sampledLocal\.rotation\)\s*;' `
    'The measured transpose error must not return at the sampled-animation boundary.'
Require-Text $source `
    'kSkeletonReferencePoseOffset\s*=\s*0x38[\s\S]*kSkeletonReferencePoseCountOffset\s*=\s*0x40[\s\S]*"RArm_Finger11"[\s\S]*"RArm_Finger53"[\s\S]*extractRightFiringFingerPose[\s\S]*findTransformTrackForBone[\s\S]*guardedCopyFromMemory\(referencePose\s*\+\s*boneIndex[\s\S]*convertHavokLocalTransform' `
    'The sampler must resolve all 15 firing fingers as local tracks and use the verified hkaSkeleton reference pose only for compressed-out tracks.'
Require-Text $source `
    'sampledFingerMask[\s\S]*referenceFingerMask[\s\S]*missingFingerMask[\s\S]*outRightFiringFingerPose\.complete\(\)[\s\S]*IncompleteFiringFingerPose[\s\S]*CaptureSource::NativeIdlePreharvest,\s*&rightFiringFingerPose' `
    'Only a complete finite 15-bone pose may become an authoritative harvested grip publication; partial poses must fail extraction.'
Require-Text 'src/physics-interaction/weapon/AuthoredWeaponGripLibrary.cpp' `
    'publicationHasRequiredFingerPose\(source\s*==\s*CaptureSource::NativeIdlePreharvest,\s*validFingerPose\)' `
    'The authored-grip cache itself must reject a native-idle authority entry without a complete finite finger pose.'
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
Reject-Text $source `
    'rockAuthoredPrimaryFiringGripTestEnabled|experimentNoLongerEligible' `
    'Native idle-grip preharvest must be a production ROCK path, not an experimental config branch.'
Reject-Text $source `
    'f4vr::isLeftHandedMode\(\)|authoredGripNoLongerEligible' `
    'Native idle-grip preharvest must use ROCK''s fixed physical-right canonical topology, never Fallout 4 VR handedness.'
Require-Text $source `
    'authored_weapon_grip_library::publishResolvedVariant[\s\S]{0,300}CaptureSource::NativeIdlePreharvest' `
    'The proof must publish only through ROCK''s bounded authored-grip cache with explicit preharvest provenance.'
Require-Text $source `
    'shouldStartNativeIdleHarvest\([\s\S]*existing\.found[\s\S]*existing\.source\s*==\s*authored_weapon_grip_library::CaptureSource::NativeIdlePreharvest[\s\S]*existing\.usedVariantFallback[\s\S]*candidate\.variant\.key' `
    'A prior live fallback or a different resolved stock variant must not suppress the exact native-idle harvest.'
Require-Text $source `
    'describeEquippedCandidate[\s\S]*candidate\.instanceData\s*=\s*RE::BSTSmartPointer<RE::TBO_InstanceData>\(instanceData\)[\s\S]*identifyWeaponVariant\(weaponRoot\)[\s\S]*CandidateOrigin::EquippedWeapon' `
    'Direct inventory equip must capture stable instance data and a value-only variant identity without retaining the scene node.'
Require-Text $source `
    'void\s+observeCandidate\(RE::NiPointer<RE::TESObjectREFR>\s+candidate\)[\s\S]{0,700}advanceAndCanStart\(state\)[\s\S]{0,300}candidate\.get\(\)[\s\S]{0,220}RE::NiPointer<RE::NiAVObject>\s+weaponRoot' `
    'Loose preharvest must own the candidate across native job progress and retain the resolved scene root while copying its variant.'
Reject-Text $source `
    'void\s+observeCandidate\(RE::TESObjectREFR\*\s*candidate\)' `
    'The preharvest boundary must never regress to a raw loose-reference parameter.'
Require-Text $source `
    'publishResolvedVariant\(job\.weapon,\s*job\.variant[\s\S]*releaseJob\(state\)' `
    'An asynchronous harvest must publish from its captured variant value even after the originating loose or equipped scene node disappears.'
Reject-Text $source `
    'looseReferenceUnavailableAtPublish' `
    'Successful asynchronous extraction must not depend on the loose reference surviving until publication.'
Require-Text $source `
    'kAnimationTypeOffset\s*=\s*0x10[\s\S]*kAnimationDurationOffset\s*=\s*0x14[\s\S]*kAnimationTransformTrackCountOffset\s*=\s*0x18[\s\S]*kAnimationFloatTrackCountOffset\s*=\s*0x1C[\s\S]*kBindingBlendHintOffset\s*=\s*0x50' `
    'The sampler diagnostics must retain the audited FO4VR hkaAnimation and hkaAnimationBinding field layout.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'NativeIdleGripPreharvest\.h[\s\S]*RE::NiPointer<RE::TESObjectREFR>\s+nativeIdleGripCandidate[\s\S]*hand\.getSavedObjectState\(\)\.retainedRef[\s\S]*hand\.getSelection\(\)\.retainedRef[\s\S]*native_idle_grip_preharvest::observeCandidate\(std::move\(nativeIdleGripCandidate\)\)[\s\S]*gripZoneHoverHapticsEnabled' `
    'The frame owner must offer retained held or selected weapons before input commit, independently of optional hover haptics.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'weaponGenerationKey[\s\S]*equippedGenerationMatchesForm[\s\S]*getCurrentObservedEquippedWeaponFormID\(\)\s*==\s*equippedWeapon->formID[\s\S]*native_idle_grip_preharvest::observeEquippedWeapon\([\s\S]*equippedWeapon[\s\S]*weaponNode[\s\S]*currentEquippedWeaponInstanceData\(equippedWeapon\)[\s\S]*_authoredPrimaryFiringGrip\.update' `
    'A stable directly equipped weapon must enter preharvest before authored pose lookup, without pairing the new form with a stale scene generation.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'observedFormID[\s\S]*getEquippedWeaponIdentityKey\([\s\S]*&observedFormID[\s\S]*_observedEquippedWeaponFormID\s*=\s*observedFormID[\s\S]*outFormID[\s\S]*\*outFormID\s*=\s*identity\.formID' `
    'The collision observer must publish the form ID paired with its stable generation witness.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'incomingRightFingerMask[\s\S]*incomingLeftFingerMask[\s\S]*fingerPoseBoundary[\s\S]*_rightFiringFingerLocalTransformMask\s*!=\s*incomingRightFingerMask[\s\S]*_leftFiringFingerLocalTransformMask\s*!=\s*incomingLeftFingerMask[\s\S]*sourceBoundary[\s\S]*fingerPoseBoundary' `
    'Canonical pose diagnostics must expose the live-fallback to exact-finger-pose boundary without relying on a source-name change.'
Reject-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'native_idle_grip_preharvest::observeCandidate' `
    'Native preharvest scheduling must not become coupled to grip-zone projection or hover-haptic feature gates again.'
Reject-Text $source `
    'PAPER' `
    'The narrow ROCK proof must not acquire a PAPER dependency.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Native idle-grip preharvest source boundaries passed.'

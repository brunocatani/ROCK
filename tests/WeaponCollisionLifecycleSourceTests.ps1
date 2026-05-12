param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-TextInTree {
    param(
        [string]$Path,
        [string[]]$Include,
        [string]$Pattern,
        [string]$Message,
        [string[]]$Exclude = @()
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $extensions = $Include | ForEach-Object { $_.Replace('*', '') }
    $rootFullPath = (Resolve-Path -LiteralPath $Root).Path
    foreach ($file in Get-ChildItem -LiteralPath $fullPath -Recurse -File) {
        if ($extensions -notcontains $file.Extension) {
            continue
        }

        $relative = $file.FullName
        if ($relative.StartsWith($rootFullPath, [System.StringComparison]::OrdinalIgnoreCase)) {
            $relative = $relative.Substring($rootFullPath.Length).TrimStart(
                [System.IO.Path]::DirectorySeparatorChar,
                [System.IO.Path]::AltDirectorySeparatorChar)
        }
        $normalizedRelative = $relative.Replace('\', '/')
        if ($Exclude -contains $normalizedRelative) {
            continue
        }

        $text = Get-Content -Raw -LiteralPath $file.FullName
        if ($text -match $Pattern) {
            $failures.Add("$Message Found in $relative.")
        }
    }
}

Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'struct GeneratedWeaponBodyCreateOptions' 'Weapon body creation must separate native creation, collision activation, and publication choices.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'collisionEnabledOnCreate' 'Weapon body creation options must name the initial filter state explicitly.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'publishAfterCreate' 'Weapon body creation must not expose publication as a creation-time option.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'createGeneratedWeaponBodiesInBank\([^\)]*bool publishAfterCreate' 'Weapon body creation must not use a boolean that couples publishing with collision filter activation.'

Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'disableExistingBodies = input\.hasExistingBodies' 'Missing visual bodies must disable immediately when their visual drive root is absent.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'disableWeaponBodiesForMissingVisual\(world\);' 'Missing visual disable must route through the explicit disable/unpublish lifecycle.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'disableWeaponBodiesForMissingVisual[\s\S]{0,260}unpublishAtomicBodyIds\(\)' 'Disabled cached weapon bodies must not remain published as active provider bodies.'

Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'publishAtomicBodyIds\(activeWeaponBodies\(\)\);\s*setWeaponBodyBankCollisionEnabled\(world,\s*activeWeaponBodies\(\),\s*true\);' 'Generated weapon bodies must publish metadata before enabling collision.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'makeGeneratedSourcePendingSettleKey' 'Pending generated source settling must use a structural owner/settings-aware settle key.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'makeGeneratedSourcePendingSettleKey' 'Pending generated source settling must not reuse volatile replacement geometry keys for owner/settings changes.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'smallSourceSet[\s\S]{0,260}requestPendingGeneratedRebuildAttemptNextFrame\(\);' 'Pending generated source settling may only request an immediate retry for bounded small source sets.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'sourceSetSettling[\s\S]{0,900}_pendingGeneratedWeaponVisualWitness = observedGeneratedVisualWitness;\s*requestPendingGeneratedRebuildAttemptNextFrame\(\);' 'Pending generated source settling must not force every source set into a next-frame rebuild loop.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'requestGeneratedSourceCompletenessProbeNextFrame\(\);' 'Late-source enrichment must request an explicit next-frame probe for improved-but-settling source sets.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Update1StPersonArm\(player,\s*&weaponNode,\s*&offsetNode\)' 'Generated weapon collision must not call the FO4VR first-person arm refresh as a source-generation retry.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'deferGenerationForWeaponVisualRefresh' 'Initial pending weapon collision generation must scan the settled candidate trees directly, not defer through a visual-refresh state machine.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'advanceWeaponVisualRefreshFrame\(\);' 'Generated weapon collision must not retain visual-refresh cooldown advancement.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Weapon visual refresh cooldown before generated collision scan' 'Pending generation must not wait through an engine visual-refresh cooldown.'
$forbiddenWeaponCollisionNativeCalls = '\b(?:AttachModToReference|AttachWeapon|QueueAttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson)\b'
$forbiddenWeaponModuleNativeCalls = '\b(?:AttachModToReference|AttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|Update1StPersonArm)\b'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' $forbiddenWeaponCollisionNativeCalls 'Weapon visual desync handling must call only the named visual remap wrapper, not broad or mutating native refresh/equip paths.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' $forbiddenWeaponCollisionNativeCalls 'Weapon collision headers must not expose broad or mutating native refresh/equip paths.'
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') $forbiddenWeaponModuleNativeCalls 'Weapon module must not call broad or mutating native refresh/equip paths.' @(
    'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp',
    'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h',
    'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp',
    'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h'
)
Reject-TextInTree 'src/physics-interaction/weapon' @('*.cpp', '*.h') '\bQueueAttachWeapon\b' 'QueueAttachWeapon is allowed only inside WeaponVisualRemapRuntime.' @(
    'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp',
    'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h'
)
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' $forbiddenWeaponModuleNativeCalls 'Weapon visual remap runtime must use only the verified queued weapon attach task.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h' $forbiddenWeaponModuleNativeCalls 'Weapon visual remap runtime header must not expose broad or mutating native refresh/equip paths.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h' '\bQueueAttachWeapon\b' 'Weapon visual remap runtime header must not expose the native task name or call surface.'
Reject-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' '\b(?:AttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|Update1StPersonArm|QueueAttachWeapon)\b' 'Weapon instance witness runtime may observe equip and attach-mod signals but must not queue or force native visual/equip refresh paths.'
Reject-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' '\b(?:AttachWeapon|HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|Update1StPersonArm|QueueAttachWeapon)\b' 'Weapon instance witness runtime header must not expose native visual/equip refresh paths.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'TESEquipEvent' 'Weapon instance witness runtime must listen for player inventory/Pip-Boy weapon equips.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'REL::RelocationID\(3303,\s*2189033\)' 'Weapon instance witness runtime must use the verified FO4VR AttachModToReference relocation.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'AttachModToReferencePostCall' 'Weapon instance witness runtime may keep attach-mod evidence only as a secondary mutation hint.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'RE::VTABLE::__ApplyChangesFunctor\[0\]' 'Workbench weapon mod commits must be witnessed by patching the ApplyChangesFunctor vtable.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'REL::Offset\(0xB53B70\)' 'Workbench ApplyChanges vtable hook must validate the FO4VR WriteDataImpl target before patching.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'expected WriteDataImpl' 'Workbench ApplyChanges hook install must fail closed if the native vtable entry changed.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'original\(functor,\s*baseObj,\s*stack\);\s*recordWorkbenchApplyChangesWitness' 'Workbench ApplyChanges witness must be captured only after the native stack mutation returns.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'RE::VTABLE::BGSInventoryItem__ModifyModDataFunctor\[0\]' 'Workbench weapon mod commits observed through ModifyModDataFunctor must be witnessed by patching its vtable.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'original\(functor,\s*baseObj,\s*stack\);\s*recordWorkbenchModifyModDataWitness' 'Workbench ModifyModData witness must be captured only after the native stack mutation returns.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'ModifyModDataFunctorWriteDataImplPostCall' 'Workbench ModifyModData witness logs must identify the post-call stack-writer boundary.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'WeaponInstanceWitnessSource::WorkbenchApplyChanges' 'Workbench ApplyChanges must be a distinct authoritative witness source.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'return "workbenchApplyChanges"' 'Workbench ApplyChanges logs must expose source=workbenchApplyChanges.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'scanEquippedWeaponInventoryStack' 'Weapon instance witness runtime must derive evidence from the equipped player inventory stack.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'objectInstanceExtraHasActiveMod' 'Attach-mod witness capture must bind to the equipped stack that contains the mutated OMOD evidence.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'modFormID\s*=\s*mod\.GetFormID\(\)' 'Attach-mod witness signals must carry the native mutated OMOD form instead of scanning the first equipped weapon stack.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'result\s*&&\s*modFormID\s*!=\s*0' 'Attach-mod witness signals must reject zero OMOD evidence instead of matching any equipped stack.'
Reject-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'result\s*&&\s*isPlayerReference\(&ref\)' 'Attach-mod witness capture must not require the native reference argument to be the player; equipped-stack OMOD binding is the authority.'
Reject-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'recordPendingSignal\(\s*WeaponInstanceWitnessSource::AttachMod,\s*0,\s*0,\s*"AttachModToReferencePostCall"' 'Attach-mod witness signals must not be recorded with no form, unique, or OMOD evidence.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'mutatingModRemoved' 'Workbench witness state must preserve whether the transaction removed an OMOD.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'active\.source == WeaponInstanceWitnessSource::WorkbenchApplyChanges && active\.mutatingModRemoved' 'Removed-mod workbench transactions must not require the changed OMOD to remain active during witness revalidation.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'equippedDataAddress' 'Authoritative equipped weapon witnesses must preserve the actual equipped stack extra-list address.'
Reject-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'identity\.equippedDataAddress\s*=\s*witness\.stackIndex' 'Authoritative equipped weapon identity must not overload equipped-data address with an inventory stack index.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'kPendingWitnessScanRetryInterval' 'Pending witness retries must be bounded by time and not consumed by repeated reads in one frame.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'kActiveWitnessLifetime' 'Authoritative equipped weapon witnesses must expire after a bounded remap window.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'clearAuthoritativeEquippedWeaponWitness' 'Weapon collision must be able to clear an authoritative witness once visible geometry catches up or pending state is cancelled.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'nativeVisualRemapAllowedForWitness' 'Native visual remap must be gated by authoritative witness source, not only by a stale visual source.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'nativeRemapBlockedInventoryEquip' 'Pip-Boy inventory equips must be observed for collision identity but blocked from QueueAttachWeapon remap.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'nativeRemapBlockedAttachModSecondary' 'AttachModToReference evidence must not authorize native weapon visual remap.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.cpp' 'nativeRemapAllowedWorkbenchApplyChanges' 'Only direct post-mutation workbench stack witnesses may authorize native weapon visual remap.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'noteGameLoadWeaponWitnessBaseline' 'Weapon instance witness runtime must expose a game-load baseline trigger.'
Require-Text 'src/physics-interaction/weapon/WeaponInstanceWitnessRuntime.h' 'tryGetAuthoritativeEquippedWeaponIdentity' 'Weapon collision must consume authoritative equipped-instance evidence through a named wrapper.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'REL::ID\(916430\)' 'Weapon visual remap runtime must resolve the verified FO4VR QueueAttachWeapon relocation ID.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'QueueAttachWeapon' 'Weapon visual remap runtime must document the only allowed native remap path.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'evaluateNativeVisualRemapTargetMatch' 'Weapon visual remap runtime must verify the equipped target before queueing the native remap.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h' 'expectedObjectInstanceExtraAddress' 'Weapon visual remap wrapper must carry the pending object-instance extra witness.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'tryGetAuthoritativeEquippedWeaponWitness' 'Weapon visual remap target acquisition must consider the externally witnessed equipped inventory stack.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'authoritativeFallbackRequiresLiveEquipSnapshot' 'Authoritative remap fallback must require live current equip state before QueueAttachWeapon can run.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'findLiveCompatibleEquipIndex' 'Authoritative remap fallback must reuse a live compatible equip index instead of defaulting to slot zero.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'tryGetAuthoritativeEquippedWeaponIdentity' 'Weapon collision equipped-instance keys must prefer the authoritative inventory witness when active.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'clearAuthoritativeWitnessForSettledInstance' 'Weapon collision must clear bounded authoritative witnesses when pending visual work settles.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'nativeVisualRemapAllowedForWitness' 'Weapon collision must block native remap requests that do not come from a direct post-mutation workbench witness.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'authorizedWitness = nativeVisualRemapAuthorized' 'Weapon collision native remap policy must receive explicit source authorization.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'staleVisualDecision\.staleVisualSource[\s\S]{0,520}_generatedSourceSettleState = \{\};' 'Repeated stale-visible observations must not reset generated source settling state.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'requestCurrentFirstPersonWeaponVisualRemap' 'Weapon collision stale-visible-source handling must route native remaps through the named wrapper.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'requestNativeVisualRemapForWitness' 'Weapon collision must use one wrapper call path for stale visible remap requests.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Queued native weapon visual remap for missing weapon visual node' 'Missing first-person weapon visuals must not queue native remaps during Pip-Boy equip/draw attach transitions.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Native weapon visual remap request failed for missing weapon visual node' 'Missing first-person weapon visuals must not call native remap acquisition.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'missingVisualBlocked' 'Missing first-person weapon visual logs must show native remap is blocked, not requested.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'nativeRemap=\{\} nativeDetail=\{\}' 'Weapon visual-missing logs must expose whether native remap was requested.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '_lastNativeVisualRemapRequestedInstanceSignature' 'Weapon collision stale-visual handling must track one native remap request per pending witness.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'evaluateMissingVisualNativeRemapWitness' 'Missing first-person weapon visuals must not be converted into native remap eligibility.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'observeNativeVisualRemapStillStale' 'Native visual remap retry state must be driven by continued stale visual evidence.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'nativeVisualRemapAlreadyRequestedForInstance' 'Weapon visual remap policy must reject duplicate native remap requests for the same pending witness.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'NativeVisualRemapAttemptState' 'Weapon visual remap policy must track bounded request lifecycle state.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'authorizedWitness' 'Weapon visual remap policy must require an authorized equipped-stack mutation witness before queueing native remap.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'nativeVisualRemapAwaitingAuthorizedWitness' 'Weapon visual remap policy must idle stale visible sources when only non-authoritative evidence exists.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'nativeVisualRemapAwaitingQueuedResult' 'Weapon visual remap policy must give a queued native task an observation window.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'nativeVisualRemapAcquireBackoff' 'Weapon visual remap policy must back off failed target acquisition instead of retrying every frame.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'nativeVisualRemapExhaustedForInstance' 'Weapon visual remap policy must stop bounded retries for one stuck pending witness.'
Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'MissingVisualNativeRemapWitness' 'Native visual remap policy must not expose missing-visual remap eligibility.'
Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'missingVisualAwaitingCachedWitness|missingVisualChangedEquippedWitness|missingVisualCachedEquippedWitness' 'Missing-visual native remap reasons must not remain after Pip-Boy containment.'
Reject-Text 'tests/WeaponCollisionLifecyclePolicyTests.cpp' 'evaluateMissingVisualNativeRemapWitness|MissingVisualNativeRemapWitness' 'Policy tests must not preserve missing-visual native remap eligibility.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'evaluateReturnedCachedVisualPending' 'Weapon lifecycle policy must decide whether a returned cached visual keeps a pending equipped witness.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'sameFormEquippedInstanceRemapWitnessChanged' 'Weapon lifecycle policy must compare native remap witness pointers, not only content signatures.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'sameWeaponRemapWitnessChangedCachedVisualReturned' 'Same-form manual mod edits must keep pending state after the missing visual root returns unchanged.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'differentWeaponReturnedCachedVisual' 'Different-form weapon swaps must not reuse the stale-visible remap path after a cached visual returns.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'keeping pending witness for stale visible remap' 'Returned cached visuals with same-form pending instances must reach the stale-visible remap path.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'remapWitnessChanged' 'Stale visible logs must expose whether native remap witness pointers changed.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'appendEquipDataSnapshots' 'Weapon visual remap runtime must add equip-data fallback candidates when middleHigh is stale.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'appendMiddleHighEquippedSnapshots' 'Weapon visual remap runtime must isolate middleHigh equipped-item scanning behind a guarded helper.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'itemCount == 0 \|\| !items' 'Weapon visual remap runtime must skip transient null middleHigh equipped-item arrays.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'kMaxQueueableEquippedItems' 'Weapon visual remap runtime must bound middleHigh equipped-item scanning.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'firstPersonEquipData' 'Weapon visual remap runtime must consider first-person equip data as a fallback source.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'actorEquipData' 'Weapon visual remap runtime must consider actor equip data as a fallback source.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'Equipped snapshots are deliberately non-owning' 'Weapon visual remap runtime must document why equipped snapshots do not own BGSObjectInstance state.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'struct BorrowedWeaponInstance' 'Weapon visual remap runtime must use a borrowed ABI view for the native queue argument.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'BorrowedWeaponInstance borrowedWeaponInstance\{\s*snapshot->weapon,\s*snapshot->instanceData\s*\}' 'Weapon visual remap runtime must populate the borrowed queue view from the matched witness only.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'std::optional<RE::BGSObjectInstance>' 'Weapon visual remap runtime must not store owning BGSObjectInstance copies in optional snapshot state.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' '\.emplace\(equipped\.item\)' 'Weapon visual remap runtime must not copy middleHigh BGSObjectInstance into remap snapshots.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' '\.emplace\(form,\s*slot\.instanceData\)' 'Weapon visual remap runtime must not synthesize owning BGSObjectInstance values in equip-data fallback snapshots.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'BGSObjectInstanceT<RE::TESObjectWEAP> weaponInstance\(' 'Weapon visual remap runtime must not construct owning BGSObjectInstanceT locals around engine-managed instance data.'
Reject-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.cpp' 'snapshot->item' 'Weapon visual remap runtime snapshots must expose raw witness fields, not copied BGSObjectInstance items.'
Require-Text 'src/physics-interaction/weapon/WeaponVisualRemapRuntime.h' 'sourceSlotIndex' 'Weapon visual remap wrapper must report which equip-data source slot produced the queued candidate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'generatedWeaponSourceMissingRequiredPackageCoverage' 'Firearm source scans must not use required front/rear package coverage as a generation gate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'WeaponVisualRefreshState' 'Generated weapon collision must not retain the failed visual-refresh state machine.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'hasLongGunRearPackageCoverage' 'Long-gun rear coverage must require stock coverage rather than accepting pistol grip as rear structure.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'missingExpectedPackageCoverageMask' 'Generated weapon replacement policy must remember expected rich package coverage and reject regressions.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'Collision generation is visibility-driven' 'Generated weapon replacement policy must document that semantic completeness is telemetry-only.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'ownerUnchangedVisibleGeometryChanged' 'Generated weapon replacement policy must allow same-owner visible geometry changes.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'sourcePackageNeedsMoreCoverage' 'Runtime generated collision must not block or retry visible weapon sources for semantic package coverage.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'missingCoverageMask' 'Runtime generated collision logs must not treat front/rear package coverage as a replacement blocker.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'childBudgetTarget' 'Generated weapon source extraction must not use semantic child budgeting as a generation gate.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'selectSemanticWeaponHullIndices' 'Generated weapon source extraction must not use semantic hull priority when the body cap is reached.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'excludeFromSemanticWeaponCollision\(sourceSemantic\)' 'Generated weapon source extraction must not exclude visible source nodes by semantic class.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Generated weapon collision is now geometry-first' 'Pending generated collision must document the visible-geometry-first source scan.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'InstanceVisualSyncDecision' 'Weapon lifecycle policy must classify equipped-instance changes separately from visual-source changes.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Equipped weapon instance changed but generated visual source is stale' 'Weapon collision must log the instance/visual desync instead of rebuilding from stale visible geometry.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'candidate roots are[\s\S]{0,120}discovery witnesses' 'Generated weapon source extraction must treat candidate roots as merged witnesses, not competing winners.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'duplicate TriShape already claimed by earlier candidate' 'Merged candidate roots must dedupe overlapping source TriShapes.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'TriShape is hidden or locally zero-scale' 'Hidden TriShapes must be skipped as sources without pruning helper-node children.'
Require-Text 'src/physics-interaction/weapon/WeaponSemantics.h' 'OptimizedTriShape = 3' 'Weapon collision grouping must expose only the OptimizedTriShape mode value.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'selectedRoot' 'Generated weapon source extraction must not select one winning candidate root.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'selectedScore' 'Generated weapon source extraction must not score roots until one candidate wins.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'weaponCollisionCandidateScore' 'Generated weapon source extraction must not use triangle-coverage root scoring.'
Reject-Text 'src/physics-interaction/weapon/WeaponSemantics.h' '\b(?:LegacyTriShape|SemanticPartNode|CompoundSemanticPart|LegacyTriShapeSupportFit)\b' 'Old weapon collision grouping modes must be removed from the grouping policy surface.'
Reject-Text 'src/physics-interaction/weapon/WeaponSemantics.h' '\b(?:WeaponSemanticHullBudgetInput|WeaponSemanticClusterBudgetInput|selectSemanticWeaponHullIndices|selectSemanticWeaponClusterBudgets)\b' 'Dead semantic hull/cluster budget APIs must not remain after geometry-first source extraction.'
Reject-Text 'src/physics-interaction/weapon/WeaponSemantics.h' '\b(?:startsSemanticWeaponPartGroup|semanticChildSplitsFromParent|excludeFromSemanticWeaponCollision|classifyCompoundSemanticWeaponPartName|compoundSemanticChildSplitsFromParent|compoundSemanticPartsMayShareBody)\b' 'Old semantic/compound grouping helpers must not remain after OptimizedTriShape becomes the only mode.'
Reject-Text 'src/physics-interaction/weapon/WeaponSemantics.h' '\b(?:weaponCollisionGroupingModeChanged|usesSingleWeaponPackageDriveRoot)\b' 'Dead grouping-mode transition helpers must not remain when old modes have no effective behavior.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' '_cachedGroupingMode' 'Weapon collision settings cache must not retain dead grouping-mode state.'
Require-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'policy=visible-traversal-order' 'Generated weapon source overflow must use the hard body cap without semantic priority selection.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'GENERATED_WEAPON_PENDING_REBUILD_INTERVAL_FRAMES\) - 1' 'Pending retry scheduling must not be hidden in interval-minus-one counter state.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'probeInterval > 0 \? probeInterval - 1 : 0' 'Late-source retry scheduling must not be hidden in interval-minus-one counter state.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'totalConvexes == MAX_WEAPON_BODIES\)\s*\{\s*return;' 'Generated weapon source budgeting must not return unchanged at the exact convex cap.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon collision lifecycle source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon collision lifecycle source boundary passed.'

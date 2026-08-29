param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -LiteralPath $path -Raw
    if ($text -notmatch $Pattern) {
        throw $Message
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -LiteralPath $path -Raw
    if ($text -match $Pattern) {
        throw $Message
    }
}

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.h' `
    'enum class GrenadeDetonationMode[\s\S]*TimedFuse[\s\S]*Impact[\s\S]*Proximity[\s\S]*GrenadeRuntimeData[\s\S]*proximityRadiusGameUnits[\s\S]*preserveReferenceAfterDetonation[\s\S]*detonationMode' `
    'Loose throwable runtime data must carry timed-fuse, impact, and proximity behavior.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'isThrowableWeapon[\s\S]*isSupportedWeaponType\([\s\S]*WEAPON_TYPE::kGrenade[\s\S]*WEAPON_TYPE::kMine' `
    'Grenade-mode eligibility must use exact grenade-or-mine enum equality.'

Require-Text 'src/physics-interaction/grenade/LooseThrowablePolicy.h' `
    'classifyDetonationMode[\s\S]*type == mineType[\s\S]*DetonationMode::Proximity[\s\S]*DetonationMode::Impact[\s\S]*molotov \? DetonationMode::Impact : DetonationMode::TimedFuse' `
    'Throwable classification must distinguish placed mines, impact throwables, Molotovs, and timed grenades.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'objectInstanceExtraHasMolotovOmod[\s\S]*GetIndexData\(\)[\s\S]*GetFormByID<RE::BGSMod::Attachment::Mod>[\s\S]*containsMolotovToken\(omod->fullName\.c_str\(\)\)[\s\S]*containsMolotovToken\(omod->model\.c_str\(\)\)' `
    'Molotov classification must inspect active OMOD identity, not only broad WEAPON_TYPE.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'isMolotovGrenade[\s\S]*keywordFormHasMolotovToken\(weapon\)[\s\S]*instanceData->GetKeywordData\(\)[\s\S]*projectile->fullName\.c_str\(\)[\s\S]*objectInstanceExtraHasMolotovOmod' `
    'Molotov classification must use weapon, instance keyword, projectile, and OMOD evidence.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'resolveEquippedGrenadeSelection[\s\S]*BSAutoReadLock[\s\S]*stack->IsEquipped\(\)[\s\S]*equippedGrenadeStackCount\s*!=\s*1[\s\S]*dropEquippedGrenadeSelectionToWorld[\s\S]*findExactInventoryStack[\s\S]*!stack\.equipped[\s\S]*RemoveItemData removeData\(selection\.weapon,\s*1\)' `
    'Quick draw must resolve one native equipped throwable stack and revalidate it before dropping exactly one item.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'kMaximumProximityActorHandlesScanned[\s\S]*scanHostileActorsWithinProximity[\s\S]*GetHostileToActor[\s\S]*highActorHandles[\s\S]*middleHighActorHandles[\s\S]*middleLowActorHandles[\s\S]*lowActorHandles' `
    'Placed mines must use a bounded loaded-actor proximity scan and require hostility.'

Reject-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'hookedEquipObject|installEquipHook|PendingEquipRequest' `
    'Pip-Boy grenade equip must remain native; the retired equip interception must not return.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    '_armedLooseGrenadeImpactBodyIds[\s\S]*_pendingLooseGrenadeImpactPair' `
    'Armed Molotov impact state must cross the physics callback through atomics only.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'looseGrenadeImpactBodyIsWatched[\s\S]*_armedLooseGrenadeImpactBodyIds[\s\S]*recordLooseGrenadeImpactIfArmed[\s\S]*watchedIsHeld[\s\S]*otherIsRightHand[\s\S]*otherIsLeftHand[\s\S]*_pendingLooseGrenadeImpactPair\.store\(packHeldImpactPair' `
    'Physics contact callback must only record released armed Molotov impacts and ignore held/hand contacts.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'updateLooseGrenadeFuses[\s\S]*_pendingLooseGrenadeImpactPair\.exchange[\s\S]*GrenadeDetonationMode::Impact[\s\S]*pendingImpactBodyId\s*!=\s*fuse\.impactBodyId[\s\S]*"impact"[\s\S]*GrenadeDetonationMode::Proximity[\s\S]*scanHostileActorsWithinProximity[\s\S]*"proximity"' `
    'Main update must consume impact contacts and poll placed-mine proximity through distinct activation paths.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'GrenadeDetonationMode::Proximity[\s\S]*_rightHand\.isHolding[\s\S]*!fuse\.releasedSinceArming[\s\S]*proximity arming delay started[\s\S]*scanHostileActorsWithinProximity' `
    'Placed-mine arming delay must begin only after the physical mine is released.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'preserveReferenceAfterDetonation[\s\S]*disableAndDeleteReference' `
    'Recoverable impact throwables must remain in the world after applying their authored impact explosion.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'directImpactDamage[\s\S]*weaponInstanceData\(weapon, instanceData\)->attackDamage[\s\S]*impactPlacedObject' `
    'Projectile-style throwables must retain authored weapon damage and avoid duplicating explosion-spawned recovery objects.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'impactOtherRef == ref \|\| impactOtherRef == player[\s\S]*directImpactDamage[\s\S]*HandleHealthDamage\(player, fuse\.runtime\.directImpactDamage\)' `
    'Impact throwables must ignore self/player contacts and apply authored direct damage to actor impacts.'

Write-Host "Loose grenade runtime source guard passed."

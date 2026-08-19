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
    'enum class GrenadeDetonationMode[\s\S]*TimedFuse[\s\S]*Impact[\s\S]*GrenadeRuntimeData[\s\S]*detonationMode' `
    'Loose grenade runtime data must carry timed-fuse vs impact detonation mode.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'objectInstanceExtraHasMolotovOmod[\s\S]*GetIndexData\(\)[\s\S]*GetFormByID<RE::BGSMod::Attachment::Mod>[\s\S]*containsMolotovToken\(omod->fullName\.c_str\(\)\)[\s\S]*containsMolotovToken\(omod->model\.c_str\(\)\)' `
    'Molotov classification must inspect active OMOD identity, not only broad WEAPON_TYPE.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'isMolotovGrenade[\s\S]*keywordFormHasMolotovToken\(weapon\)[\s\S]*instanceData->GetKeywordData\(\)[\s\S]*projectile->fullName\.c_str\(\)[\s\S]*objectInstanceExtraHasMolotovOmod' `
    'Molotov classification must use weapon, instance keyword, projectile, and OMOD evidence.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'resolveEquippedGrenadeSelection[\s\S]*BSAutoReadLock[\s\S]*stack->IsEquipped\(\)[\s\S]*equippedGrenadeStackCount\s*!=\s*1[\s\S]*dropEquippedGrenadeSelectionToWorld[\s\S]*findExactInventoryStack[\s\S]*!stack\.equipped[\s\S]*RemoveItemData removeData\(selection\.weapon,\s*1\)' `
    'Quick draw must resolve one native equipped stack and revalidate it before dropping exactly one grenade.'

Reject-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'hookedEquipObject|installEquipHook|PendingEquipRequest' `
    'Pip-Boy grenade equip must remain native; the retired equip interception must not return.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    '_armedLooseGrenadeImpactBodyIds[\s\S]*_pendingLooseGrenadeImpactPair' `
    'Armed Molotov impact state must cross the physics callback through atomics only.'


Write-Host "Loose grenade runtime source guard passed."

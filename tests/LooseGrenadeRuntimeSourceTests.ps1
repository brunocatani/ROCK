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

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.h' `
    'enum class GrenadeDetonationMode[\s\S]*TimedFuse[\s\S]*Impact[\s\S]*GrenadeRuntimeData[\s\S]*detonationMode' `
    'Loose grenade runtime data must carry timed-fuse vs impact detonation mode.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'objectInstanceExtraHasMolotovOmod[\s\S]*GetIndexData\(\)[\s\S]*GetFormByID<RE::BGSMod::Attachment::Mod>[\s\S]*containsMolotovToken\(omod->fullName\.c_str\(\)\)[\s\S]*containsMolotovToken\(omod->model\.c_str\(\)\)' `
    'Molotov classification must inspect active OMOD identity, not only broad WEAPON_TYPE.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'isMolotovGrenade[\s\S]*keywordFormHasMolotovToken\(weapon\)[\s\S]*instanceData->GetKeywordData\(\)[\s\S]*projectile->fullName\.c_str\(\)[\s\S]*objectInstanceExtraHasMolotovOmod' `
    'Molotov classification must use weapon, instance keyword, projectile, and OMOD evidence.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    '_armedLooseGrenadeImpactBodyIds[\s\S]*_pendingLooseGrenadeImpactPair' `
    'Armed Molotov impact state must cross the physics callback through atomics only.'

Require-Text 'src/physics-interaction/core/PhysicsInteractionContacts.inl' `
    'looseGrenadeImpactBodyIsWatched[\s\S]*_armedLooseGrenadeImpactBodyIds[\s\S]*recordLooseGrenadeImpactIfArmed[\s\S]*watchedIsHeld[\s\S]*otherIsRightHand[\s\S]*otherIsLeftHand[\s\S]*_pendingLooseGrenadeImpactPair\.store\(packHeldImpactPair' `
    'Physics contact callback must only record released armed Molotov impacts and ignore held/hand contacts.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'updateLooseGrenadeFuses[\s\S]*_pendingLooseGrenadeImpactPair\.exchange[\s\S]*GrenadeDetonationMode::Impact[\s\S]*pendingImpactBodyId\s*!=\s*fuse\.impactBodyId[\s\S]*detonateLooseGrenade\(fuse,\s*slotIndex,\s*ref,\s*"impact"\)' `
    'Main update must consume recorded Molotov impacts and detonate through the normal explosion path.'

Write-Host "Loose grenade runtime source guard passed."

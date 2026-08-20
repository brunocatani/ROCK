param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    if ((Read-Source $RelativePath) -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param([string]$RelativePath, [string]$Pattern, [string]$Message)

    if ((Read-Source $RelativePath) -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Read-IniSection {
    param([string]$RelativePath, [string]$Section)

    $source = Read-Source $RelativePath
    $escapedSection = [Regex]::Escape($Section)
    $match = [Regex]::Match($source, "(?ms)^\[$escapedSection\]\s*(.*?)(?=^\[[^\r\n]+\]|\z)")
    if (-not $match.Success) {
        $failures.Add("$RelativePath`: missing [$Section] section")
        return ''
    }

    return $match.Groups[1].Value
}

Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    'triggerPressedEdge\s*&&\s*input\.triggerInputHand\s*==\s*input\.heldWeaponHand' `
    'Held-weapon trigger equip must require the trigger and held weapon to belong to the same physical hand.'

Require-Text 'src/physics-interaction/input/InputRemapPolicy.h' `
    '\(input\.gripZoneEquipEnabled\s*&&\s*input\.gripZoneEquipSettled\)' `
    'Firing-grip-zone equip must be available to either physical hand without a primary-hand gate.'

foreach ($legacyPath in @(
        'src/RockConfig.cpp',
        'src/RockConfig.h',
        'src/physics-interaction/core/PhysicsInteraction.h',
        'src/physics-interaction/input/InputRemapPolicy.h',
        'data/config/ROCK.ini')) {
    Reject-Text $legacyPath `
        'GrabbedWeaponAutoEquip|HeldWeaponAutoEquip|legacyAutoEquip|autoEquipEnabled|autoEquipSettled|autoEquipState|settled-auto-held-weapon-equip' `
        'The superseded position-blind loose-weapon auto-equip path must remain deleted.'
}

foreach ($configSource in @('src/RockConfig.cpp', 'src/RockConfig.h')) {
    Reject-Text $configSource `
        'rock(?:RealisticWeaponHandlingEnabled|GrabbedWeaponGripZone\w+|WeaponGripHaptic\w+|WeaponFiringGrip\w+|WeaponSupportGripHaptic\w+|GripZoneHover\w+|GrabbedWeaponEquipBridge\w+|MenuTriggerHandEquipEnabled|EquipPreferredHandLeft)' `
        'Base ROCK config must not regain addon-owned realistic detach, grip-zone, haptic, bridge, or Pip-Boy settings.'
}

Require-Text 'src/RockConfig.h' `
    'rockAmbidextrousFiringGripEnabled\s*=\s*true[\s\S]{0,220}rockFiringGripPromotionRadius\s*=\s*5\.0f[\s\S]{0,320}rockLeftFiringAimYawDegrees\s*=\s*0\.0f[\s\S]{0,320}rockLeftFiringAimOffsetZGameUnits\s*=\s*0\.0f' `
    'ROCK must own an enabled standalone ambidextrous baseline and its bounded left-firing tuning.'
Require-Text 'src/RockConfig.cpp' `
    'AMBIDEXTROUS_FIRING_SECTION\s*=\s*"AmbidextrousFiring"[\s\S]*GetBoolValue\(\s*AMBIDEXTROUS_FIRING_SECTION,\s*"bAmbidextrousFiringGripEnabled"[\s\S]*"fFiringGripPromotionRadius"[\s\S]*"fLeftFiringAimOffsetZGameUnits"' `
    'ROCK must load its standalone handoff switch and tuning only from [AmbidextrousFiring].'

foreach ($configPath in @('data/config/ROCK.ini')) {
    $configText = Read-Source $configPath
    $realisticSection = Read-IniSection $configPath 'RealisticWeapons'
    $realisticAssignments = [regex]::Matches($realisticSection, '(?m)^[A-Za-z]\w*\s*=')
    if ($realisticAssignments.Count -ne 1 -or
        $realisticSection -notmatch '(?m)^fRealisticGrenadeFuseSeconds\s*=') {
        $failures.Add("$configPath`: [RealisticWeapons] must retain only ROCK grenade fuse ownership.")
    }
    $handednessSection = Read-IniSection $configPath 'WeaponHandedness'
    if ($handednessSection -notmatch '(?m)^bLeftHandedMode\s*=\s*false\s*$') {
        $failures.Add("$configPath`: [WeaponHandedness] must expose the sole fixed-hand option and default right.")
    }
    $ambidextrousSection = Read-IniSection $configPath 'AmbidextrousFiring'
    $ambidextrousAssignments = [regex]::Matches($ambidextrousSection, '(?m)^[A-Za-z]\w*\s*=')
    if ($ambidextrousAssignments.Count -ne 7 -or
        $ambidextrousSection -notmatch '(?m)^bAmbidextrousFiringGripEnabled\s*=\s*true\s*$' -or
        $ambidextrousSection -notmatch '(?m)^fFiringGripPromotionRadius\s*=\s*5\.0\s*$' -or
        ([regex]::Matches($ambidextrousSection, '(?m)^fLeftFiringAim\w+\s*=\s*0\.0\s*$')).Count -ne 5) {
        $failures.Add("$configPath`: [AmbidextrousFiring] must expose exactly the ROCK baseline switch, promotion radius, and five zeroed left-hold trims.")
    }
    $physicsSection = Read-IniSection $configPath 'PhysicsInteraction'
    if ($physicsSection -notmatch '(?m)^bEquippedWeaponShoulderStashEnabled\s*=\s*true\s*$') {
        $failures.Add("$configPath`: [PhysicsInteraction] must expose ROCK's enabled equipped-weapon shoulder stash switch.")
    }
    if ($configText -match '(?m)^(?:bRealisticWeaponHandlingEnabled|fGrabbedWeaponGripZone\w+|fWeapon(?:Firing|Support)?GripHaptic\w+|bGripZoneHoverHapticsEnabled|bGrabbedWeaponEquipBridgeEnabled|bMenuTriggerHandEquipEnabled)\s*=') {
        $failures.Add("$configPath`: addon-owned realistic detach, grip-zone, haptic, bridge, and Pip-Boy options must remain absent from ROCK.")
    }
}

Require-Text 'src/api/ROCKProviderApi.h' `
    'RockProviderEquippedWeaponHandlingFlagV1[\s\S]*FiringGripOwnership[\s\S]*PrimaryDetach[\s\S]*AmbidextrousHandoff[\s\S]*GripZoneEquip[\s\S]*PipboyTriggerHandEquip[\s\S]*RockProviderEquippedWeaponHandlingRequestV1' `
    'ROCK V1 must expose the complete owner-bound equipped-weapon policy consumed by the addon.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'rolling lease returns to ROCK''s configured fallback handling policy' `
    'The V1 lease contract must describe fallback to ROCK configuration rather than a hard-coded firing hand.'




Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'std::array<std::atomic<bool>,\s*2>\s+s_handHeldWeapon' `
    'Native trigger suppression must track held-weapon ownership for both physical hands.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'eventHandHeldWeapon\s*=\s*s_handHeldWeapon\[eventHandIsLeft\s*\?\s*0u\s*:\s*1u\]' `
    'Native trigger suppression must read held-weapon ownership for the physical hand that emitted the event.'


Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'gripWeaponLocal\s*=\s*transform_math::worldPointToLocal\([\s\S]{0,120}attachedRootWorld,[\s\S]{0,120}canonicalPalmWorld\)[\s\S]{0,900}localPointToWorld\(looseRoot->world,\s*state\.gripWeaponLocal\)' `
    'The hFRIK firing-grip point must be derived from the canonical primary attach pose and projected through the loose weapon, never derived from the tested hand.'

Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'weapon_grip_authority_policy::select\([\s\S]{0,1200}frikCustomFile[\s\S]{0,600}authoredAnimation[\s\S]{0,600}frikEmbeddedResource[\s\S]{0,1600}Source::AuthoredAnimation[\s\S]{0,600}authoredLookup\.rightHandWeaponLocal' `
    'Loose weapons must select explicit hFRIK JSON before ROCK-authored data, then consume the learned Hand-in-Weapon relation directly.'

Reject-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'worldPointToLocal\(attachedRootWorld,\s*palmWorld\)' `
    'The tested palm must not define the firing-grip point.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    'struct\s+HeldWeaponTriggerEquipIntent[\s\S]{0,240}formID[\s\S]{0,160}remainingSeconds' `
    'The retained trigger intent must be identity-bound and time-bounded.'





Require-Text 'src/physics-interaction/object/ObjectDetection.h' `
    'struct\s+SelectedObject[\s\S]{0,500}RE::NiPointer<RE::TESObjectREFR>\s+retainedRef[\s\S]{0,180}RE::TESObjectREFR\*\s+refr[\s\S]*setReference\(RE::TESObjectREFR\*\s+value\)[\s\S]{0,180}retainedRef\.reset\(value\)[\s\S]{0,120}refr\s*=\s*retainedRef\.get\(\)' `
    'A selection that outlives its physics query must retain the TESObjectREFR behind its raw hot-path alias.'

Require-Text 'src/physics-interaction/grab/GrabConstraint.h' `
    'struct\s+SavedObjectState[\s\S]*RE::NiPointer<RE::TESObjectREFR>\s+retainedRef[\s\S]{0,180}RE::TESObjectREFR\*\s+refr[\s\S]*setReference\(const\s+RE::NiPointer<RE::TESObjectREFR>&\s+value\)' `
    'An active grab must retain its world reference instead of relying on the visual model or a raw pointer.'

Require-Text 'src/physics-interaction/hand/Hand.h' `
    'takeRetainedReference\(\)[\s\S]{0,180}refr\s*=\s*nullptr;[\s\S]{0,100}std::move\(retainedRef\)' `
    'Native transfers must be able to consume the release pin while invalidating its raw alias.'


# Pull-catch/force-grab canonical auto-align must cover both physical hands
# through the shared firing-hold resolver.

Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'tryResolveLooseWeaponFiringHandHold\([\s\S]{0,700}tryResolveGripWorld\(isLeft,\s*weaponRef,\s*scratch,\s*&testedHandWorld\)' `
    'The one-shot firing-hold resolver must share the grip-zone projection core so both paths seat identical holds.'

# Non-throwable weapons never participate in saved grab offsets, on either
# side: the shared custom/authored weapon-grip pipeline owns their seat.

if ($failures.Count -gt 0) {
    Write-Host 'Held weapon hand-specific equip source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Held weapon hand-specific equip source boundary passed.'

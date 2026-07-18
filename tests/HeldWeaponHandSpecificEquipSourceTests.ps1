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
        'src/physics-interaction/core/PhysicsInteraction.cpp',
        'src/physics-interaction/core/PhysicsInteraction.h',
        'src/physics-interaction/input/InputRemapPolicy.h',
        'data/config/ROCK.ini',
        'data/mod/ROCK_Config/ROCK.ini')) {
    Reject-Text $legacyPath `
        'GrabbedWeaponAutoEquip|HeldWeaponAutoEquip|legacyAutoEquip|autoEquipEnabled|autoEquipSettled|autoEquipState|settled-auto-held-weapon-equip' `
        'The superseded position-blind loose-weapon auto-equip path must remain deleted.'
}

foreach ($legacyPath in @(
        'src/RockConfig.cpp',
        'src/RockConfig.h',
        'src/physics-interaction/core/PhysicsInteraction.cpp',
        'src/physics-interaction/weapon/WeaponSupport.h',
        'data/config/ROCK.ini',
        'data/mod/ROCK_Config/ROCK.ini')) {
    Reject-Text $legacyPath `
        'bGrabbedWeaponGripZoneEquipEnabled|rockGrabbedWeaponGripZoneEquipEnabled|bWeaponGripHapticsEnabled|rockWeaponGripHapticsEnabled' `
        'Grip-zone equip and weapon-grip haptics must not retain independently configurable Boolean gates.'
}

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $ambidextrousSection = Read-IniSection $configPath 'AmbidextrousFiring'
    foreach ($key in @(
            'bAmbidextrousFiringGripEnabled',
            'fFiringGripPromotionRadius',
            'fLeftFiringAimYawDegrees',
            'fLeftFiringAimPitchDegrees',
            'fLeftFiringAimOffsetXGameUnits',
            'fLeftFiringAimOffsetYGameUnits',
            'fLeftFiringAimOffsetZGameUnits')) {
        if ($ambidextrousSection -notmatch "(?m)^$([Regex]::Escape($key))\s*=") {
            $failures.Add("$configPath`: [$key] must live in [AmbidextrousFiring].")
        }
    }

    $realisticSection = Read-IniSection $configPath 'RealisticWeapons'
    foreach ($key in @(
            'fGrabbedWeaponGripZoneEquipRadius',
            'fGrabbedWeaponGripZoneEquipSettleSeconds',
            'fWeaponGripHapticDurationSeconds',
            'fWeaponFiringGripAttachHapticIntensity',
            'fWeaponFiringGripDetachHapticIntensity',
            'fWeaponSupportGripHapticIntensity')) {
        if ($realisticSection -notmatch "(?m)^$([Regex]::Escape($key))\s*=") {
            $failures.Add("$configPath`: [$key] must remain configurable in [RealisticWeapons].")
        }
    }
    if ($realisticSection -match '(?m)^(bAmbidextrousFiringGripEnabled|fFiringGripPromotionRadius|fLeftFiringAim\w+)\s*=') {
        $failures.Add("$configPath`: ambidextrous firing keys must not remain in [RealisticWeapons].")
    }
}

Require-Text 'src/RockConfig.cpp' `
    'constexpr\s+auto\s+AMBIDEXTROUS_FIRING_SECTION\s*=\s*"AmbidextrousFiring"' `
    'Ambidextrous firing must have an independent INI section.'

Require-Text 'src/RockConfig.cpp' `
    'ini\.GetBoolValue\(\s*AMBIDEXTROUS_FIRING_SECTION,\s*"bAmbidextrousFiringGripEnabled"' `
    'The ambidextrous firing switch must load from its independent INI section.'

foreach ($key in @(
        'fFiringGripPromotionRadius',
        'fLeftFiringAimYawDegrees',
        'fLeftFiringAimPitchDegrees',
        'fLeftFiringAimOffsetXGameUnits',
        'fLeftFiringAimOffsetYGameUnits',
        'fLeftFiringAimOffsetZGameUnits')) {
    Require-Text 'src/RockConfig.cpp' `
        "readClampedFloat\(ini,\s*AMBIDEXTROUS_FIRING_SECTION,\s*`"$key`"" `
        "$key must load from [AmbidextrousFiring]."
}

Reject-Text 'src/RockConfig.cpp' `
    'REALISTIC_WEAPONS_SECTION,\s*"(bAmbidextrousFiringGripEnabled|fFiringGripPromotionRadius|fLeftFiringAim\w+)"' `
    'Ambidextrous firing settings must not retain a hidden [RealisticWeapons] loader path.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'canSettleEquipInGripZone\(\s*bool realisticWeaponHandlingEnabled\)[\s\S]{0,160}return realisticWeaponHandlingEnabled\s*;' `
    'Grip-zone settle equip must follow realistic weapon handling directly, without a second setting.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gripZoneSettleEquipEnabled\s*=\s*[\s\S]{0,180}canSettleEquipInGripZone\(\s*g_rockConfig\.rockRealisticWeaponHandlingEnabled\s*\)' `
    'Runtime grip-zone equip and hover ownership must use realistic handling as their only feature gate.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'consumeHapticEvents\(\);\s*const auto queueGripHaptic' `
    'Valid weapon-grip transitions must queue their configured haptics without an optional Boolean gate.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldStartHeldWeaponEquipOwnership[\s\S]{0,260}\.modes\s*=\s*firingGripModes[\s\S]{0,120}\.handIsLeft\s*=\s*isLeft[\s\S]{0,120}\.gripHeld\s*=\s*rawGrabInput\.held' `
    'Direct trigger equip must independently start left-hand ambidextrous ownership from the originating hand grip.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'EquippedWeaponGripMode\{[\s\S]{0,220}\.firingGripOwnershipEnabled\s*=\s*firingGripOwnershipFeatureAvailable[\s\S]{0,160}\.ambidextrousHandoffEnabled\s*=\s*ambidextrousFiringAvailable[\s\S]{0,160}\.primaryDetachEnabled\s*=\s*primaryDetachFeatureAvailable' `
    'Two-handed weapon state must receive independent ownership, handoff, and realistic-detach gates.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'updatePrimaryOnlyGrip[\s\S]{0,1000}primaryGripRetained\s*=\s*equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership\(\s*primaryDetachEnabled,\s*primaryGripInput\.held\)' `
    'Ambidextrous-only firing must ignore grip release while still running equipped-weapon identity cleanup.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'std::array<std::atomic<bool>,\s*2>\s+s_handHeldWeapon' `
    'Native trigger suppression must track held-weapon ownership for both physical hands.'

Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    'eventHandHeldWeapon\s*=\s*s_handHeldWeapon\[eventHandIsLeft\s*\?\s*0u\s*:\s*1u\]' `
    'Native trigger suppression must read held-weapon ownership for the physical hand that emitted the event.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'loose_weapon_grip_zone::tryGetFiringHandWeaponLocal\([\s\S]{0,180}pendingGripStart\.firingHandWeaponLocal' `
    'Loose equip must carry the canonical weapon-relative firing-hand frame across inventory transfer.'

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

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'triggerEquipIntent\s*=\s*HeldWeaponTriggerEquipIntent\{[\s\S]{0,220}\.formID\s*=\s*selectedRef->GetFormID\(\)[\s\S]{0,120}\.remainingSeconds\s*=\s*0\.35f' `
    'A same-hand trigger edge coincident with grab commit must be retained for that exact weapon instead of being lost.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'replayedSameHandTrigger\s*=\s*triggerEquipIntent\.pending\s*&&\s*heldRefForGameplay\s*&&\s*triggerEquipIntent\.formID\s*==\s*heldRefForGameplay->GetFormID\(\)' `
    'A retained trigger edge must replay only after the same hand holds the exact selected weapon.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'processHand\(_rightHand,\s*false\);\s*publishHandInputOwnership\(_rightHand,\s*false\);[\s\S]{0,120}processHand\(_leftHand,\s*true\);\s*publishHandInputOwnership\(_leftHand,\s*true\)' `
    'Input ownership must be republished immediately after each hand transition so left Pip-Boy suppression cannot lag a committed grab.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_pendingEquippedWeaponPrimaryOnlyGripStart\s*=\s*pendingGripStart' `
    'Loose equip must preserve the originating hand and captured weapon-local frame across inventory transfer.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'firingHandIsLeft\s*&&[\s\S]{0,180}!capturedFiringHandWeaponLocal[\s\S]{0,500}blockFrikPrimaryWeaponPose\(\)[\s\S]{0,500}setFiringHand\(firingHandIsLeft,[\s\S]{0,300}_primaryHandWeaponLocal\s*=\s*\*capturedFiringHandWeaponLocal' `
    'Left primary-only ownership must fail closed without the captured loose-weapon hold and commit the originating hand before transition.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'RE::TESObjectREFR\* gripZoneHoverCandidate\s*=\s*nullptr;\s*if\s*\(\s*!isLeft\s*&&' `
    'Grip-zone hover/equip discovery must not remain right-hand-only.'

# Pull-catch/force-grab canonical auto-align must cover both physical hands
# through the shared firing-hold resolver.
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'loose_weapon_grip_zone::tryResolveLooseWeaponFiringHandHold\([\s\S]{0,160}isLeft,[\s\S]{0,160}selection\.refr[\s\S]{0,300}multiplyTransforms\([\s\S]{0,120}handWorld,[\s\S]{0,120}transform_math::invertTransform\(handWeaponLocal\)\)' `
    'Pull-catch/force-grab must seat either hand from the shared in-memory canonical hold (weapon = hand world o inverse(hold)).'

Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    '"notPrimaryHand"' `
    'The secondary hand must no longer be excluded from the loose-weapon FRIK-offset attach.'

Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'tryResolveLooseWeaponFiringHandHold\([\s\S]{0,700}tryResolveGripWorld\(isLeft,\s*weaponRef,\s*scratch,\s*&testedHandWorld\)' `
    'The one-shot firing-hold resolver must share the grip-zone projection core so both paths seat identical holds.'

# Non-throwable weapons never participate in saved grab offsets, on either
# side: the shared custom/authored weapon-grip pipeline owns their seat.
Require-Text 'src/physics-interaction/grab/SavedGrabOffsetStore.h' `
    'constexpr\s+bool\s+participatesInSavedGrabOffsets\(' `
    'Saved-grab-offset weapon eligibility must be one shared policy for the save and apply sides.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'programmaticArrival\s*&&[\s\S]{0,200}participatesInSavedGrabOffsets\(\s*looseWeaponGrab,' `
    'Pull-catch/force-grab must not resolve saved grab offsets for non-throwable weapons.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'participatesInSavedGrabOffsets\(weaponForm\s*!=\s*nullptr,\s*throwableWeapon\)' `
    'The dev-mode save gesture must refuse to record grab offsets for non-throwable weapons.'

if ($failures.Count -gt 0) {
    Write-Host 'Held weapon hand-specific equip source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Held weapon hand-specific equip source boundary passed.'

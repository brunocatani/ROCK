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

foreach ($configSource in @('src/RockConfig.cpp', 'src/RockConfig.h')) {
    Reject-Text $configSource `
        'rock(?:RealisticWeaponHandlingEnabled|GrabbedWeaponGripZone\w+|WeaponGripHaptic\w+|WeaponFiringGrip\w+|WeaponSupportGripHaptic\w+|EquippedWeaponShoulderStashEnabled|GripZoneHover\w+|GrabbedWeaponEquipBridge\w+|MenuTriggerHandEquipEnabled|EquipPreferredHandLeft)' `
        'Base ROCK config must not regain addon-owned realistic detach, grip-zone, stash, haptic, bridge, or Pip-Boy settings.'
}

Require-Text 'src/RockConfig.h' `
    'rockAmbidextrousFiringGripEnabled\s*=\s*true[\s\S]{0,220}rockFiringGripPromotionRadius\s*=\s*5\.0f[\s\S]{0,320}rockLeftFiringAimYawDegrees\s*=\s*0\.0f[\s\S]{0,320}rockLeftFiringAimOffsetZGameUnits\s*=\s*0\.0f' `
    'ROCK must own an enabled standalone ambidextrous baseline and its bounded left-firing tuning.'
Require-Text 'src/RockConfig.cpp' `
    'AMBIDEXTROUS_FIRING_SECTION\s*=\s*"AmbidextrousFiring"[\s\S]*GetBoolValue\(\s*AMBIDEXTROUS_FIRING_SECTION,\s*"bAmbidextrousFiringGripEnabled"[\s\S]*"fFiringGripPromotionRadius"[\s\S]*"fLeftFiringAimOffsetZGameUnits"' `
    'ROCK must load its standalone handoff switch and tuning only from [AmbidextrousFiring].'

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
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
    if ($configText -match '(?m)^(?:bRealisticWeaponHandlingEnabled|fGrabbedWeaponGripZone\w+|fWeapon(?:Firing|Support)?GripHaptic\w+|bGripZoneHoverHapticsEnabled|bEquippedWeaponShoulderStashEnabled|bGrabbedWeaponEquipBridgeEnabled|bMenuTriggerHandEquipEnabled)\s*=') {
        $failures.Add("$configPath`: addon-owned realistic detach, grip-zone, haptic, stash, bridge, and Pip-Boy options must remain absent from ROCK.")
    }
}

Require-Text 'src/api/ROCKProviderApi.h' `
    'RockProviderEquippedWeaponHandlingFlagV1[\s\S]*FiringGripOwnership[\s\S]*PrimaryDetach[\s\S]*AmbidextrousHandoff[\s\S]*GripZoneEquip[\s\S]*PipboyTriggerHandEquip[\s\S]*RockProviderEquippedWeaponHandlingRequestV1' `
    'ROCK V1 must expose the complete owner-bound equipped-weapon policy consumed by the addon.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'rolling lease returns to ROCK''s configured fallback handling policy' `
    'The V1 lease contract must describe fallback to ROCK configuration rather than a hard-coded firing hand.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'RockEquippedWeaponHandlingBaseline[\s\S]*ambidextrousHandoffEnabled[\s\S]*firingGripPromotionRadiusGameUnits[\s\S]*leftFiringAimOffsetZGameUnits[\s\S]*makeEquippedWeaponHandlingSettings[\s\S]*settings\.firingGripOwnershipEnabled\s*=[\s\S]{0,120}rockBaseline\.ambidextrousHandoffEnabled[\s\S]*externalAuthorityActive\s*=\s*true[\s\S]*AmbidextrousHandoff[\s\S]*leftFiringAimOffsetGameUnits' `
    'ROCK must seed standalone handoff ownership/tuning and let one validated V1 request replace the policy snapshot.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'requiresEquippedWeaponHandlingModeReconcile[\s\S]*fixedFiringHandChanged[\s\S]*previous\.firingGripOwnershipEnabled[\s\S]*previous\.primaryDetachEnabled[\s\S]*previous\.ambidextrousHandoffEnabled[\s\S]*previous\.pipboyTriggerHandEquipEnabled' `
    'Mode reconciliation must follow removed effective capabilities rather than the raw presence of an addon lease.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'getEquippedWeaponHandlingAuthorityV1\(request\)[\s\S]*RockEquippedWeaponHandlingBaseline[\s\S]*rockAmbidextrousFiringGripEnabled[\s\S]*makeEquippedWeaponHandlingSettings[\s\S]*if \(fixedFiringHandIsLeft\)[\s\S]*settings\.firingGripOwnershipEnabled\s*=\s*true[\s\S]*requiresEquippedWeaponHandlingModeReconcile' `
    'ROCK must build its native baseline before the addon overlay, preserve fixed-left ownership under either source, and reconcile effective capability loss.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(_equippedWeaponHandlingSettings\.ambidextrousHandoffEnabled\s*&&\s*_twoHandedGrip\.isManualOwnershipActive\(\)\)' `
    'Fixed-left fallback must preserve a deliberate handoff from either ROCK or the addon instead of checking external ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gripZoneSettleEquipEnabled\s*=\s*[\s\S]{0,180}canSettleEquipInGripZone\(\s*_equippedWeaponHandlingSettings\.gripZoneEquipEnabled\s*\)' `
    'Grip-zone equip and hover discovery must activate only from the addon snapshot.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'consumeHapticEvents\(\);[\s\S]{0,700}if \(_equippedWeaponHandlingSettings\.externalAuthorityActive\)' `
    'Equipped-weapon transition haptics must remain addon-owned while events are always drained.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldStartHeldWeaponEquipOwnership[\s\S]{0,260}\.modes\s*=\s*firingGripModes[\s\S]{0,120}\.handIsLeft\s*=\s*isLeft[\s\S]{0,120}\.gripHeld\s*=\s*rawGrabInput\.held' `
    'Direct trigger equip must start core-or-addon hand-specific ownership from the originating hand grip.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'effectiveHandlingSettings\.firingGripOwnershipEnabled\s*=[\s\S]{0,160}firingGripOwnershipFeatureAvailable[\s\S]{0,220}effectiveHandlingSettings\.ambidextrousHandoffEnabled\s*=[\s\S]{0,160}ambidextrousHandoffAvailable[\s\S]{0,220}effectiveHandlingSettings\.primaryDetachEnabled\s*=[\s\S]{0,160}primaryDetachFeatureAvailable[\s\S]{0,260}_twoHandedGrip\.update\([\s\S]*effectiveHandlingSettings' `
    'Two-handed weapon state must receive infrastructure-gated ownership, handoff, and detach values while preserving the active ROCK-or-addon tuning.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'updatePrimaryOnlyGrip[\s\S]{0,1000}primaryGripRetained\s*=\s*equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership\(\s*primaryDetachEnabled,\s*primaryGripInput\.held\)' `
    'Non-detaching fixed or addon ownership must ignore grip release while still running equipped-weapon identity cleanup.'

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

Require-Text 'src/physics-interaction/object/ObjectDetection.h' `
    'struct\s+SelectedObject[\s\S]{0,500}RE::NiPointer<RE::TESObjectREFR>\s+retainedRef[\s\S]{0,180}RE::TESObjectREFR\*\s+refr[\s\S]*setReference\(RE::TESObjectREFR\*\s+value\)[\s\S]{0,180}retainedRef\.reset\(value\)[\s\S]{0,120}refr\s*=\s*retainedRef\.get\(\)' `
    'A selection that outlives its physics query must retain the TESObjectREFR behind its raw hot-path alias.'

Require-Text 'src/physics-interaction/grab/GrabConstraint.h' `
    'struct\s+SavedObjectState[\s\S]*RE::NiPointer<RE::TESObjectREFR>\s+retainedRef[\s\S]{0,180}RE::TESObjectREFR\*\s+refr[\s\S]*setReference\(const\s+RE::NiPointer<RE::TESObjectREFR>&\s+value\)' `
    'An active grab must retain its world reference instead of relying on the visual model or a raw pointer.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'const\s+auto\s+selectedRef\s*=\s*sel\.retainedRef[\s\S]*_savedObjectState\.setReference\(selectedRef\)[\s\S]*outcome\.retainedRef\s*=\s*_savedObjectState\.retainedRef[\s\S]{0,120}outcome\.refr\s*=\s*outcome\.retainedRef\.get\(\)[\s\S]*_savedObjectState\.clear\(\)' `
    'Grab commit and release must carry one strong reference across cleanup until the caller explicitly takes transfer ownership.'

Require-Text 'src/physics-interaction/hand/Hand.h' `
    'takeRetainedReference\(\)[\s\S]{0,180}refr\s*=\s*nullptr;[\s\S]{0,100}std::move\(retainedRef\)' `
    'Native transfers must be able to consume the release pin while invalidating its raw alias.'

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

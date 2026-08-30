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
        'data/config/ROCK_example.ini')) {
    Reject-Text $legacyPath `
        'GrabbedWeaponAutoEquip|HeldWeaponAutoEquip|legacyAutoEquip|autoEquipEnabled|autoEquipSettled|autoEquipState|settled-auto-held-weapon-equip' `
        'The superseded position-blind loose-weapon auto-equip path must remain deleted.'
}

foreach ($configSource in @('src/RockConfig.cpp', 'src/RockConfig.h')) {
    Reject-Text $configSource `
        'rock(?:RealisticWeaponHandlingEnabled|GrabbedWeaponGripZone\w+|WeaponGripHaptic\w+|WeaponFiringGrip\w+|WeaponSupportGripHaptic\w+|GripZoneHover\w+|GrabbedWeaponEquipBridge\w+|MenuTriggerHandEquipEnabled|EquipPreferredHandLeft)' `
        'ROCK must not regain the old bundled realistic, grip-zone, support-haptic, bridge, or Pip-Boy settings.'
}

Require-Text 'src/RockConfig.h' `
    'rockPhysicalRightFiringGripDetachEnabled\s*=\s*true[\s\S]{0,220}rockPhysicalRightFiringGripDetachPosePreservationEnabled\s*=\s*true[\s\S]{0,220}rockPhysicalRightFiringGripReattachRadiusGameUnits\s*=\s*3\.0f[\s\S]{0,260}rockPhysicalRightFiringGripHapticDurationSeconds\s*=\s*0\.10f[\s\S]{0,220}rockPhysicalRightFiringGripAttachHapticIntensity\s*=\s*0\.85f[\s\S]{0,220}rockPhysicalRightFiringGripDetachHapticIntensity\s*=\s*0\.30f' `
    'ROCK must own one physical-right-only detach contract with enabled pose preservation and bounded attach/detach haptic tuning.'
Require-Text 'src/RockConfig.cpp' `
    'IMMERSIVE_WEAPONS_SECTION\s*=\s*"ImmersiveWeapons"[\s\S]*"bPhysicalRightFiringGripDetachEnabled"[\s\S]*"bPhysicalRightFiringGripDetachPosePreservationEnabled"[\s\S]*"fPhysicalRightFiringGripReattachRadiusGameUnits"[\s\S]*"fPhysicalRightFiringGripHapticDurationSeconds"[\s\S]*"fPhysicalRightFiringGripAttachHapticIntensity"[\s\S]*"fPhysicalRightFiringGripDetachHapticIntensity"' `
    'ROCK must load only the narrow physical-right detach, pose-preservation, and attach/detach haptic catalog from [ImmersiveWeapons].'

Require-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'IntegratedPhysicalRight[\s\S]*ExternalProvider[\s\S]*appliesToPhysicalHand\([\s\S]{0,260}enabled\s*&&\s*!handIsLeft[\s\S]*externalPrimaryDetachEnabled[\s\S]*IntegratedPhysicalRight' `
    'The integrated policy must bind detach to physical right while preserving explicit provider all-hand authority.'
Require-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'preserveWeaponPoseOnDetach\s*\{\s*false\s*\}[\s\S]*if \(input\.externalPrimaryDetachEnabled\)[\s\S]*else if \(appliesToPhysicalHand[\s\S]*decision\.preserveWeaponPoseOnDetach\s*=\s*input\.integrated\.[\s\r\n ]*physicalRightFiringGripDetachPosePreservationEnabled' `
    'Only integrated physical-right authority may select the ROCK detach pose-preservation policy.'
Reject-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'VirtualHolsters|RIW\.dll|ROCKIMMERSIVEWEAPONSAPI|GripZoneEquip|PipboyTriggerHandEquip|EquippedWeaponShoulderStash|EquipVisualBridge' `
    'The integrated physical-right policy must not absorb RIW compatibility or unrelated weapon features.'

Require-Text 'src/RockConfig.h' `
    'rockAmbidextrousFiringGripEnabled\s*=\s*true[\s\S]{0,420}rockAuthoredOnlyEquippedWeaponSupportGrabsEnabled\s*=\s*true[\s\S]{0,220}rockEquippedWeaponToggleGrabEnabled\s*=\s*false[\s\S]{0,220}rockFiringGripPromotionRadius\s*=\s*5\.0f[\s\S]{0,320}rockLeftFiringAimYawDegrees\s*=\s*0\.0f[\s\S]{0,320}rockLeftFiringAimOffsetZGameUnits\s*=\s*0\.0f' `
    'ROCK must own its ambidextrous baseline, enabled authored-only support policy, opt-in weapon toggle input, and bounded left-firing tuning.'
Require-Text 'src/RockConfig.cpp' `
    'AMBIDEXTROUS_FIRING_SECTION\s*=\s*"AmbidextrousFiring"[\s\S]*GetBoolValue\(\s*AMBIDEXTROUS_FIRING_SECTION,\s*"bAmbidextrousFiringGripEnabled"[\s\S]*"bAuthoredOnlyEquippedWeaponSupportGrabsEnabled"[\s\S]*"bEquippedWeaponToggleGrabEnabled"[\s\S]*"fFiringGripPromotionRadius"[\s\S]*"fLeftFiringAimOffsetZGameUnits"' `
    'ROCK must load its handoff, authored-only support policy, weapon toggle input, and tuning only from [AmbidextrousFiring].'

foreach ($configPath in @('data/config/ROCK_example.ini')) {
    $configText = Read-Source $configPath
    $realisticSection = Read-IniSection $configPath 'RealisticWeapons'
    $realisticAssignments = [regex]::Matches($realisticSection, '(?m)^[A-Za-z]\w*\s*=')
    if ($realisticAssignments.Count -ne 1 -or
        $realisticSection -notmatch '(?m)^fRealisticGrenadeFuseSeconds\s*=') {
        $failures.Add("$configPath`: [RealisticWeapons] must retain only ROCK grenade fuse ownership.")
    }
    $immersiveSection = Read-IniSection $configPath 'ImmersiveWeapons'
    $immersiveAssignments = [regex]::Matches($immersiveSection, '(?m)^[A-Za-z]\w*\s*=')
    if ($immersiveAssignments.Count -ne 6 -or
        $immersiveSection -notmatch '(?m)^bPhysicalRightFiringGripDetachEnabled\s*=\s*true\s*$' -or
        $immersiveSection -notmatch '(?m)^bPhysicalRightFiringGripDetachPosePreservationEnabled\s*=\s*true\s*$' -or
        $immersiveSection -notmatch '(?m)^fPhysicalRightFiringGripReattachRadiusGameUnits\s*=\s*3\.0\s*$' -or
        $immersiveSection -notmatch '(?m)^fPhysicalRightFiringGripHapticDurationSeconds\s*=\s*0\.10\s*$' -or
        $immersiveSection -notmatch '(?m)^fPhysicalRightFiringGripAttachHapticIntensity\s*=\s*0\.85\s*$' -or
        $immersiveSection -notmatch '(?m)^fPhysicalRightFiringGripDetachHapticIntensity\s*=\s*0\.30\s*$') {
        $failures.Add("$configPath`: [ImmersiveWeapons] must expose exactly the enabled physical-right detach and pose preservation, radius, and attach/detach haptic defaults.")
    }
    $handednessSection = Read-IniSection $configPath 'WeaponHandedness'
    if ($handednessSection -notmatch '(?m)^bLeftHandedMode\s*=\s*false\s*$') {
        $failures.Add("$configPath`: [WeaponHandedness] must expose the sole fixed-hand option and default right.")
    }
    $ambidextrousSection = Read-IniSection $configPath 'AmbidextrousFiring'
    $ambidextrousAssignments = [regex]::Matches($ambidextrousSection, '(?m)^[A-Za-z]\w*\s*=')
    if ($ambidextrousAssignments.Count -ne 9 -or
        $ambidextrousSection -notmatch '(?m)^bAmbidextrousFiringGripEnabled\s*=\s*true\s*$' -or
        $ambidextrousSection -notmatch '(?m)^bAuthoredOnlyEquippedWeaponSupportGrabsEnabled\s*=\s*true\s*$' -or
        $ambidextrousSection -notmatch '(?m)^bEquippedWeaponToggleGrabEnabled\s*=\s*false\s*$' -or
        $ambidextrousSection -notmatch '(?m)^fFiringGripPromotionRadius\s*=\s*5\.0\s*$' -or
        ([regex]::Matches($ambidextrousSection, '(?m)^fLeftFiringAim\w+\s*=\s*0\.0\s*$')).Count -ne 5) {
        $failures.Add("$configPath`: [AmbidextrousFiring] must expose enabled handoff/authored-only support, opt-in weapon toggle input, promotion radius, and five zeroed left-hold trims.")
    }
    $physicsSection = Read-IniSection $configPath 'PhysicsInteraction'
    if ($physicsSection -notmatch '(?m)^bEquippedWeaponShoulderStashEnabled\s*=\s*true\s*$') {
        $failures.Add("$configPath`: [PhysicsInteraction] must expose ROCK's enabled equipped-weapon shoulder stash switch.")
    }
    if ($configText -match '(?m)^(?:bRealisticWeaponHandlingEnabled|fGrabbedWeaponGripZone\w+|fWeapon(?:Firing|Support)?GripHaptic\w+|bGripZoneHoverHapticsEnabled|bGrabbedWeaponEquipBridgeEnabled|bMenuTriggerHandEquipEnabled)\s*=') {
        $failures.Add("$configPath`: old bundled realistic, grip-zone, support-haptic, bridge, and Pip-Boy options must remain absent from ROCK.")
    }
}

Require-Text 'src/api/ROCKProviderApi.h' `
    'RockProviderEquippedWeaponHandlingFlagV1[\s\S]*FiringGripOwnership[\s\S]*PrimaryDetach[\s\S]*AmbidextrousHandoff[\s\S]*GripZoneEquip[\s\S]*PipboyTriggerHandEquip[\s\S]*RockProviderEquippedWeaponHandlingRequestV1' `
    'ROCK V1 must expose the complete owner-bound equipped-weapon policy consumed by the addon.'
Require-Text 'src/api/ROCKProviderApi.h' `
    'rolling lease returns to ROCK''s configured fallback handling policy' `
    'The V1 lease contract must describe fallback to ROCK configuration rather than a hard-coded firing hand.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'RockEquippedWeaponHandlingBaseline[\s\S]*immersive_weapon_policy::Config\s+immersiveWeapon[\s\S]*makeEquippedWeaponHandlingSettings[\s\S]*settings\.primaryDetachEnabled\s*=\s*false[\s\S]*settings\.immersiveWeapon\s*=\s*rockBaseline\.immersiveWeapon[\s\S]*externalAuthorityActive\s*=\s*true[\s\S]*PrimaryDetach[\s\S]*resolveEquippedWeaponDetachDecision[\s\S]*settings\.immersiveWeapon[\s\S]*externalPrimaryDetachEnabled' `
    'ROCK must retain provider detach separately, carry the integrated policy snapshot, and resolve both against physical hand identity.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'toggleGrabEnabled[\s\S]*settings\.toggleGrabEnabled\s*=\s*rockBaseline\.toggleGrabEnabled[\s\S]*if \(!request\)' `
    'Equipped-weapon toggle grab must remain a ROCK-owned input preference across addon handling leases.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'authoredOnlySupportGrabsEnabled[\s\S]*settings\.authoredOnlySupportGrabsEnabled\s*=[\s\r\n]+\s*rockBaseline\.authoredOnlySupportGrabsEnabled[\s\S]*if \(!request\)' `
    'Authored-only support acquisition must remain a ROCK-owned preference across addon handling leases; exact part targets override later at capture.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'requiresEquippedWeaponHandlingModeReconcile[\s\S]*fixedFiringHandChanged[\s\S]*integratedPhysicalRightDetachRemoved[\s\S]*previous\.firingGripOwnershipEnabled[\s\S]*previous\.primaryDetachEnabled[\s\S]*previous\.ambidextrousHandoffEnabled[\s\S]*previous\.pipboyTriggerHandEquipEnabled' `
    'Mode reconciliation must follow removed effective capabilities rather than the raw presence of an addon lease.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'getEquippedWeaponHandlingAuthorityV1\(request\)[\s\S]*RockEquippedWeaponHandlingBaseline[\s\S]*rockAmbidextrousFiringGripEnabled[\s\S]*rockAuthoredOnlyEquippedWeaponSupportGrabsEnabled[\s\S]*rockEquippedWeaponShoulderStashEnabled[\s\S]*rockPhysicalRightFiringGripDetachEnabled[\s\S]*rockPhysicalRightFiringGripDetachPosePreservationEnabled[\s\S]*rockPhysicalRightFiringGripDetachHapticIntensity[\s\S]*makeEquippedWeaponHandlingSettings[\s\S]*if \(fixedFiringHandIsLeft\)[\s\S]*settings\.firingGripOwnershipEnabled\s*=\s*true[\s\S]*requiresEquippedWeaponHandlingModeReconcile' `
    'ROCK must compose handoff/support/stash and integrated physical-right settings before the provider overlay and fixed-left reconciliation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(_equippedWeaponHandlingSettings\.ambidextrousHandoffEnabled\s*&&\s*_twoHandedGrip\.isManualOwnershipActive\(\)\)' `
    'Fixed-left fallback must preserve a deliberate handoff from either ROCK or the addon instead of checking external ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gripZoneSettleEquipEnabled\s*=\s*[\s\S]{0,180}canSettleEquipInGripZone\(\s*_equippedWeaponHandlingSettings\.gripZoneEquipEnabled\s*\)' `
    'Grip-zone equip and hover discovery must remain separate from integrated physical-right detach.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'consumeHapticEvents\(\);[\s\S]*resolveEquippedWeaponDetachDecision\([\s\S]{0,220}_equippedWeaponHandlingSettings,[\s\S]{0,120}isLeft[\s\S]*IntegratedPhysicalRight[\s\S]*if \(_equippedWeaponHandlingSettings\.externalAuthorityActive\)' `
    'Firing-grip haptics must select integrated physical-right tuning first while preserving external-provider event tuning.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldStartHeldWeaponEquipOwnership[\s\S]{0,260}\.modes\s*=\s*firingGripModes[\s\S]{0,120}\.handIsLeft\s*=\s*isLeft[\s\S]{0,120}\.gripHeld\s*=\s*rawGrabInput\.held' `
    'Direct trigger equip must start integrated-or-provider hand-specific ownership from the originating hand grip.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'FiringGripModeAvailability[\s\S]{0,260}physicalRightDetachEnabled[\s\S]*shouldStartHeldWeaponEquipOwnership[\s\S]{0,320}appliesToPhysicalHand\([\s\S]{0,160}input\.modes\.physicalRightDetachEnabled[\s\S]{0,120}input\.handIsLeft' `
    'Held-trigger equip must allow integrated detach ownership only for the physical right hand.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'effectiveHandlingSettings\.firingGripOwnershipEnabled\s*=[\s\S]{0,160}firingGripOwnershipFeatureAvailable[\s\S]{0,220}effectiveHandlingSettings\.ambidextrousHandoffEnabled\s*=[\s\S]{0,160}ambidextrousHandoffAvailable[\s\S]{0,220}effectiveHandlingSettings\.primaryDetachEnabled\s*=[\s\S]{0,180}primaryDetachFeatureAvailable[\s\S]*effectiveHandlingSettings\.detachAuthority[\s\S]*effectiveHandlingSettings\.preserveWeaponPoseOnDetach[\s\S]*effectiveHandlingSettings\.firingGripReattachRadiusGameUnits[\s\S]*_twoHandedGrip\.update\([\s\S]*effectiveHandlingSettings' `
    'Two-handed weapon state must receive one infrastructure-gated, source-labelled physical-hand detach decision, pose policy, and selected tuning.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'primary-only-drop[\s\S]{0,500}IntegratedPhysicalRight[\s\S]{0,180}recordFiringGripDetachedHaptic\(\)[\s\S]{0,260}requestEquippedWeaponDrop[\s\S]*ambidextrous-firing-hand-promotion[\s\S]{0,300}IntegratedPhysicalRight[\s\S]{0,180}recordFiringGripDetachedHaptic\(\)' `
    'Integrated physical-right direct drop and handoff must emit one detach transition event without widening provider behavior.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'canCarryAfterFiringGripDetach\(_authorityMode\)[\s\S]{0,900}transitionToPartCarry\(\)[\s\S]*requestEquippedWeaponDrop\([\s\S]{0,180}primary-released-without-carry-authority' `
    'A firing-grip release must enter part carry only with carry authority and otherwise use the physical drop path.'

Require-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'resolveDetachedFiringHandPartGrab[\s\S]*IntegratedPhysicalRight[\s\S]{0,180}detachedHandIsLeft[\s\S]{0,180}authoredOnlySupportGrabsEnabled[\s\S]{0,240}DetachedFiringHandPartGrabSelection::Standard[\s\S]{0,220}exactProviderPartTargetActive[\s\S]{0,180}ExactProviderTarget[\s\S]{0,180}Reject' `
    'Authored-only integrated physical-right carry must reserve ordinary grabs for firing-grip reattach while allowing only exact provider part targets.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'transitionToPartCarry\(\)[\s\S]*_partCarryDetachAuthority\s*=\s*_handlingSettings\.detachAuthority[\s\S]{0,180}_state\s*=\s*TwoHandedState::PartCarry[\s\S]*updatePartCarryGrip\([\s\S]*tryReattachFiringGrip\([\s\S]*resolveDetachedFiringHandPartGrab\([\s\S]{0,900}providerPartAuthority\.active[\s\S]{0,500}DetachedFiringHandPartGrabSelection::Reject[\s\S]*capturePartGrip\(' `
    'PartCarry must retain its detach origin, try the authored firing grip first, and gate only the former firing hand before generic part capture.'

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
    'weapon_grip_authority_policy::select\([\s\S]{0,1200}frikCustomFile[\s\S]{0,600}authoredAnimation[\s\S]{0,600}frikEmbeddedResource[\s\S]{0,1600}Source::AuthoredAnimation[\s\S]{0,600}authoredLookup\.rightHandWeaponLocal[\s\S]{0,300}authoredPositionOnly\s*=\s*true' `
    'Loose weapons must select explicit hFRIK JSON before ROCK-authored data and mark only the learned authored relation as position-only weapon authority.'

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
    'processGrabInputHand\(frame,\s*_rightHand,\s*false,\s*handContext\);\s*publishHandInputOwnership\(_rightHand,\s*false\);[\s\S]{0,160}processGrabInputHand\(frame,\s*_leftHand,\s*true,\s*handContext\);\s*publishHandInputOwnership\(_leftHand,\s*true\)' `
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
    '(?s)^(?=.*outSelection\.retainedRef\s*=\s*selection\.retainedRef)(?=.*const\s+auto\s+selectedRef\s*=\s*validatedSelection\.retainedRef)(?=.*_savedObjectState\.setReference\(selectedRef\))(?=.*outcome\.retainedRef\s*=\s*_savedObjectState\.retainedRef)(?=.*outcome\.refr\s*=\s*outcome\.retainedRef\.get\(\))(?=.*_savedObjectState\.clear\(\))' `
    'Grab commit and release must carry one strong reference across cleanup until the caller explicitly takes transfer ownership.'

Require-Text 'src/physics-interaction/hand/Hand.h' `
    'takeRetainedReference\(\)[\s\S]{0,180}refr\s*=\s*nullptr;[\s\S]{0,100}std::move\(retainedRef\)' `
    'Native transfers must be able to consume the release pin while invalidating its raw alias.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'if\s*\(firingHandIsLeft\)[\s\S]{0,500}tryBuildCurrentLeftFiringGripCapture[\s\S]{0,1800}!retainUntilPhysicalGrip[\s\S]{0,900}capturedFiringHandWeaponLocal[\s\S]{0,1200}hasRightNativeWeaponAimFrame[\s\S]{0,500}captureRightNativeWeaponAimFrame[\s\S]{0,900}blockFrikPrimaryWeaponPose\(\)[\s\S]{0,500}setFiringHand\(firingHandIsLeft,[\s\S]{0,300}_primaryHandWeaponLocal\s*=\s*resolvedLeftHandWeaponLocal' `
    'Left primary-only ownership must resolve a current normalized seat or committed equipped transfer, require the native aim baseline, and commit the originating hand before transition.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'RE::TESObjectREFR\* gripZoneHoverCandidate\s*=\s*nullptr;\s*if\s*\(\s*!isLeft\s*&&' `
    'Grip-zone hover/equip discovery must not remain right-hand-only.'

# Pull-catch/force-grab canonical auto-align must cover both physical hands
# through the shared firing-hold resolver. Authored cache data supplies only
# the firing point; explicit hFRIK offsets retain full-rigid correction.
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'loose_weapon_grip_zone::tryResolveLooseWeaponFiringHandHold\([\s\S]{0,400}authoredPositionOnly[\s\S]{0,600}resolveAuthoredPrimaryWeaponWorldPositionOnly\([\s\S]{0,300}rootNode->world[\s\S]{0,500}else\s*\{[\s\S]{0,300}multiplyTransforms\([\s\S]{0,120}handWorld,[\s\S]{0,120}transform_math::invertTransform\(handWeaponLocal\)\)' `
    'Pull-catch/force-grab must translate authored cached grips without rotating the loose weapon while retaining full-rigid explicit hFRIK authority.'

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

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
    'rockFiringGripDetachEnabled\s*=\s*true[\s\S]{0,220}rockFiringGripDetachPosePreservationEnabled\s*=\s*true[\s\S]{0,220}rockFiringGripReattachRadiusGameUnits\s*=\s*3\.0f[\s\S]{0,260}rockFiringGripHapticDurationSeconds\s*=\s*0\.10f[\s\S]{0,220}rockFiringGripAttachHapticIntensity\s*=\s*0\.85f[\s\S]{0,220}rockFiringGripDetachHapticIntensity\s*=\s*0\.30f' `
    'ROCK must own one role-neutral firing-grip detach contract with enabled pose preservation and bounded attach/detach haptic tuning.'
Require-Text 'src/RockConfig.cpp' `
    'IMMERSIVE_WEAPONS_SECTION\s*=\s*"ImmersiveWeapons"[\s\S]*"bFiringGripDetachEnabled"[\s\S]*"bFiringGripDetachPosePreservationEnabled"[\s\S]*"fFiringGripReattachRadiusGameUnits"[\s\S]*"fFiringGripHapticDurationSeconds"[\s\S]*"fFiringGripAttachHapticIntensity"[\s\S]*"fFiringGripDetachHapticIntensity"' `
    'ROCK must load only the role-neutral detach, pose-preservation, and attach/detach haptic catalog from [ImmersiveWeapons].'

Require-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'IntegratedImmersive[\s\S]*ExternalProvider[\s\S]*externalPrimaryDetachEnabled[\s\S]*else if \(input\.integrated\.firingGripDetachEnabled\)[\s\S]*IntegratedImmersive' `
    'The integrated policy must bind detach to the current firing role while preserving explicit provider authority.'
Require-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'preserveWeaponPoseOnDetach\s*\{\s*false\s*\}[\s\S]*if \(input\.externalPrimaryDetachEnabled\)[\s\S]*else if \(input\.integrated\.firingGripDetachEnabled\)[\s\S]*decision\.preserveWeaponPoseOnDetach\s*=\s*input\.integrated\.[\s\r\n ]*firingGripDetachPosePreservationEnabled' `
    'Only integrated immersive authority may select the ROCK detach pose-preservation policy.'
Reject-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'VirtualHolsters|RIW\.dll|ROCKIMMERSIVEWEAPONSAPI|GripZoneEquip|PipboyTriggerHandEquip|EquippedWeaponShoulderStash|EquipVisualBridge' `
    'The integrated immersive policy must not absorb RIW compatibility or unrelated weapon features.'

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
        $immersiveSection -notmatch '(?m)^bFiringGripDetachEnabled\s*=\s*true\s*$' -or
        $immersiveSection -notmatch '(?m)^bFiringGripDetachPosePreservationEnabled\s*=\s*true\s*$' -or
        $immersiveSection -notmatch '(?m)^fFiringGripReattachRadiusGameUnits\s*=\s*3\.0\s*$' -or
        $immersiveSection -notmatch '(?m)^fFiringGripHapticDurationSeconds\s*=\s*0\.10\s*$' -or
        $immersiveSection -notmatch '(?m)^fFiringGripAttachHapticIntensity\s*=\s*0\.85\s*$' -or
        $immersiveSection -notmatch '(?m)^fFiringGripDetachHapticIntensity\s*=\s*0\.30\s*$') {
        $failures.Add("$configPath`: [ImmersiveWeapons] must expose exactly the enabled role-neutral detach and pose preservation, radius, and attach/detach haptic defaults.")
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
Require-Text 'src/api/ROCKProviderApi.h' `
    'Explicit right and left assignments have the same authority[\s\S]{0,180}AmbidextrousHandoff governs in-world role swaps' `
    'Explicit provider hand assignment must not require extra authority merely because the requested hand is left.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(assignment\.assignedLeft\)[\s\S]{0,400}requiredFlags[\s\S]{0,260}AmbidextrousHandoff' `
    'The runtime must not add an AmbidextrousHandoff requirement only to explicit left-hand assignment.'
Reject-Text 'src/api/ROCKProviderApi.cpp' `
    'request->hand\s*==\s*RockProviderHand::Left[\s\S]{0,500}AmbidextrousHandoff' `
    'The provider API boundary must not add AmbidextrousHandoff authority only to an explicit left-hand assignment.'
Require-Text 'src/api/ROCKProviderApi.cpp' `
    'apiRequestEquippedWeaponHandV1\([\s\S]{0,4200}requiredFlags\s*=[\s\S]{0,260}FiringGripOwnership[\s\S]{0,800}s_equippedWeaponHandlingAuthority\.request\.flags[\s\S]{0,260}requiredFlags' `
    'The provider API boundary must authorize either explicit hand from the same firing-grip ownership flag.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'RockEquippedWeaponHandlingBaseline[\s\S]*immersive_weapon_policy::Config\s+immersiveWeapon[\s\S]*makeEquippedWeaponHandlingSettings[\s\S]*settings\.primaryDetachEnabled\s*=\s*false[\s\S]*settings\.immersiveWeapon\s*=\s*rockBaseline\.immersiveWeapon[\s\S]*externalAuthorityActive\s*=\s*true[\s\S]*PrimaryDetach[\s\S]*resolveEquippedWeaponDetachDecision[\s\S]*settings\.immersiveWeapon[\s\S]*externalPrimaryDetachEnabled' `
    'ROCK must retain provider detach separately, carry the integrated policy snapshot, and resolve both into one firing-role decision.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'toggleGrabEnabled[\s\S]*settings\.toggleGrabEnabled\s*=\s*rockBaseline\.toggleGrabEnabled[\s\S]*if \(!request\)' `
    'Equipped-weapon toggle grab must remain a ROCK-owned input preference across addon handling leases.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'authoredOnlySupportGrabsEnabled[\s\S]*settings\.authoredOnlySupportGrabsEnabled\s*=[\s\r\n]+\s*rockBaseline\.authoredOnlySupportGrabsEnabled[\s\S]*if \(!request\)' `
    'Authored-only support acquisition must remain a ROCK-owned preference across addon handling leases; exact part targets override later at capture.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'requiresEquippedWeaponHandlingModeReconcile[\s\S]*fixedFiringHandChanged[\s\S]*integratedImmersiveDetachRemoved[\s\S]*previous\.firingGripOwnershipEnabled[\s\S]*previous\.primaryDetachEnabled[\s\S]*previous\.ambidextrousHandoffEnabled[\s\S]*previous\.pipboyTriggerHandEquipEnabled' `
    'Mode reconciliation must follow removed effective capabilities rather than the raw presence of an addon lease.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'getEquippedWeaponHandlingAuthorityV1\(request\)[\s\S]*RockEquippedWeaponHandlingBaseline[\s\S]*rockAmbidextrousFiringGripEnabled[\s\S]*rockAuthoredOnlyEquippedWeaponSupportGrabsEnabled[\s\S]*rockEquippedWeaponShoulderStashEnabled[\s\S]*rockFiringGripDetachEnabled[\s\S]*rockFiringGripDetachPosePreservationEnabled[\s\S]*rockFiringGripDetachHapticIntensity[\s\S]*makeEquippedWeaponHandlingSettings[\s\S]*if \(fixedFiringHandIsLeft\)[\s\S]*settings\.firingGripOwnershipEnabled\s*=\s*true[\s\S]*requiresEquippedWeaponHandlingModeReconcile' `
    'ROCK must compose handoff/support/stash and integrated firing-role settings before the provider overlay and fixed-left reconciliation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if \(_equippedWeaponHandlingSettings\.ambidextrousHandoffEnabled\s*&&\s*_twoHandedGrip\.isManualOwnershipActive\(\)\)' `
    'Fixed-left fallback must preserve a deliberate handoff from either ROCK or the addon instead of checking external ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'gripZoneSettleEquipEnabled\s*=\s*[\s\S]{0,180}canSettleEquipInGripZone\(\s*_equippedWeaponHandlingSettings\.gripZoneEquipEnabled\s*\)' `
    'Grip-zone equip and hover discovery must remain separate from integrated firing-role detach.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'consumeHapticEvents\(\);[\s\S]*resolveEquippedWeaponDetachDecision\([\s\S]{0,220}_equippedWeaponHandlingSettings[\s\S]*IntegratedImmersive[\s\S]*queueGripHaptic\([\s\S]{0,160}isLeft[\s\S]*if \(_equippedWeaponHandlingSettings\.externalAuthorityActive\)' `
    'Firing-grip haptics must select integrated firing-role tuning and pulse the actual event hand while preserving external-provider tuning.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldStartHeldWeaponEquipOwnership[\s\S]{0,260}\.modes\s*=\s*firingGripModes[\s\S]{0,120}\.handIsLeft\s*=\s*isLeft[\s\S]{0,120}\.gripHeld\s*=\s*rawGrabInput\.held' `
    'Direct trigger equip must start integrated-or-provider hand-specific ownership from the originating hand grip.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'heldWeaponEquipOwnershipEligible\s*=[\s\S]{0,420}shouldStartHeldWeaponEquipOwnership[\s\S]{0,900}updateHeldLooseWeapon\([\s\S]{0,220}shouldTrackHeldWeaponGripFrame\([\s\S]{0,180}hand\.isHoldingLooseWeapon\(\)[\s\S]{0,180}gripZoneSettleEquipEnabled[\s\S]{0,180}heldWeaponEquipOwnershipEligible[\s\S]*pendingGripStart\.pending\s*=\s*heldWeaponEquipOwnershipEligible' `
    'Hand-preserving trigger equip must keep its canonical loose-weapon carry frame even when optional grip-zone settle is unavailable.'
Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'FiringGripModeAvailability[\s\S]{0,260}integratedDetachEnabled[\s\S]*shouldStartHeldWeaponEquipOwnership[\s\S]{0,320}input\.modes\.integratedDetachEnabled[\s\S]{0,200}input\.handIsLeft\s*&&\s*input\.modes\.ambidextrousHandoffAvailable' `
    'Held-trigger equip must allow integrated detach ownership for either firing hand while keeping handoff-only manual carry limited to the left native-adapter path.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'effectiveHandlingSettings\.firingGripOwnershipEnabled\s*=[\s\S]{0,160}firingGripOwnershipFeatureAvailable[\s\S]{0,220}effectiveHandlingSettings\.ambidextrousHandoffEnabled\s*=[\s\S]{0,160}ambidextrousHandoffAvailable[\s\S]{0,220}effectiveHandlingSettings\.primaryDetachEnabled\s*=[\s\S]{0,180}primaryDetachFeatureAvailable[\s\S]*effectiveHandlingSettings\.detachAuthority[\s\S]*effectiveHandlingSettings\.preserveWeaponPoseOnDetach[\s\S]*effectiveHandlingSettings\.firingGripReattachRadiusGameUnits[\s\S]*_twoHandedGrip\.update\([\s\S]*effectiveHandlingSettings' `
    'Two-handed weapon state must receive one infrastructure-gated, source-labelled physical-hand detach decision, pose policy, and selected tuning.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'primary-only-drop[\s\S]{0,500}IntegratedImmersive[\s\S]{0,180}recordFiringGripDetachedHaptic\(\)[\s\S]{0,260}requestEquippedWeaponDrop[\s\S]*ambidextrous-firing-hand-promotion[\s\S]{0,300}IntegratedImmersive[\s\S]{0,180}recordFiringGripDetachedHaptic\(\)' `
    'Integrated immersive direct drop and handoff must emit one detach transition event for either firing hand without widening provider behavior.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'canCarryAfterFiringGripDetach\(_authorityMode\)[\s\S]{0,900}transitionToPartCarry\(\)[\s\S]*requestEquippedWeaponDrop\([\s\S]{0,180}primary-released-without-carry-authority' `
    'A firing-grip release must enter part carry only with carry authority and otherwise use the physical drop path.'

Require-Text 'src/physics-interaction/weapon/immersive/ImmersiveWeaponPolicy.h' `
    'resolveDetachedFiringHandPartGrab[\s\S]*IntegratedImmersive[\s\S]{0,220}authoredOnlySupportGrabsEnabled[\s\S]{0,240}DetachedFiringHandPartGrabSelection::Standard[\s\S]{0,220}exactProviderPartTargetActive[\s\S]{0,180}ExactProviderTarget[\s\S]{0,180}Reject' `
    'Authored-only integrated carry must reserve ordinary grabs for firing-grip reattach for either detached firing hand while allowing exact provider part targets.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'transitionToPartCarry\(\)[\s\S]*_partCarryDetachAuthority\s*=\s*_handlingSettings\.detachAuthority[\s\S]{0,180}_state\s*=\s*TwoHandedState::PartCarry[\s\S]*updatePartCarryGrip\([\s\S]*tryReattachFiringGrip\([\s\S]*resolveDetachedFiringHandPartGrab\([\s\S]{0,900}providerPartAuthority\.active[\s\S]{0,500}DetachedFiringHandPartGrabSelection::Reject[\s\S]*capturePartGrip\(' `
    'PartCarry must retain its detach origin, try the authored firing grip first, and gate only the former firing hand before generic part capture.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'updatePrimaryOnlyGrip[\s\S]{0,1000}primaryGripRetained\s*=\s*equipped_weapon_manual_ownership_policy::shouldRetainPrimaryOnlyOwnership\(\s*primaryDetachEnabled,\s*_handlingSettings\.toggleGrabEnabled,\s*primaryGripInput\.held\)' `
    'Primary-only ownership must preserve non-detaching holds while allowing an explicit toggle-open command to release either firing hand.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingToggleGripRetained\s*=[\s\S]{0,260}toggleGrabEnabled[\s\S]{0,220}toggleAcquisitionCommitted[\s\S]{0,7000}getLeftFiringTakeoverReadiness\([\s\S]{0,500}currentWeaponGenerationKey[\s\S]{0,900}leftFiringTakeoverReady\([\s\S]{0,1600}else\s+if\s*\(primaryOnlyStartRequested[\s\S]{0,150}beginPrimaryOnlyGrip[\s\S]{0,500}retainUntilPhysicalGrip' `
    'A pending physical-left equip must retain a consumed toggle acquisition and wait for final-generation mirrored-support readiness or a qualified fallback verdict.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingHandPhysicalGrip\.released\s*\|\|[\s\S]{0,180}!pendingHandPhysicalGrip\.held[\s\S]{0,180}!pendingHandPhysicalGrip\.pressed[\s\S]{0,240}toggleAcquisitionReleased\s*=\s*true[\s\S]{0,260}pendingToggleCancelRequested\s*=[\s\S]{0,220}toggleAcquisitionReleased[\s\S]{0,160}pendingHandPhysicalGrip\.pressed[\s\S]{0,600}_equippedWeaponToggleGrabReleasePressConsumedThisFrame[\s\S]{0,350}=\s*true' `
    'A second press that cancels a pending toggle acquisition must remain consumed after toggle preparation for either physical hand.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_equippedWeaponToggleGrabReleasePressConsumedThisFrame\[[\s\S]{0,160}handIndex\(true\)[\s\S]{0,180}=\s*_equippedWeaponToggleGrabReleasePressConsumedThisFrame\[[\s\S]{0,160}handIndex\(true\)[\s\S]{0,100}\|\|[\s\S]{0,120}toggleGrabDecision\.leftReleasePressConsumed[\s\S]{0,420}_equippedWeaponToggleGrabReleasePressConsumedThisFrame\[[\s\S]{0,160}handIndex\(false\)[\s\S]{0,180}=\s*_equippedWeaponToggleGrabReleasePressConsumedThisFrame\[[\s\S]{0,160}handIndex\(false\)[\s\S]{0,100}\|\|[\s\S]{0,120}toggleGrabDecision\.rightReleasePressConsumed' `
    'Toggle preparation must preserve an already-consumed pending-cancellation press for both physical hands.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    'nativeRightSupportCaptureFrameReserved' `
    'Left takeover readiness must not regress to a fixed frame-count reservation.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponHandAssignment\([\s\S]{0,140}currentWeaponGenerationKey[\s\S]{0,220}serviceFixedWeaponHand\([\s\S]{0,140}currentWeaponGenerationKey' `
    'Fixed and assigned left entry must receive the raw collision generation rather than the provisional authored fallback key.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'void PhysicsInteraction::serviceFixedWeaponHand\([\s\S]{0,9000}getLeftFiringTakeoverReadiness\([\s\S]{0,600}leftFiringTakeoverReady' `
    'Fixed-left entry must wait for the shared final-generation authored-support verdict.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'void PhysicsInteraction::serviceEquippedWeaponHandAssignment\([\s\S]{0,14000}getLeftFiringTakeoverReadiness\([\s\S]{0,600}leftFiringTakeoverReady' `
    'Pip-Boy and provider left assignment must wait for the shared final-generation authored-support verdict.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'roleNeutralFiringGripOwnership\s*=[\s\S]{0,260}resolveEquippedWeaponDetachDecision\([\s\S]{0,180}firingGripOwnershipEnabled[\s\S]{0,1200}handAllowedByHandlingMode\s*=[\s\S]{0,180}roleNeutralFiringGripOwnership' `
    'Physical shoulder retrieval must allow either hand whenever the same firing-role ownership contract can carry it.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    '_persistentEquippedCarryActive\s*&&\s*isManualOwnershipActive\(\)[\s\S]{0,700}primaryGripInput\.held[\s\S]{0,160}primaryGripInput\.pressed[\s\S]{0,220}commitPersistentEquippedCarryInputAcquisition\([\s\S]{0,120}_firingHandIsLeft[\s\S]*bool\s+TwoHandedGrip::commitPersistentEquippedCarryInputAcquisition\([\s\S]{0,900}_persistentEquippedCarryInputAcquisitionPending\s*=\s*false[\s\S]{0,220}_hapticEvents\.firingGripAttached\s*=\s*true[\s\S]{0,220}_hapticEvents\.firingGripAttachedHandIsLeft\s*=\s*handIsLeft[\s\S]{0,220}_persistentEquippedCarryDetachArmed\s*=\s*true' `
    'A programmatic left carry must arm and emit its attach haptic on the actual firing hand only after the first physical acquisition.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'beginPersistentEquippedCarry\([\s\S]{0,2600}beginPrimaryOnlyGrip\([\s\S]{0,500}false,\s*false\)[\s\S]{0,300}_persistentEquippedCarryActive\s*=\s*true[\s\S]{0,180}_persistentEquippedCarryDetachArmed\s*=\s*false[\s\S]{0,180}_persistentEquippedCarryInputAcquisitionPending\s*=\s*true' `
    'Programmatic left carry must defer physical acquisition and its attach haptic instead of pretending that the grip was already pressed.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'auto\s+toggleOccupancyBefore\s*=\s*_twoHandedGrip\.getGripOccupancy\(\)[\s\S]{0,500}isPersistentEquippedCarryInputAcquisitionPending\(\)[\s\S]{0,320}pendingFiringOccupancy\.firingGripActive\s*=\s*false[\s\S]{0,12000}equipped_weapon_toggle_grab_policy::prepare\([\s\S]{0,800}toggleOccupancyBefore\.left[\s\S]{0,220}toggleOccupancyBefore\.right' `
    'Toggle preparation must see a programmatic left carry as unacquired until the physical firing-hand grip is pressed.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'auto\s+toggleOccupancyAfter\s*=\s*gripUpdateResult\.after[\s\S]{0,500}isPersistentEquippedCarryInputAcquisitionPending\(\)[\s\S]{0,320}pendingFiringOccupancy\.firingGripActive\s*=\s*false[\s\S]{0,500}equipped_weapon_toggle_grab_policy::reconcile\([\s\S]{0,500}toggleOccupancyAfter\.left[\s\S]{0,220}toggleOccupancyAfter\.right' `
    'Toggle reconciliation must latch programmatic left carry only in the frame where physical acquisition clears the pending gate.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'consumeToggleAcquisitionPress\s*=[\s\S]{0,500}if\s*\(!acquired\)[\s\S]{0,220}commitPersistentEquippedCarryInputAcquisition\(isLeft\)[\s\S]{0,2200}toggleReconcileDecision\.leftGripAcquired[\s\S]{0,220}toggleReconcileDecision\.rightGripAcquired' `
    'A reconciled toggle acquisition must arm persistent direct-equip and shoulder-transfer ownership for either firing hand.'

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
    'gripWeaponLocal\s*=\s*transform_math::worldPointToLocal\([\s\S]{0,120}attachedRootWorld,[\s\S]{0,120}canonicalPalmWorld\)[\s\S]{0,4500}localPointToWorld\(looseRoot->world,\s*state\.gripWeaponLocal\)' `
    'The hFRIK firing-grip point must be derived from the canonical primary attach pose and projected through the loose weapon, never derived from the tested hand.'

Reject-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'hasRightPositionOnlyHandWeaponLocal|rightPositionOnlyHandWeaponLocal|positionOnlyFrikOffsetRevision|authoredEquippedPositionOnlyCache' `
    'Loose weapon placement must never consume the equipped-created position-only cache.'

Require-Text 'src/physics-interaction/weapon/LooseWeaponGripZone.cpp' `
    'Source::AuthoredAnimation[\s\S]{0,1200}canonicalHandWeaponLocal\s*=\s*authoredLookup\.rightHandWeaponLocal[\s\S]{0,1200}tryResolveAttachedRootWorld\([\s\S]{0,700}resolveAuthoredPrimaryWeaponWorldPositionOnly[\s\S]{0,1400}authoredNativeCarrierPositionOnly[\s\S]{0,1400}authoredFullRigidFallback[\s\S]{0,6000}loosePlacementHandWeaponLocal[\s\S]{0,2500}tryBuildMirroredLeftFiringHandWeaponLocal\([\s\S]{0,600}canonicalPlacementHandWeaponLocal' `
    'Every authored loose grab must derive the first-grab native-carrier position-only hold, retain an aligned fallback, and mirror placement independently for the left hand.'

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
# through the shared firing-hold resolver.
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'loose_weapon_grip_zone::tryResolveLooseWeaponFiringHandHold\([\s\S]{0,300}multiplyTransforms\([\s\S]{0,120}handWorld,[\s\S]{0,120}transform_math::invertTransform\(handWeaponLocal\)\)' `
    'Pull-catch/force-grab must seat either hand from the resolved placement hold (weapon = hand world o inverse(hold)).'

Reject-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'authoredPositionOnly|resolveAuthoredPrimaryWeaponWorldPositionOnly\([\s\S]{0,220}rootNode->world' `
    'Loose authored seating must not preserve the incoming world rotation or discard the cached hand-alignment rotation.'

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

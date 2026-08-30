param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Require-MatchCount {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [int]$Expected,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -Raw -LiteralPath $path
    $actual = [regex]::Matches($text, $Pattern).Count
    if ($actual -ne $Expected) {
        $failures.Add("$RelativePath`: $Message (expected $Expected, found $actual)")
    }
}

Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' `
    'hmdBackRightOffsetGameUnits\{\s*14\.0f,\s*-18\.0f,\s*-6\.85f\s*\}' `
    'Detector fallback should bias the right HMD back volume farther behind the shoulder.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' `
    'hmdBackLeftOffsetGameUnits\{\s*-14\.0f,\s*-18\.0f,\s*-6\.85f\s*\}' `
    'Detector fallback should bias the left HMD back volume farther behind the shoulder.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'hmdBackEnterPaddingGameUnits\s*=\s*0\.0f' `
    'Detector fallback should use precise HMD-specific enter padding.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'hmdBackExitPaddingGameUnits\s*=\s*2\.0f' `
    'Detector fallback should use narrow HMD-specific exit padding.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' 'hmdBackMinBehindGameUnits\s*=\s*4\.0f' `
    'Detector fallback should require the hand behind the HMD for HMD stash.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'hmdBackBehindGateAllows\(\s*hmdForwardOffset,\s*input\.config\.hmdBackMinBehindGameUnits\s*\)' `
    'HMD stash detector must reject forward-side probes before sphere scoring.'
Require-Text 'src/physics-interaction/stash/ShoulderStashDetector.cpp' 'input\.config\.hmdBackExitPaddingGameUnits\s*:\s*input\.config\.hmdBackEnterPaddingGameUnits' `
    'HMD stash detector should use HMD-specific padding rather than body-zone padding.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'makeEquippedWeaponStashDetectorConfig\([^)]*\)[\s\S]*?config\.maxSpeedGameUnitsPerSecond\s*=\s*0\.0f[\s\S]*?return config' `
    'Equipped-weapon stash candidate acquisition must retain the configured anti-throw speed gate.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponDropPolicy.h' `
    'equippedWeaponShoulderStashAvailable\([\s\r\n]*bool shoulderStashConfigured[\s\S]{0,120}return shoulderStashConfigured' `
    'ROCK''s equipped-weapon shoulder stash must remain independent of addon detach capability.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equippedWeaponShoulderStashActive\s*=\s*[\s\S]{0,220}equippedWeaponShoulderStashAvailable\(\s*_equippedWeaponHandlingSettings\.equippedWeaponShoulderStashEnabled\s*\)' `
    'Runtime must derive one effective equipped-weapon shoulder gate from ROCK''s handling settings.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'settings\.primaryDetachEnabled\s*=\s*false' `
    'Only an explicit provider PrimaryDetach lease may enable equipped-weapon world dropping.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'settings\.primaryDetachEnabled\s*=[\s\r\n]+\s*settings\.primaryDetachEnabled\s*\|\|\s*enabled\([\s\r\n]*\s*provider::RockProviderEquippedWeaponHandlingFlagV1::PrimaryDetach' `
    'The provider overlay must retain its explicit PrimaryDetach capability.'
Reject-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'settings\.primaryDetachEnabled\s*=[\s\r\n]+\s*rockBaseline\.equippedWeaponShoulderStashEnabled' `
    'ROCK''s shoulder-stash baseline must never feed the physical-detach capability.'

Require-MatchCount 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equipped_weapon_shoulder::advance\s*\(' 1 `
    'Exactly one coordinator advance must own both equipped-weapon shoulder directions.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    'equipped_weapon_shoulder::RuntimeState[\s\r\n]+\s*_equippedWeaponShoulderCoordinatorState' `
    'PhysicsInteraction must retain one value-state coordinator for equipped shoulder gestures.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'advanceEquippedWeaponShoulderCoordinator\([\s\S]*?makeEquippedWeaponStashDetectorConfig\(handlingEnabled\)[\s\S]*?publishDetectorDecision[\s\S]*?equipped_weapon_shoulder::advance\(' `
    'The coordinator adapter must consume the existing equipped HMD detector decisions unchanged.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'manualStashCarryHand[\s\S]{0,900}manualCarryActive[\s\S]{0,500}attachedShoulderGestureHand' `
    'One current carry hand or attached firing hand must feed the coordinator across provider and ROCK-native modes.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'result\.decision\.action\s*!=[\s\r\n]+\s*equipped_weapon_shoulder::Action::SubmitRetrieve[\s\S]{0,1500}submitExactCurrent\([\s\r\n]*\s*currentIdentity\)' `
    'Coordinator-selected retrieval must retain exact-current native draw execution.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'if\s*\(nativeShoulderSheathRequested\)\s*\{[\s\S]*?submitEquippedWeaponShoulderSheath[\s\S]*?reportExecutionResult' `
    'Coordinator-selected sheath must execute once and report acceptance back into coordinator state.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'stashCommitSelected\s*=[\s\S]{0,220}suppressEquippedDrop[\s\S]{0,220}shouldAttemptPhysicalDrop\(stashCommitSelected\)' `
    'Provider drop routing must consume the coordinator''s fail-closed suppression decision.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_equippedWeaponShoulderGestureConsumedThisFrame\[handIndex\][\s\S]{0,900}readGrabButtonState\(isLeft,\s*grabButton\)[\s\S]{0,500}clearSelectionState\(false\)' `
    'A coordinator-owned gesture must be excluded from normal world-grab selection.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'captureCurrentIdentity[\s\S]*?submitSheatheExactCurrent\(sheathIdentity\)[\s\S]*?_equippedWeaponShoulderSheath\s*=[\s\S]*?weaponInstanceData\s*=\s*sheathIdentity\.instanceData[\s\S]*?equipIndex\s*=\s*sheathIdentity\.equipIndex[\s\S]*?weaponOwnershipKey\s*=[\s\r\n]+\s*currentEquippedWeaponOwnershipKey[\s\S]*?zone\s*=\s*stashDecision\.zone' `
    'Shoulder storage must retain exact identity, stable ownership, and the selected HMD shoulder.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shoulderOwnershipKey\s*=[\s\S]{0,500}_equippedWeaponShoulderCoordinatorState\.activeAction[\s\S]{0,250}_equippedWeaponShoulderCoordinatorState\.weaponOwnershipKey' `
    'The native hidden/draw transition must not erase coordinator gesture identity.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.h' `
    '_equippedWeaponStashCommitLeases|_equippedWeaponStashTapIntentLeases|_equippedWeaponShoulderInputGuards|_equippedWeaponSheathCommittedThisFrame|_equippedWeaponUnsheathCommittedThisFrame' `
    'Superseded equipped-weapon shoulder leases, guards, and directional flags must stay removed.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponShoulderSheathRetrieval|manualStashCommitSelected|canCommitNativeShoulderSheath|hasToggleShoulderTapIntent' `
    'The old independent sheath/retrieval evaluators must not return.'
Reject-Text 'src/physics-interaction/weapon/EquippedWeaponToggleGrabPolicy.h' `
    'ShoulderLeaseActive|shoulderLeaseActive' `
    'Shoulder proximity must not bypass the toggle latch without coordinator intent.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'RockEquippedWeaponHandlingBaseline[\s\S]{0,420}equippedWeaponShoulderStashEnabled\s*=[\s\r\n]+\s*g_rockConfig\.rockEquippedWeaponShoulderStashEnabled' `
    'The equipped-weapon handling baseline must consume ROCK''s shoulder stash switch.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'g_rockConfig\.rockRealisticWeaponHandlingEnabled' `
    'The focused ROCK sheath feature must not restore the obsolete realistic-weapons master switch.'

Require-Text 'src/physics-interaction/weapon/NativeEquippedWeaponDraw.cpp' `
    'resolveExactCurrent\(expected\)[\s\S]{0,900}shouldSubmitSheatheFollowup[\s\S]{0,1000}DrawWeaponMagicHands\(false\)[\s\S]{0,300}getNativeWeaponState\(current\.player\)' `
    'Native sheath must revalidate exact identity and observe both sides of the verified FO4VR state transition.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'for\s*\(const bool isLeft\s*:\s*\{\s*false,\s*true\s*\}\)[\s\S]*?ambidextrousHandoffEnabled[\s\S]*?PendingEquippedWeaponPrimaryOnlyGripStart[\s\S]*?\.isLeft\s*=\s*retrieveWithLeftHand' `
    'Retrieval must evaluate both eligible physical hands and carry the selected hand into weapon ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingPrimaryStartMatchesCurrentWeapon[\s\S]*?tryBuildCurrentLeftFiringGripCapture[\s\S]*?beginPrimaryOnlyGrip' `
    'A left-hand retrieval must retain the canonical lazy transfer fallback.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'submitEquippedWeaponShoulderSheath[\s\S]{0,1800}tryCaptureLeftFiringGripTransfer[\s\S]{0,2400}hasLeftFiringGripTransfer\s*=[\s\r\n]+\s*hasLeftFiringGripTransfer' `
    'Sheathing must preserve a left firing-grip transfer before native presentation clears ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'PendingEquippedWeaponPrimaryOnlyGripStart\{[\s\S]{0,500}\.committedTransfer\s*=\s*true[\s\S]{0,700}\.hasFiringHandWeaponLocal\s*=\s*retrieveWithLeftHand\s*&&[\s\S]{0,180}hasLeftFiringGripTransfer' `
    'Accepted retrieval must become a durable selected-hand transfer.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'bool\s+TwoHandedGrip::beginPrimaryOnlyGrip\([\s\S]{0,700}retainUntilPhysicalGrip[\s\S]{0,5600}_persistentEquippedCarryActive\s*=\s*true[\s\S]{0,200}_persistentEquippedCarryDetachArmed\s*=\s*false' `
    'A committed retrieval must not interpret its initiating gesture as an immediate weapon drop.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceFixedWeaponHand[\s\S]*?if\s*\(_equippedWeaponShoulderSheath\.active\s*\|\|[\s\S]*?_pendingEquippedWeaponPrimaryOnlyGripStart\.pending\)' `
    'Fixed-hand enforcement must yield while shoulder storage or selected retrieval owns the weapon.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponHandAssignment[\s\S]{0,500}if\s*\(_equippedWeaponShoulderSheath\.active\)' `
    'Provider hand assignment must not acquire a hidden shoulder-sheathed weapon.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.cpp' `
    'currentMatchesIntentionalShoulderSheath[\s\S]*?shoulderSheathFormID[\s\S]*?shoulderSheathInstanceData[\s\S]*?shoulderSheathEquipIndex[\s\S]*?intentional-shoulder-sheathe' `
    'Presentation recovery must yield only to ROCK''s exact identity-bound shoulder sheath.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'unequipEquippedWeaponFromPlayer|unequipReasonName' `
    'Equipped shoulder stash must not remove the weapon from its equipped inventory stack.'
Reject-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.h' `
    'EquippedUnequip|UnequipReason|unequipEquippedWeaponFromPlayer' `
    'The superseded inventory-unequip shoulder path must stay removed.'

Require-Text 'src/RockConfig.h' `
    'rockEquippedWeaponShoulderStashEnabled\s*=\s*true' `
    'RockConfig must default the standalone equipped-weapon sheath feature on.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*SECTION,\s*"bEquippedWeaponShoulderStashEnabled"' `
    'RockConfig must load equipped-weapon shoulder stash from [PhysicsInteraction].'
Require-Text 'data/config/ROCK_example.ini' `
    '(?m)^bEquippedWeaponShoulderStashEnabled\s*=\s*true\s*$' `
    'The reference INI must expose ROCK''s standalone equipped-weapon sheath switch.'
Require-Text 'src/RockConfig.h' `
    'rockShoulderStashHmdBackRightOffsetGameUnits\s*=\s*RE::NiPoint3\(14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig header default should match the behind-shoulder HMD right volume.'
Require-Text 'src/RockConfig.h' `
    'rockShoulderStashHmdBackLeftOffsetGameUnits\s*=\s*RE::NiPoint3\(-14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig header default should match the behind-shoulder HMD left volume.'
Require-Text 'src/RockConfig.cpp' `
    'resetToDefaults\(\)[\s\S]{0,180}static_cast<RockConfigValues&>\(\*this\)\s*=\s*RockConfigValues\{\}' `
    'RockConfig reset must restore the canonical header defaults, including both HMD back volumes.'
Require-Text 'src/RockConfig.h' 'rockShoulderStashHmdBackEnterPaddingGameUnits\s*=\s*0\.0f' `
    'RockConfig header default should expose precise HMD enter padding.'
Require-Text 'src/RockConfig.h' 'rockShoulderStashHmdBackExitPaddingGameUnits\s*=\s*2\.0f' `
    'RockConfig header default should expose precise HMD exit padding.'
Require-Text 'src/RockConfig.h' 'rockShoulderStashHmdBackMinBehindGameUnits\s*=\s*4\.0f' `
    'RockConfig header default should expose the behind-HMD gate.'
Require-Text 'src/RockConfig.cpp' 'fShoulderStashHmdBackEnterPaddingGameUnits' `
    'RockConfig loader should read HMD-specific enter padding.'
Require-Text 'src/RockConfig.cpp' 'fShoulderStashHmdBackExitPaddingGameUnits' `
    'RockConfig loader should read HMD-specific exit padding.'
Require-Text 'src/RockConfig.cpp' 'fShoulderStashHmdBackMinBehindGameUnits' `
    'RockConfig loader should read the behind-HMD gate.'

Reject-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' '17\.5f,\s*-5\.0f,\s*-6\.85f' `
    'Detector fallback must not keep the old front-biased HMD back volume.'
Reject-Text 'src/physics-interaction/stash/ShoulderStashDetector.h' '14\.0f,\s*-12\.0f,\s*-6\.85f' `
    'Detector fallback must not keep the previous still-too-forward HMD back volume.'
Reject-Text 'src/RockConfig.h' '17\.5f,\s*-5\.0f,\s*-6\.85f' `
    'RockConfig header default must not keep the old front-biased HMD back volume.'
Reject-Text 'src/RockConfig.h' '14\.0f,\s*-12\.0f,\s*-6\.85f' `
    'RockConfig header default must not keep the previous still-too-forward HMD back volume.'
if ($failures.Count -gt 0) {
    Write-Host 'ShoulderStashHmdBackVolumeSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ShoulderStashHmdBackVolumeSourceTests passed.' -ForegroundColor Green

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
    'equippedWeaponShoulderStashAvailable[\s\S]{0,240}primaryDetachEnabled\s*&&\s*shoulderStashConfigured' `
    'Equipped-weapon shoulder stash must require addon-owned detach authority and its own setting.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equippedWeaponShoulderStashActive\s*=\s*[\s\S]{0,220}equippedWeaponShoulderStashAvailable\(\s*_equippedWeaponHandlingSettings\.primaryDetachEnabled,\s*_equippedWeaponHandlingSettings\.equippedWeaponShoulderStashEnabled\s*\)' `
    'Runtime must derive one effective equipped-weapon stash gate from the leased addon policy.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'stashCarryHand\s*=\s*equippedWeaponShoulderStashActive\s*\?[\s\S]{0,500}SourceHand::None' `
    'Disabled equipped-weapon stash must skip carry-hand acquisition.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '!stashCarryEligible[\s\S]{0,300}resetRuntime\(stashState\)[\s\S]{0,120}commitLease\s*=\s*\{\}' `
    'Disabled equipped-weapon stash must clear both dwell and fast-release commit-lease state.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'stashCommitSelected\s*=[\s\S]{0,180}equippedWeaponShoulderStashActive\s*&&[\s\S]{0,180}confirmedForCommit' `
    'Final equipped-weapon sheath commit must use the same addon authority gate.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'g_rockConfig\.rockEquippedWeaponShoulderStashEnabled|g_rockConfig\.rockRealisticWeaponHandlingEnabled' `
    'The detector must not retain removed ROCK-owned immersive-weapon config gates.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldArmEquippedWeaponFastReleaseCommitLease[\s\S]*?currentEquippedWeaponOwnershipKey[\s\S]*?equippedWeaponFastReleaseCommitLeaseIsUsable' `
    'Fast release may bridge the release debounce only through an ownership-bound, spatially revalidated commit lease.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shoulder sheathe failed[\s\S]*?physical drop is suppressed[\s\S]*?shouldAttemptPhysicalDrop\(stashCommitSelected\)' `
    'A selected equipped-weapon stash must fail closed instead of falling through to a physical drop.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'captureCurrentIdentity[\s\S]*?submitSheatheExactCurrent\(sheathIdentity\)[\s\S]*?_equippedWeaponShoulderSheath\s*=[\s\S]*?weaponInstanceData\s*=\s*sheathIdentity\.instanceData[\s\S]*?equipIndex\s*=\s*sheathIdentity\.equipIndex[\s\S]*?zone\s*=\s*stashDecision\.zone' `
    'Shoulder stash must sheath and retain the exact equipped identity plus the confirmed shoulder zone.'
Require-Text 'src/physics-interaction/weapon/NativeEquippedWeaponDraw.cpp' `
    'resolveExactCurrent\(expected\)[\s\S]{0,900}shouldSubmitSheatheFollowup[\s\S]{0,500}DrawWeaponMagicHands\(false\)[\s\S]{0,300}getNativeWeaponState\(current\.player\)' `
    'Native sheath must revalidate exact identity and observe both sides of the verified FO4VR state transition.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponShoulderSheathRetrieval[\s\S]*?observedWeaponFormID\s*==\s*currentIdentity\.formID[\s\S]*?decision\.zone\s*==\s*_equippedWeaponShoulderSheath\.zone[\s\S]*?isRawButtonPhysicallyHeld[\s\S]*?selectShoulderRetrievalHand[\s\S]*?submitExactCurrent\(\s*currentIdentity\s*\)' `
    'Retrieval must require the same exact equipped identity, same stored shoulder, physical squeeze, and exact native draw.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'for\s*\(const bool isLeft\s*:\s*\{\s*false,\s*true\s*\}\)[\s\S]*?ambidextrousHandoffEnabled[\s\S]*?PendingEquippedWeaponPrimaryOnlyGripStart[\s\S]*?\.isLeft\s*=\s*retrieveWithLeftHand' `
    'Retrieval must evaluate both physical hands and carry the selected hand into equipped-weapon ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingPrimaryStartMatchesCurrentWeapon[\s\S]*?tryBuildCurrentLeftFiringGripCapture[\s\S]*?beginPrimaryOnlyGrip' `
    'A left-hand unsheath must lazily build the canonical mirrored firing-grip capture before ownership begins.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_equippedWeaponUnsheathCommittedThisFrame\[handIndex\][\s\S]{0,900}readGrabButtonState\(isLeft,\s*grabButton\)[\s\S]{0,500}clearSelectionState\(false\)' `
    'The retrieval squeeze must be consumed and excluded from normal world-grab selection in its commit frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'firingGripOwnershipFeatureAvailable\s*=\s*equipped_weapon_manual_ownership_policy::featureAvailable\(\s*!_equippedWeaponShoulderSheath\.active[\s\S]{0,500}primaryDetachFeatureAvailable\s*=\s*equipped_weapon_manual_ownership_policy::featureAvailable\(\s*!_equippedWeaponShoulderSheath\.active' `
    'Ordinary firing-grip and detach input must remain disabled during the shoulder retrieval dwell.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceFixedWeaponHand[\s\S]*?if\s*\(_equippedWeaponShoulderSheath\.active\s*\|\|[\s\S]*?_pendingEquippedWeaponPrimaryOnlyGripStart\.pending\)' `
    'Fixed-hand enforcement must yield while a shoulder sheath or selected retrieval owns the weapon.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponHandAssignment[\s\S]{0,500}if\s*\(_equippedWeaponShoulderSheath\.active\)' `
    'Pip-Boy/provider hand assignment must not acquire a hidden shoulder-sheathed weapon.'
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
    'rockShoulderStashHmdBackRightOffsetGameUnits\s*=\s*RE::NiPoint3\(14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig header default should match the behind-shoulder HMD right volume.'
Require-Text 'src/RockConfig.h' `
    'rockShoulderStashHmdBackLeftOffsetGameUnits\s*=\s*RE::NiPoint3\(-14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig header default should match the behind-shoulder HMD left volume.'
Require-Text 'src/RockConfig.cpp' `
    'rockShoulderStashHmdBackRightOffsetGameUnits\s*=\s*RE::NiPoint3\(14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig reset default should match the behind-shoulder HMD right volume.'
Require-Text 'src/RockConfig.cpp' `
    'rockShoulderStashHmdBackLeftOffsetGameUnits\s*=\s*RE::NiPoint3\(-14\.0f,\s*-18\.0f,\s*-6\.85f\)' `
    'RockConfig reset default should match the behind-shoulder HMD left volume.'
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
Reject-Text 'src/RockConfig.cpp' '17\.5f,\s*-5\.0f,\s*-6\.85f' `
    'RockConfig reset default must not keep the old front-biased HMD back volume.'
Reject-Text 'src/RockConfig.cpp' '14\.0f,\s*-12\.0f,\s*-6\.85f' `
    'RockConfig reset default must not keep the previous still-too-forward HMD back volume.'

if ($failures.Count -gt 0) {
    Write-Host 'ShoulderStashHmdBackVolumeSourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'ShoulderStashHmdBackVolumeSourceTests passed.' -ForegroundColor Green

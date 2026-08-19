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

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equippedWeaponShoulderStashActive\s*=\s*[\s\S]{0,220}equippedWeaponShoulderStashAvailable\(\s*_equippedWeaponHandlingSettings\.equippedWeaponShoulderStashEnabled\s*\)' `
    'Runtime must derive one effective equipped-weapon stash gate from ROCK''s handling settings.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'nativeShoulderGestureAvailable\s*=[\s\S]{0,180}equippedWeaponShoulderStashActive\s*&&[\s\S]{0,180}!_equippedWeaponHandlingSettings\.primaryDetachEnabled[\s\S]{0,600}nativeShoulderGestureHand' `
    'ROCK''s attached shoulder gesture must use the current firing hand only when addon detach is off.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '!stashCarryEligible[\s\S]{0,300}resetRuntime\(stashState\)[\s\S]{0,120}commitLease\s*=\s*\{\}' `
    'Disabled equipped-weapon stash must clear both dwell and fast-release commit-lease state.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'manualStashCommitSelected\s*=[\s\S]{0,180}equippedWeaponShoulderStashActive\s*&&[\s\S]{0,180}confirmedForCommit[\s\S]{0,220}stashCommitSelected\s*=[\s\S]{0,120}nativeShoulderSheathSelected\s*\|\|[\s\S]{0,120}manualStashCommitSelected' `
    'Final sheath selection must combine the ROCK-native shoulder release and provider manual-carry release without allowing a drop fallthrough.'




Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'canCommitNativeShoulderSheath[\s\S]{0,1800}primaryState\.released[\s\S]*?_equippedWeaponSheathCommittedThisFrame\[sheathHandIndex\]\s*=\s*true[\s\S]{0,500}submitEquippedWeaponShoulderSheath' `
    'ROCK must commit only the firing hand''s in-zone release and consume that edge before normal world-grab handling.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'RockEquippedWeaponHandlingBaseline[\s\S]{0,240}equippedWeaponShoulderStashEnabled\s*=[\s\r\n]+\s*g_rockConfig\.rockEquippedWeaponShoulderStashEnabled' `
    'The equipped-weapon handling baseline must consume ROCK''s shoulder stash switch.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'g_rockConfig\.rockRealisticWeaponHandlingEnabled' `
    'The focused ROCK sheath feature must not restore the obsolete realistic-weapons master switch.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldArmEquippedWeaponFastReleaseCommitLease[\s\S]*?currentEquippedWeaponOwnershipKey[\s\S]*?equippedWeaponFastReleaseCommitLeaseIsUsable' `
    'Fast release may bridge the release debounce only through an ownership-bound, spatially revalidated commit lease.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'stashCommitSelected\s*=[\s\S]{0,1400}shouldAttemptPhysicalDrop\(stashCommitSelected\)' `
    'Physical-drop routing must consume the combined stash selection.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shoulder sheathe failed[\s\S]{0,600}physical drop is suppressed' `
    'A selected equipped-weapon stash must fail closed when native sheathing fails.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'captureCurrentIdentity[\s\S]*?submitSheatheExactCurrent\(sheathIdentity\)[\s\S]*?_equippedWeaponShoulderSheath\s*=[\s\S]*?weaponInstanceData\s*=\s*sheathIdentity\.instanceData[\s\S]*?equipIndex\s*=\s*sheathIdentity\.equipIndex[\s\S]*?zone\s*=\s*stashDecision\.zone' `
    'Shoulder stash must sheath and retain the exact equipped identity plus the confirmed shoulder zone.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponShoulderSheathRetrieval[\s\S]*?observedWeaponFormID\s*==\s*currentIdentity\.formID[\s\S]*?decision\.zone\s*==\s*_equippedWeaponShoulderSheath\.zone[\s\S]*?isRawButtonPhysicallyHeld[\s\S]*?selectShoulderRetrievalHand[\s\S]*?submitExactCurrent\(\s*currentIdentity\s*\)' `
    'Retrieval must require the same exact equipped identity, same stored shoulder, physical squeeze, and exact native draw.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'advanceShoulderDrawWait\([\s\S]*?ShoulderDrawAction::CommitRetrieval[\s\S]*?commitRetrieval\([\s\S]*?ShoulderDrawAction::RestartGesture[\s\S]*?restartRetrievalGesture\([\s\S]*?ShoulderDrawAction::SubmitDraw[\s\S]*?submitExactCurrent\([\s\S]*?nativeStateAcknowledgesShoulderDraw' `
    'A shoulder draw submission must remain identity-bound and retryable until native state acknowledges it.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'beginShoulderDrawWait\([\s\S]{0,500}retaining transaction until native acknowledgement' `
    'An unacknowledged first draw submission must preserve the shoulder transaction instead of committing retrieval.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'physical-hand-unsheath-committed' `
    'Shoulder retrieval must not treat a void native draw submission as an acknowledgement.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'for\s*\(const bool isLeft\s*:\s*\{\s*false,\s*true\s*\}\)[\s\S]*?ambidextrousHandoffEnabled[\s\S]*?selectShoulderRetrievalHand' `
    'Retrieval must evaluate both physical hands before selecting ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'commitRetrieval[\s\S]*?PendingEquippedWeaponPrimaryOnlyGripStart[\s\S]*?\.isLeft\s*=\s*retrieveWithLeftHand' `
    'Acknowledged retrieval must carry the selected hand into equipped-weapon ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingPrimaryStartMatchesCurrentWeapon[\s\S]*?tryBuildCurrentLeftFiringGripCapture[\s\S]*?beginPrimaryOnlyGrip' `
    'A left-hand unsheath must retain the canonical lazy fallback when no pre-sheath transfer frame is available.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'submitEquippedWeaponShoulderSheath[\s\S]{0,1800}tryCaptureLeftFiringGripTransfer[\s\S]{0,2200}hasLeftFiringGripTransfer\s*=[\s\r\n]+\s*hasLeftFiringGripTransfer' `
    'Sheathing must capture and retain a left firing-grip frame before native presentation clears live ownership.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'PendingEquippedWeaponPrimaryOnlyGripStart\{[\s\S]{0,500}\.committedTransfer\s*=\s*true[\s\S]{0,700}\.hasFiringHandWeaponLocal\s*=\s*retrieveWithLeftHand\s*&&[\s\S]{0,180}hasLeftFiringGripTransfer' `
    'An accepted shoulder draw must become a durable selected-hand transfer carrying the pre-sheath left frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldStartPendingPrimaryOnlyGrip\([\s\S]{0,320}committedTransfer[\s\S]{0,6000}beginPrimaryOnlyGrip\([\s\S]{0,700}committedTransfer' `
    'Committed shoulder retrieval must start without a still-held squeeze and protect the carry until physical grip input resumes.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    '_equippedWeaponSheathCommittedThisFrame\[handIndex\]\s*\|\|[\s\S]{0,120}_equippedWeaponUnsheathCommittedThisFrame\[handIndex\][\s\S]{0,900}readGrabButtonState\(isLeft,\s*grabButton\)[\s\S]{0,500}clearSelectionState\(false\)' `
    'Both sheath release and retrieval squeeze must be consumed and excluded from normal world-grab selection in their commit frame.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'firingGripOwnershipFeatureAvailable\s*=\s*equipped_weapon_manual_ownership_policy::featureAvailable\(\s*!_equippedWeaponShoulderSheath\.active[\s\S]{0,500}primaryDetachFeatureAvailable\s*=\s*equipped_weapon_manual_ownership_policy::featureAvailable\(\s*!_equippedWeaponShoulderSheath\.active' `
    'Ordinary firing-grip and detach input must remain disabled during the shoulder retrieval dwell.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceFixedWeaponHand[\s\S]*?if\s*\(_equippedWeaponShoulderSheath\.active\s*\|\|[\s\S]*?_pendingEquippedWeaponPrimaryOnlyGripStart\.pending\)' `
    'Fixed-hand enforcement must yield while a shoulder sheath or selected retrieval owns the weapon.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'serviceEquippedWeaponHandAssignment[\s\S]{0,500}if\s*\(_equippedWeaponShoulderSheath\.active\)' `
    'Pip-Boy/provider hand assignment must not acquire a hidden shoulder-sheathed weapon.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'unequipEquippedWeaponFromPlayer|unequipReasonName' `
    'Equipped shoulder stash must not remove the weapon from its equipped inventory stack.'


Require-Text 'src/RockConfig.h' `
    'rockEquippedWeaponShoulderStashEnabled\s*=\s*true' `
    'RockConfig must default the standalone equipped-weapon sheath feature on.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*SECTION,\s*"bEquippedWeaponShoulderStashEnabled"' `
    'RockConfig must load equipped-weapon shoulder stash from [PhysicsInteraction].'
Require-Text 'data/config/ROCK.ini' `
    '(?m)^bEquippedWeaponShoulderStashEnabled\s*=\s*true\s*$' `
    'The reference INI must expose ROCK''s standalone equipped-weapon sheath switch.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' `
    '(?m)^bEquippedWeaponShoulderStashEnabled\s*=\s*true\s*$' `
    'The packaged INI must expose ROCK''s standalone equipped-weapon sheath switch.'
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

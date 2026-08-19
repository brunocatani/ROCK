param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) { $failures.Add($Message) }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) { $failures.Add($Message) }
}

Require-Text 'src/RockConfig.h' `
    'rockLeftHandedMode\s*=\s*false' `
    'Base ROCK must expose one fixed-hand preference, defaulting to physical right.'
Require-Text 'src/RockConfig.cpp' `
    'WEAPON_HANDEDNESS_SECTION\s*=\s*"WeaponHandedness"[\s\S]*GetBoolValue\(\s*WEAPON_HANDEDNESS_SECTION,\s*"bLeftHandedMode"' `
    'The fixed ROCK-exclusive hand preference must load only from [WeaponHandedness].'

foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    $configText = Get-Content -Raw -LiteralPath (Join-Path $Root $configPath)
    $handednessMatch = [regex]::Match($configText, '(?ms)^\[WeaponHandedness\]\s*(?<body>.*?)(?=^\[[^\]]+\])')
    if (!$handednessMatch.Success -or
        $handednessMatch.Groups['body'].Value -notmatch '(?m)^bLeftHandedMode\s*=\s*false\s*$') {
        $failures.Add("$configPath`: ROCK fixed handedness must be under [WeaponHandedness] and default right.")
    }
    if ($configText -match '(?m)^b(?:MenuTriggerHandEquipEnabled|EquipPreferredHandLeft|PipboyTriggerHandEquipEnabled|PipboyPreferredHandLeft)\s*=') {
        $failures.Add("$configPath`: addon-owned Pip-Boy hand-selection keys must not remain in ROCK.")
    }
}















Require-Text 'src/physics-interaction/input/InputRemapRuntime.cpp' `
    's_pipboyMenuGeneration[\s\S]*rawTransition\.pressedEdges\s*\|\s*rawTransition\.releasedEdges[\s\S]*publishPipboyTriggerTransition\(hand\)[\s\S]*consumePipboyEquipTriggerResolution' `
    'Pip-Boy trigger evidence must be captured before blocking-menu gameplay edges are cleared and consumed per selection.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'inspectStack\(\s*assignment\.handleId,\s*assignment\.stackId[\s\S]*equippedWeapon->formID\s*==\s*assignment\.formId[\s\S]*beginPersistentEquippedCarry[\s\S]*left-carry-resolve-timeout' `
    'Left carry acquisition must bind exact inventory and equipped-weapon identities and fail closed to right after a bounded retry.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'f4vr::isNodeVisible\(weaponNode\)[\s\S]*nativeOffsetSample\s*=\s*weaponNode->local[\s\S]*advanceNativeOffsetReadiness[\s\S]*beginPersistentEquippedCarry' `
    'Direct left carry must wait for a visible, stable hFRIK-owned offset and a reserved canonical-refresh frame before ownership transfer.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'resolveEquipMode\([\s\S]{0,260}handlingSettings\.externalAuthorityActive[\s\S]{0,180}handlingSettings\.pipboyTriggerHandEquipEnabled[\s\S]{0,180}_fixedFiringHandIsLeft[\s\S]{0,300}pipboyAssignmentManaged\s*=[\s\S]{0,180}managesHandAssignment\(equipMode\)[\s\S]{0,700}consumeSelectionEvent\([\s\S]{0,900}clearEquippedWeaponHandAssignment\([\s\S]{0,120}"native-right-preference"[\s\S]{0,120}true\)' `
    'Native-right preference must drain stale selection intent, while fixed-left comes from ROCK and trigger-hand mode only from addon authority.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'commitRight\s*=\s*\[&\]\(const char\* reason\)[\s\S]{0,900}restoreNativeRightEquippedCarry\(reason\)[\s\S]{0,1400}pipboy_equip_policy::Hand::Right' `
    'A fresh right-trigger assignment must restore native-right ownership before publishing the right side.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'shouldReacquirePersistentLeftCarry\([\s\S]{0,500}isManualOwnershipActive\(\)[\s\S]*assignment\.assignedLeft\s*=\s*currentLeft' `
    'Persistent left reacquisition must not fight manual ownership, and deliberate handovers must replace the durable assigned side.'

$physicsInteractionText = Get-Content -Raw -LiteralPath (Join-Path $Root 'src/physics-interaction/core/PhysicsInteraction.cpp')
$assignmentServiceCall = $physicsInteractionText.IndexOf('serviceEquippedWeaponHandAssignment(')
$gripUpdateCall = if ($assignmentServiceCall -ge 0) {
    $physicsInteractionText.IndexOf('_twoHandedGrip.update(', $assignmentServiceCall)
} else {
    -1
}
if ($assignmentServiceCall -lt 0 -or $gripUpdateCall -lt 0 -or $assignmentServiceCall -gt $gripUpdateCall) {
    $failures.Add('Pip-Boy assignment readiness must be serviced before TwoHandedGrip update so the first offset match reserves a full canonical refresh.')
}

if ($failures.Count -gt 0) {
    foreach ($failure in $failures) { Write-Error $failure }
    exit 1
}

Write-Host 'Pip-Boy trigger-hand equip source boundaries passed.'

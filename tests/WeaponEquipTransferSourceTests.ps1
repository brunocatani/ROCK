param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}





Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'observedEquipped=\{:08X\}' `
    'Auto-equip logging must include the observed equipped form for mismatch diagnosis.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'DrawWeaponMagicHands\s*\(\s*true' `
    'Held equip must not submit an uncoordinated native draw from the transfer callsite.'





















Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equipResult\.success[\s\S]{0,500}_equippedWeaponTransition\.beginHeldTransition[\s\S]{0,500}requestedInstanceData' `
    'Every accepted held equip must arm the shared exact-instance transition coordinator.'

Require-Text 'src/ROCKMain.cpp' `
    'updateEquippedWeaponTransition\(\);[\s\S]{0,500}updateAuthoredPrimaryFiringGrip\(\);[\s\S]{0,180}update\(\);' `
    'Native weapon presentation recovery must run before authored grip and the normal ROCK frame.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'requestWeaponCollisionRebuildAfterWorkbenchExit[\s\S]{0,400}requestCurrentWeaponReconcile[\s\S]{0,180}WorkbenchExit' `
    'Workbench exit must use the shared transition coordinator instead of a timer-only collision repair.'




















Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'const bool nativeWeaponAnimationActive\s*=[\s\S]*?currentNativeAnimationAuthorityFlagsV1\(\)[\s\S]*?GUN_STATE::kReloading[\s\S]*?\.nativeWeaponAnimationActive' `
    'Equip recovery must yield during provider-owned and base-game reload presentation windows.'



Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'const auto previousNativeInstanceNode\s*=[\s\S]{0,400}equipped_weapon_visual_state::observe[\s\S]*\.previousNativeInstanceNode\s*=[\s\S]{0,180}previousNativeInstanceNode' `
    'Held equip must carry the pre-request native scene witness into exact-instance reconciliation.'



Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingPrimaryStartMatchesCurrentWeapon[\s\S]{0,500}matchesExpectedIdentity[\s\S]{0,400}targetWeaponInstanceData[\s\S]*remainingSeconds\s*=\s*10\.0f' `
    'Deferred manual hand ownership must bind to the accepted instance or a changed native clone and expire if it never commits.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equipVisualBridgeEnabled' `
    'Core equip continuity must not depend on an addon authority flag.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'canBeginEquip\(nativeStateBeforeEquip\)[\s\S]{0,900}shouldRearmTrigger\(nativeStateBeforeEquip,\s*triggeredByInput\)[\s\S]*hand\.captureHeldReleaseMotion' `
    'A native weapon transition must defer before physical release and preserve the same-hand trigger request.'





Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'releaseGrabbedObject[\s\S]{0,320}transferHeldWeaponToPlayerAndEquip[\s\S]{0,180}releaseOutcome\.takeRetainedReference\(\)[\s\S]*postEquipRef\s*=\s*equipResult\.untransferredRef\.get\(\)' `
    'The handoff caller must move release ownership into the transaction and use its failure pin for events.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon equip transfer source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon equip transfer source boundary passed.'

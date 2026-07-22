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

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.h' `
    'EquipAcceptedPending' `
    'Held weapon equip transfer must represent accepted asynchronous native equips.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'const auto equippedAfter = readEquippedWeaponSnapshot\(\);[\s\S]{0,260}observedEquippedFormID[\s\S]{0,260}result\.success = true;[\s\S]{0,260}result\.committed[\s\S]{0,260}EquipReason::EquipAcceptedPending' `
    'Held weapon equip transfer must distinguish accepted requests from same-frame native commit.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'observedEquipped=\{:08X\}' `
    'Auto-equip logging must include the observed equipped form for mismatch diagnosis.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'DrawWeaponMagicHands\s*\(\s*true' `
    'Held equip must not submit an uncoordinated native draw from the transfer callsite.'

Require-Text 'src/physics-interaction/weapon/NativeEquippedWeaponDraw.cpp' `
    'object->formID != expected\.formID[\s\S]{0,300}instanceData\) != expected\.instanceData[\s\S]{0,300}equipIndex\.index != expected\.equipIndex[\s\S]{0,1200}DrawWeaponMagicHands\(true\)' `
    'Native draw recovery must revalidate the exact current form, instance, and equip index before submission.'

Require-Text 'src/physics-interaction/weapon/EquippedWeaponTransitionPolicy.h' `
    'kMaximumDrawAttempts[\s\S]{0,10000}state\.drawAttempts < kMaximumDrawAttempts[\s\S]{0,500}RepairAction::RequestDraw[\s\S]{0,300}RepairAction::DrawExhausted' `
    'Native draw recovery must remain bounded inside the shared transition policy.'

Require-Text 'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.cpp' `
    'RepairAction::RequestDraw[\s\S]{0,500}native_equipped_weapon_draw::submitExactCurrent' `
    'Every draw retry must pass through the exact-identity transition coordinator.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'inventoryBeforeTransfer\s*=\s*captureWeaponStacks[\s\S]{0,1800}ActivateRef\([\s\S]{0,1800}untransferredRef\.reset\(\);[\s\S]{0,500}inventoryAfterTransfer\s*=\s*captureWeaponStacks[\s\S]{0,500}selectTransferredStack' `
    'Held pickup must select the acquired stack from a pre/post inventory differential.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equipResult\.success[\s\S]{0,500}_equippedWeaponTransition\.beginHeldTransition[\s\S]{0,500}requestedInstanceData' `
    'Every accepted held equip must arm the shared exact-instance transition coordinator.'

Require-Text 'src/ROCKMain.cpp' `
    'updateEquippedWeaponTransition\(\);[\s\S]{0,500}updateAuthoredPrimaryFiringGrip\(\);[\s\S]{0,180}update\(\);' `
    'Native weapon presentation recovery must run before authored grip and the normal ROCK frame.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'requestWeaponCollisionRebuildAfterWorkbenchExit[\s\S]{0,400}requestCurrentWeaponReconcile[\s\S]{0,180}WorkbenchExit' `
    'Workbench exit must use the shared transition coordinator instead of a timer-only collision repair.'

Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' `
    'maybeFireWorkbenchWeaponReattach|WORKBENCH-REATTACH' `
    'The superseded debug-only workbench reattach path must stay removed.'

Require-Text 'src/physics-interaction/weapon/NativeEquippedWeaponAttach.cpp' `
    'RUNTIME_VR_1_2_72[\s\S]{0,700}kExpectedAttachEntry[\s\S]*object->formID != expected\.formID[\s\S]{0,220}instanceData\) != expected\.instanceData[\s\S]{0,900}using QueueAttach = void \(\*\)' `
    'Native attach recovery must validate FO4VR 1.2.72, exact current identity, verified bytes, and the wrapper void ABI.'

Require-Text 'src/physics-interaction/weapon/EquipVisualBridge.cpp' `
    'hideModelForNativeStandby[\s\S]*synchronizeNativeInstanceCull[\s\S]*restoreNativeInstanceCull' `
    'The visual bridge must retain a hidden standby and restore every exact-child cull.'

Require-Text 'src/physics-interaction/weapon/EquipVisualBridge.cpp' `
    'native-standby-republish-failed[\s\S]{0,700}applyExternalHandWorldTransform' `
    'The standby bridge must keep the authored finger and hand-transform payload alive until the equipped owner acquires it.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'const bool nativeWeaponAnimationActive\s*=[\s\S]{0,300}currentNativeAnimationAuthorityFlagsV1\(\)[\s\S]{0,300}GUN_STATE::kReloading[\s\S]{0,700}\.nativeWeaponAnimationActive' `
    'Equip recovery must yield during provider-owned and base-game reload presentation windows.'

Require-Text 'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.cpp' `
    'nativeWeaponAnimationActive[\s\S]{0,700}completeHandPoseHandoff[\s\S]{0,500}presentModel\s*=\s*false' `
    'The transition coordinator must park the phantom and release its hand pose during native weapon animation authority.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'const auto previousNativeInstanceNode\s*=[\s\S]{0,400}equipped_weapon_visual_state::observe[\s\S]*\.previousNativeInstanceNode\s*=[\s\S]{0,180}previousNativeInstanceNode' `
    'Held equip must carry the pre-request native scene witness into exact-instance reconciliation.'

Require-Text 'src/physics-interaction/weapon/EquippedWeaponVisualState.cpp' `
    'excludedInstanceAddress[\s\S]*reinterpret_cast<std::uintptr_t>\(node\)\s*!=' `
    'Visual reconciliation must exclude a stale same-base native scene instance while locating its replacement.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'pendingPrimaryStartMatchesCurrentWeapon[\s\S]{0,500}matchesExpectedIdentity[\s\S]{0,400}targetWeaponInstanceData[\s\S]*remainingSeconds\s*=\s*10\.0f' `
    'Deferred manual hand ownership must bind to the accepted instance or a changed native clone and expire if it never commits.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'equipVisualBridgeEnabled' `
    'Core equip continuity must not depend on an addon authority flag.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'canBeginEquip\(nativeStateBeforeEquip\)[\s\S]{0,900}shouldRearmTrigger\(nativeStateBeforeEquip,\s*triggeredByInput\)[\s\S]*hand\.captureHeldReleaseMotion' `
    'A native weapon transition must defer before physical release and preserve the same-hand trigger request.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.h' `
    'struct\s+EquipInput[\s\S]{0,300}NiPointer<RE::TESObjectREFR>\s+heldRef[\s\S]*struct\s+EquipResult[\s\S]{0,2000}NiPointer<RE::TESObjectREFR>\s+untransferredRef' `
    'The equip transaction must own the released reference and return it only when native pickup did not acquire it.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'transferHeldWeaponToPlayerAndEquip\(EquipInput input\)[\s\S]{0,300}result\.untransferredRef\s*=\s*std::move\(input\.heldRef\)[\s\S]*ActivateRef\([\s\S]{0,1400}result\.untransferredRef\.reset\(\);[\s\S]*EquipObject\(' `
    'ROCK must release its world-reference lease after ActivateRef and before EquipObject.'

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

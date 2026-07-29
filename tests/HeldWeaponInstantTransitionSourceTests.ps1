param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
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
    param([string]$Path, [string]$Pattern, [string]$Message)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }
    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

$nativePath = 'src/physics-interaction/native/HeldWeaponInstantTransition.cpp'
$nativeHeaderPath = 'src/physics-interaction/native/HeldWeaponInstantTransition.h'
$coordinatorPath = 'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.cpp'
$physicsPath = 'src/physics-interaction/core/PhysicsInteraction.cpp'

Require-Text $nativePath `
    'kPlayerDrawWeaponEntry\s*=\s*0x0F78D10[\s\S]*kEquipManagerDrawCallsite\s*=\s*0x0E107A1[\s\S]*kEquipManagerDrawReturn\s*=\s*0x0E107A7[\s\S]*kEquipManagerSheatheCallsite\s*=\s*0x0E10988[\s\S]*kEquipManagerSheatheReturn\s*=\s*0x0E1098E' `
    'The focused native owner must retain the independently verified FO4VR interceptor addresses.'

Require-Text $nativePath `
    'kExpectedPlayerDrawEntry[\s\S]*0x48, 0x89, 0x74, 0x24, 0x18[\s\S]*kExpectedEquipManagerVirtualCall[\s\S]*0xFF, 0x90, 0x48, 0x06, 0x00, 0x00' `
    'The interceptor entry and manager callsites must be protected by exact runtime byte contracts.'

Require-Text $nativePath `
    'entry_trampoline_hook::install\([\s\S]{0,600}kPlayerDrawWeaponEntry[\s\S]{0,300}onDrawWeaponMagicHands' `
    'The draw interceptor must reuse the byte-validated shared entry trampoline.'

Require-Text $nativePath `
    'thread_local\s+TransactionScope\*\s+t_activeScope[\s\S]*_ReturnAddress\(\)[\s\S]*classifyHookCall[\s\S]*PassThroughPlayerMismatch[\s\S]*s_originalDrawWeaponMagicHands' `
    'The interceptor must be transaction-local, exact-caller gated, and pass through mismatches.'

Require-Text $nativePath `
    'equipImmediatelyWithoutActions[\s\S]{0,2600}ScopeLease[\s\S]{0,900}EquipObject\([\s\S]{0,350}false,[\s\S]{0,80}false,[\s\S]{0,80}false,[\s\S]{0,80}true,[\s\S]{0,80}false\)' `
    'The native transaction must own exactly one silent immediate manager call inside the RAII scope.'

Require-Text $nativePath `
    'equipImmediatelyWithoutActions[\s\S]{0,3200}isValidEquipActionTrace[\s\S]{0,500}ImmediateEquipCode::Accepted' `
    'The immediate manager transaction must fail closed on its exact action trace before reporting acceptance.'

Require-Text $nativeHeaderPath `
    'bool success\(\) const noexcept[\s\S]{0,160}return code == ImmediateEquipCode::Accepted;' `
    'Immediate transaction success must depend on the validated manager transaction, not a later presentation permit.'

Require-Text $physicsPath `
    'readinessFor\(player\)[\s\S]{0,3000}hand\.captureHeldReleaseMotion[\s\S]*beginHeldTransition[\s\S]{0,4000}equipResult\.success\s*&&\s*pendingGripStart\.pending[\s\S]{0,1600}_pendingEquippedWeaponPrimaryOnlyGripStart\s*=\s*pendingGripStart' `
    'Held equip must preflight before release, retain its bridge, and arm the captured physical hand at exact transaction success.'

Require-Text $coordinatorPath `
    'equipped_weapon_transition_policy::advance[\s\S]{0,3000}RepairAction::RequestDraw[\s\S]{0,500}native_equipped_weapon_draw::submitExactCurrent[\s\S]*_bridge\.update' `
    'Normal exact-current native draw recovery and bridge presentation must remain owned by the transition coordinator.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'equipImmediatelyWithoutActions[\s\S]{0,1800}readEquippedWeaponSnapshot[\s\S]{0,900}findEquippedWeaponStack[\s\S]{0,700}ActivateRefThenInstantEquip' `
    'Transfer success must require the scoped manager call, exact equipped identity, and exact equipped stack.'

Reject-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'equipManager->EquipObject\(' `
    'Held weapon transfer must not bypass the scoped native transaction with a direct or queued manager call.'

Reject-Text $nativePath `
    '0x0DBE590|0x0DBE6D0|CompleteWeaponDraw|WeaponBeginDraw|WeaponBeginSheathe|PlayerFastEquipSound' `
    'The held equip transaction must not directly run native completion or add global animation/sound suppression.'

foreach ($path in @($nativeHeaderPath, $nativePath, $physicsPath, $coordinatorPath)) {
    Reject-Text $path `
        'completionPermit|CompletionResult|CompletionCode|completeDrawForExactCurrent|synchronizeAfterInstantCompletion|failHeldCompletion' `
        "Direct native completion surface must remain absent from $path."
}

Require-Text 'src/ROCKMain.cpp' `
    'Install held weapon instant-transition capability[\s\S]{0,300}held_weapon_instant_transition::install\(\)[\s\S]{0,300}Held trigger/grip-zone equip disabled' `
    'Startup must install the optional capability and continue ROCK with held auto-equip disabled on failure.'

if ($failures.Count -gt 0) {
    Write-Host 'Held weapon instant transition source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Held weapon instant transition source boundary passed.'

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

Require-Text $nativePath `
    'kPlayerDrawWeaponEntry\s*=\s*0x0F78D10[\s\S]*kEquipManagerDrawCallsite\s*=\s*0x0E107A1[\s\S]*kEquipManagerDrawReturn\s*=\s*0x0E107A7[\s\S]*kEquipManagerSheatheCallsite\s*=\s*0x0E10988[\s\S]*kEquipManagerSheatheReturn\s*=\s*0x0E1098E[\s\S]*kCompleteWeaponDraw\s*=\s*0x0DBE590' `
    'The focused native owner must retain the independently verified FO4VR transition addresses.'

Require-Text $nativePath `
    'kExpectedPlayerDrawEntry[\s\S]*0x48, 0x89, 0x74, 0x24, 0x18[\s\S]*kExpectedEquipManagerVirtualCall[\s\S]*0xFF, 0x90, 0x48, 0x06, 0x00, 0x00[\s\S]*kExpectedCompleteWeaponDrawEntry[\s\S]*0x40, 0x53, 0x48, 0x83, 0xEC, 0x30' `
    'Entry, caller, and completion addresses must be protected by exact runtime byte contracts.'

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
    'completeDrawForExactCurrent[\s\S]{0,1800}equippedIdentityMatches[\s\S]{0,900}CompleteWeaponDraw[\s\S]{0,500}stateAfter\s*!=\s*3' `
    'Direct completion must consume typed evidence, revalidate exact identity, and require exact Drawn state.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'readinessFor\(player\)[\s\S]{0,3000}hand\.captureHeldReleaseMotion[\s\S]*beginHeldTransition[\s\S]{0,4000}completeDrawForExactCurrent[\s\S]{0,900}synchronizeAfterInstantCompletion' `
    'Held equip must preflight before release, begin its bridge, then complete and synchronize the exact native weapon.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'completionResult\.success\(\)[\s\S]{0,400}failHeldCompletion[\s\S]*equipFinalized[\s\S]{0,3000}pendingGripStart\.pending' `
    'Completion failure must clear the bridge and manual hand ownership must wait for final success.'

Require-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'equipImmediatelyWithoutActions[\s\S]{0,1800}readEquippedWeaponSnapshot[\s\S]{0,900}findEquippedWeaponStack[\s\S]{0,700}ActivateRefThenInstantEquip' `
    'Transfer success must require the scoped manager call, exact equipped identity, and exact equipped stack.'

Reject-Text 'src/physics-interaction/weapon/WeaponEquipTransfer.cpp' `
    'equipManager->EquipObject\(' `
    'Held weapon transfer must not bypass the scoped native transaction with a direct or queued manager call.'

Reject-Text $nativePath `
    '0x0DBE6D0|WeaponBeginDraw|WeaponBeginSheathe|PlayerFastEquipSound' `
    'The held draw feature must not add sheathe completion or global animation/sound suppression.'

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

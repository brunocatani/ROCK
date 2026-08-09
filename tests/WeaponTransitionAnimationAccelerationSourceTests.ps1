param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param([string]$Path, [string]$Pattern, [string]$Message)
    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

$source = 'src/physics-interaction/weapon/WeaponTransitionAnimationAcceleration.cpp'
$policy = 'src/physics-interaction/weapon/WeaponTransitionAnimationAccelerationPolicy.h'
$nativeDraw = 'src/physics-interaction/weapon/NativeEquippedWeaponDraw.cpp'
$coordinator = 'src/physics-interaction/weapon/EquippedWeaponTransitionCoordinator.cpp'
$physics = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$main = 'src/ROCKMain.cpp'

Require-Text $source `
    'kClipGeneratorActivate\s*=\s*0x192CA40[\s\S]*kClipGeneratorUpdate\s*=\s*0x192D0D0[\s\S]*kClipGeneratorDeactivate\s*=\s*0x192D510' `
    'The acceleration hooks must retain PAPER''s independently verified FO4VR hkbClipGenerator lifecycle entries.'
Require-Text $source `
    'kExpectedActivateEntry\{[\s\S]*0x40,\s*0x53[\s\S]*0x48,\s*0x81,\s*0xEC,\s*0x50,\s*0x01,\s*0x00,\s*0x00[\s\S]*kExpectedUpdateEntry\{[\s\S]*0x4C,\s*0x8B,\s*0xDC[\s\S]*0x48,\s*0x81,\s*0xEC,\s*0xB0,\s*0x00,\s*0x00,\s*0x00[\s\S]*kExpectedDeactivateEntry\{[\s\S]*0x48,\s*0x89,\s*0x5C,\s*0x24,\s*0x20[\s\S]*0x48,\s*0x8B,\s*0x81,\s*0xD0,\s*0x00,\s*0x00,\s*0x00' `
    'All lifecycle entry trampolines must remain pinned to exact whole-instruction relocatable FO4VR prologues.'
Require-Text $source `
    'REL::Module::IsVR\(\)[\s\S]{0,180}RUNTIME_VR_1_2_72[\s\S]*entry_trampoline_hook::install\([\s\S]*kClipGeneratorActivate[\s\S]*entry_trampoline_hook::install\([\s\S]*kClipGeneratorUpdate[\s\S]*entry_trampoline_hook::install\([\s\S]*kClipGeneratorDeactivate[\s\S]*activateInstalled\s*&&\s*updateInstalled\s*&&\s*deactivateInstalled' `
    'The capability must fail closed unless executable identity and all three clip lifecycle entry hooks validate.'
Require-Text $source `
    'kRefrGraphHolderInterfaceOffset\s*=\s*0x48[\s\S]*kGetGraphManagerVtableSlotOffset\s*=\s*0x20[\s\S]*kGraphManagerVtableModuleOffset\s*=\s*0x2E00550[\s\S]*kManagerGraphsCapacityOffset\s*=\s*0x40[\s\S]*kManagerGraphsStorageOffset\s*=\s*0x48[\s\S]*kGraphVtableModuleOffset\s*=\s*0x2E00A48[\s\S]*kGraphCharacterOffset\s*=\s*0x1C8' `
    'Player graph-character discovery must retain the independently verified PAPER/ROCK pointer-walk contract.'
Require-Text $source `
    'publishPlayerGraphCharacters[\s\S]*kGraphsInlineStorageFlag[\s\S]*kMaxGraphSlots[\s\S]*graphVtable\s*!=\s*REL::Module::get\(\)\.base\(\)\s*\+[\s\S]*kGraphVtableModuleOffset[\s\S]*s_targetCharacterCount\.store' `
    'Graph discovery must stay bounded, handle inline/heap storage, exact-vtable gate every graph, and publish characters atomically.'
Require-Text $source `
    'contextMatchesPlayerGraph[\s\S]*array<std::uintptr_t,\s*4>\s+contextSlots[\s\S]*guardedCopyFromMemory[\s\S]*s_targetCharacters[\s\S]*registerActivatedClip[\s\S]*s_transitionClips' `
    'Only clip instances activated by a registered player graph context may enter the transition registry.'
Require-Text $source `
    'onClipGeneratorActivate[\s\S]*s_originalClipActivate\(clipGenerator, context\)[\s\S]{0,180}registerActivatedClip\(clipGenerator, context\)[\s\S]*onClipGeneratorDeactivate[\s\S]*unregisterClip\(clipGenerator\)[\s\S]{0,180}s_originalClipDeactivate\(clipGenerator, context\)' `
    'Activation must register only after native setup, while deactivation must retire exact clip identity before native teardown.'
Require-Text $source `
    'acceleratedTimestep[\s\S]*s_encodedLease\.load[\s\S]*isRegisteredClip\(clipGenerator\)[\s\S]*getNativeWeaponState\(player\)[\s\S]*shouldAccelerateSample[\s\S]*scaleTransitionTimestep\(timestep\)[\s\S]*onClipGeneratorUpdate[\s\S]*s_originalClipUpdate\([\s\S]*acceleratedTimestep\(clipGenerator, timestep\)' `
    'The update hook must pass a finite scaled timestep into native evaluation only for a registered clip in the exact transition state.'
Require-Text $source `
    'publishPlayerGraphCharacters\(input\.player\)[\s\S]*s_runtimeLease\s*=\s*RuntimeLease[\s\S]*s_encodedLease\.store\(encodedLease' `
    'The game thread must publish verified player graph targets before making the transition lease visible to animation callbacks.'
Require-Text $source `
    'std::atomic<std::uintptr_t>\s+s_encodedLease[\s\S]*transitionState\(direction\)[\s\S]*lease\s*&\s*kEncodedStateMask' `
    'Player and transition direction must cross into clip evaluation as one coherent atomic lease.'
Require-Text $policy `
    'kAcceleratedSpeedMultiplier\s*=\s*100\.0f[\s\S]*scaleTransitionTimestep[\s\S]*nativeState\s*==\s*transitionState\(direction\)[\s\S]*nativeState\s*==\s*terminalState\(direction\)[\s\S]*kLeaseTimeoutSeconds' `
    'Policy must use a finite high multiplier, exact drawing/sheathing gates, terminal completion, and a bounded watchdog.'
Require-Text $nativeDraw `
    'requestAnimationAcceleration\([\s\S]{0,500}Direction::Draw\)[\s\S]{0,180}DrawWeaponMagicHands\(true\)[\s\S]*requestAnimationAcceleration\([\s\S]{0,500}Direction::Sheathe\)[\s\S]{0,180}DrawWeaponMagicHands\(false\)' `
    'ROCK-owned draw and sheathe submissions must arm acceleration before native action submission.'
Require-Text $coordinator `
    'completesSuppressedHeldDraw[\s\S]{0,300}Source::HeldTriggerEquip[\s\S]{0,180}Source::HeldGripZoneEquip[\s\S]{0,3000}native_equipped_weapon_draw::submitExactCurrent' `
    'Both held trigger-equip and RIW grip-zone auto-equip must enter the accelerated exact-current draw boundary immediately.'
Require-Text $physics `
    'updateEquippedWeaponTransition\(\)[\s\S]{0,1000}weapon_transition_animation_acceleration::service[\s\S]{0,900}runtimeAllowed[\s\S]{0,500}localMenuBlocking[\s\S]{0,500}compatibilityConfigBlocking' `
    'The main frame must service exact identity and runtime cancellation before transition coordination.'
Require-Text $main `
    'held_weapon_instant_transition::install\(\)[\s\S]{0,700}weapon_transition_animation_acceleration::install\(\)' `
    'The clip acceleration capability must install after the held-equip action interceptor.'
Reject-Text $source `
    'SetWeaponState|0x0DBE590|0x0DBE6D0|kPrimaryWeaponSpeedEvaluator|kLeftWeaponSpeedEvaluator|kChannelOutputOffset|REL::safe_write|AnimationName|Equip\.hkx|UnEquip\.hkx|FLT_MAX|numeric_limits<float>::max' `
    'Acceleration must not force native state, retain the ineffective actor channels, compete for PAPER''s vtable slots, classify asset names, or use an unbounded float.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Weapon transition animation acceleration source boundaries passed.'

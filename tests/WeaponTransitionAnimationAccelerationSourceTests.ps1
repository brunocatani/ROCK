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
$physics = 'src/physics-interaction/core/PhysicsInteraction.cpp'
$main = 'src/ROCKMain.cpp'

Require-Text $source `
    'kPrimaryWeaponSpeedEvaluator\s*=\s*0x0E40160[\s\S]*kLeftWeaponSpeedEvaluator\s*=\s*0x0E3FE90[\s\S]*kChannelOutputOffset\s*=\s*0x18[\s\S]*kChannelOwnerOffset\s*=\s*0x20' `
    'The acceleration hook must retain the independently verified FO4VR channel entries and layout.'
Require-Text $source `
    'kExpectedEvaluatorEntry\{[\s\S]*0x40,\s*0x53[\s\S]*0x48,\s*0x83,\s*0xEC,\s*0x30[\s\S]*0x4C,\s*0x8B,\s*0x49,\s*0x20[\s\S]*0x48,\s*0x8B,\s*0xD9[\s\S]*0x48,\s*0x83,\s*0xC1,\s*0x20' `
    'Both entry trampolines must remain pinned to the exact 17-byte relocatable FO4VR prologue.'
Require-Text $source `
    'REL::Module::IsVR\(\)[\s\S]{0,180}RUNTIME_VR_1_2_72[\s\S]*entry_trampoline_hook::install\([\s\S]*kPrimaryWeaponSpeedEvaluator[\s\S]*entry_trampoline_hook::install\([\s\S]*kLeftWeaponSpeedEvaluator[\s\S]*primaryInstalled\s*&&\s*leftInstalled' `
    'The capability must fail closed unless executable identity and both hand-aware hooks validate.'
Require-Text $source `
    's_originalPrimaryEvaluator\(channel, graphPass\)[\s\S]{0,180}accelerateEvaluatedChannel\(channel, false\)[\s\S]*s_originalLeftEvaluator\(channel, graphPass\)[\s\S]{0,180}accelerateEvaluatedChannel\(channel, true\)' `
    'Each hook must preserve native sampling before replacing only its completed channel output.'
Require-Text $source `
    'kChannelOwnerOffset[\s\S]{0,900}getNativeWeaponState\(owner\)[\s\S]{0,700}shouldAccelerateSample[\s\S]{0,1600}kChannelOutputOffset' `
    'The animation-thread hook must match the leased player and verified native transitional state before writing the channel output.'
Require-Text $source `
    'std::atomic<std::uintptr_t>\s+s_encodedLease[\s\S]*transitionState\(direction\)[\s\S]*encodedLease\s*&\s*kEncodedStateMask' `
    'Player and transition direction must cross into animation evaluation as one coherent atomic lease.'
Require-Text $policy `
    'kAcceleratedSpeedMultiplier\s*=\s*100\.0f[\s\S]*nativeState\s*==\s*transitionState\(direction\)[\s\S]*nativeState\s*==\s*terminalState\(direction\)[\s\S]*kLeaseTimeoutSeconds' `
    'Policy must use a finite high multiplier, exact drawing/sheathing gates, terminal completion, and a bounded watchdog.'
Require-Text $nativeDraw `
    'requestAnimationAcceleration\([\s\S]{0,500}Direction::Draw\)[\s\S]{0,180}DrawWeaponMagicHands\(true\)[\s\S]*requestAnimationAcceleration\([\s\S]{0,500}Direction::Sheathe\)[\s\S]{0,180}DrawWeaponMagicHands\(false\)' `
    'ROCK-owned draw and sheathe submissions must arm acceleration before native action submission.'
Require-Text $physics `
    'updateEquippedWeaponTransition\(\)[\s\S]{0,1000}weapon_transition_animation_acceleration::service[\s\S]{0,900}runtimeAllowed[\s\S]{0,500}localMenuBlocking[\s\S]{0,500}compatibilityConfigBlocking' `
    'The main frame must service exact identity and runtime cancellation before transition coordination.'
Require-Text $main `
    'held_weapon_instant_transition::install\(\)[\s\S]{0,700}weapon_transition_animation_acceleration::install\(\)' `
    'The speed-channel capability must install after the held-equip action interceptor.'
Reject-Text $source `
    'SetWeaponState|0x0DBE590|0x0DBE6D0|FLT_MAX|numeric_limits<float>::max' `
    'Acceleration must not force native state/completion or use an unbounded floating-point value.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'Weapon transition animation acceleration source boundaries passed.'

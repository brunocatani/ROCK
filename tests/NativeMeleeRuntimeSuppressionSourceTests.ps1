param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()
$Root = (Resolve-Path -LiteralPath $Root).Path

function Get-RepoText {
    param([string]$Path)

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return $null
    }
    return Get-Content -Raw -LiteralPath $fullPath
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-RepoText $Path
    if ($null -eq $text -or $text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $text = Get-RepoText $Path
    if ($null -ne $text -and $text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kData_PlayerActorSingleton\s*=\s*0x5A38518' 'Native melee suppression must keep the FO4VR player actor global offset documented.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.h' 'enforceNativeMeleeRuntimeSuppression' 'Native melee runtime setting suppression must be exported from PhysicsHooks.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'bMeleeVelocityCheck:VRInput' 'Full native melee suppression must keep the FO4VR VRInput velocity gate under ROCK control.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'fMeleeLinearVelocityThreshold:VRInput' 'Full native melee suppression must raise the FO4VR VRInput linear melee threshold.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'fMeleeAngularVelocityThreshold:VRInput' 'Full native melee suppression must raise the FO4VR VRInput angular melee threshold.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'RE::GetINISetting\(settingName\)' 'Runtime suppression must resolve live game INI settings through CommonLib.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'setting->SetBinary\(desiredValue\)' 'Runtime suppression must write the desired native VR melee velocity gate state.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'setting->SetFloat\(desiredMinimum\)' 'Runtime suppression must raise native VR melee velocity thresholds.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'kNativeMeleeSuppressedVelocityThreshold\s*=\s*1\.0e9f' 'Runtime suppression must use unreachable but finite native VR melee thresholds.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'rockNativeMeleeFullSuppression' 'VRInput melee suppression must be tied to ROCK full native melee suppression.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'g_nativeMeleeSuppressionHooksInstalled' 'VRInput melee suppression must be gated behind successful native melee hook installation.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'resolveNativePlayerActorGlobal' 'Animation-event hooks must compare against the native FO4VR player actor global.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'matchesCommonLib\s*\|\|\s*matchesNative' 'Player detection must accept either CommonLib or native player actor singleton.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_AttackBlockHandler_ShouldHandleEvent\s*=\s*0x0FCD770' 'Native melee suppression must document the verified FO4VR AttackBlockHandler input-gate function.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kVtableEntry_AttackBlockHandler_ShouldHandleEvent\s*=\s*0x2D8A350' 'Native melee suppression must document the verified FO4VR AttackBlockHandler input-gate vtable slot.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_PlayerCharacter_WeaponSwingCallBack\s*=\s*0x0F23E00' 'Native melee suppression must document the verified FO4VR PlayerCharacter weapon swing callback function.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kVtableEntry_PlayerCharacter_WeaponSwingCallBack\s*=\s*0x2D817A8' 'Native melee suppression must document the verified FO4VR PlayerCharacter weapon swing callback vtable slot.'
Require-Text 'src/physics-interaction/native/HavokOffsets.h' 'kFunc_VRMeleeImpactCallback\s*=\s*0x0EFF000' 'Native melee suppression must document the verified FO4VR VRMeleeImpact callback function.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'hookedAttackBlockShouldHandleEvent' 'Full native melee suppression must hook the narrow AttackBlockHandler input gate.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'hookedPlayerWeaponSwingCallback' 'Full native melee suppression must hook PlayerCharacter::WeaponSwingCallBack.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'hookedVrMeleeImpactCallback' 'Full native melee suppression must hook the FO4VR native VRMeleeImpact callback.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'installEntryTrampolineHook\("VRMeleeImpact"' 'VRMeleeImpact suppression must use a validated entry trampoline.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'validateEntryTrampolineTarget\(\s*"VRMeleeImpact"' 'VRMeleeImpact entry bytes must be validated before any native melee hook installation.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'rollbackNativeMeleeSuppressionHooks' 'Native melee suppression hook installation must roll back partial installs.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'playerSwingCallback=\{\}' 'Native melee suppression install logging must report the PlayerCharacter weapon swing callback hook.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'vrMeleeImpact=\{\}' 'Native melee suppression install logging must report the VRMeleeImpact callback hook.'
Require-Text 'src/physics-interaction/NativeMeleeSuppressionPolicy.h' 'evaluateNativeMeleeImpactSuppression' 'VRMeleeImpact suppression must use an explicit impact policy.'
Require-Text 'tests/TransformConventionTests.cpp' 'native melee full suppression suppresses player impact callback' 'VRMeleeImpact full-suppression behavior must be covered by compiled policy tests.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'NativeMeleeInputEvent::RightStick' 'AttackBlock input-gate suppression must classify the VR RightStick event.'
Require-Text 'src/physics-interaction/NativeMeleeSuppressionPolicy.h' 'PrimaryAttack' 'AttackBlock input-gate policy must account for PrimaryAttack as a non-RightStick input.'
Require-Text 'src/physics-interaction/NativeMeleeSuppressionPolicy.h' 'SecondaryAttack' 'AttackBlock input-gate policy must account for SecondaryAttack as a non-RightStick input.'
Require-Text 'src/physics-interaction/NativeMeleeSuppressionPolicy.h' 'input\.inputEvent\s*!=\s*NativeMeleeInputEvent::RightStick' 'AttackBlock input-gate policy must suppress only RightStick.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'enforceNativeMeleeRuntimeSuppression\(true\)' 'PhysicsInteraction init must immediately apply native VR melee runtime suppression.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'nativeMeleeSuppressionHooksInstalled' 'PhysicsInteraction init must apply native VR melee runtime settings only after hook installation succeeds.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'enforceNativeMeleeRuntimeSuppression\(\)' 'PhysicsInteraction update must watchdog native VR melee runtime suppression.'

Reject-Text 'src/physics-interaction/core/PhysicsHooks.cpp' 'SetBinary\(false\)' 'Native VR melee suppression must not disable bMeleeVelocityCheck; false bypasses the velocity gate on FO4VR.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'Melee|melee' 'Weapon collider creation must remain unified and must not reintroduce melee-specific collider branches.'

if ($failures.Count -gt 0) {
    Write-Error ($failures -join [Environment]::NewLine)
}

Write-Host 'Native melee runtime suppression source boundaries passed.'

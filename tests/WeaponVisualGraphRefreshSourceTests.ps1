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

function Reject-DirectoryText {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $matches = Get-ChildItem -Recurse -File -LiteralPath $fullPath |
        Select-String -Pattern $Pattern
    if ($matches) {
        $failures.Add($Message)
    }
}

Require-Text 'src/ROCKMain.cpp' 'weapon_workbench_graph_refresh::installHook\(\)' 'ROCK must install the workbench weapon graph refresh hook during plugin load.'
Require-Text 'src/physics-interaction/native/WeaponWorkbenchGraphRefreshHook.cpp' 'kBuildConfirmedNativeRefreshCallSite\s*=\s*0xB3AA24' 'Workbench refresh hook must target the verified ExamineMenu::BuildConfirmed native refresh callsite.'
Require-Text 'src/physics-interaction/native/WeaponWorkbenchGraphRefreshHook.cpp' '0xFF,\s*0x90,\s*0x20,\s*0x01,\s*0x00,\s*0x00' 'Workbench refresh hook must validate the verified vtable-call bytes before patching.'
Require-Text 'src/physics-interaction/native/WeaponWorkbenchGraphRefreshHook.cpp' 'kExamineMenuAttachModToPreviewSlotOffset\s*=\s*0x120' 'Workbench refresh hook must replay the verified ExamineMenu visual-refresh vtable slot.'
Require-Text 'src/physics-interaction/native/WeaponWorkbenchGraphRefreshHook.cpp' 'write_call<6>' 'Workbench refresh hook must replace the six-byte native indirect call with an exact-size call hook.'
Require-Text 'src/physics-interaction/native/WeaponWorkbenchGraphRefreshHook.cpp' 'nativeRefresh\(examineMenu,\s*applySelection\);[\s\S]*nativeRefresh\(examineMenu,\s*applySelection\);' 'Workbench refresh hook must replay the native visual refresh after the natural call when the equipped signature changes.'
Require-Text 'src/physics-interaction/native/WeaponEquippedModSignature.cpp' 'ROCKEquippedWeaponModGraphRefreshSignatureV2' 'Weapon graph refresh must be driven by equipped mod-instance signature changes.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'currentRefreshEpoch\(\)' 'Weapon graph refresh coordinator must consume workbench native-refresh replay epochs.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'if \(!input\.menuBlocking\)[\s\S]{0,120}publishObservedEquippedWeaponSignature\(signature\.key\)' 'Weapon graph refresh coordinator must publish stable non-menu equipped signatures for the native hook.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' 'WeaponVisualGraphRefreshCoordinator _weaponVisualGraphRefresh' 'PhysicsInteraction must own the weapon graph refresh coordinator outside the weapon collision module.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'invalidateForVisualGraphRefresh\(hknp,\s*"workbench-native-refresh-replay"\)' 'PhysicsInteraction must invalidate generated weapon collision after a native workbench refresh replay.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'if \(weaponGraphRefresh\.deferWeaponCollision\)[\s\S]{0,80}weaponNode = nullptr' 'Weapon collision must not rebuild from the graph on the refresh frame.'

Reject-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' '\b(?:HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|AttachWeapon|AttachModToReference)\b' 'Weapon graph refresh coordinator must not use equip refresh, broad queued 3D refresh, or direct mod-attach paths.'
Reject-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'equipped-weapon-observed|equipped-mod-instance-changed' 'Weapon graph refresh must not trigger from first observation or ordinary equip/unholster signature changes.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '\bSet3DUpdateFlag\b' 'Weapon collision must remain geometry-only and must not call native visual graph refresh.'
Reject-DirectoryText 'src/physics-interaction/native' '\bSet3DUpdateFlag\b' 'Weapon native graph refresh must not use the failed loaded-reference dirty-flag path.'
Reject-DirectoryText 'src/physics-interaction/native' 'workbench-menu-closed-mod-signature-changed|equipped-weapon-3d-update-flag' 'Weapon native graph refresh must not keep the failed menu-close dirty-flag trigger.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon visual graph refresh source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon visual graph refresh source boundary passed.'

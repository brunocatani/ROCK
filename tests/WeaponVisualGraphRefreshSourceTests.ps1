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

Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'Set3DUpdateFlag\(static_cast<RE::RESET_3D_FLAGS>\(kWeaponReferenceRefreshFlags\)\)' 'Weapon graph refresh must use the loaded-reference 3D dirty path.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'static_assert\(kWeaponReferenceRefreshFlags == 0x37\)' 'Weapon graph refresh must document the AttachModToReference actor dirty flag mask.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'ROCKEquippedWeaponModGraphRefreshSignatureV1' 'Weapon graph refresh must be driven by equipped mod-instance signature changes.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'weaponGraphRefreshMenuOpen' 'Weapon graph refresh must be gated by the workbench/menu window.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'ExamineMenu' 'Weapon graph refresh must watch the weapon examine menu window.'
Require-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'workbench-menu-closed-mod-signature-changed' 'Weapon graph refresh must apply only after a workbench/menu signature change closes.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.h' 'WeaponVisualGraphRefreshCoordinator _weaponVisualGraphRefresh' 'PhysicsInteraction must own the weapon graph refresh coordinator outside the weapon collision module.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'invalidateForVisualGraphRefresh\(hknp,\s*"equipped-weapon-3d-update-flag"\)' 'PhysicsInteraction must invalidate generated weapon collision after applying a native graph refresh.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'if \(weaponGraphRefresh\.deferWeaponCollision\)[\s\S]{0,80}weaponNode = nullptr' 'Weapon collision must not rebuild from the graph on the refresh frame.'

Reject-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' '\b(?:HandleItemEquip|SetEquippedItem|RequestLoadAnimationsForWeaponChange|QueueUpdate3D|QueueShow1stPerson|AttachWeapon|AttachModToReference)\b' 'Weapon graph refresh coordinator must not use equip refresh, broad queued 3D refresh, or direct mod-attach paths.'
Reject-Text 'src/physics-interaction/native/WeaponVisualGraphRefreshCoordinator.cpp' 'equipped-weapon-observed|equipped-mod-instance-changed' 'Weapon graph refresh must not trigger from first observation or ordinary equip/unholster signature changes.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' '\bSet3DUpdateFlag\b' 'Weapon collision must remain geometry-only and must not call native visual graph refresh.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon visual graph refresh source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon visual graph refresh source boundary passed.'

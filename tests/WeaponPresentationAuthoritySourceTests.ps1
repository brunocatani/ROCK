param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Text {
    param(
        [string]$Text,
        [string]$Pattern,
        [string]$Message
    )

    if ($Text -match $Pattern) {
        $failures.Add($Message)
    }
}

function Select-Boundary {
    param(
        [string]$Text,
        [string]$Start,
        [string]$End,
        [string]$Description
    )

    $startIndex = $Text.IndexOf($Start)
    $endIndex = if ($startIndex -ge 0) {
        $Text.IndexOf($End, $startIndex + $Start.Length)
    } else {
        -1
    }
    if ($startIndex -lt 0 -or $endIndex -le $startIndex) {
        $failures.Add("Could not isolate $Description source boundary.")
        return ''
    }
    return $Text.Substring($startIndex, $endIndex - $startIndex)
}

$authorityHeader = Get-Content -Raw -LiteralPath (
    Join-Path $Root 'src/physics-interaction/weapon/WeaponAuthority.h')
$gripSource = Get-Content -Raw -LiteralPath (
    Join-Path $Root 'src/physics-interaction/weapon/TwoHandedGrip.cpp')

Require-Text $authorityHeader `
    'makePresentationWorldDelta[\s\S]*newWeaponWorld[\s\S]*invertTransform\(oldWeaponWorld\)[\s\S]*applyPresentationWorldDelta[\s\S]*presentationWorldDelta[\s\S]*presentationWorld' `
    'Weapon authority math must preserve already-evaluated presentation worlds through one precomputed rigid root delta.'
Require-Text $authorityHeader `
    'preserveLiveWeaponWorldScale[\s\S]*result\s*=\s*requestedWeaponWorld[\s\S]*result\.scale\s*=\s*liveWeaponWorld\.scale' `
    'Rigid weapon authority must preserve hFRIK''s live inherited world scale.'

$descendantMove = Select-Boundary `
    $gripSource `
    'void applyWeaponPresentationDeltaToDescendants(' `
    'bool tryResolveWeaponRootLocal(' `
    'weapon presentation descendant movement'
Require-Text $descendantMove `
    'child->world\s*=[\s\S]*applyPresentationWorldDelta[\s\S]*applyWeaponPresentationDeltaToDescendants' `
    'Rigid presentation movement must update every descendant world recursively.'
Reject-Text $descendantMove `
    '(?:child|childNode|parent)->local\s*=' `
    'Rigid presentation movement must never overwrite native/controller-owned descendant locals.'
Reject-Text $descendantMove `
    'updateTransformsDown\s*\(' `
    'Rigid presentation movement must not rebuild descendants from their locals.'

$rigidMove = Select-Boundary `
    $gripSource `
    'bool moveWeaponPresentationRigidly(' `
    'bool areTransformsNearlyEqual(' `
    'weapon presentation transaction'
Require-Text $rigidMove `
    'validateWeaponPresentationSubtree\([\s\S]*weaponNode->local\s*=\s*weaponLocal[\s\S]*weaponNode->world\s*=\s*solvedWeaponWorld[\s\S]*applyWeaponPresentationDeltaToDescendants' `
    'Weapon presentation movement must validate the bounded subtree before atomically writing the root and descendants.'
Reject-Text $rigidMove `
    'updateTransformsDown\s*\(' `
    'Weapon presentation movement must not invoke local-based recursive propagation.'

$publisher = Select-Boundary `
    $gripSource `
    'bool TwoHandedGrip::applyWeaponVisualAuthority(' `
    'bool TwoHandedGrip::applyFiringHandLockedVisual(' `
    'weapon visual authority publisher'
Require-Text $publisher `
    'preserveLiveWeaponWorldScale\([\s\S]*weaponNode->world[\s\S]*solvedWeaponWorld[\s\S]*_weaponVisualIntentObserver\([\s\S]*scaleStableSolvedWeaponWorld[\s\S]*moveWeaponPresentationRigidly\([\s\S]*scaleStableSolvedWeaponWorld' `
    'The central weapon publisher must normalize inherited scale before both collision intent and the rigid presentation transaction.'
Reject-Text $publisher `
    'updateTransformsDown\s*\(' `
    'The central weapon publisher must not retain a competing local-propagation path.'

$weaponReturn = Select-Boundary `
    $gripSource `
    'void TwoHandedGrip::beginWeaponVisualReturn(' `
    'void TwoHandedGrip::updateWeaponVisualReturn(' `
    'weapon visual return initialization'
Require-Text $weaponReturn `
    'moveWeaponPresentationRigidly\(_activeWeaponNode,\s*startWorld\)' `
    'Weapon return initialization must preserve the evaluated descendant presentation.'
Reject-Text $weaponReturn `
    'updateTransformsDown\s*\(' `
    'Weapon return initialization must not rebuild animated descendants from locals.'

if ($failures.Count -gt 0) {
    Write-Host 'WeaponPresentationAuthoritySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'WeaponPresentationAuthoritySourceTests passed.' -ForegroundColor Green

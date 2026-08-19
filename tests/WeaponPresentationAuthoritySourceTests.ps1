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

$authorityHeader = Get-Content -Raw -LiteralPath (
    Join-Path $Root 'src/physics-interaction/weapon/WeaponAuthority.h')

Require-Text $authorityHeader `
    'makePresentationWorldDelta[\s\S]*newWeaponWorld[\s\S]*invertTransform\(oldWeaponWorld\)[\s\S]*applyPresentationWorldDelta[\s\S]*presentationWorldDelta[\s\S]*presentationWorld' `
    'Weapon authority math must preserve already-evaluated presentation worlds through one precomputed rigid root delta.'
Require-Text $authorityHeader `
    'preserveLiveWeaponWorldScale[\s\S]*result\s*=\s*requestedWeaponWorld[\s\S]*result\.scale\s*=\s*liveWeaponWorld\.scale' `
    'Rigid weapon authority must preserve hFRIK''s live inherited world scale.'

if ($failures.Count -gt 0) {
    Write-Host 'WeaponPresentationAuthoritySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'WeaponPresentationAuthoritySourceTests passed.' -ForegroundColor Green

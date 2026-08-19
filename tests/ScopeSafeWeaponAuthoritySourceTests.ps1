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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
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

    $text = Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

Require-Text 'src/physics-interaction/core/RockRuntimeState.cpp' 'localScopeMenuOpen\s*=\s*s_menuHandlerInitialized\s*&&\s*s_gameMenus\.isInScopeMenu\(\)' `
    'Runtime state must sample FO4VR ScopeMenu explicitly instead of treating it as a generic blocking menu.'


Reject-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'followWeaponWorldChange\s*\(' `
    'Native-scope camera authority must not retain the controller-relative rigid-delta fallback.'
Require-Text 'src/physics-interaction/weapon/WeaponAuthority.h' 'enum class HandAuthorityRole[\s\S]*DesiredHandAuthorityInput[\s\S]*desiredRolesForHand[\s\S]*DeferredClearAction[\s\S]*resolveDeferredClearAction' `
    'Scope-exit cleanup must derive role-aware live authority and wait for replacement publication before clearing stale tags.'
if ($failures.Count -gt 0) {
    foreach ($failure in $failures) {
        Write-Error $failure
    }
    exit 1
}

Write-Host 'Scope-safe weapon authority source boundaries passed.'

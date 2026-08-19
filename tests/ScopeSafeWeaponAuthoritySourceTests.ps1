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
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'scopeHandDriverNode\s*=\s*\[playerNodes\]\(bool isLeft\)[\s\S]*return isLeft\s*\?[\s\S]*SecondaryMeleeWeaponOffsetNode2\s*:[\s\S]*primaryWeaponOffsetNOde[\s\S]*scopeMenuOpen\s*=\s*runtime\.localScopeMenuOpen[\s\S]*leftHandDriverFrame\s*=\s*leftHandDriverFrame[\s\S]*rightHandDriverFrame\s*=\s*rightHandDriverFrame' `
    'Two-hand authority must receive fixed physical-left secondary and physical-right primary hFRIK driver frames together with explicit ScopeMenu state.'
Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'nativeScopeRequestStateValid\s*=\s*tryReadNativeScopeRequestState\(nativeScopeRequestActive\)[\s\S]*manualScopeActivationRequested\s*=\s*input_remap_runtime::isManualScopeActivationRequested\(\)[\s\S]*manualScopeActivationRequested\s*=\s*manualScopeActivationRequested[\s\S]*nativeScopeRequestStateValid\s*=\s*nativeScopeRequestStateValid[\s\S]*nativeScopeRequestActive\s*=\s*nativeScopeRequestActive' `
    'The physical button request and verified native renderer state must be sampled beside the UI signal for scope-transition diagnosis.'
Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'isLeftHandedMode' `
    'Scope authority must not reinterpret ROCK controller identity through Fallout 4 VR native handedness.'

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' 'tryGetManualScopeDirectTransitionTarget[\s\S]*matchesCurrentEquippedWeapon\(publishedIdentity,\s*currentIdentity\)' `
    'Manual direct scope transitions must reject a previous weapon body-set publication during equip replacement.'
Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.h' 'struct NativeScopeResolvedAnchorSnapshot[\s\S]*weaponGenerationKey[\s\S]*equippedWeaponOwnershipKey[\s\S]*weaponFormID[\s\S]*anchorWeaponLocal' `
    'The selected native-scope anchor must retain generation, ownership, and form identity across internal and provider readbacks.'
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

param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    return Get-Content -Raw -LiteralPath (Join-Path $Root $RelativePath)
}

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -notmatch $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $text = Read-Source $RelativePath
    if ($text -match $Pattern) {
        $failures.Add("$RelativePath`: $Message")
    }
}

Require-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'supportAuthorityMode\s*=\s*weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;[\s\S]{0,1800}canApplyFiringGripProximityAuthority\(\s*_equippedWeaponHandlingSettings\.firingGripProximitySupportEnabled,\s*supportAuthorityProviderOverride\)' `
    'Equipped weapons must enter the addon-leased proximity contract from full authority after provider overrides are resolved.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'if \(firingGripProximityAuthorityEnabled\)[\s\S]{0,900}resolveFiringGripProximityAuthorityMode\([\s\S]{0,260}_handlingSettings\.firingGripProximitySupportRadiusGameUnits' `
    'Grip capture must resolve support authority from the addon-supplied firing-grip proximity radius.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'return proximityAuthorityEnabled && !providerGrabModeOverride;' `
    'Explicit provider grab modes must bypass the generic proximity authority contract.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryPromoteSupportGripToFiringGrip[\s\S]*canPromoteSupportGripToFiringGrip\([\s\S]*_authorityMode,[\s\S]*supportGrip\.authoredSupportGrip' `
    'An authored visual-only grip must remain presentation-only instead of inheriting firing authority through handoff promotion.'

Require-Text 'src/api/ROCKProviderApi.h' `
    'FiringGripProximitySupport[\s\S]*firingGripProximitySupportRadiusGameUnits' `
    'ROCK V1 must expose the addon-owned proximity feature and bounded radius.'
Reject-Text 'data/config/ROCK.ini' `
    'bFiringGripProximitySupportEnabled|fFiringGripProximitySupportRadius' `
    'Base ROCK config must not retain addon-owned proximity tuning.'

Reject-Text 'src/physics-interaction/core/PhysicsInteraction.cpp' `
    'AnimsGripPistol|classifyEquippedWeaponForSupportGrip|resolveEquippedWeaponSupportAuthorityMode' `
    'Runtime support authority must not retain the superseded pistol classifier.'

Reject-Text 'data/config/ROCK.ini' `
    'VisualOnlySidearmSupportGrip|SidearmVisualOnlySupportGrip' `
    'The source config must not retain obsolete sidearm-only contract keys.'

if ($failures.Count -gt 0) {
    Write-Host 'Weapon firing-grip proximity source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Weapon firing-grip proximity source boundary passed.'

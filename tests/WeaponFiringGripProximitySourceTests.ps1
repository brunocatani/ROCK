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
    'supportAuthorityMode\s*=\s*weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;[\s\S]{0,1800}canApplyFiringGripProximityAuthority\(\s*g_rockConfig\.rockFiringGripProximitySupportEnabled,\s*supportAuthorityProviderOverride\)' `
    'Equipped weapons must enter the generic proximity contract from full authority after provider overrides are resolved.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'if \(firingGripProximityAuthorityEnabled\)[\s\S]{0,900}resolveFiringGripProximityAuthorityMode\([\s\S]{0,220}rockFiringGripProximitySupportRadius' `
    'Grip capture must resolve support authority from firing-grip proximity for every eligible equipped weapon.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'return proximityAuthorityEnabled && !providerGrabModeOverride;' `
    'Explicit provider grab modes must bypass the generic proximity authority contract.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryPromoteSupportGripToFiringGrip[\s\S]*canPromoteSupportGripToFiringGrip\([\s\S]*_authorityMode,[\s\S]*supportGrip\.authoredSupportGrip' `
    'An authored visual-only grip must remain presentation-only instead of inheriting firing authority through handoff promotion.'

Require-Text 'data/config/ROCK.ini' `
    'bFiringGripProximitySupportEnabled\s*=\s*true[\s\S]*fFiringGripProximitySupportRadius\s*=\s*6\.0' `
    'The source config must publish the weapon-generic firing-grip proximity contract.'

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

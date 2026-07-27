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
    'supportAuthorityMode\s*=\s*weapon_support_authority_policy::WeaponSupportAuthorityMode::FullTwoHandedSolver;[\s\S]{0,1800}canApplyFiringGripProximityAuthority\(\s*supportAuthorityProviderOverride\)' `
    'Equipped weapons must always enter ROCK proximity support after explicit provider grab modes are resolved.'

Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'Near-firing-grip VisualOnlySupport is a ROCK weapon-support safety[\s\S]{0,320}settings\.firingGripProximitySupportRadiusGameUnits\s*=\s*\r?\n\s*rockBaseline\.firingGripProximitySupportRadiusGameUnits' `
    'ROCK must seed the support contract from its configured baseline radius.'
Require-Text 'src/physics-interaction/weapon/EquippedWeaponHandlingSettings.h' `
    'if \(enabled\(provider::RockProviderEquippedWeaponHandlingFlagV1::FiringGripProximitySupport\)\)[\s\S]{0,240}request->firingGripProximitySupportRadiusGameUnits' `
    'An active V1 authority owner may explicitly override only the proximity radius.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'if \(firingGripProximityAuthorityEnabled\)[\s\S]{0,900}resolveFiringGripProximityAuthorityMode\([\s\S]{0,260}_handlingSettings\.firingGripProximitySupportRadiusGameUnits' `
    'Grip capture must resolve support authority from the active core-or-addon firing-grip proximity radius.'

Require-Text 'src/physics-interaction/weapon/WeaponSupport.h' `
    'canApplyFiringGripProximityAuthority\(\s*bool providerGrabModeOverride\)[\s\S]{0,120}return !providerGrabModeOverride;' `
    'Core proximity support must be unconditional except for explicit provider grab modes.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'tryPromoteSupportGripToFiringGrip[\s\S]*canPromoteSupportGripToFiringGrip\([\s\S]*_authorityMode,[\s\S]*supportGrip\.authoredSupportGrip' `
    'An authored visual-only grip must remain presentation-only instead of inheriting firing authority through handoff promotion.'

Require-Text 'src/physics-interaction/weapon/TwoHandedGrip.cpp' `
    'handlingSettings\.ambidextrousHandoffEnabled\s*&&\s*!primaryGripInput\.held\s*&&\s*tryPromoteSupportGripToFiringGrip' `
    'Core VisualOnlySupport must never switch firing hands unless the effective ROCK-or-addon ambidextrous policy is active.'

Require-Text 'src/api/ROCKProviderApi.h' `
    'core VisualOnlySupport behavior itself remains always enabled[\s\S]*FiringGripProximitySupport[\s\S]*firingGripProximitySupportRadiusGameUnits' `
    'ROCK V1 must document that its existing proximity flag overrides radius without disabling core support.'
Require-Text 'src/RockConfig.cpp' `
    'fFiringGripProximitySupportRadius[\s\S]{0,180}0\.25f,[\s\S]{0,40}30\.0f' `
    'ROCK must load and bound its core firing-grip proximity radius.'
foreach ($configPath in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Text $configPath `
        'fFiringGripProximitySupportRadius\s*=\s*6\.0' `
        'ROCK config must publish the core firing-grip proximity radius.'
    Reject-Text $configPath `
        'bFiringGripProximitySupportEnabled' `
        'ROCK proximity support must not expose an off switch.'
}

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

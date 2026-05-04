$ErrorActionPreference = 'Stop'

<#
ROCK's reload detachment is a source-boundary invariant, not just a runtime
toggle. PAPER now owns native reload events, ammo gates, profiles, bodies,
visuals, authoring, and overlay diagnostics; ROCK must keep only generic hand,
weapon, contact, support-grip, and provider evidence. This test scans the active
ROCK runtime/config files for reload-owned symbols so future cleanup cannot
silently regress into a half-owned implementation.
#>

$repoRoot = Resolve-Path (Join-Path $PSScriptRoot '..')

$checks = @(
    @{
        Path = 'src/api/ROCKApi.h'
        Forbidden = @(
            'WeaponReload',
            'ReloadProfile',
            'getActiveWeaponReloadState',
            'getObservedWeaponReloadStage',
            'getWeaponReloadStageSource'
        )
    },
    @{
        Path = 'src/physics-interaction/core/PhysicsInteraction.h'
        Forbidden = @(
            'WeaponReloadEventBridge',
            'WeaponReloadAmmoGate',
            'WeaponReloadActionDriver',
            'WeaponReloadAuthoring',
            'WeaponReloadBodySet',
            'WeaponReloadDetachableMagazineModule',
            'WeaponReloadProfileBinding',
            'WeaponReloadStageObserver',
            'WeaponReloadVisualGraph',
            'WeaponSemanticProfileStore',
            '_weaponReload',
            '_physicalReload',
            '_reloadAuthoring',
            'updateWeaponReloadState',
            'handlePhysicalReloadInteraction'
        )
    },
    @{
        Path = 'src/physics-interaction/core/PhysicsInteraction.cpp'
        Forbidden = @(
            'WeaponReloadClipWriteGateRuntime',
            'WeaponReloadAuthoring',
            'WeaponReloadNativeAccess',
            'WeaponSemanticProfileStore',
            '_weaponReload',
            '_physicalReload',
            '_reloadAuthoring',
            'resolvePhysicalReloadProfile',
            'updateWeaponReloadState',
            'handlePhysicalReloadInteraction',
            'resetWeaponReloadStateForInterruption',
            'resetReloadProfileAuthoring'
        )
    },
    @{
        Path = 'src/physics-interaction/core/PhysicsHooks.h'
        Forbidden = @(
            'PhysicalReload',
            'AmmoClipHook'
        )
    },
    @{
        Path = 'src/physics-interaction/core/PhysicsHooks.cpp'
        Forbidden = @(
            'WeaponReloadClipWriteGateRuntime',
            'ActorSetCurrentAmmoCount',
            'installPhysicalReloadAmmoClipHook',
            'isPhysicalReloadAmmoClipHookInstalled',
            'PhysicalReload clip'
        )
    },
    @{
        Path = 'src/RockConfig.h'
        Forbidden = @(
            'WeaponProfile',
            'PhysicalReload'
        )
    },
    @{
        Path = 'src/RockConfig.cpp'
        Forbidden = @(
            'WeaponProfile',
            'PhysicalReload'
        )
    },
    @{
        Path = 'data/config/ROCK.ini'
        Forbidden = @(
            'WeaponProfiles',
            'PhysicalReload'
        )
    },
    @{
        Path = 'CMakeLists.txt'
        Forbidden = @(
            'WeaponReload',
            'PhysicalReload'
        )
    },
    @{
        Path = 'cmake/package.cmake'
        Forbidden = @(
            'ReloadConfig',
            'PhysicalReload'
        )
    }
)

$failures = New-Object System.Collections.Generic.List[string]
foreach ($check in $checks) {
    $path = Join-Path $repoRoot $check.Path
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("Missing checked file: $($check.Path)")
        continue
    }

    $content = Get-Content -LiteralPath $path -Raw
    foreach ($pattern in $check.Forbidden) {
        if ($content.Contains($pattern)) {
            $failures.Add("$($check.Path) still contains forbidden reload ownership token '$pattern'")
        }
    }
}

$dataModRoot = Join-Path $repoRoot 'data/mod'
if (Test-Path -LiteralPath $dataModRoot) {
    Get-ChildItem -LiteralPath $dataModRoot -Recurse -File | ForEach-Object {
        $relative = $_.FullName.Substring($dataModRoot.Length).TrimStart('\', '/')
        if ($_.Extension -in @('.esp', '.esm', '.esl', '.pex', '.psc', '.nif')) {
            $failures.Add("ROCK data/mod still ships game-data asset '$relative'. Reload game data belongs to PAPER.")
        }

        if ($_.Name -like '*ReloadConfig*' -or $_.Name -like '*PhysicalReload*') {
            $failures.Add("ROCK data/mod still ships reload asset: $($_.FullName)")
        }
    }
}

$pluginRoot = Join-Path $repoRoot 'plugin'
if (Test-Path -LiteralPath $pluginRoot) {
    Get-ChildItem -LiteralPath $pluginRoot -Recurse -File | ForEach-Object {
        $relative = $_.FullName.Substring($pluginRoot.Length).TrimStart('\', '/')
        if ($_.Name -like '*ReloadConfig*' -or $_.Name -like '*ROCK_Reload*' -or $_.Extension -eq '.esp') {
            $failures.Add("ROCK plugin source still contains reload-owned plugin artifact '$relative'")
        }
    }
}

$cmakePath = Join-Path $repoRoot 'CMakeLists.txt'
$cmake = Get-Content -LiteralPath $cmakePath -Raw
foreach ($deployCleanup in @('ROCK.esp', 'Scripts/ROCK', 'Meshes/ROCK')) {
    if (-not $cmake.Contains($deployCleanup)) {
        $failures.Add("ROCK deploy step does not clean stale reload payload path '$deployCleanup'")
    }
}

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host 'ROCK reload detachment source boundary passed.'

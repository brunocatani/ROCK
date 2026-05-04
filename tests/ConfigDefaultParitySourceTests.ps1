param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Text {
    param([string]$Path)
    return Get-Content -Raw -LiteralPath (Join-Path $Root $Path)
}

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Require-Ini-Key {
    param(
        [string]$Key,
        [string]$Value
    )

    $escapedKey = [regex]::Escape($Key)
    $escapedValue = [regex]::Escape($Value)
    Require-Text 'data/config/ROCK.ini' "(?m)^\s*$escapedKey\s*=\s*$escapedValue\s*$" "Packaged ROCK.ini must keep $Key = $Value in parity with compiled defaults."
}

Require-Text 'src/RockConfig.h' 'bool rockWeaponCollisionEnabled = true;' 'RockConfig member initializer for weapon collision must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockWeaponCollisionEnabled = true;' 'RockConfig::resetToDefaults must enable weapon collision when packaged ROCK.ini enables it.'
Require-Ini-Key 'bWeaponCollisionEnabled' 'true'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowColliders = true;' 'RockConfig member initializer for collider overlay must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowColliders = true;' 'RockConfig::resetToDefaults must match packaged collider overlay default.'
Require-Ini-Key 'bDebugShowColliders' 'true'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowTargetColliders = true;' 'RockConfig member initializer for target collider overlay must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowTargetColliders = true;' 'RockConfig::resetToDefaults must match packaged target collider overlay default.'
Require-Ini-Key 'bDebugShowTargetColliders' 'true'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowHandAxes = true;' 'RockConfig member initializer for hand axes overlay must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowHandAxes = true;' 'RockConfig::resetToDefaults must match packaged hand axes overlay default.'
Require-Ini-Key 'bDebugShowHandAxes' 'true'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxWeaponBodiesDrawn = 100;' 'RockConfig member initializer for weapon body draw limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxWeaponBodiesDrawn = 100;' 'RockConfig::resetToDefaults must match packaged weapon body draw limit.'
Require-Ini-Key 'iDebugMaxWeaponBodiesDrawn' '100'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxShapeGenerationsPerFrame = 100;' 'RockConfig member initializer for shape generation limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxShapeGenerationsPerFrame = 100;' 'RockConfig::resetToDefaults must match packaged shape generation limit.'
Require-Ini-Key 'iDebugMaxShapeGenerationsPerFrame' '100'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxConvexSupportVertices = 6;' 'RockConfig member initializer for convex support vertex limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxConvexSupportVertices = 6;' 'RockConfig::resetToDefaults must match packaged convex support vertex limit.'
Require-Ini-Key 'iDebugMaxConvexSupportVertices' '6'

Require-Text 'src/RockConfig.h' 'bool rockHandBoneCollidersRequireAllFingerBones = true;' 'RockConfig member initializer for all-finger hand collider requirement must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockHandBoneCollidersRequireAllFingerBones = true;' 'RockConfig::resetToDefaults must match packaged all-finger hand collider default.'
Require-Ini-Key 'bHandBoneCollidersRequireAllFingerBones' 'true'

Require-Text 'src/RockConfig.cpp' 'rockNearCastDistanceGameUnits = static_cast<float>\(ini\.GetDoubleValue\(SECTION, "fNearCastDistanceGameUnits", rockNearCastDistanceGameUnits\)\);' 'fNearCastDistanceGameUnits must use its own current default, not fNearDetectionRange.'

if ($failures.Count -gt 0) {
    Write-Host 'Config default parity source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Config default parity source boundary passed.'

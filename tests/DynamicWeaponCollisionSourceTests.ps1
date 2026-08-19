param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$Path)
    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add("Missing source file: $Path")
        return ''
    }
    return Get-Content -Raw -LiteralPath $fullPath
}

function Require-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Reject-Pattern {
    param([string]$Path, [string]$Pattern, [string]$Message)
    if ((Read-Source $Path) -match $Pattern) {
        $failures.Add($Message)
    }
}

function Require-Order {
    param([string]$Path, [string[]]$Patterns, [string]$Message)
    $text = Read-Source $Path
    $offset = 0
    foreach ($pattern in $Patterns) {
        $match = [regex]::Match(
            $text.Substring($offset),
            $pattern,
            [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

$layers = 'src/physics-interaction/collision/CollisionLayerPolicy.h'

# The validated dynamic compound and hand-interaction graph ship enabled.
# Layer-44 weapon hulls continue to own ordinary gameplay contact evidence.
Require-Pattern 'src/RockConfig.h' `
    'rockWeaponCollisionDynamicBoxEnabled\s*=\s*true' `
    'Dynamic weapon collision must default enabled in compiled configuration.'
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Pattern $ini `
        '(?m)^bWeaponCollisionDynamicBoxEnabled\s*=\s*true\s*$' `
        "$ini must ship dynamic weapon collision enabled."
    Require-Pattern $ini `
        '(?m)^fWeaponCollisionDynamicInverseInertiaMultiplier\s*=\s*1\.2\s*$' `
        "$ini must ship the qualified dynamic-weapon rotational compliance multiplier."
}
Require-Pattern 'src/RockConfig.h' `
    'rockWeaponCollisionDynamicInverseInertiaMultiplier\s*=\s*1\.2f' `
    'The compiled dynamic-weapon rotational compliance default must match the qualified runtime value.'
Require-Pattern 'src/RockConfig.cpp' `
    'fWeaponCollisionDynamicInverseInertiaMultiplier[\s\S]*kDefaultWeaponCollisionDynamicInverseInertiaMultiplier[\s\S]*0\.25f[\s\S]*4\.0f' `
    'The dynamic-weapon rotational compliance setting must load through a finite positive range.'
Require-Pattern $layers `
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY\s*=\s*51' `
    'The dynamic weapon proxy must retain its dedicated layer-51 row.'
Require-Pattern $layers `
    'isDynamicWeaponProxyObstacleLayer\(std::uint32_t layer\)[\s\S]*isWorldSurfaceLayer\(layer\)[\s\S]*isDynamicWorldCarLayer\(layer\)[\s\S]*buildRockDynamicWeaponProxyExpectedMask\([\s\S]*rightHandInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_RIGHT_HAND_PROXY[\s\S]*leftHandInteractionEnabled[\s\S]*ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY' `
    'The dynamic weapon proxy row must retain world/car obstacles and gate each dynamic hand independently.'
Require-Pattern $layers `
    'buildRockDynamicWorldCarExpectedMask[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY\)[\s\S]*withLayer\(mask, FO4_LAYER_CHARCONTROLLER\)' `
    'Tagged car rows must symmetrically admit both hands and the dynamic weapon without exposing generated gameplay colliders.'
Require-Pattern $layers `
    'applyRockGeneratedLayerPolicies[\s\S]*applyRockDynamicWeaponProxyLayerPolicy\([\s\S]*dynamicWeaponRightHandInteractionEnabled[\s\S]*dynamicWeaponLeftHandInteractionEnabled' `
    'Layer 51 must be registered with side-specific free-hand collision eligibility.'

# World contact remains exactly one dynamic body whose child instances follow
# the shared layer-44 hull sources. A second tiny body is permitted only as the
# noncolliding keyframed constraint authority.



# Contact callbacks identify positive-point solved manifolds; the current
# post-solve body snapshot owns the actual collision-resolved pose.
# The proxy participates in the same callback-clock drive and deterministic
# live/stale-world cleanup contract as the existing generated bodies.

foreach ($path in @('src/RockConfig.h', 'src/RockConfig.cpp', 'data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Reject-Pattern $path `
        'WeaponCollisionDynamicContactPressMaxVelocityHavok' `
        "$path must not retain the unused direct-velocity weapon contact cap after the constraint architecture replacement."
}


# FO4VR 0x1417A3A90 is initializeAsKeyFramed. It zeros motion-cinfo inverse
# mass, so it must never contaminate the shared velocity-driven body wrapper.
Reject-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'deriveMotionCinfo\s*\(' `
    'Generated dynamic bodies must not pass through the keyframed initializer mislabeled as mass derivation.'
Require-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    '0x1417A3A90 is initializeAsKeyFramed[\s\S]{0,500}motionCinfoCtor\(motionCinfo\);' `
    'The generated-body wrapper must document and preserve the dynamic-safe constructor profile.'
Require-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'kGeneratedBodyRuntimeFlags\s*=\s*0x0802''0000[\s\S]{0,250}kRebuildBodyCollisionState\s*=\s*0[\s\S]*setFilterInfo\(world,\s*bodyId,\s*filterInfo,\s*1\);[\s\S]{0,1000}enableFlags\(world,\s*bodyId\.value,\s*kGeneratedBodyRuntimeFlags,\s*kRebuildBodyCollisionState\);' `
    'Generated weapon bodies must publish filter and modifier eligibility with one native collision-state rebuild.'

Reject-Pattern 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'kGeneratedBodyRuntimeFlags\s*=\s*0x0*4020000' `
    'Processed-manifold event opt-in must not leak into the shared generated-body defaults.'

# The dormant overlay flag is now the primary in-game shape/contact diagnostic.
# A surviving runtime mismatch must distinguish callback admission, snapshot
# admission, and immediate visual-authority readback without hot-path log spam.

Require-Pattern 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'rockDebugDrawDynamicWeaponColliders[\s\S]*proxyBodyIdForDebug\(\)[\s\S]*DWC ACTIVE[\s\S]*addScreenTextLine\(20\.0f,\s*90\.0f[\s\S]*DWC COMPOUND[\s\S]*authorityBody[\s\S]*children=%u points=%llu[\s\S]*gripPivot[\s\S]*callbacks pair/obstacle/raw/manifold/admit[\s\S]*snapshot read/valid/id/contact/tele' `
    'The dedicated debug flag must draw the compound and expose geometry, authority, pivot, callback, and snapshot telemetry.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic weapon collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic weapon collision source boundary passed.'

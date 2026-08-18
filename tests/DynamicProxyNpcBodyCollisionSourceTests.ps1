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

$layers = 'src/physics-interaction/collision/CollisionLayerPolicy.h'
$core = 'src/physics-interaction/core/PhysicsInteraction.cpp'

# The compiled default must stay fail-closed: without the INI key the NPC
# body-contact experiment never activates.
Require-Pattern 'src/RockConfig.h' `
    'rockDynamicColliderNpcBodyCollisionEnabled\s*=\s*false' `
    'The compiled NPC body-collision default must remain false (fail closed without the INI key).'

# The shipped interim-melee config enables the experiment for release users.
foreach ($ini in @('data/config/ROCK.ini', 'data/mod/ROCK_Config/ROCK.ini')) {
    Require-Pattern $ini `
        '(?m)^bDynamicCollidersNpcBodyCollisionEnabled\s*=\s*true\s*$' `
        "$ini must ship the interim NPC body-collision melee experiment enabled."
}

# Player-exclusion coupling: the feature may only enable while the native
# character-controller contact filter is on, because that toggle owns the
# bit-14 suppression leases that keep the player's own biped-family bodies
# out of the dynamic proxies (self-propulsion feedback otherwise).
Require-Pattern $core `
    'dynamicProxyNpcBodyCollisionEnabled\(\)[\s\S]{0,800}rockDynamicColliderNpcBodyCollisionEnabled\s*&&\s*[\r\n\s]*g_rockConfig\.rockNativeCharacterControllerObjectContactFilterEnabled' `
    'NPC body collision must stay conjunction-gated on the native controller contact filter (player suppression leases).'

# The proxy masks must add exactly the biped family, and only under the flag.
Require-Pattern $layers `
    'dynamicProxyNpcBodyLayerBits\(\)[\s\S]{0,300}FO4_LAYER_BIPED\)[\s\S]{0,120}FO4_LAYER_DEADBIP\)[\s\S]{0,120}FO4_LAYER_BIPED_NO_CC\)' `
    'dynamicProxyNpcBodyLayerBits must cover exactly the biped family (8/32/33).'
Require-Pattern $layers `
    'buildRockDynamicHandProxyExpectedMask\([\s\S]{0,600}npcBodyCollisionEnabled[\s\S]{0,900}if\s*\(npcBodyCollisionEnabled\)\s*\{[\s\S]{0,120}dynamicProxyNpcBodyLayerBits\(\)' `
    'The hand proxy mask must gate the biped family behind npcBodyCollisionEnabled.'
Require-Pattern $layers `
    'buildRockDynamicWeaponProxyExpectedMask\([\s\S]{0,600}npcBodyCollisionEnabled[\s\S]{0,900}if\s*\(npcBodyCollisionEnabled\)\s*\{[\s\S]{0,120}dynamicProxyNpcBodyLayerBits\(\)' `
    'The weapon proxy mask must gate the biped family behind npcBodyCollisionEnabled.'

# Watchdog tolerance: the BIPED/BIPED_NO_CC rows are SCISSORS-owned while its
# global collision policy runs; ROCK must not fight over biped bits on the
# proxy rows or the two plugins churn the whole matrix.
Require-Pattern $core `
    'dynamicHandProxyMaskDrifted\s*=\s*_expectedDynamicHandProxyLayerMask\s*!=\s*0\s*&&[\r\n\s]*!collision_layer_policy::bodyManagedLayerMaskMatches' `
    'The right-hand proxy drift check must use the biped-tolerant compare.'
Require-Pattern $core `
    'dynamicLeftHandProxyMaskDrifted\s*=[\s\S]{0,300}bodyManagedLayerMaskMatches' `
    'The left-hand proxy drift check must use the biped-tolerant compare.'
Require-Pattern $core `
    'dynamicWeaponProxyMaskDrifted\s*=\s*_expectedDynamicWeaponProxyLayerMask\s*!=\s*0\s*&&[\r\n\s]*!collision_layer_policy::bodyManagedLayerMaskMatches' `
    'The weapon proxy drift check must use the biped-tolerant compare.'

if ($failures.Count -gt 0) {
    $failures | ForEach-Object { Write-Error $_ -ErrorAction Continue }
    exit 1
}

Write-Host 'DynamicProxyNpcBodyCollisionSourceTests passed.'
exit 0

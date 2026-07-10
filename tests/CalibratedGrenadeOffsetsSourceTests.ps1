param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
)

$ErrorActionPreference = "Stop"

function Require-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -LiteralPath $path -Raw
    if ($text -notmatch $Pattern) {
        throw $Message
    }
}

function Reject-Text {
    param(
        [string]$RelativePath,
        [string]$Pattern,
        [string]$Message
    )

    $path = Join-Path $Root $RelativePath
    $text = Get-Content -LiteralPath $path -Raw
    if ($text -match $Pattern) {
        throw $Message
    }
}

Require-Text 'src/RockConfig.h' `
    'rockCalibratedGrenadeOffsetsEnabled\s*=\s*true' `
    'Calibrated grenade offsets must have an explicit enabled compiled default.'

Require-Text 'src/RockConfig.cpp' `
    'rockCalibratedGrenadeOffsetsEnabled\s*=\s*true[\s\S]*GetBoolValue\([\s\S]*REALISTIC_WEAPONS_SECTION,[\s\S]*"bCalibratedGrenadeOffsetsEnabled"' `
    'RockConfig must reset and load the calibrated grenade offset flag from RealisticWeapons.'

Require-Text 'data/config/ROCK.ini' `
    '\[RealisticWeapons\][\s\S]*bCalibratedGrenadeOffsetsEnabled\s*=\s*true' `
    'The embedded first-run INI must document and enable calibrated grenade offsets.'

Require-Text 'data/mod/ROCK_Config/ROCK.ini' `
    '\[RealisticWeapons\][\s\S]*bCalibratedGrenadeOffsetsEnabled\s*=\s*true' `
    'The mod-package INI must document and enable calibrated grenade offsets.'

Require-Text 'src/physics-interaction/grenade/CalibratedGrenadeOffsetPolicy.h' `
    'selectPreset[\s\S]*!input\.enabled\s*\|\|\s*!input\.isGrenade[\s\S]*input\.isMolotov\s*\?\s*Preset::Molotov\s*:\s*Preset::GenericGrenade' `
    'Preset policy must fail closed, then give Molotov semantic classification priority.'

Require-Text 'src/physics-interaction/grenade/CalibratedGrenadeOffsetPolicy.h' `
    'kGenericGrenadeLeft[\s\S]*kGenericGrenadeRight[\s\S]*kMolotovLeft[\s\S]*kMolotovRight[\s\S]*handOffsetForPreset' `
    'All four immutable hand calibrations must remain present and selectable.'

Reject-Text 'src/physics-interaction/grenade/CalibratedGrenadeOffsetPolicy.h' `
    '00004877|0010C3C6|CheatTerminal\.esp|Fallout4\.esm' `
    'Built-in calibration selection must never special-case source form IDs or plugins.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.h' `
    'enum class GrenadeKind[\s\S]*NotGrenade[\s\S]*Generic[\s\S]*Molotov[\s\S]*classifyGrenadeRef' `
    'Loose grenade runtime must expose one semantic classifier for attach and detonation behavior.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'classifyGrenadeSources[\s\S]*isMolotovGrenade\([\s\S]*GrenadeKind::Molotov[\s\S]*resolveGrenadeRuntimeDataForSources[\s\S]*classifyGrenadeSources\([\s\S]*kind\s*==\s*GrenadeKind::Molotov' `
    'Detonation behavior must consume the same semantic Molotov classifier as calibrated offsets.'

Require-Text 'src/physics-interaction/grenade/LooseGrenadeRuntime.cpp' `
    'classifyGrenadeRef[\s\S]*resolveReferenceInstanceData\(ref\)[\s\S]*resolveProjectile\(weapon,\s*instanceData\.get\(\)\)[\s\S]*resolveReferenceObjectInstanceExtra\(ref\)[\s\S]*classifyGrenadeSources' `
    'Reference classification must include weapon, instance, projectile, and active OMOD evidence.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'resolveGrabOffsetSource[\s\S]*rockCalibratedGrenadeOffsetsEnabled[\s\S]*classifyGrenadeRef\(refr\)[\s\S]*selectPreset[\s\S]*handOffsetForPreset[\s\S]*return source;[\s\S]*tryLoadSavedGrabOffsetHandOffset' `
    'Enabled calibrated grenade offsets must resolve before the per-object saved-offset fallback.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'resolvedGrabOffsetSource\s*=\s*resolveGrabOffsetSource[\s\S]*resolveGrabOffsetAttachSource\([\s\S]*resolvedGrabOffsetSource[\s\S]*resolveGrabOffsetFingerPoseSource\(resolvedGrabOffsetSource\)' `
    'One grab-time offset resolution must feed both the object transform and finger pose.'

Require-Text 'src/physics-interaction/hand/HandGrab.cpp' `
    'calibratedGrenadeOffset[\s\S]*calibratedMolotovOffset[\s\S]*savedGrabOffset' `
    'Runtime diagnostics must distinguish calibrated grenade, calibrated Molotov, and saved sources.'

Write-Host "Calibrated grenade offset source guard passed."

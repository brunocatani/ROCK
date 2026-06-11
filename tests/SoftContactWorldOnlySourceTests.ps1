param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Read-Source {
    param([string]$RelativePath)

    $path = Join-Path $Root $RelativePath
    if (-not (Test-Path -LiteralPath $path)) {
        $failures.Add("$RelativePath`: missing required file")
        return ''
    }

    return Get-Content -Raw -LiteralPath $path
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

$removedConfigPattern = 'rockSoftContact(?:Enabled|HandHandEnabled|WeaponHandEnabled|BodyEnabled|RadiusPaddingGameUnits|MaxCorrectionGameUnits|WeaponHand)'
$removedIniPattern = 'bSoftContact(?:Enabled|HandHandEnabled|WeaponHandEnabled|BodyEnabled)|fSoftContact(?:RadiusPaddingGameUnits|MaxCorrectionGameUnits|WeaponHand)'

Reject-Text 'src/RockConfig.h' $removedConfigPattern `
    'RockConfig must not expose removed non-world or legacy global soft-contact settings.'
Reject-Text 'src/RockConfig.cpp' "$removedConfigPattern|$removedIniPattern" `
    'RockConfig loader must not read removed non-world or legacy global soft-contact INI keys.'
Require-Text 'src/RockConfig.h' 'rockSoftContactWorldEnabled' `
    'World soft contact must keep a source config toggle.'
Require-Text 'src/RockConfig.cpp' 'bSoftContactWorldEnabled' `
    'World soft contact must keep reading its source config toggle.'
Require-Text 'src/RockConfig.h' 'rockSoftContactWorldShapeCastFilterInfo' `
    'World soft contact must own a dedicated fallback shape-cast filter config.'
Require-Text 'src/RockConfig.cpp' 'sSoftContactWorldShapeCastFilterInfo' `
    'World soft contact must read its dedicated fallback shape-cast filter config.'
Require-Text 'src/RockConfig.h' 'rockSoftContactWorldCachedPlaneMaxClearDistanceGameUnits[\s\S]*rockSoftContactWorldReleaseLerpEnabled[\s\S]*rockSoftContactWorldReleaseLerpTimeMin[\s\S]*rockSoftContactWorldReleaseLerpTimeMax' `
    'World soft contact must expose clear-distance and release-lerp tuning.'
Require-Text 'src/RockConfig.cpp' 'fSoftContactWorldCachedPlaneMaxClearDistanceGameUnits[\s\S]*bSoftContactWorldReleaseLerpEnabled[\s\S]*fSoftContactWorldReleaseLerpTimeMin[\s\S]*fSoftContactWorldReleaseLerpTimeMax' `
    'World soft contact must read clear-distance and release-lerp tuning.'

Reject-Text 'data/config/ROCK.ini' $removedIniPattern `
    'Default config must not document removed soft-contact toggles or tuning.'
Reject-Text 'data/mod/ROCK_Config/ROCK.ini' $removedIniPattern `
    'Packaged mod config must not document removed soft-contact toggles or tuning.'
Require-Text 'data/config/ROCK.ini' 'bSoftContactWorldEnabled\s*=\s*true' `
    'Default config must expose the world soft-contact toggle.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' 'bSoftContactWorldEnabled\s*=\s*true' `
    'Packaged mod config must expose the world soft-contact toggle.'
Require-Text 'data/config/ROCK.ini' 'sSoftContactWorldShapeCastFilterInfo\s*=\s*000B002D' `
    'Default config must expose the dedicated world soft-contact fallback query filter.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' 'sSoftContactWorldShapeCastFilterInfo\s*=\s*000B002D' `
    'Packaged mod config must expose the dedicated world soft-contact fallback query filter.'
Require-Text 'data/config/ROCK.ini' 'fSoftContactWorldCachedPlaneMaxClearDistanceGameUnits\s*=\s*18\.0[\s\S]*bSoftContactWorldReleaseLerpEnabled\s*=\s*true' `
    'Default config must expose soft-contact clear distance and release lerp.'
Require-Text 'data/mod/ROCK_Config/ROCK.ini' 'fSoftContactWorldCachedPlaneMaxClearDistanceGameUnits\s*=\s*18\.0[\s\S]*bSoftContactWorldReleaseLerpEnabled\s*=\s*true' `
    'Packaged mod config must expose soft-contact clear distance and release lerp.'

Reject-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'buildHandShapes|buildBodyShapes|solveShapeAgainstShapes|solveShapeAgainstWeapon|RuntimeShape|DirectSkeletonBoneSnapshot|_bodyReader|std::vector<|WeaponCollision|NiAVObject' `
    'SoftContactRuntime must not retain hand-hand, body, weapon, or per-frame shape-vector solve paths.'
Reject-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'ContactKind::(?:HandHand|WeaponHand|Body)|rockSoftContact(?:Enabled|HandHandEnabled|WeaponHandEnabled|BodyEnabled|RadiusPaddingGameUnits|MaxCorrectionGameUnits|WeaponHand)' `
    'SoftContactRuntime must not branch on removed non-world contact kinds or config.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' '!g_rockConfig\.rockSoftContactWorldEnabled\s*\|\|\s*!frame\.worldReady\s*\|\|\s*frame\.menuBlocked' `
    'SoftContactRuntime must use the world toggle as the runtime gate.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'solveWorldStaticContact\(' `
    'SoftContactRuntime must keep the world contact solve path.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'worldOnly=yes' `
    'SoftContactRuntime active logging must report world-only behavior.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'withinClearDistanceLimit\([\s\S]*applyReleaseBlend[\s\S]*blendTransformOverDuration' `
    'SoftContactRuntime must clear cached planes by all-direction distance and smooth visual release.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'CandidateManifold[\s\S]*addCandidateToManifold[\s\S]*correctionForManifold[\s\S]*collectNativeWorldStaticContacts[\s\S]*solveCachedWorldPlaneContacts[\s\S]*collectWorldProbeCastContacts' `
    'SoftContactRuntime must collect bounded multi-direction world contacts instead of collapsing to one candidate.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'projectTrackedMagnetPlaneSetCorrection' `
    'SoftContactRuntime must solve combined correction from the accepted contact manifold.'
Reject-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'keepStrongerCandidate|Candidate\s+best\{\}' `
    'SoftContactRuntime must not retain the old single-winner soft-contact path.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'collisionFilterInfo\s*=\s*g_rockConfig\.rockSoftContactWorldShapeCastFilterInfo' `
    'SoftContactRuntime fallback casts must use the dedicated world soft-contact filter, not selection tuning.'
Reject-Text 'src/physics-interaction/contact/SoftContactRuntime.cpp' 'collisionFilterInfo\s*=\s*g_rockConfig\.rockSelectionShapeCastFilterInfo' `
    'SoftContactRuntime fallback casts must not share selection shape-cast filter tuning.'

Require-Text 'src/physics-interaction/contact/SoftContactRuntime.h' 'enum class SoftContactDebugSource' `
    'Soft contact debug snapshots must expose source kind for world-contact tuning.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.h' 'SoftContactDebugSource source' `
    'Soft contact debug contacts must carry source kind.'
Require-Text 'src/physics-interaction/contact/SoftContactRuntime.h' 'kMaxWorldContactManifoldContactsPerHand[\s\S]*cachedWorldPlanes' `
    'SoftContactRuntime must cache a bounded set of world contact planes per hand.'
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'softContactSourceName' `
    'Debug overlay must render soft-contact source kind.'

Reject-Text 'src/physics-interaction/contact/SoftContactMath.h' 'ContactKind\s*:\s*std::uint8_t[\s\S]{0,120}(HandHand|WeaponHand|Body)|struct\s+(?:Capsule|Aabb)\b|solveCapsule(?:Pair|Aabb)|compliantHardStopResponseScale|projectCompliantTrackedMagnetCorrection' `
    'SoftContactMath must not retain removed synthetic capsule or weapon compliance helpers.'
Require-Text 'src/physics-interaction/contact/SoftContactMath.h' 'projectTrackedMagnetPlaneSetCorrection' `
    'SoftContactMath must expose the bounded multi-plane correction solver.'

Reject-Text 'src/physics-interaction/weapon/WeaponCollision.h' 'tryFindSoftContactForCapsule|WeaponSoftContactResult' `
    'WeaponCollision must not expose the removed hand-weapon soft-contact query.'
Reject-Text 'src/physics-interaction/weapon/WeaponCollision.cpp' 'tryFindSoftContactForCapsule|SoftContactMath|solveCapsuleAabb|WeaponSoftContactResult' `
    'WeaponCollision must not retain the removed hand-weapon soft-contact implementation.'
Reject-Text 'src/physics-interaction/weapon/WeaponTypes.h' 'WeaponSoftContactResult' `
    'WeaponTypes must not retain the removed soft-contact result payload.'

Reject-Text 'src/physics-interaction/debug/DebugBodyOverlay.h' 'RightSoftContact|LeftSoftContact|RightSoftContactCorrection|LeftSoftContactCorrection' `
    'Debug overlay roles must only expose world soft-contact markers.'
Reject-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.inl' 'RightSoftContact|LeftSoftContact|RightSoftContactCorrection|LeftSoftContactCorrection' `
    'Debug overlay drawing must not branch to removed non-world soft-contact markers.'

if ($failures.Count -gt 0) {
    Write-Host 'SoftContactWorldOnlySourceTests failed:' -ForegroundColor Red
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'SoftContactWorldOnlySourceTests passed.' -ForegroundColor Green

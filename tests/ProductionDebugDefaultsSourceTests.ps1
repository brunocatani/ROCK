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
        [string]$Value,
        [string]$Message
    )

    $escapedKey = [regex]::Escape($Key)
    $escapedValue = [regex]::Escape($Value)
    Require-Text 'data/config/ROCK.ini' "(?m)^\s*$escapedKey\s*=\s*$escapedValue\s*$" $Message
}

$quietDebugDefaults = @(
    @{ Key = 'bDebugShowColliders'; Member = 'rockDebugShowColliders' },
    @{ Key = 'bDebugShowTargetColliders'; Member = 'rockDebugShowTargetColliders' },
    @{ Key = 'bDebugShowHandAxes'; Member = 'rockDebugShowHandAxes' },
    @{ Key = 'bDebugShowGrabPivots'; Member = 'rockDebugShowGrabPivots' },
    @{ Key = 'bDebugShowPalmVectors'; Member = 'rockDebugShowPalmVectors' },
    @{ Key = 'bDebugDrawHandColliders'; Member = 'rockDebugDrawHandColliders' },
    @{ Key = 'bDebugDrawHandBoneColliders'; Member = 'rockDebugDrawHandBoneColliders' },
    @{ Key = 'bDebugDrawHandBoneContacts'; Member = 'rockDebugDrawHandBoneContacts' },
    @{ Key = 'bDebugDrawSoftContacts'; Member = 'rockDebugDrawSoftContacts' },
    @{ Key = 'bDebugDrawWeaponColliders'; Member = 'rockDebugDrawWeaponColliders' },
    @{ Key = 'bDebugContactTargetIdentityLogging'; Member = 'rockDebugContactTargetIdentityLogging' },
    @{ Key = 'bDebugGrabFrameLogging'; Member = 'rockDebugGrabFrameLogging' },
    @{ Key = 'bDebugGrabTransformTelemetry'; Member = 'rockDebugGrabTransformTelemetry' },
    @{ Key = 'bDebugGrabTransformTelemetryText'; Member = 'rockDebugGrabTransformTelemetryText' },
    @{ Key = 'bDebugGrabTransformTelemetryAxes'; Member = 'rockDebugGrabTransformTelemetryAxes' },
    @{ Key = 'bDebugShowRootFlattenedFingerSkeletonMarkers'; Member = 'rockDebugShowRootFlattenedFingerSkeletonMarkers' },
    @{ Key = 'bDebugShowSkeletonBoneVisualizer'; Member = 'rockDebugShowSkeletonBoneVisualizer' },
    @{ Key = 'bDebugDrawSkeletonBoneAxes'; Member = 'rockDebugDrawSkeletonBoneAxes' },
    @{ Key = 'bDebugLogSkeletonBones'; Member = 'rockDebugLogSkeletonBones' },
    @{ Key = 'bDebugLogSkeletonBoneTruncation'; Member = 'rockDebugLogSkeletonBoneTruncation' },
    @{ Key = 'bPerformanceProfilerEnabled'; Member = 'rockPerformanceProfilerEnabled' },
    @{ Key = 'bPerformanceProfilerOverlayText'; Member = 'rockPerformanceProfilerOverlayText' }
)

foreach ($entry in $quietDebugDefaults) {
    $member = [regex]::Escape($entry.Member)
    Require-Text 'src/RockConfig.h' "bool\s+$member\s*=\s*false;" "$($entry.Member) must default off in RockConfig.h."
    Require-Text 'src/RockConfig.cpp' "$member\s*=\s*false;" "$($entry.Member) must reset off in RockConfig::resetToDefaults."
    Require-Ini-Key $entry.Key 'false' "Packaged fallback ROCK.ini must keep $($entry.Key) = false."
}

$enabledGameplayDefaults = @(
    @{ Key = 'bEnabled'; Member = 'rockEnabled' },
    @{ Key = 'bWeaponCollisionEnabled'; Member = 'rockWeaponCollisionEnabled' },
    @{ Key = 'bWeaponCollisionStaticWorldEnabled'; Member = 'rockWeaponCollisionStaticWorldEnabled' },
    @{ Key = 'bWeaponCollisionNativeVisualRemapEnabled'; Member = 'rockWeaponCollisionNativeVisualRemapEnabled' },
    @{ Key = 'bSoftContactEnabled'; Member = 'rockSoftContactEnabled' },
    @{ Key = 'bSoftContactHandHandEnabled'; Member = 'rockSoftContactHandHandEnabled' },
    @{ Key = 'bSoftContactWeaponHandEnabled'; Member = 'rockSoftContactWeaponHandEnabled' },
    @{ Key = 'bSoftContactBodyEnabled'; Member = 'rockSoftContactBodyEnabled' },
    @{ Key = 'bSoftContactWorldEnabled'; Member = 'rockSoftContactWorldEnabled' },
    @{ Key = 'bNativeCharacterControllerObjectContactFilterEnabled'; Member = 'rockNativeCharacterControllerObjectContactFilterEnabled' },
    @{ Key = 'bHighlightEnabled'; Member = 'rockHighlightEnabled' },
    @{ Key = 'bBodyBoneCollidersEnabled'; Member = 'rockBodyBoneCollidersEnabled' },
    @{ Key = 'bBodyBoneCollisionStaticWorldEnabled'; Member = 'rockBodyBoneCollisionStaticWorldEnabled' },
    @{ Key = 'bHandCollisionStaticWorldEnabled'; Member = 'rockHandCollisionStaticWorldEnabled' },
    @{ Key = 'bGrabHapticsEnabled'; Member = 'rockGrabHapticsEnabled' },
    @{ Key = 'bPullCatchWideReacquireEnabled'; Member = 'rockPullCatchWideReacquireEnabled' }
)

foreach ($entry in $enabledGameplayDefaults) {
    $member = [regex]::Escape($entry.Member)
    Require-Text 'src/RockConfig.h' "bool\s+$member\s*=\s*true;" "$($entry.Member) must remain enabled in RockConfig.h."
    Require-Text 'src/RockConfig.cpp' "$member\s*=\s*true;" "$($entry.Member) must reset enabled in RockConfig::resetToDefaults."
    Require-Ini-Key $entry.Key 'true' "Packaged fallback ROCK.ini must keep gameplay key $($entry.Key) = true."
}

Require-Ini-Key 'iLogLevel' '2' 'Packaged fallback ROCK.ini must use info logging in the production profile.'
Require-Text 'src/RockConfig.h' 'int rockLogLevel = 2;' 'Compiled log level default must remain info.'
Require-Text 'src/RockConfig.cpp' 'rockLogLevel = logging_policy::DefaultLogLevel;' 'RockConfig reset must use the logging policy default.'

if ($failures.Count -gt 0) {
    Write-Host 'Production debug defaults source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Production debug defaults source boundary passed.'

param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

# The active production INI is the runtime authority; packaged INI and compiled
# defaults are fallback artifacts. This source-boundary test keeps those fallback
# artifacts coherent, rejects removed knobs, and optionally audits the local prod
# INI when it exists on this machine so runtime keys cannot silently go unread.

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

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    if ((Read-Text $Path) -match $Pattern) {
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
    Require-Text 'data/config/ROCK.ini' "(?m)^\s*$escapedKey\s*=\s*$escapedValue\s*$" "Packaged fallback ROCK.ini must keep $Key = $Value in parity with compiled fallback defaults."
}

function Read-IniKeyMap {
    param([string]$Path)

    $map = [ordered]@{}
    Get-Content -LiteralPath $Path | ForEach-Object {
        if ($_ -match '^\s*([A-Za-z0-9_]+)\s*=\s*(.*?)\s*$') {
            $map[$matches[1]] = $matches[2]
        }
    }
    return $map
}

function Require-ProdIniCoverage {
    $prodPath = 'C:\Users\SENECA\Documents\My Games\Fallout4VR\ROCK_Config\ROCK.ini'
    if (-not (Test-Path -LiteralPath $prodPath)) {
        return
    }

    $prod = Read-IniKeyMap $prodPath
    foreach ($key in $prod.Keys) {
        $escapedKeyLiteral = [regex]::Escape('"' + $key + '"')
        Require-Text 'src/RockConfig.cpp' $escapedKeyLiteral "Active prod ROCK.ini key $key must be read by RockConfig.cpp or explicitly removed from prod."
    }

    foreach ($oldKey in @('bDebugShowFrikFingerSkeletonMarkers', 'fDebugFrikFingerSkeletonMarkerSize')) {
        if ($prod.Contains($oldKey)) {
            $failures.Add("Active prod ROCK.ini must not contain removed skeleton debug alias $oldKey.")
        }
    }

    if (-not $prod.Contains('fCloseSelectionBehindPalmToleranceGameUnits')) {
        $failures.Add('Active prod ROCK.ini must explicitly expose fCloseSelectionBehindPalmToleranceGameUnits.')
    } elseif ($prod['fCloseSelectionBehindPalmToleranceGameUnits'] -ne '2.0') {
        $failures.Add("Active prod ROCK.ini must keep fCloseSelectionBehindPalmToleranceGameUnits = 2.0 unless the production tune changes.")
    }
}

Require-ProdIniCoverage

Require-Text 'src/RockConfig.h' 'bool rockGrabInputIntentStateEnabled = true;' 'RockConfig member initializer for grab input intent must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabInputIntentStateEnabled = true;' 'RockConfig::resetToDefaults must enable grab input intent when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bGrabInputIntentStateEnabled' 'true'
Require-Text 'src/RockConfig.h' 'bool rockReverseFarGrabNormal = true;' 'RockConfig member initializer for far-grab normal reversal must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockReverseFarGrabNormal = true;' 'RockConfig::resetToDefaults must match far-grab normal reversal default.'
Require-Ini-Key 'bReverseFarGrabNormal' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabInputLeewaySeconds = 0\.12f;' 'RockConfig member initializer for grab input leeway must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabInputLeewaySeconds = 0\.12f;' 'RockConfig::resetToDefaults must match grab input leeway default.'
Require-Ini-Key 'fGrabInputLeewaySeconds' '0.12'
Require-Text 'src/RockConfig.h' 'float rockGrabInputForceSeconds = 0\.08f;' 'RockConfig member initializer for grab input force window must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabInputForceSeconds = 0\.08f;' 'RockConfig::resetToDefaults must match grab input force default.'
Require-Ini-Key 'fGrabInputForceSeconds' '0.08'

Require-Text 'src/RockConfig.h' 'bool rockGrabAdaptiveMotorEnabled = true;' 'RockConfig member initializer for adaptive grab motor must match production-tuned packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabAdaptiveMotorEnabled = true;' 'RockConfig::resetToDefaults must enable adaptive grab motor when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bGrabAdaptiveMotorEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabNativeMouseSpringLinearResponseScale = 1\.35f;' 'RockConfig member initializer for native mouse-spring linear response must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNativeMouseSpringLinearResponseScale = kDefaultGrabNativeMouseSpringLinearResponseScale;' 'RockConfig::resetToDefaults must use the native mouse-spring linear response default.'
Require-Ini-Key 'fGrabNativeMouseSpringLinearResponseScale' '1.35'
Require-Text 'src/RockConfig.h' 'float rockGrabNativeMouseSpringAngularResponseScale = 0\.75f;' 'RockConfig member initializer for native mouse-spring angular response must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNativeMouseSpringAngularResponseScale = kDefaultGrabNativeMouseSpringAngularResponseScale;' 'RockConfig::resetToDefaults must use the native mouse-spring angular response default.'
Require-Ini-Key 'fGrabNativeMouseSpringAngularResponseScale' '0.75'
Require-Text 'src/RockConfig.h' 'float rockGrabNativeMouseSpringAngularClampScale = 0\.85f;' 'RockConfig member initializer for native mouse-spring angular clamp must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNativeMouseSpringAngularClampScale = kDefaultGrabNativeMouseSpringAngularClampScale;' 'RockConfig::resetToDefaults must use the native mouse-spring angular clamp default.'
Require-Ini-Key 'fGrabNativeMouseSpringAngularClampScale' '0.85'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponNativeLinearResponseMultiplier = 1\.0f;' 'Loose non-equipped weapon native linear multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponNativeLinearResponseMultiplier = kDefaultGrabLooseWeaponNativeLinearResponseMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon native linear multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponNativeLinearResponseMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponNativeAngularResponseMultiplier = 1\.0f;' 'Loose non-equipped weapon native angular multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponNativeAngularResponseMultiplier = kDefaultGrabLooseWeaponNativeAngularResponseMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon native angular multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponNativeAngularResponseMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponNativeAngularClampMultiplier = 1\.0f;' 'Loose non-equipped weapon native angular clamp multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponNativeAngularClampMultiplier = kDefaultGrabLooseWeaponNativeAngularClampMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon native angular clamp multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponNativeAngularClampMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponAdaptiveLeadMultiplier = 1\.0f;' 'Loose non-equipped weapon adaptive lead multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponAdaptiveLeadMultiplier = kDefaultGrabLooseWeaponAdaptiveLeadMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon adaptive lead multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponAdaptiveLeadMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = 1\.0f;' 'Loose non-equipped weapon shared linear tau multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponSharedConstraintLinearTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintLinearTauMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon shared linear tau multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintLinearTauMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = 1\.0f;' 'Loose non-equipped weapon shared angular tau multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponSharedConstraintAngularTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintAngularTauMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon shared angular tau multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintAngularTauMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = 1\.0f;' 'Loose non-equipped weapon shared collision tau multiplier must default neutral.'
Require-Text 'src/RockConfig.cpp' 'rockGrabLooseWeaponSharedConstraintCollisionTauMultiplier = kDefaultGrabLooseWeaponSharedConstraintCollisionTauMultiplier;' 'RockConfig::resetToDefaults must keep loose weapon shared collision tau multiplier neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintCollisionTauMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintLinearDampingMultiplier = 1\.0f;' 'Loose non-equipped weapon shared linear damping multiplier must default neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintLinearDampingMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintAngularDampingMultiplier = 1\.0f;' 'Loose non-equipped weapon shared angular damping multiplier must default neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintAngularDampingMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintMaxForceMultiplier = 1\.0f;' 'Loose non-equipped weapon shared max force multiplier must default neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintMaxForceMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintAngularForceMultiplier = 1\.0f;' 'Loose non-equipped weapon shared angular force multiplier must default neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintAngularForceMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier = 1\.0f;' 'Loose non-equipped weapon shared linear recovery multiplier must default neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintLinearRecoveryMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier = 1\.0f;' 'Loose non-equipped weapon shared angular recovery multiplier must default neutral.'
Require-Ini-Key 'fGrabLooseWeaponSharedConstraintAngularRecoveryMultiplier' '1.0'
Require-Text 'src/RockConfig.h' 'bool rockGrabAdaptiveHeldResponseEnabled = true;' 'Adaptive held response must default on for ordinary native mouse-spring grabs.'
Require-Text 'src/RockConfig.cpp' 'rockGrabAdaptiveHeldResponseEnabled = true;' 'RockConfig::resetToDefaults must enable adaptive held response by default.'
Require-Ini-Key 'bGrabAdaptiveHeldResponseEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabAdaptiveHeldLeadTimeMax = 0\.035f;' 'Adaptive held lead-time default must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabAdaptiveHeldLeadTimeMax = kDefaultGrabAdaptiveHeldLeadTimeMax;' 'RockConfig::resetToDefaults must use the adaptive held lead-time default.'
Require-Ini-Key 'fGrabAdaptiveHeldLeadTimeMax' '0.035'
Require-Text 'src/RockConfig.h' 'float rockGrabAdaptiveHeldMaxLeadDistanceGameUnits = 8\.0f;' 'Adaptive held max lead distance default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabAdaptiveHeldMaxLeadDistanceGameUnits' '8.0'
Require-Text 'src/RockConfig.h' 'float rockGrabAdaptiveHeldMaxAngularLeadDegrees = 18\.0f;' 'Adaptive held max angular lead default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabAdaptiveHeldMaxAngularLeadDegrees' '18.0'
Require-Text 'src/RockConfig.h' 'float rockGrabAdaptiveHeldFullErrorGameUnits = 20\.0f;' 'Adaptive held full-error default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabAdaptiveHeldFullErrorGameUnits' '20.0'
Require-Text 'src/RockConfig.h' 'float rockGrabAdaptiveHeldCollisionLeadScale = 0\.25f;' 'Adaptive held collision lead scale default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabAdaptiveHeldCollisionLeadScale' '0.25'

Require-Text 'src/RockConfig.h' 'bool rockGrabControllerDerivedThrowVelocityEnabled = true;' 'Controller-derived throw velocity must default on.'
Require-Text 'src/RockConfig.cpp' 'rockGrabControllerDerivedThrowVelocityEnabled = true;' 'RockConfig::resetToDefaults must enable controller-derived throw velocity by default.'
Require-Ini-Key 'bGrabControllerDerivedThrowVelocityEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabThrowObjectVelocityBlend = 0\.35f;' 'Throw object-velocity blend default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabThrowObjectVelocityBlend' '0.35'
Require-Text 'src/RockConfig.h' 'float rockGrabThrowTangentialVelocityScale = 1\.0f;' 'Throw tangential velocity scale default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabThrowTangentialVelocityScale' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabThrowMaxVelocityHavok = 12\.0f;' 'Throw max velocity default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabThrowMaxVelocityHavok' '12.0'
Require-Text 'src/RockConfig.h' 'float rockGrabThrowAngularVelocityScale = 1\.0f;' 'Throw angular velocity scale default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabThrowAngularVelocityScale' '1.0'
Require-Text 'src/RockConfig.h' 'float rockGrabThrowMaxAngularVelocityRadiansPerSecond = 18\.0f;' 'Throw angular velocity cap default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabThrowMaxAngularVelocityRadiansPerSecond' '18.0'
Require-Text 'src/RockConfig.h' 'bool rockGrabThumbSurfaceSafetyEnabled = true;' 'Thumb surface safety must default on when local thumb transforms are enabled.'
Require-Text 'src/RockConfig.cpp' 'rockGrabThumbSurfaceSafetyEnabled = true;' 'RockConfig::resetToDefaults must enable thumb surface safety by default.'
Require-Ini-Key 'bGrabThumbSurfaceSafetyEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabThumbSurfaceSafetyMarginGameUnits = 1\.0f;' 'Thumb surface safety margin default must match packaged fallback ROCK.ini value.'
Require-Ini-Key 'fGrabThumbSurfaceSafetyMarginGameUnits' '1.0'

Require-Text 'src/RockConfig.h' 'bool rockPerformanceProfilerEnabled = false;' 'Performance profiler must default off so production frames do not sample high-resolution timers.'
Require-Text 'src/RockConfig.cpp' 'rockPerformanceProfilerEnabled = false;' 'RockConfig::resetToDefaults must keep the performance profiler opt-in.'
Require-Ini-Key 'bPerformanceProfilerEnabled' 'false'
Require-Text 'src/RockConfig.h' 'int rockPerformanceProfilerLogIntervalFrames = 300;' 'Performance profiler log interval default must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPerformanceProfilerLogIntervalFrames = 300;' 'RockConfig::resetToDefaults must match the profiler log interval fallback.'
Require-Ini-Key 'iPerformanceProfilerLogIntervalFrames' '300'
Require-Text 'src/RockConfig.h' 'int rockPerformanceProfilerWarmupFrames = 120;' 'Performance profiler warmup default must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPerformanceProfilerWarmupFrames = 120;' 'RockConfig::resetToDefaults must match the profiler warmup fallback.'
Require-Ini-Key 'iPerformanceProfilerWarmupFrames' '120'
Require-Text 'src/RockConfig.h' 'bool rockPerformanceProfilerOverlayText = false;' 'Performance profiler overlay text must default off.'
Require-Text 'src/RockConfig.cpp' 'rockPerformanceProfilerOverlayText = false;' 'RockConfig::resetToDefaults must keep profiler overlay text opt-in.'
Require-Ini-Key 'bPerformanceProfilerOverlayText' 'false'

Require-Text 'src/RockConfig.h' 'bool rockDebugGrabFrameLogging = false;' 'RockConfig member initializer for grab frame logging must match production quiet default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugGrabFrameLogging = false;' 'RockConfig::resetToDefaults must match grab frame logging quiet default.'
Require-Ini-Key 'bDebugGrabFrameLogging' 'false'
Require-Text 'src/RockConfig.h' 'bool rockDebugGrabTransformTelemetry = false;' 'RockConfig member initializer for grab transform telemetry must match production quiet default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugGrabTransformTelemetry = false;' 'RockConfig::resetToDefaults must match grab transform telemetry quiet default.'
Require-Ini-Key 'bDebugGrabTransformTelemetry' 'false'
Require-Text 'src/RockConfig.h' 'int rockDebugGrabTransformTelemetryLogIntervalFrames = 1;' 'RockConfig member initializer for grab telemetry interval must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugGrabTransformTelemetryLogIntervalFrames = 1;' 'RockConfig::resetToDefaults must match grab telemetry interval default.'
Require-Ini-Key 'iDebugGrabTransformTelemetryLogIntervalFrames' '1'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowGrabFingerProbes = false;' 'RockConfig member initializer for grab finger probes must match production quiet overlay default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowGrabFingerProbes = false;' 'RockConfig::resetToDefaults must match grab finger probe overlay default.'
Require-Ini-Key 'bDebugShowGrabFingerProbes' 'false'
Require-Text 'src/RockConfig.h' 'bool rockDebugShowGrabPocketNormal = false;' 'RockConfig member initializer for grab pocket normal must match production quiet overlay default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowGrabPocketNormal = false;' 'RockConfig::resetToDefaults must match grab pocket normal overlay default.'
Require-Ini-Key 'bDebugShowGrabPocketNormal' 'false'
Require-Text 'src/RockConfig.h' 'bool rockDebugDrawGrabContactPatch = false;' 'RockConfig member initializer for grab contact patch must match production quiet overlay default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugDrawGrabContactPatch = false;' 'RockConfig::resetToDefaults must match grab contact patch overlay default.'
Require-Ini-Key 'bDebugDrawGrabContactPatch' 'false'

Require-Text 'src/RockConfig.h' 'bool rockWeaponCollisionEnabled = true;' 'RockConfig member initializer for weapon collision must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockWeaponCollisionEnabled = true;' 'RockConfig::resetToDefaults must enable weapon collision when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bWeaponCollisionEnabled' 'true'

Require-Text 'src/RockConfig.h' 'bool rockWeaponCollisionStaticWorldEnabled = true;' 'RockConfig member initializer for weapon static-world collision must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockWeaponCollisionStaticWorldEnabled = true;' 'RockConfig::resetToDefaults must enable weapon static-world collision when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bWeaponCollisionStaticWorldEnabled' 'true'
Require-Text 'src/RockConfig.h' 'bool rockWeaponCollisionNativeVisualRemapEnabled = true;' 'RockConfig member initializer for weapon native visual remap must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockWeaponCollisionNativeVisualRemapEnabled = true;' 'RockConfig::resetToDefaults must enable weapon native visual remap when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bWeaponCollisionNativeVisualRemapEnabled' 'true'
Require-Ini-Key 'iWeaponCollisionGroupingMode' '3'
Require-Text 'src/physics-interaction/weapon/WeaponSemantics.h' 'OptimizedTriShape = 3' 'Weapon collision grouping default must remain the OptimizedTriShape compatibility value.'

Require-Text 'src/RockConfig.h' 'int rockWeaponCollisionSupportFitTargetPoints = 96;' 'RockConfig member initializer for weapon support-fit target points must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockWeaponCollisionSupportFitTargetPoints = kDefaultWeaponCollisionSupportFitTargetPoints;' 'RockConfig::resetToDefaults must match weapon support-fit target default.'
Require-Ini-Key 'iWeaponCollisionSupportFitTargetPoints' '96'

Require-Text 'src/RockConfig.h' 'float rockWeaponCollisionSupportFitMaxErrorGameUnits = 0\.5f;' 'RockConfig member initializer for weapon support-fit max error must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockWeaponCollisionSupportFitMaxErrorGameUnits = kDefaultWeaponCollisionSupportFitMaxErrorGameUnits;' 'RockConfig::resetToDefaults must match weapon support-fit max error default.'
Require-Ini-Key 'fWeaponCollisionSupportFitMaxErrorGameUnits' '0.5'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowColliders = false;' 'RockConfig member initializer for collider overlay must match production quiet default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowColliders = false;' 'RockConfig::resetToDefaults must match packaged collider overlay quiet default.'
Require-Ini-Key 'bDebugShowColliders' 'false'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowTargetColliders = false;' 'RockConfig member initializer for target collider overlay must match production quiet default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowTargetColliders = false;' 'RockConfig::resetToDefaults must match packaged target collider overlay quiet default.'
Require-Ini-Key 'bDebugShowTargetColliders' 'false'

Require-Text 'src/RockConfig.h' 'bool rockDebugShowHandAxes = false;' 'RockConfig member initializer for hand axes overlay must match production quiet default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugShowHandAxes = false;' 'RockConfig::resetToDefaults must match packaged hand axes overlay quiet default.'
Require-Ini-Key 'bDebugShowHandAxes' 'false'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxWeaponBodiesDrawn = 100;' 'RockConfig member initializer for weapon body draw limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxWeaponBodiesDrawn = 100;' 'RockConfig::resetToDefaults must match packaged weapon body draw limit.'
Require-Ini-Key 'iDebugMaxWeaponBodiesDrawn' '100'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxBodyBoneBodiesDrawn = 32;' 'RockConfig member initializer for body bone body draw limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxBodyBoneBodiesDrawn = 32;' 'RockConfig::resetToDefaults must match packaged body bone body draw limit.'
Require-Ini-Key 'iDebugMaxBodyBoneBodiesDrawn' '32'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxShapeGenerationsPerFrame = 100;' 'RockConfig member initializer for shape generation limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxShapeGenerationsPerFrame = 100;' 'RockConfig::resetToDefaults must match packaged shape generation limit.'
Require-Ini-Key 'iDebugMaxShapeGenerationsPerFrame' '100'

Require-Text 'src/RockConfig.h' 'int rockDebugMaxConvexSupportVertices = 6;' 'RockConfig member initializer for convex support vertex limit must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugMaxConvexSupportVertices = 6;' 'RockConfig::resetToDefaults must match packaged convex support vertex limit.'
Require-Ini-Key 'iDebugMaxConvexSupportVertices' '6'

Require-Text 'src/RockConfig.h' 'bool rockDebugContactTargetIdentityLogging = false;' 'RockConfig member initializer for contact target identity logging must match production quiet default.'
Require-Text 'src/RockConfig.cpp' 'rockDebugContactTargetIdentityLogging = false;' 'RockConfig::resetToDefaults must keep contact target identity logging opt-in.'
Require-Ini-Key 'bDebugContactTargetIdentityLogging' 'false'

Require-Text 'src/RockConfig.h' 'int rockDebugContactTargetIdentitySampleMilliseconds = 500;' 'RockConfig member initializer for contact target identity sampling must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockDebugContactTargetIdentitySampleMilliseconds = 500;' 'RockConfig::resetToDefaults must match contact target identity sampling default.'
Require-Ini-Key 'iDebugContactTargetIdentitySampleMilliseconds' '500'

Require-Text 'src/RockConfig.h' 'bool rockHandBoneCollidersRequireAllFingerBones = true;' 'RockConfig member initializer for all-finger hand collider requirement must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockHandBoneCollidersRequireAllFingerBones = true;' 'RockConfig::resetToDefaults must match packaged all-finger hand collider default.'
Require-Ini-Key 'bHandBoneCollidersRequireAllFingerBones' 'true'

Require-Text 'src/RockConfig.h' 'bool rockBodyBoneCollidersEnabled = true;' 'RockConfig member initializer for full body generated colliders must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockBodyBoneCollidersEnabled = true;' 'RockConfig::resetToDefaults must enable full body generated colliders when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bBodyBoneCollidersEnabled' 'true'

Require-Text 'src/RockConfig.h' 'bool rockBodyBoneCollisionStaticWorldEnabled = true;' 'RockConfig member initializer for body static-world collision must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockBodyBoneCollisionStaticWorldEnabled = true;' 'RockConfig::resetToDefaults must enable body static-world collision when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bBodyBoneCollisionStaticWorldEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockBodyBoneColliderPowerArmorRadiusScale = 1\.0f;' 'Power armor body collider radius scale must be configurable from RockConfig.'
Require-Text 'src/RockConfig.cpp' 'rockBodyBoneColliderPowerArmorRadiusScale = 1\.0f;' 'Power armor body collider radius scale reset default must stay explicit.'
Require-Ini-Key 'fBodyBoneColliderStandardRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderStandardLengthScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderStandardConvexRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderPowerArmorRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderPowerArmorLengthScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderPowerArmorConvexRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderTorsoRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderArmRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderLegRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderFootRadiusScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderTorsoLengthScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderArmLengthScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderLegLengthScale' '1.0'
Require-Ini-Key 'fBodyBoneColliderFootLengthScale' '1.0'
Require-Text 'src/RockConfig.h' 'std::string rockBodyBoneColliderZoneScaleOverrides = "";' 'Body collider zone override config must default empty in RockConfig.'
Require-Text 'src/RockConfig.cpp' 'rockBodyBoneColliderZoneScaleOverrides = "";' 'Body collider zone override reset default must stay explicit.'
Require-Ini-Key 'sBodyBoneColliderZoneScaleOverrides' ''
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'bodyColliderTuningSignature\(snapshot\.inPowerArmor\)' 'Body collider tuning changes must trigger generated body rebuilds after config reload.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' 'rockBodyBoneColliderZoneScaleOverrides' 'Descriptor-specific body zone overrides must participate in body collider tuning.'

Require-Text 'src/RockConfig.h' 'bool rockHandCollisionStaticWorldEnabled = true;' 'RockConfig member initializer for hand static-world collision must match reset/default INI value.'
Require-Text 'src/RockConfig.cpp' 'rockHandCollisionStaticWorldEnabled = true;' 'RockConfig::resetToDefaults must enable hand static-world collision when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bHandCollisionStaticWorldEnabled' 'true'

Require-Text 'src/RockConfig.cpp' 'rockNearCastDistanceGameUnits = static_cast<float>\(ini\.GetDoubleValue\(SECTION, "fNearCastDistanceGameUnits", rockNearCastDistanceGameUnits\)\);' 'fNearCastDistanceGameUnits must use its own current default, not fNearDetectionRange.'
Require-Text 'src/RockConfig.h' 'bool rockFarSelectionHmdConeEnabled = true;' 'RockConfig member initializer for far HMD cone gate must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockFarSelectionHmdConeEnabled = true;' 'RockConfig::resetToDefaults must enable the far HMD cone gate by default.'
Require-Ini-Key 'bFarSelectionHmdConeEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockFarSelectionHmdConeHalfAngleDegrees = selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees;' 'RockConfig member initializer for far HMD cone angle must use the policy-owned default.'
Require-Text 'src/RockConfig.cpp' 'rockFarSelectionHmdConeHalfAngleDegrees = selection_query_policy::kDefaultFarSelectionHmdConeHalfAngleDegrees;' 'RockConfig::resetToDefaults must use the policy-owned far HMD cone angle default.'
Require-Ini-Key 'fFarSelectionHmdConeHalfAngleDegrees' '50.0'
Require-Text 'src/RockConfig.cpp' 'sanitizeFarSelectionHmdConeHalfAngleDegrees' 'Far HMD cone half-angle must be sanitized before runtime use.'
Require-Text 'src/RockConfig.cpp' 'ini\.GetDoubleValue\(SECTION, "fCloseSelectionBehindPalmToleranceGameUnits", rockCloseSelectionBehindPalmToleranceGameUnits\)' 'fCloseSelectionBehindPalmToleranceGameUnits must be read from the active/prod INI when present.'
Require-Ini-Key 'fCloseSelectionBehindPalmToleranceGameUnits' '2.0'
Reject-Text 'src/RockConfig.cpp' 'FrikFingerSkeleton' 'Removed FrikFingerSkeleton debug aliases must not remain in RockConfig reads.'
Reject-Text 'data/config/ROCK.ini' 'FrikFingerSkeleton' 'Packaged fallback ROCK.ini must not expose removed FrikFingerSkeleton debug aliases.'

Require-Text 'src/RockConfig.h' 'float rockPullOwnerGraceSeconds = 1\.0f;' 'RockConfig member initializer for pull owner grace must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPullOwnerGraceSeconds = 1\.0f;' 'RockConfig::resetToDefaults must keep pull owner alive after the velocity window.'
Require-Ini-Key 'fPullOwnerGraceSeconds' '1.0'
Require-Text 'src/RockConfig.h' 'float rockPullCatchRetryMaxTimeSeconds = 0\.65f;' 'RockConfig member initializer for pull catch retry window must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPullCatchRetryMaxTimeSeconds = 0\.65f;' 'RockConfig::resetToDefaults must match pull catch retry window default.'
Require-Ini-Key 'fPullCatchRetryMaxTimeSeconds' '0.65'
Require-Text 'src/RockConfig.h' 'bool rockPullCatchWideReacquireEnabled = true;' 'RockConfig member initializer for pull catch wide reacquire must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPullCatchWideReacquireEnabled = true;' 'RockConfig::resetToDefaults must enable pull catch wide reacquire when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bPullCatchWideReacquireEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockPullCatchWideReacquireRadiusGameUnits = 32\.0f;' 'RockConfig member initializer for pull catch wide radius must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPullCatchWideReacquireRadiusGameUnits = 32\.0f;' 'RockConfig::resetToDefaults must match pull catch wide radius default.'
Require-Ini-Key 'fPullCatchWideReacquireRadiusGameUnits' '32.0'
Require-Text 'src/RockConfig.h' 'float rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42\.0f;' 'RockConfig member initializer for pull catch max body distance must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPullCatchWideReacquireMaxBodyDistanceGameUnits = 42\.0f;' 'RockConfig::resetToDefaults must match pull catch max body distance default.'
Require-Ini-Key 'fPullCatchWideReacquireMaxBodyDistanceGameUnits' '42.0'

Require-Text 'src/RockConfig.h' 'int rockGrabConvergeStableFrames = 3;' 'RockConfig member initializer for convergence stable frames must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabConvergeStableFrames = 3;' 'RockConfig::resetToDefaults must match convergence stable-frame default.'
Require-Ini-Key 'iGrabConvergeStableFrames' '3'
Require-Text 'src/RockConfig.h' 'float rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40\.0f;' 'RockConfig member initializer for convergence separating-speed guard must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond = 40\.0f;' 'RockConfig::resetToDefaults must match convergence separating-speed guard default.'
Require-Ini-Key 'fGrabConvergeMaxSeparatingSpeedGameUnitsPerSecond' '40.0'
Require-Text 'src/RockConfig.h' 'float rockGrabAcquisitionVisualStartDistanceGameUnits = 28\.0f;' 'RockConfig member initializer for acquisition visual start must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabAcquisitionVisualStartDistanceGameUnits = 28\.0f;' 'RockConfig::resetToDefaults must match acquisition visual start default.'
Require-Ini-Key 'fGrabAcquisitionVisualStartDistanceGameUnits' '28.0'

Require-Text 'src/RockConfig.h' 'bool rockGrabPlayerSpaceTransformWarpEnabled = true;' 'RockConfig member initializer must enable held-object player-space transform warp for snap-turn/room-rotation compensation.'
Require-Text 'src/RockConfig.cpp' 'rockGrabPlayerSpaceTransformWarpEnabled = true;' 'RockConfig::resetToDefaults must enable held-object player-space transform warp.'
Require-Ini-Key 'bGrabPlayerSpaceTransformWarpEnabled' 'true'

Require-Text 'src/RockConfig.h' 'bool rockGrabNearbyDampingEnabled = true;' 'RockConfig member initializer must enable nearby damping after verified FO4VR hknp motion-property writes.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNearbyDampingEnabled = true;' 'RockConfig::resetToDefaults must enable nearby damping after runtime writes are verified.'
Require-Ini-Key 'bGrabNearbyDampingEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabNearbyLinearDamping = 3\.0f;' 'RockConfig member initializer for nearby linear damping must use hknp coefficient units.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNearbyLinearDamping = 3\.0f;' 'RockConfig::resetToDefaults must use hknp coefficient units for nearby linear damping.'
Require-Ini-Key 'fGrabNearbyLinearDamping' '3.0'
Require-Text 'src/RockConfig.h' 'float rockGrabNearbyAngularDamping = 5\.5f;' 'RockConfig member initializer for nearby angular damping must use hknp coefficient units.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNearbyAngularDamping = 5\.5f;' 'RockConfig::resetToDefaults must use hknp coefficient units for nearby angular damping.'
Require-Ini-Key 'fGrabNearbyAngularDamping' '5.5'

Require-Text 'src/RockConfig.h' 'float rockPulledGrabHandAdjustDistanceGameUnits = 10\.5f;' 'RockConfig member initializer for pulled grab hand adjustment must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockPulledGrabHandAdjustDistanceGameUnits = 10\.5f;' 'RockConfig::resetToDefaults must match pulled grab hand adjustment default.'
Require-Ini-Key 'fPulledGrabHandAdjustDistanceGameUnits' '10.5'

Require-Text 'src/RockConfig.h' 'bool rockGrabNodeRejectOppositeHandAnchor = true;' 'RockConfig member initializer for opposite-hand grab node rejection must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNodeRejectOppositeHandAnchor = true;' 'RockConfig::resetToDefaults must enable opposite-hand grab node rejection.'
Require-Ini-Key 'bGrabNodeRejectOppositeHandAnchor' 'true'
Require-Text 'src/RockConfig.h' 'std::string rockGrabNodeNameBlacklist = "ROCK:GrabR,ROCK:GrabL";' 'RockConfig member initializer for grab node blacklist must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabNodeNameBlacklist = std::string\(grab_node_name_policy::kDefaultGrabNodeNameBlacklist\);' 'RockConfig::resetToDefaults must use the policy-owned default grab node blacklist.'
Require-Ini-Key 'sGrabNodeNameBlacklist' 'ROCK:GrabR,ROCK:GrabL'

Require-Text 'src/RockConfig.cpp' 'rockGrabFingerPoseUpdateInterval = std::clamp\(rockGrabFingerPoseUpdateInterval, 1, 60\);' 'Grab finger pose update interval must be sanitized before runtime use.'
Require-Text 'src/RockConfig.cpp' 'rockGrabFingerMinValue = std::clamp\(rockGrabFingerMinValue, 0\.0f, 1\.0f\);' 'Grab finger minimum curl must be clamped before reaching FRIK pose publishing.'
Require-Text 'src/RockConfig.cpp' 'Invalid fGrabFingerMinValue' 'Grab finger minimum curl must reject non-finite INI values.'
Require-Text 'src/RockConfig.cpp' 'Invalid fSelectedCloseFingerAnimMaxHandSpeed' 'Selected-close hand speed threshold must reject non-finite or negative INI values.'
Require-Text 'src/RockConfig.cpp' 'rockSelectedCloseFingerAnimValue = std::clamp\(rockSelectedCloseFingerAnimValue, 0\.0f, 1\.0f\);' 'Selected-close finger value must be clamped before FRIK pose publishing.'
Require-Text 'src/physics-interaction/hand/HandGrab.cpp' 'configuredFallback[\s\S]*std::isfinite\(g_rockConfig\.rockSelectedCloseFingerAnimValue\)' 'Grab fallback pose must guard against stale or non-finite selected-close values even after config load.'

Require-Text 'src/RockConfig.h' 'bool rockSoftContactWorldEnabled = true;' 'RockConfig member initializer for world soft contact must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldEnabled = true;' 'RockConfig::resetToDefaults must enable world soft contact when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bSoftContactWorldEnabled' 'true'

Require-Text 'src/RockConfig.h' 'float rockSoftContactWorldMaxCorrectionGameUnits = 18\.0f;' 'RockConfig member initializer for world soft contact max correction must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldMaxCorrectionGameUnits = 18\.0f;' 'RockConfig::resetToDefaults must match world soft contact max correction default.'
Require-Ini-Key 'fSoftContactWorldMaxCorrectionGameUnits' '18.0'

Require-Text 'src/RockConfig.h' 'float rockSoftContactWeaponHandRadiusPaddingGameUnits = 0\.25f;' 'RockConfig member initializer for weapon-hand soft contact radius padding must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWeaponHandRadiusPaddingGameUnits = kDefaultSoftContactWeaponHandRadiusPaddingGameUnits;' 'RockConfig::resetToDefaults must use the weapon-hand soft contact radius padding default.'
Require-Ini-Key 'fSoftContactWeaponHandRadiusPaddingGameUnits' '0.25'
Require-Text 'src/RockConfig.h' 'float rockSoftContactWeaponHandMaxCorrectionGameUnits = 3\.0f;' 'RockConfig member initializer for weapon-hand soft contact max correction must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWeaponHandMaxCorrectionGameUnits = kDefaultSoftContactWeaponHandMaxCorrectionGameUnits;' 'RockConfig::resetToDefaults must use the weapon-hand soft contact max correction default.'
Require-Ini-Key 'fSoftContactWeaponHandMaxCorrectionGameUnits' '3.0'
Require-Text 'src/RockConfig.h' 'float rockSoftContactWeaponHandCorrectionScale = 0\.35f;' 'RockConfig member initializer for weapon-hand soft contact correction scale must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWeaponHandCorrectionScale = kDefaultSoftContactWeaponHandCorrectionScale;' 'RockConfig::resetToDefaults must use the weapon-hand soft contact correction scale default.'
Require-Ini-Key 'fSoftContactWeaponHandCorrectionScale' '0.35'
Require-Text 'src/RockConfig.h' 'float rockSoftContactWeaponHandHardStopPenetrationGameUnits = 4\.0f;' 'RockConfig member initializer for weapon-hand soft contact hard-stop depth must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWeaponHandHardStopPenetrationGameUnits = kDefaultSoftContactWeaponHandHardStopPenetrationGameUnits;' 'RockConfig::resetToDefaults must use the weapon-hand soft contact hard-stop depth default.'
Require-Ini-Key 'fSoftContactWeaponHandHardStopPenetrationGameUnits' '4.0'

Require-Text 'src/RockConfig.h' 'float rockSoftContactWorldContactPaddingGameUnits = 0\.35f;' 'RockConfig member initializer for world contact padding must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldContactPaddingGameUnits = 0\.35f;' 'RockConfig::resetToDefaults must match world contact padding default.'
Require-Ini-Key 'fSoftContactWorldContactPaddingGameUnits' '0.35'

Require-Text 'src/RockConfig.h' 'float rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits = 0\.025f;' 'RockConfig member initializer for world post-release reentry approach distance must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits = 0\.025f;' 'RockConfig::resetToDefaults must match world post-release reentry approach distance default.'
Require-Ini-Key 'fSoftContactWorldPostReleaseReentryMinApproachDistanceGameUnits' '0.025'

Require-Text 'src/RockConfig.h' 'float rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits = 10\.0f;' 'RockConfig member initializer for world cached-plane tangent drift must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldCachedPlaneMaxTangentDriftGameUnits = 10\.0f;' 'RockConfig::resetToDefaults must match world cached-plane tangent drift default.'
Require-Ini-Key 'fSoftContactWorldCachedPlaneMaxTangentDriftGameUnits' '10.0'

Reject-Text 'src/RockConfig.h' 'rockSoftContactRecoverySmoothingSpeed' 'Soft contact must not expose recovery smoothing; no-contact frames must clear external authority.'
Reject-Text 'src/RockConfig.h' 'rockSoftContactSmoothingSpeed' 'Soft contact must not expose visual smoothing that allows current-frame penetration.'
Reject-Text 'src/RockConfig.h' 'rockSoftContactWorldSmoothingSpeed' 'World soft contact must not expose visual smoothing that allows current-frame penetration.'
Reject-Text 'data/config/ROCK.ini' 'fSoftContactRecoverySmoothingSpeed' 'Packaged fallback ROCK.ini must not expose a sticky recovery smoothing knob.'
Reject-Text 'data/config/ROCK.ini' 'fSoftContactSmoothingSpeed' 'Packaged fallback ROCK.ini must not expose a visual smoothing knob that weakens the hard-stop projection.'
Reject-Text 'data/config/ROCK.ini' 'fSoftContactWorldSmoothingSpeed' 'Packaged fallback ROCK.ini must not expose a world visual smoothing knob that weakens the hard-stop projection.'
Reject-Text 'data/config/ROCK.ini' 'fSoftContactWorldReleaseHysteresisGameUnits' 'Packaged fallback ROCK.ini must not expose world release hysteresis that lets surfaces become visual authority.'

Require-Text 'src/RockConfig.h' 'bool rockSoftContactWorldHapticsEnabled = true;' 'RockConfig member initializer for world soft contact haptics must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldHapticsEnabled = true;' 'RockConfig::resetToDefaults must enable world soft contact haptics when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bSoftContactWorldHapticsEnabled' 'true'

Require-Text 'src/RockConfig.h' 'bool rockNativeCharacterControllerObjectContactFilterEnabled = true;' 'RockConfig member initializer for native player character-controller object filtering must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockNativeCharacterControllerObjectContactFilterEnabled = true;' 'RockConfig::resetToDefaults must enable native player character-controller object filtering when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bNativeCharacterControllerObjectContactFilterEnabled' 'true'

Require-Text 'src/RockConfig.h' 'float rockSoftContactWorldHapticCooldownSeconds = 0\.12f;' 'RockConfig member initializer for world haptic cooldown must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSoftContactWorldHapticCooldownSeconds = 0\.12f;' 'RockConfig::resetToDefaults must match world haptic cooldown default.'
Require-Ini-Key 'fSoftContactWorldHapticCooldownSeconds' '0.12'

Require-Text 'src/RockConfig.h' 'bool rockGrabHapticsEnabled = true;' 'RockConfig member initializer for grab haptics must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabHapticsEnabled = true;' 'RockConfig::resetToDefaults must enable grab haptics when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bGrabHapticsEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockGrabHapticDurationSeconds = 0\.055f;' 'RockConfig member initializer for grab haptic duration must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockGrabHapticDurationSeconds = 0\.055f;' 'RockConfig::resetToDefaults must match grab haptic duration default.'
Require-Ini-Key 'fGrabHapticDurationSeconds' '0.055'
Require-Text 'src/RockConfig.h' 'float rockSelectionLockHapticIntensity = 0\.15f;' 'RockConfig member initializer for selection lock haptic intensity must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSelectionLockHapticIntensity = 0\.15f;' 'RockConfig::resetToDefaults must match selection lock haptic default.'
Require-Ini-Key 'fSelectionLockHapticIntensity' '0.15'
Require-Text 'src/RockConfig.h' 'float rockSelectionLockReleaseHapticDurationSeconds = 0\.02f;' 'RockConfig member initializer for selection lock release haptic duration must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockSelectionLockReleaseHapticDurationSeconds = 0\.02f;' 'RockConfig::resetToDefaults must match selection lock release duration default.'
Require-Ini-Key 'fSelectionLockReleaseHapticDurationSeconds' '0.02'
Require-Text 'src/RockConfig.h' 'bool rockHeldImpactHapticsEnabled = true;' 'RockConfig member initializer for held impact haptics must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockHeldImpactHapticsEnabled = true;' 'RockConfig::resetToDefaults must enable held impact haptics when packaged fallback ROCK.ini enables it.'
Require-Ini-Key 'bHeldImpactHapticsEnabled' 'true'
Require-Text 'src/RockConfig.h' 'float rockHeldImpactHapticCooldownSeconds = 0\.12f;' 'RockConfig member initializer for held impact haptic cooldown must match packaged fallback ROCK.ini value.'
Require-Text 'src/RockConfig.cpp' 'rockHeldImpactHapticCooldownSeconds = 0\.12f;' 'RockConfig::resetToDefaults must match held impact haptic cooldown default.'
Require-Ini-Key 'fHeldImpactHapticCooldownSeconds' '0.12'

if ($failures.Count -gt 0) {
    Write-Host 'Config default parity source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Config default parity source boundary passed.'

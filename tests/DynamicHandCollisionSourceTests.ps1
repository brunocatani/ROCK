param(
    [string]$Root = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$failures = [System.Collections.Generic.List[string]]::new()

function Require-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -notmatch $Pattern) {
        $failures.Add($Message)
    }
}

function Require-OrderedText {
    param(
        [string]$Path,
        [string[]]$Patterns,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        $failures.Add($Message)
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    $offset = 0
    foreach ($pattern in $Patterns) {
        $remaining = $text.Substring($offset)
        $match = [regex]::Match($remaining, $pattern, [System.Text.RegularExpressions.RegexOptions]::Singleline)
        if (-not $match.Success) {
            $failures.Add($Message)
            return
        }
        $offset += $match.Index + $match.Length
    }
}

function Reject-Text {
    param(
        [string]$Path,
        [string]$Pattern,
        [string]$Message
    )

    $fullPath = Join-Path $Root $Path
    if (-not (Test-Path -LiteralPath $fullPath)) {
        return
    }

    $text = Get-Content -Raw -LiteralPath $fullPath
    if ($text -match $Pattern) {
        $failures.Add($Message)
    }
}

# Canonical free-hand world collision uses dynamic velocity-driven proxies.

# The shared wrapper must not run FO4VR's keyframed initializer over dynamic
# twins. That routine zeros inverse mass, allowing tracked bodies to move while
# preventing the solver from displacing hands at static-world contact.
Reject-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    'deriveMotionCinfo\s*\(' `
    'Dynamic hand twins must retain the motion-cinfo constructor inverse mass.'
Require-Text 'src/physics-interaction/native/BethesdaPhysicsBody.cpp' `
    '0x1417A3A90 is initializeAsKeyFramed[\s\S]{0,500}motionCinfoCtor\(motionCinfo\);' `
    'The shared body wrapper must preserve the dynamic-safe native constructor profile.'

# Live-world teardown must use deferred retirement (2026-07-08 UAF lesson).

# The twins must mirror the production collider conventions: hand frames/shapes
# come from HandBoneColliderSet and forearm frames/shapes from BodyBoneColliderSet,
# never from independently re-derived geometry.
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'makeDescriptorFrame\(',
    'collectForearmTwinMergeSource\(forearmTwinMergeSources, descriptor, frame\);',
    'queueBodyTarget\(instance\.body, frame\.transform',
    'publishMergedForearmTwinTargets\('
) 'BodyBoneColliderSet must merge the exact three frames queued to its keyframed forearm/wrist bodies.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'BoneColliderRole::ForearmSegment',
    'kForearmUpperMergeSource',
    'kForearmLowerMergeSource',
    'BoneColliderRole::HandSegment',
    'kWristMergeSource'
) 'The merged dynamic forearm must include ForeArm1->2, ForeArm2->3, and ForeArm3->Hand sources.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'forearmStartBone = isLeft \? "LArm_ForeArm1" : "RArm_ForeArm1"',
    'handBone = isLeft \? "LArm_Hand" : "RArm_Hand"',
    'mergedFrame\.length = sources\[0\]\.length \+ sources\[1\]\.length \+ sources\[2\]\.length'
) 'The single dynamic forearm must span ForeArm1->Hand and retain all three tuned source lengths.'
Require-Text 'src/physics-interaction/body/BodyBoneColliderSet.cpp' `
    'buildDynamicForearmTwinShape[\s\S]*buildShapeForFrame\(frame\)' `
    'Forearm twins must share the production body-collider hull construction.'
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    '_canonicalForearmTwinDimensions = forearmTwinTargets;',
    'applyCanonicalForearmDimensions\(',
    '_dynamicForearmTwinTargets = forearmTwinTargets;'
) 'Forearm twins must retain generation-canonical dimensions while publishing live rigid targets.'

# One animated dynamic compound owns the 17 semantic children. Child IDs are
# decoded from key-2 shape keys; no callback may guess a child from the shared
# body ID or from key-3's unrelated +0x10 payload.
# Collider frames author axes as columns; scene NiTransforms author them as
# rows. Composing a physics delta with the scene hand without converting both
# sampled transforms transposes (= inverts) the rotation delta: the exact
# reversed-hand-rotation failure of the reverted first compound attempt.
Require-OrderedText 'src/RockConfig.cpp' @(
    'fHandCollisionDynamicCompoundMass',
    'fHandCollisionDynamicInverseInertiaMultiplier'
) 'The hand compound mass and inverse-inertia controls must load through the ROCK INI path.'

# The flexion is physical: compound finger children always chase the LIVE
# published role frames so the colliders curl and slide with the rendered
# pose. No frozen capture-time collider intent may exist anywhere — that is
# exactly what welded the rigid compound into walls while the rendered
# fingers curled away from it.
# Probes are remeasured around the CURRENT pose and the solve re-baselines on
# the current open values every frame, so sustained blocked contact keeps
# stepping the curl toward the anatomical stop instead of deflecting around a
# stale captured baseline.

# Render-follow pipeline: apply the compound body's coherent SE(3) readback and
# reserve translation smoothing for explicit teleport recovery only.
Require-OrderedText 'src/physics-interaction/body/BodyBoneColliderSet.cpp' @(
    'shoulderBone = isLeft \? "LArm_UpperArm" : "RArm_UpperArm"',
    'forearmHandTargetResponseScale\(',
    'shoulder\.translate',
    'input\.end\.translate',
    'mergedFrame\.transform\.translate'
) 'Merged forearm response must derive its IK leverage from the live shoulder, hand, and proxy center.'

# Physics-owned telemetry must cross to the main frame through atomics and
# expose requested/commanded/live positions without changing provider API V1.

# Haptics are generated from real post-solve contact entry, consumed under
# stronger ownership, and delivered through the shared main-thread mixer.
# Manifold contact fires before any penetration deviation exists, so a
# grazing/resting touch reports ~zero approach speed. Latching the haptic
# entry there consumes the episode and the later real press stays silent.

# Debug visualization must consume the same telemetry snapshot intended for a
# later API adapter, rather than re-reading live bodies through a second path.
Require-OrderedText 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' @(
    'if \(drawDynamicHandColliders\)',
    'getTelemetrySnapshot\(telemetry\)',
    'requestedGapGameUnits',
    'approachSpeedGameUnitsPerSecond'
) 'Dynamic hand overlay must visualize the canonical collision telemetry snapshot.'

# Profiling distinguishes frame work, pre-collide drive, and post-solve reads.
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'DynamicHandCollisionFrame' `
    'Dynamic hand main-frame work must have a dedicated profiler scope.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'DynamicHandCollisionPhysicsDrive' `
    'Dynamic hand physics drive must have a dedicated profiler scope.'
Require-Text 'src/physics-interaction/performance/PerformanceProfiler.h' `
    'DynamicHandCollisionPostSolve' `
    'Dynamic hand post-solve sampling must have a dedicated profiler scope.'

# The developer reference carries tuning but cannot disable the canonical runtime or its haptics.
foreach ($configPath in @('data/config/ROCK.dev.ini')) {
    Reject-Text $configPath `
        'bHandCollisionDynamicDrive|bHandCollisionDynamicHapticsEnabled' `
        "$configPath must not expose mandatory dynamic collision or haptics."
    Require-Text $configPath `
        'fHandCollisionDynamicHapticMinApproachSpeedGameUnitsPerSecond' `
        "$configPath must document the dynamic hand haptic speed units."
    Reject-Text $configPath `
        'SoftContact|ContactTargetIdentity|fHandCollisionDynamicConstraint' `
        "$configPath must not retain superseded soft-contact, target-identity, or constraint-drive keys."
    Require-Text $configPath `
        'fHandCollisionDynamicContactPressMaxVelocityHavok' `
        "$configPath must retain the established dynamic-collider contact press cap."
}
Require-Text 'src/RockConfig.cpp' `
    'fHandCollisionDynamicContactPressMaxVelocityHavok' `
    'Dynamic collider press-cap tuning must load through the ROCK configuration boundary.'

# Contact identity comes from the verified key-2 manifold shape keys, while
# post-solve readback measures each animated child through the one compound
# body's coherent rigid transform.

# The divergence dwell and the drive-side recovery teleport must run on the
# REQUESTED-target gap: the commanded-target delta (bodyDeltaGameUnits)
# saturates at maxLinearVelocity * driveDt and can never cross a divergence
# threshold, which silently makes the recovery teleport dead code.
Require-OrderedText 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.cpp' @(
    'result\.requestedTargetGamePosition = requestedTarget\.translate;',
    'requestedGapGameUnits > mode\.divergenceTeleportGameUnits',
    'target = requestedTarget;'
) 'Dynamic drive divergence teleport must measure against and place at the requested target.'

# Preserve the proven dynamic-collider movement contract: target the exact
# published palm collider frame, then hard-keyframe the dynamic compound body
# itself so the solver clips one coherent body. Compound grouping is the only
# behavior borrowed from the weapon path.

# The twins get their own visualization flag, independent of the keyframed
# collider debug draws.
Require-Text 'src/physics-interaction/core/PhysicsInteractionDebugOverlay.cpp' `
    'if \(drawDynamicHandColliders\)' `
    'Dynamic hand twins must draw behind their own bDebugDrawDynamicHandColliders flag.'

# The drive keeps chasing the wand while another system owns the hand pose:
# target queueing must happen BEFORE the ownership gate.

# Dynamic collision is the only free-hand world-collision implementation.
Reject-Text 'CMakeLists.txt' `
    'ROCKSoftContact|SoftContactWorld' `
    'The build must not register legacy soft-contact tests or targets.'
foreach ($legacyPath in @(
        'src/physics-interaction/contact/SoftContactMath.h',
        'src/physics-interaction/contact/SoftContactRuntime.cpp',
        'src/physics-interaction/contact/SoftContactRuntime.h',
        'src/physics-interaction/contact/SoftContactWorldPolicy.h',
        'src/physics-interaction/contact/NativeContactEvidence.h',
        'src/physics-interaction/contact/ContactTargetIdentity.cpp',
        'src/physics-interaction/contact/ContactTargetIdentity.h',
        'tests/SoftContactWorldOnlySourceTests.ps1',
        'tests/SoftContactWorldPolicyTests.cpp')) {
    if (Test-Path -LiteralPath (Join-Path $Root $legacyPath)) {
        $failures.Add("Legacy soft-contact file must be deleted: $legacyPath")
    }
}

# Existing keyframed callers must keep their behavior: the drive-mode parameter
# stays defaulted to keyframe placement.
Require-Text 'src/physics-interaction/native/GeneratedKeyframedBodyDrive.h' `
    'const GeneratedBodyDriveMode& mode = \{\}' `
    'driveGeneratedKeyframedBody must default to keyframe placement for existing callers.'

# Each hand keeps a stable row. Both retain world/car collision while the
# experimental graph gates only the opposite hand and weapon edges.
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'ROCK_LAYER_DYNAMIC_HAND_PROXY = 48',
    'ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER = 49',
    'ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER = 50',
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY = 51',
    'ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY = 52',
    'buildRockDynamicHandProxyExpectedMask\(',
    'isWorldSurfaceLayer\(layer\)',
    'withLayer\(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_CLUTTER\)',
    'withLayer\(mask, ROCK_LAYER_DYNAMIC_WORLD_CAR_LARGE_CLUTTER\)',
    'interactionsEnabled',
    'ROCK_LAYER_DYNAMIC_WEAPON_PROXY'
) 'Dynamic hand proxy rows must preserve world/car collision and explicitly gate cross-owner interaction edges.'
Require-Text 'src/physics-interaction/object/DynamicWorldCarCollision.cpp' `
    'isExplodableCarReference[\s\S]*dynamicWorldCarLayerForNativeLayer[\s\S]*setFilterInfo' `
    'Only verified ExplodableCar references may be tagged onto dynamic-world car layers.'
Require-Text 'src/physics-interaction/object/DynamicWorldCarCollision.cpp' `
    'currentFilterInfo != tagged\.taggedFilterInfo[\s\S]*resolveBodyToRef[\s\S]*tagged\.originalFilterInfo' `
    'Car filter restoration must verify both current filter ownership and native body identity.'
Require-Text 'src/physics-interaction/object/DynamicWorldCarCollision.cpp' `
    'DynamicWorldCarCollisionRuntime::restoreSlot[\s\S]*restoreTaggedBodiesForReference\(bhkWorld, hknpWorld, ref, seedBodyId, reason\)' `
    'Car teardown must restore the native layer even if another owner changed non-layer filter bits.'
Require-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'nativeCharacterControllerObjectSuppressionLayerMask[\s\S]*FO4_LAYER_CLUTTER[\s\S]*FO4_LAYER_CLUTTER_LARGE[\s\S]*originalMask & ~nativeCharacterControllerObjectSuppressionLayerMask' `
    'Ordinary clutter and large clutter must be rejected at the character-controller matrix.'
Reject-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'nativeCharacterControllerBodyFilteredLayerMask|isNativeCharacterControllerBodyFilteredLayer' `
    'Native clutter must not be globally re-enabled for a late per-body character-controller filter.'
Require-Text 'src/physics-interaction/collision/CollisionLayerPolicy.h' `
    'buildRockDynamicWorldCarExpectedMask[\s\S]*withoutLayer\(mask, ROCK_LAYER_HAND\)[\s\S]*withoutLayer\(mask, ROCK_LAYER_WEAPON\)[\s\S]*withoutLayer\(mask, ROCK_LAYER_BODY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_LEFT_HAND_PROXY\)[\s\S]*withLayer\(mask, ROCK_LAYER_DYNAMIC_WEAPON_PROXY\)' `
    'Car-only rows must reject generated gameplay colliders while admitting both hands and the weapon solver proxy.'
Require-Text 'src/physics-interaction/core/PhysicsHooks.cpp' `
    'isDynamicWorldCarLayer\(layer\)[\s\S]*targetIsCar = targetIdentity\.isCar' `
    'Character-controller contact identity must be evaluated only on dedicated car rows.'
Require-OrderedText 'src/physics-interaction/collision/CollisionLayerPolicy.h' @(
    'inline void applyRockGeneratedLayerPolicies\(',
    'applyRockDynamicHandProxyLayerPolicies\(',
    'dynamicHandInteractionsEnabled'
) 'Both dynamic hand rows must be applied with the other generated layer rows.'

# The proxy drive flush must run beside the other generated collider flushes.

# Fixed-surface grabs use a separate bounded contact channel. Palm and
# fingertips are eligible; the forearm and ordinary loose-object semantic set
# remain excluded.
# The opt-in API remains authoritative, while the shipped INI enables a
# built-in world-surface fallback for direct play and testing.
foreach ($configPath in @('data/config/ROCK.dev.ini')) {
    Require-Text $configPath `
        'bGlobalSurfaceGrabEnabled\s*=\s*true' `
        "$configPath must globally enable fixed-surface grabs."
}
foreach ($configPath in @('data/config/ROCK.dev.ini')) {
    Require-Text $configPath `
        'bHandCollisionSurfaceFingerResponseEnabled\s*=\s*true[\s\S]*fHandCollisionSurfaceFingerProbeDeltaOpenUnits[\s\S]*fHandCollisionSurfaceFingerResponseGain[\s\S]*fHandCollisionSurfaceFingerMaximumDeflectionOpenUnits[\s\S]*fHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits[\s\S]*fHandCollisionSurfaceFingerSmoothingSpeed[\s\S]*fHandCollisionSurfaceFingerReleaseDelaySeconds' `
        "$configPath must ship the globally enabled, bounded experimental surface finger response."
}
Require-Text 'src/RockConfig.h' `
    'rockHandCollisionSurfaceFingerResponseEnabled\s*=\s*true' `
    'Older INIs must inherit the enabled surface finger response default.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(\s*SECTION,\s*"bHandCollisionSurfaceFingerResponseEnabled"' `
    'The surface finger feature switch must load through the ROCK INI path.'
Require-OrderedText 'src/RockConfig.cpp' @(
    'fHandCollisionSurfaceFingerProbeDeltaOpenUnits',
    'fHandCollisionSurfaceFingerResponseGain',
    'fHandCollisionSurfaceFingerMaximumDeflectionOpenUnits',
    'fHandCollisionSurfaceFingerMinimumHelpfulTravelGameUnits',
    'fHandCollisionSurfaceFingerSmoothingSpeed',
    'fHandCollisionSurfaceFingerReleaseDelaySeconds'
) 'Every bounded surface finger control must load through the ROCK INI path.'
Require-Text 'src/RockConfig.h' `
    'rockGlobalSurfaceGrabEnabled\s*=\s*true' `
    'The compiled global surface-grab default must remain enabled when an older INI lacks the key.'
Require-Text 'src/RockConfig.cpp' `
    'GetBoolValue\(SECTION,\s*"bGlobalSurfaceGrabEnabled",\s*rockGlobalSurfaceGrabEnabled\)' `
    'The global surface-grab switch must load through the normal ROCK INI path.'
Require-Text 'src/physics-interaction/grab/GlobalSurfaceGrabPolicy.h' `
    'enabled\s*&&[\s\S]*!providerMatched[\s\S]*wildcardPass[\s\S]*dynamicSurfaceContact[\s\S]*isDynamicHandProxySurfaceLayer' `
    'The global path must be a dynamic-surface wildcard fallback that never overrides a provider match.'
Require-Text 'src/physics-interaction/grab/GlobalSurfaceGrabPolicy.h' `
    'canFollowUnclassifiedMotion[\s\S]{0,220}globalSurfaceFallback\s*&&\s*fixedAnchor' `
    'Only a built-in global FixedAnchor may follow a body whose motion-property handle is not classified.'
# A successful fixed-surface latch owns the feedback for that hand. Its
# one-shot confirmation is intentionally stronger and longer than the dynamic
# touch pulse, and both values remain user-tunable.
foreach ($configPath in @('data/config/ROCK.dev.ini')) {
    Require-Text $configPath `
        'fSurfaceGrabHapticDurationSeconds\s*=\s*0\.075[\s\S]*fSurfaceGrabHapticIntensity\s*=\s*0\.85' `
        "$configPath must retain distinct surface-latch haptic tuning."
    Reject-Text $configPath `
        'bSurfaceGrabHapticsEnabled' `
        "$configPath must not expose mandatory surface-grab haptics."
}
Require-Text 'src/RockConfig.h' `
    'rockSurfaceGrabHapticsEnabled\s*=\s*true[\s\S]{0,180}rockSurfaceGrabHapticDurationSeconds\s*=\s*0\.075f[\s\S]{0,180}rockSurfaceGrabHapticIntensity\s*=\s*0\.85f' `
    'The compiled surface-latch pulse must remain stronger and longer than the touch maximum.'
Reject-Text 'src/RockConfig.cpp' `
    '"bSurfaceGrabHapticsEnabled"' `
    'Mandatory surface-latch haptics must not load from INI.'

if ($failures.Count -gt 0) {
    Write-Host 'Dynamic hand collision source boundary failed:'
    foreach ($failure in $failures) {
        Write-Host " - $failure"
    }
    exit 1
}

Write-Host 'Dynamic hand collision source boundary passed.'
